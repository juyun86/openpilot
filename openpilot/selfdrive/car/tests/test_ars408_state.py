import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import custom, log
from openpilot.cereal.services import QueueSize, SERVICE_LIST
from openpilot.selfdrive.car.ars408_state import (
  ARS408StatePublisher, CORE_TARGET_LIMIT, DIAGNOSTIC_PUBLISH_NS, DIAGNOSTIC_TARGET_LIMIT,
  INVALID_HEARTBEAT_NS, SOURCE_FRESH_NS,
)


SOURCE_NS = 10_000_000_000
_ARS408_OUTER_AUDIT_ROOT = "audit/ars408_shadow_review_20260829/"


def _consumer_scan_path_in_scope(repo_label: str, relative_path: str, *, tracked: bool) -> bool:
  """Keep all tracked paths; omit only the known outer untracked audit package."""
  normalized = relative_path.replace("\\", "/").removeprefix("./")
  if tracked or repo_label != "outer":
    return True
  audit_directory = _ARS408_OUTER_AUDIT_ROOT.removesuffix("/")
  return normalized != audit_directory and not normalized.startswith(_ARS408_OUTER_AUDIT_ROOT)


def target(logical_id: int, *, lifecycle: str = "confirmedFresh", semantic_group: str = "vehicle",
           possible_vru: bool = False, extended_fresh: bool = True, reason_bits: int = 0,
           d_rel: float = 20.0):
  return SimpleNamespace(
    logical_id=logical_id, raw_id=logical_id & 0xFF, lifecycle=lifecycle, semantic_group=semantic_group,
    fresh_measured=True, possible_vru=possible_vru, extended_fresh=extended_fresh,
    hit_count=3, miss_count=0, age_cycles=3, last_measured_age_ms=0.0,
    raw_d_rel=d_rel, raw_y_rel=0.3, raw_v_rel=-1.0, raw_yv_rel=0.0,
    d_rel=d_rel, y_rel=0.3, v_rel=-1.0, yv_rel=0.0,
    a_rel_long=-0.1, a_rel_lat=0.0, orientation=0.0, length=4.0, width=1.8, rcs=8.0,
    existence_probability_code=4, measurement_state=2, dynamic_property=0,
    raw_class=1, stable_class=1, quality_score=0.8, clutter_score=0.0,
    rms_codes=(5,) * 7, rms_upper_bounds=(0.1,) * 7, reason_bits=reason_bits,
  )


def snapshot(sequence: int = 1, *, source_ns: int = SOURCE_NS, cycle_status: str = "exact",
             general_complete: bool = True, quality_complete: bool = True,
             extended_complete: bool = True, motion_input_valid: bool = True,
             stable=(), uncertain=(), clutter=(), expired=(), counter_gap: int = 0,
             cadence_valid: bool = True, cadence_verified: bool = True, cadence_inferred: bool = False,
             expected_counter_step: int = 1, step_source: str = "deviceCapture"):
  return SimpleNamespace(
    sequence=sequence, source_mono_time=source_ns,
    last_component_mono_time=source_ns + 1_000_000, assembly_close_mono_time=source_ns + 2_000_000,
    measurement_counter=sequence,
    cycle_status=cycle_status, motion_input_valid=motion_input_valid,
    general_complete=general_complete, quality_complete=quality_complete,
    extended_complete=extended_complete, raw_count=len(stable) + len(uncertain) + len(clutter),
    accepted_count=len(stable), counter_gap=counter_gap, dropped_cycle_count=int(counter_gap > 0),
    counter_anomaly_count=int(counter_gap > 0),
    cadence_valid=cadence_valid, cadence_verified=cadence_verified, cadence_inferred=cadence_inferred,
    expected_counter_step=expected_counter_step, step_source=step_source,
    stable_targets=tuple(stable), uncertain_targets=tuple(uncertain),
    clutter_suspects=tuple(clutter), expired_events=tuple(expired),
  )


def perception_state(snap, *, parser_valid: bool = True, radar_state_ready: bool = True,
                     radar_state_fresh: bool = True, lateral_transform_valid: bool = True,
                     motion_input_valid: bool | None = None,
                     producer_fault: bool = False,
                     interference_active: bool = False, config_observed: bool = True,
                     config_valid: bool = True,
                     transient_failure_bits: int = 0,
                     shadow_now_source_ns: int = SOURCE_NS,
                     can_error: bool = False, radar_fault: bool = False,
                     radar_unavailable_temporary: bool = False, wrong_config: bool = False,
                     interference_count: int = 0, producer_epoch: int = 1):
  errors = SimpleNamespace(
    can_error=can_error, radar_fault=radar_fault,
    radar_unavailable_temporary=radar_unavailable_temporary, wrong_config=wrong_config,
  )
  diagnostics = SimpleNamespace(
    errors=errors, trusted_errors=errors, radar_state_ready=radar_state_ready,
    interference_count=interference_count, trusted_interference_count=interference_count,
  )
  current_motion_valid = bool(getattr(snap, "motion_input_valid", False)) if motion_input_valid is None else motion_input_valid
  return SimpleNamespace(
    producer_epoch=producer_epoch,
    snapshot=snap, diagnostics=diagnostics, parser_valid=parser_valid,
    motion_input_valid=current_motion_valid,
    radar_state_ready=radar_state_ready, radar_state_fresh=radar_state_fresh,
    shadow_now_source_ns=shadow_now_source_ns,
    lateral_transform_valid=lateral_transform_valid,
    interference_active=interference_active, config_observed=config_observed, config_valid=config_valid,
    transient_failure_bits=transient_failure_bits,
    producer_fault=producer_fault,
    producer_fault_count=int(producer_fault),
  )


def build(snap, *, now_ns: int = SOURCE_NS, publisher: ARS408StatePublisher | None = None, **state_kwargs):
  publisher = publisher or ARS408StatePublisher()
  state_kwargs.setdefault("shadow_now_source_ns", now_ns)
  result = publisher.build(perception_state(snap, **state_kwargs), now_ns)
  assert result is not None
  return (*result, publisher)


def build_diagnostics(snap, *, now_ns: int = SOURCE_NS,
                      publisher: ARS408StatePublisher | None = None, **state_kwargs):
  publisher = publisher or ARS408StatePublisher()
  state_kwargs.setdefault("shadow_now_source_ns", now_ns)
  result = publisher.build_diagnostics(perception_state(snap, **state_kwargs), now_ns)
  assert result is not None
  return (*result, publisher)


def test_diagnostic_prepare_send_commit_retains_full_window_after_failed_send() -> None:
  publisher = ARS408StatePublisher()
  first_target = target(7, lifecycle="tentative", reason_bits=1 << 4)
  first_state = perception_state(snapshot(sequence=1, uncertain=(first_target,)))
  prepared = publisher.prepare_diagnostics(first_state, SOURCE_NS)
  assert prepared.result is not None
  publisher.commit_diagnostics(prepared, sent=False)

  second_target = target(8, lifecycle="clutterSuspect", reason_bits=1 << 6)
  second_time = SOURCE_NS + DIAGNOSTIC_PUBLISH_NS
  second_state = perception_state(
    snapshot(sequence=2, source_ns=second_time, clutter=(second_target,)),
    shadow_now_source_ns=second_time, producer_epoch=2, producer_fault=True,
    transient_failure_bits=1 << 6,
  )
  retried = publisher.prepare_diagnostics(second_state, second_time)
  assert retried.result is not None
  diagnostic, _valid = retried.result
  assert diagnostic.fullCount == 2 and diagnostic.targetCount == 2
  assert diagnostic.producerEpoch == 2 and diagnostic.producerFailed
  assert {item.kind for item in diagnostic.targets} == {"uncertain", "clutter"}
  assert {(item.observedProducerEpoch, item.target.logicalId) for item in diagnostic.targets} == {(1, 7), (2, 8)}
  assert diagnostic.windowReasonBits & (1 << 4)
  assert diagnostic.windowReasonBits & (1 << 6)
  publisher.commit_diagnostics(retried, sent=True)

  third_time = second_time + DIAGNOSTIC_PUBLISH_NS
  cleared = publisher.prepare_diagnostics(
    perception_state(
      snapshot(sequence=3, source_ns=third_time), shadow_now_source_ns=third_time, producer_epoch=2,
    ), third_time,
  )
  assert cleared.result is not None
  assert cleared.result[0].fullCount == 0 and cleared.result[0].targetCount == 0


def test_schema_defaults_are_fail_safe_and_wire_slots_are_preserved() -> None:
  state = custom.ARS408StateSP.new_message()
  assert not state.hasData
  assert state.health == "unavailable"
  assert state.cycleStatus == "invalid"
  assert state.forwardPresenceState == "unknown"
  assert not state.cadenceValid and not state.cadenceVerified and not state.cadenceInferred
  assert state.stepSource == "unknown"
  assert state.producerEpoch == 0
  assert len(state.diagnosticTargets) == 0 and state.diagnosticCount == 0
  assert not state.diagnosticsTruncated and not state.diagnosticsUpdated
  assert state.diagnosticSequence == 0
  compact = state.init("targets", 1)[0]
  assert compact.lifecycle == "unavailable"
  assert compact.semanticGroup == "unknownObstacle"

  repo = Path(__file__).resolve().parents[4]
  custom_schema = (repo / "openpilot/cereal/custom.capnp").read_text(encoding="utf-8")
  event_schema = (repo / "openpilot/cereal/log.capnp").read_text(encoding="utf-8")
  # dev-sp-egpu reserves slots 138/139 for these additions; existing
  # trafficRadarState/customReserved11 wire slots 136/137 remain untouched.
  assert "struct ARS408StateSP @0x9ccdc8676701b412" in custom_schema
  assert "struct ARS408DiagnosticsSP @0xcd96dafb67a082d0" in custom_schema
  assert "ars408StateSP @138 :Custom.ARS408StateSP" in event_schema
  assert "ars408DiagnosticsSP @139 :Custom.ARS408DiagnosticsSP" in event_schema

  diagnostic = custom.ARS408DiagnosticsSP.new_message()
  diagnostic.targetCount = 0
  event = log.Event.new_message()
  event.valid = False
  event.ars408DiagnosticsSP = diagnostic
  with log.Event.from_bytes(event.to_bytes()) as restored_event:
    assert restored_event.which() == "ars408DiagnosticsSP"
    assert not restored_event.valid and restored_event.ars408DiagnosticsSP.targetCount == 0


def test_epoch_zero_cannot_publish_valid_or_targets() -> None:
  snap = snapshot(stable=(target(7),))
  publisher = ARS408StatePublisher()
  core = publisher.build(perception_state(snap, producer_epoch=0), SOURCE_NS)
  assert core is not None
  state, valid = core
  assert not valid and state.producerEpoch == 0 and state.health == "unavailable"
  assert state.forwardPresenceState == "unknown" and state.targetCount == 0 and len(state.targets) == 0
  diagnostics = publisher.build_diagnostics(perception_state(snap, producer_epoch=0), SOURCE_NS)
  assert diagnostics is not None
  diagnostic_state, diagnostic_valid = diagnostics
  assert not diagnostic_valid and diagnostic_state.producerEpoch == 0


def test_diagnostics_window_units_and_provenance_are_documented_as_non_permissive() -> None:
  repo = Path(__file__).resolve().parents[4]
  schema = (repo / "openpilot/cereal/custom.capnp").read_text(encoding="utf-8")
  readme = (repo / "opendbc_repo/opendbc/sunnypilot/car/tesla/ars408/README.md").read_text(encoding="utf-8")
  normalized_schema = " ".join(schema.replace("#", " ").split())
  normalized_readme = " ".join(readme.replace("`", "").split())

  shared_contract = (
    "bounded incremental audit collection accumulated since the previous physical ars408DiagnosticsSP event",
    "deduplicated by (observedProducerEpoch, logicalId): possible-VRU/class-conflict records have highest safety priority",
    "same (observedProducerEpoch, logicalId) and priority, the newer observation replaces the older one",
    "rawCount and acceptedCount describe only the latest snapshot header; they are not window totals",
    "A targetCount of zero is not a current target-absence claim and must never authorize free space, lane use, or motion",
    "observedProducerEpoch, observedSequence, and observedMonoTime identify the selected source observation",
  )
  for required_text in shared_contract:
    assert required_text in normalized_schema
    assert required_text in normalized_readme

  assert "consumer must independently enforce service liveness, local TTL, monotonic" in normalized_schema
  assert "teardown barrier is best-effort" in normalized_schema

  assert "dRel/yRel use metres (m); vRel uses m/s; lastMeasuredAgeMs uses ms" in normalized_schema
  assert "aRelLong/aRelLat use m/s²; orientation uses degrees; rcs uses dBm² (DBC spelling: dBm2)" in normalized_schema
  assert "dRel, yRel, rawDRel, rawYRel, length, and width: metres (m)" in normalized_readme
  assert "vRel, yvRel, rawVRel, and rawYvRel: metres per second (m/s)" in normalized_readme
  assert "aRelLong and aRelLat: metres per second squared (m/s²)" in normalized_readme
  assert "orientation: degrees" in normalized_readme
  assert "rcs: dBm² (the DBC spelling is dBm2)" in normalized_readme
  assert "sensor-to-openpilot sign transform, which remains unverified on the installed vehicle" in normalized_schema
  assert "sensor-to-openpilot sign transform, which remains unverified on the installed vehicle" in normalized_readme


def test_healthy_confirmed_vru_roundtrip_uses_compact_channel() -> None:
  snap = snapshot(stable=(target(5, semantic_group="pedestrian", possible_vru=True),))
  state, valid, _ = build(snap)
  assert valid and state.health == "healthy" and state.sourceFresh
  assert state.cadenceValid and state.cadenceVerified and not state.cadenceInferred
  assert state.stepSource == "deviceCapture"
  assert (state.rawCount, state.acceptedCount, state.stableCount, state.targetCount) == (1, 1, 1, 1)
  assert state.forwardPresenceState == "present"
  assert state.measurementMonoTime == SOURCE_NS
  assert state.lastComponentMonoTime == SOURCE_NS + 1_000_000
  assert state.assemblyCloseMonoTime == SOURCE_NS + 2_000_000
  assert state.targets[0].semanticGroup == "pedestrian"
  assert state.targets[0].possibleVru
  # Original wire slots remain for compatibility, but the high-rate service
  # never carries the full diagnostic payload.
  assert len(state.diagnosticTargets) == 0 and state.diagnosticCount == 0
  assert not state.diagnosticsTruncated and not state.diagnosticsUpdated
  assert state.diagnosticSequence == 0
  with custom.ARS408StateSP.from_bytes(state.to_bytes()) as restored:
    assert restored.targets[0].logicalId == 5
    assert restored.targets[0].semanticGroup == "pedestrian"
    assert restored.cadenceVerified and restored.stepSource == "deviceCapture"


def test_child_cadence_provenance_roundtrip_only_accepts_device_capture() -> None:
  from opendbc.sunnypilot.car.tesla.ars408.stabilizer import (
    CadenceStepSource, CycleStatus as ChildCycleStatus, PerceptionSnapshot,
  )

  def child_snapshot(step_source: CadenceStepSource, *, cadence_verified: bool) -> PerceptionSnapshot:
    return PerceptionSnapshot(
      sequence=7, source_mono_time=SOURCE_NS,
      last_component_mono_time=SOURCE_NS + 1_000_000,
      assembly_close_mono_time=SOURCE_NS + 2_000_000,
      measurement_counter=7, cycle_status=ChildCycleStatus.EXACT,
      motion_input_valid=True, general_complete=True, quality_complete=True, extended_complete=True,
      raw_count=0, accepted_count=0, counter_gap=0, dropped_cycle_count=0, counter_anomaly_count=0,
      cadence_valid=True, cadence_verified=cadence_verified, cadence_inferred=False,
      expected_counter_step=1, step_source=step_source,
      stable_targets=(), uncertain_targets=(), clutter_suspects=(), expired_events=(),
    )

  external, external_valid, _ = build(
    child_snapshot(CadenceStepSource.EXTERNAL, cadence_verified=False),
  )
  assert not external_valid and external.cadenceValid and not external.cadenceVerified
  assert external.stepSource == "external" and external.health == "degraded"

  captured, captured_valid, _ = build(
    child_snapshot(CadenceStepSource.DEVICE_CAPTURE, cadence_verified=True),
  )
  assert captured_valid and captured.cadenceValid and captured.cadenceVerified
  assert captured.stepSource == "deviceCapture" and captured.health == "healthy"
  with custom.ARS408StateSP.from_bytes(captured.to_bytes()) as restored:
    assert restored.cadenceValid and restored.cadenceVerified
    assert restored.stepSource == "deviceCapture"


def test_standard_diagnostic_ready_flag_cannot_substitute_for_trusted_shadow_state() -> None:
  snap = snapshot()
  state_input = perception_state(snap, radar_state_ready=True)
  del state_input.radar_state_ready
  built = ARS408StatePublisher().build(state_input, SOURCE_NS)
  assert built is not None
  state, valid = built
  assert not valid and not state.radarStateReady and state.health == "unavailable"


def test_standard_arrival_faults_cannot_substitute_for_trusted_shadow_faults() -> None:
  state_input = perception_state(snapshot())
  del state_input.diagnostics.trusted_errors
  built = ARS408StatePublisher().build(state_input, SOURCE_NS)
  assert built is not None
  state, valid = built
  assert not valid and state.health == "degraded"
  assert state.errors.canError and state.errors.radarFault
  assert state.errors.radarUnavailableTemporary and state.errors.wrongConfig


def test_source_freshness_uses_anchored_source_domain_not_host_epoch() -> None:
  route_source_ns = 9_000_000_000_000_000_000
  snap = snapshot(source_ns=route_source_ns)
  publisher = ARS408StatePublisher()
  built = publisher.build(
    perception_state(snap, shadow_now_source_ns=route_source_ns + 100_000_000),
    now_ns=123_456_789,
  )
  assert built is not None and built[1]
  assert built[0].sourceFresh and built[0].sourceAgeMs == 100

  no_anchor = ARS408StatePublisher().build(
    perception_state(snap, shadow_now_source_ns=0), now_ns=123_456_789,
  )
  assert no_anchor is not None and not no_anchor[1]
  assert not no_anchor[0].sourceFresh and no_anchor[0].health == "stale"


def test_counter_anomaly_preserves_verified_identity_but_invalidates_current_cadence() -> None:
  snap = snapshot(
    cycle_status="invalid", counter_gap=0xFFFF,
    cadence_valid=False, cadence_verified=True, step_source="deviceCapture",
  )
  state, valid, _ = build(snap)
  assert not valid and state.health == "degraded"
  assert not state.cadenceValid and state.cadenceVerified
  assert state.counterGap == 0xFFFF and state.counterAnomalyCount == 1
  assert state.targetCount == 0 and state.forwardPresenceState == "unknown"
  with custom.ARS408StateSP.from_bytes(state.to_bytes()) as restored:
    assert not restored.cadenceValid and restored.cadenceVerified
    assert restored.stepSource == "deviceCapture"


@pytest.mark.parametrize("snapshot_kwargs,state_kwargs,now_ns", [
  ({"cycle_status": "partial"}, {}, SOURCE_NS),
  ({"cycle_status": "invalid"}, {}, SOURCE_NS),
  ({"general_complete": False}, {}, SOURCE_NS),
  ({"quality_complete": False}, {}, SOURCE_NS),
  ({"counter_gap": 2}, {}, SOURCE_NS),
  ({"cadence_valid": False}, {}, SOURCE_NS),
  ({"cadence_verified": False}, {}, SOURCE_NS),
  ({"cadence_verified": False, "cadence_inferred": True, "step_source": "inferred"}, {}, SOURCE_NS),
  ({}, {"parser_valid": False}, SOURCE_NS),
  ({}, {"radar_state_ready": False}, SOURCE_NS),
  ({}, {"radar_state_fresh": False}, SOURCE_NS),
  ({}, {"lateral_transform_valid": False}, SOURCE_NS),
  ({"motion_input_valid": False}, {}, SOURCE_NS),
  ({}, {"config_observed": False}, SOURCE_NS),
  ({}, {"config_valid": False}, SOURCE_NS),
  ({}, {"producer_fault": True}, SOURCE_NS),
  ({}, {"can_error": True}, SOURCE_NS),
  ({}, {"radar_fault": True}, SOURCE_NS),
  ({}, {"radar_unavailable_temporary": True}, SOURCE_NS),
  ({}, {"wrong_config": True}, SOURCE_NS),
  ({"source_ns": 0}, {}, SOURCE_NS),
  ({"source_ns": SOURCE_NS + 1}, {}, SOURCE_NS),
  ({}, {}, SOURCE_NS + SOURCE_FRESH_NS + 1),
])
def test_invalid_health_or_cycle_never_exposes_compact_targets(snapshot_kwargs, state_kwargs, now_ns) -> None:
  snap = snapshot(stable=(target(1),), **snapshot_kwargs)
  state, valid, _ = build(snap, now_ns=now_ns, **state_kwargs)
  assert not valid
  assert len(state.targets) == 0
  assert state.targetCount == 0
  assert state.forwardPresenceState == "unknown"


def test_missing_extended_is_invalid_and_never_exposes_targets() -> None:
  snap = snapshot(
    stable=(target(3, semantic_group="pedestrian", possible_vru=True, extended_fresh=False),),
    extended_complete=False,
  )
  state, valid, _ = build(snap)
  assert not valid
  assert not state.extendedComplete and state.motionInputValid
  assert len(state.targets) == 0 and state.forwardPresenceState == "unknown"


def test_unverified_motion_or_lateral_transform_never_exposes_compact_targets() -> None:
  state, valid, _ = build(snapshot(stable=(target(3),), motion_input_valid=False))
  assert not valid and state.health == "degraded"
  assert len(state.targets) == 0 and state.forwardPresenceState == "unknown"

  transform_state, transform_valid, _ = build(
    snapshot(stable=(target(4),)), lateral_transform_valid=False,
  )
  assert not transform_valid and len(transform_state.targets) == 0
  diagnostic, diagnostic_valid, _ = build_diagnostics(
    snapshot(stable=(target(4),)), lateral_transform_valid=False,
  )
  assert not diagnostic_valid and diagnostic.targets[0].kind == "stableAudit"
  assert diagnostic.coordinateUnverified and diagnostic.targets[0].coordinateUnverified


@pytest.mark.parametrize("interference_count", (1, 9, 10))
def test_any_active_interference_is_explicit_and_fail_safe(interference_count: int) -> None:
  state, valid, _ = build(
    snapshot(stable=(target(1),)), interference_count=interference_count, interference_active=True,
  )
  assert not valid and state.health == "degraded"
  assert state.interferenceActive and state.interferenceCount == interference_count
  assert len(state.targets) == 0 and state.forwardPresenceState == "unknown"


def test_transient_failure_stays_fail_safe_after_final_state_recovers() -> None:
  transient_config_failure = 1 << 2
  snap = snapshot(stable=(target(1),))
  state, valid, _ = build(snap, transient_failure_bits=transient_config_failure)
  assert not valid and state.health == "degraded"
  assert state.transientFailureBits == transient_config_failure
  assert state.configValid and state.targetCount == 0
  assert state.forwardPresenceState == "unknown"

  diagnostic, diagnostic_valid, _ = build_diagnostics(
    snap, transient_failure_bits=transient_config_failure,
  )
  assert not diagnostic_valid and diagnostic.health == "degraded"
  assert diagnostic.transientFailureBits == transient_config_failure
  assert diagnostic.configFailed and diagnostic.worstHealth == "degraded"


def test_only_forward_targets_claim_presence() -> None:
  rear = snapshot(stable=(target(1, d_rel=-2.0),))
  state, valid, _ = build(rear, interference_count=9)
  assert valid and len(state.targets) == 1
  assert not state.interferenceActive and state.interferenceCount == 9
  assert state.forwardPresenceState == "unknown"


def test_stale_transition_heartbeat_and_recovery_are_rate_limited() -> None:
  publisher = ARS408StatePublisher()
  fresh_state = perception_state(snapshot(
    stable=(target(1),), uncertain=(target(2, lifecycle="tentative"),),
  ))
  first = publisher.build(fresh_state, SOURCE_NS)
  assert first is not None and first[1]
  fresh_state.shadow_now_source_ns = SOURCE_NS + SOURCE_FRESH_NS
  assert publisher.build(fresh_state, SOURCE_NS + SOURCE_FRESH_NS) is None

  stale_time = SOURCE_NS + SOURCE_FRESH_NS + 1
  fresh_state.shadow_now_source_ns = stale_time
  stale = publisher.build(fresh_state, stale_time)
  assert stale is not None and not stale[1]
  assert stale[0].health == "stale" and len(stale[0].targets) == 0
  assert publisher.build(fresh_state, stale_time + INVALID_HEARTBEAT_NS - 1) is None
  fresh_state.shadow_now_source_ns = stale_time + INVALID_HEARTBEAT_NS
  heartbeat = publisher.build(fresh_state, stale_time + INVALID_HEARTBEAT_NS)
  assert heartbeat is not None and not heartbeat[1]

  recovered_snapshot = snapshot(sequence=2, source_ns=stale_time + INVALID_HEARTBEAT_NS, stable=(target(1),))
  recovered = publisher.build(
    perception_state(recovered_snapshot, shadow_now_source_ns=recovered_snapshot.source_mono_time),
    recovered_snapshot.source_mono_time,
  )
  assert recovered is not None and recovered[1]


def test_diagnostics_are_deduplicated_bounded_and_report_full_counts() -> None:
  duplicate_ids = tuple(target(i, lifecycle="tentative") for i in range(40))
  clutter = tuple(target(i, lifecycle="clutterSuspect", reason_bits=1 << 9) for i in range(10))
  diagnostic, valid, _ = build_diagnostics(snapshot(uncertain=duplicate_ids, clutter=clutter))
  assert valid
  assert diagnostic.uncertainFullCount == 30 and diagnostic.clutterFullCount == 10
  assert diagnostic.fullCount == 40
  assert len(diagnostic.targets) == DIAGNOSTIC_TARGET_LIMIT
  assert diagnostic.truncated
  ids = [item.target.logicalId for item in diagnostic.targets]
  assert len(ids) == len(set(ids))


def test_diagnostic_accumulator_retains_one_shot_vru_and_expired_events_until_flush() -> None:
  publisher = ARS408StatePublisher()
  first, _, _ = build_diagnostics(snapshot(sequence=1), now_ns=SOURCE_NS, publisher=publisher)
  assert len(first.targets) == 0
  vru = target(
    90, lifecycle="tentative", semantic_group="pedestrian", possible_vru=True, reason_bits=1 << 5,
  )
  expired = target(91, lifecycle="expired")
  assert publisher.build_diagnostics(
    perception_state(snapshot(sequence=2, source_ns=SOURCE_NS + 100_000_000,
                              uncertain=(vru,), expired=(expired,)),
                     shadow_now_source_ns=SOURCE_NS + 100_000_000),
    SOURCE_NS + 100_000_000,
  ) is None
  assert publisher.build_diagnostics(
    perception_state(snapshot(sequence=3, source_ns=SOURCE_NS + 200_000_000),
                     shadow_now_source_ns=SOURCE_NS + 200_000_000),
    SOURCE_NS + 200_000_000,
  ) is None
  flushed = publisher.build_diagnostics(
    perception_state(snapshot(sequence=4, source_ns=SOURCE_NS + DIAGNOSTIC_PUBLISH_NS),
                     shadow_now_source_ns=SOURCE_NS + DIAGNOSTIC_PUBLISH_NS),
    SOURCE_NS + DIAGNOSTIC_PUBLISH_NS,
  )
  assert flushed is not None
  kinds = {item.target.logicalId: item.kind for item in flushed[0].targets}
  assert kinds == {90: "uncertain", 91: "expired"}
  vru_record = next(item for item in flushed[0].targets if item.target.logicalId == 90)
  assert vru_record.target.possibleVru
  assert vru_record.observedSequence == 2
  assert vru_record.observedMonoTime == SOURCE_NS + 100_000_000
  assert flushed[0].windowStartMonoTime == SOURCE_NS + 100_000_000
  assert flushed[0].windowEndMonoTime == SOURCE_NS + DIAGNOSTIC_PUBLISH_NS
  assert flushed[0].windowReasonBits & (1 << 5)


def test_bad_diagnostic_sequence_is_transactional_not_retried_and_next_sequence_recovers() -> None:
  publisher = ARS408StatePublisher()
  build_diagnostics(snapshot(sequence=1), now_ns=SOURCE_NS, publisher=publisher)

  good_prefix = target(70, lifecycle="tentative", reason_bits=1 << 4)
  malformed = target(71, lifecycle="tentative")
  malformed.reason_bits = object()
  bad_time = SOURCE_NS + 100_000_000
  bad_state = perception_state(
    snapshot(sequence=2, source_ns=bad_time, uncertain=(good_prefix, malformed)),
    shadow_now_source_ns=bad_time,
  )
  with pytest.raises((TypeError, ValueError)):
    publisher.build_diagnostics(bad_state, bad_time)

  # The attempted sequence is suppressed at 100 Hz and did not partially add
  # the valid prefix that preceded the malformed record.
  assert publisher.build_diagnostics(bad_state, SOURCE_NS + 200_000_000) is None

  recovery_time = SOURCE_NS + 300_000_000
  recovered_target = target(72, lifecycle="tentative", reason_bits=1 << 7)
  recovery_state = perception_state(
    snapshot(sequence=3, source_ns=recovery_time, uncertain=(recovered_target,)),
    shadow_now_source_ns=recovery_time,
  )
  assert publisher.build_diagnostics(recovery_state, recovery_time) is None

  flush_time = SOURCE_NS + DIAGNOSTIC_PUBLISH_NS
  recovery_state.shadow_now_source_ns = flush_time
  flushed = publisher.build_diagnostics(recovery_state, flush_time)
  assert flushed is not None
  assert [item.target.logicalId for item in flushed[0].targets] == [72]
  assert flushed[0].targets[0].observedSequence == 3
  assert flushed[0].targets[0].observedMonoTime == recovery_time
  assert not (flushed[0].windowReasonBits & (1 << 4))
  assert flushed[0].windowReasonBits & (1 << 7)

  frozen_time = flush_time + DIAGNOSTIC_PUBLISH_NS
  recovery_state.shadow_now_source_ns = frozen_time
  frozen = publisher.build_diagnostics(recovery_state, frozen_time)
  assert frozen is not None and frozen[0].targetCount == 0


def test_diagnostic_service_is_nominal_2hz_and_logger_decimation_one_keeps_every_event() -> None:
  publisher = ARS408StatePublisher()
  emitted = []
  for sequence in range(1, 15):
    now_ns = SOURCE_NS + (sequence - 1) * 75_000_000
    result = publisher.build_diagnostics(
      perception_state(snapshot(sequence=sequence, source_ns=now_ns,
                                uncertain=(target(20, lifecycle="tentative"),)),
                       shadow_now_source_ns=now_ns),
      now_ns,
    )
    if result is not None:
      emitted.append(result[0])
  assert [event.sequence for event in emitted] == [1, 8]
  assert all(event.targets[0].target.logicalId == 20 for event in emitted)
  qlog_samples = [event for index, event in enumerate(emitted) if index % 1 == 0]
  assert qlog_samples == emitted


def test_transition_burst_is_immediate_bounded_and_reorders_next_nominal_emit() -> None:
  publisher = ARS408StatePublisher()
  build_diagnostics(snapshot(sequence=1), now_ns=SOURCE_NS, publisher=publisher)
  fault_time = SOURCE_NS + 1_000_000
  fault = snapshot(
    sequence=2, source_ns=fault_time, cycle_status="invalid", counter_gap=4, cadence_valid=False,
  )
  early = publisher.build_diagnostics(
    perception_state(fault, shadow_now_source_ns=fault_time), fault_time,
  )
  assert early is not None and not early[1]
  assert early[0].cycleStatus == "invalid" and early[0].counterGap == 4
  assert not early[0].cadenceValid and early[0].cadenceVerified
  assert early[0].counterFailed and early[0].cadenceFailed
  assert early[0].transitionCount == 1
  second_transition = publisher.build_diagnostics(
    perception_state(snapshot(sequence=3, source_ns=SOURCE_NS + 100_000_000),
                     shadow_now_source_ns=SOURCE_NS + 100_000_000),
    SOURCE_NS + 100_000_000,
  )
  assert second_transition is None
  assert publisher.build_diagnostics(
    perception_state(snapshot(sequence=4, source_ns=fault_time + DIAGNOSTIC_PUBLISH_NS - 1),
                     shadow_now_source_ns=fault_time + DIAGNOSTIC_PUBLISH_NS - 1),
    fault_time + DIAGNOSTIC_PUBLISH_NS - 1,
  ) is None
  reordered = publisher.build_diagnostics(
    perception_state(snapshot(sequence=5, source_ns=fault_time + DIAGNOSTIC_PUBLISH_NS),
                     shadow_now_source_ns=fault_time + DIAGNOSTIC_PUBLISH_NS),
    fault_time + DIAGNOSTIC_PUBLISH_NS,
  )
  assert reordered is not None


def test_severe_fault_after_early_emit_remains_in_next_window_evidence() -> None:
  publisher = ARS408StatePublisher()
  build_diagnostics(snapshot(sequence=1), now_ns=SOURCE_NS, publisher=publisher)

  light_time = SOURCE_NS + 1_000_000
  light = publisher.build_diagnostics(
    perception_state(snapshot(sequence=2, source_ns=light_time), config_valid=False,
                     shadow_now_source_ns=light_time), light_time,
  )
  assert light is not None and light[0].configFailed

  severe_time = SOURCE_NS + 100_000_000
  assert publisher.build_diagnostics(
    perception_state(snapshot(sequence=3, source_ns=severe_time), producer_fault=True,
                     shadow_now_source_ns=severe_time), severe_time,
  ) is None
  recovery_time = SOURCE_NS + 200_000_000
  assert publisher.build_diagnostics(
    perception_state(snapshot(sequence=4, source_ns=recovery_time),
                     shadow_now_source_ns=recovery_time), recovery_time,
  ) is None

  flush_time = light_time + DIAGNOSTIC_PUBLISH_NS
  flushed = publisher.build_diagnostics(
    perception_state(snapshot(sequence=5, source_ns=flush_time), shadow_now_source_ns=flush_time),
    flush_time,
  )
  assert flushed is not None
  state = flushed[0]
  assert state.worstHealth == "producerFault"
  assert state.producerFailed
  assert state.windowReasonBits & (1 << 24)
  assert state.transitionCount >= 2


def test_transition_burst_ceiling_is_four_events_per_second() -> None:
  publisher = ARS408StatePublisher()
  event_times = []
  cases = (
    (0, True),
    (1_000_000, False),
    (100_000_000, True),
    (501_000_000, True),
    (502_000_000, False),
    (600_000_000, True),
    (1_002_000_000, True),
  )
  for sequence, (offset, config_valid) in enumerate(cases, 1):
    event_time = SOURCE_NS + offset
    result = publisher.build_diagnostics(
      perception_state(snapshot(sequence=sequence, source_ns=event_time), config_valid=config_valid,
                       shadow_now_source_ns=event_time), event_time,
    )
    if result is not None:
      event_times.append(event_time)
  assert event_times == [
    SOURCE_NS, SOURCE_NS + 1_000_000, SOURCE_NS + 501_000_000,
    SOURCE_NS + 502_000_000, SOURCE_NS + 1_002_000_000,
  ]
  for window_start in event_times:
    assert sum(window_start <= event_time < window_start + 1_000_000_000
               for event_time in event_times) <= 4


def test_stable_audit_is_retained_even_with_verified_coordinates() -> None:
  snap = snapshot(stable=(target(7),))
  compact, compact_valid, publisher = build(snap, lateral_transform_valid=True)
  diagnostic, diagnostic_valid, _ = build_diagnostics(
    snap, publisher=publisher, lateral_transform_valid=True,
  )
  assert compact_valid and diagnostic_valid
  assert compact.targets[0].logicalId == 7
  assert diagnostic.targets[0].kind == "stableAudit"
  assert diagnostic.targets[0].target.logicalId == 7
  assert not diagnostic.coordinateUnverified and not diagnostic.targets[0].coordinateUnverified


def test_core_limit_is_deterministic_and_truncation_never_claims_presence() -> None:
  stable = tuple(target(i, possible_vru=(i == 104), d_rel=float(i + 1)) for i in range(105))
  state, valid, _ = build(snapshot(stable=stable))
  assert valid and len(state.targets) == CORE_TARGET_LIMIT
  assert state.targetsTruncated
  assert state.forwardPresenceState == "unknown"
  assert state.targets[0].logicalId == 104


@pytest.mark.parametrize("stable_count,diagnostic_count", [(0, 0), (10, 4), (100, 24)])
def test_zero_typical_and_maximum_payload_fit_the_service_queue(stable_count: int, diagnostic_count: int) -> None:
  stable = tuple(target(i) for i in range(stable_count))
  uncertain = tuple(target(1000 + i, lifecycle="tentative") for i in range(diagnostic_count))
  snap = snapshot(stable=stable, uncertain=uncertain)
  state, _, publisher = build(snap)
  diagnostic, _, _ = build_diagnostics(snap, publisher=publisher)
  assert len(state.targets) <= CORE_TARGET_LIMIT
  assert len(diagnostic.targets) <= DIAGNOSTIC_TARGET_LIMIT
  event = log.Event.new_message()
  event.valid = True
  event.ars408StateSP = state
  assert len(event.to_bytes()) < int(QueueSize.SMALL)
  diagnostic_event = log.Event.new_message()
  diagnostic_event.valid = True
  diagnostic_event.ars408DiagnosticsSP = diagnostic
  assert len(diagnostic_event.to_bytes()) < int(QueueSize.SMALL)


def test_services_split_unlogged_14hz_state_from_logged_transition_ceiling_diagnostics() -> None:
  state_service = SERVICE_LIST["ars408StateSP"]
  assert not state_service.should_log and state_service.frequency == 14.0
  assert state_service.decimation is None and state_service.queue_size == QueueSize.SMALL
  diagnostic_service = SERVICE_LIST["ars408DiagnosticsSP"]
  assert diagnostic_service.should_log and diagnostic_service.frequency == 4.0
  assert diagnostic_service.decimation == 1 and diagnostic_service.queue_size == QueueSize.SMALL


def test_bad_snapshot_is_not_retried_at_100hz_and_next_sequence_recovers() -> None:
  publisher = ARS408StatePublisher()
  bad = target(1)
  bad.d_rel = object()
  bad_state = perception_state(snapshot(sequence=1, stable=(bad,)))
  with pytest.raises((TypeError, ValueError)):
    publisher.build(bad_state, SOURCE_NS)
  assert publisher.build(bad_state, SOURCE_NS) is None

  good_state = perception_state(snapshot(sequence=2, stable=(target(1),)))
  recovered = publisher.build(good_state, SOURCE_NS)
  assert recovered is not None and recovered[1]


def test_card_has_no_shadow_tracker_publisher_or_service_registration() -> None:
  repo = Path(__file__).resolve().parents[4]
  card_source = (repo / "openpilot/selfdrive/car/card.py").read_text(encoding="utf-8")
  tree = ast.parse(card_source)
  car_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Car")
  init = next(node for node in car_class.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
  assert all(not (isinstance(node, ast.FunctionDef) and node.name.startswith("_publish_ars408"))
             for node in car_class.body)
  assert "ARS408StatePublisher" not in card_source
  assert "perception_state" not in card_source

  pubmaster_call = next(
    node for node in ast.walk(init) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and
    node.func.attr == "PubMaster"
  )
  registered_services = {
    node.value for node in ast.walk(pubmaster_call.args[0])
    if isinstance(node, ast.Constant) and isinstance(node.value, str)
  }
  assert {"ars408StateSP", "ars408DiagnosticsSP"}.isdisjoint(registered_services)


def test_consumer_scan_path_filter_only_omits_the_known_outer_untracked_audit_root() -> None:
  ordinary_paths = ("foo/build/x.py", "foo/cache/x.cc", "foo/audit/x.h")
  for relative_path in ordinary_paths:
    assert _consumer_scan_path_in_scope("outer", relative_path, tracked=True)
    assert _consumer_scan_path_in_scope("outer", relative_path, tracked=False)
    assert _consumer_scan_path_in_scope("nested", relative_path, tracked=True)
    assert _consumer_scan_path_in_scope("nested", relative_path, tracked=False)

  audit_file = f"{_ARS408_OUTER_AUDIT_ROOT}superproject-radar.patch"
  assert _consumer_scan_path_in_scope("outer", audit_file, tracked=True)
  assert not _consumer_scan_path_in_scope("outer", audit_file, tracked=False)
  assert _consumer_scan_path_in_scope("nested", audit_file, tracked=False)
  assert _consumer_scan_path_in_scope("outer", "audit/another_review/output.txt", tracked=False)
  assert _consumer_scan_path_in_scope(
    "outer", "audit/ars408_shadow_review_20260829-other/output.txt", tracked=False,
  )


def test_no_control_or_planner_subscribes_and_no_permission_semantics_exist() -> None:
  import subprocess

  repo = Path(__file__).resolve().parents[4]
  child_repo = repo / "opendbc_repo"
  service_tokens = ("ars408StateSP", "ars408DiagnosticsSP")
  expected_occurrences = {
    ("outer", "openpilot/cereal/custom.capnp"),
    ("outer", "openpilot/cereal/log.capnp"),
    ("outer", "openpilot/cereal/services.py"),
    ("outer", "openpilot/selfdrive/car/ars408_state.py"),
    ("outer", "openpilot/selfdrive/car/ars408_shadowd.py"),
    ("outer", "openpilot/selfdrive/car/tests/test_ars408_state.py"),
    ("outer", "openpilot/selfdrive/car/tests/test_ars408_shadowd.py"),
    ("nested", "opendbc/sunnypilot/car/tesla/ars408/README.md"),
  }

  def git(repo_root: Path, *args: str, allow_no_matches: bool = False) -> list[str]:
    result = subprocess.run(
      ["git", "-C", str(repo_root), *args], capture_output=True, text=True, encoding="utf-8", check=False,
    )
    expected_codes = (0, 1) if allow_no_matches else (0,)
    assert result.returncode in expected_codes, result.stderr
    assert not result.stderr.strip(), result.stderr
    return [line.removeprefix("./").replace("\\", "/") for line in result.stdout.splitlines() if line]

  occurrences: set[tuple[str, str]] = set()
  scanned_repositories: set[str] = set()
  for repo_label, scan_root in (("outer", repo), ("nested", child_repo)):
    tracked = git(
      scan_root, "grep", "-Il", "-e", service_tokens[0], "-e", service_tokens[1], "--", ".",
      allow_no_matches=True,
    )
    untracked = git(scan_root, "ls-files", "--others", "--exclude-standard")
    scanned_repositories.add(repo_label)
    # Every tracked text match is security-relevant and must be represented in
    # the exact allowlist. There are deliberately no path or suffix filters.
    occurrences.update((repo_label, path) for path in tracked)

    # git-grep cannot inspect untracked files. Scan those as bytes, use Git's
    # NUL heuristic to ignore binary content, and deliberately apply no suffix
    # or filename exemptions.
    for relative_path in untracked:
      if not _consumer_scan_path_in_scope(repo_label, relative_path, tracked=False):
        continue
      contents = (scan_root / relative_path).read_bytes()
      if b"\0" in contents[:8_000]:
        continue
      if any(token.encode() in contents for token in service_tokens):
        occurrences.add((repo_label, relative_path))

  assert scanned_repositories == {"outer", "nested"}
  assert occurrences == expected_occurrences

  prohibited_roots = (
    repo / "openpilot/selfdrive/controls", repo / "openpilot/selfdrive/modeld",
    repo / "openpilot/selfdrive/selfdrived", repo / "openpilot/sunnypilot/selfdrive",
  )
  lateral_velocity_consumers = []
  for root in prohibited_roots:
    if root.exists():
      for path in root.rglob("*.py"):
        source = path.read_text(encoding="utf-8", errors="ignore")
        if "deprecated.yvRel" in source:
          lateral_velocity_consumers.append(path)
  assert lateral_velocity_consumers == []

  schema = (repo / "openpilot/cereal/custom.capnp").read_text(encoding="utf-8")
  adapter = (repo / "openpilot/selfdrive/car/ars408_state.py").read_text(encoding="utf-8")
  for prohibited in ("mayProceed", "clear", "occupancyState", "occupied"):
    assert prohibited not in schema
  for prohibited in ("mayProceed", "occupancyState", "occupied", '"clear"', "'clear'"):
    assert prohibited not in adapter

  compact_schema = schema.split("struct CompactTarget", 1)[1].split("struct DiagnosticTarget", 1)[0]
  assert all(field not in compact_schema for field in ("yvRel", "aRelLat", "orientation", "qualityScore"))

  tree = ast.parse(adapter)
  assert all(not (isinstance(node, ast.ImportFrom) and node.module and node.module.endswith("ars408.stabilizer"))
             for node in tree.body)
