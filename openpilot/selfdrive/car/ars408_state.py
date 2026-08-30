from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from openpilot.cereal import custom

if TYPE_CHECKING:
  from opendbc.sunnypilot.car.tesla.ars408.stabilizer import PerceptionSnapshot, PerceptionTarget


SCHEMA_VERSION = 1
SOURCE_FRESH_NS = 250_000_000
INVALID_HEARTBEAT_NS = 1_000_000_000
CORE_TARGET_LIMIT = 64
DIAGNOSTIC_TARGET_LIMIT = 24
DIAGNOSTIC_PUBLISH_NS = 500_000_000
DIAGNOSTIC_ACCUMULATOR_LIMIT = 768

_UINT16_MAX = (1 << 16) - 1
_UINT32_MAX = (1 << 32) - 1
_CLASS_CONFLICT_BIT = 1 << 6
_WINDOW_SOURCE_FAILED = 1 << 16
_WINDOW_RADAR_FAILED = 1 << 17
_WINDOW_PARSER_FAILED = 1 << 18
_WINDOW_MOTION_FAILED = 1 << 19
_WINDOW_CONFIG_FAILED = 1 << 20
_WINDOW_INTERFERENCE_FAILED = 1 << 21
_WINDOW_CADENCE_FAILED = 1 << 22
_WINDOW_COUNTER_FAILED = 1 << 23
_WINDOW_PRODUCER_FAILED = 1 << 24
_TRANSIENT_SOURCE_FAILED = 1 << 0
_TRANSIENT_RADAR_FAILED = 1 << 1
_TRANSIENT_CONFIG_FAILED = 1 << 2
_TRANSIENT_INTERFERENCE_FAILED = 1 << 3
_TRANSIENT_MOTION_FAILED = 1 << 4
_TRANSIENT_PARSER_FAILED = 1 << 5
_TRANSIENT_PRODUCER_FAILED = 1 << 6
_HEALTH_SEVERITY = {"healthy": 0, "degraded": 1, "stale": 2, "unavailable": 3, "producerFault": 4}

_CYCLE_STATUSES = frozenset(("exact", "partial", "duplicate", "invalid"))
_LIFECYCLES = frozenset(("tentative", "confirmedFresh", "coasting", "clutterSuspect", "expired"))
_SEMANTIC_GROUPS = frozenset(("vehicle", "pedestrian", "rider", "unknownObstacle"))
_CADENCE_STEP_SOURCES = frozenset(("unknown", "external", "inferred", "deviceCapture"))
_VERIFIED_CADENCE_STEP_SOURCES = frozenset(("deviceCapture",))
_VERIFIED_COUNTER_STEPS = frozenset((1, 2))


@dataclass(frozen=True, slots=True)
class PreparedDiagnostics:
  result: tuple[Any, bool] | None
  commit_state: dict[str, Any]
  retained_state: dict[str, Any]


def _value(value: Any, default: str) -> str:
  enum_value = getattr(value, "value", value)
  return enum_value if isinstance(enum_value, str) else default


def _bounded(value: int, maximum: int) -> int:
  return min(max(int(value), 0), maximum)


def _logical_id(target: PerceptionTarget) -> int:
  return int(target.logical_id)


def _semantic_group(target: PerceptionTarget) -> str:
  # The ARS408 class is carried in the Extended component. A retained class is
  # diagnostic history, not a current semantic observation.
  if not bool(target.extended_fresh):
    return "unknownObstacle"
  semantic_group = _value(target.semantic_group, "unknownObstacle")
  return semantic_group if semantic_group in _SEMANTIC_GROUPS else "unknownObstacle"


def _write_compact_target(dst, target: PerceptionTarget) -> None:
  lifecycle = _value(target.lifecycle, "unavailable")
  dst.logicalId = _bounded(target.logical_id, (1 << 64) - 1)
  dst.rawId = _bounded(target.raw_id, _UINT16_MAX)
  dst.lifecycle = lifecycle if lifecycle in _LIFECYCLES else "unavailable"
  dst.semanticGroup = _semantic_group(target)
  dst.freshMeasured = bool(target.fresh_measured)
  dst.possibleVru = bool(target.possible_vru)
  dst.dRel = float(target.d_rel)
  dst.yRel = float(target.y_rel)
  dst.vRel = float(target.v_rel)
  dst.lastMeasuredAgeMs = float(target.last_measured_age_ms)
  dst.ageCycles = _bounded(target.age_cycles, _UINT32_MAX)
  dst.existenceProbabilityCode = _bounded(target.existence_probability_code, 0xFF)
  dst.measurementState = _bounded(target.measurement_state, 0xFF)
  dst.dynamicProperty = _bounded(target.dynamic_property, 0xFF)
  dst.rawClass = _bounded(target.raw_class, 0xFF)
  dst.stableClass = _bounded(target.stable_class, 0xFF)
  dst.reasonBits = _bounded(target.reason_bits, _UINT32_MAX)


def _write_diagnostic_target(dst, kind: str, target: PerceptionTarget, coordinate_unverified: bool,
                             observed_producer_epoch: int, observed_sequence: int, observed_mono_time: int) -> None:
  dst.kind = kind
  _write_compact_target(dst.target, target)
  dst.extendedFresh = bool(target.extended_fresh)
  dst.hitCount = _bounded(target.hit_count, _UINT32_MAX)
  dst.missCount = _bounded(target.miss_count, _UINT32_MAX)
  dst.rawDRel = float(target.raw_d_rel)
  dst.rawYRel = float(target.raw_y_rel)
  dst.rawVRel = float(target.raw_v_rel)
  dst.rawYvRel = float(target.raw_yv_rel)
  dst.aRelLong = float(target.a_rel_long)
  dst.aRelLat = float(target.a_rel_lat)
  dst.orientation = float(target.orientation)
  dst.length = float(target.length)
  dst.width = float(target.width)
  dst.rcs = float(target.rcs)
  dst.clutterScore = float(target.clutter_score)
  dst.yvRel = float(target.yv_rel)
  dst.qualityScore = float(target.quality_score)
  dst.coordinateUnverified = coordinate_unverified
  dst.observedProducerEpoch = _bounded(observed_producer_epoch, (1 << 64) - 1)
  dst.observedSequence = _bounded(observed_sequence, _UINT32_MAX)
  dst.observedMonoTime = _bounded(observed_mono_time, (1 << 64) - 1)
  dst.rmsCodes = [_bounded(code, 0xFF) for code in target.rms_codes]
  valid_mask = 0
  upper_bounds: list[float] = []
  for index, upper_bound in enumerate(target.rms_upper_bounds):
    if upper_bound is None:
      upper_bounds.append(0.0)
    else:
      valid_mask |= 1 << index
      upper_bounds.append(float(upper_bound))
  dst.rmsUpperBounds = upper_bounds
  dst.rmsValidMask = _bounded(valid_mask, 0xFF)


def _error_flags(perception_state: Any) -> tuple[bool, bool, bool, bool]:
  diagnostics = getattr(perception_state, "diagnostics", None)
  # Standard diagnostics preserve their historical arrival-based behavior.
  # The shadow channel only consumes source-monotonic trusted faults.
  errors = getattr(diagnostics, "trusted_errors", None)
  return (
    bool(getattr(errors, "can_error", True)),
    bool(getattr(errors, "radar_fault", True)),
    bool(getattr(errors, "radar_unavailable_temporary", True)),
    bool(getattr(errors, "wrong_config", True)),
  )


class ARS408StatePublisher:
  """Build a bounded fail-safe shadow message without touching RadarData."""

  def __init__(self) -> None:
    self._active_epoch = 0
    self._last_attempted_sequence: int | None = None
    self._last_attempt_signature: tuple[object, ...] | None = None
    self._last_invalid_heartbeat_ns: int | None = None
    self._last_diagnostic_publish_ns: int | None = None
    self._last_diagnostic_accumulated_sequence: tuple[int, int] | None = None
    self._diagnostic_accumulator: dict[
      tuple[int, int], tuple[int, str, PerceptionTarget, bool, int, int, int]
    ] = {}
    self._diagnostic_full_count_pending = 0
    self._diagnostics_truncated_pending = False
    self._last_diagnostic_status_signature: tuple[object, ...] | None = None
    self._diagnostic_early_transition_used = False
    self._window_start_mono_time: int | None = None
    self._window_end_mono_time = 0
    self._window_reason_bits = 0
    self._window_transition_count = 0
    self._window_worst_health: str | None = None
    self._window_failures = 0
    self._diagnostic_prepare_retained_state: dict[str, Any] | None = None

  def _publisher_state(self) -> dict[str, Any]:
    state = {key: value for key, value in self.__dict__.items() if key != "_diagnostic_prepare_retained_state"}
    state["_diagnostic_accumulator"] = dict(self._diagnostic_accumulator)
    return state

  def _restore_publisher_state(self, state: dict[str, Any]) -> None:
    self.__dict__.update(state)
    self._diagnostic_prepare_retained_state = None

  def prepare_diagnostics(self, perception_state: Any, now_ns: int) -> PreparedDiagnostics:
    before = self._publisher_state()
    self._diagnostic_prepare_retained_state = None
    try:
      result = self._build_diagnostics_candidate(perception_state, now_ns)
      commit_state = self._publisher_state()
      retained_state = self._diagnostic_prepare_retained_state or commit_state
    except Exception:
      self._restore_publisher_state(before)
      producer_epoch = _bounded(getattr(perception_state, "producer_epoch", 0), (1 << 64) - 1)
      self._start_epoch(producer_epoch)
      snapshot = getattr(perception_state, "snapshot", None)
      if snapshot is not None:
        # Malformed observational input is never hot-loop retried, while the
        # previously committed accumulator/window remains unmodified.
        self._last_diagnostic_accumulated_sequence = (
          producer_epoch, int(getattr(snapshot, "sequence", 0)),
        )
      raise
    self._restore_publisher_state(before)
    return PreparedDiagnostics(result, commit_state, retained_state)

  def commit_diagnostics(self, prepared: PreparedDiagnostics, *, sent: bool) -> None:
    self._restore_publisher_state(prepared.commit_state if sent else prepared.retained_state)

  def _start_epoch(self, producer_epoch: int) -> None:
    if producer_epoch == self._active_epoch:
      return
    self._active_epoch = producer_epoch
    self._last_attempted_sequence = None
    self._last_attempt_signature = None
    self._last_invalid_heartbeat_ns = None
    self._last_diagnostic_accumulated_sequence = None
    # Physical diagnostic cadence, unsent targets, and the fault/status window
    # cross epochs and are cleared only by a committed physical send. Target
    # identity remains unambiguous because the accumulator key includes epoch.
    self._last_diagnostic_status_signature = None

  def build(self, perception_state: Any, now_ns: int):
    producer_epoch = _bounded(getattr(perception_state, "producer_epoch", 0), (1 << 64) - 1)
    self._start_epoch(producer_epoch)
    snapshot = getattr(perception_state, "snapshot", None)
    sequence = int(getattr(snapshot, "sequence", 0)) if snapshot is not None else 0
    measurement_mono_time = int(getattr(snapshot, "source_mono_time", 0)) if snapshot is not None else 0
    # CAN/log source timestamps and the host monotonic clock are not assumed to
    # share an epoch. The interface supplies an anchored source-domain "now".
    shadow_now_source_ns = int(getattr(perception_state, "shadow_now_source_ns", 0))
    source_age_ms, source_fresh = self._source_age(shadow_now_source_ns, measurement_mono_time)
    errors = _error_flags(perception_state)
    diagnostics_snapshot = getattr(perception_state, "diagnostics", None)
    # Shadow readiness must be explicitly bound to the source-monotonic trusted
    # RadarState path. Never inherit the standard arrival-based diagnostic gate.
    radar_state_ready = bool(getattr(perception_state, "radar_state_ready", False))
    radar_state_fresh = bool(getattr(perception_state, "radar_state_fresh", False))
    parser_valid = bool(getattr(perception_state, "parser_valid", False))
    motion_input_valid = bool(getattr(perception_state, "motion_input_valid", False))
    lateral_transform_valid = bool(getattr(perception_state, "lateral_transform_valid", False))
    interference_active = bool(getattr(perception_state, "interference_active", False))
    config_observed = bool(getattr(perception_state, "config_observed", False))
    config_valid = bool(getattr(perception_state, "config_valid", False))
    producer_fault = bool(getattr(perception_state, "producer_fault", False))
    producer_fault_count = int(getattr(perception_state, "producer_fault_count", 0))
    transient_failure_bits = int(getattr(perception_state, "transient_failure_bits", 0))
    cadence_inferred = bool(getattr(snapshot, "cadence_inferred", False)) if snapshot is not None else False
    step_source = _value(getattr(snapshot, "step_source", "unknown"), "unknown") if snapshot is not None else "unknown"
    if step_source not in _CADENCE_STEP_SOURCES:
      step_source = "unknown"
    expected_counter_step = int(getattr(snapshot, "expected_counter_step", 0)) if snapshot is not None else 0
    cadence_valid = bool(getattr(snapshot, "cadence_valid", False)) if snapshot is not None else False
    cadence_verified = bool(
      snapshot is not None and getattr(snapshot, "cadence_verified", False) and
      step_source in _VERIFIED_CADENCE_STEP_SOURCES and not cadence_inferred and
      expected_counter_step in _VERIFIED_COUNTER_STEPS
    )
    health = self._health(snapshot, source_fresh, radar_state_ready, radar_state_fresh,
                          parser_valid, motion_input_valid, lateral_transform_valid,
                          cadence_valid, cadence_verified, interference_active,
                          config_observed, config_valid, producer_fault, transient_failure_bits, errors)
    if producer_epoch == 0:
      health = "unavailable"
    event_valid = self._event_valid(snapshot, health)
    cycle_status = self._cycle_status(snapshot)
    counter_gap = int(getattr(snapshot, "counter_gap", 0)) if snapshot is not None else 0
    signature = (producer_epoch, health, event_valid, cycle_status, counter_gap, cadence_valid, cadence_verified,
                 cadence_inferred, step_source,
                 source_fresh, radar_state_ready,
                 radar_state_fresh, parser_valid, motion_input_valid, lateral_transform_valid, interference_active,
                 config_observed, config_valid, producer_fault, errors)
    signature += (transient_failure_bits,)
    new_sequence = snapshot is not None and sequence != self._last_attempted_sequence

    heartbeat_clock_reset = (self._last_invalid_heartbeat_ns is not None and
                             now_ns < self._last_invalid_heartbeat_ns)
    invalid_heartbeat_due = (not event_valid and
                             (self._last_invalid_heartbeat_ns is None or heartbeat_clock_reset or
                              now_ns - self._last_invalid_heartbeat_ns >= INVALID_HEARTBEAT_NS))
    if not (new_sequence or signature != self._last_attempt_signature or invalid_heartbeat_due):
      return None

    # Record attempts before serialization or send. A malformed shadow snapshot
    # must not be retried by the 100 Hz card loop; the next sequence can recover.
    if snapshot is not None:
      self._last_attempted_sequence = sequence
    self._last_attempt_signature = signature
    if not event_valid:
      self._last_invalid_heartbeat_ns = now_ns

    targets, targets_truncated = self._core_targets(snapshot) if event_valid else ([], False)
    state = custom.ARS408StateSP.new_message()
    state.schemaVersion = SCHEMA_VERSION
    state.producerEpoch = producer_epoch
    state.sequence = _bounded(sequence, _UINT32_MAX)
    state.measurementMonoTime = _bounded(measurement_mono_time, (1 << 64) - 1)
    state.lastComponentMonoTime = _bounded(getattr(snapshot, "last_component_mono_time", 0), (1 << 64) - 1)
    state.assemblyCloseMonoTime = _bounded(getattr(snapshot, "assembly_close_mono_time", 0), (1 << 64) - 1)
    state.measurementCounter = _bounded(getattr(snapshot, "measurement_counter", 0), _UINT16_MAX)
    state.counterGap = _bounded(counter_gap, _UINT16_MAX)
    state.droppedCycleCount = _bounded(getattr(snapshot, "dropped_cycle_count", 0), _UINT32_MAX)
    state.counterAnomalyCount = _bounded(getattr(snapshot, "counter_anomaly_count", 0), _UINT32_MAX)
    state.acceptedCount = _bounded(getattr(snapshot, "accepted_count", 0), _UINT16_MAX)
    state.cadenceValid = cadence_valid
    state.cadenceVerified = cadence_verified
    state.cadenceInferred = cadence_inferred
    state.stepSource = step_source
    state.expectedCounterStep = _bounded(expected_counter_step, _UINT16_MAX)
    state.cycleStatus = cycle_status
    state.motionInputValid = motion_input_valid
    state.generalComplete = bool(getattr(snapshot, "general_complete", False))
    state.qualityComplete = bool(getattr(snapshot, "quality_complete", False))
    state.extendedComplete = bool(getattr(snapshot, "extended_complete", False))
    state.rawCount = _bounded(getattr(snapshot, "raw_count", 0), _UINT16_MAX)
    state.health = health
    state.hasData = snapshot is not None
    state.sourceFresh = source_fresh
    state.sourceAgeMs = source_age_ms
    state.radarStateReady = radar_state_ready
    state.radarStateFresh = radar_state_fresh
    state.parserValid = parser_valid
    state.lateralTransformValid = lateral_transform_valid
    state.configObserved = config_observed
    state.configValid = config_valid
    state.producerFault = producer_fault
    state.producerFaultCount = _bounded(producer_fault_count, _UINT32_MAX)
    state.transientFailureBits = _bounded(transient_failure_bits, _UINT32_MAX)
    state.errors.canError, state.errors.radarFault, state.errors.radarUnavailableTemporary, state.errors.wrongConfig = errors
    state.interferenceCount = _bounded(
      getattr(diagnostics_snapshot, "trusted_interference_count", 0), _UINT32_MAX,
    )
    state.interferenceActive = interference_active

    stable, uncertain, clutter, expired = self._exclusive_counts(snapshot)
    state.stableCount = _bounded(stable, _UINT16_MAX)
    state.uncertainCount = _bounded(uncertain, _UINT16_MAX)
    state.clutterCount = _bounded(clutter, _UINT16_MAX)
    state.expiredCount = _bounded(expired, _UINT16_MAX)
    state.targetCount = len(targets)
    # Preserve the original wire slots while keeping the high-rate service
    # compact. Full diagnostics are emitted only on ars408DiagnosticsSP.
    state.init("diagnosticTargets", 0)
    state.diagnosticCount = 0
    state.diagnosticsTruncated = False
    state.diagnosticsUpdated = False
    state.diagnosticSequence = 0
    state.targetsTruncated = targets_truncated
    forward_present = any(float(target.d_rel) > 0.0 for target in targets)
    state.forwardPresenceState = "present" if event_valid and forward_present and not targets_truncated else "unknown"

    target_builders = state.init("targets", len(targets))
    for builder, target in zip(target_builders, targets, strict=True):
      _write_compact_target(builder, target)
    return state, event_valid

  def build_diagnostics(self, perception_state: Any, now_ns: int):
    """Compatibility helper for offline callers without a send transaction."""
    prepared = self.prepare_diagnostics(perception_state, now_ns)
    self.commit_diagnostics(prepared, sent=prepared.result is not None)
    return prepared.result

  def _build_diagnostics_candidate(self, perception_state: Any, now_ns: int):
    producer_epoch = _bounded(getattr(perception_state, "producer_epoch", 0), (1 << 64) - 1)
    self._start_epoch(producer_epoch)
    snapshot = getattr(perception_state, "snapshot", None)
    sequence = int(getattr(snapshot, "sequence", 0)) if snapshot is not None else 0
    lateral_transform_valid = bool(getattr(perception_state, "lateral_transform_valid", False))
    observation_key = (producer_epoch, sequence)
    if snapshot is not None and observation_key != self._last_diagnostic_accumulated_sequence:
      # Mark the sequence before touching accumulator state. A malformed
      # snapshot is observational data, not work to retry at card's 100 Hz.
      self._last_diagnostic_accumulated_sequence = observation_key
      self._accumulate_diagnostics(
        snapshot, producer_epoch=producer_epoch, coordinate_unverified=not lateral_transform_valid,
      )

    measurement_mono_time = int(getattr(snapshot, "source_mono_time", 0)) if snapshot is not None else 0
    shadow_now_source_ns = int(getattr(perception_state, "shadow_now_source_ns", 0))
    source_age_ms, source_fresh = self._source_age(shadow_now_source_ns, measurement_mono_time)
    errors = _error_flags(perception_state)
    diagnostics_snapshot = getattr(perception_state, "diagnostics", None)
    radar_state_ready = bool(getattr(perception_state, "radar_state_ready", False))
    radar_state_fresh = bool(getattr(perception_state, "radar_state_fresh", False))
    parser_valid = bool(getattr(perception_state, "parser_valid", False))
    motion_input_valid = bool(getattr(perception_state, "motion_input_valid", False))
    interference_active = bool(getattr(perception_state, "interference_active", False))
    config_observed = bool(getattr(perception_state, "config_observed", False))
    config_valid = bool(getattr(perception_state, "config_valid", False))
    producer_fault = bool(getattr(perception_state, "producer_fault", False))
    producer_fault_count = int(getattr(perception_state, "producer_fault_count", 0))
    transient_failure_bits = int(getattr(perception_state, "transient_failure_bits", 0))
    step_source = _value(getattr(snapshot, "step_source", "unknown"), "unknown") if snapshot is not None else "unknown"
    if step_source not in _CADENCE_STEP_SOURCES:
      step_source = "unknown"
    cadence_inferred = bool(getattr(snapshot, "cadence_inferred", False)) if snapshot is not None else False
    expected_counter_step = int(getattr(snapshot, "expected_counter_step", 0)) if snapshot is not None else 0
    cadence_valid = bool(getattr(snapshot, "cadence_valid", False)) if snapshot is not None else False
    cadence_verified = bool(
      snapshot is not None and getattr(snapshot, "cadence_verified", False) and
      step_source == "deviceCapture" and not cadence_inferred and
      expected_counter_step in _VERIFIED_COUNTER_STEPS
    )
    health = self._health(
      snapshot, source_fresh, radar_state_ready, radar_state_fresh, parser_valid, motion_input_valid,
      lateral_transform_valid, cadence_valid, cadence_verified, interference_active,
      config_observed, config_valid, producer_fault, transient_failure_bits, errors,
    )
    if producer_epoch == 0:
      health = "unavailable"
    cycle_status = self._cycle_status(snapshot)
    counter_gap = int(getattr(snapshot, "counter_gap", 0)) if snapshot is not None else 0
    counter_anomaly_count = int(getattr(snapshot, "counter_anomaly_count", 0)) if snapshot is not None else 0
    status_signature = (
      health, source_fresh, radar_state_ready, radar_state_fresh, parser_valid, motion_input_valid,
      lateral_transform_valid, interference_active, config_observed, config_valid, producer_fault, errors,
      cycle_status, counter_gap, counter_anomaly_count, cadence_valid, cadence_verified, cadence_inferred, step_source,
      transient_failure_bits,
    )
    status_transition = (self._last_diagnostic_status_signature is not None and
                         status_signature != self._last_diagnostic_status_signature)
    self._last_diagnostic_status_signature = status_signature
    window_time_ns = shadow_now_source_ns if shadow_now_source_ns > 0 else measurement_mono_time
    self._accumulate_window_status(
      window_time_ns=window_time_ns,
      health=health,
      status_transition=status_transition,
      source_fresh=source_fresh,
      radar_state_ready=radar_state_ready,
      radar_state_fresh=radar_state_fresh,
      parser_valid=parser_valid,
      motion_input_valid=motion_input_valid,
      config_observed=config_observed,
      config_valid=config_valid,
      interference_active=interference_active,
      cadence_valid=cadence_valid,
      cadence_verified=cadence_verified,
      cadence_inferred=cadence_inferred,
      cycle_status=cycle_status,
      counter_gap=counter_gap,
      producer_fault=producer_fault,
      transient_failure_bits=transient_failure_bits,
      errors=errors,
    )
    clock_reset = self._last_diagnostic_publish_ns is not None and now_ns < self._last_diagnostic_publish_ns
    interval_due = (self._last_diagnostic_publish_ns is None or clock_reset or
                    now_ns - self._last_diagnostic_publish_ns >= DIAGNOSTIC_PUBLISH_NS)
    early_transition_due = status_transition and not self._diagnostic_early_transition_used
    if not interval_due and not early_transition_due:
      return None
    # Retain the fully accumulated window before publication state advances or
    # the selected target/window records are taken. A suppressed/failed send
    # commits this state so the next physical event can retry the evidence.
    self._diagnostic_prepare_retained_state = self._publisher_state()
    # Record the attempt before serialization so malformed diagnostic input is
    # not retried from card's 100 Hz loop.
    self._last_diagnostic_publish_ns = now_ns
    self._diagnostic_early_transition_used = not interval_due
    diagnostic_targets, full_count, truncated, kind_counts = self._take_diagnostics()
    window = self._take_window()

    state = custom.ARS408DiagnosticsSP.new_message()
    state.schemaVersion = SCHEMA_VERSION
    state.producerEpoch = producer_epoch
    state.sequence = _bounded(sequence, _UINT32_MAX)
    state.measurementMonoTime = _bounded(measurement_mono_time, (1 << 64) - 1)
    state.health = health
    state.hasData = snapshot is not None
    state.sourceFresh = source_fresh
    state.sourceAgeMs = source_age_ms
    state.coordinateUnverified = not lateral_transform_valid
    state.fullCount = _bounded(full_count, _UINT16_MAX)
    state.targetCount = len(diagnostic_targets)
    state.truncated = truncated
    state.lastComponentMonoTime = _bounded(getattr(snapshot, "last_component_mono_time", 0), (1 << 64) - 1)
    state.assemblyCloseMonoTime = _bounded(getattr(snapshot, "assembly_close_mono_time", 0), (1 << 64) - 1)
    state.parserValid = parser_valid
    state.errors.canError, state.errors.radarFault, state.errors.radarUnavailableTemporary, state.errors.wrongConfig = errors
    state.radarStateReady = radar_state_ready
    state.radarStateFresh = radar_state_fresh
    state.configObserved = config_observed
    state.configValid = config_valid
    state.interferenceActive = interference_active
    state.interferenceCount = _bounded(
      getattr(diagnostics_snapshot, "trusted_interference_count", 0), _UINT32_MAX,
    )
    state.motionInputValid = motion_input_valid
    state.lateralTransformValid = lateral_transform_valid
    state.producerFault = producer_fault
    state.producerFaultCount = _bounded(producer_fault_count, _UINT32_MAX)
    state.transientFailureBits = _bounded(transient_failure_bits, _UINT32_MAX)
    state.cycleStatus = cycle_status
    state.measurementCounter = _bounded(getattr(snapshot, "measurement_counter", 0), _UINT16_MAX)
    state.counterGap = _bounded(counter_gap, _UINT16_MAX)
    state.droppedCycleCount = _bounded(getattr(snapshot, "dropped_cycle_count", 0), _UINT32_MAX)
    state.counterAnomalyCount = _bounded(counter_anomaly_count, _UINT32_MAX)
    state.cadenceValid = cadence_valid
    state.cadenceVerified = cadence_verified
    state.cadenceInferred = cadence_inferred
    state.expectedCounterStep = _bounded(expected_counter_step, _UINT16_MAX)
    state.stepSource = step_source
    state.generalComplete = bool(getattr(snapshot, "general_complete", False))
    state.qualityComplete = bool(getattr(snapshot, "quality_complete", False))
    state.extendedComplete = bool(getattr(snapshot, "extended_complete", False))
    state.rawCount = _bounded(getattr(snapshot, "raw_count", 0), _UINT16_MAX)
    state.acceptedCount = _bounded(getattr(snapshot, "accepted_count", 0), _UINT16_MAX)
    state.stableFullCount = _bounded(kind_counts["stableAudit"], _UINT16_MAX)
    state.uncertainFullCount = _bounded(kind_counts["uncertain"], _UINT16_MAX)
    state.clutterFullCount = _bounded(kind_counts["clutter"], _UINT16_MAX)
    state.expiredFullCount = _bounded(kind_counts["expired"], _UINT16_MAX)
    state.windowStartMonoTime = _bounded(window["start"], (1 << 64) - 1)
    state.windowEndMonoTime = _bounded(window["end"], (1 << 64) - 1)
    state.windowReasonBits = _bounded(window["reason_bits"], _UINT32_MAX)
    state.transitionCount = _bounded(window["transition_count"], _UINT32_MAX)
    state.worstHealth = window["worst_health"]
    state.sourceFailed = bool(window["failures"] & _WINDOW_SOURCE_FAILED)
    state.radarFailed = bool(window["failures"] & _WINDOW_RADAR_FAILED)
    state.parserFailed = bool(window["failures"] & _WINDOW_PARSER_FAILED)
    state.motionFailed = bool(window["failures"] & _WINDOW_MOTION_FAILED)
    state.configFailed = bool(window["failures"] & _WINDOW_CONFIG_FAILED)
    state.interferenceFailed = bool(window["failures"] & _WINDOW_INTERFERENCE_FAILED)
    state.cadenceFailed = bool(window["failures"] & _WINDOW_CADENCE_FAILED)
    state.counterFailed = bool(window["failures"] & _WINDOW_COUNTER_FAILED)
    state.producerFailed = bool(window["failures"] & _WINDOW_PRODUCER_FAILED)
    builders = state.init("targets", len(diagnostic_targets))
    for builder, (kind, target, coordinate_unverified, observed_epoch, observed_sequence, observed_mono_time) in zip(
      builders, diagnostic_targets, strict=True,
    ):
      _write_diagnostic_target(
        builder, kind, target, coordinate_unverified, observed_epoch, observed_sequence, observed_mono_time,
      )
    return state, health == "healthy"

  @staticmethod
  def _source_age(now_ns: int, source_mono_time: int) -> tuple[int, bool]:
    if source_mono_time <= 0 or now_ns < source_mono_time:
      return _UINT32_MAX, False
    age_ns = now_ns - source_mono_time
    return _bounded(age_ns // 1_000_000, _UINT32_MAX), age_ns <= SOURCE_FRESH_NS

  @staticmethod
  def _cycle_status(snapshot: PerceptionSnapshot | None) -> str:
    if snapshot is None:
      return "invalid"
    cycle_status = _value(snapshot.cycle_status, "invalid")
    return cycle_status if cycle_status in _CYCLE_STATUSES else "invalid"

  @staticmethod
  def _health(snapshot: PerceptionSnapshot | None, source_fresh: bool, radar_state_ready: bool,
              radar_state_fresh: bool, parser_valid: bool, motion_input_valid: bool,
              lateral_transform_valid: bool, cadence_valid: bool,
              cadence_verified: bool,
              interference_active: bool, config_observed: bool, config_valid: bool, producer_fault: bool,
              transient_failure_bits: int,
              errors: tuple[bool, bool, bool, bool]) -> str:
    if producer_fault:
      return "producerFault"
    if snapshot is None or not radar_state_ready or not config_observed:
      return "unavailable"
    if not source_fresh:
      return "stale"
    if (transient_failure_bits != 0 or not radar_state_fresh or not parser_valid or not lateral_transform_valid or
        not cadence_valid or not cadence_verified or not motion_input_valid or
        interference_active or not config_valid or
        any(errors)):
      return "degraded"
    return "healthy"

  @classmethod
  def _event_valid(cls, snapshot: PerceptionSnapshot | None, health: str) -> bool:
    return bool(snapshot is not None and health == "healthy" and cls._cycle_status(snapshot) == "exact" and
                snapshot.general_complete and snapshot.quality_complete and snapshot.extended_complete and
                int(getattr(snapshot, "counter_gap", 0)) == 0)

  @staticmethod
  def _exclusive_maps(snapshot: PerceptionSnapshot | None) -> tuple[dict[int, PerceptionTarget], ...]:
    if snapshot is None:
      return {}, {}, {}, {}
    expired = {_logical_id(target): target for target in snapshot.expired_events}
    clutter = {_logical_id(target): target for target in snapshot.clutter_suspects
               if _logical_id(target) not in expired}
    stable = {_logical_id(target): target for target in snapshot.stable_targets
              if _logical_id(target) not in clutter and _logical_id(target) not in expired}
    uncertain = {_logical_id(target): target for target in snapshot.uncertain_targets
                 if _logical_id(target) not in clutter and _logical_id(target) not in stable and
                 _logical_id(target) not in expired}
    return stable, uncertain, clutter, expired

  @classmethod
  def _exclusive_counts(cls, snapshot: PerceptionSnapshot | None) -> tuple[int, int, int, int]:
    stable, uncertain, clutter, expired = cls._exclusive_maps(snapshot)
    return len(stable), len(uncertain), len(clutter), len(expired)

  @classmethod
  def _core_targets(cls, snapshot: PerceptionSnapshot) -> tuple[list[PerceptionTarget], bool]:
    stable, _, _, _ = cls._exclusive_maps(snapshot)
    candidates = list(stable.values())
    candidates.sort(key=lambda target: (
      not bool(target.possible_vru), not bool(target.fresh_measured), abs(float(target.d_rel)), _logical_id(target),
    ))
    return candidates[:CORE_TARGET_LIMIT], len(candidates) > CORE_TARGET_LIMIT

  @staticmethod
  def _diagnostic_priority(kind: str, target: PerceptionTarget) -> int:
    if bool(target.possible_vru) or int(target.reason_bits) & _CLASS_CONFLICT_BIT:
      return 0
    return {"expired": 1, "clutter": 2, "uncertain": 3, "stableAudit": 4}.get(kind, 5)

  def _accumulate_diagnostics(self, snapshot: PerceptionSnapshot, *, producer_epoch: int,
                              coordinate_unverified: bool) -> None:
    stable, uncertain, clutter, expired = self._exclusive_maps(snapshot)
    observed_sequence = int(getattr(snapshot, "sequence", 0))
    observed_mono_time = int(getattr(snapshot, "source_mono_time", 0))
    candidates: list[tuple[str, PerceptionTarget]] = []
    candidates.extend(("expired", target) for target in expired.values())
    candidates.extend(("clutter", target) for target in clutter.values())
    candidates.extend(("uncertain", target) for target in uncertain.values())
    candidates.extend(("stableAudit", target) for target in stable.values())

    # Construct and validate an isolated candidate delta first. Nothing below
    # mutates the live window until every candidate and ordering key succeeds.
    candidate_delta: dict[tuple[int, int], tuple[int, str, PerceptionTarget, bool, int, int, int]] = {}
    candidate_reason_bits = 0
    for kind, target in candidates:
      logical_id = _logical_id(target)
      identity = (producer_epoch, logical_id)
      priority = self._diagnostic_priority(kind, target)
      candidate_reason_bits |= _bounded(target.reason_bits, _UINT32_MAX)
      previous = candidate_delta.get(identity)
      if previous is None or priority <= previous[0]:
        candidate_delta[identity] = (
          priority, kind, target, coordinate_unverified, producer_epoch, observed_sequence, observed_mono_time,
        )

    merged = dict(self._diagnostic_accumulator)
    for identity, candidate in candidate_delta.items():
      previous = merged.get(identity)
      if previous is None or candidate[0] <= previous[0]:
        # Keep the selected record's actual observation provenance. This is not
        # replaced with the later diagnostic publication header.
        merged[identity] = candidate
    full_count_pending = max(self._diagnostic_full_count_pending, len(merged))
    truncated_pending = self._diagnostics_truncated_pending
    if len(merged) > DIAGNOSTIC_ACCUMULATOR_LIMIT:
      ordered_items = sorted(merged.items(), key=lambda item: (
        item[1][0], not bool(item[1][2].possible_vru), _logical_id(item[1][2]),
      ))
      merged = dict(ordered_items[:DIAGNOSTIC_ACCUMULATOR_LIMIT])
      truncated_pending = True

    # Atomic commit after all candidate conversion and ordering has completed.
    self._diagnostic_accumulator = merged
    self._diagnostic_full_count_pending = full_count_pending
    self._diagnostics_truncated_pending = truncated_pending
    self._window_reason_bits |= candidate_reason_bits
    self._update_window_bounds(observed_mono_time)

  def _take_diagnostics(self) -> tuple[
    list[tuple[str, PerceptionTarget, bool, int, int, int]], int, bool, dict[str, int],
  ]:
    ordered = sorted(self._diagnostic_accumulator.values(), key=lambda item: (
      item[0], not bool(item[2].possible_vru), _logical_id(item[2]),
    ))
    full_count = max(self._diagnostic_full_count_pending, len(ordered))
    truncated = self._diagnostics_truncated_pending or full_count > DIAGNOSTIC_TARGET_LIMIT
    kind_counts = dict.fromkeys(("stableAudit", "uncertain", "clutter", "expired"), 0)
    for _, kind, *_ in ordered:
      if kind in kind_counts:
        kind_counts[kind] += 1
    selected = [
      (kind, target, coordinate_unverified, observed_epoch, observed_sequence, observed_mono_time)
      for _, kind, target, coordinate_unverified, observed_epoch, observed_sequence, observed_mono_time
      in ordered[:DIAGNOSTIC_TARGET_LIMIT]
    ]
    self._diagnostic_accumulator = {}
    self._diagnostic_full_count_pending = 0
    self._diagnostics_truncated_pending = False
    return selected, full_count, truncated, kind_counts

  def _update_window_bounds(self, mono_time_ns: int) -> None:
    if mono_time_ns <= 0:
      return
    if self._window_start_mono_time is None:
      self._window_start_mono_time = mono_time_ns
    else:
      self._window_start_mono_time = min(self._window_start_mono_time, mono_time_ns)
    self._window_end_mono_time = max(self._window_end_mono_time, mono_time_ns)

  def _accumulate_window_status(
    self, *, window_time_ns: int, health: str, status_transition: bool,
    source_fresh: bool, radar_state_ready: bool, radar_state_fresh: bool,
    parser_valid: bool, motion_input_valid: bool, config_observed: bool,
    config_valid: bool, interference_active: bool, cadence_valid: bool,
    cadence_verified: bool, cadence_inferred: bool, cycle_status: str,
    counter_gap: int, producer_fault: bool,
    transient_failure_bits: int,
    errors: tuple[bool, bool, bool, bool],
  ) -> None:
    self._update_window_bounds(window_time_ns)
    if status_transition:
      self._window_transition_count += 1
    if (self._window_worst_health is None or
        _HEALTH_SEVERITY.get(health, _HEALTH_SEVERITY["producerFault"]) >
        _HEALTH_SEVERITY.get(self._window_worst_health, -1)):
      self._window_worst_health = health

    can_error, radar_fault, radar_unavailable_temporary, wrong_config = errors
    failures = 0
    if not source_fresh:
      failures |= _WINDOW_SOURCE_FAILED
    if (not radar_state_ready or not radar_state_fresh or can_error or radar_fault or
        radar_unavailable_temporary):
      failures |= _WINDOW_RADAR_FAILED
    if not parser_valid:
      failures |= _WINDOW_PARSER_FAILED
    if not motion_input_valid:
      failures |= _WINDOW_MOTION_FAILED
    if not config_observed or not config_valid or wrong_config:
      failures |= _WINDOW_CONFIG_FAILED
    if interference_active:
      failures |= _WINDOW_INTERFERENCE_FAILED
    if not cadence_valid or not cadence_verified or cadence_inferred:
      failures |= _WINDOW_CADENCE_FAILED
    if counter_gap != 0 or cycle_status in ("invalid", "duplicate"):
      failures |= _WINDOW_COUNTER_FAILED
    if producer_fault:
      failures |= _WINDOW_PRODUCER_FAILED
    if transient_failure_bits & _TRANSIENT_SOURCE_FAILED:
      failures |= _WINDOW_SOURCE_FAILED
    if transient_failure_bits & _TRANSIENT_RADAR_FAILED:
      failures |= _WINDOW_RADAR_FAILED
    if transient_failure_bits & _TRANSIENT_CONFIG_FAILED:
      failures |= _WINDOW_CONFIG_FAILED
    if transient_failure_bits & _TRANSIENT_INTERFERENCE_FAILED:
      failures |= _WINDOW_INTERFERENCE_FAILED
    if transient_failure_bits & _TRANSIENT_MOTION_FAILED:
      failures |= _WINDOW_MOTION_FAILED
    if transient_failure_bits & _TRANSIENT_PARSER_FAILED:
      failures |= _WINDOW_PARSER_FAILED
    if transient_failure_bits & _TRANSIENT_PRODUCER_FAILED:
      failures |= _WINDOW_PRODUCER_FAILED
    self._window_failures |= failures
    self._window_reason_bits |= failures

  def _take_window(self) -> dict[str, int | str]:
    result: dict[str, int | str] = {
      "start": self._window_start_mono_time or 0,
      "end": self._window_end_mono_time,
      "reason_bits": self._window_reason_bits,
      "transition_count": self._window_transition_count,
      "worst_health": self._window_worst_health or "unavailable",
      "failures": self._window_failures,
    }
    self._window_start_mono_time = None
    self._window_end_mono_time = 0
    self._window_reason_bits = 0
    self._window_transition_count = 0
    self._window_worst_health = None
    self._window_failures = 0
    return result
