from openpilot.sunnypilot.navassist.protocol import AcceptedSnapshot, parse_snapshot
from openpilot.sunnypilot.navassist.publisher import build_nav_assist_message
from openpilot.sunnypilot.navassist.tests.test_protocol import encode, payload


def accepted(*, receive_ns=1_000_000_000, expires_ns=1_500_000_000, **kwargs):
  return AcceptedSnapshot(parse_snapshot(encode(payload(**kwargs))), receive_ns, expires_ns)


def test_no_data_is_invalid_and_stale():
  message = build_nav_assist_message(None, 1_000_000_000)
  assert not message.valid
  assert not message.navAssistStateSP.valid
  assert message.navAssistStateSP.stale
  assert message.navAssistStateSP.rejectReason == "noData"


def test_fresh_snapshot_maps_to_typed_cereal_without_control_commands():
  message = build_nav_assist_message(
    accepted(), 1_100_000_000, local_localization_valid=True,
  )
  state = message.navAssistStateSP
  assert message.valid and state.valid and not state.stale
  assert state.source == "android"
  assert state.mode == "realtime"
  assert state.maneuver == "turnRight"
  assert state.sourceAgeMs == 100.0
  assert state.advisorySpeedValid and state.advisorySpeedMps == 5.0
  assert state.lanes[0].recommended
  assert not state.lanes[0].routeAvoid
  assert state.parallelRoadStatus == "main"
  assert state.elevatedRoadStatus == "side"
  assert state.routeNoticeType == "roadClosed"
  assert state.routeNoticeDistanceM == 350.0
  assert state.rejectReason == "none"


def test_local_expiry_fails_closed_while_retaining_diagnostics():
  message = build_nav_assist_message(
    accepted(), 1_500_000_001, local_localization_valid=True,
  )
  state = message.navAssistStateSP
  assert message.valid
  assert not state.valid and state.stale
  assert state.sessionId == "session-a"
  assert state.rejectReason == "stale"
  assert len(state.lanes) == 0


def test_local_localization_is_diagnostic_only_for_fresh_matched_phone_guidance():
  no_localization = build_nav_assist_message(
    accepted(), 1_100_000_000, local_localization_valid=False,
  ).navAssistStateSP
  assert no_localization.valid and no_localization.rejectReason == "localLocalization"


def test_phone_gps_weak_flag_is_diagnostic_not_a_control_gate():
  raw = payload()
  raw["gpsWeak"] = True
  current = AcceptedSnapshot(parse_snapshot(encode(raw)), 1_000_000_000, 1_500_000_000)
  state = build_nav_assist_message(
    current, 1_100_000_000, local_localization_valid=True,
  ).navAssistStateSP
  assert state.gpsWeak
  assert state.valid and state.rejectReason == "none"


def test_non_realtime_or_non_mobile_source_cannot_become_control_valid():
  raw = payload()
  raw["navigationMode"] = "idle"
  current = AcceptedSnapshot(parse_snapshot(encode(raw)), 1_000_000_000, 1_500_000_000)
  state = build_nav_assist_message(
    current, 1_100_000_000, local_localization_valid=True,
  ).navAssistStateSP
  assert not state.valid and state.rejectReason == "noData"

  raw = payload()
  raw["sourcePlatform"] = "track"
  current = AcceptedSnapshot(parse_snapshot(encode(raw)), 1_000_000_000, 1_500_000_000)
  state = build_nav_assist_message(
    current, 1_100_000_000, local_localization_valid=True,
  ).navAssistStateSP
  assert not state.valid and state.rejectReason == "noData"


def test_missing_phone_observation_blocks_are_retained_as_invalid_diagnostics():
  raw = payload()
  raw.pop("location")
  raw.pop("guidance")
  raw.pop("lanes")
  current = AcceptedSnapshot(parse_snapshot(encode(raw)), 1_000_000_000, 1_500_000_000)
  state = build_nav_assist_message(
    current, 1_100_000_000, local_localization_valid=True,
  ).navAssistStateSP
  assert not state.valid and state.rejectReason == "noData"


def test_unchanged_phone_guidance_remains_valid_while_transport_and_route_are_fresh():
  raw = payload()
  raw["guidance"]["observedAtMs"] = raw["sourceWallTimeMs"] - 45_000
  raw["location"]["observedAtMs"] = raw["sourceWallTimeMs"] - 45_000
  old_guidance = AcceptedSnapshot(parse_snapshot(encode(raw)), 1_000_000_000, 1_500_000_000)
  state = build_nav_assist_message(
    old_guidance, 1_100_000_000, local_localization_valid=True,
  ).navAssistStateSP
  assert state.valid and state.rejectReason == "phoneLocalization"


def test_phone_location_quality_is_diagnostic_only_for_fresh_matched_guidance():
  raw = payload()
  raw["location"]["accuracyM"] = 26.0
  inaccurate = AcceptedSnapshot(parse_snapshot(encode(raw)), 1_000_000_000, 1_500_000_000)
  state = build_nav_assist_message(
    inaccurate, 1_100_000_000, local_localization_valid=True,
  ).navAssistStateSP
  assert state.valid and state.rejectReason == "phoneLocalization"


def test_unchanged_lane_event_is_held_by_fresh_snapshots_without_refreshing_observation_time():
  raw = payload()
  observed_at_ms = raw["lanes"]["observedAtMs"]
  for sequence, age_ms in enumerate((0, 2_001, 8_052, 45_000), start=1):
    raw["sequence"] = sequence
    raw["sourceWallTimeMs"] = observed_at_ms + age_ms
    receive_ns = 1_000_000_000 + age_ms * 1_000_000
    current = AcceptedSnapshot(parse_snapshot(encode(raw)), receive_ns, receive_ns + 500_000_000)
    state = build_nav_assist_message(current, receive_ns + 100_000_000, local_localization_valid=True).navAssistStateSP
    assert state.valid and not state.stale and state.sourceAgeMs == 100.0
    assert len(state.lanes) == 1 and state.lanes[0].recommended
    assert state.lanes[0].recommendedActions == current.snapshot.lanes[0].recommended_actions
    assert state.laneGuidanceObservedAtMs == observed_at_ms


def test_absent_or_empty_lane_block_clears_previously_published_lanes():
  for absence in ("missing", "empty"):
    assert len(build_nav_assist_message(accepted(), 1_100_000_000).navAssistStateSP.lanes) == 1
    raw = payload(sequence=2)
    if absence == "missing":
      raw.pop("lanes")
    else:
      raw["lanes"]["items"] = []
    current = AcceptedSnapshot(parse_snapshot(encode(raw)), 1_100_000_000, 1_600_000_000)
    state = build_nav_assist_message(current, 1_200_000_000, local_localization_valid=True).navAssistStateSP
    assert state.valid and len(state.lanes) == 0


def test_future_lane_observation_is_hidden_without_disabling_longitudinal_guidance():
  raw = payload()
  raw["lanes"]["observedAtMs"] = raw["sourceWallTimeMs"] + 1
  current = AcceptedSnapshot(parse_snapshot(encode(raw)), 1_000_000_000, 1_500_000_000)
  state = build_nav_assist_message(current, 1_100_000_000, local_localization_valid=True).navAssistStateSP
  assert state.valid and len(state.lanes) == 0
  assert state.laneGuidanceObservedAtMs == raw["lanes"]["observedAtMs"]


def test_new_route_without_lane_block_does_not_reuse_previous_route_lanes():
  first = build_nav_assist_message(accepted(), 1_100_000_000).navAssistStateSP
  assert len(first.lanes) == 1
  raw = payload(sequence=2, route_revision=2)
  raw.pop("lanes")
  current = AcceptedSnapshot(parse_snapshot(encode(raw)), 1_100_000_000, 1_600_000_000)
  state = build_nav_assist_message(current, 1_200_000_000, local_localization_valid=True).navAssistStateSP
  assert state.valid and state.routeRevision == 2 and len(state.lanes) == 0
