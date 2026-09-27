import json
from pathlib import Path

import pytest

from openpilot.sunnypilot.navassist.protocol import SCHEMA_VERSION, LANE_ACTION_BITS, NavAssistProtocolError, NavAssistStore, parse_snapshot


APP_KEY_ID = "a" * 32
SOURCE_WALL_MS = 1_700_000_000_000


def test_checked_in_schema_matches_the_runtime_protocol_version():
  schema_path = Path(__file__).parents[1] / "nav-assist-v3.schema.json"
  schema = json.loads(schema_path.read_text())
  assert schema["properties"]["schemaVersion"]["const"] == SCHEMA_VERSION
  assert schema["$id"].endswith("navassist-v3.schema.json")


def store(**kwargs):
  return NavAssistStore(wall_clock_ms=lambda: SOURCE_WALL_MS, **kwargs)


def payload(*, session_id="session-a", sequence=1, route_revision=1, valid_for_ms=500):
  return {
    "schemaVersion": 3,
    "messageType": "navigation_snapshot",
    "sessionId": session_id,
    "sequence": sequence,
    "routeRevision": route_revision,
    "maneuverEventId": 7,
    "sourcePlatform": "android",
    "sourceWallTimeMs": SOURCE_WALL_MS,
    "validForMs": valid_for_ms,
    "navigationMode": "realtime",
    "routeActive": True,
    "routeMatched": True,
    "gpsWeak": False,
    "coordinateSystem": "gcj02",
    "location": {
      "latitude": 31.2,
      "longitude": 121.4,
      "accuracyM": 3.0,
      "bearingDeg": 90.0,
      "speedKph": 30.0,
      "observedAtMs": SOURCE_WALL_MS,
      "currentStepIndex": 2,
      "currentLinkIndex": 1,
      "currentPointIndex": 10,
    },
    "guidance": {
      "observedAtMs": SOURCE_WALL_MS,
      "maneuver": "turn_right",
      "maneuverDistanceM": 80,
      "nextManeuver": "straight",
      "nextManeuverDistanceM": 450,
      "currentRoad": "测试主路",
      "nextRoad": "测试匝道",
      "roadClass": 0,
      "roadType": 6,
      "advisorySpeedMps": 5.0,
      "parallelRoadStatus": "main",
      "elevatedRoadStatus": "side",
      "routeNoticeType": "road_closed",
      "routeNoticeDistanceM": 350,
      "routeNoticeObservedAtMs": SOURCE_WALL_MS,
    },
    "lanes": {
      "observedAtMs": SOURCE_WALL_MS,
      "items": [{
        "index": 0,
        "allowedActions": ["STRAIGHT", "RIGHT"],
        "recommendedActions": ["RIGHT"],
        "recommended": True,
        "routeAvoid": False,
      }],
    },
  }


def encode(value):
  return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode()


def test_parses_bounded_normalized_snapshot():
  snapshot = parse_snapshot(encode(payload()))
  assert snapshot.session_id == "session-a"
  assert snapshot.current_link_index == 1
  assert snapshot.maneuver == "turn_right"
  assert snapshot.guidance_observed_at_ms == 1_700_000_000_000
  assert snapshot.lanes[0].allowed_actions == LANE_ACTION_BITS["STRAIGHT"] | LANE_ACTION_BITS["RIGHT"]
  assert snapshot.lanes[0].recommended_actions == LANE_ACTION_BITS["RIGHT"]
  assert not snapshot.lanes[0].route_avoid
  assert snapshot.parallel_road_status == "main"
  assert snapshot.elevated_road_status == "side"
  assert snapshot.route_notice_type == "road_closed"
  assert snapshot.route_notice_distance_m == 350


def test_sequence_route_revision_and_local_ttl():
  now = 1_000_000_000
  receiver = store(clock_ns=lambda: now)
  first_body = encode(payload())
  accepted = receiver.accept(first_body, APP_KEY_ID)
  assert not accepted.is_stale(now + 500_000_000)
  assert accepted.is_stale(now + 500_000_001)

  with pytest.raises(NavAssistProtocolError, match="sequence") as duplicate:
    receiver.accept(first_body, APP_KEY_ID)
  assert duplicate.value.reason == "replay"

  rollback_body = encode(payload(sequence=2, route_revision=0))
  with pytest.raises(NavAssistProtocolError, match="routeRevision"):
    receiver.accept(rollback_body, APP_KEY_ID)

  next_body = encode(payload(sequence=2, route_revision=2))
  assert receiver.accept(next_body, APP_KEY_ID).snapshot.sequence == 2


def test_new_session_may_join_after_failed_sends_but_retired_session_cannot_return():
  receiver = store(clock_ns=lambda: 1)
  first = encode(payload())
  receiver.accept(first, APP_KEY_ID)

  # The phone increments sequence on failed HTTP attempts, so a receiver that
  # starts later must be able to join an authenticated session mid-stream.
  valid_new = encode(payload(session_id="session-b", sequence=20))
  receiver.accept(valid_new, APP_KEY_ID)
  retired = encode(payload(session_id="session-a", sequence=2))
  with pytest.raises(NavAssistProtocolError, match="retired session"):
    receiver.accept(retired, APP_KEY_ID)


def test_second_paired_app_cannot_preempt_a_fresh_active_navigation_session():
  now = 1_000_000_000
  receiver = store(clock_ns=lambda: now)
  receiver.accept(encode(payload(session_id="android-active")), "a" * 32)

  with pytest.raises(NavAssistProtocolError, match="another paired app") as busy:
    receiver.accept(encode(payload(session_id="ios-active")), "b" * 32)
  assert busy.value.reason == "replay"

  now += 500_000_001
  assert receiver.accept(encode(payload(session_id="ios-active")), "b" * 32).snapshot.session_id == "ios-active"


def test_rejects_bad_app_identity_non_finite_unknown_fields_and_ranges():
  body = encode(payload())
  receiver = store()
  with pytest.raises(NavAssistProtocolError) as auth:
    receiver.accept(body, "not-a-key-id")
  assert auth.value.reason == "authentication"

  non_finite = body.replace(b'"accuracyM":3.0', b'"accuracyM":NaN')
  with pytest.raises(NavAssistProtocolError) as bad_number:
    parse_snapshot(non_finite)
  assert bad_number.value.reason == "malformed"

  extra = payload()
  extra["steeringAngleDeg"] = 5.0
  with pytest.raises(NavAssistProtocolError, match="unknown fields"):
    parse_snapshot(encode(extra))

  out_of_range = payload()
  out_of_range["location"]["latitude"] = 100.0
  with pytest.raises(NavAssistProtocolError, match="latitude"):
    parse_snapshot(encode(out_of_range))

  duplicate_key = body.replace(b'"sequence":1', b'"sequence":1,"sequence":2')
  with pytest.raises(NavAssistProtocolError, match="duplicate JSON field sequence"):
    parse_snapshot(duplicate_key)


def test_rejects_duplicate_lanes_and_excessive_ttl():
  duplicate = payload()
  duplicate["lanes"]["items"].append(dict(duplicate["lanes"]["items"][0]))
  with pytest.raises(NavAssistProtocolError, match="duplicate lane"):
    parse_snapshot(encode(duplicate))

  with pytest.raises(NavAssistProtocolError, match="validForMs"):
    parse_snapshot(encode(payload(valid_for_ms=10_000)))

  duplicate_action = payload()
  duplicate_action["lanes"]["items"][0]["allowedActions"] = ["RIGHT", "RIGHT"]
  with pytest.raises(NavAssistProtocolError, match="duplicate action"):
    parse_snapshot(encode(duplicate_action))


def test_first_observed_authenticated_session_can_start_above_sequence_one():
  receiver = store(clock_ns=lambda: 1)
  body = encode(payload(sequence=42))
  assert receiver.accept(body, APP_KEY_ID).snapshot.sequence == 42


def test_source_wall_time_bounds_replay_even_without_a_checkpoint():
  old = payload()
  old["sourceWallTimeMs"] = SOURCE_WALL_MS - 1_001
  body = encode(old)
  with pytest.raises(NavAssistProtocolError, match="freshness window") as error:
    store().accept(body, APP_KEY_ID)
  assert error.value.reason == "replay"

  future = payload()
  future["sourceWallTimeMs"] = SOURCE_WALL_MS + 1_001
  body = encode(future)
  with pytest.raises(NavAssistProtocolError, match="freshness window"):
    store().accept(body, APP_KEY_ID)


def test_replay_checkpoint_survives_daemon_store_restart(tmp_path):
  checkpoint = tmp_path / "navassist-replay.json"
  first_body = encode(payload(sequence=7))
  store(checkpoint_path=checkpoint).accept(first_body, APP_KEY_ID)

  restarted = store(checkpoint_path=checkpoint)
  with pytest.raises(NavAssistProtocolError, match="sequence") as replay:
    restarted.accept(first_body, APP_KEY_ID)
  assert replay.value.reason == "replay"

  next_body = encode(payload(sequence=8))
  assert restarted.accept(next_body, APP_KEY_ID).snapshot.sequence == 8


def test_explicit_offroad_pairing_reset_clears_live_and_persisted_replay_state(tmp_path):
  checkpoint = tmp_path / "navassist-replay.json"
  receiver = store(checkpoint_path=checkpoint)
  body = encode(payload(sequence=7))
  receiver.accept(body, APP_KEY_ID)
  assert checkpoint.exists() and receiver.current() is not None

  receiver.reset()
  assert not checkpoint.exists() and receiver.current() is None
  assert receiver.accept(body, APP_KEY_ID).snapshot.sequence == 7


def test_corrupt_matching_replay_checkpoint_fails_closed(tmp_path):
  checkpoint = tmp_path / "navassist-replay.json"
  checkpoint.write_text("not-json")
  with pytest.raises(RuntimeError, match="checkpoint is unreadable"):
    store(checkpoint_path=checkpoint)


def test_idle_snapshot_may_omit_observation_blocks_but_is_not_control_valid():
  idle = payload()
  idle["navigationMode"] = "idle"
  idle["routeActive"] = False
  idle.pop("routeMatched")
  idle.pop("location")
  idle.pop("guidance")
  idle.pop("lanes")
  snapshot = parse_snapshot(encode(idle))
  assert not snapshot.route_matched
  assert not snapshot.location_present
  assert not snapshot.guidance_present
  assert not snapshot.lane_guidance_present

  incomplete = payload()
  incomplete["location"].pop("latitude")
  with pytest.raises(NavAssistProtocolError, match="latitude"):
    parse_snapshot(encode(incomplete))


def test_undocumented_road_metadata_is_rejected():
  invalid = payload()
  invalid["guidance"]["roadType"] = 57
  with pytest.raises(NavAssistProtocolError, match="roadType"):
    parse_snapshot(encode(invalid))
  invalid = payload()
  invalid["guidance"]["roadClass"] = 11
  with pytest.raises(NavAssistProtocolError, match="roadClass"):
    parse_snapshot(encode(invalid))

  invalid = payload()
  invalid["location"]["currentStepIndex"] = -1
  with pytest.raises(NavAssistProtocolError, match="currentStepIndex"):
    parse_snapshot(encode(invalid))

  invalid = payload()
  invalid["guidance"]["currentRoad"] = ""
  with pytest.raises(NavAssistProtocolError, match="currentRoad"):
    parse_snapshot(encode(invalid))
