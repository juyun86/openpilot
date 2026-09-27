from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.navassist.oem_navigation_feedback import OemNavigationFeedback, GPS_MAX_AGE_NS


T = 20_000_000_000


def frame(address, value, src=1):
  return SimpleNamespace(address=address, dat=value.to_bytes(8, "little") if isinstance(value, int) else bytes.fromhex(value), src=src)


def gps(lat=32_392873, lon=120_591314):
  return frame(0x04F, (lat & ((1 << 28) - 1)) | ((lon & ((1 << 29) - 1)) << 28) | (6 << 57))


def motion(extra=0):
  return frame(0x2F8, 7 | (11300 << 8) | (6253 << 24) | extra)


def ingest(observer, frames, now=T, **kwargs):
  observer.ingest(frames, now_ns=now, received_ns=now, **kwargs)


def test_real_debug_frames_and_model_y_gps_layout():
  observer = OemNavigationFeedback()
  ingest(observer, [gps(), motion(), frame(0x247, "04001DFC00400800"), frame(0x24A, "0000010F9F008001")])
  value = observer.snapshot(T)
  assert value == {
    "gpsStatus": "observed", "latitude": 32.392873, "longitude": 120.591314,
    "gpsTimeMs": None, "gpsTimeAgeMs": None, "gpsPositionAgeMs": 0, "gpsMotionAgeMs": 0,
    "gpsAccuracyRaw": None, "gpsAccuracyAgeMs": None,
    "accuracyValue": 1.2, "hdop": 0.7, "headingDeg": 88.281, "speedValue": 24.426,
    "mapAvailable": False, "controllerHealth": 0, "alcState": 0, "forkState": 2, "abortReason": 0,
    "roadEstimator": 0, "oemLaneChangeState": 2,
    "navAvailable": True, "navUsage": 0, "autosteerHealth": 0, "plannerState": 0, "navDistanceM": 0,
  }


def test_signed_coordinates_are_not_unsigned_or_float32():
  observer = OemNavigationFeedback()
  ingest(observer, [gps(-32_392873, -120_591314), motion()])
  assert observer.snapshot(T)["latitude"] == -32.392873
  assert observer.snapshot(T)["longitude"] == -120.591314


def test_expiry_is_per_source_and_exact_boundary():
  observer = OemNavigationFeedback()
  ingest(observer, [gps(), motion(), frame(0x247, 0), frame(0x24A, 0)])
  assert observer.snapshot(T + 499_999_999)["mapAvailable"] is False
  value = observer.snapshot(T + 500_000_000)
  assert value["gpsStatus"] == "observed" and value["mapAvailable"] is None and value["navAvailable"] is None
  assert observer.snapshot(T + GPS_MAX_AGE_NS)["gpsStatus"] == "stale"
  assert observer.snapshot(T + GPS_MAX_AGE_NS)["latitude"] is None


@pytest.mark.parametrize("bad", [frame(0x04F, "010203"), frame(0x04F, "FFFFFFFFFFFFFFFF"), gps(100_000000), gps(lon=190_000000), frame(0x04F, 0)])
def test_bad_position_immediately_removes_previous_fix(bad):
  observer = OemNavigationFeedback()
  ingest(observer, [gps(), motion()])
  ingest(observer, [bad], T + 1)
  assert observer.snapshot(T + 1)["gpsStatus"] == "invalid"
  assert observer.snapshot(T + 1)["latitude"] is None


@pytest.mark.parametrize("extra", [1 << 53, 1 << 54, 248])
def test_gps_faults_and_hdop_sentinel_invalidate_fix(extra):
  observer = OemNavigationFeedback()
  ingest(observer, [gps(), motion(extra)])
  assert observer.snapshot(T)["gpsStatus"] == "invalid"


def test_heading_and_speed_sentinels_do_not_invent_zero_motion():
  observer = OemNavigationFeedback()
  ingest(observer, [gps(), frame(0x2F8, 7 | (65535 << 8) | (65535 << 24))])
  value = observer.snapshot(T)
  assert value["gpsStatus"] == "observed"
  assert value["headingDeg"] is None and value["speedValue"] is None


def test_jump_not_accepted_as_new_anchor_and_recovers_on_good_sample():
  observer = OemNavigationFeedback()
  ingest(observer, [gps(), motion()])
  ingest(observer, [gps(lon=121_591314), motion()], T + 1_000_000_000)
  assert observer.snapshot(T + 1_000_000_000)["gpsStatus"] == "jump"
  ingest(observer, [gps(), motion()], T + 2_000_000_000)
  assert observer.snapshot(T + 2_000_000_000)["gpsStatus"] == "observed"


def test_wrong_bus_unknown_ids_delays_future_duplicates_and_reordered_events_do_not_refresh():
  observer = OemNavigationFeedback()
  ingest(observer, [frame(0x04F, 0, src=2), frame(0x25D, 0)])
  assert observer.snapshot(T)["gpsStatus"] == "missing"
  ingest(observer, [gps(), motion()])
  for source, received in [(T, T+1), (T-1, T), (T+1, T), (T+1, T+1_000_000_000)]:
    observer.ingest([frame(0x04F, 0)], now_ns=source, received_ns=received)
  assert observer.snapshot(T+1)["gpsStatus"] == "observed"
  assert observer.snapshot(T+GPS_MAX_AGE_NS)["gpsStatus"] == "stale"


def test_invalid_event_clears_observations_and_pair_requires_both_frames():
  observer = OemNavigationFeedback()
  ingest(observer, [gps()])
  assert observer.snapshot(T)["latitude"] is None
  ingest(observer, [motion()])
  assert observer.snapshot(T)["gpsStatus"] == "observed"
  ingest(observer, [], T+1, valid=False)
  assert observer.snapshot(T+1)["gpsStatus"] == "missing"


def test_all_ones_debug_frames_are_unavailable_and_distance_needs_navigation():
  observer = OemNavigationFeedback()
  ingest(observer, [frame(0x247, "FFFFFFFFFFFFFFFF"), frame(0x24A, "FFFFFFFFFFFFFFFF")])
  assert observer.snapshot(T)["mapAvailable"] is None
  assert observer.snapshot(T)["navAvailable"] is None
  ingest(observer, [frame(0x24A, 12 << 40)], T+1)
  assert observer.snapshot(T+1)["navDistanceM"] is None
  ingest(observer, [frame(0x24A, (12 << 40) | (1 << 39))], T+2)
  assert observer.snapshot(T+2)["navDistanceM"] == 1200


@pytest.mark.parametrize("state", [2, 3, 8, 9, 10, 12, 63])
def test_247_diagnostic_state_uses_distinct_six_bit_layout_and_expires(state):
  observer = OemNavigationFeedback()
  ingest(observer, [frame(0x247, 3 | (state << 50))])
  value = observer.snapshot(T)
  assert value["roadEstimator"] == 3
  assert value["oemLaneChangeState"] == state
  assert value["alcState"] == 0
  assert "leftAllowed" not in value and "rightAllowed" not in value
  expired = observer.snapshot(T + 500_000_000)
  assert expired["roadEstimator"] is None and expired["oemLaneChangeState"] is None
  ingest(observer, [frame(0x247, "FFFFFFFFFFFFFFFF")], T+1_000_000_000)
  assert observer.snapshot(T+1_000_000_000)["oemLaneChangeState"] is None


@pytest.mark.parametrize("raw", ["0805060700", "0805060700000000"])
def test_party_accuracy_retains_real_five_byte_and_eight_byte_payload_without_claiming_units(raw):
  observer = OemNavigationFeedback()
  ingest(observer, [frame(0x32B, raw, src=1), frame(0x32B, raw, src=128)])
  assert observer.snapshot(T)["gpsAccuracyRaw"] is None
  ingest(observer, [frame(0x32B, raw, src=2)])
  snapshot = observer.snapshot(T + 100_000_000)
  assert snapshot["gpsAccuracyRaw"] == raw and snapshot["gpsAccuracyAgeMs"] == 100
  assert snapshot["gpsStatus"] == "missing"  # Does not manufacture a position fix.
  assert observer.snapshot(T + GPS_MAX_AGE_NS)["gpsAccuracyRaw"] is None


@pytest.mark.parametrize("raw", ["08050607", "080506070000", "FFFFFFFFFF", "FFFFFFFFFFFFFFFF"])
def test_bad_accuracy_payload_revokes_raw_diagnostic_only(raw):
  observer = OemNavigationFeedback()
  ingest(observer, [gps(), motion(), frame(0x32B, "0805060700", src=2)])
  ingest(observer, [frame(0x32B, raw, src=2)], T+1)
  assert observer.snapshot(T+1)["gpsAccuracyRaw"] is None
  assert observer.snapshot(T+1)["gpsStatus"] == "observed"


def test_gps_clock_progress_is_not_renewed_by_repeated_frames_or_snapshots():
  observer = OemNavigationFeedback()
  value = 1790093244712
  ingest(observer, [frame(0x324, value), gps(), motion()])
  assert observer.snapshot(T)["gpsTimeMs"] == value
  ingest(observer, [frame(0x324, value)], T+2_000_000_000)
  assert observer.snapshot(T+2_000_000_000)["gpsTimeAgeMs"] == 2000
  expired = observer.snapshot(T+GPS_MAX_AGE_NS)
  assert expired["gpsTimeMs"] is None and expired["gpsTimeAgeMs"] is None
  ingest(observer, [frame(0x324, value+3000)], T+3_000_000_000)
  assert observer.snapshot(T+3_000_000_000)["gpsTimeAgeMs"] == 0
  ingest(observer, [frame(0x324, value)], T+3_000_000_001)
  assert observer.snapshot(T+3_000_000_001)["gpsTimeMs"] is None
  ingest(observer, [], T+4_000_000_000, valid=False)
  assert observer.snapshot(T+4_000_000_000)["gpsTimeMs"] is None


@pytest.mark.parametrize("bad", [frame(0x324, 0), frame(0x324, "FFFFFFFFFFFFFFFF"),
                                 frame(0x324, "01020304"), frame(0x324, 253402300800000)])
def test_invalid_clock_never_invalidates_existing_position_or_invents_measurement_time(bad):
  observer = OemNavigationFeedback()
  ingest(observer, [gps(), motion(), bad])
  assert observer.snapshot(T)["gpsTimeMs"] is None
  assert observer.snapshot(T)["gpsStatus"] == "observed"
