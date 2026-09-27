from dataclasses import dataclass

import pytest

from openpilot.sunnypilot.navassist.oem_lane_feedback import OemLaneFeedback


@dataclass
class Frame:
  address: int
  dat: bytes
  src: int = 1


def frame(address: int, payload: list[int]) -> Frame:
  data = bytearray(payload + [0] * (7 - len(payload)))
  data.append(((address & 0xFF) + (address >> 8) + sum(data)) & 0xFF)
  return Frame(address, bytes(data))


def topology(first_byte: int, counter: int, *, view_range: int = 50, usage: int = 0x0A,
             width_nibble: int = 5) -> Frame:
  first = (first_byte & 0x03) | ((width_nibble & 0x0F) << 4)
  return Frame(0x239, bytes([first, view_range, 100, 125, 125, 125, usage, (counter & 0x0F) << 4]))


def state_399(state: int, blind_left: int = 0, blind_right: int = 0) -> Frame:
  raw = (state << 46) | (blind_left << 4) | (blind_right << 6)
  return frame(0x399, list(raw.to_bytes(7, "little")))


def test_topology_requires_three_stable_samples_and_expires():
  decoder = OemLaneFeedback()
  for now_ns in (1, 2):
    decoder.ingest([topology(0x02, now_ns)], now_ns)
  assert decoder.snapshot(2)["position"] == "unknown"
  decoder.ingest([topology(0x02, 3)], 3)
  state = decoder.snapshot(3, include_topology_details=True)
  assert state["position"] == "leftmost"
  assert state["rightLaneEvidenceValid"] is True
  assert state["laneWidthM"] == 3.5625
  assert state["viewRangeM"] == 50
  assert decoder.snapshot(400_000_004)["positionValid"] is False


def test_topology_counter_quality_and_view_range_fail_closed_then_recover():
  decoder = OemLaneFeedback()
  decoder.ingest([topology(0x03, 1), topology(0x03, 2), topology(0x03, 3)], 3)
  assert decoder.snapshot(3)["position"] == "middle"

  decoder.ingest([topology(0x03, 3)], 4)  # duplicate counter
  assert decoder.snapshot(4)["positionValid"] is False
  for counter, stamp in ((4, 5), (5, 6), (6, 7)):
    decoder.ingest([topology(0x03, counter)], stamp)
  assert decoder.snapshot(7)["position"] == "middle"

  decoder.ingest([topology(0x03, 7, view_range=10)], 8)
  assert decoder.snapshot(8)["positionValid"] is False


def test_unusable_line_quality_does_not_confirm_that_neighbor_for_control():
  decoder = OemLaneFeedback()
  # left AVAILABLE, right BLACKLISTED
  for counter in (1, 2, 3):
    decoder.ingest([topology(0x03, counter, usage=0x0D)], counter)
  state = decoder.snapshot(3, include_topology_details=True)
  assert state["positionValid"] is True
  assert state["leftLaneEvidenceValid"] is True
  assert state["rightLaneEvidenceValid"] is False


def test_app_snapshot_keeps_existing_vehicle_lane_schema():
  decoder = OemLaneFeedback()
  for counter in (1, 2, 3):
    decoder.ingest([topology(0x03, counter)], counter)

  app_state = decoder.snapshot(3)
  internal_state = decoder.snapshot(3, include_topology_details=True)
  assert "topologyCounter" not in app_state
  assert "laneWidthM" not in app_state
  assert "leftLaneEvidenceValid" not in app_state
  assert internal_state["topologyCounter"] == 3
  assert internal_state["leftLaneEvidenceValid"] is True


def test_lane_change_requires_two_samples_and_blocks_immediately():
  decoder = OemLaneFeedback()
  decoder.ingest([state_399(8)], 1)
  assert decoder.snapshot(1)["leftAllowed"] is False
  decoder.ingest([state_399(8)], 2)
  assert decoder.snapshot(2)["leftAllowed"] is True
  assert decoder.snapshot(2)["rightAllowed"] is True
  decoder.ingest([state_399(8, blind_left=1)], 3)
  assert decoder.snapshot(3)["leftAllowed"] is False
  assert decoder.snapshot(3)["rightAllowed"] is True


def test_bad_checksum_and_wrong_bus_never_enable_change():
  decoder = OemLaneFeedback()
  valid = state_399(6)
  decoder.ingest([Frame(valid.address, valid.dat, src=0), valid], 1)
  bad = Frame(valid.address, valid.dat[:-1] + bytes([valid.dat[-1] ^ 0xFF]))
  decoder.ingest([bad], 2)
  assert decoder.snapshot(2)["leftAllowed"] is False


def test_vehicle_blindspot_radar_and_lane_change_state_are_bounded():
  decoder = OemLaneFeedback()
  decoder.update_vehicle(
    now_ns=1, vehicle_valid=True, radar_valid=True, blindspot_valid=True,
    left_blindspot=True, right_blindspot=False, ego_speed_mps=20.0, lateral_active=True,
    brake_pressed=False, gas_pressed=False, lane_change_state=2, lane_change_direction=1,
    lead_present=True, lead_distance_m=32.0, lead_speed_mps=15.0, lead_relative_speed_mps=-5.0,
  )
  state = decoder.snapshot(2)
  assert state["leftBlindspot"] is True
  assert state["leadDistanceM"] == 32.0
  assert state["egoSpeedKph"] == 72.0
  assert state["laneChangeState"] == 2
  assert decoder.snapshot(500_000_002)["vehicleStateValid"] is False


def confirmed_topology():
  decoder = OemLaneFeedback()
  for counter in (1, 2, 3):
    decoder.ingest([topology(2, counter)], counter)
  return decoder


def test_position_transition_never_exposes_previous_position():
  decoder = confirmed_topology()
  for counter in (4, 5, 6):
    decoder.ingest([topology(1, counter)], counter)
    state = decoder.snapshot(counter)
    assert state["position"] == ("rightmost" if counter == 6 else "unknown")
    assert state["positionValid"] == (counter == 6)


@pytest.mark.parametrize("observe_expiry", [False, True])
def test_topology_gap_restarts_confirmation_even_without_snapshot(observe_expiry):
  decoder = confirmed_topology()
  if observe_expiry:
    assert not decoder.snapshot(500_000_000)["positionValid"]
  for counter in (4, 5, 6):
    stamp = 500_000_000 + counter
    decoder.ingest([topology(2, counter)], stamp)
    assert decoder.snapshot(stamp)["positionValid"] == (counter == 6)


@pytest.mark.parametrize("size", [0, 7, 9])
def test_bad_topology_dlc_invalidates_and_requires_three_new_frames(size):
  decoder = confirmed_topology()
  decoder.ingest([Frame(0x239, bytes(size))], 4)
  assert not decoder.snapshot(4)["positionValid"]
  for counter in (4, 5, 6):
    decoder.ingest([topology(2, counter)], counter + 1)
    assert decoder.snapshot(counter + 1)["positionValid"] == (counter == 6)


@pytest.mark.parametrize("usage,invalid_key,healthy_key", [
  (0x08, "leftLaneEvidenceValid", "rightLaneEvidenceValid"),
  (0x02, "rightLaneEvidenceValid", "leftLaneEvidenceValid"),
])
def test_line_evidence_recovers_independently_after_three_frames(usage, invalid_key, healthy_key):
  decoder = OemLaneFeedback()
  for counter in (1, 2, 3):
    decoder.ingest([topology(3, counter)], counter)
  decoder.ingest([topology(3, 4, usage=usage)], 4)
  state = decoder.snapshot(4, include_topology_details=True)
  assert not state[invalid_key] and state[healthy_key]
  for counter in (5, 6, 7):
    decoder.ingest([topology(3, counter)], counter)
    state = decoder.snapshot(counter, include_topology_details=True)
    assert state[invalid_key] == (counter == 7)
    assert state[healthy_key]


def test_future_snapshot_cannot_revive_previously_confirmed_evidence():
  decoder = confirmed_topology()
  decoder.ingest([state_399(8)], 2)
  decoder.ingest([state_399(8)], 3)
  state = decoder.snapshot(1)
  assert not state["positionValid"] and not state["permissionValid"]
  state = decoder.snapshot(3)
  assert not state["positionValid"] and not state["leftAllowed"]


@pytest.mark.parametrize("address", [0x239, 0x399])
@pytest.mark.parametrize("source_ns", [0, 101, -1, 1])
def test_ingest_rejects_invalid_source_age(address, source_ns):
  decoder = confirmed_topology()
  decoder.ingest([state_399(8)], 2)
  decoder.ingest([state_399(8)], 3)
  received_ns = 2_000_000_000 if source_ns == 1 else 100
  sample = topology(2, 4) if address == 0x239 else state_399(8)
  decoder.ingest([sample], source_ns, received_ns=received_ns)
  state = decoder.snapshot(received_ns)
  assert not state["positionValid" if address == 0x239 else "permissionValid"]


def test_399_gap_requires_two_new_samples():
  decoder = OemLaneFeedback()
  for stamp in (1, 2):
    decoder.ingest([state_399(8)], stamp)
  for stamp in (2_000_000_000, 2_000_000_001):
    decoder.ingest([state_399(8)], stamp)
    assert decoder.snapshot(stamp)["leftAllowed"] == (stamp == 2_000_000_001)


@pytest.mark.parametrize("address", [0x239, 0x399])
def test_replayed_timestamp_cannot_authorize(address):
  decoder = confirmed_topology()
  for stamp in (2, 3):
    decoder.ingest([state_399(8)], stamp)
  sample = topology(2, 4) if address == 0x239 else state_399(8)
  decoder.ingest([sample], 3)
  state = decoder.snapshot(3)
  assert not state["positionValid" if address == 0x239 else "leftAllowed"]


def test_counter_wrap_is_valid_and_wrong_bus_does_not_invalidate():
  decoder = OemLaneFeedback()
  for stamp, counter in enumerate((14, 15, 0), 1):
    decoder.ingest([topology(2, counter)], stamp)
  decoder.ingest([Frame(0x239, b"", src=0)], 4)
  assert decoder.snapshot(4)["positionValid"]


def test_duplicate_399_frames_in_one_packet_do_not_confirm_permission():
  decoder = OemLaneFeedback()
  decoder.ingest([state_399(8), state_399(8)], 1)
  assert not decoder.snapshot(1)["leftAllowed"]


@pytest.mark.parametrize("address,ttl,key", [
  (0x239, 400_000_000, "positionValid"),
  (0x399, 1_250_000_000, "permissionValid"),
])
def test_freshness_boundary_is_inclusive_and_expiry_is_latched(address, ttl, key):
  decoder = confirmed_topology()
  decoder.ingest([state_399(8)], 2)
  decoder.ingest([state_399(8)], 3)
  assert decoder.snapshot(3 + ttl)[key]
  assert not decoder.snapshot(4 + ttl)[key]
  assert not decoder.snapshot(3 + ttl)[key]


@pytest.mark.parametrize("state,left,right", [(6, True, False), (7, False, True), (8, True, True),
                                             (0, False, False), (9, False, False), (10, False, False)])
def test_399_direction_and_in_progress_states(state, left, right):
  decoder = OemLaneFeedback()
  for stamp in (1, 2):
    decoder.ingest([state_399(state)], stamp)
  result = decoder.snapshot(2)
  assert (result["leftAllowed"], result["rightAllowed"]) == (left, right)


@pytest.mark.parametrize("blind", [1, 2, 3])
def test_399_negative_in_same_packet_vetoes_and_requires_reconfirmation(blind):
  decoder = OemLaneFeedback()
  for stamp in (1, 2):
    decoder.ingest([state_399(8)], stamp)
  decoder.ingest([state_399(8), state_399(8, blind_left=blind), state_399(8)], 3)
  result = decoder.snapshot(3, include_topology_details=True)
  assert not result["leftAllowed"] and result["rightAllowed"]
  assert result["leftSafetyBlocked"] and not result["rightSafetyBlocked"]
  for stamp in (4, 5):
    decoder.ingest([state_399(8)], stamp)
    assert decoder.snapshot(stamp)["leftAllowed"] == (stamp == 5)
    assert not decoder.snapshot(stamp, include_topology_details=True)["leftSafetyBlocked"]


def test_rejected_future_frame_does_not_poison_recovery_clock():
  decoder = confirmed_topology()
  decoder.ingest([topology(2, 4)], 1_000_000_000, received_ns=4)
  assert not decoder.snapshot(4)["positionValid"]
  for counter in (5, 6, 7):
    decoder.ingest([topology(2, counter)], counter, received_ns=counter)
    assert decoder.snapshot(counter)["positionValid"] == (counter == 7)


def test_backward_time_invalidates_without_advancing_source_watermark():
  decoder = confirmed_topology()
  decoder.ingest([topology(2, 4)], 2, received_ns=4)
  assert not decoder.snapshot(4)["positionValid"]
  for counter in (5, 6, 7):
    decoder.ingest([topology(2, counter)], counter, received_ns=counter)
    assert decoder.snapshot(counter)["positionValid"] == (counter == 7)


def test_vehicle_future_time_does_not_become_fresh_when_clock_catches_up():
  decoder = OemLaneFeedback()
  decoder.update_vehicle(
    now_ns=10, vehicle_valid=True, radar_valid=True, blindspot_valid=True,
    left_blindspot=False, right_blindspot=False, ego_speed_mps=20.0, lateral_active=True,
    brake_pressed=False, gas_pressed=False, lane_change_state=0, lane_change_direction=0,
    lead_present=False, lead_distance_m=0.0, lead_speed_mps=0.0, lead_relative_speed_mps=0.0,
  )
  for stamp in (9, 10):
    result = decoder.snapshot(stamp)
    assert not result["vehicleStateValid"]
    assert not result["blindspotValid"]
    assert not result["radarValid"]
