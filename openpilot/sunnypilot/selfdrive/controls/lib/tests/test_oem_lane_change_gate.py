from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.navassist.tests.test_oem_lane_feedback import state_399, topology as state_239
from openpilot.sunnypilot.selfdrive.controls.lib.oem_lane_change_gate import OemLaneChangeGate, confirmed_crossable_lanes, lane_change_start_permissions
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_lane_intent_desire import helper, car_state, intent, LaneChangeState


def event(state, stamp, **kwargs):
  return SimpleNamespace(valid=True, logMonoTime=stamp, can=[state_399(state, **kwargs)])


NOW_NS = 2_000_000_000


def visual_topology(**updates):
  values = dict(validForControl=True, leftEvidenceValid=True, rightEvidenceValid=True,
                leftEgoSideMarking='dashed', rightEgoSideMarking='dashed',
                leftNeighborExists=True, rightNeighborExists=True,
                leftCrossingAllowed=True, rightCrossingAllowed=True,
                publishMonoTime=NOW_NS, modelMonoTime=NOW_NS, imageMonoTime=NOW_NS)
  values.update(updates)
  return SimpleNamespace(**values)


@pytest.mark.parametrize('state', range(32))
@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('trigger', ['manual', 'torque', 'navigation'])
def test_all_399_states_gate_oem_only_start_path(state, side, trigger):
  gate = OemLaneChangeGate()
  allowed = gate.update([event(state, 1_000_000_000), event(state, 1_050_000_000)], 1_050_000_000)
  dh = helper()
  cs = car_state(**{side+'Blinker': True, 'steeringPressed': trigger == 'torque',
                    'steeringTorque': 1.0 if side == 'left' else -1.0})
  kwargs = dict(left_start_allowed=allowed[0], right_start_allowed=allowed[1])
  if trigger == 'navigation':
    kwargs.update(nav_lane_intent=intent(direction=side), left_crossing_allowed=True, right_crossing_allowed=True)
  for _ in range(5):
    dh.update(cs, True, 1.0, **kwargs)
  expected = state in ((6, 8) if side == 'left' else (7, 8))
  assert (dh.lane_change_state == LaneChangeState.laneChangeStarting) == expected
  if not expected:
    assert dh.lane_change_state == LaneChangeState.preLaneChange


def test_absent_expired_invalid_wrong_bus_and_bad_checksum_fail_closed():
  gate = OemLaneChangeGate()
  assert gate.update([], 1_000_000_000) == (False, False)
  assert gate.update([event(8, 1_000_000_000)], 1_000_000_000) == (False, False)
  assert gate.update([event(8, 1_050_000_000)], 1_050_000_000) == (True, True)
  assert gate.update([], 2_400_000_000) == (False, False)
  for kind in ['invalid', 'bus', 'checksum', 'old', 'future']:
    gate = OemLaneChangeGate()
    e = event(8, 3_000_000_000)
    if kind == 'invalid': e.valid = False
    if kind == 'bus': e.can[0].src = 0
    if kind == 'checksum': e.can[0].dat = b'\x00'*8
    if kind == 'old': e.logMonoTime = 1
    if kind == 'future': e.logMonoTime = 4_000_000_000
    assert gate.update([e, e], 3_000_000_000) == (False, False)


def test_stale_399_does_not_veto_fresh_visual_but_fresh_safety_block_does():
  gate = OemLaneChangeGate()
  topology = visual_topology()
  assert gate.update([], NOW_NS) == (False, False)
  assert gate.safety_blocks == (True, True)
  assert lane_change_start_permissions(topology, healthy=True, now_ns=NOW_NS,
                                      oem_permissions=(False, False), safety_blocks=gate.lane_change_safety_blocks) == (True, True)
  assert gate.update([event(8, NOW_NS + 10, blind_left=1), event(8, NOW_NS + 20, blind_left=1)], NOW_NS + 20) == (False, True)
  topology = visual_topology(publishMonoTime=NOW_NS + 20, modelMonoTime=NOW_NS + 20,
                             imageMonoTime=NOW_NS + 20)
  assert lane_change_start_permissions(topology, healthy=True, now_ns=NOW_NS + 20,
                                      oem_permissions=(False, True), safety_blocks=gate.lane_change_safety_blocks) == (False, True)


def test_negative_frame_revokes_permission_and_blindspot_is_directional():
  gate = OemLaneChangeGate()
  assert gate.update([event(8, 10), event(8, 20)], 20) == (True, True)
  assert gate.update([event(1, 30)], 30) == (False, False)
  assert gate.update([event(8, 40, blind_left=1), event(8, 50, blind_left=1)], 50) == (False, True)


def test_239_does_not_supply_lane_change_neighbor_position():
  gate = OemLaneChangeGate()
  for stamp in (1, 2, 3):
    packet = event(8, stamp)
    packet.can.append(state_239(0x02, stamp))
    gate.update([packet], stamp)
  assert gate.neighbors == (None, None)


def test_permission_loss_does_not_abruptly_reset_an_active_manoeuvre():
  dh = helper()
  cs = car_state(leftBlinker=True)
  for _ in range(3): dh.update(cs, True, 1.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  dh.update(cs, True, 1.0, left_start_allowed=False, right_start_allowed=False)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('trigger', ['manual', 'torque', 'navigation'])
@pytest.mark.parametrize('marking,evidence,healthy,control_valid,expected', [
  ('dashed', True, True, True, True),
  ('doubleDashed', True, True, True, True),
  ('solid', True, True, True, False),
  ('doubleSolid', True, True, True, False),
  ('solidDashed', True, True, True, False),
  ('roadEdge', True, True, True, False),
  ('unknown', True, True, True, False),
  ('dashed', False, True, True, False),
  ('dashed', True, False, True, False),
  ('dashed', True, True, False, False),
])
def test_visual_only_permission_for_every_start_path(side, trigger, marking, evidence, healthy, control_valid, expected):
  topology = visual_topology(validForControl=control_valid, leftEvidenceValid=False, rightEvidenceValid=False,
                             leftEgoSideMarking='unknown', rightEgoSideMarking='unknown')
  setattr(topology, side+'EvidenceValid', evidence)
  setattr(topology, side+'EgoSideMarking', marking)
  left, right = confirmed_crossable_lanes(topology, healthy=healthy, now_ns=NOW_NS)
  assert not (right if side == 'left' else left)
  dh = helper()
  cs = car_state(**{side+'Blinker': True, 'steeringPressed': trigger == 'torque',
                    'steeringTorque': 1.0 if side == 'left' else -1.0})
  kwargs = dict(left_start_allowed=left, right_start_allowed=right)
  if trigger == 'navigation':
    kwargs.update(nav_lane_intent=intent(direction=side), left_crossing_allowed=True, right_crossing_allowed=True)
  for _ in range(5): dh.update(cs, True, 1.0, **kwargs)
  assert (dh.lane_change_state == LaneChangeState.laneChangeStarting) == expected


@pytest.mark.parametrize('field,limit', [('publishMonoTime', 150_000_000),
                                       ('modelMonoTime', 150_000_000), ('imageMonoTime', 500_000_000)])
def test_visual_age_is_revalidated_at_use_time(field, limit):
  topology = visual_topology(**{field: NOW_NS - limit})
  assert confirmed_crossable_lanes(topology, healthy=True, now_ns=NOW_NS) == (True, True)
  # Same cached packet remains "valid" and service may remain alive.
  assert confirmed_crossable_lanes(topology, healthy=True, now_ns=NOW_NS + 1) == (False, False)


@pytest.mark.parametrize('field', ['publishMonoTime', 'modelMonoTime', 'imageMonoTime'])
@pytest.mark.parametrize('stamp', [None, 0, -1, NOW_NS + 1])
def test_missing_or_future_visual_time_fails_closed(field, stamp):
  topology = visual_topology()
  if stamp is None:
    delattr(topology, field)
  else:
    setattr(topology, field, stamp)
  assert confirmed_crossable_lanes(topology, healthy=True, now_ns=NOW_NS) == (False, False)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('trigger', ['manual', 'torque', 'navigation'])
@pytest.mark.parametrize('reason', ['no_neighbor', 'publisher_denied', 'stale_model', 'stale_image', 'stale_publication'])
def test_oem_authorizes_when_visual_source_is_unavailable(side, trigger, reason):
  topology = visual_topology()
  if reason == 'no_neighbor':
    setattr(topology, side+'NeighborExists', False)
  elif reason == 'publisher_denied':
    setattr(topology, side+'CrossingAllowed', False)
  else:
    field, age = {'stale_model': ('modelMonoTime', 400_000_000),
                  'stale_image': ('imageMonoTime', 500_000_001),
                  'stale_publication': ('publishMonoTime', 150_000_001)}[reason]
    setattr(topology, field, NOW_NS - age)
  left, right = confirmed_crossable_lanes(topology, healthy=True, now_ns=NOW_NS)
  gate = OemLaneChangeGate()
  oem_left, oem_right = gate.update([event(8, NOW_NS-50_000_000), event(8, NOW_NS)], NOW_NS)
  assert oem_left and oem_right
  if reason in ('no_neighbor', 'publisher_denied'):
    assert right if side == 'left' else left
  dh = helper()
  cs = car_state(**{side+'Blinker': True, 'steeringPressed': trigger == 'torque',
                    'steeringTorque': 1.0 if side == 'left' else -1.0})
  left, right = lane_change_start_permissions(topology, healthy=True, now_ns=NOW_NS,
                                               oem_permissions=(oem_left, oem_right))
  kwargs = dict(left_start_allowed=left, right_start_allowed=right)
  if trigger == 'navigation':
    kwargs.update(nav_lane_intent=intent(direction=side), left_crossing_allowed=True, right_crossing_allowed=True)
  for _ in range(5): dh.update(cs, True, 1.0, **kwargs)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_fresh_visual_evidence_and_neighbor_recovery_reenable_start():
  dh = helper()
  cs = car_state(leftBlinker=True)
  for topology in (visual_topology(leftNeighborExists=False), visual_topology(modelMonoTime=NOW_NS-400_000_000)):
    left, right = confirmed_crossable_lanes(topology, healthy=True, now_ns=NOW_NS)
    for _ in range(5):
      dh.update(cs, True, 1.0, left_line_blocked=not left, right_line_blocked=not right,
                left_start_allowed=True, right_start_allowed=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
  left, right = confirmed_crossable_lanes(visual_topology(), healthy=True, now_ns=NOW_NS)
  dh.update(cs, True, 1.0, left_line_blocked=not left, right_line_blocked=not right,
            left_start_allowed=True, right_start_allowed=True)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('gate', ['visual', 'oem'])
def test_permission_recovery_does_not_add_blindspot_delay(side, gate):
  dh = helper()
  dh.alc.lane_change_bsm_delay = True
  cs = car_state(**{side+'Blinker': True})
  blocked = {side+'_line_blocked': True} if gate == 'visual' else {side+'_start_allowed': False}
  for _ in range(30):
    dh.update(cs, True, 1.0, **blocked)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
  dh.update(cs, True, 1.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('hazard', ['blindspot', 'edge'])
def test_actual_obstruction_keeps_recovery_delay(side, hazard):
  dh = helper()
  dh.alc.lane_change_bsm_delay = True
  cs = car_state(**{side+'Blinker': True})
  blocked_cs = car_state(**{side+'Blinker': True, side+'Blindspot': True}) if hazard == 'blindspot' else cs
  kwargs = {side+'_edge_detected': True} if hazard == 'edge' else {}
  for _ in range(10):
    dh.update(blocked_cs, True, 1.0, **kwargs)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
  for _ in range(10):
    dh.update(cs, True, 1.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
  for _ in range(15): dh.update(cs, True, 1.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_ready_timer_never_bypasses_permission_or_brake_latch():
  dh = helper()
  dh.alc.lane_change_bsm_delay = True
  cs = car_state(leftBlinker=True, brakePressed=True)
  for _ in range(40):
    dh.update(cs, True, 1.0, left_start_allowed=False)
    assert dh.lane_change_state == LaneChangeState.preLaneChange

  for _ in range(40):
    dh.update(car_state(leftBlinker=True), True, 1.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('trigger', ['manual', 'torque', 'navigation'])
@pytest.mark.parametrize('visual,oem,expected', [(False, False, False), (True, False, True),
                                               (False, True, True), (True, True, True)])
def test_public_permission_truth_table(side, trigger, visual, oem, expected):
  topology = visual_topology()
  if not visual:
    setattr(topology, side+'EgoSideMarking', 'unknown')
    setattr(topology, side+'EvidenceValid', False)
  permissions = (oem, False) if side == 'left' else (False, oem)
  left, right = lane_change_start_permissions(topology, healthy=True, now_ns=NOW_NS, oem_permissions=permissions)
  dh = helper()
  cs = car_state(**{side+'Blinker': True, 'steeringPressed': trigger == 'torque',
                    'steeringTorque': 1.0 if side == 'left' else -1.0})
  kwargs = dict(left_start_allowed=left, right_start_allowed=right)
  if trigger == 'navigation':
    kwargs.update(nav_lane_intent=intent(direction=side), left_crossing_allowed=True, right_crossing_allowed=True)
  for _ in range(5): dh.update(cs, True, 1.0, **kwargs)
  assert (dh.lane_change_state == LaneChangeState.laneChangeStarting) == expected


@pytest.mark.parametrize('marking', ['solid', 'doubleSolid', 'solidDashed', 'roadEdge'])
def test_oem_positive_replaces_visual_paint_but_not_road_edge(marking):
  topology = visual_topology(leftEgoSideMarking=marking)
  assert lane_change_start_permissions(topology, healthy=True, now_ns=NOW_NS,
                                      oem_permissions=(True, True)) == (marking != 'roadEdge', True)
