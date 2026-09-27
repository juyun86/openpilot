from dataclasses import replace

import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_lane_intent_desire import helper, car_state
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_oem_lane_change_gate import event, visual_topology, NOW_NS
from openpilot.sunnypilot.selfdrive.controls.lib.oem_lane_change_gate import OemLaneChangeGate, lane_change_start_permissions
from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_blocker import LaneChangeBoundaryBlocker
from openpilot.sunnypilot.navassist.lane_intent import (
  NavLaneIntentCoordinator, NavLanePlan, LaneTopologyInput, LaneVehicleInput, LaneIntentDirection, ObservedLaneChangeState,
)
from openpilot.sunnypilot.navassist.nav_lane_intentd import oem_crossing_allowed


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('allowed', [False, True])
@pytest.mark.parametrize('hazards', [False, True])
def test_low_speed_consumes_common_permission_and_hazards(side, allowed, hazards):
  dh = helper()
  dh.lane_turn_controller.enabled = True
  dh.lane_turn_controller.lane_turn_value = 8.5
  dh.update(car_state(vEgo=5, **{side+'Blinker': True}), True, 1,
            **{side+'_start_allowed': allowed, side+'_safety_blocked': hazards})
  expected = (log.Desire.turnLeft if side == 'left' else log.Desire.turnRight) if allowed and not hazards else log.Desire.none
  assert dh.desire == expected


def test_asymmetric_permission_cannot_turn_hazard_lamps_into_one_turn():
  dh = helper(); dh.lane_turn_controller.enabled = True; dh.lane_turn_controller.lane_turn_value = 8.5
  dh.update(car_state(vEgo=5, leftBlinker=True, rightBlinker=True), True, 1,
            left_start_allowed=True, right_start_allowed=False)
  assert dh.desire == log.Desire.none


@pytest.mark.parametrize('state,left,right', [
  (1, False, False), (5, False, False), (8, False, False),
  (11, True, False), (13, True, False), (15, True, False), (22, True, False), (23, True, False), (26, True, False),
  (12, False, True), (14, False, True), (16, False, True), (24, False, True), (25, False, True), (27, False, True),
  (21, True, True),
])
def test_oem_hazards_veto_visual_but_unavailability_keeps_or(state, left, right):
  gate = OemLaneChangeGate()
  permissions = gate.update([event(state, NOW_NS-50_000_000), event(state, NOW_NS)], NOW_NS)
  assert gate.safety_blocks == (left, right)
  assert lane_change_start_permissions(visual_topology(), healthy=True, now_ns=NOW_NS,
                                      oem_permissions=permissions, safety_blocks=gate.safety_blocks) == (not left, not right)
  snapshot = gate.feedback.snapshot(NOW_NS, include_topology_details=True)
  for side, blocked in [('left', left), ('right', right)]:
    assert oem_crossing_allowed(visual_topology(), snapshot, side=side, visual_healthy=True, now_ns=NOW_NS) == (not blocked)
    dh = helper()
    for _ in range(5):
      dh.update(car_state(**{side+'Blinker': True}), True, 1,
                **{side+'_start_allowed': True, side+'_safety_blocked': blocked})
    assert (dh.lane_change_state == log.LaneChangeState.laneChangeStarting) == (not blocked)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('raw', [1, 2, 3])
def test_raw_blindspot_warning_or_unavailable_blocks_visual_override(side, raw):
  gate = OemLaneChangeGate(); kwargs = {'blind_'+side: raw}
  permissions = gate.update([event(8, NOW_NS-50_000_000, **kwargs), event(8, NOW_NS, **kwargs)], NOW_NS)
  result = lane_change_start_permissions(visual_topology(), healthy=True, now_ns=NOW_NS,
                                         oem_permissions=permissions, safety_blocks=gate.safety_blocks)
  assert result == ((False, True) if side == 'left' else (True, False))


def test_missing_expired_or_corrupt_399_only_releases_lane_change_veto():
  gate = OemLaneChangeGate()
  gate.update([], NOW_NS)
  assert gate.safety_blocks == (True, True)
  assert gate.lane_change_safety_blocks == (False, False)
  assert not gate.feedback.snapshot(NOW_NS)['permissionValid']
  gate.update([event(8, NOW_NS-50_000_000), event(8, NOW_NS)], NOW_NS)
  assert gate.safety_blocks == (False, False)
  gate.update([], NOW_NS+1_250_000_001)
  assert gate.safety_blocks == (True, True)
  assert gate.lane_change_safety_blocks == (False, False)
  assert not gate.feedback.snapshot(NOW_NS+1_250_000_001)['permissionValid']
  bad = event(8, NOW_NS+2_000_000_000); bad.can[0].dat = b'\x00'*8
  gate.update([bad], bad.logMonoTime)
  assert gate.safety_blocks == (True, True)
  assert gate.lane_change_safety_blocks == (False, False)
  assert not gate.feedback.snapshot(bad.logMonoTime)['permissionValid']


@pytest.mark.parametrize('speed', [5, 15])
def test_solid_memory_survives_unknown_with_oem_allow_and_speed_transition(speed):
  dh = helper(); dh.lane_turn_controller.enabled = True; dh.lane_turn_controller.lane_turn_value = 8.5
  blocker = LaneChangeBoundaryBlocker()
  blocked = blocker.update(visual_topology(leftEgoSideMarking='doubleSolid'), healthy=True)
  dh.update(car_state(vEgo=15, leftBlinker=True), True, 1, left_line_blocked=blocked[0], left_start_allowed=False)
  for _ in range(40):
    blocked = blocker.update(visual_topology(leftEgoSideMarking='unknown', leftEvidenceValid=False), healthy=True)
    dh.update(car_state(vEgo=speed, leftBlinker=True), True, 1, left_line_blocked=blocked[0], left_start_allowed=True)
    assert dh.desire == log.Desire.none


@pytest.mark.parametrize('side', ['left', 'right'])
def test_oem_hazard_cancels_active_change_without_success_direction(side):
  dh = helper(); cs = car_state(**{side+'Blinker': True})
  for _ in range(4): dh.update(cs, True, 1)
  assert dh.lane_change_state == log.LaneChangeState.laneChangeStarting
  dh.update(cs, True, 1, **{side+'_safety_blocked': True})
  assert dh.lane_change_state == log.LaneChangeState.laneChangeFinishing
  assert dh.lane_change_direction == log.LaneChangeDirection.none
  assert dh.desire == log.Desire.none


def test_cancelled_relative_change_does_not_increment_completion_budget():
  c = NavLaneIntentCoordinator()
  plan = NavLanePlan(True, 'test', 1, 1, 3, (0,), heuristic=True, edge_direction=LaneIntentDirection.left)
  topology = LaneTopologyInput(True, 3, 1, True, True, True, False)
  vehicle = LaneVehicleInput(True, 15, left_blinker=True, lane_change_direction=LaneIntentDirection.left)
  for i in range(25): c.update(plan, topology, vehicle, now_ns=1_000_000_000+i*100_000_000)
  c.update(plan, topology, replace(vehicle, lane_change_state=ObservedLaneChangeState.starting), now_ns=4_000_000_000)
  cancelled = replace(vehicle, lane_change_state=ObservedLaneChangeState.finishing, lane_change_direction=LaneIntentDirection.none)
  out = c.update(plan, topology, cancelled, now_ns=4_100_000_000)
  assert out.reason == 'laneChangeCancelled' and not out.signal_requested
  for i in range(30):
    out = c.update(plan, topology, vehicle, now_ns=4_200_000_000+i*100_000_000)
    assert not out.signal_requested
    assert 'Complete' not in out.reason and 'Observed' not in out.reason
  assert c._relative_consistency._completed_changes == 0
  plan = replace(plan, maneuver_event_id=2)
  for i in range(25):
    out = c.update(plan, topology, vehicle, now_ns=8_000_000_000+i*100_000_000)
  assert out.signal_requested
