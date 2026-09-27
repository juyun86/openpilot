import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_lane_intent_desire import helper, car_state, intent, LaneChangeState


def turn_helper():
  dh = helper()
  dh.lane_turn_controller.enabled = True
  dh.lane_turn_controller.lane_turn_value = 8.5
  return dh


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('navigation', [False, True])
@pytest.mark.parametrize('hazard', ['line', 'edge', 'blindspot'])
def test_low_speed_turn_cannot_bypass_known_boundary_or_blindspot(side, navigation, hazard):
  dh = turn_helper()
  values = {side+'Blinker': True, 'vEgo': 5.0}
  kwargs = {}
  if hazard == 'blindspot': values[side+'Blindspot'] = True
  else: kwargs[side+('_line_blocked' if hazard == 'line' else '_edge_detected')] = True
  if navigation: kwargs['nav_lane_intent'] = intent(direction=side, target=-1)
  for _ in range(10):
    dh.update(car_state(**values), True, 1.0, **kwargs)
    assert dh.desire == log.Desire.none
    assert dh.lane_change_state == LaneChangeState.off


@pytest.mark.parametrize('side', ['left', 'right'])
def test_unblocked_low_speed_turn_is_preserved_and_requires_lateral_active(side):
  dh = turn_helper()
  cs = car_state(vEgo=5.0, **{side+'Blinker': True})
  dh.update(cs, False, 1.0)
  assert dh.desire == log.Desire.none
  dh.update(cs, True, 1.0)
  assert dh.desire == (log.Desire.turnLeft if side == 'left' else log.Desire.turnRight)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('hazard', ['line', 'edge', 'blindspot', 'low_speed'])
def test_active_change_exits_without_turn_fallback_or_held_lamp_restart(side, hazard):
  dh = turn_helper()
  cs = car_state(**{side+'Blinker': True})
  for _ in range(4): dh.update(cs, True, 1.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  values = {side+'Blinker': True}
  kwargs = {}
  if hazard == 'low_speed': values['vEgo'] = 5.0
  elif hazard == 'blindspot': values[side+'Blindspot'] = True
  else: kwargs[side+('_line_blocked' if hazard == 'line' else '_edge_detected')] = True
  dh.update(car_state(**values), True, 1.0, **kwargs)
  assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
  assert dh.desire == log.Desire.none
  for _ in range(60):
    dh.update(car_state(vEgo=5.0, **{side+'Blinker': True}), True, 1.0)
    assert dh.desire == log.Desire.none
  assert dh.lane_change_state == LaneChangeState.off
  # Raising speed and applying confirmation torque cannot bypass the latch.
  for _ in range(10):
    dh.update(car_state(**{side+'Blinker': True, 'steeringPressed': True,
                           'steeringTorque': 1.0 if side == 'left' else -1.0}), True, 1.0)
    assert dh.lane_change_state == LaneChangeState.off
  dh.update(car_state(), True, 1.0)
  for _ in range(4): dh.update(cs, True, 1.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_opposite_boundary_does_not_cancel_active_change():
  dh = helper()
  cs = car_state(leftBlinker=True)
  for _ in range(4): dh.update(cs, True, 1.0)
  dh.update(cs, True, 1.0, right_line_blocked=True)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('already_turning', [False, True])
def test_low_speed_solid_veto_survives_vision_dropout_until_lamp_off(side, already_turning):
  dh = turn_helper()
  cs = car_state(vEgo=5.0, **{side+'Blinker': True})
  if already_turning:
    dh.update(cs, True, 1.0)
    assert dh.desire == (log.Desire.turnLeft if side == 'left' else log.Desire.turnRight)
  dh.update(cs, True, 1.0, **{side+'_line_blocked': True})
  assert dh.desire == log.Desire.none
  # Replay the observed solid -> unknown transition without another lamp edge.
  for _ in range(80):
    dh.update(cs, True, 1.0)
    assert dh.desire == log.Desire.none
  dh.update(car_state(vEgo=5.0), True, 1.0)
  dh.update(cs, True, 1.0)
  assert dh.desire == (log.Desire.turnLeft if side == 'left' else log.Desire.turnRight)
