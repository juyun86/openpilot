from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeDirection, LaneChangeState
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode


def car_state(**updates):
  values = {
    "vEgo": 15.0,
    "leftBlinker": False,
    "rightBlinker": False,
    "leftBlindspot": False,
    "rightBlindspot": False,
    "brakePressed": False,
    "steeringPressed": False,
    "steeringTorque": 0.0,
  }
  values.update(updates)
  return SimpleNamespace(**values)


def intent(*, direction="left", signal=True, target=0, ready=True):
  return SimpleNamespace(valid=True, signalRequested=signal, direction=direction, targetLaneIndex=target,
                         spLaneChangeReady=ready)


def test_held_navigation_lamp_cannot_start_lane_change_before_ready():
  desire = helper()
  waiting = intent(ready=False)
  desire.update(car_state(), True, 1.0, nav_lane_intent=waiting, left_crossing_allowed=True)
  for _ in range(100):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=waiting, left_crossing_allowed=True)
    assert desire.lane_change_state == LaneChangeState.preLaneChange


def helper():
  desire = DesireHelper()
  desire.alc.update_params = lambda: None
  desire.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGELESS
  # Keep tests independent of the device's persisted post-block delay option.
  desire.alc.lane_change_bsm_delay = False
  desire.lane_turn_controller.update_params = lambda: None
  return desire


@pytest.mark.parametrize("bsm_delay", [False, True])
def test_navigation_gate_recovery_does_not_restart_blindspot_delay(bsm_delay):
  desire = helper()
  desire.alc.lane_change_bsm_delay = bsm_delay
  desire.update(car_state(), True, 1.0, nav_lane_intent=intent())
  for _ in range(10):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=intent())
  desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=intent(), left_crossing_allowed=True)
  assert desire.lane_change_state == LaneChangeState.laneChangeStarting
  for _ in range(25):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=intent(), left_crossing_allowed=True)
  assert desire.lane_change_state == LaneChangeState.laneChangeStarting


def test_navigation_signal_enters_pre_lane_change_but_waits_for_sp_crossing_gate():
  desire = helper()
  desire.update(car_state(), True, 1.0, nav_lane_intent=intent())
  assert desire.lane_change_state == LaneChangeState.preLaneChange

  for _ in range(10):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=intent())
  assert desire.lane_change_state == LaneChangeState.preLaneChange

  desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=intent(),
                left_crossing_allowed=True)
  assert desire.lane_change_state == LaneChangeState.laneChangeStarting


def test_accelerator_does_not_cancel_navigation_change_after_start():
  desire = helper()
  desire.update(car_state(), True, 1.0, nav_lane_intent=intent(), left_crossing_allowed=True)
  for _ in range(4):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=intent(), left_crossing_allowed=True)
  assert desire.lane_change_state == LaneChangeState.laneChangeStarting
  desire.update(car_state(leftBlinker=True, gasPressed=True), True, 1.0,
                nav_lane_intent=intent(), left_crossing_allowed=True)
  assert desire.lane_change_state == LaneChangeState.laneChangeStarting


def test_navigation_target_never_overrides_blindspot_or_conflicting_driver_signal():
  blocked = helper()
  blocked.update(car_state(leftBlindspot=True), True, 1.0, nav_lane_intent=intent(), left_crossing_allowed=True)
  blocked.update(car_state(leftBlinker=True, leftBlindspot=True), True, 1.0,
                 nav_lane_intent=intent(), left_crossing_allowed=True)
  assert blocked.lane_change_state == LaneChangeState.preLaneChange

  conflict = helper()
  conflict.update(car_state(rightBlinker=True), True, 1.0, nav_lane_intent=intent(direction="left"))
  assert conflict.lane_change_state == LaneChangeState.preLaneChange
  assert conflict.lane_change_direction == LaneChangeDirection.right


def test_navigation_target_never_overrides_solid_line_or_road_edge():
  for blockers in (
    {"left_line_blocked": True},
    {"left_edge_detected": True},
  ):
    desire = helper()
    desire.update(car_state(), True, 1.0, nav_lane_intent=intent(), left_crossing_allowed=True, **blockers)
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=intent(),
                  left_crossing_allowed=True, **blockers)

    assert desire.lane_change_state == LaneChangeState.preLaneChange


@pytest.mark.parametrize(("direction", "start_index", "target_index", "blocker"), [
  ("left", 1, 0, {"left_edge_detected": True}),
  ("right", 0, 1, {"right_line_blocked": True}),
])
def test_stable_navigation_target_completes_slow_model_without_visual_self_cancel(
    direction, start_index, target_index, blocker):
  desire = helper()
  lane_target = intent(direction=direction, target=target_index)
  lamp_on = {f"{direction}Blinker": True}
  crossing_allowed = {f"{direction}_crossing_allowed": True}

  desire.update(car_state(), True, 1.0, nav_lane_intent=lane_target,
                observed_lane_index=start_index, observed_lane_count=2, **crossing_allowed)
  for _ in range(3):
    desire.update(car_state(**lamp_on), True, 1.0, nav_lane_intent=lane_target,
                  observed_lane_index=start_index, observed_lane_count=2, **crossing_allowed)
  assert desire.lane_change_state == LaneChangeState.laneChangeStarting

  observed_count = 1 if direction == "left" else 2
  for _ in range(12):
    desire.update(car_state(**lamp_on), True, 1.0, nav_lane_intent=lane_target,
                  observed_lane_index=target_index, observed_lane_count=observed_count,
                  **crossing_allowed, **blocker)

  assert desire.lane_change_state == LaneChangeState.laneChangeFinishing
  assert desire.lane_change_direction == getattr(LaneChangeDirection, direction)


def test_stable_navigation_target_never_overrides_live_safety_block():
  desire = helper()
  lane_target = intent(direction="left", target=0)
  desire.update(car_state(), True, 1.0, nav_lane_intent=lane_target,
                observed_lane_index=1, observed_lane_count=2, left_crossing_allowed=True)
  for _ in range(3):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=lane_target,
                  observed_lane_index=1, observed_lane_count=2, left_crossing_allowed=True)
  assert desire.lane_change_state == LaneChangeState.laneChangeStarting

  for _ in range(12):
    desire.update(car_state(leftBlinker=True), True, 1.0,
                  nav_lane_intent=lane_target, observed_lane_index=0, observed_lane_count=1,
                  left_crossing_allowed=True, left_edge_detected=True)
  assert desire.lane_change_state == LaneChangeState.laneChangeFinishing
  assert desire.lane_change_direction == LaneChangeDirection.left

  desire.update(car_state(leftBlinker=True, leftBlindspot=True), True, 1.0,
                nav_lane_intent=lane_target, observed_lane_index=0, observed_lane_count=1,
                left_crossing_allowed=True, left_edge_detected=True)

  assert desire.lane_change_state == LaneChangeState.laneChangeFinishing
  assert desire.lane_change_direction == LaneChangeDirection.none


def test_navigation_turn_signal_does_not_enter_lane_change_state_machine():
  desire = helper()
  turn_only = intent(target=-1)

  desire.update(car_state(), True, 1.0, nav_lane_intent=turn_only)

  # The Tesla turn-signal controller makes the requested lamp physical. Its
  # feedback returns through carState and must retain the turn-only intent
  # instead of looking like a new driver lane-change request.
  for _ in range(10):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=turn_only)

  assert desire.lane_change_state == LaneChangeState.off
  assert desire.lane_change_direction == LaneChangeDirection.none


def test_navigation_lane_change_lamp_does_not_trigger_lane_turn_desire():
  desire = helper()
  desire.lane_turn_controller.enabled = True
  desire.lane_turn_controller.update_params = lambda: None
  desire.lane_turn_controller.lane_turn_value = 20.0
  lane_target = intent(target=0)

  desire.update(car_state(vEgo=5.0), True, 1.0, nav_lane_intent=lane_target)
  desire.update(car_state(vEgo=5.0, leftBlinker=True), True, 1.0, nav_lane_intent=lane_target,
                left_crossing_allowed=True)

  assert desire.lane_turn_direction == 0
  assert desire.desire == 0


def test_navigation_turn_only_lamp_retains_lane_turn_desire():
  desire = helper()
  desire.lane_turn_controller.enabled = True
  desire.lane_turn_controller.update_params = lambda: None
  desire.lane_turn_controller.lane_turn_value = 20.0
  turn_only = intent(target=-1)

  desire.update(car_state(vEgo=5.0), True, 1.0, nav_lane_intent=turn_only)
  desire.update(car_state(vEgo=5.0, leftBlinker=True), True, 1.0, nav_lane_intent=turn_only)

  assert int(desire.lane_turn_direction) != 0


def test_navigation_lane_target_respects_sp_alc_off():
  desire = helper()
  desire.alc.lane_change_set_timer = AutoLaneChangeMode.OFF
  lane_target = intent(target=0)

  desire.update(car_state(), True, 1.0, nav_lane_intent=lane_target,
                left_crossing_allowed=True)
  for _ in range(10):
    desire.alc.lane_change_set_timer = AutoLaneChangeMode.OFF
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=lane_target,
                  left_crossing_allowed=True)

  assert desire.lane_change_state == LaneChangeState.off
  assert desire.desire == 0


def test_navigation_lane_target_uses_sp_alc_after_physical_lamp_and_dashed_boundary():
  desire = helper()
  lane_target = intent(target=0)

  desire.update(car_state(), True, 1.0, nav_lane_intent=lane_target,
                left_crossing_allowed=True)
  for _ in range(10):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=lane_target,
                  left_crossing_allowed=True)

  assert desire.lane_change_state == LaneChangeState.laneChangeStarting


def test_navigation_physical_lamp_tail_cannot_start_an_extra_lane_change():
  desire = helper()
  lane_target = intent(target=0)

  desire.update(car_state(), True, 1.0, nav_lane_intent=lane_target)
  for _ in range(10):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=lane_target)
  assert desire.lane_change_state == LaneChangeState.preLaneChange

  stopped = intent(signal=False, target=0)
  for _ in range(5):
    desire.update(car_state(leftBlinker=True), True, 1.0, nav_lane_intent=stopped)

  assert desire.lane_change_state == LaneChangeState.off
  assert desire.desire == 0


@pytest.mark.parametrize("direction", ["left", "right"])
def test_completed_lane_change_tail_transfers_to_same_direction_turn(direction):
  desire = helper()
  desire.lane_turn_controller.enabled = True
  desire.lane_turn_controller.lane_turn_value = 19 * 0.44704
  lane_target = intent(direction=direction, target=0)
  lamp_on = {f"{direction}Blinker": True}
  crossing_allowed = {f"{direction}_crossing_allowed": True}
  expected_turn = log.Desire.turnLeft if direction == "left" else log.Desire.turnRight

  desire.update(car_state(), True, 1.0, nav_lane_intent=lane_target, **crossing_allowed)
  for _ in range(3):
    desire.update(car_state(**lamp_on), True, 1.0, nav_lane_intent=lane_target, **crossing_allowed)
  assert desire.lane_change_state == LaneChangeState.laneChangeStarting

  for _ in range(15):
    desire.update(car_state(**lamp_on), True, 0.0, nav_lane_intent=lane_target, **crossing_allowed)
  assert desire.lane_change_state == LaneChangeState.laneChangeFinishing
  assert desire.lane_change_direction == getattr(LaneChangeDirection, direction)
  for _ in range(20):
    desire.update(car_state(**lamp_on), True, 0.0, nav_lane_intent=lane_target, **crossing_allowed)
  assert desire.lane_change_state == LaneChangeState.preLaneChange

  # Cancellation reaches the model before the physical lamp switches off.
  stopped = intent(direction=direction, signal=False)
  desire.update(car_state(**lamp_on), True, 0.0, nav_lane_intent=stopped, **crossing_allowed)
  assert desire.lane_change_state == LaneChangeState.off
  assert desire.desire == log.Desire.none
  desire.update(car_state(vEgo=5.0, **lamp_on), True, 0.0, nav_lane_intent=stopped)
  assert desire.desire == log.Desire.none

  # The approaching turn takes over the still-lit lamp. It must not inherit
  # the old lane-change tail's suppression of the model's turn input.
  turn_only = intent(direction=direction, target=-1)
  desire.update(car_state(vEgo=5.0, **lamp_on), True, 0.0, nav_lane_intent=turn_only)
  assert desire.desire == expected_turn
  assert desire.lane_change_state == LaneChangeState.off
  desire.update(car_state(**lamp_on), True, 0.0, nav_lane_intent=turn_only, **crossing_allowed)
  assert desire.lane_change_state == LaneChangeState.off

  # A later cancellation retains turn-only lamp semantics, and once the lamp
  # clears an ordinary driver signal still follows SP's original behavior.
  desire.update(car_state(vEgo=5.0, **lamp_on), True, 0.0, nav_lane_intent=stopped)
  assert desire.desire == expected_turn
  desire.update(car_state(vEgo=5.0), True, 0.0)
  assert desire.desire == log.Desire.none
  desire.update(car_state(vEgo=5.0, **lamp_on), True, 0.0)
  assert desire.desire == expected_turn
