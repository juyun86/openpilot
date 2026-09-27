from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode
from openpilot.sunnypilot.navassist.lane_intent import (
  LaneIntentDirection as Direction, ObservedLaneChangeState as State,
  NavLaneIntentCoordinator, NavLaneIntent,
)
from openpilot.sunnypilot.navassist.efficiency_lane import EfficiencyLaneSelector
from openpilot.sunnypilot.navassist.tests.test_efficiency_lane import inputs


def helper():
  dh = DesireHelper()
  dh.alc.update_params = lambda: None
  dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGELESS
  dh.alc.lane_change_bsm_delay = False
  dh.lane_turn_controller.update_params = lambda: None
  dh.lane_turn_controller.enabled = False
  return dh


def physical(side):
  return NS(vEgo=25., leftBlinker=side == 'left', rightBlinker=side == 'right',
            leftBlindspot=False, rightBlindspot=False, brakePressed=False,
            gasPressed=False, steeringPressed=False, steeringTorque=0.)


def request(side, request_id=1):
  return NS(valid=True, signalRequested=True, spLaneChangeReady=True,
            direction=side, targetLaneIndex=0, sessionId='route', maneuverEventId=1,
            requestId=request_id)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_cp_normal_finish_keeps_direction_and_consumes_request(side):
  dh = helper(); cs = physical(side); req = request(side)
  kwargs = {side + '_crossing_allowed': True, 'nav_lane_intent': req}
  for _ in range(4): dh.update(cs, True, 1., **kwargs)
  assert int(dh.lane_change_state) == State.starting
  for _ in range(12):
    dh.update(cs, True, 0., **kwargs)
    if int(dh.lane_change_state) == State.finishing: break
  assert int(dh.lane_change_state) == State.finishing
  assert int(dh.lane_change_direction) == getattr(Direction, side)
  assert dh.desire == log.Desire.none
  for _ in range(19): dh.update(cs, True, 0., **kwargs)
  assert int(dh.lane_change_state) == State.finishing
  for _ in range(5): dh.update(cs, True, 0., **kwargs)
  assert int(dh.lane_change_state) == State.pre
  for _ in range(80):
    dh.update(cs, True, 0., **kwargs)
    assert int(dh.lane_change_state) == State.pre
  # A fresh explicit request can start, but must pass the usual readiness gate.
  req.requestId += 1; req.spLaneChangeReady = False
  for _ in range(5): dh.update(cs, True, 1., **kwargs)
  assert int(dh.lane_change_state) == State.pre
  req.spLaneChangeReady = True
  for _ in range(5): dh.update(cs, True, 1., **kwargs)
  assert int(dh.lane_change_state) == State.pre  # Existing ALC still needs lamp release.
  req.valid = req.signalRequested = False
  dh.update(physical('none'), True, 1., **kwargs)
  req.valid = req.signalRequested = True
  for _ in range(5): dh.update(cs, True, 1., **kwargs)
  assert int(dh.lane_change_state) == State.starting


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('fault', ['blindspot', 'line', 'oem', 'brake', 'gas', 'steering', 'lamp', 'request', 'lat', 'off'])
def test_normal_finishing_interruption_never_returns_success_pre(side, fault):
  dh = helper(); cs = physical(side); req = request(side)
  kwargs = {side + '_crossing_allowed': True, 'nav_lane_intent': req}
  for _ in range(4): dh.update(cs, True, 1., **kwargs)
  for _ in range(11): dh.update(cs, True, 0., **kwargs)
  assert int(dh.lane_change_state) == State.finishing
  if fault == 'blindspot': setattr(cs, side + 'Blindspot', True)
  if fault in ('line', 'oem'): kwargs[side + ('_line_blocked' if fault == 'line' else '_safety_blocked')] = True
  if fault in ('brake', 'gas', 'steering'): setattr(cs, fault + 'Pressed', True)
  if fault == 'lamp': setattr(cs, side + 'Blinker', False)
  if fault == 'request': req.requestId += 1
  if fault == 'off': dh.alc.lane_change_set_timer = AutoLaneChangeMode.OFF
  dh.update(cs, fault != 'lat', 0., **kwargs)
  if fault == 'gas':
    assert int(dh.lane_change_direction) == getattr(Direction, side)
    assert int(dh.lane_change_state) == State.finishing
    return
  assert int(dh.lane_change_direction) == Direction.none
  assert int(dh.lane_change_state) in (State.off, State.finishing)


class Loop:
  """Real selector/coordinator/DesireHelper, with physical lamp feedback and 20 Hz model frames."""
  def __init__(self, side):
    self.data = inputs(); self.side = side
    if side == 'right':
      self.data['oem']['leftAllowed'] = False
      self.data['topology'] = replace(self.data['topology'], left_crossing_allowed=False)
    self.selector = EfficiencyLaneSelector(); self.coordinator = NavLaneIntentCoordinator()
    self.dh = helper(); self.intent = NavLaneIntent(); self.now = 1_000_000_000
    self.results = []; self.selected_count = 0

  def tick(self, *, fault=None):
    self.now += 50_000_000
    if fault == 'gap': self.now += 300_000_000
    cs = physical(self.side)
    cs.leftBlinker = self.intent.signal_requested and self.side == 'left'
    cs.rightBlinker = self.intent.signal_requested and self.side == 'right'
    req = request(self.side, self.intent.request_id)
    req.valid = req.signalRequested = self.intent.signal_requested
    req.spLaneChangeReady = self.intent.lane_change_ready
    if self.selector.active:
      req.maneuverEventId = self.selector.active.maneuver_event_id
    self.dh.update(cs, True, 0., nav_lane_intent=req,
                   **{self.side + '_crossing_allowed': True})
    v = replace(self.data['vehicle'], left_blinker=cs.leftBlinker, right_blinker=cs.rightBlinker,
                lane_change_state=State(int(self.dh.lane_change_state)),
                lane_change_direction=Direction(int(self.dh.lane_change_direction)), model_mono_time_ns=self.now)
    if fault == 'off': v = replace(v, lane_change_state=State.off, lane_change_direction=Direction.none)
    if fault == 'cancel': v = replace(v, lane_change_state=State.finishing, lane_change_direction=Direction.none)
    if fault == 'driver': v = replace(v, steering_pressed=True)
    if fault == 'oldProducer':
      v = replace(v, lane_change_state=State.pre, lane_change_direction=getattr(Direction, self.side))
    self.data['vehicle'] = v
    plan, selected = self.selector.select(**self.data, now_ns=self.now)
    self.intent = self.coordinator.update(plan, self.data['topology'], v, now_ns=self.now)
    if selected: self.selector.observe(self.intent, self.now)
    self.results.append(self.intent.reason)
    return self.intent


@pytest.mark.parametrize('side', ['left', 'right'])
def test_real_loop_completes_cools_down_and_evaluates_again(side):
  loop = Loop(side)
  for _ in range(180):
    loop.tick()
    if loop.selector.reason == 'efficiencyComplete': break
  assert loop.selector.reason == 'efficiencyComplete'
  assert 'laneChangeObserved' in loop.results
  assert loop.selector.unconfirmed_session is None
  assert loop.coordinator._relative_consistency._completed_changes == 1
  until = loop.selector.cooldown_until
  while loop.now < until - 50_000_000:
    loop.tick()
    assert loop.selector.sequence == 1
    assert not loop.intent.signal_requested
  # Re-evaluate fresh benefit rather than returning to the previous lane.
  for _ in range(50): loop.tick()
  assert loop.selector.sequence == 2
  assert loop.selector.active.edge_direction == getattr(Direction, side)
  for _ in range(100):
    loop.tick()
    if loop.selector.reason == 'efficiencyComplete': break
  assert loop.selector.reason == 'efficiencyComplete'
  assert loop.results.count('laneChangeObserved') == 2
  assert loop.selector.unconfirmed_session is None


@pytest.mark.parametrize('side', ['left', 'right'])
def test_real_loop_completes_after_normal_topology_gap(side):
  loop = Loop(side)
  for _ in range(100):
    loop.tick()
    if int(loop.dh.lane_change_state) == State.starting: break
  assert loop.coordinator._phase == 'changing'
  original_topology = loop.data['topology']
  loop.data['topology'] = replace(original_topology, valid_for_control=False,
                                  visible_lane_count=0, ego_lane_index=-1)
  for _ in range(54):
    loop.tick()
  assert 'laneChangeObserved' in loop.results
  assert loop.selector.unconfirmed_session is None
  assert loop.selector.sequence == 1
  loop.data['topology'] = original_topology


def test_started_change_completes_through_entry_only_radar_speed_and_road_class_loss():
  loop = Loop('left')
  for _ in range(100):
    loop.tick()
    if int(loop.dh.lane_change_state) == State.starting: break
  assert loop.coordinator._phase == 'changing'
  loop.data['healthy'] = False  # Radar entry evidence drops; model/control remain healthy.
  loop.data['execution_healthy'] = True
  loop.data['nav'].roadClass = 7
  loop.data['vehicle'] = replace(loop.data['vehicle'], speed_mps=14.)
  for _ in range(100):
    loop.tick()
    if 'laneChangeObserved' in loop.results: break
  assert 'laneChangeObserved' in loop.results
  assert loop.selector.unconfirmed_session is None


@pytest.mark.parametrize('fault', ['off', 'cancel', 'driver', 'oldProducer'])
def test_bad_completion_cannot_restart_with_a_new_efficiency_id(fault):
  loop = Loop('left')
  for _ in range(100):
    loop.tick()
    if int(loop.dh.lane_change_state) == State.starting: break
  assert loop.coordinator._phase == 'changing'
  for _ in range(340): loop.tick(fault=fault)
  assert 'laneChangeObserved' not in loop.results
  assert loop.selector.sequence == 1
  assert loop.selector.unconfirmed_session == (None if fault == 'cancel' else 'route')
  assert not loop.intent.signal_requested


def test_one_frame_reset_between_finishing_and_pre_cannot_be_hidden_by_recovery():
  loop = Loop('left')
  for _ in range(150):
    loop.tick()
    if loop.coordinator._normal_finish_seen: break
  assert loop.coordinator._normal_finish_seen
  loop.tick(fault='off')
  for _ in range(300): loop.tick()
  assert 'laneChangeObserved' not in loop.results
  assert loop.selector.sequence == 1
  assert loop.selector.unconfirmed_session == 'route'
