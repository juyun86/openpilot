from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.turn_entry import TurnEntryGate, TurnIntentClassifier
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_separate_turn_entry import turn_helper, update
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_lane_intent_desire import car_state, intent
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_oem_lane_change_gate import visual_topology, NOW_NS


def cp_model(prob=0.1, near=2.0, far=6.0):
  def line(y):
    return NS(y=[y] * 33)

  # T[10] < 1 s, T[15] > 2 s. Interpolate the two checkpoints exactly.
  from openpilot.selfdrive.modeld.constants import ModelConstants

  edges = []
  for sign in (-1, 1):
    edges.append(NS(y=[sign * (1.8 + near + (far - near) * max(0.0, min(1.0, t - 1.0))) for t in ModelConstants.T_IDXS]))
  return NS(
    laneLines=[line(-5.3), line(-1.8), line(1.8), line(5.3)],
    roadEdges=edges,
    roadEdgeStds=[0.9, 0.9],
    laneLineProbs=[prob, 0.9, 0.9, prob],
    meta=NS(desireState=[1.0, 0.0, 0.0]),
  )


def classify(classifier, i, model=None, **kwargs):
  stamp = NOW_NS + i * 50_000_000
  return classifier.update(model or cp_model(), kwargs.pop('carstate', car_state(vEgo=7.5, aEgo=0.0)), stamp=stamp, now_ns=stamp, **kwargs)


def cp_tick(gate, i, *, md=None, topology_options=None, **kwargs):
  stamp = NOW_NS + i * 50_000_000
  topo = {'leftNeighborExists': False, 'rightNeighborExists': False, 'publishMonoTime': stamp, 'modelMonoTime': stamp, 'imageMonoTime': stamp}
  topo.update(topology_options or {})
  options = {'healthy': True, 'neighbors': (False, False), 'carstate': car_state(vEgo=7.5, aEgo=0.0)}
  options.update(kwargs)
  return gate.update(md or cp_model(), visual_topology(**topo), now_ns=stamp, model_stamp_ns=stamp, **options)


def test_cp_entry_uses_current_model_without_paint_topology_or_model_turn_prediction():
  gate = TurnEntryGate()
  for i in range(12):
    assert cp_tick(gate, i, topology_options={'validForControl': False}) == (False, False)
  assert cp_tick(gate, 12, topology_options={'validForControl': False}) == (True, True)
  assert gate.input_reason == 'cpModelObserved'


@pytest.mark.parametrize('prob,near,far,expected', [(0.9, 4.0, 6.0, False), (0.1, 2.0, 6.0, True), (0.1, 2.0, 3.0, False)])
def test_cp_score_distinguishes_stable_lane_and_turn(prob, near, far, expected):
  classifier = TurnIntentClassifier()
  for i in range(25):
    result = classify(classifier, i, cp_model(prob, near, far))
  assert result[0] == (expected, expected)


def test_new_stream_does_not_misclassify_a_stable_lane_during_counter_warmup():
  classifier = TurnIntentClassifier()
  for i in range(25):
    assert classify(classifier, i, cp_model(prob=.9, near=4., far=6.))[0] == (False, False)


@pytest.mark.parametrize('fault', ['nan', 'short', 'opposite', 'stale', 'duplicate', 'gap', 'unhealthy'])
def test_invalid_model_cannot_start_cp_turn(fault):
  c = TurnIntentClassifier()
  for i in range(15):
    classify(c, i)
  md = cp_model()
  kwargs = {}
  i = 15
  if fault == 'nan':
    md.roadEdges[0].y[0] = float('nan')
  if fault == 'short':
    md.roadEdges[0].y = []
  if fault == 'opposite':
    md.roadEdges[0].y = [5.0] * 33
  if fault == 'unhealthy':
    kwargs['healthy'] = False
  if fault == 'duplicate':
    i = 14
  if fault == 'gap':
    i = 30
  if fault == 'stale':
    result = c.update(md, car_state(vEgo=5.0, aEgo=0.0), stamp=NOW_NS, now_ns=NOW_NS + 1_000_000_000)
  else:
    result = classify(c, i, md, **kwargs)
  assert not result[0][0]


@pytest.mark.parametrize(
  'opts',
  [
    {'neighbors': (True, True)},
    {'neighbors': (None, None)},
    {'safety_blocks': (True, True)},
    {'model_healthy': False},
    {'topology_options': {'leftEgoSideMarking': 'solid', 'rightEgoSideMarking': 'roadEdge'}},
  ],
)
def test_cp_score_does_not_remove_control_vetoes(opts):
  gate = TurnEntryGate()
  for i in range(20):
    assert cp_tick(gate, i, **opts) == (False, False)


def test_cp_keeps_active_turn_when_opening_moves_alongside():
  gate = TurnEntryGate()
  for i in range(13):
    allowed = cp_tick(gate, i)
  assert allowed == (True, True)
  assert cp_tick(gate, 13, md=cp_model(far=3.0)) == (False, False)
  assert gate.keep_allowed == (True, True)
  cp_tick(gate, 14, safety_blocks=(True, True))
  assert gate.keep_allowed == (False, False)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_pending_manual_change_can_become_independently_confirmed_turn(side):
  dh = turn_helper()
  update(dh, side, speed=15.0, **{side + '_neighbor_exists': True, side + '_start_allowed': False})
  assert dh.lane_change_state == log.LaneChangeState.preLaneChange
  update(dh, side, speed=5.0)
  assert dh.desire == (log.Desire.turnLeft if side == 'left' else log.Desire.turnRight)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_pending_nav_lane_request_does_not_become_turn(side):
  dh = turn_helper()
  update(dh, side, speed=15.0, nav_lane_intent=intent(direction=side), **{side + '_neighbor_exists': True})
  update(dh, side, speed=5.0)
  assert dh.desire == log.Desire.none


@pytest.mark.parametrize('side', ['left', 'right'])
def test_driver_opposing_torque_consumes_turn(side):
  dh = turn_helper()
  update(dh, side)
  assert dh.desire != log.Desire.none
  dh.update(
    car_state(vEgo=5.0, steeringPressed=True, steeringTorque=-1 if side == 'left' else 1, **{side + 'Blinker': True}),
    True,
    1.0,
    left_turn_allowed=True,
    right_turn_allowed=True,
    left_turn_keep_allowed=True,
    right_turn_keep_allowed=True,
  )
  assert dh.desire == log.Desire.none
  update(dh, side)
  assert dh.desire == log.Desire.none


def test_navigation_score_requires_live_matching_source():
  c = TurnIntentClassifier()
  n = intent(target=-1)
  n.sessionId = 's'
  n.routeRevision = 1
  n.maneuverEventId = 2
  nav = NS(
    valid=True,
    stale=False,
    gpsWeak=False,
    routeActive=True,
    routeMatched=True,
    publishMonoTime=NOW_NS,
    sessionId='s',
    routeRevision=1,
    maneuverEventId=2,
    maneuver='turnLeft',
  )
  md = cp_model(prob=0.9, near=4.0, far=6.0)
  for i in range(25):
    classify(c, i, md)
  nav.publishMonoTime = NOW_NS + 25 * 50_000_000
  assert classify(c, 25, md, nav_intent=n, nav_state=nav)[0] == (True, False)
  nav.gpsWeak = True
  assert classify(c, 26, md, nav_intent=n, nav_state=nav)[0] == (False, False)
  nav.gpsWeak = False
  nav.maneuverEventId = 3
  assert classify(c, 27, md, nav_intent=n, nav_state=nav)[0] == (False, False)


def test_soft_loss_waits_for_reconfirmed_entry_but_hard_exit_cannot_retry():
  dh = turn_helper()
  update(dh, turn_soft_reentry=True)
  assert dh.desire == log.Desire.turnLeft
  update(dh, turn_soft_reentry=True, left_turn_allowed=False, left_turn_keep_allowed=False)
  assert dh.desire == log.Desire.none and dh.turn_maneuver.state == 'waiting'
  update(dh, turn_soft_reentry=True, left_turn_allowed=False, left_turn_keep_allowed=True)
  assert dh.desire == log.Desire.none
  update(dh, turn_soft_reentry=True)
  assert dh.desire == log.Desire.turnLeft
  update(dh, turn_soft_reentry=True, left_safety_blocked=True)
  update(dh, turn_soft_reentry=True)
  assert dh.desire == log.Desire.none and dh.turn_maneuver.state == 'finished'


def test_unknown_oem_recovery_needs_new_side_history():
  gate = TurnEntryGate()
  for i in range(13):
    cp_tick(gate, i)
  assert cp_tick(gate, 13, neighbors=(None, None)) == (False, False)
  for i in range(14, 26):
    assert cp_tick(gate, i) == (False, False)
  assert cp_tick(gate, 26) == (True, True)
