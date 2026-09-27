from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.turn_entry import TurnEntryGate, TurnCompletionTracker, side_opening, turn_end_detected
from openpilot.sunnypilot.selfdrive.controls.lib.oem_lane_change_gate import OemLaneChangeGate, lane_change_start_permissions
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_lane_intent_desire import helper, car_state, intent
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_oem_lane_change_gate import visual_topology, NOW_NS
from openpilot.sunnypilot.navassist.tests.test_oem_lane_feedback import topology


def model():
  def line(a, b): return NS(x=[0., 5., 20., 40.], y=[a, a, b, b])
  return NS(roadEdges=[line(-2., -7.), line(2., 7.)], roadEdgeStds=[.1, .1],
            laneLines=[line(-5., -5.), line(-1.8, -1.8), line(1.8, 1.8), line(5., 5.)])


def tick(gate, step, *, md=None, neighbors=(False, False), healthy=True, safety=(False, False), **updates):
  stamp = NOW_NS + step*50_000_000
  values = dict(leftNeighborExists=False, rightNeighborExists=False,
                publishMonoTime=stamp, modelMonoTime=stamp, imageMonoTime=stamp)
  values.update(updates)
  return gate.update(md or model(), visual_topology(**values), healthy=healthy, now_ns=stamp,
                     model_stamp_ns=stamp, neighbors=neighbors, safety_blocks=safety)


def ready(gate, **kwargs):
  result = None
  for step in range(11): result = tick(gate, step, **kwargs)
  return result


def test_entry_diagnostics_separate_held_boundary_from_oem_safety():
  gate = TurnEntryGate()
  tick(gate, 0, leftEgoSideMarking='solid')
  md = model(); md.roadEdgeStds[0] = 1.1
  assert not tick(gate, 1, md=md, leftEgoSideMarking='unknown', leftEvidenceValid=False)[0]
  assert gate.detail_reasons[0] == 'boundaryHeld/roadEdgeUncertain'
  assert gate.input_reason == 'observed'
  tick(gate, 2, md=md, safety=(True, False))
  assert gate.detail_reasons[0] == 'oemSafetyBlocked/roadEdgeUncertain'


def test_entry_diagnostics_distinguish_invalid_topology_from_message_dropout():
  gate = TurnEntryGate()
  assert not tick(gate, 0, validForControl=False)[0]
  assert gate.input_reason == 'topologyInvalidForControl'
  assert gate.detail_reasons[0] == 'evidenceUnavailable/notEvaluated'
  tick(gate, 1, healthy=False)
  assert gate.input_reason == 'sourceUnavailable'
  tick(gate, 2, imageMonoTime=1)
  assert gate.input_reason == 'topologyTimeInvalid'


def test_diagnostic_curve_failure_does_not_change_entry_decision():
  gate = TurnEntryGate(); md = model(); md.roadEdges[0].x = [0., 5.]
  assert not tick(gate, 0, md=md)[0]
  assert gate.detail_reasons[0] == 'geometryUnavailable/curveInvalidOrShort'


def test_opening_needs_distinct_continuous_evidence_and_both_sides_are_independent():
  gate = TurnEntryGate()
  for i in range(10): assert tick(gate, i) == (False, False)
  assert tick(gate, 10) == (True, True)
  assert tick(gate, 11, neighbors=(True, False)) == (False, True)
  assert tick(gate, 12, neighbors=(None, False)) == (False, True)
  assert tick(gate, 13) == (False, True)


@pytest.mark.parametrize('kwargs', [
  {'healthy': False}, {'neighbors': (None, None)}, {'neighbors': (True, True)},
  {'leftNeighborExists': True, 'rightNeighborExists': True}, {'validForControl': False},
  {'publishMonoTime': 1}, {'modelMonoTime': 1}, {'imageMonoTime': 1},
  {'publishMonoTime': NOW_NS+10_000_000_000}, {'safety': (True, True)},
  {'leftEgoSideMarking': 'solid', 'rightEgoSideMarking': 'doubleSolid'},
  {'leftEgoSideMarking': 'roadEdge', 'rightEgoSideMarking': 'solidDashed'},
])
def test_missing_conflicting_stale_or_blocked_evidence_never_authorizes(kwargs):
  assert ready(TurnEntryGate(), **kwargs) == (False, False)


@pytest.mark.parametrize('fault', ['uncertain', 'nan', 'short', 'reversed', 'duplicate', 'parallel', 'wrong_side', 'missing'])
def test_invalid_geometry_is_not_an_opening(fault):
  md = model()
  if fault == 'uncertain': md.roadEdgeStds = [.9, .9]
  if fault == 'nan': md.roadEdges[0].y[2] = float('nan')
  if fault == 'short': md.roadEdges[0].x = [0., 5., 10., 15.]
  if fault == 'reversed': md.roadEdges[0].x.reverse()
  if fault == 'duplicate': md.roadEdges[0].x[2] = 5.
  if fault == 'parallel': md.roadEdges[0].y = [-2.]*4
  if fault == 'wrong_side': md.roadEdges[0].y = [2., 2., 7., 7.]
  if fault == 'missing': md.roadEdges = []
  assert not side_opening(md, 'left')
  assert not ready(TurnEntryGate(), md=md)[0]


def test_duplicate_gap_and_dropout_restart_confirmation():
  gate = TurnEntryGate()
  assert ready(gate) == (True, True)
  assert tick(gate, 10) == (False, False)
  assert tick(gate, 11) == (False, False)
  assert tick(gate, 30) == (False, False)
  for i in range(31, 41): result = tick(gate, i)
  assert result == (True, True)
  assert tick(gate, 41, healthy=False) == (False, False)
  assert tick(gate, 42) == (False, False)


@pytest.mark.parametrize('bits,expected', [(0, (False, False)), (1, (True, False)), (2, (False, True)), (3, (True, True))])
def test_oem_absence_requires_confirmed_239_and_expires(bits, expected):
  gate = OemLaneChangeGate()
  assert gate.neighbors == (None, None)
  for i in range(1, 4):
    stamp = NOW_NS+i*50_000_000
    gate.update([NS(valid=True, logMonoTime=stamp, can=[topology(bits, i)])], stamp)
    assert gate.neighbors == (expected if i == 3 else (None, None))
  gate.update([], stamp+400_000_001)
  assert gate.neighbors == (None, None)


def turn_helper():
  dh = helper()
  dh.lane_turn_controller.enabled = True
  dh.lane_turn_controller.lane_turn_value = 8.5
  return dh


def update(dh, side='left', speed=5., **kwargs):
  options = dict(left_turn_allowed=True, right_turn_allowed=True,
                 left_neighbor_exists=False, right_neighbor_exists=False,
                 left_start_allowed=False, right_start_allowed=False)
  options.update(kwargs)
  dh.update(car_state(vEgo=speed, **{side+'Blinker': True}), True, 1., **options)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_manual_turn_uses_own_permission_without_navigation_or_lane_crossing(side):
  dh = turn_helper()
  update(dh, side, **{side+'_start_allowed': False})
  assert dh.desire == (log.Desire.turnLeft if side == 'left' else log.Desire.turnRight)
  update(dh, side, **{side+'_safety_blocked': True})
  assert dh.desire == log.Desire.none
  update(dh, side)
  assert dh.desire == log.Desire.none


@pytest.mark.parametrize('side', ['left', 'right'])
def test_low_speed_neighbor_observation_waits_for_confirmed_turn_entry(side):
  dh = turn_helper(); gate = TurnEntryGate()
  expected = log.Desire.turnLeft if side == 'left' else log.Desire.turnRight
  for i in range(10):
    drive_tick(dh, gate, i, side, gate_options={side+'NeighborExists': True})
    assert dh.desire == log.Desire.none
  # This signal never requested or entered a lane change. After a genuinely
  # different, continuously confirmed opening, it can become a manual turn.
  for i in range(10, 20):
    drive_tick(dh, gate, i, side)
    assert dh.desire == log.Desire.none
  drive_tick(dh, gate, 20, side)
  assert dh.desire == expected


def test_confirmed_outer_lane_turn_can_signal_before_slowing():
  dh = turn_helper()
  update(dh, speed=12.)
  assert dh.lane_change_state == log.LaneChangeState.off
  assert dh.desire == log.Desire.none
  update(dh, speed=5.)
  assert dh.desire == log.Desire.turnLeft


def test_lane_change_started_at_speed_cannot_fall_back_to_turn():
  dh = turn_helper()
  for _ in range(4): update(dh, speed=15., left_neighbor_exists=True, left_start_allowed=True)
  assert dh.desire == log.Desire.laneChangeLeft
  for _ in range(60):
    update(dh)
    assert dh.desire == log.Desire.none


def test_opening_permission_withdrawal_does_not_repeat_model_turn_pulses():
  dh = turn_helper()
  update(dh)
  assert dh.desire == log.Desire.turnLeft
  update(dh, left_turn_allowed=False)
  assert dh.desire == log.Desire.none
  update(dh)
  assert dh.desire == log.Desire.none


def test_navigation_turn_cannot_bypass_missing_opening():
  dh = turn_helper()
  update(dh, left_turn_allowed=False, nav_lane_intent=intent(target=-1))
  assert dh.desire == log.Desire.none


@pytest.mark.parametrize('side', ['left', 'right'])
def test_navigation_lane_direction_is_selected_before_physical_lamp(side):
  dh = turn_helper()
  options = dict(left_turn_allowed=False, right_turn_allowed=False,
                 left_neighbor_exists=side == 'left', right_neighbor_exists=side == 'right',
                 nav_lane_intent=intent(direction=side), left_crossing_allowed=True, right_crossing_allowed=True)
  dh.update(car_state(), True, 1., **options)
  assert dh.lane_change_state == log.LaneChangeState.preLaneChange
  for _ in range(5): dh.update(car_state(**{side+'Blinker': True}), True, 1., **options)
  assert dh.lane_change_state == log.LaneChangeState.laneChangeStarting


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('marking', ['solid', 'doubleSolid', 'solidDashed', 'roadEdge'])
def test_current_boundary_veto_reaches_manual_turn_desire(side, marking):
  gate = TurnEntryGate()
  allowed = ready(gate, **{side+'EgoSideMarking': marking})
  dh = turn_helper()
  update(dh, side, left_turn_allowed=allowed[0], right_turn_allowed=allowed[1])
  assert dh.desire == log.Desire.none


def test_reported_low_speed_adjacent_lane_fixture_is_rejected_even_if_paint_is_dashed():
  # Recorded 22:03:37 topology had leftNeighborExists=True. This is a
  # branch reproduction with synthetic edges, not a full model/video replay.
  gate = TurnEntryGate()
  for i in range(30):
    allowed = tick(gate, i, leftNeighborExists=True, leftEgoSideMarking='dashed')
    assert not allowed[0]
  dh = turn_helper()
  update(dh, speed=27.2/3.6, left_turn_allowed=allowed[0])
  assert dh.desire == log.Desire.none


def drive_tick(dh, gate, step, side='left', *, gate_options=None, **options):
  entry = tick(gate, step, **(gate_options or {}))
  kwargs = dict(left_turn_allowed=entry[0], right_turn_allowed=entry[1],
                left_turn_keep_allowed=gate.keep_allowed[0], right_turn_keep_allowed=gate.keep_allowed[1],
                left_neighbor_exists=gate.neighbors[0], right_neighbor_exists=gate.neighbors[1])
  kwargs.update(options)
  update(dh, side, **kwargs)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('marking', ['solid', 'doubleSolid', 'solidDashed', 'roadEdge'])
def test_retained_boundary_cannot_be_erased_by_unknown_or_new_signal(side, marking):
  gate = TurnEntryGate()
  dh = turn_helper()
  drive_tick(dh, gate, 0, side, gate_options={side+'EgoSideMarking': marking})
  for i in range(1, 41):
    if i == 20:
      dh.update(car_state(), True, 1., left_turn_allowed=False, right_turn_allowed=False)
    drive_tick(dh, gate, i, side, gate_options={side+'EgoSideMarking': 'unknown', side+'EvidenceValid': False})
    assert dh.desire == log.Desire.none
    assert gate.hard_blocks[0 if side == 'left' else 1]
  # Fresh positive paint evidence can clear the hold. Unknown and signal reset cannot.
  for i in range(41, 57): drive_tick(dh, gate, i, side)
  assert dh.desire == (log.Desire.turnLeft if side == 'left' else log.Desire.turnRight)


def test_repeated_topology_cannot_clear_solid_memory():
  gate = TurnEntryGate()
  tick(gate, 0, leftEgoSideMarking='solid')
  for i in range(1, 31):
    tick(gate, i, modelMonoTime=NOW_NS+50_000_000)
    assert gate.hard_blocks[0]


def test_caller_boundary_memory_also_vetoes_entry_with_positive_geometry():
  dh = turn_helper()
  update(dh, left_line_blocked=True)
  assert dh.desire == log.Desire.none


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('navigation', [False, True])
def test_visual_lane_permission_survives_conflicting_239_absence(side, navigation):
  gate = TurnEntryGate()
  dh = turn_helper()
  permissions = lane_change_start_permissions(visual_topology(), healthy=True, now_ns=NOW_NS,
                                              oem_permissions=(False, False))
  assert permissions == (True, True)
  options = dict(left_start_allowed=permissions[0], right_start_allowed=permissions[1],
                 left_crossing_allowed=True, right_crossing_allowed=True)
  if navigation: options['nav_lane_intent'] = intent(direction=side)
  for i in range(5):
    drive_tick(dh, gate, i, side, speed=15., gate_options={side+'NeighborExists': True}, **options)
  assert dh.desire == (log.Desire.laneChangeLeft if side == 'left' else log.Desire.laneChangeRight)
  assert gate.neighbors[0 if side == 'left' else 1] is True


def test_raw_oem_absence_cannot_be_an_extra_lane_permission_veto():
  dh = turn_helper()
  for _ in range(5): update(dh, speed=15., left_start_allowed=True, left_turn_allowed=False)
  assert dh.desire == log.Desire.laneChangeLeft


@pytest.mark.parametrize('side', ['left', 'right'])
def test_active_turn_survives_one_unknown_frame_without_requalifying_entry(side):
  dh = turn_helper(); gate = TurnEntryGate()
  expected = log.Desire.turnLeft if side == 'left' else log.Desire.turnRight
  for i in range(11): drive_tick(dh, gate, i, side)
  assert dh.desire == expected
  drive_tick(dh, gate, 11, side, gate_options={'healthy': False})
  assert dh.desire == expected
  for i in range(12, 25):
    drive_tick(dh, gate, i, side)
    assert dh.desire == expected
  assert not dh._cancelled_signal


def test_keep_permission_does_not_admit_new_turn_during_dropout():
  gate = TurnEntryGate()
  ready(gate)
  dh = turn_helper()
  drive_tick(dh, gate, 11, gate_options={'healthy': False})
  assert gate.keep_allowed[0]
  assert dh.desire == log.Desire.none


def test_long_model_loop_pause_cannot_refresh_away_an_expired_active_turn():
  gate = TurnEntryGate(); dh = turn_helper()
  for i in range(11): drive_tick(dh, gate, i)
  assert dh.desire == log.Desire.turnLeft
  # No update at all for 0.5 s, then a new healthy frame arrives.
  drive_tick(dh, gate, 20)
  assert dh.desire == log.Desire.none
  for i in range(21, 35):
    drive_tick(dh, gate, i)
    assert dh.desire == log.Desire.none


@pytest.mark.parametrize('side', ['left', 'right'])
def test_driver_can_signal_when_preconfirmed_opening_reaches_the_car(side):
  gate = TurnEntryGate()
  assert ready(gate) == (True, True)  # Confirmation can occur before the lamp.
  md = model()
  index = 0 if side == 'left' else 1
  sign = -1 if side == 'left' else 1
  md.roadEdges[index].y = [sign*6.2]*4  # Both net clearances now exceed 4 m.
  dh = turn_helper()
  drive_tick(dh, gate, 11, side, gate_options={'md': md})
  assert dh.desire == (log.Desire.turnLeft if side == 'left' else log.Desire.turnRight)


@pytest.mark.parametrize('break_history', ['never_confirmed', 'dropout', 'neighbor', 'solid', 'time_gap'])
def test_wide_space_without_continuous_confirmed_approach_cannot_start_turn(break_history):
  gate = TurnEntryGate()
  if break_history != 'never_confirmed': ready(gate)
  options = {'dropout': {'healthy': False}, 'neighbor': {'neighbors': (True, False)},
             'solid': {'leftEgoSideMarking': 'solid'}}
  if break_history in options: tick(gate, 11, **options[break_history])
  md = model(); md.roadEdges[0].y = [-6.2]*4
  start = 20 if break_history == 'time_gap' else 12
  dh = turn_helper()
  for i in range(start, start+30):
    drive_tick(dh, gate, i, gate_options={'md': md})
    assert dh.desire == log.Desire.none


def test_unconfirmed_approach_cannot_skip_half_second_confirmation_at_the_mouth():
  gate = TurnEntryGate()
  for i in range(5): assert not tick(gate, i)[0]
  md = model(); md.roadEdges[0].y = [-6.2]*4
  for i in range(5, 25): assert not tick(gate, i, md=md)[0]


@pytest.mark.parametrize('navigation', [False, True])
def test_pending_change_needs_independent_turn_evidence_and_preserves_navigation_purpose(navigation):
  dh = turn_helper()
  opts = {'nav_lane_intent': intent()} if navigation else {}
  update(dh, speed=15., left_neighbor_exists=True, left_start_allowed=False, **opts)
  assert dh.lane_change_state == log.LaneChangeState.preLaneChange
  for _ in range(20):
    update(dh, speed=5., left_turn_allowed=False, **opts)
    assert dh.desire == log.Desire.none
  update(dh, speed=5., **opts)
  assert dh.desire == (log.Desire.none if navigation else log.Desire.turnLeft)


def test_low_speed_navigation_lane_request_is_not_reclassified_as_manual_turn():
  dh = turn_helper()
  update(dh, nav_lane_intent=intent())
  update(dh)
  assert dh.desire == log.Desire.none


def test_opening_can_move_behind_an_active_turn_but_cannot_start_a_new_one():
  gate = TurnEntryGate(); active = turn_helper(); waiting = turn_helper()
  for i in range(11): drive_tick(active, gate, i)
  md = model()
  md.roadEdges[0].y = [-2.]*4  # Valid outer edge, no widening opening now.
  for i in range(11, 40):
    entry = tick(gate, i, md=md)
    assert not entry[0] and gate.keep_allowed[0]
    for dh in (active, waiting):
      update(dh, left_turn_allowed=entry[0], left_turn_keep_allowed=gate.keep_allowed[0])
    assert active.desire == log.Desire.turnLeft
    assert waiting.desire == log.Desire.none


@pytest.mark.parametrize('fault', ['unhealthy', 'geometry', 'neighbor', 'duplicate'])
def test_prolonged_loss_ends_once_and_recovery_cannot_retrigger(fault):
  dh = turn_helper(); gate = TurnEntryGate()
  for i in range(11): drive_tick(dh, gate, i)
  md = model(); md.roadEdgeStds[0] = .9
  for i in range(11, 21):
    opts = {'unhealthy': {'healthy': False}, 'geometry': {'md': md},
            'neighbor': {'neighbors': (None, False)}, 'duplicate': {}}[fault]
    if fault == 'duplicate':
      # A replayed frame's evidence expires against wall time, not call count.
      stamp = NOW_NS+10*50_000_000
      a = gate.update(model(), visual_topology(leftNeighborExists=False, rightNeighborExists=False,
                      publishMonoTime=stamp, modelMonoTime=stamp, imageMonoTime=stamp), healthy=True,
                      now_ns=NOW_NS+i*50_000_000, model_stamp_ns=stamp, neighbors=(False, False))
      update(dh, left_turn_allowed=a[0], left_turn_keep_allowed=gate.keep_allowed[0])
    else:
      drive_tick(dh, gate, i, gate_options=opts)
    if i >= 14: assert dh.desire == log.Desire.none
  for i in range(21, 40):
    drive_tick(dh, gate, i)
    assert dh.desire == log.Desire.none
  assert dh.turn_maneuver.state == 'finished'


@pytest.mark.parametrize('hazard', ['solid', 'blindspot', '399', 'neighbor', 'edge_conflict'])
def test_hazard_withdraws_immediately_without_dropout_grace(hazard):
  dh = turn_helper(); gate = TurnEntryGate()
  for i in range(11): drive_tick(dh, gate, i)
  opts = {}
  if hazard == 'solid': opts = {'leftEgoSideMarking': 'solid'}
  if hazard == '399': opts = {'safety': (True, False), 'healthy': False}
  if hazard == 'neighbor': opts = {'neighbors': (True, False), 'healthy': False}
  if hazard == 'edge_conflict':
    md = model(); md.roadEdges[0].y = [1.]*4; opts = {'md': md}
  if hazard == 'blindspot':
    dh.update(car_state(vEgo=5., leftBlinker=True, leftBlindspot=True), True, 1.,
              left_turn_allowed=True, right_turn_allowed=False, left_turn_keep_allowed=True)
  else:
    drive_tick(dh, gate, 11, gate_options=opts)
  assert dh.desire == log.Desire.none
  assert dh.turn_maneuver.state == 'finished'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_speed_exit_consumes_signal_and_cannot_restart_as_turn_or_lane_change(side):
  dh = turn_helper()
  update(dh, side, speed=8.4)
  assert dh.desire != log.Desire.none
  for speed in (8.6, 8.4, 5., 15., 8.4):
    update(dh, side, speed=speed, left_start_allowed=True, right_start_allowed=True)
    assert dh.desire == log.Desire.none
  dh.update(car_state(), True, 1., left_turn_allowed=False, right_turn_allowed=False)
  update(dh, side)
  assert dh.desire != log.Desire.none


@pytest.mark.parametrize('angle,near,far,expected', [
  (90., .3, .1, True), (-90., -.3, -.1, True), (80., .3, .1, False),
  (90., .1, .3, False), (float('nan'), .3, .1, False), (90., float('nan'), .1, False),
])
def test_cp_completion_hint_and_lifecycle(angle, near, far, expected):
  md = model(); md.orientationRate = NS(z=[0.]*16)
  md.orientationRate.z[5], md.orientationRate.z[15] = near, far
  completed = turn_end_detected(md, car_state(steeringAngleDeg=angle))
  assert completed == expected
  dh = turn_helper(); update(dh)
  update(dh, turn_completed=completed)
  assert (dh.desire == log.Desire.none) == expected
  update(dh)
  assert (dh.desire == log.Desire.none) == expected


def test_lateral_deactivation_ends_active_turn_until_signal_is_reset():
  dh = turn_helper(); update(dh)
  dh.update(car_state(vEgo=5., leftBlinker=True), False, 1., left_turn_allowed=True, right_turn_allowed=False)
  assert dh.desire == log.Desire.none
  update(dh)
  assert dh.desire == log.Desire.none


@pytest.mark.parametrize('modeld_path', ['openpilot/selfdrive/modeld/modeld.py', 'openpilot/sunnypilot/modeld_v2/modeld.py'])
def test_both_modeld_adapters_with_cereal_messages(modeld_path):
  # Execute the real adapter statements against real messages, without loading
  # a GPU model or sending CAN. This catches mismatched fields and DH keywords.
  import ast
  from pathlib import Path
  from openpilot.cereal import messaging

  source = ast.parse((Path(__file__).parents[6] / modeld_path).read_text())
  main = next(n for n in source.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
  statements = None
  for block in ast.walk(main):
    body = getattr(block, 'body', None)
    if not isinstance(body, list): continue
    for index, stmt in enumerate(body):
      if (isinstance(stmt, ast.Assign) and isinstance(stmt.value, ast.Dict) and not stmt.value.keys
          and any(isinstance(t, ast.Name) and t.id == 'turn_permissions' for t in stmt.targets)):
        statements = body[index:index+3]
  assert statements is not None
  for block in ast.walk(main):
    body = getattr(block, 'body', None)
    if not isinstance(body, list): continue
    for index, stmt in enumerate(body):
      if (isinstance(stmt, ast.Assign) and any(isinstance(t, ast.Attribute) and t.attr == 'laneTurnDirection'
                                              for t in stmt.targets)):
        statements += body[index:index+2]
  adapter = compile(ast.Module(body=statements, type_ignores=[]), modeld_path, 'exec')
  md_msg = messaging.new_message('modelV2')
  md = model()
  md.roadEdges = [NS(x=list(range(33)), y=[y]*33) for y in (-7., 7.)]
  md.laneLines = [NS(x=list(range(33)), y=[y]*33) for y in (-5., -1.8, 1.8, 5.)]
  md_msg.modelV2.roadEdges = [vars(line) for line in md.roadEdges]
  md_msg.modelV2.laneLines = [vars(line) for line in md.laneLines]
  md_msg.modelV2.roadEdgeStds = md.roadEdgeStds
  md_msg.modelV2.orientationRate.z = [0.]*33
  md_msg.modelV2.laneLineProbs = [.1, .9, .9, .1]
  md_msg.modelV2.meta.desireState = [1., 0., 0.]
  topo_msg = messaging.new_message('laneTopologyStateSP')
  topo_msg.laneTopologyStateSP = vars(visual_topology(leftNeighborExists=False, rightNeighborExists=False))
  car_msg = messaging.new_message('carState')
  car_msg.carState.vEgo = 5.
  car_msg.carState.leftBlinker = True
  control_msg = messaging.new_message('carControl')
  control_msg.carControl.latActive = True
  class TestSubMaster(dict):
    def all_checks(self, services):
      return 'navAssistStateSP' not in services
  env = dict(turn_entry=TurnEntryGate(), oem_gate=NS(neighbors=(False, False), safety_blocks=(False, False)),
             mdv2sp_send=NS(modelDataV2SP=NS()), live_calib_seen=True, extrinsics_calibration_seen=True,
             turn_completion=TurnCompletionTracker(.1), modelv2_send=md_msg, topology=topo_msg.laneTopologyStateSP,
             lane_topology_healthy=True, DH=turn_helper(), lane_change_prob=1., left_edge=False, right_edge=False,
             nav_lane_intent=None, left_line_blocked=False, right_line_blocked=False,
             left_crossing_allowed=False, right_crossing_allowed=False, left_start_allowed=False, right_start_allowed=False,
             sm=TestSubMaster(carState=car_msg.carState, carControl=control_msg.carControl))
  for i in range(14):
    stamp = NOW_NS+i*50_000_000
    env.update(gate_now_ns=stamp, meta_main=NS(timestamp_eof=stamp))
    for field in ('publishMonoTime', 'modelMonoTime', 'imageMonoTime'): setattr(env['topology'], field, stamp)
    exec(adapter, env)
  assert env['DH'].desire == log.Desire.turnLeft
  diagnostic = env['mdv2sp_send'].modelDataV2SP
  assert diagnostic.turnEntryModelMonoTime == stamp
  assert diagnostic.turnEntryInputReason == 'cpModelObserved'
  assert diagnostic.turnEntryLeftReason == 'turnOpeningConfirmed/cpTurn'
  assert diagnostic.turnDecisionReason == 'turnActive'
  md_msg.modelV2.orientationRate.z = [0.]*5+[.3]+[0.]*9+[.1]+[0.]*17
  for i, angle in enumerate((100., 87.5, 75.)):
    car_msg.carState.steeringAngleDeg = angle
    stamp += 50_000_000
    env.update(gate_now_ns=stamp, meta_main=NS(timestamp_eof=stamp))
    for field in ('publishMonoTime', 'modelMonoTime', 'imageMonoTime'): setattr(env['topology'], field, stamp)
    exec(adapter, env)
    if i < 2: assert env['DH'].desire == log.Desire.turnLeft
  assert env['DH'].desire == log.Desire.none
  assert env['mdv2sp_send'].modelDataV2SP.turnDecisionReason == 'signalConsumed'


def completion_tick(tracker, step, angle, *, side='left', active=True, near=.3, far=.1):
  md = model(); md.orientationRate = NS(z=[0.]*16)
  md.orientationRate.z[5], md.orientationRate.z[15] = near, far
  stamp = NOW_NS+step*50_000_000
  return tracker.update(md, car_state(steeringAngleDeg=angle, **{side+'Blinker': True}),
                        active=active, model_stamp_ns=stamp, now_ns=stamp)


@pytest.mark.parametrize('side,sign', [('left', 1.), ('right', -1.)])
def test_modely_unwinding_and_actuator_delay_confirm_cp_completion(side, sign):
  tracker = TurnCompletionTracker(.10000000149011612)  # Device CarParams float32.
  assert tracker.confirm_ns == 100_000_000
  assert not completion_tick(tracker, 0, sign*100., side=side)
  assert not completion_tick(tracker, 1, sign*95., side=side)
  assert completion_tick(tracker, 2, sign*90., side=side)


@pytest.mark.parametrize('angles', [(85., 90., 100.), (100., 100., 100.), (-100., -95., -90.)])
def test_winding_up_steady_or_opposite_steering_is_not_completion(angles):
  tracker = TurnCompletionTracker(.1)
  for i, angle in enumerate(angles): assert not completion_tick(tracker, i, angle)


@pytest.mark.parametrize('fault', ['duplicate', 'gap', 'inactive', 'direction', 'prediction', 'nonfinite'])
def test_completion_requires_continuous_matching_measurements(fault):
  tracker = TurnCompletionTracker(.1)
  completion_tick(tracker, 0, 110.)
  assert not completion_tick(tracker, 1, 105.)
  if fault == 'duplicate': assert not completion_tick(tracker, 1, 104.)
  elif fault == 'gap': assert not completion_tick(tracker, 10, 104.)
  elif fault == 'inactive': assert not completion_tick(tracker, 2, 104., active=False)
  elif fault == 'direction': assert not completion_tick(tracker, 2, -104., side='right')
  elif fault == 'prediction': assert not completion_tick(tracker, 2, 104., near=.1, far=.3)
  elif fault == 'nonfinite': assert not completion_tick(tracker, 2, float('nan'))
  assert not completion_tick(tracker, 3 if fault != 'gap' else 11, 100.)


@pytest.mark.parametrize('delay', [float('nan'), -1., 5.])
def test_invalid_vehicle_delay_cannot_prove_turn_finished(delay):
  tracker = TurnCompletionTracker(delay)
  for i in range(10): assert not completion_tick(tracker, i, 150.-i)


@pytest.mark.parametrize('side,sign', [('left', 1.), ('right', -1.)])
@pytest.mark.parametrize('angles,expected', [
  ([100., 87.5, 75., 62.5, 50.], [False, False, True, True, True]),
  ([100., 99.9, 99.8], [False, False, False]),
  ([100.-i*.1 for i in range(30)], [False]*30),
  ([100., 98., 96., 95.], [False, False, False, True]),
  ([80., 70., 60.], [False, False, False]),
  ([100., 90., 92., 89., 87.], [False, False, False, False, True]),
  ([100., 90., 90., 88., 85.], [False, False, False, False, True]),
])
def test_completion_tracks_turn_history_and_meaningful_continuous_unwind(side, sign, angles, expected):
  tracker = TurnCompletionTracker(.1)
  assert [completion_tick(tracker, i, sign*angle, side=side) for i, angle in enumerate(angles)] == expected


@pytest.mark.parametrize('fault', ['duplicate', 'backward', 'gap', 'stale', 'future', 'inactive',
                                 'direction', 'nonfinite', 'opposite', 'no_signal', 'both_signals'])
def test_interrupted_completion_cannot_reuse_old_high_angle(fault):
  tracker = TurnCompletionTracker(.1)
  assert not completion_tick(tracker, 0, 100.)
  assert not completion_tick(tracker, 1, 87.5)
  if fault in ('duplicate', 'backward', 'gap'):
    assert not completion_tick(tracker, {'duplicate': 1, 'backward': 0, 'gap': 10}[fault], 75.)
  elif fault in ('stale', 'future', 'no_signal', 'both_signals'):
    stamp = NOW_NS+100_000_000
    now = stamp+1_000_000_000 if fault == 'stale' else stamp-1 if fault == 'future' else stamp
    md = NS(orientationRate=NS(z=[0.]*5+[.3]+[0.]*9+[.1]))
    assert not tracker.update(md, car_state(steeringAngleDeg=75., leftBlinker=fault != 'no_signal',
                                           rightBlinker=fault == 'both_signals'), active=True,
                              model_stamp_ns=stamp, now_ns=now)
  else:
    opts = {'inactive': {'active': False}, 'direction': {'side': 'right'},
            'nonfinite': {}, 'opposite': {}}[fault]
    angle = float('nan') if fault == 'nonfinite' else -75. if fault == 'opposite' else 75.
    assert not completion_tick(tracker, 2, angle, **opts)
  next_step = 11 if fault == 'gap' else 23 if fault == 'stale' else 3
  for i in range(next_step, next_step+4):
    assert not completion_tick(tracker, i, 70.-(i-next_step)*10.)


@pytest.mark.parametrize('rates', [[], [0.]*15, [0.]*5+[float('nan')]+[0.]*9+[.1],
                                 [0.]*5+[.3]+[0.]*9+[float('inf')], [0.]*16])
def test_crossing_angle_threshold_still_requires_current_prediction(rates):
  tracker = TurnCompletionTracker(.1)
  assert not completion_tick(tracker, 0, 100.)
  assert not completion_tick(tracker, 1, 87.5)
  stamp = NOW_NS+100_000_000
  assert not tracker.update(NS(orientationRate=NS(z=rates)), car_state(steeringAngleDeg=75., leftBlinker=True),
                            active=True, model_stamp_ns=stamp, now_ns=stamp)
