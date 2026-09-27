"""jihui approach -> fork now -> native model desire -> one-shot completion."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.navassist.lane_intent import (
  LaneIntentDirection, LaneTopologyInput, LaneVehicleInput, NavLaneIntent, NavLaneIntentCoordinator,
  NavTurnPlan, NavTurnSignalCoordinator, ObservedLaneChangeState,
)
from openpilot.sunnypilot.navassist.nav_lane_intentd import build_lane_plan, apply_oem_signal_gate, lane_alignment_may_start
from openpilot.sunnypilot.navassist.settings import NavAssistSettings
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_lane_intent_desire import helper, car_state


def guidance(side, distance=50., **updates):
  values = dict(valid=True, stale=False, lanes=[], maneuver='ramp' + side.title(), maneuverDistanceM=distance,
                sessionId='fork-route', routeRevision=1, maneuverEventId=55, roadClass=6)
  values.update(updates)
  return SimpleNamespace(**values)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_final_fork_reaches_model_and_completes_once_without_an_adjacent_lane_index(side):
  # Existing approach alignment waits at an edge; final fork can use 0x399's
  # positive permission even before vision labels a full adjacent lane.
  topology = LaneTopologyInput(True, 1, 0, None, None, True, True)
  coordinator = NavLaneIntentCoordinator()
  plan = build_lane_plan(guidance(side), SimpleNamespace(visibleLaneCount=1), healthy=True)
  assert plan.force_fork
  dh = helper()
  direction = LaneIntentDirection.left if side == 'left' else LaneIntentDirection.right
  desired = log.Desire.laneChangeLeft if side == 'left' else log.Desire.laneChangeRight
  saw_start = saw_finish = completed = False
  car = car_state(vEgo=5.)  # CP fork requests do not use its normal lane-change speed floor.
  last_request = None
  for frame in range(100):
    vehicle = LaneVehicleInput(True, 5., left_blinker=car.leftBlinker, right_blinker=car.rightBlinker,
                               lane_change_state=ObservedLaneChangeState(int(dh.lane_change_state)),
                               lane_change_direction=LaneIntentDirection(int(dh.lane_change_direction)))
    request = coordinator.update(plan, topology, vehicle, now_ns=frame * 50_000_000)
    wire = SimpleNamespace(valid=True, signalRequested=request.signal_requested, targetLaneIndex=request.target_lane_index,
                           direction=side, spLaneChangeReady=request.lane_change_ready, forkNow=coordinator.fork_active,
                           sessionId=plan.session_id, maneuverEventId=plan.maneuver_event_id, requestId=request.request_id)
    probability = 0. if saw_start else 1.
    dh.update(car, True, probability, nav_lane_intent=wire,
              left_crossing_allowed=True, right_crossing_allowed=True)
    if dh.lane_change_state == log.LaneChangeState.laneChangeStarting:
      saw_start = True
      assert dh.desire == desired
    saw_finish |= dh.lane_change_state == log.LaneChangeState.laneChangeFinishing
    if request.reason == 'forkCompleted':
      completed = True
      last_request = request
      break
    if request.signal_requested:  # Synthetic physical lamp feedback, never sent to the car.
      car = car_state(vEgo=5., **{side + 'Blinker': True})
  assert saw_start and saw_finish and completed
  assert not last_request.signal_requested and not coordinator.fork_active
  assert apply_oem_signal_gate(last_request, 'visualNoRightNeighbor', 'idle') == last_request
  for frame in range(100, 120):
    request = coordinator.update(plan, topology, LaneVehicleInput(True, 5.), now_ns=frame * 50_000_000)
    assert request.reason == 'forkCompleted' and not request.signal_requested
  fresh = replace(plan, maneuver_event_id=56)
  assert coordinator.update(fresh, topology, LaneVehicleInput(True, 5.), now_ns=7_000_000_000).signal_requested


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('block', ['permission', 'blindspot', 'edge', 'solid', 'driver', 'standstill'])
def test_fork_does_not_bypass_control_and_crossing_gates(side, block):
  plan = build_lane_plan(guidance(side), SimpleNamespace(visibleLaneCount=1), healthy=True)
  coordinator = NavLaneIntentCoordinator()
  observed = LaneTopologyInput(True, 1, 0, None, None, block != 'permission', block != 'permission')
  dh = helper()
  for frame in range(10):
    cs = car_state(vEgo=0. if block == 'standstill' else 5.,
                   **{side + 'Blinker': True, side + 'Blindspot': block == 'blindspot'})
    vehicle = LaneVehicleInput(block != 'driver', cs.vEgo,
                               left_blinker=cs.leftBlinker, right_blinker=cs.rightBlinker,
                               left_blindspot=cs.leftBlindspot, right_blindspot=cs.rightBlindspot)
    request = coordinator.update(plan, observed, vehicle, now_ns=frame * 50_000_000)
    wire = SimpleNamespace(valid=True, signalRequested=request.signal_requested, targetLaneIndex=request.target_lane_index,
                           direction=side, spLaneChangeReady=request.lane_change_ready, forkNow=coordinator.fork_active)
    kwargs = {side + '_crossing_allowed': block != 'permission', side + '_start_allowed': block != 'permission',
              side + '_edge_detected': block == 'edge', side + '_line_blocked': block == 'solid'}
    dh.update(cs, block != 'driver', 1., nav_lane_intent=wire, **kwargs)
    assert dh.lane_change_state != log.LaneChangeState.laneChangeStarting
    assert dh.desire not in (log.Desire.laneChangeLeft, log.Desire.laneChangeRight)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_fork_window_is_reference_timing_and_never_turns_keep_or_straight_into_fork(side):
  for distance, expected in ((1500., False), (80., False), (79., True), (1., True), (0., False), (-1., False)):
    plan = build_lane_plan(guidance(side, distance), SimpleNamespace(visibleLaneCount=1), healthy=True)
    assert plan.force_fork == expected
  for maneuver in ('straight', 'keepLeft', 'slightRight', 'turnRight'):
    plan = build_lane_plan(guidance(side, maneuver=maneuver), SimpleNamespace(visibleLaneCount=1), healthy=True)
    assert not plan.force_fork
  plan = build_lane_plan(guidance(side), SimpleNamespace(visibleLaneCount=1), healthy=True,
                         settings=NavAssistSettings(lane_change_enabled=False))
  assert not plan.valid and not plan.force_fork


@pytest.mark.parametrize('side', ['left', 'right'])
def test_held_turn_only_lamp_hands_off_to_fork_model_request(side):
  dh = helper()
  car = car_state(vEgo=5., **{side + 'Blinker': True})
  wire = SimpleNamespace(valid=True, signalRequested=True, targetLaneIndex=-1, direction=side,
                         spLaneChangeReady=False, forkNow=False)
  for _ in range(5):
    dh.update(car, True, 1., nav_lane_intent=wire)
  assert dh.lane_change_state == log.LaneChangeState.off
  wire.targetLaneIndex = 0
  wire.spLaneChangeReady = wire.forkNow = True
  for _ in range(3):
    dh.update(car, True, 1., nav_lane_intent=wire, left_crossing_allowed=True, right_crossing_allowed=True)
  assert dh.lane_change_state == log.LaneChangeState.laneChangeStarting


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('driver', ['steeringPressed', 'brakePressed'])
def test_driver_override_blocks_even_a_ready_fork_packet(side, driver):
  dh = helper()
  car = car_state(vEgo=5., **{side + 'Blinker': True, driver: True})
  wire = SimpleNamespace(valid=True, signalRequested=True, targetLaneIndex=0, direction=side,
                         spLaneChangeReady=True, forkNow=True)
  for _ in range(5):
    dh.update(car, True, 1., nav_lane_intent=wire, left_crossing_allowed=True, right_crossing_allowed=True)
    assert dh.lane_change_state != log.LaneChangeState.laneChangeStarting


def test_fork_purpose_survives_distance_jitter_but_not_driver_override():
  plan = build_lane_plan(guidance('right'), SimpleNamespace(visibleLaneCount=1), healthy=True)
  observed = LaneTopologyInput(True, 1, 0, None, None, True, True)
  car = LaneVehicleInput(True, 5., right_blinker=True)
  coordinator = NavLaneIntentCoordinator()
  assert coordinator.update(plan, observed, car, now_ns=0).lane_change_ready
  outside_window = replace(plan, force_fork=False)
  assert coordinator.update(outside_window, observed, car, now_ns=50_000_000).lane_change_ready
  assert coordinator.fork_active
  assert not coordinator.update(plan, observed, replace(car, steering_pressed=True), now_ns=100_000_000).signal_requested


@pytest.mark.parametrize('model_path', ['selfdrive/modeld/modeld.py', 'sunnypilot/modeld_v2/modeld.py'])
@pytest.mark.parametrize('fork,signal,target,valid,speed,expected', [
  (True, True, 0, True, 5., True),
  (False, True, 0, True, 5., False),
  (True, False, 0, True, 5., False),
  (True, True, -1, True, 5., False),
  (True, True, 0, False, 5., False),
  (False, False, -1, False, 25., True),
])
def test_both_model_entries_use_lane_permission_for_low_speed_fork(model_path, fork, signal, target, valid, speed, expected):
  import ast
  from pathlib import Path
  source = Path(__file__).resolve().parents[3] / model_path
  tree = ast.parse(source.read_text())
  wanted = ('lane_change_entry', 'oem_solid_override', 'entry_safety_blocks')
  assignments = {node.targets[0].id: node for node in ast.walk(tree)
                 if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
                 and node.targets[0].id in wanted and not isinstance(node.value, ast.Tuple)}
  gate = SimpleNamespace(lane_change_safety_blocks=(False, False), safety_blocks=(True, True))
  scope = dict(v_ego=speed, LANE_CHANGE_SPEED_MIN=20 * .44704, oem_gate=gate,
               oem_permissions=(False, False), nav_lane_intent=SimpleNamespace(
                 forkNow=fork, signalRequested=signal, targetLaneIndex=target, valid=valid))
  for name in wanted:
    exec(compile(ast.Module(body=[assignments[name]], type_ignores=[]), str(source), 'exec'), scope)
  assert scope['lane_change_entry'] is expected
  assert scope['entry_safety_blocks'] == ((False, False) if expected else (True, True))
  gate.safety_blocks = (False, False)
  scope['oem_permissions'] = (True, True)
  exec(compile(ast.Module(body=[assignments['oem_solid_override']], type_ignores=[]), str(source), 'exec'), scope)
  assert scope['oem_solid_override'] == (expected, expected)


@pytest.mark.parametrize('road_class', [0, 6, 1, 7, -1])
@pytest.mark.parametrize('maneuver', ['mergeLeft', 'mergeRight', 'rampLeft', 'rampRight', 'exitLeft', 'exitRight'])
def test_reference_road_scope_limits_final_fork_without_disabling_approach(road_class, maneuver):
  nav = guidance('left', maneuver=maneuver, roadClass=road_class)
  topology = SimpleNamespace(visibleLaneCount=3)
  near = build_lane_plan(nav, topology, healthy=True)
  assert near.valid and near.heuristic
  assert near.force_fork == (road_class in (0, 6))
  nav.maneuverDistanceM = 1500.
  far = build_lane_plan(nav, topology, healthy=True)
  assert far.valid and far.heuristic and not far.force_fork
  assert far.edge_direction == near.edge_direction


@pytest.mark.parametrize('elevated_status', ['main', 'side'])
@pytest.mark.parametrize('maneuver', ['mergeLeft', 'mergeRight', 'rampLeft', 'rampRight', 'exitLeft', 'exitRight'])
def test_confirmed_elevated_road_qualifies_final_fork(elevated_status, maneuver):
  nav = guidance('left', maneuver=maneuver, roadClass=1, elevatedRoadStatus=elevated_status)
  plan = build_lane_plan(nav, SimpleNamespace(visibleLaneCount=3), healthy=True)
  assert plan.valid and plan.heuristic and plan.force_fork


def update_lane_from_navigation(coordinator, nav, plan, topology, vehicle, *, now_ns):
  return coordinator.update(
    plan, topology, vehicle, now_ns=now_ns,
    allow_new_lane_change=lane_alignment_may_start(nav, NavLaneIntent(), speed_mps=vehicle.speed_mps),
  )


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('visible_lane_count', [2, 3])
def test_active_fork_survives_zero_distance_with_unmapped_middle_recommendation(side, visible_lane_count):
  nav = guidance(side, lanes=[SimpleNamespace(index=i, recommended=i == 1) for i in range(3)])
  visual = SimpleNamespace(visibleLaneCount=visible_lane_count)
  topology = LaneTopologyInput(True, visible_lane_count, 1, True, True, True, True)
  direction = LaneIntentDirection.left if side == 'left' else LaneIntentDirection.right
  car = LaneVehicleInput(True, 20., **{side + '_blinker': True}, lane_change_direction=direction)
  coordinator = NavLaneIntentCoordinator()
  plan = build_lane_plan(nav, visual, healthy=True)
  ready = update_lane_from_navigation(coordinator, nav, plan, topology, replace(car, lane_change_state=ObservedLaneChangeState.pre), now_ns=0)
  assert plan.force_fork and ready.lane_change_ready
  started = update_lane_from_navigation(coordinator, nav, plan, topology, replace(car, lane_change_state=ObservedLaneChangeState.starting),
                               now_ns=50_000_000)
  assert started.signal_requested and coordinator.fork_active

  # Rebuild the actual plan: the middle recommendation has no absolute visual
  # mapping once the positive-distance final-fork entry window ends.
  nav.maneuverDistanceM = 0.
  zero_plan = build_lane_plan(nav, visual, healthy=True)
  assert not zero_plan.valid and not zero_plan.force_fork
  for now_ns, state in ((100_000_000, ObservedLaneChangeState.starting),
                        (150_000_000, ObservedLaneChangeState.finishing)):
    request = update_lane_from_navigation(coordinator, nav, zero_plan, topology, replace(car, lane_change_state=state), now_ns=now_ns)
    assert request.signal_requested and coordinator.fork_active
    assert request.direction == direction and request.request_id == ready.request_id
  complete = update_lane_from_navigation(coordinator, nav, zero_plan, topology, replace(car, lane_change_state=ObservedLaneChangeState.pre),
                                now_ns=200_000_000)
  assert complete.reason == 'forkCompleted' and not complete.signal_requested and not coordinator.fork_active
  for now_ns in (250_000_000, 5_000_000_000):
    repeated = update_lane_from_navigation(coordinator, nav, plan, topology, car, now_ns=now_ns)
    assert repeated.reason == 'forkCompleted' and not repeated.signal_requested

  # A zero-distance, unmapped plan may preserve an already selected fork; it
  # cannot authorize a new request by itself.
  fresh = update_lane_from_navigation(NavLaneIntentCoordinator(), nav, zero_plan, topology, car, now_ns=0)
  assert not fresh.signal_requested and not fresh.lane_change_ready


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('gate', ['permission', 'physical_lamp'])
@pytest.mark.parametrize('initial_count,visible_lane_count,ego_index', [(2, 2, 1), (3, 3, 1), (3, 2, 0)])
def test_zero_distance_fork_hold_before_start_still_requires_crossing_and_lamp(side, gate, initial_count,
                                                                           visible_lane_count, ego_index):
  nav = guidance(side, lanes=[SimpleNamespace(index=i, recommended=i == 1) for i in range(3)])
  visual = SimpleNamespace(visibleLaneCount=initial_count)
  topology = LaneTopologyInput(True, initial_count, 1, True, True, True, True)
  direction = LaneIntentDirection.left if side == 'left' else LaneIntentDirection.right
  car = LaneVehicleInput(True, 20., **{side + '_blinker': True}, lane_change_direction=direction,
                         lane_change_state=ObservedLaneChangeState.pre)
  coordinator = NavLaneIntentCoordinator()
  ready = update_lane_from_navigation(coordinator, nav, build_lane_plan(nav, visual, healthy=True), topology, car, now_ns=0)
  assert ready.lane_change_ready
  nav.maneuverDistanceM = 0.
  visual.visibleLaneCount = visible_lane_count
  topology = replace(topology, visible_lane_count=visible_lane_count, ego_lane_index=ego_index)
  zero_plan = build_lane_plan(nav, visual, healthy=True)
  blocked_topology = replace(topology, **{side + '_crossing_allowed': False}) if gate == 'permission' else topology
  blocked_car = replace(car, **{side + '_blinker': False}) if gate == 'physical_lamp' else car
  held = update_lane_from_navigation(coordinator, nav, zero_plan, blocked_topology, blocked_car, now_ns=50_000_000)
  assert held.signal_requested and not held.lane_change_ready and coordinator.fork_active
  assert held.direction == direction and held.request_id == ready.request_id
  recovered = update_lane_from_navigation(coordinator, nav, zero_plan, topology, car, now_ns=100_000_000)
  assert recovered.lane_change_ready and recovered.request_id == ready.request_id
  nav.valid = False
  lost = update_lane_from_navigation(coordinator, nav, build_lane_plan(nav, visual, healthy=True), topology, car, now_ns=150_000_000)
  assert not lost.signal_requested and not coordinator.fork_active


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('interrupt', [
  'invalid', 'stale', 'unhealthy', 'disabled', 'lane_change_disabled',
  'steering', 'brake', 'lateral', 'session', 'revision', 'event',
])
def test_zero_distance_fork_hold_preserves_source_route_and_driver_cancellation(side, interrupt):
  nav = guidance(side, lanes=[SimpleNamespace(index=i, recommended=i == 1) for i in range(3)])
  visual = SimpleNamespace(visibleLaneCount=3)
  topology = LaneTopologyInput(True, 3, 1, True, True, True, True)
  direction = LaneIntentDirection.left if side == 'left' else LaneIntentDirection.right
  car = LaneVehicleInput(True, 20., **{side + '_blinker': True}, lane_change_direction=direction)
  coordinator = NavLaneIntentCoordinator()
  plan = build_lane_plan(nav, visual, healthy=True)
  assert update_lane_from_navigation(coordinator, nav, plan, topology, car, now_ns=0).lane_change_ready
  car = replace(car, lane_change_state=ObservedLaneChangeState.starting)
  assert update_lane_from_navigation(coordinator, nav, plan, topology, car, now_ns=50_000_000).signal_requested
  nav.maneuverDistanceM = 0.
  if interrupt == 'invalid':
    nav.valid = False
  elif interrupt == 'stale':
    nav.stale = True
  elif interrupt == 'session':
    nav.sessionId = 'different-route'
  elif interrupt == 'revision':
    nav.routeRevision += 1
  elif interrupt == 'event':
    nav.maneuverEventId += 1
  elif interrupt == 'steering':
    car = replace(car, steering_pressed=True)
  elif interrupt == 'brake':
    car = replace(car, brake_pressed=True)
  elif interrupt == 'lateral':
    car = replace(car, lateral_active=False)
  settings = NavAssistSettings(enabled=interrupt != 'disabled', lane_change_enabled=interrupt != 'lane_change_disabled')
  interrupted = build_lane_plan(nav, visual, healthy=interrupt != 'unhealthy', settings=settings)
  request = update_lane_from_navigation(coordinator, nav, interrupted, topology, car, now_ns=100_000_000)
  assert not request.signal_requested and not request.lane_change_ready and not coordinator.fork_active
  assert request.reason != 'forkCompleted'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_completed_approach_budget_does_not_consume_the_single_final_fork(side):
  nav = guidance(side, distance=100.)
  visual = SimpleNamespace(visibleLaneCount=3)
  topology = LaneTopologyInput(True, 3, 1, True, True, True, True)
  direction = LaneIntentDirection.left if side == 'left' else LaneIntentDirection.right
  car = LaneVehicleInput(True, 20., **{side + '_blinker': True}, lane_change_direction=direction)
  coordinator = NavLaneIntentCoordinator(max_changes=1)
  approach = build_lane_plan(nav, visual, healthy=True)
  assert approach.valid and not approach.force_fork
  for now_ns, state in ((0, ObservedLaneChangeState.off),
                        (500_000_000, ObservedLaneChangeState.off),
                        (1_000_000_000, ObservedLaneChangeState.pre),
                        (1_050_000_000, ObservedLaneChangeState.starting),
                        (1_100_000_000, ObservedLaneChangeState.finishing),
                        (1_150_000_000, ObservedLaneChangeState.pre),
                        (2_000_000_000, ObservedLaneChangeState.off)):
    request = update_lane_from_navigation(coordinator, nav, approach, topology, replace(car, lane_change_state=state), now_ns=now_ns)
    if state == ObservedLaneChangeState.starting:
      assert request.signal_requested and request.lane_change_ready
    elif now_ns == 1_150_000_000:
      assert request.reason == 'laneChangeObserved'
    elif now_ns == 2_000_000_000:
      assert request.reason == 'laneChangeComplete'
  limited = update_lane_from_navigation(coordinator, nav, approach, topology, car, now_ns=2_050_000_000)
  assert limited.reason == 'heuristicChangeLimit' and not limited.signal_requested

  nav.maneuverDistanceM = 50.
  fork = build_lane_plan(nav, visual, healthy=True)
  ready = update_lane_from_navigation(coordinator, nav, fork, topology, car, now_ns=2_100_000_000)
  assert fork.force_fork and ready.signal_requested and ready.lane_change_ready and coordinator.fork_active
  for now_ns, state in ((2_150_000_000, ObservedLaneChangeState.starting),
                        (2_200_000_000, ObservedLaneChangeState.finishing)):
    request = update_lane_from_navigation(coordinator, nav, fork, topology, replace(car, lane_change_state=state), now_ns=now_ns)
    assert request.signal_requested and request.direction == direction and request.request_id == ready.request_id
  complete = update_lane_from_navigation(coordinator, nav, fork, topology, replace(car, lane_change_state=ObservedLaneChangeState.pre),
                                now_ns=2_250_000_000)
  assert complete.reason == 'forkCompleted' and not complete.signal_requested and not coordinator.fork_active
  for now_ns, plan in ((2_300_000_000, fork), (3_000_000_000, approach), (4_000_000_000, fork)):
    repeated = update_lane_from_navigation(coordinator, nav, plan, topology, car, now_ns=now_ns)
    assert repeated.reason == 'forkCompleted' and not repeated.signal_requested


@pytest.mark.parametrize('side', ['left', 'right'])
def test_absolute_approach_upgrades_to_one_relative_fork_model_cycle(side):
  recommended = 0 if side == 'left' else 2
  nav = guidance(side, distance=100., lanes=[SimpleNamespace(index=i, recommended=i == recommended) for i in range(3)])
  visual = SimpleNamespace(visibleLaneCount=3)
  topology = LaneTopologyInput(True, 3, 1, True, True, True, True)
  direction = LaneIntentDirection.left if side == 'left' else LaneIntentDirection.right
  car = LaneVehicleInput(True, 20., **{side + '_blinker': True}, lane_change_direction=direction,
                         lane_change_state=ObservedLaneChangeState.pre)
  coordinator = NavLaneIntentCoordinator()
  approach = build_lane_plan(nav, visual, healthy=True, amap_ego_index=1)
  assert approach.valid and not approach.heuristic and not approach.force_fork
  update_lane_from_navigation(coordinator, nav, approach, topology, car, now_ns=0)
  ready = update_lane_from_navigation(coordinator, nav, approach, topology, car, now_ns=500_000_000)
  assert ready.lane_change_ready
  nav.maneuverDistanceM = 50.
  fork = build_lane_plan(nav, visual, healthy=True, amap_ego_index=1)
  upgraded = update_lane_from_navigation(coordinator, nav, fork, topology, car, now_ns=550_000_000)
  assert upgraded.lane_change_ready and coordinator.fork_active
  assert upgraded.direction == direction and upgraded.request_id == ready.request_id
  assert coordinator.hold_active_fork(fork, topology).heuristic
  for now_ns, state in ((600_000_000, ObservedLaneChangeState.starting),
                        (650_000_000, ObservedLaneChangeState.finishing)):
    request = update_lane_from_navigation(coordinator, nav, fork, topology, replace(car, lane_change_state=state), now_ns=now_ns)
    assert request.signal_requested and request.direction == direction and request.request_id == ready.request_id
  # The split follows the model cycle; an absolute visual lane-index shift is
  # not required after the ready approach has become a final relative fork.
  complete = update_lane_from_navigation(coordinator, nav, fork, topology, car, now_ns=700_000_000)
  assert complete.reason == 'forkCompleted' and not complete.signal_requested
  repeated = update_lane_from_navigation(coordinator, nav, fork, topology, car, now_ns=750_000_000)
  assert repeated.reason == 'forkCompleted' and not repeated.signal_requested


@pytest.mark.parametrize('side', ['left', 'right'])
def test_zero_distance_cancelled_fork_is_not_replaced_by_turn_approach_in_main(side):
  import ast
  from pathlib import Path

  # Execute the actual main-loop arbitration so a coordinator-only pass cannot
  # hide a downstream turn-only lamp replacing a terminal fork response.
  source = Path(__file__).resolve().parents[1] / 'nav_lane_intentd.py'
  tree = ast.parse(source.read_text(encoding='utf-8'))
  arbitration = [node for node in ast.walk(tree) if isinstance(node, ast.If) and any(
    isinstance(item, ast.Attribute) and item.attr == 'signal_requested'
    and isinstance(item.value, ast.Name) and item.value.id == 'lane_intent'
    for item in ast.walk(node.test)
  )]
  assert len(arbitration) == 1
  arbitration_code = compile(ast.Module(body=arbitration, type_ignores=[]), str(source), 'exec')
  nav = guidance(side, lanes=[SimpleNamespace(index=i, recommended=i == 1) for i in range(3)])
  visual = SimpleNamespace(visibleLaneCount=2)
  topology = LaneTopologyInput(True, 2, 1, True, True, True, True)
  direction = LaneIntentDirection.left if side == 'left' else LaneIntentDirection.right
  coordinator = NavLaneIntentCoordinator()
  turn_coordinator = NavTurnSignalCoordinator()
  car = LaneVehicleInput(True, 20., **{side + '_blinker': True}, lane_change_direction=direction)
  for now_ns, distance, state, observed_direction in (
    (0, 50., ObservedLaneChangeState.pre, direction),
    (50_000_000, 50., ObservedLaneChangeState.starting, direction),
    (100_000_000, 0., ObservedLaneChangeState.finishing, LaneIntentDirection.none),
    (150_000_000, 0., ObservedLaneChangeState.off, LaneIntentDirection.none),
  ):
    nav.maneuverDistanceM = distance
    plan = coordinator.hold_active_fork(build_lane_plan(nav, visual, healthy=True), topology)
    vehicle = replace(car, lane_change_state=state, lane_change_direction=observed_direction)
    turn_intent = turn_coordinator.update(
      NavTurnPlan(True, nav.sessionId, nav.routeRevision, nav.maneuverEventId, nav.maneuver, distance),
      speed_mps=vehicle.speed_mps, now_ns=now_ns,
    )
    assert turn_intent.signal_requested
    lane_intent = coordinator.update(
      plan, topology, vehicle, now_ns=now_ns,
      allow_new_lane_change=lane_alignment_may_start(nav, turn_intent, speed_mps=vehicle.speed_mps),
    )
    scope = dict(lane_intent=apply_oem_signal_gate(lane_intent, None, coordinator._phase), plan=plan, coordinator=coordinator,
                 settings=NavAssistSettings(), turn_intent=turn_intent, gate_reason=None,
                 efficiency=SimpleNamespace(reason='unused'), NavLaneIntent=NavLaneIntent, replace=replace)
    exec(arbitration_code, scope)
    if distance > 0.:
      assert scope['intent'].signal_requested
    else:
      assert scope['intent'].reason == 'laneChangeCancelled'
      assert not scope['intent'].signal_requested and not scope['intent'].lane_change_ready

  # A valid new navigation event releases the previous fork terminal even
  # when its middle-lane recommendation cannot map to the visual window.
  nav.maneuverEventId += 1
  nav.maneuverDistanceM = 100.
  for now_ns in (200_000_000, 250_000_000):
    plan = coordinator.hold_active_fork(build_lane_plan(nav, visual, healthy=True), topology)
    assert not plan.valid and not plan.force_fork
    turn_intent = turn_coordinator.update(
      NavTurnPlan(True, nav.sessionId, nav.routeRevision, nav.maneuverEventId, nav.maneuver, nav.maneuverDistanceM),
      speed_mps=car.speed_mps, now_ns=now_ns,
    )
    lane_intent = coordinator.update(
      plan, topology, car, now_ns=now_ns,
      allow_new_lane_change=lane_alignment_may_start(nav, turn_intent, speed_mps=car.speed_mps),
    )
    assert not coordinator.fork_terminal
    scope.update(plan=plan, turn_intent=turn_intent, lane_intent=lane_intent)
    exec(arbitration_code, scope)
    if now_ns == 250_000_000:
      assert scope['intent'].reason == 'turnApproach' and scope['intent'].signal_requested
      assert not scope['intent'].lane_change_ready
