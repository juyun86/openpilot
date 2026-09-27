from dataclasses import replace

from openpilot.sunnypilot.navassist.lane_intent import (
  LaneIntentDirection,
  LaneTopologyInput,
  LaneVehicleInput,
  NavLaneIntentCoordinator,
  NavLanePlan,
  NavTurnPlan,
  NavTurnSignalCoordinator,
  ObservedLaneChangeState,
)


def plan(*, recommended=(0,), lane_count=3, valid=True, session="session-a", revision=1, event=1):
  return NavLanePlan(valid, session, revision, event, lane_count, tuple(recommended))


def topology(*, ego=1, count=3, left=True, right=True, left_cross=False, right_cross=False, valid=True):
  return LaneTopologyInput(valid, count, ego, left, right, left_cross, right_cross)


def vehicle(*, bsm_left=False, bsm_right=False, state=ObservedLaneChangeState.off,
            direction=LaneIntentDirection.none, lat=True, left_blinker=False, right_blinker=False):
  return LaneVehicleInput(lat, 15.0, bsm_left, bsm_right, left_blinker, right_blinker, lane_change_state=state,
                          lane_change_direction=direction)


def turn_plan(*, valid=True, maneuver="turnLeft", distance=100.0, session="session-a", revision=1, event=11,
              source_interrupted=False):
  return NavTurnPlan(valid, session, revision, event, maneuver, distance, source_interrupted)


def test_source_zero_event_keeps_existing_lamp_identity_until_source_recovers():
  coordinator = NavTurnSignalCoordinator()
  started = coordinator.update(turn_plan(distance=38), speed_mps=4.5, now_ns=0)
  for now in (100_000_000, 1_600_000_000, 3_900_000_000):
    held = coordinator.update(turn_plan(valid=False, event=0, distance=17, source_interrupted=True),
                              speed_mps=0.3, now_ns=now)
    assert held.signal_requested and held.request_id == started.request_id == 11
    assert held.direction == LaneIntentDirection.left and held.target_lane_index == -1
    assert not held.lane_change_ready
  recovered = coordinator.update(turn_plan(distance=17), speed_mps=0.3, now_ns=4_000_000_000)
  assert recovered.signal_requested and recovered.request_id == 11


def test_source_zero_event_cannot_start_a_lamp_and_expires_without_fresh_source():
  gap = turn_plan(valid=False, event=0, source_interrupted=True)
  coordinator = NavTurnSignalCoordinator()
  assert not coordinator.update(gap, speed_mps=10, now_ns=0).signal_requested
  coordinator.update(turn_plan(), speed_mps=10, now_ns=1)
  coordinator.update(gap, speed_mps=10, now_ns=100)
  expired = coordinator.update(gap, speed_mps=10, now_ns=101 + coordinator.SOURCE_GAP_GRACE_NS)
  assert not expired.signal_requested
  assert not coordinator.update(gap, speed_mps=10, now_ns=102 + coordinator.SOURCE_GAP_GRACE_NS).signal_requested


def test_source_hold_does_not_mask_stop_route_or_direction_changes():
  for gap in (turn_plan(valid=False, event=0),
              turn_plan(valid=False, event=0, source_interrupted=True, revision=2),
              turn_plan(valid=False, event=0, source_interrupted=True, session="new-session"),
              turn_plan(valid=False, event=0, source_interrupted=True, maneuver="turnRight")):
    coordinator = NavTurnSignalCoordinator()
    coordinator.update(turn_plan(), speed_mps=10, now_ns=0)
    assert not coordinator.update(gap, speed_mps=10, now_ns=1, turn_geometry_active=True).signal_requested


def test_navigation_turn_signal_starts_before_turn_without_a_lane_target():
  coordinator = NavTurnSignalCoordinator()

  intent = coordinator.update(turn_plan(), speed_mps=15.0, now_ns=0)

  assert intent.signal_requested
  assert not intent.lane_change_ready
  assert intent.direction == LaneIntentDirection.left
  assert intent.target_lane_index == -1
  assert intent.request_id == 11
  assert intent.reason == "turnApproach"


def test_heuristic_extreme_lane_waits_for_stable_neighbor_then_signals_one_lane_at_a_time():
  coordinator = NavLaneIntentCoordinator()
  heuristic = NavLanePlan(True, "session-a", 1, 7, 3, (0,), heuristic=True)

  first = coordinator.update(heuristic, topology(ego=2), vehicle(), now_ns=0)
  stable_neighbor = coordinator.update(heuristic, topology(ego=2), vehicle(), now_ns=500_000_000)
  intent = coordinator.update(heuristic, topology(ego=2, left_cross=True), vehicle(), now_ns=1_000_000_000)

  assert not first.signal_requested and first.reason == "heuristicStabilizingNeighbor"
  assert not stable_neighbor.signal_requested and stable_neighbor.reason == "heuristicStabilizingLaneAlignment"
  assert intent.signal_requested and not intent.lane_change_ready
  assert intent.direction == LaneIntentDirection.left
  assert intent.target_lane_index == 1
  assert intent.reason == "heuristicSignaling"


def test_fork_now_can_signal_without_a_visible_neighbor_but_keeps_lane_change_state_machine():
  coordinator = NavLaneIntentCoordinator()
  forced = NavLanePlan(
    True, "session-a", 1, 7, 1, (0,), heuristic=True,
    edge_direction=LaneIntentDirection.right, force_fork=True,
    allow_unknown_crossing=True, ignore_solid_boundary=True,
  )

  intent = coordinator.update(
    forced, topology(ego=0, count=1, left=False, right=False, right_cross=True), vehicle(), now_ns=0,
  )

  assert intent.signal_requested
  assert intent.direction == LaneIntentDirection.right
  assert intent.target_lane_index == 1
  assert not intent.lane_change_ready


def test_navigation_turn_signal_supports_right_turns_and_waits_until_lookahead_window():
  coordinator = NavTurnSignalCoordinator()

  early = coordinator.update(turn_plan(maneuver="turnRight", distance=200.0), speed_mps=15.0, now_ns=0)
  active = coordinator.update(turn_plan(maneuver="turnRight", distance=130.0), speed_mps=15.0, now_ns=1_000_000_000)

  assert not early.signal_requested
  assert active.signal_requested
  assert active.direction == LaneIntentDirection.right


def test_navigation_merge_requests_directional_lamp_without_authorizing_lane_change():
  coordinator = NavTurnSignalCoordinator()

  intent = coordinator.update(turn_plan(maneuver="mergeRight", distance=80.0), speed_mps=10.0, now_ns=0)

  assert intent.signal_requested
  assert intent.direction == LaneIntentDirection.right
  assert not intent.lane_change_ready
  assert intent.target_lane_index == -1


def test_navigation_turn_signal_stays_on_through_zero_distance_then_cancels_on_event_change():
  coordinator = NavTurnSignalCoordinator()
  coordinator.update(turn_plan(distance=50.0), speed_mps=10.0, now_ns=0)

  at_turn = coordinator.update(turn_plan(distance=0.0), speed_mps=8.0, now_ns=5_000_000_000)
  changed = coordinator.update(turn_plan(maneuver="straight", distance=300.0, event=12), speed_mps=8.0,
                               now_ns=6_000_000_000)

  assert at_turn.signal_requested
  assert not changed.signal_requested


def test_navigation_turn_signal_waits_for_sp_turn_geometry_to_clear_after_event_change():
  coordinator = NavTurnSignalCoordinator()
  coordinator.update(turn_plan(distance=80.0), speed_mps=12.0, now_ns=0)

  changed_while_turning = coordinator.update(
    turn_plan(maneuver="straight", distance=500.0, event=12), speed_mps=6.0,
    now_ns=5_000_000_000, turn_geometry_active=True,
  )
  first_clear = coordinator.update(
    turn_plan(maneuver="straight", distance=490.0, event=12), speed_mps=6.0,
    now_ns=6_000_000_000, turn_geometry_active=False,
  )
  completed = coordinator.update(
    turn_plan(maneuver="straight", distance=480.0, event=12), speed_mps=6.0,
    now_ns=6_500_000_001, turn_geometry_active=False,
  )

  assert changed_while_turning.signal_requested and changed_while_turning.reason == "turnCompletion"
  assert first_clear.signal_requested and first_clear.reason == "turnCompletion"
  assert not completed.signal_requested


def test_navigation_turn_signal_does_not_survive_a_route_revision_change():
  coordinator = NavTurnSignalCoordinator()
  coordinator.update(turn_plan(), speed_mps=10.0, now_ns=0)

  changed = coordinator.update(
    turn_plan(revision=2, event=12), speed_mps=6.0, now_ns=1_000_000_000,
    turn_geometry_active=True,
  )

  assert not changed.signal_requested and changed.reason == "turnChanged"


def test_navigation_turn_signal_holds_through_recorded_snapshot_gaps_until_maneuver_changes():
  coordinator = NavTurnSignalCoordinator()
  started = coordinator.update(turn_plan(maneuver="turnRight", distance=120.0), speed_mps=13.5, now_ns=0)
  first_gap = coordinator.update(
    turn_plan(valid=False, maneuver="turnRight", distance=11.0), speed_mps=2.0, now_ns=17_946_000_000,
  )
  recovered = coordinator.update(
    turn_plan(maneuver="turnRight", distance=10.0), speed_mps=2.0, now_ns=18_046_000_000,
  )
  second_gap = coordinator.update(
    turn_plan(valid=False, maneuver="turnRight", distance=8.0), speed_mps=2.0, now_ns=19_747_000_000,
  )
  at_turn = coordinator.update(
    turn_plan(maneuver="turnRight", distance=0.0), speed_mps=1.0, now_ns=24_046_000_000,
  )
  completed = coordinator.update(
    turn_plan(maneuver="straight", distance=4_964.0, event=12), speed_mps=6.0, now_ns=29_446_000_000,
  )

  assert started.signal_requested
  assert first_gap.signal_requested and first_gap.reason == "turnApproachGrace"
  assert recovered.signal_requested
  assert second_gap.signal_requested and second_gap.reason == "turnApproachGrace"
  assert at_turn.signal_requested
  assert not completed.signal_requested


def test_navigation_turn_signal_retains_a_bounded_hard_timeout():
  coordinator = NavTurnSignalCoordinator()
  coordinator.update(turn_plan(), speed_mps=15.0, now_ns=0)

  timed_out = coordinator.update(
    turn_plan(), speed_mps=15.0, now_ns=NavTurnSignalCoordinator.SIGNAL_TIMEOUT_NS + 1,
  )
  assert not timed_out.signal_requested


def test_navigation_turn_signal_drops_after_the_bounded_plan_gap_grace():
  coordinator = NavTurnSignalCoordinator()
  coordinator.update(turn_plan(), speed_mps=15.0, now_ns=0)
  first_gap = coordinator.update(turn_plan(valid=False), speed_mps=15.0, now_ns=1_000_000_000)
  expired = coordinator.update(
    turn_plan(valid=False), speed_mps=15.0,
    now_ns=1_000_000_000 + NavTurnSignalCoordinator.PLAN_GAP_GRACE_NS + 1,
  )

  assert first_gap.signal_requested
  assert not expired.signal_requested and expired.reason == "turnUnavailable"


def test_navigation_signals_at_solid_line_then_authorizes_on_dashed_and_physical_lamp():
  coordinator = NavLaneIntentCoordinator()
  first_mismatch = coordinator.update(plan(), topology(), vehicle(), now_ns=0)
  assert not first_mismatch.signal_requested and not first_mismatch.lane_change_ready
  assert first_mismatch.reason == "stabilizingLaneAlignment"
  waiting = coordinator.update(plan(), topology(), vehicle(), now_ns=500_000_000)
  assert waiting.signal_requested and not waiting.lane_change_ready and waiting.reason == "signaling"
  first_dashed = coordinator.update(plan(), topology(left_cross=True), vehicle(left_blinker=True), now_ns=600_000_000)
  assert first_dashed.signal_requested and first_dashed.lane_change_ready
  authorized = coordinator.update(plan(), topology(left_cross=True), vehicle(left_blinker=True), now_ns=900_000_000)
  assert authorized.signal_requested and authorized.lane_change_ready
  assert authorized.direction == LaneIntentDirection.left


def test_observation_recovery_uses_existing_alignment_wait():
  coordinator = NavLaneIntentCoordinator()
  current = plan()
  observed = topology(left_cross=True)
  coordinator.update(current, observed, vehicle(), now_ns=0)
  coordinator.update(current, observed, vehicle(), now_ns=500_000_000)
  assert coordinator.update(current, observed, vehicle(left_blinker=True), now_ns=600_000_000).lane_change_ready
  missing = replace(observed, valid_for_control=False)
  coordinator.update(current, missing, vehicle(left_blinker=True), now_ns=1_600_000_000)
  paused = coordinator.update(current, missing, vehicle(left_blinker=True), now_ns=2_650_000_000)
  assert paused.reason == 'neighborObservationPaused' and not paused.signal_requested
  coordinator.update(current, observed, vehicle(), now_ns=2_700_000_000)
  resumed = coordinator.update(current, observed, vehicle(), now_ns=3_200_000_000)
  assert resumed.signal_requested and not resumed.lane_change_ready
  assert coordinator.update(current, observed, vehicle(left_blinker=True), now_ns=3_250_000_000).lane_change_ready


def test_blindspot_allows_intent_signal_but_delays_lane_change_readiness():
  coordinator = NavLaneIntentCoordinator()
  coordinator.update(plan(recommended=(2,)), topology(right_cross=True), vehicle(), now_ns=0)
  coordinator.update(plan(recommended=(2,)), topology(right_cross=True),
                     vehicle(bsm_right=True, right_blinker=True), now_ns=500_000_000)
  blocked = coordinator.update(plan(recommended=(2,)), topology(right_cross=True),
                               vehicle(bsm_right=True, right_blinker=True), now_ns=900_000_000)
  assert blocked.signal_requested and not blocked.lane_change_ready and blocked.reason == "signaling"

  signaling = coordinator.update(
    plan(recommended=(2,)), topology(right_cross=True), vehicle(right_blinker=True), now_ns=1_000_000_000,
  )
  assert signaling.signal_requested and signaling.lane_change_ready


def test_software_signal_request_never_authorizes_without_physical_blinker_feedback():
  coordinator = NavLaneIntentCoordinator()
  coordinator.update(plan(), topology(left_cross=True), vehicle(), now_ns=0)
  coordinator.update(plan(), topology(left_cross=True), vehicle(), now_ns=500_000_000)
  waiting = coordinator.update(plan(), topology(left_cross=True), vehicle(), now_ns=1_000_000_000)
  assert waiting.signal_requested and not waiting.lane_change_ready


def test_lane_count_mismatch_unknown_topology_and_no_neighbor_fail_closed():
  for observed in (
    topology(count=2),
    topology(valid=False),
    topology(left=False),
  ):
    coordinator = NavLaneIntentCoordinator()
    coordinator.update(plan(), observed, vehicle(), now_ns=0)
    assert not coordinator.update(plan(), observed, vehicle(), now_ns=1_000_000_000).signal_requested


def test_one_lane_change_completes_before_another_request_is_considered():
  coordinator = NavLaneIntentCoordinator()
  coordinator.update(plan(recommended=(0,)), topology(ego=2, left_cross=True), vehicle(), now_ns=0)
  coordinator.update(plan(recommended=(0,)), topology(ego=2, left_cross=True), vehicle(), now_ns=500_000_000)
  coordinator.update(plan(recommended=(0,)), topology(ego=2, left_cross=True), vehicle(left_blinker=True), now_ns=800_000_000)
  changing = coordinator.update(
    plan(recommended=(0,)), topology(ego=2, left_cross=True),
    vehicle(state=ObservedLaneChangeState.starting, direction=LaneIntentDirection.left, left_blinker=True), now_ns=900_000_000,
  )
  assert changing.signal_requested and changing.target_lane_index == 1
  observed = coordinator.update(
    plan(recommended=(0,)), topology(ego=1, left_cross=True),
    vehicle(state=ObservedLaneChangeState.pre, direction=LaneIntentDirection.left, left_blinker=True), now_ns=1_000_000_000,
  )
  assert observed.signal_requested
  finishing = coordinator.update(
    plan(recommended=(0,)), topology(ego=1, left_cross=True),
    vehicle(state=ObservedLaneChangeState.pre, direction=LaneIntentDirection.left, left_blinker=True), now_ns=1_500_000_000,
  )
  assert not finishing.signal_requested and finishing.reason == "laneChangeObserved"
  complete = coordinator.update(plan(recommended=(0,)), topology(ego=1, left_cross=True), vehicle(),
                                now_ns=2_300_000_000)
  assert not complete.signal_requested and complete.reason == "laneChangeComplete"


def test_relative_extreme_lane_change_ends_unconfirmed_when_visual_index_recenters():
  coordinator = NavLaneIntentCoordinator()
  heuristic = NavLanePlan(True, "session-a", 1, 7, 3, (0,), heuristic=True)
  coordinator.update(heuristic, topology(ego=1, left_cross=True), vehicle(), now_ns=0)
  coordinator.update(heuristic, topology(ego=1, left_cross=True), vehicle(), now_ns=500_000_000)
  coordinator.update(heuristic, topology(ego=1, left_cross=True), vehicle(left_blinker=True), now_ns=1_000_000_000)
  coordinator.update(heuristic, topology(ego=1, left_cross=True), vehicle(left_blinker=True), now_ns=1_300_000_000)
  coordinator.update(
    heuristic, topology(ego=1, left_cross=True),
    vehicle(state=ObservedLaneChangeState.starting, direction=LaneIntentDirection.left, left_blinker=True),
    now_ns=1_400_000_000,
  )

  completing = coordinator.update(
    heuristic, topology(ego=1, left_cross=True),
    vehicle(state=ObservedLaneChangeState.pre, direction=LaneIntentDirection.left, left_blinker=True),
    now_ns=1_500_000_000,
  )
  completed = coordinator.update(
    heuristic, topology(ego=1, left_cross=True),
    vehicle(state=ObservedLaneChangeState.pre, direction=LaneIntentDirection.left, left_blinker=True),
    now_ns=2_000_000_000,
  )

  assert not completing.signal_requested and completing.reason == "laneChangeCompletionUnconfirmed"
  assert not completed.signal_requested and completed.reason == "laneChangeCompletionUnconfirmed"
  assert coordinator._relative_consistency._completed_changes == 0


def relative_change_started(direction):
  coordinator = NavLaneIntentCoordinator()
  left = direction == LaneIntentDirection.left
  current_plan = NavLanePlan(True, "session-a", 1, 7, 3, (0 if left else 2,),
                             heuristic=True, edge_direction=direction)
  observed = topology(ego=1, left_cross=True, right_cross=True)
  car = replace(vehicle(left_blinker=left), right_blinker=not left, speed_mps=25.0)
  for now_ns in (1_000_000_000, 1_600_000_000, 2_200_000_000, 2_600_000_000):
    intent = coordinator.update(current_plan, observed, car, now_ns=now_ns)
  assert intent.lane_change_ready
  starting = replace(car, lane_change_state=ObservedLaneChangeState.starting, lane_change_direction=direction)
  coordinator.update(current_plan, observed, starting, now_ns=2_700_000_000)
  return coordinator, current_plan, observed, starting


def test_relative_cycle_cannot_prove_completion_even_when_local_index_changes():
  for direction in (LaneIntentDirection.left, LaneIntentDirection.right):
    for state in (ObservedLaneChangeState.pre, ObservedLaneChangeState.off):
      for ego_index in (0, 1, 2):
        coordinator, current_plan, observed, car = relative_change_started(direction)
        ended = replace(car, lane_change_state=state)
        local = replace(observed, ego_lane_index=ego_index)
        waiting = coordinator.update(current_plan, local, ended, now_ns=3_300_000_000)
        assert not waiting.signal_requested and waiting.reason == "laneChangeCompletionUnconfirmed"
        result = coordinator.update(current_plan, local, ended, now_ns=3_900_000_000)
        assert result.reason == "laneChangeCompletionUnconfirmed"
        assert not result.signal_requested and not result.lane_change_ready
        assert coordinator._relative_consistency._completed_changes == 0


def test_terminal_relative_event_survives_invalid_inputs_and_revision_changes():
  for direction in (LaneIntentDirection.left, LaneIntentDirection.right):
    for cancelled in (False, True):
      coordinator, current_plan, observed, car = relative_change_started(direction)
      if cancelled:
        ended = replace(car, lane_change_state=ObservedLaneChangeState.finishing,
                        lane_change_direction=LaneIntentDirection.none)
        expected = "laneChangeCancelled"
      else:
        ended = replace(car, lane_change_state=ObservedLaneChangeState.pre)
        expected = "laneChangeCompletionUnconfirmed"
      for now_ns in (3_300_000_000, 3_900_000_000):
        result = coordinator.update(current_plan, observed, ended, now_ns=now_ns)
      assert result.reason == expected
      idle_car = replace(car, left_blinker=False, right_blinker=False,
                         lane_change_state=ObservedLaneChangeState.off, lane_change_direction=LaneIntentDirection.none)
      gaps = (
        replace(current_plan, valid=False, maneuver_event_id=0),
        replace(current_plan, valid=False, session_id="", route_revision=0, maneuver_event_id=0),
        replace(current_plan, valid=False, session_id="other", maneuver_event_id=99),
        replace(current_plan, valid=True, maneuver_event_id=0),
        replace(current_plan, valid=True, session_id=""),
        replace(current_plan, route_revision=2),
        current_plan,
      )
      for index, gap in enumerate(gaps):
        result = coordinator.update(gap, observed, idle_car, now_ns=(5 + index) * 1_000_000_000)
        assert result.reason == expected
        assert not result.signal_requested and not result.lane_change_ready
      assert coordinator._relative_consistency._completed_changes == 0


def test_terminal_relative_event_releases_for_valid_new_maneuver_or_session():
  for cancelled in (False, True):
    for new_session in (False, True):
      coordinator, current_plan, observed, car = relative_change_started(LaneIntentDirection.left)
      ended = replace(car, lane_change_state=ObservedLaneChangeState.finishing if cancelled else ObservedLaneChangeState.pre,
                      lane_change_direction=LaneIntentDirection.none if cancelled else LaneIntentDirection.left)
      for now_ns in (3_300_000_000, 3_900_000_000):
        coordinator.update(current_plan, observed, ended, now_ns=now_ns)
      fresh = replace(current_plan, session_id="session-b") if new_session else replace(current_plan, maneuver_event_id=8)
      car = replace(car, lane_change_state=ObservedLaneChangeState.off, lane_change_direction=LaneIntentDirection.none)
      for now_ns in (5_000_000_000, 5_600_000_000, 6_200_000_000, 6_600_000_000):
        result = coordinator.update(fresh, observed, car, now_ns=now_ns)
      assert result.signal_requested and result.lane_change_ready


def test_relative_completion_wait_resets_on_model_cycle_or_observation_gap():
  for gap_is_topology in (False, True):
    coordinator, current_plan, observed, car = relative_change_started(LaneIntentDirection.left)
    ended = replace(car, lane_change_state=ObservedLaneChangeState.pre)
    gap = coordinator.update(current_plan, replace(observed, valid_for_control=False) if gap_is_topology else observed,
                             car, now_ns=3_600_000_000)
    if gap_is_topology:
      assert gap.signal_requested and gap.lane_change_ready
    result = coordinator.update(current_plan, observed, ended, now_ns=3_900_000_000)
    assert result.reason == "laneChangeCompletionUnconfirmed"


def test_execution_abort_cannot_restart_same_event_after_recovery():
  for direction in (LaneIntentDirection.left, LaneIntentDirection.right):
    for fault in ("navigation", "brake", "lateral", "lamp", "timeout", "topology"):
      coordinator, current_plan, observed, car = relative_change_started(direction)
      bad_plan = replace(current_plan, valid=False, maneuver_event_id=0) if fault == "navigation" else current_plan
      bad_lane = replace(observed, valid_for_control=False) if fault == "topology" else observed
      bad_car = replace(car, brake_pressed=fault == "brake",
                        lateral_active=fault != "lateral")
      if fault == "lamp":
        bad_car = replace(bad_car, left_blinker=False, right_blinker=False)
      now_ns = 13_000_000_000 if fault == "timeout" else 3_000_000_000
      result = coordinator.update(bad_plan, bad_lane, bad_car, now_ns=now_ns)
      if fault == "topology":
        assert result.signal_requested and result.reason == "heuristicTopologyTransition"
        now_ns = 13_000_000_000
        result = coordinator.update(bad_plan, bad_lane, bad_car, now_ns=now_ns)
      assert not result.signal_requested
      ended = replace(car, lane_change_state=ObservedLaneChangeState.off, lane_change_direction=LaneIntentDirection.none)
      for index in range(1, 8):
        result = coordinator.update(current_plan, observed, ended, now_ns=now_ns + index * 600_000_000)
        assert result.reason == "laneChangeCompletionUnconfirmed"
        assert not result.signal_requested and not result.lane_change_ready
      assert coordinator._relative_consistency._completed_changes == 0


def test_invalid_topology_does_not_hide_opposite_model_direction():
  coordinator, current_plan, observed, car = relative_change_started(LaneIntentDirection.left)
  opposite = replace(car, lane_change_direction=LaneIntentDirection.right)
  result = coordinator.update(current_plan, replace(observed, valid_for_control=False), opposite,
                              now_ns=3_000_000_000)
  assert result.reason == 'directionMismatch'
  assert not result.signal_requested and not result.lane_change_ready


def test_accelerator_and_highway_speed_do_not_cancel_navigation_lane_change():
  coordinator, plan, topology, car = relative_change_started(LaneIntentDirection.left)
  car = replace(car, gas_pressed=True, speed_mps=35.)
  result = coordinator.update(plan, topology, car, now_ns=3_000_000_000)
  assert result.signal_requested and result.lane_change_ready


def test_navigation_signal_can_wait_for_crossing_permission_then_become_ready():
  coordinator = NavLaneIntentCoordinator()
  plan = NavLanePlan(True, 'route', 1, 17, 3, (0,))
  denied = LaneTopologyInput(True, 3, 1, True, True, False, True)
  car = LaneVehicleInput(True, 25.)
  coordinator.update(plan, denied, car, now_ns=1_000_000_000)
  waiting = coordinator.update(plan, denied, car, now_ns=1_500_000_000)
  assert waiting.signal_requested and not waiting.lane_change_ready
  car = replace(car, left_blinker=True)
  assert not coordinator.update(plan, denied, car, now_ns=1_600_000_000).lane_change_ready
  allowed = replace(denied, left_crossing_allowed=True)
  coordinator.update(plan, allowed, car, now_ns=1_700_000_000)
  ready = coordinator.update(plan, allowed, car, now_ns=2_100_000_000)
  assert ready.signal_requested and ready.lane_change_ready


def test_prestart_observation_recovery_does_not_gain_execution_failure_lock():
  coordinator, current_plan, observed, car = relative_change_started(LaneIntentDirection.left)
  # Build a separate coordinator that has only reached the start-ready phase.
  coordinator = NavLaneIntentCoordinator()
  car = replace(car, lane_change_state=ObservedLaneChangeState.off, lane_change_direction=LaneIntentDirection.none)
  for stamp in (1_000_000_000, 1_600_000_000, 2_200_000_000, 2_600_000_000):
    result = coordinator.update(current_plan, observed, car, now_ns=stamp)
  assert result.lane_change_ready
  coordinator.update(replace(current_plan, valid=False), observed, car, now_ns=3_000_000_000)
  for stamp in (4_000_000_000, 4_600_000_000, 5_200_000_000, 5_600_000_000):
    result = coordinator.update(current_plan, observed, car, now_ns=stamp)
  assert result.lane_change_ready


def test_wrong_observed_lane_change_direction_aborts_and_latches_event():
  coordinator = NavLaneIntentCoordinator()
  coordinator.update(plan(), topology(left_cross=True), vehicle(), now_ns=0)
  coordinator.update(plan(), topology(left_cross=True), vehicle(left_blinker=True), now_ns=500_000_000)
  aborted = coordinator.update(
    plan(), topology(left_cross=True),
    vehicle(state=ObservedLaneChangeState.starting, direction=LaneIntentDirection.right, left_blinker=True), now_ns=900_000_000,
  )
  assert not aborted.signal_requested and aborted.reason == "directionMismatch"
  assert coordinator.update(plan(), topology(left_cross=True), vehicle(), now_ns=2_000_000_000).reason == "blockedEvent"


def test_new_navigation_session_never_inherits_old_block_or_active_intent():
  coordinator = NavLaneIntentCoordinator()
  coordinator.update(plan(session="old"), topology(left_cross=True), vehicle(), now_ns=0)
  coordinator.update(plan(session="old"), topology(left_cross=True), vehicle(left_blinker=True), now_ns=500_000_000)
  coordinator.update(
    plan(session="old"), topology(left_cross=True),
    vehicle(state=ObservedLaneChangeState.starting, direction=LaneIntentDirection.right, left_blinker=True),
    now_ns=900_000_000,
  )
  assert coordinator.update(plan(session="old"), topology(left_cross=True), vehicle(),
                            now_ns=1_000_000_000).reason == "blockedEvent"
  fresh = coordinator.update(plan(session="new"), topology(left_cross=True), vehicle(), now_ns=1_100_000_000)
  assert fresh.reason == "stabilizingLaneAlignment"


def test_expected_source_pair_transition_during_lane_change_has_bounded_grace():
  coordinator = NavLaneIntentCoordinator()
  coordinator.update(plan(recommended=(0,)), topology(ego=2, left_cross=True), vehicle(), now_ns=0)
  coordinator.update(plan(recommended=(0,)), topology(ego=2, left_cross=True),
                     vehicle(left_blinker=True), now_ns=500_000_000)
  coordinator.update(plan(recommended=(0,)), topology(ego=2, left_cross=True),
                     vehicle(left_blinker=True), now_ns=800_000_000)
  coordinator.update(
    plan(recommended=(0,)), topology(ego=2, left_cross=True),
    vehicle(state=ObservedLaneChangeState.starting, direction=LaneIntentDirection.left, left_blinker=True),
    now_ns=900_000_000,
  )
  transition = coordinator.update(
    plan(recommended=(0,)), topology(ego=-1, valid=False),
    vehicle(state=ObservedLaneChangeState.starting, direction=LaneIntentDirection.left, left_blinker=True),
    now_ns=1_000_000_000,
  )
  assert transition.signal_requested and transition.reason == "topologyTransition"
  prolonged = coordinator.update(
    plan(recommended=(0,)), topology(ego=-1, valid=False),
    vehicle(state=ObservedLaneChangeState.starting, direction=LaneIntentDirection.left, left_blinker=True),
    now_ns=3_700_000_000,
  )
  assert prolonged.signal_requested and prolonged.reason == "topologyTransition"
  coordinator.update(
    plan(recommended=(0,)), topology(ego=1, left_cross=True),
    vehicle(state=ObservedLaneChangeState.pre, direction=LaneIntentDirection.left, left_blinker=True),
    now_ns=3_900_000_000,
  )
  observed = coordinator.update(
    plan(recommended=(0,)), topology(ego=1, left_cross=True),
    vehicle(state=ObservedLaneChangeState.pre, direction=LaneIntentDirection.left, left_blinker=True),
    now_ns=4_400_000_000,
  )
  assert not observed.signal_requested and observed.reason == "laneChangeObserved"


def test_long_solid_wait_gets_a_fresh_lane_change_timeout_when_dashed_appears():
  coordinator = NavLaneIntentCoordinator()
  coordinator.update(plan(), topology(), vehicle(), now_ns=0)
  coordinator.update(plan(), topology(), vehicle(), now_ns=500_000_000)
  coordinator.update(plan(), topology(left_cross=True), vehicle(left_blinker=True), now_ns=11_000_000_000)
  authorized = coordinator.update(plan(), topology(left_cross=True), vehicle(left_blinker=True), now_ns=11_300_000_000)
  assert authorized.lane_change_ready
  still_authorized = coordinator.update(plan(), topology(left_cross=True), vehicle(left_blinker=True), now_ns=11_350_000_000)
  assert still_authorized.lane_change_ready
