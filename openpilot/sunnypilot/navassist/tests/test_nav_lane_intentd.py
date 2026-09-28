from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.navassist.lane_intent import (
  LaneIntentDirection, LaneTopologyInput, LaneVehicleInput, NavLaneIntent, NavLaneIntentCoordinator,
  NavTurnPlan, NavTurnSignalCoordinator, ObservedLaneChangeState,
)
from openpilot.sunnypilot.navassist.nav_lane_intentd import (
  FinalForkScope,
  apply_oem_signal_gate, apply_radar_target_gate, build_lane_plan, lane_alignment_may_start, navigation_change_pending, navigation_lane_guidance_active, navigation_linked, oem_crossing_allowed, oem_gate_reason,
  anchored_amap_ego_index, visual_lane_geometry,
)
from openpilot.sunnypilot.selfdrive.controls.lib.nav_turn_completion import sp_turn_geometry_active


def nav(**updates):
  values = {
    "stale": False,
    "routeActive": True,
    "routeMatched": True,
    "mode": "realtime",
    "maneuverEventId": 11,
  }
  values.update(updates)
  return SimpleNamespace(**values)


def test_final_fork_scope_retains_highway_evidence_when_current_link_becomes_ramp():
  scope = FinalForkScope()
  approach = nav(sessionId="route", routeRevision=3, maneuver="rampRight", roadClass=6,
                 elevatedRoadStatus="unknown", valid=True, lanes=[], maneuverDistanceM=1500.0)
  assert scope.update(approach, linked=True)

  ramp_link = SimpleNamespace(**{**vars(approach), "roadClass": 7, "maneuverDistanceM": 50.0})
  final_fork_allowed = scope.update(ramp_link, linked=True)
  assert final_fork_allowed
  plan = build_lane_plan(ramp_link, topology(count=3), healthy=True,
                         final_fork_allowed=final_fork_allowed)
  assert plan.valid and plan.force_fork and plan.edge_direction == LaneIntentDirection.right
  assert not scope.update(SimpleNamespace(**{**vars(ramp_link), "maneuverEventId": 12}), linked=True)


def test_final_fork_scope_accepts_confirmed_elevated_route_but_not_ordinary_road():
  elevated = nav(sessionId="route", routeRevision=3, maneuver="exitLeft", roadClass=1,
                 elevatedRoadStatus="main")
  ordinary = SimpleNamespace(**{**vars(elevated), "maneuverEventId": 12, "elevatedRoadStatus": "unknown"})
  scope = FinalForkScope()
  assert scope.update(elevated, linked=True)
  assert not scope.update(ordinary, linked=True)


def test_final_fork_scope_does_not_miss_reference_entry_between_amap_distance_samples():
  scope = FinalForkScope()
  guidance = nav(sessionId="route", routeRevision=3, maneuver="mergeLeft", roadClass=6,
                 elevatedRoadStatus="unknown", valid=True, lanes=[], maneuverDistanceM=131.0)
  assert scope.update(guidance, linked=True)
  assert not scope.entry_reached

  guidance.maneuverDistanceM = 90.0
  assert scope.update(guidance, linked=True)
  assert scope.entry_reached
  plan = build_lane_plan(guidance, topology(count=0), healthy=True,
                         final_fork_allowed=True, final_fork_entry_reached=scope.entry_reached)
  assert plan.valid and plan.force_fork and plan.edge_direction == LaneIntentDirection.left

  next_event = SimpleNamespace(**{**vars(guidance), "maneuverEventId": 12, "maneuverDistanceM": 90.0})
  assert scope.update(next_event, linked=True)
  assert not scope.entry_reached


def test_final_fork_scope_does_not_predict_entry_without_a_decreasing_sample():
  scope = FinalForkScope()
  guidance = nav(sessionId="route", routeRevision=3, maneuver="rampRight", roadClass=6,
                 elevatedRoadStatus="unknown", valid=True, lanes=[], maneuverDistanceM=90.0)
  assert scope.update(guidance, linked=True)
  assert not scope.entry_reached
  assert scope.update(guidance, linked=True)
  assert not scope.entry_reached


def test_navigation_ready_waits_for_measured_unsafe_adjacent_radar_target():
  model = SimpleNamespace(laneLines=[SimpleNamespace(x=[0., 100.], y=[y, y])
                                     for y in (-5.25, -1.75, 1.75, 5.25)])
  unsafe = SimpleNamespace(dRel=16., yRel=3.4, vRel=-6.5, deprecated=SimpleNamespace(measured=True))
  radar = SimpleNamespace(points=[unsafe])
  ready = NavLaneIntent(signal_requested=True, lane_change_ready=True,
                        direction=LaneIntentDirection.left, request_id=5, target_lane_index=0)
  held = apply_radar_target_gate(ready, radar, model, speed_mps=23.5, healthy=True)
  assert held.signal_requested and not held.lane_change_ready and held.reason == 'radarLeftTargetUnsafe'
  assert apply_radar_target_gate(ready, SimpleNamespace(points=[]), model, speed_mps=23.5, healthy=True) == ready
  other_side = SimpleNamespace(dRel=16., yRel=-3.4, vRel=-6.5, deprecated=SimpleNamespace(measured=True))
  assert apply_radar_target_gate(ready, SimpleNamespace(points=[other_side]), model,
                                 speed_mps=23.5, healthy=True) == ready
  assert apply_radar_target_gate(ready, radar, model, speed_mps=23.5, healthy=False) == ready


@pytest.mark.parametrize('road_class', [0, 6])
@pytest.mark.parametrize('maneuver', ['exitLeft', 'exitRight', 'rampLeft', 'mergeRight', 'keepLeft', 'keepRight'])
@pytest.mark.parametrize('speed', [60 / 3.6, 25.0, 120 / 3.6])
def test_highway_navigation_may_request_a_lane_until_the_maneuver(road_class, maneuver, speed):
  guidance = lane_guidance_nav(roadClass=road_class, maneuver=maneuver)
  turn = NavTurnSignalCoordinator().update(NavTurnPlan(False, '', 0, 0, maneuver, 0), speed_mps=speed, now_ns=0)
  for distance, expected in ((-1., False), (0., False), (1., True), (500., True)):
    guidance.maneuverDistanceM = distance
    assert lane_alignment_may_start(guidance, turn, speed_mps=speed) == expected


@pytest.mark.parametrize('distance', [float('nan'), float('inf'), -1., 0.])
def test_highway_invalid_distance_cannot_start_lane_alignment(distance):
  guidance = lane_guidance_nav(roadClass=0, maneuver='exitRight', maneuverDistanceM=distance)
  turn = NavTurnSignalCoordinator().update(NavTurnPlan(False, '', 0, 0, 'exitRight', 0), speed_mps=25., now_ns=0)
  assert not lane_alignment_may_start(guidance, turn, speed_mps=25.)


def test_highway_pending_request_remains_available_near_exit():
  for started in (False, True):
    coordinator = NavLaneIntentCoordinator()
    guidance = lane_guidance_nav(roadClass=0, maneuver='exitRight', maneuverDistanceM=500.)
    current_plan = build_lane_plan(guidance, topology(), healthy=True)
    observed = LaneTopologyInput(True, 3, 1, True, True, True, True)
    car = LaneVehicleInput(True, 25., right_blinker=True)
    for stamp in (1_000_000_000, 1_600_000_000, 2_200_000_000, 2_600_000_000):
      coordinator.update(current_plan, observed, car, now_ns=stamp)
    if started:
      car = replace(car, lane_change_state=ObservedLaneChangeState.starting, lane_change_direction=LaneIntentDirection.right)
    guidance.maneuverDistanceM = 22.5
    turn = NavTurnSignalCoordinator().update(NavTurnPlan(False, '', 0, 0, 'exitRight', 0), speed_mps=25., now_ns=0)
    allowed = lane_alignment_may_start(guidance, turn, speed_mps=25.)
    assert allowed
    result = coordinator.update(current_plan, observed, car, now_ns=2_700_000_000, allow_new_lane_change=allowed)
    assert result.signal_requested


def test_only_an_actual_navigation_lane_target_reserves_overtake():
  observed = LaneTopologyInput(True, 3, 1, True, True, True, True)
  no_target = replace(build_lane_plan(lane_guidance_nav(maneuver='none', maneuverDistanceM=1000.), topology(), healthy=True),
                      lane_count=3)
  assert not navigation_change_pending(no_target, observed)
  assert not navigation_change_pending(replace(no_target, recommended_indices=(1,)), observed)
  assert navigation_change_pending(replace(no_target, recommended_indices=(0,)), observed)
  assert navigation_change_pending(replace(no_target, recommended_indices=(2,)), observed)
  assert not navigation_change_pending(replace(no_target, lane_count=4, recommended_indices=(0,)), observed)
  assert not navigation_change_pending(replace(no_target, recommended_indices=(0,)),
                                       replace(observed, valid_for_control=False))


def test_active_lane_guidance_keeps_efficiency_out_after_reaching_a_recommended_lane():
  lanes = [SimpleNamespace(index=i, recommended=i == 2, routeAvoid=i != 2) for i in range(3)]
  guidance = lane_guidance_nav(lanes=lanes)
  current_plan = build_lane_plan(guidance, topology(), healthy=True)
  observed = LaneTopologyInput(True, 3, 2, True, False, True, False)

  assert not navigation_change_pending(current_plan, observed)
  assert navigation_lane_guidance_active(current_plan, guidance)


def test_lane_guidance_without_a_lane_constraint_does_not_reserve_efficiency():
  all_recommended = lane_guidance_nav(
    lanes=[SimpleNamespace(index=i, recommended=True, routeAvoid=False) for i in range(3)],
  )
  empty = lane_guidance_nav(lanes=[])
  assert not navigation_lane_guidance_active(build_lane_plan(all_recommended, topology(), healthy=True),
                                             all_recommended)
  assert not navigation_lane_guidance_active(build_lane_plan(empty, topology(), healthy=True), empty)
  stale = lane_guidance_nav(stale=True, lanes=all_recommended.lanes)
  assert not navigation_lane_guidance_active(build_lane_plan(stale, topology(), healthy=True), stale)


def test_pre_turn_signal_uses_fresh_linked_route_without_full_gps_control_validity():
  assert navigation_linked(nav(), base_healthy=True)


def test_pre_turn_signal_rejects_stale_inactive_unmatched_or_non_realtime_routes():
  assert not navigation_linked(nav(stale=True), base_healthy=True)
  assert not navigation_linked(nav(routeActive=False), base_healthy=True)
  assert not navigation_linked(nav(routeMatched=False), base_healthy=True)
  assert not navigation_linked(nav(mode="simulation"), base_healthy=True)
  assert not navigation_linked(nav(maneuverEventId=0), base_healthy=True)
  assert not navigation_linked(nav(), base_healthy=False)


def test_sp_turn_geometry_uses_existing_vision_lateral_acceleration_threshold():
  straight = SimpleNamespace(desiredCurvature=0.01)
  turning = SimpleNamespace(desiredCurvature=0.015)
  assert not sp_turn_geometry_active(straight, 10.0)
  assert sp_turn_geometry_active(turning, 10.0)


def lane_guidance_nav(**updates):
  values = {
    "valid": True,
    "stale": False,
    "sessionId": "session-a",
    "routeRevision": 3,
    "maneuverEventId": 17,
    "maneuver": "turnLeft",
    "maneuverDistanceM": 800.0,
    "lanes": [],
  }
  values.update(updates)
  return SimpleNamespace(**values)


def topology(count=3):
  return SimpleNamespace(visibleLaneCount=count)


def oem_topology(count=3, index=1, *, valid=True, left_marking="dashed", right_marking="dashed"):
  return SimpleNamespace(
    visibleLaneCount=count, egoLaneIndexFromLeft=index, validForControl=valid,
    leftNeighborExists=index > 0, rightNeighborExists=index < count - 1,
    leftEvidenceValid=True, rightEvidenceValid=True,
    leftCrossingAllowed=left_marking == "dashed", rightCrossingAllowed=right_marking == "dashed",
    leftEgoSideMarking=left_marking, rightEgoSideMarking=right_marking,
    publishMonoTime=1_000_000_000, modelMonoTime=1_000_000_000, imageMonoTime=1_000_000_000,
  )


def test_visual_geometry_does_not_depend_on_239_position():
  observed = oem_topology(count=4, index=2)
  assert visual_lane_geometry(observed, healthy=True) == (True, 4, 2, True, True)
  assert visual_lane_geometry(oem_topology(count=3, index=0), healthy=True) == (True, 3, 0, None, True)
  assert visual_lane_geometry(oem_topology(count=3, index=0, left_marking="roadEdge"), healthy=True) == (
    True, 3, 0, False, True,
  )
  assert visual_lane_geometry(observed, healthy=False) == (False, 0, -1, None, None)


def test_399_permission_can_authorize_without_fresh_visual_or_with_visual_solid():
  observed = oem_topology()
  denied = {"permissionValid": True, "leftAllowed": False, "rightAllowed": True,
            "leftLaneEvidenceValid": True, "rightLaneEvidenceValid": True}
  assert not oem_crossing_allowed(observed, denied, side="left", visual_healthy=True)
  assert oem_crossing_allowed(observed, denied, side="right", visual_healthy=True)
  assert not oem_crossing_allowed(observed, {"permissionValid": False}, side="right", visual_healthy=True)
  assert oem_crossing_allowed(observed, denied, side="left", visual_healthy=True, now_ns=1_000_000_000)
  assert oem_crossing_allowed(observed, {"permissionValid": False, "leftSafetyBlocked": True},
                              side="left", visual_healthy=True, now_ns=1_000_000_000)
  assert not oem_crossing_allowed(observed, {**denied, "leftSafetyBlocked": True},
                                  side="left", visual_healthy=True, now_ns=1_000_000_000)

  allowed = {"permissionValid": True, "leftAllowed": True, "rightAllowed": True,
             "leftLaneEvidenceValid": True, "rightLaneEvidenceValid": True}
  solid = oem_topology(left_marking="solid")
  assert oem_crossing_allowed(solid, allowed, side="left", visual_healthy=True)
  # Without either fresh positive source, there is no permission.
  assert oem_crossing_allowed(solid, allowed, side="left", visual_healthy=False)
  assert not oem_crossing_allowed(observed, {"permissionValid": False, "leftAllowed": True},
                                  side="left", visual_healthy=False)


def test_oem_gate_reason_distinguishes_neighbor_permission_and_visual_veto():
  plan = build_lane_plan(lane_guidance_nav(maneuver="turnLeft"), topology(), healthy=True)
  lane = LaneTopologyInput(True, 3, 1, True, True, True, True)
  blocked = replace(lane, left_crossing_allowed=False)
  assert oem_gate_reason(plan, blocked, {"permissionValid": False}) == "oem399Stale"
  denied_neighbor = replace(lane, left_neighbor_exists=False)
  valid = {"permissionValid": True, "leftAllowed": True}
  assert oem_gate_reason(plan, denied_neighbor, valid) == "visualNoLeftNeighbor"
  assert oem_gate_reason(plan, blocked, {**valid, "leftAllowed": False}) == "oem399LeftDenied"
  assert oem_gate_reason(plan, lane, {"permissionValid": False}) is None
  assert oem_gate_reason(plan, replace(lane, left_crossing_allowed=False), valid) == "visualLeftBoundaryVeto"
  assert oem_gate_reason(plan, lane, valid) is None


@pytest.mark.parametrize("reason", ["visualNoRightNeighbor", "oem399RightDenied", "visualRightBoundaryVeto"])
def test_oem_gate_keeps_waiting_signal_but_not_start_readiness(reason):
  pending = NavLaneIntent(True, False, LaneIntentDirection.right, 2, 2, "heuristicSignaling")
  blocked = apply_oem_signal_gate(pending, reason, "idle")
  assert blocked == NavLaneIntent(reason=reason)
  assert apply_oem_signal_gate(pending, reason, "signaling") == replace(pending, reason=reason)
  assert apply_oem_signal_gate(pending, None, "signaling") == pending
  active = replace(pending, lane_change_ready=True)
  assert apply_oem_signal_gate(active, reason, "changing") == replace(active, reason=reason)


def test_oem_denial_allows_navigation_signal_then_permission_releases_ready():
  coordinator = NavLaneIntentCoordinator()
  plan = build_lane_plan(lane_guidance_nav(maneuver="exitRight", roadClass=6), topology(), healthy=True)
  observed = LaneTopologyInput(True, 3, 1, True, True, True, True)
  car = LaneVehicleInput(True, 25.0)
  denied = replace(observed, right_crossing_allowed=False)
  for stamp in (1_000_000_000, 1_500_000_000, 2_000_000_000):
    waiting = coordinator.update(plan, denied, car, now_ns=stamp)
  waiting = apply_oem_signal_gate(waiting, "oem399RightDenied", coordinator._phase)
  assert waiting.signal_requested and not waiting.lane_change_ready
  assert coordinator._phase == "signaling"

  car = replace(car, right_blinker=True)
  waiting = coordinator.update(plan, denied, car, now_ns=2_100_000_000)
  assert waiting.signal_requested and not waiting.lane_change_ready
  coordinator.update(plan, observed, car, now_ns=2_200_000_000)
  assert coordinator.update(plan, observed, car, now_ns=2_600_000_000).lane_change_ready


def test_oem_lane_denial_leaves_turn_signal_intent_available():
  lane = apply_oem_signal_gate(NavLaneIntent(True, False, LaneIntentDirection.right, 17, 2),
                               "oem399RightDenied", "idle")
  turn = NavTurnSignalCoordinator().update(
    NavTurnPlan(True, "session-a", 3, 17, "turnRight", 50.0), speed_mps=10.0, now_ns=1_000_000_000,
  )
  assert not lane.signal_requested
  assert turn.signal_requested and turn.target_lane_index == -1


def test_visual_permission_can_authorize_navigation_without_399():
  observed = oem_topology()
  observed.publishMonoTime = observed.modelMonoTime = observed.imageMonoTime = 2_000_000_000
  observed.leftNeighborExists = observed.rightNeighborExists = True
  observed.leftCrossingAllowed = observed.rightCrossingAllowed = True
  for side in ('left', 'right'):
    assert oem_crossing_allowed(observed, {}, side=side, visual_healthy=True, now_ns=2_000_000_000)
    assert not oem_crossing_allowed(observed, {}, side=side, visual_healthy=True, now_ns=2_200_000_000)


def test_missing_lane_info_uses_visual_extreme_lane_for_ordinary_turns():
  left = build_lane_plan(lane_guidance_nav(maneuver="turnLeft"), topology(), healthy=True)
  right = build_lane_plan(lane_guidance_nav(maneuver="turnRight"), topology(), healthy=True)

  assert left.valid and left.heuristic and left.lane_count == 3 and left.recommended_indices == (0,)
  assert right.valid and right.heuristic and right.lane_count == 3 and right.recommended_indices == (2,)
  assert not left.allow_unknown_crossing and not right.allow_unknown_crossing
  assert not left.ignore_solid_boundary and not right.ignore_solid_boundary


def test_unanchored_middle_amap_lane_is_not_treated_as_a_visual_absolute_index():
  lanes = [
    SimpleNamespace(index=0, recommended=False),
    SimpleNamespace(index=1, recommended=True),
    SimpleNamespace(index=2, recommended=False),
  ]
  plan = build_lane_plan(lane_guidance_nav(maneuver="turnLeft", lanes=lanes), topology(), healthy=True)

  assert not plan.valid
  assert not plan.heuristic
  assert plan.recommended_indices == ()


def test_matching_visual_and_oem_position_selects_nearest_amap_lane():
  observed = oem_topology(count=3, index=2)
  oem = {"positionValid": True, "position": "rightmost"}
  anchor = anchored_amap_ego_index(observed, oem, 3)
  lanes = [SimpleNamespace(index=i, recommended=i == 1, routeAvoid=False) for i in range(3)]
  plan = build_lane_plan(lane_guidance_nav(lanes=lanes), observed, healthy=True,
                         lane_count_override=3, amap_ego_index=anchor)
  assert anchor == 2 and plan.valid and not plan.heuristic and plan.recommended_indices == (1,)


def test_recommended_current_edge_does_not_request_a_lane_change_with_local_visual_window():
  lanes = [SimpleNamespace(index=i, recommended=i == 3, routeAvoid=False) for i in range(4)]
  observed = oem_topology(count=3, index=2)
  plan = build_lane_plan(lane_guidance_nav(maneuver="exitRight", lanes=lanes), observed, healthy=True,
                         lane_count_override=3, oem_edge_position="rightmost")
  assert plan.valid and not plan.heuristic and plan.recommended_indices == (2,)
  coordinator = NavLaneIntentCoordinator()
  result = coordinator.update(plan, LaneTopologyInput(True, 3, 2, True, False, True, False),
                              LaneVehicleInput(True, 20.0), now_ns=1_000_000_000)
  assert not result.signal_requested and result.reason == "alreadyInRecommendedLane"


def test_unknown_middle_position_uses_route_edge_change_then_stops_at_confirmed_edge():
  lanes = [SimpleNamespace(index=i, recommended=i == 3, routeAvoid=False) for i in range(4)]
  guidance = lane_guidance_nav(maneuver="exitRight", lanes=lanes)
  middle = build_lane_plan(guidance, oem_topology(count=3, index=1), healthy=True,
                           lane_count_override=3, oem_edge_position="middle")
  assert middle.valid and middle.heuristic and middle.edge_direction == LaneIntentDirection.right
  edge = build_lane_plan(guidance, oem_topology(count=3, index=2), healthy=True,
                         lane_count_override=3, oem_edge_position="rightmost")
  assert edge.valid and not edge.heuristic and edge.recommended_indices == (2,)


def test_local_visual_window_cannot_claim_an_amap_absolute_lane():
  observed = oem_topology(count=3, index=1)
  assert anchored_amap_ego_index(observed, {"positionValid": True, "position": "middle"}, 4) is None
  # Three visible lanes may be only a window of a wider road. 0x239 says
  # "middle", not which absolute middle lane this is.
  assert anchored_amap_ego_index(observed, {"positionValid": True, "position": "middle"}, 3) is None
  assert anchored_amap_ego_index(observed, {"positionValid": True, "position": "leftmost"}, 3) is None
  observed.validForControl = False
  assert anchored_amap_ego_index(observed, {"positionValid": True, "position": "middle"}, 3) is None


def test_amap_edge_recommendation_qualifies_relative_extreme_on_wider_road():
  lanes = [SimpleNamespace(index=index, recommended=index == 0) for index in range(4)]

  plan = build_lane_plan(lane_guidance_nav(maneuver="turnLeft", lanes=lanes), topology(count=3), healthy=True)

  assert plan.valid and plan.heuristic
  assert plan.lane_count == 3
  assert plan.recommended_indices == (0,)


def test_turn_fallback_is_bounded_while_exit_fallback_starts_farther_out():
  far_turn = build_lane_plan(
    lane_guidance_nav(maneuver="turnLeft", maneuverDistanceM=1_500.0), topology(), healthy=True,
  )
  far_exit = build_lane_plan(
    lane_guidance_nav(maneuver="exitRight", maneuverDistanceM=1_500.0), topology(), healthy=True,
  )

  assert not far_turn.heuristic and far_turn.recommended_indices == ()
  assert far_exit.heuristic and far_exit.recommended_indices == (2,)


def test_imminent_exit_selects_fork_but_never_bypasses_crossing_permission():
  plan = build_lane_plan(
    lane_guidance_nav(maneuver="exitRight", maneuverDistanceM=50.0, roadClass=6), topology(count=1), healthy=True,
  )

  assert plan.valid and plan.heuristic
  assert plan.edge_direction == LaneIntentDirection.right
  assert plan.force_fork
  assert not plan.allow_unknown_crossing
  assert not plan.ignore_solid_boundary


@pytest.mark.parametrize("maneuver,direction", [
  ("exitLeft", LaneIntentDirection.left), ("exitRight", LaneIntentDirection.right),
  ("rampLeft", LaneIntentDirection.left), ("rampRight", LaneIntentDirection.right),
  ("mergeLeft", LaneIntentDirection.left), ("mergeRight", LaneIntentDirection.right),
])
@pytest.mark.parametrize("distance", [20.0, 50.0, 51.0])
def test_near_exit_alignment_remains_enabled_without_forcing_crossing(maneuver, direction, distance):
  guidance = lane_guidance_nav(maneuver=maneuver, maneuverDistanceM=distance)
  plan = build_lane_plan(guidance, topology(), healthy=True)
  turn = NavTurnSignalCoordinator().update(
    NavTurnPlan(True, "session-a", 3, 17, maneuver, distance), speed_mps=15.0, now_ns=0,
  )
  assert plan.valid and plan.edge_direction == direction
  assert lane_alignment_may_start(guidance, turn, speed_mps=15.0)
  assert not plan.force_fork and not plan.ignore_solid_boundary
  coordinator = NavLaneIntentCoordinator()
  observed = LaneTopologyInput(True, 3, 1, True, True, True, True)
  car = LaneVehicleInput(True, 15.0)
  for stamp in (0, 500_000_000, 1_000_000_000):
    result = coordinator.update(plan, observed, car, now_ns=stamp)
  assert result.signal_requested and not result.lane_change_ready
  car = replace(car, left_blinker=direction == LaneIntentDirection.left,
                right_blinker=direction == LaneIntentDirection.right)
  for stamp in (1_100_000_000, 1_500_000_000):
    result = coordinator.update(plan, observed, car, now_ns=stamp)
  assert result.lane_change_ready and result.direction == direction


@pytest.mark.parametrize("marking,permission", [("roadEdge", True), ("unknown", False)])
def test_near_exit_can_signal_but_cannot_start_through_boundary_or_missing_permission(marking, permission):
  guidance = lane_guidance_nav(maneuver="exitRight", maneuverDistanceM=20.0)
  plan = build_lane_plan(guidance, topology(), healthy=True)
  oem = dict(permissionValid=permission, rightAllowed=True, rightLaneEvidenceValid=True)
  crossing = oem_crossing_allowed(oem_topology(right_marking=marking), oem, side="right", visual_healthy=True)
  assert not crossing
  observed = LaneTopologyInput(True, 3, 1, True, True, True, crossing)
  coordinator = NavLaneIntentCoordinator()
  car = LaneVehicleInput(True, 15.0, right_blinker=True)
  for stamp in (0, 500_000_000, 1_000_000_000, 2_000_000_000):
    result = coordinator.update(plan, observed, car, now_ns=stamp)
    assert not result.lane_change_ready
  assert result.signal_requested


def test_slight_turn_without_lane_info_does_not_force_an_extreme_lane():
  plan = build_lane_plan(lane_guidance_nav(maneuver="slightLeft"), topology(), healthy=True)

  assert not plan.heuristic
  assert plan.recommended_indices == ()


@pytest.mark.parametrize('road_class', [0, 6])
@pytest.mark.parametrize('maneuver,direction', [('keepLeft', LaneIntentDirection.left), ('keepRight', LaneIntentDirection.right)])
def test_highway_and_expressway_keep_guidance_uses_existing_exit_alignment(road_class, maneuver, direction):
  guidance = lane_guidance_nav(roadClass=road_class, maneuver=maneuver, maneuverDistanceM=1500.)
  plan = build_lane_plan(guidance, topology(), healthy=True)
  assert plan.valid and plan.heuristic and plan.edge_direction == direction
  assert not plan.force_fork and not plan.ignore_solid_boundary and not plan.allow_unknown_crossing
  guidance.maneuverDistanceM = 2001.
  assert not build_lane_plan(guidance, topology(), healthy=True).heuristic


@pytest.mark.parametrize('road_class', [-1, 1, 2, 3, 4, 5, 7])
def test_elevated_flag_or_road_name_does_not_invent_highway_classification(road_class):
  guidance = lane_guidance_nav(roadClass=road_class, elevatedRoadStatus='primary', currentRoad='某高架', maneuver='keepRight')
  plan = build_lane_plan(guidance, topology(), healthy=True)
  assert not plan.heuristic and not plan.recommended_indices


@pytest.mark.parametrize('maneuver,edge', [('slightLeft', 0), ('slightRight', 3),
                                          ('keepLeft', 0), ('keepRight', 3)])
@pytest.mark.parametrize('road_class', [7, 8])
def test_explicit_amap_edge_recommendation_works_without_highway_class(road_class, maneuver, edge):
  guidance = lane_guidance_nav(
    roadClass=road_class, maneuver=maneuver, maneuverDistanceM=1200.,
    lanes=[SimpleNamespace(index=i, recommended=i == edge, routeAvoid=False) for i in range(4)],
  )
  plan = build_lane_plan(guidance, topology(count=2), healthy=True)
  direction = LaneIntentDirection.left if edge == 0 else LaneIntentDirection.right
  target = 0 if edge == 0 else 1
  assert plan.valid and plan.heuristic and plan.lane_count == 2
  assert plan.recommended_indices == (target,) and plan.edge_direction == direction
  assert not plan.force_fork and not plan.allow_unknown_crossing and not plan.ignore_solid_boundary
  guidance.maneuverDistanceM = 2001.
  assert not build_lane_plan(guidance, topology(count=2), healthy=True).valid
  guidance.maneuverDistanceM = 1200.
  guidance.lanes[edge].routeAvoid = True
  assert not build_lane_plan(guidance, topology(count=2), healthy=True).valid
  guidance.lanes = [SimpleNamespace(index=i, recommended=False, routeAvoid=i != edge) for i in range(4)]
  assert not build_lane_plan(guidance, topology(count=2), healthy=True).valid


@pytest.mark.parametrize('road_class', [0, 6])
@pytest.mark.parametrize('maneuver,edge', [('slightLeft', 0), ('slightRight', 2)])
def test_highway_slight_guidance_needs_matching_edge_recommendation(road_class, maneuver, edge):
  guidance = lane_guidance_nav(roadClass=road_class, maneuver=maneuver, maneuverDistanceM=1200.)
  assert not build_lane_plan(guidance, topology(), healthy=True).heuristic
  guidance.lanes = [SimpleNamespace(index=i, recommended=i == edge, routeAvoid=False) for i in range(3)]
  assert build_lane_plan(guidance, topology(), healthy=True).heuristic
  guidance.lanes[edge].routeAvoid = True
  assert not build_lane_plan(guidance, topology(), healthy=True).valid


@pytest.mark.parametrize('fault', ['stale', 'invalid', 'disabled'])
def test_highway_keep_respects_route_health_and_existing_switch(fault):
  from openpilot.sunnypilot.navassist.settings import NavAssistSettings
  guidance = lane_guidance_nav(roadClass=0, maneuver='keepRight')
  settings = NavAssistSettings(lane_change_enabled=fault != 'disabled')
  if fault == 'stale': guidance.stale = True
  if fault == 'invalid': guidance.valid = False
  assert not build_lane_plan(guidance, topology(), healthy=True, settings=settings).valid


@pytest.mark.parametrize('road_class', [0, 6])
@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('block', [None, 'blindspot', 'boundary', 'neighbor', 'driver'])
def test_highway_keep_execution_retains_existing_vehicle_gates(road_class, side, block):
  guidance = lane_guidance_nav(roadClass=road_class, maneuver='keep' + side.title())
  plan = build_lane_plan(guidance, topology(), healthy=True)
  coordinator = NavLaneIntentCoordinator()
  observed = LaneTopologyInput(True, 3, 1, True, True, True, True)
  car = LaneVehicleInput(True, 25.0)
  if block == 'boundary':
    observed = replace(observed, **{side + '_crossing_allowed': False})
  if block == 'neighbor':
    observed = replace(observed, **{side + '_neighbor_exists': False})
  if block == 'blindspot':
    car = replace(car, **{side + '_blindspot': True})
  if block == 'driver':
    car = replace(car, steering_pressed=True)
  for stamp in (0, 500_000_000, 1_000_000_000):
    result = coordinator.update(plan, observed, car, now_ns=stamp)
    assert not result.lane_change_ready  # Physical lamp feedback is required.
  car = replace(car, **{side + '_blinker': True})
  for stamp in (1_100_000_000, 1_500_000_000):
    result = coordinator.update(plan, observed, car, now_ns=stamp)
  assert result.lane_change_ready == (block is None)
  if block is None:
    assert result.direction == getattr(LaneIntentDirection, side)


def test_started_relative_change_keeps_identity_through_local_lane_count_change_and_observation_gap():
  coordinator = NavLaneIntentCoordinator()
  guidance = lane_guidance_nav(maneuver="turnRight")
  current_plan = build_lane_plan(guidance, topology(count=3), healthy=True)
  observed = LaneTopologyInput(True, 3, 1, True, True, True, True)
  car = LaneVehicleInput(True, 15.0)
  for now_ns in (0, 500_000_000, 1_000_000_000):
    coordinator.update(current_plan, observed, car, now_ns=now_ns)
  changing_car = replace(car, right_blinker=True, lane_change_state=ObservedLaneChangeState.starting,
                         lane_change_direction=LaneIntentDirection.right)
  assert coordinator.update(current_plan, observed, changing_car, now_ns=1_200_000_000).signal_requested

  # The route and physical lateral state remain valid while visible lanes
  # recenter and then disappear briefly from the observation.
  recentered_plan = build_lane_plan(guidance, topology(count=2), healthy=True)
  recentered = coordinator.update(recentered_plan, replace(observed, visible_lane_count=2), changing_car,
                                 now_ns=1_300_000_000)
  assert recentered.signal_requested and recentered.reason == "heuristicChanging"
  missing_plan = build_lane_plan(guidance, topology(count=0), healthy=True)
  assert missing_plan.valid
  unavailable = replace(observed, valid_for_control=False, visible_lane_count=0, ego_lane_index=-1)
  gap = coordinator.update(missing_plan, unavailable, changing_car, now_ns=1_400_000_000)
  assert gap.signal_requested and gap.reason == "heuristicTopologyTransition"
  crossing_gap = coordinator.update(missing_plan, unavailable, changing_car, now_ns=4_000_000_000)
  assert crossing_gap.signal_requested and crossing_gap.reason == "heuristicTopologyTransition"
  expired = coordinator.update(missing_plan, unavailable, changing_car, now_ns=11_200_000_001)
  assert not expired.signal_requested and expired.reason == "laneChangeTimeout"


def test_existing_turn_window_hands_off_after_started_sp_cycle_without_another_lane_change():
  coordinator = NavLaneIntentCoordinator()
  turn_coordinator = NavTurnSignalCoordinator()
  guidance = lane_guidance_nav(maneuver="turnLeft", maneuverDistanceM=500.0)
  current_plan = build_lane_plan(guidance, topology(), healthy=True)
  observed = LaneTopologyInput(True, 3, 1, True, True, True, True)
  car = LaneVehicleInput(True, 15.0)
  for now_ns in (0, 500_000_000, 1_000_000_000):
    coordinator.update(current_plan, observed, car, now_ns=now_ns)

  # At 15 m/s the existing pre-turn window is 140 m. Reaching it while
  # SP reports starting must retain the same lane-change lamp and desire.
  guidance.maneuverDistanceM = 140.0
  turn = turn_coordinator.update(NavTurnPlan(True, "session-a", 3, 17, "turnLeft", 140.0),
                                 speed_mps=15.0, now_ns=1_200_000_000)
  assert turn.signal_requested and turn.target_lane_index == -1
  changing_car = replace(car, left_blinker=True, lane_change_state=ObservedLaneChangeState.starting,
                         lane_change_direction=LaneIntentDirection.left, model_mono_time_ns=1_200_000_000)
  started = coordinator.update(build_lane_plan(guidance, topology(), healthy=True), observed, changing_car,
                               now_ns=1_200_000_000, allow_new_lane_change=lane_alignment_may_start(guidance, turn, speed_mps=15.0))
  assert started.signal_requested and started.target_lane_index >= 0

  # Deceleration below the start threshold must not cut an SP action already
  # underway. The normal SP state cycle ends at pre.
  finishing_car = replace(changing_car, speed_mps=8.0, lane_change_state=ObservedLaneChangeState.finishing)
  for now_ns in (1_400_000_000, 1_600_000_000):
    assert coordinator.update(current_plan, observed, replace(finishing_car, model_mono_time_ns=now_ns), now_ns=now_ns,
                              allow_new_lane_change=False).signal_requested
  completed_car = replace(finishing_car, lane_change_state=ObservedLaneChangeState.pre,
                          model_mono_time_ns=1_800_000_000)
  completed = coordinator.update(current_plan, observed, completed_car, now_ns=1_800_000_000,
                                 allow_new_lane_change=False)
  assert not completed.signal_requested and completed.reason == "laneChangeObserved"
  selected = completed if completed.signal_requested else turn
  assert selected.target_lane_index == -1 and selected.signal_requested

  next_request = coordinator.update(current_plan, observed, car, now_ns=6_000_000_000,
                                    allow_new_lane_change=False)
  assert not next_request.signal_requested


def test_turn_window_releases_pending_lane_signal_before_sp_starts():
  coordinator = NavLaneIntentCoordinator()
  current_plan = build_lane_plan(lane_guidance_nav(), topology(), healthy=True)
  observed = LaneTopologyInput(True, 3, 1, True, True, True, True)
  car = LaneVehicleInput(True, 15.0)
  for now_ns in (0, 500_000_000):
    coordinator.update(current_plan, observed, car, now_ns=now_ns)
  assert coordinator.update(current_plan, observed, car, now_ns=1_000_000_000).signal_requested
  approaching = coordinator.update(current_plan, observed, car, now_ns=1_200_000_000,
                                   allow_new_lane_change=False)
  assert not approaching.signal_requested and approaching.reason == "turnApproachHandoff"


def test_zero_distance_allows_existing_plan_to_finish_but_never_starts_a_new_change():
  for maneuver in ("turnLeft", "exitRight"):
    guidance = lane_guidance_nav(maneuver=maneuver, maneuverDistanceM=0.0)
    turn = NavTurnSignalCoordinator().update(NavTurnPlan(True, "session-a", 3, 17, maneuver, 0.0),
                                            speed_mps=15.0, now_ns=0)
    assert not turn.signal_requested
    assert build_lane_plan(guidance, topology(), healthy=True).valid
    assert not lane_alignment_may_start(guidance, turn, speed_mps=15.0)


def test_lane_change_hard_timeout_still_applies_during_observation_grace():
  coordinator = NavLaneIntentCoordinator()
  current_plan = build_lane_plan(lane_guidance_nav(), topology(), healthy=True)
  observed = LaneTopologyInput(True, 3, 1, True, True, True, True)
  car = LaneVehicleInput(True, 15.0)
  for now_ns in (0, 500_000_000, 1_000_000_000):
    coordinator.update(current_plan, observed, car, now_ns=now_ns)
  changing_car = replace(car, left_blinker=True, lane_change_state=ObservedLaneChangeState.starting,
                         lane_change_direction=LaneIntentDirection.left)
  coordinator.update(current_plan, observed, changing_car, now_ns=1_200_000_000)
  missing = replace(observed, valid_for_control=False)
  grace = coordinator.update(current_plan, missing, changing_car, now_ns=11_000_000_000)
  assert grace.signal_requested
  expired = coordinator.update(current_plan, missing, changing_car, now_ns=11_200_000_001)
  assert not expired.signal_requested and expired.reason == "laneChangeTimeout"
