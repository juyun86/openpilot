#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from dataclasses import replace
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

from openpilot.sunnypilot.navassist.lane_intent import (
  LaneIntentDirection,
  LaneTopologyInput,
  LaneVehicleInput,
  NavLaneIntent,
  NavLaneIntentCoordinator,
  NavLanePlan,
  NavTurnPlan,
  NavTurnSignalCoordinator,
  ObservedLaneChangeState,
)
from openpilot.sunnypilot.navassist.oem_lane_feedback import OemLaneFeedback
from openpilot.sunnypilot.selfdrive.controls.lib.oem_lane_change_gate import lane_change_start_permissions
from openpilot.sunnypilot.selfdrive.controls.lib.nav_turn_completion import sp_turn_geometry_active
from openpilot.sunnypilot.navassist.settings import NavAssistSettings, SettingsCache
from openpilot.sunnypilot.navassist.efficiency_lane import EfficiencyLaneSelector, lane_leads, side_lead_unsafe


PUBLISH_HZ = 20
BASE_SERVICES = ("navAssistStateSP", "carState", "carControl", "controlsState")
MODEL_SERVICES = ("modelV2",)
VISUAL_LANE_SERVICES = ("laneTopologyStateSP",)
LANE_SERVICES = VISUAL_LANE_SERVICES + MODEL_SERVICES
SERVICES = BASE_SERVICES + LANE_SERVICES + ("radarTracks", "carParamsSP")
TURN_LANE_LOOKAHEAD_M = 1_000.0
EXIT_LANE_LOOKAHEAD_M = 2_000.0
# jihui Amap 141ebea0 MainActivitySettings.kt: fork_dist_h defaults to 80 m.
# This selects the final fork action; it never supplies crossing permission.
FORK_ENTRY_DISTANCE_M = 80.0
LEFT_TURN_LANE_MANEUVERS = frozenset(("turnLeft", "sharpLeft", "uTurnLeft"))
RIGHT_TURN_LANE_MANEUVERS = frozenset(("turnRight", "sharpRight", "uTurnRight"))
LEFT_EXIT_LANE_MANEUVERS = frozenset(("exitLeft", "rampLeft", "mergeLeft"))
RIGHT_EXIT_LANE_MANEUVERS = frozenset(("exitRight", "rampRight", "mergeRight"))
# AMap raw road classes; amapnavi's internal roadcate uses a different mapping.
# Elevated/parallel-road layer flags alone do not establish a controlled-access road.
HIGHWAY_ROAD_CLASSES = frozenset((0, 6))
LEFT_HIGHWAY_LANE_MANEUVERS = frozenset(("keepLeft", "slightLeft"))
RIGHT_HIGHWAY_LANE_MANEUVERS = frozenset(("keepRight", "slightRight"))


def controlled_access_observed(nav) -> bool:
  return bool(
    getattr(nav, "roadClass", -1) in HIGHWAY_ROAD_CLASSES
    or str(getattr(nav, "elevatedRoadStatus", "unknown")) in ("main", "side")
  )


class FinalForkScope:
  """Remember controlled-access evidence for one stable navigation maneuver."""

  def __init__(self) -> None:
    self._qualified_key: tuple[str, int, int] | None = None
    self._distance_key: tuple[str, int, int] | None = None
    self._last_distance_m: float | None = None
    self.entry_reached = False

  def update(self, nav, *, linked: bool) -> bool:
    event_key = (str(nav.sessionId), int(nav.routeRevision), int(nav.maneuverEventId))
    maneuver = str(nav.maneuver)
    if linked and event_key[2] != 0 and maneuver in LEFT_EXIT_LANE_MANEUVERS | RIGHT_EXIT_LANE_MANEUVERS:
      if event_key != self._distance_key:
        self._distance_key = event_key
        self._last_distance_m = None
        self.entry_reached = False
      if controlled_access_observed(nav):
        self._qualified_key = event_key
      distance_m = float(getattr(nav, "maneuverDistanceM", math.nan))
      if math.isfinite(distance_m) and 0.0 < distance_m:
        # AMap reports distance in discrete updates. Preserve jihui's 80 m
        # entry point, but arm one observed distance step before it when the
        # next equal step would cross 80 m; otherwise a 131 -> 90 -> event-end
        # sequence never reaches the final-fork branch.
        if distance_m < FORK_ENTRY_DISTANCE_M:
          self.entry_reached = True
        elif (distance_m > FORK_ENTRY_DISTANCE_M and self._last_distance_m is not None
              and self._last_distance_m > distance_m
              and self._last_distance_m - distance_m >= distance_m - FORK_ENTRY_DISTANCE_M):
          self.entry_reached = True
        if self._last_distance_m != distance_m:
          self._last_distance_m = distance_m
      return self._qualified_key == event_key
    return False


def selected_services_healthy(sm, services: tuple[str, ...]) -> bool:
  return all(sm.seen[service] and sm.alive[service] and sm.valid[service] for service in services)


def efficiency_radar_healthy(sm, now_ns: int) -> bool:
  radar = sm["radarTracks"]
  fresh = all(0 < sm.logMonoTime[s] <= now_ns and now_ns - sm.logMonoTime[s] <= 200_000_000
              for s in ("radarTracks", "modelV2"))
  return bool(selected_services_healthy(sm, ("radarTracks", "modelV2")) and fresh
              and abs(sm.logMonoTime["radarTracks"] - sm.logMonoTime["modelV2"]) <= 100_000_000
              and not any((radar.errors.canError, radar.errors.radarFault,
                           radar.errors.radarUnavailableTemporary, radar.errors.wrongConfig))
              and sm.seen["carParamsSP"] and sm.valid["carParamsSP"]
              and sm["carParamsSP"].flags & TeslaFlagsSP.ARS408_RADAR)


def navigation_linked(nav, *, base_healthy: bool) -> bool:
  return bool(
    base_healthy and not nav.stale and nav.routeActive and nav.routeMatched
    and str(nav.mode) == "realtime" and int(nav.maneuverEventId) != 0
  )


def neighbor_observation(topology, *, side: str, healthy: bool) -> bool | None:
  """Visible lane / confirmed road boundary / unknown, without a schema change."""
  if side not in ("left", "right"):
    raise ValueError("side must be left or right")
  if not healthy or not topology.validForControl:
    return None
  road_edge = (bool(getattr(topology, f"{side}EvidenceValid")) and
               str(getattr(topology, f"{side}EgoSideMarking")) == "roadEdge")
  if road_edge:
    return False
  return True if getattr(topology, f"{side}NeighborExists") else None


def build_lane_plan(nav, topology, *, healthy: bool, settings: NavAssistSettings | None = None,
                    lane_count_override: int | None = None, amap_ego_index: int | None = None,
                    oem_edge_position: str | None = None,
                    final_fork_allowed: bool | None = None,
                    final_fork_entry_reached: bool | None = None) -> NavLanePlan:
  settings = settings if settings is not None else NavAssistSettings()
  nav_valid = bool(healthy and nav.valid and not nav.stale and settings.enabled and settings.lane_change_enabled)
  lanes = tuple(nav.lanes)
  recommended = tuple(int(lane.index) for lane in lanes if lane.recommended and not getattr(lane, "routeAvoid", False))
  if not recommended and any(getattr(lane, "routeAvoid", False) for lane in lanes):
    # AMap may expose only route-avoided lanes (legacy F / modern 255) and no
    # explicit foreground recommendation. Preserve the full observation and
    # infer the complement only when C3 sees the same lane count.
    recommended = tuple(int(lane.index) for lane in lanes if not getattr(lane, "routeAvoid", False))
  maneuver = str(nav.maneuver)
  distance_m = float(nav.maneuverDistanceM)
  lane_count = int(topology.visibleLaneCount) if lane_count_override is None else lane_count_override
  fallback_side = None
  lookahead_m = 0.0
  highway = getattr(nav, 'roadClass', -1) in HIGHWAY_ROAD_CLASSES
  final_fork_allowed = controlled_access_observed(nav) if final_fork_allowed is None else final_fork_allowed
  final_fork_entry_reached = (math.isfinite(distance_m) and 0.0 < distance_m < FORK_ENTRY_DISTANCE_M
                              if final_fork_entry_reached is None else final_fork_entry_reached)
  if highway and maneuver in LEFT_HIGHWAY_LANE_MANEUVERS:
    fallback_side, lookahead_m = "left", settings.exit_lane_lookahead_m
  elif highway and maneuver in RIGHT_HIGHWAY_LANE_MANEUVERS:
    fallback_side, lookahead_m = "right", settings.exit_lane_lookahead_m
  elif maneuver in LEFT_TURN_LANE_MANEUVERS:
    fallback_side, lookahead_m = "left", settings.turn_lane_lookahead_m
  elif maneuver in RIGHT_TURN_LANE_MANEUVERS:
    fallback_side, lookahead_m = "right", settings.turn_lane_lookahead_m
  elif maneuver in LEFT_EXIT_LANE_MANEUVERS:
    fallback_side, lookahead_m = "left", settings.exit_lane_lookahead_m
  elif maneuver in RIGHT_EXIT_LANE_MANEUVERS:
    fallback_side, lookahead_m = "right", settings.exit_lane_lookahead_m
  if (nav_valid and final_fork_allowed and int(nav.maneuverEventId) != 0
      and maneuver in LEFT_EXIT_LANE_MANEUVERS | RIGHT_EXIT_LANE_MANEUVERS
      and final_fork_entry_reached):
    # jihui's 80 m doLaneForkNow is in its controlled-access branch. The main
    # loop retains earlier highway/elevated evidence because AMap may switch
    # current-link class to the ramp itself before entering this window.
    # Keep ordinary-road approach alignment below; only final fork is scoped.
    # A split need not already be classified as a complete adjacent lane.
    # Retain C3's visual/OEM crossing, road-edge, blindspot and radar gates.
    fork_lane_count = max(1, lane_count)
    return NavLanePlan(
      True, str(nav.sessionId), int(nav.routeRevision), int(nav.maneuverEventId), fork_lane_count,
      (0 if fallback_side == "left" else fork_lane_count - 1,), heuristic=True,
      edge_direction=LaneIntentDirection.left if fallback_side == "left" else LaneIntentDirection.right,
      force_fork=True, allow_unknown_crossing=False, ignore_solid_boundary=False,
      navigation_valid=nav_valid,
    )
  if lanes:
    amap_lane_count = len(lanes)
    # AMap's edge recommendation supplies direction even when its road class
    # is not 0/6. Keep maneuver-only fallback limited to the existing cases.
    explicit_edges = {int(lane.index) for lane in lanes if lane.recommended and not getattr(lane, "routeAvoid", False)}
    if fallback_side is None and maneuver in LEFT_HIGHWAY_LANE_MANEUVERS and 0 in explicit_edges:
      fallback_side, lookahead_m = "left", settings.exit_lane_lookahead_m
    elif fallback_side is None and maneuver in RIGHT_HIGHWAY_LANE_MANEUVERS and amap_lane_count - 1 in explicit_edges:
      fallback_side, lookahead_m = "right", settings.exit_lane_lookahead_m
    # The 0x239 edge position identifies the same outer lane even when vision
    # covers only a local window of a wider road. If it is recommended, stay.
    edge_current = (0 if oem_edge_position == "leftmost" else
                    amap_lane_count - 1 if oem_edge_position == "rightmost" else None)
    if nav_valid and edge_current in recommended and lane_count > 0:
      visual_current = 0 if oem_edge_position == "leftmost" else lane_count - 1
      return NavLanePlan(True, str(nav.sessionId), int(nav.routeRevision), int(nav.maneuverEventId),
                         lane_count, (visual_current,), navigation_valid=nav_valid)
    if (nav_valid and amap_ego_index is not None and amap_lane_count == lane_count
        and fallback_side is not None and math.isfinite(distance_m) and 0.0 <= distance_m <= lookahead_m):
      aligned = tuple(index for index in recommended if
                      (index <= amap_ego_index if fallback_side == "left" else index >= amap_ego_index))
      if aligned:
        return NavLanePlan(True, str(nav.sessionId), int(nav.routeRevision), int(nav.maneuverEventId),
                           lane_count, aligned, navigation_valid=nav_valid)
    edge_recommended = bool(
      (fallback_side == "left" and 0 in recommended)
      or (fallback_side == "right" and amap_lane_count - 1 in recommended)
    )
    if nav_valid and edge_recommended and math.isfinite(distance_m) and 0.0 <= distance_m <= lookahead_m:
      target = 0 if fallback_side == "left" else max(0, lane_count - 1)
      return NavLanePlan(
        True, str(nav.sessionId), int(nav.routeRevision), int(nav.maneuverEventId),
        lane_count, (target,), heuristic=True,
        edge_direction=LaneIntentDirection.left if fallback_side == "left" else LaneIntentDirection.right,
        # Near-exit alignment uses the normal OEM gates even inside 50 m.
        # It never forces a missing neighbor or bypasses a solid boundary.
        force_fork=False, allow_unknown_crossing=False, ignore_solid_boundary=False,
        navigation_valid=nav_valid,
      )
    # AMap lane indices describe the complete road while modelV2 exposes only a
    # local visible window. Without an edge-qualified directional target there
    # is no common absolute index, so retain the observation but do not control.
    return NavLanePlan(
      False, str(nav.sessionId), int(nav.routeRevision), int(nav.maneuverEventId), amap_lane_count, (),
      navigation_valid=nav_valid,
    )

  # A slight bend may simply follow the current road. Unlike explicit keep/exit
  # guidance it needs an edge-qualified lane recommendation before moving lanes.
  if maneuver in ("slightLeft", "slightRight"):
    fallback_side = None

  heuristic_valid = bool(
    nav_valid and int(nav.maneuverEventId) != 0 and fallback_side is not None
    and math.isfinite(distance_m) and 0.0 <= distance_m <= lookahead_m
  )
  if heuristic_valid:
    target = 0 if fallback_side == "left" else max(0, lane_count - 1)
    return NavLanePlan(
      True, str(nav.sessionId), int(nav.routeRevision), int(nav.maneuverEventId), lane_count, (target,), heuristic=True,
      edge_direction=LaneIntentDirection.left if fallback_side == "left" else LaneIntentDirection.right,
      force_fork=False, allow_unknown_crossing=False, ignore_solid_boundary=False,
      navigation_valid=nav_valid,
    )

  return NavLanePlan(
    nav_valid, str(nav.sessionId), int(nav.routeRevision), int(nav.maneuverEventId), len(lanes), recommended,
    navigation_valid=nav_valid,
  )


def visual_lane_geometry(topology, *, healthy: bool) -> tuple[bool, int, int, bool | None, bool | None]:
  """Use the control-valid visual topology for lane count and neighbors."""
  count = int(getattr(topology, "visibleLaneCount", 0))
  index = int(getattr(topology, "egoLaneIndexFromLeft", -1))
  valid = bool(healthy and topology.validForControl and count > 0 and 0 <= index < count)
  if not valid:
    return False, 0, -1, None, None
  return (True, count, index,
          neighbor_observation(topology, side="left", healthy=healthy),
          neighbor_observation(topology, side="right", healthy=healthy))


def anchored_amap_ego_index(topology, oem: dict, amap_lane_count: int) -> int | None:
  """Align AMap indices only from a confirmed road edge and matching visual count."""
  count = int(getattr(topology, "visibleLaneCount", 0))
  index = int(getattr(topology, "egoLaneIndexFromLeft", -1))
  if not (topology.validForControl and oem.get("positionValid", False)
          and count == amap_lane_count and 1 <= count <= 3 and 0 <= index < count):
    return None
  position = str(oem.get("position", "unknown"))
  if (position == "single" and count == 1 and index == 0 or
      position == "leftmost" and count >= 2 and index == 0 or
      position == "rightmost" and count >= 2 and index == count - 1):
    return index
  return None


def oem_crossing_allowed(topology, oem: dict, *, side: str, visual_healthy: bool, now_ns: int | None = None) -> bool:
  """Either 0x399 permission or visual evidence may allow crossing; known hazards veto."""
  if side not in ("left", "right"):
    raise ValueError("side must be left or right")
  permissions = lane_change_start_permissions(
    topology, healthy=visual_healthy, now_ns=time.monotonic_ns() if now_ns is None else now_ns,
    oem_permissions=(bool(oem.get("permissionValid") and oem.get("leftAllowed")),
                     bool(oem.get("permissionValid") and oem.get("rightAllowed"))),
    safety_blocks=(bool(oem.get("permissionValid") and oem.get("leftSafetyBlocked")),
                   bool(oem.get("permissionValid") and oem.get("rightSafetyBlocked"))),
  )
  return permissions[0 if side == "left" else 1]


def oem_gate_reason(plan: NavLanePlan, topology: LaneTopologyInput, oem: dict) -> str | None:
  """Expose the exact C3-local OEM gate that is holding a navigation request."""
  if not plan.valid or not plan.recommended_indices:
    return None
  direction = plan.edge_direction
  if direction == LaneIntentDirection.none:
    valid_targets = tuple(index for index in plan.recommended_indices if 0 <= index < plan.lane_count)
    target = min(valid_targets, key=lambda index: (abs(index - topology.ego_lane_index), index)) if valid_targets else None
    if target is not None and target < topology.ego_lane_index:
      direction = LaneIntentDirection.left
    elif target is not None and target > topology.ego_lane_index:
      direction = LaneIntentDirection.right
  if direction == LaneIntentDirection.none:
    return None
  side = "left" if direction == LaneIntentDirection.left else "right"
  neighbor = topology.left_neighbor_exists if side == "left" else topology.right_neighbor_exists
  if neighbor is not True and not plan.force_fork:
    return f"visualNo{side.title()}Neighbor"
  if (topology.left_crossing_allowed if side == "left" else topology.right_crossing_allowed):
    return None
  if not oem.get("permissionValid", False):
    return "oem399Stale"
  if not oem.get(f"{side}Allowed", False):
    return f"oem399{side.title()}Denied"
  if not (topology.left_crossing_allowed if side == "left" else topology.right_crossing_allowed):
    return f"visual{side.title()}BoundaryVeto"
  return None


def apply_oem_signal_gate(intent: NavLaneIntent, gate_reason: str | None, phase: str) -> NavLaneIntent:
  if gate_reason is None or intent.reason == "forkCompleted":
    return intent
  if phase in ("signaling", "ready", "observing", "changing") and intent.signal_requested:
    return replace(intent, reason=gate_reason)
  return NavLaneIntent(reason=gate_reason)


def lane_alignment_may_start(nav, turn_intent: NavLaneIntent, *, speed_mps: float) -> bool:
  distance_m = float(nav.maneuverDistanceM)
  approaching_turn = bool(
    turn_intent.signal_requested and str(nav.maneuver) in LEFT_TURN_LANE_MANEUVERS | RIGHT_TURN_LANE_MANEUVERS
  )
  return math.isfinite(distance_m) and distance_m > 0.0 and not approaching_turn


def navigation_change_pending(plan: NavLanePlan, topology: LaneTopologyInput) -> bool:
  if not plan.valid or not topology.valid_for_control or plan.lane_count != topology.visible_lane_count:
    return False
  valid_targets = tuple(index for index in plan.recommended_indices if 0 <= index < plan.lane_count)
  target = min(valid_targets, key=lambda index: (abs(index - topology.ego_lane_index), index)) if valid_targets else None
  return target is not None and target != topology.ego_lane_index


def navigation_lane_guidance_active(plan: NavLanePlan, nav) -> bool:
  """Keep efficiency changes out while a fresh route constrains the usable lanes."""
  if not plan.navigation_valid:
    return False
  lanes = tuple(nav.lanes)
  if not lanes:
    return False
  preferred = tuple(lane for lane in lanes if lane.recommended and not getattr(lane, "routeAvoid", False))
  avoided = tuple(lane for lane in lanes if getattr(lane, "routeAvoid", False))
  return bool(preferred or avoided) and not (len(preferred) == len(lanes) and not avoided)


def apply_radar_target_gate(intent: NavLaneIntent, radar, model, *, speed_mps: float, healthy: bool) -> NavLaneIntent:
  if not (healthy and intent.signal_requested and intent.lane_change_ready and intent.target_lane_index >= 0
          and intent.direction in (LaneIntentDirection.left, LaneIntentDirection.right)):
    return intent
  left = intent.direction == LaneIntentDirection.left
  targets = lane_leads(radar, model, left_neighbor=left, right_neighbor=not left, all_targets=True)
  if targets is None or not any(side_lead_unsafe(point, speed_mps) for point in targets[0 if left else 2]):
    return intent
  return replace(intent, lane_change_ready=False, reason="radarLeftTargetUnsafe" if left else "radarRightTargetUnsafe")


def main() -> None:
  from openpilot.cereal import messaging
  from openpilot.common.realtime import Ratekeeper

  settings_cache = SettingsCache()
  coordinator = NavLaneIntentCoordinator(max_changes=settings_cache.read().max_lane_changes)
  efficiency = EfficiencyLaneSelector()
  turn_signal_coordinator = NavTurnSignalCoordinator()
  final_fork_scope = FinalForkScope()
  oem_lane_feedback = OemLaneFeedback()
  sm = messaging.SubMaster(list(SERVICES), poll="navAssistStateSP")
  can_sock = messaging.sub_sock("can", conflate=False)
  pm = messaging.PubMaster(["navLaneIntentSP"])
  ratekeeper = Ratekeeper(PUBLISH_HZ)
  while True:
    sm.update(50)
    for event in messaging.drain_sock(can_sock):
      oem_lane_feedback.ingest(event.can, now_ns=int(event.logMonoTime), received_ns=time.monotonic_ns())
    now_ns = time.monotonic_ns()
    oem = oem_lane_feedback.snapshot(now_ns, include_topology_details=True)
    base_healthy = selected_services_healthy(sm, BASE_SERVICES)
    model_healthy = selected_services_healthy(sm, MODEL_SERVICES)
    visual_lane_healthy = selected_services_healthy(sm, VISUAL_LANE_SERVICES)
    nav = sm["navAssistStateSP"]
    settings = settings_cache.read()
    topology = sm["laneTopologyStateSP"]
    car_state = sm["carState"]
    car_control = sm["carControl"]
    model_meta = sm["modelV2"].meta
    geometry_valid, lane_count, ego_lane_index, left_neighbor, right_neighbor = visual_lane_geometry(
      topology, healthy=visual_lane_healthy,
    )
    # A brief lane-observation gap is not a navigation outage or a loss of
    # actual lateral control. The coordinator bounds it with its existing grace.
    nav_linked = navigation_linked(nav, base_healthy=base_healthy)
    final_fork_allowed = final_fork_scope.update(nav, linked=nav_linked)
    plan = build_lane_plan(
      nav, topology, healthy=base_healthy, settings=settings,
      lane_count_override=lane_count if geometry_valid else None,
      final_fork_allowed=final_fork_allowed,
      final_fork_entry_reached=final_fork_scope.entry_reached,
    )
    topology_input = LaneTopologyInput(
      valid_for_control=bool(base_healthy and model_healthy and geometry_valid),
      visible_lane_count=lane_count,
      ego_lane_index=ego_lane_index,
      left_neighbor_exists=left_neighbor,
      right_neighbor_exists=right_neighbor,
      left_crossing_allowed=oem_crossing_allowed(
        topology, oem, side="left", visual_healthy=visual_lane_healthy, now_ns=now_ns,
      ),
      right_crossing_allowed=oem_crossing_allowed(
        topology, oem, side="right", visual_healthy=visual_lane_healthy, now_ns=now_ns,
      ),
    )
    # Use the same held purpose for priority, gates and final lamp arbitration.
    plan = coordinator.hold_active_fork(plan, topology_input)
    vehicle = LaneVehicleInput(
      lateral_active=bool(base_healthy and car_control.latActive),
      speed_mps=float(car_state.vEgo),
      left_blindspot=bool(car_state.leftBlindspot),
      right_blindspot=bool(car_state.rightBlindspot),
      left_blinker=bool(car_state.leftBlinker),
      right_blinker=bool(car_state.rightBlinker),
      brake_pressed=bool(car_state.brakePressed),
      gas_pressed=bool(car_state.gasPressed),
      steering_pressed=bool(car_state.steeringPressed),
      lane_change_state=ObservedLaneChangeState(int(model_meta.laneChangeState.raw)),
      lane_change_direction=LaneIntentDirection(int(model_meta.laneChangeDirection.raw)),
      model_mono_time_ns=int(sm.logMonoTime["modelV2"]),
    )
    turn_plan = NavTurnPlan(
      valid=nav_linked,
      session_id=str(nav.sessionId),
      route_revision=int(nav.routeRevision),
      maneuver_event_id=int(nav.maneuverEventId),
      maneuver=str(nav.maneuver),
      distance_m=float(nav.maneuverDistanceM),
      source_interrupted=bool(base_healthy and not nav.stale and str(nav.mode) == "realtime"
                              and nav.routeMatched and not nav.routeActive and nav.maneuverEventId == 0),
    )
    turn_intent = turn_signal_coordinator.update(
      turn_plan, speed_mps=float(car_state.vEgo), now_ns=now_ns,
      turn_geometry_active=sp_turn_geometry_active(sm["controlsState"], float(car_state.vEgo)),
      lookahead_time_s=settings.signal_lead_time_s,
    )
    # Reuse ARS408's multi-target output. The selector may use an empty adjacent
    # forward corridor only with a measured ego lead and separate blindspot gate.
    # Radar/model timestamps must be fresh and close enough to align.
    radar = sm["radarTracks"]
    radar_healthy = efficiency_radar_healthy(sm, now_ns)
    route_reserved = (turn_intent.signal_requested or navigation_change_pending(plan, topology_input)
                      or navigation_lane_guidance_active(plan, nav))
    plan, efficiency_selected = efficiency.select(
      plan, nav, topology_input, vehicle, radar, sm["modelV2"], oem,
      healthy=bool(base_healthy and model_healthy and radar_healthy),
      execution_healthy=bool(base_healthy and model_healthy),
      enabled=settings.enabled and settings.lane_change_enabled and settings.efficiency_lane_change_enabled,
      cruise_mps=float(car_state.vCruise) / 3.6,
      route_reserved=route_reserved, now_ns=now_ns, settings=settings,
    )
    # Reuse the existing pre-turn window. An SP change already in progress
    # finishes before handoff; a further approach-lane change must not suppress
    # the model's turn desire at the intersection.
    gate_reason = oem_gate_reason(plan, topology_input, oem)
    lane_intent = coordinator.update(
      plan, topology_input, vehicle, now_ns=now_ns,
      allow_new_lane_change=(efficiency_selected or lane_alignment_may_start(nav, turn_intent, speed_mps=vehicle.speed_mps)),
      spoken_announcement_id=str(nav.laneChangeSpeechCompletedId) if base_healthy and nav.valid and not nav.stale else "",
    )
    if efficiency_selected:
      efficiency.observe(lane_intent, now_ns)
      lane_intent = replace(lane_intent, reason="efficiency:" + lane_intent.reason)
    lane_intent = apply_oem_signal_gate(lane_intent, gate_reason, coordinator._phase)
    lane_intent = apply_radar_target_gate(
      lane_intent, radar, sm["modelV2"], speed_mps=vehicle.speed_mps,
      healthy=bool(model_healthy and radar_healthy),
    )
    if lane_intent.signal_requested or coordinator.fork_terminal or plan.force_fork:
      intent = lane_intent
    elif settings.turn_signal_enabled and turn_intent.signal_requested:
      intent = turn_intent
    elif gate_reason is not None:
      intent = lane_intent
    else:
      intent = turn_intent if settings.turn_signal_enabled else NavLaneIntent(reason="turnSignalsDisabled")
      if settings.efficiency_lane_change_enabled and not intent.signal_requested:
        intent = replace(intent, reason=efficiency.reason)
    if not settings.enabled:
      intent = NavLaneIntent(reason="navigationDisabled")

    message = messaging.new_message("navLaneIntentSP")
    message.valid = base_healthy
    state = message.navLaneIntentSP
    state.publishMonoTime = now_ns
    state.valid = base_healthy
    state.signalRequested = intent.signal_requested
    state.laneChangeAuthorized = False
    state.spLaneChangeReady = intent.lane_change_ready
    state.announcementId = (coordinator.announcement_id if intent.signal_requested and intent.target_lane_index >= 0
                            and not intent.lane_change_ready and coordinator._phase == "signaling" else "")
    state.direction = {LaneIntentDirection.none: "none", LaneIntentDirection.left: "left",
                       LaneIntentDirection.right: "right"}[intent.direction]
    state.requestId = intent.request_id
    state.targetLaneIndex = intent.target_lane_index
    lane_request_active = bool(lane_intent.signal_requested and lane_intent.target_lane_index >= 0)
    state.forkNow = bool(lane_request_active and coordinator.fork_active)
    state.allowUnknownCrossing = bool(lane_request_active and plan.allow_unknown_crossing)
    state.ignoreSolidBoundary = bool(lane_request_active and plan.ignore_solid_boundary)
    state.routeRevision = plan.route_revision
    state.maneuverEventId = intent.request_id if intent.signal_requested and intent.target_lane_index < 0 else plan.maneuver_event_id
    state.reason = intent.reason
    state.sessionId = plan.session_id
    pm.send("navLaneIntentSP", message)
    ratekeeper.keep_time()


if __name__ == "__main__":
  main()
