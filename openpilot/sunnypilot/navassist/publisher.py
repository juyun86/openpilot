from __future__ import annotations

from openpilot.cereal import messaging
from openpilot.sunnypilot.navassist.protocol import AcceptedSnapshot


SOURCE_TO_CEREAL = {"android": "android", "ios": "ios", "track": "track"}
MODE_TO_CEREAL = {
  "idle": "idle",
  "route_planned": "routePlanned",
  "realtime": "realtime",
  "simulation": "simulation",
  "arrived": "arrived",
  "recalculating": "recalculating",
}
COORDINATE_TO_CEREAL = {"unknown": "unknown", "gcj02": "gcj02", "wgs84": "wgs84"}
MANEUVER_TO_CEREAL = {
  "none": "none", "straight": "straight", "slight_left": "slightLeft", "slight_right": "slightRight",
  "turn_left": "turnLeft", "turn_right": "turnRight", "sharp_left": "sharpLeft", "sharp_right": "sharpRight",
  "u_turn_left": "uTurnLeft", "u_turn_right": "uTurnRight", "keep_left": "keepLeft", "keep_right": "keepRight",
  "merge_left": "mergeLeft", "merge_right": "mergeRight", "exit_left": "exitLeft", "exit_right": "exitRight",
  "ramp_left": "rampLeft", "ramp_right": "rampRight", "roundabout": "roundabout", "destination": "destination",
  "unknown": "unknown",
}
ROUTE_NOTICE_TO_CEREAL = {
  "none": "none", "restricted_area": "restrictedArea", "forbidden_area": "forbiddenArea",
  "road_closed": "roadClosed", "congestion": "congestion", "dispatch": "dispatch",
  "route_changed": "routeChanged", "gps_weak": "gpsWeak", "unknown": "unknown",
}

MAX_PHONE_LOCATION_AGE_MS = 1_000
MAX_ROUTE_NOTICE_AGE_MS = 30_000
MAX_PHONE_LOCATION_ACCURACY_M = 25.0


def _phone_observations_valid(snapshot) -> bool:
  location_age_ms = snapshot.source_wall_time_ms - snapshot.location_observed_at_ms
  return bool(
    snapshot.coordinate_system != "unknown"
    and snapshot.accuracy_m <= MAX_PHONE_LOCATION_ACCURACY_M
    and 0 <= location_age_ms <= MAX_PHONE_LOCATION_AGE_MS
  )


def build_nav_assist_message(current: AcceptedSnapshot | None, now_ns: int, *, local_localization_valid: bool = False):
  message = messaging.new_message("navAssistStateSP")
  state = message.navAssistStateSP
  state.publishMonoTime = now_ns
  if current is None:
    message.valid = False
    state.valid = False
    state.stale = True
    state.rejectReason = "noData"
    return message

  snapshot = current.snapshot
  stale = current.is_stale(now_ns)
  message.valid = True
  state.receiveMonoTime = current.receive_mono_ns
  state.sourceWallTimeMs = snapshot.source_wall_time_ms
  state.sequence = snapshot.sequence
  state.routeRevision = snapshot.route_revision
  state.maneuverEventId = snapshot.maneuver_event_id
  state.sessionId = snapshot.session_id
  state.laneChangeSpeechCompletedId = "" if stale else snapshot.lane_change_speech_completed_id
  state.source = SOURCE_TO_CEREAL[snapshot.source_platform]
  state.mode = MODE_TO_CEREAL[snapshot.navigation_mode]
  state.coordinateSystem = COORDINATE_TO_CEREAL[snapshot.coordinate_system]
  state.stale = stale
  state.routeActive = snapshot.route_active
  state.routeMatched = snapshot.route_matched
  state.gpsWeak = snapshot.gps_weak
  state.latitude = snapshot.latitude
  state.longitude = snapshot.longitude
  state.locationAccuracyM = snapshot.accuracy_m
  state.bearingDeg = snapshot.bearing_deg
  state.speedKph = snapshot.speed_kph
  state.locationObservedAtMs = snapshot.location_observed_at_ms
  state.currentStepIndex = snapshot.current_step_index
  state.currentLinkIndex = snapshot.current_link_index
  state.currentPointIndex = snapshot.current_point_index
  state.maneuver = MANEUVER_TO_CEREAL[snapshot.maneuver]
  state.guidanceObservedAtMs = snapshot.guidance_observed_at_ms
  state.maneuverDistanceM = snapshot.maneuver_distance_m
  state.nextManeuver = MANEUVER_TO_CEREAL[snapshot.next_maneuver]
  state.nextManeuverDistanceM = snapshot.next_maneuver_distance_m
  state.advisorySpeedValid = snapshot.advisory_speed_mps is not None
  state.advisorySpeedMps = snapshot.advisory_speed_mps or 0.0
  state.roadClass = snapshot.road_class
  state.roadType = snapshot.road_type
  state.currentRoad = snapshot.current_road
  state.nextRoad = snapshot.next_road
  state.parallelRoadStatus = snapshot.parallel_road_status
  state.elevatedRoadStatus = snapshot.elevated_road_status
  route_notice_age_ms = snapshot.source_wall_time_ms - snapshot.route_notice_observed_at_ms
  route_notice_valid = bool(
    snapshot.route_notice_observed_at_ms > 0 and 0 <= route_notice_age_ms <= MAX_ROUTE_NOTICE_AGE_MS
  )
  state.routeNoticeType = ROUTE_NOTICE_TO_CEREAL[snapshot.route_notice_type if route_notice_valid else "none"]
  state.routeNoticeDistanceM = snapshot.route_notice_distance_m if route_notice_valid else 0.0
  state.routeNoticeObservedAtMs = snapshot.route_notice_observed_at_ms
  state.laneGuidanceObservedAtMs = snapshot.lane_guidance_observed_at_ms
  state.sourceAgeMs = current.age_ms(now_ns)
  lane_guidance_age_ms = snapshot.source_wall_time_ms - snapshot.lane_guidance_observed_at_ms
  # AMap lane guidance is held between show/hide events. Each full snapshot
  # carries the current lanes; its transport TTL owns liveness, not callback age.
  lane_guidance_valid = bool(
    not stale and snapshot.lane_guidance_present and lane_guidance_age_ms >= 0
  )
  lanes = state.init("lanes", len(snapshot.lanes) if lane_guidance_valid else 0)
  for target, source in zip(lanes, snapshot.lanes if lane_guidance_valid else (), strict=True):
    target.index = source.index
    target.allowedActions = source.allowed_actions
    target.recommendedActions = source.recommended_actions
    target.recommended = source.recommended
    target.routeAvoid = source.route_avoid

  phone_observations_valid = _phone_observations_valid(snapshot)
  control_source_valid = snapshot.source_platform in ("android", "ios") and snapshot.navigation_mode == "realtime"
  # The phone SDK owns route progress and maneuver distance. C3XL localization
  # and phone observation quality remain visible diagnostics, but they do not
  # veto an otherwise fresh, matched realtime snapshot. AMap does not promise
  # periodic guidance callbacks when an instruction is unchanged, so the
  # receiver-assigned snapshot TTL—not guidanceObservedAtMs—is the liveness gate.
  state.valid = bool(not stale and control_source_valid and snapshot.route_active and snapshot.route_matched
                     and snapshot.location_present and snapshot.guidance_present)
  if stale:
    state.rejectReason = "stale"
  elif not control_source_valid:
    state.rejectReason = "noData"
  elif not snapshot.route_active:
    state.rejectReason = "noData"
  elif not snapshot.route_matched:
    state.rejectReason = "routeUnmatched"
  elif not snapshot.location_present or not snapshot.guidance_present:
    state.rejectReason = "noData"
  elif not phone_observations_valid:
    state.rejectReason = "phoneLocalization"
  elif not local_localization_valid:
    state.rejectReason = "localLocalization"
  else:
    state.rejectReason = "none"
  return message
