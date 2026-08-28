from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from typing import Any

from openpilot.sunnypilot.selfdrive.radar_lane.occupancy import (
  LANE_LEFT_MASK,
  LANE_RIGHT_MASK,
  MAX_FORWARD_DISTANCE_M,
  MIN_FORWARD_DISTANCE_M,
  ClassifiedTarget,
  LaneResult,
  RadarLaneResult,
  classify_radar_lanes,
  empty_result,
  radar_target_from_point,
)


# Three expected 20 Hz input periods. Replay/HIL should validate this budget on
# each target device before any consumer treats the output as time-sensitive.
MAX_INPUT_AGE_MS = 150.0
MAX_PUBLISHED_TARGETS = 24
PREDICTION_HORIZON_S = 3.0
MIN_MOTION_DT_S = 0.03
MAX_MOTION_DT_S = 0.25
MAX_LATERAL_SPEED_MPS = 8.0
MIN_CUT_IN_SPEED_MPS = 0.25
MOTION_FILTER_ALPHA = 0.5


@dataclass(frozen=True)
class TargetMotion:
  lateral_speed: float = 0.0
  valid: bool = False
  predicted_d_path: float = 0.0
  time_to_lane_cross: float = -1.0
  cut_in_candidate: bool = False


@dataclass(frozen=True)
class _MotionState:
  radar_mono_time: int
  d_path: float
  lateral_speed: float = 0.0
  valid: bool = False


class TargetMotionTracker:
  def __init__(self) -> None:
    self._states: dict[int, _MotionState] = {}
    self._last_radar_mono_time = 0

  @staticmethod
  def _estimate(target: ClassifiedTarget, state: _MotionState | None) -> TargetMotion:
    if state is None or not state.valid:
      return TargetMotion(predicted_d_path=target.d_path)

    predicted_d_path = target.d_path + state.lateral_speed * PREDICTION_HORIZON_S
    toward_center_speed = 0.0
    if target.lane_mask & LANE_LEFT_MASK:
      toward_center_speed = max(toward_center_speed, -state.lateral_speed)
    if target.lane_mask & LANE_RIGHT_MASK:
      toward_center_speed = max(toward_center_speed, state.lateral_speed)

    time_to_lane_cross = -1.0
    cut_in_candidate = False
    if target.center_boundary_distance >= 0.0 and toward_center_speed >= MIN_CUT_IN_SPEED_MPS:
      time_to_lane_cross = target.center_boundary_distance / toward_center_speed
      cut_in_candidate = time_to_lane_cross <= PREDICTION_HORIZON_S

    return TargetMotion(
      state.lateral_speed,
      True,
      predicted_d_path,
      time_to_lane_cross,
      cut_in_candidate,
    )

  def update(self, targets: tuple[ClassifiedTarget, ...], radar_mono_time: int) -> dict[int, TargetMotion]:
    target_by_id = {target.track_id: target for target in targets}
    if radar_mono_time < self._last_radar_mono_time:
      self._states.clear()
      self._last_radar_mono_time = 0

    if radar_mono_time > self._last_radar_mono_time:
      next_states: dict[int, _MotionState] = {}
      for track_id, target in target_by_id.items():
        previous = self._states.get(track_id)
        if not target.measured and previous is not None:
          next_states[track_id] = previous
          continue

        state = _MotionState(radar_mono_time, target.d_path)
        if target.measured and previous is not None and radar_mono_time > previous.radar_mono_time:
          dt = (radar_mono_time - previous.radar_mono_time) * 1e-9
          raw_speed = (target.d_path - previous.d_path) / dt
          if MIN_MOTION_DT_S <= dt <= MAX_MOTION_DT_S and math.isfinite(raw_speed) and abs(raw_speed) <= MAX_LATERAL_SPEED_MPS:
            lateral_speed = raw_speed
            if previous.valid:
              lateral_speed = MOTION_FILTER_ALPHA * raw_speed + (1.0 - MOTION_FILTER_ALPHA) * previous.lateral_speed
            state = _MotionState(radar_mono_time, target.d_path, lateral_speed, True)
        next_states[track_id] = state
      self._states = next_states
      self._last_radar_mono_time = radar_mono_time

    return {
      track_id: self._estimate(target, self._states.get(track_id))
      for track_id, target in target_by_id.items()
    }


def _service_flag(sm: Any, attribute: str, service: str) -> bool:
  values = getattr(sm, attribute, None)
  if values is None:
    return True
  try:
    return bool(values[service])
  except (KeyError, TypeError):
    return True


def populate_input_status(state: Any, sm: Any) -> tuple[int, int]:
  model_mono_time = int(sm.logMonoTime.get("modelV2", 0))
  radar_mono_time = int(sm.logMonoTime.get("radarTracks", 0))
  latest_mono_time = max((int(value) for value in sm.logMonoTime.values()), default=0)
  model_age_ms = max(0.0, (latest_mono_time - model_mono_time) * 1e-6) if model_mono_time else -1.0
  radar_age_ms = max(0.0, (latest_mono_time - radar_mono_time) * 1e-6) if radar_mono_time else -1.0

  state.modelMonoTime = model_mono_time
  state.radarMonoTime = radar_mono_time
  state.radarAgeMs = radar_age_ms
  state.radarFresh = (
    0.0 <= radar_age_ms <= MAX_INPUT_AGE_MS and
    _service_flag(sm, "alive", "radarTracks") and _service_flag(sm, "valid", "radarTracks")
  )
  state.modelFresh = (
    0.0 <= model_age_ms <= MAX_INPUT_AGE_MS and
    _service_flag(sm, "alive", "modelV2") and _service_flag(sm, "valid", "modelV2")
  )
  state.forwardMinDistance = MIN_FORWARD_DISTANCE_M
  state.forwardMaxDistance = MAX_FORWARD_DISTANCE_M
  return model_mono_time, radar_mono_time


def _radar_has_error(radar_data: Any) -> bool:
  errors = getattr(radar_data, "errors", None)
  if errors is None:
    return False
  try:
    return any(bool(value) for value in errors.to_dict().values())
  except AttributeError:
    return any(bool(getattr(errors, name, False)) for name in (
      "canError", "radarFault", "wrongConfig", "radarUnavailableTemporary",
    ))


def _target_dict(target: ClassifiedTarget, motion: TargetMotion | None = None) -> dict[str, Any]:
  motion = motion or TargetMotion(predicted_d_path=target.d_path)
  return {
    "present": True,
    "trackId": target.track_id,
    "dRel": target.d_rel,
    "yRel": target.y_rel,
    "vRel": target.v_rel,
    "dPath": target.d_path,
    "ambiguous": target.ambiguous,
    "measured": target.measured,
    "yvRel": target.yv_rel,
    "yvRelValid": target.yv_rel_valid,
    "laneMask": target.lane_mask,
    "lateralSpeed": motion.lateral_speed,
    "lateralSpeedValid": motion.valid,
    "predictedDPath": motion.predicted_d_path,
    "timeToLaneCross": motion.time_to_lane_cross,
    "cutInCandidate": motion.cut_in_candidate,
  }


def _populate_lane(builder: Any, result: LaneResult, motions: dict[int, TargetMotion]) -> None:
  builder.occupancy = result.occupancy.value
  builder.geometrySource = result.geometry_source.value
  builder.geometryConfidence = result.geometry_confidence
  builder.evaluatedDistance = result.evaluated_distance
  builder.targetCount = min(len(result.targets), 0xFFFF)
  if not result.targets:
    return
  target = result.targets[0]
  builder.closestTarget = _target_dict(target, motions.get(target.track_id))


def _unique_targets(result: RadarLaneResult) -> tuple[ClassifiedTarget, ...]:
  target_by_id: dict[int, ClassifiedTarget] = {}
  for lane in (result.left, result.center, result.right):
    for target in lane.targets:
      target_by_id[target.track_id] = target
  return tuple(target_by_id.values())


def _published_target_sort_key(target: ClassifiedTarget, motions: dict[int, TargetMotion]) -> tuple:
  motion = motions.get(target.track_id, TargetMotion())
  crossing_time = motion.time_to_lane_cross if motion.cut_in_candidate else math.inf
  return not motion.cut_in_candidate, crossing_time, target.d_rel, target.track_id


class RadarLaneStatePublisher:
  def __init__(self, radar_to_camera: float, message_factory: Callable[[str], Any] | None = None,
               radar_available: bool = True) -> None:
    self.radar_to_camera = radar_to_camera
    self.message_factory = message_factory
    self.radar_available = radar_available
    self.motion_tracker = TargetMotionTracker()

  def build_message(self, sm: Any, radar_data: Any) -> Any:
    if self.message_factory is None:
      from openpilot.cereal import messaging
      message = messaging.new_message("radarLaneStateSP")
    else:
      message = self.message_factory("radarLaneStateSP")
    state = message.radarLaneStateSP
    model_mono_time, radar_mono_time = populate_input_status(state, sm)

    radar_error = _radar_has_error(radar_data)
    if radar_error:
      state.radarFresh = False
    healthy = (
      self.radar_available and bool(sm.all_checks()) and state.modelFresh and state.radarFresh and
      model_mono_time > 0 and radar_mono_time > 0 and not radar_error
    )
    result: RadarLaneResult = empty_result()
    if healthy:
      targets = tuple(
        target for target in (radar_target_from_point(point) for point in radar_data.points)
        if target is not None
      )
      result = classify_radar_lanes(sm["modelV2"], targets, self.radar_to_camera)

    message.valid = healthy and result.valid
    unique_targets = _unique_targets(result)
    motions = self.motion_tracker.update(unique_targets, radar_mono_time)
    ordered_targets = tuple(sorted(unique_targets, key=lambda target: _published_target_sort_key(target, motions)))
    published_targets = ordered_targets[:MAX_PUBLISHED_TARGETS]
    cut_in_candidates = tuple(target for target in ordered_targets if motions[target.track_id].cut_in_candidate)

    state.uniqueTargetCount = min(len(unique_targets), 0xFFFF)
    state.publishedTargetLimit = MAX_PUBLISHED_TARGETS
    state.targetsTruncated = len(unique_targets) > MAX_PUBLISHED_TARGETS
    state.targets = [_target_dict(target, motions.get(target.track_id)) for target in published_targets]
    state.cutInCandidateCount = min(len(cut_in_candidates), 0xFFFF)
    state.predictionHorizon = PREDICTION_HORIZON_S
    if cut_in_candidates:
      candidate = cut_in_candidates[0]
      state.cutInCandidate = _target_dict(candidate, motions[candidate.track_id])

    _populate_lane(state.leftAhead, result.left, motions)
    _populate_lane(state.centerAhead, result.center, motions)
    _populate_lane(state.rightAhead, result.right, motions)
    return message

  def publish(self, pm: Any, sm: Any, radar_data: Any) -> None:
    pm.send("radarLaneStateSP", self.build_message(sm, radar_data))
