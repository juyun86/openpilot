from __future__ import annotations

from collections.abc import Callable
from typing import Any

from openpilot.sunnypilot.selfdrive.radar_lane.occupancy import (
  MAX_FORWARD_DISTANCE_M,
  MIN_FORWARD_DISTANCE_M,
  LaneResult,
  RadarLaneResult,
  classify_radar_lanes,
  empty_result,
  radar_target_from_point,
)


# Three expected 20 Hz input periods. Replay/HIL should validate this budget on
# each target device before any consumer treats the output as time-sensitive.
MAX_INPUT_AGE_MS = 150.0


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


def _populate_lane(builder: Any, result: LaneResult) -> None:
  builder.occupancy = result.occupancy.value
  builder.geometrySource = result.geometry_source.value
  builder.geometryConfidence = result.geometry_confidence
  builder.evaluatedDistance = result.evaluated_distance
  builder.targetCount = min(len(result.targets), 0xFFFF)
  if not result.targets:
    return
  target = result.targets[0]
  builder.closestTarget = {
    "present": True,
    "trackId": target.track_id,
    "dRel": target.d_rel,
    "yRel": target.y_rel,
    "vRel": target.v_rel,
    "dPath": target.d_path,
    "ambiguous": target.ambiguous,
    "measured": target.measured,
  }


class RadarLaneStatePublisher:
  def __init__(self, radar_to_camera: float, message_factory: Callable[[str], Any] | None = None,
               radar_available: bool = True) -> None:
    self.radar_to_camera = radar_to_camera
    self.message_factory = message_factory
    self.radar_available = radar_available

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
    _populate_lane(state.leftAhead, result.left)
    _populate_lane(state.centerAhead, result.center)
    _populate_lane(state.rightAhead, result.right)
    return message

  def publish(self, pm: Any, sm: Any, radar_data: Any) -> None:
    pm.send("radarLaneStateSP", self.build_message(sm, radar_data))
