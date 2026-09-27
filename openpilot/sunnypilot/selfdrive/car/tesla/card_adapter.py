"""Thin card Adapter for Tesla state-machine context.

The vehicle state machine remains in opendbc. This Module supplies fresh
planner/control context and observes the original speed-wheel template without
adding Tesla branches throughout generic card.
"""

import hashlib
import math
import time
from typing import Any

from opendbc.sunnypilot.car.tesla.values import TeslaSafetyFlagsSP
from opendbc.sunnypilot.car.tesla.ars408.transmitter import MOTION_MAX_AGE_NS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator
from openpilot.sunnypilot.selfdrive.car.tesla.ambient_lighting import AmbientLightingController
from openpilot.sunnypilot.selfdrive.car.tesla.validation_controller import TeslaTurnSignalRealtimeController
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import (
  TeslaTrafficControlObserver, publish_tesla_traffic_control,
)


CONTEXT_STALE_S = 0.2
NAV_SIGNAL_SESSION_TIMEOUT_NS = 60_000_000_000
NAV_SIGNAL_RETRY_NS = 500_000_000
LIGHTING_STALE_NS = 2_000_000_000
LIGHTING_MESSAGE = "ID3F5VCFRONT_lighting"
CONTEXT_SERVICES = ("selfdriveStateSP", "modelV2", "navLaneIntentSP", "deviceMotion", "extrinsicsCalibration")
RADAR_CALIBRATION_MAX_AGE_NS = 2_000_000_000


def radar_yaw_rate_context(sm, now_ns: int, calibrator: PoseCalibrator, cache: dict | None = None) -> tuple[float | None, int]:
  for service, max_age in (("deviceMotion", MOTION_MAX_AGE_NS), ("extrinsicsCalibration", RADAR_CALIBRATION_MAX_AGE_NS)):
    if (not sm.seen[service] or not sm.valid[service] or sm.logMonoTime[service] <= 0 or
        not 0 <= now_ns - sm.logMonoTime[service] <= max_age):
      return None, 0

  motion = sm["deviceMotion"]
  calibration = sm["extrinsicsCalibration"]
  angular = motion.angularVelocityDevice
  rpy = tuple(calibration.rpyCalib)
  xyz = (angular.x, angular.y, angular.z)
  if (not (motion.inputsOK and motion.sensorsOK and motion.posenetOK and angular.valid) or
      motion.timestamp <= 0 or not 0 <= now_ns - motion.timestamp <= MOTION_MAX_AGE_NS or
      len(rpy) != 3 or not all(math.isfinite(v) for v in rpy) or
      not all(math.isfinite(v) for v in xyz)):
    return None, 0

  # Check freshness/health above even when returning a cached measurement.
  calibration_key = (sm.logMonoTime["extrinsicsCalibration"], rpy, str(calibration.calStatus))
  if cache is None or cache.get("calibration_key") != calibration_key:
    calibrator.feed_extrinsics_calibration(calibration)
    if cache is not None:
      cache["calibration_key"] = calibration_key
  if not calibrator.calib_valid:
    return None, 0
  motion_key = (calibration_key, sm.logMonoTime["deviceMotion"], motion.timestamp, xyz)
  if cache is not None and cache.get("motion_key") == motion_key:
    return cache["result"]
  # Only the calibrated angular z component is needed, not a complete Pose
  # or its velocity/acceleration/orientation/covariance transformations.
  yaw_rate = float(calibrator.calib_from_device[2] @ xyz)
  result = (yaw_rate, min(motion.timestamp, sm.logMonoTime["deviceMotion"])) if math.isfinite(yaw_rate) else (None, 0)
  if cache is not None:
    cache.update(motion_key=motion_key, result=result)
  return result


def longitudinal_context(sm, now: float) -> tuple[int, bool, bool, float, bool, bool, bool, float, bool, float, bool]:
  plan = sm["longitudinalPlanSP"]
  plan_source = int(getattr(plan.longitudinalPlanSource, "raw", plan.longitudinalPlanSource))
  plan_recv_time = float(sm.recv_time["longitudinalPlanSP"])
  plan_valid = (sm.seen["longitudinalPlanSP"] and sm.valid["longitudinalPlanSP"] and
                now - plan_recv_time <= CONTEXT_STALE_S)

  car_control = sm["carControl"]
  car_control_valid = (sm.seen["carControl"] and sm.valid["carControl"] and
                       now - sm.recv_time["carControl"] <= CONTEXT_STALE_S)
  lane_change_active = bool(car_control.leftBlinker or car_control.rightBlinker)

  selfdrive_state_sp = sm["selfdriveStateSP"]
  mads_state_valid = (sm.seen["selfdriveStateSP"] and sm.valid["selfdriveStateSP"] and
                      now - sm.recv_time["selfdriveStateSP"] <= CONTEXT_STALE_S)
  lateral_control_ready = ((car_control_valid and bool(car_control.latActive)) or
                           (mads_state_valid and bool(selfdrive_state_sp.mads.active)))

  return (plan_source, sm.updated["longitudinalPlanSP"], plan_valid, plan_recv_time,
          lane_change_active, car_control_valid, lateral_control_ready, now,
          bool(car_control.longActive), float(car_control.actuators.accel), car_control_valid)


def speed_limit_context(sm, now: float, assist_configured: bool | None = None) -> tuple[float, bool]:
  plan = sm["longitudinalPlanSP"]
  plan_recv_time = float(sm.recv_time["longitudinalPlanSP"])
  plan_valid = (sm.seen["longitudinalPlanSP"] and sm.valid["longitudinalPlanSP"] and
                now - plan_recv_time <= CONTEXT_STALE_S)
  resolver = plan.speedLimit.resolver
  limit_valid = bool(resolver.speedLimitValid or resolver.speedLimitLastValid)
  target = float(resolver.speedLimitFinalLast)
  configured = bool(plan.speedLimit.assist.enabled) if assist_configured is None else assist_configured
  valid = plan_valid and configured and limit_valid and target > 0.0
  return (target if valid else 0.0, valid)


class TeslaCardAdapter:
  SPEED_BUTTON_ADDRESS = 0x3C2
  VEHICLE_BUS = 1

  def __init__(self, brand: str, car_interface: Any, submaster: Any):
    self.enabled = brand == "tesla"
    self.car_interface = car_interface
    self.sm = submaster
    self.radar_transmitter = getattr(getattr(car_interface, "CC", None), "ars408_transmitter", None) if self.enabled else None
    self.radar_pose_calibrator = PoseCalibrator() if self.radar_transmitter is not None and self.radar_transmitter.enabled else None
    self.radar_yaw_cache: dict = {}
    self.traffic_control_observer = TeslaTrafficControlObserver() if self.enabled else None
    self.road_context_parser = self._create_road_context_parser() if self.enabled else None
    self.lighting_parser = self._create_lighting_parser() if self.enabled else None
    self.speed_limit_assist_configured: bool | None = None
    configured = bool(getattr(car_interface, "CP_SP", None) and
                      car_interface.CP_SP.safetyParam & TeslaSafetyFlagsSP.TURN_SIGNAL_VALIDATION)
    self.validation = TeslaTurnSignalRealtimeController(configured) if self.enabled else None
    self._last_nav_signal_request: tuple[str, int, int, str] | None = None
    self._active_nav_signal_test_id: str | None = None
    self._nav_signal_retry_after_ns = 0
    self.ambient = AmbientLightingController() if self.enabled else None

  def _create_road_context_parser(self):
    try:
      from opendbc.can import CANParser
      from opendbc.car import Bus
      from opendbc.car.tesla.values import CANBUS, DBC

      fingerprint = self.car_interface.CP.carFingerprint
      return CANParser(DBC[fingerprint][Bus.party], [("DAS_road", float("nan"))], CANBUS.party)
    except (AttributeError, KeyError):
      return None

  def _create_lighting_parser(self):
    try:
      from opendbc.can import CANParser
      from opendbc.car import Bus
      from opendbc.car.tesla.values import CANBUS, DBC

      fingerprint = self.car_interface.CP.carFingerprint
      return CANParser(DBC[fingerprint][Bus.adas], [(LIGHTING_MESSAGE, float("nan"))], CANBUS.vehicle)
    except (AttributeError, KeyError):
      return None

  def _night_mode(self, now_ns: int) -> bool:
    if self.lighting_parser is None:
      return True
    timestamps = self.lighting_parser.ts_nanos.get(LIGHTING_MESSAGE, {})
    timestamp = max(timestamps.values(), default=0)
    if timestamp <= 0 or not 0 <= now_ns - timestamp <= LIGHTING_STALE_NS:
      return True
    values = self.lighting_parser.vl[LIGHTING_MESSAGE]
    low_beam_on = (int(values["VCFRONT_lowBeamLeftStatus"]) == 1 or
                   int(values["VCFRONT_lowBeamRightStatus"]) == 1)
    return low_beam_on and not bool(values["VCFRONT_lowBeamsOnForDRL"])

  def observe_can(self, can_list) -> list:
    if not self.enabled:
      return []

    if self.traffic_control_observer is not None:
      self.traffic_control_observer.update(can_list, time.monotonic_ns())
    if self.road_context_parser is not None:
      self.road_context_parser.update(can_list)
    if self.lighting_parser is not None:
      self.lighting_parser.update(can_list)

    state = getattr(self.car_interface, "CS", None)
    update_template = getattr(state, "update_speed_button_template", None)

    for mono_time, frames in can_list:
      for address, data, source in frames:
        if self.ambient is not None:
          self.ambient.observe_frame(mono_time, address, data, source)
        if self.validation is not None:
          self.validation.observe_frame(mono_time, address, data, source)
        if update_template is not None and source == self.VEHICLE_BUS and address == self.SPEED_BUTTON_ADDRESS:
          update_template(data, mono_time)

    if self.validation is None:
      return []
    now_nanos = time.monotonic_ns()
    self.validation.advance_time(now_nanos)
    # Cancellation cannot depend on controlsd continuing to publish carControl.
    return self.validation.take_can_sends(now_nanos, cancel_only=True) + self.ambient.take_can_sends(now_nanos)

  def control_sends(self, car_state, car_control, now_nanos: int) -> list:
    if self.validation is None:
      return []
    self._update_nav_turn_signal(now_nanos, lateral_active=bool(car_control.latActive))
    now = time.monotonic()
    model_valid = (self.sm.seen["modelV2"] and self.sm.valid["modelV2"] and
                   now - self.sm.recv_time["modelV2"] <= CONTEXT_STALE_S)
    lane_change = self.sm["modelV2"].meta
    self.validation.update_lane_change_context(
      now_nanos,
      valid=model_valid,
      state=int(getattr(lane_change.laneChangeState, "raw", lane_change.laneChangeState)),
      direction=int(getattr(lane_change.laneChangeDirection, "raw", lane_change.laneChangeDirection)),
      lateral_active=bool(car_control.latActive),
      brake_pressed=bool(car_state.brakePressed),
    )
    state = getattr(self.car_interface, "CS", None)
    left_level = int(getattr(state, "tesla_blindspot_left_level", 2 if getattr(car_state, "leftBlindspot", False) else 0))
    right_level = int(getattr(state, "tesla_blindspot_right_level", 2 if getattr(car_state, "rightBlindspot", False) else 0))
    # Both valid OEM warnings flash red; these enums are not measured distances.
    # Change only the accessory display, keeping the vehicle blindspot state intact.
    left_level = 2 if left_level in (1, 2) else 0
    right_level = 2 if right_level in (1, 2) else 0
    self.ambient.update_blindspot(left_level, right_level, self._night_mode(now_nanos), now_nanos)
    return self.validation.take_can_sends(now_nanos) + self.ambient.take_can_sends(now_nanos)

  def _update_nav_turn_signal(self, now_nanos: int, *, lateral_active: bool = True) -> None:
    if self.validation is None:
      return
    status_fn = getattr(self.validation, "status", None)
    if self._active_nav_signal_test_id is not None and callable(status_fn):
      status = status_fn()
      if status is None or status.get("test_id") != self._active_nav_signal_test_id:
        # The realtime controller can finish a session asynchronously after a
        # context loss. Clear the adapter-side ownership so the same still-live
        # navigation event may retry when lateral control becomes available.
        self._active_nav_signal_test_id = None
        self._last_nav_signal_request = None
    now = time.monotonic()
    service = "navLaneIntentSP"
    fresh = bool(
      self.sm.seen[service] and self.sm.alive[service] and self.sm.valid[service] and
      0 <= now - self.sm.recv_time[service] <= CONTEXT_STALE_S
    )
    intent = self.sm[service]
    direction = str(intent.direction) if fresh else "none"
    requested = bool(fresh and intent.valid and intent.signalRequested and direction in ("left", "right"))
    if not requested:
      if self._active_nav_signal_test_id is not None:
        self.validation.request_cancel(self._active_nav_signal_test_id, now_nanos)
        self._active_nav_signal_test_id = None
      self._last_nav_signal_request = None
      self._nav_signal_retry_after_ns = 0
      return

    if not lateral_active:
      if self._active_nav_signal_test_id is not None:
        self.validation.request_cancel(self._active_nav_signal_test_id, now_nanos)
      self._active_nav_signal_test_id = None
      self._last_nav_signal_request = None
      self._nav_signal_retry_after_ns = 0
      return

    session_id = str(intent.sessionId)
    key = (session_id, int(intent.routeRevision), int(intent.requestId), direction)
    if key == self._last_nav_signal_request:
      return
    if self._active_nav_signal_test_id is not None and self._last_nav_signal_request is not None:
      previous_session, previous_revision, _previous_request, previous_direction = self._last_nav_signal_request
      if (session_id, key[1], direction) == (previous_session, previous_revision, previous_direction):
        # A pre-turn lamp may become a same-direction lane-change request once
        # lane alignment stabilizes. Keep the physical lamp continuously on and
        # transfer logical ownership without opening a second CAN session.
        self._last_nav_signal_request = key
        return
      self.validation.request_cancel(self._active_nav_signal_test_id, now_nanos)
      self._active_nav_signal_test_id = None
      self._last_nav_signal_request = None
      self._nav_signal_retry_after_ns = now_nanos + NAV_SIGNAL_RETRY_NS
      return
    if now_nanos < self._nav_signal_retry_after_ns:
      return
    # The existing ownership and retry interval bound submissions. Do not
    # exhaust a navigation session with a separate lifetime request quota.
    event_id = int(getattr(intent, "maneuverEventId", 0))
    if not session_id or event_id <= 0:
      return
    session_tag = hashlib.sha256(session_id.encode()).hexdigest()[:8]
    test_id = f"nav-{session_tag}-{key[1]}-{key[2]}-{direction}"
    accepted = self.validation.submit_request(
      test_id, direction, now_nanos, session_timeout_ns=NAV_SIGNAL_SESSION_TIMEOUT_NS,
      hold_until_cancel=True,
    )
    if accepted:
      self._last_nav_signal_request = key
      self._active_nav_signal_test_id = test_id
      self._nav_signal_retry_after_ns = now_nanos + NAV_SIGNAL_RETRY_NS
    elif not self.validation.configured:
      # Capability is fixed when card/Panda initialize; retrying cannot make it
      # available until the next onroad cycle.
      self._last_nav_signal_request = key
    else:
      # BUSY/cancelling is temporary. Retry at a bounded rate while the typed
      # intent and all upstream gates remain valid.
      self._nav_signal_retry_after_ns = now_nanos + NAV_SIGNAL_RETRY_NS

  def service_params(self, params) -> None:
    self.speed_limit_assist_configured = params.get("SpeedLimitMode", return_default=True) == Mode.assist
    if self.validation is not None:
      self.validation.service_params(params)
    if self.ambient is not None:
      self.ambient.service_params(params)

  def update_state(self, state_sp, now_ns: int | None = None) -> None:
    if self.road_context_parser is None:
      return

    from opendbc.sunnypilot.car.tesla.carstate_ext import publish_tesla_road_context

    timestamp_ns = self.road_context_parser.ts_nanos["DAS_road"]["DAS_stopLineDist"]
    publish_tesla_road_context(
      state_sp,
      self.road_context_parser.vl["DAS_road"],
      timestamp_ns,
      time.monotonic_ns() if now_ns is None else now_ns,
    )

  def publish_state(self, state_sp, now_ns: int | None = None) -> None:
    if self.traffic_control_observer is None:
      return
    publish_tesla_traffic_control(
      state_sp,
      self.traffic_control_observer.snapshot(time.monotonic_ns() if now_ns is None else now_ns),
    )

  def update_context(self, now: float | None = None, *, radar_now_ns: int | None = None, radar_data=None) -> None:
    timestamp = time.monotonic() if now is None else now
    if self.radar_pose_calibrator is not None:
      motion_now_ns = int(timestamp * 1e9) if radar_now_ns is None else radar_now_ns
      self.radar_transmitter.update_yaw_rate(*radar_yaw_rate_context(self.sm, motion_now_ns, self.radar_pose_calibrator, self.radar_yaw_cache))
      if radar_data is not None and not self.radar_transmitter.yaw_rate_valid(motion_now_ns):
        radar_data.errors.radarUnavailableTemporary = True
    state = getattr(self.car_interface, "CS", None)
    update_longitudinal = getattr(state, "update_longitudinal_context", None)
    if not self.enabled or update_longitudinal is None:
      return

    update_longitudinal(*longitudinal_context(self.sm, timestamp))

    update_speed_limit = getattr(state, "update_speed_limit_target", None)
    if update_speed_limit is not None:
      state.tesla_speed_limit_assist_enabled = self.speed_limit_assist_configured
      update_speed_limit(*speed_limit_context(self.sm, timestamp, self.speed_limit_assist_configured))
