from __future__ import annotations

import math
import time

from openpilot.cereal import custom
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.car.tesla.control_runtime import TeslaControlState, TeslaLongitudinalOwner
from openpilot.sunnypilot.navassist.settings import NavAssistSettings, SettingsCache


NavManeuver = custom.NavAssistStateSP.Maneuver

COMFORT_BRAKE = 1.2
ACTUATION_DELAY_S = 1.0
ADMISSION_MARGIN_M = 5.0
ACTIVATION_MARGIN_M = 25.0
MAX_MANEUVER_DISTANCE_M = 2_000.0
MAX_NAV_SPEED_MPS = V_CRUISE_MAX / 3.6
MIN_TARGET_SPEED_MPS = 2.0
RELEASE_ACCEL_MPS2 = 1.0
SOURCE_RECOVERY_NS = 4_000_000_000

TARGET_SPEEDS = {
  NavManeuver.slightLeft: 8.0,
  NavManeuver.slightRight: 8.0,
  NavManeuver.turnLeft: 5.0,
  NavManeuver.turnRight: 5.0,
  NavManeuver.sharpLeft: 3.5,
  NavManeuver.sharpRight: 3.5,
  NavManeuver.uTurnLeft: 2.5,
  NavManeuver.uTurnRight: 2.5,
  NavManeuver.roundabout: 6.0,
}


class NavigationSpeedController:
  """Closed-course navigation speed ceiling; never requests acceleration or a stop."""

  def __init__(self, *, enabled: bool | None = None, require_sp_longitudinal_owner: bool = False, settings_provider=None):
    self.enabled = True if enabled is None else enabled
    self.require_sp_longitudinal_owner = require_sp_longitudinal_owner
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.0
    self.is_active = False
    self.is_releasing = False
    self.event_admitted = False
    self.event_rejected = False
    self.event_key: tuple[str, int, int] | None = None
    self.target_speed = 0.0
    self.required_distance = 0.0
    self.event_activated = False
    self.source_interrupted_ns = 0
    self._settings_provider = settings_provider if settings_provider is not None else SettingsCache().read

  @staticmethod
  def _healthy(sm) -> bool:
    service = "navAssistStateSP"
    return bool(sm.seen[service] and sm.alive[service] and sm.valid[service] and sm[service].valid and not sm[service].stale)

  def _sp_owns_longitudinal(self, sm) -> bool:
    if not self.require_sp_longitudinal_owner:
      return True
    service = "carStateSP"
    if not (sm.seen[service] and sm.alive[service] and sm.valid[service]):
      return False
    owner = TeslaControlState(TeslaFlagsSP(int(sm[service].flags))).longitudinal_owner
    return owner in (TeslaLongitudinalOwner.sp, TeslaLongitudinalOwner.ap_hybrid_sp)

  def _pause_event(self) -> None:
    # Only a maneuver whose braking already started is latched out. The phone
    # may already show the next turn while the driver finishes the current one:
    # never bind that future event to an unrelated pedal/ownership interruption.
    if self.event_activated:
      self.event_rejected = True
    self.event_admitted = False
    self.source_interrupted_ns = 0

  @staticmethod
  def _required_distance(v_ego: float, target_speed: float) -> float:
    braking = max(0.0, v_ego ** 2 - target_speed ** 2) / (2.0 * COMFORT_BRAKE)
    return braking + v_ego * ACTUATION_DELAY_S

  @staticmethod
  def _target_for(nav, settings: NavAssistSettings | None = None) -> float | None:
    # A ramp/exit name does not specify its curvature or a safe target speed.
    # Reuse existing curve/longitudinal planners unless an explicit advisory exists,
    # as for highway slight guidance. Approach roads need not be road class 0/6.
    if (nav.maneuver.raw in (NavManeuver.exitLeft, NavManeuver.exitRight, NavManeuver.rampLeft, NavManeuver.rampRight)
        or (getattr(nav, 'roadClass', -1) in (0, 6)
            and nav.maneuver.raw in (NavManeuver.slightLeft, NavManeuver.slightRight))):
      if not nav.advisorySpeedValid:
        return None
      advisory = float(nav.advisorySpeedMps)
      return max(MIN_TARGET_SPEED_MPS, min(MAX_NAV_SPEED_MPS, advisory)) if math.isfinite(advisory) and advisory > 0 else None
    # Cap'n Proto enum readers compare with integers but hash differently.
    # Use the numeric wire value when looking up the integer-keyed speed table.
    default = TARGET_SPEEDS.get(nav.maneuver.raw)
    if settings is not None and nav.maneuver.raw in (NavManeuver.turnLeft, NavManeuver.turnRight):
      default = settings.turn_speed_kph / 3.6
    if default is None:
      return None
    if nav.advisorySpeedValid:
      return max(MIN_TARGET_SPEED_MPS, min(default, float(nav.advisorySpeedMps)))
    return default

  def _release(self, v_cruise: float, a_ego: float) -> None:
    self.is_active = False
    self.output_a_target = a_ego
    if self.output_v_target == V_CRUISE_UNSET:
      self.is_releasing = False
      return
    self.output_v_target = min(v_cruise, self.output_v_target + RELEASE_ACCEL_MPS2 * DT_MDL)
    self.is_releasing = self.output_v_target < v_cruise - 1e-3
    if not self.is_releasing:
      self.output_v_target = V_CRUISE_UNSET

  def update(self, sm, *, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise: float,
             planner_verified: bool = True) -> None:
    settings = self._settings_provider()
    if not self.enabled or not settings.enabled or not settings.turn_slowdown_enabled:
      self.output_v_target = V_CRUISE_UNSET
      self.output_a_target = a_ego
      self.is_active = self.is_releasing = False
      return

    driver_override = bool(long_override or sm["carState"].gasPressed or sm["carState"].brakePressed)
    if driver_override:
      self._pause_event()
      self._release(v_cruise, a_ego)
      return

    temporarily_unavailable = (not long_enabled or not planner_verified
                               or not math.isfinite(v_ego) or not 0.0 <= v_ego <= MAX_NAV_SPEED_MPS
                               or not self._sp_owns_longitudinal(sm))
    if temporarily_unavailable:
      # Preparing a route is not executing its braking maneuver. Recheck entry
      # distance after recovery; keep the cancellation latch once braking began.
      self._pause_event()
      self._release(v_cruise, a_ego)
      return

    if not self._healthy(sm):
      if self.event_activated and not self.event_rejected:
        if not self.source_interrupted_ns:
          self.source_interrupted_ns = time.monotonic_ns()
        elif time.monotonic_ns() - self.source_interrupted_ns > SOURCE_RECOVERY_NS:
          self._pause_event()
      else:
        self._pause_event()
      self._release(v_cruise, a_ego)
      return

    nav = sm["navAssistStateSP"]
    target_speed = self._target_for(nav, settings)
    distance = float(nav.maneuverDistanceM)
    event_key = (str(nav.sessionId), int(nav.routeRevision), int(nav.maneuverEventId))
    if target_speed is None or event_key[2] == 0 or not math.isfinite(distance) or not 0.0 <= distance <= MAX_MANEUVER_DISTANCE_M:
      self._pause_event()
      self._release(v_cruise, a_ego)
      return

    required_distance = self._required_distance(v_ego, target_speed)
    if self.source_interrupted_ns:
      recovering_same_event = event_key == self.event_key
      expired = time.monotonic_ns() - self.source_interrupted_ns > SOURCE_RECOVERY_NS
      self.source_interrupted_ns = 0
      if recovering_same_event and (expired or (v_ego > target_speed and distance < required_distance + ADMISSION_MARGIN_M)):
        self._pause_event()
        self._release(v_cruise, a_ego)
        return
    if event_key != self.event_key or (not self.event_admitted and not self.event_rejected):
      self.event_key = event_key
      self.target_speed = target_speed
      self.required_distance = required_distance
      self.event_admitted = distance >= required_distance + ADMISSION_MARGIN_M
      self.event_rejected = not self.event_admitted
      self.event_activated = False

    if self.event_rejected or not self.event_admitted:
      self._release(v_cruise, a_ego)
      return

    if not self.event_activated:
      self.required_distance = max(self.required_distance, required_distance)
      if distance < required_distance:
        # Reject an approach that became infeasible before braking began.
        self.event_rejected = True
        self._release(v_cruise, a_ego)
        return
      if distance > self.required_distance + ACTIVATION_MARGIN_M:
        self._release(v_cruise, a_ego)
        return
      self.event_activated = True

    # Once braking begins, consuming the remaining distance (including zero)
    # must not revoke the same event's ceiling. Health/authority/driver gates
    # above still cancel it. The common target selector compares this ceiling
    # with SCC-V every cycle, so a lower vision target wins without permanently
    # dropping navigation when vision later releases.
    self.is_active = True
    self.is_releasing = False
    self.output_v_target = min(v_cruise, self.target_speed)
    # The selected planner/MPC produces the deceleration trajectory. Preserve
    # its current acceleration seed instead of injecting an actuator command.
    self.output_a_target = a_ego
