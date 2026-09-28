from __future__ import annotations

from dataclasses import dataclass, replace
from enum import IntEnum
import math
import secrets

from openpilot.sunnypilot.selfdrive.controls.lib.relative_lane_consistency import RelativeLaneConsistencyFilter


class LaneIntentDirection(IntEnum):
  none = 0
  left = 1
  right = 2


class ObservedLaneChangeState(IntEnum):
  off = 0
  pre = 1
  starting = 2
  finishing = 3


@dataclass(frozen=True)
class NavLanePlan:
  valid: bool
  session_id: str
  route_revision: int
  maneuver_event_id: int
  lane_count: int
  recommended_indices: tuple[int, ...]
  heuristic: bool = False
  edge_direction: LaneIntentDirection = LaneIntentDirection.none
  force_fork: bool = False
  allow_unknown_crossing: bool = False
  ignore_solid_boundary: bool = False
  # Source health is distinct from mapping AMap's full-road lane indices.
  navigation_valid: bool = False


@dataclass(frozen=True)
class LaneTopologyInput:
  valid_for_control: bool
  visible_lane_count: int
  ego_lane_index: int
  left_neighbor_exists: bool | None
  right_neighbor_exists: bool | None
  left_crossing_allowed: bool
  right_crossing_allowed: bool


@dataclass(frozen=True)
class LaneVehicleInput:
  lateral_active: bool
  speed_mps: float
  left_blindspot: bool = False
  right_blindspot: bool = False
  left_blinker: bool = False
  right_blinker: bool = False
  brake_pressed: bool = False
  gas_pressed: bool = False
  steering_pressed: bool = False
  lane_change_state: ObservedLaneChangeState = ObservedLaneChangeState.off
  lane_change_direction: LaneIntentDirection = LaneIntentDirection.none
  model_mono_time_ns: int = 0


@dataclass(frozen=True)
class NavTurnPlan:
  valid: bool
  session_id: str
  route_revision: int
  maneuver_event_id: int
  maneuver: str
  distance_m: float
  source_interrupted: bool = False


@dataclass(frozen=True)
class NavLaneIntent:
  signal_requested: bool = False
  lane_change_ready: bool = False
  direction: LaneIntentDirection = LaneIntentDirection.none
  request_id: int = 0
  target_lane_index: int = -1
  reason: str = "idle"


class NavTurnSignalCoordinator:
  """Requests a physical lamp for a route turn without authorizing a lane change."""

  SIGNAL_TIMEOUT_NS = 60_000_000_000
  PLAN_GAP_GRACE_NS = 1_500_000_000
  SOURCE_GAP_GRACE_NS = 4_000_000_000  # 2 s SDK update + 1 s source recovery + delivery margin.
  LOOKAHEAD_TIME_S = 8.0
  MIN_LOOKAHEAD_M = 40.0
  MAX_LOOKAHEAD_M = 250.0
  LOOKAHEAD_MARGIN_M = 20.0
  TURN_CLEAR_STABLE_NS = 500_000_000
  MANEUVER_DIRECTIONS = {
    "slightLeft": LaneIntentDirection.left,
    "turnLeft": LaneIntentDirection.left,
    "sharpLeft": LaneIntentDirection.left,
    "uTurnLeft": LaneIntentDirection.left,
    "exitLeft": LaneIntentDirection.left,
    "rampLeft": LaneIntentDirection.left,
    "mergeLeft": LaneIntentDirection.left,
    "slightRight": LaneIntentDirection.right,
    "turnRight": LaneIntentDirection.right,
    "sharpRight": LaneIntentDirection.right,
    "uTurnRight": LaneIntentDirection.right,
    "exitRight": LaneIntentDirection.right,
    "rampRight": LaneIntentDirection.right,
    "mergeRight": LaneIntentDirection.right,
  }

  def __init__(self) -> None:
    self._active_key: tuple[str, int, int] | None = None
    self._active_direction = LaneIntentDirection.none
    self._active_since_ns = 0
    self._plan_gap_since_ns: int | None = None
    self._completion_hold = False
    self._turn_clear_since_ns = 0

  def _reset(self, reason: str = "idle") -> NavLaneIntent:
    self._active_key = None
    self._active_direction = LaneIntentDirection.none
    self._active_since_ns = 0
    self._plan_gap_since_ns = None
    self._completion_hold = False
    self._turn_clear_since_ns = 0
    return NavLaneIntent(reason=reason)

  def update(self, plan: NavTurnPlan, *, speed_mps: float, now_ns: int,
             turn_geometry_active: bool = False, lookahead_time_s: float | None = None) -> NavLaneIntent:
    direction = self.MANEUVER_DIRECTIONS.get(plan.maneuver, LaneIntentDirection.none)
    event_key = (plan.session_id, plan.route_revision, plan.maneuver_event_id)

    if self._active_key is not None:
      if now_ns - self._active_since_ns > self.SIGNAL_TIMEOUT_NS:
        return self._reset("turnSignalTimeout")
      same_route_direction = ((plan.session_id, plan.route_revision) == self._active_key[:2]
                              and direction == self._active_direction)
      source_gap = bool(plan.source_interrupted and plan.maneuver_event_id == 0 and same_route_direction)
      if not plan.valid and plan.maneuver_event_id == 0 and not source_gap:
        return self._reset("turnUnavailable")
      if not plan.valid and (event_key == self._active_key or source_gap):
        if self._plan_gap_since_ns is None:
          self._plan_gap_since_ns = now_ns
        grace_ns = self.SOURCE_GAP_GRACE_NS if source_gap else self.PLAN_GAP_GRACE_NS
        if now_ns - self._plan_gap_since_ns <= grace_ns:
          return NavLaneIntent(signal_requested=True, direction=self._active_direction,
                               request_id=self._active_key[2], target_lane_index=-1, reason="turnApproachGrace")
        return self._reset("turnUnavailable")
      self._plan_gap_since_ns = None
      if event_key != self._active_key or direction != self._active_direction:
        route_continuous = (plan.session_id, plan.route_revision) == self._active_key[:2]
        if not route_continuous:
          return self._reset("turnChanged")
        if turn_geometry_active:
          self._completion_hold = True
          self._turn_clear_since_ns = 0
        elif self._completion_hold:
          if self._turn_clear_since_ns == 0:
            self._turn_clear_since_ns = now_ns
          elif now_ns - self._turn_clear_since_ns > self.TURN_CLEAR_STABLE_NS:
            return self._reset("turnChanged")
        else:
          return self._reset("turnChanged")
        return NavLaneIntent(
          signal_requested=True,
          direction=self._active_direction,
          request_id=plan.maneuver_event_id,
          target_lane_index=-1,
          reason="turnCompletion",
        )
      self._completion_hold = False
      self._turn_clear_since_ns = 0
      if plan.valid:
        self._plan_gap_since_ns = None
        return NavLaneIntent(
          signal_requested=True,
          direction=self._active_direction,
          request_id=plan.maneuver_event_id,
          target_lane_index=-1,
          reason="turnApproach",
        )

    if not plan.valid or direction == LaneIntentDirection.none or plan.maneuver_event_id == 0:
      return self._reset("turnUnavailable")

    lookahead_m = max(self.MIN_LOOKAHEAD_M, min(
      self.MAX_LOOKAHEAD_M, max(0.0, speed_mps) * (self.LOOKAHEAD_TIME_S if lookahead_time_s is None else lookahead_time_s) + self.LOOKAHEAD_MARGIN_M,
    ))
    if not 0.0 < plan.distance_m <= lookahead_m:
      return self._reset("turnOutsideWindow")

    self._active_key = event_key
    self._active_direction = direction
    self._active_since_ns = now_ns
    self._plan_gap_since_ns = None
    return NavLaneIntent(
      signal_requested=True,
      direction=direction,
      request_id=plan.maneuver_event_id,
      target_lane_index=-1,
      reason="turnApproach",
    )


class NavLaneIntentCoordinator:
  MISMATCH_STABLE_NS = 500_000_000
  LANE_INDEX_STABLE_NS = 500_000_000
  TOPOLOGY_TRANSITION_GRACE_NS = 1_000_000_000
  SIGNAL_WAIT_TIMEOUT_NS = 60_000_000_000
  LANE_CHANGE_TIMEOUT_NS = 10_000_000_000
  COOLDOWN_NS = 750_000_000
  MIN_SPEED_MPS = 20 * 0.44704

  def __init__(self, *, max_changes: int = 5, require_announcement: bool = False) -> None:
    self.require_announcement = require_announcement
    self.announcement_id = ""
    self._announcement_since_ns = 0
    self._announcement_confirmed = False
    self._phase = "idle"
    self._candidate = None
    self._candidate_fork = False
    self._candidate_since_ns = 0
    self._phase_since_ns = 0
    self._signal_since_ns = 0
    self._blocked_key = None
    self._terminal_event_key = None
    self._terminal_reason = ""
    self._terminal_fork = False
    self._request_id = 0
    self._expected_lane_index = -1
    self._completion_since_ns = 0
    self._normal_finish_seen = False
    self._starting_model_seen = False
    self._observation_since_ns: int | None = None
    self._observation_recovery_ns: int | None = None
    self._observation_lamp_released = False
    self._relative_consistency = RelativeLaneConsistencyFilter(max_changes=max_changes)

  def _idle(self, reason: str = "idle") -> NavLaneIntent:
    return NavLaneIntent(reason=reason)

  @property
  def fork_active(self) -> bool:
    return self._candidate_fork and self._phase in ("signaling", "ready", "observing", "changing")

  @property
  def fork_terminal(self) -> bool:
    return self._terminal_fork

  @staticmethod
  def _target(plan: NavLanePlan, ego_index: int) -> int | None:
    valid = tuple(index for index in plan.recommended_indices if 0 <= index < plan.lane_count)
    return min(valid, key=lambda index: (abs(index - ego_index), index)) if valid else None

  @staticmethod
  def _direction(ego_index: int, target_index: int) -> LaneIntentDirection:
    if target_index < ego_index:
      return LaneIntentDirection.left
    if target_index > ego_index:
      return LaneIntentDirection.right
    return LaneIntentDirection.none

  @staticmethod
  def _adjacent_target_index(ego_index: int, direction: LaneIntentDirection) -> int:
    return max(0, ego_index - 1) if direction == LaneIntentDirection.left else ego_index + 1

  @staticmethod
  def _event_key(plan: NavLanePlan) -> tuple[str, int, int]:
    # Visible lane indices recenter while crossing a boundary. They describe an
    # observation, not a new navigation event or a reason to cancel its lamp.
    return (plan.session_id, plan.route_revision, plan.maneuver_event_id)

  @staticmethod
  def _plan_reason(plan: NavLanePlan, reason: str) -> str:
    return f"heuristic{reason[0].upper()}{reason[1:]}" if plan.heuristic and reason else reason

  def _observe_model_cycle(self, vehicle: LaneVehicleInput, direction: LaneIntentDirection) -> None:
    if vehicle.lane_change_state == ObservedLaneChangeState.starting:
      self._starting_model_seen = True
    if (vehicle.lane_change_state == ObservedLaneChangeState.finishing
        and vehicle.lane_change_direction == direction and self._starting_model_seen):
      self._normal_finish_seen = True

  def _relative_model_completion(self, vehicle: LaneVehicleInput, direction: LaneIntentDirection,
                                 event_key, now_ns: int) -> NavLaneIntent | None:
    if vehicle.lane_change_state not in (ObservedLaneChangeState.pre, ObservedLaneChangeState.off):
      return None
    if (self._normal_finish_seen and vehicle.lane_change_state == ObservedLaneChangeState.pre
        and vehicle.lane_change_direction == direction):
      self._relative_consistency.note_lane_change_completed(now_ns)
      if self._candidate_fork:
        # jihui CP desire_helper: a completed fork_now consumes its one request.
        self._terminal_event_key = event_key
        self._terminal_reason = "forkCompleted"
        self._terminal_fork = True
        self._reset()
        return self._idle("forkCompleted")
      self._phase = "cooldown"
      self._phase_since_ns = now_ns
      return self._idle("laneChangeObserved")
    return self._abort(event_key, "laneChangeCompletionUnconfirmed")

  @staticmethod
  def _relative_direction(plan: NavLanePlan) -> LaneIntentDirection:
    if not plan.heuristic:
      return LaneIntentDirection.none
    if plan.edge_direction != LaneIntentDirection.none:
      return plan.edge_direction
    if plan.lane_count > 1 and plan.recommended_indices == (0,):
      return LaneIntentDirection.left
    if plan.lane_count > 1 and plan.recommended_indices == (plan.lane_count - 1,):
      return LaneIntentDirection.right
    return LaneIntentDirection.none

  def _reset(self) -> None:
    self.announcement_id = ""
    self._announcement_since_ns = 0
    self._announcement_confirmed = False
    self._phase = "idle"
    self._candidate = None
    self._candidate_fork = False
    self._candidate_since_ns = 0
    self._phase_since_ns = 0
    self._signal_since_ns = 0
    self._expected_lane_index = -1
    self._completion_since_ns = 0
    self._normal_finish_seen = False
    self._starting_model_seen = False
    self._observation_since_ns = None
    self._observation_recovery_ns = None
    self._observation_lamp_released = False

  def _observe_before_start(self, now_ns: int, *, recovering: bool) -> NavLaneIntent | None:
    """Keep one request through a bounded gap, never its permission to start."""
    assert self._candidate is not None
    self._phase = "observing"
    if self._observation_since_ns is None:
      self._observation_since_ns = now_ns
    if not recovering:
      self._observation_recovery_ns = None
    elif self._observation_recovery_ns is None:
      self._observation_recovery_ns = now_ns
    if recovering and now_ns - self._observation_recovery_ns >= self.MISMATCH_STABLE_NS:
      self._phase = "signaling"
      self._phase_since_ns = now_ns
      if self._observation_lamp_released:
        self._signal_since_ns = now_ns
      self._observation_since_ns = self._observation_recovery_ns = None
      self._observation_lamp_released = False
      return None
    if not self._observation_lamp_released and now_ns - self._observation_since_ns > self.TOPOLOGY_TRANSITION_GRACE_NS:
      self._observation_lamp_released = True
      self._observation_recovery_ns = now_ns if recovering else None
    _event, _ego, direction, target, _relative = self._candidate
    return NavLaneIntent(
      signal_requested=not self._observation_lamp_released, direction=direction,
      request_id=self._request_id, target_lane_index=target,
      reason="neighborObservationPaused" if self._observation_lamp_released else "neighborObservationHold",
    )

  def _abort(self, event_key, reason: str) -> NavLaneIntent:
    if self.announcement_id and self._phase in ("signaling", "ready", "observing"):
      reason = "laneChangeAnnouncementFailed"
    if self._phase == "changing" or reason in ("laneChangeCancelled", "laneChangeCompletionUnconfirmed", "laneChangeAnnouncementFailed"):
      self._terminal_event_key = event_key
      self._terminal_reason = ("laneChangeCompletionUnconfirmed" if self._phase == "changing"
                               and reason != "laneChangeCancelled" else reason)
      self._terminal_fork = self._candidate_fork
    self._blocked_key = event_key
    self._reset()
    return self._idle(reason)

  def hold_active_fork(self, plan: NavLanePlan, topology: LaneTopologyInput) -> NavLanePlan:
    """Share the selected fork purpose with arbitration until model completion."""
    if self.fork_active and self._candidate[0] == self._event_key(plan):
      # Keep the selected fork purpose through distance jitter/zero while the
      # same valid navigation action completes, even if lane mapping now fails.
      # Source loss and all existing driver/control vetoes still apply below.
      fork_lane_count = max(1, topology.visible_lane_count)
      return replace(plan, valid=plan.valid or plan.navigation_valid, force_fork=True,
                     heuristic=self._candidate[4], edge_direction=self._candidate[2],
                     lane_count=fork_lane_count,
                     recommended_indices=(0 if self._candidate[2] == LaneIntentDirection.left
                                          else fork_lane_count - 1,))
    return plan

  def update(self, plan: NavLanePlan, topology: LaneTopologyInput, vehicle: LaneVehicleInput,
             *, now_ns: int, allow_new_lane_change: bool = True, spoken_announcement_id: str = "") -> NavLaneIntent:
    plan = self.hold_active_fork(plan, topology)
    if self.fork_active and self._candidate[0] == self._event_key(plan) and plan.valid:
      # The distance window qualifies a new action, not an already selected fork.
      allow_new_lane_change = True
    if self._terminal_event_key is not None:
      session, _revision, event = self._terminal_event_key
      # A source gap (including event=0) is not a new manoeuvre. Revision
      # changes alone must not revive a cancelled or unconfirmed action.
      if (not (plan.valid or (self._terminal_fork and plan.navigation_valid))
          or not plan.session_id or plan.maneuver_event_id == 0
          or (plan.session_id == session and plan.maneuver_event_id == event)):
        return self._idle(self._terminal_reason)
      self._terminal_event_key = None
      self._terminal_reason = ""
      self._terminal_fork = False
    if (self._candidate is not None and self._phase in ("signaling", "ready", "observing", "changing")
        and vehicle.lane_change_state == ObservedLaneChangeState.finishing
        and vehicle.lane_change_direction == LaneIntentDirection.none):
      return self._abort(self._candidate[0], "laneChangeCancelled")
    if self._phase in ("signaling", "ready", "observing") and self._candidate is not None:
      _event_key, start_ego, direction, _target_index, _relative_edge = self._candidate
      if (plan.force_fork and self._event_key(plan) == _event_key
          and vehicle.lane_change_state in (ObservedLaneChangeState.off, ObservedLaneChangeState.pre)):
        self._candidate_fork = True
        self._candidate = (*self._candidate[:4], True)
      if self.announcement_id:
        neighbor = topology.left_neighbor_exists if direction == LaneIntentDirection.left else topology.right_neighbor_exists
        if vehicle.steering_pressed or (neighbor is not True and not plan.force_fork):
          return self._abort(_event_key, "laneChangeAnnouncementFailed")
      if vehicle.lane_change_state == ObservedLaneChangeState.starting and vehicle.lane_change_direction == direction:
        if self.require_announcement and not self._announcement_confirmed:
          return self._abort(_event_key, "laneChangeAnnouncementFailed")
        # Observe the SP transition before deciding whether a new action may
        # start: reaching the turn window must not cut an action already begun.
        self._phase = "changing"
        self._phase_since_ns = now_ns
        self._expected_lane_index = self._adjacent_target_index(start_ego, direction)
        self._completion_since_ns = 0
    base_healthy = bool(
      plan.valid and vehicle.lateral_active and not vehicle.brake_pressed
      and not ((self._phase == "changing" or plan.force_fork) and vehicle.steering_pressed)
      and math.isfinite(vehicle.speed_mps)
      and (self._phase == "changing" or vehicle.speed_mps >= self.MIN_SPEED_MPS
           or (plan.force_fork and vehicle.speed_mps > 0.0))
    )
    topology_healthy = bool(plan.force_fork or (
      topology.valid_for_control and plan.lane_count == topology.visible_lane_count
      and 0 <= topology.ego_lane_index < topology.visible_lane_count
    ))
    relative_direction = self._relative_direction(plan)
    if relative_direction != LaneIntentDirection.none and (not base_healthy or not topology_healthy):
      self._relative_consistency.update(
        (plan.session_id, plan.route_revision, plan.maneuver_event_id),
        direction="left" if relative_direction == LaneIntentDirection.left else "right",
        neighbor_exists=False, observation_valid=False, lane_change_active=False,
        steering_pressed=vehicle.steering_pressed, now_ns=now_ns,
      )
    if not base_healthy:
      if self._phase not in ("idle", "cooldown") and self._candidate is not None:
        return self._abort(self._candidate[0], "health")
      self._reset()
      return self._idle("health")

    if self._phase in ("signaling", "ready", "observing") and not allow_new_lane_change:
      if self.announcement_id:
        return self._abort(self._candidate[0], "laneChangeAnnouncementFailed")
      self._reset()
      return self._idle("turnApproachHandoff")

    if self._phase == "changing" and self._candidate is not None:
      if now_ns - self._phase_since_ns > self.LANE_CHANGE_TIMEOUT_NS:
        return self._abort(self._candidate[0], "laneChangeTimeout")

    if not topology_healthy:
      if self._phase in ("signaling", "ready", "observing") and self._candidate is not None:
        if self.announcement_id:
          return self._abort(self._candidate[0], "laneChangeAnnouncementFailed")
        if self._event_key(plan) != self._candidate[0]:
          return self._abort(self._candidate[0], "routeChanged")
        return self._observe_before_start(now_ns, recovering=False)
      if self._phase == "changing" and self._candidate is not None:
        self._completion_since_ns = 0
        event_key, _start_ego, direction, target_index, _relative_edge = self._candidate
        if self._event_key(plan) != event_key:
          return self._abort(event_key, "routeChanged")
        if (vehicle.lane_change_state in (ObservedLaneChangeState.pre, ObservedLaneChangeState.starting,
                                          ObservedLaneChangeState.finishing) and
            vehicle.lane_change_direction != direction):
          return self._abort(event_key, "directionMismatch")
        physical_signal_on = ((vehicle.left_blinker and not vehicle.right_blinker)
                              if direction == LaneIntentDirection.left else
                              (vehicle.right_blinker and not vehicle.left_blinker))
        if not physical_signal_on:
          return self._abort(event_key, "physicalSignalLost")
        self._observe_model_cycle(vehicle, direction)
        if _relative_edge:
          completed = self._relative_model_completion(vehicle, direction, event_key, now_ns)
          if completed is not None:
            return completed
        return NavLaneIntent(
          signal_requested=True,
          lane_change_ready=vehicle.lane_change_state == ObservedLaneChangeState.starting,
          direction=direction,
          request_id=self._request_id, target_lane_index=self._expected_lane_index,
          reason=self._plan_reason(plan, "topologyTransition"),
        )
      self._reset()
      return self._idle("health")

    relative_status = None
    if relative_direction != LaneIntentDirection.none:
      relative_neighbor = (topology.left_neighbor_exists if relative_direction == LaneIntentDirection.left
                           else topology.right_neighbor_exists)
      relative_status = self._relative_consistency.update(
        (plan.session_id, plan.route_revision, plan.maneuver_event_id),
        direction="left" if relative_direction == LaneIntentDirection.left else "right",
        neighbor_exists=relative_neighbor,
        observation_valid=topology_healthy,
        lane_change_active=vehicle.lane_change_state in (ObservedLaneChangeState.starting, ObservedLaneChangeState.finishing),
        steering_pressed=vehicle.steering_pressed,
        now_ns=now_ns,
      )

    if self._phase in ("signaling", "ready", "observing") and self._candidate is not None:
      event_key, _ego, direction, _target, _relative = self._candidate
      if self._event_key(plan) != event_key:
        return self._abort(event_key, "routeChanged")
      neighbor = topology.left_neighbor_exists if direction == LaneIntentDirection.left else topology.right_neighbor_exists
      if neighbor is False and not plan.force_fork:
        return self._abort(event_key, "noNeighbor")
      if (neighbor is None and not plan.force_fork) or self._phase == "observing":
        waiting = self._observe_before_start(
          now_ns, recovering=(neighbor is True or plan.force_fork) and not vehicle.steering_pressed,
        )
        if waiting is not None:
          return waiting

    if self._phase == "cooldown":
      if now_ns - self._phase_since_ns < self.COOLDOWN_NS:
        return self._idle("cooldown")
      relative_edge = bool(self._candidate is not None and self._candidate[4])
      if not relative_edge and topology.ego_lane_index != self._expected_lane_index:
        return self._abort(self._candidate[0] if self._candidate is not None else None, "laneChangeNotObserved")
      self._reset()
      return self._idle("laneChangeComplete")

    if self._phase == "changing" and self._candidate is not None:
      event_key, _start_ego, direction, target_index, relative_edge = self._candidate
      if self._event_key(plan) != event_key:
        return self._abort(event_key, "routeChanged")
      if (vehicle.lane_change_state in (ObservedLaneChangeState.pre, ObservedLaneChangeState.starting,
                                        ObservedLaneChangeState.finishing) and
          vehicle.lane_change_direction != direction):
        return self._abort(event_key, "directionMismatch")
      physical_signal_on = ((vehicle.left_blinker and not vehicle.right_blinker)
                            if direction == LaneIntentDirection.left else
                            (vehicle.right_blinker and not vehicle.left_blinker))
      if not physical_signal_on:
        return self._abort(event_key, "physicalSignalLost")
      self._observe_model_cycle(vehicle, direction)
      if relative_edge:
        completed = self._relative_model_completion(vehicle, direction, event_key, now_ns)
        if completed is not None:
          return completed
      model_cycle_complete = vehicle.lane_change_state in (
        ObservedLaneChangeState.off, ObservedLaneChangeState.pre,
      )
      if topology.ego_lane_index == self._expected_lane_index and model_cycle_complete:
        if self._completion_since_ns == 0:
          self._completion_since_ns = now_ns
        elif now_ns - self._completion_since_ns >= self.LANE_INDEX_STABLE_NS:
          self._phase = "cooldown"
          self._phase_since_ns = now_ns
          return self._idle("laneChangeObserved")
      else:
        self._completion_since_ns = 0
      return NavLaneIntent(
        signal_requested=True,
        lane_change_ready=vehicle.lane_change_state == ObservedLaneChangeState.starting,
        direction=direction,
        request_id=self._request_id,
        target_lane_index=self._expected_lane_index,
        reason=self._plan_reason(plan, self._phase),
      )

    # jihui CP resets fork_now to its own one-shot allowance. Approach changes
    # keep their limit; the existing forkCompleted terminal state consumes fork.
    if (not plan.force_fork and relative_status is not None
        and relative_status.completed_changes >= self._relative_consistency.max_changes):
      self._reset()
      return self._idle(self._plan_reason(plan, "changeLimit"))

    if not allow_new_lane_change:
      self._reset()
      return self._idle("turnApproachHandoff")

    target_index = self._target(plan, topology.ego_lane_index)
    if target_index is None:
      self._reset()
      return self._idle("noRecommendedLane")
    direction = relative_direction if relative_direction != LaneIntentDirection.none else self._direction(topology.ego_lane_index, target_index)
    relative_cycle_active = vehicle.lane_change_state in (ObservedLaneChangeState.starting, ObservedLaneChangeState.finishing)
    if (relative_status is not None and not relative_status.ready and not plan.force_fork
        and self._phase != "changing" and not relative_cycle_active):
      self._blocked_key = None
      self._reset()
      return self._idle(self._plan_reason(plan, relative_status.reason))
    if direction == LaneIntentDirection.none:
      self._blocked_key = None
      self._reset()
      return self._idle("alreadyInRecommendedLane")

    event_key = self._event_key(plan)
    if self._blocked_key == event_key:
      return self._idle("blockedEvent")

    neighbor_exists = topology.left_neighbor_exists if direction == LaneIntentDirection.left else topology.right_neighbor_exists
    if neighbor_exists is None and not plan.force_fork:
      self._reset()
      return self._idle("neighborUnknown")
    if not neighbor_exists and not plan.force_fork:
      return self._abort(event_key, "noNeighbor")

    candidate = (event_key, topology.ego_lane_index, direction, target_index, plan.heuristic)
    if self.fork_active and self._candidate[0] == event_key:
      # A relative fork keeps its identity when the visible lane window recenters.
      candidate = self._candidate
    if self._phase == "idle":
      if candidate != self._candidate:
        self._candidate = candidate
        self._candidate_since_ns = now_ns
        if not plan.force_fork:
          return self._idle(self._plan_reason(plan, "stabilizingLaneAlignment"))
      if not plan.force_fork and now_ns - self._candidate_since_ns < self.MISMATCH_STABLE_NS:
        return self._idle(self._plan_reason(plan, "stabilizingLaneAlignment"))
      # Navigation owns lamp timing. OEM permission, BSM and the optional
      # visual boundary veto qualify readiness below; they do not suppress the
      # physical intent lamp while the vehicle waits for a safe opening.
      self._phase = "signaling"
      self._candidate_fork = plan.force_fork
      self._phase_since_ns = now_ns
      self._signal_since_ns = now_ns
      self._request_id += 1

    if self._candidate != candidate and self._phase != "cooldown":
      self._reset()
      self._candidate = candidate
      self._candidate_since_ns = now_ns
      return self._idle("routeChanged")

    if self._phase in ("ready", "changing") and now_ns - self._phase_since_ns > self.LANE_CHANGE_TIMEOUT_NS:
      return self._abort(event_key, "laneChangeTimeout")

    if vehicle.lane_change_state in (ObservedLaneChangeState.starting, ObservedLaneChangeState.finishing):
      if self.require_announcement and not self._announcement_confirmed:
        return self._abort(event_key, "laneChangeAnnouncementFailed")
      if vehicle.lane_change_direction != direction:
        return self._abort(event_key, "directionMismatch")
      if vehicle.lane_change_state == ObservedLaneChangeState.starting:
        self._phase = "changing"
        self._phase_since_ns = now_ns
        self._expected_lane_index = self._adjacent_target_index(topology.ego_lane_index, direction)
        self._completion_since_ns = 0
      else:
        # A first observed finishing frame cannot establish a successful start.
        # Relative plans require the complete normal lifecycle above.
        self._phase = "changing"

    crossing_allowed = topology.left_crossing_allowed if direction == LaneIntentDirection.left else topology.right_crossing_allowed
    blindspot = vehicle.left_blindspot if direction == LaneIntentDirection.left else vehicle.right_blindspot
    physical_signal_on = ((vehicle.left_blinker and not vehicle.right_blinker)
                          if direction == LaneIntentDirection.left else
                          (vehicle.right_blinker and not vehicle.left_blinker))
    if self.announcement_id and self._phase in ("signaling", "ready"):
      if (not crossing_allowed or blindspot or not physical_signal_on or vehicle.steering_pressed
          or (not self._announcement_confirmed and now_ns - self._announcement_since_ns > 8_000_000_000)):
        return self._abort(event_key, "laneChangeAnnouncementFailed")
      if spoken_announcement_id == self.announcement_id:
        self._announcement_confirmed = True
    if self._phase == "signaling":
      if now_ns - self._signal_since_ns > self.SIGNAL_WAIT_TIMEOUT_NS:
        return self._abort(event_key, "crossingWaitTimeout")
      if crossing_allowed and not blindspot and physical_signal_on:
        if self.require_announcement and not self._announcement_confirmed:
          if not self.announcement_id:
            self.announcement_id = secrets.token_hex(16)
            self._announcement_since_ns = now_ns
        else:
          self._phase = "ready"
          self._phase_since_ns = now_ns
    elif self._phase == "ready" and (not crossing_allowed or blindspot or not physical_signal_on):
      self._phase = "signaling"

    ready = self._phase in ("ready", "changing")
    return NavLaneIntent(
      signal_requested=True,
      lane_change_ready=ready,
      direction=direction,
      request_id=self._request_id,
      target_lane_index=self._adjacent_target_index(topology.ego_lane_index, direction),
      reason=self._plan_reason(plan, self._phase),
    )
