from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController
from openpilot.sunnypilot.selfdrive.controls.lib.turn_entry import TurnManeuver

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5
LANE_CHANGE_CANCEL_TIME_MAX = 2.0
NAV_LANE_CHANGE_FINISH_TIME = 1.0  # CP's finishing/recovery phase, without a new lateral controller.
NAV_TARGET_LANE_STABLE_TIME = 0.5

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    self.turn_decision_reason = 'notObserved'
    self._active_nav_signal_direction: str | None = None
    self._active_nav_signal_turn_only = False
    self._active_nav_signal_feedback = False
    self._nav_signal_tail_direction: str | None = None
    self._nav_signal_tail_turn_only = False
    self._cancelled_signal = False
    self._signal_direction = None
    self._signal_is_lane_change = False
    self.turn_maneuver = TurnManeuver()
    self._nav_change_key = None
    self._completed_nav_change_key = None
    self._nav_target_observed_timer = 0.0
    self._nav_target_confirmed = False

  @staticmethod
  def get_lane_change_direction(left_blinker, right_blinker):
    return LaneChangeDirection.left if left_blinker and not right_blinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False,
             nav_lane_intent=None, left_line_blocked=False, right_line_blocked=False,
             left_crossing_allowed=False, right_crossing_allowed=False,
             left_start_allowed=True, right_start_allowed=True,
             left_safety_blocked=False, right_safety_blocked=False,
             left_turn_allowed=None, right_turn_allowed=None,
             left_neighbor_exists=None, right_neighbor_exists=None,
             left_turn_keep_allowed=None, right_turn_keep_allowed=None, turn_completed=False, turn_soft_reentry=False,
             observed_lane_index=None, observed_lane_count=0):
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    # Explicit permissions opt the Tesla adapter into the separated entry.
    # Other callers retain their existing turn contract.
    separate_turn_entry = left_turn_allowed is not None or right_turn_allowed is not None
    if not separate_turn_entry:
      left_turn_allowed, right_turn_allowed = left_start_allowed, right_start_allowed
    # targetLaneIndex < 0 is a physical pre-turn lamp request only. Keep it out
    # of the lane-change state machine so an upcoming turn cannot become a
    # lateral lane-change desire.
    nav_requested = bool(
      nav_lane_intent is not None and nav_lane_intent.valid and nav_lane_intent.signalRequested
    )
    nav_direction = str(nav_lane_intent.direction) if nav_requested else "none"
    nav_signal = bool(nav_requested and nav_lane_intent.targetLaneIndex >= 0)
    nav_fork = bool(nav_signal and getattr(nav_lane_intent, "forkNow", False))
    nav_turn_only = bool(nav_requested and nav_lane_intent.targetLaneIndex < 0 and nav_direction in ("left", "right"))
    nav_change_key = ((getattr(nav_lane_intent, "sessionId", ""),
                       getattr(nav_lane_intent, "maneuverEventId", 0),
                       getattr(nav_lane_intent, "requestId", 0), nav_direction) if nav_signal else None)
    if (self._active_nav_signal_direction is not None
        and (not nav_requested or nav_direction != self._active_nav_signal_direction)):
      if self._active_nav_signal_feedback:
        self._nav_signal_tail_direction = self._active_nav_signal_direction
        self._nav_signal_tail_turn_only = self._active_nav_signal_turn_only
      self._active_nav_signal_direction = None
      self._active_nav_signal_turn_only = False
      self._active_nav_signal_feedback = False
    if nav_requested and nav_direction in ("left", "right"):
      if nav_direction != self._active_nav_signal_direction:
        self._active_nav_signal_feedback = False
      self._active_nav_signal_direction = nav_direction
      self._active_nav_signal_turn_only = nav_turn_only
      # A new request owns a same-direction lamp even if the previous request's
      # cancellation has not turned it off. Its current purpose replaces the
      # old tail; turn-only requests still suppress the ALC entrance below.
      if self._nav_signal_tail_direction == nav_direction:
        self._nav_signal_tail_direction = None
        self._nav_signal_tail_turn_only = False
      physical_nav_signal_on = ((nav_direction == "left" and carstate.leftBlinker and not carstate.rightBlinker)
                                or (nav_direction == "right" and carstate.rightBlinker and not carstate.leftBlinker))
      self._active_nav_signal_feedback |= physical_nav_signal_on
    if self._nav_signal_tail_direction is not None:
      tail_signal_on = ((self._nav_signal_tail_direction == "left" and carstate.leftBlinker and not carstate.rightBlinker)
                        or (self._nav_signal_tail_direction == "right" and carstate.rightBlinker and not carstate.leftBlinker))
      if not tail_signal_on:
        self._nav_signal_tail_direction = None
        self._nav_signal_tail_turn_only = False

    nav_left = nav_signal and str(nav_lane_intent.direction) == "left"
    nav_right = nav_signal and str(nav_lane_intent.direction) == "right"
    physical_conflict = ((carstate.leftBlinker and nav_right) or (carstate.rightBlinker and nav_left))
    if physical_conflict:
      nav_signal = nav_left = nav_right = False
    suppress_turn_only_left = bool(
      ((self._active_nav_signal_turn_only and self._active_nav_signal_direction == "left")
       or self._nav_signal_tail_direction == "left") and carstate.leftBlinker and not carstate.rightBlinker
    )
    suppress_turn_only_right = bool(
      ((self._active_nav_signal_turn_only and self._active_nav_signal_direction == "right")
       or self._nav_signal_tail_direction == "right") and carstate.rightBlinker and not carstate.leftBlinker
    )
    left_blinker = bool((carstate.leftBlinker and not suppress_turn_only_left) or nav_left)
    right_blinker = bool((carstate.rightBlinker and not suppress_turn_only_right) or nav_right)
    one_blinker = left_blinker != right_blinker
    # jihui CP fork now clears its ordinary lane-change speed floor. Keep this
    # scoped to a navigation fork; physical lamps alone do not grant it.
    below_lane_change_speed = v_ego <= 0.0 or (v_ego < LANE_CHANGE_SPEED_MIN and not nav_fork)
    if not carstate.leftBlinker and not carstate.rightBlinker and not nav_requested:
      self._cancelled_signal = False
    left_hard_blocked = carstate.leftBlindspot or left_edge_detected or left_line_blocked or left_safety_blocked
    right_hard_blocked = carstate.rightBlindspot or right_edge_detected or right_line_blocked or right_safety_blocked
    target_lane_index = int(getattr(nav_lane_intent, "targetLaneIndex", -1)) if nav_lane_intent is not None else -1
    nav_target_observed = bool(
      nav_signal and self._nav_change_key is not None and observed_lane_index is not None
      and 0 <= target_lane_index < observed_lane_count and observed_lane_index == target_lane_index
    )

    # A navigation lane-change lamp must not look like an intersection turn to
    # LaneTurnDesire. Turn-only navigation lamps and driver lamps retain the
    # existing SP behavior.
    nav_lane_signal_left = bool(
      self._active_nav_signal_direction == "left" and not self._active_nav_signal_turn_only
    )
    nav_lane_signal_right = bool(
      self._active_nav_signal_direction == "right" and not self._active_nav_signal_turn_only
    )
    nav_lane_tail_left = bool(
      self._nav_signal_tail_direction == "left" and not self._nav_signal_tail_turn_only
    )
    nav_lane_tail_right = bool(
      self._nav_signal_tail_direction == "right" and not self._nav_signal_tail_turn_only
    )
    lane_turn_left_blinker = bool(carstate.leftBlinker and not (nav_lane_signal_left or nav_lane_tail_left))
    lane_turn_right_blinker = bool(carstate.rightBlinker and not (nav_lane_signal_right or nav_lane_tail_right))

    # Classify the signal before considering speed eligibility. An observed
    # neighbor blocks a turn below, but does not establish the driver's intent.
    # A navigation lane request or an executed change consumes this purpose.
    # CP permits a pending manual change to be reclassified by turn evidence.
    signal_direction = ('left' if carstate.leftBlinker and not carstate.rightBlinker else
                        'right' if carstate.rightBlinker and not carstate.leftBlinker else None)
    if signal_direction != self._signal_direction:
      self._signal_direction = signal_direction
      self._signal_is_lane_change = False
    neighbor = left_neighbor_exists if signal_direction == 'left' else right_neighbor_exists
    lane_neighbor = left_neighbor_exists if left_blinker else right_neighbor_exists
    lane_start_allowed = left_start_allowed if left_blinker else right_start_allowed
    if separate_turn_entry and not nav_turn_only and signal_direction is not None and (
        nav_signal or self.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)):
      self._signal_is_lane_change = True
    if nav_turn_only and nav_direction == signal_direction:
      self._signal_is_lane_change = False  # Explicit new turn purpose, not a speed change.

    reclassify_turn = (separate_turn_entry and not self._signal_is_lane_change and not self._cancelled_signal
                      and signal_direction is not None and neighbor is False
                      and self.lane_change_state == LaneChangeState.preLaneChange
                      and self.lane_turn_controller.enabled and 0 <= v_ego < self.lane_turn_controller.lane_turn_value
                      and (left_turn_allowed if signal_direction == 'left' else right_turn_allowed))
    if reclassify_turn:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self._nav_target_observed_timer = 0.0
      self._nav_target_confirmed = False

    if (not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or
        self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF):
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self._nav_target_observed_timer = 0.0
      self._nav_target_confirmed = False
    else:
      if self._cancelled_signal and self.lane_change_state != LaneChangeState.laneChangeFinishing:
        self.lane_change_state = LaneChangeState.off
        self.lane_change_direction = LaneChangeDirection.none
        self.lane_change_timer = 0.0
        self._nav_target_observed_timer = 0.0
        self._nav_target_confirmed = False
      elif (self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker
            and not below_lane_change_speed
            and not (separate_turn_entry and lane_neighbor is False and not lane_start_allowed)):
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        self._nav_target_observed_timer = 0.0
        self._nav_target_confirmed = False
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(left_blinker, right_blinker)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(left_blinker, right_blinker)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        left_blocked = left_hard_blocked
        right_blocked = right_hard_blocked
        blindspot_detected = ((left_blocked and self.lane_change_direction == LaneChangeDirection.left) or
                              (right_blocked and self.lane_change_direction == LaneChangeDirection.right))

        physical_nav_signal_on = bool(
          (nav_left and carstate.leftBlinker and not carstate.rightBlinker)
          or (nav_right and carstate.rightBlinker and not carstate.leftBlinker)
        )
        nav_crossing_allowed = bool(
          (nav_left and left_crossing_allowed) or (nav_right and right_crossing_allowed)
        )
        nav_not_ready = nav_signal and (nav_change_key == self._completed_nav_change_key or not (
          physical_nav_signal_on and nav_crossing_allowed and bool(getattr(nav_lane_intent, "spLaneChangeReady", False))
        ) or (nav_fork and (carstate.steeringPressed or carstate.brakePressed)))
        permission_blocked = not (left_start_allowed if self.lane_change_direction == LaneChangeDirection.left else right_start_allowed)
        lane_change_blocked = blindspot_detected or nav_not_ready or permission_blocked

        # Eligibility still gates every start below. Only an actual blindspot
        # or road-edge obstruction resets the configured recovery timer; vision
        # and OEM permission already have their own confirmation requirements.
        recovery_blocked = (
          (carstate.leftBlindspot or left_edge_detected)
          if self.lane_change_direction == LaneChangeDirection.left
          else (carstate.rightBlindspot or right_edge_detected)
        )
        self.alc.update_lane_change(recovery_blocked, carstate.brakePressed)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        else:
          if (torque_applied or self.alc.auto_lane_change_allowed) and not lane_change_blocked:
            self.lane_change_state = LaneChangeState.laneChangeStarting
            self.lane_change_timer = 0.0
            self._nav_change_key = nav_change_key if nav_signal else None
            self._nav_target_observed_timer = 0.0
            self._nav_target_confirmed = False

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL
        selected_left = self.lane_change_direction == LaneChangeDirection.left
        safety_blocked = ((carstate.leftBlindspot or left_safety_blocked) if selected_left
                          else (carstate.rightBlindspot or right_safety_blocked))
        visual_blocked = ((left_edge_detected or left_line_blocked) if selected_left
                          else (right_edge_detected or right_line_blocked))
        if nav_target_observed:
          self._nav_target_observed_timer += DT_MDL
        else:
          self._nav_target_observed_timer = 0.0
        nav_interrupted = self._nav_change_key is not None and (
          nav_change_key != self._nav_change_key or not one_blinker or
          not (carstate.leftBlinker if self.lane_change_direction == LaneChangeDirection.left else carstate.rightBlinker) or
          carstate.steeringPressed or carstate.brakePressed)
        if safety_blocked or below_lane_change_speed or nav_interrupted or (visual_blocked and not nav_target_observed):
          # Withdraw the manoeuvre desire, not steering control. Do not turn a
          # held lane-change lamp into a fresh low-speed turn or another change.
          self._cancelled_signal = True
          self.lane_change_state = LaneChangeState.laneChangeFinishing
          # Finishing without a direction denotes cancellation, not success.
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
          self._nav_target_observed_timer = 0.0
          self._nav_target_confirmed = False
        elif self._nav_change_key is not None and self._nav_target_observed_timer >= NAV_TARGET_LANE_STABLE_TIME:
          # A slower model can keep its lane-change probability high after the
          # control-valid topology has already settled in the requested lane.
          # Finish the existing model desire normally instead of treating the
          # destination-side line or road edge as a new mid-change obstacle.
          self._nav_target_confirmed = True
          self.lane_change_state = LaneChangeState.laneChangeFinishing
          self.lane_change_timer = 0.0
        elif lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if self._nav_change_key is not None:
            # Same-direction finishing denotes normal model completion. A
            # direction of none remains the existing cancellation contract.
            self.lane_change_state = LaneChangeState.laneChangeFinishing
          elif one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(left_blinker, right_blinker)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        self.lane_change_timer += DT_MDL
        if self.lane_change_direction != LaneChangeDirection.none and self._nav_change_key is not None:
          selected_left = self.lane_change_direction == LaneChangeDirection.left
          safety_blocked = ((carstate.leftBlindspot or left_safety_blocked) if selected_left
                            else (carstate.rightBlindspot or right_safety_blocked))
          visual_blocked = ((left_edge_detected or left_line_blocked) if selected_left
                            else (right_edge_detected or right_line_blocked))
          physical_on = ((carstate.leftBlinker and not carstate.rightBlinker)
                         if self.lane_change_direction == LaneChangeDirection.left else
                         (carstate.rightBlinker and not carstate.leftBlinker))
          if (safety_blocked or (visual_blocked and not self._nav_target_confirmed)
              or below_lane_change_speed or not physical_on or nav_change_key != self._nav_change_key
              or carstate.steeringPressed or carstate.brakePressed):
            self._cancelled_signal = True
            self.lane_change_direction = LaneChangeDirection.none
            self.lane_change_timer = 0.0
            self._nav_target_observed_timer = 0.0
            self._nav_target_confirmed = False
          elif self.lane_change_timer >= NAV_LANE_CHANGE_FINISH_TIME:
            self._completed_nav_change_key = self._nav_change_key
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_timer = 0.0
            self._nav_target_observed_timer = 0.0
            self._nav_target_confirmed = False
        elif ((lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME)
            or self.lane_change_timer >= LANE_CHANGE_CANCEL_TIME_MAX):
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0

    # Boundary memory cannot be erased just by calling the move a turn.
    # TurnEntryGate also keeps its own memory, independent of navigation policy.
    left_turn_blocked = (carstate.leftBlindspot or left_safety_blocked or left_line_blocked) if separate_turn_entry else left_hard_blocked
    right_turn_blocked = (carstate.rightBlindspot or right_safety_blocked or right_line_blocked) if separate_turn_entry else right_hard_blocked
    # CP's turn branch lacks its lane-change torque cancellation. Keep driver
    # override priority when reusing that intent on SP.
    opposing_torque = carstate.steeringPressed and (
      (signal_direction == 'left' and carstate.steeringTorque < 0)
      or (signal_direction == 'right' and carstate.steeringTorque > 0))
    if separate_turn_entry and opposing_torque:
      self._cancelled_signal = True
    low_speed_turn_blocked = (
      (lane_turn_left_blinker and not lane_turn_right_blinker and left_turn_blocked)
      or (lane_turn_right_blinker and not lane_turn_left_blinker and right_turn_blocked)
    )
    if (not separate_turn_entry and lateral_active and self.lane_turn_controller.enabled
        and v_ego < self.lane_turn_controller.lane_turn_value and low_speed_turn_blocked):
      self._cancelled_signal = True
    turn_allowed = (lateral_active and not self._cancelled_signal and not self._signal_is_lane_change
                    and self.lane_change_state == LaneChangeState.off
                    and not (separate_turn_entry and neighbor is True))
    if separate_turn_entry:
      left_keep = left_turn_allowed if left_turn_keep_allowed is None else left_turn_keep_allowed
      right_keep = right_turn_allowed if right_turn_keep_allowed is None else right_turn_keep_allowed
      selected_left = signal_direction == 'left'
      requested = lane_turn_left_blinker if selected_left else lane_turn_right_blinker
      active_turn = self.turn_maneuver.update(
        signal_direction,
        eligible=(turn_allowed and requested and self.lane_turn_controller.enabled
                  and 0 <= v_ego < self.lane_turn_controller.lane_turn_value),
        entry_allowed=left_turn_allowed if selected_left else right_turn_allowed,
        keep_allowed=left_keep if selected_left else right_keep,
        hard_blocked=left_turn_blocked if selected_left else right_turn_blocked,
        completed=turn_completed,
        retry_soft_loss=turn_soft_reentry,
      )
      left_turn_allowed = active_turn and selected_left
      right_turn_allowed = active_turn and not selected_left
      if self.turn_maneuver.state == 'finished':
        self._cancelled_signal = True  # All exits consume this signal, including a speed exit.
    self.lane_turn_controller.update_lane_turn(
      blindspot_left=left_turn_blocked, blindspot_right=right_turn_blocked,
      left_blinker=lane_turn_left_blinker and turn_allowed and left_turn_allowed and not lane_turn_right_blinker,
      right_blinker=lane_turn_right_blinker and turn_allowed and right_turn_allowed and not lane_turn_left_blinker, v_ego=v_ego,
    )
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()
    # Diagnostic only: keep the output and all authorization conditions above unchanged.
    self.turn_decision_reason = (
      'legacyEntry' if not separate_turn_entry else
      'noSingleSignal' if signal_direction is None else
      'lateralInactive' if not lateral_active else
      'turnDisabled' if not self.lane_turn_controller.enabled else
      'signalConsumed' if self._cancelled_signal else
      'signalClassifiedAsLaneChange' if self._signal_is_lane_change else
      'laneChangeInProgress' if self.lane_change_state != LaneChangeState.off else
      'neighborPresent' if neighbor is True else
      'signalOwnedByLaneChange' if not requested else
      'speedIneligible' if not 0 <= v_ego < self.lane_turn_controller.lane_turn_value else
      'boundaryOrSafetyBlocked' if (left_turn_blocked if selected_left else right_turn_blocked) else
      'turnActive' if active_turn else 'entryNotConfirmed'
    )

    self.prev_one_blinker = one_blinker and lateral_active

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight

    self.alc.update_state()
