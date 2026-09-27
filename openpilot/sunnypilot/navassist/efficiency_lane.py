"""ARS408 route-compatible lane benefit selection; execution remains in SP."""
import math
from dataclasses import replace

from openpilot.sunnypilot.navassist.settings import NavAssistSettings
from openpilot.sunnypilot.navassist.lane_intent import LaneIntentDirection, NavLanePlan, ObservedLaneChangeState


def lane_leads(radar, model, *, left_neighbor=True, right_neighbor=True, all_targets=False):
  """Nearest measured target per model corridor; None list means unknown geometry.

  Radar y is left-positive, model y is right-positive. Interpolate each model
  boundary at target range, without extrapolating beyond the visible corridor.
  """
  if len(model.laneLines) != 4:
    return None
  # An absent outer lane must not veto evaluation of the other side. Keep
  # both ego boundaries, plus only the outer boundaries of existing neighbors.
  lanes = [1] + ([0] if left_neighbor else []) + ([2] if right_neighbor else [])
  indices = sorted({i for lane in lanes for i in (lane, lane+1)})
  lines = model.laneLines
  for index in indices:
    line = lines[index]
    if len(line.x) < 2 or len(line.x) != len(line.y):
      return None
    if not all(math.isfinite(v) for v in (*line.x, *line.y)):
      return None
    if any(line.x[i] <= line.x[i-1] for i in range(1, len(line.x))):
      return None
  leads = [[], [], []]
  for point in radar.points:
    d, y, v = float(point.dRel), -float(point.yRel), float(point.vRel)
    if not all(math.isfinite(value) for value in (d, y, v)):
      return None
    if d <= 0 or d > 100:
      continue
    # Same radar-to-model origin convention as controls/radard.py.
    model_x = d + 1.52
    # Beyond the model's ego corridor there is no supported lane assignment.
    # Do not let an out-of-horizon roadside track invalidate an observed side.
    if any(not lines[index].x[0] <= model_x <= lines[index].x[-1] for index in (1, 2)):
      continue
    bounds = {}
    for index in indices:
      line = lines[index]
      if not line.x[0] <= model_x <= line.x[len(line.x)-1]:
        bounds[index] = None
        continue
      for i in range(1, len(line.x)):
        if model_x <= line.x[i]:
          ratio = (model_x - line.x[i-1]) / (line.x[i] - line.x[i-1])
          bounds[index] = line.y[i-1] + ratio * (line.y[i] - line.y[i-1])
          break
    # These boundaries only associate radar tracks with a corridor. Crossing
    # permission comes from the C3 visual/0x399 gate, not a lane-width rule.
    if any(b is not None and math.isclose(y, b, rel_tol=0., abs_tol=1e-6) for b in bounds.values()):
      return None  # A target exactly on a boundary has no lane assignment.
    for lane in lanes:
      if bounds[lane] is None or bounds[lane+1] is None:
        if (lane == 0 and y < bounds[1]) or (lane == 2 and y > bounds[2]):
          return None  # The selected side has a target beyond its visible outer line.
        continue
      if bounds[lane] < y < bounds[lane+1]:
        if not point.deprecated.measured:
          return None
        leads[lane].append(point)
  return leads if all_targets else [min(targets, key=lambda point: point.dRel, default=None) for targets in leads]


def side_lead_unsafe(lead, speed_mps):
  """Adjacent forward target check from CarrotPilot SideState, using ARS408 vRel."""
  if lead is None:
    return False
  distance = lead.dRel
  relative_speed = lead.vRel
  if distance <= 5.:
    return True
  min_gap = min(12., max(6., speed_mps * .30))
  closing_speed = max(0., -relative_speed)
  if closing_speed > .5:
    ttc_limit = 2. + min(speed_mps, 15.) / 15. if speed_mps < 15. else 3. + min(speed_mps - 15., 15.) / 30.
    if distance < max(min_gap + 6., speed_mps * 1.2) and distance / closing_speed < ttc_limit:
      return True
    if distance + relative_speed * 1.5 < min_gap:
      return True
  return distance < min_gap and relative_speed < 1.


class EfficiencyLaneSelector:
  """One adjacent change, persistent benefit, then a fresh evaluation after cooldown."""
  COOLDOWN_NS = 10_000_000_000
  FAILED_COOLDOWN_NS = 4_000_000_000

  def __init__(self):
    self.settings = NavAssistSettings()
    self.active = None
    self.started = False
    self.candidate = None
    self.since = 0
    self.cooldown_until = 0
    self.sequence = 0
    self.reason = 'efficiencyIdle'
    self.unconfirmed_session = None
    self.terminal_reason = 'efficiencyCompletionUnconfirmed'

  def cancel(self, now_ns, reason):
    if self.active is not None:
      if self.started and reason not in ('efficiencyComplete', 'efficiencyCancelled'):
        self.unconfirmed_session = self.active.session_id
        self.terminal_reason = 'efficiencyCompletionUnconfirmed'
      self.cooldown_until = now_ns + self.settings.overtake_failed_cooldown_s * 1_000_000_000
    self.active = None
    self.started = False
    self.candidate = None
    self.reason = reason

  def select(self, route_plan, nav, topology, vehicle, radar, model, oem, *,
             healthy, enabled, cruise_mps, route_reserved, now_ns, settings=None, execution_healthy=None):
    if settings is not None:
      self.settings = settings
    policy = self.settings
    route_key = (str(nav.sessionId), int(nav.routeRevision))
    if (self.active is not None and vehicle.lane_change_state == ObservedLaneChangeState.starting
        and vehicle.lane_change_direction == self.active.edge_direction):
      self.started = True
    source_valid = (enabled and healthy and nav.valid and not nav.stale and nav.routeActive
             and nav.routeMatched and str(nav.mode) == 'realtime' and nav.roadClass in (0, 6)
             and vehicle.lateral_active
             and not (vehicle.brake_pressed or vehicle.steering_pressed)
             and math.isfinite(vehicle.speed_mps) and vehicle.speed_mps >= 60 / 3.6)
    valid = source_valid and topology.valid_for_control
    execution_valid = (enabled and (healthy if execution_healthy is None else execution_healthy)
                       and nav.valid and not nav.stale and nav.routeActive and nav.routeMatched
                       and str(nav.mode) == 'realtime' and vehicle.lateral_active
                       and not (vehicle.brake_pressed or vehicle.steering_pressed)
                       and math.isfinite(vehicle.speed_mps))
    # An interrupted attempt is not a completed change. Wait for the original
    # model and physical lamps to settle, then use the existing failure cooldown
    # and a fresh benefit evaluation (jihui finishOvertake(false)). A route/session
    # change alone cannot establish that the previous manoeuvre has stopped.
    if self.unconfirmed_session is not None:
      if (not valid or not execution_valid or not route_key[0]
          or vehicle.lane_change_state != ObservedLaneChangeState.off
          or vehicle.lane_change_direction != LaneIntentDirection.none
          or vehicle.left_blinker or vehicle.right_blinker):
        self.reason = self.terminal_reason
        return route_plan, False
      self.unconfirmed_session = None
    # Radar, road class, start speed and topology qualify a new request only.
    if (self.active is not None and self.started and execution_valid
        and (self.active.session_id, self.active.route_revision) == route_key):
      return replace(self.active, lane_count=topology.visible_lane_count), True
    if self.active is not None and self.started and not execution_valid:
      self.cancel(now_ns, 'efficiencyUnavailable')
      return route_plan, False
    if not valid:
      self.cancel(now_ns, 'efficiencyUnavailable')
      return route_plan, False
    if self.active is not None:
      if (self.active.session_id, self.active.route_revision) != route_key:
        self.cancel(now_ns, 'efficiencyRouteChanged')
        return route_plan, False
      if self.started:
        # Do not replace an executing manoeuvre with a new navigation request.
        return replace(self.active, lane_count=topology.visible_lane_count), True
    if route_reserved:
      self.cancel(now_ns, 'efficiencyNavigationPriority')
      return route_plan, False
    if now_ns < self.cooldown_until:
      self.reason = 'efficiencyCooldown'
      return route_plan, False
    if not math.isfinite(cruise_mps) or cruise_mps <= 0:
      self.cancel(now_ns, 'efficiencyCruiseUnavailable')
      return route_plan, False
    # Each eligible side is associated separately, so its unrelated neighbor
    # cannot turn a known forward gap into an unknown one.
    active_direction = self.active.edge_direction if self.active is not None else LaneIntentDirection.none
    choices = []
    gains = {}
    observations = {}
    for lane, side, direction in ((0, 'left', LaneIntentDirection.left), (2, 'right', LaneIntentDirection.right)):
      if (active_direction not in (LaneIntentDirection.none, direction)
          or side == 'right' and not policy.overtake_allow_right):
        continue
      permitted = (not (oem.get('permissionValid', False) and oem.get(side + 'SafetyBlocked', False))
                   and getattr(topology, side + '_neighbor_exists') is True
                   and getattr(topology, side + '_crossing_allowed')
                   and not getattr(vehicle, side + '_blindspot'))
      if not permitted:
        continue
      targets = lane_leads(radar, model, left_neighbor=side == 'left', right_neighbor=side == 'right', all_targets=True)
      observations[direction] = targets
      if targets is None:
        continue
      current = min(targets[1], key=lambda point: point.dRel, default=None)
      if current is None:
        continue
      demand = (policy.overtake_min_distance_m <= current.dRel <= policy.overtake_max_distance_m
                and vehicle.speed_mps + current.vRel >= policy.overtake_lead_min_kph / 3.6
                and (current.vRel < -policy.overtake_closing_kph / 3.6
                     or current.dRel / vehicle.speed_mps <= policy.overtake_time_gap_tenths / 10
                     or vehicle.speed_mps < cruise_mps * policy.overtake_cruise_percent / 100))
      if not demand:
        continue
      lead = min(targets[lane], key=lambda point: point.dRel, default=None)
      # The nearest target sets expected speed; every measured side target must
      # pass the same CP forward-gap rule before requesting the original SP path.
      gap_ok = all(not side_lead_unsafe(point, vehicle.speed_mps) for point in targets[lane])
      target_speed = cruise_mps if lead is None else min(cruise_mps, vehicle.speed_mps + lead.vRel)
      gain = target_speed - min(cruise_mps, vehicle.speed_mps + current.vRel)
      if gap_ok and gain > 0:
        choices.append(direction)
        gains[direction] = gain
    direction = (choices[0] if policy.overtake_prefer_left else max(choices, key=gains.get)) if choices else LaneIntentDirection.none
    if self.active is not None:
      if self.active.edge_direction in choices:
        return self.active, True
      side = 'left' if active_direction == LaneIntentDirection.left else 'right'
      targets = observations.get(active_direction)
      side_targets = targets[0 if side == 'left' else 2] if targets is not None else []
      reason = ('efficiencyRightDisabled' if side == 'right' and not policy.overtake_allow_right
                else 'efficiencySafetyBlocked' if (oem.get('permissionValid', False) and oem.get(side + 'SafetyBlocked', False)) or getattr(vehicle, side + '_blindspot')
                else 'efficiencyCrossingLost' if not getattr(topology, side + '_crossing_allowed')
                else 'efficiencyNeighborLost' if getattr(topology, side + '_neighbor_exists') is not True
                else 'efficiencyLaneGeometryUnknown' if targets is None
                else 'efficiencyTargetUnknown' if not targets[1]
                else 'efficiencyGapLost' if any(side_lead_unsafe(point, vehicle.speed_mps) for point in side_targets)
                else 'efficiencyBenefitLost')
      self.cancel(now_ns, reason)
      return route_plan, False
    if direction == LaneIntentDirection.none:
      self.candidate = None
      self.reason = ('efficiencyLaneGeometryUnknown' if observations and all(value is None for value in observations.values()) else
                     'efficiencyNoConfirmedBenefit')
      return route_plan, False
    if vehicle.left_blinker or vehicle.right_blinker or vehicle.lane_change_state != ObservedLaneChangeState.off:
      self.candidate = None
      self.reason = 'efficiencyManualOrActiveChange'
      return route_plan, False
    # Confirm continuing benefit in the chosen direction, as in jihui's demand
    # timer. Radar track identity changes are not a loss of benefit or safety;
    # all current targets were checked above on this frame.
    key = (route_key, direction)
    if key != self.candidate:
      self.candidate, self.since = key, now_ns
    self.reason = 'efficiencyBenefitStabilizing'
    if now_ns - self.since < policy.overtake_stable_ms * 1_000_000:
      return route_plan, False
    self.sequence += 1
    target = 0 if direction == LaneIntentDirection.left else max(0, topology.visible_lane_count - 1)
    self.active = NavLanePlan(True, *route_key, (1 << 63) + self.sequence,
                              topology.visible_lane_count, (target,), heuristic=True, edge_direction=direction)
    self.reason = 'efficiencyRequest'
    return self.active, True

  def observe(self, intent, now_ns):
    if self.active is None:
      return
    if intent.reason == 'laneChangeCancelled':
      self.cancel(now_ns, 'efficiencyCancelled')
    elif intent.reason in ('laneChangeCompletionUnconfirmed', 'laneChangeAnnouncementFailed'):
      self.unconfirmed_session = self.active.session_id
      self.cancel(now_ns, 'efficiencyCompletionUnconfirmed')
      self.terminal_reason = ('efficiencyCancelled' if intent.reason == 'laneChangeCancelled'
                              else 'efficiencyAnnouncementFailed' if intent.reason == 'laneChangeAnnouncementFailed'
                              else 'efficiencyCompletionUnconfirmed')
      self.reason = self.terminal_reason
    elif intent.reason in ('laneChangeObserved', 'laneChangeComplete'):
      self.cancel(now_ns, 'efficiencyComplete')
      self.cooldown_until = now_ns + self.settings.overtake_success_cooldown_s * 1_000_000_000
    elif intent.reason in ('blockedEvent', 'health', 'noNeighbor', 'routeChanged',
                           'directionMismatch', 'physicalSignalLost', 'laneChangeTimeout', 'crossingWaitTimeout'):
      self.cancel(now_ns, 'efficiencyAborted:' + intent.reason)
