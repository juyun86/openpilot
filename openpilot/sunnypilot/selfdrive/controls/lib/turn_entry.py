"""CP-style turn intent with SP control vetoes; legacy opening API is retained."""
import math
from collections import deque

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants

from openpilot.sunnypilot.navassist.lane_publisher import MODEL_MAX_AGE_NS, MARKING_MAX_AGE_NS
from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_blocker import LaneChangeBoundaryBlocker

OPENING_STABLE_NS = 500_000_000
FAR_CLEARANCE_M = 4.0  # Candidate distance-space threshold, not CP's time-space metric.
MIN_WIDENING_M = 1.0
MIN_TURN_UNWIND_DEG = 5.0  # Candidate noise floor; requires matched-road validation.


class _ExistCounter:
  # CP cp-dev-rs408@072b807d: four consecutive observations before changing sign.
  def __init__(self):
    self.counter = self.true_count = self.false_count = 0

  def update(self, present):
    self.true_count = self.true_count + 1 if present else 0
    self.false_count = 0 if present else self.false_count + 1
    if self.true_count >= 4:
      self.counter = max(self.counter + 1, 1)
    elif self.false_count >= 4:
      self.counter = min(self.counter - 1, -1)


class TurnIntentClassifier:
  """CP's maneuver score/side history; this class grants no control permission."""
  def __init__(self):
    self.widths = [deque(maxlen=20), deque(maxlen=20)]
    self.counters = [[_ExistCounter() for _ in range(3)] for _ in range(2)]
    self.stamp = 0

  def update(self, model, carstate, *, stamp, now_ns, healthy=True, nav_intent=None, nav_state=None):
    live = healthy and 0 < stamp <= now_ns and now_ns-stamp <= MODEL_MAX_AGE_NS
    if not live or stamp <= self.stamp or (self.stamp and stamp-self.stamp > MODEL_MAX_AGE_NS):
      self.__init__()
      if not live:
        return ((False, False), (False, False))
      # A duplicate/gap cannot establish new evidence on this observation.
      self.stamp = stamp
      return ((False, False), (False, False))
    self.stamp = stamp
    turns, observed = [], []
    for i, (outer, ego) in enumerate(((0, 1), (3, 2))):
      try:
        curves = [list(model.laneLines[j].y) for j in (outer, ego)] + [list(model.roadEdges[i].y)]
        prob = float(model.laneLineProbs[outer])
        speed, accel = float(carstate.vEgo)*3.6, float(carstate.aEgo)
        valid = (all(len(y) == len(ModelConstants.T_IDXS) and all(map(math.isfinite, y)) for y in curves)
                 and all(map(math.isfinite, (prob, speed, accel))) and 0 <= prob <= 1 and speed >= 0)
        if not valid:
          raise ValueError('invalid model geometry')
        lane, current, edge = [float(np.interp(1., ModelConstants.T_IDXS, y)) for y in curves]
        far_edge = float(np.interp(2., ModelConstants.T_IDXS, curves[2]))
        sign = -1 if i == 0 else 1
        # CP uses absolute distances. Preserve SP's rejection of crossing/inverted edges.
        near, far = sign*(edge-current), sign*(far_edge-current)
        if near < 0 or far < 0:
          raise ValueError('edge on opposite side')
      except (AttributeError, IndexError, TypeError, ValueError):
        self.widths[i].clear()
        self.counters[i] = [_ExistCounter() for _ in range(3)]
        turns.append(False)
        observed.append(False)
        continue
      self.widths[i].append(min(abs(lane-current), near))
      lane_count, width_count, edge_count = self.counters[i]
      evidence = (prob > .5, sum(self.widths[i])/len(self.widths[i]) > 2.5, near > 2.5)
      for counter, value in zip(self.counters[i], evidence, strict=True):
        counter.update(value)
      lane_available = width_count.counter > 4
      edge_available = edge_count.counter > 4 and far > 2.5
      score = int(speed < 30 or (speed < 40 and accel < -1))
      score += int(speed < 40 and not lane_available and not edge_available)
      score += int(speed < 40 and lane_count.counter < 10)
      desires = list(getattr(getattr(model, 'meta', None), 'desireState', ()))
      if len(desires) >= 3 and all(math.isfinite(v) for v in desires[1:3]):
        score += int(sum(desires[1:3]) > .1)
      direction = ('left', 'right')[i]
      if (nav_intent is not None and nav_intent.valid and nav_intent.signalRequested
          and str(nav_intent.direction) == direction):
        if nav_intent.targetLaneIndex >= 0:
          score -= 2  # A navigation lane request must not become a turn by slowing.
        elif (nav_state is not None and nav_state.valid and not nav_state.stale and not nav_state.gpsWeak
              and nav_state.routeActive and nav_state.routeMatched
              and 0 < nav_state.publishMonoTime <= now_ns
              and now_ns-nav_state.publishMonoTime <= MARKING_MAX_AGE_NS
              and nav_state.maneuverEventId != 0
              and (nav_state.sessionId, nav_state.routeRevision, nav_state.maneuverEventId)
                  == (nav_intent.sessionId, nav_intent.routeRevision, nav_intent.maneuverEventId)
              and str(nav_state.maneuver) == ('turnLeft', 'turnRight')[i]):
          score += 2
      # Allow CP's four-frame hysteresis plus ten positive counts to settle.
      # Otherwise a stable lane is initially mis-scored as missing (counter < 10).
      turns.append(len(self.widths[i]) >= 13 and score >= 2 and far > 4.)
      # Existing turn may continue as the edge moves alongside; caller applies hazards/age.
      observed.append(True)
    return tuple(turns), tuple(observed)


def _sample(line, distance):
  xs, ys = list(line.x), list(line.y)
  if len(xs) != len(ys) or len(xs) < 2:
    return None
  if not all(math.isfinite(v) for v in xs + ys) or any(b <= a for a, b in zip(xs, xs[1:], strict=False)):
    return None
  for i in range(1, len(xs)):
    if xs[i-1] <= distance <= xs[i]:
      weight = (distance-xs[i-1])/(xs[i]-xs[i-1])
      return ys[i-1] + weight*(ys[i]-ys[i-1])
  return None  # Never extrapolate a missing far edge into an opening.


def _side_clearances(model, side):
  """Signed distances outside the ego boundary; None means unobserved."""
  return _side_geometry(model, side)[0]


def _side_geometry(model, side):
  """Preserve the existing gate and expose why a curve was rejected."""
  if len(model.roadEdges) != 2 or len(model.roadEdgeStds) != 2 or len(model.laneLines) != 4:
    return None, 'geometryMissing'
  i = 0 if side == 'left' else 1
  std = model.roadEdgeStds[i]
  if not math.isfinite(std) or not 0 <= std <= 0.35:
    return None, 'roadEdgeUncertain'
  edge, boundary = model.roadEdges[i], model.laneLines[i+1]
  points = [_sample(line, distance) for distance in (5., 20.) for line in (edge, boundary)]
  if any(v is None for v in points):
    return None, 'curveInvalidOrShort'
  sign = -1 if side == 'left' else 1
  return (sign*(points[0]-points[1]), sign*(points[2]-points[3])), 'geometryObserved'


def side_opening(model, side):
  """Require a confidently observed edge that opens beyond the ego boundary."""
  points = _side_clearances(model, side)
  return points is not None and _is_opening(*points)


def _is_opening(near, far):
  return 0 <= near < FAR_CLEARANCE_M and far >= FAR_CLEARANCE_M and far-near >= MIN_WIDENING_M


def turn_end_detected(model, carstate):
  """CP's completion hint; callers apply it only to an already active turn."""
  rates = model.orientationRate.z
  angle = getattr(carstate, 'steeringAngleDeg', 0.)
  return (len(rates) > 15 and all(math.isfinite(v) for v in (angle, rates[5], rates[15]))
          and abs(angle) > 80. and abs(rates[15]) < abs(rates[5]))


class TurnCompletionTracker:
  """Confirm CP's end hint against measured unwinding, using actuator delay.

  This withdraws a model intent; it does not command steering or change limits.
  """
  def __init__(self, actuator_delay):
    self.confirm_ns = (int(round(max(0.05, actuator_delay)*1e6))*1000
                       if math.isfinite(actuator_delay) and 0 <= actuator_delay <= 1. else None)
    self._reset()

  def _reset(self):
    self._direction = None
    self._stamp = 0
    self._angle = None
    self._unwind_since = None
    self._peak_angle = 0.
    self._unwind_start_angle = None

  def update(self, model, carstate, *, active, model_stamp_ns, now_ns):
    direction = ('left' if carstate.leftBlinker and not carstate.rightBlinker else
                 'right' if carstate.rightBlinker and not carstate.leftBlinker else None)
    angle = getattr(carstate, 'steeringAngleDeg', float('nan'))
    if (not active or direction is None or self.confirm_ns is None or not math.isfinite(angle)
        or model_stamp_ns <= 0 or not 0 <= now_ns-model_stamp_ns <= MODEL_MAX_AGE_NS):
      self._reset()
      return False
    if direction != self._direction:
      self._reset()
      self._direction = direction
    if model_stamp_ns <= self._stamp:
      self._angle = None
      self._unwind_since = None
      self._peak_angle = 0.
      self._unwind_start_angle = None
      return False
    if model_stamp_ns > self._stamp+MODEL_MAX_AGE_NS:
      # First sample or a gap cannot prove unwinding.
      self._angle = None
      self._unwind_since = None
      self._peak_angle = 0.
      self._unwind_start_angle = None
    # CarState steering angle is left-positive, unlike model lateral y.
    directed_angle = angle if direction == 'left' else -angle
    if directed_angle < 0:
      self._peak_angle = 0.
    else:
      self._peak_angle = max(self._peak_angle, directed_angle)
    rates = model.orientationRate.z
    # The angle threshold arms this continuous turn, not every unwind sample.
    # Keep requiring a current prediction; a historical hint is not completion.
    hint = (directed_angle >= 0 and self._peak_angle > 80. and len(rates) > 15
            and all(math.isfinite(v) for v in (rates[5], rates[15]))
            and abs(rates[15]) < abs(rates[5]))
    if hint and self._angle is not None and directed_angle < self._angle:
      if self._unwind_since is None:
        self._unwind_since = self._stamp
        self._unwind_start_angle = self._angle
    else:
      self._unwind_since = None
      self._unwind_start_angle = None
    self._angle, self._stamp = directed_angle, model_stamp_ns
    return (self._unwind_since is not None and model_stamp_ns-self._unwind_since >= self.confirm_ns
            and self._unwind_start_angle-directed_angle >= MIN_TURN_UNWIND_DEG)


class TurnManeuver:
  """Hard exits consume the lamp session; qualified soft-loss recovery is optional."""
  def __init__(self):
    self.direction = None
    self.state = 'waiting'

  def update(self, direction, *, eligible, entry_allowed, keep_allowed, hard_blocked, completed=False, retry_soft_loss=False):
    if direction != self.direction:
      self.direction = direction
      self.state = 'waiting'
    if direction is None:
      return False
    if self.state == 'active':
      if not eligible or hard_blocked or completed:
        self.state = 'finished'
      elif not keep_allowed:
        self.state = 'waiting' if retry_soft_loss else 'finished'
    elif self.state == 'waiting' and eligible and entry_allowed and not hard_blocked:
      self.state = 'active'
    return self.state == 'active'


class TurnEntryGate:
  def __init__(self):
    self._since = [None, None]
    self._last_stamp = [0, 0]
    self._last_good = [0, 0]
    self._boundary = LaneChangeBoundaryBlocker()
    self._boundary_stamp = 0
    self.hard_blocks = (False, False)
    self.keep_allowed = (False, False)
    self.neighbors = (None, None)
    self.reasons = ('unknown', 'unknown')
    self.detail_reasons = ('unknown', 'unknown')
    self.input_reason = 'notObserved'
    self.classifier = TurnIntentClassifier()

  def update(self, model, topology, *, healthy, now_ns, model_stamp_ns, neighbors,
             safety_blocks=(False, False), carstate=None, model_healthy=True, nav_intent=None, nav_state=None):
    fresh = bool(healthy and topology.validForControl and 0 < model_stamp_ns <= now_ns
                 and now_ns-model_stamp_ns <= MODEL_MAX_AGE_NS)
    for field, age in [('publishMonoTime', MODEL_MAX_AGE_NS), ('modelMonoTime', MODEL_MAX_AGE_NS),
                       ('imageMonoTime', MARKING_MAX_AGE_NS)]:
      stamp = getattr(topology, field, 0)
      fresh = fresh and stamp > 0 and 0 <= now_ns-stamp <= age
    self.input_reason = ('sourceUnavailable' if not healthy else
                         'topologyInvalidForControl' if not topology.validForControl else
                         'modelTimeInvalid' if not 0 < model_stamp_ns <= now_ns or now_ns-model_stamp_ns > MODEL_MAX_AGE_NS else
                         'topologyTimeInvalid' if not fresh else 'observed')
    # A solid->unknown transition cannot erase boundary memory. Count each
    # topology observation only once; repeated model frames cannot clear it.
    topology_stamp = getattr(topology, 'modelMonoTime', 0)
    if fresh and topology_stamp > self._boundary_stamp:
      self.hard_blocks = self._boundary.update(topology, healthy=True)
      self._boundary_stamp = topology_stamp
    elif not fresh:
      self.hard_blocks = self._boundary.update(topology, healthy=False)
    self.neighbors = tuple(True if fresh and getattr(topology, side+'NeighborExists') else neighbors[i]
                           for i, side in enumerate(('left', 'right')))
    cp_mode = carstate is not None
    cp_turns, cp_observed = ((False, False), (False, False))
    if cp_mode:
      cp_turns, cp_observed = self.classifier.update(
        model, carstate, stamp=model_stamp_ns, now_ns=now_ns, healthy=model_healthy,
        nav_intent=nav_intent, nav_state=nav_state,
      )
      # Topology still provides boundary/neighbor vetoes above. A lost paint
      # classification must not invalidate the current model's separate turn evidence.
      fresh = bool(model_healthy and 0 < model_stamp_ns <= now_ns
                   and now_ns-model_stamp_ns <= MODEL_MAX_AGE_NS)
      self.input_reason = 'cpModelObserved' if fresh else 'modelUnavailable'
    allowed, continuing, reasons, details = [], [], [], []
    for i, side in enumerate(('left', 'right')):
      continuity_lost = self._last_good[i] > 0 and not 0 <= now_ns-self._last_good[i] <= MODEL_MAX_AGE_NS
      clearances, geometry_reason = _side_geometry(model, side) if fresh and not cp_mode else (None, 'notEvaluated')
      if cp_mode:
        geometry_reason = 'cpTurn' if cp_turns[i] else 'cpNotTurn' if cp_observed[i] else 'cpGeometryUnavailable'
      advancing = model_stamp_ns > self._last_stamp[i]
      # An already confirmed opening may move alongside the car. Preserve
      # that continuous observation instead of restarting confirmation merely
      # because the near clearance crossed 4 m. A wide shoulder on its own
      # cannot establish this history; gaps/unknowns still reset entry.
      opening_at_car = (clearances is not None and self._since[i] is not None
                        and self._last_stamp[i]-self._since[i] >= OPENING_STABLE_NS
                        and 0 < model_stamp_ns-self._last_stamp[i] <= MODEL_MAX_AGE_NS
                        and min(clearances) >= FAR_CLEARANCE_M)
      # Hazards and positive contradictions revoke immediately, even during
      # a transient loss of another input. Unknown is not a new entry permit.
      if safety_blocks[i] or self.hard_blocks[i]:
        reason = 'boundaryOrSafetyBlocked'
      elif self.neighbors[i] is True:
        reason = 'neighborPresent'
      elif not fresh:
        reason = 'evidenceUnavailable'
      elif neighbors[i] is not False:
        reason = 'neighborUnknown'
      elif not advancing:
        reason = 'modelNotAdvancing'
      elif cp_mode and not cp_observed[i]:
        reason = 'geometryUnavailable'
      elif cp_mode and not cp_turns[i]:
        reason = 'openingUnconfirmed'
      elif not cp_mode and clearances is None:
        reason = 'geometryUnavailable'
      elif not cp_mode and min(clearances) < 0:
        reason = 'boundaryConflict'
      elif not cp_mode and not _is_opening(*clearances) and not opening_at_car:
        reason = 'openingUnconfirmed'
      else:
        reason = 'openingConfirming'
      if reason != 'openingConfirming':
        self._since[i] = None
        allowed.append(False)
      else:
        if self._since[i] is None or model_stamp_ns-self._last_stamp[i] > MODEL_MAX_AGE_NS:
          self._since[i] = model_stamp_ns
        ready = cp_mode or model_stamp_ns-self._since[i] >= OPENING_STABLE_NS
        allowed.append(ready)
        if ready:
          reason = 'turnOpeningConfirmed'
      if reason in ('openingConfirming', 'turnOpeningConfirmed', 'openingUnconfirmed'):
        # Once inside a turn, the widening entrance need not stay ahead of us.
        # Still require fresh, non-conflicting boundaries and neighbor data.
        self._last_good[i] = model_stamp_ns
      elif reason not in ('evidenceUnavailable', 'neighborUnknown', 'modelNotAdvancing', 'geometryUnavailable'):
        self._last_good[i] = 0
      if cp_mode and reason in ('evidenceUnavailable', 'neighborUnknown'):
        # Re-entry after losing a source needs new history, not the pre-gap score.
        self.classifier.widths[i].clear()
        self.classifier.counters[i] = [_ExistCounter() for _ in range(3)]
      continuing.append((not continuity_lost or allowed[-1]) and self._last_good[i] > 0
                        and 0 <= now_ns-self._last_good[i] <= MODEL_MAX_AGE_NS)
      if advancing:
        self._last_stamp[i] = model_stamp_ns
      reasons.append(reason)
      primary = ('oemSafetyBlocked' if safety_blocks[i] else 'boundaryHeld') if reason == 'boundaryOrSafetyBlocked' else reason
      details.append(primary + '/' + geometry_reason)
    self.keep_allowed = tuple(continuing)
    self.reasons = tuple(reasons)
    self.detail_reasons = tuple(details)
    return tuple(allowed)
