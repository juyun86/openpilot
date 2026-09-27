from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import threading
import time
from typing import Any


SCHEMA_VERSION = 3
MESSAGE_TYPE = "navigation_snapshot"
MAX_BODY_BYTES = 64 * 1024
MAX_VALID_FOR_MS = 2_000
MIN_VALID_FOR_MS = 100
MAX_SOURCE_AGE_MS = 2_000
SOURCE_DELIVERY_GRACE_MS = 500
MAX_SOURCE_FUTURE_SKEW_MS = 1_000
MAX_LANES = 16
MAX_SESSION_ID_LENGTH = 64
MAX_ROAD_NAME_LENGTH = 256
CHECKPOINT_VERSION = 2
SESSION_ID_PATTERN = re.compile(r"^[A-Za-z0-9._:-]+$")
APP_KEY_ID_PATTERN = re.compile(r"^[0-9a-f]{32}$")

SOURCE_VALUES = frozenset(("android", "ios", "track"))
MODE_VALUES = frozenset(("idle", "route_planned", "realtime", "simulation", "arrived", "recalculating"))
COORDINATE_SYSTEM_VALUES = frozenset(("gcj02", "wgs84", "unknown"))
ROAD_LAYER_VALUES = frozenset(("unknown", "main", "side"))
ROUTE_NOTICE_VALUES = frozenset((
  "none", "restricted_area", "forbidden_area", "road_closed", "congestion", "dispatch", "route_changed",
  "gps_weak", "unknown",
))
MANEUVER_VALUES = frozenset((
  "none", "straight", "slight_left", "slight_right", "turn_left", "turn_right", "sharp_left", "sharp_right",
  "u_turn_left", "u_turn_right", "keep_left", "keep_right", "merge_left", "merge_right", "exit_left",
  "exit_right", "ramp_left", "ramp_right", "roundabout", "destination", "unknown",
))
ROAD_TYPE_VALUES = frozenset((1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 53, 56, 58))

LANE_ACTION_BITS = {
  "STRAIGHT": 1 << 0,
  "LEFT": 1 << 1,
  "RIGHT": 1 << 2,
  "U_TURN": 1 << 3,
  "LEFT_U_TURN": 1 << 3,
  "RIGHT_U_TURN": 1 << 4,
  "BUS": 1 << 5,
  "VARIABLE": 1 << 6,
  "DEDICATED": 1 << 7,
  "TIDAL": 1 << 8,
  "UNKNOWN": 1 << 15,
}

TOP_LEVEL_KEYS = frozenset((
  "schemaVersion", "messageType", "sessionId", "sequence", "routeRevision", "maneuverEventId", "sourcePlatform",
  "sourceWallTimeMs", "validForMs", "navigationMode", "routeActive", "routeMatched", "gpsWeak",
  "coordinateSystem", "location", "guidance", "lanes", "laneChangeSpeechCompletedId",
))
LOCATION_KEYS = frozenset((
  "latitude", "longitude", "accuracyM", "bearingDeg", "speedKph", "observedAtMs", "currentStepIndex",
  "currentLinkIndex", "currentPointIndex",
))
GUIDANCE_KEYS = frozenset((
  "maneuver", "maneuverDistanceM", "nextManeuver", "nextManeuverDistanceM", "currentRoad", "nextRoad",
  "roadClass", "roadType", "advisorySpeedMps", "parallelRoadStatus", "elevatedRoadStatus", "routeNoticeType",
  "routeNoticeDistanceM", "routeNoticeObservedAtMs", "observedAtMs",
))
LANES_KEYS = frozenset(("observedAtMs", "items"))
LANE_ITEM_KEYS = frozenset(("index", "allowedActions", "recommendedActions", "recommended", "routeAvoid"))


class NavAssistProtocolError(ValueError):
  def __init__(self, reason: str, message: str):
    super().__init__(message)
    self.reason = reason


@dataclass(frozen=True)
class LaneGuidance:
  index: int
  allowed_actions: int
  recommended_actions: int
  recommended: bool
  route_avoid: bool


@dataclass(frozen=True)
class NavAssistSnapshot:
  session_id: str
  sequence: int
  route_revision: int
  maneuver_event_id: int
  source_platform: str
  source_wall_time_ms: int
  valid_for_ms: int
  navigation_mode: str
  route_active: bool
  route_matched: bool
  gps_weak: bool
  coordinate_system: str
  location_present: bool
  latitude: float
  longitude: float
  accuracy_m: float
  bearing_deg: float
  speed_kph: float
  location_observed_at_ms: int
  current_step_index: int
  current_link_index: int
  current_point_index: int
  guidance_present: bool
  maneuver: str
  guidance_observed_at_ms: int
  maneuver_distance_m: float
  next_maneuver: str
  next_maneuver_distance_m: float
  advisory_speed_mps: float | None
  road_class: int
  road_type: int
  current_road: str
  next_road: str
  parallel_road_status: str
  elevated_road_status: str
  route_notice_type: str
  route_notice_distance_m: float
  route_notice_observed_at_ms: int
  lane_guidance_present: bool
  lane_guidance_observed_at_ms: int
  lanes: tuple[LaneGuidance, ...]
  lane_change_speech_completed_id: str = ""


@dataclass(frozen=True)
class AcceptedSnapshot:
  snapshot: NavAssistSnapshot
  receive_mono_ns: int
  expires_mono_ns: int

  def age_ms(self, now_ns: int) -> float:
    return max(0.0, (now_ns - self.receive_mono_ns) / 1e6)

  def is_stale(self, now_ns: int) -> bool:
    return now_ns > self.expires_mono_ns


def _reject(reason: str, message: str) -> None:
  raise NavAssistProtocolError(reason, message)


def _object(value: Any, field: str, allowed_keys: frozenset[str]) -> dict[str, Any]:
  if not isinstance(value, dict):
    _reject("malformed", f"{field} must be an object")
  unknown = set(value) - allowed_keys
  if unknown:
    _reject("malformed", f"{field} contains unknown fields: {sorted(unknown)}")
  return value


def _required(obj: dict[str, Any], field: str) -> Any:
  if field not in obj:
    _reject("malformed", f"missing required field {field}")
  return obj[field]


def _boolean(obj: dict[str, Any], field: str) -> bool:
  value = _required(obj, field)
  if not isinstance(value, bool):
    _reject("malformed", f"{field} must be boolean")
  return value


def _optional_boolean(obj: dict[str, Any], field: str, default: bool = False) -> bool:
  if field not in obj:
    return default
  return _boolean(obj, field)


def _integer(obj: dict[str, Any], field: str, minimum: int, maximum: int, *, default: int | None = None) -> int:
  value = obj.get(field, default)
  if value is None or isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
    _reject("malformed", f"{field} must be an integer in [{minimum}, {maximum}]")
  return value


def _optional_integer(obj: dict[str, Any], field: str, minimum: int, maximum: int, *, default: int = -1) -> int:
  if field not in obj:
    return default
  return _integer(obj, field, minimum, maximum)


def _number(obj: dict[str, Any], field: str, minimum: float, maximum: float, *, default: float | None = None) -> float:
  value = obj.get(field, default)
  if value is None or isinstance(value, bool) or not isinstance(value, (int, float)):
    _reject("malformed", f"{field} must be numeric")
  result = float(value)
  if not math.isfinite(result) or not minimum <= result <= maximum:
    _reject("malformed", f"{field} must be finite and in [{minimum}, {maximum}]")
  return result


def _string(obj: dict[str, Any], field: str, *, default: str | None = None, max_length: int = 128) -> str:
  value = obj.get(field, default)
  if value is None or not isinstance(value, str) or len(value) > max_length:
    _reject("malformed", f"{field} must be a string no longer than {max_length}")
  return value


def _enum(obj: dict[str, Any], field: str, values: frozenset[str], *, default: str | None = None) -> str:
  value = _string(obj, field, default=default)
  if value not in values:
    _reject("malformed", f"{field} has unsupported value {value!r}")
  return value


def _lane_action_mask(value: Any, field: str) -> int:
  if not isinstance(value, list) or len(value) > 16:
    _reject("malformed", f"{field} must be an action list")
  mask = 0
  seen: set[str] = set()
  for item in value:
    if not isinstance(item, str) or item not in LANE_ACTION_BITS:
      _reject("malformed", f"{field} contains unsupported action {item!r}")
    if item in seen:
      _reject("malformed", f"{field} contains duplicate action {item!r}")
    seen.add(item)
    mask |= LANE_ACTION_BITS[item]
  return mask


def _documented_road_type(obj: dict[str, Any]) -> int:
  if "roadType" not in obj:
    return -1
  value = _integer(obj, "roadType", 1, 255)
  if value not in ROAD_TYPE_VALUES:
    _reject("malformed", f"roadType has unsupported value {value}")
  return value


def _optional_road_name(obj: dict[str, Any], field: str) -> str:
  if field not in obj:
    return ""
  value = _string(obj, field, max_length=MAX_ROAD_NAME_LENGTH)
  if not value:
    _reject("malformed", f"{field} must not be empty")
  return value


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
  result: dict[str, Any] = {}
  for key, value in pairs:
    if key in result:
      _reject("malformed", f"duplicate JSON field {key}")
    result[key] = value
  return result


def parse_snapshot(body: bytes) -> NavAssistSnapshot:
  if not body or len(body) > MAX_BODY_BYTES:
    _reject("malformed", f"body must contain 1..{MAX_BODY_BYTES} bytes")
  try:
    raw = json.loads(
      body.decode("utf-8"),
      parse_constant=lambda value: _reject("malformed", f"invalid number {value}"),
      object_pairs_hook=_reject_duplicate_keys,
    )
  except (UnicodeDecodeError, json.JSONDecodeError) as error:
    _reject("malformed", f"invalid JSON: {error}")
  top = _object(raw, "snapshot", TOP_LEVEL_KEYS)
  speech_id = _string(top, "laneChangeSpeechCompletedId", max_length=32) if "laneChangeSpeechCompletedId" in top else ""
  if speech_id and re.fullmatch(r"[0-9a-f]{32}", speech_id) is None:
    _reject("malformed", "invalid laneChangeSpeechCompletedId")

  if _integer(top, "schemaVersion", 0, 1_000) != SCHEMA_VERSION:
    _reject("malformed", "unsupported schemaVersion")
  if _string(top, "messageType") != MESSAGE_TYPE:
    _reject("malformed", "unsupported messageType")
  session_id = _string(top, "sessionId", max_length=MAX_SESSION_ID_LENGTH)
  if not session_id or SESSION_ID_PATTERN.fullmatch(session_id) is None:
    _reject("malformed", "sessionId has invalid characters")

  location_present = isinstance(top.get("location"), dict)
  guidance_present = isinstance(top.get("guidance"), dict)
  lane_guidance_present = isinstance(top.get("lanes"), dict)
  location = _object(top.get("location", {}), "location", LOCATION_KEYS)
  guidance = _object(top.get("guidance", {}), "guidance", GUIDANCE_KEYS)
  lane_block = _object(top.get("lanes", {}), "lanes", LANES_KEYS)
  lane_items = _required(lane_block, "items") if lane_guidance_present else []
  if not isinstance(lane_items, list) or len(lane_items) > MAX_LANES:
    _reject("malformed", f"lanes.items must contain no more than {MAX_LANES} lanes")

  lanes: list[LaneGuidance] = []
  seen_indexes: set[int] = set()
  for position, raw_lane in enumerate(lane_items):
    lane = _object(raw_lane, f"lanes.items[{position}]", LANE_ITEM_KEYS)
    index = _integer(lane, "index", 0, 31)
    if index in seen_indexes:
      _reject("malformed", f"duplicate lane index {index}")
    seen_indexes.add(index)
    lanes.append(LaneGuidance(
      index=index,
      allowed_actions=_lane_action_mask(_required(lane, "allowedActions"), f"lanes.items[{position}].allowedActions"),
      recommended_actions=_lane_action_mask(
        _required(lane, "recommendedActions"), f"lanes.items[{position}].recommendedActions",
      ),
      recommended=_boolean(lane, "recommended"),
      route_avoid=_optional_boolean(lane, "routeAvoid"),
    ))

  advisory_speed = guidance.get("advisorySpeedMps")
  if advisory_speed is not None:
    advisory_speed = _number(guidance, "advisorySpeedMps", 0.0, 60.0)
    if advisory_speed <= 0.0:
      _reject("malformed", "advisorySpeedMps must be greater than zero")

  latitude = _number(location, "latitude", -90.0, 90.0) if location_present else 0.0
  longitude = _number(location, "longitude", -180.0, 180.0) if location_present else 0.0
  accuracy_m = _number(location, "accuracyM", 0.0, 200.0) if location_present else 200.0
  bearing_deg = _number(location, "bearingDeg", 0.0, 360.0) if location_present else 0.0
  speed_kph = _number(location, "speedKph", 0.0, 300.0) if location_present else 0.0
  location_observed_at_ms = _integer(location, "observedAtMs", 0, 2**63 - 1) if location_present else 0
  guidance_observed_at_ms = _integer(guidance, "observedAtMs", 0, 2**63 - 1) if guidance_present else 0
  maneuver = _enum(guidance, "maneuver", MANEUVER_VALUES) if guidance_present else "none"
  lane_observed_at_ms = _integer(lane_block, "observedAtMs", 0, 2**63 - 1) if lane_guidance_present else 0

  return NavAssistSnapshot(
    session_id=session_id,
    lane_change_speech_completed_id=speech_id,
    sequence=_integer(top, "sequence", 1, 2**63 - 1),
    route_revision=_integer(top, "routeRevision", 0, 2**63 - 1),
    maneuver_event_id=_integer(top, "maneuverEventId", 0, 2**63 - 1),
    source_platform=_enum(top, "sourcePlatform", SOURCE_VALUES),
    source_wall_time_ms=_integer(top, "sourceWallTimeMs", 0, 2**63 - 1),
    valid_for_ms=_integer(top, "validForMs", MIN_VALID_FOR_MS, MAX_VALID_FOR_MS),
    navigation_mode=_enum(top, "navigationMode", MODE_VALUES),
    route_active=_boolean(top, "routeActive"),
    route_matched=_optional_boolean(top, "routeMatched"),
    gps_weak=_boolean(top, "gpsWeak"),
    coordinate_system=_enum(top, "coordinateSystem", COORDINATE_SYSTEM_VALUES),
    location_present=location_present,
    latitude=latitude,
    longitude=longitude,
    accuracy_m=accuracy_m,
    bearing_deg=bearing_deg,
    speed_kph=speed_kph,
    location_observed_at_ms=location_observed_at_ms,
    current_step_index=_optional_integer(location, "currentStepIndex", 0, 1_000_000),
    current_link_index=_optional_integer(location, "currentLinkIndex", 0, 1_000_000),
    current_point_index=_optional_integer(location, "currentPointIndex", 0, 10_000_000),
    guidance_present=guidance_present,
    maneuver=maneuver,
    guidance_observed_at_ms=guidance_observed_at_ms,
    maneuver_distance_m=float(_optional_integer(guidance, "maneuverDistanceM", 0, 100_000, default=0)),
    next_maneuver=_enum(guidance, "nextManeuver", MANEUVER_VALUES, default="none"),
    next_maneuver_distance_m=float(_optional_integer(guidance, "nextManeuverDistanceM", 0, 100_000, default=0)),
    advisory_speed_mps=advisory_speed,
    road_class=_optional_integer(guidance, "roadClass", 0, 10),
    road_type=_documented_road_type(guidance),
    current_road=_optional_road_name(guidance, "currentRoad"),
    next_road=_optional_road_name(guidance, "nextRoad"),
    parallel_road_status=_enum(guidance, "parallelRoadStatus", ROAD_LAYER_VALUES, default="unknown"),
    elevated_road_status=_enum(guidance, "elevatedRoadStatus", ROAD_LAYER_VALUES, default="unknown"),
    route_notice_type=_enum(guidance, "routeNoticeType", ROUTE_NOTICE_VALUES, default="none"),
    route_notice_distance_m=float(_optional_integer(guidance, "routeNoticeDistanceM", 0, 100_000, default=0)),
    route_notice_observed_at_ms=_optional_integer(guidance, "routeNoticeObservedAtMs", 0, 2**63 - 1, default=0),
    lane_guidance_present=lane_guidance_present,
    lane_guidance_observed_at_ms=lane_observed_at_ms,
    lanes=tuple(lanes),
  )


class NavAssistStore:
  """Thread-safe freshness and replay boundary for one active phone session."""

  def __init__(self, *, clock_ns: Callable[[], int] = time.monotonic_ns,
               wall_clock_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000,
               remembered_sessions: int = 32, checkpoint_path: str | Path | None = None):
    if remembered_sessions <= 0:
      raise ValueError("remembered_sessions must be positive")
    self._clock_ns = clock_ns
    self._wall_clock_ms = wall_clock_ms
    self._checkpoint_path = Path(checkpoint_path) if checkpoint_path is not None else None
    self._lock = threading.Lock()
    self._current: AcceptedSnapshot | None = None
    self._retired_sessions: deque[str] = deque(maxlen=remembered_sessions)
    self._app_key_id: str | None = None
    self._active_session_id: str | None = None
    self._last_sequence = 0
    self._last_route_revision = 0
    self._load_checkpoint()

  @staticmethod
  def _valid_checkpoint_session(value: Any) -> bool:
    return (isinstance(value, str) and 0 < len(value) <= MAX_SESSION_ID_LENGTH
            and SESSION_ID_PATTERN.fullmatch(value) is not None)

  @staticmethod
  def _valid_checkpoint_counter(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= 2**63 - 1

  def _load_checkpoint(self) -> None:
    path = self._checkpoint_path
    if path is None or not path.exists():
      return
    try:
      raw = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_keys)
      legacy_keys = {"version", "tokenId", "activeSessionId", "sequence", "routeRevision", "retiredSessions"}
      if isinstance(raw, dict) and set(raw) == legacy_keys and raw.get("version") == 1:
        return
      expected_keys = {"version", "appKeyId", "activeSessionId", "sequence", "routeRevision", "retiredSessions"}
      if not isinstance(raw, dict) or set(raw) != expected_keys or raw["version"] != CHECKPOINT_VERSION:
        raise ValueError("unsupported replay checkpoint")
      app_key_id = raw["appKeyId"]
      active = raw["activeSessionId"]
      retired = raw["retiredSessions"]
      if (not isinstance(app_key_id, str) or APP_KEY_ID_PATTERN.fullmatch(app_key_id) is None
          or not self._valid_checkpoint_session(active) or not self._valid_checkpoint_counter(raw["sequence"])
          or not self._valid_checkpoint_counter(raw["routeRevision"]) or not isinstance(retired, list)
          or len(retired) > self._retired_sessions.maxlen
          or any(not self._valid_checkpoint_session(session) for session in retired)
          or len(set(retired)) != len(retired) or active in retired):
        raise ValueError("invalid replay checkpoint")
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, NavAssistProtocolError, ValueError) as error:
      raise RuntimeError("NavAssist replay checkpoint is unreadable") from error
    self._app_key_id = app_key_id
    self._active_session_id = active
    self._last_sequence = raw["sequence"]
    self._last_route_revision = raw["routeRevision"]
    self._retired_sessions.extend(retired)

  def _save_checkpoint(self, app_key_id: str, active_session_id: str, sequence: int, route_revision: int,
                       retired_sessions: deque[str]) -> None:
    path = self._checkpoint_path
    if path is None:
      return
    payload = json.dumps({
      "version": CHECKPOINT_VERSION,
      "appKeyId": app_key_id,
      "activeSessionId": active_session_id,
      "sequence": sequence,
      "routeRevision": route_revision,
      "retiredSessions": list(retired_sessions),
    }, separators=(",", ":"), sort_keys=True).encode()
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
      fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
      with os.fdopen(fd, "wb") as checkpoint:
        checkpoint.write(payload)
        checkpoint.flush()
        os.fsync(checkpoint.fileno())
      os.replace(temporary, path)
    except OSError as error:
      try:
        temporary.unlink(missing_ok=True)
      except OSError:
        pass
      _reject("replay", f"replay checkpoint unavailable: {type(error).__name__}")

  def accept(self, body: bytes, app_key_id: str) -> AcceptedSnapshot:
    if not isinstance(app_key_id, str) or APP_KEY_ID_PATTERN.fullmatch(app_key_id) is None:
      _reject("authentication", "missing or malformed app key id")
    snapshot = parse_snapshot(body)
    source_age_ms = self._wall_clock_ms() - snapshot.source_wall_time_ms
    maximum_age_ms = min(MAX_SOURCE_AGE_MS, snapshot.valid_for_ms + SOURCE_DELIVERY_GRACE_MS)
    if source_age_ms < -MAX_SOURCE_FUTURE_SKEW_MS or source_age_ms > maximum_age_ms:
      _reject("replay", "sourceWallTimeMs is outside the receiver freshness window")
    now_ns = self._clock_ns()
    accepted = AcceptedSnapshot(snapshot, now_ns, now_ns + snapshot.valid_for_ms * 1_000_000)
    with self._lock:
      if self._app_key_id is not None and self._app_key_id != app_key_id:
        if self._current is not None and not self._current.is_stale(now_ns) and self._current.snapshot.route_active:
          _reject("replay", "another paired app owns the active navigation session")
        self._active_session_id = None
        self._last_sequence = 0
        self._last_route_revision = 0
        self._retired_sessions.clear()
      retired_sessions = deque(self._retired_sessions, maxlen=self._retired_sessions.maxlen)
      if self._active_session_id is not None:
        if snapshot.session_id == self._active_session_id:
          if snapshot.sequence <= self._last_sequence:
            _reject("replay", "sequence must increase within a session")
          if snapshot.route_revision < self._last_route_revision:
            _reject("replay", "routeRevision must not decrease within a session")
        else:
          if snapshot.session_id in retired_sessions:
            _reject("replay", "retired session cannot become active again")
          retired_sessions.append(self._active_session_id)
      self._save_checkpoint(app_key_id, snapshot.session_id, snapshot.sequence, snapshot.route_revision, retired_sessions)
      self._app_key_id = app_key_id
      self._active_session_id = snapshot.session_id
      self._last_sequence = snapshot.sequence
      self._last_route_revision = snapshot.route_revision
      self._retired_sessions = retired_sessions
      self._current = accepted
    return accepted

  def reset(self) -> None:
    with self._lock:
      self._current = None
      self._app_key_id = None
      self._active_session_id = None
      self._last_sequence = 0
      self._last_route_revision = 0
      self._retired_sessions.clear()
      if self._checkpoint_path is not None:
        try:
          self._checkpoint_path.unlink(missing_ok=True)
        except OSError as error:
          raise RuntimeError("NavAssist replay checkpoint could not be reset") from error

  def current(self, now_ns: int | None = None) -> AcceptedSnapshot | None:
    with self._lock:
      return self._current

  def is_stale(self, now_ns: int | None = None) -> bool:
    current = self.current()
    if current is None:
      return True
    return current.is_stale(self._clock_ns() if now_ns is None else now_ns)
