"""Data adapter for comma's nav model; this module grants no control authority.

Command indices and distance scaling follow commaai/openpilot nav-model-integration
a03bb1d4, selfdrive/navigation/helpers.py. GCJ-02 remains explicitly labelled:
these route points must not be rendered against a WGS-84 basemap unchanged.
"""
from dataclasses import dataclass
import json

from openpilot.sunnypilot.navassist.protocol import (
  AcceptedSnapshot, NavAssistProtocolError, _reject_duplicate_keys, _integer, _number, _string,
)

ROUTE_PATH = "/v3/route"
MAX_ROUTE_BYTES = 2 * 1024 * 1024
MAX_ROUTE_POINTS = 20_000
# Unknown/merge/arrival intentionally have no inferred instruction.
COMMANDS = {"straight": 8, "slight_right": 9, "turn_right": 10, "sharp_right": 11,
            "u_turn_left": 13, "u_turn_right": 13, "sharp_left": 14, "turn_left": 15,
            "slight_left": 16, "ramp_right": 18, "ramp_left": 19, "exit_right": 20,
            "exit_left": 21, "keep_right": 23, "keep_left": 24, "roundabout": 26}


@dataclass(frozen=True)
class ModelRoute:
  session_id: str
  sequence: int
  route_revision: int
  source_wall_time_ms: int
  coordinate_system: str
  points: tuple[tuple[float, float], ...]  # latitude, longitude; original SDK geometry


def parse_model_route(body: bytes) -> ModelRoute:
  if not 0 < len(body) <= MAX_ROUTE_BYTES:
    raise NavAssistProtocolError("malformed", "route body size")
  try:
    raw = json.loads(body, object_pairs_hook=_reject_duplicate_keys)
  except (ValueError, UnicodeDecodeError) as error:
    raise NavAssistProtocolError("malformed", "invalid route JSON") from error
  keys = {"schemaVersion", "messageType", "sessionId", "sequence", "routeRevision",
          "sourceWallTimeMs", "coordinateSystem", "points"}
  if (not isinstance(raw, dict) or set(raw) != keys or type(raw["schemaVersion"]) is not int
      or raw["schemaVersion"] != 3 or raw["messageType"] != "navigation_route"):
    raise NavAssistProtocolError("malformed", "unsupported route envelope")
  from openpilot.sunnypilot.navassist.protocol import SESSION_ID_PATTERN
  session = _string(raw, "sessionId", max_length=64)
  if not session or SESSION_ID_PATTERN.fullmatch(session) is None or raw["coordinateSystem"] not in ("gcj02", "wgs84"):
    raise NavAssistProtocolError("malformed", "route identity or coordinates")
  points = raw["points"]
  if not isinstance(points, list) or not 2 <= len(points) <= MAX_ROUTE_POINTS:
    raise NavAssistProtocolError("malformed", "route point count")
  parsed = []
  for point in points:
    if not isinstance(point, dict) or set(point) != {"latitude", "longitude"}:
      raise NavAssistProtocolError("malformed", "route point shape")
    try:
      parsed.append((_number(point, "latitude", -90, 90), _number(point, "longitude", -180, 180)))
    except OverflowError as error:
      raise NavAssistProtocolError("malformed", "route coordinate overflow") from error
  return ModelRoute(session, _integer(raw, "sequence", 1, 2**63 - 1),
                    _integer(raw, "routeRevision", 0, 2**63 - 1),
                    _integer(raw, "sourceWallTimeMs", 0, 2**63 - 1), raw["coordinateSystem"], tuple(parsed))


def matches(route: ModelRoute, current: AcceptedSnapshot, now_ns: int) -> bool:
  s = current.snapshot
  return (not current.is_stale(now_ns) and s.route_active and s.route_matched and not s.gps_weak
          and s.navigation_mode == "realtime" and s.location_present and s.guidance_present
          and (route.session_id, route.route_revision, route.coordinate_system) ==
              (s.session_id, s.route_revision, s.coordinate_system))


@dataclass(frozen=True)
class ModelNavigationInput:
  route: ModelRoute
  latitude: float
  longitude: float
  bearing_deg: float
  navigation: tuple[float, ...]  # 46 command slots plus distance / 100 metres
  sequence: int
  location_age_ms: float
  speed_mps: float


def prepare_model_input(route: ModelRoute, current: AcceptedSnapshot, now_ns: int) -> ModelNavigationInput | None:
  if not matches(route, current, now_ns):
    return None
  s = current.snapshot
  elapsed_ms = current.age_ms(now_ns)
  if any(not 0 <= s.source_wall_time_ms - observed + elapsed_ms <= s.valid_for_ms
         for observed in (s.location_observed_at_ms, s.guidance_observed_at_ms)):
    return None
  vector = [0.0] * 47
  command = COMMANDS.get(s.maneuver)
  if command is not None and s.maneuver_distance_present:
    vector[command], vector[-1] = 1.0, s.maneuver_distance_m / 100.0
  return ModelNavigationInput(route, s.latitude, s.longitude, s.bearing_deg, tuple(vector), s.sequence,
                              s.source_wall_time_ms - s.location_observed_at_ms + elapsed_ms, s.speed_kph / 3.6)
