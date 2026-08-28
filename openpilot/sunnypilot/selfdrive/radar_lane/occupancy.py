from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Iterable, Sequence


MIN_FORWARD_DISTANCE_M = 0.0
MAX_FORWARD_DISTANCE_M = 120.0
DEFAULT_RADAR_TO_CAMERA_M = 1.52
MIN_LANE_LINE_PROBABILITY = 0.5
NOMINAL_LANE_WIDTH_M = 3.6
MIN_PLAUSIBLE_LANE_WIDTH_M = 2.2
MAX_PLAUSIBLE_LANE_WIDTH_M = 5.5
MIN_LANE_LINE_RANGE_M = 10.0
LANE_LINE_START_TOLERANCE_M = 0.05
LANE_BOUNDARY_AMBIGUITY_M = 0.30

LANE_LEFT_MASK = 1
LANE_CENTER_MASK = 2
LANE_RIGHT_MASK = 4
LANE_MASKS = {"left": LANE_LEFT_MASK, "center": LANE_CENTER_MASK, "right": LANE_RIGHT_MASK}


class Occupancy(str, Enum):
  unknown = "unknown"
  clear = "clear"
  occupied = "occupied"


class GeometrySource(str, Enum):
  none = "none"
  lane_lines = "laneLines"
  model_path_estimate = "modelPathEstimate"


@dataclass(frozen=True)
class RadarTarget:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  measured: bool = True
  yv_rel: float = 0.0
  yv_rel_valid: bool = False


@dataclass(frozen=True)
class ClassifiedTarget(RadarTarget):
  d_path: float = 0.0
  ambiguous: bool = False
  lane_mask: int = 0
  center_boundary_distance: float = -1.0


@dataclass(frozen=True)
class LaneResult:
  occupancy: Occupancy
  geometry_source: GeometrySource
  geometry_confidence: float
  evaluated_distance: float
  targets: tuple[ClassifiedTarget, ...] = ()


@dataclass(frozen=True)
class RadarLaneResult:
  valid: bool
  left: LaneResult
  center: LaneResult
  right: LaneResult


@dataclass(frozen=True)
class _Curve:
  xs: tuple[float, ...]
  ys: tuple[float, ...]

  @classmethod
  def from_values(cls, xs: Sequence[Any], ys: Sequence[Any]) -> _Curve | None:
    if len(xs) < 2 or len(xs) != len(ys):
      return None
    try:
      x_values = tuple(float(value) for value in xs)
      y_values = tuple(float(value) for value in ys)
    except (TypeError, ValueError):
      return None
    if not all(math.isfinite(value) for value in (*x_values, *y_values)):
      return None
    if any(right <= left for left, right in zip(x_values, x_values[1:], strict=False)):
      return None
    return cls(x_values, y_values)

  @property
  def min_x(self) -> float:
    return self.xs[0]

  @property
  def max_x(self) -> float:
    return self.xs[-1]

  def sample(self, x: float) -> float | None:
    if not math.isfinite(x) or x < self.min_x or x > self.max_x:
      return None
    index = bisect.bisect_right(self.xs, x)
    if index == len(self.xs):
      return self.ys[-1]
    if index == 0:
      return self.ys[0]
    x0, x1 = self.xs[index - 1], self.xs[index]
    y0, y1 = self.ys[index - 1], self.ys[index]
    ratio = (x - x0) / (x1 - x0)
    return y0 + ratio * (y1 - y0)


@dataclass(frozen=True)
class _Projection:
  center_x: float
  center_y: float
  d_path: float


@dataclass(frozen=True)
class _PathGeometry:
  points: tuple[tuple[float, float], ...]

  @classmethod
  def from_model(cls, position: Any) -> _PathGeometry | None:
    curve = _Curve.from_values(getattr(position, "x", ()), getattr(position, "y", ()))
    if curve is None:
      return None
    # Model lateral coordinates are right-positive. Radar yRel is left-positive.
    return cls(tuple((x, -y) for x, y in zip(curve.xs, curve.ys, strict=True)))

  @property
  def min_x(self) -> float:
    return self.points[0][0]

  @property
  def max_x(self) -> float:
    return self.points[-1][0]

  def project(self, x: float, y: float) -> _Projection | None:
    if not (math.isfinite(x) and math.isfinite(y)) or x < self.min_x or x > self.max_x:
      return None

    best: _Projection | None = None
    best_distance_sq = math.inf
    for (x0, y0), (x1, y1) in zip(self.points, self.points[1:], strict=False):
      dx, dy = x1 - x0, y1 - y0
      length = math.hypot(dx, dy)
      if length < 1e-6:
        continue
      tangent_x, tangent_y = dx / length, dy / length
      ratio = min(1.0, max(0.0, ((x - x0) * dx + (y - y0) * dy) / (length * length)))
      center_x, center_y = x0 + ratio * dx, y0 + ratio * dy
      offset_x, offset_y = x - center_x, y - center_y
      distance_sq = offset_x * offset_x + offset_y * offset_y
      if distance_sq < best_distance_sq:
        best_distance_sq = distance_sq
        best = _Projection(center_x, center_y, -tangent_y * offset_x + tangent_x * offset_y)
    return best


@dataclass(frozen=True)
class _LaneGeometry:
  name: str
  source: GeometrySource
  confidence: float
  evaluated_distance: float
  first_boundary: _Curve | None = None
  second_boundary: _Curve | None = None

  def bounds(self, projection: _Projection, sample_x: float) -> tuple[float, float] | None:
    if self.source == GeometrySource.lane_lines:
      assert self.first_boundary is not None and self.second_boundary is not None
      first = self.first_boundary.sample(sample_x)
      second = self.second_boundary.sample(sample_x)
      if first is None or second is None:
        return None
      # Convert model right-positive lane-line coordinates to radar left-positive.
      return min(-first, -second), max(-first, -second)

    if self.source == GeometrySource.model_path_estimate:
      lane_center = {
        "left": NOMINAL_LANE_WIDTH_M,
        "center": 0.0,
        "right": -NOMINAL_LANE_WIDTH_M,
      }[self.name]
      half_width = NOMINAL_LANE_WIDTH_M / 2.0
      return lane_center - half_width, lane_center + half_width
    return None

  def lateral_value(self, target: RadarTarget, projection: _Projection) -> float:
    return target.y_rel if self.source == GeometrySource.lane_lines else projection.d_path


def _unknown_lane() -> LaneResult:
  return LaneResult(Occupancy.unknown, GeometrySource.none, 0.0, 0.0)


def empty_result() -> RadarLaneResult:
  unknown = _unknown_lane()
  return RadarLaneResult(False, unknown, unknown, unknown)


def _curve_from_lane_line(line: Any) -> _Curve | None:
  return _Curve.from_values(getattr(line, "x", ()), getattr(line, "y", ()))


def _validated_lane_line_geometry(name: str, first: _Curve | None, second: _Curve | None,
                                  confidence: float, path: _PathGeometry,
                                  radar_to_camera: float) -> _LaneGeometry | None:
  if (first is None or second is None or not math.isfinite(confidence) or
      not 0.0 <= confidence <= 1.0 or confidence < MIN_LANE_LINE_PROBABILITY):
    return None
  radar_start_x = radar_to_camera + MIN_FORWARD_DISTANCE_M
  data_start_x = max(first.min_x, second.min_x, path.min_x)
  # Without per-lane minimum coverage in the output contract, a gap near the
  # car would make a later "clear" result falsely describe the entire range.
  if data_start_x > radar_start_x + LANE_LINE_START_TOLERANCE_M:
    return None
  start_x = max(data_start_x, radar_start_x)
  end_x = min(first.max_x, second.max_x, path.max_x, radar_to_camera + MAX_FORWARD_DISTANCE_M)
  if end_x <= start_x:
    return None

  # Keep the longest trustworthy prefix. Far lane lines often cross as confidence decays.
  valid_end = start_x
  sample_count = max(2, int((end_x - start_x) / 5.0) + 1)
  for index in range(sample_count + 1):
    x = start_x + (end_x - start_x) * index / sample_count
    first_y, second_y = first.sample(x), second.sample(x)
    if first_y is None or second_y is None:
      break
    # modelV2 laneLines are ordered from left to right in model coordinates.
    # A signed width also rejects crossed/reversed boundaries.
    width = second_y - first_y
    if not MIN_PLAUSIBLE_LANE_WIDTH_M <= width <= MAX_PLAUSIBLE_LANE_WIDTH_M:
      break
    valid_end = x

  evaluated_distance = max(0.0, valid_end - radar_to_camera)
  if evaluated_distance < MIN_LANE_LINE_RANGE_M:
    return None
  return _LaneGeometry(name, GeometrySource.lane_lines, confidence, evaluated_distance, first, second)


def _lane_geometries(model: Any, path: _PathGeometry, radar_to_camera: float) -> dict[str, _LaneGeometry]:
  lane_lines = tuple(getattr(model, "laneLines", ()))
  try:
    lane_probs = tuple(float(value) for value in getattr(model, "laneLineProbs", ()))
  except (TypeError, ValueError):
    lane_probs = ()
  curves = tuple(_curve_from_lane_line(line) for line in lane_lines[:4])
  path_evaluated_distance = max(0.0, min(MAX_FORWARD_DISTANCE_M, path.max_x - radar_to_camera))

  geometries: dict[str, _LaneGeometry] = {}
  for name, first_index, second_index in (("left", 0, 1), ("center", 1, 2), ("right", 2, 3)):
    geometry = None
    if len(curves) == 4 and len(lane_probs) >= 4:
      geometry = _validated_lane_line_geometry(
        name, curves[first_index], curves[second_index], min(lane_probs[first_index], lane_probs[second_index]),
        path, radar_to_camera,
      )
    geometries[name] = geometry or _LaneGeometry(
      name, GeometrySource.model_path_estimate, 0.25, path_evaluated_distance,
    )
  return geometries


def radar_target_from_point(point: Any) -> RadarTarget | None:
  try:
    track_id = int(point.trackId)
    d_rel, y_rel, v_rel = float(point.dRel), float(point.yRel), float(point.vRel)
  except (AttributeError, TypeError, ValueError):
    return None
  if not all(math.isfinite(value) for value in (d_rel, y_rel, v_rel)):
    return None

  measured = True
  yv_rel = 0.0
  yv_rel_valid = False
  deprecated = getattr(point, "deprecated", None)
  if deprecated is not None:
    try:
      measured = bool(deprecated.measured)
    except (AttributeError, TypeError):
      pass
    try:
      yv_rel = float(deprecated.yvRel)
      yv_rel_valid = math.isfinite(yv_rel)
    except (AttributeError, TypeError, ValueError):
      pass
  return RadarTarget(track_id, d_rel, y_rel, v_rel, measured, yv_rel if yv_rel_valid else 0.0, yv_rel_valid)


def classify_radar_lanes(model: Any, targets: Iterable[RadarTarget],
                         radar_to_camera: float) -> RadarLaneResult:
  path = _PathGeometry.from_model(getattr(model, "position", None))
  if path is None:
    return empty_result()
  geometries = _lane_geometries(model, path, radar_to_camera)
  classified: dict[str, list[ClassifiedTarget]] = {"left": [], "center": [], "right": []}

  for target in targets:
    if not all(math.isfinite(value) for value in (target.d_rel, target.y_rel, target.v_rel)):
      continue
    if not MIN_FORWARD_DISTANCE_M < target.d_rel <= MAX_FORWARD_DISTANCE_M:
      continue
    projection = path.project(target.d_rel + radar_to_camera, target.y_rel)
    if projection is None:
      continue

    matches: dict[str, tuple[float, float, float, float]] = {}
    for name, geometry in geometries.items():
      if target.d_rel > geometry.evaluated_distance:
        continue
      bounds = geometry.bounds(projection, target.d_rel + radar_to_camera)
      if bounds is None:
        continue
      value = geometry.lateral_value(target, projection)
      lower, upper = bounds
      if lower - LANE_BOUNDARY_AMBIGUITY_M <= value <= upper + LANE_BOUNDARY_AMBIGUITY_M:
        boundary_distance = min(abs(value - lower), abs(value - upper))
        matches[name] = (boundary_distance, value, lower, upper)

    ambiguous = len(matches) > 1 or any(
      distance <= LANE_BOUNDARY_AMBIGUITY_M for distance, _, _, _ in matches.values()
    )
    lane_mask = sum(LANE_MASKS[name] for name in matches)
    center_boundary_distances = []
    if "left" in matches:
      _, value, lower, _ = matches["left"]
      center_boundary_distances.append(max(0.0, value - lower))
    if "right" in matches:
      _, value, _, upper = matches["right"]
      center_boundary_distances.append(max(0.0, upper - value))
    center_boundary_distance = min(center_boundary_distances, default=-1.0)
    if lane_mask & LANE_CENTER_MASK and lane_mask & (LANE_LEFT_MASK | LANE_RIGHT_MASK):
      center_boundary_distance = 0.0

    classified_target = ClassifiedTarget(
      track_id=target.track_id,
      d_rel=target.d_rel,
      y_rel=target.y_rel,
      v_rel=target.v_rel,
      measured=target.measured,
      yv_rel=target.yv_rel,
      yv_rel_valid=target.yv_rel_valid,
      d_path=projection.d_path,
      ambiguous=ambiguous,
      lane_mask=lane_mask,
      center_boundary_distance=center_boundary_distance,
    )
    for name in matches:
      classified[name].append(classified_target)

  lane_results = {}
  for name, geometry in geometries.items():
    lane_targets = tuple(sorted(classified[name], key=lambda target: (target.d_rel, target.track_id)))
    if lane_targets:
      occupancy = Occupancy.occupied
    elif geometry.source == GeometrySource.lane_lines:
      occupancy = Occupancy.clear
    else:
      # A nominal path corridor can provide positive evidence, but cannot prove
      # that an adjacent lane exists or is clear.
      occupancy = Occupancy.unknown
    lane_results[name] = LaneResult(
      occupancy, geometry.source, geometry.confidence, geometry.evaluated_distance, lane_targets,
    )

  return RadarLaneResult(True, lane_results["left"], lane_results["center"], lane_results["right"])
