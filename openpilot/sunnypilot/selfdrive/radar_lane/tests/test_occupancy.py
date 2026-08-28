import math
from types import SimpleNamespace as namespace

from openpilot.sunnypilot.selfdrive.radar_lane.occupancy import (
  GeometrySource,
  Occupancy,
  RadarTarget,
  classify_radar_lanes,
  radar_target_from_point,
)


RADAR_TO_CAMERA_M = 1.52
XS = [0.0, 40.0, 80.0, 130.0]


def _line(y_values):
  if isinstance(y_values, (float, int)):
    y_values = [float(y_values)] * len(XS)
  return namespace(x=XS, y=y_values)


def _model(path_y=0.0, lane_offsets=(-5.4, -1.8, 1.8, 5.4), probs=(1.0, 1.0, 1.0, 1.0)):
  path_values = [float(path_y)] * len(XS) if isinstance(path_y, (float, int)) else path_y
  lane_lines = []
  for offset in lane_offsets:
    if isinstance(offset, (float, int)):
      lane_lines.append(_line([value + float(offset) for value in path_values]))
    else:
      lane_lines.append(_line(offset))
  return namespace(
    position=namespace(x=XS, y=path_values),
    laneLines=lane_lines,
    laneLineProbs=probs,
  )


def _classify(model, *targets):
  return classify_radar_lanes(model, targets, RADAR_TO_CAMERA_M)


def test_straight_road_places_targets_in_left_center_and_right_lanes():
  result = _classify(
    _model(),
    RadarTarget(1, 30.0, 3.6, -1.0),
    RadarTarget(2, 25.0, 0.0, 0.0),
    RadarTarget(3, 35.0, -3.6, 1.0),
  )

  assert result.valid
  assert [result.left.targets[0].track_id, result.center.targets[0].track_id, result.right.targets[0].track_id] == [1, 2, 3]
  assert all(lane.occupancy == Occupancy.occupied for lane in (result.left, result.center, result.right))


def test_curved_path_uses_local_lane_geometry_instead_of_ego_axis():
  result = _classify(
    _model(path_y=[0.0, 2.0, 8.0, 18.0]),
    RadarTarget(10, 38.48, -2.0, 0.0),  # model x=40 m, on the path center
    RadarTarget(11, 38.48, 1.6, 0.0),   # one lane left at the same x
  )

  assert result.center.targets[0].track_id == 10
  assert math.isclose(result.center.targets[0].d_path, 0.0, abs_tol=1e-6)
  assert result.left.targets[0].track_id == 11


def test_lane_lines_are_sampled_at_target_x_not_path_projection_x():
  result = _classify(
    _model(path_y=[0.0, 20.0, 40.0, 65.0]),
    RadarTarget(12, 38.48, -17.8, 0.0),
  )

  assert result.left.targets[0].track_id == 12
  assert not result.center.targets
  assert result.center.occupancy == Occupancy.clear


def test_real_lane_width_takes_priority_over_nominal_width():
  result = _classify(_model(lane_offsets=(-6.0, -2.0, 2.0, 6.0)), RadarTarget(7, 50.0, 4.0, 0.0))

  assert result.left.geometry_source == GeometrySource.lane_lines
  assert result.left.targets[0].track_id == 7


def test_missing_outer_lines_can_detect_but_cannot_prove_adjacent_lanes_clear():
  model = _model(probs=(0.1, 1.0, 1.0, 0.1))
  empty = _classify(model)

  assert empty.center.occupancy == Occupancy.clear
  assert empty.center.geometry_source == GeometrySource.lane_lines
  assert empty.left.occupancy == Occupancy.unknown
  assert empty.right.occupancy == Occupancy.unknown

  occupied = _classify(model, RadarTarget(8, 20.0, 3.6, 0.0))
  assert occupied.left.occupancy == Occupancy.occupied
  assert occupied.left.geometry_source == GeometrySource.model_path_estimate


def test_reversed_lane_boundaries_fall_back_to_unknown():
  result = _classify(_model(lane_offsets=(5.4, -1.8, 1.8, 5.4)))

  assert result.left.geometry_source == GeometrySource.model_path_estimate
  assert result.left.occupancy == Occupancy.unknown


def test_lane_lines_starting_far_ahead_cannot_claim_near_range_clear():
  late_xs = [20.0, 50.0, 90.0, 130.0]

  def late_line(y):
    return namespace(x=late_xs, y=[y] * len(late_xs))

  model = namespace(
    position=_line(0.0),
    laneLines=[late_line(-5.4), late_line(-1.8), late_line(1.8), late_line(5.4)],
    laneLineProbs=[1.0, 1.0, 1.0, 1.0],
  )
  empty = _classify(model)
  occupied = _classify(model, RadarTarget(13, 5.0, 0.0, 0.0))

  assert empty.center.occupancy == Occupancy.unknown
  assert empty.center.geometry_source == GeometrySource.model_path_estimate
  assert occupied.center.occupancy == Occupancy.occupied
  assert occupied.center.targets[0].track_id == 13


def test_non_finite_lane_probability_cannot_produce_clear():
  result = _classify(_model(probs=(float("nan"),) * 4))

  assert all(lane.occupancy == Occupancy.unknown for lane in (result.left, result.center, result.right))
  assert all(lane.geometry_source == GeometrySource.model_path_estimate for lane in (result.left, result.center, result.right))


def test_boundary_target_conservatively_occupies_both_adjacent_lanes():
  result = _classify(_model(), RadarTarget(9, 30.0, 1.8, 0.0))

  assert result.left.targets[0].track_id == 9
  assert result.center.targets[0].track_id == 9
  assert result.left.targets[0].ambiguous
  assert result.center.targets[0].ambiguous


def test_closest_target_is_first_and_track_zero_is_valid():
  result = _classify(
    _model(),
    RadarTarget(4, 40.0, 0.0, 0.0),
    RadarTarget(0, 12.0, 0.0, -1.0),
    RadarTarget(2, 25.0, 0.0, 1.0),
  )

  assert [target.track_id for target in result.center.targets] == [0, 2, 4]


def test_invalid_and_out_of_scope_targets_are_ignored():
  result = _classify(
    _model(),
    RadarTarget(1, 20.0, float("nan"), 0.0),
    RadarTarget(2, 20.0, 6.0, 0.0),
    RadarTarget(3, -1.0, 0.0, 0.0),
    RadarTarget(4, 121.0, 0.0, 0.0),
  )

  assert all(not lane.targets for lane in (result.left, result.center, result.right))
  assert all(lane.occupancy == Occupancy.clear for lane in (result.left, result.center, result.right))


def test_missing_model_path_returns_unknown_not_clear():
  model = namespace(position=namespace(x=[], y=[]), laneLines=[], laneLineProbs=[])
  result = _classify(model, RadarTarget(1, 20.0, 0.0, 0.0))

  assert not result.valid
  assert all(lane.occupancy == Occupancy.unknown for lane in (result.left, result.center, result.right))


def test_radar_point_adapter_preserves_unmeasured_track_and_id_zero():
  point = namespace(
    trackId=0,
    dRel=8.0,
    yRel=1.0,
    vRel=-0.5,
    deprecated=namespace(measured=False),
  )
  target = radar_target_from_point(point)

  assert target is not None
  assert target.track_id == 0
  assert not target.measured


if __name__ == "__main__":
  tests = [value for name, value in globals().items() if name.startswith("test_") and callable(value)]
  for test in tests:
    test()
  print(f"{len(tests)} radar lane occupancy tests passed")
