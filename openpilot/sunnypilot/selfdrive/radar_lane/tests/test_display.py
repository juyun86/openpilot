from types import SimpleNamespace as namespace

from openpilot.sunnypilot.selfdrive.radar_lane.display import (
  format_target_label,
  rendered_radar_track_ids,
  select_lane_display_targets,
)


def _target(track_id, lane_mask, d_rel, *, cut_in=False, crossing=-1.0):
  return namespace(
    present=True,
    trackId=track_id,
    laneMask=lane_mask,
    dRel=d_rel,
    cutInCandidate=cut_in,
    timeToLaneCross=crossing,
  )


def test_display_selects_one_unique_target_per_lane_and_prioritizes_cut_in():
  targets = [
    _target(1, 1, 8.0),
    _target(2, 1, 30.0, cut_in=True, crossing=0.8),
    _target(3, 2, 12.0),
    _target(4, 4, 10.0),
    _target(5, 4, 20.0, cut_in=True, crossing=1.2),
  ]

  assert [target.trackId for target in select_lane_display_targets(targets)] == [2, 3, 5]


def test_boundary_target_is_not_drawn_twice():
  boundary = _target(7, 3, 15.0, cut_in=True, crossing=0.0)
  center = _target(8, 2, 20.0)

  selected = select_lane_display_targets([boundary, center])

  assert [target.trackId for target in selected] == [7, 8]


def test_target_label_reports_distance_and_estimated_absolute_speed():
  assert format_target_label(30.0, -2.0, 20.0, True) == "30m  65km/h"
  assert format_target_label(30.0, -2.0, 20.0, False) == "98ft  40mph"


def test_existing_radar_chevrons_are_not_drawn_twice():
  radar_state = namespace(
    leadOne=namespace(present=True, radar=True, radarTrackId=7),
    leadTwo=namespace(present=True, radar=False, radarTrackId=8),
  )

  assert rendered_radar_track_ids(radar_state) == frozenset({7})
  assert rendered_radar_track_ids(None) == frozenset()
