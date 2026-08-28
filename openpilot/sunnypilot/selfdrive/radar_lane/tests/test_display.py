from types import SimpleNamespace as namespace

from openpilot.sunnypilot.selfdrive.radar_lane.display import (
  format_target_label,
  rendered_radar_track_ids,
  select_lane_display_targets,
  should_render_second_lead,
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
    leadTwo=namespace(present=True, radar=True, radarTrackId=8),
  )

  assert rendered_radar_track_ids(radar_state) == frozenset({7, 8})
  assert rendered_radar_track_ids(radar_state, include_lead_two=False) == frozenset({7})
  assert rendered_radar_track_ids(None) == frozenset()


def _lead(*, present=True, radar=True, track_id=-1, d_rel=20.0, y_rel=0.0):
  return namespace(present=present, radar=radar, radarTrackId=track_id, dRel=d_rel, yRel=y_rel)


def test_second_lead_with_same_radar_track_is_always_suppressed():
  assert not should_render_second_lead(_lead(track_id=4), _lead(track_id=4, d_rel=30.0), True)


def test_second_lead_uses_distance_hysteresis_for_unstable_association():
  lead_one = _lead(radar=False, d_rel=20.0)
  assert not should_render_second_lead(lead_one, _lead(radar=False, d_rel=24.0), False)
  assert should_render_second_lead(lead_one, _lead(radar=False, d_rel=24.0), True)
  assert should_render_second_lead(lead_one, _lead(radar=False, d_rel=26.0), False)
  assert not should_render_second_lead(lead_one, _lead(radar=False, d_rel=22.0), True)


def test_laterally_separate_second_lead_is_preserved():
  assert should_render_second_lead(_lead(d_rel=20.0, y_rel=-1.0), _lead(d_rel=20.0, y_rel=1.0), False)
