from __future__ import annotations

import math
from typing import Any, Iterable

from openpilot.sunnypilot.selfdrive.radar_lane.occupancy import (
  LANE_CENTER_MASK,
  LANE_LEFT_MASK,
  LANE_RIGHT_MASK,
)


DISPLAY_LANE_ORDER = (LANE_LEFT_MASK, LANE_CENTER_MASK, LANE_RIGHT_MASK)
M_TO_FT = 3.28084
MS_TO_KPH = 3.6
MS_TO_MPH = 2.2369363


def _target_priority(target: Any) -> tuple[bool, float, float, int]:
  cut_in = bool(getattr(target, "cutInCandidate", False))
  crossing_time = float(getattr(target, "timeToLaneCross", -1.0))
  if not cut_in or not math.isfinite(crossing_time) or crossing_time < 0.0:
    crossing_time = math.inf
  return not cut_in, crossing_time, float(target.dRel), int(target.trackId)


def select_lane_display_targets(targets: Iterable[Any]) -> tuple[Any, ...]:
  """Select at most one unique, risk-prioritized target for each displayed lane."""
  candidates = tuple(target for target in targets if bool(getattr(target, "present", False)))
  selected = []
  selected_ids: set[int] = set()
  for lane_mask in DISPLAY_LANE_ORDER:
    lane_targets = [
      target for target in candidates
      if int(getattr(target, "laneMask", 0)) & lane_mask and int(target.trackId) not in selected_ids
    ]
    if not lane_targets:
      continue
    target = min(lane_targets, key=_target_priority)
    selected.append(target)
    selected_ids.add(int(target.trackId))
  return tuple(selected)


def rendered_radar_track_ids(radar_state: Any | None) -> frozenset[int]:
  if radar_state is None:
    return frozenset()
  track_ids = set()
  for lead in (radar_state.leadOne, radar_state.leadTwo):
    track_id = int(getattr(lead, "radarTrackId", -1))
    if bool(getattr(lead, "present", False)) and bool(getattr(lead, "radar", False)) and track_id >= 0:
      track_ids.add(track_id)
  return frozenset(track_ids)


def format_target_label(d_rel: float, v_rel: float, v_ego: float, metric: bool) -> str:
  """Format distance and estimated longitudinal target speed for the overlay."""
  target_speed = v_ego + v_rel
  if metric:
    return f"{d_rel:.0f}m  {target_speed * MS_TO_KPH:.0f}km/h"
  return f"{d_rel * M_TO_FT:.0f}ft  {target_speed * MS_TO_MPH:.0f}mph"
