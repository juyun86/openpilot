"""Render on one bounded worker, alongside navassistd, never in its snapshot loop."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import os
from pathlib import Path
import time

import numpy as np

from openpilot.selfdrive.navigation.pmtiles import PMTiles
from openpilot.selfdrive.navigation.renderer import MapRenderer
from openpilot.selfdrive.navigation.route import RouteTracker
from openpilot.sunnypilot.navassist.nav_model_input import ModelNavigationInput
from openpilot.sunnypilot.navassist.map_coordinates import to_wgs84

MAP_MAX_AGE_NS = 500_000_000


class LocalTiles:
  def __init__(self, path):
    self.file = open(path, 'rb')
    self.size = os.fstat(self.file.fileno()).st_size

  def read(self, offset, length):
    self.file.seek(offset)
    return self.file.read(length)

  def prefetch(self, ranges):
    pass  # Local reads need no network prefetch.

  def close(self):
    self.file.close()


def open_basemap():
  path = os.environ.get('NAV_MODEL_MAP_ARCHIVE')
  if path:
    return PMTiles(LocalTiles(Path(path)))
  from openpilot.selfdrive.navigation.tiles import RemoteTiles
  return PMTiles(RemoteTiles())


@dataclass(frozen=True)
class MapFrame:
  route_key: tuple
  mono_time_ns: int
  pixels: bytes


def route_key(value):
  route = value.route
  return route.session_id, route.route_revision, route.coordinate_system, route.points


class ModelMapRenderer:
  def __init__(self, archive_factory=open_basemap):
    self.archive_factory = archive_factory
    self.renderer = None
    self.tracker = None
    self.key = None
    self.retry_at = 0

  def render(self, value: ModelNavigationInput, now_ns: int) -> MapFrame | None:
    if now_ns < self.retry_at:
      return None
    try:
      key = route_key(value)
      if self.key != key:
        route = np.array([to_wgs84(lon, lat, value.route.coordinate_system) for lat, lon in value.route.points])
        if not np.isfinite(route).all() or np.any(np.abs(route[:, 1]) >= 85):
          return None
        self.tracker = RouteTracker(route)
        self.key = key
      if self.renderer is None:
        self.renderer = MapRenderer(self.archive_factory())
      lon, lat = to_wgs84(value.longitude, value.latitude, value.route.coordinate_system)
      pose = self.tracker.pose(self.tracker.project(lon, lat) + max(value.speed_mps, 0) * value.location_age_ms / 1000)
      pixels, valid = self.renderer.render(*pose, self.tracker.route)
      if valid:
        return MapFrame(key, now_ns, pixels.tobytes())
    except Exception:
      self.close()
      self.retry_at = time.monotonic_ns() + 10_000_000_000
    return None

  def close(self):
    if self.renderer is not None:
      self.renderer.archive.close()
    self.renderer = self.tracker = self.key = None


class ModelMapWorker:
  def __init__(self, renderer=None, *, enabled=None):
    self.renderer = renderer or ModelMapRenderer()
    self.enabled = (bool(os.environ.get('NAV_MODEL_MAP_ARCHIVE')) or os.environ.get('NAV_MODEL_ONLINE_MAP') == '1') if enabled is None else enabled
    self.executor = None  # Do not start a thread before receiving eligible navigation.
    self.pending = None
    self.cached = None
    self.active_key = None
    self.generation = self.pending_generation = self.renderer_generation = 0

  def _render(self, value, now_ns, generation):
    if self.renderer_generation != generation:
      self.renderer.close()
      self.renderer_generation = generation
    return self.renderer.render(value, now_ns)

  def update(self, value: ModelNavigationInput | None, now_ns: int) -> MapFrame | None:
    if not self.enabled:
      return None
    key = route_key(value) if value is not None else None
    if key != self.active_key:
      self.active_key = key
      self.generation += 1
      self.cached = None
    if self.pending is not None and self.pending.done():
      try:
        frame = self.pending.result()
        self.cached = frame if self.pending_generation == self.generation else None
      except Exception:
        self.cached = None
      self.pending = None
    if value is None:
      self.cached = None
      return None
    if self.executor is None:
      self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='navassist-map')
    if self.pending is None:
      self.pending_generation = self.generation
      self.pending = self.executor.submit(self._render, value, now_ns, self.generation)
    frame = self.cached
    if frame is not None and frame.route_key == route_key(value) and 0 <= now_ns - frame.mono_time_ns <= MAP_MAX_AGE_NS:
      return frame
    return None

  def close(self):
    if self.executor is not None:
      # Close on the same worker after any bounded network operation finishes.
      self.executor.submit(self.renderer.close)
      self.executor.shutdown(wait=False, cancel_futures=False)
