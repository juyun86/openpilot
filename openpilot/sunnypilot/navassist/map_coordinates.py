"""GCJ-02 inverse for map context and the C3XL Tesla GPS position adapter.

Offset formula adapted from wandergis/coordtransform (MIT), index.js.
Iteration reduces numerical inverse error, not the underlying map/GPS error.
"""
import math


def gcj_offset(longitude, latitude):
  if not (73.66 < longitude < 135.05 and 3.86 < latitude < 53.55):
    return 0.0, 0.0
  x, y = longitude - 105, latitude - 35
  lat = -100 + 2*x + 3*y + .2*y*y + .1*x*y + .2*math.sqrt(abs(x))
  lon = 300 + x + 2*y + .1*x*x + .1*x*y + .1*math.sqrt(abs(x))
  common = (20*math.sin(6*x*math.pi) + 20*math.sin(2*x*math.pi))*2/3
  lat += common + (20*math.sin(y*math.pi) + 40*math.sin(y/3*math.pi))*2/3
  lat += (160*math.sin(y/12*math.pi) + 320*math.sin(y/30*math.pi))*2/3
  lon += common + (20*math.sin(x*math.pi) + 40*math.sin(x/3*math.pi))*2/3
  lon += (150*math.sin(x/12*math.pi) + 300*math.sin(x/30*math.pi))*2/3
  rad = math.radians(latitude)
  magic = 1 - .00669342162296594323 * math.sin(rad)**2
  root = math.sqrt(magic)
  return (lon*180/(6378245/root*math.cos(rad)*math.pi),
          lat*180/(6378245*(1-.00669342162296594323)/(magic*root)*math.pi))


def to_wgs84(longitude, latitude, coordinate_system):
  if not math.isfinite(longitude) or not math.isfinite(latitude) or not -180 <= longitude <= 180 or not -90 <= latitude <= 90:
    raise ValueError('Invalid map coordinates')
  if coordinate_system == 'wgs84':
    return longitude, latitude
  if coordinate_system != 'gcj02':
    raise ValueError('Unknown map coordinate system')
  lon, lat = longitude, latitude
  for _ in range(8):
    dx, dy = gcj_offset(lon, lat)
    ex, ey = lon + dx - longitude, lat + dy - latitude
    lon, lat = lon - ex, lat - ey
    if max(abs(ex), abs(ey)) < 1e-9:
      return lon, lat
  raise ValueError('GCJ-02 inverse did not converge')
