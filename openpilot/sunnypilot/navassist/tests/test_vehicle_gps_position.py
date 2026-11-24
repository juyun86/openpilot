from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.navassist.map_coordinates import gcj_offset
from openpilot.sunnypilot.navassist.oem_navigation_feedback import GPS_MAX_AGE_NS, OemNavigationFeedback
from openpilot.sunnypilot.navassist.vehicle_gps_position import replace_position


T = 20_000_000_000


def native_gps():
  return SimpleNamespace(latitude=31.0, longitude=121.0, altitude=33.2, speed=8.3,
                         bearingDeg=42.0, horizontalAccuracy=2.1, verticalAccuracy=3.2,
                         speedAccuracy=0.2, bearingAccuracyDeg=1.3, vNED=[6.0, 5.0, -0.1],
                         unixTimestampMillis=1792734507957, source='ublox', flags=1,
                         hasFix=True, satelliteCount=16)


def observed():
  return {'gpsStatus': 'observed', 'latitude': 32.392873, 'longitude': 120.591314, 'gpsTimeMs': 1792734507957}


def test_only_coordinates_change_and_inverse_matches_input():
  gps = native_gps()
  before = vars(gps).copy()
  source = observed()
  assert replace_position(gps, source)
  assert {k for k in before if before[k] != vars(gps)[k]} == {'latitude', 'longitude'}
  dx, dy = gcj_offset(gps.longitude, gps.latitude)
  assert gps.longitude + dx == pytest.approx(source['longitude'], abs=1e-9)
  assert gps.latitude + dy == pytest.approx(source['latitude'], abs=1e-9)


@pytest.mark.parametrize('changes', [{'gpsStatus': s} for s in ('missing', 'stale', 'invalid', 'jump')] +
                         [{'gpsTimeMs': None}, {'latitude': None}, {'longitude': None},
                          {'latitude': float('nan')}, {'longitude': float('inf')},
                          {'latitude': 91}, {'longitude': 181}])
def test_unavailable_position_preserves_entire_native_message(changes):
  gps = native_gps()
  before = vars(gps).copy()
  assert not replace_position(gps, observed() | changes)
  assert vars(gps) == before


def test_original_no_fix_is_never_promoted():
  gps = native_gps()
  gps.hasFix = False
  gps.flags = 0
  assert replace_position(gps, observed())
  assert not gps.hasFix and gps.flags == 0


def test_observer_expiry_and_stalled_clock_restore_native_position():
  observer = OemNavigationFeedback()
  position = (32_392873 | (120_591314 << 28) | (5 << 57))
  def ingest(at):
    frames = [SimpleNamespace(address=a, src=1, dat=v.to_bytes(8, 'little'))
              for a, v in ((0x04f, position), (0x2f8, 8), (0x324, 1792734507957))]
    observer.ingest(frames, now_ns=at, received_ns=at)
  ingest(T)
  assert replace_position(native_gps(), observer.snapshot(T))
  assert not replace_position(native_gps(), observer.snapshot(T + GPS_MAX_AGE_NS))
  ingest(T + GPS_MAX_AGE_NS)
  gps = native_gps()
  before = vars(gps).copy()
  assert not replace_position(gps, observer.snapshot(T + GPS_MAX_AGE_NS))
  assert vars(gps) == before
