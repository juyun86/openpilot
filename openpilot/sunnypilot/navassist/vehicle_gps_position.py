"""Replace only GPS latitude/longitude on the C3XL Tesla installation.

The receiver retains cadence, time, motion, accuracy and fix status. Missing
CAN observations leave its message unchanged. CAN ages are receive-time ages,
not verified GNSS measurement times.
"""
import time

from openpilot.sunnypilot.navassist.map_coordinates import to_wgs84
from openpilot.sunnypilot.navassist.oem_navigation_feedback import OemNavigationFeedback


def replace_position(gps, observation):
  if observation.get('gpsStatus') != 'observed' or observation.get('gpsTimeMs') is None:
    return False
  latitude, longitude = observation.get('latitude'), observation.get('longitude')
  if latitude is None or longitude is None:
    return False
  try:
    longitude, latitude = to_wgs84(longitude, latitude, 'gcj02')
  except (ValueError, TypeError):
    return False
  gps.latitude, gps.longitude = latitude, longitude
  return True


class VehicleGpsPosition:
  def __init__(self):
    from openpilot.sunnypilot.hardware.profile import HardwareProfile, get_hardware_profile
    self.enabled = get_hardware_profile() == HardwareProfile.C3XL
    self.brand = None
    self.sock = None
    self.observer = OemNavigationFeedback()

  def apply(self, gps):
    if not self.enabled:
      return False
    from openpilot.cereal import messaging
    if self.brand is None:
      from opendbc.car.structs import car
      from openpilot.common.params import Params
      raw = Params().get('CarParams')
      if not raw:
        return False
      with car.CarParams.from_bytes(raw) as cp:
        self.brand = cp.brand
    if self.brand != 'tesla':
      return False
    if self.sock is None:
      self.sock = messaging.sub_sock('can', conflate=False)
    for event in messaging.drain_sock(self.sock, wait_for_one=False):
      self.observer.ingest(event.can, now_ns=event.logMonoTime,
                           received_ns=time.monotonic_ns(), valid=event.valid)
    return replace_position(gps, self.observer.snapshot(time.monotonic_ns()))
