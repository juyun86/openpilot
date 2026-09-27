"""Read-only Model Y CH observations. Never an input to localization or control.

Layouts: TCAN ModelY_CH frame records captured 2026-09-20; 0x24A also
matches tesla_modely_hw4_perception.dbc. Road sampling on 2026-09-20 supports
km/h for speedValue; GPS datum and accuracy units remain unverified.
Existing CH observations and VEH 0x324 reach Cereal bus 1; PARTY 0x32B
is observed on Cereal bus 2. The 0x324 clock is diagnostic: its association
with the position fix is unverified. Ages use Cereal receive/progress times,
not certified GNSS measurement times. No verified checksum/counter exists.
"""
from __future__ import annotations

import math
import threading
import time


GPS_MAX_AGE_NS = 2_500_000_000
DEBUG_MAX_AGE_NS = 500_000_000
INGRESS_MAX_AGE_NS = 500_000_000
ADDRESSES = (0x04F, 0x2F8, 0x247, 0x24A)
GPS_DIAGNOSTIC_ADDRESSES = (0x324, 0x32B)


def bits(value: int, start: int, length: int, signed: bool = False) -> int:
  raw = (value >> start) & ((1 << length) - 1)
  return raw - (1 << length) if signed and raw & (1 << (length - 1)) else raw


class OemNavigationFeedback:
  def __init__(self):
    self._lock = threading.Lock()
    self._frames: dict[int, tuple[int, int | None, str]] = {}
    self._position_anchor: tuple[int, float, float] | None = None
    self._gps_time_value: int | None = None
    self._gps_time_progress_ns: int | None = None
    self._accuracy_raw: str | None = None

  def ingest(self, frames, *, now_ns: int, received_ns: int, valid: bool = True) -> None:
    with self._lock:
      if not valid:
        self._frames.clear()
        self._position_anchor = None
        self._gps_time_value = self._gps_time_progress_ns = None
        self._accuracy_raw = None
        return
      for frame in frames:
        address = int(frame.address)
        expected_bus = 2 if address == 0x32B else 1
        if int(frame.src) != expected_bus or address not in ADDRESSES + GPS_DIAGNOSTIC_ADDRESSES:
          continue
        if not 0 < now_ns <= received_ns or received_ns - now_ns > INGRESS_MAX_AGE_NS:
          continue  # Do not refresh observations from delayed/future events.
        previous = self._frames.get(address)
        if previous is not None and now_ns <= previous[0]:
          continue
        data = bytes(frame.dat)
        lengths = (5, 8) if address == 0x32B else (8,)
        value = int.from_bytes(data, "little") if len(data) in lengths and data != b"\xff" * len(data) else None
        status = "observed" if value is not None else "invalid"
        if address == 0x32B:
          # Real PARTY samples have DLC5, unlike the aggregate DLC8 definition.
          # Preserve bytes only; do not claim verified layout, units or accuracy.
          self._accuracy_raw = data.hex() if value is not None else None
        elif address == 0x324:
          if value is None or not 0 < value <= 253402300799999:
            value, status = None, "invalid"
          elif self._gps_time_value is None or value > self._gps_time_value:
            self._gps_time_value, self._gps_time_progress_ns = value, now_ns
          elif value < self._gps_time_value:
            value, status = None, "invalid"
          # Repeated CAN delivery of the same clock value cannot renew its age.
        elif value is not None and address == 0x04F:
          lat, lon = bits(value, 0, 28, True) * 1e-6, bits(value, 28, 29, True) * 1e-6
          accuracy = bits(value, 57, 7)
          if not (-90 <= lat <= 90 and -180 <= lon <= 180) or accuracy in (0, 127):
            value, status = None, "invalid"
          else:
            anchor = self._position_anchor
            if anchor is not None and 0 < now_ns - anchor[0] <= 10_000_000_000:
              dt = (now_ns - anchor[0]) / 1e9
              a = math.sin(math.radians(lat - anchor[1]) / 2) ** 2
              a += math.cos(math.radians(lat)) * math.cos(math.radians(anchor[1])) * math.sin(math.radians(lon - anchor[2]) / 2) ** 2
              distance = 12_742_000 * math.asin(min(1.0, math.sqrt(a)))
              if distance > 100 + 100 * dt:
                value, status = None, "jump"
            if value is not None:
              self._position_anchor = (now_ns, lat, lon)
        elif value is not None and address == 0x2F8:
          if bits(value, 0, 8) in (0, 255) or bits(value, 53, 2):
            value, status = None, "invalid"
        self._frames[address] = (now_ns, value, status)

  def snapshot(self, now_ns: int | None = None) -> dict:
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    with self._lock:
      def fresh(address):
        frame = self._frames.get(address)
        if frame is None:
          return None, "missing"
        budget = GPS_MAX_AGE_NS if address in (0x04F, 0x2F8) + GPS_DIAGNOSTIC_ADDRESSES else DEBUG_MAX_AGE_NS
        if not 0 <= now_ns - frame[0] < budget:
          return None, "stale"
        return frame[1], frame[2]

      position, position_status = fresh(0x04F)
      motion, motion_status = fresh(0x2F8)
      autopilot, _ = fresh(0x247)
      visual, _ = fresh(0x24A)
      gps_status = next((s for s in (position_status, motion_status) if s != "observed"), "observed")
      gps_ok = gps_status == "observed"
      gps_time, _ = fresh(0x324)
      accuracy, _ = fresh(0x32B)

      def age_ms(address):
        value, _ = fresh(address)
        return (now_ns - self._frames[address][0]) // 1_000_000 if value is not None else None

      time_age_ns = now_ns - self._gps_time_progress_ns if self._gps_time_progress_ns is not None else -1
      time_current = gps_time is not None and 0 <= time_age_ns < GPS_MAX_AGE_NS

      def field(value, start, length):
        return bits(value, start, length) if value is not None else None

      heading = field(motion, 8, 16)
      speed = field(motion, 24, 16)
      return {
        "gpsStatus": gps_status,
        # Diagnostic source ages, not a guarantee of GNSS measurement coherence.
        "gpsTimeMs": gps_time if time_current else None,
        "gpsTimeAgeMs": time_age_ns // 1_000_000 if time_current else None,
        "gpsPositionAgeMs": age_ms(0x04F),
        "gpsMotionAgeMs": age_ms(0x2F8),
        "gpsAccuracyRaw": self._accuracy_raw if accuracy is not None else None,
        "gpsAccuracyAgeMs": age_ms(0x32B),
        "latitude": round(bits(position, 0, 28, True) * 1e-6, 6) if gps_ok else None,
        "longitude": round(bits(position, 28, 29, True) * 1e-6, 6) if gps_ok else None,
        "accuracyValue": round(bits(position, 57, 7) * 0.2, 1) if gps_ok else None,
        "hdop": round(bits(motion, 0, 8) * 0.1, 1) if motion is not None else None,
        "headingDeg": round(heading / 128, 3) if heading is not None and heading < 46080 else None,
        "speedValue": round(speed / 256, 3) if speed is not None and speed != 65535 else None,
        "mapAvailable": bool(bits(autopilot, 8, 1)) if autopilot is not None else None,
        "roadEstimator": field(autopilot, 0, 2),
        "oemLaneChangeState": field(autopilot, 50, 6),
        "controllerHealth": field(autopilot, 24, 2),
        "alcState": field(autopilot, 12, 4),
        "forkState": field(autopilot, 45, 5),
        "abortReason": field(autopilot, 32, 8),
        "navAvailable": bool(bits(visual, 39, 1)) if visual is not None else None,
        "navUsage": field(visual, 8, 2),
        "autosteerHealth": field(visual, 21, 3),
        "plannerState": field(visual, 28, 4),
        "navDistanceM": bits(visual, 40, 8) * 100 if visual is not None and bits(visual, 39, 1) and bits(visual, 40, 8) != 255 else None,
      }
