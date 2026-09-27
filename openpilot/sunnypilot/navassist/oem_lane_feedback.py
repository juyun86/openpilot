from __future__ import annotations

import threading
import time
from typing import Iterable
from openpilot.sunnypilot.navassist.lane_can import decode_lane_topology


LANE_TOPOLOGY_ADDRESS = 0x239
LANE_CHANGE_ADDRESS = 0x399
TESLA_VEHICLE_BUS = 1
LANE_TOPOLOGY_MAX_AGE_NS = 400_000_000
LANE_CHANGE_MAX_AGE_NS = 1_250_000_000
MIN_VIRTUAL_LANE_VIEW_RANGE_M = 20
MIN_VIRTUAL_LANE_WIDTH_M = 2.0
MAX_VIRTUAL_LANE_WIDTH_M = 6.6875
USABLE_LINE_STATES = frozenset((1, 2))  # AVAILABLE, FUSED


def tesla_checksum_valid(address: int, data: bytes) -> bool:
  if len(data) != 8:
    return False
  expected = ((address & 0xFF) + ((address >> 8) & 0xFF) + sum(data[:7])) & 0xFF
  return data[7] == expected


class OemLaneFeedback:
  """Fail-closed decoder for the Tesla 0x239/0x399 lane signals."""

  def __init__(self):
    self._lock = threading.Lock()
    self._candidate_position = "unknown"
    self._candidate_count = 0
    self._position = "unknown"
    self._topology_ns = 0
    self._topology_counter = None
    self._topology_counter_valid = False
    self._lane_width_m = None
    self._view_range_m = None
    self._center_coefficients = (None, None, None, None)
    self._left_line_usage = 0
    self._right_line_usage = 0
    self._left_line_samples = 0
    self._right_line_samples = 0
    self._last_frame_ns: dict[int, int] = {}
    self._left_fork = 0
    self._right_fork = 0
    self._geometry_valid = False
    self._change_ns = 0
    self._left_allowed_samples = 0
    self._right_allowed_samples = 0
    self._left_allowed = False
    self._right_allowed = False
    self._auto_lane_change_state = 0
    self._left_safety_blocked = True
    self._right_safety_blocked = True
    self._vehicle_ns = 0
    self._blindspot_valid = False
    self._left_blindspot = False
    self._right_blindspot = False
    self._radar_valid = False
    self._lead_present = False
    self._lead_distance_m = None
    self._lead_speed_kph = None
    self._lead_relative_speed_kph = None
    self._vehicle_state_valid = False
    self._ego_speed_kph = None
    self._lateral_active = False
    self._brake_pressed = False
    self._gas_pressed = False
    self._lane_change_state = 0
    self._lane_change_direction = 0

  def ingest(self, frames: Iterable, now_ns: int | None = None, *, received_ns: int | None = None) -> None:
    """Ingest one CAN event; supply received_ns when now_ns is its source timestamp."""
    observed_ns = time.monotonic_ns() if now_ns is None else now_ns
    current_ns = observed_ns if received_ns is None else received_ns
    with self._lock:
      checked = set()
      rejected = set()
      for frame in frames:
        if int(frame.src) != TESLA_VEHICLE_BUS:
          continue
        address = int(frame.address)
        data = bytes(frame.dat)
        if address not in (LANE_TOPOLOGY_ADDRESS, LANE_CHANGE_ADDRESS):
          continue
        if address not in checked:
          checked.add(address)
          max_age = LANE_TOPOLOGY_MAX_AGE_NS if address == LANE_TOPOLOGY_ADDRESS else LANE_CHANGE_MAX_AGE_NS
          previous_ns = self._last_frame_ns.get(address, 0)
          if not self._fresh(observed_ns, current_ns, max_age) or observed_ns <= previous_ns:
            rejected.add(address)
            self._invalidate(address)
          else:
            self._last_frame_ns[address] = observed_ns
            if previous_ns and observed_ns - previous_ns > max_age:
              self._invalidate(address)
        if address in rejected:
          continue
        if len(data) != 8:
          self._invalidate(address)
          rejected.add(address)
          continue
        # 0x239 ends with a rolling counter and has no checksum. 0x399 uses
        # Tesla's additive checksum in byte 7.
        if address == LANE_CHANGE_ADDRESS and not tesla_checksum_valid(address, data):
          self._block_lane_changes()
          rejected.add(address)
          continue
        if address == LANE_TOPOLOGY_ADDRESS:
          self._ingest_topology(data, observed_ns)
        else:
          self._ingest_lane_change(data, observed_ns)

  def _ingest_topology(self, data: bytes, now_ns: int) -> None:
    decoded = decode_lane_topology(data)
    assert decoded is not None  # ingest has already checked DLC.
    counter = decoded["counter"]
    if self._topology_counter is not None and ((counter - self._topology_counter) & 0x0F) != 1:
      # 0x239 has no checksum. Counter discontinuity therefore invalidates the
      # observation until three fresh, consecutive frames establish it again.
      self._invalidate_topology()
    else:
      self._topology_counter_valid = True
    self._topology_counter = counter

    left_exists = decoded["left_lane_exists"]
    right_exists = decoded["right_lane_exists"]
    lane_width_m = decoded["lane_width_m"]
    view_range_m = decoded["view_range_m"]
    self._lane_width_m = lane_width_m
    self._view_range_m = view_range_m
    self._center_coefficients = tuple(decoded["center_coefficients"])
    self._left_line_usage = decoded["left_line_usage"]
    self._right_line_usage = decoded["right_line_usage"]
    self._left_fork = decoded["left_fork"]
    self._right_fork = decoded["right_fork"]
    self._geometry_valid = bool(
      self._topology_counter_valid
      and MIN_VIRTUAL_LANE_WIDTH_M <= lane_width_m <= MAX_VIRTUAL_LANE_WIDTH_M
      and view_range_m >= MIN_VIRTUAL_LANE_VIEW_RANGE_M
    )
    if not self._geometry_valid:
      self._invalidate_topology()
      # Preserve this counter as the baseline, without counting its evidence.
      self._topology_counter = counter
      return

    position = {
      (False, False): "single",
      (False, True): "leftmost",
      (True, False): "rightmost",
      (True, True): "middle",
    }[(left_exists, right_exists)]
    self._topology_ns = now_ns
    if position == self._candidate_position:
      self._candidate_count += 1
    else:
      self._position = "unknown"
      self._candidate_position = position
      self._candidate_count = 1
      self._left_line_samples = self._right_line_samples = 0
    self._candidate_count = min(self._candidate_count, 3)
    self._left_line_samples = min(self._left_line_samples + 1, 3) if self._left_line_usage in USABLE_LINE_STATES else 0
    self._right_line_samples = min(self._right_line_samples + 1, 3) if self._right_line_usage in USABLE_LINE_STATES else 0
    if self._candidate_count >= 3:
      self._position = position

  def _ingest_lane_change(self, data: bytes, now_ns: int) -> None:
    raw = int.from_bytes(data, "little")
    state = (raw >> 46) & 0x1F
    blind_left = (raw >> 4) & 0x03
    blind_right = (raw >> 6) & 0x03
    # Keep physical hazards separate from ordinary ALC unavailability. Vision
    # may replace OEM lane permission, but cannot replace blind-spot health.
    same_sample = now_ns == self._change_ns
    self._left_safety_blocked = ((same_sample and self._left_safety_blocked)
                                 or blind_left != 0 or state in (11, 13, 15, 21, 22, 23, 26))
    self._right_safety_blocked = ((same_sample and self._right_safety_blocked)
                                  or blind_right != 0 or state in (12, 14, 16, 21, 24, 25, 27))
    # In-progress states (9/10) describe an already executing manoeuvre; they
    # are not permission to begin another one.
    left_sample = state in (6, 8) and blind_left == 0
    right_sample = state in (7, 8) and blind_right == 0
    # A CAN packet can contain repeated 399 frames but has only one timestamp.
    # Count it at most once, while still applying every negative observation.
    new_sample = int(now_ns != self._change_ns)
    self._change_ns = now_ns
    self._auto_lane_change_state = state
    self._left_allowed_samples = min(self._left_allowed_samples + new_sample, 2) if left_sample else 0
    self._right_allowed_samples = min(self._right_allowed_samples + new_sample, 2) if right_sample else 0
    self._left_allowed = left_sample and self._left_allowed_samples >= 2
    self._right_allowed = right_sample and self._right_allowed_samples >= 2

  def _block_lane_changes(self) -> None:
    self._change_ns = 0
    self._left_allowed_samples = 0
    self._right_allowed_samples = 0
    self._left_allowed = False
    self._right_allowed = False

  @staticmethod
  def _fresh(stamp_ns: int, now_ns: int, max_age_ns: int) -> bool:
    return stamp_ns > 0 and 0 <= now_ns - stamp_ns <= max_age_ns

  def _invalidate_topology(self) -> None:
    self._position = self._candidate_position = "unknown"
    self._candidate_count = 0
    self._topology_ns = 0
    self._topology_counter = None
    self._topology_counter_valid = self._geometry_valid = False
    self._left_line_samples = self._right_line_samples = 0

  def _invalidate(self, address: int) -> None:
    if address == LANE_TOPOLOGY_ADDRESS:
      self._invalidate_topology()
    else:
      self._block_lane_changes()

  def update_vehicle(self, *, now_ns: int, vehicle_valid: bool, radar_valid: bool,
                     blindspot_valid: bool, left_blindspot: bool, right_blindspot: bool,
                     ego_speed_mps: float, lateral_active: bool, brake_pressed: bool, gas_pressed: bool,
                     lane_change_state: int, lane_change_direction: int, lead_present: bool,
                     lead_distance_m: float, lead_speed_mps: float, lead_relative_speed_mps: float) -> None:
    with self._lock:
      self._vehicle_ns = now_ns
      self._vehicle_state_valid = vehicle_valid
      self._radar_valid = radar_valid
      self._blindspot_valid = blindspot_valid
      self._left_blindspot = left_blindspot if blindspot_valid else False
      self._right_blindspot = right_blindspot if blindspot_valid else False
      self._ego_speed_kph = ego_speed_mps * 3.6 if vehicle_valid else None
      self._lateral_active = lateral_active if vehicle_valid else False
      self._brake_pressed = brake_pressed if vehicle_valid else False
      self._gas_pressed = gas_pressed if vehicle_valid else False
      self._lane_change_state = lane_change_state if vehicle_valid else 0
      self._lane_change_direction = lane_change_direction if vehicle_valid else 0
      self._lead_present = lead_present if radar_valid else False
      self._lead_distance_m = lead_distance_m if radar_valid and lead_present else None
      self._lead_speed_kph = lead_speed_mps * 3.6 if radar_valid and lead_present else None
      self._lead_relative_speed_kph = lead_relative_speed_mps * 3.6 if radar_valid and lead_present else None

  def snapshot(self, now_ns: int | None = None, *, include_topology_details: bool = False) -> dict:
    with self._lock:
      current_ns = time.monotonic_ns() if now_ns is None else now_ns
      topology_fresh = self._fresh(self._topology_ns, current_ns, LANE_TOPOLOGY_MAX_AGE_NS)
      change_fresh = self._fresh(self._change_ns, current_ns, LANE_CHANGE_MAX_AGE_NS)
      vehicle_fresh = self._fresh(self._vehicle_ns, current_ns, 500_000_000)
      if not topology_fresh:
        self._invalidate_topology()
      if not change_fresh:
        self._block_lane_changes()
      if not vehicle_fresh:
        self._vehicle_ns = 0
      state = {
        "position": self._position if topology_fresh else "unknown",
        "positionValid": topology_fresh and self._position != "unknown" and self._geometry_valid,
        "leftAllowed": change_fresh and self._left_allowed,
        "rightAllowed": change_fresh and self._right_allowed,
        "permissionValid": change_fresh,
        "autoLaneChangeState": self._auto_lane_change_state if change_fresh else 0,
        "blindspotValid": vehicle_fresh and self._blindspot_valid,
        "leftBlindspot": vehicle_fresh and self._left_blindspot,
        "rightBlindspot": vehicle_fresh and self._right_blindspot,
        "radarValid": vehicle_fresh and self._radar_valid,
        "leadPresent": vehicle_fresh and self._lead_present,
        "leadDistanceM": self._lead_distance_m if vehicle_fresh else None,
        "leadSpeedKph": self._lead_speed_kph if vehicle_fresh else None,
        "leadRelativeSpeedKph": self._lead_relative_speed_kph if vehicle_fresh else None,
        "vehicleStateValid": vehicle_fresh and self._vehicle_state_valid,
        "egoSpeedKph": self._ego_speed_kph if vehicle_fresh else None,
        "lateralActive": vehicle_fresh and self._lateral_active,
        "brakePressed": vehicle_fresh and self._brake_pressed,
        "gasPressed": vehicle_fresh and self._gas_pressed,
        "laneChangeState": self._lane_change_state if vehicle_fresh else 0,
        "laneChangeDirection": self._lane_change_direction if vehicle_fresh else 0,
      }
      # Keep the UDP acknowledgement consumed by TesNav schema-compatible.
      # Only the C3 controller asks for the richer 0x239 diagnostics.
      if include_topology_details:
        state.update({
          "leftSafetyBlocked": not change_fresh or self._left_safety_blocked,
          "rightSafetyBlocked": not change_fresh or self._right_safety_blocked,
          "topologyCounter": self._topology_counter,
          "topologyCounterValid": topology_fresh and self._topology_counter_valid,
          "laneWidthM": self._lane_width_m if topology_fresh else None,
          "viewRangeM": self._view_range_m if topology_fresh else None,
          "centerC0": self._center_coefficients[0] if topology_fresh else None,
          "centerC1": self._center_coefficients[1] if topology_fresh else None,
          "centerC2": self._center_coefficients[2] if topology_fresh else None,
          "centerC3": self._center_coefficients[3] if topology_fresh else None,
          "leftLineUsage": self._left_line_usage if topology_fresh else 0,
          "rightLineUsage": self._right_line_usage if topology_fresh else 0,
          "leftLaneEvidenceValid": topology_fresh and self._left_line_samples >= 3,
          "rightLaneEvidenceValid": topology_fresh and self._right_line_samples >= 3,
          "leftFork": self._left_fork if topology_fresh else 0,
          "rightFork": self._right_fork if topology_fresh else 0,
        })
      return state
