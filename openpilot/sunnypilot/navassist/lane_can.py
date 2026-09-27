"""Shared, stateless decoding of the read-only Tesla lane observation."""


def decode_lane_topology(data: bytes) -> dict | None:
  """Decode values only; callers must check source, freshness and continuity.

  Line usage is not paint type. Neither fork nor missing-lane bits grant crossing.
  """
  if len(data) != 8:
    return None
  return {
    "counter": (data[7] >> 4) & 0x0F,
    "left_lane_exists": bool(data[0] & 0x01),
    "right_lane_exists": bool(data[0] & 0x02),
    "lane_width_m": ((data[0] >> 4) & 0x0F) * 0.3125 + 2.0,
    "view_range_m": data[1],
    "center_coefficients": [data[2] * 0.035 - 3.5, data[3] * 0.0016 - 0.2,
                            data[4] * 0.00002 - 0.0025, data[5] * 0.00000024 - 0.00003],
    "left_line_usage": data[6] & 0x03,
    "right_line_usage": (data[6] >> 2) & 0x03,
    "left_fork": (data[6] >> 4) & 0x03,
    "right_fork": (data[6] >> 6) & 0x03,
  }
