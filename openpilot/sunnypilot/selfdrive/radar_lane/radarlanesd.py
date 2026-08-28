#!/usr/bin/env python3
from __future__ import annotations

import time

from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.radar_lane.occupancy import DEFAULT_RADAR_TO_CAMERA_M
from openpilot.sunnypilot.selfdrive.radar_lane.publisher import RadarLaneStatePublisher, populate_input_status


ERROR_LOG_INTERVAL_S = 10.0


def _invalid_message(sm):
  message = messaging.new_message("radarLaneStateSP")
  message.valid = False
  state = message.radarLaneStateSP
  populate_input_status(state, sm)
  return message


def main() -> None:
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  sm = messaging.SubMaster(["modelV2", "radarTracks"], poll="modelV2")
  pm = messaging.PubMaster(["radarLaneStateSP"])
  publisher = RadarLaneStatePublisher(
    DEFAULT_RADAR_TO_CAMERA_M,
    radar_available=not CP.notCar and not CP.radarUnavailable,
  )
  last_error_log = -ERROR_LOG_INTERVAL_S

  while True:
    sm.update()
    if not sm.updated["modelV2"]:
      continue
    try:
      message = publisher.build_message(sm, sm["radarTracks"])
    except Exception:
      now = time.monotonic()
      if now - last_error_log >= ERROR_LOG_INTERVAL_S:
        cloudlog.exception("radarlanesd failed; publishing UNKNOWN occupancy")
        last_error_log = now
      message = _invalid_message(sm)
    pm.send("radarLaneStateSP", message)


if __name__ == "__main__":
  main()
