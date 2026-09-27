#!/usr/bin/env python3
from __future__ import annotations

import math
import threading
import time

from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.navassist.discovery import DISCOVERY_HOST, DISCOVERY_PORT, NavAssistDiscoveryServer
from openpilot.sunnypilot.navassist.identity import NavAssistDeviceIdentity, NavAssistPairingStore
from openpilot.sunnypilot.navassist.oem_lane_feedback import OemLaneFeedback
from openpilot.sunnypilot.navassist.oem_navigation_feedback import OemNavigationFeedback
from openpilot.sunnypilot.navassist.protocol import NavAssistStore
from openpilot.sunnypilot.navassist.publisher import build_nav_assist_message
from openpilot.sunnypilot.navassist.server import NavAssistHTTPServer
from openpilot.sunnypilot.navassist.udp_receiver import NavAssistUDPServer, UDP_SNAPSHOT_PORT
from openpilot.sunnypilot.navassist.diagnostics import ingress_status_path
from openpilot.sunnypilot.navassist.settings import atomic_json


LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 7766
PUBLISH_HZ = 20
LOCALIZATION_MAX_AGE_NS = 500_000_000
LOCAL_POSITION_MAX_STD_M = 10.0
REPLAY_CHECKPOINT_PATH = "/dev/shm/navassist_replay_state.json"
MAINTENANCE_REFRESH_NS = 1_000_000_000


def pending_lane_announcement(intent, *, healthy: bool, now_ns: int, model_state: int) -> dict:
  if (not healthy or not intent.valid or not intent.signalRequested or intent.spLaneChangeReady
      or intent.targetLaneIndex < 0 or model_state not in (0, 1)
      or not 0 <= now_ns - intent.publishMonoTime <= 200_000_000
      or len(intent.announcementId) != 32 or str(intent.direction) not in ("left", "right")):
    return {}
  return {'id': str(intent.announcementId), 'sessionId': str(intent.sessionId), 'direction': str(intent.direction)}


def current_lane_decision(intent, *, healthy: bool, now_ns: int) -> dict:
  if (not healthy or not intent.valid or not intent.sessionId
      or not 0 <= now_ns - intent.publishMonoTime <= 200_000_000):
    return {}
  return {'sessionId': str(intent.sessionId), 'reason': str(intent.reason),
          'signalRequested': bool(intent.signalRequested), 'direction': str(intent.direction)}


def local_position_std_m(location) -> float:
  position_ecef = location.positionECEF
  if not position_ecef.valid or len(position_ecef.std) < 3:
    return math.inf
  values = tuple(float(value) for value in position_ecef.std[:3])
  if not all(math.isfinite(value) and value >= 0.0 for value in values):
    return math.inf
  return math.sqrt(sum(value * value for value in values))


def local_localization_valid(sm, now_ns: int) -> bool:
  service = "liveLocationKalman"
  location = sm[service]
  age_ns = now_ns - sm.logMonoTime[service]
  return bool(
    sm.seen[service] and sm.alive[service] and sm.valid[service]
    and 0 <= age_ns <= LOCALIZATION_MAX_AGE_NS
    and location.status == log.LiveLocationKalman.Status.valid and location.positionGeodetic.valid
    and len(location.positionGeodetic.value) >= 2
    and location.gpsOK and location.inputsOK and location.sensorsOK and location.deviceStable
    and not location.excessiveResets and local_position_std_m(location) <= LOCAL_POSITION_MAX_STD_M
  )


def main() -> None:
  params = Params()
  identity = NavAssistDeviceIdentity.load_or_create(params=params)
  pairing = NavAssistPairingStore(params)

  store = NavAssistStore(checkpoint_path=REPLAY_CHECKPOINT_PATH)
  oem_lane_feedback = OemLaneFeedback()
  oem_navigation_feedback = OemNavigationFeedback()
  speech_feedback = {}
  lane_decision = {}
  server = NavAssistHTTPServer((LISTEN_HOST, LISTEN_PORT), store, identity, pairing)
  udp_server = NavAssistUDPServer(
    (LISTEN_HOST, UDP_SNAPSHOT_PORT), store, ack_payload_provider=oem_lane_feedback.snapshot,
    telemetry_provider=oem_navigation_feedback.snapshot,
    lane_decision_provider=lambda: lane_decision,
    announcement_provider=lambda: speech_feedback,
  )
  try:
    discovery_server = NavAssistDiscoveryServer(
      (DISCOVERY_HOST, DISCOVERY_PORT), identity, pairing, is_offroad=lambda: params.get_bool("IsOffroad"),
    )
  except BaseException:
    udp_server.server_close()
    server.server_close()
    raise
  server_thread = threading.Thread(target=server.serve_forever, name="navassist-http", daemon=True)
  discovery_thread = threading.Thread(
    target=discovery_server.serve_forever, name="navassist-discovery", daemon=True,
  )
  udp_thread = threading.Thread(target=udp_server.serve_forever, name="navassist-udp", daemon=True)
  server_started = False
  discovery_started = False
  udp_started = False
  try:
    server_thread.start()
    server_started = True
    discovery_thread.start()
    discovery_started = True
    udp_thread.start()
    udp_started = True
    cloudlog.warning(
      "navassistd receiver online on HTTP port %d, discovery port %d and data-only UDP port %d",
      LISTEN_PORT, DISCOVERY_PORT, UDP_SNAPSHOT_PORT,
    )

    pm = messaging.PubMaster(["navAssistStateSP"])
    sm = messaging.SubMaster(["liveLocationKalman", "carState", "carControl", "radarState", "modelV2", "navLaneIntentSP"])
    can_sock = messaging.sub_sock("can", conflate=False)
    ratekeeper = Ratekeeper(PUBLISH_HZ)
    next_maintenance_ns = 0
    while True:
      sm.update(0)
      for event in messaging.drain_sock(can_sock):
        oem_lane_feedback.ingest(event.can, now_ns=int(event.logMonoTime), received_ns=time.monotonic_ns())
        oem_navigation_feedback.ingest(
          event.can, now_ns=int(event.logMonoTime), received_ns=time.monotonic_ns(), valid=bool(event.valid),
        )
      now_ns = time.monotonic_ns()
      car_valid = all(sm.seen[name] and sm.alive[name] and sm.valid[name] for name in ("carState", "carControl", "modelV2"))
      radar_valid = bool(sm.seen["radarState"] and sm.alive["radarState"] and sm.valid["radarState"])
      car_state = sm["carState"]
      lead = sm["radarState"].leadOne
      model_meta = sm["modelV2"].meta
      speech_feedback = pending_lane_announcement(
        sm['navLaneIntentSP'], now_ns=now_ns, model_state=int(model_meta.laneChangeState.raw),
        healthy=bool(car_valid and sm['carControl'].latActive and sm.seen['navLaneIntentSP']
                     and sm.alive['navLaneIntentSP'] and sm.valid['navLaneIntentSP']),
      )
      lane_decision = current_lane_decision(
        sm['navLaneIntentSP'], now_ns=now_ns,
        healthy=bool(sm.seen['navLaneIntentSP'] and sm.alive['navLaneIntentSP']
                     and sm.valid['navLaneIntentSP']),
      )
      oem_lane_feedback.update_vehicle(
        now_ns=now_ns, vehicle_valid=car_valid, radar_valid=radar_valid, blindspot_valid=car_valid,
        left_blindspot=bool(car_state.leftBlindspot), right_blindspot=bool(car_state.rightBlindspot),
        ego_speed_mps=float(car_state.vEgo), lateral_active=bool(sm["carControl"].latActive),
        brake_pressed=bool(car_state.brakePressed), gas_pressed=bool(car_state.gasPressed),
        lane_change_state=int(model_meta.laneChangeState.raw), lane_change_direction=int(model_meta.laneChangeDirection.raw),
        lead_present=bool(lead.present), lead_distance_m=float(lead.dRel), lead_speed_mps=float(lead.vLead),
        lead_relative_speed_mps=float(lead.vRel),
      )
      if now_ns >= next_maintenance_ns:
        if params.get_bool("NavAssistPairingReset"):
          pairing.reset()
          store.reset()
          params.put_bool("NavAssistPairingReset", False, block=True)
        next_maintenance_ns = now_ns + MAINTENANCE_REFRESH_NS
        try:
          atomic_json(ingress_status_path(), {'updated_mono_ns': now_ns, **udp_server.diagnostics()})
        except OSError:
          pass
      localization_valid = local_localization_valid(sm, now_ns)
      message = build_nav_assist_message(
        store.current(), now_ns, local_localization_valid=localization_valid,
      )
      pm.send("navAssistStateSP", message)
      ratekeeper.keep_time()
  finally:
    if udp_started:
      udp_server.shutdown()
    if discovery_started:
      discovery_server.shutdown()
    if server_started:
      server.shutdown()
    if discovery_started:
      discovery_thread.join(timeout=2)
    if server_started:
      server_thread.join(timeout=2)
    if udp_started:
      udp_thread.join(timeout=2)
    discovery_server.server_close()
    udp_server.server_close()
    server.server_close()


if __name__ == "__main__":
  main()
