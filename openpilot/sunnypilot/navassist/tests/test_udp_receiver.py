import json
import socket
import threading

from openpilot.sunnypilot.navassist.protocol import NavAssistStore
from openpilot.sunnypilot.navassist.server import ClientRateLimiter
from openpilot.sunnypilot.navassist.tests.test_protocol import SOURCE_WALL_MS, encode, payload
from openpilot.sunnypilot.navassist.udp_receiver import MAX_UDP_SNAPSHOT_BYTES, NavAssistUDPServer, UDP_ACK_TYPE


def run_server(*, max_requests: int = 20, ack_payload_provider=None, telemetry_provider=None, lane_decision_provider=None):
  store = NavAssistStore(wall_clock_ms=lambda: SOURCE_WALL_MS)
  server = NavAssistUDPServer(
    ("127.0.0.1", 0), store,
    rate_limiter=ClientRateLimiter(max_requests=max_requests),
    ack_payload_provider=ack_payload_provider,
    telemetry_provider=telemetry_provider,
    lane_decision_provider=lane_decision_provider,
  )
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  return store, server, thread


def stop_server(server: NavAssistUDPServer, thread: threading.Thread) -> None:
  server.shutdown()
  thread.join(timeout=1)
  server.server_close()
  assert not thread.is_alive()


def send(port: int, body: bytes) -> dict | None:
  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
    client.settimeout(0.25)
    client.sendto(body, ("127.0.0.1", port))
    try:
      response, _ = client.recvfrom(1024)
    except TimeoutError:
      return None
  return json.loads(response)


def test_udp_accepts_canonical_v3_snapshot_without_credentials():
  store, server, thread = run_server()
  try:
    response = send(server.server_address[1], encode(payload()))
    assert response == {
      "messageType": UDP_ACK_TYPE,
      "schemaVersion": 3,
      "sessionId": "session-a",
      "sequence": 1,
    }
    assert store.current().snapshot.session_id == "session-a"
    assert server.diagnostics()['accepted'] == 1
    assert server.diagnostics()['received'] == 1
  finally:
    stop_server(server, thread)


def test_udp_ack_can_include_oem_vehicle_lane_feedback():
  _, server, thread = run_server(ack_payload_provider=lambda: {
    "position": "middle", "positionValid": True,
    "leftAllowed": True, "rightAllowed": False,
    "permissionValid": True, "autoLaneChangeState": 6,
    "blindspotValid": True, "leftBlindspot": False, "rightBlindspot": False,
    "radarValid": True, "leadPresent": True, "leadDistanceM": 30.0,
    "leadSpeedKph": 60.0, "leadRelativeSpeedKph": -10.0,
    "vehicleStateValid": True, "egoSpeedKph": 70.0, "lateralActive": True,
    "brakePressed": False, "gasPressed": False, "laneChangeState": 0, "laneChangeDirection": 0,
  })
  try:
    response = send(server.server_address[1], encode(payload()))
    assert response["vehicleLane"] == {
      "position": "middle", "positionValid": True,
      "leftAllowed": True, "rightAllowed": False,
      "permissionValid": True, "autoLaneChangeState": 6,
      "blindspotValid": True, "leftBlindspot": False, "rightBlindspot": False,
      "radarValid": True, "leadPresent": True, "leadDistanceM": 30.0,
      "leadSpeedKph": 60.0, "leadRelativeSpeedKph": -10.0,
      "vehicleStateValid": True, "egoSpeedKph": 70.0, "lateralActive": True,
      "brakePressed": False, "gasPressed": False, "laneChangeState": 0, "laneChangeDirection": 0,
    }
  finally:
    stop_server(server, thread)


def test_udp_silently_drops_replay_malformed_oversize_and_rate_limited_data():
  _, server, thread = run_server(max_requests=2)
  try:
    body = encode(payload())
    assert send(server.server_address[1], body) is not None
    assert send(server.server_address[1], body) is None
    assert send(server.server_address[1], b'{"echo_cmd":"id"}') is None
    assert send(server.server_address[1], b"x" * (MAX_UDP_SNAPSHOT_BYTES + 1)) is None
    stats = server.diagnostics()
    assert stats['received'] == 4 and stats['accepted'] == 1
    assert stats['rejected']['replay'] == 1
    assert stats['rejected']['invalidLength'] == 1
    assert 'session-a' not in json.dumps(stats)
  finally:
    stop_server(server, thread)


def test_extended_ack_preserves_legacy_packet_and_stays_below_mtu():
  from openpilot.sunnypilot.navassist.oem_lane_feedback import OemLaneFeedback
  from openpilot.sunnypilot.navassist.oem_navigation_feedback import OemNavigationFeedback
  _, server, thread = run_server(ack_payload_provider=OemLaneFeedback().snapshot,
                                telemetry_provider=OemNavigationFeedback().snapshot)
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
      client.settimeout(1)
      client.sendto(encode(payload()), server.server_address)
      extended, _ = client.recvfrom(2048)
      legacy, _ = client.recvfrom(2048)
    assert len(extended) <= 1400
    new = json.loads(extended)
    old = json.loads(legacy)
    assert new.pop("vehicleTelemetry")["gpsStatus"] == "missing"
    assert new == old
    assert set(old) == {"messageType", "schemaVersion", "sessionId", "sequence", "vehicleLane"}
  finally:
    stop_server(server, thread)


def test_extended_ack_reports_only_current_session_lane_decision():
  from openpilot.sunnypilot.navassist.oem_navigation_feedback import OemNavigationFeedback
  decision = {'sessionId': 'session-a', 'reason': 'efficiencySafetyBlocked',
              'signalRequested': False, 'direction': 'none'}
  _, server, thread = run_server(telemetry_provider=OemNavigationFeedback().snapshot,
                                lane_decision_provider=lambda: decision)
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
      client.settimeout(1)
      client.sendto(encode(payload()), server.server_address)
      extended, _ = client.recvfrom(2048)
      legacy, _ = client.recvfrom(2048)
    assert len(extended) <= 1400
    assert json.loads(extended)['laneDecision'] == decision
    assert 'laneDecision' not in json.loads(legacy)
  finally:
    stop_server(server, thread)


def test_broken_oversized_or_nonfinite_telemetry_still_delivers_legacy_ack():
  def failed():
    raise ValueError("diagnostic unavailable")
  for provider in (failed, lambda: {"tooBig": "x" * 2000}, lambda: {"latitude": float("nan")}):
    _, server, thread = run_server(telemetry_provider=provider)
    try:
      assert set(send(server.server_address[1], encode(payload()))) == {"messageType", "schemaVersion", "sessionId", "sequence"}
    finally:
      stop_server(server, thread)
