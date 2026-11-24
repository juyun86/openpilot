import http.client
import json

import pytest

from openpilot.sunnypilot.navassist.nav_model_input import ROUTE_PATH, parse_model_route
from openpilot.sunnypilot.navassist.protocol import NavAssistProtocolError
from openpilot.sunnypilot.navassist.tests.test_protocol import APP_KEY_ID, SOURCE_WALL_MS, encode, payload, store


def route(**changes):
  return {"schemaVersion": 3, "messageType": "navigation_route", "sessionId": "session-a", "sequence": 1,
          "routeRevision": 1, "sourceWallTimeMs": SOURCE_WALL_MS, "coordinateSystem": "gcj02",
          "points": [{"latitude": 31.2, "longitude": 121.4}, {"latitude": 31.201, "longitude": 121.401}], **changes}


def test_route_identity_freshness_and_invalidation():
  now = [0]
  receiver = store(clock_ns=lambda: now[0])
  receiver.accept(encode(payload()), APP_KEY_ID)
  receiver.accept_model_route(encode(route()), APP_KEY_ID)
  value = receiver.model_input()
  assert value.route.points == ((31.2, 121.4), (31.201, 121.401))
  assert value.route.coordinate_system == "gcj02"
  assert value.navigation[10] == 1 and value.navigation[-1] == 0.8
  assert len(value.navigation) == 47 and sum(value.navigation[:-1]) == 1
  now[0] = 501_000_000
  assert receiver.model_input() is None
  receiver.accept(encode(payload(sequence=2)), APP_KEY_ID)
  assert receiver.model_input() is None  # Expired route cannot revive after a gap.
  receiver.accept_model_route(encode(route(sequence=2)), APP_KEY_ID)
  receiver.accept(encode(payload(sequence=3, route_revision=2)), APP_KEY_ID)
  assert receiver.model_input() is None
  with pytest.raises(NavAssistProtocolError):
    receiver.accept_model_route(encode(route(sequence=3)), APP_KEY_ID)
  receiver.reset()
  assert receiver.model_input() is None


@pytest.mark.parametrize("maneuver,command", [("straight", 8), ("slight_right", 9), ("turn_right", 10), ("sharp_right", 11),
  ("u_turn_left", 13), ("u_turn_right", 13), ("sharp_left", 14), ("turn_left", 15), ("slight_left", 16),
  ("ramp_right", 18), ("ramp_left", 19), ("exit_right", 20), ("exit_left", 21), ("keep_right", 23), ("keep_left", 24),
  ("roundabout", 26), ("unknown", None), ("merge_left", None), ("merge_right", None), ("destination", None)])
def test_official_instruction_encoding(maneuver, command):
  receiver = store(clock_ns=lambda: 0)
  snapshot = payload()
  snapshot["guidance"]["maneuver"] = maneuver
  receiver.accept(encode(snapshot), APP_KEY_ID)
  receiver.accept_model_route(encode(route()), APP_KEY_ID)
  vector = receiver.model_input().navigation
  assert vector == tuple(1.0 if i == command else 0.8 if i == 46 and command else 0.0 for i in range(47))


@pytest.mark.parametrize("field,value", [("routeMatched", False), ("gpsWeak", True), ("routeActive", False),
                                          ("navigationMode", "simulation")])
def test_ineligible_navigation_discards_geometry(field, value):
  receiver = store(clock_ns=lambda: 0)
  receiver.accept(encode(payload()), APP_KEY_ID)
  receiver.accept_model_route(encode(route()), APP_KEY_ID)
  snapshot = payload(sequence=2)
  snapshot[field] = value
  receiver.accept(encode(snapshot), APP_KEY_ID)
  assert receiver.model_input() is None


def test_missing_distance_and_old_guidance_are_not_turn_commands():
  receiver = store(clock_ns=lambda: 0)
  snapshot = payload()
  del snapshot["guidance"]["maneuverDistanceM"]
  receiver.accept(encode(snapshot), APP_KEY_ID)
  receiver.accept_model_route(encode(route()), APP_KEY_ID)
  assert not any(receiver.model_input().navigation)
  snapshot = payload(sequence=2)
  snapshot["guidance"]["observedAtMs"] -= 501
  receiver.accept(encode(snapshot), APP_KEY_ID)
  assert receiver.model_input() is None


def test_cached_geometry_requires_new_sequence_and_revision_for_changes():
  receiver = store(clock_ns=lambda: 0)
  receiver.accept(encode(payload()), APP_KEY_ID)
  with pytest.raises(NavAssistProtocolError, match="snapshot owner"):
    receiver.accept_model_route(encode(route()), "b" * 32)
  receiver.accept_model_route(encode(route()), APP_KEY_ID)
  assert receiver.model_route_revision() == 1
  with pytest.raises(NavAssistProtocolError, match="sequence must increase"):
    receiver.accept_model_route(encode(route()), APP_KEY_ID)
  receiver.accept(encode(payload(sequence=2)), APP_KEY_ID)
  changed = route(sequence=2, points=[{"latitude": 32, "longitude": 120}, {"latitude": 32.1, "longitude": 120.1}])
  with pytest.raises(NavAssistProtocolError, match="new route revision"):
    receiver.accept_model_route(encode(changed), APP_KEY_ID)
  receiver.accept_model_route(encode(route(sequence=2)), APP_KEY_ID)
  receiver.accept(encode(payload(sequence=3, route_revision=2)), APP_KEY_ID)
  assert receiver.model_route_revision() == -1
  changed.update(sequence=3, routeRevision=2)
  receiver.accept_model_route(encode(changed), APP_KEY_ID)
  assert receiver.model_input().route.points[0] == (32, 120)


@pytest.mark.parametrize("change", [{"sessionId": "other"}, {"routeRevision": 2}, {"sequence": 2},
                                    {"coordinateSystem": "wgs84"}, {"sourceWallTimeMs": SOURCE_WALL_MS - 2001}])
def test_route_rejects_wrong_binding(change):
  receiver = store(clock_ns=lambda: 0)
  receiver.accept(encode(payload()), APP_KEY_ID)
  with pytest.raises(NavAssistProtocolError):
    receiver.accept_model_route(encode(route(**change)), APP_KEY_ID)
  assert receiver.model_input() is None


@pytest.mark.parametrize("change", [{"points": []}, {"points": [{"latitude": float("nan"), "longitude": 1}] * 2},
                                    {"points": [{"latitude": True, "longitude": 1}] * 2},
                                    {"points": [{"latitude": 91, "longitude": 1}] * 2},
                                    {"points": [{"latitude": 1, "longitude": 1}] * 20001},
                                    {"unknown": 1}, {"schemaVersion": True}])
def test_malformed_geometry_is_rejected(change):
  with pytest.raises(NavAssistProtocolError):
    parse_model_route(encode(route(**change)))


def test_route_http_authentication_is_bound_to_path_and_current_owner(tmp_path):
  pytest.importorskip("fcntl", reason="HTTP identity imports C3 Linux hardware modules")
  from openpilot.sunnypilot.navassist.server import KEY_ID_HEADER, SIGNATURE_HEADER, SNAPSHOT_PATH, snapshot_signature_material
  from openpilot.sunnypilot.navassist.tests.test_server import identities, run_server, stop_server
  from openpilot.sunnypilot.navassist.udp_receiver import source_key_id
  device, app, pairing = identities(tmp_path)
  receiver = store(clock_ns=lambda: 0)
  server, thread = run_server(receiver, device, pairing)

  def post(body, key=None, signature=None):
    conn = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
    headers = {"Content-Type": "application/json"}
    if key is not None:
      headers[KEY_ID_HEADER] = key
    if signature is not None:
      headers[SIGNATURE_HEADER] = signature
    conn.request("POST", ROUTE_PATH, body, headers)
    response = conn.getresponse()
    result = response.status, json.loads(response.read())
    conn.close()
    return result

  try:
    body = encode(route())
    assert post(body)[0] == 401
    receiver.accept(encode(payload()), app.device_id)
    wrong = app.sign(snapshot_signature_material(device.device_id, app.device_id, SNAPSHOT_PATH, body))
    assert post(body, app.device_id, wrong)[0] == 401
    good = app.sign(snapshot_signature_material(device.device_id, app.device_id, ROUTE_PATH, body))
    assert post(body, app.device_id, good)[0] == 202
    assert post(body, app.device_id, good)[0] == 409
    receiver.reset()
    receiver.accept(encode(payload()), source_key_id("127.0.0.1"))
    assert post(body, app.device_id, wrong)[0] == 401  # No fallback on failed signed auth.
    assert post(body)[0] == 202
    assert receiver.model_input() is not None
  finally:
    stop_server(server, thread)
