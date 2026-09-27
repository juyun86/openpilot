import http.client
import json
import threading
import urllib.error
import urllib.request

import pytest

from openpilot.selfdrive.debug import device_console
from openpilot.selfdrive.gateway_configd.model import ConfigError
from openpilot.selfdrive.gateway_configd.protocol import ProtocolError


EMPTY_CONFIG = {"version": 2, "rules": []}


class FakeAnalyzer:
  def __init__(self):
    self.provider = None
    self.started = False
    self.stopped = False

  def start(self, provider):
    self.provider = provider
    self.started = True

  def stop(self):
    self.stopped = True


class FakeGatewayService:
  def __init__(self):
    self.lock = threading.RLock()
    self.active = EMPTY_CONFIG
    self.analyzer = FakeAnalyzer()
    self.calls = []
    self.failure = None

  def _record(self, name, *args):
    self.calls.append((name, *args))
    if self.failure is not None:
      raise self.failure

  def state(self):
    self._record("state")
    return {"connected": True, "synchronized": True, "config": EMPTY_CONFIG,
            "previous": None, "digest": "00000000", "message": "ready"}

  def save(self, data):
    self._record("save", data)
    return {"saved": True, "config": data["config"]}

  def analysis(self):
    self._record("analysis")
    return {"available": True, "frames": []}

  def analysis_frame(self, bus, address):
    self._record("analysis_frame", bus, address)
    return {"bus": bus, "address": address}

  def reset_analysis(self):
    self._record("reset_analysis")
    return {"ok": True}


@pytest.fixture
def gateway_server():
  service = FakeGatewayService()
  server = device_console.DeviceConsoleServer(("127.0.0.1", 0), gateway_service=service)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  try:
    yield server, service
  finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def response(server, path, *, data=None, headers=None, method=None):
  request = urllib.request.Request(
    f"http://127.0.0.1:{server.server_port}{path}",
    data=None if data is None else json.dumps(data).encode(),
    headers=headers or ({"Content-Type": "application/json"} if data is not None else {}),
    method=method,
  )
  try:
    result = urllib.request.urlopen(request, timeout=2)
  except urllib.error.HTTPError as error:
    result = error
  with result:
    return result.status, result.headers, result.read()


def json_response(server, path, **kwargs):
  status, _, body = response(server, path, **kwargs)
  return status, json.loads(body)


def host_response(server, host, *, origin=None):
  connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
  headers = {"Host": host}
  if origin is not None:
    headers["Origin"] = origin
  connection.request("GET", "/api/gateway/analysis", headers=headers)
  result = connection.getresponse()
  status, body = result.status, json.loads(result.read())
  connection.close()
  return status, body


def test_homepage_links_to_gateway_without_embedding_it(gateway_server):
  server, _ = gateway_server
  status, _, body = response(server, "/")

  assert status == 200
  assert b'id="gateway-link" href="/gateway/"' in body
  assert b"showPanel('gateway')" not in body
  assert b"<iframe" not in body


@pytest.mark.parametrize("path,name,content_type", (
  ("/gateway/", "index.html", "text/html"),
  ("/gateway/app.js?v=test", "app.js", "text/javascript"),
  ("/gateway/style.css?v=test", "style.css", "text/css"),
))
def test_gateway_static_assets_are_served_from_8088(gateway_server, path, name, content_type):
  server, _ = gateway_server
  status, headers, body = response(server, path)

  assert status == 200
  assert headers.get_content_type() == content_type
  assert body == (device_console.GATEWAY_STATIC / name).read_bytes()


def test_gateway_get_routes_use_shared_service(gateway_server):
  server, service = gateway_server

  state_status, state = json_response(server, "/api/gateway/state")
  analysis_status, analysis = json_response(server, "/api/gateway/analysis")
  frame_status, frame = json_response(server, "/api/gateway/analysis/frame?bus=1&id=0x399")

  assert (state_status, state["connected"]) == (200, True)
  assert (analysis_status, analysis["available"]) == (200, True)
  assert (frame_status, frame) == (200, {"bus": 1, "address": 0x399})
  assert ("analysis_frame", 1, 0x399) in service.calls


def test_gateway_post_routes_validate_save_and_reset(gateway_server):
  server, service = gateway_server

  validate_status, validated = json_response(server, "/api/gateway/validate", data=EMPTY_CONFIG, method="POST")
  save_payload = {"config": EMPTY_CONFIG, "expectedDigest": validated["digest"]}
  save_status, saved = json_response(server, "/api/gateway/save", data=save_payload, method="POST")
  reset_status, reset = json_response(server, "/api/gateway/analysis/reset", data={}, method="POST")

  assert validate_status == save_status == reset_status == 200
  assert validated["config"] == EMPTY_CONFIG
  assert saved["saved"] is True
  assert reset == {"ok": True}
  assert ("save", save_payload) in service.calls
  assert ("reset_analysis",) in service.calls


@pytest.mark.parametrize("host", ("localhost", "127.0.0.1", "192.168.43.1", "10.42.0.1",
                                         "172.16.0.1", "169.254.8.1", "[::1]", "[fe80::1]", "99.99.99.99"))
def test_gateway_accepts_expected_local_hosts(gateway_server, host):
  server, _ = gateway_server
  netloc = f"{host}:{server.server_port}"

  assert host_response(server, netloc)[0] == 200
  assert host_response(server, netloc, origin=f"http://{netloc}")[0] == 200


@pytest.mark.parametrize("host", ("8.8.8.8", "example.com", "0.0.0.0", "192.0.2.1"))
def test_gateway_rejects_non_local_or_named_hosts(gateway_server, host):
  server, _ = gateway_server
  status, _ = host_response(server, f"{host}:{server.server_port}")
  assert status == 403


def test_gateway_rejects_wrong_port_and_cross_site_origin(gateway_server):
  server, _ = gateway_server
  host = f"127.0.0.1:{server.server_port}"

  assert host_response(server, "127.0.0.1:8080")[0] == 403
  assert host_response(server, host, origin="http://127.0.0.1:9999")[0] == 403


def test_gateway_rejects_missing_or_duplicate_host(gateway_server):
  server, _ = gateway_server
  for hosts in ((), (f"127.0.0.1:{server.server_port}", f"localhost:{server.server_port}")):
    connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
    connection.putrequest("GET", "/api/gateway/analysis", skip_host=True)
    for host in hosts:
      connection.putheader("Host", host)
    connection.endheaders()
    result = connection.getresponse()
    assert result.status == 403
    result.read()
    connection.close()


def test_gateway_post_requires_json_and_one_bounded_content_length(gateway_server):
  server, service = gateway_server
  status, payload = json_response(
    server, "/api/gateway/validate", data=EMPTY_CONFIG,
    headers={"Content-Type": "text/plain"}, method="POST",
  )
  assert status == 400
  assert payload["error"] == "网关配置或请求内容无效"

  connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
  body = b"{}"
  connection.putrequest("POST", "/api/gateway/analysis/reset")
  connection.putheader("Content-Type", "application/json")
  connection.putheader("Content-Length", str(len(body)))
  connection.putheader("Content-Length", str(len(body)))
  connection.endheaders(body)
  result = connection.getresponse()
  assert result.status == 400
  result.read()
  connection.close()

  connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
  connection.putrequest("POST", "/api/gateway/analysis/reset")
  connection.putheader("Content-Type", "application/json")
  connection.putheader("Content-Length", str(device_console.MAX_GATEWAY_BODY + 1))
  connection.endheaders()
  result = connection.getresponse()
  assert result.status == 400
  result.read()
  connection.close()
  assert ("reset_analysis",) not in service.calls


@pytest.mark.parametrize("failure,status", (
  (ConfigError("sensitive validation detail"), 400),
  (ProtocolError("sensitive protocol detail"), 409),
  (OSError("sensitive path detail"), 503),
  (RuntimeError("sensitive traceback detail"), 500),
))
def test_gateway_route_errors_are_fixed_and_do_not_leak(gateway_server, failure, status):
  server, service = gateway_server
  service.failure = failure

  actual_status, payload = json_response(server, "/api/gateway/state")

  assert actual_status == status
  assert "sensitive" not in payload["error"]


def test_gateway_start_failure_leaves_homepage_available(monkeypatch):
  class BrokenGatewayService:
    def __init__(self, _state_dir):
      raise OSError("broken state directory")

  monkeypatch.setattr(device_console, "GatewayConfigService", BrokenGatewayService)
  assert device_console.start_gateway_service() is None

  server = device_console.DeviceConsoleServer(("127.0.0.1", 0), gateway_service=None)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  try:
    assert response(server, "/")[0] == 200
    gateway_status, payload = json_response(server, "/api/gateway/state")
    assert gateway_status == 503
    assert payload["error"] == "网关服务暂不可用"
  finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_gateway_partial_start_is_stopped(monkeypatch):
  service = FakeGatewayService()

  def fail_start(_provider):
    raise RuntimeError("collector failed")

  service.analyzer.start = fail_start
  monkeypatch.setattr(device_console, "GatewayConfigService", lambda _state_dir: service)

  assert device_console.start_gateway_service() is None
  assert service.analyzer.stopped is True


def test_device_console_rejects_requests_above_the_worker_limit_as_busy():
  assert device_console.MAX_REQUEST_THREADS == 8
  entered = threading.Event()
  release = threading.Event()

  class BlockingGatewayService(FakeGatewayService):
    def state(self):
      entered.set()
      assert release.wait(2)
      return super().state()

  service = BlockingGatewayService()
  server = device_console.DeviceConsoleServer(("127.0.0.1", 0), gateway_service=service, max_workers=1)
  serving = threading.Thread(target=server.serve_forever, daemon=True)
  serving.start()
  first_result = []

  def first_request():
    first_result.append(json_response(server, "/api/gateway/state"))

  first = threading.Thread(target=first_request)
  first.start()
  try:
    assert entered.wait(2)
    busy_status, busy = json_response(server, "/api/gateway/analysis")
    assert busy_status == 503
    assert busy["error"] == "服务繁忙，请稍后重试"
  finally:
    release.set()
    first.join(timeout=2)
    server.shutdown()
    server.server_close()
    serving.join(timeout=2)

  assert first_result[0][0] == 200
