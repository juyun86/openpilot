from __future__ import annotations

from collections import defaultdict, deque
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
from urllib.parse import parse_qs, urlsplit

from openpilot.sunnypilot.navassist.identity import NavAssistDeviceIdentity, NavAssistPairingStore, verify_signature
from openpilot.sunnypilot.navassist.protocol import MAX_BODY_BYTES, NavAssistProtocolError, NavAssistStore
from openpilot.sunnypilot.navassist.nav_model_input import MAX_ROUTE_BYTES, ROUTE_PATH


SNAPSHOT_PATH = "/v3/snapshot"
KEY_ID_HEADER = "X-NavAssist-Key-Id"
SIGNATURE_HEADER = "X-NavAssist-Signature"


def snapshot_signature_material(device_id: str, app_key_id: str, path: str, body: bytes) -> bytes:
  prefix = f"navassist_snapshot\n3\nPOST\n{path}\n{device_id}\n{app_key_id}\n{len(body)}\n".encode("ascii")
  return prefix + body


class ClientRateLimiter:
  def __init__(self, *, max_requests: int = 20, window_s: float = 1.0,
               clock: Callable[[], float] = time.monotonic):
    self.max_requests = max_requests
    self.window_s = window_s
    self._clock = clock
    self._events: dict[str, deque[float]] = defaultdict(deque)
    self._lock = threading.Lock()

  def allow(self, client: str) -> bool:
    now = self._clock()
    with self._lock:
      events = self._events[client]
      while events and now - events[0] >= self.window_s:
        events.popleft()
      if len(events) >= self.max_requests:
        return False
      events.append(now)
      return True


class NavAssistHTTPServer(ThreadingHTTPServer):
  daemon_threads = True
  allow_reuse_address = True
  request_queue_size = 4

  def __init__(self, address: tuple[str, int], store: NavAssistStore, identity: NavAssistDeviceIdentity,
               pairing: NavAssistPairingStore, *, rate_limiter: ClientRateLimiter | None = None,
               max_concurrent_requests: int = 4, diagnostics_provider=None):
    if not 1 <= max_concurrent_requests <= 16:
      raise ValueError("max_concurrent_requests must be in [1, 16]")
    self.store = store
    self.identity = identity
    self.pairing = pairing
    self.diagnostics_provider = diagnostics_provider
    self.rate_limiter = rate_limiter or ClientRateLimiter()
    self._request_slots = threading.BoundedSemaphore(max_concurrent_requests)
    super().__init__(address, NavAssistRequestHandler)

  def process_request(self, request, client_address) -> None:
    # ThreadingMixIn otherwise creates an unbounded thread per accepted socket.
    if not self._request_slots.acquire(blocking=False):
      self.shutdown_request(request)
      return
    try:
      super().process_request(request, client_address)
    except BaseException:
      self._request_slots.release()
      raise

  def process_request_thread(self, request, client_address) -> None:
    try:
      super().process_request_thread(request, client_address)
    finally:
      self._request_slots.release()


class NavAssistRequestHandler(BaseHTTPRequestHandler):
  server: NavAssistHTTPServer

  def setup(self) -> None:
    super().setup()
    self.connection.settimeout(2.0)

  def log_message(self, format_string: str, *args) -> None:
    # Never include navigation payloads, tokens, or signatures in HTTP access logs.
    pass

  def _respond(self, status: int, payload: dict) -> None:
    body = json.dumps(payload, separators=(",", ":")).encode()
    self.send_response(status)
    self.send_header("Content-Type", "application/json")
    self.send_header("Content-Length", str(len(body)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    self.wfile.write(body)

  def do_POST(self) -> None:
    if self.path not in (SNAPSHOT_PATH, ROUTE_PATH):
      self._respond(404, {"accepted": False, "reason": "not_found"})
      return
    if not self.server.rate_limiter.allow(self.client_address[0]):
      self._respond(429, {"accepted": False, "reason": "rate_limited"})
      return
    if self.headers.get_content_type() != "application/json":
      self._respond(415, {"accepted": False, "reason": "content_type"})
      return
    if self.headers.get("Transfer-Encoding") is not None:
      self._respond(400, {"accepted": False, "reason": "chunked_not_supported"})
      return
    try:
      content_length = int(self.headers.get("Content-Length", "-1"))
    except ValueError:
      content_length = -1
    if not 0 < content_length <= (MAX_ROUTE_BYTES if self.path == ROUTE_PATH else MAX_BODY_BYTES):
      self._respond(413, {"accepted": False, "reason": "body_size"})
      return
    body = self.rfile.read(content_length)
    app_key_id = self.headers.get(KEY_ID_HEADER)
    app_public_key = self.server.pairing.public_key_for(app_key_id) if app_key_id is not None else None
    signature = self.headers.get(SIGNATURE_HEADER)
    if self.path == ROUTE_PATH and not app_key_id and not signature:
      # Reuse the existing data-only UDP source boundary. Store checks ownership
      # and session atomically; malformed supplied credentials never fall back.
      from openpilot.sunnypilot.navassist.udp_receiver import source_key_id
      app_key_id = source_key_id(self.client_address[0])
    elif app_public_key is None or signature is None or not verify_signature(
      app_public_key,
      snapshot_signature_material(self.server.identity.device_id, app_key_id, self.path, body),
      signature,
    ):
      self._respond(401, {"accepted": False, "reason": "authentication"})
      return
    try:
      if self.path == ROUTE_PATH:
        route = self.server.store.accept_model_route(body, app_key_id)
        self._respond(202, {"accepted": True, "sessionId": route.session_id,
                            "sequence": route.sequence, "routeRevision": route.route_revision})
        return
      accepted = self.server.store.accept(body, app_key_id)
    except NavAssistProtocolError as error:
      status = 401 if error.reason == "authentication" else 409 if error.reason == "replay" else 400
      self._respond(status, {"accepted": False, "reason": error.reason})
      return
    self._respond(202, {
      "accepted": True,
      "sessionId": accepted.snapshot.session_id,
      "sequence": accepted.snapshot.sequence,
      "routeRevision": accepted.snapshot.route_revision,
    })

  def do_GET(self) -> None:
    url = urlsplit(self.path)
    if url.path == '/v3/diagnostics' and self.server.diagnostics_provider is not None:
      if not self.server.rate_limiter.allow(self.client_address[0]):
        self._respond(429, {'reason': 'rate_limited'})
        return
      query = parse_qs(url.query)
      current = self.server.store.current()
      # Same data-only UDP owner/session boundary, not a new control capability.
      from openpilot.sunnypilot.navassist.udp_receiver import source_key_id
      if (current is None or current.is_stale(time.monotonic_ns())
          or query.get('sessionId') != [current.snapshot.session_id]
          or not self.server.store.owns_source(source_key_id(self.client_address[0]))):
        self._respond(403, {'reason': 'session_owner'})
        return
      try:
        after = int(query.get('after', ['0'])[0])
        if after < 0 or len(query.get('after', ['0'])) != 1:
          raise ValueError()
      except ValueError:
        self._respond(400, {'reason': 'cursor'})
        return
      self._respond(200, self.server.diagnostics_provider(after))
      return
    self._respond(405, {"accepted": False, "reason": "method_not_allowed"})
