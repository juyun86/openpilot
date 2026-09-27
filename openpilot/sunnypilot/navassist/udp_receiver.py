from __future__ import annotations

import hashlib
import json
import socket
import threading
import time
from collections.abc import Callable

from openpilot.sunnypilot.navassist.protocol import NavAssistProtocolError, NavAssistStore
from openpilot.sunnypilot.navassist.server import ClientRateLimiter


UDP_SNAPSHOT_PORT = 4213
MAX_UDP_SNAPSHOT_BYTES = 8 * 1024
UDP_ACK_TYPE = "navassist_udp_ack"


def source_key_id(source_host: str) -> str:
  """Stable replay namespace for an unauthenticated UDP source address."""
  return hashlib.sha256(b"navassist-udp-v3\0" + source_host.encode("ascii")).hexdigest()[:32]


class NavAssistUDPServer:
  """Bounded, data-only UDP ingress for canonical NavAssist v3 snapshots.

  Authentication is deliberately absent for CP transport compatibility. The
  strict protocol parser, monotonic session/sequence checks, short TTL and rate
  limiter still apply. No command or generic request dispatch exists here.
  """

  def __init__(self, address: tuple[str, int], store: NavAssistStore, *,
               rate_limiter: ClientRateLimiter | None = None,
               ack_payload_provider: Callable[[], dict] | None = None,
               telemetry_provider: Callable[[], dict] | None = None,
               lane_decision_provider: Callable[[], dict] | None = None,
               announcement_provider: Callable[[], dict] | None = None):
    self.store = store
    self.rate_limiter = rate_limiter or ClientRateLimiter()
    self.ack_payload_provider = ack_payload_provider
    self.telemetry_provider = telemetry_provider
    self.lane_decision_provider = lane_decision_provider
    self.announcement_provider = announcement_provider
    self._stopping = threading.Event()
    self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self._socket.bind(address)
    self._socket.settimeout(0.2)
    self._stats_lock = threading.Lock()
    self._stats = {'received': 0, 'accepted': 0, 'rejected': {}, 'last_sender_ip': None, 'last_packet_mono_ns': 0}

  def diagnostics(self) -> dict:
    with self._stats_lock:
      return {**self._stats, 'rejected': dict(self._stats['rejected'])}

  def _reject_record(self, reason: str) -> None:
    with self._stats_lock:
      rejected = self._stats['rejected']
      rejected[reason] = rejected.get(reason, 0) + 1

  @property
  def server_address(self) -> tuple[str, int]:
    host, port = self._socket.getsockname()[:2]
    return str(host), int(port)

  def serve_forever(self) -> None:
    while not self._stopping.is_set():
      try:
        body, address = self._socket.recvfrom(MAX_UDP_SNAPSHOT_BYTES + 1)
      except TimeoutError:
        continue
      except OSError:
        if self._stopping.is_set():
          break
        raise
      source_host = str(address[0])
      with self._stats_lock:
        self._stats['received'] += 1
        self._stats['last_sender_ip'] = source_host
        self._stats['last_packet_mono_ns'] = time.monotonic_ns()
      if not body or len(body) > MAX_UDP_SNAPSHOT_BYTES:
        self._reject_record('invalidLength')
        continue
      if not self.rate_limiter.allow(source_host):
        self._reject_record('rateLimited')
        continue
      try:
        accepted = self.store.accept(body, source_key_id(source_host))
      except NavAssistProtocolError as error:
        self._reject_record(error.reason)
        continue
      with self._stats_lock:
        self._stats['accepted'] += 1
      acknowledgement_payload = {
        "messageType": UDP_ACK_TYPE,
        "schemaVersion": 3,
        "sessionId": accepted.snapshot.session_id,
        "sequence": accepted.snapshot.sequence,
      }
      if self.ack_payload_provider is not None:
        acknowledgement_payload["vehicleLane"] = self.ack_payload_provider()
      acknowledgement = json.dumps(acknowledgement_payload, separators=(",", ":")).encode()
      decision = {}
      if self.lane_decision_provider is not None:
        try:
          decision = self.lane_decision_provider()
          if decision.get('sessionId') != accepted.snapshot.session_id:
            decision = {}
        except Exception:
          self._reject_record('laneDecisionUnavailable')
          decision = {}
      decision_payload = {'laneDecision': decision} if decision else {}
      # Speech is an optional receipt request, not a driving command. Keep the
      # original ACK for old clients; only echo a request into its own session.
      if self.announcement_provider is not None:
        try:
          announcement = self.announcement_provider()
          if (announcement and announcement.get('sessionId') == accepted.snapshot.session_id
              and accepted.snapshot.navigation_mode == 'realtime' and accepted.snapshot.route_active
              and not accepted.is_stale(time.monotonic_ns())):
            extended = json.dumps({**acknowledgement_payload, **decision_payload,
                                   'laneAnnouncement': announcement},
                                  separators=(',', ':'), allow_nan=False).encode()
            if len(extended) <= 1400:
              self._socket.sendto(extended, address)
        except Exception:
          self._reject_record('announcementUnavailable')
      # Legacy Android/iOS reject unknown keys. Send an optional extended ACK
      # first, then the unchanged ACK; diagnostics must never prevent delivery.
      if self.telemetry_provider is not None:
        try:
          extended = json.dumps(
            {**acknowledgement_payload, "vehicleTelemetry": self.telemetry_provider(),
             **decision_payload},
            separators=(",", ":"), allow_nan=False,
          ).encode()
          if len(extended) <= 1400:
            self._socket.sendto(extended, address)
        except Exception:
          self._reject_record('telemetryUnavailable')
      try:
        self._socket.sendto(acknowledgement, address)
      except OSError:
        if self._stopping.is_set():
          break

  def shutdown(self) -> None:
    self._stopping.set()

  def server_close(self) -> None:
    self._socket.close()
