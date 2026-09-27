#!/usr/bin/env python3
"""Narrow local IPC bridge from gateway_configd to card's sole sendcan publisher."""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import logging
import os
import socket
import struct
import threading
import time

from opendbc.car.can_definitions import CanData

GATEWAY_REQUEST_ID = 0x6E0
GATEWAY_RESPONSE_ID = 0x6E1
GATEWAY_BUS = 1
SOCKET_NAME = "\0c3_gateway_config_v1"
UNIX_SOCKET_FAMILY = getattr(socket, "AF_UNIX", 1)
LOG = logging.getLogger(__name__)

BRIDGE_OK = 0
BRIDGE_UNSAFE = 1
BRIDGE_TIMEOUT = 2
BRIDGE_BUSY = 3
BRIDGE_INVALID = 4


def valid_request(data: bytes) -> bool:
  if len(data) != 8:
    return False
  command = data[0]
  if command in (0x01, 0x03, 0x13):
    return not any(data[2:])
  if command == 0x02:
    return data[2] < 48 and not any(data[3:])
  if command == 0x10:
    return data[2] <= 48 and data[3] == 0
  if command == 0x11:
    identifier = int.from_bytes(data[4:6], "little")
    return (data[2] < 48 and data[3] in (2, 3, 4) and identifier <= 0x7FF and
            identifier not in (GATEWAY_REQUEST_ID, GATEWAY_RESPONSE_ID) and
            (data[6] <= 8 or data[6] == 0xFF) and data[7] <= 1)
  if command == 0x12:
    return data[2] == 0 and data[3] == 0
  return False


@dataclass
class _Pending:
  request: bytes
  deadline: float
  event: threading.Event = field(default_factory=threading.Event)
  sent: bool = False
  status: int = BRIDGE_TIMEOUT
  response: bytes = b""


class GatewayConfigBridge:
  """Owns no CAN socket. The card loop calls observe() and next_send()."""

  def __init__(self, socket_name: str = SOCKET_NAME, retry_interval: float = 5.0,
               socket_factory: Callable[..., socket.socket] | None = None,
               clock: Callable[[], float] = time.monotonic):
    self.socket_name = socket_name
    self.retry_interval = retry_interval
    self._socket_factory = socket.socket if socket_factory is None else socket_factory
    self._clock = clock
    self._lock = threading.Lock()
    self._pending: _Pending | None = None
    self._stop = threading.Event()
    self._ready = threading.Event()
    self._closed = False
    self._consecutive_failures = 0
    self._server: socket.socket | None = None
    self._thread: threading.Thread | None = None

  def start(self) -> bool:
    """Start the non-blocking supervisor, or confirm that it is still alive."""
    with self._lock:
      if self._closed:
        return False
      if self._thread is not None and self._thread.is_alive():
        return True

      thread = threading.Thread(target=self._run, name="gateway-config-bridge", daemon=True)
      self._thread = thread
      try:
        thread.start()
      except RuntimeError:
        self._thread = None
        return False
      return True

  @property
  def is_listening(self) -> bool:
    return self._ready.is_set()

  def wait_until_ready(self, timeout: float | None = None) -> bool:
    return self._ready.wait(timeout)

  def _open_server(self) -> socket.socket:
    server = self._socket_factory(UNIX_SOCKET_FAMILY, socket.SOCK_STREAM)
    try:
      server.bind(self.socket_name)
      server.listen(1)
      server.settimeout(0.5)
    except Exception:
      self._close_socket(server)
      raise
    return server

  def close(self) -> None:
    self._stop.set()
    with self._lock:
      self._closed = True
      self._ready.clear()
      server = self._server
      thread = self._thread
      self._finish_locked(BRIDGE_TIMEOUT)
    if server is not None:
      self._close_socket(server)
    if thread is not None and thread is not threading.current_thread():
      thread.join(timeout=1.5)

  def _finish_locked(self, status: int, response: bytes = b"") -> None:
    if self._pending is not None and not self._pending.event.is_set():
      self._pending.status = status
      self._pending.response = response
      self._pending.event.set()

  def submit(self, request: bytes, timeout: float = 0.75) -> tuple[int, bytes]:
    if not valid_request(request):
      return BRIDGE_INVALID, b""
    pending = _Pending(request, self._clock() + max(timeout, 0.0))
    with self._lock:
      if self._closed:
        return BRIDGE_TIMEOUT, b""
      if self._pending is not None:
        return BRIDGE_BUSY, b""
      self._pending = pending
    pending.event.wait(timeout)
    with self._lock:
      if not pending.event.is_set():
        pending.status = BRIDGE_TIMEOUT
      if self._pending is pending:
        self._pending = None
    return pending.status, pending.response

  def observe(self, can_packets: list[tuple[int, list[tuple[int, bytes, int]]]]) -> None:
    with self._lock:
      if self._closed:
        return
      pending = self._pending
      if pending is None or not pending.sent:
        return
      if self._clock() >= pending.deadline:
        self._finish_locked(BRIDGE_TIMEOUT)
        return
      for _, messages in can_packets:
        for address, data, source in messages:
          if (source == GATEWAY_BUS and address == GATEWAY_RESPONSE_ID and len(data) == 8 and
              data[0] == (pending.request[0] | 0x80) and data[1] == pending.request[1]):
            self._finish_locked(BRIDGE_OK, bytes(data))
            return

  def next_send(self, safe: bool) -> list[CanData]:
    with self._lock:
      if self._closed:
        return []
      pending = self._pending
      if pending is None or pending.sent or pending.event.is_set():
        return []
      if self._clock() >= pending.deadline:
        self._finish_locked(BRIDGE_TIMEOUT)
        return []
      if not safe:
        self._finish_locked(BRIDGE_UNSAFE)
        return []
      pending.sent = True
      return [CanData(GATEWAY_REQUEST_ID, pending.request, GATEWAY_BUS)]

  def _serve(self, server: socket.socket) -> None:
    while not self._stop.is_set():
      try:
        connection, _ = server.accept()
      except TimeoutError:
        self._mark_healthy()
        continue
      except OSError:
        if not self._stop.is_set():
          raise
        return
      with connection:
        try:
          connection.settimeout(1)
          if hasattr(socket, "SO_PEERCRED"):
            _, uid, _ = struct.unpack("3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
            if uid != os.getuid():
              continue
          request = b""
          while len(request) < 8:
            part = connection.recv(8 - len(request))
            if not part:
              break
            request += part
          status, response = self.submit(request)
          connection.sendall(bytes([status]) + response.ljust(8, b"\0"))
        except (OSError, TimeoutError):
          pass
      self._mark_healthy()

  @staticmethod
  def _close_socket(server: socket.socket) -> None:
    try:
      server.close()
    except OSError:
      pass

  def _mark_healthy(self) -> None:
    if self._consecutive_failures:
      LOG.info("gateway config bridge recovered after %d consecutive failures", self._consecutive_failures)
      self._consecutive_failures = 0

  def _record_failure(self, error: Exception) -> None:
    self._consecutive_failures += 1
    count = self._consecutive_failures
    if count == 1 or (count & (count - 1)) == 0:
      LOG.warning("gateway config bridge unavailable; retrying in %.1fs (failure %d): %s",
                  self.retry_interval, count, error, exc_info=True)

  def _run(self) -> None:
    current_thread = threading.current_thread()
    try:
      while not self._stop.is_set():
        server = None
        try:
          server = self._open_server()
          with self._lock:
            if self._closed:
              return
            self._server = server
            self._ready.set()
          self._serve(server)
        except Exception as error:
          # A stale abstract socket owner or a transient service failure must not
          # disable card. The supervisor retries after a bounded quiet interval.
          self._record_failure(error)
        finally:
          if server is not None:
            self._close_socket(server)
          with self._lock:
            if self._server is server:
              self._server = None
              self._ready.clear()
            self._finish_locked(BRIDGE_TIMEOUT)

        if self._stop.wait(self.retry_interval):
          break
    finally:
      with self._lock:
        if self._thread is current_thread:
          self._thread = None
