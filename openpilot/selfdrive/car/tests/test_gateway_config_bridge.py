import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

from opendbc.car.can_definitions import CanData
from opendbc.car import structs
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.gateway_config_bridge import (BRIDGE_BUSY, BRIDGE_INVALID, BRIDGE_OK,
                                                            BRIDGE_TIMEOUT, BRIDGE_UNSAFE,
                                                            GATEWAY_RESPONSE_ID, GatewayConfigBridge,
                                                            _Pending, valid_request)


def valid_info(transaction=7):
  return bytes([1, transaction, 0, 0, 0, 0, 0, 0])


def submit_async(bridge, request, timeout=0.2):
  result = []
  thread = threading.Thread(target=lambda: result.append(bridge.submit(request, timeout=timeout)))
  thread.start()
  for _ in range(100):
    with bridge._lock:
      if bridge._pending is not None:
        break
    time.sleep(0.001)
  return thread, result


def wait_for(predicate, timeout=0.5):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if predicate():
      return True
    time.sleep(0.001)
  return predicate()


class FakeServer:
  def __init__(self, bind_error=False, accept_error=False):
    self.bind_error = bind_error
    self.accept_error = accept_error
    self.listening = threading.Event()
    self.closed = threading.Event()
    self.accept_calls = 0

  def bind(self, _socket_name):
    if self.bind_error:
      raise OSError("address in use")

  def listen(self, _backlog):
    self.listening.set()

  def settimeout(self, _timeout):
    pass

  def accept(self):
    self.accept_calls += 1
    if self.accept_error:
      self.accept_error = False
      raise RuntimeError("unexpected service failure")
    self.closed.wait(0.005)
    if self.closed.is_set():
      raise OSError("closed")
    raise TimeoutError

  def close(self):
    self.closed.set()


class FakeSocketFactory:
  def __init__(self, servers):
    self.servers = list(servers)
    self.calls = 0
    self.lock = threading.Lock()

  def __call__(self, _family, _kind):
    with self.lock:
      index = self.calls
      self.calls += 1
    if index >= len(self.servers):
      raise AssertionError("unexpected socket creation")
    return self.servers[index]


class FakeCardBridge:
  def __init__(self):
    self.safe_values = []

  def next_send(self, safe):
    self.safe_values.append(safe)
    return [CanData(0x6E0, b"12345678", 1)] if safe else []


class TestCardGatewayConfigPath(unittest.TestCase):
  def test_parked_config_send_does_not_depend_on_onroad_events(self):
    bridge = FakeCardBridge()
    fake = SimpleNamespace(
      gateway_config_bridge=bridge,
      sm=SimpleNamespace(all_alive=lambda services: services == ['carControl']),
      pm=SimpleNamespace(send=mock.Mock()),
    )
    cs = SimpleNamespace(canValid=True, standstill=True, vEgo=0.0,
                         gearShifter=structs.CarState.GearShifter.park)
    cc = SimpleNamespace(enabled=False, latActive=False, longActive=False)
    with mock.patch("openpilot.selfdrive.car.card.can_list_to_can_capnp", return_value="packet"):
      Car.gateway_config_update(fake, cs, cc)
    self.assertEqual(bridge.safe_values, [True])
    fake.pm.send.assert_called_once_with('sendcan', 'packet')

  def test_config_send_rejects_each_unsafe_state(self):
    cases = (
      {"alive": False}, {"canValid": False}, {"standstill": False}, {"vEgo": 0.1},
      {"gearShifter": structs.CarState.GearShifter.drive},
      {"enabled": True}, {"latActive": True}, {"longActive": True},
    )
    for changed in cases:
      with self.subTest(changed=changed):
        bridge = FakeCardBridge()
        alive = changed.pop("alive", True)
        fake = SimpleNamespace(
          gateway_config_bridge=bridge,
          sm=SimpleNamespace(all_alive=lambda _services, value=alive: value),
          pm=SimpleNamespace(send=mock.Mock()),
        )
        cs_values = {"canValid": True, "standstill": True, "vEgo": 0.0,
                     "gearShifter": structs.CarState.GearShifter.park}
        cc_values = {"enabled": False, "latActive": False, "longActive": False}
        (cs_values if set(changed) & set(cs_values) else cc_values).update(changed)
        Car.gateway_config_update(fake, SimpleNamespace(**cs_values), SimpleNamespace(**cc_values))
        self.assertEqual(bridge.safe_values, [False])
        fake.pm.send.assert_not_called()


class TestGatewayConfigBridge(unittest.TestCase):

  def test_request_validation(self):
    self.assertTrue(valid_request(valid_info()))
    self.assertTrue(valid_request(bytes([0x11, 1, 47, 4, 0xFF, 7, 0xFF, 1])))
    self.assertFalse(valid_request(b""))
    self.assertFalse(valid_request(bytes([1, 1, 0, 0, 0, 0, 0, 1])))
    self.assertFalse(valid_request(bytes([0x11, 1, 48, 4, 1, 0, 8, 1])))
    self.assertFalse(valid_request(bytes([0x11, 1, 0, 4, 0xE0, 6, 8, 1])))

  def test_safe_send_and_matched_response(self):
    bridge = GatewayConfigBridge()
    request = valid_info()
    thread, result = submit_async(bridge, request)
    self.assertEqual(bridge.next_send(True), [CanData(0x6E0, request, 1)])
    self.assertEqual(bridge.next_send(True), [])
    bridge.observe([(1, [(GATEWAY_RESPONSE_ID, bytes([0x81, 8, 0, 0, 0, 0, 0, 0]), 1)])])
    self.assertTrue(thread.is_alive())
    reply = bytes([0x81, 7, 1, 20, 1, 2, 3, 4])
    bridge.observe([(2, [(GATEWAY_RESPONSE_ID, reply, 1)])])
    thread.join()
    self.assertEqual(result, [(BRIDGE_OK, reply)])

  def test_unsafe_busy_invalid_and_timeout(self):
    bridge = GatewayConfigBridge()
    thread, result = submit_async(bridge, valid_info())
    self.assertEqual(bridge.submit(valid_info(8), timeout=0), (BRIDGE_BUSY, b""))
    self.assertEqual(bridge.next_send(False), [])
    thread.join()
    self.assertEqual(result, [(BRIDGE_UNSAFE, b"")])
    self.assertEqual(bridge.submit(b"bad", timeout=0), (BRIDGE_INVALID, b""))
    self.assertEqual(bridge.submit(valid_info(), timeout=0), (BRIDGE_TIMEOUT, b""))

  def test_unsafe_completed_request_cannot_send_after_state_turns_safe(self):
    bridge = GatewayConfigBridge()
    pending = _Pending(valid_info(), time.monotonic() + 1.0)
    bridge._pending = pending

    self.assertEqual(bridge.next_send(False), [])
    self.assertTrue(pending.event.is_set())
    self.assertEqual(pending.status, BRIDGE_UNSAFE)
    self.assertEqual(bridge.next_send(True), [])
    self.assertFalse(pending.sent)

  def test_expired_unsent_request_never_reaches_can(self):
    now = [10.0]
    bridge = GatewayConfigBridge(clock=lambda: now[0])
    thread, result = submit_async(bridge, valid_info(), timeout=30.0)

    now[0] = 40.0
    self.assertEqual(bridge.next_send(True), [])
    thread.join(0.5)

    self.assertFalse(thread.is_alive())
    self.assertEqual(result, [(BRIDGE_TIMEOUT, b"")])

  def test_response_at_deadline_is_rejected(self):
    now = [10.0]
    bridge = GatewayConfigBridge(clock=lambda: now[0])
    request = valid_info()
    thread, result = submit_async(bridge, request, timeout=30.0)
    self.assertEqual(bridge.next_send(True), [CanData(0x6E0, request, 1)])

    now[0] = 40.0
    reply = bytes([0x81, 7, 1, 20, 1, 2, 3, 4])
    bridge.observe([(1, [(GATEWAY_RESPONSE_ID, reply, 1)])])
    thread.join(0.5)

    self.assertFalse(thread.is_alive())
    self.assertEqual(result, [(BRIDGE_TIMEOUT, b"")])

  def test_supervisor_retries_bind_without_blocking_start(self):
    unavailable = FakeServer(bind_error=True)
    available = FakeServer()
    factory = FakeSocketFactory([unavailable, available])
    bridge = GatewayConfigBridge(retry_interval=0.001, socket_factory=factory)

    with self.assertLogs("openpilot.selfdrive.car.gateway_config_bridge", level="INFO") as logs:
      try:
        self.assertTrue(bridge.start())
        self.assertTrue(bridge.wait_until_ready(0.5))
        self.assertTrue(available.listening.is_set())
        self.assertEqual(factory.calls, 2)
        self.assertTrue(unavailable.closed.is_set())
        self.assertTrue(wait_for(lambda: bridge._consecutive_failures == 0))
      finally:
        bridge.close()
    self.assertTrue(any("failure 1" in message for message in logs.output))
    self.assertTrue(any("recovered after 1" in message for message in logs.output))

  def test_start_is_idempotent_and_close_prevents_restart(self):
    server = FakeServer()
    factory = FakeSocketFactory([server])
    bridge = GatewayConfigBridge(retry_interval=0.001, socket_factory=factory)

    results = []
    starters = [threading.Thread(target=lambda: results.append(bridge.start())) for _ in range(8)]
    for starter in starters:
      starter.start()
    for starter in starters:
      starter.join()
    self.assertTrue(server.listening.wait(0.5))
    self.assertTrue(bridge.is_listening)
    self.assertEqual(results, [True] * 8)
    self.assertEqual(factory.calls, 1)

    bridge.close()
    self.assertFalse(bridge.start())
    self.assertFalse(bridge.is_listening)
    self.assertTrue(server.closed.is_set())
    self.assertTrue(wait_for(lambda: bridge._thread is None))

  def test_supervisor_recovers_after_unexpected_service_failure(self):
    failed = FakeServer(accept_error=True)
    recovered = FakeServer()
    factory = FakeSocketFactory([failed, recovered])
    bridge = GatewayConfigBridge(retry_interval=0.001, socket_factory=factory)

    with self.assertLogs("openpilot.selfdrive.car.gateway_config_bridge", level="INFO") as logs:
      try:
        self.assertTrue(bridge.start())
        self.assertTrue(recovered.listening.wait(0.5))
        self.assertEqual(factory.calls, 2)
        self.assertTrue(failed.closed.is_set())
        self.assertTrue(wait_for(lambda: bridge._consecutive_failures == 0))
      finally:
        bridge.close()
    self.assertTrue(any("unexpected service failure" in message for message in logs.output))
    self.assertTrue(any("recovered after 1" in message for message in logs.output))

  def test_repeated_failures_use_exponential_log_sampling(self):
    unavailable = [FakeServer(bind_error=True) for _ in range(5)]
    available = FakeServer()
    factory = FakeSocketFactory([*unavailable, available])
    bridge = GatewayConfigBridge(retry_interval=0.001, socket_factory=factory)

    with self.assertLogs("openpilot.selfdrive.car.gateway_config_bridge", level="INFO") as logs:
      try:
        self.assertTrue(bridge.start())
        self.assertTrue(bridge.wait_until_ready(0.5))
        self.assertTrue(wait_for(lambda: bridge._consecutive_failures == 0))
      finally:
        bridge.close()

    failures = [message for message in logs.output if "bridge unavailable" in message]
    self.assertEqual(len(failures), 3)
    self.assertIn("failure 1", failures[0])
    self.assertIn("failure 2", failures[1])
    self.assertIn("failure 4", failures[2])
    self.assertEqual(sum("recovered after 5" in message for message in logs.output), 1)

  def test_close_unblocks_pending_request(self):
    bridge = GatewayConfigBridge()
    thread, result = submit_async(bridge, valid_info())

    bridge.close()
    thread.join(0.5)
    self.assertFalse(thread.is_alive())
    self.assertEqual(result, [(BRIDGE_TIMEOUT, b"")])


if __name__ == "__main__":
  unittest.main()
