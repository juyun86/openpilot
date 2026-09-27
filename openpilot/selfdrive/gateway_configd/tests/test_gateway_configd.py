from __future__ import annotations

import json
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest import mock

from openpilot.selfdrive.car.gateway_config_bridge import GATEWAY_RESPONSE_ID, GatewayConfigBridge
from openpilot.selfdrive.gateway_configd.model import MAX_RULES, VERSION, ConfigError, decode, defaults, digest, encode, encode_rule, validate
from openpilot.selfdrive.gateway_configd.protocol import (ABORT, BEGIN, COMMIT, INFO, READ, RULE, STATUS, Client,
                                                          ProtocolError, UnixExchange, packet)
from openpilot.selfdrive.gateway_configd.service import GatewayConfigService


class MemoryGateway:
  def __init__(self, active=None):
    self.active = validate(active or defaults())
    self.stage = None
    self.synchronized = True

  def reply(self, request, status=0):
    return packet(request[0] | 0x80, request[1], bytes([status, 0]) + struct.pack("<I", digest(self.active)))

  def __call__(self, request):
    command, transaction = request[:2]
    if command == INFO:
      return packet(0x81, transaction, bytes([VERSION, len(self.active["rules"])]) + struct.pack("<I", digest(self.active)))
    if command == STATUS:
      return packet(0x83, transaction, bytes([int(self.synchronized), int(self.stage is not None)]) + struct.pack("<I", digest(self.active)))
    if command == READ and request[2] < len(self.active["rules"]):
      return packet(0x82, transaction, bytes([request[2]]) + encode_rule(self.active["rules"][request[2]]))
    if command == BEGIN:
      if struct.unpack_from("<I", request, 4)[0] != digest(self.active):
        return self.reply(request, 3)
      self.stage = {"transaction": transaction, "count": request[2], "rules": {}}
      return self.reply(request)
    if command == RULE and self.stage is not None and transaction == self.stage["transaction"]:
      self.stage["rules"][request[2]] = request[3:]
      return self.reply(request)
    if command == COMMIT and self.stage is not None and transaction == self.stage["transaction"]:
      raw = bytes([self.stage["count"]]) + b"".join(self.stage["rules"][index] for index in range(self.stage["count"]))
      wanted = decode(raw + bytes((MAX_RULES - self.stage["count"]) * 5))
      if digest(wanted) != struct.unpack_from("<I", request, 4)[0]:
        return self.reply(request, 5)
      self.active, self.stage = wanted, None
      return self.reply(request)
    if command == ABORT:
      self.stage = None
      return self.reply(request)
    return self.reply(request, 1)


class TestModelAndProtocol(unittest.TestCase):
  def test_defaults_round_trip_and_validation(self):
    config = defaults()
    self.assertEqual(decode(encode(config)), config)
    self.assertEqual(len(config["rules"]), 20)
    duplicate = {"version": 2, "rules": [config["rules"][0], config["rules"][0]]}
    with self.assertRaises(ConfigError):
      validate(duplicate)

  def test_read_save_and_stale_rejection(self):
    gateway = MemoryGateway()
    client = Client(gateway, 9)
    before, synchronized = client.read()
    self.assertTrue(synchronized)
    changed = {"version": 2, "rules": before["rules"] + [{"source": 4, "id": "0x456", "dlc": 8, "enabled": True}]}
    self.assertEqual(client.save(changed, digest(before)), changed)
    with self.assertRaises(ProtocolError):
      client.save(before, digest(before))
    self.assertIsNone(gateway.stage)

  def test_full_48_rule_round_trip(self):
    config = {"version": VERSION, "rules": [
      {"source": 2 + index % 3, "id": f"0x{0x400 + index:03X}", "dlc": "any", "enabled": True}
      for index in range(MAX_RULES)
    ]}
    gateway = MemoryGateway()
    self.assertEqual(Client(gateway, 19).save(config, digest(defaults())), config)

  def test_service_cache_and_offline_state(self):
    with tempfile.TemporaryDirectory() as directory:
      gateway = MemoryGateway()
      service = GatewayConfigService(Path(directory), gateway)
      state = service.state()
      self.assertTrue(state["connected"])
      changed = {"version": 2, "rules": state["config"]["rules"][:1]}
      saved = service.save({"config": changed, "expectedDigest": state["digest"]})
      self.assertEqual(saved["config"], changed)
      self.assertEqual(saved["previous"], defaults())
      offline = GatewayConfigService(Path(directory), lambda _request: None).state()
      self.assertFalse(offline["connected"])
      self.assertEqual(offline["config"], changed)

  def test_unix_bridge_round_trip_is_rate_limited(self):
    socket_name = f"\0gateway-config-test-{time.monotonic_ns()}"
    bridge = GatewayConfigBridge(socket_name)
    gateway = MemoryGateway()
    request_times = []
    stop = threading.Event()

    def card_loop():
      while not stop.is_set():
        messages = bridge.next_send(True)
        if messages:
          request_times.append(time.monotonic())
          response = gateway(bytes(messages[0].dat))
          bridge.observe([(0, [(GATEWAY_RESPONSE_ID, response, messages[0].src)])])
        time.sleep(0.001)

    self.assertTrue(bridge.start())
    self.assertTrue(bridge.wait_until_ready(1.0))
    thread = threading.Thread(target=card_loop, daemon=True)
    thread.start()
    try:
      config, synchronized = Client(UnixExchange(socket_name), 41).read()
      self.assertEqual(config, defaults())
      self.assertTrue(synchronized)
      self.assertGreater(len(request_times), 20)
      self.assertGreaterEqual(min(b - a for a, b in zip(request_times, request_times[1:], strict=False)), 0.024)
    finally:
      stop.set()
      thread.join()
      bridge.close()


class RecordingAnalyzer:
  def __init__(self):
    self.reset_count = 0
    self.snapshots = []
    self.frames = []

  def reset(self):
    self.reset_count += 1

  def snapshot(self, config):
    self.snapshots.append(config)
    return {"config": config}

  def frame_detail(self, config, bus, address):
    self.frames.append((config, bus, address))
    return {"bus": bus, "address": address}


class TestGatewayConfigService(unittest.TestCase):
  def test_corrupt_cache_falls_back_to_defaults(self):
    invalid_caches = (
      "not json",
      json.dumps({"active": {"version": VERSION, "rules": "invalid"}}),
      json.dumps({"previous": defaults()}),
    )
    for contents in invalid_caches:
      with self.subTest(contents=contents), tempfile.TemporaryDirectory() as directory:
        (Path(directory) / "cache.json").write_text(contents, encoding="utf-8")
        service = GatewayConfigService(Path(directory), MemoryGateway())
        self.assertEqual(service.active, defaults())
        self.assertIsNone(service.previous)

  def test_persist_failure_keeps_hardware_truth_in_memory(self):
    with tempfile.TemporaryDirectory() as directory:
      analyzer = RecordingAnalyzer()
      service = GatewayConfigService(Path(directory), MemoryGateway(), analyzer)
      before = service.active
      changed = {"version": VERSION, "rules": before["rules"][:1]}
      with mock.patch("openpilot.selfdrive.gateway_configd.service.atomic_json", side_effect=OSError("disk full")):
        self.assertFalse(service.persist(changed, before))
      self.assertEqual(service.active, changed)
      self.assertEqual(service.previous, before)
      self.assertFalse(service.cache_persisted)
      self.assertEqual(analyzer.reset_count, 1)

  def test_save_reports_applied_configuration_when_cache_write_fails(self):
    with tempfile.TemporaryDirectory() as directory:
      analyzer = RecordingAnalyzer()
      service = GatewayConfigService(Path(directory), MemoryGateway(), analyzer)
      changed = {"version": VERSION, "rules": service.active["rules"][:1]}
      expected = f"{digest(service.active):08X}"
      with mock.patch("openpilot.selfdrive.gateway_configd.service.atomic_json", side_effect=OSError("disk full")):
        saved = service.save({"config": changed, "expectedDigest": expected})
      self.assertTrue(saved["connected"])
      self.assertTrue(saved["synchronized"])
      self.assertFalse(saved["cachePersisted"])
      self.assertEqual(saved["config"], changed)
      self.assertIn("已经生效", saved["message"])

  def test_persist_reloads_cache_and_resets_analysis_only_for_change(self):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory)
      analyzer = RecordingAnalyzer()
      service = GatewayConfigService(path, MemoryGateway(), analyzer)
      before = service.active
      changed = {"version": VERSION, "rules": before["rules"][:1]}
      service.persist(changed, before)
      self.assertEqual(analyzer.reset_count, 1)
      service.persist(changed, before)
      self.assertEqual(analyzer.reset_count, 1)

      restored = GatewayConfigService(path, MemoryGateway(), RecordingAnalyzer())
      self.assertEqual(restored.active, changed)
      self.assertEqual(restored.previous, before)
      self.assertFalse(any(entry.name.endswith(".tmp") for entry in path.iterdir()))

  def test_state_only_hides_protocol_errors(self):
    def unavailable(_request):
      return None

    def broken(_request):
      raise RuntimeError("unexpected exchange failure")

    with tempfile.TemporaryDirectory() as directory:
      service = GatewayConfigService(Path(directory), unavailable)
      state = service.state()
      self.assertFalse(state["connected"])
      self.assertIn("无有效响应", state["message"])

    with tempfile.TemporaryDirectory() as directory:
      service = GatewayConfigService(Path(directory), broken)
      with self.assertRaisesRegex(RuntimeError, "unexpected exchange failure"):
        service.state()

  def test_save_validates_envelope_before_contacting_gateway(self):
    exchange = mock.Mock(side_effect=AssertionError("gateway must not be contacted"))
    with tempfile.TemporaryDirectory() as directory:
      service = GatewayConfigService(Path(directory), exchange)
      invalid = (
        {},
        {"config": defaults(), "expectedDigest": "1234567"},
        {"config": defaults(), "expectedDigest": "not-hex!"},
      )
      for data in invalid:
        with self.subTest(data=data), self.assertRaises(ConfigError):
          service.save(data)
      exchange.assert_not_called()

  def test_analysis_methods_delegate_with_active_config(self):
    with tempfile.TemporaryDirectory() as directory:
      analyzer = RecordingAnalyzer()
      service = GatewayConfigService(Path(directory), MemoryGateway(), analyzer)
      active = service.active
      self.assertEqual(service.analysis(), {"config": active})
      self.assertEqual(service.analysis_frame(1, 0x399), {"bus": 1, "address": 0x399})
      self.assertEqual(service.reset_analysis(), {"ok": True})
      self.assertEqual(analyzer.snapshots, [active])
      self.assertEqual(analyzer.frames, [(active, 1, 0x399)])
      self.assertEqual(analyzer.reset_count, 1)


if __name__ == "__main__":
  unittest.main()
