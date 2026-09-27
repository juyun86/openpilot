import threading
from types import SimpleNamespace

from openpilot.selfdrive.gateway_configd.analysis import (LOG, MESSAGE_DEFINITIONS, RETRY_INITIAL_S, RETRY_MAX_S,
                                                          SOURCE_MESSAGE_NAMES, LiveCanAnalyzer, decode_lanes,
                                                          decode_status)


CONFIG = {"version": 2, "rules": [
  {"source": 3, "id": "0x239", "dlc": "any", "enabled": True},
  {"source": 3, "id": "0x399", "dlc": "any", "enabled": True},
  {"source": 3, "id": "0x5D9", "dlc": "any", "enabled": True},
]}


def frame(address: int, payload: str):
  return SimpleNamespace(address=address, dat=bytes.fromhex(payload), src=1)


def test_can3_long_control_uses_tcan_hw4_definition():
  assert SOURCE_MESSAGE_NAMES[3][0x209] == "DAS_longControl"
  assert MESSAGE_DEFINITIONS[(3, 0x209)] == ("tesla_modely_hw4_perception", "DAS_longControl")

  config = {"version": 2, "rules": [{"source": 3, "id": "0x209", "dlc": 8, "enabled": True}]}
  analyzer = LiveCanAnalyzer(sock=object(), clock=lambda: 1_100_000_000)
  analyzer.ingest(1_000_000_000, [frame(0x209, "06 C1 FF FF FF FF FF 3F")], config)
  found = analyzer._frame(1, 0x209, 1_100_000_000)
  assert found["available"] and found["physical_source"] == 3
  assert found["name"] == "DAS_longControl" and found["dbc_available"]


def test_long_control_only_exposes_the_active_mux_branch():
  class Parser:
    vl = {"DAS_longControl": {
      "DAS_longControlStack": 3.0,
      "DAS_longControlChecksum": 12.0,
      "DAS_longControlCounter": 7.0,
      "DAS_gearRequest": 0.0,
      "DAS_velocityProfile_futureTargetSpeedFwd": 80.0,
      "DAS_torqueControl_sysTorqueCommandFwd": -2.0,
    }}

    def update(self, _frames):
      pass

  analyzer = LiveCanAnalyzer(sock=object())
  analyzer.parsers = {"tesla_modely_hw4_perception": Parser()}
  signals = analyzer._dbc_signals(3, 1, 0x209, 1_000_000_000, bytes.fromhex("0C C7 FF FF FF FF FF 3F"))
  assert signals["DAS_longControlStack"] == 3.0
  assert signals["DAS_velocityProfile_futureTargetSpeedFwd"] == 80.0
  assert "DAS_torqueControl_sysTorqueCommandFwd" not in signals


def test_decodes_lane_and_status_fields():
  lanes = decode_lanes(bytes.fromhex("03 32 64 7D 7D 7D 06 00"))
  assert lanes["left_lane_exists"] and lanes["right_lane_exists"]
  assert lanes["left_line_usage"] == 2 and lanes["right_line_usage"] == 1

  # ALC=6 at bits 46..50, counter=8 at bits 52..55, checksum=0xBE.
  value = (6 << 46) | (8 << 52)
  payload = bytearray(value.to_bytes(8, "little"))
  payload[7] = 0xBE
  status = decode_status(payload)
  assert status["auto_lane_change_state"] == 6
  assert status["auto_lane_change_text"] == "仅左侧可变道"
  assert status["rolling_counter"] == 8 and status["checksum"] == 0xBE


def test_tracks_counter_health_and_mux_groups():
  analyzer = LiveCanAnalyzer(sock=object(), clock=lambda: 1_400_000_000)
  for index, counter in enumerate((7, 8, 10)):
    payload = bytearray(8)
    payload[6] = counter << 4
    analyzer.ingest(1_000_000_000 + index * 100_000_000, [SimpleNamespace(address=0x399, dat=payload)], CONFIG)
  analyzer.ingest(1_350_000_000, [frame(0x5D9, "03 00 FF 00 00 00 00 00")], CONFIG)

  status = analyzer._frame(1, 0x399, 1_400_000_000)
  assert status["consecutive"] == 1
  assert status["missing"] == 1
  assert status["counter_health"] == "发现缺口或重复"
  carlog = analyzer._frame(1, 0x5D9, 1_400_000_000)
  assert carlog["decoded"]["mux"] == 3
  assert carlog["decoded"]["mux_counts"] == [0, 0, 0, 1]


def test_only_tracks_whitelist_frames_on_gateway_output():
  analyzer = LiveCanAnalyzer(sock=object(), clock=lambda: 1_100_000_000)
  analyzer.ingest(1_000_000_000, [SimpleNamespace(address=0x123, dat=b"12345678", src=1),
                                  SimpleNamespace(address=0x239, dat=b"12345678", src=0),
                                  SimpleNamespace(address=0x239, dat=b"12345678", src=1)], CONFIG)
  assert (1, 0x123) not in analyzer.frames
  assert (0, 0x239) not in analyzer.frames
  found = analyzer._frame(1, 0x239, 1_100_000_000)
  assert found["available"] and found["physical_source"] == 3


def test_respects_dlc_and_resets_when_rule_source_changes():
  analyzer = LiveCanAnalyzer(sock=object(), clock=lambda: 1_200_000_000)
  exact = {"version": 2, "rules": [{"source": 2, "id": "0x123", "dlc": 8, "enabled": True}]}
  analyzer.ingest(1_000_000_000, [frame(0x123, "01 02 03 04")], exact)
  assert (1, 0x123) not in analyzer.frames
  analyzer.ingest(1_100_000_000, [frame(0x123, "01 02 03 04 05 06 07 08")], exact)
  assert analyzer._frame(1, 0x123, 1_200_000_000)["count"] == 1

  moved = {"version": 2, "rules": [{"source": 3, "id": "0x123", "dlc": 8, "enabled": True}]}
  analyzer.ingest(1_150_000_000, [frame(0x123, "11 12 13 14 15 16 17 18")], moved)
  changed = analyzer._frame(1, 0x123, 1_200_000_000)
  assert changed["count"] == 1 and changed["physical_source"] == 3
  analyzer.ingest(1_175_000_000, [], {"version": 2, "rules": []})
  assert (1, 0x123) not in analyzer.frames


def test_invalid_status_structure_has_no_counter_fields():
  analyzer = LiveCanAnalyzer(sock=object(), clock=lambda: 1_100_000_000)
  analyzer.ingest(1_000_000_000, [frame(0x399, "01 02 03 04")], CONFIG)
  status = analyzer._frame(1, 0x399, 1_100_000_000)
  assert not status["structure_valid"]
  assert status["decoded"] == {}
  assert "counter_health" not in status


def test_lane_change_evidence_requires_status_and_lane_geometry():
  supported = LiveCanAnalyzer._side("left", {"auto_lane_change_state": 6, "auto_lane_change_text": "仅左侧可变道",
                                                      "blind_left": 0},
                                    {"left_lane_exists": True, "left_line_usage": 2})
  assert supported["supported"]
  blocked = LiveCanAnalyzer._side("right", {"auto_lane_change_state": 6, "auto_lane_change_text": "仅左侧可变道",
                                                    "blind_right": 0},
                                  {"right_lane_exists": True, "right_line_usage": 2})
  assert not blocked["supported"]


def test_collector_recovers_after_receive_error_and_rebuilds_socket(monkeypatch):
  broken_socket = object()
  replacement_socket = object()
  received = threading.Event()
  running = threading.Event()
  replacement_created = threading.Event()
  analyzer = LiveCanAnalyzer(sock=broken_socket, clock=lambda: 1_100_000_000,
                             socket_factory=lambda: (replacement_created.set(), replacement_socket)[1])
  event = SimpleNamespace(logMonoTime=1_000_000_000,
                          can=[frame(0x239, "03 32 64 7D 7D 7D 06 00")])
  replacement_delivered = False

  def recv_one():
    nonlocal replacement_delivered
    if analyzer.sock is broken_socket:
      raise OSError("CAN subscriber disconnected")
    if not replacement_delivered:
      replacement_delivered = True
      return event
    running.set()
    analyzer.collector_stop.wait(0.1)
    return None

  original_ingest = analyzer.ingest

  def ingest(*args):
    original_ingest(*args)
    received.set()

  logged = []
  monkeypatch.setattr(LOG, "exception", lambda *args, **_kwargs: logged.append(args))
  monkeypatch.setattr(analyzer, "_receive", recv_one)
  monkeypatch.setattr(analyzer, "ingest", ingest)
  analyzer.start(lambda: CONFIG)
  try:
    assert received.wait(1), "collector did not resume after the receive error"
    assert running.wait(1), "collector did not remain live after ingesting the next event"
    assert replacement_created.is_set()
    assert len(logged) == 1
    assert analyzer._frame(1, 0x239, 1_100_000_000)["available"]

    health = analyzer.snapshot(CONFIG)["collector"]
    assert health["state"] == "running"
    assert health["thread_alive"]
    assert health["consecutive_failures"] == 0
    assert health["total_failures"] == 1
    assert health["last_error"] == "OSError: CAN subscriber disconnected"
  finally:
    analyzer.stop()
  stopped_collector = analyzer.collector
  assert analyzer.snapshot(CONFIG)["collector"]["state"] == "stopped"
  assert analyzer.collector is stopped_collector
  assert not analyzer.collector.is_alive()
  assert analyzer._collector_health()["state"] == "stopped"


def test_collector_uses_capped_backoff_for_repeated_failures(monkeypatch):
  class StopAfterRetries:
    def __init__(self):
      self.stopped = False
      self.delays = []

    def clear(self):
      self.stopped = False

    def is_set(self):
      return self.stopped

    def set(self):
      self.stopped = True

    def wait(self, delay):
      self.delays.append(delay)
      if len(self.delays) == 8:
        self.stopped = True
      return self.stopped

  socket_count = 0

  def socket_factory():
    nonlocal socket_count
    socket_count += 1
    return object()

  analyzer = LiveCanAnalyzer(socket_factory=socket_factory)
  stop_signal = StopAfterRetries()
  analyzer.collector_stop = stop_signal

  def recv_one():
    raise RuntimeError("persistent receive failure")

  logged_failures = []
  monkeypatch.setattr(LOG, "exception", lambda *args, **_kwargs: logged_failures.append(args[-1]))
  monkeypatch.setattr(analyzer, "_receive", recv_one)
  analyzer.start(lambda: CONFIG)
  analyzer.collector.join(1)

  assert not analyzer.collector.is_alive()
  assert stop_signal.delays == [RETRY_INITIAL_S, 0.2, 0.4, 0.8, 1.6, 3.2, RETRY_MAX_S, RETRY_MAX_S]
  assert all(delay > 0 for delay in stop_signal.delays)
  assert socket_count == len(stop_signal.delays)
  assert logged_failures == [1, 2, 4, 8]
  health = analyzer._collector_health()
  assert health["state"] == "stopped"
  assert health["consecutive_failures"] == len(stop_signal.delays)
  assert health["total_failures"] == len(stop_signal.delays)


def test_snapshot_restarts_unexpectedly_dead_collector(monkeypatch):
  stale_socket = object()
  replacement_socket = object()
  replacement_created = threading.Event()
  received = threading.Event()
  running = threading.Event()
  analyzer = LiveCanAnalyzer(sock=stale_socket, clock=lambda: 1_100_000_000,
                             socket_factory=lambda: (replacement_created.set(), replacement_socket)[1])
  analyzer.config_provider = lambda: CONFIG

  dead_collector = threading.Thread(target=lambda: None)
  dead_collector.start()
  dead_collector.join()
  analyzer.collector = dead_collector
  analyzer.collector_state = "running"

  event = SimpleNamespace(logMonoTime=1_000_000_000,
                          can=[frame(0x239, "03 32 64 7D 7D 7D 06 00")])
  delivered = False

  def recv_one():
    nonlocal delivered
    if not delivered:
      delivered = True
      return event
    running.set()
    analyzer.collector_stop.wait(0.1)
    return None

  original_ingest = analyzer.ingest

  def ingest(*args):
    original_ingest(*args)
    received.set()

  monkeypatch.setattr(analyzer, "_receive", recv_one)
  monkeypatch.setattr(analyzer, "ingest", ingest)
  analyzer.snapshot(CONFIG)
  try:
    assert received.wait(1), "snapshot did not restart the dead collector"
    assert running.wait(1), "replacement collector did not remain live"
    recovered = analyzer.snapshot(CONFIG)
    assert replacement_created.is_set()
    assert analyzer.collector is not dead_collector
    assert recovered["collector"]["state"] == "running"
    assert recovered["collector"]["thread_alive"]
    assert next(item for item in recovered["frames"] if item["id"] == "0x239")["available"]
  finally:
    analyzer.stop()
