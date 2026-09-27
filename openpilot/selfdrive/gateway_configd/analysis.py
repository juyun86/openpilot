"""Read-only live CAN evidence for the gateway web page."""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque


WINDOW_NS = 10_000_000_000
STALE_NS = 2_000_000_000
RETRY_INITIAL_S = 0.1
RETRY_MAX_S = 5.0
LOG = logging.getLogger(__name__)
ALC_TEXT = {
  0: "功能关闭", 1: "无可用车道", 2: "超声波不可用", 3: "跟车限制", 4: "正在驶出高速",
  5: "车速不满足", 6: "仅左侧可变道", 7: "仅右侧可变道", 8: "左右均可变道",
  9: "正在向左变道", 10: "正在向右变道", 11: "等待左侧障碍离开", 12: "等待右侧障碍离开",
  13: "等待左前障碍离开", 14: "等待右前障碍离开", 15: "左侧障碍导致中止",
  16: "右侧障碍导致中止", 17: "视距不足", 18: "变道健康状态异常", 19: "转向灯已关闭",
  20: "其他原因中止", 21: "实线限制", 22: "左侧目标 TTC 阻止", 23: "左侧目标/超声波阻止",
  24: "右侧目标 TTC 阻止", 25: "右侧目标/超声波阻止", 26: "左侧车道类型限制",
  27: "右侧车道类型限制", 28: "等待手握方向盘", 29: "变道超时", 30: "导航计划无效", 31: "无有效状态",
}
USAGE_TEXT = {0: "不可用", 1: "可用", 2: "融合采用", 3: "禁用"}
FORK_TEXT = {0: "无", 1: "可用", 2: "已选择", 3: "不可用"}
CARLOG_MUX = {0: "功能状态", 1: "雷达", 2: "FCM 前向感知", 3: "Profiler"}
SOURCE_MESSAGE_NAMES = {
  2: {
    0x221: "VCFRONT_LVPowerState", 0x3C2: "VCLEFT_switchStatus", 0x3DF: "UI_status2",
    0x3E9: "DAS_bodyControls", 0x3F5: "VCFRONT_lighting", 0x132: "BMS_hvBusStatus",
    0x212: "BMS_status", 0x219: "VCSEC_TPMSData", 0x25A: "VCSEC_TPMSDisplay",
    0x292: "BMS_socStatus", 0x31F: "TPMS_data", 0x33A: "UI_range",
    0x3B6: "DI_odometerStatus", 0x3D2: "BMS_kwhCounter", 0x3FE: "DI_estimatedBrakeTemp",
    0x679: "UI_ambientLightingCtrls",
  },
  3: {0x209: "DAS_longControl", 0x239: "DAS_lanes", 0x399: "DAS_statusCH"},
  4: {0x39B: "DAS_status"},
}
MESSAGE_DEFINITIONS = {
  (2, 0x221): ("tesla_model3_party", "VCFRONT_LVPowerState"),
  (2, 0x238): ("tesla_model3_vehicle", "STW_ACTN_RQ"),
  (2, 0x3C2): ("tesla_model3_vehicle", "VCLEFT_switchStatus"),
  (2, 0x3DF): ("tesla_model3_vehicle", "UI_status2"),
  (2, 0x3E9): ("tesla_model3_vehicle", "DAS_bodyControls"),
  (2, 0x3F5): ("tesla_model3_vehicle", "ID3F5VCFRONT_lighting"),
  **{(2, address): ("tesla_modely_hw4_perception", name) for address, name in SOURCE_MESSAGE_NAMES[2].items()
     if address not in (0x221, 0x3C2, 0x3DF, 0x3E9, 0x3F5)},
  **{(3, address): ("tesla_modely_hw4_perception", name) for address, name in SOURCE_MESSAGE_NAMES[3].items()},
  **{(4, address): ("tesla_modely_hw4_perception", name) for address, name in SOURCE_MESSAGE_NAMES[4].items()},
}
CEREAL_MESSAGE_DEFINITIONS = {
  **{(bus, address): ("tesla_model3_party", name) for bus in (0, 2) for address, name in {
    0x102: "VCLEFT_doorStatus", 0x103: "VCRIGHT_doorStatus", 0x129: "SCCM_steeringAngleSensor",
    0x27D: "APS_eacMonitor", 0x293: "DAS_settings", 0x2B9: "DAS_control", 0x311: "UI_warning",
    0x39D: "IBST_status", 0x488: "DAS_steeringControl",
  }.items()},
  **{(bus, address): ("tesla_modely_hw4_perception", name) for bus in (0, 2) for address, name in {
    0x209: "DAS_longControl", 0x299: "DAS_integratedSafetyFront", 0x30A: "DAS_object",
    0x389: "DAS_status2", 0x39B: "DAS_status",
  }.items()},
}
SPECIAL_NAMES = {
  0x238: "STW_ACTN_RQ",
  0x247: "DAS_autopilotDebug",
  0x24A: "DAS_visualDebug",
  0x25D: "含义待确认",
  0x298: "UI_csaOfframpCurvature",
  0x2A7: "UI_csaRoadCurvature",
  0x5D9: "DAS_carLog",
}
LONG_CONTROL_PREFIX = {
  2: "DAS_torqueProfiler_", 3: "DAS_velocityProfile_", 4: "DAS_aebControl_",
  5: "DAS_pedalControl_", 6: "DAS_torqueControl_",
}


def _bits(payload: bytes, start: int, length: int) -> int:
  return (int.from_bytes(payload, "little") >> start) & ((1 << length) - 1)


def decode_lanes(payload: bytes) -> dict[str, object]:
  if len(payload) != 8:
    return {}
  return {
    "left_lane_exists": bool(_bits(payload, 0, 1)),
    "right_lane_exists": bool(_bits(payload, 1, 1)),
    "virtual_lane_width_m": round(_bits(payload, 4, 4) * 0.3125 + 2.0, 2),
    "view_range_m": _bits(payload, 8, 8),
    "left_line_usage": _bits(payload, 48, 2),
    "left_line_usage_text": USAGE_TEXT[_bits(payload, 48, 2)],
    "right_line_usage": _bits(payload, 50, 2),
    "right_line_usage_text": USAGE_TEXT[_bits(payload, 50, 2)],
    "left_fork": _bits(payload, 52, 2),
    "left_fork_text": FORK_TEXT[_bits(payload, 52, 2)],
    "right_fork": _bits(payload, 54, 2),
    "right_fork_text": FORK_TEXT[_bits(payload, 54, 2)],
  }


def decode_status(payload: bytes) -> dict[str, object]:
  if len(payload) != 8:
    return {}
  alc = _bits(payload, 46, 5)
  return {
    "autopilot_state": _bits(payload, 0, 4),
    "blind_left": _bits(payload, 4, 2),
    "blind_right": _bits(payload, 6, 2),
    "fused_speed_limit_kph": _bits(payload, 8, 5) * 5,
    "vision_speed_limit_kph": _bits(payload, 16, 5) * 5,
    "forward_collision_warning": _bits(payload, 22, 2),
    "side_collision_avoid": _bits(payload, 32, 2),
    "side_collision_warning": _bits(payload, 34, 2),
    "side_collision_inhibit": bool(_bits(payload, 36, 1)),
    "lane_departure_warning": _bits(payload, 37, 3),
    "hands_on_state": _bits(payload, 42, 4),
    "auto_lane_change_state": alc,
    "auto_lane_change_text": ALC_TEXT.get(alc, "未知"),
    "rolling_counter": _bits(payload, 52, 4),
    "checksum": payload[7],
  }


class LiveCanAnalyzer:
  def __init__(self, sock=None, clock=time.monotonic_ns, socket_factory=None):
    self.sock = sock
    self.clock = clock
    self.socket_factory = socket_factory
    self.frames: dict[tuple[int, int], dict[str, object]] = {}
    self.parsers = None
    self.lock = threading.RLock()
    self.collector = None
    self.collector_stop = threading.Event()
    self.collector_state = "not_started"
    self.collector_consecutive_failures = 0
    self.collector_total_failures = 0
    self.collector_last_error = None
    self.collector_last_error_ns = None
    self.collector_last_event_ns = None
    self.config_provider = None
    self.rule_specs: dict[int, tuple[int, int | None]] = {}

  def reset(self) -> None:
    with self.lock:
      self.frames.clear()
      self.parsers = None
      self.rule_specs.clear()

  def start(self, config_provider) -> None:
    with self.lock:
      if self.collector is not None and self.collector.is_alive():
        return
      self.config_provider = config_provider
      self.collector_stop.clear()
      self.collector_state = "starting"
      self.collector = threading.Thread(target=self._collect, name="gateway-can-analyzer", daemon=True)
      self.collector.start()

  def stop(self, timeout: float = 1.0) -> None:
    with self.lock:
      self.collector_stop.set()
      collector = self.collector
    if collector is not None and collector is not threading.current_thread():
      collector.join(timeout)

  def _collect(self) -> None:
    retry_delay = RETRY_INITIAL_S
    try:
      while not self.collector_stop.is_set():
        try:
          self._ensure_socket()
          event = self._receive()
          if event is not None:
            self.ingest(int(event.logMonoTime), event.can, self.config_provider())
          with self.lock:
            self.collector_state = "running"
            self.collector_consecutive_failures = 0
            if event is not None:
              self.collector_last_event_ns = time.monotonic_ns()
          retry_delay = RETRY_INITIAL_S
        except Exception as error:
          with self.lock:
            self.collector_state = "recovering"
            self.collector_consecutive_failures += 1
            self.collector_total_failures += 1
            failure_count = self.collector_consecutive_failures
            self.collector_last_error = f"{type(error).__name__}: {error}"[:240]
            self.collector_last_error_ns = time.monotonic_ns()
            # Dropping the reference destroys the msgq socket and makes the
            # next iteration subscribe again instead of retrying a bad link.
            self.sock = None
          # Log the first failure and then powers of two. Health remains
          # queryable on every failure without flooding logs during outages.
          if failure_count == 1 or (failure_count & (failure_count - 1)) == 0:
            LOG.exception("gateway CAN analyzer failed; rebuilding subscriber after %.1fs (failure %d)",
                          retry_delay, failure_count)
          if self._wait_for_retry(retry_delay):
            break
          retry_delay = min(retry_delay * 2, RETRY_MAX_S)
    finally:
      with self.lock:
        self.collector_state = "stopped" if self.collector_stop.is_set() else "failed"

  def _wait_for_retry(self, delay: float) -> bool:
    return self.collector_stop.wait(delay)

  def _receive(self):
    from openpilot.cereal import messaging
    return messaging.recv_one(self.sock)

  def _collector_health(self) -> dict[str, object]:
    with self.lock:
      now_ns = time.monotonic_ns()
      state = self.collector_state
      thread_alive = self.collector is not None and self.collector.is_alive()
      if self.collector is not None and not thread_alive and state not in ("stopped", "not_started"):
        state = "failed"
      health = {
        "state": state,
        "thread_alive": thread_alive,
        "consecutive_failures": self.collector_consecutive_failures,
        "total_failures": self.collector_total_failures,
        "last_error": self.collector_last_error,
        "last_error_age_ms": None,
        "last_event_age_ms": None,
      }
      if self.collector_last_error_ns is not None:
        health["last_error_age_ms"] = max(0, round((now_ns - self.collector_last_error_ns) / 1e6))
      if self.collector_last_event_ns is not None:
        health["last_event_age_ms"] = max(0, round((now_ns - self.collector_last_event_ns) / 1e6))
      return health

  def _ensure_parsers(self) -> None:
    if self.parsers is not None:
      return
    try:
      from opendbc.can import CANParser
      grouped: dict[str, set[str]] = {}
      for dbc, name in (*MESSAGE_DEFINITIONS.values(), *CEREAL_MESSAGE_DEFINITIONS.values()):
        grouped.setdefault(dbc, set()).add(name)
      self.parsers = {dbc: CANParser(dbc, [(name, math.nan) for name in sorted(names)], 1)
                      for dbc, names in grouped.items()}
    except (ImportError, RuntimeError):
      self.parsers = {}

  def _dbc_signals(self, physical_source: int | None, cereal_source: int, address: int,
                   timestamp_ns: int, payload: bytes) -> dict[str, object]:
    definition = MESSAGE_DEFINITIONS.get((physical_source, address)) if physical_source is not None else None
    definition = definition or CEREAL_MESSAGE_DEFINITIONS.get((cereal_source, address))
    if definition is None:
      return {}
    dbc, name = definition
    self._ensure_parsers()
    parser = self.parsers.get(dbc)
    if parser is None:
      return {}
    parser.update([(timestamp_ns, [(address, payload, 1)])])
    result = {}
    for signal, value in parser.vl[name].items():
      number = float(value)
      if math.isfinite(number):
        result[signal] = round(number, 5)
    if name == "DAS_longControl":
      stack = int(result.get("DAS_longControlStack", -1))
      prefix = LONG_CONTROL_PREFIX.get(stack)
      common = {"DAS_longControlStack", "DAS_longControlChecksum", "DAS_longControlCounter", "DAS_gearRequest"}
      result = {signal: value for signal, value in result.items()
                if signal in common or (prefix is not None and signal.startswith(prefix))}
    return result

  def _ensure_socket(self):
    if self.sock is None:
      if self.socket_factory is not None:
        self.sock = self.socket_factory()
      else:
        from openpilot.cereal import messaging
        # Block briefly while idle instead of spinning continuously when no CAN
        # events are available. Incoming data still wakes the socket immediately.
        self.sock = messaging.sub_sock("can", conflate=False, timeout=100)

  @staticmethod
  def _enabled_rules(config) -> dict[int, tuple[int, int | None]]:
    enabled = {}
    for rule in config.get("rules", []):
      if rule.get("enabled"):
        dlc = None if rule["dlc"] == "any" else int(rule["dlc"])
        enabled[int(rule["id"], 16)] = (int(rule["source"]), dlc)
    return enabled

  def _sync_rules(self, enabled: dict[int, tuple[int, int | None]]) -> None:
    changed = {address for address, spec in self.rule_specs.items() if enabled.get(address) != spec}
    changed.update(address for address in enabled if self.rule_specs.get(address) != enabled[address])
    if changed:
      for key in [key for key in self.frames if key[1] in changed]:
        del self.frames[key]
    self.rule_specs = enabled.copy()

  def ingest(self, timestamp_ns: int, frames, config) -> None:
    enabled = self._enabled_rules(config)
    with self.lock:
      self._sync_rules(enabled)
      for frame in frames:
        self._ingest_frame(timestamp_ns, frame, enabled)

  def _ingest_frame(self, timestamp_ns: int, frame, enabled: dict[int, tuple[int, int | None]]) -> None:
    address = int(frame.address if hasattr(frame, "address") else frame[0])
    payload = bytes(frame.dat if hasattr(frame, "dat") else frame[1])
    cereal_source = int(getattr(frame, "src", 1))
    # The analyzer verifies the whitelist at the gateway output. Raw vehicle
    # traffic on C3 CAN0/CAN2 and IDs outside the active rules are irrelevant.
    rule = enabled.get(address)
    if cereal_source != 1 or rule is None:
      return
    physical_source, expected_dlc = rule
    if expected_dlc is not None and len(payload) != expected_dlc:
      return
    key = (cereal_source, address)
    record = self.frames.setdefault(key, {
      "count": 0, "first_ns": timestamp_ns, "last_ns": timestamp_ns, "arrivals": deque(maxlen=512),
      "payload": b"", "dlc": 0, "counter": None, "counter_samples": 0, "consecutive": 0,
      "missing": 0, "duplicates": 0, "mux_counts": [0, 0, 0, 0],
      "change_mask": bytearray(len(payload)), "cereal_source": cereal_source,
      "physical_source": physical_source, "rule_signature": rule, "signals": {}, "decoded_payload": None,
    })
    previous_payload = record["payload"]
    if len(record["change_mask"]) != len(payload):
      record["change_mask"] = bytearray(len(payload))
      if address == 0x399:
        record.update({"counter": None, "counter_samples": 0, "consecutive": 0, "missing": 0, "duplicates": 0})
    if len(previous_payload) == len(payload):
      for index, (old, new) in enumerate(zip(previous_payload, payload, strict=True)):
        record["change_mask"][index] |= old ^ new
    record["count"] += 1
    record["last_ns"] = timestamp_ns
    record["arrivals"].append(timestamp_ns)
    record["payload"] = payload
    record["dlc"] = len(payload)
    if address == 0x399 and len(payload) == 8:
      counter = _bits(payload, 52, 4)
      previous = record["counter"]
      if previous is not None:
        delta = (counter - previous) & 0xF
        if delta == 0:
          record["duplicates"] += 1
        elif delta == 1:
          record["consecutive"] += 1
        else:
          record["missing"] += delta - 1
      record["counter"] = counter
      record["counter_samples"] += 1
    if address == 0x5D9 and len(payload):
      record["mux_counts"][payload[0] & 0x3] += 1

  def _drain(self, config) -> None:
    with self.lock:
      collector = self.collector
      if collector is not None:
        if collector.is_alive():
          return
        if not self.collector_stop.is_set() and self.config_provider is not None:
          # An unexpectedly dead collector may have left a poisoned socket.
          # Discard it before start() installs a replacement thread.
          self.sock = None
          self.start(self.config_provider)
        return
    self._ensure_socket()
    from openpilot.cereal import messaging
    for event in messaging.drain_sock(self.sock):
      self.ingest(int(event.logMonoTime), event.can, config)

  def _frame(self, cereal_source: int, address: int, now_ns: int, include_signals: bool = False,
             expected_rule: tuple[int, int | None] | None = None) -> dict[str, object]:
    record = self.frames.get((cereal_source, address))
    if record is None or (expected_rule is not None and record.get("rule_signature") != expected_rule):
      physical_source = expected_rule[0] if expected_rule is not None else None
      return {"source": physical_source if physical_source is not None else cereal_source,
              "physical_source": physical_source, "cereal_source": cereal_source,
              "id": f"0x{address:03X}", "available": False}
    cutoff = now_ns - WINDOW_NS
    arrivals = [stamp for stamp in record["arrivals"] if stamp >= cutoff]
    rate = 0.0
    if len(arrivals) > 1 and arrivals[-1] > arrivals[0]:
      rate = (len(arrivals) - 1) * 1e9 / (arrivals[-1] - arrivals[0])
    payload = record["payload"]
    definition = (MESSAGE_DEFINITIONS.get((record["physical_source"], address))
                  if record["physical_source"] is not None else None)
    definition = definition or CEREAL_MESSAGE_DEFINITIONS.get((cereal_source, address))
    if include_signals and record["decoded_payload"] != payload:
      record["signals"] = self._dbc_signals(record["physical_source"], cereal_source, address,
                                             record["last_ns"], payload)
      record["decoded_payload"] = payload
    age_ms = max(0, round((now_ns - record["last_ns"]) / 1e6))
    frame = {
      "source": record["physical_source"] if record["physical_source"] is not None else cereal_source,
      "physical_source": record["physical_source"], "id": f"0x{address:03X}",
      "available": True, "fresh": age_ms <= STALE_NS / 1e6,
      "dlc": record["dlc"], "age_ms": age_ms, "count": record["count"], "rate_hz": round(rate, 1),
      "payload": " ".join(f"{byte:02X}" for byte in payload),
      "change_mask": " ".join(f"{byte:02X}" for byte in record["change_mask"]),
      "cereal_source": record["cereal_source"],
      "dbc_available": definition is not None,
      "signals": record.get("signals", {}) if include_signals else {},
    }
    if address == 0x239:
      frame.update({"name": "DAS_lanes", "decoded": decode_lanes(payload), "structure_valid": len(payload) == 8})
    elif address == 0x399:
      decoded = decode_status(payload)
      frame.update({
        "name": "DAS_statusCH", "decoded": decoded, "structure_valid": len(payload) == 8,
      })
      if len(payload) == 8:
        frame.update({
          "counter_health": "连续性良好" if record["missing"] == 0 and record["duplicates"] == 0 else "发现缺口或重复",
          "counter_samples": int(record["counter_samples"]), "consecutive": record["consecutive"],
          "missing": record["missing"], "duplicates": record["duplicates"],
        })
    elif address == 0x5D9:
      mux = payload[0] & 3 if payload else None
      frame.update({"name": "DAS_carLog", "structure_valid": len(payload) == 8,
                    "decoded": {"mux": mux, "mux_text": CARLOG_MUX.get(mux, "未知"),
                                "mux_counts": record["mux_counts"]}})
    else:
      cereal_definition = CEREAL_MESSAGE_DEFINITIONS.get((cereal_source, address))
      name = SOURCE_MESSAGE_NAMES.get(record["physical_source"], {}).get(address) if record["physical_source"] is not None else None
      frame.update({"name": name or (cereal_definition[1] if cereal_definition else SPECIAL_NAMES.get(address, "未知报文")),
                    "decoded": {}, "structure_valid": True})
    return frame

  @staticmethod
  def _side(direction: str, status: dict, lanes: dict) -> dict[str, object]:
    alc = status.get("auto_lane_change_state")
    is_left = direction == "left"
    alc_support = alc in ((6, 8, 9) if is_left else (7, 8, 10))
    lane_exists = bool(lanes.get(f"{direction}_lane_exists"))
    usage = int(lanes.get(f"{direction}_line_usage", 0))
    blind = int(status.get(f"blind_{direction}", 0))
    supported = alc_support and lane_exists and usage in (1, 2)
    reasons = [f"0x399 ALC {alc}: {status.get('auto_lane_change_text', '等待报文')}"]
    reasons.append(f"0x399 {('左' if is_left else '右')}后盲区当前" + ("无告警（不等于无车）" if blind == 0 else f"等级 {blind}"))
    reasons.append(f"0x239 {('左' if is_left else '右')}侧相邻车道" + ("候选存在" if lane_exists else "未确认") +
                   f"，线型质量 {USAGE_TEXT.get(usage, '未知')}")
    return {"supported": supported, "status": "原车证据支持" if supported else "原车当前不支持",
            "summary": "原车状态与车道候选交叉支持" if supported else f"原车当前不支持{('左' if is_left else '右')}侧",
            "reasons": reasons}

  def snapshot(self, config) -> dict[str, object]:
    self._drain(config)
    with self.lock:
      now_ns = self.clock()
      enabled = self._enabled_rules(config)
      self._sync_rules(enabled)
      frames = [self._frame(1, address, now_ns, expected_rule=enabled[address]) for address in sorted(enabled)]
      for frame in frames:
        if not frame.get("available"):
          address = int(frame["id"], 16)
          physical_source = enabled[address][0]
          frame.update({"physical_source": physical_source, "cereal_source": 1,
                        "name": SOURCE_MESSAGE_NAMES.get(physical_source, {}).get(address,
                               SPECIAL_NAMES.get(address, "未知报文"))})
    by_id = {frame["id"]: frame for frame in frames}
    status = by_id.get("0x399", {}).get("decoded", {})
    lanes = by_id.get("0x239", {}).get("decoded", {})
    left = self._side("left", status, lanes)
    right = self._side("right", status, lanes)
    fresh = [frame for frame in frames if frame.get("fresh")]
    source_counts = {str(source): {"configured": 0, "fresh": 0} for source in (2, 3, 4)}
    for frame in frames:
      source = str(frame.get("physical_source"))
      if source in source_counts:
        source_counts[source]["configured"] += 1
        if frame.get("fresh"):
          source_counts[source]["fresh"] += 1
    return {
      "available": bool(fresh), "read_only": True, "updated_ms": round(now_ns / 1e6),
      "collector": self._collector_health(),
      "summary": {"status": "按当前白名单验证", "global_no_go": "只检查 C3 CAN1 转发结果",
                   "fresh_frames": len(fresh), "configured_frames": len(frames),
                   "discovered_frames": sum(bool(frame.get("available")) for frame in frames), "sources": source_counts},
      "left": left, "right": right, "frames": frames,
    }

  def frame_detail(self, config, cereal_source: int, address: int) -> dict[str, object]:
    self._drain(config)
    with self.lock:
      enabled = self._enabled_rules(config)
      self._sync_rules(enabled)
      expected_rule = enabled.get(address) if cereal_source == 1 else None
      if expected_rule is None or (cereal_source, address) not in self.frames:
        raise KeyError("报文尚未发现")
      return self._frame(cereal_source, address, self.clock(), include_signals=True, expected_rule=expected_rule)
