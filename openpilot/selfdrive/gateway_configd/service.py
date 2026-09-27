from __future__ import annotations

import json
import logging
import os
from pathlib import Path
import secrets
import threading

from openpilot.selfdrive.gateway_configd.analysis import LiveCanAnalyzer
from openpilot.selfdrive.gateway_configd.model import ConfigError, defaults, digest, validate
from openpilot.selfdrive.gateway_configd.protocol import Client, ProtocolError, UnixExchange


DEFAULT_STATE_DIR = Path("/data/gateway-config")
LOG = logging.getLogger(__name__)


def atomic_json(path: Path, value) -> None:
  temporary = path.with_name(f".{path.name}.{secrets.token_hex(4)}.tmp")
  try:
    with temporary.open("x", encoding="utf-8") as output:
      json.dump(value, output, ensure_ascii=False, indent=2)
      output.flush()
      os.fsync(output.fileno())
    os.replace(temporary, path)
  finally:
    temporary.unlink(missing_ok=True)


class GatewayConfigService:
  """Gateway configuration and live-analysis service without an HTTP transport."""

  def __init__(self, state_dir: Path = DEFAULT_STATE_DIR, exchange=None, analyzer=None):
    self.lock = threading.RLock()
    self.state_dir = Path(state_dir)
    self.state_dir.mkdir(parents=True, exist_ok=True)
    os.chmod(self.state_dir, 0o700)
    self.cache_path = self.state_dir / "cache.json"
    self.exchange = UnixExchange() if exchange is None else exchange
    self.analyzer = LiveCanAnalyzer() if analyzer is None else analyzer
    self.transaction = secrets.randbelow(256)
    self.cache_persisted = True
    self.active, self.previous = defaults(), None
    try:
      cached = json.loads(self.cache_path.read_text(encoding="utf-8"))
      self.active = validate(cached["active"])
      self.previous = validate(cached["previous"]) if cached.get("previous") is not None else None
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
      pass

  def client(self) -> Client:
    with self.lock:
      self.transaction = (self.transaction + 1) & 0xFF
      return Client(self.exchange, self.transaction)

  def persist(self, active, previous) -> bool:
    with self.lock:
      changed = active != self.active
      cache_was_persisted = self.cache_persisted
      try:
        atomic_json(self.cache_path, {"active": active, "previous": previous})
        os.chmod(self.cache_path, 0o600)
        self.cache_persisted = True
        if not cache_was_persisted:
          LOG.info("gateway configuration cache recovered")
      except OSError:
        # The MCU readback is authoritative. A cache failure must not leave the
        # analyzer showing an old configuration or imply that a commit failed.
        self.cache_persisted = False
        if cache_was_persisted:
          LOG.exception("gateway configuration cache write failed; keeping MCU readback in memory")
      self.active, self.previous = active, previous
      if changed:
        self.analyzer.reset()
      return self.cache_persisted

  def _state_payload(self, connected: bool, synchronized: bool, message: str):
    if not self.cache_persisted:
      message += " 本地缓存写入失败；网关配置已经生效，后续将自动重试缓存。"
    return {"mode": "vehicle", "connected": connected, "config": self.active,
            "digest": f"{digest(self.active):08X}", "previous": self.previous,
            "synchronized": synchronized, "cachePersisted": self.cache_persisted,
            "message": message}

  def state(self):
    with self.lock:
      connected, synchronized = False, False
      message = "车辆唤醒并保持 P 挡、零车速后可读取和保存。"
      try:
        config, synchronized = self.client().read()
        if config != self.active:
          self.persist(config, self.active)
        elif not self.cache_persisted:
          self.persist(self.active, self.previous)
        connected = True
        message = "网关已连接，可以安全编辑。" if synchronized else "双 MCU 尚未同步，当前只允许重新读取。"
      except ProtocolError as error:
        message = str(error)
      return self._state_payload(connected, synchronized, message)

  def save(self, data):
    if not isinstance(data, dict) or set(data) != {"config", "expectedDigest"}:
      raise ConfigError("提交字段不正确")
    expected = data["expectedDigest"]
    if (not isinstance(expected, str) or len(expected) != 8 or
        any(char not in "0123456789abcdefABCDEF" for char in expected)):
      raise ConfigError("必须提供读取时的配置摘要")
    wanted = validate(data["config"])
    with self.lock:
      before = self.active
      actual = self.client().save(wanted, int(expected, 16))
      self.persist(actual, before)
      return self._state_payload(True, True, "网关已保存并完成双 MCU 回读校验。")

  def analysis(self):
    with self.lock:
      active = self.active
    return self.analyzer.snapshot(active)

  def reset_analysis(self):
    self.analyzer.reset()
    return {"ok": True}

  def analysis_frame(self, bus: int, address: int):
    with self.lock:
      active = self.active
    return self.analyzer.frame_detail(active, bus, address)
