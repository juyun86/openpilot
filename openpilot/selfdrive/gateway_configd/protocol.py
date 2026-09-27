from __future__ import annotations

import socket
import struct
import threading
import time

from openpilot.selfdrive.car.gateway_config_bridge import (BRIDGE_BUSY, BRIDGE_INVALID, BRIDGE_OK,
                                                            BRIDGE_TIMEOUT, BRIDGE_UNSAFE, SOCKET_NAME)
from openpilot.selfdrive.gateway_configd.model import MAX_RULES, VERSION, decode, digest, encode, encode_rule, validate

INFO, READ, STATUS = 1, 2, 3
BEGIN, RULE, COMMIT, ABORT = 0x10, 0x11, 0x12, 0x13
OK = 0
MIN_REQUEST_INTERVAL = 0.030


class ProtocolError(RuntimeError):
  pass


def packet(command, transaction, payload=b""):
  if not 0 <= transaction <= 255 or len(payload) > 6:
    raise ValueError("invalid packet")
  return bytes([command, transaction]) + payload + bytes(6 - len(payload))


class UnixExchange:
  def __init__(self, socket_name=SOCKET_NAME):
    self.socket_name = socket_name
    self.lock = threading.Lock()
    self.last_request = 0.0

  def __call__(self, request):
    with self.lock:
      delay = MIN_REQUEST_INTERVAL - (time.monotonic() - self.last_request)
      if delay > 0:
        time.sleep(delay)
      try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
          client.settimeout(1)
          client.connect(self.socket_name)
          client.sendall(request)
          self.last_request = time.monotonic()
          response = b""
          while len(response) < 9:
            part = client.recv(9 - len(response))
            if not part:
              break
            response += part
      except (OSError, TimeoutError):
        return None
    if len(response) != 9:
      return None
    status = response[0]
    if status == BRIDGE_OK:
      return response[1:]
    if status == BRIDGE_TIMEOUT:
      return None
    if status == BRIDGE_UNSAFE:
      raise ProtocolError("车辆必须保持 P 挡、零车速，驾驶控制必须关闭")
    if status == BRIDGE_BUSY:
      raise ProtocolError("另一个白名单请求正在进行，请稍后重试")
    if status == BRIDGE_INVALID:
      raise ProtocolError("C3 拒绝了无效配置请求")
    raise ProtocolError("C3 配置通道返回未知状态")


class Client:
  def __init__(self, exchange, transaction):
    self.exchange, self.transaction = exchange, transaction

  def call(self, command, payload=b"", mutation=False):
    request = packet(command, self.transaction, payload)
    for _ in range(3):
      response = self.exchange(request)
      if response is None:
        continue
      if len(response) != 8 or response[:2] != bytes([command | 0x80, self.transaction]):
        continue
      if mutation and response[2] != OK:
        raise ProtocolError(f"网关拒绝配置命令 0x{command:02X}，状态 {response[2]}")
      return response
    raise ProtocolError("网关无有效响应；车辆可能休眠或固件尚未安装")

  def read(self):
    before = self.call(INFO)
    if before[2] != VERSION or before[3] > MAX_RULES:
      raise ProtocolError("网关协议版本或规则数量不兼容")
    raw = bytes([before[3]])
    for index in range(before[3]):
      response = self.call(READ, bytes([index]))
      if response[2] != index:
        raise ProtocolError("规则索引不匹配")
      raw += response[3:]
    config = decode(raw + bytes((MAX_RULES - before[3]) * 5))
    after = self.call(INFO)
    expected = struct.unpack_from("<I", before, 4)[0]
    if before[2:] != after[2:] or digest(config) != expected:
      raise ProtocolError("读取期间配置已变化，请重试")
    status = self.call(STATUS)
    return config, status[2] == 1 and struct.unpack_from("<I", status, 4)[0] == expected

  def save(self, config, expected_digest):
    config = validate(config)
    try:
      self.call(BEGIN, bytes([len(config["rules"]), 0]) + struct.pack("<I", expected_digest), True)
      for index, rule in enumerate(config["rules"]):
        self.call(RULE, bytes([index]) + encode_rule(rule), True)
      wanted = digest(config)
      response = self.call(COMMIT, b"\0\0" + struct.pack("<I", wanted), True)
      if struct.unpack_from("<I", response, 4)[0] != wanted:
        raise ProtocolError("提交回执摘要不匹配")
      actual, synchronized = self.read()
      if encode(actual) != encode(config):
        raise ProtocolError("网关回读与提交内容不一致")
      if not synchronized:
        raise ProtocolError("MCU1 已保存，MCU2 尚未同步；请重新读取状态")
      return actual
    except ProtocolError:
      try:
        self.call(ABORT, mutation=True)
      except ProtocolError:
        pass
      raise
