from __future__ import annotations

import struct
import zlib

VERSION = 2
MAX_RULES = 48
REQUEST_ID = 0x6E0
RESPONSE_ID = 0x6E1
DEFAULT_IDS = (0x221, 0x3C2, 0x3DF, 0x3E9, 0x3F5, 0x238, 0x25D, 0x132, 0x212,
               0x219, 0x25A, 0x292, 0x31F, 0x33A, 0x3B6, 0x3D2, 0x3FE, 0x679)


class ConfigError(ValueError):
  pass


def defaults():
  pairs = [(2, identifier) for identifier in DEFAULT_IDS] + [(3, 0x239), (3, 0x399)]
  return {"version": VERSION, "rules": [
    {"source": source, "id": f"0x{identifier:03X}", "dlc": "any", "enabled": True}
    for source, identifier in pairs
  ]}


def validate(config):
  if not isinstance(config, dict) or set(config) != {"version", "rules"}:
    raise ConfigError("配置只能包含 version 和 rules")
  if type(config["version"]) is not int or config["version"] != VERSION:
    raise ConfigError("不支持的配置版本")
  rules = config["rules"]
  if not isinstance(rules, list) or len(rules) > MAX_RULES:
    raise ConfigError(f"最多允许 {MAX_RULES} 条规则")
  normalized, identifiers = [], set()
  for index, rule in enumerate(rules):
    prefix = f"第 {index + 1} 条："
    if not isinstance(rule, dict) or set(rule) != {"source", "id", "dlc", "enabled"}:
      raise ConfigError(prefix + "规则字段不完整或含未知字段")
    if type(rule["source"]) is not int or rule["source"] not in (2, 3, 4):
      raise ConfigError(prefix + "来源只能是 CAN2、CAN3 或 CAN4")
    value = rule["id"]
    if (not isinstance(value, str) or not value.startswith(("0x", "0X")) or
        not 3 <= len(value) <= 5 or any(c not in "0123456789abcdefABCDEF" for c in value[2:])):
      raise ConfigError(prefix + "ID 必须为 0x 开头的十六进制标准 ID")
    identifier = int(value, 16)
    if identifier > 0x7FF or identifier in (REQUEST_ID, RESPONSE_ID):
      raise ConfigError(prefix + "ID 超出标准帧范围或为配置协议保留 ID")
    if identifier in identifiers:
      raise ConfigError(prefix + "同一 ID 只能有一个来源，不能重复")
    identifiers.add(identifier)
    dlc = rule["dlc"]
    if dlc != "any" and (type(dlc) is not int or not 0 <= dlc <= 8):
      raise ConfigError(prefix + "DLC 必须为 any 或整数 0–8")
    if type(rule["enabled"]) is not bool:
      raise ConfigError(prefix + "enabled 必须为布尔值")
    normalized.append({"source": rule["source"], "id": f"0x{identifier:03X}",
                       "dlc": dlc, "enabled": rule["enabled"]})
  return {"version": VERSION, "rules": normalized}


def encode_rule(rule):
  return struct.pack("<BHBB", rule["source"], int(rule["id"], 16),
                     0xFF if rule["dlc"] == "any" else rule["dlc"], int(rule["enabled"]))


def encode(config):
  config = validate(config)
  rules = config["rules"]
  return bytes([len(rules)]) + b"".join(encode_rule(rule) for rule in rules) + bytes((MAX_RULES - len(rules)) * 5)


def decode(data):
  if len(data) != 1 + MAX_RULES * 5 or data[0] > MAX_RULES:
    raise ConfigError("无效的二进制配置长度")
  if any(data[1 + data[0] * 5:]):
    raise ConfigError("配置尾部必须补零")
  rules = []
  for index in range(data[0]):
    source, identifier, dlc, enabled = struct.unpack_from("<BHBB", data, 1 + index * 5)
    if enabled not in (0, 1):
      raise ConfigError("无效的启用标记")
    rules.append({"source": source, "id": f"0x{identifier:03X}",
                  "dlc": "any" if dlc == 0xFF else dlc, "enabled": bool(enabled)})
  return validate({"version": VERSION, "rules": rules})


def digest(config):
  return zlib.crc32(encode(config))
