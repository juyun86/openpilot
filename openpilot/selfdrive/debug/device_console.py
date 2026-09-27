#!/usr/bin/env python3
# ruff: noqa: E501  # The embedded HTML/CSS/JavaScript is intentionally compact.
import ipaddress
import json
from pathlib import Path
import socket
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import BinaryIO, cast
from urllib.parse import parse_qs, urlparse

from openpilot.sunnypilot.selfdrive.car.tesla.validation_controller import VALIDATION_LOG_PATH
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.debug.tesla_turn_signal_test import (
  cancel_validation_session,
  get_validation_status,
  start_validation_session,
)
from openpilot.selfdrive.debug.device_settings import settings_snapshot, validate_and_write
from openpilot.selfdrive.debug.device_hotspot import hotspot_status, set_hotspot_enabled
from openpilot.selfdrive.debug.device_console_auth import client_is_local, console_status, require_offroad
from openpilot.selfdrive.debug.device_log_download import (
  LogSelection,
  available_log_range,
  delete_log_selection,
  download_filename,
  select_log_range,
  stream_log_zip,
)
from openpilot.selfdrive.debug.navigation_log_web import PANEL as NAVIGATION_LOG_PANEL, SCRIPT as NAVIGATION_LOG_SCRIPT
from openpilot.sunnypilot.navassist.diagnostics import flush_recording, log_status as navigation_log_status, select_logs as select_navigation_logs, stream_logs as stream_navigation_logs
from openpilot.selfdrive.debug.device_system_diagnostics import (
  MAX_SYSTEM_DIAGNOSTIC_BYTES,
  collect_system_diagnostics,
)
from openpilot.selfdrive.debug.device_terminal import change_password, run_command, terminal_status
from openpilot.selfdrive.debug.device_network import device_ip_address
from openpilot.selfdrive.debug.driving_status import driving_status_snapshot
from openpilot.selfdrive.debug.tesla_ambient_test import run_ambient_test
from openpilot.selfdrive.debug.unknown_can_observer import start_unknown_can_observer
from openpilot.selfdrive.debug.tesla_speed_button_test import SpeedButtonAction, run_validation
from openpilot.selfdrive.gateway_configd.model import ConfigError, digest as gateway_digest, validate as validate_gateway_config
from openpilot.selfdrive.gateway_configd.protocol import ProtocolError
from openpilot.selfdrive.gateway_configd.service import DEFAULT_STATE_DIR, GatewayConfigService
from openpilot.sunnypilot.hardware.profile import HardwareProfile, get_hardware_profile


HOST = "0.0.0.0"
PORT = 8088
GATEWAY_STATIC = Path(__file__).resolve().parents[1] / "gateway_configd" / "static"
MAX_GATEWAY_BODY = 16 * 1024
MAX_REQUEST_THREADS = 8
SOCKET_TIMEOUT_S = 10.0
_PRIVATE_NETWORKS = tuple(ipaddress.ip_network(value) for value in (
  "10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16", "127.0.0.0/8", "169.254.0.0/16",
  "fc00::/7", "fe80::/10", "::1/128",
))
_TESLA_CONSOLE_ADDRESS = ipaddress.ip_address("99.99.99.99")
_SESSION_LOCK = threading.Lock()
_ACTIVE_WEB_TEST_ID: str | None = None
_ACTIVE_WEB_SESSION_STARTED = 0.0
WEB_SESSION_TIMEOUT_S = 20.0
_LOG_DOWNLOAD_LOCK = threading.Lock()


def _clear_active_session(test_id: str) -> None:
  global _ACTIVE_WEB_SESSION_STARTED, _ACTIVE_WEB_TEST_ID
  with _SESSION_LOCK:
    if _ACTIVE_WEB_TEST_ID == test_id:
      _ACTIVE_WEB_TEST_ID = None
      _ACTIVE_WEB_SESSION_STARTED = 0.0


def render_page() -> bytes:
  return """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
  <title>车载设置</title>
  <style>
    :root { color-scheme:dark; font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
    body { margin:0; background:#111827; color:#f9fafb; } main { max-width:840px; margin:auto; padding:18px 14px 42px; }
    h1 { font-size:24px; margin:4px 0; } p { color:#cbd5e1; line-height:1.45; } .tabs { display:flex; gap:8px; margin:18px 0; }
    .tab, button { border:0; border-radius:12px; font-weight:700; color:white; background:#334155; padding:12px 16px; font-size:16px; text-decoration:none; }
    .tab.active { background:#2563eb; } .notice { padding:11px 13px; border-radius:10px; margin:12px 0; background:#14532d; color:#dcfce7; }
    .notice.onroad { background:#7c2d12; color:#ffedd5; } .group { margin:22px 0 9px; color:#93c5fd; font-size:15px; }
    .category-nav { display:flex; gap:8px; overflow-x:auto; padding:4px 0 10px; position:sticky; top:0; background:#111827; z-index:1; } .category { white-space:nowrap; padding:9px 12px; font-size:14px; }
    .category.active { background:#2563eb; }
    .card { display:grid; grid-template-columns:1fr auto; gap:10px; align-items:center; background:#1e293b; border-radius:13px; padding:14px; margin:8px 0; }
    .card h2 { font-size:16px; margin:0 0 5px; } .card p { font-size:13px; margin:0; } .lock { color:#fbbf24; font-size:12px; }
    input[type=number], input[type=datetime-local], select { width:120px; padding:9px; border:1px solid #475569; border-radius:9px; background:#0f172a; color:white; font-size:16px; }
    input[type=checkbox] { width:28px; height:28px; accent-color:#2563eb; } input:disabled, select:disabled, button:disabled { opacity:.45; }
    #turn-panel { text-align:center; } .buttons { display:grid; grid-template-columns:1fr 1fr; gap:16px; margin-top:28px; }
    .turn { min-height:120px; font-size:26px; } #left { background:#2563eb; } #right { background:#ea580c; } #cancel { display:none; width:100%; min-height:64px; margin-top:16px; background:#dc2626; }
    #status { min-height:52px; margin-top:18px; color:#fde68a; white-space:pre-wrap; }
    textarea { width:100%; min-height:130px; box-sizing:border-box; padding:11px; border:1px solid #475569; border-radius:10px; background:#020617; color:#e2e8f0; font:14px ui-monospace,monospace; }
    #terminal-output { min-height:100px; max-height:420px; overflow:auto; text-align:left; white-space:pre-wrap; background:#020617; border-radius:10px; padding:12px; color:#cbd5e1; }
    .terminal-row { display:flex; gap:8px; margin:10px 0; } .terminal-row input { min-width:0; flex:1; padding:9px; border:1px solid #475569; border-radius:9px; background:#0f172a; color:white; }
    .drive-alert { white-space:pre-wrap; } #driving-canvas { display:block; width:100%; height:min(72vh,640px); margin:12px 0; border:1px solid #334155; border-radius:18px; background:#07111f; }
    .chart-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(330px,1fr)); gap:12px; margin:14px 0; } .chart-card { min-width:0; padding:12px; border:1px solid #334155; border-radius:14px; background:#0b1525; } .chart-card h2 { margin:0 0 3px; color:#dbeafe; font-size:15px; } .chart-card p { margin:0 0 8px; font-size:11px; color:#94a3b8; } .signal-chart { display:block; width:100%; height:230px; border-radius:9px; background:#07111f; }
    .ped-coordinate-lab { display:flex; flex-wrap:wrap; align-items:center; gap:8px; margin:10px 0 4px; padding:10px 12px; border:1px solid #92400e; border-radius:11px; background:#451a0355; color:#fde68a; font-size:12px; } .ped-coordinate-lab select { width:auto; max-width:100%; font-size:13px; padding:7px; }
    .can-diagnostics { margin-top:10px; } .can-diagnostics summary { cursor:pointer; color:#94a3b8; font-size:13px; padding:8px 2px; user-select:none; }
    .can-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); gap:9px; margin-top:10px; } .can-detail { background:#1e293b; border-radius:11px; padding:11px; color:#cbd5e1; font-size:12px; line-height:1.55; } .can-detail strong { display:block; color:#93c5fd; font-size:14px; margin-bottom:3px; } .can-detail .ok { color:#86efac; } .can-detail.warn { color:#fbbf24; }
    .log-range { display:grid; grid-template-columns:1fr 1fr; gap:12px; margin:16px 0; } .log-range label { display:grid; gap:6px; color:#cbd5e1; font-size:13px; } .log-range input { width:100%; box-sizing:border-box; }
    .log-actions { display:flex; flex-wrap:wrap; gap:10px; align-items:center; } #log-download { background:#2563eb; } #log-delete { background:#dc2626; } #log-preview { min-height:52px; margin-top:14px; white-space:pre-wrap; }
    @media (max-width:600px) { .log-range, .chart-grid { grid-template-columns:1fr; } .chart-grid { gap:9px; } .signal-chart { height:210px; } }
    .tabs { overflow-x:auto; } .tabs .tab { white-space:nowrap; }
    #vehicle-panel { margin:24px 0 30px; background:#111418; border:1px solid #282d33; border-radius:24px; padding:24px; }
    .vehicle-head { display:flex; align-items:center; justify-content:space-between; color:#dce3e8; letter-spacing:3px; font-size:16px; }
    .vehicle-head span { font-size:12px; letter-spacing:0; color:#8a97a3; } .vehicle-head #vehicle-connection::before { content:''; display:inline-block; width:6px; height:6px; border-radius:50%; background:#6ee2b4; margin-right:8px; }
    .vehicle-icon { width:24px; height:24px; fill:none; stroke:currentColor; stroke-width:1.6; stroke-linecap:round; stroke-linejoin:round; flex:none; }
    .vehicle-hero { display:grid; grid-template-columns:1.5fr 1fr; gap:36px; padding:30px 0 26px; }
    .vehicle-label { display:flex; align-items:center; gap:10px; color:#8996a3; font-size:13px; }
    .vehicle-value { font-size:38px; font-weight:500; letter-spacing:-1.5px; line-height:1.25; font-variant-numeric:tabular-nums; color:#eef1f3; }
    #vehicle-soc { font-size:64px; margin:8px 0 14px; } .battery-track { height:4px; border-radius:8px; background:#2d333a; overflow:hidden; }
    #vehicle-battery-fill { height:100%; width:0; background:#6ee2b4; border-radius:8px; transition:width .3s; }
    .vehicle-note { margin-top:5px; color:#8996a3; font-size:12px; } #vehicle-range { margin-top:12px; }
    .vehicle-body { display:grid; grid-template-columns:1.6fr 1fr; gap:30px; align-items:center; padding:10px 0 24px; }
    .vehicle-car-stage { height:265px; position:relative; min-width:0; } .vehicle-car { height:255px; width:132px; position:absolute; left:50%; top:4px; transform:translateX(-50%); }
    .vehicle-wheel { position:absolute; width:88px; } .vehicle-wheel:nth-of-type(1) { left:0; top:14px; } .vehicle-wheel:nth-of-type(2) { right:0; top:14px; text-align:right; }
    .vehicle-wheel:nth-of-type(3) { left:0; bottom:12px; } .vehicle-wheel:nth-of-type(4) { right:0; bottom:12px; text-align:right; }
    .vehicle-wheel small { color:#8996a3; font-size:12px; } .vehicle-wheel strong { display:block; color:#eef1f3; font-size:27px; font-weight:500; line-height:1.5; font-variant-numeric:tabular-nums; }
    .vehicle-wheel.warn strong { color:#f5616c; } .vehicle-stats { display:grid; gap:32px; border-left:1px solid #2a3037; padding-left:26px; }
    .vehicle-stats .vehicle-value { font-size:32px; margin-top:12px; } .vehicle-lights { background:#1a1e23; border-radius:16px; padding:16px; }
    .vehicle-data-grid { display:grid; grid-template-columns:repeat(4,1fr); gap:10px; margin:12px 0; }
    .vehicle-data-cell { background:#1a1e23; border-radius:12px; padding:13px; min-width:0; }
    .vehicle-data-cell small { color:#8996a3; display:block; font-size:11px; margin-bottom:6px; }
    .vehicle-data-cell strong { color:#eef1f3; font-size:20px; font-weight:500; font-variant-numeric:tabular-nums; }
    .vehicle-light-head { display:flex; align-items:center; justify-content:space-between; color:#8996a3; font-size:12px; }
    .vehicle-light-head .vehicle-label { color:#e1e6e9; font-size:15px; } .vehicle-light-head svg { color:#f5616c; }
    .vehicle-light-buttons { display:grid; grid-template-columns:1fr 1fr; gap:12px; margin-top:14px; }
    .ambient-red { display:flex; align-items:center; gap:12px; background:#2b2429; border:1px solid #523038; border-radius:10px; text-align:left; padding:14px; font-weight:500; }
    .ambient-red::before { content:''; width:7px; height:7px; border-radius:50%; background:#f5616c; } .ambient-red svg { margin-left:auto; color:#f5616c; width:18px; }
    .ambient-red:hover:not(:disabled) { background:#49272f; border-color:#f5616c; } .ambient-red:disabled { opacity:.4; }
    #ambient-result { color:#8996a3; font-size:12px; margin-top:12px; min-height:18px; }
    .vehicle-details { margin-top:12px; color:#8996a3; font-size:12px; line-height:1.8; } .vehicle-details summary { cursor:pointer; width:max-content; }
    .vehicle-details p { font-size:12px; color:#8996a3; margin:7px 0; }
    @media(max-width:600px) { #vehicle-panel { padding:18px; border-radius:18px; } .vehicle-body { grid-template-columns:1fr; gap:22px; }
      .vehicle-stats { border-left:0; border-top:1px solid #2a3037; padding:20px 0 0; grid-template-columns:1fr 1fr; gap:20px; }
      .vehicle-data-grid { grid-template-columns:repeat(2,1fr); }
      .vehicle-stats .vehicle-value { font-size:29px; } #vehicle-soc { font-size:48px; } .vehicle-hero { gap:20px; }
      .vehicle-car-stage { max-width:390px; width:100%; margin:auto; } .vehicle-value { font-size:32px; }
    }
  </style>
</head><body><main>
  <h1>车载设置</h1><p>连接设备局域网后可直接访问普通设置；任意 Bash 终端单独使用密码。</p>
  <div class="tabs"><button class="tab active" id="settings-tab" onclick="showPanel('settings')">设置</button><button class="tab" id="driving-tab" onclick="showPanel('driving')">行驶信息</button><button class="tab" id="logs-tab" onclick="showPanel('logs')">日志下载</button><button class="tab" id="navlogs-tab" onclick="showPanel('navlogs')">导航日志</button><button class="tab" id="turn-tab" onclick="showPanel('turn')">Tesla 验证</button><button class="tab" id="terminal-tab" onclick="showPanel('terminal')">终端</button><a class="tab" id="gateway-link" href="/gateway/">CAN 网关</a></div>
  <section id="vehicle-panel" aria-label="Tesla 车辆信息">
    <div class="vehicle-head">TESLA <span id="vehicle-ip">IP —</span><span id="vehicle-connection">等待车辆</span></div><div id="vehicle-metrics"></div>
    <div class="vehicle-lights"><div class="vehicle-light-head"><div class="vehicle-label"><svg class="vehicle-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M8 16c0-3-3-3-3-7a7 7 0 0 1 14 0c0 4-3 4-3 7M8 19h8m-6 3h4"/></svg>氛围灯</div><span>红色 · 3 秒</span></div>
    <div class="vehicle-light-buttons"><button class="ambient-red" onclick="runAmbient('left')" aria-label="左侧氛围灯，红色测试三秒">左侧<svg class="vehicle-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M4 12h16m-6-6 6 6-6 6"/></svg></button><button class="ambient-red" onclick="runAmbient('right')" aria-label="右侧氛围灯，红色测试三秒">右侧<svg class="vehicle-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M4 12h16m-6-6 6 6-6 6"/></svg></button></div></div>
    <div id="ambient-result" role="status">收到原车氛围灯信号后可测试</div>
    <details class="vehicle-details"><summary>详细信息</summary><p id="vehicle-extra"></p><p id="ambient-detail">offroad 和 onroad 均可测试。固定红色，10 Hz，持续 3 秒，最多 30 帧；实际变色请观察灯带。</p></details>
  </section>
  <section id="settings-panel"><div id="mode" class="notice">正在读取设置…</div><div id="category-nav" class="category-nav"></div><div id="settings"></div></section>
  <section id="driving-panel" hidden><h1>行驶道路视图</h1><p>只读实时视图；融合 SP 模型与 HW4 Model Y 原车 CAN，不启动视频或屏幕采集。</p><div id="driving-state" class="notice">正在连接车辆数据…</div><div class="ped-coordinate-lab"><strong>行人坐标</strong><select id="pedestrian-coordinate-mode" onchange="setPedestrianCoordinateMode(this.value)"><option value="off">关闭（默认）</option><option value="dx_forward_dy_left">dX 前后 / dY 左右</option><option value="dx_forward_dy_right">dX 前后 / -dY 左右</option><option value="dy_forward_dx_left">dY 前后 / dX 左右</option><option value="dy_forward_dx_right">dY 前后 / -dX 左右</option></select><span>黄色/蓝色/粉色对应行人 #1/#2/#3；坐标单位为米。</span></div><canvas id="driving-canvas" aria-label="预测道路轨迹与原车感知"></canvas><div class="chart-grid"><article class="chart-card"><h2>横向 · OEM 0x488 / SP</h2><p>原车请求、SP 最终输出与实际方向盘转角；单位 deg，最近 60 秒。</p><canvas class="signal-chart" id="lateral-chart"></canvas></article><article class="chart-card"><h2>纵向加速度 · 0x2B9 / 0x209 / SP</h2><p>巡航上下限、FSD velocity profile、SP 规划/命令与实车；单位 m/s²。</p><canvas class="signal-chart" id="accel-chart"></canvas></article><article class="chart-card"><h2>纵向目标速度 · 0x2B9 / 0x209 / SP</h2><p>原车巡航、FSD 未来目标和 SP 速度输出；单位 km/h。</p><canvas class="signal-chart" id="speed-chart"></canvas></article><article class="chart-card"><h2>SP / FSD 车道线</h2><p>20m 前视横向偏移；SP modelV2 始终可用，FSD 0x239 仅在 CH 报文真实到达时显示。</p><canvas class="signal-chart" id="lane-chart"></canvas></article></div><div class="log-actions"><button id="unknown-export" onclick="exportUnknownCan()">导出 0x37A / 0x3A9 / 0x3B1 采样 JSON</button></div><div id="unknown-can-state" class="notice">后台采集最近 60 秒；等待目标报文。</div><details id="can-diagnostics" class="can-diagnostics"><summary>CAN 诊断详情（可选）</summary><div id="can-details" class="can-grid"></div></details><div id="driving-alert" class="notice drive-alert" hidden></div></section>
  <section id="turn-panel" hidden>
    <h1>Tesla CAN 验证</h1><p>转向请求由 card 实时线程跟随原车 0x3E9 模板发送；速度按钮使用新鲜的原车 0x3C2 模板。所有发送仍受 Panda safety 限制。</p>
    <div class="buttons"><button class="turn" id="left" onclick="runTurn('left')">← 左转</button><button class="turn" id="right" onclick="runTurn('right')">右转 →</button></div>
    <div class="buttons"><button onclick="runSpeed('decrease')">− 速度按钮</button><button onclick="runSpeed('increase')">+ 速度按钮</button></div>
    <button id="cancel" onclick="cancelSession()">立即取消</button><div id="status"></div>
  </section>
  <section id="logs-panel" hidden>
    <h1>日志下载与清理</h1><p>导出仅包含低频 qlog、本地功能独立诊断、无法进入 onroad 的阻挡记录和当前 boot 的系统错误日志。rlog 与摄像头视频永远不会进入 ZIP；行驶中禁止操作。</p>
    <div id="log-state" class="notice">正在读取可用日志时间…</div>
    <div class="log-range"><label>开始时间<input id="log-start" type="datetime-local" onchange="previewLogs()"></label><label>结束时间<input id="log-end" type="datetime-local" onchange="previewLogs()"></label></div>
    <div class="log-actions"><button onclick="previewLogs()">刷新范围</button><button id="log-download" onclick="downloadLogs()" disabled>打包并下载</button><button id="log-delete" onclick="deleteLogs()" disabled>清理所选日志</button></div>
    <div id="log-preview" class="notice">尚未选择日志范围</div>
  </section>
  <!-- NAVIGATION_LOG_PANEL -->
  <section id="terminal-panel" hidden>
    <h1>设备终端</h1><p>仅在设置模式（非行驶状态）且设备端显式启用后可用。终端单独使用密码；命令最长 20 秒，输出上限 64 KiB。</p>
    <div id="terminal-state" class="notice">正在检查终端状态…</div><div class="terminal-row"><input id="terminal-password" type="password" autocomplete="off" placeholder="终端密码"><button onclick="runTerminal()">运行</button></div><div class="terminal-row"><input id="terminal-new-password" type="password" autocomplete="new-password" placeholder="新密码（4-64个字符）"><button onclick="changeTerminalPassword()">修改密码</button></div>
    <textarea id="terminal-command" spellcheck="false" placeholder="git status --short"></textarea><pre id="terminal-output"></pre>
  </section>
<script>
let settingsState = null, hotspotState = null, selectedCategory = null, currentPanel = 'settings', drivingLoading = false, logsStatus = null, logsInitialized = false, logsPreviewValid = false, lastDrivingData = null;
const signalHistory = [], SIGNAL_WINDOW_MS = 60000;
let ambientBusy = false, vehicleLoading = false, ambientResultShown = false;
function renderVehicle(data) {
  const v=data.vehicle||{}, root=document.getElementById('vehicle-metrics');
  if(!root.children.length) root.innerHTML=`
    <div class="vehicle-hero"><div><div class="vehicle-label"><svg class="vehicle-icon" viewBox="0 0 24 24" aria-hidden="true"><rect x="2" y="6" width="18" height="12" rx="2"/><path d="M23 10v4M6 9v6m4-6v6m4-6v6"/></svg>电量</div><div class="vehicle-value" id="vehicle-soc">—</div><div class="battery-track"><div id="vehicle-battery-fill"></div></div></div>
    <div><div class="vehicle-label">续航</div><div class="vehicle-value" id="vehicle-range">—</div><div class="vehicle-note" id="vehicle-range-note"></div></div></div>
    <div class="vehicle-body"><div class="vehicle-car-stage" aria-label="四轮胎压位置图">
    <svg class="vehicle-car" viewBox="0 0 140 270" aria-hidden="true"><rect x="8" y="54" width="10" height="42" rx="4" fill="#050709"/><rect x="122" y="54" width="10" height="42" rx="4" fill="#050709"/><rect x="8" y="199" width="10" height="39" rx="4" fill="#050709"/><rect x="122" y="199" width="10" height="39" rx="4" fill="#050709"/>
    <path d="M70 5C26 5 15 28 15 68v153c0 31 14 43 55 43s55-12 55-43V68C125 28 114 5 70 5Z" fill="#b2bbc4" stroke="#46505b" stroke-width="3"/>
    <path d="M23 63h94l-12 41H35Z" fill="#111418"/><rect x="34" y="106" width="72" height="80" rx="8" fill="#262c33"/><path d="M35 190h70l12 29H23Z" fill="#111418"/>
    <path d="M29 35h82" stroke="#e5edf2" stroke-width="3"/><path d="M28 241h25m34 0h25" stroke="#f5616c" stroke-width="3"/>
    <path id="vehicle-light-left" d="M22 108v76" stroke="#71818f" stroke-width="3"/><path id="vehicle-light-right" d="M118 108v76" stroke="#71818f" stroke-width="3"/></svg>
    <div class="vehicle-wheel"><small>左前</small><strong id="pressure-0">—</strong><small id="tpms-battery-0">电池 —</small></div><div class="vehicle-wheel"><small>右前</small><strong id="pressure-1">—</strong><small id="tpms-battery-1">电池 —</small></div><div class="vehicle-wheel"><small>左后</small><strong id="pressure-2">—</strong><small id="tpms-battery-2">电池 —</small></div><div class="vehicle-wheel"><small>右后</small><strong id="pressure-3">—</strong><small id="tpms-battery-3">电池 —</small></div></div>
    <div class="vehicle-stats"><div><div class="vehicle-label"><svg class="vehicle-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="m6 2-3 20M18 2l3 20M12 3v5m0 8v5"/></svg>总里程</div><div class="vehicle-value" id="vehicle-odometer">—</div><div class="vehicle-note">km</div></div>
    <div><div class="vehicle-label"><svg class="vehicle-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="m14 2-9 12h7l-2 8 9-12h-7Z"/></svg>累计放电</div><div class="vehicle-value" id="vehicle-discharge">—</div><div class="vehicle-note">kWh</div></div>
    <div><div class="vehicle-label">累计充电</div><div class="vehicle-value" id="vehicle-charge">—</div><div class="vehicle-note">kWh</div></div></div></div>
    <div class="vehicle-data-grid"><div class="vehicle-data-cell"><small>高压电池电压</small><strong id="hv-voltage">—</strong></div><div class="vehicle-data-cell"><small>高压电池电流</small><strong id="hv-current">—</strong></div><div class="vehicle-data-cell"><small>高压电池功率</small><strong id="hv-power">—</strong></div><div class="vehicle-data-cell"><small>高压状态</small><strong id="hv-state">—</strong></div></div>
    <div class="vehicle-data-grid"><div class="vehicle-data-cell"><small>左前刹车</small><strong id="brake-0">—</strong></div><div class="vehicle-data-cell"><small>右前刹车</small><strong id="brake-1">—</strong></div><div class="vehicle-data-cell"><small>左后刹车</small><strong id="brake-2">—</strong></div><div class="vehicle-data-cell"><small>右后刹车</small><strong id="brake-3">—</strong></div></div>`;
  const set=(id,value)=>document.getElementById(id).textContent=value;
  set('vehicle-connection',[v.soc,v.odometer].some(value=>value&&value!=='—')?'车辆信息':'等待车辆');set('vehicle-soc',v.soc||'—');
  const soc=parseFloat(v.soc);document.getElementById('vehicle-battery-fill').style.width=(Number.isFinite(soc)?Math.max(0,Math.min(100,soc)):0)+'%';
  set('vehicle-ip','IP '+(data.device_ip||'—'));
  set('vehicle-range','—');set('vehicle-range-note',v.range==='待核实单位'?'单位待核实':'等待数据');
  set('vehicle-odometer',(v.odometer||'—').replace(' km',''));set('vehicle-discharge',(v.discharge||'—').replace(' kWh',''));set('vehicle-charge',(v.charge||'—').replace(' kWh',''));
  for(let i=0;i<4;i++){const w=(v.pressure||[])[i]||{};set('pressure-'+i,(w.text||'—').replace(' bar',''));set('tpms-battery-'+i,'电池 '+(w.battery||'—'));document.getElementById('pressure-'+i).parentElement.classList.toggle('warn',!!w.warning);}
  const hv=v.high_voltage||{};set('hv-voltage',hv.voltage||'—');set('hv-current',hv.current||'—');set('hv-power',hv.power||'—');set('hv-state',hv.hv_state||'—');
  for(let i=0;i<4;i++)set('brake-'+i,(v.brakes||[])[i]?.text||'—');
  set('vehicle-extra',(v.range_note||'等待续航信号')+'；接触器 '+(hv.contactor||'—')+'；电池状态 '+(hv.state||'—')+'；充电 '+(hv.charge_status||'—'));
  document.querySelectorAll('.ambient-red').forEach(b=>{b.disabled=ambientBusy;b.classList.toggle('unready',!data.ambient_test_available);});
  if(!ambientResultShown&&!ambientBusy)set('ambient-result',data.ambient_test_available?'可测试':'等待原车氛围灯信号');
}
renderVehicle({});

async function loadVehicle() {
  if(vehicleLoading)return;vehicleLoading=true;
  try{const r=await apiFetch('/api/vehicle',{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);renderVehicle(await r.json());}
  catch(e){renderVehicle({});document.getElementById('ambient-result').textContent='等待车辆连接';}
  finally{vehicleLoading=false;}
}
async function runAmbient(side) {
  if(ambientBusy)return;ambientBusy=true;ambientResultShown=true;document.querySelectorAll('.ambient-red').forEach(b=>b.disabled=true);
  const status=document.getElementById('ambient-result');status.textContent=(side==='left'?'左侧':'右侧')+' · 正在发送 3 秒';document.getElementById('vehicle-light-'+side)?.setAttribute('stroke','#f5616c');
  try{const r=await apiFetch('/api/tesla/ambient',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({side})});const result=await r.json();status.textContent=({sent:'测试结束 · 请观察灯带',no_echo:'未收到回显',rejected:'发送被拒绝',blocked:'测试已停止',timeout:'测试超时'})[result.state]||(r.ok?'测试结束':result.message||'暂不可测试');document.getElementById('ambient-detail').textContent=result.message||'未知结果';}
  catch(e){status.textContent='连接失败';document.getElementById('ambient-detail').textContent='不能确认发送：'+e;}finally{ambientBusy=false;document.getElementById('vehicle-light-'+side)?.setAttribute('stroke','#71818f');loadVehicle();}
}
setInterval(()=>{if(!document.hidden)loadVehicle();},1000);loadVehicle();
function apiFetch(url, options = {}) { return fetch(url, options); }
let pedestrianCoordinateMode = localStorage.getItem('pedestrianCoordinateMode') || 'off';
function setPedestrianCoordinateMode(value) { pedestrianCoordinateMode = value; localStorage.setItem('pedestrianCoordinateMode', value); loadDrivingStatus(); }
document.getElementById('pedestrian-coordinate-mode').value = pedestrianCoordinateMode;
function showPanel(name) {
  currentPanel = name;
  document.getElementById('navlogs-panel').hidden=name!=='navlogs';document.getElementById('navlogs-tab').classList.toggle('active',name==='navlogs');
  if(name==='navlogs')loadNavigationLogs(true);
  document.getElementById('settings-panel').hidden = name !== 'settings'; document.getElementById('driving-panel').hidden = name !== 'driving'; document.getElementById('logs-panel').hidden = name !== 'logs'; document.getElementById('turn-panel').hidden = name !== 'turn'; document.getElementById('terminal-panel').hidden = name !== 'terminal';
  document.getElementById('settings-tab').classList.toggle('active', name === 'settings'); document.getElementById('driving-tab').classList.toggle('active', name === 'driving'); document.getElementById('logs-tab').classList.toggle('active', name === 'logs'); document.getElementById('turn-tab').classList.toggle('active', name === 'turn'); document.getElementById('terminal-tab').classList.toggle('active', name === 'terminal');
  if (name === 'driving') loadDrivingStatus();
  if (name === 'logs') loadLogStatus();
}
function element(tag, attrs = {}, text = '') { const e = document.createElement(tag); Object.assign(e, attrs); if (text) e.textContent = text; return e; }
function renderSettings(data) {
  settingsState = data; const mode = document.getElementById('mode'); mode.textContent = data.onroad ? '行驶中：只允许修改标注“行驶中可调”的设置。' : '设置模式：可修改全部白名单设置。'; mode.className = 'notice' + (data.onroad ? ' onroad' : '');
  const categories = data.menu; if (!selectedCategory || !categories.includes(selectedCategory)) selectedCategory = categories[0];
  const nav = document.getElementById('category-nav'); nav.replaceChildren(); categories.forEach(category => { const button = element('button', {className:'category' + (category === selectedCategory ? ' active' : '')}, category); button.onclick = () => { selectedCategory = category; renderSettings(data); }; nav.append(button); });
  const root = document.getElementById('settings'); root.replaceChildren(); let group = '';
  data.settings.filter(setting => setting.category === selectedCategory).sort((a,b) => a.group.localeCompare(b.group) || a.title.localeCompare(b.title)).forEach(setting => {
    if (setting.group !== group) { group = setting.group; root.append(element('div', {className:'group'}, group)); }
    const card = element('div', {className:'card'}), description = element('div'), title = element('h2', {}, setting.title || setting.key);
    description.append(title); if (setting.description) description.append(element('p', {}, setting.description));
    const locked = (data.onroad && setting.offroad_only) || setting.enabled === false;
    if (data.onroad && setting.offroad_only) description.append(element('div', {className:'lock'}, '仅设置模式可调'));
    else if (setting.enabled === false) description.append(element('div', {className:'lock'}, '当前规划器不支持此参数'));
    let control;
    if (setting.widget === 'toggle') { control = element('input', {type:'checkbox', checked:setting.value, disabled:locked}); control.onchange = () => save(setting, control.checked, control); }
    else if (setting.options) { control = element('select', {disabled:locked}); setting.options.forEach(option => control.append(element('option', {value:String(option.value), selected:option.value === setting.value}, option.label))); control.onchange = () => save(setting, control.value === '' ? '' : Number(control.value), control); }
    else { control = element('input', {type:'number', value:setting.value, min:setting.min ?? '', max:setting.max ?? '', step:setting.step ?? 1, disabled:locked}); control.onchange = () => save(setting, Number(control.value), control); }
    control.title = setting.key; card.append(description, control); root.append(card);
  });
  if (selectedCategory === '设备' && hotspotState?.available) {
    const card = element('div', {className:'card'}), description = element('div');
    description.append(element('h2', {}, '设备 Wi-Fi 热点'));
    const accessText = '普通设备：' + hotspotState.url + '；Tesla：' + hotspotState.tesla_url;
    const readiness = hotspotState.tesla_address_ready ? '' : '（Tesla 地址正在准备）';
    description.append(element('p', {}, hotspotState.active ? '热点已开启。' + accessText + readiness : '开启后连接热点并访问。' + accessText));
    const control = element('input', {type:'checkbox', checked:hotspotState.active});
    control.onchange = () => saveHotspot(control.checked, control); card.append(description, control); root.append(card);
  }
}
async function loadSettings() { try { const [settingsResponse, hotspotResponse] = await Promise.all([apiFetch('/api/settings', {cache:'no-store'}), apiFetch('/api/hotspot', {cache:'no-store'})]); if (!settingsResponse.ok) throw new Error('HTTP ' + settingsResponse.status); hotspotState = hotspotResponse.ok ? await hotspotResponse.json() : null; renderSettings(await settingsResponse.json()); } catch (e) { document.getElementById('mode').textContent = '设置读取失败：' + e; } }
async function save(setting, value, control) { control.disabled = true; try { const r = await apiFetch('/api/settings/' + encodeURIComponent(setting.key), {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({value})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); setting.value = result.value; await loadSettings(); } catch (e) { alert('保存失败：' + e); } finally { if (settingsState) renderSettings(settingsState); } }
async function saveHotspot(enabled, control) { control.disabled = true; try { const r = await apiFetch('/api/hotspot', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({enabled})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); hotspotState = result; renderSettings(settingsState); } catch (e) { alert('热点切换失败：' + e); renderSettings(settingsState); } }
loadSettings();
function localTimeInput(ms) { const d = new Date(ms), local = new Date(d.getTime() - d.getTimezoneOffset() * 60000); return local.toISOString().slice(0,16); }
function selectedLogRange() { const start = new Date(document.getElementById('log-start').value).getTime(), end = new Date(document.getElementById('log-end').value).getTime(); return {start,end}; }
function formatBytes(bytes) { if (!Number.isFinite(bytes)) return '—'; const units=['B','KiB','MiB','GiB','TiB']; let value=bytes, unit=0; while(value>=1024&&unit<units.length-1){value/=1024;unit++;} return (unit?value.toFixed(value>=10?1:2):String(value))+' '+units[unit]; }
async function loadLogStatus() { const state=document.getElementById('log-state'),button=document.getElementById('log-download'),deleteButton=document.getElementById('log-delete');try{const response=await apiFetch('/api/logs/status',{cache:'no-store'}),data=await response.json();if(!response.ok)throw new Error(data.message||'HTTP '+response.status);logsStatus=data;state.className='notice'+(data.onroad?' onroad':'');if(!data.available){state.textContent='没有找到可下载的 qlog 或本地诊断';button.disabled=true;deleteButton.disabled=true;return;}state.textContent=(data.onroad?'行驶中：仅可查看范围，禁止下载或清理。':'设置模式：可以打包下载或清理。')+' 可用范围：'+new Date(data.start_ms).toLocaleString()+' → '+new Date(data.end_ms).toLocaleString()+' · '+data.segment_count+' 个路线段 · '+data.local_diagnostic_count+' 个本地诊断文件';if(!logsInitialized){const end=data.end_ms,start=Math.max(data.start_ms,end-30*60*1000);document.getElementById('log-start').value=localTimeInput(start);document.getElementById('log-end').value=localTimeInput(end);logsInitialized=true;}await previewLogs();}catch(error){state.className='notice onroad';state.textContent='日志范围读取失败：'+error;button.disabled=true;deleteButton.disabled=true;} }
async function previewLogs() { const preview=document.getElementById('log-preview'),button=document.getElementById('log-download'),deleteButton=document.getElementById('log-delete'),range=selectedLogRange();logsPreviewValid=false;button.disabled=true;deleteButton.disabled=true;if(!Number.isFinite(range.start)||!Number.isFinite(range.end)||range.end<=range.start){preview.className='notice onroad';preview.textContent='请选择有效的开始和结束时间';return;}try{const response=await apiFetch('/api/logs/preview?start_ms='+range.start+'&end_ms='+range.end,{cache:'no-store'}),data=await response.json();if(!response.ok)throw new Error(data.message||'HTTP '+response.status);preview.className='notice';preview.textContent='qlog '+data.route_file_count+' 个 · 本地诊断 '+data.local_diagnostic_count+' 个 · '+formatBytes(data.total_bytes)+'\\n另附 onroad 阻挡记录和系统错误日志（最多 '+formatBytes(data.max_system_diagnostic_bytes)+'）；不包含 rlog 或视频。';logsPreviewValid=Boolean(data.includes_system_diagnostics)||data.file_count>0;const onroad=!logsStatus||Boolean(logsStatus.onroad);button.disabled=!logsPreviewValid||onroad;deleteButton.disabled=data.file_count<=0||onroad;}catch(error){preview.className='notice onroad';preview.textContent='日志范围无效：'+error;} }
function downloadLogs() { if(!logsPreviewValid||logsStatus?.onroad)return;const range=selectedLogRange();window.location.assign('/api/logs/download?start_ms='+range.start+'&end_ms='+range.end); }
/* NAVIGATION_LOG_SCRIPT */
async function deleteLogs() { if(!logsPreviewValid||logsStatus?.onroad)return;const range=selectedLogRange();if(!confirm('确定永久删除所选时间范围内的 qlog、本地诊断及遗留 rlog？视频不会删除，此操作无法撤销。'))return;const button=document.getElementById('log-delete'),preview=document.getElementById('log-preview');button.disabled=true;preview.className='notice';preview.textContent='正在清理所选日志…';try{const response=await apiFetch('/api/logs/delete',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({start_ms:range.start,end_ms:range.end,confirm:true})}),data=await response.json();if(!response.ok)throw new Error(data.message||'HTTP '+response.status);logsInitialized=false;await loadLogStatus();preview.className=data.skipped_files?.length?'notice onroad':'notice';preview.textContent='已清理 '+data.file_count+' 个日志文件 · 释放 '+formatBytes(data.total_bytes)+(data.skipped_files?.length?'\\n跳过 '+data.skipped_files.length+' 个已变化或不安全的文件。':'');}catch(error){preview.className='notice onroad';preview.textContent='日志清理失败：'+error;await previewLogs();} }
function drawLine(ctx, points, xScale, yScale, color, width) { if (!points.length) return; ctx.beginPath(); points.forEach(([x,y], i) => { const px = ctx.canvas.clientWidth / 2 - y * yScale, py = ctx.canvas.clientHeight - 38 - x * xScale; i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); }); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); }
function drawModelLine(ctx, points, xScale, yScale, color, width) { if (!points.length) return; ctx.beginPath(); points.forEach(([x,y], i) => { const px = ctx.canvas.clientWidth / 2 + y * yScale, py = ctx.canvas.clientHeight - 38 - x * xScale; i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); }); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); }
const canText = {
  lead:'前方', left:'左侧', right:'右侧', cutin:'切入', car:'车辆', truck:'卡车', motorcycle:'摩托车', bicycle:'自行车', pedestrian:'行人', unknown:'未知',
  red:'红灯', green:'绿灯', yellow:'黄灯', white:'白灯', off:'信号灯熄灭', none:'无', stop_sign:'停止标志', traffic_light:'红绿灯', yield:'让行', crosswalk:'人行横道', pedestrian_crossing:'行人过街', ramp_meter:'匝道灯', speed_bump:'减速带', speed_hump:'减速丘',
  disabled:'禁用', unavailable:'不可用', available:'可用', active:'激活', standby:'待机', aware:'感知', warning:'警告', stopping:'停车中', stopped:'已停车', continuing:'继续通行',
  map:'地图', vision:'视觉', map_and_vision:'地图+视觉', navigation:'导航', fused:'已融合', rejected:'拒绝', blacklisted:'黑名单', nominal:'正常', degraded:'降级', severely_degraded:'严重降级', fault:'故障', normal:'普通路面', enhanced:'增强路面',
  regular:'常规限速', advisory:'建议限速', dependent:'条件限速', bumps:'减速设施', class_1_major:'一级主干道', class_2:'二级道路', class_3:'三级道路', class_4:'四级道路', class_5:'五级道路', class_6_minor:'六级支路', circle:'圆形', straight:'直行',
  camera_detection:'相机区域检测', positioned_object:'CH 定位对象',
  in_lane:'车道内', lane_change_left:'向左变道', lane_change_right:'向右变道', virtual_lane:'虚拟车道', follow:'跟车', lane_change_requested:'请求变道', lane_change_in_progress:'变道中', waiting_side_obstacle:'等待侧方车辆', waiting_forward_obstacle:'等待前方车辆', lane_change_abort:'变道中止',
  open:'断开', opening:'正在断开', closing:'正在闭合', closed:'已闭合', welded:'触点粘连', blocked:'被阻止', disconnected:'未连接', no_power:'无电源', about_to_charge:'准备充电', charging:'充电中', charge_complete:'充电完成', charge_stopped:'充电停止', calibrating:'校准中', drive:'行驶', support:'供电支持', diagnostic:'诊断', down:'高压关闭', coming_up:'高压上电中', going_down:'高压下电中', up_for_drive:'行驶高压就绪', up_for_charge:'充电高压就绪', up_for_dc_charge:'直流充电高压就绪', up:'高压已开启', front_left:'左前', front_right:'右前', rear_left:'左后', rear_right:'右后', solid:'常亮', flashing:'闪烁', wait_for_stationary:'等待静止', ready:'就绪', irrational:'异常', rational:'正常', initializing:'初始化', auto:'自动', low:'低矮', high:'较高', no_object:'无障碍', pcb:'控制板', inverter:'逆变器', stator:'定子', dc_capacitor:'直流电容', heatsink:'散热器', heatsink_1:'散热器1', heatsink_2:'散热器2', heatsink_3:'散热器3', pcb_2:'控制板2', junction:'结温', t_pak_1:'功率模块1', t_pak_2:'功率模块2', inlet:'入口', stator_housing:'定子壳体', front_left_door:'左前门', front_right_door:'右前门', rear_left_door:'左后门', rear_right_door:'右后门', instrument_panel_left:'仪表台左', instrument_panel_right:'仪表台右'
};
function ct(value) { return canText[value] || String(value ?? '—').replaceAll('_', ' '); }
// The compact modelV2 top-down view and Tesla OEM diagnostics have opposite
// observed lateral signs. Keep separate transforms; sharing one mirrored SP.
function canvasPoint(canvas, x, y, xScale, yScale) { return [canvas.clientWidth / 2 - y * yScale, canvas.clientHeight - 38 - x * xScale]; }
function modelCanvasPoint(canvas, x, y, xScale, yScale) { return [canvas.clientWidth / 2 + y * yScale, canvas.clientHeight - 38 - x * xScale]; }
function roundedRect(ctx,x,y,w,h,r=8){const q=Math.min(r,w/2,h/2);ctx.beginPath();ctx.moveTo(x+q,y);ctx.lineTo(x+w-q,y);ctx.quadraticCurveTo(x+w,y,x+w,y+q);ctx.lineTo(x+w,y+h-q);ctx.quadraticCurveTo(x+w,y+h,x+w-q,y+h);ctx.lineTo(x+q,y+h);ctx.quadraticCurveTo(x,y+h,x,y+h-q);ctx.lineTo(x,y+q);ctx.quadraticCurveTo(x,y,x+q,y);ctx.closePath();}
function fillRoundedRect(ctx,x,y,w,h,color,r=8){roundedRect(ctx,x,y,w,h,r);ctx.fillStyle=color;ctx.fill();}
function drawRoadBackground(ctx,width,height,xScale){
  const sky=ctx.createLinearGradient(0,0,0,height);sky.addColorStop(0,'#13233b');sky.addColorStop(.42,'#0b1728');sky.addColorStop(1,'#020617');ctx.fillStyle=sky;ctx.fillRect(0,0,width,height);
  const horizon=58,roadBottom=Math.min(width*.47,330);const road=ctx.createLinearGradient(0,horizon,0,height);road.addColorStop(0,'#172033');road.addColorStop(1,'#090f1c');ctx.fillStyle=road;ctx.beginPath();ctx.moveTo(width/2-22,horizon);ctx.lineTo(width/2+22,horizon);ctx.lineTo(width/2+roadBottom,height);ctx.lineTo(width/2-roadBottom,height);ctx.closePath();ctx.fill();
  ctx.strokeStyle='#33415588';ctx.lineWidth=1;ctx.font='10px sans-serif';ctx.fillStyle='#64748b';for(const metres of [20,40,60,80]){const y=height-38-metres*xScale;if(y<=horizon)continue;const progress=(height-y)/(height-horizon),half=roadBottom-(roadBottom-22)*progress;ctx.beginPath();ctx.moveTo(width/2-half,y);ctx.lineTo(width/2+half,y);ctx.stroke();ctx.fillText(metres+'m',width/2+half+5,y+3);}
}
function drawObjectLabel(ctx,text,x,y,color){ctx.font='bold 11px sans-serif';const w=Math.ceil(ctx.measureText(text).width)+14;fillRoundedRect(ctx,x,y-15,w,21,'#020617dd',7);ctx.fillStyle=color;ctx.fillRect(x,y-15,3,21);ctx.fillStyle='#f8fafc';ctx.fillText(text,x+8,y);}
function drawCanVehicle(ctx, vehicle, xScale, yScale) {
  const [px, py] = canvasPoint(ctx.canvas, vehicle.x_m, vehicle.y_m, xScale, yScale); if (py < 52 || py > ctx.canvas.clientHeight - 20) return;
  const color = vehicle.relevant_for_control ? '#ef4444' : vehicle.category === 'left' ? '#60a5fa' : vehicle.category === 'right' ? '#c084fc' : '#f97316'; ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  if (vehicle.type === 'pedestrian') { ctx.beginPath(); ctx.arc(px, py - 6, 4, 0, Math.PI * 2); ctx.fill(); ctx.beginPath(); ctx.moveTo(px, py - 2); ctx.lineTo(px, py + 8); ctx.moveTo(px, py + 2); ctx.lineTo(px - 5, py + 7); ctx.moveTo(px, py + 2); ctx.lineTo(px + 5, py + 7); ctx.stroke(); }
  else if (vehicle.type === 'bicycle' || vehicle.type === 'motorcycle') { ctx.beginPath(); ctx.arc(px - 6, py + 4, 4, 0, Math.PI * 2); ctx.arc(px + 6, py + 4, 4, 0, Math.PI * 2); ctx.moveTo(px - 6, py + 4); ctx.lineTo(px, py - 4); ctx.lineTo(px + 6, py + 4); ctx.stroke(); }
  else { const w = vehicle.type === 'truck' ? 22 : 17, h = vehicle.type === 'truck' ? 29 : 22; fillRoundedRect(ctx,px-w/2,py-h/2,w,h,color,vehicle.type==='truck'?4:7);ctx.fillStyle='#0f172a';fillRoundedRect(ctx,px-w/2+3,py-h/2+3,w-6,6,'#0f172a',2); }
  const rel = Number(vehicle.relative_speed); const velocity = Number.isFinite(rel) ? '  Δv ' + rel.toFixed(0) : ''; drawObjectLabel(ctx,ct(vehicle.category)+' '+vehicle.x_m.toFixed(0)+'m'+velocity,px+12,py+4,color);
}
function drawHudChip(ctx, label, x, y, color = '#2563eb') {
  ctx.font='bold 12px sans-serif'; const w=Math.ceil(ctx.measureText(label).width)+18; fillRoundedRect(ctx,x,y-15,w,22,'#020617cc',8);ctx.fillStyle=color;ctx.fillRect(x,y-15,4,22);ctx.fillStyle='#f8fafc';ctx.fillText(label,x+10,y);return w;
}
function drawPedestrianCameraIndicators(ctx, pedestrian, width, height) {
  if (!pedestrian?.available) return;
  const front=pedestrian.front_main||pedestrian.front_fisheye||pedestrian.front_narrow, left=pedestrian.left_pillar||pedestrian.left_repeater, right=pedestrian.right_pillar||pedestrian.right_repeater;
  ctx.fillStyle='#fbbf24'; ctx.font='bold 13px sans-serif';
  if(front){ctx.textAlign='center';ctx.fillText('⚠ 前方相机发现行人',width/2,174);}
  if(left){ctx.textAlign='left';ctx.fillText('⚠ 左侧行人',12,height/2);}
  if(right){ctx.textAlign='right';ctx.fillText('右侧行人 ⚠',width-12,height/2);}
  if(pedestrian.backup){ctx.textAlign='center';ctx.fillText('⚠ 后方行人',width/2,height-58);}
  ctx.textAlign='left';
}
const pedestrianCoordinateModeLabels = {
  dx_forward_dy_left:'dX→前，dY→左', dx_forward_dy_right:'dX→前，-dY→左',
  dy_forward_dx_left:'dY→前，dX→左', dy_forward_dx_right:'dY→前，-dX→左'
};
function experimentalPedestrianPosition(slot, mode) {
  const dx=Number(slot.dx_scaled),dy=Number(slot.dy_scaled);if(!Number.isFinite(dx)||!Number.isFinite(dy))return null;
  if(mode==='dx_forward_dy_left')return {x:dx,y:dy};
  if(mode==='dx_forward_dy_right')return {x:dx,y:-dy};
  if(mode==='dy_forward_dx_left')return {x:dy,y:dx};
  if(mode==='dy_forward_dx_right')return {x:dy,y:-dx};
  return null;
}
function drawExperimentalPedestrianSlots(ctx, pedestrian, xScale, yScale, width, height) {
  if(pedestrianCoordinateMode==='off'||!pedestrian?.detected_any)return;
  const colors=['#fbbf24','#38bdf8','#f472b6'];
  const slots=(pedestrian.coordinate_slots||[]).map(slot=>({slot,pos:experimentalPedestrianPosition(slot,pedestrianCoordinateMode)})).filter(item=>item.pos&&!(item.pos.x===0&&item.pos.y===0));
  ctx.save();
  for(const {slot,pos} of slots){
    if(pos.x<0||pos.x>100||Math.abs(pos.y)>14)continue;
    const [px,py]=canvasPoint(ctx.canvas,pos.x,pos.y,xScale,yScale);if(py<52||py>height-20)continue;
    const color=colors[(slot.index-1)%colors.length];ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash([4,3]);ctx.beginPath();ctx.arc(px,py,9,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);drawObjectLabel(ctx,'行人 #'+slot.index+' '+pos.x.toFixed(1)+'m/'+pos.y.toFixed(1)+'m',px+12,py+4,color);
  }
  const size=Math.min(132,width*.36),left=width-size-12,top=height-size-48,cx=left+size/2,cy=top+size/2,scale=(size-24)/25.6;
  fillRoundedRect(ctx,left,top,size,size,'#020617e8',10);ctx.strokeStyle='#64748b';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(cx,top+8);ctx.lineTo(cx,top+size-8);ctx.moveTo(left+8,cy);ctx.lineTo(left+size-8,cy);ctx.stroke();ctx.fillStyle='#cbd5e1';ctx.font='10px sans-serif';ctx.fillText('+前',cx+4,top+12);ctx.fillText('+左',left+6,cy-4);ctx.fillText(pedestrianCoordinateModeLabels[pedestrianCoordinateMode],left+7,top+size-7);
  for(const {slot,pos} of slots){const px=cx-pos.y*scale,py=cy-pos.x*scale,color=colors[(slot.index-1)%colors.length];ctx.fillStyle=color;ctx.beginPath();ctx.arc(px,py,5,0,Math.PI*2);ctx.fill();ctx.font='bold 10px sans-serif';ctx.fillText('#'+slot.index,px+7,py+3);}
  if(!slots.length){ctx.fillStyle='#94a3b8';ctx.font='11px sans-serif';ctx.fillText('坐标槽均为 0',left+22,cy+4);}
  ctx.restore();
}
function drawParkingObstacle(ctx, obstacle, xScale, yScale, width, height) {
  if (!obstacle?.valid_obstacle) return;
  let px=width/2, py=height-48; if(obstacle.x_m!=null&&obstacle.y_m!=null){[px,py]=canvasPoint(ctx.canvas,obstacle.x_m,obstacle.y_m,xScale,yScale);} else if(obstacle.collision_side==='left')px=width/2-46;else if(obstacle.collision_side==='right')px=width/2+46;else if(obstacle.collision_side==='front')py=height-82;
  ctx.strokeStyle='#fb7185';ctx.lineWidth=3;ctx.beginPath();ctx.arc(px,py,10,0,Math.PI*2);ctx.stroke();drawObjectLabel(ctx,'障碍 '+obstacle.distance_m.toFixed(1)+'m',px+14,py+4,'#fb7185');
}
function drawCanvasSummary(ctx, can, width) {
  const vehicles=can.vehicles||[], pedestrian=can.pedestrian_detection||{}, traffic=can.traffic||{}, positionedPedestrians=vehicles.filter(v=>v.type==='pedestrian').length, chips=[];
  chips.push([can.available?'原车 CAN':'等待原车 CAN',can.available?'#34d399':'#94a3b8']);
  if(vehicles.length)chips.push(['目标 '+vehicles.length,'#f97316']);
  if(positionedPedestrians)chips.push(['CH 定位行人 '+positionedPedestrians,'#fbbf24']);
  else if(pedestrian.detected_any)chips.push(['相机检测到行人','#fbbf24']);
  if(traffic.control_available||traffic.light_observation_available){const lightColor=({red:'#ef4444',green:'#22c55e',yellow:'#facc15'})[traffic.light_state]||'#94a3b8';chips.push([ct(traffic.light_state)+(traffic.control_distance_m!=null?' '+traffic.control_distance_m.toFixed(0)+'m':''),lightColor]);}
  if(can.lanes?.available)chips.push(['车道 CAN','#22d3ee']);
  let x=12,y=116; for(const [label,color] of chips){const w=drawHudChip(ctx,label,x,y,color);x+=w+6;if(x>width-105){x=12;y+=28;}}
}
function renderOptionalCanDetails(can) {
  const diagnostics=document.getElementById('can-diagnostics'),root=document.getElementById('can-details');if(diagnostics.open)renderCanDetails(can);else if(root.childElementCount)root.replaceChildren();
}
function drawTraffic(ctx, traffic, xScale, width, height, mapSign) {
  const sign = traffic?.road_sign_available ? {distance: traffic.stop_line_distance_m, color: traffic.road_sign_color, arrow: traffic.road_sign_arrow, label: traffic.road_sign_type} : null;
  const control = traffic?.control_available || traffic?.light_observation_available ? {distance: traffic.control_distance_m, color: traffic.light_state, arrow: null, label: traffic.light_state, type: traffic.control_type} : null;
  const mapLight = mapSign && mapSign.traffic_light_stop_line_distance_m != null ? {distance: mapSign.traffic_light_stop_line_distance_m, color: null, arrow: null, label: 'traffic_light', type: null, map: true} : null;
  const directional = sign && (sign.arrow === 'left' || sign.arrow === 'right' || sign.arrow === 'straight');
  const src = directional && sign.distance != null ? sign : (control && control.distance != null ? control : (sign && sign.distance != null ? sign : (mapLight || null)));
  if (!src) return; const distance = src.distance; if (distance < 0 || distance > 100) return;
  const color = ({red:'#ef4444',green:'#22c55e',yellow:'#facc15',white:'#f8fafc'})[src.color] || ({red:'#ef4444',green:'#22c55e',yellow:'#facc15',red_yellow:'#f97316'})[src.color] || '#94a3b8'; const py = height - 32 - distance * xScale;
  ctx.strokeStyle = color; ctx.lineWidth = 5; ctx.beginPath(); ctx.moveTo(width / 2 - 78, py); ctx.lineTo(width / 2 + 78, py); ctx.stroke(); ctx.fillStyle = color; ctx.font = 'bold 13px sans-serif';
  let label = ''; if (['red','green','yellow','white','off','other'].includes(src.label)) label = ct(src.label); else if (src.label === 'traffic_light' || src.label === 'stop_sign') label = ct(src.label); else if (src.type === 'crosswalk' || src.type === 'pedestrian_crossing') label = ct(src.type); else label = '红绿灯';
  const arrow = ({left:' 左转', right:' 右转', straight:' 直行', circle:' 圆灯'})[src.arrow] || '';
  ctx.fillText(label + arrow + (src.map ? ' 地图' : '') + '  ' + distance.toFixed(0) + 'm', width / 2 + 86, py + 4);
  if (traffic.control_type === 'crosswalk' || traffic.control_type === 'pedestrian_crossing') { ctx.strokeStyle = '#f8fafc'; ctx.lineWidth = 2; for (let i=-3;i<=3;i++) { ctx.beginPath(); ctx.moveTo(width/2+i*17-6,py+8); ctx.lineTo(width/2+i*17+6,py+8); ctx.stroke(); } }
}
function renderCanDetails(can) {
  const root = document.getElementById('can-details'); root.replaceChildren(); if (!can?.available) { const box = element('div',{className:'can-detail warn'},'尚未收到当前线束可达的 HW4 CAN 报文'); root.append(box); }
  const add = (title, lines) => { const box = element('div',{className:'can-detail'}); box.append(element('strong',{},title)); lines.filter(Boolean).forEach(line => box.append(element('div',{},line))); root.append(box); };
  const val = (value, unit='', digits=2) => value == null ? '—' : Number(value).toFixed(digits).replace(/\\.00$/,'') + unit;
  const yn = value => value ? '是' : '否';
  const temps = values => Object.entries(values||{}).filter(([,value])=>value!=null).map(([name,value])=>ct(name)+' '+val(value,'°C')).join(' · ');
  const n = can.navigation || {}; if(n.available){const rejects = [['导航',n.reject_navigation],['HPP',n.reject_hpp],['左车道',n.reject_left_lane],['右车道',n.reject_right_lane],['左自由空间',n.reject_left_free_space],['右自由空间',n.reject_right_free_space],['自动转向',n.reject_autosteer],['手扶方向盘',n.reject_hands_on]].filter(([,value]) => value).map(([name]) => name).join('/'); add('导航 / 地图', [n.route_active ? '路线：已激活' : '路线：未激活', '地图导航：' + (n.nav_available ? '可用' : '不可用') + (n.nav_distance_m != null ? ' · ' + n.nav_distance_m + 'm' : ''), '道路匹配：' + (n.gps_road_match ? '是' : '否'), n.next_branch_distance_m != null ? '下一分支：' + n.next_branch_distance_m + 'm' + (n.next_branch_left_off_ramp ? ' 左出口' : n.next_branch_right_off_ramp ? ' 右出口' : '') : '', n.speed_limit_unlimited ? '限速：不限速' : n.speed_limit != null ? '限速：' + n.speed_limit + ' ' + n.speed_limit_unit.toUpperCase() + ' · ' + ct(n.speed_limit_type) : '', rejects ? '拒绝项：' + rejects : '', '数据源总线：' + (n.sources || []).join(' / ')]);}
  const l = can.lanes || {}; if(l.available)add('车道 · 0x239（CH）', ['宽度：' + l.width_m + 'm · 范围：' + l.view_range_m + 'm', '左线：' + (l.left_exists ? ct(l.left_usage) : '不存在'), '右线：' + (l.right_exists ? ct(l.right_usage) : '不存在'), '数据源总线：' + (l.bus || '—')]);
  const t = can.traffic || {}, trafficObserved=t.control_available||t.light_observation_available; add('交通控制 · 0x25D', ['状态：'+(trafficObserved?ct(t.control_type):'当前无有效控制'), '灯态：'+(trafficObserved?ct(t.light_state):'--')+' · 来源：'+(trafficObserved?ct(t.control_source):'--'), '距离：'+(trafficObserved&&t.control_distance_m!=null?t.control_distance_m.toFixed(0)+' m':'--')+' · state='+(t.control_frame_fresh?t.state_machine_code:'--'), '功能状态：'+(t.control_frame_fresh?ct(t.feature_state)+' ('+t.feature_state_code+')':'--'), '数据源：'+((t.sources||[]).join(' / ')||'等待 AP-PARTY')]);
  const vehicles = can.vehicles || []; if(vehicles.length)add('周边目标 · DAS_object 0x30A（CH）', [...vehicles.map(v => ct(v.category) + ' ' + ct(v.type) + ' #' + v.track_id + '：纵向 ' + v.x_m + 'm，横向 ' + v.y_m + 'm，相对速度值 ' + v.relative_speed), '最多 7 个目标；当前仅配置独立 CH 时可用']);
  const a = can.driver_assist || {}; if(a.available)add('驾驶辅助', ['规划：' + ct(a.planner_state) + ' · 行为 ' + ct(a.behavior), '健康：' + ct(a.health) + ' · 异常等级 ' + a.health_anomaly_level, '数据源总线：' + (a.bus || '—')]);
  const rs = can.road_sign || {}; if(rs.available)add('道路标志 · 0x218（CH）', [rs.traffic_light_stop_line_distance_m != null ? '红绿灯停止线：' + rs.traffic_light_stop_line_distance_m + 'm' : '', rs.stop_sign_stop_line_distance_m != null ? '停止标志停止线：' + rs.stop_sign_stop_line_distance_m + 'm' : '', '数据源总线：' + (rs.bus || '—')]);
  const p = can.pedestrian_detection || {}, cameraNames={front_main:'前主',front_fisheye:'前鱼眼',front_narrow:'前窄',left_pillar:'左柱',left_repeater:'左镜',right_pillar:'右柱',right_repeater:'右镜',backup:'后视'}; if(p.available){const cameraList=(p.active_cameras||[]).map(name=>cameraNames[name]||name), slots=p.coordinate_slots||[]; add('行人检测 · 0x400（VEH）', ['相机区域：' + (cameraList.length?cameraList.join(' / '):'未检测到'), p.simultaneous_front_rear ? '原始前后位同时置位 · mask 0x'+Number(p.camera_mask||0).toString(16).padStart(2,'0') : '', slots.length ? '原始坐标槽：' + slots.map(s=>'#'+s.index+' dX '+s.dx_scaled+' / dY '+s.dy_scaled).join(' · ') : '', '坐标解析：' + (pedestrianCoordinateModeLabels[pedestrianCoordinateMode]||'关闭') + ' · 数据源：' + (p.bus || '—')]);}
  const f = can.front_safety || {}; if(f.available)add('前向安全 · 0x299（PARTY）', [f.valid_target ? '近距目标：' + f.target_distance_m + 'm' : '报文在线，当前无有效目标', f.relative_velocity_mps != null ? '相对速度 ' + f.relative_velocity_mps + ' m/s' : '', f.time_to_impact_s != null ? 'TTI ' + f.time_to_impact_s + 's' : '', '数据源总线：' + (f.bus || '—')]);
  const ac=can.actuation_commands||{},st=ac.steering||{},cr=ac.cruise||{};if(st.available)add('原车横向命令 · 0x488',['请求转角 '+val(st.angle_request_deg,'deg')+' · 控制类型 '+(st.control_type??'—'), '触觉请求 '+yn(st.haptic_request)+' · 数据源总线 '+(st.bus||'—')]);if(cr.available)add('原车巡航命令 · 0x2B9',['设定速度 '+val(cr.set_speed_kph,'km/h')+' · ACC 状态 '+(cr.acc_state??'—'), '加速度 '+val(cr.accel_min_mps2,'m/s²')+' → '+val(cr.accel_max_mps2,'m/s²'), 'Jerk '+val(cr.jerk_min_mps3,'m/s³')+' → '+val(cr.jerk_max_mps3,'m/s³'), '数据源总线：'+(cr.bus||'—')]);
  const unknown=can.unknown_frames||{};for(const frame of unknown.frames||[]){const changed=(frame.byte_stats||[]).filter(b=>b.changes).map(b=>'B'+b.index+' '+b.min+'→'+b.max+'（'+b.changes+'次）').join(' · ');add('未知 Tesla CAN · '+frame.address+' @ source'+frame.source,['最近 '+frame.sample_count+' 帧 / 累计 '+frame.total_count+' 帧 · '+(frame.frequency_hz??'—')+' Hz', '周期中位数 '+(frame.median_period_ms??'—')+' ms · DLC '+Object.entries(frame.dlc_counts||{}).map(([dlc,count])=>dlc+'×'+count).join(' / '), '最新：'+frame.latest_hex, changed?'变化字节：'+changed:'最近窗口内字节未变化', '近期：'+(frame.recent_samples||[]).slice(-3).map(s=>s.age_ms+'ms '+s.hex).join(' | ')]);}
  const lc = can.longitudinal_shadow || {}, vp = lc.velocity_profile || {}, tp = lc.torque_profiler || {}; if(lc.available)add('FSD 专属纵向 · 0x209', ['当前栈：' + (lc.current_stack || 'unknown') + '（' + (lc.current_stack_code ?? '—') + '）', tp.available ? '目标速度 ' + (tp.target_speed_kph ?? '—') + ' km/h · 加速度 '+val(tp.accel_min_mps2,'m/s²')+' → '+val(tp.accel_max_mps2,'m/s²') : '', vp.available ? '未来目标速度 ' + (vp.future_target_speed_kph ?? '—') + ' km/h · 当前加速度 '+val(vp.accel_mps2,'m/s²') : '', '数据源总线：' + (lc.bus || '—')]);
  const po = can.parking_obstacle || {}; if(po.available)add('泊车障碍 · 0x23E（VEH）', [po.valid_obstacle ? '距离 '+val(po.distance_m,'m')+' · 方向 '+ct(po.collision_side)+' · 高度 '+ct(po.height)+' · 置信度 '+po.confidence : '报文在线，当前无可信障碍', po.valid_obstacle ? '车辆坐标：X '+val(po.x_m,'m')+' · Y '+val(po.y_m,'m')+' · off-course '+(po.off_course??'—') : '', po.valid_obstacle ? '仅直接回波：'+yn(po.direct_echo_only)+' · 未跟踪 '+val(po.untracked_time_s,'s') : '', '数据源总线：' + (po.bus || '—')]);
  if(n.available)add('导航详细 · 0x238（VEH）', ['道路等级：'+ct(n.road_class)+' · controlled access：'+yn(n.controlled_access), '国家代码：'+n.country_code+' · street count：'+n.street_count, '限速原始依赖：'+n.speed_limit_dependency+' · Botts dots：'+yn(n.accept_botts_dots), '拒绝导航 '+yn(n.reject_navigation)+' · HPP '+yn(n.reject_hpp)+' · Autosteer '+yn(n.reject_autosteer)+' · Hands-on '+yn(n.reject_hands_on), '拒绝左/右车道：'+yn(n.reject_left_lane)+' / '+yn(n.reject_right_lane)+' · 左/右自由空间：'+yn(n.reject_left_free_space)+' / '+yn(n.reject_right_free_space), 'Autosteer 受限：'+yn(n.autosteer_restricted)+' · PMM：'+yn(n.pmm_enabled)+' · SCA：'+yn(n.sca_enabled), '平行泊车：'+yn(n.parallel_autopark_enabled)+' · 垂直泊车：'+yn(n.perpendicular_autopark_enabled)]);
  const rd=can.road_disturbance||{};if(rd.available)add('路面扰动 · 0x1FC（VEH）',['索引：'+rd.index+' · 高度 '+val(rd.height_m,'m'), '范围：X '+val(rd.x0_m,'m')+' → '+val(rd.x1_m,'m')+'（跨度 '+val(rd.longitudinal_span_m,'m')+'）', '范围：Y '+val(rd.y0_m,'m')+' → '+val(rd.y1_m,'m')+'（跨度 '+val(rd.lateral_span_m,'m')+'）', '悬架高度请求：'+rd.suspension_level_request+' · 数据源：'+(rd.bus||'—')]);
  const b=can.battery_diagnostics||{};if(b.available)add('高压电池 · 0x132 / 0x212（VEH）',['母线电压 '+val(b.dc_link_voltage_v,'V')+' · 电池电流 '+val(b.pack_current_a,'A')+' · 未滤波 '+val(b.current_unfiltered_a,'A'), 'BMS：'+ct(b.state)+'（'+(b.state_code??'—')+'） · 请求 '+ct(b.requested_state), '接触器：'+ct(b.contactor_state)+' · 高压：'+ct(b.hv_state)+' · 充电：'+ct(b.charge_status), '电池输入功率 '+val(b.battery_input_power_kw,'kW')+' · 可用充电功率 '+val(b.charge_power_available_kw,'kW'), '预热允许 '+yn(b.precondition_allowed)+' · 调节请求 '+yn(b.conditioning_request)+' · HVAC 请求 '+yn(b.hvac_power_request), '行驶功率不足 '+yn(b.not_enough_power_for_drive)+' · 支持功率不足 '+yn(b.not_enough_power_for_support)+' · limp '+yn(b.limp_request), '充电请求 '+yn(b.charge_request)+' · 重试 '+(b.charge_retry_count??'—')+' · PCS PWM '+yn(b.pcs_pwm_enabled), '更新允许 '+yn(b.update_allowed)+' · 充电口 HVS MIA '+yn(b.charge_port_missing_on_hv_system)+' · 空运/陆运许可 '+yn(b.ok_to_ship_by_air)+' / '+yn(b.ok_to_ship_by_land), '数据源：'+(b.sources||[]).join(' / ')]);
  const tm=can.tpms||{};if(tm.available){const wheelNames={front_left:'左前',front_right:'右前',rear_left:'左后',rear_right:'右后'};const wheelLines=Object.entries(tm.wheels||{}).map(([name,w])=>wheelNames[name]+': 显示 '+val(w.display_pressure_bar,'bar',3)+' · 直接 '+val(w.direct_pressure_bar,'bar',3)+' / '+val(w.direct_temperature_c,'°C')+' · 上次 '+val(w.last_known_pressure_bar,'bar',3)+(w.soft_warning?' · 软告警':'')+(w.hard_warning?' · 硬告警':''));const sensorLines=(tm.sensors||[]).map(s=>'传感器 #'+s.sensor_index+' '+ct(s.location)+': '+val(s.pressure_bar,'bar',3)+' / '+val(s.temperature_c,'°C')+' · 补偿 '+val(s.temperature_compensated_pressure_bar,'bar',3)+' · 变化率 '+val(s.pressure_rate)+' · 电池 '+val(s.battery_voltage_v,'V')+' · 广播压力 '+yn(s.pressure_in_advertisement)+' · 可配置 '+yn(s.configurable_pressure));add('胎压 · 0x219 / 0x25A / 0x31F（VEH）',[...wheelLines,...sensorLines,'冷胎建议：前 '+val(tm.recommended_cold_pressure_front_bar,'bar',3)+' · 后 '+val(tm.recommended_cold_pressure_rear_bar,'bar',3),'指示灯：'+ct(tm.telltale)+' · 功能 '+ct(tm.feature_state)+' · 接近状态 '+ct(tm.proximity_state)+' · 次数 '+(tm.feature_count??'—')+' · 时间 '+val(tm.feature_time_s,'s'),'Autonomy：'+ct(tm.autonomy_status)+' · MIA '+val(tm.autonomy_mia_time_s,'s'),'数据源：'+(tm.sources||[]).join(' / ')]);}
  const dp=can.drive_power||{};if(dp.available){const power=(label,p)=>p?.available?label+': 电功率 '+val(p.electrical_power_kw,'kW')+' · 最大驱动 '+val(p.drive_power_max_kw,'kW')+' · 实际热功率 '+val(p.heat_power_actual_kw,'kW')+' / 最优 '+val(p.heat_power_optimal_kw,'kW')+' / 最大 '+val(p.heat_power_max_kw,'kW')+' · 余热请求 '+val(p.excess_heat_command_kw,'kW'):'';add('前后驱动功率 · 0x266 / 0x2E5（VEH）',[power('前驱动',dp.front),power('后驱动',dp.rear),'单位来自信号语义，需与实车功率页复核 · 数据源：'+(dp.sources||[]).join(' / ')]);}
  const dt=can.drive_temperatures||{};if(dt.available){const side=(label,s)=>{const lines=[label+' mux页：'+(s.received_pages||[]).join(','),label+'运行：'+temps(s.operating_c),label+'温度百分比：逆变器 '+val(s.operating_percent?.inverter,'%')+' · 定子 '+val(s.operating_percent?.stator,'%'),label+'散热/功率模块：'+temps(s.heatsink_and_pack_c),label+'冷却液入口：'+val(s.fluid_in_c,'°C')+' · FET burn-in '+val(s.fet_burn_in?.normal)+' / '+val(s.fet_burn_in?.additional),label+'估算：'+temps(s.estimated_c),label+'寿命估算：当前 '+val(s.life_estimates?.current_weibull_miles,'mi',0)+' · 终点 '+val(s.life_estimates?.end_of_service_weibull_miles,'mi',0)+' · 损伤比 '+val(s.life_estimates?.burn_in_damage_ratio)];return lines;};add('驱动温度 · 0x315 / 0x376（VEH）',[...side('前',dt.front),...side('后',dt.rear),'数据源：'+(dt.bus||'—')]);}
  const vt=can.vehicle_totals||{};if(vt.available)add('里程 / 能量 / 刹车温度 · 0x3B6 / 0x3D2 / 0x3FE（VEH）',['里程 '+val(vt.odometer_km,'km',3)+' · OBD drive cycle '+yn(vt.obd_drive_cycle_active), '累计放电 '+val(vt.discharge_total_kwh,'kWh',3)+' · 累计充电 '+val(vt.charge_total_kwh,'kWh',3), '刹车温度：左前 '+val(vt.brake_temperature_c?.front_left,'°C')+' · 右前 '+val(vt.brake_temperature_c?.front_right,'°C')+' · 左后 '+val(vt.brake_temperature_c?.rear_left,'°C')+' · 右后 '+val(vt.brake_temperature_c?.rear_right,'°C'), 'MCP '+val(vt.mcp_index)+' · filtered '+val(vt.mcp_index_filtered), '数据源：'+(vt.sources||[]).join(' / ')]);
  const al=can.ambient_lighting||{};if(al.available)add('氛围灯 · 0x679（VEH）',['状态：'+ct(al.enable_state)+' · 强制供电 '+yn(al.power_override)+' · 亮度 '+(al.brightness??'—'), '颜色：'+al.hex_color+' · RGB '+al.rgb.red+'/'+al.rgb.green+'/'+al.rgb.blue, '效果：'+al.effect_code+'（'+al.effect_duration_ms+'ms） · 音频可视化 '+yn(al.audio_visualizer), '目标：'+((al.targets||[]).map(ct).join(' / ')||'无')+' · 数据源：'+(al.bus||'—')]);
}
function addComparisonSample(comparison) { if (!comparison) return; const now=Date.now(); signalHistory.push({t:now,c:comparison}); while(signalHistory.length&&signalHistory[0].t<now-SIGNAL_WINDOW_MS)signalHistory.shift(); }
function sizeSignalCanvas(canvas) { const ratio=window.devicePixelRatio||1,width=Math.max(1,canvas.clientWidth),height=Math.max(1,canvas.clientHeight);if(canvas.width!==width*ratio||canvas.height!==height*ratio){canvas.width=width*ratio;canvas.height=height*ratio;}const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);return{ctx,width,height}; }
function drawHistoryChart(id,series,unit) {
  const canvas=document.getElementById(id);if(!canvas)return;const {ctx,width,height}=sizeSignalCanvas(canvas);ctx.clearRect(0,0,width,height);ctx.fillStyle='#07111f';ctx.fillRect(0,0,width,height);const latest=signalHistory.at(-1)?.c;
  const columns=width<480?1:2,legendRows=Math.ceil(series.length/columns),left=45,right=9,top=10+legendRows*17,bottom=22,plotW=Math.max(1,width-left-right),plotH=Math.max(1,height-top-bottom),now=Date.now(),start=now-SIGNAL_WINDOW_MS;
  ctx.font='11px sans-serif';series.forEach((s,i)=>{const col=i%columns,row=Math.floor(i/columns),x=left+col*(plotW/columns),y=15+row*17,v=latest?s.value(latest):null;ctx.strokeStyle=s.color;ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(x,y-4);ctx.lineTo(x+14,y-4);ctx.stroke();ctx.fillStyle='#cbd5e1';ctx.fillText(s.name+' '+(Number.isFinite(v)?Number(v).toFixed(s.digits??2):'—'),x+19,y);});
  const all=[];for(const sample of signalHistory)for(const s of series){const v=s.value(sample.c);if(Number.isFinite(v))all.push(Number(v));}if(!all.length){ctx.fillStyle='#64748b';ctx.textAlign='center';ctx.font='13px sans-serif';ctx.fillText('等待对应报文 / SP 数据',width/2,top+plotH/2);ctx.textAlign='left';return;}
  let min=Math.min(0,...all),max=Math.max(0,...all);if(max-min<.1){min-=1;max+=1;}const pad=(max-min)*.08;min-=pad;max+=pad;const px=t=>left+Math.max(0,Math.min(1,(t-start)/SIGNAL_WINDOW_MS))*plotW,py=v=>top+(max-v)/(max-min)*plotH;
  ctx.lineWidth=1;ctx.font='10px sans-serif';for(let i=0;i<=4;i++){const y=top+i*plotH/4,v=max-i*(max-min)/4;ctx.strokeStyle='#33415588';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(width-right,y);ctx.stroke();ctx.fillStyle='#64748b';ctx.textAlign='right';ctx.fillText(v.toFixed(Math.abs(v)<10?1:0),left-5,y+3);}ctx.textAlign='left';ctx.fillStyle='#64748b';ctx.fillText('-60s',left,height-6);ctx.textAlign='right';ctx.fillText('现在 · '+unit,width-right,height-6);ctx.textAlign='left';
  for(const s of series){ctx.strokeStyle=s.color;ctx.lineWidth=s.width||2;ctx.beginPath();let drawing=false;for(const sample of signalHistory){const v=s.value(sample.c);if(!Number.isFinite(v)){drawing=false;continue;}const x=px(sample.t),y=py(Number(v));if(drawing)ctx.lineTo(x,y);else{ctx.moveTo(x,y);drawing=true;}}ctx.stroke();}
}
function drawComparisonCharts(comparison) {
  addComparisonSample(comparison);const l=c=>c.lateral||{},n=c=>c.longitudinal||{},q=c=>c.lanes||{};
  drawHistoryChart('lateral-chart',[{name:'OEM 488 对齐',color:'#f59e0b',value:c=>l(c).oem_0x488_angle_deg},{name:'SP 请求',color:'#60a5fa',value:c=>l(c).sp_request_angle_deg},{name:'SP 最终',color:'#22c55e',value:c=>l(c).sp_output_angle_deg,width:3},{name:'实际转角',color:'#e2e8f0',value:c=>l(c).actual_angle_deg}], 'deg');
  drawHistoryChart('accel-chart',[{name:'OEM 2B9 下限',color:'#fb7185',value:c=>n(c).oem_0x2b9_accel_min_mps2},{name:'OEM 2B9 上限',color:'#f59e0b',value:c=>n(c).oem_0x2b9_accel_max_mps2},{name:'FSD 209',color:'#c084fc',value:c=>n(c).fsd_0x209_accel_mps2},{name:'SP 规划',color:'#60a5fa',value:c=>n(c).sp_plan_accel_mps2},{name:'SP 命令',color:'#22c55e',value:c=>n(c).sp_command_accel_mps2,width:3},{name:'实车',color:'#e2e8f0',value:c=>n(c).actual_accel_mps2}], 'm/s²');
  drawHistoryChart('speed-chart',[{name:'OEM 0x2B9',color:'#f59e0b',value:c=>n(c).oem_0x2b9_set_speed_kph},{name:'FSD 0x209',color:'#c084fc',value:c=>n(c).fsd_0x209_target_speed_kph},{name:'SP 设定',color:'#22c55e',value:c=>n(c).sp_set_speed_kph,width:3},{name:'SP 轨迹',color:'#60a5fa',value:c=>n(c).sp_plan_speed_kph},{name:'实际车速',color:'#e2e8f0',value:c=>n(c).actual_speed_kph}], 'km/h');
  drawHistoryChart('lane-chart',[{name:'SP 左线',color:'#e2e8f0',value:c=>q(c).sp_left_offset_m},{name:'SP 路径',color:'#22c55e',value:c=>q(c).sp_path_offset_m,width:3},{name:'SP 右线',color:'#94a3b8',value:c=>q(c).sp_right_offset_m},{name:'FSD 左线',color:'#22d3ee',value:c=>q(c).oem_left_offset_m},{name:'FSD 中心',color:'#0891b2',value:c=>q(c).oem_center_offset_m,width:3},{name:'FSD 右线',color:'#06b6d4',value:c=>q(c).oem_right_offset_m}], 'm @20m');
}
function renderUnknownCan(can) { const capture=can?.unknown_frames||{},state=document.getElementById('unknown-can-state'),frames=capture.frames||[];if(!frames.length){state.className='notice onroad';state.textContent='后台采集最近 60 秒：尚未观察到 0x37A / 0x3A9 / 0x3B1。完整原始 CAN 仍保存在 rlog。';return;}state.className='notice';state.textContent='已捕获 '+frames.length+' 个 ID/总线组合：'+frames.map(f=>f.address+'@source'+f.source+' · '+f.sample_count+'帧 · '+(f.frequency_hz??'—')+'Hz · DLC '+Object.keys(f.dlc_counts).join('/')).join('；'); }
function exportUnknownCan() { const capture=lastDrivingData?.geometry?.oem_can?.unknown_frames;if(!capture){alert('尚无采样数据');return;}const blob=new Blob([JSON.stringify({exported_at:new Date().toISOString(),branch:lastDrivingData?.branch??null,capture},null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='tesla-unknown-can-'+new Date().toISOString().replaceAll(':','-')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000); }
function drawDrivingGeometry(geometry, data) {
  const canvas = document.getElementById('driving-canvas'), ratio = window.devicePixelRatio || 1, width = Math.max(1, canvas.clientWidth), height = Math.max(1, canvas.clientHeight); if (canvas.width !== width * ratio || canvas.height !== height * ratio) { canvas.width = width * ratio; canvas.height = height * ratio; }
  const ctx = canvas.getContext('2d'); ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, width, height); const xScale = (height - 78) / 100, yScale = Math.min(width / 12, 31), danger = geometry.hard_brake_predicted, can = geometry.oem_can || {}; drawRoadBackground(ctx,width,height,xScale);
  ctx.setLineDash([7,7]); (geometry.edges || []).forEach(line => drawModelLine(ctx,line,xScale,yScale,'#64748b',2)); ctx.setLineDash([]); (geometry.lanes || []).forEach(line => drawModelLine(ctx,line,xScale,yScale,'#e2e8f0',2));
  const oemLanes = can.lanes || {}; if (oemLanes.available) { drawLine(ctx,oemLanes.left||[],xScale,yScale,'#22d3ee',3); drawLine(ctx,oemLanes.right||[],xScale,yScale,'#22d3ee',3); ctx.setLineDash([5,5]); drawLine(ctx,oemLanes.center||[],xScale,yScale,'#0891b2',2); ctx.setLineDash([]); }
  drawModelLine(ctx, geometry.path || [], xScale, yScale, danger ? '#ef4444' : '#22c55e', 5); const legacyTraffic=geometry.oem_traffic||{}; const traffic=can.traffic?.available?can.traffic:legacyTraffic.available?{available:true,control_available:true,light_state:['none','red','green','yellow'][legacyTraffic.light_color]||'unknown',control_distance_m:legacyTraffic.stop_line_distance,control_type:'traffic_light'}:{}; drawTraffic(ctx, traffic, xScale, width, height, can.road_sign || {});
  (can.vehicles || []).forEach(vehicle => drawCanVehicle(ctx,vehicle,xScale,yScale)); (geometry.leads || []).forEach(lead => { if ((can.vehicles||[]).some(v => v.category==='lead')) return; const [px,py]=modelCanvasPoint(canvas,lead.x,lead.y,xScale,yScale); if(py>50&&py<height){const color=danger?'#ef4444':'#f97316';fillRoundedRect(ctx,px-8,py-8,16,16,color,6);drawObjectLabel(ctx,'SP '+Math.round(lead.x)+'m',px+11,py+4,color);} });
  const ped = can.pedestrian_detection || {};
  drawPedestrianCameraIndicators(ctx,ped,width,height); drawExperimentalPedestrianSlots(ctx,ped,xScale,yScale,width,height); drawParkingObstacle(ctx,can.parking_obstacle||{},xScale,yScale,width,height);
  ctx.fillStyle='#2563eb';roundedRect(ctx,width/2-17,height-35,34,25,9);ctx.fill();ctx.fillStyle='#93c5fd';ctx.fillRect(width/2-8,height-31,16,5);fillRoundedRect(ctx,10,10,178,62,'#020617b8',12);ctx.fillStyle='#f8fafc';ctx.font='bold 21px sans-serif';ctx.fillText(data.speed_kph.toFixed(0)+' km/h',22,36);ctx.font='13px sans-serif';const mode=data.openpilot_enabled?'SP 接管':data.mads_enabled?'MADS 横向':'未接管';ctx.fillStyle=data.openpilot_enabled?'#86efac':'#cbd5e1';ctx.fillText(mode+'  ·  设定 '+data.set_speed_kph.toFixed(0),22,58);
  const fs = can.front_safety || {}; if (fs.available && fs.target_distance_m != null) { drawHudChip(ctx,'前方 '+fs.target_distance_m.toFixed(1)+'m'+(fs.time_to_impact_s!=null?' · TTI '+fs.time_to_impact_s.toFixed(1)+'s':''),12,91,fs.imminent_collision?'#ef4444':'#34d399'); }
  const nav=can.navigation||{}; if(nav.available){const panelW=Math.min(172,width*.42);fillRoundedRect(ctx,width-panelW-10,10,panelW,50,'#020617b8',12);ctx.textAlign='right';ctx.fillStyle='#dbeafe';ctx.font='bold 14px sans-serif';const branch=nav.next_branch_distance_m!=null?nav.next_branch_distance_m+'m '+(nav.next_branch_left_off_ramp?'↖ 左出口':nav.next_branch_right_off_ramp?'右出口 ↗':'下一分支'):nav.route_active?'导航路线已激活':'导航可用';ctx.fillText(branch,width-20,31);ctx.font='12px sans-serif';const limit=nav.speed_limit_unlimited?'不限速':nav.speed_limit!=null?'限速 '+nav.speed_limit+' '+nav.speed_limit_unit.toUpperCase():ct(nav.road_class);ctx.fillText(limit,width-20,50);ctx.textAlign='left';}
  if(geometry.lane_change!=='off'){ctx.fillStyle='#fbbf24';ctx.font='bold 15px sans-serif';ctx.fillText('变道 '+(geometry.lane_change_direction==='left'?'←':geometry.lane_change_direction==='right'?'→':'进行中'),width-92,70);} drawCanvasSummary(ctx,can,width); renderOptionalCanDetails(can);
}
async function loadDrivingStatus() { if (drivingLoading) return; drivingLoading = true; const state = document.getElementById('driving-state'), alert = document.getElementById('driving-alert'); try { const r = await apiFetch('/api/driving-status', {cache:'no-store'}); const data = await r.json(); if (!r.ok) throw new Error(data.message || 'HTTP ' + r.status); lastDrivingData=data;const connected = Object.values(data.connected).every(Boolean); state.textContent = !data.onroad ? '设置模式：等待车辆启动。' : connected ? '行驶中：车辆数据正常。' : '行驶中：部分车辆数据暂未收到。'; state.className = 'notice' + ((!data.onroad || !connected) ? ' onroad' : ''); drawDrivingGeometry(data.geometry, data); drawComparisonCharts(data.comparison);renderUnknownCan(data.geometry?.oem_can); alert.hidden = !data.alert; alert.textContent = data.alert || ''; } catch (e) { state.textContent = '行驶数据读取失败：' + e; state.className = 'notice onroad'; } finally { drivingLoading = false; } }
setInterval(() => { if (currentPanel === 'driving') loadDrivingStatus(); }, 500);
async function loadTerminalStatus() { const el = document.getElementById('terminal-state'); try { const r = await apiFetch('/api/terminal/status', {cache:'no-store'}); const s = await r.json(); el.textContent = !s.terminal_enabled ? '终端未启用：请先在设置中开启。' : s.onroad ? '行驶中：请先进入设置模式。' : '终端已启用：请输入密码后运行。'; el.className = 'notice' + ((!s.terminal_enabled || s.onroad) ? ' onroad' : ''); } catch (e) { el.textContent = '终端状态读取失败：' + e; } }
const terminalPasswordInput = document.getElementById('terminal-password'); terminalPasswordInput.value = localStorage.getItem('openpilotTerminalPassword') || '';
async function runTerminal() { const password = terminalPasswordInput.value, command = document.getElementById('terminal-command').value, output = document.getElementById('terminal-output'); output.textContent = '正在运行…'; try { const r = await apiFetch('/api/terminal/exec', {method:'POST', headers:{'Content-Type':'application/json', 'X-Terminal-Password':password}, body:JSON.stringify({command})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); localStorage.setItem('openpilotTerminalPassword', password); output.textContent = `[exit ${result.exit_code}${result.timed_out ? ', timeout' : ''}${result.blocked_onroad ? ', onroad blocked' : ''}]\\n` + result.output; } catch (e) { output.textContent = '运行失败：' + e; } }
async function changeTerminalPassword() { const password = terminalPasswordInput.value, newPassword = document.getElementById('terminal-new-password').value; try { const r = await apiFetch('/api/terminal/password', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({current_password:password, new_password:newPassword})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); terminalPasswordInput.value = newPassword; localStorage.setItem('openpilotTerminalPassword', newPassword); document.getElementById('terminal-new-password').value = ''; alert('密码已修改'); } catch (e) { alert('修改失败：' + e); } }
loadTerminalStatus();
let activeTestId = null;
const phaseText = {queued:'请求已提交',waiting_vehicle_feedback:'等待车辆转向灯响应',waiting_sp_start:'等待 SP 开始变道',lane_changing:'SP 正在执行变道',cancelling:'正在关闭转向灯',confirming_cancel:'正在确认转向灯关闭'};
async function runTurn(direction) { document.querySelectorAll('#left,#right').forEach(button => button.disabled = true); const status = document.getElementById('status'); status.textContent = '正在提交…'; try { const response = await apiFetch('/api/turn/' + direction, {method:'POST'}); const result = await response.json(); if (!response.ok) throw new Error(result.message || '提交失败'); activeTestId = result.test_id; document.getElementById('cancel').style.display = 'block'; await pollStatus(); } catch (error) { finishTurnUi('请求失败：' + error); } }
async function pollStatus() { if (!activeTestId) return; try { const response = await apiFetch('/api/status/' + activeTestId, {cache:'no-store'}); const result = await response.json(); const detail = '已发送 ' + (result.action_frames_sent || 0) + ' 帧'; if (result.done) { const ok = result.result === 'PASS'; finishTurnUi((ok ? '完成：转向灯已自动关闭' : '结束：' + result.result) + '\\n' + detail); return; } document.getElementById('status').textContent = (phaseText[result.phase] || result.phase) + '\\n' + detail; setTimeout(pollStatus, 200); } catch (error) { finishTurnUi('状态读取失败：' + error); } }
async function cancelSession() { if (!activeTestId) return; document.getElementById('status').textContent = '正在请求关闭转向灯…'; try { await apiFetch('/api/cancel/' + activeTestId, {method:'POST'}); } catch (error) { finishTurnUi('取消请求失败：' + error); } }
function finishTurnUi(message) { document.getElementById('status').textContent = message; document.querySelectorAll('#left,#right').forEach(button => button.disabled = false); document.getElementById('cancel').style.display = 'none'; activeTestId = null; }
async function runSpeed(action) { const status = document.getElementById('status'); status.textContent = '正在发送速度按钮模板…'; try { const response = await apiFetch('/api/speed/' + action, {method:'POST'}); const result = await response.json(); if (!response.ok) throw new Error(result.message || '测试失败'); status.textContent = result.message; } catch (error) { status.textContent = '速度按钮测试失败：' + error; } }
</script></main></body></html>""".replace('<!-- NAVIGATION_LOG_PANEL -->', NAVIGATION_LOG_PANEL).replace('/* NAVIGATION_LOG_SCRIPT */', NAVIGATION_LOG_SCRIPT).encode()


def _gateway_host_allowed(host: str, port: int) -> bool:
  if not host or host != host.strip():
    return False
  try:
    parsed = urlparse(f"//{host}")
    if (parsed.username is not None or parsed.password is not None or parsed.path or parsed.query or
        parsed.fragment or parsed.port != port or parsed.hostname is None):
      return False
  except ValueError:
    return False
  if parsed.hostname.lower() == "localhost":
    return True
  try:
    address = ipaddress.ip_address(parsed.hostname)
  except ValueError:
    return False
  return address == _TESLA_CONSOLE_ADDRESS or any(address in network for network in _PRIVATE_NETWORKS)


def start_gateway_service() -> GatewayConfigService | None:
  service = None
  try:
    service = GatewayConfigService(DEFAULT_STATE_DIR)
    service.analyzer.start(lambda: service.active)
    return service
  except Exception:
    cloudlog.exception("CAN gateway service initialization failed; keeping device console available")
    if service is not None:
      try:
        service.analyzer.stop()
      except Exception:
        pass
    return None


class DeviceConsoleServer(ThreadingHTTPServer):
  daemon_threads = True

  def __init__(self, server_address, handler_class=None, *, gateway_service=None,
               max_workers: int = MAX_REQUEST_THREADS):
    if max_workers < 1:
      raise ValueError("max_workers must be positive")
    self.gateway_service = gateway_service
    self.request_slots = threading.BoundedSemaphore(max_workers)
    # DeviceConsoleHandler is defined immediately below. Keeping the default
    # resolution here also makes test handlers injectable without another server.
    super().__init__(server_address, handler_class or DeviceConsoleHandler)

  def process_request(self, request: socket.socket, client_address) -> None:
    if not self.request_slots.acquire(blocking=False):
      self._reject_busy(request)
      return
    try:
      super().process_request(request, client_address)
    except BaseException:
      self.request_slots.release()
      raise

  def process_request_thread(self, request: socket.socket, client_address) -> None:
    try:
      super().process_request_thread(request, client_address)
    finally:
      self.request_slots.release()

  def _reject_busy(self, request: socket.socket) -> None:
    body = json.dumps({"ok": False, "error": "服务繁忙，请稍后重试"}, ensure_ascii=False).encode()
    response = (b"HTTP/1.1 503 Service Unavailable\r\n" +
                b"Content-Type: application/json; charset=utf-8\r\n" +
                b"Cache-Control: no-store\r\n" +
                b"Connection: close\r\n" +
                f"Content-Length: {len(body)}\r\n\r\n".encode("ascii") + body)
    try:
      request.settimeout(0.1)
      request_head = b""
      while b"\r\n\r\n" not in request_head and len(request_head) < MAX_GATEWAY_BODY:
        chunk = request.recv(min(4096, MAX_GATEWAY_BODY - len(request_head)))
        if not chunk:
          return
        request_head += chunk
      request.sendall(response)
    except (ConnectionError, TimeoutError):
      pass
    finally:
      self.shutdown_request(request)

  def handle_error(self, request, client_address) -> None:
    if isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):
      return
    super().handle_error(request, client_address)


class DeviceConsoleHandler(BaseHTTPRequestHandler):
  server_version = "DeviceConsole/1.0"

  def setup(self) -> None:
    super().setup()
    self.connection.settimeout(SOCKET_TIMEOUT_S)

  def _send(self, status: HTTPStatus, content_type: str, body: bytes) -> None:
    try:
      self.send_response(status)
      self.send_header("Content-Type", content_type)
      self.send_header("Content-Length", str(len(body)))
      self.send_header("Cache-Control", "no-store")
      self.end_headers()
      self.wfile.write(body)
    except (ConnectionError, TimeoutError):
      self.close_connection = True

  def _gateway_authorized(self) -> bool:
    hosts = self.headers.get_all("Host", [])
    if len(hosts) != 1 or not _gateway_host_allowed(hosts[0], self.server.server_address[1]):
      self._json(HTTPStatus.FORBIDDEN, {"ok": False, "error": "网关请求地址不允许"})
      return False
    origins = self.headers.get_all("Origin", [])
    if len(origins) > 1 or (origins and origins[0] != f"http://{hosts[0]}"):
      self._json(HTTPStatus.FORBIDDEN, {"ok": False, "error": "网关跨站请求不允许"})
      return False
    return True

  def _gateway_service(self) -> GatewayConfigService:
    service = getattr(self.server, "gateway_service", None)
    if service is None:
      raise OSError("gateway unavailable")
    return service

  def _gateway_error(self, error: Exception) -> None:
    if isinstance(error, ConfigError):
      self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "网关配置或请求内容无效"})
    elif isinstance(error, ProtocolError):
      self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "网关状态已变化，请重新读取"})
    elif isinstance(error, OSError):
      self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "error": "网关服务暂不可用"})
    else:
      cloudlog.exception("CAN gateway request failed")
      self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": "网关服务内部错误"})

  def _gateway_get(self, path: str, query: dict[str, list[str]]) -> None:
    try:
      static = {
        "/gateway/": ("index.html", "text/html; charset=utf-8"),
        "/gateway/app.js": ("app.js", "text/javascript; charset=utf-8"),
        "/gateway/style.css": ("style.css", "text/css; charset=utf-8"),
      }
      if path in static:
        name, content_type = static[path]
        self._send(HTTPStatus.OK, content_type, (GATEWAY_STATIC / name).read_bytes())
        return
      service = self._gateway_service()
      if path == "/api/gateway/state":
        with service.lock:
          self._json(HTTPStatus.OK, service.state())
        return
      if path == "/api/gateway/analysis":
        self._json(HTTPStatus.OK, service.analysis())
        return
      if path == "/api/gateway/analysis/frame":
        if set(query) != {"bus", "id"} or len(query["bus"]) != 1 or len(query["id"]) != 1:
          raise ConfigError("invalid analysis query")
        try:
          bus = int(query["bus"][0])
          address = int(query["id"][0], 16)
        except ValueError:
          raise ConfigError("invalid analysis query") from None
        if bus not in (0, 1, 2) or not 0 <= address <= 0x7FF:
          raise ConfigError("invalid analysis query")
        self._json(HTTPStatus.OK, service.analysis_frame(bus, address))
        return
      self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "网关路径不存在"})
    except (ConnectionError, TimeoutError):
      pass
    except Exception as error:
      self._gateway_error(error)

  def _gateway_json_body(self):
    if self.headers.get("Transfer-Encoding") is not None:
      raise ConfigError("chunked requests are not accepted")
    if self.headers.get_content_type() != "application/json":
      raise ConfigError("JSON required")
    lengths = self.headers.get_all("Content-Length", [])
    if len(lengths) != 1 or not lengths[0].isdigit():
      raise ConfigError("exactly one Content-Length is required")
    size = int(lengths[0])
    if not 0 < size <= MAX_GATEWAY_BODY:
      raise ConfigError("request size is invalid")
    body = self.rfile.read(size)
    if len(body) != size:
      raise ConfigError("request body is incomplete")
    try:
      return json.loads(body)
    except (UnicodeError, json.JSONDecodeError):
      raise ConfigError("invalid JSON") from None

  def _gateway_post(self, path: str) -> None:
    try:
      service = self._gateway_service()
      data = self._gateway_json_body()
      if path == "/api/gateway/validate":
        config = validate_gateway_config(data)
        self._json(HTTPStatus.OK, {"config": config, "digest": f"{gateway_digest(config):08X}"})
        return
      if path == "/api/gateway/save":
        with service.lock:
          self._json(HTTPStatus.OK, service.save(data))
        return
      if path == "/api/gateway/analysis/reset":
        if data != {}:
          raise ConfigError("reset does not accept parameters")
        self._json(HTTPStatus.OK, service.reset_analysis())
        return
      self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "网关路径不存在"})
    except (ConnectionError, TimeoutError):
      pass
    except Exception as error:
      self._gateway_error(error)

  def _authorize_api(self) -> bool:
    try:
      if not client_is_local(self.client_address[0]):
        raise PermissionError("仅允许本机、热点或局域网客户端访问")
      return True
    except PermissionError as error:
      self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      return False

  @staticmethod
  def _range_from_query(query: dict[str, list[str]]) -> tuple[int, int]:
    try:
      start_ms = int(query.get("start_ms", [""])[0])
      end_ms = int(query.get("end_ms", [""])[0])
    except (TypeError, ValueError):
      raise ValueError("必须提供有效的开始和结束时间") from None
    return start_ms, end_ms

  def _stream_log_download(self, selection: LogSelection) -> None:
    if not _LOG_DOWNLOAD_LOCK.acquire(blocking=False):
      self._json(HTTPStatus.CONFLICT, {"ok": False, "message": "已有日志下载正在进行"})
      return
    try:
      self.send_response(HTTPStatus.OK)
      self.send_header("Content-Type", "application/zip")
      self.send_header("Content-Disposition", f'attachment; filename="{download_filename(selection)}"')
      self.send_header("Cache-Control", "no-store")
      self.send_header("X-Content-Type-Options", "nosniff")
      self.send_header("Connection", "close")
      self.end_headers()
      self.close_connection = True
      stream_log_zip(selection, cast(BinaryIO, self.wfile), diagnostics=collect_system_diagnostics())
      self.wfile.flush()
    except (BrokenPipeError, ConnectionResetError):
      pass
    finally:
      _LOG_DOWNLOAD_LOCK.release()

  def _delete_selected_logs(self, selection: LogSelection) -> None:
    if not _LOG_DOWNLOAD_LOCK.acquire(blocking=False):
      self._json(HTTPStatus.CONFLICT, {"ok": False, "message": "已有日志下载或清理正在进行"})
      return
    try:
      result = delete_log_selection(selection)
      self._json(HTTPStatus.OK, {"ok": True, **result.summary()})
    finally:
      _LOG_DOWNLOAD_LOCK.release()

  def _navigation_download(self, start_ms: int, end_ms: int) -> None:
    if not _LOG_DOWNLOAD_LOCK.acquire(blocking=False):
      self._json(HTTPStatus.CONFLICT, {'message': '已有日志下载正在进行'})
      return
    try:
      flush_recording()
      files = select_navigation_logs(start_ms, end_ms)
      if not files:
        self._json(HTTPStatus.NOT_FOUND, {'message': '所选时间没有导航日志，请刷新最近记录'})
        return
      self.send_response(HTTPStatus.OK)
      self.send_header('Content-Type', 'application/zip')
      self.send_header('Content-Disposition', f'attachment; filename="navigation-logs-{start_ms}-{end_ms}.zip"')
      self.send_header('Cache-Control', 'no-store')
      self.send_header('X-Content-Type-Options', 'nosniff')
      self.send_header('Connection', 'close')
      self.end_headers()
      self.close_connection = True
      stream_navigation_logs(files, self.wfile, start_ms=start_ms, end_ms=end_ms)
      self.wfile.flush()
    except (BrokenPipeError, ConnectionResetError):
      pass
    finally:
      _LOG_DOWNLOAD_LOCK.release()

  def do_GET(self) -> None:
    request = urlparse(self.path)
    path = request.path
    query = parse_qs(request.query, keep_blank_values=True)
    if not client_is_local(self.client_address[0]):
      self._send(HTTPStatus.FORBIDDEN, "text/plain; charset=utf-8", "仅允许本地网络访问".encode())
      return
    gateway_route = path in ("/gateway", "/api/gateway") or path.startswith(("/gateway/", "/api/gateway/"))
    if gateway_route:
      if not self._gateway_authorized():
        return
      self._gateway_get("/gateway/" if path == "/gateway" else path, query)
      return
    if path.startswith("/api/") and not self._authorize_api():
      return
    if path == '/api/navigation/logs/status':
      self._json(HTTPStatus.OK, {**navigation_log_status(), **console_status()})
      return
    if path == '/api/navigation/logs/preview':
      try:
        files = select_navigation_logs(*self._range_from_query(query))
        self._json(HTTPStatus.OK, {'file_count': len(files), 'total_bytes': sum(f.size for f in files),
                                  'navigation_only': True})
      except ValueError as error:
        self._json(HTTPStatus.BAD_REQUEST, {'message': str(error)})
      return
    if path == '/api/navigation/logs/download':
      try:
        require_offroad()
        start, end = self._range_from_query(query)
        select_navigation_logs(start, end)
        self._navigation_download(start, end)
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {'message': str(error)})
      except ValueError as error:
        self._json(HTTPStatus.BAD_REQUEST, {'message': str(error)})
      return
    if path == "/api/hotspot":
      self._json(HTTPStatus.OK, hotspot_status())
      return
    if path == "/api/logs/status":
      log_range = available_log_range()
      if not log_range["available"]:
        now_ms = time.time_ns() // 1_000_000
        log_range = {**log_range, "available": True, "start_ms": now_ms - 30 * 60 * 1000, "end_ms": now_ms}
      self._json(HTTPStatus.OK, {**log_range, **console_status(),
                                "system_diagnostics_available": True,
                                "export_excludes_rlog": True, "export_excludes_video": True})
      return
    if path == "/api/logs/preview":
      try:
        selection = select_log_range(*self._range_from_query(query))
        self._json(HTTPStatus.OK, {"ok": True, **selection.summary(),
                                  "includes_system_diagnostics": True,
                                  "max_system_diagnostic_bytes": MAX_SYSTEM_DIAGNOSTIC_BYTES})
      except ValueError as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if path == "/api/logs/download":
      try:
        require_offroad()
        selection = select_log_range(*self._range_from_query(query))
        self._stream_log_download(selection)
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except ValueError as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if path == "/api/vehicle":
      data = driving_status_snapshot()
      self._json(HTTPStatus.OK, {"vehicle": data["vehicle"], "ambient_test_available": data["ambient_test_available"],
                                "ambient_test_ready": data["ambient_test_ready"],
                                "device_ip": device_ip_address()})
      return
    if path == "/api/driving-status":
      try:
        self._json(HTTPStatus.OK, driving_status_snapshot())
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except Exception as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": f"行驶数据暂不可用：{error}"})
      return
    if path == "/api/terminal/status":
      self._json(HTTPStatus.OK, terminal_status())
      return
    if path == "/api/settings":
      self._json(HTTPStatus.OK, settings_snapshot())
      return
    if path.startswith("/api/status/"):
      test_id = path.removeprefix("/api/status/")
      with _SESSION_LOCK:
        session_expired = (_ACTIVE_WEB_TEST_ID == test_id and
                           time.monotonic() - _ACTIVE_WEB_SESSION_STARTED >= WEB_SESSION_TIMEOUT_S)
      if session_expired:
        cancel_validation_session(test_id)
        _clear_active_session(test_id)
        self._json(HTTPStatus.OK, {"test_id": test_id, "done": True, "result": "WEB_SESSION_TIMEOUT"})
        return
      status = get_validation_status(test_id)
      if status.get("done"):
        _clear_active_session(test_id)
      self._json(HTTPStatus.OK, status)
      return
    if path not in ("/", "/index.html"):
      self._send(HTTPStatus.NOT_FOUND, "text/plain; charset=utf-8", b"Not found")
      return
    self._send(HTTPStatus.OK, "text/html; charset=utf-8", render_page())

  def do_POST(self) -> None:
    if not self._authorize_api():
      return
    request = urlparse(self.path)
    path = request.path
    if path == "/api/gateway" or path.startswith("/api/gateway/"):
      if not self._gateway_authorized():
        return
      self._gateway_post(path)
      return
    if self.path == '/api/navigation/logs/flush':
      try:
        require_offroad()
        flushed = flush_recording()
        self._json(HTTPStatus.OK, {**navigation_log_status(), **console_status(), 'flushed': flushed})
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {'message': str(error)})
      return
    if self.path == "/api/tesla/ambient":
      try:
        length = int(self.headers.get("Content-Length", "0"))
        if not 0 < length <= 256:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(length))
        if not isinstance(payload, dict) or set(payload) != {"side"}:
          raise ValueError("仅接受 side；颜色固定为红色")
        self._json(HTTPStatus.OK, run_ambient_test(payload["side"]))
      except ValueError as error:
        self._json(HTTPStatus.BAD_REQUEST, {"message": str(error)})
      except RuntimeError as error:
        self._json(HTTPStatus.CONFLICT, {"message": str(error)})
      return
    if self.path == "/api/logs/delete":
      try:
        require_offroad()
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 4096:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or payload.get("confirm") is not True:
          raise ValueError("必须明确确认清理日志")
        try:
          start_ms = int(payload.get("start_ms"))
          end_ms = int(payload.get("end_ms"))
        except (TypeError, ValueError):
          raise ValueError("必须提供有效的开始和结束时间") from None
        selection = select_log_range(start_ms, end_ms, include_rlog=True)
        if not selection.files and not selection.diagnostic_files:
          raise ValueError("所选时间段没有可清理的日志")
        self._delete_selected_logs(selection)
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path == "/api/hotspot":
      try:
        require_offroad()
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 4096:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or not isinstance(payload.get("enabled"), bool):
          raise ValueError("请求必须包含 enabled 开关")
        self._json(HTTPStatus.OK, set_hotspot_enabled(payload["enabled"]))
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except RuntimeError as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path == "/api/terminal/exec":
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 8192:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or "command" not in payload:
          raise ValueError("请求必须包含 command")
        self._json(HTTPStatus.OK, run_command(payload["command"], self.headers.get("X-Terminal-Password")))
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path == "/api/terminal/password":
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 8192:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict):
          raise ValueError("请求必须为 JSON 对象")
        change_password(payload.get("current_password"), payload.get("new_password"))
        self._json(HTTPStatus.OK, {"ok": True})
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path.startswith("/api/settings/"):
      key = self.path.removeprefix("/api/settings/")
      if not key or "/" in key:
        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知设置"})
        return
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 4096:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or "value" not in payload:
          raise ValueError("请求必须包含 value")
        self._json(HTTPStatus.OK, {"ok": True, **validate_and_write(key, payload["value"])})
      except KeyError:
        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知或不允许修改的设置"})
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path.startswith("/api/cancel/"):
      test_id = self.path.removeprefix("/api/cancel/")
      cancel_validation_session(test_id)
      self._json(HTTPStatus.ACCEPTED, {"ok": True, "test_id": test_id})
      return
    if self.path.startswith("/api/speed/"):
      action_value = self.path.removeprefix("/api/speed/")
      if action_value not in (SpeedButtonAction.increase.value, SpeedButtonAction.decrease.value):
        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知速度按钮测试"})
        return
      try:
        result = run_validation(SpeedButtonAction(action_value))
      except RuntimeError as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": f"测试被阻止：{error}"})
        return
      messages = {
        0: "验证通过：设定速度已按预期改变",
        1: "验证被阻止：检查车辆状态、参数和新鲜原车模板",
        2: "验证失败：Panda 拒绝或未观察到发送回显",
        3: "模板已发送；请观察车辆设定速度显示",
      }
      self._json(HTTPStatus.OK if result in (0, 3) else HTTPStatus.CONFLICT,
                 {"ok": result in (0, 3), "result": result, "message": messages[result]})
      return
    direction = self.path.removeprefix("/api/turn/")
    if direction in ("left", "right") and self.path == f"/api/turn/{direction}":
      try:
        global _ACTIVE_WEB_SESSION_STARTED, _ACTIVE_WEB_TEST_ID
        with _SESSION_LOCK:
          if _ACTIVE_WEB_TEST_ID is not None:
            existing = get_validation_status(_ACTIVE_WEB_TEST_ID)
            session_expired = time.monotonic() - _ACTIVE_WEB_SESSION_STARTED >= WEB_SESSION_TIMEOUT_S
            if not existing.get("done") and not session_expired:
              self._json(HTTPStatus.CONFLICT, {"ok": False, "message": "已有变道请求正在运行"})
              return
            if not existing.get("done"):
              cancel_validation_session(_ACTIVE_WEB_TEST_ID)
          test_id = start_validation_session(direction)
          _ACTIVE_WEB_TEST_ID = test_id
          _ACTIVE_WEB_SESSION_STARTED = time.monotonic()
        self._json(HTTPStatus.ACCEPTED, {"ok": True, "test_id": test_id, "log": VALIDATION_LOG_PATH})
      except RuntimeError as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": f"测试被阻止：{error}"})
      return
    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知请求"})

  def _json(self, status: HTTPStatus, payload: dict) -> None:
    self._send(status, "application/json; charset=utf-8", json.dumps(payload, ensure_ascii=False).encode())

  def log_message(self, format: str, *args) -> None:  # noqa: A002 - stdlib override requires this parameter name
    pass


def main() -> None:
  start_unknown_can_observer()
  gateway_service = (start_gateway_service()
                     if get_hardware_profile() == HardwareProfile.C3XL else None)
  server = None
  try:
    server = DeviceConsoleServer((HOST, PORT), gateway_service=gateway_service)
    server.serve_forever()
  finally:
    if server is not None:
      server.server_close()
    if gateway_service is not None:
      try:
        gateway_service.analyzer.stop()
      except Exception:
        pass


if __name__ == "__main__":
  main()
