from __future__ import annotations

from dataclasses import dataclass
import math


MARKING_LABELS = {
  "solid": "实线",
  "dashed": "虚线",
  "doubleSolid": "双实线",
  "doubleDashed": "双虚线",
  "solidDashed": "实虚线",
  "roadEdge": "路缘",
  "unknown": "未知",
}

MANEUVER_LABELS = {
  "none": ("·", "等待指令"),
  "straight": ("↑", "直行"),
  "slightLeft": ("↖", "左前方"),
  "slightRight": ("↗", "右前方"),
  "turnLeft": ("←", "左转"),
  "turnRight": ("→", "右转"),
  "sharpLeft": ("↖", "急左转"),
  "sharpRight": ("↗", "急右转"),
  "uTurnLeft": ("↶", "左掉头"),
  "uTurnRight": ("↷", "右掉头"),
  "keepLeft": ("↖", "靠左行驶"),
  "keepRight": ("↗", "靠右行驶"),
  "mergeLeft": ("↖", "向左汇入"),
  "mergeRight": ("↗", "向右汇入"),
  "exitLeft": ("↖", "左侧出口"),
  "exitRight": ("↗", "右侧出口"),
  "rampLeft": ("↖", "进入左匝道"),
  "rampRight": ("↗", "进入右匝道"),
  "roundabout": ("↻", "进入环岛"),
  "destination": ("●", "到达目的地"),
  "unknown": ("·", "等待指令"),
}

REJECT_LABELS = {
  "none": "导航可用",
  "disabled": "导航未启用",
  "noData": "等待导航数据",
  "authentication": "等待手机连接",
  "malformed": "导航数据异常",
  "replay": "等待最新导航",
  "stale": "导航数据已过期",
  "routeUnmatched": "等待匹配路线",
  "gpsWeak": "GPS 信号弱",
  "localLocalization": "等待设备定位",
  "phoneLocalization": "等待手机定位",
  "guidanceStale": "导航指令已过期",
  "outsideTrackDEPRECATED": "导航等待",
}


@dataclass(frozen=True)
class LaneOverlayDisplay:
  left: str
  center: str
  right: str
  left_marking: str = "unknown"
  right_marking: str = "unknown"
  reliable: bool = False


@dataclass(frozen=True)
class NavigationOverlayDisplay:
  title: str
  subtitle: str
  detail: str
  ready: bool = False
  receiving: bool = False
  linked: bool = False
  maneuver: str = "none"
  distance: str = ""
  instruction: str = ""
  current_guidance: bool = False
  warning: bool = False


@dataclass(frozen=True)
class OverlayLayout:
  navigation: tuple[float, float, float, float]
  left_lane: tuple[float, float, float, float]
  center_lane: tuple[float, float, float, float]
  right_lane: tuple[float, float, float, float]


def overlay_layout(width: float, height: float, *, bottom_inset: float = 24) -> OverlayLayout:
  nav_width = min(920.0, max(0.0, width - 48.0))
  nav_height = 172.0
  lane_gap = 0.0
  lane_content = nav_width
  lane_widths = (lane_content * 0.36, lane_content * 0.28, lane_content * 0.36)
  lane_height = 44.0
  lane_total = sum(lane_widths) + 2 * lane_gap
  nav_y = max(0.0, height - bottom_inset - nav_height)
  lane_y = nav_y + nav_height - lane_height
  nav_x = (width - nav_width) / 2
  lane_x = (width - lane_total) / 2
  return OverlayLayout(
    navigation=(nav_x, nav_y, nav_width, nav_height),
    left_lane=(lane_x, lane_y, lane_widths[0], lane_height),
    center_lane=(lane_x + lane_widths[0] + lane_gap, lane_y, lane_widths[1], lane_height),
    right_lane=(lane_x + lane_widths[0] + lane_gap + lane_widths[1] + lane_gap, lane_y, lane_widths[2], lane_height),
  )


def lane_display_from_service(topology, *, seen: bool, alive: bool, valid: bool) -> LaneOverlayDisplay | None:
  if not seen:
    return None
  if not alive or not valid:
    return LaneOverlayDisplay("左侧  未知", "车道识别中", "右侧  未知")

  reliable = bool(topology.valid and not topology.stale and not topology.ambiguous)
  left_marking = str(topology.leftMarking) if reliable else "unknown"
  right_marking = str(topology.rightMarking) if reliable else "unknown"
  left = MARKING_LABELS.get(left_marking, "未知")
  right = MARKING_LABELS.get(right_marking, "未知")
  lane_index = int(topology.egoLaneIndexFromLeft)
  lane_count = int(topology.visibleLaneCount)
  center = f"可见车道  {lane_index + 1} / {lane_count}" if reliable and 0 <= lane_index < lane_count else "车道识别中"
  return LaneOverlayDisplay(
    left=f"左  {left}",
    center=center,
    right=f"右  {right}",
    left_marking=left_marking,
    right_marking=right_marking,
    reliable=reliable,
  )


def lane_display_from_ui_bridge(topology, marking_types) -> LaneOverlayDisplay | None:
  if topology is None or marking_types is None:
    return None
  left_type, right_type = marking_types
  left_marking = getattr(left_type, "name", "unknown")
  right_marking = getattr(right_type, "name", "unknown")
  left = MARKING_LABELS.get(left_marking, "未知")
  right = MARKING_LABELS.get(right_marking, "未知")
  lane_index = int(topology.ego_lane_index_from_left)
  lane_count = int(topology.visible_lane_count)
  reliable = bool(not topology.stale and lane_index >= 0 and lane_count > 0)
  if not reliable:
    left_marking = right_marking = "unknown"
    left = right = "未知"
  center = f"可见车道  {lane_index + 1} / {lane_count}" if reliable else "车道识别中"
  return LaneOverlayDisplay(
    left=f"左  {left}",
    center=center,
    right=f"右  {right}",
    left_marking=left_marking,
    right_marking=right_marking,
    reliable=reliable,
  )


def navigation_display_from_service(
  nav,
  *,
  seen: bool,
  alive: bool,
  valid: bool,
  lane_intent=None,
  lane_intent_healthy: bool = False,
  decel_active: bool = False,
  signal_configured: bool | None = None,
) -> NavigationOverlayDisplay | None:
  if not seen:
    return None
  if not alive or not valid:
    return NavigationOverlayDisplay("导航连接中断", "等待手机重新连接", "请保持 TesNav 运行", warning=True)

  reject_reason = str(nav.rejectReason)
  if bool(nav.stale) or reject_reason in ("stale", "guidanceStale"):
    return NavigationOverlayDisplay("导航已过期", "等待手机更新路线", "当前没有有效指引", receiving=True, warning=True)
  if not bool(nav.routeActive):
    inactive_mode_labels = {
      "realtime": ("手机导航中", "C3 暂无有效导航指引"),
      "recalculating": ("手机正在重新规划", "等待手机更新路线"),
      "simulation": ("手机模拟导航中", "模拟路线不用于车辆联动"),
      "routePlanned": ("手机路线已规划", "请在手机上开始导航"),
      "arrived": ("手机导航已到达", "可在手机上选择新目的地"),
    }
    labels = inactive_mode_labels.get(str(getattr(nav, "mode", "idle")))
    if labels is not None:
      return NavigationOverlayDisplay(*labels, "手机已连接", receiving=True)
    return NavigationOverlayDisplay("等待开始导航", "在手机上选择目的地", "手机已连接", receiving=True)
  if not bool(nav.routeMatched):
    return NavigationOverlayDisplay("正在匹配路线", "等待手机更新路线", "暂不显示转向指引", receiving=True, warning=True)
  distance_value = float(nav.maneuverDistanceM)
  if not math.isfinite(distance_value) or distance_value < 0:
    return NavigationOverlayDisplay("导航距离待更新", "等待手机更新路线", "当前没有有效指引", receiving=True, warning=True)

  maneuver_key = str(nav.maneuver)
  symbol, maneuver = MANEUVER_LABELS.get(maneuver_key, MANEUVER_LABELS["unknown"])
  distance = int(distance_value)
  distance_text = f"{distance / 1000:.1f} km" if distance >= 1000 else f"{distance} m"
  title = f"{symbol}  {distance_text}  {maneuver}"

  current_road = str(nav.currentRoad).strip()
  next_road = str(nav.nextRoad).strip()
  if current_road and next_road:
    subtitle = f"{current_road}  →  {next_road}"
  else:
    subtitle = next_road or current_road or "路线已接收"

  details: list[str] = []
  navigation_disabled = bool(lane_intent_healthy and lane_intent is not None and
                             str(getattr(lane_intent, 'reason', '')) == 'navigationDisabled')
  recommended = [str(int(lane.index) + 1) for lane in nav.lanes if lane.recommended]
  if navigation_disabled:
    details.append('导航联动已关闭')
  elif signal_configured is False:
    details.append("自动打灯未启用")
  elif lane_intent_healthy and lane_intent is not None and lane_intent.signalRequested:
    direction = "左" if str(lane_intent.direction) == "left" else "右"
    if int(lane_intent.targetLaneIndex) < 0:
      details.append(f"请求{direction}转灯")
    elif str(getattr(lane_intent, "reason", "")) == "neighborObservationHold":
      details.append("确认邻车道 · 暂停变道")
    elif getattr(lane_intent, "forkNow", False):
      details.append(f"{direction}分叉请求" + (" · 实线放行" if getattr(lane_intent, "ignoreSolidBoundary", False) else ""))
    elif getattr(lane_intent, "spLaneChangeReady", False):
      details.append(f"请求向{direction}变道")
    else:
      details.append(f"请求{direction}灯 · 等待 SP")
  elif lane_intent_healthy and lane_intent is not None:
    consistency = {
      "heuristicStabilizingNeighbor": "确认邻车道",
      "heuristicNeighborUnknown": "邻车道未知",
      "neighborUnknown": "邻车道未知",
      "neighborObservationPaused": "等待邻车道稳定",
      "heuristicStabilizingEdge": "确认已靠边",
      "heuristicEdgeConfirmed": "已靠近目标侧",
      "heuristicStabilizingNewNeighbor": "确认新增车道",
      "heuristicCooldown": "变道间隔中",
      "heuristicChangeLimit": "暂停连续变道",
      "heuristicDriverSteering": "驾驶员转向中",
      "waitingCrossing": "等待可变道路段",
      "heuristicWaitingCrossing": "等待可变道路段",
      "waitingBlindspot": "等待盲区清除",
      "heuristicWaitingBlindspot": "等待盲区清除",
      "turnApproachHandoff": "路口转向准备",
      "turnSignalsDisabled": "转弯灯请求已关闭",
    }.get(str(lane_intent.reason))
    if consistency is not None:
      details.append(consistency)
  if decel_active:
    details.append("转弯减速中")

  ready = bool(nav.valid and not navigation_disabled)
  linked = bool(
    getattr(nav, "routeActive", False)
    and getattr(nav, "routeMatched", False)
    and not getattr(nav, "stale", True)
  )
  gps_diagnostic = {
    "gpsWeak": "手机 GPS 仅提示",
    "phoneLocalization": "手机 GPS 仅提示",
    "localLocalization": "设备 GPS 仅提示",
  }.get(reject_reason)
  if ready and gps_diagnostic is not None:
    status = gps_diagnostic
  elif ready:
    status = "导航可用"
  elif linked and gps_diagnostic is not None:
    status = gps_diagnostic
  else:
    status = REJECT_LABELS.get(reject_reason, "导航等待")
  if not details:
    details.append(status)
  if recommended and len(details) < 2:
    details.append(f"推荐 {','.join(recommended)} 车道")
  return NavigationOverlayDisplay(
    title, subtitle, " · ".join(details[:2]), ready=ready, receiving=True, linked=linked,
    maneuver=maneuver_key, distance=distance_text, instruction=maneuver,
    current_guidance=maneuver_key not in ("none", "unknown"), warning=signal_configured is False,
  )
