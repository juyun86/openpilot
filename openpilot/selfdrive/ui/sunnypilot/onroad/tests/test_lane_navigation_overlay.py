from types import SimpleNamespace
from openpilot.cereal import custom

from openpilot.selfdrive.ui.sunnypilot.onroad.lane_navigation_state import (
  lane_display_from_service,
  lane_display_from_ui_bridge,
  navigation_display_from_service,
  overlay_layout,
)
from openpilot.sunnypilot.lane_topology.types import LaneMarkingType


def test_disabled_navigation_still_displays_route_without_claiming_ready():
  nav = custom.NavAssistStateSP.new_message(valid=True, routeActive=True, routeMatched=True, stale=False,
                                            maneuver='turnLeft', maneuverDistanceM=80, currentRoad='主路', nextRoad='支路')
  intent = custom.NavLaneIntentSP.new_message(valid=True, signalRequested=False, reason='navigationDisabled')
  display = navigation_display_from_service(nav, seen=True, alive=True, valid=True, lane_intent=intent, lane_intent_healthy=True)
  assert display.linked and display.receiving and not display.ready
  assert display.detail == '导航联动已关闭'
  assert '80 m' in display.title


def test_lane_overlay_shows_markings_and_lane_position_when_control_is_observation_only():
  topology = SimpleNamespace(
    leftMarking="dashed",
    rightMarking="solid",
    egoLaneIndexFromLeft=1,
    visibleLaneCount=3,
    valid=True,
    stale=False,
    ambiguous=False,
    validForControl=False,
  )

  display = lane_display_from_service(topology, seen=True, alive=True, valid=True)

  assert display is not None
  assert display.left == "左  虚线"
  assert display.center == "可见车道  2 / 3"
  assert display.right == "右  实线"
  assert display.reliable


def test_lane_overlay_reports_uncertainty_instead_of_disappearing():
  topology = SimpleNamespace(
    leftMarking="unknown",
    rightMarking="solid",
    egoLaneIndexFromLeft=-1,
    visibleLaneCount=0,
    valid=False,
    stale=True,
    ambiguous=True,
    validForControl=False,
  )

  display = lane_display_from_service(topology, seen=True, alive=True, valid=True)

  assert display is not None
  assert display.left == "左  未知"
  assert display.right == "右  未知"
  assert display.center == "车道识别中"
  assert not display.reliable


def test_lane_overlay_uses_display_only_ui_bridge_when_track_service_is_absent():
  topology = SimpleNamespace(ego_lane_index_from_left=0, visible_lane_count=2, stale=False)

  display = lane_display_from_ui_bridge(topology, (LaneMarkingType.solid, LaneMarkingType.dashed))

  assert display is not None
  assert display.left == "左  实线"
  assert display.center == "可见车道  1 / 2"
  assert display.right == "右  虚线"
  assert display.reliable


def test_navigation_overlay_treats_phone_gps_as_diagnostic_after_route_is_linked():
  nav = SimpleNamespace(
    maneuver="turnLeft",
    maneuverDistanceM=184,
    currentRoad="测试路",
    nextRoad="场地西路",
    lanes=[SimpleNamespace(index=1, recommended=True)],
    routeActive=True,
    routeMatched=True,
    stale=False,
    valid=False,
    rejectReason="phoneLocalization",
  )

  display = navigation_display_from_service(nav, seen=True, alive=True, valid=True)

  assert display is not None
  assert display.title == "←  184 m  左转"
  assert display.subtitle == "测试路  →  场地西路"
  assert display.detail == "手机 GPS 仅提示 · 推荐 2 车道"
  assert display.receiving
  assert display.linked
  assert not display.ready


def test_navigation_overlay_treats_device_gps_as_diagnostic_after_route_is_linked():
  nav = SimpleNamespace(
    maneuver="turnRight",
    maneuverDistanceM=80,
    currentRoad="测试路",
    nextRoad="场地东路",
    lanes=[],
    routeActive=True,
    routeMatched=True,
    stale=False,
    valid=False,
    rejectReason="localLocalization",
  )

  display = navigation_display_from_service(nav, seen=True, alive=True, valid=True)

  assert display is not None
  assert display.detail == "设备 GPS 仅提示"
  assert display.linked
  assert not display.ready


def test_navigation_overlay_keeps_gps_diagnostic_visible_when_control_is_valid():
  nav = SimpleNamespace(
    maneuver="turnRight",
    maneuverDistanceM=80,
    currentRoad="测试路",
    nextRoad="场地东路",
    lanes=[],
    routeActive=True,
    routeMatched=True,
    stale=False,
    valid=True,
    rejectReason="localLocalization",
  )

  display = navigation_display_from_service(nav, seen=True, alive=True, valid=True)

  assert display is not None
  assert display.detail == "设备 GPS 仅提示"
  assert display.ready


def test_navigation_overlay_labels_pre_turn_lamp_without_calling_it_a_lane_change():
  nav = SimpleNamespace(
    maneuver="turnLeft",
    maneuverDistanceM=80,
    currentRoad="测试路",
    nextRoad="场地西路",
    lanes=[],
    routeActive=True,
    routeMatched=True,
    stale=False,
    valid=True,
    rejectReason="none",
  )
  turn_intent = SimpleNamespace(
    signalRequested=True,
    direction="left",
    targetLaneIndex=-1,
    laneChangeAuthorized=False,
  )

  display = navigation_display_from_service(
    nav,
    seen=True,
    alive=True,
    valid=True,
    lane_intent=turn_intent,
    lane_intent_healthy=True,
  )

  assert display is not None
  assert display.detail == "请求左转灯"
  assert "开启" not in display.detail


def test_navigation_overlay_makes_fork_now_bypass_visible():
  nav = SimpleNamespace(
    maneuver="exitRight", maneuverDistanceM=40, currentRoad="主路", nextRoad="出口",
    lanes=[], routeActive=True, routeMatched=True, stale=False, valid=True, rejectReason="none",
  )
  lane_intent = SimpleNamespace(
    signalRequested=True, direction="right", targetLaneIndex=1,
    forkNow=True, spLaneChangeReady=False, ignoreSolidBoundary=True,
  )

  display = navigation_display_from_service(
    nav, seen=True, alive=True, valid=True,
    lane_intent=lane_intent, lane_intent_healthy=True,
  )

  assert display is not None
  assert display.detail == "右分叉请求 · 实线放行"
  lane_intent.ignoreSolidBoundary = False
  display = navigation_display_from_service(
    nav, seen=True, alive=True, valid=True,
    lane_intent=lane_intent, lane_intent_healthy=True,
  )
  assert display.detail == "右分叉请求"



def test_tici_overlay_layout_is_bounded_and_embeds_lane_footer_in_navigation_card():
  layout = overlay_layout(2160, 1080)
  nav_x, nav_y, nav_width, nav_height = layout.navigation
  left_x, lane_y, left_width, lane_height = layout.left_lane
  center_x, center_y, center_width, center_height = layout.center_lane
  right_x, right_y, right_width, right_height = layout.right_lane

  assert 0 <= nav_x and nav_x + nav_width <= 2160
  assert 0 <= left_x and right_x + right_width <= 2160
  assert nav_y < lane_y < nav_y + nav_height
  assert lane_y == center_y == right_y
  assert lane_height == center_height == right_height
  assert left_x + left_width == center_x
  assert center_x + center_width == right_x
  assert lane_y + lane_height == nav_y + nav_height


def test_stale_and_unmatched_guidance_never_retains_an_old_arrow_or_distance():
  for updates in ({"stale": True}, {"rejectReason": "guidanceStale"}, {"routeMatched": False}, {"routeActive": False}):
    values = {"maneuver": "turnLeft", "maneuverDistanceM": 80, "currentRoad": "旧道路", "nextRoad": "旧路口",
              "lanes": [], "routeActive": True, "routeMatched": True, "stale": False, "valid": True, "rejectReason": "none"}
    values.update(updates)
    display = navigation_display_from_service(SimpleNamespace(**values), seen=True, alive=True, valid=True)
    assert not display.current_guidance and not display.distance
    assert display.maneuver == "none"
    assert "80" not in display.title and "旧" not in display.subtitle


def test_disabled_signal_configuration_is_visible_without_claiming_a_physical_lamp():
  nav = SimpleNamespace(maneuver="turnLeft", maneuverDistanceM=80, currentRoad="测试路", nextRoad="场地西路",
                        lanes=[], routeActive=True, routeMatched=True, stale=False, valid=True, rejectReason="none")
  intent = SimpleNamespace(signalRequested=True, direction="left", targetLaneIndex=-1)
  display = navigation_display_from_service(nav, seen=True, alive=True, valid=True, lane_intent=intent,
                                             lane_intent_healthy=True, signal_configured=False)
  assert display.current_guidance and display.warning
  assert display.detail == "自动打灯未启用"


def test_sidebar_and_narrow_layouts_keep_every_card_inside_the_available_hud():
  for width, height, inset in ((2160, 1080, 84), (1860, 1080, 24), (960, 540, 24), (640, 480, 84)):
    layout = overlay_layout(width, height, bottom_inset=inset)
    for x, y, box_width, box_height in (layout.navigation, layout.left_lane, layout.center_lane, layout.right_lane):
      assert 0 <= x < x + box_width <= width
      assert 0 <= y < y + box_height <= height - inset
