import unittest
from types import SimpleNamespace

from openpilot.selfdrive.ui.sunnypilot.onroad.lane_navigation_state import navigation_display_from_service


class TestInactiveNavigationDisplay(unittest.TestCase):
  def nav(self, **updates):
    values = dict(mode="realtime", routeActive=False, stale=False, rejectReason="noData",
                  valid=False, maneuver="turnLeft", maneuverDistanceM=80, nextRoad="旧路口")
    values.update(updates)
    return SimpleNamespace(**values)

  def test_phone_modes_do_not_claim_control_or_retain_previous_guidance(self):
    for mode in ("realtime", "recalculating", "simulation", "routePlanned", "arrived"):
      with self.subTest(mode=mode):
        display = navigation_display_from_service(self.nav(mode=mode), seen=True, alive=True, valid=True)
        self.assertNotEqual(display.title, "等待开始导航")
        self.assertTrue(display.receiving)
        self.assertFalse(display.ready or display.linked or display.current_guidance)
        self.assertEqual(display.maneuver, "none")
        self.assertEqual(display.distance, "")
        self.assertNotIn("旧路口", display.subtitle)

  def test_connection_and_expiry_override_phone_mode(self):
    for mode in ("realtime", "recalculating", "simulation", "routePlanned", "arrived"):
      for updates, alive, valid, expected in (
        ({}, False, True, "导航连接中断"),
        ({}, True, False, "导航连接中断"),
        ({"stale": True}, True, True, "导航已过期"),
        ({"rejectReason": "guidanceStale"}, True, True, "导航已过期"),
      ):
        with self.subTest(mode=mode, updates=updates, alive=alive, valid=valid):
          display = navigation_display_from_service(self.nav(mode=mode, **updates), seen=True, alive=alive, valid=valid)
          self.assertEqual(display.title, expected)
          self.assertFalse(display.ready or display.linked or display.current_guidance)

  def test_unknown_or_missing_mode_keeps_idle_fallback(self):
    for mode in ("idle", "unknown", None):
      nav = self.nav(mode=mode)
      if mode is None:
        del nav.mode
      display = navigation_display_from_service(nav, seen=True, alive=True, valid=True)
      self.assertEqual(display.title, "等待开始导航")
    self.assertIsNone(navigation_display_from_service(self.nav(), seen=False, alive=True, valid=True))
