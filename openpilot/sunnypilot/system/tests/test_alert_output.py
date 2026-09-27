import os
from types import SimpleNamespace
from unittest.mock import Mock, call

import pytest

import openpilot.system.manager.process_config as process_config
from openpilot.sunnypilot.system.alert_output import BEEP_GAP_SECONDS, BEEP_PULSE_SECONDS, Beepd
from openpilot.sunnypilot.hardware.profile import HardwareProfile


@pytest.fixture
def beepd():
  beep = Beepd.__new__(Beepd)
  beep.mads_enabled = None
  beep.lane_change_initialized = False
  beep.lane_change_beep_request_id = 0
  beep.dispatch_beep = Mock()
  return beep


def test_mads_initial_state_is_silent(beepd):
  beepd.update_mads(True)

  beepd.dispatch_beep.assert_not_called()


def test_mads_enable_and_disable_have_distinct_beeps(beepd):
  beepd.update_mads(False)
  beepd.update_mads(True)
  beepd.update_mads(True)
  beepd.update_mads(False)

  assert beepd.dispatch_beep.call_args_list == [call(beepd.engage), call(beepd.disengage)]


def test_lane_change_buzzer_fires_once_when_each_automatic_change_starts(beepd):
  def intent(request_id, ready=False, valid=True):
    return SimpleNamespace(valid=valid, spLaneChangeReady=ready, requestId=request_id)

  beepd.update_lane_change(intent(4), enabled=True)
  beepd.update_lane_change(intent(4, ready=True), enabled=True)
  beepd.update_lane_change(intent(4, ready=True), enabled=True)
  beepd.update_lane_change(intent(4), enabled=True)
  beepd.update_lane_change(intent(5, ready=True), enabled=True)

  assert beepd.dispatch_beep.call_args_list == [call(beepd.engage), call(beepd.engage)]


def test_lane_change_buzzer_is_silent_on_initial_active_or_disabled_state(beepd):
  active = SimpleNamespace(valid=True, spLaneChangeReady=True, requestId=9)
  beepd.update_lane_change(active, enabled=True)
  disabled = SimpleNamespace(valid=True, spLaneChangeReady=True, requestId=10)
  beepd.update_lane_change(disabled, enabled=False)
  beepd.update_lane_change(disabled, enabled=True)
  beepd.update_lane_change(SimpleNamespace(valid=False, spLaneChangeReady=True, requestId=11), enabled=True)

  beepd.dispatch_beep.assert_not_called()


def test_warning_and_prompt_repeat_follow_legacy_beep_rules(beepd, monkeypatch):
  from opendbc.car.structs import car

  alert = car.CarControl.HUDControl.AudibleAlert
  beepd.current_alert = alert.none
  beepd.prompt_suppress_until = 0
  timestamps = iter((100.0, 100.5, 101.0, 101.5, 102.0, 102.5, 111.0, 111.5))
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.time.monotonic", lambda: next(timestamps))

  beepd.update_alert(alert.warningSoft)
  beepd.update_alert(alert.none)
  beepd.update_alert(alert.promptRepeat)
  beepd.update_alert(alert.none)
  beepd.update_alert(alert.promptRepeat)
  beepd.update_alert(alert.none)
  beepd.update_alert(alert.promptRepeat)
  beepd.update_alert(alert.none)

  assert beepd.dispatch_beep.call_args_list == [
    call(beepd.warning),
    call(beepd.engage),
    call(beepd.engage),
  ]


def test_mads_beeps_use_very_short_pulses(monkeypatch):
  assert BEEP_PULSE_SECONDS == pytest.approx(0.010)
  beep = Beepd.__new__(Beepd)
  beep._beep = Mock()
  sleep = Mock()
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.time.sleep", sleep)

  beep.engage()
  assert beep._beep.call_args_list == [call(True), call(False)]
  sleep.assert_called_once_with(BEEP_PULSE_SECONDS)

  beep._beep.reset_mock()
  sleep.reset_mock()
  beep.disengage()
  assert beep._beep.call_args_list == [call(True), call(False), call(True), call(False)]
  assert sleep.call_args_list == [call(BEEP_PULSE_SECONDS), call(BEEP_GAP_SECONDS), call(BEEP_PULSE_SECONDS)]

  beep._beep.reset_mock()
  sleep.reset_mock()
  beep.warning()
  assert beep._beep.call_args_list == [call(True), call(False)] * 3
  assert sleep.call_args_list == [
    call(BEEP_PULSE_SECONDS), call(BEEP_GAP_SECONDS),
    call(BEEP_PULSE_SECONDS), call(BEEP_GAP_SECONDS),
    call(BEEP_PULSE_SECONDS),
  ]


def test_gpio_edges_use_persistent_fd_without_subprocess(monkeypatch):
  beep = Beepd.__new__(Beepd)
  beep.gpio_fd = 42
  run = Mock()
  seek = Mock()
  write = Mock()
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.subprocess.run", run)
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.os.lseek", seek)
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.os.write", write)

  beep._beep(True)
  beep._beep(False)

  run.assert_not_called()
  assert seek.call_args_list == [call(42, 0, os.SEEK_SET), call(42, 0, os.SEEK_SET)]
  assert write.call_args_list == [call(42, b"1"), call(42, b"0")]


def test_standard_profile_never_probes_gpio42(monkeypatch):
  run = Mock()
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.get_hardware_profile",
                      lambda: HardwareProfile.STANDARD)
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.subprocess.run", run)
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.threading.Thread.start", lambda _: None)

  beep = Beepd()

  assert beep.gpio_fd is None
  run.assert_not_called()


def test_c3xl_buzzer_process_is_always_on_without_enable_param(monkeypatch):
  params = Mock()
  monkeypatch.setattr(process_config, "PC", False)
  monkeypatch.setattr(process_config, "get_hardware_profile", lambda: HardwareProfile.C3XL, raising=False)

  assert process_config.use_external_buzzer(False, params, Mock())
  assert process_config.use_external_buzzer(True, params, Mock())
  params.get_bool.assert_not_called()

  monkeypatch.setattr(process_config, "get_hardware_profile", lambda: HardwareProfile.STANDARD)
  assert not process_config.use_external_buzzer(False, params, Mock())
