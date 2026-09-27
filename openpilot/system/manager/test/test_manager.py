#!/usr/bin/env python3

import os
import inspect
from pathlib import Path
import unittest
import signal
import time

from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.params import Params
import openpilot.system.manager.manager as manager
import openpilot.system.manager.process as manager_process
import openpilot.system.manager.process_config as process_config
from openpilot.system.manager.process import RestartingPythonProcess, ensure_running
from openpilot.system.manager.process_config import managed_processes, procs
from openpilot.selfdrive.selfdrived.events import IGNORED_PROCESSES
from openpilot.common.hardware import HARDWARE
from openpilot.sunnypilot.hardware.profile import HardwareProfile

os.environ['FAKEUPLOAD'] = "1"

MAX_STARTUP_TIME = 3
BLACKLIST_PROCS = ['manage_athenad', 'pandad', 'pigeond']


class TestManager(OpenpilotTestCase):
  def setup_method(self):
    HARDWARE.set_power_save(False)

    # ensure clean CarParams
    params = Params()
    params.clear_all()

  def teardown_method(self):
    manager.manager_cleanup()

  def test_duplicate_procs(self):
    assert len(procs) == len(managed_processes), "Duplicate process names"

  def test_blacklisted_procs(self):
    # TODO: ensure there are blacklisted procs until we have a dedicated test
    assert len(BLACKLIST_PROCS), "No blacklisted procs to test not_run"

  def test_set_params_with_default_value(self):
    params = Params()
    params.clear_all()

    os.environ['PREPAREONLY'] = '1'
    manager.main()
    for k in params.all_keys():
      default_value = params.get_default_value(k)
      if default_value is not None:
        assert params.get(k) == default_value
    assert params.get("OpenpilotEnabledToggle")
    assert params.get("RouteCount") == 0

  def test_c3xl_startup_preserves_enabled_road_video_setting(self, monkeypatch):
    monkeypatch.setattr(manager, "get_hardware_profile", lambda: HardwareProfile.C3XL)
    params = Params()
    params.put_bool("RecordRoadVideo", True, block=True)

    manager.apply_local_recording_policy(params)

    assert params.get_bool("RecordRoadVideo")

  def test_quick_boot_never_fabricates_prebuilt_marker(self):
    source = inspect.getsource(manager.manager_init)
    assert "QuickBootToggle" not in source
    assert "prebuilt" not in source

  def test_livestream_processes_follow_livestream_param_until_ignition_clear(self):
    params = Params()
    params.put_bool("IsLiveStreaming", True, block=True)
    CP = car.CarParams.new_message()
    CP.notCar = False

    assert managed_processes["stream_encoderd"].should_run(True, params, CP)
    assert managed_processes["webrtcd"].should_run(True, params, CP)

  def test_local_process_seams_remain_registered(self):
    for name in ("device_console", "tesla_hotspotd", "alert_output", "chestnut_statusd", "trafficcontrold", "local_diagnosticsd"):
      assert name in managed_processes

  def test_device_console_is_restartable_and_noncritical(self):
    process = managed_processes["device_console"]
    assert isinstance(process, RestartingPythonProcess)
    assert IGNORED_PROCESSES == frozenset({"mapd", process.name})
    assert "gateway_configd" not in managed_processes

  def test_restarting_python_process_reaps_dead_child_after_backoff(self, monkeypatch):
    class FakeProcess:
      next_pid = 100

      def __init__(self, *args, **kwargs):
        self.pid = self.next_pid
        FakeProcess.next_pid += 1
        self.exitcode = None

      def start(self):
        pass

      def is_alive(self):
        return self.exitcode is None

    now = [0.0]
    monkeypatch.setattr(manager_process, "Process", FakeProcess)
    monkeypatch.setattr(manager_process.time, "monotonic", lambda: now[0])
    process = RestartingPythonProcess("optional", "fake.module", lambda *_: True, restart_delay=5.0)

    process.start()
    first_pid = process.proc.pid
    process.proc.exitcode = 1

    now[0] = 4.9
    process.start()
    assert process.proc.pid == first_pid

    now[0] = 5.0
    process.start()
    assert process.proc.pid != first_pid
    assert process.proc.exitcode is None

  def test_tesla_hotspot_process_is_c3xl_profile_scoped(self, monkeypatch):
    params = Params()
    CP = car.CarParams.new_message()
    predicate = managed_processes["tesla_hotspotd"].should_run
    monkeypatch.setattr(process_config, "PC", False)

    monkeypatch.setattr(process_config, "get_hardware_profile", lambda: process_config.HardwareProfile.C3XL)
    assert predicate(True, params, CP)
    assert predicate(False, params, CP)

    monkeypatch.setattr(process_config, "get_hardware_profile", lambda: process_config.HardwareProfile.STANDARD)
    assert not predicate(True, params, CP)

  def test_github_runner_process_is_retired(self):
    assert "github_runner_start" not in managed_processes
    repo_root = Path(process_config.__file__).parents[3]
    assert not (Path(process_config.__file__).parent / "github_runner.sh").exists()
    assert not (repo_root / "release/ci/install_github_runner.sh").exists()
    assert not (repo_root / "release/ci/uninstall_github_runner.sh").exists()

  @unittest.skip("this test is flaky the way it's currently written, should be moved to test_onroad")
  def test_clean_exit(self, subtests):
    """
      Ensure all processes exit cleanly when stopped.
    """
    HARDWARE.set_power_save(False)
    manager.manager_init()

    CP = car.CarParams.new_message()
    procs = ensure_running(managed_processes.values(), True, Params(), CP, not_run=BLACKLIST_PROCS)

    time.sleep(10)

    for p in procs:
      with subtests.test(proc=p.name):
        state = p.get_process_state_msg()
        assert state.running, f"{p.name} not running"
        exit_code = p.stop(retry=False)

        assert p.name not in BLACKLIST_PROCS, f"{p.name} was started"

        assert exit_code is not None, f"{p.name} failed to exit"

        # TODO: interrupted blocking read exits with 1 in cereal. use a more unique return code
        exit_codes = [0, 1]
        if p.sigkill:
          exit_codes = [-signal.SIGKILL]
        assert exit_code in exit_codes, f"{p.name} died with {exit_code}"


if __name__ == "__main__":
  unittest.main()
