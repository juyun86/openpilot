from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[5]


def _read(relative_path: str) -> str:
  return (REPO_ROOT / relative_path).read_text(encoding="utf-8")


def test_custom_event_keeps_reserved_slot_and_logged_service():
  custom_schema = _read("openpilot/cereal/custom.capnp")
  event_schema = _read("openpilot/cereal/log.capnp")
  services = _read("openpilot/cereal/services.py")

  assert "struct RadarLaneStateSP @0xc2243c65e0340384" in custom_schema
  assert "targets @13 :List(Target)" in custom_schema
  assert "cutInCandidate @15 :Target" in custom_schema
  assert "radarLaneStateSP @137 :Custom.RadarLaneStateSP" in event_schema
  assert '"radarLaneStateSP": (True, 20., 5)' in services


def test_read_only_daemon_is_managed_but_never_control_critical():
  process_config = _read("openpilot/system/manager/process_config.py")
  selfdrived = _read("openpilot/selfdrive/selfdrived/selfdrived.py")
  daemon = _read("openpilot/sunnypilot/selfdrive/radar_lane/radarlanesd.py")

  assert "return started and not CP.notCar and not CP.radarUnavailable" in process_config
  assert 'PythonProcess("radarlanesd", "openpilot.sunnypilot.selfdrive.radar_lane.radarlanesd", radar_lane_available)' in process_config
  assert "'radarlanesd'" in selfdrived
  assert "config_realtime_process" not in daemon


def test_control_and_existing_radar_paths_do_not_consume_lane_occupancy():
  prohibited_consumers = (
    "openpilot/selfdrive/controls/radard.py",
    "openpilot/selfdrive/controls/plannerd.py",
    "openpilot/selfdrive/controls/lib/longitudinal_planner.py",
    "openpilot/selfdrive/controls/lib/desire_helper.py",
    "openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py",
  )

  for relative_path in prohibited_consumers:
    assert "radarLaneStateSP" not in _read(relative_path), relative_path


def test_lane_occupancy_has_a_ui_consumer_without_becoming_control_input():
  ui_state = _read("openpilot/selfdrive/ui/sunnypilot/ui_state.py")
  renderer = _read("openpilot/selfdrive/ui/onroad/model_renderer.py")

  assert '"radarLaneStateSP"' in ui_state
  assert "select_lane_display_targets(radar_lane_state.targets)" in renderer
  assert "format_target_label" in renderer


if __name__ == "__main__":
  tests = [value for name, value in globals().items() if name.startswith("test_") and callable(value)]
  for test in tests:
    test()
  print(f"{len(tests)} radar lane safety-boundary tests passed")
