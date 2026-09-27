from dataclasses import replace

import pytest

from openpilot.sunnypilot.navassist.settings import NavAssistSettings, SettingsCache, SettingsStore, parse_settings
from openpilot.sunnypilot.navassist.tests.test_speed_controller import FakeSM, nav, update
from openpilot.sunnypilot.navassist.speed_controller import NavigationSpeedController
from openpilot.sunnypilot.navassist.lane_intent import NavTurnPlan, NavTurnSignalCoordinator
from openpilot.sunnypilot.navassist.nav_lane_intentd import build_lane_plan
from types import SimpleNamespace
from openpilot.sunnypilot.navassist.tests.test_lane_intent import topology, vehicle
from openpilot.sunnypilot.navassist.lane_intent import NavLanePlan, NavLaneIntentCoordinator, LaneIntentDirection, ObservedLaneChangeState


def test_settings_persist_and_keep_unmodified_defaults(tmp_path):
  store = SettingsStore(tmp_path / 'settings.json')
  assert store.read() == NavAssistSettings()
  store.update({'turn_lane_lookahead_m': 300, 'turn_speed_kph': 20})
  assert SettingsStore(store.path).read() == replace(NavAssistSettings(), turn_lane_lookahead_m=300, turn_speed_kph=20)


def test_lane_change_buzzer_switch_is_enabled_by_default_and_persists(tmp_path):
  store = SettingsStore(tmp_path / 'settings.json')
  assert store.read().lane_change_buzzer_enabled
  assert not store.update({'lane_change_buzzer_enabled': False}).lane_change_buzzer_enabled
  assert not SettingsStore(store.path).read().lane_change_buzzer_enabled


@pytest.mark.parametrize('changes', [{'enabled': 1}, {'turn_speed_kph': 100}, {'signal_lead_time_s': 2},
                                    {'turn_lane_lookahead_m': 333}, {'unexpected': True}])
def test_invalid_values_are_rejected(changes):
  with pytest.raises(ValueError):
    parse_settings(changes)


def test_corrupt_update_keeps_last_valid_runtime_configuration(tmp_path):
  store = SettingsStore(tmp_path / 'settings.json')
  store.update({'enabled': False})
  cache = SettingsCache(store)
  assert not cache.read().enabled
  store.path.write_text('{broken')
  cache.next_read = 0
  assert not cache.read().enabled and cache.error


@pytest.mark.parametrize('field', ['enabled', 'turn_slowdown_enabled'])
def test_speed_switch_returns_original_cruise_target(field):
  settings = replace(NavAssistSettings(), **{field: False})
  controller = NavigationSpeedController(settings_provider=lambda: settings)
  update(controller, FakeSM(nav(distance=120)))
  update(controller, FakeSM(nav(distance=60)))
  assert not controller.is_active and not controller.event_admitted


def test_selected_turn_speed_is_used_by_real_cereal_message():
  settings = replace(NavAssistSettings(), turn_speed_kph=24)
  controller = NavigationSpeedController(settings_provider=lambda: settings)
  update(controller, FakeSM(nav(distance=120)))
  update(controller, FakeSM(nav(distance=50)))
  assert controller.is_active
  assert controller.output_v_target == pytest.approx(24 / 3.6)


def test_lane_distance_setting_applies_to_heuristic_and_lane_info():
  settings = replace(NavAssistSettings(), turn_lane_lookahead_m=300)
  n = SimpleNamespace(valid=True, stale=False, lanes=[], maneuver='turnLeft', maneuverDistanceM=600,
                      sessionId='x', routeRevision=1, maneuverEventId=1)
  topology = SimpleNamespace(visibleLaneCount=3)
  assert not build_lane_plan(n, topology, healthy=True, settings=settings).recommended_indices
  n.lanes = [SimpleNamespace(index=0, recommended=True)]
  assert not build_lane_plan(n, topology, healthy=True, settings=settings).valid
  n.maneuverDistanceM = 200
  assert build_lane_plan(n, topology, healthy=True, settings=settings).valid
  assert not build_lane_plan(n, topology, healthy=True, settings=replace(settings, lane_change_enabled=False)).valid


def test_route_avoid_lanes_infer_the_remaining_turn_lane_when_counts_match():
  n = SimpleNamespace(
    valid=True, stale=False, maneuver='turnLeft', maneuverDistanceM=150,
    sessionId='x', routeRevision=1, maneuverEventId=1,
    lanes=[
      SimpleNamespace(index=0, recommended=False, routeAvoid=False),
      SimpleNamespace(index=1, recommended=False, routeAvoid=True),
      SimpleNamespace(index=2, recommended=False, routeAvoid=True),
    ],
  )
  topo = SimpleNamespace(visibleLaneCount=3)

  plan = build_lane_plan(n, topo, healthy=True)

  assert plan.valid and plan.recommended_indices == (0,)
  assert plan.edge_direction == LaneIntentDirection.left


def test_signal_time_setting_changes_the_request_window():
  plan = NavTurnPlan(True, 'x', 1, 1, 'turnLeft', 100)
  assert not NavTurnSignalCoordinator().update(plan, speed_mps=10, now_ns=1, lookahead_time_s=3).signal_requested
  assert NavTurnSignalCoordinator().update(plan, speed_mps=10, now_ns=1, lookahead_time_s=10).signal_requested


def test_configured_change_count_stops_next_request_after_completed_cycle():
  coordinator = NavLaneIntentCoordinator(max_changes=1)
  plan = NavLanePlan(True, 'session-a', 1, 7, 3, (0,), heuristic=True)
  topo = topology(ego=1, left_cross=True)
  for now in (0, 500_000_000, 1_000_000_000):
    result = coordinator.update(plan, topo, vehicle(), now_ns=now)
  assert result.signal_requested
  for now in range(1_050_000_000, 2_650_000_001, 50_000_000):
    state = (ObservedLaneChangeState.starting if now < 1_500_000_000 else
             ObservedLaneChangeState.finishing if now < 2_000_000_000 else ObservedLaneChangeState.pre)
    sample = replace(vehicle(left_blinker=True, state=state, direction=LaneIntentDirection.left),
                     model_mono_time_ns=now)
    completed = coordinator.update(plan, topo, sample, now_ns=now)
  assert not completed.signal_requested
  coordinator.update(plan, topo, vehicle(), now_ns=4_000_000_000)
  result = coordinator.update(plan, topo, vehicle(), now_ns=7_000_000_000)
  assert not result.signal_requested and result.reason == 'heuristicChangeLimit'


@pytest.mark.parametrize('mode', [1, 2, 3])
def test_overtake_presets_persist_all_values_and_keep_navigation_settings(tmp_path, mode):
  from openpilot.sunnypilot.navassist.settings import OVERTAKE_PRESETS
  store = SettingsStore(tmp_path / 'settings.json')
  store.update({'signal_lead_time_s': 5, 'enabled': False})
  configured = store.update({'overtake_mode': mode})
  assert configured.overtake_mode == mode
  assert not configured.enabled and configured.signal_lead_time_s == 5
  for field, value in OVERTAKE_PRESETS[mode].items():
    assert getattr(configured, field) == value
  assert store.read() == configured
  custom = store.update({'overtake_closing_kph': 11})
  assert custom.overtake_mode == 4 and custom.overtake_closing_kph == 11
  assert custom.overtake_max_distance_m == configured.overtake_max_distance_m


@pytest.mark.parametrize('changes', [dict(overtake_mode=0), dict(overtake_mode=5),
  dict(overtake_stable_ms=450), dict(overtake_allow_right=1),
  dict(overtake_min_distance_m=40, overtake_max_distance_m=40)])
def test_invalid_overtake_settings_do_not_overwrite_file(tmp_path, changes):
  store = SettingsStore(tmp_path / 'settings.json')
  before = store.update({'overtake_mode': 2})
  with pytest.raises(ValueError): store.update(changes)
  assert store.read() == before


def test_overtake_defaults_match_normal_preset():
  from openpilot.sunnypilot.navassist.settings import OVERTAKE_PRESETS
  assert all(getattr(NavAssistSettings(), k) == v for k, v in OVERTAKE_PRESETS[2].items())


def test_legacy_gain_floor_is_ignored_when_reading_saved_settings(tmp_path):
  import json
  path = tmp_path / 'settings.json'
  path.write_text(json.dumps({'overtake_gain_kph': 10, 'overtake_mode': 2}))
  store = SettingsStore(path)
  assert store.read().overtake_mode == 2
  store.update({'overtake_stable_ms': 900})
  assert 'overtake_gain_kph' not in json.loads(path.read_text())
