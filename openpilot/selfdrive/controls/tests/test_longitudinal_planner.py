from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  A_CRUISE_MAX_BP, J_CRUISE_VALS, get_cruise_accel, LongitudinalPlanner, LongitudinalPlannerSP,
  FirstOrderFilter, LongCtrlState, T_IDXS_MPC, LongitudinalPlanSource,
)


def test_e2e_cruise_accel_respects_jerk_limit():
  v_ego = 15.0
  a_cruise_prev = -1.0
  dt = 0.05

  accel = get_cruise_accel(
    True,
    v_cruise=40.0,
    v_ego=v_ego,
    a_cruise_prev=a_cruise_prev,
    angle_steers=0.0,
    CP=SimpleNamespace(steerRatio=1.0, wheelbase=1.0),
    dt=dt,
    accel_coast=0.0,
    allow_throttle=True,
  )

  jerk_limit = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  assert accel == pytest.approx(a_cruise_prev + jerk_limit * dt)


def curve_accel(*, e2e=True, brand='tesla', angle=0., curvature=None, v_ego=10., previous=0., cruise=30., applied=None):
  return get_cruise_accel(
    e2e, v_cruise=cruise, v_ego=v_ego, a_cruise_prev=previous, angle_steers=angle,
    CP=SimpleNamespace(brand=brand, steerRatio=12., wheelbase=2.89), dt=.05,
    accel_coast=0., allow_throttle=True, model_curvature=curvature, a_target_prev=applied,
  )


@pytest.mark.parametrize('sign', [-1, 1])
def test_tesla_e2e_curve_over_budget_requests_smooth_deceleration(sign):
  # 36 km/h with this curve exceeds the existing 1.7 m/s^2 envelope.
  accel = curve_accel(curvature=sign * .025)
  assert -.061 < accel < 0.


@pytest.mark.parametrize('sign', [-1, 1])
def test_actual_turn_holds_constraint_when_model_predicts_straight(sign):
  assert curve_accel(angle=sign * 60., curvature=0.) < 0.


def test_model_curve_can_reduce_acceleration_before_steering_response():
  assert curve_accel(angle=0., curvature=.025) < 0.
  assert curve_accel(angle=0., curvature=0.) > 0.


def test_curve_does_not_override_lower_cruise_or_stronger_braking():
  assert curve_accel(curvature=.025, previous=-1.2, cruise=0.) == pytest.approx(-1.2)


def test_unselected_cruise_candidate_does_not_delay_curve_response():
  assert curve_accel(curvature=.025, previous=3.5, applied=0.) == pytest.approx(-.06)
  assert curve_accel(curvature=0., previous=3.5, applied=0.) == curve_accel(curvature=0., previous=3.5)


def test_curve_entry_and_exit_keep_existing_jerk_bound():
  previous = 1.6
  for curvature in [.03] * 60 + [0.] * 60:
    current = curve_accel(previous=previous, curvature=curvature)
    assert abs(current - previous) <= .060001
    previous = current
  assert previous > 0.


@pytest.mark.parametrize('curvature', [None, 0., float('nan'), float('inf')])
def test_absent_or_invalid_prediction_does_not_invent_straight_road_braking(curvature):
  assert curve_accel(curvature=curvature) == curve_accel(brand='other')


@pytest.mark.parametrize('e2e,brand', [(False, 'tesla'), (True, 'other'), (False, 'other')])
def test_other_planners_and_brands_do_not_gain_prediction_constraint(e2e, brand):
  assert curve_accel(e2e=e2e, brand=brand, curvature=.05) == curve_accel(e2e=e2e, brand=brand)


def planner_inputs(monkeypatch, *, model_accel=1., model_stop=False, model_healthy=True, angle=0.):
  # Exercise the actual upstream candidate selection with a deterministic MPC
  # result; no Params access, navigation source, vehicle IO, or real solver.
  monkeypatch.setattr(LongitudinalPlannerSP, 'update', lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlannerSP, 'update_targets', lambda self, sm, v, a, cruise: (cruise, a))
  cp = SimpleNamespace(brand='tesla', steerRatio=12., wheelbase=2.89,
                       openpilotLongitudinalControl=True, longitudinalActuatorDelay=.2)
  mpc = SimpleNamespace(v_solution=10. + 2. * T_IDXS_MPC, a_solution=np.full(len(T_IDXS_MPC), 2.),
                        j_solution=np.zeros(len(T_IDXS_MPC)-1), source=LongitudinalPlanSource.cruise, crash_cnt=0,
                        set_weights=lambda *args, **kwargs: None, set_cur_state=lambda *args: None,
                        update=lambda *args, **kwargs: None)
  planner = LongitudinalPlanner.__new__(LongitudinalPlanner)
  planner.CP, planner.mpc, planner.dt = cp, mpc, .05
  planner.v_desired_filter = FirstOrderFilter(10., 2., .05)
  planner.a_cruise, planner.output_a_target = 3.5, 0.
  planner.is_e2e = lambda sm: True

  class SM(dict):
    seen = alive = valid = {'modelV2': model_healthy}

  sm = SM(carState=SimpleNamespace(vEgo=10., vCruise=108., standstill=False, steeringAngleDeg=angle),
          carControl=SimpleNamespace(orientationNED=[]),
          controlsState=SimpleNamespace(forceDecel=False, longControlState=LongCtrlState.pid),
          selfdriveState=SimpleNamespace(personality=1),
          vehicleParameters=SimpleNamespace(angleOffsetDeg=0.), radarState=SimpleNamespace(),
          modelV2=SimpleNamespace(meta=SimpleNamespace(disengagePredictions=SimpleNamespace(gasPressProbs=[1., 1.])),
                                  action=SimpleNamespace(desiredCurvature=.025, desiredAcceleration=model_accel,
                                                         shouldStop=model_stop)))
  return planner, sm


def test_e2e_selector_rejects_positive_model_acceleration_in_observed_curve(monkeypatch):
  planner, sm = planner_inputs(monkeypatch)
  planner.update(sm)
  assert planner.output_a_target == pytest.approx(-.06)
  assert planner.mpc.source == LongitudinalPlanSource.cruise


def test_e2e_stronger_model_braking_and_stop_remain_selected(monkeypatch):
  planner, sm = planner_inputs(monkeypatch, model_accel=-2., model_stop=True)
  planner.update(sm)
  assert planner.output_a_target == -2.
  assert planner.output_should_stop
  assert planner.mpc.source == LongitudinalPlanSource.e2e


def test_unhealthy_model_cannot_create_curve_ceiling_but_measured_turn_can(monkeypatch):
  planner, sm = planner_inputs(monkeypatch, model_healthy=False)
  planner.update(sm)
  assert planner.output_a_target == 1.
  planner, sm = planner_inputs(monkeypatch, model_healthy=False, angle=60.)
  planner.update(sm)
  assert planner.output_a_target < 0.
