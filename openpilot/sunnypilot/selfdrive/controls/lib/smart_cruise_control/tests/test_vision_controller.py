"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
from pathlib import Path
from typing import Any

import numpy as np
from openpilot.common.parameterized import parameterized

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import (
  SmartCruiseControlVision, _ENTERING_CONFIRMATION_FRAMES, _ENTERING_PRED_LAT_ACC_TH, _EXIT_CONFIRMATION_FRAMES,
)
from openpilot.common.test import OpenpilotTestCase

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState


def _th_above_f32(th: float) -> float:
  """
  Return the next representable float32 *above* `th`.
  This avoids flaky comparisons around thresholds due to float32 rounding.
  """
  th32 = np.float32(th)
  above32 = np.nextafter(th32, np.float32(np.inf), dtype=np.float32)
  return float(above32)


def _build_single_spike_filtered(n: int, base: float = 1.0) -> np.ndarray:
  """
  Create an array where max() is >= threshold but p97 is < threshold.
  This demonstrates the behavior difference vs np.amax().

  Note: We intentionally construct using float32-representable values to match
  the data path through cereal/capnp.
  """
  th = float(_ENTERING_PRED_LAT_ACC_TH)
  th32 = float(np.float32(th))

  # numpy percentile default is linear interpolation: idx=(n-1)*p/100
  idx = (n - 1) * 0.97
  w = float(idx - np.floor(idx))

  base32 = float(np.float32(base))

  # Choose spike so that p97 = base + w*(spike-base) < th
  # -> spike < base + (th-base)/w. Use a margin (0.9) and ensure spike >= th.
  if w == 0.0:
    spike = th32 + 1.0
  else:
    spike = base32 + (th32 - base32) / w * 0.9
    spike = max(spike, th32 + 0.01)

  arr = np.full(n, base32, dtype=np.float32)
  arr[-1] = np.float32(spike)
  return arr


def generate_modelV2():
  model = messaging.new_message('modelV2')
  position = log.XYZTData.new_message()
  speed = 30
  position.x = [float(x) for x in (speed + 0.5) * np.array(ModelConstants.T_IDXS)]
  model.modelV2.position = position
  orientation = log.XYZTData.new_message()
  curvature = 0.05
  orientation.x = [float(curvature) for _ in ModelConstants.T_IDXS]
  orientation.y = [0.0 for _ in ModelConstants.T_IDXS]
  model.modelV2.orientation = orientation
  orientationRate = log.XYZTData.new_message()
  orientationRate.z = [float(z) for z in ModelConstants.T_IDXS]
  model.modelV2.orientationRate = orientationRate
  velocity = log.XYZTData.new_message()
  velocity.x = [float(x) for x in (speed + 0.5) * np.ones_like(ModelConstants.T_IDXS)]
  velocity.x[0] = float(speed)  # always start at current speed
  model.modelV2.velocity = velocity
  acceleration = log.XYZTData.new_message()
  acceleration.x = [float(x) for x in np.zeros_like(ModelConstants.T_IDXS)]
  acceleration.y = [float(y) for y in np.zeros_like(ModelConstants.T_IDXS)]
  model.modelV2.acceleration = acceleration

  return model


def generate_carState():
  car_state = messaging.new_message('carState')
  speed = 30
  v_cruise = 50
  car_state.carState.vEgo = float(speed)
  car_state.carState.standstill = False
  car_state.carState.vCruise = float(v_cruise * 3.6)

  return car_state


def generate_controlsState():
  controls_state = messaging.new_message('controlsState')
  controls_state.controlsState.curvature = 0.05
  controls_state.controlsState.desiredCurvature = 0.05

  return controls_state


class TestSmartCruiseControlVision(OpenpilotTestCase):

  def setup_method(self):
    self.params = Params()
    self.reset_params()
    self.scc_v = SmartCruiseControlVision()

    mdl = generate_modelV2()
    cs = generate_carState()
    controls_state = generate_controlsState()
    self.sm: Any = {'modelV2': mdl.modelV2, 'carState': cs.carState, 'controlsState': controls_state.controlsState}

  def reset_params(self):
    self.params.put_bool("SmartCruiseControlVision", True, block=True)

  def test_initial_state(self):
    assert self.scc_v.state == VisionState.disabled
    assert not self.scc_v.is_active
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
    assert self.scc_v.output_a_target == 0.

  def test_system_disabled(self):
    self.params.put_bool("SmartCruiseControlVision", False, block=True)
    self.scc_v.enabled = self.params.get_bool("SmartCruiseControlVision")

    for _ in range(int(10. / DT_MDL)):
      self.scc_v.update(self.sm, True, False, 0., 0., 0.)
    assert self.scc_v.state == VisionState.disabled
    assert not self.scc_v.is_active

  def test_disabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_v.update(self.sm, False, False, 0., 0., 0.)
    assert self.scc_v.state == VisionState.disabled

  def test_transition_disabled_to_enabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_v.update(self.sm, True, False, 0., 0., 0.)
    assert self.scc_v.state == VisionState.enabled

  def test_turn_control_uses_planned_curvature(self):
    self.sm['controlsState'].curvature = 0.001
    self.sm['controlsState'].desiredCurvature = 0.01
    self.scc_v.update(self.sm, True, False, 10.0, 0.0, 0.0)
    assert np.isclose(self.scc_v.current_lat_acc, 1.0)

  def test_route_verified_straight_transient_never_activates_turn_control(self):
    fixture_path = Path(__file__).parent / "fixtures/sccv_straight_transient.json"
    fixture = json.loads(fixture_path.read_text())
    states = []
    for frame in fixture["frames"]:
      mdl = generate_modelV2()
      mdl.modelV2.velocity.x = [1.0] * len(frame["predicted_lateral_accel"])
      mdl.modelV2.orientationRate.z = frame["predicted_lateral_accel"]
      self.sm["modelV2"] = mdl.modelV2
      self.sm["controlsState"].desiredCurvature = frame["desired_curvature"]
      self.sm["controlsState"].curvature = frame["actual_curvature"]
      self.scc_v.update(self.sm, True, False, frame["v_ego"], 0.0, 0.0)
      states.append(self.scc_v.state)

    assert states == [VisionState.enabled] * len(states)

  @parameterized.expand([
      ("p97_just_above_threshold", True),
      ("single_spike_filtered", False),
      ("persistent_high_values", True),
    ], names=["case", "should_enter"])
  def test_max_pred_lat_acc_uses_p97_and_threshold(self, case, should_enter):
    n = len(ModelConstants.T_IDXS)
    th = float(_ENTERING_PRED_LAT_ACC_TH)

    if case == "p97_just_above_threshold":
      # Use the next representable float32 above threshold to avoid float32 rounding flakiness.
      val = _th_above_f32(th)
      pred_lat_accels = np.full(n, np.float32(val), dtype=np.float32)

    elif case == "single_spike_filtered":
      pred_lat_accels = _build_single_spike_filtered(n, base=1.0)

    elif case == "persistent_high_values":
      # Make enough "high" samples so p97 is driven by the persistent trend, not a single outlier.
      high_count = max(2, int(np.ceil(n * 0.03)) + 1)
      pred_lat_accels = np.full(n, np.float32(1.0), dtype=np.float32)
      pred_lat_accels[-high_count:] = np.float32(2.0)
      pred_lat_accels[-1] = np.float32(8.0)  # keep one big outlier too

    else:
      raise AssertionError(f"Unknown case: {case}")

    # Override model predictions so:
    # predicted_lat_accels = abs(orientationRate.z) * velocity.x == pred_lat_accels
    mdl = generate_modelV2()
    mdl.modelV2.velocity.x = [1.0 for _ in range(n)]
    mdl.modelV2.orientationRate.z = [float(x) for x in pred_lat_accels]
    self.sm["modelV2"] = mdl.modelV2

    v_ego = float(MIN_V + 5.0)

    # 1st update: disabled -> enabled
    self.scc_v.update(self.sm, True, False, v_ego, 0.0, 0.0)
    # A real curve must remain above the existing p97 threshold across the
    # route-derived confirmation window before turn control becomes active.
    for _ in range(_ENTERING_CONFIRMATION_FRAMES):
      self.scc_v.update(self.sm, True, False, v_ego, 0.0, 0.0)

    # Controller does percentile on numpy float64 arrays (values already quantized by capnp),
    # so compute expected in float64 to match behavior and avoid interpolation/rounding deltas.
    expected_p97 = float(np.percentile(pred_lat_accels.astype(np.float64), 97))

    # allow tiny numeric differences due to float conversions/interpolation
    assert np.isclose(self.scc_v.max_pred_lat_acc, expected_p97, rtol=1e-6, atol=1e-5)

    if should_enter:
      # We assert entering primarily by state (this is the actual intended behavior).
      assert self.scc_v.state == VisionState.entering
      # Optional sanity: should be >= threshold with some margin (since we used nextafter above threshold).
      assert self.scc_v.max_pred_lat_acc > th

    else:
      # Difference vs np.amax(): max can be above threshold, but p97 stays below it.
      assert float(np.max(pred_lat_accels)) >= th
      assert self.scc_v.max_pred_lat_acc < th
      assert self.scc_v.state == VisionState.enabled

  def set_turn_demand(self, predicted: float, current: float) -> None:
    n = len(ModelConstants.T_IDXS)
    self.sm['modelV2'].velocity.x = [10.0] * n
    self.sm['modelV2'].orientationRate.z = [predicted / 10.0] * n
    self.sm['controlsState'].desiredCurvature = current / 100.0

  def test_recorded_curve_prediction_dip_does_not_release_mid_bend(self):
    fixture = json.loads((Path(__file__).parent / 'fixtures/sccv_turn_release.json').read_text())
    self.scc_v.state = VisionState.entering
    self.scc_v.long_enabled = True
    self.scc_v.long_override = False
    for frame in fixture['frames']:
      self.scc_v.v_ego = frame['v']
      self.scc_v.current_lat_acc = frame['current']
      self.scc_v.max_pred_lat_acc = frame['predicted']
      self.scc_v._update_state_machine()
      # At 07:28:40 the prediction briefly clears, then current/predicted
      # demand returns. Keep control through that dip, but release on exit.
      if frame['ms'] < 1790033322000:
        assert self.scc_v.state == VisionState.entering
    assert self.scc_v.state == VisionState.enabled

  @parameterized.expand([('entering', VisionState.entering), ('leaving', VisionState.leaving)])
  def test_turn_release_requires_sustained_clear_demand(self, _, state):
    self.scc_v.state = state
    self.set_turn_demand(0.9, 0.9)
    for _ in range(_EXIT_CONFIRMATION_FRAMES - 1):
      self.scc_v.update(self.sm, True, False, 10.0, 0.0, 20.0)
      assert self.scc_v.state == state
      assert self.scc_v.is_active
      assert self.scc_v.output_v_target != V_CRUISE_UNSET
    self.scc_v.update(self.sm, True, False, 10.0, 0.0, 20.0)
    assert self.scc_v.state == VisionState.enabled
    assert not self.scc_v.is_active
    assert self.scc_v.output_v_target == V_CRUISE_UNSET

  @parameterized.expand([
    ('entering_current_curve', VisionState.entering, 0.9, 1.2),
    ('entering_prediction_returns', VisionState.entering, 1.2, 0.9),
    ('leaving_current_curve', VisionState.leaving, 0.9, 1.2),
    ('leaving_prediction_returns', VisionState.leaving, 1.2, 0.9),
  ])
  def test_curve_evidence_resets_release_confirmation(self, _, state, predicted, current):
    self.scc_v.state = state
    self.set_turn_demand(0.9, 0.9)
    for _ in range(_EXIT_CONFIRMATION_FRAMES - 1):
      self.scc_v.update(self.sm, True, False, 10.0, 0.0, 20.0)
    self.set_turn_demand(predicted, current)
    self.scc_v.update(self.sm, True, False, 10.0, 0.0, 20.0)
    assert self.scc_v.state == state
    self.set_turn_demand(0.9, 0.9)
    for _ in range(_EXIT_CONFIRMATION_FRAMES - 1):
      self.scc_v.update(self.sm, True, False, 10.0, 0.0, 20.0)
      assert self.scc_v.state == state
    self.scc_v.update(self.sm, True, False, 10.0, 0.0, 20.0)
    assert self.scc_v.state == VisionState.enabled

  @parameterized.expand([
    ('gas', True, True, True, VisionState.overriding),
    ('longitudinal_off', False, False, True, VisionState.disabled),
    ('feature_off', True, False, False, VisionState.disabled),
  ])
  def test_turn_release_confirmation_never_delays_override_or_disable(self, _, long_enabled, override, enabled, expected):
    self.scc_v.state = VisionState.entering
    self.set_turn_demand(0.9, 0.9)
    for _ in range(_EXIT_CONFIRMATION_FRAMES - 1):
      self.scc_v.update(self.sm, True, False, 10.0, 0.0, 20.0)
    self.params.put_bool('SmartCruiseControlVision', enabled, block=True)
    self.scc_v.enabled = enabled
    self.scc_v.update(self.sm, long_enabled, override, 10.0, 0.0, 20.0)
    assert self.scc_v.state == expected
    assert not self.scc_v.is_active
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
