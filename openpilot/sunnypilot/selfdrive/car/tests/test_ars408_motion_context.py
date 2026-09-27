import math
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.locationd.helpers import Pose, PoseCalibrator
from openpilot.sunnypilot.selfdrive.car.tesla.card_adapter import (
  RADAR_CALIBRATION_MAX_AGE_NS, TeslaCardAdapter, radar_yaw_rate_context,
)
from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.ars408.transmitter import ARS408Transmitter, MOTION_MAX_AGE_NS
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


NOW = 10_000_000_000


class MotionSubMaster:
  def __init__(self):
    self.data = {
      "deviceMotion": log.DeviceMotion.new_message(
        timestamp=NOW, inputsOK=True, sensorsOK=True, posenetOK=True,
        angularVelocityDevice={"x": 0.0, "y": 0.0, "z": math.radians(1.0), "valid": True},
      ),
      "extrinsicsCalibration": log.ExtrinsicsCalibration.new_message(calStatus="calibrated", rpyCalib=[0.0, 0.0, 0.0]),
    }
    self.seen = dict.fromkeys(self.data, True)
    self.valid = dict.fromkeys(self.data, True)
    self.logMonoTime = dict.fromkeys(self.data, NOW)

  def __getitem__(self, key):
    return self.data[key]


def test_local_angular_measurement_does_not_require_global_orientation():
  sm = MotionSubMaster()
  assert not sm["deviceMotion"].orientationNED.valid
  yaw, timestamp = radar_yaw_rate_context(sm, NOW, PoseCalibrator())
  assert yaw == pytest.approx(math.radians(1)) and timestamp == NOW


def test_mount_pitch_is_applied_using_existing_pose_calibration():
  sm = MotionSubMaster()
  sm["extrinsicsCalibration"].rpyCalib = [0, math.pi / 6, 0]
  sm["deviceMotion"].angularVelocityDevice.x = 0.1
  sm["deviceMotion"].angularVelocityDevice.z = 0.2
  yaw, _ = radar_yaw_rate_context(sm, NOW, PoseCalibrator())
  assert yaw == pytest.approx(0.05 + 0.1 * math.sqrt(3))


@pytest.mark.parametrize("service", ["deviceMotion", "extrinsicsCalibration"])
@pytest.mark.parametrize("condition", ["unseen", "invalid", "stale", "future", "zero_time"])
def test_invalid_or_delayed_publication_is_rejected(service, condition):
  sm = MotionSubMaster()
  max_age = MOTION_MAX_AGE_NS if service == "deviceMotion" else RADAR_CALIBRATION_MAX_AGE_NS
  if condition == "unseen":
    sm.seen[service] = False
  elif condition == "invalid":
    sm.valid[service] = False
  else:
    sm.logMonoTime[service] = {"stale": NOW - max_age - 1, "future": NOW + 1, "zero_time": 0}[condition]
  assert radar_yaw_rate_context(sm, NOW, PoseCalibrator()) == (None, 0)


@pytest.mark.parametrize("field", ["inputsOK", "sensorsOK", "posenetOK"])
def test_unhealthy_motion_is_rejected(field):
  sm = MotionSubMaster()
  setattr(sm["deviceMotion"], field, False)
  assert radar_yaw_rate_context(sm, NOW, PoseCalibrator()) == (None, 0)


@pytest.mark.parametrize("case", ["measurement_invalid", "measurement_stale", "measurement_future", "measurement_zero",
                                 "nan", "inf", "uncalibrated", "bad_calibration", "short_calibration"])
def test_measurement_and_calibration_failures_invalidate_cached_sample(case):
  sm = MotionSubMaster()
  calibrator = PoseCalibrator()
  cache = {}
  assert radar_yaw_rate_context(sm, NOW, calibrator, cache)[0] is not None
  motion, calibration = sm["deviceMotion"], sm["extrinsicsCalibration"]
  if case == "measurement_invalid":
    motion.angularVelocityDevice.valid = False
  elif case.startswith("measurement_"):
    motion.timestamp = {"measurement_stale": NOW - MOTION_MAX_AGE_NS - 1, "measurement_future": NOW + 1,
                        "measurement_zero": 0}[case]
  elif case in ("nan", "inf"):
    motion.angularVelocityDevice.x = float(case)
  elif case == "uncalibrated":
    calibration.calStatus = "uncalibrated"
  elif case == "bad_calibration":
    calibration.rpyCalib = [0, math.nan, 0]
  else:
    calibration.rpyCalib = [0, 0]
  assert radar_yaw_rate_context(sm, NOW, calibrator, cache) == (None, 0)


def test_cached_sample_keeps_measurement_age_and_recovers_on_new_sample(monkeypatch):
  sm, calibrator, cache = MotionSubMaster(), PoseCalibrator(), {}
  calls = []
  feed = calibrator.feed_extrinsics_calibration

  def counted_feed(value):
    calls.append(1)
    feed(value)

  monkeypatch.setattr(calibrator, "feed_extrinsics_calibration", counted_feed)
  expected = radar_yaw_rate_context(sm, NOW, calibrator, cache)
  for age in (10_000_000, 50_000_000, MOTION_MAX_AGE_NS):
    assert radar_yaw_rate_context(sm, NOW + age, calibrator, cache) == expected
  assert len(calls) == 1
  assert radar_yaw_rate_context(sm, NOW + MOTION_MAX_AGE_NS + 1, calibrator, cache) == (None, 0)
  now = NOW + MOTION_MAX_AGE_NS + 2
  sm.logMonoTime["deviceMotion"] = sm["deviceMotion"].timestamp = now
  sm["deviceMotion"].angularVelocityDevice.z = 0.2
  assert radar_yaw_rate_context(sm, now, calibrator, cache) == (pytest.approx(0.2), now)
  assert len(calls) == 1
  sm["extrinsicsCalibration"].rpyCalib = [0, math.pi / 3, 0]
  sm.logMonoTime["extrinsicsCalibration"] = now
  assert radar_yaw_rate_context(sm, now, calibrator, cache) == (pytest.approx(0.1), now)
  assert len(calls) == 2


@pytest.mark.parametrize("rpy", [[0, 0, 0], [0.12, -0.23, 0.34], [-0.4, 0.3, -0.2]])
@pytest.mark.parametrize("xyz", [[0.1, -0.2, 0.3], [-0.2, 0.3, -0.1], [0, 0, 0]])
def test_angular_only_matches_full_pose_with_cache(rpy, xyz):
  sm, calibrator, cache = MotionSubMaster(), PoseCalibrator(), {}
  sm["extrinsicsCalibration"].rpyCalib = rpy
  angular = sm["deviceMotion"].angularVelocityDevice
  angular.x, angular.y, angular.z = xyz
  actual, stamp = radar_yaw_rate_context(sm, NOW, calibrator, cache)
  reference = calibrator.build_calibrated_pose(Pose.from_device_motion(sm["deviceMotion"])).angular_velocity.z
  assert actual == pytest.approx(reference, abs=1e-14)
  assert stamp == NOW
  assert radar_yaw_rate_context(sm, NOW + 1, calibrator, cache) == (actual, stamp)


def test_adapter_feeds_transmitter_without_modifying_carstate_and_invalidates_on_loss():
  tx = ARS408Transmitter(structs.CarParamsSP(flags=TeslaFlagsSP.ARS408_RADAR))
  ci = SimpleNamespace(CC=SimpleNamespace(ars408_transmitter=tx), CS=SimpleNamespace())
  sm = MotionSubMaster()
  adapter = TeslaCardAdapter("tesla", ci, sm)
  state = structs.CarState(canValid=True, vEgoRaw=20.0, yawRate=0.0)
  # Replay clock is separate from the wall-clock context used by other features.
  adapter.update_context(1000.0, radar_now_ns=NOW)
  assert tx.update(5, state, NOW)[1][1].hex() == "8064"
  assert state.yawRate == 0.0
  sm["deviceMotion"].sensorsOK = False
  radar_data = structs.RadarData()
  adapter.update_context(1000.0, radar_now_ns=NOW, radar_data=radar_data)
  assert tx.update(10, state, NOW) == []
  assert radar_data.errors.radarUnavailableTemporary
  sm["deviceMotion"].sensorsOK = True
  radar_data = structs.RadarData()
  radar_data.errors.radarFault = True
  adapter.update_context(1000.0, radar_now_ns=NOW, radar_data=radar_data)
  assert len(tx.update(15, state, NOW)) == 2
  assert not radar_data.errors.radarUnavailableTemporary
  assert radar_data.errors.radarFault


def test_out_of_range_source_marks_radar_unavailable_before_transmit():
  tx = ARS408Transmitter(structs.CarParamsSP(flags=TeslaFlagsSP.ARS408_RADAR))
  ci = SimpleNamespace(CC=SimpleNamespace(ars408_transmitter=tx), CS=SimpleNamespace())
  sm = MotionSubMaster()
  sm["deviceMotion"].angularVelocityDevice.z = math.radians(101)
  adapter = TeslaCardAdapter("tesla", ci, sm)
  radar_data = structs.RadarData()
  adapter.update_context(10.0, radar_data=radar_data)
  assert radar_data.errors.radarUnavailableTemporary
  assert not tx.yaw_rate_valid(NOW)


@pytest.mark.parametrize("brand, flags", [("tesla", 0), ("other", TeslaFlagsSP.ARS408_RADAR)])
def test_disabled_radar_does_not_read_motion_services(brand, flags):
  tx = ARS408Transmitter(structs.CarParamsSP(flags=flags))
  ci = SimpleNamespace(CC=SimpleNamespace(ars408_transmitter=tx), CS=SimpleNamespace())
  adapter = TeslaCardAdapter(brand, ci, None)
  adapter.update_context(10.0)
  assert tx.yaw_rate_rad_s is None
