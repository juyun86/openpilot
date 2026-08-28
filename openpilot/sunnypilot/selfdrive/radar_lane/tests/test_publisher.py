from types import SimpleNamespace as namespace

import pytest

from openpilot.sunnypilot.selfdrive.radar_lane.publisher import RadarLaneStatePublisher


class FakeLane:
  def __init__(self):
    self.occupancy = "unknown"
    self.geometrySource = "none"
    self.geometryConfidence = 0.0
    self.evaluatedDistance = 0.0
    self.targetCount = 0
    self.closestTarget = {}


class FakeMessage:
  def __init__(self):
    self.valid = False
    self.radarLaneStateSP = namespace(
      leftAhead=FakeLane(),
      centerAhead=FakeLane(),
      rightAhead=FakeLane(),
    )


class FakeSubMaster:
  def __init__(self, model, healthy=True, radar_updated=True, radar_age_ms=100.0,
               radar_alive=True, radar_valid=True, model_mono_time=1_100_000_000,
               radar_mono_time=None):
    self.data = {"modelV2": model}
    if radar_mono_time is None:
      radar_mono_time = model_mono_time - int(radar_age_ms * 1e6)
    self.logMonoTime = {"modelV2": model_mono_time, "radarTracks": radar_mono_time}
    self.updated = {"modelV2": True, "radarTracks": radar_updated}
    self.alive = {"modelV2": True, "radarTracks": radar_alive}
    self.valid = {"modelV2": True, "radarTracks": radar_valid}
    self.healthy = healthy

  def __getitem__(self, name):
    return self.data[name]

  def all_checks(self):
    return self.healthy


def _model():
  xs = [0.0, 50.0, 100.0]

  def line(y):
    return namespace(x=xs, y=[y, y, y])

  return namespace(
    position=line(0.0),
    laneLines=[line(-5.4), line(-1.8), line(1.8), line(5.4)],
    laneLineProbs=[1.0, 1.0, 1.0, 1.0],
  )


def _point(track_id=0, d_rel=20.0, y_rel=3.6, v_rel=-1.0, yv_rel=0.0, measured=True):
  return namespace(
    trackId=track_id,
    dRel=d_rel,
    yRel=y_rel,
    vRel=v_rel,
    deprecated=namespace(measured=measured, yvRel=yv_rel),
  )


def _factory(service):
  assert service == "radarLaneStateSP"
  return FakeMessage()


def test_publisher_builds_logged_three_lane_contract_and_preserves_id_zero():
  publisher = RadarLaneStatePublisher(1.52, _factory)
  message = publisher.build_message(
    FakeSubMaster(_model()),
    namespace(points=[_point()], errors=namespace(to_dict=lambda: {})),
  )

  assert message.valid
  assert message.radarLaneStateSP.radarAgeMs == 100.0
  assert message.radarLaneStateSP.radarFresh
  assert message.radarLaneStateSP.uniqueTargetCount == 1
  assert len(message.radarLaneStateSP.targets) == 1
  assert message.radarLaneStateSP.targets[0]["laneMask"] == 1
  assert message.radarLaneStateSP.leftAhead.occupancy == "occupied"
  assert message.radarLaneStateSP.leftAhead.closestTarget["trackId"] == 0
  assert message.radarLaneStateSP.centerAhead.occupancy == "clear"


def test_radar_error_returns_invalid_unknown_lanes():
  publisher = RadarLaneStatePublisher(1.52, _factory)
  message = publisher.build_message(
    FakeSubMaster(_model()),
    namespace(points=[_point()], errors=namespace(to_dict=lambda: {"canError": True})),
  )

  assert not message.valid
  assert not message.radarLaneStateSP.radarFresh
  assert message.radarLaneStateSP.leftAhead.occupancy == "unknown"
  assert message.radarLaneStateSP.centerAhead.occupancy == "unknown"
  assert message.radarLaneStateSP.rightAhead.occupancy == "unknown"


def test_all_targets_are_counted_and_bounded_unique_list_reports_truncation():
  publisher = RadarLaneStatePublisher(1.52, _factory)
  points = [_point(track_id=index, d_rel=float(index + 1), y_rel=0.0) for index in range(30)]
  message = publisher.build_message(
    FakeSubMaster(_model()),
    namespace(points=points, errors=namespace(to_dict=lambda: {})),
  )

  state = message.radarLaneStateSP
  assert message.valid
  assert state.centerAhead.targetCount == 30
  assert state.uniqueTargetCount == 30
  assert state.publishedTargetLimit == 24
  assert state.targetsTruncated
  assert len(state.targets) == 24
  assert [target["trackId"] for target in state.targets] == list(range(24))


def test_far_overtaking_target_is_published_as_cut_in_candidate_even_when_not_closest():
  publisher = RadarLaneStatePublisher(1.52, _factory)
  first_points = [
    _point(track_id=0, d_rel=10.0, y_rel=3.6),
    _point(track_id=5, d_rel=30.0, y_rel=3.0, yv_rel=99.0),
  ]
  publisher.build_message(
    FakeSubMaster(_model(), model_mono_time=1_020_000_000, radar_mono_time=1_000_000_000),
    namespace(points=first_points, errors=namespace(to_dict=lambda: {})),
  )

  second_points = [
    _point(track_id=0, d_rel=10.0, y_rel=3.6),
    _point(track_id=5, d_rel=30.0, y_rel=2.5, yv_rel=99.0),
  ]
  message = publisher.build_message(
    FakeSubMaster(_model(), model_mono_time=1_120_000_000, radar_mono_time=1_100_000_000),
    namespace(points=second_points, errors=namespace(to_dict=lambda: {})),
  )

  state = message.radarLaneStateSP
  assert message.valid
  assert state.leftAhead.closestTarget["trackId"] == 0
  assert state.cutInCandidateCount == 1
  assert state.cutInCandidate["trackId"] == 5
  assert state.cutInCandidate["cutInCandidate"]
  assert state.cutInCandidate["lateralSpeedValid"]
  assert state.cutInCandidate["lateralSpeed"] == pytest.approx(-5.0)
  assert state.cutInCandidate["timeToLaneCross"] == pytest.approx(0.14)
  assert state.targets[0]["trackId"] == 5
  # The unverified raw ARS408 velocity is exposed, but not used for prediction.
  assert state.cutInCandidate["yvRel"] == 99.0


def test_right_lane_motion_toward_center_is_candidate_but_motion_away_is_not():
  publisher = RadarLaneStatePublisher(1.52, _factory)
  publisher.build_message(
    FakeSubMaster(_model(), model_mono_time=1_020_000_000, radar_mono_time=1_000_000_000),
    namespace(points=[_point(track_id=9, y_rel=-3.0), _point(track_id=10, y_rel=-3.0)], errors=namespace(to_dict=lambda: {})),
  )
  message = publisher.build_message(
    FakeSubMaster(_model(), model_mono_time=1_120_000_000, radar_mono_time=1_100_000_000),
    namespace(points=[_point(track_id=9, y_rel=-2.5), _point(track_id=10, y_rel=-3.5)], errors=namespace(to_dict=lambda: {})),
  )

  targets = {target["trackId"]: target for target in message.radarLaneStateSP.targets}
  assert targets[9]["lateralSpeed"] == pytest.approx(5.0)
  assert targets[9]["cutInCandidate"]
  assert targets[10]["lateralSpeed"] == pytest.approx(-5.0)
  assert not targets[10]["cutInCandidate"]
  assert message.radarLaneStateSP.cutInCandidate["trackId"] == 9


def test_repeated_model_tick_does_not_reapply_same_radar_sample_as_motion():
  publisher = RadarLaneStatePublisher(1.52, _factory)
  radar_data = namespace(points=[_point(track_id=8, y_rel=3.0)], errors=namespace(to_dict=lambda: {}))
  publisher.build_message(
    FakeSubMaster(_model(), model_mono_time=1_020_000_000, radar_mono_time=1_000_000_000),
    radar_data,
  )
  repeated = publisher.build_message(
    FakeSubMaster(_model(), model_mono_time=1_040_000_000, radar_mono_time=1_000_000_000, radar_updated=False),
    radar_data,
  )

  target = repeated.radarLaneStateSP.targets[0]
  assert not target["lateralSpeedValid"]
  assert not target["cutInCandidate"]


def test_recent_radar_is_fresh_even_when_not_updated_on_model_tick():
  publisher = RadarLaneStatePublisher(1.52, _factory)
  message = publisher.build_message(
    FakeSubMaster(_model(), radar_updated=False),
    namespace(points=[_point()], errors=namespace(to_dict=lambda: {})),
  )

  assert message.valid
  assert message.radarLaneStateSP.radarFresh


def test_stale_or_unhealthy_radar_returns_invalid_unknown():
  publisher = RadarLaneStatePublisher(1.52, _factory)
  radar_data = namespace(points=[_point()], errors=namespace(to_dict=lambda: {}))

  for sm in (
    FakeSubMaster(_model(), radar_age_ms=151.0),
    FakeSubMaster(_model(), radar_alive=False),
    FakeSubMaster(_model(), radar_valid=False),
  ):
    message = publisher.build_message(sm, radar_data)
    assert not message.valid
    assert not message.radarLaneStateSP.radarFresh
    assert message.radarLaneStateSP.leftAhead.occupancy == "unknown"


def test_radar_unavailable_cannot_publish_clear_or_occupied():
  publisher = RadarLaneStatePublisher(1.52, _factory, radar_available=False)
  message = publisher.build_message(
    FakeSubMaster(_model()),
    namespace(points=[_point()], errors=namespace(to_dict=lambda: {})),
  )

  assert not message.valid
  assert message.radarLaneStateSP.leftAhead.occupancy == "unknown"
  assert message.radarLaneStateSP.centerAhead.occupancy == "unknown"
  assert message.radarLaneStateSP.rightAhead.occupancy == "unknown"


if __name__ == "__main__":
  tests = [value for name, value in globals().items() if name.startswith("test_") and callable(value)]
  for test in tests:
    test()
  print(f"{len(tests)} radar lane publisher tests passed")
