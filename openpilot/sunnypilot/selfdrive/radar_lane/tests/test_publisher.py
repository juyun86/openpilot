from types import SimpleNamespace as namespace

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
               radar_alive=True, radar_valid=True):
    self.data = {"modelV2": model}
    model_mono_time = 1_100_000_000
    self.logMonoTime = {"modelV2": model_mono_time, "radarTracks": model_mono_time - int(radar_age_ms * 1e6)}
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


def _point(track_id=0, y_rel=3.6):
  return namespace(
    trackId=track_id,
    dRel=20.0,
    yRel=y_rel,
    vRel=-1.0,
    deprecated=namespace(measured=True),
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
