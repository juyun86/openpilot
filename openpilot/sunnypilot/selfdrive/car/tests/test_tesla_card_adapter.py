from types import SimpleNamespace
import time
import pytest

from openpilot.sunnypilot.selfdrive.car.tesla.card_adapter import CONTEXT_STALE_S, LIGHTING_MESSAGE, TeslaCardAdapter, speed_limit_context


class FakeState:
  def __init__(self):
    self.templates = []
    self.longitudinal = []
    self.speed_limit = []

  def update_speed_button_template(self, data, mono_time):
    self.templates.append((data, mono_time))

  def update_longitudinal_context(self, *context):
    self.longitudinal.append(context)

  def update_speed_limit_target(self, *context):
    self.speed_limit.append(context)


class FakeSubMaster:
  def __init__(self, now=10.0):
    self.data = {
      "longitudinalPlanSP": SimpleNamespace(
        longitudinalPlanSource=SimpleNamespace(raw=3),
        speedLimit=SimpleNamespace(
          assist=SimpleNamespace(enabled=True),
          resolver=SimpleNamespace(speedLimitValid=True, speedLimitLastValid=False, speedLimitFinalLast=22.0),
        ),
      ),
      "carControl": SimpleNamespace(
        leftBlinker=True, rightBlinker=False, latActive=True, longActive=True,
        actuators=SimpleNamespace(accel=0.5),
      ),
      "selfdriveStateSP": SimpleNamespace(mads=SimpleNamespace(active=False)),
      "navLaneIntentSP": SimpleNamespace(
        valid=False, signalRequested=False, direction="none", sessionId="", routeRevision=0, requestId=0, maneuverEventId=0,
      ),
      "modelV2": SimpleNamespace(meta=SimpleNamespace(laneChangeState=SimpleNamespace(raw=0), laneChangeDirection=SimpleNamespace(raw=0))),
    }
    self.recv_time = dict.fromkeys(self.data, now)
    self.seen = dict.fromkeys(self.data, True)
    self.valid = dict.fromkeys(self.data, True)
    self.alive = dict.fromkeys(self.data, True)
    self.updated = dict.fromkeys(self.data, True)

  def __getitem__(self, name):
    return self.data[name]


def test_adapter_observes_only_vehicle_speed_templates():
  state = FakeState()
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), FakeSubMaster())

  adapter.observe_can([(100, [(0x3C2, b"valid", 1), (0x3C2, b"wrong-bus", 0), (0x123, b"other", 1)])])

  assert state.templates == [(b"valid", 100)]


def test_adapter_supplies_fresh_context():
  state = FakeState()
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), FakeSubMaster())

  adapter.update_context(10.0)

  assert state.longitudinal[0][0] == 3
  assert state.longitudinal[0][2] is True
  assert state.speed_limit == [(22.0, True)]


def test_adapter_invalidates_stale_context():
  state = FakeState()
  sm = FakeSubMaster()
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), sm)

  adapter.update_context(10.0 + CONTEXT_STALE_S + 0.01)

  assert state.longitudinal[0][2] is False
  assert state.speed_limit == [(0.0, False)]


def test_configured_assist_keeps_resolved_target_while_runtime_state_is_inactive():
  sm = FakeSubMaster()
  sm.data["longitudinalPlanSP"].speedLimit.assist.enabled = False

  assert speed_limit_context(sm, 10.0, assist_configured=True) == (22.0, True)


def test_non_assist_mode_never_exposes_automatic_speed_target():
  sm = FakeSubMaster()

  assert speed_limit_context(sm, 10.0, assist_configured=False) == (0.0, False)


def test_assist_setting_is_forwarded_independently_of_stale_limit():
  state = FakeState()
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), FakeSubMaster())
  adapter.speed_limit_assist_configured = True
  adapter.update_context(10.0 + CONTEXT_STALE_S + 0.01)
  assert state.tesla_speed_limit_assist_enabled is True
  assert state.speed_limit[-1] == (0.0, False)
  adapter.speed_limit_assist_configured = False
  adapter.update_context(10.0)
  assert state.tesla_speed_limit_assist_enabled is False
  assert state.speed_limit[-1] == (0.0, False)


def test_blindspot_state_is_forwarded_to_ambient_controller():
  state = FakeState()
  state.tesla_blindspot_left_level = 1
  state.tesla_blindspot_right_level = 2
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), FakeSubMaster())
  adapter._night_mode = lambda _now_ns: True
  car_state = SimpleNamespace(brakePressed=False, leftBlindspot=True, rightBlindspot=False)
  car_control = SimpleNamespace(latActive=False)

  adapter.control_sends(car_state, car_control, 1_000_000_000)

  assert adapter.ambient.blindspot_side == "both"
  assert adapter.ambient.blindspot_level == 2
  assert adapter.ambient.blindspot_night is True


@pytest.mark.parametrize("side", ["left", "right"])
def test_captured_level_one_flashes_red_without_turn_signal(side):
  from opendbc.can import CANParser

  # Real 0x39B@bus2 warning frame, mirrored for the left-side case.
  data = bytearray.fromhex("4218dfa0b0c89180")
  if side == "left":
    data[0] = 0x12
    data[-1] = (sum(data[:-1]) + 0x9B + 0x03) & 0xFF
  parser = CANParser("tesla_model3_party", [("DAS_status", 0)], 2)
  parser.update([(1_000_000_000, [(0x39B, bytes(data), 2)])])
  values = parser.vl["DAS_status"]
  state = FakeState()
  state.tesla_blindspot_left_level = int(values["DAS_blindSpotRearLeft"])
  state.tesla_blindspot_right_level = int(values["DAS_blindSpotRearRight"])
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), FakeSubMaster())
  adapter._night_mode = lambda _now_ns: True
  cs = SimpleNamespace(brakePressed=False, leftBlinker=False, rightBlinker=False)
  cc = SimpleNamespace(latActive=False)
  frames = []
  for index in range(4):
    now = 1_000_000_000 + index * 100_000_000
    adapter.ambient.observe_frame(now, 0x679, bytes.fromhex("0cffffff00f801"), 1)
    frames.extend(adapter.control_sends(cs, cc, now))
  assert [frame.dat[1:4] for frame in frames] == [bytes((255, 0, 0))] * 4
  assert [frame.dat[4] & 127 for frame in frames] == [50, 0, 50, 0]
  assert adapter.ambient.blindspot_side == side
  # This is an accessory display decision, not a change to vehicle blindspot state.
  assert getattr(state, f"tesla_blindspot_{side}_level") == 1


@pytest.mark.parametrize("level,left_signal,right_signal,expected", [
  (1, False, False, 2), (1, False, True, 2), (1, True, False, 2), (1, True, True, 2),
  (2, False, False, 2), (0, True, False, 0), (3, True, False, 0),
])
def test_ambient_red_requires_valid_warning_but_not_turn_signal(level, left_signal, right_signal, expected):
  state = FakeState()
  state.tesla_blindspot_left_level = level
  state.tesla_blindspot_right_level = 0
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), FakeSubMaster())
  cs = SimpleNamespace(brakePressed=False, leftBlinker=left_signal, rightBlinker=right_signal)
  adapter.control_sends(cs, SimpleNamespace(latActive=False), 1_000_000_000)
  assert adapter.ambient.blindspot_level == expected


def test_signal_changes_do_not_restart_flashing_and_clear_stops_alert():
  state = FakeState()
  state.tesla_blindspot_left_level = 1
  state.tesla_blindspot_right_level = 0
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), FakeSubMaster())
  cs = SimpleNamespace(brakePressed=False, leftBlinker=True, rightBlinker=False)
  cc = SimpleNamespace(latActive=False)
  adapter.control_sends(cs, cc, 1_000_000_000)
  assert adapter.ambient.blindspot_level == 2
  cs.leftBlinker = False
  adapter.control_sends(cs, cc, 1_100_000_000)
  assert adapter.ambient.blindspot_level == 2
  assert adapter.ambient.blindspot_started_ns == 1_000_000_000
  state.tesla_blindspot_left_level = 0
  adapter.control_sends(cs, cc, 1_200_000_000)
  assert adapter.ambient.blindspot_side is None


def lighting_parser(timestamp, left, right, drl):
  return SimpleNamespace(
    ts_nanos={LIGHTING_MESSAGE: {"VCFRONT_lowBeamLeftStatus": timestamp}},
    vl={LIGHTING_MESSAGE: {
      "VCFRONT_lowBeamLeftStatus": left,
      "VCFRONT_lowBeamRightStatus": right,
      "VCFRONT_lowBeamsOnForDRL": drl,
    }},
  )


def test_night_mode_uses_fresh_non_drl_low_beams():
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), FakeSubMaster())
  adapter.lighting_parser = lighting_parser(2_000_000_000, 1, 0, 0)
  assert adapter._night_mode(3_000_000_000)
  adapter.lighting_parser = lighting_parser(2_000_000_000, 1, 0, 1)
  assert not adapter._night_mode(3_000_000_000)
  adapter.lighting_parser = lighting_parser(2_000_000_000, 0, 0, 0)
  assert not adapter._night_mode(3_000_000_000)


def test_unknown_or_stale_lighting_uses_lower_brightness_mode():
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), FakeSubMaster())
  adapter.lighting_parser = None
  assert adapter._night_mode(3_000_000_000)
  adapter.lighting_parser = lighting_parser(1, 0, 0, 0)
  assert adapter._night_mode(3_000_000_001)


def test_non_tesla_adapter_is_inert():
  state = FakeState()
  adapter = TeslaCardAdapter("toyota", SimpleNamespace(CS=state), FakeSubMaster())

  adapter.observe_can([(100, [(0x3C2, b"ignored", 1)])])
  adapter.update_context(10.0)

  assert not state.templates
  assert not state.longitudinal
  assert not state.speed_limit


def test_navigation_lane_intent_requests_and_cancels_bounded_tesla_signal_session():
  now = time.monotonic()
  sm = FakeSubMaster(now=now)
  sm.data["navLaneIntentSP"] = SimpleNamespace(
    valid=True, signalRequested=True, direction="left", sessionId="session-a", routeRevision=7, requestId=3, maneuverEventId=3,
  )
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), sm)

  class FakeValidation:
    def __init__(self):
      self.requests = []
      self.cancels = []

    def submit_request(self, test_id, direction, now_nanos, session_timeout_ns=None, **kwargs):
      self.requests.append((test_id, direction, now_nanos, session_timeout_ns, kwargs))
      return True

    def request_cancel(self, test_id, now_nanos):
      self.cancels.append((test_id, now_nanos))
      return True

  validation = FakeValidation()
  adapter.validation = validation
  adapter._update_nav_turn_signal(100)
  adapter._update_nav_turn_signal(101)
  assert validation.requests == [(
    "nav-fa57a52d-7-3-left", "left", 100, 60_000_000_000, {"hold_until_cancel": True},
  )]

  sm.data["navLaneIntentSP"].signalRequested = False
  adapter._update_nav_turn_signal(102)
  assert validation.cancels == [("nav-fa57a52d-7-3-left", 102)]


def test_temporarily_busy_signal_controller_retries_with_bounded_backoff():
  now = time.monotonic()
  sm = FakeSubMaster(now=now)
  sm.data["navLaneIntentSP"] = SimpleNamespace(
    valid=True, signalRequested=True, direction="right", sessionId="session-b", routeRevision=2, requestId=4, maneuverEventId=4,
  )
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), sm)

  class BusyOnce:
    configured = True

    def __init__(self):
      self.calls = []

    def submit_request(self, *args, **kwargs):
      self.calls.append((args, kwargs))
      return len(self.calls) > 1

  validation = BusyOnce()
  adapter.validation = validation
  adapter._update_nav_turn_signal(100)
  adapter._update_nav_turn_signal(200)
  adapter._update_nav_turn_signal(500_000_100)
  assert len(validation.calls) == 2


def test_navigation_signal_retries_after_realtime_controller_cancels_inactive_lateral_session():
  now = time.monotonic()
  sm = FakeSubMaster(now=now)
  sm.data["navLaneIntentSP"] = SimpleNamespace(
    valid=True, signalRequested=True, direction="right", sessionId="session-retry", routeRevision=3, requestId=9, maneuverEventId=9,
  )
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), sm)
  adapter.validation.configured = True

  adapter._update_nav_turn_signal(12_000_000_000)
  assert adapter.validation.status() is not None
  adapter.validation.update_lane_change_context(
    12_000_000_001, valid=True, state=0, direction=0, lateral_active=False, brake_pressed=False,
  )
  assert adapter.validation.status() is None

  adapter._update_nav_turn_signal(13_000_000_000)
  assert adapter.validation.status() is not None


def test_navigation_signal_waits_for_lateral_control_before_opening_session():
  now = time.monotonic()
  sm = FakeSubMaster(now=now)
  sm.data["navLaneIntentSP"] = SimpleNamespace(
    valid=True, signalRequested=True, direction="left", sessionId="session-wait", routeRevision=2, requestId=5, maneuverEventId=5,
  )
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), sm)
  adapter.validation.configured = True

  adapter._update_nav_turn_signal(12_000_000_000, lateral_active=False)
  assert adapter.validation.status() is None
  adapter._update_nav_turn_signal(12_500_000_000, lateral_active=True)
  assert adapter.validation.status() is not None


def test_pre_turn_lamp_transitions_to_same_direction_lane_change_without_blinking_off():
  now = time.monotonic()
  sm = FakeSubMaster(now=now)
  sm.data["navLaneIntentSP"] = SimpleNamespace(
    valid=True, signalRequested=True, direction="left", sessionId="session-a", routeRevision=7, requestId=11, maneuverEventId=11,
  )
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), sm)

  class Validation:
    configured = True

    def __init__(self):
      self.requests = []
      self.cancels = []

    def submit_request(self, test_id, direction, now_nanos, session_timeout_ns=None, **kwargs):
      self.requests.append((test_id, direction, now_nanos, session_timeout_ns, kwargs))
      return True

    def request_cancel(self, test_id, now_nanos):
      self.cancels.append((test_id, now_nanos))
      return True

  validation = Validation()
  adapter.validation = validation
  adapter._update_nav_turn_signal(100)

  # The lane coordinator takes ownership after the lamp is already on. A new
  # request id in the same session/revision/direction must reuse that session.
  sm.data["navLaneIntentSP"].requestId = 1
  adapter._update_nav_turn_signal(101)

  assert len(validation.requests) == 1
  assert not validation.cancels

  sm.data["navLaneIntentSP"].signalRequested = False
  adapter._update_nav_turn_signal(102)
  assert validation.cancels == [("nav-fa57a52d-7-11-left", 102)]


def test_navigation_signal_direction_change_cancels_old_lamp_before_requesting_new_one():
  now = time.monotonic()
  sm = FakeSubMaster(now=now)
  sm.data["navLaneIntentSP"] = SimpleNamespace(
    valid=True, signalRequested=True, direction="left", sessionId="session-a", routeRevision=7, requestId=11, maneuverEventId=11,
  )
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), sm)

  class Validation:
    configured = True

    def __init__(self):
      self.requests = []
      self.cancels = []

    def submit_request(self, test_id, direction, now_nanos, session_timeout_ns=None, **kwargs):
      self.requests.append((test_id, direction, now_nanos, session_timeout_ns, kwargs))
      return True

    def request_cancel(self, test_id, now_nanos):
      self.cancels.append((test_id, now_nanos))
      return True

  validation = Validation()
  adapter.validation = validation
  adapter._update_nav_turn_signal(100)

  sm.data["navLaneIntentSP"].direction = "right"
  sm.data["navLaneIntentSP"].requestId = 12
  adapter._update_nav_turn_signal(101)
  assert validation.cancels == [("nav-fa57a52d-7-11-left", 101)]
  assert len(validation.requests) == 1

  adapter._update_nav_turn_signal(101 + 500_000_000)
  assert validation.requests[-1][1] == "right"


def test_unsent_navigation_attempts_do_not_exhaust_the_session():
  sm = FakeSubMaster(now=time.monotonic())
  sm.data["navLaneIntentSP"] = SimpleNamespace(
    valid=True, signalRequested=True, direction="left", sessionId="session-failure",
    routeRevision=1, requestId=1, maneuverEventId=42,
  )
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), sm)
  adapter.validation.configured = True
  intent = sm.data["navLaneIntentSP"]
  for attempt in range(20):
    stamp = 10_000_000_000 * (attempt + 1)
    intent.requestId += 1
    intent.routeRevision += 1
    adapter._update_nav_turn_signal(stamp)
    assert adapter.validation.status() is not None
    adapter.validation.advance_time(stamp + 2_500_000_000)
    assert adapter.validation.status() is None
  adapter._update_nav_turn_signal(210_000_000_000)
  assert adapter.validation.status() is not None


def test_navigation_future_receive_time_cannot_open_lamp_session():
  sm = FakeSubMaster(now=time.monotonic() + 60)
  sm.data["navLaneIntentSP"] = SimpleNamespace(
    valid=True, signalRequested=True, direction="left", sessionId="session-future",
    routeRevision=1, requestId=1, maneuverEventId=42,
  )
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=FakeState()), sm)
  adapter.validation.configured = True
  adapter._update_nav_turn_signal(10_000_000_000)
  assert adapter.validation.status() is None
