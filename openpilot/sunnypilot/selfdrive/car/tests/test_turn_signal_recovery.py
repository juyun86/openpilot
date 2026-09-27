"""Real controller/adapter with raw CAN inputs; returned bytes are never sent."""
import time

import pytest

from openpilot.sunnypilot.selfdrive.car.tesla import validation_controller as v
from openpilot.sunnypilot.selfdrive.car.tests.test_nav_signal_retry_budget import adapter_for


def template(counter):
  data = bytearray([0xA5, 0x8C, 0x61, 0xB4, 0x5A, 0xC3, (counter << 4) | 7, 0])
  data[7] = v.tesla_body_controls_checksum(data)
  return bytes(data)


def front(left=0, right=0):
  return ((left << 50) | (right << 52)).to_bytes(8, 'little')


def ui(left=0, right=0):
  data = bytearray(7)
  data[3] = left | (right << 2)
  return bytes(data)


def cancelling(direction='left', *, prior_on=False, ack=True):
  c = v.TeslaTurnSignalRealtimeController(True)
  t = 10_000_000_000
  assert c.submit_request('nav-original', direction, t, hold_until_cancel=True)
  c.observe_frame(t, 0x3E9, template(4), 1)
  sent = c.take_can_sends(t)[0]
  c.observe_frame(t + 1, 0x3E9, sent.dat, 0x81)
  if prior_on:
    c.observe_frame(t + 2, 0x3F5, front(**{direction: 1}), 1)
    c.observe_frame(t + 3, 0x311, ui(**{direction: 2}), 0)
  c.request_cancel(None, t + 4)
  c.observe_frame(t + 5, 0x3E9, template(5), 1)
  cancel = c.take_can_sends(t + 5)[0]
  if ack:
    c.observe_frame(t + 6, 0x3E9, cancel.dat, 0x81)
  return c, t, cancel


def observe_off(c, stamp):
  c.observe_frame(stamp, 0x311, ui(), 0)
  c.observe_frame(stamp + 1, 0x3F5, front(), 1)


@pytest.mark.parametrize('direction', ['left', 'right'])
@pytest.mark.parametrize('prior_on', [False, True])
@pytest.mark.parametrize('late', [False, True])
def test_confirmed_cancel_recovers_without_requiring_prior_lamp_on(direction, prior_on, late):
  c, t, _ = cancelling(direction, prior_on=prior_on)
  if late:
    c.advance_time(t + 2_000_000_000)
    assert c.status()['phase'] == 'cancel_failed'
    assert c.drain_completed()[0][0]['result'] == 'CANCEL_NOT_CONFIRMED'
    assert not c.submit_request('other-route', 'right', t + 2_000_000_001)
  observe_off(c, t + 3_000_000_000 if late else t + 10)
  result = c.drain_completed()[0][0]
  assert result['result'] == ('CANCEL_RECOVERED' if late else 'PASS')
  assert c.status() is None
  assert c.submit_request('next-route', 'right', t + 4_000_000_000)


@pytest.mark.parametrize('bad_ui', [1, 2, 3])
def test_dark_blink_phase_and_unknown_ui_are_not_off(bad_ui):
  c, t, _ = cancelling()
  c.observe_frame(t + 10, 0x311, ui(left=bad_ui), 0)
  c.observe_frame(t + 11, 0x3F5, front(), 1)
  assert c.status() is not None
  c.advance_time(t + 2_000_000_000)
  assert c.status()['phase'] == 'cancel_failed'
  observe_off(c, t + 3_000_000_000)
  assert c.status() is None


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('state', [1, 2, 3])
def test_on_fault_or_sna_of_either_lamp_blocks_recovery(side, state):
  c, t, _ = cancelling()
  c.advance_time(t + 2_000_000_000)
  c.observe_frame(t + 3_000_000_000, 0x311, ui(), 0)
  c.observe_frame(t + 3_000_000_001, 0x3F5, front(**{side: state}), 1)
  assert c.status()['phase'] == 'cancel_failed'


@pytest.mark.parametrize('source', [0, 0x81, 0xC1])
def test_wrong_bus_or_non_rx_front_frame_cannot_recover(source):
  c, t, _ = cancelling()
  c.advance_time(t + 2_000_000_000)
  c.observe_frame(t + 3_000_000_000, 0x311, ui(), 0)
  c.observe_frame(t + 3_000_000_001, 0x3F5, front(), source)
  assert c.status()['phase'] == 'cancel_failed'


def test_stale_or_pre_cancel_off_is_not_recovery_proof():
  c, t, _ = cancelling()
  observe_off(c, t - 1_000_000_000)
  assert c.status() is not None
  c.advance_time(t + 2_000_000_000)
  c.observe_frame(t + 3_000_000_000, 0x311, ui(), 0)
  c.observe_frame(t + 3_000_000_001 + v.VEHICLE_FEEDBACK_TIMEOUT_NS, 0x3F5, front(), 1)
  assert c.status()['phase'] == 'cancel_failed'
  observe_off(c, t + 6_000_000_000)
  assert c.status() is None


def test_delayed_cancel_echo_requires_new_off_feedback_and_can_recover():
  c, t, cancel = cancelling(ack=False)
  c.advance_time(t + v.CANCEL_TOTAL_TIMEOUT_NS + 10)
  c.advance_time(t + v.CANCEL_TOTAL_TIMEOUT_NS + 11)
  assert c.status()['phase'] == 'cancel_failed'
  observe_off(c, t + 6_000_000_000)
  assert c.status() is not None  # Panda cancellation still unresolved.
  c.observe_frame(t + 6_000_000_002, 0x3E9, cancel.dat, 0x81)
  assert c.status() is not None  # Old OFF cannot be reused.
  observe_off(c, t + 6_000_000_003)
  assert c.status() is None


def test_recovered_template_resumes_only_unsent_cancel_then_allows_new_action():
  c = v.TeslaTurnSignalRealtimeController(True)
  t = 10_000_000_000
  c.submit_request('first', 'left', t, hold_until_cancel=True)
  c.observe_frame(t, 0x3E9, template(4), 1)
  action = c.take_can_sends(t)[0]
  c.observe_frame(t + 1, 0x3E9, action.dat, 0x81)
  c.request_cancel(None, t + 2)
  c.advance_time(t + 2_000_000_000)
  assert c.status()['result'] == 'CANCEL_NOT_SENT'
  c.observe_frame(t + 3_000_000_000, 0x3E9, template(6), 1)
  cancel = c.take_can_sends(t + 3_000_000_000, cancel_only=True)[0]
  assert v.decode_body_controls(cancel.dat)['turn_request'] == 3
  assert not c.submit_request('next', 'right', t + 3_000_000_001)
  c.observe_frame(t + 3_000_000_002, 0x3E9, cancel.dat, 0x81)
  observe_off(c, t + 3_000_000_003)
  assert c.status() is None
  assert c.submit_request('next', 'right', t + 3_000_000_005)


def test_recovery_clears_adapter_owner_without_adding_another_delay():
  c, t, _ = cancelling()
  a = adapter_for()
  a.validation = c
  a._active_nav_signal_test_id = 'nav-original'
  a._last_nav_signal_request = ('route', 1, 1, 'left')
  c.advance_time(t + 2_000_000_000)
  a._update_nav_turn_signal(t + 2_000_000_001)
  assert c.status()['phase'] == 'cancel_failed'
  observe_off(c, t + 3_000_000_000)
  a.sm.recv_time['navLaneIntentSP'] = time.monotonic()
  a._update_nav_turn_signal(t + 3_000_000_002)
  assert c.status()['phase'] == 'waiting_vehicle_feedback'


def test_cancel_failure_remains_visible_until_real_recovery():
  class Params:
    def __init__(self):
      self.data = {}
    def get(self, key):
      return self.data.get(key)
    def put(self, key, value):
      self.data[key] = value
    def remove(self, key):
      self.data.pop(key, None)

  c, t, _ = cancelling()
  p = Params()
  c.advance_time(t + 2_000_000_000)
  c.service_params(p, t + 2_000_000_001)
  assert p.get(v.STATUS_PARAM)['result'] == 'CANCEL_NOT_CONFIRMED'
  observe_off(c, t + 3_000_000_000)
  c.service_params(p, t + 3_000_000_002)
  assert p.get(v.STATUS_PARAM) is None
  assert p.get(v.RESULT_PARAM)['result'] == 'CANCEL_RECOVERED'
