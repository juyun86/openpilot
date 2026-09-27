"""Navigation retries use the existing interval, with no session lifetime quota."""
import time
from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.selfdrive.car.tesla.card_adapter import TeslaCardAdapter, NAV_SIGNAL_RETRY_NS


def adapter_for(accept=False):
  class Controller:
    configured = True
    current = None
    calls = 0

    def status(self):
      return self.current

    def submit_request(self, test_id, direction, now_nanos, **kwargs):
      self.calls += 1
      if accept:
        self.current = {'test_id': test_id}
      return accept

    def request_cancel(self, test_id, now_nanos):
      self.current = None

  class SM:
    seen = alive = valid = {'navLaneIntentSP': True}
    recv_time = {'navLaneIntentSP': time.monotonic()}
    intent = SimpleNamespace(valid=True, signalRequested=True, direction='left', sessionId='route',
                             routeRevision=1, requestId=1, maneuverEventId=1)
    def __getitem__(self, key):
      return self.intent

  adapter = TeslaCardAdapter.__new__(TeslaCardAdapter)
  adapter.validation = Controller()
  adapter.sm = SM()
  adapter._active_nav_signal_test_id = None
  adapter._last_nav_signal_request = None
  adapter._nav_signal_retry_after_ns = 0
  return adapter


def test_busy_retries_remain_rate_limited_beyond_eight_attempts():
  a = adapter_for()
  for tick in range(20):
    stamp = 1 + tick * NAV_SIGNAL_RETRY_NS
    a._update_nav_turn_signal(stamp)
    assert a.validation.calls == tick + 1
    a._update_nav_turn_signal(stamp + NAV_SIGNAL_RETRY_NS - 1)
    assert a.validation.calls == tick + 1


def test_successful_navigation_is_not_exhausted_after_sixty_four_events():
  a = adapter_for(accept=True)
  for event in range(1, 101):
    a.sm.intent.maneuverEventId = a.sm.intent.requestId = event
    a._update_nav_turn_signal(event * 2_000_000_000)
    assert a.validation.calls == event
    a._update_nav_turn_signal(event * 2_000_000_000 + 1)
    assert a.validation.calls == event
    a.sm.intent.signalRequested = False
    a._update_nav_turn_signal(event * 2_000_000_000 + 2)
    a.sm.intent.signalRequested = True


def test_completed_attempt_preserves_existing_retry_interval():
  a = adapter_for(accept=True)
  a._update_nav_turn_signal(1)
  a.validation.current = None
  a._update_nav_turn_signal(2)
  a._update_nav_turn_signal(NAV_SIGNAL_RETRY_NS)
  assert a.validation.calls == 1
  a._update_nav_turn_signal(NAV_SIGNAL_RETRY_NS + 1)
  assert a.validation.calls == 2


@pytest.mark.parametrize('session,event,direction', [('', 1, 'left'), ('s', 0, 'left'), ('s', 1, 'none')])
def test_unknown_navigation_event_cannot_submit(session, event, direction):
  a = adapter_for()
  a.sm.intent.sessionId = session
  a.sm.intent.maneuverEventId = event
  a.sm.intent.direction = direction
  a._update_nav_turn_signal(1)
  assert a.validation.calls == 0
