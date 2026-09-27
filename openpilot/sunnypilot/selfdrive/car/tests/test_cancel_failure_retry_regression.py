"""Memory-only real-module regression. Never publishes returned CAN bytes."""
import time
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.car.tesla.card_adapter import TeslaCardAdapter
from openpilot.sunnypilot.selfdrive.car.tesla.validation_controller import (
  TeslaTurnSignalRealtimeController, tesla_body_controls_checksum,
)


class SubMasterInput:
  def __init__(self):
    self.seen = self.alive = self.valid = {"navLaneIntentSP": True}
    self.recv_time = {"navLaneIntentSP": time.monotonic()}
    self.intent = SimpleNamespace(
      valid=True, signalRequested=True, direction="left", sessionId="same-session",
      routeRevision=1, requestId=1, maneuverEventId=42,
    )

  def __getitem__(self, key):
    assert key == "navLaneIntentSP"
    return self.intent


def test_same_navigation_event_does_not_reopen_after_cancel_not_sent():
  controller = TeslaTurnSignalRealtimeController(True)
  # Avoid constructing unrelated parsers/controllers; call the actual adapter
  # method on the exact fields it consumes. This remains a component regression.
  adapter = TeslaCardAdapter.__new__(TeslaCardAdapter)
  adapter.validation = controller
  adapter.sm = SubMasterInput()
  adapter._last_nav_signal_request = None
  adapter._active_nav_signal_test_id = None
  adapter._nav_signal_retry_after_ns = 0
  start = 10_000_000_000
  adapter._update_nav_turn_signal(start)
  original = bytearray([0xA5, 0x8C, 0x61, 0xB4, 0x5A, 0xC3, 0x47, 0])
  original[7] = tesla_body_controls_checksum(original)
  controller.observe_frame(start, 0x3E9, bytes(original), 1)
  sent = controller.take_can_sends(start)[0]
  controller.observe_frame(start + 1, 0x3E9, sent.dat, 0x81)
  controller.request_cancel(None, start + 2)
  # No fresh original template exists to emit a cancellation frame.
  controller.advance_time(start + 1_500_000_002)
  assert controller.drain_completed()[0][0]["result"] == "CANCEL_NOT_SENT"
  assert controller.status()['phase'] == 'cancel_failed'
  adapter.sm.recv_time["navLaneIntentSP"] = time.monotonic()
  adapter._update_nav_turn_signal(start + 2_000_000_000)
  assert controller.status()['phase'] == 'cancel_failed'
  assert not controller.submit_request('different-request', 'right', start + 3_000_000_000,
                                       hold_until_cancel=True)
  assert not controller.submit_request('validation-request', 'left', start + 4_000_000_000)
  assert controller.status()['phase'] == 'cancel_failed'


def test_cancel_before_any_send_does_not_latch_automatic_requests():
  controller = TeslaTurnSignalRealtimeController(True)
  assert controller.submit_request('first', 'left', 1_000_000_000, hold_until_cancel=True)
  controller.request_cancel('first', 1_100_000_000)
  assert controller.drain_completed()[0][0]['result'] == 'CANCELLED_BEFORE_SEND'
  assert controller.submit_request('next', 'right', 1_200_000_000, hold_until_cancel=True)
