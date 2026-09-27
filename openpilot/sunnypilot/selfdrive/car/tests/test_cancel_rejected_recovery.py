"""Rejected cancellation retries stay inside the existing CAN session budget."""
import json

import pytest

from openpilot.sunnypilot.selfdrive.car.tesla import validation_controller as v
from openpilot.sunnypilot.selfdrive.car.tests.test_turn_signal_recovery import cancelling, observe_off, template


@pytest.mark.parametrize('direction', ['left', 'right'])
def test_first_cancel_rejection_retries_fresh_template_then_confirms_off(direction):
  c, t, cancel = cancelling(direction, prior_on=True, ack=False)
  c.observe_frame(t + 7, 0x3E9, cancel.dat, 0xC1)
  assert c.status()['phase'] == 'cancelling'
  assert not c.submit_request('next', 'right', t + 8)
  observe_off(c, t + 10)
  assert c.status() is not None  # OFF alone cannot acknowledge Panda cancellation.
  assert c.take_can_sends(t + 20) == []  # No reuse of the rejected frame's template.
  c.observe_frame(t + 30, 0x3E9, template(6), 1)
  retry = c.take_can_sends(t + 30, cancel_only=True)[0]
  assert v.decode_body_controls(retry.dat)['turn_request'] == 3
  assert c.status()['cancel_attempts'] == 2
  c.observe_frame(t + 31, 0x3E9, retry.dat, 0x81)
  assert c.status() is not None  # Previous OFF must not be reused.
  observe_off(c, t + 32)
  assert c.drain_completed()[-1][0]['result'] == 'PASS'
  assert c.status() is None
  assert c.submit_request('next', 'right', t + 40)


def test_all_cancel_attempts_rejected_are_not_mistaken_for_missing_echo():
  c, t, cancel = cancelling(ack=False)
  for attempt in range(v.MAX_CANCEL_ATTEMPTS):
    stamp = t + 10 + attempt * 10
    if attempt:
      c.observe_frame(stamp, 0x3E9, template(5 + attempt), 1)
      cancel = c.take_can_sends(stamp, cancel_only=True)[0]
    c.observe_frame(stamp + 1, 0x3E9, cancel.dat, 0xC1)
  assert c.status()['phase'] == 'cancel_failed'
  assert c.status()['cancel_attempts'] == v.MAX_CANCEL_ATTEMPTS
  observe_off(c, t + 100)
  assert c.status()['result'] == 'PANDA_REJECTED'
  c.observe_frame(t + 110, 0x3E9, template(9), 1)
  assert c.take_can_sends(t + 110, cancel_only=True) == []
  assert not c.submit_request('next', 'left', t + 120)


def test_rejected_cancel_without_new_template_keeps_total_timeout():
  c, t, cancel = cancelling(ack=False)
  c.observe_frame(t + 7, 0x3E9, cancel.dat, 0xC1)
  assert c.take_can_sends(t + v.TEMPLATE_MAX_AGE_NS + 10) == []
  c.advance_time(t + v.CANCEL_TOTAL_TIMEOUT_NS + 10)
  assert c.status()['phase'] == 'cancel_failed'
  assert c.status()['result'] == 'CANCEL_TIMEOUT'
  assert c.status()['cancel_attempts'] == 1


def test_recovered_rejection_preserves_raw_evidence_with_normal_logging_disabled(tmp_path, monkeypatch):
  monkeypatch.setattr(v, 'TURN_SIGNAL_VALIDATION_LOGGING_ENABLED', False)
  c, t, cancel = cancelling(ack=False)
  c.observe_frame(t + 6, 0x3E9, template(6), 1)
  c.observe_frame(t + 7, 0x3E9, cancel.dat, 0xC1)
  c.observe_frame(t + 10, 0x3E9, template(7), 1)
  retry = c.take_can_sends(t + 10)[0]
  c.observe_frame(t + 11, 0x3E9, retry.dat, 0x81)
  observe_off(c, t + 12)
  result, records = c.drain_completed()[0]
  assert result['result'] == 'PASS'
  path = tmp_path / 'rejection.jsonl'
  v.persist_validation_records(records, str(path))
  saved = [json.loads(line) for line in path.read_text().splitlines()]
  rejected = [record for record in saved if record.get('can_direction') == 'rejected']
  assert len(rejected) == 1
  assert rejected[0]['data'] == cancel.dat.hex()
  assert any(record['event'] == 'baseline_frame' for record in saved)
  assert any(record['event'] == 'cancel_pending_template' and record['data'] == template(6).hex() for record in saved)


def test_normal_session_does_not_enable_raw_logging(tmp_path, monkeypatch):
  monkeypatch.setattr(v, 'TURN_SIGNAL_VALIDATION_LOGGING_ENABLED', False)
  c, t, _ = cancelling()
  observe_off(c, t + 10)
  result, records = c.drain_completed()[0]
  assert result['result'] == 'PASS'
  path = tmp_path / 'normal.jsonl'
  v.persist_validation_records(records, str(path))
  assert not path.exists()
