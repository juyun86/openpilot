from dataclasses import replace
from types import SimpleNamespace as NS
import json
import socket
import threading

import pytest

from openpilot.sunnypilot.navassist.lane_intent import (
  NavLaneIntentCoordinator, NavLanePlan, LaneTopologyInput, LaneVehicleInput, LaneIntentDirection as Direction,
  ObservedLaneChangeState as State,
)
from openpilot.sunnypilot.navassist.protocol import NavAssistProtocolError, NavAssistStore, AcceptedSnapshot, parse_snapshot
from openpilot.sunnypilot.navassist.publisher import build_nav_assist_message
from openpilot.sunnypilot.navassist.navassistd import pending_lane_announcement, current_lane_decision


def test_lane_decision_requires_fresh_valid_intent():
  intent = NS(valid=True, sessionId='session-a', reason='efficiencySafetyBlocked',
              signalRequested=False, direction='none', publishMonoTime=900_000_000)
  assert current_lane_decision(intent, healthy=True, now_ns=1_000_000_000) == {
    'sessionId': 'session-a', 'reason': 'efficiencySafetyBlocked',
    'signalRequested': False, 'direction': 'none',
  }
  assert current_lane_decision(intent, healthy=True, now_ns=1_200_000_001) == {}
  assert current_lane_decision(intent, healthy=False, now_ns=1_000_000_000) == {}
from openpilot.sunnypilot.navassist.udp_receiver import NavAssistUDPServer
from openpilot.sunnypilot.navassist.tests.test_protocol import payload, encode, SOURCE_WALL_MS


def waiting(side='left'):
  c = NavLaneIntentCoordinator(require_announcement=True)
  p = NavLanePlan(True, 'session-a', 1, 7, 3, (0 if side == 'left' else 2,),
                  heuristic=True, edge_direction=getattr(Direction, side))
  t = LaneTopologyInput(True, 3, 1, True, True, True, True)
  v = LaneVehicleInput(True, 25., left_blinker=side == 'left', right_blinker=side == 'right')
  for stamp in (1, 500_000_001, 1_000_000_001, 1_400_000_001):
    result = c.update(p, t, v, now_ns=stamp)
    assert not result.lane_change_ready
  assert len(c.announcement_id) == 32 and result.signal_requested
  return c, p, t, v


@pytest.mark.parametrize('side', ['left', 'right'])
def test_exact_speech_receipt_is_required_before_ready(side):
  c, p, t, v = waiting(side)
  for receipt in ('', '0'*32, 'old-request'):
    assert not c.update(p, t, v, now_ns=1_500_000_001, spoken_announcement_id=receipt).lane_change_ready
  assert c.update(p, t, v, now_ns=1_600_000_001, spoken_announcement_id=c.announcement_id).lane_change_ready


@pytest.mark.parametrize('state', [State.starting, State.finishing])
def test_existing_model_activity_cannot_bypass_pending_speech(state):
  c, p, t, v = waiting()
  result = c.update(p, t, replace(v, lane_change_state=state, lane_change_direction=Direction.left),
                    now_ns=1_500_000_001)
  assert result.reason == 'laneChangeAnnouncementFailed'
  assert not result.signal_requested and not result.lane_change_ready


@pytest.mark.parametrize('fault', ['timeout', 'blindspot', 'boundary', 'lamp', 'steering', 'neighbor', 'topology', 'route', 'brake', 'handoff'])
def test_speech_never_overrides_a_cancelled_or_unsafe_attempt(fault):
  c, p, t, v = waiting(); token = c.announcement_id
  now = 1_600_000_001
  if fault == 'timeout': now = 10_000_000_000
  if fault == 'blindspot': v = replace(v, left_blindspot=True)
  if fault == 'boundary': t = replace(t, left_crossing_allowed=False)
  if fault == 'lamp': v = replace(v, left_blinker=False)
  if fault == 'steering': v = replace(v, steering_pressed=True)
  if fault == 'topology': t = replace(t, valid_for_control=False)
  if fault == 'neighbor': t = replace(t, left_neighbor_exists=None)
  if fault == 'route': p = replace(p, valid=False)
  if fault == 'brake': v = replace(v, brake_pressed=True)
  result = c.update(p, t, v, now_ns=now, spoken_announcement_id=token, allow_new_lane_change=fault != 'handoff')
  assert result.reason == 'laneChangeAnnouncementFailed' and not result.lane_change_ready
  p = replace(p, valid=True); t = replace(t, valid_for_control=True, left_crossing_allowed=True, left_neighbor_exists=True)
  v = LaneVehicleInput(True, 25., left_blinker=True)
  for stamp in (11_000_000_000, 15_000_000_000):
    result = c.update(p, t, v, now_ns=stamp, spoken_announcement_id=token)
    assert not result.signal_requested and not result.lane_change_ready


def test_speech_receipt_round_trips_into_cereal_and_expires_with_snapshot():
  p = payload(); p['laneChangeSpeechCompletedId'] = 'a'*32
  accepted = AcceptedSnapshot(parse_snapshot(encode(p)), 1_000_000_000, 1_500_000_000)
  assert build_nav_assist_message(accepted, 1_100_000_000).navAssistStateSP.laneChangeSpeechCompletedId == 'a'*32
  assert build_nav_assist_message(accepted, 1_600_000_000).navAssistStateSP.laneChangeSpeechCompletedId == ''
  assert parse_snapshot(encode(payload())).lane_change_speech_completed_id == ''


@pytest.mark.parametrize('bad', [None, True, 1, [], {}, 'x'*32, 'a'*33])
def test_invalid_receipt_is_not_coerced(bad):
  p = payload(); p['laneChangeSpeechCompletedId'] = bad
  with pytest.raises(NavAssistProtocolError): parse_snapshot(encode(p))


@pytest.mark.parametrize('fault', [None, 'turn', 'ready', 'stale', 'future', 'health', 'starting', 'missing'])
def test_only_fresh_pending_automatic_lane_request_is_announced(fault):
  intent = NS(valid=True, signalRequested=True, spLaneChangeReady=False, targetLaneIndex=0,
              announcementId='a'*32, publishMonoTime=1_000_000_000, direction='left', sessionId='session-a')
  if fault == 'turn': intent.targetLaneIndex = -1
  if fault == 'ready': intent.spLaneChangeReady = True
  if fault == 'stale': intent.publishMonoTime -= 300_000_000
  if fault == 'future': intent.publishMonoTime += 1
  if fault == 'missing': intent.announcementId = ''
  result = pending_lane_announcement(intent, healthy=fault != 'health', now_ns=1_000_000_000,
                                     model_state=2 if fault == 'starting' else 1)
  assert bool(result) == (fault is None)


@pytest.mark.parametrize('same_session', [True, False])
def test_udp_delivers_speech_extension_before_legacy_ack_for_its_own_session(same_session):
  store = NavAssistStore(wall_clock_ms=lambda: SOURCE_WALL_MS)
  request = dict(id='b'*32, sessionId='session-a' if same_session else 'another-session', direction='right')
  server = NavAssistUDPServer(('127.0.0.1', 0), store, announcement_provider=lambda: request)
  thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
      client.settimeout(.5); client.sendto(encode(payload()), server.server_address)
      first = json.loads(client.recv(2048))
      if same_session:
        assert first['laneAnnouncement'] == request
        assert 'laneAnnouncement' not in json.loads(client.recv(2048))
      else:
        assert 'laneAnnouncement' not in first
  finally:
    server.shutdown(); thread.join(timeout=1); server.server_close()
