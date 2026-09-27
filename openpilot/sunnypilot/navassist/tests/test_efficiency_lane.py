from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from openpilot.sunnypilot.navassist.efficiency_lane import EfficiencyLaneSelector, lane_leads, side_lead_unsafe
from openpilot.sunnypilot.navassist.lane_intent import (
  LaneIntentDirection as Direction, LaneTopologyInput, LaneVehicleInput,
  NavLanePlan, NavLaneIntentCoordinator, ObservedLaneChangeState as State,
)


def point(track, d, y, v):
  return NS(trackId=track, dRel=d, yRel=y, vRel=v, deprecated=NS(measured=True))


def inputs():
  return dict(
    route_plan=NavLanePlan(True, 'route', 1, 2, 3, ()),
    nav=NS(sessionId='route', routeRevision=1, valid=True, stale=False, routeActive=True,
           routeMatched=True, mode='realtime', roadClass=0),
    topology=LaneTopologyInput(True, 3, 1, True, True, True, True),
    vehicle=LaneVehicleInput(True, 25.),
    radar=NS(points=[point(1, 40., 0., -5.), point(2, 70., 3.5, 0.), point(3, 75., -3.5, 0.)]),
    model=NS(laneLineProbs=[.99]*4, laneLines=[NS(x=[0., 100.], y=[y, y]) for y in (-5.25, -1.75, 1.75, 5.25)]),
    oem=dict(permissionValid=True, leftAllowed=True, rightAllowed=True, leftSafetyBlocked=False, rightSafetyBlocked=False),
    healthy=True, enabled=True, cruise_mps=30., route_reserved=False,
  )


def select(selector, data, ns):
  return selector.select(**data, now_ns=ns)


def test_curved_corridors_and_sign_convention():
  d = inputs()
  for line in d['model'].laneLines:
    line.y[1] += 10.
  d['radar'].points = [point(1, 40., -4., -5.), point(2, 70., -3.5, 0.), point(3, 75., -11., 0.)]
  assert [p.trackId for p in lane_leads(d['radar'], d['model'])] == [2, 1, 3]


@pytest.mark.parametrize('fault', [None, 'unordered', 'short', 'nan', 'outsideRange'])
def test_lane_leads_with_real_capnp_readers(fault):
  from openpilot.cereal import messaging

  data = inputs()
  model = messaging.new_message('modelV2').modelV2
  model.laneLineProbs = data['model'].laneLineProbs
  model.init('laneLines', 4)
  for target, source in zip(model.laneLines, data['model'].laneLines):
    target.x, target.y = source.x, source.y
  if fault == 'unordered': model.laneLines[0].x = [100., 0.]
  if fault == 'short': model.laneLines[0].y = []
  if fault == 'nan': model.laneLines[0].x = [0., float('nan')]
  if fault == 'outsideRange': model.laneLines[0].x = [0., 60.]
  radar = messaging.new_message('radarTracks').radarTracks
  radar.init('points', 3)
  for target, source in zip(radar.points, data['radar'].points):
    target.trackId, target.dRel, target.yRel, target.vRel = source.trackId, source.dRel, source.yRel, source.vRel
    target.deprecated.measured = True
  result = lane_leads(radar.as_reader(), model.as_reader())
  if fault is None:
    assert [p.trackId for p in result] == [2, 1, 3]
  else:
    assert result is None


@pytest.mark.parametrize('absent_side', ['left', 'right'])
@pytest.mark.parametrize('fault', [None, 'egoBoundary', 'targetBoundary', 'missingLead'])
def test_absent_neighbor_does_not_veto_existing_side(absent_side, fault):
  from openpilot.cereal import messaging

  data = inputs()
  left = absent_side != 'left'
  right = absent_side != 'right'
  outer = 0 if absent_side == 'left' else 3
  target_outer = 3 if absent_side == 'left' else 0
  model = messaging.new_message('modelV2').modelV2
  model.laneLineProbs = [.99] * 4
  model.init('laneLines', 4)
  for target, source in zip(model.laneLines, data['model'].laneLines):
    target.x, target.y = source.x, source.y
  model.laneLineProbs[outer] = .001
  model.laneLines[outer].x = []
  model.laneLines[outer].y = []
  if fault == 'egoBoundary': model.laneLines[1].x = []
  if fault == 'targetBoundary': model.laneLines[target_outer].x = []
  if fault == 'missingLead': data['radar'].points = data['radar'].points[:1]
  data['model'] = model.as_reader()
  data['topology'] = replace(data['topology'], visible_lane_count=2,
                             ego_lane_index=0 if right else 1,
                             left_neighbor_exists=left, right_neighbor_exists=right)
  result = lane_leads(data['radar'], data['model'], left_neighbor=left, right_neighbor=right)
  selector = EfficiencyLaneSelector()
  select(selector, data, 1_000_000_000)
  plan, selected = select(selector, data, 2_000_000_000)
  if fault is None or fault == 'missingLead':
    expected = ([None, 1, 3] if right else [2, 1, None]) if fault is None else [None, 1, None]
    assert [p.trackId if p else None for p in result] == expected
    assert selected
    assert plan.edge_direction == (Direction.right if right else Direction.left)
  else:
    assert not selected
    assert result is None
    assert selector.reason == 'efficiencyLaneGeometryUnknown'


@pytest.mark.parametrize('fault', ['nan', 'unordered', 'short', 'boundary', 'predicted', 'outsideRange'])
def test_unknown_geometry_or_target_does_not_become_clear_space(fault):
  d = inputs()
  if fault == 'nan': d['radar'].points[0].vRel = float('nan')
  if fault == 'unordered': d['model'].laneLines[0].x = [100., 0.]
  if fault == 'short': d['model'].laneLines[0].y = []
  if fault == 'boundary': d['radar'].points[1].yRel = 1.75
  if fault == 'predicted': d['radar'].points[1].deprecated.measured = False
  if fault == 'outsideRange': d['model'].laneLines[0].x[-1] = 60.
  assert lane_leads(d['radar'], d['model']) is None


def test_radar_association_does_not_gate_on_invented_width_or_boundary_margin():
  d = inputs()
  for line, y in zip(d['model'].laneLines, (-6., -2., 2., 6.)):
    line.y = [y, y]  # 4 m corridors, with a track near the inner boundary.
  d['radar'].points[1].yRel = 2.05
  assert [p.trackId if p else None for p in lane_leads(d['radar'], d['model'])] == [2, 1, 3]
  for line, y in zip(d['model'].laneLines, (-7., -2., 2., 7.)):
    line.y = [y, y]  # Outer corridors are 5 m; not a crossing veto.
  assert [p.trackId if p else None for p in lane_leads(d['radar'], d['model'])] == [2, 1, 3]


def test_line_probability_does_not_gate_valid_geometry():
  d = inputs()
  d['model'].laneLineProbs = [.729] * 4
  assert [p.trackId for p in lane_leads(d['radar'], d['model'])] == [2, 1, 3]
  d['model'].laneLineProbs[3] = .01
  assert [p.trackId for p in lane_leads(d['radar'], d['model'])] == [2, 1, 3]


@pytest.mark.parametrize('fault', ['healthy', 'enabled', 'route', 'blind', 'sna', 'neighbor', 'boundary',
                                   'driver', 'brake', 'lowSpeed', 'road', 'slowSide', 'closeSide', 'manual'])
def test_no_automatic_request_without_positive_conditions(fault):
  d = inputs()
  if fault in ('healthy', 'enabled'): d[fault] = False
  if fault == 'route': d['route_reserved'] = True
  if fault == 'blind': d['vehicle'] = replace(d['vehicle'], left_blindspot=True, right_blindspot=True)
  if fault == 'sna': d['oem'].update(leftSafetyBlocked=True, rightSafetyBlocked=True)
  if fault == 'neighbor': d['topology'] = replace(d['topology'], left_neighbor_exists=None, right_neighbor_exists=False)
  if fault == 'boundary': d['topology'] = replace(d['topology'], left_crossing_allowed=False, right_crossing_allowed=False)
  if fault in ('driver', 'brake'):
    field = {'driver': 'steering_pressed', 'brake': 'brake_pressed'}[fault]
    d['vehicle'] = replace(d['vehicle'], **{field: True})
  if fault == 'lowSpeed': d['vehicle'] = replace(d['vehicle'], speed_mps=10.)
  if fault == 'road': d['nav'].roadClass = 1
  if fault == 'slowSide':
    for p in d['radar'].points[1:]: p.vRel = -6.
  if fault == 'closeSide':
    for p in d['radar'].points[1:]: p.dRel = 5.
  if fault == 'manual': d['vehicle'] = replace(d['vehicle'], left_blinker=True)
  s = EfficiencyLaneSelector()
  for stamp in (0, 800_000_000, 2_000_000_000):
    assert not select(s, d, stamp)[1]


def test_no_adjacent_radar_target_allows_overtake_when_current_lead_and_gates_are_valid():
  d = inputs(); s = EfficiencyLaneSelector()
  d['radar'].points = d['radar'].points[:1]
  assert not select(s, d, 0)[1]
  plan, selected = select(s, d, 800_000_000)
  assert selected and plan.edge_direction == Direction.left
  assert select(s, d, 900_000_000) == (plan, True)
  d['vehicle'] = replace(d['vehicle'], left_blindspot=True, right_blindspot=True)
  assert not select(s, d, 950_000_000)[1]
  assert s.reason == 'efficiencySafetyBlocked'


def test_adjacent_forward_gap_uses_cp_side_target_rule():
  assert not side_lead_unsafe(None, 25.)
  assert not side_lead_unsafe(point(2, 49., 3.5, 0.), 25.)
  assert side_lead_unsafe(point(2, 5., 3.5, 0.), 25.)
  assert side_lead_unsafe(point(2, 25., 3.5, -10.), 25.)
  assert side_lead_unsafe(point(2, 10., 3.5, -4.), 25.)


def test_reference_side_gap_can_request_below_old_speed_times_two_floor():
  d = inputs(); s = EfficiencyLaneSelector()
  d['radar'].points[1].dRel = 49.  # Old C3 floor was 50 m at 25 m/s.
  select(s, d, 0)
  plan, selected = select(s, d, 800_000_000)
  assert selected and plan.edge_direction == Direction.left


def test_accelerator_does_not_block_navigation_overtake_request():
  d = inputs(); s = EfficiencyLaneSelector()
  d['vehicle'] = replace(d['vehicle'], gas_pressed=True)
  d['radar'].points = d['radar'].points[:1]
  select(s, d, 0)
  assert select(s, d, 800_000_000)[1]


@pytest.mark.parametrize('absent_side', ['left', 'right'])
def test_empty_existing_neighbor_is_selectable_when_opposite_neighbor_does_not_exist(absent_side):
  d = inputs(); s = EfficiencyLaneSelector()
  d['radar'].points = d['radar'].points[:1]
  d['topology'] = replace(d['topology'], left_neighbor_exists=absent_side != 'left',
                          right_neighbor_exists=absent_side != 'right')
  d['model'].laneLines[0 if absent_side == 'left' else 3].x = []
  select(s, d, 0)
  plan, selected = select(s, d, 800_000_000)
  assert selected
  assert plan.edge_direction == (Direction.right if absent_side == 'left' else Direction.left)


def test_no_adjacent_target_never_replaces_missing_current_lead_or_invalid_geometry():
  d = inputs(); s = EfficiencyLaneSelector()
  d['radar'].points = []
  for stamp in (0, 800_000_000):
    assert not select(s, d, stamp)[1]
  d['radar'].points = [point(1, 40., 0., -5.)]
  d['model'].laneLines[1].x = []  # An ego boundary is required on both sides.
  assert not select(s, d, 1_600_000_000)[1]
  assert s.reason == 'efficiencyLaneGeometryUnknown'


@pytest.mark.parametrize('fault', ['outerMissing', 'oppositeBoundary', 'oppositePredicted', 'roadsideBeyondHorizon'])
def test_unusable_opposite_side_does_not_veto_observed_left_lane(fault):
  d = inputs(); s = EfficiencyLaneSelector()
  d['topology'] = replace(d['topology'], right_crossing_allowed=False)
  if fault == 'outerMissing':
    d['model'].laneLines[3].x = []
  elif fault == 'oppositeBoundary':
    d['radar'].points[2].yRel = -5.25
  elif fault == 'oppositePredicted':
    d['radar'].points[2].deprecated.measured = False
  else:
    for line in d['model'].laneLines:
      line.x[-1] = 80.
    d['radar'].points.append(point(7, 90., 30., 0.))
  assert not select(s, d, 0)[1]
  plan, selected = select(s, d, 800_000_000)
  assert selected and plan.edge_direction == Direction.left


def test_every_observed_side_target_must_pass_reference_gap_check():
  d = inputs(); s = EfficiencyLaneSelector()
  d['topology'] = replace(d['topology'], right_crossing_allowed=False)
  d['radar'].points[1] = point(2, 8., 3.5, 3.)
  faster_target = point(4, 20., 3.5, -10.)
  d['radar'].points.append(faster_target)
  assert not select(s, d, 0)[1]
  assert not select(s, d, 800_000_000)[1]
  assert s.reason == 'efficiencyNoConfirmedBenefit'
  faster_target.vRel = 0.
  assert not select(s, d, 900_000_000)[1]
  plan, selected = select(s, d, 1_700_000_000)
  assert selected and plan.edge_direction == Direction.left


def test_adjacent_target_disappears_after_selection_without_aborting_clear_side():
  d = inputs(); s = EfficiencyLaneSelector()
  select(s, d, 0)
  active, selected = select(s, d, 800_000_000)
  assert selected and active.edge_direction == Direction.left
  d['radar'].points.pop(1)
  assert select(s, d, 900_000_000) == (active, True)


def test_persistent_benefit_and_left_priority_with_right_fallback():
  d = inputs(); s = EfficiencyLaneSelector()
  assert not select(s, d, 0)[1]
  assert not select(s, d, 799_000_000)[1]
  assert select(s, d, 800_000_000)[0].edge_direction == Direction.left
  d['oem']['leftAllowed'] = False
  d['topology'] = replace(d['topology'], left_crossing_allowed=False)
  assert not select(s, d, 900_000_000)[1]
  assert not select(s, d, 5_000_000_000)[1]
  assert select(s, d, 5_800_000_000)[0].edge_direction == Direction.right


def test_visual_crossing_can_select_when_399_does_not_allow():
  d = inputs(); s = EfficiencyLaneSelector()
  d['oem']['leftAllowed'] = False
  select(s, d, 0)
  plan, selected = select(s, d, 800_000_000)
  assert selected and plan.edge_direction == Direction.left


def test_positive_gain_below_old_fixed_floor_can_start_request():
  d = inputs(); s = EfficiencyLaneSelector()
  d['radar'].points[1].vRel = -2.3
  assert not select(s, d, 0)[1]
  plan, selected = select(s, d, 800_000_000)
  assert selected and plan.edge_direction == Direction.left


@pytest.mark.parametrize('target', [0, 1])
def test_target_identity_change_keeps_continuous_safe_benefit(target):
  d = inputs(); s = EfficiencyLaneSelector()
  select(s, d, 0)
  d['radar'].points[target].trackId = 9
  assert select(s, d, 800_000_000)[1]


@pytest.mark.parametrize('side', ['left', 'right'])
def test_selected_side_ignores_unrelated_outer_boundary(side):
  d = inputs(); s = EfficiencyLaneSelector()
  if side == 'right':
    d['oem']['leftAllowed'] = False
    d['topology'] = replace(d['topology'], left_crossing_allowed=False)
  select(s, d, 0); active, selected = select(s, d, 800_000_000)
  assert selected
  outer = 3 if side == 'left' else 0
  d['model'].laneLineProbs[outer] = .01
  d['model'].laneLines[outer].x = []
  assert select(s, d, 900_000_000) == (active, True)
  d['model'].laneLineProbs[0 if side == 'left' else 3] = .729
  d['model'].laneLines[0 if side == 'left' else 3].x = []
  assert not select(s, d, 950_000_000)[1]
  assert s.reason == 'efficiencyLaneGeometryUnknown'


def test_small_positive_benefit_keeps_event_until_gain_disappears():
  d = inputs(); s = EfficiencyLaneSelector()
  select(s, d, 0); active, _ = select(s, d, 800_000_000)
  d['radar'].points[1].vRel = -2.3  # Positive gain smaller than the removed 10 km/h floor.
  assert select(s, d, 900_000_000) == (active, True)
  assert select(s, d, 1_600_000_000) == (active, True)
  d['radar'].points[1].vRel = 0.
  assert select(s, d, 1_650_000_000) == (active, True)
  d['radar'].points[1].vRel = -2.3
  assert select(s, d, 1_700_000_000) == (active, True)
  assert select(s, d, 2_500_000_000) == (active, True)
  d['radar'].points[1].vRel = -5.
  assert not select(s, d, 2_550_000_000)[1]
  assert s.reason == 'efficiencyBenefitLost' and s.sequence == 1


@pytest.mark.parametrize('fault,reason', [
  ('permission', 'efficiencyCrossingLost'), ('blind', 'efficiencySafetyBlocked'),
  ('solid', 'efficiencyCrossingLost'), ('neighbor', 'efficiencyNeighborLost'),
  ('gap', 'efficiencyGapLost'),
  ('ttc', 'efficiencyGapLost'), ('noGain', 'efficiencyBenefitLost'),
])
def test_benefit_hold_never_delays_safety_or_evidence_loss(fault, reason):
  d = inputs(); s = EfficiencyLaneSelector()
  select(s, d, 0); select(s, d, 800_000_000)
  d['radar'].points[1].vRel = -2.3
  assert select(s, d, 900_000_000)[1]
  if fault == 'permission':
    d['oem']['leftAllowed'] = False
    d['topology'] = replace(d['topology'], left_crossing_allowed=False)
  if fault == 'blind': d['vehicle'] = replace(d['vehicle'], left_blindspot=True)
  if fault == 'solid': d['topology'] = replace(d['topology'], left_crossing_allowed=False)
  if fault == 'neighbor': d['topology'] = replace(d['topology'], left_neighbor_exists=False)
  if fault == 'gap': d['radar'].points[1].dRel = 5.
  if fault == 'ttc':
    d['radar'].points[1].dRel = 25.
    d['radar'].points[1].vRel = -10.
  if fault == 'noGain': d['radar'].points[1].vRel = -5.
  assert not select(s, d, 950_000_000)[1]
  assert s.reason == reason


@pytest.mark.parametrize('side', ['left', 'right'])
def test_benefit_dip_preserves_speech_owner_and_never_bypasses_receipt(side):
  d = inputs(); s = EfficiencyLaneSelector(); executor = NavLaneIntentCoordinator(require_announcement=True)
  if side == 'right':
    d['oem']['leftAllowed'] = False
    d['topology'] = replace(d['topology'], left_crossing_allowed=False)
  def tick(stamp, receipt=''):
    plan, selected = select(s, d, stamp)
    intent = executor.update(plan, d['topology'], d['vehicle'], now_ns=stamp,
                             spoken_announcement_id=receipt)
    if selected: s.observe(intent, stamp)
    return intent
  for t in (0, 800_000_000, 1_300_000_000, 1_800_000_000): tick(t)
  d['vehicle'] = replace(d['vehicle'], **{side + '_blinker': True}, lane_change_state=State.pre,
                         lane_change_direction=getattr(Direction, side))
  tick(1_900_000_000); before = tick(2_300_000_000)
  token = executor.announcement_id
  assert token and before.signal_requested and not before.lane_change_ready
  d['radar'].points[1 if side == 'left' else 2].vRel = -2.3
  during = tick(2_400_000_000, 'wrong-receipt')
  assert during.signal_requested and not during.lane_change_ready
  assert executor.announcement_id == token and during.request_id == before.request_id
  d['radar'].points[1 if side == 'left' else 2].vRel = 0.
  recovered = tick(2_600_000_000, token)
  assert recovered.lane_change_ready and recovered.request_id == before.request_id


def test_navigation_preempts_preparation_but_not_an_executing_change():
  d = inputs(); s = EfficiencyLaneSelector()
  select(s, d, 0); select(s, d, 800_000_000)
  d['route_reserved'] = True
  assert not select(s, d, 900_000_000)[1]
  d['route_reserved'] = False
  select(s, d, 5_000_000_000); active, _ = select(s, d, 5_800_000_000)
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.starting, lane_change_direction=Direction.left)
  d['route_reserved'] = True
  assert select(s, d, 6_000_000_000)[0] == active
  d['nav'].routeRevision = 2
  assert not select(s, d, 6_100_000_000)[1]


def test_real_coordinator_unconfirmed_attempt_waits_for_exit_then_retries():
  d = inputs(); s = EfficiencyLaneSelector(); executor = NavLaneIntentCoordinator()
  def tick(stamp):
    plan, selected = select(s, d, stamp)
    intent = executor.update(plan, d['topology'], d['vehicle'], now_ns=stamp)
    if selected: s.observe(intent, stamp)
    return intent
  for t in (0, 800_000_000, 1_300_000_000, 1_800_000_000): intent = tick(t)
  assert intent.signal_requested and not intent.lane_change_ready
  d['vehicle'] = replace(d['vehicle'], left_blinker=True)
  tick(1_900_000_000); intent = tick(2_300_000_000)
  assert intent.lane_change_ready
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.starting, lane_change_direction=Direction.left)
  assert tick(2_400_000_000).lane_change_ready
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.finishing)
  tick(2_600_000_000)
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.off, lane_change_direction=Direction.none)
  assert tick(3_000_000_000).reason == 'laneChangeCompletionUnconfirmed'
  tick(4_000_000_000)
  assert s.active is None and s.reason == 'efficiencyCompletionUnconfirmed'
  d['vehicle'] = replace(d['vehicle'], left_blinker=False)
  assert not select(s, d, 5_000_000_000)[1]
  assert not tick(7_000_000_000).signal_requested
  tick(7_800_000_000)
  tick(8_300_000_000)
  assert tick(8_800_000_000).signal_requested
  assert s.sequence == 2 and s.unconfirmed_session is None


@pytest.mark.parametrize('fault', [None, 'stale', 'future', 'skew', 'canError', 'radarFault',
                                   'radarUnavailableTemporary', 'wrongConfig', 'backend', 'unseen', 'invalid'])
def test_real_message_radar_health_and_timestamp_gates(fault):
  from openpilot.cereal import messaging, custom
  from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
  from openpilot.sunnypilot.navassist.nav_lane_intentd import efficiency_radar_healthy
  class SM(dict): pass
  sm = SM(radarTracks=messaging.new_message('radarTracks').radarTracks, carParamsSP=custom.CarParamsSP.new_message())
  sm['carParamsSP'].flags = int(TeslaFlagsSP.ARS408_RADAR)
  services = ('radarTracks', 'modelV2', 'carParamsSP')
  sm.seen = dict.fromkeys(services, True)
  sm.alive = dict.fromkeys(services, True)
  sm.valid = dict.fromkeys(services, True)
  sm.logMonoTime = dict.fromkeys(services, 1_000_000_000)
  if fault == 'stale': sm.logMonoTime['radarTracks'] -= 300_000_000
  if fault == 'future': sm.logMonoTime['radarTracks'] += 1
  if fault == 'skew': sm.logMonoTime['radarTracks'] -= 150_000_000
  if fault in ('canError', 'radarFault', 'radarUnavailableTemporary', 'wrongConfig'):
    setattr(sm['radarTracks'].errors, fault, True)
  if fault == 'backend': sm['carParamsSP'].flags = 0
  if fault == 'unseen': sm.seen['carParamsSP'] = False
  if fault == 'invalid': sm.valid['radarTracks'] = False
  assert efficiency_radar_healthy(sm, 1_000_000_000) == (fault is None)


def test_explicit_cancel_waits_for_cooldown_then_can_request_again():
  d = inputs(); s = EfficiencyLaneSelector(); executor = NavLaneIntentCoordinator()
  def tick(stamp):
    plan, selected = select(s, d, stamp)
    intent = executor.update(plan, d['topology'], d['vehicle'], now_ns=stamp)
    if selected:
      s.observe(intent, stamp)
    return intent
  for t in (0, 800_000_000, 1_300_000_000, 1_800_000_000):
    tick(t)
  d['vehicle'] = replace(d['vehicle'], left_blinker=True)
  tick(1_900_000_000)
  assert tick(2_300_000_000).lane_change_ready
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.starting, lane_change_direction=Direction.left)
  tick(2_400_000_000)
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.finishing, lane_change_direction=Direction.none)
  assert tick(2_600_000_000).reason == 'laneChangeCancelled'
  d['vehicle'] = replace(d['vehicle'], left_blinker=False, lane_change_state=State.off)
  assert not tick(3_000_000_000).signal_requested
  assert s.reason == 'efficiencyCooldown' and s.sequence == 1
  tick(7_400_000_000)
  assert s.reason == 'efficiencyBenefitStabilizing'
  tick(8_200_000_000)
  assert s.sequence == 2 and s.unconfirmed_session is None


def test_topology_gap_during_change_allows_explicit_cancel_and_retry():
  d = inputs(); s = EfficiencyLaneSelector(); executor = NavLaneIntentCoordinator()
  def tick(stamp):
    plan, selected = select(s, d, stamp)
    intent = executor.update(plan, d['topology'], d['vehicle'], now_ns=stamp)
    if selected:
      s.observe(intent, stamp)
    return intent
  for t in (0, 800_000_000, 1_300_000_000, 1_800_000_000):
    tick(t)
  d['vehicle'] = replace(d['vehicle'], left_blinker=True)
  tick(1_900_000_000)
  assert tick(2_300_000_000).lane_change_ready
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.starting, lane_change_direction=Direction.left)
  tick(2_400_000_000)
  d['topology'] = LaneTopologyInput(False, 0, -1, None, None, False, False)
  assert tick(2_500_000_000).reason == 'heuristicTopologyTransition'
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.finishing, lane_change_direction=Direction.none)
  assert tick(2_600_000_000).reason == 'laneChangeCancelled'
  assert s.unconfirmed_session is None
  d['topology'] = inputs()['topology']
  d['vehicle'] = inputs()['vehicle']
  tick(7_400_000_000)
  tick(8_200_000_000)
  assert s.sequence == 2


def test_long_topology_gap_keeps_change_active_until_existing_timeout():
  d = inputs(); s = EfficiencyLaneSelector(); executor = NavLaneIntentCoordinator()
  def tick(stamp):
    plan, selected = select(s, d, stamp)
    intent = executor.update(plan, d['topology'], d['vehicle'], now_ns=stamp)
    if selected:
      s.observe(intent, stamp)
    return intent
  for t in (0, 800_000_000, 1_300_000_000, 1_800_000_000):
    tick(t)
  d['vehicle'] = replace(d['vehicle'], left_blinker=True)
  tick(1_900_000_000)
  tick(2_300_000_000)
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.starting, lane_change_direction=Direction.left)
  tick(2_400_000_000)
  d['topology'] = LaneTopologyInput(False, 0, -1, None, None, False, False)
  tick(2_500_000_000)
  assert tick(5_100_000_000).reason == 'heuristicTopologyTransition'
  assert tick(12_500_000_001).reason == 'laneChangeTimeout'
  assert s.unconfirmed_session == 'route'


@pytest.mark.parametrize('fault', ['brake', 'steering', 'health', 'route'])
def test_efficiency_execution_loss_recovers_same_session_after_exit(fault):
  d = inputs(); s = EfficiencyLaneSelector()
  select(s, d, 0); select(s, d, 800_000_000)
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.starting, lane_change_direction=Direction.left)
  if fault in ('brake', 'steering'):
    d['vehicle'] = replace(d['vehicle'], **{fault + '_pressed': True})
  elif fault == 'health':
    d['healthy'] = False
  else:
    d['nav'].routeRevision += 1
  assert not select(s, d, 900_000_000)[1]
  assert s.unconfirmed_session == 'route'
  d['vehicle'] = inputs()['vehicle']; d['healthy'] = True
  assert not select(s, d, 5_000_000_000)[1]
  assert select(s, d, 5_800_000_000)[1]
  assert s.unconfirmed_session is None


@pytest.mark.parametrize('mode,stable_ms', [(1, 400), (2, 800), (3, 1200)])
def test_configured_preset_reaches_real_selector(mode, stable_ms):
  from openpilot.sunnypilot.navassist.settings import NavAssistSettings, OVERTAKE_PRESETS
  d = inputs(); s = EfficiencyLaneSelector()
  d['settings'] = replace(NavAssistSettings(), overtake_mode=mode, **OVERTAKE_PRESETS[mode])
  # Both leads satisfy even conservative entry; isolate configured dwell time.
  d['radar'].points[0].dRel = 35.
  select(s, d, 0)
  assert not select(s, d, stable_ms * 1_000_000 - 1)[1]
  assert select(s, d, stable_ms * 1_000_000)[1]
  s.cancel(stable_ms * 1_000_000 + 1, 'test')
  assert s.cooldown_until == stable_ms * 1_000_000 + 1 + d['settings'].overtake_failed_cooldown_s * 1_000_000_000


def test_configured_right_toggle_and_benefit_preference():
  from openpilot.sunnypilot.navassist.settings import NavAssistSettings
  d = inputs(); s = EfficiencyLaneSelector()
  d['radar'].points[1].vRel = -2.
  d['settings'] = replace(NavAssistSettings(), overtake_prefer_left=False)
  select(s, d, 0); plan, selected = select(s, d, 800_000_000)
  assert selected and plan.edge_direction == Direction.right
  d['settings'] = replace(d['settings'], overtake_allow_right=False)
  assert not select(s, d, 900_000_000)[1]
  assert s.reason == 'efficiencyRightDisabled'
  select(s, d, 5_000_000_000)
  assert select(s, d, 5_800_000_000)[0].edge_direction == Direction.left


@pytest.mark.parametrize('field,value', [('overtake_min_distance_m', 40), ('overtake_max_distance_m', 40),
  ('overtake_lead_min_kph', 60), ('overtake_closing_kph', 15),
  ('overtake_time_gap_tenths', 10), ('overtake_cruise_percent', 70)])
def test_custom_threshold_is_consumed_not_just_displayed(field, value):
  from openpilot.sunnypilot.navassist.settings import NavAssistSettings
  d = inputs()
  if field == 'overtake_min_distance_m': d['radar'].points[0].dRel = 39.
  if field == 'overtake_lead_min_kph': d['vehicle'] = replace(d['vehicle'], speed_mps=20.)
  if field == 'overtake_max_distance_m': d['radar'].points[0].dRel = 41.
  if field in ('overtake_closing_kph', 'overtake_time_gap_tenths', 'overtake_cruise_percent'):
    d['radar'].points[0].dRel = 80.
    d['radar'].points[1].dRel = 90.
    d['radar'].points[2].dRel = 95.
    d['radar'].points[0].vRel = -1.
    d['radar'].points[1].vRel = 4.
    d['radar'].points[2].vRel = 4.
    d['cruise_mps'] = 27.
    if field == 'overtake_time_gap_tenths':
      d['radar'].points[0].vRel = -0.5; d['radar'].points[0].dRel = 60.
      d['cruise_mps'] = 27.5
    if field == 'overtake_cruise_percent':
      d['radar'].points[0].vRel = 0.; d['cruise_mps'] = 30.
  before = EfficiencyLaneSelector()
  select(before, d, 0)
  assert select(before, d, 800_000_000)[1]
  d['settings'] = replace(NavAssistSettings(), **{field: value})
  after = EfficiencyLaneSelector()
  select(after, d, 0)
  assert not select(after, d, 800_000_000)[1]


def test_success_cooldown_uses_configured_value():
  from openpilot.sunnypilot.navassist.settings import NavAssistSettings
  from openpilot.sunnypilot.navassist.lane_intent import NavLaneIntent
  d = inputs(); s = EfficiencyLaneSelector()
  d['settings'] = replace(NavAssistSettings(), overtake_success_cooldown_s=25)
  select(s, d, 0); select(s, d, 800_000_000)
  s.observe(NavLaneIntent(reason='laneChangeComplete'), 1_000_000_000)
  assert s.cooldown_until == 26_000_000_000


@pytest.mark.parametrize('fault', ['modelStarting', 'modelFinishing', 'modelPre', 'leftLamp', 'rightLamp',
                                 'brake', 'steering', 'health', 'executionHealth', 'navStale'])
def test_interrupted_attempt_cannot_retry_before_current_inputs_are_ready(fault):
  d = inputs()
  s = EfficiencyLaneSelector()
  select(s, d, 0)
  select(s, d, 800_000_000)
  d['vehicle'] = replace(d['vehicle'], lane_change_state=State.starting, lane_change_direction=Direction.left)
  select(s, d, 900_000_000)
  d['execution_healthy'] = False
  assert not select(s, d, 1_000_000_000)[1]
  d['execution_healthy'] = True
  d['vehicle'] = inputs()['vehicle']
  if fault.startswith('model'):
    state = {'modelStarting': State.starting, 'modelFinishing': State.finishing, 'modelPre': State.pre}[fault]
    d['vehicle'] = replace(d['vehicle'], lane_change_state=state, lane_change_direction=Direction.left)
  elif fault.endswith('Lamp'):
    d['vehicle'] = replace(d['vehicle'], **{fault[:-4].lower() + '_blinker': True})
  elif fault in ('brake', 'steering'):
    d['vehicle'] = replace(d['vehicle'], **{fault + '_pressed': True})
  elif fault == 'health':
    d['healthy'] = False
  elif fault == 'executionHealth':
    d['execution_healthy'] = False
  else:
    d['nav'].stale = True
  for stamp in (10_000_000_000, 60_000_000_000):
    d['nav'].sessionId = 'new-session'  # Session churn is not recovery evidence.
    assert not select(s, d, stamp)[1]
  d = inputs()
  assert not select(s, d, 61_000_000_000)[1]
  assert select(s, d, 61_800_000_000)[1]


def test_track_changes_do_not_bypass_a_new_unsafe_target_or_lost_benefit():
  d = inputs()
  s = EfficiencyLaneSelector()
  d['topology'] = replace(d['topology'], right_crossing_allowed=False)
  select(s, d, 0)
  d['radar'].points[1].trackId += 1
  d['radar'].points[1].dRel = 3.
  assert not select(s, d, 800_000_000)[1]
  d['radar'].points[1].dRel = 60.
  assert not select(s, d, 900_000_000)[1]
  assert select(s, d, 1_700_000_000)[1]
