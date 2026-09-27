import gzip
import io
import json
import zipfile

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.debug.local_diagnostics import LOCAL_DIAGNOSTIC_SERVICES
from openpilot.sunnypilot.navassist.diagnostics import NavigationLogWriter, NAVIGATION_ONLY_SERVICES, scan_logs, select_logs, stream_logs
from openpilot.sunnypilot.navassist.diagnosticsd import SERVICES, collect_sample


def test_navigation_is_excluded_from_generic_diagnostics():
  assert not NAVIGATION_ONLY_SERVICES.intersection(LOCAL_DIAGNOSTIC_SERVICES)
  assert 'trafficRadarState' in LOCAL_DIAGNOSTIC_SERVICES


def test_rotated_files_are_independent_parseable_and_bounded(tmp_path):
  writer = NavigationLogWriter(tmp_path, max_files=2, max_seconds=1)
  writer.metadata = {'kind': 'metadata', 'settings': {'enabled': True}}
  for index in range(3):
    writer.write({'kind': 'sample', 'sequence': index}, wall_ms=1000 + index * 1001)
  writer.close(wall_ms=5000)
  files = scan_logs(tmp_path)
  assert len(files) == 2
  assert not list(tmp_path.glob('*.partial'))
  records = [[json.loads(line) for line in gzip.decompress(item.path.read_bytes()).splitlines()] for item in files]
  assert [rows[0]['kind'] for rows in records] == ['metadata', 'metadata']
  assert [rows[1]['sequence'] for rows in records] == [1, 2]


def test_download_contains_only_navigation_and_manifest(tmp_path):
  writer = NavigationLogWriter(tmp_path)
  writer.write({'kind': 'sample', 'value': float('nan')}, wall_ms=1000)
  writer.close(wall_ms=2000)
  (tmp_path / 'qlog.zst').write_bytes(b'not navigation')
  (tmp_path / 'spdiag-private.zst').write_bytes(b'not navigation')
  real = scan_logs(tmp_path)[0].path
  (tmp_path / 'navdiag-0000000001000-0000000002000-12345678.jsonl.gz').symlink_to(real)
  files = select_logs(1000, 3000, tmp_path)
  assert len(files) == 1
  output = io.BytesIO()
  stream_logs(files, output, start_ms=1000, end_ms=3000)
  with zipfile.ZipFile(io.BytesIO(output.getvalue())) as archive:
    assert archive.namelist() == ['manifest.json', f'navigation/{real.name}']
    assert not json.loads(archive.read('manifest.json'))['contains_other_diagnostics']
    assert json.loads(gzip.decompress(archive.read(f'navigation/{real.name}')))['value'] is None


@pytest.mark.parametrize('start,end', [(2000, 1000), (-1, 2000), (0, 8 * 24 * 3600 * 1000)])
def test_invalid_download_ranges_fail(start, end, tmp_path):
  with pytest.raises(ValueError):
    select_logs(start, end, tmp_path)


def test_sample_records_required_context_without_full_model_payload():
  class SM(dict):
    pass
  sm = SM({name: getattr(messaging.new_message(name), name) for name in SERVICES})
  sm.seen = sm.alive = sm.valid = dict.fromkeys(SERVICES, True)
  sm.logMonoTime = dict.fromkeys(SERVICES, 1_000_000_000)
  sample = collect_sample(sm, wall_ms=2000, mono_ns=1_100_000_000)
  assert sample['health']['modelV2']['age_ms'] == 100
  assert 'desireState' in sample['signals']['modelV2']
  assert 'laneTurnDirection' in sample['signals']['modelDataV2SP']
  assert 'leftRawMarking' in sample['signals']['laneTopologyStateSP']
  assert 'leftBlinker' in sample['signals']['carState']
  assert 'position' not in sample['signals']['modelV2']
  assert 'can' not in sample['signals']
  json.dumps(sample, allow_nan=False)


def test_turn_rejection_survives_cereal_roundtrip_and_log_collection():
  class SM(dict):
    pass
  sm = SM({name: getattr(messaging.new_message(name), name) for name in SERVICES})
  sm.seen = sm.alive = sm.valid = dict.fromkeys(SERVICES, True)
  sm.logMonoTime = dict.fromkeys(SERVICES, 1_000_000_000)
  message = messaging.new_message('modelDataV2SP')
  message.modelDataV2SP.turnEntryModelMonoTime = 900_000_000
  message.modelDataV2SP.turnEntryInputReason = 'observed'
  message.modelDataV2SP.turnEntryLeftReason = 'boundaryHeld/roadEdgeUncertain'
  message.modelDataV2SP.turnEntryRightReason = 'geometryUnavailable/roadEdgeUncertain'
  message.modelDataV2SP.turnDecisionReason = 'signalClassifiedAsLaneChange'
  with messaging.log.Event.from_bytes(message.to_bytes()) as reader:
    sm['modelDataV2SP'] = reader.modelDataV2SP
    sample = collect_sample(sm, wall_ms=2000, mono_ns=1_100_000_000)
  recorded = sample['signals']['modelDataV2SP']
  assert recorded['turnEntryModelMonoTime'] == 900_000_000
  assert recorded['turnEntryLeftReason'] == 'boundaryHeld/roadEdgeUncertain'
  assert recorded['turnDecisionReason'] == 'signalClassifiedAsLaneChange'



def efficiency_sm():
  class SM(dict): pass
  sm = SM({name: getattr(messaging.new_message(name), name) for name in SERVICES})
  sm.seen = dict.fromkeys(SERVICES, True)
  sm.alive = dict.fromkeys(SERVICES, True)
  sm.valid = dict.fromkeys(SERVICES, True)
  sm.logMonoTime = dict.fromkeys(SERVICES, 1_000_000_000)
  model = sm['modelV2']; model.laneLineProbs = [.99] * 4
  model.init('laneLines', 4)
  for line, y in zip(model.laneLines, (-5.25, -1.75, 1.75, 5.25)):
    line.x = [0., 100.]; line.y = [y, y]
  radar = sm['radarTracks']; radar.init('points', 3)
  for point, (track, d, y, v) in zip(radar.points, [(1, 40., 0., -5.), (2, 70., 3.5, 0.), (3, 75., -3.5, 0.)]):
    point.trackId = track; point.dRel = d; point.yRel = y; point.vRel = v; point.deprecated.measured = True
  sm['carState'].vEgo = 25.; sm['carState'].vCruise = 108.
  sm['laneTopologyStateSP'].leftNeighborExists = True
  sm['laneTopologyStateSP'].rightNeighborExists = True
  sm['navLaneIntentSP'].reason = 'efficiency:heuristicSignaling'
  sm['navLaneIntentSP'].direction = 'right'
  sm['navLaneIntentSP'].signalRequested = True
  return sm


def test_radar_snapshot_records_raw_inputs_and_reconstructs_lane_benefit(tmp_path):
  from openpilot.sunnypilot.navassist.efficiency_lane import lane_leads
  from types import SimpleNamespace as NS
  sm = efficiency_sm()
  sample = collect_sample(sm, wall_ms=2000, mono_ns=1_050_000_000, include_model_geometry=True)
  radar = sample['signals']['radarTracks']; ev = sample['efficiencyEvidence']
  assert len(radar['points']) == 3 and not radar['truncated']
  assert ev['basis'] == 'recorderSnapshot' and not ev['synchronousDecisionInputs']
  assert ev['laneLeads']['ego']['trackId'] == 1 and ev['laneLeads']['right']['trackId'] == 3
  assert ev['sides']['right']['gainKph'] == pytest.approx(18.)
  assert ev['sides']['right']['forwardGapClear']
  assert ev['sourceMonoNs']['radarTracks'] == sm.logMonoTime['radarTracks']
  writer = NavigationLogWriter(tmp_path); writer.write(sample, wall_ms=2000); writer.close(wall_ms=3000)
  saved = json.loads(gzip.decompress(scan_logs(tmp_path)[0].path.read_bytes()))
  points = [NS(**{k:v for k,v in p.items() if k!='measured'}, deprecated=NS(measured=p['measured'])) for p in saved['signals']['radarTracks']['points']]
  model = NS(laneLineProbs=saved['signals']['modelV2']['laneLineProbs'],
             laneLines=[NS(**line) for line in saved['signals']['modelV2']['geometry']['laneLines']])
  leads = lane_leads(NS(points=points), model, left_neighbor=False, right_neighbor=True)
  assert [p.trackId if p else None for p in leads] == [None, 1, 3]


def test_radar_snapshot_separates_unobserved_side_and_reports_all_unsafe_targets():
  sm = efficiency_sm()
  sm['navLaneIntentSP'].reason = 'efficiencyIdle'
  sm['navLaneIntentSP'].signalRequested = False
  sm['modelV2'].laneLines[3].x = []
  evidence = collect_sample(sm, wall_ms=2000, mono_ns=1_050_000_000)['efficiencyEvidence']
  assert evidence['classification'] == 'partialObserved'
  assert evidence['sideAssociationKnown'] == {'left': True, 'right': False}
  assert evidence['sides']['left']['forwardGapClear']
  assert evidence['sides']['right']['forwardGapClear'] is None

  sm = efficiency_sm()
  sm['navLaneIntentSP'].direction = 'left'
  radar = sm['radarTracks']
  radar.init('points', 4)
  for point, (track, d, y, v) in zip(radar.points, [(1, 40., 0., -5.), (2, 8., 3.5, 3.),
                                                   (3, 75., -3.5, 0.), (4, 20., 3.5, -10.)]):
    point.trackId = track; point.dRel = d; point.yRel = y; point.vRel = v; point.deprecated.measured = True
  evidence = collect_sample(sm, wall_ms=2000, mono_ns=1_050_000_000)['efficiencyEvidence']
  assert evidence['sides']['left']['forwardGapClear'] is False
  assert evidence['sides']['left']['unsafeTargetIds'] == [4]


@pytest.mark.parametrize('fault', ['unseen', 'stale', 'invalid', 'future', 'skew', 'error', 'empty', 'truncated', 'predicted', 'line'])
def test_missing_radar_or_geometry_never_records_clear_space(fault):
  sm = efficiency_sm()
  if fault == 'unseen': sm.seen['radarTracks'] = False
  if fault == 'stale': sm.logMonoTime['radarTracks'] = 500_000_000
  if fault == 'invalid': sm.valid['radarTracks'] = False
  if fault == 'future': sm.logMonoTime['radarTracks'] = 1_060_000_000
  if fault == 'skew': sm.logMonoTime['radarTracks'] = 850_000_000
  if fault == 'error': sm['radarTracks'].errors.radarFault = True
  if fault == 'empty': sm['radarTracks'].init('points', 0)
  if fault == 'truncated': sm['radarTracks'].init('points', 257)
  if fault == 'predicted': sm['radarTracks'].points[2].deprecated.measured = False
  if fault == 'line': sm['modelV2'].laneLineProbs[3] = .1
  sample = collect_sample(sm, wall_ms=2000, mono_ns=1_050_000_000)
  ev = sample['efficiencyEvidence']
  if fault == 'unseen': assert 'radarTracks' in ev['missing']
  elif fault in ('stale', 'invalid', 'future'): assert ev['classification'] == 'sourceUnavailable'
  elif fault == 'skew': assert ev['classification'] == 'sourceMisaligned'
  elif fault == 'error': assert ev['classification'] == 'radarError'
  elif fault == 'truncated':
    assert ev['classification'] == 'recordingTruncated'
    assert len(sample['signals']['radarTracks']['points']) == 256
  elif fault == 'empty': assert all(v is None for v in ev['laneLeads'].values())
  elif fault == 'line': assert ev['classification'] == 'observed'
  else: assert ev['classification'] == 'geometryOrAssociationUnknown'
  assert not ev['synchronousDecisionInputs']
