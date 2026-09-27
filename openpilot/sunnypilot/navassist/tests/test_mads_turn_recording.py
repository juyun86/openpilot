import gzip
import json

import pytest
from openpilot.cereal import messaging
from openpilot.sunnypilot.navassist.diagnostics import NavigationLogWriter, scan_logs
from openpilot.sunnypilot.navassist.diagnosticsd import SERVICES, collect_sample, MAX_PENDING_CAN239


def state():
  class SM(dict): pass
  sm = SM({name: getattr(messaging.new_message(name), name) for name in SERVICES})
  sm.seen = dict.fromkeys(SERVICES, True)
  sm.alive = dict.fromkeys(SERVICES, True)
  sm.valid = dict.fromkeys(SERVICES, True)
  sm.logMonoTime = dict.fromkeys(SERVICES, 1_000_000_000)
  sm['selfdriveStateSP'].mads.available = True
  sm['selfdriveStateSP'].mads.state = 'paused'
  sm['carState'].steeringPressed = True
  sm['carState'].steeringTorque = 1.5
  sm['carState'].steeringAngleDeg = 65.
  sm['carControl'].actuators.steeringAngleDeg = 40.
  sm['carOutput'].actuatorsOutput.steeringAngleDeg = 35.
  return sm


def test_paused_mads_manual_steering_is_recorded_with_separate_outputs():
  sample = collect_sample(state(), wall_ms=0, mono_ns=1_100_000_000)
  assert sample['driverIntervention'] == {'valid': True, 'madsSteeringPressed': True}
  signals = sample['signals']
  assert signals['selfdriveStateSP']['mads']['state'] == 'paused'
  assert signals['carState']['steeringAngleDeg'] == 65.
  assert signals['carControl']['actuators']['steeringAngleDeg'] == 40.
  assert signals['carOutput']['actuatorsOutput']['steeringAngleDeg'] == 35.
  assert not signals['carControl']['latActive']


@pytest.mark.parametrize('fault', ['unseen', 'invalid', 'dead', 'old', 'future'])
def test_missing_or_stale_intervention_is_unknown_not_false(fault):
  sm = state()
  if fault == 'unseen': sm.seen['selfdriveStateSP'] = False
  if fault == 'invalid': sm.valid['carState'] = False
  if fault == 'dead': sm.alive['selfdriveStateSP'] = False
  if fault == 'old': sm.logMonoTime['carState'] = 1
  if fault == 'future': sm.logMonoTime['carState'] = 2_000_000_000
  sample = collect_sample(sm, wall_ms=0, mono_ns=1_100_000_000)
  assert sample['driverIntervention'] == {'valid': False, 'madsSteeringPressed': None}


def test_turn_curves_keep_time_axes_and_video_alignment_under_record_size_limit(tmp_path):
  sm = state()
  md = sm['modelV2']
  md.frameId = 1234
  md.timestampEof = 999_000_000
  times = [i*.1 for i in range(33)]
  xs = [i*.2718281828459045 for i in range(33)]
  ys = [i*i*.03141592653589793 for i in range(33)]
  for name in ['position', 'velocity', 'orientation', 'orientationRate']:
    getattr(md, name).t = times
    getattr(md, name).x = xs
    getattr(md, name).y = ys
    getattr(md, name).z = ys
  for name, count in [('laneLines', 4), ('roadEdges', 2)]:
    for line in md.init(name, count):
      line.t, line.x, line.y = times, xs, ys
  md.roadEdgeStds = [.1, .2]
  sample = collect_sample(sm, wall_ms=1000, mono_ns=1_100_000_000, include_model_geometry=True)
  geometry = sample['signals']['modelV2']['geometry']
  assert geometry['position']['y'] == pytest.approx(ys)
  assert geometry['orientationRate']['t'] == pytest.approx(times)
  assert len(geometry['laneLines']) == 4 and len(geometry['roadEdges']) == 2
  assert sample['signals']['modelV2']['timestampEof'] == 999_000_000
  from openpilot.sunnypilot.navassist.diagnosticsd import collect_lane_can
  from types import SimpleNamespace as NS
  frames, _ = collect_lane_can([NS(logMonoTime=1, valid=True, can=[
    NS(address=0x239, src=1, dat=bytes.fromhex('511b6a7e787dfa80'))]*MAX_PENDING_CAN239)])
  sample['can239'] = frames
  writer = NavigationLogWriter(tmp_path)
  writer.write(sample, wall_ms=1000)
  writer.close(wall_ms=2000)
  raw = gzip.decompress(scan_logs(tmp_path)[0].path.read_bytes())
  assert len(raw) < 64*1024
  saved = json.loads(raw)
  assert saved['signals']['modelV2']['frameId'] == 1234
  assert saved['driverIntervention']['madsSteeringPressed']
  assert 'rawPredictions' not in saved['signals']['modelV2']
  assert not collect_sample(sm, wall_ms=0, mono_ns=1_100_000_000)['signals']['modelV2']['geometryIncluded']
