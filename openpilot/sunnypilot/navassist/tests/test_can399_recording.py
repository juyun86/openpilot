import json
import gzip
from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.navassist import diagnosticsd
from openpilot.sunnypilot.navassist.diagnostics import NavigationLogWriter, scan_logs
from openpilot.sunnypilot.navassist.diagnosticsd import collect_can399, collect_lane_can
from openpilot.sunnypilot.navassist.oem_lane_feedback import OemLaneFeedback


def test_records_each_frame_with_source_timestamp_and_raw_bytes():
  def frame(address, src, data):
    return SimpleNamespace(address=address, src=src, dat=data)

  raw = ((7 << 46) | (2 << 4) | (1 << 6)).to_bytes(8, 'little')
  events = [SimpleNamespace(logMonoTime=123, valid=True, can=[
    frame(0x239, 1, raw), frame(0x399, 1, raw), frame(0x399, 0, raw),
  ]), SimpleNamespace(logMonoTime=456, valid=False, can=[frame(0x399, 1, b'\x01')])]
  rows = collect_can399(events)
  assert len(rows) == 3
  assert rows[0] == dict(mono_ns=123, event_valid=True, address=0x399, src=1,
                         data=raw.hex(), dlc=8, auto_lane_change_state=7,
                         blind_spot_left=2, blind_spot_right=1)
  assert rows[1]['src'] == 0
  assert rows[2] == dict(mono_ns=456, event_valid=False, address=0x399, src=1, data='01', dlc=1)
  assert json.loads(json.dumps(rows)) == rows
  assert collect_can399([]) == []


def lane_event(stamp, raw, *, valid=True, src=1):
  return SimpleNamespace(logMonoTime=stamp, valid=valid,
                         can=[SimpleNamespace(address=0x239, src=src, dat=raw)])


def test_239_records_shared_decoding_and_can_be_replayed(tmp_path):
  # Captured field combination: left present, right absent; both lines FUSED,
  # both forks UNAVAILABLE. Neither usage nor fork is a paint-type permission.
  raw = bytes.fromhex('511b6a7e787dfa80')
  events = [lane_event(2_000_000_000+i*100_000_000, raw[:7]+bytes([(8+i)<<4])) for i in range(3)]
  rows, other = collect_lane_can(iter(events))
  assert other == []
  first = rows[0]
  assert first['counter'] == 8
  assert first['left_lane_exists'] and not first['right_lane_exists']
  assert first['lane_width_m'] == 3.5625 and first['view_range_m'] == 27
  assert first['center_coefficients'] == pytest.approx([.21, .0016, -.0001, 0.])
  assert (first['left_line_usage'], first['right_line_usage']) == (2, 2)
  assert (first['left_fork'], first['right_fork']) == (3, 3)
  writer = NavigationLogWriter(tmp_path)
  writer.write({'kind': 'sample', 'can239': rows, 'can399': []}, wall_ms=1000)
  writer.close(wall_ms=2000)
  saved = json.loads(gzip.decompress(scan_logs(tmp_path)[0].path.read_bytes()))
  decoder = OemLaneFeedback()
  for row in saved['can239']:
    decoder.ingest([SimpleNamespace(address=row['address'], src=row['src'], dat=bytes.fromhex(row['data']))], row['mono_ns'])
  state = decoder.snapshot(events[-1].logMonoTime, include_topology_details=True)
  assert state['positionValid'] and state['position'] == 'rightmost'
  assert state['leftLaneEvidenceValid'] and state['rightLaneEvidenceValid']
  assert [state[f'centerC{i}'] for i in range(4)] == first['center_coefficients']
  assert not state['leftAllowed'] and not state['rightAllowed']


@pytest.mark.parametrize('size', [0, 1, 7, 9, 64])
def test_malformed_239_is_preserved_without_decoded_geometry(size):
  rows, other = collect_lane_can([lane_event(123, bytes(size), valid=False, src=0)])
  assert other == []
  assert rows == [dict(mono_ns=123, event_valid=False, address=0x239, src=0,
                       data=bytes(size).hex(), dlc=size)]


def test_mixed_generator_keeps_both_ids_sources_and_event_validity():
  def events():
    yield lane_event(1, bytes(8), valid=False, src=0)
    yield SimpleNamespace(logMonoTime=2, valid=True, can=[
      SimpleNamespace(address=0x399, src=1, dat=bytes(8)),
      SimpleNamespace(address=0x123, src=1, dat=bytes(8))])
    yield lane_event(3, bytes(8), src=1)
  rows239, rows399 = collect_lane_can(events())
  assert [(r['mono_ns'], r['src'], r['event_valid']) for r in rows239] == [(1, 0, False), (3, 1, True)]
  assert len(rows399) == 1 and rows399[0]['mono_ns'] == 2


@pytest.mark.parametrize('mode', ['offroad', 'disabled', 'overflow', 'write_failure'])
def test_recorder_keeps_original_cadence_and_bounds_pending_239(monkeypatch, tmp_path, mode):
  clock = SimpleNamespace(step=0)
  writes = []
  class Writer:
    metadata = None
    def close(self): pass
    def write(self, sample, **kwargs):
      if mode == 'write_failure' and clock.step == 0:
        raise OSError('temporary write failure')
      writes.append((clock.step, json.loads(json.dumps(sample))))
  class SM:
    seen = {'deviceState': True}
    def update(self, timeout): pass
    def __getitem__(self, key): return SimpleNamespace(started=False)
  class Keeper:
    def keep_time(self):
      clock.step += 1
      if clock.step > 20: raise KeyboardInterrupt
  def drain(*args, **kwargs):
    count = 40 if mode == 'overflow' and clock.step == 1 else 1
    return [lane_event(1000+clock.step*100+i, bytes(8)) for i in range(count)]
  settings = SimpleNamespace(logging_enabled=True)
  def read_settings():
    settings.logging_enabled = not (mode == 'disabled' and 5 <= clock.step < 15)
    return settings
  monkeypatch.setattr(diagnosticsd, 'SettingsCache', lambda: SimpleNamespace(read=read_settings, error=None))
  monkeypatch.setattr(diagnosticsd, 'Params', lambda: None)
  monkeypatch.setattr(diagnosticsd.messaging, 'SubMaster', lambda services: SM())
  monkeypatch.setattr(diagnosticsd.messaging, 'sub_sock', lambda *args, **kwargs: None)
  monkeypatch.setattr(diagnosticsd.messaging, 'drain_sock', drain)
  monkeypatch.setattr(diagnosticsd, 'NavigationLogWriter', Writer)
  monkeypatch.setattr(diagnosticsd, 'Ratekeeper', lambda hz: Keeper())
  monkeypatch.setattr(diagnosticsd, 'log_root', lambda: tmp_path)
  monkeypatch.setattr(diagnosticsd, 'ingress_status_path', lambda: tmp_path/'absent')
  monkeypatch.setattr(diagnosticsd, 'atomic_json', lambda *args, **kwargs: None)
  monkeypatch.setattr(diagnosticsd, 'recording_metadata', lambda *args: {})
  monkeypatch.setattr(diagnosticsd, 'collect_sample', lambda *args, **kwargs: {'kind': 'sample'})
  monkeypatch.setattr(diagnosticsd.time, 'monotonic', lambda: clock.step*.05)
  monkeypatch.setattr(diagnosticsd.time, 'monotonic_ns', lambda: int(clock.step*50_000_000))
  monkeypatch.setattr(diagnosticsd.time, 'time_ns', lambda: 1_000_000_000+clock.step*50_000_000)
  with pytest.raises(KeyboardInterrupt):
    diagnosticsd.main()
  if mode == 'write_failure':
    # The pending frame survives a failed write and is not silently discarded.
    assert [step for step, _ in writes] == [1]
    assert len(writes[0][1]['can239']) == 2
  else:
    assert [step for step, _ in writes] == [0, 20]
    last = writes[-1][1]
    assert len(last['can239']) == {'offroad': 20, 'disabled': 6, 'overflow': 32}[mode]
    assert last['can239_dropped'] == (27 if mode == 'overflow' else 0)
    if mode == 'disabled': assert last['can239'][0]['mono_ns'] == 2500
  assert all(sample['can399'] == [] for _, sample in writes)
