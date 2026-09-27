"""Read-only navigation recorder: no controller or publisher is created here."""
from __future__ import annotations

import time
import json
import math
from dataclasses import asdict

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.sunnypilot.navassist.diagnostics import NavigationLogWriter, ingress_status_path, log_root, recording_metadata
from openpilot.sunnypilot.navassist.settings import NavAssistSettings, SettingsCache, atomic_json
from openpilot.sunnypilot.navassist.efficiency_lane import lane_leads, side_lead_unsafe
from openpilot.sunnypilot.navassist.lane_can import decode_lane_topology


SERVICES = ('navAssistStateSP', 'navLaneIntentSP', 'laneTopologyStateSP', 'modelV2', 'modelDataV2SP',
            'carState', 'carStateSP', 'carControl', 'carOutput', 'selfdriveStateSP',
            'controlsState', 'longitudinalPlanSP', 'deviceState', 'radarTracks', 'carParamsSP')


MAX_PENDING_CAN239 = 32


def collect_lane_can(events) -> tuple[list[dict], list[dict]]:
  """Split 239/399 in one pass; preserve invalid observations for offline replay."""
  frames239, frames399 = [], []
  for event in events:
    for frame in event.can:
      if frame.address not in (0x239, 0x399):
        continue
      data = bytes(frame.dat)
      item = {'mono_ns': event.logMonoTime, 'event_valid': event.valid,
              'address': frame.address, 'src': frame.src, 'data': data.hex(), 'dlc': len(data)}
      if frame.address == 0x239:
        decoded = decode_lane_topology(data)
        if decoded is not None:
          item.update(decoded)
        frames239.append(item)
      elif len(data) == 8:
        raw = int.from_bytes(data, 'little')
        item.update(auto_lane_change_state=(raw >> 46) & 31,
                    blind_spot_left=(raw >> 4) & 3, blind_spot_right=(raw >> 6) & 3)
      if frame.address == 0x399:
        frames399.append(item)
  return frames239, frames399


def collect_can399(events) -> list[dict]:
  """Retain the existing recorder/test API and 399 row format."""
  return collect_lane_can(events)[1]


def model_geometry(model) -> dict:
  """Bounded model-space curves, including horizons; never copy raw predictions."""
  def curve(value, fields):
    return {field: list(getattr(value, field))[:33] for field in fields}
  return {
    'position': curve(model.position, ('t', 'x', 'y', 'z')),
    'velocity': curve(model.velocity, ('t', 'x', 'y', 'z')),
    'orientation': curve(model.orientation, ('t', 'z')),
    'orientationRate': curve(model.orientationRate, ('t', 'z')),
    'laneLines': [curve(line, ('t', 'x', 'y')) for line in list(model.laneLines)[:4]],
    'roadEdges': [curve(line, ('t', 'x', 'y')) for line in list(model.roadEdges)[:2]],
    'laneLineStds': list(model.laneLineStds)[:4],
    'roadEdgeStds': list(model.roadEdgeStds)[:2],
  }


MAX_RECORDED_RADAR_POINTS = 256


def efficiency_evidence(sm, signals, health, settings):
  """Recorder-side reconstruction, not the selector's synchronous decision inputs."""
  required = ('radarTracks', 'modelV2', 'carState', 'laneTopologyStateSP', 'navLaneIntentSP')
  evidence = {
    'basis': 'recorderSnapshot', 'synchronousDecisionInputs': False,
    'settings': {k: v for k, v in asdict(settings).items() if k.startswith('overtake_') or k == 'efficiency_lane_change_enabled'},
    'sourceMonoNs': {k: health[k]['mono_ns'] for k in required},
    'missing': [k for k in required if k not in signals],
  }
  if evidence['missing']:
    return evidence
  intent = signals['navLaneIntentSP']
  evidence['actualIntent'] = {k: intent.get(k) for k in (
    'publishMonoTime', 'sessionId', 'maneuverEventId', 'requestId', 'reason',
    'signalRequested', 'direction', 'spLaneChangeReady', 'announcementId')}
  evidence['radarModelSkewMs'] = abs(health['radarTracks']['mono_ns'] - health['modelV2']['mono_ns']) / 1e6
  if signals['radarTracks']['truncated']:
    evidence['classification'] = 'recordingTruncated'
    return evidence
  if not all(health[k]['alive'] and health[k]['valid'] and health[k]['age_ms'] is not None
             and 0 <= health[k]['age_ms'] <= 200 for k in ('radarTracks', 'modelV2')):
    evidence['classification'] = 'sourceUnavailable'
    return evidence
  if evidence['radarModelSkewMs'] > 100:
    evidence['classification'] = 'sourceMisaligned'
    return evidence
  if any(signals['radarTracks']['errors'].values()):
    evidence['classification'] = 'radarError'
    return evidence
  topo = sm['laneTopologyStateSP']
  # Only mirror the producer's side-specific corridor when its published intent
  # explicitly identifies an efficiency request. Other snapshots evaluate both.
  side = intent.get('direction', 'none') if intent.get('reason', '').startswith('efficiency:') and intent.get('signalRequested', False) else 'none'
  enabled_sides = {'left': bool(topo.leftNeighborExists) and side != 'right',
                   'right': bool(topo.rightNeighborExists) and side != 'left'}
  observations = {}
  for name, enabled in enabled_sides.items():
    observations[name] = (lane_leads(sm['radarTracks'], sm['modelV2'],
                                     left_neighbor=name == 'left', right_neighbor=name == 'right', all_targets=True)
                          if enabled else None)
  known = next((targets for targets in observations.values() if targets is not None), None)
  leads = ([min(observations['left'][0], key=lambda point: point.dRel, default=None) if observations['left'] is not None else None,
            min(known[1], key=lambda point: point.dRel, default=None),
            min(observations['right'][2], key=lambda point: point.dRel, default=None) if observations['right'] is not None else None]
           if known is not None else None)
  evidence['corridorDirection'] = side
  evidence['sideAssociationKnown'] = {name: targets is not None for name, targets in observations.items()}
  evidence['classification'] = ('geometryOrAssociationUnknown' if leads is None else
                                'partialObserved' if any(observations[name] is None for name in enabled_sides if enabled_sides[name])
                                else 'observed')
  if leads is None:
    return evidence
  evidence['laneLeads'] = {name: None if lead is None else {
    'trackId': lead.trackId, 'dRel': lead.dRel, 'yRel': lead.yRel, 'vRel': lead.vRel,
    'measured': lead.deprecated.measured,
  } for name, lead in zip(('left', 'ego', 'right'), leads)}
  current = leads[1]; car = sm['carState']; speed = float(car.vEgo); cruise = float(car.vCruise) / 3.6
  if current is None or not all(math.isfinite(v) for v in (speed, cruise)) or speed <= 0:
    return evidence
  evidence['demand'] = {
    'distanceInWindow': settings.overtake_min_distance_m <= current.dRel <= settings.overtake_max_distance_m,
    'leadSpeedKph': (speed + current.vRel) * 3.6,
    'leadSpeedEnough': speed + current.vRel >= settings.overtake_lead_min_kph / 3.6,
    'closingEnough': current.vRel < -settings.overtake_closing_kph / 3.6,
    'timeGapS': current.dRel / speed,
    'timeGapTriggered': current.dRel / speed <= settings.overtake_time_gap_tenths / 10,
    'cruiseTriggered': speed < cruise * settings.overtake_cruise_percent / 100,
  }
  evidence['sides'] = {}
  for name, lead in (('left', leads[0]), ('right', leads[2])):
    if observations[name] is None:
      evidence['sides'][name] = {'targetObserved': None, 'forwardGapClear': None}
      continue
    if lead is None:
      evidence['sides'][name] = {'targetObserved': False, 'forwardGapClear': True}
      continue
    ttc = lead.dRel / -lead.vRel if lead.vRel < 0 else None
    gain = (min(cruise, speed + lead.vRel) - min(cruise, speed + current.vRel)) * 3.6
    unsafe_ids = [point.trackId for point in observations[name][0 if name == 'left' else 2]
                  if side_lead_unsafe(point, speed)]
    evidence['sides'][name] = {'targetObserved': True, 'forwardGapClear': not unsafe_ids,
      'unsafeTargetIds': unsafe_ids,
      'ttcS': ttc, 'gainKph': gain,
      'gainEnough': gain > 0}
  return evidence


def collect_sample(sm, *, wall_ms: int, mono_ns: int, include_model_geometry: bool = False,
                   settings: NavAssistSettings | None = None) -> dict:
  signals = {}
  health = {}
  for name in SERVICES:
    health[name] = {
      'seen': sm.seen[name], 'alive': sm.alive[name], 'valid': sm.valid[name],
      'mono_ns': sm.logMonoTime[name],
      'age_ms': (mono_ns - sm.logMonoTime[name]) / 1e6 if sm.seen[name] else None,
    }
    if not sm.seen[name]:
      continue
    value = sm[name]
    if name in ('navAssistStateSP', 'navLaneIntentSP', 'laneTopologyStateSP'):
      signals[name] = value.to_dict()
    elif name == 'radarTracks':
      total = len(value.points)
      signals[name] = {'errors': value.errors.to_dict(), 'pointCount': total,
        'truncated': total > MAX_RECORDED_RADAR_POINTS,
        'points': [{key: getattr(point, key) for key in ('trackId', 'dRel', 'yRel', 'vRel')} |
                   {'measured': point.deprecated.measured} for point in list(value.points)[:MAX_RECORDED_RADAR_POINTS]]}
    elif name == 'carParamsSP':
      signals[name] = {'flags': value.flags}
    elif name == 'carState':
      signals[name] = {key: getattr(value, key) for key in (
        'vEgo', 'aEgo', 'steeringAngleDeg', 'steeringTorque', 'steeringPressed', 'brakePressed', 'gasPressed',
        'leftBlinker', 'rightBlinker', 'leftBlindspot', 'rightBlindspot', 'vCruise',
        'steeringRateDeg', 'steeringTorqueEps', 'yawRate', 'standstill',
        'steerFaultTemporary', 'steerFaultPermanent',
      )}
      signals[name]['gearShifter'] = str(value.gearShifter)
    elif name == 'carStateSP':
      signals[name] = {'flags': value.flags}
    elif name == 'carControl':
      signals[name] = {'enabled': value.enabled, 'latActive': value.latActive, 'longActive': value.longActive,
                       'override': value.cruiseControl.override, 'actuators': value.actuators.to_dict()}
    elif name == 'carOutput':
      signals[name] = {'actuatorsOutput': value.actuatorsOutput.to_dict()}
    elif name == 'selfdriveStateSP':
      signals[name] = {'mads': value.mads.to_dict()}
    elif name == 'controlsState':
      signals[name] = {'desiredCurvature': value.desiredCurvature, 'curvature': value.curvature,
                       'longControlState': str(value.longControlState), 'lateralControlState': value.lateralControlState.to_dict()}
    elif name == 'modelV2':
      signals[name] = {'frameId': value.frameId, 'laneChangeState': str(value.meta.laneChangeState),
                       'laneChangeDirection': str(value.meta.laneChangeDirection), 'desireState': list(value.meta.desireState)[:8],
                       'laneLineProbs': list(value.laneLineProbs)[:4], 'action': value.action.to_dict()}
      signals[name].update(timestampEof=value.timestampEof, frameIdExtra=value.frameIdExtra,
                           frameDropPerc=value.frameDropPerc, modelExecutionTime=value.modelExecutionTime,
                           geometryIncluded=include_model_geometry)
      if include_model_geometry:
        signals[name]['geometry'] = model_geometry(value)
    elif name == 'modelDataV2SP':
      signals[name] = {'laneTurnDirection': str(value.laneTurnDirection)}
      for field in ('turnEntryModelMonoTime', 'turnEntryInputReason', 'turnEntryLeftReason',
                    'turnEntryRightReason', 'turnDecisionReason'):
        signals[name][field] = getattr(value, field, 0 if field == 'turnEntryModelMonoTime' else '')
    elif name == 'longitudinalPlanSP':
      signals[name] = {'source': str(value.longitudinalPlanSource), 'vTarget': value.vTarget, 'aTarget': value.aTarget,
                       'vision': value.smartCruiseControl.vision.to_dict(), 'accelController': value.accelController.to_dict()}
    elif name == 'deviceState':
      signals[name] = {'started': value.started}
  # A paused/overriding MADS state is still relevant. Do not require latActive.
  def fresh(name):
    item = health[name]
    return (item['seen'] and item['alive'] and item['valid']
            and 0 <= mono_ns-item['mono_ns'] <= 250_000_000)
  valid = fresh('carState') and fresh('selfdriveStateSP')
  intervention = {'valid': valid, 'madsSteeringPressed': (
    bool(sm['selfdriveStateSP'].mads.available and sm['carState'].steeringPressed) if valid else None)}
  return {'kind': 'sample', 'wall_time_ms': wall_ms, 'mono_time_ns': mono_ns,
          'health': health, 'signals': signals, 'driverIntervention': intervention,
          'efficiencyEvidence': efficiency_evidence(sm, signals, health, settings or NavAssistSettings())}


def main() -> None:
  cache = SettingsCache()
  params = Params()
  sm = messaging.SubMaster(list(SERVICES))
  can_sock = messaging.sub_sock('can', conflate=False)
  writer = NavigationLogWriter()
  ratekeeper = Ratekeeper(20)
  previous_settings = None
  previous_started = False
  next_sample = next_status = 0.0
  next_ingress = 0.0
  ingress = {}
  last_sample_ms = None
  last_error = None
  pending_can239 = []
  dropped_can239 = 0
  root = log_root()
  try:
    while True:
      sm.update(0)
      now = time.monotonic()
      wall_ms = time.time_ns() // 1_000_000
      settings = cache.read()
      # Drain even while recording is disabled; never retain an unbounded backlog.
      can239, can399 = collect_lane_can(messaging.drain_sock(can_sock, wait_for_one=False))
      if settings.logging_enabled:
        # Preserve 239 between existing offroad writes; do not raise the sample rate.
        pending_can239.extend(can239)
        dropped_can239 += max(0, len(pending_can239) - MAX_PENDING_CAN239)
        pending_can239 = pending_can239[-MAX_PENDING_CAN239:]
      else:
        pending_can239 = []
        dropped_can239 = 0
      started = bool(sm.seen['deviceState'] and sm['deviceState'].started)
      if now >= next_ingress:
        next_ingress = now + 1.0
        try:
          path = ingress_status_path()
          ingress = json.loads(path.read_text()) if path.stat().st_size <= 4096 else {}
        except (OSError, ValueError):
          ingress = {}
      try:
        if settings != previous_settings or started != previous_started:
          writer.close()
          writer.metadata = recording_metadata(settings, params)
          writer.metadata['recording_features'] = ['can239', 'mads_turn_path_v1', 'efficiency_radar_snapshot_v1']
          previous_settings, previous_started = settings, started
        marker = root / 'flush.request'
        if marker.exists():
          writer.close()
          marker.unlink(missing_ok=True)
        if settings.logging_enabled and (started or can399 or now >= next_sample):
          sample = collect_sample(sm, wall_ms=wall_ms, mono_ns=time.monotonic_ns(), include_model_geometry=started, settings=settings)
          sample['can399'] = can399
          sample['can239'] = pending_can239
          sample['can239_dropped'] = dropped_can239
          sample['udp_ingress'] = ingress
          writer.write(sample, wall_ms=wall_ms)
          pending_can239 = []
          dropped_can239 = 0
          last_sample_ms = wall_ms
          next_sample = now + (0.05 if started else 1.0)
        last_error = cache.error
      except (OSError, ValueError) as error:
        last_error = str(error)
      if now >= next_status:
        next_status = now + 5.0
        try:
          atomic_json(root / 'status.json', {
            'updated_ms': wall_ms, 'last_sample_ms': last_sample_ms,
            'recording_enabled': settings.logging_enabled, 'error': last_error,
          })
        except OSError:
          pass
      ratekeeper.keep_time()
  finally:
    writer.close()


if __name__ == '__main__':
  main()
