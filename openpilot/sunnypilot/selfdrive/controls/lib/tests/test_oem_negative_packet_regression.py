"""Pure input regression: no messaging sockets, device I/O, or CAN publication."""
from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.navassist.oem_lane_feedback import OemLaneFeedback
from openpilot.sunnypilot.selfdrive.controls.lib.oem_lane_change_gate import (
  OemLaneChangeGate, lane_change_start_permissions,
)


@dataclass
class Frame:
  address: int
  dat: bytes
  src: int = 1


def frame_399(*, side=None, blind=0):
  raw = (8 << 46) | (blind << (4 if side == "left" else 6))
  data = bytearray(raw.to_bytes(7, "little"))
  data.append((0x99 + 0x03 + sum(data)) & 0xff)
  return Frame(0x399, bytes(data))


NOW = 2_000_000_000


def event(stamp, frames):
  return SimpleNamespace(valid=True, logMonoTime=stamp, can=frames)


def visual_topology():
  return SimpleNamespace(
    validForControl=True, leftEvidenceValid=True, rightEvidenceValid=True,
    leftNeighborExists=True, rightNeighborExists=True,
    leftCrossingAllowed=True, rightCrossingAllowed=True,
    leftEgoSideMarking="dashed", rightEgoSideMarking="dashed",
    publishMonoTime=NOW, modelMonoTime=NOW, imageMonoTime=NOW,
  )


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("blind", [1, 2, 3])
def test_same_packet_negative_observation_remains_a_safety_veto(side, blind):
  decoder = OemLaneFeedback()
  for stamp in (NOW - 2, NOW - 1):
    decoder.ingest([frame_399()], stamp)
  decoder.ingest([frame_399(), frame_399(side=side, blind=blind), frame_399()], NOW)
  result = decoder.snapshot(NOW, include_topology_details=True)
  assert not result[side + "Allowed"]
  # A duplicate positive has no independent source time to clear the hazard.
  assert result[side + "SafetyBlocked"], result


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("blind", [1, 2, 3])
def test_visual_positive_cannot_replace_same_packet_negative_observation(side, blind):
  gate = OemLaneChangeGate()
  oem = gate.update([
    event(NOW - 2, [frame_399()]),
    event(NOW - 1, [frame_399()]),
    event(NOW, [frame_399(), frame_399(side=side, blind=blind), frame_399()]),
  ], NOW)
  permissions = lane_change_start_permissions(
    visual_topology(), healthy=True, now_ns=NOW,
    oem_permissions=oem, safety_blocks=gate.safety_blocks,
  )
  index = 0 if side == "left" else 1
  assert not oem[index]
  assert not permissions[index], (oem, gate.safety_blocks, permissions)


@pytest.mark.parametrize('fault', ['checksum', 'dlc'])
def test_invalid_399_cannot_recover_from_later_frames_in_same_packet(fault):
  decoder = OemLaneFeedback()
  for stamp in (NOW - 2, NOW - 1):
    decoder.ingest([frame_399()], stamp)
  data = bytearray(frame_399().dat)
  if fault == 'checksum':
    data[-1] ^= 1
  else:
    data.pop()
  decoder.ingest([Frame(0x399, bytes(data)), frame_399(), frame_399()], NOW)
  result = decoder.snapshot(NOW, include_topology_details=True)
  assert not result['permissionValid']
  assert result['leftSafetyBlocked'] and result['rightSafetyBlocked']
  decoder.ingest([frame_399()], NOW + 1)
  result = decoder.snapshot(NOW + 1, include_topology_details=True)
  assert not result['leftAllowed'] and not result['rightAllowed']
  assert not result['leftSafetyBlocked'] and not result['rightSafetyBlocked']
