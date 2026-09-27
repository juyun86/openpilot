from openpilot.sunnypilot.navassist.oem_lane_feedback import OemLaneFeedback, LANE_CHANGE_ADDRESS
from openpilot.sunnypilot.navassist.lane_publisher import MODEL_MAX_AGE_NS, MARKING_MAX_AGE_NS
from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_blocker import CROSSABLE_EGO_MARKINGS, SOLID_EGO_MARKINGS, lane_topology_change_blocks


def confirmed_crossable_lanes(topology, *, healthy: bool, now_ns: int) -> tuple[bool, bool]:
  """Require current, control-valid visual evidence on each ego-side boundary."""
  if not healthy or not topology.validForControl:
    return False, False
  # Publisher validity describes the time the packet was built. Recheck its
  # evidence at use time so a stalled publisher cannot leave an old permission.
  for field, max_age_ns in (('publishMonoTime', MODEL_MAX_AGE_NS),
                            ('modelMonoTime', MODEL_MAX_AGE_NS),
                            ('imageMonoTime', MARKING_MAX_AGE_NS)):
    stamp_ns = getattr(topology, field, 0)
    if not (stamp_ns > 0 and 0 <= now_ns - stamp_ns <= max_age_ns):
      return False, False

  def side_allowed(side: str) -> bool:
    return bool(getattr(topology, side + 'EvidenceValid') and
                getattr(topology, side + 'NeighborExists') and
                getattr(topology, side + 'CrossingAllowed') and
                str(getattr(topology, side + 'EgoSideMarking')) in CROSSABLE_EGO_MARKINGS)

  return side_allowed('left'), side_allowed('right')


def lane_change_start_permissions(topology, *, healthy: bool, now_ns: int,
                                  oem_permissions: tuple[bool, bool],
                                  safety_blocks: tuple[bool, bool] = (False, False)) -> tuple[bool, bool]:
  """Either source may authorize crossing; a road edge or known hazard vetoes."""
  visual = confirmed_crossable_lanes(topology, healthy=healthy, now_ns=now_ns)
  blocked = lane_topology_change_blocks(topology, healthy=healthy)
  def side_allowed(side: str, index: int) -> bool:
    marking = str(getattr(topology, side + 'EgoSideMarking'))
    visual_solid = (healthy and topology.validForControl and
                    getattr(topology, side + 'EvidenceValid') and marking in SOLID_EGO_MARKINGS)
    return bool((visual[index] or oem_permissions[index]) and
                (not blocked[index] or (oem_permissions[index] and visual_solid)) and
                not safety_blocks[index])

  return side_allowed('left', 0), side_allowed('right', 1)


class OemLaneChangeGate:
  """Tesla start permission, independent of navigation and 0x239 topology."""

  def __init__(self):
    self.feedback = OemLaneFeedback()
    self.safety_blocks = (True, True)
    self.lane_change_safety_blocks = (False, False)
    self.neighbors = (None, None)

  def update(self, events, now_ns: int) -> tuple[bool, bool]:
    for event in events:
      if not event.valid:
        self.feedback = OemLaneFeedback()
        continue
      self.feedback.ingest((f for f in event.can if f.address == LANE_CHANGE_ADDRESS),
                           event.logMonoTime, received_ns=now_ns)
    state = self.feedback.snapshot(now_ns, include_topology_details=True)
    self.safety_blocks = (state['leftSafetyBlocked'], state['rightSafetyBlocked'])
    # A missing 0x399 is not a fresh veto of visual lane-change permission.
    # Keep the original fail-closed blocks for the separate low-speed turn path.
    self.lane_change_safety_blocks = (state['permissionValid'] and self.safety_blocks[0],
                                      state['permissionValid'] and self.safety_blocks[1])
    self.neighbors = (None, None)
    return state['leftAllowed'], state['rightAllowed']
