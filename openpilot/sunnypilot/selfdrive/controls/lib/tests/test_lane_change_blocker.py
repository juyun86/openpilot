from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_blocker import (
  LaneChangeBoundaryBlocker,
  lane_topology_nav_crossing_allowed,
  lane_topology_change_blocks,
  nav_lane_crossing_policy,
)


def topology(*, left="unknown", right="unknown", left_valid=False, right_valid=False, control_valid=True):
  return SimpleNamespace(
    validForControl=control_valid,
    leftEvidenceValid=left_valid,
    rightEvidenceValid=right_valid,
    leftEgoSideMarking=left,
    rightEgoSideMarking=right,
  )


@pytest.mark.parametrize("marking", ("solid", "doubleSolid", "solidDashed", "roadEdge"))
def test_reliable_non_crossable_marking_blocks_its_side(marking):
  assert lane_topology_change_blocks(topology(left=marking, left_valid=True), healthy=True) == (True, False)
  assert lane_topology_change_blocks(topology(right=marking, right_valid=True), healthy=True) == (False, True)


@pytest.mark.parametrize("marking", ("unknown", "dashed", "doubleDashed"))
def test_unknown_and_crossable_markings_do_not_create_a_global_block(marking):
  assert lane_topology_change_blocks(
    topology(left=marking, right=marking, left_valid=True, right_valid=True), healthy=True,
  ) == (False, False)


def test_unhealthy_or_observation_only_topology_is_not_used_as_a_global_block():
  observed = topology(left="solid", right="solid", left_valid=True, right_valid=True)
  assert lane_topology_change_blocks(observed, healthy=False) == (False, False)
  assert lane_topology_change_blocks(
    topology(left="solid", right="solid", left_valid=True, right_valid=True, control_valid=False), healthy=True,
  ) == (False, False)
  assert lane_topology_change_blocks(topology(left="solid", right="solid"), healthy=True) == (False, False)


def test_confirmed_solid_requires_crossable_evidence_to_clear():
  blocker = LaneChangeBoundaryBlocker(clear_frames=3)

  assert blocker.update(topology(left="solid", left_valid=True), healthy=True) == (True, False)
  assert blocker.update(topology(control_valid=False), healthy=True) == (True, False)
  assert blocker.update(topology(control_valid=False), healthy=True) == (True, False)
  for _ in range(30):
    assert blocker.update(topology(control_valid=False), healthy=True) == (True, False)
  for _ in range(2):
    assert blocker.update(topology(left="dashed", left_valid=True), healthy=True) == (True, False)
  assert blocker.update(topology(left="dashed", left_valid=True), healthy=True) == (False, False)


def test_solid_hold_is_independent_per_side_and_unknown_does_not_clear_it():
  blocker = LaneChangeBoundaryBlocker(clear_frames=2)

  assert blocker.update(topology(right="solid", right_valid=True), healthy=True) == (False, True)
  assert blocker.update(topology(), healthy=False) == (False, True)
  assert blocker.update(topology(), healthy=False) == (False, True)
  assert blocker.update(topology(right="doubleDashed", right_valid=True), healthy=True) == (False, True)
  assert blocker.update(topology(right="doubleDashed", right_valid=True), healthy=True) == (False, False)


def test_fork_policy_allows_unknown_or_solid_but_never_stale_geometry_or_road_edge():
  unknown = topology(left="unknown", left_valid=False)
  solid = topology(left="solid", left_valid=True)
  road_edge = topology(left="roadEdge", left_valid=True)

  assert lane_topology_nav_crossing_allowed(unknown, side="left", healthy=True, allow_unknown=True)
  assert lane_topology_nav_crossing_allowed(solid, side="left", healthy=True, ignore_solid=True)
  assert not lane_topology_nav_crossing_allowed(road_edge, side="left", healthy=True, ignore_solid=True)
  assert not lane_topology_nav_crossing_allowed(
    topology(left="unknown", left_valid=False, control_valid=False),
    side="left", healthy=True, allow_unknown=True,
  )


def test_fork_policy_clears_a_solid_hold_but_not_a_road_edge_hold():
  blocker = LaneChangeBoundaryBlocker(clear_frames=3)
  assert blocker.update(topology(left="solid", left_valid=True), healthy=True) == (True, False)
  assert blocker.update(topology(), healthy=True, ignore_left_solid=True) == (False, False)

  assert blocker.update(topology(left="roadEdge", left_valid=True), healthy=True) == (True, False)
  assert blocker.update(topology(left="roadEdge", left_valid=True), healthy=True,
                        ignore_left_solid=True) == (True, False)


def test_oem_positive_masks_solid_hold_only_while_present_and_never_road_edge():
  blocker = LaneChangeBoundaryBlocker(clear_frames=3)
  solid = topology(left="solid", left_valid=True)
  assert blocker.update(solid, healthy=True) == (True, False)
  assert blocker.update(solid, healthy=True, allow_left_oem_solid=True) == (False, False)
  assert blocker.update(solid, healthy=True) == (True, False)
  road_edge = topology(left="roadEdge", left_valid=True)
  assert blocker.update(road_edge, healthy=True, allow_left_oem_solid=True) == (True, False)


def test_model_policy_accepts_ordinary_unknown_without_applying_it_to_turn_only_or_opposite_side():
  intent = SimpleNamespace(valid=True, signalRequested=True, direction="left", targetLaneIndex=0,
                           allowUnknownCrossing=True, ignoreSolidBoundary=True, forkNow=False)
  assert nav_lane_crossing_policy(intent, "left") == (True, False)
  assert nav_lane_crossing_policy(intent, "right") == (False, False)
  allow_unknown, ignore_solid = nav_lane_crossing_policy(intent, "left")
  assert lane_topology_nav_crossing_allowed(topology(), side="left", healthy=True,
                                          allow_unknown=allow_unknown, ignore_solid=ignore_solid)
  assert not lane_topology_nav_crossing_allowed(topology(left="solid", left_valid=True), side="left", healthy=True,
                                              allow_unknown=allow_unknown, ignore_solid=ignore_solid)
  intent.forkNow = True
  assert nav_lane_crossing_policy(intent, "left") == (True, True)
  intent.targetLaneIndex = -1
  assert nav_lane_crossing_policy(intent, "left") == (False, False)
  intent.targetLaneIndex = 0
  intent.valid = False
  assert nav_lane_crossing_policy(intent, "left") == (False, False)


@pytest.mark.parametrize("healthy", [False, True])
def test_oem_ready_allows_unknown_without_visual_positive_permission(healthy):
  intent = SimpleNamespace(valid=True, signalRequested=True, direction="left", targetLaneIndex=0,
                           spLaneChangeReady=True)
  assert lane_topology_nav_crossing_allowed(topology(), side="left", healthy=healthy, nav_intent=intent)
  assert not lane_topology_nav_crossing_allowed(topology(), side="right", healthy=healthy, nav_intent=intent)


def test_oem_ready_overrides_visual_solid_but_not_road_edge():
  intent = SimpleNamespace(valid=True, signalRequested=True, direction="left", targetLaneIndex=0,
                           spLaneChangeReady=True)
  solid = topology(left="solid", left_valid=True)
  edge = topology(left="roadEdge", left_valid=True)
  assert not lane_topology_nav_crossing_allowed(solid, side="left", healthy=True, nav_intent=intent)
  assert lane_topology_nav_crossing_allowed(solid, side="left", healthy=True, nav_intent=intent,
                                           ignore_solid=True)
  assert not lane_topology_nav_crossing_allowed(edge, side="left", healthy=True, nav_intent=intent,
                                                ignore_solid=True)


@pytest.mark.parametrize("marking", ["solid", "doubleSolid", "solidDashed", "roadEdge"])
def test_oem_ready_does_not_override_confirmed_visual_veto(marking):
  intent = SimpleNamespace(valid=True, signalRequested=True, direction="left", targetLaneIndex=0,
                           spLaneChangeReady=True)
  assert not lane_topology_nav_crossing_allowed(
    topology(left=marking, left_valid=True), side="left", healthy=True, nav_intent=intent,
  )


@pytest.mark.parametrize("update", [dict(valid=False), dict(signalRequested=False), dict(targetLaneIndex=-1),
                                    dict(spLaneChangeReady=False), dict(direction="right")])
def test_visual_dashed_cannot_replace_navigation_authorization(update):
  values = dict(valid=True, signalRequested=True, direction="left", targetLaneIndex=0, spLaneChangeReady=True)
  intent = SimpleNamespace(**(values | update))
  assert not lane_topology_nav_crossing_allowed(
    topology(left="dashed", left_valid=True), side="left", healthy=True, nav_intent=intent,
  )
