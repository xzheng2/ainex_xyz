#!/usr/bin/env python3
"""
{{PROJECT_CLASS}} behavior tree wiring and bootstrap factory.

All leaf nodes receive only the Project RuntimeFacade (XyzBTFacade).
No node may hold gait_manager, motion_manager, or raw ROS references.

Tree structure:
  [-] {{PROJECT_CLASS}}BT (Sequence, memory=False)
      [o] SafetyGate (Selector, memory=False)
          --> L1_Balance_IsStanding
          [-] Recovery (Sequence, memory=False)
              --> L2_Gait_Stop
              --> L2_Balance_RecoverFromFall
      [o] PatrolControl (Selector, memory=False)
          [-] LineFollowing (Sequence, memory=False)
              --> L1_Vision_IsLineDetected
              --> L2_Gait_FollowLine
          --> L2_Gait_FindLine
          --> L2_Gait_Stop
"""
import py_trees

# xyz_bt_edu standard library nodes
from xyz_bt_edu.behaviours.L1_perception.L1_Balance_IsStanding import L1_Balance_IsStanding
from xyz_bt_edu.behaviours.L1_perception.L1_Vision_IsLineDetected import L1_Vision_IsLineDetected
from xyz_bt_edu.behaviours.L2_locomotion.L2_Gait_Stop import L2_Gait_Stop
from xyz_bt_edu.behaviours.L2_locomotion.L2_Gait_FollowLine import L2_Gait_FollowLine
from xyz_bt_edu.behaviours.L2_locomotion.L2_Gait_FindLine import L2_Gait_FindLine
from xyz_bt_edu.behaviours.L2_locomotion.L2_Balance_RecoverFromFall import L2_Balance_RecoverFromFall
# TODO: import project-specific nodes from behaviours/actions.py if needed


def bootstrap(runtime_facade, robot_state_setter=None,
              logger=None, tick_id_getter=None):
    """Build and return the {{PROJECT_CLASS}} behaviour tree.

    Args:
        runtime_facade:     {{PROJECT_CLASS}}RuntimeFacade instance (XyzBTFacade).
        robot_state_setter: callable(str) to sync ImuBalanceStateAdapter live store.
        logger:             BT observability logger (None = zero-cost no-op).
        tick_id_getter:     callable → current tick_id (int).

    Returns:
        py_trees.trees.BehaviourTree
    """
    # ── Nodes ────────────────────────────────────────────────────────────
    is_standing = L1_Balance_IsStanding(
        logger=logger, tick_id_getter=tick_id_getter)

    gait_stop_recovery = L2_Gait_Stop(
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter)

    recover_from_fall = L2_Balance_RecoverFromFall(
        facade=runtime_facade,
        robot_state_setter=robot_state_setter,
        logger=logger, tick_id_getter=tick_id_getter)

    is_line_detected = L1_Vision_IsLineDetected(
        logger=logger, tick_id_getter=tick_id_getter)

    follow_line = L2_Gait_FollowLine(
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter)

    find_line = L2_Gait_FindLine(
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter)

    gait_stop_fallback = L2_Gait_Stop(
        name='L2_Gait_Stop_fallback',
        facade=runtime_facade, logger=logger, tick_id_getter=tick_id_getter)

    # ── Tree ─────────────────────────────────────────────────────────────
    recovery_seq = py_trees.composites.Sequence(
        name='Recovery', memory=False)
    recovery_seq.add_children([gait_stop_recovery, recover_from_fall])

    safety_gate = py_trees.composites.Selector(
        name='SafetyGate', memory=False)
    safety_gate.add_children([is_standing, recovery_seq])

    line_following_seq = py_trees.composites.Sequence(
        name='LineFollowing', memory=False)
    line_following_seq.add_children([is_line_detected, follow_line])

    patrol_control = py_trees.composites.Selector(
        name='PatrolControl', memory=False)
    patrol_control.add_children([line_following_seq, find_line, gait_stop_fallback])

    root = py_trees.composites.Sequence(
        name='{{PROJECT_CLASS}}BT', memory=False)
    root.add_children([safety_gate, patrol_control])

    return py_trees.trees.BehaviourTree(root)
