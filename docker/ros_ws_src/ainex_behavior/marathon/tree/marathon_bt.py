#!/usr/bin/env python3
"""
Marathon behavior tree wiring and bootstrap factory.

Tree structure:
  [-] MarathonBT (Sequence, memory=False)
      [o] SafetyGate (Selector, memory=False)
          --> L1_Balance_IsStanding
          [-] Recovery (Sequence, memory=False)
              --> L2_Gait_Stop
              --> L2_Balance_RecoverFromFall
      [o] PatrolControl (Selector, memory=False)
          [-] LineFollowing (Sequence, memory=False)
              --> L1_Vision_IsLineDetected
              --> IsHeadCentered  (head-sweep mode only)
              --> L2_Gait_FollowLine
          --> L2_Gait_FindLine / L2_Head_FindLineSweep  (RUNNING while searching)
          --> L2_Gait_Stop                              (fallback)

All leaf nodes receive only the Project Semantic Facade (AinexBTFacade).
No direct manager, publisher, or CommFacade references appear in this file.
"""
import py_trees

# ainex_bt_edu standard library nodes
from ainex_bt_edu.behaviours.L1_perception.L1_Balance_IsStanding import L1_Balance_IsStanding
from ainex_bt_edu.behaviours.L1_perception.L1_Vision_IsLineDetected import L1_Vision_IsLineDetected
from ainex_bt_edu.behaviours.L2_locomotion.L2_Gait_Stop import L2_Gait_Stop
from ainex_bt_edu.behaviours.L2_locomotion.L2_Gait_FollowLine import L2_Gait_FollowLine
from ainex_bt_edu.behaviours.L2_locomotion.L2_Gait_FindLine import L2_Gait_FindLine
from ainex_bt_edu.behaviours.L2_locomotion.L2_Head_FindLineSweep import L2_Head_FindLineSweep
from ainex_bt_edu.behaviours.L2_locomotion.L2_Balance_RecoverFromFall import L2_Balance_RecoverFromFall

# marathon-specific node (no generic equivalent — depends on /head_pan_pos BB key
# written by L2_Head_FindLineSweep; used only in head-sweep mode)
from marathon.behaviours.conditions import IsHeadCentered


class MarathonBT(py_trees.composites.Sequence):
    """Root sequence: SafetyGate must succeed before PatrolControl runs."""

    def __init__(self, semantic_facade,
                 find_line_cls=None, logger=None, tick_id_getter=None,
                 robot_state_setter=None):
        super().__init__(name="MarathonBT", memory=False)

        # --- SafetyGate ---
        safety_gate = py_trees.composites.Selector(
            name="SafetyGate", memory=False)
        recovery = py_trees.composites.Sequence(
            name="Recovery", memory=False)
        recovery.add_children([
            L2_Gait_Stop("StopWalking_recovery",
                         facade=semantic_facade,
                         tick_id_getter=tick_id_getter),
            L2_Balance_RecoverFromFall("RecoverFromFall",
                                       facade=semantic_facade,
                                       robot_state_setter=robot_state_setter,
                                       tick_id_getter=tick_id_getter),
        ])
        safety_gate.add_children([
            L1_Balance_IsStanding("IsRobotStanding",
                                  logger=logger,
                                  tick_id_getter=tick_id_getter),
            recovery,
        ])

        # --- PatrolControl ---
        patrol_control = py_trees.composites.Selector(
            name="PatrolControl", memory=False)
        line_following = py_trees.composites.Sequence(
            name="LineFollowing", memory=False)

        # Select FindLine variant: default (None) → gait-based L2_Gait_FindLine
        if find_line_cls is None or find_line_cls is L2_Gait_FindLine:
            line_following.add_children([
                L1_Vision_IsLineDetected("IsLineDetected",
                                         logger=logger,
                                         tick_id_getter=tick_id_getter),
                L2_Gait_FollowLine("FollowLine",
                                   facade=semantic_facade,
                                   tick_id_getter=tick_id_getter),
            ])
            find_line_node = L2_Gait_FindLine("FindLine",
                                               facade=semantic_facade,
                                               tick_id_getter=tick_id_getter)
        else:
            # Head-sweep mode: gate FollowLine until body is aligned (head centred)
            line_following.add_children([
                L1_Vision_IsLineDetected("IsLineDetected",
                                         logger=logger,
                                         tick_id_getter=tick_id_getter),
                IsHeadCentered("IsHeadCentered"),
                L2_Gait_FollowLine("FollowLine",
                                   facade=semantic_facade,
                                   tick_id_getter=tick_id_getter),
            ])
            find_line_node = find_line_cls("FindLine",
                                           facade=semantic_facade,
                                           tick_id_getter=tick_id_getter)

        patrol_control.add_children([
            line_following,
            find_line_node,
            L2_Gait_Stop("StopWalking_idle",
                         facade=semantic_facade,
                         tick_id_getter=tick_id_getter),
        ])

        self.add_children([safety_gate, patrol_control])


def bootstrap(semantic_facade,
              find_line_cls=None, logger=None, tick_id_getter=None,
              robot_state_setter=None):
    """
    Build and set up the full marathon BehaviourTree.

    Returns a ready-to-tick py_trees.trees.BehaviourTree.
    Pass find_line_cls=L2_Head_FindLineSweep to use head-sweep search instead of
    the default gait-based L2_Gait_FindLine.
    Pass logger + tick_id_getter to enable BT observability.
    """
    root = MarathonBT(semantic_facade,
                      find_line_cls=find_line_cls,
                      logger=logger,
                      tick_id_getter=tick_id_getter,
                      robot_state_setter=robot_state_setter)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)
    return tree
