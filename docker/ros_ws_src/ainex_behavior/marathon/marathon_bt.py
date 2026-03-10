#!/usr/bin/env python3
"""
Marathon behavior tree wiring and bootstrap factory.

Tree structure:
  [-] MarathonBT (Sequence, memory=False)
      [o] SafetyGate (Selector, memory=False)
          --> IsRobotStanding
          [-] Recovery (Sequence, memory=False)
              --> StopWalking
              --> RecoverFromFall
      [o] PatrolControl (Selector, memory=False)
          [-] LineFollowing (Sequence, memory=False)
              --> IsLineDetected
              --> FollowLine
          --> StopWalking
"""
import py_trees

from behaviours.conditions import IsRobotStanding, IsLineDetected
from behaviours.actions import RecoverFromFall, FollowLine, StopWalking


class MarathonBT(py_trees.composites.Sequence):
    """Root sequence: SafetyGate must succeed before PatrolControl runs."""

    def __init__(self, motion_manager, gait_manager, visual_patrol, buzzer_pub):
        super().__init__(name="MarathonBT", memory=False)

        # --- SafetyGate ---
        safety_gate = py_trees.composites.Selector(
            name="SafetyGate", memory=False)
        recovery = py_trees.composites.Sequence(
            name="Recovery", memory=False)
        recovery.add_children([
            StopWalking("StopWalking_recovery", gait_manager),
            RecoverFromFall("RecoverFromFall", motion_manager,
                            gait_manager, buzzer_pub),
        ])
        safety_gate.add_children([
            IsRobotStanding("IsRobotStanding"),
            recovery,
        ])

        # --- PatrolControl ---
        patrol_control = py_trees.composites.Selector(
            name="PatrolControl", memory=False)
        line_following = py_trees.composites.Sequence(
            name="LineFollowing", memory=False)
        line_following.add_children([
            IsLineDetected("IsLineDetected"),
            FollowLine("FollowLine", visual_patrol),
        ])
        patrol_control.add_children([
            line_following,
            StopWalking("StopWalking_idle", gait_manager),
        ])

        self.add_children([safety_gate, patrol_control])


def bootstrap(motion_manager, gait_manager, visual_patrol, buzzer_pub):
    """
    Build and set up the full marathon BehaviourTree.

    Returns a ready-to-tick py_trees.trees.BehaviourTree.
    """
    root = MarathonBT(motion_manager, gait_manager, visual_patrol, buzzer_pub)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)
    return tree
