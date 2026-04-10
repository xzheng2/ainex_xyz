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
              --> IsHeadCentered  (head-sweep mode only)
              --> FollowLine
          --> FindLine / FindLineHeadSweep   (RUNNING while searching/aligning)
          --> StopWalking                    (fallback)

All leaf nodes receive only the Project Semantic Facade.  No direct
manager, publisher, or CommFacade references appear in this file.
"""
import py_trees

from marathon.behaviours.conditions import IsRobotStanding, IsLineDetected, IsHeadCentered
from marathon.behaviours.actions import RecoverFromFall, FollowLine, StopWalking, FindLine, FindLineHeadSweep


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
            StopWalking("StopWalking_recovery", semantic_facade,
                        tick_id_getter=tick_id_getter),
            RecoverFromFall("RecoverFromFall", semantic_facade,
                            robot_state_setter=robot_state_setter,
                            tick_id_getter=tick_id_getter),
        ])
        safety_gate.add_children([
            IsRobotStanding("IsRobotStanding", logger=logger,
                            tick_id_getter=tick_id_getter),
            recovery,
        ])

        # --- PatrolControl ---
        patrol_control = py_trees.composites.Selector(
            name="PatrolControl", memory=False)
        line_following = py_trees.composites.Sequence(
            name="LineFollowing", memory=False)

        # Select FindLine variant: default (None) → gait-based FindLine
        if find_line_cls is None or find_line_cls is FindLine:
            line_following.add_children([
                IsLineDetected("IsLineDetected", logger=logger,
                               tick_id_getter=tick_id_getter),
                FollowLine("FollowLine", semantic_facade,
                           tick_id_getter=tick_id_getter),
            ])
            find_line_node = FindLine("FindLine", semantic_facade,
                                     tick_id_getter=tick_id_getter)
        else:
            # Head-sweep mode: gate FollowLine until body is aligned (head centred)
            line_following.add_children([
                IsLineDetected("IsLineDetected", logger=logger,
                               tick_id_getter=tick_id_getter),
                IsHeadCentered("IsHeadCentered"),
                FollowLine("FollowLine", semantic_facade,
                           tick_id_getter=tick_id_getter),
            ])
            find_line_node = find_line_cls("FindLine", semantic_facade,
                                           tick_id_getter=tick_id_getter)

        patrol_control.add_children([
            line_following,
            find_line_node,
            StopWalking("StopWalking_idle", semantic_facade,
                        tick_id_getter=tick_id_getter),
        ])

        self.add_children([safety_gate, patrol_control])


def bootstrap(semantic_facade,
              find_line_cls=None, logger=None, tick_id_getter=None,
              robot_state_setter=None):
    """
    Build and set up the full marathon BehaviourTree.

    Returns a ready-to-tick py_trees.trees.BehaviourTree.
    Pass find_line_cls=FindLineHeadSweep to use head-sweep search instead of
    the default gait-based FindLine.
    Pass logger + tick_id_getter to enable BT observability.
    """
    root = MarathonBT(semantic_facade,
                      find_line_cls=find_line_cls,
                      logger=logger, tick_id_getter=tick_id_getter,
                      robot_state_setter=robot_state_setter)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)
    return tree
