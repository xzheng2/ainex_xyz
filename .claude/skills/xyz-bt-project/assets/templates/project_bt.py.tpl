#!/usr/bin/env python3
"""
{{PROJECT_CLASS}} behavior tree wiring and bootstrap factory.

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

All leaf nodes receive only the Project Semantic Facade (XyzBTFacade).
"""
import py_trees

# xyz_bt_edu standard library nodes
from xyz_bt_edu.behaviours.L1_perception.L1_Balance_IsStanding import L1_Balance_IsStanding
from xyz_bt_edu.behaviours.L1_perception.L1_Vision_IsLineDetected import L1_Vision_IsLineDetected
from xyz_bt_edu.behaviours.L2_locomotion.L2_Gait_Stop import L2_Gait_Stop
from xyz_bt_edu.behaviours.L2_locomotion.L2_Gait_FollowLine import L2_Gait_FollowLine
from xyz_bt_edu.behaviours.L2_locomotion.L2_Gait_FindLine import L2_Gait_FindLine
from xyz_bt_edu.behaviours.L2_locomotion.L2_Balance_RecoverFromFall import L2_Balance_RecoverFromFall

# {{PROJECT_CLASS}}-specific nodes (only those with no xyz_bt_edu equivalent)
# from {{PROJECT}}.behaviours.conditions import SomeProjectSpecificCondition


class {{PROJECT_CLASS}}BT(py_trees.composites.Sequence):
    """Root sequence: SafetyGate must succeed before PatrolControl runs."""

    def __init__(self, semantic_facade,
                 logger=None, tick_id_getter=None,
                 robot_state_setter=None):
        super().__init__(name="{{PROJECT_CLASS}}BT", memory=False)

        # --- SafetyGate ---
        safety_gate = py_trees.composites.Selector(name="SafetyGate", memory=False)
        recovery = py_trees.composites.Sequence(name="Recovery", memory=False)
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
        patrol_control = py_trees.composites.Selector(name="PatrolControl", memory=False)
        line_following = py_trees.composites.Sequence(name="LineFollowing", memory=False)
        line_following.add_children([
            L1_Vision_IsLineDetected("IsLineDetected",
                                     logger=logger,
                                     tick_id_getter=tick_id_getter),
            L2_Gait_FollowLine("FollowLine",
                               facade=semantic_facade,
                               tick_id_getter=tick_id_getter),
        ])
        patrol_control.add_children([
            line_following,
            L2_Gait_FindLine("FindLine",
                             facade=semantic_facade,
                             tick_id_getter=tick_id_getter),
            L2_Gait_Stop("StopWalking_idle",
                         facade=semantic_facade,
                         tick_id_getter=tick_id_getter),
        ])

        self.add_children([safety_gate, patrol_control])


def bootstrap(semantic_facade,
              logger=None, tick_id_getter=None,
              robot_state_setter=None):
    """Build and return a ready-to-tick BehaviourTree."""
    root = {{PROJECT_CLASS}}BT(semantic_facade,
                               logger=logger,
                               tick_id_getter=tick_id_getter,
                               robot_state_setter=robot_state_setter)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)
    return tree
