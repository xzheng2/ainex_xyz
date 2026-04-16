#!/usr/bin/env python3
"""
Fall-recovery behavior tree wiring and bootstrap factory.

Tree structure:
  [o] FallRecoveryBT (Selector, memory=False)
      --> L1_Balance_IsStanding
      [-] Recovery (Sequence, memory=False)
          --> L2_Gait_Stop
          --> L2_Balance_RecoverFromFall

All leaf nodes receive only the Project Semantic Facade (AinexBTFacade).
No direct manager, publisher, or CommFacade references appear in this file.
"""
import py_trees

# ainex_bt_edu standard library nodes
from ainex_bt_edu.behaviours.L1_perception.L1_Balance_IsStanding import L1_Balance_IsStanding
from ainex_bt_edu.behaviours.L2_locomotion.L2_Gait_Stop import L2_Gait_Stop
from ainex_bt_edu.behaviours.L2_locomotion.L2_Balance_RecoverFromFall import L2_Balance_RecoverFromFall


class FallRecoveryBT(py_trees.composites.Selector):
    """Root selector: succeeds when robot is standing, else runs recovery."""

    def __init__(self, semantic_facade,
                 logger=None, tick_id_getter=None,
                 robot_state_setter=None):
        super().__init__(name="FallRecoveryBT", memory=False)

        # --- Recovery sequence ---
        recovery = py_trees.composites.Sequence(name="Recovery", memory=False)
        recovery.add_children([
            L2_Gait_Stop("StopWalking",
                         facade=semantic_facade,
                         tick_id_getter=tick_id_getter),
            L2_Balance_RecoverFromFall("RecoverFromFall",
                                       facade=semantic_facade,
                                       robot_state_setter=robot_state_setter,
                                       tick_id_getter=tick_id_getter),
        ])

        self.add_children([
            L1_Balance_IsStanding("IsRobotStanding",
                                  logger=logger,
                                  tick_id_getter=tick_id_getter),
            recovery,
        ])


def bootstrap(semantic_facade,
              logger=None, tick_id_getter=None,
              robot_state_setter=None):
    """
    Build and set up the full fall-recovery BehaviourTree.

    Returns a ready-to-tick py_trees.trees.BehaviourTree.
    Pass logger + tick_id_getter to enable BT observability.
    """
    root = FallRecoveryBT(semantic_facade,
                          logger=logger,
                          tick_id_getter=tick_id_getter,
                          robot_state_setter=robot_state_setter)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)
    return tree
