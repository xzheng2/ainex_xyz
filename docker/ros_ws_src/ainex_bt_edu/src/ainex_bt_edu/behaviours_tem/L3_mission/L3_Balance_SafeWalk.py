#!/usr/bin/env python3
"""L3 Mission: Safety-guarded walking — checks standing, recovers from falls.
Implements the marathon SafetyGate pattern as a composite sub-tree."""
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB
from ainex_bt_edu.behaviours.L1_perception.L1_Balance_IsStanding import L1_Balance_IsStanding
from ainex_bt_edu.behaviours.L2_locomotion.L2_Gait_Disable import L2_Gait_Disable
from ainex_bt_edu.behaviours.L2_locomotion.L2_Balance_RecoverFromFall import L2_Balance_RecoverFromFall


class L3_Balance_SafeWalk(AinexBTNode):
    """
    Internal sub-tree:
        Selector(SafeWalk):
            --> L1_Balance_IsStanding
            --> Sequence(Recovery):
                --> L2_Gait_Disable
                --> L2_Balance_RecoverFromFall
    Returns SUCCESS if robot is standing or recovery succeeds.
    """
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.ROBOT_STATE, BB.GAIT_ENABLED]

    def __init__(self, motion_manager, gait_manager, buzzer_pub,
                 name='L3_Balance_SafeWalk'):
        super().__init__(name)
        self._motion_manager = motion_manager
        self._gait_manager = gait_manager
        self._buzzer_pub = buzzer_pub
        self._subtree = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        is_standing = L1_Balance_IsStanding()
        disable = L2_Gait_Disable(self._gait_manager)
        recover = L2_Balance_RecoverFromFall(
            self._motion_manager, self._gait_manager, self._buzzer_pub)

        recovery_seq = py_trees.composites.Sequence(
            name='Recovery', memory=False,
            children=[disable, recover])

        self._subtree = py_trees.composites.Selector(
            name='SafeWalk', memory=False,
            children=[is_standing, recovery_seq])

        self._subtree.setup(**kwargs)

    def update(self):
        self._subtree.tick_once()
        return self._subtree.status

    def terminate(self, new_status):
        if self._subtree:
            self._subtree.stop(Status.INVALID)
        super().terminate(new_status)
