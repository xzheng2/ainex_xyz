#!/usr/bin/env python3
"""L2 Action: Enable gait control. Always returns SUCCESS."""
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L2_Gait_Enable(AinexBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.GAIT_ENABLED]

    def __init__(self, gait_manager, name='L2_Gait_Enable'):
        super().__init__(name)
        self.gait_manager = gait_manager

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def update(self):
        self.gait_manager.enable()
        py_trees.blackboard.Blackboard.storage[
            BB.GAIT_ENABLED] = True
        return Status.SUCCESS
