#!/usr/bin/env python3
"""L1 Condition: Enumerates available .d6a action files, writes to BB."""
import os
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Motion_ListActions(AinexBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.AVAILABLE_ACTIONS]

    ACTION_PATH = '/home/ubuntu/software/ainex_controller/ActionGroups'

    def __init__(self, action_path=None, name='L1_Motion_ListActions'):
        super().__init__(name)
        self._action_path = action_path or self.ACTION_PATH
        self._actions = []

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._scan_actions()

    def _scan_actions(self):
        if os.path.isdir(self._action_path):
            self._actions = sorted([
                f[:-4] for f in os.listdir(self._action_path)
                if f.endswith('.d6a')
            ])

    def update(self):
        py_trees.blackboard.Blackboard.storage[
            BB.AVAILABLE_ACTIONS] = self._actions
        return Status.SUCCESS
