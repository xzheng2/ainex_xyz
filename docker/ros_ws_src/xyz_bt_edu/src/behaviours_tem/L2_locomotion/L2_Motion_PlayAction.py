#!/usr/bin/env python3
"""L2 Action: Play a named .d6a action group (blocking)."""
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzBTNode


class L2_Motion_PlayAction(XyzBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = []

    def __init__(self, motion_manager, action_name,
                 name='L2_Motion_PlayAction'):
        super().__init__(name)
        self.motion_manager = motion_manager
        self._action_name = action_name

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def update(self):
        self.motion_manager.run_action(self._action_name)
        return Status.SUCCESS
