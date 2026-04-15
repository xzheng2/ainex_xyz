#!/usr/bin/env python3
"""L1 Condition: SUCCESS if target_pixel_x < image center (target is on left)."""
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Vision_IsTargetOnLeft(AinexBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.TARGET_PIXEL_X]

    def __init__(self, image_width=160, name='L1_Vision_IsTargetOnLeft'):
        super().__init__(name)
        self._center_x = image_width / 2.0

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def update(self):
        x = py_trees.blackboard.Blackboard.storage.get(
            BB.TARGET_PIXEL_X)
        if x is not None and x < self._center_x:
            return Status.SUCCESS
        return Status.FAILURE
