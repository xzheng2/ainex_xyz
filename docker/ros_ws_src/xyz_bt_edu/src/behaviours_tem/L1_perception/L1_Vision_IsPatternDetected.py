#!/usr/bin/env python3
"""L1 Condition: SUCCESS if detected_objects contains an object of given type."""
import py_trees
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L1_Vision_IsPatternDetected(XyzBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.DETECTED_OBJECTS]

    def __init__(self, pattern_type='circle',
                 name='L1_Vision_IsPatternDetected'):
        super().__init__(name)
        self._pattern_type = pattern_type

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def update(self):
        objects = py_trees.blackboard.Blackboard.storage.get(
            BB.DETECTED_OBJECTS)
        if objects is not None:
            for obj in objects.data:
                if obj.type == self._pattern_type:
                    return Status.SUCCESS
        return Status.FAILURE
