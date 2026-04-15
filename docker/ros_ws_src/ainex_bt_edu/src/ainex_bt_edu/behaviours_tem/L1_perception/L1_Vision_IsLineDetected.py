#!/usr/bin/env python3
"""L1 Condition: SUCCESS if line_data is present on the blackboard."""
import py_trees
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Vision_IsLineDetected(AinexBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.LINE_DATA]

    def __init__(self, name='L1_Vision_IsLineDetected'):
        super().__init__(name)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(key=BB.LINE_DATA, access=Access.READ)

    def update(self):
        val = py_trees.blackboard.Blackboard.storage.get(
            BB.LINE_DATA)
        if val is not None:
            return Status.SUCCESS
        return Status.FAILURE
