#!/usr/bin/env python3
"""L1 Condition: SUCCESS if robot_state == 'stand'."""
import py_trees
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Balance_IsStanding(AinexBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.ROBOT_STATE]

    def __init__(self, name='L1_Balance_IsStanding'):
        super().__init__(name)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(key=BB.ROBOT_STATE, access=Access.READ)

    def update(self):
        if self._bb.get(BB.ROBOT_STATE) == 'stand':
            return Status.SUCCESS
        return Status.FAILURE
