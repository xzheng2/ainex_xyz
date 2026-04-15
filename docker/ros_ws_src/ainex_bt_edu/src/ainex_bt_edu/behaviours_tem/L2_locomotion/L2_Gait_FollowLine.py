#!/usr/bin/env python3
"""L2 Action: Read line_data from BB and run visual patrol step."""
import py_trees
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L2_Gait_FollowLine(AinexBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.LINE_DATA, BB.ROBOT_STATE]

    def __init__(self, visual_patrol, name='L2_Gait_FollowLine'):
        super().__init__(name)
        self.visual_patrol = visual_patrol
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(key=BB.LINE_DATA, access=Access.READ)

    def update(self):
        line_data = py_trees.blackboard.Blackboard.storage.get(
            BB.LINE_DATA)
        if line_data is None:
            return Status.FAILURE
        self.visual_patrol.process(line_data.x, line_data.width)
        return Status.SUCCESS
