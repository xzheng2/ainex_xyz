#!/usr/bin/env python3
"""L2 Action: Start walking. Always returns SUCCESS."""
import rospy
from ainex_interfaces.srv import SetWalkingCommand
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L2_Gait_Start(XyzBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.GAIT_ENABLED]

    def __init__(self, name='L2_Gait_Start'):
        super().__init__(name)

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._cmd_srv = rospy.ServiceProxy('walking/command',
                                           SetWalkingCommand)

    def update(self):
        self._cmd_srv('start')
        return Status.SUCCESS
