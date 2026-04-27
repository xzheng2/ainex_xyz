#!/usr/bin/env python3
"""L2 Action: Play action whose name comes from BB /mission/action_name."""
import os
import rospy
import py_trees
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L2_Motion_PlayFromBB(XyzBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.ACTION_NAME]

    ACTION_PATH = '/home/ubuntu/software/ainex_controller/ActionGroups'

    def __init__(self, motion_manager, name='L2_Motion_PlayFromBB'):
        super().__init__(name)
        self.motion_manager = motion_manager

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def update(self):
        action_name = py_trees.blackboard.Blackboard.storage.get(
            BB.ACTION_NAME)
        if not action_name:
            rospy.logwarn('[L2_Motion_PlayFromBB] no action_name on BB')
            return Status.FAILURE

        path = os.path.join(self.ACTION_PATH, action_name + '.d6a')
        if not os.path.isfile(path):
            rospy.logwarn('[L2_Motion_PlayFromBB] action not found: %s',
                          action_name)
            return Status.FAILURE

        self.motion_manager.run_action(action_name)
        return Status.SUCCESS
