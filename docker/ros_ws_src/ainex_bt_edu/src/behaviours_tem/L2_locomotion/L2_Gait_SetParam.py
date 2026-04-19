#!/usr/bin/env python3
"""L2 Action: Publish walking parameters from BB to /app/set_walking_param."""
import rospy
import py_trees
from py_trees.common import Status
from ainex_interfaces.msg import AppWalkingParam
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L2_Gait_SetParam(AinexBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.WALK_X, BB.WALK_Y, BB.WALK_ANGLE, BB.WALK_BODY_HEIGHT]

    def __init__(self, speed=1, name='L2_Gait_SetParam'):
        super().__init__(name)
        self._speed = speed
        self._pub = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._pub = rospy.Publisher('/app/set_walking_param',
                                    AppWalkingParam, queue_size=1)

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage
        msg = AppWalkingParam()
        msg.speed = self._speed
        msg.x = float(storage.get(BB.WALK_X, 0.0))
        msg.y = float(storage.get(BB.WALK_Y, 0.0))
        msg.angle = float(storage.get(BB.WALK_ANGLE, 0.0))
        msg.height = float(storage.get(
            BB.WALK_BODY_HEIGHT, 0.0))
        self._pub.publish(msg)
        return Status.SUCCESS
