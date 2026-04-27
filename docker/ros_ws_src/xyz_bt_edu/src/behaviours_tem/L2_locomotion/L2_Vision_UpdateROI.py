#!/usr/bin/env python3
"""L2 Action: Update color detection ROI from BB /perception/detect_roi."""
import rospy
import py_trees
from py_trees.common import Status
from ainex_interfaces.msg import ColorDetect, ColorsDetect
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L2_Vision_UpdateROI(XyzBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = []

    def __init__(self, name='L2_Vision_UpdateROI'):
        super().__init__(name)
        self._pub = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._pub = rospy.Publisher('/color_detection/update_detect',
                                    ColorsDetect, queue_size=1)

    def update(self):
        roi = py_trees.blackboard.Blackboard.storage.get(
            BB.DETECT_ROI)
        if roi is None:
            return Status.FAILURE

        param = ColorDetect()
        param.use_name = True
        color = py_trees.blackboard.Blackboard.storage.get(
            BB.TARGET_COLOR, '')
        param.color_name = color
        param.detect_type = 'circle'
        param.image_process_size = [160, 120]

        if isinstance(roi, (list, tuple)) and len(roi) >= 4:
            param.roi.x_min = int(roi[0])
            param.roi.y_min = int(roi[1])
            param.roi.x_max = int(roi[2])
            param.roi.y_max = int(roi[3])

        msg = ColorsDetect()
        msg.data = [param]
        self._pub.publish(msg)
        return Status.SUCCESS
