#!/usr/bin/env python3
"""L3 Mission: Dynamic color detection config from BB.
Supports both named colors (use_name=True) and custom LAB ranges (use_name=False)."""
import rospy
import py_trees
from py_trees.common import Status
from ainex_interfaces.msg import ColorDetect, ColorsDetect
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L3_Vision_StartColorDetectFromBB(XyzBTNode):
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.TARGET_COLOR]

    IMAGE_SIZE = [160, 120]

    def __init__(self, name='L3_Vision_StartColorDetectFromBB'):
        super().__init__(name)
        self._pub = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._pub = rospy.Publisher('/color_detection/update_detect',
                                    ColorsDetect, queue_size=1)

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage
        param = ColorDetect()
        param.image_process_size = self.IMAGE_SIZE
        param.min_area = 1
        param.max_area = self.IMAGE_SIZE[0] * self.IMAGE_SIZE[1]

        # Check for custom LAB range (use_name=False path)
        lab_min = storage.get(BB.COLOR_LAB_MIN)
        lab_max = storage.get(BB.COLOR_LAB_MAX)

        if lab_min is not None and lab_max is not None:
            param.use_name = False
            param.lab_min = list(lab_min)
            param.lab_max = list(lab_max)
        else:
            color = storage.get(BB.TARGET_COLOR)
            if not color:
                return Status.FAILURE
            param.use_name = True
            param.color_name = color

        # Detect type from BB or default
        detect_type = storage.get(BB.COLOR_DETECT_TYPE, 'circle')
        param.detect_type = detect_type

        # ROI from BB
        roi = storage.get(BB.DETECT_ROI)
        if roi and isinstance(roi, (list, tuple)) and len(roi) >= 4:
            param.roi.x_min = int(roi[0])
            param.roi.y_min = int(roi[1])
            param.roi.x_max = int(roi[2])
            param.roi.y_max = int(roi[3])

        msg = ColorsDetect()
        msg.data = [param]
        self._pub.publish(msg)
        return Status.SUCCESS
