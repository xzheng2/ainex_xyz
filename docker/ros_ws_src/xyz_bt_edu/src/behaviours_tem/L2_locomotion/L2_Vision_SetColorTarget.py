#!/usr/bin/env python3
"""L2 Action: Configure color detection for a named color."""
import rospy
import py_trees
from py_trees.common import Status
from ainex_interfaces.msg import ColorDetect, ColorsDetect
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L2_Vision_SetColorTarget(XyzBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.TARGET_COLOR]

    # Default line ROI (same as marathon_bt_node)
    LINE_ROI = [
        (5 / 12, 6 / 12, 1 / 4, 3 / 4),
        (6 / 12, 7 / 12, 1 / 4, 3 / 4),
        (7 / 12, 8 / 12, 1 / 4, 3 / 4),
    ]
    IMAGE_SIZE = [160, 120]

    def __init__(self, color_name=None, detect_type='circle',
                 name='L2_Vision_SetColorTarget'):
        """
        Args:
            color_name: Fixed color name, or None to read from BB.TARGET_COLOR
            detect_type: 'circle', 'line', etc.
        """
        super().__init__(name)
        self._color_name = color_name
        self._detect_type = detect_type
        self._pub = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._pub = rospy.Publisher('/color_detection/update_detect',
                                    ColorsDetect, queue_size=1)

    def update(self):
        color = self._color_name
        if color is None:
            color = py_trees.blackboard.Blackboard.storage.get(
                BB.TARGET_COLOR)
        if not color:
            return Status.FAILURE

        param = ColorDetect()
        param.color_name = color
        param.use_name = True
        param.detect_type = self._detect_type
        param.image_process_size = self.IMAGE_SIZE
        param.min_area = 1
        param.max_area = self.IMAGE_SIZE[0] * self.IMAGE_SIZE[1]

        if self._detect_type == 'line':
            w, h = self.IMAGE_SIZE
            param.line_roi.up.y_min = int(self.LINE_ROI[0][0] * h)
            param.line_roi.up.y_max = int(self.LINE_ROI[0][1] * h)
            param.line_roi.up.x_min = int(self.LINE_ROI[0][2] * w)
            param.line_roi.up.x_max = int(self.LINE_ROI[0][3] * w)
            param.line_roi.center.y_min = int(self.LINE_ROI[1][0] * h)
            param.line_roi.center.y_max = int(self.LINE_ROI[1][1] * h)
            param.line_roi.center.x_min = int(self.LINE_ROI[1][2] * w)
            param.line_roi.center.x_max = int(self.LINE_ROI[1][3] * w)
            param.line_roi.down.y_min = int(self.LINE_ROI[2][0] * h)
            param.line_roi.down.y_max = int(self.LINE_ROI[2][1] * h)
            param.line_roi.down.x_min = int(self.LINE_ROI[2][2] * w)
            param.line_roi.down.x_max = int(self.LINE_ROI[2][3] * w)

        msg = ColorsDetect()
        msg.data = [param]
        self._pub.publish(msg)
        return Status.SUCCESS
