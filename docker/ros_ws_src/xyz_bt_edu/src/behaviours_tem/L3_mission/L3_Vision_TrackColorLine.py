#!/usr/bin/env python3
"""L3 Mission: Configure color detection for line following from BB target_color.
Encapsulates the full _set_color logic from marathon_bt_node."""
import rospy
import py_trees
from py_trees.common import Status
from ainex_interfaces.msg import ColorDetect, ColorsDetect
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L3_Vision_TrackColorLine(XyzBTNode):
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.TARGET_COLOR, BB.LINE_DATA]

    LINE_ROI = [
        (5 / 12, 6 / 12, 1 / 4, 3 / 4),
        (6 / 12, 7 / 12, 1 / 4, 3 / 4),
        (7 / 12, 8 / 12, 1 / 4, 3 / 4),
    ]
    IMAGE_SIZE = [160, 120]

    def __init__(self, name='L3_Vision_TrackColorLine'):
        super().__init__(name)
        self._pub = None
        self._configured = False

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._pub = rospy.Publisher('/color_detection/update_detect',
                                    ColorsDetect, queue_size=1)

    def update(self):
        color = py_trees.blackboard.Blackboard.storage.get(
            BB.TARGET_COLOR)
        if not color:
            return Status.FAILURE

        if not self._configured:
            self._set_color(color)
            self._configured = True

        return Status.SUCCESS

    def _set_color(self, color_name):
        w, h = self.IMAGE_SIZE
        param = ColorDetect()
        param.color_name = color_name
        param.use_name = True
        param.detect_type = 'line'
        param.image_process_size = self.IMAGE_SIZE
        param.min_area = 1
        param.max_area = w * h

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
        rospy.loginfo('[L3_Vision_TrackColorLine] set color: %s', color_name)

    def terminate(self, new_status):
        self._configured = False
        super().terminate(new_status)
