#!/usr/bin/env python3
"""L1 Condition: SUCCESS if battery voltage >= threshold."""
import threading
import rospy
from std_msgs.msg import UInt16
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzBTNode


class L1_Battery_IsVoltageOk(XyzBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = []

    def __init__(self, threshold_mv=11100,
                 name='L1_Battery_IsVoltageOk'):
        super().__init__(name)
        self._threshold_mv = threshold_mv
        self._voltage_mv = 0
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        rospy.Subscriber('/ainex/battery', UInt16, self._callback)

    def _callback(self, msg):
        with self._lock:
            self._voltage_mv = msg.data

    def update(self):
        with self._lock:
            voltage = self._voltage_mv
        if voltage >= self._threshold_mv:
            return Status.SUCCESS
        return Status.FAILURE
