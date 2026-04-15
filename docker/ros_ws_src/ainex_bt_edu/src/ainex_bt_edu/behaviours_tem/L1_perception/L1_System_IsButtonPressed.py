#!/usr/bin/env python3
"""L1 Condition: SUCCESS if button is pressed."""
import threading
import rospy
from std_msgs.msg import Bool
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode


class L1_System_IsButtonPressed(AinexBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = []

    def __init__(self, topic='/sensor/button/get_button_state',
                 name='L1_System_IsButtonPressed'):
        super().__init__(name)
        self._topic = topic
        self._pressed = False
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        rospy.Subscriber(self._topic, Bool, self._callback)

    def _callback(self, msg):
        with self._lock:
            self._pressed = msg.data

    def update(self):
        with self._lock:
            pressed = self._pressed
        if pressed:
            return Status.SUCCESS
        return Status.FAILURE
