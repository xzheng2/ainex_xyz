#!/usr/bin/env python3
"""L3 Mission: Subscribe to gesture detection, write label to BB."""
import threading
import rospy
import py_trees
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB

try:
    from ainex_interfaces.msg import FingerPosition
    _HAS_FINGER = True
except ImportError:
    _HAS_FINGER = False


class L3_Gesture_RecognizeAndAct(XyzBTNode):
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.GESTURE_LABEL]

    def __init__(self, topic='~points', name='L3_Gesture_RecognizeAndAct'):
        super().__init__(name)
        self._topic = topic
        self._latest_label = None
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        if _HAS_FINGER:
            rospy.Subscriber(self._topic, FingerPosition, self._callback)

    def _callback(self, msg):
        with self._lock:
            self._latest_label = msg.label

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage
        with self._lock:
            label = self._latest_label

        if label:
            storage[BB.GESTURE_LABEL] = label
            return Status.SUCCESS
        return Status.RUNNING
