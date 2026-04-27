#!/usr/bin/env python3
"""L1 Condition: Subscribes to face detection output.
SUCCESS if face detected, FAILURE otherwise."""
import threading
import rospy
import py_trees
from py_trees.common import Status
from ainex_interfaces.msg import ObjectsInfo
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L1_Vision_IsFaceDetected(XyzBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.FACE_DETECTED]

    def __init__(self, topic='/object/pixel_coords',
                 name='L1_Vision_IsFaceDetected'):
        super().__init__(name)
        self._topic = topic
        self._face_detected = False
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        rospy.Subscriber(self._topic, ObjectsInfo, self._callback)

    def _callback(self, msg):
        detected = any(obj.type == 'face' for obj in msg.data)
        with self._lock:
            self._face_detected = detected

    def update(self):
        with self._lock:
            detected = self._face_detected

        py_trees.blackboard.Blackboard.storage[
            BB.FACE_DETECTED] = detected
        if detected:
            return Status.SUCCESS
        return Status.FAILURE
