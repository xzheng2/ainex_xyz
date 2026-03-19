#!/usr/bin/env python3
"""L1 Condition: Subscribes to /object/pixel_coords, writes BB detected_objects.
SUCCESS if objects detected, FAILURE otherwise."""
import threading
import rospy
import py_trees
from py_trees.common import Access, Status
from ainex_interfaces.msg import ObjectsInfo
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Vision_IsObjectDetected(AinexBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.DETECTED_OBJECTS]

    def __init__(self, name='L1_Vision_IsObjectDetected'):
        super().__init__(name)
        self._latest_msg = None
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo,
                         self._objects_callback)

    def _objects_callback(self, msg):
        with self._lock:
            self._latest_msg = msg

    def update(self):
        with self._lock:
            msg = self._latest_msg

        if msg is not None and len(msg.data) > 0:
            py_trees.blackboard.Blackboard.storage[
                BB.DETECTED_OBJECTS] = msg
            return Status.SUCCESS
        py_trees.blackboard.Blackboard.storage[
            BB.DETECTED_OBJECTS] = None
        return Status.FAILURE
