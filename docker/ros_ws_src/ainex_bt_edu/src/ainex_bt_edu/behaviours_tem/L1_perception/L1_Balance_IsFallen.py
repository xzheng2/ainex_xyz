#!/usr/bin/env python3
"""L1 Condition: Detects falls via IMU. Writes robot_state to BB.
SUCCESS when a fall is detected, FAILURE when standing."""
import math
import threading
import rospy
from sensor_msgs.msg import Imu
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Balance_IsFallen(AinexBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.ROBOT_STATE]
    FALL_COUNT_THRESHOLD = 100

    def __init__(self, name='L1_Balance_IsFallen'):
        super().__init__(name)
        self._bb = None
        self._count_lie = 0
        self._count_recline = 0
        self._latest_imu = None
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(key=BB.ROBOT_STATE, access=Access.WRITE)
        rospy.Subscriber('/imu', Imu, self._imu_callback)

    def _imu_callback(self, msg):
        with self._lock:
            self._latest_imu = msg

    def update(self):
        with self._lock:
            imu = self._latest_imu

        if imu is None:
            return Status.FAILURE

        current_state = self._bb.get(BB.ROBOT_STATE, 'stand')
        if current_state != 'stand':
            return Status.SUCCESS

        angle = abs(int(math.degrees(
            math.atan2(imu.linear_acceleration.y,
                       imu.linear_acceleration.z))))

        if angle < 30:
            self._count_lie += 1
        else:
            self._count_lie = 0

        if angle > 150:
            self._count_recline += 1
        else:
            self._count_recline = 0

        if self._count_lie > self.FALL_COUNT_THRESHOLD:
            self._count_lie = 0
            self._count_recline = 0
            setattr(self._bb, BB.ROBOT_STATE.replace('/', '.'),
                    'lie_to_stand')
            py_trees.blackboard.Blackboard.storage[
                BB.ROBOT_STATE] = 'lie_to_stand'
            return Status.SUCCESS
        elif self._count_recline > self.FALL_COUNT_THRESHOLD:
            self._count_lie = 0
            self._count_recline = 0
            py_trees.blackboard.Blackboard.storage[
                BB.ROBOT_STATE] = 'recline_to_stand'
            return Status.SUCCESS

        return Status.FAILURE
