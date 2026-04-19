#!/usr/bin/env python3
"""L3 Mission: PID head tracking for faces."""
import threading
import rospy
import py_trees
from py_trees.common import Status
from ainex_interfaces.msg import ObjectsInfo
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L3_Vision_TrackFace(AinexBTNode):
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.FACE_DETECTED, BB.TARGET_PIXEL_X, BB.TARGET_PIXEL_Y]

    IMAGE_SIZE = [160, 120]

    def __init__(self, motion_manager, rl_track, ud_track,
                 name='L3_Vision_TrackFace'):
        super().__init__(name)
        self.motion_manager = motion_manager
        self._rl_track = rl_track
        self._ud_track = ud_track
        self._center_x = self.IMAGE_SIZE[0] / 2.0
        self._center_y = self.IMAGE_SIZE[1] / 2.0
        self._latest_face = None
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo,
                         self._callback)

    def _callback(self, msg):
        for obj in msg.data:
            if obj.type == 'face':
                with self._lock:
                    self._latest_face = obj
                return
        with self._lock:
            self._latest_face = None

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage
        with self._lock:
            face = self._latest_face

        if face is None:
            storage[BB.FACE_DETECTED] = False
            return Status.RUNNING

        storage[BB.FACE_DETECTED] = True
        storage[BB.TARGET_PIXEL_X] = face.x
        storage[BB.TARGET_PIXEL_Y] = face.y

        pan_val = self._rl_track.track(face.x, self._center_x)
        tilt_val = self._ud_track.track(face.y, self._center_y)
        self.motion_manager.set_servos_position(
            20, [[23, int(pan_val)], [24, int(tilt_val)]])
        return Status.RUNNING

    def terminate(self, new_status):
        self._latest_face = None
        super().terminate(new_status)
