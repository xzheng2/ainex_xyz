#!/usr/bin/env python3
"""L3 Mission: PID head tracking for color objects.
Sets up color detection and tracks with head servos."""
import threading
import rospy
import py_trees
from py_trees.common import Status
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L3_Vision_TrackColorObject(XyzBTNode):
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.TARGET_PIXEL_X, BB.TARGET_PIXEL_Y, BB.TARGET_COLOR]

    IMAGE_SIZE = [160, 120]

    def __init__(self, motion_manager, rl_track, ud_track,
                 name='L3_Vision_TrackColorObject'):
        """
        Args:
            motion_manager: MotionManager instance
            rl_track: PIDTrack for pan (servo 23)
            ud_track: PIDTrack for tilt (servo 24)
        """
        super().__init__(name)
        self.motion_manager = motion_manager
        self._rl_track = rl_track
        self._ud_track = ud_track
        self._center_x = self.IMAGE_SIZE[0] / 2.0
        self._center_y = self.IMAGE_SIZE[1] / 2.0
        self._detect_pub = None
        self._configured = False
        self._latest_obj = None
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._detect_pub = rospy.Publisher(
            '/color_detection/update_detect', ColorsDetect, queue_size=1)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo,
                         self._objects_callback)

    def _objects_callback(self, msg):
        for obj in msg.data:
            if obj.type == 'circle':
                with self._lock:
                    self._latest_obj = obj
                return

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage
        color = storage.get(BB.TARGET_COLOR)

        if not self._configured and color:
            param = ColorDetect()
            param.color_name = color
            param.use_name = True
            param.detect_type = 'circle'
            param.image_process_size = self.IMAGE_SIZE
            param.min_area = 1
            param.max_area = self.IMAGE_SIZE[0] * self.IMAGE_SIZE[1]
            msg = ColorsDetect()
            msg.data = [param]
            self._detect_pub.publish(msg)
            self._configured = True

        with self._lock:
            obj = self._latest_obj

        if obj is None:
            return Status.RUNNING

        storage[BB.TARGET_PIXEL_X] = obj.x
        storage[BB.TARGET_PIXEL_Y] = obj.y

        pan_val = self._rl_track.track(obj.x, self._center_x)
        tilt_val = self._ud_track.track(obj.y, self._center_y)
        self.motion_manager.set_servos_position(
            20, [[23, int(pan_val)], [24, int(tilt_val)]])
        return Status.RUNNING

    def terminate(self, new_status):
        self._configured = False
        self._latest_obj = None
        super().terminate(new_status)
