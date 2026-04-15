#!/usr/bin/env python3
"""L3 Mission: AprilTag detection + head tracking + walk toward tag."""
import threading
import rospy
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB

try:
    from apriltag_ros.msg import AprilTagDetectionArray
    _HAS_APRILTAG = True
except ImportError:
    _HAS_APRILTAG = False


class L3_Mission_TagNavigate(AinexBTNode):
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.TARGET_WORLD_POS]

    IMAGE_CENTER = (320, 240)  # apriltag uses full-res image

    def __init__(self, motion_manager, rl_track, ud_track,
                 gait_manager=None,
                 name='L3_Mission_TagNavigate'):
        super().__init__(name)
        self.motion_manager = motion_manager
        self._rl_track = rl_track
        self._ud_track = ud_track
        self._gait_manager = gait_manager
        self._latest_detection = None
        self._lock = threading.Lock()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        if _HAS_APRILTAG:
            rospy.Subscriber('/tag_detections', AprilTagDetectionArray,
                             self._callback)

    def _callback(self, msg):
        if msg.detections:
            with self._lock:
                self._latest_detection = msg.detections[0]

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage
        with self._lock:
            det = self._latest_detection

        if det is None:
            return Status.RUNNING

        # Extract tag center from corners
        corners = det.corners if hasattr(det, 'corners') else []
        if corners:
            cx = sum(c.x for c in corners) / len(corners)
            cy = sum(c.y for c in corners) / len(corners)
        else:
            pose = det.pose.pose.pose.position
            storage[BB.TARGET_WORLD_POS] = pose
            return Status.SUCCESS

        # PID head tracking
        pan_val = self._rl_track.track(cx, self.IMAGE_CENTER[0])
        tilt_val = self._ud_track.track(cy, self.IMAGE_CENTER[1])
        self.motion_manager.set_servos_position(
            20, [[23, int(pan_val)], [24, int(tilt_val)]])

        return Status.RUNNING

    def terminate(self, new_status):
        self._latest_detection = None
        super().terminate(new_status)
