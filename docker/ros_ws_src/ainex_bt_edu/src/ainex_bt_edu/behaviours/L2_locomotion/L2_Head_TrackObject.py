#!/usr/bin/env python3
"""L2 Action: PID head tracking from BB target_pixel_x/y."""
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L2_Head_TrackObject(AinexBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.TARGET_PIXEL_X, BB.TARGET_PIXEL_Y]

    def __init__(self, motion_manager, rl_track, ud_track,
                 image_width=160, image_height=120,
                 name='L2_Head_TrackObject'):
        """
        Args:
            motion_manager: MotionManager instance
            rl_track: PIDTrack for pan (servo 23)
            ud_track: PIDTrack for tilt (servo 24)
            image_width: detection image width (pixels)
            image_height: detection image height (pixels)
        """
        super().__init__(name)
        self.motion_manager = motion_manager
        self._rl_track = rl_track
        self._ud_track = ud_track
        self._center_x = image_width / 2.0
        self._center_y = image_height / 2.0

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage
        px = storage.get(BB.TARGET_PIXEL_X)
        py_val = storage.get(BB.TARGET_PIXEL_Y)

        if px is None or py_val is None:
            return Status.FAILURE

        pan_val = self._rl_track.track(px, self._center_x)
        tilt_val = self._ud_track.track(py_val, self._center_y)

        self.motion_manager.set_servos_position(
            20, [[23, int(pan_val)], [24, int(tilt_val)]])
        return Status.RUNNING
