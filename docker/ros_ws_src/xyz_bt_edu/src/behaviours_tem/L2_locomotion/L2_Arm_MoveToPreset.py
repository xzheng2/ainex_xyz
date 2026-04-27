#!/usr/bin/env python3
"""L2 Action: Move arm servos to a preset position."""
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzBTNode


class L2_Arm_MoveToPreset(XyzBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = []

    def __init__(self, motion_manager, positions, duration=500,
                 name='L2_Arm_MoveToPreset'):
        """
        Args:
            motion_manager: MotionManager instance
            positions: list of [servo_id, pulse] for arm servos (IDs 13-22)
            duration: movement time in ms
        """
        super().__init__(name)
        self.motion_manager = motion_manager
        self._positions = positions
        self._duration = duration

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def update(self):
        self.motion_manager.set_servos_position(
            self._duration, self._positions)
        return Status.SUCCESS
