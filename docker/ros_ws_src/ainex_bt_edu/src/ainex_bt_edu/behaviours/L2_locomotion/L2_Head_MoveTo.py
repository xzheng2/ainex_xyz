#!/usr/bin/env python3
"""L2 Action: Move head to fixed pan/tilt position."""
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode


class L2_Head_MoveTo(AinexBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = []

    def __init__(self, motion_manager, pan=500, tilt=500, duration=200,
                 name='L2_Head_MoveTo'):
        super().__init__(name)
        self.motion_manager = motion_manager
        self._pan = pan
        self._tilt = tilt
        self._duration = duration

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def update(self):
        self.motion_manager.set_servos_position(
            self._duration, [[23, self._pan], [24, self._tilt]])
        return Status.SUCCESS
