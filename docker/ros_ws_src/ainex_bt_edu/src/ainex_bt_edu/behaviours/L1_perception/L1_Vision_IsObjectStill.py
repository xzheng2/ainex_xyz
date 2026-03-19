#!/usr/bin/env python3
"""L1 Condition: SUCCESS if detected object position is stable over N frames."""
import math
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Vision_IsObjectStill(AinexBTNode):
    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.DETECTED_OBJECTS]

    def __init__(self, threshold=2.0, frames=5,
                 name='L1_Vision_IsObjectStill'):
        super().__init__(name)
        self._threshold = threshold
        self._frames = frames
        self._positions = []

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def update(self):
        objects = py_trees.blackboard.Blackboard.storage.get(
            BB.DETECTED_OBJECTS)
        if objects is None or len(objects.data) == 0:
            self._positions.clear()
            return Status.FAILURE

        obj = objects.data[0]
        self._positions.append((obj.x, obj.y))
        if len(self._positions) > self._frames + 1:
            self._positions = self._positions[-(self._frames + 1):]

        if len(self._positions) < self._frames + 1:
            return Status.FAILURE

        for i in range(len(self._positions) - self._frames,
                       len(self._positions) - 1):
            dx = self._positions[i + 1][0] - self._positions[i][0]
            dy = self._positions[i + 1][1] - self._positions[i][1]
            if math.sqrt(dx * dx + dy * dy) >= self._threshold:
                return Status.FAILURE
        return Status.SUCCESS
