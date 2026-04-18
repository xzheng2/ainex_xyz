#!/usr/bin/env python3
"""L1 Condition: check whether the first detected object has stopped moving.

Reads BB.DETECTED_OBJECTS (ObjectsInfo) from the blackboard.
Tracks the centroid of objects.data[0] across N consecutive ticks.

SUCCESS  → object has been within `threshold` pixels of its previous position
           for at least `frames` consecutive ticks
FAILURE  → no object detected, fewer than `frames` samples, or object still moving

Requires BB.DETECTED_OBJECTS to be populated by an ObjectDetectionAdapter.
"""
import math
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB

_PERCEPTION_NS = '/perception'
_DETECTED_OBJECTS_KEY = 'detected_objects'


class L1_Vision_IsObjectStill(AinexBTNode):
    """SUCCESS if the first detected object's position is stable across N ticks."""

    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.DETECTED_OBJECTS]

    def __init__(self, threshold: float = 2.0, frames: int = 5,
                 name: str = 'L1_Vision_IsObjectStill',
                 logger=None, tick_id_getter=None):
        super().__init__(name)
        self._threshold = threshold
        self._frames = frames
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None
        self._positions = []

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=_PERCEPTION_NS)
        self._bb.register_key(key=_DETECTED_OBJECTS_KEY, access=Access.READ)

    def update(self) -> Status:
        objects = self._bb.detected_objects
        if objects is None or len(objects.data) == 0:
            self._positions.clear()
            status = Status.FAILURE
            reason = 'no objects detected'
        else:
            obj = objects.data[0]
            self._positions.append((obj.x, obj.y))
            # Keep only the last (frames + 1) positions
            if len(self._positions) > self._frames + 1:
                self._positions = self._positions[-(self._frames + 1):]

            if len(self._positions) < self._frames + 1:
                status = Status.FAILURE
                reason = f'accumulating samples ({len(self._positions)}/{self._frames + 1})'
            else:
                moving = any(
                    math.sqrt(
                        (self._positions[i + 1][0] - self._positions[i][0]) ** 2 +
                        (self._positions[i + 1][1] - self._positions[i][1]) ** 2
                    ) >= self._threshold
                    for i in range(len(self._positions) - self._frames,
                                   len(self._positions) - 1)
                )
                status = Status.FAILURE if moving else Status.SUCCESS
                reason = 'object moving' if moving else 'object still'

        if self._logger:
            self._logger.emit_bt({
                'event':  'decision',
                'node':   self.name,
                'inputs': {'samples': len(self._positions),
                           'threshold': self._threshold},
                'status': str(status),
                'reason': reason,
            })

        return status
