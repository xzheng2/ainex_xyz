#!/usr/bin/env python3
"""L1_Vision_IsObjectStill — condition: first detected object is stationary.

BB reads:
  BB.DETECTED_OBJECTS  (/latched/detected_objects)
    Written by ObjectDetectionAdapter as a plain list[ObjectInfo].

BB writes:
  none

Question judged:
  Has the first detected object remained within `threshold` pixels of its
  previous position for at least `frames` consecutive ticks?

Judgement helper:
  _evaluate_stillness(positions, threshold, frames)

SUCCESS:
  object centroid has moved < threshold pixels for all consecutive pairs
  across the last `frames` ticks (i.e. frames+1 positions accumulated)

FAILURE:
  no object detected, fewer than frames+1 samples accumulated,
  or any consecutive pair exceeds the threshold

CONFIG_DEFAULTS:
  threshold: 2.0  — max Euclidean pixel distance to classify as 'still'.
  frames:    5    — consecutive tick count required for SUCCESS.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
import math
from py_trees.common import Access, Status
from xyz_bt_edu.base_node import XyzL1ConditionNode
from xyz_bt_edu.blackboard_keys import BB


class L1_Vision_IsObjectStill(XyzL1ConditionNode):
    """SUCCESS if the first detected object's position is stable across N ticks."""

    LEVEL = 'L1'
    BB_READS = [BB.DETECTED_OBJECTS]
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'threshold': 2.0,
        'frames':    5,
    }

    def __init__(self, name: str = 'L1_Vision_IsObjectStill',
                 threshold: float = 2.0, frames: int = 5,
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:           BT node name.
            threshold:      Max Euclidean pixel distance to classify as 'still'.
            frames:         Consecutive tick count required for SUCCESS.
            logger:         DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._threshold = threshold
        self._frames = frames
        self._positions = []  # list of (x, y) tuples, one per tick
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.DETECTED_OBJECTS_KEY, access=Access.READ)

    def _evaluate_stillness(self, positions: list) -> tuple:
        """Return (passed, reason) for the stillness condition.

        positions: list of (x, y) tuples, length == frames+1.
        Checks all consecutive pairs across the last `frames` steps.
        No BB reads/writes, ROS calls, or logger calls here.
        """
        for i in range(len(positions) - self._frames, len(positions) - 1):
            dx = positions[i + 1][0] - positions[i][0]
            dy = positions[i + 1][1] - positions[i][1]
            if math.sqrt(dx * dx + dy * dy) >= self._threshold:
                return False, 'object moving'
        return True, 'object still'

    def update(self) -> Status:
        objects = self._bb.detected_objects  # list[ObjectInfo] or []
        if not objects:
            self._positions.clear()
            status = Status.FAILURE
            reason = 'no objects detected'
        else:
            obj = objects[0]
            self._positions.append((obj.x, obj.y))
            if len(self._positions) > self._frames + 1:
                self._positions = self._positions[-(self._frames + 1):]

            if len(self._positions) < self._frames + 1:
                status = Status.FAILURE
                reason = (f'accumulating samples '
                          f'({len(self._positions)}/{self._frames + 1})')
            else:
                passed, reason = self._evaluate_stillness(self._positions)
                status = self.status_from_bool(passed)

        self.emit_decision(
            inputs={'samples': len(self._positions),
                    'threshold': self._threshold,
                    'frames': self._frames},
            status=status,
            reason=reason,
        )

        return status
