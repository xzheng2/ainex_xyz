#!/usr/bin/env python3
"""L1_Head_IsHeadCentered — condition: head pan servo is near the centre position.

BB reads:
  BB.HEAD_PAN_POS  (/head_pan_pos)   — root namespace, written by L2_Head_*

BB writes:
  none

Question judged:
  Is the head pan position within center_threshold servo counts of head_pan_center?

Judgement helper:
  _is_centered(head_pan)

SUCCESS:
  abs(head_pan - head_pan_center) <= center_threshold

FAILURE:
  abs(head_pan - head_pan_center) > center_threshold

CONFIG_DEFAULTS:
  head_pan_center:  500  — servo count for the head-forward position.
  center_threshold: 30   — max allowed offset from centre (servo counts).
                           Must match L2_Head_FindLineSweep center_threshold
                           when both nodes are used in the same tree.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_edu.base_node import XyzL1ConditionNode
from xyz_bt_edu.blackboard_keys import BB


class L1_Head_IsHeadCentered(XyzL1ConditionNode):
    """SUCCESS when /head_pan_pos is within center_threshold of head_pan_center."""

    LEVEL = 'L1'
    BB_READS = [BB.HEAD_PAN_POS]
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'head_pan_center':  500,
        'center_threshold': 30,
    }

    def __init__(self, name: str = 'L1_Head_IsHeadCentered',
                 head_pan_center: int = 500, center_threshold: int = 30,
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:             BT node name.
            head_pan_center:  Servo count for the head-forward position.
            center_threshold: Max allowed offset from centre (servo counts).
            logger:           DebugEventLogger-compatible object, or None.
            tick_id_getter:   Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._head_pan_center = head_pan_center
        self._center_threshold = center_threshold
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        # HEAD_PAN_POS lives at root namespace, not /latched/ — no namespace arg.
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(key=BB.HEAD_PAN_POS, access=Access.READ)

    def _is_centered(self, head_pan: int) -> tuple:
        """Return (passed, offset) for the head-centred condition.

        No BB reads/writes, ROS calls, or logger calls here.
        """
        offset = abs(head_pan - self._head_pan_center)
        return offset <= self._center_threshold, offset

    def update(self) -> Status:
        head_pan = self._bb.head_pan_pos
        passed, offset = self._is_centered(head_pan)
        status = self.status_from_bool(passed)

        self.emit_decision(
            inputs={'head_pan_pos': head_pan, 'offset': offset,
                    'center': self._head_pan_center,
                    'threshold': self._center_threshold},
            status=status,
            reason=(f'offset {offset} <= {self._center_threshold}' if passed
                    else f'offset {offset} > {self._center_threshold}'),
        )

        return status
