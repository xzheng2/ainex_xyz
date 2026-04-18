#!/usr/bin/env python3
"""L1 Condition: check whether line detection data is present on the blackboard.

Reads /latched/line_data from the blackboard.
SUCCESS  → line_data is not None
FAILURE  → line_data is None (line lost or detector not running)

Reference implementation: marathon/behaviours/conditions.py :: IsLineDetected
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Vision_IsLineDetected(AinexBTNode):
    """SUCCESS if /latched/line_data is present (not None)."""

    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.LINE_DATA]

    def __init__(self, name: str = 'L1_Vision_IsLineDetected',
                 logger=None, tick_id_getter=None):
        super().__init__(name)
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.LINE_DATA_KEY, access=Access.READ)

    def update(self) -> Status:
        line_data = self._bb.line_data
        passed = (line_data is not None)
        status = Status.SUCCESS if passed else Status.FAILURE

        if self._logger:
            inputs = {'line_detected': passed}
            if passed:
                inputs['line_x'] = line_data.x
                inputs['line_width'] = line_data.width
            self._logger.emit_bt({
                'event': 'decision',
                'node': self.name,
                'inputs': inputs,
                'status': str(status),
                'reason': 'line_data present' if passed else 'line_data is None',
            })

        return status
