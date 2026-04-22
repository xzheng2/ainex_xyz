#!/usr/bin/env python3
"""L1_Vision_IsLineDetected — condition: line detection data is present.

BB reads:
  BB.LINE_DATA  (/latched/line_data)

BB writes:
  none

Question judged:
  Is line detection data currently available on the blackboard?

Judgement helper:
  _is_detected(line_data)

SUCCESS:
  line_data is not None

FAILURE:
  line_data is None  (line lost or detector not running)

Constructor defaults:
  none

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexL1ConditionNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Vision_IsLineDetected(AinexL1ConditionNode):
    """SUCCESS if /latched/line_data is not None."""

    LEVEL = 'L1'
    BB_READS = [BB.LINE_DATA]
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {}

    def __init__(self, name: str = 'L1_Vision_IsLineDetected',
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:           BT node name.
            logger:         DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.LINE_DATA_KEY, access=Access.READ)

    def _is_detected(self, line_data) -> bool:
        """Return True when line_data is not None.

        No BB reads/writes, ROS calls, or logger calls here.
        """
        return line_data is not None

    def update(self) -> Status:
        line_data = self._bb.line_data
        passed = self._is_detected(line_data)
        status = self.status_from_bool(passed)

        inputs = {'line_detected': passed}
        if passed:
            inputs['line_x'] = line_data.x
            inputs['line_width'] = line_data.width

        self.emit_decision(
            inputs=inputs,
            status=status,
            reason='line_data present' if passed else 'line_data is None',
        )

        return status
