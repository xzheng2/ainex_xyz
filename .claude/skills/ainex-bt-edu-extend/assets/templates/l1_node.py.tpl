#!/usr/bin/env python3
"""{{CLASS_NAME}} - {{DESCRIPTION}}

L1 condition node.

BB reads:
  {{BB_READS}}

BB writes:
  none

Question judged:
  {{DESCRIPTION}}

Judgement helper:
  _evaluate(...)

SUCCESS:
  TODO: document the exact condition that returns Status.SUCCESS.

FAILURE:
  TODO: document the exact condition that returns Status.FAILURE.

Constructor defaults:
  TODO: list every threshold, expected state/label, center value, tolerance,
  frame count, etc. Project trees may override these constructor defaults.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexL1ConditionNode
from ainex_bt_edu.blackboard_keys import BB


class {{CLASS_NAME}}(AinexL1ConditionNode):
    """{{DESCRIPTION}}"""

    LEVEL = 'L1'
    BB_READS = {{BB_READS}}
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        # TODO: replace with explicit documented defaults, e.g.
        # 'expected_state': 'stand',
        # 'threshold': 0,
    }
    BB_LOG_KEYS = BB_READS

    def __init__(self, name: str = {{DEFAULT_NAME}},
                 logger=None, tick_id_getter=None):
        """
        Args:
            name: BT node name.
            logger: DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.

        TODO: add explicit constructor default args for every judgement setting
        instead of hard-coding constants in update().
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._bb = None

        # TODO: store constructor defaults on self, e.g.
        # self._expected_state = expected_state
        # self._threshold = threshold

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        # Register every BB key this node reads. Use BB.*_KEY for /latched keys.
        # Example:
        # self._bb.register_key(key=BB.SOME_KEY, access=Access.READ)
        raise NotImplementedError("Fill in Access.READ register_key() calls")

    def _evaluate(self, *values) -> tuple:
        """Return (passed, reason) for the documented L1 condition.

        This helper must be side-effect-free:
        - no BB reads/writes
        - no facade calls
        - no ROS calls
        - no logger calls
        """
        raise NotImplementedError("Fill in side-effect-free judgement logic")

    def update(self) -> Status:
        # Read BB values only, then call _evaluate().
        # Example:
        # value = self._bb.some_key
        # passed, reason = self._evaluate(value)
        # inputs = {'some_key': value}
        raise NotImplementedError("Fill in BB reads and _evaluate() call")

        status = Status.SUCCESS if passed else Status.FAILURE

        self.emit_decision(
            inputs=inputs,
            status=status,
            reason=reason,
        )

        return status
