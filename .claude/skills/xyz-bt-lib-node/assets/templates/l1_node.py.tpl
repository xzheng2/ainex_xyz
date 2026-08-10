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

CONFIG_DEFAULTS:
  TODO: list every threshold, expected state/label, center value, tolerance,
  frame count, etc. Project trees may override these via constructor args.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class {{CLASS_NAME}}(XyzL1ConditionNode):
    """{{DESCRIPTION}}"""

    LEVEL = 'L1'
    BB_READS = {{BB_READS}}
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'expected_state': 'stand',      # example: expected robot state or label
        'threshold': 0,                 # example: numeric threshold for condition
        # TODO: add all judgement thresholds, expected states/labels, centres,
        # tolerances here. These must match __init__ defaults.
        # Do NOT add dwell/hysteresis knobs: L1 is a pure predicate. For
        # "condition stable for N ticks", wrap this node at the TREE layer in
        # xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator.
    }
    BB_LOG_KEYS = BB_READS

    # Param order: name first, then domain params, then logger, tick_id_getter.
    def __init__(self, name: str = {{DEFAULT_NAME}},
                 expected_state: str = 'stand',
                 threshold: int = 0,
                 logger=None, tick_id_getter=None):
        """
        Args:
            name: BT node name.
            expected_state: Expected robot state or label for condition.
            threshold: Numeric threshold for condition judgement.
            logger: DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.

        Every CONFIG_DEFAULTS entry must have a matching __init__ arg stored on self._.
        Runtime logic must use self._ fields, not raw literals.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._bb = None
        self._expected_state = expected_state
        self._threshold = threshold

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        # Register every BB key this node reads. Use BB.*_KEY for /latched keys.
        # Example:
        # self._bb.register_key(key=BB.SOME_KEY, access=Access.READ)
        # TODO(DELETE THIS LINE once the register_key() calls above are filled in)
        pass

    def _evaluate(self, *values) -> tuple:
        """Return (passed, reason) for the documented L1 condition.

        This helper must be side-effect-free:
        - no BB reads/writes
        - no facade calls
        - no ROS calls
        - no logger calls

        Use self._ instance fields (self._threshold, self._expected_state, etc.),
        not raw literals. Hard-coded literals are a conformance violation.
        """
        # TODO(REPLACE): compute the real judgement. Placeholder keeps the
        # unedited template syntactically complete and every name defined.
        passed, reason = False, 'not implemented'
        return passed, reason

    def update(self) -> Status:
        # Read BB values only, then call _evaluate().
        # Example:
        # value = self._bb.some_key
        # passed, reason = self._evaluate(value)
        # inputs = {'some_key': value}
        # TODO(REPLACE): read the BB value(s) and call the judgement helper, e.g.
        #   value = self._bb.some_key
        #   passed, reason = self._evaluate(value)
        #   inputs = {'some_key': value}
        value = None
        passed, reason = self._evaluate(value)
        inputs = {}

        # L1 is a pure predicate: status_from_bool() returns SUCCESS or FAILURE
        # only — never RUNNING. Do NOT add cross-tick state here; timing/dwell
        # belongs to a tree-layer LatchedDwellDecorator.
        status = self.status_from_bool(passed)

        self.emit_decision(
            inputs=inputs,
            status=status,
            reason=reason,
        )

        return status
