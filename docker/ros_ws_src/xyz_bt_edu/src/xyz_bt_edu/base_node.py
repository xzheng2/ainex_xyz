#!/usr/bin/env python3
"""Base classes for AiNex behaviour-tree nodes.

The observable-BT runtime owns global tick logging through BTDebugVisitor.
These classes only provide node-side helpers for structured semantic events
and keep the L1/L2 responsibility boundary explicit.
"""
import py_trees
from py_trees.common import Status


class XyzBTNode(py_trees.behaviour.Behaviour):
    """Common base for AiNex BT behaviours.

    This class intentionally does not override ``tick()`` and does not publish
    ``/bt_node_events``. Per-node tick status is captured by BTDebugVisitor.
    Subclasses can use ``emit_bt()`` / ``emit_decision()`` for optional semantic
    events that are not available from the py_trees visitor alone.
    """

    LEVEL = 'BT'
    BB_READS = []
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {}

    # Compatibility alias for existing nodes and older docs/templates.
    BB_LOG_KEYS = []

    def __init__(self, name: str, logger=None, tick_id_getter=None):
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._session_id = 'unknown'
        super().__init__(name)

    def setup(self, **kwargs):
        """Compatibility no-op.

        Existing nodes call ``super().setup(**kwargs)``. Observable BT logging is
        mounted at the tree level, so the base node has no ROS publishers or
        blackboard clients to create here.
        """
        pass

    @property
    def tick_id(self):
        """Return the current externally-owned BT tick id."""
        return self._tick_id_getter()

    def emit_bt(self, payload: dict) -> None:
        """Emit a BT-layer semantic event if a logger was injected."""
        if self._logger is None:
            return
        event = dict(payload)
        event.setdefault('tick_id', self.tick_id)
        event.setdefault('node', self.name)
        event.setdefault('level', self.LEVEL)
        if self._session_id != 'unknown':
            event.setdefault('session_id', self._session_id)
        self._logger.emit_bt(event)

    def emit_decision(self, *, inputs, status, reason, **extra) -> None:
        """Emit a standard decision event.

        Args:
            inputs: Structured input values used by this node's decision.
            status: py_trees Status or status string.
            reason: Human-readable reason for the selected status.
            **extra: Optional additional JSON-serializable fields.
        """
        event = {
            'event': 'decision',
            'inputs': inputs,
            'status': getattr(status, 'name', str(status)),
            'reason': reason,
        }
        event.update(extra)
        self.emit_bt(event)


class XyzL1ConditionNode(XyzBTNode):
    """Base class for L1 condition nodes.

    L1 nodes read blackboard state and normally return SUCCESS or FAILURE.
    They must not dispatch actions, call facades, publish/subscribe ROS, or
    emit communication-layer events.
    """

    LEVEL = 'L1'
    BB_WRITES = []
    FACADE_CALLS = []

    @staticmethod
    def status_from_bool(passed: bool) -> Status:
        return Status.SUCCESS if passed else Status.FAILURE


class XyzL2ActionNode(XyzBTNode):
    """Base class for L2 action/strategy nodes.

    L2 nodes may read/write documented blackboard keys and dispatch side effects
    only through an injected XyzBTFacade implementation.
    """

    LEVEL = 'L2'

    def __init__(self, name: str, logger=None, tick_id_getter=None, facade=None):
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._facade = facade

    def emit_action_intent(self, *, action, inputs=None, **extra) -> None:
        """Emit a standard action_intent event for action start observability."""
        event = {
            'event': 'action_intent',
            'action': action,
            'inputs': inputs or {},
        }
        event.update(extra)
        self.emit_bt(event)

    def call_facade(self, method_name: str, **kwargs):
        """Call a facade method with standard BT attribution fields."""
        if self._facade is None:
            raise RuntimeError(
                '{} requires a facade before calling {}'.format(
                    self.name, method_name))
        kwargs.setdefault('bt_node', self.name)
        kwargs.setdefault('tick_id', self.tick_id)
        return getattr(self._facade, method_name)(**kwargs)
