#!/usr/bin/env python3
"""L3 Action: start or stop a registered background process.

Node kind: finite_action

BB reads:  none
BB writes: none
Facade:    start_process(target) | stop_process(target)

Strategy:  none — dispatches one facade call and returns SUCCESS once.

Use this to start/stop *another* program (e.g. a stand-alone servo script that
is not a ROS node) while the BT node's own launch stays up. The `target` must be
a key in the project ProcessManager registry, wired via the project facade's
start_process/stop_process overrides.

Both facade calls are idempotent (start is a no-op if already running; stop is a
no-op if not running), so this node returns SUCCESS exactly once per activation.
The tree must not re-tick this node continuously (place under a Selector/Sequence
that does not re-enter until SUCCESS propagates), otherwise the process would be
repeatedly (re)started or stopped.

Returns:
  SUCCESS — the start/stop facade call returned.

CONFIG_DEFAULTS:
  target: ''       — registry key of the process to control; must be overridden
                     by the project tree via constructor argument.
  action: 'start'  — 'start' or 'stop'.
"""
from py_trees.common import Status
from xyz_bt_lib.core.base_node import XyzL3ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade


class L3_Process_Control(XyzL3ActionNode):
    """Start or stop a registered background process. Returns SUCCESS once."""

    LEVEL        = 'L3'
    BB_READS     = []
    BB_WRITES    = []
    FACADE_CALLS = ['start_process', 'stop_process']
    CONFIG_DEFAULTS = {
        'target': '',
        'action': 'start',
    }

    _ACTIONS = {
        'start': 'start_process',
        'stop':  'stop_process',
    }

    def __init__(self, name: str = 'L3_Process_Control',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 target: str = '',
                 action: str = 'start'):
        """
        Args:
            target:  Registry key of the process to control. Must be provided by
                     the project tree; empty string is an invalid default that
                     will raise in the facade.
            action:  'start' or 'stop'.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        if action not in self._ACTIONS:
            raise ValueError(
                "L3_Process_Control action must be 'start' or 'stop', got {!r}".format(action))
        self._target = target
        self._action = action

    def initialise(self):
        self.emit_action_intent(
            action=self._action,
            inputs={'target': self._target},
        )

    def update(self) -> Status:
        method = self._ACTIONS[self._action]
        self.call_facade(method, target=self._target)
        self.emit_decision(
            inputs={'target': self._target, 'action': self._action},
            status=Status.SUCCESS,
            reason='{} {!r} dispatched'.format(self._action, self._target),
        )
        return Status.SUCCESS
