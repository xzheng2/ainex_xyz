#!/usr/bin/env python3
"""L2 Action: execute a named stand-alone motion action (blocking).

Node kind: finite_action

BB reads:  none
BB writes: none
Facade:    run_action(action_name)

Strategy:  none — calls run_action directly; facade is blocking.

run_action is a blocking call (defined in XyzBTFacade contract). The node
returns SUCCESS exactly once: after run_action returns. The tree must not
re-tick this node during execution (use a Selector / Sequence that does not
re-enter until SUCCESS propagates).

Returns:
  SUCCESS — action completed (run_action returned)

CONFIG_DEFAULTS:
  action_name: ''  — name of the motion action to execute; must be overridden
                     by the project tree via constructor argument.
"""
from py_trees.common import Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade


class L2_Motion_RunAction(XyzL2ActionNode):
    """Execute a named motion action. Returns SUCCESS when the action completes."""

    LEVEL        = 'L2'
    BB_READS     = []
    BB_WRITES    = []
    FACADE_CALLS = ['run_action']
    CONFIG_DEFAULTS = {
        'action_name': '',
    }

    def __init__(self, name: str = 'L2_Motion_RunAction',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 action_name: str = ''):
        """
        Args:
            action_name:  Name of the motion action to execute.
                          Must be provided by the project tree; empty string is
                          an invalid default that will raise in the facade.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._action_name = action_name

    def initialise(self):
        self.emit_action_intent(
            action='run_action',
            inputs={'action_name': self._action_name},
        )

    def update(self) -> Status:
        self.call_facade('run_action', action_name=self._action_name)
        self.emit_decision(
            inputs={'action_name': self._action_name},
            status=Status.SUCCESS,
            reason=f'action {self._action_name!r} completed',
        )
        return Status.SUCCESS
