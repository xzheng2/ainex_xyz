#!/usr/bin/env python3
"""L1_Balance_IsStanding — condition: robot is in the standing state.

BB reads:
  BB.ROBOT_STATE  (/latched/robot_state)

BB writes:
  none

Question judged:
  Is the robot currently standing upright?

Judgement helper:
  _is_standing(state)

Behaviour (pure predicate):
  SUCCESS:  robot_state == expected_stand_label  (default: 'stand')
  FAILURE:  any other state (lie, recline, …)

For "standing stable for N ticks", wrap this node at the tree layer in
xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator — keep this node stateless.

CONFIG_DEFAULTS:
  expected_stand_label: 'stand'  — BB value that represents the upright state.
  Project trees may override this via constructor args.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Balance_IsStanding(XyzL1ConditionNode):
    """SUCCESS if /latched/robot_state == expected_stand_label."""

    LEVEL = 'L1'
    BB_READS = [BB.ROBOT_STATE]
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'expected_stand_label': 'stand',
    }

    def __init__(self, name: str = 'L1_Balance_IsStanding',
                 expected_stand_label: str = 'stand',
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:                 BT node name.
            expected_stand_label: BB robot_state value that means upright.
            logger:               DebugEventLogger-compatible object, or None.
            tick_id_getter:       Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._expected_stand_label = expected_stand_label
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.ROBOT_STATE_KEY, access=Access.READ)

    def _is_standing(self, state: str) -> bool:
        """Return True when robot_state matches the expected standing label."""
        return state == self._expected_stand_label

    def update(self) -> Status:
        state = self._bb.robot_state
        passed = self._is_standing(state)
        inputs = {'robot_state': state,
                  'expected_stand_label': self._expected_stand_label}

        status = self.status_from_bool(passed)
        reason = (f"robot_state == '{self._expected_stand_label}'" if passed
                  else f"robot_state == '{state}'")

        self.emit_decision(inputs=inputs, status=status, reason=reason)
        return status
