#!/usr/bin/env python3
"""L1_Balance_IsFallen — condition: robot is not in the standing state.

BB reads:
  BB.ROBOT_STATE  (/latched/robot_state)

BB writes:
  none

Question judged:
  Is the robot in a fallen or transitional state (not standing upright)?

Judgement helper:
  _is_fallen(state)

SUCCESS:
  robot_state != expected_stand_label  ('lie' or 'recline')

FAILURE:
  robot_state == expected_stand_label  ('stand' — robot is upright)

CONFIG_DEFAULTS:
  expected_stand_label: 'stand'  — BB value that represents the upright state.
  Project trees may override this via constructor args.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexL1ConditionNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Balance_IsFallen(AinexL1ConditionNode):
    """SUCCESS if /latched/robot_state != expected_stand_label."""

    LEVEL = 'L1'
    BB_READS = [BB.ROBOT_STATE]
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'expected_stand_label': 'stand',
    }

    def __init__(self, expected_stand_label: str = 'stand',
                 name: str = 'L1_Balance_IsFallen',
                 logger=None, tick_id_getter=None):
        """
        Args:
            expected_stand_label: BB robot_state value that means upright.
            name:                 BT node name.
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

    def _is_fallen(self, state: str) -> bool:
        """Return True when robot_state indicates a fallen or transitional state."""
        return state != self._expected_stand_label

    def update(self) -> Status:
        state = self._bb.robot_state
        passed = self._is_fallen(state)
        status = self.status_from_bool(passed)

        self.emit_decision(
            inputs={'robot_state': state,
                    'expected_stand_label': self._expected_stand_label},
            status=status,
            reason=(f"robot_state == '{state}'" if passed
                    else f"robot_state == '{self._expected_stand_label}'"),
        )

        return status
