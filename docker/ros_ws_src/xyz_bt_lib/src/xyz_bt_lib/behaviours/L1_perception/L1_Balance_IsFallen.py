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
  robot_state is a confirmed fallen posture ('lie' or 'recline')

FAILURE:
  robot_state == expected_stand_label ('stand' — robot is upright), OR
  robot_state == 'pending' (recovery played, IMU still confirming)

CONFIG_DEFAULTS:
  expected_stand_label: 'stand'  — BB value that represents the upright state.
  Project trees may override this via constructor args.

Related node — NOT a strict inverse (since Aug 2026):
  L1_Balance_IsStanding asks the opposite question, and both are kept rather than
  collapsed into one node plus py_trees Inverter, because the tree wiring is the
  layer humans and agents read: a safety branch guarded by `L1_Balance_IsFallen`
  states its intent directly, where `Inverter(L1_Balance_IsStanding)` makes the
  reader resolve a double negative. But they are no longer exact complements:
  robot_state has a fourth value, 'pending' (stand-up action played, awaiting IMU
  confirmation), and BOTH nodes return FAILURE for it — so recovery is not
  re-triggered while the IMU is still deciding. FAILURE here is the safe
  direction: it means "do not start another recovery", not "the robot is fine".

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Balance_IsFallen(XyzL1ConditionNode):
    """SUCCESS if /latched/robot_state != expected_stand_label."""

    LEVEL = 'L1'
    BB_READS = [BB.ROBOT_STATE]
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'expected_stand_label': 'stand',
    }

    def __init__(self, name: str = 'L1_Balance_IsFallen',
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

    PENDING_LABEL = 'pending'

    def _is_fallen(self, state: str) -> bool:
        """Return True only for a CONFIRMED fallen posture.

        'pending' (recovery action played, IMU not yet confirmed) is explicitly
        not fallen: returning True there would re-trigger recovery on the very
        next tick, before the robot has had a chance to be judged upright.
        """
        return (state != self._expected_stand_label
                and state != self.PENDING_LABEL)

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
