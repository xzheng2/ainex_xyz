#!/usr/bin/env python3
"""L2 Action: blocking fall-recovery sequence.

BB reads:  BB.ROBOT_STATE  (/latched/robot_state) — posture: 'lie' | 'recline'
BB writes: BB.ROBOT_STATE  = 'stand' after recovery completes
Facade:    recover_from_fall(recovery_action)
Strategy:  _compute_recovery_action(robot_state) → action str
           _perform_recovery(robot_state) — map posture, delegate to facade,
                                            write 'stand', sync setter hook

Posture → recovery action mapping (class constant, not tunable):
  'lie'     → 'lie_to_stand'
  'recline' → 'recline_to_stand'

Delegates the full recovery sequence (buzzer → disable gait → stand-up action)
to the project SemanticFacade; this node only owns the posture→action mapping,
blackboard I/O, and the optional post-recovery setter hook.

Returns:
    SUCCESS → recovery complete (facade is blocking; returns only after stand-up finishes)

CONFIG_DEFAULTS: none

Constructor extras:
    robot_state_setter: optional callable(str) — called with 'stand' after BB write,
                        to sync the live ImuBalanceStateAdapter store and prevent the
                        next pre-tick latch from overwriting the BB with a stale value.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexL2ActionNode
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.blackboard_keys import BB


class L2_Balance_RecoverFromFall(AinexL2ActionNode):
    """Map posture state to recovery action, execute via facade, write 'stand'."""

    LEVEL        = 'L2'
    BB_READS     = [BB.ROBOT_STATE]
    BB_WRITES    = [BB.ROBOT_STATE]
    FACADE_CALLS = ['recover_from_fall']
    CONFIG_DEFAULTS = {}

    # Posture → recovery action name (symbolic constants, not tunable).
    _RECOVERY_ACTIONS = {
        'lie':     'lie_to_stand',
        'recline': 'recline_to_stand',
    }

    def __init__(self, name: str = 'L2_Balance_RecoverFromFall',
                 facade: AinexBTFacade = None,
                 robot_state_setter=None,
                 logger=None,
                 tick_id_getter=None):
        """
        Args:
            robot_state_setter: optional callable(str) — called with 'stand' after
                                BB write, to sync the live ImuBalanceStateAdapter store.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._robot_state_setter = robot_state_setter
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.ROBOT_STATE_KEY, access=Access.WRITE)
        # Access.WRITE implicitly grants read (py_trees Access is enum.Enum, not IntFlag)

    def _compute_recovery_action(self, robot_state: str) -> str:
        """Map posture state to the facade recovery action name.

        'lie'     → 'lie_to_stand'
        'recline' → 'recline_to_stand'
        Unknown   → 'lie_to_stand'  (safe fallback; logs via facade)

        No BB reads/writes, ROS calls, or facade calls here.
        """
        return self._RECOVERY_ACTIONS.get(robot_state, 'lie_to_stand')

    def _perform_recovery(self, robot_state: str) -> str:
        """Map posture, delegate recovery to facade, update BB, sync live store.

        Returns the recovery_action string for observability.
        """
        recovery_action = self._compute_recovery_action(robot_state)
        self.call_facade('recover_from_fall', robot_state=recovery_action)
        self._bb.robot_state = 'stand'
        if self._robot_state_setter is not None:
            self._robot_state_setter('stand')
        return recovery_action

    def initialise(self):
        self.emit_action_intent(
            action='recover_from_fall',
            inputs={'robot_state': self._bb.robot_state},
        )

    def update(self) -> Status:
        state = self._bb.robot_state
        recovery_action = self._perform_recovery(state)
        self.emit_decision(
            inputs={'robot_state': state, 'recovery_action': recovery_action},
            status=Status.SUCCESS,
            reason=f'{state} → {recovery_action}, robot_state set to stand',
            bb_writes={BB.ROBOT_STATE: 'stand'},
        )
        return Status.SUCCESS
