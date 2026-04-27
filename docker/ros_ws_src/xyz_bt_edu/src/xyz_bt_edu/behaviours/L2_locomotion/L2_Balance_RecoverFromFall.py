#!/usr/bin/env python3
"""L2 Action: blocking fall-recovery sequence.

BB reads:  BB.ROBOT_STATE  (/latched/robot_state) — posture: 'lie' | 'recline'
BB writes: BB.ROBOT_STATE  = 'stand' after recovery completes
Facade:    publish_buzzer(freq, on_time, off_time, repeat)
           disable_gait()
           run_action(action_name)
Strategy:  _compute_recovery_action(robot_state) → action str
           _perform_recovery(robot_state) — map posture, run buzzer + sleep +
                                            disable_gait + run_action + sleep,
                                            write 'stand', sync setter hook

Posture → recovery action mapping (class constant, not tunable):
  'lie'     → 'lie_to_stand'
  'recline' → 'recline_to_stand'

This node owns the full recovery sequence inline; no high-level facade composite
is required. All timing and action defaults are exposed via CONFIG_DEFAULTS.

Returns:
    SUCCESS → recovery complete (run_action is blocking; returns after stand-up finishes)

CONFIG_DEFAULTS:
    lie_action:          'lie_to_stand'   — action name for face-down posture
    recline_action:      'recline_to_stand' — action name for face-up posture
    buzzer_freq:         1900             — tone frequency (Hz)
    buzzer_on_time:      0.1             — beep on duration (s)
    buzzer_off_time:     0.01            — beep off duration (s)
    buzzer_repeat:       1               — number of beeps
    pre_action_delay_s:  2.0             — sleep after buzzer, before disable_gait+run_action
    post_action_delay_s: 0.5             — sleep after run_action before returning

Constructor extras:
    robot_state_setter: optional callable(str) — called with 'stand' after BB write,
                        to sync the live ImuBalanceStateAdapter store and prevent the
                        next pre-tick latch from overwriting the BB with a stale value.
"""
import time
from py_trees.common import Access, Status
from xyz_bt_edu.base_node import XyzL2ActionNode
from xyz_bt_edu.base_facade import XyzBTFacade
from xyz_bt_edu.blackboard_keys import BB


class L2_Balance_RecoverFromFall(XyzL2ActionNode):
    """Map posture state to recovery action, execute inline recovery sequence, write 'stand'."""

    LEVEL        = 'L2'
    BB_READS     = [BB.ROBOT_STATE]
    BB_WRITES    = [BB.ROBOT_STATE]
    FACADE_CALLS = ['publish_buzzer', 'disable_gait', 'run_action']
    CONFIG_DEFAULTS = {
        'lie_action':          'lie_to_stand',
        'recline_action':      'recline_to_stand',
        'buzzer_freq':         1900,
        'buzzer_on_time':      0.1,
        'buzzer_off_time':     0.01,
        'buzzer_repeat':       1,
        'pre_action_delay_s':  2.0,
        'post_action_delay_s': 0.5,
    }

    # Posture → recovery action name (symbolic constants, not tunable).
    _RECOVERY_ACTIONS = {
        'lie':     'lie_to_stand',
        'recline': 'recline_to_stand',
    }

    def __init__(self, name: str = 'L2_Balance_RecoverFromFall',
                 facade: XyzBTFacade = None,
                 robot_state_setter=None,
                 logger=None,
                 tick_id_getter=None,
                 lie_action: str = 'lie_to_stand',
                 recline_action: str = 'recline_to_stand',
                 buzzer_freq: int = 1900,
                 buzzer_on_time: float = 0.1,
                 buzzer_off_time: float = 0.01,
                 buzzer_repeat: int = 1,
                 pre_action_delay_s: float = 2.0,
                 post_action_delay_s: float = 0.5):
        """
        Args:
            robot_state_setter:  optional callable(str) — called with 'stand' after
                                 BB write, to sync the live ImuBalanceStateAdapter store.
            lie_action:          action name for face-down posture.
            recline_action:      action name for face-up posture.
            buzzer_freq:         tone frequency in Hz.
            buzzer_on_time:      beep on duration (seconds).
            buzzer_off_time:     beep off duration (seconds).
            buzzer_repeat:       number of beeps.
            pre_action_delay_s:  sleep after buzzer, before gait disable + stand-up action.
            post_action_delay_s: sleep after stand-up action before returning.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._robot_state_setter  = robot_state_setter
        self._lie_action          = lie_action
        self._recline_action      = recline_action
        self._buzzer_freq         = buzzer_freq
        self._buzzer_on_time      = buzzer_on_time
        self._buzzer_off_time     = buzzer_off_time
        self._buzzer_repeat       = buzzer_repeat
        self._pre_action_delay_s  = pre_action_delay_s
        self._post_action_delay_s = post_action_delay_s
        self._bb                  = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.ROBOT_STATE_KEY, access=Access.WRITE)
        # Access.WRITE implicitly grants read (py_trees Access is enum.Enum, not IntFlag)

    def _compute_recovery_action(self, robot_state: str) -> str:
        """Map posture state to the recovery action name.

        'lie'     → self._lie_action
        'recline' → self._recline_action
        Unknown   → self._lie_action  (safe fallback)

        No BB reads/writes, ROS calls, or facade calls here.
        """
        if robot_state == 'lie':
            return self._lie_action
        elif robot_state == 'recline':
            return self._recline_action
        return self._lie_action  # safe fallback

    def _perform_recovery(self, robot_state: str) -> str:
        """Run the full recovery sequence inline and update BB.

        Sequence: buzzer alert → pre-action delay → disable gait →
                  run stand-up action → post-action delay → write 'stand'.

        Returns the recovery_action string for observability.
        """
        recovery_action = self._compute_recovery_action(robot_state)
        self.call_facade('publish_buzzer',
                         freq=self._buzzer_freq,
                         on_time=self._buzzer_on_time,
                         off_time=self._buzzer_off_time,
                         repeat=self._buzzer_repeat)
        time.sleep(self._pre_action_delay_s)
        self.call_facade('disable_gait')
        self.call_facade('run_action', action_name=recovery_action)
        time.sleep(self._post_action_delay_s)
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
