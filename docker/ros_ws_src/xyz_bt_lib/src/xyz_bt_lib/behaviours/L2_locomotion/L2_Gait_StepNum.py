#!/usr/bin/env python3
"""L2_Gait_StepNum - one-shot fixed-count reposition step.

L2 action/strategy node (shared library).

Node kind: dispatch (one-shot)

BB reads:
  none

BB writes:
  none

Facade calls:
  go_step(x, y, yaw, step_num, period_time_ms, dsp_ratio, y_swap_amplitude, arm_swap, gait_param)

Action strategy:
  Issues a single blocking N-step gait command (step_num > 0) to nudge the robot into a
  target pose (a small sidestep by default), then returns SUCCESS. The command is dispatched
  exactly once per activation (guarded by _dispatched) so the reposition never repeats every
  tick. Callers typically pass mirror-image y (and/or yaw) per branch to reposition toward a
  chosen side.

Strategy helper:
  _select_action() → ('go_step', kwargs, reason)

CONFIG_DEFAULTS:
  x:                0.0    — forward step magnitude (m); + forward, − back
  y:                0.015  — lateral step magnitude (m); sign chooses side (tree sets per branch)
  yaw:              0      — rotation per step (deg)
  step_num:         2      — number of steps to walk (blocking; 0 = continuous — do not use here)
  period_time_ms:   None   — gait cycle (ms); None = project default
  dsp_ratio:        None   — double-support fraction; None = project default
  y_swap_amplitude: None   — lateral body swing (m); None = project default
  arm_swap:         None   — arm swing amplitude (deg); None = project default
  gait_param:       None   — partial WalkingParam dict (e.g. {'step_height': 0.03}); None = no override

Returns:
  SUCCESS: the reposition step was dispatched (always, one-shot).
  RUNNING: never.
  FAILURE: never.

Observability:
  Emits 'action_intent' on initialise and 'decision' on update.
  Never emits ros_out/ros_result or any comm event; those belong to _RuntimeIO.
"""
from py_trees.common import Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade


class L2_Gait_StepNum(XyzL2ActionNode):
    """Dispatch a single blocking N-step reposition walk, then SUCCESS (one-shot)."""

    LEVEL = 'L2'
    BB_READS = []
    BB_WRITES = []
    FACADE_CALLS = ['go_step']
    CONFIG_DEFAULTS = {
        'x':                0.0,
        'y':                0.015,
        'yaw':              0,
        'step_num':         2,
        'period_time_ms':   None,
        'dsp_ratio':        None,
        'y_swap_amplitude': None,
        'arm_swap':         None,
        'gait_param':       None,
    }
    BB_LOG_KEYS = BB_READS

    def __init__(self, name: str = 'L2_Gait_StepNum',
                 facade: XyzBTFacade = None,
                 x: float = 0.0,
                 y: float = 0.015,
                 yaw: int = 0,
                 step_num: int = 2,
                 period_time_ms: int = None,
                 dsp_ratio: float = None,
                 y_swap_amplitude: float = None,
                 arm_swap: int = None,
                 gait_param: dict = None,
                 logger=None,
                 tick_id_getter=None):
        """
        CONFIG_DEFAULTS:
            x:                Forward step magnitude (m); + forward, − back.
            y:                Lateral step magnitude (m); sign chooses the side.
            yaw:              Rotation per step (deg).
            step_num:         Number of steps to walk (blocking; must be > 0).
            period_time_ms:   Gait cycle (ms). None = project default.
            dsp_ratio:        Double-support fraction. None = project default.
            y_swap_amplitude: Lateral body swing (m). None = project default.
            arm_swap:         Arm swing amplitude (deg). None = project default.
            gait_param:       Partial WalkingParam dict. None = no override.
        """
        super().__init__(
            name,
            facade=facade,
            logger=logger,
            tick_id_getter=tick_id_getter,
        )
        self._x                = x
        self._y                = y
        self._yaw              = yaw
        self._step_num         = step_num
        self._period_time_ms   = period_time_ms
        self._dsp_ratio        = dsp_ratio
        self._y_swap_amplitude = y_swap_amplitude
        self._arm_swap         = arm_swap
        self._gait_param       = gait_param
        self._dispatched       = False

    def initialise(self):
        """Reset the one-shot guard and emit action_intent when the step starts."""
        self._dispatched = False
        self.emit_action_intent(
            action='gait_step_num',
            inputs={'x': self._x, 'y': self._y, 'yaw': self._yaw,
                    'step_num': self._step_num},
        )

    def _select_action(self) -> tuple:
        """Return ('go_step', kwargs, reason). Side-effect-free."""
        kwargs = {
            'x':                self._x,
            'y':                self._y,
            'yaw':              self._yaw,
            'step_num':         self._step_num,
            'period_time_ms':   self._period_time_ms,
            'dsp_ratio':        self._dsp_ratio,
            'y_swap_amplitude': self._y_swap_amplitude,
            'arm_swap':         self._arm_swap,
            'gait_param':       self._gait_param,
            'semantic_source':  'gait_step_num',
        }
        return 'go_step', kwargs, 'dispatch %d-step reposition' % self._step_num

    def update(self) -> Status:
        # One-shot: dispatch the reposition step exactly once per activation.
        if not self._dispatched:
            method_name, kwargs, reason = self._select_action()
            self.call_facade(method_name, **kwargs)
            self._dispatched = True
        else:
            reason = 'already dispatched (one-shot)'

        self.emit_decision(
            inputs={'x': self._x, 'y': self._y, 'yaw': self._yaw,
                    'step_num': self._step_num},
            status=Status.SUCCESS,
            reason=reason,
        )
        return Status.SUCCESS
