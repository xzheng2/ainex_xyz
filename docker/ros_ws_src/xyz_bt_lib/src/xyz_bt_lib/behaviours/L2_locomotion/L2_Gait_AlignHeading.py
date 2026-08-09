#!/usr/bin/env python3
"""L2_Gait_AlignHeading - turn the body in place (or walk forward) until the heading
matches a target.

L2 action/strategy node (shared library).

Node kind: finite_action  (x_speed == 0: discrete turn → SUCCESS on arrival)
           continuous_controller-like when x_speed > 0 and wrapped in SuccessIsRunning
           (forward-hold: keeps walking straight on the target heading)

BB reads:
  BB.NAV_IMU_HEADING  (/latched/nav_imu_heading)  — body heading (deg; 0 = straight ahead,
                       + = left, − = right), written each tick by DepthNavAdapter.
  BB.HEADING_TARGET   (/latched/heading_target)   — READ ONLY when target_bb_key is set;
                       the absolute heading (deg) the body should reach. When unset the
                       static target_deg is used instead.

BB writes:
  none

Facade calls:
  turn_step(x=0, y, yaw)         — one in-place turn step per tick (x_speed == 0).
  go_step(x=x_speed, y, yaw)     — one forward+steer step per tick (x_speed > 0).

Action strategy:
  Reads the body heading and the target heading and turns/walks to drive
  error = (heading − target) toward 0. When |error| <= threshold_deg it is aligned;
  otherwise it issues one step and returns RUNNING.
  - Turn-to-target (x_speed == 0): turn in place; returns SUCCESS on arrival. This is
    the original behaviour (target defaults to 0 → align to front, identical math).
  - Forward-hold (x_speed > 0): walk straight while correcting heading. On arrival it
    still issues a forward step and returns SUCCESS; wrap it in SuccessIsRunning to make
    the robot keep walking straight on the target heading indefinitely.

Strategy helper:
  _compute_yaw(error) → signed gait yaw (mirrors L2_Gait_VisionToObject._compute_yaw).
  _resolve_target()   → the active target heading (deg) from BB or the static default.

CONFIG_DEFAULTS:
  threshold_deg:  8    — |error| below this (deg) counts as aligned → SUCCESS. Keep <= the
                        downstream L1_Nav_IsHeadingAligned threshold so the gate passes.
  kp:             0.5  — heading error (deg) → gait yaw (deg) proportional gain.
  max_yaw_deg:    8    — maximum absolute gait yaw (deg).
  min_yaw_deg:    2    — minimum |yaw| when outside the deadband (0 = purely proportional).
  deadband_deg:   2    — |error| deadband (deg) inside which yaw = 0.
  yaw_sign:      -1.0  — error→yaw sign. -1.0: error>0 (heading left of target) → yaw<0
                        (turn right) → error decreases toward 0. VALIDATE on the robot
                        (flip to +1.0 if it turns wrong).
  target_deg:     0    — static target heading (deg) used when target_bb_key is None.
  target_bb_key:  None — when set (e.g. BB.HEADING_TARGET_KEY), read the target heading
                        from the blackboard each tick instead of target_deg.
  x_speed:        0.0  — forward step (m) per tick. 0 = turn in place (discrete). >0 =
                        forward-hold: walk straight while holding the target heading.
  period_time_ms: None — gait cycle duration (ms) override. None = project profile default
                        (_GO_STEP_VEL on the forward-hold path, _TURN_STEP_VEL on the turn path).
  dsp_ratio:      None — double-support fraction (0–1) override; None = profile default.
  y_swap_amplitude: None — lateral body swing (m) override; None = profile default.
  arm_swap:       None — arm swing amplitude (°); None = profile default (→ 30).
  step_num:       None — steps per command (0 = continuous); None = profile default
                        (_GO_CFG on the forward-hold path, _TURN_CFG on the turn path).
  gait_param:     None — partial WalkingParam dict (e.g. {'step_height': 0.03}); None = no override.

Returns:
  RUNNING: |error| > threshold_deg — one step issued this tick.
  SUCCESS: |error| <= threshold_deg — aligned (x_speed>0 also issues a forward step first).
  FAILURE: never.

Observability:
  Emits 'action_intent' on initialise and 'decision' on update.
  Never emits ros_out/ros_result or any comm event; those belong to _RuntimeIO.
"""
import math
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L2_Gait_AlignHeading(XyzL2ActionNode):
    """Turn/walk until abs(nav_imu_heading − target) <= threshold_deg, then SUCCESS."""

    LEVEL = 'L2'
    BB_READS = [BB.NAV_IMU_HEADING, BB.HEADING_TARGET]
    BB_WRITES = []
    FACADE_CALLS = ['turn_step', 'go_step']
    CONFIG_DEFAULTS = {
        'threshold_deg': 8,
        'kp':            0.5,
        'max_yaw_deg':   8,
        'min_yaw_deg':   2,
        'deadband_deg':  2,
        'yaw_sign':     -1.0,
        'target_deg':    0,
        'target_bb_key': None,
        'x_speed':       0.0,
        'period_time_ms': None,
        'dsp_ratio':        None,
        'y_swap_amplitude': None,
        'arm_swap':         None,
        'step_num':         None,
        'gait_param':       None,
    }
    BB_LOG_KEYS = BB_READS

    def __init__(self, name: str = 'L2_Gait_AlignHeading',
                 facade: XyzBTFacade = None,
                 threshold_deg: float = 8,
                 kp: float = 0.5,
                 max_yaw_deg: int = 8,
                 min_yaw_deg: int = 2,
                 deadband_deg: float = 2,
                 yaw_sign: float = -1.0,
                 target_deg: float = 0,
                 target_bb_key: str = None,
                 x_speed: float = 0.0,
                 period_time_ms: int = None,
                 dsp_ratio: float = None,
                 y_swap_amplitude: float = None,
                 arm_swap: int = None,
                 step_num: int = None,
                 gait_param: dict = None,
                 logger=None,
                 tick_id_getter=None):
        """
        CONFIG_DEFAULTS:
            threshold_deg: |error| below this (deg) = aligned → SUCCESS. Keep <= the
                           downstream L1_Nav_IsHeadingAligned threshold.
            kp:            heading error (deg) → gait yaw (deg) proportional gain.
            max_yaw_deg:   Maximum absolute gait yaw (deg).
            min_yaw_deg:   Minimum |yaw| when outside the deadband (0 = purely proportional).
            deadband_deg:  |error| deadband (deg) inside which yaw = 0.
            yaw_sign:      error→yaw sign (-1.0 default). VALIDATE on the robot.
            target_deg:    Static target heading (deg) used when target_bb_key is None.
            target_bb_key: BB latched short key (e.g. BB.HEADING_TARGET_KEY) to read the
                           target heading from each tick; None uses target_deg.
            x_speed:       Forward step (m) per tick. 0 = turn in place; >0 = forward-hold.
            period_time_ms: Gait cycle duration (ms) override. None = project profile default
                           (_GO_STEP_VEL forward-hold, _TURN_STEP_VEL turn path).
            dsp_ratio:        Double-support fraction (0–1) override. None = profile default.
            y_swap_amplitude: Lateral body swing (m) override. None = profile default.
            arm_swap:         Arm swing amplitude (°). None = profile default (→ 30).
            step_num:         Steps per command (0 = continuous). None = profile default.
            gait_param:       Partial WalkingParam dict. None = no override.
        """
        super().__init__(
            name,
            facade=facade,
            logger=logger,
            tick_id_getter=tick_id_getter,
        )
        self._threshold_deg = threshold_deg
        self._kp            = kp
        self._max_yaw_deg   = max_yaw_deg
        self._min_yaw_deg   = min_yaw_deg
        self._deadband_deg  = deadband_deg
        self._yaw_sign      = yaw_sign
        self._target_deg    = target_deg
        self._target_bb_key = target_bb_key
        self._x_speed       = x_speed
        self._period_time_ms = period_time_ms
        self._dsp_ratio        = dsp_ratio
        self._y_swap_amplitude = y_swap_amplitude
        self._arm_swap         = arm_swap
        self._step_num         = step_num
        self._gait_param       = gait_param
        self._bb            = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.NAV_IMU_HEADING_KEY, access=Access.READ)
        if self._target_bb_key is not None:
            self._bb.register_key(key=self._target_bb_key, access=Access.READ)

    def _resolve_target(self) -> float:
        """Return the active target heading (deg): the BB value when target_bb_key is set
        and valid (not NaN), else the static target_deg. Side-effect-free (BB read only)."""
        if self._target_bb_key is not None:
            val = getattr(self._bb, self._target_bb_key)
            if val is not None and val == val:   # not None, not NaN
                return float(val)
        return float(self._target_deg)

    def _compute_yaw(self, error: float) -> int:
        """Return signed gait yaw to drive the heading error toward 0. Side-effect-free.

        Inside deadband → 0. Else raw = yaw_sign * kp * error, clamped to ±max_yaw_deg,
        with a min_yaw_deg floor when outside the deadband.
        """
        if abs(error) <= self._deadband_deg:
            return 0
        raw = self._yaw_sign * self._kp * error
        clamped = int(max(-self._max_yaw_deg, min(self._max_yaw_deg, raw)))
        if self._min_yaw_deg > 0 and abs(clamped) < self._min_yaw_deg:
            clamped = int(math.copysign(self._min_yaw_deg, raw))
        return clamped

    def initialise(self):
        self.emit_action_intent(
            action='align_heading',
            inputs={'threshold_deg': self._threshold_deg,
                    'kp': self._kp, 'yaw_sign': self._yaw_sign,
                    'x_speed': self._x_speed,
                    'target': self._resolve_target() if self._bb is not None else self._target_deg},
        )

    def _gait_kwargs(self) -> dict:
        """Common gait-parameter overrides forwarded to both go_step and turn_step."""
        return {
            'period_time_ms':   self._period_time_ms,
            'dsp_ratio':        self._dsp_ratio,
            'y_swap_amplitude': self._y_swap_amplitude,
            'arm_swap':         self._arm_swap,
            'step_num':         self._step_num,
            'gait_param':       self._gait_param,
        }

    def _step(self, yaw: int) -> None:
        """Issue one gait step this tick: forward+steer when x_speed>0, else turn in place.

        All gait-parameter overrides default to None (= each facade profile picks its own
        default), so both go_step and turn_step accept the same kwargs unchanged.
        """
        kw = self._gait_kwargs()
        if self._x_speed > 0:
            self.call_facade('go_step', x=self._x_speed, y=0.0, yaw=yaw,
                             semantic_source='align_heading', **kw)
        else:
            self.call_facade('turn_step', x=0.0, y=0.0, yaw=yaw,
                             semantic_source='align_heading', **kw)

    def update(self) -> Status:
        heading = float(self._bb.nav_imu_heading)
        target  = self._resolve_target()
        error   = heading - target

        # SUCCESS: heading already at target
        if abs(error) <= self._threshold_deg:
            if self._x_speed > 0:
                # Forward-hold: keep walking straight on the target heading.
                self._step(yaw=0)
            self.emit_decision(
                inputs={'nav_imu_heading': heading, 'target': target,
                        'threshold_deg': self._threshold_deg},
                status=Status.SUCCESS,
                reason='heading aligned (|%.1f| <= %s)' % (error, self._threshold_deg),
            )
            return Status.SUCCESS

        # RUNNING: turn/walk to reduce heading error
        yaw = self._compute_yaw(error)
        self._step(yaw=yaw)
        self.emit_decision(
            inputs={'nav_imu_heading': heading, 'target': target,
                    'threshold_deg': self._threshold_deg, 'yaw': yaw,
                    'x_speed': self._x_speed},
            status=Status.RUNNING,
            reason='aligning heading (err=%.1f > %s, yaw=%d)'
                   % (error, self._threshold_deg, yaw),
        )
        return Status.RUNNING
