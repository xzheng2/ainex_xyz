#!/usr/bin/env python3
"""L2 Action: walk forward steering by the depth-nav path geometry at rows 7/10.

Note: the two sampled rows are set by planner YAW_ROWS (currently 7 and 10). They
are still carried on the message/BB as nav_u9 (row 7) and nav_u10 (row 10) — the
field names are kept for back-compat, so nav_u9 no longer means "row 9".

Node kind: continuous_controller

BB reads:
  BB.NAV_U9   (/latched/nav_u9)   — path block-centre pixel-x at depth grid row 7
  BB.NAV_U10  (/latched/nav_u10)  — path block-centre pixel-x at depth grid row 10
  BB.NAV_CX   (/latched/nav_cx)   — camera principal point x (px)
  BB.NAV_FX   (/latched/nav_fx)   — camera focal length x (px)
  BB.NAV_START_BX (/latched/nav_start_bx) — A* start block column (ball column); drives
                                    the optional fixed lateral (y) step toward the ball.
  BB.NAV_START_BY (/latched/nav_start_by) — A* start block row (ball row; larger = nearer);
                                    drives the optional near-slow forward (x) rule.
                                    All written each tick by DepthNavAdapter; u9/u10 are
                                    NaN when the path misses that row (or there is no path).

Facade:
  go_step(x, y, yaw)  — one forward+steer step per tick while RUNNING

Strategy:
  The planner no longer computes a heading; it exposes the raw path pixel-x at grid
  rows 7 and 10. This node:
    1. Forms candidate offsets ``u - cx`` for each non-NaN row.
    2. Selects one per ``select_mode``:
         'larger'  → the candidate with the larger |u - cx| (more aggressive steer, default)
         'smaller' → the candidate with the smaller |u - cx| (gentler)
       The chosen offset keeps its sign.
    3. Converts to a true bearing ``ang = degrees(atan(offset / fx))``
       (phase3 convention: + = path right of optical axis).
    4. Maps to a gait yaw: ``gait_yaw = clamp(yaw_sign * kp * ang)`` with deadband +
       min/max limits. ``yaw_sign`` default -1.0 (gait turn-right is negative yaw);
       **validate yaw_sign on the robot.**

  If both rows are NaN (no usable path geometry this frame), returns FAILURE so a
  parent Selector can re-route. Head is NOT moved here — the depth-nav grid geometry
  assumes a fixed head pitch pre-positioned by a separate node / launch step.

Returns:
  RUNNING  — usable path geometry; one forward+steer step issued
  FAILURE  — both u9 and u10 are NaN (no path this frame); lets a parent Selector re-route

CONFIG_DEFAULTS:
  select_mode:       'larger' — 'larger'|'smaller': pick larger/smaller |u-cx| of rows 7/10
  x_speed:           0.010   — forward step magnitude (m)
  x_speed_near:      None    — slow forward step (m) when the ball is near AND laterally off;
                              None (default) = disabled (x always = x_speed)
  x_near_col_err:    4       — |start_bx-center_col| (blocks) must exceed this to slow forward
  x_near_by_min:     13      — near row-band lower bound (inclusive) on start_by
  x_near_by_max:     15      — near row-band upper bound (inclusive) on start_by
  kp:                1.0     — bearing (deg) → gait yaw (deg) gain
  yaw_sign:          -1.0    — planner→gait sign (validate on robot)
  max_yaw_deg:       10      — maximum absolute gait yaw (deg)
  min_yaw_deg:       0       — minimum |yaw| when outside deadband (0 = purely proportional)
  deadband:          1.0     — bearing deadband (deg) inside which yaw = 0
  ema_enabled:       True    — EMA-smooth the steering bearing before deadband/gain/clamp
  ema_alpha:         0.3     — EMA weight when enabled; ang = a*raw + (1-a)*prev
  y_speed:           0.0     — MAX lateral step (m) toward ball column (0 = disabled); also the
                              fixed magnitude when y_kp == 0 (legacy). Ceiling in proportional mode.
  y_kp:              0.0     — proportional lateral gain (m per BLOCK of column bias). 0 = legacy
                              fixed y_speed step (backward compatible).
  y_min:             0.0     — minimum |y| (m) when outside the deadband (0 = purely proportional)
  center_col:        8       — reference grid column to align to (fallback start cell)
  y_sign:            -1.0    — column-offset → gait-y sign (validate on robot)
  y_deadband_cols:   1       — |start_bx - center_col| (blocks) within this → y = 0 (deadband)

Gait pass-through (period_time_ms / dsp_ratio / y_swap_amplitude / arm_swap /
step_num / gait_param): inherited from XyzL2GaitActionNode — see that class's
docstring. Leg-lift height, hip pitch, body height and per-step heading bias now
go in the dict, e.g. gait_param={'step_height': 0.03, 'hip_pitch_offset': 12,
'body_height': 0.019, 'init_yaw_offset': 2}. NOTE the rename: this node used to
accept init_z_offset= and silently map it to the 'body_height' key — write
'body_height' directly now.
"""
import math
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2GaitActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L2_Gait_FollowNavPath(XyzL2GaitActionNode):
    """Walk forward steering by the selected row-7/10 path point; FAILURE when no path."""

    LEVEL        = 'L2'
    BB_READS     = [BB.NAV_U9, BB.NAV_U10, BB.NAV_CX, BB.NAV_FX, BB.NAV_START_BX, BB.NAV_START_BY]
    BB_WRITES    = []
    FACADE_CALLS = ['go_step']
    CONFIG_DEFAULTS = {
        'select_mode':      'larger',
        'x_speed':          0.010,
        # Near-slow forward rule (opt-in; x_speed_near=None = disabled, constant x_speed):
        'x_speed_near':     None,    # slow forward step (m) when ball near AND laterally off
        'x_near_col_err':   4,       # |start_bx-center_col| (blocks) must exceed this to slow
        'x_near_by_min':    13,      # near row-band lower bound (inclusive) on start_by
        'x_near_by_max':    15,      # near row-band upper bound (inclusive) on start_by
        'kp':               1.0,
        'yaw_sign':         -1.0,
        'max_yaw_deg':      10,
        'min_yaw_deg':      0,
        'deadband':         1.0,
        'ema_enabled':      True,
        'ema_alpha':        0.3,
        'y_speed':          0.0,
        'y_kp':             0.0,
        'y_min':            0.0,
        'center_col':       8,
        'y_sign':           -1.0,
        'y_deadband_cols':  1,
        # Gait pass-through knobs are inherited from
        # XyzL2GaitActionNode.GAIT_CONFIG_DEFAULTS — do not repeat them here.
    }

    def __init__(self, name: str = 'L2_Gait_FollowNavPath',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 select_mode: str = 'larger',
                 x_speed: float = 0.010,
                 x_speed_near: float = None,
                 x_near_col_err: int = 4,
                 x_near_by_min: int = 13,
                 x_near_by_max: int = 15,
                 kp: float = 1.0,
                 yaw_sign: float = -1.0,
                 max_yaw_deg: int = 10,
                 min_yaw_deg: int = 0,
                 deadband: float = 1.0,
                 ema_enabled: bool = True,
                 ema_alpha: float = 0.3,
                 y_speed: float = 0.0,
                 y_kp: float = 0.0,
                 y_min: float = 0.0,
                 center_col: float = 8,
                 y_sign: float = -1.0,
                 y_deadband_cols: int = 1,
                 period_time_ms: int = None,
                 dsp_ratio: float = None,
                 y_swap_amplitude: float = None,
                 arm_swap: int = None,
                 step_num: int = None,
                 gait_param: dict = None):
        """
        CONFIG_DEFAULTS:
            select_mode:       'larger'|'smaller' — pick the row-7/10 candidate with the
                               larger (default, more aggressive) or smaller (gentler)
                               |u - cx|; the chosen offset keeps its sign.
            x_speed:           Forward step magnitude (m).
            x_speed_near:      Slow forward step (m) applied when the ball is near the feet
                               AND still laterally off-centre. None (default) disables the
                               rule (x is always x_speed) — keeps other trees unchanged.
                               When set: x = x_speed_near iff
                               |start_bx-center_col| > x_near_col_err AND
                               x_near_by_min <= start_by <= x_near_by_max, else x_speed.
                               Lets the robot crawl forward while the y-strafe finishes
                               aligning near the ball instead of overshooting it.
            x_near_col_err:    Lateral column-error threshold (blocks); the near-slow rule
                               requires |start_bx - center_col| to exceed this.
            x_near_by_min:     Near row-band lower bound (inclusive) on start_by (ball row).
            x_near_by_max:     Near row-band upper bound (inclusive) on start_by.
            kp:                Bearing (deg) → gait yaw (deg) gain.
            yaw_sign:          Planner→gait sign. Bearing uses phase3 convention
                               (+ = right of axis); gait turn-right is negative yaw,
                               so default -1.0. Validate on the robot.
            max_yaw_deg:       Maximum absolute gait yaw (deg).
            min_yaw_deg:       Minimum |yaw| when outside deadband (0 = proportional).
            deadband:          Bearing deadband (deg) inside which yaw = 0.
            ema_enabled:       Enable EMA smoothing of the steering bearing (before the
                               deadband/gain/clamp). False = raw per-tick bearing (identical
                               to the pre-EMA behaviour).
            ema_alpha:         EMA weight on the new bearing when enabled (0–1; ~0.3 typical).
                               ang = ema_alpha*raw + (1-ema_alpha)*prev. Higher = more
                               responsive/less smoothing; lower = smoother/more lag.
            y_speed:           MAX lateral (strafe) step magnitude (m) toward the ball column.
                               0.0 (default) disables the lateral term (y = 0). When y_kp == 0
                               it is the fixed step magnitude (legacy behaviour); when y_kp > 0
                               it is the proportional ceiling.
            y_kp:              Proportional lateral gain — metres of strafe per BLOCK of column
                               bias |start_bx - center_col|. 0.0 (default) = legacy fixed y_speed
                               step. NOTE: block-based (grid columns), not pixels — unlike the
                               yaw channel which works in pixels→degrees.
            y_min:             Minimum |y| (m) when the bias is outside the deadband
                               (0.0 = purely proportional; mirrors min_yaw_deg).
            center_col:        Reference grid column the robot aligns to (default 8 =
                               the fallback start cell, so "no ball" → no strafe).
            y_sign:            start_bx-offset → gait-y sign. Validate on the robot
                               (mirrors yaw_sign).
            y_deadband_cols:   |start_bx - center_col| within this many blocks (columns) → y = 0.

        Gait pass-through (period_time_ms / dsp_ratio / y_swap_amplitude /
        arm_swap / step_num / gait_param): see XyzL2GaitActionNode. Useful keys
        for this node: 'step_height' (leg lift, 0.01–0.04 m), 'hip_pitch_offset'
        (deg), 'body_height' (0.015–0.06 m), 'init_yaw_offset' (deg — a
        persistent per-step yaw bias, distinct from the per-tick steering yaw;
        use it to cancel a mechanical veer, and validate its sign on the robot).
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade,
                         period_time_ms=period_time_ms, dsp_ratio=dsp_ratio,
                         y_swap_amplitude=y_swap_amplitude, arm_swap=arm_swap,
                         step_num=step_num, gait_param=gait_param)
        self._select_mode      = 'smaller' if str(select_mode).lower() == 'smaller' else 'larger'
        self._x_speed          = x_speed
        self._x_speed_near     = x_speed_near
        self._x_near_col_err   = x_near_col_err
        self._x_near_by_min    = x_near_by_min
        self._x_near_by_max    = x_near_by_max
        self._kp               = kp
        self._yaw_sign         = yaw_sign
        self._max_yaw_deg      = max_yaw_deg
        self._min_yaw_deg      = min_yaw_deg
        self._deadband         = deadband
        self._ema_enabled      = bool(ema_enabled)
        self._ema_alpha        = ema_alpha
        self._ema_ang          = None            # EMA state (bearing deg); reset on path loss
        self._y_speed          = y_speed
        self._y_kp             = y_kp
        self._y_min            = y_min
        self._center_col       = center_col
        self._y_sign           = y_sign
        self._y_deadband_cols  = y_deadband_cols
        self._bb               = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.NAV_U9_KEY,  access=Access.READ)
        self._bb.register_key(key=BB.NAV_U10_KEY, access=Access.READ)
        self._bb.register_key(key=BB.NAV_CX_KEY,  access=Access.READ)
        self._bb.register_key(key=BB.NAV_FX_KEY,  access=Access.READ)
        self._bb.register_key(key=BB.NAV_START_BX_KEY, access=Access.READ)
        self._bb.register_key(key=BB.NAV_START_BY_KEY, access=Access.READ)

    def _select_offset(self, u9: float, u10: float, cx: float):
        """Return the signed pixel offset (u - cx) of the selected row, or None.

        Candidates are the non-NaN rows; selection picks larger/smaller |offset| per
        ``select_mode``. Returns None when both rows are NaN.
        """
        # u != u is True only for NaN.
        cands = [u - cx for u in (u9, u10) if u == u]
        if not cands:
            return None
        if self._select_mode == 'smaller':
            return min(cands, key=abs)
        return max(cands, key=abs)

    def _yaw_from_offset(self, offset: float, fx: float):
        """Map a signed pixel offset to (gait_yaw_deg:int, bearing_deg:float).

        When ema_enabled, the raw bearing is EMA-smoothed before the deadband/gain/clamp
        so both the deadband test and the emitted yaw act on the filtered signal. The
        filter state is reset on path loss / fresh start (see initialise / update).
        """
        ang = math.degrees(math.atan(offset / fx)) if fx else 0.0
        if self._ema_enabled and self._ema_alpha > 0.0:
            self._ema_ang = ang if self._ema_ang is None else (
                self._ema_alpha * ang + (1.0 - self._ema_alpha) * self._ema_ang)
            ang = self._ema_ang
        if abs(ang) <= self._deadband:
            return 0, ang
        raw     = self._yaw_sign * ang * self._kp
        clamped = int(max(-self._max_yaw_deg, min(self._max_yaw_deg, raw)))
        if self._min_yaw_deg > 0 and abs(clamped) < self._min_yaw_deg:
            clamped = int(math.copysign(self._min_yaw_deg, raw))
        return clamped, ang

    def _forward_x(self, start_bx, start_by):
        """Forward step (m): slow to x_speed_near when the ball is NEAR the feet AND
        still laterally off-centre; otherwise the normal x_speed.

        Opt-in: disabled (returns x_speed) unless x_speed_near is set. The near-slow
        zone is the AND of a column-error gate and a row-band gate:
          col_err = |start_bx - center_col| (blocks) must exceed x_near_col_err, AND
          start_by (ball row; larger = nearer) within [x_near_by_min, x_near_by_max].
        Lets the robot crawl forward while the y-strafe finishes aligning near the ball
        instead of overshooting it.
        """
        if self._x_speed_near is None or start_bx is None or start_by is None:
            return self._x_speed
        col_err = abs(start_bx - self._center_col)
        if col_err > self._x_near_col_err \
                and self._x_near_by_min <= start_by <= self._x_near_by_max:
            return self._x_speed_near
        return self._x_speed

    def _lateral_y(self, start_bx):
        """Lateral (strafe) step (m) toward the ball column start_bx, or 0.0.

        Mirrors the yaw controller but on the BLOCK bias off = start_bx - center_col
        (grid columns, not pixels):
          - disabled when y_speed == 0.0 (y_speed is the max ceiling / disable flag);
          - 0 inside the column deadband (|off| <= y_deadband_cols);
          - y_kp > 0 → proportional: |y| = clamp(y_kp * |off|, y_min, y_speed);
          - y_kp == 0 → legacy fixed ±y_speed step (backward compatible).
        y_sign maps the column-offset direction → gait y.
        """
        if self._y_speed == 0.0 or start_bx is None:
            return 0.0
        off = start_bx - self._center_col
        if abs(off) <= self._y_deadband_cols:
            return 0.0
        if self._y_kp > 0.0:
            mag = min(self._y_speed, max(self._y_min, self._y_kp * abs(off)))
        else:
            mag = self._y_speed
        return self._y_sign * math.copysign(mag, off)

    def initialise(self):
        self._ema_ang = None            # fresh start — no stale bearing to carry in
        self.emit_action_intent(
            action='follow_nav_path',
            inputs={'select_mode': self._select_mode,
                    'x_speed':     self._x_speed,
                    'kp':          self._kp,
                    'yaw_sign':    self._yaw_sign,
                    'max_yaw_deg': self._max_yaw_deg,
                    'deadband':    self._deadband},
        )

    def update(self) -> Status:
        u9 = self._bb.nav_u9
        u10 = self._bb.nav_u10
        cx = self._bb.nav_cx
        fx = self._bb.nav_fx

        offset = self._select_offset(u9, u10, cx)

        # FAILURE: no usable path geometry this frame — let a parent Selector re-route.
        if offset is None:
            self._ema_ang = None   # path lost — drop stale bearing so it can't leak on resume
            self.emit_decision(
                inputs={'nav_u9': u9, 'nav_u10': u10},
                status=Status.FAILURE,
                reason='no path geometry (u9 and u10 both NaN)',
            )
            return Status.FAILURE

        yaw, ang = self._yaw_from_offset(offset, fx)
        start_bx = self._bb.nav_start_bx
        start_by = self._bb.nav_start_by
        y = self._lateral_y(start_bx)
        x = self._forward_x(start_bx, start_by)
        self.call_facade('go_step',
                         x=x, y=y, yaw=yaw,
                         semantic_source='follow_nav_path',
                         **self.gait_kwargs())
        self.emit_decision(
            inputs={'select_mode': self._select_mode,
                    'nav_u9':      round(u9, 1) if u9 == u9 else None,
                    'nav_u10':     round(u10, 1) if u10 == u10 else None,
                    'offset_px':   round(offset, 1),
                    'bearing_deg': round(ang, 2),   # post-EMA when ema_enabled
                    'ema_on':      self._ema_enabled,
                    'gait_yaw':    yaw,
                    'x':           round(x, 4),
                    'start_bx':    start_bx,
                    'start_by':    start_by,
                    'y_off':       (start_bx - self._center_col) if start_bx is not None else None,
                    'y':           round(y, 4)},
            status=Status.RUNNING,
            reason='stepping forward along selected nav heading',
        )
        return Status.RUNNING
