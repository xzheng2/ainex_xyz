#!/usr/bin/env python3
"""L2 Action: steer the body to keep a tracked target centred while walking.

Node kind: continuous_controller

Follows a target indefinitely — it never judges "arrived". For approach-and-stop
behaviour use L2_Gait_VisionToObject instead.

Pick this node when the path bends and the gait profile matters: it switches
between go_step and turn_step, so sharp corners get the rotation-biased profile
(this is what line following needs). Pick VisionToObject when you must stop at
the target, or when the heading has to be preserved while closing a lateral
offset (its steer_axis='lateral' mode). The two use different control laws, not
just different termination.

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects) — uses the target's 'error_x'
                      (signed pixel offset from the adapter-calibrated centre)
                      and 'lost_count'.

BB writes: none
Facade:    move_head(pan_pos), go_step(x, y, yaw), turn_step(x, y, yaw)
Strategy:  _compute_follow_step(err) → (x_out, yaw, profile)

Also centres the head each tick so it does not drift after a head sweep.

Algorithm (unchanged from the former L2_Gait_FollowLine):
  - yaw_out from error_x via deadband → linear ramp → saturation.
  - |yaw_out| < go_turn_threshold → go_step (straight walk profile),
    otherwise                     → turn_step (rotation-biased profile).
  - Forward speed drops to x_hi_yaw at high yaw so the robot does not over-run
    a corner.

Generalised from L2_Gait_FollowLine (Aug 2026): nothing in the algorithm was
line-specific, so it now takes a target_id and reads the unified
tracked_objects registry. Works for a line, a ball, or a tag.

The ramp width is now an explicit parameter. The old node ramped over
``line_data.width / 6``, where ``width`` was secretly the IMAGE width the vendor
line detector stuffs into that field — an implicit coupling that broke as soon
as the target was anything but a line. ``ramp_full_px`` defaults to 107 (= 640/6)
so line-following behaviour is unchanged.

Returns:
    SUCCESS → gait step dispatched
    FAILURE → target not visible (absent, or lost_count > lost_count_threshold),
              so a parent PrioritySelector can route to a search branch. The old
              node had no such guard and raised AttributeError if its upstream
              gate was ever missing.

CONFIG_DEFAULTS:
    target_id:            'line'      key to look up in tracked_objects
    x_range:              [0, 0.015]  forward speed range (m/step)
    yaw_range:            [-8, 10]    yaw saturation limits (deg)
    deadband_px:          10          |error_x| below this → yaw = 0
    ramp_full_px:         107         |error_x| at which yaw saturates (= old width/6)
    go_turn_threshold:    4           |yaw_out| below this → go profile
    head_pan_center:      500         servo centre position
    hi_yaw_threshold:     6           |yaw_out| above this → use x_hi_yaw speed
    x_hi_yaw:             0.008       forward speed (m/step) at high yaw
    lost_count_threshold: 0           tolerate this many missed frames before FAILURE

Gait pass-through (period_time_ms / dsp_ratio / y_swap_amplitude / arm_swap /
step_num / gait_param): inherited from XyzL2GaitActionNode — see that class's
docstring. They are forwarded to whichever profile (go_step / turn_step) this
node selects for the tick.
"""
import math
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2GaitActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L2_Gait_FollowTarget(XyzL2GaitActionNode):
    """Keep a tracked target centred while walking; never judges arrival."""

    LEVEL        = 'L2'
    BB_READS     = [BB.TRACKED_OBJECTS]
    BB_WRITES    = []
    FACADE_CALLS = ['move_head', 'go_step', 'turn_step']
    # Gait pass-through knobs are inherited from XyzL2GaitActionNode.
    # GAIT_CONFIG_DEFAULTS — do not repeat them here.
    CONFIG_DEFAULTS = {
        'target_id':            'line',
        'x_range':              [0, 0.015],
        'yaw_range':            [-8, 10],
        'deadband_px':          10,
        'ramp_full_px':         107,
        'go_turn_threshold':    4,
        'head_pan_center':      500,
        'hi_yaw_threshold':     6,
        'x_hi_yaw':             0.008,
        'lost_count_threshold': 0,
    }

    def __init__(self, name: str = 'L2_Gait_FollowTarget',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 target_id: str = 'line',
                 x_range: list = None,
                 yaw_range: list = None,
                 deadband_px: float = 10,
                 ramp_full_px: float = 107,
                 go_turn_threshold: float = 4,
                 head_pan_center: int = 500,
                 hi_yaw_threshold: float = 6,
                 x_hi_yaw: float = 0.008,
                 lost_count_threshold: int = 0,
                 period_time_ms: int = None,
                 dsp_ratio: float = None,
                 y_swap_amplitude: float = None,
                 arm_swap: int = None,
                 step_num: int = None,
                 gait_param: dict = None):
        """
        Args:
            target_id:            Key to look up in the tracked_objects dict.
            x_range:              [x_min, x_max] forward speed range (metres/step).
            yaw_range:            [yaw_min, yaw_max] yaw saturation limits (deg).
            deadband_px:          |error_x| below this → yaw = 0 (no correction).
            ramp_full_px:         |error_x| at which yaw reaches saturation. Default
                                  107 reproduces the old image-width/6 behaviour.
            go_turn_threshold:    |yaw_out| below this → go profile, else turn profile.
            head_pan_center:      Servo centre position (units).
            hi_yaw_threshold:     |yaw_out| above this → use x_hi_yaw forward speed.
            x_hi_yaw:             Forward speed (m/step) when yaw exceeds the threshold.
            lost_count_threshold: Missed frames tolerated before returning FAILURE.

        Gait pass-through (period_time_ms / dsp_ratio / y_swap_amplitude /
        arm_swap / step_num / gait_param): see XyzL2GaitActionNode.
        """
        super().__init__(
            name,
            facade=facade,
            logger=logger,
            tick_id_getter=tick_id_getter,
            period_time_ms=period_time_ms,
            dsp_ratio=dsp_ratio,
            y_swap_amplitude=y_swap_amplitude,
            arm_swap=arm_swap,
            step_num=step_num,
            gait_param=gait_param,
        )
        self._target_id       = target_id
        self._x_range         = x_range or [0, 0.015]
        self._yaw_range       = yaw_range or [-8, 10]
        self._deadband_px     = deadband_px
        self._ramp_full_px    = ramp_full_px
        self._go_turn_thr     = go_turn_threshold
        self._head_pan_center = head_pan_center
        self._hi_yaw_thr      = hi_yaw_threshold
        self._x_hi_yaw        = x_hi_yaw
        self._lost_count_threshold = lost_count_threshold
        self._bb              = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)

    def _compute_follow_step(self, err: float) -> tuple:
        """Compute (x_out, yaw, profile) from the signed pixel error.

        profile is 'go' or 'turn'. No BB/ROS/facade calls here.
        """
        r = self._ramp_full_px
        if abs(err) < self._deadband_px:
            yaw_out = 0.0
        elif abs(err) < r:
            # Linear ramp: ±1 deg at the deadband edge → ±(yaw_max-1) at ramp_full_px
            yaw_out = math.copysign(1, err) + self._lerp(
                err, -r, r,
                self._yaw_range[0] + 1, self._yaw_range[1] - 1,
            )
        else:
            yaw_out = math.copysign(self._yaw_range[1], err)

        # Reduce forward speed at high yaw to avoid overshooting corners
        x_out = self._x_hi_yaw if abs(yaw_out) > self._hi_yaw_thr else self._x_range[1]
        yaw   = int(-yaw_out)  # negate: positive error → turn left → negative yaw cmd
        profile = 'go' if abs(yaw_out) < self._go_turn_thr else 'turn'
        return x_out, yaw, profile

    def update(self) -> Status:
        obj = (self._bb.tracked_objects or {}).get(self._target_id)
        if obj is None or obj['lost_count'] > self._lost_count_threshold:
            self.emit_decision(
                inputs={'target_id': self._target_id,
                        'lost_count': obj['lost_count'] if obj else None},
                status=Status.FAILURE,
                reason=f'target {self._target_id!r} not visible',
            )
            return Status.FAILURE

        # Centre the head each tick while following. After a head sweep the servo
        # may still be off-centre even once the condition gate has passed, so this
        # keeps it driven toward centre for as long as we are following.
        self.call_facade('move_head', pan_pos=self._head_pan_center)

        error_x = obj['error_x']
        x_out, yaw, profile = self._compute_follow_step(error_x)
        self.call_facade(f'{profile}_step', x=x_out, y=0, yaw=yaw,
                         **self.gait_kwargs(),
                         semantic_source='follow_target')
        self.emit_decision(
            inputs={'target_id': self._target_id, 'error_x': error_x,
                    'x_out': x_out, 'yaw': yaw, 'profile': profile},
            status=Status.SUCCESS,
            reason=f'follow step dispatched ({profile})',
        )
        return Status.SUCCESS

    @staticmethod
    def _lerp(x, x0, x1, y0, y1):
        """Linear map x∈[x0,x1] → [y0,y1]. Pure Python; replaces ainex_sdk misc.val_map."""
        return (x - x0) / (x1 - x0) * (y1 - y0) + y0
