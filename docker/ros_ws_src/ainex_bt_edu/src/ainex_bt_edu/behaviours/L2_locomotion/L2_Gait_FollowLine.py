#!/usr/bin/env python3
"""L2 Action: read line_error_x + line_data from blackboard and follow the line.

BB reads:
  BB.LINE_DATA    (/latched/line_data)     — detection result (.width used for ramp threshold)
  BB.LINE_ERROR_X (/latched/line_error_x)  — pre-computed signed offset from calibrated centre,
                    written by LineDetectionAdapter

BB writes: none
Facade:    move_head(pan_pos), go_step(x, y, yaw), turn_step(x, y, yaw)
Strategy:  _compute_follow_step(err, w) → (x_out, yaw, profile)

Also centres the head each tick so it does not drift in head-sweep mode.
Returns SUCCESS every tick (the Sequence upstream gates entry via IsLineDetected).

Algorithm (from former VisualPatrol.compute_follow_command):
  - Compute yaw_out from line_error_x using deadband + linear ramp + saturation.
  - If |yaw_out| < go_turn_threshold → go_step (straight walk profile).
  - Otherwise                        → turn_step (rotation-biased profile).
  - x_output is reduced at high yaw to keep the robot from over-running a corner.

All algorithm tuning params are constructor defaults; center_x_offset is NOT
a parameter here — it is baked into line_error_x by LineDetectionAdapter.

Returns:
    SUCCESS → gait step dispatched (always)

CONFIG_DEFAULTS:
    x_range:           [0, 0.015]  forward speed range (m/step)
    yaw_range:         [-8, 10]    yaw saturation limits (deg)
    deadband_px:       10          |error| below this → yaw = 0
    go_turn_threshold: 4           |yaw_out| below this → go profile
    head_pan_center:   500         servo centre position
    hi_yaw_threshold:  6           |yaw_out| above this → use x_hi_yaw speed
    x_hi_yaw:          0.008       forward speed (m/step) at high yaw
"""
import math
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexL2ActionNode
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.blackboard_keys import BB


class L2_Gait_FollowLine(AinexL2ActionNode):
    """Read line_error_x from BB and run one visual-patrol step via facade."""

    LEVEL        = 'L2'
    BB_READS     = [BB.LINE_DATA, BB.LINE_ERROR_X]
    BB_WRITES    = []
    FACADE_CALLS = ['move_head', 'go_step', 'turn_step']
    CONFIG_DEFAULTS = {
        'x_range':           [0, 0.015],
        'yaw_range':         [-8, 10],
        'deadband_px':       10,
        'go_turn_threshold': 4,
        'head_pan_center':   500,    # servo centre position
        'hi_yaw_threshold':  6,      # |yaw_out| above this → use x_hi_yaw speed
        'x_hi_yaw':          0.008,  # forward speed (m/step) at high yaw
    }

    def __init__(self, name: str = 'L2_Gait_FollowLine',
                 facade: AinexBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 x_range: list = None,
                 yaw_range: list = None,
                 deadband_px: float = 10,
                 go_turn_threshold: float = 4,
                 head_pan_center: int = 500,
                 hi_yaw_threshold: float = 6,
                 x_hi_yaw: float = 0.008):
        """
        Args:
            x_range:           [x_min, x_max] forward speed range (metres/step).
            yaw_range:         [yaw_min, yaw_max] yaw output saturation limits (deg).
            deadband_px:       |line_error_x| below this → yaw = 0 (no correction).
            go_turn_threshold: |yaw_out| below this → go profile, else turn profile.
            head_pan_center:   servo centre position (units).
            hi_yaw_threshold:  |yaw_out| above this → use x_hi_yaw forward speed.
            x_hi_yaw:          forward speed (m/step) when yaw exceeds hi_yaw_threshold.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._x_range         = x_range or [0, 0.015]
        self._yaw_range       = yaw_range or [-8, 10]
        self._deadband_px     = deadband_px
        self._go_turn_thr     = go_turn_threshold
        self._head_pan_center = head_pan_center
        self._hi_yaw_thr      = hi_yaw_threshold
        self._x_hi_yaw        = x_hi_yaw
        self._bb              = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.LINE_DATA_KEY,    access=Access.READ)
        self._bb.register_key(key=BB.LINE_ERROR_X_KEY, access=Access.READ)

    def _compute_follow_step(self, err: float, w: float) -> tuple:
        """Compute (x_out, yaw, profile) from line error and line width.

        profile is 'go' or 'turn'.
        No BB/ROS/facade calls here.
        """
        if abs(err) < self._deadband_px:
            yaw_out = 0.0
        elif abs(err) < w / 6:
            # Linear ramp: ±1 deg at deadband edge → ±(yaw_max-1) at w/6
            yaw_out = math.copysign(1, err) + self._lerp(
                err,
                -w / 6, w / 6,
                self._yaw_range[0] + 1, self._yaw_range[1] - 1,
            )
        else:
            # Saturate at yaw_max beyond w/6
            yaw_out = math.copysign(self._yaw_range[1], err)

        # Reduce forward speed at high yaw to avoid overshooting corners
        x_out = self._x_hi_yaw if abs(yaw_out) > self._hi_yaw_thr else self._x_range[1]
        yaw   = int(-yaw_out)  # negate: positive error → turn left → negative yaw cmd
        profile = 'go' if abs(yaw_out) < self._go_turn_thr else 'turn'
        return x_out, yaw, profile

    def update(self) -> Status:
        line_data    = self._bb.line_data
        line_error_x = self._bb.line_error_x  # pre-computed by LineDetectionAdapter

        # Centre the head each tick while following.  In head-sweep mode
        # IsHeadCentered may pass before the servo physically reaches centre,
        # so this keeps the head driven toward centre during line-following.
        self.call_facade('move_head', pan_pos=self._head_pan_center)

        x_out, yaw, profile = self._compute_follow_step(line_error_x, line_data.width)
        self.call_facade(f'{profile}_step', x=x_out, y=0, yaw=yaw,
                         semantic_source='follow_line')
        self.emit_decision(
            inputs={'line_error_x': line_error_x, 'width': line_data.width,
                    'x_out': x_out, 'yaw': yaw, 'profile': profile},
            status=Status.SUCCESS,
            reason=f'follow step dispatched ({profile})',
        )
        return Status.SUCCESS

    @staticmethod
    def _lerp(x, x0, x1, y0, y1):
        """Linear map x∈[x0,x1] → [y0,y1]. Pure Python; replaces ainex_sdk misc.val_map."""
        return (x - x0) / (x1 - x0) * (y1 - y0) + y0
