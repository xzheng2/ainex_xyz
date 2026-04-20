#!/usr/bin/env python3
"""L2 Action: read line_error_x + line_data from blackboard and follow the line.

Reads /latched/line_error_x (pre-computed signed offset from line_center_x,
written by LineDetectionAdapter) and /latched/line_data (for width).
Also centres the head each tick so it does not drift in head-sweep mode.
Returns SUCCESS every tick (the Sequence upstream gates entry via IsLineDetected).

Algorithm (inlined from the former VisualPatrol.compute_follow_command):
  - Compute yaw_out from line_error_x using deadband + linear ramp + saturation.
  - If |yaw_out| < go_turn_threshold → facade.go_step (straight walk profile).
  - Otherwise                        → facade.turn_step (rotation-biased profile).
  - x_output is reduced at high yaw to keep the robot from over-running a corner.

All algorithm tuning params are constructor defaults; center_x_offset is NOT
a parameter here — it is baked into line_error_x by LineDetectionAdapter.
"""
import math
import rospy
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.blackboard_keys import BB


class L2_Gait_FollowLine(AinexBTNode):
    """Read line_error_x from BB and run one visual-patrol step via facade."""

    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.LINE_DATA, BB.LINE_ERROR_X]

    HEAD_PAN_CENTER = 500  # servo units; centre position

    def __init__(self, name: str = 'L2_Gait_FollowLine',
                 facade: AinexBTFacade = None, tick_id_getter=None,
                 x_range: list = None,
                 yaw_range: list = None,
                 deadband_px: float = 10,
                 go_turn_threshold: float = 4):
        """
        Args:
            x_range:           [x_min, x_max] forward speed range (metres/step).
            yaw_range:         [yaw_min, yaw_max] yaw output saturation limits (deg).
            deadband_px:       |line_error_x| below this → yaw = 0 (no correction).
            go_turn_threshold: |yaw_out| below this → go profile, else turn profile.
        """
        super().__init__(name)
        self._facade          = facade
        self._tick_id_getter  = tick_id_getter or (lambda: -1)
        self._x_range         = x_range or [0, 0.015]
        self._yaw_range       = yaw_range or [-8, 10]
        self._deadband_px     = deadband_px
        self._go_turn_thr     = go_turn_threshold
        self._bb              = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.LINE_DATA_KEY,    access=Access.READ)
        self._bb.register_key(key=BB.LINE_ERROR_X_KEY, access=Access.READ)

    def update(self) -> Status:
        line_data    = self._bb.line_data
        line_error_x = self._bb.line_error_x  # pre-computed by LineDetectionAdapter
        tid          = self._tick_id_getter()

        # Centre the head each tick while following.  In head-sweep mode
        # IsHeadCentered may pass before the servo physically reaches centre,
        # so this keeps the head driven toward centre during line-following.
        self._facade.move_head(
            pan_pos=self.HEAD_PAN_CENTER,
            bt_node=self.name,
            tick_id=tid,
        )

        # ── Visual-patrol algorithm ────────────────────────────────────────
        # Inputs: line_error_x (signed pixel offset from calibrated centre),
        #         line_data.width (for proportional threshold).
        w   = line_data.width
        err = line_error_x

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
        x_out = 0.008 if abs(yaw_out) > 6 else self._x_range[1]
        yaw   = int(-yaw_out)  # negate: positive error → turn left → negative yaw cmd

        rospy.loginfo('[L2_Gait_FollowLine] err=%.1f w=%d yaw=%+d', err, w, yaw)

        if abs(yaw_out) < self._go_turn_thr:
            self._facade.go_step(
                x=x_out, y=0, yaw=yaw,
                bt_node=self.name, tick_id=tid,
                semantic_source='follow_line',
            )
        else:
            self._facade.turn_step(
                x=x_out, y=0, yaw=yaw,
                bt_node=self.name, tick_id=tid,
                semantic_source='follow_line',
            )
        return Status.SUCCESS

    @staticmethod
    def _lerp(x, x0, x1, y0, y1):
        """Linear map x∈[x0,x1] → [y0,y1]. Pure Python; replaces ainex_sdk misc.val_map."""
        return (x - x0) / (x1 - x0) * (y1 - y0) + y0
