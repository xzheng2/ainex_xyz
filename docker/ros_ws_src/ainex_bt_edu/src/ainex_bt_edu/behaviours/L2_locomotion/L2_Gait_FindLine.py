#!/usr/bin/env python3
"""L2 Action: turn in-place to recover a lost line.

Reads /latched/last_line_error_x (sticky signed offset from the last detected
line, written by LineDetectionAdapter) and /latched/camera_lost_count to
bias the search direction and scale the turn magnitude.

Direction is determined by the sign of last_line_error_x:
  < 0  → line was to the LEFT of centre  → turn left  (positive yaw)
  >= 0 → line was to the RIGHT of centre → turn right (negative yaw)

Turn magnitude scales linearly with camera_lost_count up to max_turn_deg.

Always returns RUNNING so the Selector keeps ticking this node each cycle
until IsLineDetected (upstream Sequence) succeeds and takes over.
The StopWalking node behind it in the Selector is a structural fallback only.
"""
import rospy
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.blackboard_keys import BB


class L2_Gait_FindLine(AinexBTNode):
    """Turn in-place to recover a lost line. Always returns RUNNING."""

    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.LAST_LINE_ERROR_X, BB.CAMERA_LOST_COUNT]

    def __init__(self, name: str = 'L2_Gait_FindLine',
                 facade: AinexBTFacade = None, tick_id_getter=None,
                 base_turn_deg: int = 3,
                 max_turn_deg:  int = 7,
                 count_scale_at: int = 30,
                 default_turn_deg: int = 3):
        """
        Args:
            base_turn_deg:     Turn angle (deg) just after losing the line.
            max_turn_deg:      Maximum turn angle (deg) after count_scale_at frames.
            count_scale_at:    camera_lost_count at which turn reaches max_turn_deg.
            default_turn_deg:  Turn angle when last_line_error_x history is absent.
        """
        super().__init__(name)
        self._facade           = facade
        self._tick_id_getter   = tick_id_getter or (lambda: -1)
        self._base_turn_deg    = base_turn_deg
        self._max_turn_deg     = max_turn_deg
        self._count_scale_at   = count_scale_at
        self._default_turn_deg = default_turn_deg
        self._bb               = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.LAST_LINE_ERROR_X_KEY,  access=Access.READ)
        self._bb.register_key(key=BB.CAMERA_LOST_COUNT_KEY,  access=Access.READ)

    def update(self) -> Status:
        last_err   = self._bb.last_line_error_x
        lost_count = self._bb.camera_lost_count
        tid        = self._tick_id_getter()

        # Scale turn magnitude with lost_count
        scale    = min(1.0, lost_count / max(self._count_scale_at, 1))
        turn_deg = int(self._base_turn_deg
                       + scale * (self._max_turn_deg - self._base_turn_deg))

        # Determine direction from the sign of last_line_error_x
        if last_err is None:
            gait_yaw = self._default_turn_deg   # no history → default left turn
        elif last_err < 0:
            gait_yaw = turn_deg                  # line was LEFT  → turn left (positive)
        else:
            gait_yaw = -5                        # line was RIGHT → turn right (negative)

        rospy.logdebug('[L2_Gait_FindLine] gait_yaw=%+d lost=%d', gait_yaw, lost_count)
        self._facade.turn_step(
            x=0, y=0, yaw=gait_yaw,
            bt_node=self.name, tick_id=tid,
            semantic_source='search_line',
        )
        return Status.RUNNING
