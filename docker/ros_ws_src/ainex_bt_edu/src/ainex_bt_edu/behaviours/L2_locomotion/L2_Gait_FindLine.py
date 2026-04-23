#!/usr/bin/env python3
"""L2 Action: turn in-place to recover a lost line.

BB reads:
  BB.LAST_LINE_ERROR_X  (/latched/last_line_error_x) — sticky signed offset from last
                          detected line, written by LineDetectionAdapter
  BB.CAMERA_LOST_COUNT  (/latched/camera_lost_count)  — frames since last line detection

BB writes: none
Facade:    turn_step(x, y, yaw)
Strategy:  _compute_search_turn(last_error_x, lost_count) → (gait_yaw, reason)

Direction is determined by the sign of last_line_error_x:
  < 0  → line was to the LEFT of centre  → turn left  (positive yaw)
  >= 0 → line was to the RIGHT of centre → turn right (negative yaw = -right_turn_deg)

Turn magnitude scales linearly with camera_lost_count up to max_turn_deg.

Always returns RUNNING so the Selector keeps ticking this node each cycle
until IsLineDetected (upstream Sequence) succeeds and takes over.
The StopWalking node behind it in the Selector is a structural fallback only.

CONFIG_DEFAULTS:
    base_turn_deg:    3  — turn angle (deg) just after losing the line
    max_turn_deg:     7  — maximum turn angle (deg) after count_scale_at frames
    count_scale_at:   30 — camera_lost_count at which turn reaches max_turn_deg
    default_turn_deg: 3  — turn angle when last_line_error_x history is absent
    right_turn_deg:   5  — turn angle magnitude (deg) when line was last to the right
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexL2ActionNode
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.blackboard_keys import BB


class L2_Gait_FindLine(AinexL2ActionNode):
    """Turn in-place to recover a lost line. Always returns RUNNING."""

    LEVEL        = 'L2'
    BB_READS     = [BB.LAST_LINE_ERROR_X, BB.CAMERA_LOST_COUNT]
    BB_WRITES    = []
    FACADE_CALLS = ['turn_step']
    CONFIG_DEFAULTS = {
        'base_turn_deg':    3,
        'max_turn_deg':     7,
        'count_scale_at':   30,
        'default_turn_deg': 3,
        'right_turn_deg':   5,
    }

    def __init__(self, name: str = 'L2_Gait_FindLine',
                 facade: AinexBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 base_turn_deg: int = 3,
                 max_turn_deg:  int = 7,
                 count_scale_at: int = 30,
                 default_turn_deg: int = 3,
                 right_turn_deg: int = 5):
        """
        Args:
            base_turn_deg:    Turn angle (deg) just after losing the line.
            max_turn_deg:     Maximum turn angle (deg) after count_scale_at frames.
            count_scale_at:   camera_lost_count at which turn reaches max_turn_deg.
            default_turn_deg: Turn angle when last_line_error_x history is absent.
            right_turn_deg:   Turn angle magnitude (deg) when line was last to the right.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._base_turn_deg    = base_turn_deg
        self._max_turn_deg     = max_turn_deg
        self._count_scale_at   = count_scale_at
        self._default_turn_deg = default_turn_deg
        self._right_turn_deg   = right_turn_deg
        self._bb               = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.LAST_LINE_ERROR_X_KEY, access=Access.READ)
        self._bb.register_key(key=BB.CAMERA_LOST_COUNT_KEY, access=Access.READ)

    def _compute_search_turn(self, last_error_x, lost_count: int) -> tuple:
        """Compute (gait_yaw, reason) for one search step.

        Direction: sign of last_error_x; magnitude: linear ramp with lost_count.
        No BB/ROS/facade calls here.
        """
        scale    = min(1.0, lost_count / max(self._count_scale_at, 1))
        turn_deg = int(self._base_turn_deg
                       + scale * (self._max_turn_deg - self._base_turn_deg))
        if last_error_x is None:
            return self._default_turn_deg, 'no history, default left turn'
        elif last_error_x < 0:
            return turn_deg, f'line was left (err={last_error_x:.1f}), turn left'
        else:
            return -self._right_turn_deg, f'line was right (err={last_error_x:.1f}), turn right (yaw={-self._right_turn_deg})'

    def update(self) -> Status:
        last_err   = self._bb.last_line_error_x
        lost_count = self._bb.camera_lost_count

        gait_yaw, reason = self._compute_search_turn(last_err, lost_count)
        self.call_facade('turn_step', x=0, y=0, yaw=gait_yaw,
                         semantic_source='search_line')
        self.emit_decision(
            inputs={'last_line_error_x': last_err,
                    'camera_lost_count': lost_count,
                    'gait_yaw': gait_yaw},
            status=Status.RUNNING,
            reason=reason,
        )
        return Status.RUNNING
