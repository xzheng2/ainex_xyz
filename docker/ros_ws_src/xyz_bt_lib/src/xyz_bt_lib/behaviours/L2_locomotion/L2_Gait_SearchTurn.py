#!/usr/bin/env python3
"""L2 Action: turn the body in place to recover a lost target.

Node kind: continuous_controller

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects) — uses the target's last known
                      'error_x' to choose which way to turn, and 'lost_count' to
                      scale how hard.

BB writes: none
Facade:    turn_step(x, y, yaw)
Strategy:  _compute_search_turn(last_error_x, lost_count) → (gait_yaw, reason)

Direction is taken from the sign of the last known error_x:
  < 0  → target was to the LEFT of centre  → turn left  (positive yaw)
  >= 0 → target was to the RIGHT of centre → turn right (negative yaw)
Turn magnitude scales linearly with lost_count up to max_turn_deg — the longer
the target has been missing, the wider the sweep.

Why the "last known" value is simply error_x:
  While a target is lost, the tracking adapter preserves its previous entry and
  only increments lost_count, so tracked_objects[target]['error_x'] IS the last
  seen error. The old node needed a separate sticky /latched/last_line_error_x
  key for this; the unified registry makes that redundant.

Always returns RUNNING so a parent Selector keeps ticking this branch until a
higher-priority branch (target reacquired) takes over.

Generalised from L2_Gait_FindLine (Aug 2026): the algorithm was never
line-specific. Note this searches by turning the BODY; to search by sweeping the
HEAD while standing still, use L2_Head_SearchSweep instead.

CONFIG_DEFAULTS:
    target_id:        'line' — key to look up in tracked_objects
    base_turn_deg:    3      — turn angle (deg) just after losing the target
    max_turn_deg:     7      — maximum turn angle (deg) after count_scale_at frames
    count_scale_at:   30     — lost_count at which the turn reaches max_turn_deg
    default_turn_deg: 3      — turn angle when there is no history at all
    right_turn_deg:   5      — turn magnitude (deg) when the target was last to the right

Gait pass-through (period_time_ms / dsp_ratio / y_swap_amplitude / arm_swap /
step_num / gait_param): inherited from XyzL2GaitActionNode — see that class's
docstring.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2GaitActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L2_Gait_SearchTurn(XyzL2GaitActionNode):
    """Turn in place to recover a lost target. Always returns RUNNING."""

    LEVEL        = 'L2'
    BB_READS     = [BB.TRACKED_OBJECTS]
    BB_WRITES    = []
    FACADE_CALLS = ['turn_step']
    # Gait pass-through knobs are inherited from XyzL2GaitActionNode.
    # GAIT_CONFIG_DEFAULTS — do not repeat them here.
    CONFIG_DEFAULTS = {
        'target_id':        'line',
        'base_turn_deg':    3,
        'max_turn_deg':     7,
        'count_scale_at':   30,
        'default_turn_deg': 3,
        'right_turn_deg':   5,
    }

    def __init__(self, name: str = 'L2_Gait_SearchTurn',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 target_id: str = 'line',
                 base_turn_deg: int = 3,
                 max_turn_deg:  int = 7,
                 count_scale_at: int = 30,
                 default_turn_deg: int = 3,
                 right_turn_deg: int = 5,
                 period_time_ms: int = None,
                 dsp_ratio: float = None,
                 y_swap_amplitude: float = None,
                 arm_swap: int = None,
                 step_num: int = None,
                 gait_param: dict = None):
        """
        Args:
            target_id:        Key to look up in the tracked_objects dict.
            base_turn_deg:    Turn angle (deg) just after losing the target.
            max_turn_deg:     Maximum turn angle (deg) after count_scale_at frames.
            count_scale_at:   lost_count at which the turn reaches max_turn_deg.
            default_turn_deg: Turn angle when there is no history (never seen).
            right_turn_deg:   Turn magnitude (deg) when target was last to the right.

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
        self._target_id        = target_id
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
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)

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
            return turn_deg, f'target was left (err={last_error_x:.1f}), turn left'
        else:
            return (-self._right_turn_deg,
                    f'target was right (err={last_error_x:.1f}), '
                    f'turn right (yaw={-self._right_turn_deg})')

    def update(self) -> Status:
        obj        = (self._bb.tracked_objects or {}).get(self._target_id)
        last_err   = obj['error_x'] if obj else None
        lost_count = obj['lost_count'] if obj else 0

        gait_yaw, reason = self._compute_search_turn(last_err, lost_count)
        self.call_facade('turn_step', x=0, y=0, yaw=gait_yaw,
                         **self.gait_kwargs(),
                         semantic_source='search_turn')
        self.emit_decision(
            inputs={'target_id': self._target_id,
                    'last_error_x': last_err,
                    'lost_count': lost_count,
                    'gait_yaw': gait_yaw},
            status=Status.RUNNING,
            reason=reason,
        )
        return Status.RUNNING
