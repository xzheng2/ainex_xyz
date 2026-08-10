#!/usr/bin/env python3
"""L2 Action: turn the body to face where the head is looking, recentring the head.

Node kind: finite_action

After a head sweep finds a target, the head is pointing at it but the BODY is
not. This node closes that gap: each tick it turns the body toward the head's
direction and steps the head back toward centre, finishing when the head is
centred — at which point the robot faces the target and downstream nodes that
assume a centred head (L2_Gait_FollowTarget, L2_Gait_VisionToObject) can take
over.

    ReactiveSequence(Reacquire)
        L2_Head_SearchSweep      -> SUCCESS when the target appears
        L2_Gait_AlignBodyToHead  -> SUCCESS when the body has caught up
        L2_Gait_FollowTarget

Extracted from the ALIGN phase of L2_Head_FindLineSweep (deleted Aug 2026),
where it was welded to that node's sweep state machine and therefore unusable
on its own.

Not to be confused with L2_Gait_ControlToObject, which computes the same kind of
head-offset-proportional body yaw but is a continuous controller with no
terminating condition and no head recentring — it steers indefinitely while
something else decides when to stop. This node's whole purpose is to finish.

BB reads:
  BB.SERVO_POSITIONS  (/latched/servo_positions) — real pan feedback, dict {id: pos}

BB writes: none
Facade:    move_head(pan_pos), turn_step(x, y, yaw)
Strategy:  _compute_align_turn(head_offset) -> gait_yaw (int)

The body yaw is derived from the HEAD SERVO offset, not from camera pixels: the
question being answered is "how far round is the head from straight ahead", and
the servo knows that directly. Magnitude is scaled linearly from
align_turn_min_deg (head near centre) to align_turn_max_deg (head at the full
sweep extent), so the body keeps turning at a useful rate throughout.

Returns:
    SUCCESS → |pan - head_pan_center| <= center_threshold; head snapped to centre
    RUNNING → still turning
    FAILURE → pan servo position unavailable (no feedback to align against)

CONFIG_DEFAULTS:
    pan_servo_id:       23  — servo ID read from BB.SERVO_POSITIONS
    head_pan_center:    500 — servo position counted as straight ahead
    center_threshold:   30  — ±units around centre that count as aligned
    align_step:         7   — servo units the head moves toward centre per tick
    align_turn_min_deg: 3   — body-turn yaw (deg) when the head is near centre
    align_turn_max_deg: 8   — body-turn yaw (deg) when the head is at full extent
    pan_full_extent:    200 — |pan - centre| treated as "full extent" for scaling
                              (the old node used sweep_left_pos - head_pan_center)

Gait pass-through (period_time_ms / dsp_ratio / y_swap_amplitude / arm_swap /
step_num / gait_param): inherited from XyzL2GaitActionNode — see that class's
docstring.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2GaitActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L2_Gait_AlignBodyToHead(XyzL2GaitActionNode):
    """Turn the body to the head's direction and recentre the head; SUCCESS when aligned."""

    LEVEL        = 'L2'
    BB_READS     = [BB.SERVO_POSITIONS]
    BB_WRITES    = []
    FACADE_CALLS = ['move_head', 'turn_step']
    PAN_INVERT   = 1     # +1: increasing pan servo counts turn the head LEFT
    # Gait pass-through knobs are inherited from XyzL2GaitActionNode.
    # GAIT_CONFIG_DEFAULTS — do not repeat them here.
    CONFIG_DEFAULTS = {
        'pan_servo_id':       23,
        'head_pan_center':    500,
        'center_threshold':   30,
        'align_step':         7,
        'align_turn_min_deg': 3,
        'align_turn_max_deg': 8,
        'pan_full_extent':    200,
    }

    def __init__(self, name: str = 'L2_Gait_AlignBodyToHead',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 pan_servo_id: int = 23,
                 head_pan_center: int = 500,
                 center_threshold: int = 30,
                 align_step: int = 7,
                 align_turn_min_deg: int = 3,
                 align_turn_max_deg: int = 8,
                 pan_full_extent: int = 200,
                 period_time_ms: int = None,
                 dsp_ratio: float = None,
                 y_swap_amplitude: float = None,
                 arm_swap: int = None,
                 step_num: int = None,
                 gait_param: dict = None):
        """
        Args:
            pan_servo_id:       Servo ID to read pan feedback from BB.SERVO_POSITIONS.
            head_pan_center:    Servo position that means straight ahead.
            center_threshold:   ±units around centre that count as aligned → SUCCESS.
                                Keep it consistent with any L1_Head_IsHeadCentered
                                used in the same tree.
            align_step:         Servo units the head steps toward centre each tick.
            align_turn_min_deg: Body yaw (deg) when the head is near centre.
            align_turn_max_deg: Body yaw (deg) when the head is at full extent.
            pan_full_extent:    |pan - centre| treated as full extent when scaling
                                the yaw. Should match the sweep half-range used by
                                L2_Head_SearchSweep (sweep_left_pos - centre).

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
        self._pan_servo_id       = pan_servo_id
        self._head_pan_center    = head_pan_center
        self._center_threshold   = center_threshold
        self._align_step         = align_step
        self._align_turn_min_deg = align_turn_min_deg
        self._align_turn_max_deg = align_turn_max_deg
        self._pan_full_extent    = max(1, pan_full_extent)
        self._bb                 = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.SERVO_POSITIONS_KEY, access=Access.READ)

    def _compute_align_turn(self, head_offset: int) -> int:
        """Return the signed gait yaw for one alignment step.

        Magnitude scales linearly from align_turn_min_deg to align_turn_max_deg
        over |head_offset| in [0, pan_full_extent]. Sign follows the head
        direction. No BB/ROS/facade calls here.
        """
        norm = min(1.0, abs(head_offset) / float(self._pan_full_extent))
        magnitude = (self._align_turn_min_deg
                     + (self._align_turn_max_deg - self._align_turn_min_deg) * norm)
        return int(magnitude) * (1 if head_offset * self.PAN_INVERT > 0 else -1)

    def _step_head_toward_centre(self, pan: int) -> int:
        """Return the next head pan position, never overshooting centre."""
        offset = pan - self._head_pan_center
        step   = -self._align_step if offset > 0 else self._align_step
        new_pan = pan + step
        return (min(new_pan, self._head_pan_center) if step > 0
                else max(new_pan, self._head_pan_center))

    def update(self) -> Status:
        pan = (self._bb.servo_positions or {}).get(self._pan_servo_id)
        if pan is None:
            self.emit_decision(
                inputs={'pan_servo_id': self._pan_servo_id},
                status=Status.FAILURE,
                reason='pan servo position unavailable',
            )
            return Status.FAILURE

        head_offset = pan - self._head_pan_center

        if abs(head_offset) <= self._center_threshold:
            # Snap to exact centre so downstream head-centred assumptions hold.
            self.call_facade('move_head', pan_pos=self._head_pan_center)
            self.emit_decision(
                inputs={'head_pan': pan, 'head_offset': head_offset},
                status=Status.SUCCESS,
                reason='head centred, body aligned',
            )
            return Status.SUCCESS

        new_pan  = self._step_head_toward_centre(pan)
        gait_yaw = self._compute_align_turn(head_offset)
        self.call_facade('move_head', pan_pos=int(new_pan))
        self.call_facade('turn_step', x=0, y=0, yaw=gait_yaw,
                         **self.gait_kwargs(),
                         semantic_source='align_body_to_head')
        self.emit_decision(
            inputs={'head_pan': pan, 'next_pan': new_pan,
                    'head_offset': head_offset, 'gait_yaw': gait_yaw},
            status=Status.RUNNING,
            reason='turning body toward head direction',
        )
        return Status.RUNNING
