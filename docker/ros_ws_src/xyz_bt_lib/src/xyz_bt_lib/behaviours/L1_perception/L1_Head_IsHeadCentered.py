#!/usr/bin/env python3
"""L1_Head_IsHeadCentered — condition: head pan (and optionally tilt) is near centre.

BB reads:
  /head_pan_pos  (root namespace) — fallback when pan_servo_id is None (legacy/line-following)
  /head_tilt_pos (root namespace) — fallback when tilt_servo_id is None and head_tilt_center set
  BB.SERVO_POSITIONS (/latched/servo_positions) — preferred; dict {servo_id: pos}
                                        used when pan_servo_id / tilt_servo_id are provided
                                        written by ServoPositionAdapter (real sensor values)

BB writes:
  none

Question judged:
  Is the head pan within center_threshold of head_pan_center?
  AND (if head_tilt_center set) is the head tilt within tilt_threshold of head_tilt_center?

Judgement helper:
  _is_centered(head_pan) → (passed, pan_offset, tilt_offset)

Behaviour (pure predicate):
  SUCCESS: centered
  FAILURE: not centered

For "centered stable for N ticks", wrap this node at the tree layer in
xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator — keep this node stateless.

CONFIG_DEFAULTS:
  head_pan_center:  500  — servo count for the head-forward pan position.
  center_threshold: 30   — max allowed pan offset from centre (servo counts).
                           Must match L2_Head_FindLineSweep center_threshold
                           when both nodes are used in the same tree.
  head_tilt_center: None — tilt servo centre; None = do not check tilt.
  tilt_threshold:   15   — max allowed tilt offset from centre (servo counts).
  pan_servo_id:     23   — servo ID to read pan from BB.SERVO_POSITIONS (default 23).
                           None = read BB.HEAD_PAN_POS (legacy/line-following fallback).
  tilt_servo_id:    24   — servo ID to read tilt from BB.SERVO_POSITIONS (default 24).
                           None = read BB.HEAD_TILT_POS (legacy/line-following fallback).
                           Only used when head_tilt_center is also set.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Head_IsHeadCentered(XyzL1ConditionNode):
    """SUCCESS when head pan (and optionally tilt) is within threshold of centre."""

    LEVEL = 'L1'
    BB_READS = [BB.SERVO_POSITIONS]  # also reads /head_pan_pos, /head_tilt_pos when pan/tilt_servo_id is None
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'head_pan_center':  500,
        'center_threshold': 30,
        'head_tilt_center': None,   # None = do not check tilt
        'tilt_threshold':   15,
        'pan_servo_id':     23,     # 23 = read from SERVO_POSITIONS (real feedback); None = legacy HEAD_PAN_POS
        'tilt_servo_id':    24,     # 24 = read from SERVO_POSITIONS (real feedback); None = legacy HEAD_TILT_POS
    }

    def __init__(self, name: str = 'L1_Head_IsHeadCentered',
                 head_pan_center: int = 500, center_threshold: int = 30,
                 head_tilt_center: int = None, tilt_threshold: int = 30,
                 pan_servo_id: int = 23, tilt_servo_id: int = 24,
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:             BT node name.
            head_pan_center:  Servo count for the head-forward pan position.
            center_threshold: Max allowed pan offset from centre (servo counts).
            head_tilt_center: Servo count for the head-forward tilt position.
                              None = do not check tilt servo.
            tilt_threshold:   Max allowed tilt offset from centre (servo counts).
            pan_servo_id:     Servo ID to read pan from BB.SERVO_POSITIONS (default 23).
                              None = read BB.HEAD_PAN_POS (legacy/line-following fallback).
            tilt_servo_id:    Servo ID to read tilt from BB.SERVO_POSITIONS (default 24).
                              None = read BB.HEAD_TILT_POS (legacy/line-following fallback).
                              Only used when head_tilt_center is also set.
            logger:           DebugEventLogger-compatible object, or None.
            tick_id_getter:   Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._head_pan_center  = head_pan_center
        self._center_threshold = center_threshold
        self._head_tilt_center = head_tilt_center
        self._tilt_threshold   = tilt_threshold
        self._pan_servo_id     = pan_servo_id
        self._tilt_servo_id    = tilt_servo_id
        self._bb               = None
        self._bb_tilt          = None
        self._bb_servo         = None  # latched client for SERVO_POSITIONS

    def setup(self, **kwargs):
        super().setup(**kwargs)

        # Pan source: servo feedback (preferred) or legacy root-ns BB key
        if self._pan_servo_id is None:
            # /head_pan_pos lives at root namespace, not /latched/ — no namespace arg.
            self._bb = self.attach_blackboard_client(name=self.name)
            self._bb.register_key(key='/head_pan_pos', access=Access.READ)

        # Tilt source: servo feedback (preferred) or legacy root-ns BB key
        if self._head_tilt_center is not None and self._tilt_servo_id is None:
            self._bb_tilt = self.attach_blackboard_client(name=f'{self.name}_tilt')
            self._bb_tilt.register_key(key='/head_tilt_pos', access=Access.READ)

        # Shared latched client for SERVO_POSITIONS (created when any servo ID is given)
        _need_servo = (self._pan_servo_id is not None or
                       (self._tilt_servo_id is not None and self._head_tilt_center is not None))
        if _need_servo:
            self._bb_servo = self.attach_blackboard_client(
                name=f'{self.name}_servo', namespace=BB.LATCHED_NS)
            self._bb_servo.register_key(key=BB.SERVO_POSITIONS_KEY, access=Access.READ)

    def _is_centered(self, head_pan: int) -> tuple:
        """Return (passed, pan_offset, tilt_offset).

        No BB reads/writes, ROS calls, or logger calls here.
        tilt_offset is 0 when tilt check is disabled, None when servo unreachable.
        """
        pan_offset = abs(head_pan - self._head_pan_center)
        pan_ok     = pan_offset <= self._center_threshold
        tilt_offset = 0
        tilt_ok     = True
        if self._head_tilt_center is not None:
            if self._tilt_servo_id is not None:
                head_tilt = (self._bb_servo.servo_positions or {}).get(self._tilt_servo_id)
                if head_tilt is None:
                    return False, pan_offset, None  # tilt servo unreachable
            else:
                head_tilt = self._bb_tilt.head_tilt_pos
            tilt_offset = abs(head_tilt - self._head_tilt_center)
            tilt_ok     = tilt_offset <= self._tilt_threshold
        return pan_ok and tilt_ok, pan_offset, tilt_offset

    def update(self) -> Status:
        # Read pan position from the configured source
        if self._pan_servo_id is not None:
            head_pan = (self._bb_servo.servo_positions or {}).get(self._pan_servo_id)
            if head_pan is None:
                self.emit_decision(
                    inputs={'pan_servo_id': self._pan_servo_id},
                    status=Status.FAILURE,
                    reason='servo position unavailable',
                )
                return Status.FAILURE
        else:
            head_pan = self._bb.head_pan_pos

        passed, pan_offset, tilt_offset = self._is_centered(head_pan)

        inputs = {'head_pan_pos': head_pan, 'pan_offset': pan_offset,
                  'center': self._head_pan_center,
                  'threshold': self._center_threshold}
        if self._head_tilt_center is not None:
            inputs['tilt_offset']    = tilt_offset
            inputs['tilt_threshold'] = self._tilt_threshold

        status = self.status_from_bool(passed)
        reason = 'centered' if passed else f'pan_off={pan_offset} tilt_off={tilt_offset}'

        self.emit_decision(inputs=inputs, status=status, reason=reason)
        return status
