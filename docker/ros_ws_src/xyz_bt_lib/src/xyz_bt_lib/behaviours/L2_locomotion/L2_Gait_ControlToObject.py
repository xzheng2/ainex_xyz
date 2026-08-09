#!/usr/bin/env python3
"""L2 Action: drive the robot toward a tracked object using real servo positions.

Node kind: dispatch

BB reads:
  BB.SERVO_POSITIONS (/latched/servo_positions) — dict {servo_id: pos} written by
                     ServoPositionAdapter; pan_servo_id (default 23) and
                     tilt_servo_id (default 24) are extracted each tick.

BB writes:
  none

Facade:
  go_step(x, y, yaw) — one step per tick toward the object

Strategy:
  Reads real servo feedback from BB.SERVO_POSITIONS (written by ServoPositionAdapter)
  and computes a proportional gait step to align the body with the ball:

  yaw  = proportional to pan_offset  (pan_pos − head_pan_center)
         → head turned left  (pan_pos > 500) → positive offset → positive yaw → turn left
         → head turned right (pan_pos < 500) → negative offset → negative yaw → turn right
  x    = 0            if |tilt_pos − head_tilt_center| ≤ tilt_threshold
         +x_speed     if tilt_pos > head_tilt_center + tilt_threshold  (ball far)
         −x_speed     if tilt_pos < head_tilt_center − tilt_threshold  (ball close)

  One go_step(x, y=0, yaw) is issued per tick.

  The parent tree gates this node with L1_Head_IsHeadCentered:

    Selector:
      Sequence:  L1_Head_IsHeadCentered   ← SUCCESS when body aligned
      L2_Gait_ControlToObject             ← drives body until centred

Dispatch semantics:
  Always returns SUCCESS.  Never RUNNING or FAILURE.

Returns:
  SUCCESS always  (dispatch — never returns RUNNING or FAILURE)

CONFIG_DEFAULTS:
  pan_servo_id:     23      — servo ID to read pan position from BB.SERVO_POSITIONS
  tilt_servo_id:    24      — servo ID to read tilt position from BB.SERVO_POSITIONS
  x_speed:          0.020   — step magnitude (meters); used as ±x_speed
  kp:               0.1     — pan servo offset (units) → gait yaw (°)
  max_yaw_deg:      10      — maximum absolute gait yaw (°)
  deadband:         10      — servo units — pan offset inside which yaw = 0
  head_pan_center:  500     — servo position for "centred" (pan reference)
  head_tilt_center: 450     — servo position for ideal distance (tilt reference)
  tilt_threshold:   20      — servo units — tilt offset inside which x = 0
  period_time_ms:   None    — gait cycle (ms); None = project default (300)
  dsp_ratio:        None    — double-support fraction (0–1); None = project default (0.2)
  y_swap_amplitude: None    — lateral body swing (m); None = project default (0.02 m)
  arm_swap:         None    — arm swing amplitude (°); None = project default (30°)
  step_num:         None    — steps per tick (0 = continuous); None = project default (0)
  gait_param:       None    — partial WalkingParam dict (e.g. {'step_height': 0.03})
  init_yaw_offset:  None    — constant per-step heading bias (deg); None = controller default (0)
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L2_Gait_ControlToObject(XyzL2ActionNode):
    """Proportional gait step toward object using BB head positions. Always SUCCESS."""

    LEVEL        = 'L2'
    BB_READS     = [BB.SERVO_POSITIONS]
    BB_WRITES    = []
    FACADE_CALLS = ['go_step']
    CONFIG_DEFAULTS = {
        'pan_servo_id':     23,
        'tilt_servo_id':    24,
        'x_speed':          0.020,
        'kp':               0.1,
        'max_yaw_deg':      10,
        'deadband':         10,
        'head_pan_center':  500,
        'head_tilt_center': 450,
        'tilt_threshold':   20,
        # Gait parameter overrides (None = use project _GO_STEP_VEL / _GO_CFG defaults)
        'period_time_ms':   None,   # step_vel[0]: gait cycle (ms)
        'dsp_ratio':        None,   # step_vel[1]: double-support fraction (0–1)
        'y_swap_amplitude': None,   # step_vel[2]: lateral body swing (m)
        'arm_swap':         None,   # arm swing (degrees); None → _GO_CFG → 30
        'step_num':         None,   # steps per tick; None → _GO_CFG → 0 (continuous)
        'gait_param':       None,   # partial WalkingParam dict (e.g. {'step_height': 0.03})
        'init_yaw_offset':  None,   # constant per-step heading bias (deg)
    }

    def __init__(self, name: str = 'L2_Gait_ControlToObject',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 pan_servo_id: int = 23,
                 tilt_servo_id: int = 24,
                 x_speed: float = 0.020,
                 kp: float = 0.1,
                 max_yaw_deg: int = 10,
                 deadband: int = 10,
                 head_pan_center: int = 500,
                 head_tilt_center: int = 450,
                 tilt_threshold: int = 20,
                 period_time_ms: int = None,
                 dsp_ratio: float = None,
                 y_swap_amplitude: float = None,
                 arm_swap: int = None,
                 step_num: int = None,
                 gait_param: dict = None,
                 init_yaw_offset: float = None):
        """
        CONFIG_DEFAULTS:
            pan_servo_id:     Servo ID for pan position in BB.SERVO_POSITIONS (default 23).
            tilt_servo_id:    Servo ID for tilt position in BB.SERVO_POSITIONS (default 24).
            x_speed:          Step magnitude (meters); emitted as +x_speed or -x_speed.
            kp:               Pan servo offset → gait yaw (°).
            max_yaw_deg:      Maximum absolute gait yaw (°).
            deadband:         Servo units — pan offset inside which yaw = 0.
            head_pan_center:  Servo position for centred (pan reference, 500 = centre).
            head_tilt_center: Servo position for ideal distance (tilt reference).
            tilt_threshold:   Servo units — tilt offset inside which x = 0.
            period_time_ms:   Gait cycle (ms). None = project default (300).
            dsp_ratio:        Double-support fraction (0–1). None = project default (0.2).
            y_swap_amplitude: Lateral body swing (m). None = project default (0.02 m).
            arm_swap:         Arm swing amplitude (°). None = project default (30°).
            step_num:         Steps per tick (0 = continuous). None = project default (0).
            gait_param:       Partial WalkingParam dict (e.g. {'step_height': 0.03}).
                              None = use controller default.
            init_yaw_offset:  Constant per-step heading bias (deg). None = controller
                              default (0). Merged into gait_param as 'init_yaw_offset';
                              applied by the walking module as a persistent yaw offset on
                              every step (distinct from the per-tick steering yaw). Validate
                              sign on the robot.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._pan_servo_id      = pan_servo_id
        self._tilt_servo_id     = tilt_servo_id
        self._x_speed           = x_speed
        self._kp                = kp
        self._max_yaw_deg       = max_yaw_deg
        self._deadband          = deadband
        self._head_pan_center   = head_pan_center
        self._head_tilt_center  = head_tilt_center
        self._tilt_threshold    = tilt_threshold
        self._period_time_ms    = period_time_ms
        self._dsp_ratio         = dsp_ratio
        self._y_swap_amplitude  = y_swap_amplitude
        self._arm_swap          = arm_swap
        self._step_num          = step_num
        self._gait_param        = gait_param
        self._init_yaw_offset   = init_yaw_offset
        self._bb                = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.SERVO_POSITIONS_KEY, access=Access.READ)

    def _effective_gait_param(self):
        """Merge the named init_yaw_offset knob into the partial gait_param dict.

        Returns None when nothing overrides (controller defaults)."""
        gp = dict(self._gait_param) if self._gait_param else {}
        if self._init_yaw_offset is not None:
            gp['init_yaw_offset'] = self._init_yaw_offset   # per-step heading bias (deg)
        return gp or None

    def _compute_yaw(self, pan_pos: int) -> int:
        """Return signed gait yaw from head pan servo position.

        pan_offset = pan_pos - head_pan_center
        Negative offset (head turned right, ball right) → negative yaw → turn right.
        Positive offset (head turned left,  ball left)  → positive yaw → turn left.
        Applies deadband, proportional gain, clamp.
        """
        pan_offset = pan_pos - self._head_pan_center
        if abs(pan_offset) <= self._deadband:
            return 0
        raw = pan_offset * self._kp
        return int(max(-self._max_yaw_deg, min(self._max_yaw_deg, raw)))

    def _compute_x(self, tilt_pos: int) -> float:
        """Return signed step magnitude based on tilt offset vs center.

        Within deadband (|offset| <= tilt_threshold): x = 0.
        Above center (ball far):   +x_speed.
        Below center (ball close): -x_speed.
        """
        tilt_offset = tilt_pos - self._head_tilt_center
        if abs(tilt_offset) <= self._tilt_threshold:
            return 0.0
        return self._x_speed if tilt_offset > 0 else -self._x_speed

    def initialise(self):
        self.emit_action_intent(
            action='control_to_object',
            inputs={'head_pan_center':  self._head_pan_center,
                    'head_tilt_center': self._head_tilt_center,
                    'dsp_ratio':        self._dsp_ratio,
                    'period_time_ms':   self._period_time_ms,
                    'step_height':      (self._gait_param or {}).get('step_height')},
        )

    def update(self) -> Status:
        positions = self._bb.servo_positions or {}
        pan_pos   = positions.get(self._pan_servo_id)
        tilt_pos  = positions.get(self._tilt_servo_id)

        if pan_pos is None or tilt_pos is None:
            self.emit_decision(
                inputs={'pan_servo_id': self._pan_servo_id,
                        'tilt_servo_id': self._tilt_servo_id},
                status=Status.SUCCESS,
                reason='servo positions unavailable',
            )
            return Status.SUCCESS

        yaw = self._compute_yaw(pan_pos)
        x   = self._compute_x(tilt_pos)

        pan_offset  = pan_pos - self._head_pan_center
        tilt_offset = tilt_pos - self._head_tilt_center

        self.call_facade('go_step', x=x, y=0, yaw=yaw,
                         period_time_ms=self._period_time_ms,
                         dsp_ratio=self._dsp_ratio,
                         y_swap_amplitude=self._y_swap_amplitude,
                         arm_swap=self._arm_swap,
                         step_num=self._step_num,
                         gait_param=self._effective_gait_param(),
                         semantic_source='control_to_object')
        self.emit_decision(
            inputs={'pan_pos': pan_pos, 'pan_offset': pan_offset,
                    'tilt_pos': tilt_pos, 'tilt_offset': tilt_offset,
                    'x': x, 'yaw': yaw},
            status=Status.SUCCESS,
            reason='go_step proportional to head servo offsets',
        )
        return Status.SUCCESS
