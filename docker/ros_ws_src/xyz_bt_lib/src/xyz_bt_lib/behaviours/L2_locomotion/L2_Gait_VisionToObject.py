#!/usr/bin/env python3
"""L2 Action: walk a detected target onto a pixel target point, then stop.

Node kind: finite_action

Approaches and ARRIVES. For following a target indefinitely with no arrival
judgement, use L2_Gait_FollowTarget instead.

Pick this node when you must stop at the target, or when the body heading has to
stay fixed while a lateral offset is closed (steer_axis='lateral'). Pick
FollowTarget when the path bends and corners need the rotation-biased gait
profile — it switches between go_step and turn_step, which this node never does.
The two use different control laws (proportional here, a piecewise ramp there),
not just different termination.

Precondition — the head must be at the pan centre:
  This node steers the BODY from the target's horizontal pixel error, which is
  only a valid body-heading signal while the head faces straight ahead. It
  therefore commands the head to pan_default once in initialise() and leaves it
  there. If the head is turned (e.g. straight after a sweep) run
  L2_Gait_AlignBodyToHead first. Tilt is optional: it is parked at tilt_default
  and defines the frame geometry that align_y is measured against.

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects) — 'x', 'y', 'error_x', 'size',
                      'lost_count' for target_id.

BB writes: none

Facade:
  move_head(pan_pos, tilt_pos)  — park the head once on initialise()
  go_step(x, y, yaw)            — one step per tick while RUNNING
  stop_gait()                   — on SUCCESS (arrived)

Alignment target:
  error_x = obj['x'] - align_x        when align_x is set, else obj['error_x']
                                      (the adapter-calibrated frame centre)
  error_y = obj['y'] - align_y        only when align_y is set

  align_x exists for deliberately off-centre alignment — e.g. placing a ball on
  the kicking foot's column rather than the body midline. Leaving it None simply
  targets the calibrated centre; that is a default, not a mode.

  align_y is the distance proxy. With the head tilted down at a fixed angle, an
  object higher in the frame is further away, so error_y drives fore/aft motion
  AND supplies the arrival gate. Without align_y there is no distance
  information at all, so the node aligns laterally and never advances.

Horizontal steering (steer_axis):
  'yaw' (default)  — yaw = -kp * error_x; deadband, then clamp to ±max_yaw_deg.
                     Turning changes the body heading.
  'lateral'        — yaw forced to 0; strafe instead: y = -lateral_kp * error_x,
                     deadband, clamp to ±max_y_speed, optional min_y_speed floor.
                     Heading is preserved while the target is centred in x.

Termination:
  |error_x| <= x_error_threshold AND (align_y unset OR
    |error_y| <= y_error_threshold)                  → stop_gait(), SUCCESS
  size > max_size_threshold (safety, only when size is known)
                                                     → step back, RUNNING
  |error_y| <= y_error_threshold                     → steer only (x=0), RUNNING
  error_y < 0  (target high in frame → far)          → step forward, RUNNING
  error_y > 0  (target low in frame → near)          → step back, RUNNING
  target absent or lost_count > lost_count_threshold → FAILURE, so a parent
                                                       Selector can re-find it

Size is no longer an arrival criterion (Aug 2026). The old size>=size_threshold
gate was replaced by pixel alignment: with the head locked at centre, error_x/
error_y are the correct signals, whereas a pixel area threshold has to be
retuned for every object and is undefined for shapes such as a line
(tracked_objects reports size=None for those). Only the max_size_threshold
"too close, back off" safety override still consults size, and it is skipped
when size is None.

CONFIG_DEFAULTS:
  target_id:           'object' — key to look up in tracked_objects
  align_x:             None     — pixel column to align to; None = calibrated centre
  align_y:             None     — pixel row to align to; None = no approach, x-align only
  x_error_threshold:   50       — max |error_x| (px) counted as aligned
  y_error_threshold:   50       — max |error_y| (px) counted as aligned
  max_size_threshold:  250000.0 — size above this → back off; None disables; skipped
                                  when the shape has no size
  x_speed:             0.010    — forward step magnitude (m)
  y_speed:             0.0      — lateral step magnitude (m); sign tracks yaw sign
  kp:                  0.2      — error_x (px) → gait yaw (deg) gain
  max_yaw_deg:         10       — maximum |gait yaw| (deg)
  deadband:            5        — pixel deadband on error_x
  min_yaw_deg:         0        — minimum |yaw| outside the deadband (0 = proportional)
  steer_axis:          'yaw'    — 'yaw' or 'lateral' (see above)
  lateral_kp:          0.0      — error_x (px) → lateral step (m) gain
  max_y_speed:         0.0      — max |lateral step| (m)
  min_y_speed:         0.0      — min |lateral step| (m) outside the deadband
  pan_default:         500      — head pan parked here in initialise()
  tilt_default:        500      — head tilt parked here in initialise()
  lost_count_threshold: 0       — missed frames tolerated before FAILURE

Gait pass-through (period_time_ms / dsp_ratio / y_swap_amplitude / arm_swap /
step_num / gait_param): inherited from XyzL2GaitActionNode — see that class's
docstring. The former per-knob arguments (step_height=, hip_pitch_offset=,
body_height=, …) are gone; put them in the dict.
"""
import math
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2GaitActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L2_Gait_VisionToObject(XyzL2GaitActionNode):
    """Walk a target onto a pixel target point using error_x / error_y, then stop.

    Returns RUNNING while approaching, SUCCESS on arrival, FAILURE when the
    target is lost. Requires the head parked at the pan centre — see the module
    docstring.
    """

    LEVEL        = 'L2'
    BB_READS     = [BB.TRACKED_OBJECTS]
    BB_WRITES    = []
    FACADE_CALLS = ['move_head', 'go_step', 'stop_gait']
    CONFIG_DEFAULTS = {
        'target_id':           'object',
        'x_error_threshold':   50,       # px; abs(error_x) <= this AND size >= threshold → SUCCESS
        'max_size_threshold':  250000.0, # px²; size > this → back up (−x_speed)
        # Explicit-target alignment (opt-in; None = legacy adapter error_x / size arrival):
        'align_x':             None,     # px; when set, steer on obj['x'] − align_x (e.g. 320 for a 640-wide frame)
        'align_y':             None,     # px; when set, drive fore/aft on obj['y'] − align_y (e.g. 240 for a 480-high frame)
        'y_error_threshold':   50,       # px; abs(obj['y'] − align_y) <= this to count as vertically aligned (align_y mode only)
        'x_speed':             0.010,
        'y_speed':          0.0,
        'kp':               0.2,
        'max_yaw_deg':      10,
        'deadband':         5,
        'min_yaw_deg':      0,
        # Horizontal steering axis (heading-preserving alternative to yaw):
        'steer_axis':       'yaw',    # 'yaw' (default) = turn on error_x; 'lateral' = strafe on error_x, yaw forced 0
        'lateral_kp':       0.0,      # error_x (px) → lateral step (m) gain (steer_axis='lateral' only)
        'max_y_speed':      0.0,     # max |lateral step| (m) clamp (steer_axis='lateral' only)
        'min_y_speed':      0.0,      # min |lateral step| (m) outside deadband; 0 = purely proportional
        'pan_default':      500,
        'tilt_default':     500,
        # Gait parameter overrides (None = use project _GO_STEP_VEL / _GO_CFG defaults)
        # Named WalkingParam knobs (mirror GaitManager.get_gait_param() keys; merged
        # into gait_param). None = controller default. Values in metres unless noted.
        'lost_count_threshold':   0,
    }

    def __init__(self, name: str = 'L2_Gait_VisionToObject',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 target_id: str = 'object',
                 x_error_threshold: int = 50,
                 max_size_threshold: float = 250000.0,
                 align_x: float = None,
                 align_y: float = None,
                 y_error_threshold: int = 50,
                 x_speed: float = 0.020,
                 y_speed: float = 0.0,
                 kp: float = 0.2,
                 max_yaw_deg: int = 10,
                 deadband: int = 5,
                 min_yaw_deg: int = 0,
                 steer_axis: str = 'yaw',
                 lateral_kp: float = 0.0,
                 max_y_speed: float = 0.02,
                 min_y_speed: float = 0.0,
                 pan_default: int = 500,
                 tilt_default: int = 500,
                 lost_count_threshold: int = 0,
                 period_time_ms: int = None,
                 dsp_ratio: float = None,
                 y_swap_amplitude: float = None,
                 arm_swap: int = None,
                 step_num: int = None,
                 gait_param: dict = None):
        """
        CONFIG_DEFAULTS:
            target_id:          Key to look up in tracked_objects dict.
            x_error_threshold:  Max abs(error_x) in pixels to count as "centred".
                                SUCCESS requires BOTH size gate AND centring gate.
                                Set large (e.g. 9999) to disable centring check.
            max_size_threshold: When obj['size'] > this value, back up (−x_speed).
                                None = never back up.
            align_x:            Explicit pixel target column. None (default) = legacy mode
                                (steer on adapter error_x, arrive on size). When set,
                                error_x = obj['x'] − align_x (e.g. 320 = centre of 640-wide).
            align_y:            Explicit pixel target row. None (default) = no vertical
                                gate/drive. When set, fore/aft is driven by obj['y'] − align_y
                                and arrival also requires abs(obj['y'] − align_y) <= y_error_threshold
                                (e.g. 240 = centre of 480-high).
            y_error_threshold:  Max abs(obj['y'] − align_y) in px for vertical alignment.
                                Used only when align_y is not None.
            x_speed:            Forward step magnitude (m).
            y_speed:          Lateral step magnitude (m); sign tracks yaw sign.
            kp:               Error_x (px) → gait yaw (°) gain.
            max_yaw_deg:      Maximum absolute gait yaw (°).
            deadband:         Pixel deadband on error_x (inside which yaw/lateral = 0).
            min_yaw_deg:      Minimum |yaw| when outside deadband (°).
                              0 = purely proportional (default).
            steer_axis:       Horizontal steering axis. 'yaw' (default) turns the body on
                              error_x (changes heading). 'lateral' strafes sideways on
                              error_x with yaw forced to 0, so the heading is preserved.
            lateral_kp:       error_x (px) → lateral step (m) gain. Used only when
                              steer_axis='lateral'. Negate to flip strafe direction.
            max_y_speed:      Maximum |lateral step| (m) clamp (steer_axis='lateral').
            min_y_speed:      Minimum |lateral step| (m) outside deadband. 0 = purely
                              proportional (default). Used only when steer_axis='lateral'.
            pan_default:      Head pan servo position set in initialise().
            tilt_default:     Head tilt servo position set in initialise().
            period_time_ms:   Gait cycle (ms). None = project default.
            dsp_ratio:        Double-support fraction (0–1). None = project default.
            y_swap_amplitude: Lateral body swing (m). None = project default.
            arm_swap:         Arm swing amplitude (°). None = project default.
            step_num:         Steps per tick (0 = continuous). None = project default.
            gait_param:           Partial WalkingParam dict (e.g. {'step_height': 0.03}).
                                  None = no override.
            init_yaw_offset:      Constant per-step heading bias (deg). None = controller
                                  default (0). Merged into gait_param as 'init_yaw_offset';
                                  applied by the walking module as a persistent yaw offset on
                                  every step (distinct from the per-tick steering yaw). Validate
                                  sign on the robot.
            hip_pitch_offset:     Hip forward/back tilt (deg). None = controller default (15).
                                  Merged into gait_param as 'hip_pitch_offset'; applied by the
                                  walking module as a persistent torso lean on every step.
                                  Validate sign/posture on the robot.
            init_x_offset:        Initial x foot offset (m). None = controller default.
            init_y_offset:        Initial y foot offset (m). None = controller default.
            body_height:          Initial z / body height (m), merged as 'body_height'
                                  (→ WalkingParam init_z_offset). None = controller default.
            init_roll_offset:     Initial roll offset (rad). None = controller default.
            init_pitch_offset:    Initial pitch offset (rad). None = controller default.
            step_fb_ratio:        Fore/aft step ratio. None = controller default.
            step_height:          Foot lift height (m), merged as 'step_height'
                                  (→ WalkingParam z_move_amplitude). None = controller default.
            z_swap_amplitude:     Vertical body swing (m). None = controller default.
            pelvis_offset:        Pelvis left/right swing (rad). None = controller default.
                                  init_x_offset .. pelvis_offset mirror GaitManager
                                  get_gait_param() keys; each is merged into gait_param and
                                  _RuntimeIO overlays it onto the controller's current params
                                  before GaitManager.set_step(). Validate values on the robot.
            lost_count_threshold: Tolerate this many consecutive missed detections
                                  before returning FAILURE. Default 0 (fail on first miss).
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade,
                         period_time_ms=period_time_ms, dsp_ratio=dsp_ratio,
                         y_swap_amplitude=y_swap_amplitude, arm_swap=arm_swap,
                         step_num=step_num, gait_param=gait_param)
        self._target_id          = target_id
        self._x_error_threshold  = x_error_threshold
        self._max_size_threshold = max_size_threshold
        self._align_x            = align_x
        self._align_y            = align_y
        self._y_error_threshold  = y_error_threshold
        self._x_speed            = x_speed
        self._y_speed          = y_speed
        self._kp               = kp
        self._max_yaw_deg      = max_yaw_deg
        self._deadband         = deadband
        self._min_yaw_deg      = min_yaw_deg
        self._steer_axis       = steer_axis
        self._lateral_kp       = lateral_kp
        self._max_y_speed      = max_y_speed
        self._min_y_speed      = min_y_speed
        self._pan_default      = pan_default
        self._tilt_default     = tilt_default
        self._lost_count_threshold   = lost_count_threshold
        self._bb                     = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)

    def _compute_yaw(self, error_x: float) -> int:
        """Return signed gait yaw from pixel error_x.

        Standard proportional yaw control:
          turn = -int((x - 320) / 320 * k)
        Here: yaw = -kp * error_x
          error_x > 0  (object right of centre) → negative yaw → turn right
          error_x < 0  (object left of centre)  → positive yaw → turn left
        Applies deadband, proportional gain, clamp.
        """
        if abs(error_x) <= self._deadband:
            return 0
        raw = - error_x * self._kp
        clamped = int(max(-self._max_yaw_deg, min(self._max_yaw_deg, raw)))
        if self._min_yaw_deg > 0 and abs(clamped) < self._min_yaw_deg:
            clamped = int(math.copysign(self._min_yaw_deg, raw))
        return clamped

    def _compute_y(self, yaw: int) -> float:
        """Return lateral step speed with same sign as yaw.

        yaw > 0 (turning left)  → +y_speed (step left)
        yaw < 0 (turning right) → -y_speed (step right)
        yaw == 0 (deadband)     →  0 (no lateral motion)
        """
        if yaw == 0 or self._y_speed == 0.0:
            return 0.0
        return math.copysign(self._y_speed, yaw)

    def _compute_lateral(self, error_x: float) -> float:
        """Return signed lateral step (m) from pixel error_x (steer_axis='lateral').

        Heading-preserving alternative to _compute_yaw: strafe sideways instead of
        turning, so the body heading stays fixed while the object is centred in x.
          error_x > 0 (object right of target) → −y (strafe right; +y = step left)
          error_x < 0 (object left  of target) → +y (strafe left)
        Applies the pixel deadband, proportional gain, clamp, and optional floor.
        VALIDATE sign on the robot: negate lateral_kp if it strafes the wrong way.
        """
        if abs(error_x) <= self._deadband:
            return 0.0
        raw = -self._lateral_kp * error_x
        clamped = max(-self._max_y_speed, min(self._max_y_speed, raw))
        if self._min_y_speed > 0 and abs(clamped) < self._min_y_speed:
            clamped = math.copysign(self._min_y_speed, raw)
        return clamped

    def _target_errors(self, obj) -> tuple:
        """Return (error_x, error_y) for this object; side-effect-free.

        error_x = obj['x'] − align_x when align_x is set (explicit-target mode),
                  else the adapter-calibrated obj['error_x'] (legacy mode).
        error_y = obj['y'] − align_y when align_y is set, else None (no vertical axis).
        """
        if self._align_x is not None:
            error_x = float(obj['x']) - self._align_x
        else:
            error_x = obj['error_x']
        error_y = (float(obj['y']) - self._align_y) if self._align_y is not None else None
        return error_x, error_y

    def initialise(self):
        self.emit_action_intent(
            action='vision_to_object',
            inputs={'target_id':          self._target_id,
                    'x_error_threshold':  self._x_error_threshold,
                    'max_size_threshold': self._max_size_threshold,
                    'align_x':            self._align_x,
                    'align_y':            self._align_y,
                    'y_error_threshold':  self._y_error_threshold,
                    'steer_axis':         self._steer_axis,
                    'pan_default':        self._pan_default,
                    'tilt_default':       self._tilt_default,
                    'dsp_ratio':          self._dsp_ratio},
        )
        self.call_facade('move_head',
                         pan_pos=self._pan_default,
                         tilt_pos=self._tilt_default)

    def update(self) -> Status:
        tracked = self._bb.tracked_objects
        obj     = tracked.get(self._target_id)

        # FAILURE: object not detected this camera frame
        if obj is None or obj['lost_count'] > self._lost_count_threshold:
            self.emit_decision(
                inputs={'target_id': self._target_id,
                        'in_dict':   self._target_id in tracked,
                        'lost_count': obj['lost_count'] if obj is not None else None},
                status=Status.FAILURE,
                reason='object not detected',
            )
            return Status.FAILURE

        error_x, error_y = self._target_errors(obj)
        size    = obj['size']
        # Horizontal correction: 'lateral' strafes on error_x (heading held fixed, yaw=0);
        # 'yaw' (default) turns on error_x. Fore/aft (error_y → x_step) is unaffected.
        if self._steer_axis == 'lateral':
            yaw = 0
            y   = self._compute_lateral(error_x)
        else:
            yaw = self._compute_yaw(error_x)
            y   = self._compute_y(yaw)

        # BACKWARD: object too close (size exceeded max threshold).
        # Guarded on size is not None: shapes with no meaningful area (a line)
        # report size=None, and this safety override simply does not apply to them.
        if (self._max_size_threshold is not None and size is not None
                and size > self._max_size_threshold):
            self.call_facade('go_step',
                             x=-self._x_speed, y=y, yaw=yaw,
                             semantic_source='vision_to_object_backup',
                             **self.gait_kwargs())
            self.emit_decision(
                inputs={'target_id':          self._target_id,
                        'error_x':            error_x,
                        'size':               round(size, 1),
                        'max_size_threshold': self._max_size_threshold,
                        'yaw':                yaw,
                        'x':                  -self._x_speed},
                status=Status.RUNNING,
                reason='backing up: size > max_size_threshold',
            )
            return Status.RUNNING

        # ── Arrival + forward-step selection (pixel alignment only) ───────
        # Both gates are pixel errors. There is no size-based arrival: with the
        # head locked at the x-centre, error_x is the correct lateral signal, and
        # error_y (when align_y is set) is the distance proxy. Size varies with
        # the object, so it never made a transferable arrival criterion.
        x_centred = abs(error_x) <= self._x_error_threshold
        y_centred = (error_y is None
                     or abs(error_y) <= self._y_error_threshold)

        if x_centred and y_centred:
            self.call_facade('stop_gait')
            self.emit_decision(
                inputs={'target_id':         self._target_id,
                        'error_x':           round(error_x, 1),
                        'error_y':           None if error_y is None else round(error_y, 1),
                        'x_error_threshold': self._x_error_threshold,
                        'y_error_threshold': self._y_error_threshold},
                status=Status.SUCCESS,
                reason=('aligned in x and y' if error_y is not None
                        else 'aligned in x (no align_y configured)'),
            )
            return Status.SUCCESS

        if error_y is None:
            # No vertical target configured → no distance information, so never
            # advance; this node then performs pure lateral alignment.
            x_step          = 0.0
            approach_reason = 'steer-only: no align_y, aligning x'
        elif y_centred:
            x_step          = 0.0
            approach_reason = 'steer-only: error_y aligned, waiting for x-centre'
        elif error_y < 0:
            x_step          = self._x_speed   # object high in frame → far → forward
            approach_reason = 'approaching: object above target (far)'
        else:
            x_step          = -self._x_speed  # object low in frame → near → back off
            approach_reason = 'backing off: object below target (near)'

        self.call_facade('go_step',
                         x=x_step, y=y, yaw=yaw,
                         semantic_source='vision_to_object_approach',
                         **self.gait_kwargs())
        self.emit_decision(
            inputs={'target_id':         self._target_id,
                    'error_x':           error_x,
                    'error_y':           error_y,
                    'size':              None if size is None else round(size, 1),
                    'x_error_threshold': self._x_error_threshold,
                    'y_error_threshold': self._y_error_threshold,
                    'yaw':               yaw,
                    'y':                 y,
                    'x':                 x_step},
            status=Status.RUNNING,
            reason=approach_reason,
        )
        return Status.RUNNING
