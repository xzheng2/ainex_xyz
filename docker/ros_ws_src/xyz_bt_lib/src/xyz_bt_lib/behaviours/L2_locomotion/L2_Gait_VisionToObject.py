#!/usr/bin/env python3
"""L2 Action: walk toward a detected object using raw pixel data from BB.

Node kind: finite_action

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects)  — written each tick by
                                                     ObjectDetectionAdapter

BB writes:
  (none)

Facade:
  move_head(pan_pos, tilt_pos)  — centre head once on initialise()
  go_step(x, y, yaw)           — one approach step per tick while RUNNING
  stop_gait()                  — called on SUCCESS (arrived)

Strategy:
  Reads tracked_objects[target_id] from the blackboard (written by
  ObjectDetectionAdapter) and drives the robot toward the target using
  raw pixel error and object size.  Head is physically centred once at
  the start of each activation via move_head(), then left fixed.

  Two alignment modes (selected by align_x):
    • Legacy (align_x is None, default): steer on the adapter-calibrated error_x
      and arrive on object size — unchanged historical behaviour.
    • Explicit-target (align_x is not None): steer on obj['x'] − align_x so the
      object is driven to a tuneable pixel column (e.g. align_x=320 for a 640-wide
      frame = centre). When align_y is also set, fore/aft motion is driven by
      obj['y'] − align_y (head-tilted-down geometry: object high in frame → far →
      step forward) and arrival requires BOTH pixel axes within their thresholds
      instead of the size gate.

  Each tick:
    error_x = obj['x'] − align_x           (explicit-target mode), else
              tracked_objects[target_id]['error_x']  (legacy; obj.x − image_center_x)
    error_y = obj['y'] − align_y           (only when align_y is not None)
    size    = tracked_objects[target_id]['size']
              (= π*radius² for circle; width*height for rect)

  Horizontal steering (selected by steer_axis):
    • 'yaw' (default, standard proportional steering):
        yaw = −kp * error_x   (positive error_x → object right → turn right → negative yaw)
        deadband applied before gain; result clamped to ±max_yaw_deg. Turning changes heading.
    • 'lateral' (heading-preserving): yaw forced 0; strafe sideways instead —
        y = −lateral_kp * error_x, deadband applied, clamped to ±max_y_speed, optional
        min_y_speed floor. Body heading stays fixed while the object is centred in x.
    In both modes fore/aft motion (error_y → x_step) is unchanged.

  Termination (legacy, align_y is None):
    size > max_size_threshold              → go_step(−x_speed, y, yaw), return RUNNING  (back up)
    size >= size_threshold
      AND abs(error_x) <= x_error_threshold → stop_gait(), return SUCCESS  (arrived + centred)
    size >= size_threshold
      AND abs(error_x) >  x_error_threshold → go_step(0, y, yaw),         return RUNNING  (steer-only)
    size <  size_threshold                 → go_step(x_speed, y, yaw),   return RUNNING  (approach)

  Termination (explicit-target, align_y is not None):
    size > max_size_threshold              → go_step(−x_speed, y, yaw), return RUNNING  (back up)
    abs(error_x) <= x_error_threshold
      AND abs(error_y) <= y_error_threshold → stop_gait(), return SUCCESS  (aligned in x AND y)
    abs(error_y) <= y_error_threshold      → go_step(0, y, yaw),        return RUNNING  (steer-only)
    error_y < 0 (object high → far)        → go_step(+x_speed, y, yaw), return RUNNING  (approach)
    error_y > 0 (object low  → near)       → go_step(−x_speed, y, yaw), return RUNNING  (back off)

  Object lost (no key or lost_count > lost_count_threshold) → return FAILURE
  immediately so a parent Selector can route to a re-find branch.

Returns:
  RUNNING   — object detected; one of: backing up / steer-only / approach step issued
  SUCCESS   — size >= size_threshold AND abs(error_x) <= x_error_threshold; gait stopped
  FAILURE   — object not detected (lost_count > lost_count_threshold or key absent)

CONFIG_DEFAULTS:
  target_id:           'object'   — key to look up in tracked_objects dict
  size_threshold:      1000.0     — lower bound for arrival: obj['size'] >= this value
                                   (π*r² for circle; w*h for rect; tune per project)
  x_error_threshold:   50         — max abs(error_x) in px to count as centred;
                                   SUCCESS requires BOTH size AND centring gates
  max_size_threshold:  250000.0   — when obj['size'] > this, back up (−x_speed);
                                   set to None to disable backward motion
  align_x:          None      — explicit pixel target column; None = legacy error_x path.
                                Recommended centre for a 640×480 frame: 320.
  align_y:          None      — explicit pixel target row; None = no vertical gate/drive.
                                Recommended centre for a 640×480 frame: 240.
  y_error_threshold:   50     — max abs(obj['y'] − align_y) in px to count as vertically
                                aligned (used only when align_y is not None)
  x_speed:          0.010     — forward step magnitude (m), range (-0.020 to 0.020)
  y_speed:          0.0       — lateral step magnitude (m); sign tracks yaw sign
  kp:               0.2       — error_x (px) → gait yaw (°) gain
  max_yaw_deg:      10        — maximum absolute gait yaw (°)
  deadband:         5         — pixel deadband on error_x (inside which yaw/lateral = 0)
  min_yaw_deg:      0         — minimum |yaw| when outside deadband (0 = purely proportional)
  steer_axis:       'yaw'     — horizontal steering: 'yaw' (turn on error_x, default) or
                               'lateral' (strafe on error_x with yaw=0; heading preserved)
  lateral_kp:       0.0       — error_x (px) → lateral step (m) gain (steer_axis='lateral')
  max_y_speed:      0.02      — max |lateral step| (m) clamp (steer_axis='lateral')
  min_y_speed:      0.0       — min |lateral step| (m) outside deadband (0 = purely proportional)
  pan_default:      500       — head pan servo position set in initialise()
  tilt_default:     500       — head tilt servo position set in initialise()
  period_time_ms:   None      — gait cycle (ms); None = project default
  dsp_ratio:        None      — double-support fraction (0–1); None = project default
  y_swap_amplitude: None      — lateral body swing (m); None = project default
  arm_swap:             None  — arm swing amplitude (°); None = project default
  step_num:             None  — steps per tick (0 = continuous); None = project default
  gait_param:           None  — partial WalkingParam dict; None = no override
  init_yaw_offset:      None  — constant per-step heading bias (deg); None = controller default (0)
  hip_pitch_offset:     None  — hip forward/back tilt (deg); None = controller default (15).
                                Merged into gait_param as 'hip_pitch_offset'.
  init_x_offset:        None  — initial x foot offset (m); None = controller default
  init_y_offset:        None  — initial y foot offset (m); None = controller default
  body_height:          None  — initial z / body height (m, → init_z_offset); None = default
  init_roll_offset:     None  — initial roll offset (rad); None = controller default
  init_pitch_offset:    None  — initial pitch offset (rad); None = controller default
  step_fb_ratio:        None  — fore/aft step ratio; None = controller default
  step_height:          None  — foot lift height (m, → z_move_amplitude); None = default
  z_swap_amplitude:     None  — vertical body swing (m); None = controller default
  pelvis_offset:        None  — pelvis left/right swing (rad); None = controller default
                                (init_x/y_offset .. pelvis_offset mirror GaitManager
                                 get_gait_param() keys; each merged into gait_param)
  lost_count_threshold: 0     — tolerate this many consecutive missed detections
                                before returning FAILURE (0 = fail on first miss)
"""
import math
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L2_Gait_VisionToObject(XyzL2ActionNode):
    """Walk toward a detected object using pixel error_x for steering and size for arrival.

    Returns RUNNING while approaching, SUCCESS on arrival, FAILURE when object is lost.
    """

    LEVEL        = 'L2'
    BB_READS     = [BB.TRACKED_OBJECTS]
    BB_WRITES    = []
    FACADE_CALLS = ['move_head', 'go_step', 'stop_gait']
    CONFIG_DEFAULTS = {
        'target_id':           'object',
        'size_threshold':      1000.0,
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
        'period_time_ms':   None,
        'dsp_ratio':        None,
        'y_swap_amplitude': None,
        'arm_swap':               None,
        'step_num':               None,
        'gait_param':             None,
        'init_yaw_offset':        None,
        'hip_pitch_offset':       None,
        # Named WalkingParam knobs (mirror GaitManager.get_gait_param() keys; merged
        # into gait_param). None = controller default. Values in metres unless noted.
        'init_x_offset':          None,  # initial x foot offset (m)
        'init_y_offset':          None,  # initial y foot offset (m)
        'body_height':            None,  # initial z / body height (m) → init_z_offset
        'init_roll_offset':       None,  # initial roll offset (rad)
        'init_pitch_offset':      None,  # initial pitch offset (rad)
        'step_fb_ratio':          None,  # fore/aft step ratio
        'step_height':            None,  # foot lift height (m) → z_move_amplitude
        'z_swap_amplitude':       None,  # vertical body swing (m)
        'pelvis_offset':          None,  # pelvis left/right swing (rad)
        'lost_count_threshold':   0,
    }

    def __init__(self, name: str = 'L2_Gait_VisionToObject',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 target_id: str = 'object',
                 size_threshold: float = 1000.0,
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
                 period_time_ms: int = None,
                 dsp_ratio: float = None,
                 y_swap_amplitude: float = None,
                 arm_swap: int = None,
                 step_num: int = None,
                 gait_param: dict = None,
                 init_yaw_offset: float = None,
                 hip_pitch_offset: float = None,
                 init_x_offset: float = None,
                 init_y_offset: float = None,
                 body_height: float = None,
                 init_roll_offset: float = None,
                 init_pitch_offset: float = None,
                 step_fb_ratio: float = None,
                 step_height: float = None,
                 z_swap_amplitude: float = None,
                 pelvis_offset: float = None,
                 lost_count_threshold: int = 0):
        """
        CONFIG_DEFAULTS:
            target_id:          Key to look up in tracked_objects dict.
            size_threshold:     Lower bound for arrival: obj['size'] >= this value.
                                π*r² for circle; w*h for rect. Tune per project.
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
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._target_id          = target_id
        self._size_threshold     = size_threshold
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
        self._period_time_ms   = period_time_ms
        self._dsp_ratio        = dsp_ratio
        self._y_swap_amplitude = y_swap_amplitude
        self._arm_swap         = arm_swap
        self._step_num         = step_num
        self._gait_param             = gait_param
        self._init_yaw_offset        = init_yaw_offset
        self._hip_pitch_offset       = hip_pitch_offset
        self._init_x_offset          = init_x_offset
        self._init_y_offset          = init_y_offset
        self._body_height            = body_height
        self._init_roll_offset       = init_roll_offset
        self._init_pitch_offset      = init_pitch_offset
        self._step_fb_ratio          = step_fb_ratio
        self._step_height            = step_height
        self._z_swap_amplitude       = z_swap_amplitude
        self._pelvis_offset          = pelvis_offset
        self._lost_count_threshold   = lost_count_threshold
        self._bb                     = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)

    def _effective_gait_param(self):
        """Merge the named WalkingParam knobs into the partial gait_param dict.

        Each named knob (init_yaw_offset, hip_pitch_offset, init_x/y offsets,
        body_height, roll/pitch offsets, step_fb_ratio, step_height,
        z_swap_amplitude, pelvis_offset) overrides its matching gait_param key
        when set. Keys mirror GaitManager.get_gait_param(); _RuntimeIO merges the
        result onto the controller's current params before GaitManager.set_step().
        Returns None when nothing overrides (controller defaults)."""
        gp = dict(self._gait_param) if self._gait_param else {}
        if self._init_yaw_offset is not None:
            gp['init_yaw_offset'] = self._init_yaw_offset   # per-step heading bias (deg)
        if self._hip_pitch_offset is not None:
            gp['hip_pitch_offset'] = self._hip_pitch_offset  # hip fwd/back tilt (deg)
        if self._init_x_offset is not None:
            gp['init_x_offset'] = self._init_x_offset        # initial x foot offset (m)
        if self._init_y_offset is not None:
            gp['init_y_offset'] = self._init_y_offset        # initial y foot offset (m)
        if self._body_height is not None:
            gp['body_height'] = self._body_height            # initial z / body height (m)
        if self._init_roll_offset is not None:
            gp['init_roll_offset'] = self._init_roll_offset  # initial roll offset (rad)
        if self._init_pitch_offset is not None:
            gp['init_pitch_offset'] = self._init_pitch_offset  # initial pitch offset (rad)
        if self._step_fb_ratio is not None:
            gp['step_fb_ratio'] = self._step_fb_ratio        # fore/aft step ratio
        if self._step_height is not None:
            gp['step_height'] = self._step_height            # foot lift height (m)
        if self._z_swap_amplitude is not None:
            gp['z_swap_amplitude'] = self._z_swap_amplitude  # vertical body swing (m)
        if self._pelvis_offset is not None:
            gp['pelvis_offset'] = self._pelvis_offset        # pelvis L/R swing (rad)
        return gp or None

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
                    'size_threshold':     self._size_threshold,
                    'x_error_threshold':  self._x_error_threshold,
                    'max_size_threshold': self._max_size_threshold,
                    'align_x':            self._align_x,
                    'align_y':            self._align_y,
                    'y_error_threshold':  self._y_error_threshold,
                    'steer_axis':         self._steer_axis,
                    'pan_default':        self._pan_default,
                    'tilt_default':       self._tilt_default,
                    'period_time_ms':     self._period_time_ms,
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

        # BACKWARD: object too close (size exceeded max threshold)
        if self._max_size_threshold is not None and size > self._max_size_threshold:
            self.call_facade('go_step',
                             x=-self._x_speed, y=y, yaw=yaw,
                             period_time_ms=self._period_time_ms,
                             dsp_ratio=self._dsp_ratio,
                             y_swap_amplitude=self._y_swap_amplitude,
                             arm_swap=self._arm_swap,
                             step_num=self._step_num,
                             gait_param=self._effective_gait_param(),
                             semantic_source='vision_to_object_backup')
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

        # ── Arrival + forward-step selection ──────────────────────────────
        if self._align_y is not None:
            # Explicit-target mode: gate/drive on pixel errors (error_x, error_y).
            x_centred = abs(error_x) <= self._x_error_threshold
            y_centred = abs(error_y) <= self._y_error_threshold
            # SUCCESS: aligned in both pixel axes
            if x_centred and y_centred:
                self.call_facade('stop_gait')
                self.emit_decision(
                    inputs={'target_id':         self._target_id,
                            'error_x':           round(error_x, 1),
                            'error_y':           round(error_y, 1),
                            'x_error_threshold': self._x_error_threshold,
                            'y_error_threshold': self._y_error_threshold},
                    status=Status.SUCCESS,
                    reason='error_x <= x_thr AND error_y <= y_thr (aligned)',
                )
                return Status.SUCCESS
            if y_centred:
                x_step          = 0.0
                approach_reason = 'steer-only: error_y aligned, waiting for x-centre'
            elif error_y < 0:
                x_step          = self._x_speed   # object high in frame → far → forward
                approach_reason = 'approaching: object above target (far)'
            else:
                x_step          = -self._x_speed  # object low in frame → near → back off
                approach_reason = 'backing off: object below target (near)'
        else:
            # Legacy mode: size-based arrival + forward step.
            if size >= self._size_threshold and abs(error_x) <= self._x_error_threshold:
                self.call_facade('stop_gait')
                self.emit_decision(
                    inputs={'target_id':         self._target_id,
                            'error_x':           error_x,
                            'size':              round(size, 1),
                            'size_threshold':    self._size_threshold,
                            'x_error_threshold': self._x_error_threshold},
                    status=Status.SUCCESS,
                    reason='size >= threshold AND error_x <= x_error_threshold',
                )
                return Status.SUCCESS

            # Determine forward step: stop advancing when already close (avoid overshoot)
            if size >= self._size_threshold:
                x_step        = 0.0
                approach_reason = 'steer-only: size >= threshold, waiting for centre'
            else:
                x_step        = self._x_speed
                approach_reason = 'approaching: size < threshold'

        self.call_facade('go_step',
                         x=x_step, y=y, yaw=yaw,
                         period_time_ms=self._period_time_ms,
                         dsp_ratio=self._dsp_ratio,
                         y_swap_amplitude=self._y_swap_amplitude,
                         arm_swap=self._arm_swap,
                         step_num=self._step_num,
                         gait_param=self._effective_gait_param(),
                         semantic_source='vision_to_object_approach')
        self.emit_decision(
            inputs={'target_id':         self._target_id,
                    'error_x':           error_x,
                    'size':              round(size, 1),
                    'size_threshold':    self._size_threshold,
                    'x_error_threshold': self._x_error_threshold,
                    'yaw':               yaw,
                    'y':                 y,
                    'x':                 x_step},
            status=Status.RUNNING,
            reason=approach_reason,
        )
        return Status.RUNNING
