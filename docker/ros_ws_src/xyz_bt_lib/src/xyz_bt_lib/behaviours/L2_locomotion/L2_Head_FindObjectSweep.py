#!/usr/bin/env python3
"""L2 Action: sweep head to find a lost object, then PID-track it continuously.

Phase 1 — SWEEP
    Robot stands still.  Head pans between sweep_left_pos and sweep_right_pos,
    pausing sweep_pause_ticks ticks after each step.
    Transitions to TRACK as soon as the target appears in tracked_objects with
    lost_count == 0.

Tilt raster scan (optional):
    When tilt_sweep_min/max are set, the tilt servo steps one row each time the
    pan reverses at an endpoint.  Rows advance from tilt_sweep_min to
    tilt_sweep_max by tilt_step units, then wrap back to tilt_sweep_min.
    This creates a 2-D grid scan.  When tilt_sweep_min is None the tilt servo
    is not commanded (same as the former head_tilt_pos=None behaviour).

Phase 2 — TRACK
    Each tick two independent PID controllers (pan + optional tilt) drive the
    head servos to keep the detected object centred in the camera frame
    (error_x → pan servo, error_y → tilt servo).  A deadband of
    track_deadband_px pixels around the centre is treated as zero error.
    The node stays in TRACK as long as lost_count <= lost_count_threshold.
    Brief occlusions (up to lost_count_threshold frames) are tolerated without
    reverting to SWEEP.  If the object is lost beyond the threshold the node
    reverts to SWEEP.
    TRACK never returns SUCCESS; it always returns RUNNING.

Re-activation (initialise):
    On each call to initialise(), _has_target() is checked first.  If the object
    is currently visible (lost_count == 0 in tracked_objects) the node enters
    TRACK immediately — skipping SWEEP.  This ensures the head does not restart
    a full sweep when the parent re-activates the node while the object is still
    in view.

State machine:
    initialise(): _has_target() → TRACK  |  else → SWEEP
    SWEEP → (target found, lost_count==0)              → TRACK
    TRACK → (object lost, lost_count>lost_count_threshold) → stop_gait(), SWEEP, _fresh_start=True
    TRACK                                              → always RUNNING

Node kind: finite_action

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects)

BB writes:
  none

Facade:    stop_gait(), move_head(pan_pos, tilt_pos)
Strategy:  _update_track() — inline PID pan + tilt via _PIDTrack

Returns:
    RUNNING → sweep or track in progress (node never returns SUCCESS)

CONFIG_DEFAULTS:
    target_id:            'object'  key to look up in tracked_objects dict
    sweep_left_pos:        700   leftmost  servo position (500=centre, 875=hard left)
    sweep_right_pos:       300   rightmost servo position (500=centre, 125=hard right)
    sweep_step:            10    servo units per 30 Hz tick (must NOT be a factor of 200)
    sweep_pause_ticks:     0     ticks to hold after each sweep step (0 = step every tick)
    head_pan_center:       500   servo centre position
    tilt_sweep_min:        None  lowest tilt pos; None = do not command tilt servo
    tilt_sweep_max:        None  highest tilt pos (wraps back to min after reaching)
    tilt_step:             50    servo units stepped at each pan reversal
    pid_pan_p:             0.1   PID P gain for pan servo
    pid_pan_i:             0.0   PID I gain for pan servo
    pid_pan_d:             0.0   PID D gain for pan servo
    pid_tilt_p:            0.1   PID P gain for tilt servo
    pid_tilt_i:            0.0   PID I gain for tilt servo
    pid_tilt_d:            0.0   PID D gain for tilt servo
    track_deadband_px:     50    pixel deadband — errors within this are treated as zero
    lost_count_threshold:  0     frames of absence before reverting TRACK→SWEEP (0 = strict)
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


# ---------------------------------------------------------------------------
# Inline PID controller (adapted from ainex_sdk/pid.py)
# Timing removed — one call = one time unit (suited for fixed-rate BT ticks).
# ---------------------------------------------------------------------------
class _PID:
    """Minimal P-I-D controller with windup guard.  No time.time() dependency."""

    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.windup_guard = 20.0
        self.clear()

    def clear(self):
        self.SetPoint  = 0.0
        self.PTerm     = 0.0
        self.ITerm     = 0.0
        self.DTerm     = 0.0
        self.last_error = 0.0
        self.output    = 0.0

    def update(self, feedback_value):
        error       = self.SetPoint - feedback_value
        delta_error = error - self.last_error

        self.PTerm = self.Kp * error
        self.ITerm = max(-self.windup_guard,
                         min(self.windup_guard, self.ITerm + error))
        self.DTerm = delta_error   # 1 tick per call implied
        self.last_error = error

        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)


# ---------------------------------------------------------------------------
# Inline PID tracker (adapted from ainex_example/pid_track.py)
# Maintains a servo position value, updates it each tick via the PID output.
# ---------------------------------------------------------------------------
class _PIDTrack:
    """Integrates PID output into a servo position, clamped to value_range."""

    def __init__(self, pid: _PID, value_range: list, value: float = 0):
        self.pid         = pid
        self.value_range = value_range   # [lo, hi]
        self.value       = value

    def update_position(self, value: float):
        """Snap the tracked value (e.g. after a sweep stops at a known position)."""
        self.value = value

    def clear(self):
        self.pid.clear()

    def track(self, current: float, target: float) -> float:
        """Compute one PID step.  Returns the updated servo position."""
        self.pid.SetPoint = target
        self.pid.update(current)
        self.value = max(self.value_range[0],
                         min(self.value_range[1], self.value + self.pid.output))
        return self.value


# ---------------------------------------------------------------------------
# BT node
# ---------------------------------------------------------------------------
class L2_Head_FindObjectSweep(XyzL2ActionNode):
    """Sweep-then-track head search for a tracked object.
    Always returns RUNNING; the node never returns SUCCESS.
    """

    LEVEL        = 'L2'
    BB_READS     = [BB.TRACKED_OBJECTS]
    BB_WRITES    = []
    FACADE_CALLS = ['stop_gait', 'move_head']
    CONFIG_DEFAULTS = {
        'target_id':           'object',
        'sweep_left_pos':      700,
        'sweep_right_pos':     300,
        'sweep_step':          10,
        'sweep_pause_ticks':   0,
        'head_pan_center':     500,
        'tilt_sweep_min':      None,   # lowest tilt position; None = do not command tilt
        'tilt_sweep_max':      None,   # highest tilt position
        'tilt_step':           50,     # servo units stepped at each pan reversal
        'pid_pan_p':           0.1,    # PID P gain for pan servo
        'pid_pan_i':           0.0,    # PID I gain for pan
        'pid_pan_d':           0.0,    # PID D gain for pan
        'pid_tilt_p':          0.1,    # PID P gain for tilt servo
        'pid_tilt_i':          0.0,
        'pid_tilt_d':          0.0,
        'track_deadband_px':   50,     # pixel deadband (errors within this = zero)
        'lost_count_threshold': 0,     # frames of absence before reverting TRACK→SWEEP (0 = strict)
    }

    # ── Direction convention ───────────────────────────────────────────────
    # +1 → increasing servo value = head pans LEFT (confirmed hardware)
    PAN_INVERT = 1

    # ── States ─────────────────────────────────────────────────────────────
    _ST_SWEEP = 'sweep'
    _ST_TRACK = 'track'

    def __init__(self, name: str = 'L2_Head_FindObjectSweep',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 target_id: str = 'object',
                 sweep_left_pos: int = 700,
                 sweep_right_pos: int = 300,
                 sweep_step: int = 10,
                 sweep_pause_ticks: int = 0,
                 head_pan_center: int = 500,
                 tilt_sweep_min: int = None,
                 tilt_sweep_max: int = None,
                 tilt_step: int = 50,
                 pid_pan_p: float = 0.1,
                 pid_pan_i: float = 0.0,
                 pid_pan_d: float = 0.0,
                 pid_tilt_p: float = 0.1,
                 pid_tilt_i: float = 0.0,
                 pid_tilt_d: float = 0.0,
                 track_deadband_px: int = 50,
                 lost_count_threshold: int = 0):
        """
        Args:
            target_id:           Key to look up in tracked_objects dict.
            sweep_left_pos:      Leftmost servo position (hw max 875).
            sweep_right_pos:     Rightmost servo position (hw min 125).
            sweep_step:          Servo units per tick while sweeping.
                                 Must NOT be a factor of 200 (half sweep range),
                                 otherwise the head will never reach exact centre.
            sweep_pause_ticks:   Ticks to hold after each sweep step (0 = step every tick).
            head_pan_center:     Servo centre position.
            tilt_sweep_min:      Lowest tilt servo position (start of each vertical row).
                                 None = do not command tilt servo at all.
            tilt_sweep_max:      Highest tilt servo position; wraps back to min after reaching.
            tilt_step:           Servo units incremented at each pan-reversal event.
            pid_pan_p:           P gain for pan PID controller.
            pid_pan_i:           I gain for pan PID controller.
            pid_pan_d:           D gain for pan PID controller.
            pid_tilt_p:          P gain for tilt PID controller.
            pid_tilt_i:          I gain for tilt PID controller.
            pid_tilt_d:          D gain for tilt PID controller.
            track_deadband_px:   Pixel deadband; errors smaller than this are treated as zero.
            lost_count_threshold:  Camera frames of absence tolerated before TRACK reverts to
                                 SWEEP.  0 = strict (any miss reverts). Default 0.

        CONFIG_DEFAULTS:
            See class-level CONFIG_DEFAULTS dict.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._target_id          = target_id
        self._sweep_left_pos     = sweep_left_pos
        self._sweep_right_pos    = sweep_right_pos
        self._sweep_step         = sweep_step
        self._sweep_pause_ticks  = sweep_pause_ticks
        self._head_pan_center    = head_pan_center
        self._tilt_sweep_min     = tilt_sweep_min
        self._tilt_sweep_max     = tilt_sweep_max
        self._tilt_step          = tilt_step
        self._track_deadband_px  = track_deadband_px
        self._lost_count_threshold = lost_count_threshold

        self._head_tilt  = tilt_sweep_min   # current tilt; None = don't command
        self._bb         = None
        self._head_pan   = self._head_pan_center
        self._sweep_dir  = +self.PAN_INVERT   # initial: sweep left
        self._pause_ticks = 0
        self._state      = self._ST_SWEEP
        self._fresh_start = True  # True on first run and after object is lost in TRACK

        # PIDTrack for pan: value_range = [right, left] = [300, 700]
        self._pid_pan_track = _PIDTrack(
            _PID(P=pid_pan_p, I=pid_pan_i, D=pid_pan_d),
            [sweep_right_pos, sweep_left_pos],
            head_pan_center,
        )

        # PIDTrack for tilt (always constructed; only used when tilt is configured)
        tilt_init = tilt_sweep_min if tilt_sweep_min is not None else 500
        tilt_lo   = tilt_sweep_min if tilt_sweep_min is not None else 315
        tilt_hi   = tilt_sweep_max if tilt_sweep_max is not None else 625
        self._pid_tilt_track = _PIDTrack(
            _PID(P=pid_tilt_p, I=pid_tilt_i, D=pid_tilt_d),
            [tilt_lo, tilt_hi],
            tilt_init,
        )

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=f'{self.name}_latched', namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)

    def initialise(self):
        """Called on first activation and after the node leaves RUNNING state.

        Checks _has_target() first: if the object is currently visible
        (lost_count == 0) the node enters TRACK immediately — no sweep reset.
        This ensures the head does not restart a full sweep when the parent
        re-activates the node while the object is still in view.

        When the object is absent the node enters SWEEP.  stop_gait() is called
        so the robot stands still while scanning.

        _sweep_dir is only reset on a genuine fresh start (_fresh_start=True):
        the very first run, or after _update_track() detected the object was
        lost.  On mid-sweep re-activations (node briefly goes INVALID while
        still in SWEEP) _sweep_dir is preserved so the sweep continues in the
        same direction.
        """
        if self._has_target():
            # Object currently visible — go directly to TRACK, no sweep reset.
            self._state       = self._ST_TRACK
            self._pause_ticks = 0
            self.emit_action_intent(
                action='head_track_resume',
                inputs={'head_pan': self._head_pan, 'head_tilt': self._head_tilt,
                        'target_id': self._target_id},
            )
            return

        # Object not visible — SWEEP: stand still while scanning.
        self.call_facade('stop_gait')
        self._state       = self._ST_SWEEP
        self._pause_ticks = 0

        if self._fresh_start:
            # Genuine fresh start: pick sweep direction from last known error_x.
            # error_x < 0 → target was to the LEFT  → look left first.
            # error_x >= 0 or None → look right (or unknown).
            self._fresh_start = False
            tracked = self._bb.tracked_objects
            obj     = tracked.get(self._target_id)
            last_err = obj.get('error_x') if obj is not None else None
            if last_err is not None and last_err < 0:
                self._sweep_dir = +self.PAN_INVERT   # target was left → look left
            else:
                self._sweep_dir = -self.PAN_INVERT   # target was right or unknown → look right
            self._head_tilt = self._tilt_sweep_min   # reset tilt to start of scan grid

            # Reset PID state so integrator doesn't carry over from a previous run
            self._pid_pan_track.pid.clear()
            self._pid_pan_track.update_position(self._head_pan_center)
            self._pid_tilt_track.pid.clear()
            self._pid_tilt_track.update_position(self._tilt_sweep_min or 500)
        # else: mid-sweep reinit — preserve _sweep_dir and _head_tilt

        self._command_head(self._head_pan)
        self.emit_action_intent(
            action='head_sweep_start',
            inputs={'head_pan': self._head_pan, 'sweep_dir': self._sweep_dir,
                    'head_tilt': self._head_tilt, 'target_id': self._target_id},
        )

    def update(self) -> Status:
        if self._state == self._ST_SWEEP:
            return self._update_sweep()
        return self._update_track()

    # ── Phase 1 ────────────────────────────────────────────────────────────

    def _update_sweep(self) -> Status:
        if self._has_target():
            self._state = self._ST_TRACK
            # Sync PID starting position to wherever the sweep stopped
            self._pid_pan_track.update_position(self._head_pan)
            if self._head_tilt is not None:
                self._pid_tilt_track.update_position(self._head_tilt)
            self.emit_decision(
                inputs={'head_pan': self._head_pan, 'transition': 'TRACK',
                        'target_id': self._target_id},
                status=Status.RUNNING,
                reason='target detected, transitioning to TRACK',
            )
            return self._update_track()

        # Hold at endpoint before reversing
        if self._pause_ticks > 0:
            self._pause_ticks -= 1
            return Status.RUNNING

        # Step head
        self._head_pan = int(self._head_pan + self._sweep_dir * self._sweep_step)

        if self._head_pan >= self._sweep_left_pos:
            self._head_pan    = self._sweep_left_pos
            self._sweep_dir   = -self.PAN_INVERT
            # tilt does NOT advance here — left endpoint starts a new return pass
        elif self._head_pan <= self._sweep_right_pos:
            self._head_pan    = self._sweep_right_pos
            self._sweep_dir   = +self.PAN_INVERT
            self._advance_tilt()

        self._command_head(self._head_pan)
        self._pause_ticks = self._sweep_pause_ticks
        self.emit_decision(
            inputs={'head_pan': self._head_pan, 'sweep_dir': self._sweep_dir,
                    'head_tilt': self._head_tilt, 'target_id': self._target_id},
            status=Status.RUNNING,
            reason='sweeping',
        )
        return Status.RUNNING

    # ── Phase 2 ────────────────────────────────────────────────────────────

    def _update_track(self) -> Status:
        tracked = self._bb.tracked_objects
        obj     = tracked.get(self._target_id)

        if obj is None or obj['lost_count'] > self._lost_count_threshold:
            self._state       = self._ST_SWEEP
            self._fresh_start = True
            self.call_facade('stop_gait')          # stop robot while sweeping
            self.emit_decision(
                inputs={'target_id': self._target_id},
                status=Status.RUNNING,
                reason='object lost during TRACK, reverting to SWEEP',
            )
            return Status.RUNNING

        error_x = float(obj['error_x'])
        error_y = float(obj['error_y'])

        # Deadband: snap to zero within threshold
        if abs(error_x) < self._track_deadband_px:
            error_x = 0.0
        if abs(error_y) < self._track_deadband_px:
            error_y = 0.0

        # PID pan: drives error_x → 0 (object to screen centre)
        self._head_pan = int(self._pid_pan_track.track(error_x, 0.0))

        # PID tilt (only when tilt is configured)
        if self._head_tilt is not None:
            self._head_tilt = int(self._pid_tilt_track.track(error_y, 0.0))

        self._command_head(self._head_pan)
        self.emit_decision(
            inputs={'head_pan': self._head_pan, 'error_x': error_x, 'error_y': error_y,
                    'target_id': self._target_id},
            status=Status.RUNNING,
            reason='tracking object with PID',
        )
        return Status.RUNNING

    # ── Helpers ────────────────────────────────────────────────────────────

    def _has_target(self) -> bool:
        """Return True if target is currently detected (lost_count == 0)."""
        tracked = self._bb.tracked_objects
        obj     = tracked.get(self._target_id)
        return obj is not None and obj['lost_count'] == 0

    def _advance_tilt(self):
        """Step tilt one row at each pan reversal.  No-op when tilt_sweep_min is None."""
        if self._tilt_sweep_min is None:
            return
        self._head_tilt += self._tilt_step
        if self._head_tilt > self._tilt_sweep_max:
            self._head_tilt = self._tilt_sweep_min

    def _command_head(self, pan_pos: int):
        self.call_facade('move_head', pan_pos=int(pan_pos), tilt_pos=self._head_tilt)
