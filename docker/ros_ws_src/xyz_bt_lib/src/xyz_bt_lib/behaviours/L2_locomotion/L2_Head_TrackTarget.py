#!/usr/bin/env python3
"""L2 Action: PID-track a visible target with the head servos.

Node kind: continuous_controller

This is PHASE 2 only — it assumes the target is already visible and keeps it
centred; it never searches. Pair it with L2_Head_SearchSweep on the tree:

    PrioritySelector(HeadControl)
        ReactiveSequence(Track)
            L1_Vision_IsObjectDetected(target_id)
            L2_Head_TrackTarget
        ReactiveSequence(Search)
            L2_Motion_StopGait
            L2_Head_SearchSweep

Replaces the TRACK phase of L2_Head_FindObjectSweep (deleted Aug 2026).

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects) — 'error_x' drives pan,
                      'error_y' drives tilt, 'lost_count' gates the whole node.

BB writes: none
Facade:    move_head(pan_pos, tilt_pos)
Strategy:  two independent PID loops driving each pixel error toward 0.

A deadband of track_deadband_px around the centre is treated as exactly zero
error, so the head does not jitter on top of a centred target.

Why PID state is instance state (unlike the sweep's position):
  L2_Head_SearchSweep keeps its pan position in the Blackboard because
  restarting a sweep from an endpoint is a visible behavioural regression when a
  flickering target bounces the tree between branches. PID integral/derivative
  terms are different: they are controller internals that re-converge within a
  tick or two, and starting them fresh on re-entry is both harmless and the
  conventional choice. Persisting stale integral windup across an interruption
  would be worse than resetting it.

Returns:
    RUNNING → target visible; head commanded this tick
    FAILURE → target absent or lost_count > lost_count_threshold, so a parent
              PrioritySelector falls through to the search branch

CONFIG_DEFAULTS:
    target_id:            'object' — key to look up in tracked_objects
    pan_range:            [300, 700] — pan servo clamp [lo, hi]
    tilt_range:           [280, 550] — tilt servo clamp (matches the servo-24 limit)
    tilt_enabled:         False   — when False the tilt servo is never commanded
    pan_start:            500     — pan position the loop starts from
    tilt_start:           450     — tilt position the loop starts from
    pid_pan_p/i/d:        0.1/0.0/0.0
    pid_tilt_p/i/d:       0.1/0.0/0.0
    track_deadband_px:    50      — |error| within this counts as zero
    lost_count_threshold: 0       — missed frames tolerated before FAILURE
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB


class _PID:
    """Minimal P-I-D controller with windup guard. No time.time() dependency.

    One call = one time unit, which suits a fixed-rate BT tick and keeps the
    controller honest when a blocking action stalls the tree (wall-clock dt
    would spike; tick-based dt does not).
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.windup_guard = 20.0
        self.clear()

    def clear(self):
        self.SetPoint   = 0.0
        self.PTerm      = 0.0
        self.ITerm      = 0.0
        self.DTerm      = 0.0
        self.last_error = 0.0
        self.output     = 0.0

    def update(self, feedback_value):
        error       = self.SetPoint - feedback_value
        delta_error = error - self.last_error

        self.PTerm = self.Kp * error
        self.ITerm = max(-self.windup_guard,
                         min(self.windup_guard, self.ITerm + error))
        self.DTerm = delta_error   # 1 tick per call implied
        self.last_error = error

        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)


class _PIDTrack:
    """Integrates PID output into a servo position, clamped to value_range."""

    def __init__(self, pid: _PID, value_range: list, value: float = 0):
        self.pid         = pid
        self.value_range = value_range   # [lo, hi]
        self.value       = value

    def update_position(self, value: float):
        """Snap the tracked value (e.g. to wherever a preceding sweep stopped)."""
        self.value = value

    def clear(self):
        self.pid.clear()

    def track(self, current: float, target: float) -> float:
        """Compute one PID step. Returns the updated servo position."""
        self.pid.SetPoint = target
        self.pid.update(current)
        self.value = max(self.value_range[0],
                         min(self.value_range[1], self.value + self.pid.output))
        return self.value


class L2_Head_TrackTarget(XyzL2ActionNode):
    """Keep a visible target centred in frame with the head servos."""

    LEVEL        = 'L2'
    BB_READS     = [BB.TRACKED_OBJECTS]
    BB_WRITES    = []
    FACADE_CALLS = ['move_head']
    CONFIG_DEFAULTS = {
        'target_id':            'object',
        'pan_range':            [300, 700],
        'tilt_range':           [280, 550],
        'tilt_enabled':         False,
        'pan_start':            500,
        'tilt_start':           450,
        'pid_pan_p':            0.1,
        'pid_pan_i':            0.0,
        'pid_pan_d':            0.0,
        'pid_tilt_p':           0.1,
        'pid_tilt_i':           0.0,
        'pid_tilt_d':           0.0,
        'track_deadband_px':    50,
        'lost_count_threshold': 0,
    }

    def __init__(self, name: str = 'L2_Head_TrackTarget',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 target_id: str = 'object',
                 pan_range: list = None,
                 tilt_range: list = None,
                 tilt_enabled: bool = False,
                 pan_start: int = 500,
                 tilt_start: int = 450,
                 pid_pan_p: float = 0.1,
                 pid_pan_i: float = 0.0,
                 pid_pan_d: float = 0.0,
                 pid_tilt_p: float = 0.1,
                 pid_tilt_i: float = 0.0,
                 pid_tilt_d: float = 0.0,
                 track_deadband_px: float = 50,
                 lost_count_threshold: int = 0):
        """
        Args:
            target_id:            Key to look up in the tracked_objects dict.
            pan_range/tilt_range: [lo, hi] servo clamps.
            tilt_enabled:         False = pan-only tracking; the tilt servo is
                                  never commanded (mirrors the old
                                  tilt_sweep_min=None behaviour).
            pan_start/tilt_start: Servo positions the PID loops start from on
                                  each activation.
            track_deadband_px:    |error| within this is treated as zero.
            lost_count_threshold: Missed frames tolerated before FAILURE.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._target_id    = target_id
        self._pan_range    = pan_range  or [300, 700]
        self._tilt_range   = tilt_range or [280, 550]
        self._tilt_enabled = tilt_enabled
        self._pan_start    = pan_start
        self._tilt_start   = tilt_start
        self._pid_pan_gains  = (pid_pan_p,  pid_pan_i,  pid_pan_d)
        self._pid_tilt_gains = (pid_tilt_p, pid_tilt_i, pid_tilt_d)
        self._track_deadband_px    = track_deadband_px
        self._lost_count_threshold = lost_count_threshold
        self._pan_track  = None
        self._tilt_track = None
        self._bb         = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)
        self._pan_track = _PIDTrack(_PID(*self._pid_pan_gains),
                                    self._pan_range, self._pan_start)
        self._tilt_track = _PIDTrack(_PID(*self._pid_tilt_gains),
                                     self._tilt_range, self._tilt_start)

    def initialise(self):
        """Start each activation from a clean controller (see module docstring)."""
        if self._pan_track is not None:
            self._pan_track.clear()
            self._pan_track.update_position(self._pan_start)
        if self._tilt_track is not None:
            self._tilt_track.clear()
            self._tilt_track.update_position(self._tilt_start)
        self.emit_action_intent(
            action='track_target',
            inputs={'target_id': self._target_id,
                    'tilt_enabled': self._tilt_enabled},
        )

    def _deadband(self, err: float) -> float:
        """Snap small errors to exactly zero. No BB/ROS/facade calls here."""
        return 0.0 if abs(err) < self._track_deadband_px else float(err)

    def update(self) -> Status:
        obj = (self._bb.tracked_objects or {}).get(self._target_id)
        if obj is None or obj['lost_count'] > self._lost_count_threshold:
            self.emit_decision(
                inputs={'target_id': self._target_id,
                        'lost_count': obj['lost_count'] if obj else None},
                status=Status.FAILURE,
                reason='target lost; hand back to the search branch',
            )
            return Status.FAILURE

        error_x = self._deadband(obj['error_x'])
        pan_pos = int(self._pan_track.track(error_x, 0.0))

        tilt_pos = None
        error_y  = None
        if self._tilt_enabled:
            error_y  = self._deadband(obj['error_y'])
            tilt_pos = int(self._tilt_track.track(error_y, 0.0))

        self.call_facade('move_head', pan_pos=pan_pos, tilt_pos=tilt_pos)
        self.emit_decision(
            inputs={'target_id': self._target_id, 'error_x': error_x,
                    'error_y': error_y, 'head_pan': pan_pos, 'head_tilt': tilt_pos},
            status=Status.RUNNING,
            reason='tracking target with PID',
        )
        return Status.RUNNING
