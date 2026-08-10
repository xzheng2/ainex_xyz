#!/usr/bin/env python3
"""L2 Action: sweep the head to find a target; SUCCESS the moment it appears.

Node kind: finite_action

This is PHASE 1 only. What happens once the target is found is the tree's
decision, not this node's — put the follow-up (head tracking, body alignment,
approach) in a higher-priority branch:

    PrioritySelector(HeadControl)
        ReactiveSequence(Track)
            L1_Vision_IsObjectDetected(target_id)
            L2_Head_TrackTarget
        ReactiveSequence(Search)
            L2_Motion_StopGait                 <- standing still is the tree's call
            L2_Head_SearchSweep

Replaces the SWEEP phase of L2_Head_FindObjectSweep and L2_Head_FindLineSweep
(both deleted Aug 2026). Those two embedded a SWEEP->(TRACK|ALIGN) state machine
inside the node class; splitting the phases moves that state machine onto the
tree, where it is readable, and lets either follow-up be composed with the same
search. It also drops their internal ``stop_gait()`` call — a Head node has no
business commanding the gait, so the tree now says so explicitly (see above).

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects) — target present with
                      lost_count == 0 ends the sweep.

BB writes:
  /node_state/<state_key>  {'pan': int, 'dir': int, 'tilt': int|None, 'pause': int}

Why sweep progress lives in the Blackboard:
  A flickering target makes the tree bounce between the Track and Search
  branches, and each re-entry calls initialise(). With instance state the head
  would jump back to a sweep endpoint every time and effectively never cover the
  arc. Keeping pan/direction in the BB (same mechanism as LatchedDwellDecorator)
  makes the sweep RESUME where it left off. PID controller state is deliberately
  NOT persisted this way — see L2_Head_TrackTarget.

Facade:    move_head(pan_pos, tilt_pos)
Strategy:  _next_pan(pan, direction) -> (pan, direction, reversed_at_right)

Sweep mechanics (unchanged from the deleted nodes):
  Pan steps by sweep_step each tick between sweep_right_pos and sweep_left_pos,
  holding sweep_pause_ticks ticks at each endpoint before reversing. When
  tilt_sweep_min/max are set, tilt advances one row by tilt_step at each RIGHT
  endpoint (a completed return pass), wrapping back to tilt_sweep_min — giving a
  2-D raster scan. When tilt_sweep_min is None the tilt servo is not commanded.

Returns:
    SUCCESS → target visible (lost_count == 0); sweep state is cleared
    RUNNING → still sweeping

CONFIG_DEFAULTS:
    target_id:         'object' — key to look up in tracked_objects
    sweep_left_pos:    700   — leftmost pan servo position (500 = centre)
    sweep_right_pos:   300   — rightmost pan servo position
    sweep_step:        10    — servo units per tick
    sweep_pause_ticks: 0     — ticks held at each endpoint before reversing
    tilt_sweep_min:    None  — lowest tilt position; None = never command tilt
    tilt_sweep_max:    None  — highest tilt position
    tilt_step:         50    — servo units advanced per completed pan pass
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL2ActionNode
from xyz_bt_lib.core.base_facade import XyzBTFacade
from xyz_bt_lib.blackboard.blackboard_keys import BB

NODE_STATE_NS = '/node_state'


class L2_Head_SearchSweep(XyzL2ActionNode):
    """Sweep the head until the target is visible, then SUCCESS."""

    LEVEL        = 'L2'
    BB_READS     = [BB.TRACKED_OBJECTS]
    BB_WRITES    = []
    FACADE_CALLS = ['move_head']
    PAN_INVERT   = 1     # +1: increasing pan servo counts turn the head LEFT
    CONFIG_DEFAULTS = {
        'target_id':         'object',
        'sweep_left_pos':    700,
        'sweep_right_pos':   300,
        'sweep_step':        10,
        'sweep_pause_ticks': 0,
        'tilt_sweep_min':    None,
        'tilt_sweep_max':    None,
        'tilt_step':         50,
    }

    def __init__(self, name: str = 'L2_Head_SearchSweep',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None,
                 state_key: str = None,
                 target_id: str = 'object',
                 sweep_left_pos: int = 700,
                 sweep_right_pos: int = 300,
                 sweep_step: int = 10,
                 sweep_pause_ticks: int = 0,
                 tilt_sweep_min: int = None,
                 tilt_sweep_max: int = None,
                 tilt_step: int = 50):
        """
        Args:
            state_key: REQUIRED unique key; sweep progress lives at
                       /node_state/<state_key>. Must be a hardcoded string
                       LITERAL in the tree (the xyz_bt_tree_pre_guard hook
                       enforces this) and unique across every node that uses the
                       /node_state/ namespace, including the dwell/hysteresis
                       decorators.
            target_id: Key to look up in the tracked_objects dict.
            (remaining args: see CONFIG_DEFAULTS in the module docstring)
        """
        if not state_key or not isinstance(state_key, str):
            raise ValueError(
                'L2_Head_SearchSweep requires a non-empty string state_key so '
                'sweep progress survives Track/Search re-entry and so key '
                'collisions surface at construction time.')
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)
        self._target_id         = target_id
        self._sweep_left_pos    = sweep_left_pos
        self._sweep_right_pos   = sweep_right_pos
        self._sweep_step        = sweep_step
        self._sweep_pause_ticks = sweep_pause_ticks
        self._tilt_sweep_min    = tilt_sweep_min
        self._tilt_sweep_max    = tilt_sweep_max
        self._tilt_step         = tilt_step
        self._state_path        = '{}/{}'.format(NODE_STATE_NS, state_key)
        self._bb                = None
        self._bb_state          = None
        self.BB_WRITES          = [self._state_path]

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)
        self._bb_state = self.attach_blackboard_client(name=self.name + '_state')
        # WRITE implicitly grants read in py_trees 2.1.6.
        self._bb_state.register_key(key=self._state_path, access=Access.WRITE)

    # ── sweep state in the Blackboard ──────────────────────────────────────

    def _read_state(self) -> dict:
        try:
            st = self._bb_state.get(self._state_path)
        except KeyError:
            st = None
        if not isinstance(st, dict):
            # First activation ever: start at the right endpoint sweeping left.
            return {'pan':   self._sweep_right_pos,
                    'dir':   +self.PAN_INVERT,
                    'tilt':  self._tilt_sweep_min,
                    'pause': 0}
        return dict(st)

    def _write_state(self, st: dict) -> None:
        self._bb_state.set(self._state_path, st)

    # ── pure helpers ───────────────────────────────────────────────────────

    def _next_pan(self, pan: int, direction: int) -> tuple:
        """Return (new_pan, new_dir, wrapped_tilt_row). No BB/ROS/facade calls."""
        pan = int(pan + direction * self._sweep_step)
        if pan >= self._sweep_left_pos:
            return self._sweep_left_pos, -self.PAN_INVERT, False
        if pan <= self._sweep_right_pos:
            # A right endpoint completes a return pass → advance the tilt row.
            return self._sweep_right_pos, +self.PAN_INVERT, True
        return pan, direction, False

    def _next_tilt(self, tilt):
        """Advance one raster row, wrapping. No-op when tilt sweeping is disabled."""
        if self._tilt_sweep_min is None or tilt is None:
            return tilt
        tilt += self._tilt_step
        if tilt > self._tilt_sweep_max:
            tilt = self._tilt_sweep_min
        return tilt

    def _has_target(self) -> bool:
        obj = (self._bb.tracked_objects or {}).get(self._target_id)
        return obj is not None and obj['lost_count'] == 0

    # ── tick ───────────────────────────────────────────────────────────────

    def update(self) -> Status:
        st = self._read_state()

        if self._has_target():
            # Clear progress so the NEXT search starts a fresh arc; the target
            # being visible is a genuine completion, not an interruption.
            self._write_state({'pan':   self._sweep_right_pos,
                               'dir':   +self.PAN_INVERT,
                               'tilt':  self._tilt_sweep_min,
                               'pause': 0})
            self.emit_decision(
                inputs={'target_id': self._target_id, 'head_pan': st['pan']},
                status=Status.SUCCESS,
                reason='target visible, search complete',
            )
            return Status.SUCCESS

        if st['pause'] > 0:
            st['pause'] -= 1
            self._write_state(st)
            self.emit_decision(
                inputs={'head_pan': st['pan'], 'pause': st['pause']},
                status=Status.RUNNING,
                reason='holding at sweep endpoint',
            )
            return Status.RUNNING

        st['pan'], st['dir'], advance_tilt = self._next_pan(st['pan'], st['dir'])
        if advance_tilt:
            st['tilt'] = self._next_tilt(st['tilt'])
        st['pause'] = self._sweep_pause_ticks
        self._write_state(st)

        self.call_facade('move_head', pan_pos=int(st['pan']), tilt_pos=st['tilt'])
        self.emit_decision(
            inputs={'target_id': self._target_id, 'head_pan': st['pan'],
                    'sweep_dir': st['dir'], 'head_tilt': st['tilt']},
            status=Status.RUNNING,
            reason='sweeping',
        )
        return Status.RUNNING
