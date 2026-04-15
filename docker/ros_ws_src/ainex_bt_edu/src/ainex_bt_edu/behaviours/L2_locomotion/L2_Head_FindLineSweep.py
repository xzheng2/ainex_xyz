#!/usr/bin/env python3
"""L2 Action: sweep head to find a lost line, then align body before handing off.

Phase 1 — SWEEP
    Robot stands still.  Head pans between SWEEP_LEFT_POS and SWEEP_RIGHT_POS,
    pausing SWEEP_PAUSE_TICKS ticks at each end.
    Transitions to ALIGN as soon as the camera sees the line.

Phase 2 — ALIGN
    Head stays at the angle where the line was spotted; body turns
    proportionally while the head moves back toward centre.
    Returns SUCCESS when head_pan is within CENTER_THRESHOLD of HEAD_PAN_CENTER
    so the Selector re-evaluates: IsLineDetected + IsHeadCentered both pass →
    FollowLine takes over.

Blackboard I/O
    Reads  /latched/line_data, /latched/last_line_x
    Writes /head_pan_pos  (read by the companion IsHeadCentered condition)

Business-logic constants (ALIGN_TURN_MAX_DEG, ALIGN_TURN_SCALE, HEAD_PAN_SERVO,
HEAD_MOVE_DELAY_MS) live in the project SemanticFacade.  This node retains only
the state-machine constants needed to run the sweep/align loop.

── Configurable constants (edit here to tune behaviour) ───────────────────────
SWEEP_LEFT_POS     leftmost  servo position (500 = centre, 875 = hard left)
SWEEP_RIGHT_POS    rightmost servo position (500 = centre, 125 = hard right)
SWEEP_STEP         servo units per 30 Hz tick while sweeping
SWEEP_PAUSE_TICKS  ticks to hold at each endpoint before reversing
CENTER_THRESHOLD   ±units around 500 that counts as 'centred'
ALIGN_STEP         servo units per tick the head moves toward centre
───────────────────────────────────────────────────────────────────────────────

Reference implementation: marathon/behaviours/actions.py :: FindLineHeadSweep
"""
import rospy
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.base_facade import AinexBTFacade


class L2_Head_FindLineSweep(AinexBTNode):
    """Sweep-then-align head search. RUNNING during search; SUCCESS when aligned."""

    LEVEL = 'L2'
    BB_LOG_KEYS = ['/latched/line_data', '/latched/last_line_x', '/head_pan_pos']

    # ── Servo ──────────────────────────────────────────────────────────────
    HEAD_PAN_CENTER   = 500

    # ── Sweep config ───────────────────────────────────────────────────────
    SWEEP_LEFT_POS    = 700   # leftmost  (hw max 875)
    SWEEP_RIGHT_POS   = 300   # rightmost (hw min 125)
    SWEEP_STEP        = 10    # servo units per step
    SWEEP_PAUSE_TICKS = 0     # ticks at each end
    IMAGE_CENTER_X    = 80    # half of 160-px detection frame

    # ── Direction convention ───────────────────────────────────────────────
    # +1 → increasing servo value = head pans LEFT (confirmed hardware)
    PAN_INVERT = 1

    # ── Alignment config ───────────────────────────────────────────────────
    CENTER_THRESHOLD  = 30    # ±units; head within 500±30 → aligned
    ALIGN_STEP        = 7     # units/tick → ~960 ms for full range ≈ 2.4 gait cycles

    # ── States ─────────────────────────────────────────────────────────────
    _ST_SWEEP = 'sweep'
    _ST_ALIGN = 'align'

    def __init__(self, name: str = 'L2_Head_FindLineSweep',
                 facade: AinexBTFacade = None, tick_id_getter=None):
        super().__init__(name)
        self._facade          = facade
        self._tick_id_getter  = tick_id_getter or (lambda: -1)
        self._bb              = None
        self._bb_pan          = None
        self._head_pan        = self.HEAD_PAN_CENTER
        self._sweep_dir       = +self.PAN_INVERT  # initial: sweep left
        self._pause_ticks     = 0
        self._state           = self._ST_SWEEP
        self._fresh_start     = True  # True on first run and after each ALIGN success

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=f'{self.name}_latched', namespace='/latched')
        self._bb.register_key(key='line_data',   access=Access.READ)
        self._bb.register_key(key='last_line_x', access=Access.READ)

        self._bb_pan = self.attach_blackboard_client(name=self.name)
        self._bb_pan.register_key(key='/head_pan_pos', access=Access.WRITE)
        self._bb_pan.head_pan_pos = self.HEAD_PAN_CENTER  # initialise for IsHeadCentered

    def initialise(self):
        """Called on first activation and after each SUCCESS.  Reset to SWEEP.

        _sweep_dir is only updated from last_x on a genuine fresh start (first
        run, or just completed an ALIGN).  On mid-sweep re-activations caused by
        the memory=False Selector restarting the node every tick, _sweep_dir is
        preserved so the sweep continues in the same direction.

        stop_walking() is only called when line_data is None (SWEEP mode — robot
        stands still while scanning).  When line_data is present the node will
        immediately re-enter ALIGN, so the gait must not be stopped mid-turn.
        """
        if self._bb.line_data is None:            # SWEEP: stand still while scanning
            self._facade.stop_walking(
                bt_node=self.name,
                tick_id=self._tick_id_getter(),
            )
        self._state       = self._ST_SWEEP
        self._pause_ticks = 0

        if self._fresh_start:
            # Genuine fresh start: pick direction from last known line position
            self._fresh_start = False
            last_x = self._bb.last_line_x
            if last_x is not None and last_x < self.IMAGE_CENTER_X:
                self._sweep_dir = +self.PAN_INVERT   # line was left  → look left first
            else:
                self._sweep_dir = -self.PAN_INVERT   # line was right or unknown → look right
        # else: mid-sweep reinit (memory=False tick) — preserve _sweep_dir

        self._command_head(self._head_pan)
        rospy.loginfo('[L2_Head_FindLineSweep] initialise → SWEEP  head=%d  dir=%+d',
                      self._head_pan, self._sweep_dir)

    def update(self) -> Status:
        if self._state == self._ST_SWEEP:
            return self._update_sweep()
        return self._update_align()

    # ── Phase 1 ────────────────────────────────────────────────────────────

    def _update_sweep(self) -> Status:
        if self._bb.line_data is not None:
            rospy.loginfo('[L2_Head_FindLineSweep] SWEEP→ALIGN  head_pan=%d', self._head_pan)
            self._state = self._ST_ALIGN
            return self._update_align()

        # Hold at endpoint before reversing
        if self._pause_ticks > 0:
            self._pause_ticks -= 1
            return Status.RUNNING

        # Step head
        self._head_pan = int(self._head_pan + self._sweep_dir * self.SWEEP_STEP)

        if self._head_pan >= self.SWEEP_LEFT_POS:
            self._head_pan    = self.SWEEP_LEFT_POS
            self._sweep_dir   = -self.PAN_INVERT
            self._pause_ticks = self.SWEEP_PAUSE_TICKS
            rospy.loginfo('[L2_Head_FindLineSweep] reached LEFT end → reversing')
        elif self._head_pan <= self.SWEEP_RIGHT_POS:
            self._head_pan    = self.SWEEP_RIGHT_POS
            self._sweep_dir   = +self.PAN_INVERT
            self._pause_ticks = self.SWEEP_PAUSE_TICKS
            rospy.loginfo('[L2_Head_FindLineSweep] reached RIGHT end → reversing')

        self._command_head(self._head_pan)
        self._write_head_pan()
        rospy.logdebug('[L2_Head_FindLineSweep] sweep  head=%d  dir=%+d',
                       self._head_pan, self._sweep_dir)
        return Status.RUNNING

    # ── Phase 2 ────────────────────────────────────────────────────────────

    def _update_align(self) -> Status:
        head_offset = self._head_pan - self.HEAD_PAN_CENTER

        if abs(head_offset) <= self.CENTER_THRESHOLD:
            # Head is centred — snap to exact centre and signal done
            self._head_pan = self.HEAD_PAN_CENTER
            self._command_head(self.HEAD_PAN_CENTER)
            self._write_head_pan()
            self._facade.stop_walking(
                bt_node=self.name,
                tick_id=self._tick_id_getter(),
            )
            self._fresh_start = True   # next initialise() is a genuine restart
            rospy.loginfo('[L2_Head_FindLineSweep] ALIGN complete → SUCCESS')
            return Status.SUCCESS

        # Move head one step toward centre (don't overshoot)
        step = -self.ALIGN_STEP if head_offset > 0 else self.ALIGN_STEP
        new_pan = self._head_pan + step
        self._head_pan = (min(new_pan, self.HEAD_PAN_CENTER) if step > 0
                          else max(new_pan, self.HEAD_PAN_CENTER))
        self._command_head(self._head_pan)
        self._write_head_pan()

        # Turn body proportionally toward where head is pointing
        gait_yaw = self._facade.head_sweep_align(
            head_offset=head_offset,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        rospy.loginfo('[L2_Head_FindLineSweep] ALIGN  head=%d  offset=%+d  yaw=%+d',
                      self._head_pan, head_offset, gait_yaw)
        return Status.RUNNING

    # ── Helpers ────────────────────────────────────────────────────────────

    def _command_head(self, pan_pos: int):
        self._facade.move_head(
            pan_pos=int(pan_pos),
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )

    def _write_head_pan(self):
        self._bb_pan.head_pan_pos = self._head_pan
