#!/usr/bin/env python3
"""Action behaviours for the marathon behavior tree.

All action nodes express business intent only — they do NOT directly call
managers, publishers, or the Generic ROS Facade.  All calls go through the
Project Semantic Facade (MarathonSemanticFacade), which translates intent to
project-semantic commands and delegates ROS I/O to CommFacade.
"""
import py_trees
from py_trees.common import Access
import rospy


class StopWalking(py_trees.behaviour.Behaviour):
    """Disable gait; always returns SUCCESS. No blackboard access."""

    def __init__(self, name, semantic_facade, tick_id_getter=None):
        super().__init__(name)
        self._semantic = semantic_facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)

    def update(self):
        rospy.loginfo('[StopWalking] %s — gait disabled', self.name)
        self._semantic.stop_walking(bt_node=self.name, tick_id=self._tick_id_getter())
        return py_trees.common.Status.SUCCESS


class FollowLine(py_trees.behaviour.Behaviour):
    """Read line_data from blackboard and run visual patrol step."""

    def __init__(self, name, semantic_facade, tick_id_getter=None):
        super().__init__(name)
        self._semantic = semantic_facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name, namespace="/latched")
        self.bb.register_key(key="line_data", access=Access.READ)

    def update(self):
        line_data = self.bb.line_data
        rospy.loginfo('[FollowLine] x=%.1f width=%d', line_data.x, line_data.width)
        self._semantic.follow_line(
            line_data=line_data,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        return py_trees.common.Status.SUCCESS


class FindLine(py_trees.behaviour.Behaviour):
    """Turn in-place to recover a lost line, using last known position as hint.

    Reads /latched/last_line_x and /latched/camera_lost_count from the BB.
    Always returns RUNNING so the Selector keeps ticking this node each cycle
    until IsLineDetected upstream succeeds and takes over.
    StopWalking behind it acts as a structural fallback only.
    """

    def __init__(self, name, semantic_facade, tick_id_getter=None):
        super().__init__(name)
        self._semantic = semantic_facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name, namespace="/latched")
        self.bb.register_key(key="last_line_x",       access=Access.READ)
        self.bb.register_key(key="camera_lost_count", access=Access.READ)

    def update(self):
        self._semantic.search_line(
            last_line_x=self.bb.last_line_x,
            lost_count=self.bb.camera_lost_count,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        return py_trees.common.Status.RUNNING


class FindLineHeadSweep(py_trees.behaviour.Behaviour):
    """Sweep head to find a lost line, then align body before handing off.

    Phase 1 — SWEEP
        Robot stands still.  Head pans between SWEEP_LEFT_POS and
        SWEEP_RIGHT_POS, pausing SWEEP_PAUSE_TICKS ticks at each end.
        Transitions to ALIGN as soon as the camera sees the line.

    Phase 2 — ALIGN
        Head stays at the angle where the line was spotted; body turns
        proportionally while the head moves back toward centre.  Returns
        SUCCESS when head_pan is within CENTER_THRESHOLD of HEAD_PAN_CENTER
        so the Selector re-evaluates: IsLineDetected + IsHeadCentered both
        pass → FollowLine takes over.

    Writes /head_pan_pos to the blackboard (read by IsHeadCentered).

    Business-logic constants (ALIGN_TURN_MAX_DEG, ALIGN_TURN_SCALE, HEAD_PAN_SERVO,
    HEAD_MOVE_DELAY_MS) have moved to MarathonSemanticFacade.  This node retains
    only the state-machine constants needed to run the sweep/align loop.

    ── Configurable constants (edit here to tune behaviour) ────────────────
    SWEEP_LEFT_POS      leftmost  servo position (500 = centre, 875 = hard left)
    SWEEP_RIGHT_POS     rightmost servo position (500 = centre, 125 = hard right)
    SWEEP_STEP          servo units per 30 Hz tick while sweeping
    SWEEP_PAUSE_TICKS   ticks to hold at each endpoint before reversing
    CENTER_THRESHOLD    ±units around 500 that counts as 'centred'
    ALIGN_STEP          servo units per tick the head moves toward centre
    ────────────────────────────────────────────────────────────────────────
    """

    # ── Servo ──────────────────────────────────────────────────────────────
    HEAD_PAN_CENTER    = 500

    # ── Sweep config ───────────────────────────────────────────────────────
    SWEEP_LEFT_POS     = 700   # leftmost  (hw max 875)
    SWEEP_RIGHT_POS    = 300   # rightmost (hw min 125)
    SWEEP_STEP         = 10    # servo units per step
    SWEEP_PAUSE_TICKS  = 0     # ticks at each end
    IMAGE_CENTER_X     = 80    # half of 160-px detection frame

    # ── Direction convention ───────────────────────────────────────────────
    # +1 → increasing servo value = head pans LEFT (confirmed hardware)
    PAN_INVERT = 1

    # ── Alignment config ───────────────────────────────────────────────────
    CENTER_THRESHOLD   = 30    # ±units; head within 500±30 → aligned
    ALIGN_STEP         = 7     # units/tick → ~960 ms for full range ≈ 2.4 gait cycles

    # ── States ─────────────────────────────────────────────────────────────
    _ST_SWEEP = 'sweep'
    _ST_ALIGN = 'align'

    def __init__(self, name, semantic_facade, tick_id_getter=None):
        super().__init__(name)
        self._semantic      = semantic_facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self.bb             = None
        self.bb_pan         = None
        self._head_pan      = self.HEAD_PAN_CENTER
        self._sweep_dir     = +self.PAN_INVERT  # initial direction: sweep left
        self._pause_ticks   = 0
        self._state         = self._ST_SWEEP
        self._fresh_start   = True   # True on first run and after each ALIGN success

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(
            name=f"{self.name}_latched", namespace="/latched")
        self.bb.register_key(key="line_data",   access=Access.READ)
        self.bb.register_key(key="last_line_x", access=Access.READ)

        self.bb_pan = self.attach_blackboard_client(name=self.name)
        self.bb_pan.register_key(key="/head_pan_pos", access=Access.WRITE)
        self.bb_pan.head_pan_pos = self.HEAD_PAN_CENTER  # initialise for IsHeadCentered

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
        if self.bb.line_data is None:            # SWEEP: stand still while scanning
            self._semantic.stop_walking(bt_node=self.name,
                                        tick_id=self._tick_id_getter())
        self._state        = self._ST_SWEEP
        self._pause_ticks  = 0

        if self._fresh_start:
            # Genuine fresh start: pick direction from last known line position
            self._fresh_start = False
            last_x = self.bb.last_line_x
            if last_x is not None and last_x < self.IMAGE_CENTER_X:
                self._sweep_dir = +self.PAN_INVERT   # line was left  → look left first
            else:
                self._sweep_dir = -self.PAN_INVERT   # line was right or unknown → look right first
        # else: mid-sweep reinit (memory=False tick) — preserve _sweep_dir

        self._command_head(self._head_pan)
        rospy.loginfo('[FindLineHeadSweep] initialise → SWEEP  head=%d  dir=%+d',
                      self._head_pan, self._sweep_dir)

    def update(self):
        if self._state == self._ST_SWEEP:
            return self._update_sweep()
        return self._update_align()

    # ── Phase 1 ────────────────────────────────────────────────────────────

    def _update_sweep(self):
        if self.bb.line_data is not None:
            rospy.loginfo('[FindLineHeadSweep] SWEEP→ALIGN  head_pan=%d', self._head_pan)
            self._state = self._ST_ALIGN
            return self._update_align()

        # Hold at endpoint before reversing
        if self._pause_ticks > 0:
            self._pause_ticks -= 1
            return py_trees.common.Status.RUNNING

        # Step head
        self._head_pan = int(self._head_pan + self._sweep_dir * self.SWEEP_STEP)

        if self._head_pan >= self.SWEEP_LEFT_POS:
            self._head_pan    = self.SWEEP_LEFT_POS
            self._sweep_dir   = -self.PAN_INVERT
            self._pause_ticks = self.SWEEP_PAUSE_TICKS
            rospy.loginfo('[FindLineHeadSweep] reached LEFT end → reversing')
        elif self._head_pan <= self.SWEEP_RIGHT_POS:
            self._head_pan    = self.SWEEP_RIGHT_POS
            self._sweep_dir   = +self.PAN_INVERT
            self._pause_ticks = self.SWEEP_PAUSE_TICKS
            rospy.loginfo('[FindLineHeadSweep] reached RIGHT end → reversing')

        self._command_head(self._head_pan)
        self._write_head_pan()
        rospy.logdebug('[FindLineHeadSweep] sweep  head=%d  dir=%+d',
                       self._head_pan, self._sweep_dir)
        return py_trees.common.Status.RUNNING

    # ── Phase 2 ────────────────────────────────────────────────────────────

    def _update_align(self):
        head_offset = self._head_pan - self.HEAD_PAN_CENTER

        if abs(head_offset) <= self.CENTER_THRESHOLD:
            # Head is centred — snap to exact centre and signal done
            self._head_pan = self.HEAD_PAN_CENTER
            self._command_head(self.HEAD_PAN_CENTER)
            self._write_head_pan()
            self._semantic.stop_walking(bt_node=self.name,
                                        tick_id=self._tick_id_getter())
            self._fresh_start = True   # next initialise() is a genuine restart
            rospy.loginfo('[FindLineHeadSweep] ALIGN complete → SUCCESS')
            return py_trees.common.Status.SUCCESS

        # Move head one step toward centre (don't overshoot)
        step = -self.ALIGN_STEP if head_offset > 0 else self.ALIGN_STEP
        new_pan = self._head_pan + step
        if step > 0:
            self._head_pan = min(new_pan, self.HEAD_PAN_CENTER)
        else:
            self._head_pan = max(new_pan, self.HEAD_PAN_CENTER)
        self._command_head(self._head_pan)
        self._write_head_pan()

        # Turn body proportionally toward where head is pointing
        gait_yaw = self._semantic.head_sweep_align(
            head_offset=head_offset,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        rospy.loginfo('[FindLineHeadSweep] ALIGN  head=%d  offset=%+d  yaw=%+d',
                      self._head_pan, head_offset, gait_yaw)
        return py_trees.common.Status.RUNNING

    # ── Helpers ────────────────────────────────────────────────────────────

    def _command_head(self, pan_pos):
        self._semantic.move_head(pan_pos=int(pan_pos), bt_node=self.name,
                                 tick_id=self._tick_id_getter())

    def _write_head_pan(self):
        self.bb_pan.head_pan_pos = self._head_pan


class RecoverFromFall(py_trees.behaviour.Behaviour):
    """
    Blocking recovery action: delegate ROS sequence to semantic facade,
    then write robot_state='stand' to blackboard.

    Reads robot_state to select the correct stand-up action; writes
    'stand' on completion.
    """

    def __init__(self, name, semantic_facade, robot_state_setter=None,
                 tick_id_getter=None):
        super().__init__(name)
        self._semantic = semantic_facade
        # Called after writing 'stand' to the latched BB so the live store is
        # also updated — prevents the next pre-tick latch from overwriting
        # the latched key with the stale pre-recovery value.
        self._robot_state_setter = robot_state_setter
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name, namespace="/latched")
        self.bb.register_key(key="robot_state", access=Access.WRITE)

    def update(self):
        state = self.bb.robot_state
        self._semantic.recover_from_fall(
            robot_state=state,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        self.bb.robot_state = 'stand'
        # Also push 'stand' into the live store so the next pre-tick latch
        # does not overwrite the BB with the stale pre-recovery live value.
        if self._robot_state_setter is not None:
            self._robot_state_setter('stand')
        return py_trees.common.Status.SUCCESS
