#!/usr/bin/env python3
"""Action behaviours for the marathon behavior tree."""
import time
import rospy
import py_trees
from py_trees.common import Access
from ros_robot_controller.msg import BuzzerState
from bt_observability.ros_comm_tracer import proxy_context


class StopWalking(py_trees.behaviour.Behaviour):
    """Disable gait; always returns SUCCESS. No blackboard access."""

    def __init__(self, name, facade):
        super().__init__(name)
        self._facade = facade

    def update(self):
        rospy.loginfo('[StopWalking] %s — gait disabled', self.name)
        self._facade.disable_gait(bt_node=self.name)
        return py_trees.common.Status.SUCCESS


class FollowLine(py_trees.behaviour.Behaviour):
    """Read line_data from blackboard and run visual patrol step."""

    def __init__(self, name, visual_patrol, logger=None, tick_id_getter=None):
        super().__init__(name)
        self.visual_patrol = visual_patrol
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name, namespace="/latched")
        self.bb.register_key(key="line_data", access=Access.READ)

    def update(self):
        line_data = self.bb.line_data
        rospy.loginfo('[FollowLine] x=%.1f width=%d', line_data.x, line_data.width)
        # visual_patrol holds a ManagerProxy-wrapped gait_manager; proxy_context
        # attributes the resulting set_step call to this BT node.
        proxy_context.current_node = self.name
        self.visual_patrol.process(line_data.x, line_data.width)
        return py_trees.common.Status.SUCCESS


class FindLine(py_trees.behaviour.Behaviour):
    """Turn in-place to recover a lost line, using last known position as hint.

    Reads /latched/last_line_x and /latched/camera_lost_count from the BB.
    Always returns RUNNING so the Selector keeps ticking this node each cycle
    until IsLineDetected upstream succeeds and takes over.
    StopWalking behind it acts as a structural fallback only.
    """

    # Half of image_process_size[0] (160px) — centre of the detection frame
    IMAGE_CENTER_X = 80

    # Turn magnitude (degrees).  Grows with lost_count, capped at MAX_TURN_DEG.
    BASE_TURN_DEG = 3     # conservative angle right after losing the line
    MAX_TURN_DEG = 7      # upper cap to keep motion stable
    COUNT_SCALE_AT = 30   # lost_count at which turn reaches MAX_TURN_DEG

    # When no last_line_x history exists, rotate slowly in a fixed direction
    DEFAULT_TURN_DEG = 3

    def __init__(self, name, visual_patrol, facade):
        super().__init__(name)
        self.visual_patrol = visual_patrol
        self._facade = facade
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name, namespace="/latched")
        self.bb.register_key(key="last_line_x",       access=Access.READ)
        self.bb.register_key(key="camera_lost_count", access=Access.READ)

    def update(self):
        last_x = self.bb.last_line_x
        lost_count = self.bb.camera_lost_count

        # Scale turn intensity linearly with lost_count, capped at MAX_TURN_DEG
        scale = min(1.0, lost_count / max(self.COUNT_SCALE_AT, 1))
        turn_deg = int(
            self.BASE_TURN_DEG
            + scale * (self.MAX_TURN_DEG - self.BASE_TURN_DEG)
        )

        if last_x is None:
            # No history: rotate slowly left (positive = left in gait convention)
            gait_yaw = self.DEFAULT_TURN_DEG
        elif last_x < self.IMAGE_CENTER_X:
            # Line was last seen on left → turn left (positive gait yaw)
            gait_yaw = turn_deg
        else:
            # Line was last seen on right → turn right (negative gait yaw)
            gait_yaw = -5

        rospy.loginfo('[FindLine] lost_count=%d  gait_yaw=%+d', lost_count, gait_yaw)
        self._facade.set_step(
            bt_node=self.name,
            dsp=self.visual_patrol.turn_dsp,
            x=0,          # no forward progress while searching
            y=0,
            yaw=gait_yaw,
            gait_param=self.visual_patrol.turn_gait_param,
            arm_swap=self.visual_patrol.turn_arm_swap,
            step_num=0,
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

    ── Configurable constants (edit here to tune behaviour) ────────────────
    SWEEP_LEFT_POS      leftmost  servo position (500 = centre, 875 = hard left)
    SWEEP_RIGHT_POS     rightmost servo position (500 = centre, 125 = hard right)
    SWEEP_STEP          servo units per 30 Hz tick while sweeping
    SWEEP_PAUSE_TICKS   ticks to hold at each endpoint before reversing
    HEAD_MOVE_DELAY_MS  ms passed to the servo command (controls response speed)
    CENTER_THRESHOLD    ±units around 500 that counts as 'centred'
    ALIGN_STEP          servo units per tick the head moves toward centre
    ALIGN_TURN_MAX_DEG  maximum body yaw commanded during alignment (degrees)
    ────────────────────────────────────────────────────────────────────────
    """

    # ── Servo ──────────────────────────────────────────────────────────────
    HEAD_PAN_SERVO     = 23
    HEAD_PAN_CENTER    = 500

    # ── Sweep config ───────────────────────────────────────────────────────
    SWEEP_LEFT_POS     = 700   # leftmost  (hw max 875)
    SWEEP_RIGHT_POS    = 300   # rightmost (hw min 125)
    SWEEP_STEP         = 10    # servo units per step
    SWEEP_PAUSE_TICKS  = 0    # ticks at each end → ≈ 0.67 s pause at 15 Hz
    HEAD_MOVE_DELAY_MS = 50

    # ── Direction convention ───────────────────────────────────────────────
    # +1 → increasing servo value = head pans LEFT (confirmed hardware)
    # Set to -1 if servo is mechanically mirrored on your unit
    PAN_INVERT = 1

    # ── Alignment config ───────────────────────────────────────────────────
    CENTER_THRESHOLD   = 30    # ±units; head within 500±30 → aligned
    ALIGN_STEP         = 7     # units/tick → ~960 ms for full range ≈ 2.4 gait cycles
    ALIGN_TURN_MAX_DEG = 7     # max body yaw during alignment (degrees)
    ALIGN_TURN_SCALE   = ALIGN_TURN_MAX_DEG / (SWEEP_LEFT_POS - HEAD_PAN_CENTER)

    # ── Image ──────────────────────────────────────────────────────────────
    IMAGE_CENTER_X = 80        # half of 160-px image_process_size width

    # ── States ─────────────────────────────────────────────────────────────
    _ST_SWEEP = 'sweep'
    _ST_ALIGN = 'align'

    def __init__(self, name, facade, visual_patrol):
        super().__init__(name)
        self._facade        = facade
        self.visual_patrol  = visual_patrol
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

        gait_manager.disable() is only called when line_data is None (SWEEP
        mode — robot stands still while scanning).  When line_data is present
        the node will immediately re-enter ALIGN, so the gait must not be
        stopped mid-turn.
        """
        if self.bb.line_data is None:            # SWEEP: stand still while scanning
            self._facade.disable_gait(bt_node=self.name)
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
            self._facade.disable_gait(bt_node=self.name)
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
        gait_yaw = int(max(-self.ALIGN_TURN_MAX_DEG,
                           min(self.ALIGN_TURN_MAX_DEG,
                               head_offset * self.ALIGN_TURN_SCALE * self.PAN_INVERT)))
        rospy.loginfo('[FindLineHeadSweep] ALIGN  head=%d  offset=%+d  yaw=%+d',
                      self._head_pan, head_offset, gait_yaw)
        if abs(gait_yaw) < 2:
            self._facade.set_step(
                bt_node=self.name,
                dsp=self.visual_patrol.go_dsp, x=0, y=0, yaw=gait_yaw,
                gait_param=self.visual_patrol.go_gait_param,
                arm_swap=self.visual_patrol.go_arm_swap, step_num=0)
        else:
            self._facade.set_step(
                bt_node=self.name,
                dsp=self.visual_patrol.turn_dsp, x=0, y=0, yaw=gait_yaw,
                gait_param=self.visual_patrol.turn_gait_param,
                arm_swap=self.visual_patrol.turn_arm_swap, step_num=0)
        return py_trees.common.Status.RUNNING

    # ── Helpers ────────────────────────────────────────────────────────────

    def _command_head(self, pan_pos):
        self._facade.set_servos_position(
            self.HEAD_MOVE_DELAY_MS, [[self.HEAD_PAN_SERVO, int(pan_pos)]],
            bt_node=self.name)

    def _write_head_pan(self):
        self.bb_pan.head_pan_pos = self._head_pan


class RecoverFromFall(py_trees.behaviour.Behaviour):
    """
    Blocking recovery action: buzz → wait → disable gait → run stand-up
    action → write robot_state='stand'.

    Reads robot_state to select the correct stand-up action; writes
    'stand' on completion.
    """

    LIE_ACTION = 'lie_to_stand'
    RECLINE_ACTION = 'recline_to_stand'

    def __init__(self, name, facade, robot_state_setter=None):
        super().__init__(name)
        self._facade = facade
        # Called after writing 'stand' to the latched BB so the live store is
        # also updated — prevents the next pre-tick latch from overwriting
        # the latched key with the stale pre-recovery value.
        self._robot_state_setter = robot_state_setter
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name, namespace="/latched")
        self.bb.register_key(key="robot_state", access=Access.WRITE)

    def update(self):
        state = self.bb.robot_state

        # Buzzer alert before recovery
        buzzer_msg = BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1)
        self._facade.publish_buzzer(
            buzzer_msg, bt_node=self.name, reason="fall detected alert")
        time.sleep(2)

        self._facade.disable_gait(bt_node=self.name)

        if state == 'lie_to_stand':
            rospy.loginfo('[RecoverFromFall] lie_to_stand')
            action_name = self.LIE_ACTION
        elif state == 'recline_to_stand':
            rospy.loginfo('[RecoverFromFall] recline_to_stand')
            action_name = self.RECLINE_ACTION
        else:
            rospy.logwarn('[RecoverFromFall] unknown state: %s', state)
            action_name = None

        if action_name:
            self._facade.run_action(action_name, bt_node=self.name)

        time.sleep(0.5)
        self.bb.robot_state = 'stand'
        # Also push 'stand' into the live store so the next pre-tick latch
        # does not overwrite the BB with the stale pre-recovery live value.
        if self._robot_state_setter is not None:
            self._robot_state_setter('stand')
        return py_trees.common.Status.SUCCESS
