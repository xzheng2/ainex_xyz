#!/usr/bin/env python3
"""Action behaviours for the marathon behavior tree."""
import time
import rospy
import py_trees
from py_trees.common import Access
from ros_robot_controller.msg import BuzzerState


class StopWalking(py_trees.behaviour.Behaviour):
    """Disable gait; always returns SUCCESS. No blackboard access."""

    def __init__(self, name, gait_manager):
        super().__init__(name)
        self.gait_manager = gait_manager

    def update(self):
        rospy.loginfo('[StopWalking] %s — gait disabled', self.name)
        self.gait_manager.disable()
        return py_trees.common.Status.SUCCESS


class FollowLine(py_trees.behaviour.Behaviour):
    """Read line_data from blackboard and run visual patrol step."""

    def __init__(self, name, visual_patrol):
        super().__init__(name)
        self.visual_patrol = visual_patrol
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/line_data", access=Access.READ)

    def update(self):
        line_data = self.bb.line_data
        rospy.loginfo('[FollowLine] x=%.1f width=%d', line_data.x, line_data.width)
        self.visual_patrol.process(line_data.x, line_data.width)
        return py_trees.common.Status.SUCCESS


class FindLine(py_trees.behaviour.Behaviour):
    """Turn in-place to recover a lost line, using last known position as hint.

    Reads /last_line_x and /line_lost_count from the blackboard.
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

    def __init__(self, name, visual_patrol):
        super().__init__(name)
        self.visual_patrol = visual_patrol
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/last_line_x", access=Access.READ)
        self.bb.register_key(key="/line_lost_count", access=Access.READ)

    def update(self):
        last_x = self.bb.last_line_x
        lost_count = self.bb.line_lost_count

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
            gait_yaw = -turn_deg

        rospy.loginfo('[FindLine] lost_count=%d  gait_yaw=%+d', lost_count, gait_yaw)
        self.visual_patrol.gait_manager.set_step(
            self.visual_patrol.turn_dsp,
            0,          # no forward progress while searching
            0,
            gait_yaw,
            self.visual_patrol.turn_gait_param,
            arm_swap=self.visual_patrol.turn_arm_swap,
            step_num=0,
        )
        return py_trees.common.Status.RUNNING


class RecoverFromFall(py_trees.behaviour.Behaviour):
    """
    Blocking recovery action: buzz → wait → disable gait → run stand-up
    action → write robot_state='stand'.

    Reads robot_state to select the correct stand-up action; writes
    'stand' on completion.
    """

    LIE_ACTION = 'lie_to_stand'
    RECLINE_ACTION = 'recline_to_stand'

    def __init__(self, name, motion_manager, gait_manager, buzzer_pub):
        super().__init__(name)
        self.motion_manager = motion_manager
        self.gait_manager = gait_manager
        self.buzzer_pub = buzzer_pub
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/robot_state", access=Access.WRITE)

    def update(self):
        state = self.bb.robot_state
        # Buzzer alert before recovery
        self.buzzer_pub.publish(
            BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1))
        time.sleep(2)

        self.gait_manager.disable()

        if state == 'lie_to_stand':
            rospy.loginfo('[RecoverFromFall] lie_to_stand')
            self.motion_manager.run_action(self.LIE_ACTION)
        elif state == 'recline_to_stand':
            rospy.loginfo('[RecoverFromFall] recline_to_stand')
            self.motion_manager.run_action(self.RECLINE_ACTION)
        else:
            rospy.logwarn('[RecoverFromFall] unknown state: %s', state)

        time.sleep(0.5)
        self.bb.robot_state = 'stand'
        return py_trees.common.Status.SUCCESS
