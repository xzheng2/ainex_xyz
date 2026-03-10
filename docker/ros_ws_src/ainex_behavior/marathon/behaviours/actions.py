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
        self.visual_patrol.process(line_data.x, line_data.width)
        return py_trees.common.Status.SUCCESS


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
