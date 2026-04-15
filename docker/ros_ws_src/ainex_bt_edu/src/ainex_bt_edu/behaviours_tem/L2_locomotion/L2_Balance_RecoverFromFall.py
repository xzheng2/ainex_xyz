#!/usr/bin/env python3
"""L2 Action: Blocking recovery — buzz, wait, disable, stand-up action, write 'stand'."""
import time
import rospy
import py_trees
from py_trees.common import Access, Status
from ros_robot_controller.msg import BuzzerState
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L2_Balance_RecoverFromFall(AinexBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.ROBOT_STATE]

    LIE_ACTION = 'lie_to_stand'
    RECLINE_ACTION = 'recline_to_stand'

    def __init__(self, motion_manager, gait_manager, buzzer_pub,
                 name='L2_Balance_RecoverFromFall'):
        super().__init__(name)
        self.motion_manager = motion_manager
        self.gait_manager = gait_manager
        self.buzzer_pub = buzzer_pub
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(key=BB.ROBOT_STATE, access=Access.WRITE)

    def update(self):
        state = py_trees.blackboard.Blackboard.storage.get(
            BB.ROBOT_STATE, 'stand')

        self.buzzer_pub.publish(
            BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1))
        time.sleep(2)

        self.gait_manager.disable()

        if state == 'lie_to_stand':
            rospy.loginfo('[L2_Balance_RecoverFromFall] lie_to_stand')
            self.motion_manager.run_action(self.LIE_ACTION)
        elif state == 'recline_to_stand':
            rospy.loginfo('[L2_Balance_RecoverFromFall] recline_to_stand')
            self.motion_manager.run_action(self.RECLINE_ACTION)
        else:
            rospy.logwarn('[L2_Balance_RecoverFromFall] unknown state: %s',
                          state)

        time.sleep(0.5)
        py_trees.blackboard.Blackboard.storage[
            BB.ROBOT_STATE] = 'stand'
        return Status.SUCCESS
