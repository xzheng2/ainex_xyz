#!/usr/bin/env python3
"""L3 Mission: Place object sequence — move arm to place, open gripper, retract."""
import time
import py_trees
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L3_Arm_PlaceObject(AinexBTNode):
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.GRIPPER_STATE]

    OPEN_POSITIONS = [[21, 200], [22, 800]]
    GRASP_DURATION = 500  # ms

    def __init__(self, motion_manager, name='L3_Arm_PlaceObject'):
        super().__init__(name)
        self.motion_manager = motion_manager
        self._state = 'OPEN'

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def initialise(self):
        self._state = 'OPEN'

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage

        if self._state == 'OPEN':
            self.motion_manager.set_servos_position(
                self.GRASP_DURATION, self.OPEN_POSITIONS)
            time.sleep(self.GRASP_DURATION / 1000.0 + 0.1)
            storage[BB.GRIPPER_STATE] = 'open'
            self._state = 'RETRACT'
            return Status.RUNNING

        elif self._state == 'RETRACT':
            self.motion_manager.run_action('stand')
            self._state = 'DONE'
            return Status.SUCCESS

        return Status.SUCCESS
