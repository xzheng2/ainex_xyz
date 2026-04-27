#!/usr/bin/env python3
"""L3 Mission: Execute a sequence of action names from BB.
Designed for ROSA agent dynamic task dispatch."""
import rospy
import py_trees
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L3_Mission_ExecuteSequence(XyzBTNode):
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.CURRENT_TASK]

    TASK_SEQUENCE_KEY = 'mission/task_sequence'

    def __init__(self, motion_manager, name='L3_Mission_ExecuteSequence'):
        super().__init__(name)
        self.motion_manager = motion_manager
        self._index = 0

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def initialise(self):
        self._index = 0

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage
        sequence = storage.get(self.TASK_SEQUENCE_KEY, [])

        if not sequence:
            return Status.FAILURE

        if self._index >= len(sequence):
            return Status.SUCCESS

        action_name = sequence[self._index]
        storage[BB.CURRENT_TASK] = action_name
        rospy.loginfo('[L3_Mission_ExecuteSequence] executing [%d/%d]: %s',
                      self._index + 1, len(sequence), action_name)

        self.motion_manager.run_action(action_name)
        self._index += 1

        if self._index >= len(sequence):
            return Status.SUCCESS
        return Status.RUNNING
