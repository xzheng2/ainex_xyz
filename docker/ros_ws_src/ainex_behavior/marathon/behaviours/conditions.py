#!/usr/bin/env python3
"""Condition behaviours for the marathon behavior tree."""
import py_trees
from py_trees.common import Access


class IsRobotStanding(py_trees.behaviour.Behaviour):
    """SUCCESS if the robot is in the 'stand' state."""

    def __init__(self, name="IsRobotStanding"):
        super().__init__(name)
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/robot_state", access=Access.READ)

    def update(self):
        if self.bb.robot_state == 'stand':
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsLineDetected(py_trees.behaviour.Behaviour):
    """SUCCESS if line detection data is present on the blackboard."""

    def __init__(self, name="IsLineDetected"):
        super().__init__(name)
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/line_data", access=Access.READ)

    def update(self):
        if self.bb.line_data is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
