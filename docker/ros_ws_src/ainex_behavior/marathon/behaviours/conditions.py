#!/usr/bin/env python3
"""Condition behaviours for the marathon behavior tree."""
import py_trees
from py_trees.common import Access


class IsRobotStanding(py_trees.behaviour.Behaviour):
    """SUCCESS if the robot is in the 'stand' state."""

    def __init__(self, name="IsRobotStanding", logger=None, tick_id_getter=None):
        super().__init__(name)
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name, namespace="/latched")
        self.bb.register_key(key="robot_state", access=Access.READ)

    def update(self):
        state = self.bb.robot_state
        passed = (state == 'stand')
        status = py_trees.common.Status.SUCCESS if passed else py_trees.common.Status.FAILURE

        if self._logger:
            self._logger.emit_bt({
                "event": "decision",
                "node": self.name,
                "inputs": {"robot_state": state},
                "status": str(status),
                "reason": "robot_state == 'stand'" if passed else f"robot_state == '{state}'",
            })

        return status


class IsLineDetected(py_trees.behaviour.Behaviour):
    """SUCCESS if line detection data is present on the blackboard."""

    def __init__(self, name="IsLineDetected", logger=None, tick_id_getter=None):
        super().__init__(name)
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name, namespace="/latched")
        self.bb.register_key(key="line_data", access=Access.READ)

    def update(self):
        line_data = self.bb.line_data
        passed = (line_data is not None)
        status = py_trees.common.Status.SUCCESS if passed else py_trees.common.Status.FAILURE

        if self._logger:
            inputs = {"line_detected": passed}
            if passed:
                inputs["line_x"] = line_data.x
                inputs["line_width"] = line_data.width
            self._logger.emit_bt({
                "event": "decision",
                "node": self.name,
                "inputs": inputs,
                "status": str(status),
                "reason": "line_data present" if passed else "line_data is None",
            })

        return status


class IsHeadCentered(py_trees.behaviour.Behaviour):
    """SUCCESS when /head_pan_pos is within CENTER_THRESHOLD of HEAD_PAN_CENTER.

    Used in head-sweep mode to gate FollowLine until FindLineHeadSweep has
    finished aligning the body.  Must match FindLineHeadSweep.CENTER_THRESHOLD.
    """

    HEAD_PAN_CENTER  = 500
    CENTER_THRESHOLD = 30

    def __init__(self, name="IsHeadCentered"):
        super().__init__(name)
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/head_pan_pos", access=Access.READ)

    def update(self):
        if abs(self.bb.head_pan_pos - self.HEAD_PAN_CENTER) <= self.CENTER_THRESHOLD:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
