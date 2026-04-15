#!/usr/bin/env python3
"""Marathon-specific condition behaviours.

Only contains conditions that have no generic equivalent in ainex_bt_edu:

IsHeadCentered — marathon-specific because it reads /head_pan_pos which is
    written exclusively by L2_Head_FindLineSweep.  Used only in head-sweep mode
    to gate L2_Gait_FollowLine until the body is aligned with the line.

Generic conditions (IsRobotStanding, IsLineDetected) have been moved to:
    ainex_bt_edu/behaviours/L1_perception/
"""
from py_trees.common import Access, Status
import py_trees


class IsHeadCentered(py_trees.behaviour.Behaviour):
    """SUCCESS when /head_pan_pos is within CENTER_THRESHOLD of HEAD_PAN_CENTER.

    Used in head-sweep mode to gate FollowLine until L2_Head_FindLineSweep has
    finished aligning the body.  Must match L2_Head_FindLineSweep.CENTER_THRESHOLD.
    """

    HEAD_PAN_CENTER  = 500
    CENTER_THRESHOLD = 30

    def __init__(self, name: str = "IsHeadCentered"):
        super().__init__(name)
        self.bb = None

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="/head_pan_pos", access=Access.READ)

    def update(self) -> Status:
        if abs(self.bb.head_pan_pos - self.HEAD_PAN_CENTER) <= self.CENTER_THRESHOLD:
            return Status.SUCCESS
        return Status.FAILURE
