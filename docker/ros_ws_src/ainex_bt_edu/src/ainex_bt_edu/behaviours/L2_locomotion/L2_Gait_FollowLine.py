#!/usr/bin/env python3
"""L2 Action: read line_data from blackboard and follow the line.

Reads /latched/line_data from the blackboard.
Also centres the head each tick so it does not drift in head-sweep mode.
Returns SUCCESS every tick (the Sequence upstream gates entry via IsLineDetected).

Reference implementation: marathon/behaviours/actions.py :: FollowLine
"""
import rospy
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.base_facade import AinexBTFacade
from ainex_bt_edu.blackboard_keys import BB


class L2_Gait_FollowLine(AinexBTNode):
    """Read line_data from BB and run one visual-patrol step via facade."""

    LEVEL = 'L2'
    BB_LOG_KEYS = [BB.LINE_DATA]

    HEAD_PAN_CENTER = 500  # servo units; centre position

    def __init__(self, name: str = 'L2_Gait_FollowLine',
                 facade: AinexBTFacade = None, tick_id_getter=None):
        super().__init__(name)
        self._facade = facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.LINE_DATA_KEY, access=Access.READ)

    def update(self) -> Status:
        line_data = self._bb.line_data
        # Centre the head each tick while following.  In head-sweep mode
        # IsHeadCentered may pass before the servo physically reaches centre,
        # so this keeps the head driven toward centre during line-following.
        self._facade.move_head(
            pan_pos=self.HEAD_PAN_CENTER,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        rospy.loginfo('[L2_Gait_FollowLine] x=%.1f width=%d',
                      line_data.x, line_data.width)
        self._facade.follow_line(
            line_data=line_data,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        return Status.SUCCESS
