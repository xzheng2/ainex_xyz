#!/usr/bin/env python3
"""L2 Action: turn in-place to recover a lost line.

Reads /latched/last_line_x and /latched/camera_lost_count from the blackboard
to bias the search direction and scale the turn magnitude.

Always returns RUNNING so the Selector keeps ticking this node each cycle
until IsLineDetected (upstream Sequence) succeeds and takes over.
The StopWalking node behind it in the Selector is a structural fallback only.

Reference implementation: marathon/behaviours/actions.py :: FindLine
"""
import rospy
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.base_facade import AinexBTFacade


class L2_Gait_FindLine(AinexBTNode):
    """Turn in-place to recover a lost line. Always returns RUNNING."""

    LEVEL = 'L2'
    BB_LOG_KEYS = ['/latched/last_line_x', '/latched/camera_lost_count']

    def __init__(self, name: str = 'L2_Gait_FindLine',
                 facade: AinexBTFacade = None, tick_id_getter=None):
        super().__init__(name)
        self._facade = facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace='/latched')
        self._bb.register_key(key='last_line_x',       access=Access.READ)
        self._bb.register_key(key='camera_lost_count', access=Access.READ)

    def update(self) -> Status:
        gait_yaw = self._facade.search_line(
            last_line_x=self._bb.last_line_x,
            lost_count=self._bb.camera_lost_count,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        rospy.logdebug('[L2_Gait_FindLine] gait_yaw=%+d', gait_yaw)
        return Status.RUNNING
