#!/usr/bin/env python3
"""L2 Action: disable gait controller. Always returns SUCCESS.

Delegates to facade.stop_walking(); no blackboard access required.

Reference implementation: marathon/behaviours/actions.py :: StopWalking
"""
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.base_facade import AinexBTFacade


class L2_Gait_Stop(AinexBTNode):
    """Disable gait; always returns SUCCESS. No blackboard access."""

    LEVEL = 'L2'
    BB_LOG_KEYS = []

    def __init__(self, name: str = 'L2_Gait_Stop',
                 facade: AinexBTFacade = None, tick_id_getter=None):
        super().__init__(name)
        self._facade = facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)

    def update(self) -> Status:
        self._facade.stop_walking(
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        return Status.SUCCESS
