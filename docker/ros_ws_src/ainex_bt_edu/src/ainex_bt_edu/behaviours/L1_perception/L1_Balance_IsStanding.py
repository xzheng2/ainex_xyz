#!/usr/bin/env python3
"""L1 Condition: check whether the robot is in the standing state.

Reads /latched/robot_state from the blackboard.
SUCCESS  → robot_state == 'stand'
FAILURE  → any other state (lie_to_stand, recline_to_stand, …)

Reference implementation: marathon/behaviours/conditions.py :: IsRobotStanding
"""
import py_trees
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Balance_IsStanding(AinexBTNode):
    """SUCCESS if /latched/robot_state == 'stand'."""

    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.ROBOT_STATE]

    def __init__(self, name: str = 'L1_Balance_IsStanding',
                 logger=None, tick_id_getter=None):
        super().__init__(name)
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.ROBOT_STATE_KEY, access=Access.READ)

    def update(self) -> Status:
        state = self._bb.robot_state
        passed = (state == 'stand')
        status = Status.SUCCESS if passed else Status.FAILURE

        if self._logger:
            self._logger.emit_bt({
                'event': 'decision',
                'node': self.name,
                'inputs': {'robot_state': state},
                'status': str(status),
                'reason': "robot_state == 'stand'" if passed
                          else f"robot_state == '{state}'",
            })

        return status
