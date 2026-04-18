#!/usr/bin/env python3
"""L1 Condition: check whether the robot is in a fallen state.

Reads /latched/robot_state from the blackboard.
SUCCESS  → robot_state != 'stand'  (lie_to_stand, recline_to_stand, …)
FAILURE  → robot_state == 'stand'

Fall detection is performed by ImuBalanceStateAdapter, which subscribes to
/imu and writes BB.ROBOT_STATE every tick via the two-phase latch protocol.
This node only reads the result — no ROS subscription here.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Balance_IsFallen(AinexBTNode):
    """SUCCESS if /latched/robot_state != 'stand' (fall in progress)."""

    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.ROBOT_STATE]

    def __init__(self, name: str = 'L1_Balance_IsFallen',
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
        fallen = (state != 'stand')
        status = Status.SUCCESS if fallen else Status.FAILURE

        if self._logger:
            self._logger.emit_bt({
                'event':  'decision',
                'node':   self.name,
                'inputs': {'robot_state': state},
                'status': str(status),
                'reason': f"robot_state == '{state}'" if fallen
                          else "robot_state == 'stand'",
            })

        return status
