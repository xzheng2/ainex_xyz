#!/usr/bin/env python3
"""L2 Action: stop gait motion. Always returns SUCCESS.

BB reads:  none
BB writes: none
Facade:    stop_gait()
Strategy:  none (trivial: always dispatches stop and returns SUCCESS)

stop_gait() stops current gait motion; the gait controller stays up and
can immediately accept new set_step() commands.

Returns:
    SUCCESS → gait stop command dispatched (fire-and-forget)
"""
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzL2ActionNode
from xyz_bt_edu.base_facade import XyzBTFacade


class L2_Gait_Stop(XyzL2ActionNode):
    """Disable gait; always returns SUCCESS. No blackboard access."""

    LEVEL        = 'L2'
    BB_READS     = []
    BB_WRITES    = []
    FACADE_CALLS = ['stop_gait']
    CONFIG_DEFAULTS = {}

    def __init__(self, name: str = 'L2_Gait_Stop',
                 facade: XyzBTFacade = None,
                 logger=None,
                 tick_id_getter=None):
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter, facade=facade)

    def initialise(self):
        self.emit_action_intent(action='stop_gait')

    def update(self) -> Status:
        self.call_facade('stop_gait')
        self.emit_decision(
            inputs={},
            status=Status.SUCCESS,
            reason='stop command dispatched',
        )
        return Status.SUCCESS
