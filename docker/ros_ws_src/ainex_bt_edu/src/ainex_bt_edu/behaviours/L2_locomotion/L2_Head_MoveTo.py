#!/usr/bin/env python3
"""L2 Action: move the head-pan servo to a fixed position.

Delegates to facade.move_head() — no direct ROS call here.

BB reads:  none
BB writes: none

Returns:
    SUCCESS → head command dispatched (always; the servo move is fire-and-forget)

Note: this node controls head pan only. Servo 23 (pan) is driven by the
facade; servo 24 (tilt) is not addressed here and retains its current position.
"""
from py_trees.common import Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.base_facade import AinexBTFacade


class L2_Head_MoveTo(AinexBTNode):
    """Move head-pan to a fixed position via facade. Always returns SUCCESS."""

    LEVEL = 'L2'
    BB_LOG_KEYS = []

    def __init__(self, pan_pos: int = 500,
                 name: str = 'L2_Head_MoveTo',
                 facade: AinexBTFacade = None,
                 tick_id_getter=None):
        super().__init__(name)
        self._pan_pos = pan_pos
        self._facade = facade
        self._tick_id_getter = tick_id_getter or (lambda: -1)

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def initialise(self):
        pass

    def update(self) -> Status:
        self._facade.move_head(
            pan_pos=self._pan_pos,
            bt_node=self.name,
            tick_id=self._tick_id_getter(),
        )
        return Status.SUCCESS

    def terminate(self, new_status: Status):
        pass
