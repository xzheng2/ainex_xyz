#!/usr/bin/env python3
"""L1 Condition: check whether the tracked target is on the left side of the frame.

Reads BB.TARGET_PIXEL_X from the blackboard.
SUCCESS  → target_pixel_x < image_width / 2  (target is left of centre)
FAILURE  → target_pixel_x >= image_width / 2, or no target present

Requires BB.TARGET_PIXEL_X to be written by an input adapter or tracking node.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexBTNode
from ainex_bt_edu.blackboard_keys import BB

_PERCEPTION_NS = '/perception'
_TARGET_PIXEL_X_KEY = 'target_pixel_x'


class L1_Vision_IsTargetOnLeft(AinexBTNode):
    """SUCCESS if /perception/target_pixel_x < image centre."""

    LEVEL = 'L1'
    BB_LOG_KEYS = [BB.TARGET_PIXEL_X]

    def __init__(self, image_width: int = 160,
                 name: str = 'L1_Vision_IsTargetOnLeft',
                 logger=None, tick_id_getter=None):
        super().__init__(name)
        self._center_x = image_width / 2.0
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=_PERCEPTION_NS)
        self._bb.register_key(key=_TARGET_PIXEL_X_KEY, access=Access.READ)

    def update(self) -> Status:
        x = self._bb.target_pixel_x
        if x is not None and x < self._center_x:
            status = Status.SUCCESS
            reason = f'target_pixel_x={x} < center={self._center_x}'
        else:
            status = Status.FAILURE
            reason = ('no target' if x is None
                      else f'target_pixel_x={x} >= center={self._center_x}')

        if self._logger:
            self._logger.emit_bt({
                'event':  'decision',
                'node':   self.name,
                'inputs': {'target_pixel_x': x, 'center_x': self._center_x},
                'status': str(status),
                'reason': reason,
            })

        return status
