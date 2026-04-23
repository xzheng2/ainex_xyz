#!/usr/bin/env python3
"""L1_Vision_IsTargetOnLeft — condition: tracked target is on the left side of frame.

BB reads:
  BB.TARGET_PIXEL_X  (/perception/target_pixel_x)

BB writes:
  none

Question judged:
  Is the detected target pixel X coordinate strictly left of the image centre?

Judgement helper:
  _is_on_left(x, center_x)

SUCCESS:
  target_pixel_x is not None AND target_pixel_x < image_center_x

FAILURE:
  target_pixel_x is None (no target present), or target_pixel_x >= image_center_x

CONFIG_DEFAULTS:
  image_width: 160  — frame width in pixels; center_x = image_width / 2.0.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from ainex_bt_edu.base_node import AinexL1ConditionNode
from ainex_bt_edu.blackboard_keys import BB


class L1_Vision_IsTargetOnLeft(AinexL1ConditionNode):
    """SUCCESS if /perception/target_pixel_x < image centre."""

    LEVEL = 'L1'
    BB_READS = [BB.TARGET_PIXEL_X]
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'image_width': 160,
    }

    def __init__(self, image_width: int = 160,
                 name: str = 'L1_Vision_IsTargetOnLeft',
                 logger=None, tick_id_getter=None):
        """
        Args:
            image_width:    Frame width in pixels; center_x = image_width / 2.0.
            name:           BT node name.
            logger:         DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._center_x = image_width / 2.0
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.PERCEPTION_NS)
        self._bb.register_key(key=BB.TARGET_PIXEL_X_KEY, access=Access.READ)

    def _is_on_left(self, x, center_x: float) -> bool:
        """Return True when x is not None and strictly less than center_x.

        No BB reads/writes, ROS calls, or logger calls here.
        """
        return x is not None and x < center_x

    def update(self) -> Status:
        x = self._bb.target_pixel_x
        passed = self._is_on_left(x, self._center_x)
        status = self.status_from_bool(passed)

        self.emit_decision(
            inputs={'target_pixel_x': x, 'center_x': self._center_x},
            status=status,
            reason=('no target' if x is None
                    else f'target_pixel_x={x} < center={self._center_x}' if passed
                    else f'target_pixel_x={x} >= center={self._center_x}'),
        )

        return status
