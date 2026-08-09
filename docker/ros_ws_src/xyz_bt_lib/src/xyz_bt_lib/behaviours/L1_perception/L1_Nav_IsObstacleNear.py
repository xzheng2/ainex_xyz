#!/usr/bin/env python3
"""L1_Nav_IsObstacleNear — condition: an obstacle is in the near-front window.

BB reads:
  BB.NAV_OBSTACLE_NEAR  (/latched/nav_obstacle_near)  — written each tick by DepthNavAdapter

BB writes:
  none

Question judged:
  Is there an obstacle block in the robot's near-front window this frame?
  (Intended for an emergency-stop / re-plan branch of a Selector.)

Judgement helper:
  _is_obstacle_near(obstacle_near) → bool

SUCCESS:
  nav_obstacle_near is True

FAILURE:
  nav_obstacle_near is False

Observability:
  Emits 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Nav_IsObstacleNear(XyzL1ConditionNode):
    """SUCCESS if /latched/nav_obstacle_near is True (dwell-configurable)."""

    LEVEL        = 'L1'
    BB_READS     = [BB.NAV_OBSTACLE_NEAR]
    BB_WRITES    = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {}

    def __init__(self, name: str = 'L1_Nav_IsObstacleNear',
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:               BT node name.
            logger:             DebugEventLogger-compatible object, or None.
            tick_id_getter:     Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.NAV_OBSTACLE_NEAR_KEY, access=Access.READ)

    def _is_obstacle_near(self, obstacle_near: bool) -> bool:
        """Return True when an obstacle is in the near-front window this frame."""
        return bool(obstacle_near)

    def update(self) -> Status:
        obstacle_near = self._bb.nav_obstacle_near
        passed        = self._is_obstacle_near(obstacle_near)
        status        = self.status_from_bool(passed)

        inputs = {'nav_obstacle_near': obstacle_near}
        reason = 'obstacle near' if passed else 'path clear ahead'

        self.emit_decision(inputs=inputs, status=status, reason=reason)
        return status
