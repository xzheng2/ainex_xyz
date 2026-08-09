#!/usr/bin/env python3
"""L1_Nav_IsObstacleDetected — condition: an obstacle is present anywhere in view.

BB reads:
  BB.NAV_OBSTACLE_DETECTED  (/latched/nav_obstacle_detected)  — written each tick by DepthNavAdapter

BB writes:
  none

Question judged:
  Is there any obstacle (depth or YOLO) anywhere in the planner's fused mask this frame?
  This is distance-independent — NOT gated to the near-front proximity window (unlike
  L1_Nav_IsObstacleNear). Intended as a "still avoiding" gate: invert it to detect that the
  scene has fully cleared.

Judgement helper:
  _is_obstacle_detected(obstacle_detected) → bool

SUCCESS:
  nav_obstacle_detected is True

FAILURE:
  nav_obstacle_detected is False

Observability:
  Emits 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Nav_IsObstacleDetected(XyzL1ConditionNode):
    """SUCCESS if /latched/nav_obstacle_detected is True (dwell-configurable)."""

    LEVEL        = 'L1'
    BB_READS     = [BB.NAV_OBSTACLE_DETECTED]
    BB_WRITES    = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {}

    def __init__(self, name: str = 'L1_Nav_IsObstacleDetected',
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
        self._bb.register_key(key=BB.NAV_OBSTACLE_DETECTED_KEY, access=Access.READ)

    def _is_obstacle_detected(self, obstacle_detected: bool) -> bool:
        """Return True when an obstacle is present anywhere in the fused mask this frame."""
        return bool(obstacle_detected)

    def update(self) -> Status:
        obstacle_detected = self._bb.nav_obstacle_detected
        passed            = self._is_obstacle_detected(obstacle_detected)
        status            = self.status_from_bool(passed)

        inputs = {'nav_obstacle_detected': obstacle_detected}
        reason = 'obstacle detected' if passed else 'scene clear'

        self.emit_decision(inputs=inputs, status=status, reason=reason)
        return status
