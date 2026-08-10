#!/usr/bin/env python3
"""L1_Nav_HasPath — condition: a navigable A* path currently exists.

BB reads:
  BB.NAV_HAS_PATH  (/latched/nav_has_path)  — written each tick by DepthNavAdapter

BB writes:
  none

Question judged:
  Did the depth-nav planner find a path from the start cell to a goal row this frame?

Judgement helper:
  _has_path(has_path) → bool

SUCCESS:
  nav_has_path is True

FAILURE:
  nav_has_path is False

Observability:
  Emits 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Nav_HasPath(XyzL1ConditionNode):
    """SUCCESS if /latched/nav_has_path is True."""

    LEVEL        = 'L1'
    BB_READS     = [BB.NAV_HAS_PATH]
    BB_WRITES    = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {}

    def __init__(self, name: str = 'L1_Nav_HasPath',
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
        self._bb.register_key(key=BB.NAV_HAS_PATH_KEY, access=Access.READ)

    def _has_path(self, has_path: bool) -> bool:
        """Return True when the planner reported a navigable path this frame."""
        return bool(has_path)

    def update(self) -> Status:
        has_path = self._bb.nav_has_path
        passed   = self._has_path(has_path)
        status   = self.status_from_bool(passed)

        inputs = {'nav_has_path': has_path}
        reason = 'path available' if passed else 'no path'

        self.emit_decision(inputs=inputs, status=status, reason=reason)
        return status
