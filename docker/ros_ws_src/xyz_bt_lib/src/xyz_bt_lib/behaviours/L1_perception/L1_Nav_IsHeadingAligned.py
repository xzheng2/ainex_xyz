#!/usr/bin/env python3
"""L1_Nav_IsHeadingAligned — condition: the robot heading is within a front deadband.

BB reads:
  BB.NAV_IMU_HEADING  (/latched/nav_imu_heading)  — written each tick by DepthNavAdapter
                       (the imu_gui cal-6ax heading, deg; 0 at start / straight ahead)

BB writes:
  none

Question judged:
  Has the robot re-aligned to (roughly) straight ahead this frame — i.e. is the
  absolute heading deviation below threshold_deg? (Intended as a "back on course"
  gate, e.g. after steering around an obstacle.)

Judgement helper:
  _is_aligned(heading) → bool

SUCCESS:
  abs(nav_imu_heading) < threshold_deg

FAILURE:
  abs(nav_imu_heading) >= threshold_deg

CONFIG_DEFAULTS:
  threshold_deg:       10 — |heading| below this (deg) counts as aligned/front.

Observability:
  Emits 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Nav_IsHeadingAligned(XyzL1ConditionNode):
    """SUCCESS if abs(/latched/nav_imu_heading) < threshold_deg."""

    LEVEL        = 'L1'
    BB_READS     = [BB.NAV_IMU_HEADING]
    BB_WRITES    = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'threshold_deg':       10,
    }

    def __init__(self, name: str = 'L1_Nav_IsHeadingAligned',
                 threshold_deg: float = 10,
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:                 BT node name.
            threshold_deg:        Absolute heading deviation (deg) below which the
                                  robot counts as aligned/front. Default 10.
            logger:               DebugEventLogger-compatible object, or None.
            tick_id_getter:       Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._threshold_deg = threshold_deg
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.NAV_IMU_HEADING_KEY, access=Access.READ)

    def _is_aligned(self, heading: float) -> bool:
        """Return True when the absolute heading deviation is below threshold."""
        return abs(float(heading)) < self._threshold_deg

    def update(self) -> Status:
        heading = self._bb.nav_imu_heading
        passed  = self._is_aligned(heading)
        status  = self.status_from_bool(passed)

        inputs = {'nav_imu_heading': heading, 'threshold_deg': self._threshold_deg}
        reason = (f'heading front (|{heading:.1f}| < {self._threshold_deg})' if passed
                  else f'heading off (|{heading:.1f}| >= {self._threshold_deg})')

        self.emit_decision(inputs=inputs, status=status, reason=reason)
        return status
