#!/usr/bin/env python3
"""L1_Vision_IsObjectStill — condition: tracked target barely moved since last frame.

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects)
    Written by ObjectDetectionAdapter / YoloObjectDetectionAdapter /
    AprilTagAdapter as dict[target_id, stat_dict]. Reads the entry for
    `target_id`; uses the 'displacement' and 'lost_count' fields.

BB writes:
  none

Question judged:
  Is the target's most recent frame-to-frame centroid displacement below
  `threshold` pixels?

Judgement helper:
  _is_still(obj) → (passed, reason)

SUCCESS:
  target_id present, lost_count == 0, displacement is not None,
  and displacement < threshold

FAILURE:
  target_id absent, lost_count > 0 (stale), displacement is None
  (first sighting, re-acquired after a loss, or a YOLO multi-instance entry —
  motion is unknown, and unknown is treated as not-still), or
  displacement >= threshold

CONFIG_DEFAULTS:
  target_id: 'object' — key to look up in the tracked_objects dict.
  threshold: 2.0      — max frame-to-frame pixel displacement to count as 'still'.

Where the cross-frame memory lives (and why it is not here):
  Motion is a two-sample question, but an L1 node is a PURE PREDICATE and may not
  hold cross-tick state. So the previous position is remembered by the tracking
  adapter — which already carries `lost_count` across frames — and exposed as the
  single latched number `displacement`
  (see `xyz_bt_lib.core.base_adapter.carry_displacement`). This node only
  compares that number against a threshold.

  Before Aug 2026 this node kept its own `self._positions` list and a `frames`
  parameter. That violated the L1 contract, and the `frames` window is now a
  tree-layer concern: for "still for N consecutive ticks", wrap this node in
  `xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator(required_ticks=N)`.
  For "treat brief jitter as still", wrap it in
  `xyz_bt_lib.core.hysteresis.HysteresisDecorator` instead.

  Note the sampling rate changed with that move, for the better: displacement is
  now measured between CAMERA FRAMES (adapter rate) rather than between BT ticks,
  so motion that used to hide between two ticks is now seen.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Vision_IsObjectStill(XyzL1ConditionNode):
    """SUCCESS if the tracked target's latest frame-to-frame displacement < threshold."""

    LEVEL = 'L1'
    BB_READS = [BB.TRACKED_OBJECTS]
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'target_id': 'object',
        'threshold': 2.0,
    }

    def __init__(self, name: str = 'L1_Vision_IsObjectStill',
                 target_id: str = 'object',
                 threshold: float = 2.0,
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:           BT node name.
            target_id:      Key to look up in tracked_objects dict (default 'object').
            threshold:      Max frame-to-frame pixel displacement to count as 'still'.
            logger:         DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._target_id = target_id
        self._threshold = threshold
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)

    def _is_still(self, obj) -> tuple:
        """Return (passed, reason). No BB reads/writes, ROS calls, or logger calls here.

        Unknown displacement (None) is treated as NOT still: a dwell/stillness gate
        is a confirmation mechanism, so "cannot tell" must fail safe.
        """
        if obj is None:
            return False, f'target {self._target_id!r} not in tracked_objects'
        if obj['lost_count'] > 0:
            return False, f'target {self._target_id!r} lost (lost_count={obj["lost_count"]})'
        disp = obj.get('displacement')
        if disp is None:
            return False, 'displacement unknown (first sighting or re-acquired)'
        return disp < self._threshold, f'displacement={disp:.2f}/{self._threshold:.2f}'

    def update(self) -> Status:
        tracked = self._bb.tracked_objects
        obj     = tracked.get(self._target_id)

        passed, reason = self._is_still(obj)
        status         = self.status_from_bool(passed)

        self.emit_decision(
            inputs={'target_id':    self._target_id,
                    'displacement': obj.get('displacement') if obj else None,
                    'lost_count':   obj['lost_count'] if obj else None,
                    'threshold':    self._threshold},
            status=status,
            reason=reason,
        )
        return status
