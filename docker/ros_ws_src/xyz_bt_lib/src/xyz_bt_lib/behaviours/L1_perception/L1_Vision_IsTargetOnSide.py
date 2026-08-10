#!/usr/bin/env python3
"""L1_Vision_IsTargetOnSide — condition: tracked target is on the given side of frame.

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects)
    Written by ObjectDetectionAdapter / YoloObjectDetectionAdapter /
    AprilTagAdapter. Reads the entry for `target_id`; uses the 'error_x' and
    'lost_count' fields.

BB writes:
  none

Question judged:
  Is the tracked target horizontally left (or right) of the calibrated image
  centre by more than `deadband_px`?

Judgement helper:
  _is_on_side(obj) → (passed, reason)

Sign convention:
  `error_x` is written by the adapters as `obj.x - calibrated_centre_x`, so
  error_x < 0 means LEFT of centre and error_x > 0 means RIGHT of centre.
  Using error_x (rather than a raw pixel x compared against image_width/2) means
  this node inherits the adapter's `center_x_offset` calibration automatically,
  and shares one source of truth with L1_Vision_IsObjectPositioned.

SUCCESS:
  target_id present, lost_count == 0, and
    side='left'  → error_x < -deadband_px
    side='right' → error_x >  deadband_px

FAILURE:
  target_id absent, lost_count > 0, or the target is on the other side / inside
  the deadband.

CONFIG_DEFAULTS:
  target_id:   'object' — key to look up in the tracked_objects dict.
  side:        'left'   — which half to test: 'left' or 'right'.
  deadband_px: 0        — |error_x| must exceed this to count as on a side.
                          0 (default) = strict split at the calibrated centre.

Replaces L1_Vision_IsTargetOnLeft (removed Aug 2026), which read
`/perception/target_pixel_x` — a key no adapter or node ever wrote, so the node
raised KeyError the moment it was ticked. It also hardcoded a 'left' question
with no 'right' counterpart and defaulted to a 160 px frame width.

Observability:
  Emits optional 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB

_SIDES = ('left', 'right')


class L1_Vision_IsTargetOnSide(XyzL1ConditionNode):
    """SUCCESS if the tracked target's error_x puts it on the configured side."""

    LEVEL = 'L1'
    BB_READS = [BB.TRACKED_OBJECTS]
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'target_id':   'object',
        'side':        'left',
        'deadband_px': 0,
    }

    def __init__(self, name: str = 'L1_Vision_IsTargetOnSide',
                 target_id: str = 'object',
                 side: str = 'left',
                 deadband_px: float = 0,
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:           BT node name.
            target_id:      Key to look up in tracked_objects dict.
            side:           'left' or 'right' — which half of frame to test for.
            deadband_px:    |error_x| must exceed this to count as on a side;
                            0 = strict split at the calibrated centre.
            logger:         DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.
        """
        if side not in _SIDES:
            raise ValueError(
                "L1_Vision_IsTargetOnSide side must be one of {} (got {!r}) — "
                "the side is part of the question this node asks, so a typo "
                "must fail at construction, not silently never fire.".format(
                    _SIDES, side))
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._target_id   = target_id
        self._side        = side
        self._deadband_px = deadband_px
        self._bb          = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)

    def _is_on_side(self, obj) -> tuple:
        """Return (passed, reason). No BB reads/writes, ROS calls, or logger calls here."""
        if obj is None:
            return False, f'target {self._target_id!r} not in tracked_objects'
        if obj['lost_count'] > 0:
            return False, f'target {self._target_id!r} lost (lost_count={obj["lost_count"]})'
        error_x = obj['error_x']
        if self._side == 'left':
            passed = error_x < -self._deadband_px
        else:
            passed = error_x > self._deadband_px
        return passed, 'error_x={:.1f} deadband={} -> {}'.format(
            error_x, self._deadband_px,
            'on ' + self._side if passed else 'not on ' + self._side)

    def update(self) -> Status:
        tracked = self._bb.tracked_objects
        obj     = tracked.get(self._target_id)

        passed, reason = self._is_on_side(obj)
        status         = self.status_from_bool(passed)

        self.emit_decision(
            inputs={'target_id':   self._target_id,
                    'side':        self._side,
                    'error_x':     obj['error_x'] if obj else None,
                    'lost_count':  obj['lost_count'] if obj else None,
                    'deadband_px': self._deadband_px},
            status=status,
            reason=reason,
        )
        return status
