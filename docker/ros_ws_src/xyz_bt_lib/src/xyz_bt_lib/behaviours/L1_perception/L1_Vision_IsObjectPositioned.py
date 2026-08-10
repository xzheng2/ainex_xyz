#!/usr/bin/env python3
"""L1 Condition: is the named target large enough, centred, and within range?

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects)

BB writes:
  none

Question judged:
  Is tracked_objects[target_id]['size'] >= min_size
  AND abs(tracked_objects[target_id]['error_x']) <= max_error_x
  AND (optionally) min_distance_mm <= tracked_objects[target_id]['depth_mm'] <= max_distance_mm?

Judgement helper:
  _is_positioned(tracked) → (passed, reason)

Distance gate (optional):
  max_distance_mm / min_distance_mm default to None, which disables the distance
  check entirely — behaviour is then identical to the original size + centring
  node, so every existing caller is unaffected. When a bound is set, the object's
  depth in millimetres (obj['depth_mm'], populated by the detection adapter when
  enable_depth=True) is compared against it. If a distance bound is set but
  depth_mm is missing/None, the object is treated as NOT positioned (distance
  cannot be confirmed).

SUCCESS:
  target_id present in tracked_objects with lost_count == 0,
  size >= min_size (skipped when size is None — see below),
  abs(error_x) <= max_error_x, and — when a distance bound is set —
  min_distance_mm <= depth_mm <= max_distance_mm.

Shapes without an area (size is None):
  A detected 'line' has no meaningful area, so the adapter writes size=None and
  this node SKIPS the min_size gate for it, still answering the centring (and
  optional distance) question. That is what lets one node serve both line and
  object targets. Note the deliberate asymmetry with the distance gate below:
  a missing depth FAILS, because a distance bound was explicitly configured and
  "cannot confirm" must not pass; a None size is not a missing measurement but a
  property that does not apply to that shape.

FAILURE:
  target_id absent, lost_count > 0, size < min_size, abs(error_x) > max_error_x,
  or depth_mm outside [min_distance_mm, max_distance_mm] (or missing while a bound is set).

CONFIG_DEFAULTS:
  target_id:       'object'  — key to look up in the tracked_objects dict
  min_size:        1000      — minimum acceptable size (pixels²; π*r² for circle, w*h for rect)
  max_error_x:     30        — maximum acceptable horizontal error from image centre (pixels)
  max_distance_mm: None      — maximum depth (mm); None = distance check disabled
  min_distance_mm: None      — minimum depth (mm); None = no lower bound

Consecutive-frame confirmation (dwell): this node stays a pure predicate. When a
tree needs "steadily positioned for N ticks", wrap it at the tree layer in
xyz_bt_lib.core.latched_dwell.LatchedDwellDecorator.

Observability:
  Emits 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Vision_IsObjectPositioned(XyzL1ConditionNode):
    """SUCCESS if target is detected, large enough, centred, and within range."""

    LEVEL        = 'L1'
    BB_READS     = [BB.TRACKED_OBJECTS]
    BB_WRITES    = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'target_id':          'object',
        'min_size':           1000,
        'max_error_x':        30,
        'max_distance_mm':    None,   # max depth (mm); None = distance check disabled
        'min_distance_mm':    None,   # min depth (mm); None = no lower bound
    }

    def __init__(self, name: str = 'L1_Vision_IsObjectPositioned',
                 logger=None, tick_id_getter=None,
                 target_id: str = 'object',
                 min_size: float = 1000,
                 max_error_x: float = 30,
                 max_distance_mm: float = None,
                 min_distance_mm: float = None):
        """
        Args:
            name:            BT node name.
            logger:          DebugEventLogger-compatible object, or None.
            tick_id_getter:  Callable returning current tick_id.
            target_id:       Key to look up in tracked_objects dict.
            min_size:        Minimum size threshold (pixels²).
            max_error_x:     Maximum horizontal error from calibrated centre (pixels).
            max_distance_mm: Maximum depth (mm). None disables the distance check
                             (default). Requires the detection adapter to run with
                             enable_depth=True so obj['depth_mm'] is populated.
            min_distance_mm: Minimum depth (mm). None means no lower bound.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._target_id       = target_id
        self._min_size        = min_size
        self._max_error_x     = max_error_x
        self._max_distance_mm = max_distance_mm
        self._min_distance_mm = min_distance_mm
        self._bb              = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)

    def _is_positioned(self, tracked: dict) -> tuple:
        """Return (passed, reason). No BB reads/writes, ROS calls, or logger calls here.

        Checks:
          1. target_id key present with lost_count == 0 (currently detected)
          2. size >= min_size
          3. abs(error_x) <= max_error_x
          4. distance gate (only when max_distance_mm or min_distance_mm is set):
             depth_mm present and min_distance_mm <= depth_mm <= max_distance_mm
        """
        obj = tracked.get(self._target_id)
        if obj is None:
            return False, f'target {self._target_id!r} not in tracked_objects'
        if obj['lost_count'] > 0:
            return False, f'target {self._target_id!r} lost (lost_count={obj["lost_count"]})'
        size     = obj['size']
        # size is None for shapes with no meaningful area (e.g. a detected line).
        # Skip the gate rather than fail: this node then still answers the
        # centring/distance question for those targets. Contrast the distance
        # gate, which FAILS on missing depth — there the bound was explicitly
        # requested, so "cannot confirm" must not pass. Here a None size means
        # the concept does not apply to this shape at all.
        size_ok  = True if size is None else size >= self._min_size
        error_ok = abs(obj['error_x'])  <= self._max_error_x
        dist_ok, dist_reason = self._distance_ok(obj)
        passed   = size_ok and error_ok and dist_ok
        size_txt = 'n/a' if size is None else f'{size:.0f}/{self._min_size:.0f}'
        reason   = (
            f"size={size_txt} "
            f"error_x={obj['error_x']:.1f}/{self._max_error_x:.1f}"
            f"{dist_reason}"
        )
        return passed, reason

    def _distance_ok(self, obj: dict) -> tuple:
        """Return (passed, reason_suffix) for the optional depth (mm) distance gate.

        When neither bound is set the gate is disabled → always passes with an
        empty reason suffix. When a bound is set but depth_mm is missing/None the
        object fails (distance cannot be confirmed).
        """
        if self._max_distance_mm is None and self._min_distance_mm is None:
            return True, ''
        depth = obj.get('depth_mm')
        if depth is None:
            return False, ' depth=None/gated'
        if self._max_distance_mm is not None and depth > self._max_distance_mm:
            return False, f' depth={depth:.0f}>{self._max_distance_mm:.0f}mm'
        if self._min_distance_mm is not None and depth < self._min_distance_mm:
            return False, f' depth={depth:.0f}<{self._min_distance_mm:.0f}mm'
        return True, f' depth={depth:.0f}mm'

    def update(self) -> Status:
        tracked          = self._bb.tracked_objects
        passed, reason   = self._is_positioned(tracked)
        status           = self.status_from_bool(passed)

        obj = tracked.get(self._target_id)
        inputs = {
            'target_id':       self._target_id,
            'size':            obj['size']    if obj else None,
            'error_x':         obj['error_x'] if obj else None,
            'lost_count':      obj['lost_count'] if obj else None,
            'depth_mm':        obj.get('depth_mm') if obj else None,
            'min_size':        self._min_size,
            'max_error_x':     self._max_error_x,
            'max_distance_mm': self._max_distance_mm,
            'min_distance_mm': self._min_distance_mm,
        }
        self.emit_decision(inputs=inputs, status=status, reason=reason)
        return status
