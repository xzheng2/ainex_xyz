#!/usr/bin/env python3
"""L1 Condition: is the named target currently detected?

BB reads:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects)

BB writes:
  none

Question judged:
  Is the specified target_id present in tracked_objects with lost_count <= lost_count_threshold
  (i.e. detected within the last lost_count_threshold+1 camera frames)?

Judgement helper:
  _is_detected(tracked, target_id) → bool

SUCCESS:
  target_id key exists in tracked_objects AND tracked_objects[target_id]['lost_count'] <= lost_count_threshold

FAILURE:
  target_id key absent OR lost_count > lost_count_threshold

CONFIG_DEFAULTS:
  target_id:             'object'  — key to look up in the tracked_objects dict
  lost_count_threshold:  0         — max consecutive missed frames still considered detected (0 = strict: current frame only)

Observability:
  Emits 'decision' via self.emit_decision(). Never emits comm events.
"""
from py_trees.common import Access, Status
from xyz_bt_lib.core.base_node import XyzL1ConditionNode
from xyz_bt_lib.blackboard.blackboard_keys import BB


class L1_Vision_IsObjectDetected(XyzL1ConditionNode):
    """SUCCESS if tracked_objects[target_id] exists with lost_count <= lost_count_threshold (default 0)."""

    LEVEL        = 'L1'
    BB_READS     = [BB.TRACKED_OBJECTS]
    BB_WRITES    = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'target_id':             'object',
        'lost_count_threshold':  0,
    }

    def __init__(self, name: str = 'L1_Vision_IsObjectDetected',
                 target_id: str = 'object',
                 lost_count_threshold: int = 0,
                 logger=None, tick_id_getter=None):
        """
        Args:
            name:                  BT node name.
            target_id:             Key to look up in tracked_objects dict.
            lost_count_threshold:  Max consecutive missed frames still considered detected.
                                   0 = strict: object must be present in the current frame.
                                   N = tolerant: object may have been missing for up to N frames.
            logger:                DebugEventLogger-compatible object, or None.
            tick_id_getter:        Callable returning current tick_id.
        """
        super().__init__(name, logger=logger, tick_id_getter=tick_id_getter)
        self._target_id = target_id
        self._lost_count_threshold = lost_count_threshold
        self._bb = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb = self.attach_blackboard_client(
            name=self.name, namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY, access=Access.READ)

    def _is_detected(self, tracked: dict) -> bool:
        """Return True when target_id is present with lost_count <= lost_count_threshold.

        No BB reads/writes, ROS calls, or logger calls here.
        """
        obj = tracked.get(self._target_id)
        return obj is not None and obj['lost_count'] <= self._lost_count_threshold

    def update(self) -> Status:
        tracked = self._bb.tracked_objects
        passed  = self._is_detected(tracked)
        status  = self.status_from_bool(passed)

        obj = tracked.get(self._target_id)
        inputs = {
            'target_id':    self._target_id,
            'in_dict':      self._target_id in tracked,
            'lost_count':   obj['lost_count'] if obj is not None else None,
        }
        if passed:
            reason = (f'target {self._target_id!r} detected'
                      f' (lost_count<={self._lost_count_threshold})')
        else:
            reason = f'target {self._target_id!r} not detected'

        self.emit_decision(inputs=inputs, status=status, reason=reason)
        return status
