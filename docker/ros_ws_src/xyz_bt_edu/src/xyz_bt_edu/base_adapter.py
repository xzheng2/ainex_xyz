#!/usr/bin/env python3
"""Base class for AiNex observable input adapters.

Input adapters are business-in components, not behaviour-tree nodes. They
subscribe to ROS input streams, maintain live state under a shared lock, and
expose a two-phase latch API used by the BT app loop:

1. ``snapshot_and_reset()`` while the caller holds the shared lock.
2. ``write_snapshot(snap, tick_id)`` after releasing the lock on the main thread.
"""
from abc import ABC, abstractmethod
import time

import py_trees

from xyz_bt_edu.blackboard_keys import BB


class XyzInputAdapter(ABC):
    """Common base for sensor/input adapters.

    Subclasses own ROS subscription setup and live-state conversion. This base
    only stores shared observability dependencies and provides standard
    ``ros_in`` / ``input_state`` emit helpers.
    """

    ROS_TOPIC = None
    ADAPTER_NAME = None

    BB_READS = []
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {}

    def __init__(self, lock, logger=None, tick_id_getter=None):
        self._lock = lock
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._received_count = 0

    @property
    def tick_id(self):
        """Return the current externally-owned BT tick id."""
        return self._tick_id_getter()

    @property
    def adapter_name(self):
        return self.ADAPTER_NAME or type(self).__name__

    def make_latched_bb_client(self, name=None):
        """Create a blackboard client for writing /latched/* keys."""
        return py_trees.blackboard.Client(
            name=name or self.adapter_name,
            namespace=BB.LATCHED_NS,
        )

    def emit_comm(self, payload: dict) -> None:
        """Emit a communication-layer event if a logger was injected."""
        if self._logger is None:
            return
        event = dict(payload)
        event.setdefault('tick_id', self.tick_id)
        event.setdefault('adapter', self.adapter_name)
        self._logger.emit_comm(event)

    def emit_ros_in(self, *, tick_id=None, received_count=None,
                    source=None, **extra) -> None:
        """Emit one ros_in event when messages arrived in this latch window."""
        count = self._received_count if received_count is None else received_count
        if count <= 0:
            return
        event = {
            'event': 'ros_in',
            'tick_id': self.tick_id if tick_id is None else tick_id,
            'ts': time.time(),
            'source': source or self.ROS_TOPIC,
            'adapter': self.adapter_name,
            'received_count': count,
        }
        event.update(extra)
        self.emit_comm(event)

    def emit_input_state(self, *, tick_id=None, bb_writes=None, **extra) -> None:
        """Emit the per-tick input_state event after writing the blackboard."""
        event = {
            'event': 'input_state',
            'tick_id': self.tick_id if tick_id is None else tick_id,
            'ts': time.time(),
            'adapter': self.adapter_name,
            'bb_writes': bb_writes or {},
        }
        event.update(extra)
        self.emit_comm(event)

    @abstractmethod
    def snapshot_and_reset(self) -> dict:
        """Return a live-state snapshot and reset per-latch counters."""
        raise NotImplementedError

    @abstractmethod
    def write_snapshot(self, snap: dict, tick_id: int) -> None:
        """Write a pre-captured snapshot to BB and emit business-in events."""
        raise NotImplementedError
