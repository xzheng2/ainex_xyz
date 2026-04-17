#!/usr/bin/env python3
# encoding: utf-8
"""LineDetectionAdapter — /object/pixel_coords → /latched/{line_data,last_line_x,camera_lost_count}

Subscribes to /object/pixel_coords, extracts the first 'line' object from each
message, and maintains three live state values under a shared lock:
  - line_data:         the latest line ObjectInfo, or None if lost
  - last_line_x:       x-coordinate when line was last detected (sticky)
  - camera_lost_count: consecutive camera frames without line detection (30 Hz)

Exposes a two-phase latch protocol for consistent per-tick snapshots:

  Phase 1 (call while holding the shared lock):
      snap = adapter.snapshot_and_reset()

  Phase 2 (call after releasing the lock, main thread only):
      adapter.write_snapshot(snap, tick_id)

Observability events emitted per tick:
  ros_in      — one per tick when ≥1 message arrived since last latch
  input_state — one per tick (always), records the values written to BB
"""
import time
import threading

import rospy
import py_trees
from ainex_interfaces.msg import ObjectsInfo


class LineDetectionAdapter:
    """Adapter: /object/pixel_coords → line state → /latched/* (per tick)."""

    def __init__(self, lock: threading.Lock, logger=None, tick_id_getter=None):
        """
        Args:
            lock:             Shared threading.Lock (same object as marathon_bt_node.lock).
            logger:           DebugEventLogger instance, or None for zero-cost no-op.
            tick_id_getter:   Callable returning the current tick_id int.
        """
        self._lock = lock
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)

        # ── Live (async) state — written only by _callback under self._lock ──
        self._live_line_data = None
        self._live_last_line_x = None
        self._live_camera_lost_count = 0
        self._received_count = 0        # messages since last snapshot_and_reset()

        # ── Latched BB client — written only by write_snapshot() (main thread) ──
        self._bb = py_trees.blackboard.Client(
            name="LineDetectionAdapter", namespace="/latched")
        self._bb.register_key(key="line_data",         access=py_trees.common.Access.WRITE)
        self._bb.register_key(key="last_line_x",       access=py_trees.common.Access.WRITE)
        self._bb.register_key(key="camera_lost_count", access=py_trees.common.Access.WRITE)
        # Initialise BB before first tick so BT nodes never hit KeyError.
        self._bb.line_data         = self._live_line_data
        self._bb.last_line_x       = self._live_last_line_x
        self._bb.camera_lost_count = self._live_camera_lost_count

        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self._callback)

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg):
        """Extract line object and update all live state under lock."""
        line_data = None
        for obj in msg.data:
            if obj.type == 'line':
                line_data = obj
                break

        with self._lock:
            self._received_count += 1
            self._live_line_data = line_data
            if line_data is not None:
                self._live_last_line_x = line_data.x
                self._live_camera_lost_count = 0
            else:
                # camera_lost_count accumulates per camera frame (≈30 Hz),
                # not per BT tick — intentional, matches original behaviour.
                self._live_camera_lost_count += 1

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def snapshot_and_reset(self) -> dict:
        """Return a snapshot of current live state and reset received_count.

        MUST be called while the caller holds self._lock (the shared lock).
        Caller example::

            with self.lock:
                imu_snap  = self._imu_adapter.snapshot_and_reset()
                line_snap = self._line_adapter.snapshot_and_reset()

        Returns:
            dict with keys 'line_data', 'last_line_x', 'camera_lost_count',
            'received_count'.
        """
        snap = {
            'line_data':         self._live_line_data,
            'last_line_x':       self._live_last_line_x,
            'camera_lost_count': self._live_camera_lost_count,
            'received_count':    self._received_count,
        }
        self._received_count = 0
        return snap

    def write_snapshot(self, snap: dict, tick_id: int) -> None:
        """Write pre-captured snapshot to BB and emit observability events.

        No lock needed — called only from the main thread after the shared
        lock has been released.

        Args:
            snap:    dict returned by snapshot_and_reset().
            tick_id: current BT tick counter.
        """
        # ros_in: only when messages arrived since the last latch
        if snap['received_count'] > 0 and self._logger is not None:
            self._logger.emit_comm({
                "event":          "ros_in",
                "tick_id":        tick_id,
                "ts":             time.time(),
                "source":         "/object/pixel_coords",
                "adapter":        "LineDetectionAdapter",
                "received_count": snap['received_count'],
            })

        # Write all three BB keys
        line_data = snap['line_data']
        self._bb.line_data         = line_data
        self._bb.last_line_x       = snap['last_line_x']
        self._bb.camera_lost_count = snap['camera_lost_count']

        # input_state: always once per tick — records what the BT tree will see
        if self._logger is not None:
            line_data_log = (
                {"x": line_data.x, "width": line_data.width}
                if line_data is not None else None
            )
            self._logger.emit_comm({
                "event":   "input_state",
                "tick_id": tick_id,
                "ts":      time.time(),
                "adapter": "LineDetectionAdapter",
                "bb_writes": {
                    "/latched/line_data":         line_data_log,
                    "/latched/last_line_x":       snap['last_line_x'],
                    "/latched/camera_lost_count": snap['camera_lost_count'],
                },
            })
