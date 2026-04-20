#!/usr/bin/env python3
# encoding: utf-8
"""LineDetectionAdapter — /object/pixel_coords → /latched/{line_data,last_line_x,
camera_lost_count,line_error_x,line_center_x,last_line_error_x}

Subscribes to /object/pixel_coords, extracts the first 'line' object from each
message, and maintains live state under a shared lock:
  - line_data:          the latest line ObjectInfo, or None if lost
  - last_line_x:        x-coordinate when line was last detected (sticky)
  - camera_lost_count:  consecutive camera frames without line detection (30 Hz)
  - line_error_x:       line_data.x - line_center_x  (None when lost)
  - line_center_x:      width/2 + center_x_offset    (None when lost)
  - last_line_error_x:  sticky signed error (last seen); used by FindLine nodes
                        for direction decisions instead of raw last_line_x

center_x_offset is read from ainex_bt_edu/config/line_perception.yaml at startup.

Exposes a two-phase latch protocol for consistent per-tick snapshots:

  Phase 1 (call while holding the shared lock):
      snap = adapter.snapshot_and_reset()

  Phase 2 (call after releasing the lock, main thread only):
      adapter.write_snapshot(snap, tick_id)

Observability events emitted per tick:
  ros_in      — one per tick when ≥1 message arrived since last latch
  input_state — one per tick (always), records the values written to BB
"""
import os
import time
import threading

import rospy
import rospkg
import yaml
import py_trees
from ainex_interfaces.msg import ObjectsInfo
from ainex_bt_edu.blackboard_keys import BB


def _load_line_perception_config():
    try:
        pkg_path = rospkg.RosPack().get_path('ainex_bt_edu')
        cfg_path = os.path.join(pkg_path, 'config', 'line_perception.yaml')
        with open(cfg_path) as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        rospy.logwarn('[LineDetectionAdapter] failed to load line_perception.yaml: %s'
                      ' — using center_x_offset=0', e)
        return {}


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

        # ── Calibration (loaded once at startup) ──────────────────────────────
        _cfg = _load_line_perception_config()
        self._center_x_offset = float(_cfg.get('center_x_offset', 0))
        rospy.loginfo('[LineDetectionAdapter] center_x_offset=%.1f', self._center_x_offset)

        # ── Live (async) state — written only by _callback under self._lock ──
        self._live_line_data          = None
        self._live_last_line_x        = None
        self._live_camera_lost_count  = 0
        self._live_line_error_x       = None   # per-frame (None when lost)
        self._live_line_center_x      = None   # per-frame (None when lost)
        self._live_last_line_error_x  = None   # sticky signed error (last seen)
        self._received_count          = 0      # messages since last snapshot_and_reset()

        # ── Latched BB client — written only by write_snapshot() (main thread) ──
        self._bb = py_trees.blackboard.Client(
            name="LineDetectionAdapter", namespace=BB.LATCHED_NS)
        self._bb.register_key(key=BB.LINE_DATA_KEY,          access=py_trees.common.Access.WRITE)
        self._bb.register_key(key=BB.LAST_LINE_X_KEY,        access=py_trees.common.Access.WRITE)
        self._bb.register_key(key=BB.CAMERA_LOST_COUNT_KEY,  access=py_trees.common.Access.WRITE)
        self._bb.register_key(key=BB.LINE_ERROR_X_KEY,       access=py_trees.common.Access.WRITE)
        self._bb.register_key(key=BB.LINE_CENTER_X_KEY,      access=py_trees.common.Access.WRITE)
        self._bb.register_key(key=BB.LAST_LINE_ERROR_X_KEY,  access=py_trees.common.Access.WRITE)
        # Initialise BB before first tick so BT nodes never hit KeyError.
        self._bb.line_data          = None
        self._bb.last_line_x        = None
        self._bb.camera_lost_count  = 0
        self._bb.line_error_x       = None
        self._bb.line_center_x      = None
        self._bb.last_line_error_x  = None

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
                # Compute calibrated error
                line_center_x = line_data.width / 2.0 + self._center_x_offset
                line_error_x  = line_data.x - line_center_x
                self._live_line_center_x     = line_center_x
                self._live_line_error_x      = line_error_x
                self._live_last_line_error_x = line_error_x  # sticky
            else:
                # camera_lost_count accumulates per camera frame (≈30 Hz),
                # not per BT tick — intentional, matches original behaviour.
                self._live_camera_lost_count += 1
                self._live_line_center_x = None
                self._live_line_error_x  = None
                # _live_last_line_error_x stays (sticky)

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
            'line_error_x', 'line_center_x', 'last_line_error_x',
            'received_count'.
        """
        snap = {
            'line_data':          self._live_line_data,
            'last_line_x':        self._live_last_line_x,
            'camera_lost_count':  self._live_camera_lost_count,
            'line_error_x':       self._live_line_error_x,
            'line_center_x':      self._live_line_center_x,
            'last_line_error_x':  self._live_last_line_error_x,
            'received_count':     self._received_count,
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

        # Write all BB keys
        line_data = snap['line_data']
        self._bb.line_data          = line_data
        self._bb.last_line_x        = snap['last_line_x']
        self._bb.camera_lost_count  = snap['camera_lost_count']
        self._bb.line_error_x       = snap['line_error_x']
        self._bb.line_center_x      = snap['line_center_x']
        self._bb.last_line_error_x  = snap['last_line_error_x']

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
                    BB.LINE_DATA:          line_data_log,
                    BB.LAST_LINE_X:        snap['last_line_x'],
                    BB.CAMERA_LOST_COUNT:  snap['camera_lost_count'],
                    BB.LINE_ERROR_X:       snap['line_error_x'],
                    BB.LINE_CENTER_X:      snap['line_center_x'],
                    BB.LAST_LINE_ERROR_X:  snap['last_line_error_x'],
                },
            })
