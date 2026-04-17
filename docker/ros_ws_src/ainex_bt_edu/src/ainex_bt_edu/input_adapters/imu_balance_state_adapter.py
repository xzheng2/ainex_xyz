#!/usr/bin/env python3
# encoding: utf-8
"""ImuBalanceStateAdapter — /imu → /latched/robot_state

Subscribes to /imu, runs fall-detection logic in the callback (updates live
robot_state under a shared lock), and exposes a two-phase latch protocol
for consistent per-tick blackboard snapshots:

  Phase 1 (call while holding the shared lock):
      snap = adapter.snapshot_and_reset()

  Phase 2 (call after releasing the lock, main thread only):
      adapter.write_snapshot(snap, tick_id)

Phase 1 of all adapters must be called inside the same `with lock:` block so
the BT tree always sees a frozen, consistent view of all sensor inputs for one
tick (no mid-snapshot async mutation between adapters).

Observability events emitted per tick:
  ros_in      — one per tick when ≥1 message arrived since last latch
  input_state — one per tick (always), records the value written to BB
"""
import math
import time
import threading

import rospy
import py_trees
from sensor_msgs.msg import Imu


class ImuBalanceStateAdapter:
    """Adapter: /imu → fall-detection logic → /latched/robot_state (per tick)."""

    FALL_COUNT_THRESHOLD = 100

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
        self._live_robot_state = 'stand'
        self._received_count = 0        # messages since last snapshot_and_reset()

        # ── Fall-detection accumulators — written only by _callback thread ──
        # Not shared with main thread, so no lock needed for these.
        self.count_lie = 0
        self.count_recline = 0

        # ── Latched BB client — written only by write_snapshot() (main thread) ──
        self._bb = py_trees.blackboard.Client(
            name="ImuBalanceStateAdapter", namespace="/latched")
        self._bb.register_key(key="robot_state", access=py_trees.common.Access.WRITE)
        # Initialise BB before first tick so BT nodes never hit KeyError.
        self._bb.robot_state = self._live_robot_state

        rospy.Subscriber('/imu', Imu, self._callback)

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg):
        """Update live robot_state based on IMU fall detection."""
        with self._lock:
            self._received_count += 1
            current_state = self._live_robot_state

        if current_state != 'stand':
            # Recovery sequence in progress — do not accumulate fall counts.
            return

        angle = abs(int(math.degrees(
            math.atan2(msg.linear_acceleration.y,
                       msg.linear_acceleration.z))))

        # count_lie and count_recline are only ever written from this callback
        # thread, so they do not need the lock.
        if angle < 30:
            self.count_lie += 1
        else:
            self.count_lie = 0

        if angle > 150:
            self.count_recline += 1
        else:
            self.count_recline = 0

        if self.count_lie > self.FALL_COUNT_THRESHOLD:
            self.count_lie = 0
            self.count_recline = 0
            with self._lock:
                self._live_robot_state = 'lie_to_stand'
            rospy.loginfo('[ImuAdapter] fall detected: lie_to_stand')
        elif self.count_recline > self.FALL_COUNT_THRESHOLD:
            self.count_lie = 0
            self.count_recline = 0
            with self._lock:
                self._live_robot_state = 'recline_to_stand'
            rospy.loginfo('[ImuAdapter] fall detected: recline_to_stand')

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def force_state(self, state: str) -> None:
        """Override live robot_state (called by RecoverFromFall's robot_state_setter hook).

        Writes 'stand' into the live store so the *next* pre-tick snapshot does
        not overwrite the latched BB with the stale pre-recovery IMU value.
        """
        with self._lock:
            self._live_robot_state = state

    def snapshot_and_reset(self) -> dict:
        """Return a snapshot of current live state and reset received_count.

        MUST be called while the caller holds self._lock (the shared lock).
        Caller example::

            with self.lock:
                imu_snap  = self._imu_adapter.snapshot_and_reset()
                line_snap = self._line_adapter.snapshot_and_reset()

        Returns:
            dict with keys 'robot_state' (str) and 'received_count' (int).
        """
        snap = {
            'robot_state':    self._live_robot_state,
            'received_count': self._received_count,
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
                "source":         "/imu",
                "adapter":        "ImuBalanceStateAdapter",
                "received_count": snap['received_count'],
            })

        # Write to BB (must happen after ros_in emit, before input_state emit)
        self._bb.robot_state = snap['robot_state']

        # input_state: always once per tick — records what the BT tree will see
        if self._logger is not None:
            self._logger.emit_comm({
                "event":   "input_state",
                "tick_id": tick_id,
                "ts":      time.time(),
                "adapter": "ImuBalanceStateAdapter",
                "bb_writes": {
                    "/latched/robot_state": snap['robot_state'],
                },
            })
