#!/usr/bin/env python3
# encoding: utf-8
"""{{CLASS_NAME}} — {{ROS_TOPIC}} → {{DESCRIPTION}}

BB keys written (all under /latched/):
  {{BB_KEY_LIST}}

Two-phase latch protocol (per BT tick):

  Phase 1 — call while holding the shared lock:
      snap = adapter.snapshot_and_reset()

  Phase 2 — call after releasing the lock (main thread only):
      adapter.write_snapshot(snap, tick_id)

Phase 1 of ALL adapters must be called inside the same `with lock:` block so
the BT tree sees a frozen, consistent view of all sensor inputs per tick.

Observability events emitted:
  ros_in      — once per tick when ≥1 message arrived since last latch
  input_state — once per tick (always), records the value written to BB
"""
import time
import threading

import rospy
import py_trees
{{MSG_TYPE_IMPORT}}
from ainex_bt_edu.blackboard_keys import BB


class {{CLASS_NAME}}:
    """Adapter: {{ROS_TOPIC}} → {{DESCRIPTION}}"""

    def __init__(self, lock: threading.Lock, logger=None, tick_id_getter=None):
        """
        Args:
            lock:           Shared threading.Lock (same as bt_node.lock).
            logger:         DebugEventLogger instance, or None for zero-cost no-op.
            tick_id_getter: Callable returning the current tick_id int.
        """
        self._lock = lock
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)

        # ── Live (async) state — written only by _callback under self._lock ──
        {{BB_KEY_INITS}}
        self._received_count = 0    # messages since last snapshot_and_reset()

        # ── Latched BB client — written only by write_snapshot() (main thread) ──
        self._bb = py_trees.blackboard.Client(
            name="{{CLASS_NAME}}", namespace=BB.LATCHED_NS)
        {{BB_REGISTER_KEYS}}
        # Initialise BB before first tick so BT nodes never hit KeyError.
        {{BB_INIT_WRITES}}

        rospy.Subscriber('{{ROS_TOPIC}}', {{MSG_TYPE}}, self._callback)

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg: {{MSG_TYPE}}) -> None:
        """Extract data from message and update live state under lock."""
        # TODO: extract relevant data from msg
        # Example:
        #   value = msg.data
        with self._lock:
            # TODO: assign to self._live_<field> variables
            {{CALLBACK_LOGIC}}
            self._received_count += 1

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def snapshot_and_reset(self) -> dict:
        """Return a snapshot of current live state and reset received_count.

        MUST be called while the caller holds self._lock (the shared lock).
        """
        snap = {
            {{SNAP_FIELDS}}
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
                "source":         "{{ROS_TOPIC}}",
                "adapter":        "{{CLASS_NAME}}",
                "received_count": snap['received_count'],
            })

        # Write to BB (must happen after ros_in emit, before input_state emit)
        {{BB_WRITES}}

        # input_state: always once per tick — records what the BT tree will see
        if self._logger is not None:
            self._logger.emit_comm({
                "event":   "input_state",
                "tick_id": tick_id,
                "ts":      time.time(),
                "adapter": "{{CLASS_NAME}}",
                "bb_writes": {
                    {{INPUT_STATE_BB_WRITES}}
                },
            })
