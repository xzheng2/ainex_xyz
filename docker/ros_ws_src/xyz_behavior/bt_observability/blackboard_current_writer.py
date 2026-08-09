#!/usr/bin/env python3
"""
BlackboardCurrentWriter: atomic current-state snapshot of the full py_trees blackboard.

Writes/refreshes a single file:
    xyz_behavior/log/bb_current.json

This is a current-state mirror, not a history log.  The file is overwritten
atomically on every call to write_current() using write-temp + fsync + os.replace.

Ownership: project/app node entry point (bt_node.py) only.
Adapters, L1/L2 nodes, and bb_ros_bridge must never write this file.
"""
import json
import os
import time

import py_trees
import rospy


class BlackboardCurrentWriter:
    """Atomically refreshes bb_current.json after each BT tick."""

    SCHEMA_VERSION = 1

    def __init__(self, output_path: str):
        self._output_path = output_path
        self._tmp_path = output_path + '.tmp'

    # ── public API ────────────────────────────────────────────────────────────

    def write_current(self, tick_id: int, root_status: str) -> None:
        """
        Snapshot the full blackboard and atomically refresh output_path.

        Parameters
        ----------
        tick_id     : tick counter of the tick that just completed (matches JSONL)
        root_status : str(tree.root.status) from the tick that just completed
        """
        try:
            storage = py_trees.blackboard.Blackboard.storage or {}
            bb_snapshot = {key: self._safe_value(val) for key, val in storage.items()}

            payload = {
                'schema_version': self.SCHEMA_VERSION,
                'tick_id':        tick_id,
                'ts':             time.time(),
                'root_status':    root_status,
                'blackboard':     bb_snapshot,
            }

            with open(self._tmp_path, 'w', encoding='utf-8') as f:
                json.dump(payload, f, indent=2)
                f.flush()
                os.fsync(f.fileno())

            os.replace(self._tmp_path, self._output_path)

        except Exception as exc:  # noqa: BLE001
            rospy.logwarn('[BlackboardCurrentWriter] write failed: %s', exc)

    # ── internal ──────────────────────────────────────────────────────────────

    @staticmethod
    def _safe_value(v):
        """Convert a BB value to a JSON-serializable object.

        Rules (same philosophy as BTDebugVisitor._safe_repr, but standalone):
        - None, bool, int, float, str  → returned as-is
        - ROS message (has __slots__)  → dict of slot → _safe_value(slot_value)
        - Anything else                → str(v)
        """
        if v is None:
            return None
        if isinstance(v, (bool, int, float, str)):
            return v
        if hasattr(v, '__slots__'):
            return {
                slot: BlackboardCurrentWriter._safe_value(getattr(v, slot, None))
                for slot in v.__slots__
            }
        return str(v)
