#!/usr/bin/env python3
"""
BTDebugVisitor: py_trees visitor for BT decision-layer observability.

Records:
  - tree_tick_start / tree_tick_end       via pre/post tick handlers
  - per-node tick_end                     via run()
  - bb_write events per tick              via activity_stream flush
  - ros_topology_snapshot                 when root status changes (daemon thread)
"""
import threading
import py_trees


class BTDebugVisitor(py_trees.visitors.VisitorBase):

    def __init__(self, logger, tick_id_getter):
        super().__init__(full=False)   # full=False: only visit actually-ticked nodes
        self._logger = logger
        self._tick_id_getter = tick_id_getter
        self._last_root_status = None

        # Enable py_trees built-in blackboard activity stream.
        # Note: if already enabled (e.g. by marathon_bt_node), this call may be
        # a no-op depending on py_trees version; maximum_size=500 should exceed
        # any single-tick BB write count.
        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=500)

    # ── tree-level hooks ──────────────────────────────────────────────────────

    def on_tree_tick_start(self, tree):
        """Assigned to tree.pre_tick_handlers."""
        tick_id = self._tick_id_getter()
        self._logger.begin_tick(tick_id)
        self._logger.emit_bt({
            "event": "tree_tick_start",
            "tick_id": tick_id,
        })

    def on_tree_tick_end(self, tree):
        """Assigned to tree.post_tick_handlers."""
        tick_id = self._tick_id_getter()
        current_status = str(tree.root.status)

        # 1. tree_tick_end
        self._logger.emit_bt({
            "event": "tree_tick_end",
            "tick_id": tick_id,
            "status": current_status,
        })

        # 2. flush BB write events (all writes this tick)
        self._flush_blackboard_writes(tick_id)

        # 3. root status changed → topology snapshot in daemon thread (low frequency)
        #    Running async to avoid blocking the 15 Hz tick loop: rosnode info
        #    over all nodes can take 5-10 s synchronously.
        if current_status != self._last_root_status:
            t = threading.Thread(
                target=self._logger.snapshot_ros_topology,
                args=(tick_id,),
                daemon=True,
            )
            t.start()
            self._last_root_status = current_status

        # 4. clear activity stream for next tick
        py_trees.blackboard.Blackboard.activity_stream.clear()

        self._logger.end_tick(tick_id)

    # ── per-node hook ─────────────────────────────────────────────────────────

    def run(self, behaviour):
        """Called by py_trees after behaviour.update() completes."""
        self._logger.emit_bt({
            "event": "tick_end",
            "tick_id": self._tick_id_getter(),
            "node": behaviour.name,
            "type": type(behaviour).__name__,
            "status": str(behaviour.status),
        })

    # ── internal ──────────────────────────────────────────────────────────────

    def _flush_blackboard_writes(self, tick_id: int):
        """
        Read all WRITE activities from Blackboard this tick and emit bb_write events.

        Compression: subscriber callbacks run at 30-100 Hz against a 15 Hz tick,
        so many writes to the same key accumulate per tick.  For each key, group
        consecutive same-value writes into runs and emit only (first, last) per run.
        This preserves every value transition while suppressing mid-run duplicates.

        Example for /line_data: [A, B, null, null, null]
          → emits: A, B, null[first], null[last]   (4 lines instead of 5)

        Example for /camera_lost_count: [0, 0, 1, 2]
          → emits: 0[first], 0[last], 1, 2         (4 lines, first+last of 0-run)

        activity_type values:
          'WRITE'       — update to existing key
          'INITIALISED' — first-ever write to a key
        """
        stream = py_trees.blackboard.Blackboard.activity_stream
        if not stream or not stream.data:
            return

        # Group write activities per key (preserving insertion order, Python 3.7+)
        key_activities = {}
        for activity in stream.data:
            act_type = str(activity.activity_type)
            if act_type not in ("WRITE", "INITIALISED"):
                continue
            key_activities.setdefault(activity.key, []).append(activity)

        # For each key: collapse consecutive same-value runs to (first, last)
        for key, activities in key_activities.items():
            runs = []   # list of lists, each list = one same-value run
            for act in activities:
                val_key = repr(act.current_value)   # stable key; works for ROS msgs
                if runs and repr(runs[-1][-1].current_value) == val_key:
                    runs[-1].append(act)
                else:
                    runs.append([act])

            for run in runs:
                self._emit_bb_write(tick_id, run[0])
                if len(run) > 1:
                    self._emit_bb_write(tick_id, run[-1])

    def _emit_bb_write(self, tick_id: int, activity):
        self._logger.emit_bt({
            "event":   "bb_write",
            "tick_id": tick_id,
            "writer":  activity.client_name,
            "key":     activity.key,
            "value":   self._safe_repr(activity.current_value),
        })

    @staticmethod
    def _safe_repr(value):
        """Convert BB value to JSON-serializable object."""
        if value is None:
            return None
        if isinstance(value, (bool, int, float, str)):
            return value
        if hasattr(value, "__slots__"):          # ROS message object
            from bt_observability.ros_comm_tracer import ROSCommTracer
            return ROSCommTracer._msg_to_dict(value)
        return str(value)
