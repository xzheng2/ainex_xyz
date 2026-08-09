#!/usr/bin/env python3
"""
DebugEventLogger: publish ROS topic + write two-tier JSONL storage.

Two-tier storage:
  last-run  — append-only during run (oldest-first on disk); reversed to
              newest-first in close() for ROSA reads after session ends.
  rolling   — fixed filename, last N ticks only, rewritten at most every
              flush_min_interval_secs seconds (default 2.0 s). For ROSA/LLM context window.

Rolling files are oldest-first (natural deque order). Lastrun files become
newest-first after a clean shutdown (close() call).
"""
import os
import json
import signal
import subprocess
import collections
import time
import atexit
import rospy
import rosgraph
from std_msgs.msg import String


class DebugEventLogger:
    """
    Usage:
        logger = DebugEventLogger(
            bt_lastrun_jsonl=".../log/bt_debug_lastrun.jsonl",
            comm_lastrun_jsonl=".../log/bt_ros_comm_debug_lastrun.jsonl",
            rolling_bt_jsonl=".../log/bt_debug_recent.jsonl",
            rolling_comm_jsonl=".../log/bt_ros_comm_debug_recent.jsonl",
            max_rolling_ticks=30,
            rosbag_topics=["/imu", "/object/pixel_coords"],
            rosbag_dir=".../log/rosbag",
        )
        logger.emit_bt({...})
        logger.emit_comm({...})
    """

    def __init__(
        self,
        bt_topic="/bt_debug",
        comm_topic="/bt_ros_comm_debug",
        bt_lastrun_jsonl="bt_debug_lastrun.jsonl",
        comm_lastrun_jsonl="bt_ros_comm_debug_lastrun.jsonl",
        rolling_bt_jsonl="bt_debug_recent.jsonl",
        rolling_comm_jsonl="bt_ros_comm_debug_recent.jsonl",
        max_rolling_ticks=30,
        flush_min_interval_secs=2.0,  # min seconds between rolling-file rewrites
        tick_id_getter=None,
        rosbag_topics=None,    # list[str] | None — None disables rosbag
        rosbag_dir=None,       # str | None — directory for rosbag files
    ):
        self._tick_id_getter = tick_id_getter or (lambda: None)
        self._flush_min_interval = flush_min_interval_secs
        self._last_flush_time    = 0.0   # 0 forces a flush on the very first tick
        self._closed = False
        atexit.register(self._atexit_close)
        self._bt_pub = rospy.Publisher(bt_topic, String, queue_size=100)
        self._comm_pub = rospy.Publisher(comm_topic, String, queue_size=100)

        # last-run: append-only during run; reversed to newest-first in close()
        self._lastrun_bt_path = bt_lastrun_jsonl
        self._lastrun_comm_path = comm_lastrun_jsonl
        open(bt_lastrun_jsonl, "w").close()        # truncate on startup
        open(comm_lastrun_jsonl, "w").close()
        self._bt_lastrun_f   = open(bt_lastrun_jsonl,  "a", buffering=1)
        self._comm_lastrun_f = open(comm_lastrun_jsonl, "a", buffering=1)

        # rolling: fixed path, atomically rewritten each tick
        self._rolling_bt_path = rolling_bt_jsonl
        self._rolling_comm_path = rolling_comm_jsonl
        self._bt_rolling = collections.deque(maxlen=max_rolling_ticks)
        self._comm_rolling = collections.deque(maxlen=max_rolling_ticks)

        # per-tick buffer (cleared by begin_tick each tick)
        self._bt_tick_buf = []
        self._comm_tick_buf = []

        # rosbag background recording
        self._rosbag_proc = None
        if rosbag_topics and rosbag_dir:
            self._start_rosbag(rosbag_topics, rosbag_dir)

    # --- Public API ---

    def emit_bt(self, payload: dict):
        """Emit a BT decision/lifecycle event."""
        payload.setdefault("tick_id", self._tick_id_getter())
        payload.setdefault("ts", rospy.Time.now().to_sec())
        line = json.dumps(payload, ensure_ascii=False)
        self._bt_pub.publish(line)
        self._bt_tick_buf.append(line)

    def emit_comm(self, payload: dict):
        """Emit a ROS communication event."""
        payload.setdefault("tick_id", self._tick_id_getter())
        payload.setdefault("ts", rospy.Time.now().to_sec())
        line = json.dumps(payload, ensure_ascii=False)
        self._comm_pub.publish(line)
        self._comm_tick_buf.append(line)

    def begin_tick(self, tick_id: int):
        """Called by BTDebugVisitor.on_tree_tick_start(). Clears BT buffer only.
        _comm_tick_buf is NOT cleared here — adapter events (ros_in, input_state)
        are emitted by write_snapshot() before tree.tick() calls begin_tick(), so
        clearing here would discard them. _comm_tick_buf is cleared at end of end_tick()."""
        self._bt_tick_buf = []

    def end_tick(self, tick_id: int):
        """Called by BTDebugVisitor.on_tree_tick_end(). Appends to files and flushes rolling."""
        if self._bt_tick_buf:
            self._bt_rolling.append(list(self._bt_tick_buf))
            for line in self._bt_tick_buf:
                self._bt_lastrun_f.write(line + "\n")
        if self._comm_tick_buf:
            self._comm_rolling.append(list(self._comm_tick_buf))
            for line in self._comm_tick_buf:
                self._comm_lastrun_f.write(line + "\n")
        now = time.time()
        if now - self._last_flush_time >= self._flush_min_interval:
            self._flush_rolling()
            self._last_flush_time = now
        self._comm_tick_buf = []

    def _atexit_close(self):
        """atexit handler — runs on any Python exit (Ctrl+C, unhandled exception).
        No-op if close() was already called explicitly by the main loop."""
        if not self._closed:
            self.close()

    def close(self):
        """Flush rolling files, close append files (reverse to newest-first), then stop rosbag."""
        if self._closed:
            return
        self._closed = True
        self._flush_rolling()   # capture any ticks buffered since last periodic flush
        for f in (self._bt_lastrun_f, self._comm_lastrun_f):
            try:
                f.close()
            except Exception:
                pass
        self._reverse_file(self._lastrun_bt_path)
        self._reverse_file(self._lastrun_comm_path)
        self._stop_rosbag()

    # --- rosbag recording ---

    def _start_rosbag(self, topics: list, rosbag_dir: str):
        """
        Start rosbag record in background, recording the given topic list.

        Split strategy:
          --split --duration 60   one .bag file per 60 s
          --max-splits 30         keep only the last 30 files (= 30 min rolling window)
                                  rosbag deletes the oldest split automatically

        File naming: rosbag_dir/bt_session_<timestamp>.bag
        Uses a separate process group so close() can kill the whole group via SIGINT.
        """
        os.makedirs(rosbag_dir, exist_ok=True)
        cmd = [
            "rosbag", "record",
            "--split",
            "--duration", "60",
            "--max-splits", "30",
            "--output-prefix", os.path.join(rosbag_dir, "bt_session"),
        ] + topics
        self._rosbag_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,   # independent process group for clean kill
        )
        rospy.loginfo("[DebugEventLogger] rosbag started (60s splits, 30min window): %s", topics)

    def _stop_rosbag(self):
        if self._rosbag_proc is not None:
            try:
                os.killpg(os.getpgid(self._rosbag_proc.pid), signal.SIGINT)
                self._rosbag_proc.wait(timeout=5)
            except Exception:
                pass
            self._rosbag_proc = None

    # --- ROS topology snapshot ---

    def snapshot_ros_topology(self, tick_id: int):
        """
        Collect a full ROS system topology snapshot and write to comm log.

        Content:
          node_details   : {node_name: {publications, subscriptions, services}}
          topic_details  : {topic_name: type_string}
          service_details: {service_name: [provider_nodes]}

        Captured once at startup (see BTDebugVisitor) in a daemon thread. Uses the
        ROS master XML-RPC API directly (two calls) rather than spawning ~131 CLI
        subprocesses — getSystemState() + getTopicTypes() return the whole graph
        in-process, so the snapshot completes in well under a second.

        Note: the master exposes no bulk service-type API, so service_details lists
        provider nodes rather than a type string (per-service `rosservice type` was
        the most expensive and least useful part of the old CLI approach).
        """
        snapshot = {
            "event":   "ros_topology_snapshot",
            "tick_id": tick_id,
            "ts":      rospy.Time.now().to_sec(),
        }

        try:
            master = rosgraph.Master("/bt_observability")
            # getSystemState() -> [publishers, subscribers, services]; each a list of
            # [name, [node, ...]] pairs. One call replaces rosnode list + rosnode info.
            publishers, subscribers, services = master.getSystemState()
            # getTopicTypes() -> [[topic, type], ...]. One call replaces rostopic type.
            topic_types = dict(master.getTopicTypes())
        except Exception as e:  # master unreachable / transient XML-RPC failure
            snapshot["error"] = "topology query failed: {}".format(e)
            self.emit_comm(snapshot)
            return

        # node_details: invert system state into per-node pub/sub/service lists
        node_details = {}

        def _node(name):
            return node_details.setdefault(
                name, {"publications": [], "subscriptions": [], "services": []}
            )

        for topic, nodes in publishers:
            for n in nodes:
                _node(n)["publications"].append(topic)
        for topic, nodes in subscribers:
            for n in nodes:
                _node(n)["subscriptions"].append(topic)
        for svc, nodes in services:
            for n in nodes:
                _node(n)["services"].append(svc)
        snapshot["node_details"] = node_details

        # topic_details: {topic: type}
        snapshot["topic_details"] = topic_types

        # service_details: {service: [provider_nodes]} (no bulk service-type API)
        snapshot["service_details"] = {svc: nodes for svc, nodes in services}

        self.emit_comm(snapshot)

    # --- Internal ---

    def _flush_rolling(self):
        # Write oldest-first (natural deque order) — ROSA tools resolve "latest"
        # via max(tick_id) so file order does not affect correctness.
        self._atomic_write(
            self._rolling_bt_path,
            [line for tick in self._bt_rolling for line in tick],
        )
        self._atomic_write(
            self._rolling_comm_path,
            [line for tick in self._comm_rolling for line in tick],
        )

    def _reverse_file(self, path):
        """Read all lines, reverse order, rewrite atomically (newest-first)."""
        try:
            with open(path, "r") as f:
                lines = [l.rstrip("\n") for l in f if l.strip()]
        except (FileNotFoundError, IOError):
            return
        if not lines:
            return
        lines.reverse()
        self._atomic_write(path, lines)

    @staticmethod
    def _atomic_write(path, lines):
        """Write to tmp then os.replace — prevents ROSA reading a partial file."""
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            if lines:
                f.write("\n".join(lines) + "\n")
        os.replace(tmp, path)

