#!/usr/bin/env python3
"""
BT observability tools for the ROSA agent.

Reads the 4 JSONL files written by ainex_behavior/bt_observability/:
  bt_debug_recent.jsonl          — rolling last-N ticks (BT decision layer)
  bt_ros_comm_debug_recent.jsonl — rolling last-N ticks (ROS comm layer)
  bt_debug_lastrun.jsonl         — full session (BT decision layer)
  bt_ros_comm_debug_lastrun.jsonl — full session (ROS comm layer)
"""
import os
import time
import json
import collections

from langchain.agents import tool

_BT_OBS_DIR = "/opt/ainex_bt_log"

# ── Summary instruction hook ──────────────────────────────────────────────────
# Injected into _summarize_bt_generic output when the lastrun branch is used.
_SUMMARY_INSTRUCTIONS: str = """\
Four JSONL files are maintained by the BT node at /opt/ainex_bt_log/:

  bt_debug_recent.jsonl          — BT decision layer, rolling last 30 ticks.
                                   Updated every tick while the BT node is live.
  bt_ros_comm_debug_recent.jsonl — ROS comm layer, rolling last 30 ticks.
                                   Paired with bt_debug_recent; same live cadence.

  bt_debug_lastrun.jsonl          — BT decision layer, full session from last run.
                                    Preserved after the node stops; not updated live.
  bt_ros_comm_debug_lastrun.jsonl — ROS comm layer, full session from last run.
                                    Paired with bt_debug_lastrun.

Reading rule:
  BT node IS running  → read the _recent pair  (live data, last 30 ticks)
  BT node NOT running → read the _lastrun pair  (complete post-run record)

All files are newest-first: LOWER tick_id = OLDER tick, HIGHER tick_id = NEWER tick.
tick_id is the BT iteration counter. ts is Unix timestamp (float).\
"""
# ─────────────────────────────────────────────────────────────────────────────


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _bt_is_running(stale_threshold_s: float = 10.0) -> bool:
    """Return True if the BT node is currently running.

    Heuristic: bt_debug_recent.jsonl is flushed every tick.
    If its mtime is within stale_threshold_s seconds, the node is live.
    """
    path = os.path.join(_BT_OBS_DIR, "bt_debug_recent.jsonl")
    try:
        mtime = os.path.getmtime(path)
        return (time.time() - mtime) < stale_threshold_s
    except OSError:
        return False


def _read_jsonl(path):
    """Read a JSONL file, return list of non-empty raw line strings.

    Returns [] (no exception) on FileNotFoundError or empty file.
    """
    try:
        with open(path, "r") as f:
            lines = [line.rstrip("\n") for line in f if line.strip()]
        return lines
    except FileNotFoundError:
        return []
    except Exception as e:
        return [json.dumps({"error": str(e), "path": path})]


def _summarize_bt_generic(lines, label):
    """Produce a structured summary of BT decision-layer JSONL lines.

    Completely dynamic — no hardcoded node names. Aggregates whatever
    node names appear in the data.

    Args:
        lines: list of raw JSON strings from bt_debug_*.jsonl
        label: section header string

    Returns:
        Formatted multi-line string, or "" if lines is empty.
    """
    if not lines:
        return f"=== {label} ===\n[no data — file empty or not found]"

    # Parse all lines (skip malformed)
    events = []
    for line in lines:
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            pass

    if not events:
        return f"=== {label} ===\n[no parseable events]"

    # ── Aggregate ────────────────────────────────────────────────────────────

    tick_starts = 0
    tick_end_statuses = collections.Counter()   # tree_tick_end status counts
    node_status = collections.defaultdict(lambda: collections.Counter())  # node→{status:count}
    last_decision = {}   # node → last decision event (inputs)
    first_ts = None
    last_ts = None

    for ev in events:
        ev_ts = ev.get("ts")
        if ev_ts is not None:
            if first_ts is None:
                first_ts = ev_ts
            last_ts = ev_ts

        ev_type = ev.get("event", "")

        if ev_type == "tree_tick_start":
            tick_starts += 1

        elif ev_type == "tree_tick_end":
            status = ev.get("status", "unknown")
            tick_end_statuses[status] += 1

        elif ev_type == "tick_end":
            node = ev.get("node", "unknown")
            status = ev.get("status", "unknown")
            node_status[node][status] += 1

        elif ev_type == "decision":
            node = ev.get("node", "unknown")
            last_decision[node] = ev

    # ── Format ───────────────────────────────────────────────────────────────

    out = []

    if _SUMMARY_INSTRUCTIONS:
        out.append(f"[Analysis focus: {_SUMMARY_INSTRUCTIONS}]")

    out.append(f"=== {label} ===")
    out.append(f"Ticks: {tick_starts}")

    if first_ts is not None and last_ts is not None and first_ts != last_ts:
        duration = last_ts - first_ts
        out.append(f"Duration: {duration:.1f}s")

    # Tree result distribution
    if tick_end_statuses:
        dist = ", ".join(
            f"{s.replace('Status.', '')}×{c}"
            for s, c in sorted(tick_end_statuses.items())
        )
        out.append(f"Tree results: {dist}")

    # Per-node execution counts
    if node_status:
        out.append("\nNode execution counts:")
        for node in sorted(node_status):
            counts = node_status[node]
            parts = []
            for status in ("Status.SUCCESS", "Status.FAILURE", "Status.RUNNING"):
                if counts[status]:
                    short = status.replace("Status.", "")
                    parts.append(f"{short}×{counts[status]}")
            out.append(f"  {node}: {', '.join(parts)}")

    # Last decision inputs per condition node
    if last_decision:
        out.append("\nLast condition inputs (highest tick_id = most recent):")
        for node in sorted(last_decision):
            ev = last_decision[node]
            inputs = ev.get("inputs", {})
            status = ev.get("status", "?").replace("Status.", "")
            out.append(f"  {node} [{status}]: {json.dumps(inputs)}")

    return "\n".join(out)


def _summarize_comm_generic(lines, label):
    """Produce a structured summary of ROS comm JSONL lines.

    Groups by comm_type (topic_publish / service_call / action_goal / method_call),
    then by target. Each entry shows: call count, emitting BT node(s), known peer
    ros_node, and the last seen payload.

    Event schema fields used:
      comm_type  : "topic_publish" | "service_call" | "action_goal" | "method_call"
      direction  : "publish" | "call" | "goal" | "internal"
      target     : ROS topic/service name, or Python method name for method_call
      ros_node   : peer ROS node name (set by ManagerProxy or ROSCommTracer)
      source     : Python call path, e.g. "gait_manager.set_step" (ManagerProxy only)
      node       : BT action node that triggered this comm (set via proxy_context)
      payload    : dict of parameters sent

    Args:
        lines: list of raw JSON strings from bt_ros_comm_debug_*.jsonl
        label: section header string

    Returns:
        Formatted multi-line string, or "" if lines is empty.
    """
    if not lines:
        return f"=== {label} ===\n[no data — file empty or not found]"

    events = []
    for line in lines:
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            pass

    if not events:
        return f"=== {label} ===\n[no parseable events]"

    # Group by (comm_type, target): count, last payload, emitting BT nodes, ros_node
    groups = collections.defaultdict(lambda: {
        "count": 0, "last_payload": {}, "bt_nodes": set(), "ros_node": None,
    })
    total = 0
    for ev in events:
        comm_type = ev.get("comm_type", "unknown")
        target    = ev.get("target", "unknown")
        if comm_type == "ros_comm_result":
            continue   # skip response records from summary
        key = (comm_type, target)
        groups[key]["count"] += 1
        groups[key]["last_payload"] = ev.get("payload") or {}
        bt_node = ev.get("node")
        if bt_node:
            groups[key]["bt_nodes"].add(bt_node)
        if groups[key]["ros_node"] is None:
            groups[key]["ros_node"] = ev.get("ros_node")
        total += 1

    out = [f"=== {label} ===", f"Total comm events: {total}"]

    _SECTION_HEADERS = {
        "topic_publish": "Topic publishes  [BT node publishes → ros_node subscribes]",
        "service_call":  "Service calls    [BT node calls → ros_node serves]",
        "action_goal":   "Action goals     [BT node sends goal → action server]",
        "method_call":   "Internal calls   [Python method — not a direct ROS interface]",
    }
    for ctype in ("topic_publish", "service_call", "action_goal", "method_call", "unknown"):
        entries = [(t, d) for (ct, t), d in groups.items() if ct == ctype]
        if not entries:
            continue
        header = _SECTION_HEADERS.get(ctype, ctype)
        out.append(f"\n{header}:")
        for target, data in sorted(entries, key=lambda x: -x[1]["count"]):
            ros_node  = data["ros_node"] or "?"
            bt_nodes  = ", ".join(sorted(data["bt_nodes"])) if data["bt_nodes"] else "?"
            payload   = json.dumps(data["last_payload"])
            out.append(
                f"  {target}"
                f"  ros_node={ros_node}"
                f"  {data['count']}×"
                f"  from=[{bt_nodes}]"
                f"  last: {payload}"
            )

    return "\n".join(out)


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------

@tool
def read_bt_obs(_input: str = "") -> str:
    """
    Read BT observability logs, auto-selecting the correct file pair based on
    whether the BT node is currently running.

    BT node IS running  → reads _recent pair (rolling last 30 ticks, raw JSONL)
    BT node NOT running → reads _lastrun pair (full session, structured summary)

    Running is detected by checking if bt_debug_recent.jsonl was modified
    within the last 10 seconds (BT node flushes every tick).

    == File descriptions ==
    bt_debug_recent.jsonl          — BT decision layer, rolling last 30 ticks (live)
    bt_ros_comm_debug_recent.jsonl — ROS comm layer, rolling last 30 ticks (live)
    bt_debug_lastrun.jsonl          — BT decision layer, full last session (post-run)
    bt_ros_comm_debug_lastrun.jsonl — ROS comm layer, full last session (post-run)

    == Data ordering (all files) ==
    Newest-first: LOWER tick_id = OLDER. HIGHER tick_id = NEWER. ts = Unix float.

    Use this tool for:
      - Which BT node is active / what conditions fired and with what inputs
      - What ROS comm the BT issued in recent or past ticks
      - Full session statistics after a run ends
    """
    running = _bt_is_running()

    if running:
        bt_lines   = _read_jsonl(os.path.join(_BT_OBS_DIR, "bt_debug_recent.jsonl"))
        comm_lines = _read_jsonl(os.path.join(_BT_OBS_DIR, "bt_ros_comm_debug_recent.jsonl"))
        header = "# SOURCE: _recent files (BT node is live). Newest-first. LOWER tick_id = OLDER.\n"
        sections = [
            "=== BT Decision Layer (recent) ===\n" +
                ("\n".join(bt_lines) if bt_lines else "[no data]"),
            "=== ROS Comm Layer (recent) ===\n" +
                ("\n".join(comm_lines) if comm_lines else "[no data]"),
        ]
        return header + "\n\n".join(sections)
    else:
        bt_lines   = list(reversed(_read_jsonl(
            os.path.join(_BT_OBS_DIR, "bt_debug_lastrun.jsonl"))))
        comm_lines = list(reversed(_read_jsonl(
            os.path.join(_BT_OBS_DIR, "bt_ros_comm_debug_lastrun.jsonl"))))
        header = "# SOURCE: _lastrun files (BT node not running). Full session summary.\n"
        sections = [
            _summarize_bt_generic(bt_lines,    "BT Last Run Analysis"),
            _summarize_comm_generic(comm_lines, "ROS Comm Last Run"),
        ]
        return header + "\n\n".join(s for s in sections if s)
