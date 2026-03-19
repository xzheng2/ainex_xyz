"""
ROS log tools — read recent rosout messages and summarize saved log files.

ROS interfaces used (all READ):
  - /rosout_agg  (rosgraph_msgs/Log)  — aggregated log stream
  - /root/.ros/log/<session>/  — saved log files (mounted read-only from ainex)
"""

from langchain.agents import tool
import rospy
from collections import deque
import threading
import time
import os
import sys


# Level names matching rosgraph_msgs/Log constants
_LEVEL_NAMES = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}
_DEFAULT_COUNT = 20
_COLLECT_SECS = 3.0  # how long to collect messages


def _level_name(level: int) -> str:
    return _LEVEL_NAMES.get(level, f"LVL{level}")


@tool
def read_recent_ros_logs(
    count: int = _DEFAULT_COUNT,
    level: str = "INFO",
    node_filter: str = "",
) -> str:
    """
    Read recent ROS log messages from the /rosout_agg topic.

    Args:
        count: Maximum number of messages to return (default 20, max 100).
        level: Minimum log level to include: DEBUG, INFO, WARN, ERROR, FATAL
               (default INFO — excludes DEBUG spam).
        node_filter: Optional node name substring to filter by (e.g. "walking",
                     "marathon_bt"). Empty string means show all nodes.

    Returns the most recent matching log messages with timestamp, level, node,
    and message text. All operations are read-only.

    Use this tool when asked about recent errors, warnings, what the robot was
    doing, or to diagnose a problem from logs.
    """
    # Sanitize inputs
    count = min(max(1, int(count)), 100)
    level_map = {"DEBUG": 1, "INFO": 2, "WARN": 4, "ERROR": 8, "FATAL": 16}
    min_level = level_map.get(level.upper(), 2)
    node_filter = str(node_filter).strip().lower()

    collected: deque = deque(maxlen=count * 3)  # over-collect, then filter
    lock = threading.Lock()
    done = threading.Event()

    def callback(msg):
        with lock:
            collected.append(msg)

    try:
        from rosgraph_msgs.msg import Log
        sub = rospy.Subscriber("/rosout_agg", Log, callback, queue_size=200)
        time.sleep(_COLLECT_SECS)
        sub.unregister()
    except Exception as e:
        return f"Failed to subscribe to /rosout_agg: {e}"

    # Filter and format
    with lock:
        msgs = list(collected)

    filtered = [
        m for m in msgs
        if m.level >= min_level
        and (not node_filter or node_filter in m.name.lower())
    ]

    # Keep the most recent `count` after filtering
    filtered = filtered[-count:]

    if not filtered:
        return (
            f"No log messages at level>={level} "
            + (f"from nodes matching '{node_filter}'" if node_filter else "")
            + f" in the last {_COLLECT_SECS:.0f}s."
        )

    lines = []
    for m in filtered:
        ts = f"{m.header.stamp.secs}.{m.header.stamp.nsecs // 1_000_000:03d}"
        lvl = _level_name(m.level)
        node = m.name
        text = m.msg.replace("\n", " ")
        lines.append(f"[{ts}] {lvl:5s} {node}: {text}")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Last-run log summary tool
# ---------------------------------------------------------------------------

_SUMMARIZER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               '..', '..')  # /opt/rosa-agent
_LOG_BASE = "/root/.ros/log"
_CONFIG_PATH = os.path.join(_SUMMARIZER_DIR, "config", "expected_nodes.yaml")


def _resolve_latest_log_dir() -> str:
    """Find newest log session directory by mtime, ignoring the 'latest' symlink."""
    if not os.path.isdir(_LOG_BASE):
        return ""
    dirs = [
        os.path.join(_LOG_BASE, d)
        for d in os.listdir(_LOG_BASE)
        if d != "latest" and os.path.isdir(os.path.join(_LOG_BASE, d))
    ]
    if not dirs:
        return ""
    return max(dirs, key=os.path.getmtime) + "/"


@tool
def read_last_run_summary(launch_name: str = "") -> str:
    """
    Summarize the saved ROS log files from the last roscore session.

    Reads all .log files from the ainex container's most recent ROS session
    (found by mtime under /root/.ros/log/) and produces a condensed Markdown
    report including:
    - Critical issues (errors, missing nodes, process deaths)
    - Node lifecycle timeline (deduplicated)
    - Repetitive event counts (ROSA polling, camera restarts collapsed)
    - Log messages grouped by level/node
    - Active topics and services at session end

    Args:
        launch_name: Optional launch file name to check expected nodes against
                     (e.g. "marathon_bringup"). If empty, auto-detected from
                     roslaunch logs.

    Returns a Markdown report suitable for LLM analysis. Typically 5-15KB
    (~2-5K tokens), much smaller than the raw logs (~100K+ tokens).

    Use this tool when asked about what happened in the last run, to diagnose
    problems from a previous session, or to understand the robot's recent
    operational history.
    """
    log_dir = _resolve_latest_log_dir()
    if not log_dir:
        return (
            "No ROS log session directory found under /root/.ros/log/. "
            "This means either roscore has not been started since the last "
            "container restart, or the log mount is not configured. "
            "The log directory is mounted read-only from the ainex container."
        )

    log_files = [f for f in os.listdir(log_dir) if f.endswith('.log')]
    if not log_files:
        return f"Log directory {log_dir} exists but contains no .log files."

    # Import the summarizer
    sys.path.insert(0, _SUMMARIZER_DIR)
    try:
        from summarize_ros_logs import generate_report
    except ImportError as e:
        return f"Failed to import summarize_ros_logs: {e}"
    finally:
        sys.path.pop(0)

    launch = launch_name.strip() if launch_name else None
    config = _CONFIG_PATH if os.path.exists(_CONFIG_PATH) else None

    try:
        report = generate_report(log_dir, launch, config)
        return report
    except Exception as e:
        return f"Error generating log summary: {e}"
