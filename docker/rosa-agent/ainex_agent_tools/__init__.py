"""
ainex_agent_tools — read-only ROS tool layer for the Ainex humanoid robot.

Phase 1: read-only diagnostics and state inspection.
Write operations are stubbed and return a "disabled" message.
"""

from ainex_agent_tools.tools.health import get_robot_health
from ainex_agent_tools.tools.behavior import get_current_behavior
from ainex_agent_tools.tools.walking import get_walking_state
from ainex_agent_tools.tools.detections import get_latest_detections
from ainex_agent_tools.tools.logs import read_recent_ros_logs, read_last_run_summary
from ainex_agent_tools.tools.disabled import stop_current_behavior, stand_safe

# Exported tool list consumed by ainex_agent.py
AINEX_TOOLS = [
    get_robot_health,
    get_current_behavior,
    get_walking_state,
    get_latest_detections,
    read_recent_ros_logs,
    read_last_run_summary,
    stop_current_behavior,   # always returns disabled message
    stand_safe,              # always returns disabled message
]
