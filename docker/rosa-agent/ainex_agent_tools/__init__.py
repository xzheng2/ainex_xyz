"""
ainex_agent_tools — read-only ROS tool layer for the Ainex humanoid robot.

Phase 1: read-only diagnostics and state inspection.
Write operations are stubbed and return a "disabled" message.
"""

from ainex_agent_tools.tools.health import get_robot_health
from ainex_agent_tools.tools.walking import get_walking_state
from ainex_agent_tools.tools.bt_monitor import get_bt_status
from ainex_agent_tools.tools.bt_tick_analysis import get_bt_tick_raw, analyze_bt_tick
from ainex_agent_tools.tools.disabled import stop_current_behavior, stand_safe

# Exported tool list consumed by ainex_agent.py
AINEX_TOOLS = [
    get_robot_health,
    get_walking_state,
    get_bt_status,
    analyze_bt_tick,
    get_bt_tick_raw,
    stop_current_behavior,
    stand_safe,
]
