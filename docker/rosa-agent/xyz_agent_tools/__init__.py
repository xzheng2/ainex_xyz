"""
xyz_agent_tools — read-only ROS tool layer for the Ainex humanoid robot.

Phase 1: read-only diagnostics and state inspection.
Write operations are stubbed and return a "disabled" message.
"""

from xyz_agent_tools.tools.health import get_robot_health
from xyz_agent_tools.tools.walking import get_walking_state
from xyz_agent_tools.tools.servo_positions import get_servo_positions
from xyz_agent_tools.tools.bt_monitor import get_bt_status
from xyz_agent_tools.tools.bt_tick_analysis import get_bt_tick_raw, analyze_bt_tick
from xyz_agent_tools.tools.cross_tick_analysis import cross_tick_analysis
from xyz_agent_tools.tools.session_digest import session_digest
from xyz_agent_tools.tools.disabled import stop_current_behavior, stand_safe

# Exported tool list consumed by xyz_agent.py
XYZ_TOOLS = [
    get_robot_health,
    get_walking_state,
    get_servo_positions,
    get_bt_status,
    analyze_bt_tick,
    get_bt_tick_raw,
    cross_tick_analysis,
    session_digest,
    stop_current_behavior,
    stand_safe,
]
