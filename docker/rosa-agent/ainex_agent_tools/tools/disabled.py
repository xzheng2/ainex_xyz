"""
Disabled write tools — Phase 1 read-only mode.

These tools are exposed to ROSA so the agent can explain why an operation
is not available, rather than silently failing.

To enable in a future phase:
  1. Set AINEX_WRITE_ENABLED=true in .env
  2. Remove the guard and implement the actual ROS call
  3. Update config/readonly.yaml
  4. Document the audit in this file
"""

import os
from langchain.agents import tool

_DISABLED_MSG = (
    "This operation is disabled in read-only mode. "
    "AINEX_WRITE_ENABLED is not set to 'true'. "
    "Write operations require explicit authorization and a container rebuild."
)


def _write_enabled() -> bool:
    return os.environ.get("AINEX_WRITE_ENABLED", "false").lower() == "true"


@tool
def stop_current_behavior(_input: str = "") -> str:
    """
    Stop the currently running behavior on the Ainex robot.

    NOTE: This tool is DISABLED in read-only mode (Phase 1).
    It would call /walking/command with 'stop' and halt the behavior tree node.

    Returns a disabled message until write access is explicitly authorized.
    """
    if _write_enabled():
        # Future implementation placeholder:
        # import rospy
        # from ainex_interfaces.srv import SetWalkingCommand
        # rospy.wait_for_service("/walking/command", timeout=3.0)
        # svc = rospy.ServiceProxy("/walking/command", SetWalkingCommand)
        # resp = svc("stop")
        # return f"Stop command sent. Result: {resp.result}"
        return "Write mode is enabled but stop_current_behavior is not yet implemented."
    return _DISABLED_MSG


@tool
def stand_safe(_input: str = "") -> str:
    """
    Command the Ainex robot to stand still in a safe upright posture.

    NOTE: This tool is DISABLED in read-only mode (Phase 1).
    It would call /walking/command with 'stand' and reset servo positions to
    the default standing configuration.

    Returns a disabled message until write access is explicitly authorized.
    """
    if _write_enabled():
        # Future implementation placeholder:
        # import rospy
        # from ainex_interfaces.srv import SetWalkingCommand
        # rospy.wait_for_service("/walking/command", timeout=3.0)
        # svc = rospy.ServiceProxy("/walking/command", SetWalkingCommand)
        # resp = svc("stand")
        # return f"Stand command sent. Result: {resp.result}"
        return "Write mode is enabled but stand_safe is not yet implemented."
    return _DISABLED_MSG
