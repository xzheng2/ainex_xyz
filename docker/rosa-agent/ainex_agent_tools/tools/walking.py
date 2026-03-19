"""
get_walking_state — query the current walking controller state.

ROS interfaces used (all READ):
  - /walking/state  (GetWalkingState.srv  →  bool state, string message)
  - /walking/param  (GetWalkingParam.srv  →  WalkingParam msg)
"""

from langchain.agents import tool
import rospy


@tool
def get_walking_state(_input: str = "") -> str:
    """
    Get the current walking controller state of the Ainex robot.

    Calls the /walking/state service to check whether walking control is active,
    and reads current gait parameters (step height, period, etc.) from
    /walking/param. All operations are read-only.

    Use this tool when asked whether the robot is walking, its gait parameters,
    or the status of the walking controller.
    """
    results = []

    # ── Walking state service ─────────────────────────────────────────────────
    try:
        from ainex_interfaces.srv import GetWalkingState
        rospy.wait_for_service("/walking/state", timeout=3.0)
        get_state = rospy.ServiceProxy("/walking/state", GetWalkingState)
        resp = get_state()
        state_str = "ACTIVE" if resp.state else "INACTIVE"
        results.append(f"Walking controller: {state_str}")
        if resp.message:
            results.append(f"  Message: {resp.message}")
    except rospy.ROSException:
        results.append("Walking state: service /walking/state unavailable (kinematics node not running?)")
    except Exception as e:
        results.append(f"Walking state: error — {e}")

    # ── Walking parameters ────────────────────────────────────────────────────
    try:
        from ainex_interfaces.srv import GetWalkingParam
        rospy.wait_for_service("/walking/param", timeout=2.0)
        get_param = rospy.ServiceProxy("/walking/param", GetWalkingParam)
        resp = get_param()
        p = resp.parameters
        results.append(
            f"Gait parameters:\n"
            f"  period_time:      {p.period_time} ms\n"
            f"  dsp_ratio:        {p.dsp_ratio}\n"
            f"  step_fb_ratio:    {p.step_fb_ratio}\n"
            f"  x_move_amplitude: {p.x_move_amplitude} mm\n"
            f"  y_move_amplitude: {p.y_move_amplitude} mm\n"
            f"  z_move_amplitude: {p.z_move_amplitude} mm\n"
            f"  angle_move_amplitude: {p.angle_move_amplitude} °\n"
            f"  move_aim_on:      {p.move_aim_on}"
        )
    except rospy.ROSException:
        results.append("Gait parameters: service /walking/param unavailable")
    except Exception as e:
        results.append(f"Gait parameters: error — {e}")

    # ── Walking offset (trim) ─────────────────────────────────────────────────
    try:
        from ainex_interfaces.srv import GetWalkingOffset
        rospy.wait_for_service("/walking/offset", timeout=2.0)
        get_offset = rospy.ServiceProxy("/walking/offset", GetWalkingOffset)
        resp = get_offset()
        o = resp.offset
        results.append(
            f"Walking offset (trim):\n"
            f"  x={o.x:.3f}  y={o.y:.3f}  z={o.z:.3f}"
        )
    except Exception:
        pass  # Non-critical, skip silently

    return "\n".join(results) if results else "No walking state data available."
