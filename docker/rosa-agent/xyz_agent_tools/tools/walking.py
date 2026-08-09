"""
get_walking_state — query the current walking controller state.

ROS interfaces used (all READ):
  - /walking/state      (GetWalkingState.srv  →  bool state, string message)
  - /walking/param      (GetWalkingParam.srv  →  WalkingParam msg)
  - /walking/offset     (GetWalkingOffset.srv →  WalkingOffset msg)
  - /walking/set_param  (WalkingParam topic   →  latest commanded params, best-effort)
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
            f"  period_time:          {p.period_time} ms\n"
            f"  dsp_ratio:            {p.dsp_ratio}\n"
            f"  step_fb_ratio:        {p.step_fb_ratio}\n"
            f"  x_move_amplitude:     {p.x_move_amplitude} m\n"
            f"  y_move_amplitude:     {p.y_move_amplitude} m\n"
            f"  z_move_amplitude:     {p.z_move_amplitude} m  (step height)\n"
            f"  angle_move_amplitude: {p.angle_move_amplitude} °\n"
            f"  y_swap_amplitude:     {p.y_swap_amplitude} m\n"
            f"  z_swap_amplitude:     {p.z_swap_amplitude} m\n"
            f"  arm_swing_gain:       {p.arm_swing_gain}\n"
            f"  move_aim_on:          {p.move_aim_on}\n"
            f"Init pose offsets:\n"
            f"  init_x_offset:        {p.init_x_offset} m\n"
            f"  init_y_offset:        {p.init_y_offset} m\n"
            f"  init_z_offset:        {p.init_z_offset} m\n"
            f"  init_roll_offset:     {p.init_roll_offset} °\n"
            f"  init_pitch_offset:    {p.init_pitch_offset} °\n"
            f"  init_yaw_offset:      {p.init_yaw_offset} °\n"
            f"  hip_pitch_offset:     {p.hip_pitch_offset} °\n"
            f"  pelvis_offset:        {p.pelvis_offset} °"
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
        o = resp.parameters
        results.append(
            f"Walking offset (trim, per direction/speed):\n"
            f"  low_fwd={o.low_speed_forward_offset:.3f}  low_bwd={o.low_speed_backward_offset:.3f}"
            f"  low_left={o.low_speed_move_left_offset:.3f}  low_right={o.low_speed_move_right_offset:.3f}\n"
            f"  med_fwd={o.medium_speed_forward_offset:.3f}  med_bwd={o.medium_speed_backward_offset:.3f}"
            f"  med_left={o.medium_speed_move_left_offset:.3f}  med_right={o.medium_speed_move_right_offset:.3f}\n"
            f"  hi_fwd={o.high_speed_forward_offset:.3f}   hi_bwd={o.high_speed_backward_offset:.3f}"
            f"  hi_left={o.high_speed_move_left_offset:.3f}   hi_right={o.high_speed_move_right_offset:.3f}"
        )
    except rospy.ROSException:
        results.append("Walking offset: service /walking/offset unavailable")
    except Exception as e:
        results.append(f"Walking offset: error — {e}")

    # ── Latest set_param command (topic snapshot) ─────────────────────────────
    try:
        from ainex_interfaces.msg import WalkingParam
        msg = rospy.wait_for_message("/walking/set_param", WalkingParam, timeout=0.5)
        results.append(
            f"Latest /walking/set_param command:\n"
            f"  init_x_offset:        {msg.init_x_offset} m\n"
            f"  init_y_offset:        {msg.init_y_offset} m\n"
            f"  init_z_offset:        {msg.init_z_offset} m\n"
            f"  init_roll_offset:     {msg.init_roll_offset} °\n"
            f"  init_pitch_offset:    {msg.init_pitch_offset} °\n"
            f"  init_yaw_offset:      {msg.init_yaw_offset} °\n"
            f"  hip_pitch_offset:     {msg.hip_pitch_offset} °\n"
            f"  pelvis_offset:        {msg.pelvis_offset} °\n"
            f"  period_time:          {msg.period_time} ms\n"
            f"  z_move_amplitude:     {msg.z_move_amplitude} m\n"
            f"  y_swap_amplitude:     {msg.y_swap_amplitude} m"
        )
    except rospy.ROSException:
        results.append("Latest set_param: no recent command on /walking/set_param (topic not latched — only captured if published within 0.5 s)")
    except Exception as e:
        results.append(f"Latest set_param: error — {e}")

    return "\n".join(results) if results else "No walking state data available."
