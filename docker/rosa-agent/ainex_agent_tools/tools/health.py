"""
get_robot_health — read battery voltage, servo status, and IMU tilt.

ROS interfaces used (all READ):
  - /ros_robot_controller/battery  (std_msgs/UInt16, mV)
  - /ros_robot_controller/bus_servo/get_state  (GetBusServoState srv)
  - /imu  (sensor_msgs/Imu, fused orientation from imu_complementary_filter)
"""

from langchain.agents import tool
import rospy


_SERVO_SEGMENTS = [
    ("Legs",  list(range(1, 13))),
    ("Arms",  list(range(13, 23))),
    ("Head",  list(range(23, 25))),
]


@tool
def get_robot_health(_input: str = "") -> str:
    """
    Get the current health status of the Ainex robot.

    Returns battery voltage (mV), all 24 servo temperatures (°C), voltages,
    torque enable state, and the current IMU roll/pitch angles.
    All operations are read-only.

    Use this tool when asked about robot health, battery level, servo temperatures,
    servo torque state, or whether the robot is in a normal operating state.
    """
    sections = []

    # ── Battery ──────────────────────────────────────────────────────────────
    try:
        from std_msgs.msg import UInt16
        battery_msg = rospy.wait_for_message(
            "/ros_robot_controller/battery", UInt16, timeout=3.0
        )
        voltage_mv = battery_msg.data
        voltage_v = voltage_mv / 1000.0
        status = "OK" if voltage_mv >= 10500 else "LOW"
        sections.append(f"Battery: {voltage_mv} mV ({voltage_v:.2f} V) [{status}]")
    except rospy.ROSException:
        sections.append("Battery: timeout (topic /ros_robot_controller/battery not publishing)")
    except Exception as e:
        sections.append(f"Battery: error — {e}")

    # ── Servo status (all 24) ────────────────────────────────────────────────
    try:
        from ros_robot_controller.srv import GetBusServoState
        from ros_robot_controller.msg import GetBusServoCmd
        rospy.wait_for_service("/ros_robot_controller/bus_servo/get_state", timeout=3.0)
        get_state = rospy.ServiceProxy(
            "/ros_robot_controller/bus_servo/get_state", GetBusServoState
        )
        servo_lines = []
        hot_servos = []
        for segment_name, ids in _SERVO_SEGMENTS:
            servo_lines.append(f"  {segment_name}:")
            for sid in ids:
                try:
                    cmd = GetBusServoCmd(
                        id=sid, get_temperature=1, get_voltage=1, get_torque_state=1
                    )
                    resp = get_state(cmd=[cmd])
                    if not resp.state or not resp.state[0].temperature:
                        servo_lines.append(f"    Servo {sid:2d}: no response")
                        continue
                    temp = resp.state[0].temperature[0]
                    sv = resp.state[0].voltage
                    torque = resp.state[0].enable_torque
                    line = f"    Servo {sid:2d}: {temp}°C"
                    if sv:
                        line += f"  {sv[0] / 1000.0:.2f}V"
                    line += f"  torque={'ON' if torque and torque[0] else 'off'}"
                    if temp >= 60:
                        line += " [HOT]"
                        hot_servos.append(sid)
                    servo_lines.append(line)
                except Exception:
                    servo_lines.append(f"    Servo {sid:2d}: no response")
        sections.append("Servo status:\n" + "\n".join(servo_lines))
        if hot_servos:
            sections.append(f"WARNING: Overheating servos: {hot_servos}")
    except rospy.ROSException:
        sections.append(
            "Servo state: service /ros_robot_controller/bus_servo/get_state unavailable"
        )
    except Exception as e:
        sections.append(f"Servo state: error — {e}")

    # ── IMU tilt ──────────────────────────────────────────────────────────────
    try:
        import math
        from sensor_msgs.msg import Imu
        # Use the fused /imu topic (imu_complementary_filter output) which provides
        # a real orientation quaternion. roll/pitch ≈ 0° when upright.
        imu_msg = rospy.wait_for_message("/imu", Imu, timeout=3.0)
        q = imu_msg.orientation
        # Quaternion → roll/pitch (ZYX / aerospace convention)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))
        upright = abs(roll - 90) < 30 and abs(pitch) < 30
        posture = "UPRIGHT" if upright else "TILTED/FALLEN"
        sections.append(f"IMU: roll={roll:.1f}° pitch={pitch:.1f}° [{posture}]")
    except rospy.ROSException:
        sections.append("IMU: timeout (topic /imu not publishing)")
    except Exception as e:
        sections.append(f"IMU: error — {e}")

    return "\n".join(sections) if sections else "No health data available."
