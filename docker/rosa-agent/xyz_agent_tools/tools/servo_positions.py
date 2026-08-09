"""
get_servo_positions — read current encoder position of all 24 servos.

ROS interfaces used (all READ):
  - /ros_robot_controller/bus_servo/get_position  (GetBusServosPosition srv)
"""

from langchain.agents import tool
import rospy


_SERVO_SEGMENTS = [
    ("Legs",  list(range(1, 13))),
    ("Arms",  list(range(13, 23))),
    ("Head",  list(range(23, 25))),
]


@tool
def get_servo_positions(_input: str = "") -> str:
    """
    Read the current position of all 24 Ainex servos.

    Positions are reported in encoder counts (0–1000); neutral/centre = 500.
    Deviation from 500 indicates how far the joint is deflected from its
    default standing pose.

    Use this tool when:
      - Asked about a specific joint angle or servo position.
      - Comparing expected vs actual posture in detail (e.g., head pan/tilt,
        arm extension, knee bend).
      - Diagnosing asymmetric posture or unexpected joint deflection.
      - get_robot_health has confirmed torque is ON but motion looks wrong.
    """
    try:
        from ros_robot_controller.srv import GetBusServosPosition
        rospy.wait_for_service(
            "/ros_robot_controller/bus_servo/get_position", timeout=3.0
        )
        get_pos = rospy.ServiceProxy(
            "/ros_robot_controller/bus_servo/get_position", GetBusServosPosition
        )
        all_ids = list(range(1, 25))
        resp = get_pos(id=all_ids)
    except rospy.ROSException:
        return "Servo positions: service /ros_robot_controller/bus_servo/get_position unavailable"
    except Exception as e:
        return f"Servo positions: error — {e}"

    if not resp.success or not resp.position:
        return "Servo positions: service returned no data"

    pos_map = {p.id: p.position for p in resp.position}

    lines = ["Servo positions (counts 0–1000, neutral=500):"]
    for seg_name, ids in _SERVO_SEGMENTS:
        lines.append(f"  {seg_name}:")
        for sid in ids:
            tick = pos_map.get(sid)
            if tick is None:
                lines.append(f"    Servo {sid:2d}: no data")
            else:
                delta = tick - 500
                lines.append(f"    Servo {sid:2d}: {tick:4d}  ({delta:+d} from neutral)")
    return "\n".join(lines)
