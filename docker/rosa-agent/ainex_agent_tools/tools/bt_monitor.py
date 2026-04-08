"""
get_bt_status — read behavior tree state from py_trees_ros topics.

ROS interfaces used (all READ):
  - /marathon_bt/log/tree    (py_trees_msgs/BehaviourTree)
  - /marathon_bt/ascii/snapshot (std_msgs/String)
  - /bt/marathon/bb/*         (std_msgs/String, JSON)
"""
import json
from langchain.agents import tool
import rospy
from std_msgs.msg import String


_STATUS_NAMES = {1: 'INVALID', 2: 'RUNNING', 3: 'SUCCESS', 4: 'FAILURE'}

_BB_KEYS = ['tick_id', 'robot_state', 'line_data', 'last_line_x', 'camera_lost_count']


@tool
def get_bt_status(query: str = "") -> str:
    """Get the current state of the marathon behavior tree.

    Returns the ASCII tree structure with node statuses and current blackboard
    values (tick_id, robot_state, line_data, last_line_x, camera_lost_count).

    tick_id = BT iteration counter (increments once per tree tick at 15 Hz).
    camera_lost_count = consecutive camera frames with no line detection (30 Hz);
      resets to 0 on any detection. NOT the same as tick_id.

    Use when asked about: BT state, which behavior is active/ticking, what the
    marathon tree is doing, behavior tree status, tick_id, or node execution state.
    """
    sections = []

    # -- ASCII tree snapshot (human-readable, always available) --
    try:
        msg = rospy.wait_for_message(
            "/marathon_bt/ascii/snapshot", String, timeout=3.0)
        sections.append("Tree snapshot:\n" + msg.data)
    except rospy.ROSException:
        sections.append(
            "Tree: no snapshot available (marathon_bt not running or not publishing)")
    except Exception as e:
        sections.append("Tree: error — {}".format(e))

    # -- Structured tree from py_trees_msgs (if available) --
    try:
        from py_trees_msgs.msg import BehaviourTree as BTMsg
        bt_msg = rospy.wait_for_message(
            "/marathon_bt/log/tree", BTMsg, timeout=2.0)
        lines = []
        for b in bt_msg.behaviours:
            st = _STATUS_NAMES.get(b.status, '?')
            active = '*' if b.is_active else ' '
            lines.append("  {} [{:7s}] {}".format(active, st, b.name))
        sections.append("Node statuses:\n" + "\n".join(lines))
    except ImportError:
        pass  # py_trees_msgs not built in ROSA container; ASCII fallback above
    except rospy.ROSException:
        pass  # already reported via ASCII section
    except Exception as e:
        sections.append("Structured tree: error — {}".format(e))

    # -- Blackboard from bridge topics --
    bb_vals = {}
    for key in _BB_KEYS:
        try:
            msg = rospy.wait_for_message(
                "/bt/marathon/bb/{}".format(key), String, timeout=1.0)
            bb_vals[key] = json.loads(msg.data)
        except Exception:
            bb_vals[key] = '<unavailable>'
    sections.append("Blackboard:\n" + "\n".join(
        "  {}: {}".format(k, v) for k, v in bb_vals.items()))

    return "\n\n".join(sections) if sections else "No BT data available."
