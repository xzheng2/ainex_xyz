"""
get_bt_status — read behavior tree state from py_trees_ros topics.

Auto-detects the active xyz_behavior BT project by scanning for
/{project}/ascii/snapshot topics. All xyz_behavior projects publish this
topic (latched) from TreeROSPublisher.

ROS interfaces used (all READ):
  - /{project}/ascii/snapshot   (std_msgs/String, latched)
  - /{project}/log/tree          (py_trees_msgs/BehaviourTree, latched)
  - /bt/bb/latched/*             (std_msgs/String, JSON, 10 Hz) — shared keys
  - /bt/{project}/bb/*           (std_msgs/String, JSON, 10 Hz) — project keys
"""
import json
from langchain.agents import tool
import rospy
from std_msgs.msg import String


_STATUS_NAMES = {1: 'INVALID', 2: 'RUNNING', 3: 'SUCCESS', 4: 'FAILURE'}


def _detect_bt_project():
    """Scan ROS topics for an active xyz_behavior BT project.

    Returns project name (node name) or None.
    All xyz_behavior projects publish /{project}/ascii/snapshot (latched).
    Topic path has exactly 3 parts: project / ascii / snapshot.
    """
    try:
        from rosgraph import Master
        for topic, msg_type in Master('/rosa_detect').getPublishedTopics('/'):
            if topic.endswith('/ascii/snapshot') and 'String' in msg_type:
                parts = topic.strip('/').split('/')
                if len(parts) == 3:          # /{project}/ascii/snapshot
                    return parts[0]          # the project/BT node name
    except Exception:
        pass
    return None


def _get_bb_topics(project):
    """Discover all active BB mirror topics for the running project.

    Returns list of (topic_path, short_key_name).
    Shared keys at /bt/bb/latched/*; project keys at /bt/{project}/bb/*.
    """
    try:
        from rosgraph import Master
        all_topics = dict(Master('/rosa_detect').getPublishedTopics('/'))
        shared = sorted(
            (t, t.split('/')[-1]) for t in all_topics
            if t.startswith('/bt/bb/latched/')
        )
        project_specific = sorted(
            (t, t.split('/')[-1]) for t in all_topics
            if t.startswith('/bt/{}/bb/'.format(project))
        )
        return shared + project_specific
    except Exception:
        return []


@tool
def get_bt_status(query: str = "") -> str:
    """Get the current state of the active xyz_behavior behavior tree.

    Auto-detects which BT project is running by scanning for
    /{project}/ascii/snapshot topics. Returns the ASCII tree structure,
    node statuses, and all available blackboard mirror values.

    tick_id = BT iteration counter (increments once per tree tick).
    camera_lost_count = consecutive camera frames with no line detection;
      resets to 0 on any detection. NOT the same as tick_id.

    Use when asked about: BT state, which behavior is active/ticking,
    what the tree is doing, behavior tree status, tick_id, or node
    execution state.
    """
    sections = []

    # -- Auto-detect active project --
    project = _detect_bt_project()
    if project is None:
        return (
            "No active xyz_behavior BT project detected. "
            "No /{project}/ascii/snapshot topic found. "
            "Check that the BT node is running (e.g. roslaunch xyz_behavior <proj>.launch)."
        )

    # -- tick_id FIRST (read up front so it survives session-log truncation) --
    # The session logger truncates each tool observation to 2000 chars; the ASCII tree
    # below can exceed that, so the tick_id must lead the output to stay correlatable.
    tick_topic = "/bt/bb/latched/tick_id"
    try:
        msg = rospy.wait_for_message(tick_topic, String, timeout=1.0)
        tick_val = json.loads(msg.data)
    except Exception:
        tick_val = "unavailable"
    sections.append("tick_id: {}".format(tick_val))

    sections.append("Active BT project: {}".format(project))

    # -- ASCII tree snapshot (human-readable, always available) --
    ascii_topic = "/{}/ascii/snapshot".format(project)
    try:
        msg = rospy.wait_for_message(ascii_topic, String, timeout=3.0)
        sections.append("Tree snapshot:\n" + msg.data)
    except rospy.ROSException:
        sections.append(
            "Tree: no snapshot available ({} not publishing)".format(ascii_topic))
    except Exception as e:
        sections.append("Tree: error — {}".format(e))

    # -- Structured tree from py_trees_msgs (if available) --
    tree_topic = "/{}/log/tree".format(project)
    try:
        from py_trees_msgs.msg import BehaviourTree as BTMsg
        bt_msg = rospy.wait_for_message(tree_topic, BTMsg, timeout=2.0)
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

    # -- Blackboard from bridge topics (shared + project-specific) --
    bb_topics = _get_bb_topics(project)
    if bb_topics:
        bb_vals = {}
        for topic_path, key in bb_topics:
            try:
                msg = rospy.wait_for_message(topic_path, String, timeout=1.0)
                bb_vals[key] = json.loads(msg.data)
            except Exception:
                bb_vals[key] = '<unavailable>'
        sections.append("Blackboard:\n" + "\n".join(
            "  {}: {}".format(k, v) for k, v in bb_vals.items()))
    else:
        sections.append("Blackboard: no BB mirror topics found for project '{}'".format(project))

    return "\n\n".join(sections) if sections else "No BT data available."
