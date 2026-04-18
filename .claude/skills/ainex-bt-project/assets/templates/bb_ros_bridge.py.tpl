#!/usr/bin/env python3
"""Mirror {{PROJECT}} blackboard keys to ROS topics for ROSA agent / debug tools.

Maintenance rule:
  Add an entry to {{PROJECT_UPPER}}_BB_TOPIC_MAP for every BB key that your
  input adapters write (or that the BT node writes directly, e.g. /tick_id).
  Keep this map in sync with PROJECT_INTERFACES in infra/infra_manifest.py.

Default topic namespace: /bt/{{PROJECT}}/bb/<key_name>
"""
import json
import rospy
import py_trees
from std_msgs.msg import String

# ── Blackboard → ROS topic map ────────────────────────────────────────────────
# Always present: tick_id (written by bt_node) and robot_state (ImuBalanceStateAdapter).
# Add project-specific BB keys below as your adapters are wired in.
{{PROJECT_UPPER}}_BB_TOPIC_MAP = {
    '/tick_id':              '/bt/{{PROJECT}}/bb/tick_id',
    '/latched/robot_state':  '/bt/{{PROJECT}}/bb/robot_state',
    # ── Add project-specific keys below ──────────────────────────────────
    # '/latched/line_data':         '/bt/{{PROJECT}}/bb/line_data',
    # '/latched/last_line_x':       '/bt/{{PROJECT}}/bb/last_line_x',
    # '/latched/camera_lost_count': '/bt/{{PROJECT}}/bb/camera_lost_count',
    # '/head_pan_pos':              '/bt/{{PROJECT}}/bb/head_pan_pos',
    # Add more as needed...
}


class {{PROJECT_CLASS}}BBBridge:
    """Periodically publish blackboard keys as JSON strings on ROS topics."""

    def __init__(self):
        self._pubs = {
            key: rospy.Publisher(topic, String, queue_size=1, latch=True)
            for key, topic in {{PROJECT_UPPER}}_BB_TOPIC_MAP.items()
        }
        self._timer = None

    def start(self, rate_hz=10):
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / rate_hz), self._publish_all)

    def _publish_all(self, _event):
        storage = py_trees.blackboard.Blackboard.storage
        for key, pub in self._pubs.items():
            val = storage.get(key)
            if isinstance(val, (str, int, float, bool, type(None))):
                serialized = val
            else:
                serialized = str(val)
            pub.publish(String(data=json.dumps(serialized)))
