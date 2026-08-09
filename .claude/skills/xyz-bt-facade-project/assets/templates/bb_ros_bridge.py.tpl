#!/usr/bin/env python3
"""{{PROJECT_CLASS}} project-local blackboard bridge.

Mirrors {{PROJECT}} project-specific BB keys to ROS topics for ROSA.
Keys already in xyz_bt_lib.BB.ROSA_TOPIC_MAP (/bt/bb/latched/*) are handled
by BlackboardROSBridge — do NOT duplicate them here.

Add entries to _BB_TOPIC_MAP for keys unique to this project.
Topics published:  /bt/{{PROJECT}}/bb/<key_name>  (std_msgs/String JSON, latched, 10 Hz)
"""
import json
import rospy
import py_trees
from std_msgs.msg import String

# Map: absolute BB storage key → ROS topic path.
# Example: '/{{PROJECT}}/some_flag': '/bt/{{PROJECT}}/bb/some_flag',
_BB_TOPIC_MAP = {
}


class {{PROJECT_CLASS}}BBBridge:
    """Publish {{PROJECT}} project-local BB keys to /bt/{{PROJECT}}/bb/* topics."""

    def __init__(self):
        self._pubs = {
            key: rospy.Publisher(topic, String, queue_size=1, latch=True)
            for key, topic in _BB_TOPIC_MAP.items()
        }

    def start(self, rate_hz: float = 10) -> None:
        """Start publishing. No-op if _BB_TOPIC_MAP is empty."""
        if not _BB_TOPIC_MAP:
            return
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self._publish_all)

    def _publish_all(self, _event) -> None:
        storage = py_trees.blackboard.Blackboard.storage
        for key, pub in self._pubs.items():
            val = storage.get(key)
            serialized = val if isinstance(val, (str, int, float, bool, type(None))) else str(val)
            pub.publish(String(data=json.dumps(serialized)))
