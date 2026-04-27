#!/usr/bin/env python3
"""BlackboardROSBridge -- mirrors BB variables to ROS topics for ROSA."""
import json
import rospy
import py_trees
from std_msgs.msg import String
from xyz_bt_edu.blackboard_keys import BB


class BlackboardROSBridge:
    """
    Periodically publishes Blackboard key values as std_msgs/String (JSON)
    to /bt/bb/* topics. ROSA agent subscribes to these for real-time state.

    Usage:
        bridge = BlackboardROSBridge()
        bridge.start(rate_hz=10)
    """

    def __init__(self, key_to_topic_map: dict = None):
        self._map = key_to_topic_map or BB.ROSA_TOPIC_MAP
        self._pubs = {
            key: rospy.Publisher(topic, String, queue_size=1)
            for key, topic in self._map.items()
        }

    def start(self, rate_hz: float = 10):
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self._publish_all)

    def _publish_all(self, _):
        storage = py_trees.blackboard.Blackboard.storage
        for key, pub in self._pubs.items():
            bare = key
            val = storage.get(bare)
            if val is not None:
                serialized = (val if isinstance(val, (str, int, float, bool))
                              else str(val))
                pub.publish(String(data=json.dumps(serialized)))
