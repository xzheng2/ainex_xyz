#!/usr/bin/env python3
"""Mirror marathon blackboard keys to ROS topics for ROSA agent."""
import json
import rospy
import py_trees
from std_msgs.msg import String

MARATHON_BB_TOPIC_MAP = {
    '/latched/robot_state':        '/bt/marathon/bb/robot_state',
    '/latched/line_data':          '/bt/marathon/bb/line_data',
    '/latched/last_line_x':        '/bt/marathon/bb/last_line_x',
    '/latched/camera_lost_count':  '/bt/marathon/bb/camera_lost_count',
    '/latched/line_error_x':       '/bt/marathon/bb/line_error_x',
    '/latched/line_center_x':      '/bt/marathon/bb/line_center_x',
    '/latched/last_line_error_x':  '/bt/marathon/bb/last_line_error_x',
    '/tick_id':                    '/bt/marathon/bb/tick_id',
    '/head_pan_pos':               '/bt/marathon/bb/head_pan_pos',
    '/latched/detected_count':     '/bt/marathon/bb/detected_count',
}


class MarathonBBBridge:
    """Periodically publish blackboard keys as JSON strings on ROS topics."""

    def __init__(self):
        self._pubs = {
            key: rospy.Publisher(topic, String, queue_size=1, latch=True)
            for key, topic in MARATHON_BB_TOPIC_MAP.items()
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
