#!/usr/bin/env python3
"""
ros_comm_tracer: ROS message serialization utility for bt_observability.
"""


def _msg_to_dict(msg):
    """Recursively convert ROS message to dict. Handles ainex_interfaces."""
    if msg is None:
        return None
    if hasattr(msg, "__slots__"):
        result = {}
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            if hasattr(value, "__slots__"):
                result[slot] = _msg_to_dict(value)
            elif isinstance(value, (list, tuple)):
                result[slot] = [
                    _msg_to_dict(v)
                    if hasattr(v, "__slots__") else v
                    for v in value
                ]
            else:
                result[slot] = value
        return result
    return str(msg)
