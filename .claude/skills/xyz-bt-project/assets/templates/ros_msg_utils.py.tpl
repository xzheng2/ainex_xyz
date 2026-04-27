#!/usr/bin/env python3
"""Standalone ROS message serialization utility.

Extracted from ROSCommTracer._msg_to_dict so callers do not need to import
the ros_comm_tracer module.  No class, no bt_observability dependency.
"""


def msg_to_dict(msg):
    """Recursively convert a ROS message to a plain dict.

    Handles nested messages and list/tuple fields (e.g. ainex_interfaces).
    Non-message objects are returned as-is; None is returned as None.
    """
    if msg is None:
        return None
    if hasattr(msg, "__slots__"):
        result = {}
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            if hasattr(value, "__slots__"):
                result[slot] = msg_to_dict(value)
            elif isinstance(value, (list, tuple)):
                result[slot] = [
                    msg_to_dict(v) if hasattr(v, "__slots__") else v
                    for v in value
                ]
            else:
                result[slot] = value
        return result
    return str(msg)
