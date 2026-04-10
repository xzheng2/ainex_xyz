#!/usr/bin/env python3
"""
ROSCommTracer: helper class for tracing ROS communication in BT leaf nodes.

Wraps:
  - topic publish
  - service call
  - action goal send
  - method call (fallback for direct publishes not covered by ManagerProxy)

Also provides:
  - ProxyContext: module-level singleton for BT node attribution in ManagerProxy
  - ManagerProxy: transparent proxy that maps manager method calls to their
    actual ROS interface events (comm_type, direction, target, ros_node)
"""
import rospy


# ---------------------------------------------------------------------------
# ProxyContext — shared node attribution for ManagerProxy
# ---------------------------------------------------------------------------

class ProxyContext:
    """Module-level shared mutable context for ManagerProxy node attribution.

    Each BT action node sets current_node = self.name at the start of update()
    so that all ManagerProxy calls within that update() are attributed to it.
    semantic_source is set by CommFacade to indicate the project-semantic command
    name (e.g. 'follow_line', 'search_line') for structured log attribution.
    """
    def __init__(self):
        self.current_node: str = "unknown"
        self.semantic_source: str = ""


proxy_context = ProxyContext()   # module-level singleton


# ---------------------------------------------------------------------------
# ManagerProxy — generic transparent proxy for ROS manager objects
# ---------------------------------------------------------------------------

class ManagerProxy:
    """Transparent proxy around a ROS manager (GaitManager, MotionManager, etc.).

    Intercepts methods listed in ros_map and emits structured ROS comm events
    (topic_publish / service_call) instead of opaque method_call events.
    Methods NOT in ros_map are forwarded to the real manager without logging.

    ros_map schema per entry:
        {
            "comm_type":  "topic_publish" | "service_call",
            "direction":  "publish"       | "call",
            "target":     "<topic or service name>",
            "ros_node":   "<peer ROS node name>",          # optional
            "payload_fn": lambda args, kwargs: {...},       # optional; default {}
        }

    Usage in marathon_bt_node.py:
        GAIT_ROS_MAP = {
            "disable":  {"comm_type": "service_call", "direction": "call",
                         "target": "/walking/command", "ros_node": "ainex_controller",
                         "payload_fn": lambda a, k: {"command": "disable"}},
            "set_step": {"comm_type": "topic_publish", "direction": "publish",
                         "target": "walking/set_param", "ros_node": "ainex_controller",
                         "payload_fn": lambda a, k: {"x": a[1] if len(a)>1 else None,
                                                     "yaw": a[3] if len(a)>3 else None}},
        }
        gait_proxy = ManagerProxy(gait_manager, obs_logger,
                                  lambda: tick_id, GAIT_ROS_MAP, "gait_manager")

    BT action nodes set proxy_context.current_node = self.name in update() so
    each logged event is attributed to the correct BT node.
    """

    def __init__(self, real_manager, logger, tick_id_getter,
                 ros_map: dict, proxy_name: str = "proxy"):
        object.__setattr__(self, '_real',    real_manager)
        object.__setattr__(self, '_logger',  logger)
        object.__setattr__(self, '_tick_id', tick_id_getter)
        object.__setattr__(self, '_map',     ros_map)
        object.__setattr__(self, '_name',    proxy_name)

    def __getattr__(self, name):
        real_method = getattr(object.__getattribute__(self, '_real'), name)
        ros_map     = object.__getattribute__(self, '_map')

        if name not in ros_map:
            return real_method   # transparent passthrough for unlisted methods

        ros_info   = ros_map[name]
        logger     = object.__getattribute__(self, '_logger')
        tick_id_fn = object.__getattribute__(self, '_tick_id')
        proxy_name = object.__getattribute__(self, '_name')

        def traced(*args, **kwargs):
            payload_fn = ros_info.get("payload_fn", lambda a, k: {})
            logger.emit_comm({
                "event":           "ros_out",
                "comm_type":       ros_info["comm_type"],
                "direction":       ros_info["direction"],
                "target":          ros_info["target"],
                "ros_node":        ros_info.get("ros_node"),
                "source":          "{}.{}".format(proxy_name, name),
                "payload":         payload_fn(args, kwargs),
                # new-format attribution fields
                "bt_node":         proxy_context.current_node,
                "semantic_source": proxy_context.semantic_source,
                # legacy field aliases (kept for transition period)
                "node":            proxy_context.current_node,
                "tick_id":         tick_id_fn(),
            })
            return real_method(*args, **kwargs)

        return traced

    def __setattr__(self, name, value):
        # Forward attribute sets to the real manager (e.g. state, is_walking)
        setattr(object.__getattribute__(self, '_real'), name, value)


# ---------------------------------------------------------------------------
# ROSCommTracer — per-node tracer for direct ROS publishes
# ---------------------------------------------------------------------------

class ROSCommTracer:
    """
    Inject into BT leaf nodes to trace direct ROS communications.

    Primarily used for topic publishes that go directly through a rospy.Publisher
    (e.g. buzzer) rather than through a manager proxy.

    Usage in node __init__:
        self._tracer = ROSCommTracer(
            logger=logger,
            node_name_getter=lambda: self.name,
            tick_id_getter=tick_id_getter,
        ) if logger else None

    Usage in update():
        self._tracer.publish(pub, msg, "/ros_robot_controller/set_buzzer",
                             topic_type="ros_robot_controller/BuzzerState",
                             reason="alert", ros_node="ros_robot_controller")
    """

    def __init__(self, logger, node_name_getter=None, tick_id_getter=None):
        self._logger = logger
        self._node_name = node_name_getter or (lambda: "unknown")
        self._tick_id = tick_id_getter or (lambda: -1)

    def _emit(self, payload: dict):
        payload["node"] = self._node_name()
        payload["tick_id"] = self._tick_id()
        self._logger.emit_comm(payload)

    # --- Public API ---

    def publish(self, pub, msg, topic_name, topic_type=None, reason=None, ros_node=None):
        """Record publish event, then actually publish."""
        self._emit({
            "event":       "ros_comm",
            "comm_type":   "topic_publish",
            "direction":   "publish",
            "target":      topic_name,
            "target_type": topic_type,
            "ros_node":    ros_node,
            "payload":     self._msg_to_dict(msg),
            "reason":      reason,
        })
        pub.publish(msg)

    def service_call(self, client, req, service_name, reason=None, ros_node=None):
        """Record request, call service, record response."""
        self._emit({
            "event":     "ros_comm",
            "comm_type": "service_call",
            "direction": "call",
            "target":    service_name,
            "ros_node":  ros_node,
            "payload":   self._msg_to_dict(req),
            "reason":    reason,
        })
        resp = client(req)
        self._emit({
            "event":     "ros_comm_result",
            "comm_type": "service_call",
            "direction": "call",
            "target":    service_name,
            "result":    self._msg_to_dict(resp),
        })
        return resp

    def send_goal(self, action_client, goal, action_name,
                  goal_id=None, reason=None, ros_node=None):
        """Record action goal, then send."""
        self._emit({
            "event":     "ros_comm",
            "comm_type": "action_goal",
            "direction": "goal",
            "target":    action_name,
            "ros_node":  ros_node,
            "goal_id":   goal_id,
            "payload":   self._msg_to_dict(goal),
            "reason":    reason,
        })
        action_client.send_goal(goal)

    def method_call(self, method_name, params=None, reason=None):
        """Record a direct Python method call (internal — not a ROS interface).

        Use only for cases not covered by ManagerProxy (e.g. when a proxy
        cannot be injected). Prefer ManagerProxy for manager objects.
        """
        self._emit({
            "event":     "ros_comm",
            "comm_type": "method_call",
            "direction": "internal",
            "target":    method_name,
            "payload":   params or {},
            "reason":    reason,
        })

    # --- Utilities ---

    @staticmethod
    def _msg_to_dict(msg):
        """Recursively convert ROS message to dict. Handles ainex_interfaces."""
        if msg is None:
            return None
        if hasattr(msg, "__slots__"):
            result = {}
            for slot in msg.__slots__:
                value = getattr(msg, slot)
                if hasattr(value, "__slots__"):
                    result[slot] = ROSCommTracer._msg_to_dict(value)
                elif isinstance(value, (list, tuple)):
                    result[slot] = [
                        ROSCommTracer._msg_to_dict(v)
                        if hasattr(v, "__slots__") else v
                        for v in value
                    ]
                else:
                    result[slot] = value
            return result
        return str(msg)
