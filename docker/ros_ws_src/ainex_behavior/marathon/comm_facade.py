#!/usr/bin/env python3
"""Unified ROS outbound communication facade for marathon BT leaf nodes.

Replaces the scattered ``proxy_context.current_node`` boilerplate that action
nodes previously had to set before calling managers, and replaces the
ROSCommTracer pattern used in RecoverFromFall for buzzer publishing.

Design
------
- gait_manager / motion_manager should be ManagerProxy-wrapped so all
  gait/motion calls are automatically logged via the existing proxy.
  CommFacade only adds automatic proxy_context attribution (sets the BT node
  name before each call so ManagerProxy knows which leaf triggered it).
- Buzzer has no ManagerProxy, so CommFacade calls logger.emit_comm() directly
  then publishes, equivalent to what ROSCommTracer.publish() did before.
- If logger is None every method still works; logging is simply skipped.
"""
from bt_observability.ros_comm_tracer import proxy_context


class CommFacade:
    """Thin attribution + buzzer-logging wrapper around ROS managers."""

    def __init__(self, gait_manager, motion_manager, buzzer_pub,
                 logger=None, tick_id_getter=None):
        self._gait   = gait_manager
        self._motion = motion_manager
        self._buzzer = buzzer_pub
        self._logger = logger
        self._tick_id = tick_id_getter or (lambda: -1)

    # ── Internal ──────────────────────────────────────────────────────────

    def _attr(self, bt_node):
        """Set proxy_context so ManagerProxy attributes this call to bt_node."""
        proxy_context.current_node = bt_node or ''

    # ── Gait ─────────────────────────────────────────────────────────────

    def disable_gait(self, bt_node=None):
        self._attr(bt_node)
        self._gait.disable()

    def enable_gait(self, bt_node=None):
        self._attr(bt_node)
        self._gait.enable()

    def set_step(self, bt_node=None, *, dsp, x, y, yaw,
                 gait_param=None, arm_swap=None, step_num=0):
        self._attr(bt_node)
        self._gait.set_step(dsp, x, y, yaw, gait_param,
                            arm_swap=arm_swap, step_num=step_num)

    # ── Motion ───────────────────────────────────────────────────────────

    def run_action(self, action_name, bt_node=None):
        self._attr(bt_node)
        self._motion.run_action(action_name)

    def set_servos_position(self, duration_ms, positions, bt_node=None):
        self._attr(bt_node)
        self._motion.set_servos_position(duration_ms, positions)

    # ── Buzzer ───────────────────────────────────────────────────────────

    def publish_buzzer(self, buzzer_msg, bt_node=None, reason=None):
        if self._logger:
            self._logger.emit_comm({
                "event":           "ros_comm",
                "tick_id":         self._tick_id(),
                "bt_node":         bt_node or '',
                "semantic_source": "publish_buzzer",
                "comm_type":       "topic_publish",
                "direction":       "publish",
                "target":          "/ros_robot_controller/set_buzzer",
                "ros_node":        "ros_robot_controller",
                "payload": {
                    "freq":     getattr(buzzer_msg, 'freq',     None),
                    "on_time":  getattr(buzzer_msg, 'on_time',  None),
                    "off_time": getattr(buzzer_msg, 'off_time', None),
                    "repeat":   getattr(buzzer_msg, 'repeat',   None),
                },
            })
        self._buzzer.publish(buzzer_msg)
