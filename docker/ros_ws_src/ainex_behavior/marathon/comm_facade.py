#!/usr/bin/env python3
"""Generic ROS Facade — unified ROS communication exit and final log outlet.

All outbound ROS communications from the marathon BT flow through this class.
Each method accepts bt_node and semantic_source for full attribution so that
every emitted log carries the new-format fields:
  bt_node, semantic_source, target, comm_type, direction, payload, tick_id.

Internal gait/motion calls are routed through ManagerProxy (which also logs);
CommFacade sets proxy_context.current_node + proxy_context.semantic_source
before each call so ManagerProxy log entries carry the same attribution.
Buzzer has no ManagerProxy so CommFacade logs it directly here.
"""
import time as _time
from bt_observability.ros_comm_tracer import proxy_context


class CommFacade:
    """Unified ROS communication exit for marathon BT leaf → semantic facade."""

    def __init__(self, gait_manager, motion_manager, buzzer_pub,
                 logger=None, tick_id_getter=None):
        self._gait   = gait_manager
        self._motion = motion_manager
        self._buzzer = buzzer_pub
        self._logger = logger
        self._tick_id = tick_id_getter or (lambda: -1)

    # ── Attribution helpers ───────────────────────────────────────────────

    def _attr(self, bt_node, semantic_source):
        """Set proxy_context so ManagerProxy attributes this call correctly."""
        proxy_context.current_node   = bt_node or ''
        proxy_context.semantic_source = semantic_source or ''

    def _reset_attr(self):
        proxy_context.current_node   = 'unknown'
        proxy_context.semantic_source = ''

    # ── Gait ─────────────────────────────────────────────────────────────

    def disable_gait(self, bt_node=None, semantic_source=None, tick_id=None):
        self._attr(bt_node, semantic_source or 'disable_gait')
        self._gait.disable()
        self._reset_attr()

    def enable_gait(self, bt_node=None, semantic_source=None, tick_id=None):
        self._attr(bt_node, semantic_source or 'enable_gait')
        self._gait.enable()
        self._reset_attr()

    def set_step(self, bt_node=None, semantic_source=None, tick_id=None, *,
                 dsp, x, y, yaw, gait_param=None, arm_swap=None, step_num=0):
        self._attr(bt_node, semantic_source or 'set_step')
        self._gait.set_step(dsp, x, y, yaw, gait_param,
                            arm_swap=arm_swap, step_num=step_num)
        self._reset_attr()

    # ── Motion ───────────────────────────────────────────────────────────

    def run_action(self, action_name, bt_node=None, semantic_source=None, tick_id=None):
        self._attr(bt_node, semantic_source or 'run_action')
        self._motion.run_action(action_name)
        self._reset_attr()

    def set_servos_position(self, duration_ms, positions,
                            bt_node=None, semantic_source=None, tick_id=None):
        self._attr(bt_node, semantic_source or 'set_servos_position')
        self._motion.set_servos_position(duration_ms, positions)
        self._reset_attr()

    # ── Buzzer ───────────────────────────────────────────────────────────

    def publish_buzzer(self, buzzer_msg, bt_node=None, semantic_source=None,
                       tick_id=None, reason=None):
        tid = tick_id if tick_id is not None else self._tick_id()
        if self._logger:
            self._logger.emit_comm({
                "event":               "ros_out",
                "ts":                  _time.time(),
                "tick_id":             tid,
                "bt_node":             bt_node or '',
                "semantic_source":     semantic_source or 'publish_buzzer',
                "target":              "/ros_robot_controller/set_buzzer",
                "comm_type":           "topic_publish",
                "direction":           "out",
                "ros_node":            "ros_robot_controller",
                "payload": {
                    "freq":     getattr(buzzer_msg, 'freq',     None),
                    "on_time":  getattr(buzzer_msg, 'on_time',  None),
                    "off_time": getattr(buzzer_msg, 'off_time', None),
                    "repeat":   getattr(buzzer_msg, 'repeat',   None),
                },
                "summary": "{} published /ros_robot_controller/set_buzzer reason={}".format(
                    bt_node or '', reason or ''),
                "attribution_confidence": "high",
                # legacy alias
                "node": bt_node or '',
            })
        self._buzzer.publish(buzzer_msg)
