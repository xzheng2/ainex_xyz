#!/usr/bin/env python3
"""Generic ROS Facade — unified ROS communication exit and sole business log outlet.

All outbound ROS communications from fall_recovery BT leaf nodes flow through this
class via the Project Semantic Facade.  Every method:
  1. Emits a 'ros_out' log entry (via _emit) with full attribution fields.
  2. Calls the real runtime object (gait_manager / motion_manager / publisher).

Log schema (ros_out event):
    event, ts, tick_id, phase, bt_node, ros_node, semantic_source,
    target, comm_type, direction, payload, summary, attribution_confidence,
    node (legacy alias for bt_node).
"""
import time as _time


class CommFacade:
    """Unified ROS communication exit for fall_recovery BT."""

    def __init__(self, gait_manager, motion_manager, buzzer_pub,
                 logger=None, tick_id_getter=None):
        self._gait   = gait_manager    # real GaitManager (not proxy)
        self._motion = motion_manager  # real MotionManager (not proxy)
        self._buzzer = buzzer_pub
        self._logger = logger
        self._tick_id = tick_id_getter or (lambda: -1)

    # ── Logging helper ───────────────────────────────────────────────────

    def _emit(self, bt_node, semantic_source, target, comm_type,
              direction, ros_node, payload, summary='', tick_id=None):
        """Emit a ros_out log entry.  No-op when logger is None."""
        if not self._logger:
            return
        tid = tick_id if tick_id is not None else self._tick_id()
        self._logger.emit_comm({
            "event":                  "ros_out",
            "ts":                     _time.time(),
            "tick_id":                tid,
            "phase":                  "tick",
            "bt_node":                bt_node or '',
            "ros_node":               ros_node or '',
            "semantic_source":        semantic_source or '',
            "target":                 target,
            "comm_type":              comm_type,
            "direction":              direction,
            "payload":                payload,
            "summary":                summary,
            "attribution_confidence": "high",
            # legacy alias kept for transition period
            "node":                   bt_node or '',
        })

    # ── Gait ─────────────────────────────────────────────────────────────

    def disable_gait(self, bt_node=None, semantic_source=None, tick_id=None):
        self._emit(bt_node, semantic_source or 'disable_gait',
                   '/walking/command', 'service_call', 'call',
                   'ainex_controller', {'command': 'disable'},
                   tick_id=tick_id)
        self._gait.disable()

    def enable_gait(self, bt_node=None, semantic_source=None, tick_id=None):
        self._emit(bt_node, semantic_source or 'enable_gait',
                   '/walking/command', 'service_call', 'call',
                   'ainex_controller', {'command': 'enable'},
                   tick_id=tick_id)
        self._gait.enable()

    # ── Motion ───────────────────────────────────────────────────────────

    def run_action(self, action_name, bt_node=None, semantic_source=None, tick_id=None):
        self._emit(bt_node, semantic_source or 'run_action',
                   'ros_robot_controller/bus_servo/set_position', 'topic_publish', 'out',
                   'ros_robot_controller', {'action': action_name},
                   tick_id=tick_id)
        self._motion.run_action(action_name)

    # ── Buzzer ───────────────────────────────────────────────────────────

    def publish_buzzer(self, buzzer_msg, bt_node=None, semantic_source=None,
                       tick_id=None, reason=None):
        payload = {
            "freq":     getattr(buzzer_msg, 'freq',     None),
            "on_time":  getattr(buzzer_msg, 'on_time',  None),
            "off_time": getattr(buzzer_msg, 'off_time', None),
            "repeat":   getattr(buzzer_msg, 'repeat',   None),
        }
        summary = "{} published /ros_robot_controller/set_buzzer reason={}".format(
            bt_node or '', reason or '')
        self._emit(bt_node, semantic_source or 'publish_buzzer',
                   '/ros_robot_controller/set_buzzer', 'topic_publish', 'out',
                   'ros_robot_controller', payload, summary=summary, tick_id=tick_id)
        self._buzzer.publish(buzzer_msg)
