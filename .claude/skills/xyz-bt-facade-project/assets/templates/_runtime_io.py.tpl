#!/usr/bin/env python3
"""{{PROJECT_CLASS}} _RuntimeIO — sole raw ROS / manager egress.

All outbound ROS communications and ros_out log entries flow through this class.
No other layer (RuntimeFacade, L2 nodes, tree) may call gait_manager,
motion_manager, or ROS publishers directly.

Layer rules:
  - _RuntimeIO is the ONLY class that holds gait_manager, motion_manager,
    publisher references, or ROS ServiceProxy objects.
  - _RuntimeIO only receives fully-resolved parameters; it does NOT perform
    profile selection, base cfg merging, or project business logic.
  - RuntimeFacade calls _RuntimeIO; L2 nodes never call _RuntimeIO directly.
"""
import time as _time
from ros_robot_controller.msg import BuzzerState


class _RuntimeIO:
    """Unified raw ROS egress for {{PROJECT_CLASS}} BT."""

    def __init__(self, gait_manager, motion_manager, buzzer_pub,
                 logger=None, tick_id_getter=None):
        """
        Args:
            gait_manager:    Walking controller handle (has enable/disable/set_step).
            motion_manager:  Motion controller handle (has run_action/set_servos_position).
            buzzer_pub:      rospy.Publisher for /ros_robot_controller/set_buzzer.
            logger:          BT observability logger (None = zero-cost no-op).
            tick_id_getter:  callable → current tick_id (int).
        """
        self._gait    = gait_manager
        self._motion  = motion_manager
        self._buzzer  = buzzer_pub
        self._logger  = logger
        self._tick_id = tick_id_getter or (lambda: -1)

    # ── Logging helper ────────────────────────────────────────────────────

    def _emit(self, bt_node, semantic_source, target, comm_type,
              direction, ros_node, payload, summary='', tick_id=None):
        """Emit a ros_out log entry. No-op when logger is None."""
        if not self._logger:
            return
        tid = tick_id if tick_id is not None else self._tick_id()
        self._logger.emit_comm({
            'event':                  'ros_out',
            'ts':                     _time.time(),
            'tick_id':                tid,
            'phase':                  'tick',
            'bt_node':                bt_node or '',
            'ros_node':               ros_node or '',
            'semantic_source':        semantic_source or '',
            'target':                 target,
            'comm_type':              comm_type,
            'direction':              direction,
            'payload':                payload,
            'summary':                summary,
            'attribution_confidence': 'high',
            'node':                   bt_node or '',  # legacy alias
        })

    # ── Gait ──────────────────────────────────────────────────────────────

    def disable_gait(self, bt_node=None, semantic_source=None, tick_id=None):
        self._emit(bt_node, semantic_source or 'disable_gait',
                   '/walking/command', 'service_call', 'call',
                   'ainex_controller', {'command': 'disable'}, tick_id=tick_id)
        self._gait.disable()

    def enable_gait(self, bt_node=None, semantic_source=None, tick_id=None):
        self._emit(bt_node, semantic_source or 'enable_gait',
                   '/walking/command', 'service_call', 'call',
                   'ainex_controller', {'command': 'enable'}, tick_id=tick_id)
        self._gait.enable()

    def stop_gait(self, bt_node=None, semantic_source=None, tick_id=None):
        """Stop current gait motion; controller stays up for new commands."""
        self._emit(bt_node, semantic_source or 'stop_gait',
                   '/walking/command', 'service_call', 'call',
                   'ainex_controller', {'command': 'stop'}, tick_id=tick_id)
        self._gait.stop()

    def set_step(self, bt_node=None, semantic_source=None, tick_id=None, *,
                 dsp, x, y, yaw, gait_param=None, arm_swap=None, step_num=0,
                 motion_profile=None):
        """Send one fully-resolved gait step. No profile inference here."""
        payload = {'dsp': dsp, 'x': x, 'y': y, 'yaw': yaw}
        if motion_profile is not None:
            payload['motion_profile'] = motion_profile
        self._emit(bt_node, semantic_source or 'set_step',
                   'walking/set_param', 'topic_publish', 'out',
                   'ainex_controller', payload, tick_id=tick_id)
        self._gait.set_step(dsp, x, y, yaw, gait_param,
                            arm_swap=arm_swap, step_num=step_num)

    # ── Motion ────────────────────────────────────────────────────────────

    def run_action(self, action_name, bt_node=None, semantic_source=None,
                   tick_id=None):
        self._emit(bt_node, semantic_source or 'run_action',
                   'ros_robot_controller/bus_servo/set_position', 'topic_publish', 'out',
                   'ros_robot_controller', {'action': action_name}, tick_id=tick_id)
        self._motion.run_action(action_name)

    def set_servos_position(self, duration_ms, positions,
                             bt_node=None, semantic_source=None, tick_id=None):
        self._emit(bt_node, semantic_source or 'set_servos_position',
                   'ros_robot_controller/bus_servo/set_position', 'topic_publish', 'out',
                   'ros_robot_controller',
                   {'duration': duration_ms, 'positions': positions}, tick_id=tick_id)
        self._motion.set_servos_position(duration_ms, positions)

    # ── Buzzer ────────────────────────────────────────────────────────────

    def publish_buzzer(self, freq, on_time, off_time, repeat,
                       bt_node=None, semantic_source=None, tick_id=None):
        """Construct BuzzerState internally; public contract uses scalar params."""
        payload = {'freq': freq, 'on_time': on_time,
                   'off_time': off_time, 'repeat': repeat}
        summary = '{} published /ros_robot_controller/set_buzzer'.format(bt_node or '')
        self._emit(bt_node, semantic_source or 'publish_buzzer',
                   '/ros_robot_controller/set_buzzer', 'topic_publish', 'out',
                   'ros_robot_controller', payload, summary=summary, tick_id=tick_id)
        msg = BuzzerState(freq=freq, on_time=on_time, off_time=off_time, repeat=repeat)
        self._buzzer.publish(msg)
