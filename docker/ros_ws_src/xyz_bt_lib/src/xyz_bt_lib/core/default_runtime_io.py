#!/usr/bin/env python3
"""_RuntimeIO — sole raw ROS / manager egress.

All outbound ROS communications and ros_out log entries flow through this class.
No other layer (RuntimeFacade, L2 nodes, tree) may call gait_manager,
motion_manager, or ROS publishers directly.

Layer rules:
  - _RuntimeIO is the ONLY class that holds gait_manager, motion_manager,
    publisher references, or ROS ServiceProxy objects.
  - _RuntimeIO only receives fully-resolved parameters; it does NOT perform
    profile selection, base cfg merging, or project business logic.
  - RuntimeFacade calls _RuntimeIO; L2 nodes never call _RuntimeIO directly.

WHY THIS IS SHARED AND NOT SCAFFOLDED
    It used to be generated per project from a template. The template carried two
    placeholders and both were in docstrings -- the code body was byte-identical
    everywhere, so every project got the same 150 lines with a different title.
    A project only ever needs its own copy to retune the step velocity profile,
    and subclassing says that far more precisely than a full copy does.

RETUNING THE STEP VELOCITY PROFILE
    ``_GO_STEP_VEL`` / ``_TURN_STEP_VEL`` are the per-project knobs. Override them
    in a subclass rather than editing this file, which every project shares::

        # <project>/runtime/_runtime_io.py   (optional -- only when retuning)
        from xyz_bt_lib.core.default_runtime_io import _RuntimeIO as _BaseRuntimeIO

        class _RuntimeIO(_BaseRuntimeIO):
            _GO_STEP_VEL = [250, 0.1, 0.01]

    The ``as`` alias is not stylistic: the subclass keeps the name ``_RuntimeIO``
    so that skill rule #3 and the guard's ``runtime/_runtime_io.py`` path key still
    describe it, which means the base class has to be imported under another name.
"""
import time as _time
from ros_robot_controller.msg import BuzzerState


class _RuntimeIO:
    """Unified raw ROS egress for a BT node. Subclass only to retune the profile."""

    # Step velocity profile: [period_time_ms, dsp_ratio, y_swap_amplitude]
    # TODO: tune per project; lower period_time_ms = faster gait cycle
    _GO_STEP_VEL   = [300, 0.1, 0.01]  # forward walk
    _TURN_STEP_VEL = [400, 0.1, 0.01]  # turn-in-place

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
                 motion_profile=None,
                 period_time_ms=None, dsp_ratio=None, y_swap_amplitude=None, **_kw):
        """Send one fully-resolved gait step. No profile inference here.

        step_vel = [period_time_ms, dsp_ratio, y_swap_amplitude]; per-call
        overrides take precedence over _GO_STEP_VEL / _TURN_STEP_VEL defaults.
        """
        # Select base step_vel from motion profile; copy to prevent mutation
        step_vel = list(self._TURN_STEP_VEL if motion_profile == 'turn'
                        else self._GO_STEP_VEL)
        if period_time_ms is not None:
            step_vel[0] = period_time_ms
        if dsp_ratio is not None:
            step_vel[1] = dsp_ratio
        if y_swap_amplitude is not None:
            step_vel[2] = y_swap_amplitude
        # Merge partial gait_param onto controller defaults
        if gait_param is not None:
            base = self._gait.get_gait_param()
            base.update(gait_param)
            gait_param = base
        payload = {'step_vel': step_vel, 'x': x, 'y': y, 'yaw': yaw}
        if motion_profile is not None:
            payload['motion_profile'] = motion_profile
        self._emit(bt_node, semantic_source or 'set_step',
                   'walking/set_param', 'topic_publish', 'out',
                   'ainex_controller', payload, tick_id=tick_id)
        self._gait.set_step(step_vel, x, y, yaw, gait_param,
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
