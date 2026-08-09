#!/usr/bin/env python3
"""{{PROJECT_CLASS}} RuntimeFacade — BT-facing runtime interface.

Layer responsibilities
----------------------
L2 nodes      → express WHAT per-tick action is needed
RuntimeFacade → profile/base cfg + override synthesis; convenience wrapper composition
_RuntimeIO    → sole raw ROS / manager egress; sole ros_out log outlet

Public contract (mirrors XyzBTFacade):
  Primitives: disable_gait / enable_gait / stop_gait / set_step /
              run_action / set_servos_position / publish_buzzer
  Wrappers:   go_step / turn_step / move_head

Rules:
  - RuntimeFacade must NOT hold gait_manager, motion_manager, or ROS publishers.
  - All ROS I/O goes through self._io (_RuntimeIO).
  - go_step / turn_step merge project profile cfg with per-tick x/y/yaw and call set_step.

go_step yaw smoothing:
  go_step() collects raw yaw over _YAW_AVG_WINDOW BT ticks and publishes the
  window average (held constant across the window) so continuous (step_num=0)
  gait executes one stable rotation per window — see _YAW_AVG_WINDOW.
"""
from xyz_bt_lib.core.base_facade import XyzBTFacade


class {{PROJECT_CLASS}}RuntimeFacade(XyzBTFacade):
    """{{PROJECT_CLASS}} RuntimeFacade: implements XyzBTFacade for this project."""

    # ── Head constants ────────────────────────────────────────────────────
    HEAD_PAN_SERVO     = 23
    HEAD_MOVE_DELAY_MS = 50

    # ── go_step yaw accumulation ──────────────────────────────────────────
    # Collect raw yaw over this many BT ticks, then publish their average for
    # the next window so the gait executes one stable rotation per window
    # (both gait phase samples see the same value). Tune to
    # gait_cycle_ms / bt_tick_ms (e.g. 300 ms / 100 ms = 3).
    _YAW_AVG_WINDOW = 3

    def __init__(self, runtime_io, go_cfg: dict, turn_cfg: dict,
                 tick_id_getter=None):
        """
        Args:
            runtime_io:      _RuntimeIO instance (sole raw ROS / manager egress).
            go_cfg:          Static gait params for the 'go' (forward) profile.
                             Keys: dsp (float), gait_param (msg|None),
                                   arm_swap (bool|None), step_num (int).
            turn_cfg:        Static gait params for the 'turn' (rotation) profile.
                             Same keys as go_cfg.
            tick_id_getter:  callable → current tick_id (int).
        """
        self._io       = runtime_io
        self._go_cfg   = go_cfg
        self._turn_cfg = turn_cfg
        self._tick_id  = tick_id_getter or (lambda: -1)

        # go_step yaw smoothing state (see _YAW_AVG_WINDOW)
        self._collect_buf = []    # raw yaw values in current collect window
        self._publish_yaw = 0     # averaged yaw being published (int)

    def _tid(self, tick_id):
        return tick_id if tick_id is not None else self._tick_id()

    # ── Primitives: gait ──────────────────────────────────────────────────

    def disable_gait(self, bt_node=None, tick_id=None):
        self._io.disable_gait(bt_node=bt_node, tick_id=self._tid(tick_id))

    def enable_gait(self, bt_node=None, tick_id=None):
        self._io.enable_gait(bt_node=bt_node, tick_id=self._tid(tick_id))

    def stop_gait(self, bt_node=None, tick_id=None):
        self._io.stop_gait(bt_node=bt_node, tick_id=self._tid(tick_id))

    def set_step(self, dsp, x, y, yaw, gait_param=None, arm_swap=None,
                 step_num=0, bt_node=None, tick_id=None,
                 semantic_source='set_step', motion_profile=None):
        self._io.set_step(
            dsp=dsp, x=x, y=y, yaw=yaw,
            gait_param=gait_param, arm_swap=arm_swap, step_num=step_num,
            bt_node=bt_node, tick_id=self._tid(tick_id),
            semantic_source=semantic_source, motion_profile=motion_profile,
        )

    # ── Primitives: motion ────────────────────────────────────────────────

    def run_action(self, action_name, bt_node=None, tick_id=None):
        self._io.run_action(action_name, bt_node=bt_node,
                            tick_id=self._tid(tick_id))

    def set_servos_position(self, duration_ms, positions,
                             bt_node=None, tick_id=None):
        self._io.set_servos_position(duration_ms, positions,
                                     bt_node=bt_node, tick_id=self._tid(tick_id))

    # ── Primitives: buzzer ────────────────────────────────────────────────

    def publish_buzzer(self, freq, on_time, off_time, repeat,
                       bt_node=None, tick_id=None):
        self._io.publish_buzzer(freq=freq, on_time=on_time, off_time=off_time,
                                repeat=repeat, bt_node=bt_node,
                                tick_id=self._tid(tick_id))

    # ── Convenience wrappers ──────────────────────────────────────────────

    def go_step(self, x, y, yaw, step_num=None,
                period_time_ms=None, dsp_ratio=None, y_swap_amplitude=None,
                gait_param=None, arm_swap=None,
                bt_node=None, tick_id=None,
                semantic_source='gait_step'):
        """Merge go_cfg (static) + per-call overrides + x/y/yaw → set_step(motion_profile='go').

        Per-call overrides (period_time_ms, dsp_ratio, y_swap_amplitude, arm_swap,
        step_num, gait_param) take precedence over _GO_CFG when not None.

        Yaw smoothing: raw yaw is collected over _YAW_AVG_WINDOW ticks; the
        published yaw is the previous window's average (held constant across the
        window) so the gait executes one stable rotation per window.
        """
        self._collect_buf.append(yaw)
        if len(self._collect_buf) >= self._YAW_AVG_WINDOW:
            _avg = sum(self._collect_buf) / len(self._collect_buf)
            self._publish_yaw = int(round(_avg))
            self._collect_buf = []

        cfg = {**self._go_cfg, 'x': x, 'y': y, 'yaw': self._publish_yaw,
               'motion_profile': 'go'}
        if step_num is not None:
            cfg['step_num'] = step_num
        if arm_swap is not None:
            cfg['arm_swap'] = arm_swap
        if gait_param is not None:
            cfg['gait_param'] = gait_param
        self._io.set_step(
            **cfg,
            period_time_ms=period_time_ms, dsp_ratio=dsp_ratio,
            y_swap_amplitude=y_swap_amplitude,
            bt_node=bt_node, tick_id=self._tid(tick_id),
            semantic_source=semantic_source,
        )

    def turn_step(self, x, y, yaw, step_num=None,
                  period_time_ms=None, dsp_ratio=None, y_swap_amplitude=None,
                  gait_param=None, arm_swap=None,
                  bt_node=None, tick_id=None,
                  semantic_source='gait_step'):
        """Merge turn_cfg (static) + per-call overrides + x/y/yaw → set_step(motion_profile='turn').

        Per-call overrides (period_time_ms, dsp_ratio, y_swap_amplitude, arm_swap,
        step_num, gait_param) take precedence over _TURN_CFG when not None.
        """
        cfg = {**self._turn_cfg, 'x': x, 'y': y, 'yaw': yaw, 'motion_profile': 'turn'}
        if step_num is not None:
            cfg['step_num'] = step_num
        if arm_swap is not None:
            cfg['arm_swap'] = arm_swap
        if gait_param is not None:
            cfg['gait_param'] = gait_param
        self._io.set_step(
            **cfg,
            period_time_ms=period_time_ms, dsp_ratio=dsp_ratio,
            y_swap_amplitude=y_swap_amplitude,
            bt_node=bt_node, tick_id=self._tid(tick_id),
            semantic_source=semantic_source,
        )

    def move_head(self, pan_pos, bt_node=None, tick_id=None):
        """Convenience wrapper: set_servos_position with HEAD_PAN_SERVO + HEAD_MOVE_DELAY_MS."""
        self._io.set_servos_position(
            self.HEAD_MOVE_DELAY_MS,
            [[self.HEAD_PAN_SERVO, int(pan_pos)]],
            bt_node=bt_node, tick_id=self._tid(tick_id),
        )

