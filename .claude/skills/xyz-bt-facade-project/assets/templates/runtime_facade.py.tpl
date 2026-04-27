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
"""
from xyz_bt_edu.base_facade import XyzBTFacade


class {{PROJECT_CLASS}}RuntimeFacade(XyzBTFacade):
    """{{PROJECT_CLASS}} RuntimeFacade: implements XyzBTFacade for this project."""

    # ── Head constants ────────────────────────────────────────────────────
    HEAD_PAN_SERVO     = 23
    HEAD_MOVE_DELAY_MS = 50

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

    def go_step(self, x, y, yaw, step_num=0, bt_node=None, tick_id=None,
                semantic_source='gait_step'):
        """Merge go_cfg (static) + x/y/yaw (per-tick) → set_step(..., motion_profile='go')."""
        self._io.set_step(
            dsp=self._go_cfg['dsp'],
            x=x, y=y, yaw=yaw,
            gait_param=self._go_cfg.get('gait_param'),
            arm_swap=self._go_cfg.get('arm_swap'),
            step_num=self._go_cfg.get('step_num', step_num),
            bt_node=bt_node, tick_id=self._tid(tick_id),
            semantic_source=semantic_source, motion_profile='go',
        )

    def turn_step(self, x, y, yaw, step_num=0, bt_node=None, tick_id=None,
                  semantic_source='gait_step'):
        """Merge turn_cfg (static) + x/y/yaw (per-tick) → set_step(..., motion_profile='turn')."""
        self._io.set_step(
            dsp=self._turn_cfg['dsp'],
            x=x, y=y, yaw=yaw,
            gait_param=self._turn_cfg.get('gait_param'),
            arm_swap=self._turn_cfg.get('arm_swap'),
            step_num=self._turn_cfg.get('step_num', step_num),
            bt_node=bt_node, tick_id=self._tid(tick_id),
            semantic_source=semantic_source, motion_profile='turn',
        )

    def move_head(self, pan_pos, bt_node=None, tick_id=None):
        """Convenience wrapper: set_servos_position with HEAD_PAN_SERVO + HEAD_MOVE_DELAY_MS."""
        self._io.set_servos_position(
            self.HEAD_MOVE_DELAY_MS,
            [[self.HEAD_PAN_SERVO, int(pan_pos)]],
            bt_node=bt_node, tick_id=self._tid(tick_id),
        )
