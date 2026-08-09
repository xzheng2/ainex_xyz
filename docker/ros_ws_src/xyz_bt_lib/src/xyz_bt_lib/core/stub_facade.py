#!/usr/bin/env python3
"""StubFacade — a no-op XyzBTFacade for offline tree construction.

Purpose
-------
Build a behaviour tree with NO ROS and NO hardware, so it can be rendered,
statically inspected, or unit-checked. Every abstract method of XyzBTFacade is
implemented as a side-effect-free no-op; the two value-returning-by-contract
hooks (start_process / stop_process) are overridden so they never raise.

Why this works
--------------
xyz_bt_lib design rule 4 ("zero-ROS __init__"): node ``__init__`` never calls a
ROS API — ROS resources are created only in ``setup()``. A tree is fully wired
by constructing its nodes, so **building without calling setup() yields the
complete structure offline.** Nodes still need *a* facade object to hold a
reference to; StubFacade satisfies the XyzBTFacade contract without doing
anything, so construction succeeds off-robot.

This is a build-time / rendering aid only. Ticking a tree wired with StubFacade
will execute no motion — it is not a simulator.
"""
from xyz_bt_lib.core.base_facade import XyzBTFacade


class StubFacade(XyzBTFacade):
    """No-op implementation of XyzBTFacade for offline build / render / inspect."""

    # ── Primitive: gait ───────────────────────────────────────────────────
    def disable_gait(self, bt_node=None, tick_id=None):
        return None

    def enable_gait(self, bt_node=None, tick_id=None):
        return None

    def stop_gait(self, bt_node=None, tick_id=None):
        return None

    def set_step(self, dsp, x, y, yaw, gait_param=None, arm_swap=None,
                 step_num=0, bt_node=None, tick_id=None,
                 semantic_source='set_step', motion_profile=None):
        return None

    # ── Primitive: motion ─────────────────────────────────────────────────
    def run_action(self, action_name, bt_node=None, tick_id=None):
        return None

    def set_servos_position(self, duration_ms, positions,
                            bt_node=None, tick_id=None):
        return None

    # ── Primitive: buzzer ─────────────────────────────────────────────────
    def publish_buzzer(self, freq, on_time, off_time, repeat,
                       bt_node=None, tick_id=None):
        return None

    # ── Convenience wrappers ──────────────────────────────────────────────
    def go_step(self, x, y, yaw, step_num=None, period_time_ms=None,
                dsp_ratio=None, y_swap_amplitude=None, gait_param=None,
                arm_swap=None, bt_node=None, tick_id=None,
                semantic_source='gait_step'):
        return None

    def turn_step(self, x, y, yaw, step_num=0, period_time_ms=None,
                  dsp_ratio=None, y_swap_amplitude=None, gait_param=None,
                  arm_swap=None, bt_node=None, tick_id=None,
                  semantic_source='gait_step'):
        return None

    def move_head(self, pan_pos, tilt_pos=None, bt_node=None, tick_id=None):
        return None

    # ── Background process control ────────────────────────────────────────
    # Base class raises NotImplementedError; override as no-ops so a tree that
    # wires L3 process-control nodes still builds offline.
    def start_process(self, target, bt_node=None, tick_id=None):
        return None

    def stop_process(self, target, bt_node=None, tick_id=None):
        return None
