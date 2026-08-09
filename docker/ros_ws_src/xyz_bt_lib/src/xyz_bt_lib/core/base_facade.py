#!/usr/bin/env python3
"""XyzBTFacade — BT-facing runtime interface.

All project RuntimeFacades must implement this abstract base class.
BT nodes in xyz_bt_lib accept XyzBTFacade, not project-specific types.

Layer contract
--------------
Leaf nodes     → express WHAT they want ("go forward", "turn", "move head")
RuntimeFacade  → implement HOW in project terms (profile/base cfg + override
                 synthesis, convenience wrapper composition)
_RuntimeIO     → sole raw ROS / manager egress; sole ros_out log outlet

Execution rule: RuntimeFacade must route all ROS I/O through _RuntimeIO.
               L2 nodes and tree code must never call _RuntimeIO, managers,
               or ROS APIs (publishers, services) directly.

All methods accept `bt_node` (node name string) and `tick_id` (int) for
cross-component observability correlation.

Primitive runtime methods:
  disable_gait / enable_gait / stop_gait / set_step / run_action /
  set_servos_position / publish_buzzer

Convenience wrappers (compose primitives with project profile config):
  go_step / turn_step / move_head

stop_gait() vs disable_gait()
------------------------------
stop_gait()    — stop the current gait motion; the gait controller remains
                 up and can immediately accept new set_step() commands.
disable_gait() — shut down the gait controller; requires enable_gait()
                 before any further gait commands will be accepted.
"""
from abc import ABC, abstractmethod


class XyzBTFacade(ABC):
    """Abstract base for all project RuntimeFacades."""

    # ── Primitive: gait ───────────────────────────────────────────────────

    @abstractmethod
    def disable_gait(self, bt_node: str = None, tick_id: int = None) -> None:
        """Shut down the gait controller. Requires enable_gait() to restart."""
        ...

    @abstractmethod
    def enable_gait(self, bt_node: str = None, tick_id: int = None) -> None:
        """Start (or restart) the gait controller."""
        ...

    @abstractmethod
    def stop_gait(self, bt_node: str = None, tick_id: int = None) -> None:
        """Stop current gait motion; controller stays up for new commands."""
        ...

    @abstractmethod
    def set_step(self, dsp: float, x: float, y: float, yaw: int,
                 gait_param=None, arm_swap=None, step_num: int = 0,
                 bt_node: str = None, tick_id: int = None,
                 semantic_source: str = 'set_step',
                 motion_profile: str = None) -> None:
        """Send one gait step with fully-resolved parameters.

        This is the core gait primitive. All parameters must be final;
        no profile inference is done here.

        Args:
            dsp:           Double-support phase duration (seconds).
            x, y:          Forward / lateral displacement.
            yaw:           Rotation command (project-defined units).
            gait_param:    Optional GaitParam message; None = use controller default.
            arm_swap:      Optional arm-swing swap flag; None = use controller default.
            step_num:      Number of steps (0 = continuous).
            motion_profile: Optional label for observability log ('go' | 'turn').
        """
        ...

    # ── Primitive: motion ─────────────────────────────────────────────────

    @abstractmethod
    def run_action(self, action_name: str,
                   bt_node: str = None, tick_id: int = None) -> None:
        """Execute a named stand-alone motion action (e.g. 'lie_to_stand')."""
        ...

    @abstractmethod
    def set_servos_position(self, duration_ms: int, positions: list,
                             bt_node: str = None, tick_id: int = None) -> None:
        """Command one or more servos to target positions.

        Args:
            duration_ms: Move duration in milliseconds.
            positions:   List of [servo_id, target_position] pairs.
        """
        ...

    # ── Primitive: buzzer ─────────────────────────────────────────────────

    @abstractmethod
    def publish_buzzer(self, freq: int, on_time: float, off_time: float,
                       repeat: int,
                       bt_node: str = None, tick_id: int = None) -> None:
        """Sound the buzzer with the given pattern.

        Args:
            freq:     Tone frequency in Hz.
            on_time:  Beep on duration (seconds).
            off_time: Beep off duration (seconds).
            repeat:   Number of beeps.

        Note: _RuntimeIO constructs the BuzzerState message internally.
              The public contract does not expose ROS message types.
        """
        ...

    # ── Convenience wrappers ──────────────────────────────────────────────

    @abstractmethod
    def go_step(self, x: float, y: float, yaw: int,
                step_num: int = None,
                period_time_ms: int = None,
                dsp_ratio: float = None,
                y_swap_amplitude: float = None,
                gait_param: dict = None,
                arm_swap: int = None,
                bt_node: str = None, tick_id: int = None,
                semantic_source: str = 'gait_step') -> None:
        """Walk forward/lateral step using the project's 'go' gait profile.

        RuntimeFacade merges go_cfg (static profile params) with x/y/yaw
        (per-tick dynamic params) and calls set_step(..., motion_profile='go').

        All override params default to None (= use project _GO_CFG / _GO_STEP_VEL defaults):
            step_num:         Steps per command (0 = continuous). None = _GO_CFG.step_num.
            period_time_ms:   Gait cycle duration (ms). None = _GO_STEP_VEL[0].
            dsp_ratio:        Double-support fraction (0–1). None = _GO_STEP_VEL[1].
            y_swap_amplitude: Lateral body swing (m). None = _GO_STEP_VEL[2].
            gait_param:       Partial WalkingParam dict (e.g. {'step_height': 0.03}).
                              Merged onto current controller params. None = no override.
            arm_swap:         Arm swing amplitude (°). None = _GO_CFG.arm_swap.
        """
        ...

    @abstractmethod
    def turn_step(self, x: float, y: float, yaw: int,
                  step_num: int = 0,
                  period_time_ms: int = None,
                  dsp_ratio: float = None,
                  y_swap_amplitude: float = None,
                  gait_param: dict = None,
                  arm_swap: int = None,
                  bt_node: str = None, tick_id: int = None,
                  semantic_source: str = 'gait_step') -> None:
        """Turn-in-place step using the project's 'turn' gait profile.

        RuntimeFacade merges turn_cfg (static profile params) with x/y/yaw
        (per-tick dynamic params) and calls set_step(..., motion_profile='turn').

        All override params default to None (= use project _TURN_CFG / _TURN_STEP_VEL defaults):
            step_num:         Steps per command (0 = continuous). None/0 uses _TURN_CFG.step_num.
            period_time_ms:   Gait cycle duration (ms). None = _TURN_STEP_VEL[0].
            dsp_ratio:        Double-support fraction (0–1). None = _TURN_STEP_VEL[1].
            y_swap_amplitude: Lateral body swing (m). None = _TURN_STEP_VEL[2].
            gait_param:       Partial WalkingParam dict (e.g. {'step_height': 0.03}).
                              Merged onto current controller params. None = no override.
            arm_swap:         Arm swing amplitude (°). None = _TURN_CFG.arm_swap.
        """
        ...

    @abstractmethod
    def move_head(self, pan_pos: int, tilt_pos: int = None,
                  bt_node: str = None, tick_id: int = None) -> None:
        """Command head servo(s) to the given position(s).

        Convenience wrapper over set_servos_position() using the project's
        HEAD_PAN_SERVO id and HEAD_MOVE_DELAY_MS.

        Args:
            pan_pos:  Target pan servo position (project-defined units; 500 = centre).
            tilt_pos: Optional tilt servo position (servo 24). None = do not move tilt.
        """
        ...

    # ── Background process control (optional facade capability) ────────────
    # Concrete defaults so library nodes (L3_Process_Control) can accept any
    # facade. Projects that need to start/stop other programs (e.g. a stand-alone
    # servo script) hold a ProcessManager and override both (see process_manager.py).

    def start_process(self, target: str,
                      bt_node: str = None, tick_id: int = None) -> None:
        """Start a registered background process by name.

        `target` is a key in the project's ProcessManager registry. The default
        raises; a project facade overrides this to delegate to its ProcessManager.
        """
        raise NotImplementedError(
            '{} does not support start_process; the project facade must override '
            'it and hold a ProcessManager.'.format(type(self).__name__))

    def stop_process(self, target: str,
                     bt_node: str = None, tick_id: int = None) -> None:
        """Stop a registered background process by name.

        The default raises; a project facade overrides this to delegate to its
        ProcessManager.
        """
        raise NotImplementedError(
            '{} does not support stop_process; the project facade must override '
            'it and hold a ProcessManager.'.format(type(self).__name__))
