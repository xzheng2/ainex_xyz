#!/usr/bin/env python3
"""AinexBTFacade — standard project facade interface.

All project SemanticFacades must implement this abstract base class.
BT nodes in ainex_bt_edu accept AinexBTFacade, not project-specific types.

Layer contract
--------------
Leaf nodes     → express WHAT they want ("follow line", "search line")
SemanticFacade → implement HOW in project terms (specific gait params,
                 servo IDs, action files, algorithm choices)
CommFacade     → handle actual ROS communication + attributed logs

ROS rule: facade implementations must route all ROS I/O through CommFacade.
          Nodes must never call ROS APIs (publishers, services) directly.

Method signatures mirror MarathonSemanticFacade (the reference implementation).
All methods accept `bt_node` (node name string) and `tick_id` (int) for
cross-component observability correlation.
"""
from abc import ABC, abstractmethod


class AinexBTFacade(ABC):
    """Abstract base for all project semantic facades."""

    # ── Locomotion ────────────────────────────────────────────────────────

    @abstractmethod
    def stop_walking(self, bt_node: str, tick_id: int = None) -> None:
        """Disable the gait controller immediately."""
        ...

    @abstractmethod
    def follow_line(self, line_data, bt_node: str, tick_id: int = None) -> None:
        """Compute and send gait command to follow the detected line.

        Deprecated — algorithm moved to L2_Gait_FollowLine.update() which calls
        go_step() / turn_step() directly.  Kept only for abstract-contract
        compatibility; not called by any BT node in the current code path.

        Args:
            line_data: Line detection result (must have .x and .width fields).
        """
        ...

    @abstractmethod
    def gait_step(self, profile: str, x: float, y: float, yaw: int,
                  step_num: int = 0, bt_node: str = None, tick_id: int = None,
                  semantic_source: str = 'gait_step') -> None:
        """Send one gait step using the named motion profile.

        Args:
            profile: 'go' (straight walking) or 'turn' (rotation-biased).
                     Any other value must raise ValueError.
        """
        ...

    @abstractmethod
    def go_step(self, x: float, y: float, yaw: int,
                step_num: int = 0, bt_node: str = None, tick_id: int = None,
                semantic_source: str = 'gait_step') -> None:
        """Thin wrapper: gait_step('go', ...)."""
        ...

    @abstractmethod
    def turn_step(self, x: float, y: float, yaw: int,
                  step_num: int = 0, bt_node: str = None, tick_id: int = None,
                  semantic_source: str = 'gait_step') -> None:
        """Thin wrapper: gait_step('turn', ...)."""
        ...

    # ── Recovery ──────────────────────────────────────────────────────────

    @abstractmethod
    def recover_from_fall(self, robot_state: str,
                          bt_node: str, tick_id: int = None) -> None:
        """Execute full fall-recovery sequence (buzzer → disable gait → stand-up action).

        Args:
            robot_state: recovery action name — 'lie_to_stand' or 'recline_to_stand'.
                         L2_Balance_RecoverFromFall maps posture ('lie'|'recline') to
                         this action name before calling; the facade receives the
                         action name directly.
        Does NOT write to the blackboard — the calling BT node handles that.
        """
        ...

    # ── Head ──────────────────────────────────────────────────────────────

    @abstractmethod
    def move_head(self, pan_pos: int, bt_node: str, tick_id: int = None) -> None:
        """Command the head-pan servo to the given position.

        Args:
            pan_pos: target servo position (project-defined units; 500 = centre).
        """
        ...

