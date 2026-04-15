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

        Args:
            line_data: Line detection result (must have .x and .width fields).
        """
        ...

    @abstractmethod
    def search_line(self, last_line_x, lost_count: int,
                    bt_node: str, tick_id: int = None) -> int:
        """Turn in-place to recover a lost line.

        Direction is biased toward where the line was last seen.
        Magnitude may scale with lost_count.

        Args:
            last_line_x: x-position of line when last seen, or None.
            lost_count:  number of consecutive ticks the line has been absent.

        Returns:
            gait_yaw (int): the yaw command actually sent (for logging).
        """
        ...

    # ── Recovery ──────────────────────────────────────────────────────────

    @abstractmethod
    def recover_from_fall(self, robot_state: str,
                          bt_node: str, tick_id: int = None) -> None:
        """Execute full fall-recovery sequence (buzzer → disable gait → stand-up action).

        Args:
            robot_state: current posture string used to select correct stand-up action.
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

    @abstractmethod
    def head_sweep_align(self, head_offset: int,
                         bt_node: str, tick_id: int = None) -> int:
        """Command body-turn proportional to head_offset during line-sweep alignment.

        Args:
            head_offset: current head_pan - HEAD_PAN_CENTER (signed).

        Returns:
            gait_yaw (int): the yaw command actually sent (for logging).
        """
        ...
