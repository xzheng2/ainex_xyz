#!/usr/bin/env python3
# encoding: utf-8
"""ImuBalanceStateAdapter — /imu → /latched/robot_state

Input adapter.

ROS topic subscribed:
  /imu

ROS message type:
  sensor_msgs/Imu  (100 Hz)

BB keys written:
  BB.ROBOT_STATE  (/latched/robot_state)

Per-BB-value traceability:
  /imu.linear_acceleration.{y,z}
    → _extract_imu()      : angle = abs(degrees(atan2(ay, az)))
    → _classify_fall(angle, current_state, count_lie, count_recline)
                          : returns (new_count_lie, new_count_recline,
                            new_state_or_none); new_state is 'lie', 'recline',
                            or None (no change); side-effect-free; no self
                            mutation, no ROS calls; skips accumulation when
                            current_state != 'stand' (recovery in progress)
    → _callback()         : applies returned counters to self._count_lie /
                            self._count_recline; calls rospy.loginfo on fall
    → _make_bb_writes()   : {BB.ROBOT_STATE: 'lie'|'recline'} or {}
    → _apply_live_writes(): updates self._live_robot_state under self._lock
    → snapshot field      : 'robot_state'
    → BB.ROBOT_STATE  ∈ {'stand', 'lie', 'recline'}

Threshold/calibration source:
  CONFIG_DEFAULTS:
    fall_count_threshold = 100   (≈ 1 s at 100 Hz)
    lie_angle_max        = 30°   (tilt < 30° → face-down)
    recline_angle_min    = 150°  (tilt > 150° → back-down)

Two-phase latch protocol:
  Phase 1: snapshot_and_reset() while caller holds the shared lock.
  Phase 2: write_snapshot(snap, tick_id) after lock release, on main thread.

Extra public API:
  force_state(state) — override live robot_state without waiting for IMU;
    called by L2_Balance_RecoverFromFall's robot_state_setter hook so the
    next pre-tick snapshot does not overwrite BB with a stale pre-recovery
    IMU value.

Observability:
  write_snapshot() emits only, via AinexInputAdapter helpers:
    - ros_in when received_count > 0
    - input_state every tick
  This adapter never emits ros_out/ros_result.
"""
import math
import threading

import rospy
from py_trees.common import Access
from sensor_msgs.msg import Imu

from ainex_bt_edu.blackboard_keys import BB
from ainex_bt_edu.base_adapter import AinexInputAdapter


class ImuBalanceStateAdapter(AinexInputAdapter):
    """Adapter: /imu → fall-detection logic → /latched/robot_state (per tick)."""

    ROS_TOPIC    = '/imu'
    ADAPTER_NAME = 'ImuBalanceStateAdapter'
    BB_READS     = []
    BB_WRITES    = [BB.ROBOT_STATE]
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'fall_count_threshold': 100,
        'lie_angle_max':         30,
        'recline_angle_min':    150,
    }

    def __init__(self, lock: threading.Lock, logger=None, tick_id_getter=None,
                 fall_count_threshold: int = 100,
                 lie_angle_max: int = 30,
                 recline_angle_min: int = 150):
        """
        Args:
            lock:                 Shared threading.Lock (same as bt_node.lock).
            logger:               DebugEventLogger-compatible object, or None.
            tick_id_getter:       Callable returning current tick_id.
            fall_count_threshold: Consecutive IMU frames above angle threshold
                                  before a fall is confirmed. Default 100 ≈ 1 s
                                  at 100 Hz.
            lie_angle_max:        Tilt angle (°) below which the robot is
                                  classified as face-down (lie_to_stand).
            recline_angle_min:    Tilt angle (°) above which the robot is
                                  classified as back-down (recline_to_stand).
        """
        super().__init__(lock=lock, logger=logger, tick_id_getter=tick_id_getter)

        self._fall_count_threshold = fall_count_threshold
        self._lie_angle_max        = lie_angle_max
        self._recline_angle_min    = recline_angle_min

        # Live (async) state — written only by _callback under self._lock.
        self._live_robot_state = 'stand'

        # Fall-detection accumulators — callback-thread-only; no lock needed.
        self._count_lie     = 0
        self._count_recline = 0

        # Latched BB client — written only by write_snapshot() (main thread).
        self._bb = self.make_latched_bb_client(name=self.ADAPTER_NAME)
        self._bb.register_key(key=BB.ROBOT_STATE_KEY, access=Access.WRITE)
        # Initialise BB before first tick so BT nodes never hit KeyError.
        self._bb.robot_state = self._live_robot_state

        rospy.Subscriber(self.ROS_TOPIC, Imu, self._callback)

    # ------------------------------------------------------------------
    # Side-effect-free conversion helpers
    # ------------------------------------------------------------------

    def _extract_imu(self, msg: Imu) -> int:
        """Extract absolute tilt angle (degrees) from linear acceleration.

        angle = abs(atan2(ay, az)) in degrees.
        No BB writes, ROS calls, logger calls, or live-state mutation here.
        """
        return abs(int(math.degrees(
            math.atan2(msg.linear_acceleration.y,
                       msg.linear_acceleration.z))))

    def _classify_fall(self, angle: int, current_state: str,
                       count_lie: int, count_recline: int) -> tuple:
        """Compute updated fall-count accumulators and optional new posture state.

        Side-effect-free: reads only parameters and constructor thresholds;
        returns (new_count_lie, new_count_recline, new_state_or_none).
        new_state is the confirmed posture: 'lie' (face-down) or 'recline'
        (back-down), or None if no threshold was crossed.
        No self-state mutation, no ROS calls, no BB reads/writes.
        """
        if current_state != 'stand':
            return count_lie, count_recline, None

        new_lie     = (count_lie + 1)     if angle < self._lie_angle_max     else 0
        new_recline = (count_recline + 1) if angle > self._recline_angle_min else 0

        if new_lie > self._fall_count_threshold:
            return 0, 0, 'lie'
        if new_recline > self._fall_count_threshold:
            return 0, 0, 'recline'
        return new_lie, new_recline, None

    def _make_bb_writes(self, new_state) -> dict:
        """Return {BB.ROBOT_STATE: new_state} when a fall was confirmed, else {}.

        Empty dict means _apply_live_writes() preserves the current live state.
        No BB reads/writes, ROS calls, or live-state mutation here.
        """
        if new_state is not None:
            return {BB.ROBOT_STATE: new_state}
        return {}

    def _apply_live_writes(self, bb_writes: dict) -> None:
        """Copy confirmed new state into self._live_robot_state.

        Called by _callback() while holding self._lock. This is the only helper
        in the conversion path that mutates live (shared) state.
        """
        if BB.ROBOT_STATE in bb_writes:
            self._live_robot_state = bb_writes[BB.ROBOT_STATE]

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg: Imu) -> None:
        """Receive IMU message, detect falls, update live state.

        Two lock acquisitions are required: the first reads current_state to
        pass to _classify_fall() (which must skip accumulation during recovery);
        the second commits the result and increments received_count.

        Counter updates and ROS logging live here (not in _classify_fall).
        """
        # Read current state to guard fall accumulation during recovery.
        with self._lock:
            current_state = self._live_robot_state

        angle = self._extract_imu(msg)
        new_count_lie, new_count_recline, new_state = self._classify_fall(
            angle, current_state, self._count_lie, self._count_recline)

        # Update callback-thread-only accumulators (no lock needed).
        self._count_lie     = new_count_lie
        self._count_recline = new_count_recline

        if new_state is not None:
            rospy.loginfo('[ImuAdapter] posture: %s', new_state)

        bb_writes = self._make_bb_writes(new_state)
        with self._lock:
            self._apply_live_writes(bb_writes)
            self._received_count += 1

    # ------------------------------------------------------------------
    # Public two-phase latch API
    # ------------------------------------------------------------------

    def force_state(self, state: str) -> None:
        """Override live robot_state (called by RecoverFromFall robot_state_setter).

        Writes the given state into the live store so the next pre-tick snapshot
        does not overwrite the latched BB with a stale pre-recovery IMU value.
        """
        with self._lock:
            self._live_robot_state = state

    def snapshot_and_reset(self) -> dict:
        """Return current live-state snapshot and reset received_count.

        Must be called while the caller holds self._lock.
        """
        snap = {
            'robot_state':    self._live_robot_state,
            'received_count': self._received_count,
        }
        self._received_count = 0
        return snap

    def write_snapshot(self, snap: dict, tick_id: int) -> None:
        """Write pre-captured snapshot to BB and emit business-in events.

        No lock is needed; called only from the main thread after the shared
        lock has been released.
        """
        self.emit_ros_in(tick_id=tick_id, received_count=snap['received_count'])

        # Write to BB before input_state so logs reflect what BT will read.
        self._bb.robot_state = snap['robot_state']

        self.emit_input_state(
            tick_id=tick_id,
            bb_writes={BB.ROBOT_STATE: snap['robot_state']},
        )
