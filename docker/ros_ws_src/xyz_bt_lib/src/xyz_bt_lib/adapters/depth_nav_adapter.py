#!/usr/bin/env python3
# encoding: utf-8
"""DepthNavAdapter — /depth_nav/state → /latched/{nav_u9, nav_u10, nav_cx, nav_fx,
nav_start_bx, nav_start_by, nav_has_path, nav_obstacle_near, nav_imu_heading,
nav_imu_heading_min, nav_imu_heading_max}

Input adapter.

ROS topic subscribed:
  /depth_nav/state

ROS message type:
  xyz_bt_lib/DepthNavState  (~15 Hz, published by depth_nav_node.py)

BB keys written:
  BB.NAV_U9            (/latched/nav_u9)             float (NaN when no path / row missing)
  BB.NAV_U10           (/latched/nav_u10)            float (NaN when no path / row missing)
  BB.NAV_CX            (/latched/nav_cx)             float (camera principal point x, px)
  BB.NAV_FX            (/latched/nav_fx)             float (camera focal length x, px)
  BB.NAV_START_BX      (/latched/nav_start_bx)       int   (A* start block column)
  BB.NAV_START_BY      (/latched/nav_start_by)       int   (A* start block row)
  BB.NAV_HAS_PATH      (/latched/nav_has_path)       bool
  BB.NAV_OBSTACLE_NEAR (/latched/nav_obstacle_near)  bool
  BB.NAV_OBSTACLE_DETECTED (/latched/nav_obstacle_detected) bool (any obstacle anywhere in
                       the fused depth+YOLO mask; not distance/near-window gated)
  BB.NAV_IMU_HEADING   (/latched/nav_imu_heading)    float (deg; depth_nav's in-process
                       /imu integration, GUI cal-6ax logic; 0 at start)
  BB.NAV_IMU_HEADING_MIN (/latched/nav_imu_heading_min) float (deg; per-run max RIGHT/−
                       heading pose reached this run; 0 at start)
  BB.NAV_IMU_HEADING_MAX (/latched/nav_imu_heading_max) float (deg; per-run max LEFT/+
                       heading pose reached this run; 0 at start)

Per-BB-value traceability:
  msg.nav_u9 / msg.nav_u10 / msg.cx / msg.fx / msg.has_path / msg.obstacle_near
    → _extract_state()  : pull the six scalar fields off the message (no transform —
                          the planner already emits NaN for u9/u10 when there is no path,
                          and the L2 node decides what to do with NaN).
    → _make_bb_writes() : maps to the six BB keys
    → _apply_live_writes(): stores the live fields under self._lock
    → snapshot fields   : nav_u9, nav_u10, nav_cx, nav_fx, nav_has_path, nav_obstacle_near
    → BB.* keys

Design note:
  This adapter does NO steering/angle math. It latches the raw row-9/10 path geometry
  (block-centre pixel-x at depth grid rows 9 & 10) plus the camera intrinsics, and the
  consumer L2_Gait_FollowNavPath selects a row and computes the bearing atan((u-cx)/fx).
  All obstacle thresholds live in depth_nav_node.py. CONFIG_DEFAULTS is therefore empty.

Two-phase latch protocol:
  Phase 1: snapshot_and_reset() while the caller holds the shared lock.
  Phase 2: write_snapshot(snap, tick_id) after lock release, on main thread.

Observability:
  write_snapshot() emits only, via XyzInputAdapter helpers:
    - ros_in when received_count > 0
    - input_state every tick
  This adapter never emits ros_out/ros_result.
"""
import threading

import rospy
from py_trees.common import Access

from xyz_bt_lib.blackboard.blackboard_keys import BB
from xyz_bt_lib.core.base_adapter import XyzInputAdapter
from xyz_bt_lib.msg import DepthNavState


class DepthNavAdapter(XyzInputAdapter):
    """Adapter: /depth_nav/state → navigation geometry → /latched/* (per tick)."""

    ROS_TOPIC    = '/depth_nav/state'
    ADAPTER_NAME = 'DepthNavAdapter'
    BB_READS     = []
    BB_WRITES    = [
        BB.NAV_U9,
        BB.NAV_U10,
        BB.NAV_CX,
        BB.NAV_FX,
        BB.NAV_START_BX,
        BB.NAV_START_BY,
        BB.NAV_HAS_PATH,
        BB.NAV_OBSTACLE_NEAR,
        BB.NAV_OBSTACLE_DETECTED,
        BB.NAV_IMU_HEADING,
        BB.NAV_IMU_HEADING_MIN,
        BB.NAV_IMU_HEADING_MAX,
    ]
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {}

    def __init__(self, lock: threading.Lock, logger=None, tick_id_getter=None):
        """
        Args:
            lock:           Shared threading.Lock (same as bt_node.lock).
            logger:         DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.
        """
        super().__init__(lock=lock, logger=logger, tick_id_getter=tick_id_getter)

        _nan = float('nan')
        # Live (async) state — written only by _callback under self._lock.
        self._live_nav_u9            = _nan
        self._live_nav_u10           = _nan
        self._live_nav_cx            = 0.0
        self._live_nav_fx            = 0.0
        self._live_nav_start_bx      = 0
        self._live_nav_start_by      = 0
        self._live_nav_has_path      = False
        self._live_nav_obstacle_near = False
        self._live_nav_obstacle_detected = False
        self._live_nav_imu_heading   = 0.0
        self._live_nav_imu_heading_min = 0.0
        self._live_nav_imu_heading_max = 0.0

        # Latched BB client — written only by write_snapshot() (main thread).
        self._bb = self.make_latched_bb_client(name=self.ADAPTER_NAME)
        self._bb.register_key(key=BB.NAV_U9_KEY,            access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_U10_KEY,           access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_CX_KEY,            access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_FX_KEY,            access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_START_BX_KEY,      access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_START_BY_KEY,      access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_HAS_PATH_KEY,      access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_OBSTACLE_NEAR_KEY, access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_OBSTACLE_DETECTED_KEY, access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_IMU_HEADING_KEY,   access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_IMU_HEADING_MIN_KEY, access=Access.WRITE)
        self._bb.register_key(key=BB.NAV_IMU_HEADING_MAX_KEY, access=Access.WRITE)
        # Initialise BB before first tick so BT nodes never hit KeyError.
        self._bb.nav_u9            = _nan
        self._bb.nav_u10           = _nan
        self._bb.nav_cx            = 0.0
        self._bb.nav_fx            = 0.0
        self._bb.nav_start_bx      = 0
        self._bb.nav_start_by      = 0
        self._bb.nav_has_path      = False
        self._bb.nav_obstacle_near = False
        self._bb.nav_obstacle_detected = False
        self._bb.nav_imu_heading   = 0.0
        self._bb.nav_imu_heading_min = 0.0
        self._bb.nav_imu_heading_max = 0.0

        rospy.Subscriber(self.ROS_TOPIC, DepthNavState, self._callback)

    # ------------------------------------------------------------------
    # Side-effect-free conversion helpers
    # ------------------------------------------------------------------

    def _extract_state(self, msg: DepthNavState) -> dict:
        """Return the six scalar fields off the message.

        No BB writes, ROS calls, logger calls, or live-state mutation here. No angle
        math — u9/u10 are passed through (NaN already encodes "no path / row missing").
        """
        return {
            'nav_u9':        float(msg.nav_u9),
            'nav_u10':       float(msg.nav_u10),
            'nav_cx':        float(msg.cx),
            'nav_fx':        float(msg.fx),
            'nav_start_bx':  int(msg.start_bx),
            'nav_start_by':  int(msg.start_by),
            'has_path':      bool(msg.has_path),
            'obstacle_near': bool(msg.obstacle_near),
            'obstacle_detected': bool(msg.obstacle_detected),
            'nav_imu_heading': float(msg.imu_heading),
            'nav_imu_heading_min': float(msg.imu_heading_min),
            'nav_imu_heading_max': float(msg.imu_heading_max),
        }

    def _make_bb_writes(self, extracted: dict) -> dict:
        """Return {BB.KEY: value} for the six latched values.

        No BB reads/writes, ROS calls, or live-state mutation here.
        """
        return {
            BB.NAV_U9:            extracted['nav_u9'],
            BB.NAV_U10:           extracted['nav_u10'],
            BB.NAV_CX:            extracted['nav_cx'],
            BB.NAV_FX:            extracted['nav_fx'],
            BB.NAV_START_BX:      extracted['nav_start_bx'],
            BB.NAV_START_BY:      extracted['nav_start_by'],
            BB.NAV_HAS_PATH:      extracted['has_path'],
            BB.NAV_OBSTACLE_NEAR: extracted['obstacle_near'],
            BB.NAV_OBSTACLE_DETECTED: extracted['obstacle_detected'],
            BB.NAV_IMU_HEADING:   extracted['nav_imu_heading'],
            BB.NAV_IMU_HEADING_MIN: extracted['nav_imu_heading_min'],
            BB.NAV_IMU_HEADING_MAX: extracted['nav_imu_heading_max'],
        }

    def _apply_live_writes(self, bb_writes: dict) -> None:
        """Update live-state fields from bb_writes.

        Called by _callback() while holding self._lock.
        """
        self._live_nav_u9            = bb_writes[BB.NAV_U9]
        self._live_nav_u10           = bb_writes[BB.NAV_U10]
        self._live_nav_cx            = bb_writes[BB.NAV_CX]
        self._live_nav_fx            = bb_writes[BB.NAV_FX]
        self._live_nav_start_bx      = bb_writes[BB.NAV_START_BX]
        self._live_nav_start_by      = bb_writes[BB.NAV_START_BY]
        self._live_nav_has_path      = bb_writes[BB.NAV_HAS_PATH]
        self._live_nav_obstacle_near = bb_writes[BB.NAV_OBSTACLE_NEAR]
        self._live_nav_obstacle_detected = bb_writes[BB.NAV_OBSTACLE_DETECTED]
        self._live_nav_imu_heading   = bb_writes[BB.NAV_IMU_HEADING]
        self._live_nav_imu_heading_min = bb_writes[BB.NAV_IMU_HEADING_MIN]
        self._live_nav_imu_heading_max = bb_writes[BB.NAV_IMU_HEADING_MAX]

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg: DepthNavState) -> None:
        """Receive nav geometry, extract, update live state under the shared lock."""
        extracted = self._extract_state(msg)
        bb_writes = self._make_bb_writes(extracted)

        with self._lock:
            self._apply_live_writes(bb_writes)
            self._received_count += 1

    # ------------------------------------------------------------------
    # Public two-phase latch API
    # ------------------------------------------------------------------

    def snapshot_and_reset(self) -> dict:
        """Return current live-state snapshot and reset received_count.

        Must be called while the caller holds self._lock.
        """
        snap = {
            'nav_u9':            self._live_nav_u9,
            'nav_u10':           self._live_nav_u10,
            'nav_cx':            self._live_nav_cx,
            'nav_fx':            self._live_nav_fx,
            'nav_start_bx':      self._live_nav_start_bx,
            'nav_start_by':      self._live_nav_start_by,
            'nav_has_path':      self._live_nav_has_path,
            'nav_obstacle_near': self._live_nav_obstacle_near,
            'nav_obstacle_detected': self._live_nav_obstacle_detected,
            'nav_imu_heading':   self._live_nav_imu_heading,
            'nav_imu_heading_min': self._live_nav_imu_heading_min,
            'nav_imu_heading_max': self._live_nav_imu_heading_max,
            'received_count':    self._received_count,
        }
        self._received_count = 0
        return snap

    def write_snapshot(self, snap: dict, tick_id: int) -> None:
        """Write pre-captured snapshot to BB and emit business-in events.

        No lock is needed; called only from the main thread after the shared
        lock has been released.
        """
        self.emit_ros_in(tick_id=tick_id, received_count=snap['received_count'])

        # Write all BB keys before input_state so logs reflect what BT reads.
        self._bb.nav_u9            = snap['nav_u9']
        self._bb.nav_u10           = snap['nav_u10']
        self._bb.nav_cx            = snap['nav_cx']
        self._bb.nav_fx            = snap['nav_fx']
        self._bb.nav_start_bx      = snap['nav_start_bx']
        self._bb.nav_start_by      = snap['nav_start_by']
        self._bb.nav_has_path      = snap['nav_has_path']
        self._bb.nav_obstacle_near = snap['nav_obstacle_near']
        self._bb.nav_obstacle_detected = snap['nav_obstacle_detected']
        self._bb.nav_imu_heading   = snap['nav_imu_heading']
        self._bb.nav_imu_heading_min = snap['nav_imu_heading_min']
        self._bb.nav_imu_heading_max = snap['nav_imu_heading_max']

        self.emit_input_state(
            tick_id=tick_id,
            bb_writes={
                BB.NAV_U9:            snap['nav_u9'],
                BB.NAV_U10:           snap['nav_u10'],
                BB.NAV_CX:            snap['nav_cx'],
                BB.NAV_FX:            snap['nav_fx'],
                BB.NAV_START_BX:      snap['nav_start_bx'],
                BB.NAV_START_BY:      snap['nav_start_by'],
                BB.NAV_HAS_PATH:      snap['nav_has_path'],
                BB.NAV_OBSTACLE_NEAR: snap['nav_obstacle_near'],
                BB.NAV_OBSTACLE_DETECTED: snap['nav_obstacle_detected'],
                BB.NAV_IMU_HEADING:   snap['nav_imu_heading'],
                BB.NAV_IMU_HEADING_MIN: snap['nav_imu_heading_min'],
                BB.NAV_IMU_HEADING_MAX: snap['nav_imu_heading_max'],
            },
        )
