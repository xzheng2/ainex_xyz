#!/usr/bin/env python3
# encoding: utf-8
"""LineDetectionAdapter — /object/pixel_coords → /latched/{line_data,
last_line_x, camera_lost_count, line_error_x, line_center_x, last_line_error_x}

Input adapter.

ROS topic subscribed:
  /object/pixel_coords

ROS message type:
  ainex_interfaces/ObjectsInfo  (~30 Hz)

BB keys written:
  BB.LINE_DATA          (/latched/line_data)
  BB.LAST_LINE_X        (/latched/last_line_x)
  BB.CAMERA_LOST_COUNT  (/latched/camera_lost_count)
  BB.LINE_ERROR_X       (/latched/line_error_x)
  BB.LINE_CENTER_X      (/latched/line_center_x)
  BB.LAST_LINE_ERROR_X  (/latched/last_line_error_x)

Per-BB-value traceability:
  msg.data[*].type == 'line'
    → _extract_line()       : first ObjectInfo with type='line', or None
    → _classify_line()      : computes line_center_x = width/2 + center_x_offset
                              line_error_x = line_data.x - line_center_x
                              returns dict with line_data/line_center_x/
                              line_error_x/has_line; side-effect-free
    → _make_bb_writes()     : maps to {BB.LINE_DATA, BB.LINE_CENTER_X,
                              BB.LINE_ERROR_X} + 'has_line' flag
    → _apply_live_writes()  : writes all six live fields under self._lock;
                              sticky fields (last_line_x, last_line_error_x)
                              updated only when has_line=True;
                              camera_lost_count increments per camera frame
                              when has_line=False
    → snapshot fields       : line_data, last_line_x, camera_lost_count,
                              line_error_x, line_center_x, last_line_error_x
    → BB.LINE_DATA, BB.LAST_LINE_X, BB.CAMERA_LOST_COUNT,
      BB.LINE_ERROR_X, BB.LINE_CENTER_X, BB.LAST_LINE_ERROR_X

Threshold/calibration source:
  CONFIG_DEFAULTS:
    center_x_offset: 66  (YAML-backed; ainex_bt_edu/config/line_perception.yaml
                          is the single source of truth; falls back to 0 if
                          file is missing)

Notes:
  camera_lost_count counts consecutive camera frames (~30 Hz) without line
  detection, not BT ticks (~10 Hz). This is intentional — it preserves
  the original behavioural semantics.

Two-phase latch protocol:
  Phase 1: snapshot_and_reset() while caller holds the shared lock.
  Phase 2: write_snapshot(snap, tick_id) after lock release, on main thread.

Observability:
  write_snapshot() emits only, via AinexInputAdapter helpers:
    - ros_in when received_count > 0
    - input_state every tick
  This adapter never emits ros_out/ros_result.
"""
import os
import threading

import rospy
import rospkg
import yaml
from py_trees.common import Access
from ainex_interfaces.msg import ObjectsInfo

from ainex_bt_edu.blackboard_keys import BB
from ainex_bt_edu.base_adapter import AinexInputAdapter


def _load_line_perception_config() -> dict:
    try:
        pkg_path = rospkg.RosPack().get_path('ainex_bt_edu')
        cfg_path = os.path.join(pkg_path, 'config', 'line_perception.yaml')
        with open(cfg_path) as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        rospy.logwarn('[LineDetectionAdapter] failed to load line_perception.yaml: %s'
                      ' — using center_x_offset=0', e)
        return {}


class LineDetectionAdapter(AinexInputAdapter):
    """Adapter: /object/pixel_coords → line state → /latched/* (per tick)."""

    ROS_TOPIC    = '/object/pixel_coords'
    ADAPTER_NAME = 'LineDetectionAdapter'
    BB_READS     = []
    BB_WRITES    = [
        BB.LINE_DATA,
        BB.LAST_LINE_X,
        BB.CAMERA_LOST_COUNT,
        BB.LINE_ERROR_X,
        BB.LINE_CENTER_X,
        BB.LAST_LINE_ERROR_X,
    ]
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'center_x_offset': 66,   # loaded from line_perception.yaml at startup
    }

    def __init__(self, lock: threading.Lock, logger=None, tick_id_getter=None):
        """
        Args:
            lock:           Shared threading.Lock (same as bt_node.lock).
            logger:         DebugEventLogger-compatible object, or None.
            tick_id_getter: Callable returning current tick_id.

        center_x_offset is loaded from ainex_bt_edu/config/line_perception.yaml
        at startup (default 66). Constructor does not accept it as a parameter
        to ensure the robot uses the calibration file as the single source of
        truth.
        """
        super().__init__(lock=lock, logger=logger, tick_id_getter=tick_id_getter)

        # Calibration (loaded once at startup, read-only after __init__).
        _cfg = _load_line_perception_config()
        self._center_x_offset = float(_cfg.get('center_x_offset', 0))
        rospy.loginfo('[LineDetectionAdapter] center_x_offset=%.1f', self._center_x_offset)

        # Live (async) state — written only by _callback under self._lock.
        self._live_line_data         = None
        self._live_last_line_x       = None
        self._live_camera_lost_count = 0
        self._live_line_error_x      = None   # per-frame; None when line lost
        self._live_line_center_x     = None   # per-frame; None when line lost
        self._live_last_line_error_x = None   # sticky signed error (last seen)

        # Latched BB client — written only by write_snapshot() (main thread).
        self._bb = self.make_latched_bb_client(name=self.ADAPTER_NAME)
        self._bb.register_key(key=BB.LINE_DATA_KEY,          access=Access.WRITE)
        self._bb.register_key(key=BB.LAST_LINE_X_KEY,        access=Access.WRITE)
        self._bb.register_key(key=BB.CAMERA_LOST_COUNT_KEY,  access=Access.WRITE)
        self._bb.register_key(key=BB.LINE_ERROR_X_KEY,       access=Access.WRITE)
        self._bb.register_key(key=BB.LINE_CENTER_X_KEY,      access=Access.WRITE)
        self._bb.register_key(key=BB.LAST_LINE_ERROR_X_KEY,  access=Access.WRITE)
        # Initialise BB before first tick so BT nodes never hit KeyError.
        self._bb.line_data          = None
        self._bb.last_line_x        = None
        self._bb.camera_lost_count  = 0
        self._bb.line_error_x       = None
        self._bb.line_center_x      = None
        self._bb.last_line_error_x  = None

        rospy.Subscriber(self.ROS_TOPIC, ObjectsInfo, self._callback)

    # ------------------------------------------------------------------
    # Side-effect-free conversion helpers
    # ------------------------------------------------------------------

    def _extract_line(self, msg: ObjectsInfo):
        """Return the first ObjectInfo with type='line', or None.

        No BB writes, ROS calls, logger calls, or live-state mutation here.
        """
        for obj in msg.data:
            if obj.type == 'line':
                return obj
        return None

    def _classify_line(self, line_obj) -> dict:
        """Compute all derived line-detection values from a line object.

        Uses self._center_x_offset (read-only calibration constant set in
        __init__) — acceptable as a read of immutable init state.

        Returns dict with keys: line_data, line_center_x, line_error_x,
        has_line. Returns None for the float fields when line_obj is None.
        No BB reads/writes, ROS calls, or live-state mutation here.
        """
        if line_obj is None:
            return {
                'line_data':     None,
                'line_center_x': None,
                'line_error_x':  None,
                'has_line':      False,
            }
        line_center_x = line_obj.width / 2.0 + self._center_x_offset
        line_error_x  = line_obj.x - line_center_x
        return {
            'line_data':     line_obj,
            'line_center_x': line_center_x,
            'line_error_x':  line_error_x,
            'has_line':      True,
        }

    def _make_bb_writes(self, classified: dict) -> dict:
        """Return {BB.KEY: value} for the three non-sticky BB values.

        The sticky values (last_line_x, last_line_error_x) and the counter
        (camera_lost_count) require context of prior state and are handled
        by _apply_live_writes(). The 'has_line' control flag is passed through
        for _apply_live_writes() to act on.
        No BB reads/writes, ROS calls, or live-state mutation here.
        """
        return {
            BB.LINE_DATA:     classified['line_data'],
            BB.LINE_CENTER_X: classified['line_center_x'],
            BB.LINE_ERROR_X:  classified['line_error_x'],
            'has_line':       classified['has_line'],
        }

    def _apply_live_writes(self, bb_writes: dict) -> None:
        """Update all six live-state fields from bb_writes.

        Called by _callback() while holding self._lock. Handles sticky state
        (last_line_x, last_line_error_x) and the camera_lost_count counter
        which both require knowledge of whether a line was detected this frame.
        This is the only helper in the conversion path that mutates live state.
        """
        self._live_line_data     = bb_writes[BB.LINE_DATA]
        self._live_line_center_x = bb_writes[BB.LINE_CENTER_X]
        self._live_line_error_x  = bb_writes[BB.LINE_ERROR_X]
        if bb_writes['has_line']:
            line_obj = bb_writes[BB.LINE_DATA]
            self._live_last_line_x       = line_obj.x
            self._live_last_line_error_x = bb_writes[BB.LINE_ERROR_X]
            self._live_camera_lost_count = 0
        else:
            # camera_lost_count accumulates per camera frame (~30 Hz), not per
            # BT tick — intentional, matches original behaviour.
            self._live_camera_lost_count += 1

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg: ObjectsInfo) -> None:
        """Receive detection message, extract line, compute derived values,
        update live state."""
        line_obj   = self._extract_line(msg)
        classified = self._classify_line(line_obj)
        bb_writes  = self._make_bb_writes(classified)

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
            'line_data':          self._live_line_data,
            'last_line_x':        self._live_last_line_x,
            'camera_lost_count':  self._live_camera_lost_count,
            'line_error_x':       self._live_line_error_x,
            'line_center_x':      self._live_line_center_x,
            'last_line_error_x':  self._live_last_line_error_x,
            'received_count':     self._received_count,
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
        self._bb.line_data          = snap['line_data']
        self._bb.last_line_x        = snap['last_line_x']
        self._bb.camera_lost_count  = snap['camera_lost_count']
        self._bb.line_error_x       = snap['line_error_x']
        self._bb.line_center_x      = snap['line_center_x']
        self._bb.last_line_error_x  = snap['last_line_error_x']

        line_data = snap['line_data']
        line_data_log = (
            {'x': line_data.x, 'width': line_data.width}
            if line_data is not None else None
        )
        self.emit_input_state(
            tick_id=tick_id,
            bb_writes={
                BB.LINE_DATA:          line_data_log,
                BB.LAST_LINE_X:        snap['last_line_x'],
                BB.CAMERA_LOST_COUNT:  snap['camera_lost_count'],
                BB.LINE_ERROR_X:       snap['line_error_x'],
                BB.LINE_CENTER_X:      snap['line_center_x'],
                BB.LAST_LINE_ERROR_X:  snap['last_line_error_x'],
            },
        )
