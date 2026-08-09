#!/usr/bin/env python3
# encoding: utf-8
"""ObjectDetectionAdapter — /object/pixel_coords → /latched/tracked_objects

Input adapter.

ROS topic subscribed:
  /object/pixel_coords

ROS message type:
  ainex_interfaces/ObjectsInfo

BB keys written:
  BB.TRACKED_OBJECTS  (/latched/tracked_objects)

BB value schema — TRACKED_OBJECTS is a dict[target_id, stat_dict]:

  {
    'ball': {
        'label':      str,    # ObjectInfo.label
        'shape':      str,    # ObjectInfo.type ('circle' | 'rect')
        'x':          float,  # raw pixel x (uncalibrated)
        'y':          float,  # raw pixel y (uncalibrated)
        'error_x':    float,  # obj.x − image_center_x (with center_x_offset)
        'error_y':    float,  # obj.y − image_center_y (with center_y_offset)
        'size':       float,  # π * radius² for circle; width * height for rect
        'lost_count': int,    # consecutive camera frames without detection (0 = seen this frame)
        # Depth fields (always present; None when depth disabled or unavailable):
        'depth_mm':   float | None,  # sampled depth at (x, y), millimetres
        'depth_m':    float | None,  # sampled depth at (x, y), metres
        'cam_x_m':    float | None,  # 3-D X in camera frame, metres
        'cam_y_m':    float | None,  # 3-D Y in camera frame, metres
        'cam_z_m':    float | None,  # 3-D Z in camera frame (= depth_m), metres
    },
    # 'hoop': { ... }  ← second target if target_specs configured for it
  }

  Default value: {}  (empty dict = nothing tracked).
  Missing key = target not detected / never seen.
  Lost target: key present, lost_count > 0, other fields preserved from last detection.

Per-BB-value traceability:
  /object/pixel_coords.data  (list of ObjectInfo)
    → _extract_detections()  : msg.data as plain list
    → _match_targets()       : {target_id: ObjectInfo | None} per target_spec
    → _build_entry()         : {label, shape, x, y, error_x, error_y, size} for matched obj
    → _compute_size()        : π*radius² (circle, uses obj.radius) | w*h (rect)
    → _compute_errors()      : (error_x, error_y) calibrated by center_x/y_offset
    → _apply_live_writes()   : merges entries + lost_counts into _live_tracked_objects
    → snapshot_and_reset()   : snapshots _live_tracked_objects
    → write_snapshot()       : writes BB.TRACKED_OBJECTS

Threshold/calibration source:
  CONFIG_DEFAULTS: image_width=640, image_height=480,
                   center_x_offset=0.0, center_y_offset=0.0
  center_x/y_offset: shifts calibrated image centre (mirrors LineDetectionAdapter pattern).

ObjectInfo field mapping:
  label       — object class name
  type        — 'circle' or 'rect'
  x, y        — center pixel coordinates
  width, height — bounding box dimensions
  radius      — circle radius (pixels); use directly for circle size (NOT estimated from w/h)

lost_count semantics:
  Per-target counter maintained in live state.
  Increments each camera frame when no matching ObjectInfo is found.
  Resets to 0 when a match is found.
  NOT reset in snapshot_and_reset() — it is sticky state that persists across ticks.
  L1_Vision_IsObjectDetected uses lost_count <= lost_count_threshold (default 0) as the detection condition.

Two-phase latch protocol:
  Phase 1: snapshot_and_reset() while caller holds the shared lock.
  Phase 2: write_snapshot(snap, tick_id) after lock release, on main thread.

Observability:
  write_snapshot() emits only, via XyzInputAdapter helpers:
    - ros_in when received_count > 0
    - input_state every tick
  This adapter never emits ros_out/ros_result.
"""
import math
import threading

import rospy
from py_trees.common import Access
from ainex_interfaces.msg import ObjectsInfo

from xyz_bt_lib.blackboard.blackboard_keys import BB
from xyz_bt_lib.core.base_adapter import DepthSampler, XyzInputAdapter

_DEFAULT_TARGET_SPECS = {'object': {'label': None, 'shape': None}}


class ObjectDetectionAdapter(XyzInputAdapter):
    """Adapter: /object/pixel_coords → tracked_objects dict → /latched/tracked_objects."""

    ROS_TOPIC    = '/object/pixel_coords'
    ADAPTER_NAME = 'ObjectDetectionAdapter'
    BB_READS     = []
    BB_WRITES    = [BB.TRACKED_OBJECTS, BB.DETECTION_SOURCE]
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'image_width':      640,
        'image_height':     480,
        'center_x_offset':  0.0,
        'center_y_offset':  0.0,
    }

    def __init__(self, lock: threading.Lock, logger=None, tick_id_getter=None,
                 target_specs: dict = None,
                 image_width: int = 640,
                 image_height: int = 480,
                 center_x_offset: float = 0.0,
                 center_y_offset: float = 0.0,
                 enable_depth: bool = False):
        """
        Args:
            lock:             Shared threading.Lock (same as bt_node.lock).
            logger:           DebugEventLogger-compatible object, or None.
            tick_id_getter:   Callable returning current tick_id.
            target_specs:     {target_id: FilterSpec} where
                              FilterSpec = {'label': str|None, 'shape': str|None}.
                              None = any. Default: {'object': {'label': None, 'shape': None}}.
            image_width:      Camera image width in pixels.
            image_height:     Camera image height in pixels.
            center_x_offset:  Calibration offset for image centre X (pixels).
            center_y_offset:  Calibration offset for image centre Y (pixels).
            enable_depth:     When True, subscribe to /camera/depth/* and populate
                              depth fields (depth_mm, depth_m, cam_x/y/z_m) in each
                              tracked_objects entry.  Default False for backward compat.
        """
        super().__init__(lock=lock, logger=logger, tick_id_getter=tick_id_getter)

        self._target_specs     = target_specs if target_specs is not None else dict(_DEFAULT_TARGET_SPECS)
        self._image_width      = image_width
        self._image_height     = image_height
        self._center_x_offset  = center_x_offset
        self._center_y_offset  = center_y_offset
        self._depth_sampler    = DepthSampler() if enable_depth else None

        # Live (async) state — written only by _apply_live_writes() under self._lock.
        self._live_tracked_objects = {}   # dict[target_id, stat_dict]

        # Latched BB client — written only by write_snapshot() (main thread).
        self._bb = self.make_latched_bb_client(name=self.ADAPTER_NAME)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY,  access=Access.WRITE)
        self._bb.register_key(key=BB.DETECTION_SOURCE_KEY, access=Access.WRITE)
        # Initialise BB before first tick so BT nodes never hit KeyError.
        self._bb.tracked_objects  = {}
        self._bb.detection_source = 'color'

        rospy.Subscriber(self.ROS_TOPIC, ObjectsInfo, self._callback)

    # ------------------------------------------------------------------
    # Side-effect-free conversion helpers
    # ------------------------------------------------------------------

    def _extract_detections(self, msg: ObjectsInfo) -> list:
        """Return msg.data as a plain list of ObjectInfo.

        No BB writes, ROS calls, logger calls, or live-state mutation here.
        """
        return list(msg.data)

    def _match_targets(self, objects: list) -> dict:
        """Return {target_id: ObjectInfo | None} for each target_spec entry.

        For each target_id, finds the first ObjectInfo in objects matching the
        spec (label and shape filters; None = any). Returns None for unmatched.
        No BB reads/writes, ROS calls, or live-state mutation here.
        """
        result = {}
        for target_id, spec in self._target_specs.items():
            result[target_id] = None
            for obj in objects:
                if self._match_spec(obj, spec):
                    result[target_id] = obj
                    break
        return result

    def _match_spec(self, obj, spec: dict) -> bool:
        """Return True if obj matches the FilterSpec (label and shape).

        None in spec means 'any'. No BB/ROS/facade calls here.
        """
        label_ok = spec.get('label') is None or obj.label == spec['label']
        shape_ok = spec.get('shape') is None or obj.type  == spec['shape']
        return label_ok and shape_ok

    def _build_entry(self, obj) -> dict:
        """Build a stat_dict for one matched ObjectInfo (without lost_count).

        No BB reads/writes, ROS calls, or live-state mutation here.
        Depth fields (depth_mm, depth_m, cam_x/y/z_m) are always present;
        values are None when enable_depth=False or depth is unavailable.
        """
        error_x, error_y = self._compute_errors(obj)
        entry = {
            'label':   obj.label,
            'shape':   obj.type,
            'x':       float(obj.x),
            'y':       float(obj.y),
            'error_x': error_x,
            'error_y': error_y,
            'size':    self._compute_size(obj),
        }
        if self._depth_sampler is not None:
            entry.update(self._depth_sampler.sample_depth(entry['x'], entry['y']))
        else:
            entry.update(DepthSampler._NONE_FIELDS)
        return entry

    def _compute_size(self, obj) -> float:
        """Return standardised size for one detected object.

        circle: π * radius²  (uses ObjectInfo.radius field directly)
        rect  : width * height
        No BB/ROS/facade calls here.
        """
        if obj.type == 'circle':
            return math.pi * float(obj.radius) * float(obj.radius)
        return float(obj.width * obj.height)

    def _compute_errors(self, obj) -> tuple:
        """Return (error_x, error_y) calibrated to image centre.

        cx = image_width / 2 + center_x_offset
        cy = image_height / 2 + center_y_offset
        error_x = obj.x - cx
        error_y = obj.y - cy
        No BB/ROS/facade calls here.
        """
        cx = self._image_width  / 2.0 + self._center_x_offset
        cy = self._image_height / 2.0 + self._center_y_offset
        return float(obj.x) - cx, float(obj.y) - cy

    def _apply_live_writes(self, matched: dict, built_entries: dict) -> None:
        """Merge new detections with existing lost_count state.

        Called by _callback() while holding self._lock.
        For found targets: create/update entry with lost_count=0.
        For lost targets: preserve last known entry, increment lost_count.
        For never-seen targets: omit from dict (missing key = never detected).
        """
        new_tracked = {}
        for target_id in self._target_specs:
            if target_id in built_entries:
                entry = dict(built_entries[target_id])
                entry['lost_count'] = 0
                new_tracked[target_id] = entry
            else:
                old = self._live_tracked_objects.get(target_id)
                if old is not None:
                    new_tracked[target_id] = dict(old, lost_count=old['lost_count'] + 1)
                # else: never seen — omit (missing key is canonical absence)
        self._live_tracked_objects = new_tracked

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg: ObjectsInfo) -> None:
        """Receive detection message, match targets, update live tracked state."""
        objects       = self._extract_detections(msg)
        matched       = self._match_targets(objects)
        built_entries = {tid: self._build_entry(obj)
                         for tid, obj in matched.items()
                         if obj is not None}

        with self._lock:
            self._apply_live_writes(matched, built_entries)
            self._received_count += 1

    # ------------------------------------------------------------------
    # Public two-phase latch API
    # ------------------------------------------------------------------

    def snapshot_and_reset(self) -> dict:
        """Return current live-state snapshot and reset received_count.

        lost_count is NOT reset — it is sticky state that persists across ticks.
        Must be called while the caller holds self._lock.
        """
        snap = {
            'tracked_objects': self._live_tracked_objects,
            'received_count':  self._received_count,
        }
        self._received_count = 0
        return snap

    def write_snapshot(self, snap: dict, tick_id: int) -> None:
        """Write pre-captured snapshot to BB and emit business-in events.

        No lock is needed; called only from the main thread after the shared
        lock has been released.
        """
        self.emit_ros_in(tick_id=tick_id, received_count=snap['received_count'])

        self._bb.tracked_objects = snap['tracked_objects']

        # Serialize for the log (omit raw pixel coords, keep key fields).
        tracked_log = {}
        for tid, e in snap['tracked_objects'].items():
            entry_log = {'shape': e['shape'], 'size': round(e['size'], 1),
                         'error_x': round(e['error_x'], 1), 'lost_count': e['lost_count']}
            if e.get('depth_m') is not None:
                entry_log['depth_m'] = round(e['depth_m'], 3)
            tracked_log[tid] = entry_log
        self.emit_input_state(
            tick_id=tick_id,
            bb_writes={BB.TRACKED_OBJECTS: tracked_log},
        )
