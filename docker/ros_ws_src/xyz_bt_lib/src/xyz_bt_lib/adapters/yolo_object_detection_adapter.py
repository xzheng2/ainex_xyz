#!/usr/bin/env python3
# encoding: utf-8
# MIGRATION NOTE: renamed from object_detection_adapter.ObjectDetectionAdapter
# (v2.7.0). Projects using YOLO detections should import YoloObjectDetectionAdapter
# from this file going forward. object_detection_adapter.py now provides the
# OpenCV /object/pixel_coords adapter.
"""YoloObjectDetectionAdapter — /yolo/detections → /latched/{tracked_objects, detection_source}

Input adapter.

ROS topic subscribed:
  /yolo/detections

ROS message type:
  ainex_interfaces/ObjectsInfo  (~10–15 Hz at 320×320, YOLO26n NCNN backend)

BB keys written:
  BB.TRACKED_OBJECTS   (/latched/tracked_objects)   — enriched dict[target_id, stat_dict]
  BB.DETECTION_SOURCE  (/latched/detection_source)  — str, always 'yolo' (written once at init)

BB.TRACKED_OBJECTS schema — same as ObjectDetectionAdapter:
  {
    'ball': {
        'label':      str,    # ObjectInfo.label
        'shape':      str,    # always 'rect' for YOLO
        'x':          float,  # raw pixel x (uncalibrated)
        'y':          float,  # raw pixel y (uncalibrated)
        'error_x':    float,  # obj.x − image_center_x (with center_x_offset)
        'error_y':    float,  # obj.y − image_center_y (with center_y_offset)
        'size':       float,  # width * height (YOLO always rect)
        'lost_count': int,    # consecutive frames without detection (0 = seen this frame)
        # Depth fields (always present; None when depth disabled or unavailable):
        'depth_mm':   float | None,  # sampled depth at (x, y), millimetres
        'depth_m':    float | None,  # sampled depth at (x, y), metres
        'cam_x_m':    float | None,  # 3-D X in camera frame, metres
        'cam_y_m':    float | None,  # 3-D Y in camera frame, metres
        'cam_z_m':    float | None,  # 3-D Z in camera frame (= depth_m), metres
        # 4 ROI-point depths (always present; sub-fields None when depth off/unavailable).
        # Each point = roi_fraction of the half-extent from centre toward that edge.
        'roi_depths': {
            'top':    {'depth_mm', 'depth_m', 'cam_x_m', 'cam_y_m', 'cam_z_m'},
            'bottom': {...},  # same sub-dict shape
            'left':   {...},
            'right':  {...},
        },
    },
    # 'hoop': { ... }  ← second target if target_specs configured for it
  }

Multi-instance targets (FilterSpec 'multi': True):
  By default each target spec yields ONE entry (the first match). A spec with
  'multi': True instead publishes EVERY matching detection as separate indexed
  keys '{target_id}_1'..'{target_id}_N', e.g. a 'goalpost' multi spec →
  tracked_objects = {'goalpost_1': {...}, 'goalpost_2': {...}}. Order is set by
  spec 'sort' ('x' asc | 'depth' asc nearest-first | 'size' desc; default 'x')
  and capped by spec 'max_instances'. Each indexed entry has the same schema as a
  single entry, so existing single-object consumers and the log path are unaffected.

Per-BB-value traceability:
  /yolo/detections.data  (list of ObjectInfo)
    → _extract_detections()  : msg.data as-is (list of ObjectInfo)
    → _resolve_entries()     : {bb_key: entry} per spec (single or _1..N multi)
    → _build_entry()         : {label, shape, x, y, error_x, error_y, size}
    → _compute_errors()      : (error_x, error_y) calibrated by center_x/y_offset
    → _sort_entries()        : orders multi matches (x / depth / size)
    → _merge_carry()         : merges entries + lost_counts into _live_tracked_objects
    → snapshot_and_reset()   : snapshots _live_tracked_objects
    → write_snapshot()       : writes BB.TRACKED_OBJECTS

Threshold/calibration source:
  CONFIG_DEFAULTS: image_width=640, image_height=480,
                   center_x_offset=0.0, center_y_offset=0.0, roi_fraction=0.5
  target_specs: dict param in __init__ (not in CONFIG_DEFAULTS — same as
                ObjectDetectionAdapter); default {'object': {'label': None, 'shape': None}}

lost_count semantics (same as ObjectDetectionAdapter):
  Per-target counter maintained in live state.
  Increments each camera frame when no matching ObjectInfo is found.
  Resets to 0 when a match is found.
  Never-seen targets are absent from the dict (missing key = never detected).
  Last known position is preserved while lost_count > 0.
  Multi-instance specs ('multi': True) do NOT carry lost_count: each frame the
  current instances are published fresh (lost_count=0) and absent indices are
  dropped (no per-object track IDs to persist positional identity across frames).

YOLO ObjectInfo field mapping:
  label  — COCO class name (e.g. 'person', 'ball')
  type   — always 'rect' for YOLO detections
  x, y   — center pixel coordinates
  width, height — bounding box

Notes:
  - /yolo/detections is published by yolo_camera.py on the host via roslibpy
    → rosbridge_server (port 9090). It is a separate topic from
    /object/pixel_coords (handled by ObjectDetectionAdapter).
  - When no objects are detected the message is published with an empty
    data list (not dropped), so tracked_objects reflects lost_count increments.

Two-phase latch protocol:
  Phase 1: snapshot_and_reset() while caller holds the shared lock.
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
from ainex_interfaces.msg import ObjectsInfo

from xyz_bt_lib.blackboard.blackboard_keys import BB
from xyz_bt_lib.core.base_adapter import DepthSampler, XyzInputAdapter

_DEFAULT_TARGET_SPECS = {'object': {'label': None, 'shape': None}}


class YoloObjectDetectionAdapter(XyzInputAdapter):
    """Adapter: /yolo/detections → tracked_objects → /latched/* (per tick)."""

    ROS_TOPIC    = '/yolo/detections'
    ADAPTER_NAME = 'YoloObjectDetectionAdapter'
    BB_READS     = []
    BB_WRITES    = [BB.TRACKED_OBJECTS, BB.DETECTION_SOURCE]
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {
        'image_width':     640,
        'image_height':    480,
        'center_x_offset': 30.0,
        'center_y_offset': 0.0,
        'roi_fraction':    0.5,
    }

    def __init__(self, lock: threading.Lock, logger=None, tick_id_getter=None,
                 target_specs: dict = None,
                 image_width: int = 640,
                 image_height: int = 480,
                 center_x_offset: float = 30.0,
                 center_y_offset: float = 0.0,
                 enable_depth: bool = False,
                 depth_window_radius: int = 4,
                 roi_fraction: float = 0.5):
        """
        Args:
            lock:             Shared threading.Lock (same as bt_node.lock).
            logger:           DebugEventLogger-compatible object, or None.
            tick_id_getter:   Callable returning current tick_id.
            target_specs:     {target_id: FilterSpec} where
                              FilterSpec = {'label': str|None, 'shape': str|None,
                                            'multi': bool (optional),
                                            'max_instances': int|None (optional),
                                            'sort': 'x'|'depth'|'size' (optional)}.
                              label/shape None = any. Default:
                              {'object': {'label': None, 'shape': None}}.
                              With 'multi': True, ALL matches are published as
                              '{target_id}_1'..N (sorted by 'sort', default 'x';
                              capped by 'max_instances'). Without 'multi', a single
                              entry is published under target_id (backward compatible).
            image_width:      Camera image width in pixels (default 640).
            image_height:     Camera image height in pixels (default 480).
            center_x_offset:  Calibration offset for image centre X in pixels (default 0.0).
            center_y_offset:  Calibration offset for image centre Y in pixels (default 0.0).
            enable_depth:     When True, subscribe to /camera/depth/* and populate
                              depth fields (depth_mm, depth_m, cam_x/y/z_m) in each
                              tracked_objects entry.  Default False for backward compat.
            depth_window_radius: Half-size of the square window (in pixels) over
                              which depth is median-sampled at the detection centre.
                              Larger = more hole-resistant, less local. Default 4
                              (9x9 window). Only used when enable_depth=True.
            roi_fraction:     Fraction of the bounding-box half-extent at which the
                              4 ROI sample points sit, measured from the box centre
                              toward each edge (top/bottom/left/right). 0.5 = middle
                              (default), 1.0 = on the edge, 0.0 = all at centre.
                              Only used when enable_depth=True.
        """
        super().__init__(lock=lock, logger=logger, tick_id_getter=tick_id_getter)

        self._target_specs    = target_specs if target_specs is not None else dict(_DEFAULT_TARGET_SPECS)
        self._image_width     = image_width
        self._image_height    = image_height
        self._center_x_offset = center_x_offset
        self._center_y_offset = center_y_offset
        self._roi_fraction    = roi_fraction
        self._depth_sampler   = DepthSampler(window_radius=depth_window_radius) if enable_depth else None

        # Live (async) state — written only by _callback under self._lock.
        self._live_tracked_objects = {}   # dict[target_id, stat_dict]

        # Latched BB client — written only by write_snapshot() (main thread).
        self._bb = self.make_latched_bb_client(name=self.ADAPTER_NAME)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY,  access=Access.WRITE)
        self._bb.register_key(key=BB.DETECTION_SOURCE_KEY, access=Access.WRITE)
        # Initialise BB before first tick so BT nodes never hit KeyError.
        self._bb.tracked_objects  = {}
        self._bb.detection_source = 'yolo'

        rospy.Subscriber(self.ROS_TOPIC, ObjectsInfo, self._callback)

    # ------------------------------------------------------------------
    # Side-effect-free conversion helpers — tracked_objects path
    # ------------------------------------------------------------------

    def _extract_detections(self, msg: ObjectsInfo) -> list:
        """Return msg.data as a plain list of ObjectInfo.

        No BB writes, ROS calls, logger calls, or live-state mutation here.
        """
        return list(msg.data)

    def _resolve_entries(self, objects: list) -> dict:
        """Return {bb_key: entry} of this-frame detections (lost_count=0).

        Single-instance spec (default): {target_id: entry} for the FIRST match;
        the key is omitted when there is no match.
        Multi-instance spec ('multi': True): EVERY match becomes a separate entry
        keyed '{target_id}_1'..'{target_id}_N', ordered by spec 'sort' ('x' asc |
        'depth' asc | 'size' desc; default 'x') and capped at spec 'max_instances'
        when set. No BB reads/writes, ROS calls, or live-state mutation here.
        """
        result = {}
        for target_id, spec in self._target_specs.items():
            matches = [o for o in objects if self._match_spec(o, spec)]
            if spec.get('multi'):
                entries = self._sort_entries(
                    [self._build_entry(o) for o in matches], spec.get('sort', 'x'))
                max_instances = spec.get('max_instances')
                if max_instances is not None:
                    entries = entries[:max_instances]
                for i, entry in enumerate(entries, 1):
                    entry['lost_count'] = 0
                    result['{}_{}'.format(target_id, i)] = entry
            elif matches:
                entry = self._build_entry(matches[0])
                entry['lost_count'] = 0
                result[target_id] = entry
        return result

    def _sort_entries(self, entries: list, sort: str) -> list:
        """Return entries ordered per `sort`. Side-effect-free.

        'x'     → ascending image x (left → right)
        'depth' → ascending depth_mm (nearest first); None depth sorts last
        'size'  → descending size (largest first)
        anything else → original order preserved.
        """
        if sort == 'depth':
            return sorted(entries, key=lambda e: (e.get('depth_mm') is None,
                                                  e.get('depth_mm') or 0.0))
        if sort == 'size':
            return sorted(entries, key=lambda e: e.get('size', 0.0), reverse=True)
        if sort == 'x':
            return sorted(entries, key=lambda e: e.get('x', 0.0))
        return entries

    def _match_spec(self, obj, spec: dict) -> bool:
        """Return True if obj matches the FilterSpec (label and shape).

        None in spec means 'any'. No BB/ROS/facade calls here.
        """
        label_ok = spec.get('label') is None or obj.label == spec['label']
        shape_ok = spec.get('shape') is None or obj.type  == spec['shape']
        return label_ok and shape_ok

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

    def _build_entry(self, obj) -> dict:
        """Build a stat_dict for one matched ObjectInfo (without lost_count).

        YOLO always returns 'rect', so size = width * height.
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
            'size':    float(obj.width * obj.height),
        }
        if self._depth_sampler is not None:
            entry.update(self._depth_sampler.sample_depth(entry['x'], entry['y']))
        else:
            entry.update(DepthSampler._NONE_FIELDS)
        entry['roi_depths'] = self._sample_roi_depths(obj)
        return entry

    def _roi_points(self, obj) -> dict:
        """Return {name: (px, py)} for the 4 edge-direction ROI sample points.

        Each point lies roi_fraction of the bounding-box half-extent from the
        box centre toward an edge (top/bottom/left/right). roi_fraction=0.5 →
        quarter of the box from centre; 1.0 → on the edge; 0.0 → all coincide at
        centre. No BB reads/writes, ROS calls, or live-state mutation here.
        """
        cx, cy = float(obj.x), float(obj.y)
        dx = obj.width  / 2.0 * self._roi_fraction
        dy = obj.height / 2.0 * self._roi_fraction
        return {
            'top':    (cx,      cy - dy),
            'bottom': (cx,      cy + dy),
            'left':   (cx - dx, cy),
            'right':  (cx + dx, cy),
        }

    def _sample_roi_depths(self, obj) -> dict:
        """Return {name: sample_dict} for the 4 ROI points.

        Each value is the full sample_depth() dict (depth_mm, depth_m,
        cam_x/y/z_m); sub-fields are None where depth is unavailable (out of
        bounds, depth hole, intrinsics not yet received). When depth is disabled
        every point is the all-None field dict. No BB/ROS/live-state side effects.
        """
        if self._depth_sampler is None:
            return {n: dict(DepthSampler._NONE_FIELDS) for n in self._roi_points(obj)}
        return {n: self._depth_sampler.sample_depth(px, py)
                for n, (px, py) in self._roi_points(obj).items()}

    def _merge_carry(self, resolved: dict) -> None:
        """Merge this-frame resolved entries with existing lost_count state.

        Called by _callback() while holding self._lock.
        Single-instance specs:
          found → entry with lost_count=0;
          lost  → preserve last known entry, increment lost_count;
          never seen → omit (missing key = never detected).
        Multi-instance specs ('multi': True):
          publish only this-frame instances ('{target_id}_1'..N); absent indices
          are dropped. YOLO has no per-object track IDs, so cross-frame positional
          identity (and lost_count carry) is NOT maintained for multi specs.
        """
        new_tracked = {}
        for target_id, spec in self._target_specs.items():
            if spec.get('multi'):
                i = 1
                while '{}_{}'.format(target_id, i) in resolved:
                    key = '{}_{}'.format(target_id, i)
                    new_tracked[key] = resolved[key]
                    i += 1
            elif target_id in resolved:
                new_tracked[target_id] = resolved[target_id]
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
        """Receive YOLO detection message, extract objects, update live state."""
        objects  = self._extract_detections(msg)
        resolved = self._resolve_entries(objects)

        with self._lock:
            self._merge_carry(resolved)
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

        # Write to BB before input_state so logs reflect what BT will read.
        self._bb.tracked_objects = snap['tracked_objects']

        # Serialize tracked_objects for the log (omit raw pixel coords).
        tracked_log = {}
        for tid, e in snap['tracked_objects'].items():
            entry_log = {'shape': e['shape'], 'size': round(e['size'], 1),
                         'error_x': round(e['error_x'], 1), 'lost_count': e['lost_count']}
            if e.get('depth_m') is not None:
                entry_log['depth_m'] = round(e['depth_m'], 3)
            roi = e.get('roi_depths')
            if roi:
                roi_log = {n: (round(v['depth_m'], 3) if v.get('depth_m') is not None else None)
                           for n, v in roi.items()}
                if any(val is not None for val in roi_log.values()):
                    entry_log['roi_depth_m'] = roi_log
            tracked_log[tid] = entry_log
        self.emit_input_state(tick_id=tick_id, bb_writes={BB.TRACKED_OBJECTS: tracked_log})
