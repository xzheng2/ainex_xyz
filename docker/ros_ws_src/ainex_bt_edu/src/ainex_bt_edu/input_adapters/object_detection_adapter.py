#!/usr/bin/env python3
# encoding: utf-8
"""ObjectDetectionAdapter — /yolo/detections → /latched/{detected_objects,
detected_count}

Input adapter.

ROS topic subscribed:
  /yolo/detections

ROS message type:
  ainex_interfaces/ObjectsInfo  (~6–7 Hz at 320×320, YOLO26n NCNN backend)

BB keys written:
  BB.DETECTED_OBJECTS  (/latched/detected_objects)
  BB.DETECTED_COUNT    (/latched/detected_count)

Per-BB-value traceability:
  /yolo/detections.data  (list of ObjectInfo)
    → _extract_detections() : msg.data as-is (list of ObjectInfo)
    → _classify_detections(): counts objects; returns (objects_list, count);
                              no filtering — YOLO publishes only object
                              detections, never line data
    → _make_bb_writes()     : {BB.DETECTED_OBJECTS: objects_list,
                              BB.DETECTED_COUNT: count}
    → _apply_live_writes()  : updates self._live_detected_objects,
                              self._live_detected_count under self._lock
    → snapshot fields       : 'detected_objects', 'detected_count'
    → BB.DETECTED_OBJECTS, BB.DETECTED_COUNT

Threshold/calibration source:
  No thresholds. Object presence is determined purely by len(msg.data) > 0.
  Filtering by object type/label is the responsibility of L1 nodes that
  read BB.DETECTED_OBJECTS (e.g. L1_Vision_IsObjectDetected with object_type).

YOLO ObjectInfo field mapping:
  label  — COCO class name (e.g. 'cat', 'stop sign', 'bottle')
  type   — always 'rect' for YOLO detections
  x, y   — center pixel coordinates
  width, height — bounding box

Notes:
  - /yolo/detections is published by yolo_camera.py on the host via roslibpy
    → rosbridge_server (port 9090). It is a separate topic from
    /object/pixel_coords (handled by LineDetectionAdapter).
  - When no objects are detected the message is published with an empty
    data list (not dropped), so detected_count stays current.

Two-phase latch protocol:
  Phase 1: snapshot_and_reset() while caller holds the shared lock.
  Phase 2: write_snapshot(snap, tick_id) after lock release, on main thread.

Observability:
  write_snapshot() emits only, via AinexInputAdapter helpers:
    - ros_in when received_count > 0
    - input_state every tick
  This adapter never emits ros_out/ros_result.
"""
import threading

import rospy
from py_trees.common import Access
from ainex_interfaces.msg import ObjectsInfo

from ainex_bt_edu.blackboard_keys import BB
from ainex_bt_edu.base_adapter import AinexInputAdapter


class ObjectDetectionAdapter(AinexInputAdapter):
    """Adapter: /yolo/detections → object count/list → /latched/* (per tick)."""

    ROS_TOPIC    = '/yolo/detections'
    ADAPTER_NAME = 'ObjectDetectionAdapter'
    BB_READS     = []
    BB_WRITES    = [BB.DETECTED_OBJECTS, BB.DETECTED_COUNT]
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

        # Live (async) state — written only by _callback under self._lock.
        self._live_detected_objects = []   # list[ObjectInfo]
        self._live_detected_count   = 0    # int

        # Latched BB client — written only by write_snapshot() (main thread).
        self._bb = self.make_latched_bb_client(name=self.ADAPTER_NAME)
        self._bb.register_key(key=BB.DETECTED_OBJECTS_KEY, access=Access.WRITE)
        self._bb.register_key(key=BB.DETECTED_COUNT_KEY,   access=Access.WRITE)
        # Initialise BB before first tick so BT nodes never hit KeyError.
        self._bb.detected_objects = []
        self._bb.detected_count   = 0

        rospy.Subscriber(self.ROS_TOPIC, ObjectsInfo, self._callback)

    # ------------------------------------------------------------------
    # Side-effect-free conversion helpers
    # ------------------------------------------------------------------

    def _extract_detections(self, msg: ObjectsInfo) -> list:
        """Return msg.data as a plain list of ObjectInfo.

        No BB writes, ROS calls, logger calls, or live-state mutation here.
        """
        return list(msg.data)

    def _classify_detections(self, objects: list) -> tuple:
        """Count detections; return (objects_list, count).

        No filtering applied — YOLO detections are already separated from
        line data at the source. Filtering by label/type is left to L1 nodes.
        No BB reads/writes, ROS calls, or live-state mutation here.
        """
        return objects, len(objects)

    def _make_bb_writes(self, classified: tuple) -> dict:
        """Return {BB.DETECTED_OBJECTS: list, BB.DETECTED_COUNT: int}.

        No BB reads/writes, ROS calls, or live-state mutation here.
        """
        objects, count = classified
        return {
            BB.DETECTED_OBJECTS: objects,
            BB.DETECTED_COUNT:   count,
        }

    def _apply_live_writes(self, bb_writes: dict) -> None:
        """Copy computed values into live state.

        Called by _callback() while holding self._lock. This is the only helper
        in the conversion path that mutates live (shared) state.
        """
        self._live_detected_objects = bb_writes[BB.DETECTED_OBJECTS]
        self._live_detected_count   = bb_writes[BB.DETECTED_COUNT]

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg: ObjectsInfo) -> None:
        """Receive YOLO detection message, extract objects, update live state."""
        objects    = self._extract_detections(msg)
        classified = self._classify_detections(objects)
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
            'detected_objects': self._live_detected_objects,
            'detected_count':   self._live_detected_count,
            'received_count':   self._received_count,
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
        self._bb.detected_objects = snap['detected_objects']
        self._bb.detected_count   = snap['detected_count']

        # Serialize detected_objects for the log (list of label strings only).
        objects_log = [
            {'label': o.label, 'x': o.x, 'y': o.y}
            for o in snap['detected_objects']
        ]
        self.emit_input_state(
            tick_id=tick_id,
            bb_writes={
                BB.DETECTED_OBJECTS: objects_log,
                BB.DETECTED_COUNT:   snap['detected_count'],
            },
        )
