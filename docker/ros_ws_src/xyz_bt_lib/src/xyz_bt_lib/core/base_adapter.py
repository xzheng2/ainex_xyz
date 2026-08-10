#!/usr/bin/env python3
"""Base class for AiNex observable input adapters.

Input adapters are inbound ROS components, not behaviour-tree nodes. They
subscribe to ROS input streams, maintain live state under a shared lock, and
expose a two-phase latch API used by the BT app loop:

1. ``snapshot_and_reset()`` while the caller holds the shared lock.
2. ``write_snapshot(snap, tick_id)`` after releasing the lock on the main thread.

Boundary rules:
  Inbound (ROS → BB): adapters own rospy.Subscriber and BB writes.
  Outbound (BT → ROS): all execution side-effects go via _RuntimeIO only.
                        Adapters must never call managers or publish ROS topics.
"""
from abc import ABC, abstractmethod
import math
import threading
import time

import py_trees

from xyz_bt_lib.blackboard.blackboard_keys import BB


def carry_displacement(new_entry: dict, old_entry) -> dict:
    """Add a ``displacement`` field to ``new_entry``, measured against ``old_entry``.

    ``displacement`` is the Euclidean pixel distance the target centroid moved
    since the PREVIOUS CONSECUTIVE detection of that target.

    This exists so that "is the object still?" can be answered by a PURE L1
    predicate. Motion is inherently a two-sample question, and an L1 condition
    node may not hold cross-tick state (see ``XyzL1ConditionNode``), so the
    cross-frame memory belongs in the tracking layer that already carries
    ``lost_count`` — here. The L1 node then only compares one latched number
    against a threshold, and "still for N ticks" is a tree-layer
    ``LatchedDwellDecorator``.

    Measured per CAMERA FRAME (adapter callback rate), not per BT tick: the
    latched value each tick is the most recent frame-to-frame displacement.

    Returns ``None`` when there is no comparable previous sample — first
    sighting, or re-acquisition after a loss. Comparing across a loss gap would
    report motion that may have happened at any point during the gap, so the
    honest answer is "unknown", and a pure predicate treats unknown as not-still
    (safe-fail, consistent with the RUNNING-is-not-passed rule in the dwell/
    hysteresis decorators).

    Args:
        new_entry: this frame's stat_dict; must contain ``x`` and ``y``.
        old_entry: the previous stat_dict for the same target, or None.

    Returns:
        ``new_entry``, mutated in place with ``displacement`` set.
    """
    if old_entry is not None and old_entry.get('lost_count') == 0:
        new_entry['displacement'] = math.hypot(
            new_entry['x'] - old_entry['x'],
            new_entry['y'] - old_entry['y'])
    else:
        new_entry['displacement'] = None
    return new_entry


class XyzInputAdapter(ABC):
    """Common base for sensor/input adapters.

    Subclasses own ROS subscription setup and live-state conversion. This base
    only stores shared observability dependencies and provides standard
    ``ros_in`` / ``input_state`` emit helpers.
    """

    ROS_TOPIC = None
    ADAPTER_NAME = None

    BB_READS = []
    BB_WRITES = []
    FACADE_CALLS = []
    CONFIG_DEFAULTS = {}

    def __init__(self, lock, logger=None, tick_id_getter=None):
        self._lock = lock
        self._logger = logger
        self._tick_id_getter = tick_id_getter or (lambda: -1)
        self._received_count = 0

    @property
    def tick_id(self):
        """Return the current externally-owned BT tick id."""
        return self._tick_id_getter()

    @property
    def adapter_name(self):
        return self.ADAPTER_NAME or type(self).__name__

    def make_latched_bb_client(self, name=None):
        """Create a blackboard client for writing /latched/* keys."""
        return py_trees.blackboard.Client(
            name=name or self.adapter_name,
            namespace=BB.LATCHED_NS,
        )

    def emit_comm(self, payload: dict) -> None:
        """Emit a communication-layer event if a logger was injected."""
        if self._logger is None:
            return
        event = dict(payload)
        event.setdefault('tick_id', self.tick_id)
        event.setdefault('adapter', self.adapter_name)
        self._logger.emit_comm(event)

    def emit_ros_in(self, *, tick_id=None, received_count=None,
                    source=None, **extra) -> None:
        """Emit one ros_in event when messages arrived in this latch window."""
        count = self._received_count if received_count is None else received_count
        if count <= 0:
            return
        event = {
            'event': 'ros_in',
            'tick_id': self.tick_id if tick_id is None else tick_id,
            'ts': time.time(),
            'source': source or self.ROS_TOPIC,
            'adapter': self.adapter_name,
            'received_count': count,
        }
        event.update(extra)
        self.emit_comm(event)

    def emit_input_state(self, *, tick_id=None, bb_writes=None, **extra) -> None:
        """Emit the per-tick input_state event after writing the blackboard."""
        event = {
            'event': 'input_state',
            'tick_id': self.tick_id if tick_id is None else tick_id,
            'ts': time.time(),
            'adapter': self.adapter_name,
            'bb_writes': bb_writes or {},
        }
        event.update(extra)
        self.emit_comm(event)

    @abstractmethod
    def snapshot_and_reset(self) -> dict:
        """Return a live-state snapshot and reset per-latch counters."""
        raise NotImplementedError

    @abstractmethod
    def write_snapshot(self, snap: dict, tick_id: int) -> None:
        """Write a pre-captured snapshot to BB and emit business-in events."""
        raise NotImplementedError


class DepthSampler:
    """Optional depth-sampling helper for input adapters.

    Subscribes to /camera/depth/image_raw and /camera/depth/camera_info.
    Caches the latest D2C-aligned depth frame (uint16 mm) and colour-space
    intrinsics thread-safely under a dedicated lock.

    Usage:
        Pass ``enable_depth=True`` when constructing a vision adapter; the
        adapter will internally create a DepthSampler and call sample_depth()
        from _build_entry().

    Sampling:
        The center pixel is located by ``_get_sample_point(px, py)`` (rounds to
        the nearest integer pixel). Depth is then read by ``_extract_depth_mm()``
        as the **median of valid (non-zero) pixels** in a ``(2r+1)x(2r+1)`` window
        around that center (``r = window_radius``, default 4). This resists depth
        holes — a single center pixel often reads 0 on round/specular surfaces.
        ``window_radius=0`` reduces to single-pixel sampling.

    Extension points:
        Override ``_get_sample_point(px, py)`` to change the center-pixel rule.
        Override ``_extract_depth_mm(arr, col, row)`` for a different aggregation
        (e.g. min-in-ROI, mean).

    Thread safety:
        ``_depth_lock`` guards ``_depth_arr`` and ``_intrinsics``.  Both are
        replaced atomically (never mutated in place) so callers always see a
        consistent snapshot.
    """

    _NONE_FIELDS = {
        'depth_mm': None,
        'depth_m':  None,
        'cam_x_m':  None,
        'cam_y_m':  None,
        'cam_z_m':  None,
    }

    def __init__(self, window_radius: int = 4):
        # Lazy-import to avoid hard module-level deps on numpy/rospy/sensor_msgs
        # in contexts that only need XyzInputAdapter without depth.
        import numpy as _np
        import rospy
        from sensor_msgs.msg import CameraInfo, Image

        self._np            = _np
        self._window_radius = int(window_radius)
        self._depth_lock    = threading.Lock()
        self._depth_arr     = None   # np.ndarray uint16 H×W mm; None until first frame
        self._intrinsics    = None   # dict {fx,fy,cx,cy}; None until first CameraInfo

        rospy.Subscriber('/camera/depth/image_raw',   Image,      self._depth_cb)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self._info_cb)

    # ------------------------------------------------------------------
    # ROS callbacks — replace cached state atomically; no lock contention
    # with sample_depth callers beyond the brief assignment.
    # ------------------------------------------------------------------

    def _depth_cb(self, msg):
        arr = self._np.frombuffer(msg.data, dtype=self._np.uint16).reshape(
            msg.height, msg.width)
        with self._depth_lock:
            self._depth_arr = arr

    def _info_cb(self, msg):
        K = msg.K  # 3×3 row-major flat list: [fx,0,cx, 0,fy,cy, 0,0,1]
        with self._depth_lock:
            self._intrinsics = {'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5]}

    # ------------------------------------------------------------------
    # Sampling rule — override for non-center-point rules
    # ------------------------------------------------------------------

    def _get_sample_point(self, px: float, py: float):
        """Return (col, row) integer pixel to sample for the given detection centre.

        Default rule: center-point — round px → col, py → row.
        Override this method to change which pixel is treated as the centre.
        """
        return int(round(px)), int(round(py))

    def _extract_depth_mm(self, arr, col, row):
        """Return a robust depth (mm) for the centre pixel (col, row), or None.

        Default rule: median of valid (non-zero) depth pixels in a
        ``(2r+1)x(2r+1)`` window around (col, row), where ``r = window_radius``.
        Returns None if every pixel in the window is zero/invalid.
        Override this method for a different aggregation (min-in-ROI, mean, …).
        """
        r = self._window_radius
        h, w = arr.shape
        c0, c1 = max(0, col - r), min(w, col + r + 1)
        r0, r1 = max(0, row - r), min(h, row + r + 1)
        valid = arr[r0:r1, c0:c1]
        valid = valid[valid > 0]
        if valid.size == 0:
            return None
        return float(self._np.median(valid))

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def sample_depth(self, px: float, py: float) -> dict:
        """Return depth fields dict for detection centre pixel (px, py).

        Returns a dict with keys: depth_mm, depth_m, cam_x_m, cam_y_m, cam_z_m.
        depth_mm is the median of valid pixels in a window around (px, py)
        (see _extract_depth_mm). Any field that cannot be computed is None
        (depth stream unavailable, pixel out of bounds, no valid depth in the
        window, or intrinsics not yet received).

        3-D projection uses the pinhole model with D2C-aligned rgb_intrinsic:
            cam_x = (px - cx) * depth_m / fx
            cam_y = (py - cy) * depth_m / fy
            cam_z = depth_m
        """
        with self._depth_lock:
            arr  = self._depth_arr
            intr = self._intrinsics

        if arr is None:
            return dict(self._NONE_FIELDS)

        col, row = self._get_sample_point(px, py)
        h, w = arr.shape
        if not (0 <= col < w and 0 <= row < h):
            return dict(self._NONE_FIELDS)

        dmm = self._extract_depth_mm(arr, col, row)
        if dmm is None:
            # No valid (non-zero) depth in the sampling window.
            return dict(self._NONE_FIELDS)

        dm = dmm / 1000.0

        if intr is None:
            # Depth value available but intrinsics not yet received; cam_xyz = None.
            return {'depth_mm': dmm, 'depth_m': dm,
                    'cam_x_m': None, 'cam_y_m': None, 'cam_z_m': None}

        return {
            'depth_mm': dmm,
            'depth_m':  dm,
            'cam_x_m':  (px - intr['cx']) * dm / intr['fx'],
            'cam_y_m':  (py - intr['cy']) * dm / intr['fy'],
            'cam_z_m':  dm,
        }
