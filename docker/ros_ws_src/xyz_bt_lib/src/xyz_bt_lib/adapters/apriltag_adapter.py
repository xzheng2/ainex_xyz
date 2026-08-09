#!/usr/bin/env python3
# encoding: utf-8
"""AprilTagAdapter — /tag_detections → /latched/tracked_objects

Input adapter.

ROS topic subscribed:
  /tag_detections

ROS message type:
  apriltag_ros/AprilTagDetectionArray

BB keys written:
  BB.TRACKED_OBJECTS   (/latched/tracked_objects)
  BB.APRILTAG_TURN_BIAS (/latched/apriltag_turn_bias) — scalar, current target's yaw bias (deg)
  BB.APRILTAG_DIRECTION (/latched/apriltag_direction) — scalar str, the fresh tag's course
                       label from tag_direction_map (e.g. 'turn_right'/'turn_left'/'forward');
                       '' when the tag is not detected this tick.
  BB.APRILTAG_TAG_ID   (/latched/apriltag_tag_id) — scalar int, the fresh tag's numeric id;
                       -1 when the tag is not detected this tick.

ROS topic published:
  /apriltag/turn_bias  (std_msgs/Float64) — the tag's in-image rotation angle (deg), i.e. the
                       tag read as a heading 'arrow'; degrees to turn to match the tag's
                       rotation. Sign matches the gait yaw convention (REP-103):
                       + = counter-clockwise (turn left), − = clockwise (turn right).
                       NaN when the tag is not detected this tick.

BB value schema — TRACKED_OBJECTS is a dict[target_id, stat_dict]:

  {
    'apriltag': {
        'label':      'apriltag',
        'shape':      'rect',
        'x':          float,  # tag centre pixel x (average of corners[0].x, corners[2].x)
        'y':          float,  # tag centre pixel y (average of corners[0].y, corners[2].y)
        'error_x':    float,  # x − image_center_x  (positive = tag right of centre)
        'error_y':    float,  # y − image_center_y  (positive = tag below centre)
        'size':       float,  # pixel area via Shoelace formula on the 4 tag corners
        'turn_bias_deg': float, # tag's in-image rotation angle (deg); + = CCW/left, − = CW/right (gait yaw)
        'tag_id':     int,    # numeric AprilTag id of this detection
        'direction':  str,    # course label from tag_direction_map ('' if id unmapped)
        'lost_count': int,    # consecutive ticks without detection (0 = seen this tick)
        # Depth fields (always present; None when depth disabled or unavailable):
        'depth_mm':   float | None,  # sampled depth at tag centre, millimetres
        'depth_m':    float | None,  # sampled depth at tag centre, metres
        'cam_x_m':    float | None,  # 3-D X in camera frame, metres
        'cam_y_m':    float | None,  # 3-D Y in camera frame, metres
        'cam_z_m':    float | None,  # 3-D Z in camera frame (= depth_m), metres
    },
  }

  Default value: {}  (empty dict = nothing tracked).
  lost_count == 0  → tag detected this camera frame.
  lost_count  > 0  → tag absent, entry preserved from last detection.

Size calculation (Shoelace / Gauss area formula):
  area = |Σ (x_i * y_j − x_j * y_i)| / 2   (Shoelace / Gauss area formula)
  where (x_i, y_i) are the four corners in order from the detection message.

Steering calculation:
  error_x = centre_x − (image_width / 2 + center_x_offset)
  → used directly by L2_Gait_VisionToObject for proportional yaw steering.

Default tag filter: accepts the first tag in msg.detections regardless of ID.
Pass tag_id_filter (list[int]) to restrict to a specific AprilTag ID.

Two-phase latch protocol:
  Phase 1: snapshot_and_reset() while caller holds the shared lock.
  Phase 2: write_snapshot(snap, tick_id) after lock release, on main thread.

Observability:
  write_snapshot() emits:
    - ros_in when received_count > 0
    - input_state every tick
  This adapter never emits ros_out/ros_result.
"""
import math
import threading

import rospy
from py_trees.common import Access
from std_msgs.msg import Float64
from apriltag_ros.msg import AprilTagDetectionArray

from xyz_bt_lib.blackboard.blackboard_keys import BB
from xyz_bt_lib.core.base_adapter import DepthSampler, XyzInputAdapter


class AprilTagAdapter(XyzInputAdapter):
    """Adapter: /tag_detections (AprilTagDetectionArray) → BB.TRACKED_OBJECTS."""

    ROS_TOPIC    = '/tag_detections'
    ADAPTER_NAME = 'AprilTagAdapter'
    BB_READS     = []
    BB_WRITES    = [BB.TRACKED_OBJECTS, BB.DETECTION_SOURCE, BB.APRILTAG_TURN_BIAS,
                    BB.APRILTAG_DIRECTION, BB.APRILTAG_TAG_ID]
    FACADE_CALLS = []
    # Default tag id → course-label map (matches xyz_bt_lib.sensors.apriltag/config/tags.yaml).
    _DEFAULT_TAG_DIRECTION_MAP = {1: 'forward', 2: 'turn_right', 3: 'turn_left'}
    CONFIG_DEFAULTS = {
        'image_width':      640,
        'image_height':     480,
        'center_x_offset':  0.0,
        'center_y_offset':  0.0,
        'turn_bias_topic':  '/apriltag/turn_bias',
        'tag_direction_map': {1: 'forward', 2: 'turn_right', 3: 'turn_left'},
    }

    def __init__(self, lock: threading.Lock, logger=None, tick_id_getter=None,
                 target_id: str = 'apriltag',
                 tag_id_filter: list = None,
                 image_width: int = 640,
                 image_height: int = 480,
                 center_x_offset: float = 0.0,
                 center_y_offset: float = 0.0,
                 turn_bias_topic: str = '/apriltag/turn_bias',
                 tag_direction_map: dict = None,
                 enable_depth: bool = False):
        """
        Args:
            lock:             Shared threading.Lock (same as bt_node.lock).
            logger:           DebugEventLogger-compatible object, or None.
            tick_id_getter:   Callable returning current tick_id.
            target_id:        Key used in tracked_objects dict (default 'apriltag').
            tag_id_filter:    List of int AprilTag IDs to accept; None = any ID.
            image_width:      Camera image width in pixels.
            image_height:     Camera image height in pixels.
            center_x_offset:  Calibration shift for image centre X (pixels).
            center_y_offset:  Calibration shift for image centre Y (pixels).
            turn_bias_topic:  Topic name for the published std_msgs/Float64 turn bias (deg) —
                              the tag's in-image rotation angle (see _compute_turn_bias_deg).
            tag_direction_map: dict[int tag_id -> str course label] used to tag each
                              detection with a 'direction' ('turn_right'/'turn_left'/
                              'forward'/…). None uses the default map (ids 1/2/3).
                              Unmapped ids resolve to ''.
            enable_depth:     When True, subscribe to /camera/depth/* and populate
                              depth fields (depth_mm, depth_m, cam_x/y/z_m) in each
                              tracked_objects entry.  Default False for backward compat.
        """
        super().__init__(lock=lock, logger=logger, tick_id_getter=tick_id_getter)

        self._target_id        = target_id
        self._tag_id_filter    = tag_id_filter
        self._image_width      = image_width
        self._image_height     = image_height
        self._center_x_offset  = center_x_offset
        self._center_y_offset  = center_y_offset
        self._tag_direction_map = (dict(self._DEFAULT_TAG_DIRECTION_MAP)
                                   if tag_direction_map is None
                                   else dict(tag_direction_map))
        self._depth_sampler    = DepthSampler() if enable_depth else None

        # Live (async) state — mutated only by _apply_live_writes() under self._lock.
        self._live_tracked_objects = {}   # dict[target_id, stat_dict]

        # Latched BB client — written only by write_snapshot() (main thread).
        self._bb = self.make_latched_bb_client(name=self.ADAPTER_NAME)
        self._bb.register_key(key=BB.TRACKED_OBJECTS_KEY,   access=Access.WRITE)
        self._bb.register_key(key=BB.DETECTION_SOURCE_KEY,  access=Access.WRITE)
        self._bb.register_key(key=BB.APRILTAG_TURN_BIAS_KEY, access=Access.WRITE)
        self._bb.register_key(key=BB.APRILTAG_DIRECTION_KEY, access=Access.WRITE)
        self._bb.register_key(key=BB.APRILTAG_TAG_ID_KEY,   access=Access.WRITE)
        self._bb.tracked_objects   = {}
        self._bb.detection_source  = 'apriltag'
        self._bb.apriltag_turn_bias = float('nan')
        self._bb.apriltag_direction = ''
        self._bb.apriltag_tag_id    = -1

        # Dedicated control-consumable topic for the yaw bias (deg), mirrors /imu_gui style.
        self._turn_bias_pub = rospy.Publisher(turn_bias_topic, Float64, queue_size=1)

        rospy.Subscriber(self.ROS_TOPIC, AprilTagDetectionArray, self._callback)

    # ------------------------------------------------------------------
    # Side-effect-free conversion helpers
    # ------------------------------------------------------------------

    def _find_detection(self, msg: AprilTagDetectionArray):
        """Return the first AprilTagDetection that passes the tag_id_filter, or None.

        No BB reads/writes, ROS calls, or live-state mutation here.
        """
        for det in msg.detections:
            if self._tag_id_filter is None:
                return det
            if any(tid in det.id for tid in self._tag_id_filter):
                return det
        return None

    def _compute_center(self, corners) -> tuple:
        """Return (centre_x, centre_y) in pixels from the 4 AprilTag corners.

        Uses opposite corners (index 0 and 2) to average the diagonal midpoint.
        Averages opposite corners 0 and 2: x = (corners[0].x + corners[2].x) / 2.
        No BB/ROS/facade calls here.
        """
        cx = (corners[0].x + corners[2].x) / 2.0
        cy = (corners[0].y + corners[2].y) / 2.0
        return float(cx), float(cy)

    def _compute_area(self, corners) -> float:
        """Return pixel area using the Shoelace (Gauss) formula.

        area = |Σ (x_i * y_{i+1} − x_{i+1} * y_i)| / 2   (cyclic over 4 corners)
        Standard 4-corner Shoelace area.
        No BB/ROS/facade calls here.
        """
        n = len(corners)
        total = 0.0
        for i in range(n):
            xi = corners[i].x
            yi = corners[i].y
            xj = corners[(i + 1) % n].x
            yj = corners[(i + 1) % n].y
            total += (xi * yj) - (xj * yi)
        return abs(total) / 2.0

    def _compute_errors(self, cx: float, cy: float) -> tuple:
        """Return (error_x, error_y) as offset from calibrated image centre.

        image_centre_x = image_width  / 2 + center_x_offset
        image_centre_y = image_height / 2 + center_y_offset
        No BB/ROS/facade calls here.
        """
        icx = self._image_width  / 2.0 + self._center_x_offset
        icy = self._image_height / 2.0 + self._center_y_offset
        return cx - icx, cy - icy

    def _compute_turn_bias_deg(self, corners) -> float:
        """In-image rotation angle of the tag (deg) — the tag read as a heading 'arrow'.

        apriltag_ros returns corners in a fixed order tied to the tag pattern
        ([0]=TL [1]=TR [2]=BR [3]=BL), so the tag's own edges rotate WITH the physical tag.
        Average the top edge (0→1) and bottom edge (3→2) direction and take its angle from
        the image horizontal:

            turn_bias_deg = degrees(atan2(dy, dx))

        0° = tag upright (top/bottom edges horizontal); the robot turns by this angle to line
        its heading up with the tag's arrow.  Uses the INTRINSIC corner order (not pixel
        sorting) so the rotation signal is preserved.  No camera intrinsics needed.

        Sign matches the gait yaw convention (REP-103): + = counter-clockwise (turn LEFT),
        − = clockwise (turn RIGHT).  Verify on the robot and flip if reversed.
        No BB/ROS/facade calls here.
        """
        if len(corners) != 4:
            return 0.0
        c0, c1, c2, c3 = corners
        dx =  ((c1.x - c0.x) + (c2.x - c3.x)) / 2.0   # mean x-run of top (0→1) + bottom (3→2) edges
        dy = -((c1.y - c0.y) + (c2.y - c3.y)) / 2.0   # negate: image-down → math y-up
        # y-up frame → atan2 is CCW-positive: + = turn LEFT, − = turn RIGHT (gait yaw sign).
        return math.degrees(math.atan2(dy, dx))

    def _resolve_direction(self, det) -> tuple:
        """Return (tag_id, direction_label) for one detection.

        tag_id is the first numeric id on the detection; direction is looked up in
        tag_direction_map ('' when the id is unmapped). No BB/ROS/facade calls here.
        """
        tag_id = int(det.id[0]) if len(det.id) else -1
        direction = self._tag_direction_map.get(tag_id, '')
        return tag_id, direction

    def _build_entry(self, det) -> dict:
        """Build a stat_dict for one matched AprilTagDetection (without lost_count).

        No BB reads/writes, ROS calls, or live-state mutation here.
        Depth fields (depth_mm, depth_m, cam_x/y/z_m) are always present;
        values are None when enable_depth=False or depth is unavailable.
        """
        corners = det.corners
        cx, cy   = self._compute_center(corners)
        err_x, err_y = self._compute_errors(cx, cy)
        area = self._compute_area(corners)
        tag_id, direction = self._resolve_direction(det)
        entry = {
            'label':   'apriltag',
            'shape':   'rect',
            'x':       cx,
            'y':       cy,
            'error_x': err_x,
            'error_y': err_y,
            'size':    area,
            'turn_bias_deg': self._compute_turn_bias_deg(corners),
            'tag_id':  tag_id,
            'direction': direction,
        }
        if self._depth_sampler is not None:
            entry.update(self._depth_sampler.sample_depth(cx, cy))
        else:
            entry.update(DepthSampler._NONE_FIELDS)
        return entry

    def _apply_live_writes(self, entry_or_none: dict) -> None:
        """Merge detection result into _live_tracked_objects under self._lock.

        If entry_or_none is not None: create/update with lost_count=0.
        If None (not detected this frame): increment lost_count, or omit if never seen.
        """
        if entry_or_none is not None:
            new_entry = dict(entry_or_none, lost_count=0)
            self._live_tracked_objects[self._target_id] = new_entry
        else:
            old = self._live_tracked_objects.get(self._target_id)
            if old is not None:
                self._live_tracked_objects[self._target_id] = dict(
                    old, lost_count=old['lost_count'] + 1)
            # else: never seen — omit (missing key = canonical absence)

    # ------------------------------------------------------------------
    # Async callback (ROS callback thread)
    # ------------------------------------------------------------------

    def _callback(self, msg: AprilTagDetectionArray) -> None:
        """Receive AprilTagDetectionArray, find the target tag, update live state."""
        det = self._find_detection(msg)
        entry = self._build_entry(det) if det is not None else None

        with self._lock:
            self._apply_live_writes(entry)
            self._received_count += 1

    # ------------------------------------------------------------------
    # Public two-phase latch API
    # ------------------------------------------------------------------

    def snapshot_and_reset(self) -> dict:
        """Return current live-state snapshot and reset received_count.

        lost_count is NOT reset — sticky state persisting across ticks.
        Must be called while the caller holds self._lock.
        """
        snap = {
            'tracked_objects': dict(self._live_tracked_objects),
            'received_count':  self._received_count,
        }
        self._received_count = 0
        return snap

    def write_snapshot(self, snap: dict, tick_id: int) -> None:
        """Write pre-captured snapshot to BB and emit observability events.

        Called only from the main thread after the shared lock is released.
        """
        self.emit_ros_in(tick_id=tick_id, received_count=snap['received_count'])

        self._bb.tracked_objects = snap['tracked_objects']

        # Yaw bias (deg) + course label/id of the tracked target; neutral values when
        # the tag is not fresh this tick (NaN bias, '' direction, -1 id).
        target = snap['tracked_objects'].get(self._target_id)
        if target is not None and target['lost_count'] == 0:
            turn_bias = float(target['turn_bias_deg'])
            direction = str(target.get('direction', ''))
            tag_id    = int(target.get('tag_id', -1))
        else:
            turn_bias = float('nan')
            direction = ''
            tag_id    = -1
        self._bb.apriltag_turn_bias = turn_bias
        self._bb.apriltag_direction = direction
        self._bb.apriltag_tag_id    = tag_id
        self._turn_bias_pub.publish(Float64(turn_bias))

        tracked_log = {}
        for tid, e in snap['tracked_objects'].items():
            entry_log = {'size': round(e['size'], 1),
                         'error_x': round(e['error_x'], 1),
                         'turn_bias_deg': round(e['turn_bias_deg'], 1),
                         'tag_id': e.get('tag_id', -1),
                         'direction': e.get('direction', ''),
                         'lost_count': e['lost_count']}
            if e.get('depth_m') is not None:
                entry_log['depth_m'] = round(e['depth_m'], 3)
            tracked_log[tid] = entry_log
        self.emit_input_state(
            tick_id=tick_id,
            bb_writes={BB.TRACKED_OBJECTS: tracked_log,
                       BB.APRILTAG_TURN_BIAS: (None if turn_bias != turn_bias
                                               else round(turn_bias, 1)),
                       BB.APRILTAG_DIRECTION: direction,
                       BB.APRILTAG_TAG_ID: tag_id},
        )
