#!/usr/bin/env python3
"""depth_nav_node — depth-based local mapping + A* navigation → /depth_nav/state.

Ported from phase3_viewer.py (macOS single-process viewer) to a headless ROS
node. Reuses the robot's existing camera + YOLO drivers instead of opening the
Orbbec SDK or running a second YOLO on the Pi.

ROS topics subscribed:
  /camera/depth/image_raw    (sensor_msgs/Image, 16UC1, millimetres)
  /camera/depth/camera_info  (sensor_msgs/CameraInfo — fx = K[0], cx = K[2])
  /yolo/detections           (ainex_interfaces/ObjectsInfo — optional obstacles)

ROS topic published:
  /depth_nav/state           (xyz_bt_lib/DepthNavState — nav_yaw, has_path, obstacle_near)

Obstacle sources:
  depth threshold  — ALWAYS active (blocks nearer than ~obstacle_dist mm)
  YOLO boxes       — OPTIONAL, gated by ~use_yolo_obstacles; only ObjectInfo
                     whose label is in ~obstacle_labels contribute. ObjectInfo gives
                     centre x,y + width,height; converted here to corner boxes.

GEOMETRY PRECONDITION (operator-owned):
  The 16x16 grid (start (8,15), goal rows 0/1, top-pad depends on color_frame_height) is only
  valid at a FIXED head pitch. The head must be pre-positioned (tilted down) by a
  separate node / launch step before this node's output is meaningful. This node
  does not move the head.

Parameters (~private):
  ~depth_topic        (str,  /camera/depth/image_raw)
  ~camera_info_topic  (str,  /camera/depth/camera_info)
  ~yolo_topic         (str,  /yolo/detections)
  ~rate_hz            (float, 10.0)   planning/publish rate
  ~obstacle_dist      (float, 200.0)  mm depth-obstacle threshold
  ~obstacle_hold_ticks (int,  10)   keep a block in the fused mask this many ticks after
                                    it stops being detected (flicker / FOV-exit); 0 = off
  ~use_yolo_obstacles (bool,  True)   fuse YOLO obstacle boxes into the mask
  ~obstacle_labels    (str,   'obstacle')  YOLO label(s) treated as obstacles;
                                           comma-separated for multiple
                                           (e.g. obstacle,chair,person)
  ~prox_mode          (str, 'horizontal')  obstacle proximity-penalty shape:
                                           'horizontal' = bx-only (same-row column
                                           distance); 'radial' = 2D DIST_L2 halo
  ~prox_near_extra_row_min/max (int, 13/15) near-feet rows given a SEPARATE proximity
                                           halo (Replace) with its own distance + scores
  ~prox_near_max_dist (float, 10.0)  "distance": near-band halo reach in blocks (its own
                                           value, not the shared PROX_MAX_DIST=4)
  ~prox_near_weight   (float, 16.0)  "scores": near-band halo peak magnitude (its own value,
                                           not the shared PROX_WEIGHT=8)
  ~prox_near_min      (float, 8.0)   floor: min penalty for in-reach near blocks
                                           (d < prox_near_max_dist); 0 = no floor
  ~near_row_min/max   (int,   12/15)  near-front window rows for obstacle_near
  ~near_col_min/max   (int,   6/9)    near-front window cols for obstacle_near
  ~fallback_fx/fy     (float, 367.011/366.836) fx/fy until camera_info arrives
  ~fallback_cx/cy     (float, 315.780/237.146) cx/cy until camera_info arrives
  ~enable_goal_correction   (bool,  True)  bias the A* goal by the IMU heading toward a
                                           target world heading
  ~target_world_heading_deg (float, 0.0)   world heading to stabilise to
                                           (0 = robot's initial heading)
  ~use_imu_accum            (bool,  True)   use the imu_gui cal-6ax heading as the
                                           goal-correction source
  ~imu_gui_topic            (str,  /imu_gui)  std_msgs/Float64 heading (deg) published by
                                           imu_gui_node.py (cal-mode 6ax); consumed as-is
  ~imu_accum_sign           (float, 1.0)   imu heading→goal-correction sign (validate on robot)
  ~imu_accum_deg_per_unit   (float, 1.0)   calibration: deg of real heading per integrated unit
  ~enable_goalpost_goal     (bool,  True)  once the obstacle is gone (mandatory midpoint
                                           waypoint dropped) and a goalpost is detected,
                                           aim the A* goal at the nearest goalpost's
                                           bearing (skips the heading correction); no
                                           goalpost → normal (heading/centre) goal
  ~goalpost_label           (str, 'goalpost')  YOLO label used as the goalpost goal
                                           (must match the model's class name)
  ~goalpost_gate_offset_blocks (int, 2)  offset the goal this many grid columns from the
                                           detected goalpost toward the goal gate; the gate
                                           side comes from the per-run IMU heading swing
                                           (|min|>|max| = left, |max|>|min| = right)
  ~goalpost_gate_sign       (float, 1.0)  flips the gate-offset direction (validate on robot)
  ~goalpost_gate_max_dist_mm (float, 1500) apply the gate offset only when the goalpost is
                                          within this depth (mm, from our grid); else aim at post
  ~enable_start_from_object (bool,  True)  start A* at a YOLO object centre instead
                                           of the fixed robot-feet cell
  ~start_object_label       (str, 'football')  YOLO label whose centre is the start
                                           (must match the model's class name)
  ~start_min_row/max_row    (int,   10/15) grid-row band in which the object-start is
                                           used; outside it → fall back to (8,15)
  ~mpc_top_k                (int,   3)    per goal, take the K lowest-cost A* paths
  ~mpc_w_near/far/turn      (float, 1/1/0.3) temporal-consistency weights: |Δnear_bx|,
                                           |Δfar_bx|, (Δturn)² vs the previous tick's path
  ~mpc_center_col           (int,   8)    reference column for the turn descriptor
  ~midpoint_deactivate_ticks (int,  15)   ticks of no obstacle (YOLO+depth) before the
                                          mandatory midpoint waypoint is dropped
  ~publish_debug      (bool,  False)  publish the two phase3-style debug views
  ~color_topic        (str,   /camera/image_raw)  colour frame for the debug overlay
  ~debug_jpeg_quality (int,   65)     JPEG quality for the compressed debug topics
  ~color_frame_height (int,   640)    colour frame height; sets the colour→grid shift
                                      cam_off = (GRID_H - color_frame_height)//BLOCK_SIZE.
                                      640 = banded view bridge (offset 0); 480 = plain
                                      down-view bridge (offset 160). Auto-refreshed from
                                      the live frame when publish_debug is on.

Optional debug views (~publish_debug=true):
  Two SEPARATE compressed topics (sensor_msgs/CompressedImage, jpeg):
    /depth_nav/debug_view/compressed  — 640x640 camera canvas: depth contours +
                                        YOLO boxes + A* path + grid labels
    /depth_nav/debug_map/compressed   — 400x350 top-down occupancy map
  View each with:  rqt_image_view  (select /depth_nav/debug_view or /depth_nav/debug_map)
  Off by default (zero overhead): no colour subscription, no rendering, no encoding.
"""
import math
import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Float64
from ainex_interfaces.msg import ObjectsInfo

from xyz_bt_lib.msg import DepthNavState
import xyz_bt_lib.sensors.depth_nav.planning as planning
from xyz_bt_lib.sensors.depth_nav import debug_view


# ── tunable config ────────────────────────────────────────────────────────────
# YOLO labels treated as obstacles. Any detection whose label is in this tuple
# is fused into the A* obstacle mask. Override per-run with the ~obstacle_labels
# param (comma-separated, e.g. obstacle_labels:=obstacle,chair,person).
OBSTACLE_LABELS = ('obstacle',)


class DepthNavNode:
    # Below this absolute per-run IMU heading swing (deg) on both sides, the gate
    # side is undetermined and no goalpost offset is applied (aim straight at post).
    _GATE_EPS_DEG = 1.0

    def __init__(self):
        rospy.init_node('depth_nav_node', log_level=rospy.INFO)

        self._depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self._info_topic   = rospy.get_param('~camera_info_topic', '/camera/depth/camera_info')
        self._yolo_topic   = rospy.get_param('~yolo_topic', '/yolo/detections')
        self._rate_hz      = float(rospy.get_param('~rate_hz', 10.0))

        self._obstacle_dist      = float(rospy.get_param('~obstacle_dist', planning.OBSTACLE_DIST))
        # Obstacle-hold: keep a block in the fused mask for this many ticks after it stops
        # being detected, so the A* path keeps avoiding an obstacle that flickered out or
        # left the FOV. 0 = off (fresh mask every tick).
        self._obstacle_hold_ticks = int(rospy.get_param('~obstacle_hold_ticks', planning.OBSTACLE_HOLD_TICKS))
        self._hold_counter = None            # per-block countdown grid, lazily sized to the mask
        self._use_yolo_obstacles = bool(rospy.get_param('~use_yolo_obstacles', True))
        labels = rospy.get_param('~obstacle_labels', list(OBSTACLE_LABELS))
        if isinstance(labels, str):              # accept "a,b,c" or a YAML list
            labels = [s.strip() for s in labels.split(',') if s.strip()]
        self._obstacle_labels = tuple(labels)
        # Obstacle proximity-penalty mode: 'horizontal' (bx-only, same-row) or 'radial' (2D halo).
        self._prox_mode = rospy.get_param('~prox_mode', 'horizontal')
        # Separate near-feet proximity halo: rows [prox_near_extra_row_min,
        # prox_near_extra_row_max] use their OWN reach (distance) + weight (scores),
        # replacing the shared halo there, so A* avoids squeezing past an obstacle right
        # at the robot's feet. Set the near params equal to the base PROX_MAX_DIST /
        # PROX_WEIGHT to make the band identical to other rows.
        self._prox_near_extra_row_min = int(rospy.get_param('~prox_near_extra_row_min', planning.NEAR_EXTRA_ROW_MIN))
        self._prox_near_extra_row_max = int(rospy.get_param('~prox_near_extra_row_max', planning.NEAR_EXTRA_ROW_MAX))
        self._prox_near_max_dist      = float(rospy.get_param('~prox_near_max_dist', planning.NEAR_PROX_MAX_DIST))
        self._prox_near_weight        = float(rospy.get_param('~prox_near_weight', planning.NEAR_PROX_WEIGHT))
        self._prox_near_min           = float(rospy.get_param('~prox_near_min', planning.NEAR_PROX_MIN))
        self._near_row_min = int(rospy.get_param('~near_row_min', 12))
        self._near_row_max = int(rospy.get_param('~near_row_max', 15))
        self._near_col_min = int(rospy.get_param('~near_col_min', 6))
        self._near_col_max = int(rospy.get_param('~near_col_max', 9))

        self._fx = float(rospy.get_param('~fallback_fx', 367.011))
        self._cx = float(rospy.get_param('~fallback_cx', 315.780))
        self._fy = float(rospy.get_param('~fallback_fy', 366.836))
        self._cy = float(rospy.get_param('~fallback_cy', 237.146))

        # Goal correction: bias the A* goal toward a target world heading using the
        # IMU heading computed by imu_gui_node.py (cal-mode 6ax), published as
        # std_msgs/Float64 on ~imu_gui_topic; this node subscribes and consumes it.
        self._enable_goal_correction   = bool(rospy.get_param('~enable_goal_correction', False))
        self._target_world_heading_deg = float(rospy.get_param('~target_world_heading_deg', 0.0))
        self._use_imu_accum            = bool(rospy.get_param('~use_imu_accum', True))
        self._imu_gui_topic            = rospy.get_param('~imu_gui_topic', '/imu_gui')
        # heading→goal-correction sign / calibration.
        self._imu_accum_sign           = float(rospy.get_param('~imu_accum_sign', 1.0))
        self._imu_accum_deg_per_unit   = float(rospy.get_param('~imu_accum_deg_per_unit', 1.0))

        # Goalpost goal: once the obstacle is gone (the mandatory midpoint waypoint has
        # been dropped) and a goalpost is detected, aim the A* goal directly at the
        # nearest goalpost's bearing — skipping the heading goal correction. When no
        # goalpost is visible, planning falls back to the normal (heading/centre) goal.
        self._enable_goalpost_goal = bool(rospy.get_param('~enable_goalpost_goal', True))
        self._goalpost_label       = rospy.get_param('~goalpost_label', 'goalpost')
        # Aim the goal offset from the detected goalpost toward the goal gate. The gate
        # side is derived from the per-run IMU heading excursion (|min|>|max| = gate left,
        # |max|>|min| = gate right); the goal is this many blocks to that side of the post.
        self._goalpost_gate_offset_blocks = int(rospy.get_param('~goalpost_gate_offset_blocks', 2))
        self._goalpost_gate_sign          = float(rospy.get_param('~goalpost_gate_sign', 1.0))
        # The 2-block gate only applies once the goalpost is within this depth (mm),
        # read from our own depth grid. Beyond it, aim straight at the post (offset 0).
        self._goalpost_gate_max_dist_mm   = float(rospy.get_param('~goalpost_gate_max_dist_mm', 1500.0))

        # A* start cell from a YOLO object centre (default the football the robot
        # pushes). Used only when the object's grid row is within [min_row, max_row];
        # otherwise the planner falls back to the default START_CELL (robot feet).
        self._enable_start_from_object = bool(rospy.get_param('~enable_start_from_object', True))
        self._start_object_label       = rospy.get_param('~start_object_label', 'football')
        self._start_min_row            = int(rospy.get_param('~start_min_row', 10))
        self._start_max_row            = int(rospy.get_param('~start_max_row', 15))

        # MPC-style multi-candidate path selection: each tick generate candidate goals
        # (target col if clear, else nearest clear left+right) × the top-k A* paths, and
        # pick the one most temporally consistent with the previous tick (no re-added
        # obstacle cost — A* already avoids obstacles).
        self._enable_mpc     = bool(rospy.get_param('~enable_mpc', True))
        self._mpc_top_k      = int(rospy.get_param('~mpc_top_k', 3))
        self._mpc_w_near     = float(rospy.get_param('~mpc_w_near', 1.0))
        self._mpc_w_far      = float(rospy.get_param('~mpc_w_far', 1.0))
        self._mpc_w_turn     = float(rospy.get_param('~mpc_w_turn', 0.3))
        self._mpc_center_col = int(rospy.get_param('~mpc_center_col', 8))
        self._mpc_far_row    = planning.YAW_ROWS[0]   # 7  (far, higher up)
        self._mpc_near_row   = planning.YAW_ROWS[1]   # 10 (near the robot)
        self._prev_desc      = None                   # previous chosen (near_bx, far_bx, turn)

        # Mandatory midpoint waypoint: force the path through the horizontal middle of the
        # largest connected corridor's cells on the near band row (by5), within by1..by5.
        self._enable_midpoint = bool(rospy.get_param('~enable_midpoint', True))
        self._midpoint_band = (int(rospy.get_param('~midpoint_row_min', planning.MIDPOINT_ROWS[0])),
                               int(rospy.get_param('~midpoint_row_max', planning.MIDPOINT_ROWS[1])))
        # Auto-deactivate the midpoint after this many consecutive ticks with no obstacle
        # (YOLO + depth) anywhere in the fused mask; re-activates the tick an obstacle returns.
        self._midpoint_deactivate_ticks = int(rospy.get_param('~midpoint_deactivate_ticks', 15))

        # Optional phase3-style debug views (two separate compressed topics, view
        # with rqt_image_view). Off by default — zero overhead when disabled.
        self._publish_debug = bool(rospy.get_param('~publish_debug', False))
        self._color_topic   = rospy.get_param('~color_topic', '/camera/image_raw')
        self._debug_jpeg_quality = int(rospy.get_param('~debug_jpeg_quality', 65))

        # Colour→grid vertical shift. The colour frame may be the plain 640x480
        # down-view (gemini305_bridge) or the 640x640 banded upper+down view
        # (gemini305_view_bridge); detections are in colour-pixel space and the depth
        # grid is 640-tall with a 160px top pad. cam_off (blocks) = (GRID_H - color_h)
        # // BLOCK_SIZE maps a colour-row to a grid-row. Default from the param; in
        # debug mode it is refreshed from the actual colour frame height (_color_cb).
        self._color_frame_height = int(rospy.get_param('~color_frame_height', planning.GRID_H))
        self._cam_off = (planning.GRID_H - self._color_frame_height) // planning.BLOCK_SIZE

        # Live state (guarded by _lock) populated from callbacks.
        self._lock        = threading.Lock()
        self._depth_arr   = None              # uint16 H×W mm
        self._obstacle_boxes = []             # list of (x1,y1,x2,y2) colour-px corners
        self._detections  = []                # full detection dicts (debug only)
        self._color_frame = None              # latest bgr8 frame (debug only)
        self._depth_smooth = None             # median-blurred depth (debug only)
        self._imu_heading = 0.0               # heading (deg) received from imu_gui (cal 6ax)
        # Per-run memory of the max LEFT (+) and max RIGHT (-) heading pose reached this
        # run — NOT an accumulator; used by the goalpost gate (_gate_offset_cols).
        self._imu_heading_max = 0.0           # max left heading (deg) so far
        self._imu_heading_min = 0.0           # max right heading (deg) so far
        self._start_px    = None              # (x,y) colour-px centre of start object, or None
        self._goalpost_px = None              # (x,y) colour-px centre of nearest goalpost, or None
        self._no_obstacle_ticks = 0           # consecutive ticks with an empty obstacle mask
        self._history     = planning.new_history()

        self.pub = rospy.Publisher('/depth_nav/state', DepthNavState, queue_size=1)
        rospy.Subscriber(self._depth_topic, Image, self._depth_cb, queue_size=1)
        rospy.Subscriber(self._info_topic, CameraInfo, self._info_cb, queue_size=1)
        rospy.Subscriber(self._yolo_topic, ObjectsInfo, self._yolo_cb, queue_size=1)
        if self._enable_goal_correction and self._use_imu_accum:
            rospy.Subscriber(self._imu_gui_topic, Float64, self._imu_gui_cb, queue_size=50)

        self._view_pub = None   # camera canvas (blocks/contours/path/grid)
        self._map_pub  = None   # 2D top-down map
        if self._publish_debug:
            self._view_pub = rospy.Publisher(
                '/depth_nav/debug_view/compressed', CompressedImage, queue_size=1)
            self._map_pub = rospy.Publisher(
                '/depth_nav/debug_map/compressed', CompressedImage, queue_size=1)
            rospy.Subscriber(self._color_topic, Image, self._color_cb, queue_size=1)

        rospy.loginfo('[depth_nav_node] ready — depth=%s info=%s yolo=%s '
                      '(use_yolo=%s labels=%r publish_debug=%s)', self._depth_topic,
                      self._info_topic, self._yolo_topic, self._use_yolo_obstacles,
                      self._obstacle_labels, self._publish_debug)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _depth_cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        with self._lock:
            self._depth_arr = arr

    def _info_cb(self, msg):
        with self._lock:
            self._fx = msg.K[0] or self._fx
            self._cx = msg.K[2] or self._cx
            self._fy = msg.K[4] or self._fy
            self._cy = msg.K[5] or self._cy

    def _imu_gui_cb(self, msg):
        """Cache the cal-6ax heading (deg) published by imu_gui_node.py (~imu_gui_topic).

        No integration here — imu_gui owns the computation; this node only consumes the
        scalar heading. The goal-correction sign is imu_accum_sign, applied later.
        """
        with self._lock:
            self._imu_heading = float(msg.data)
            # Remember the max left/right heading pose reached this run.
            self._imu_heading_max = max(self._imu_heading_max, self._imu_heading)
            self._imu_heading_min = min(self._imu_heading_min, self._imu_heading)

    def _color_cb(self, msg):
        """Cache the latest colour frame for the debug overlay (debug only)."""
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        except Exception:
            return
        with self._lock:
            self._color_frame = arr
            # Auto-correct the colour→grid shift from the live frame height so the
            # overlay (and obstacle/start-cell mapping) follow the actual camera.
            self._cam_off = (planning.GRID_H - arr.shape[0]) // planning.BLOCK_SIZE

    def _yolo_cb(self, msg):
        """Convert ObjectInfo centre+w/h to corner boxes for obstacle labels.

        When publish_debug is on, also retain all detections (any label) so the
        debug overlay can draw every box, not just obstacles.
        """
        boxes = []
        dets  = []
        start_px      = None
        start_area    = -1.0
        goalpost_px   = None
        goalpost_area = -1.0
        for obj in msg.data:
            x1 = obj.x - obj.width / 2.0
            y1 = obj.y - obj.height / 2.0
            x2 = obj.x + obj.width / 2.0
            y2 = obj.y + obj.height / 2.0
            if self._use_yolo_obstacles and obj.label in self._obstacle_labels:
                boxes.append((x1, y1, x2, y2))
            # Start-object centre: the largest-area detection matching the start label.
            if self._enable_start_from_object and obj.label == self._start_object_label:
                area = float(obj.width) * float(obj.height)
                if area > start_area:
                    start_area = area
                    start_px   = (float(obj.x), float(obj.y))
            # Goalpost centre: the largest-area (nearest proxy) matching goalpost_label.
            if self._enable_goalpost_goal and obj.label == self._goalpost_label:
                area = float(obj.width) * float(obj.height)
                if area > goalpost_area:
                    goalpost_area = area
                    goalpost_px   = (float(obj.x), float(obj.y))
            if self._publish_debug:
                dets.append({'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'label': obj.label})
        with self._lock:
            self._obstacle_boxes = boxes
            self._start_px = start_px
            self._goalpost_px = goalpost_px
            if self._publish_debug:
                self._detections = dets

    # ── per-frame grid build ─────────────────────────────────────────────────────

    def _build_grid(self, depth_arr) -> np.ndarray:
        """Resize depth to (COLOR_W, color_frame_height), median-blur, top-pad to GRID_H,
        → 16x16 mean grid.

        Honours ``color_frame_height`` so the depth grid matches ``cam_off``: 640 (banded
        view) → full 640-tall real depth, no top pad (all 16 grid rows real); 480 (plain
        down-view) → 480 real + 160 px (4-block) top pad.
        """
        ch = self._color_frame_height
        h, w = depth_arr.shape
        if (h, w) != (ch, planning.COLOR_W):
            depth_arr = cv2.resize(depth_arr, (planning.COLOR_W, ch),
                                   interpolation=cv2.INTER_NEAREST)
        depth_smooth = cv2.medianBlur(depth_arr, 3)
        self._depth_smooth = depth_smooth
        pad = planning.GRID_H - ch
        depth_640 = (np.vstack([np.repeat(depth_smooth[0:1], pad, axis=0), depth_smooth])
                     if pad > 0 else depth_smooth)
        small = planning.to_block_grid(depth_640)
        self._history.append(small)
        return planning.temporal_mean(self._history)

    # ── goal-column selection (pure; no ROS/lock/side effects) ────────────────────

    def _goal_column(self, midpoint, goalpost_px, goalpost_depth_mm, imu_hmax, imu_hmin,
                     accum, accum_sign, accum_dpu, fx, cx, bcols):
        """Choose the A* goal block-column and the debug-map heading.

        Priority:
          1. Goalpost goal — when enabled, the obstacle is gone (``midpoint is None``,
             i.e. the mandatory waypoint was dropped after the scene stayed clear) and a
             goalpost is detected: aim at the goalpost's bearing OFFSET toward the goal
             gate. The gate side comes from the per-run IMU heading excursion: if
             |imu_hmin| > |imu_hmax| the gate is to the LEFT (goal = post − offset cols);
             if |imu_hmax| > |imu_hmin| it is to the RIGHT (goal = post + offset cols).
             With no meaningful swing yet (both ≈ 0) no offset is applied (aim at the
             post). The offset is ALSO only applied once the goalpost is within
             goalpost_gate_max_dist_mm (from our depth grid); beyond it, aim at the post.
             The accum goal correction is intentionally skipped here.
          2. accum goal correction — bias toward the target world heading. ``accum`` is
             the active heading-accumulator (imu_accum) already resolved by run(),
             with its matching sign / deg-per-unit calibration.
          3. Centre column — straight ahead (follow-nav-path as-is).

        Returns (target_bx, goal_heading_deg). goal_heading_deg is the (unclamped)
        heading used for the debug-map cross, or None when centred.
        """
        if (self._enable_goalpost_goal and midpoint is None
                and goalpost_px is not None):
            heading = math.degrees(math.atan2(goalpost_px[0] - cx, fx))
            gp_col  = planning.heading_to_goal_col(heading, fx, cx, bcols)
            off_col = self._gate_offset_cols(imu_hmax, imu_hmin, goalpost_depth_mm)
            target  = int(max(0, min(bcols - 1, gp_col + off_col)))
            return target, heading
        if self._enable_goal_correction and accum is not None:
            # 1 unit of accum = accum_dpu degrees of real heading (calibration).
            heading = self._target_world_heading_deg + accum_sign * accum_dpu * accum
            return planning.heading_to_goal_col(heading, fx, cx, bcols), heading
        return int(round((bcols - 1) / 2.0)), None

    def _gate_offset_cols(self, imu_hmax, imu_hmin, goalpost_depth_mm):
        """Signed goal-column offset from the goalpost toward the goal gate.

        Only applied once the goalpost is within goalpost_gate_max_dist_mm (0 = not
        detected / invalid depth → no offset). Gate side from the per-run IMU heading
        excursion (larger absolute swing wins):
          |imu_hmin| > |imu_hmax| → gate LEFT  → negative offset (smaller column)
          |imu_hmax| > |imu_hmin| → gate RIGHT → positive offset (larger column)
        A swing below _GATE_EPS_DEG on both sides yields 0 (aim at the post). The
        magnitude is goalpost_gate_offset_blocks, direction flipped by goalpost_gate_sign.
        Pure — no ROS/lock/side effects.
        """
        # Distance gate: only offset when the post is within range (mm).
        if not (0.0 < goalpost_depth_mm < self._goalpost_gate_max_dist_mm):
            return 0
        hmax = abs(imu_hmax if imu_hmax is not None else 0.0)
        hmin = abs(imu_hmin if imu_hmin is not None else 0.0)
        if max(hmax, hmin) < self._GATE_EPS_DEG:
            return 0
        side = -1 if hmin > hmax else 1   # left(-) when the min excursion dominates
        return int(round(self._goalpost_gate_sign * side
                         * self._goalpost_gate_offset_blocks))

    def _goalpost_depth_mm(self, goalpost_px, grid, cam_off):
        """Depth (mm) of the goalpost from our own depth grid, or 0.0 if unavailable.

        Maps the goalpost colour-pixel centre to a grid block and returns the median of
        the valid (>0) blocks in a 3x3 window around it (robust vs a thin post landing on
        a background block / depth holes). 0.0 = no goalpost / no valid depth. Pure — no
        ROS/lock/side effects.
        """
        if goalpost_px is None or grid is None:
            return 0.0
        brows, bcols = grid.shape
        gbx = int(goalpost_px[0]) // planning.BLOCK_SIZE
        gby = int(goalpost_px[1]) // planning.BLOCK_SIZE + cam_off
        c0, c1 = max(0, gbx - 1), min(bcols, gbx + 2)
        r0, r1 = max(0, gby - 1), min(brows, gby + 2)
        if c1 <= c0 or r1 <= r0:
            return 0.0
        win = grid[r0:r1, c0:c1]
        valid = win[win > 0.0]
        return float(np.median(valid)) if valid.size else 0.0

    # ── main loop ────────────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown():
            with self._lock:
                depth_arr = self._depth_arr
                boxes     = list(self._obstacle_boxes)
                fx, cx    = self._fx, self._cx
                fy, cy    = self._fy, self._cy
                dets      = list(self._detections)
                color     = self._color_frame
                imu_accum = self._imu_heading   # cal-6ax heading received from imu_gui
                imu_hmax  = self._imu_heading_max   # per-run max left/right pose memory
                imu_hmin  = self._imu_heading_min
                start_px  = self._start_px
                goalpost_px = self._goalpost_px
                cam_off   = self._cam_off
            if depth_arr is None:
                rate.sleep()
                continue

            grid    = self._build_grid(depth_arr)
            mask    = planning.build_obstacle_mask(
                grid, obstacle_boxes=boxes, obstacle_dist=self._obstacle_dist,
                cam_off=cam_off)
            # Persist obstacle blocks for a few ticks after they stop being detected so the
            # path keeps avoiding an obstacle that flickered out or left the FOV. Applied to
            # the fused mask so everything downstream (penalty, obstacle_near, midpoint) sees it.
            if self._obstacle_hold_ticks > 0:
                if self._hold_counter is None or self._hold_counter.shape != mask.shape:
                    self._hold_counter = np.zeros(mask.shape, dtype=int)
                mask, self._hold_counter = planning.apply_obstacle_hold(
                    mask, self._hold_counter, self._obstacle_hold_ticks)
            if self._prox_mode == 'radial':
                penalty = planning.compute_proximity_penalty(mask)
            else:
                penalty = planning.compute_proximity_penalty_bx(mask)
            # Near-feet rows get a separately-tuned halo (own distance + scores + min floor), Replace.
            penalty = planning.apply_near_row_extra(
                mask, penalty, self._prox_mode,
                self._prox_near_extra_row_min, self._prox_near_extra_row_max,
                self._prox_near_max_dist, self._prox_near_weight, self._prox_near_min)
            # Track how long the scene has been clear (no YOLO or depth obstacle anywhere
            # in the fused mask). Resets to 0 the tick an obstacle reappears.
            obstacle_present = bool(mask.any())
            if obstacle_present:
                self._no_obstacle_ticks = 0
            else:
                self._no_obstacle_ticks += 1
            # Mandatory midpoint waypoint on the near band row (by5): the path must pass
            # through it. Centre column when clear, else the nearest clear cell to centre.
            # Auto-deactivated after midpoint_deactivate_ticks of no obstacle so an open
            # scene plans straight start→goal instead of being pinned to the corridor middle.
            midpoint = None
            if (self._enable_midpoint
                    and self._no_obstacle_ticks < self._midpoint_deactivate_ticks):
                midpoint = planning.find_midpoint(mask, grid, band=self._midpoint_band)
            # Heading-accumulator source for goal correction: IMU (imu_accum).
            if self._use_imu_accum and imu_accum is not None:
                nav_accum, nav_sign, nav_dpu = (
                    imu_accum, self._imu_accum_sign, self._imu_accum_deg_per_unit)
            else:
                nav_accum, nav_sign, nav_dpu = None, 1.0, 1.0
            # Goalpost depth (mm) from our own grid — gates the 2-block goal offset (<1500mm).
            goalpost_depth_mm = self._goalpost_depth_mm(goalpost_px, grid, cam_off)
            # Live print of the imu_gui-provided cal-6ax heading (~5 Hz) for tuning.
            rospy.loginfo_throttle(0.2,
                "[depth_nav] imu_heading=%+.2f deg (L %+.2f / R %+.2f)  goalpost=%.0fmm"
                % (imu_accum, imu_hmax, imu_hmin, goalpost_depth_mm))
            # Target column: goalpost (when obstacle gone), else accum→heading projection
            # (goal correction), else centre. midpoint is None once the obstacle has been
            # gone long enough that the waypoint was dropped.
            target_bx, goal_heading_deg = self._goal_column(
                midpoint, goalpost_px, goalpost_depth_mm, imu_hmax, imu_hmin,
                nav_accum, nav_sign, nav_dpu, fx, cx, grid.shape[1])

            # Start the path at the YOLO start-object (e.g. football) centre when its
            # grid row is within [start_min_row, start_max_row]; else default START_CELL.
            start_cell = planning.START_CELL
            if self._enable_start_from_object and start_px is not None:
                sc = planning.object_start_cell(
                    start_px[0], start_px[1], grid.shape[1], grid.shape[0],
                    self._start_min_row, self._start_max_row, cam_off=cam_off)
                if sc is not None:
                    start_cell = sc

            goals = [planning.find_goal(mask, grid, goal_col=target_bx)]
            # Every candidate path is routed start→goal, or start→midpoint→goal when the
            # mandatory waypoint is enabled (astar_via returns [] if the via is unreachable).
            k = self._mpc_top_k if self._enable_mpc else 1
            cands = []
            for gcell in goals:
                paths = (planning.astar_via(mask, penalty, midpoint, gcell,
                                            start=start_cell, k=k)
                         if midpoint is not None
                         else planning.astar_topk(mask, penalty, gcell,
                                                  start=start_cell, k=k))
                cands += [
                    (p, planning.path_descriptors(
                        p, self._mpc_near_row, self._mpc_far_row, self._mpc_center_col))
                    for p in paths
                ]
            if self._enable_mpc:
                # Pick the candidate most temporally consistent with the previous tick.
                if cands:
                    path, best_desc = min(cands, key=lambda pd: planning.mpc_path_cost(
                        pd[1], self._prev_desc,
                        self._mpc_w_near, self._mpc_w_far, self._mpc_w_turn))
                    self._prev_desc = best_desc
                else:
                    path = None
            else:
                # MPC off: single best A* path (first reachable candidate goal).
                path = cands[0][0] if cands else None
                self._prev_desc = None

            msg = DepthNavState()
            msg.header.stamp = rospy.Time.now()
            msg.obstacle_near = planning.is_obstacle_near(
                mask, self._near_row_min, self._near_row_max,
                self._near_col_min, self._near_col_max)
            # Distance-independent "obstacle present anywhere in the fused mask" (depth+YOLO),
            # NOT gated to the near-front proximity window — consumed by L1_Nav_IsObstacleDetected.
            msg.obstacle_detected = obstacle_present
            # Camera intrinsics travel with the message so the consumer (L2 node) can
            # compute the bearing atan((u-cx)/fx) itself.
            msg.cx = float(cx)
            msg.fx = float(fx)
            # Publish the A* start cell so the L2 node can read it (e.g. ball position).
            msg.start_bx = int(start_cell[0])
            msg.start_by = int(start_cell[1])
            # Republish the imu_gui-provided cal-6ax heading out the
            # same DepthNavState → DepthNavAdapter → /latched/nav_imu_heading chain.
            msg.imu_heading = float(imu_accum)
            # Per-run heading excursion (max LEFT/+ and max RIGHT/−) reached this run,
            # out the same DepthNavState → DepthNavAdapter → /latched/nav_imu_heading_{min,max} chain.
            msg.imu_heading_min = float(imu_hmin)
            msg.imu_heading_max = float(imu_hmax)
            _nan = float('nan')
            if path:
                msg.has_path = True
                # Raw row-9/10 path geometry; the L2 node selects a row and computes yaw.
                msg.nav_u9, msg.nav_u10 = planning.path_cols_from_rows(path)
            else:
                msg.has_path = False
                msg.nav_u9 = _nan
                msg.nav_u10 = _nan
            self.pub.publish(msg)

            if self._publish_debug and self._depth_smooth is not None:
                self._publish_debug_images(color, self._depth_smooth, grid,
                                           path, dets, fx, fy, cx, cy, msg, cam_off,
                                           goal_heading_deg, midpoint)
            rate.sleep()

    def _publish_debug_images(self, color, depth_smooth, grid, path, dets,
                              fx, fy, cx, cy, msg, cam_off, goal_heading_deg=None,
                              midpoint=None):
        """Render the camera canvas + top-down map as two separate compressed topics."""
        try:
            # render_canvas auto-derives the box y-offset from the colour frame height;
            # render_map needs the equivalent block-row shift explicitly.
            canvas = debug_view.render_canvas(
                color, depth_smooth, grid, dets, path, self._obstacle_labels,
                nav_u9=msg.nav_u9, nav_u10=msg.nav_u10, has_path=msg.has_path,
                obstacle_near=msg.obstacle_near, midpoint=midpoint)
            top = debug_view.render_map(
                grid, dets, depth_smooth, path, fx, fy, cx, cy, self._obstacle_labels,
                cam_off=cam_off, goal_heading_deg=goal_heading_deg, midpoint=midpoint)
            self._publish_jpeg(self._view_pub, canvas, msg.header.stamp)
            self._publish_jpeg(self._map_pub, top, msg.header.stamp)
        except Exception as e:
            rospy.logwarn_throttle(5.0, '[depth_nav_node] debug render failed: %s', e)

    def _publish_jpeg(self, pub, image, stamp):
        """JPEG-encode a BGR image and publish it as sensor_msgs/CompressedImage."""
        ok, buf = cv2.imencode('.jpg', image,
                               [cv2.IMWRITE_JPEG_QUALITY, self._debug_jpeg_quality])
        if not ok:
            return
        out = CompressedImage()
        out.header.stamp = stamp
        out.format = 'jpeg'
        out.data = buf.tobytes()
        pub.publish(out)


if __name__ == '__main__':
    DepthNavNode().run()
