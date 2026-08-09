#!/usr/bin/env python3
"""phase3-style debug visualisation for depth_nav (render helpers only).

Ported from phase3_viewer.py's drawing code. These functions produce BGR images;
they NEVER call cv2.imshow. depth_nav_node.py calls them when its ~publish_debug
parameter is enabled, then JPEG-encodes each into its own compressed topic
(view with rqt_image_view).

Two independent images are produced (NOT one composite):
    render_canvas() -> 640x640 camera canvas: depth contours + YOLO boxes +
                       A* path blend + grid labels + HUD
                       (-> /depth_nav/debug_view/compressed)
    render_map()    -> 400x350 top-down occupancy map: depth bands + detection
                       markers + path polyline
                       (-> /depth_nav/debug_map/compressed)

All map geometry is visualisation-only; intrinsics are passed in by the node.
The phase3 ``draw_*`` helpers stay here, out of planning.py, so the algorithm
layer remains ROS-free and display-free.
"""
import math

import numpy as np
import cv2

from xyz_bt_lib.sensors.depth_nav.planning import (
    BLOCK_SIZE, THRESHOLDS, COLOR_H, COLOR_W, GRID_H, CAM_OFF, YAW_ROWS,
)

_FONT = cv2.FONT_HERSHEY_SIMPLEX

# Top-down map extents (visualisation only).
MAP_W, MAP_H = 400, 350
MAP_X_RANGE  = 4400
MAP_Z_RANGE  = 2500

# Goal-heading cross: a marker drawn this far (mm) from the robot along the true
# (unclamped) corrected heading, even when the A* goal column saturated at the edge.
GOAL_CROSS_RANGE_MM = 1000
_GOAL_CROSS_COLOR   = (255, 0, 255)  # magenta (BGR)

_CONTOUR_COLORS = [
    (0, 0, 255), (0, 80, 255), (0, 165, 255), (0, 220, 255), (0, 255, 180),
    (0, 255, 80), (60, 255, 0), (200, 255, 0), (255, 200, 0), (255, 80, 0),
]
_OBSTACLE_COLOR = (0, 0, 220)   # red (BGR) — YOLO obstacle label
_OBJECT_COLOR   = (0, 220, 0)   # green (BGR) — any other YOLO label
_PATH_COLOR     = (0, 220, 220)  # cyan
_YAW_COLOR      = (0, 200, 255)  # yellow
_MIDPOINT_COLOR = (220, 0, 220)  # magenta — corridor attraction midpoint

_GRID_OVERLAY = None


# ── camera canvas ───────────────────────────────────────────────────────────

def depth_colormap(depth_smooth, max_mm: int = MAP_Z_RANGE):
    """Render a uint16 mm depth frame as a JET colormap (0 = black)."""
    norm = np.clip(depth_smooth.astype(np.float32) / max_mm * 255.0, 0, 255).astype(np.uint8)
    cm = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    cm[depth_smooth == 0] = 0
    return cm


def draw_contours(canvas_640, grid):
    """Draw threshold contour boundaries from the 16x16 temporal-mean grid."""
    small_u16 = grid.astype(np.uint16)
    valid = small_u16 > 0
    for i, thresh in enumerate(THRESHOLDS):
        mask  = valid & (small_u16 < thresh)
        color = _CONTOUR_COLORS[i % len(_CONTOUR_COLORS)]
        hy, hx = np.where(mask[:-1] != mask[1:])
        for by, bx in zip(hy, hx):
            y = (by + 1) * BLOCK_SIZE
            canvas_640[y, bx * BLOCK_SIZE:(bx + 1) * BLOCK_SIZE] = color
        vy, vx = np.where(mask[:, :-1] != mask[:, 1:])
        for by, bx in zip(vy, vx):
            x = (bx + 1) * BLOCK_SIZE
            canvas_640[by * BLOCK_SIZE:(by + 1) * BLOCK_SIZE, x] = color


def draw_detections(canvas_640, detections, depth_smooth, obstacle_labels,
                    y_offset=GRID_H - COLOR_H):
    """Draw YOLO boxes + median depth labels on the camera canvas.

    ``y_offset`` is the color→grid vertical shift (px), = GRID_H - color_height.
    The colour frame and detections share the same pixel space; x needs no scaling
    because the bridge width is fixed at COLOR_W. Detection y is in colour space, so
    the box sits at ``y + y_offset`` on the grid canvas, and its depth (depth_smooth
    occupies grid rows [GRID_H-COLOR_H : GRID_H]) is sampled at ``y + y_offset - pad``
    where ``pad = GRID_H - COLOR_H``. Upper-band detections (no depth) clamp to "--".
    """
    dy = y_offset - (GRID_H - COLOR_H)   # color-y → depth-row shift
    for d in detections:
        x1, x2 = int(d['x1']), int(d['x2'])
        y1_raw, y2_raw = int(d['y1']), int(d['y2'])
        y1, y2 = y1_raw + y_offset, y2_raw + y_offset
        color = _OBSTACLE_COLOR if d['label'] in obstacle_labels else _OBJECT_COLOR
        cv2.rectangle(canvas_640, (x1, y1), (x2, y2), color, 2)
        roi   = depth_smooth[max(0, y1_raw + dy):max(0, y2_raw + dy), max(0, x1):max(0, x2)]
        valid = roi[roi > 0]
        dist  = f"{int(np.median(valid))}mm" if valid.size > 0 else "--"
        cv2.putText(canvas_640, f"{d['label']} {dist}", (x1, max(y1 - 6, y_offset + 12)),
                    _FONT, 0.45, color, 1, cv2.LINE_AA)


def _grid_overlay():
    global _GRID_OVERLAY
    if _GRID_OVERLAY is None:
        img = np.zeros((GRID_H, COLOR_W, 3), dtype=np.uint8)
        brows = GRID_H // BLOCK_SIZE
        bcols = COLOR_W // BLOCK_SIZE
        for by in range(brows):
            for bx in range(bcols):
                cx = bx * BLOCK_SIZE + BLOCK_SIZE // 2
                cy = by * BLOCK_SIZE + BLOCK_SIZE // 2
                cv2.putText(img, f"{bx},{by}", (cx - 10, cy + 4),
                            _FONT, 0.22, (80, 80, 80), 1)
        _GRID_OVERLAY = img
    return _GRID_OVERLAY


def draw_grid_labels(canvas_640):
    overlay = _grid_overlay()
    mask = overlay > 0
    canvas_640[mask] = overlay[mask]


def draw_hud(canvas_640, nav_u9, nav_u10, has_path, obstacle_near):
    def _fmt(u):
        return "--" if (u is None or u != u) else f"{u:.0f}"   # u != u → NaN
    txt = (f"u9={_fmt(nav_u9)} u10={_fmt(nav_u10)}" if has_path else "no path")
    cv2.putText(canvas_640, txt, (8, 22), _FONT, 0.55, (0, 220, 220), 1, cv2.LINE_AA)
    if obstacle_near:
        cv2.putText(canvas_640, "OBSTACLE NEAR", (8, 44), _FONT, 0.55,
                    (0, 0, 255), 2, cv2.LINE_AA)


# ── top-down map ────────────────────────────────────────────────────────────

def _band_color(z_mm):
    for i, t in enumerate(THRESHOLDS):
        if z_mm < t:
            return _CONTOUR_COLORS[i]
    return _CONTOUR_COLORS[-1]


def build_top_view(grid, fx, fy, cx, cy):
    """Render the top-down occupancy map; returns (map_img, block_map_pos)."""
    map_img = np.full((MAP_H, MAP_W, 3), 25, dtype=np.uint8)
    block_map_pos = {}

    for thresh in THRESHOLDS:
        if thresh > MAP_Z_RANGE:
            break
        my = MAP_H - int(thresh * MAP_H / MAP_Z_RANGE)
        if 0 <= my < MAP_H:
            cv2.line(map_img, (0, my), (MAP_W, my), (55, 55, 55), 1)
            cv2.putText(map_img, f"{thresh}", (2, my - 2),
                        _FONT, 0.28, (100, 100, 100), 1)

    brows, bcols = grid.shape
    for by in range(brows):
        for bx in range(bcols):
            z = float(grid[by, bx])
            if z <= 0 or z > MAP_Z_RANGE:
                continue
            u      = bx * BLOCK_SIZE + BLOCK_SIZE // 2
            x_real = (u - cx) * z / fx
            map_x  = MAP_W // 2 + int(x_real * MAP_W / MAP_X_RANGE)
            map_y  = MAP_H      - int(z       * MAP_H / MAP_Z_RANGE)
            block_map_pos[(bx, by)] = (map_x, map_y)
            half_w = max(1, int(BLOCK_SIZE * z / fx * MAP_W / MAP_X_RANGE / 2))
            half_h = max(1, int(BLOCK_SIZE * z / fy * MAP_H / MAP_Z_RANGE / 2))
            cv2.rectangle(map_img, (map_x - half_w, map_y - half_h),
                                   (map_x + half_w, map_y + half_h), _band_color(z), -1)

    cv2.drawMarker(map_img, (MAP_W // 2, MAP_H - 4), (255, 255, 255),
                   cv2.MARKER_TRIANGLE_UP, 8, 1)
    return map_img, block_map_pos


def mark_detections_map(map_img, detections, depth_smooth, grid,
                        fx, fy, cx, cy, obstacle_labels, cam_off=CAM_OFF):
    """Fill obstacle-label blocks, mark other labels as points, on the map.

    ``cam_off`` is the color→grid block-row shift = (GRID_H - color_height)//BLOCK_SIZE
    (default CAM_OFF assumes a 480-high colour frame).
    """
    brows, bcols = grid.shape
    for d in detections:
        if d['label'] in obstacle_labels:
            color = _OBSTACLE_COLOR
            bx0 = max(0,         int(d['x1']) // BLOCK_SIZE)
            bx1 = min(bcols - 1, int(d['x2']) // BLOCK_SIZE)
            by0 = max(0,         int(d['y1']) // BLOCK_SIZE + cam_off)
            by1 = min(brows - 1, int(d['y2']) // BLOCK_SIZE + cam_off)
            for by in range(by0, by1 + 1):
                for bx in range(bx0, bx1 + 1):
                    z = float(grid[by, bx])
                    if z <= 0 or z > MAP_Z_RANGE:
                        continue
                    u      = bx * BLOCK_SIZE + BLOCK_SIZE // 2
                    x_real = (u - cx) * z / fx
                    mx     = int(np.clip(MAP_W // 2 + x_real * MAP_W / MAP_X_RANGE, 0, MAP_W - 1))
                    my     = int(np.clip(MAP_H      - z       * MAP_H / MAP_Z_RANGE, 0, MAP_H - 1))
                    cv2.circle(map_img, (mx, my), 2, color, -1)
        else:
            u_c   = (int(d['x1']) + int(d['x2'])) // 2
            roi   = depth_smooth[max(0, int(d['y1'])):max(0, int(d['y2'])),
                                 max(0, int(d['x1'])):max(0, int(d['x2']))]
            valid = roi[roi > 0]
            if valid.size == 0:
                continue
            z = float(np.median(valid))
            if z <= 0 or z > MAP_Z_RANGE:
                continue
            x_real = (u_c - cx) * z / fx
            mx = int(np.clip(MAP_W // 2 + x_real * MAP_W / MAP_X_RANGE, 0, MAP_W - 1))
            my = int(np.clip(MAP_H      - z       * MAP_H / MAP_Z_RANGE, 0, MAP_H - 1))
            cv2.circle(map_img, (mx, my), 7, _OBJECT_COLOR, -1)
            cv2.putText(map_img, d['label'], (mx + 9, my + 4), _FONT, 0.32, _OBJECT_COLOR, 1)


def _draw_path_on_canvas(canvas_640, path):
    """Cyan path blend on the camera canvas (yaw-extraction rows in yellow)."""
    yaw_rows = frozenset(YAW_ROWS)
    for bx, by in path:
        color = _YAW_COLOR if by in yaw_rows else _PATH_COLOR
        x0, y0 = bx * BLOCK_SIZE, by * BLOCK_SIZE
        roi   = canvas_640[y0:y0 + BLOCK_SIZE, x0:x0 + BLOCK_SIZE]
        block = np.full((BLOCK_SIZE, BLOCK_SIZE, 3), color, dtype=np.uint8)
        cv2.addWeighted(roi, 0.5, block, 0.5, 0, roi)


def _draw_midpoint_on_canvas(canvas_640, midpoint):
    """Magenta corridor-midpoint block on the camera canvas (blended fill + outline)."""
    bx, by = int(midpoint[0]), int(midpoint[1])
    x0, y0 = bx * BLOCK_SIZE, by * BLOCK_SIZE
    roi   = canvas_640[y0:y0 + BLOCK_SIZE, x0:x0 + BLOCK_SIZE]
    block = np.full(roi.shape, _MIDPOINT_COLOR, dtype=np.uint8)
    cv2.addWeighted(roi, 0.4, block, 0.6, 0, roi)
    cv2.rectangle(canvas_640, (x0, y0),
                  (x0 + BLOCK_SIZE - 1, y0 + BLOCK_SIZE - 1), _MIDPOINT_COLOR, 2)


def _draw_path_on_map(map_img, path, block_map_pos):
    """Path polyline + endpoint marker on the top-down map."""
    pts = [block_map_pos[node] for node in path if node in block_map_pos]
    if len(pts) >= 2:
        cv2.polylines(map_img, [np.array(pts, dtype=np.int32).reshape(-1, 1, 2)],
                      False, _PATH_COLOR, 1)
    if pts:
        cv2.circle(map_img, pts[-1], 4, _PATH_COLOR, -1)


# ── compositors (two independent images) ─────────────────────────────────────

def render_canvas(color_bgr, depth_smooth, grid, detections, path, obstacle_labels,
                  nav_u9=float('nan'), nav_u10=float('nan'),
                  has_path=False, obstacle_near=False, midpoint=None):
    """Render the 640x640 camera canvas: depth contours + YOLO boxes + A* path + grid.

    ``midpoint`` (bx, by) or None highlights the corridor attraction midpoint (magenta block).
    """
    canvas_640 = np.zeros((GRID_H, COLOR_W, 3), dtype=np.uint8)
    base = color_bgr if color_bgr is not None else depth_colormap(depth_smooth)
    canvas_640[GRID_H - base.shape[0]:] = base

    # Detections live in colour-pixel space; the base frame is bottom-aligned in the
    # 640-tall canvas, so the colour→grid shift is derived from the actual frame height
    # (0 for a 640x640 banded view, 160 for a plain 640x480 down-view).
    y_offset = GRID_H - base.shape[0]
    draw_contours(canvas_640, grid)
    draw_detections(canvas_640, detections, depth_smooth, obstacle_labels, y_offset=y_offset)
    if path:
        _draw_path_on_canvas(canvas_640, path)
    if midpoint is not None:
        _draw_midpoint_on_canvas(canvas_640, midpoint)
    draw_grid_labels(canvas_640)
    draw_hud(canvas_640, nav_u9, nav_u10, has_path, obstacle_near)
    return canvas_640


def _draw_goal_cross(map_img, heading_deg, range_mm=GOAL_CROSS_RANGE_MM):
    """Mark the true corrected heading with a cross ``range_mm`` from the robot.

    The robot is at the bottom-centre of the map; a point ``range_mm`` away at bearing
    ``heading_deg`` (phase3: + = right of optical axis) projects to:
        x_mm = range*sin(θ)  (lateral, + = right)
        z_mm = range*cos(θ)  (forward)
    then onto the map with the same x/z scales used by build_top_view. The marker is
    clipped to the map so a heading near/over the edge still appears at the boundary.
    """
    th = math.radians(heading_deg)
    x_mm = range_mm * math.sin(th)
    z_mm = range_mm * math.cos(th)
    mx = int(np.clip(MAP_W // 2 + x_mm * MAP_W / MAP_X_RANGE, 0, MAP_W - 1))
    my = int(np.clip(MAP_H      - z_mm * MAP_H / MAP_Z_RANGE, 0, MAP_H - 1))
    cv2.drawMarker(map_img, (mx, my), _GOAL_CROSS_COLOR, cv2.MARKER_CROSS, 14, 2)
    cv2.putText(map_img, f"goal {heading_deg:+.0f}deg", (mx + 8, my - 6),
                _FONT, 0.34, _GOAL_CROSS_COLOR, 1, cv2.LINE_AA)


def render_map(grid, detections, depth_smooth, path, fx, fy, cx, cy, obstacle_labels,
               cam_off=CAM_OFF, goal_heading_deg=None, midpoint=None):
    """Render the top-down occupancy map (MAP_H x MAP_W) as its own image.

    ``cam_off`` = (GRID_H - color_height)//BLOCK_SIZE, the colour→grid block-row shift.
    ``goal_heading_deg`` (deg, or None) draws a cross at the true corrected heading
    1 m out — shown even when the A* goal column clamped to the grid edge.
    ``midpoint`` (bx, by) or None marks the corridor attraction midpoint (magenta dot).
    """
    map_img, block_map_pos = build_top_view(grid, fx, fy, cx, cy)
    mark_detections_map(map_img, detections, depth_smooth, grid, fx, fy, cx, cy,
                        obstacle_labels, cam_off=cam_off)
    if path:
        _draw_path_on_map(map_img, path, block_map_pos)
    if goal_heading_deg is not None:
        _draw_goal_cross(map_img, goal_heading_deg)
    if midpoint is not None:
        mp = tuple(midpoint)
        pos = block_map_pos.get(mp)
        if pos is None:                        # cell omitted (depth > MAP_Z_RANGE): project + clamp
            bx, by = int(mp[0]), int(mp[1])
            z = float(grid[by, bx])
            if z > 0:
                u      = bx * BLOCK_SIZE + BLOCK_SIZE // 2
                x_real = (u - cx) * z / fx
                mx  = int(np.clip(MAP_W // 2 + x_real * MAP_W / MAP_X_RANGE, 0, MAP_W - 1))
                my  = int(np.clip(MAP_H      - z       * MAP_H / MAP_Z_RANGE, 0, MAP_H - 1))
                pos = (mx, my)
        if pos is not None:
            cv2.circle(map_img, pos, 5, _MIDPOINT_COLOR, -1)
    return map_img
