#!/usr/bin/env python3
"""
Phase 3 viewer — A* local path planning on 16×16 block grid.
Extends Phase 2 (YOLO + depth contour) with:
  - obstacle mask: depth threshold + YOLO obstacle blocks (combined)
  - proximity penalty: blocks near obstacles incur extra A* cost
  - A* path: 8-directional, Euclidean heuristic, start=(8,15), goal=by=4 or by=5
  - nav_yaw: extracted from path at by=9,10, for go_step yaw argument

Run with sudo (macOS UVC driver conflict):
    sudo /Users/xingyuzheng/ultralytics_env/bin/python3 phase3_viewer.py
"""

import heapq
import math
import queue
import threading
import time
from collections import deque

import cv2
import numpy as np
from pyorbbecsdk import Config, OBFormat, OBSensorType, Pipeline

# ── tunable config ────────────────────────────────────────────────────────────
BLOCK_SIZE = 40
TEMPORAL_N = 10
THRESHOLDS = [240, 260, 280, 300, 330, 360, 400, 500, 700, 1000]
COLOR_W, COLOR_H = 640, 480

# ── YOLO config ───────────────────────────────────────────────────────────────
YOLO_MODEL_PATH = "best.pt"
YOLO_CONF       = 0.40
YOLO_IMGSZ      = 256
YOLO_CLASSES    = ["obstacle", "football", "goalpost"]
_DET_COLORS     = [(0, 0, 220), (0, 220, 0), (220, 100, 0)]   # BGR per class

# ── path planning config ──────────────────────────────────────────────────────
OBSTACLE_DIST = 200    # mm; blocks with depth < this are obstacles
PROX_MAX_DIST = 4.0    # blocks; proximity penalty drops to 0 beyond this
PROX_WEIGHT   = 10.0    # max penalty value for block immediately adjacent to obstacle
_CAM_OFF      = (640 - COLOR_H) // BLOCK_SIZE   # = 4: YOLO y-coord → 16×16 by

_DIRS  = [(0,-1),(0,1),(-1,0),(1,0),(-1,-1),(1,-1),(-1,1),(1,1)]
_DCOST = [1.0, 1.0, 1.0, 1.0,
          math.sqrt(2), math.sqrt(2), math.sqrt(2), math.sqrt(2)]


# ── pipeline setup ────────────────────────────────────────────────────────────

def open_pipeline():
    pipeline = Pipeline()
    config   = Config()

    color_list    = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    color_profile = color_list.get_video_stream_profile(COLOR_W, COLOR_H, OBFormat.MJPG, 15)
    config.enable_stream(color_profile)

    depth_list    = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    depth_profile = depth_list.get_video_stream_profile(640, 480, OBFormat.Y16, 15)
    config.enable_stream(depth_profile)

    try:
        from pyorbbecsdk import OBAlignMode
        supported = pipeline.get_d2c_depth_profile_list(color_profile, OBAlignMode.HW_MODE)
        if supported.get_count() > 0:
            config.set_align_mode(OBAlignMode.HW_MODE)
            print("Align : Hardware D2C")
        else:
            config.set_align_mode(OBAlignMode.SW_MODE)
            print("Align : Software D2C")
    except Exception as e:
        print(f"Align : skipped ({e})")

    try:
        pipeline.enable_frame_sync()
        print("Sync  : enabled")
    except Exception as e:
        print(f"Sync  : skipped ({e})")

    pipeline.start(config)
    print(f"Color : {color_profile.get_width()}×{color_profile.get_height()}"
          f"  @{color_profile.get_fps()}fps  {color_profile.get_format()}")
    print(f"Depth : {depth_profile.get_width()}×{depth_profile.get_height()}"
          f"  @{depth_profile.get_fps()}fps")
    return pipeline


# ── frame decode ──────────────────────────────────────────────────────────────

def decode_color(frame):
    data = np.frombuffer(frame.get_data(), dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def decode_depth_mm(frame):
    raw   = np.frombuffer(frame.get_data(), dtype=np.uint16)
    scale = frame.get_depth_scale()
    depth = (raw.astype(np.float32) * scale).astype(np.uint16)
    return depth.reshape(frame.get_height(), frame.get_width()), scale


# ── contour drawing ───────────────────────────────────────────────────────────

_CONTOUR_COLORS = [
    (0,   0,   255),
    (0,   80,  255),
    (0,   165, 255),
    (0,   220, 255),
    (0,   255, 180),
    (0,   255, 80),
    (60,  255, 0),
    (200, 255, 0),
    (255, 200, 0),
    (255, 80,  0),
]

_depth_history: deque = deque(maxlen=TEMPORAL_N)
_rng = np.random.default_rng()


def _to_block(depth_smooth):
    """Monte Carlo block sampling: 10 random points per block, vectorized."""
    h, w = depth_smooth.shape
    brows, bcols = h // BLOCK_SIZE, w // BLOCK_SIZE
    dr = _rng.integers(0, BLOCK_SIZE, size=10)
    dc = _rng.integers(0, BLOCK_SIZE, size=10)
    samples = np.zeros((10, brows, bcols), dtype=np.float32)
    for i in range(10):
        rows = np.arange(brows) * BLOCK_SIZE + dr[i]
        cols = np.arange(bcols) * BLOCK_SIZE + dc[i]
        samples[i] = depth_smooth[np.ix_(rows, cols)].astype(np.float32)
    valid = samples > 0
    count = valid.sum(axis=0)
    total = (samples * valid).sum(axis=0)
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(count > 0, total / count, 0.0).astype(np.float32)


def draw_contours(canvas, depth_smooth):
    small = _to_block(depth_smooth)
    _depth_history.append(small)
    temporal_mean = np.mean(np.stack(_depth_history, axis=0), axis=0)
    small_u16 = temporal_mean.astype(np.uint16)
    valid = small_u16 > 0

    for i, thresh in enumerate(THRESHOLDS):
        mask = valid & (small_u16 < thresh)
        color = _CONTOUR_COLORS[i]
        hy, hx = np.where(mask[:-1] != mask[1:])
        for by, bx in zip(hy, hx):
            y = (by + 1) * BLOCK_SIZE
            canvas[y, bx * BLOCK_SIZE:(bx + 1) * BLOCK_SIZE] = color
        vy, vx = np.where(mask[:, :-1] != mask[:, 1:])
        for by, bx in zip(vy, vx):
            x = (bx + 1) * BLOCK_SIZE
            canvas[by * BLOCK_SIZE:(by + 1) * BLOCK_SIZE, x] = color

    return temporal_mean


# ── top-down map ──────────────────────────────────────────────────────────────

MAP_W, MAP_H = 400, 350
MAP_X_RANGE  = 4400
MAP_Z_RANGE  = 2500
_FX, _FY     = 367.011, 366.836
_CX, _CY     = 315.780, 237.146


def _band_color(z_mm):
    for i, t in enumerate(THRESHOLDS):
        if z_mm < t:
            return _CONTOUR_COLORS[i]
    return _CONTOUR_COLORS[-1]


def build_top_view(temporal_mean):
    map_img = np.full((MAP_H, MAP_W, 3), 25, dtype=np.uint8)
    block_map_pos = {}

    for thresh in THRESHOLDS:
        if thresh > MAP_Z_RANGE:
            break
        my = MAP_H - int(thresh * MAP_H / MAP_Z_RANGE)
        if 0 <= my < MAP_H:
            cv2.line(map_img, (0, my), (MAP_W, my), (55, 55, 55), 1)
            cv2.putText(map_img, f"{thresh}", (2, my - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (100, 100, 100), 1)

    brows, bcols = temporal_mean.shape
    for by in range(brows):
        for bx in range(bcols):
            z = float(temporal_mean[by, bx])
            if z <= 0 or z > MAP_Z_RANGE:
                continue
            u      = bx * BLOCK_SIZE + BLOCK_SIZE // 2
            x_real = (u - _CX) * z / _FX
            map_x  = MAP_W // 2 + int(x_real * MAP_W / MAP_X_RANGE)
            map_y  = MAP_H      - int(z       * MAP_H / MAP_Z_RANGE)
            block_map_pos[(bx, by)] = (map_x, map_y)
            half_w = max(1, int(BLOCK_SIZE * z / _FX * MAP_W / MAP_X_RANGE / 2))
            half_h = max(1, int(BLOCK_SIZE * z / _FY * MAP_H / MAP_Z_RANGE / 2))
            cv2.rectangle(map_img, (map_x - half_w, map_y - half_h),
                                   (map_x + half_w, map_y + half_h), _band_color(z), -1)

    cv2.drawMarker(map_img, (MAP_W // 2, MAP_H - 4), (255, 255, 255),
                   cv2.MARKER_TRIANGLE_UP, 8, 1)
    return map_img, block_map_pos


# ── HUD ──────────────────────────────────────────────────────────────────────

_FONT = cv2.FONT_HERSHEY_SIMPLEX


def draw_hud(canvas, fps, nav_yaw=None):
    n_acc = len(_depth_history)
    cv2.putText(canvas, f"{fps:.1f} fps  t={n_acc}/{TEMPORAL_N}", (8, 22),
                _FONT, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
    if nav_yaw is not None:
        cv2.putText(canvas, f"yaw={nav_yaw:+.1f}deg", (8, 44),
                    _FONT, 0.55, (0, 220, 220), 1, cv2.LINE_AA)


# ── navigation grid overlay ───────────────────────────────────────────────────

_GRID_OVERLAY: "np.ndarray | None" = None


def _get_grid_overlay() -> np.ndarray:
    global _GRID_OVERLAY
    if _GRID_OVERLAY is None:
        img = np.zeros((640, 640, 3), dtype=np.uint8)
        for by in range(16):
            for bx in range(16):
                cx = bx * BLOCK_SIZE + BLOCK_SIZE // 2
                cy = by * BLOCK_SIZE + BLOCK_SIZE // 2
                label = f"{bx},{by}"
                cv2.putText(img, label, (cx - 10, cy + 4), _FONT, 0.22, (80, 80, 80), 1)
        _GRID_OVERLAY = img
    return _GRID_OVERLAY


def draw_grid_labels(canvas_640: np.ndarray) -> None:
    overlay = _get_grid_overlay()
    mask = overlay > 0
    canvas_640[mask] = overlay[mask]


# ── YOLO publisher thread ─────────────────────────────────────────────────────

def yolo_publisher(color_q: queue.Queue, roi_q: queue.Queue, stop_evt: threading.Event):
    from ultralytics import YOLO
    model = YOLO(YOLO_MODEL_PATH)
    names = model.names
    print(f"[YOLO] model loaded  classes={names}")

    while not stop_evt.is_set():
        try:
            ts, color_bgr = color_q.get(timeout=0.5)
        except queue.Empty:
            continue

        res  = model(color_bgr, verbose=False, imgsz=YOLO_IMGSZ, conf=YOLO_CONF)
        dets = []
        for r in res:
            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                cls = int(b.cls[0])
                dets.append({
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                    "conf": round(float(b.conf[0]), 3),
                    "cls": cls,
                    "label": names.get(cls, str(cls)),
                })

        msg = {"ts": ts, "detections": dets}
        try:
            roi_q.put_nowait(msg)
        except queue.Full:
            pass


# ── detection overlay ─────────────────────────────────────────────────────────

def draw_detections(canvas, roi_msg, depth_smooth, y_offset=0):
    for d in roi_msg.get("detections", []):
        x1, x2 = d["x1"], d["x2"]
        y1_raw, y2_raw = d["y1"], d["y2"]
        y1, y2 = y1_raw + y_offset, y2_raw + y_offset
        color = _DET_COLORS[d["cls"]]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        roi   = depth_smooth[max(0, y1_raw):y2_raw, max(0, x1):x2]
        valid = roi[roi > 0]
        dist  = f"{int(np.median(valid))}mm" if valid.size > 0 else "--"
        label = f"{d['label']} {d['conf']:.2f} {dist}"
        cv2.putText(canvas, label, (x1, max(y1 - 6, y_offset + 12)),
                    _FONT, 0.45, color, 1, cv2.LINE_AA)


OBSTACLE_LABEL = "obstacle"


def mark_detections_map(map_img, roi_msg, depth_smooth, temporal_mean):
    brows, bcols = temporal_mean.shape

    for d in roi_msg.get("detections", []):
        color = _DET_COLORS[d["cls"] % len(_DET_COLORS)]

        if d["label"] == OBSTACLE_LABEL:
            bx0 = max(0,         d["x1"] // BLOCK_SIZE)
            bx1 = min(bcols - 1, d["x2"] // BLOCK_SIZE)
            by0 = max(0,         d["y1"] // BLOCK_SIZE + _CAM_OFF)
            by1 = min(brows - 1, d["y2"] // BLOCK_SIZE + _CAM_OFF)
            for by in range(by0, by1 + 1):
                for bx in range(bx0, bx1 + 1):
                    z = float(temporal_mean[by, bx])
                    if z <= 0 or z > MAP_Z_RANGE:
                        continue
                    u      = bx * BLOCK_SIZE + BLOCK_SIZE // 2
                    x_real = (u - _CX) * z / _FX
                    mx     = int(np.clip(MAP_W // 2 + x_real * MAP_W / MAP_X_RANGE, 0, MAP_W - 1))
                    my     = int(np.clip(MAP_H      - z       * MAP_H / MAP_Z_RANGE, 0, MAP_H - 1))
                    half_w = max(1, int(BLOCK_SIZE * z / _FX * MAP_W / MAP_X_RANGE / 2))
                    half_h = max(1, int(BLOCK_SIZE * z / _FY * MAP_H / MAP_Z_RANGE / 2))
                    cv2.rectangle(map_img,
                                  (mx - half_w, my - half_h), (mx + half_w, my + half_h),
                                  color, -1)
        else:
            u_c   = (d["x1"] + d["x2"]) // 2
            roi   = depth_smooth[max(0, d["y1"]):d["y2"], max(0, d["x1"]):d["x2"]]
            valid = roi[roi > 0]
            if valid.size == 0:
                continue
            z = float(np.median(valid))
            if z <= 0 or z > MAP_Z_RANGE:
                continue
            x_real = (u_c - _CX) * z / _FX
            mx = int(np.clip(MAP_W // 2 + x_real * MAP_W / MAP_X_RANGE, 0, MAP_W - 1))
            my = int(np.clip(MAP_H      - z       * MAP_H / MAP_Z_RANGE, 0, MAP_H - 1))
            cv2.circle(map_img, (mx, my), 7, color, -1)
            cv2.putText(map_img, d["label"], (mx + 9, my + 4), _FONT, 0.32, color, 1)


# ── path planning ─────────────────────────────────────────────────────────────

def build_obstacle_mask(temporal_mean, latest_roi):
    """Build 16×16 bool mask: True = obstacle (depth-based OR YOLO obstacle)."""
    mask = np.zeros((16, 16), dtype=bool)
    mask[:13] |= (temporal_mean[:13] > 0) & (temporal_mean[:13] < OBSTACLE_DIST)
    brows, bcols = temporal_mean.shape
    for d in latest_roi.get("detections", []):
        if d["label"] == OBSTACLE_LABEL:
            bx0 = max(0,         d["x1"] // BLOCK_SIZE)
            bx1 = min(bcols - 1, d["x2"] // BLOCK_SIZE)
            by0 = max(0,         d["y1"] // BLOCK_SIZE + _CAM_OFF)
            by1 = min(brows - 1, d["y2"] // BLOCK_SIZE + _CAM_OFF)
            mask[by0:by1 + 1, bx0:bx1 + 1] = True
    return mask


def compute_proximity_penalty(obstacle_mask):
    """Per-block penalty inversely proportional to distance to nearest obstacle."""
    free    = (~obstacle_mask).astype(np.uint8)
    dist    = cv2.distanceTransform(free, cv2.DIST_L2, 3)
    penalty = np.where(dist < PROX_MAX_DIST,
                       PROX_WEIGHT * (1.0 - dist / PROX_MAX_DIST),
                       0.0).astype(np.float32)
    penalty[obstacle_mask] = 0.0
    return penalty


def find_goal(obstacle_mask, temporal_mean):
    """Find nearest clear block to center in by=4 (fallback: by=5, then hardcoded)."""
    for by_g in [4, 5]:
        for bx_g in sorted(range(16), key=lambda b: abs(b - 7.5)):
            if not obstacle_mask[by_g, bx_g] and temporal_mean[by_g, bx_g] > 0:
                return (bx_g, by_g)
    return (8, 4)


def astar(obstacle_mask, penalty, goal):
    """8-directional A* from (8,15) to goal. Returns path list or None."""
    start  = (8, 15)
    open_q = [(0.0, start)]
    came_from = {}
    g = {start: 0.0}

    while open_q:
        _, cur = heapq.heappop(open_q)
        if cur == goal:
            path, node = [], cur
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            return path[::-1]

        bx, by = cur
        for (dbx, dby), step in zip(_DIRS, _DCOST):
            nb = (bx + dbx, by + dby)
            if not (0 <= nb[0] < 16 and 0 <= nb[1] < 16):
                continue
            if obstacle_mask[nb[1], nb[0]]:
                continue
            ng = g[cur] + step + float(penalty[nb[1], nb[0]])
            if ng < g.get(nb, 1e9):
                g[nb] = ng
                came_from[nb] = cur
                h = math.sqrt((nb[0] - goal[0])**2 + (nb[1] - goal[1])**2)
                heapq.heappush(open_q, (ng + h, nb))

    return None


def path_yaw_from_rows(path, target_rows=(9, 10)):
    """Extract yaw (degrees) from average bx of path steps at target_rows."""
    bx_list = [bx for bx, by in path if by in target_rows]
    if not bx_list:
        bx_list = [path[min(3, len(path) - 1)][0]]
    bx = max(bx_list, key=lambda b: abs(b * BLOCK_SIZE + BLOCK_SIZE // 2 - _CX))
    u  = bx * BLOCK_SIZE + BLOCK_SIZE // 2
    return math.degrees(math.atan((u - _CX) / _FX))


_YAW_ROWS = frozenset((9, 10))   # rows used for yaw extraction

def draw_path(canvas_640, map_img, path, block_map_pos):
    """Highlight path on camera canvas (cyan blend) and map (polyline).
    Yaw-extraction rows (by=9,10) are drawn in yellow instead of cyan."""
    PATH_COLOR = (0, 220, 220)    # cyan
    YAW_COLOR  = (0, 200, 255)    # yellow (BGR)
    for bx, by in path:
        color = YAW_COLOR if by in _YAW_ROWS else PATH_COLOR
        x0, y0 = bx * BLOCK_SIZE, by * BLOCK_SIZE
        roi   = canvas_640[y0:y0 + BLOCK_SIZE, x0:x0 + BLOCK_SIZE]
        block = np.full((BLOCK_SIZE, BLOCK_SIZE, 3), color, dtype=np.uint8)
        cv2.addWeighted(roi, 0.5, block, 0.5, 0, roi)

    pts = [block_map_pos[node] for node in path if node in block_map_pos]
    if len(pts) >= 2:
        cv2.polylines(map_img,
                      [np.array(pts, dtype=np.int32).reshape(-1, 1, 2)],
                      False, PATH_COLOR, 1)
    if pts:
        cv2.circle(map_img, pts[-1], 4, PATH_COLOR, -1)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    color_q  = queue.Queue(maxsize=1)
    roi_q    = queue.Queue(maxsize=1)
    stop_evt = threading.Event()

    yolo_thread = threading.Thread(
        target=yolo_publisher,
        args=(color_q, roi_q, stop_evt),
        daemon=True,
    )
    yolo_thread.start()

    pipeline   = open_pipeline()
    fps        = 0.0
    prev_ts    = None
    first      = True
    latest_roi = {"detections": []}
    nav_yaw    = None

    print("\nPress 'q' to quit\n")
    try:
        while True:
            frames = pipeline.wait_for_frames(200)
            if not frames:
                continue

            cf = frames.get_color_frame()
            df = frames.get_depth_frame()
            if not cf or not df:
                continue

            t               = time.time()
            color_bgr       = decode_color(cf)
            depth_mm, scale = decode_depth_mm(df)

            if color_bgr is None:
                continue

            if first:
                print(f"depth_scale = {scale}")
                print(f"color shape = {color_bgr.shape}   depth shape = {depth_mm.shape}")
                first = False

            try:
                color_q.put_nowait((t, color_bgr.copy()))
            except queue.Full:
                pass

            try:
                latest_roi = roi_q.get_nowait()
            except queue.Empty:
                pass

            dh, dw = depth_mm.shape
            ch, cw = color_bgr.shape[:2]
            if (dh, dw) != (ch, cw):
                depth_mm = cv2.resize(depth_mm, (cw, ch), interpolation=cv2.INTER_NEAREST)

            depth_smooth = cv2.medianBlur(depth_mm, 3)
            depth_640 = np.vstack([np.repeat(depth_smooth[0:1], 640 - COLOR_H, axis=0),
                                   depth_smooth])

            canvas_640 = np.zeros((640, COLOR_W, 3), dtype=np.uint8)
            canvas_640[640 - COLOR_H:] = color_bgr

            temporal_mean = draw_contours(canvas_640, depth_640)

            draw_detections(canvas_640, latest_roi, depth_smooth, y_offset=640 - COLOR_H)

            if prev_ts:
                fps = 0.8 * fps + 0.2 / (t - prev_ts)
            prev_ts = t

            # top-down map
            map_img, block_map_pos = build_top_view(temporal_mean)
            mark_detections_map(map_img, latest_roi, depth_smooth, temporal_mean)

            # path planning
            obstacle_mask = build_obstacle_mask(temporal_mean, latest_roi)
            penalty       = compute_proximity_penalty(obstacle_mask)
            goal          = find_goal(obstacle_mask, temporal_mean)
            path          = astar(obstacle_mask, penalty, goal)

            nav_yaw = None
            if path:
                nav_yaw = path_yaw_from_rows(path)
                draw_path(canvas_640, map_img, path, block_map_pos)

            draw_hud(canvas_640, fps, nav_yaw)
            draw_grid_labels(canvas_640)

            map_pad = np.zeros((640 - MAP_H, MAP_W, 3), dtype=np.uint8)
            display = cv2.hconcat([canvas_640, np.vstack([map_img, map_pad])])

            n_obs = int(obstacle_mask.sum())
            if nav_yaw is not None:
                print(f"\r{fps:.1f} fps  yaw={nav_yaw:+.1f}  obs={n_obs}", end="", flush=True)
            else:
                print(f"\r{fps:.1f} fps  no path  obs={n_obs}  goal={goal}", end="", flush=True)

            cv2.imshow("Phase3 — A* nav  (q=quit)", display)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        yolo_thread.join(timeout=3)
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\nStopped.")


if __name__ == "__main__":
    main()
