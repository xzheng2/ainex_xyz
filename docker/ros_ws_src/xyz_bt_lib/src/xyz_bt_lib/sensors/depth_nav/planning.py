#!/usr/bin/env python3
"""Pure local-mapping + A* planning functions (ported from phase3_viewer.py).

ROS-free and display-free. Every function here is side-effect-free except for
the module-level numpy RNG used by ``to_block_grid`` for Monte-Carlo depth
sampling (randomness only — no observable state mutation). cv2 is used purely
for the distance transform, never for display.

Pipeline (mirrors phase3_viewer.py):

    depth_640 (640x640 uint16 mm)
        -> to_block_grid()         16x16 per-frame block grid (mm)
        -> temporal_mean(history)  time-smoothed grid (mm)
        -> build_obstacle_mask()   16x16 bool (depth-threshold OR YOLO boxes)
        -> compute_proximity_penalty()  16x16 float A* cost padding
        -> find_goal()             nearest clear block on a goal row
        -> astar()                 8-directional path  start -> goal
                                   (start = object_start_cell() from a YOLO object
                                    centre when available, else default (8,15))
        -> path_cols_from_rows()   path pixel-x (u9, u10) at rows 9,10 (consumer computes yaw)

Grid geometry is only valid at a fixed camera pitch (see depth_nav_node.py):
the 480-high depth frame is top-padded by 160 px (= 4 blocks) so the bottom 12
rows are real depth and the start cell (8, 15) sits at the robot's feet.
"""
import heapq
import math
from collections import deque

import cv2
import numpy as np

# ── tunable constants (defaults; nodes may override via parameters) ───────────
BLOCK_SIZE    = 40
TEMPORAL_N    = 3
OBSTACLE_HOLD_TICKS = 10                      # keep an obstacle block in the mask this many
                                             # ticks after it stops being detected; 0 = off
THRESHOLDS    = [240, 260, 280, 300, 330, 360, 400, 500, 700, 1000]
COLOR_W       = 640
COLOR_H       = 480
GRID_H        = 640                          # padded depth height fed to grid
OBSTACLE_DIST = 320                          # mm; blocks nearer than this = obstacle
PROX_MAX_DIST = 4.0                          # blocks; penalty fades to 0 beyond this
PROX_WEIGHT   = 8.0                         # max penalty for a block adjacent to obstacle
# Separate near-feet halo: rows [NEAR_EXTRA_ROW_MIN, NEAR_EXTRA_ROW_MAX] get their OWN
# reach ("distance") + magnitude ("scores"), independent of the shared values above.
NEAR_EXTRA_ROW_MIN = 13                       # by rows (inclusive) governed by the near halo
NEAR_EXTRA_ROW_MAX = 15                       # clamped to the grid; rows 0..15 (15 = feet)
NEAR_PROX_MAX_DIST = 10.0                     # "distance": near-band halo reach (blocks)
NEAR_PROX_WEIGHT   = 16.0                     # "scores": near-band halo magnitude (peak, d->0)
NEAR_PROX_MIN      = 8.0                       # floor: min penalty for in-reach near blocks
                                             # (d < max_dist); 0 = no floor (pure falloff to 0)
# YOLO color-y (480 space) -> 16x16 block-row offset = (640-480)//40 = 4
CAM_OFF       = (GRID_H - COLOR_H) // BLOCK_SIZE

START_CELL          = (8, 15)                # (bx, by) A* start at robot feet
GOAL_ROWS           = (0, 1)                 # candidate goal rows (nearest clear to centre); row 0 = farthest
YAW_ROWS            = (7, 10)                # rows sampled for steering (published as nav_u9/nav_u10)
DEPTH_MASK_MAX_ROW  = 13                     # depth obstacles only considered above this row

_DIRS  = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (1, -1), (-1, 1), (1, 1)]
_DCOST = [1.0, 1.0, 1.0, 1.0,
          math.sqrt(2), math.sqrt(2), math.sqrt(2), math.sqrt(2)]

_rng = np.random.default_rng()


# ── depth → block grid ────────────────────────────────────────────────────────

def new_history(maxlen: int = TEMPORAL_N) -> deque:
    """Return an empty temporal-smoothing history deque for a node to own."""
    return deque(maxlen=maxlen)


def to_block_grid(depth_640, block_size: int = BLOCK_SIZE):
    """Monte-Carlo block sampling: 10 random points per block, vectorized.

    depth_640: (H, W) uint16/float depth in mm. Returns (H//bs, W//bs) float32
    grid of mean valid (non-zero) depth per block (0.0 where no valid samples).
    """
    h, w = depth_640.shape
    brows, bcols = h // block_size, w // block_size
    dr = _rng.integers(0, block_size, size=10)
    dc = _rng.integers(0, block_size, size=10)
    samples = np.zeros((10, brows, bcols), dtype=np.float32)
    for i in range(10):
        rows = np.arange(brows) * block_size + dr[i]
        cols = np.arange(bcols) * block_size + dc[i]
        samples[i] = depth_640[np.ix_(rows, cols)].astype(np.float32)
    valid = samples > 0
    count = valid.sum(axis=0)
    total = (samples * valid).sum(axis=0)
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(count > 0, total / count, 0.0).astype(np.float32)


def temporal_mean(history) -> np.ndarray:
    """Mean of the grids currently in ``history`` (a deque of equal-shaped grids)."""
    return np.mean(np.stack(list(history), axis=0), axis=0)


# ── obstacle mask + proximity penalty ─────────────────────────────────────────

def build_obstacle_mask(grid, obstacle_boxes=None,
                        obstacle_dist: float = OBSTACLE_DIST,
                        cam_off: int = CAM_OFF,
                        block_size: int = BLOCK_SIZE,
                        depth_mask_max_row: int = DEPTH_MASK_MAX_ROW):
    """Build a (brows, bcols) bool obstacle mask.

    Depth source (always on): blocks with 0 < depth < obstacle_dist in rows
    above ``depth_mask_max_row`` are obstacles.

    YOLO source (optional): ``obstacle_boxes`` is an iterable of (x1, y1, x2, y2)
    corner tuples in colour-pixel coords. Each box marks its covered blocks True
    after adding ``cam_off`` to the block-row, where ``cam_off`` =
    (GRID_H - color_height)//BLOCK_SIZE is supplied by the caller (defaults to CAM_OFF,
    i.e. a 480-high colour frame). Pass None or an empty list to disable the YOLO
    contribution.
    """
    brows, bcols = grid.shape
    mask = np.zeros((brows, bcols), dtype=bool)

    top = min(depth_mask_max_row, brows)
    mask[:top] |= (grid[:top] > 0) & (grid[:top] < obstacle_dist)

    for (x1, y1, x2, y2) in (obstacle_boxes or []):
        bx0 = max(0,         int(x1) // block_size)
        bx1 = min(bcols - 1, int(x2) // block_size)
        by0 = max(0,         int(y1) // block_size + cam_off)
        by1 = min(brows - 1, int(y2) // block_size + cam_off)
        if bx1 >= bx0 and by1 >= by0:
            mask[by0:by1 + 1, bx0:bx1 + 1] = True
    return mask


def compute_proximity_penalty(obstacle_mask,
                              prox_max_dist: float = PROX_MAX_DIST,
                              prox_weight: float = PROX_WEIGHT):
    """Per-block A* penalty inversely proportional to distance to nearest obstacle."""
    free    = (~obstacle_mask).astype(np.uint8)
    dist    = cv2.distanceTransform(free, cv2.DIST_L2, 3)
    penalty = np.where(dist < prox_max_dist,
                       prox_weight * (1.0 - dist / prox_max_dist),
                       0.0).astype(np.float32)
    penalty[obstacle_mask] = 0.0
    return penalty


def compute_proximity_penalty_bx(obstacle_mask,
                                 prox_max_dist: float = PROX_MAX_DIST,
                                 prox_weight: float = PROX_WEIGHT):
    """Horizontal (bx-only) proximity penalty.

    For each row independently (same depth / "同一距离"), penalize a free block by its
    COLUMN distance to the nearest obstacle IN THAT SAME ROW; linear falloff to 0 at
    ``prox_max_dist`` columns. Rows with no obstacle get no penalty. Unlike the radial
    (DIST_L2) version there is NO cross-row (by) coupling — a block is only penalized by
    obstacles at its own depth. Pure — no ROS/BB/side effects.
    """
    brows, bcols = obstacle_mask.shape
    penalty = np.zeros((brows, bcols), dtype=np.float32)
    cols = np.arange(bcols)
    for by in range(brows):
        obs = np.flatnonzero(obstacle_mask[by])
        if obs.size == 0:
            continue
        d = np.abs(cols[:, None] - obs[None, :]).min(axis=1)   # horizontal dist per column
        penalty[by] = np.where(d < prox_max_dist,
                               prox_weight * (1.0 - d / prox_max_dist), 0.0)
    penalty[obstacle_mask] = 0.0
    return penalty


def apply_near_row_extra(obstacle_mask, penalty, prox_mode='horizontal',
                         row_min: int = NEAR_EXTRA_ROW_MIN,
                         row_max: int = NEAR_EXTRA_ROW_MAX,
                         near_prox_max_dist: float = NEAR_PROX_MAX_DIST,
                         near_prox_weight: float = NEAR_PROX_WEIGHT,
                         near_prox_min: float = NEAR_PROX_MIN):
    """Replace the near-feet rows' proximity penalty with a separately-tuned halo.

    Rows ``[row_min, row_max]`` (inclusive, clamped to the grid) are recomputed with a
    DEDICATED reach (``near_prox_max_dist``, the "distance") and peak magnitude
    (``near_prox_weight``, the "scores"), independent of the shared PROX_MAX_DIST /
    PROX_WEIGHT the other rows use, then written over those rows (Replace). ``prox_mode``
    selects the same halo shape as the base ('horizontal' = bx-only same-row, 'radial' =
    2D DIST_L2).

    ``near_prox_min`` is a FLOOR (like the y-controller's y_min): every in-reach block
    (base penalty > 0, i.e. distance < near_prox_max_dist) is raised to at least this
    value, so the near-band penalty is ``clamp(weight*(1 - d/max_dist), near_prox_min,
    near_prox_weight)``. Obstacle cells and out-of-reach blocks (d >= max_dist) stay 0 —
    the floor never lifts a 0, so there is a hard step from the floor to 0 at the reach
    edge. ``near_prox_min`` = 0 disables the floor (pure linear falloff to 0).

    Returns a new array; the input is not mutated. Pure — no ROS/BB/side effects.
    """
    r0 = max(0, int(row_min))
    r1 = min(penalty.shape[0] - 1, int(row_max))
    if r0 > r1:
        return penalty
    fn = compute_proximity_penalty if prox_mode == 'radial' else compute_proximity_penalty_bx
    near = fn(obstacle_mask, prox_max_dist=near_prox_max_dist, prox_weight=near_prox_weight)
    if near_prox_min > 0.0:
        # Floor only the in-reach halo (near > 0); leave obstacle / out-of-reach 0 blocks at 0.
        near = np.where(near > 0.0, np.maximum(near, float(near_prox_min)), 0.0).astype(np.float32)
    out = penalty.copy()
    out[r0:r1 + 1] = near[r0:r1 + 1]
    return out.astype(np.float32)


def apply_obstacle_hold(obstacle_mask, hold_counter, hold_ticks: int = OBSTACLE_HOLD_TICKS):
    """Persist obstacle blocks for ``hold_ticks`` planning ticks after they disappear.

    ``hold_counter`` is a per-block int array (same shape as the mask) the caller owns
    across ticks. The EFFECTIVE mask is ``obstacle_mask | (hold_counter > 0)`` computed
    from the incoming counter, so a block detected at tick t stays an obstacle through
    ticks t+1 .. t+hold_ticks even with no further detection — letting A* keep avoiding it
    across brief drop-outs / FOV exit. The counter is then updated: reset to ``hold_ticks``
    where detected this tick, else decremented by 1 (floored at 0). ``hold_ticks`` <= 0 →
    returns the mask unchanged with a zeroed counter (feature off). Returns
    ``(effective_mask, new_counter)``; inputs are not mutated. Pure — no ROS/BB/side effects.
    """
    if hold_ticks <= 0:
        return obstacle_mask, np.zeros_like(hold_counter)
    effective   = obstacle_mask | (hold_counter > 0)
    new_counter = np.where(obstacle_mask, int(hold_ticks),
                           np.maximum(hold_counter.astype(int) - 1, 0))
    return effective, new_counter


# ── goal selection + A* ────────────────────────────────────────────────────────

def heading_to_goal_col(heading_deg, fx, cx, bcols, block_size: int = BLOCK_SIZE):
    """Body-frame heading (deg) → goal block column.

    Inverse of the pixel→bearing convention used by path_cols_from_rows / the L2
    node (phase3: + = right of optical axis): u = cx + fx*tan(heading);
    col = (u - block_size/2) / block_size. Result clamped to [0, bcols-1].
    Pure — no ROS, no BB, no side effects. heading=0 → centre column → straight.

    The heading is first clamped to the camera FOV (the angles that map to the two
    edge columns) so a heading beyond the view saturates at the *correct* outermost
    block rather than wrapping past tan's 90° asymptote to the opposite edge.
    """
    # FOV edge angles (deg) that map to col 0 and col (bcols-1) centres.
    th_lo = math.degrees(math.atan((block_size / 2.0 - cx) / fx))
    th_hi = math.degrees(math.atan(((bcols - 1) * block_size + block_size / 2.0 - cx) / fx))
    lo, hi = (th_lo, th_hi) if th_lo <= th_hi else (th_hi, th_lo)
    h = max(lo, min(hi, heading_deg))                # clamp into FOV → saturate at edge
    u   = cx + fx * math.tan(math.radians(h))
    col = (u - block_size / 2.0) / block_size
    return int(max(0, min(bcols - 1, round(col))))


def find_goal(obstacle_mask, grid, goal_rows=GOAL_ROWS, goal_col=None):
    """Nearest clear block to a target column on the first viable goal row.

    ``goal_col`` is the column to bias toward (e.g. from heading_to_goal_col).
    When None, biases to the geometric centre — byte-identical to the original
    behaviour. Falls back across ``goal_rows`` in order, then to (target_col, first_row).
    """
    brows, bcols = grid.shape
    centre = (bcols - 1) / 2.0 if goal_col is None else float(goal_col)
    for by_g in goal_rows:
        if not (0 <= by_g < brows):
            continue
        for bx_g in sorted(range(bcols), key=lambda b: abs(b - centre)):
            if not obstacle_mask[by_g, bx_g] and grid[by_g, bx_g] > 0:
                return (bx_g, by_g)
    return (int(round(centre)), goal_rows[0])


def object_start_cell(px, py, bcols, brows, min_row, max_row,
                      block_size: int = BLOCK_SIZE, cam_off: int = CAM_OFF):
    """Map a colour-space object centre (px, py) → A* start block cell.

    Column clamped to [0, bcols-1]; row = py//block_size + cam_off (same mapping as
    build_obstacle_mask, where cam_off = (GRID_H - color_height)//BLOCK_SIZE is supplied
    by the caller). Returns (bx, by) only when the row is within
    [min_row, max_row] (the "available" band), else None so the caller falls back to
    the default start. Pure — no ROS, no BB, no side effects.
    """
    bx = int(max(0, min(bcols - 1, int(px) // block_size)))
    by = int(py) // block_size + cam_off
    if by < min_row or by > max_row:
        return None
    return (bx, int(max(0, min(brows - 1, by))))


def astar(obstacle_mask, penalty, goal, start=START_CELL):
    """8-directional A* from ``start`` to ``goal``. Returns path list or None."""
    brows, bcols = obstacle_mask.shape
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
            if not (0 <= nb[0] < bcols and 0 <= nb[1] < brows):
                continue
            if obstacle_mask[nb[1], nb[0]]:
                continue
            ng = g[cur] + step + float(penalty[nb[1], nb[0]])
            if ng < g.get(nb, 1e9):
                g[nb] = ng
                came_from[nb] = cur
                h = math.sqrt((nb[0] - goal[0]) ** 2 + (nb[1] - goal[1]) ** 2)
                heapq.heappush(open_q, (ng + h, nb))
    return None


# ── mandatory midpoint waypoint (path must pass through it) ──────────────────────

MIDPOINT_ROWS = (1, 5)          # forward band (by1..by5 incl.) scanned for the corridor


def find_midpoint(obstacle_mask, grid, band=MIDPOINT_ROWS):
    """Mandatory-waypoint cell from the largest connected corridor in the forward band.

    Within rows ``band`` = (r_min, r_max) inclusive (r_max = ``by5``, the near band row),
    traversable blocks are obstacle-free AND depth-valid (``~mask & grid>0``). The band is scanned
    in isolation (rows outside it zeroed) so 8-connected components cannot bridge through out-of-band
    rows. The LARGEST component (by cell count) is selected; the midpoint is the horizontal middle
    of that component's cells on row ``r_max`` — returned AS-IS even when that cell is an obstacle
    (the component's ``by5`` cells can straddle one).

    Returns ``(mid_bx, r_max)``, or ``None`` when there is no traversable cell in the band or the
    largest component does not reach row ``r_max`` (feature inert → caller plans start→goal).
    Pure — no ROS/BB/side effects.
    """
    r_min, r_max = int(band[0]), int(band[1])
    brows, _ = grid.shape
    if not (0 <= r_min <= r_max < brows):
        return None
    trav = (~obstacle_mask) & (grid > 0)
    trav_band = np.zeros_like(trav, dtype=np.uint8)
    trav_band[r_min:r_max + 1] = trav[r_min:r_max + 1]
    n_labels, labels = cv2.connectedComponents(trav_band, connectivity=8)
    if n_labels <= 1:                                   # only background → no corridor
        return None
    counts = np.bincount(labels.ravel())
    counts[0] = 0                                       # ignore background label
    largest = int(counts.argmax())
    cols = np.flatnonzero(labels[r_max] == largest)     # component's cells on the near row
    if cols.size == 0:                                  # largest corridor doesn't reach by5
        return None
    mid_bx = int((int(cols.min()) + int(cols.max())) // 2)   # horizontal middle of the by5 span
    return (mid_bx, r_max)


def astar_via(obstacle_mask, penalty, via, goal, start=START_CELL, k: int = 1):
    """Up to ``k`` lowest-cost paths ``start → via → goal`` (mandatory waypoint ``via``).

    ``leg1`` = single best A* ``start→via``; if unreachable returns ``[]`` (no mandatory path).
    ``leg2`` = up to ``k`` shortest A* ``via→goal`` (via ``astar_topk``). Each result is
    ``leg1[:-1] + leg2`` so the via cell appears exactly once. Reuses ``astar`` / ``astar_topk``.
    Pure — no ROS/BB/side effects.
    """
    leg1 = astar(obstacle_mask, penalty, via, start=start)
    if leg1 is None:
        return []
    legs2 = astar_topk(obstacle_mask, penalty, goal, start=via, k=k)
    return [leg1[:-1] + leg2 for leg2 in legs2]


# ── MPC-style multi-candidate path selection ────────────────────────────────────

def astar_topk(obstacle_mask, penalty, goal, start=START_CELL, k: int = 3):
    """Up to ``k`` lowest-cost paths from ``start`` to ``goal`` (K-shortest paths).

    Uses the node-pop-count method (Dijkstra cost, no heuristic): each node may be
    popped up to k times; the i-th time ``goal`` is popped yields the i-th shortest
    path. Returns a list of paths (each a list of (bx, by)) in non-decreasing cost;
    ``results[0]`` equals the plain-``astar`` optimum. Cheap on the 16×16 grid.
    Pure — no ROS/BB/side effects.
    """
    brows, bcols = obstacle_mask.shape
    counts = {}
    open_q = [(0.0, start, (start,))]
    results = []
    while open_q and len(results) < k:
        cost, cur, path = heapq.heappop(open_q)
        if counts.get(cur, 0) >= k:
            continue
        counts[cur] = counts.get(cur, 0) + 1
        if cur == goal:
            results.append(list(path))
            continue
        bx, by = cur
        for (dbx, dby), step in zip(_DIRS, _DCOST):
            nb = (bx + dbx, by + dby)
            if not (0 <= nb[0] < bcols and 0 <= nb[1] < brows):
                continue
            if obstacle_mask[nb[1], nb[0]]:
                continue
            if counts.get(nb, 0) >= k:
                continue
            ncost = cost + step + float(penalty[nb[1], nb[0]])
            heapq.heappush(open_q, (ncost, nb, path + (nb,)))
    return results


def path_descriptors(path, near_row, far_row, center_col, block_size: int = BLOCK_SIZE):
    """Return (near_bx, far_bx, turn) for a candidate path.

    near_bx / far_bx = the path's block column at ``near_row`` / ``far_row`` (None if
    the path misses that row). ``turn`` = the SIGNED mean column offset from
    ``center_col`` over the available near/far rows (negative = left, positive = right;
    None if both rows missing). No ``abs``: the sign is a stable direction indicator that
    does not flip when near/far dominance swaps, so mpc_path_cost can keep a committed
    turn on its side. Pure — no ROS/BB/side effects.
    """
    near_bx = next((bx for bx, by in path if by == near_row), None)
    far_bx  = next((bx for bx, by in path if by == far_row), None)
    offs = [bx - center_col for bx in (near_bx, far_bx) if bx is not None]
    turn = (sum(offs) / len(offs)) if offs else None
    return near_bx, far_bx, turn


def mpc_path_cost(desc, prev, w_near, w_far, w_turn):
    """Temporal-consistency cost of a candidate vs the previous chosen path.

    desc / prev = (near_bx, far_bx, turn). prev=None (first tick) → 0.0. Obstacle
    safety is NOT included here — it is already inside the A* path cost.

    The near/far terms smooth column magnitude symmetrically. The turn term is
    ASYMMETRIC (directional hysteresis): once a side is committed (sign of the
    previous ``pt``), continuing on / further to that side is free, and only motion
    back toward or across to the opposite side is penalized — superlinearly in the
    opposing amount, so "the more it reverses, the more the penalty". A straight
    previous turn (pt == 0) has no committed side yet, so it stays symmetric and can
    settle onto either side. Pure — no ROS/BB/side effects.
    """
    if prev is None:
        return 0.0
    near, far, turn = desc
    pn, pf, pt = prev
    c = 0.0
    if near is not None and pn is not None:
        c += w_near * abs(near - pn)
    if far is not None and pf is not None:
        c += w_far * abs(far - pf)
    if turn is not None and pt is not None:
        if pt == 0:
            c += w_turn * (turn - pt) ** 2                  # no committed side yet
        else:
            opposing = max(0.0, -math.copysign(1.0, pt) * (turn - pt))
            c += w_turn * opposing ** 2                     # only charge for reversing
    return c


def path_cols_from_rows(path, target_rows=YAW_ROWS, block_size: int = BLOCK_SIZE):
    """Return the path's block-centre pixel-x at each row in ``target_rows``.

    For each target row, finds the path cell whose block-row (by) equals that row and
    returns its block-centre pixel-x (``bx*block_size + block_size//2``); ``NaN`` when
    the path does not reach that row. No angle math here — the consumer (the L2 node)
    selects a row and computes the bearing ``atan((u - cx) / fx)`` itself.

    Returns:
        tuple of pixel-x floats, one per row in ``target_rows`` (same order).
        With the default YAW_ROWS=(7, 10) this is ``(u_row7, u_row10)`` — published as
        the message fields nav_u9/nav_u10 (names kept for back-compat).
    """
    nan = float('nan')
    out = []
    for row in target_rows:
        bx = next((bx for bx, by in path if by == row), None)
        out.append(nan if bx is None else float(bx * block_size + block_size // 2))
    return tuple(out)


# ── proximity / "obstacle near" heuristic ───────────────────────────────────────

def is_obstacle_near(obstacle_mask, row_min: int = 12, row_max: int = 15,
                     col_min: int = 6, col_max: int = 9) -> bool:
    """True if any obstacle block lies in the near-front window [rows]x[cols]."""
    brows, bcols = obstacle_mask.shape
    r0, r1 = max(0, row_min), min(brows - 1, row_max)
    c0, c1 = max(0, col_min), min(bcols - 1, col_max)
    return bool(obstacle_mask[r0:r1 + 1, c0:c1 + 1].any())
