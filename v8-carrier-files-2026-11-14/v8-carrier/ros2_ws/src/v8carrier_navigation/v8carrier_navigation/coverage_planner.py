"""Boustrophedon coverage over an occupancy grid. Pure Python + numpy.

Input: a nav_msgs/OccupancyGrid-style array (shape (height, width), values
-1 unknown, 0 free, 100 occupied; row j is y, column i is x, origin at the
bottom-left cell's corner). Output: a list of waypoints (x, y, yaw) in the map
frame: the start and end of each row segment, ordered so the robot sweeps
back and forth. Nav2's FollowWaypoints drives between them.

Steps:
  1. obstacles (occupied + unknown) are inflated by robot_radius + margin so
     a row centre line keeps the whole robot in free space;
  2. rows are laid `row_spacing` apart (brush 240 mm minus overlap) across the
     free area, along the longer side of its bounding box (or a fixed axis);
  3. each row is cut into runs of free cells; runs shorter than min_segment
     are dropped;
  4. rows are ordered boustrophedon: each row is swept in the direction whose
     start is nearest the end of the previous one, starting at the row end
     nearest the robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

Segment = Tuple[float, float, float, float]  # x0, y0, x1, y1 (map frame, metres)
Waypoint = Tuple[float, float, float]        # x, y, yaw


@dataclass
class CoverageConfig:
    row_spacing: float = 0.22       # brush width 0.24 m minus 0.02 m overlap
    robot_radius: float = 0.291     # from cad/params.yaml via geometry.yaml
    margin: float = 0.03            # extra clearance on top of the radius
    min_segment: float = 0.30       # drop runs shorter than this (m)
    row_axis: str = "auto"          # "x": rows run along x; "y": along y; "auto": longer side
    unknown_is_obstacle: bool = True


# ---- Grid helpers --------------------------------------------------------------
def obstacle_mask(grid: np.ndarray, unknown_is_obstacle: bool = True, occupied_thresh: int = 50) -> np.ndarray:
    g = np.asarray(grid)
    occ = g >= occupied_thresh
    if unknown_is_obstacle:
        occ |= g < 0
    return occ


def disk_offsets(radius_cells: int) -> List[Tuple[int, int]]:
    r2 = radius_cells * radius_cells
    return [(dx, dy) for dx in range(-radius_cells, radius_cells + 1)
            for dy in range(-radius_cells, radius_cells + 1) if dx * dx + dy * dy <= r2]


def inflate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Binary dilation by a disk, done with shifted ORs (no scipy needed)."""
    if radius_cells <= 0:
        return mask.copy()
    h, w = mask.shape
    out = mask.copy()
    for dx, dy in disk_offsets(radius_cells):
        if dx == 0 and dy == 0:
            continue
        src_y = slice(max(0, -dy), min(h, h - dy))
        dst_y = slice(max(0, dy), min(h, h + dy))
        src_x = slice(max(0, -dx), min(w, w - dx))
        dst_x = slice(max(0, dx), min(w, w + dx))
        out[dst_y, dst_x] |= mask[src_y, src_x]
    return out


def free_mask(grid: np.ndarray, resolution: float, cfg: CoverageConfig) -> np.ndarray:
    """Cells whose centre the robot may occupy with its whole footprint free."""
    occ = obstacle_mask(grid, cfg.unknown_is_obstacle)
    radius_cells = int(math.ceil((cfg.robot_radius + cfg.margin) / resolution))
    return ~inflate(occ, radius_cells)


# ---- Rows -------------------------------------------------------------------------
def _runs(line: np.ndarray) -> List[Tuple[int, int]]:
    """[start, end] index pairs (inclusive) of True runs in a 1-D bool array."""
    if line.size == 0:
        return []
    padded = np.concatenate(([False], line, [False]))
    d = np.diff(padded.astype(np.int8))
    starts = np.flatnonzero(d == 1)
    ends = np.flatnonzero(d == -1) - 1
    return list(zip(starts.tolist(), ends.tolist()))


def choose_axis(free: np.ndarray, cfg: CoverageConfig) -> str:
    if cfg.row_axis in ("x", "y"):
        return cfg.row_axis
    ys, xs = np.nonzero(free)
    if xs.size == 0:
        return "x"
    return "x" if (xs.max() - xs.min()) >= (ys.max() - ys.min()) else "y"


def plan_rows(free: np.ndarray, resolution: float, origin: Tuple[float, float],
              cfg: CoverageConfig) -> List[List[Segment]]:
    """Segments grouped per row, rows in increasing position across the axis."""
    axis = choose_axis(free, cfg)
    ox, oy = origin
    ys, xs = np.nonzero(free)
    if xs.size == 0:
        return []
    step = max(1, int(round(cfg.row_spacing / resolution)))
    min_cells = max(1, int(round(cfg.min_segment / resolution)))
    rows: List[List[Segment]] = []

    def centre(i: int) -> float:  # cell index -> metres along that axis (added to the origin by the caller)
        return (i + 0.5) * resolution

    if axis == "x":
        first, last = ys.min(), ys.max()
        # Centre the row pattern in the free band so the two edges get equal coverage.
        n_rows = max(1, (last - first) // step + 1)
        j0 = first + ((last - first) - (n_rows - 1) * step) // 2
        for j in range(j0, last + 1, step):
            segs = [(ox + centre(a), oy + centre(j), ox + centre(b), oy + centre(j))
                    for a, b in _runs(free[j, :]) if b - a + 1 >= min_cells]
            if segs:
                rows.append(segs)
    else:
        first, last = xs.min(), xs.max()
        n_rows = max(1, (last - first) // step + 1)
        i0 = first + ((last - first) - (n_rows - 1) * step) // 2
        for i in range(i0, last + 1, step):
            segs = [(ox + centre(i), oy + centre(a), ox + centre(i), oy + centre(b))
                    for a, b in _runs(free[:, i]) if b - a + 1 >= min_cells]
            if segs:
                rows.append(segs)
    return rows


# ---- Ordering -------------------------------------------------------------------
def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _sweep(segs: List[Segment], forward: bool) -> List[Segment]:
    """Order a row's segments for one sweep direction; flip each segment to match."""
    def key(s: Segment) -> float:
        return min(s[0], s[2]) + min(s[1], s[3])  # works for both axes (the other coordinate is constant)
    ordered = sorted(segs, key=key, reverse=not forward)
    out = []
    for x0, y0, x1, y1 in ordered:
        lo, hi = ((x0, y0), (x1, y1)) if (x0 + y0) <= (x1 + y1) else ((x1, y1), (x0, y0))
        out.append((lo[0], lo[1], hi[0], hi[1]) if forward else (hi[0], hi[1], lo[0], lo[1]))
    return out


def boustrophedon_order(rows: List[List[Segment]], start: Optional[Tuple[float, float]] = None) -> List[Segment]:
    """Flatten rows into one sweep sequence, choosing the direction of each row
    (and whether to begin at the first or the last row) by nearest distance."""
    if not rows:
        return []
    candidates = []
    for row_order in (rows, rows[::-1]):
        for first_forward in (True, False):
            seq: List[Segment] = []
            total = 0.0
            pos = start
            forward = first_forward
            for k, segs in enumerate(row_order):
                if k > 0 and pos is not None:
                    # Pick the direction whose first waypoint is nearest the current position.
                    f = _sweep(segs, True)
                    b = _sweep(segs, False)
                    forward = _dist(pos, (f[0][0], f[0][1])) <= _dist(pos, (b[0][0], b[0][1]))
                sweep = _sweep(segs, forward)
                for s in sweep:
                    if pos is not None:
                        total += _dist(pos, (s[0], s[1]))
                    total += _dist((s[0], s[1]), (s[2], s[3]))
                    pos = (s[2], s[3])
                seq.extend(sweep)
            candidates.append((total, seq))
            if start is None:
                break  # without a start pose, the direction choice is arbitrary: keep it simple
        if start is None:
            break
    return min(candidates, key=lambda c: c[0])[1]


def waypoints_from_segments(segments: Sequence[Segment]) -> List[Waypoint]:
    out: List[Waypoint] = []
    for x0, y0, x1, y1 in segments:
        yaw = math.atan2(y1 - y0, x1 - x0)
        out.append((x0, y0, yaw))
        out.append((x1, y1, yaw))
    return out


def plan_coverage(grid: np.ndarray, resolution: float, origin: Tuple[float, float],
                  cfg: Optional[CoverageConfig] = None,
                  start: Optional[Tuple[float, float]] = None) -> List[Waypoint]:
    """Full pipeline: grid -> ordered waypoints."""
    cfg = cfg or CoverageConfig()
    free = free_mask(np.asarray(grid), resolution, cfg)
    rows = plan_rows(free, resolution, origin, cfg)
    return waypoints_from_segments(boustrophedon_order(rows, start))


def path_length(waypoints: Sequence[Waypoint]) -> float:
    return sum(_dist(waypoints[i][:2], waypoints[i + 1][:2]) for i in range(len(waypoints) - 1))


def coverage_fraction(grid: np.ndarray, resolution: float, origin: Tuple[float, float],
                      waypoints: Sequence[Waypoint], brush_width: float,
                      cfg: Optional[CoverageConfig] = None) -> float:
    """Fraction of the (inflated-)free cells within brush_width/2 of a sweep
    segment. A planning self-check, not needed at run time."""
    cfg = cfg or CoverageConfig()
    free = free_mask(np.asarray(grid), resolution, cfg)
    if not free.any():
        return 0.0
    ys, xs = np.nonzero(free)
    px = origin[0] + (xs + 0.5) * resolution
    py = origin[1] + (ys + 0.5) * resolution
    covered = np.zeros(xs.shape, dtype=bool)
    half = brush_width / 2.0
    for k in range(0, len(waypoints) - 1, 2):
        x0, y0, _ = waypoints[k]
        x1, y1, _ = waypoints[k + 1]
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        if L2 == 0:
            continue
        t = np.clip(((px - x0) * dx + (py - y0) * dy) / L2, 0.0, 1.0)
        d = np.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        covered |= d <= half + 1e-9
    return float(covered.mean())
