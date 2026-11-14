import math

import numpy as np
import pytest

from v8carrier_navigation import coverage_planner as C

RES = 0.05


def room(width_m=6.0, height_m=4.0, obstacle=True):
    """Occupancy grid of a walled room (unknown outside the walls), optional
    0.6 x 1.0 m obstacle (a table leg cluster) in the middle."""
    w, h = int(width_m / RES), int(height_m / RES)
    grid = np.full((h + 10, w + 10), -1, dtype=np.int8)  # unknown border
    grid[5:5 + h, 5:5 + w] = 0
    grid[5, 5:5 + w] = 100
    grid[4 + h, 5:5 + w] = 100
    grid[5:5 + h, 5] = 100
    grid[5:5 + h, 4 + w] = 100
    if obstacle:
        cy, cx = 5 + h // 2, 5 + w // 2
        grid[cy - 10:cy + 10, cx - 6:cx + 6] = 100
    return grid, (-5 * RES, -5 * RES)


def cfg():
    return C.CoverageConfig(row_spacing=0.22, robot_radius=0.20, margin=0.02, min_segment=0.3)


def test_inflate_grows_a_point_into_a_disk():
    m = np.zeros((21, 21), dtype=bool)
    m[10, 10] = True
    out = C.inflate(m, 3)
    assert out[10, 13] and out[13, 10] and out[12, 12]
    assert not out[10, 14] and not out[13, 13]
    assert out.sum() == len(C.disk_offsets(3))


def test_free_mask_keeps_robot_clear_of_walls():
    grid, _ = room(obstacle=False)
    free = C.free_mask(grid, RES, cfg())
    # 0.22 m clearance = 5 cells: the first free row is 5 cells inside the wall.
    assert not free[6, 60] and not free[10, 60]
    assert free[11, 60]
    assert not free[0, 0]  # unknown is an obstacle


def test_runs():
    line = np.array([0, 1, 1, 0, 1, 1, 1, 0, 1], dtype=bool)
    assert C._runs(line) == [(1, 2), (4, 6), (8, 8)]
    assert C._runs(np.zeros(3, dtype=bool)) == []


def test_rows_are_spaced_by_row_spacing_and_inside_free_space():
    grid, origin = room(obstacle=False)
    free = C.free_mask(grid, RES, cfg())
    rows = C.plan_rows(free, RES, origin, cfg())
    assert len(rows) >= 14  # ~3.5 m usable / 0.22
    ys = [segs[0][1] for segs in rows]
    for a, b in zip(ys, ys[1:]):
        assert b - a == pytest.approx(0.20, abs=RES)  # 0.22 rounds to 4 cells = 0.20
    for segs in rows:
        assert len(segs) == 1
        x0, y0, x1, y1 = segs[0]
        assert y0 == y1
        assert x1 - x0 > 5.0  # 6 m room minus clearance on both sides


def test_obstacle_splits_rows():
    grid, origin = room(obstacle=True)
    free = C.free_mask(grid, RES, cfg())
    rows = C.plan_rows(free, RES, origin, cfg())
    split = [segs for segs in rows if len(segs) == 2]
    assert len(split) >= 4  # the obstacle is 1 m tall: 1.0 + 2 * 0.22 clearance / 0.2 spacing
    for segs in split:
        (x0, _, x1, _), (x2, _, x3, _) = sorted(segs)
        assert x1 < 3.0 - 0.3 < 3.0 + 0.3 < x2  # gap around the obstacle centre at x = 3.0


def test_boustrophedon_alternates_direction():
    grid, origin = room(obstacle=False)
    wps = C.plan_coverage(grid, RES, origin, cfg(), start=(0.3, 0.3))
    assert len(wps) % 2 == 0 and len(wps) >= 28
    # Starts near the robot (bottom-left corner).
    assert wps[0][0] < 1.0 and wps[0][1] < 1.0
    dirs = [1 if wps[k + 1][0] > wps[k][0] else -1 for k in range(0, len(wps), 2)]
    assert all(a == -b for a, b in zip(dirs, dirs[1:]))
    # Yaw points along each row.
    for k in range(0, len(wps), 2):
        assert wps[k][2] == pytest.approx(0.0 if dirs[k // 2] == 1 else math.pi, abs=1e-6)
        assert wps[k][2] == wps[k + 1][2]
    # Rows ascend in y.
    ys = [wps[k][1] for k in range(0, len(wps), 2)]
    assert ys == sorted(ys)


def test_start_at_far_corner_reverses_row_order():
    grid, origin = room(obstacle=False)
    wps = C.plan_coverage(grid, RES, origin, cfg(), start=(5.7, 3.7))
    assert wps[0][0] > 5.0 and wps[0][1] > 3.0
    ys = [wps[k][1] for k in range(0, len(wps), 2)]
    assert ys == sorted(ys, reverse=True)


def test_every_waypoint_and_segment_is_in_free_space():
    grid, origin = room(obstacle=True)
    free = C.free_mask(grid, RES, cfg())
    wps = C.plan_coverage(grid, RES, origin, cfg(), start=(0.3, 0.3))

    def is_free(x, y):
        i = int((x - origin[0]) / RES)
        j = int((y - origin[1]) / RES)
        return bool(free[j, i])

    for k in range(0, len(wps), 2):
        x0, y0, _ = wps[k]
        x1, y1, _ = wps[k + 1]
        for t in np.linspace(0, 1, 50):
            assert is_free(x0 + t * (x1 - x0), y0 + t * (y1 - y0))


def test_coverage_fraction_is_high():
    grid, origin = room(obstacle=True)
    wps = C.plan_coverage(grid, RES, origin, cfg(), start=(0.3, 0.3))
    frac = C.coverage_fraction(grid, RES, origin, wps, brush_width=0.24, cfg=cfg())
    assert frac > 0.9
    assert C.path_length(wps) > 60.0


def test_row_axis_y_runs_rows_vertically():
    grid, origin = room(obstacle=False)
    c = cfg()
    c.row_axis = "y"
    wps = C.plan_coverage(grid, RES, origin, c, start=(0.3, 0.3))
    assert wps[0][0] == wps[1][0]
    assert abs(wps[0][2]) == pytest.approx(math.pi / 2)


def test_auto_axis_follows_the_longer_side():
    grid, origin = room(width_m=3.0, height_m=5.0, obstacle=False)
    wps = C.plan_coverage(grid, RES, origin, cfg())
    assert wps[0][0] == wps[1][0]  # rows along y, the longer side


def test_empty_or_tiny_map_gives_no_waypoints():
    assert C.plan_coverage(np.full((10, 10), -1, dtype=np.int8), RES, (0, 0), cfg()) == []
    small = np.full((12, 12), 100, dtype=np.int8)
    small[5:7, 5:7] = 0
    assert C.plan_coverage(small, RES, (0, 0), cfg()) == []
