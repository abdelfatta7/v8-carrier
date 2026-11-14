import math

import numpy as np

from v8carrier_bringup.sector_blank import blank_sector, in_sector, rear_sector, sector_mask


def test_in_sector_simple_and_wrapping():
    assert in_sector(180, 130, 230)
    assert in_sector(130, 130, 230)
    assert in_sector(230, 130, 230)
    assert not in_sector(0, 130, 230)
    assert not in_sector(129.9, 130, 230)
    assert in_sector(-180, 130, 230)  # normalised
    assert in_sector(5, 350, 10)       # sector across 0
    assert in_sector(355, 350, 10)
    assert not in_sector(180, 350, 10)


def test_rear_sector_default_is_100_degrees():
    assert rear_sector(100) == (130.0, 230.0)


def test_mask_counts_beams_in_rear_sector():
    # A full 360 scan with 1 degree beams starting at -pi (C1 style: angle_min = -pi).
    n = 360
    mask = sector_mask(-math.pi, math.radians(1.0), n, 130, 230)
    assert mask.sum() == 101  # 130..230 inclusive
    angles = np.degrees(-math.pi + math.radians(1.0) * np.arange(n)) % 360
    assert all(130 <= a <= 230 for a in angles[mask])


def test_blank_sector_sets_nan_and_keeps_rest():
    n = 720
    ranges = np.full(n, 2.0, dtype=np.float32)
    out = blank_sector(ranges, 0.0, math.radians(0.5), 130, 230)
    assert out.dtype == np.float32
    assert np.isnan(out).sum() == 201
    assert np.all(out[:260] == 2.0)   # 0 .. 129.5 deg
    assert np.all(out[461:] == 2.0)   # 230.5 .. 359.5 deg
    assert np.all(ranges == 2.0)      # input untouched


def test_blank_with_custom_fill_value():
    out = blank_sector([1.0, 1.0, 1.0, 1.0], 0.0, math.radians(90), 170, 190, fill=float("inf"))
    assert math.isinf(out[2])
    assert out[0] == 1.0
