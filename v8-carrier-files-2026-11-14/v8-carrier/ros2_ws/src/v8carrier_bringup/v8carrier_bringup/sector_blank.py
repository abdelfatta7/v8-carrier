"""Blank the LiDAR sector the vacuum body blocks. Pure Python / numpy.

The RPLIDAR C1 sits on the nose tower; behind it the Dyson's cyclone and bin
stand higher than the scan plane and return a fixed wall of hits. Those beams
are set to NaN (invalid), which slam_toolbox and Nav2 ignore.

Angles are in the laser frame, degrees, counter-clockwise, 0 = straight ahead
(+x), 180 = straight back. The sector is given as [start, end] going
counter-clockwise from start to end, so the rear 100 degrees is start=130,
end=230, and a sector across 0 (e.g. start=350, end=10) also works.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def normalize_deg(a: float) -> float:
    """Map an angle in degrees to [0, 360)."""
    return a % 360.0


def in_sector(angle_deg: float, start_deg: float, end_deg: float) -> bool:
    """True when angle lies in the counter-clockwise sector from start to end."""
    a = normalize_deg(angle_deg)
    s = normalize_deg(start_deg)
    e = normalize_deg(end_deg)
    if s <= e:
        return s <= a <= e
    return a >= s or a <= e  # wraps through 0


def sector_mask(angle_min: float, angle_increment: float, count: int,
                start_deg: float, end_deg: float) -> np.ndarray:
    """Boolean mask (True = blank it) for each beam of a LaserScan."""
    angles = np.degrees(angle_min + angle_increment * np.arange(count))
    a = np.mod(angles, 360.0)
    s = normalize_deg(start_deg)
    e = normalize_deg(end_deg)
    if s <= e:
        return (a >= s) & (a <= e)
    return (a >= s) | (a <= e)


def blank_sector(ranges: Sequence[float], angle_min: float, angle_increment: float,
                 start_deg: float = 130.0, end_deg: float = 230.0,
                 fill: float = math.nan) -> np.ndarray:
    """Return a copy of ranges with the sector set to `fill` (NaN by default)."""
    out = np.array(ranges, dtype=np.float32, copy=True)
    mask = sector_mask(angle_min, angle_increment, len(out), start_deg, end_deg)
    out[mask] = fill
    return out


def rear_sector(width_deg: float) -> tuple:
    """(start, end) for a sector of `width_deg` centred on straight back."""
    return 180.0 - width_deg / 2.0, 180.0 + width_deg / 2.0
