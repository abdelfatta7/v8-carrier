"""Differential-drive odometry from cumulative encoder ticks.

Same maths as firmware/lib/odometry/ (midpoint integration), so the firmware's
wheel-speed estimate and the Pi's pose estimate agree. Pure Python.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def unwrap_i32_delta(now: int, before: int) -> int:
    """Difference of two int32 counters that may have wrapped."""
    d = (now - before) & 0xFFFFFFFF
    return d - 0x100000000 if d >= 0x80000000 else d


@dataclass
class OdomParams:
    ticks_per_rev: int = 1320
    wheel_diameter_m: float = 0.080
    track_width_m: float = 0.333

    @property
    def metres_per_tick(self) -> float:
        return math.pi * self.wheel_diameter_m / self.ticks_per_rev


class Odometry:
    def __init__(self, params: OdomParams) -> None:
        self.p = params
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0
        self._last = None  # (left_ticks, right_ticks)

    def reset(self, left_ticks: int, right_ticks: int) -> None:
        self.x = self.y = self.theta = 0.0
        self.v = self.w = 0.0
        self._last = (left_ticks, right_ticks)

    def update(self, left_ticks: int, right_ticks: int, dt: float) -> None:
        if self._last is None:
            self.reset(left_ticks, right_ticks)
            return
        d_left = unwrap_i32_delta(left_ticks, self._last[0])
        d_right = unwrap_i32_delta(right_ticks, self._last[1])
        self._last = (left_ticks, right_ticks)
        self.update_delta(d_left, d_right, dt)

    def update_delta(self, d_left: int, d_right: int, dt: float) -> None:
        dl = d_left * self.p.metres_per_tick
        dr = d_right * self.p.metres_per_tick
        d = 0.5 * (dl + dr)
        dtheta = (dr - dl) / self.p.track_width_m if self.p.track_width_m > 0 else 0.0
        mid = self.theta + 0.5 * dtheta
        self.x += d * math.cos(mid)
        self.y += d * math.sin(mid)
        self.theta = wrap_angle(self.theta + dtheta)
        if dt > 0:
            self.left_speed = dl / dt
            self.right_speed = dr / dt
            self.v = d / dt
            self.w = dtheta / dt


def twist_to_wheels(params: OdomParams, v: float, w: float):
    """Body twist (m/s, rad/s) -> (left, right) wheel rim speeds in m/s."""
    return v - 0.5 * w * params.track_width_m, v + 0.5 * w * params.track_width_m
