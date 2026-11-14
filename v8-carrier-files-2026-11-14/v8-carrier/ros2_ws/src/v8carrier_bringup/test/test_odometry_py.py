import math

from v8carrier_bringup.odometry import OdomParams, Odometry, twist_to_wheels, unwrap_i32_delta, wrap_angle


def params():
    return OdomParams(ticks_per_rev=1000, wheel_diameter_m=0.08, track_width_m=0.24)


def test_metres_per_tick():
    assert params().metres_per_tick == math.pi * 0.08 / 1000


def test_straight_line():
    o = Odometry(params())
    o.update(0, 0, 0.01)
    o.update(1000, 1000, 1.0)
    assert o.x == math.pi * 0.08
    assert o.y == 0.0
    assert o.theta == 0.0
    assert o.v == math.pi * 0.08
    assert o.left_speed == o.right_speed == math.pi * 0.08


def test_spin_in_place():
    o = Odometry(params())
    o.update(0, 0, 0.01)
    o.update(-100, 100, 0.1)
    d = math.pi * 0.08 * 0.1
    assert abs(o.x) < 1e-12 and abs(o.y) < 1e-12
    assert abs(o.theta - 2 * d / 0.24) < 1e-12
    assert abs(o.w - 2 * d / 0.24 / 0.1) < 1e-9


def test_quarter_arc():
    o = Odometry(params())
    o.update(0, 0, 0.01)
    R, t = 0.5, 0.24
    mpt = params().metres_per_tick
    steps = 500
    l = r = 0.0
    for _ in range(steps):
        l += (R - t / 2) * (math.pi / 2) / steps / mpt
        r += (R + t / 2) * (math.pi / 2) / steps / mpt
        o.update(round(l), round(r), 0.01)
    assert abs(o.x - R) < 0.01
    assert abs(o.y - R) < 0.01
    assert abs(o.theta - math.pi / 2) < 0.01


def test_wraparound():
    assert unwrap_i32_delta(-2147483648, 2147483647) == 1
    assert unwrap_i32_delta(2147483647, -2147483648) == -1
    o = Odometry(params())
    o.update(2147483600, 2147483600, 0.01)
    wrapped = ((2147483600 + 100) + 2**31) % 2**32 - 2**31
    o.update(wrapped, wrapped, 0.1)
    assert abs(o.x - math.pi * 0.08 * 0.1) < 1e-9


def test_wrap_angle_and_ik():
    assert wrap_angle(3 * math.pi) == math.pi
    assert wrap_angle(-math.pi) == math.pi
    assert twist_to_wheels(params(), 0.3, 0.0) == (0.3, 0.3)
    l, r = twist_to_wheels(params(), 0.0, 1.0)
    assert abs(l + 0.12) < 1e-12 and abs(r - 0.12) < 1e-12
