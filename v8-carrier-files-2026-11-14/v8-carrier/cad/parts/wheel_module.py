"""Wheel module: swing-arm sprung drive mount, one per side.

Pieces (left side modelled, right side mirrored):
    wheel_bracket_left / _right   plate bolted to the pod's outer face (4 x M3 into pod inserts)
    wheel_arm_left / _right       trailing arm on a 625ZZ bearing, carries the JGB37-520
    wheel_hub                     PETG hub, D-bore with a radial grub-screw boss (print 2)
    wheel_tyre                    TPU tyre ring, keyed to the hub (print 2)

Y stack outward from the pod's outer face (params.wheel_module):
    pod wall | bracket plate | arm_gap (1 mm M5 washer) | arm plate | hub_gap | hub
The motor bolts to the arm's inner face on its 6 x M3 / 31 mm pattern and its
body reaches inward through the bracket window and the pod wall into the pod's
wheel box.  The pivot is an M5 bolt through the bearing, the bracket plate and
the pod wall (nylock inside the pod).  A compression spring (10 mm OD) sits on
the arm's post above the motor and pushes against a ledge on the bracket.  The
spring axis is spring_y_from_pod outside the pod wall, inboard of the arm's
mid-plane, so the spring clears both the pod wall and the hub; the bracket
plate is notched above the motor window where the spring and post pass it.
A peg under the ledge locates the spring's top; the post meets the peg at
~6 mm bump, and a stop block over the arm's tail limits droop to ~5 mm, so the
axle travels about 10 mm in all.

Print the bracket and arm flat (plate on the bed), the hub open side up, the
tyre flat.
"""

from __future__ import annotations

import math

from build123d import Part

from cad.common import (
    EPS,
    box,
    clearance_hole,
    cyl_x,
    cyl_y,
    cyl_z,
    insert_hole,
    layout,
    mirror_y,
    union,
)

NOTES = {
    "wheel_bracket_left": "4 x M3 x 10 into the pod's outer-wall inserts; M5 x 30 pivot bolt through bracket and pod wall",
    "wheel_bracket_right": "mirror of the left bracket",
    "wheel_arm_left": "press a 625ZZ into the pocket from the inner face; motor bolts from the outer face with 6 x M3 x 8",
    "wheel_arm_right": "mirror of the left arm",
    "wheel_hub": "print 2 (same part both sides); M3 grub screw in a heat-set insert on the D-flat",
    "wheel_tyre": "print 2 in TPU; stretch over the hub, the inner rib drops into the hub groove",
}


def _ys(p: dict) -> dict:
    """Y coordinates of the stack (left side)."""
    L = layout(p)
    wm = p["wheel_module"]
    b0 = L["pod_outer"]
    b1 = b0 + wm["bracket_thickness"]
    a0 = b1 + wm["arm_gap"]
    a1 = a0 + wm["arm_thickness"]
    h0 = a1 + wm["hub_gap"]
    h1 = h0 + p["wheels"]["width"]
    return dict(L=L, b0=b0, b1=b1, a0=a0, a1=a1, h0=h0, h1=h1, hub_centre=(h0 + h1) / 2,
                spring_y=b0 + wm["spring_y_from_pod"])


def motor_window(p: dict, y0: float, y1: float) -> Part:
    """Open-bottom stadium that clears the motor body plus suspension travel."""
    L = layout(p)
    hw = p["motors"]["drive"]["diameter"] / 2 + p["pods"]["wheel_box_clearance"]
    zc = L["axle_z"] + p["wheels"]["suspension_travel"] / 2
    return box(-hw, hw, y0, y1, L["axle_z"] - 40, zc) + cyl_y(2 * hw, y0, y1, 0, zc)


def _notch_half_width(p: dict) -> float:
    """Half width of the bracket notch: spring radius + 2 mm + the sideways shift of
    the post top as the arm swings through half the travel."""
    wm = p["wheel_module"]
    L = layout(p)
    post_top = _arm_post_base_z(p) + wm["spring_post"]["height"]
    shift = (post_top - L["axle_z"]) * (p["wheels"]["suspension_travel"] / 2) / wm["arm_length"]
    return p["wheels"]["spring_od"] / 2 + 2 + shift


def _arm_post_base_z(p: dict) -> float:
    """Top of the arm's spring pad: 4 mm pad on the flange disc, 1.5 mm above the motor body."""
    L = layout(p)
    return L["axle_z"] + p["motors"]["drive"]["diameter"] / 2 + 5.5


def _bracket(p: dict, Y: dict) -> Part:
    wm = p["wheel_module"]
    L = Y["L"]
    y0, y1 = Y["b0"], Y["b1"]
    xa, xb = wm["bracket_x"]
    za, zb = wm["bracket_z"]
    plate = box(xa, xb, y0, y1, za, zb)
    ys = Y["spring_y"]
    # spring ledge over the arm's post: spans from the plate to the arm's outer face (clear of the hub)
    post_top = _arm_post_base_z(p) + wm["spring_post"]["height"]
    ledge_z0 = _arm_post_base_z(p) + wm["seat_gap"]
    nw = _notch_half_width(p)
    ledge_len = max(wm["ledge"]["min_length"], 2 * nw + 6)
    ledge = box(-ledge_len / 2, ledge_len / 2, y0 + EPS, Y["a1"], ledge_z0, ledge_z0 + wm["ledge"]["thickness"])
    peg = wm["spring_peg"]
    peg_solid = cyl_z(peg["dia"], ledge_z0 - peg["height"], ledge_z0 + 2, 0, ys)
    # droop stop block over the arm's tail
    px = wm["arm_length"]
    tail = wm["droop_stop"]["tail_length"]
    beam_top = L["axle_z"] + wm["arm_beam_height"] / 2
    # the tail tip rises travel/2 * tail/arm at full droop; the stop sits 0.3 mm above that
    droop_rise = p["wheels"]["suspension_travel"] / 2 * tail / px
    stop = box(px + 7, px + tail + 2, y1 - EPS, Y["a1"], beam_top + droop_rise + 0.3, beam_top + droop_rise + 7)
    solid = union([plate, ledge, stop, peg_solid])
    cuts = [motor_window(p, y0 - 1, y1 + 1)]
    # notch above the window where the spring, post and pad pass the plate through the whole travel
    cuts.append(box(-nw, nw, y0 - 1, y1 + 1, L["axle_z"], ledge_z0))
    for bx, bz in wm["bracket_bolts"]:
        cuts.append(clearance_hole(p, "y", y0 - 1, y1 + 1, bx, bz))
    cuts.append(cyl_y(wm["pivot_bolt"]["clearance"], y0 - 1, y1 + 1, px, L["axle_z"]))
    assert post_top + (p["wheels"]["suspension_travel"] / 2) <= ledge_z0 - peg["height"] + 1, "spring post hits the peg before full bump"
    assert ys - p["wheels"]["spring_od"] / 2 >= Y["b0"] + 1, "spring too close to the pod wall"
    assert ys + p["wheels"]["spring_od"] / 2 <= Y["h0"] - 1, "spring too close to the hub"
    return solid - cuts


def _arm(p: dict, Y: dict) -> Part:
    wm = p["wheel_module"]
    mot = p["motors"]["drive"]
    brg = p["wheels"]["pivot_bearing"]
    L = Y["L"]
    y0, y1 = Y["a0"], Y["a1"]
    yc = (y0 + y1) / 2
    az = L["axle_z"]
    px = wm["arm_length"]
    bh = wm["arm_beam_height"]
    beam = box(0, px + wm["droop_stop"]["tail_length"], y0, y1, az - bh / 2, az + bh / 2)
    pivot_boss = cyl_y(brg["od"] + 8, y0, y1, px, az)
    flange = cyl_y(mot["diameter"] + 6, y0, y1, 0, az)
    pad_top = _arm_post_base_z(p)
    ys = Y["spring_y"]
    pad = box(-4, 4, min(y0, ys - wm["spring_post"]["dia"] / 2), y1, pad_top - 4, pad_top)
    post = cyl_z(wm["spring_post"]["dia"], pad_top - EPS, pad_top + wm["spring_post"]["height"], 0, ys)
    solid = union([beam, pivot_boss, flange, pad, post])
    cuts = [
        # bearing pocket from the inner face, lip hole through the rest
        cyl_y(brg["od"], y0 - 1, y0 + brg["width"], px, az),
        cyl_y(brg["od"] - 4, y0 - 1, y1 + 1, px, az),
        # gearbox boss and shaft
        cyl_y(wm["motor_boss_dia"], y0 - 1, y1 + 1, 0, az),
    ]
    pcd = mot["mount_holes"]["pattern_dia"] / 2
    for i in range(mot["mount_holes"]["count"]):
        a = 2 * math.pi * i / mot["mount_holes"]["count"]
        cuts.append(clearance_hole(p, "y", y0 - 1, y1 + 1, pcd * math.cos(a), az + pcd * math.sin(a)))
    return solid - cuts


def _hub(p: dict, Y: dict) -> Part:
    w = p["wheels"]
    hub = p["wheel_module"]["hub"]
    key = p["wheel_module"]["tyre_key"]
    mot = p["motors"]["drive"]
    L = Y["L"]
    az = L["axle_z"]
    y0, y1 = Y["h0"], Y["h1"]
    yc = Y["hub_centre"]
    od = w["diameter"] - 2 * w["tyre_thickness"]
    rim = cyl_y(od, y0, y1, 0, az)
    web = cyl_y(od, y0, y0 + hub["web"], 0, az)
    centre = cyl_y(hub["centre_od"], y0, y1, 0, az)
    grub_y = y0 + hub["bore_depth"] / 2
    grub_boss = cyl_x(hub["grub_boss_od"], 0, hub["centre_od"] / 2 + 4, grub_y, az)
    # hollow the rim and cut the tyre key groove first, then add the grub boss so it stays solid
    solid = union([rim, web, centre]) - [
        cyl_y(od - 2 * hub["rim_wall"], y0 + hub["web"], y1 + 1, 0, az) - cyl_y(hub["centre_od"], y0, y1 + 2, 0, az),
        cyl_y(od + 2, yc - key["width"] / 2, yc + key["width"] / 2, 0, az) - cyl_y(od - 2 * key["depth"], yc - key["width"], yc + key["width"], 0, az),
    ]
    solid = solid + grub_boss
    cuts = []
    # D bore: shaft diameter plus clearance, flat on the +X side
    d = mot["shaft_diameter"] + 0.2
    flat = d / 2 - hub["d_flat_depth"]
    bore = cyl_y(d, y0 - 1, y0 + hub["bore_depth"], 0, az) & box(-d, flat, y0 - 2, y1, az - d, az + d)
    cuts.append(bore)
    # grub screw: insert from the boss end, clearance hole on to the bore
    boss_end = hub["centre_od"] / 2 + 4
    cuts.append(insert_hole(p, "x", boss_end, -1, grub_y, az))
    cuts.append(cyl_x(p["fasteners"]["clearance_hole"], 0, boss_end, grub_y, az))
    # lightening holes in the web
    for i in range(4):
        a = math.pi / 4 + i * math.pi / 2
        r = (od / 2 - hub["rim_wall"] + hub["centre_od"] / 2) / 2
        cuts.append(cyl_y(10, y0 - 1, y0 + hub["web"] + 1, r * math.cos(a), az + r * math.sin(a)))
    return solid - cuts


def _tyre(p: dict, Y: dict) -> Part:
    w = p["wheels"]
    key = p["wheel_module"]["tyre_key"]
    az = Y["L"]["axle_z"]
    y0, y1 = Y["h0"], Y["h1"]
    yc = Y["hub_centre"]
    od = w["diameter"]
    id_ = od - 2 * w["tyre_thickness"]
    ring = cyl_y(od, y0, y1, 0, az) - cyl_y(id_, y0 - 1, y1 + 1, 0, az)
    rib = cyl_y(id_ + 1, yc - key["width"] / 2, yc + key["width"] / 2, 0, az) - cyl_y(id_ - 2 * key["depth"], y0 - 1, y1 + 1, 0, az)
    return ring + rib


def build(p: dict) -> dict[str, Part]:
    Y = _ys(p)
    bracket = _bracket(p, Y)
    arm = _arm(p, Y)
    return {
        "wheel_bracket_left": bracket,
        "wheel_bracket_right": mirror_y(bracket),
        "wheel_arm_left": arm,
        "wheel_arm_right": mirror_y(arm),
        "wheel_hub": _hub(p, Y),
        "wheel_tyre": _tyre(p, Y),
    }
