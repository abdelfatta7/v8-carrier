"""pod_cover_left: smooth cover over the left (battery) pod and lid, with a
fender over the top half of the left wheel.

Built in the left frame (y > 0).  pod_cover_right mirrors the same body and
applies its own cut-outs.  Rests on the lid top (z 105) on screw bosses and
plain pads; 4 x M3 x 10 from above into the lid's existing insert grid.
"""
from __future__ import annotations

import math

from shell_common import (box, cyl_y, cyl_z, edge_on_line, extrude_outline,
                          fillet_top, inner_face_offset, load_params,
                          round_rect_z, stadium_slot_z)


def geometry(p: dict) -> dict:
    """Derived dimensions of the pod-cover body (left frame)."""
    pc = p["pod_cover"]
    c = p["clearance"]
    t, R = pc["wall"], pc["edge_radius"]
    z_in = pc["rest_z"] + pc["skin_gap"]
    rc = pc["plan_corner_radius_rear"]
    y_side_in = inner_face_offset(R, t, c, pc["rest_z"], z_in, pc["lid_outer_y"], rc)
    x_rear_in = -inner_face_offset(R, t, c, pc["rest_z"], z_in, -pc["lid_rear_x"], rc)
    return dict(t=t, R=R, z_in=z_in, z_out=z_in + t,
                y_side=y_side_in + t, x_rear=x_rear_in - t,
                x_front=pc["front_x"], y_in=pc["inner_y"],
                y_fender=pc["fender"]["outer_y"])


def outline(p: dict, g: dict, d: float, x_end: float):
    """Closed plan outline, offset inward by d (arc centres fixed).  The
    front is a square end at x_end, placed well ahead of the real front so
    the edge rounds run out cleanly; the body is trimmed to front_x later."""
    pc = p["pod_cover"]
    fe = pc["fender"]
    xr, yin = g["x_rear"], g["y_in"]
    Ys, Yf = g["y_side"], g["y_fender"]
    Rr, Rt = pc["plan_corner_radius_rear"], fe["taper_radius"]
    xt0 = fe["full_width_rear_x"]
    th = math.acos(1 - (Yf - Ys) / (2 * Rt))
    C1 = (xt0, Yf - Rt)
    C2 = (xt0 - 2 * Rt * math.sin(th), Ys + Rt)
    Cr = (xr + Rr, Ys - Rr)
    assert C2[0] > Cr[0] + 1, "taper runs into the rear corner"

    def pt(c, r, a):
        return (c[0] + r * math.cos(math.radians(a)), c[1] + r * math.sin(math.radians(a)))

    thd = math.degrees(th)
    P0 = (xr + d, yin + d)
    P1 = (x_end, yin + d)
    P3 = (x_end, Yf - d)
    P4 = (xt0, Yf - d)
    P5 = pt(C1, Rt - d, 90 + thd)
    P6 = pt(C2, Rt + d, -90)
    P7 = (Cr[0], Ys - d)
    P8 = pt(Cr, Rr - d, 180)
    return [
        ("L", P0, P1),
        ("L", P1, P3),
        ("L", P3, P4),
        ("A", P4, pt(C1, Rt - d, 90 + thd / 2), P5),
        ("A", P5, pt(C2, Rt + d, -90 + thd / 2), P6),
        ("L", P6, P7),
        ("A", P7, pt(Cr, Rr - d, 135), P8),
        ("L", P8, P0),
    ]


def body_left(p: dict):
    """Hollow cover body in the left frame, without side-specific features."""
    pc = p["pod_cover"]
    g = geometry(p)
    t, R = g["t"], g["R"]
    zb = pc["skirt_bottom_z"]

    xe = g["x_front"] + 25.0
    outer = extrude_outline(outline(p, g, 0.0, xe), zb, g["z_out"])
    outer = fillet_top(outer, R, lambda e: edge_on_line(e, "y", g["y_in"]) or edge_on_line(e, "x", xe))
    inner = extrude_outline(outline(p, g, t, xe + 5), zb - 1, g["z_in"])
    inner = fillet_top(inner, R - t, lambda e: edge_on_line(e, "y", g["y_in"] + t) or edge_on_line(e, "x", xe + 5))
    shell = outer - inner
    # trim to the real front; the fender keeps a flat front wall outboard of the wheel bracket / arm
    shell -= box(g["x_front"], xe + 10, g["y_in"] - 10, 300, zb - 5, g["z_out"] + 5)
    shell += outer & box(g["x_front"] - t, g["x_front"], pc["fender"]["front_wall_min_y"], 300, zb - 1, g["z_out"] + 1)

    big = 400
    # open inner edge below the short lip
    shell -= box(g["x_rear"] - 5, g["x_front"] + 5, g["y_in"] - 5, g["y_in"] + t + 0.01, zb - 1, pc["inner_lip_bottom_z"])
    # open front over the lid / pod / wheel bracket; fender front wall stays outboard of front_wall_min_y
    shell -= box(g["x_front"] - t - 0.01, g["x_front"] + 5, g["y_in"] - 5, pc["fender"]["front_wall_min_y"], zb - 1, g["z_in"])
    # rear wall stops above the rear-face switch / socket
    # (ends at the rear corner's tangent point so the skirt end is square, not a thin oblique cut)
    shell -= box(g["x_rear"] - 5, g["x_rear"] + pc["plan_corner_radius_rear"], g["y_in"] - 5, big, zb - 1, pc["rear_skirt_bottom_z"])
    # wheel arch: nothing within arch_radius of the axle, nothing below the axle over the arch
    w = pc["wheel"]
    y0, y1 = w["region_y"]
    shell -= cyl_y(2 * w["arch_radius"], y0, y1, w["axle_x"], w["axle_z"])
    shell -= box(w["axle_x"] - w["arch_radius"], g["x_front"] + 5, y0, y1, zb - 1, w["axle_z"])
    # fender front wall: flat bottom just above the arch, so the arch does not leave a knife edge on it
    xw = g["x_front"] - t
    zf = w["axle_z"] + math.sqrt(max(w["arch_radius"] ** 2 - (xw - w["axle_x"]) ** 2, 0.0)) + 0.5
    xw -= 0.01
    shell -= box(xw, g["x_front"] + 5, y0, y1, zb - 1, zf)
    return shell, g


def wall_point(p: dict, g: dict, x: float):
    """Outer skirt point (left frame) at x, and the outward normal angle (deg)
    of the skirt there (straight side, S-curve or fender side)."""
    pc = p["pod_cover"]
    fe = pc["fender"]
    Ys, Yf, Rt, xt0 = g["y_side"], g["y_fender"], fe["taper_radius"], fe["full_width_rear_x"]
    th = math.acos(1 - (Yf - Ys) / (2 * Rt))
    c1x, c1y = xt0, Yf - Rt
    c2x, c2y = xt0 - 2 * Rt * math.sin(th), Ys + Rt
    xj = xt0 - Rt * math.sin(th)
    if x >= xt0:
        return Yf, 90.0
    if x >= xj:
        a = math.acos((x - c1x) / Rt)
        return c1y + Rt * math.sin(a), math.degrees(a)
    if x >= c2x:
        a = -math.acos((x - c2x) / Rt)
        return c2y + Rt * math.sin(a), math.degrees(a) + 180.0
    return Ys, 90.0


def add_fixings(shell, p: dict, g: dict, screws, pads):
    pc = p["pod_cover"]
    b = pc["boss"]
    z0, ztop = pc["rest_z"], g["z_in"] + 0.5
    for x, y in screws:
        shell += cyl_z(b["od"], z0, ztop, x, y)
    for x, y in pads:
        shell += cyl_z(pc["rest_pad_od"], z0, ztop, x, y)
    for x, y in screws:
        shell -= cyl_z(b["hole"], z0 - 1, g["z_out"] + 1, x, y)
        shell -= cyl_z(b["cbore_d"], g["z_out"] - b["cbore_depth"], g["z_out"] + 5, x, y)
    return shell


def cut_footprints(shell, p: dict, g: dict, cutouts: dict, notch_front_of=None):
    pc = p["pod_cover"]
    c = p["clearance"] + p.get("cut_margin", 0.0)
    for name, ((x0, y0), (x1, y1)) in cutouts.items():
        xa, xb = x0 - c, x1 + c
        if notch_front_of == name:
            xb = g["x_front"] + 5
        shell -= round_rect_z(xa, xb, y0 - c, y1 + c, pc["rest_z"] + 0.5, g["z_out"] + 5, 3.0)
        # a cut-out that reaches into the outer edge round takes the round with it down to the
        # straight side wall (square 2 mm ends instead of a thin curved strip)
        ys = g["y_side"] - g["R"]
        # (only along the straight pod side; over the fender the skin is flat there)
        straight = wall_point(p, g, xb)[0] <= g["y_side"] + 1e-6
        if straight and y1 > 0 and y1 + c > ys:
            shell -= box(xa, xb, y1 + c - 1, g["y_side"] + 1, g["z_out"] - g["R"], g["z_out"] + 5)
        if straight and y0 < 0 and -y0 + c > ys:
            shell -= box(xa, xb, -(g["y_side"] + 1), y0 - c + 1, g["z_out"] - g["R"], g["z_out"] + 5)
    return shell


def build(params: dict | None = None):
    p = params or load_params()
    pc = p["pod_cover"]
    side = pc["left"]
    shell, g = body_left(p)
    shell = add_fixings(shell, p, g, side["screws"], side["rest_pads"])
    shell = cut_footprints(shell, p, g, side["cutouts"],
                           "speaker_box" if side.get("speaker_front_notch") else None)
    return shell


NOTES = ("left pod cover + wheel fender; rests on lid_left top on 4 screw bosses + 2 pads; "
         "4 x M3 x 10 from above into free lid-grid inserts; cut-outs: mic post, speaker box (+3 mm)")

if __name__ == "__main__":
    part = build()
    print(part.bounding_box(), part.volume)
