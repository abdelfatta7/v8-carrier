"""nose_shroud_left: rounded cap over the left half of the nose top and its
outer side.  Split on the centreline (the nose is 280 wide); the only
material that crosses the split is the brow ahead of the LiDAR opening, which
ends in a centre chin joined by 2 x M3 bolts (along Y) and 2 x 3 mm pins.

Left half: bolt clearance holes + head pockets, pin holes.
Right half (nose_shroud_right): heat-set inserts, pin holes.
Built in the left frame (y >= 0); the right half mirrors the body.
"""
from __future__ import annotations

import math

from shell_common import (box, prism_xz, cyl_y, cyl_z, edge_on_line, extrude_outline,
                          fillet_top, inner_face_offset, load_params)


def geometry(p: dict) -> dict:
    ns = p["nose_shroud"]
    c = p["clearance"]
    t, R = ns["wall"], ns["edge_radius"]
    z_in = ns["rest_z"] + ns["skin_gap"]
    rc = ns["plan_corner_radius_front"]
    y_in = inner_face_offset(R, t, c, ns["rest_z"], z_in, ns["nose_outer_y"], rc)
    x_in = inner_face_offset(R, t, c, ns["rest_z"], z_in, ns["nose_front_x"], rc)
    return dict(t=t, R=R, z_in=z_in, z_out=z_in + t, y_side=y_in + t, x_front=x_in + t, x_rear=ns["rear_x"])


def outline(p, g, d, y0):
    ns = p["nose_shroud"]
    Rf = ns["plan_corner_radius_front"]
    xf, xr, Ys = g["x_front"], g["x_rear"], g["y_side"]
    Cf = (xf - Rf, Ys - Rf)
    pt = lambda a: (Cf[0] + (Rf - d) * math.cos(math.radians(a)), Cf[1] + (Rf - d) * math.sin(math.radians(a)))
    P0 = (xr + d, y0)
    P1 = (xf - d, y0)
    P2 = (xf - d, Cf[1])
    P3 = pt(90)
    P4 = (xr + d, Ys - d)
    return [("L", P0, P1), ("L", P1, P2), ("A", P2, pt(45), P3), ("L", P3, P4), ("L", P4, P0)]


def body_left(p: dict):
    ns = p["nose_shroud"]
    g = geometry(p)
    t, R = g["t"], g["R"]
    zb = ns["skirt_bottom_z"]
    c = p["clearance"]

    outer = extrude_outline(outline(p, g, 0.0, 0.0), zb, g["z_out"])
    outer = fillet_top(outer, R, lambda e: edge_on_line(e, "y", 0.0) or edge_on_line(e, "x", g["x_rear"]))
    inner = extrude_outline(outline(p, g, t, -5.0), zb - 1, g["z_in"])
    inner = fillet_top(inner, R - t, lambda e: edge_on_line(e, "y", -5.0) or edge_on_line(e, "x", g["x_rear"] + t))
    shell = outer - inner

    # solid brow ahead of the LiDAR opening (the bridge to the chin)
    lo = ns["lidar_opening"]
    # solid brow strip along the whole front: from brow_fill_from_x down to the nose top level, and from
    # the LiDAR opening's front edge above z rest + 3 (keeps 3 mm from the nose's front-top edge)
    shell += outer & box(ns["brow_fill_from_x"], g["x_front"] + 1, 0, 300, ns["rest_z"], g["z_out"])
    shell += outer & box(lo["x"][1], g["x_front"] + 1, 0, 300, ns["rest_z"] + c, g["z_out"])

    # chin block across the split
    ch = ns["split"]["chin"]
    chin = box(ch["x"][0], ch["x"][1], 0, ch["half_y"], ch["z"][0], ch["z"][1])
    from build123d import Axis
    vert = [e for e in chin.edges().filter_by(Axis.Z) if abs(e.center().X - ch["x"][1]) < 0.01 and e.center().Y > 0.01]
    chin = chin.fillet(ch["corner_radius"], vert)
    shell += chin

    # rest pads on the nose top; side ribs bearing on the nose side
    rp = ns["rest_pads"]
    for x, y in rp["xy"]:
        shell += cyl_z(rp["od"], ns["rest_z"], g["z_in"] + 0.5, x, y)
    sp = ns["side_pads"]
    for x in sp["x"]:
        shell += box(x - sp["length"] / 2, x + sp["length"] / 2,
                     ns["nose_outer_y"] - sp["interference"], g["y_side"] - t + 0.3, sp["z"][0], sp["z"][1])

    # rear: wall only as a short lip, 3 mm above the nose top
    shell -= box(g["x_rear"] - 1, g["x_rear"] + t + 0.01, -10, g["y_side"] - t - 0.01, zb - 1, ns["rear_lip_bottom_z"])
    # front face stays uncovered below z 125 (radar window, bumper)
    shell -= box(ns["nose_front_x"] + c + p.get("cut_margin", 0.0), ch["x"][1] + 10, -10, 300, zb - 1, ns["front_open_below_z"])
    # side skirt ends square at the front corner's tangent point (no thin oblique end)
    shell -= box(g["x_front"] - ns["plan_corner_radius_front"], ch["x"][1] + 10, -10, 300,
                 zb - 1, ns["front_open_below_z"])
    # brow lower front edge chamfered back to the LD2410 cone (outside the chin)
    rc = ns["radar_chamfer"]
    tn = math.tan(math.radians(rc["angle_deg"]))
    x0 = ns["nose_front_x"] + c + p.get("cut_margin", 0.0)
    x1 = ch["x"][1] + 20
    zc = lambda x: rc["top_z"] + (x - rc["face_x"]) * tn + rc["margin_z"]
    xf0 = x0 + rc.get("flat", 0.0)        # short flat ahead of the brow's rear face (no < 50 deg wedge there)
    shell -= prism_xz([(x0 - 0.01, zb - 1), (x0 - 0.01, zc(xf0)), (xf0, zc(xf0)), (x1, zc(x1)), (x1, zb - 1)], rc["from_y"], 300)
    # LiDAR plate / tower opening (+3 mm), run out through the rear edge
    shell -= box(g["x_rear"] - 10, lo["x"][1], -10, lo["half_y"], zb - 1, g["z_out"] + 10)
    return shell, g


def split_features(shell, p: dict, side: str):
    sp = p["nose_shroud"]["split"]
    z = sp["axis_z"]
    hy = sp["chin"]["half_y"]
    for x in sp["bolts_x"]:
        if side == "left":
            shell -= cyl_y(sp["bolt_hole"], -0.01, hy + 1, x, z)
            shell -= cyl_y(sp["head_pocket"]["d"], hy - sp["head_pocket"]["depth"], hy + 1, x, z)
        else:
            shell -= cyl_y(sp["insert_hole"]["d"], -sp["insert_hole"]["depth"], 0.01, x, z)
    for x in sp["pins_x"]:
        d = sp["pin_hole"]["depth"]
        shell -= cyl_y(sp["pin_hole"]["d"], -0.01, d, x, z) if side == "left" else cyl_y(sp["pin_hole"]["d"], -d, 0.01, x, z)
    return shell


def build(params: dict | None = None):
    p = params or load_params()
    shell, _ = body_left(p)
    return split_features(shell, p, "left")


NOTES = ("left half of the nose shroud; rests on the nose top (3 pads, VHB tape) and bears on the nose side (2 ribs); "
         "2 x M3 x 16 heads + 2 x 3 mm pins at the centre chin; LiDAR opening +3 mm; front face open below z 125")

if __name__ == "__main__":
    part = build()
    print(part.bounding_box(), part.volume)
