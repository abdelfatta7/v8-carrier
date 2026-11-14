"""Side pods: the two boxes either side of the vacuum slot, with lids.

Pieces: pod_left (battery), pod_right (electronics), lid_left, lid_right.

Both pods are modelled in the LEFT frame (y > 0) and the electronics pod is
mirrored across the centreline at the end, so every layout number in
params.yaml is given for a left-handed pod.

Each pod is an open-top box (wall 3, floor 4) on the tray, bolted down with
vertical M3 bolts into the tray's inserts.  A "wheel box" (two full-height
partition walls around the drive motor) keeps the compartments closed while
the motor and its swing arm pass through the outer wall and the floor.  The
outer wall carries the wheel-module bracket inserts and the M5 pivot bolt
hole.  A pass-through in the floor opens into the tray's cable channel.

Battery pod (left): 4S pack on the rear-compartment floor with end stops, a
side rail and zip-tie slots; rocker switch cutout and charge socket hole in
the rear end wall; fuse holder, 5V5A buck and INA226 hang from the lid.

Electronics pod (right), all in the rear compartment: ESP32-S3 DevKitC-1
standing on end in vertical rails against the outer wall just behind the
wheel box, the third DRV8871 beside it on the inner side, then the Pi 5 on
its 58 x 49 hole pattern (walls recessed 1 mm so the 56 mm board clears), and
two DRV8871 behind the Pi; vent slots in the outer wall and lid.

Each pod also carries the wheel-cliff-sensor holes in its short front
compartment floor (sensor body along Y), looking down through the matching
window in the tray.

Print pods open side up, lids flat (standoffs up).
"""

from __future__ import annotations

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
    stadium_y,
    stadium_z,
    standoff,
    union,
)

NOTES = {
    "pod_left": "battery pod; bolts to the tray with M3 x 8 through the floor; M5 pivot nut inside the front compartment",
    "pod_right": "electronics pod (mirror of the left layout); Pi 5 on M2.5 self-tap standoffs",
    "lid_left": "carries the fuse holder, 5V5A buck and INA226 on insert standoffs underneath",
    "lid_right": "vent slots over the Pi; slot over the ESP32-S3 USB end",
}


def _geom(p: dict) -> dict:
    """Pod box coordinates in the left frame."""
    L = layout(p)
    w = p["body"]["wall"]
    fl = p["body"]["floor"]
    pods = p["pods"]
    mot = p["motors"]["drive"]
    wm = p["wheel_module"]
    half_window = mot["diameter"] / 2 + pods["wheel_box_clearance"]
    g = dict(
        L=L,
        w=w,
        fl=fl,
        xf=L["pod_front"],
        xr=L["pod_rear"],
        yi=L["pod_inner"],
        yo=L["pod_outer"],
        z0=L["pod_z0"],
        z1=L["pod_z1"],
        half_window=half_window,
        part_in=half_window,  # partition inner faces at +-half_window
        part_out=half_window + pods["wheel_box_wall"],
        # the motor body reaches inward from the arm's inner face
        well_y_inner=L["pod_outer"] + wm["bracket_thickness"] + wm["arm_gap"] - mot["body_length"] - pods["wheel_box_clearance"],
        window_top_centre_z=L["axle_z"] + p["wheels"]["suspension_travel"] / 2,
        col=pods["corner_column"],
    )
    g["xi0"], g["xi1"] = g["xr"] + w, g["xf"] - w  # interior X range
    g["yi0"], g["yi1"] = g["yi"] + w, g["yo"] - w  # interior Y range
    g["zf"] = g["z0"] + fl  # floor top
    return g


def _lid_bolt_xy(g: dict) -> list[tuple[float, float]]:
    c = g["col"] / 2
    pts = []
    for x in (g["xi0"] + c, g["xi1"] - c):
        for y in (g["yi0"] + c, g["yi1"] - c):
            pts.append((x, y))
    # mid columns beside each wheel-box partition: inner wall ahead of it (the front
    # compartment holds the cliff sensor against the outer wall), outer wall behind it
    pts.append((g["part_out"] + c, g["yi0"] + c))
    pts.append((-g["part_out"] - c, g["yi1"] - c))
    return pts


def _board_standoffs(p, x_range, y0, width, z0, inset=None, pitch_x=None, pitch_y=None, hole=None):
    """Bosses and holes for a rectangular board whose X span is x_range and
    whose inner edge is at y0 (width along +Y).  Holes are inset from the
    corners (params.pods.module_hole_inset) unless an explicit pitch is given."""
    inset = p["pods"]["module_hole_inset"] if inset is None else inset
    xa, xb = sorted(x_range)
    if pitch_x is None:
        xs = (xa + inset, xb - inset)
    else:
        cx = (xa + xb) / 2
        xs = (cx - pitch_x / 2, cx + pitch_x / 2)
    if pitch_y is None:
        ys = (y0 + inset, y0 + width - inset)
    else:
        cy = y0 + width / 2
        ys = (cy - pitch_y / 2, cy + pitch_y / 2)
    bosses, holes = [], []
    for x in xs:
        for y in ys:
            b, h = standoff(p, x, y, z0, hole=hole, od=p["pods"]["standoff"]["od"] if hole else None)
            bosses.append(b)
            holes.append(h)
    return bosses, holes


def _shell(p: dict, g: dict, pi_recess_x=None) -> tuple[list[Part], list[Part]]:
    """Common pod body in the left frame: returns (solids, cuts).  solids[0] is
    the hollowed shell (interior and Pi recess already removed); the other
    solids are interior features to fuse onto it; cuts are drilled last."""
    f = p["fasteners"]
    pods = p["pods"]
    wm = p["wheel_module"]
    t = p["tray"]
    xf, xr, yi, yo, z0, z1 = g["xf"], g["xr"], g["yi"], g["yo"], g["z0"], g["z1"]
    solids, cuts = [], []

    hollow = [box(g["xi0"], g["xi1"], g["yi0"], g["yi1"], g["zf"], z1 + 1)]
    if pi_recess_x is not None:
        xa, xb = sorted(pi_recess_x)
        r = pods["pi_recess"]
        zr = g["zf"] + p["electronics"]["pi5"]["height_with_cooler"] + pods["standoff"]["height"] + 8
        hollow.append(box(xa - 2, xb + 2, g["yi0"] - r, g["yi0"] + 1, g["zf"] - EPS, zr))
        hollow.append(box(xa - 2, xb + 2, g["yi1"] - 1, g["yi1"] + r, g["zf"] - EPS, zr))
    solids.append(box(xr, xf, yi, yo, z0, z1) - hollow)

    # lid columns with inserts from the top
    c = g["col"] / 2
    for x, y in _lid_bolt_xy(g):
        solids.append(box(x - c, x + c, y - c, y + c, g["zf"] - EPS, z1))
        cuts.append(insert_hole(p, "z", z1, -1, x, y))

    # wheel box partitions
    for s in (1, -1):
        solids.append(box(s * g["part_in"], s * g["part_out"], yi, yo, z0, z1))

    # motor well through the floor and the outer wall window (open at the bottom)
    hw = g["half_window"]
    cuts.append(box(-hw, hw, g["well_y_inner"], yo + 1, z0 - 1, g["zf"] + 1))
    zc = g["window_top_centre_z"]
    cuts.append(box(-hw, hw, yo - g["w"] - 1, yo + 1, z0 - 1, zc))
    cuts.append(cyl_y(2 * hw, yo - g["w"] - 1, yo + 1, 0, zc))

    # wheel-module bracket inserts (horizontal, from the outer face) with bosses inside the wall
    for bx, bz in wm["bracket_bolts"]:
        solids.append(cyl_y(f["boss_od"], yo - f["insert_depth"] - 2, yo - EPS, bx, bz))
        cuts.append(insert_hole(p, "y", yo, -1, bx, bz))
    # M5 pivot bolt through the outer wall
    cuts.append(cyl_y(wm["pivot_bolt"]["clearance"], yo - g["w"] - 1, yo + 1, wm["arm_length"], g["L"]["axle_z"]))

    # pass-through into the tray channel
    pt = t["pass_through"]
    pt_y = yi + 3 + pt["width"] / 2
    cuts.append(box(t["channel"]["x"] - pt["length"] / 2, t["channel"]["x"] + pt["length"] / 2,
                    pt_y - pt["width"] / 2, pt_y + pt["width"] / 2, z0 - 1, g["zf"] + 1))
    # hold-down bolts through the floor
    for bx in t["pod_bolt_x"]:
        for by in t["pod_bolt_y"]:
            cuts.append(clearance_hole(p, "z", z0 - 1, g["zf"] + 1, bx, by))

    # wheel cliff sensor (body along Y): window through the floor plus two M2 self-tap holes
    cw = t["cliff_window"]
    cx, cy = t["cliff_wheel_x"], t["cliff_wheel_y"]
    cuts.append(box(cx - cw["width"] / 2, cx + cw["width"] / 2, cy - cw["length"] / 2, cy + cw["length"] / 2, z0 - 1, g["zf"] + 1))
    for dy in (-1, 1):
        cuts.append(cyl_z(p["fasteners_extra"]["m2_hole"], z0 - 1, g["zf"] + 1, cx, cy + dy * p["sensors"]["cliff_hole_spacing"] / 2))
    return solids, cuts


def _battery_pod(p: dict, g: dict) -> Part:
    pods = p["pods"]
    lay = pods["battery_layout"]
    el = p["electronics"]
    solids, cuts = _shell(p, g)

    # battery bay: end stops and an outer side rail, 10 mm tall, zip-tie slots in the floor
    bx0, bx1 = sorted(lay["battery"]["x"])
    by0 = g["yi"] + lay["battery"]["y_offset"]
    by1 = by0 + el["battery_4s"]["width"]
    zf = g["zf"]
    stop_h = 10
    solids.append(box(bx0, bx0 + 3, by0, by1, zf - EPS, zf + stop_h))
    solids.append(box(bx1 - 3, bx1, by0, by1, zf - EPS, zf + stop_h))
    solids.append(box(bx0, bx1, by1, by1 + 3, zf - EPS, zf + stop_h))
    for x in (bx0 + 20, bx1 - 20):
        for y in (by0 - 1.5, by1 + 3 + 2):
            cuts.append(box(x - 2.5, x + 2.5, y - 1.25, y + 1.25, g["z0"] - 1, zf + 1))

    # rocker switch cutout and charge socket in the rear end wall
    zp = pods["panel_z"]
    rs = el["rocker_switch_cutout"]
    ry = g["yi"] + lay["rocker_switch_offset"]
    cuts.append(box(g["xr"] - 1, g["xi0"] + 1, ry - rs["width"] / 2, ry + rs["width"] / 2, zp - rs["height"] / 2, zp + rs["height"] / 2))
    cy = g["yi"] + lay["charge_socket_offset"]
    cuts.append(cyl_x(el["charge_socket_panel_hole"], g["xr"] - 1, g["xi0"] + 1, cy, zp))
    return union(solids) - cuts


def _electronics_pod(p: dict, g: dict) -> Part:
    pods = p["pods"]
    lay = pods["electronics_layout"]
    el = p["electronics"]
    pi = el["pi5"]
    zf = g["zf"]
    solids, cuts = _shell(p, g, pi_recess_x=lay["pi5"]["x"])

    # Pi 5 standoffs (M2.5 self-tap) on the 58 x 49 pattern
    so = pods["standoff"]
    b, h = _board_standoffs(p, lay["pi5"]["x"], g["yi"] + lay["pi5"]["y_offset"], pi["width"], zf,
                            pitch_x=pi["hole_pitch_x"], pitch_y=pi["hole_pitch_y"], hole=so["hole"])
    solids += b
    cuts += h
    # DRV8871 x 3 on M3 insert standoffs
    drv = el["drv8871"]
    for yoff in lay["drv8871_rear"]["y_offsets"]:
        b, h = _board_standoffs(p, lay["drv8871_rear"]["x"], g["yi"] + yoff, drv["width"], zf)
        solids += b
        cuts += h
    b, h = _board_standoffs(p, lay["drv8871_front"]["x"], g["yi"] + lay["drv8871_front"]["y_offset"], drv["width"], zf)
    solids += b
    cuts += h
    # ESP32-S3 rails: two grooved posts, board on end, groove centre at y_offset from the inner wall
    er = pods["esp_rail"]
    esp = el["esp32_s3_devkitc"]
    x0 = lay["esp32"]["x0"]
    yg = g["yi"] + lay["esp32"]["y_offset"]
    for xa, xb, gx0, gx1 in (
        (x0 - er["rib"], x0, x0 - er["depth"], x0 + EPS),
        (x0 + esp["width"], x0 + esp["width"] + er["rib"], x0 + esp["width"] - EPS, x0 + esp["width"] + er["depth"]),
    ):
        solids.append(box(xa, xb, yg - er["gap"] / 2 - 3, yg + er["gap"] / 2 + 3, zf - EPS, zf + er["height"]))
        cuts.append(box(gx0, gx1, yg - er["gap"] / 2, yg + er["gap"] / 2, zf + 1, zf + er["height"] + 1))
    # vent slots in the outer wall (vertical stadiums), rear compartment only
    vs = pods["vent_slot"]
    va, vb = sorted(lay["vent_x"])
    x = vb
    zc = (g["zf"] + g["z1"]) / 2 + 4
    while x >= va:
        cuts.append(stadium_y(vs["length"], vs["width"], g["yo"] - g["w"] - 1, g["yo"] + 1, x, zc, along="z"))
        x -= vs["pitch"]
    return union(solids) - cuts


def _lid(p: dict, g: dict, side: str) -> Part:
    pods = p["pods"]
    el = p["electronics"]
    z0 = g["z1"]
    z1 = z0 + pods["lid_thickness"]
    lid = box(g["xr"], g["xf"], g["yi"], g["yo"], z0, z1)
    cuts = [clearance_hole(p, "z", z0 - 1, z1 + 1, x, y) for x, y in _lid_bolt_xy(g)]
    solids = [lid]
    if side == "left":
        boards = pods["battery_layout"]["lid_boards"]
        sizes = {"fuse_holder": el["fuse_holder"], "buck_5v5a": el["buck_5v5a"], "ina226": el["ina226"]}
        for name, spec in boards.items():
            dims = sizes[name]
            xa, xb = sorted(spec["x"])
            y0 = g["yi"] + spec["y_offset"]
            inset = pods["module_hole_inset"]
            for x in (xa + inset, xb - inset):
                for y in (y0 + inset, y0 + dims["width"] - inset):
                    solids.append(cyl_z(p["fasteners"]["boss_od"], z0 - pods["standoff"]["height"], z0 + EPS, x, y))
                    cuts.append(insert_hole(p, "z", z0 - pods["standoff"]["height"], +1, x, y))
    else:
        lay = pods["electronics_layout"]
        vs = pods["vent_slot"]
        xa, xb = sorted(lay["pi5"]["x"])
        x = xb - 6
        yc = (g["yi"] + g["yo"]) / 2
        while x >= xa + 6:
            cuts.append(stadium_z(vs["length"], vs["width"], z0 - 1, z1 + 1, x, yc, along="y"))
            x -= vs["pitch"]
        # slot over the ESP32-S3 USB end
        sl = lay["lid_usb_slot"]
        esp = el["esp32_s3_devkitc"]
        ex = lay["esp32"]["x0"] + esp["width"] / 2
        ey = g["yi"] + lay["esp32"]["y_offset"]
        cuts.append(box(ex - sl[0] / 2, ex + sl[0] / 2, ey - sl[1] / 2, ey + sl[1] / 2, z0 - 1, z1 + 1))
    return union(solids) - cuts


def build(p: dict) -> dict[str, Part]:
    g = _geom(p)
    left = _battery_pod(p, g)
    right = mirror_y(_electronics_pod(p, g))
    lid_left = _lid(p, g, "left")
    lid_right = mirror_y(_lid(p, g, "right"))
    return {"pod_left": left, "pod_right": right, "lid_left": lid_left, "lid_right": lid_right}
