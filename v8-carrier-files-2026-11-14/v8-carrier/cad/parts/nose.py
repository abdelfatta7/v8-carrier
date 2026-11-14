"""Nose: the front of the body, ahead of the tray.

Pieces: nose_left, nose_right (centreline split), brush_cap (removable bearing
cap on the left end of the brush housing), lidar_plate (platform that bolts
across the top of the tower and ties the halves together).

Contents, all from params.yaml:
  * closed shell (wall 3) from the floor gap up to nose.height, bottom open
    only at the suction mouth, the belt bay and the access slot behind the
    housing
  * brush housing: tube around the bought 240 x 50 bar along Y, axis at
    nose.brush_axis_x, bottom tangent to the floor-gap plane, tied to the
    shell floor by a rib along each side of the suction mouth (the mouth is a
    radial sector of the tube so no wall tapers to a knife edge); the bar is
    shifted brush_offset_y toward the cap end so a pulley fits beside the
    drive-end wall.  625ZZ seats: in the drive-end wall (pressed from the belt
    bay) and in the removable cap (left end, bar slides out through a hole in
    the left shell wall)
  * GT2 belt bay on the right between the drive-end wall and the shell wall,
    open at the bottom; the JGA25-370 brush motor bolts to the inner face of
    the drive-end wall with its shaft through into the bay, axis set by the
    closed-belt centre distance
  * plenum box above the housing whose interior cut opens the bar cavity into
    it; round duct outlet in the rear face with four M3 inserts (the collar
    that mates with the vacuum is blocked on measurements)
  * LiDAR tower on the top plate, with corner columns for the lidar_plate
  * inserts in the rear wall for the four bolts that come forward through the
    tray's nose rim
  * guide-pin sleeves, spring pockets, ToF and microswitch holes for the
    bumper on the front face
  * split ribs on the top plate (inside and ahead of the tower) and on the
    floor behind the housing: horizontal bolts and dowels across the split

Print each half on its outer (Y = +-140) face, split face up.
"""

from __future__ import annotations

import math

from build123d import Part, Plane, Polygon, extrude

from cad.common import (
    EPS,
    belt_centre_distance,
    box,
    clearance_hole,
    cyl_x,
    cyl_y,
    cyl_z,
    insert_hole,
    layout,
    split_centreline,
    union,
)

NOTES = {
    "nose_left": "carries the split inserts and the brush cap bosses; lift the lidar_plate to reach the two tower split bolts",
    "nose_right": "carries the split clearance holes, the brush drive-end wall, belt bay and brush motor mount",
    "brush_cap": "press a 625ZZ into the inner face; 3 x M3 x 10 into the housing end bosses; remove to slide the bar out",
    "lidar_plate": "bolts to the four tower columns (4 x M3 x 8) across the split; RPLIDAR C1 screws from below",
}


def geom(p: dict) -> dict:
    L = layout(p)
    n = p["nose"]
    br = p["brush_roll"]
    bar_r = br["diameter"] / 2
    cav_r = bar_r + n["brush_clearance"]
    tube_r = cav_r + br["housing_wall"]
    bar_z = br["floor_gap"] + tube_r
    y_c = n["brush_offset_y"]
    cav_y0 = y_c - br["length"] / 2 - n["brush_clearance"]
    cav_y1 = y_c + br["length"] / 2 + n["brush_clearance"]
    C = belt_centre_distance(p)
    dx = n["brush_motor_offset_x"]
    motor_z = bar_z + math.sqrt(C * C - dx * dx)
    tower_cx = L["x_front"] - p["lidar"]["position_from_front"]
    return dict(
        L=L,
        x0=L["nose_rear"],
        x1=L["nose_front"],
        hw=L["half_w"],
        z0=br["floor_gap"],
        z1=n["height"],
        bar_x=n["brush_axis_x"],
        bar_z=bar_z,
        cav_r=cav_r,
        tube_r=tube_r,
        cav_y0=cav_y0,
        cav_y1=cav_y1,
        wall_y0=cav_y0 - n["end_wall"],  # drive-end wall outer face
        motor_x=n["brush_axis_x"] + dx,
        motor_z=motor_z,
        tower_cx=tower_cx,
    )


def _mouth_floor_half(p: dict, g: dict) -> float:
    """Half width of the vertical-walled floor cut under the mouth: the sector's half
    width at the top of the floor plate plus 2.5 mm."""
    n = p["nose"]
    z_top = g["z0"] + n["shell"] + 0.5
    return (g["bar_z"] - z_top) * math.tan(math.radians(n["mouth_half_angle"])) + 2.5


def _cap_profile(g: dict, cap: dict, cap_pts, y0, y1, grow=0.0) -> Part:
    """Disc the size of the housing tube plus an ear around each bolt boss.
    With ``grow`` (the opening cut) the profile is offset by that much and
    extended straight down to the floor plane so the shell floor under the
    cap is removed cleanly (no knife-edge wedges where the arc met it)."""
    r = g["tube_r"] + grow
    disc = cyl_y(2 * r, y0, y1, g["bar_x"], g["bar_z"])
    ears = [cyl_y(cap["ear_dia"] + 2 * grow, y0, y1, cx, cz) for cx, cz in cap_pts]
    parts = [disc, *ears]
    if grow > 0:
        parts.append(box(g["bar_x"] - r, g["bar_x"] + r, y0, y1, g["z0"] - 1, g["bar_z"]))
    return union(parts)


def build(p: dict) -> dict[str, Part]:
    g = geom(p)
    n = p["nose"]
    f = p["fasteners"]
    fx = p["fasteners_extra"]
    sh = n["shell"]
    x0, x1, hw, z0, z1 = g["x0"], g["x1"], g["hw"], g["z0"], g["z1"]
    bx, bz = g["bar_x"], g["bar_z"]
    bm = p["bumper"]

    # ---- shell ------------------------------------------------------------
    shell = box(x0, x1, -hw, hw, z0, z1) - box(x0 + sh, x1 - sh, -hw + sh, hw - sh, z0 + sh, z1 - sh)
    solids = [shell]

    # brush housing tube and drive-end wall (the left end stays open for the cap)
    solids.append(cyl_y(2 * g["tube_r"], g["wall_y0"], g["cav_y1"], bx, bz))
    solids.append(box(x0 + sh - 0.5, x1 - sh + 0.5, g["wall_y0"], g["cav_y0"], z0 + sh - 0.5, g["motor_z"] + 25))
    # ribs along Y either side of the mouth tying the housing tube to the shell floor
    tr = n["tube_rib"]
    fb = _mouth_floor_half(p, g)
    for sx in (1, -1):
        xa = bx + sx * fb
        xb = bx + sx * (fb + tr["width"])
        solids.append(box(xa, xb, g["cav_y0"], g["cav_y1"], z0 + sh - 0.5, z0 + tr["height"]))
    # bosses on the housing end for the cap bolts
    cap = n["cap"]
    cap_pts = []
    for i in range(cap["bolt_count"]):
        a = math.radians(cap["first_bolt_angle"]) + 2 * math.pi * i / cap["bolt_count"]
        cap_pts.append((bx + cap["bolt_circle"] / 2 * math.cos(a), bz + cap["bolt_circle"] / 2 * math.sin(a)))
    for cx, cz in cap_pts:
        solids.append(cyl_y(f["boss_od"], g["cav_y1"] - 18, g["cav_y1"], cx, cz))

    # plenum box
    pl = n["plenum"]
    solids.append(box(pl["x"][0], pl["x"][1], -pl["y_half"], pl["y_half"], pl["z"][0], pl["z"][1]))
    # duct flange inserts and tray-bolt inserts: bosses inside the rear wall
    do = n["duct_outlet"]
    flange_pts = [(sy * do["bolt_pitch"] / 2, do["z"] + sz * do["bolt_pitch"] / 2) for sy in (1, -1) for sz in (1, -1)]
    t = p["tray"]
    tray_pts = [(sy * y, t["nose_bolt_z"]) for y in t["nose_bolt_y"] for sy in (1, -1)]
    rear_bosses = [cyl_x(f["boss_od"], x0 + EPS, x0 + f["insert_depth"] + 3, y, z) for y, z in flange_pts + tray_pts]

    # LiDAR tower (hollow square) with corner columns for the lidar plate
    tw = n["tower"]
    ts = tw["size"] / 2
    tcx = g["tower_cx"]
    tz1 = tw["platform_z"] - 3  # columns stop under the 3 mm plate
    solids.append(box(tcx - ts, tcx + ts, -ts, ts, z1 - EPS, tz1) - box(tcx - ts + tw["wall"], tcx + ts - tw["wall"], -ts + tw["wall"], ts - tw["wall"], z1 - 1, tz1 + 1))
    col = p["pods"]["corner_column"]
    col_pts = [(tcx + sx * (ts - tw["wall"] - col / 2), sy * (ts - tw["wall"] - col / 2)) for sx in (1, -1) for sy in (1, -1)]
    for cx, cy in col_pts:
        solids.append(box(cx - col / 2, cx + col / 2, cy - col / 2, cy + col / 2, z1 - EPS, tz1))

    # split ribs: on the top plate (through the tower) and on the floor behind the housing
    rib = n["split_rib"]
    solids.append(box(x0 + sh - EPS, x1 - sh + EPS, -rib["width"], rib["width"], z1 - EPS, z1 + rib["height"]))
    solids.append(box(x0 + sh - EPS, bx - g["tube_r"] - 4, -rib["width"], rib["width"], z0 + sh - EPS, z0 + sh + rib["height"]))

    # bumper guide-pin sleeves inside the front wall
    gp = bm["guide_pin"]
    for sy in (1, -1):
        solids.append(cyl_x(gp["sleeve_od"], x1 - gp["sleeve_length"], x1 - EPS, sy * gp["y"], n["guide_pin_z"]))

    solid = union(solids)
    assert g["wall_y0"] - (-hw + sh) >= n["pulley_width"], "belt bay narrower than nose.pulley_width"
    assert math.hypot(g["motor_x"] - bx, g["motor_z"] - bz) >= g["tube_r"] + p["motors"]["brush"]["diameter"] / 2 + 2, "brush motor touches the housing"

    # ---- cuts -------------------------------------------------------------
    # plenum interior first (then the rear-wall bosses go in untrimmed), then the bar cavity, which opens into it
    w = pl["wall"]
    solid = solid - box(pl["x"][0] + w, pl["x"][1] - w, -pl["y_half"] + w, pl["y_half"] - w, pl["z"][0] + w, pl["z"][1] - w)
    # fill block behind the cavity opening so the plenum floor does not taper to a knife edge against the cavity
    zf_top = pl["z"][0] + w
    x_cross = bx - math.sqrt(g["cav_r"] ** 2 - (zf_top - bz) ** 2)
    fill = box(bx - g["cav_r"], x_cross + 0.5, -pl["y_half"] + w, pl["y_half"] - w, pl["z"][0] - 4, zf_top + 3.5)
    solid = solid + [fill, *rear_bosses]
    cuts = []
    cuts.append(cyl_y(2 * g["cav_r"], g["cav_y0"], g["cav_y1"] + 1, bx, bz))
    # suction mouth: a sector with radial side faces through the tube wall, plus a wider box with
    # vertical faces through the floor plate (so neither cut meets a surface at a glancing angle)
    th = math.radians(n["mouth_half_angle"])
    depth = bz + 5
    sector = extrude(Plane.XZ * Polygon((bx, bz), (bx + depth * math.tan(th), bz - depth), (bx - depth * math.tan(th), bz - depth), align=None),
                     amount=200, both=True) & box(bx - 100, bx + 100, g["cav_y0"], g["cav_y1"], -10, bz + 1)
    cuts.append(sector)
    fb = _mouth_floor_half(p, g)
    cuts.append(box(bx - fb, bx + fb, g["cav_y0"], g["cav_y1"], z0 - 1, z0 + sh + 0.5))
    # drive end: stub hole and bearing seat pressed from the belt bay
    brg = p["brush_roll"]["end_bearing"]
    cuts.append(cyl_y(brg["id"] + 1, g["wall_y0"] - 1, g["cav_y0"] + 1, bx, bz))
    cuts.append(cyl_y(brg["od"], g["wall_y0"] - 1, g["wall_y0"] + brg["width"], bx, bz))
    # brush motor: boss hole and two M3 clearance holes through the drive-end wall
    bmm = n["brush_motor_mount"]
    cuts.append(cyl_y(bmm["boss_dia"], g["wall_y0"] - 1, g["cav_y0"] + 1, g["motor_x"], g["motor_z"]))
    for dz in (1, -1):
        cuts.append(clearance_hole(p, "y", g["wall_y0"] - 1, g["cav_y0"] + 1, g["motor_x"], g["motor_z"] + dz * bmm["hole_spacing"] / 2))
    # belt bay floor opening (access from below)
    cuts.append(box(x0 + sh + 2, x1 - sh - 2, -hw + sh, g["wall_y0"] - 1, z0 - 1, z0 + sh + 1))
    # access slot in the floor behind the housing (reach the lower split pin, cables)
    cuts.append(box(x0 + sh + 2, bx - g["tube_r"] - 6, -hw + sh + 2, hw - sh - 2, z0 - 1, z0 + sh + 1)
                - box(x0 - 1, x1 + 1, -rib["width"] - 2, rib["width"] + 2, z0 - 2, z0 + sh + 2))
    # cap end: hole in the left shell wall shaped like the cap plus 1 mm, cap bolt inserts in the bosses
    cuts.append(_cap_profile(g, cap, cap_pts, g["cav_y1"], hw + 1, grow=cap["opening_clearance"]))  # also clears the floor plate under the cap
    for cx, cz in cap_pts:
        cuts.append(insert_hole(p, "y", g["cav_y1"], -1, cx, cz))
    # duct outlet and its inserts; tray-bolt inserts
    cuts.append(cyl_x(do["diameter"], x0 - 1, pl["x"][0] + w + 1, 0, do["z"]))
    for y, z in flange_pts + tray_pts:
        cuts.append(insert_hole(p, "x", x0, +1, y, z))
    # tower: opening through the top plate for cables (ahead of the plenum, which stays sealed), column inserts
    cuts.append(box(max(tcx - ts + tw["wall"] + col + 2, pl["x"][1] + 3), tcx + ts - tw["wall"] - col - 2,
                    -ts + tw["wall"] + col + 2, ts - tw["wall"] - col - 2, z1 - sh - 1, z1 + 1)
                - box(x0 - 1, x1 + 1, -rib["width"] - 2, rib["width"] + 2, z1 - sh - 2, z1 + 2))
    for cx, cy in col_pts:
        cuts.append(insert_hole(p, "z", tz1, -1, cx, cy))
    # bumper interface on the front face
    sp = bm["spring"]
    for sy in (1, -1):
        cuts.append(cyl_x(gp["clearance"], x1 - gp["sleeve_length"] - 1, x1 + 1, sy * gp["y"], n["guide_pin_z"]))
        cuts.append(cyl_x(sp["pocket_dia"], x1 - sp["pocket_depth"], x1 + 1, sy * gp["y"], n["guide_pin_z"]))
    tof_ys = [bm["tof_y"][0]] + [sy * y for y in bm["tof_y"][1:] for sy in (1, -1)]
    for y in tof_ys:
        for dz in (1, -1):
            cuts.append(cyl_x(fx["m2_hole"], x1 - sh - 1, x1 + 1, y, bm["tof_z"] + dz * bm["tof_hole_spacing"] / 2))
    sw = bm["switch_body"]
    for sy in (1, -1):
        for dy in (1, -1):
            cuts.append(cyl_x(fx["m2_hole"], x1 - sh - 1, x1 + 1, sy * bm["switch_pad"]["y"] + dy * sw["hole_spacing"] / 2, bm["switch_pad"]["z"]))
    solid = solid - cuts

    # ---- split ------------------------------------------------------------
    left, right = split_centreline(solid, p, n["split_bolts"], n["split_pins"], flange_w=rib["width"])

    # ---- brush cap --------------------------------------------------------
    cap_part = _cap_profile(g, cap, cap_pts, g["cav_y1"], g["cav_y1"] + cap["thickness"])
    cap_cuts = [cyl_y(brg["od"], g["cav_y1"] - 1, g["cav_y1"] + brg["width"], bx, bz)]
    for cx, cz in cap_pts:
        cap_cuts.append(clearance_hole(p, "y", g["cav_y1"] - 1, g["cav_y1"] + cap["thickness"] + 1, cx, cz))
    cap_part = cap_part - cap_cuts

    # ---- lidar plate ------------------------------------------------------
    plate = box(tcx - ts - 3, tcx + ts + 3, -ts - 3, ts + 3, tz1, tw["platform_z"])
    pcuts = [clearance_hole(p, "z", tz1 - 1, tw["platform_z"] + 1, cx, cy) for cx, cy in col_pts]
    pitch = p["lidar"]["mount_hole_pitch"] / 2
    for sx in (1, -1):
        for sy in (1, -1):
            pcuts.append(cyl_z(n["lidar_hole_dia"], tz1 - 1, tw["platform_z"] + 1, tcx + sx * pitch, sy * pitch))
    cs = tw["cable_slot"]
    pcuts.append(box(tcx - ts + 6, tcx - ts + 6 + cs[0], -cs[1] / 2, cs[1] / 2, tz1 - 1, tw["platform_z"] + 1))
    plate = plate - pcuts

    return {"nose_left": left, "nose_right": right, "brush_cap": cap_part, "lidar_plate": plate}
