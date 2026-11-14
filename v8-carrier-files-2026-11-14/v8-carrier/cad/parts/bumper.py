"""Bumper: sprung front shell ahead of the nose.

Pieces: bumper_left, bumper_right (the 288 mm shell is wider than the bed, so
it is split on the centreline), bumper_joining_plate (bolts across the split
behind the centre ToF window).

Features (params.bumper):
  * 3 mm front plate from z[0] to z[1] with return walls wrapping the nose
    corners (1 mm outside the nose's side walls)
  * two 6 mm guide pins printed on the back face (y = +-guide_pin.y,
    z = nose.guide_pin_z) that slide in the nose's sleeves; a boss around each
    pin base seats a 10 mm compression spring that bears on the nose's front
    face with bumper.spring.preload at rest; travel is sensors.bumper_travel.
    A cross hole near each pin tip, just behind the sleeve's rear face at
    rest, takes an M2 screw or a split pin as the end stop.
  * two pads on the back face that press the microswitch levers mounted on the
    nose face at the corners
  * three windows for the VL53L1X sensors on the nose face
  * two cliff-sensor pockets under the bottom corners, open at the top, with a
    window in their floor, for the nose-corner cliff sensors (the wheel ones
    live in the pods, see side_pods.py)

Print each half face down (front plate on the bed); the pins, pads and pockets
grow upward from it.
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
    halves_y,
    insert_hole,
    layout,
    union,
)

NOTES = {
    "bumper_left": "guide pin and spring at y=+80; pad presses the left microswitch; corner cliff sensor pocket",
    "bumper_right": "mirror geometry of the left half (built as one shell and cut)",
    "bumper_joining_plate": "4 x M3 x 8 into the bosses on the back of the halves; window for the centre ToF sensor",
}


def build(p: dict) -> dict[str, Part]:
    L = layout(p)
    b = p["bumper"]
    n = p["nose"]
    fx = p["fasteners_extra"]
    f = p["fasteners"]
    sh = b["shell"]
    x1 = L["x_front"]
    x0 = x1 - sh  # back face of the front plate
    hw = L["half_w"] + 1 + sh  # outer edge of the wrap
    z0, z1 = b["z"]
    nose_front = L["nose_front"]

    solids = [box(x0, x1, -hw, hw, z0, z1)]
    # wraps around the nose corners
    for sy in (1, -1):
        solids.append(box(x1 - b["wrap"], x1, sy * (hw - sh), sy * hw, z0, z1))
    # guide pins and spring bosses
    gp = b["guide_pin"]
    pz = n["guide_pin_z"]
    pb = b["pin_boss"]
    for sy in (1, -1):
        solids.append(cyl_x(gp["dia"], x0 - gp["length"], x0 + EPS, sy * gp["y"], pz))
        solids.append(cyl_x(pb["od"], x0 - pb["height"], x0 + EPS, sy * gp["y"], pz))
    # switch pads
    sp = b["switch_pad"]
    for sy in (1, -1):
        solids.append(box(x0 - sp["depth"], x0 + EPS, sy * sp["y"] - sp["width"] / 2, sy * sp["y"] + sp["width"] / 2,
                          sp["z"] - sp["height"] / 2, sp["z"] + sp["height"] / 2))
    # cliff pockets under the bottom corners
    cp = b["cliff_pocket"]
    cb = p["sensors"]["cliff_body"]
    in_x = cb["width"] + 2 * cp["clearance"]
    in_y = cb["length"] + 2 * cp["clearance"]
    in_z = cb["height"] + cp["clearance"]
    pockets = []
    for sy in (1, -1):
        yc = sy * b["cliff_nose_y"]
        outer = box(x0 - in_x - cp["wall"], x0 + EPS, yc - in_y / 2 - cp["wall"], yc + in_y / 2 + cp["wall"], z0, z0 + cp["wall"] + in_z)
        pockets.append((outer, yc))
        solids.append(outer)
    # joining-plate bosses on the back of the front plate
    jp = b["joining_plate"]
    boss_pts = [(sy * jp["bolt_dx"], b["tof_z"] + sz * jp["bolt_dz"]) for sy in (1, -1) for sz in (1, -1)]
    for y, z in boss_pts:
        solids.append(cyl_x(f["boss_od"], x0 - jp["boss_height"], x0 + EPS, y, z))
    solid = union(solids)

    cuts = []
    # spring seats in the pin bosses, end-stop cross holes near the pin tips
    spr = b["spring"]
    seat_bumper = x0 - pb["height"] + pb["pocket_depth"]
    seat_nose = nose_front - spr["pocket_depth"]
    preload = spr["free_length"] - (seat_bumper - seat_nose)
    assert abs(preload - spr["preload"]) < 0.5, f"bumper spring preload {preload} mm, params say {spr['preload']}"
    sleeve_rear = nose_front - gp["sleeve_length"]
    stop_x = sleeve_rear - gp["stop_clearance"] - fx["m2_hole"] / 2
    assert stop_x - fx["m2_hole"] / 2 - 2 >= x0 - gp["length"], "guide pin too short for the end-stop cross hole"
    for sy in (1, -1):
        cuts.append(cyl_x(spr["pocket_dia"], x0 - pb["height"] - 1, seat_bumper, sy * gp["y"], pz)
                    - cyl_x(gp["dia"], x0 - pb["height"] - 2, x0 + 1, sy * gp["y"], pz))
        cuts.append(cyl_z(fx["m2_hole"], pz - gp["dia"], pz + gp["dia"], stop_x, sy * gp["y"]))
    # ToF windows
    tw = p["sensors"]["tof_window"]
    tof_ys = [b["tof_y"][0]] + [sy * y for y in b["tof_y"][1:] for sy in (1, -1)]
    for y in tof_ys:
        cuts.append(box(x0 - 1, x1 + 1, y - tw["width"] / 2, y + tw["width"] / 2, b["tof_z"] - tw["height"] / 2, b["tof_z"] + tw["height"] / 2))
    # cliff pocket interiors, floor windows and M2 holes
    cw = p["tray"]["cliff_window"]
    for outer, yc in pockets:
        cuts.append(box(x0 - in_x - cp["wall"] + cp["wall"], x0, yc - in_y / 2, yc + in_y / 2, z0 + cp["wall"], z0 + cp["wall"] + in_z + 1))
        xc = x0 - cp["wall"] - in_x / 2
        cuts.append(box(xc - cw["width"] / 2, xc + cw["width"] / 2, yc - cw["length"] / 2, yc + cw["length"] / 2, z0 - 1, z0 + cp["wall"] + 1))
        for dy in (1, -1):
            cuts.append(cyl_z(fx["m2_hole"], z0 - 1, z0 + cp["wall"] + 1, xc, yc + dy * p["sensors"]["cliff_hole_spacing"] / 2))
    # joining plate inserts
    for y, z in boss_pts:
        cuts.append(insert_hole(p, "x", x0 - jp["boss_height"], +1, y, z))
    solid = solid - cuts
    assert x0 - in_x - cp["wall"] - p["sensors"]["bumper_travel"] > nose_front, "cliff pockets would hit the nose at full travel"

    left, right = halves_y(solid)

    # joining plate behind the front plate, with its own ToF window
    px1 = x0 - jp["boss_height"]
    plate = box(px1 - jp["thickness"], px1, -jp["width"] / 2, jp["width"] / 2, b["tof_z"] - jp["height"] / 2, b["tof_z"] + jp["height"] / 2)
    pcuts = [clearance_hole(p, "x", px1 - jp["thickness"] - 1, px1 + 1, y, z) for y, z in boss_pts]
    pcuts.append(box(px1 - jp["thickness"] - 1, px1 + 1, -tw["width"] / 2 - 2, tw["width"] / 2 + 2, b["tof_z"] - tw["height"] / 2 - 2, b["tof_z"] + tw["height"] / 2 + 2))
    plate = plate - pcuts

    return {"bumper_left": left, "bumper_right": right, "bumper_joining_plate": plate}
