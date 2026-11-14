"""Floor tray: the floor of the body between the nose and the rear.

Four print pieces (the 315 x 280 mm floor is split on the centreline, as
decided, and again across the body at ``tray.cross_split_x`` so that every
piece fits the bed):

    tray_front_left, tray_front_right, tray_rear_left, tray_rear_right

Features (all positions from params.yaml):
  * flat slot floor between the pods with a raised rim at the slot edges
  * split flanges hanging under the floor along both split planes, carrying
    M3 insert / clearance holes and 4 mm dowel sockets
  * caster mount pad at the rear (the split flange is cut away under it so the
    caster base sits flat across both halves)
  * cable channel tube under the slot floor joining the two pod footprints,
    with a pass-through opening inside each pod
  * M3 inserts (bosses hanging below the floor) for the pod hold-down bolts
  * upstanding nose rim at the front with horizontal clearance holes for the
    nose bolts (the nose carries the inserts; heads are reached from the slot)
  * motor wells (open to the outer edge) under the drive motors
  * windows under the wheel cliff sensors (the sensors sit in the pods)

Printed with the slot floor on the bed (upside down); everything else grows
upward from it.
"""

from __future__ import annotations

from build123d import Part

from cad.common import (
    EPS,
    box,
    clearance_hole,
    cyl_x,
    cyl_z,
    insert_hole,
    layout,
    split_centreline,
    split_plane_x,
    union,
)

NOTES = {
    "tray_front_left": "centreline flange carries the inserts; cross-split flange carries the inserts for the rear piece; fit the two nose bolts (counterbored in the nose rim) before the pod goes on",
    "tray_front_right": "centreline flange carries the clearance holes (bolt heads face -Y, reach them from below); fit the two nose bolts before the pod goes on",
    "tray_rear_left": "carries one caster screw hole; bolts to tray_front_left across the cross split",
    "tray_rear_right": "carries one caster screw hole; bolts to tray_front_right across the cross split",
}


def build(p: dict) -> dict[str, Part]:
    L = layout(p)
    t = p["tray"]
    f = p["fasteners"]
    x0, x1 = L["x_rear"], L["nose_rear"]
    hw = L["half_w"]
    z0, z1 = L["floor_z0"], L["floor_z1"]
    fw, fh = t["flange"]["width"], t["flange"]["height"]
    xs = t["cross_split_x"]

    # ---- solids -----------------------------------------------------------
    floor = box(x0, x1, -hw, hw, z0, z1)
    rim = t["rim"]
    rims = [
        box(x0, x1, L["slot_half"], L["slot_half"] + rim["width"], z1 - EPS, z1 + rim["height"]),
        box(x0, x1, -L["slot_half"] - rim["width"], -L["slot_half"], z1 - EPS, z1 + rim["height"]),
    ]
    nr = t["nose_rim"]
    nose_rims = [
        box(x1 - nr["thickness"], x1, L["pod_inner"], hw, z1 - EPS, z1 + nr["height"]),
        box(x1 - nr["thickness"], x1, -hw, -L["pod_inner"], z1 - EPS, z1 + nr["height"]),
    ]
    flange_c = box(x0, x1, -fw, fw, z0 - fh, z0 + EPS)
    flange_x = box(xs - fw, xs + fw, -hw, hw, z0 - fh, z0 + EPS)
    ch = t["channel"]
    ye = t["channel_y_extent"]
    tube = box(
        ch["x"] - ch["clear_width"] / 2 - ch["wall"],
        ch["x"] + ch["clear_width"] / 2 + ch["wall"],
        -ye,
        ye,
        z0 - ch["clear_height"] - ch["wall"],
        z0 + EPS,
    )
    bosses = []
    for bx in t["pod_bolt_x"]:
        for by in t["pod_bolt_y"]:
            for sy in (1, -1):
                bosses.append(cyl_z(f["boss_od"], z0 - t["boss_below"], z0 + EPS, bx, sy * by))

    solid = union([floor, *rims, *nose_rims, flange_c, flange_x, tube, *bosses])

    # ---- cuts -------------------------------------------------------------
    cuts = []
    # caster: flange cut away under the base, two screw holes through the floor
    cb = p["caster"]["base_diameter"] / 2 + 2
    cuts.append(box(L["caster_x"] - cb, L["caster_x"] + cb, -fw - 1, fw + 1, z0 - fh - 2, z0 + EPS))
    for sy in (1, -1):
        cuts.append(cyl_z(p["caster"]["mount_hole_dia"], z0 - 3, z1 + 1, L["caster_x"], sy * p["caster"]["mount_hole_spacing"] / 2))
    # channel interior and the pass-throughs into the pods
    cuts.append(box(ch["x"] - ch["clear_width"] / 2, ch["x"] + ch["clear_width"] / 2, -ye + ch["wall"], ye - ch["wall"],
                    z0 - ch["clear_height"], z0 + EPS))
    pt = t["pass_through"]
    pt_y = L["pod_inner"] + 3 + pt["width"] / 2
    for sy in (1, -1):
        cuts.append(box(ch["x"] - pt["length"] / 2, ch["x"] + pt["length"] / 2, sy * pt_y - pt["width"] / 2,
                        sy * pt_y + pt["width"] / 2, z0 - 1, z1 + 1))
    # pod hold-down inserts from the top of the floor
    for bx in t["pod_bolt_x"]:
        for by in t["pod_bolt_y"]:
            for sy in (1, -1):
                cuts.append(insert_hole(p, "z", z1, -1, bx, sy * by))
    # nose bolts: horizontal clearance holes through the nose rim (inserts are in the nose); the heads are
    # counterbored into the rim's rear face so the pod's front wall can sit against it
    cb_ = nr["counterbore"]
    for by in t["nose_bolt_y"]:
        for sy in (1, -1):
            cuts.append(clearance_hole(p, "x", x1 - nr["thickness"] - 1, x1 + 1, sy * by, t["nose_bolt_z"]))
            cuts.append(cyl_x(cb_["dia"], x1 - nr["thickness"] - 1, x1 - nr["thickness"] + cb_["depth"], sy * by, t["nose_bolt_z"]))
    # motor wells under the drive motors, open to the outer edge
    mw = t["motor_well"]
    for sy in (1, -1):
        cuts.append(box(-mw["x_half"], mw["x_half"], sy * mw["y_inner"], sy * (hw + 2), z0 - fh - 2, z1 + nr["height"] + 2))
    # cliff-sensor windows under the pods
    cw = t["cliff_window"]  # sensor body along Y
    for sy in (1, -1):
        cuts.append(box(t["cliff_wheel_x"] - cw["width"] / 2, t["cliff_wheel_x"] + cw["width"] / 2,
                        sy * t["cliff_wheel_y"] - cw["length"] / 2, sy * t["cliff_wheel_y"] + cw["length"] / 2, z0 - 2, z1 + 2))
        for dy in (-1, 1):  # the sensor's M2 screws pass the pod floor and bite here too
            cuts.append(cyl_z(p["fasteners_extra"]["m2_hole"], z0 - 1, z1 + 1,
                              t["cliff_wheel_x"], sy * t["cliff_wheel_y"] + dy * p["sensors"]["cliff_hole_spacing"] / 2))
    solid = solid - cuts

    # ---- splits -----------------------------------------------------------
    zf = z0 - fh / 2  # bolt axis height in the flanges
    bolts_x, pins_x = t["centre_bolt_x"], t["centre_pin_x"]
    left, right = split_centreline(solid, p, [(x, zf) for x in bolts_x], [(x, zf) for x in pins_x])

    # cross split: bolts along X through the X-flange, positions mirrored per side
    cross_bolts_y = t["cross_bolt_y"]
    cross_pins_y = t["cross_pin_y"]
    out = {}
    for name, half, sy in (("left", left, 1), ("right", right, -1)):
        bolts = [(sy * y, zf) for y in cross_bolts_y]
        pins = [(sy * y, zf) for y in cross_pins_y]
        front, rear = split_plane_x(half, p, xs, bolts, pins)
        out[f"tray_front_{name}"] = front
        out[f"tray_rear_{name}"] = rear
    return out
