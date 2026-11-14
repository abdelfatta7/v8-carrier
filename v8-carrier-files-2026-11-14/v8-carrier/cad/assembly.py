"""Full assembly in the robot frame for the checker.

``assembled(p)`` returns every print piece (mirrored copies added for the
pieces that are printed twice) plus PROVISIONAL placeholder bodies for the
bought parts and the vacuum, so intersection and centre-of-mass tests can run.
Placeholder keys start with ``placeholder_``; they are never exported as print
pieces.  ``placeholder_masses(p)`` gives their masses in kg.

The vacuum placeholder is a 1.6 kg box of params.placeholders.vacuum
(PROVISIONAL 320 x 130 x 220 mm) sitting on the slot floor with its nose
forward, its front face on the nose's duct flange.
"""

from __future__ import annotations

import importlib

from build123d import Part

from cad.common import box, cyl_y, cyl_z, layout, mirror_y

PART_MODULES = ["floor_tray", "side_pods", "wheel_module", "nose", "bumper"]

# print pieces that are printed twice: the exported piece is the left instance
MIRRORED_TWICE = {"wheel_hub", "wheel_tyre"}


def build_pieces(p: dict) -> dict[str, tuple[str, Part]]:
    """name -> (module, Part) for every print piece."""
    out = {}
    for name in PART_MODULES:
        mod = importlib.import_module(f"cad.parts.{name}")
        for piece, part in mod.build(p).items():
            if piece in out:
                raise ValueError(f"duplicate piece name {piece}")
            out[piece] = (name, part)
    return out


def placeholders(p: dict) -> dict[str, Part]:
    """PROVISIONAL stand-ins for bought parts and the vacuum, robot frame."""
    L = layout(p)
    el = p["electronics"]
    pods = p["pods"]
    n = p["nose"]
    out = {}

    # vacuum: PROVISIONAL box, nose forward, front face at the duct flange, on the slot floor
    v = p["placeholders"]["vacuum"]
    out["placeholder_vacuum"] = box(L["nose_rear"] - v["length"], L["nose_rear"], -v["width"] / 2, v["width"] / 2,
                                    L["floor_z1"], L["floor_z1"] + v["height"])

    # drive motors: body from the arm's inner face inward
    mot = p["motors"]["drive"]
    wm = p["wheel_module"]
    y_face = L["pod_outer"] + wm["bracket_thickness"] + wm["arm_gap"]
    for side, s in (("left", 1), ("right", -1)):
        out[f"placeholder_drive_motor_{side}"] = cyl_y(mot["diameter"], s * (y_face - mot["body_length"]), s * y_face, 0, L["axle_z"])

    # battery in the left pod bay
    bl = pods["battery_layout"]["battery"]
    bx0, bx1 = sorted(bl["x"])
    by0 = L["pod_inner"] + bl["y_offset"]
    zf = L["pod_z0"] + p["body"]["floor"]
    bat = el["battery_4s"]
    out["placeholder_battery"] = box(bx0 + 3, bx1 - 3, by0, by0 + bat["width"], zf, zf + bat["height"])

    # Pi 5 with cooler on its standoffs in the right pod (mirrored layout)
    pl = pods["electronics_layout"]["pi5"]
    px0, px1 = sorted(pl["x"])
    py0 = L["pod_inner"] + pl["y_offset"]
    pz0 = zf + pods["standoff"]["height"]
    pi = el["pi5"]
    out["placeholder_pi5"] = mirror_y(box(px0, px1, py0, py0 + pi["width"], pz0, pz0 + pi["height_with_cooler"]))

    # ESP32-S3 on end in the right pod
    es = pods["electronics_layout"]["esp32"]
    esp = el["esp32_s3_devkitc"]
    yg = L["pod_inner"] + es["y_offset"]
    out["placeholder_esp32"] = mirror_y(box(es["x0"], es["x0"] + esp["width"], yg - 1, yg + 1, zf + 2, zf + 2 + esp["length"]))

    # LiDAR on the tower platform
    lid = p["lidar"]
    tcx = L["x_front"] - lid["position_from_front"]
    out["placeholder_lidar"] = cyl_z(lid["diameter"], n["tower"]["platform_z"], n["tower"]["platform_z"] + lid["height"], tcx, 0)

    # brush bar and brush motor
    from cad.parts.nose import geom as nose_geom

    g = nose_geom(p)
    br = p["brush_roll"]
    yc = n["brush_offset_y"]
    out["placeholder_brush_bar"] = cyl_y(br["diameter"], yc - br["length"] / 2, yc + br["length"] / 2, g["bar_x"], g["bar_z"])
    bm = p["motors"]["brush"]
    out["placeholder_brush_motor"] = cyl_y(bm["diameter"], g["cav_y0"], g["cav_y0"] + bm["body_length"], g["motor_x"], g["motor_z"])

    # caster under the rear of the tray
    c = p["caster"]
    out["placeholder_caster"] = cyl_z(c["base_diameter"], 0, c["height"], L["caster_x"], 0)
    return out


def placeholder_masses(p: dict) -> dict[str, float]:
    m = p["masses"]
    ph = p["placeholders"]
    return {
        "placeholder_vacuum": m["vacuum"],
        "placeholder_drive_motor_left": m["motor_drive_each"],
        "placeholder_drive_motor_right": m["motor_drive_each"],
        "placeholder_battery": m["battery_4s"],
        "placeholder_pi5": m["pi5_with_cooler"],
        "placeholder_esp32": ph["esp32_mass"],
        "placeholder_lidar": m["lidar"],
        "placeholder_brush_bar": m["brush_roll"],
        "placeholder_brush_motor": m["brush_motor"],
        "placeholder_caster": ph["caster_mass"],
    }


def assembled(p: dict) -> dict[str, Part]:
    """Every print piece in the robot frame (twice-printed pieces as _left/_right
    instances) plus the placeholders."""
    out = {}
    for piece, (_, part) in build_pieces(p).items():
        if piece in MIRRORED_TWICE:
            out[f"{piece}_left"] = part
            out[f"{piece}_right"] = mirror_y(part)
        else:
            out[piece] = part
    out.update(placeholders(p))
    return out
