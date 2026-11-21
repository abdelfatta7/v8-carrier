"""pod_cover_right: mirror of the left cover body over the right
(electronics) pod and lid, with its own cut-outs (stop button, sensor post,
ESP32 USB slot), vent slots over the Pi in the skin, and skirt slots facing
the pod's outer-wall vents."""
from __future__ import annotations

from pod_cover_left import add_fixings, body_left, cut_footprints
from shell_common import box, cyl_y, load_params, mirror_y, stadium_slot_z


def _slot_y(x, w, y0, y1, z0, z1):
    """Slot through a wall facing Y: width w along X, rounded at top/bottom."""
    r = w / 2
    s = box(x - r, x + r, y0, y1, z0 + r, z1 - r)
    s += cyl_y(w, y0, y1, x, z0 + r)
    s += cyl_y(w, y0, y1, x, z1 - r)
    return s


def build(params: dict | None = None):
    p = params or load_params()
    pc = p["pod_cover"]
    side = pc["right"]
    left, g = body_left(p)
    shell = mirror_y(left)
    shell = add_fixings(shell, p, g, side["screws"], side["rest_pads"])
    shell = cut_footprints(shell, p, g, side["cutouts"])

    tv = side["top_vents"]
    x = tv["x"][0] + tv["width"] / 2
    while x <= tv["x"][1] - tv["width"] / 2 + 1e-6:
        shell -= stadium_slot_z(x - tv["width"] / 2, x + tv["width"] / 2, tv["y"][0], tv["y"][1],
                                g["z_in"] - 3, g["z_out"] + 3)
        x += tv["pitch"]

    sv = side["side_vents"]
    x = sv["x"][0] + sv["width"] / 2
    while x <= sv["x"][1] - sv["width"] / 2 + 1e-6:
        shell -= _slot_y(x, sv["width"], -(pc["fender"]["outer_y"] + 5), -(pc["lid_outer_y"] + 1),
                         sv["z"][0], sv["z"][1])
        x += sv["pitch"]
    return shell


NOTES = ("right pod cover + wheel fender (mirror of left body); 4 x M3 x 10 into free lid-grid inserts + 2 pads; "
         "cut-outs: stop button, sensor post, ESP32 USB slot; vent slots over the Pi and in the skirt over the pod wall vents")

if __name__ == "__main__":
    part = build()
    print(part.bounding_box(), part.volume)
