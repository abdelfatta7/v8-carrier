"""pod_cover_right: mirror of the left cover body over the right
(electronics) pod and lid, with its own cut-outs (stop button, sensor post,
ESP32 USB slot), vent slots over the Pi in the skin, and skirt slots facing
the pod's outer-wall vents."""
from __future__ import annotations

from build123d import Pos, Rot

from pod_cover_left import add_fixings, body_left, cut_footprints, wall_point
from shell_common import box, cyl_y, load_params, mirror_y, stadium_slot_z


def _slot_normal(x, y, normal_deg, w, length, z0, z1):
    """Slot of width w through the wall at (x, y), cut along the wall's
    outward normal (deg, about Z), rounded at top and bottom."""
    r = w / 2
    s = box(-r, r, -length / 2, length / 2, z0 + r, z1 - r)
    s += cyl_y(w, -length / 2, length / 2, 0, z0 + r)
    s += cyl_y(w, -length / 2, length / 2, 0, z1 - r)
    return Pos(x, y, 0) * Rot(0, 0, normal_deg - 90.0) * s


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
    keep_off = [((x0 - cc, y0 - cc), (x1 + cc, y1 + cc)) for (x0, y0), (x1, y1) in side["cutouts"].values()
                for cc in [p["clearance"] + p.get("cut_margin", 0.0) + tv.get("web_min", 2.0)]]
    while x <= tv["x"][1] - tv["width"] / 2 + 1e-6:
        xa, xb = x - tv["width"] / 2, x + tv["width"] / 2
        if any(xa < q1[0] and xb > q0[0] and tv["y"][0] < q1[1] and tv["y"][1] > q0[1] for q0, q1 in keep_off):
            x += tv["pitch"]
            continue                                  # would leave a thin web next to a cut-out
        shell -= stadium_slot_z(x - tv["width"] / 2, x + tv["width"] / 2, tv["y"][0], tv["y"][1],
                                g["z_in"] - 3, g["z_out"] + 3)
        x += tv["pitch"]

    sv = side["side_vents"]
    x = sv["x"][0] + sv["width"] / 2
    while x <= sv["x"][1] - sv["width"] / 2 + 1e-6:
        yw, nd = wall_point(p, g, x)                     # left frame
        shell -= _slot_normal(x, -yw, -nd, sv["width"], 4 * g["t"] + 2, sv["z"][0], sv["z"][1])
        x += sv["pitch"]
    return shell


NOTES = ("right pod cover + wheel fender (mirror of left body); 4 x M3 x 10 into free lid-grid inserts + 2 pads; "
         "cut-outs: stop button, sensor post, ESP32 USB slot; vent slots over the Pi and in the skirt over the pod wall vents")

if __name__ == "__main__":
    part = build()
    print(part.bounding_box(), part.volume)
