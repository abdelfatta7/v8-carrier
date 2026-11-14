"""Check 4: the cable channel between the pods is clear.

The channel path comes from ``params.tray.channel`` / ``channel_y_extent`` /
``pass_through``: a tube under the slot floor running along Y at
``channel.x`` with a ``clear_width`` x ``clear_height`` interior, and a
pass-through at each end rising through the tray floor and the pod floor
into the pod.  Three probe boxes with a 15 x 10 mm cross-section are swept
along that path (one along Y through the tube, one up each pass-through to
the pod floor top) and intersected with every assembled body; any common
volume over ``VOLUME_LIMIT`` is an intrusion.

Two sanity tests make sure the probe is where the channel is: a probe grown
by the tube wall must hit the tray halves (the walls exist around the
probe), and the probe's end faces must open into the pod interiors.  The
check also lists any centreline fastener whose axis (from params) crosses
the tube interior: such a bolt either blocks the channel or, as here, cuts
no material at all.
"""

from __future__ import annotations

import sys
import time

from cad.common import box

from checks.common import FAIL, Context, CheckResult, bbox_of, bbox_overlap

NAME = "cable_clearance"
VOLUME_LIMIT = 1.0  # mm3


def probes(ctx: Context) -> dict[str, object]:
    p, L = ctx.p, ctx.L
    t = p["tray"]
    ch, pt = t["channel"], t["pass_through"]
    cw, chh = ch["clear_width"], ch["clear_height"]
    ye = t["channel_y_extent"] - ch["wall"]  # interior Y extent
    z0 = L["floor_z0"]
    zt, zb = z0, z0 - chh  # tube interior top (tray underside) and bottom
    pt_y = L["pod_inner"] + 3 + pt["width"] / 2  # same formula as floor_tray.py / side_pods.py
    pod_floor_top = L["pod_z0"] + p["body"]["floor"]
    out = {
        "tube along Y": box(ch["x"] - cw / 2, ch["x"] + cw / 2, -ye, ye, zb, zt),
    }
    for side, s in (("left", 1), ("right", -1)):
        out[f"pass-through up into pod_{side}"] = box(ch["x"] - cw / 2, ch["x"] + cw / 2,
                                                      s * pt_y - chh / 2, s * pt_y + chh / 2, zb, pod_floor_top)
    geom = dict(x=ch["x"], clear_width=cw, clear_height=chh, y_extent=ye, z=[zb, zt], pass_through_y=pt_y,
                pass_through_size=[pt["length"], pt["width"]], pod_floor_top=pod_floor_top)
    return out, geom


def run(ctx: Context) -> CheckResult:
    t0 = time.time()
    res = CheckResult(NAME)
    probe_set, geom = probes(ctx)
    bodies = ctx.assembled
    boxes = {n: bbox_of(b) for n, b in bodies.items()}
    rows = []
    for pname, probe in probe_set.items():
        pb = bbox_of(probe)
        for name, body in bodies.items():
            if not bbox_overlap(pb, boxes[name], 0.5):
                continue
            try:
                vol = float((probe & body).volume)
            except Exception:
                vol = None
            rows.append({"probe": pname, "body": name, "volume_mm3": None if vol is None else round(vol, 3)})
            if vol is None:
                res.fail(f"{pname} & {name}: boolean failed")
            elif vol > VOLUME_LIMIT:
                res.fail(f"{pname}: {name} intrudes by {vol:.1f} mm3 (limit {VOLUME_LIMIT})")
            elif vol > 0.01:
                res.warn(f"{pname}: {name} overlaps the probe by {vol:.3f} mm3 (sliver)")

    # sanity 1: the tube walls exist around the probe (grown probe hits the tray)
    p, L = ctx.p, ctx.L
    ch = p["tray"]["channel"]
    w = ch["wall"]
    grown = box(geom["x"] - geom["clear_width"] / 2 - w, geom["x"] + geom["clear_width"] / 2 + w,
                -geom["y_extent"], geom["y_extent"], geom["z"][0] - w, geom["z"][1])
    wall_hits = {n: round(float((grown & bodies[n]).volume), 1) for n in ("tray_front_left", "tray_front_right")}
    if min(wall_hits.values()) <= 0:
        res.fail(f"sanity: a probe grown by the {w} mm wall does not meet the tray halves ({wall_hits}); the tube is not where params say")
    # sanity 2: the pass-through opens into each pod interior (a thin slab above the pod floor top is clear)
    for side, s in (("left", 1), ("right", -1)):
        slab = box(geom["x"] - 7.5, geom["x"] + 7.5, s * geom["pass_through_y"] - 5, s * geom["pass_through_y"] + 5,
                   geom["pod_floor_top"], geom["pod_floor_top"] + 5)
        v = float((slab & bodies[f"pod_{side}"]).volume)
        if v > VOLUME_LIMIT:
            res.fail(f"sanity: the pass-through into pod_{side} is blocked above the pod floor ({v:.1f} mm3 of pod in the way)")

    # fasteners from params whose axis crosses the tube interior
    t = p["tray"]
    zf = L["floor_z0"] - t["flange"]["height"] / 2
    crossing = []
    x_lo, x_hi = geom["x"] - geom["clear_width"] / 2, geom["x"] + geom["clear_width"] / 2
    for kind, xs_, r in (("centre bolt", t["centre_bolt_x"], p["fasteners"]["insert_hole"] / 2),
                         ("centre pin", t["centre_pin_x"], p["fasteners_extra"]["pin_socket_dia"] / 2)):
        for x in xs_:
            if x_lo - r < x < x_hi + r and geom["z"][0] - r < zf < geom["z"][1] + r:
                crossing.append({"fastener": kind, "x": x, "z": zf})
                res.fail(f"params.tray.centre_{'bolt' if kind == 'centre bolt' else 'pin'}_x = {x} lies inside the channel tube interior "
                         f"(x {x_lo}..{x_hi}, z {geom['z'][0]}..{geom['z'][1]} at the flange height {zf}): the bolt would cross the cable channel "
                         f"and the hole cuts no flange material (see hole_alignment)")

    worst = max([r["volume_mm3"] or 0 for r in rows], default=0.0)
    res.data = {"geometry": geom, "volume_limit_mm3": VOLUME_LIMIT, "rows": rows, "wall_sanity_mm3": wall_hits, "fasteners_crossing": crossing}
    res.summary = (f"{geom['clear_width']} x {geom['clear_height']} mm probe along {2 * geom['y_extent']:.0f} mm of tube plus two pass-throughs "
                   f"to z={geom['pod_floor_top']:.0f}: {len(rows)} bodies tested, largest intrusion {worst:.2f} mm3; "
                   f"{len(crossing)} fastener(s) cross the channel")
    res.details = [f"{r['probe']:34s} {r['body']:26s} {r['volume_mm3']!s:>8s} mm3" for r in rows]
    res.details.append(f"tube walls around the probe: {wall_hits} mm3 of tray material in the grown probe")
    res.seconds = time.time() - t0
    return res


if __name__ == "__main__":
    r = run(Context())
    print(r.summary)
    print("\n".join(r.details))
    for f in r.failures:
        print("FAIL:", f)
    for w in r.warnings:
        print("WARN:", w)
    sys.exit(0 if r.status != FAIL else 1)
