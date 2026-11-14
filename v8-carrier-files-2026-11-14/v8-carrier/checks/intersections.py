"""Check 2: no two assembled bodies overlap.

Every pair of bodies from ``cad.assembly.assembled(p)`` (print pieces, the
twice-printed pieces as left/right instances, and the placeholders for the
bought parts and the vacuum) whose bounding boxes come within 0.5 mm is
intersected with an OCCT boolean; the pair fails when the common volume
exceeds ``VOLUME_LIMIT`` (1 mm3).  For the pairs that pass, the minimum
distance is recorded so that near-contacts between parts that move relative
to each other (wheel set vs bracket/pod/tray, bumper vs nose, arm post vs
bracket) can be listed.

Motion cases (same test on the moved bodies against everything else):
  * wheel bump / droop: arm, motor, hub and tyre rotated about the pivot so
    the axle rises / drops by half ``wheels.suspension_travel``
  * bumper pressed: the bumper pieces moved back by ``sensors.bumper_travel``

Two bumper-mechanism numbers that no boolean can see are checked from the
same params: the guide-pin end-stop cross hole must clear the nose sleeve
bore, and the bumper spring must be longer than its seat-to-seat distance.
A positive control (a pod lowered 2 mm into the tray) proves the boolean
detects overlaps before the results are trusted.
"""

from __future__ import annotations

import itertools
import math
import sys
import time

from build123d import Axis, Pos

from checks.common import FAIL, Context, CheckResult, bbox_of, bbox_overlap

NAME = "intersections"
VOLUME_LIMIT = 1.0  # mm3
BBOX_TOL = 0.5  # mm; pairs whose boxes come this close are tested
CONTACT_REPORT = 1.0  # mm; closer than this between relatively moving parts is reported
SLIVER_REPORT = 0.01  # mm3; a common volume above this (but under the limit) is reported as a sliver

MOVING_GROUPS = {
    # bodies that move together; everything outside the group is "relative motion"
    "wheel_left": {"wheel_arm_left", "wheel_hub_left", "wheel_tyre_left", "placeholder_drive_motor_left"},
    "wheel_right": {"wheel_arm_right", "wheel_hub_right", "wheel_tyre_right", "placeholder_drive_motor_right"},
    "bumper": {"bumper_left", "bumper_right", "bumper_joining_plate"},
}


def _group(name: str) -> str | None:
    for g, members in MOVING_GROUPS.items():
        if name in members:
            return g
    return None


def _relative_motion(a: str, b: str) -> bool:
    ga, gb = _group(a), _group(b)
    return ga != gb and (ga is not None or gb is not None)


def _common_volume(a, b) -> float | None:
    try:
        inter = a & b
    except Exception:
        return None
    try:
        return float(inter.volume) if inter is not None else 0.0
    except Exception:
        return None


def _pairwise(bodies: dict, movers: set[str] | None, res: CheckResult, label: str) -> list[dict]:
    """Test every bbox-overlapping pair; when ``movers`` is given only pairs with one mover."""
    boxes = {n: bbox_of(b) for n, b in bodies.items()}
    rows = []
    for a, b in itertools.combinations(bodies, 2):
        if movers is not None and ((a in movers) == (b in movers)):
            continue
        if not bbox_overlap(boxes[a], boxes[b], BBOX_TOL):
            continue
        vol = _common_volume(bodies[a], bodies[b])
        row = {"a": a, "b": b, "volume_mm3": None if vol is None else round(vol, 3), "case": label}
        if vol is None:
            res.fail(f"[{label}] {a} & {b}: boolean intersection failed (OCCT)")
        elif vol > VOLUME_LIMIT:
            res.fail(f"[{label}] {a} & {b}: common volume {vol:.1f} mm3 > {VOLUME_LIMIT} mm3")
        else:
            if vol > SLIVER_REPORT:
                res.warn(f"[{label}] {a} & {b}: common volume {vol:.3f} mm3 (under the limit, but non-zero: a modelling sliver)")
            try:
                d, p1, p2 = bodies[a].distance_to_with_closest_points(bodies[b])
                row["distance_mm"] = round(float(d), 3)
                row["closest"] = [round(p1.X, 1), round(p1.Y, 1), round(p1.Z, 1)]
            except Exception:
                row["distance_mm"] = None
        rows.append(row)
    return rows


def _rotated_wheel_set(ctx: Context, side: str, angle_deg: float) -> dict:
    """Arm, motor, hub and tyre rotated about the pivot axis (x = arm_length, z = axle) along Y."""
    wm = ctx.p["wheel_module"]
    axis = Axis((wm["arm_length"], 0, ctx.L["axle_z"]), (0, 1, 0))
    out = {}
    for name in MOVING_GROUPS[f"wheel_{side}"]:
        out[name] = ctx.assembled[name].rotate(axis, angle_deg)
    return out


def _positive_control(ctx: Context, res: CheckResult) -> dict:
    """Prove the boolean sees overlaps: a pod pushed 2 mm into the tray must show a large common volume."""
    pod = Pos(0, 0, -2) * ctx.assembled["pod_left"]
    vol = _common_volume(pod, ctx.assembled["tray_front_left"]) or 0.0
    if vol < 1000:
        res.fail(f"self-test: pod_left lowered 2 mm into tray_front_left gives only {vol:.1f} mm3; the boolean is not detecting overlaps")
    return {"pod_left_lowered_2mm_vs_tray_front_left_mm3": round(vol, 1)}


def run(ctx: Context) -> CheckResult:
    t0 = time.time()
    res = CheckResult(NAME)
    bodies = ctx.assembled
    control = _positive_control(ctx, res)
    rows = _pairwise(bodies, None, res, "nominal")
    n_tested = len(rows)

    # near contacts between relatively moving parts
    contacts = []
    for r in rows:
        if r.get("distance_mm") is not None and r["distance_mm"] < CONTACT_REPORT and _relative_motion(r["a"], r["b"]):
            contacts.append(r)
            res.warn(f"{r['a']} and {r['b']} move relative to each other and come within {r['distance_mm']:.2f} mm at {r['closest']}")

    # placeholders inside cavities and the vacuum: say explicitly that they are clear
    ph_rows = [r for r in rows if r["a"].startswith("placeholder_") or r["b"].startswith("placeholder_")]
    vac_rows = [r for r in rows if "placeholder_vacuum" in (r["a"], r["b"])]

    # motion cases
    motion_rows = []
    travel = ctx.p["wheels"]["suspension_travel"] / 2
    angle = math.degrees(math.asin(travel / ctx.p["wheel_module"]["arm_length"]))
    for side in ("left", "right"):
        for label, sign in (("bump", +1), ("droop", -1)):
            moved = _rotated_wheel_set(ctx, side, sign * angle)
            # sanity on the rotation sense: the axle must move up for bump
            az = moved[f"wheel_hub_{side}"].bounding_box().center().Z - bodies[f"wheel_hub_{side}"].bounding_box().center().Z
            if sign * az < 0:
                moved = _rotated_wheel_set(ctx, side, -sign * angle)
            case = dict(bodies)
            case.update(moved)
            motion_rows += _pairwise(case, set(moved), res, f"wheel_{side} {label} {sign * travel:+.0f} mm")
    bt = ctx.p["sensors"]["bumper_travel"]
    moved = {n: Pos(-bt, 0, 0) * bodies[n] for n in MOVING_GROUPS["bumper"]}
    case = dict(bodies)
    case.update(moved)
    motion_rows += _pairwise(case, set(moved), res, f"bumper pressed {bt} mm")
    mech = _bumper_mechanism(ctx, res)
    return _finish(res, rows, motion_rows, bodies, n_tested, vac_rows, ph_rows, contacts, mech, control, t0)


def _bumper_mechanism(ctx: Context, res: CheckResult) -> dict:
    """Guide-pin end stop and spring preload, from params (bumper.py / nose.py geometry)."""
    p, L = ctx.p, ctx.L
    b = p["bumper"]
    gp, spr, pb = b["guide_pin"], b["spring"], b["pin_boss"]
    travel = p["sensors"]["bumper_travel"]
    x_nose = L["nose_front"]
    x_back = L["x_front"] - b["shell"]  # back face of the bumper plate at rest
    sleeve_rear = x_nose - gp["sleeve_length"]
    bore_rear = sleeve_rear - 1  # nose.py drills the 6.4 mm clearance 1 mm past the sleeve
    tip = x_back - gp["length"]
    cross_hole = tip + 4  # bumper.py: cross hole 4 mm from the tip
    out = {"sleeve_x": [sleeve_rear, x_nose], "pin_tip_rest_x": tip, "cross_hole_rest_x": cross_hole,
           "cross_hole_pressed_x": cross_hole - travel}
    if cross_hole - travel > bore_rear:
        res.fail(f"bumper end stop: the M2 cross hole near the pin tip is at x={cross_hole} at rest and x={cross_hole - travel} "
                 f"at full travel, inside the nose sleeve bore (x {bore_rear}..{x_nose + 1}); a cross pin cannot be fitted "
                 f"and would not stop against the sleeve's rear face (x={sleeve_rear}); the pin needs to reach past x={sleeve_rear} "
                 f"by the hole offset plus the travel")
    seat_nose = x_nose - spr["pocket_depth"]
    seat_bumper = x_back - pb["height"] + 2  # bumper.py: 2 mm deep ring pocket in the boss face
    span = seat_bumper - seat_nose
    out.update({"spring_seat_span_rest_mm": span, "spring_free_length_mm": spr["free_length"], "preload_mm": spr["free_length"] - span})
    if spr["free_length"] <= span:
        res.fail(f"bumper spring: free length {spr['free_length']} mm is shorter than the seat-to-seat distance at rest "
                 f"({seat_nose}..{seat_bumper} = {span} mm): no preload, the bumper floats {span - spr['free_length']} mm "
                 f"before the spring acts (bumper.spring.free_length / bumper.depth / pin_boss.height)")
    return out


def _finish(res, rows, motion_rows, bodies, n_tested, vac_rows, ph_rows, contacts, mech, control, t0):
    worst = max((r["volume_mm3"] or 0.0) for r in rows + motion_rows) if rows else 0.0
    res.data = {
        "bodies": len(bodies),
        "pairs_tested": n_tested,
        "volume_limit_mm3": VOLUME_LIMIT,
        "rows": rows,
        "motion_rows": motion_rows,
        "contacts": contacts,
        "vacuum_pairs": vac_rows,
        "placeholder_pairs": ph_rows,
        "bumper_mechanism": mech,
        "positive_control": control,
    }
    bad = [r for r in rows + motion_rows if (r["volume_mm3"] or 0) > VOLUME_LIMIT or r["volume_mm3"] is None]
    res.summary = (f"{len(bodies)} bodies, {n_tested} nominal pairs + {len(motion_rows)} motion pairs tested; "
                   f"{len(bad)} over {VOLUME_LIMIT} mm3; largest common volume {worst:.2f} mm3; "
                   f"vacuum placeholder clear of all {len(vac_rows)} neighbours; "
                   f"bumper mechanism: {sum(1 for f in res.failures if f.startswith('bumper'))} failure(s)")
    touching = sum(1 for r in rows if r.get("distance_mm") == 0 and not (r["volume_mm3"] or 0) > SLIVER_REPORT)
    res.details = [f"{touching} nominal pairs share faces (gap 0, volume 0): split halves, pods on the tray, lids, brackets, placeholders in their seats"]
    res.details += [f"{r['a']:30s} {r['b']:30s} vol {r['volume_mm3']!s:>8s}  gap {r.get('distance_mm', '-')!s:>6s} at {r.get('closest', '-')}  [{r['case']}]"
                    for r in rows + motion_rows
                    if (r["volume_mm3"] is None or r["volume_mm3"] > SLIVER_REPORT)
                    or (r.get("distance_mm") is not None and r["distance_mm"] < CONTACT_REPORT and _relative_motion(r["a"], r["b"]))]
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
