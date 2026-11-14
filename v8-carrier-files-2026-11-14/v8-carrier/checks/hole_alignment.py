"""Check 3: M3 features line up across every bolted joint.

For every joint the fastener features are found on both sides from the
circular edges of the known diameters (``Context.holes``): insert holes
(4.0 mm), clearance holes (3.4 mm), dowel sockets (4.1 mm), the nose-rim
counterbores (6.5 mm), the M5 pivot holes (5.3 mm) and the 625ZZ seats
(16 mm).  A feature belongs to a joint when its axis is the joint's bolt
axis and one end of it lies within ``PLANE_TOL`` of the joint plane (or it
spans the plane).  Each insert hole on the insert side must have a coaxial
clearance hole on the mating side within ``COAX_TOL`` (0.2 mm), each dowel
socket a coaxial socket, each M5 pivot hole a coaxial pivot hole / bearing
seat.  Clearance holes with no insert behind them are reported too.

Expected counts and positions come from ``params.yaml`` (and the part
modules' layout helpers), so a bolt the model lost (a hole that cut nothing)
shows up as "expected but not found".  Insert holes must be at least
``fasteners.insert_depth`` deep and sockets ``fasteners_extra.pin_socket_depth``.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field

from checks.common import FAIL, Context, CheckResult, Hole, dist_axes

NAME = "hole_alignment"
COAX_TOL = 0.2  # mm, axis to axis
PLANE_TOL = 0.6  # mm, hole end to joint plane (the insert cut overshoots the face by 0.5)
POS_TOL = 0.25  # mm, found feature to expected position


@dataclass
class Pair:
    kind_a: str
    kind_b: str
    label: str
    expected: list[tuple[float, float]] = field(default_factory=list)  # (u, v) on the axis plane
    min_depth_a: float | None = None
    min_depth_b: float | None = None
    plane_tol_b: float | None = None  # mating-side features may sit further from the plane (counterbores)


@dataclass
class Joint:
    name: str
    a: list[str]  # pieces carrying the inserts / sockets
    b: list[str]  # mating pieces with the clearance holes / sockets
    axis: str
    plane: float
    pairs: list[Pair]
    plane_tol: float = PLANE_TOL
    fasteners: str = ""  # free text for the report / assembly doc


def _near_plane(h: Hole, plane: float, tol: float) -> bool:
    return abs(h.a0 - plane) <= tol or abs(h.a1 - plane) <= tol or (h.a0 < plane < h.a1)


def joints(ctx: Context) -> list[Joint]:
    p, L = ctx.p, ctx.L
    t, n, wm, f, fx = p["tray"], p["nose"], p["wheel_module"], p["fasteners"], p["fasteners_extra"]
    zf = L["floor_z0"] - t["flange"]["height"] / 2  # bolt axis height in the tray flanges
    xs = t["cross_split_x"]
    ins_d, pin_d = f["insert_depth"], fx["pin_socket_depth"]

    from cad.parts import side_pods  # layout helpers only (read-only use)
    from cad.parts.nose import geom as nose_geom

    g = side_pods._geom(p)
    lid_xy = side_pods._lid_bolt_xy(g)
    ng = nose_geom(p)
    cap = n["cap"]
    cap_pts = []
    for i in range(cap["bolt_count"]):
        a = math.pi / 2 + 2 * math.pi * i / cap["bolt_count"]
        cap_pts.append((ng["bar_x"] + cap["bolt_circle"] / 2 * math.cos(a), ng["bar_z"] + cap["bolt_circle"] / 2 * math.sin(a)))
    tw = n["tower"]
    ts = tw["size"] / 2
    col = p["pods"]["corner_column"]
    col_pts = [(ng["tower_cx"] + sx * (ts - tw["wall"] - col / 2), sy * (ts - tw["wall"] - col / 2)) for sx in (1, -1) for sy in (1, -1)]
    tz1 = tw["platform_z"] - 3
    bm = p["bumper"]
    jp = bm["joining_plate"]
    boss_pts = [(sy * jp["bolt_dx"], bm["tof_z"] + sz * jp["bolt_dz"]) for sy in (1, -1) for sz in (1, -1)]
    plate_x = L["x_front"] - bm["shell"] - jp["boss_height"]

    def bolt(expected, min_a=ins_d):
        return Pair("insert", "clearance", "M3 insert -> clearance", expected, min_a, None)

    def pin(expected):
        return Pair("pin", "pin", "4 mm dowel socket -> socket", expected, pin_d, pin_d)

    out = []
    # tray centreline (front and rear pairs): Y bolts through the centre flange, inserts in the left halves
    front_x = [x for x in t["centre_bolt_x"] if x > xs]
    rear_x = [x for x in t["centre_bolt_x"] if x <= xs]
    out.append(Joint("tray centreline, front halves", ["tray_front_left"], ["tray_front_right"], "y", 0.0,
                     [bolt([(x, zf) for x in front_x]), pin([(x, zf) for x in t["centre_pin_x"] if x > xs])],
                     fasteners=f"M3 x 12 horizontal through the right flange into inserts in the left flange; {len(front_x)} bolts"))
    out.append(Joint("tray centreline, rear halves", ["tray_rear_left"], ["tray_rear_right"], "y", 0.0,
                     [bolt([(x, zf) for x in rear_x]), pin([(x, zf) for x in t["centre_pin_x"] if x <= xs])],
                     fasteners=f"M3 x 12 horizontal through the right flange into inserts in the left flange; {len(rear_x)} bolts"))
    # tray cross split per side: X bolts, inserts in the front pieces
    for side, sy in (("left", 1), ("right", -1)):
        out.append(Joint(f"tray cross split ({side})", [f"tray_front_{side}"], [f"tray_rear_{side}"], "x", float(xs),
                         [bolt([(sy * y, zf) for y in t["cross_bolt_y"]]), pin([(sy * y, zf) for y in t["cross_pin_y"]])],
                         fasteners="M3 x 12 horizontal through the rear flange into inserts in the front flange"))
    # nose centreline
    out.append(Joint("nose centreline", ["nose_left"], ["nose_right"], "y", 0.0,
                     [bolt([tuple(b) for b in n["split_bolts"]]), pin([tuple(q) for q in n["split_pins"]])],
                     fasteners="M3 x 12 horizontal through the right rib into inserts in the left rib"))
    out.append(Joint("lidar plate to nose tower", ["nose_left", "nose_right"], ["lidar_plate"], "z", float(tz1),
                     [bolt(col_pts)], fasteners="M3 x 8 down through the plate into the tower column inserts"))
    out.append(Joint("brush cap to nose (left housing end)", ["nose_left"], ["brush_cap"], "y", float(ng["cav_y1"]),
                     [bolt(cap_pts)], fasteners="M3 x 10 through the cap into the housing end bosses"))
    out.append(Joint("bumper halves via joining plate", ["bumper_left", "bumper_right"], ["bumper_joining_plate"], "x", float(plate_x),
                     [bolt(boss_pts)], fasteners="M3 x 8 through the plate into the bosses on the back of both halves"))
    # nose to tray: inserts in the nose rear wall, clearance + counterbore in the tray's nose rim
    tray_pts = [(sy * y, t["nose_bolt_z"]) for y in t["nose_bolt_y"] for sy in (1, -1)]
    rim_t = t["nose_rim"]["thickness"]
    out.append(Joint("nose to tray (nose rim)", ["nose_left", "nose_right"], ["tray_front_left", "tray_front_right"], "x", float(L["nose_rear"]),
                     [bolt(tray_pts), Pair("insert", "cbore", "counterbore for the head", tray_pts, plane_tol_b=rim_t + 1)],
                     fasteners="M3 x 16 forward through the tray's nose rim (heads counterbored in the rim's rear face) into nose inserts"))
    # pods to tray: vertical, inserts in the tray
    for side, sy in (("left", 1), ("right", -1)):
        pts = [(bx, sy * by) for bx in t["pod_bolt_x"] for by in t["pod_bolt_y"]]
        out.append(Joint(f"pod_{side} to tray", [f"tray_front_{side}", f"tray_rear_{side}"], [f"pod_{side}"], "z", float(L["floor_z1"]),
                         [bolt(pts)], fasteners="M3 x 8 down through the pod floor into tray inserts"))
    # wheel bracket to pod outer wall: Y, inserts in the pod; M5 pivot through both
    for side, sy in (("left", 1), ("right", -1)):
        pts = [tuple(b) for b in wm["bracket_bolts"]]
        out.append(Joint(f"wheel bracket to pod ({side})", [f"pod_{side}"], [f"wheel_bracket_{side}"], "y", sy * L["pod_outer"],
                         [bolt(pts), Pair("m5", "m5", "M5 pivot hole -> pivot hole", [(wm["arm_length"], L["axle_z"])])],
                         fasteners="M3 x 10 through the bracket plate into pod inserts; M5 x 30 pivot through bracket and wall"))
        y_arm = sy * (L["pod_outer"] + wm["bracket_thickness"] + wm["arm_gap"])
        out.append(Joint(f"wheel arm pivot on bracket ({side})", [f"wheel_bracket_{side}"], [f"wheel_arm_{side}"], "y", float(y_arm),
                         [Pair("m5", "brg625", "M5 pivot hole -> 625ZZ seat in the arm", [(wm["arm_length"], L["axle_z"])])],
                         plane_tol=1.5, fasteners="M5 x 30 through the arm's 625ZZ, the bracket and the pod wall; nylock inside"))
    # lids
    for side, sy in (("left", 1), ("right", -1)):
        pts = [(x, sy * y) for x, y in lid_xy]
        out.append(Joint(f"lid_{side} to pod_{side}", [f"pod_{side}"], [f"lid_{side}"], "z", float(L["pod_z1"]),
                         [bolt(pts)], fasteners="M3 x 8 down through the lid into the corner-column inserts"))
    return out


def check_joint(ctx: Context, j: Joint) -> dict:
    report = {"joint": j.name, "a": j.a, "b": j.b, "axis": j.axis, "plane": j.plane, "fasteners": j.fasteners, "pairs": []}
    for pr in j.pairs:
        fa_all = [(piece, h) for piece in j.a for h in ctx.holes(piece) if h.kind == pr.kind_a and h.axis == j.axis and _near_plane(h, j.plane, j.plane_tol)]
        tol_b = pr.plane_tol_b if pr.plane_tol_b is not None else j.plane_tol
        fb = [(piece, h) for piece in j.b for h in ctx.holes(piece) if h.kind == pr.kind_b and h.axis == j.axis and _near_plane(h, j.plane, tol_b)]
        if pr.expected:
            # only the features this joint owns; other features on the same face (e.g. the nose's duct-flange
            # inserts beside the tray bolts) are listed for information
            fa = [(pc, h) for pc, h in fa_all if any(math.hypot(h.uv[0] - u, h.uv[1] - v) <= POS_TOL for u, v in pr.expected)]
            other_a = [f"{pc} {h}" for pc, h in fa_all if (pc, h) not in fa]
        else:
            fa, other_a = fa_all, []
        matched, unmatched_a, used_b = [], [], set()
        for pa, ha in fa:
            best = None
            for k, (pb, hb) in enumerate(fb):
                d = dist_axes(ha, hb)
                if d <= COAX_TOL and (best is None or d < best[0]):
                    best = (d, k, pb, hb)
            if best is None:
                unmatched_a.append({"piece": pa, "feature": str(ha)})
            else:
                used_b.add(best[1])
                matched.append({"a": pa, "b": best[2], "offset_mm": round(best[0], 3), "feature": str(ha),
                                "depth_a": round(ha.depth, 2), "depth_b": round(best[3].depth, 2)})
        unmatched_b = [{"piece": pb, "feature": str(hb)} for k, (pb, hb) in enumerate(fb) if k not in used_b]
        # expected positions not found on the A side
        missing = []
        for u, v in pr.expected:
            if not any(math.hypot(h.uv[0] - u, h.uv[1] - v) <= POS_TOL for _, h in fa):
                missing.append((round(u, 2), round(v, 2)))
        shallow = [{"piece": pa, "feature": str(ha), "depth": round(ha.depth, 2), "min": pr.min_depth_a}
                   for pa, ha in fa if pr.min_depth_a is not None and ha.depth < pr.min_depth_a - 0.05]
        shallow += [{"piece": pb, "feature": str(hb), "depth": round(hb.depth, 2), "min": pr.min_depth_b}
                    for pb, hb in fb if pr.min_depth_b is not None and hb.depth < pr.min_depth_b - 0.05]
        report["pairs"].append({
            "label": pr.label, "expected": len(pr.expected), "found_a": len(fa), "found_b": len(fb),
            "matched": len(matched), "unmatched_a": unmatched_a, "unmatched_b": unmatched_b,
            "missing_expected": missing, "shallow": shallow, "matches": matched, "other_features_on_face": other_a,
        })
    return report


def run(ctx: Context) -> CheckResult:
    t0 = time.time()
    res = CheckResult(NAME)
    reports = []
    tot_m = tot_u = 0
    for j in joints(ctx):
        r = check_joint(ctx, j)
        reports.append(r)
        line_parts = []
        for pr in r["pairs"]:
            m, ua, ub, miss = pr["matched"], len(pr["unmatched_a"]), len(pr["unmatched_b"]), len(pr["missing_expected"])
            tot_m += m
            tot_u += ua + ub + miss
            line_parts.append(f"{pr['label'].split(' ->')[0]}: {m}/{pr['expected']} matched" + (f", {ua} unmatched" if ua else "") + (f", {ub} orphan on mating side" if ub else ""))
            for u in pr["unmatched_a"]:
                res.fail(f"{j.name}: {u['piece']} {u['feature']} has no coaxial {pr['label'].split('-> ')[-1]} on {'/'.join(j.b)} within {COAX_TOL} mm")
            for u in pr["unmatched_b"]:
                res.fail(f"{j.name}: {u['piece']} {u['feature']} has no coaxial {pr['label'].split(' ->')[0]} on {'/'.join(j.a)} within {COAX_TOL} mm")
            if pr["missing_expected"]:
                res.fail(f"{j.name}: {pr['label'].split(' ->')[0]} expected at {pr['missing_expected']} (params) but not found on {'/'.join(j.a)}: the hole cut no material")
            for s in pr["shallow"]:
                res.fail(f"{j.name}: {s['piece']} {s['feature']} is only {s['depth']} mm deep (needs {s['min']} mm)")
            for mt in pr["matches"]:
                if mt["offset_mm"] > 0.0:
                    res.warn(f"{j.name}: {mt['feature']} offset {mt['offset_mm']} mm between {mt['a']} and {mt['b']}")
        r["line"] = "; ".join(line_parts)
        res.details.append(f"{j.name:42s} {r['line']}")
    res.data = {"coax_tol_mm": COAX_TOL, "plane_tol_mm": PLANE_TOL, "joints": reports}
    res.summary = f"{len(reports)} joints, {tot_m} features matched, {tot_u} unmatched/missing/orphan; {len(res.failures)} failures"
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
