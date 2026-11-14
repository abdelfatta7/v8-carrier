"""Check 6: wall thickness sanity (cheap ray-cast estimate).

For every exported STL, a ray is cast inward from the centre of every
triangle along its normal and the distance to the next surface is the local
thickness (a numpy Moller-Trumbore caster on a uniform grid; no rtree or
embree needed).  Rays are capped at ``RAY_MAX`` mm, so only thin material is
measured exactly.  A sample thinner than ``MIN_WALL`` (2.5 mm) is a failure
unless it lies in a zone the parameters deliberately make thinner:

* the wall of an 8 mm boss around a 4.0 mm insert hole (2.0 mm, every boss
  and standoff in the design) and the land around other fastener holes
* the 1 mm lip behind a 625ZZ seat (arm, brush cap, nose drive-end wall)
* the pod walls recessed ``pods.pi_recess`` over the Pi (2.0 mm)
* the bumper cliff pockets (``bumper.cliff_pocket.wall`` = 2 mm)
* the bracket ledge roof over the spring pocket (6 - 4 = 2 mm)

Those zones still have a floor (``ZONE_FLOOR``), so a boss wall that has
been eaten to 1 mm or a hole that breaks into a window (0 mm) fails.  Thin
samples are clustered and reported with their location and face normal.
Limits of the method: it measures along the normal only, so knife-edge
wedges where a cylindrical cut crosses a plate show up as very thin spots
(they are real, but tapered, geometry); internal faces (slivers) show up as
~0.01 mm.
"""

from __future__ import annotations

import sys
import time
from collections import defaultdict

import numpy as np
import trimesh

from checks.common import FAIL, Context, CheckResult

NAME = "wall_thickness"
MIN_WALL = 2.5  # mm
RAY_MAX = 12.0  # mm; thicker than this is not measured exactly
CLUSTER_MM = 8.0
FACET_TOL = 0.05  # mm; STL facets of a cylinder sit inside the true surface by up to the export tolerance


def cast_thickness(mesh: trimesh.Trimesh, ray_max: float = RAY_MAX) -> np.ndarray:
    """Thickness at every triangle centre along -normal (inf when nothing within ray_max)."""
    tri = mesh.triangles
    origins = mesh.triangles_center - mesh.face_normals * 1e-4
    dirs = -mesh.face_normals
    cell = ray_max
    lo = tri.reshape(-1, 3).min(axis=0) - 1.0
    tb0 = np.floor((tri.min(axis=1) - lo) / cell).astype(int)
    tb1 = np.floor((tri.max(axis=1) - lo) / cell).astype(int)
    grid: dict[tuple, list] = defaultdict(list)
    for k in range(len(tri)):
        for i in range(tb0[k, 0], tb1[k, 0] + 1):
            for j in range(tb0[k, 1], tb1[k, 1] + 1):
                for l in range(tb0[k, 2], tb1[k, 2] + 1):
                    grid[(i, j, l)].append(k)
    grid = {key: np.array(v) for key, v in grid.items()}
    ray_cell = np.floor((origins - lo) / cell).astype(int)
    groups: dict[tuple, list] = defaultdict(list)
    for r, key in enumerate(map(tuple, ray_cell)):
        groups[key].append(r)
    v0 = tri[:, 0]
    e1 = tri[:, 1] - v0
    e2 = tri[:, 2] - v0
    out = np.full(len(origins), np.inf)
    for key, rays in groups.items():
        cand = []
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for dl in (-1, 0, 1):
                    c = grid.get((key[0] + di, key[1] + dj, key[2] + dl))
                    if c is not None:
                        cand.append(c)
        if not cand:
            continue
        cand = np.unique(np.concatenate(cand))
        rays = np.array(rays)
        o = origins[rays]
        d = dirs[rays]
        pvec = np.cross(d[:, None, :], e2[cand][None, :, :])
        det = np.einsum("mk,nmk->nm", e1[cand], pvec)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        tvec = o[:, None, :] - v0[cand][None, :, :]
        u = np.einsum("nmk,nmk->nm", tvec, pvec) * inv
        qvec = np.cross(tvec, e1[cand][None, :, :])
        v = np.einsum("nk,nmk->nm", d, qvec) * inv
        t = np.einsum("mk,nmk->nm", e2[cand], qvec) * inv
        hit = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-3) & (t <= ray_max)
        t = np.where(hit, t, np.inf)
        out[rays] = t.min(axis=1)
    return out


# --------------------------------------------------------------------------
# zones the parameters deliberately make thinner than MIN_WALL
# --------------------------------------------------------------------------
def zones(ctx: Context, piece: str):
    """List of (label, floor_mm, predicate(points (n,3)) -> bool array)."""
    p, L = ctx.p, ctx.L
    f = p["fasteners"]
    boss_r = f["boss_od"] / 2 + 0.6
    convention = f"{f['boss_od']} mm boss / {f['insert_hole']} mm insert hole = {f['boss_od'] / 2 - f['insert_hole'] / 2:.1f} mm wall"
    out = []
    holes = ctx.holes(piece if piece in ctx.pieces else piece)
    fast = [h for h in holes if h.kind in ("insert", "clearance", "pin", "m2", "m25", "cbore")]
    brg = [h for h in holes if h.kind == "brg625"]

    def near_axes(hs, radius, axial_pad):
        def pred(pts):
            res = np.zeros(len(pts), bool)
            for h in hs:
                i = "xyz".index(h.axis)
                others = [k for k in range(3) if k != i]
                d = np.hypot(pts[:, others[0]] - h.uv[0], pts[:, others[1]] - h.uv[1])
                a = pts[:, i]
                res |= (d <= radius) & (a >= h.a0 - axial_pad) & (a <= h.a1 + axial_pad)
            return res
        return pred

    if fast:
        out.append((f"boss / land around a fastener hole ({convention})", 1.8, near_axes(fast, boss_r, 6.0)))
    if brg:
        out.append(("1 mm lip behind a 625ZZ seat (seat depth = bearing width, wall 1 mm thicker)", 0.9, near_axes(brg, p["wheels"]["pivot_bearing"]["od"] / 2 + 1, 2.0)))
    if piece == "pod_right":
        lay = p["pods"]["electronics_layout"]["pi5"]
        xa, xb = sorted(lay["x"])
        wall = p["body"]["wall"] - p["pods"]["pi_recess"]
        out.append((f"pod wall recessed pods.pi_recess over the Pi ({wall:.1f} mm)", wall - 0.1,
                    lambda pts: (pts[:, 0] >= xa - 2.5) & (pts[:, 0] <= xb + 2.5) & (np.abs(pts[:, 1]) >= L["pod_inner"] - 0.5) & (np.abs(pts[:, 1]) <= L["pod_outer"] + 0.5)))
    if piece.startswith("bumper_") and piece != "bumper_joining_plate":
        b = p["bumper"]
        cp, cb = b["cliff_pocket"], p["sensors"]["cliff_body"]
        x0 = L["x_front"] - b["shell"]
        gp = b["guide_pin"]
        pin_wall = (gp["dia"] - p["fasteners_extra"]["m2_hole"]) / 2
        pin_hole_x = x0 - gp["length"] + 4  # bumper.py: cross hole 4 mm from the pin tip
        out.append((f"guide pin beside its M2 end-stop cross hole ((pin {gp['dia']} - hole {p['fasteners_extra']['m2_hole']}) / 2 = {pin_wall} mm)", pin_wall - 0.2,
                    lambda pts: (np.abs(pts[:, 0] - pin_hole_x) <= 3) & (np.abs(np.abs(pts[:, 1]) - gp["y"]) <= gp["dia"] / 2 + 0.5)
                    & (np.abs(pts[:, 2] - p["nose"]["guide_pin_z"]) <= gp["dia"] / 2 + 0.5)))
        in_x = cb["width"] + 2 * cp["clearance"]
        in_y = cb["length"] + 2 * cp["clearance"]
        in_z = cb["height"] + cp["clearance"]
        z0 = b["z"][0]
        out.append((f"cliff pocket wall (bumper.cliff_pocket.wall = {cp['wall']} mm)", cp["wall"] - 0.1,
                    lambda pts: (pts[:, 0] >= x0 - in_x - cp["wall"] - 0.5) & (pts[:, 0] <= x0 + 0.5)
                    & (np.abs(np.abs(pts[:, 1]) - b["cliff_nose_y"]) <= in_y / 2 + cp["wall"] + 0.5)
                    & (pts[:, 2] >= z0 - 0.5) & (pts[:, 2] <= z0 + cp["wall"] + in_z + 0.5)))
    if piece.startswith("wheel_bracket"):
        wm = p["wheel_module"]
        roof = wm["ledge"]["thickness"] - wm["spring_pocket"]["depth"]
        base = L["axle_z"] + p["motors"]["drive"]["diameter"] / 2 + 3.5
        lz0 = base + wm["seat_gap"]
        out.append((f"ledge roof over the spring pocket (ledge.thickness - spring_pocket.depth = {roof} mm)", roof - 0.1,
                    lambda pts: (np.abs(pts[:, 0]) <= wm["ledge"]["length"] / 2 + 0.5) & (pts[:, 2] >= lz0 - 0.5) & (pts[:, 2] <= lz0 + wm["ledge"]["thickness"] + 0.5)
                    & (np.abs(pts[:, 1]) >= L["pod_outer"] + wm["bracket_thickness"] - 0.5)))
    return out


def clusters(points: np.ndarray, values: np.ndarray, normals: np.ndarray, radius: float):
    """Greedy clustering; returns (min value, its point, its normal, count) per cluster."""
    order = np.argsort(values)
    taken = np.zeros(len(points), bool)
    out = []
    for i in order:
        if taken[i]:
            continue
        near = np.linalg.norm(points - points[i], axis=1) <= radius
        near &= ~taken
        taken |= near
        out.append((float(values[i]), points[i], normals[i], int(near.sum())))
    return out


def run(ctx: Context) -> CheckResult:
    t0 = time.time()
    res = CheckResult(NAME)
    rows = []
    pieces = list(ctx.manifest.get("pieces", {})) or ctx.print_pieces
    for piece in pieces:
        path = ctx.stl_path(piece)
        if not path.exists():
            res.fail(f"{piece}: {path} missing")
            continue
        mesh = trimesh.load(path, force="mesh")
        th = cast_thickness(mesh)
        pts = mesh.triangles_center
        nrm = mesh.face_normals
        thin = np.where(th < MIN_WALL - FACET_TOL)[0]
        finite = th[np.isfinite(th)]
        row = {"piece": piece, "samples": int(len(th)), "min_mm": round(float(finite.min()), 2) if len(finite) else None,
               "thin_samples": int(len(thin)), "clusters": []}
        if len(thin):
            zs = zones(ctx, piece)
            label = np.array(["unexplained"] * len(thin), dtype=object)
            floor = np.full(len(thin), MIN_WALL)
            for name, zfloor, pred in zs:
                m = pred(pts[thin]) & (label == "unexplained")
                label[m] = name
                floor[m] = zfloor
            for name in dict.fromkeys(label):
                sel = thin[label == name]
                fl = float(floor[label == name][0])
                ok_clusters = []
                for tmin, pt, n, count in clusters(pts[sel], th[sel], nrm[sel], CLUSTER_MM):
                    c = {"zone": name, "min_mm": round(tmin, 2), "at": [round(float(v), 1) for v in pt],
                         "normal": [round(float(v), 2) for v in n], "samples": count, "floor_mm": fl, "ok": bool(tmin >= fl - FACET_TOL)}
                    row["clusters"].append(c)
                    where = f"{piece} at {c['at']} (normal {c['normal']}, {count} samples)"
                    if name == "unexplained":
                        res.fail(f"{where}: {tmin:.2f} mm wall < {MIN_WALL} mm")
                    elif not c["ok"]:
                        res.fail(f"{where}: {tmin:.2f} mm in zone '{name}', below that zone's own floor {fl} mm")
                    else:
                        ok_clusters.append(c)
                if ok_clusters:
                    res.warn(f"{piece}: {len(ok_clusters)} spot(s) at {min(c['min_mm'] for c in ok_clusters):.2f} mm "
                             f"({sum(c['samples'] for c in ok_clusters)} samples), deliberate: {name}")
        rows.append(row)
    res.data = {"min_wall_mm": MIN_WALL, "ray_max_mm": RAY_MAX, "rows": rows}
    n_bad = sum(1 for r in rows for c in r["clusters"] if not c["ok"])
    n_pieces_bad = sum(1 for r in rows if any(not c["ok"] for c in r["clusters"]))
    res.summary = f"{len(rows)} pieces ray-sampled ({sum(r['samples'] for r in rows)} rays); {n_pieces_bad} pieces with {n_bad} thin spots under {MIN_WALL} mm not explained by params"
    res.details = [f"{r['piece']:24s} min {r['min_mm']!s:>5s} mm  thin samples {r['thin_samples']:5d}  "
                   f"clusters: {', '.join(f'{c['min_mm']}@{c['at']}' + ('' if c['ok'] else ' FAIL') for c in r['clusters'][:4])}"
                   for r in rows]
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
