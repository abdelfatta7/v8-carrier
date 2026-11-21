#!/usr/bin/env python3
"""Independent fit check of the bolt-on shell covers against the 16 Nov assembly.

Run from the repo root:  python3 cad/shell/check_shell.py
Reads cad/shell/out/*.stl (as installed, robot frame) and
cad/shell/reference/model_2026-11-16.json, writes cad/shell/out/fit-report.md.
Headless; needs numpy, scipy, trimesh, pyyaml, matplotlib (Path only).  No rtree.

Distance method: dense surface sampling of both sides (KD-trees, cover >= 150k
samples + all vertices, reference ~3 samples/mm^2 + all vertices), then the
closest candidates of every pair are refined with exact point-to-triangle
distances in both directions.  Overlap: generalised winding number of the
near-contact points of one mesh against the other mesh.
"""
from __future__ import annotations

import base64
import json
import math
import os
import sys

import numpy as np
import trimesh
import yaml
from matplotlib.path import Path
from scipy import ndimage
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
REF = os.path.join(HERE, "reference", "model_2026-11-16.json")
OLD = os.path.join(os.path.dirname(os.path.dirname(HERE)), "v8-carrier-files-2026-11-14", "v8-carrier")
COVERS = ["pod_cover_left", "pod_cover_right", "nose_shroud_left", "nose_shroud_right"]
RNG = np.random.default_rng(1)
N_COVER = 150_000
REF_DENSITY = 3.0          # samples per mm^2
CLEAR = 3.0

rows = []                  # (check, result, number, note)
detail = []                # extra markdown sections


def add(check, ok, number, note=""):
    res = ok if isinstance(ok, str) else ("PASS" if bool(ok) else "FAIL")
    rows.append((check, res, number, note))
    print(f"[{res}] {check}: {number}  {note}")


# ---------------------------------------------------------------------------
# loading
def load_reference():
    d = json.load(open(REF))
    out, info = {}, {}
    bad = []
    for m in d["meshes"]:
        v = np.frombuffer(base64.b64decode(m["v"]), dtype="<i2").astype(float).reshape(-1, 3) * 0.05
        dt = "<u2" if m.get("itype") == "u16" else "<u4"
        f = np.frombuffer(base64.b64decode(m["i"]), dtype=dt).astype(np.int64).reshape(-1, 3)
        bb = np.array(m["bbox"], float)
        err = np.abs(np.array([v.min(0), v.max(0)]) - bb).max()
        if err > 0.06 or f.max() >= len(v):
            bad.append((m["name"], err))
        tm = trimesh.Trimesh(v, f, process=False)
        tm.merge_vertices()
        out[m["name"]] = tm
        info[m["name"]] = m
    return out, info, d, bad


def sample(tm, n=None, density=None):
    if n is None:
        n = int(max(2000, min(600_000, tm.area * density)))
    p, fi = trimesh.sample.sample_surface(tm, n, seed=int(RNG.integers(1e9)))
    return np.vstack([p, tm.vertices]), np.concatenate([fi, -np.ones(len(tm.vertices), int)])


# ---------------------------------------------------------------------------
# exact geometry helpers
def closest_on_tris(points, tm, chunk=1_500_000):
    """Exact distance from each point to the nearest triangle of tm (pre-filtered by AABB)."""
    tris = tm.triangles
    tmin, tmax = tris.min(1), tris.max(1)
    out = np.full(len(points), np.inf)
    cp_out = np.zeros_like(points)
    for k, p in enumerate(points):
        # AABB distance lower bound
        dlo = np.linalg.norm(np.maximum(0, np.maximum(tmin - p, p - tmax)), axis=1)
        order = np.argsort(dlo)[:400]
        T = tris[order]
        cp = trimesh.triangles.closest_point(T, np.repeat(p[None], len(T), 0))
        dd = np.linalg.norm(cp - p, axis=1)
        j = np.argmin(dd)
        out[k] = dd[j]
        cp_out[k] = cp[j]
    return out, cp_out


def winding(points, tm, chunk=200_000):
    """Generalised winding number of points w.r.t. a (near-)closed mesh."""
    T = tm.triangles
    w = np.zeros(len(points))
    step = max(1, chunk // len(T))
    for s in range(0, len(points), step):
        P = points[s:s + step]
        a = T[None, :, 0, :] - P[:, None, :]
        b = T[None, :, 1, :] - P[:, None, :]
        c = T[None, :, 2, :] - P[:, None, :]
        la, lb, lc = (np.linalg.norm(x, axis=2) for x in (a, b, c))
        det = np.einsum("ijk,ijk->ij", a, np.cross(b, c))
        den = la * lb * lc + np.einsum("ijk,ijk->ij", a, b) * lc + np.einsum("ijk,ijk->ij", b, c) * la \
            + np.einsum("ijk,ijk->ij", c, a) * lb
        w[s:s + step] = np.arctan2(det, den).sum(1) / (2 * np.pi)
    return w


def seg_hits(origins, direction, length, tm):
    """Moller-Trumbore: for each segment origin + t*direction, t in (0, length], does it hit tm?"""
    T = tm.triangles
    v0, e1, e2 = T[:, 0], T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]
    pvec = np.cross(direction, e2)
    det = np.einsum("ij,ij->i", e1, pvec)
    ok = np.abs(det) > 1e-12
    v0, e1, e2, pvec, det = v0[ok], e1[ok], e2[ok], pvec[ok], det[ok]
    inv = 1.0 / det
    hits = np.zeros(len(origins), bool)
    tmin = np.full(len(origins), np.inf)
    for s in range(0, len(origins), 64):
        O = origins[s:s + 64]
        tv = O[:, None, :] - v0[None]
        u = np.einsum("ijk,jk->ij", tv, pvec) * inv
        q = np.cross(tv, e1[None])
        v = np.einsum("k,ijk->ij", direction, q) * inv
        t = np.einsum("ijk,jk->ij", q, e2) * inv
        m = (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6) & (t <= length)
        hits[s:s + 64] = m.any(1)
        tt = np.where(m, t, np.inf).min(1)
        tmin[s:s + 64] = tt
    return hits, tmin


def ray_first(origins, dirs, tm, maxlen=50.0):
    """First hit distance along per-point directions (for wall thickness)."""
    T = tm.triangles
    v0, e1, e2 = T[:, 0], T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]
    res = np.full(len(origins), np.inf)
    for s in range(0, len(origins), 32):
        O, D = origins[s:s + 32], dirs[s:s + 32]
        pvec = np.cross(D[:, None, :], e2[None])
        det = np.einsum("jk,ijk->ij", e1, pvec)
        det = np.where(np.abs(det) < 1e-12, np.nan, det)
        inv = 1.0 / det
        tv = O[:, None, :] - v0[None]
        u = np.einsum("ijk,ijk->ij", tv, pvec) * inv
        q = np.cross(tv, e1[None])
        v = np.einsum("ik,ijk->ij", D, q) * inv
        t = np.einsum("ijk,jk->ij", q, e2) * inv
        m = (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0.02) & (t <= maxlen)
        res[s:s + 32] = np.where(m, t, np.inf).min(1)
    return res


def raster_top(tm, z, res=0.25, xy_bounds=None):
    """Rasterise the up-facing faces of tm at height z; return holes (enclosed uncovered regions)."""
    fn, c = tm.face_normals, tm.triangles_center
    tri = tm.triangles[(fn[:, 2] > 0.99) & (np.abs(c[:, 2] - z) < 0.01)]
    bb = tm.bounds
    xs = np.arange(bb[0, 0], bb[1, 0], res) + res / 2
    ys = np.arange(bb[0, 1], bb[1, 1], res) + res / 2
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    P = np.c_[X.ravel(), Y.ravel()]
    cov = np.zeros(len(P), bool)
    for tr in tri:
        mn, mx = tr[:, :2].min(0), tr[:, :2].max(0)
        idx = np.where((P[:, 0] >= mn[0]) & (P[:, 0] <= mx[0]) & (P[:, 1] >= mn[1]) & (P[:, 1] <= mx[1]))[0]
        if len(idx):
            cov[idx] |= Path(tr[:, :2]).contains_points(P[idx], radius=1e-9)
    cov = cov.reshape(X.shape)
    lab, nl = ndimage.label(~cov)
    holes = []
    for k in range(1, nl + 1):
        ii, jj = np.where(lab == k)
        if ii.min() == 0 or jj.min() == 0 or ii.max() == len(xs) - 1 or jj.max() == len(ys) - 1:
            continue
        holes.append(dict(x=float(xs[ii].mean()), y=float(ys[jj].mean()),
                          sx=float(xs[ii].max() - xs[ii].min() + res), sy=float(ys[jj].max() - ys[jj].min() + res)))
    return holes


# ---------------------------------------------------------------------------
def fixing_mask(name, pts, sp):
    """True where a cover point belongs to a rest / fixing feature (boss, pad, rib)."""
    m = np.zeros(len(pts), bool)
    if name.startswith("pod"):
        pc = sp["pod_cover"]
        side = pc["left" if name.endswith("left") else "right"]
        ztop = pc["rest_z"] + pc["skin_gap"] + 0.5
        for (x, y), r in [(xy, pc["boss"]["od"] / 2) for xy in side["screws"]] + \
                         [(xy, pc["rest_pad_od"] / 2) for xy in side["rest_pads"]]:
            m |= (np.hypot(pts[:, 0] - x, pts[:, 1] - y) <= r + 0.3) & (pts[:, 2] <= ztop + 0.3)
    else:
        ns = sp["nose_shroud"]
        s = 1 if name.endswith("left") else -1
        ztop = ns["rest_z"] + ns["skin_gap"] + 0.5
        for x, y in ns["rest_pads"]["xy"]:
            m |= (np.hypot(pts[:, 0] - x, pts[:, 1] - s * y) <= ns["rest_pads"]["od"] / 2 + 0.3) & (pts[:, 2] <= ztop + 0.3)
        r = ns["side_pads"]
        for x in r["x"]:
            m |= (np.abs(pts[:, 0] - x) <= r["length"] / 2 + 0.3) & (s * pts[:, 1] >= ns["nose_outer_y"] - 0.5) \
                & (s * pts[:, 1] <= ns["nose_outer_y"] + 12) & (pts[:, 2] >= r["z"][0] - 0.3) & (pts[:, 2] <= r["z"][1] + 0.3)
    return m


def allowed_contact(name):
    return {"lid_left", "lid_right", "pod_left", "pod_right"} if name.startswith("pod") else {"nose_left", "nose_right"}


def pair_distance(cname, cov, cpts, cfix, rname, rm, rtree, rpts, exempt):
    """Exact-refined min distance cover <-> ref mesh.  exempt: ignore cover fixing points.
    Returns (dist, cover_point, ref_point, overlap_depth)."""
    use = ~cfix if exempt else np.ones(len(cpts), bool)
    P = cpts[use]
    if len(P) == 0:
        return np.inf, None, None, 0.0
    d, _ = rtree.query(P, distance_upper_bound=25.0)
    if not np.isfinite(d).any():
        return float(d.min()), None, None, 0.0
    order = np.argsort(d)[:60]
    de, cpr = closest_on_tris(P[order], rm)
    j = int(np.argmin(de))
    best = (float(de[j]), P[order][j], cpr[j])
    # reverse direction: reference samples near the cover -> exact to cover triangles
    ctree = cKDTree(P)
    d2, _ = ctree.query(rpts, distance_upper_bound=25.0)
    if np.isfinite(d2).any():
        order2 = np.argsort(d2)[:60]
        de2, cpc = closest_on_tris(rpts[order2], cov)
        if exempt:   # discard hits that land on a fixing feature
            okm = ~fixing_mask(cname, cpc, SP)
            de2 = np.where(okm, de2, np.inf)
        k = int(np.argmin(de2))
        if de2[k] < best[0]:
            best = (float(de2[k]), cpc[k], rpts[order2][k])
    # overlap: points of either surface that lie inside the other solid
    depth = 0.0
    if best[0] < 1.0:
        near_c = P[d < 1.5][:3000]
        if len(near_c):
            w = winding(near_c, rm)
            ins = near_c[w > 0.75]
            if len(ins):
                dd, _ = closest_on_tris(ins[:200], rm)
                depth = max(depth, float(dd.max()))
        near_r = rpts[d2 < 1.5][:3000]
        if len(near_r):
            w = winding(near_r, cov)
            ins = near_r[w > 0.75]
            if len(ins):
                dd, _ = closest_on_tris(ins[:200], cov)
                depth = max(depth, float(dd.max()))
    return best[0], best[1], best[2], depth


def fmt(p):
    return "(" + ", ".join(f"{v:.1f}" for v in p) + ")" if p is not None else "-"


# ---------------------------------------------------------------------------
def main():
    global SP
    SP = yaml.safe_load(open(os.path.join(HERE, "params.yaml")))
    ref, info, D, bad = load_reference()
    add("0 reference decoding", not bad, f"{len(ref)} meshes, max |bbox err| checked",
        "decoded vertices match every stored bbox within 0.06 mm" if not bad else f"mismatch {bad}")
    manifest = json.load(open(os.path.join(OUT, "manifest.json")))
    mf = {p["name"]: p for p in manifest["parts"]}
    covers = {}
    for n in COVERS:
        tm = trimesh.load(os.path.join(OUT, f"{n}.stl"))
        covers[n] = tm
    cs = {}
    for n, tm in covers.items():
        p, fi = sample(tm, n=N_COVER)
        cs[n] = (p, fi, fixing_mask(n, p, SP))
    print("sampling reference ...")
    rs = {}
    for n, tm in ref.items():
        p, _ = sample(tm, density=REF_DENSITY)
        rs[n] = (p, cKDTree(p))

    # 1 bed fit --------------------------------------------------------------
    import itertools
    for n, tm in covers.items():
        ext = tm.extents
        fits = [perm for perm in itertools.permutations(range(3)) if all(ext[list(perm)] <= 248 + 1e-6)]
        add(f"1 bed fit {n}", bool(fits), " x ".join(f"{e:.2f}" for e in ext),
            f"fits 248^3 in {len(fits)}/6 permutations; largest margin {248 - ext.max():.2f} mm")

    # 2 clearance ------------------------------------------------------------
    detail.append("## 2. Clearance, 5 closest pairs per cover\n\nMethod: dense surface sampling "
                  f"({N_COVER} random samples + all vertices per cover, {REF_DENSITY:g} samples/mm^2 + vertices per "
                  "reference mesh, KD-trees), exact point-to-triangle refinement of the 60 closest candidates in both "
                  "directions; overlap by winding number of near-contact points. Fixing features (bosses, pads, ribs) "
                  "are excluded only against their allowed rest parts; their contact distance is listed separately.\n")
    pairs_all = {}
    for n, tm in covers.items():
        cpts, _, cfix = cs[n]
        allowed = allowed_contact(n)
        res = []
        overlaps = []
        contacts = []
        for rn, rm in ref.items():
            rpts, rtree = rs[rn]
            ex = rn in allowed
            d, pc, pr, depth = pair_distance(n, tm, cpts, cfix, rn, rm, rtree, rpts, exempt=ex)
            res.append((d, rn, pc, pr))
            if depth > 0.05:
                overlaps.append((rn, depth))
            if ex:
                dc, pcc, _, depthc = pair_distance(n, tm, cpts[cfix], np.zeros(cfix.sum(), bool), rn, rm, rtree,
                                                   rpts, exempt=False) if cfix.any() else (np.inf, None, None, 0)
                if np.isfinite(dc) and dc < CLEAR:
                    contacts.append((rn, dc, depthc, pcc))
                if depthc > 0.05:
                    overlaps.append((rn + " (at fixing)", depthc))
        # other covers (intersection with anything)
        for on, om in covers.items():
            if on == n:
                continue
            d2, _ = cKDTree(cs[on][0]).query(cpts, distance_upper_bound=5)
            if np.isfinite(d2).any() and d2.min() < 1.0:
                near = cpts[d2 < 1.0][:2000]
                w = winding(near, om)
                ins = near[w > 0.75]
                if len(ins):
                    dd, _ = closest_on_tris(ins[:200], om)
                    if dd.max() > 0.05:
                        overlaps.append((on, float(dd.max())))
        res.sort(key=lambda r: r[0])
        pairs_all[n] = res
        dmin = res[0][0]
        add(f"2 clearance {n}", dmin >= CLEAR and not overlaps,
            f"min {dmin:.3f} mm to {res[0][1]}",
            ("OVERLAP " + ", ".join(f"{a} {b:.2f} mm deep" for a, b in overlaps) + "; ") if overlaps else
            "no volume overlap; " + f"fixing contacts: " +
            (", ".join(f"{a} {b:.2f}" for a, b, _, _ in contacts) or "none"))
        detail.append(f"\n**{n}** (excluding fixing features vs {', '.join(sorted(allowed))})\n\n"
                      "| # | reference mesh | distance mm | cover point | ref point |\n|---|---|---|---|---|")
        for i, (d, rn, pc, pr) in enumerate(res[:5]):
            detail.append(f"| {i + 1} | {rn} | {d:.2f} | {fmt(pc)} | {fmt(pr)} |")
        if contacts:
            detail.append("\nFixing-feature contacts: " + "; ".join(
                f"{a} {b:.2f} mm (overlap {c:.2f}) at {fmt(p)}" for a, b, c, p in contacts))
        if overlaps:
            detail.append("\nOverlaps: " + "; ".join(f"{a} {b:.2f} mm" for a, b in overlaps))

    # 3 wheels ---------------------------------------------------------------
    for n in COVERS:
        if not n.startswith("pod"):
            continue
        side = "left" if n.endswith("left") else "right"
        tm = covers[n]
        cpts, _, cfix = cs[n]
        out = []
        for part in (f"wheel_tyre_{side}", f"wheel_hub_{side}"):
            rm = ref[part].copy()
            rm.apply_translation([0, 0, 10])
            rp, _ = sample(rm, density=REF_DENSITY)
            d, pc, pr, depth = pair_distance(n, tm, cpts, cfix, part, rm, cKDTree(rp), rp, exempt=False)
            out.append((part, d, pc, depth))
        dt = out[0][1]
        add(f"3 wheel +10 mm {n}", dt >= CLEAR and out[0][3] <= 0.05,
            f"tyre {dt:.2f} mm (hub {out[1][1]:.2f})", f"closest at cover {fmt(out[0][2])}")
        ay = np.abs(cpts[:, 1])
        sel = (ay >= 140) & (ay <= 190)
        r = np.hypot(cpts[sel, 0] - 0.0, cpts[sel, 2] - 40.0)
        k = int(np.argmin(r))
        need = 40 + 10 + CLEAR
        add(f"3 axle-line distance {n}", bool(r[k] >= need), f"{r[k]:.2f} mm",
            f"|y| 140..190; at {fmt(cpts[sel][k])}; needs >= {need:.0f} (tyre r 40 + 10 travel + 3)")

    # 4 vacuum lift-out --------------------------------------------------------
    for n in COVERS:
        p = cs[n][0]
        inside = (np.abs(p[:, 1]) < 75) & (p[:, 0] > -362) & (p[:, 0] < 58) & (p[:, 2] > 27)
        # distance to the box (0 inside)
        lo = np.array([-362, -75, 27]); hi = np.array([58, 75, 1e4])
        dbox = np.linalg.norm(np.maximum(0, np.maximum(lo - p, p - hi)), axis=1)
        add(f"4 vacuum lift-out box {n}", not inside.any(), f"{int(inside.sum())} pts inside; min gap {dbox.min():.2f} mm",
            "|y|<75, x -362..58, z>27")

    # 5 bumper travel ----------------------------------------------------------
    for n in COVERS:
        if not n.startswith("nose"):
            continue
        tm = covers[n]
        cpts, _, cfix = cs[n]
        best = (np.inf, None)
        for part in ("bumper_left", "bumper_right", "bumper_joining_plate"):
            rm = ref[part].copy()
            rm.apply_translation([-8, 0, 0])
            rp, _ = sample(rm, density=REF_DENSITY)
            d, pc, pr, depth = pair_distance(n, tm, cpts, cfix, part, rm, cKDTree(rp), rp, exempt=False)
            if d < best[0]:
                best = (d, part, pc, depth)
        add(f"5 bumper -8 mm X {n}", best[0] >= 1.0 and best[3] <= 0.05, f"{best[0]:.2f} mm to {best[1]}",
            f"at cover {fmt(best[2])}")

    # 6 sensors ----------------------------------------------------------------
    allp = np.vstack([cs[n][0] for n in COVERS])
    zmax = allp[:, 2].max()
    lb = ref["placeholder_lidar"].bounds
    lc = (lb[0] + lb[1]) / 2
    lr = max(lb[1, 0] - lb[0, 0], lb[1, 1] - lb[0, 1]) / 2 + 3
    dl = np.hypot(allp[:, 0] - lc[0], allp[:, 1] - lc[1]).min()
    add("6a LiDAR", zmax <= 160 and dl >= lr, f"max z {zmax:.1f}; {dl:.2f} mm from axis (r {lr:.1f})",
        f"footprint centre ({lc[0]:.1f}, {lc[1]:.1f}); scan plane ~{lb[0, 2] + 26:.0f}")

    rb = ref["placeholder_ld2410"].bounds
    t40 = math.tan(math.radians(40))
    worst = (np.inf, None)
    for n in COVERS:
        p = cs[n][0]
        s = (p[:, 0] >= 141) & (p[:, 0] <= 400)
        q = p[s]
        g = (q[:, 0] - 141) * t40
        m = np.max(np.c_[rb[0, 1] - g - q[:, 1], q[:, 1] - rb[1, 1] - g, rb[0, 2] - g - q[:, 2], q[:, 2] - rb[1, 2] - g], axis=1)
        m = m * math.cos(math.radians(40))       # distance normal to the cone face
        if len(m) and m.min() < worst[0]:
            worst = (float(m.min()), n, q[np.argmin(m)])
    if worst[2] is not None:
        zc_ = rb[1, 2] + (worst[2][0] - 141) * t40
        detail.append(f"\n## 6b. LD2410 cone\n\nWorst cover point {fmt(worst[2])} ({worst[1]}): the cone's upper face is at z "
                      f"{zc_:.2f} there, so the cover is {zc_ - worst[2][2]:.2f} mm inside vertically "
                      f"({-worst[0]:.2f} mm normal to the cone face). Cover front x max over |y| 20..100: "
                      f"{max(cs[n][0][(np.abs(cs[n][0][:, 1]) > 20) & (np.abs(cs[n][0][:, 1]) < 100), 0].max() for n in COVERS[2:]):.2f}.")
    add("6b LD2410 radar cone", worst[0] > 0, f"min margin {worst[0]:.2f} mm ({worst[1]})",
        f"frustum from the board face x 141, y {rb[0, 1]:.0f}..{rb[1, 1]:.0f}, z {rb[0, 2]:.0f}..{rb[1, 2]:.0f}, 40 deg; closest {fmt(worst[2])}")

    tz = D["params"]["bumper"]["tof_z"]
    t13 = math.tan(math.radians(13.5))
    worst = (np.inf, None, None)
    for y0 in (0.0, 96.0, -96.0):
        a = allp[:, 0] - 173.0
        r = np.hypot(allp[:, 1] - y0, allp[:, 2] - tz)
        s = (a > 0) & (a <= 300)
        if s.any():
            m = (r[s] - a[s] * t13) * math.cos(math.radians(13.5))
            if m.min() < worst[0]:
                worst = (float(m.min()), y0, allp[s][np.argmin(m)])
    add("6c bumper ToF cones", not (worst[0] < 0), "no cover ahead of x 173" if worst[1] is None else
        f"min margin {worst[0]:.2f} mm (y {worst[1]:.0f})", f"apex x 173, z {tz}, y 0/+-96, 13.5 deg, 300 mm; cover max x {allp[:, 0].max():.1f}")

    # speaker grille
    sb = ref["speaker_box"]
    th = math.radians(D["params"]["speaker_box"]["tilt_deg"])
    dirv = np.array([math.cos(th), 0, math.sin(th)])
    up = np.array([-math.sin(th), 0, math.cos(th)])
    fn = sb.face_normals
    front = (fn @ dirv > 0.98)
    off = np.median(sb.triangles_center[front] @ dirv)
    front &= np.abs(sb.triangles_center @ dirv - off) < 0.5
    tri = sb.triangles[front]
    res = 0.25
    yy = np.arange(tri[:, :, 1].min(), tri[:, :, 1].max(), res) + res / 2
    w_all = tri @ up
    ww = np.arange(w_all.min(), w_all.max(), res) + res / 2
    Y, W = np.meshgrid(yy, ww, indexing="ij")
    P2 = np.c_[Y.ravel(), W.ravel()]
    cov = np.zeros(len(P2), bool)
    for tr in tri:
        cov |= Path(np.c_[tr[:, 1], tr @ up]).contains_points(P2, radius=1e-9)
    cov = cov.reshape(Y.shape)
    lab, nl = ndimage.label(~cov)
    hole_pts = []
    for k in range(1, nl + 1):
        ii, jj = np.where(lab == k)
        if ii.min() == 0 or jj.min() == 0 or ii.max() == len(yy) - 1 or jj.max() == len(ww) - 1:
            continue
        if len(ii) * res * res > 6.0:          # skip the four ~3 mm screw holes in the rim
            hole_pts.append(np.c_[yy[ii], ww[jj]])
    hp = np.vstack(hole_pts)
    xz0 = off * dirv
    origins = np.c_[xz0[0] + hp[:, 1] * up[0], hp[:, 0], xz0[2] + hp[:, 1] * up[2]] + 0.05 * dirv
    hit_any, tfirst = np.zeros(len(origins), bool), np.full(len(origins), np.inf)
    for n in COVERS:
        h, t = seg_hits(origins, dirv, 30.0, covers[n])
        hit_any |= h
        tfirst = np.minimum(tfirst, t)
    gz = origins[:, 2]
    # clearance of the segments: sample along them against the cover samples
    segp = (origins[:, None, :] + np.linspace(0, 30, 61)[None, :, None] * dirv).reshape(-1, 3)
    dseg = min(cKDTree(cs[n][0]).query(segp)[0].min() for n in COVERS)
    add("6d speaker grille line of sight", not hit_any.any(),
        f"{int(hit_any.sum())}/{len(origins)} rays blocked; min clearance {dseg:.2f} mm",
        f"grille openings z {gz.min():.1f}..{gz.max():.1f} on the 15 deg front face; 30 mm rays +X 15 deg up")

    # camera
    cb = ref["camera_carrier"].bounds
    cc = (cb[0] + cb[1]) / 2
    tilt = math.radians(D["params"]["camera"]["tilt"])
    f = np.array([math.cos(tilt), 0, math.sin(tilt)])
    u = np.array([-math.sin(tilt), 0, math.cos(tilt)])
    l = np.array([0, 1, 0])
    q = allp - cc
    a, b, h = q @ f, q @ l, q @ u
    rr = np.linalg.norm(q, axis=1)
    s = (a > 0) & (rr <= 380)
    ang = np.maximum(np.degrees(np.arctan2(np.abs(b[s]), a[s])) - 46, np.degrees(np.arctan2(np.abs(h[s]), a[s])) - 33)
    inside = (ang < 0).sum()
    add("6e camera view", inside == 0, f"min angular margin {ang.min():.1f} deg",
        f"centre {fmt(cc)}, 40 deg up, 92 x 66 deg, 380 mm")

    mb = ref["mic_cover"].bounds
    s = (allp[:, 0] > mb[0, 0] - 15) & (allp[:, 0] < mb[1, 0] + 15) & (np.abs(allp[:, 1]) > 0) & \
        (allp[:, 1] > mb[0, 1] - 15) & (allp[:, 1] < mb[1, 1] + 15) & (allp[:, 2] > mb[1, 2]) & (allp[:, 2] < mb[1, 2] + 15)
    add("6f mic", not s.any(), f"{int(s.sum())} pts; cover max z {zmax:.1f} vs mic top {mb[1, 2]:.1f}", "")

    # 7 fasteners --------------------------------------------------------------
    grid_lines = []
    fast_ok = True
    used = {}
    for part in ("mic_post", "speaker_box", "stop_button_cup", "sensor_post"):
        used[part] = ref[part].bounds
    for side in ("left", "right"):
        holes = raster_top(ref[f"lid_{side}"], 105.0)
        ins = [h for h in holes if 3.8 <= h["sx"] <= 4.3 and 3.8 <= h["sy"] <= 4.3]
        clr = [h for h in holes if 3.3 <= h["sx"] <= 3.7 and 3.3 <= h["sy"] <= 3.7]
        xs = sorted({round(h["x"]) for h in ins}); ys = sorted({round(h["y"]) for h in ins})
        # bore depth at one insert: lowest up-facing face inside the hole
        lm = ref[f"lid_{side}"]
        h0 = ins[len(ins) // 2]
        c = lm.triangles_center
        inb = (np.hypot(c[:, 0] - h0["x"], c[:, 1] - h0["y"]) < 1.9) & (lm.face_normals[:, 2] > 0.99)
        bore_floor = c[inb, 2].min() if inb.any() else None
        grid_lines.append(f"lid_{side}: {len(ins)} insert bores (dia 4.0) at x {xs}, y {ys}; bore floor z "
                          f"{bore_floor}; {len(clr)} M3 clearance holes (lid hold-down) at "
                          + ", ".join(f"({h['x']:.1f}, {h['y']:.1f})" for h in clr))
        for x, y in SP["pod_cover"][side]["screws"]:
            dd = [math.hypot(h["x"] - x, h["y"] - y) for h in ins]
            k = int(np.argmin(dd))
            h = ins[k]
            occ = [pn for pn, bb in used.items() if bb[0, 0] <= h["x"] <= bb[1, 0] and bb[0, 1] <= h["y"] <= bb[1, 1]]
            ok = dd[k] <= 0.2 and not occ
            fast_ok &= ok
            grid_lines.append(f"  cover screw ({x}, {y}) -> nearest insert ({h['x']:.1f}, {h['y']:.1f}), offset "
                              f"{dd[k]:.2f} mm{'; under ' + ','.join(occ) if occ else ''} -> {'OK' if ok else 'MISALIGNED'}")
    old = os.path.join(OLD, "cad", "parts", "side_pods.py")
    old_note = ""
    if os.path.exists(old):
        src = open(old).read()
        old_note = ("14 Nov side_pods.py: _lid() is a plain box (lid_thickness 3) with only the hold-down clearance holes "
                    "(_lid_bolt_xy, 6 per lid); no top insert grid" if "grid" not in src else "14 Nov side_pods.py mentions a grid")
    offs = [l for l in grid_lines if "MISALIGNED" in l]
    add("7 fasteners into lid insert grid", fast_ok,
        f"{len(offs)}/8 screws off the grid (offset {max(float(l.split('offset ')[1].split(' mm')[0]) for l in offs) if offs else 0:.2f} mm)",
        "grid exists in the 16 Nov lid meshes (x -170+20k, y +-85/+-133); cover params assume x -171+20k")
    detail.append("\n## 7. Fasteners\n\n" + "\n".join("    " + l for l in grid_lines) + "\n\n" + old_note +
                  "\n\nLid notes (16 Nov): " + "; ".join(f"{n}: {info[n]['notes'].get('notes', '')}" for n in ("lid_left", "lid_right")))

    # 8 centre of mass ---------------------------------------------------------
    S = D["summary"]
    M0 = S["total_mass_kg_with_placeholders"]
    c0 = np.array(S["centre_of_mass_estimate"])
    cx = S["caster_x"]
    tot_m, mom = M0, M0 * c0
    lines = []
    for n in COVERS:
        mass = mf[n]["mass_g"] / 1000.0
        tm = covers[n]
        cen = tm.center_mass if tm.volume > 0 else tm.centroid
        vol_g = abs(tm.volume) / 1000 * SP["material"]["density_g_cm3"]
        lines.append(f"{n}: {mf[n]['mass_g']} g (STL volume gives {vol_g:.1f} g) at {fmt(cen)}")
        tot_m += mass
        mom = mom + mass * cen
    c1 = mom / tot_m
    sh0 = (0 - c0[0]) / (0 - cx) * 100
    sh1 = (0 - c1[0]) / (0 - cx) * 100
    add("8 caster share (limit 35 %)", sh1 <= 35.0, f"{sh0:.2f} % -> {sh1:.2f} %",
        f"CoM {fmt(c0)} -> {fmt(c1)}, mass {M0:.3f} -> {tot_m:.3f} kg")
    detail.append("\n## 8. Centre of mass\n\n" + "\n".join("- " + l for l in lines))

    # 9 printability -----------------------------------------------------------
    pl = []
    for n, tm in covers.items():
        fn = tm.face_normals
        zc = tm.triangles_center[:, 2]
        bed = zc < tm.bounds[0, 2] + 0.05
        over = (fn[:, 2] < -math.cos(math.radians(40))) & ~bed     # face more than 50 deg from vertical, facing down
        ov_area = tm.area_faces[over].sum()
        # downward faces that are rest / contact faces of fixing features (printed on support upright)
        fixc = fixing_mask(n, tm.triangles_center, SP) & over
        # knife edges: convex edges whose wedge (interior) angle is below 50 deg
        ang = tm.face_adjacency_angles
        conv = tm.face_adjacency_convex
        knife = conv & (ang > math.radians(130))
        ev = tm.vertices[tm.face_adjacency_edges[knife]]
        klen = np.linalg.norm(ev[:, 0] - ev[:, 1], axis=1)
        sharp = conv & (ang > math.radians(25))
        sv = tm.vertices[tm.face_adjacency_edges[sharp]]
        sp_ = (sv[:, :1, :] + np.linspace(0, 1, 12)[None, :, None] * (sv[:, 1:, :] - sv[:, :1, :])).reshape(-1, 3)
        # wall thickness: rays inward from random samples, ignoring samples within 1 mm of a sharp convex edge
        p, fi = trimesh.sample.sample_surface(tm, 2500, seed=3)
        nrm = tm.face_normals[fi]
        tdist = ray_first(p - 0.01 * nrm, -nrm, tm, 40.0)
        far = cKDTree(sp_).query(p)[0] > 1.0 if len(sp_) else np.ones(len(p), bool)
        fin = np.isfinite(tdist) & far
        thin = fin & (tdist < 1.6)
        kc = (ev.mean(1)) if len(ev) else np.zeros((0, 3))
        groups = []
        if len(kc):
            # crude clustering of knife-edge midpoints on a 10 mm grid
            keys = np.round(kc / 15.0).astype(int)
            uk, inv = np.unique(keys, axis=0, return_inverse=True)
            for g in range(len(uk)):
                sel = inv.ravel() == g
                groups.append((klen[sel].sum(), kc[sel].min(0), kc[sel].max(0)))
            groups.sort(key=lambda g: -g[0])
        pl.append(dict(n=n, ov=ov_area, fix=tm.area_faces[fixc].sum(), thin=int(thin.sum()), N=int(fin.sum()),
                       tmin=float(tdist[fin].min()) if fin.any() else np.nan, thin_pts=p[thin][:4], thin_t=tdist[thin][:4],
                       klen=float(klen.sum()), groups=groups[:6]))
        add(f"9 printability {n}", "NOTE",
            f"overhang >50 deg {ov_area / 100:.0f} cm^2; knife edges (<50 deg wedge) {klen.sum():.0f} mm; "
            f"walls <1.6 mm away from edges {thin.sum()}/{fin.sum()} samples",
            f"upright as installed: needs support under roof; fixing/rest faces on support: {tm.area_faces[fixc].sum() / 100:.1f} cm^2")
    detail.append("\n## 9. Printability samples (stated orientation: upright as installed)\n")
    for d_ in pl:
        detail.append(f"- **{d_['n']}**: downward faces >50 deg from vertical {d_['ov'] / 100:.0f} cm^2 (rest/fixing faces among them "
                      f"{d_['fix'] / 100:.1f} cm^2, so the contact faces print on support). Wall samples >1 mm from sharp edges: "
                      f"{d_['thin']}/{d_['N']} below 1.6 mm (min {d_['tmin']:.2f})" +
                      ("; e.g. " + ", ".join(f"{fmt(q)} {t:.2f}" for q, t in zip(d_['thin_pts'], d_['thin_t'])) if d_['thin'] else "") +
                      f". Knife edges (convex wedge < 50 deg) {d_['klen']:.0f} mm in total, largest clusters: " +
                      "; ".join(f"{L:.0f} mm in {fmt(a)}..{fmt(b)}" for L, a, b in d_['groups']))

    # report -------------------------------------------------------------------
    md = ["# Shell fit report (check_shell.py)", "",
          "Covers: cad/shell/out/*.stl (as installed). Reference: reference/model_2026-11-16.json. "
          "Generated by `python3 cad/shell/check_shell.py`.", "",
          "| check | result | number | note |", "|---|---|---|---|"]
    esc = lambda t: str(t).replace("|", "\\|")
    md += [f"| {esc(a)} | {esc(b)} | {esc(c)} | {esc(d)} |" for a, b, c, d in rows]
    md += [""] + detail
    with open(os.path.join(OUT, "fit-report.md"), "w") as fh:
        fh.write("\n".join(md) + "\n")
    nf = sum(r[1] == "FAIL" for r in rows)
    print(f"{nf} FAIL -> {os.path.join(OUT, 'fit-report.md')}")
    return 1 if nf else 0


if __name__ == "__main__":
    sys.exit(main())
