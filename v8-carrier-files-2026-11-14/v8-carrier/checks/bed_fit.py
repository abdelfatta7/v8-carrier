"""Check 1: every exported print piece fits the printer bed.

Loads each ``cad/out/<piece>.stl`` with trimesh, measures its axis-aligned
extents and checks that, in at least one axis-aligned orientation, every
extent is at or under ``min(printer.bed) - 2 * printer.split_margin``
(248 mm here; ``frame.max_piece`` = 252 is the looser design limit and is
reported too).  The bed is a cube, so every orientation fits or none does;
the reported orientation is the one the part module says to print in
(``INTENDED_UP``: which robot-frame axis points up on the bed), falling
back to the orientation with the lowest height.

Also reports export sanity: STL triangle count, watertightness, and the
difference between the STL extents and the manifest's bounding box.
"""

from __future__ import annotations

import itertools
import sys
import time

import numpy as np
import trimesh

from checks.common import FAIL, PASS, Context, CheckResult, fmt

NAME = "bed_fit"

# robot-frame axis that points up on the bed, from the module docstrings
INTENDED_UP = {
    "tray_front_left": "z", "tray_front_right": "z", "tray_rear_left": "z", "tray_rear_right": "z",  # slot floor on the bed (upside down)
    "pod_left": "z", "pod_right": "z", "lid_left": "z", "lid_right": "z",  # open side up, lids flat
    "wheel_bracket_left": "y", "wheel_bracket_right": "y", "wheel_arm_left": "y", "wheel_arm_right": "y",  # plate on the bed
    "wheel_hub": "y", "wheel_tyre": "y",  # open side up / flat
    "nose_left": "y", "nose_right": "y",  # on the outer face, split face up
    "brush_cap": "y", "lidar_plate": "z",
    "bumper_left": "x", "bumper_right": "x", "bumper_joining_plate": "x",  # front plate on the bed
}


def orientations(ext):
    """All axis permutations as (bed_x, bed_y, bed_z) extents with the robot axis that is up."""
    for perm in itertools.permutations(range(3)):
        yield perm, (ext[perm[0]], ext[perm[1]], ext[perm[2]])


def run(ctx: Context) -> CheckResult:
    t0 = time.time()
    res = CheckResult(NAME)
    p = ctx.p["printer"]
    limit = ctx.max_piece
    design_limit = ctx.p["frame"]["max_piece"]
    bed = (p["bed_x"], p["bed_y"], p["bed_z"])
    rows = []
    pieces = list(ctx.manifest.get("pieces", {})) or ctx.print_pieces
    for piece in pieces:
        path = ctx.stl_path(piece)
        if not path.exists():
            res.fail(f"{piece}: {path} is missing")
            continue
        mesh = trimesh.load(path, force="mesh")
        ext = mesh.extents
        up = INTENDED_UP.get(piece, "z")
        up_i = "xyz".index(up)
        # intended orientation: the 'up' axis is the bed Z, the other two keep their order
        foot = [ext[i] for i in range(3) if i != up_i]
        intended = (foot[0], foot[1], ext[up_i])
        fits_intended = all(v <= limit + 1e-6 for v in intended)
        best = min(orientations(ext), key=lambda o: (max(o[1]) > limit, o[1][2]))
        fits_any = max(ext) <= limit + 1e-6
        if fits_intended:
            chosen, how = intended, f"as designed ({up} up)"
        else:
            chosen, how = best[1], f"{'xyz'[best[0][2]]} up (intended {up} up does not fit)"
        man = ctx.manifest.get("pieces", {}).get(piece, {})
        man_size = man.get("bbox_mm", {}).get("size")
        delta = max(abs(a - b) for a, b in zip(ext, man_size)) if man_size else None
        row = {
            "piece": piece,
            "extents_mm": [round(float(v), 2) for v in ext],
            "orientation": how,
            "bed_xyz_mm": [round(float(v), 2) for v in chosen],
            "fits": bool(fits_any),
            "fits_design_limit": bool(max(ext) <= design_limit + 1e-6),
            "margin_mm": round(float(limit - max(ext)), 2),
            "triangles": int(len(mesh.faces)),
            "watertight": bool(mesh.is_watertight),
            "stl_vs_manifest_mm": round(float(delta), 3) if delta is not None else None,
        }
        rows.append(row)
        if not fits_any:
            res.fail(f"{piece}: extents {np.round(ext, 1).tolist()} exceed {limit} mm in every orientation")
        if not row["watertight"]:
            res.warn(f"{piece}: exported STL is not watertight (trimesh), slicer repair may be needed")
        if delta is not None and delta > 0.3:
            res.warn(f"{piece}: STL extents differ from the manifest bbox by {delta:.2f} mm")
    res.data = {"limit_mm": limit, "design_limit_mm": design_limit, "bed_mm": list(bed), "rows": rows}
    n_fit = sum(r["fits"] for r in rows)
    res.summary = f"{n_fit}/{len(rows)} pieces fit the {limit} mm cube (bed {bed[0]} - 2 x {p['split_margin']}); tightest margin {min(r['margin_mm'] for r in rows):.0f} mm"
    res.details = [
        f"{r['piece']:24s} {fmt(r['extents_mm'][0], 0):>4s} x {fmt(r['extents_mm'][1], 0):>4s} x {fmt(r['extents_mm'][2], 0):>4s}  "
        f"{'fits' if r['fits'] else 'NO'}  margin {r['margin_mm']:5.0f}  {r['orientation']}  "
        f"tris {r['triangles']}{'' if r['watertight'] else '  NOT watertight'}"
        for r in rows
    ]
    res.seconds = time.time() - t0
    return res


if __name__ == "__main__":
    r = run(Context(build=False))
    print(r.summary)
    print("\n".join(r.details))
    for f in r.failures:
        print("FAIL:", f)
    for w in r.warnings:
        print("WARN:", w)
    sys.exit(0 if r.status != FAIL else 1)
