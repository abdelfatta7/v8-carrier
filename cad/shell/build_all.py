#!/usr/bin/env python3
"""Build every shell part and write STEP + STL + manifest.json to cad/shell/out/.

Run from the repo root:  python3 cad/shell/build_all.py
Parts are exported in the robot frame (as installed); README.md gives the
print orientation.
"""
from __future__ import annotations

import itertools
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from build123d import export_step, export_stl  # noqa: E402

import nose_shroud_left  # noqa: E402
import nose_shroud_right  # noqa: E402
import pod_cover_left  # noqa: E402
import pod_cover_right  # noqa: E402
from shell_common import load_params  # noqa: E402

PARTS = {
    "pod_cover_left": pod_cover_left,
    "pod_cover_right": pod_cover_right,
    "nose_shroud_left": nose_shroud_left,
    "nose_shroud_right": nose_shroud_right,
}


def fits_bed(size, limit):
    """Axis-aligned fit in some 90-degree orientation (a box is symmetric, so
    sorted extents against a cube are enough)."""
    return all(s <= limit + 1e-6 for s in sorted(size))


def main():
    p = load_params()
    out = os.path.join(HERE, "out")
    os.makedirs(out, exist_ok=True)
    limit = p["bed"]["size"] - 2 * p["bed"]["margin"]
    rho = p["material"]["density_g_cm3"]
    manifest, total, ok = [], 0.0, True
    for name, mod in PARTS.items():
        part = mod.build(p)
        if not part.is_valid:
            print(f"WARNING: {name} is not a valid solid")
            ok = False
        bb = part.bounding_box()
        size = [bb.size.X, bb.size.Y, bb.size.Z]
        vol = sum(s.volume for s in part.solids())
        mass = vol / 1000.0 * rho
        total += mass
        fit = fits_bed(size, limit)
        if bb.max.Z > p["limits"]["max_z"] + 1e-6:
            print(f"WARNING: {name} rises to z {bb.max.Z:.1f} > {p['limits']['max_z']}")
            ok = False
        if not fit:
            ok = False
        export_step(part, os.path.join(out, f"{name}.step"))
        export_stl(part, os.path.join(out, f"{name}.stl"), tolerance=0.02, angular_tolerance=0.15)
        manifest.append({
            "name": name,
            "bbox": [[round(bb.min.X, 2), round(bb.min.Y, 2), round(bb.min.Z, 2)],
                     [round(bb.max.X, 2), round(bb.max.Y, 2), round(bb.max.Z, 2)]],
            "size": [round(s, 2) for s in size],
            "volume_cm3": round(vol / 1000.0, 2),
            "mass_g": round(mass, 1),
            "material": p["material"]["name"],
            "solids": len(part.solids()),
            "fits_bed": fit,
            "notes": mod.NOTES,
        })
        print(f"{name:20s} size {size[0]:6.1f} x {size[1]:6.1f} x {size[2]:6.1f}  "
              f"{mass:6.1f} g  fits_bed={fit}")
    with open(os.path.join(out, "manifest.json"), "w") as f:
        json.dump({"frame": "robot frame (mm), X forward, Y left, Z up; parts exported as installed",
                   "reference": "reference/model_2026-11-16.json",
                   "bed_limit_mm": limit, "density_g_cm3": rho,
                   "total_mass_g": round(total, 1), "parts": manifest}, f, indent=2)
    print(f"total {total:.1f} g -> {out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
