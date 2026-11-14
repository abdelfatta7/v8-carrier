"""Build every part module and export the print pieces.

    cd <repo> && python3 -m cad.build_all

Writes cad/out/<piece>.stl, cad/out/<piece>.step and cad/out/manifest.json.
The manifest lists, per piece: module, bounding box (mm, robot frame), volume
(mm3), estimated PETG mass (g), robot-frame position (bounding-box centre and
centre of mass), whether it fits the bed, print count and notes.  It also
lists the provisional placeholder bodies (not exported) and an estimated
centre of mass of the whole assembly.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from build123d import CenterOf, export_step, export_stl

from cad import assembly
from cad.common import ROOT, bbox, fits_bed, layout, load_measurements, load_params, mass_g, max_piece

OUT = ROOT / "out"
STL_TOLERANCE = 0.1
STL_ANGULAR = 0.2


def _any_value(tree) -> bool:
    """True when measurements.yaml holds at least one non-null leaf."""
    if isinstance(tree, dict):
        return any(_any_value(v) for v in tree.values())
    return tree is not None


def piece_notes(module: str, piece: str) -> dict:
    mod = sys.modules[f"cad.parts.{module}"]
    notes = getattr(mod, "NOTES", {})
    return notes.get(piece, "")


def main() -> int:
    t0 = time.time()
    p = load_params()
    measurements = load_measurements()  # unused until the vacuum is measured
    L = layout(p)
    OUT.mkdir(parents=True, exist_ok=True)

    warnings = []
    if abs(L["track_width"] - p["wheels"]["track_width"]) > 0.01:
        warnings.append(
            f"params.wheels.track_width={p['wheels']['track_width']} but the wheel-module stack gives {L['track_width']}"
        )

    pieces = assembly.build_pieces(p)
    manifest = {
        "frame": p["frame"]["description"],
        "units": "mm, mm3, g, kg",
        "petg_density_g_cm3": p["materials"]["petg_density"],
        "max_piece_mm": max_piece(p),
        "layout": {k: round(v, 2) for k, v in L.items()},
        "pieces": {},
        "placeholders": {},
        "warnings": warnings,
    }
    failures = []
    total_mass = 0.0
    moment = [0.0, 0.0, 0.0]
    for name, (module, part) in pieces.items():
        try:
            stl = OUT / f"{name}.stl"
            step = OUT / f"{name}.step"
            export_stl(part, str(stl), tolerance=STL_TOLERANCE, angular_tolerance=STL_ANGULAR)
            export_step(part, str(step))
        except Exception as exc:  # keep going so the manifest shows what failed
            failures.append(f"{name}: {exc}")
            continue
        bb = bbox(part)
        com = part.center(CenterOf.MASS)
        m = mass_g(part, p)
        count = 2 if name in assembly.MIRRORED_TWICE else 1
        entry = {
            "module": module,
            "bbox_mm": bb,
            "volume_mm3": round(part.volume, 1),
            "mass_petg_g": round(m, 1),
            "position": {
                "bbox_centre": [round((a + b) / 2, 2) for a, b in zip(bb["min"], bb["max"])],
                "centre_of_mass": [round(com.X, 2), round(com.Y, 2), round(com.Z, 2)],
            },
            "fits_bed": fits_bed(part, p),
            "solids": len(part.solids()),
            "print_count": count,
            "notes": piece_notes(module, name),
            "files": [stl.name, step.name],
        }
        if count == 2:
            entry["notes"] = (entry["notes"] + "; " if entry["notes"] else "") + "print 2: the right-hand copy is this piece mirrored across the centreline"
        if not entry["fits_bed"]:
            warnings.append(f"{name} does not fit the bed: {bb['size']}")
        if entry["solids"] != 1:
            warnings.append(f"{name} has {entry['solids']} solids")
        manifest["pieces"][name] = entry
        # mass accounting (both copies of twice-printed pieces)
        for k in range(count):
            c = (com.X, com.Y if k == 0 else -com.Y, com.Z)
            total_mass += m / 1000
            for i in range(3):
                moment[i] += m / 1000 * c[i]

    ph = assembly.placeholders(p)
    masses = assembly.placeholder_masses(p)
    for name, part in ph.items():
        bb = bbox(part)
        c = part.center(CenterOf.BOUNDING_BOX)
        entry = {
            "provisional": True,
            "mass_kg": masses[name],
            "bbox_mm": bb,
            "centre": [round(c.X, 2), round(c.Y, 2), round(c.Z, 2)],
        }
        if name == "placeholder_vacuum":
            entry["note"] = "PROVISIONAL box %s mm, nose forward, front face on the duct flange, on the slot floor" % (
                [p["placeholders"]["vacuum"][k] for k in ("length", "width", "height")]
            )
        manifest["placeholders"][name] = entry
        total_mass += masses[name]
        for i, v in enumerate((c.X, c.Y, c.Z)):
            moment[i] += masses[name] * v

    manifest["summary"] = {
        "printed_mass_kg": round(sum(e["mass_petg_g"] * e["print_count"] for e in manifest["pieces"].values()) / 1000, 3),
        "total_mass_kg_with_placeholders": round(total_mass, 3),
        "centre_of_mass_estimate": [round(v / total_mass, 1) for v in moment],
        "axle_x": 0.0,
        "caster_x": L["caster_x"],
        "measurements_filled": _any_value(measurements),
        "failures": failures,
        "build_seconds": round(time.time() - t0, 1),
    }
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2))

    print(f"{'piece':24s} {'module':13s} {'size (mm)':24s} {'mass g':>7s}  fits")
    for name, e in manifest["pieces"].items():
        size = "x".join(f"{v:.0f}" for v in e["bbox_mm"]["size"])
        print(f"{name:24s} {e['module']:13s} {size:24s} {e['mass_petg_g']:7.1f}  {'yes' if e['fits_bed'] else 'NO'}")
    s = manifest["summary"]
    print(f"printed mass {s['printed_mass_kg']} kg; with placeholders {s['total_mass_kg_with_placeholders']} kg; "
          f"COM estimate {s['centre_of_mass_estimate']} (axle at x=0, caster at x={L['caster_x']})")
    for w in warnings:
        print("WARNING:", w)
    for f in failures:
        print("FAILED:", f)
    print(f"wrote {OUT / 'manifest.json'} in {s['build_seconds']} s")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
