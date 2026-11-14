"""Check 5: centre of mass between the drive axle and the caster.

Every print piece in the assembly (both instances of the twice-printed hub
and tyre) is weighed at PETG density (``materials.petg_density``; the TPU
tyres are counted at PETG density too, as asked) with its exact centre of
mass; every placeholder is weighed at its ``params`` mass (``cad.assembly.
placeholder_masses``) at its own centre of mass (the vacuum 1.6 kg in the
slot).  The combined centre of mass must lie between the axle (X = 0) and
the caster contact (``layout.caster_x``), with the share of the weight on
the caster between ``CASTER_MIN`` and ``CASTER_MAX`` and the lateral offset
|Y| under ``LATERAL_MAX``.  The same numbers are reported with the vacuum
removed (informational: a warning when that case leaves the limits).

Not weighed (no placeholder in the assembly): fasteners, springs, bearings,
belt, cables, the bumper switches, ToF and cliff sensors, the DRV8871s,
bucks, fuse holder and INA226.
"""

from __future__ import annotations

import sys
import time

from build123d import CenterOf

from checks.common import FAIL, Context, CheckResult

NAME = "centre_of_mass"
CASTER_MIN, CASTER_MAX = 0.05, 0.35  # share of the weight on the caster
LATERAL_MAX = 10.0  # mm


def _combine(items):
    m = sum(w for w, _ in items)
    c = [sum(w * pt[i] for w, pt in items) / m for i in range(3)]
    return m, c


def _evaluate(ctx: Context, items, label: str, res: CheckResult, enforce: bool) -> dict:
    m, c = _combine(items)
    caster_x = ctx.L["caster_x"]
    frac = (0.0 - c[0]) / (0.0 - caster_x)  # 0 at the axle, 1 at the caster
    out = {"case": label, "mass_kg": round(m, 3), "com_mm": [round(v, 1) for v in c],
           "caster_share": round(frac, 3), "wheel_share": round(1 - frac, 3), "lateral_mm": round(c[1], 1)}
    ok_x = CASTER_MIN <= frac <= CASTER_MAX
    ok_y = abs(c[1]) < LATERAL_MAX
    out["within_limits"] = bool(ok_x and ok_y)
    msg = (f"{label}: mass {m:.3f} kg, COM x={c[0]:.1f} y={c[1]:.1f} z={c[2]:.1f} mm; caster carries {100 * frac:.1f} % "
           f"(limits {100 * CASTER_MIN:.0f}-{100 * CASTER_MAX:.0f} %), lateral offset {c[1]:+.1f} mm (limit {LATERAL_MAX})")
    if not ok_x:
        where = "ahead of the axle (the carrier tips onto its nose)" if frac < 0 else ("too close to the axle" if frac < CASTER_MIN else "too far back")
        (res.fail if enforce else res.warn)(f"{msg}: centre of mass {where}")
    if not ok_y:
        (res.fail if enforce else res.warn)(f"{msg}: lateral offset over {LATERAL_MAX} mm")
    res.details.append(msg)
    return out


def run(ctx: Context) -> CheckResult:
    t0 = time.time()
    res = CheckResult(NAME)
    rho = ctx.p["materials"]["petg_density"]  # g/cm3
    items, per_body = [], []
    for name, part in ctx.assembled.items():
        c = part.center(CenterOf.MASS)
        pt = (c.X, c.Y, c.Z)
        if name.startswith("placeholder_"):
            w = ctx.placeholder_masses[name]
        else:
            w = part.volume / 1000.0 * rho / 1000.0  # kg
        items.append((w, pt))
        per_body.append({"body": name, "mass_kg": round(w, 4), "com_mm": [round(v, 1) for v in pt]})
    printed = sum(w for (w, _), b in zip(items, per_body) if not b["body"].startswith("placeholder_"))
    loaded = _evaluate(ctx, items, "with vacuum (1.6 kg)", res, enforce=True)
    unloaded = _evaluate(ctx, [(w, pt) for (w, pt), b in zip(items, per_body) if b["body"] != "placeholder_vacuum"],
                         "vacuum removed", res, enforce=False)
    budget = ctx.p["masses"]["printed_chassis_total"]
    if printed > budget:
        res.warn(f"printed mass {printed:.2f} kg exceeds masses.printed_chassis_total = {budget} kg (note only)")
    res.data = {"axle_x": 0.0, "caster_x": ctx.L["caster_x"], "petg_density": rho, "printed_mass_kg": round(printed, 3),
                "loaded": loaded, "unloaded": unloaded, "bodies": per_body,
                "limits": {"caster_share": [CASTER_MIN, CASTER_MAX], "lateral_mm": LATERAL_MAX}}
    res.summary = (f"loaded: COM x={loaded['com_mm'][0]} mm, caster {100 * loaded['caster_share']:.1f} %, lateral {loaded['lateral_mm']:+.1f} mm; "
                   f"vacuum removed: x={unloaded['com_mm'][0]} mm, caster {100 * unloaded['caster_share']:.1f} %")
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
