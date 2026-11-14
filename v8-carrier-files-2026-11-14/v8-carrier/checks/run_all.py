"""Run every fit check, print a table and exit non-zero on any failure.

    cd <repo> && python3 -m checks.run_all [--only name,name] [--json path]

Builds the CAD once (``checks.common.Context``), runs the checks in order,
prints one row per check (status, seconds, summary), then every failure and
warning, and writes the full results to ``checks/out/results.json`` (the
report in ``docs/fit-report.md`` is written from that file).
"""

from __future__ import annotations

import argparse
import importlib
import sys
import time
import traceback
from datetime import date

from checks.common import CHECK_OUT, FAIL, Context, CheckResult, write_json

CHECKS = ["bed_fit", "intersections", "hole_alignment", "cable_clearance", "centre_of_mass", "wall_thickness"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--only", help="comma-separated subset of checks")
    ap.add_argument("--json", default=str(CHECK_OUT / "results.json"), help="where to write the results")
    args = ap.parse_args(argv)
    names = args.only.split(",") if args.only else CHECKS
    t0 = time.time()
    ctx = Context()
    print(f"built {len(ctx.pieces)} print pieces and {len(ctx.assembled)} assembled bodies in {ctx.build_seconds:.1f} s")
    results: list[CheckResult] = []
    for name in names:
        mod = importlib.import_module(f"checks.{name}")
        t = time.time()
        try:
            r = mod.run(ctx)
        except Exception as exc:  # a crashed check is a failure, not a pass
            r = CheckResult(name, status=FAIL, summary=f"check crashed: {exc}")
            r.failures.append(traceback.format_exc())
        r.seconds = time.time() - t
        results.append(r)
        print(f"  {name:16s} {r.status:5s} {r.seconds:6.1f} s  {r.summary}")

    print()
    print(f"{'check':16s} {'status':6s} {'fails':>5s} {'warns':>5s}  summary")
    for r in results:
        print(f"{r.name:16s} {r.status:6s} {len(r.failures):5d} {len(r.warnings):5d}  {r.summary}")
    for r in results:
        for f in r.failures:
            print(f"FAIL  [{r.name}] {f}")
    for r in results:
        for w in r.warnings:
            print(f"WARN  [{r.name}] {w}")

    out = {
        "date": date.today().isoformat(),
        "params": "cad/params.yaml",
        "layout": {k: round(v, 2) for k, v in ctx.L.items()},
        "max_piece_mm": ctx.max_piece,
        "checks": [r.as_dict() for r in results],
        "seconds": round(time.time() - t0, 1),
    }
    write_json(type(CHECK_OUT)(args.json), out)
    n_fail = sum(1 for r in results if r.status == FAIL)
    print(f"\n{len(results)} checks, {n_fail} failed, {sum(1 for r in results if r.status == 'WARN')} with warnings; "
          f"results in {args.json} ({out['seconds']} s)")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
