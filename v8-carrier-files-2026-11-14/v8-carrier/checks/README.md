# Fit checks

Scripts the checker runs against the CAD (`cad/out/*.stl` and the parts built
in memory from `cad/params.yaml`): bed fit, intersections between assembled
parts (plus suspension bump/droop and bumper travel), M3 hole alignment across
every joint, cable clearance between the pods, centre of mass with 1.6 kg in
the slot, and a ray-cast wall-thickness sanity pass.

```
cd <repository root>
python3 -m checks.run_all            # all checks, table, exit 1 on any failure
python3 -m checks.run_all --only bed_fit,centre_of_mass
python3 -m checks.hole_alignment     # one check on its own
```

`run_all` builds the CAD once (`checks/common.py: Context`), prints one row
per check and every failure/warning, and writes `checks/out/results.json`
(the numbers behind `docs/fit-report.md`).  Nothing under `cad/` is written.

| Module | Checks |
| --- | --- |
| `bed_fit.py` | every STL fits `min(bed) - 2 * split_margin` in its intended print orientation; STL watertightness |
| `intersections.py` | pairwise OCCT boolean of every assembled body (print pieces and placeholders), limit 1 mm3; near-contacts between moving parts; wheel bump/droop and bumper-pressed cases; bumper end stop and spring preload |
| `hole_alignment.py` | insert / clearance / dowel / pivot features found from circular edges, matched across 17 joints within 0.2 mm; expected counts from params; insert depth |
| `cable_clearance.py` | 15 x 10 mm probe swept along the channel tube and up both pass-throughs; fasteners whose axis crosses the channel |
| `centre_of_mass.py` | PETG pieces plus placeholder masses: caster share 5-35 %, lateral under 10 mm; vacuum-removed case reported |
| `wall_thickness.py` | thickness at every STL triangle along its normal (numpy ray caster); under 2.5 mm fails unless in a zone params make thinner on purpose |
