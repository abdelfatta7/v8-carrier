# Fit report — 2026-11-10, run 2 (after the fix pass)

Second checker pass, on the CAD as rebuilt by `python3 -m cad.build_all` after
the design agent fixed every item on run 1's failure list and the coordinator
moved `wheels.axle_from_front` from 190 to 173 (electronics pod re-laid in its
rear compartment, swing-arm pivot 35 mm ahead of the axle, cliff windows
18 mm, bumper pins 54 mm with 3 mm spring preload, tray centreline bolts
re-spread).  21 print pieces, 21 STL + 21 STEP in `cad/out/`, `manifest.json`
current, `measurements.yaml` still empty.  `python3 -m checks.run_all`:
**6 checks, 0 failed, 3 with warnings, exit code 0** (16 s); numbers in
`checks/out/results.json`.  Frame: X forward, Y left, Z up, origin on the
floor under the drive-axle midpoint; axle at x = 0, caster contact now at
x = −212, nose rear face (tray front) at x = 58, bumper front at x = 173,
tray rear at x = −247.

| Check | Result | One line |
| --- | --- | --- |
| 1 bed fit | pass | 21/21 pieces fit the 248 mm cube in their intended orientation; tightest margin 3 mm (pods and lids, 245 mm); all 21 STLs watertight |
| 2 intersections | pass (2 warnings) | 0 of 88 pairs over 1 mm³, largest common volume 0.00 mm³; vacuum placeholder clear; bumper end stop and spring preload now work; only the 0.2 mm pin-in-sleeve fit is reported |
| 3 hole alignment | pass | 17 joints, 80 features matched at 0.000 mm offset, 0 unmatched, every insert 6 mm deep |
| 4 cable clearance | pass | 15 × 10 probe clear along the whole path, 0.00 mm³ intrusion, no fastener crosses the tube |
| 5 centre of mass | pass loaded, warn unloaded | loaded x = −34.6 mm, 16.3 % on the caster, lateral +5.3 mm; vacuum removed x = −5.7 mm, 2.7 % (under the 5 % limit but behind the axle) |
| 6 wall thickness | pass (20 deliberate-zone warnings) | 94 238 rays, 0 spots under 2.5 mm outside the zones params make thinner on purpose |

## Per-piece table

Size = STL extents (X × Y × Z, robot frame); mass = PETG at 1.27 g/cm³ from
the manifest.  Holes = fastener features matched / unmatched over every joint
the piece takes part in (a feature counts once per piece).  Thin = minimum
wall measured; everything under 2.5 mm is in a deliberate zone (8 mm boss /
4.0 mm hole walls, 1 mm bearing lips, Pi recess, cliff pockets, guide-pin
cross holes).

| Piece | Module | Size (mm) | Mass (g) | Bed fit | Intersection | Holes m/u | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| tray_front_left | floor_tray | 183 × 140 × 37 | 176 | pass (z up, 65 mm margin) | pass | 20 / 0 | centreline inserts at x = 45, 5, −60, −90; min wall 2.0 at the hold-down bosses |
| tray_rear_left | floor_tray | 122 × 140 × 16 | 110 | pass (z up) | pass | 10 / 0 | inserts at x = −150, −240; one caster screw hole; min 2.0 at bosses |
| tray_front_right | floor_tray | 183 × 140 × 37 | 176 | pass (z up) | pass | 20 / 0 | clearance side of the centreline; min 2.0 at bosses |
| tray_rear_right | floor_tray | 122 × 140 × 16 | 110 | pass (z up) | pass | 10 / 0 | min 2.0 at bosses |
| pod_left | side_pods | 245 × 62 × 75 | 289 | pass (z up, 3 mm) | pass | 17 / 0 | columns, partitions, bracket bosses, battery stops and rail all present; 6 lid inserts, 4 bracket inserts 6 mm deep, M5 pivot; min 2.0 at boss walls |
| pod_right | side_pods | 245 × 62 × 75 | 281 | pass (z up, 3 mm) | pass | 17 / 0 | Pi, DRV8871 standoffs and ESP32 rails present; min 2.0 at boss walls and the Pi-recess walls (by params) |
| lid_left | side_pods | 245 × 62 × 9 | 61 | pass (z up, 3 mm) | pass | 6 / 0 | 12 board standoffs underneath, 2.0 mm walls (convention) |
| lid_right | side_pods | 245 × 62 × 3 | 54 | pass (z up, 3 mm) | pass | 6 / 0 | min 3.0; USB slot 26 × 10 now 4 mm from the corner bolt |
| wheel_bracket_left | wheel_module | 86 × 12 × 65 | 25 | pass (y up) | pass | 6 / 0 | spring peg under the ledge instead of a pocket; min 3.33 |
| wheel_bracket_right | wheel_module | 86 × 12 × 65 | 25 | pass (y up) | pass | 6 / 0 | mirror |
| wheel_arm_left | wheel_module | 70 × 9 × 56 | 13 | pass (y up) | pass | 1 / 0 | 1.0 mm bearing lip (by design); post on the arm's pad at y = 146.5 |
| wheel_arm_right | wheel_module | 70 × 9 × 56 | 13 | pass (y up) | pass | 1 / 0 | mirror |
| wheel_hub (×2) | wheel_module | 70 × 25 × 70 | 48 | pass (y up) | pass | 0 / 0 | grub boss solid again, insert 6 mm deep; min 3.0 |
| wheel_tyre (×2) | wheel_module | 80 × 25 × 80 | 43 | pass (y up) | pass | 0 / 0 | min 4.99 |
| nose_left | nose | 85 × 140 × 134 | 334 | pass (y up) | pass | 14 / 0 | cap opening 0.5 mm clearance, cut straight to the floor (no sliver, no wedges); min 2.0 at boss walls |
| nose_right | nose | 85 × 140 × 134 | 390 | pass (y up) | pass | 11 / 0 | 1.0 mm lip behind the drive-end bearing seat (by design); mouth is a radial sector with tube ribs, plenum floor filled behind the cavity opening |
| brush_cap | nose | 64 × 6 × 66 | 21 | pass (y up) | pass | 3 / 0 | 1.0 mm lip behind the bearing seat (by design) |
| lidar_plate | nose | 76 × 76 × 3 | 21 | pass (z up) | pass | 4 / 0 | min 3.0 |
| bumper_left | bumper | 57 × 144 × 78 | 58 | pass (x up) | pass | 2 / 0 | 54 mm guide pin; 1.85 mm beside the M2 end-stop cross hole (6 mm pin, by params); cliff pocket walls 2.0 (by params) |
| bumper_right | bumper | 57 × 144 × 78 | 58 | pass (x up) | pass | 2 / 0 | mirror |
| bumper_joining_plate | bumper | 3 × 60 × 50 | 11 | pass (x up) | pass | 4 / 0 | min 3.0 |

Printed total 2.408 kg (hub and tyre twice; the pods grew 70 g each with
their interior features back) against `masses.printed_chassis_total`
= 1.6 kg.

## 1. Bed fit

Limit: min(bed) − 2 × split_margin = 256 − 8 = **248 mm** (`frame.max_piece`
252 is the looser design limit).  Every STL fits in its intended print
orientation (tray and pods Z up, nose halves and wheel parts Y up, bumper X
up).  Largest extent 245 mm (pod_left/right, lid_left/right along X, now
x −195..50): 3 mm margin to 248, 7 mm to 252.  The tray front pieces are
183 mm (x −125..58) and the rear pieces 122 mm (x −247..−125) after the axle
move.  STL extents agree with the manifest bounding boxes to < 0.01 mm.
**All 21 STLs are watertight** (run 1's four non-watertight meshes were caused
by the tangent cliff holes, now 2 mm clear of the 18 mm windows).

## 2. Intersections

33 bodies (21 pieces, the hub and tyre twice, 10 placeholders); 65 pairs come
within 0.5 mm by bounding box and were intersected with OCCT booleans, plus
23 pairs in the motion cases.  Positive control: pod_left lowered 2 mm into
tray_front_left gives 15 699 mm³, so the boolean sees overlaps.

* **Largest common volume 0.00 mm³** (every pair exactly 0; run 1's 0.498 mm³
  nose/cap sliver is gone).
* 44 pairs share faces with zero volume as intended (split halves, pods and
  lids on their seats, brackets on the pods, placeholders in their cavities).
* Placeholders in cavities: battery, Pi 5 (0.5 mm to the recessed walls),
  ESP32 (its top edge at z = 102 sits 0.3 mm inside the lid's USB slot edge,
  as intended), brush bar, brush motor, LiDAR, caster, drive motors — all
  0 mm³.  The **vacuum placeholder** (now x −262..58) touches the slot floor
  and the nose rear face and intersects nothing (6 neighbours); it overhangs
  the tray's rear edge (x = −247) by 15 mm as before.
* Motion cases, all 0 mm³: wheel bump/droop (arm, motor, hub and tyre
  rotated ±8.2° about the pivot at x = 35 so the axle moves ±5 mm) and the
  bumper pushed back 8 mm.  Designed near-contacts in motion: at full droop
  the arm tail passes 0.40 mm under the bracket's stop block (46, ±152, 52);
  at full bump the arm's spring pad passes 0.85 mm from the bracket's notch
  edge (2, ±146.5, 80).  Nominal gaps: bracket plate to arm 1.0 mm (the M5
  washer), guide pins 0.2 mm radial in their Ø6.4 sleeves (reported as the
  two warnings; both are the numbers in params).
* **Bumper mechanism (pass):** sleeve x 123..143; pin tip at x = 116 at rest;
  end-stop cross hole at x = 121 (`bumper.py`: 1 mm `stop_clearance` plus the
  hole radius behind the sleeve's rear face at 123), moving to 113 at full
  travel — never inside the bore.  Spring seats 22 mm apart at rest (nose
  pocket bottom x = 139, bumper boss pocket x = 161) against a 25 mm free
  length: **3 mm preload**.  Note: the check still estimates the cross hole
  as "4 mm from the tip" (x = 120) from run 1's bumper.py; the true position
  is 121 and the conclusion is the same.  Worth updating in `checks/` on the
  next edit.

## 3. Hole alignment

Features found from circular edges (4.0 insert, 3.4 clearance, 4.1 dowel
socket, 6.5 counterbore, 5.3 M5, 16 625ZZ seat), matched coaxially within
**0.2 mm**; every match is at 0.000 mm.  Expected counts and positions from
params; every insert hole is 6.0 mm deep and every socket 8.0 mm.

| Joint | Insert side → mating side | Matched | Fasteners |
| --- | --- | --- | --- |
| tray centreline, front halves | tray_front_left → tray_front_right | 4/4 bolts (x = 45, 5, −60, −90), 3/3 dowels (x = 25, −20, −110) | M3 × 12, heads on the right flange (y = −8) |
| tray centreline, rear halves | tray_rear_left → tray_rear_right | 2/2 bolts (x = −150, −240), 1/1 dowel (x = −185) | M3 × 12 |
| tray cross split, left / right | tray_front_* → tray_rear_* | 3/3 bolts (y = ±25, ±75, ±125), 2/2 dowels (±50, ±100) each | M3 × 12, heads on the rear flange (x = −133) |
| nose centreline | nose_left → nose_right | 3/3 bolts (x = 65, 78, 128 at z = 131), 2/2 dowels ((69, 12), (103, 131)) | M3 × 12 through the top rib |
| lidar plate to tower | nose_left + right → lidar_plate | 4/4 at (75, ±28), (131, ±28) | M3 × 8 |
| brush cap to housing end | nose_left → brush_cap | 3/3 at (81, 17), (108, 63.5), (135, 17), y = 128 | M3 × 10 |
| bumper halves via joining plate | bumper_left + right → plate | 4/4 at (±20, 37 / 73), x = 163 | M3 × 8 |
| nose to tray | nose_left + right → tray_front_left + right | 4/4 bolts + 4/4 counterbores at (±95, 37), (±125, 37), x = 58 (the 4 duct-flange inserts at (±34, 51 / 119) beside them are the collar's, later) | M3 × 16 forward, heads counterbored in the rim (x 50..53.5) |
| pod_left / pod_right to tray | tray_* → pod_* | 6/6 each at x = 43, −48, −145 and y = ±87, ±131 | M3 × 8 down through the pod floor |
| wheel bracket to pod, left / right | pod_* → wheel_bracket_* | 4/4 bolts at (−30, 36), (−30, 86), (33, 86), (44, 72) in 6 mm bosses + M5 pivot at (35, 40) each | M3 × 10, M5 × 30 |
| wheel arm pivot, left / right | bracket M5 hole → arm 625ZZ seat | 1/1 each | M5 × 30, nylock in the pod |
| lid_left / lid_right to pod | pod_* → lid_* | 6/6 each at (−188, ±85), (−188, ±133), (43, ±85), (43, ±133), (28.5, ±85), (−28.5, ±133) | M3 × 8 |

Total 80 matched, 0 unmatched.

## 4. Cable clearance

Path from params, unchanged by the axle move: tube interior x −42.5..−27.5,
z 13..23 (15 × 10), y −109..109 (218 mm) under the slot floor; pass-throughs
15 × 15 at (−35, ±88.5) through the tray floor (23..27) and the pod floor
(27..31).  The 15 × 10 × 218 probe along Y and the two 15 × 10 columns from
z = 13 to the pod floor top z = 31 give **0.00 mm³** against all 6 bodies
their boxes touch (tray_front_left/right, pod_left/right).  Sanity: a probe
grown by the 3 mm wall meets 13 407 mm³ of tray per side; a slab above each
pod floor is clear.  **No fastener crosses the tube**: the nearest
centreline bolt is now at x = −60 (tube outer faces at −45.5 / −24.5) and
the pin at x = −20 clears by 2.5 mm; the hold-down bosses at x = −48
(radius 4) clear the tube by 1.5 mm.

## 5. Centre of mass

Every piece at PETG 1.27 g/cm³ with its exact centre of mass (tyres counted
as PETG), placeholders at params masses; axle at x = 0, caster at x = −212.

| Case | Mass | COM x, y, z (mm) | Caster share | Lateral |
| --- | --- | --- | --- | --- |
| with vacuum (1.6 kg at x = −102, z = 137) | 5.323 kg | −34.6, +5.3, 79.2 | **16.3 %** (limit 5–35 %) → pass | +5.3 mm (limit 10) → pass |
| vacuum removed | 3.723 kg | **−5.7**, +7.6, 54.4 | **2.7 %** (under 5 %) | +7.6 mm |

Loaded, the carrier is comfortably inside the limits (run 1: 11.2 %).  With
the vacuum removed the centre of mass is now 5.7 mm behind the axle, so the
caster stays on the floor and the carrier no longer tips onto its nose (run
1: 6.5 mm ahead), but only 2.7 % of the unloaded weight (≈ 1.0 N) rests on
the caster, under the 5 % the loaded case is held to.  The sensitivity is
unchanged: each millimetre the axle moves forward moves both COMs 1 mm
rearward, so ≈ 5 mm more (`axle_from_front` ≈ 168) would reach 5 % unloaded
(the loaded share would rise to ≈ 18 %).  The lateral offset is toward +Y (battery side) in both
cases.  Printed mass 2.408 kg is 0.8 kg over the `printed_chassis_total`
budget.  Not weighed: fasteners, springs, bearings, belt, cables, switches,
ToF and cliff sensors, the three DRV8871, two bucks, fuse holder and INA226
(`placeholders.drv8871_mass` / `buck_mass` exist in params but have no
placeholder bodies).

## 6. Wall thickness

A ray from the centre of every STL triangle along its inward normal (94 238
rays, numpy caster, capped at 12 mm).  Under 2.5 mm fails unless inside a
zone params make thinner on purpose, each with its own floor.  **No
unexplained spot in any piece.**  The 20 deliberate groups reported as
warnings, with the measured value:

* 2.00 mm boss walls around 4.0 mm insert holes (8 mm boss convention):
  tray × 4 (hold-down bosses, 1194 / 506 / 950 / 382 samples), pod_left
  (521), pod_right (4559: bracket bosses, lid columns, DRV standoffs),
  lid_left (3024, the 12 board standoffs), nose_left (917), nose_right
  (336), bumper_left/right (806 each, joining-plate bosses).
* 2.00 mm pod walls recessed `pods.pi_recess` over the Pi, pod_right (724).
* 2.00 mm cliff pocket walls (`bumper.cliff_pocket.wall`), both bumpers
  (23 each).
* 1.85 mm guide pin beside its Ø2 end-stop cross hole ((6 − 2) / 2 = 2 mm
  nominal, less the pin's curvature), both bumpers (81 each).
* 1.00 mm lips behind 625ZZ seats: wheel_arm × 2 (144 each), nose_right
  drive-end wall (126), brush_cap (62).

Pieces with nothing under 2.5 mm: lid_right (3.0), wheel_bracket × 2 (3.33),
wheel_hub (3.0), wheel_tyre (4.99), lidar_plate (3.0), bumper_joining_plate
(3.0).  Run 1's 0.0 mm tangencies, 0.01 mm sliver, 0.09 mm pocket breakout,
0.6 mm slot wall, 1.13 mm hub boss, 1.5 mm duct-flange boss ends, 1.66 mm
front-wall notch, 1.75 mm zip-tie and vent ribs and the 2.0 mm notches are
all gone; the nose's tapered mouth and plenum edges are replaced by radial
mouth faces, tube ribs and a fill block.  Method limits as before: thickness
is taken along the surface normal only; this is a sanity pass, not a slicer.

## Failures to send back to the design agent

None.

## Warnings (for the design agent's judgement, not failures)

* Vacuum removed: 2.7 % of the weight on the caster (COM 5.7 mm behind the
  axle, limit 5 %); the carrier stands, but a light push on the nose lifts
  the caster.  ≈ 5 mm more axle-forward (`axle_from_front` ≈ 168) or
  ~18 kg·mm of mass moved rearward would reach 5 %.
* Printed mass 2.41 kg against the 1.6 kg `printed_chassis_total` budget.
* Guide pins run 0.2 mm radial in their sleeves (6.0 in 6.4, by params);
  designed motion clearances of 0.40 mm (droop stop) and 0.85 mm (spring
  pad in the bracket notch) at the ends of the suspension travel.
* The provisional vacuum box overhangs the tray's rear by 15 mm.
* `checks/intersections.py` still places the bumper end-stop hole 4 mm from
  the pin tip (x = 120) rather than reading `guide_pin.stop_clearance`
  (x = 121); same verdict, to be aligned on the next checks edit.

## Provisional inputs

Numbers the checks used that are not measured or not decided (all marked in
`params.yaml`):

* **Vacuum placeholder** 320 × 130 × 220 mm box, 1.6 kg, nose forward, front
  face on the duct flange (x = 58), sitting on the slot floor — every value
  provisional until `measurements.yaml` is filled; the collar, cradle rails
  and cheeks, rear latch and servo mast are not built, so the vacuum has no
  modelled support beyond the floor.
* **Slot width 150** (from `measurements.body_outline.width` + 2 × 3 clearance
  once measured); pod width 62 and the pod layouts follow from it.
* **Track width 333** — pod outer face plus bracket 5 + gap 1 + arm 6 + hub
  gap 2 + wheel 25/2; the wheels run outside the pods; `build_all` checks it
  against `wheels.track_width`.
* **`wheels.axle_from_front` 173** — set by the coordinator from run 1's
  centre-of-mass result; still "the owner may move it".  Body length 420.
* **Caster height 23** = ground clearance (CONFIRM against the Pololu 3/4"
  caster), mount holes 24 mm apart, Ø2.4 (CONFIRM), base Ø28; position 35 mm
  from the rear (x = −212).
* **Duct outlet z = 85, Ø60, bolt pitch 68** (the vacuum inlet decides).
* **Bought-part geometry marked CONFIRM:** LiDAR mount pitch 46 and M2.5
  holes; cliff hole spacing 24; ToF hole spacing 15; brush motor mount 17 /
  Ø7.5 boss; JGB37 gearbox boss Ø13; pulley width 15; breakout-board hole
  inset 3; battery pack 80 × 42 × 40.
* **Masses:** placeholder masses in `masses` / `placeholders`; no placeholders
  for the DRV8871s, bucks, fuse, INA226, sensors, switches, fasteners; tyres
  counted at PETG density.
* **Bed limit** taken as 248 mm (256 − 2 × 4); nothing is between 248 and 252.
