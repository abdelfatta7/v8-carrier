# CAD

Parametric CAD in [build123d](https://build123d.readthedocs.io/), one Python
module per part under `parts/`.  Every dimension comes from `params.yaml`
(design parameters, decided and provisional) and, once filled, from
`measurements.yaml` (Stage 0 vacuum measurements).  Do not hard-code a number
that `params.yaml` holds; add a parameter with a short comment instead.

## Robot frame

All parts are modelled in place, in one frame:

* **X** forward, toward the nose and the bumper
* **Y** left (the battery pod is at +Y, the electronics pod at -Y)
* **Z** up
* **origin** on the floor plane, directly below the midpoint of the
  drive-wheel axle (so the axle is at x = 0, z = wheel radius)

`common.layout(p)` derives the key coordinates from the parameters (body front
and rear, nose faces, pod walls, floor levels, wheel centre, caster position).
Parts that exist on both sides are modelled on the left (y > 0) and mirrored
with `common.mirror_y`.

## Building

```
cd <repository root>
python3 -m cad.build_all
```

Headless.  Exports `out/<piece>.stl` (tolerance 0.1 mm, angular 0.2 rad) and
`out/<piece>.step` for every print piece and writes `out/manifest.json`
(module, bounding box, volume, estimated PETG mass, robot-frame position,
bed fit, print count and notes per piece; the provisional placeholders; an
estimated centre of mass).  Exit code 0 when every piece exported.

`assembly.assembled(p)` returns every piece plus placeholder bodies for the
bought parts and the vacuum (keys `placeholder_*`, provisional sizes from
`params.placeholders`) for the checker's intersection and centre-of-mass
tests.  Placeholders are not exported.

## Modules and pieces

| Module | Pieces | What |
| --- | --- | --- |
| `parts/floor_tray.py` | `tray_front_left/right`, `tray_rear_left/right` | floor between nose and rear: slot floor with rim, split flanges with M3 inserts and 4 mm dowel sockets, caster mount, cable channel tube under the slot floor with pass-throughs into the pods, pod hold-down inserts, nose rim, motor wells, cliff windows |
| `parts/side_pods.py` | `pod_left`, `pod_right`, `lid_left`, `lid_right` | battery pod (left) and electronics pod (right) with wheel boxes around the drive motors, bracket inserts and pivot hole on the outer walls, lids on M3 inserts |
| `parts/wheel_module.py` | `wheel_bracket_left/right`, `wheel_arm_left/right`, `wheel_hub`, `wheel_tyre` | swing-arm sprung drive mount: bracket on the pod, trailing arm on a 625ZZ, JGB37-520 bolted to the arm, spring on the arm's post under a bracket ledge; hub and tyre are printed twice |
| `parts/nose.py` | `nose_left`, `nose_right`, `brush_cap`, `lidar_plate` | brush-roll housing with bearing seats and removable cap, GT2 belt bay and brush motor mount, plenum and round duct outlet with flange inserts, LiDAR tower |
| `parts/bumper.py` | `bumper_left`, `bumper_right`, `bumper_joining_plate` | sprung front shell on two guide pins, switch pads, ToF windows, corner cliff pockets |

Not built yet: the dock contacts (rear plate with charging pads) and, blocked
on measurements, the collar, cradle rails and cheeks, rear latch and servo
mast.

## Conventions

* Each part module exposes `build(p) -> dict[str, Part]` (piece name to a
  build123d `Part` in the robot frame) and an optional `NOTES` dict with a
  line of assembly guidance per piece that goes into the manifest.
* Every print piece fits in 252 x 252 x 252 mm (and inside the bed minus
  `printer.split_margin`).  Bigger bodies are split with
  `common.split_centreline` (plane y = 0) or `common.split_plane_x`, which
  drill M3 insert / clearance holes and dowel sockets through flange material
  that the part module models on both sides of the cut.
* Fasteners: M3 heat-set inserts (4.0 x 6 holes) in 8 mm bosses, 3.4 mm
  clearance holes, 4 mm dowels in 4.1 mm sockets, M2 / M2.5 self-tap holes for
  small boards.  Helpers in `common.py`.
* The robot-frame Y of the wheel centre follows from the pod's outer face plus
  the bracket / arm / hub stack; `params.wheels.track_width` is kept equal to
  it and `build_all` warns if they disagree.
