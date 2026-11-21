# Cuby exterior shell (bolt-on covers)

The covers go over the 16 Nov assembly (`reference/model_2026-11-16.json`). Nothing inside the robot changes:
the covers sit on the lid tops and the nose top, and they screw into the lids' existing M3 insert grid.
Every dimension is in `params.yaml`. Values marked PROVISIONAL depend on the open body-length question
(420 vs 522 mm); only the covers' rear ends (behind x -195) depend on it.

    python3 cad/shell/build_all.py      # from the repo root; writes out/*.step, out/*.stl, out/manifest.json

The parts are exported in the robot frame, as installed. Material is PETG with a 2.0 mm wall.

| part | what it is | size (mm) | mass |
|---|---|---|---|
| pod_cover_left | Cover over the left pod and lid, with a fender over the top half of the left wheel. Cut-outs (+3.4 mm) for the mic post and speaker box. | 247.7 x 108 x 81 | 82 g |
| pod_cover_right | Mirror body over the right pod and lid. Cut-outs for the stop button, the sensor post and the ESP32 USB slot. Vent slots over the Pi, and skirt slots in front of the pod's outer-wall vents. | 247.7 x 108 x 81 | 85 g |
| nose_shroud_left / _right | Rounded cap over the nose top and its outer sides, split on the centreline. Open around the LiDAR plate (+3.5 mm). The nose front face stays uncovered below z 125 (radar window, bumper). | 110.5 x 148 x 41 each | 42 g each |

Total is about 251 g. Covers top out at z 113 (pods) and z 135 (shroud chin), well below the LiDAR scan plane at z ~174.

## Geometry rules built in
- The pod covers float 6 mm above the lid top, on screw bosses and pads. That gap lets the 12 mm outer edge round
  clear the lid's sharp corner by 3 mm. The side and rear offsets are computed from it in `shell_common.inner_face_offset`.
- The inner edge stays at |y| >= 78, so the handheld slot stays open. A short lip runs down to z 108.5.
- Wheel arch: nothing within 55.5 mm of the axle (tyre 40 + 10 mm travel + 5 margin + 0.5 for tessellation), and
  nothing below the axle over the arch. The fender front wall sits only outboard of |y| 155, clear of the bracket and arm.
- The rear wall stops at z 68, above the rocker switch and charge socket in the left pod's rear face. It sits 3.8 mm
  ahead of the servo mast. Both values are PROVISIONAL.
- Shroud: rear edge at x 61.5 (duct riser + 3). Skirt bottom at z 94 (bumper top 90 + 3, so the bumper is never in
  its path). Nothing ahead of x 146.4 below z 125.

## Fasteners
- **Pod covers:** 4 x M3 x 10 button or socket head per cover, from above, through counterbored bosses into **free
  inserts of the lid's existing top grid** (left: (-131,85) (-131,133) (-71,85) (29,133); right: (-111,-85)
  (-111,-133) (-11,-85) (29,-133)). Two plain rest pads per side.
  Lift the cover off to reach the lid bolts.
- **Nose shroud:** 2 x M3 x 16 socket head along Y through the centre chin. The heads sit in the left half and go into
  2 x M3 x 5.7 heat-set inserts in the right half, with 2 x dowel pins (3 x 10 mm).
  The shroud bears on the nose top (3 pads per half) and on the nose sides (2 ribs per half), so bolting the chin
  clamps it on. Add VHB tape on the top pads to hold the rear edge. Nothing is screwed into the nose, which stays unchanged.

## Print orientation
- **Pod covers:** print upright as installed (skirt rim on the bed, 247.7 mm along the bed diagonal or X).
  Use build-plate-only supports under the roof. The outer show faces need no support.
  The part fits the 248 mm envelope with 0.3 mm to spare. Check the slicer's usable area.
- **Nose shroud halves:** print upright with supports under the roof and the chin. Or print inverted on a smooth
  sheet for a glossy top, with supports under the edge rounds.

## Files
`shell_common.py` (helpers), `pod_cover_left.py`, `pod_cover_right.py`, `nose_shroud_left.py`,
`nose_shroud_right.py` (each has `build(params) -> Part`), `refmesh.py` (loads the reference JSON into trimesh meshes),
`build_all.py`. The optional servo_arm_tip_v2 was not made.
