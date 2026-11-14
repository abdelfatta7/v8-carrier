# Assembly — one step per joint, in build order

Derived from the CAD on 2026-11-10 after the fix pass (`cad/out/manifest.json`
notes, the part modules and `checks/hole_alignment.py`; fit report run 2
passes every check).  Coordinates are robot frame: X forward, Y left (battery
pod at +Y), Z up, origin on the floor under the drive axle; the nose rear
face / tray front is at x = 58, the bumper front at x = 173, the tray rear at
x = −247, the caster at x = −212.  "M3 × n" means a socket-head M3 screw of
that length into an M3 heat-set insert (4.0 × 6 hole) unless a nut is named;
dowels are 4 × 16 mm pins in 4.1 mm sockets, 8 mm into each side.  Fit all
heat-set inserts into a piece before it is joined to anything (the
split-face inserts are listed per step).  Steps marked **later** depend on
the vacuum measurements and are placeholders in the order only; their parts
are not designed yet.

## Tray

1. **Tray front halves (centreline).**  `tray_front_left` carries the inserts
   in its centre flange's split face (y = 0, z = 18): fit them at x = 45, 5,
   −60, −90.  Push 3 dowels into the left flange at x = 25, −20, −110, offer
   up `tray_front_right` (clearance holes 8 mm through its flange) and fit
   4 × M3 × 12 horizontally from the right side; the heads sit on the right
   flange's outer face (y = −8), reached from below.  Work with the tray
   upside down, slot floor on the bench (its print orientation).  The
   channel tube (x −45.5..−24.5) passes between the bolts at −20 and −60.
2. **Tray rear halves (centreline).**  Same joint between `tray_rear_left`
   (inserts) and `tray_rear_right`: 1 dowel at x = −185, 2 × M3 × 12 at
   x = −150 and −240 (z = 18, heads at y = −8).  The flange is cut away
   between x −228 and −196 for the caster pad; nothing goes there.
3. **Tray cross split.**  The front pair carries the inserts in the cross
   flange's split face (x = −125, z = 18); the rear pair has the clearance
   holes through its 8 mm flange.  Per side 2 dowels at y = ±50, ±100 and
   3 × M3 × 12 at y = ±25, ±75, ±125 fitted from behind (heads on the rear
   flange's rear face, x = −133): 4 dowels and 6 bolts in all.  The channel
   tube and the nose rim are now continuous across both splits.
4. **Caster.**  Pololu 3/4" metal ball caster (height 23 = the ground
   clearance) on the pad under the floor at x = −212, centred on the split:
   2 × M2 (#2) screws up through the Ø2.4 holes at y = ±12, one in each rear
   half.  No spacers.
5. **Cable channel.**  Thread the pod-to-pod cables through the 15 × 10 mm
   tube under the slot floor (x = −35, 218 mm between the pass-throughs) and
   out of the 15 × 15 pass-throughs at (−35, ±88.5) before the pods go on;
   leave the ends hanging up through the floor.  From the pod layouts these
   are the battery-pod to electronics-pod runs: fused battery power for the
   drivers, 5 V from the bucks to the Pi and ESP32, the left drive motor
   leads, the left wheel cliff sensor and the INA226 bus.

## Nose and bumper (one sub-assembly, built with the halves apart)

6. **Prepare the nose halves.**  `nose_right`: press a 625ZZ into the
   drive-end wall seat from the belt-bay side (y = −122, 5 mm deep, 1 mm lip
   behind it), bolt the JGA25-370 brush motor to the inner face of that wall
   (y = −116) at x = 76, z = 70.9 with 2 × M3 on the 17 mm hole spacing, its
   boss in the Ø7.5 hole and the shaft into the belt bay, and fit the 20T GT2
   pulley to the shaft.  `nose_left`: fit the 3 split inserts in the top rib
   (y = 0; x = 65, 78, 128 at z = 131) and the 3 cap inserts in the housing
   end (y = 128).  Both halves: the tray-bolt inserts in the rear face
   (x = 58; `nose_left` at y = 95, 125, `nose_right` at −95, −125, z = 37),
   the 2 duct-flange inserts per half at (±34, 51) and (±34, 119) (for the
   collar, **later**) and the 2 tower-column inserts per half at (75, ±28),
   (131, ±28) from the top.  On each half's front face (x = 143) fit its
   VL53L1X boards behind the ToF positions (centre board on the split, y = 0,
   and y = ±96; 2 × M2 each at z = 55 ± 7.5) and its lever microswitch at
   y = ±108 (2 × M2 at ±103.25 / ±112.75, z = 55, lever forward) — they sit
   behind the bumper and are far easier with the halves apart.  Print
   orientation: each half on its outer face, split face up.
7. **Bumper halves onto the nose halves (pins, springs, end stops, cliff
   sensors).**  Put a cliff sensor in the pocket under each bumper half's
   bottom corner (y = ±118, pocket x 153..170; 2 × M2 at y = ±106 / ±130
   through the pocket floor, looking through the 9 × 18 window).  Drop a
   10 mm OD × 25 mm spring into each nose-face pocket (Ø11 × 4 at (±80, 75))
   and slide the bumper half's 54 mm guide pin (Ø6 at (±80, 75), tip at
   x = 116 when home) into the Ø6.4 sleeve (x 123..143); the spring's other
   end seats in the 2 mm pocket of the Ø16 × 11 boss on the bumper's back
   and is compressed 3 mm at rest (seats 22 mm apart, x 139 and 161).  Fit
   the end stop — an M2 screw or split pin — through the Ø2 vertical cross
   hole at x = 121, which sits 1 mm behind the sleeve's rear face (x = 123)
   with the bumper home and 9 mm behind it at full 8 mm travel; the pin
   tip region is inside the nose shell above the brush housing, reachable
   only while the halves are apart (once joined, only through the tower's
   19 × 44 cable opening with a long tool), which is why the bumper goes on
   now.  The bumper's 10 × 12 pads at y = ±108 press the switch levers; the
   12 × 8 windows at y = 0, ±96 face the ToF boards.  Print each bumper half
   face down.
8. **Join the nose halves and the bumper halves.**  2 dowels at (x 69, z 12)
   in the floor rib and (103, 131) in the top rib, then 3 × M3 × 12 through
   the right rib into the left rib's inserts at x = 65 (behind the tower),
   78 and 128 (inside the tower footprint x 68..138, so before the lidar
   plate).  The two bumper halves meet on the centreline at the same time:
   bridge them with `bumper_joining_plate` behind the centre ToF window,
   4 × M3 × 8 through the plate (x 160..163) into the bosses on the back of
   the halves at y = ±20, z = 37 and 73, reached in the 27 mm gap between
   the nose face and the bumper.
9. **Brush bar, bearings, belt and cap.**  Slide the 240 × 50 bar in through
   the cap-shaped opening in the left wall (y 128..141, 0.5 mm larger than
   the cap) with its drive-end stub into the 625ZZ in the drive-end wall and
   the 20T pulley on the stub inside the belt bay (y −137..−122); loop the
   closed GT2 belt (140 mm, 70 teeth; centre distance 50, motor 32 mm behind
   and 38 mm above the bar axis at x = 108, z = 32.5) over both pulleys
   through the open belt-bay floor.  Press the second 625ZZ into the inner
   face of `brush_cap` (5 mm deep), put the cap over the bar's left stub
   against the housing end (y = 128) and fit 3 × M3 × 10 through the cap into
   the housing-end bosses at (108, 63.5), (81, 17) and (135, 17).  Remove the
   cap to slide the bar out.  The suction mouth is a ±36° sector of the
   housing under the bar, tied to the shell floor by a rib each side.
10. **LiDAR plate.**  4 × M3 × 8 down through `lidar_plate` into the tower's
    column inserts at (75, ±28) and (131, ±28), z = 137; it ties the two nose
    halves together across the top.  The RPLIDAR C1 screws to the plate from
    below with 4 × M2.5 (Ø2.8 holes on a 46 mm square, pitch to be confirmed),
    its cable down through the 20 × 8 slot at the plate's rear (x 74..94, on
    the centreline) and the top-plate opening ahead of the plenum.  Can be
    left until last to keep the sensor out of harm's way.

## Nose to tray, pods, wheels

11. **Nose to tray.**  Stand the nose sub-assembly against the tray's front
    edge (its rear face x = 58 on the tray's nose rim, x 50..58) and fit
    4 × M3 × 16 forward from the slot side through the rim's counterbored
    holes (y = ±95, ±125, z = 37; Ø6.5 × 3.5 counterbores in the rim's rear
    face at x = 50) into the nose's rear-face inserts.  Do both bolts on each
    side before that side's pod goes on: the pod front wall (x = 50) sits
    against the rim's rear face and covers the heads (manifest note).
12. **Collar — later.**  Bolts to the four inserts around the Ø60 duct outlet
    on the nose's rear face (±34, 51 / 119; outlet centre z = 85, provisional)
    from inside the slot; mates with the vacuum's inlet.  Waits on the nose
    diameter and length measurements.
13. **Pods to tray.**  `pod_left` (battery pod) over the left footprint with
    its front wall against the nose rim (x = 50), the cables from step 5
    coming up through its floor pass-through at (−35, 88.5) and the motor
    window in its outer wall over the tray's motor well at x = 0; 6 × M3 × 8
    down through the pod floor into the tray's inserts at x = 43, −48, −145
    and y = 87, 131 (manifest: "bolts to the tray with M3 × 8 through the
    floor").  `pod_right` (electronics pod, the same shell mirrored) likewise
    at y = −87, −131.  Pod contents go in now, before the wheels close the
    outer wall and before the lids:
    * *Left pod, rear compartment (x −192..−24.5):* battery between the end
      stops at x = −140 / −60 (y 87..129) with zip ties through the four
      floor slots at x = −120 / −80, y = 85.5 and 134; rocker switch (19 × 13)
      and charge socket (Ø8) in the rear end wall (x = −195) at y = 100 and
      120, z = 55.
    * *Right pod, rear compartment (everything behind the wheel box):* the
      ESP32-S3 DevKitC-1 standing on end against the outer wall just behind
      the wheel box, in the two grooved rails at x −61..−27 (board x
      −57..−31.5, groove on y = −123), USB end up under the lid's 26 × 10
      slot; the third DRV8871 beside it on the inner side at x −56..−30,
      y −98..−118 on 4 M3 insert standoffs; the Pi 5 (cooler up) on its four
      M2.5 self-tap standoffs on the 58 × 49 pattern at x −149..−64, the
      walls recessed 1 mm over its length; the two rear DRV8871 behind the
      Pi at x −179..−153 (y −90..−110 and −114..−134) on M3 insert standoffs.
    * *Both pods, front compartment (x 24.5..47):* a wheel cliff sensor
      lying along Y over the 9 × 18 floor window at (32, ±114), 2 × M2 at
      y = ±102 and ±126 through the pod floor and into the matching holes in
      the tray.
14. **Wheel modules (one per side; the right side is the mirror).**
    * *Bracket:* `wheel_bracket_left` flat on the pod's outer wall (y
      140..145, x −36..50), its open-bottom motor window over the wall's
      window and its notch above the window on the centre of the axle,
      4 × M3 × 10 through the plate into the pod's 6 mm bosses at (x, z) =
      (−30, 36), (−30, 86), (33, 86), (44, 72) (manifest: "4 × M3 × 10 into the
      pod's outer-wall inserts").
    * *Arm and motor:* press a 625ZZ into `wheel_arm_left`'s pocket from the
      inner face (y = 146, 5 deep; 1 mm lip outside).  Bolt the JGB37-520 to
      the arm's inner face with 6 × M3 × 8 on the Ø31 pattern, gearbox boss in
      the Ø13 bore, shaft out through the arm.
    * *Spring and pivot:* the 10 mm OD × 30 mm spring stands on the arm's Ø7
      post (at x = 0, y = 146.5, inboard of the arm's mid-plane; post base
      z = 64) and its top end is located by the Ø6 peg under the bracket's
      ledge (ledge z 84..90, spanning from the plate to the arm's outer face
      y = 152); the seat gap is 20 mm at ride height, so the spring is
      compressed 10 mm.  Put the spring on the post, offer the arm up with
      the motor body passing inward through the bracket and pod-wall windows
      into the wheel box and the post, pad and spring passing through the
      plate's notch, push the arm up until the spring meets the peg, and fit
      the M5 × 30 pivot through the arm's bearing, a 1 mm M5 washer, the
      bracket and the pod wall at (35, 40); nylock inside the pod's front
      compartment.  Droop (−5 mm) is stopped by the block over the arm's
      13 mm tail (0.4 mm clear at the stop), bump (+5 mm) by the post meeting
      the peg; the pad clears the notch by 0.85 mm at full bump.
    * *Hub and tyre:* `wheel_hub` onto the 6 mm D-shaft (flat toward +X, bore
      13 deep) and lock it with an M3 grub screw in the radial insert on the
      +X boss (6 mm deep, at y = 160.5); stretch the TPU `wheel_tyre` over the
      hub so its inner rib drops into the hub groove.  Both hubs and both
      tyres are the same part; print 2 of each.
15. **Cradle rails and cheeks, rear latch — later.**  They sit on the slot
    floor between the pod rims and at the tray's rear (x = −247) to hold the
    vacuum body; wait on the body outline and battery footprint
    measurements.

## Lids

16. **Lids.**  `lid_left`: first hang the fuse holder, 5V5A buck and INA226
    from the insert standoffs under the lid (M3 × 6 up into the 6 mm bosses
    at x −30..−70, −75..−125 and −130..−155, over the battery), connect them,
    then 6 × M3 × 8 down through the lid into the pod's columns at (−188, 85),
    (−188, 133), (43, 85), (43, 133) (corners), (28.5, 85) (inner wall, ahead
    of the wheel box) and (−28.5, 133) (outer wall, behind it).  `lid_right`:
    the vent slots go over the Pi and the 26 × 10 slot over the ESP32's USB
    end at (−44, −123); same six bolts mirrored.  Print lids flat, standoffs
    up.
17. **Servo mast — later.**  Presses the vacuum's power button; position and
    height wait on the power-button measurement.  The dock contact plate at
    the rear is also not built yet.

## Fastener count

| Item | Qty | Where |
| --- | --- | --- |
| 4 × 16 dowel | 10 | tray centreline 4 (front 3, rear 1), cross split 4, nose split 2 |
| M3 × 12 | 15 | tray centreline 6, cross split 6, nose split 3 |
| M3 × 16 | 4 | nose to tray |
| M3 × 8 | 44 | pods to tray 12, lids 12, lidar plate 4, joining plate 4, motor flanges 12 |
| M3 × 10 | 11 | brackets 8, brush cap 3 |
| M3 × 6 | 12 | boards under lid_left |
| M3 grub screw | 2 | hubs |
| M5 × 30 + nylock + 1 mm washer | 2 | wheel pivots |
| M3 heat-set insert | 92 | tray flanges 12, tray floor 12, nose 18 (incl. 4 for the collar), pod outer walls 8, pod lid columns 12, DRV8871 standoffs 12, lid_left bosses 12, bumper bosses 4, hubs 2 |
| M2 self-tap | 20 + 2 | ToF 6, switches 4, cliff sensors 8, bumper end stops 2; caster 2 (#2 / M2) |
| M2.5 self-tap | 8 | Pi 5 standoffs 4, LiDAR 4 |
| 625ZZ | 4 | brush bar ends 2, wheel arms 2 |
| springs | 4 | wheel 2 (10 OD × 30, 10 mm preload), bumper 2 (10 OD × 25, 3 mm preload) |
