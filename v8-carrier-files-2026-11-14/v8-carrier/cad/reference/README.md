# Reference geometry (not our designs)

Files the owner supplied on 2026-11-13: community 3D-printable attachments for
the Dyson V7/V8 quick-release socket. They are external designs of unknown
licence, kept here only as reference and for test fits; every part we print
is re-modelled parametrically in `cad/parts/`.

| File | What it is |
| --- | --- |
| `community_dyson_v7_v8_attachments.3mf` | Four attachments (long narrow nozzle, two crevice/brush variants, a wide nozzle). All share the same male connector at the socket end. |
| `dyson_v7_v8_connector_50mm.stl` | The first 50 mm of object 1 of that set, cut by the coordinator: the bare connector, as a test piece for the handheld's socket. Watertight, 14.7 cm3. |

## Connector profile measured from the model (mm, from the tip)

| Feature | Value |
| --- | --- |
| Tube outside diameter | 34.8, round, 0 to 47 |
| Bore (airway) | 30.7, constant |
| Entry ring | 0 to 4.75 plain round |
| Side bumps (catch shoulders), at 3 and 9 o'clock | 5 to 34.5, ramp 5 to 7.25, protrude 2.45 each side (39.7 across) |
| Top key rib, 12 o'clock | about 10 to 43, protrudes up to 2.85 (40.5 overall) |
| Insertion length to the shoulder | about 47; the body necks to 33.5 outside / 28.5 bore behind it |

The printed test piece tells us whether the handheld's socket accepts this
standard connector (slide in, click, release with the red button) and in
which clock position the key rib sits. If it fits, the duct plug copies this
profile and sheet items N1 to N5 are no longer critical.
