# scan-to-plan — LiDAR floor-plan & damage pipeline

One command per capture: a dimensioned plan + per-surface damage report from
Record3D-format iPhone LiDAR captures (depth, confidence, odometry, RGB).

```
python scan_to_plan.py <capture_dir> [--out OUT_DIR]
```

Pip-only dependencies, no weights, no network calls at inference time.
Outputs `plan.json` + rendered plans to `OUT_DIR` (default
`<capture_dir>/plan_output/`).

Measured runtimes (7 GB RAM laptop): 37 s capture -> 8.1 min; 115 s capture
-> 72.6 min; 162 s capture -> ~11 min with `--fast` (sparser frames, 1 cm
cells, 1 raised surface), full quality longer. The per-surface damage
mosaics dominate runtime; the geometry stages finish in ~2 min for all
captures. On a memory-tight machine run captures one at a time.

## Input format (Record3D-style)

```
<capture_dir>/
  odometry.csv       per-frame timestamp, frame, x,y,z, qx,qy,qz,qw, fx,fy,cx,cy
  camera_matrix.csv  reference intrinsics (native resolution)
  imu.csv            accelerometer + gyroscope
  rgb.mp4            1920x1440 colour video, 1 frame per depth frame
  depth/??????.png   256x192 uint16, millimetres, 0 = invalid
  confidence/??????.png  256x192 uint8
```

The sample data was captured with Record3D (free App Store app); the
format is its unpacked export. odometry.csv intrinsics are native-res
(1920x1440) and are rescaled to the 256x192 depth frames internally.

## Output contract (plan.json)

| Field | Meaning |
|---|---|
| `calibration` | odometry-depth translation scale fit + per-pair reprojection ratios |
| `camera_height_m` (+CI) | median trajectory height above the floor plane |
| `raised_surfaces` | dominant horizontal planes above the floor (furniture tops) |
| `ceiling_height_m` | histogram peak above 2 m, or `null` + note when the capture never looks up |
| `walls` | wall segments from near-vertical planes; empty + note when the capture is floor-focused |
| `floor_area_m2` | polygon area (shoelace) or occupancy fallback |
| `damage.surfaces[]` | per-surface orthomosaic stats: cracks, stains, concealed-damage flags (each with the rule that fired), coverage fraction, extent CI |
| `scope_items` | per-damage-region work items keyed to surface |

Every measurement carries a confidence interval derived from the geometry
(point spread, cell size), reported rather than guessed.

## Pipeline stages

1. **Calibration** — odometry translation scale fit against the depth stream
   (cross-frame reprojection), converged at k = 1.000 on the sample data
   after correcting the native-res intrinsics.
2. **Fusion** — batched backprojection of depth+confidence with voxel
   downsampling (1 cm), bounded to 6M points.
3. **Floor** — RANSAC plane through the bottom envelope of the cloud
   (the lowest 10% of points), so raised surfaces can't be mistaken for the
   floor. The full-cloud fit RMS doubles as drift evidence.
4. **Raised surfaces** — sequential RANSAC for dominant horizontal planes
   above the floor (furniture).
5. **Walls** — near-vertical planes; empty on floor-focused captures, with
   an explicit note instead of a hallucinated polygon.
6. **Damage** — per-surface orthomosaics (RGB orthorectified onto each
   dominant plane), then: cracks (dark elongated components vs local
   background), stains (local chroma outliers), concealed-damage flags
   (coverage gaps inside the scanned region). Heuristic and local by design;
   see the technical report for the threshold sensitivity analysis.

## Sample-data findings (see docs/technical_report.md)

Three sample captures: two floor-focused walks (`single_room`,
`single_scan_floor_only` — camera pitched down the whole capture) and one
`single_scan_with_ceiling` (camera pitches up to +25 deg, ceiling points
observed). On the floor-focused captures, ceiling/wall geometry is not
observable and is reported as such with pitch evidence; on the ceiling
capture the ceiling histogram fires and reports a real height. The rooms
are furnished: furniture tops occlude much of the floor, so damage
detection runs per-surface (floor + furniture tops), not just the floor.

## Reproduce

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python scan_to_plan.py <capture_dir>
```

Deterministic: fixed RNG seeds throughout; cached model outputs are not
needed because no pretrained models are used.
