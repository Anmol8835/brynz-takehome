# scan-to-plan — LiDAR floor-plan & damage pipeline

One command per capture: a dimensioned plan + per-surface damage report from
Record3D-format iPhone LiDAR captures (depth, confidence, odometry, RGB).

```
python scan_to_plan.py <capture_dir> [--out OUT_DIR]
```

Runs on a clean machine in <15 min (pip-only dependencies, no weights, no
network calls at inference time). Outputs `plan.json` + rendered plans to
`OUT_DIR` (default `<capture_dir>/plan_output/`).

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

The two sample captures are floor-focused walks (camera pitched 17-42 deg
down for the whole capture), so wall and ceiling geometry are not observable
and are reported as such with evidence. The captures contain a furnished
room: furniture tops occlude most of the floor, so damage detection runs
per-surface (floor + furniture tops), not just on the floor.

## Reproduce

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python scan_to_plan.py <capture_dir>
```

Deterministic: fixed RNG seeds throughout; cached model outputs are not
needed because no pretrained models are used.
