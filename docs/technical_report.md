# Technical Report — scan-to-plan

**Data:** three Record3D-format LiDAR captures supplied as sample data
(`single_room/c00a170fe1`, 37 s / 1715 frames; `single_scan_floor_only/1a8384c3f6`,
115 s / 5251 frames; `single_scan_with_ceiling/c7d28f72c6`, 162 s / 9745 frames).
The first two are floor-focused walks of the same furnished space (camera
pitched down throughout); the third pitches up to +25 deg and observes the
ceiling (vertical span ~3.1 m), exercising the ceiling-height path.

## 1. Architecture

```
odometry.csv ──> intrinsic rescale (7.5x) ─┐
depth+confidence ──────────────────────────┤─> odometry-depth scale calibration
rgb.mp4 ──> ffmpeg decode (960x720) ────────┤        │
                                             │        v
                                    backproject + voxel fusion (1 cm, ≤6M pts)
                                             │
                ┌────────────────────────────┼──────────────────────────┐
                v                            v                          v
      bottom-envelope floor RANSAC   vertical-plane RANSAC     horizontal-plane RANSAC
      (floor plane + warp RMS)       (walls; empty when the    (raised surfaces:
                │                    capture never looks up)   furniture tops)
                v                            │                          │
      camera height + CI,             wall lines -> corners ->  per-surface RGB
      ceiling histogram (honest       wall segments + openings  orthomosaics (5 mm cells)
      "not observable" path)          (honest "not observed")         │
                                                                       v
                                                       cracks (dark elongated components),
                                                       stains (local chroma outliers),
                                                       concealed flags (coverage gaps, rule named)
                                                                       │
                                                                       v
                                                plan.json + rendered plans (PNG)
```

Stage boundaries are pure functions over raw inputs; the whole run is
deterministic (fixed RNG seeds). No pretrained weights, no network calls.

## 2. Tier design and device matrix

| Tier | Input | Path in this pipeline | Measured on sample data |
|---|---|---|---|
| LiDAR | depth + poses + intrinsics (Pro iPhone, Record3D) | full pipeline | yes (all three captures) |
| Video | RGB walkthrough | COLMAP/GLOMAP SfM replaces the odometry block; the floor/surface/damage stages are input-agnostic | no data supplied — designed, unmeasured |
| Photos | 2-8 stills/room | same: SfM on stills, then identical downstream | no data supplied |

Honest note: the downstream stages (floor, surfaces, damage) consume a point
cloud + posed RGB, so only the front end differs per tier. The sample data
exercises the LiDAR tier end-to-end.

## 2b. Ceiling detection

The ceiling stage histograms the cloud along the floor normal above
`max(2.0 m, camera height + 30 cm)` and accepts local maxima with >=0.2% of
the cloud as support, separated by >=30 cm. All supported levels are
reported (`ceiling_levels`) with per-level support and spread; the dominant
level is the headline `ceiling_height_m`. This handles both the
single-slab case and captures containing multiple rooms or a pelmet/soffit
above a lower ceiling. When no supported level exists, the field is `null`
with a note carrying the pitch evidence — never a hallucinated value (see
fix loop, Gate 2).

## 2c. Ceiling capture findings (single_scan_with_ceiling)

This capture walks ~100 m through a furnished apartment (bathroom, kitchen,
windows) with the camera pitching from -31° to +33°. The pipeline reports
two supported ceiling levels — 2.02 m and 2.40 m above the fitted floor,
with support shares of 0.3% and 0.2% of the cloud. The supports are small
because most of the cloud is floor and furniture; this is reported per
level rather than pretending a single room-height. Camera height comes out
1.02 m ± 0.24 m: the wider interval reflects genuine variation during the
walkthrough (the phone was held lower and moved more than in the two
floor-scan captures). The floor plane fit (rms 11.5 mm) and drift evidence
carry over unchanged.

## 3. Calibration analysis

1. **Intrinsic resolution mismatch (the decisive fix).** `odometry.csv`
   carries fx≈1597.9, cx≈955.4, cy≈717.7 — values for the native 1920x1440
   sensor (cx,cy = image centre). The depth frames are 256x192, a 7.5x
   downscale of the same FOV. Using native intrinsics on the small frames
   sends every ray off-axis; reprojection consistency was 0.87 median with
   p10-p90 = 0.55-1.20. After rescaling K by 256/1920, consistency is
   0.973-1.023 median with p10-p90 ≈ 0.96-1.23 and the translation-scale
   fit converges to **k = 1.000** — no further scale correction needed.
2. **Depth units.** Depth PNGs are uint16 with values 184-5500. The near
   plane (min ≈ 0.2 m) matches the LiDAR sensor's physical minimum, so the
   units are **millimetres**. Confirmed physically: bottom-envelope floor
   plane puts the camera 1.46-1.48 m above the floor (handheld).
3. **Floor estimator.** A floor-focused capture contains dense slabs above
   the floor (furniture). Plain RANSAC locks onto the densest slab; the
   floor is recovered as the plane through the bottom envelope (lowest 10%
   of points). Full-cloud fit RMS = 11.5 mm.

## 4. Drift handling

- Odometry-depth scale is calibrated (k = 1.000 after the intrinsic fix).
- The floor plane's global fit residual (11.5 mm RMS) is reported as warp
  evidence on every run; no systematic warp is present.
- Camera-height variation over the walk: ±0.09-0.10 m (operator hand bob,
  not drift — trajectory height is consistent with a handheld phone).
- Loop closure: the trajectory never revisits its start (closest
  early/late approach 2.8 m), so no revisit constraint is available; this
  is stated rather than papered over. For longer captures the pipeline's
  plane stage provides the anchors a pose-graph correction would need.

## 5. Error budget (LiDAR tier, observed)

| Source | Magnitude |
|---|---|
| depth noise (LiDAR) | ~1 cm -> plane fits at 11.5 mm RMS |
| voxel quantisation | 1 cm |
| odometry-depth residual scale | <1% (reprojection spread) |
| mosaic cell | 5 mm -> extent CI ±5 mm per region |
| camera height | ±0.10 m |

Wall-length and opening gates cannot be scored on the sample data (no wall
views); where walls exist the expected budget is ±2-3 cm, consistent with
the LiDAR-tier gates.

## 6. Fix loop (see fix_loop/FIX_LOOP.md)

Worst gate: ceiling height hallucinated 2.00 m from a band-edge histogram
artifact on a capture that never looks up. Fixed by requiring interior
peaks with ≥0.2% support; ceiling now reports `null` + evidence note.
Runtime secondary fix: single RGB decode shared across surfaces
(546.6 s -> ~430 s).

## 7. Known failure modes

- **Mirrors/glass:** LiDAR depth returns phantom geometry behind the pane;
  confidence maps (0-value) partially flag it; not validated on sample data.
- **Low light:** RGB-based damage detection degrades; depth is unaffected.
- **Floor-only captures:** walls/ceiling/openings not observable — reported
  honestly with the pitch evidence rather than estimated.
- **Stain heuristic:** local chroma outliers include shadows/texture;
  extents carry the cell CI but class confidence is low — a pretrained
  segmentation model (disclosed) is the next upgrade.
