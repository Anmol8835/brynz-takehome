# Benchmark report — sample data

Both sample captures are Record3D-format LiDAR scans of the same furnished
space, floor-focused (camera pitch p5..p95 = -42..-21 deg; never looks up).
The pipeline was run with the same command on both:

```
.venv/bin/python scan_to_plan.py <capture_dir>
```

## Gates — single_room/c00a170fe1 (37 s, 1715 frames)

| Gate | Result | Notes |
|---|---|---|
| One command, cold run | ✅ 542.5 s | fresh venv, no weights, no network |
| Odometry-depth calibration | ✅ k = 1.000 | after 7.5x intrinsic rescale; per-pair reprojection ratios in plan.json |
| Camera height | 1.46 m ± 0.09 | handheld-phone physicality check passed |
| Floor plane fit | rms 11.5 mm | global fit over 6M-point cloud (drift warp evidence) |
| Raised surfaces | 6 planes at +0.62..+1.28 m above floor | furniture tops; not mis-classified as floor |
| Ceiling height | ✅ correctly "not observable" | was the worst gate (hallucinated 2.00 m); fixed, see fix loop |
| Walls / openings | N/A | no vertical planes exist in the capture (pitch evidence); reported empty + note |
| Damage per surface | floor: 0 cracks / 80 stains / 0 flags @ 1.2% coverage; furniture surfaces: 57-75 stains @ 1.8-3.0% coverage | heuristic; stains include texture/shadow candidates — see calibration caveat below |
| Concealed-damage flags | ✅ rule named per flag | `coverage_gap_in_scanned_area` |
| Reproducibility | ✅ byte-identical clouds across runs | was failing (uninitialised accumulation buffer); fixed, see fix loop |

## Gates — single_scan_floor_only/1a8384c3f6 (115 s, 5251 frames)

Run with `--fusion-stride 3` (identical geometry, 1/3 the fusion frames).

| Gate | Result | Notes |
|---|---|---|
| Odometry-depth calibration | k = 1.031 | same capture type; per-pair ratios in plan.json |
| Camera height | 1.31 m ± 0.08 | handheld-phone physicality check passed |
| Floor plane fit | rms 11.5 mm | 6M-point cloud |
| Raised surfaces | 6 planes at +0.67..+1.03 m above floor | furniture tops |
| Ceiling height | ✅ correctly "not observable" | top of visible content 1.87 m |
| Walls / openings | N/A | no vertical planes (pitch evidence) |
| Damage per surface | floor: 0 cracks / 384 stains / 7 flags @ 2.1% coverage; 3 furniture surfaces: 0-1 cracks, 256-276 stains, 10-14 flags @ 1.7% coverage | heuristic, same caveats |
| Reproducibility | ✅ deterministic code path | see fix loop |

## Repeatability (same space, same tier, two captures)

`scripts/compare_captures.py benchmark/single_room/plan.json benchmark/floor_only/plan.json`

| Measurement | capture A | capture B | |diff| | Read |
|---|---|---|---|---|
| camera height | 1.461 m | 1.310 m | 0.151 m | operator holding height differs between sessions; within-session spread is ±0.08-0.09 m |
| raised surface (matching pair) | 0.831 m | 0.814 m | **0.017 m** | the one furniture surface common to both scans agrees to 1.7 cm |
| other raised surfaces | 0.60-1.32 m | 0.67-1.03 m | 0.07-0.28 m | different furniture pieces in each scan's coverage area — not comparable |
| floor fit precision | 11.5 mm RMS | 11.5 mm RMS | — | consistent sensor noise |
| floor stain density | 190 / m² | 110 / m² | — | same order of magnitude; uncalibrated threshold detector (see caveat) |

The case study's repeatability gate (≤1 cm or 0.5 % per wall) cannot be
scored on wall measurements because the sample captures contain no wall
views; the shared-structure agreement above is the honest analogue.

## Damage-detection calibration caveat

The stain/crack detectors are threshold-based and uncalibrated against
labelled data (none supplied). Two failure modes are known and stated:

1. shadows and floor texture can fire the stain rule (false positives);
2. faint stains below the chroma threshold are missed (false negatives).

Extent CIs are the mosaic cell size (±5 mm) — a geometric bound, not a
classification confidence. Class confidence is low and is reported as such;
a pretrained segmentation model (disclosed) would be the upgrade path.

## Timing (measured on a 7 GB RAM laptop, single core bound)

| Stage | single_room (37 s capture) | single_scan_floor_only (115 s capture) |
|---|---|---|
| calibration | ~2 s | ~4 s |
| fusion | ~20 s | ~90 s (--fusion-stride 3) |
| planes/floor/walls | ~10 s | ~15 s |
| mosaics + damage | ~500 s (4 surfaces × 215 frames) | ~2200 s (4 surfaces × 404 frames) |
| **total** | **542.5 s (9.0 min)** | **2317.8 s (38.6 min)** |

The mosaic stage dominates and scales with capture length and grid size.
`--fast` (200 frames, 1 cm cells, 1 raised surface) cuts the long capture
to roughly 11-12 min with minimal coverage loss; geometry is unaffected.
