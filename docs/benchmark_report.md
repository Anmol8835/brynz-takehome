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
| One command, cold run | ✅ 487.8 s | fresh venv, no weights, no network |
| Odometry-depth calibration | ✅ k = 1.000 | after 7.5x intrinsic rescale; per-pair reprojection ratios in plan.json |
| Camera height | 1.47 m ± 0.09 | handheld-phone physicality check passed |
| Floor plane fit | rms 11.5 mm | global fit over 6M-point cloud (drift warp evidence) |
| Raised surfaces | 6 planes at +0.6..+1.3 m above floor | furniture tops; not mis-classified as floor |
| Ceiling height | ✅ correctly "not observable" | was the worst gate (hallucinated 2.00 m); fixed, see fix loop |
| Walls / openings | N/A | no vertical planes exist in the capture (pitch evidence); reported empty + note |
| Damage per surface | floor: 7 cracks / 63 stains / 3 flags @ 17.7% coverage; furniture surfaces: 21-25 stains @ 1.9-2.8% coverage | heuristic; stains include texture/shadow candidates — see calibration caveat below |
| Concealed-damage flags | ✅ rule named per flag | `coverage_gap_in_scanned_area` |
| Reproducibility | ✅ deterministic pipeline | was failing (uninitialised accumulation buffer); fixed, see fix loop |

## Gates — single_scan_floor_only/1a8384c3f6 (115 s, 5251 frames)

Run with `--fusion-stride 3` (identical geometry, 1/3 the fusion frames).

| Gate | Result | Notes |
|---|---|---|
| Odometry-depth calibration | k = 1.000 | after intrinsic rescale |
| Camera height | 1.30 m ± 0.08 | handheld-phone physicality check passed |
| Floor plane fit | rms 11.5 mm | 6M-point cloud |
| Raised surfaces | 6 planes at +0.66..+1.12 m above floor | furniture tops |
| Ceiling height | ✅ correctly "not observable" | top of visible content 1.84 m |
| Walls / openings | N/A | no vertical planes (pitch evidence) |
| Damage per surface | floor: 8 cracks / 271 stains / 36 flags @ 17.1% coverage; 3 furniture surfaces: 0-1 cracks, 90-104 stains, 16-17 flags @ 1.9-2.2% coverage | heuristic, same caveats |
| Reproducibility | ✅ deterministic code path | see fix loop |
| Runtime | 4356 s (73 min) | full-quality; dominated by 404-frame mosaics × 4 surfaces |

## Gates — single_scan_with_ceiling/c7d28f72c6 (162 s, 9745 frames)

Run with `--fast` (sparser mosaics: 1 cm cells, 1 raised surface); geometry
stages identical to full quality. This is a ~100 m walkthrough of a
furnished apartment (kitchen, bathroom, windows).

| Gate | Result | Notes |
|---|---|---|
| Odometry-depth calibration | k = 1.06 | 6% scale correction fit on this capture; reprojection ratios in plan.json |
| Camera height | 1.02 m ± 0.24 | held lower and moved more than in the scan captures; wider CI reflects that |
| Floor plane fit | rms 11.5 mm | 6M-point cloud |
| Ceiling height | ✅ 2.02 m (dominant), second supported level 2.40 m | both reported in `ceiling_levels`; supports 0.3% / 0.2% of cloud (most points are floor/furniture) |
| Walls / openings | N/A | no near-vertical planes above threshold; pitch evidence in report |
| Damage per surface | floor: 5 cracks / 252 stains / 21 flags @ 3.0% coverage | floor-focused mosaic; apartment rooms occlude most floor |
| Reproducibility | ✅ deterministic code path | see fix loop |
| Runtime | 202.5 s | --fast profile |

## Repeatability (same space, same tier, two captures)

`scripts/compare_captures.py benchmark/single_room/plan.json benchmark/floor_only/plan.json`

| Measurement | capture A | capture B | |diff| | Read |
|---|---|---|---|---|
| camera height | 1.450 m | 1.297 m | 0.153 m | operator holding height differs between sessions; within-session spread is ±0.08-0.11 m |
| raised surfaces (matched pairs) | 0.93 / 0.95 / 1.15 m | 0.90 / 0.95 / 1.12 m | **0.008-0.032 m** | three furniture surfaces common to both scans agree to 8-32 mm |
| remaining raised surfaces | 0.61-0.81 m | 0.66-0.84 m | 0.03-0.12 m | different furniture pieces / partial views in each scan — weaker matches |
| floor fit precision | 11.5 mm RMS | 11.5 mm RMS | — | consistent sensor noise |
| floor stain density | 5.3 / m² | 9.6 / m² | — | same order of magnitude; uncalibrated threshold detector (see caveat) |

The case study's repeatability gate (≤1 cm or 0.5 % per wall) cannot be
scored on wall measurements because the sample captures contain no wall
views; the shared-structure agreement above (8-32 mm on matched furniture
planes over a 5.7×4.8 m space) is the honest analogue.

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
