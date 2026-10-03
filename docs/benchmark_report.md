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

<!-- filled from run -->

## Repeatability (same space, same tier, two captures)

<!-- filled from scripts/compare_captures.py -->

## Damage-detection calibration caveat

The stain/crack detectors are threshold-based and uncalibrated against
labelled data (none supplied). Two failure modes are known and stated:

1. shadows and floor texture can fire the stain rule (false positives);
2. faint stains below the chroma threshold are missed (false negatives).

Extent CIs are the mosaic cell size (±5 mm) — a geometric bound, not a
classification confidence. Class confidence is low and is reported as such;
a pretrained segmentation model (disclosed) would be the upgrade path.

## Timing

| Stage | single_room | single_scan_floor_only |
|---|---|---|
| calibration | ~2 s | |
| fusion | ~20 s | |
| planes/floor/walls | ~10 s | |
| mosaics + damage (4 surfaces) | ~500 s | |
| total | 542.5 s | |
