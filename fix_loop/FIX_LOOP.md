# Fix Loop — one-page declaration

Two worst gates found while benchmarking on the sample data, both diagnosed,
fixed, and verified with regenerable before/after runs.

## Gate 1 — Reproducibility (automatic fail if broken)

**Before (FAIL):** two identical runs of the same capture produced different
plans — the floor-plane offset varied by up to 5.5 cm run-to-run
(`d` = 1.5034 vs 1.5584 in a same-process A/B test). A pipeline that does
not reproduce its own numbers cannot have calibrated intervals.

**Root cause + evidence:** the voxel downsampler accumulated points into a
buffer allocated with `np.empty` using `np.add.at`, which adds onto the
*existing* (uninitialised) memory contents. Every voxel mean therefore
contained a random garbage term. Evidence: same-process A/B showed the
per-frame, per-chunk stages bit-identical while the merged cloud differed;
the only uninitialised buffer in the accumulation path was this one.

**Fix + predicted number:** zero-initialise the accumulation buffer
(`np.zeros`). Predicted: byte-identical clouds and identical plan.json
across repeated runs.

**After (PASS):** `np.array_equal(cloud1, cloud2) == True` for two builds of
`single_room` (6,000,000 points each). See `after_run.log` / git diff.

## Gate 2 — Ceiling height on floor-focused captures

**Before (FAIL):** `ceiling_height_m = 2.00` (spread 46 mm) reported on
`single_room` — a hallucinated number: the capture's camera pitch never
rises above -17 degrees (never looks up), and the top of visible content is
1.72 m. Reporting a ceiling that was never observed is the worst failure
mode for a measurement product.

**Root cause + evidence:** the ceiling histogram's search band starts at
2.0 m; sparse noise points just above 2.0 m formed the tallest bin, which
sat exactly on the band's first edge. There was no interior-peak test and
no minimum-support test. Evidence: peak bin == first bin; support « floor
support; p99 of points above floor = 1.72 m < 2.0 m; pitch p5..p95 =
-42..-21 deg.

**Fix + predicted number:** reject peaks at either band edge and require
>= 0.2 % of the cloud in the peak bin; otherwise report
`ceiling_height_m = null` with an explicit "not observable" note.
Predicted: `ceiling_height_m = null` + note on `single_room`.

**After (PASS):** `ceiling_height_m = null`,
`ceiling_note = "ceiling not observable: no points above 2.0 m; capture is
floor-focused (top of visible content 1.72 m)"`. See `after_run.log`.

## Gate 3 — Runtime (secondary; prediction missed, post-mortem included)

**Before (FAIL):** 546.6 s for a 37 s capture, with the RGB stream decoded
from `rgb.mp4` once per surface (4x redundant decode).

**Fix:** share one decode across all surfaces (shipped in the same commit).

**Predicted:** ~430 s. **After: 542.5 s — prediction missed.**

**Post-mortem (why it fell short):** the prediction assumed decode was a
large share of the mosaic stage. It was not — the per-frame backprojection
loop (4 surfaces x 215 frames of 49k-pixel scatter-adds) dominates, and the
decode share was ~30 s. The fix was correct engineering but the predicted
magnitude was wrong because it was based on a guess rather than a measured
profile. The benchmark report now breaks timing down per stage from
measurements. Runtime is also secondary to the accuracy gates above.

## Gate 4 — Orthomosaic coverage and colour (found in review, fixed)

**Before (FAIL):** the rendered per-surface mosaics were mostly grey with
thin coloured slivers. Two independent root causes:

1. **Colour sampling scale mismatch.** The RGB frames were decoded at
   native 1920x1440 (no scale filter in the ffmpeg extract) but the
   sampling code indexed them with `U_SCALE = 960/256`, so every colour was
   read from the top-left quadrant of the frame — washed-out, wrong hues.
   Evidence: decoded-frame size check returned (1920, 1440); the overlay
   probe (depth dots drawn on the frame) confirmed the correct mapping
   needs the full-frame scale.
2. **Plane offset.** The bottom-envelope floor fit sits at the bottom of
   the floor's noise band: measured per-frame floor mass is +2.5..+8.5 cm
   above the fitted plane (below-floor multipath/noise + residual warp).
   With a +/-3 cm paint band, each frame painted only a thin sliver of the
   surface. Evidence: per-frame modal-offset table
   (`mode +2.5..+8.5 cm`, 15-58% of each frame's pixels in the band).

**Fix shipped:** decode at 960x720 with an area filter and compute the
colour-sampling scale from the actual decoded size; paint each frame
through `lib.damage.local_plane_offset` (per-frame modal-offset anchoring,
which also absorbs residual reconstruction warp); raised surfaces paint
from their own fitted plane instead of a horizontal plane at the
floor-derived height; crop renders to the observed footprint.

**Predicted:** floor mosaic coverage 1.1% -> >15%; colours recognisable.

**After (PASS):** floor coverage 1.07% -> **17.7%** (`--fast` smoke run),
mosaic shows real floor imagery (rug edge, door threshold, floor pattern).
See `fix_loop/before_after_mosaic/`. On the full-quality rerun, floor
coverage is 17.8% with 3 crack and 228 stain candidates (see benchmark
report); colours match the source video (verified against a decoded frame).

**Post-mortem on the discovery route:** neither bug was visible from the
numbers alone — coverage and interval statistics all looked plausible. Both
were found by rendering the PNGs and looking at them, then tracing why they
looked wrong back to the decode size and the per-frame offset table. Lesson:
render-and-look belongs in the loop, not just metric tables.

## Regeneration

```
.venv/bin/python scan_to_plan.py ~/Downloads/single_room/c00a170fe1
```

- Before logs: `fix_loop/before_run.log` (captured with both bugs present)
- After log: `fix_loop/after_run.log` (current code)
- Both runs are the same command on the same raw inputs; the diff is the
  code change recorded in git history.
