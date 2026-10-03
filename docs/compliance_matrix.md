# Compliance matrix

Requirement -> file path -> artifact -> status. Every requirement from the
case-study output contract, scored against the provided sample data.

## Output contract (per capture)

| # | Requirement | File | Artifact | Status |
|---|---|---|---|---|
| 1 | One command per capture | `scan_to_plan.py` | CLI, `<15 min` on clean machine | ✅ |
| 2 | JSON to published schema | `plan.json` (schema v0.1, documented in README) | `plan_output/plan.json` per capture | ✅ |
| 3 | Dimensioned room plan: walls | `scan_to_plan.py` wall-plane stage | `walls[]` | ⚠️ N/A on sample data — no vertical planes exist in floor-focused captures (pitch ≤ -17° entire walk); reported as empty with evidence + note, never hallucinated |
| 4 | Ceiling height | ceiling histogram stage | `ceiling_height_m` + `ceiling_note` | ⚠️ N/A on sample data — correctly reported `null` + not-observable note (see fix loop: this was the worst gate, now fixed) |
| 5 | Floor area | `floor_area_m2` (observed floor via mosaic) + `scanned_footprint_m2` | plan.json | ✅ |
| 6 | Openings (widths, detection scored) | wall openings stage | — | ⚠️ N/A — walls not observable in sample data |
| 7 | Per-surface damage regions, class + metric extent | `lib/damage.py` | `damage.surfaces[]` (floor + 3 furniture surfaces), cracks/stains with length/area in metres | ✅ heuristic, disclosed |
| 8 | Concealed-damage flags with the rule that fired | `lib/damage.py` gap detection | `flags[]` with `rule` + `rule_description` | ✅ |
| 9 | Scope line items keyed to surfaces | `lib/damage.py::scope_items` | `scope_items[]` | ✅ |
| 10 | Confidence interval on every measurement | geometry-derived CIs (point spread, cell size) | `*_ci_m` / `extent_ci_m` fields | ✅ |
| 11 | Rendered plan | matplotlib render stage | `plan_output/density_debug.png`, `mosaic_*.png` | ✅ |

## Deliverables

| # | Deliverable | File | Status |
|---|---|---|---|
| 1 | Compliance matrix | this file | ✅ |
| 2 | Capture route + device matrix | sample data supplied by employer (Record3D-format); format documented in README. No self-capture route needed for this round | ✅ (documented) |
| 3 | Repo + README, fresh run <15 min, one command | `README.md`, `requirements.txt` | ✅ |
| 4 | Reproduction bundle | pip-only deps, fixed RNG seeds, no pretrained weights -> every number regenerable from raw inputs | ✅ |
| 5 | Benchmark report: gates, repeatability, head-to-head, timing | `docs/technical_report.md`, `benchmark/` | ✅ gates + repeatability (two captures of the same space); head-to-head N/A — no physical access to the scanned space |
| 6 | Fix loop bundle | `fix_loop/FIX_LOOP.md`, `fix_loop/before_run.log`, `fix_loop/after_run.log` | ✅ |
| 7 | Technical report ≤6 pages | `docs/technical_report.md` | ✅ |
| 8 | Raw benchmark data | provided sample data + all outputs under `plan_output/` | ✅ |

## Gates (sample-data reality)

- Opening widths / detection: not measurable (no walls in data) — stated, not faked.
- Ceiling height: not observable — fixed to report honestly (fix loop).
- Repeatability: the two sample captures scan the same space at the same tier -> floor height and camera height agreement reported in the benchmark section.
- Drift: odometry-depth calibration (k=1.000 after intrinsic rescale), floor global-fit RMS as warp evidence, no revisit event available for loop closure — stated with numbers.
- Photo/video tiers: no data supplied; the tier design + error budget are described in the report, unmeasured.
