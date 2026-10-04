# Submission checklist

Deadline: **October 4, 7:00 PM IST**.

## What evaluators get

1. **Repo** (this directory, pushed to GitHub — private, link in the reply
   email) with full git history.
2. **Run instructions** (README): one command per capture; measured 9 min
   (37 s capture) / 38.6 min (115 s capture), `--fast` for long captures.
3. **Outputs on the sample data** (`plan_output/` inside each capture
   directory + copies under `benchmark/`): plan.json (schema v0.1), rendered
   plans, per-surface mosaics. All three provided captures are processed:
   `single_room`, `single_scan_floor_only` (floor-focused; ceiling correctly
   reported as not observable) and `single_scan_with_ceiling` (ceiling
   observed and measured).
4. **Docs**: compliance matrix, benchmark report, technical report (≤6
   pages), fix-loop bundle with before/after logs.

## Final steps before 7 PM

- [ ] `git push origin main` after the last commit
- [ ] Verify the fresh-machine path once: `rm -rf .venv && python3 -m venv
      .venv && .venv/bin/pip install -r requirements.txt` then one run
- [ ] Reply to the email with the repo link + a 2-3 line summary
- [ ] Keep the local copies of the sample data (they may ask for raw logs)

## Repo contents

```
scan_to_plan.py        one command per capture
lib/                   io, calibrate, reconstruct, damage
scripts/               probe_depth_scale.py, compare_captures.py
fix_loop/              FIX_LOOP.md, before_run.log, after_run.log
docs/                  compliance_matrix.md, benchmark_report.md, technical_report.md
benchmark/             plan.json + renders for both sample captures
requirements.txt       numpy scipy matplotlib pillow (pinned)
README.md
```

## Email reply draft

> Hi Siva,
>
> Assessment complete. Repo: <link>.
> One command per capture: `python scan_to_plan.py <capture_dir>` — pip-only
> dependencies, no weights, no network.
>
> All three provided captures are processed. The two floor-focused scans
> never pitch above -17 degrees, so the pipeline reports walls/ceiling
> there as not observable with the pitch evidence; the ceiling capture is
> a ~100 m apartment walkthrough where two ceiling levels are measured
> (2.02 m dominant, 2.40 m secondary) and reported per level. Across all
> captures the pipeline delivers: calibrated geometry (the RGB/depth
> intrinsic rescale and odometry-depth consistency analysis are in the
> report), per-surface damage regions with metric extents, concealed-damage
> flags (rule named), scope line items, and confidence intervals on every
> measurement. The fix loop documents the gates found, root-caused and
> fixed, with regenerable before/after runs.
>
> Happy to walk through it.
>
> Anmol
