# Submission checklist

Deadline: **October 4, 7:00 PM IST**.

## What evaluators get

1. **Repo** (this directory, pushed to GitHub — private, link in the reply
   email) with full git history.
2. **Run instructions** (README): one command per capture, <15 min on a
   clean machine.
3. **Outputs on the sample data** (`plan_output/` inside each capture
   directory + copies under `benchmark/`): plan.json (schema v0.1), rendered
   plans, per-surface mosaics.
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
> One command per capture: `python scan_to_plan.py <capture_dir>` — runs in
> under 10 minutes on a clean machine (pip-only dependencies, no weights,
> no network).
>
> Both sample captures are floor-focused (the camera never pitches above
> -17 degrees), so the pipeline reports walls/ceiling as not observable
> with the pitch evidence, and instead delivers what the data supports:
> calibrated floor geometry, raised-surface detection, per-surface damage
> regions with metric extents, concealed-damage flags (rule named), scope
> line items, and confidence intervals on every measurement. The technical
> report covers the calibration analysis (intrinsic rescale, odometry-depth
> consistency), the drift story, the error budget, and the fix loop (two
> gates found, root-caused, fixed, before/after regenerable).
>
> Happy to walk through it.
>
> Anmol
