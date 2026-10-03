"""Odometry<->depth scale calibration.

The VIO odometry translation scale drifts relative to the depth stream
(measured: predicted scene motion is ~13% larger than the depth field shows).
The depth camera is the metrically calibrated sensor, so we scale odometry
translations by k such that cross-frame reprojection is consistent
(median measured/predicted depth ratio -> 1.0).

The fitted k and the before/after ratios are reported so the calibration
analysis in the technical report is fully regenerable.
"""
import numpy as np

from lib.io import DEPTH_H, DEPTH_W, load_depth, load_odometry, quat_to_R


def _backproject_frame(pose, depth):
    """depth (H,W) metres -> camera-frame points (H*W,3), valid only."""
    fx, fy, cx, cy = pose["K"]
    v, u = np.mgrid[0:DEPTH_H, 0:DEPTH_W]
    x = (u - cx) / fx * depth
    y = (v - cy) / fy * depth
    pts = np.stack([x, y, depth], axis=-1).reshape(-1, 3)
    return pts[~np.isnan(depth.ravel())]


def reprojection_ratios(odo, depth_i, depth_j, i, j, k):
    """Median (measured/predicted) depth ratio when mapping frame i -> j,
    with odometry translation scaled by k."""
    Ri = quat_to_R(odo["q"][i : i + 1])[0]
    Rj = quat_to_R(odo["q"][j : j + 1])[0]
    ti, tj = odo["t"][i] * k, odo["t"][j] * k
    Ki, Kj = odo["K"][i], odo["K"][j]

    pts = _backproject_frame({"K": Ki}, depth_i)
    world = pts @ Ri.T + ti
    cam_j = (world - tj) @ Rj  # world -> camera j (Rj^T = Rj.T for rotation)
    z = cam_j[:, 2]
    ok = z > 0.05
    u = Kj[0] * cam_j[ok, 0] / z[ok] + Kj[2]
    v = Kj[1] * cam_j[ok, 1] / z[ok] + Kj[3]
    ui = np.clip(np.round(u).astype(int), 0, DEPTH_W - 1)
    vi = np.clip(np.round(v).astype(int), 0, DEPTH_H - 1)
    d2 = depth_j[vi, ui]
    valid = ~np.isnan(d2)
    if valid.sum() < 100:
        return np.nan
    ratios = d2[valid] / z[ok][valid]
    return float(np.median(ratios))


def fit_translation_scale(capture_dir, pairs=None, k0=1.0, iters=6):
    """Fit k such that reprojection ratios across sample frame pairs are ~1.

    Returns (k, report) where report is a list of (pair, ratio_at_k) for the
    final k, for the calibration section of the report.
    """
    odo = load_odometry(capture_dir)
    n = len(odo["frames"])
    if pairs is None:
        # sample pairs spread across the capture with a short baseline
        step = max(1, n // 12)
        starts = np.arange(0, n - 40, step)
        pairs = [(int(s), int(s + 30)) for s in starts[:12]]

    depths = {}
    for i, j in pairs:
        for f in (i, j):
            if f not in depths:
                depths[f] = load_depth(capture_dir, f)

    k = k0
    for _ in range(iters):
        ratios = [
            reprojection_ratios(odo, depths[i], depths[j], i, j, k) for i, j in pairs
        ]
        ratios = [r for r in ratios if not np.isnan(r)]
        if not ratios:
            break
        med = float(np.median(ratios))
        if abs(np.log(med)) < 1e-3:
            break
        k *= med  # ratio < 1 -> predicted too big -> shrink translation

    report = []
    for i, j in pairs:
        r = reprojection_ratios(odo, depths[i], depths[j], i, j, k)
        report.append(((i, j), None if np.isnan(r) else float(r)))
    return k, report
