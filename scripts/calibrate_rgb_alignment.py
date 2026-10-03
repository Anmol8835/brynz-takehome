"""Calibrate the depth->RGB sampling scale empirically.

The RGB stream (1920x1440) and LiDAR depth (256x192) may have different
fields of view, so the naive scale (1920/256 = 7.5) can be wrong. We find
the true scale by photometric consistency: the same 3D point viewed in two
different frames must sample the same colour. We search the scale that
maximizes the cross-frame colour correlation.

    python scripts/calibrate_rgb_alignment.py <capture_dir>
"""
import argparse
import os
import subprocess
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.io import DEPTH_H, DEPTH_W, load_depth, load_odometry, quat_to_R

FRAMES_DIR = "/tmp/rgb_calib_frames"


def decode_frames(capture_dir, frame_ids):
    """Decode specific source frames at full 1920x1440 for calibration."""
    os.makedirs(FRAMES_DIR, exist_ok=True)
    for f in frame_ids:
        out = os.path.join(FRAMES_DIR, f"{f:06d}.jpg")
        if not os.path.exists(out):
            subprocess.run(
                ["ffmpeg", "-loglevel", "error", "-i",
                 os.path.join(capture_dir, "rgb.mp4"),
                 "-vf", f"select=eq(n\\,{f})", "-vsync", "vfr", "-frames:v", "1",
                 "-qscale:v", "2", "-y", out], check=True)


def sample(rgb, u, v, w, h):
    ur = np.clip(np.round(u).astype(int), 0, w - 1)
    vr = np.clip(np.round(v).astype(int), 0, h - 1)
    return rgb[vr, ur].mean(axis=-1)[:, None]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture_dir")
    args = ap.parse_args()
    cap = args.capture_dir

    odo = load_odometry(cap)
    R_all = quat_to_R(odo["q"])
    t_all = odo["t"]
    n = len(odo["frames"])

    pairs = [(0, 25), (n // 4, n // 4 + 25), (n // 2, n // 2 + 25)]
    frames_needed = sorted({f for p in pairs for f in p})
    decode_frames(cap, frames_needed)

    h, w = 1440, 1920
    cx_r, cy_r = w / 2, h / 2

    # precompute per-pair point correspondences (scale-independent)
    corr = []
    for i, j in pairs:
        di, dj = load_depth(cap, i), load_depth(cap, j)
        u, v = np.meshgrid(np.arange(DEPTH_W), np.arange(DEPTH_H))
        valid = (~np.isnan(di)) & (di > 0.3)
        d = di[valid]
        uu, vv = u[valid], v[valid]
        Ki, Kj = odo["K"][i], odo["K"][j]
        pts = np.stack([(uu - Ki[2]) / Ki[0] * d, (vv - Ki[3]) / Ki[1] * d, d], axis=-1)
        world = pts @ R_all[i].T + t_all[i]
        cam_j = (world - t_all[j]) @ R_all[j]
        z = cam_j[:, 2]
        ok = z > 0.3
        uj = Kj[0] * cam_j[ok, 0] / z[ok] + Kj[2]
        vj = Kj[1] * cam_j[ok, 1] / z[ok] + Kj[3]
        inb = (uj > 2) & (uj < DEPTH_W - 2) & (vj > 2) & (vj < DEPTH_H - 2)
        d_ok = load_depth(cap, j)[
            np.clip(np.round(vj[inb]).astype(int), 0, DEPTH_H - 1),
            np.clip(np.round(uj[inb]).astype(int), 0, DEPTH_W - 1)]
        keep = ~np.isnan(d_ok)
        # subsample for speed
        sel = np.random.default_rng(0).choice(keep.sum(), min(15000, keep.sum()), replace=False)
        idx = np.where(keep)[0][sel]
        corr.append({
            "i": i, "j": j,
            "u_i": uu[idx], "v_i": vv[idx],
            "u_j": uj[inb][idx], "v_j": vj[inb][idx],
        })

    rgb_cache = {}
    def get_rgb(f):
        if f not in rgb_cache:
            rgb_cache[f] = np.asarray(
                Image.open(os.path.join(FRAMES_DIR, f"{f:06d}.jpg")), dtype=np.float64)
        return rgb_cache[f]

    def score(sx, sy):
        cols = []
        for c in corr:
            ri, rj = get_rgb(c["i"]), get_rgb(c["j"])
            # depth pixel -> RGB pixel: centred linear map
            ui = cx_r + (c["u_i"] - DEPTH_W / 2 + 0.5) * sx
            vi = cy_r + (c["v_i"] - DEPTH_H / 2 + 0.5) * sy
            uj = cx_r + (c["u_j"] - DEPTH_W / 2 + 0.5) * sx
            vj = cy_r + (c["v_j"] - DEPTH_H / 2 + 0.5) * sy
            a = sample(ri, ui, vi, w, h).ravel()
            b = sample(rj, uj, vj, w, h).ravel()
            if len(a) < 100:
                continue
            if a.std() < 1e-6 or b.std() < 1e-6:
                continue
            cols.append(np.corrcoef(a, b)[0, 1])
        return float(np.mean(cols)) if cols else -1.0

    nominal = w / DEPTH_W
    print(f"nominal scale (same FOV): {nominal:.3f}")
    best = (None, -2)
    for sx in np.arange(6.0, 7.8, 0.2):
        for sy in np.arange(6.0, 7.8, 0.2):
            s = score(sx, sy)
            if s > best[1]:
                best = ((sx, sy), s)
    print(f"coarse best sx={best[0][0]:.2f} sy={best[0][1]:.2f} corr={best[1]:.4f}")
    sx0, sy0 = best[0]
    for sx in np.arange(sx0 - 0.25, sx0 + 0.25, 0.05):
        for sy in np.arange(sy0 - 0.25, sy0 + 0.25, 0.05):
            s = score(sx, sy)
            if s > best[1]:
                best = ((sx, sy), s)
    print(f"fine   best sx={best[0][0]:.2f} sy={best[0][1]:.2f} corr={best[1]:.4f}")
    print(f"corr at nominal {nominal:.2f}: {score(nominal, nominal):.4f}")


if __name__ == "__main__":
    main()
