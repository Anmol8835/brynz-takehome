"""Floor orthomosaic + heuristic damage detection.

The sample captures are floor-focused (camera pitched 17-42 degrees down for
the whole walk), so the observable surface is the floor. We orthorectify the
RGB stream onto the floor plane to build a top-down mosaic, then detect:

  - cracks: dark, elongated structures vs the local background
  - stains: local chroma outliers
  - concealed-damage flags: coverage gaps inside the scanned area

All extents are metric (world units), with a confidence interval derived from
the mosaic cell size and the detection thresholds (reported, not guessed).
The method is deliberately simple and fully local (no pretrained weights);
this is disclosed in the technical report.
"""
import os
import shutil
import subprocess

import numpy as np
from PIL import Image
from scipy import ndimage

from lib.io import DEPTH_H, DEPTH_W, load_confidence, load_depth, load_odometry, quat_to_R

RGB_W, RGB_H = 960, 720
U_SCALE, V_SCALE = RGB_W / DEPTH_W, RGB_H / DEPTH_H


def local_plane_offset(s, band=0.15, min_pts=400, max_off=0.12, dominance=2.5,
                       bins=30):
    """Per-frame modal offset of a surface relative to a reference plane.

    `s` is signed distance of the frame's points to the plane. The floor fit
    can sit at the bottom of the surface's noise band and the reconstruction
    warps slightly per frame, so mosaics anchor on the modal surface mass.
    Returns 0.0 when no dominant planar spike exists (surface not in view).
    """
    sel = s[np.abs(s) < band]
    if sel.size < min_pts:
        return 0.0
    hist, edges = np.histogram(sel, bins=bins, range=(-band, band))
    peak = int(hist.max())
    med = float(np.median(hist))
    if peak < dominance * max(med, 1.0):
        return 0.0  # no dominant spike (e.g. only walls/uniform noise in band)
    o = float(edges[int(hist.argmax())] + (edges[1] - edges[0]) / 2)
    if abs(o) > max_off:
        return 0.0
    return o


def extract_rgb_frames(capture_dir, out_dir, stride):
    """Decode every stride-th frame of rgb.mp4 at 960x720 to out_dir.

    Output files are numbered 1..N in extraction order; source frame
    k (0-based) = (output_index - 1) * stride.
    """
    os.makedirs(out_dir, exist_ok=True)
    cmd = ["ffmpeg", "-loglevel", "error", "-i",
           os.path.join(capture_dir, "rgb.mp4"),
           "-vf", f"select=not(mod(n\\,{stride})),scale={RGB_W}:{RGB_H}:flags=area",
           "-vsync", "vfr", "-qscale:v", "3", os.path.join(out_dir, "%06d.jpg")]
    subprocess.run(cmd, check=True)


def build_floor_mosaic(capture_dir, odo, floor_n, floor_d, stride,
                       cell=0.005, conf_min=1, max_frames=450, frames_dir=None):
    """Orthorectify RGB onto the floor plane -> top-down mosaic.

    Returns dict: color (nx,nz,3) float 0-255, coverage (nx,nz) bool,
    origin (x0,z0) world coords of cell (0,0), cell (m).

    Pass frames_dir (pre-decoded by extract_rgb_frames) to reuse one decode
    across several surfaces; otherwise frames are decoded and removed here.
    """
    n_total = len(odo["frames"])
    if n_total // stride > max_frames:
        stride = int(np.ceil(n_total / max_frames))
    R_all = quat_to_R(odo["q"])
    t_all = odo["t"]

    pad = 2.0
    xmin = float(t_all[:, 0].min() - pad)
    xmax = float(t_all[:, 0].max() + pad)
    zmin = float(t_all[:, 2].min() - pad)
    zmax = float(t_all[:, 2].max() + pad)
    nx = int(np.ceil((xmax - xmin) / cell))
    nz = int(np.ceil((zmax - zmin) / cell))

    color = np.zeros((nx, nz, 3), dtype=np.float64)
    counts = np.zeros((nx, nz), dtype=np.int32)

    own_tmp = frames_dir is None
    tmp = frames_dir or f"/tmp/mosaic_frames_{os.path.basename(capture_dir)}"
    if own_tmp:
        extract_rgb_frames(capture_dir, tmp, stride)
    frame_files = sorted(f for f in os.listdir(tmp) if f.endswith(".jpg"))
    frame_lookup = {int(f): i for i, f in enumerate(odo["frames"])}
    offsets = []

    for fname in frame_files:
        out_idx = int(fname.split(".")[0])
        src_frame = (out_idx - 1) * stride
        row = frame_lookup.get(src_frame)
        if row is None:
            continue
        depth = load_depth(capture_dir, src_frame)
        conf = load_confidence(capture_dir, src_frame)
        rgb = np.asarray(Image.open(os.path.join(tmp, fname)), dtype=np.float64)
        # scale from actual decoded size (robust to decoder settings)
        rgb_h, rgb_w = rgb.shape[:2]
        u_scale, v_scale = rgb_w / DEPTH_W, rgb_h / DEPTH_H
        K = odo["K"][row]
        fx, fy, cx, cy = K[0], K[1], K[2], K[3]
        v, u = np.mgrid[0:DEPTH_H, 0:DEPTH_W]
        valid = (~np.isnan(depth)) & (conf >= conf_min) & (depth > 0.15)
        d = depth[valid]
        uu = u[valid]
        vv = v[valid]
        pts = np.stack([(uu - cx) / fx * d, (vv - cy) / fy * d, d], axis=-1)
        world = pts @ R_all[row].T + t_all[row]
        s_all = world @ floor_n + floor_d
        # plane-anchored correction: drift + fit bias vary per frame
        o = local_plane_offset(s_all)
        offsets.append(o)
        on_floor = np.abs(s_all - o) < 0.035
        world = world[on_floor]
        uu = uu[on_floor]
        vv = vv[on_floor]
        ur = np.clip(np.round(uu * u_scale).astype(int), 0, rgb_w - 1)
        vr = np.clip(np.round(vv * v_scale).astype(int), 0, rgb_h - 1)
        cols = rgb[vr, ur]
        ix = np.floor((world[:, 0] - xmin) / cell).astype(int)
        iz = np.floor((world[:, 2] - zmin) / cell).astype(int)
        keep = (ix >= 0) & (ix < nx) & (iz >= 0) & (iz < nz)
        ix, iz, cols = ix[keep], iz[keep], cols[keep]
        np.add.at(color, (ix, iz), cols)
        np.add.at(counts, (ix, iz), 1)

    if own_tmp:
        shutil.rmtree(tmp, ignore_errors=True)
    mean = np.zeros_like(color)
    ok = counts > 0
    mean[ok] = color[ok] / counts[ok][:, None]
    return {"color": mean, "coverage": ok, "origin": np.array([xmin, zmin]),
            "cell": cell, "counts": counts,
            "frame_offsets": offsets}


def _elongated_components(mask, min_len_cells, min_ratio=3.0):
    """Dark mask -> crack candidates via PCA elongation (cell units)."""
    lab, n = ndimage.label(mask)
    out = []
    for i in range(1, n + 1):
        ys, xs = np.where(lab == i)
        if len(xs) < 8:
            continue
        pts = np.stack([xs, ys], axis=1).astype(float)
        c = pts.mean(axis=0)
        vals = np.linalg.eigvalsh(np.cov(pts.T))
        length = 4 * np.sqrt(max(vals))
        width = 4 * np.sqrt(min(vals))
        if length < min_len_cells or length / (width + 1e-9) < min_ratio:
            continue
        out.append({
            "class": "crack",
            "center_cell": [float(c[0]), float(c[1])],
            "length_cells": float(length),
            "width_cells": float(width),
            "pixel_count": len(xs),
        })
    return out


def _blobs(mask, min_area_cells):
    lab, n = ndimage.label(mask)
    out = []
    for i in range(1, n + 1):
        ys, xs = np.where(lab == i)
        if len(xs) < min_area_cells:
            continue
        out.append({
            "class": "stain",
            "center_cell": [float(xs.mean()), float(ys.mean())],
            "area_cells": float(len(xs)),
            "pixel_count": len(xs),
        })
    return out


def detect_damage(mosaic):
    """Heuristic damage detection on the mosaic. All outputs in world metres."""
    cell = mosaic["cell"]
    origin = mosaic["origin"]
    color = mosaic["color"]
    cov = mosaic["coverage"]
    rgb = color / 255.0
    gray = rgb.mean(axis=2)
    r = max(2, int(round(0.10 / cell)))  # 10 cm local background
    bg = ndimage.uniform_filter(gray, size=2 * r + 1, mode="nearest")
    dark = (bg - gray) > 0.15
    cracks = _elongated_components(dark & cov, min_len_cells=0.03 / cell)

    bgr = ndimage.uniform_filter(rgb, size=(2 * r + 1, 2 * r + 1, 0), mode="nearest")
    dev = np.abs(rgb - bgr).max(axis=2)
    stains = _blobs((dev > 0.12) & cov & ~dark, min_area_cells=(0.02 / cell) ** 2)

    # concealed damage: unobserved holes inside the scanned region
    filled = ndimage.binary_fill_holes(cov)
    gaps = filled & ~cov
    gap_lab, gap_n = ndimage.label(gaps)
    flags = []
    for i in range(1, gap_n + 1):
        ys, xs = np.where(gap_lab == i)
        area = len(xs) * cell * cell
        if area < 2.5e-3:  # 25 cm^2
            continue
        flags.append({
            "rule": "coverage_gap_in_scanned_area",
            "rule_description": "no RGB/depth observation on the floor inside "
                                "the scanned region; damage cannot be ruled out",
            "center_m": [round(float(xs.mean() * cell + origin[0]), 3),
                         round(float(ys.mean() * cell + origin[1]), 3)],
            "area_m2": round(float(area), 4),
        })

    for c in cracks:
        c["center_m"] = [round(c["center_cell"][0] * cell + origin[0], 3),
                         round(c["center_cell"][1] * cell + origin[1], 3)]
        c["length_m"] = round(c["length_cells"] * cell, 3)
        c["width_m"] = round(c["width_cells"] * cell, 4)
        c["extent_ci_m"] = round(cell, 4)
        c.pop("center_cell"), c.pop("length_cells"), c.pop("width_cells")
    for s in stains:
        s["center_m"] = [round(s["center_cell"][0] * cell + origin[0], 3),
                         round(s["center_cell"][1] * cell + origin[1], 3)]
        s["area_m2"] = round(s["area_cells"] * cell * cell, 5)
        s["extent_ci_m"] = round(cell, 4)
        s.pop("center_cell"), s.pop("area_cells")

    return {"cracks": cracks, "stains": stains, "flags": flags,
            "coverage_fraction": round(float(cov.sum()) / cov.size, 4),
            "covered_area_m2": round(float(cov.sum()) * cell * cell, 3),
            "cell_m": cell, "extent_ci_m": round(cell, 4)}


def scope_items(damage):
    """Scope line items keyed to the floor surface."""
    items = []
    for reg in damage["cracks"]:
        items.append({"surface": "floor", "class": "crack",
                      "action": "inspect and seal crack",
                      "extent_m": reg["length_m"], "center_m": reg["center_m"],
                      "ci_m": reg["extent_ci_m"]})
    for reg in damage["stains"]:
        items.append({"surface": "floor", "class": "stain",
                      "action": "clean / assess stain",
                      "extent_m2": reg["area_m2"], "center_m": reg["center_m"],
                      "ci_m": reg["extent_ci_m"]})
    return items
