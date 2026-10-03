"""Depth fusion: batched backprojection into a world point cloud with
voxel downsampling. Memory-safe for ~5k frames (processes in chunks and
downsamples each chunk before merging)."""
import numpy as np

from lib.io import DEPTH_H, DEPTH_W, load_confidence, load_depth, load_odometry, quat_to_R

_u, _v = np.mgrid[0:DEPTH_H, 0:DEPTH_W].astype(np.float32)


def _frame_points(pose, depth, conf, conf_min):
    """Backproject one frame to camera space; return (pts, conf, mask)."""
    fx, fy, cx, cy = pose["K"]
    valid = (~np.isnan(depth)) & (conf >= conf_min) & (depth > 0.15)
    x = (_u - cx) / fx * depth
    y = (_v - cy) / fy * depth
    pts = np.stack([x, y, depth], axis=-1).reshape(-1, 3)
    return pts[valid.ravel()], conf.ravel()[valid.ravel()], valid


def _voxel_downsample(pts, voxel, values=None):
    """Mean point (and mean `values`) per voxel cell. voxel in metres."""
    idx = np.floor(pts / voxel).astype(np.int64)
    imin = idx.min(axis=0)
    idx = idx - imin
    dims = idx.max(axis=0) + 2
    key = np.ravel_multi_index((idx[:, 0], idx[:, 1], idx[:, 2]), dims)
    uniq, inv = np.unique(key, return_inverse=True)
    out = np.empty((len(uniq), 3), dtype=np.float64)
    np.add.at(out, inv, pts)
    counts = np.bincount(inv, minlength=len(uniq))
    out /= counts[:, None]
    if values is None:
        return out
    vout = np.zeros(len(uniq), dtype=np.float64)
    np.add.at(vout, inv, values)
    return out, vout / counts


def build_cloud(capture_dir, odometry_scale=1.0, stride=2, conf_min=1,
                voxel=0.01, chunk=60, max_points=6_000_000):
    """Fuse depth frames into a world point cloud.

    Returns dict with 'points' (M,3) world metres and 'conf' (M,) mean
    confidence. Frames are decimated by `stride`; chunks of `chunk` frames
    are fused and voxel-downsampled, then a final global downsample caps
    memory.
    """
    odo = load_odometry(capture_dir)
    R_all = quat_to_R(odo["q"])
    t_all = odo["t"] * odometry_scale
    frames = odo["frames"][::stride]

    chunks = []
    for start in range(0, len(frames), chunk):
        pts_c, conf_c = [], []
        for f in frames[start : start + chunk]:
            depth = load_depth(capture_dir, int(f))
            conf = load_confidence(capture_dir, int(f))
            idx = np.searchsorted(odo["frames"], f)
            pose = {"K": odo["K"][idx]}
            pts, cvals, _ = _frame_points(pose, depth, conf, conf_min)
            if len(pts) == 0:
                continue
            world = pts @ R_all[idx].T + t_all[idx]
            pts_c.append(world)
            conf_c.append(cvals)
        if pts_c:
            merged = np.concatenate(pts_c)
            confs = np.concatenate(conf_c)
            ds, cout = _voxel_downsample(merged, voxel, values=confs)
            chunks.append((ds, cout))
            del merged, confs

    pts = np.concatenate([c[0] for c in chunks])
    conf = np.concatenate([c[1] for c in chunks])
    if len(pts) > max_points:
        sel = np.random.default_rng(0).choice(len(pts), max_points, replace=False)
        pts, conf = pts[sel], conf[sel]
    return {"points": pts, "conf": conf, "frames_used": len(frames)}
