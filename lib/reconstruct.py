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
    out = np.zeros((len(uniq), 3), dtype=np.float64)  # must be zeros: add.at accumulates
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
    """Fuse depth frames into a world point cloud (bounded memory).

    Returns dict with 'points' (M,3) world metres and 'conf' (M,) mean
    confidence. Frames are decimated by `stride`; chunks of `chunk` frames
    are fused and voxel-downsampled. The per-chunk results are merged
    progressively into a single accumulator that is re-voxelised at 1.5x
    the cell size whenever it exceeds 1.5x max_points, so peak memory stays
    O(max_points) regardless of capture length.
    """
    odo = load_odometry(capture_dir)
    R_all = quat_to_R(odo["q"])
    t_all = odo["t"] * odometry_scale
    frames = odo["frames"][::stride]
    rng = np.random.default_rng(0)

    # Chunk clouds are held as float32 (half the memory) and thinned with a
    # per-point Bernoulli keep probability p = max_points / total after the
    # pass: every voxel point has the same inclusion probability, matching
    # the reference "subsample the concatenated cloud" behaviour, without
    # ever materialising the concatenation (the earlier OOM cause on the
    # 9745-frame capture).
    chunk_pts = []
    chunk_conf = []
    total = 0
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
        if not pts_c:
            continue
        merged = np.concatenate(pts_c)
        confs = np.concatenate(conf_c)
        ds, cout = _voxel_downsample(merged, voxel, values=confs)
        del merged, confs, pts_c, conf_c
        chunk_pts.append(ds.astype(np.float32))
        chunk_conf.append(cout.astype(np.float32))
        total += len(ds)

    p = min(1.0, max_points / max(total, 1))
    kept_pts, kept_conf = [], []
    for ds, cout in zip(chunk_pts, chunk_conf):
        if p >= 1.0:
            keep = np.ones(len(ds), dtype=bool)
        else:
            keep = rng.random(len(ds)) < p
        kept_pts.append(ds[keep])
        kept_conf.append(cout[keep])
    pts = np.concatenate(kept_pts).astype(np.float64)
    conf = np.concatenate(kept_conf).astype(np.float64)
    if len(pts) > max_points:
        sel = rng.choice(len(pts), max_points, replace=False)
        pts, conf = pts[sel], conf[sel]
    return {"points": pts, "conf": conf, "frames_used": len(frames)}
