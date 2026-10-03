"""Loaders for the Record3D-style capture format.

Layout per capture directory:
    odometry.csv       per-frame timestamp, frame, x,y,z, qx,qy,qz,qw, fx,fy,cx,cy
    camera_matrix.csv  K for the depth camera (reference only)
    imu.csv            accelerometer + gyroscope (currently unused)
    rgb.mp4            colour video, one frame per depth frame
    depth/??????.png   256x192 uint16, millimetres, 0 = invalid
    confidence/??????.png  256x192 uint8, 0 = lowest confidence
"""
import array
import csv
import os

import numpy as np
from PIL import Image

DEPTH_W, DEPTH_H = 256, 192
DEPTH_SCALE = 0.001  # mm -> m

# odometry.csv intrinsics refer to the native RGB sensor (1920x1440); the
# depth frames are a 7.5x downscale of the same FOV, so K must be rescaled.
RGB_W, RGB_H = 1920, 1440
K_SCALE_X = DEPTH_W / RGB_W
K_SCALE_Y = DEPTH_H / RGB_H


def load_odometry(capture_dir):
    """Return dict with keys:
    frames  (N,) int
    t       (N,3) float  camera position, metres
    q       (N,4) float  quaternion (x,y,z,w), camera->world
    K       (N,4) float  per-frame fx,fy,cx,cy of the DEPTH camera
                         (native-resolution values rescaled to 256x192)
    """
    frames, t, q, K = [], [], [], []
    with open(os.path.join(capture_dir, "odometry.csv")) as fh:
        for row in csv.DictReader(fh, skipinitialspace=True):
            frames.append(int(row["frame"]))
            t.append((float(row["x"]), float(row["y"]), float(row["z"])))
            q.append((float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])))
            K.append((float(row["fx"]) * K_SCALE_X, float(row["fy"]) * K_SCALE_Y,
                      float(row["cx"]) * K_SCALE_X, float(row["cy"]) * K_SCALE_Y))
    return {
        "frames": np.asarray(frames, dtype=np.int64),
        "t": np.asarray(t, dtype=np.float64),
        "q": np.asarray(q, dtype=np.float64),
        "K": np.asarray(K, dtype=np.float64),
    }


def quat_to_R(q):
    """Quaternions (N,4) x,y,z,w -> rotation matrices (N,3,3), row-major,
    such that world = R @ cam + t."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    xw, yw, zw = x * w, y * w, z * w
    R = np.empty((q.shape[0], 3, 3), dtype=np.float64)
    R[:, 0, 0] = 1 - 2 * (yy + zz)
    R[:, 0, 1] = 2 * (xy - zw)
    R[:, 0, 2] = 2 * (xz + yw)
    R[:, 1, 0] = 2 * (xy + zw)
    R[:, 1, 1] = 1 - 2 * (xx + zz)
    R[:, 1, 2] = 2 * (yz - xw)
    R[:, 2, 0] = 2 * (xz - yw)
    R[:, 2, 1] = 2 * (yz + xw)
    R[:, 2, 2] = 1 - 2 * (xx + yy)
    return R


def load_depth(capture_dir, frame):
    """Depth image (256,192) float metres; 0 -> nan."""
    path = os.path.join(capture_dir, "depth", f"{frame:06d}.png")
    im = Image.open(path)
    if im.mode == "I;16":
        raw = np.asarray(im, dtype=np.uint16)
    elif im.mode == "I":
        raw = np.asarray(im, dtype=np.int32).astype(np.uint32)
    else:
        raise ValueError(f"unexpected depth mode {im.mode} in {path}")
    d = raw.astype(np.float32) * DEPTH_SCALE
    d[raw == 0] = np.nan
    return d


def load_confidence(capture_dir, frame):
    path = os.path.join(capture_dir, "confidence", f"{frame:06d}.png")
    return np.asarray(Image.open(path), dtype=np.uint8)
