"""Probe the sample data format: depth scale, odometry consistency, physical sanity.

Answers three questions:
1. Are odometry poses consistent with the depth stream? (static-scene reprojection)
2. Is depth encoded in millimetres? (physical camera-height test)
3. What is the overall depth distribution?

Run:  python scripts/probe_depth_scale.py <capture_dir>
"""
import argparse
import array
import csv
import math
import sys

from PIL import Image

W, H = 256, 192


def load_depth(path):
    im = Image.open(path)
    if im.mode == "I;16":
        return array.array("H", im.tobytes())
    if im.mode == "I":
        return array.array("i", im.tobytes())
    raise AssertionError(f"unexpected depth mode {im.mode}")


def quat_to_R(q):
    qx, qy, qz, qw = q
    return (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )


def transpose(R):
    return tuple(tuple(R[r][c] for r in range(3)) for c in range(3))


def load_poses(capture_dir):
    poses = {}
    with open(f"{capture_dir}/odometry.csv") as fh:
        for row in csv.DictReader(fh, skipinitialspace=True):
            f = int(row["frame"])
            # odometry intrinsics are native-res (1920x1440); depth is 256x192
            poses[f] = {
                "t": (float(row["x"]), float(row["y"]), float(row["z"])),
                "q": (float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])),
                "fx": float(row["fx"]) * 256 / 1920,
                "fy": float(row["fy"]) * 192 / 1440,
                "cx": float(row["cx"]) * 256 / 1920,
                "cy": float(row["cy"]) * 192 / 1440,
            }
    return poses


def apply(R, v):
    return tuple(R[r][0] * v[0] + R[r][1] * v[1] + R[r][2] * v[2] for r in range(3))


def reprojection_check(capture_dir, poses, i, j):
    """Backproject frame i, transform into frame j, reproject, compare depth."""
    di = load_depth(f"{capture_dir}/depth/{i:06d}.png")
    dj = load_depth(f"{capture_dir}/depth/{j:06d}.png")
    Ri = quat_to_R(poses[i]["q"])
    Rjt = transpose(quat_to_R(poses[j]["q"]))
    ti, tj = poses[i]["t"], poses[j]["t"]
    kx, ky, cx, cy = poses[i]["fx"], poses[i]["fy"], poses[i]["cx"], poses[i]["cy"]
    jx, jy, jcx, jcy = poses[j]["fx"], poses[j]["fy"], poses[j]["cx"], poses[j]["cy"]

    ratios = []
    for v in range(H):
        for u in range(W):
            d = di[v * W + u]
            if d <= 0:
                continue
            # depth PNG -> metres (assumed mm; ratio test is scale-invariant anyway)
            P = ((u - cx) / kx * d, (v - cy) / ky * d, d)
            X = apply(Ri, P)
            X = (X[0] + ti[0], X[1] + ti[1], X[2] + ti[2])
            Pj = apply(Rjt, (X[0] - tj[0], X[1] - tj[1], X[2] - tj[2]))
            if Pj[2] <= 0:
                continue
            uu, vv = jx * Pj[0] / Pj[2] + jcx, jy * Pj[1] / Pj[2] + jcy
            ui, vi = int(round(uu)), int(round(vv))
            if 0 <= ui < W and 0 <= vi < H:
                d2 = dj[vi * W + ui]
                if d2 > 0:
                    ratios.append(d2 / Pj[2])
    ratios.sort()
    n = len(ratios)
    return n, ratios[n // 2], ratios[n // 10], ratios[9 * n // 10]


def physical_check(capture_dir, poses, frames):
    """Median world-y of bottom-of-frame points -> camera height above floor.

    Physical only if depth is in mm: floor should sit ~1.2-1.8 m below the
    camera. If depth were metres, this lands ~1500 m away. Also computes the
    ceiling offset for a room-height sanity check.
    """
    floor_ys, ceil_ys = [], []
    for f in frames:
        d = load_depth(f"{capture_dir}/depth/{f:06d}.png")
        R = quat_to_R(poses[f]["q"])
        t = poses[f]["t"]
        fx, fy, cx, cy = poses[f]["fx"], poses[f]["fy"], poses[f]["cx"], poses[f]["cy"]
        ys = []
        for v in range(H):
            for u in range(W):
                z = d[v * W + u]
                if z <= 0:
                    continue
                # mm -> m
                P = ((u - cx) / fx * z * 0.001, (v - cy) / fy * z * 0.001, z * 0.001)
                ys.append(apply(R, P)[1] + t[1])
        ys.sort()
        n = len(ys)
        floor_ys.append(ys[n // 20])  # ~5th percentile
        ceil_ys.append(ys[19 * n // 20])  # ~95th percentile
    floor_ys.sort()
    ceil_ys.sort()
    m = len(floor_ys)
    return floor_ys[m // 2], ceil_ys[m // 2]


def depth_stats(capture_dir, poses):
    vals = []
    for f in sorted(poses)[::25]:
        d = load_depth(f"{capture_dir}/depth/{f:06d}.png")
        vals.extend(x for x in d if x > 0)
    vals.sort()
    n = len(vals)
    return vals[n // 100], vals[n // 2], vals[99 * n // 100]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture_dir")
    args = ap.parse_args()
    poses = load_poses(args.capture_dir)
    frames = sorted(poses)
    print(f"frames: {len(frames)}")

    i, j = frames[0], frames[min(40, len(frames) - 1)]
    n, med, p10, p90 = reprojection_check(args.capture_dir, poses, i, j)
    print(f"reprojection {i}->{j}: n={n} median_ratio={med:.4f} (p10={p10:.4f} p90={p90:.4f})")
    print("  ratio ~1 => odometry and depth describe the same 3D scene")

    floor_y, ceil_y = physical_check(args.capture_dir, poses, frames[::50])
    print(f"camera->floor = {-floor_y:.2f} m, camera->ceiling = {ceil_y:.2f} m "
          f"(room height ~{ceil_y - floor_y:.2f} m at mm scale)")
    print("  floor at ~1.2-1.8 m => depth is mm; floor at ~1500 m => depth is metres")

    p1, p50, p99 = depth_stats(args.capture_dir, poses)
    print(f"depth distribution: p1={p1} p50={p50} p99={p99} (raw units)")


if __name__ == "__main__":
    main()
