"""Visual check: project depth points into the RGB frame and render an overlay.

If the mapping is right, the projected depth skeleton lands on the same
edges/objects visible in the RGB image.

    python scripts/check_rgb_overlay.py <capture_dir> <frame> <out.png>
"""
import argparse
import os
import subprocess
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.io import DEPTH_H, DEPTH_W, load_depth, load_odometry, quat_to_R

RGB_W, RGB_H = 1920, 1440


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture_dir")
    ap.add_argument("frame", type=int)
    ap.add_argument("out")
    args = ap.parse_args()

    tmp = "/tmp/rgb_overlay_frame.jpg"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-i",
                    os.path.join(args.capture_dir, "rgb.mp4"),
                    "-vf", f"select=eq(n\\,{args.frame})", "-vsync", "vfr",
                    "-frames:v", "1", "-qscale:v", "2", "-y", tmp], check=True)
    rgb = np.asarray(Image.open(tmp).convert("RGB"), dtype=np.uint8)

    depth = load_depth(args.capture_dir, args.frame)
    odo = load_odometry(args.capture_dir)
    row = int(np.searchsorted(odo["frames"], args.frame))
    K = odo["K"][row]
    fx, fy, cx, cy = K

    u, v = np.meshgrid(np.arange(DEPTH_W), np.arange(DEPTH_H))
    valid = (~np.isnan(depth)) & (depth > 0.3)
    d = depth[valid]
    uu, vv = u[valid], v[valid]

    # depth pixel -> native RGB pixel: exact affine from the intrinsics
    sx, sy = RGB_W / DEPTH_W, RGB_H / DEPTH_H
    ur = (uu - cx) * sx + RGB_W / 2
    vr = (vv - cy) * sy + RGB_H / 2
    keep = (ur > 0) & (ur < RGB_W - 1) & (vr > 0) & (vr < RGB_H - 1)
    ur, vr, d = ur[keep], vr[keep], d[keep]

    img = rgb.copy()
    # colour dots by depth (near=red, far=blue)
    t = np.clip((d - d.min()) / (d.max() - d.min() + 1e-9), 0, 1)
    colors = np.stack([(1 - t) * 255, np.zeros_like(t), t * 255], axis=1).astype(np.uint8)
    # scatter as 2x2 blocks for visibility
    for du in (0, 1):
        for dv in (0, 1):
            xs = np.clip(ur.astype(int) + du, 0, RGB_W - 1)
            ys = np.clip(vr.astype(int) + dv, 0, RGB_H - 1)
            img[ys, xs] = colors

    out = Image.fromarray(img)
    out.thumbnail((900, 700))
    out.save(args.out.replace(".png", ".jpg"), quality=88)
    print(f"saved {args.out.replace('.png', '.jpg')} ({out.size})")


if __name__ == "__main__":
    main()
