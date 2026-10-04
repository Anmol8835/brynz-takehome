"""Independent ceiling verification for the with-ceiling capture.

Conflicting probe results motivated this: the same capture produced ceiling
peaks at ~1.8m, ~2.0m and ~2.5m depending on probe method (raw frames vs
fused cloud, scaled vs unscaled odometry). This script compares raw-frame
and fused-cloud height distributions on IDENTICAL bins and reports which
horizontal structures are real.

    python scripts/verify_ceiling.py
"""
import sys

import numpy as np

sys.path.insert(0, ".")
from lib.io import load_confidence, load_depth, load_odometry, quat_to_R
from lib.reconstruct import build_cloud

CAP = "/home/anmol/Downloads/single_scan_with_ceiling/c7d28f72c6"
K_SCALE = 1.0605
N_PLANE = np.array([-0.004, 1.0, -0.01])  # pipeline floor plane normal
D_PLANE = 1.08                            # pipeline floor plane offset


def raw_heights(cap, k, n_rows=200):
    odo = load_odometry(cap)
    R = quat_to_R(odo["q"])
    t = odo["t"] * k
    rows = np.linspace(0, len(odo["frames"]) - 1, n_rows).astype(int)
    s_all = []
    for row in rows:
        f = int(odo["frames"][row])
        dep = load_depth(cap, f)
        conf = load_confidence(cap, f)
        u, v = np.meshgrid(np.arange(256), np.arange(192))
        valid = (~np.isnan(dep)) & (conf >= 1) & (dep > 0.15)
        dd, uu, vv = dep[valid], u[valid], v[valid]
        K = odo["K"][row]
        pts = np.stack([(uu - K[2]) / K[0] * dd,
                        (vv - K[3]) / K[1] * dd, dd], axis=-1)
        w = pts @ R[row].T + t[row]
        s_all.append(w @ N_PLANE + D_PLANE)
    return np.concatenate(s_all)


def top_bins(s, lo, hi, n_bins, label, top=8):
    band = s[(s > lo) & (s < hi)]
    hist, edges = np.histogram(band, bins=n_bins, range=(lo, hi))
    width = (hi - lo) / n_bins
    print(f"{label}: band n={len(band)} ({100*len(band)/len(s):.1f}% of points)")
    for i in np.argsort(hist)[-top:][::-1]:
        print(f"   {edges[i]:.2f}-{edges[i]+width:.2f} m : {100*hist[i]/len(band):5.2f}% of band")
    return hist, edges


def mass_frac(s, lo, hi):
    return 100 * ((s > lo) & (s < hi)).mean()


def main():
    s_raw = raw_heights(CAP, K_SCALE)
    print(f"raw frames: {len(s_raw)} points (240 sampled frames, k={K_SCALE})")
    cloud = build_cloud(CAP, odometry_scale=K_SCALE, stride=3)
    s_fus = cloud["points"] @ N_PLANE + D_PLANE
    print(f"fused cloud: {len(s_fus)} points (stride 3, same k)")

    print("\n--- height-above-floor-plane histograms (1 cm bins) ---")
    h_raw, e_raw = top_bins(s_raw, 0.0, 3.5, 350, "RAW frames")
    print()
    h_fus, e_fus = top_bins(s_fus, 0.0, 3.5, 350, "FUSED cloud")

    print("\n--- mass fractions (%% of points) ---")
    for lo, hi, tag in [(0.0, 0.3, "floor zone"),
                        (0.3, 1.5, "waist zone"),
                        (1.5, 2.0, "pre-ceiling"),
                        (2.0, 3.2, "ceiling zone")]:
        print(f"  {tag:12s} {lo:.1f}-{hi:.1f}m : raw {mass_frac(s_raw, lo, hi):5.1f}%"
              f"   fused {mass_frac(s_fus, lo, hi):5.1f}%")


if __name__ == "__main__":
    main()
