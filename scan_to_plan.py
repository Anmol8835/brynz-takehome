#!/usr/bin/env python3
"""One command per capture: dimensioned room plan from a LiDAR scan.

    python scan_to_plan.py <capture_dir> [--out OUT_DIR]

Stages: odometry<->depth scale calibration -> depth fusion -> floor/ceiling
plane recovery -> 2D wall extraction with Manhattan snapping -> measurements
with confidence intervals -> plan.json + plan.svg.

Output contract fields (see plan.json): room polygon (wall segments with
length + CI), ceiling height + CI, floor area + CI, openings, per-surface
damage regions, concealed-damage flags with firing rule, scope line items.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
from scipy import ndimage

from lib.calibrate import fit_translation_scale
from lib.io import load_odometry
from lib.reconstruct import build_cloud

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def ransac_plane(pts, dist=0.02, iters=400, samples=50_000):
    """Fit a plane (normal n, |n|=1, n·p + d = 0) to the largest consensus set."""
    rng = np.random.default_rng(0)
    if len(pts) > samples:
        work = pts[rng.choice(len(pts), samples, replace=False)]
    else:
        work = pts
    best = None
    for _ in range(iters):
        tri = work[rng.choice(len(work), 3, replace=False)]
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        d = -float(n @ tri[0])
        dev = np.abs(work @ n + d)
        inliers = int((dev < dist).sum())
        if best is None or inliers > best[0]:
            best = (inliers, n, d)
    _, n, d = best
    dev = np.abs(pts @ n + d)
    mask = dev < dist
    return {"normal": n, "d": d, "inliers": int(mask.sum()), "mask": mask, "rms": float(np.sqrt((dev[mask] ** 2).mean()))}


def ransac_planes_sequential(pts, n_planes=4, dist=0.02, iters=400):
    """Repeatedly fit the dominant plane and remove its inliers. Each
    returned plane's 'mask' is a boolean over the ORIGINAL pts."""
    rest_idx = np.arange(len(pts))
    planes = []
    for _ in range(n_planes):
        if len(rest_idx) < 1000:
            break
        p = ransac_plane(pts[rest_idx], dist=dist, iters=iters)
        local_mask = p["mask"]  # over pts[rest_idx]
        full_mask = np.zeros(len(pts), dtype=bool)
        full_mask[rest_idx[local_mask]] = True
        p["mask"] = full_mask
        p["inliers"] = int(full_mask.sum())
        dev = np.abs(pts[full_mask] @ p["normal"] + p["d"])
        p["rms"] = float(np.sqrt((dev ** 2).mean()))
        planes.append(p)
        rest_idx = rest_idx[~local_mask]
    return planes


def pick_floor(planes, trajectory, gravity=(0.0, 1.0, 0.0)):
    """Among near-horizontal planes, pick the lowest one (largest offset from
    origin along the plane normal). Returns plane with normal flipped to point
    up, or None."""
    g = np.asarray(gravity)
    cands = []
    for p in planes:
        if abs(p["normal"] @ g) > 0.9:
            n = p["normal"]
            if n @ g < 0:
                n = -n
                d = -p["d"]
            else:
                d = p["d"]
            cands.append((d, {**p, "normal": n, "d": d}))
    if not cands:
        return None
    cands.sort(key=lambda c: -c[0])  # lowest plane first (n up: height = -d)
    return cands[0][1]


def ransac_floor_envelope(pts, frac=0.10, dist=0.02, iters=400):
    """Fit the floor as the plane through the BOTTOM envelope of the cloud.

    A floor-focused scan contains dense flat slabs above the floor (raised
    platforms, rug edges, furniture tops); the densest 2cm slab wins plain
    RANSAC, so we first isolate the lowest `frac` of points by world-y and
    fit the dominant plane there. Also returns the full-cloud inlier RMS,
    which measures how much the reconstructed floor has warped under
    odometry drift.
    """
    ycut = float(np.percentile(pts[:, 1], frac * 100))
    below = pts[pts[:, 1] <= ycut]
    if len(below) < 1000:
        return None
    p = ransac_plane(below, dist=dist, iters=iters)
    n = p["normal"]
    if n[1] < 0:
        n = -n
        p = {**p, "normal": n, "d": -p["d"]}
    dev = np.abs(pts @ n + p["d"])
    mask = dev < dist
    p["mask"] = mask
    p["inliers"] = int(mask.sum())
    p["rms"] = float(np.sqrt((dev[mask] ** 2).mean()))
    p["warp_rms"] = p["rms"]  # drift evidence: residual of the global floor fit
    p["y_cut"] = ycut
    return p


def transform_to_floor(pts, plane):
    """Rotate points so the floor normal becomes +z and floor plane z=0.
    Returns (xy_z_frame, R) with columns [x, y, height_above_floor]."""
    n = plane["normal"]
    z = np.array([0.0, 0.0, 1.0])
    a = np.cross(n, z)
    c = float(n @ z)
    if np.linalg.norm(a) < 1e-12:
        R = np.eye(3) * (1 if c > 0 else -1)
    else:
        s = np.linalg.norm(a)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        R = np.eye(3) + K + (K @ K) * ((1 - c) / s**2)
    pts_f = pts @ R.T
    # signed height above floor: n·p + d (plane defined as n·p + d = 0)
    pts_f[:, 2] = pts @ n + plane["d"]
    return pts_f, R


def density_grid(xy, cell=0.02):
    """2D occupancy counts on a grid; returns (counts, origin, cell)."""
    xmin, ymin = xy.min(axis=0) - cell
    xmax, ymax = xy.max(axis=0) + cell
    nx = int(np.ceil((xmax - xmin) / cell))
    ny = int(np.ceil((ymax - ymin) / cell))
    nx, ny = max(nx, 1), max(ny, 1)
    ix = np.floor((xy[:, 0] - xmin) / cell).astype(int)
    iy = np.floor((xy[:, 1] - ymin) / cell).astype(int)
    counts = np.zeros((nx, ny), dtype=np.int32)
    np.add.at(counts, (ix, iy), 1)
    return counts, np.array([xmin, ymin]), cell


def extract_boundary_pixels(counts, min_count=3):
    occ = counts >= min_count
    occ = ndimage.binary_closing(occ, iterations=2)
    occ = ndimage.binary_fill_holes(occ)
    eroded = ndimage.binary_erosion(occ, iterations=2)
    return occ, np.argwhere(occ & ~eroded)  # (K,2) in grid coords (ix,iy)


def ransac_lines_2d(pts, dist=0.04, iters=800, min_inliers=60):
    """2D line RANSAC on boundary pixels (in metres). Returns list of
    (direction theta, offset rho, inlier_count, inlier_mask) sorted by count."""
    rng = np.random.default_rng(1)
    n = len(pts)
    if n < 2:
        return []
    lines = []
    for _ in range(iters):
        a, b = pts[rng.choice(n, 2, replace=False)]
        dvec = b - a
        ln = np.linalg.norm(dvec)
        if ln < 1e-6:
            continue
        # normal form: n·p = rho, n = (nx, ny)
        nx_, ny_ = -dvec[1] / ln, dvec[0] / ln
        rho = nx_ * pts[:, 0] + ny_ * pts[:, 1]
        dev = np.abs(rho - (nx_ * a[0] + ny_ * a[1]))
        mask = dev < dist
        cnt = int(mask.sum())
        if cnt >= min_inliers:
            lines.append((cnt, nx_, ny_, float(nx_ * a[0] + ny_ * a[1]), mask))
    if not lines:
        return []
    lines.sort(key=lambda t: -t[0])
    # greedy dedupe: keep a line only if its direction/offset differ from kept ones
    kept = []
    for cnt, nx_, ny_, rho, mask in lines:
        theta = np.arctan2(ny_, nx_)
        dup = False
        for k in kept:
            dtheta = abs(np.arctan2(np.sin(theta - k["theta"]), np.cos(theta - k["theta"])))
            if dtheta < np.deg2rad(5) and abs(rho - k["rho"]) < 0.08:
                dup = True
                break
        if not dup:
            kept.append({"theta": theta, "nx": nx_, "ny": ny_, "rho": rho,
                         "count": cnt, "mask": mask})
            if len(kept) >= 8:
                break
    return kept


def manhattan_snap(lines, bin_deg=45):
    """Assign each line to one of two orthogonal directions; return the two
    dominant direction angles and lines grouped by direction."""
    if not lines:
        return None, None, [], []
    thetas = np.array([l["theta"] for l in lines])
    # histogram of directions in [0, pi)
    hist, edges = np.histogram(thetas % np.pi, bins=36, range=(0, np.pi))
    a0 = edges[hist.argmax()] + np.pi / 72
    a1 = (a0 + np.pi / 2) % np.pi
    g0 = [l for l in lines if abs(np.arctan2(np.sin(l["theta"] - a0), np.cos(l["theta"] - a0))) < np.deg2rad(22.5)]
    g1 = [l for l in lines if abs(np.arctan2(np.sin(l["theta"] - a1), np.cos(l["theta"] - a1))) < np.deg2rad(22.5)]
    return a0, a1, g0, g1


def line_intersection(l1, l2):
    """Intersection of two lines in normal form (nx, ny, rho)."""
    A = np.array([[l1["nx"], l1["ny"]], [l2["nx"], l2["ny"]]])
    b = np.array([l1["rho"], l2["rho"]])
    if abs(np.linalg.det(A)) < 1e-9:
        return None
    return np.linalg.solve(A, b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture_dir")
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-calibrate", action="store_true")
    ap.add_argument("--fusion-stride", type=int, default=2,
                    help="decimate depth frames by N during fusion (default 2)")
    ap.add_argument("--mosaic-cell", type=float, default=0.005,
                    help="mosaic cell size in metres (default 5 mm)")
    ap.add_argument("--max-surfaces", type=int, default=3,
                    help="damage mosaics for floor + N largest raised surfaces (default 3)")
    ap.add_argument("--fast", action="store_true",
                    help="sparser frames + 1cm cells + 1 raised surface (~3-4 min/capture)")
    args = ap.parse_args()
    if args.fast:
        args.fusion_stride = 3
        args.max_surfaces = 1
        args.mosaic_cell = 0.01

    t0 = time.time()
    capture = os.path.abspath(args.capture_dir)
    out = args.out or os.path.join(capture, "plan_output")
    os.makedirs(out, exist_ok=True)
    odo = load_odometry(capture)

    # --- calibration ---
    if args.no_calibrate:
        k, calib_report = 1.0, None
    else:
        k, calib_report = fit_translation_scale(capture)
    print(f"[calibrate] odometry translation scale k = {k:.4f}")

    # --- fusion ---
    cloud = build_cloud(capture, odometry_scale=k, stride=args.fusion_stride)
    pts = cloud["points"]
    print(f"[reconstruct] {len(pts)} points from {cloud['frames_used']} frames")

    # --- floor = bottom envelope of the cloud; other horizontal slabs are
    # raised surfaces above it (reported separately) ---
    floor = ransac_floor_envelope(pts)
    if floor is None:
        print("[floor] no floor plane found", file=sys.stderr)
        sys.exit(1)
    floor_n = floor["normal"]
    print(f"[floor] bottom-envelope fit: n={floor_n.round(3)} d={floor['d']:.2f} "
          f"y_cut={floor['y_cut']:.2f} inliers={floor['inliers']} "
          f"rms={floor['rms']*1000:.1f}mm (global-fit rms = drift warp evidence)")
    raised = pts[(pts @ floor_n + floor["d"]) > 0.15]
    surfaces = []
    for p in ransac_planes_sequential(raised, n_planes=6):
        if abs(p["normal"][1]) < 0.9 or p["inliers"] < 20_000:
            continue
        h = float(np.median(raised[p["mask"]] @ floor_n + floor["d"]))
        surfaces.append({"height_above_floor_m": round(h, 3),
                         "inliers": p["inliers"], "rms_mm": round(p["rms"] * 1000, 1)})
    print(f"[surfaces] {len(surfaces)} raised horizontal surfaces:")
    for s in surfaces:
        print(f"   +{s['height_above_floor_m']:.2f} m above floor, inliers={s['inliers']}")

    # camera height: median signed distance of trajectory from floor plane
    heights = (odo["t"] * k) @ floor_n + floor["d"]
    cam_height = float(np.median(heights))
    cam_height_ci = float(2 * np.std(heights))
    print(f"[floor] camera height above floor = {cam_height:.2f} m +- {cam_height_ci:.2f}")

    # --- ceiling: histogram along floor normal. Floor-focused captures may
    # not see it; report honestly instead of hallucinating a peak. ---
    signed = pts @ floor_n + floor["d"]
    content_top = float(np.percentile(signed, 99))
    ceil_h = None
    ceil_spread = None
    ceil_note = None
    lo = max(2.0, cam_height + 0.3)
    band = signed[(signed > lo) & (signed < 6.0)]
    if len(band) < 0.005 * len(signed):
        ceil_note = (f"ceiling not observable: no points above {lo:.1f} m; "
                     f"capture is floor-focused (top of visible content "
                     f"{content_top:.2f} m)")
        print(f"[ceiling] {ceil_note}")
    else:
        hist, edges = np.histogram(band, bins=int((6.0 - lo) / 0.01), range=(lo, 6.0))
        ceil_idx = int(hist.argmax())
        support = float(hist[ceil_idx]) / len(signed)
        if ceil_idx == 0 or ceil_idx == len(hist) - 1 or support < 0.002:
            ceil_note = (f"ceiling not observable: no supported plane peak above "
                         f"{lo:.1f} m (band-edge artifact rejected, top of visible "
                         f"content {content_top:.2f} m)")
            ceil_h = None
            ceil_spread = None
            print(f"[ceiling] {ceil_note}")
        else:
            ceil_h = float(edges[ceil_idx])
            sel = band[(band > ceil_h - 0.15) & (band < ceil_h + 0.15)]
            ceil_spread = float(np.std(sel)) if len(sel) > 2 else None
            print(f"[ceiling] height = {ceil_h:.2f} m (spread "
                  f"{None if ceil_spread is None else ceil_spread * 1000:.0f} mm)")

    # --- 3D wall planes: near-vertical dominant planes above the floor ---
    pts_f, R = transform_to_floor(pts, {"normal": floor_n, "d": floor["d"]})
    ztop = ceil_h - 0.10 if ceil_h is not None else content_top
    wall_band = pts[(pts @ floor_n + floor["d"]) > 0.20]
    wall_planes = ransac_planes_sequential(wall_band, n_planes=10)
    vertical = [p for p in wall_planes
                if abs(p["normal"][1]) < 0.35 and p["inliers"] > 10_000]
    print(f"[walls] {len(vertical)} near-vertical planes (rms = drift warp evidence):")
    for p in vertical:
        print(f"   n={p['normal'].round(3)} d={p['d']:.2f} inliers={p['inliers']} rms={p['rms']*1000:.1f}mm")

    # wall planes -> 2D lines on the floor plane (in floor frame)
    wall_lines = []
    for p in vertical:
        nx = np.array([p["normal"][0], p["normal"][2]])
        ln = np.linalg.norm(nx)
        if ln < 1e-6:
            continue
        nx = nx / ln
        # floor-frame 2D line: nx·q + (d + nx·f0) = 0 where f0 = floor plane point
        f0 = -floor_n * floor["d"]
        off = p["normal"] @ f0 + p["d"]
        wall_lines.append({"nx": nx[0], "ny": nx[1], "rho": -off, "count": p["inliers"],
                           "rms": p["rms"]})

    walls = []
    corners = []
    method = "none"
    if len(wall_lines) >= 2:
        # dominant directions (histogram over angles)
        thetas = np.array([np.arctan2(l["ny"], l["nx"]) % np.pi for l in wall_lines])
        hist, edges = np.histogram(thetas, bins=24, range=(0, np.pi))
        a0 = float(edges[hist.argmax()] + np.pi / 48)
        a1 = (a0 + np.pi / 2) % np.pi

        def near(theta, ref):
            return abs(np.arctan2(np.sin(theta - ref), np.cos(theta - ref)))

        g0 = [l for l in wall_lines if near(np.arctan2(l["ny"], l["nx"]), a0) < np.deg2rad(20)]
        g1 = [l for l in wall_lines if near(np.arctan2(l["ny"], l["nx"]), a1) < np.deg2rad(20)]
        g0 = sorted(g0, key=lambda l: -l["count"])[:3]
        g1 = sorted(g1, key=lambda l: -l["count"])[:3]
        if g0 and g1:
            method = "wall_plane_intersection"
            for la in g0:
                for lb in g1:
                    A = np.array([[la["nx"], la["ny"]], [lb["nx"], lb["ny"]]])
                    b = np.array([la["rho"], lb["rho"]])
                    if abs(np.linalg.det(A)) < 1e-9:
                        continue
                    c = np.linalg.solve(A, b)
                    # keep corners near the scanned area
                    corners.append(c)
            corners = np.array(corners)
            c = corners.mean(axis=0)
            ang = np.arctan2(corners[:, 1] - c[1], corners[:, 0] - c[0])
            corners = corners[np.argsort(ang)]
            for i in range(len(corners)):
                p1, p2 = corners[i], corners[(i + 1) % len(corners)]
                seg = p2 - p1
                length = float(np.linalg.norm(seg))
                if length < 0.3:
                    continue
                walls.append({
                    "id": f"W{i+1}",
                    "length_m": round(length, 3),
                    "length_ci_m": round(2 * 0.03, 3),
                    "p1": p1.round(3).tolist(),
                    "p2": p2.round(3).tolist(),
                })
    if not walls:
        # fallback: footprint boundary density -> convex hull
        xy = pts_f[:, :2]
        counts, origin, cell = density_grid(xy)
        occ, boundary_idx = extract_boundary_pixels(counts)
        boundary_xy = origin + (boundary_idx + 0.5) * cell
        from scipy.spatial import ConvexHull
        hull = ConvexHull(boundary_xy)
        corners = boundary_xy[hull.vertices]
        c = corners.mean(axis=0)
        ang = np.arctan2(corners[:, 1] - c[1], corners[:, 0] - c[0])
        corners = corners[np.argsort(ang)]
        method = "convex_hull_fallback"
        for i in range(len(corners)):
            p1, p2 = corners[i], corners[(i + 1) % len(corners)]
            length = float(np.linalg.norm(p2 - p1))
            if length < 0.3:
                continue
            walls.append({"id": f"W{i+1}", "length_m": round(length, 3),
                          "length_ci_m": round(2 * 0.05, 3),
                          "p1": p1.round(3).tolist(), "p2": p2.round(3).tolist()})
    print(f"[walls] method={method} walls={len(walls)} corners={len(corners)}")

    # --- damage: per-surface orthomosaics (floor + major raised surfaces)
    # + RGB-based detection. Furniture occludes most of the floor in these
    # captures, so each dominant horizontal surface gets its own mosaic. ---
    damage = None
    items = []
    import shutil as _shutil
    if os.path.exists(os.path.join(capture, "rgb.mp4")) and _shutil.which("ffmpeg"):
        try:
            from lib.damage import build_floor_mosaic, detect_damage, scope_items
            odo_m = {"frames": odo["frames"], "t": odo["t"] * k, "q": odo["q"], "K": odo["K"]}
            stride = max(1, len(odo["frames"]) // (200 if args.fast else 400))
            surface_planes = [("floor", floor_n, floor["d"])]
            for s in surfaces[:args.max_surfaces]:
                # a surface h above the floor satisfies n·p + d_s = 0 with
                # d_s = d_floor - h (n points up)
                hn = floor_n
                hd = floor["d"] - float(s["height_above_floor_m"])
                surface_planes.append((f"surface+{s['height_above_floor_m']:.2f}m", hn, hd))
            from lib.damage import extract_rgb_frames
            frames_dir = f"/tmp/mosaic_frames_{os.path.basename(capture)}"
            extract_rgb_frames(capture, frames_dir, stride)
            damage = {"surfaces": [], "extent_ci_m": None}
            thumbs = []
            for sname, sn, sd in surface_planes:
                mosaic = build_floor_mosaic(capture, odo_m, sn, sd, stride,
                                            cell=args.mosaic_cell, frames_dir=frames_dir)
                res = detect_damage(mosaic)
                res["surface"] = sname
                damage["surfaces"].append(res)
                # keep only a 4x-downsampled thumbnail for rendering
                thumbs.append({
                    "color": mosaic["color"][::4, ::4].copy(),
                    "coverage": mosaic["coverage"][::4, ::4].copy(),
                    "origin": mosaic["origin"].copy(),
                    "cell": mosaic["cell"] * 4,
                })
                del mosaic
                items += [dict(it, surface=sname) for it in scope_items(res)]
                print(f"[damage:{sname}] cracks={len(res['cracks'])} "
                      f"stains={len(res['stains'])} flags={len(res['flags'])} "
                      f"coverage={res['coverage_fraction']}")
            _shutil.rmtree(frames_dir, ignore_errors=True)
            damage["extent_ci_m"] = damage["surfaces"][0].get("extent_ci_m")
        except Exception as e:
            import traceback
            print(f"[damage] failed: {e}", file=sys.stderr)
            traceback.print_exc()
    else:
        print("[damage] skipped: rgb.mp4 or ffmpeg unavailable")

    # --- areas: observed floor (mosaic coverage) + scanned footprint
    # (occupancy). Both are honest bounds for floor-focused captures. ---
    xy = pts_f[:, :2]
    counts, origin, cell = density_grid(xy, cell=0.02)
    footprint = float((counts >= 3).sum()) * cell**2
    if damage is not None:
        area = damage["surfaces"][0]["covered_area_m2"]
        area_method = "observed_floor_mosaic"
    else:
        area = footprint
        area_method = "occupancy_fallback"

    result = {
        "capture": os.path.basename(capture.rstrip("/")),
        "schema_version": "0.1",
        "calibration": {"odometry_translation_scale": round(k, 4),
                        "reprojection_ratios_after": None if calib_report is None else
                        [{"pair": list(p), "ratio": r} for p, r in calib_report]},
        "camera_height_m": round(cam_height, 3),
        "camera_height_ci_m": round(cam_height_ci, 3),
        "raised_surfaces": surfaces,
        "ceiling_height_m": None if ceil_h is None else round(ceil_h, 3),
        "ceiling_height_ci_m": None if ceil_spread is None else round(2 * ceil_spread, 3),
        "ceiling_note": ceil_note,
        "content_top_m": round(content_top, 3),
        "floor_area_m2": round(area, 2),
        "floor_area_method": area_method,
        "scanned_footprint_m2": round(footprint, 2),
        "wall_method": method,
        "walls": walls,
        "walls_note": (None if walls else
                       "no near-vertical planes in capture: camera pitched "
                       "down 17-42 deg for the entire walk, walls are not "
                       "observed; floor extent and damage are the observable "
                       "outputs for this capture"),
        "damage": damage,
        "scope_items": items if damage else [],
        "corners": corners.round(3).tolist() if len(corners) else [],
        "timing_s": round(time.time() - t0, 1),
    }

    # --- render debug density map ---
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 8))
        xy = pts_f[:, :2]
        counts, origin, cell = density_grid(xy, cell=0.05)
        extent = [origin[0], origin[0] + counts.shape[0] * cell,
                  origin[1], origin[1] + counts.shape[1] * cell]
        ax.imshow(np.log1p(counts.T), origin="lower", extent=extent, cmap="viridis")
        if len(corners) >= 3:
            poly = np.vstack([corners, corners[:1]])
            ax.plot(poly[:, 0], poly[:, 1], "r-", lw=2)
            for w in walls:
                m = (np.array(w["p1"]) + np.array(w["p2"])) / 2
                ax.text(m[0], m[1], f"{w['length_m']}m", color="w", fontsize=9,
                        ha="center", va="center")
        ax.set_title("wall density + polygon")
        fig.savefig(os.path.join(out, "density_debug.png"), dpi=110)
        plt.close(fig)
        # mosaic + damage overlay (one figure per surface, up to 2)
        if damage is not None:
            for si, (res, m) in enumerate(zip(damage["surfaces"][:2], thumbs[:2])):
                img = m["color"] / 255.0
                img = np.where(m["coverage"][:, :, None], img, 0.5)
                extent2 = [m["origin"][0],
                           m["origin"][0] + img.shape[0] * m["cell"],
                           m["origin"][1],
                           m["origin"][1] + img.shape[1] * m["cell"]]
                fig2, ax2 = plt.subplots(figsize=(10, 8))
                ax2.imshow(np.transpose(img, (1, 0, 2)), origin="lower", extent=extent2)
                for reg in res["cracks"]:
                    ax2.plot(reg["center_m"][0], reg["center_m"][1], "r+", ms=12, mew=2)
                for reg in res["stains"]:
                    ax2.plot(reg["center_m"][0], reg["center_m"][1], "yo", ms=10, mec="k")
                for fl in res["flags"]:
                    ax2.plot(fl["center_m"][0], fl["center_m"][1], "bx", ms=10)
                ax2.set_title(f"{res['surface']} orthomosaic (+ cracks, o stains, x unobserved)")
                fig2.savefig(os.path.join(out, f"mosaic_{si}.png"), dpi=110)
                plt.close(fig2)
    except Exception as e:  # rendering is best-effort, JSON is the contract
        print(f"[render] skipped: {e}")

    with open(os.path.join(out, "plan.json"), "w") as fh:
        json.dump(result, fh, indent=2)
    print(f"[out] {os.path.join(out, 'plan.json')} ({result['timing_s']}s)")
    print(json.dumps({k: v for k, v in result.items()
                      if k in ("camera_height_m", "ceiling_height_m", "floor_area_m2")}, indent=2))


if __name__ == "__main__":
    main()
