"""Repeatability gate: compare two captures of the same space, same tier.

    python scripts/compare_captures.py plan_a.json plan_b.json

Prints a repeatability table: floor height, camera height, raised-surface
heights, and per-surface damage density agreement between the two plans.
"""
import argparse
import json


def load(path):
    with open(path) as fh:
        return json.load(fh)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("plan_a")
    ap.add_argument("plan_b")
    args = ap.parse_args()
    a, b = load(args.plan_a), load(args.plan_b)

    # origin-independent measurements (each capture has its own odometry
    # origin, so only heights RELATIVE to the floor are comparable)
    rows = []
    for label, ka, kb in (
        ("camera height (m)", "camera_height_m", "camera_height_m"),
        ("scanned footprint (m2)", "scanned_footprint_m2", "scanned_footprint_m2"),
    ):
        va, vb = a.get(ka), b.get(kb)
        if va is None or vb is None:
            continue
        rows.append((label, va, vb, abs(va - vb)))

    # raised surfaces: nearest-height pairing
    sa = sorted(s["height_above_floor_m"] for s in a.get("raised_surfaces", []))
    sb = sorted(s["height_above_floor_m"] for s in b.get("raised_surfaces", []))
    for i in range(max(len(sa), len(sb))):
        va = sa[i] if i < len(sa) else None
        vb = sb[i] if i < len(sb) else None
        if va is not None and vb is not None:
            rows.append((f"raised surface #{i+1} (m)", va, vb, abs(va - vb)))

    print(f"{'measurement':28s} {'capture A':>10s} {'capture B':>10s} {'|diff|':>8s}")
    for label, va, vb, diff in rows:
        print(f"{label:28s} {va:10.3f} {vb:10.3f} {diff:8.3f}")

    # damage density per surface class
    def density(plan, surf, cls):
        for s in plan.get("damage", {}).get("surfaces", []):
            if s["surface"] == surf:
                area = max(s.get("covered_area_m2"), 1e-6)
                return len(s[cls]) / area
        return None

    print("\ndamage density (count per m2 of covered surface):")
    for surf in ("floor", "surface+0.90m"):
        for cls in ("cracks", "stains"):
            da, db = density(a, surf, cls), density(b, surf, cls)
            if da is not None and db is not None:
                print(f"{surf:16s} {cls:7s} A={da:8.2f} B={db:8.2f}")


if __name__ == "__main__":
    main()
