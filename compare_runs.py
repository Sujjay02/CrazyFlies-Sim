#!/usr/bin/env python3
"""Score logged lawnmower flights and compare configurations.

Reads runs/<config>_<k>/drone<i>.csv (written by fly_lawnmower.py via
run_sweep.sh), computes the metrics below per run, and prints mean +/- std per
config. The same script can score hardware runs later: put real logs in
runs/hw_<k>/ with the same CSV columns (t_s, x, y, z) and they show up as the
config "hw", next to the simulated ones. That is the sim-to-real comparison.

Metrics (airborne samples only, z > 0.15 m, so takeoff and landing are excluded):
  track_rmse   RMS distance of the logged xy path from the commanded zig-zag
               polyline, metres (mean over drones)
  track_max    worst such distance, any drone
  wp_miss_max  worst miss distance to a commanded waypoint, any drone
  z_over       peak height minus the 0.5 m command, any drone
  t_air        time airborne, seconds (mean over drones)
  sep_min      smallest 3D distance between any two drones while all are
               airborne, with drones aligned on their own takeoff time

Usage:  python3 compare_runs.py [runs_dir]        (default /work/runs or ./runs)
"""
import csv
import glob
import os
import re
import sys
from collections import defaultdict

import numpy as np

from lawnmower_path import HEIGHT, N_DRONES, spawn, strip_waypoints

AIRBORNE_Z = 0.15
METRICS = ["track_rmse", "track_max", "wp_miss_max", "z_over", "t_air", "sep_min"]


def load(path):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return None
    a = np.array([[float(r["t_s"]), float(r["x"]), float(r["y"]), float(r["z"])]
                  for r in rows])
    return a[np.argsort(a[:, 0])]


def dist_to_polyline(xy, poly):
    """Distance from each row of xy (N,2) to the polyline poly (M,2)."""
    best = np.full(len(xy), np.inf)
    for a, b in zip(poly[:-1], poly[1:]):
        ab = b - a
        denom = float(ab @ ab)
        t = np.clip(((xy - a) @ ab) / denom, 0.0, 1.0) if denom > 0 else np.zeros(len(xy))
        proj = a + t[:, None] * ab
        best = np.minimum(best, np.linalg.norm(xy - proj, axis=1))
    return best


def drone_metrics(i, a):
    air = a[a[:, 3] > AIRBORNE_Z]
    if len(air) < 5:
        return None
    xy = air[:, 1:3]
    poly = np.array([spawn(i)] + list(strip_waypoints(i)), dtype=float)
    d = dist_to_polyline(xy, poly)
    wps = np.array(strip_waypoints(i), dtype=float)
    miss = [float(np.min(np.linalg.norm(xy - w, axis=1))) for w in wps]
    return {
        "track_rmse": float(np.sqrt(np.mean(d ** 2))),
        "track_max": float(np.max(d)),
        "wp_miss_max": float(max(miss)),
        "z_over": float(np.max(air[:, 3]) - HEIGHT),
        "t_air": float(air[-1, 0] - air[0, 0]),
        "t0": float(air[0, 0]),
        "air": air,
    }


def separation(per_drone):
    """Min pairwise 3D distance, aligning drones on their own takeoff time."""
    series = []
    for m in per_drone:
        air = m["air"]
        series.append((air[:, 0] - m["t0"], air[:, 1:4]))
    t_end = min(s[0][-1] for s in series)
    if t_end <= 0:
        return float("nan")
    grid = np.arange(0.0, t_end, 0.1)
    pos = [np.column_stack([np.interp(grid, t, p[:, k]) for k in range(3)])
           for t, p in series]
    best = np.inf
    for a in range(len(pos)):
        for b in range(a + 1, len(pos)):
            best = min(best, float(np.min(np.linalg.norm(pos[a] - pos[b], axis=1))))
    return best


def score_run(run_dir):
    per_drone = []
    for i in range(N_DRONES):
        path = os.path.join(run_dir, f"drone{i}.csv")
        if not os.path.exists(path):
            return None
        a = load(path)
        m = drone_metrics(i, a) if a is not None else None
        if m is None:
            return None
        per_drone.append(m)
    return {
        "track_rmse": float(np.mean([m["track_rmse"] for m in per_drone])),
        "track_max": max(m["track_max"] for m in per_drone),
        "wp_miss_max": max(m["wp_miss_max"] for m in per_drone),
        "z_over": max(m["z_over"] for m in per_drone),
        "t_air": float(np.mean([m["t_air"] for m in per_drone])),
        "sep_min": separation(per_drone),
    }


CHANGE_METRICS = ["track_rmse", "track_max", "wp_miss_max", "z_over"]
MIN_EFFECT_M = 0.002   # differences smaller than 2 mm are never flagged, however tight the spread


def reference_for(cfg, configs):
    """pose_* configs are compared with pose30 (same 30 Hz rate, nothing else
    degraded); everything else, including pose30 itself, with baseline."""
    if cfg.startswith("pose_") and "pose30" in configs:
        return "pose30"
    return "baseline"


def print_changes(by_config):
    """Change in each config's mean versus its reference. A '*' marks a difference
    larger than 2 standard errors of the difference and at least MIN_EFFECT_M, a
    rough screen for 'bigger than the run-to-run spread', not a formal test (runs
    per config are few)."""
    rows = []
    for cfg in sorted(by_config):
        ref = reference_for(cfg, by_config)
        if cfg == ref or ref not in by_config:
            continue
        a_runs, b_runs = by_config[cfg], by_config[ref]
        cells = []
        for m in CHANGE_METRICS:
            a = np.array([r[m] for r in a_runs])
            b = np.array([r[m] for r in b_runs])
            d = float(a.mean() - b.mean())
            if len(a) > 1 and len(b) > 1:
                se = float(np.sqrt(a.var(ddof=1) / len(a) + b.var(ddof=1) / len(b)))
                flag = "*" if abs(d) > 2.0 * se and abs(d) >= MIN_EFFECT_M else " "
                cells.append(f"{d:+.3f}{flag}")
            else:
                cells.append(f"{d:+.3f}?")   # one run: no spread to judge against
        rows.append((cfg, ref, cells))
    if not rows:
        return
    print("\nChange vs reference, in metres (* = more than 2 standard errors and >= 2 mm; "
          "? = too few runs to tell)")
    print(f"{'config':<16}{'vs':<10}" + "".join(f"{m:>14}" for m in CHANGE_METRICS))
    for cfg, ref, cells in rows:
        print(f"{cfg:<16}{ref:<10}" + "".join(f"{c:>14}" for c in cells))


def main():
    default = "/work/runs" if os.path.isdir("/work/runs") else "runs"
    runs_dir = sys.argv[1] if len(sys.argv) > 1 else default
    by_config = defaultdict(list)
    per_run_rows = []
    skipped = []
    for d in sorted(glob.glob(os.path.join(runs_dir, "*_*"))):
        if not os.path.isdir(d):
            continue
        m = re.match(r"(.+)_(\d+)$", os.path.basename(d))
        if not m:
            continue
        cfg, k = m.group(1), int(m.group(2))
        s = score_run(d)
        if s is None:
            skipped.append(os.path.basename(d))
            continue
        by_config[cfg].append(s)
        per_run_rows.append({"config": cfg, "run": k, **s})

    if not by_config:
        print(f"No scorable runs found in {runs_dir}")
        if skipped:
            print("Skipped (missing or empty logs):", ", ".join(skipped))
        return 1

    with open(os.path.join(runs_dir, "summary_per_run.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["config", "run"] + METRICS)
        w.writeheader()
        w.writerows(per_run_rows)

    agg_rows = []
    header = f"{'config':<16}{'n':>3}" + "".join(f"{m:>16}" for m in METRICS)
    print(header)
    print("-" * len(header))
    for cfg in sorted(by_config, key=lambda c: (c != "baseline", c)):
        runs = by_config[cfg]
        row = {"config": cfg, "n": len(runs)}
        cells = []
        for m in METRICS:
            v = np.array([r[m] for r in runs])
            mean = float(np.nanmean(v))
            std = float(np.nanstd(v)) if len(v) > 1 else 0.0
            row[m + "_mean"], row[m + "_std"] = mean, std
            cells.append(f"{mean:>9.3f}+-{std:<5.3f}")
        agg_rows.append(row)
        print(f"{cfg:<16}{len(runs):>3}" + "".join(f"{c:>16}" for c in cells))

    print_changes(by_config)

    fields = ["config", "n"] + [m + s for m in METRICS for s in ("_mean", "_std")]
    with open(os.path.join(runs_dir, "summary_by_config.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(agg_rows)

    print("\nUnits: metres, except t_air in seconds. Wrote summary_per_run.csv and "
          "summary_by_config.csv to", runs_dir)
    if skipped:
        print("Skipped (missing or empty logs):", ", ".join(skipped))
    return 0


if __name__ == "__main__":
    sys.exit(main())
