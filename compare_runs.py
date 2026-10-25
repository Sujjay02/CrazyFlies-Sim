#!/usr/bin/env python3
"""Score logged lawnmower flights and compare configurations.

Reads runs/<config>_<k>/drone<i>.csv (written by fly_lawnmower.py via
run_sweep.sh), computes the metrics below per run, and prints mean +/- std per
config. The same script can score hardware runs later: put real logs in
runs/hw_<k>/ with the same CSV columns (t_s, x, y, z) and they show up as the
config "hw", next to the simulated ones. That is the sim-to-real comparison.

ESTIMATE metrics, from the position the drone believes it has (works for hardware too).
Airborne samples only, z > 0.15 m, so takeoff and landing are excluded:
  track_rmse   RMS distance of the logged xy path from the commanded zig-zag
               polyline, metres (mean over drones)
  track_max    worst such distance, any drone
  wp_miss_max  worst miss distance to a commanded waypoint, any drone
  z_over       peak height minus the 0.5 m command, any drone
  t_air        time airborne, seconds (mean over drones)
  sep_min      smallest 3D distance between any two drones while all are
               airborne, with drones aligned on their own takeoff time

GROUND-TRUTH metrics, simulation only, from runs/<config>_<k>/truth_drone<i>.csv
(written by the simulator when run_sweep.sh sets CF_TRUTH_DIR). These are where the
drones really were, which hardware cannot tell you without motion capture:
  true_track_rmse  like track_rmse, but for the true position
  true_track_max   like track_max, true position
  true_z_over      like z_over, true position
  true_sep_min     smallest true 3D distance between any two drones at the same
                   moment (all drones share one clock)
  est_err_rms      RMS 3D distance between the estimate and the truth at the same
                   moment, metres (estimation error)
  est_err_max      worst such distance
  est_lag_ms       how far the estimate trails the truth, milliseconds: the time
                   shift that best lines the two up. Includes a few ms of logging
                   delay, so compare configs with each other, not with zero.

Estimate and truth are matched by wall-clock time (the flight script stamps each
sample with time.time(); so does the simulator), so the match does not depend on how
fast the simulation runs. Accuracy is a few milliseconds.

Usage:  python3 compare_runs.py [runs_dir]        (default /work/runs or ./runs)
"""
import csv
import glob
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np

from lawnmower_path import HEIGHT, N_DRONES, spawn, strip_waypoints

AIRBORNE_Z = 0.15
METRICS = ["track_rmse", "track_max", "wp_miss_max", "z_over", "t_air", "sep_min"]
TRUTH_METRICS = ["true_track_rmse", "true_track_max", "true_z_over", "true_sep_min",
                 "est_err_rms", "est_err_max", "est_lag_ms"]
LAG_GRID_S = np.arange(-0.300, 0.3001, 0.005)

NAN = float("nan")


def load(path):
    """Estimate log -> array with columns t_s, x, y, z, wall_s (wall_s is NaN when the
    log has none, as for older runs and for hardware)."""
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return None
    a = np.array([[float(r["t_s"]), float(r["x"]), float(r["y"]), float(r["z"]),
                   float(r.get("wall_s") or "nan")] for r in rows])
    return a[np.argsort(a[:, 0])]


def load_truth(path):
    """Truth log -> array with columns wall_s, x, y, z, sorted by wall time."""
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return None
    a = np.array([[float(r["wall_s"]), float(r["x"]), float(r["y"]), float(r["z"])]
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


def interp_xyz(tr, t):
    """Truth position at wall times t (tr columns: wall, x, y, z)."""
    return np.column_stack([np.interp(t, tr[:, 0], tr[:, k]) for k in (1, 2, 3)])


def score_truth(run_dir, ests):
    """Ground-truth metrics for one run, or None when the run has no truth logs or
    its estimate logs carry no wall-clock column. ests: per-drone estimate arrays."""
    truths = []
    for i in range(N_DRONES):
        p = os.path.join(run_dir, f"truth_drone{i}.csv")
        if not os.path.exists(p):
            return None
        tr = load_truth(p)
        if tr is None or len(tr) < 10:
            return None
        truths.append(tr)
    if any(np.all(np.isnan(e[:, 4])) for e in ests):
        return None

    rms, mx, zo, air_windows = [], [], [], []
    sq_by_lag = np.zeros(len(LAG_GRID_S))
    sq0, n0, max_err = 0.0, 0, 0.0
    for i in range(N_DRONES):
        tr, e = truths[i], ests[i]
        air = tr[tr[:, 3] > AIRBORNE_Z]
        if len(air) < 5:
            return None
        poly = np.array([spawn(i)] + list(strip_waypoints(i)), dtype=float)
        d = dist_to_polyline(air[:, 1:3], poly)
        rms.append(float(np.sqrt(np.mean(d ** 2))))
        mx.append(float(np.max(d)))
        zo.append(float(np.max(air[:, 3]) - HEIGHT))
        air_windows.append((air[0, 0], air[-1, 0]))

        ok = (e[:, 3] > AIRBORNE_Z) & np.isfinite(e[:, 4]) \
            & (e[:, 4] >= tr[0, 0]) & (e[:, 4] <= tr[-1, 0])
        ew, epos = e[ok, 4], e[ok, 1:4]
        if len(ew) < 5:
            return None
        err = np.linalg.norm(epos - interp_xyz(tr, ew), axis=1)
        sq0 += float(np.sum(err ** 2))
        n0 += len(err)
        max_err = max(max_err, float(np.max(err)))
        for k, lag in enumerate(LAG_GRID_S):
            # the estimate at time t is compared with where the drone was at t - lag
            sq_by_lag[k] += float(np.sum(np.linalg.norm(epos - interp_xyz(tr, ew - lag),
                                                        axis=1) ** 2))

    lo = max(w[0] for w in air_windows)
    hi = min(w[1] for w in air_windows)
    sep = NAN
    if hi > lo:
        grid = np.arange(lo, hi, 0.01)
        pos = [interp_xyz(tr, grid) for tr in truths]
        sep = min(float(np.min(np.linalg.norm(pos[a] - pos[b], axis=1)))
                  for a in range(N_DRONES) for b in range(a + 1, N_DRONES))
    return {
        "true_track_rmse": float(np.mean(rms)),
        "true_track_max": max(mx),
        "true_z_over": max(zo),
        "true_sep_min": sep,
        "est_err_rms": math.sqrt(sq0 / n0),
        "est_err_max": max_err,
        "est_lag_ms": float(LAG_GRID_S[int(np.argmin(sq_by_lag))] * 1000.0),
    }


def score_run(run_dir):
    per_drone, ests = [], []
    for i in range(N_DRONES):
        path = os.path.join(run_dir, f"drone{i}.csv")
        if not os.path.exists(path):
            return None
        a = load(path)
        m = drone_metrics(i, a) if a is not None else None
        if m is None:
            return None
        per_drone.append(m)
        ests.append(a)
    out = {
        "track_rmse": float(np.mean([m["track_rmse"] for m in per_drone])),
        "track_max": max(m["track_max"] for m in per_drone),
        "wp_miss_max": max(m["wp_miss_max"] for m in per_drone),
        "z_over": max(m["z_over"] for m in per_drone),
        "t_air": float(np.mean([m["t_air"] for m in per_drone])),
        "sep_min": separation(per_drone),
    }
    truth = score_truth(run_dir, ests)
    out.update(truth if truth is not None else {m: NAN for m in TRUTH_METRICS})
    return out


CHANGE_METRICS = ["track_rmse", "track_max", "wp_miss_max", "z_over"]
TRUTH_CHANGE_METRICS = ["true_track_rmse", "true_track_max", "true_z_over",
                        "true_sep_min", "est_err_rms", "est_err_max", "est_lag_ms"]
MIN_EFFECT_M = 0.002   # differences smaller than 2 mm are never flagged, however tight the spread
MIN_EFFECT_MS = 5.0    # same idea for the estimate lag


def values(runs, m):
    v = np.array([r[m] for r in runs], dtype=float)
    return v[np.isfinite(v)]


def reference_for(cfg, configs):
    """pose_* configs are compared with pose30 (same 30 Hz rate, nothing else
    degraded); everything else, including pose30 itself, with baseline."""
    if cfg.startswith("pose_") and "pose30" in configs:
        return "pose30"
    return "baseline"


def print_changes(by_config, metrics, title):
    """Change in each config's mean versus its reference. A '*' marks a difference
    larger than 2 standard errors of the difference and at least MIN_EFFECT_M (or
    MIN_EFFECT_MS for the lag), a rough screen for 'bigger than the run-to-run
    spread', not a formal test (runs per config are few)."""
    rows = []
    for cfg in sorted(by_config):
        ref = reference_for(cfg, by_config)
        if cfg == ref or ref not in by_config:
            continue
        cells, any_data = [], False
        for m in metrics:
            a, b = values(by_config[cfg], m), values(by_config[ref], m)
            if len(a) == 0 or len(b) == 0:
                cells.append("n/a")
                continue
            any_data = True
            d = float(a.mean() - b.mean())
            lag = m == "est_lag_ms"
            txt = f"{d:+.0f}" if lag else f"{d:+.3f}"
            if len(a) > 1 and len(b) > 1:
                se = float(np.sqrt(a.var(ddof=1) / len(a) + b.var(ddof=1) / len(b)))
                floor = MIN_EFFECT_MS if lag else MIN_EFFECT_M
                flag = "*" if abs(d) > 2.0 * se and abs(d) >= floor else " "
                cells.append(txt + flag)
            else:
                cells.append(txt + "?")   # one run: no spread to judge against
        if any_data:
            rows.append((cfg, ref, cells))
    if not rows:
        return
    print(f"\n{title}")
    print(f"{'config':<16}{'vs':<10}" + "".join(f"{m:>17}" for m in metrics))
    for cfg, ref, cells in rows:
        print(f"{cfg:<16}{ref:<10}" + "".join(f"{c:>17}" for c in cells))


def fmt_cell(v, m):
    mean, std = float(np.mean(v)), float(np.std(v)) if len(v) > 1 else 0.0
    if m == "est_lag_ms":
        return f"{mean:>7.0f}+-{std:<5.0f}"
    return f"{mean:>9.3f}+-{std:<5.3f}"


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
        w = csv.DictWriter(fh, fieldnames=["config", "run"] + METRICS + TRUTH_METRICS)
        w.writeheader()
        w.writerows(per_run_rows)

    order = sorted(by_config, key=lambda c: (c != "baseline", c))
    agg_rows = {cfg: {"config": cfg, "n": len(by_config[cfg])} for cfg in order}

    # Table 1: what the drone believes (estimate)
    header = f"{'config':<16}{'n':>3}" + "".join(f"{m:>16}" for m in METRICS)
    print("Estimate: where the drone believes it is")
    print(header)
    print("-" * len(header))
    for cfg in order:
        runs = by_config[cfg]
        cells = []
        for m in METRICS:
            v = values(runs, m)
            agg_rows[cfg][m + "_mean"] = float(np.mean(v)) if len(v) else ""
            agg_rows[cfg][m + "_std"] = float(np.std(v)) if len(v) > 1 else 0.0
            cells.append(fmt_cell(v, m) if len(v) else "n/a")
        print(f"{cfg:<16}{len(runs):>3}" + "".join(f"{c:>16}" for c in cells))

    print_changes(by_config, CHANGE_METRICS,
                  "Change vs reference, in metres (* = more than 2 standard errors "
                  "and >= 2 mm; ? = too few runs to tell)")

    # Table 2: where the drone really is (simulation ground truth)
    with_truth = [c for c in order if len(values(by_config[c], "true_track_rmse"))]
    if with_truth:
        print("\nGround truth (simulation only): where the drone really is")
        header = f"{'config':<16}{'n':>3}" + "".join(f"{m:>18}" for m in TRUTH_METRICS)
        print(header)
        print("-" * len(header))
        for cfg in with_truth:
            runs = by_config[cfg]
            n_truth = len(values(runs, "true_track_rmse"))
            agg_rows[cfg]["truth_n"] = n_truth
            cells = []
            for m in TRUTH_METRICS:
                v = values(runs, m)
                agg_rows[cfg][m + "_mean"] = float(np.mean(v)) if len(v) else ""
                agg_rows[cfg][m + "_std"] = float(np.std(v)) if len(v) > 1 else 0.0
                cells.append(fmt_cell(v, m) if len(v) else "n/a")
            print(f"{cfg:<16}{n_truth:>3}" + "".join(f"{c:>18}" for c in cells))
        print_changes(by_config, TRUTH_CHANGE_METRICS,
                      "Ground truth, change vs reference (metres; est_lag_ms in ms; "
                      "* = more than 2 standard errors and >= 2 mm / 5 ms)")

    fields = ["config", "n", "truth_n"] + [m + s for m in METRICS + TRUTH_METRICS
                                           for s in ("_mean", "_std")]
    with open(os.path.join(runs_dir, "summary_by_config.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, restval="")
        w.writeheader()
        w.writerows(agg_rows[c] for c in order)

    print("\nUnits: metres, except t_air in seconds and est_lag_ms in milliseconds. "
          "Wrote summary_per_run.csv and summary_by_config.csv to", runs_dir)
    if skipped:
        print("Skipped (missing or empty logs):", ", ".join(skipped))
    return 0


if __name__ == "__main__":
    sys.exit(main())
