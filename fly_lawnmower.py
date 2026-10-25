#!/usr/bin/env python3
"""Four simulated Crazyflies each sweep one vertical strip of a 4 m x 4 m arena.

Logs each drone's position estimate (10 Hz) to logs/drone<i>.csv, with a wall-clock
column so it can be matched against the simulator's ground-truth log, and prints a
per-drone summary, so you can confirm the flight without a viewer.

Verified on the first run (2026-10-20), with the sim in the Docker container:
  * CrazySim serves drone i at udp://127.0.0.1:(19850 + i).
  * Commanded positions are in the same world frame as the spawn positions in
    surg_four.txt (0,0 / 1,0 / 2,0 / 3,0).

Usage (inside the container, with the sim already running):
    python3 fly_lawnmower.py
"""
import csv
import os
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.positioning.position_hl_commander import PositionHlCommander

from lawnmower_path import HEIGHT, N_DRONES, VELOCITY, spawn, strip_waypoints

BASE_PORT = 19850
# Where logs are written. run_sweep.sh sets LOG_DIR per run; default is ./logs.
LOG_DIR = os.environ.get("LOG_DIR", "logs")

URIS = [f"udp://127.0.0.1:{BASE_PORT + i}" for i in range(N_DRONES)]
STARTS = {URIS[i]: spawn(i) for i in range(N_DRONES)}  # must match agents.txt


def start_logging(scf, idx):
    f = open(os.path.join(LOG_DIR, f"drone{idx}.csv"), "w", newline="")
    w = csv.writer(f)
    # t_s is the firmware's own clock. wall_s is this computer's clock (time.time()) when
    # the sample arrived; the simulator's truth log uses the same clock, which is what
    # lets compare_runs.py line estimates up with the true positions.
    w.writerow(["t_s", "x", "y", "z", "wall_s"])
    cfg = LogConfig(name="pos", period_in_ms=100)
    for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
        cfg.add_variable(v, "float")

    def on_data(timestamp, data, _logconf):
        w.writerow([timestamp / 1000.0, data["stateEstimate.x"],
                    data["stateEstimate.y"], data["stateEstimate.z"],
                    f"{time.time():.4f}"])

    scf.cf.log.add_config(cfg)
    cfg.data_received_cb.add_callback(on_data)
    cfg.start()
    return f, cfg


def fly(scf, idx):
    f, cfg = start_logging(scf, idx)
    try:
        sx, sy = STARTS[scf._link_uri]
        with PositionHlCommander(
            scf, x=sx, y=sy, z=0.0,
            default_velocity=VELOCITY, default_height=HEIGHT,
        ) as pc:
            for (x, y) in strip_waypoints(idx):
                pc.go_to(x, y)
            time.sleep(0.5)
    finally:
        cfg.stop()
        f.close()


def summarize():
    print("\nPer-drone summary from logs (metres):")
    for i in range(N_DRONES):
        path = os.path.join(LOG_DIR, f"drone{i}.csv")
        with open(path) as fh:
            rows = list(csv.DictReader(fh))
        if not rows:
            print(f"  drone {i}: NO DATA LOGGED")
            continue
        xs = [float(r["x"]) for r in rows]
        ys = [float(r["y"]) for r in rows]
        zs = [float(r["z"]) for r in rows]
        print(f"  drone {i}: {len(rows)} samples, "
              f"x [{min(xs):.2f}, {max(xs):.2f}], "
              f"y [{min(ys):.2f}, {max(ys):.2f}], max z {max(zs):.2f}")


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache="./cache")
    with Swarm(URIS, factory=factory) as swarm:
        swarm.reset_estimators()
        swarm.parallel_safe(fly, args_dict={uri: [i] for i, uri in enumerate(URIS)})
    summarize()


if __name__ == "__main__":
    main()
