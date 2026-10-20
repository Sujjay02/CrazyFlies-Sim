#!/usr/bin/env python3
"""Four simulated Crazyflies each sweep one vertical strip of a 4 m x 4 m arena.

Logs each drone's position estimate (10 Hz) to logs/drone<i>.csv and prints a
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

N_DRONES = 4
BASE_PORT = 19850
ARENA = 4.0      # metres, square arena
HEIGHT = 0.5     # metres
VELOCITY = 0.4   # m/s
ROWS = 5         # sweep passes per strip
LOG_DIR = "logs"

URIS = [f"udp://127.0.0.1:{BASE_PORT + i}" for i in range(N_DRONES)]
STARTS = {URIS[i]: (float(i), 0.0) for i in range(N_DRONES)}  # must match surg_four.txt


def strip_waypoints(i):
    """Zig-zag path over strip i (width ARENA / N_DRONES)."""
    width = ARENA / N_DRONES
    x0 = i * width + width / 2.0
    ys = [ARENA * k / (ROWS - 1) for k in range(ROWS)]
    pts = []
    for k, y in enumerate(ys):
        dx = -width / 4.0 if k % 2 == 0 else width / 4.0
        pts.append((x0 + dx, y))
    return pts


def start_logging(scf, idx):
    f = open(os.path.join(LOG_DIR, f"drone{idx}.csv"), "w", newline="")
    w = csv.writer(f)
    w.writerow(["t_s", "x", "y", "z"])
    cfg = LogConfig(name="pos", period_in_ms=100)
    for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
        cfg.add_variable(v, "float")

    def on_data(timestamp, data, _logconf):
        w.writerow([timestamp / 1000.0, data["stateEstimate.x"],
                    data["stateEstimate.y"], data["stateEstimate.z"]])

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
