# CrazyFlies-Sim

Four-drone Crazyflie simulation for a sim-to-real gap study in multi-agent aerial
coverage. It uses [CrazySim](https://github.com/gtfactslab/CrazySim) (Crazyflie
firmware in the loop, MuJoCo physics) inside an Ubuntu 22.04 Docker container, and
drives the drones with [cflib](https://github.com/bitcraze/crazyflie-lib-python).

Status: the four simulated drones connect and the lawnmower sweep runs end to end.
The position-log check passed on the first run (2026-10-20): each drone logged 158
samples, stayed in its own strip (x from its spawn x to spawn + 0.75 m), covered
y 0 to 4.01 m, and peaked at z 0.56 m against a 0.5 m command. This confirms
commanded positions share the world frame of the spawn positions in `agents.txt`.

## Layout

| File | Purpose |
| --- | --- |
| `Dockerfile` | Ubuntu 22.04, Gazebo Garden (optional backend), build prerequisites |
| `run_container.sh` | Interactive shell in the container, this repo mounted at `/work` |
| `build_crazysim.sh` | Clone and build CrazySim, install cflib from source, MuJoCo deps |
| `launch_headless.sh` | Start the sim with no viewer window (no X display needed) |
| `agents.txt` | Four spawn positions, 1 m apart along x |
| `fly_lawnmower.py` | Each drone sweeps one strip of a 4 m x 4 m arena, logs positions |
| `lawnmower_path.py` | The sweep geometry (waypoints, height, spawn points), shared by the flight and analysis scripts |
| `run_sweep.sh` | Runs the sweep under several physics configs with repeats, one log folder per run |
| `compare_runs.py` | Scores logged flights and compares configs (and, later, hardware runs) |

CrazySim and cflib are cloned into this folder by the build script and are not
tracked here (see `.gitignore`).

## Setup

Host requirements: Docker, and your user in the `docker` group.

```bash
# 1. Build the image (once)
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -t crazysim:22.04 .

# 2. Open a shell in the container
bash run_container.sh

# 3. Inside the container: clone and build CrazySim (takes a few minutes)
bash build_crazysim.sh
```

## Run

Terminal 1, inside the container:

```bash
bash /work/launch_headless.sh -m cf2x_T350 -f surg_four.txt
```

Optional physics flags, all off by default (from the CrazySim README):
`--sensor-noise --ground-effect --downwash --wind-speed <m/s> --turbulence <level>`.

Terminal 2, a second shell in the same container:

```bash
docker exec -it $(docker ps -q | head -n1) bash
cd /work && python3 fly_lawnmower.py
```

The script logs `logs/drone<i>.csv` (10 Hz position estimates) and prints a
per-drone summary. Drone i is at `udp://127.0.0.1:(19850 + i)`.

## Parameter sweep (the sim side of the sim-to-real comparison)

Stop any sim you have running (Ctrl-C), then inside the container, from one terminal:

```bash
bash /work/run_sweep.sh                  # baseline, noise, downwash, noise_downwash x 3 runs
python3 /work/compare_runs.py            # table of mean +/- std per config
```

`run_sweep.sh` starts the headless sim, flies the sweep, saves the logs to
`runs/<config>_<k>/`, and stops the sim, for each run. `REPEATS=5` changes the repeat
count, and naming configs (`bash run_sweep.sh baseline noise`) runs a subset. To sweep
another knob, add a row to the `FLAGS` table at the top of the script (for example
`[wind]="--wind-speed 0.3"`).

`compare_runs.py` measures, on airborne samples only: tracking error against the
commanded zig-zag (`track_rmse`, `track_max`), worst waypoint miss, height overshoot,
time airborne, and the smallest gap between any two drones. It writes
`runs/summary_per_run.csv` and `runs/summary_by_config.csv`.

When the real drones fly, log them to the same CSV format (`t_s,x,y,z` per drone,
`drone0.csv` to `drone3.csv`) in `runs/hw_1/`, `runs/hw_2/`, and so on. Rerunning
`compare_runs.py` then puts `hw` in the same table as the simulated configs.

## Notes

- `run_container.sh` uses `--rm`, so packages installed with `pip --user` inside
  the container disappear when it exits. Re-run `build_crazysim.sh` in a new
  container, or move those installs into the Dockerfile.
- The stock CrazySim MuJoCo launcher always opens a viewer window, which needs an
  X display. `launch_headless.sh` makes a copy without that flag.
- Snap Docker needs a `docker` group created manually
  (`sudo addgroup --system docker`) before non-root use.
- `fly_lawnmower.py` assumes commanded positions share the world frame of the spawn
  positions in `agents.txt`. Confirmed from the logs on the first run (see Status).
