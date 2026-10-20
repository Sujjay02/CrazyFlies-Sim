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
