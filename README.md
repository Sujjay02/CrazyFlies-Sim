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
| `apply_pose_patch.py` | Patches CrazySim so the position measurement can have noise, latency, dropout and a limited rate, and so it can log the drones' true positions (applied automatically by `launch_headless.sh`) |
| `run_sweep.sh` | Runs the sweep under several physics and position-measurement configs with repeats, one log folder per run |
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
bash /work/run_sweep.sh                  # all configs, 3 runs each (10 configs = 30 flights)
bash /work/run_sweep.sh physics          # only baseline, noise, downwash, noise_downwash
bash /work/run_sweep.sh pose             # only the position-measurement configs
python3 /work/compare_runs.py            # table of mean +/- std per config
```

`run_sweep.sh` starts the headless sim, flies the sweep, saves the logs to
`runs/<config>_<k>/`, and stops the sim, for each run. If a run fails, rerun with
`SKIP_EXISTING=1` (for example `SKIP_EXISTING=1 bash /work/run_sweep.sh`) to redo only
the runs that have no logs yet. `REPEATS=5` changes the repeat
count, and naming configs (`bash run_sweep.sh baseline noise`) runs a subset. To sweep
another knob, add a row to the `FLAGS` table at the top of the script (for example
`[wind]="--wind-speed 0.3"`).

`compare_runs.py` measures, on airborne samples only: tracking error against the
commanded zig-zag (`track_rmse`, `track_max`), worst waypoint miss, height overshoot,
time airborne, and the smallest gap between any two drones. It writes
`runs/summary_per_run.csv` and `runs/summary_by_config.csv`.

### Position noise, latency and dropout

Stock CrazySim hands the firmware the exact ground-truth position 1000 times a second.
`--sensor-noise` only affects the IMU and barometer, not the position, and
`--downwash` only acts between drones closer than 0.3 m (the lawnmower keeps them about
1 m apart), so neither one models the main way a real positioning system differs from
the sim. `apply_pose_patch.py` adds that, through environment variables read by the sim:

| Variable | Meaning | Default |
| --- | --- | --- |
| `CF_POSE_RATE_HZ` | Position updates per second (0 = every physics step) | 0 |
| `CF_POSE_NOISE_STD` | Gaussian noise on x, y, z, metres per axis | 0 |
| `CF_POSE_LATENCY_MS` | Delay before an update reaches the firmware | 0 |
| `CF_POSE_DROPOUT` | Fraction of updates lost (0 to 1) | 0 |
| `CF_POSE_SEED` | Integer for reproducible noise and dropouts (`run_sweep.sh` uses the run number) | random |

With none set, the sim behaves exactly like stock CrazySim. Latency and rate run on
simulation time, so results don't depend on how fast the computer is. Try it by hand:

```bash
CF_POSE_RATE_HZ=30 CF_POSE_NOISE_STD=0.02 bash /work/launch_headless.sh -m cf2x_T350 -f surg_four.txt
```

The sweep's `pose` configs are `pose30` (30 Hz, nothing else degraded), `pose_noise1`,
`pose_noise3`, `pose_lat50`, `pose_drop30` and `pose_all`. Compare them with `pose30`,
not `baseline`, since dropping from 1 kHz to 30 Hz changes the flight on its own;
`compare_runs.py` does this automatically in its "change vs reference" table. The
levels in `run_sweep.sh` are placeholders: replace them with the noise, latency, rate and
dropout you measure on the real Crazyflies. Run just these with
`bash /work/run_sweep.sh pose`.

Two caveats. The firmware's position filter assumes 1 cm of noise on external
positions by default (parameter `locSrv.extPosStdDev`), so large injected noise is
trusted more than it deserves; setting that parameter to your measured noise level is
part of making the sim match the hardware. And the patch is tied to this version of
CrazySim: if a CrazySim update changes the code it edits, the patch refuses to apply
rather than guessing.

### Ground truth: where the drones really are

The flight script logs what the drone *believes* (its position estimate), because that is
all real hardware can log. In the sim you can also record where each drone really is.
`run_sweep.sh` sets `CF_TRUTH_DIR` for every run, and the patched sim writes
`truth_drone<i>.csv` (columns `sim_s, wall_s, x, y, z`, 100 Hz) into the run folder next
to the estimate logs. The flight script stamps its estimate samples with the same
wall clock, which is how the two get matched, even though the sim doesn't run at exactly
real time. `compare_runs.py` then prints a second table:

| Column | Meaning |
| --- | --- |
| `true_track_rmse`, `true_track_max` | How far the drone really strays from the commanded path |
| `true_z_over` | Real height overshoot |
| `true_sep_min` | Real closest approach between any two drones |
| `est_err_rms`, `est_err_max` | Estimation error: estimate vs truth at the same moment |
| `est_lag_ms` | How far the estimate trails the truth (includes a few ms of logging delay, so compare configs with each other) |

Why it matters: the estimate-based numbers (`track_*`, `z_over`, `sep_min`) mix real
motion with measurement noise in the log itself, and the extreme-value ones are biased
upward by noise. The truth columns separate the two: a degraded position sensor can
hurt through a bad estimate (`est_err_*`) or through the drone actually flying worse
(`true_*`). Runs made before this was added have no truth logs; they still score on the
estimate table and show blanks in the truth table. Set `CF_TRUTH_RATE_HZ` to change
the 100 Hz logging rate.

#### Simulator speed

The firmware runs in real time, so if the simulator can't keep up (it often runs at
0.8 to 0.9 of real time) every config flies worse than it would on a faster machine.
`compare_runs.py` therefore prints a speed table: `rtf` (simulated seconds per wall
second while flying; 1.00 is real time) and `stall_pct` (% of 10 ms steps that took
over 20 ms). Compare configs only within one sweep, and treat absolute errors as
upper bounds when `rtf` is below about 0.9. To check whether truth logging itself costs
speed, run `NO_TRUTH=1 OUT=/work/runs_ab bash /work/run_sweep.sh baseline` and then
`python3 /work/compare_runs.py /work/runs_ab`. `TRUTH_RATE_HZ` (default 100) sets the
logging rate for `run_sweep.sh`.

When the real drones fly, log them to the same CSV format (`t_s,x,y,z` per drone,
`drone0.csv` to `drone3.csv`) in `runs/hw_1/`, `runs/hw_2/`, and so on. Rerunning
`compare_runs.py` then puts `hw` in the same estimate table as the simulated configs
(`wall_s` is optional for hardware; there is no truth table without motion capture).

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
