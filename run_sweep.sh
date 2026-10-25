#!/usr/bin/env bash
# Run INSIDE the container, from one terminal. For each physics configuration it
# starts the headless sim, flies the lawnmower sweep, saves the logs, and stops
# the sim, repeated REPEATS times (sensor noise is random, so one run is not enough).
#
# Usage:
#   bash /work/run_sweep.sh                      # all configs below
#   bash /work/run_sweep.sh pose                 # only the position-measurement configs
#   bash /work/run_sweep.sh baseline noise       # only these configs
#   REPEATS=5 bash /work/run_sweep.sh            # more repeats
#   SKIP_EXISTING=1 bash /work/run_sweep.sh ...  # redo only runs that have no logs yet
#   NO_TRUTH=1 OUT=/work/runs_ab bash /work/run_sweep.sh baseline
#                                                # no ground-truth logging (A/B check of its cost)
#
# Output: /work/runs/<config>_<k>/{drone0..3.csv, sim.log, fly.log}
# Then:   python3 /work/compare_runs.py
#
# Stop any sim you already have running (Ctrl-C in its terminal) before starting.
set -uo pipefail

# The sim's prints go to a file here, and Python buffers file output by default,
# so without this the "firmware connected" lines never show up and the wait loop
# below times out.
export PYTHONUNBUFFERED=1

REPEATS=${REPEATS:-3}
OUT=${OUT:-/work/runs}
MODEL=${MODEL:-cf2x_T350}
SPAWN_FILE=${SPAWN_FILE:-surg_four.txt}

# Don't let CF_POSE_* values left in your shell leak into the configs below.
unset CF_POSE_RATE_HZ CF_POSE_NOISE_STD CF_POSE_LATENCY_MS CF_POSE_DROPOUT CF_POSE_SEED
unset CF_TRUTH_DIR CF_TRUTH_RATE_HZ
# Set NO_TRUTH=1 to skip the ground-truth log entirely; TRUTH_RATE_HZ changes its rate.
NO_TRUTH=${NO_TRUTH:-0}
TRUTH_RATE_HZ=${TRUTH_RATE_HZ:-100}

# name -> extra CrazySim flags. Add rows here to sweep more knobs
# (e.g. [wind]="--wind-speed 0.3", [ground]="--ground-effect").
declare -A FLAGS=(
    [baseline]=""
    [noise]="--sensor-noise"
    [downwash]="--downwash"
    [noise_downwash]="--sensor-noise --downwash"
)

# name -> position-measurement settings (see apply_pose_patch.py). Every "pose" config
# runs at 30 Hz, so compare them with pose30, NOT with baseline: the baseline sends
# the exact pose at 1 kHz, and the rate drop alone changes the flight a little.
# The levels are placeholders, not measured values: replace them with what your
# hardware shows (noise std in m, latency in ms, dropout as a fraction 0 to 1).
declare -A ENVS=(
    [pose30]="CF_POSE_RATE_HZ=30"
    [pose_noise1]="CF_POSE_RATE_HZ=30 CF_POSE_NOISE_STD=0.01"
    [pose_noise3]="CF_POSE_RATE_HZ=30 CF_POSE_NOISE_STD=0.03"
    [pose_lat50]="CF_POSE_RATE_HZ=30 CF_POSE_LATENCY_MS=50"
    [pose_drop30]="CF_POSE_RATE_HZ=30 CF_POSE_DROPOUT=0.3"
    [pose_all]="CF_POSE_RATE_HZ=30 CF_POSE_NOISE_STD=0.02 CF_POSE_LATENCY_MS=40 CF_POSE_DROPOUT=0.1"
)

PHYSICS_ORDER=(baseline noise downwash noise_downwash)
POSE_ORDER=(pose30 pose_noise1 pose_noise3 pose_lat50 pose_drop30 pose_all)

# Arguments are config names or the groups "physics" and "pose".
ORDER=()
if [ "$#" -eq 0 ]; then
    ORDER=("${PHYSICS_ORDER[@]}" "${POSE_ORDER[@]}")
else
    for arg in "$@"; do
        case "$arg" in
            physics) ORDER+=("${PHYSICS_ORDER[@]}") ;;
            pose)    ORDER+=("${POSE_ORDER[@]}") ;;
            *)       ORDER+=("$arg") ;;
        esac
    done
fi

# pkill may not exist in the minimal image, so scan /proc instead.
# Matches the firmware processes (comm "cf2") and the Python sim, whose command line is
# "python3 .../mujoco/launch/../crazysim.py ...". Only processes whose command starts
# with "python" are matched on the script name, so this shell is never hit.
# The sim holds UDP ports 19950+ until it exits, so wait for it to die (SIGKILL if it
# ignores SIGTERM) before the next run tries to bind them.
sim_pids() {
    local p pid cmd
    for p in /proc/[0-9]*; do
        pid=${p#/proc/}
        [ "$pid" = "$$" ] && continue
        [ "$(cat "$p/comm" 2>/dev/null)" = "cf2" ] && { echo "$pid"; continue; }
        cmd=$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null) || continue
        case "$cmd" in
            python*crazysim.py*) echo "$pid" ;;
        esac
    done
}

stop_sim() {
    local pids i
    pids=$(sim_pids)
    [ -z "$pids" ] && return 0
    # shellcheck disable=SC2086
    kill $pids 2>/dev/null || true
    for i in 1 2 3 4 5 6 7 8 9 10; do
        sleep 1
        [ -z "$(sim_pids)" ] && { sleep 1; return 0; }
    done
    # shellcheck disable=SC2046
    kill -9 $(sim_pids) 2>/dev/null || true
    sleep 2
}

run_one() {
    local name=$1 k=$2 flags=$3 envs=$4
    local dir="$OUT/${name}_${k}"
    if [ "${SKIP_EXISTING:-0}" = "1" ]; then
        local have=1 i
        for i in 0 1 2 3; do
            [ -s "$dir/drone$i.csv" ] && [ "$(wc -l < "$dir/drone$i.csv")" -gt 5 ] || have=0
        done
        if [ "$have" = "1" ]; then
            echo "=== $name run $k: already done, skipped ==="
            return 0
        fi
    fi
    mkdir -p "$dir"
    echo "=== $name run $k  (flags: ${flags:-none}; pose: ${envs:-stock}) ==="

    stop_sim
    # The run number seeds the pose noise: repeats differ, but the sweep can be redone
    # exactly. env with no assignments is harmless for configs that set none.
    # shellcheck disable=SC2086
    # CF_TRUTH_DIR makes the sim write each drone's true position into the run folder.
    local truth=""
    [ "$NO_TRUTH" = "1" ] || truth="CF_TRUTH_DIR=$dir CF_TRUTH_RATE_HZ=$TRUTH_RATE_HZ"
    # shellcheck disable=SC2086
    env $envs $truth CF_POSE_SEED="$k" nohup bash /work/launch_headless.sh \
        -m "$MODEL" -f "$SPAWN_FILE" $flags > "$dir/sim.log" 2>&1 &

    # Wait until all four firmware instances have connected to the sim.
    local i n=0
    for i in $(seq 1 90); do
        # Count messages, not lines: the sim prints from several threads, so two
        # messages sometimes land on one line and grep -c would undercount.
        n=$(grep -o "firmware connected" "$dir/sim.log" 2>/dev/null | wc -l)
        [ "${n:-0}" -ge 4 ] && break
        sleep 1
    done
    if [ "${n:-0}" -lt 4 ]; then
        echo "  FAILED: only ${n:-0}/4 firmware instances connected, see $dir/sim.log"
        echo "  last lines of sim.log:"
        tail -n 5 "$dir/sim.log" 2>/dev/null | sed 's/^/    /'
        stop_sim
        return 1
    fi
    sleep 5   # let the firmware finish sensor calibration

    if LOG_DIR="$dir" timeout 240 python3 /work/fly_lawnmower.py > "$dir/fly.log" 2>&1; then
        echo "  ok"
    else
        echo "  FAILED: flight script error or timeout, see $dir/fly.log"
    fi
    stop_sim
}

cd /work
for name in "${ORDER[@]}"; do
    if [ -z "${FLAGS[$name]+set}" ] && [ -z "${ENVS[$name]+set}" ]; then
        echo "Unknown config '$name'."
        echo "Known: ${PHYSICS_ORDER[*]} ${POSE_ORDER[*]}   (groups: physics, pose)"
        exit 1
    fi
    for k in $(seq 1 "$REPEATS"); do
        run_one "$name" "$k" "${FLAGS[$name]-}" "${ENVS[$name]-}"
    done
done

echo
echo "Done. Compare with: python3 /work/compare_runs.py"
