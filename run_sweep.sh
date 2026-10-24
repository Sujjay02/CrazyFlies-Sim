#!/usr/bin/env bash
# Run INSIDE the container, from one terminal. For each physics configuration it
# starts the headless sim, flies the lawnmower sweep, saves the logs, and stops
# the sim, repeated REPEATS times (sensor noise is random, so one run is not enough).
#
# Usage:
#   bash /work/run_sweep.sh                      # all configs below
#   bash /work/run_sweep.sh baseline noise       # only these configs
#   REPEATS=5 bash /work/run_sweep.sh            # more repeats
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

# name -> extra CrazySim flags. Add rows here to sweep more knobs
# (e.g. "wind:--wind-speed 0.3", "ground:--ground-effect").
declare -A FLAGS=(
    [baseline]=""
    [noise]="--sensor-noise"
    [downwash]="--downwash"
    [noise_downwash]="--sensor-noise --downwash"
)
ORDER=(baseline noise downwash noise_downwash)

if [ "$#" -gt 0 ]; then
    ORDER=("$@")
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
    local name=$1 k=$2 flags=$3
    local dir="$OUT/${name}_${k}"
    mkdir -p "$dir"
    echo "=== $name run $k  (flags: ${flags:-none}) ==="

    stop_sim
    # shellcheck disable=SC2086
    nohup bash /work/launch_headless.sh -m "$MODEL" -f "$SPAWN_FILE" $flags \
        > "$dir/sim.log" 2>&1 &

    # Wait until all four firmware instances have connected to the sim.
    local i n=0
    for i in $(seq 1 90); do
        n=$(grep -c "firmware connected" "$dir/sim.log" 2>/dev/null || true)
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
    if [ -z "${FLAGS[$name]+set}" ]; then
        echo "Unknown config '$name'. Known: ${!FLAGS[*]}"
        exit 1
    fi
    for k in $(seq 1 "$REPEATS"); do
        run_one "$name" "$k" "${FLAGS[$name]}"
    done
done

echo
echo "Done. Compare with: python3 /work/compare_runs.py"
