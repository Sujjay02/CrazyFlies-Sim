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
stop_sim() {
    local p pid
    for p in /proc/[0-9]*; do
        pid=${p#/proc/}
        [ "$pid" = "$$" ] && continue
        if [ "$(cat "$p/comm" 2>/dev/null)" = "cf2" ] \
           || tr '\0' ' ' < "$p/cmdline" 2>/dev/null | grep -q 'mujoco/crazysim\.py'; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    sleep 3
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
