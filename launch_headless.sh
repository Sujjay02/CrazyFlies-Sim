#!/usr/bin/env bash
# Run INSIDE the container. Launches the CrazySim MuJoCo multi-agent sim with no
# viewer window (the stock script hard-codes --vis, which needs an X display).
#
# Usage:  bash /work/launch_headless.sh -m cf2x_T350 -f surg_four.txt [--sensor-noise ...]
set -euo pipefail
L=/work/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/launch

# Spawn list for the four drones
cp /work/agents.txt /work/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/drone_spawn_list/surg_four.txt

# Copy of the stock launcher with the --vis line removed (must live in the same
# folder, because the script locates crazysim.py relative to itself).
sed '/^[[:space:]]*--vis \\$/d' "$L/sitl_multiagent_text.sh" > "$L/sitl_multiagent_text_headless.sh"

exec bash "$L/sitl_multiagent_text_headless.sh" "$@"
