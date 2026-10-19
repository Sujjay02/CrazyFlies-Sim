#!/usr/bin/env bash
# Run INSIDE the container (bash build_crazysim.sh).
# Follows the current CrazySim README:
#   - official cflib installed from source (not pip release, not a submodule)
#   - firmware SITL built with cmake/make
#   - MuJoCo backend: pip install mujoco numpy tomli (+ drone-models submodule)
set -euo pipefail
cd /work

if [ ! -d CrazySim ]; then
    git clone https://github.com/gtfactslab/CrazySim.git --recursive
fi

if [ ! -d crazyflie-lib-python ]; then
    git clone https://github.com/bitcraze/crazyflie-lib-python.git
fi

# Ubuntu 22.04's pip is too old for cflib's build backend in editable mode,
# so upgrade the build tools and do a regular (non-editable) install.
python3 -m pip install --user -U pip setuptools wheel
(cd crazyflie-lib-python && python3 -m pip install --user .)

# tomli is needed on Python < 3.11 (Ubuntu 22.04 ships 3.10)
python3 -m pip install --user mujoco numpy tomli Jinja2

cd CrazySim/crazyflie-firmware

# drone-models is a submodule of tools/crazyflie-simulation (nested), normally
# already fetched by the recursive clone. Only fetch it if it is empty.
SIM=tools/crazyflie-simulation
if [ ! -f "$SIM/simulator_files/mujoco/drone-models/pyproject.toml" ]; then
    (cd "$SIM" && git submodule update --init simulator_files/mujoco/drone-models)
fi

mkdir -p sitl_make/build
(cd sitl_make/build && cmake .. && make all -j4)

# Our four-drone spawn list (1 m apart along x)
cp /work/agents.txt "$SIM/drone_spawn_list/surg_four.txt"

echo
echo "Build finished. Launch from CrazySim/crazyflie-firmware, e.g.:"
echo "  cd /work/CrazySim/crazyflie-firmware"
echo "  bash tools/crazyflie-simulation/simulator_files/mujoco/launch/sitl_multiagent_text.sh -m cf2x_T350 -f surg_four.txt"
