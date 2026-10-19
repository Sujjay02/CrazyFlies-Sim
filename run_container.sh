#!/usr/bin/env bash
# Start an interactive shell in the CrazySim container.
# Host networking so the cflib UDP ports (19850+) are reachable from the host too.
set -euo pipefail
exec docker run --rm -it --net=host \
    -v "$HOME/crazyflie-sim":/work \
    -w /work \
    crazysim:22.04 bash
