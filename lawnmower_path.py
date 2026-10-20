"""Shared definition of the lawnmower sweep, used by fly_lawnmower.py (to fly it)
and compare_runs.py (to score logged flights against it). No cflib import here,
so the analysis also runs on a machine without the Crazyflie library."""

N_DRONES = 4
ARENA = 4.0      # metres, square arena
HEIGHT = 0.5     # metres, commanded flight height
VELOCITY = 0.4   # m/s
ROWS = 5         # sweep passes per strip


def spawn(i):
    """Spawn (x, y) of drone i; must match agents.txt (0,0 / 1,0 / 2,0 / 3,0)."""
    return (float(i), 0.0)


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
