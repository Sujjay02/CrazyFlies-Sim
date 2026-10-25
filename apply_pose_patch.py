#!/usr/bin/env python3
"""Patch CrazySim's MuJoCo simulator so the position measurement sent to the
firmware can be degraded the way a real positioning system degrades it.

Stock CrazySim sends the exact ground-truth pose to the firmware on every physics
step (1 kHz). --sensor-noise only corrupts the IMU and barometer, never the
position. This patch adds a "pose link" between the sim and the firmware with four
settings, read from environment variables so the stock launcher script (which drops
flags it does not know) needs no changes:

  CF_POSE_RATE_HZ      measurement rate in Hz; 0 (default) = every physics step
  CF_POSE_NOISE_STD    Gaussian noise on x, y, z in metres, per axis; default 0
  CF_POSE_LATENCY_MS   delay before a measurement reaches the firmware; default 0
  CF_POSE_DROPOUT      probability (0 to 1) that a measurement is lost; default 0
  CF_POSE_SEED         integer; set it for reproducible noise and dropouts

With none of them set the behaviour is identical to stock CrazySim. Latency and rate
use simulation time, so results do not depend on how fast the host runs. Dropout is
applied per measurement (after the rate limit), so combine it with CF_POSE_RATE_HZ:
at 1 kHz a 30% dropout still leaves hundreds of measurements per second.

The same script also adds a ground-truth logger. When CF_TRUTH_DIR is set, each drone's
true simulated position is written to <CF_TRUTH_DIR>/truth_drone<i>.csv (columns sim_s,
wall_s, x, y, z) at CF_TRUTH_RATE_HZ (default 100). Real hardware cannot give you this,
and it separates what the drone believes (the estimate the flight script logs) from where
it really is.

The patch is idempotent, can upgrade a clone patched by an older version, and keeps a
backup at crazysim.py.orig.

Usage:  python3 apply_pose_patch.py [path/to/crazysim.py]
"""
import os
import shutil
import sys

DEFAULT_PATH = ("/work/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/"
                "simulator_files/mujoco/crazysim.py")
MARKER_POSE = "# --- pose-link patch (apply_pose_patch.py) ---"
MARKER_TRUTH = "# --- truth-log patch (apply_pose_patch.py) ---"

POSE_LINK = '''\
# --- pose-link patch (apply_pose_patch.py) ---
class PoseLink:
    """Degrades the position measurement sent to the firmware: rate limit,
    Gaussian noise, fixed latency and random dropout. Configured from CF_POSE_*
    environment variables; with none set it sends the stock exact pose every step."""

    def __init__(self, rate_hz=0.0, noise_std=0.0, latency_ms=0.0, dropout=0.0,
                 rng=None):
        if rate_hz < 0 or noise_std < 0 or latency_ms < 0 or not 0.0 <= dropout <= 1.0:
            raise ValueError('pose link settings out of range: '
                             f'rate={rate_hz} noise={noise_std} '
                             f'latency_ms={latency_ms} dropout={dropout}')
        self.rate_hz = rate_hz
        self.noise_std = noise_std
        self.latency_s = latency_ms / 1000.0
        self.dropout = dropout
        self._rng = rng if rng is not None else np.random.default_rng()
        self._period = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self._acc = 0.0
        self._pending = []   # (release_time, packet), in release order
        self.passthrough = (rate_hz == 0 and noise_std == 0
                            and latency_ms == 0 and dropout == 0)

    @classmethod
    def from_env(cls, agent_id):
        def num(name, default=0.0):
            raw = os.environ.get(name, '').strip()
            return float(raw) if raw else default
        seed = os.environ.get('CF_POSE_SEED', '').strip()
        rng = np.random.default_rng(int(seed) * 1000 + agent_id) if seed else None
        return cls(rate_hz=num('CF_POSE_RATE_HZ'),
                   noise_std=num('CF_POSE_NOISE_STD'),
                   latency_ms=num('CF_POSE_LATENCY_MS'),
                   dropout=num('CF_POSE_DROPOUT'), rng=rng)

    def describe(self):
        if self.passthrough:
            return 'stock (exact pose every physics step)'
        rate = f'{self.rate_hz:g} Hz' if self.rate_hz > 0 else 'every step'
        return (f'rate={rate}  noise_std={self.noise_std:g} m  '
                f'latency={self.latency_s * 1000:g} ms  dropout={self.dropout:g}')

    def step(self, sock, addr, t, dt, pos, quat):
        """Call once per physics step with the current simulation time t."""
        if self.passthrough:
            sock.sendto(make_pose_packet(pos, quat), addr)
            return
        if self._period > 0.0:
            self._acc += dt
            measure = self._acc + 1e-9 >= self._period
            if measure:
                self._acc -= self._period
        else:
            measure = True
        if measure and not (self.dropout > 0.0 and self._rng.random() < self.dropout):
            p = pos
            if self.noise_std > 0.0:
                p = pos + self._rng.normal(0.0, self.noise_std, 3)
            self._pending.append((t + self.latency_s, make_pose_packet(p, quat)))
        while self._pending and self._pending[0][0] <= t + 1e-9:
            sock.sendto(self._pending.pop(0)[1], addr)
# --- end pose-link patch ---


'''

TRUTH_LOG = '''\
# --- truth-log patch (apply_pose_patch.py) ---
class TruthLog:
    """Writes the simulator's TRUE position of one drone to
    <CF_TRUTH_DIR>/truth_drone<i>.csv (columns sim_s, wall_s, x, y, z), at
    CF_TRUTH_RATE_HZ (default 100). wall_s is time.time(), the same clock the flight
    script stamps its estimate log with, so estimates and truth can be matched in time
    whatever the simulation speed. Does nothing when CF_TRUTH_DIR is not set."""

    def __init__(self, path=None, dt=0.001, rate_hz=100.0):
        self._f = None
        self._n = 0
        self._every = max(1, int(round(1.0 / (rate_hz * dt)))) if rate_hz > 0 else 1
        if path:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            # line-buffered, so rows survive the sim being stopped with SIGTERM
            self._f = open(path, 'w', buffering=1)
            self._f.write('sim_s,wall_s,x,y,z\\n')

    @classmethod
    def from_env(cls, agent_id, dt):
        d = os.environ.get('CF_TRUTH_DIR', '').strip()
        rate = os.environ.get('CF_TRUTH_RATE_HZ', '').strip()
        path = os.path.join(d, f'truth_drone{agent_id}.csv') if d else None
        return cls(path, dt, float(rate) if rate else 100.0)

    @property
    def enabled(self):
        return self._f is not None

    def log(self, sim_t, pos):
        if self._f is None:
            return
        self._n += 1
        if self._n % self._every:
            return
        self._f.write(f'{sim_t:.4f},{time.time():.4f},'
                      f'{pos[0]:.5f},{pos[1]:.5f},{pos[2]:.5f}\\n')
# --- end truth-log patch ---


'''

# (anchor text that must occur exactly once, replacement)
POSE_EDITS = [
    (
        'class DroneAgent:\n    """Per-drone state: UDP sockets, motor dynamics, sensor indices."""\n',
        POSE_LINK + 'class DroneAgent:\n'
        '    """Per-drone state: UDP sockets, motor dynamics, sensor indices."""\n',
    ),
    (
        "        self._camera = camera_renderer\n\n"
        "        print(f'[crazysim] Agent {agent_id}: fw_port={fw_port}  cflib_port={cflib_port}')\n",
        "        self._camera = camera_renderer\n"
        "        self._pose_link = PoseLink.from_env(agent_id)  # pose-link patch\n"
        "        if agent_id == 0:\n"
        "            print(f'[crazysim] Pose link: {self._pose_link.describe()}')\n\n"
        "        print(f'[crazysim] Agent {agent_id}: fw_port={fw_port}  cflib_port={cflib_port}')\n",
    ),
    (
        "            self._sock.sendto(make_pose_packet(pos, quat), self._firmware_addr)\n",
        "            self._pose_link.step(self._sock, self._firmware_addr, self.data.time,\n"
        "                                 self.dt, pos, quat)  # pose-link patch\n",
    ),
]

TRUTH_EDITS = [
    (
        'class DroneAgent:\n    """Per-drone state: UDP sockets, motor dynamics, sensor indices."""\n',
        TRUTH_LOG + 'class DroneAgent:\n'
        '    """Per-drone state: UDP sockets, motor dynamics, sensor indices."""\n',
    ),
    (
        "        print(f'[crazysim] Agent {agent_id}: fw_port={fw_port}  cflib_port={cflib_port}')\n",
        "        self._truth = TruthLog.from_env(agent_id, dt)  # truth-log patch\n"
        "        if agent_id == 0 and self._truth.enabled:\n"
        "            print(f\"[crazysim] Truth log: {os.environ.get('CF_TRUTH_DIR')}\")\n\n"
        "        print(f'[crazysim] Agent {agent_id}: fw_port={fw_port}  cflib_port={cflib_port}')\n",
    ),
    (
        "        acc, gyro = self.read_imu()\n        pos, quat = self.read_pose()\n",
        "        acc, gyro = self.read_imu()\n        pos, quat = self.read_pose()\n"
        "        self._truth.log(self.data.time, pos)  # truth-log patch\n",
    ),
]

# (name, marker that says it is already applied, edits). Applied in this order; the
# groups are independent, so a clone patched by an older version just gets the new one.
GROUPS = [
    ("pose-link", MARKER_POSE, POSE_EDITS),
    ("truth-log", MARKER_TRUTH, TRUTH_EDITS),
]


def apply(path):
    with open(path) as fh:
        src = fh.read()
    applied, present = [], []
    for name, marker, edits in GROUPS:
        if marker in src:
            present.append(name)
            continue
        for n, (anchor, repl) in enumerate(edits, 1):
            count = src.count(anchor)
            if count != 1:
                print(f"ERROR: {name} patch point {n} found {count} times in {path} "
                      "(expected 1). This CrazySim version differs from the one the "
                      "patch was written for, so nothing was changed.", file=sys.stderr)
                return 1
            src = src.replace(anchor, repl)
        applied.append(name)
    if not applied:
        print(f"patches already applied ({', '.join(present)}): {path}")
        return 0
    backup = path + ".orig"
    if not os.path.exists(backup):
        shutil.copyfile(path, backup)
    with open(path, "w") as fh:
        fh.write(src)
    print(f"patches applied ({', '.join(applied)}): {path} (backup: {backup})")
    return 0


if __name__ == "__main__":
    sys.exit(apply(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH))
