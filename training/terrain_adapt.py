"""Terrain trials of the live controller (tools/terrain_gait.py) in reference MuJoCo + full BAM.

The controller gets what the robot gets: an emulated MPU6050 (accelerometer from the body's real
acceleration, gyro with noise and a per-episode bias, the same complementary filter as on the Pi), the
velocity command and its own oscillator. No terrain map, no ground truth. Episodes are scored with the
terrain curriculum's own gates (curriculum.gate), so 'trained policy' and 'adaptive' are compared fairly.

    python training/terrain_adapt.py --stages higher_obstacles mixed_terrain --workers 4
    python training/terrain_adapt.py --extra --workers 4          # also harder terrain than the curriculum
    python training/terrain_adapt.py --check                      # adapt=False reproduces the policy exactly
"""
import argparse, json, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np

from gait import ROOT, load, target
from evaluate import evaluate
from curriculum import gate, commands

sys.path.insert(0, str(ROOT/'tools'))
from terrain_gait import TerrainGait                                   # noqa: E402
from imu import GravityFilter                                         # noqa: E402

CONFIG = json.loads((ROOT/'training/terrain_curriculum.json').read_text())
EXTRA = [   # harder than the curriculum; same gates as its last stage
    {"name": "plateaus_12mm", "terrain": {"kind": "terraces", "amplitude_m": .012}, "speed_m_s": .10, "tilt_rms_max_deg": 3.0, "clearance_std_max_mm": 6.0},
    {"name": "plateaus_16mm", "terrain": {"kind": "terraces", "amplitude_m": .016}, "speed_m_s": .10, "tilt_rms_max_deg": 3.5, "clearance_std_max_mm": 7.0},
    {"name": "bumps_8mm", "terrain": {"kind": "bumps", "amplitude_m": .008, "wavelength_m": [.10, .20]}, "speed_m_s": .10, "tilt_rms_max_deg": 3.0, "clearance_std_max_mm": 6.0},
    {"name": "slope_5deg", "terrain": {"kind": "slope", "slope_deg": 5.0}, "speed_m_s": .10, "tilt_rms_max_deg": 5.0, "clearance_std_max_mm": 8.0},
    {"name": "slope_8deg", "terrain": {"kind": "slope", "slope_deg": 8.0}, "speed_m_s": .10, "tilt_rms_max_deg": 6.0, "clearance_std_max_mm": 10.0},
    {"name": "cobbles_mild", "terrain": {"kind": "cobbles", "height_m": .002, "dome_m": .003, "joint_depth_m": .006}, "speed_m_s": .10, "tilt_rms_max_deg": 3.0, "clearance_std_max_mm": 6.0},
    {"name": "cobbles", "terrain": {"kind": "cobbles", "height_m": .004, "dome_m": .004, "joint_depth_m": .008}, "speed_m_s": .10, "tilt_rms_max_deg": 3.5, "clearance_std_max_mm": 7.0},
    {"name": "cobbles_rough", "terrain": {"kind": "cobbles", "height_m": .007, "dome_m": .006, "joint_depth_m": .012}, "speed_m_s": .10, "tilt_rms_max_deg": 4.0, "clearance_std_max_mm": 8.0},
    {"name": "mixed_10mm_4deg", "terrain": {"kind": "mixed", "amplitude_m": .010, "slope_deg": 4.0, "wavelength_m": [.10, .22]}, "speed_m_s": .10, "tilt_rms_max_deg": 4.0, "clearance_std_max_mm": 8.0},
]


class SimIMU:
    """MPU6050 seen from the body: specific force (g) and angular rate (rad/s), body frame."""

    def __init__(self, rng, gyro_noise=.005, gyro_bias=.01, acc_noise=.01):
        self.rng = rng; self.v = None
        self.bias = rng.normal(scale=gyro_bias, size=3); self.gn, self.an = gyro_noise, acc_noise
        self.filter = GravityFilter()

    def read(self, d, dt):
        q = d.qpos[3:7]; w, u = q[0], q[1:]
        R = np.array([[1-2*(u[1]**2+u[2]**2), 2*(u[0]*u[1]-w*u[2]), 2*(u[0]*u[2]+w*u[1])],
                      [2*(u[0]*u[1]+w*u[2]), 1-2*(u[0]**2+u[2]**2), 2*(u[1]*u[2]-w*u[0])],
                      [2*(u[0]*u[2]-w*u[1]), 2*(u[1]*u[2]+w*u[0]), 1-2*(u[0]**2+u[1]**2)]])
        v = d.qvel[:3].copy(); a = np.zeros(3) if self.v is None else (v - self.v)/dt; self.v = v
        acc = R.T @ ((a + [0, 0, 9.81])/9.81) + self.rng.normal(scale=self.an, size=3)
        gyro = d.qvel[3:6] + self.bias + self.rng.normal(scale=self.gn, size=3)
        return np.asarray(self.filter.update(acc, gyro, dt)), gyro, acc


def make_controller(params, adapt, gains=None, exact=False):
    """Factory for evaluate(controller=...). exact: true orientation, no IMU model (for --check)."""
    def factory(m, d, rng):
        joints = [m.joint(f'L{l}_{p}').id for l in range(1, 7) for p in ('yaw', 'hip', 'knee')]
        low, high = m.jnt_range[joints, 0], m.jnt_range[joints, 1]
        g = TerrainGait(params, low, high, adapt=adapt, **(gains or {}), **({'cmd_tau': 0.} if exact else {}))
        imu = SimIMU(rng)
        def act(d, command, t):
            if exact:
                from gait import rotate
                grav = rotate(d.qpos[None, 3:7], np.array([[0., 0., -1.]]), inverse=True)[0]
                return g.step(command, grav, d.qvel[3:6].copy())
            grav, gyro, acc = imu.read(d, .01)
            return g.step(command, grav, gyro, acc=acc)
        return act
    return factory


def episode(task):
    params, cmd, stage, seconds, seed, mode, gains = task
    ctrl = None if mode == 'policy' else make_controller(params, mode == 'adaptive', gains)
    r = evaluate(params, cmd, seconds, terrain=stage['terrain'], seed=seed, imu_noise=.15 if mode == 'policy' else 0.,
                 controller=ctrl)
    return r


def assess(pool, params, stage, modes, seeds, gains=None):
    out = {}
    for mode in modes:
        tasks = [(params, c, stage, CONFIG['episode_seconds'], s, mode, gains) for s in seeds for c in commands(stage, True)]
        rows = [dict(seed=t[4], command=t[1], failures=gate(r, stage, CONFIG), metrics=r) for t, r in zip(tasks, pool.map(episode, tasks))]
        ok = sum(not r['failures'] for r in rows)
        reasons = {}
        for r in rows:
            for f in r['failures']: reasons[f] = reasons.get(f, 0) + 1
        out[mode] = dict(passed=ok, total=len(rows), fraction=ok/len(rows), failures=reasons,
                         falls=sum(not r['metrics']['passed'] for r in rows),
                         tilt_max_deg=max(r['metrics']['tilt_max_deg'] for r in rows),
                         progress=float(np.mean([np.asarray(r['metrics']['measured_velocity'][:2]) @ np.asarray(r['command'][:2])
                                                 / max(1e-9, np.linalg.norm(r['command'][:2]))**2 for r in rows
                                                 if np.linalg.norm(r['command'][:2]) > .01] or [0.])),
                         vertical_velocity_rms=float(np.mean([r['metrics']['vertical_velocity_rms'] for r in rows])),
                         cases=rows)
        print(f"  {stage['name']:18s} {mode:9s} {ok:3d}/{len(rows)} pass  falls {out[mode]['falls']:2d}  "
              f"max tilt {out[mode]['tilt_max_deg']:5.2f} deg  progress {out[mode]['progress']*100:5.1f} %  "
              f"body bob {out[mode]['vertical_velocity_rms']*1000:4.1f} mm/s  {reasons}", flush=True)
    return out


def check():
    """adapt=False with the true orientation must reproduce the trained policy bit for bit."""
    p = load(ROOT/'training/policies/omni.json')
    a = evaluate(p, [.12, .03, .1], 3.)
    b = evaluate(p, [.12, .03, .1], 3., controller=make_controller(p, False, exact=True))
    keys = ['displacement_m', 'measured_velocity', 'tilt_max_deg', 'height_mean_m']
    same = all(np.allclose(a[k], b[k], rtol=0, atol=1e-12) for k in keys)      # floating-point rounding only
    print('same as the trained policy:', same, {k: (a[k], b[k]) for k in keys} if not same else '')
    return same


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--policy', type=Path, default=ROOT/'training/policies/omni.json')
    ap.add_argument('--stages', nargs='*', default=['higher_obstacles', 'mixed_terrain'])
    ap.add_argument('--extra', action='store_true', help='also run the harder EXTRA stages')
    ap.add_argument('--modes', nargs='*', default=['policy', 'adaptive'])
    ap.add_argument('--seeds', nargs='*', type=int, default=[701, 702, 703])
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    if a.check: raise SystemExit(0 if check() else 1)
    params = load(a.policy)
    stages = [s for s in CONFIG['stages'] if s['name'] in a.stages] + (EXTRA if a.extra else [])
    result = {}
    with ProcessPoolExecutor(a.workers) as pool:
        for st in stages: result[st['name']] = assess(pool, params, st, a.modes, a.seeds)
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(dict(policy=str(a.policy.relative_to(ROOT)), seeds=a.seeds, gains=TerrainGait.GAINS,
                                         stages=result), indent=1, default=float) + '\n')


if __name__ == '__main__': main()
