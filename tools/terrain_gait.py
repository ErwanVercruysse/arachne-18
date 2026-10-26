"""The trained walking policy (training/gait.py) as a live controller, with terrain adaptation on top.

The real robot used to play gait frames that were planned once from the standing pose. This runs the
policy itself every 10 ms: the same oscillator, foot paths, IK, target filter and slew limit as in
training/evaluate.py, fed with the IMU (gravity + gyro in the body frame) and a velocity command.

On top of the policy, three adaptations that only use the IMU (the policy's own observation contract):
  level  - the body is kept level: the foot heights get a slowly integrated plane offset (slopes, steps);
  rough  - shaking of the body (roll/pitch rate) raises the feet higher, with a broader swing, and slows down;
  fall   - more than FALL_DEG of tilt stops the gait (the caller then holds or lies down).
With adapt=False the output is exactly the trained policy (tested against training/gait.target).

    g = TerrainGait(params, low, high)                  # low/high: joint ranges [18] in model rad
    q = g.step(command, gravity, gyro, dt)              # (vx, vy, wz), body-frame unit gravity, rad/s
Gravity comes from tools/imu.py (GravityFilter, body frame); the simulator tests use the same filter.
"""
import math, sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'training'))
import gait                                                           # noqa: E402

DT = .01                       # the policy's control step (s); longer steps are run as several of these
FILTER, SLEW = .35, .047171    # target low-pass and slew limit per step, as in training (4.717 rad/s)
FALL_DEG = 30.
NEUTRAL = gait.COXA + (gait.R0 + 0.)*gait.RADIAL                      # nominal foot xy per leg [6,2]


class TerrainGait:
    GAINS = dict(
        level=2.5,             # 1/s: plane slope rate per unit of measured tilt (gravity x/y)
        level_tau=1.0,         # s: only lasting tilt is levelled (a slope), not the rocking over a step
        level_dead=1.0,        # deg of lasting tilt that is left alone (the IMU's own error while walking is ~0.5)
        level_max=.040,        # m: largest foot height offset the levelling may use
        rough_tau=1.0,         # s: averaging of the body shaking
        rough_lo=.06,          # rad/s roll/pitch rate RMS where rough mode starts (flat walking: ~0.02)
        rough_hi=.20,          # rad/s where it is fully on
        lift_max=2.0,          # foot lift x this in full rough mode (15 mm -> 30 mm)
        slow_max=0.,           # command scaled down by this fraction in full rough mode (off: costs progress)
        cmd_tau=.25,           # s: command filter (as in the training schedule)
    )

    def __init__(self, params, low, high, adapt=True, **gains):
        p = np.asarray(params, float)
        self.p0 = np.r_[p, 0.] if len(p) == 12 else p.copy()          # 13th = broad-lift blend
        self.blend0 = float(self.p0[12])
        self.low, self.high = np.asarray(low, float), np.asarray(high, float)
        self.adapt = adapt
        self.gains = dict(self.GAINS, **gains)
        self.reset()

    def reset(self, q_now=None):
        self.phase = np.zeros(1); self.t = 0.; self.cmd = np.zeros(3)
        self.sm = np.zeros((1, 18)) if q_now is None else np.asarray(q_now, float).reshape(1, 18)
        self.plane = np.zeros(2)                                       # dz/dx, dz/dy of the foot offsets
        self.g_slow = np.zeros(2)                                      # low-passed gravity x/y (lasting tilt)
        self.rough_ms = 0.; self.rough = 0.; self.fallen = False; self.tilt = 0.
        self.last = None

    def neutral(self):
        """Joint angles of the policy's stance at rest (t = 0, no command, level): where to step in to."""
        f = gait.plan(self.p0[None, :12 if self.blend0 == 0 else 13], np.zeros(1), np.zeros((1, 3)), None, 0.,
                      gravity=np.array([[0., 0., -1.]]))
        return gait.leg_ik(f['x'], f['y'], f['z'], low=self.low, high=self.high)[0]

    # ------------------------------------------------------------------ one control period
    def step(self, command, gravity, gyro, dt=DT, acc=None):
        """acc: accelerometer (body frame, g). Levelling then uses its slow average instead of the gravity
        estimate: walking accelerations average out, and a gyro bias cannot tilt the body."""
        n = max(1, int(round(dt/DT)))
        for _ in range(n): q = self._step(np.asarray(command, float), np.asarray(gravity, float), np.asarray(gyro, float),
                                          None if acc is None else np.asarray(acc, float))
        return q

    def _step(self, command, gravity, gyro, acc=None):
        G = self.gains
        self.tilt = math.degrees(math.acos(max(-1., min(1., -gravity[2]))))
        if self.tilt > FALL_DEG: self.fallen = True
        if self.fallen: command = np.zeros(3)
        p = self.p0.copy()
        if self.adapt:
            # body shaking -> rough-ground factor 0..1 (slow up, slow down)
            a = 1 - math.exp(-DT/G['rough_tau'])
            self.rough_ms += a*(float(gyro[0]**2 + gyro[1]**2) - self.rough_ms)
            r = (math.sqrt(self.rough_ms) - G['rough_lo'])/(G['rough_hi'] - G['rough_lo'])
            self.rough = min(1., max(0., r))
            p[2] *= 1 + (G['lift_max'] - 1)*self.rough
            p[12] = self.blend0 + (1 - self.blend0)*self.rough
            command = command*(1 - G['slow_max']*self.rough)
        self.cmd += (1 - math.exp(-DT/G['cmd_tau']) if G['cmd_tau'] > 0 else 1.)*(command - self.cmd)
        P = p[None, :] if self.adapt else self.p0[None, :12 if self.blend0 == 0 else 13]
        f = gait.plan(P, self.phase, self.cmd[None, :], None, self.t, gyro=gyro[None, :], gravity=gravity[None, :])
        z = f['z']
        if self.adapt:
            # levelling: integrate a plane offset of the foot heights until the body is level.
            # Nose down (+x gravity) -> front feet further down (z more negative).
            src = gravity if acc is None or np.linalg.norm(acc) < .2 else -acc/np.linalg.norm(acc)
            self.g_slow += (1 - math.exp(-DT/G['level_tau']))*(src[:2] - self.g_slow)
            e = self.g_slow; n = float(np.linalg.norm(e)); dead = math.radians(G['level_dead'])
            if n > dead: self.plane -= G['level']*DT*e*(n - dead)/n
            off = NEUTRAL @ self.plane
            worst = np.abs(off).max()
            if worst > G['level_max']: self.plane *= G['level_max']/worst; off = NEUTRAL @ self.plane
            z = z + off[None, :]
            desired = gait.leg_ik(f['x'], f['y'], z, low=self.low, high=self.high)
        else:
            desired = np.clip(gait.leg_ik(f['x'], f['y'], z), self.low, self.high)
        self.sm = np.clip(self.sm + FILTER*(desired - self.sm), self.sm - SLEW, self.sm + SLEW)
        self.phase = (self.phase + gait.frequency(P, self.cmd[None, :])*DT) % 1
        self.t += DT
        self.last = f
        return self.sm[0].copy()

    def status(self):
        return dict(tilt_deg=round(self.tilt, 2), rough=round(self.rough, 2), fallen=self.fallen,
                    level_mm=[round(float(v)*1000, 1) for v in NEUTRAL @ self.plane],
                    command=[round(float(v), 3) for v in self.cmd], phase=round(float(self.phase[0]), 3))
