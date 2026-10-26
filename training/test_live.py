"""Checks for the live controller and the moves that run on the robot (no servos or IMU needed)."""
import sys
import unittest
from pathlib import Path
import numpy as np

from gait import ROOT, load, target, rotate

sys.path.insert(0, str(ROOT/'tools'))
from terrain_gait import TerrainGait                                  # noqa: E402
from imu import GravityFilter                                         # noqa: E402
import policy_walk as pw                                              # noqa: E402
import fall_guard                                                     # noqa: E402
import grab                                                           # noqa: E402


class LiveController(unittest.TestCase):
    def setUp(self):
        self.p = load(ROOT/'training/policies/omni.json')

    def test_without_adaptation_it_is_the_trained_policy(self):
        g = TerrainGait(self.p, -np.ones(18)*9, np.ones(18)*9, adapt=False, cmd_tau=0.)
        rng = np.random.default_rng(3); phase = np.zeros(1); sm = np.zeros((1, 18))
        for i in range(300):
            cmd = np.array([.12, -.04, .2]); quat = np.r_[1, rng.normal(scale=.02, size=3)]; quat /= np.linalg.norm(quat)
            gyro = rng.normal(scale=.05, size=3); qpos = np.r_[0, 0, .1, quat, np.zeros(18)][None, :]
            grav = rotate(qpos[:, 3:7], np.array([[0., 0., -1.]]), inverse=True)[0]
            want = target(self.p[None], phase, cmd[None], qpos, i*.01, gyro=gyro[None])
            sm = np.clip(sm + .35*(want - sm), sm - .047171, sm + .047171)
            got = g.step(cmd, grav, gyro)
            np.testing.assert_allclose(got, sm[0], atol=1e-12)
            phase = (phase + __import__('gait').frequency(self.p[None], cmd[None])*.01) % 1

    def test_levelling_lowers_the_low_side(self):
        """Nose down (body pitched forward): the front feet must go further down to lift the nose."""
        g = TerrainGait(self.p, -np.ones(18)*9, np.ones(18)*9)
        s = np.sin(np.radians(5)); grav = np.array([s, 0, -np.sqrt(1 - s*s)])
        for _ in range(300): g.step(np.zeros(3), grav, np.zeros(3), acc=-grav)
        lvl = np.array(g.status()['level_mm'])
        self.assertLess(lvl[0], -5); self.assertLess(lvl[5], -5); self.assertGreater(lvl[2], 5)   # L1, L6 down; L3 up

    def test_gravity_filter_rejects_gyro_bias_slowly(self):
        f = GravityFilter()
        for _ in range(2000): g = f.update([0, 0, 1.], [.01, 0, 0], .01)
        self.assertLess(np.degrees(np.arccos(-g[2])), 1.)


class Robot(unittest.TestCase):
    def test_fk_inverts_ik(self):
        J = pw.Joints(); q = TerrainGait(load(ROOT/'training/policies/omni.json'), *J.policy).neutral()
        np.testing.assert_allclose(pw.ik(pw.fk(q), *J.real), q, atol=1e-9)

    def test_stepping_in_and_out_stays_reachable(self):
        J = pw.Joints(); q0 = J.stand_q(); qn = TerrainGait(load(ROOT/'training/policies/omni.json'), *J.policy).neutral()
        for a, b, first in ((q0, qn, False), (qn, q0, True)):
            frames = list(pw.restance(a, b, *J.real, 50, first))
            np.testing.assert_allclose(frames[-1], b, atol=1e-6)
            self.assertLess(np.abs(np.diff(frames, axis=0)).max()*50, 5.)        # rad/s, well below the servo

    def test_fall_detector(self):
        d = fall_guard.Detector(); events = []
        for k in range(150):
            a = 1. if k < 50 else .05 if k < 90 else 5. if k < 92 else 1.
            e = d.update(a, 2., .01)
            if e: events.append(e)
        self.assertEqual(events, ['free', 'impact'])
        walk = fall_guard.Detector()
        self.assertFalse(any(walk.update(a, 3., .01) for a in np.r_[np.full(3, .5), np.full(20, 1.4)]))

    def test_grab_plan_is_reachable_and_stable(self):
        J = pw.Joints(); f0 = pw.fk(J.stand_q()); home = {L: f0[i] for i, L in enumerate(grab.LEGS)}
        for feet, shift, label in grab.frames(grab.plan(home, .20, .13)):
            q, err = grab.solve(J, feet, shift)
            if label != 'knijpen': self.assertLess(err, .004, label)
            if label not in ('staan', 'L2 stap', 'L5 stap'): self.assertGreater(grab.support_margin(feet, shift), .015)


if __name__ == '__main__':
    unittest.main()
