"""Falling off a table in simulation: which reaction hurts the robot least?

Physics + servo model of the browser simulation (tools/sim_world.py: measured joint ranges, STS3215 torque
limit register, gear friction, 3S LiPo). The robot stands, then leaves the table edge in one of several ways.
The reaction starts when tools/fall_guard.py would notice: free fall seen for 40 ms, plus 20 ms of bus delay.

Scored on what breaks things:
  gear    - peak torque through each servo's gears (motor + gear friction), Nm; STS3215 stall is ~2.9 Nm
  shock   - peak acceleration of the body (Pi, battery, IMU), g
  chassis - peak ground force on anything that is not a foot (shell, chassis, servos), N

    python training/fall_test.py --height .75
"""
import argparse, json, math, sys, time
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import sim_world as sw                                                # noqa: E402
import fall_guard                                                     # noqa: E402

SCENARIOS = {   # initial rotation when it leaves the table (deg, deg/s): walked off the front, the side, or dropped level
    'level': dict(pitch=0, rate=(0, 0, 0)),
    'nose_first': dict(pitch=25, rate=(0, 120, 0)),
    'side': dict(roll=-25, rate=(-120, 0, 0)),
}


def run(w, scenario, reaction, height, detect=.06, seconds=1.4, rng=None):
    m, d = w.model, w.data
    with w.lock:
        w.paused = True; w.supply_key = 'lipo3s'
        w.reset('stand'); w.v = sw.SUPPLIES['lipo3s']['v']
        sc = dict(SCENARIOS[scenario])
        if rng is not None:                       # a different fall every trial: angles, tumble rate, walking speed
            sc = dict(roll=sc.get('roll', 0) + rng.uniform(-8, 8), pitch=sc.get('pitch', 0) + rng.uniform(-8, 8),
                      rate=np.asarray(sc['rate'], float)*rng.uniform(.6, 1.4) + rng.uniform(-20, 20, 3))
        r, p = math.radians(sc.get('roll', 0)), math.radians(sc.get('pitch', 0))
        q = np.zeros(4); mujoco.mju_euler2Quat(q, [r, p, 0], 'xyz')
        d.qpos[3:7] = q; d.qpos[2] += height; d.qvel[:] = 0; d.qvel[3:6] = np.radians(sc['rate'])
        if rng is not None: d.qvel[0] = rng.uniform(0, .12)
        mujoco.mj_forward(m, d)
        dt = m.opt.timestep*w.SUB; t = 0.; done = False
        feet = {m.geom(f'{L}_foot_contact').id for L in sw.LEGS}
        gear = np.zeros(18); shock = 0.; chassis = 0.; v_prev = d.qvel[:3].copy(); f6 = np.zeros(6)
        while t < seconds:
            if not done and t >= detect:
                fall_guard.apply_sim(w, reaction); done = True
            w._step(m.opt.timestep); t += dt
            a = (d.qvel[:3] - v_prev)/dt; v_prev = d.qvel[:3].copy()
            if t > .02: shock = max(shock, float(np.linalg.norm(a + [0, 0, 9.81])/9.81))
            # torque through the gears: motor torque + gear friction (friction constraint rows of these dofs)
            tq = np.abs(d.qfrc_actuator[w.dadr]).copy()
            for i in range(d.nefc):
                if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF:
                    k = np.where(w.dadr == d.efc_id[i])[0]
                    if len(k): tq[k[0]] += abs(d.efc_force[i])
            gear = np.maximum(gear, tq)
            for i in range(d.ncon):
                c = d.contact[i]
                if w.ground not in (c.geom1, c.geom2): continue
                other = c.geom2 if c.geom1 == w.ground else c.geom1
                if other in feet: continue
                mujoco.mj_contactForce(m, d, i, f6); chassis = max(chassis, float(f6[0]))
        up = float(quat_up(d.qpos[3:7]))
    return dict(gear_max=float(gear.max()), gear_worst=sw.JOINTS[int(gear.argmax())], shock_g=shock, chassis_N=chassis,
                upright=up > .8, brownouts=w.brownouts)


def quat_up(q):
    w, x, y, z = q; return 1 - 2*(x*x + y*y)                        # body z . world z


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--height', type=float, default=.75)
    ap.add_argument('--trials', type=int, default=12, help='random variations of every fall (same ones for each reaction)')
    ap.add_argument('--out', type=Path)
    a = ap.parse_args()
    w = sw.SimWorld(); w.paused = True
    rows = []
    for sc in SCENARIOS:
        for re in fall_guard.REACTIONS:
            rs = [run(w, sc, re, a.height, rng=np.random.default_rng(1000 + k)) for k in range(a.trials)]
            r = dict(scenario=sc, reaction=re, trials=rs, **{f'{k}_mean': float(np.mean([x[k] for x in rs])) for k in ('gear_max', 'shock_g', 'chassis_N')},
                     gear_peak=max(x['gear_max'] for x in rs), upright=sum(x['upright'] for x in rs))
            rows.append(r)
            print(f"{sc:11s} {re:14s} gears gem. {r['gear_max_mean']:4.1f} max {r['gear_peak']:4.1f} Nm  shock {r['shock_g_mean']:5.1f} g  "
                  f"chassis {r['chassis_N_mean']:6.1f} N  rechtop {r['upright']}/{a.trials}", flush=True)
    if a.out: a.out.write_text(json.dumps(dict(height_m=a.height, rows=rows), indent=1) + '\n')


if __name__ == '__main__': main()
