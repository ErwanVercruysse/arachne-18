"""Does tools/grab.py really pick something up? A box in front of the robot in the browser simulation's
physics (tools/sim_world.py: measured joint ranges, STS3215 servo model, torque limit register).

    python training/grab_test.py --width .13 --mass .06
"""
import argparse, json, math, sys
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import sim_world as sw                                                # noqa: E402
import grab                                                           # noqa: E402
from policy_walk import Joints, fk, JOINTS                            # noqa: E402


def world_with_box(width, depth, height, mass, friction):
    orig = sw.load_model
    def load(path, meshes=False):
        if meshes or 'fixed' in str(path): return orig(path, meshes)
        spec = mujoco.MjSpec.from_file(str(path))
        for g in list(spec.geoms):
            if g.type == mujoco.mjtGeom.mjGEOM_MESH: spec.delete(g)
        for m in list(spec.meshes): spec.delete(m)
        b = spec.worldbody.add_body(name='box', pos=[5, 0, height/2])   # parked away during reset, placed later
        b.add_freejoint(name='box_free')
        b.mass = mass; b.ipos = [0, 0, 0]; b.explicitinertial = True   # the robot model does not derive inertia from geoms
        b.inertia = [mass*(width**2 + height**2)/12, mass*(depth**2 + height**2)/12, mass*(depth**2 + width**2)/12]
        b.add_geom(name='box_geom', type=mujoco.mjtGeom.mjGEOM_BOX, size=[depth/2, width/2, height/2], mass=mass,
                   friction=[friction, .01, .001], rgba=[.9, .6, .1, 1])
        return spec.compile()
    sw.load_model = load
    try: return sw.SimWorld()
    finally: sw.load_model = orig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--width', type=float, default=.13); ap.add_argument('--depth', type=float, default=.08)
    ap.add_argument('--height', type=float, default=.06); ap.add_argument('--mass', type=float, default=.06)
    ap.add_argument('--friction', type=float, default=.8); ap.add_argument('--at', type=float, default=.20)
    ap.add_argument('--out', type=Path)
    a = ap.parse_args()
    w = world_with_box(a.width, a.depth, a.height, a.mass, a.friction); m, d = w.model, w.data
    adr = m.jnt_qposadr[m.joint('box_free').id]
    J = Joints()
    with w.lock:
        w.paused = True; w.supply_key = 'lipo3s'; w.reset('stand')
        for _ in range(250): w._step(m.opt.timestep)                    # settle 1 s
        # the box goes where grab.py expects it: --at in front of the body centre, on the floor
        R = sw.quat_mat(d.qpos[3:7]); p = d.qpos[:3] + R @ [a.at, 0, 0]
        d.qpos[adr:adr+3] = [p[0], p[1], a.height/2 + .0005]; d.qpos[adr+3:adr+7] = d.qpos[3:7]; mujoco.mj_forward(m, d)
        q0 = J.q_of({j: int(t) for j, t in zip(JOINTS, w.present_ticks())})
        f0 = fk(q0); home = {L: f0[i] for i, L in enumerate(grab.LEGS)}
        traj = [(grab.solve(J, feet, shift)[0], label) for feet, shift, label in grab.frames(grab.plan(home, a.at, a.width))]
        yaw = [JOINTS.index(f'{L}_yaw') for L in grab.ARMS]
        steps = int(round(1/grab.HZ/(m.opt.timestep*w.SUB)))
        log = []; z_box0 = float(d.qpos[adr+2]); tilt_max = 0.; low_body = 1.
        def play(seq, phase):
            nonlocal tilt_max, low_body
            for q, label in seq:
                if label == 'knijpen': w.limit_reg[yaw] = grab.SQUEEZE_LIMIT
                w.sync(J.ticks_of(q))
                for _ in range(steps): w._step(m.opt.timestep)
                up = 1 - 2*(d.qpos[4]**2 + d.qpos[5]**2); tilt_max = max(tilt_max, math.degrees(math.acos(min(1, up))))
                low_body = min(low_body, float(d.qpos[2]))
            log.append(dict(phase=phase, box_lift_cm=round((float(d.qpos[adr+2]) - z_box0)*100, 2), body_z=round(float(d.qpos[2]), 3)))
        play(traj, 'opgepakt')
        for _ in range(int(2/(m.opt.timestep*w.SUB))): w._step(m.opt.timestep)    # hold 2 s
        log.append(dict(phase='2 s vasthouden', box_lift_cm=round((float(d.qpos[adr+2]) - z_box0)*100, 2), body_z=round(float(d.qpos[2]), 3)))
        play(traj[::-1], 'neergezet'); w.limit_reg[yaw] = 1000
    res = dict(box=vars(a), log=log, tilt_max_deg=round(tilt_max, 2), body_min_z=round(low_body, 3),
               held=log[1]['box_lift_cm'] > 2.5, upright=tilt_max < 20)
    print(json.dumps(res, indent=1))
    if a.out: a.out.write_text(json.dumps(res, indent=1) + '\n')


if __name__ == '__main__': main()
