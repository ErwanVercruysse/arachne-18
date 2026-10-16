"""Motion engine for ARACHNE: gait with velocity commands, body pose and short shows.

Pure kinematics on simulation/arachne_fixed.xml (body fixed, feet planned in the body frame),
so it can drive the physics simulation now and the real robot later: tick() returns model
joint angles (rad); callers turn them into servo ticks with calibration/twin_map.json.

    m = Motion()
    m.start('stand', q_now)          # stand / lie / dance / wave / pushups / look
    m.drive(vx, vy, wz)              # m/s, m/s, rad/s (body frame, +x = front = L1/L6 side)
    q = m.tick(dt, q_now)            # {joint: rad} or None (= do not command)
"""
import json, math
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TICK = 2*math.pi/4096
LEGS = ('L1', 'L2', 'L3', 'L4', 'L5', 'L6')
PARTS = ('yaw', 'hip', 'knee')
JOINTS = [f'{L}_{p}' for L in LEGS for p in PARTS]
STRAIGHT = {'yaw': 0., 'hip': math.radians(15), 'knee': math.radians(-87.56918699361886)}   # offset of the straight-out pose
STAND = {'yaw': 0, 'hip': -307, 'knee': -694}             # stand pose = straight-out middle + these ticks (tools/stand.py)

# phase offset of every leg in the cycle and the stance fraction (duty factor)
GAITS = {
    'tripod': ({'L1': 0, 'L3': 0, 'L5': 0, 'L2': .5, 'L4': .5, 'L6': .5}, .5),
    'ripple': ({'L3': 0, 'L2': 1/3, 'L1': 2/3, 'L4': .5, 'L5': 5/6, 'L6': 1/6}, 2/3),
    'wave':   ({'L3': 0, 'L2': 1/6, 'L1': 2/6, 'L4': 3/6, 'L5': 4/6, 'L6': 5/6}, 5/6),
}
SHOWS = {'dance': 12., 'wave': 7., 'pushups': 7.5, 'look': 9., 'bow': 5., 'twist': 8., 'notice': 2.4}


def load_model(path, meshes=False):
    """Compile an MJCF; without the visual meshes (collisions are primitives, masses explicit) it needs
    ~50 MB instead of ~900 MB, which matters on the Pi."""
    if meshes: return mujoco.MjModel.from_xml_path(str(path))
    spec = mujoco.MjSpec.from_file(str(path))
    for g in list(spec.geoms):
        if g.type == mujoco.mjtGeom.mjGEOM_MESH: spec.delete(g)
    for m in list(spec.meshes): spec.delete(m)
    return spec.compile()


def smooth(x): x = min(max(x, 0.), 1.); return x*x*(3-2*x)


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


class Kinematics:
    """FK/IK per leg on the fixed-body model, ticks <-> rad with the twin calibration."""

    def __init__(self):
        self.model = load_model(ROOT/'simulation/arachne_fixed.xml')
        self.data = mujoco.MjData(self.model)
        self.cal = json.loads((ROOT/'calibration/twin_map.json').read_text())
        self.lim = json.loads((ROOT/'calibration/joint_limits.json').read_text())['joints']
        self.adr = {j: self.model.joint(j).qposadr[0] for j in JOINTS}
        self.dof = {j: self.model.joint(j).dofadr[0] for j in JOINTS}
        self.site = {L: self.model.site(f'{L}_foot').id for L in LEGS}
        mujoco.mj_kinematics(self.model, self.data)
        self.base = self.data.body('BODY').xpos.copy()          # the fixed model floats the body; work relative to it
        # joint range in rad from the measured tick range (40 ticks margin, like the robot scripts)
        self.qlim = {}
        for j in JOINTS:
            a, b = self.q_of(j, self.lim[j]['min']+40), self.q_of(j, self.lim[j]['max']-40)
            self.qlim[j] = (min(a, b), max(a, b))

    def q_of(self, j, ticks): c = self.cal[j]; return c['sign']*(ticks-c['zero'])*TICK
    def ticks_of(self, j, q): c = self.cal[j]; return c['zero'] + q/(c['sign']*TICK)

    def pose(self, kind):
        """'lie' = legs straight out, 'stand' = the calibrated stand pose (both as {joint: rad})."""
        out = {}
        for j in JOINTS:
            part = j.split('_')[1]; c = self.cal[j]
            mid = c['zero'] - STRAIGHT[part]/(c['sign']*TICK)
            out[j] = self.q_of(j, mid + (STAND[part] if kind == 'stand' else 0))
        return out

    def feet(self, q):
        d = self.data; d.qpos[:] = 0
        for j, v in q.items(): d.qpos[self.adr[j]] = v
        mujoco.mj_kinematics(self.model, d)
        return {L: d.site_xpos[self.site[L]] - self.base for L in LEGS}

    def ik(self, leg, target, q, iters=40):
        """Damped least squares on one leg (target in the body frame); q is updated in place, returns error (m)."""
        m, d = self.model, self.data; target = target + self.base
        js = [f'{leg}_{p}' for p in PARTS]; dofs = [self.dof[j] for j in js]
        for j, v in q.items(): d.qpos[self.adr[j]] = v
        err = np.zeros(3); jp = np.zeros((3, m.nv))
        for _ in range(iters):
            mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
            err = target - d.site_xpos[self.site[leg]]
            if err @ err < 4e-8: break
            mujoco.mj_jacSite(m, d, jp, None, self.site[leg])
            J = jp[:, dofs]; dq = J.T @ np.linalg.solve(J @ J.T + 1e-4*np.eye(3), err)
            for j, x in zip(js, dq):
                lo, hi = self.qlim[j]; q[j] = min(hi, max(lo, q[j] + x)); d.qpos[self.adr[j]] = q[j]
        return float(np.linalg.norm(err))


class Motion:
    def __init__(self, kin=None):
        self.k = kin or Kinematics()
        self.q_lie, self.q_stand = self.k.pose('lie'), self.k.pose('stand')
        self.home = self.k.feet(self.q_stand)
        self.params = dict(gait='tripod', cycle=1.1, step_h=.025, height=0., speed=1.)
        self.body = dict(roll=0., pitch=0., yaw=0., x=0., y=0.)      # requested body pose (rad / m)
        self.body_now = dict(self.body, height=0.)                     # pose actually used: follows at a limited rate
        self.cmd = np.zeros(3); self.vel = np.zeros(3)                 # requested / smoothed (vx, vy, wz)
        self.mode = 'idle'; self.label = 'los'; self.q = None
        self.queue = []; self.t = 0.
        self._gait_reset()

    # ------------------------------------------------------------- commands
    def drive(self, vx, vy, wz):
        s = self.params['speed']
        self.cmd = np.array([max(-.08, min(.08, vx))*s, max(-.06, min(.06, vy))*s, max(-.6, min(.6, wz))*s])

    def start(self, what, q_now):
        """Start a behaviour from the current joint angles; shows stand up first when needed."""
        self.q = dict(q_now); self.cmd[:] = 0
        if what == 'stand': self._transition(self.q_stand, 1.6, then='gait', label='opstaan')
        elif what == 'lie': self._settle_then(lambda: self._transition(self.q_lie, 1.6, then='idle', label='gaan liggen'))
        elif what in SHOWS:
            if self.mode == 'show': self._end_show(); self.queue = [lambda: self._show(what)]      # fade out first
            elif self.mode == 'gait': self._settle_then(lambda: self._show(what))
            else: self._transition(self.q_stand, 1.6, then='gait', label='opstaan'); self.queue.append(lambda: self._show(what))
        elif what == 'stop':
            self.queue.clear()
            if self.mode == 'show': self._end_show()
        return self.label

    @property
    def standing(self): return self.mode in ('gait', 'show')

    # ------------------------------------------------------------- internals
    def _gait_reset(self):
        self.phase = 0.; self.off = {L: np.zeros(2) for L in LEGS}; self.zl = {L: 0. for L in LEGS}
        self.swing0 = {L: np.zeros(2) for L in LEGS}; self.prev_sw = {L: False for L in LEGS}

    def _transition(self, target, dur, then, label):
        self.mode = 'trans'; self.label = label
        self.tr = dict(a=dict(self.q), b=dict(target), t=0., dur=dur, then=then)

    def _settle_then(self, fn):
        """Let the gait bring all feet home before fn (so a pose change never starts mid-step)."""
        if self.mode == 'gait' and (np.linalg.norm(self.vel) > 1e-3 or any(np.linalg.norm(o) > .002 for o in self.off.values())):
            self.cmd[:] = 0; self.queue.append(fn)
        else: fn()

    def _show(self, what):
        self.mode = 'show'; self.label = {'dance': 'dansen', 'wave': 'zwaaien', 'pushups': 'push-ups', 'look': 'rondkijken', 'bow': 'buigen', 'twist': 'twist', 'notice': 'luisteren'}[what]
        self.sh = dict(what=what, t=0., dur=SHOWS[what], end=None)
        self.vel[:] = 0; self.cmd[:] = 0; self._gait_reset()

    def _end_show(self):
        if self.sh['end'] is None: self.sh['end'] = self.sh['t']

    def tick(self, dt, q_now=None):
        self.t += dt
        if self.q is None:
            if q_now is None: return None
            self.q = dict(q_now)
        if self.mode == 'idle':
            if self.queue: self.queue.pop(0)()
            return None if self.mode == 'idle' else self.tick(0.)
        if self.mode == 'trans':
            tr = self.tr; tr['t'] += dt; s = smooth(tr['t']/tr['dur'])
            self.q = {j: tr['a'][j] + (tr['b'][j]-tr['a'][j])*s for j in JOINTS}
            if tr['t'] >= tr['dur']:
                self.mode = tr['then']; self.label = 'staan' if self.mode == 'gait' else 'ligt'
                if self.mode == 'gait': self._gait_reset(); self.body_now = {k: 0. for k in self.body_now}   # pose eases in from here
            return dict(self.q)
        if self.mode == 'show': return self._tick_show(dt)
        return self._tick_gait(dt)

    def _feet_to_q(self, tg):
        for L in LEGS: self.k.ik(L, tg[L], self.q)
        return dict(self.q)

    def _body_targets(self, feet, shift, R):
        return {L: R.T @ (feet[L] - shift) for L in LEGS}

    RATE = dict(roll=.5, pitch=.5, yaw=.6, x=.04, y=.04, height=.04)      # per second: no jumps = no current spikes

    def _follow_pose(self, dt):
        want = dict(self.body, height=self.params['height'])
        for k, r in self.RATE.items():
            self.body_now[k] += max(-r*dt, min(r*dt, want[k] - self.body_now[k]))

    def _pose(self, extra_shift=(0, 0, 0), extra_rpy=(0, 0, 0)):
        b = self.body_now
        R = rot(b['roll']+extra_rpy[0], b['pitch']+extra_rpy[1], b['yaw']+extra_rpy[2])
        shift = np.array([b['x'], b['y'], b['height']]) + np.array(extra_shift)
        return shift, R

    def _tick_gait(self, dt):
        self._follow_pose(dt)
        p = self.params; phases, duty = GAITS[p['gait']]; T = p['cycle']
        # smooth the command (accel limit) so the body never jerks
        acc = np.array([.15, .15, 1.2])*dt
        self.vel += np.clip(self.cmd - self.vel, -acc, acc)
        v = self.vel
        moving = np.linalg.norm(v) > 1e-3 or any(np.linalg.norm(o) > .002 for o in self.off.values()) or any(z > 1e-4 for z in self.zl.values())
        if not moving:
            self.phase = 0.; self.prev_sw = {L: False for L in LEGS}
            if self.queue: self.queue.pop(0)(); return self.tick(0.)
        else: self.phase = (self.phase + dt/T) % 1.
        feet = {}
        for L in LEGS:
            h = self.home[L]
            vel_at = lambda pt: np.array([v[0] - v[2]*pt[1], v[1] + v[2]*pt[0]])     # body velocity at a point (xy)
            psi = (self.phase - phases[L]) % 1.
            swing = moving and psi >= duty
            if not swing:
                if moving: self.off[L] = self.off[L] - vel_at(h[:2] + self.off[L])*dt
                self.zl[L] = 0.
            else:
                if not self.prev_sw[L]: self.swing0[L] = self.off[L].copy()
                s = (psi - duty)/(1 - duty)
                land = vel_at(h[:2])*duty*T/2
                self.off[L] = self.swing0[L] + (land - self.swing0[L])*smooth(s)
                self.zl[L] = p['step_h']*math.sin(math.pi*s)
            n = np.linalg.norm(self.off[L])
            if n > .045: self.off[L] *= .045/n                             # reach limit
            self.prev_sw[L] = swing
            feet[L] = h + np.array([self.off[L][0], self.off[L][1], self.zl[L]])
        self.label = 'lopen' if moving else 'staan'
        shift, R = self._pose()
        return self._feet_to_q(self._body_targets(feet, shift, R))

    def _tick_show(self, dt):
        self._follow_pose(dt)
        sh = self.sh; sh['t'] += dt; t = sh['t']; L_ = sh['dur']
        if sh['end'] is not None: e = smooth(1 - (t - sh['end'])/1.)      # fade out after stop
        else: e = smooth(t/1.)*smooth((L_ - t)/1.)
        if (sh['end'] is not None and t - sh['end'] >= 1.) or t >= L_:
            self.mode = 'gait'; self.label = 'staan'; self._gait_reset()
            return self._tick_gait(0.)
        feet = {L: self.home[L].copy() for L in LEGS}
        w = sh['what']
        if w == 'dance':                                                   # body circles and tilts, feet stay put
            ph = 2*math.pi*.45*t
            shift = (.018*e*math.cos(ph), .018*e*math.sin(ph), .006*e*math.sin(2*ph))
            rpy = (math.radians(7)*e*math.sin(ph), -math.radians(7)*e*math.cos(ph), math.radians(6)*e*math.sin(.5*ph))
        elif w == 'pushups':
            k = (1 - math.cos(2*math.pi*t/1.4))/2
            shift = (0, 0, -.03*e*k); rpy = (0, math.radians(5)*e*k, 0)
        elif w == 'bow':                                                   # dip the front, hold, come back up
            k = smooth(t/1.4)*smooth((L_ - t)/1.4)
            shift = (.012*k, 0, -.012*k); rpy = (0, math.radians(12)*k, 0)
        elif w == 'twist':                                                 # quick yaw twist with a bounce on every beat
            ph = 2*math.pi*.9*t
            shift = (0, 0, -.008*e*(1 - math.cos(2*ph))/2); rpy = (math.radians(3)*e*math.sin(2*ph), 0, math.radians(14)*e*math.sin(ph))
        elif w == 'notice':                                                # "hey spin": perk up and look up at whoever called
            k = smooth(t/.5)*smooth((L_ - t)/.7)
            shift = (-.006*k, 0, .012*k); rpy = (0, -math.radians(7)*k, 0)
        elif w == 'look':
            shift = (0, 0, .008*e*math.sin(2*math.pi*t/3)); rpy = (math.radians(4)*e*math.sin(2*math.pi*t/2.2), math.radians(6)*e*math.sin(2*math.pi*t/4.5), math.radians(16)*e*math.sin(2*math.pi*t/4.5+1))
        else:                                                              # wave with the front-left leg
            others = np.mean([self.home[L][:2] for L in LEGS if L != 'L1'], axis=0)
            lean = smooth(t/1.) if sh['end'] is None and t < L_-1 else e
            shift = (others[0]*.9*lean, others[1]*.9*lean, .01*lean); rpy = (math.radians(-4)*lean, math.radians(-5)*lean, 0)
            up = smooth((t-1.)/.8) * (1 - smooth((t-(L_-1.8))/.8)) if sh['end'] is None else e*smooth((t-1.)/.8)
            h = self.home['L1']; out = h[:2]/np.linalg.norm(h[:2])
            sway = math.sin(2*math.pi*1.3*max(0, t-1.8))*up
            side = np.array([-out[1], out[0]])
            feet['L1'] = h + np.array([*(out*.02*up + side*.035*sway), .075*up])
        shift, R = self._pose(shift, rpy)
        return self._feet_to_q(self._body_targets(feet, shift, R))
