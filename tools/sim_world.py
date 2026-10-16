"""Physics simulation of ARACHNE for the /sim page of live_twin.py.

MuJoCo (simulation/arachne.xml, free-floating body) with 18 virtual STS3215 servos that speak the
same registers as the real ones (goal 42, speed 46, torque 40, torque limit 48), so the robot
scripts (stand.py, walk.py, spin.py, dance.py) run unchanged against /sim/state + /sim/cmd.

Servo model = the BAM 'm6' STS3215 fit (simulation/bam_sts3215_12v_approx_m6.json) without the
BAM package: speed-limited internal target, P-controller to a PWM duty, DC motor with back-EMF.
The 12 V supply has an internal resistance and a current limit, so the voltage sags like the lab
supply does; below the brown-out voltage every servo resets (torque off), as on the real robot.
With torque off the gear friction still holds the legs (the real robot keeps standing).
"""
import json, math, os, subprocess, sys, threading, time
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from motion import Motion, Kinematics, JOINTS, LEGS, TICK, load_model

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT/'simulation'
SID = {j['name']: j['servo_id'] for j in json.loads((SIM/'joint_map.json').read_text())['joints']}
P = json.loads((SIM/'bam_sts3215_12v_approx_m6.json').read_text())
KP, ERROR_GAIN, MAX_PWM = 32, .166, .97                                   # STS3215 firmware (BAM)
HOLD_FRICTION = 1.2           # Nm, gearbox friction with torque off (the robot keeps standing)
IDLE_CURRENT = .3             # A, Pi 4 + adapters through the buck converter
BROWNOUT_V = 6.5
SUPPLIES = {                  # open-circuit V, internal resistance, current limit
    'lab5': dict(name='labvoeding 12 V / 5 A', v=12.0, r=.25, ilim=5.),
    'lab10': dict(name='labvoeding 12 V / 10 A', v=12.0, r=.15, ilim=10.),
    'lipo3s': dict(name='3S LiPo (vol)', v=12.6, r=.05, ilim=60.),
}
SCRIPTS = {'stand': ['tools/stand.py'], 'lie': ['tools/stand.py', '--lie'], 'walk': ['tools/walk.py', '--loop'],
           'back': ['tools/walk.py', '--back', '--loop'], 'spin': ['tools/spin.py', '--loop'], 'dance': ['tools/dance.py', '--loop']}

# body parts that the real robot does not have yet: they can be hidden and then drop out of the mass
OPTIONAL = {
    'deck': ('02_DORSAL_DECK', 'BODY_deck'), 'tray': ('03_BATTERY_TRAY', 'BODY_tray'),
    'ctrlplate': ('04_CONTROLLER_PLATE', 'BODY_controller', 'PCB_spacer'), 'shell': ('05_SHELL', 'BODY_shell'),
    'battery': ('REF_3S_BATTERY',), 'controller': ('REF_CONTROLLER',),
}
FIXED_MASS = {'REF_3S_BATTERY': .19, 'REF_CONTROLLER': .045}
DENSITY = {'RXM': 8500., 'M3': 7800.}                                     # inserts (brass), screws (steel); printed = 700


def category(name):
    """Mesh name -> part group shown in the 'onderdelen' list."""
    n = name.upper()
    for cat, keys in OPTIONAL.items():                    # incl. the screws that hold these parts
        if any(n.startswith(k.upper()) for k in keys): return cat
    if 'STS3215' in n: return 'servos'
    if 'M3' in n or 'LOCKNUT' in n: return 'screws'
    if n.startswith('01_CHASSIS'): return 'chassis'
    if 'CLAMP' in n or 'DISC' in n: return 'clamps'
    if 'TIBIA' in n or 'FOOT' in n: return 'tibia'
    if 'FEMUR' in n: return 'femur'
    if 'COXA' in n: return 'coxa'
    return 'other'


def mesh_volume(m, mid):
    v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]].astype(float)
    f = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid]+m.mesh_facenum[mid]]
    return abs(np.einsum('ij,ij->i', v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum()/6)


def quat_mat(q):
    m = np.zeros(9); mujoco.mju_quat2Mat(m, q); return m.reshape(3, 3)


class SimWorld:
    def __init__(self, mesh_model=None):
        """mesh_model: a compiled ARACHNE model WITH meshes (e.g. live_twin's), used for the part masses
        and the browser scene; the physics model itself is compiled without visual meshes."""
        self.model = m = load_model(SIM/'arachne.xml')
        self.mesh_model = mesh_model if mesh_model is not None else load_model(SIM/'arachne_fixed.xml', meshes=True)
        self.data = mujoco.MjData(m)
        self.kin = Kinematics()
        self.motion = Motion(self.kin)
        self.lock = threading.RLock()
        self.qadr = np.array([m.joint(j).qposadr[0] for j in JOINTS]); self.dadr = np.array([m.joint(j).dofadr[0] for j in JOINTS])
        self.act = np.array([m.actuator(j).id for j in JOINTS])
        self.sign = np.array([self.kin.cal[j]['sign'] for j in JOINTS], float)
        self.zero = np.array([self.kin.cal[j]['zero'] for j in JOINTS], float)
        # joint ranges = the measured range of the real robot (the CAD ranges are tighter than reality)
        for j in JOINTS:
            jid = m.joint(j).id; lo, hi = self.kin.qlim[j]; m.jnt_range[jid] = (lo - .08, hi + .08)
        m.dof_armature[self.dadr] = P['armature']
        m.dof_damping[self.dadr] = P['friction_viscous']
        self.ground = m.geom('ground').id
        self.tibia = {L: m.body(f'{L}_TIBIA').id for L in LEGS}
        self.base_body = m.body('BODY').id
        self._mass_setup()
        self._setup_actuators()
        self.supply_key = 'lab5'; self.v = SUPPLIES['lab5']['v']; self.i_supply = 0.
        self.speed = 1.; self.paused = False; self.slope = 0.; self.friction = .8
        self.proc = None; self.proc_what = None; self.stop_file = f'/tmp/arachne_sim_stop_{os.getpid()}'
        self.events = []; self.viewers = 0; self.last_seen = time.time()
        self.drive_t = 0.
        self.reset()
        self.snapshot_json = b'{}'
        self.thread = threading.Thread(target=self._run, daemon=True); self.thread.start()

    # ------------------------------------------------------------- setup
    def _mass_setup(self):
        b = self.base_body; mm = self.model
        self.base_mass0 = float(mm.body_mass[b]); self.base_com0 = mm.body_ipos[b].copy(); self.base_inertia0 = mm.body_inertia[b].copy()
        m = self.mesh_model
        self.part_mass = {}                                       # optional group -> (kg, com in body frame)
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH: continue
            name = m.mesh(m.geom_dataid[g]).name; cat = category(name)
            if cat not in OPTIONAL: continue
            mid = m.geom_dataid[g]
            kg = next((v for k, v in FIXED_MASS.items() if name.startswith(k)), None)
            if kg is None: kg = mesh_volume(m, mid)*next((v for k, v in DENSITY.items() if k in name.upper()), 700.)
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
            com = quat_mat(m.geom_quat[g]) @ v.mean(axis=0) + m.geom_pos[g]
            km, kc = self.part_mass.get(cat, (0., np.zeros(3)))
            self.part_mass[cat] = (km + kg, (kc*km + com*kg)/(km + kg))
        self.hidden_mass = set()

    def set_hidden_mass(self, cats):
        """Remove the mass of hidden optional parts from the body (COM shifts, inertia scales)."""
        with self.lock:
            self.hidden_mass = {c for c in cats if c in self.part_mass}
            m = self.model; b = self.base_body
            mass = self.base_mass0; mom = self.base_com0*self.base_mass0
            for c in self.hidden_mass:
                kg, com = self.part_mass[c]; mass -= kg; mom -= com*kg
            m.body_mass[b] = mass; m.body_ipos[b] = mom/mass; m.body_inertia[b] = self.base_inertia0*mass/self.base_mass0
            return self.total_mass()

    def total_mass(self): return float(self.model.body_mass[1:].sum())

    def reset(self, pose='lie'):
        with self.lock:
            self._stop_script()
            m, d = self.model, self.data
            mujoco.mj_resetData(m, d)
            q0 = self.kin.pose(pose)
            for k, j in enumerate(JOINTS): d.qpos[self.qadr[k]] = q0[j]
            d.qpos[2] = .2; d.qpos[3:7] = (1, 0, 0, 0)
            self.ticks_goal = self._ticks(np.array([q0[j] for j in JOINTS]))
            self.goal_q = np.array([q0[j] for j in JOINTS]); self.q_smooth = self.goal_q.copy()
            self.speed_reg = np.zeros(18); self.limit_reg = np.full(18, 1000.); self.torque = np.zeros(18, bool)
            if pose == 'stand': self.torque[:] = True
            self.v = SUPPLIES[self.supply_key]['v']; self.i_supply = 0.
            self.motion = Motion(self.kin)
            if pose == 'stand': self.motion.mode = 'gait'; self.motion.label = 'staan'
            self.motion.q = dict(q0)
            self.tau = np.zeros(18); self.i_motor = np.zeros(18)
            self.trail = []; self.dist = 0.; self.prev_xy = None; self.t_wall0 = time.time()
            self.hist = []; self.peak_i = 0.; self.brownouts = 0
            mujoco.mj_forward(m, d)
            # drop it on the floor: lowest point of any collision geom just above the ground
            low = min(d.geom_xpos[g][2] - m.geom_rbound[g]*.5 for g in range(m.ngeom) if m.geom_contype[g] and g != self.ground)
            d.qpos[2] -= low - .002
            mujoco.mj_forward(m, d)
            for _ in range(200): self._step(m.opt.timestep)       # settle (0.8 s)
            self.event('simulatie gestart' if pose == 'lie' else 'reset: staand')

    def event(self, msg, level='info'):
        self.events.append(dict(t=time.time(), msg=msg, level=level)); self.events = self.events[-12:]

    # ------------------------------------------------------------- servo registers
    def _ticks(self, q): return self.zero + q/(self.sign*TICK)
    def _q(self, ticks): return self.sign*(np.asarray(ticks, float) - self.zero)*TICK

    def _k(self, sid):
        for k, j in enumerate(JOINTS):
            if SID[j] == sid: return k
        return None

    def write(self, sid, addr, data):
        k = self._k(sid)
        if k is None: return
        r = dict(enumerate(data, addr))
        if 40 in r:
            on = bool(r[40])
            if on and not self.torque[k]: self.q_smooth[k] = self.data.qpos[self.qadr[k]]
            self.torque[k] = on
        if 42 in r and 43 in r:
            t = r[42] | r[43] << 8; t = -(t & 0x7fff) if t & 0x8000 else t
            self.ticks_goal[k] = t; self.goal_q[k] = self.sign[k]*(t - self.zero[k])*TICK
        if 46 in r and 47 in r: self.speed_reg[k] = r[46] | r[47] << 8
        if 48 in r and 49 in r: self.limit_reg[k] = min(1000, r[48] | r[49] << 8)

    def read(self, sid, addr, n):
        k = self._k(sid)
        if k is None: return None
        t = int(self.present_ticks()[k])
        load = int(min(1000, abs(self.tau[k])/self.tau_stall*1000)) | (1024 if self.tau[k] < 0 else 0)
        g = int(self.ticks_goal[k])
        regs = {40: int(self.torque[k]), 42: g & 0xff, 43: g >> 8 & 0xff, 56: t & 0xff, 57: t >> 8 & 0xff,
                60: load & 0xff, 61: load >> 8, 62: int(self.v*10), 63: 32}
        return [regs.get(a, 0) for a in range(addr, addr+n)]

    def cmd(self, d):
        """Same JSON as live_twin's POST /cmd: ping / read / write / sync."""
        with self.lock:
            if self.proc is None and d['op'] in ('write', 'sync'): self.motion.mode = 'idle'; self.motion.queue.clear()
            if d['op'] == 'ping': return {'ok': self._k(d['id']) is not None}
            if d['op'] == 'read': r = self.read(d['id'], d['addr'], d['n']); return {'data': r}
            if d['op'] == 'write': self.write(d['id'], d['addr'], d['data']); return {'ok': True}
            if d['op'] == 'sync':
                for sid, v in d['data'].items(): self.write(int(sid), d['addr'], v)
                return {'ok': True}
        return {'error': 'onbekende op'}

    def present_ticks(self): return np.round(self._ticks(self.data.qpos[self.qadr])).astype(int)

    @property
    def tau_stall(self): return P['kt']*12.*MAX_PWM/P['R']

    # ------------------------------------------------------------- physics
    SUB = 4                   # physics steps per servo-control update (1 ms steps, control at 250 Hz)

    def _setup_actuators(self):
        """The motors become MuJoCo position servos (P-gain + force limit, back-EMF as joint damping), so
        the inner loop runs in C; _step() updates gains/limits from the registers and the supply voltage."""
        m = self.model; a = self.act
        m.actuator_gaintype[a] = mujoco.mjtGain.mjGAIN_FIXED; m.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_AFFINE
        m.actuator_forcelimited[a] = 1; m.actuator_ctrllimited[a] = 0
        self.duty_per_rad = KP*ERROR_GAIN*P['error_gain_ratio']
        self.emf = P['kt']**2/P['R']

    def _step(self, dt):
        """One servo-control update followed by SUB physics steps of dt."""
        m, d = self.model, self.data; a = self.act; dtc = dt*self.SUB
        q = d.qpos[self.qadr]; dq = d.qvel[self.dadr]
        vmax = np.where(self.speed_reg > 0, np.minimum(self.speed_reg*TICK, P['max_velocity']), P['max_velocity'])
        self.q_smooth = np.clip(self.goal_q, self.q_smooth - vmax*dtc, self.q_smooth + vmax*dtc)
        on = self.torque
        dmax = MAX_PWM*self.limit_reg/1000
        G = np.where(on, self.duty_per_rad*self.v*P['kt']/P['R'], 0.)
        m.actuator_gainprm[a, 0] = G; m.actuator_biasprm[a, 1] = -G
        lim = np.where(on, dmax*self.v*P['kt']/P['R'], 0.)
        m.actuator_forcerange[a, 0] = -lim; m.actuator_forcerange[a, 1] = lim
        m.dof_damping[self.dadr] = P['friction_viscous'] + np.where(on, self.emf, 0.)
        m.dof_frictionloss[self.dadr] = np.where(on, P['friction_base'], HOLD_FRICTION)
        d.ctrl[a] = self.q_smooth
        # electrical side (same model, evaluated at the control rate): PWM duty, motor current, supply
        duty = np.where(on, np.clip((self.q_smooth - q)*self.duty_per_rad, -dmax, dmax), 0.)
        i = np.where(on, (self.v*duty - P['kt']*dq)/P['R'], 0.)
        self.tau = P['kt']*i; self.i_motor = i
        s = SUPPLIES[self.supply_key]
        i_dem = max(0., float((duty*i).sum())) + IDLE_CURRENT
        v_t = s['v'] - s['r']*i_dem
        if i_dem > s['ilim']: v_t = min(v_t, self.v*s['ilim']/i_dem)       # constant-current mode
        self.v += (v_t - self.v)*min(1., dtc/.004)
        self.i_supply = min(i_dem, s['ilim']*1.05)
        if self.v < BROWNOUT_V and on.any():
            self.torque[:] = False; self.brownouts += 1
            self.motion.mode = 'idle'; self.motion.queue.clear()
            self.event(f'brown-out: voeding zakte naar {self.v:.1f} V, alle servo\'s resetten (torque uit)', 'bad')
        mujoco.mj_step(m, d, nstep=self.SUB)

    def _run(self):
        m, d = self.model, self.data; dt = m.opt.timestep
        ctrl_every = 20//self.SUB                                   # motion engine at 50 Hz (sim time)
        last = time.time(); acc = 0.; k = 0; snap_t = 0.; rtf_win = []
        while True:
            time.sleep(.004)
            now = time.time(); wall = now - last; last = now
            if self.paused or now - self.last_seen > 90: rtf_win.clear(); continue      # nobody watching: sleep
            acc = min(acc + wall*self.speed, .05)
            n = 0; t0 = time.perf_counter()
            with self.lock:
                if self.drive_t and now - self.drive_t > .6: self.motion.drive(0, 0, 0); self.drive_t = 0.   # dead-man
                while acc >= dt*self.SUB and time.perf_counter() - t0 < .02:
                    if k % ctrl_every == 0 and self.proc is None: self._motion_tick(dt*self.SUB*ctrl_every)
                    self._step(dt); acc -= dt*self.SUB; n += self.SUB; k += 1
                self._track()
            rtf_win.append((wall, n*dt)); rtf_win[:] = rtf_win[-100:]
            self.rtf = sum(x[1] for x in rtf_win)/max(1e-6, sum(x[0] for x in rtf_win))
            if now - snap_t > 1/30:
                snap_t = now
                with self.lock: self.snapshot_json = json.dumps(self._snapshot()).encode()
            if self.proc is not None and self.proc.poll() is not None:
                self.event(f'script {self.proc_what} klaar'); self.proc = None; self.proc_what = None

    def _motion_tick(self, dt):
        mo = self.motion
        if mo.mode == 'idle' and not mo.queue: return
        q_now = {j: float(self.data.qpos[self.qadr[k]]) for k, j in enumerate(JOINTS)}
        if mo.q is None: mo.q = q_now
        tg = mo.tick(dt, q_now)
        if tg is None: return
        if not self.torque.all():
            # torque on like on the real robot: hold where it is, one servo at a time (here: all off ones at once, 30 %)
            off = ~self.torque
            self.q_smooth[off] = self.data.qpos[self.qadr][off]; self.limit_reg[off] = 300; self.torque[:] = True
            self.ramp = time.time()
        if getattr(self, 'ramp', None):
            r = time.time() - self.ramp
            self.limit_reg[:] = np.maximum(self.limit_reg, min(1000, 300 + 700*r/.6))
            if r > .6: self.ramp = None
        g = np.array([tg[j] for j in JOINTS])
        self.goal_q = g; self.ticks_goal = np.round(self._ticks(g)); self.speed_reg[:] = 0

    def _track(self):
        d = self.data; xy = d.qpos[:2].copy()
        if self.prev_xy is not None: self.dist += float(np.linalg.norm(xy - self.prev_xy))
        self.prev_xy = xy
        if not self.trail or np.linalg.norm(self.trail[-1][:2] - xy) > .004:
            self.trail.append(np.array([xy[0], xy[1], d.time])); self.trail = self.trail[-400:]

    # ------------------------------------------------------------- controls
    def do(self, what):
        with self.lock:
            if what.startswith('script:'): return self._script(what[7:])
            self._stop_script()
            q_now = {j: float(self.data.qpos[self.qadr[k]]) for k, j in enumerate(JOINTS)}
            if self.motion.mode == 'idle': self.motion.q = None
            label = self.motion.start(what, q_now)
            return {'ok': True, 'label': label}

    def drive(self, vx, vy, wz):
        with self.lock:
            if self.proc is not None: return
            mo = self.motion
            if not mo.standing and any(abs(x) > 1e-3 for x in (vx, vy, wz)):
                if mo.mode != 'trans': self.do('stand')
            mo.drive(vx, vy, wz); self.drive_t = time.time()

    def set(self, d):
        with self.lock:
            mo = self.motion
            for k in ('gait', 'cycle', 'step_h', 'height', 'speed'):
                if k in d: mo.params[k] = d[k] if k == 'gait' else float(d[k])
            if 'gait' in d and d['gait'] not in ('tripod', 'ripple', 'wave'): mo.params['gait'] = 'tripod'
            for k in ('roll', 'pitch', 'yaw', 'x', 'y'):
                if k in d: mo.body[k] = float(d[k])
            if 'supply' in d and d['supply'] in SUPPLIES: self.supply_key = d['supply']; self.event(f'voeding: {SUPPLIES[self.supply_key]["name"]}')
            if 'speed_sim' in d: self.speed = max(.05, min(2., float(d['speed_sim'])))
            if 'paused' in d: self.paused = bool(d['paused'])
            if 'friction' in d:
                self.friction = max(.1, min(1.5, float(d['friction']))); self.model.geom_friction[:, 0] = self.friction
            if 'slope' in d:
                self.slope = max(-25., min(25., float(d['slope']))); a = math.radians(self.slope)
                self.model.opt.gravity[:] = (-9.81*math.sin(a), 0, -9.81*math.cos(a))
            if 'hidden_mass' in d: self.set_hidden_mass(d['hidden_mass'])
        return {'ok': True}

    def push(self, fx=None, fy=None):
        """Short shove on the body (a 'kick' from the side), random direction when not given."""
        with self.lock:
            a = np.random.uniform(0, 2*math.pi) if fx is None else math.atan2(fy, fx)
            v = self.data.qvel
            v[0] += .8*math.cos(a); v[1] += .8*math.sin(a); v[2] += .25          # shove + a bit of lift
            v[3] += -4*math.sin(a); v[4] += 4*math.cos(a); v[5] += np.random.uniform(-2, 2)   # tips it over the push axis
            self.event('duw!')

    def _script(self, name):
        if name not in SCRIPTS: return {'error': 'onbekend script'}
        self._stop_script()
        self.motion.mode = 'idle'; self.motion.queue.clear(); self.motion.cmd[:] = 0
        if os.path.exists(self.stop_file): os.remove(self.stop_file)
        cmd = [sys.executable] + SCRIPTS[name] + ([self.stop_file] if SCRIPTS[name][-1] == '--loop' else [])
        env = dict(os.environ, ARACHNE_URL=f'http://127.0.0.1:{os.environ.get("ARACHNE_PORT", 5870)}/sim')
        self.proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.proc_what = name; self.event(f'script gestart: {" ".join(SCRIPTS[name]).replace("--loop", "").strip()}')
        return {'ok': True}

    def _stop_script(self):
        p = self.proc
        if p is None: return
        Path(self.stop_file).touch()
        def finish():
            try: p.wait(5)
            except subprocess.TimeoutExpired: p.terminate()
        threading.Thread(target=finish, daemon=True).start()
        self.proc = None; self.proc_what = None; self.event('script gestopt')

    # ------------------------------------------------------------- telemetry
    def _contacts(self):
        m, d = self.model, self.data
        feet = {L: [0., np.zeros(3)] for L in LEGS}; other = 0.; f6 = np.zeros(6)
        body_ids = {v: L for L, v in self.tibia.items()}
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 != self.ground and c.geom2 != self.ground: continue
            g = c.geom2 if c.geom1 == self.ground else c.geom1
            mujoco.mj_contactForce(m, d, i, f6); fn = abs(f6[0])
            L = body_ids.get(m.geom_bodyid[g])
            if L: feet[L][0] += fn; feet[L][1] += c.pos*fn
            else: other += fn
        out = {}
        for L, (fn, p) in feet.items():
            out[L] = dict(f=round(fn, 2), p=[round(x, 4) for x in (p/fn if fn > 1e-6 else d.site(f'{L}_foot').xpos)])
        return out, other

    @staticmethod
    def _hull(pts):
        pts = sorted(set(pts))
        if len(pts) < 3: return pts
        cross = lambda o, a, b: (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
        lo, hi = [], []
        for p in pts:
            while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
            lo.append(p)
        for p in reversed(pts):
            while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0: hi.pop()
            hi.append(p)
        return lo[:-1] + hi[:-1]

    @staticmethod
    def _margin(pt, hull):
        if len(hull) < 3: return -1.
        m = 1e9
        for i in range(len(hull)):
            a, b = np.array(hull[i]), np.array(hull[(i+1) % len(hull)])
            e = b - a; n = np.array([-e[1], e[0]])/max(1e-9, np.linalg.norm(e))
            m = min(m, float(np.dot(np.array(pt) - a, n)))
        return m

    def _snapshot(self):
        m, d = self.model, self.data
        feet, other = self._contacts()
        on = [(round(f['p'][0], 4), round(f['p'][1], 4)) for f in feet.values() if f['f'] > .3]
        hull = self._hull(on); com = d.subtree_com[1]
        margin = self._margin(com[:2], hull)
        q = d.qpos[self.qadr]; ticks = self.present_ticks()
        R = quat_mat(d.qpos[3:7]); yaw = math.atan2(R[1, 0], R[0, 0])
        roll = math.atan2(R[2, 1], R[2, 2]); pitch = -math.asin(max(-1, min(1, R[2, 0])))
        vb = R.T @ d.qvel[:3]
        joints = {}
        for k, j in enumerate(JOINTS):
            joints[j] = dict(sid=SID[j], port='sim', ticks=int(ticks[k]), goal=int(self.ticks_goal[k]),
                             deg=round(math.degrees(q[k]), 1), load=int(min(1000, abs(self.tau[k])/self.tau_stall*1000)),
                             tau=round(float(self.tau[k]), 3), amp=round(float(abs(self.i_motor[k])), 2),
                             torque=int(self.torque[k]), volt=round(self.v, 2), temp=32)
        self.peak_i = max(self.peak_i*.999, self.i_supply)
        mo = self.motion
        return dict(
            sim=True, t=round(d.time, 3), rtf=round(getattr(self, 'rtf', 0.), 2), paused=self.paused, speed=self.speed,
            bodies=[[round(float(x), 5) for x in (*d.xpos[b], *d.xquat[b])] for b in range(m.nbody)],
            joints=joints, feet=feet, other_contact=round(other, 2), support=hull, com=[round(float(x), 4) for x in com],
            margin=round(margin, 4),
            base=dict(x=round(float(d.qpos[0]), 4), y=round(float(d.qpos[1]), 4), z=round(float(d.qpos[2]), 4),
                      yaw=round(math.degrees(yaw), 1), roll=round(math.degrees(roll), 1), pitch=round(math.degrees(pitch), 1),
                      vx=round(float(vb[0]), 3), vy=round(float(vb[1]), 3), wz=round(math.degrees(float(d.qvel[5])), 1)),
            dist=round(self.dist, 3), trail=[[round(float(p[0]), 3), round(float(p[1]), 3)] for p in self.trail[::2]],
            supply=dict(key=self.supply_key, name=SUPPLIES[self.supply_key]['name'], v=round(self.v, 2), i=round(self.i_supply, 2),
                        ilim=SUPPLIES[self.supply_key]['ilim'], peak=round(self.peak_i, 2), brownouts=self.brownouts),
            motion=dict(mode=mo.mode, label=mo.label if self.proc is None else f'script: {self.proc_what}', standing=mo.standing,
                        params=mo.params, body=mo.body, cmd=[round(float(x), 3) for x in mo.vel]),
            script=self.proc_what, mass=round(self.total_mass(), 3), hidden_mass=sorted(self.hidden_mass),
            slope=self.slope, friction=self.friction, events=self.events[-6:], viewers=self.viewers, lift=0.,
            ground=0., hz=round(1/m.opt.timestep*getattr(self, 'rtf', 0.)))

    def state(self):
        """Script-compatible /state (same keys the robot scripts read from live_twin)."""
        return json.loads(self.snapshot_json or b'{}')


# ------------------------------------------------------------- scene for the browser
def build_scene(model):
    """Every mesh geom (also the parts not built yet) merged per body and colour, tagged with its group."""
    per = {}
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH: continue
        b = model.geom_bodyid[g]
        if b == 0: continue
        mid = model.geom_dataid[g]; name = model.mesh(mid).name; cat = category(name)
        va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        v = model.mesh_vert[va:va+vn] @ quat_mat(model.geom_quat[g]).T + model.geom_pos[g]
        f = model.mesh_face[fa:fa+fn]
        rgba = model.geom_rgba[g] if model.geom_matid[g] < 0 else model.mat_rgba[model.geom_matid[g]]
        per.setdefault((b, cat, tuple(np.round(rgba, 3))), []).append((v.astype(np.float32), f.astype(np.uint32)))
    meta, blobs, off = [], [], 0
    for (b, cat, c), items in per.items():
        verts, faces, n = [], [], 0
        for v, f in items: verts.append(v); faces.append(f+n); n += len(v)
        V = np.concatenate(verts).tobytes(); F = np.concatenate(faces).tobytes()
        meta.append(dict(body=int(b), name=model.body(b).name, cat=cat, rgba=[float(x) for x in c],
                         v_off=off, v_len=len(V), f_off=off+len(V), f_len=len(F)))
        blobs += [V, F]; off += len(V)+len(F)
    # collision primitives (shown as wireframes on request)
    prims = []
    for g in range(model.ngeom):
        t = model.geom_type[g]
        if not model.geom_contype[g] or model.geom_bodyid[g] == 0 or t == mujoco.mjtGeom.mjGEOM_MESH: continue
        prims.append(dict(body=int(model.geom_bodyid[g]), type=mujoco.mjtGeom(t).name[7:].lower(),
                          size=[float(x) for x in model.geom_size[g]], pos=[float(x) for x in model.geom_pos[g]],
                          quat=[float(x) for x in model.geom_quat[g]]))
    return meta, prims, b''.join(blobs)
