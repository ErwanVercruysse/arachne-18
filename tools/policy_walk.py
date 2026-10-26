"""Walk with the trained policy LIVE on the real robot: closed loop on the IMU, adapting to the ground.

Unlike the gamepad's planned cycles (plan_gaits.py, played back frame by frame), this runs the policy itself
(tools/terrain_gait.py on training/gait.py) every control period with the IMU's gravity and turn rate, so the
robot levels itself on slopes, lifts its feet higher when the ground gets rough and stops when it tips over.

Needs live_twin.py (it owns the servo bus and the IMU) and calibration/imu_map.json (tools/imu_align.py).
The robot must be standing (stand.py). It first steps into the policy's stance (lower, feet further out),
walks, and on stop steps back into the normal stand pose, so the other scripts keep working.

    .venv/bin/python tools/policy_walk.py --dry                    # plan + check limits and speeds, no motion
    .venv/bin/python tools/policy_walk.py --drive .08 0 0 --seconds 6
    .venv/bin/python tools/policy_walk.py --loop STOPFILE          # commands over UDP 127.0.0.1:5872 until STOPFILE
    ARACHNE_URL=http://localhost:5870/sim .venv/bin/python tools/policy_walk.py ...   # the browser simulation

UDP command (gamepad_robot.py sends these): {"vx": m/s, "vy": m/s, "wz": rad/s}; no command for 0.5 s = stand still.
"""
import argparse, json, math, os, socket, sys, threading, time, urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from terrain_gait import TerrainGait, gait                            # noqa: E402
import stand                                                          # noqa: E402  (stand pose definition)

URL = os.environ.get('ARACHNE_URL', 'http://localhost:5870')
ROOT = Path(__file__).resolve().parents[1]
UDP_PORT = 5872
STATUS = Path('/tmp/arachne_policy.json')
JOINTS = [f'L{l}_{p}' for l in range(1, 7) for p in ('yaw', 'hip', 'knee')]
TICK = 2*math.pi/4096
LIMIT = dict(vx=.14, vy=.14, wz=.35)            # the commands the policy was trained and validated on
V_MIN, T_MAX = 10.0, 70                          # stop below this supply voltage (V) / above this servo temperature (C)


def get(path): return json.loads(urllib.request.urlopen(URL + path, timeout=2).read())
def post(path, d): return json.loads(urllib.request.urlopen(urllib.request.Request(URL + path, json.dumps(d).encode(), method='POST'), timeout=2).read())
def smooth(s): s = min(1., max(0., s)); return s*s*s*(10 + s*(-15 + 6*s))


class Joints:
    """ticks <-> model rad with calibration/twin_map.json; two joint ranges in model rad:
    real = the measured limits minus 40 ticks (for stepping in and out of the stance),
    policy = real AND the simulation model's range (the envelope the controller was tested in)."""

    def __init__(self):
        self.cal = json.loads((ROOT/'calibration/twin_map.json').read_text())
        self.lim = json.loads((ROOT/'calibration/joint_limits.json').read_text())['joints']
        xml = {j.get('name'): [float(v) for v in j.get('range').split()]
               for j in ET.parse(ROOT/'simulation/arachne.xml').getroot().iter('joint') if j.get('name') in JOINTS}
        lo, hi = [], []
        for j in JOINTS:
            a, b = self.q(j, self.lim[j]['min'] + 40), self.q(j, self.lim[j]['max'] - 40)
            lo.append(min(a, b)); hi.append(max(a, b))
        self.real = np.array(lo), np.array(hi)
        self.policy = np.maximum(self.real[0], [xml[j][0] for j in JOINTS]), np.minimum(self.real[1], [xml[j][1] for j in JOINTS])

    def q(self, j, ticks): c = self.cal[j]; return c['sign']*(ticks - c['zero'])*TICK

    def q_of(self, ticks): return np.array([self.q(j, ticks[j]) for j in JOINTS])

    def ticks_of(self, q):
        out = {}
        for j, v in zip(JOINTS, q):
            c = self.cal[j]; t = c['zero'] + v/(c['sign']*TICK)
            out[j] = int(round(min(self.lim[j]['max'] - 40, max(self.lim[j]['min'] + 40, t))))
        return out

    def stand_q(self):
        """The normal stand pose of stand.py, in model rad."""
        t = {}
        for j in JOINTS:
            part = j.split('_')[1]; c = self.cal[j]
            t[j] = round(c['zero'] - stand.OFF[part]/(c['sign']*TICK)) + stand.STAND[part]
        return self.q_of(t)


def fk(q):
    """Foot positions [6,3] in the body frame, with the policy's own leg model (inverse of gait.leg_ik)."""
    q = np.asarray(q).reshape(6, 3)
    az = gait.AZIMUTH + q[:, 0]; a, b = gait.A0 + q[:, 1], gait.B0 + q[:, 2]
    r = .05 + gait.L1*np.cos(a) + gait.L2*np.cos(a + b)
    return np.stack([gait.COXA[:, 0] + r*np.cos(az), gait.COXA[:, 1] + r*np.sin(az), gait.L1*np.sin(a) + gait.L2*np.sin(a + b)], -1)


def ik(feet, low, high): return gait.leg_ik(feet[None, :, 0], feet[None, :, 1], feet[None, :, 2], low=low, high=high)[0]


def restance(q_from, q_to, low, high, hz, height_first, lift=.02, t_step=.7, t_height=1.4):
    """Joint targets from one stance to another without dragging the feet: change the body height with all
    feet down, and move the feet one tripod at a time (L1 L3 L5, then L2 L4 L6). Into the policy stance the feet
    step out first and then the body goes down (the other way round folds the knees past their limits)."""
    a, b = fk(q_from), fk(q_to); cur = a.copy()
    def height():
        start = cur.copy(); n = max(1, int(t_height*hz))
        for k in range(1, n + 1):
            cur[:, 2] = start[:, 2] + (b[:, 2] - start[:, 2])*smooth(k/n); yield ik(cur, low, high)
    def steps():
        for tri in ((0, 2, 4), (1, 3, 5)):
            start = cur.copy(); n = max(1, int(t_step*hz))
            for k in range(1, n + 1):
                s = smooth(k/n)
                for i in tri:
                    cur[i, :2] = start[i, :2] + (b[i, :2] - start[i, :2])*s
                    cur[i, 2] = start[i, 2] + lift*math.sin(math.pi*k/n)
                yield ik(cur, low, high)
    for part in ((height, steps) if height_first else (steps, height)): yield from part()


class Commands(threading.Thread):
    """Latest velocity command from UDP (gamepad_robot.py, the cockpit) with a 0.5 s dead-man."""

    def __init__(self, port=UDP_PORT):
        super().__init__(daemon=True)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.s.bind(('127.0.0.1', port))
        self.v = np.zeros(3); self.t = 0.

    def run(self):
        while True:
            try:
                d = json.loads(self.s.recv(512))
                self.v = np.array([float(d.get('vx', 0)), float(d.get('vy', 0)), float(d.get('wz', 0))]); self.t = time.time()
            except (ValueError, OSError): pass

    def get(self): return self.v.copy() if time.time() - self.t < .5 else np.zeros(3)


def clip_cmd(v, scale=1.):
    return np.array([np.clip(v[0], -LIMIT['vx'], LIMIT['vx']), np.clip(v[1], -LIMIT['vy'], LIMIT['vy']),
                     np.clip(v[2], -LIMIT['wz'], LIMIT['wz'])])*scale


def imu_of(s, use_imu):
    """(gravity, gyro, accelerometer, ok) from a /sense or /sync answer, body frame; without IMU: level and still."""
    if not use_imu: return np.array([0., 0., -1.]), np.zeros(3), None, True
    m = s.get('imu') or {}
    if not m.get('ok') or not m.get('gravity') or m.get('age', 1) > .25: return None, None, None, False
    return np.array(m['gravity']), np.array(m['gyro_body']), np.array(m['acc_body']) if m.get('acc_body') else None, True


def write_status(**k):
    try: STATUS.write_text(json.dumps(dict(k, t=time.time())))
    except OSError: pass


def dry_run(J, params, hz):
    """Everything except the servos: stand pose -> policy stance -> 12 s of commands -> back, on a level IMU."""
    q0 = J.stand_q(); q_n = TerrainGait(params, *J.policy).neutral()
    frames = [q0] + list(restance(q0, q_n, *J.real, hz, False))
    g = TerrainGait(params, *J.policy); g.reset(q_now=frames[-1]); w0 = len(frames)
    sched = [(0, [.10, 0, 0]), (3, [0, .10, 0]), (5, [-.08, 0, 0]), (7, [0, 0, .3]), (9, [.14, 0, 0]), (11, [0, 0, 0])]
    for k in range(int(12*hz)):
        c = next(v for t0, v in reversed(sched) if k/hz >= t0)
        frames.append(g.step(np.array(c, float), np.array([0, 0, -1.]), np.zeros(3), 1/hz))
    w1 = len(frames); frames += list(restance(frames[-1], q0, *J.real, hz, True))
    T = np.array([[J.ticks_of(q)[j] for j in JOINTS] for q in frames])
    speed = np.abs(np.diff(T, axis=0)).max(0)*hz
    print(f'{len(frames)} frames @ {hz} Hz ({len(frames)/hz:.1f} s)')
    for i, j in enumerate(JOINTS):
        lo, hi = J.lim[j]['min'] + 40, J.lim[j]['max'] - 40
        pol = sorted(J.ticks_of(np.where(np.arange(18) == i, b, 0.))[j] for b in J.policy)
        W = T[w0:w1, i]; edge = int(((W <= pol[0]) | (W >= pol[1])).sum())       # while walking: tested envelope
        print(f'  {j:8s} ticks {T[:, i].min():4d}..{T[:, i].max():4d}  (grenzen {lo}..{hi})  piek {speed[i]:5.0f} ticks/s'
              + (f'  AAN DE GRENS tijdens lopen: {edge} van {w1 - w0} frames' if edge else ''))
    print(f'snelste gewricht {speed.max():.0f} ticks/s (STS3215 ~3000 bij 12 V)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--policy', type=Path, default=ROOT/'training/policies/omni.json')
    ap.add_argument('--hz', type=float, default=50)
    ap.add_argument('--dry', action='store_true', help='plan and check, no motion')
    ap.add_argument('--drive', nargs=3, type=float, metavar=('VX', 'VY', 'WZ'), help='fixed command instead of UDP')
    ap.add_argument('--seconds', type=float, default=5., help='with --drive: how long')
    ap.add_argument('--loop', help='keep going until this file exists (gamepad_robot.py)')
    ap.add_argument('--udp', type=int, default=UDP_PORT, help='command port (the browser simulation uses 5873)')
    ap.add_argument('--speed', type=float, default=1., help='scale every command (0.5 = half speed for first trials)')
    ap.add_argument('--no-adapt', action='store_true', help='only the trained policy, no levelling / rough mode')
    ap.add_argument('--no-imu', action='store_true', help='no IMU: open loop (and no adaptation)')
    a = ap.parse_args()
    J = Joints(); params = gait.load(a.policy); hz = a.hz; dt = 1/hz
    if a.dry: return dry_run(J, params, hz)

    s = get('/sense')
    if s['servos'] < 18: raise SystemExit(f"maar {s['servos']} servo's gevonden, ik verwacht er 18")
    if s['torque'] < 18: raise SystemExit('niet alle servo\'s staan aan: eerst opstaan (stand.py)')
    use_imu = not a.no_imu
    if use_imu and not imu_of(s, True)[3]:
        m = s.get('imu') or {}
        why = 'geen IMU' if not m.get('ok') else 'IMU nog niet uitgelijnd: tools/imu_align.py' if not m.get('aligned') else 'IMU-data te oud'
        raise SystemExit(f'{why} (of start met --no-imu: dan zonder terugkoppeling)')
    cmds = None
    if not a.drive: cmds = Commands(a.udp); cmds.start()

    g = TerrainGait(params, *J.policy, adapt=not (a.no_adapt or a.no_imu))
    q_neutral = g.neutral()
    q_start = J.q_of(s['ticks'])
    send = lambda q: post('/sync', {'ticks': J.ticks_of(q)})
    def step_through(frames, what):
        nonlocal s
        try:
            for q in frames: s = send(q); pace()
        except Exception as e:                       # e.g. 409: the fall guard holds the servos
            write_status(mode='gevallen', why=f'{what}: {e}'); raise SystemExit(f'STOP tijdens {what}: {e}')
    t_next = time.time()
    def pace():
        nonlocal t_next
        t_next += dt; w = t_next - time.time()
        if w > 0: time.sleep(w)
        else: t_next = time.time()

    print('instappen: lager en voeten verder naar buiten…', flush=True); write_status(mode='instappen')
    step_through(restance(q_start, q_neutral, *J.real, hz, False), 'instappen')
    g.reset(q_now=q_neutral)
    t0 = time.time(); stop_at = None; why = 'klaar'; imu_bad = 0.; last_print = 0.; fell = False
    last_grav = np.array([0, 0, -1.])
    while True:
        now = time.time()
        if stop_at is None:
            if a.drive and now - t0 > a.seconds: stop_at, why = now, 'tijd om'
            elif a.loop and os.path.exists(a.loop): stop_at, why = now, 'gestopt'
            elif (s.get('volt') or 99) < V_MIN: stop_at, why = now, f"spanning te laag ({s['volt']} V)"
            elif (s.get('temp') or 0) > T_MAX: stop_at, why = now, f"servo te warm ({s['temp']} °C)"
        want = np.zeros(3) if stop_at else clip_cmd(np.array(a.drive) if a.drive else cmds.get(), a.speed)
        grav, gyro, acc, ok = imu_of(s, use_imu)
        if not ok:                                   # IMU dropped out: stand still on the last good estimate
            imu_bad += dt; want = np.zeros(3)
            grav, gyro, acc = last_grav, np.zeros(3), None
            if imu_bad > 1. and stop_at is None: stop_at, why = now, 'IMU weg'
        else: imu_bad = 0.; last_grav = grav
        q = g.step(want, grav, gyro, dt, acc=acc)
        if g.fallen:
            fell = True; why = f'omgevallen ({g.tilt:.0f}°)'; break
        try: s = send(q)
        except Exception as e: why = f'servo-bus: {e}'; fell = True; break
        if s.get('servos', 18) < 18: why = 'servo weg'; fell = True; break
        if now - last_print > 1:
            last_print = now; st = g.status()
            print(f"{now - t0:5.1f} s  cmd {st['command']}  kanteling {st['tilt_deg']:4.1f}°  ruw {st['rough']:.2f}  "
                  f"niveau {max(map(abs, st['level_mm'])):4.1f} mm  {s.get('volt')} V", flush=True)
            write_status(mode='lopen', **st)
        # stopping: let the command run down to zero and the feet come down for one cycle, then step out
        if stop_at and np.linalg.norm(g.cmd) < .004 and now - stop_at > 1.2: break
        pace()
    if fell:
        print('STOP:', why, '- servo\'s houden hun laatste stand', flush=True); write_status(mode='gevallen', why=why)
        raise SystemExit(2)
    print(f'{why}: uitstappen naar de gewone stand…', flush=True); write_status(mode='uitstappen')
    step_through(restance(g.sm[0], J.stand_q(), *J.real, hz, True), 'uitstappen')
    write_status(mode='klaar', why=why); print('klaar', flush=True)


if __name__ == '__main__': main()
