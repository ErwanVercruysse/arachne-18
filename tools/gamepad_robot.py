"""Drive the REAL robot with a Bluetooth/USB game controller connected to the Pi (Recon Cloud, Xbox, PS…).

    A            stand up              (stand.py)
    B            lie down              (stand.py --lie)
    X            dance on/off          (dance.py --amp .5 --loop)
    Y            horn (on the Mac, tools/mac_sounds.py)
    View         rest: lie down, then torque off
    L-stick ↑/↓, RT/LT       walk forward/back while held
    R-stick ←/→, d-pad ←/→   turn while held
    RB / d-pad ↑↓            speed: normaal -> snel -> max (max = fastest joint at the servo's speed limit)
    LB           back to normaal
    L3           say the status (Mac)
    R3           self-walk on/off: the trained policy runs live (tools/policy_walk.py), closed loop on the IMU;
                 then L-stick = walk in any direction, R-stick ←/→ = turn, continuously
    Menu/Xbox    stop everything (the robot keeps standing)

The same actions are available to the web pages through a local command port (127.0.0.1:5871), which the
twin server exposes as /robot/... for logged-in users.

    .venv/bin/python tools/gamepad_robot.py        # runs until Ctrl-C; reconnects when the pad comes back
    (also installed as the systemd service arachne-pad)

Walking and turning are planned ONCE after the robot stands (tools/plan_gaits.py: the same maths as walk.py /
spin.py) and then played back from memory, so a stick push starts the robot at once. Releasing the stick stops
at the next moment that all six feet are down (at most one phase, ~0.7 s); pushing again resumes from there.
Stand, lie and dance still run their own scripts, one at a time, after the feet are back home.
Reads /dev/input/event* directly (no packages).
"""
import fcntl, glob, json, os, socket, struct, subprocess, sys, threading, time, urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PY = str(ROOT/'.venv/bin/python') if (ROOT/'.venv/bin/python').exists() else sys.executable
URL = os.environ.get('ARACHNE_URL', 'http://localhost:5870')
STOP = '/tmp/arachne_pad_stop'
POLICY_UDP = ('127.0.0.1', 5872)                    # tools/policy_walk.py listens here in self-walk mode
POLICY_STATUS = Path('/tmp/arachne_policy.json')
POLICY_MAX = (.14, .14, .35)                         # vx, vy (m/s), wz (rad/s): what the policy was validated on
EV_KEY, EV_ABS = 1, 3
BTN = {304: 'A', 305: 'B', 307: 'X', 308: 'Y', 310: 'LB', 311: 'RB', 314: 'View', 315: 'Menu', 316: 'Xbox', 317: 'L3', 318: 'R3'}
ABS_X, ABS_Y, ABS_Z, ABS_RZ = 0, 1, 2, 5
EVENT = struct.Struct('llHHi')                       # struct input_event (64-bit: 24 bytes)


def log(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
def get(path): return json.loads(urllib.request.urlopen(URL + path, timeout=2).read())
def post(path, d): return json.loads(urllib.request.urlopen(urllib.request.Request(URL + path, json.dumps(d).encode(), method='POST'), timeout=2).read())


def standing():
    try: return get('/state').get('lift', 0) > .03
    except Exception: return False


class Gait(threading.Thread):
    """Plays the pre-planned cycles at 25 Hz.
    want = what the sticks ask for; cur = gait being played; k = frame. cur None with k > 0 = parked mid-cycle
    (all feet down, but not home); home_req = finish that cycle so the feet are home (before a script)."""
    def __init__(self):
        super().__init__(daemon=True)
        self.plan = None; self.planning = False
        self.want = None; self.cur = None; self.k = 0; self.parked = None
        self.finishing = False; self.home_req = False
        self.level = 'normaal'

    def replan(self):
        """Plan from the current standing pose, in a separate process (MuJoCo is heavy and frees its memory after)."""
        if self.planning: return
        self.planning = True; self.plan = None
        def work():
            t = time.time(); r = None
            try:
                r = subprocess.run([PY, str(ROOT/'tools/plan_gaits.py')], cwd=ROOT, capture_output=True, text=True, timeout=90)
                p = json.loads(r.stdout)
                bad = {m: v['clipped'] for m, v in p['motions'].items() if v['clipped']}
                if bad: log('LET OP, buiten de grenzen geklemd:', bad)
                self.plan = p; log(f'lopen/draaien klaar ({time.time()-t:.1f} s gepland)')
            except Exception as e: log('plannen mislukt:', e, (r.stderr[-300:] if r else ''))
            self.planning = False
        threading.Thread(target=work, daemon=True).start()

    def send(self, frame):
        p = self.plan; ports = {}
        for j, t in frame.items(): ports.setdefault(p['port'][j], {})[str(p['sid'][j])] = [t & 0xff, t >> 8, 0, 0, 0, 0]
        for port, data in ports.items(): post('/cmd', {'port': port, 'op': 'sync', 'addr': 42, 'data': data})

    def run(self):
        nxt = time.time()
        while True:
            p = self.plan
            if not p: time.sleep(.05); nxt = time.time(); continue
            if self.cur is None:
                if self.k:                                          # parked mid-cycle
                    if self.home_req or (self.want and self.want != self.parked): self.cur, self.finishing = self.parked, True
                    elif self.want == self.parked: self.cur, self.finishing = self.parked, False
                elif self.want:
                    self.cur, self.finishing = self.want, False; log('start', self.want)
                if not self.k: self.home_req = False                # feet are home already
                if self.cur is None: time.sleep(.02); nxt = time.time(); continue
            m = p['motions'][self.cur]; n = m['n']; hz = m['hz']
            try: self.send(m['frames'][self.k])
            except Exception as e: log('sturen mislukt:', e); self.want = None
            self.k = (self.k + 1) % n
            if self.k == 0:
                try:
                    v = min((d.get('volt') or 99) for d in get('/state')['joints'].values())
                    if v < 10.: log(f'spanning te laag ({v} V), gestopt'); self.want = None
                except Exception: pass
            if self.k % (n//3) == 0:                                # all six feet are on the ground here
                if self.finishing:
                    if self.k == 0: self.cur = None; self.finishing = False; self.home_req = False
                elif self.want != self.cur or self.home_req:
                    self.parked = self.cur; self.cur = None
                    log('gestopt' if self.k == 0 else 'gestopt (voeten neer, gaat verder waar hij was)')
            nxt += 1/hz
            dt = nxt - time.time()
            if dt > 0: time.sleep(dt)
            else: nxt = time.time()

    def settle(self, timeout=6.):
        """Finish a parked cycle so the feet are home before a script takes over."""
        self.want = None; self.home_req = True
        end = time.time() + timeout
        while (self.home_req or self.cur is not None) and time.time() < end and self.plan: time.sleep(.02)
        self.home_req = False


class Actions(threading.Thread):
    """stand / lie / dance strictly one after the other (a button press only queues a request).
    A new request ends a running dance first (stop file, 1 s fade); stand.py is short and is let finish."""
    def __init__(self, g):
        super().__init__(daemon=True)
        self.g = g; self.cv = threading.Condition(); self.pending = None; self.proc = None; self.what = None

    def busy(self): return self.what is not None or self.pending is not None

    def request(self, what, args, loop=False):
        with self.cv:
            if what == self.what or (self.pending and self.pending[0] == what): return     # already doing / queued
            self.pending = (what, args, loop)
            if self.proc and self.proc.poll() is None: Path(STOP).touch()
            self.cv.notify()

    def stop(self):
        with self.cv:
            self.pending = None
            if self.proc and self.proc.poll() is None: Path(STOP).touch(); log(f'{self.what} stoppen')

    def run(self):
        while True:
            try: self.step()
            except Exception as e:
                log('actie mislukt:', e); self.proc = None; self.what = None; time.sleep(.5)

    def step(self):
        if True:
            with self.cv:
                while self.pending is None: self.cv.wait()
            if self.proc:                                            # let the current script end first
                try: self.proc.wait(10)
                except subprocess.TimeoutExpired: log('script stopt niet, terminate'); self.proc.terminate(); self.proc.wait(3)
            with self.cv: job, self.pending = self.pending, None
            if not job: return
            what, args, loop = job
            self.g.settle(); self.g.plan = None
            if subprocess.run([str(ROOT/'tools/busy.sh')], capture_output=True).returncode != 0:
                log('er draait al een ander bewegingsscript, ik doe niks'); return
            try: os.remove(STOP)
            except FileNotFoundError: pass
            cmd = [PY, str(ROOT/'tools'/args[0]), *args[1:]] + (['--loop', STOP] if loop else [])
            log('start', what)
            with self.cv: self.what = what; self.proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.proc.wait()
            if what == 'rust' and self.proc.returncode == 0:              # lie down done: switch all torque off
                J = get('/state')['joints']; ports = {}
                for d in J.values(): ports.setdefault(d['port'], {})[str(d['sid'])] = [0]
                for port, data in ports.items(): post('/cmd', {'port': port, 'op': 'sync', 'addr': 40, 'data': data})
            log(f'{what} klaar')
            with self.cv: self.proc = None; self.what = None
            if self.pending is None and standing(): self.g.replan()


def find_pad():
    for dev in sorted(glob.glob('/sys/class/input/event*')):
        try:
            name = Path(dev, 'device/name').read_text().strip()
            keys = Path(dev, 'device/capabilities/key').read_text().split()
        except OSError: continue
        if len(keys) >= 5 and int(keys[-5], 16) >> 48 & 1:          # BTN_SOUTH (304): a gamepad
            return '/dev/input/' + Path(dev).name, name
    return None, None


def absinfo(fd, code):
    _, lo, hi, *_ = struct.unpack('6i', fcntl.ioctl(fd, 0x80184540 + code, b'\0'*24))
    return lo, hi


ABS_GAS, ABS_BRAKE, ABS_HAT0X, ABS_HAT0Y = 9, 10, 16, 17
LEVELS = ('normaal', 'snel', 'max')
SAY = {'opstaan': 'Ik sta op', 'liggen': 'Ik ga liggen', 'dansen': 'Dansen!', 'rust': 'Ik ga rusten', 'stop': 'Stop',
       'zelf': 'Ik loop zelf', 'pakken': 'Ik pak het'}


class Robot:
    """Everything the robot can do, shared by the controller on the Pi and the web pages (/robot/... on the twin).
    Also keeps a list of events (things to say or play) that tools/mac_sounds.py turns into sound on the Mac."""
    def __init__(self):
        self.g = Gait(); self.a = Actions(self.g); self.g.start(); self.a.start()
        self.events = []; self.n = 0; self.ev = threading.Condition()
        self.web_drive = 0.                          # time of the last drive command from a web page (dead-man)
        self.pad = None
        self.pad_vel = [0., 0., 0.]; self.web_vel = ([0., 0., 0.], 0.)   # self-walk: continuous velocities
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        threading.Thread(target=self._deadman, daemon=True).start()
        threading.Thread(target=self._velocities, daemon=True).start()

    def event(self, **e):
        with self.ev:
            self.n += 1; self.events.append(dict(e, n=self.n, t=time.time())); self.events = self.events[-50:]; self.ev.notify_all()

    def wait_events(self, since, timeout=20.):
        with self.ev:
            if self.n <= since: self.ev.wait(timeout)
            return [e for e in self.events if e['n'] > since], self.n

    def do(self, what):
        g, a = self.g, self.a
        if what == 'opstaan': g.want = None; a.request('opstaan', ['stand.py'])
        elif what == 'liggen': g.want = None; a.request('liggen', ['stand.py', '--lie'])
        elif what == 'rust': g.want = None; a.request('rust', ['stand.py', '--lie'])          # lie down, then torque off
        elif what == 'dansen':
            if a.what == 'dansen': a.stop(); what = 'stop'
            elif standing(): g.want = None; a.request('dansen', ['dance.py', '--amp', '.5'], loop=True)   # amp 1 leaves the joint limits
            else: self.event(say='Ik moet eerst rechtstaan'); return
        elif what == 'zelf':                                     # self-walk: the policy runs live, toggles
            if a.what == 'zelf': a.stop(); what = 'stop'
            elif standing(): g.want = None; a.request('zelf', ['policy_walk.py'], loop=True)
            else: self.event(say='Ik moet eerst rechtstaan'); return
        elif what == 'pakken':                                   # four legs, pick up what is in front (tools/grab.py)
            if a.what == 'pakken': a.stop(); what = 'stop'
            elif standing(): g.want = None; a.request('pakken', ['grab.py'], loop=True)
            else: self.event(say='Ik moet eerst rechtstaan'); return
        elif what == 'stop': g.want = None; a.stop()
        elif what == 'toeter': self.event(sound='toeter'); return
        elif what == 'status': self.event(say=self.spoken_status()); return
        elif what == 'hallo': self.event(say='Hallo! Ik ben Arachne, een spin met achttien servo\'s.'); return
        else: return
        if what in SAY: self.event(say=SAY[what])

    def drive(self, name, web=False):
        """name = lopen / achteruit / links draaien / rechts draaien, or None to stop at the next feet-down moment."""
        g, a = self.g, self.a
        if web: self.web_drive = time.time() if name else 0.
        if not name:
            g.want = None; return True
        if a.busy(): return False                                  # a script owns the servos
        if not g.plan:
            if not g.planning and standing(): g.replan()
            if not g.planning: self.event(say='Ik moet eerst rechtstaan')
            return False
        key = f'{g.level}/{name}'
        if g.want != key: g.want = key
        return True

    def level(self, lvl=None, step=0):
        g = self.g
        lvl = lvl if lvl in LEVELS else LEVELS[max(0, min(2, LEVELS.index(g.level) + step))] if step else LEVELS[(LEVELS.index(g.level) + 1) % 3]
        if lvl == g.level: return
        g.level = lvl; log('snelheid:', lvl); self.event(say=f'Snelheid {lvl}')
        if g.want: g.want = f"{lvl}/{g.want.split('/')[1]}"         # switches after this cycle

    def self_walking(self): return self.a.what == 'zelf'

    def web_velocity(self, vx, vy, wz):
        """Cockpit stick (its own scale: .07 / .05 m/s, .5 rad/s) -> the policy's range."""
        self.web_vel = ([vx/.07*POLICY_MAX[0], vy/.05*POLICY_MAX[1], wz/.5*POLICY_MAX[2]], time.time())

    def _velocities(self):
        """Self-walk: send the pad's (else the web page's) velocity to policy_walk.py 10x per second;
        it stands still by itself when this stops (0.5 s dead-man)."""
        while True:
            time.sleep(.1)
            if not self.self_walking(): continue
            v, t = self.web_vel
            if any(self.pad_vel) or time.time() - t > .7: v = self.pad_vel
            try: self.udp.sendto(json.dumps(dict(zip(('vx', 'vy', 'wz'), v))).encode(), POLICY_UDP)
            except OSError: pass

    def _deadman(self):
        while True:
            time.sleep(.1)
            if self.web_drive and time.time() - self.web_drive > .7: self.web_drive = 0.; self.g.want = None

    def spoken_status(self):
        try:
            s = get('/state'); v = min((d.get('volt') or 99) for d in s['joints'].values())
            on = sum(1 for d in s['joints'].values() if d.get('torque'))
            return f"Spanning {v:.1f} volt. {on} servo's aan. Snelheid {self.g.level}.".replace('.', ' komma ', 1)
        except Exception: return 'Ik kan de servo\'s niet lezen.'

    def status(self):
        g, a = self.g, self.a
        pol = None
        if self.self_walking():
            try: pol = json.loads(POLICY_STATUS.read_text())
            except (OSError, ValueError): pass
        return dict(level=g.level, walking=(g.cur or g.want or '').split('/')[-1] or None, script=a.what, queued=a.pending[0] if a.pending else None,
                    planned=bool(g.plan), planning=g.planning, standing=standing(), controller=self.pad, events=self.n, policy=pol)


def run(path, name, r):
    fd = os.open(path, os.O_RDONLY)
    codes = (ABS_X, ABS_Y, ABS_Z, ABS_RZ, ABS_GAS, ABS_BRAKE, ABS_HAT0X, ABS_HAT0Y)
    rng = {}
    for c in codes:
        try: rng[c] = absinfo(fd, c)
        except OSError: pass
    norm = lambda c, v: 0. if rng[c][1] == rng[c][0] else (2*(v - rng[c][0])/(rng[c][1] - rng[c][0]) - 1)
    trig = lambda c, v: 0. if rng[c][1] == rng[c][0] else (v - rng[c][0])/(rng[c][1] - rng[c][0])
    ax = {}; last = {}; held = None; r.pad = name; log(f'controller: {name} ({path})'); r.event(sound='verbonden')
    BUTTONS = {'A': 'opstaan', 'B': 'liggen', 'X': 'dansen', 'Y': 'toeter', 'View': 'rust', 'Menu': 'stop', 'Xbox': 'stop',
               'L3': 'status', 'R3': 'zelf'}
    try:
        while True:
            _, _, typ, code, val = EVENT.unpack(os.read(fd, EVENT.size))
            if typ == EV_KEY and val == 1 and code in BTN:
                b = BTN[code]; now = time.time()
                if now - last.get(b, 0) < .4: continue                      # ignore bouncing / mashing
                last[b] = now; log('knop', b)
                if b in BUTTONS: r.do(BUTTONS[b])
                elif b == 'RB': r.level()                                   # normaal -> snel -> max -> normaal
                elif b == 'LB': r.level('normaal')
            elif typ == EV_ABS and code in rng:
                if code in (ABS_HAT0X, ABS_HAT0Y): ax[code] = val          # -1 / 0 / 1
                elif code in (ABS_GAS, ABS_BRAKE): ax[code] = trig(code, val)
                else: ax[code] = norm(code, val)
                if code == ABS_HAT0Y and val: r.level(step=-val); continue  # d-pad up/down: faster / slower
                y, rx = ax.get(ABS_Y, 0), ax.get(ABS_Z, 0)
                rt, lt, hx = ax.get(ABS_GAS, 0), ax.get(ABS_BRAKE, 0), ax.get(ABS_HAT0X, 0)
                if r.self_walking():                                       # continuous: any direction + turn
                    dz = lambda v: 0. if abs(v) < .12 else (v - .12*(1 if v > 0 else -1))/.88
                    x = ax.get(ABS_X, 0)
                    r.pad_vel = [max(-1., min(1., -dz(y) + rt - lt))*POLICY_MAX[0], -dz(x)*POLICY_MAX[1],
                                 max(-1., min(1., -dz(rx) - hx))*POLICY_MAX[2]]
                    held = None; continue
                r.pad_vel = [0., 0., 0.]
                want = ('lopen' if y < -.5 or rt > .5 else 'achteruit' if y > .5 or lt > .5 else
                        'links draaien' if rx < -.5 or hx < 0 else 'rechts draaien' if rx > .5 or hx > 0 else None)
                calm = abs(y) < .35 and abs(rx) < .35 and rt < .3 and lt < .3 and not hx
                if want and want != held: held = want if r.drive(want) else None
                elif not want and held and calm: held = None; r.drive(None)
    finally: os.close(fd); r.pad = None; r.pad_vel = [0., 0., 0.]


def serve(r, port=5871):
    """Local command port for the twin server (/robot/... is proxied here); only 127.0.0.1."""
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def reply(self, obj, code=200):
            b = json.dumps(obj).encode(); self.send_response(code); self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        def do_GET(self):
            if self.path.startswith('/status'): return self.reply(r.status())
            if self.path.startswith('/events'):
                since = int(self.path.split('since=')[1].split('&')[0]) if 'since=' in self.path else r.n
                ev, n = r.wait_events(since); return self.reply({'events': ev, 'n': n})
            self.reply({'error': 'onbekend'}, 404)
        def do_POST(self):
            d = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
            if self.path == '/do': r.do(d.get('what')); return self.reply({'ok': True, **r.status()})
            if self.path == '/drive':
                name = d.get('name')
                if r.self_walking() and any(k in d for k in ('vx', 'vy', 'wz')):
                    r.web_velocity(float(d.get('vx', 0)), float(d.get('vy', 0)), float(d.get('wz', 0))); return self.reply({'ok': True})
                if name is None and any(k in d for k in ('vx', 'vy', 'wz')):   # cockpit-style velocities
                    vx, wz = float(d.get('vx', 0)), float(d.get('wz', 0))
                    name = 'lopen' if vx > .02 else 'achteruit' if vx < -.02 else 'links draaien' if wz > .15 else 'rechts draaien' if wz < -.15 else None
                return self.reply({'ok': r.drive(name, web=True)})
            if self.path == '/level': r.level(d.get('level'), int(d.get('step', 0))); return self.reply({'ok': True, 'level': r.g.level})
            self.reply({'error': 'onbekend'}, 404)
    ThreadingHTTPServer(('127.0.0.1', port), H).serve_forever()


def main():
    r = Robot(); told = False
    threading.Thread(target=serve, args=(r,), daemon=True).start()
    if standing(): r.g.replan()
    while True:
        path, name = find_pad()
        if not path:
            if not told: log('geen controller, ik wacht…'); told = True
            time.sleep(2); continue
        told = False
        try: run(path, name, r)
        except OSError: log('controller weg'); r.g.want = None; r.event(sound='weg')


if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: pass
