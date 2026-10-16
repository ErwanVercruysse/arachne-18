"""Live 3D twin of the real ARACHNE: every servo the adapters see moves the MuJoCo model.

    .venv/bin/python tools/live_twin.py      -> http://localhost:5870

Reads both Waveshare adapters continuously (only the IDs that answer, missing IDs are
re-pinged one at a time), converts ticks to joint angles with calibration/twin_map.json
and streams body poses + telemetry over SSE. Read-only by default; POST /cmd lets other
tools talk to the bus through this server while it owns the serial ports.

/sim is a separate physics simulation of the robot (tools/sim_world.py), started on first use;
it never touches the real servos.
"""
import glob, gzip, ipaddress, json, math, os, re, shutil, socket, subprocess, sys, threading, time, urllib.error, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sts import Bus, ADDR_POS, ADDR_LOAD, ADDR_VOLT, ADDR_TORQUE

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT/'simulation'
MAP_FILE = ROOT/'calibration/twin_map.json'
PORT = 5870
TICK = 2*math.pi/4096

JOINTS = json.loads((SIM/'joint_map.json').read_text())['joints']
SID2JOINT = {j['servo_id']: j['name'] for j in JOINTS}

# ---------------------------------------------------------------- calibration map
# ticks -> rad: sign * (ticks - zero) * 2pi/4096. zero = ticks in the CAD-neutral (standing) pose.
DEFAULT = {j['name']: {'zero': 2048, 'sign': 1, 'checked': False} for j in JOINTS}
DEFAULT.update({
    'L5_yaw': {'zero': 2049, 'sign': 1, 'checked': False},
    'L5_hip': {'zero': 1646, 'sign': 1, 'checked': True},   # hand-placed stand pose; +ticks = up = +model
    'L5_knee': {'zero': 1283, 'sign': 1, 'checked': False},
    'L4_hip': {'zero': 2048, 'sign': 1, 'checked': False},
})
cal = {k: dict(v) for k, v in DEFAULT.items()}
if MAP_FILE.exists(): cal.update(json.loads(MAP_FILE.read_text()))
def save_cal(): MAP_FILE.write_text(json.dumps(cal, indent=2))

# ---------------------------------------------------------------- model + mesh export
model = mujoco.MjModel.from_xml_path(str(SIM/'arachne_fixed.xml'))
data = mujoco.MjData(model)
QADR = {model.joint(i).name: model.joint(i).qposadr[0] for i in range(model.njnt)}


def quat_mat(q):
    m = np.zeros(9); mujoco.mju_quat2Mat(m, q); return m.reshape(3, 3)


HIDE = ('shell', 'dorsal_deck', 'body_deck', 'ref_', 'battery_tray', 'controller_plate', 'pcb_spacer', 'body_controller', 'body_tray')   # not built on the real robot yet


def build_scene():
    """All mesh geoms merged per body, vertices in that body's frame -> one binary blob."""
    per = {}
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH: continue
        b = model.geom_bodyid[g]
        if b == 0: continue
        mid = model.geom_dataid[g]
        if any(h in model.mesh(mid).name.lower() for h in HIDE): continue
        va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        v = model.mesh_vert[va:va+vn] @ quat_mat(model.geom_quat[g]).T + model.geom_pos[g]
        f = model.mesh_face[fa:fa+fn]
        rgba = model.geom_rgba[g] if model.geom_matid[g] < 0 else model.mat_rgba[model.geom_matid[g]]
        per.setdefault(b, []).append((v.astype(np.float32), f.astype(np.uint32), rgba))
    meta, blobs, off = [], [], 0
    for b, parts in per.items():
        # split each body by colour so materials stay right
        groups = {}
        for v, f, c in parts: groups.setdefault(tuple(np.round(c, 3)), []).append((v, f))
        for c, items in groups.items():
            verts, faces, n = [], [], 0
            for v, f in items: verts.append(v); faces.append(f+n); n += len(v)
            V = np.concatenate(verts).tobytes(); F = np.concatenate(faces).tobytes()
            meta.append(dict(body=int(b), name=model.body(b).name, rgba=[float(x) for x in c],
                             v_off=off, v_len=len(V), f_off=off+len(V), f_len=len(F)))
            blobs += [V, F]; off += len(V)+len(F)
    return meta, b''.join(blobs)


def tip_offset():
    """Lowest tibia vertex below its foot site, in the stand pose (foot sits on the ground there)."""
    data.qpos[:] = 0; mujoco.mj_kinematics(model, data); off = []
    for n in range(1, 7):
        b = model.body(f'L{n}_TIBIA').id; zs = []
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != b or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH: continue
            mid = model.geom_dataid[g]; v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid]+model.mesh_vertnum[mid]]
            v = v @ quat_mat(model.geom_quat[g]).T + model.geom_pos[g]
            zs.append((v @ data.xmat[b].reshape(3, 3).T + data.xpos[b])[:, 2].min())
        off.append(min(zs) - data.site(f'L{n}_foot').xpos[2])
    return min(off)


print('meshes samenvoegen...', flush=True)
SCENE_META, SCENE_BIN = build_scene()
TIP = tip_offset()
data.qpos[:] = 0; mujoco.mj_kinematics(model, data)
GROUND = min(data.site(f'L{n}_foot').xpos[2] for n in range(1, 7)) + TIP   # floor height in the stand pose
print(f'{len(SCENE_META)} mesh-groepen, {len(SCENE_BIN)/1e6:.1f} MB', flush=True)

# ---------------------------------------------------------------- bus polling
buses = {p: Bus(p) for p in sorted(glob.glob('/dev/cu.usbmodem5A*') + glob.glob('/dev/cu.usbserial*'))}
locks = {p: threading.Lock() for p in buses}
state = {}          # sid -> dict(port, ticks, load, volt, temp, torque, t)
state_lock = threading.Lock()
status = {p: 'ok' for p in buses}
hz = {'value': 0.0}


def load_mag(r): return ((r[0] | r[1] << 8) & 0x3ff) if r else None


def reconnect(port):
    """USB adapters drop out when the 12 V sags or a servo is hot-plugged: reopen until it is back."""
    with locks[port]:
        try: buses[port].s.close()
        except Exception: pass
        with state_lock:
            for sid in [k for k, v in state.items() if v.get('port') == port[-5:]]: state.pop(sid)
        status[port] = 'weg'
        while True:
            time.sleep(1)
            try: buses[port] = Bus(port); break
            except Exception: continue
        status[port] = 'ok'
    print(f'{port}: opnieuw verbonden', flush=True)


def poller(port):
    while True:
        try: poll(port)
        except Exception as e:                   # serial.SerialException, OSError, termios.error
            print(f'{port}: verbinding weg ({e.__class__.__name__}), opnieuw verbinden...', flush=True)
            reconnect(port)


def poll(port):
    lk = locks[port]
    present = set()
    with lk:
        b = buses[port]
        for sid in SID2JOINT:
            if b.ping(sid): present.add(sid)
    missing = [s for s in SID2JOINT if s not in present]
    k = 0; t_last = time.time()
    while True:
        with lk:
            b = buses[port]
            for sid in list(present):
                p = b.read(sid, ADDR_POS, 2)
                if p is None:
                    if not b.ping(sid):
                        present.discard(sid); missing.append(sid)
                        with state_lock: state.pop(sid, None)
                    continue
                v = p[0] | p[1] << 8
                v = -(v & 0x7fff) if v & 0x8000 else v
                with state_lock:
                    s = state.setdefault(sid, {'port': port[-5:]})
                    s['ticks'] = v; s['t'] = time.time()
                if k % 10 == 0:
                    ld = b.read(sid, ADDR_LOAD, 2); vt = b.read(sid, ADDR_VOLT, 2); tq = b.read(sid, ADDR_TORQUE, 1)
                    with state_lock:
                        s['load'] = load_mag(ld)
                        if vt: s['volt'], s['temp'] = vt[0]/10, vt[1]
                        if tq: s['torque'] = tq[0]
            if missing and k % 20 == 0:          # re-ping one missing ID now and then
                sid = missing.pop(0)
                if b.ping(sid):
                    present.add(sid)
                else:
                    missing.append(sid)
        k += 1
        if k % 50 == 0:
            now = time.time(); hz['value'] = 50/(now-t_last); t_last = now
        time.sleep(.001)


def port_watcher():
    """Start polling adapters that are plugged in after the server started (e.g. on the Pi at boot)."""
    while True:
        for p in sorted(glob.glob('/dev/cu.usbmodem5A*') + glob.glob('/dev/cu.usbserial*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')):
            if p in buses: continue
            try: buses[p] = Bus(p)
            except Exception: continue
            locks[p] = threading.Lock(); status[p] = 'ok'
            print(f'nieuwe adapter: {p}', flush=True)
            threading.Thread(target=poller, args=(p,), daemon=True).start()
        time.sleep(3)


for p in buses: threading.Thread(target=poller, args=(p,), daemon=True).start()

import auth
import imu as imu_mod
IMU = imu_mod.start()          # MPU6050 on the Pi's I2C bus, None when there is none
if IMU: print(f'IMU: {IMU.model} op 0x{IMU.addr:02x}', flush=True)
threading.Thread(target=port_watcher, daemon=True).start()


model_lock = threading.Lock()


def snapshot():
    with model_lock: return _snapshot()


def _snapshot():
    with state_lock: st = {sid: dict(s) for sid, s in state.items()}
    data.qpos[:] = 0
    joints = {}
    for sid, s in st.items():
        name = SID2JOINT[sid]; c = cal[name]
        ang = c['sign']*(s['ticks']-c['zero'])*TICK
        lo, hi = model.jnt_range[model.joint(name).id]
        data.qpos[QADR[name]] = ang
        joints[name] = dict(s, sid=sid, deg=math.degrees(ang), zero=c['zero'], sign=c['sign'],
                            checked=c.get('checked', False), out=bool(ang < lo-.05 or ang > hi+.05))
    mujoco.mj_fwdPosition(model, data)          # kinematics + collision detection
    hits = set()
    for c in data.contact[:data.ncon]:
        a, b = model.body(model.geom_bodyid[c.geom1]).name, model.body(model.geom_bodyid[c.geom2]).name
        if a[:2] != b[:2] and (a[0] == 'L' or b[0] == 'L') and c.dist < 0:
            hits.add(tuple(sorted((a, b))))
    # a foot pushed below the floor lifts the whole robot, like the real one would
    lowest = min(data.site(f'L{n}_foot').xpos[2] for n in range(1, 7)) + TIP
    lift = max(0.0, GROUND - lowest)
    bodies = [[float(data.xpos[b][0]), float(data.xpos[b][1]), float(data.xpos[b][2] + lift), *map(float, data.xquat[b])]
              for b in range(model.nbody)]
    return dict(bodies=bodies, joints=joints, hz=hz['value'], adapters={p[-5:]: v for p, v in status.items()}, ground=float(GROUND), lift=lift, hits=sorted(hits), t=time.time(),
                imu=IMU.state() if IMU else None)


# ---------------------------------------------------------------- simulation (separate page, lazy)
WEB = Path(__file__).parent/'web'
TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.css': 'text/css', '.svg': 'image/svg+xml', '.png': 'image/png', '.json': 'application/json'}
_sim = {}
_sim_lock = threading.Lock()


def sim_world():
    """Physics world + browser scene, built on the first /sim request (keeps the Pi idle otherwise)."""
    with _sim_lock:
        if not _sim:
            import sim_world as sw
            print('simulatie starten...', flush=True)
            w = sw.SimWorld(mesh_model=model)                 # reuse the twin's meshes (same bodies/frames)
            meta, prims, blob = sw.build_scene(model)
            parts = {c: round(kg, 4) for c, (kg, _) in w.part_mass.items()}
            _sim.update(world=w, meta=dict(groups=meta, prims=prims, parts=parts, joints=list(sw.JOINTS), sid=sw.SID,
                                           supplies={k: v['name'] for k, v in sw.SUPPLIES.items()}, mass=w.total_mass()),
                        bin=blob, bin_gz=gzip.compress(blob, 5))
            print(f'simulatie klaar ({len(blob)/1e6:.1f} MB scene, {len(_sim["bin_gz"])/1e6:.1f} MB gzip)', flush=True)
        return _sim


def net_info():
    """Addresses other devices on the wifi can use to open the page (for the share/QR dialog)."""
    import socket
    host = os.environ.get('ARACHNE_HOST', '127.0.0.1'); name = socket.gethostname().split('.')[0]; ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(('10.255.255.255', 1)); ips.append(s.getsockname()[0]); s.close()
    except OSError: pass
    urls = [f'http://{ip}:{PORT}/sim' for ip in ips] + [f'http://{name}.local:{PORT}/sim']
    return dict(shared=host == '0.0.0.0', host=name, urls=urls)


# ---------------------------------------------------------------- camera (optional, for the cockpit)
class Camera:
    """One capture process per camera, shared by every viewer: MJPEG on stdout, split into JPEG frames.
    Camera 0: ARACHNE_CAMERA (a shell command writing MJPEG to stdout), else the Pi camera via rpicam-vid.
    Camera 1, 2…: ARACHNE_CAMERA_1…, else USB webcams (/dev/v4l/by-id/*usb*) via ffmpeg.
    A process only runs while somebody watches (stops 10 s after the last viewer)."""
    def __init__(self, idx=0):
        self.idx = idx
        self.cv = threading.Condition(); self.frame = None; self.n = 0; self.proc = None; self.viewers = 0; self.idle = time.time()

    @staticmethod
    def sources():
        """Shell commands for every camera that is configured or plugged in, in order."""
        out = []
        csi = os.environ.get('ARACHNE_CAMERA') or ('rpicam-vid -t 0 -n --codec mjpeg --width 1280 --height 720 --framerate 20 -o -'
                                                 if shutil.which('rpicam-vid') and _pi_camera_present() else None)
        if csi: out.append(csi)
        k = 1
        while os.environ.get(f'ARACHNE_CAMERA_{k}'): out.append(os.environ[f'ARACHNE_CAMERA_{k}']); k += 1
        if shutil.which('ffmpeg'):
            for dev in sorted(glob.glob('/dev/v4l/by-id/*usb*-video-index0')):
                out.append(f'ffmpeg -loglevel quiet -f v4l2 -input_format mjpeg -video_size 1280x720 -framerate 20 -i {dev} -c:v copy -f mjpeg -')
        return out

    def command(self):
        src = self.sources()
        return src[self.idx] if self.idx < len(src) else None

    def ensure(self):
        with self.cv:
            if self.proc and self.proc.poll() is None: return True
            cmd = self.command()
            if not cmd: return False
            self.frame = None
            self.proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
            threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()
            if not getattr(self, '_reaper', None):
                self._reaper = threading.Thread(target=self._reap, daemon=True); self._reaper.start()
            return True

    def _read(self, p):
        buf = b''
        while True:
            chunk = p.stdout.read(65536)
            if not chunk: break
            buf += chunk
            while True:
                a = buf.find(b'\xff\xd8')
                if a < 0: buf = buf[-1:]; break
                b = buf.find(b'\xff\xd9', a + 2)
                if b < 0: buf = buf[a:]; break
                with self.cv: self.frame = buf[a:b + 2]; self.n += 1; self.cv.notify_all()
                buf = buf[b + 2:]
        with self.cv: self.cv.notify_all()

    def _reap(self):
        while True:
            time.sleep(2)
            with self.cv:
                if self.viewers == 0 and self.proc and self.proc.poll() is None and time.time() - self.idle > 10:
                    self.proc.terminate(); self.proc = None

    def wait(self, n, timeout=5.):
        """Next frame after frame number n, or None."""
        with self.cv:
            end = time.time() + timeout
            while self.n <= n:
                left = end - time.time()
                if left <= 0 or not (self.proc and self.proc.poll() is None): return None, n
                self.cv.wait(left)
            return self.frame, self.n


_picam = {'t': 0, 'v': False}
def _pi_camera_present():
    """Is a CSI camera connected? (cached for a minute: rpicam-hello takes a moment)"""
    if time.time() - _picam['t'] > 60:
        try: _picam['v'] = 'Available cameras' in subprocess.run(['rpicam-hello', '--list-cameras'], capture_output=True, text=True, timeout=8).stdout
        except Exception: _picam['v'] = False
        _picam['t'] = time.time()
    return _picam['v']


threading.Thread(target=_pi_camera_present, daemon=True).start()     # warm the cache, the app scan has a short timeout
cameras = {}
def camera_for(idx):
    if idx not in cameras: cameras[idx] = Camera(idx)
    return cameras[idx]


# ---------------------------------------------------------------- http
PAGE = (Path(__file__).parent/'live_twin.html')



# ---------------------------------------------------------------- hub: the home server passes commands on to the robot
# The Pi (the machine running arachne-pad on 127.0.0.1:5871) announces itself every 20 s to the home server
# (ARACHNE_HUB, default https://spin.vanrol.eu) with its wifi address and quick-tunnel URL, signed with the shared
# hub key (tools/auth.py hubkey). The server then forwards /robot/* to the Pi: everyone sees the status, but only a
# logged-in user on the same wifi as the robot may move it.
HUB = os.environ.get('ARACHNE_HUB', 'https://spin.vanrol.eu').rstrip('/')
robots = {}                       # on the server: host -> last announce
_pad = [0, False]                 # cache: [time, arachne-pad answers]


def pad_up():
    if time.time() - _pad[0] > 2:
        try: socket.create_connection(('127.0.0.1', 5871), .2).close(); _pad[1] = True
        except OSError: _pad[1] = False
        _pad[0] = time.time()
    return _pad[1]


def _my_ip(v6=False):
    try:
        with socket.socket(socket.AF_INET6 if v6 else socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('2001:4860:4860::8888' if v6 else '8.8.8.8', 53)); return s.getsockname()[0]
    except OSError: return None


def _tunnel_url():
    if os.environ.get('ARACHNE_TUNNEL_URL'): return os.environ['ARACHNE_TUNNEL_URL']
    try:
        log = subprocess.run(['journalctl', '-u', 'arachne-tunnel', '-n', '300', '--no-pager', '-o', 'cat'],
                             capture_output=True, text=True, timeout=5).stdout
        urls = re.findall(r'https://[a-z0-9-]+\.trycloudflare\.com', log)
        return urls[-1] if urls else None
    except (OSError, subprocess.SubprocessError): return None


def announcer():
    """On the robot: tell the home server where we are, as long as the robot control runs and a hub key exists."""
    while True:
        key = auth.hub_key()
        if key and pad_up():
            body = {'host': socket.gethostname(), 'lan': _my_ip(), 'v6': _my_ip(True), 'url': _tunnel_url()}
            try:
                req = urllib.request.Request(HUB + '/hub/announce', json.dumps(body).encode(), method='POST',
                                             headers={'X-Arachne-Hub': key, 'Content-Type': 'application/json', 'User-Agent': 'arachne-robot'})
                urllib.request.urlopen(req, timeout=10).read()
            except OSError: pass
        time.sleep(20)


def hub_robot():
    """On the server: the robot that announced itself most recently (within the last minute), or None."""
    fresh = [r for r in robots.values() if time.time() - r['t'] < 60]
    return max(fresh, key=lambda r: r['t']) if fresh else None


def _lan_reachable(r):
    """Can this server reach the robot's wifi address directly (both at home)? Cached 30 s."""
    if time.time() - r.get('lan_t', 0) > 30:
        try: socket.create_connection((r['lan'], PORT), .4).close(); r['lan_ok'] = True
        except (OSError, TypeError): r['lan_ok'] = False
        r['lan_t'] = time.time()
    return r['lan_ok']


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def send(self, body, ctype='application/json', code=200):
        if isinstance(body, (dict, list)): body = json.dumps(body).encode()
        if isinstance(body, str): body = body.encode()
        self.send_response(code); self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body)

    def page(self, path):
        """Serve an HTML page with the login gate: who is logged in, and a redirect to /login for new visitors."""
        html = path.read_text()
        if auth.has_accounts():
            user = auth.cookie_user(self.headers)
            gate = ('<script>window.ARACHNE_USER=' + json.dumps(user) + ';(function(){'
                    'addEventListener("DOMContentLoaded",function(){var w=document.getElementById("who");if(!w)return;'
                    'if(window.ARACHNE_USER){w.textContent=window.ARACHNE_USER+" · uitloggen";w.href="/logout";}'
                    'else{w.textContent="gast · inloggen";w.href="/login?next="+encodeURIComponent(location.pathname+location.search);}w.hidden=false;});'
                    'if(window.ARACHNE_USER||window.top!==window||/[?&]embed/.test(location.search))return;'
                    'try{if(localStorage.getItem("arachneGuest"))return}catch(e){}'
                    'location.replace("/login?next="+encodeURIComponent(location.pathname+location.search));})();</script>')
            html = html.replace('<head>', '<head>\n' + gate, 1)
        return self.send(html, 'text/html; charset=utf-8')

    ROBOT_NAMES = {'stand': 'opstaan', 'lie': 'liggen', 'dance': 'dansen', 'stop': 'stop'}

    def robot(self, method, path, d=None):
        """The real robot's actions live in tools/gamepad_robot.py (service arachne-pad, port 5871): pass through.
        Accepts the simulation's names too, so the cockpit can drive the real robot with the same calls."""
        path = path.split('?')[0]; path = {'/info': '/status', '/set': '/level'}.get(path, path)
        if method == 'POST' and path == '/do' and d: d = dict(d, what=self.ROBOT_NAMES.get(d.get('what'), d.get('what')))
        if method == 'POST' and path == '/level' and d and 'level' not in d and 'step' not in d: return self.send({'ok': True})
        if not pad_up() and hub_robot() and not auth.hub_ok(self.headers): return self.robot_via_hub(method, path, d)   # never pass on twice
        try:
            req = urllib.request.Request('http://127.0.0.1:5871' + path + (('?' + self.path.split('?', 1)[1]) if '?' in self.path else ''),
                                         json.dumps(d or {}).encode() if method == 'POST' else None, method=method)
            with urllib.request.urlopen(req, timeout=25) as r: return self.send(r.read(), 'application/json')
        except urllib.error.HTTPError as e: return self.send(e.read(), 'application/json', code=e.code)
        except OSError:
            if auth.hub_key(): return self.send({'error': 'de spin is niet verbonden (staat de Pi aan en heeft hij internet?)'}, code=503)
            return self.send({'error': 'de besturing (arachne-pad) draait niet'}, code=503)

    def robot_via_hub(self, method, path, d):
        """On the server: pass the call on to the announced robot, directly over the wifi when we can, else its tunnel."""
        r = hub_robot(); q = ('?' + self.path.split('?', 1)[1]) if '?' in self.path else ''
        bases = ([f"http://{r['lan']}:{PORT}"] if _lan_reachable(r) else []) + ([r['url']] if r.get('url') else [])
        for base in bases:
            try:
                req = urllib.request.Request(base + '/robot' + path + q, json.dumps(d or {}).encode() if method == 'POST' else None,
                                             method=method, headers={'X-Arachne-Hub': auth.hub_key() or '', 'User-Agent': 'arachne-hub'})
                with urllib.request.urlopen(req, timeout=25) as res: body = res.read()
            except urllib.error.HTTPError as e: return self.send(e.read(), 'application/json', code=e.code)
            except OSError: continue
            if path == '/status':
                try: body = dict(json.loads(body), via=r['host'], same_wifi=self.same_wifi(r))
                except ValueError: pass
            return self.send(body, 'application/json')
        return self.send({'error': f"{r['host']} heeft zich aangemeld maar antwoordt niet"}, code=503)

    def client_ip(self): return self.headers.get('Cf-Connecting-Ip') or self.client_address[0]

    def same_wifi(self, r):
        """Is this visitor on the same network as the robot? Same public address, same /24 at home, or same IPv6 /64."""
        try:
            ip = ipaddress.ip_address(self.client_ip().removeprefix('::ffff:'))
            if str(ip) == r.get('pub'): return True
            for other, bits in ((r.get('lan'), 24), (r.get('v6'), 64)):
                if other and ip.is_private == (bits == 24) and ip in ipaddress.ip_network(f'{other}/{bits}', strict=False): return True
        except ValueError: pass
        return False

    def may_drive(self):
        """(allowed, reason) for moving the real robot from this request."""
        if not self.robot_allowed(): return False, 'log in om de echte robot te bewegen'
        if auth.hub_ok(self.headers) or pad_up(): return True, ''
        r = hub_robot()
        if r and not self.same_wifi(r): return False, f"je zit niet op dezelfde wifi als {r['host']}: alleen kijken"
        return True, ''

    def robot_allowed(self):
        """May this request move the real robot? Logged in, or a script on the robot itself, or the home server."""
        if auth.hub_ok(self.headers): return True
        remote = bool(self.headers.get('Cf-Connecting-Ip') or self.headers.get('Cf-Ray'))
        local = self.client_address[0] in ('127.0.0.1', '::1') and not remote
        if local or auth.cookie_user(self.headers): return True
        return not auth.has_accounts() and not remote       # no accounts set up yet: old behaviour on the wifi

    def do_GET(self):
        if self.path.startswith('/login'): return self.send((WEB/'login.html').read_text(), 'text/html; charset=utf-8')
        if self.path.startswith('/logout'):
            self.send_response(302); self.send_header('Set-Cookie', f'{auth.COOKIE}=; Path=/; Max-Age=0; HttpOnly; SameSite=Lax')
            self.send_header('Location', '/login'); self.end_headers(); return
        if self.path.startswith('/robot/'): return self.robot('GET', self.path[6:])
        if self.path == '/me': return self.send({'user': auth.cookie_user(self.headers), 'robot': self.may_drive()[0], 'accounts': auth.has_accounts()})
        if self.path == '/' or self.path.startswith('/?'): return self.page(PAGE)
        if self.path.startswith('/web/'):
            f = (WEB/self.path[5:].split('?')[0]).resolve()
            if WEB.resolve() in f.parents and f.is_file():
                self.send_response(200); self.send_header('Content-Type', TYPES.get(f.suffix, 'application/octet-stream'))
                self.send_header('Cache-Control', 'max-age=3600' if '/vendor/' in self.path else 'no-cache')
                b = f.read_bytes(); self.send_header('Content-Length', str(len(b))); self.end_headers(); return self.wfile.write(b)
            return self.send({'error': 'not found'}, code=404)
        if self.path == '/sim' or self.path.startswith('/sim?') or self.path == '/sim/':
            return self.page(WEB/'sim.html')
        if self.path.startswith('/sim/'): return self.sim_get(self.path[4:].split('?')[0])
        if self.path in ('/cockpit', '/cockpit/') or self.path.startswith('/cockpit?'):
            return self.page(WEB/'cockpit.html')
        if self.path.split('?')[0] in ('/camera.mjpg', '/camera.jpg'):
            q = self.path.split('?')[1] if '?' in self.path else ''
            idx = next((int(v) for k, v in (kv.split('=', 1) for kv in q.split('&') if '=' in kv) if k == 'cam' and v.isdigit()), 0)
            return self.camera(self.path.startswith('/camera.jpg'), idx)
        if self.path == '/cameras': return self.send({'count': len(Camera.sources())})
        if self.path == '/app/info':     # the phone app scans the wifi for this (readable cross-origin, nothing secret)
            body = json.dumps(dict(arachne=1, **net_info(), adapters={p[-5:]: v for p, v in status.items()},
                                   imu=IMU.model if IMU else None, cameras=len(Camera.sources()))).encode()
            self.send_response(200); self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*'); self.send_header('Content-Length', str(len(body)))
            self.end_headers(); return self.wfile.write(body)
        if self.path == '/scene.json': return self.send(dict(groups=SCENE_META, joints=[j['name'] for j in JOINTS],
                                                             sid={j['name']: j['servo_id'] for j in JOINTS}))
        if self.path == '/scene.bin': return self.send(SCENE_BIN, 'application/octet-stream')
        if self.path == '/state': return self.send(snapshot())
        if self.path == '/stream':
            self.send_response(200); self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache'); self.end_headers()
            try:
                while True:
                    self.wfile.write(b'data: '+json.dumps(snapshot()).encode()+b'\n\n'); self.wfile.flush()
                    time.sleep(1/60)
            except (BrokenPipeError, ConnectionResetError): return
        self.send({'error': 'not found'}, code=404)

    def camera(self, single, idx=0):
        camera = camera_for(idx)
        if not camera.ensure(): return self.send({'error': 'geen camera'}, code=404)
        frame, n = camera.wait(camera.n if camera.frame is None else camera.n - 1)   # latest frame, or wait for the first
        if frame is None: return self.send({'error': 'camera geeft geen beeld'}, code=503)
        if single: return self.send(frame, 'image/jpeg')
        self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache'); self.end_headers()
        with camera.cv: camera.viewers += 1
        try:
            while frame is not None:
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                frame, n = camera.wait(n)
        except (BrokenPipeError, ConnectionResetError): pass
        finally:
            with camera.cv: camera.viewers -= 1; camera.idle = time.time()

    def sim_get(self, path):
        S = sim_world(); w = S['world']
        if path == '/scene.json': return self.send(S['meta'])
        if path == '/scene.bin':
            gz = 'gzip' in self.headers.get('Accept-Encoding', ''); body = S['bin_gz'] if gz else S['bin']
            self.send_response(200); self.send_header('Content-Type', 'application/octet-stream')
            if gz: self.send_header('Content-Encoding', 'gzip')
            self.send_header('Content-Length', str(len(body))); self.end_headers(); return self.wfile.write(body)
        if path == '/state': w.last_seen = time.time(); return self.send(w.snapshot_json, 'application/json')
        if path == '/info': return self.send(net_info())
        if path == '/stream':
            self.send_response(200); self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache'); self.end_headers()
            w.viewers += 1
            try:
                while True:
                    w.last_seen = time.time()
                    self.wfile.write(b'data: ' + w.snapshot_json + b'\n\n'); self.wfile.flush()
                    time.sleep(1/30)
            except (BrokenPipeError, ConnectionResetError, OSError): return
            finally: w.viewers -= 1
        return self.send({'error': 'not found'}, code=404)

    def sim_post(self, path, d):
        w = sim_world()['world']; w.last_seen = time.time()
        if path == '/cmd': return self.send(w.cmd(d))
        if path == '/do': return self.send(w.do(d.get('what', 'stop')))
        if path == '/drive': w.drive(float(d.get('vx', 0)), float(d.get('vy', 0)), float(d.get('wz', 0))); return self.send({'ok': True})
        if path == '/set': return self.send(w.set(d))
        if path == '/push': w.push(d.get('fx'), d.get('fy')); return self.send({'ok': True})
        if path == '/reset': w.reset(d.get('pose', 'lie')); return self.send({'ok': True})
        return self.send({'error': 'unknown'}, code=400)

    def do_POST(self):
        d = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
        if self.path.startswith('/sim/'):
            try: return self.sim_post(self.path[4:], d)
            except Exception as e: return self.send({'error': f'{e.__class__.__name__}: {e}'}, code=500)
        if self.path == '/login':
            ip = self.headers.get('Cf-Connecting-Ip') or self.client_address[0]
            if auth.too_many(ip): return self.send({'error': 'te veel foute pogingen, wacht 5 minuten'}, code=429)
            user = str(d.get('user', ''))
            if not auth.check_pw(user, str(d.get('password', ''))):
                auth.failed(ip); return self.send({'error': 'verkeerde gebruiker of wachtwoord'}, code=401)
            body = json.dumps({'ok': True, 'user': user}).encode()
            self.send_response(200); self.send_header('Content-Type', 'application/json')
            self.send_header('Set-Cookie', f'{auth.COOKIE}={auth.make_token(user)}; Path=/; Max-Age={auth.DAYS*86400}; HttpOnly; SameSite=Lax')
            self.send_header('Content-Length', str(len(body))); self.end_headers(); return self.wfile.write(body)
        if self.path == '/hub/announce':
            if not auth.hub_ok(self.headers): return self.send({'error': 'verkeerde hub-sleutel'}, code=403)
            old = robots.get(d.get('host'), {})
            robots[str(d.get('host'))] = dict(old, **{k: d.get(k) for k in ('host', 'lan', 'v6', 'url')}, pub=self.client_ip(), t=time.time())
            if old.get('lan') != d.get('lan'): robots[str(d.get('host'))].pop('lan_t', None)
            return self.send({'ok': True})
        if self.path.startswith('/robot/'):
            ok, why = self.may_drive()
            if not ok: return self.send({'error': why}, code=403)
            return self.robot('POST', self.path[6:], d)
        # Guests may watch the real robot and play with the simulation, but never move the real servos or change the calibration.
        if (self.path == '/cal' or (self.path == '/cmd' and d.get('op') not in ('read', 'ping'))) and not self.robot_allowed():
            return self.send({'error': 'log in om de echte robot te bewegen'}, code=403)
        if self.path == '/cal':
            # {"joint": "L5_knee", "sign": -1} or {"joint": ..., "zero_here": true}
            name = d['joint']; c = cal[name]
            if 'sign' in d: c['sign'] = int(d['sign'])
            if 'zero' in d: c['zero'] = int(d['zero'])
            if d.get('zero_here'):
                sid = next(j['servo_id'] for j in JOINTS if j['name'] == name)
                with state_lock: c['zero'] = state[sid]['ticks']
            if 'checked' in d: c['checked'] = bool(d['checked'])
            save_cal(); return self.send(c)
        if self.path == '/cmd':
            # {"port": "13851", "op": "read"/"write", "id": 13, "addr": 56, "n": 2 | "data": [..]}
            port = next(p for p in buses if p.endswith(d['port']))
            if status[port] != 'ok': return self.send({'error': 'adapter niet verbonden'}, code=503)
            try:
              with locks[port]:
                b = buses[port]
                if d['op'] == 'read': r = b.read(d['id'], d['addr'], d['n']); return self.send({'data': list(r) if r else None})
                if d['op'] == 'write': b.write(d['id'], d['addr'], d['data']); return self.send({'ok': True})
                if d['op'] == 'ping': return self.send({'ok': b.ping(d['id'])})
                if d['op'] == 'sync': b.sync_write(d['addr'], {int(k): v for k, v in d['data'].items()}); return self.send({'ok': True})
            except Exception as e:
                return self.send({'error': f'bus fout: {e.__class__.__name__}'}, code=503)
        self.send({'error': 'unknown'}, code=400)


if __name__ == '__main__':
    print(f'ARACHNE live twin: http://localhost:{PORT}  (adapters: {", ".join(buses) or "geen"}) · simulatie: http://localhost:{PORT}/sim', flush=True)
    host = os.environ.get('ARACHNE_HOST', '127.0.0.1')     # 0.0.0.0 on the Pi so other computers can watch
    threading.Thread(target=announcer, daemon=True).start()
    ThreadingHTTPServer((host, PORT), H).serve_forever()
