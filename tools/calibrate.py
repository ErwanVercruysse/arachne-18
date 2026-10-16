"""Measure every joint's end stops by hand.

    .venv/bin/python tools/calibrate.py      -> opens http://localhost:5861

All servos go slack. Start with the legs folded inward (that pose is stored as
"folded"), then move each joint by hand to both mechanical limits. The page
tracks the extremes live (unwrapped across the 4095 -> 0 seam) and "Opslaan"
writes calibration/joint_limits.json.
"""
import glob, json, sys, threading, time, webbrowser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from sts import Bus

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'calibration'/'joint_limits.json'
NAMES = {j['servo_id']: j['name'] for j in json.loads((ROOT/'simulation/joint_map.json').read_text())['joints']}
SCAN_IDS = range(0, 41)
PORT = 5861

buses = {p: Bus(p) for p in sorted(glob.glob('/dev/cu.usbmodem5A*') + glob.glob('/dev/cu.usbserial*'))}
lock = threading.Lock()
joints = {}   # key "port#id" -> state


def key(port, sid): return f'{port}#{sid}'


def discover():
    with lock:
        for port, b in buses.items():
            for sid in SCAN_IDS:
                if not b.ping(sid) or key(port, sid) in joints: continue
                p = b.position(sid)
                if p is None or sum(b.read(sid, 56, 2) is not None for _ in range(5)) < 4:
                    continue  # shared ID: cannot be calibrated until it has its own ID
                b.torque(sid, 0)
                joints[key(port, sid)] = dict(port=port, id=sid, name=NAMES.get(sid, f'ID {sid}'), raw=p,
                                              start=p, unwrapped=0, min=0, max=0, last=p, ok=True)


def poll():
    while True:
        with lock:
            for j in joints.values():
                p = buses[j['port']].position(j['id'])
                if p is None: j['ok'] = False; continue
                j['ok'] = True
                d = (p-j['last']+2048) % 4096-2048   # shortest step, so crossing 4095->0 stays continuous
                j['unwrapped'] += d; j['last'] = p; j['raw'] = p
                j['min'] = min(j['min'], j['unwrapped']); j['max'] = max(j['max'], j['unwrapped'])
        time.sleep(.03)


def deg(t): return round(t*360/4096, 1)


def summary():
    rows = []
    for k, j in sorted(joints.items(), key=lambda kv: (kv[1]['id'], kv[0])):
        rows.append(dict(key=k, port=j['port'][-5:], id=j['id'], name=j['name'], raw=j['raw'], ok=j['ok'],
                         now_deg=deg(j['unwrapped']), min_deg=deg(j['min']), max_deg=deg(j['max']),
                         range_deg=deg(j['max']-j['min'])))
    return rows


def save():
    data = dict(created=datetime.now().isoformat(timespec='seconds'),
                note='Measured by hand. *_raw are servo ticks (0-4095, may wrap past 0). folded_raw = legs folded inward at start.',
                joints={})
    for j in joints.values():
        data['joints'][j['name']] = dict(servo_id=j['id'], port=j['port'], folded_raw=j['start'],
                                         min_raw=(j['start']+j['min']) % 4096, max_raw=(j['start']+j['max']) % 4096,
                                         min_from_folded_deg=deg(j['min']), max_from_folded_deg=deg(j['max']),
                                         range_deg=deg(j['max']-j['min']))
    OUT.parent.mkdir(exist_ok=True); OUT.write_text(json.dumps(data, indent=2)+'\n')
    return str(OUT)


PAGE = """<!doctype html><html lang="nl"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>ARACHNE eindstops</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;background:#1d2224;color:#e8ecea;max-width:900px;margin:24px auto;padding:0 16px}
h1{font-size:22px}table{width:100%;border-collapse:collapse}td,th{padding:8px;border-bottom:1px solid #333;text-align:left}
.bar{position:relative;height:14px;background:#333;border-radius:7px;min-width:220px}
.rng{position:absolute;top:0;height:14px;background:#3fa66b;border-radius:7px}.now{position:absolute;top:-3px;width:3px;height:20px;background:#ffd166}
button{font-size:16px;padding:10px 18px;border-radius:10px;border:0;cursor:pointer;background:#3fa66b;color:#fff;margin-right:8px}
.grey{background:#4a5358}.small{color:#9aa;font-size:13px}#msg{color:#ffd166}
</style>
<h1>ARACHNE &ndash; eindstops meten</h1>
<p class=small>Alle servo&rsquo;s zijn slap. Begin met de poten naar binnen gevouwen (= 0&deg;), beweeg elk gewricht met de hand
tot tegen zijn ene aanslag en dan tot tegen de andere. Groen = gemeten bereik, geel = nu.</p>
<p><button onclick="post('/save').then(r=>msg('Opgeslagen: '+r.path))">Opslaan</button>
<button class=grey onclick="post('/rescan').then(()=>msg('Opnieuw gescand'))">Nieuwe servo's zoeken</button> <span id=msg></span></p>
<table id=t></table>
<script>
function post(u){return fetch(u,{method:'POST'}).then(r=>r.json())}
function msg(t){document.getElementById('msg').textContent=t}
async function tick(){const rows=await (await fetch('/state')).json();
  let h='<tr><th>Gewricht</th><th>ID</th><th>Nu</th><th>Min</th><th>Max</th><th>Bereik</th><th style="width:40%"></th></tr>';
  for(const r of rows){const s=v=>50+v/3.6/2; // -180..180 deg -> 0..100 %
    h+=`<tr style="${r.ok?'':'opacity:.4'}"><td>${r.name}</td><td>${r.id} <span class=small>${r.port}</span></td><td>${r.now_deg}&deg;</td><td>${r.min_deg}&deg;</td><td>${r.max_deg}&deg;</td><td><b>${r.range_deg}&deg;</b></td>
    <td><div class=bar><div class=rng style="left:${s(r.min_deg)}%;width:${(r.max_deg-r.min_deg)/3.6}%"></div><div class=now style="left:${s(r.now_deg)}%"></div></div></td></tr>`}
  if(!rows.length) h+='<tr><td colspan=7 class=small>Geen servo&rsquo;s met een eigen ID gevonden.</td></tr>';
  document.getElementById('t').innerHTML=h}
setInterval(tick,150);tick();
</script></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def send(self, obj, ctype='application/json'):
        b = (obj if isinstance(obj, str) else json.dumps(obj)).encode()
        self.send_response(200); self.send_header('Content-Type', ctype); self.send_header('Content-Length', len(b)); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        if self.path == '/state': return self.send(summary())
        self.send(PAGE, 'text/html; charset=utf-8')

    def do_POST(self):
        if self.path == '/save': return self.send({'path': save()})
        if self.path == '/rescan': discover(); return self.send({})
        self.send({})


if __name__ == '__main__':
    discover()
    print('Gevonden:', ', '.join(f"{j['name']} (ID {j['id']}, {j['port'][-5:]})" for j in joints.values()) or 'niets')
    threading.Thread(target=poll, daemon=True).start()
    print(f'Open http://localhost:{PORT}  (Ctrl+C om te stoppen)')
    threading.Timer(.5, lambda: webbrowser.open(f'http://localhost:{PORT}')).start()
    ThreadingHTTPServer(('127.0.0.1', PORT), H).serve_forever()
