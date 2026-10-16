"""Control one ARACHNE leg (3 STS3215 servos) with sliders in the browser.

    .venv/bin/python tools/arm_gui.py        -> opens http://localhost:5859

Sliders start at the servos' current positions and are limited to +-90 deg
around them. "Slap" turns torque off so you can move the leg by hand.
"""
import json, sys, threading, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from sts import Bus

JOINTS = [('Lijf (yaw)', 1), ('Dij (hip)', 2), ('Knie (knee)', 3)]
RANGE = 1024  # +-90 deg in servo ticks (4096 per turn)
PORT = 5859

bus = Bus(); lock = threading.Lock()
with lock:
    present = {sid: bus.position(sid) for _, sid in JOINTS}
missing = [sid for sid, p in present.items() if p is None]
if missing: print('Let op: geen antwoord van servo', missing)
limits = {sid: (max(0, p-RANGE), min(4095, p+RANGE)) for sid, p in present.items() if p is not None}

PAGE = """<!doctype html><html lang="nl"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>ARACHNE arm</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;background:#1d2224;color:#e8ecea;max-width:640px;margin:30px auto;padding:0 16px}
h1{font-size:22px} .j{background:#2a3134;border-radius:12px;padding:14px 18px;margin:14px 0}
.top{display:flex;justify-content:space-between} input[type=range]{width:100%;height:34px}
button{font-size:16px;padding:10px 16px;border-radius:10px;border:0;margin-right:8px;cursor:pointer}
.on{background:#3fa66b;color:#fff}.off{background:#c9573f;color:#fff}.small{color:#9aa}
</style>
<h1>ARACHNE &ndash; arm L1</h1>
<p class=small>Beweeg langzaam: de grenzen zijn &plusmn;90&deg; rond de startpositie, niet de mechanische limieten.</p>
<div id=joints></div>
<p><button class=on onclick="torque(1)">Kracht aan</button><button class=off onclick="torque(0)">Slap (met de hand bewegen)</button>
<button onclick="center()">Terug naar start</button></p>
<p class=small id=status></p>
<script>
const J=__JOINTS__;
const box=document.getElementById('joints');
for(const j of J){
  box.insertAdjacentHTML('beforeend',`<div class=j><div class=top><b>${j.name} &middot; ID ${j.id}</b><span id=v${j.id}></span></div>
  <input type=range id=s${j.id} min=${j.min} max=${j.max} value=${j.pos}></div>`);
  const s=document.getElementById('s'+j.id);
  let t=0; s.oninput=()=>{label(j,s.value); clearTimeout(t); t=setTimeout(()=>post('/move',{id:j.id,pos:+s.value}),30)};
  label(j,j.pos);
}
function label(j,v){document.getElementById('v'+j.id).textContent=`${v}  (${((v-j.start)*360/4096).toFixed(0)}°)`}
function post(u,b){return fetch(u,{method:'POST',body:JSON.stringify(b)}).then(r=>r.json())}
function torque(on){post('/torque',{on})}
function center(){for(const j of J){document.getElementById('s'+j.id).value=j.start;label(j,j.start);post('/move',{id:j.id,pos:j.start})}}
setInterval(async()=>{const r=await (await fetch('/state')).json();
  document.getElementById('status').textContent=Object.entries(r).map(([k,v])=>`ID ${k}: pos ${v.pos} · ${v.volt} V · ${v.temp} °C`).join('   |   ');
  for(const j of J){const s=document.getElementById('s'+j.id); if(r[j.id]&&r[j.id].pos!=null&&document.activeElement!==s&&r.slack){s.value=r[j.id].pos;label(j,r[j.id].pos)}}},700);
</script></html>"""

slack = False


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def send(self, body, ctype='application/json'):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(200); self.send_header('Content-Type', ctype); self.send_header('Content-Length', len(b)); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        if self.path == '/state':
            with lock: st = {sid: bus.status(sid) for _, sid in JOINTS}
            st['slack'] = slack
            return self.send(json.dumps(st))
        j = [dict(name=n, id=sid, pos=present[sid], start=present[sid], min=limits[sid][0], max=limits[sid][1]) for n, sid in JOINTS if sid in limits]
        self.send(PAGE.replace('__JOINTS__', json.dumps(j)), 'text/html; charset=utf-8')

    def do_POST(self):
        global slack
        d = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        with lock:
            if self.path == '/move' and d['id'] in limits:
                lo, hi = limits[d['id']]
                if slack: bus.torque(d['id'], 1)
                bus.move(d['id'], min(hi, max(lo, d['pos'])), speed=1200, acc=40); slack = False
            elif self.path == '/torque':
                for _, sid in JOINTS: bus.torque(sid, d['on'])
                slack = not d['on']
        self.send('{}')


if __name__ == '__main__':
    print('Servo-posities bij start:', present)
    print(f'Open http://localhost:{PORT}  (Ctrl+C om te stoppen)')
    threading.Timer(.5, lambda: webbrowser.open(f'http://localhost:{PORT}')).start()
    ThreadingHTTPServer(('127.0.0.1', PORT), H).serve_forever()
