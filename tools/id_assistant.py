"""Give all 18 ARACHNE servos their ID (1..18, as in simulation/joint_map.json).

    .venv/bin/python tools/id_assistant.py      -> opens http://localhost:5860

Plug in ONE new servo at a time (new STS3215s all ship as ID 1), press Scan,
press Wiggle to see which one it is, then pick its joint: the ID is written
immediately. Works on every Waveshare adapter connected over USB.
"""
import glob, json, sys, threading, time, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from sts import Bus, ADDR_ID, ADDR_LOCK

ROOT = Path(__file__).resolve().parents[1]
JOINTS = [(j['name'], j['servo_id']) for j in json.loads((ROOT/'simulation/joint_map.json').read_text())['joints']]
# Leg numbers = the tape numbers on the robot (same order as L1..L6 in the model).
LEG_NAMES = {f'L{n}': f'poot {n}' for n in range(1, 7)}
SKIP = {'L2', 'L5'}   # middle legs not used for now
# skipped legs stay in the list (their servos still need a unique ID), only greyed out
PART = {'yaw': 'lijf (yaw)', 'hip': 'dij (hip)', 'knee': 'knie (knee)'}
SCAN_IDS = range(0, 41)
PORT = 5860

buses = {p: Bus(p) for p in sorted(glob.glob('/dev/cu.usbmodem5A*') + glob.glob('/dev/cu.usbserial*'))}
locks = {p: threading.Lock() for p in buses}
if not buses: raise SystemExit('Geen servo-adapter gevonden op USB')


def scan():
    out = {}
    for port, b in buses.items():
        with locks[port]:
            servos = []
            for i in SCAN_IDS:
                if not b.ping(i): continue
                reads = sum(b.read(i, 56, 2) is not None for _ in range(5))
                st = b.status(i)
                servos.append(dict(id=i, pos=st['pos'], volt=st['volt'], temp=st['temp'],
                                   collision=reads < 4))  # ping OK but garbled reads = two servos share this ID
        out[port] = servos
    return out


def wiggle(port, sid):
    b = buses[port]
    with locks[port]:
        p = b.position(sid)
        if p is None: return 'kan positie niet lezen (botsing?)'
        b.torque(sid, 1)
        for d in (120, -120, 120, 0):  # about +-10 degrees
            b.move(sid, p+d, speed=1500, acc=60); time.sleep(.35)
        b.torque(sid, 0)  # slack again, so cables can be swapped safely
    return 'ok'


def set_id(port, old, new):
    b = buses[port]
    with locks[port]:
        if old == new: return 'ok'
        others = [p for p in buses if p != port]
        if b.ping(new): return f'ID {new} bestaat al op deze adapter: geef die servo eerst een ander gewricht'
        for p in others:
            with locks[p]:
                if buses[p].ping(new): return f'ID {new} bestaat al op de andere adapter'
        b.write(old, ADDR_LOCK, [0]); b.write(old, ADDR_ID, [new]); time.sleep(.05)
        b.write(new, ADDR_LOCK, [1]); time.sleep(.05)
        return 'ok' if b.ping(new) else 'schrijven mislukt'


PAGE = """<!doctype html><html lang="nl"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>ARACHNE servo-ID's</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;background:#1d2224;color:#e8ecea;max-width:900px;margin:24px auto;padding:0 16px}
h1{font-size:22px}h2{font-size:16px;margin:22px 0 8px;color:#9fb}
.grid{display:grid;grid-template-columns:repeat(6,1fr);gap:6px}
.leg{background:#2a3134;border-radius:10px;padding:8px}.leg b{display:block;margin-bottom:4px}
.j{font-size:13px;padding:3px 6px;border-radius:6px;margin:2px 0;background:#3a4246}.j.ok{background:#2f7a4d}
table{width:100%;border-collapse:collapse}td,th{padding:7px;border-bottom:1px solid #333;text-align:left}
button{font-size:15px;padding:8px 14px;border-radius:9px;border:0;cursor:pointer;background:#4a7dbd;color:#fff}
.big{font-size:18px;padding:12px 22px;background:#3fa66b}select{font-size:14px;padding:6px}
.warn{color:#ffb35c}.small{color:#9aa;font-size:13px}#msg{min-height:22px;color:#ffd166}
</style>
<h1>ARACHNE &ndash; servo-ID's instellen</h1>
<p class=small>Steek <b>&eacute;&eacute;n nieuwe servo</b> per keer in (nieuwe servo's hebben allemaal ID 1) en druk op Scan.
Wiebel om te zien welke het is, kies dan zijn gewricht. Poten 2 en 5 slaan we over. Doe <b>poot 1 lijf (ID 1)</b> als allerlaatste.</p>
<p><button class=big onclick=scan()>Scan</button> <span id=msg></span></p>
<h2>Gewrichten</h2><div class=grid id=legs></div>
<h2>Gevonden servo's</h2><div id=buses></div>
<script>
const JOINTS=__JOINTS__, LEGS=__LEGS__, PART=__PART__, SKIP=__SKIP__;
let state={};
function label(n){const [leg,part]=n.split('_');return `${LEGS[leg]} &ndash; ${PART[part]}`}
function render(){
  const used=new Set(); for(const p in state) for(const s of state[p]) if(!s.collision) used.add(s.id);
  let h=''; for(const leg of Object.keys(LEGS)){h+=`<div class=leg style="${SKIP.includes(leg)?'opacity:.35':''}"><b>${LEGS[leg]}</b><span class=small>${SKIP.includes(leg)?'overgeslagen':''}</span>`;
    for(const [n,id] of JOINTS.filter(j=>j[0].startsWith(leg))) h+=`<div class="j ${used.has(id)?'ok':''}">${PART[n.split('_')[1]].split(' ')[0]} &middot; ID ${id}</div>`; h+='</div>'}
  document.getElementById('legs').innerHTML=h;
  let b=''; for(const p in state){b+=`<h3 class=small>Adapter ${p}</h3><table><tr><th>ID</th><th>Positie</th><th>Spanning</th><th></th><th>Is gewricht&hellip;</th></tr>`;
    if(!state[p].length) b+='<tr><td colspan=5 class=small>geen servo&rsquo;s</td></tr>';
    for(const s of state[p]){const cur=JOINTS.find(j=>j[1]==s.id);
      b+=`<tr><td>${s.id}${s.collision?' <span class=warn>&#9888; meerdere servo&rsquo;s met dit ID &ndash; trek er losse uit</span>':''}</td><td>${s.pos??'&ndash;'}</td><td>${s.volt??'&ndash;'} V</td>
      <td><button onclick="wig('${p}',${s.id})" ${s.collision?'disabled':''}>Wiebel</button></td>
      <td><select onchange="assign('${p}',${s.id},this.value)" ${s.collision?'disabled':''}><option value="">${cur?label(cur[0]):'kies&hellip;'}</option>
      ${JOINTS.map(([n,id])=>`<option value=${id}>${label(n)} (ID ${id})</option>`).join('')}</select></td></tr>`}
    b+='</table>'}
  document.getElementById('buses').innerHTML=b;
}
async function api(u,body){const r=await fetch(u,{method:body?'POST':'GET',body:body&&JSON.stringify(body)});return r.json()}
async function scan(){msg('scannen&hellip;');state=await api('/scan');render();msg('')}
async function wig(p,id){msg('wiebelen&hellip;');const r=await api('/wiggle',{port:p,id});msg(r.msg=='ok'?'':r.msg)}
async function assign(p,id,n){if(!n)return;msg('ID schrijven&hellip;');const r=await api('/assign',{port:p,old:id,new:+n});msg(r.msg=='ok'?`ID ${n} ingesteld`:r.msg);await scan()}
function msg(t){document.getElementById('msg').innerHTML=t}
scan();
</script></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def send(self, obj, ctype='application/json'):
        b = (obj if isinstance(obj, str) else json.dumps(obj)).encode()
        self.send_response(200); self.send_header('Content-Type', ctype); self.send_header('Content-Length', len(b)); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        if self.path == '/scan': return self.send(scan())
        self.send(PAGE.replace('__JOINTS__', json.dumps(JOINTS)).replace('__LEGS__', json.dumps(LEG_NAMES))
                  .replace('__PART__', json.dumps(PART)).replace('__SKIP__', json.dumps(sorted(SKIP))), 'text/html; charset=utf-8')

    def do_POST(self):
        d = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        if self.path == '/wiggle': return self.send({'msg': wiggle(d['port'], d['id'])})
        if self.path == '/assign': return self.send({'msg': set_id(d['port'], d['old'], d['new'])})
        self.send({'msg': 'onbekend'})


if __name__ == '__main__':
    print('Adapters:', ', '.join(buses))
    print(f'Open http://localhost:{PORT}  (Ctrl+C om te stoppen)')
    threading.Timer(.5, lambda: webbrowser.open(f'http://localhost:{PORT}')).start()
    ThreadingHTTPServer(('127.0.0.1', PORT), H).serve_forever()
