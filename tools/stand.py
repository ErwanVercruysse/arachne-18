"""Stand up or lie down (needs live_twin.py running; works on the Mac and on the Pi).

    .venv/bin/python tools/stand.py              # stand up in 1.5 s
    .venv/bin/python tools/stand.py --time .8    # faster
    .venv/bin/python tools/stand.py --lie        # back onto the belly, legs straight out

Torque is switched on one servo at a time at 30 % and ramped up first: switching all 18 on at
100 % at once makes the 12 V supply sag, the servos reset and the USB adapter drops out.
Stand pose = the calibrated 'straight out' middle + hip -307 / knee -694 ticks (leg 5 hand-placed).
"""
import argparse, json, math, os, time, urllib.request
from pathlib import Path

URL = os.environ.get('ARACHNE_URL', 'http://localhost:5870')   # ARACHNE_URL=.../sim drives the simulation
ROOT = Path(__file__).resolve().parents[1]
T = 2*math.pi/4096
OFF = {'yaw': 0., 'hip': math.radians(15), 'knee': math.radians(-87.56918699361886)}   # straight-out pose in model q
STAND = {'yaw': 0, 'hip': -307, 'knee': -694}


def get(): return json.loads(urllib.request.urlopen(URL+'/state').read())
def post(d): return json.loads(urllib.request.urlopen(urllib.request.Request(URL+'/cmd', json.dumps(d).encode(), method='POST')).read())


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--time', type=float, default=1.5); ap.add_argument('--lie', action='store_true')
    a = ap.parse_args()
    cal = json.loads((ROOT/'calibration/twin_map.json').read_text())
    J = get()['joints']
    if len(J) < 18: raise SystemExit(f'maar {len(J)} servo\'s gevonden, ik verwacht er 18')
    w = lambda j, adr, data: post({'port': J[j]['port'], 'op': 'write', 'id': J[j]['sid'], 'addr': adr, 'data': data})
    mid = {j: round(cal[j]['zero'] - OFF[j.split('_')[1]]/(cal[j]['sign']*T)) for j in J}
    tgt = {j: mid[j] + (0 if a.lie else STAND[j.split('_')[1]]) for j in J}
    off = [j for j in J if not J[j].get('torque')]
    for j in sorted(off, key=lambda j: J[j]['sid']):          # gentle torque-on, hold where it is
        t = J[j]['ticks']; w(j, 48, [0x2c, 1]); w(j, 42, [t & 0xff, t >> 8, 0, 0, 0, 0]); w(j, 40, [1]); time.sleep(.1)
    if off:
        for lim in (500, 750, 1000):
            for j in J: w(j, 48, [lim & 0xff, lim >> 8])
            time.sleep(.15)
        J = get()['joints']
    for j in J:                                                 # one goal each, speeds so all arrive together
        d = abs(tgt[j]-J[j]['ticks']); sp = max(50, min(3400, int(d/a.time*1.15))); t = tgt[j]
        w(j, 41, [80 if a.time < 1 else 40]); w(j, 42, [t & 0xff, t >> 8, 0, 0, sp & 0xff, sp >> 8])
    t0 = time.time(); vmin = 99
    while time.time()-t0 < a.time+1.5:
        s = get()['joints']; vmin = min([vmin]+[(v.get('volt') or 99) for v in s.values()]); time.sleep(.05)
    f = get(); s = f['joints']
    print(f"{'liggen' if a.lie else 'staan'}: max afwijking {max(abs(s[j]['ticks']-tgt[j]) for j in s)} ticks, "
          f"max load {max((v.get('load') or 0) for v in s.values())/10:.0f} %, laagste spanning {vmin} V, "
          f"lichaam {f['lift']*100:.1f} cm omhoog in het model, torque aan {sum(1 for v in s.values() if v.get('torque'))}/18")


if __name__ == '__main__': main()
