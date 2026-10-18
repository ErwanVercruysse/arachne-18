"""ARACHNE body dance on the real robot (the 'body dance' from simulation/stunts.py).

    .venv/bin/python tools/dance.py --dry     # plan + limit check
    .venv/bin/python tools/dance.py           # dance (live_twin.py must be running, robot standing)

Feet stay where they are now; the body circles and tilts. Joint angles come from MuJoCo IK on the
real standing pose, ticks from calibration/twin_map.json, sent with one SYNC WRITE per adapter.
"""
import argparse, json, math, os, sys, time, urllib.request
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import crab_walk as cw            # model, IK, tick conversion

URL = os.environ.get('ARACHNE_URL', 'http://localhost:5870')   # ARACHNE_URL=.../sim drives the simulation
LEGS = ('L1', 'L2', 'L3', 'L4', 'L5', 'L6')


def smooth(x): x = min(max(x, 0.), 1.); return x*x*(3-2*x)
def envelope(t, L, rise=1.): return smooth(t/rise)*smooth((L-t)/rise)


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


DIP = None          # m: how deep the body bends down on every beat; None = as deep as the joint limits allow
TILT = 7.           # degrees of body tilt (scaled by --amp)


def body_dance(t, L=9., amp=1., e=None, dip=None, tilt=None):
    env = envelope(t, L) if e is None else e
    e = env*amp; w = 2*math.pi*.45*t
    dip = DIP if dip is None else dip; tilt = TILT if tilt is None else tilt
    # body circles and tilts; twice per circle it bends deep down and comes back up (feet stay put)
    z = -dip*env*(1 - math.cos(2*w))/2
    return (np.array([.018*e*math.cos(w), .018*e*math.sin(w), z]),
            (math.radians(tilt)*e*math.sin(w), -math.radians(tilt)*e*math.cos(w), 0))


def max_dip(J, q, feet, amp, tilt, hi=.06):
    """Deepest dip (m) for which one full dance circle stays inside every joint's limits (40-tick margin).
    Checked on the current standing pose, so it adapts to how the robot stands right now."""
    def fits(d):
        qq = q.copy()
        for t in np.arange(0, 1/.45, .08):
            shift, rpy = body_dance(t, amp=amp, e=1., dip=d, tilt=tilt); R = rot(*rpy)
            for Lg in LEGS: qq, _ = cw.ik(Lg, R.T @ (feet[Lg] - shift), qq)
            for j in J:
                t_ = cw.ticks_of(j, qq[cw.model.joint(j).qposadr[0]])
                if not cw.LIM[j]['min']+40 <= t_ <= cw.LIM[j]['max']-40: return False
        return True
    lo = 0.
    if not fits(lo): return 0.
    for _ in range(6):
        mid = (lo + hi)/2
        if fits(mid): lo = mid
        else: hi = mid
    return lo


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--dry', action='store_true')
    ap.add_argument('--amp', type=float, default=1.); ap.add_argument('--hz', type=float, default=25)
    ap.add_argument('--dip', type=float, default=DIP, help='how deep the body bends down (m); default: as deep as possible')
    ap.add_argument('--tilt', type=float, default=TILT, help='body tilt (degrees, x amp)')
    ap.add_argument('--loop', help='dance until this file exists (fade in/out 1 s)')
    a = ap.parse_args()
    J = cw.get('/state')['joints']
    q = np.zeros(cw.model.nq)
    for j, d in J.items(): q[cw.model.joint(j).qposadr[0]] = cw.q_of(j, d['ticks'])
    feet = {L: cw.foot(L, q) for L in LEGS}
    hip = {L: cw.model.body(f'{L}_COXA').id for L in LEGS}
    if a.dip is None:
        a.dip = max_dip(J, q, feet, a.amp, a.tilt)
        print(f'zo diep mogelijk: {a.dip*100:.1f} cm', flush=True)
    if a.loop: return loop(a, J, q, feet)
    frames = []; worst = 0; clip = 0; L_ = 9.
    for k in range(int(L_*a.hz)+1):
        t = k/a.hz; shift, rpy = body_dance(t, L_, a.amp, dip=a.dip, tilt=a.tilt); R = rot(*rpy)
        # body moves by (R, shift) about its origin; feet fixed in the world -> targets in the body frame
        for Lg in LEGS:
            q, e = cw.ik(Lg, R.T @ (feet[Lg] - shift), q); worst = max(worst, e)
        tk = {}
        for j in J:
            t_ = cw.ticks_of(j, q[cw.model.joint(j).qposadr[0]]); lo, hi = cw.LIM[j]['min']+40, cw.LIM[j]['max']-40
            if not lo <= t_ <= hi: clip += 1
            tk[j] = int(round(min(hi, max(lo, t_))))
        frames.append(tk)
    rng = {j: (min(f[j] for f in frames), max(f[j] for f in frames)) for j in J}
    print(f'{len(frames)} frames, IK-fout max {worst*1000:.2f} mm, buiten grenzen {clip}, grootste uitslag {max(r[1]-r[0] for r in rng.values())} ticks')
    if a.dry:
        for j, r in sorted(rng.items()): print(f'  {j:8s} {r[0]}..{r[1]} (nu {J[j]["ticks"]})')
        return
    ports = sorted({d['port'] for d in J.values()})
    t0 = time.time(); vmin = 99; lagmax = 0
    for k, tk in enumerate(frames):
        for p in ports:
            data = {str(J[j]['sid']): [tk[j] & 0xff, tk[j] >> 8, 0, 0, 0, 0] for j in J if J[j]['port'] == p}
            cw.post('/cmd', {'port': p, 'op': 'sync', 'addr': 42, 'data': data})
        if k % 5 == 0:
            s = cw.get('/state')['joints']
            vmin = min([vmin]+[(d.get('volt') or 99) for d in s.values()])
            lagmax = max([lagmax]+[abs(s[j]['ticks']-frames[max(0, k-3)][j]) for j in s if j in tk])
            if vmin < 10.2: print('spanning te laag, gestopt'); break
        dt = t0 + (k+1)/a.hz - time.time()
        if dt > 0: time.sleep(dt)
    print(f'klaar in {time.time()-t0:.1f} s, min spanning {vmin} V, max achterstand {lagmax} ticks')


def frame(J, q, feet, t, amp, e, dip=None, tilt=None):
    shift, rpy = body_dance(t, amp=amp, e=e, dip=dip, tilt=tilt); R = rot(*rpy)
    for Lg in LEGS: q, _ = cw.ik(Lg, R.T @ (feet[Lg] - shift), q)
    tk = {}
    for j in J:
        t_ = cw.ticks_of(j, q[cw.model.joint(j).qposadr[0]]); lo, hi = cw.LIM[j]['min']+40, cw.LIM[j]['max']-40
        tk[j] = int(round(min(hi, max(lo, t_))))
    return q, tk


def loop(a, J, q, feet):
    import os
    ports = sorted({d['port'] for d in J.values()})
    t0 = time.time(); k = 0; stop_at = None; vmin = 99
    while True:
        t = k/a.hz
        if stop_at is None and os.path.exists(a.loop): stop_at = t
        e = smooth(t) if stop_at is None else smooth(1-(t-stop_at))
        if stop_at is not None and t-stop_at >= 1: break
        q, tk = frame(J, q, feet, t, a.amp, e, a.dip, a.tilt)
        for p in ports:
            data = {str(J[j]['sid']): [tk[j] & 0xff, tk[j] >> 8, 0, 0, 0, 0] for j in J if J[j]['port'] == p}
            cw.post('/cmd', {'port': p, 'op': 'sync', 'addr': 42, 'data': data})
        if k % 25 == 0:
            s = cw.get('/state')['joints']; vmin = min([vmin]+[(d.get('volt') or 99) for d in s.values()])
            print(f'{t:6.1f} s  spanning {vmin} V', flush=True)
            if vmin < 10.2: print('spanning te laag, gestopt', flush=True); break
        k += 1
        dt = t0 + k/a.hz - time.time()
        if dt > 0: time.sleep(dt)
    print(f'gestopt na {time.time()-t0:.0f} s, terug in stahouding', flush=True)


if __name__ == '__main__': main()
