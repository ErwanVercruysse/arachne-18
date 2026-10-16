"""Turn on the spot with a tripod gait (A = L1 L3 L5, B = L2 L4 L6). Needs live_twin.py and a standing robot.

    .venv/bin/python tools/spin.py --dry               # plan one turn step + check limits
    .venv/bin/python tools/spin.py --loop STOPFILE     # keep turning until STOPFILE exists
"""
import argparse, math, os, sys, time
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import crab_walk as cw

LEGS = ('L1', 'L2', 'L3', 'L4', 'L5', 'L6'); A, B = ('L1', 'L3', 'L5'), ('L2', 'L4', 'L6')


def rz(a, p): c, s = math.cos(a), math.sin(a); return np.array([c*p[0]-s*p[1], s*p[0]+c*p[1], p[2]])
def smooth(x): x = min(max(x, 0.), 1.); return x*x*(3-2*x)


def cycle(home, step, lift):
    """Foot targets (body frame) as a function of phase u in [0,3): A swings, B swings, body turns."""
    def f(u):
        tg = {}
        for L in LEGS:
            grp = 0 if L in A else 1; p = home[L]
            if u < 2:
                if grp == int(u):                      # this tripod is swinging
                    s = u - grp; ang = step*smooth(s); h = lift*math.sin(math.pi*min(1, s))
                    tg[L] = rz(ang, p) + np.array([0, 0, h])
                else:
                    tg[L] = rz(step, p) if grp < int(u) else p.copy()
            else:                                      # all down: body turns, feet slide back in body frame
                tg[L] = rz(step*(1-smooth(u-2)), p)
        return tg
    return f


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--dry', action='store_true'); ap.add_argument('--loop')
    ap.add_argument('--step', type=float, default=8., help='degrees per cycle'); ap.add_argument('--lift', type=float, default=.02)
    ap.add_argument('--phase', type=float, default=.8, help='seconds per phase'); ap.add_argument('--hz', type=float, default=25)
    a = ap.parse_args()
    J = cw.get('/state')['joints']
    q = np.zeros(cw.model.nq)
    for j, d in J.items(): q[cw.model.joint(j).qposadr[0]] = cw.q_of(j, d['ticks'])
    home = {L: cw.foot(L, q) for L in LEGS}
    f = cycle(home, math.radians(a.step), a.lift)
    def solve(u, q):
        tg = f(u); worst = 0
        for L in LEGS: q, e = cw.ik(L, tg[L], q); worst = max(worst, e)
        tk, clip = {}, 0
        for j in J:
            t = cw.ticks_of(j, q[cw.model.joint(j).qposadr[0]]); lo, hi = cw.LIM[j]['min']+40, cw.LIM[j]['max']-40
            if not lo <= t <= hi: clip += 1
            tk[j] = int(round(min(hi, max(lo, t))))
        return q, tk, worst, clip
    n = int(3*a.phase*a.hz)
    if a.dry:
        qq = q.copy(); W = C = 0; rng = {}
        for k in range(n+1):
            qq, tk, w, c = solve(3*k/n, qq); W = max(W, w); C += c
            for j, t in tk.items(): lo, hi = rng.get(j, (t, t)); rng[j] = (min(lo, t), max(hi, t))
        print(f'1 cyclus = {3*a.phase:.1f} s, {a.step}°: IK-fout max {W*1000:.2f} mm, buiten grenzen {C}, grootste uitslag {max(h-l for l, h in rng.values())} ticks')
        return
    ports = sorted({d['port'] for d in J.values()})
    t0 = time.time(); k = 0; turned = 0
    while True:
        u = 3*(k % n)/n
        if k % n == 0 and k and (not a.loop or os.path.exists(a.loop)): break
        q, tk, _, _ = solve(u, q)
        for p in ports:
            cw.post('/cmd', {'port': p, 'op': 'sync', 'addr': 42,
                             'data': {str(J[j]['sid']): [tk[j] & 0xff, tk[j] >> 8, 0, 0, 0, 0] for j in J if J[j]['port'] == p}})
        if k % n == 0:
            turned = a.step*(k//n)
            s = cw.get('/state')['joints']; v = min((d.get('volt') or 99) for d in s.values())
            print(f'{time.time()-t0:5.1f} s  gedraaid ~{turned:.0f}°  spanning {v} V', flush=True)
            if v < 10.0: print('spanning te laag, gestopt', flush=True); break
        k += 1
        dt = t0 + k/a.hz - time.time()
        if dt > 0: time.sleep(dt)
    print(f'gestopt na {time.time()-t0:.0f} s, ~{a.step*(k//n):.0f}° gedraaid', flush=True)


if __name__ == '__main__': main()
