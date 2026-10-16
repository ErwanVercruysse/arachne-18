"""Walk straight ahead with a tripod gait (A = L1 L3 L5, B = L2 L4 L6). Front = the L1/L6 side (+x).

    .venv/bin/python tools/walk.py --dry               # plan one step + check limits
    .venv/bin/python tools/walk.py --loop STOPFILE     # keep walking until STOPFILE exists
    .venv/bin/python tools/walk.py --back ...          # walk backwards
"""
import argparse, math, os, sys, time
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import crab_walk as cw
from spin import LEGS, A, smooth


def cycle(home, d, lift):
    """Foot targets (body frame) for phase u in [0,3): A swings forward, B swings forward, body moves forward."""
    fwd = np.array([d, 0, 0])
    def f(u):
        tg = {}
        for L in LEGS:
            grp = 0 if L in A else 1; p = home[L]
            if u < 2:
                if grp == int(u):
                    s = u - grp; tg[L] = p + fwd*smooth(s) + np.array([0, 0, lift*math.sin(math.pi*min(1, s))])
                else:
                    tg[L] = p + fwd if grp < int(u) else p.copy()
            else:                                  # all feet down: body moves forward = feet slide back
                tg[L] = p + fwd*(1-smooth(u-2))
        return tg
    return f


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--dry', action='store_true'); ap.add_argument('--loop')
    ap.add_argument('--d', type=float, default=.03, help='step length (m)'); ap.add_argument('--lift', type=float, default=.025)
    ap.add_argument('--phase', type=float, default=.7, help='seconds per phase'); ap.add_argument('--hz', type=float, default=25)
    ap.add_argument('--back', action='store_true')
    a = ap.parse_args()
    J = cw.get('/state')['joints']
    q = np.zeros(cw.model.nq)
    for j, d in J.items(): q[cw.model.joint(j).qposadr[0]] = cw.q_of(j, d['ticks'])
    home = {L: cw.foot(L, q) for L in LEGS}
    f = cycle(home, -a.d if a.back else a.d, a.lift)
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
        print(f'stap {a.d*100:.1f} cm in {3*a.phase:.1f} s: IK-fout max {W*1000:.2f} mm, buiten grenzen {C}, grootste uitslag {max(h-l for l, h in rng.values())} ticks')
        return
    ports = sorted({d['port'] for d in J.values()})
    t0 = time.time(); k = 0
    while True:
        u = 3*(k % n)/n
        if k % n == 0 and k and (not a.loop or os.path.exists(a.loop)): break
        q, tk, _, _ = solve(u, q)
        for p in ports:
            cw.post('/cmd', {'port': p, 'op': 'sync', 'addr': 42,
                             'data': {str(J[j]['sid']): [tk[j] & 0xff, tk[j] >> 8, 0, 0, 0, 0] for j in J if J[j]['port'] == p}})
        if k % n == 0:
            s = cw.get('/state')['joints']; v = min((d.get('volt') or 99) for d in s.values())
            print(f'{time.time()-t0:5.1f} s  ~{a.d*100*(k//n):.0f} cm  spanning {v} V', flush=True)
            if v < 10.0: print('spanning te laag, gestopt', flush=True); break
        k += 1
        dt = t0 + k/a.hz - time.time()
        if dt > 0: time.sleep(dt)
    print(f'gestopt na {time.time()-t0:.0f} s, ~{a.d*100*(k//n):.0f} cm gestapt', flush=True)


if __name__ == '__main__': main()
