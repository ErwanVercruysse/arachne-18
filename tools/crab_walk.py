"""Slow sideways (crab) walk on the legs that are connected, driven through live_twin's /cmd.

    .venv/bin/python tools/crab_walk.py --dry          # plan + check limits only
    .venv/bin/python tools/crab_walk.py --steps 2      # walk (live_twin.py must be running)

Feet are planned in the body frame with MuJoCo FK/IK on arachne_fixed.xml; ticks come from
calibration/twin_map.json (zero/sign) and are clamped to calibration/joint_limits.json.
Before each leg lifts, the body shifts toward the other three feet so the COM stays inside them.
"""
import argparse, json, math, os, sys, time, urllib.request
from pathlib import Path
import mujoco, numpy as np

ROOT = Path(__file__).resolve().parents[1]
URL = os.environ.get('ARACHNE_URL', 'http://localhost:5870')   # ARACHNE_URL=.../sim drives the simulation
T = 2*math.pi/4096
model = mujoco.MjModel.from_xml_path(str(ROOT/'simulation/arachne_fixed.xml')); data = mujoco.MjData(model)
CAL = json.loads((ROOT/'calibration/twin_map.json').read_text())
LIM = json.loads((ROOT/'calibration/joint_limits.json').read_text())['joints']
PARTS = ('yaw', 'hip', 'knee')
MAX_SHIFT = .03   # m, body shift before a leg lifts
MARGIN = .018     # m, COM inside the stance triangle


def get(path): return json.loads(urllib.request.urlopen(URL+path).read())
def post(path, d): return json.loads(urllib.request.urlopen(urllib.request.Request(URL+path, json.dumps(d).encode(), method='POST')).read())


def q_of(j, ticks): c = CAL[j]; return c['sign']*(ticks-c['zero'])*T
def ticks_of(j, q): c = CAL[j]; return c['zero'] + q/(c['sign']*T)


def foot(leg, q):
    data.qpos[:] = q; mujoco.mj_kinematics(model, data); return data.site(f'{leg}_foot').xpos.copy()


def ik(leg, target, q):
    """Damped least squares on the 3 joints of one leg; body is fixed in the model."""
    q = q.copy(); dofs = [model.joint(f'{leg}_{p}').dofadr[0] for p in PARTS]; adr = [model.joint(f'{leg}_{p}').qposadr[0] for p in PARTS]
    sid = model.site(f'{leg}_foot').id
    for _ in range(60):
        data.qpos[:] = q; mujoco.mj_kinematics(model, data); mujoco.mj_comPos(model, data)
        err = target - data.site_xpos[sid]
        if np.linalg.norm(err) < 2e-4: break
        jp = np.zeros((3, model.nv)); mujoco.mj_jacSite(model, data, jp, None, sid)
        Jl = jp[:, dofs]; dq = Jl.T @ np.linalg.solve(Jl @ Jl.T + 1e-4*np.eye(3), err)
        for a, d in zip(adr, dq): q[a] += d
    return q, float(np.linalg.norm(err))


def margin(pt, tri):
    """Signed distance (m) from pt to the nearest edge of triangle tri (xy); >0 = inside."""
    m = 1e9
    for i in range(3):
        a, b, o = tri[i][:2], tri[(i+1) % 3][:2], tri[(i+2) % 3][:2]
        n = np.array([-(b-a)[1], (b-a)[0]]); n /= np.linalg.norm(n)
        if np.dot(o-a, n) < 0: n = -n
        m = min(m, float(np.dot(pt-a, n)))
    return m


def safe_shift(stance, body, axis):
    """Smallest body move from the recentred pose toward the triangle's incentre giving MARGIN."""
    tri = [p for p in stance]; P = [t[:2] for t in tri]
    sides = [np.linalg.norm(P[(i+1) % 3]-P[(i+2) % 3]) for i in range(3)]
    inc = sum(s*p for s, p in zip(sides, P))/sum(sides)
    home = np.zeros(3); home[axis] = body[axis]
    for a in np.linspace(0, 1, 41):
        cand = home.copy(); cand[:2] += a*(inc-home[:2])
        if margin(cand[:2], tri) >= MARGIN or np.linalg.norm(cand[:2]-home[:2]) >= MAX_SHIFT: return cand
    return cand


def plan(legs, q0, steps, d, lift, axis):
    """Keyframes: list of (seconds, {leg: foot target in body frame}). Body shift = moving all feet opposite."""
    feet = {L: foot(L, q0) for L in legs}
    z0 = {L: feet[L][2] for L in legs}
    keys = []; body = np.zeros(3); world = {L: feet[L].copy() for L in legs}
    def frame(dur): keys.append((dur, {L: world[L]-body for L in legs}))
    order = sorted(legs, key=lambda L: -feet[L][axis])        # leading side first
    for _ in range(steps):
        for L in order:
            others = [world[o] for o in legs if o != L]
            body = safe_shift(others, body, axis); frame(1.5)                               # move COM over the other three
            world[L][2] += lift; frame(.8)                            # lift
            world[L][axis] += d; frame(1.0)                           # swing sideways
            world[L][2] -= lift; frame(.8)                            # put down
        body = np.zeros(3); body[axis] = np.mean([world[L][axis] for L in legs]) - np.mean([feet[L][axis] for L in legs])
        body[1-axis] = 0; frame(1.5)                                   # recentre over all feet
    return keys


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--dry', action='store_true'); ap.add_argument('--steps', type=int, default=2)
    ap.add_argument('--d', type=float, default=.02); ap.add_argument('--lift', type=float, default=.02)
    a = ap.parse_args()
    st = get('/state'); J = st['joints']
    legs = sorted({j[:2] for j in J if all(f'{j[:2]}_{p}' in J for p in PARTS)})
    q0 = np.zeros(model.nq)
    for j, d in J.items(): q0[model.joint(j).qposadr[0]] = q_of(j, d['ticks'])
    keys = plan(legs, q0, a.steps, a.d, a.lift, axis=1)
    # solve every keyframe, check limits
    q = q0.copy(); sol = []; worst = 0; clip = []
    for dur, tg in keys:
        for L, p in tg.items():
            q, e = ik(L, p, q); worst = max(worst, e)
        tk = {}
        for L in legs:
            for p in PARTS:
                j = f'{L}_{p}'; t = ticks_of(j, q[model.joint(j).qposadr[0]]); lo, hi = LIM[j]['min']+40, LIM[j]['max']-40
                if not lo <= t <= hi: clip.append((j, round(t)))
                tk[j] = int(round(min(hi, max(lo, t))))
        sol.append((dur, tk))
    print(f'poten {legs}, {len(sol)} keyframes, {sum(d for d, _ in sol):.0f} s, IK-fout max {worst*1000:.1f} mm, buiten grenzen: {len(clip)} {clip[:6]}')
    rng = {j: (min(t[j] for _, t in sol), max(t[j] for _, t in sol)) for j in sol[0][1]}
    for j, r in sorted(rng.items()): print(f'  {j:8s} {r[0]}..{r[1]}  (nu {J[j]["ticks"]})')
    if a.dry: return
    def w(j, adr, dd): post('/cmd', {'port': J[j]['port'], 'op': 'write', 'id': J[j]['sid'], 'addr': adr, 'data': dd})
    cur = {j: J[j]['ticks'] for j in sol[0][1]}
    for dur, tk in sol:
        n = max(1, int(dur/.05)); a0 = dict(cur)
        for k in range(1, n+1):
            s = k/n; s = s*s*(3-2*s)
            for j in tk: cur[j] = a0[j] + s*(tk[j]-a0[j]); t = int(round(cur[j])); w(j, 42, [t & 0xff, t >> 8, 0, 0, 0xe8, 3])
            time.sleep(.05)
        js = get('/state')['joints']
        lag = max(abs(js[j]['ticks']-tk[j]) for j in tk); v = min((js[j].get('volt') or 99) for j in tk)
        print(f'keyframe ok, max achterstand {lag} ticks, {v} V', flush=True)
        if lag > 250 or v < 10.0:
            for j in tk: t = js[j]['ticks']; w(j, 42, [t & 0xff, t >> 8, 0, 0, 0, 0])
            print('GESTOPT: houdt vast waar hij staat'); return
    print('klaar')


if __name__ == '__main__': main()
