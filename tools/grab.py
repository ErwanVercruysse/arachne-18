"""Stand on four legs and pick something up between the two front feet (L1 and L6), then put it down again.

    .venv/bin/python tools/grab.py --dry                     # plan + limit/stability check, no motion
    .venv/bin/python tools/grab.py --at .20 --width .13      # object 20 cm in front of the body centre, 13 cm wide
    .venv/bin/python tools/grab.py --loop STOPFILE           # hold it until STOPFILE exists, then put it down

How: L2 and L5 step forward, the body leans back over L2-L5 (four-leg support), L1/L6 lift like arms, reach
around the object, squeeze it with soft yaw servos (torque limit 40 %, so nothing gets crushed or overloaded),
lift it and bring it in. Putting down is the same in reverse. The front legs can only swing ~45 deg inwards, so
the toes cannot come closer than ~12 cm: objects 12-15 cm wide can be held (a small box, a big ball, a cup).
Simulation (training/grab_test.py): 12-15 cm boxes up to 250 g, friction down to 0.4, lifted ~4.5 cm, max body tilt 2 deg.
Needs live_twin.py and a standing robot; ARACHNE_URL=.../sim does the same in the simulation.
"""
import argparse, json, math, os, sys, time, urllib.request
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from motion import smooth                                             # noqa: E402
from policy_walk import Joints, JOINTS, fk, ik                        # noqa: E402  (closed-form leg IK, measured limits)

LEGS = ('L1', 'L2', 'L3', 'L4', 'L5', 'L6')

URL = os.environ.get('ARACHNE_URL', 'http://localhost:5870')
HZ = 50
ARMS, SUPPORT = ('L1', 'L6'), ('L2', 'L3', 'L4', 'L5')
SQUEEZE_LIMIT = 400                                                   # register 48 on the arm yaw servos while holding
Y_MIN = .061                     # the front legs' yaw limit keeps each toe at least this far from the centre line


def get(path): return json.loads(urllib.request.urlopen(URL + path, timeout=2).read())
def post(path, d): return json.loads(urllib.request.urlopen(urllib.request.Request(URL + path, json.dumps(d).encode(), method='POST'), timeout=2).read())


def plan(home, at=.20, width=.13, ground=None, grip_h=.025, lean=.045, step=.04):
    """Key frames [(seconds, feet {leg: xyz body frame at the START pose}, body shift xyz, label)].
    home: foot positions of the standing robot (body frame); the floor is at the feet's height."""
    z0 = np.mean([home[L][2] for L in LEGS]) if ground is None else ground
    f = {L: home[L].copy() for L in LEGS}; shift = np.zeros(3); out = []
    def key(t, label): out.append((t, {L: v.copy() for L, v in f.items()}, shift.copy(), label))
    key(0., 'staan')
    for L in ('L2', 'L5'):                                            # middle legs one step forward (two keys: up, down)
        up = f[L] + [step/2, 0, .025]; f[L] = up; key(.35, f'{L} stap'); f[L] = up + [step/2, 0, -.025]; key(.35, f'{L} stap')
    shift = np.array([-lean, 0, 0.]); key(1.2, 'leunt achterover')    # body back over L2..L5
    for L in ARMS: f[L] = f[L] + [.02, 0, .06]
    key(.9, 'armen op')
    half = width/2 + .03                                              # open: toes outside the object
    for L, s in zip(ARMS, (1, -1)): f[L] = np.array([at, s*half, z0 + .06])
    key(1.2, 'reiken')
    for L, s in zip(ARMS, (1, -1)): f[L] = np.array([at, s*half, z0 + grip_h])
    key(.8, 'zakken')
    # squeeze: aim 12 mm inside the object's side, but not past what the yaw can reach (the soft servo then
    # pushes against its limit); the toe ball (7.5 mm) touches the object before that
    for L, s in zip(ARMS, (1, -1)): f[L] = np.array([at, s*max(width/2 - .012, Y_MIN + .002), z0 + grip_h])
    key(.8, 'knijpen')
    for L in ARMS: f[L] = f[L] + [-.015, 0, .05]                      # straight up, a little in (L1's knee range is short)
    key(1.2, 'optillen')
    return out


def frames(keys, hz=HZ):
    """Smooth interpolation between key frames -> [(feet, shift, label)] at hz; each leg's z gets a lift when it steps."""
    out = []
    for (_, fa, sa, _), (dur, fb, sb, label) in zip(keys, keys[1:]):
        n = max(1, int(dur*hz))
        for k in range(1, n + 1):
            s = smooth(k/n); out.append(({L: fa[L] + (fb[L] - fa[L])*s for L in LEGS}, sa + (sb - sa)*s, label))
    return out


def solve(J, feet, shift):
    """Body-frame foot targets (start-pose frame) + body shift -> joint angles [18]; and worst IK error (m)."""
    t = np.array([feet[L] - shift for L in LEGS])
    q = ik(t, *J.real)
    return q, float(np.abs(fk(q) - t).max())


def support_margin(feet, shift, com_xy=(0., 0.)):
    """Distance (m) of the centre of mass inside the support polygon of the legs that are down (negative = outside)."""
    down = [feet[L][:2] - shift[:2] for L in SUPPORT]
    pts = np.array(down); c = np.array(com_xy)
    hull = pts[np.argsort(np.arctan2(pts[:, 1] - pts[:, 1].mean(), pts[:, 0] - pts[:, 0].mean()))]
    m = np.inf
    for a, b in zip(hull, np.roll(hull, -1, 0)):
        e = b - a; n = np.array([e[1], -e[0]])/np.linalg.norm(e)
        m = min(m, float(-(c - a) @ n))
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--at', type=float, default=.20, help='object centre, m in front of the body centre')
    ap.add_argument('--width', type=float, default=.13, help='object width (m), 0.12-0.15')
    ap.add_argument('--hold', type=float, default=3., help='seconds to hold it (without --loop)')
    ap.add_argument('--loop', help='hold until this file exists')
    ap.add_argument('--dry', action='store_true')
    a = ap.parse_args()
    if not .115 <= a.width <= .155: raise SystemExit('breedte 12-15 cm: de voorpoten komen niet dichter dan ~12 cm bij elkaar')
    Jt = Joints()
    if a.dry: q0 = Jt.stand_q(); S = None
    else:
        S = get('/state')['joints']
        if len(S) < 18 or sum(1 for d in S.values() if d.get('torque')) < 18: raise SystemExit('eerst opstaan (alle 18 servo\'s aan)')
        q0 = Jt.q_of({j: S[j]['ticks'] for j in JOINTS})
    f0 = fk(q0); home = {L: f0[i] for i, L in enumerate(LEGS)}
    keys = plan(home, a.at, a.width)
    traj = []; worst = 0.; margin = np.inf
    for feet, shift, label in frames(keys):
        q, e = solve(Jt, feet, shift); traj.append((q, label))
        if label != 'knijpen': worst = max(worst, e)
        if label not in ('staan', 'L2 stap', 'L5 stap'): margin = min(margin, support_margin(feet, shift))
    ticks = [Jt.ticks_of(q) for q, _ in traj]
    print(f'{len(traj)} frames ({len(traj)/HZ:.1f} s heen), IK-fout max {worst*1000:.1f} mm, '
          f'stabiliteitsmarge op 4 poten {margin*100:.1f} cm (zwaartepunt binnen L2-L5)')
    if worst > .004: raise SystemExit('niet bereikbaar binnen de gewrichtsgrenzen: dichterbij zetten (--at) of andere breedte')
    if margin < .015: raise SystemExit('te weinig marge op vier poten, ik doe het niet')
    if a.dry: return
    send = lambda t: post('/sync', {'ticks': t})
    yaw = {L: S[f'{L}_yaw'] for L in ARMS}
    def soft(limit):
        for d in yaw.values(): post('/cmd', {'port': d['port'], 'op': 'write', 'id': d['sid'], 'addr': 48, 'data': [limit & 0xff, limit >> 8]})
    t_next = time.time(); squeezing = False
    def play(seq):
        nonlocal t_next, squeezing
        for t, (_, label) in seq:
            if label == 'knijpen' and not squeezing: soft(SQUEEZE_LIMIT); squeezing = True
            send(t); t_next += 1/HZ; time.sleep(max(0., t_next - time.time()))
    try:
        play(list(zip(ticks, traj)))
        print('vast', flush=True)
        end = time.time() + a.hold
        while (os.path.exists(a.loop) is False) if a.loop else time.time() < end: time.sleep(.05)
        print('neerzetten', flush=True); t_next = time.time()
        play(list(zip(ticks[::-1], traj[::-1])))
    finally:
        soft(1000)
    print('klaar', flush=True)


if __name__ == '__main__': main()
