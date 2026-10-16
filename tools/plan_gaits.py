"""Pre-plan the walk/turn cycles from the robot's current standing pose and print them as JSON ticks.

Same maths as walk.py and spin.py (their cycle() functions, crab_walk's IK, clamped to joint_limits - 40),
so gamepad_robot.py can play them back instantly instead of starting a new script (2.8 s + 880 MB on the Pi)
for every stick push. Each cycle has 3 phases of the same number of frames, so frames n/3 and 2n/3 are moments
with all six feet on the ground: the controller can stop there.

    .venv/bin/python tools/plan_gaits.py > /tmp/gaits.json
"""
import json, math, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import crab_walk as cw
import walk, spin

# Speed levels (RB on the controller). Limits measured on the robot (2026-10-14) from the standing pose:
# step <= 5 cm (5.5 cm makes the IK jump), turn <= 24 deg (26 deg leaves the joint limits), and the STS3215 does
# ~3000 ticks/s unloaded at 12 V. 'max' runs the fastest joint right at that servo limit.
LEVELS = {                        # level: (hz, {name: (cycle function, step, lift, frames per phase)})
    'normaal': (25, {             # exactly walk.py / spin.py defaults (0.72 s/phase walk, 0.8 s turn)
        'lopen': (walk.cycle, .03, .025, 18), 'achteruit': (walk.cycle, -.03, .025, 18),
        'links draaien': (spin.cycle, math.radians(8), .02, 20), 'rechts draaien': (spin.cycle, math.radians(-8), .02, 20)}),
    'snel': (50, {                # walk 4 cm / 0.6 s (peak ~2400 ticks/s), turn 16 deg / 0.5 s
        'lopen': (walk.cycle, .04, .025, 30), 'achteruit': (walk.cycle, -.04, .025, 30),
        'links draaien': (spin.cycle, math.radians(16), .02, 25), 'rechts draaien': (spin.cycle, math.radians(-16), .02, 25)}),
    'max': (50, {                 # walk 4 cm / 0.48 s (peak ~3000 ticks/s = servo limit), turn 24 deg / 0.3 s
        'lopen': (walk.cycle, .04, .025, 24), 'achteruit': (walk.cycle, -.04, .025, 24),
        'links draaien': (spin.cycle, math.radians(24), .02, 15), 'rechts draaien': (spin.cycle, math.radians(-24), .02, 15)}),
}


def main():
    J = cw.get('/state')['joints']
    q0 = np.zeros(cw.model.nq)
    for j, d in J.items(): q0[cw.model.joint(j).qposadr[0]] = cw.q_of(j, d['ticks'])
    home = {L: cw.foot(L, q0) for L in spin.LEGS}
    out = {'sid': {j: J[j]['sid'] for j in J}, 'port': {j: J[j]['port'] for j in J}, 'motions': {}}
    for level, (hz, motions) in LEVELS.items():
        for name, (cyc, step, lift, per) in motions.items():
            f = cyc(home, step, lift); n = 3*per; q = q0.copy(); frames = []; clip = 0; worst = 0
            for k in range(n):
                tg = f(3*k/n)
                for L in spin.LEGS: q, e = cw.ik(L, tg[L], q); worst = max(worst, e)
                fr = {}
                for j in J:
                    t = cw.ticks_of(j, q[cw.model.joint(j).qposadr[0]]); lo, hi = cw.LIM[j]['min'] + 40, cw.LIM[j]['max'] - 40
                    if not lo <= t <= hi: clip += 1
                    fr[j] = int(round(min(hi, max(lo, t))))
                frames.append(fr)
            fs = [list(fr.values()) for fr in frames]; fs.append(fs[0])
            peak = max(abs(b - a) for r0, r1 in zip(fs, fs[1:]) for a, b in zip(r0, r1))*hz
            out['motions'][f'{level}/{name}'] = {'n': n, 'hz': hz, 'frames': frames, 'clipped': clip, 'ik_mm': round(worst*1000, 2), 'peak': peak}
            print(f'{level}/{name}: {n} frames @ {hz} Hz, buiten grenzen {clip}, IK-fout max {worst*1000:.2f} mm, piek {peak} ticks/s', file=sys.stderr)
    json.dump(out, sys.stdout)


if __name__ == '__main__': main()
