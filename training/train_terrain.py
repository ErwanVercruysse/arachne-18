"""Train the live controller (policy + terrain adaptation) on mixed rough terrain with domain randomization.

CEM (cross-entropy method), like the repository's original training, over the 12 policy parameters of
training/gait.py and 5 adaptation gains of tools/terrain_gait.py. Every generation draws new episodes and
all candidates are scored on the same ones:

  terrain   flat, bumps 3-9 mm, slopes 2-8 deg, steps 4-12 mm, cobblestones, mixed (bumps + steps + slope)
  command   random heading at 0.06-0.14 m/s, sometimes turning, sometimes turning on the spot
  robot     friction 0.5-1.2, payload 0-300 g, supply 11.1-12.6 V (3S), extra latency 0-15 ms,
            emulated MPU6050 with a new gyro bias every episode
  pushes    1-3 shoves of 8-25 N for 0.1-0.2 s from a random direction (the feet's friction holds ~18 N)

Reference MuJoCo + full BAM (the measured STS3215 model with --actuator), parallel CPU workers. Score per
episode = the terrain curriculum's reward (tracking, tilt, slip, smoothness; a fall scores -100); per
candidate 70 % mean + 30 % worst episode.

    python training/train_terrain.py --actuator simulation/bam_sts3215_12v_measured_m6.json --workers 4
    python training/train_terrain.py ... --resume training/runs/terrain_dr/state.json
    python training/train_terrain.py --actuator ... --assess training/policies/omni.json training/runs/terrain_dr/policy.json
"""
import argparse, json, math, sys, time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np

from gait import ROOT, LOW, HIGH, NAMES, load
from evaluate import evaluate
from curriculum import reward
from terrain_adapt import make_controller

sys.path.insert(0, str(ROOT/'tools'))
from terrain_gait import TerrainGait                                  # noqa: E402

GAIN_NAMES = ['level', 'level_dead', 'rough_lo', 'rough_span', 'lift_max']
GAIN_LOW = np.array([0., .3, .02, .05, 1.])
GAIN_HIGH = np.array([6., 2., .15, .40, 2.6])


def gains_of(g):
    level, dead, lo, span, lift = g
    return dict(level=float(level), level_dead=float(dead), rough_lo=float(lo), rough_hi=float(lo + span), lift_max=float(lift))


def initial_gains():
    G = TerrainGait.GAINS
    return np.array([G['level'], G['level_dead'], G['rough_lo'], G['rough_hi'] - G['rough_lo'], G['lift_max']])


def sample_episode(rng, seconds):
    kind = rng.choice(['flat', 'bumps', 'slope', 'terraces', 'cobbles', 'mixed'], p=[.12, .16, .16, .2, .2, .16])
    if kind == 'flat': terrain = {'kind': 'flat'}
    elif kind == 'bumps': terrain = {'kind': 'bumps', 'amplitude_m': rng.uniform(.003, .009), 'wavelength_m': [.10, .22]}
    elif kind == 'slope': terrain = {'kind': 'slope', 'slope_deg': rng.uniform(2, 8)}
    elif kind == 'terraces': terrain = {'kind': 'terraces', 'amplitude_m': rng.uniform(.004, .012)}
    elif kind == 'cobbles': terrain = {'kind': 'cobbles', 'height_m': rng.uniform(.002, .005), 'dome_m': rng.uniform(.002, .005),
                                       'joint_depth_m': rng.uniform(.004, .010)}
    else: terrain = {'kind': 'mixed', 'amplitude_m': rng.uniform(.004, .010), 'slope_deg': rng.uniform(0, 4), 'wavelength_m': [.10, .22]}
    r = rng.uniform()
    if r < .1: cmd = [0., 0., float(rng.choice([-1, 1])*rng.uniform(.15, .35))]
    else:
        a, v = rng.uniform(0, 2*math.pi), rng.uniform(.06, .14)
        cmd = [v*math.cos(a), v*math.sin(a), float(rng.uniform(-.3, .3)) if r < .35 else 0.]
    pushes = []
    for _ in range(int(rng.integers(1, 4))):
        a, f = rng.uniform(0, 2*math.pi), rng.uniform(8, 25)
        pushes.append((float(rng.uniform(2., seconds - 1.)), float(rng.uniform(.1, .2)), f*math.cos(a), f*math.sin(a)))
    return dict(terrain=terrain, command=[float(c) for c in cmd], seed=int(rng.integers(1 << 30)),
                friction=float(rng.uniform(.5, 1.2)), payload=float(rng.uniform(0, .3)), voltage=float(rng.uniform(11.1, 12.6)),
                extra_delay=float(rng.uniform(0, .015)), pushes=pushes)


def episode(task):
    params, g, ep, seconds, actuator = task
    try:
        r = evaluate(params, ep['command'], seconds, terrain=ep['terrain'], seed=ep['seed'], friction=ep['friction'],
                     payload=ep['payload'], voltage=ep['voltage'], extra_delay=ep['extra_delay'], pushes=ep['pushes'],
                     actuator_params=actuator, controller=make_controller(params, True, gains_of(g)))
        return reward(r), bool(r['passed'])
    except Exception as e:                                             # a solver blow-up counts as a fall
        print('episode failed:', e, flush=True); return -100., False


def assess(files, actuator, n, seconds, workers, seed=999):
    """Score policy files on the same n held-out episodes (a seed training never uses)."""
    rng = np.random.default_rng(seed); eps = [sample_episode(rng, seconds) for _ in range(n)]
    out = {}
    with ProcessPoolExecutor(workers) as pool:
        for f in files:
            d = json.loads(Path(f).read_text()); g = d.get('adaptation') or gains_of(initial_gains())
            vec = np.r_[np.array(d['parameters'])[:12], [g['level'], g['level_dead'], g['rough_lo'], g['rough_hi'] - g['rough_lo'], g['lift_max']]]
            res = list(pool.map(episode, [(vec[:12], vec[12:], ep, seconds, actuator) for ep in eps]))
            per = {}
            for ep, (sc, ok) in zip(eps, res): per.setdefault(ep['terrain']['kind'], []).append((sc, ok))
            out[str(f)] = dict(mean=float(np.mean([r[0] for r in res])), falls=int(sum(not r[1] for r in res)),
                               terrain={k: dict(mean=float(np.mean([x[0] for x in v])), falls=int(sum(not x[1] for x in v)), n=len(v)) for k, v in per.items()})
            o = out[str(f)]
            print(f"{Path(f).name:28s} mean score {o['mean']:7.2f}   falls {o['falls']}/{n}   " +
                  '  '.join(f"{k} {v['mean']:.1f} ({v['falls']} falls/{v['n']})" for k, v in sorted(o['terrain'].items())), flush=True)
    return dict(episodes=eps, results=out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--policy', type=Path, default=ROOT/'training/policies/omni.json')
    ap.add_argument('--actuator', type=Path, help='BAM parameter file (default: the simulator default)')
    ap.add_argument('--population', type=int, default=16)
    ap.add_argument('--elites', type=int, default=4)
    ap.add_argument('--episodes', type=int, default=4, help='episodes per candidate per generation')
    ap.add_argument('--seconds', type=float, default=8.)
    ap.add_argument('--generations', type=int, default=20)
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--seed', type=int, default=11)
    ap.add_argument('--out', type=Path, default=ROOT/'training/runs/terrain_dr')
    ap.add_argument('--resume', type=Path)
    ap.add_argument('--assess', type=Path, nargs='+', help='only score these policy files on held-out episodes')
    ap.add_argument('--assess-episodes', type=int, default=24)
    a = ap.parse_args()
    if a.assess:
        r = assess(a.assess, str(a.actuator) if a.actuator else None, a.assess_episodes, a.seconds, a.workers)
        a.out.mkdir(parents=True, exist_ok=True); (a.out/'assessment.json').write_text(json.dumps(r, indent=1) + '\n'); return
    a.out.mkdir(parents=True, exist_ok=True)
    low, high = np.r_[LOW, GAIN_LOW], np.r_[HIGH, GAIN_HIGH]
    if a.resume:
        st = json.loads(a.resume.read_text()); mean, std = np.array(st['mean']), np.array(st['std']); gen0 = st['generation'] + 1
        history = st['history']; best = st['best']
    else:
        mean = np.r_[load(a.policy)[:12], initial_gains()]; std = (high - low)*.06; gen0 = 0; history = []; best = None
    floor = (high - low)*.01
    actuator = str(a.actuator) if a.actuator else None
    rng = np.random.default_rng(a.seed + 1000*gen0)
    with ProcessPoolExecutor(a.workers) as pool:
        for gen in range(gen0, a.generations):
            t0 = time.time()
            eps = [sample_episode(rng, a.seconds) for _ in range(a.episodes)]
            cands = [mean.copy()] + [np.clip(mean + std*rng.normal(size=mean.size), low, high) for _ in range(a.population - 1)]
            tasks = [(c[:12], c[12:], ep, a.seconds, actuator) for c in cands for ep in eps]
            out = list(pool.map(episode, tasks))
            scores = np.array([s for s, _ in out]).reshape(len(cands), len(eps))
            falls = np.array([not ok for _, ok in out]).reshape(len(cands), len(eps))
            fitness = .7*scores.mean(1) + .3*scores.min(1)
            order = np.argsort(-fitness); elite = np.array([cands[i] for i in order[:a.elites]])
            mean = .3*mean + .7*elite.mean(0)                          # smoothed update, incumbent kept as candidate 0
            std = np.maximum(.5*std + .5*elite.std(0), floor)
            row = dict(generation=gen, seconds=round(time.time() - t0, 1), incumbent=float(fitness[0]), best=float(fitness[order[0]]),
                       median=float(np.median(fitness)), incumbent_falls=int(falls[0].sum()), episodes=eps)
            history.append(row)
            if best is None or fitness[order[0]] > best['fitness']:
                best = dict(fitness=float(fitness[order[0]]), generation=gen, vector=cands[order[0]].tolist())
            print(f"gen {gen:2d}  incumbent {fitness[0]:7.2f} ({falls[0].sum()} falls)  best {fitness[order[0]]:7.2f}  "
                  f"median {np.median(fitness):7.2f}  terrains {[e['terrain']['kind'] for e in eps]}  {row['seconds']:.0f} s", flush=True)
            (a.out/'state.json').write_text(json.dumps(dict(mean=mean.tolist(), std=std.tolist(), generation=gen, history=history,
                                                            best=best, actuator=actuator, args={k: str(v) for k, v in vars(a).items()}), indent=1))
    v = np.array(mean)
    policy = dict(algorithm='CEM, mixed terrain + domain randomization + pushes (training/train_terrain.py)',
                  parameter_names=NAMES, parameters=v[:12].tolist(), adaptation=gains_of(v[12:]),
                  actuator=actuator, generations=a.generations, population=a.population, episodes_per_generation=a.episodes,
                  start=str(a.policy.relative_to(ROOT)) if a.policy.is_relative_to(ROOT) else str(a.policy))
    (a.out/'policy.json').write_text(json.dumps(policy, indent=2) + '\n')
    print('saved', a.out/'policy.json')


if __name__ == '__main__': main()
