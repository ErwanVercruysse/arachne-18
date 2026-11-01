# Uneven-ground curriculum

> **Mechanical revision v4:** checkpoints, recorded metrics and videos below are historical pre-v4 results. No RL retraining was performed. Current model/IK adapters use v4 geometry; prior performance is not established for v4. See [revision notes](../docs/revision-v4.md).


Watch the [terrain films](https://tom-michiels.github.io/arachne-18/#terrain):
rolling ground up to 4 mm, slopes up to 2°, and 4 mm plateaus. Each is a
continuous 12-second original MuJoCo/BAM run, at real-time playback and true
terrain scale. Reproduce them with `python training/render_terrain.py`.

The curriculum admits only an already smooth flat-ground controller. It then
increases terrain difficulty while retaining velocity tracking, a steady body
and smooth motor targets. It uses the original MuJoCo model and full BAM M6
servo runtime in parallel CPU workers. The local fused Metal backend does not
support heightfields; terrain training does **not** silently flatten the ground.

## Current measured status

The original general policy (`policies/omni.json`) passed **318 admission
episodes** across the first five stages. Each episode lasts 10 seconds, uses
held-out headings and includes 0.15° orientation-estimate noise plus 0.005 rad/s
gyro noise. These are simulation checks, not hardware results.

| Stage | Terrain | Walking command | Admission result |
|---|---|---:|---|
| 0 | Flat ground, including pure turns and standing | 0.14 m/s | 78/78 passed |
| 1 | Smooth bumps, amplitude at most 2 mm | 0.10 m/s | 60/60 passed |
| 2 | Rolling ground, amplitude at most 4 mm | 0.10 m/s | 60/60 passed |
| 3 | Smooth slopes, sampled gradient bounded to 2° | 0.10 m/s | 60/60 passed |
| 4 | Raised 4 mm plateaus | 0.10 m/s | 60/60 passed |
| 5 | Raised 8 mm plateaus | 0.10 m/s | Not promoted |
| 6 | Bumps, slopes and plateaus combined | 0.10 m/s | Not reached |

At 8 mm, progress and velocity/yaw tracking failed, and some trials did not
reach enough uneven ground. A bounded three-generation CEM training attempt
also did not pass promotion. The runner kept the last admitted policy and
stopped; it did not relax the thresholds. This curriculum is executable and
partially validated, not a claim of mastered arbitrary rough terrain.

The faster, longer-stride flat-ground policy is a separate checkpoint. Terrain
admission deliberately starts from the gentler general policy; high speed is
not assumed to transfer to obstacles.

Raw results: [admission](results/terrain_admission.json) and
[three-generation training attempt](results/terrain_training.json).

## Promotion and retention

`terrain_curriculum.json` defines all stages and thresholds. Promotion requires
two consecutive batches on fresh terrain seeds, each with at least 95% of
episodes passing every gate. No falling or solver-warning episode can be
hidden by that aggregate percentage. Each rough-ground batch combines three
seeds with eight held-out translation directions and two walking turns. Pure
turns and standing are also checked on flat ground. Turning in place after
walking onto an obstacle is a useful future extension, not covered by this
first battery.

Gates include:

- At least 70% and at most 125% of requested forward progress; linear velocity
  RMSE below 0.045 m/s and yaw-rate RMSE below 0.12 rad/s.
- Stage-specific body-tilt and terrain-relative clearance variability limits;
  at least 60 mm base-frame clearance, no non-foot ground contacts or warnings.
- Stance-foot slip RMS at most 0.050 m/s, target-rate RMS at most 1.8 rad/s,
  target-acceleration RMS at most 55 rad/s², and target-jerk RMS at most
  1600 rad/s³. Actual maximum joint speed stays below 4.8 rad/s.
- At least 25% of measured samples outside the flat spawn/entry area. Waiting
  near the start cannot pass an obstacle stage.

Training evaluates three terrain tasks and one flat replay task per candidate.
The reward balances command tracking against slip, tilt, body angular rates,
terrain-relative vertical motion, and motor-target rate, acceleration and
jerk. Selection combines 70% mean return with 30% worst-case return. Every
candidate that passes a new stage must also pass the complete flat battery
before promotion. `last_promoted.json` is only replaced after those checks.

## Terrain and observation contract

Seeded heightfields cover 4 × 4 m at 10 mm grid spacing. A 0.32 m radius flat
spawn disk blends smoothly into the ground by 0.54 m. Plateaus have one-cell
edges, not perfectly vertical stair risers. The terrain report records actual
height extrema. Dimensions remain in metres; the original robot mass, joints,
collision proxies and mechanical limits are retained.

The policy receives IMU projected gravity and gyro, a velocity command and
oscillator phase. It does not receive a height map, future footholds or perfect
linear velocity. Ground rays are exclusively for reward and evaluation, and
ray/grid agreement is regression-tested against MuJoCo's collision geometry.

## Run it

Use the [standard training dependencies](requirements.txt); no custom GPU
backend is needed. From the repository root:

```sh
python -m unittest discover -s training -p 'test_*.py' -v

# Assess admission, stopping at the first unmet stage.
python training/curriculum.py --assess-only --workers 8 \
  --out training/runs/terrain-admission

# Reproduce the bounded three-generation attempt at the 8 mm stage.
python training/curriculum.py --workers 8 --population 16 --generations 3 \
  --max-stage 5 --out training/runs/terrain-trained

# A larger training budget. It still cannot bypass any promotion gate.
python training/curriculum.py --workers 8 --population 24 --generations 8 \
  --out training/runs/terrain
```

For subsequent work, increase the training budget and test touch-down timing,
clearance and IMU feedback within the existing joint limits. Once 8 mm and
mixed ground pass, add walk-to-turn transitions on obstacles, friction and
payload randomization, then gradually increase speed while keeping the same
smoothness gates. Hardware terrain trials require identified actuator dynamics
and a separate physical validation process.
