# Faster forward walking with larger steps

> **Mechanical revision v4:** checkpoints, recorded metrics and videos below are historical pre-v4 results. No RL retraining was performed. Current model/IK adapters use v4 geometry; prior performance is not established for v4. See [revision notes](../docs/revision-v4.md).


[Watch the first video](https://tom-michiels.github.io/arachne-18/#fast).

The new forward gait reaches **29.9 cm/s**, compared with **22.4 cm/s** in the
previous first film. Actual foot excursion increases from **4.90 to 6.45 cm**.
That is **34% more speed and 32% larger steps**, while cadence decreases 1.9%.
The 20-second film shows continuous acceleration, forward walking and stopping.
Playback follows simulation time; the video is not sped up.

## Reference measurements

Both policies use the original MuJoCo model, original joint limits and full BAM
servo dynamics. Each comparison runs for 12 seconds, excluding the first two
seconds from steady-state averages. Foot excursion measures body-relative
peak-to-peak travel along the command direction, averaged across all six feet
and complete cycles. It measures actual motion, not just commanded step length.

| Metric | Previous first film policy | New first film policy |
|---|---:|---:|
| Forward command | 0.26 m/s | 0.34 m/s |
| Measured forward speed | 22.38 cm/s | 29.88 cm/s |
| Actual foot excursion | 4.90 cm | 6.45 cm |
| Body travel per cycle | 9.31 cm | 12.67 cm |
| Cadence | 2.40 Hz | 2.36 Hz |
| Body tilt RMS | 0.105° | 0.212° |
| Body height standard deviation | 0.509 mm | 0.623 mm |
| Stance-foot slip RMS | 4.37 cm/s | 5.36 cm/s |
| Motor-target jerk RMS | 2797 rad/s³ | 2884 rad/s³ |

The higher speed increases slip and body motion slightly. It remains within the
existing faster-gait limits; those limits were not relaxed. In the actual film,
body tilt stays below **0.51°**, height standard deviation is **0.58 mm**, and
there are no falls, non-foot ground contacts or solver warnings. The complete
film, including its transitions, also passes the faster-gait gates.

## Foot trajectory and learning

The controller adds one optional parameter to the existing IMU-aware gait:
a blend between the original swing-height curve `sin(pi*u)^4` and the wider
curve `64*u^3*(1-u)^3`, where `u` runs from zero to one during swing. Both curves
join the stance phase with zero velocity and acceleration. The wider curve
lifts the foot earlier and lowers it later. The selected blend is 0.9895.
Horizontal foot trajectories, joint limits, command filtering and servo limits
remain unchanged. Existing 12-parameter policies retain their original output.

CEM searches cadence, step gain, lift, body height, phase offsets and feedback,
then the added foot-arc blend. The final search evaluates 160 candidates per
generation for 10 generations in parallel reference MuJoCo/BAM simulations.
Its reward combines measured speed and actual foot excursion, with squared
penalties for exceeding the existing tracking, tilt, slip, acceleration, jerk
and joint-speed gates. Final selection uses longer 12-second checks, including
payload and IMU noise. This is compact policy search with a gait prior, not PPO.

## Validation and operating range

**26/26 reference checks pass**: 16 translation headings at a **0.24 m/s command**,
one forward case at **0.34 m/s**, slow motion, both turning directions, a walking
turn, standing, and friction/payload/IMU perturbations at **0.24 m/s**. The
maximum forward speed is not a claim about every heading or surface.

Separate tests at the **0.34 m/s command** pass for the nominal surface, 200 g
payload and IMU noise. Friction 0.5 fails the unchanged slip limit; friction 1.1
fails the velocity-tracking limit. These failures are retained in the comparison
report. Use the lower command for broader operating conditions. The terrain
films continue to use the separate general policy. Hardware performance has
not been validated.

- [Full comparison and high-speed perturbations](results/sprint_comparison.json)
- [26-case reference validation](results/sprint_validation.json)
- [Search settings and generation summaries](results/sprint_training.json)
- [Recorded film metrics and policy hash](../assets/arachne-faster-stride.json)
- [Previous longer-stride report](LONG_STRIDE.md)

## Reproduce

Use the existing [MuJoCo/BAM setup](README.md), then:

```sh
python training/test_gait.py
python training/benchmark_speed.py --policy training/policies/sprint.json \
  --speed .24 --forward-speed .34 --out training/results/sprint_validation.json
python training/render_sprint.py
```

To reproduce the broad-lift search from its saved initial mean:

```sh
python training/train_sprint.py --resume training/policies/sprint_search_start.json \
  --command .34 --generations 10 --population 160 --out training/runs/sprint
```

The highest six-second search score is a candidate, not an automatically accepted
replacement: use the longer reference battery and perturbation checks before
selecting a new controller. The renderer also saves per-control-step states in
an adjacent `.npz` file, allowing the motion and timing to be inspected locally.
