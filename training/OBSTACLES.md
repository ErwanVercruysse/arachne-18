# Stones, rocks and bending grass

> **Mechanical revision v4:** checkpoints, recorded metrics and videos below are historical pre-v4 results. No RL retraining was performed. Current model/IK adapters use v4 geometry; prior performance is not established for v4. See [revision notes](../docs/revision-v4.md).


Uneven terrain includes discrete obstacles as well as height variations. The
reference simulator now includes individual collision stones and passive,
bending grass tufts. [Watch the obstacle films](https://tom-michiels.github.io/arachne-18/#terrain-pebbles).

## Physical models

- **Stones:** individually placed, partly buried ellipsoids with random size,
  height, orientation and position. They are fixed to the ground in this first
  model; rolling or displaced stones are a later extension.
- **Grass:** seven capsule-shaped collision blades per tuft, mounted on two passive
  spring-and-damper hinges. Contact bends the tuft and the spring restores it.
  Tufts interact with the robot and ground, but not with each other.
  This is an uncalibrated mechanical proxy, not a botanical or soft-soil model.
- **Mixed obstacles:** both models in the same scene. Heights are actual metres;
  the videos do not exaggerate them. A clear spawn area prevents initial overlap.

The actor still receives IMU gravity and angular velocity, commands and gait
phase. It has no obstacle map or camera. This tests a gait's ability to tolerate
contact; it does not yet demonstrate visual route planning or deliberate
selection of individual footholds. The added objects run in reference MuJoCo
with full BAM servo dynamics. They are not represented by the experimental
Metal heightfield backend.

## Curriculum

[obstacle_curriculum.json](obstacle_curriculum.json) is a separate progression,
so the earlier heightfield results retain their original meaning:

| Stage | Physical obstacle | Command speed |
|---|---|---:|
| Flat admission | Original plane | 14 cm/s |
| Small pebbles | 4–8 mm protruding stones | 10 cm/s |
| Short grass | 18–30 mm stem lengths | 10 cm/s |
| Larger stones | 8–18 mm protruding stones | 10 cm/s |
| Stones and grass | 8–18 mm stones with 20–40 mm grass | 10 cm/s |
| Tall grass | 30–60 mm stems | 10 cm/s |
| Large rocks | 25–50 mm protruding rocks | 10 cm/s |

These are curriculum targets, not a claim that every stage is mastered.
Promotion uses two independent seed batches, intermediate headings, walking
turns, IMU noise and flat-ground retention after refinement. Training replays
25% flat-ground tasks. It rewards progress and tracking while penalizing tilt,
bouncing, slipping and abrupt motor motion. Failed promotion preserves the last
accepted policy. Future work on large rocks may need more expressive control,
contact sensing and perception rather than only a higher open-loop leg lift.

Encounter gates additionally require recorded foot–stone contact or vegetation
contact; walking through an empty patch cannot pass solely through forward
progress. Solid stone contact is included in support-foot slip and non-foot
collision checks. Flexible grass contact is tracked separately: brushing a
blade is not treated as striking the body against a rock. Grass joints are
excluded from robot motor-speed metrics, and grass tips do not redefine the
load-bearing ground-clearance reference.

## Dense grass update

The current grass film replaces the sparse initial example. Tuft spacing is
**3 cm instead of 16 cm**, and each tuft carries **seven blades instead of three**.
That increases nominal density from 117 to **7,778 collision blades per square
metre**, about **66 times** the initial density. The rendered patch contains
987 passive tufts and 6,909 physical blades. The grass stages now explicitly use
this density; omitted parameters still reproduce the original sparse scenes.

The forward film uses a bounded patch from x=0.30 to 1.30 m and y=−0.48 to 0.48 m,
with the existing spawn exclusion and radial boundary. This bounds the physics
cost without adding decorative, non-colliding filler. Curriculum scenes retain
the full surrounding field for multiple headings. Each tuft shares two passive
hinges across its seven blades.

All three dense forward layouts (seeds 701–703) pass their gates. The film's
seed 701 records 884 control steps with vegetation contact, versus 98 in the
old sparse film. It reaches 7.48 cm/s with 0.155° body-tilt RMS. Grass resistance
now measurably slows the robot. These tests are not full stage promotion.
[Dense-grass validation](results/dense_grass_validation.json).

## Initial sparse-scene results

The following measurements predate the density increase; the saved report
contains the original specifications.

The existing general controller was evaluated on three layouts per obstacle
class, using a 10 cm/s forward command for 12 seconds:

| Obstacle class | Passed examples | Result |
|---|---:|---|
| Small pebbles | 3/3 | 8.37–8.77 cm/s actual speed; 0.43–0.62° tilt RMS |
| Short grass | 3/3 | 9.11–9.15 cm/s actual speed; 0.06–0.08° tilt RMS |
| Larger stones | 0/3 | Tracking and clearance-rate limits exceeded |
| Stones and grass | 0/3 | Tracking and clearance-rate limits exceeded |

The pebble film and the earlier sparse grass film use seed 701 at simulation
time. They record 486 foot–stone and 98 vegetation contact steps respectively.
Both pass their gates. The current denser grass film is described above. These
forward examples are not equivalent to
full curriculum promotion. Larger obstacles remain unmastered; failed trials
are retained in the [assessment report](results/obstacle_admission.json).

The subsequent full admission check passes both flat batches. For small stones,
29/30 and 28/30 held-out cases pass; the second batch falls below the required
95%. Three cases exceed velocity-tracking limits. No stone-stage promotion is
claimed, and later stages were not attempted by that admission run.
[Full admission results](results/obstacle_curriculum_admission.json).

## Reproduce

```sh
python -m unittest discover -s training -p 'test_*.py'
python training/assess_obstacles.py  # current, denser curriculum specifications
python training/assess_dense_grass.py
python training/render_obstacles.py
python training/curriculum.py --config training/obstacle_curriculum.json \
  --assess-only --max-stage 6 --out training/runs/obstacle-admission
```

To refine a failing stage after admission checks, omit `--assess-only` and set
a training budget, for example `--generations 8 --population 24`. The curriculum
stops at the first stage that cannot be promoted. Tall grass and large rocks
are proposed later stages, not validated capabilities. Spring stiffness,
damping, stem geometry and terrain friction need measurements before this
proxy can predict performance in real grass.
