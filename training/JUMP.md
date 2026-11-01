# Maximum whole-robot jump clearance

> **Mechanical revision v4:** checkpoints, recorded metrics and videos below are historical pre-v4 results. No RL retraining was performed. Current model/IK adapters use v4 geometry; prior performance is not established for v4. See [revision notes](../docs/revision-v4.md).


[Watch the jump](https://tom-michiels.github.io/arachne-18/#jump).

Two separate single-jump policies are available:

- **[jump.json](policies/jump.json):** the highest clearance found in the standard
  1 ms MuJoCo/BAM model. At the recorded apex, the entire collision model clears
  the ground by **2.51 cm**. An independent check of every rendered CAD mesh
  vertex gives **2.40 cm** clearance for the lowest actual model part.
- **[jump_robust.json](policies/jump_robust.json):** selected for consistent
  clearance across 1, 0.5 and 0.25 ms physics timesteps: **2.08, 2.04 and 2.01 cm**
  respectively in the collision model.

These are the best candidates found in this finite search, not a proof of the
mechanism's absolute maximum. The walking policies remain separate.

## Objective

At each sample, compute the lowest world-space point of every robot collision
geometry, including all six feet, shins, motor housings and the body. Define
`clearance(t)` as the minimum of those heights above the ground plane.

The main reward is the maximum clearance sustained across three consecutive
10 ms samples, with no ground contact and at least 1 mm clearance throughout.
This gives a 20 ms persistence requirement and excludes the initial settling
period. A robot with one grounded foot cannot earn a positive jump reward.
**There is no upright, yaw, tilt or landing-posture reward.** Tilt is recorded
for diagnosis only. Before a candidate gets airborne, an upward-velocity shaping
term supplies a nonpositive search score; it cannot outrank a valid jump.

This metric measures clearance under the whole robot, not the change in body
height from its initial standing pose. The robot can improve it by pushing off
and retracting its legs. Original CAD joint ranges, gravity, robot masses and
BAM servo dynamics remain in place. Actuator targets use the existing 100 Hz
filter and rate limit.

## Controller and search

The timed policy waits 0.6 s in the neutral state, crouches, holds briefly,
pushes off, retracts the legs, holds the tuck and then blends back toward neutral.
Four learned durations and three learned yaw/hip/knee poses define the selected
13-parameter policy. Its JSON includes parameter names and control constants.
It uses elapsed time from the jump command; it assumes the neutral starting
state and does not use ideal simulator position or velocity as actor inputs.
It currently has no IMU feedback. Landing recovery has not been optimized.

CEM searched 20 generations of 128 symmetric candidates, then 18 generations
with independently parameterized legs and 18 further symmetric generations.
The asymmetric search did not beat the symmetric result. Coordinate refinement
improved the maximum-height candidate. A second refinement maximized the worst
sustained clearance across three physics timesteps, producing the robust policy.
This is learned parameter search over a motion prior, not a neural PPO policy.

## Reference checks

Peak collision-model clearance, in centimetres:

| Condition | Maximum-height policy | Robust policy |
|---|---:|---:|
| Standard 1 ms timestep | 2.51 | 2.08 |
| 0.5 ms timestep | 1.94 | 2.04 |
| 0.25 ms timestep | 1.84 | 2.01 |
| Friction 0.5, 1 ms | 2.02 | 1.92 |
| Friction 1.1, 1 ms | 1.91 | 1.86 |
| Additional 200 g, 1 ms | 1.57 | 2.14 |

All six conditions get the whole collision model airborne without solver
warnings. These are deterministic model checks, not a statistical hardware
success rate. Timestep sensitivity is substantial for the maximum-height policy;
use the robust variant when consistency across these numerical settings matters.
The numerical checks do not establish real-world actuator or landing performance.

The maximum-height recording has approximately 0.13 s of airborne samples,
3.82 rad/s maximum joint speed and 3.76 Nm maximum modeled motor torque. At the
clearance apex, body tilt happens to be only 0.20°; staying upright was not rewarded.

- [Maximum-height checks](results/jump_validation.json)
- [Robust-policy checks](results/jump_robust_validation.json)
- [Search settings and histories](results/jump_training.json)
- [Film metrics, CAD clearance audit and policy hash](../assets/arachne-jump.json)

## Use and reproduce

From the repository root, with the existing MuJoCo/BAM environment:

```sh
python training/test_jump.py
python training/check_jump.py
python training/check_jump.py --policy training/policies/jump_robust.json \
  --out training/results/jump_robust_validation.json
python training/render_jump.py
```

The film shows 3.5 seconds at real time, followed by a **labelled 4× slow replay**
of exactly the same state recording. The total video duration is 17.5 seconds.
The physics states and targets are saved in `assets/arachne-jump.npz` at 100 Hz.

For further search, copy checkpoints you want to keep: these commands write
`training/policies/jump.json` as they find candidates.

```sh
python training/jump.py --generations 20 --population 128 --std-floor .025
cp training/policies/jump.json training/runs/jump/symmetric.json
python training/jump.py --resume training/runs/jump/symmetric.json --asymmetric \
  --generations 18 --population 128 --out training/runs/jump-asymmetric
cp training/runs/jump/symmetric.json training/policies/jump.json
python training/refine_jump.py
```

`refine_jump.py` expects a 13-parameter symmetric checkpoint and refines against
all three timesteps. Select that checkpoint before invoking it after an
asymmetric search. `target(parameters, elapsed_seconds)` in `jump.py` provides
18 motor targets; apply the filter, delay and BAM dynamics shown in the evaluator.
