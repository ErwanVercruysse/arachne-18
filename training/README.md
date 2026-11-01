# Smooth, IMU-aware locomotion

> **Mechanical revision v4:** checkpoints, recorded metrics and videos below are historical pre-v4 results. No RL retraining was performed. Current model/IK adapters use v4 geometry; prior performance is not established for v4. See [revision notes](../docs/revision-v4.md).


**Latest:** [faster walking with longer strides](LONG_STRIDE.md), including
a 44-second video and 26/26 reference checks. The new forward gait reaches
22.4 cm/s with 43% greater foot excursion than the original solo gait.
The [uneven-ground curriculum](TERRAIN.md) has passed admission through
4 mm obstacles and stops before the unmastered 8 mm stage.

ARACHNE now has a learned command-conditioned walking policy, independent BAM
validation, and real physics recordings. Start with
[`policies/omni.json`](policies/omni.json): it walks forward, backward, sideways,
diagonally, turns both ways, combines translation and rotation, and stops.

This is **episodic reward-based policy search using the cross-entropy method
(CEM)** over a 12-parameter controller. It is not a trained neural network or
PPO. The tripod pattern and inverse kinematics are explicit priors; training
learns cadence, stance fraction, foot clearance, stride/yaw/lateral gains,
body height, stance radius, IMU feedback strength, and phase corrections.

## Recorded results before the symmetric-leg revision

These archived verification figures used the original collision proxies and
the complete BAM M6 runtime at a 1 ms physics step. The symmetric-leg CAD
revision subsequently updated `simulation/arachne.xml`. Gait, terrain and jump
unit tests pass with the revised model, and the general walking policy passed
a five-second BAM smoke run; the full performance table below has not yet been
remeasured on that geometry. Training used a separate approximate model.

| Case | Measured motion | Body tilt RMS | Height variation, standard deviation |
|---|---:|---:|---:|
| General policy, 14 cm/s forward command | 13.29 cm/s | 0.060° | 0.395 mm |
| General policy, 20 cm/s forward command | 19.3 cm/s | 0.064° | See validation JSON |
| Fast policy, 20 cm/s forward command | 18.98 cm/s | 0.068° | 0.431 mm |
| Continuous 40-second direction/turn/stop demo | Multiple commands | 0.084° | 0.376 mm |

The fast policy trades a small amount of speed for less foot slip: 2.36 cm/s
RMS versus approximately 3.7 cm/s for the general policy at the 20 cm/s command.
Use the general policy for arbitrary directions; the fast policy was optimized
and verified for forward motion only.

**26/26 held-out/reference checks passed**, including 16 directions (eight
intermediate directions were not in the training command set), both yaw signs,
a combined command, slow/fast movement, standing, friction 0.5/1.1, a 200 g
payload, and 0.25° orientation-estimate noise with 0.005 rad/s gyro noise. Each
case lasts 12 seconds. The 40-second continuous demo has no falls, no non-foot
ground contacts, no solver warnings, and a maximum tilt of 0.297°.

Reports: [`results/validation.json`](results/validation.json),
[`results/fast_bam.json`](results/fast_bam.json),
[`../assets/arachne-learned-omni.json`](../assets/arachne-learned-omni.json).
These are simulation results, with the repository's provisional 12 V BAM
approximation. A separate [uneven-ground curriculum](TERRAIN.md) now has bounded
simulation checks. Hardware, impacts, thermal limits and measured actuator
parameters have not been validated by this work.

## Why the movement is smooth

- Alternating support tripods have coordinated phases. The stance/swing
  horizontal paths join with continuous position, velocity and acceleration;
  the `sin^4` foot lift also has zero velocity and acceleration at touchdown.
- A five-degree polynomial ramps motion in; a 250 ms command filter blends
  changes of direction. Cadence and foot clearance decrease at lower speeds.
- Joint targets pass through a low-pass filter and a 4.717 rad/s slew limit,
  then stay inside the original ±20° yaw and ±15° hip/knee limits.
- IMU projected gravity controls body tilt. A short prediction using body-frame
  gyro adds damping. The controller never observes global position, absolute
  heading or simulator ground-truth linear velocity. Ground-truth velocity is
  used only by the training reward and evaluation metrics.

The original policy uses this running reward:

```text
+ 5.0 exp(-||v_xy - command_xy||² / 0.0016)
+ 1.5 exp(-(yaw_rate - command_yaw)² / 0.09)
- 90 tilt_xy²
- 20 ||roll_pitch_rate||²
- 80 vertical_velocity²
- 400 (body_height - 0.095)²
- 15 stance_foot_slip²
- 0.025 mean(target_rate²)
- 0.00015 mean(target_acceleration²)
- 0.025 mean(motor_torque²)
- 30 for a fall, excessive tilt/height or non-finite state
```

Search maximizes average return after a 1.5 s settling period. Omnidirectional
selection uses 70% mean command return plus 30% worst command return. The top
eighth of the population updates the sampling distribution, with variance
floors and a retained incumbent. There is no bonus for arbitrary speed, so
overshooting or standing still cannot replace command tracking.

The newer longer-stride experiment adds measured progress per cycle (capped
at 0.12 m), stronger slip penalties, and a target-jerk penalty. Its exact
weights and run configuration are stored alongside the results. The original
policy and its validation report remain available for comparison.

## Run the trained controller or render the videos

Python 3.12, from the repository root, in a separate environment:

```sh
python3.12 -m venv .venv-walk
source .venv-walk/bin/activate
python -m pip install -r training/requirements.txt
python training/test_gait.py
python training/evaluate.py --policy training/policies/omni.json --command .14 0 0
python training/validate_policy.py --policy training/policies/omni.json
python training/render_demo.py --policy training/policies/omni.json
python training/evaluate.py --policy training/policies/fast.json --command .20 0 0 --seconds 12
python training/render_army.py --run training/recordings/army
```

Evaluation, tests, and rendering need standard MuJoCo and BAM; they do not need
the custom Metal physics library. The HUD is telemetry over a real simulation
capture. macOS rendering needs a graphics-capable session.

## Train with the fast local simulator

Training requires the **fused** `_mjmlx_native` build of MuJoCo-MLX-Cpp. This run
used the existing local `metal-opt` branch, commit
`4d84f84871f59378fa9f5636cf40efe16ed3f321`, on an Apple M4 Max. At publication time
that local fused implementation is not on the public upstream `main` branch;
cloning public main alone does not provide this training backend. The scripts
check support and fail explicitly if the necessary fused API/model support is
missing. Point `PYTHONPATH` to the local build with a matching Python/MLX ABI.

```sh
export PYTHONPATH=/path/to/MuJoCo-MLX-Cpp/build
python training/build_model.py
python training/check_metal.py

# First train and independently verify forward walking.
python training/metal_search.py --population 256 --generations 30 --speed .14 --out training/runs/forward
python training/evaluate.py --policy training/runs/forward/policy.json --command .14 0 0

# Then train eight directions, turns, a curve, and standing together.
python training/metal_search.py --stage omni --population 128 --generations 30 --speed .14 \
  --resume training/runs/forward/policy.json --out training/runs/omni
python training/validate_policy.py --policy training/runs/omni/policy.json

# Optional faster forward policy, preserving the smoothness objective.
python training/metal_search.py --population 256 --generations 25 --speed .20 \
  --resume training/runs/omni/policy.json --out training/runs/fast

# Record genuine population rollouts for the army video.
python training/metal_search.py --population 128 --generations 16 --seconds 8 --speed .14 \
  --capture-generations 0 5 15 --out training/runs/army
python training/render_army.py --run training/runs/army
```

The published weights came from an initial forward exploration, a second
forward smoothness pass, then the omni and fast stages. The current commands
use the final reward from the beginning and retrain a new run, not a bitwise
reproduction of the earlier exploration. Seeds, generation counts, returns,
timings and controller hash are in
[`results/training_summary.json`](results/training_summary.json).

The omni stage evaluated 1,536 environments per generation. It completed
27.648 million control steps / 138.24 million physics substeps in approximately
132 seconds, including logging and local background work. This is an observed
run timing, not an isolated benchmark.

## Model approximation and validation boundaries

`build_model.py` preserves CAD inertias, kinematics, mass and joint limits.
For the fused solver it uses Euler at 2 ms, pyramidal friction, 26-vertex foot
hulls and a torso hull. Training excludes self-collision and uses static
BAM-derived proportional gains, rotor inertia, damping/back-EMF and Coulomb
friction. It includes the external target filter, slew limit and 10 ms target
delay; the original reference adds BAM's approximately 5 ms command delay and
dynamic load-dependent friction. Foot slip during training uses a height-based
contact estimate; reference reports use actual ground contacts.

The independent 32-environment, five-substep comparison against MuJoCo C on the
same approximate model gave maximum errors of `9.04e-8` in qpos and `2.51e-6`
in qvel. This tests the backend for those states; it does not make the approximate
model identical to the full BAM model. Final policy validation and all solo
video physics use the original model and complete BAM runtime.

## Training-army provenance

The army video shows **128 independent candidates**, not copies of one trained
animation. The three clips replay actual qpos tensors recorded during
generations 0, 5 and 15 of a new training run (displayed as generations 1, 6 and
16). Original structural CAD meshes are positioned from those tensors in a
MuJoCo renderer. Small hardware is omitted in the distant overview.

Grid placement is a display offset: the training worlds do not collide with
one another. Gold shells mark the elite candidates selected from the completed
generation. Playback is at simulation time; the title does not claim rendering
ran live at training speed. Raw recordings include candidate parameters,
commands, rewards and metrics in [`recordings/army/`](recordings/army/), with
SHA-256 hashes in the [video manifest](../assets/arachne-training-army.json).
