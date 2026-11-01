# Faster walking with longer strides

> **Mechanical revision v4:** checkpoints, recorded metrics and videos below are historical pre-v4 results. No RL retraining was performed. Current model/IK adapters use v4 geometry; prior performance is not established for v4. See [revision notes](../docs/revision-v4.md).


This report records the previous first film. The [latest forward gait](SPRINT.md)
reaches 29.9 cm/s with larger steps.

The `policies/long_stride.json` reaches **22.4 cm/s** in the original MuJoCo
model with full BAM M6 servo dynamics. Compared with the original 13.3 cm/s
solo gait, speed increases **68%**, actual foot excursion increases **43%**,
and cadence increases only **11%**. Original joint limits remain unchanged.

[Watch this earlier recording](../assets/arachne-long-stride.mp4).

## Measured comparison

Each reference case runs for 12 simulated seconds, with the first two seconds
excluded from steady-state averages. Foot excursion is the mean peak-to-peak
body-relative foot travel along the command direction across all six feet and
complete cycles. Body travel per cycle is measured speed divided by cadence;
slip is checked separately, so a long commanded trajectory alone is not evidence
of a longer useful stride.

| Metric | Original solo gait | Previous fast forward gait | New longer-stride gait |
|---|---:|---:|---:|
| Forward command | 0.14 m/s | 0.20 m/s | 0.26 m/s |
| Measured forward speed | 13.29 cm/s | 18.98 cm/s | 22.38 cm/s |
| Foot excursion | 3.42 cm | 4.33 cm | 4.90 cm |
| Body travel per cycle | 6.12 cm | 7.97 cm | 9.31 cm |
| Cadence | 2.17 Hz | 2.38 Hz | 2.40 Hz |
| Body tilt RMS | 0.060° | 0.068° | 0.105° |
| Body height standard deviation | 0.395 mm | 0.431 mm | 0.509 mm |
| Stance-foot slip RMS | 2.23 cm/s | 2.36 cm/s | 4.37 cm/s |
| Target jerk RMS | 1294 rad/s³ | 2028 rad/s³ | 2797 rad/s³ |

The faster gait has a measured smoothness tradeoff: slip and target jerk are
higher than in the gentle original gait. It stays within the declared faster
gait limits and keeps a steady body. The 68% comparison is against the original
solo gait, not against the previous 20 cm/s command test.

## Verification

**26/26 checks pass** for the final checkpoint: 16 translation headings at a
0.24 m/s command, 0.26 m/s straight ahead, slow movement, both turning signs,
a walking turn, standing, friction 0.5 and 1.1, a 200 g payload, and 0.25°
orientation-estimate noise with 0.005 rad/s gyro noise. Sideways and diagonal
measured speeds are lower than the forward maximum; see the per-case report.

The predeclared faster-gait bounds are velocity RMSE < 0.065 m/s, yaw-rate RMSE
< 0.12 rad/s, body tilt RMS < 0.75°, stance-foot slip RMS < 0.055 m/s, target
acceleration RMS < 60 rad/s², target jerk RMS < 3000 rad/s³ and actual maximum
joint speed < 4.8 rad/s, with no falls, non-foot ground contacts or warnings.
These bounds are distinct from the stricter, slower [terrain gates](TERRAIN.md).
Some cases are close to their limits, so this is a bounded simulation result,
not evidence of hardware robustness or unrestricted operating speed.

Several commands were used during checkpoint selection; the 26 checks are a
reference verification battery, not 26 entirely unseen learning tasks.

- [Final validation and policy hashes](results/long_stride_validation.json)
- [Checkpoint selection, including rejected candidates](results/stride_selection.json)
- [Training configuration, weights and return histories](results/long_stride_training.json)

## Training and selection

The fast fused MuJoCo/Metal simulator first searches forward motion and then
eight directions, turns, a curve and standing. Compared with the original
reward, slip weight increases from 15 to 80, target rate/acceleration costs
are multiplied by 1.5, target jerk receives weight `8e-8`, and measured
progress per cycle receives a bonus capped at 0.12 m. The stride term uses
actual progress, rather than the desired foot position.

The static training servo approximation did not transfer every motion cleanly
to BAM. Parallel reference-model CEM therefore refines the GPU proposal,
including opposite directions and low friction. Its reward gives yaw tracking
more weight and penalizes slip/jerk threshold violations. Final selection then
checks the saved CEM checkpoints and a small 27-point cadence/lift/stride grid
against hard reference gates. The highest training return alone did not choose
the published policy. The chosen checkpoint is from reference generation 2,
with cadence multiplied by 1.01 and stride gain by 0.96.

All of this remains a compact 12-parameter controller with a tripod and inverse
kinematics prior. It is CEM policy search plus a local parameter grid, not PPO
or a neural policy. The actor uses the same IMU-only body feedback as before.

## Reproduce

Use the [training setup](README.md) and the recorded run configs. Evaluation
and video rendering need only the standard MuJoCo/BAM dependencies:

```sh
python training/benchmark_speed.py --policy training/policies/long_stride.json --speed .24
python training/render_demo.py --policy training/policies/long_stride.json \
  --fast --out assets/arachne-long-stride.mp4
```

The training chain is recorded in `results/long_stride_training.json`.
After the GPU stages, the reference refinement and selection commands are:

```sh
python training/refine_bam.py --resume training/runs/long-stride-omni/policy.json \
  --out training/runs/bam-refinement
python training/refine_bam.py --resume training/runs/bam-refinement/policy.json \
  --robust --speed .24 --generations 8 --out training/runs/bam-robust
python training/select_stride.py --runs training/runs/bam-refinement \
  training/runs/bam-robust --local-grid
```

The 44-second continuous recording uses the selected checkpoint, a smooth
command filter and native simulation-time playback. It contains startup,
acceleration, sideways/backward/diagonal walking, turning and stopping.
Lighting changes affect rendering only. No frame retiming or robot animation
is used. Hardware performance still depends on identified servo dynamics,
mechanical tolerances and physical validation.
