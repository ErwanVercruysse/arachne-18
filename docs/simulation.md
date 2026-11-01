# Simulating v4

The canonical model is `simulation/arachne.xml`. It is a floating-base, 19-link robot with 18 hinges. The fixed-base model `arachne_fixed.xml` suspends the robot for joint sweeps; it is not used for the grounded animation.

## Setup and checks

Install Python 3.12 and `simulation/requirements.txt` in a virtual environment. No CAD software, Onshape access, API keys or Git LFS are needed to simulate the committed files.

```sh
python simulation/simulate.py --headless --seconds 10 --voltage 11.1
python simulation/validate.py
python simulation/grounded_demo.py --headless --seconds 10
```

For an interactive viewer use `mjpython simulation/grounded_demo.py --seconds 300` on macOS, or `python` on Linux/Windows. Run `python simulation/simulate.py --help` for standing and fixed-base sweep options. A window may need a working graphics driver; headless physics does not need a renderer.

## Joint convention

`joint_map.json` is authoritative. Servo IDs 1–18 follow `L1_yaw, L1_hip, L1_knee, L2_yaw, … L6_knee`. Coordinates use metres, kilograms, seconds and radians. +X is forward, +Z is up; the six leg roots lie at 45°, 90°, 135°, 225°, 270°, 315° around the body.

All joint positions are **offsets from the assembled neutral pose**, not absolute servo encoder values. Neutral hip angle is +20° and knee angle is −85°. Coxa length is 62 mm, hip vertical offset −6.95 mm, and femur length 78 mm. In `q=0`, all CAD and MuJoCo link frames coincide; link axes are aligned with the world axes. Pitch axes are the local negative Y direction, expressed in world coordinates.

| Joint | MuJoCo offset limits | Absolute mechanical angle |
|---|---|---|
| Front/rear yaw | 60° inward, 35° outward | Same as offset |
| Middle yaw | ±35° | Same as offset |
| Hip | −50° to +25° | −30° to +45° |
| Knee | −20° to +100° | −105° to +15° |

The front-pair toe-touch example has L1 yaw −57.50946°, L6 yaw +57.50946°, hip offset −40° and knee offset +85°. Rear uses L3 positive yaw and L4 negative yaw. The checked path linearly interpolates all three angles together from neutral over 13 samples. Keep the other legs neutral. Individual axis limits do not imply that every multi-leg combination is collision-free.

## Actuation and contacts

`simulate.Robot` runs BAM at a 1 ms timestep. `robot.step(target)` accepts 18 joint targets in radians, applies the approximate 12 V STS3215 firmware/motor/friction model and advances one physics tick. For a 50 Hz policy, hold the target for 20 ticks. Reset with `robot.reset()` so the firmware target and delay history reset too.

The `arachne.xml` actuators accept **torque**, not position. Loading the XML in MuJoCo without the Python controller bypasses BAM; sending joint angles straight to those actuators is incorrect.

Foot contacts use the convex hull of the actual TPU shoe meshes. A foot can generate multiple simultaneous contact points, so support checks count distinct feet rather than demanding exactly six points. The v4 grounded exercise produces 12 points on six feet in the tested run. The base is free and settles from 2 mm above the CAD sole plane.

Servo cases have box collision proxies, the canopy an ellipsoid proxy, and lower links simplified capsules. Printed visual meshes are not all used for self-collision. These approximations cannot certify CAD clearance; use the CAD reports for that. Cables, TPU compliance, backlash and structural flex are not modelled. Total estimated mass is 2.333 kg; substitute actual sliced/measured part and payload masses before hardware transfer.

The 12 V BAM approximation is retained unchanged from the prior release. See [actuator assumptions](bam-model.md).

## Training compatibility

`training/gait.py` now reads the v4 geometry from `joint_map.json`. `training/build_model.py` rebuilds the fast approximate training model with v4 inertias, limits and rotated shoe contact boxes. That model uses static BAM-derived coefficients and omits several effects of the full controller. Final policy evaluation should use `simulation/simulate.py`.

```sh
python training/build_model.py
python -m unittest discover -s training -p 'test_*.py'
```

These commands build and test; they do not train. All 17 geometry/controller/terrain tests passed for this release. Existing policy files and recorded results remain historical; no v4 locomotion performance has been established and no RL retraining was requested or performed. See [training history](../training/README.md).
