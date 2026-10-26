# Self-walking: the trained policy live on the robot

Until now the real robot played back gait frames that were planned once from the standing pose
([`plan_gaits.py`](../tools/plan_gaits.py)): an animation, blind to the ground. **Self-walking** runs the
trained policy itself ([`training/gait.py`](../training/gait.py), [`policies/omni.json`](../training/policies/omni.json))
on the Pi, every 10 ms, closed loop on the IMU, with terrain adaptation on top
([`tools/terrain_gait.py`](../tools/terrain_gait.py), runner [`tools/policy_walk.py`](../tools/policy_walk.py)).

Start it with **R3** on the gamepad or **Zelf lopen** in the cockpit; press again to stop. While it runs, the left
stick walks in any direction and the right stick turns, continuously. The robot first steps into the policy's stance
(lower, feet further out: the pose the policy was trained in), and steps back into the normal stand pose when you stop,
so stand/lie/dance keep working.

Everything below is measured in simulation. **None of it has been tried on the hardware yet.**

## What the controller adds to the policy

The policy already reads the IMU (gravity and gyro). The adaptations use only the IMU too, the same observation
contract as in training:

- **Levelling**: lasting tilt (a slope, a step) is integrated into a plane offset of the six foot heights until
  the body is level again (up to 40 mm per foot). It uses a 1 s average of the accelerometer, so the walking rhythm
  and gyro bias do not tilt the body, and it ignores the last 1° (the IMU's own error while walking).
- **Rough-ground mode**: when the body starts to shake (roll/pitch rate), the feet are lifted up to twice as high
  on a broader swing. It switches itself off again on smooth ground.
- **Fall stop**: more than 30° of tilt stops the gait.

With adaptation off (`--no-adapt`) the output is exactly the trained policy (checked to floating-point rounding).

Foot contact sensing was tried and left out: in the BAM servo model the gearbox friction carries most of the weight,
so neither the load register nor the position error separates stance from swing reliably.

## Results in simulation

[`training/terrain_adapt.py`](../training/terrain_adapt.py), reference MuJoCo + full BAM, the curriculum's own gates,
two terrain seeds × ten held-out directions per stage. The adaptive controller gets a realistic MPU6050 (accelerometer
from the real body motion, gyro noise and bias, the same complementary filter as on the Pi); the trained policy gets
the near-perfect orientation it was validated with. Raw data: [`results/terrain_adaptive.json`](../training/results/terrain_adaptive.json).

| Terrain | Trained policy: passed | speed kept | Adaptive: passed | speed kept | max tilt policy / adaptive |
|---|---:|---:|---:|---:|---:|
| Flat (incl. turning on the spot, standing) | 26/26 | 94 % | **26/26** | 94 % | 0.3° / 0.4° |
| Bumps 2 mm | 20/20 | 90 % | **20/20** | 91 % | 0.7° / 0.6° |
| Rolling ground 4 mm | 20/20 | 89 % | **20/20** | 91 % | 0.8° / 0.8° |
| Slopes 2° | 20/20 | 90 % | **20/20** | 91 % | 1.0° / 1.1° |
| Steps 4 mm | 20/20 | 85 % | **20/20** | 90 % | 1.1° / 1.2° |
| Steps 8 mm (curriculum stage the policy failed) | 3/20 | 69 % | **15/20** | 87 % | 2.4° / 2.7° |
| Mixed: bumps + steps + 2° slope | 12/20 | 78 % | **20/20** | 89 % | 3.2° / 3.2° |
| Steps 12 mm | 0/20 | 52 % | **0/20** | 82 % | 3.8° / 4.2° |
| Steps 16 mm | 0/20 | 42 % | **0/20** | 72 % | 4.1° / 5.7° |
| Bumps 8 mm | 20/20 | 86 % | **20/20** | 90 % | 1.4° / 1.4° |
| Slope 5° | 20/20 | 86 % | **20/20** | 91 % | 2.4° / 2.1° |
| Slope 8° | 17/20 | 79 % | **20/20** | 89 % | 3.6° / 3.3° |
| Cobblestones, mild | 11/20 | 75 % | **20/20** | 89 % | 2.3° / 2.4° |
| Cobblestones (joints 8 mm deep, stones ±4 mm) | 0/20 | 65 % | **1/20** | 87 % | 3.3° / 3.2° |
| Cobblestones, rough (joints 12 mm, ±7 mm) | 0/20 | 44 % | **0/20** | 77 % | 4.5° / 5.4° |
| Mixed: 10 mm + 4° slope | 0/20 | 46 % | **4/20** | 77 % | 7.8° / 5.0° |

No run fell over, with either controller. On 12-16 mm steps, normal/rough cobblestones and the 10 mm + 4° mix the
adaptive controller keeps walking (72-87 % of the commanded speed, the trained policy 42-65 %) but bobs more than the
gates allow; the clearance-rate gate also measures the ground jumping under the body at every cobble joint, so it is
strict there. So: mild cobblestones yes, normal cobblestones slowly and bumpy, rough ones barely.

**Stairs**: no. The lowest point of the chassis is 4.2 cm below the hip axis and the legs can lift the hips at most
16-18 cm above the feet, with almost straight legs. A 17-19 cm stair riser cannot be cleared; the body would have to be
dragged over the edge. Low steps (6-8 cm thresholds, kerbs) are geometrically possible with a dedicated climbing routine.

## Setting it up on the Pi

1. `tools/deploy_pi.sh` (it now also copies `training/gait.py` and `training/policies/`).
2. Measure how the IMU sits in the robot, once: on the Pi, `.venv/bin/python tools/imu_align.py` (flat, front lifted,
   left side lifted) writes `calibration/imu_map.json`. Do the first pose on a level table: it also takes out the
   accelerometer's own offset (up to ~3° on an MPU6050), and the robot will level itself to what it saw there.
   Then `sudo systemctl restart arachne-twin`.
3. **Check the L1 knee range.** `calibration/joint_limits.json` gives L1_knee only 110 ticks below its zero; the
   other knees have 240-280. The policy needs more there during the swing, so until it is re-measured L1 lifts its
   foot less (`policy_walk.py --dry` shows it: `L1_knee ... AAN DE GRENS`).
4. `.venv/bin/python tools/policy_walk.py --dry`: plans the step-in, 12 s of walking and the step-out, and checks
   joint limits and speeds without moving anything.

First trials, in this order:

```sh
.venv/bin/python tools/stand.py
.venv/bin/python tools/policy_walk.py --drive 0 0 0 --seconds 5           # step in, stand on the policy, step out
.venv/bin/python tools/policy_walk.py --drive .07 0 0 --seconds 4 --speed .5
.venv/bin/python tools/policy_walk.py --drive .14 0 0 --seconds 6
```

Then R3 / Zelf lopen. Stop: Menu on the pad, ■ STOP in the cockpit, or Ctrl-C. It also stops by itself below
10 V, above 70 °C, when the IMU or a servo drops out, or past 30° of tilt. `--no-adapt` runs the plain policy,
`--no-imu` runs it open loop (no feedback at all).

The browser simulation runs the same script: **Zelf lopen** on the sim page, or
`ARACHNE_URL=http://localhost:5870/sim .venv/bin/python tools/policy_walk.py --drive .1 0 0 --seconds 8`.

## Fall guard

[`tools/fall_guard.py`](../tools/fall_guard.py) runs inside `live_twin.py` (it owns the IMU and the bus), always,
whatever script is running:

- **Free fall** (off a table, dropped): the accelerometer reads under 0.4 g for 40 ms. Reaction **brace**: feet down
  and wide, servos soft (torque limit 35 %), so the legs absorb the landing like springs.
- **Tipping over**: more than 35° of tilt. Reaction **limp**: torque off.
- Then every moving script is stopped and servo writes are refused for 3 s, so nothing fights the landing. Once the
  robot lies still, full torque comes back (it does not move) and it can stand up with A / Opstaan.

Chosen with [`training/fall_test.py`](../training/fall_test.py): a 75 cm table, three ways to leave it (level, nose
first, sideways), 12 random falls each, servo model with torque limit and gear friction
([`results/fall_table_75cm.json`](../training/results/fall_table_75cm.json)):

| Reaction | Gear torque (STS3215 stall ~2.9 Nm) | Chassis impact, nose first | Chassis impact, sideways |
|---|---:|---:|---:|
| stiff (what it did before) | 3.2-4.6 Nm | 657 N | 988 N |
| limp | 1.2 Nm | 652 N | 628 N |
| **brace** | **1.7 Nm** | **443 N** | **643 N** |

A fall that starts with a forward or sideways tumble cannot be turned around in 0.4 s; the brace mainly protects
the gears and softens the hit.

## Picking things up (Pakken)

[`tools/grab.py`](../tools/grab.py): L2 and L5 step forward, the body leans back over L2-L5, L1 and L6 lift like arms,
reach around the object, squeeze it with soft yaw servos (40 %) and lift it. Pressing again puts it down. Put the
object about 20 cm in front of the body centre (`--at`). The front legs swing at most ~45° inwards, so the toes
cannot come closer than ~12 cm: it holds things **12-15 cm wide** (a small box, a large ball, a mug).

[`training/grab_test.py`](../training/grab_test.py), physics of the browser simulation: 12-15 cm boxes up to 250 g,
friction down to 0.4, lifted about 4.5 cm and held, maximum body tilt 2°; on four legs the centre of mass stays
6.7 cm inside the support. An 11 cm box is too narrow to grip ([`results/grab_box.json`](../training/results/grab_box.json)).

## Tests

```sh
python -m unittest discover -s training -p 'test_*.py'      # includes test_live.py
python training/terrain_adapt.py --check                    # adaptation off = the trained policy
python training/terrain_adapt.py --stages higher_obstacles --extra --workers 4
python training/fall_test.py --height .75
python training/grab_test.py --width .13 --mass .1
```
