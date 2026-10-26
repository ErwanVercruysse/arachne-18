<div align="center">

# ARACHNE / 18

**Six legs. Eighteen actuators. A compact, printable robot with an organic shell.**

![ARACHNE 18 — workbench presentation artwork based on the CAD design](assets/hero-workbench.png)

**18 × STS3215 12 V** · **3 DoF per leg** · **3S battery bay** · **Onshape + MuJoCo + BAM**

[Explore the Onshape assembly](https://cad.onshape.com/documents/88edbeda4232642329938c14/w/19f5435855c2974f02fa7c68/e/ea6ef29b6f90442d8a9d563b) · [Print & build](docs/build-guide.md) · [Run the simulation](docs/simulation.md) · [Browse the CAD](cad)

</div>

> **Deze fork (KVR-CIBER):** het ontwerp, de CAD en de simulatie hieronder zijn van [Tom Michiels](https://github.com/Tom-Michiels/arachne-18). Kobe heeft de robot echt gebouwd en er deze software voor geschreven, samen met [Claude Code](https://claude.com/claude-code).
>
> | Wat | Waar |
> |---|---|
> | Alle 18 servo's gekalibreerd, gewrichtslimieten gemeten | [`calibration/`](calibration/) |
> | Bewegingen op de echte robot: opstaan, lopen, draaien, dansen | [`tools/stand.py`](tools/stand.py), [`walk.py`](tools/walk.py), [`spin.py`](tools/spin.py), [`dance.py`](tools/dance.py), [`motion.py`](tools/motion.py) |
> | Besturen met een Bluetooth-gamepad, veilig via geplande gangen | [`tools/gamepad_robot.py`](tools/gamepad_robot.py), [`plan_gaits.py`](tools/plan_gaits.py) |
> | Live 3D-tweeling + camera + cockpit in de browser, met login | [`tools/live_twin.py`](tools/live_twin.py), [`tools/web/`](tools/web/), [`auth.py`](tools/auth.py) |
> | MuJoCo-simulatie in de browser | [`tools/sim_world.py`](tools/sim_world.py), [`tools/web/sim.html`](tools/web/sim.html) |
> | Zelf lopen: het getrainde beleid live op de robot, met de IMU; houdt zich vlak op hellingen, tilt de poten hoger op oneffen grond | [`tools/policy_walk.py`](tools/policy_walk.py), [`terrain_gait.py`](tools/terrain_gait.py), [uitleg](docs/self-walking.md) |
> | Valbeveiliging: vrije val of omkantelen → zacht landen | [`tools/fall_guard.py`](tools/fall_guard.py), [`training/fall_test.py`](training/fall_test.py) |
> | Pakken: op vier poten iets van 12-15 cm breed oppakken met de voorpoten | [`tools/grab.py`](tools/grab.py), [`training/grab_test.py`](training/grab_test.py) |
> | BAM-metingen (38 / 173 / 263 / 383 g) en identificatie | [`bam_data/`](bam_data/), [`simulation/bam_identify.py`](simulation/bam_identify.py) |
> | ESP32-C3 one-wire firmware voor de STS3215-bus | [`firmware/`](firmware/) |
> | Android-app (wifi-scan, cockpit, "hey spin") | [arachne-app](https://github.com/KVR-CIBER/arachne-app) |

ARACHNE 18 is a six-legged robotics prototype built around eighteen Feetech STS3215 12 V bus servos. A scalloped chassis carries six identical three-axis legs, while a removable dorsal shell conceals a battery tray and a universal controller deck. Paired printed brackets support the supplied metal servo horns on both sides of each joint.

The repository contains editable solid geometry, print-ready meshes, the CadQuery source, assembly guidance, and a working MuJoCo model driven by Better Actuator Models (BAM).

*The hero is AI-assisted presentation artwork. The CAD views and MuJoCo capture below are generated directly from the model. See [image provenance](docs/media.md).*

## At a glance

| | Specification |
|---|---|
| Actuators | 18 × Feetech STS3215 **12 V**, three per leg |
| Mechanism | Six yaw–hip–knee chains; 18 revolute joints |
| Chassis footprint | Approximately 188 × 150 mm, with swept leg-root reliefs |
| Neutral overall envelope | Approximately 374 × 514 × 214 mm |
| Link dimensions | Coxa 50 mm; femur 78 mm; tibia approximately 110 mm |
| Battery space | **115 × 40 × 35 mm**, excluding protruding plugs |
| Power | 3S pack: 11.1 V nominal, 12.6 V fully charged |
| Controller space | **90 × 60 × 18 mm**, on a slotted 108 × 75 mm plate |
| Printed components | 14 unique files; 63 robot pieces, plus a direct-horn fit gauge |
| Hardware in CAD | 262 DIN 7380 M3 screws, 112 ruthex-style inserts, 6 foot locknuts |
| Simulation | 19 rigid links, 18 motors, 499 visual meshes |
| Estimated model mass | 2.346 kg, including assumed battery and hardware |

## Studio portrait

![ARACHNE with internal horns and outside button-head screws](assets/hero-studio.png)

## Look inside

| Serviceable dorsal compartment | Six identical leg modules |
|---|---|
| ![CAD view with the removable shell hidden](assets/cad-internals.png) | ![Top view of the actual CAD geometry](assets/cad-top.png) |

The battery is restrained with two straps. The controller has its own removable deck above it. The shell lifts off after removing four accessible screws. The horns seat directly against equally spaced 5.25 mm printed cheeks; no loose shims or one-sided spacer webs are used. Swept local openings on the dummy side clear the two servo plugs without shifting the leg off the servo centerline.

The mechanical revision uses the owner's **36.5 mm measured outside-to-outside horn span**, with a 36.9 mm inside cheek span, a 7.2 mm shaft-access opening, and 4.0 mm insert bores for ruthex RX-M3x5.7. The outside horn screws are recessed DIN 7380 **M3 × 6**; the other assembly screws and inserts are individual CAD parts. Print the [fit gauge](print/stl/14_direct_horn_fit_gauge.stl) and trial-fit one real servo and both plugs before producing all six legs.

![Orthographic top view of one leg from the actual CAD, showing equally spaced coxa, femur and tibia cheeks](assets/cad-leg-top.png)

*One-leg orthographic CAD view. The joint cheeks share the same ±23.7 mm outside planes; only the cable relief on the dummy side differs. The paired femur cheeks flare equally at the knee.*

**Horn placement:** the metal horn sits on the **inside** of the printed cheek, against the servo. The outside has four screw heads and a central access hole. Do not mount a metal disc on the outside of the bracket.

![Knee detail from CAD showing all four horn screws supported by the printed tibia eye](assets/cad-horn-detail.png)

The knee detail is generated from the corrected CAD. A bearing-support check samples material beneath all four horn screw heads on each side of the coxa, femur and tibia.

The front and rear pairs can bring their TPU toes together with a **coordinated** yaw, hip and knee motion. Turning two bent legs inward independently can make their knee forks collide; the [joint guide](docs/simulation.md) gives the checked reach pose and its limits.

## Simulate it

Moving to a training computer? Start with the [clone and setup guide](docs/another-computer.md).

Use Python 3.12. From the repository root:

```sh
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r simulation/requirements.txt
python simulation/simulate.py --headless --seconds 10 --voltage 11.1
```

Interactive viewer on macOS:

```sh
mjpython simulation/simulate.py --seconds 300
mjpython simulation/grounded_demo.py --seconds 300
```

On Linux and Windows, use `python` for the viewer. On Windows, activate the environment with `.venv\Scripts\activate`. See the [full simulation guide](docs/simulation.md) for controls, joint frames, custom commands, model regeneration, and troubleshooting.

![Actual MuJoCo model after settling into a six-foot stance](assets/mujoco-preview.png)

![Actual BAM-driven motion with all six feet on the floor](assets/mujoco-grounded.gif)

*Actual floating-base MuJoCo simulation: the body rises and lowers while all six feet keep contact with the ground.*

### In the browser, shared over wifi

`tools/live_twin.py` also serves a physics simulation at `/sim` (on the robot: `http://spin.local:5870/sim`). Everyone on the network sees and drives the same virtual robot: pick which parts to show (hidden parts also leave the physics mass), drive with the pad or WASD/QE in tripod, ripple or wave gait, tilt the body, run the shows, or run the real robot scripts (`stand.py`, `walk.py`, `spin.py`, `dance.py`) unchanged against virtual STS3215 servos. The meter panel shows supply current and voltage sag (5 A lab supply, 10 A, 3S LiPo, with brown-out), torque per servo, foot forces and the stability margin. The real servos are never touched.

```sh
.venv/bin/python tools/live_twin.py          # http://localhost:5870/sim
tools/deploy_pi.sh                           # copy to the Pi and restart the arachne-twin service
```

![Browser simulation: tripod walk with footprints and the support triangle](assets/sim-browser.png)

`/cockpit` puts the camera feed, the 3D view and the controls on one screen for a landscape tablet. It shows a picture-in-picture that you tap to swap. It has touch sticks, every move as a button, and gamepad support (Xbox/Recon or PlayStation, over USB or Bluetooth).

Cameras are found automatically:
- the Pi camera (`rpicam-vid`);
- USB webcams (`/dev/v4l/by-id/*usb*`, via ffmpeg);
- any MJPEG command in `ARACHNE_CAMERA`, `ARACHNE_CAMERA_1`, and so on.

Streams are served at `/camera.mjpg?cam=N`. A Pi 4 has a single CSI connector, so a second camera has to be USB.

An MPU6050 on I2C (SDA GPIO2, SCL GPIO3) is read by `tools/imu.py`. It appears in `/state` and in the cockpit. Its axes still have to be matched to the body by tilting the robot by hand.

The Android app (`~/arachne-app`, Capacitor) works like this:
- It scans the wifi for the twin.
- It opens the cockpit with an on-device voice assistant: say "hey spin, dans", "kom naar mij", "draai om" or "hoeveel volt?".
- Without a key it understands fixed Dutch commands. With a Claude API key it answers free questions and chooses moves through tool calls.
- Turning and walking distances are closed-loop on the measured pose.

### Learned walking and a training army

The videos and quoted performance below were recorded before the symmetric-leg CAD revision. The revised MuJoCo model retains the same 18-joint topology; gait, terrain and jump unit tests pass, and the general walking policy completed a five-second BAM smoke run without a fall. The archived performance figures have not yet been remeasured against this revision.

A separate [jump policy and video](https://tom-michiels.github.io/arachne-18/#jump)
maximize clearance beneath every leg and the body, with no upright reward.
The highest recorded jump clears all CAD parts by **2.40 cm**; a second policy
prioritizes consistency across physics timesteps. [Policies and measurements](training/JUMP.md).


The [obstacle videos](https://tom-michiels.github.io/arachne-18/#terrain-pebbles)
now include individual collision stones and passive grass tufts that bend on
contact. Small-stone and short-grass forward examples pass; larger rocks remain
curriculum targets. [Models, curriculum and measured limits](training/OBSTACLES.md).


**[Open the video page — faster walking, all directions and the training army](https://tom-michiels.github.io/arachne-18/)**

The first [walking video](https://tom-michiels.github.io/arachne-18/#fast)
now reaches **29.9 cm/s**, up from 22.4 cm/s: **34% faster**, with **32% greater
foot travel** (6.45 cm versus 4.90 cm) and slightly lower cadence. A broader
smooth foot arc makes the larger steps possible within the original joint limits.
The checkpoint passes **26/26 reference checks**: directions and perturbations
at a 0.24 m/s command, plus fast forward motion at 0.34 m/s. The 20-second
recording includes acceleration and a gentle stop at real simulation time.
[Measurements, operating limits and reproduction](training/SPRINT.md).

A new IMU-aware CEM policy walks in every horizontal direction, turns, follows
curves and stops smoothly. It was trained with the fast local MuJoCo/Metal
simulator and independently checked in the original MuJoCo + BAM model:
**26/26 validation cases pass**, with about **13.3 cm/s** at the normal command
and **19.3 cm/s** at a faster command. The 40-second direction-change demo has
0.084° RMS body tilt and no falls or non-foot ground contacts.

[![128 independent spiders during actual training](assets/arachne-training-army.jpg)](assets/arachne-training-army.mp4)

**[Watch the 128-spider training army](assets/arachne-training-army.mp4)** ·
**[Watch walking, direction changes and turning](assets/arachne-learned-omni.mp4)** ·
[Controller, objective, measured results and reproduction](training/README.md)

The army uses real recorded states from distinct training candidates, arranged
for display in independent cells. The solo demonstration uses full BAM physics.
These are simulated gaits; hardware walking has not been validated.

The [uneven-ground curriculum](training/TERRAIN.md) has passed 318 admission
episodes through 4 mm obstacles. It keeps strict body-stability and smoothness
gates, retains flat-ground walking, and stops before unmastered 8 mm obstacles.

**BAM is activated by `simulate.py`. Loading the XML alone does not activate the servo model.** The supplied 12 V M6 parameter set is an approximation fitted to manufacturer torque and speed points, with friction and controller behavior inherited from BAM's identified 7.4 V STS3215 model. It is clearly versioned separately from the original. Read the [model assumptions](docs/bam-model.md) before using it for actuator studies.

## Build it

1. Print the **fit gauge** and one servo clamp. Check a real servo and both supplied horns.
2. Check the measured 36.5 mm horn span, the 2.5/2.0 mm horn thicknesses, the dummy-side plugs, and the shaft clearance against the gauge.
3. Print the structural parts, fit inserts, and build one complete leg.
4. Assemble the six legs, dorsal compartment, electronics, and cable loops.
5. Calibrate servo IDs, encoder offsets, and directions before applying motion.

Start with the [build guide](docs/build-guide.md), [print BOM](print/print_bom.csv), and [hardware BOM](docs/hardware-bom.csv). All individual parts fit the Bambu Lab H2D. STLs use **millimetres** and must be sliced at **100% scale**.

## Repository map

| Path | Contents |
|---|---|
| [`cad/`](cad) | Complete assembly STEP, unique-part STEP layout, individual STEP parts, dimensions and native-name mapping |
| [`print/stl/`](print/stl) | 14 printable parts in bed-oriented poses |
| [`simulation/`](simulation) | MJCF, BAM parameters, runtime, tests, joint map and local-frame meshes |
| [`src/`](src) | CadQuery builder, CAD checks, MuJoCo exporter and rendering source |
| [`tools/`](tools) | Real-robot tools: servo driver, calibration, live twin + browser simulation (`/sim`), motion scripts, Pi deploy |
| [`docs/`](docs) | Build, simulation, actuator model, Onshape and development guides |
| [`validation/`](validation) | CAD and mesh validation reports |
| [`assets/`](assets) | Studio artwork, CAD renders and simulation media |
| [`licenses/`](licenses) | Third-party license material |

## Validation and project status

The supplied geometry has **14/14 valid single-solid print parts**, **14/14 watertight single-component STLs**, a 1,152-point horn-bearing support check and a CAD check of 262 screw placements and screwdriver routes. Ninety-nine representative leg poses, an 18-pose paired-toe reach path, and 594 nominal connector/straight-exit positions were checked for hard-part interference. MuJoCo tests cover topology, all 18 joint axes, inertia, reset reproducibility, standing at three voltages, and a raised-base joint sweep. See [validation details](docs/validation.md) and the machine-readable reports.

This is a **digitally checked prototype**. The cable sweep uses an estimated plug envelope based on the owner's servo photograph; physical plug fit, flexible cable routing, fatigue, loads and payload still require a one-leg prototype. The runtime includes standing, a grounded body-height exercise and a bench joint sweep; a walking controller is a next development step. The Onshape mating state is recorded separately in [the Onshape guide](docs/onshape.md).

## Sources and attribution

Mechanical dimensions and 12 V operating points come from Feetech documentation. The actuator model builds on [Rhoban/BAM](https://github.com/Rhoban/bam); the simulator uses [MuJoCo](https://github.com/google-deepmind/mujoco). See [sources and third-party notices](THIRD_PARTY_NOTICES.md). No project-wide open-source license has been selected; the included BAM material retains its Apache-2.0 license.
