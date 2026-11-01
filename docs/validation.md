# V4 validation record

These checks establish consistency of the digital prototype. They do not certify physical fit, load capacity or hardware performance.

| Check | Result / scope | Evidence |
|---|---|---|
| Print solids | 8/8 valid, one solid each | [CAD report](../validation/cad_validation.json) |
| Print meshes | 8/8 watertight, one component; flat bed contact | [Integration report](../validation/integration_v4.json) |
| Baseline geometry | Chassis and four leg parts match the checked v4 prototypes | [Integration report](../validation/integration_v4.json) |
| Packaging | Battery, PCB, controller envelope, carrier and canopy clearance | [Integration report](../validation/integration_v4.json) |
| Screw/print fit | 160 M3 and 72 factory screws; no print overlap above 0.05 mm³ | [Integration report](../validation/integration_v4.json) |
| Dorsal tool access | Carrier, PCB and cover driver paths clear in assembly order | [Integration report](../validation/integration_v4.json) |
| Leg interfaces | Case insertion, horns, screw bearing, head access, dummy plugs and sampled joint sweeps | [Leg report](../validation/leg_v4.json) |
| Chassis interfaces | Six case insertion paths and plug service columns; factory and horn tool access | [Body report](../validation/body_fit_v4.json) |
| Chassis motion | 90 combined representative poses, yaw sweep, neutral leg pairs | [Body report](../validation/body_fit_v4.json) |
| Paired toe reach | Front and rear, 13 coordinated samples per path; shoes meet | [Body report](../validation/body_fit_v4.json) |
| Dorsal motion | 162 sampled leg poses against carrier and canopy | [Integration report](../validation/integration_v4.json) |
| Installation | Fork insertion, case-head clearance and battery-strap corridors | [Installation report](../validation/body_mount_sequence_v4.json) |
| Slicing | 8 parts sliced without reported warnings; A1 geometry proxy, not H2D G-code | [Slicer report](../validation/slicer_v4.json) |
| Onshape native graph | 336 instances, 317 fastened, 18 revolute; fixed chassis | [Native report](../simulation/onshape_validation.json) |
| Onshape limits | All 18 native limits match joint map | [Limit report](../simulation/onshape_motion_validation.json) |
| MuJoCo | 19 links, 18 hinges; positive inertias, correct subtrees, reproducible reset | [Simulation report](../simulation/simulation_validation.json) |
| Standing | 10 seconds each at 11.1/12.0/12.6 V; all six feet support the body | [Simulation report](../simulation/simulation_validation.json) |
| Fixed sweep | 12 seconds; tracking error below 0.73° after settling | [Simulation report](../simulation/simulation_validation.json) |
| Grounded exercise | 8 seconds; six supporting feet, 12 contact points; no warnings | [Grounded report](../simulation/grounded_validation.json) |
| Animation | 100 frames / 4 seconds; six feet support the free base | [Animation report](../validation/grounded_animation.json) |
| Training compatibility | 17 existing unit tests pass; no training performed | [Compatibility report](../validation/training_compatibility_v4.json) |

The leg horn-bearing test samples material under all four screws on both faces of every joint type. The chassis and leg source tests use the vendor case reference plus the owner's measured Ø6.5 mm dummy shaft. Connector checks use an estimated two-plug and straight-exit envelope on the dummy side. Flexible wire movement and real plug dimensions still need a physical trial.

Motion checks are discrete samples and selected installation paths. They are not exhaustive continuous collision proofs across all 18 joint combinations. The current simulation uses approximate collision shapes and nominal mass distributions; standing does not validate walking, jumping, thermal limits or structural strength.

No physical print, pull-out test, FEA, fatigue or impact test has been completed. Existing RL metrics and videos belong to earlier geometry. Historical v3 evidence is isolated under `validation/legacy-v3/` and is not used to claim current performance.
