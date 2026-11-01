# Native Onshape v4 assembly

[Open ARACHNE 18 — v4 articulated assembly](https://cad.onshape.com/documents/88edbeda4232642329938c14/w/19f5435855c2974f02fa7c68/e/916e6d95108202647e946643)

[Open its source Part Studio](https://cad.onshape.com/documents/88edbeda4232642329938c14/w/19f5435855c2974f02fa7c68/e/d68fc9f6aee00c72d944de06)

The v4 assembly is a separate named tab in the existing project document. Earlier tabs preserve the preceding design. All 336 imported source parts are solid bodies, including the printed structure, 18 servo exterior references, 36 horns, 160 DIN 7380 M3 screws, 72 factory case screws, 16 inserts and the battery/controller references.

[Open the eight unique printable parts](https://cad.onshape.com/documents/88edbeda4232642329938c14/w/19f5435855c2974f02fa7c68/e/f85bc8278acffc5ff06239e0)

## Mate graph

The fixed chassis anchors 19 rigid groups. **317 fastened mates** attach each group's printed components, cases, horns, screws and inserts. **18 revolute mates** provide the six yaw–hip–knee chains. Servo cases belong to the upstream group; rotating horns and their screws belong to the downstream group.

```text
fixed chassis → yaw → coxa → hip → femur → knee → tibia + TPU shoe
```

`R_L1_yaw`, `R_L1_hip`, `R_L1_knee` and equivalents for L2–L6 identify the revolute mates. `F_` names are fastened mates; `MC_` and `J_` names are explicit mate connectors. All names are English.

Hip neutral is +20°, knee neutral is −85°. Joint values are offsets from that pose. Native limits use front/rear yaw 60° inward/35° outward, middle yaw ±35°, hip −50°/+25° and knee −20°/+100°. [Joint conventions](simulation.md).

The precise instance graph is in [onshape_mate_plan.json](../simulation/onshape_mate_plan.json). The [native readback report](../simulation/onshape_validation.json) checks instance/mate counts, solver feature status, fixed chassis and neutral transforms. The [limit report](../simulation/onshape_motion_validation.json) checks all 18 limit definitions. These API checks do not claim a separate interactive animation test. CAD and MuJoCo motion checks are documented independently.

## Editing and export

The STEP import preserves solids and placement, not CadQuery feature history. Change the parametric source in `src/v4_*.py` and regenerate when modifying geometry. The assembled servo references simplify the exterior; the original vendor STEP is retained locally for mechanical fit checks.

The MuJoCo model is generated from the same positioned CAD solids, rigid groups and joint frames. The floating base receives one global height adjustment to place the actual TPU shoes on the floor. See [development.md](development.md) for reproducible build and native mate commands.
