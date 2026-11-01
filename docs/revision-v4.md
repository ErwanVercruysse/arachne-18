# Complete mechanical revision v4

This release integrates the independently developed v4 leg and body into the complete six-leg robot.

- Three monolithic moving links per leg replace split cheek plates, clamps and structural joining hardware.
- Symmetric tibia and equal fork spacing; functional dummy-side cable and shaft-entry reliefs remain local.
- Factory servo case holes, recessed DIN 7380 horn screws and measured 36.5 mm horn span are retained.
- A one-piece scalloped chassis supports all six yaw servos and the battery straps.
- A removable carrier with integral posts, a configurable PCB mount and a ventilated organic canopy complete the dorsal compartment.
- Eight unique print files produce 31 printed pieces. The four small standoffs provide PCB electrical clearance; they are not leg shims.
- STEP, oriented STL, native Onshape mates, MJCF meshes, inertias, joint maps, contact geometry, print BOM and English documentation are updated together.
- The previous 50 mm coxa is now 62 mm; hip height is −6.95 mm and neutral hip angle is +20°. The training IK derives these values from the joint map.
- The former spherical foot-contact approximation is replaced by the v4 shoe's mesh hull in the full simulation, and a rotated thin box in the fast training model.

No RL training was repeated. Policies, results and recorded videos are unchanged. To reproduce earlier measurements with their earlier robot, check out Git commit `9a6ca7c` in a separate clone/worktree. Current policy compatibility tests do not establish current walking speed, jump height or terrain performance.

V4 remains a physical prototype candidate: no load rating, fatigue life, insert pull-out strength or hardware gait has been measured.
