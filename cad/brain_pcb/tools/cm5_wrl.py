"""Simple VRML stand-in for the Raspberry Pi CM5 (footprint origin = mounting hole, module x -3.5..36.5, y -51.5..3.5)."""
import os
U = 1 / 2.54                      # KiCad VRML unit = 0.1 inch
boxes = [  # (x0, y0, x1, y1, z0, z1, rgb) in footprint mm, KiCad y-down
    (-3.5, -51.5, 36.5, 3.5, 1.5, 2.5, (0.10, 0.40, 0.20)),       # module PCB
    (8.0, -30.0, 24.0, -14.0, 2.5, 3.6, (0.75, 0.75, 0.78)),        # BCM2712 lid
    (25.5, -30.0, 33.5, -18.0, 2.5, 3.4, (0.08, 0.08, 0.08)),       # LPDDR4X
    (0.0, -44.0, 10.0, -33.0, 2.5, 3.5, (0.08, 0.08, 0.08)),        # eMMC
    (12.0, -46.0, 28.0, -34.0, 2.5, 4.2, (0.80, 0.80, 0.82)),       # RP1 / PMIC shield
    (2.0, -10.0, 18.0, 1.5, 2.5, 4.4, (0.82, 0.82, 0.85)),          # wireless shield
    (20.0, -4.0, 33.0, 2.5, 2.5, 2.6, (0.85, 0.75, 0.30)),          # PCB antenna
]
out = ['#VRML V2.0 utf8']
for x0, y0, x1, y1, z0, z1, c in boxes:
    cx, cy, cz = (x0 + x1) / 2 * U, -(y0 + y1) / 2 * U, (z0 + z1) / 2 * U
    sx, sy, sz = (x1 - x0) * U, (y1 - y0) * U, (z1 - z0) * U
    out.append(f'Transform {{ translation {cx:.4f} {cy:.4f} {cz:.4f} children [ Shape {{ appearance Appearance {{ material Material {{ '
               f'diffuseColor {c[0]} {c[1]} {c[2]} specularColor 0.3 0.3 0.3 shininess 0.4 }} }} geometry Box {{ size {sx:.4f} {sy:.4f} {sz:.4f} }} }} ] }}')
open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), '3d', 'cm5_standin.wrl'), 'w').write('\n'.join(out) + '\n')
