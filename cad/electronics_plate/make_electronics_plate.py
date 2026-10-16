"""Acrylic plate for a Raspberry Pi 4, two Waveshare Bus Servo Adapters (A) and an LM2596
step-down module on top of the BUILT ARACHNE (v2, 2026-10-11; v1 had only the two adapters).

Shape and holes come from the printed top plate, ref/01_chassis_built.stl
(z = 0 is its top face; the repo's cad/parts/01_chassis.step is a different design):
  * outline = the chassis outline, 2 mm smaller all round, without the thin leg fingers;
  * the chassis' cable holes are copied, so wires still pass straight through;
  * 4 mounting slots on the M3 insert holes at (+-64, +-25) mm. Hand measurement
    gave 130.2 x 50.7 mm, so each slot spans both 128/130.2 and 50/50.7.

    python cad/electronics_plate/make_electronics_plate.py
    -> electronics_plate.dxf / .svg / _preview.png beside this file

Layers: CUT (red, through-cut), ENGRAVE (blue, board outlines and labels).
"""
from pathlib import Path
import re, struct
import numpy as np
import ezdxf
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
STL = HERE/'ref'/'01_chassis_built.stl'

INSET = 2.                       # plate outline inside the chassis outline (mm)
TRIM = 6.                        # remove fingers narrower than 12 mm
MOUNT_CAD, MOUNT_MEAS = (64., 25.), (65.1, 25.35)   # half spacings: STL vs hand-measured
MOUNT_HOLE = 3.4                 # M3 clearance
# Waveshare Bus Servo Adapter (A): 42 x 33 mm, 4x M2.5 holes on 37 x 28 mm.
BOARD, PITCH, BOARD_HOLE = (42., 33.), (37., 28.), 2.7
BOARDS = [(-66., 0.), (66., 0.)]
# Raspberry Pi 4 B: 85 x 56 mm, 4x M2.5 on 58 x 49 mm, holes 3.5 mm from the edge opposite the USB/LAN end.
PI, PI_C, PI_HOLE = (85., 56.), (0., -8.), 2.7
PI_USB_RIGHT = True              # USB/LAN end towards +x
# LM2596 step-down module ~43 x 21 mm: held by two zip ties (no hole pattern needed).
BUCK, BUCK_C = (43., 21.), (0., 34.)
TIE = (3.6, 1.8)                 # zip-tie slot (for ties up to ~3 mm wide)


def load_stl(path):
    b = path.read_bytes(); n = struct.unpack('<I', b[80:84])[0]
    if 84+n*50 == len(b):
        a = np.frombuffer(b[84:], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]), count=n)
        return a['v'].reshape(-1, 3, 3).astype(float)
    return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), dtype=float).reshape(-1, 3, 3)


def section_loops(tri, z):
    """Closed XY loops where the plane z cuts the mesh."""
    d = tri[:, :, 2]-z; s = np.sign(d); m = (s.max(1) > 0) & (s.min(1) < 0)
    key = lambda p: (round(p[0], 3), round(p[1], 3))
    adj = {}
    for t, dd in zip(tri[m], d[m]):
        pts = [t[i]+dd[i]/(dd[i]-dd[j])*(t[j]-t[i]) for i, j in ((0, 1), (1, 2), (2, 0)) if (dd[i] > 0) != (dd[j] > 0)]
        if len(pts) == 2:
            a, c = key(pts[0]), key(pts[1]); adj.setdefault(a, []).append(c); adj.setdefault(c, []).append(a)
    seen, loops = set(), []
    for start in adj:
        if start in seen: continue
        loop, prev, cur = [start], None, start; seen.add(start)
        while True:
            nxt = [p for p in adj[cur] if p != prev and p not in seen]
            if not nxt: break
            prev, cur = cur, nxt[0]; seen.add(cur); loop.append(cur)
        if len(loop) > 8: loops.append(Polygon(loop).buffer(0))
    return sorted(loops, key=lambda p: -p.area)


def mount_slots():
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            a = (sx*MOUNT_CAD[0], sy*MOUNT_CAD[1]); b = (sx*MOUNT_MEAS[0], sy*MOUNT_MEAS[1])
            out.append(LineString([a, b]).buffer(MOUNT_HOLE/2, quad_segs=16))
    return out


def board_holes(bx, by):
    return [Point(bx+sx*PITCH[0]/2, by+sy*PITCH[1]/2).buffer(BOARD_HOLE/2, quad_segs=16) for sx in (-1, 1) for sy in (-1, 1)]


def rect(cx, cy, w, h): return Polygon([(cx-w/2, cy-h/2), (cx+w/2, cy-h/2), (cx+w/2, cy+h/2), (cx-w/2, cy+h/2)])


def pi_holes():
    cx, cy = PI_C; s = 1 if PI_USB_RIGHT else -1
    xs = [cx-s*(PI[0]/2-3.5), cx-s*(PI[0]/2-61.5)]
    return [Point(x, cy+dy).buffer(PI_HOLE/2, quad_segs=16) for x in xs for dy in (-24.5, 24.5)]


def tie_slots():
    cx, cy = BUCK_C
    return [rect(cx+dx, cy+dy, *TIE) for dx in (-BUCK[0]/2+9, BUCK[0]/2-9) for dy in (-BUCK[1]/2-2.6, BUCK[1]/2+2.6)]


def build_geometry():
    loops = section_loops(load_stl(STL), .3)
    outer, holes = loops[0], loops[1:]
    mounts = mount_slots()
    # The insert holes sit in small ears on the chassis edge: keep full material there.
    ears = [m.buffer(4.5).intersection(outer.buffer(-.5)) for m in mounts]
    body = outer.buffer(-INSET, join_style='round')
    # Drop the thin fingers around the leg cradles (< 2*TRIM wide): fragile in acrylic
    # and right where the coxa brackets swing.
    body = body.buffer(-TRIM, join_style='round').buffer(TRIM, join_style='round')
    plate = unary_union([body, *ears])
    plate = plate.buffer(-1., join_style='round').buffer(1., join_style='round').simplify(.05)  # no sharp slivers
    if plate.geom_type != 'Polygon': plate = max(plate.geoms, key=lambda g: g.area)
    plate = Polygon(plate.exterior)
    # copy chassis openings, except the four insert holes that become our mounting slots
    cable = [h.simplify(.05) for h in holes if not any(h.intersects(m) for m in mounts)]
    bholes = [h for bx, by in BOARDS for h in board_holes(bx, by)] + pi_holes() + tie_slots()
    # board screws need plexi around them: drop any copied opening they would fall into
    clash = [c for c in cable if any(c.buffer(1.).intersects(h) for h in bholes)]
    cable = [c for c in cable if c not in clash]
    return plate, cable, mounts, bholes, clash


def add_poly(msp, poly, layer):
    msp.add_lwpolyline(list(poly.exterior.coords), close=True, dxfattribs={'layer': layer})


def build(path):
    plate, cable, mounts, bholes, clash = build_geometry()
    outlines = [rect(bx, by, *BOARD) for bx, by in BOARDS] + [rect(*PI_C, *PI), rect(*BUCK_C, *BUCK)]
    for r in outlines: assert plate.buffer(-1).contains(r), 'board does not fit on the plate'
    for i, a in enumerate(outlines):
        for b in outlines[i+1:]: assert a.distance(b) >= 1.5, 'boards overlap'
    assert all(plate.buffer(-1.5).contains(t) for t in tie_slots()), 'zip-tie slot too close to the edge'
    assert all(plate.buffer(-1.5).contains(m) for m in mounts), 'mounting slot too close to the edge'
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.MM
    doc.layers.add('CUT', color=1); doc.layers.add('ENGRAVE', color=5)
    msp = doc.modelspace()
    for g in [plate, *cable, *mounts, *bholes]:
        add_poly(msp, g, 'CUT')
    for n, (bx, by) in enumerate(BOARDS, 1):
        add_poly(msp, rect(bx, by, *BOARD), 'ENGRAVE')
        msp.add_text(f'SERVO {n}', height=3.5, dxfattribs={'layer': 'ENGRAVE'}).set_placement(
            (bx, by+11), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    add_poly(msp, rect(*PI_C, *PI), 'ENGRAVE'); add_poly(msp, rect(*BUCK_C, *BUCK), 'ENGRAVE')
    usb = PI_C[0] + (PI[0]/2-8 if PI_USB_RIGHT else -PI[0]/2+8)
    msp.add_text('RASPBERRY PI 4', height=4., dxfattribs={'layer': 'ENGRAVE'}).set_placement(
        (PI_C[0]-8, PI_C[1]), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    msp.add_text('USB', height=3., dxfattribs={'layer': 'ENGRAVE'}).set_placement(
        (usb, PI_C[1]), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    msp.add_text('5V BUCK', height=3., dxfattribs={'layer': 'ENGRAVE'}).set_placement(
        BUCK_C, align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    # (no ARACHNE label: no free spot left that clears the holes)
    doc.saveas(path)
    return doc, plate, cable, clash


def preview(doc, plate, png, svg):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from ezdxf.addons.drawing import RenderContext, Frontend
    from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
    x0, y0, x1, y1 = plate.bounds; w, h = x1-x0, y1-y0
    fig = plt.figure(figsize=(w/20, h/20)); ax = fig.add_axes([0, 0, 1, 1])
    Frontend(RenderContext(doc), MatplotlibBackend(ax)).draw_layout(doc.modelspace())
    ax.set_aspect('equal'); fig.savefig(png, dpi=120, facecolor='white')
    # SVG at true scale (mm) for laser software that prefers SVG
    fig.set_size_inches((w+4)/25.4, (h+4)/25.4); ax.set_xlim(x0-2, x1+2); ax.set_ylim(y0-2, y1+2)
    fig.savefig(svg, facecolor='none')


if __name__ == '__main__':
    out = HERE/'electronics_plate.dxf'
    doc, plate, cable, clash = build(out)
    preview(doc, plate, HERE/'electronics_plate_preview.png', HERE/'electronics_plate.svg')
    x0, y0, x1, y1 = plate.bounds
    print(f'written {out}\nplate {x1-x0:.1f} x {y1-y0:.1f} mm, {len(cable)} cable openings copied, {len(clash)} left solid under board screws')
