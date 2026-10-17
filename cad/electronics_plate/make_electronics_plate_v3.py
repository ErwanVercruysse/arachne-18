"""Acrylic plate v3 for the ARACHNE-18 brain PCB (cad/brain_pcb, 140 x 70 mm, CM5 + ESP32-S3).

Same outline, chassis cable openings and mounting slots as v2 (see make_electronics_plate.py), but:
  * no holes for the Pi 4, the two Waveshare adapters or the LM2596: everything is on the PCB now;
  * the PCB shares the four chassis M3 screws: M3 x 25 through PCB -> 10 mm spacer -> plate -> insert,
    so the servo cables still pass underneath the PCB through the chassis openings;
  * two zip-tie slots as strain relief for the 12 V battery lead, next to the PCB's solder pads;
  * the PCB outline (same shape as this plate, notched at the corner legs), the antenna edge and
    the leg-bus sides are engraved.

    .venv/bin/python cad/electronics_plate/make_electronics_plate_v3.py
    -> electronics_plate_v3.dxf / .svg / _preview.png beside this file
"""
from pathlib import Path
import ezdxf
from shapely.geometry import Polygon, box
from shapely.ops import unary_union
import make_electronics_plate as v2

HERE = Path(__file__).resolve().parent
PCB_W, PCB_H, PCB_R = 140., 70., 6.
# PCB y axis points the other way than the DXF y axis; the 12 V pads sit at PCB (-49, +30) = plate (-49, -30)
PADS = (-40.5, -30.)
TIES = [(-38., -39.5), (-30., -39.5)]
OUTLINE = HERE.parent/'brain_pcb'/'tools'/'outline.json'   # written while designing the PCB


def pcb_outline():
    """The PCB edge (it follows this plate, 1 mm inside, with notches at the corner legs)."""
    import json
    if OUTLINE.exists():
        return Polygon([(x, -y) for x, y in json.load(open(OUTLINE))['edge']])
    return box(-PCB_W/2, -PCB_H/2, PCB_W/2, PCB_H/2).buffer(-PCB_R).buffer(PCB_R, quad_segs=16)


def build_geometry():
    loops = v2.section_loops(v2.load_stl(v2.STL), .3)
    outer, holes = loops[0], loops[1:]
    mounts = v2.mount_slots()
    ears = [m.buffer(4.5).intersection(outer.buffer(-.5)) for m in mounts]
    body = outer.buffer(-v2.INSET, join_style='round')
    body = body.buffer(-v2.TRIM, join_style='round').buffer(v2.TRIM, join_style='round')
    plate = unary_union([body, *ears])
    plate = plate.buffer(-1., join_style='round').buffer(1., join_style='round').simplify(.05)
    if plate.geom_type != 'Polygon':
        plate = max(plate.geoms, key=lambda g: g.area)
    plate = Polygon(plate.exterior)
    cable = [h.simplify(.05) for h in holes if not any(h.intersects(m) for m in mounts)]
    ties = [v2.rect(x, y, 5.0, 2.2) for x, y in TIES]
    for t in ties:
        assert plate.buffer(-2).contains(t), 'tie slot too close to the plate edge'
        assert all(t.distance(c) > 2 for c in cable), 'tie slot clashes with a cable opening'
    return plate, cable, mounts, ties


def build(path):
    plate, cable, mounts, ties = build_geometry()
    pcb = pcb_outline()
    assert plate.contains(pcb), 'PCB does not fit on the plate'
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.MM
    doc.layers.add('CUT', color=1); doc.layers.add('ENGRAVE', color=5)
    msp = doc.modelspace()
    for g in [plate, *cable, *mounts, *ties]:
        v2.add_poly(msp, g, 'CUT')
    v2.add_poly(msp, pcb, 'ENGRAVE')
    T = lambda s, at, h=3.: msp.add_text(s, height=h, dxfattribs={'layer': 'ENGRAVE'}).set_placement(
        at, align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    T('ARACHNE-18 BRAIN PCB', (0, 0), 4.)
    T('ANTENNE-RAND', (0, PCB_H/2 - 4), 2.5)
    T('BUS A  L1-L3', (PCB_W/2 - 14, 0), 2.5)
    T('BUS B  L4-L6', (-PCB_W/2 + 14, 0), 2.5)
    T('12V', (-34., -43.5), 2.5)
    doc.saveas(path)
    return doc, plate


if __name__ == '__main__':
    out = HERE/'electronics_plate_v3.dxf'
    doc, plate = build(out)
    v2.preview(doc, plate, HERE/'electronics_plate_v3_preview.png', HERE/'electronics_plate_v3.svg')
    x0, y0, x1, y1 = plate.bounds
    print(f'written {out}\nplate {x1-x0:.1f} x {y1-y0:.1f} mm')
