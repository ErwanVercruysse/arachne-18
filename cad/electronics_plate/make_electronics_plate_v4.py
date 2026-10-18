"""Acrylic plate v4 for the ARACHNE-18 MINI brain PCB (cad/brain_mini, 92 x 60 mm, ESP32-S3, Pi 4 over USB).

Same outline, chassis cable openings and chassis mounting slots as v2/v3, plus:
  * four M3 holes (3.2 mm) for the mini PCB: M3 x 12 screw -> PCB -> 8 mm nylon spacer -> plate -> nut;
  * two zip-tie slots as strain relief for the 12 V battery lead, below the PCB's 12 V solder pads;
  * the PCB outline, the antenna edge and the leg-bus sides are engraved.

    .venv/bin/python cad/electronics_plate/make_electronics_plate_v4.py
    -> electronics_plate_v4.dxf / .svg / _preview.png beside this file
"""
from pathlib import Path
import json
import ezdxf
from shapely.geometry import Point, Polygon
import make_electronics_plate as v2
import make_electronics_plate_v3 as v3

HERE = Path(__file__).resolve().parent
OUTLINE = HERE.parent/'brain_mini'/'tools'/'outline.json'
PCB_W, PCB_H = 92., 60.
MOUNT = (41.5, 25.5)
HOLES = [(sx*MOUNT[0], sy*MOUNT[1]) for sx in (-1, 1) for sy in (-1, 1)]
# PCB y axis points the other way than the DXF y axis: 12 V pads at PCB (-29..-22, +25) = plate (-29..-22, -25)
TIES = [(-31., -36.), (-25., -36.)]          # vertical slots left and right of the lead


def build(path):
    plate, cable, mounts, _ = v3.build_geometry()
    pcb = Polygon([(x, -y) for x, y in json.load(open(OUTLINE))['edge']])
    assert plate.contains(pcb), 'PCB does not fit on the plate'
    holes = [Point(p).buffer(1.6, quad_segs=16) for p in HOLES]
    for p in HOLES:
        spacer = Point(p).buffer(3.5)
        assert plate.buffer(-1).contains(spacer), f'hole {p} too close to the plate edge'
        assert all(spacer.distance(c) > 0.8 for c in cable + mounts), f'hole {p} clashes with a cable opening'
    ties = [v2.rect(x, y, 2.2, 5.0) for x, y in TIES]
    for t in ties:
        assert plate.buffer(-2).contains(t), 'tie slot too close to the plate edge'
        assert all(t.distance(c) > 2 for c in cable + mounts), 'tie slot clashes with a cable opening'
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.MM
    doc.layers.add('CUT', color=1); doc.layers.add('ENGRAVE', color=5)
    msp = doc.modelspace()
    for g in [plate, *cable, *mounts, *holes, *ties]:
        v2.add_poly(msp, g, 'CUT')
    v2.add_poly(msp, pcb, 'ENGRAVE')
    T = lambda s, at, h=3.: msp.add_text(s, height=h, dxfattribs={'layer': 'ENGRAVE'}).set_placement(
        at, align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    T('ARACHNE-18 MINI', (0, 0), 4.)
    T('ANTENNE-RAND', (4, PCB_H/2 - 2.5), 2.2)
    T('L1-L3', (PCB_W/2 - 10, 0), 2.5)
    T('L4-L6', (-PCB_W/2 + 10, 0), 2.5)
    T("12V", (-28., -40.5), 2.5)
    doc.saveas(path)
    return doc, plate


if __name__ == '__main__':
    out = HERE/'electronics_plate_v4.dxf'
    doc, plate = build(out)
    v2.preview(doc, plate, HERE/'electronics_plate_v4_preview.png', HERE/'electronics_plate_v4.svg')
    x0, y0, x1, y1 = plate.bounds
    print(f'written {out}\nplate {x1-x0:.1f} x {y1-y0:.1f} mm')
