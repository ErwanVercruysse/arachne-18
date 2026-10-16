"""BAM pendulum arm for one STS3215: two identical cut plates.

Each plate bolts to one supplied metal horn (driven side and dummy side) on
the 14 mm pitch circle. An M10 bolt through both plate tips carries the tip
mass: nuts and washers, clamped against each plate.

    python cad/bam_rig/make_bam_rig.py   ->  DXF, STEP and STL beside this file
"""
from pathlib import Path
import json, math
import cadquery as cq
import ezdxf

HERE = Path(__file__).resolve().parent
PARAMS = json.loads((HERE.parents[0]/'parameters.json').read_text())

L, W, T = 250., 30., 3.           # plate length, width, thickness (mm)
CORNER = 3.                       # corner radius, avoids sharp laser-cut corners
PIVOT = 15.                       # servo axis from the plate end
SHAFT_D = 7.2                     # dummy-shaft clearance / driven screw access
HORN_PCD = PARAMS['horn_pcd']     # 14 mm
HORN_HOLE_D = 3.2                 # M3 clearance
SPAN = PARAMS['horn_outer_span']  # 36.5 mm between outer horn faces = plate gap
TIP = L-PIVOT                     # M10 hole centre from the plate end, mirrors the pivot
TIP_D = 10.5                      # M10 clearance (ISO 273 fine)

horn_holes = [(PIVOT+HORN_PCD/2*math.cos(math.radians(a)), W/2+HORN_PCD/2*math.sin(math.radians(a)))
              for a in (45, 135, 225, 315)]


def plate():
    p = cq.Workplane('XY').box(L, W, T, centered=False).edges('|Z').fillet(CORNER)
    p = p.faces('>Z').workplane(origin=(0, 0, T)).pushPoints([(PIVOT, W/2)]).hole(SHAFT_D)
    p = p.faces('>Z').workplane(origin=(0, 0, T)).pushPoints([(TIP, W/2)]).hole(TIP_D)
    return p.faces('>Z').workplane(origin=(0, 0, T)).pushPoints(horn_holes).hole(HORN_HOLE_D)


def dxf(path):
    doc = ezdxf.new('R2010', setup=True); doc.units = ezdxf.units.MM
    doc.layers.add('CUT', color=1); msp = doc.modelspace()
    for n in range(2):
        oy = n*(W+10)
        r = CORNER
        pts = [(r, 0), (L-r, 0), (L, r), (L, W-r), (L-r, W), (r, W), (0, W-r), (0, r)]
        # Rounded rectangle as one closed polyline with bulges on the corners.
        bulge = math.tan(math.pi/8)
        poly = [(x, y+oy, 0, 0, bulge if i % 2 else 0) for i, (x, y) in enumerate(pts)]
        msp.add_lwpolyline([(x, y, s, e, b) for x, y, s, e, b in poly], format='xyseb', close=True, dxfattribs={'layer': 'CUT'})
        msp.add_circle((PIVOT, W/2+oy), SHAFT_D/2, dxfattribs={'layer': 'CUT'})
        msp.add_circle((TIP, W/2+oy), TIP_D/2, dxfattribs={'layer': 'CUT'})
        for x, y in horn_holes:
            msp.add_circle((x, y+oy), HORN_HOLE_D/2, dxfattribs={'layer': 'CUT'})
    doc.saveas(path)


def main():
    dxf(HERE/'bam_arm_plates.dxf')
    cq.exporters.export(plate(), str(HERE/'bam_arm_plate.step'))
    # Assembly: servo axis on the origin, arm along +X, plates outside the horns.
    asm = cq.Assembly(name='bam_pendulum_arm')
    asm.add(plate().translate((-PIVOT, -W/2, SPAN/2)), name='plate_driven', color=cq.Color(.75, .75, .78))
    asm.add(plate().translate((-PIVOT, -W/2, -SPAN/2-T)), name='plate_dummy', color=cq.Color(.75, .75, .78))
    bolt = cq.Workplane('XY').circle(5).extrude(SPAN+2*T+20).translate((TIP-PIVOT, 0, -SPAN/2-T-10))
    asm.add(bolt, name='m10_bolt_reference', color=cq.Color(.3, .3, .3))
    asm.save(str(HERE/'bam_pendulum_arm.step'))
    print('pivot to M10 bolt (length for BAM):', TIP-PIVOT, 'mm')
    print('plate mass, 3 mm aluminium ~', round(2.7e-3*(L*W-math.pi*(SHAFT_D**2+TIP_D**2+4*HORN_HOLE_D**2)/4)*T, 1), 'g each')


if __name__ == '__main__': main()
