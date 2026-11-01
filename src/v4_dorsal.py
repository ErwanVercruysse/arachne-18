"""Removable controller carrier and organic cover for the v4 body (mm)."""
import math
import cadquery as cq
from v4_leg import box, cylinder
from v4_body import INSERTS

PCB_MOUNTS=[(x,y) for x in (-32,32) for y in (-24,24)]
COVER_MOUNTS=[(-68,0),(68,0)]

def rounded_box(x,y,z,dx,dy,dz,r):
    return cq.Workplane(obj=box(x,y,z,dx,dy,dz)).edges('|Z').fillet(r).val()

def insert_hole(s,x,y,top):
    return s.cut(cylinder((x,y,top+.01),(0,0,-1),2,7.01),
                 cylinder((x,y,top+.01),(0,0,-1),2.25,.46))

def carrier():
    # Print dorsal face down. All four posts grow from the same flat plate.
    s=rounded_box(0,0,78,145,78,4,10)
    for x,y in INSERTS:
        s=s.fuse(cylinder((x,y,32),(0,0,1),6,46))
        # A recessed M3x10 seats at Z=36 and reaches the chassis insert at 32.
        s=s.cut(cylinder((x,y,31.9),(0,0,1),1.7,4.2),
                cylinder((x,y,36),(0,0,1),3.2,50))
    for x,y in COVER_MOUNTS:
        s=s.fuse(cylinder((x,y,71),(0,0,1),5.5,9))
        s=insert_hole(s,x,y,80)
    for x,y in PCB_MOUNTS:
        s=s.fuse(cylinder((x,y,71),(0,0,1),5.2,9))
        s=insert_hole(s,x,y,80)
    for x in (-24,0,24):
        for y in (-12,0,12):
            s=s.cut(cq.Workplane('XY').slot2D(17,3.4).extrude(6).val().translate((x,y,75)))
    for x in (-52,52):
        s=s.cut(cq.Workplane('XY').slot2D(20,8,90).extrude(6).val().translate((x,0,75)))
    return s.clean()

def section(z,cx,rx,ry,n=2.7):
    points=[]
    for k in range(64):
        a=2*math.pi*k/64;c=math.cos(a);s=math.sin(a)
        points.append((cx+rx*math.copysign(abs(c)**(2/n),c),ry*math.copysign(abs(s)**(2/n),s)))
    return cq.Workplane('XY').spline(points,periodic=True).close().val().translate((0,0,z))

def loft(inner=False):
    sections=[(32.4,0,92,58),(57,0,92,58),(82,-1,90,57),(100,-3,82,52),
              (112,-5,65,43),(120,-6,37,26),(123,-6,8,6)]
    if inner:
        sections=[(z,cx,rx-2.6,ry-2.6) for z,cx,rx,ry in sections[:-2]]+[(117.4,-6,35,24),(120.4,-6,5.4,3.4)]
    w=cq.Workplane('XY')
    for row in sections:w=w.add(section(*row))
    return w.toPending().loft(combine=True,ruled=False).val()

def canopy():
    outer=loft();s=outer.cut(loft(True))
    for x,y in COVER_MOUNTS:
        tab=rounded_box(math.copysign(76,x),0,82,32,20,4,3).intersect(outer)
        s=s.fuse(tab).cut(cylinder((x,y,79.9),(0,0,1),1.7,4.2),
                         cylinder((x,y,84),(0,0,1),3.5,50))
    for x in (-36,-18,0,18,36):
        vent=cq.Workplane('XZ',origin=(x,75,99-.05*abs(x))).slot2D(12,3.5,12 if x>0 else -12).extrude(150).val()
        s=s.cut(vent)
    s=s.cut(rounded_box(-88,0,48,34,26,16,3))
    return s.clean()

def pcb():
    s=rounded_box(0,0,86.8,90,60,1.6,2)
    for x,y in PCB_MOUNTS:s=s.cut(cylinder((x,y,85.9),(0,0,1),1.7,2))
    return s

def controller_envelope():
    return box(0,0,95,90,60,18)

def pcb_standoff():
    # A functional 6 mm PCB air gap; structural links still have no shims.
    return cylinder((0,0,0),(0,0,1),4.5,6).cut(cylinder((0,0,-.1),(0,0,1),1.7,6.2))
