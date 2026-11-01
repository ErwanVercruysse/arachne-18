"""DIN 7380 button-heads, factory screws and measured internal horns (mm)."""
import cadquery as cq
from v4_leg import cylinder, HORN_BOLTS

def screw(p, direction, length=6, case=False):
    # Shank axis points inward; socket faces out toward the driver.
    r,h=(2.0,1.6) if case else (2.85,1.65)
    b=cq.Workplane("XY").circle(r).extrude(h).edges(">Z").fillet(.45).val()
    b=b.fuse(cylinder((0,0,-length),(0,0,1),1 if case else 1.5,length))
    if case:
        cut=(cq.Workplane("XY").rect(2.6,.6).extrude(1.2).val()
             .fuse(cq.Workplane("XY").rect(.6,2.6).extrude(1.2).val()).translate((0,0,.6)))
    else:
        cut=cq.Workplane("XY").polygon(6,2.31).extrude(1.2).val().translate((0,0,.6))
    b=b.cut(cut)
    if direction==(0,-1,0): b=b.rotate((0,0,0),(1,0,0),-90)
    elif direction==(0,1,0): b=b.rotate((0,0,0),(1,0,0),90)
    elif direction==(0,0,1): b=b.rotate((0,0,0),(1,0,0),180)
    return b.translate(p)


def horn(axis,side):
    thick=2.5 if side==1 else 2.0
    start=18.25-thick if side==1 else -18.25
    p=lambda u,v,w: (u,v,w) if axis=="z" else (u,w,v)
    d=(0,0,1) if axis=="z" else (0,1,0)
    s=cylinder(p(0,0,start),d,9.975,thick)
    s=s.cut(cylinder(p(0,0,start-.1),d,3.3,thick+.2))
    for x,z in HORN_BOLTS: s=s.cut(cylinder(p(x,z,start-.1),d,1.5,thick+.2))
    return s
