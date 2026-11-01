"""Valid exterior servo reference for assembly and rendering; units mm.

Mount locations follow the vendor STEP. This simplified reference intentionally
omits internal motor/gear geometry. Fit validation retains the original vendor
STEP, which contains an invalid subsolid that must not enter native CAD imports.
"""
import cadquery as cq
from v4_leg import box, cylinder

def servo():
    s=box(-12.5,-.3,0,45.22,31.6,24.72)
    s=cq.Workplane(obj=s).edges('|Y').fillet(2.2).val()
    s=s.fuse(cylinder((0,15.5,0),(0,1,0),5.2,1.9),
             cylinder((0,-18.9,0),(0,1,0),3.25,2.8))
    for side,xs,face in ((1,(-29,-8.3),15.5),(-1,(-32.75,-8.3),-16.1)):
        for x in xs:
            for z in (-10.25,10.25):
                s=s.cut(cylinder((x,face+side*.01,z),(0,-side,0),.8,4))
    for z in (-5.25,5.25):s=s.cut(box(-15.75,-15.55,z,10.9,1.3,9.4))
    # Central retainers/shaft access are visible through the printed forks.
    s=s.cut(cylinder((0,17.5,0),(0,-1,0),1.3,4),
            cylinder((0,-19,0),(0,1,0),1.3,4))
    return s.clean()
