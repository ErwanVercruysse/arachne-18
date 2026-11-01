"""ARACHNE v4 — independently built, monolithic one-leg print trial (mm).

Three rigid moving links, no cheek seams, spacers or structural assembly bolts.
The case-mount pattern and measured horn interfaces are the only reused geometry.
"""
from pathlib import Path
import json
import math
import cadquery as cq

OUT = Path(__file__).resolve().parent.parent / "build" / "v4_leg"
OUT.mkdir(parents=True,exist_ok=True)
COXA, HIP_Z, FEMUR = 62.0, -6.95, 78.0
INNER, OUTER = 18.45, 23.95  # 36.5 measured span + 0.4 fitting clearance
RECESS = 1.5                # 4 mm remains beneath each horn screw head
CASE_SEAT = 18.25           # 2.55 mm local bearing pads, inside fork entry
FLOOR, DECK = -17.0, -12.8
HORN_BOLTS = [(7*math.cos(math.radians(a)), 7*math.sin(math.radians(a)))
              for a in (45, 135, 225, 315)]


def box(cx, cy, cz, dx, dy, dz):
    return cq.Workplane("XY").box(dx, dy, dz).val().translate((cx,cy,cz))


def cylinder(p, d, r, length):
    return cq.Solid.makeCylinder(r, length, cq.Vector(*p), cq.Vector(*d))


def xy_profile(points, z, height, radius=0):
    w = cq.Workplane("XY").polyline(points).close().extrude(height)
    if radius:
        w = w.edges("|Z").fillet(radius)
    return w.val().translate((0,0,z))


def xz_profile(points, ymin, width, radius=0):
    # Construct flat in XY and rotate, to avoid Workplane XZ's negative normal.
    return (xy_profile(points, 0, width, radius)
            .rotate((0,0,0),(1,0,0),90).translate((0,ymin+width,0)))


def symmetric_plan(stations, z, height, radius=2):
    return xy_profile([(x,-y) for x,y in stations] +
                      [(x,y) for x,y in reversed(stations)],z,height,radius)


def horn_interface(shape, axis):
    """Complete four-hole disc seat, center access and recessed M3x6 heads."""
    cuts=[]
    for side in (-1,1):
        d=(0,0,-side) if axis=="z" else (0,-side,0)
        pos=lambda a,b,c:(a,b,c) if axis=="z" else (a,c,b)
        for u,v in HORN_BOLTS:
            cuts += [cylinder(pos(u,v,side*(OUTER+.1)),d,1.7,6),
                     cylinder(pos(u,v,side*(OUTER+.1)),d,3.15,RECESS+.1)]
        cuts.append(cylinder(pos(0,0,side*(OUTER+.1)),d,3.65,6))
        # Shallow entry track lets the protruding dummy shaft slide in from
        # the open end. The outer 3 mm under each head remains uninterrupted.
        entry=box(-9,side*(INNER+.495),0,18,1.01,7.3)
        if axis=="z":entry=entry.rotate((0,0,0),(1,0,0),90)
        cuts.append(entry)
        # Recess the inner outer-annulus, without removing any bolt's seat.
        din=(0,0,side) if axis=="z" else (0,side,0)
        p=pos(0,0,side*(INNER-.01))
        ring=cylinder(p,din,24.5,2.21).cut(cylinder(p,din,10.7,2.22))
        cuts.append(ring)
    return shape.cut(*cuts).clean()


def collar(joint):
    """Rear torque collar and floor, with lateral servo insertion from -Y.

    The four DRIVEN-face factory holes retain the case. The dummy face is
    open for lateral insertion of the complete case and connector access.
    """
    shell=box(joint-31.05,0,0,16.9,2*OUTER,34)
    shell=cq.Workplane(obj=shell).edges("|X").fillet(2.4).val()
    cavity=box(joint-12,-.65,0,47.2,37.5,25.6)
    shell=shell.cut(cavity)
    shell=shell.cut(box(joint-12,-30,0,47.2,60,25.6))
    # Local Ø6 bearing pads reach the recessed factory mounting faces. Their
    # contours leave a straight insertion path from the opposite, dummy side.
    for z in (-10.25,10.25):
        rail=box(joint-19.3,(16.9+CASE_SEAT)/2,z,26.4,CASE_SEAT-16.9,7.0)
        rail=cq.Workplane(obj=rail).edges("|Y").fillet(2).val()
        shell=shell.fuse(rail)
        for offset in (-29,-8.3):
            shell=shell.fuse(cylinder((joint+offset,15.7,z),(0,1,0),3.0,CASE_SEAT-15.7))
            shell=shell.cut(cylinder((joint+offset,15.6,z),(0,1,0),1.1,9),
                             cylinder((joint+offset,CASE_SEAT,z),(0,1,0),2.6,8))
    shell=shell.cut(cylinder((joint,-30,0),(0,1,0),10.95,60))
    return shell.clean()


def case_screw_passages(shape,joint,zc):
    for x in (-29,-8.3):
        for z in (-10.25,10.25):
            shape=shape.cut(cylinder((joint+x,15.6,zc+z),(0,1,0),1.1,10),
                            cylinder((joint+x,CASE_SEAT,zc+z),(0,1,0),2.6,10))
    return shape.clean()


def cable_relief(shape, axis, lo, hi):
    """Mirror the conservative plug swept envelope to preserve fork symmetry.

    The plug housings, unlike the wires, cannot pass through a rotating cheek.
    A 0.25 mm envelope expansion covers intermediate sampled orientations.
    """
    # An analytic sector conservatively contains every intermediate position
    # of the rectangular plug; there are no discrete scallops to snag a wire.
    start=math.atan2(10.7,-10.05)-math.radians(hi)
    end=2*math.pi-math.atan2(10.7,-10.05)-math.radians(lo)
    mid=(start+end)/2
    q=lambda r,a:(r*math.cos(a),r*math.sin(a))
    # Extend the relief to an open edge: no thin crescent tabs remain outside
    # the actual plug sweep. Those tabs add support and no useful load path.
    profile=(cq.Workplane("XY").moveTo(*q(10.05,start)).lineTo(*q(60.0,start))
       .threePointArc(q(60.0,mid),q(60.0,end)).lineTo(*q(10.05,end))
       .threePointArc(q(10.05,mid),q(10.05,start)).close().extrude(13.0).val())
    swept=profile.rotate((0,0,0),(1,0,0),90).translate((0,-16.05,0))
    cuts=[swept,swept.mirror("XZ")]
    if axis=="z": cuts=[s.rotate((0,0,0),(1,0,0),90) for s in cuts]
    return shape.cut(*cuts).clean()


def coxa():
    # Balanced yaw fork. A central full-depth bulkhead carries both cheeks
    # into the rear servo collar; no outboard wall or removable saddle.
    outline=[(-14.5,-7),(-11,-12.5),(-4,-14.5),(8,-14.5),
             (23,-23.95),(37,-23.95),(40,-20),(40,20),(37,23.95),
             (23,23.95),(8,14.5),(-4,14.5),(-11,12.5),(-14.5,7)]
    lower=xy_profile(outline,-OUTER,OUTER-INNER,2)
    upper=xy_profile(outline,INNER,OUTER-INNER,2)
    # Upper cheek ends at the central bulkhead, clear of the hip body.
    upper=upper.intersect(box(0,0,22,54,60,10))
    wall=box(23.5,0,0,5,2*OUTER,2*OUTER)
    wall=cq.Workplane(obj=wall).edges("|Z").fillet(2).val()
    rear=collar(COXA).translate((0,0,HIP_Z))
    sole=symmetric_plan([(21,OUTER),(38,OUTER),(44,16),(COXA-3,16)],
                         -OUTER,4.4,2)
    body=lower.fuse(upper,wall,sole).clean()
    # Clear the real servo cavity through the former lower yaw tongue.
    body=body.cut(box(COXA-12,-.65,HIP_Z,48,37.5,25.6),
                   box(COXA-12,-30,HIP_Z,47.2,60,25.6))
    body=horn_interface(body.fuse(rear).clean(),"z")
    return case_screw_passages(cable_relief(body,"z",-63,63),COXA,HIP_Z)


def femur():
    # Both upright sides start on the same continuous bed plane. The lower
    # web transfers load into a short closed servo collar, not narrow posts.
    profile=[(-14.5,-17),(-14.5,5),(-12,11),(-6,14.5),(5,14.5),
             (18,11),(30,10),(43,17),(55.4,17),(55.4,-17)]
    cheek=xz_profile(profile,INNER,OUTER-INNER,2)
    pair=cheek.fuse(cheek.mirror("XZ"))
    sole=symmetric_plan([(22,OUTER),(43,OUTER),(51,OUTER),(56,16),
                         (FEMUR-3,16)],FLOOR,DECK-FLOOR,2)
    # Broad 45-degree internal root haunches spread loads into the floor.
    ribs=[]
    for side in (-1,1):
        tri=cq.Workplane("YZ").polyline([(side*INNER,-12.8),
                  (side*(INNER-3.0),-12.8),(side*INNER,-9.8)]).close().extrude(19).val()
        ribs.append(tri.translate((23,0,0)))
    body=pair.fuse(sole,*ribs).clean()
    body=body.cut(box(FEMUR-12,-.65,0,48,37.5,25.6),
                   box(FEMUR-12,-30,0,47.2,60,25.6))
    body=horn_interface(body.fuse(collar(FEMUR)).clean(),"y")
    return case_screw_passages(cable_relief(body,"y",-33,48),FEMUR,0)


def tibia():
    # A single solid with a level underside: stout, curved fork arms merge
    # into a central tapered toe, rather than thin bars and a separate foot.
    # Smooth outer splines retain the plane underneath and are mirrored
    # exactly about Y=0. The taper is part of the load-bearing beam itself.
    side=(cq.Workplane("XY").moveTo(-14.5,-17).lineTo(-14.5,5)
          .spline([(-6,14.5),(7,14.5),(26,12),(49,8),(76,1),(101,-4),
                   (114,-7),(119,-12),(116,-17)],includeCurrent=True)
          .close().extrude(2*OUTER).val()
          .rotate((0,0,0),(1,0,0),90).translate((0,OUTER,0)))
    edge=[(19,-OUTER),(42,-18.7),(65,-12.8),(89,-10.5),(109,-9),(118,-5),(119,0)]
    plan=(cq.Workplane("XY").moveTo(-14.5,-OUTER).lineTo(19,-OUTER)
          .spline(edge[1:],includeCurrent=True)
          .spline([(x,-y) for x,y in reversed(edge[:-1])],includeCurrent=True)
          .lineTo(-14.5,OUTER).close().extrude(40).val().translate((0,0,-20)))
    body=side.intersect(plan)
    # Rounded closed end of the fork slot reduces the notch stress.
    slot=xy_profile([(-20,-INNER),(19,-INNER),(50,-6),(53,0),
                     (50,6),(19,INNER),(-20,INNER)],-25,60,3)
    body=body.cut(slot)
    # One replaceable TPU shoe can be retained with an M3 insert. This is
    # the only plastic thread in v4; all servo screws engage real metal/case.
    body=body.cut(cylinder((110,0,-17.01),(0,0,1),2.0,7.01),
                   cylinder((110,0,-17.01),(0,0,1),2.25,.46))
    for y in (-5,5):
        body=body.cut(cylinder((104,y,-17.01),(0,0,1),1.8,3.01))
    return cable_relief(horn_interface(body,"y"),"y",-108,18)


def shoe():
    pad=box(109,0,-19.5,20,19,5)
    pad=cq.Workplane(obj=pad).edges("|Z").fillet(4).val()
    pad=pad.cut(cylinder((110,0,-22.1),(0,0,1),1.7,5.2),
                cylinder((110,0,-22.1),(0,0,1),3.2,2.5))
    for y in (-5,5):
        pad=pad.fuse(cylinder((104,y,-17.01),(0,0,1),1.6,2.51))
    return pad.clean()


def build():
    return {"01_coxa":coxa(),"02_femur":femur(),"03_tibia":tibia(),
            "04_tpu_shoe":shoe()}


def on_bed(shape):
    return shape.translate((0,0,-shape.BoundingBox().zmin))
