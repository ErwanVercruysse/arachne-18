from pathlib import Path
"""Inspect the actual CAD; never substitutes a photograph or generated concept."""
import json
import sys
import math
import cadquery as cq
import trimesh
from v4_leg import COXA, HIP_Z, FEMUR, INNER, OUTER, HORN_BOLTS, RECESS, CASE_SEAT, box, cylinder


OUT=Path(__file__).resolve().parent.parent/'build/v4'
PART_NAMES={'01_coxa':'04_coxa','02_femur':'05_femur','03_tibia':'06_tibia','04_tpu_shoe':'07_tpu_shoe'}

def read_parts():
    return {n:cq.importers.importStep(str(OUT/"STEP_parts"/f"{PART_NAMES[n]}.step")).val()
            for n in ("01_coxa","02_femur","03_tibia","04_tpu_shoe")}


def reference(path):
    a=cq.importers.importStep(path).val().rotate((0,0,0),(0,1,0),180).translate((-25.5,9.3,0))
    return cq.Compound.makeCompound(a.Solids()[:6])


def overlap(a,b):
    return max(0,a.intersect(b).Volume())


def turn(s,angle):
    return s.rotate((0,0,0),(0,1,0),-angle)


def main(path):
    p=read_parts(); c,f,t=p["01_coxa"],p["02_femur"],p["03_tibia"]
    native=reference(path)
    # Owner measured Ø6.5, whereas this reference STEP uses a Ø6 dummy tip.
    case=cq.Compound.makeCompound([native,cylinder((0,-18.9,0),(0,1,0),3.25,2.65)])
    yaw=case.rotate((0,0,0),(1,0,0),90)
    case_heads=cq.Compound.makeCompound([cylinder((x,CASE_SEAT,z),(0,1,0),2,1.6)
                for x in (-29,-8.3) for z in (-10.25,10.25)])
    plug=box(-15.75,-22.55,0,10.9,12.5,20.9)
    import hashlib
    out={"prototype":"v4","units":"mm","dummy_shaft_checked_diameter_mm":6.5,
         "reference_sha256":hashlib.sha256(open(path,"rb").read()).hexdigest(),
         "step_sha256":{n:hashlib.sha256((OUT/"STEP_parts"/f"{PART_NAMES[n]}.step").read_bytes()).hexdigest() for n in p},"checks":{}}
    checks=out["checks"]
    checks["fixed_case_overlap_mm3"]={"yaw":overlap(c,yaw),
           "hip":overlap(c,case.translate((COXA,0,HIP_Z))),
           "knee":overlap(f,case.translate((FEMUR,0,0)))}
    # Check lateral insertion from the dummy side, with horns removed.
    checks["case_insertion_overlap_mm3"]={}
    for name,part,joint,z in (("hip",c,COXA,HIP_Z),("knee",f,FEMUR,0)):
        values=[(dy,overlap(part,case.translate((joint,dy,z)))) for dy in range(-60,1,3)]
        checks["case_insertion_overlap_mm3"][name]=max(values,key=lambda a:a[1])
    checks["yaw_sweep"]={}
    for angle in range(-60,61,5):
        moved=c.rotate((0,0,0),(0,0,1),angle)
        checks["yaw_sweep"][angle]={"case":overlap(moved,yaw),
                  "plug":overlap(moved,plug.rotate((0,0,0),(1,0,0),90))}
    checks["hip_sweep"]={}
    for angle in range(-30,46,5):
        moved=turn(f,angle).translate((COXA,0,HIP_Z))
        checks["hip_sweep"][angle]={"case":overlap(moved,case.translate((COXA,0,HIP_Z))),
            "plug":overlap(moved,plug.translate((COXA,0,HIP_Z))),"parent":overlap(moved,c),
            "case_screw_heads":overlap(moved,case_heads.translate((COXA,0,HIP_Z)))}
    checks["knee_sweep"]={}
    for angle in range(-105,16,5):
        moved=turn(t,angle).translate((FEMUR,0,0))
        checks["knee_sweep"][angle]={"case":overlap(moved,case.translate((FEMUR,0,0))),
            "plug":overlap(moved,plug.translate((FEMUR,0,0))),"parent":overlap(moved,f),
            "case_screw_heads":overlap(moved,case_heads.translate((FEMUR,0,0)))}
    checks["tibia_mirror_difference_mm3"]=t.cut(t.mirror("XZ")).Volume()+t.mirror("XZ").cut(t).Volume()
    checks["horn_bearing_missing_samples"]={}
    for name,shape,axis in (("coxa",c,"z"),("femur",f,"y"),("tibia",t,"y")):
        misses=0
        for side in (-1,1):
            for u,v in HORN_BOLTS:
                for depth in (RECESS+.4,4.4):
                    for rad in (2.0,2.85):
                        for a in range(0,360,30):
                            x=u+rad*math.cos(math.radians(a)); z=v+rad*math.sin(math.radians(a))
                            q=(x,z,side*(OUTER-depth)) if axis=="z" else (x,side*(OUTER-depth),z)
                            if not shape.isInside(q,.005): misses+=1
        checks["horn_bearing_missing_samples"][name]=misses
    checks["parts"]={}
    for n,s in p.items():
        mesh=trimesh.load(OUT/"STL"/f"{PART_NAMES[n]}.stl")
        bottom=mesh.vertices[mesh.faces][:,:,2].max(axis=1)<.001
        checks["parts"][n]={"valid":s.isValid(),"solids":len(s.Solids()),
                 "watertight":mesh.is_watertight,"bed_area_mm2":float(mesh.area_faces[bottom].sum())}
    # Print and manufacturer meshes are distinct. Fasteners must not hit either.
    checks["hardware_clearance_mm3"]={}
    for name,shape,axis,fixed in (("coxa",c,"z",yaw),("femur",f,"y",case),("tibia",t,"y",case)):
        maximum=0
        for side in (-1,1):
            pos=lambda u,v,w:(u,v,w) if axis=="z" else (u,w,v)
            d=(0,0,-side) if axis=="z" else (0,-side,0)
            dout=tuple(-v for v in d)
            for u,v in HORN_BOLTS:
                seat=pos(u,v,side*(OUTER-RECESS))
                shank=cylinder(seat,d,1.5,6)
                head=cylinder(seat,dout,2.85,1.65)
                tool=cylinder(pos(u,v,side*(OUTER+.2)),dout,2.5,25)
                maximum=max(maximum,overlap(shape,shank),overlap(shape,head),
                            overlap(shape,tool),overlap(fixed,shank))
        checks["hardware_clearance_mm3"][name+"_horn_screws"]=maximum
    # Case screw/tool access is checked with the outgoing link at zero degrees.
    for name,parent,child,joint,z in (("hip",c,f,COXA,HIP_Z),("knee",f,t,FEMUR,0)):
        maximum=0
        child=child.translate((joint,0,z))
        for x in (-29,-8.3):
            for zz in (-10.25,10.25):
                pilot=cylinder((joint+x,15.71,z+zz),(0,1,0),1.0,CASE_SEAT-15.72)
                driver=cylinder((joint+x,CASE_SEAT+.01,z+zz),(0,1,0),2.35,30)
                maximum=max(maximum,overlap(parent,pilot),overlap(parent,driver),overlap(child,driver))
        checks["hardware_clearance_mm3"][name+"_factory_screw_access"]=maximum
    checks["shoe_to_tibia_overlap_mm3"]=overlap(t,p["04_tpu_shoe"])
    foot_shank=cylinder((110,0,-19.6),(0,0,1),1.5,8)
    foot_head=cylinder((110,0,-21.25),(0,0,1),2.85,1.65)
    checks["hardware_clearance_mm3"]["foot_screw"]=max(overlap(t,foot_shank),overlap(p["04_tpu_shoe"],foot_shank),overlap(p["04_tpu_shoe"],foot_head))
    # Incoming fork slides over the fitted discs from the open +X end.
    checks["fork_insertion_overlap_mm3"]={}
    checks["fork_insertion_overlap_mm3"]["yaw"]=max(overlap(c.translate((dx,0,0)),yaw) for dx in range(0,61,3))
    for name,parent,child,joint,z in (("hip",c,f,COXA,HIP_Z),("knee",f,t,FEMUR,0)):
        maximum=0
        for dx in range(0,61,3):
            moved=child.translate((joint+dx,0,z))
            maximum=max(maximum,overlap(parent,moved),overlap(case.translate((joint,0,z)),moved))
        checks["fork_insertion_overlap_mm3"][name]=maximum
    checks["nonadjacent_combined_pose_overlap_mm3"]=0
    hip_case=case.translate((COXA,0,HIP_Z))
    for hip in (-30,0,20,45):
        knee_case=turn(case.translate((FEMUR,0,0)),hip).translate((COXA,0,HIP_Z))
        for knee in (-105,-60,0,15):
            tip=turn(turn(t,knee).translate((FEMUR,0,0)),hip).translate((COXA,0,HIP_Z))
            checks["nonadjacent_combined_pose_overlap_mm3"]=max(checks["nonadjacent_combined_pose_overlap_mm3"],
                overlap(tip,c),overlap(tip,hip_case),overlap(knee_case,c))
    # Report section geometry only. This is not a material/load certification.
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    sections=[]
    for n,xs in (("02_femur",(10,15,20,25,35)),("03_tibia",(10,15,20,25,35,60,85,100))):
        for x in xs:
            slab=p[n].intersect(box(x,0,0,.2,100,100))
            g=GProp_GProps();BRepGProp.VolumeProperties_s(slab.wrapped,g)
            b=slab.BoundingBox();z=g.CentreOfMass().Z()
            iy=g.MatrixOfInertia().Value(2,2)/.2
            sections.append({"part":n,"x_mm":x,"area_mm2":round(g.Mass()/.2,2),
                        "Iyy_mm4":round(iy,2),"Zyy_mm3":round(iy/max(b.zmax-z,z-b.zmin),2)})
    out["solid_section_geometry_not_FEA"]=sections
    failures=[]
    for k in ("fixed_case_overlap_mm3","hardware_clearance_mm3","fork_insertion_overlap_mm3"):
        failures.extend(f"{k}: {n} = {v}" for n,v in checks[k].items() if v>.005)
    failures.extend(f"case insertion: {n} = {v}" for n,v in checks["case_insertion_overlap_mm3"].items() if v[1]>.005)
    for k in ("yaw_sweep","hip_sweep","knee_sweep"):
        failures.extend(f"{k} at {a}: {n} = {v}" for a,vals in checks[k].items() for n,v in vals.items() if v>.005)
    failures.extend(f"bearing ring: {n}" for n,v in checks["horn_bearing_missing_samples"].items() if v)
    for n,vals in checks["parts"].items():
        if not (vals["valid"] and vals["solids"]==1 and vals["watertight"] and vals["bed_area_mm2"]>200):failures.append(f"invalid print: {n}")
    if checks["tibia_mirror_difference_mm3"]>.001:failures.append("tibia symmetry")
    if checks["shoe_to_tibia_overlap_mm3"]>.005:failures.append("shoe locating pins")
    if checks["nonadjacent_combined_pose_overlap_mm3"]>.005:failures.append("nonadjacent links")
    out["passed"]=not failures
    out["failures"]=failures
    (OUT/"leg_fit.json").write_text(json.dumps(out,indent=2)+"\n")
    summary={k:v for k,v in checks.items() if "sweep" not in k}
    for k in ("yaw_sweep","hip_sweep","knee_sweep"):
        summary[k]={q:max((v[q],a) for a,v in checks[k].items()) for q in next(iter(checks[k].values()))}
    print(json.dumps(summary,indent=2))
    assert not failures, failures


if __name__=="__main__": main(sys.argv[1])
