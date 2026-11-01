from pathlib import Path
"""BRep fit and assembly checks for the isolated v4 chassis."""
import sys, json, math, hashlib
from functools import lru_cache
import cadquery as cq
import trimesh
from v4_body import ANCHORS, TOP, INSERTS, place, box, cylinder
from v4_leg import COXA, HIP_Z, FEMUR, CASE_SEAT, OUTER, RECESS, HORN_BOLTS
from check_v4_leg import read_parts, reference, turn

OUT=Path(__file__).resolve().parent.parent/'build/v4'

@lru_cache(maxsize=4096)
def bounds(s):
    return s.BoundingBox()

def intersects(a,b):
    aa,bb=bounds(a),bounds(b)
    if any(getattr(aa,k+'max')<=getattr(bb,k+'min')+.00001 or
           getattr(bb,k+'max')<=getattr(aa,k+'min')+.00001 for k in 'xyz'):return 0.0
    return max(0.,a.intersect(b).Volume())

def pieces(p,case,yaw=0,hip=20,knee=-85):
    f=lambda s:turn(s,hip).translate((COXA,0,HIP_Z))
    t=lambda s:f(turn(s,knee).translate((FEMUR,0,0)))
    result={'coxa':p['01_coxa'],'femur':f(p['02_femur']),
            'tibia':t(p['03_tibia']),'shoe':t(p['04_tpu_shoe']),
            'hip_case':case.translate((COXA,0,HIP_Z)),
            'knee_case':f(case.translate((FEMUR,0,0)))}
    result={n:s.rotate((0,0,0),(0,0,1),yaw) for n,s in result.items()}
    result['yaw_case']=case.rotate((0,0,0),(1,0,0),90)
    return result

def group_overlap(left,right):
    # Component broad phase avoids expensive whole-compound booleans across
    # internally overlapping manufacturer reference solids.
    return sum(intersects(a,b) for n,a in left.items() if n!='shoe'
               for m,b in right.items() if m!='shoe')

def touch_angle(p,case):
    shoe=turn(p['04_tpu_shoe'].translate((FEMUR,0,0)),-20).translate((COXA,0,HIP_Z))
    lo,hi=45.,60.
    for _ in range(28):
        mid=(lo+hi)/2
        s=place(shoe.rotate((0,0,0),(0,0,1),-mid),ANCHORS[0])
        if s.BoundingBox().ymin>0:lo=mid
        else:hi=mid
    return (lo+hi)/2

def main(path):
    body=cq.importers.importStep(str(OUT/'STEP_parts/01_chassis.step')).val()
    p=read_parts(); native=reference(path)
    case=cq.Compound.makeCompound([native,cylinder((0,-18.9,0),(0,1,0),3.25,2.65)])
    yawcase=case.rotate((0,0,0),(1,0,0),90)
    plug=box(-15.75,0,-22.55,10.9,20.9,12.5)
    checks={}; report={'units':'mm','prototype':'body-v4','checks':checks,
       'step_sha256':hashlib.sha256((OUT/'STEP_parts/01_chassis.step').read_bytes()).hexdigest(),
       'stl_sha256':hashlib.sha256((OUT/'STL/01_chassis.stl').read_bytes()).hexdigest()}
    mesh=trimesh.load(OUT/'STL/01_chassis.stl')
    contact=mesh.vertices[mesh.faces][:,:,2].max(axis=1)<.001
    checks['body']={'valid':body.isValid(),'solids':len(body.Solids()),'watertight':mesh.is_watertight,
       'bed_area_mm2':float(mesh.area_faces[contact].sum()),'print_zmin':float(mesh.bounds[0,2])}
    checks['symmetry_difference_mm3']={axis:body.cut(body.mirror(axis)).Volume()+body.mirror(axis).cut(body).Volume() for axis in ('XZ','YZ')}
    checks['servo_installation']={}
    for i,a in enumerate(ANCHORS):
        cases=[intersects(body,place(yawcase.translate((0,0,dz)),a)) for dz in range(-60,1,5)]
        checks['servo_installation'][i+1]={'case_insertion_max_overlap_mm3':max(cases),
            'plug_overlap_mm3':intersects(body,place(plug,a)),
            'plug_service_column_overlap_mm3':intersects(body,place(box(-15.75,0,-50,11.4,21.4,70),a))}
    print('Checked six case insertion paths and dummy-side plug access.',flush=True)
    checks['factory_screw_access_max_overlap_mm3']=0.
    checks['horn_screw_access_max_overlap_mm3']=0.
    for a in ANCHORS:
        for x in (-29,-8.3):
            for y in (-10.25,10.25):
                for s in [cylinder((x,y,15.71),(0,0,1),1,2.53),
                          cylinder((x,y,CASE_SEAT+.01),(0,0,1),2.35,70)]:
                    checks['factory_screw_access_max_overlap_mm3']=max(checks['factory_screw_access_max_overlap_mm3'],intersects(body,place(s,a)))
        for side in (-1,1):
            for x,y in HORN_BOLTS:
                s=cylinder((x,y,side*(OUTER-RECESS+.01)),(0,0,side),3.0,65)
                checks['horn_screw_access_max_overlap_mm3']=max(checks['horn_screw_access_max_overlap_mm3'],intersects(body,place(s,a)))
    # Symmetric chassis has two distinct mount environments: diagonal and lateral.
    checks['yaw_mount_sweep']={}
    for i in (0,1):
        values=[]
        for yaw in range(-60,61,5):
            coxa=place(p['01_coxa'].rotate((0,0,0),(0,0,1),yaw),ANCHORS[i])
            values.append({'yaw':yaw,'overlap_mm3':intersects(body,coxa)})
        checks['yaw_mount_sweep'][i+1]=values
    print('Checked coxa sweep and screwdriver access.',flush=True)
    (OUT/'fit_checks.in_progress.json').write_text(json.dumps(report,indent=2)+'\n')
    checks['combined_pose_worst']={'overlap_mm3':0.}
    for i in (0,1):
        for yaw in (-60,-30,0,30,60):
            for hip in (-30,0,45):
                for knee in (-105,-45,15):
                    group=pieces(p,case,yaw,hip,knee)
                    for n,s in group.items():
                        v=intersects(body,place(s,ANCHORS[i]))
                        if v>checks['combined_pose_worst']['overlap_mm3']:
                            checks['combined_pose_worst']={'leg':i+1,'yaw':yaw,'hip':hip,'knee':knee,'part':n,'overlap_mm3':v}
        print('Checked combined poses at mount',i+1,flush=True)
        (OUT/'fit_checks.in_progress.json').write_text(json.dumps(report,indent=2)+'\n')
    return finish(report,p,case,body)

def finish(report,p,case,body):
    checks=report['checks']
    # Nominal standing pose: every pair, including diagonal/middle neighbours.
    neutral=[{n:place(s,a) for n,s in pieces(p,case).items()} for a in ANCHORS]
    checks['neutral_leg_pair_max_overlap_mm3']=max(group_overlap(neutral[i],neutral[j]) for i in range(6) for j in range(i+1,6))
    print('Checked neutral leg pairs.',flush=True)
    angle=touch_angle(p,case)
    checks['pair_touch']={'yaw_degrees':angle,'hip_degrees':-20,'knee_degrees':0,'pairs':{}}
    for title,left,right,leftsign in [('front',0,5,-1),('rear',2,3,1)]:
        worst=0.;bodyworst=0.;otherworst=0.
        for j in range(13):
            f=j/12
            groups=[]
            for idx,sgn in ((left,leftsign),(right,-leftsign)):
                g=pieces(p,case,sgn*angle*f,20-40*f,-85+85*f)
                groups.append({n:place(s,ANCHORS[idx]) for n,s in g.items()})
            worst=max(worst,group_overlap(*groups));bodyworst=max(bodyworst,*(intersects(body,s) for g in groups for n,s in g.items() if n!='shoe'))
            for idx in range(6):
                if idx in (left,right):continue
                otherworst=max(otherworst,*(group_overlap(g,neutral[idx]) for g in groups))
        checks['pair_touch']['pairs'][title]={'hard_parts_max_overlap_mm3':worst,
            'body_max_overlap_mm3':bodyworst,'other_legs_max_overlap_mm3':otherworst,
            'shoe_distance_mm':groups[0]['shoe'].distance(groups[1]['shoe']),
            'shoe_overlap_mm3':intersects(groups[0]['shoe'],groups[1]['shoe'])}
        (OUT/'fit_checks.in_progress.json').write_text(json.dumps(report,indent=2)+'\n')
        print('Checked',title,'touch path:',checks['pair_touch']['pairs'][title],flush=True)
    # Nominal payload is above the flat deck. Inserts expand their pilots
    # intentionally during installation and therefore are not interference tests.
    checks['battery_envelope_overlap_mm3']=intersects(body,box(0,0,49.5,115,40,35))
    checks['insert_driver_access_mm3']=max(intersects(body,cylinder((x,y,TOP+.01),(0,0,1),3.5,40)) for x,y in INSERTS)
    def overlaps(obj):
        if isinstance(obj,dict):
            for k,v in obj.items():
                if str(k).endswith('overlap_mm3') and isinstance(v,(int,float)):yield v
                else:yield from overlaps(v)
        elif isinstance(obj,list):
            for v in obj:yield from overlaps(v)
    report['passed']=checks['body']['valid'] and checks['body']['solids']==1 and checks['body']['watertight'] and max(overlaps(checks),default=0)<.05 and max(checks['symmetry_difference_mm3'].values())<.05
    report['passed']=report['passed'] and checks['insert_driver_access_mm3']<.05 and all(v['shoe_distance_mm']<.05 for v in checks['pair_touch']['pairs'].values())
    (OUT/'body_fit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:sys.exit(1)

if __name__=='__main__':main(sys.argv[1])
