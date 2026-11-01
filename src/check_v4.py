"""Validate v4 component equivalence, dorsal fit, hardware access and motion."""
from pathlib import Path
import json, math, hashlib, itertools, re
from functools import lru_cache
import cadquery as cq
import trimesh
from v4_body import ANCHORS, place
from v4_leg import COXA, FEMUR, HIP_Z, box, cylinder
from v4_dorsal import INSERTS, PCB_MOUNTS, COVER_MOUNTS, controller_envelope

ROOT=Path(__file__).resolve().parent.parent;OUT=ROOT/'build/v4';CACHE=OUT/'.cad_cache'
@lru_cache(maxsize=2048)
def bounds(s):return s.BoundingBox()
def overlap(a,b):
    aa,bb=bounds(a),bounds(b)
    if any(getattr(aa,k+'max')<=getattr(bb,k+'min')+1e-5 or getattr(bb,k+'max')<=getattr(aa,k+'min')+1e-5 for k in 'xyz'):return 0.
    return max(0.,a.intersect(b).Volume())
def turn(s,a):return s.rotate((0,0,0),(0,1,0),-a)
def leg_pose(parts,yaw,hip,knee):
    fem=lambda s:turn(s,hip).translate((COXA,0,HIP_Z))
    tib=lambda s:fem(turn(s,knee).translate((FEMUR,0,0)))
    return [s.rotate((0,0,0),(0,0,1),yaw) for s in (parts['04_coxa'],fem(parts['05_femur']),tib(parts['06_tibia']),tib(parts['07_tpu_shoe']))]

def main():
    print('Loading v4 CAD.',flush=True)
    meta=json.loads((CACHE/'placed.json').read_text())
    parts={p.stem:cq.importers.importStep(str(p)).val() for p in sorted((OUT/'STEP_parts').glob('*.step'))}
    report={'revision':'v4','checks':{},'step_sha256':{},'stl_sha256':{}};c=report['checks']
    c['parts']={}
    for n,s in parts.items():
        m=trimesh.load(OUT/'STL'/f'{n}.stl');atbed=m.vertices[m.faces][:,:,2].max(axis=1)<.001
        c['parts'][n]=dict(valid=s.isValid(),solids=len(s.Solids()),watertight=m.is_watertight,
            mesh_components=len(m.split(only_watertight=False)),bed_area_mm2=float(m.area_faces[atbed].sum()))
        report['step_sha256'][n]=hashlib.sha256((OUT/'STEP_parts'/f'{n}.step').read_bytes()).hexdigest()
        report['stl_sha256'][n]=hashlib.sha256((OUT/'STL'/f'{n}.stl').read_bytes()).hexdigest()
    c['validated_component_difference_mm3']={}
    for n,base in [('01_chassis','chassis'),('04_coxa','coxa'),('05_femur','femur'),('06_tibia','tibia'),('07_tpu_shoe','shoe')]:
        ref=cq.importers.importStep(str(ROOT/'cad/reference/validated_v4'/f'{base}.step')).val();s=parts[n]
        # Coincident complex STEP surfaces can defeat OCC subtraction. Compare the
        # entire STEP entity stream first, excluding only the generated product label.
        normalize=lambda p:re.sub(r'Open CASCADE STEP translator 7\.9 \d+', 'CAD part',p.read_text().split('DATA;')[1])
        identical=normalize(OUT/'STEP_parts'/f'{n}.step')==normalize(ROOT/'cad/reference/validated_v4'/f'{base}.step')
        c['validated_component_difference_mm3'][n]=0. if identical else s.cut(ref).Volume()+ref.cut(s).Volume()
    print('Checked print solids and exact equivalence to validated v4 components.',flush=True)
    objects={o['name']:(o,cq.Shape.importBrep(str(CACHE/o['file']))) for o in meta}
    structural=[(n,o,s) for n,(o,s) in objects.items() if o['kind'] in ('printed','reference','servo')]
    c['dorsal_intersections']=[]
    dorsal=['01_CHASSIS','02_CONTROLLER_CARRIER','05_SHELL','REF_3S_BATTERY_115x40x35','REF_CONTROLLER_PCB_90x60','REF_CONTROLLER_COMPONENTS']
    for a,b in itertools.combinations(dorsal,2):
        v=overlap(objects[a][1],objects[b][1])
        if v>.05:c['dorsal_intersections'].append([a,b,v])
    c['controller_envelope_overlap_mm3']=max(overlap(parts[n],controller_envelope()) for n in ('01_chassis','02_controller_carrier','03_organic_canopy'))
    c['fastener_print_intersections']=[]
    for n,(o,s) in objects.items():
        if o['kind'] not in ('fastener','factory_fastener'):continue
        for target,t,shape in structural:
            if t['kind']=='servo':continue # Thread engagement is intentional.
            v=overlap(s,shape)
            if v>.05:c['fastener_print_intersections'].append([n,target,v])
    print('Checked dorsal packaging and all screw/print clearances.',flush=True)
    c['dorsal_driver_obstructions']=[]
    for kind,points,z in [('carrier',INSERTS,37.7),('PCB',PCB_MOUNTS,89.3),('cover',COVER_MOUNTS,85.7)]:
        for x,y in points:
            tool=cylinder((x,y,z),(0,0,1),2.5,80)
            for n in ('01_CHASSIS','02_CONTROLLER_CARRIER','05_SHELL','REF_3S_BATTERY_115x40x35','REF_CONTROLLER_PCB_90x60','REF_CONTROLLER_COMPONENTS'):
                if n=='05_SHELL' and kind!='cover':continue
                v=overlap(tool,objects[n][1])
                if v>.05:c['dorsal_driver_obstructions'].append([kind,x,y,n,v])
    # All six legs; broad phase rejects cases that cannot reach the upper shell.
    c['dorsal_motion_max_overlap_mm3']=0.;worst=None
    for i,a in enumerate(ANCHORS):
        for yaw in (-60,0,60):
            for hip in (-30,20,45):
                for knee in (-105,-85,15):
                    for moving in leg_pose(parts,yaw,hip,knee):
                        moved=place(moving,a)
                        for fixed in ('02_controller_carrier','03_organic_canopy'):
                            v=overlap(moved,parts[fixed])
                            if v>c['dorsal_motion_max_overlap_mm3']:
                                c['dorsal_motion_max_overlap_mm3']=v;worst=[i+1,yaw,hip,knee,fixed]
        print('Checked dorsal motion at leg',i+1,flush=True)
    c['dorsal_motion_worst_pose']=worst
    c['body_and_tibia_symmetry_mm3']={n:parts[n].cut(parts[n].mirror('XZ')).Volume()+parts[n].mirror('XZ').cut(parts[n]).Volume() for n in ('01_chassis','06_tibia')}
    report['passed']=(all(p['valid'] and p['solids']==1 and p['watertight'] and p['mesh_components']==1 and p['bed_area_mm2']>10 for p in c['parts'].values())
        and max(c['validated_component_difference_mm3'].values())<.05 and not c['dorsal_intersections']
        and c['controller_envelope_overlap_mm3']<.05 and not c['fastener_print_intersections']
        and not c['dorsal_driver_obstructions'] and c['dorsal_motion_max_overlap_mm3']<.05
        and max(c['body_and_tibia_symmetry_mm3'].values())<.05)
    (OUT/'integration_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:raise SystemExit(1)

if __name__=='__main__':main()
