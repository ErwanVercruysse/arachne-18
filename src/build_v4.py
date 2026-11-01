"""Build the complete ARACHNE v4 robot, with all coordinates in millimetres."""
from pathlib import Path
import argparse, csv, json, math, zipfile
import cadquery as cq
import numpy as np
from v4_leg import build as build_leg, COXA, HIP_Z, FEMUR, OUTER, RECESS, HORN_BOLTS, CASE_SEAT, box, cylinder
from v4_body import build as build_body, ANCHORS, INSERTS, place, bed
from v4_dorsal import carrier, canopy, pcb, pcb_standoff, PCB_MOUNTS, COVER_MOUNTS, controller_envelope
from v4_hardware import screw, horn
from v4_servo import servo

ROOT=Path(__file__).resolve().parent.parent
OUT=ROOT/'build/v4'
P=dict(revision='v4',body_rx=97,body_ry=91,anchor_rx=86,anchor_ry=86,
       coxa=COXA,hip_z=HIP_Z,femur=FEMUR,tibia=110,femur_up_deg=20,knee_relative_deg=-85,
       tibia_down_deg=65,yaw_inward_limit_deg=60,yaw_outward_limit_deg=35,yaw_middle_limit_deg=35,
       hip_absolute_limits_deg=[-30,45],knee_absolute_limits_deg=[-105,15],
       horn_outer_span=36.5,horn_thickness_driven=2.5,horn_thickness_dummy=2.0,
       cheek_gap=36.9,cheek_t=5.5,shaft_clearance_d=7.3,dummy_shaft_d=6.5,
       insert_hole_d=4.0,insert_depth=7.0,insert_length=5.7,
       button_head_d=5.7,button_head_h=1.65,horn_recess_d=6.3,horn_recess_depth=1.5,
       battery_L=115,battery_W=40,battery_H=35,battery_bottom_z=32,
       controller_envelope_mm=[90,60,18],controller_bottom_z=86,
       controller_mount_spacing_mm=[64,48],shell_mounts_mm=COVER_MOUNTS)
TEAL=(.13,.46,.44); ORANGE=(.82,.46,.21); DARK=(.10,.12,.14); METAL=(.68,.71,.74); BRASS=(.73,.52,.19)

def turn(s,a):return s.rotate((0,0,0),(0,1,0),-a)
def insert(x,y,z):
    return cylinder((x,y,z-5.7),(0,0,1),2.3,5.7).cut(cylinder((x,y,z-5.71),(0,0,1),1.55,5.72))

def build(output=OUT):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    for name in ('STL','STEP_parts','.cad_cache'): (output/name).mkdir(exist_ok=True)
    print('Building v4 monolithic links, chassis and dorsal parts.',flush=True)
    leg=build_leg();body=build_body();electronics=carrier();cover=canopy();standoff=pcb_standoff()
    parts={'01_chassis':(body,1,'PLA/PETG',True),
           '02_controller_carrier':(electronics,1,'PLA/PETG',True),
           '03_organic_canopy':(cover,1,'PLA/PETG',False),
           '04_coxa':(leg['01_coxa'],6,'PLA/PETG',False),
           '05_femur':(leg['02_femur'],6,'PLA/PETG',False),
           '06_tibia':(leg['03_tibia'],6,'PLA/PETG',False),
           '07_tpu_shoe':(leg['04_tpu_shoe'],6,'TPU95A',False),
           '08_pcb_standoff':(standoff,4,'PLA/PETG',False)}
    items=[]
    def add(name,s,group='BODY',kind='printed',color=TEAL,mass_kg=None,**extra):
        assert s.isValid() and len(s.Solids())==1,(name,'invalid or multi-solid')
        items.append(dict(name=name,shape=s,group=group,kind=kind,color=list(color),mass_kg=mass_kg,**extra))
    add('01_CHASSIS',body);add('02_CONTROLLER_CARRIER',electronics);add('05_SHELL',cover)
    battery=cq.Workplane(obj=box(0,0,49.5,115,40,35)).edges('|Z').fillet(2).val()
    add('REF_3S_BATTERY_115x40x35',battery,kind='reference',color=(.65,.42,.12),mass_kg=.190)
    add('REF_CONTROLLER_PCB_90x60',pcb(),kind='reference',color=(.13,.34,.20),mass_kg=.010)
    add('REF_CONTROLLER_COMPONENTS',box(0,0,94.6,65,38,14),kind='reference',color=(.19,.21,.25),mass_kg=.035)
    for x,y in INSERTS:
        add(f'BODY_carrier_insert_{x}_{y}',insert(x,y,32),kind='insert',color=BRASS)
        add(f'BODY_carrier_M3x10_{x}_{y}',screw((x,y,36),(0,0,-1),10),kind='fastener',color=METAL)
    for x,y in COVER_MOUNTS:
        add(f'BODY_cover_insert_{x}_{y}',insert(x,y,80),kind='insert',color=BRASS)
        add(f'BODY_cover_M3x10_{x}_{y}',screw((x,y,84),(0,0,-1),10),kind='fastener',color=METAL)
    for x,y in PCB_MOUNTS:
        add(f'BODY_PCB_standoff_{x}_{y}',standoff.translate((x,y,80)))
        add(f'BODY_PCB_insert_{x}_{y}',insert(x,y,80),kind='insert',color=BRASS)
        add(f'BODY_PCB_M3x12_{x}_{y}',screw((x,y,87.6),(0,0,-1),12),kind='fastener',color=METAL)
    sv=servo()
    for i,anchor in enumerate(ANCHORS,1):
        root=lambda s:place(s,anchor)
        at_hip=lambda s:root(s.translate((COXA,0,HIP_Z)))
        fem=lambda s:at_hip(turn(s,P['femur_up_deg']))
        at_knee=lambda s:fem(s.translate((FEMUR,0,0)))
        tib=lambda s:at_knee(turn(s,P['knee_relative_deg']))
        g=f'L{i}'
        add(g+'_coxa',root(leg['01_coxa']),g+'_COXA')
        add(g+'_femur',fem(leg['02_femur']),g+'_FEMUR',color=ORANGE)
        add(g+'_tibia',tib(leg['03_tibia']),g+'_TIBIA')
        add(g+'_foot',tib(leg['04_tpu_shoe']),g+'_TIBIA',color=DARK)
        yaw_pose=lambda s:root(s.rotate((0,0,0),(1,0,0),90))
        for joint,pose,link in [('YAW',yaw_pose,'BODY'),('HIP',at_hip,g+'_COXA'),('KNEE',at_knee,g+'_FEMUR')]:
            # Store a complete collision reference transform rather than guessing it in the exporter.
            origin=np.array(pose(cq.Vertex.makeVertex(0,0,0)).Center().toTuple())
            axes=[]
            for v in ((1,0,0),(0,1,0),(0,0,1)):
                axes.append(np.array(pose(cq.Vertex.makeVertex(*v)).Center().toTuple())-origin)
            add(g+'_'+joint+'_STS3215',pose(sv),link,'servo',(.16,.18,.20),.055,
                servo_frame_origin_mm=origin.tolist(),servo_frame_rotation=np.array(axes).T.tolist())
        for joint,axis,pose,link in [('yaw','z',root,g+'_COXA'),('hip','y',fem,g+'_FEMUR'),('knee','y',tib,g+'_TIBIA')]:
            for side in (-1,1):
                add(f'{g}_{joint}_horn_{side}',pose(horn(axis,side)),link,'hardware',METAL,.002)
                for k,(u,v) in enumerate(HORN_BOLTS,1):
                    p=(u,v,side*(OUTER-RECESS)) if axis=='z' else (u,side*(OUTER-RECESS),v)
                    direction=(0,0,-side) if axis=='z' else (0,-side,0)
                    add(f'{g}_{joint}_M3x6_{side}_{k}',pose(screw(p,direction)),link,'fastener',METAL)
        for joint,pose,link in [('yaw',yaw_pose,'BODY'),('hip',at_hip,g+'_COXA'),('knee',at_knee,g+'_FEMUR')]:
            for x in (-29,-8.3):
                for z in (-10.25,10.25):
                    add(f'{g}_{joint}_factory_PA2_{x}_{z}',pose(screw((x,CASE_SEAT,z),(0,-1,0),5,True)),link,'factory_fastener',METAL)
        foot_insert=cylinder((110,0,-17),(0,0,1),2.3,5.7).cut(cylinder((110,0,-17.1),(0,0,1),1.55,5.9))
        add(g+'_shoe_insert',tib(foot_insert),g+'_TIBIA','insert',BRASS)
        add(g+'_shoe_M3x8',tib(screw((110,0,-19.6),(0,0,1),8)),g+'_TIBIA','fastener',METAL)
    print('Exporting',len(items),'valid solids and',len(parts),'unique print parts.',flush=True)
    report={'revision':'v4','parts':{},'instances':len(items),'parameters_mm_deg':P}
    for name,(s,qty,material,inverted) in parts.items():
        bb=s.BoundingBox();flat=bed(s) if inverted else s.translate((0,0,-bb.zmin))
        cq.exporters.export(s,str(output/'STEP_parts'/f'{name}.step'))
        cq.exporters.export(flat,str(output/'STL'/f'{name}.stl'),tolerance=.045,angularTolerance=.09)
        report['parts'][name]=dict(valid=s.isValid(),solids=len(s.Solids()),quantity=qty,material=material,
          size_mm=[bb.xlen,bb.ylen,bb.zlen],solid_volume_cm3=s.Volume()/1000,print_inverted=inverted)
    assembly=cq.Assembly(name='ARACHNE_18_V4')
    for idx,o in enumerate(items):
        s=o.pop('shape');assembly.add(s,name=o['name'],color=cq.Color(*o['color']))
        o['file']=f'{idx}.brep';s.exportBrep(str(output/'.cad_cache'/o['file']))
    assembly.export(str(output/'ARACHNE_18_v4_assembly.step'))
    layout=cq.Assembly(name='ARACHNE_18_V4_print_parts')
    for i,(name,(s,qty,mat,inverted)) in enumerate(parts.items()):
        layout.add(s,name=name,loc=cq.Location(cq.Vector((i%4)*220,(i//4)*180,0)),color=cq.Color(*TEAL))
    layout.export(str(output/'ARACHNE_18_v4_parts.step'))
    (output/'.cad_cache/placed.json').write_text(json.dumps(items,indent=2)+'\n')
    (output/'parameters.json').write_text(json.dumps(P,indent=2)+'\n')
    (output/'cad_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    with (output/'print_bom.csv').open('w') as f:
        w=csv.writer(f,lineterminator="\n");w.writerow(['part','quantity','material','size_X_mm','size_Y_mm','size_Z_mm'])
        for n,d in report['parts'].items():w.writerow([n,d['quantity'],d['material'],*[round(v,2) for v in d['size_mm']]])
    for path in output.glob('ARACHNE*.step'):
        with zipfile.ZipFile(path.with_suffix('.step.zip'),'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:z.write(path,path.name)
    print('Complete v4 CAD build:',output,flush=True)
    return report

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,default=OUT)
    build(ap.parse_args().output)
