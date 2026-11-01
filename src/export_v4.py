"""Export ARACHNE's CAD solids to a 19-link, 18-hinge MuJoCo model.

Run with CadQuery 2.8, numpy, scipy and trimesh. CAD uses mm, MJCF uses SI.
All link frames have world-aligned axes in the neutral CAD pose.
"""
from pathlib import Path
import json, math, argparse
import xml.etree.ElementTree as ET
import cadquery as cq
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

BASE = Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cad-cache',type=Path,default=BASE.parent/'build/v4/.cad_cache')
parser.add_argument('--output',type=Path,default=BASE.parent/'simulation')
args=parser.parse_args()
OUT = args.output.resolve()
CACHE = args.cad_cache.resolve()
OUT.mkdir(parents=True, exist_ok=True)
(OUT/'meshes').mkdir(exist_ok=True)
for obsolete_mesh in (OUT/'meshes').glob('*.stl'):obsolete_mesh.unlink()
meta = json.loads((CACHE/'placed.json').read_text())
P=json.loads((CACHE.parent/'parameters.json').read_text())
groups = {'BODY': {'origin': np.zeros(3), 'parent': None, 'anchor': '01_CHASSIS'}}
joints = []
yaw_ranges=[[-P['yaw_inward_limit_deg'],P['yaw_outward_limit_deg']],
            [-P['yaw_middle_limit_deg'],P['yaw_middle_limit_deg']],
            [-P['yaw_outward_limit_deg'],P['yaw_inward_limit_deg']],
            [-P['yaw_inward_limit_deg'],P['yaw_outward_limit_deg']],
            [-P['yaw_middle_limit_deg'],P['yaw_middle_limit_deg']],
            [-P['yaw_outward_limit_deg'],P['yaw_inward_limit_deg']]]
for i, degrees in enumerate([45,90,135,225,270,315],1):
    a=math.radians(degrees); r=np.array([math.cos(a),math.sin(a),0.])
    root=.001*np.array([P['anchor_rx']*math.cos(a),P['anchor_ry']*math.sin(a),0.]); hip=root+.001*P['coxa']*r+np.array([0,0,.001*P['hip_z']])
    knee=hip+.001*P['femur']*math.cos(math.radians(P['femur_up_deg']))*r+np.array([0,0,.001*P['femur']*math.sin(math.radians(P['femur_up_deg']))])
    axis=np.array([math.sin(a),-math.cos(a),0.])
    for suffix,parent,origin,anchor,joint,ax,lim,reference in [
        ('COXA','BODY',root,f'L{i}_coxa','yaw',np.array([0.,0.,1.]),yaw_ranges[i-1],0),
        ('FEMUR',f'L{i}_COXA',hip,f'L{i}_femur','hip',axis,[-50,25],20),
        ('TIBIA',f'L{i}_FEMUR',knee,f'L{i}_tibia','knee',axis,[-20,100],-85)]:
        name=f'L{i}_{suffix}'; jname=f'L{i}_{joint}'
        groups[name]={'origin':origin,'parent':parent,'anchor':anchor,'joint':jname,'axis':ax,'range':lim}
        joints.append(dict(name=jname,servo_id=len(joints)+1,parent=parent,child=name,
            origin_m=origin.tolist(),axis=ax.tolist(),range_delta_deg=lim,neutral_absolute_deg=reference))

# Effective printed density is an estimate, to be replaced with slicer/measured masses.
MASS_SETTINGS=dict(printed_effective_density_kg_m3=850.,tpu_effective_density_kg_m3=1000.,
    steel_kg_m3=7800.,brass_kg_m3=8500.,servo_kg=.055,disc_kg=.002,
    battery_kg=.190,controller_kg=.045,body_wiring_fasteners_kg=.050,
    per_coxa_fasteners_kg=0.,per_femur_fasteners_kg=0.,per_tibia_fasteners_kg=0.)
body_items={g:[] for g in groups}
assets=[]; collision=[]; floor_heights=[]; foot_proxies=[];foot_sites={}
for o in meta:
    s=cq.Shape.importBrep(str(CACHE/o['file'])); name=o['name']; g=o['group']; origin=groups[g]['origin']
    vol=s.Volume()*1e-9; center=np.array(s.Center().toTuple())*.001-origin
    density=MASS_SETTINGS['tpu_effective_density_kg_m3'] if name.endswith('_foot') else MASS_SETTINGS['printed_effective_density_kg_m3']
    mass=vol*density
    if o.get('mass_kg') is not None:mass=o['mass_kg']
    elif o['kind']=='servo':mass=MASS_SETTINGS['servo_kg']
    elif o['kind']=='hardware':mass=MASS_SETTINGS['disc_kg']
    elif o['kind'] in ('fastener','factory_fastener','nut'):mass=vol*MASS_SETTINGS['steel_kg_m3']
    elif o['kind']=='insert':mass=vol*MASS_SETTINGS['brass_kg_m3']
    elif name.startswith('REF_3S'):mass=MASS_SETTINGS['battery_kg']
    elif name.startswith('REF_CONTROLLER'):mass=MASS_SETTINGS['controller_kg']
    inertia=np.array(cq.Shape.matrixOfInertia(s))*(mass/s.Volume())*1e-6
    body_items[g].append(dict(name=name,mass=mass,com=center,inertia=inertia,kind=o['kind']))
    verts,faces=s.tessellate(.22,.23)
    mesh=trimesh.Trimesh(np.array([v.toTuple() for v in verts])*.001-origin,np.array(faces),process=True)
    path='meshes/'+name+'.stl'; mesh.export(OUT/path)
    assets.append(dict(name=name,file=path,group=g,color=o['color'],kind=o['kind']))
    if o['kind']=='servo':
        rot=Rotation.from_matrix(np.array(o['servo_frame_rotation']))
        axcenter=.001*np.array(o['servo_frame_origin_mm'])
        xyzw=rot.as_quat(); quat=np.r_[xyzw[3],xyzw[:3]]
        collision.append(dict(name=name+'_collision',group=g,type='box',size=[.02261,.0158,.01236],
            pos=(axcenter+rot.apply([-.0125,-.0003,0])-origin).tolist(),quat=quat.tolist()))
    if name.endswith('_foot'):
        # MuJoCo uses the convex hull of the actual shoe mesh for contact.
        # The former 15 mm sphere does not fit inside this v4 5 mm shoe.
        bb=s.BoundingBox();floor_heights.append(bb.zmin*.001)
        collision.append(dict(name=name+'_contact',group=g,type='mesh',mesh=name))
        leg=int(name[1]);angle=[45,90,135,225,270,315][leg-1]
        rot=Rotation.from_euler('z',angle,degrees=True)*Rotation.from_euler('y',65,degrees=True)
        foot_sites[g]=rot.apply([.119,0,-.022])
        foot_proxies.append(dict(name=name+'_contact',group=g,pos=rot.apply([.109,0,-.0195]).tolist(),
            rotation=rot.as_matrix().tolist(),half_sizes_m=[.010,.0095,.0025]))

for g,items in body_items.items():
    extra=MASS_SETTINGS['body_wiring_fasteners_kg'] if g=='BODY' else MASS_SETTINGS['per_'+g.split('_')[1].lower()+'_fasteners_kg']
    mass0=sum(o['mass'] for o in items); com=sum(o['mass']*o['com'] for o in items)/mass0
    items.append(dict(name=g+'_unmodelled_fasteners',mass=extra,com=com,inertia=np.eye(3)*extra*1e-5,kind='mass_estimate'))

def fmt(v):
    if isinstance(v,(list,tuple,np.ndarray)):return ' '.join(f'{float(x):.10g}' for x in v)
    return str(v)
def add(parent,tag,**atts):return ET.SubElement(parent,tag,{k:fmt(v) for k,v in atts.items()})

link_report={}
def make_xml(fixed=False):
    xml=ET.Element('mujoco',model='ARACHNE_18_V4_STS3215_12V_approx')
    add(xml,'compiler',angle='radian',autolimits='true',inertiafromgeom='false',meshdir='meshes')
    add(xml,'option',timestep='.001',integrator='implicitfast',solver='Newton',iterations='60',cone='elliptic')
    visual=add(xml,'visual'); add(visual,'global',offwidth='1400',offheight='1000',azimuth='135',elevation='-25')
    asset=add(xml,'asset')
    add(asset,'texture',name='floor_texture',type='2d',builtin='checker',rgb1=[.24,.28,.27],rgb2=[.32,.36,.34],width=512,height=512)
    add(asset,'material',name='floor_mat',texture='floor_texture',texrepeat=[8,8],reflectance=.15)
    for o in assets:add(asset,'mesh',name=o['name'],file=Path(o['file']).name)
    default=add(xml,'default'); add(default,'joint',limited='true',damping='.02',armature='.0001',frictionloss='.001')
    add(default,'geom',friction=[.8,.005,.0002],solref=[.006,1],solimp=[.95,.99,.001],condim='4')
    world=add(xml,'worldbody');add(world,'light',pos=[.2,-.4,1.5],dir=[-.2,.4,-1],diffuse=[.9,.9,.9],castshadow='true')
    add(world,'geom',name='ground',type='plane',size=[2,2,.1],material='floor_mat')
    # Ground height follows actual CAD foot minimum, with 2 mm clearance.
    zmin=min(floor_heights)
    base_z=-zmin+.002+(.18 if fixed else 0.)
    body_nodes={}
    for g,definition in groups.items():
        origin=definition['origin']; parent=definition['parent']
        pos=np.array([0,0,base_z]) if parent is None else origin-groups[parent]['origin']
        body=add(world if parent is None else body_nodes[parent],'body',name=g,pos=pos)
        body_nodes[g]=body
        if parent is None:
            if not fixed:add(body,'freejoint',name='floating_base')
            add(body,'site',name='imu',size=.004,pos=[0,0,.078])
            add(body,'camera',name='follow',mode='trackcom',pos=[.65,-.85,.50],xyaxes=[.794,.607,0,-.24,.32,.916])
        else:add(body,'joint',name=definition['joint'],type='hinge',axis=definition['axis'],range=np.deg2rad(definition['range']))
        items=body_items[g]; mass=sum(o['mass'] for o in items); com=sum(o['mass']*o['com'] for o in items)/mass
        inertia=sum(o['inertia']+o['mass']*(np.dot(d:=o['com']-com,d)*np.eye(3)-np.outer(d,d)) for o in items)
        assert np.all(np.linalg.eigvalsh(inertia)>0)
        add(body,'inertial',pos=com,mass=mass,fullinertia=[inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]])
        link_report[g]=dict(mass_kg=mass,com_m=com.tolist(),inertia_kg_m2=inertia.tolist(),parent=parent,origin_neutral_m=origin.tolist(),parts=[o['name'] for o in items])
        for o in assets:
            if o['group']==g:add(body,'geom',name=o['name']+'_visual',type='mesh',mesh=o['name'],rgba=o['color']+[1],contype=0,conaffinity=0,group=2)
        for c in collision:
            if c['group']==g:add(body,'geom',**{k:v for k,v in c.items() if k!='group'},rgba=[.8,.4,.1,.35],group=3)
        if g=='BODY':
            add(body,'geom',name='body_collision',type='ellipsoid',pos=[-.003,0,.078],size=[.091,.057,.045],rgba=[.8,.4,.1,.3],group=3)
            add(body,'geom',name='deck_collision',type='box',pos=[0,0,.0295],size=[.074,.033,.0025],rgba=[.8,.4,.1,.3],group=3)
        if g.endswith('_TIBIA'):
            leg=int(g[1]); a=math.radians([45,90,135,225,270,315][leg-1]);r=np.array([math.cos(a),math.sin(a),0]);z=np.array([0,0,1.])
            f=lambda x,h:(math.cos(math.radians(-65))*x-math.sin(math.radians(-65))*h)*r+(math.sin(math.radians(-65))*x+math.cos(math.radians(-65))*h)*z
            add(body,'geom',name=g+'_shin_collision',type='capsule',fromto=np.r_[f(.030,0),f(.097,-.01)],size=.006,group=3,rgba=[.8,.4,.1,.3])
            add(body,'site',name=f'L{leg}_foot',pos=foot_sites[g],size=.003,rgba=[0,1,0,.3])
    actuator=add(xml,'actuator')
    for j in joints:add(actuator,'motor',name=j['name'],joint=j['name'],gear='1')
    sensor=add(xml,'sensor'); add(sensor,'gyro',name='body_gyro',site='imu');add(sensor,'accelerometer',name='body_accel',site='imu')
    for j in joints:
        add(sensor,'jointpos',name=j['name']+'_q',joint=j['name']);add(sensor,'jointvel',name=j['name']+'_dq',joint=j['name'])
    keyframe=add(xml,'keyframe');add(keyframe,'key',name='neutral',qpos=([0,0,base_z,1,0,0,0] if not fixed else [])+[0]*18)
    ET.indent(xml,space='  ')
    path=OUT/('arachne_fixed.xml' if fixed else 'arachne.xml');ET.ElementTree(xml).write(path,encoding='unicode',xml_declaration=True)
    return base_z

base_z=make_xml();make_xml(True)
(OUT/'joint_map.json').write_text(json.dumps(dict(revision='v4',convention='q=0 is the neutral CAD pose; radians; right-hand axis',base_height_m=base_z,
    kinematics=dict(coxa_m=.001*P['coxa'],hip_z_m=.001*P['hip_z'],femur_m=.001*P['femur'],hip_neutral_deg=20,knee_neutral_deg=-85,foot_site_tibia_local_m=[.119,0,-.022]),joints=joints),indent=2))
(OUT/'mass_properties.json').write_text(json.dumps(dict(assumptions=MASS_SETTINGS,total_mass_kg=sum(v['mass_kg'] for v in link_report.values()),links=link_report),indent=2))
(OUT/'cad_instances.json').write_text(json.dumps([{**o,'group':o['group']} for o in meta],indent=2))
(OUT/'collision_description.json').write_text(json.dumps(dict(revision='v4',foot_contact='Convex hull of the actual v4 TPU shoe CAD mesh',
    foot_training_proxies=foot_proxies,limitations=['Servo cases use box collision proxies','Canopy uses an ellipsoid proxy','Flexible cables and compliant TPU deformation are not modelled']),indent=2)+'\n')
print('Exported',len(assets),'meshes;',len(groups),'links;',len(joints),'hinges; mass',sum(v['mass_kg'] for v in link_report.values()),flush=True)
