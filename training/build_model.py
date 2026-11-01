"""Build an explicitly approximate model for the fused Metal simulator.

Keep CAD inertias and joint limits; use convex foot/chassis contacts, static
BAM-derived servo coefficients, Euler and pyramidal contacts. Validate winners
in simulation/simulate.py (original collision geometry and complete BAM M6).
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'training' / 'arachne_metal.xml'


def build():
    tree = ET.parse(ROOT / 'simulation/arachne.xml')
    root = tree.getroot()
    root.find('compiler').set('meshdir', '../simulation/meshes')
    root.find('option').attrib.update(timestep='.002', integrator='Euler',
                                      cone='pyramidal', iterations='12')
    root.remove(root.find('sensor'))  # IMU observations are assembled from simulator state.
    default = root.find('default')
    p = json.loads((ROOT / 'simulation/bam_sts3215_12v_approx_m6.json').read_text())
    damping = p['friction_viscous'] + p['kt']**2 / p['R']
    kp = .166 * 32 * p['error_gain_ratio'] * 12 * p['kt'] / p['R']
    torque = .97 * 12 * p['kt'] / p['R']
    default.find('joint').attrib.update(armature=str(p['armature']),
                damping=str(damping), frictionloss=str(p['friction_base']))
    default.find('geom').set('condim', '3')
    asset = root.find('asset')
    # 26 vertices, including exact poles: bottom support matches the sphere.
    vertices = np.array([(x,y,z) for x in (-1,0,1) for y in (-1,0,1)
                          for z in (-1,0,1) if (x,y,z)!=(0,0,0)], float)
    vertices /= np.linalg.norm(vertices, axis=1, keepdims=True)
    ET.SubElement(asset,'mesh',name='torso_hull',
                  vertex=' '.join(f'{v:.9g}' for v in (vertices*[.091,.057,.045]).ravel()))
    foot_proxies=json.loads((ROOT/'simulation/collision_description.json').read_text())['foot_training_proxies']
    foot_meshes={}
    for foot in foot_proxies:
        meshname=foot['group']+'_training_foot'
        corners=np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])*foot['half_sizes_m']
        corners=corners@np.array(foot['rotation']).T+foot['pos']
        ET.SubElement(asset,'mesh',name=meshname,vertex=' '.join(f'{v:.9g}' for v in corners.ravel()))
        foot_meshes[foot['name']]=meshname
    # Strip purely visual meshes from training for quick load/compile.
    for body in root.iter('body'):
        for geom in list(body.findall('geom')):
            name = geom.get('name', '')
            if name.endswith('_foot_contact') or name == 'body_collision':
                geom.attrib.pop('size', None)
                geom.set('type', 'mesh')
                geom.set('mesh', 'torso_hull' if name == 'body_collision' else foot_meshes[name])
                geom.set('contype', '2'); geom.set('conaffinity', '1')
            else:
                body.remove(geom)
    ground = root.find("worldbody/geom[@name='ground']")
    ground.set('contype', '1'); ground.set('conaffinity', '2')
    for mesh in list(asset.findall('mesh')):
        if mesh.get('name') not in {*foot_meshes.values(),'torso_hull'}:
            asset.remove(mesh)
    actuators = root.find('actuator')
    ranges={j.get('name'):j.get('range') for j in root.iter('joint') if j.get('name')}
    for motor in actuators:
        motor.tag = 'position'
        motor.attrib.update(kp=str(kp), ctrllimited='true',
                            ctrlrange=ranges[motor.get('joint')],
                            forcelimited='true', forcerange=f'{-torque} {torque}')
    ET.indent(tree)
    tree.write(OUT, encoding='unicode')
    meta = dict(kp_Nm_per_rad=kp, damping=damping, armature=p['armature'],
                frictionloss=p['friction_base'], max_motor_torque_Nm=torque,
                omissions=['load-dependent and Stribeck friction', 'self collision',
                           'command delay and firmware slew (applied by controller)'],
                timestep=.002, revision='v4', contact='8-vertex rotated v4 shoe boxes; pyramidal condim 3')
    (OUT.parent/'model_approximation.json').write_text(json.dumps(meta, indent=2)+'\n')
    print(OUT)


if __name__ == '__main__':
    build()
