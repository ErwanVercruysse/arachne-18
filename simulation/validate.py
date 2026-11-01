"""Executable checks for topology, inertias, BAM integration and stability."""
from pathlib import Path
import json, math
import numpy as np
import mujoco
from simulate import Robot, target_at
from contact_utils import supporting_feet

HERE=Path(__file__).resolve().parent
results={"revision":"v4"}
robot=Robot()
m,d=robot.model,robot.data
assert (m.nq,m.nv,m.nu)==(25,24,18)
assert m.nbody==20
assert np.all(m.body_mass[1:]>0) and np.all(m.body_inertia[1:]>0)
assert all(m.joint(name).type[0]==mujoco.mjtJoint.mjJNT_HINGE for name in robot.names)
assert d.ncon==0
results['topology']={'rigid_links':19,'hinges':18,'floating_base_dofs':6,'mesh_assets':m.nmesh,'initial_contacts':d.ncon,'mass_kg':float(m.body_mass.sum())}

# Each hinge's small positive motion must affect only its own subtree.
moved={}
for j in robot.joint_map:
    robot.reset();before=d.xpos.copy()
    dof=m.joint(j['name']).qposadr[0];d.qpos[dof]=.1;mujoco.mj_forward(m,d)
    leg=j['name'].split('_')[0];kind=j['name'].split('_')[1]
    expected={'yaw':[leg+'_FEMUR',leg+'_TIBIA'],'hip':[leg+'_TIBIA'],'knee':[]}[kind]
    actual=[m.body(i).name for i in range(1,m.nbody) if np.linalg.norm(d.xpos[i]-before[i])>1e-6]
    assert set(actual)==set(expected),(j['name'],actual,expected)
    # The hinge body's origin stays fixed; its orientation changes.
    assert np.linalg.norm(d.body(j['child']).xmat.reshape(3,3)-np.eye(3))>.05
    moved[j['name']]=actual
results['kinematics']={'passed':18,'body_origins_moved':moved}

trials=[]
for fixed,mode,voltage,seconds in [(False,'stand',12.,10),(False,'stand',11.1,10),(False,'stand',12.6,10),(True,'sweep',12.,12)]:
    robot=Robot(fixed,voltage);m,d=robot.model,robot.data
    max_error=0.;peak_torque=0.;max_speed=0.
    for k in range(round(seconds/m.opt.timestep)):
        target=target_at(d.time,mode);robot.step(target)
        assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
        peak_torque=max(peak_torque,float(np.max(np.abs(d.ctrl))))
        max_speed=max(max_speed,float(np.max(np.abs(d.qvel[robot.controller.dof_indexes]))))
        if k>2000:max_error=max(max_error,float(np.max(np.abs(d.qpos[robot.controller.qpos_indexes]-target))))
    assert not any(w.number for w in d.warning),[(w.number,w.lastinfo) for w in d.warning]
    q=d.qpos[robot.controller.qpos_indexes];lim=m.jnt_range[robot.controller.joint_indexes]
    assert np.all(q>lim[:,0]-.001) and np.all(q<lim[:,1]+.001)
    if not fixed:
        assert d.body('BODY').xpos[2]>.07
        assert np.linalg.norm(d.body('BODY').xpos[:2])<.015
        assert d.body('BODY').xmat.reshape(3,3)[2,2]>.98
        assert len(supporting_feet(m,d))==6
        pairs=[(m.geom(c.geom1).name,m.geom(c.geom2).name) for c in d.contact]
        assert all('ground' in p and any('_foot_contact' in n for n in p) for p in pairs)
    assert max_error<math.radians(8)
    trials.append(dict(fixed=fixed,mode=mode,voltage=voltage,duration_s=seconds,
        max_tracking_error_after_2s_deg=math.degrees(max_error),peak_motor_torque_before_friction_Nm=peak_torque,
        max_joint_speed_rad_s=max_speed,final_base_xyz_m=d.body('BODY').xpos.tolist(),final_contacts=d.ncon))
    print('PASS',trials[-1],flush=True)
results['dynamic_trials']=trials

# Reset must also reset BAM's firmware target and delay history.
robot=Robot(True)
for _ in range(2):
    robot.reset()
    for _ in range(1000):robot.step(np.full(18,.06))
    state=np.r_[robot.data.qpos,robot.data.qvel]
    if 'first' not in locals():first=state.copy()
    else:assert np.max(np.abs(first-state))<1e-9
results['reset_reproducible']=True
results['status']='PASS; numerical/model checks only, not hardware validation'
(HERE/'simulation_validation.json').write_text(json.dumps(results,indent=2))
print('ALL CHECKS PASSED')
