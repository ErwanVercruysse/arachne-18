"""Independent MuJoCo C/BAM evaluation and real physics video capture."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import mujoco
from gait import ROOT, INITIAL, target, rotate, feet, load, frequency


def evaluate(params, command, seconds=10., backend='bam', video=None, width=960, height=640,
             schedule=None, friction=None, payload=0., imu_noise=0., seed=23, terrain=None, video_label=None,
             controller=None):
    """controller: optional factory (model, data, rng) -> act(data, command, time) returning the filtered
    joint targets [18]; it replaces the policy, filter and slew below (e.g. tools/terrain_gait.py)."""
    terrain_info=None
    if terrain is not None and backend!='bam':
        raise ValueError('Terrain requires the original MuJoCo/BAM backend')
    if backend=='bam':
        sys.path.insert(0,str(ROOT/'simulation'))
        from simulate import Robot
        scene=None
        if terrain is not None:
            from terrain import make_model, height_below
            scene,terrain_info=make_model(terrain,seed)
        robot=Robot(model=scene); m,d=robot.model,robot.data
    else:
        m=mujoco.MjModel.from_xml_path(str(ROOT/'training/arachne_metal.xml'))
        d=mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m,d,0)
        robot=None
    if friction is not None:m.geom_friction[:,0]=friction
    if payload:
        body=m.body('BODY').id
        old=m.body_mass[body];m.body_mass[body]+=payload
        m.body_inertia[body]*=(old+payload)/old
        mujoco.mj_setConst(m,d)
    rng=np.random.default_rng(seed)
    dt=.01; substeps=round(dt/m.opt.timestep)
    p=np.asarray(params,dtype=float)[None,:]; cmd=np.asarray(command,dtype=float)[None,:]
    phase=np.zeros(1); sm=np.zeros((1,18)); prev=sm.copy(); prev2=sm.copy()
    samples=[]; all_q=[]; all_t=[]; all_ctrl=[]; all_commands=[]; last_feet=None
    clearances=[]; exposure=[]; local_feet=[]; cycles=[]; cycle=0.
    ground_id=m.geom('ground').id
    foot_ids=[m.geom(f'L{j}_foot_contact').id for j in range(1,7)]
    foot_geoms=set(foot_ids)
    rock_ids={i for i in range(m.ngeom) if (mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith('obstacle_rocks_')}
    grass_ids={i for i in range(m.ngeom) if (mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith('obstacle_grass_')}
    support_ids={ground_id}|rock_ids
    robot_ids={i for i in range(m.ngeom) if m.geom_bodyid[i]>0 and i not in grass_ids}
    motor_dofs=robot.controller.dof_indexes if robot else np.arange(6,24)
    rock_steps=grass_steps=0;grass_deflection=0.
    grass_qpos=[m.jnt_qposadr[j] for j in range(m.njnt) if (mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,j) or '').startswith('obstacle_grass_')]
    cmd_filtered=cmd.copy()
    act=controller(m,d,rng) if controller else None
    render=None; writer=None
    if video:
        import imageio.v2 as imageio
        from video_hud import HUD
        hud=HUD(width,height,subtitle=video_label)
        Path(video).parent.mkdir(parents=True,exist_ok=True)
        m.vis.global_.offwidth=width; m.vis.global_.offheight=height
        m.mat_reflectance[:]=0
        # A directional key light keeps the robot visible after it has walked
        # several metres beyond the original scene's fixed spotlight cone.
        m.light_type[:]=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        m.light_ambient[:]=[.15,.15,.15]
        m.light_diffuse[:]=[.7,.7,.7]
        m.vis.headlight.ambient[:]=[.3,.3,.3]
        m.vis.headlight.diffuse[:]=[.55,.55,.55]
        if m.geom_type[ground_id]==mujoco.mjtGeom.mjGEOM_PLANE:
            # Extend only the plane's finite visual patch, keeping tile scale.
            material=m.geom_matid[ground_id]
            scale=20/m.geom_size[ground_id,0]
            m.geom_size[ground_id,:2]*=scale
            if material>=0 and not m.mat_texuniform[material]:
                m.mat_texrepeat[material]*=scale
        render=mujoco.Renderer(m,height,width)
        writer=imageio.get_writer(str(video),fps=50,codec='libx264',quality=None,
                                  macro_block_size=1,ffmpeg_params=['-crf','23','-movflags','+faststart'])
        camera=mujoco.MjvCamera(); mujoco.mjv_defaultCamera(camera)
        camera.distance=.82;camera.azimuth=125;camera.elevation=-27
        opt=mujoco.MjvOption();opt.geomgroup[3]=0;opt.sitegroup[:]=0
    try:
        for i in range(round(seconds/dt)):
            if schedule:
                requested=next(c for start,c in reversed(schedule) if i*dt>=start)
                cmd_filtered += (1-np.exp(-dt/.25))*(np.array(requested)[None,:]-cmd_filtered)
                cmd=cmd_filtered
            obs_q=d.qpos[None,:].copy();gyro=d.qvel[None,3:6].copy()
            if imu_noise:
                # Perturb orientation estimate only; true state remains untouched.
                axis=rng.normal(size=3);angle=rng.normal(scale=np.deg2rad(imu_noise))
                axis/=np.linalg.norm(axis); dq=np.r_[np.cos(angle/2),axis*np.sin(angle/2)]
                perturbed=np.empty(4);mujoco.mju_mulQuat(perturbed,obs_q[0,3:7],dq)
                obs_q[0,3:7]=perturbed;gyro+=rng.normal(scale=.005,size=(1,3))
            if act:filtered=np.asarray(act(d,cmd[0],i*dt),dtype=float)[None,:]
            else:
                desired=target(p,phase,cmd,obs_q,i*dt,gyro=gyro)
                filtered=np.clip(sm+.35*(desired-sm),sm-.047171,sm+.047171)
            # Same one-control-step delay/filter as training; BAM adds its real delay.
            for _ in range(substeps):
                if robot:robot.step(sm[0])
                else:d.ctrl[:]=sm[0];mujoco.mj_step(m,d)
            prev2,prev,sm=prev,sm,filtered
            phase=(phase+frequency(p,cmd)*dt)%1
            cycle+=float(frequency(p,cmd)[0])*dt
            q=d.qpos[None,:];v=d.qvel[None,:]
            gravity=rotate(q[:,3:7],np.array([[0.,0.,-1.]]),inverse=True)[0]
            lin=rotate(q[:,3:7],v[:,:3],inverse=True)[0]
            foot=np.array([d.site(f'L{j}_foot').xpos.copy() for j in range(1,7)])
            local_feet.append(np.array([rotate(q[:,3:7],(f-d.qpos[:3])[None,:],inverse=True)[0]
                                       for f in foot]))
            cycles.append(cycle)
            ground_z=height_below(m,d,*d.qpos[:2]) if terrain is not None else 0.
            clearances.append(d.qpos[2]-ground_z)
            exposure.append(np.linalg.norm(d.qpos[:2])>.54)
            contacts=set();nonfoot=0;rock_touch=False;grass_touch=False
            for c in d.contact:
                pair={int(c.geom1),int(c.geom2)}
                if pair & support_ids and pair & robot_ids:
                    geom=next(iter(pair & robot_ids))
                    if geom in foot_geoms:
                        contacts.add(geom)
                        rock_touch |= bool(pair & rock_ids)
                    else:nonfoot+=1
                grass_touch |= bool(pair & grass_ids and pair & robot_ids)
            rock_steps+=int(rock_touch);grass_steps+=int(grass_touch)
            if grass_qpos:grass_deflection=max(grass_deflection,float(np.max(np.abs(d.qpos[grass_qpos]))))
            vel=np.zeros_like(foot) if last_feet is None else (foot-last_feet)/dt
            last_feet=foot
            mask=np.array([geom in contacts for geom in foot_ids])
            slip=np.mean(np.sum(vel[mask,:2]**2,-1)) if mask.any() else 0.
            samples.append([*lin[:2],d.qvel[5],np.arccos(np.clip(-gravity[2],-1,1)),
                            d.qpos[2],d.qvel[2],np.linalg.norm(d.qvel[3:5]),slip,
                            len(contacts),nonfoot,np.sqrt(np.mean(((sm-prev)/dt)**2)),
                            np.sqrt(np.mean(((sm-2*prev+prev2)/dt**2)**2)),
                            np.max(np.abs(d.qvel[motor_dofs])),np.max(np.abs(d.actuator_force))])
            all_q.append(d.qpos.copy());all_ctrl.append(sm[0].copy());all_t.append(d.time)
            all_commands.append(cmd[0].copy())
            if render and i%2==0:
                camera.lookat[:]=d.qpos[:3]+[0,0,.015]
                render.update_scene(d,camera,scene_option=opt)
                frame=hud.draw(render.render(),d.time,cmd[0],lin,samples[-1][3],d.qpos)
                writer.append_data(frame)
        data=np.asarray(samples); skip=min(200,len(data)//2);steady=data[skip:]
        requested=np.asarray(all_commands)[skip:]
        clearance=np.asarray(clearances);ctrl=np.asarray(all_ctrl)
        jerk=np.diff(ctrl,n=3,axis=0)/dt**3
        # Actual body-relative foot travel, averaged over complete gait cycles.
        excursions=[]; cycle_ids=np.floor(cycles).astype(int)
        direction=np.asarray(command[:2],dtype=float).copy()
        translating=bool(np.linalg.norm(direction)>.001)
        direction/=max(np.linalg.norm(direction),1e-9)
        for c in np.unique(cycle_ids[skip:])[1:-1]:
            mask=(cycle_ids==c)&(np.arange(len(cycle_ids))>=skip)
            trace=np.asarray(local_feet)[mask,:,:2]@direction
            excursions.append(float(np.ptp(trace,axis=0).mean()))
        result=dict(backend=backend,command=list(command),seconds=seconds,
            measured_velocity=steady[:,:3].mean(0).tolist(),
            velocity_rmse=float(np.sqrt(np.mean(np.sum((steady[:,:2]-requested[:,:2])**2,-1)))),
            yaw_rmse=float(np.sqrt(np.mean((steady[:,2]-requested[:,2])**2))),
            tilt_rms_deg=float(np.rad2deg(np.sqrt(np.mean(steady[:,3]**2)))),
            tilt_max_deg=float(np.rad2deg(np.max(data[:,3]))),
            height_mean_m=float(steady[:,4].mean()),height_std_mm=float(steady[:,4].std()*1000),
            vertical_velocity_rms=float(np.sqrt(np.mean(steady[:,5]**2))),
            body_rp_rate_rms=float(np.sqrt(np.mean(steady[:,6]**2))),
            slip_rms_m_s=float(np.sqrt(steady[:,7].mean())),
            feet_contacts_mean=float(steady[:,8].mean()),nonfoot_contacts=int(data[:,9].sum()),
            target_rate_rms=float(np.sqrt(np.mean(steady[:,10]**2))),
            target_acceleration_rms=float(np.sqrt(np.mean(steady[:,11]**2))),
            joint_speed_max=float(data[:,12].max()),motor_torque_max=float(data[:,13].max()),
            cadence_hz=float(np.mean(frequency(np.repeat(p,len(requested),axis=0),requested))),
            body_distance_per_cycle_m=(float(steady[:,:2].mean(0)@direction/frequency(p,np.asarray(command)[None,:])[0])
                                       if not schedule and translating else None),
            foot_excursion_mean_m=float(np.mean(excursions)) if excursions and not schedule and translating else None,
            target_jerk_rms=float(np.sqrt(np.mean(jerk[max(skip-3,0):]**2))),
            clearance_min_m=float(clearance.min()),clearance_std_mm=float(clearance[skip:].std()*1000),
            clearance_rate_rms=float(np.sqrt(np.mean((np.diff(clearance)[skip-1:]/dt)**2))),
            terrain_exposure_fraction=float(np.mean(exposure[skip:])),terrain=terrain_info,
            displacement_m=d.qpos[:2].tolist(),warnings=int(sum(w.number for w in d.warning)),
            passed=bool(np.all(clearance>.060) and np.max(data[:,3])<np.deg2rad(15)
                        and not data[:,9].any() and np.isfinite(data).all()),
            friction=friction,payload_kg=payload,imu_orientation_noise_deg=imu_noise,
            rock_foot_contact_steps=rock_steps,vegetation_contact_steps=grass_steps,
            grass_deflection_max_deg=float(np.rad2deg(grass_deflection)),
            schedule=schedule)
        if video:
            np.savez_compressed(str(Path(video).with_suffix('.npz')),time=all_t,qpos=all_q,
                                target=all_ctrl,metrics=data,command=all_commands)
        return result
    finally:
        if writer:writer.close()
        if render:render.close()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--policy',type=Path)
    ap.add_argument('--backend',choices=['bam','surrogate'],default='bam')
    ap.add_argument('--command',nargs=3,type=float,default=[.08,0,0])
    ap.add_argument('--seconds',type=float,default=10.)
    ap.add_argument('--video',type=Path)
    ap.add_argument('--out',type=Path)
    args=ap.parse_args()
    r=evaluate(load(args.policy) if args.policy else INITIAL,args.command,args.seconds,args.backend,args.video)
    print(json.dumps(r,indent=2))
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(r,indent=2)+'\n')


if __name__=='__main__':main()
