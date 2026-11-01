"""Single-jump policy search in original MuJoCo/BAM; no upright reward."""
import argparse,json,sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
import mujoco
from gait import ROOT,LIMITS,smooth5
sys.path.insert(0,str(ROOT/'simulation'))
from simulate import Robot

LOW=np.r_[.12,0.,.035,.04,np.tile(-LIMITS[:3]+.004,3)]
HIGH=np.r_[.65,.25,.24,.30,np.tile(LIMITS[:3]-.004,3)]
INITIAL=np.r_[.35,.06,.08,.10,0,.22,-.12,0,-.24,.20,0,.24,-.24]


def target(p,t):
    """Symmetric crouch, push and tuck; all joint angles stay inside CAD limits."""
    width=(len(p)-4)//3
    shaped=[p[4+width*i:4+width*(i+1)] for i in range(3)]
    poses=[np.zeros(18),*[np.tile(q,6) if width==3 else q for q in shaped],np.zeros(18)]
    starts=[.6,.6+p[0]+p[1],.6+p[0]+p[1]+p[2],.6+sum(p[:4])+.30]
    durations=[p[0],.025,p[3],.45]
    q=poses[0]
    for i,(start,duration) in enumerate(zip(starts,durations)):
        if t<start:break
        q=poses[i]+smooth5(np.clip((t-start)/duration,0,1))*(poses[i+1]-poses[i])
    return q


def geometry_floor(m,d,ids):
    """Lowest support point of each collision proxy, including rotated bodies."""
    size=m.geom_size[ids];rot=d.geom_xmat[ids].reshape(-1,3,3)[:,2,:];types=m.geom_type[ids]
    extent=np.zeros(len(ids))
    sphere=types==mujoco.mjtGeom.mjGEOM_SPHERE;extent[sphere]=size[sphere,0]
    box=types==mujoco.mjtGeom.mjGEOM_BOX;extent[box]=(np.abs(rot[box])*size[box]).sum(1)
    capsule=types==mujoco.mjtGeom.mjGEOM_CAPSULE;extent[capsule]=size[capsule,0]+size[capsule,1]*np.abs(rot[capsule,2])
    ellipsoid=types==mujoco.mjtGeom.mjGEOM_ELLIPSOID;extent[ellipsoid]=np.linalg.norm(size[ellipsoid]*rot[ellipsoid],axis=1)
    mesh=types==mujoco.mjtGeom.mjGEOM_MESH
    for index in np.flatnonzero(mesh):
        mesh_id=m.geom_dataid[ids[index]]
        start=m.mesh_vertadr[mesh_id];count=m.mesh_vertnum[mesh_id]
        # MuJoCo's compiled mesh vertices include principal-axis recentering;
        # geom_xmat/geom_xpos carry the corresponding compiled transform.
        extent[index]=-np.min(m.mesh_vert[start:start+count]@rot[index])
    if not np.all(sphere|box|capsule|ellipsoid|mesh):raise ValueError('Unsupported collision proxy')
    return d.geom_xpos[ids,2]-extent


def evaluate(p,seconds=3.5,record=False,friction=None,payload=0.,timestep=None):
    robot=Robot();m,d=robot.model,robot.data
    if timestep is not None:m.opt.timestep=timestep
    if friction is not None:m.geom_friction[:,0]=friction
    if payload:
        b=m.body('BODY').id;old=m.body_mass[b];m.body_mass[b]+=payload;m.body_inertia[b]*=(old+payload)/old;mujoco.mj_setConst(m,d)
    ids=np.flatnonzero((m.geom_bodyid>0)&((m.geom_contype!=0)|(m.geom_conaffinity!=0)))
    ground=m.geom('ground').id
    sm=np.zeros(18);times=[];clearance=[];contacts=[];heights=[];tilts=[];states=[];targets=[];maxspeed=0.;torque=0.
    for i in range(round(seconds/.01)):
        q=target(p,i*.01);filtered=np.clip(sm+.35*(q-sm),sm-.047171,sm+.047171)
        for _ in range(round(.01/m.opt.timestep)):robot.step(sm)
        sm=filtered;mujoco.mj_forward(m,d)
        floor=geometry_floor(m,d,ids)
        contact=any(c.geom1==ground or c.geom2==ground for c in d.contact)
        times.append(d.time);clearance.append(float(floor.min()));contacts.append(contact);heights.append(float(d.qpos[2]))
        tilts.append(float(np.rad2deg(np.arccos(np.clip(d.xmat[m.body('BODY').id].reshape(3,3)[2,2],-1,1)))))
        maxspeed=max(maxspeed,float(np.abs(d.qvel[robot.controller.dof_indexes]).max()));torque=max(torque,float(np.abs(d.actuator_force).max()))
        if record:states.append(d.qpos.copy());targets.append(sm.copy())
    t=np.array(times);h=np.array(clearance);air=(~np.array(contacts))&(h>.001)&(t>.7)
    # Require sustained whole-robot clearance: a single noisy frame cannot win.
    robust=np.minimum.reduce([h[:-2],h[1:-1],h[2:]])
    valid=air[:-2]&air[1:-1]&air[2:];score=float(np.max(np.where(valid,robust,0)))
    best=int(np.argmax(np.where(air,h,0)));warnings=int(sum(w.number for w in d.warning))
    launch_vz=float(np.max(np.diff(heights)/.01))
    search_score=score if score>0 else -.01+.01*min(launch_vz,1.)
    if warnings or not np.isfinite(h).all():score=-1.;search_score=-1.
    result=dict(score_m=score,search_score=search_score,peak_vertical_speed_m_s=launch_vz,peak_whole_robot_clearance_m=float(h[best]) if air.any() else 0.,
        apex_time_s=float(t[best]),body_height_at_clearance_apex_m=heights[best],tilt_at_apex_deg=tilts[best],
        airborne_time_s=float(air.sum()*.01),peak_body_height_m=max(heights),joint_speed_max=maxspeed,
        motor_torque_max=torque,warnings=warnings,friction=friction,payload_kg=payload,physics_timestep_s=float(m.opt.timestep),
        objective='Maximum minimum clearance of all collision geometry, sustained across three 10 ms samples; no upright reward')
    if record:return result,dict(time=t,qpos=np.array(states),target=np.array(targets),clearance=h,airborne=air,body_height=np.array(heights))
    return result


def worker(p):return evaluate(p)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--generations',type=int,default=16);ap.add_argument('--population',type=int,default=96);ap.add_argument('--workers',type=int,default=8);ap.add_argument('--std-floor',type=float,default=.012);ap.add_argument('--resume',type=Path);ap.add_argument('--asymmetric',action='store_true');ap.add_argument('--out',type=Path,default=ROOT/'training/runs/jump');args=ap.parse_args()
    out=args.out;out.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(123);mean=np.array(json.loads(args.resume.read_text())['parameters']) if args.resume else INITIAL.copy()
    if args.asymmetric and len(mean)==13:mean=np.r_[mean[:4],*[np.tile(mean[4+3*i:7+3*i],6) for i in range(3)]]
    low=np.r_[LOW[:4],np.tile(-LIMITS+.004,3)] if len(mean)==58 else LOW
    high=np.r_[HIGH[:4],np.tile(LIMITS-.004,3)] if len(mean)==58 else HIGH
    sd=(high-low)*(.08 if args.resume else .28);best=None;history=[]
    (out/'initial.json').write_text(json.dumps(dict(parameters=mean.tolist())))
    with ProcessPoolExecutor(args.workers) as pool:
        for gen in range(args.generations):
            pop=np.clip(rng.normal(mean,sd,(args.population,len(mean))),low,high);pop[0]=mean
            if best is not None:pop[1]=best['parameters']
            results=list(pool.map(worker,pop));scores=np.array([r['search_score'] for r in results]);elite=np.argsort(scores)[-max(8,args.population//6):]
            mean=.2*mean+.8*pop[elite].mean(0);sd=np.maximum(.3*sd+.7*pop[elite].std(0),(high-low)*args.std_floor)
            i=int(scores.argmax())
            if best is None or scores[i]>best['metrics'].get('search_score',-1):
                best=dict(algorithm='CEM single-jump policy search',parameters=pop[i].tolist(),metrics=results[i],seed=123,generation=gen)
                (ROOT/'training/policies/jump.json').write_text(json.dumps(best,indent=2)+'\n')
            history.append(dict(generation=gen,best_m=best['metrics']['score_m'],population_best_m=float(scores[i])))
            (out/'history.json').write_text(json.dumps(history,indent=2)+'\n')
            print(gen,'clearance cm',100*best['metrics']['score_m'],flush=True)


if __name__=='__main__':main()
