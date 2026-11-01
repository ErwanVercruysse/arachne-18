"""Run the ARACHNE MJCF model with BAM M6 actuator dynamics.

macOS interactive: mjpython simulate.py
Headless: python simulate.py --headless --seconds 10
Joint exercise: mjpython simulate.py --fixed --mode sweep
All target angles are radians relative to the neutral CAD pose.
"""
from pathlib import Path
import argparse, collections, json, math, time
import numpy as np
import mujoco
from bam.model import load_model
from bam.mujoco import MujocoController

HERE=Path(__file__).resolve().parent

class Robot:
    def __init__(self,fixed=False,voltage=12.,parameter_file=None,model=None):
        # Optional independently constructed scene, e.g. curriculum terrain.
        # Ownership transfers to this robot: BAM mutates per-joint coefficients.
        self.model=model if model is not None else mujoco.MjModel.from_xml_path(str(HERE/('arachne_fixed.xml' if fixed else 'arachne.xml')))
        self.data=mujoco.MjData(self.model)
        self.joint_map=json.loads((HERE/'joint_map.json').read_text())['joints']
        self.names=[j['name'] for j in self.joint_map]
        parameter_file=Path(parameter_file or HERE/'bam_sts3215_12v_measured_m6.json')
        self.bam=load_model(str(parameter_file))
        self.bam.actuator.vin=float(voltage)
        # One controller shares the state model across all 18 motors. Voltage
        # sag is off: effective R/kt are not an identified electrical model.
        self.controller=MujocoController(self.bam,self.names,self.model,self.data)
        self.delay=float(json.loads(parameter_file.read_text()).get('command_delay',0.))
        self.reset()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model,self.data,0)
        mujoco.mj_forward(self.model,self.data)
        # BAM 1.0.2's Feetech class seeds this firmware state only in load_log;
        # simulation has no hardware log, so initialize it from robot position.
        self.bam.actuator.q_target_smooth=self.data.qpos[self.controller.qpos_indexes].copy()
        self.controller.last_ts=self.data.time
        friction,damping=self.bam.compute_frictions(0.,0.,0.)
        self.model.dof_frictionloss[self.controller.dof_indexes]=friction
        self.model.dof_damping[self.controller.dof_indexes]=damping
        self.controller.q_target=np.zeros(18)
        self.history=collections.deque([(-1.,np.zeros(18))])

    def step(self,target):
        target=np.asarray(target,dtype=float)
        if target.shape!=(18,) or not np.isfinite(target).all():raise ValueError('Expected 18 finite joint targets in radians')
        ranges=self.model.jnt_range[self.controller.joint_indexes]
        target=np.clip(target,ranges[:,0],ranges[:,1])
        now=self.data.time
        self.history.append((now,target.copy()))
        delayed_time=now-self.delay
        while len(self.history)>2 and self.history[1][0]<=delayed_time:self.history.popleft()
        t0,q0=self.history[0]
        t1,q1=self.history[1]
        alpha=np.clip((delayed_time-t0)/max(t1-t0,1e-12),0.,1.)
        self.controller.q_target=(1-alpha)*q0+alpha*q1
        self.controller.update()
        mujoco.mj_step(self.model,self.data)

    def state(self):
        return dict(time=float(self.data.time),q=self.data.qpos[self.controller.qpos_indexes].tolist(),
                    q_target=self.controller.q_target.tolist(),torque_before_friction_Nm=self.data.ctrl.tolist(),
                    base_position_m=self.data.body('BODY').xpos.tolist(),contacts=int(self.data.ncon))

def target_at(t,mode):
    if mode=='stand':return np.zeros(18)
    ramp=min(t/2.,1.)
    return np.array([math.radians(6)*ramp*math.sin(2*math.pi*.25*t+(i%3)*math.pi/3) for i in range(18)])

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--fixed',action='store_true')
    parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--voltage',type=float,default=12.)
    parser.add_argument('--mode',choices=['stand','sweep'],default='stand')
    parser.add_argument('--parameters',type=Path)
    parser.add_argument('--log',type=Path)
    args=parser.parse_args()
    robot=Robot(args.fixed,args.voltage,args.parameters)
    print('BAM M6 / STS3215 12V APPROXIMATION; not identified on the 12V hardware.')
    frames=[]; step=0
    def advance():
        nonlocal step
        robot.step(target_at(robot.data.time,args.mode));step+=1
        if step%20==0 and args.log:frames.append(robot.state())
    if args.headless:
        while robot.data.time<args.seconds:advance()
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(robot.model,robot.data) as viewer:
            viewer.cam.lookat[:]=[0,0,.10];viewer.cam.distance=.90;viewer.cam.azimuth=135;viewer.cam.elevation=-25
            viewer.opt.geomgroup[3]=0
            start=time.monotonic()
            while viewer.is_running() and robot.data.time<args.seconds:
                with viewer.lock():
                    for _ in range(10):advance()
                viewer.sync()
                time.sleep(max(0,robot.data.time-(time.monotonic()-start)))
    if args.log:args.log.write_text(json.dumps(frames))
    print(json.dumps(robot.state(),indent=2))

if __name__=='__main__':main()
