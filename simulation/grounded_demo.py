"""BAM-driven body-height exercise with all six feet on the floor; not a gait."""
from pathlib import Path
import argparse,json,math,time
import numpy as np
from simulate import Robot
from contact_utils import supporting_feet

class GroundedMotion:
    def __init__(self,robot,amplitude=.008,frequency=.25):
        # The neutral L1 chain gives the exact foot-site geometry. All legs share it.
        radial=np.array([math.sqrt(.5),math.sqrt(.5),0.])
        u=robot.model.body('L1_TIBIA').pos
        v=robot.model.site('L1_foot').pos
        u=np.array([u@radial,u[2]]);v=np.array([v@radial,v[2]])
        self.l1=np.linalg.norm(u);self.l2=np.linalg.norm(v)
        self.a0=math.atan2(u[1],u[0]);self.b0=math.atan2(v[1],v[0])-self.a0
        self.x,self.z=u+v;self.amplitude=amplitude;self.frequency=frequency
    def target(self,t):
        dz=self.amplitude*math.sin(2*math.pi*self.frequency*t)*min(t/2,1)
        x,z=self.x,self.z-dz;l1,l2=self.l1,self.l2
        b=-math.acos(np.clip((x*x+z*z-l1*l1-l2*l2)/(2*l1*l2),-1,1))
        a=math.atan2(z,x)-math.atan2(l2*math.sin(b),l1+l2*math.cos(b))
        return np.tile([0,a-self.a0,b-self.b0],6)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless',action='store_true');p.add_argument('--seconds',type=float,default=20)
    p.add_argument('--report',type=Path)
    args=p.parse_args();robot=Robot();motion=GroundedMotion(robot)
    contacts=[];feet=[];heights=[]
    def step():
        robot.step(motion.target(robot.data.time))
        if robot.data.time>2:
            contacts.append(int(robot.data.ncon));feet.append(len(supporting_feet(robot.model,robot.data)));heights.append(float(robot.data.body('BODY').xpos[2]))
    if args.headless:
        while robot.data.time<args.seconds:step()
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(robot.model,robot.data) as viewer:
            viewer.cam.lookat[:]=[0,0,.09];viewer.cam.distance=.78;viewer.cam.azimuth=135;viewer.cam.elevation=-20
            viewer.opt.geomgroup[3]=0;start=time.monotonic()
            while viewer.is_running() and robot.data.time<args.seconds:
                with viewer.lock():
                    for _ in range(10):step()
                viewer.sync();time.sleep(max(0,robot.data.time-(time.monotonic()-start)))
    result=dict(mode='grounded_body_height_exercise',duration_s=float(robot.data.time),amplitude_m=.008,frequency_hz=.25,
                min_contacts_after_2s=min(contacts) if contacts else None,max_contacts_after_2s=max(contacts) if contacts else None,
                min_supporting_feet=min(feet) if feet else None,
                body_height_range_m=[min(heights),max(heights)] if heights else None,solver_warnings=robot.data.warning.number.tolist())
    if contacts:
        assert min(feet)==max(feet)==6,'Expected all six feet to support the body after settling'
        assert not any(robot.data.warning.number),'Unexpected MuJoCo warning'
    if args.report:args.report.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
