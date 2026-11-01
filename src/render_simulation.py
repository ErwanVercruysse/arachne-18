"""Capture real BAM-driven grounded body motion with six feet in contact."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT/'simulation'))
from simulate import Robot
from grounded_demo import GroundedMotion
from contact_utils import supporting_feet
import mujoco
from PIL import Image
robot=Robot();motion=GroundedMotion(robot)
# Suppress the decorative mirror reflection so floor contact is unambiguous.
robot.model.mat_reflectance[:]=0
robot.model.light_ambient[:]=[.20,.20,.20]
for _ in range(8000):robot.step(motion.target(robot.data.time))
renderer=mujoco.Renderer(robot.model,height=650,width=1000)
cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.095];cam.distance=.80;cam.azimuth=135;cam.elevation=-22
opt=mujoco.MjvOption();opt.geomgroup[3]=0
frames=[];contacts=[];feet=[];heights=[]
for frame in range(100):
 for _ in range(40):robot.step(motion.target(robot.data.time))
 contacts.append(int(robot.data.ncon));feet.append(len(supporting_feet(robot.model,robot.data)));heights.append(float(robot.data.body('BODY').xpos[2]))
 renderer.update_scene(robot.data,camera=cam,scene_option=opt)
 pixels=renderer.render()
 frames.append(Image.fromarray(pixels).convert('P',palette=Image.Palette.ADAPTIVE,colors=256))
 if frame==25:Image.fromarray(pixels).save(ROOT/'assets/mujoco-preview.png')
renderer.close()
assert min(feet)==max(feet)==6
assert not any(robot.data.warning.number)
frames[0].save(ROOT/'assets/mujoco-grounded.gif',save_all=True,append_images=frames[1:],duration=40,loop=0,optimize=False)
(ROOT/'validation/grounded_animation.json').write_text(json.dumps(dict(frames=100,duration_s=4,min_contacts=min(contacts),max_contacts=max(contacts),min_supporting_feet=min(feet),revision="v4",body_height_range_m=[min(heights),max(heights)],base_fixed=False,solver_warnings=robot.data.warning.number.tolist()),indent=2))
print('PASS: 100 frames of grounded BAM motion, all six feet supporting the body throughout')
