"""ARACHNE stunt show: a choreographed BAM/MuJoCo routine of tricks.

macOS viewer:  mjpython simulation/stunts.py
Headless:      python simulation/stunts.py --headless --report stunts_run.json
Video:         python simulation/stunts.py --video stunts.mp4
One stunt:     mjpython simulation/stunts.py --only wave --loop

Every stunt is physics, not animation: targets go through the same 10 ms
filter used in training, BAM computes motor torques and MuJoCo solves contact.
Body-pose stunts keep feet planted by inverse kinematics in the body frame.
"""
from pathlib import Path
import argparse, json, math, sys, time
import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent/'training'))
from simulate import Robot
import gait
import jump

DT = .01
FEET0 = gait.COXA + gait.R0*gait.RADIAL  # neutral foot xy, body frame
FEET0 = np.c_[FEET0, np.full(6, gait.Z0)]
FRONT, MIDDLE, REAR = [0, 5], [1, 4], [2, 3]  # L1/L6 at +x, L2/L5, L3/L4


def smooth(u):
    return gait.smooth5(np.clip(u, 0, 1))


def envelope(t, length, rise=1.):
    return smooth(t/rise)*smooth((length-t)/rise)


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def ik(feet):
    """Body-frame foot positions [6,3] -> 18 joint offsets from CAD neutral."""
    dx, dy = feet[:, 0]-gait.COXA[:, 0], feet[:, 1]-gait.COXA[:, 1]
    yaw = np.arctan2(dy, dx)-gait.AZIMUTH
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
    r, z = np.hypot(dx, dy)-.05, feet[:, 2]
    b = -np.arccos(np.clip((r*r+z*z-gait.L1**2-gait.L2**2)/(2*gait.L1*gait.L2), -.999999, .999999))
    a = np.arctan2(z, r)-np.arctan2(gait.L2*np.sin(b), gait.L1+gait.L2*np.cos(b))
    return np.stack([yaw, a-gait.A0, b-gait.B0], -1).reshape(18)


def body_pose(shift=(0, 0, 0), rpy=(0, 0, 0), feet=None):
    """Feet stay put in the world while the body translates and rotates."""
    feet = FEET0 if feet is None else feet
    R = rot(*rpy)
    return ik((feet-np.asarray(shift))@R)


# ---------------------------------------------------------------- stunts
# Each stunt: (name, seconds, fn(t, state) -> 18 targets). All begin and end
# near the neutral pose so they can be chained in any order.

def pushups(t, s, L=7.):
    e = envelope(t, L, .8)
    dz = e*(.5-.5*math.cos(2*math.pi*.6*max(t-.8, 0)))*-.028 + e*.010
    return body_pose((0, 0, dz))


def body_dance(t, s, L=9.):
    e = envelope(t, L)
    w = 2*math.pi*.45*t
    return body_pose((.018*e*math.cos(w), .018*e*math.sin(w), .006*e*math.sin(2*w)),
                     (math.radians(7)*e*math.sin(w), -math.radians(7)*e*math.cos(w), 0))


def twist(t, s, L=6.):
    e = envelope(t, L)
    return body_pose((0, 0, .008*e), (0, 0, math.radians(16)*e*math.sin(2*math.pi*.5*t)))


def wave(t, s, L=7.):
    """Shift weight back-right, raise L1 high and wave hello."""
    e = envelope(t, L)
    q = body_pose((-.018*e, -.014*e, .004*e), (0, math.radians(-5)*e, 0))
    lift = smooth((t-1.)/.6)*smooth((L-1.-t)/.6)
    w = math.sin(2*math.pi*1.4*max(t-1.6, 0))
    raised = q.reshape(6, 3)[0]+lift*(np.array([math.radians(18)*w, math.radians(15), math.radians(55+15*w)])-q.reshape(6, 3)[0])
    q = q.reshape(6, 3); q[0] = raised
    return q.reshape(18)


def rear_up(t, s, L=8.):
    """Lean back on four legs and box with both front legs."""
    e = envelope(t, L, 1.2)
    feet = FEET0.copy()
    feet[MIDDLE, 0] += .025*e  # plant middle feet further forward for support
    q = body_pose((-.030*e, 0, .006*e), (0, math.radians(-10)*e, 0), feet)
    lift = smooth((t-1.4)/.6)*smooth((L-1.3-t)/.6)
    q = q.reshape(6, 3)
    for n, i in enumerate(FRONT):
        punch = .5+.5*math.sin(2*math.pi*1.6*max(t-2., 0)+n*math.pi)
        pose = np.array([math.radians(-12 if i == 0 else 12), math.radians(15), math.radians(20+60*punch)])
        q[i] += lift*(pose-q[i])
    return q.reshape(18)


def ripple(t, s, L=7.):
    """A Mexican wave: each leg lifts in turn around the body."""
    e = envelope(t, L, .6)
    q = body_pose((0, 0, .004*e)).reshape(6, 3)
    order = [0, 1, 2, 3, 4, 5]
    for k, i in enumerate(order):
        ph = (1.1*max(t-.7, 0)-k/6) % 1
        bump = math.sin(math.pi*min(ph/.3, 1))**2 if ph < .3 else 0.
        q[i] += e*bump*np.array([0, math.radians(15)-q[i][1], math.radians(40)])
    return q.reshape(18)


def bow(t, s, L=5.):
    """Take a bow: front dips down, rear rises."""
    e = envelope(t, L, 1.)
    return body_pose((.012*e, 0, .004*e), (0, math.radians(13)*e, 0))


class Spin:
    """Spin on the spot with the learned omnidirectional tripod gait."""
    def __init__(self, L=8., rate=.55):
        self.L, self.rate = L, rate
        self.params = gait.load(HERE.parent/'training/policies/omni.json')[None, :]
    def __call__(self, t, s):
        if t < DT/2: s['phase'] = np.zeros(1)
        yaw_cmd = self.rate*smooth(t/1.)*smooth((self.L-1.5-t)/1.)
        cmd = np.array([[0, 0, yaw_cmd]])
        q = gait.target(self.params, s['phase'], cmd, s['qpos'][None, :], t)[0]
        s['phase'] = (s['phase']+gait.frequency(self.params, cmd)*DT) % 1
        return q*smooth((self.L-.4-t)/1.)


class Jump:
    """Crouch, launch and tuck with the CEM-searched jump policy."""
    def __init__(self, policy='jump.json'):
        self.params = np.array(json.loads((HERE.parent/'training/policies'/policy).read_text())['parameters'])
        self.L = 4.
    def __call__(self, t, s):
        return jump.target(self.params, t)


def hold(t, s):
    return np.zeros(18)


SHOW = [('stand', 2., hold), ('push-ups', 7., pushups), ('body dance', 9., body_dance),
        ('twist', 6., twist), ('wave hello', 7., wave), ('rear up & box', 8., rear_up),
        ('ripple', 7., ripple), ('spin', 8., None), ('jump!', 4., None), ('bow', 5., bow)]


def build_show(only=None):
    show = []
    for name, L, fn in SHOW:
        if fn is None: fn = Spin(L) if name == 'spin' else Jump()
        elif fn is not hold: fn = (lambda f, L: lambda t, s: f(t, s, L))(fn, L)
        show.append((name, L, fn))
    if only:
        show = [x for x in show if x[0].split()[0].rstrip('!-') == only or x[0] == only]
        if not show: raise SystemExit(f'Unknown stunt {only!r}; choose from '+', '.join(n for n, _, _ in SHOW))
        show = [('stand', 1.5, hold)]+show
    # A short neutral hold after every stunt lets the robot settle.
    out = []
    for item in show:
        out += [item, ('settle', 1., hold)]
    return out


class Director:
    def __init__(self, robot, show, loop=False):
        self.robot, self.show, self.loop = robot, show, loop
        self.index, self.start = 0, 0.
        self.sm = np.zeros(18); self.state = {}
        self.log = []; self.stats = None; self.new_stats()
    @property
    def name(self): return self.show[self.index][0]
    def new_stats(self):
        self.stats = dict(stunt=self.name, start_s=round(self.start, 2), max_tilt_deg=0., min_foot_contacts=6,
                          max_body_height_m=0., body_yaw_change_deg=0., _yaw0=self.yaw())
    def yaw(self):
        w, x, y, z = self.robot.data.qpos[3:7]
        return math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    def done(self): return self.index >= len(self.show)
    def control(self):
        """One 10 ms control tick followed by ten 1 ms physics steps."""
        r = self.robot; d = r.data
        t = d.time-self.start
        name, L, fn = self.show[self.index]
        self.state['qpos'] = d.qpos.copy()
        q = fn(t, self.state)
        for _ in range(round(DT/r.model.opt.timestep)): r.step(self.sm)
        self.sm = np.clip(self.sm+.35*(q-self.sm), self.sm-.047171, self.sm+.047171)
        self.measure()
        if d.time-self.start >= L-1e-9:
            self.finish()
            self.index += 1; self.start = d.time
            if self.done() and self.loop: self.index = 0
            if not self.done(): self.new_stats()
    def measure(self):
        d, m, s = self.robot.data, self.robot.model, self.stats
        tilt = math.degrees(math.acos(np.clip(d.xmat[m.body('BODY').id][8], -1, 1)))
        feet = {m.geom(f'L{j}_foot_contact').id for j in range(1, 7)}
        touching = {c.geom1 for c in d.contact} | {c.geom2 for c in d.contact}
        s['max_tilt_deg'] = max(s['max_tilt_deg'], tilt)
        s['min_foot_contacts'] = min(s['min_foot_contacts'], len(feet & touching))
        s['max_body_height_m'] = max(s['max_body_height_m'], float(d.qpos[2]))
        s['body_yaw_change_deg'] = (self.yaw()-s['_yaw0']+180) % 360-180
    def finish(self):
        s = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in self.stats.items() if not k.startswith('_')}
        if s['stunt'] != 'settle':
            self.log.append(s)
            print(f"{s['stunt']:>14}: tilt {s['max_tilt_deg']:5.1f} deg, min feet {s['min_foot_contacts']}, "
                  f"peak body {100*s['max_body_height_m']:.1f} cm, yaw {s['body_yaw_change_deg']:+.0f} deg", flush=True)


def label(director):
    if director.done(): return ''
    name = director.name
    if name == 'settle' and director.index > 0: name = director.show[director.index-1][0]
    return name.upper()


def run_viewer(director, seconds):
    import mujoco.viewer
    r = director.robot
    with mujoco.viewer.launch_passive(r.model, r.data) as viewer:
        viewer.cam.distance = .85; viewer.cam.azimuth = 135; viewer.cam.elevation = -22
        viewer.opt.geomgroup[3] = 0
        start = time.monotonic()
        while viewer.is_running() and not director.done() and r.data.time < seconds:
            with viewer.lock():
                director.control()
                viewer.cam.lookat[:] = .9*viewer.cam.lookat+.1*np.r_[r.data.qpos[:2], .08]
            try: viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_200, mujoco.mjtGridPos.mjGRID_TOPLEFT, label(director), ''))
            except AttributeError: pass
            viewer.sync()
            time.sleep(max(0, r.data.time-(time.monotonic()-start)))


def run_video(director, path, seconds, width=1280, height=720):
    import imageio.v2 as imageio
    r = director.robot; m = r.model
    m.vis.global_.offwidth, m.vis.global_.offheight = width, height
    m.vis.quality.offsamples = 8; m.vis.quality.shadowsize = 8192  # anti-aliasing, crisp shadows
    renderer = mujoco.Renderer(m, height, width)
    cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.azimuth, cam.elevation = .85, 135, -22; cam.lookat[:] = [0, 0, .08]
    opt = mujoco.MjvOption(); opt.geomgroup[3] = 0
    writer = imageio.get_writer(str(path), fps=50, codec='libx264', macro_block_size=1,
                                ffmpeg_params=['-crf', '16', '-preset', 'slow', '-pix_fmt', 'yuv420p', '-movflags', '+faststart'])
    try:
        from PIL import Image, ImageDraw, ImageFont
        try: font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Bold.ttf', 44)
        except OSError: font = ImageFont.load_default()
    except ImportError: Image = None
    frame = 0
    while not director.done() and r.data.time < seconds:
        director.control(); frame += 1
        if frame % 2 or director.done(): continue
        cam.lookat[:] = .92*cam.lookat+.08*np.r_[r.data.qpos[:2], .08]
        cam.azimuth += .08
        renderer.update_scene(r.data, cam, opt)
        img = renderer.render()
        if Image is not None:
            im = Image.fromarray(img); draw = ImageDraw.Draw(im)
            draw.text((round(40*height/720), round(30*height/720)), label(director), font=font, fill=(255, 255, 255), stroke_width=round(3*height/720), stroke_fill=(0, 0, 0))
            img = np.asarray(im)
        writer.append_data(img)
    writer.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--video', type=Path, help='Render an MP4 instead of opening the viewer')
    ap.add_argument('--only', help='Run a single stunt: '+', '.join(n.split()[0].rstrip('!-') for n, _, _ in SHOW[1:]))
    ap.add_argument('--loop', action='store_true', help='Repeat the show in the viewer')
    ap.add_argument('--seconds', type=float, default=1e9)
    ap.add_argument('--voltage', type=float, default=12.)
    ap.add_argument('--report', type=Path)
    ap.add_argument('--parameters', type=Path, help='BAM parameter file, e.g. a measured 12 V model')
    ap.add_argument('--resolution', choices=['720p', '1080p', '4k'], default='720p', help='Video resolution')
    args = ap.parse_args()
    robot = Robot(voltage=args.voltage, parameter_file=args.parameters)
    director = Director(robot, build_show(args.only), loop=args.loop and not (args.headless or args.video))
    print('ARACHNE stunt show:', ', '.join(n for n, _, _ in director.show if n != 'settle'), flush=True)
    if args.video: run_video(director, args.video, args.seconds, *dict(zip(['720p', '1080p', '4k'], [(1280, 720), (1920, 1080), (3840, 2160)]))[args.resolution])
    elif args.headless:
        while not director.done() and robot.data.time < args.seconds: director.control()
    else: run_viewer(director, args.seconds)
    warnings = int(sum(w.number for w in robot.data.warning))
    result = dict(duration_s=round(float(robot.data.time), 2), solver_warnings=warnings, stunts=director.log)
    if args.report: args.report.write_text(json.dumps(result, indent=2)+'\n')
    if args.video: print('Video written to', args.video)


if __name__ == '__main__': main()
