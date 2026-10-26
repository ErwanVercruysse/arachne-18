"""Small command-conditioned locomotion policy, shared by NumPy and MLX.

Tripod phase prior, C2 swing/stance joins, geometric IK, tilt feedback.
Parameters are learned by episodic reward-based policy search (CEM), not PPO.
All angles remain inside the original CAD joint limits.
"""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NAMES = ['frequency_hz', 'stance_fraction', 'lift_m', 'stride_gain', 'yaw_gain',
         'height_offset_m', 'radial_offset_m', 'tilt_feedback',
         'phase_front', 'phase_middle', 'phase_rear', 'lateral_gain']
LOW = np.array([1.0,.52,.005,.6,.6,-.006,-.008,0.,-.045,-.045,-.045,.7])
HIGH = np.array([3.8,.72,.021,1.5,1.6,.007,.008,.8,.045,.045,.045,1.4])
INITIAL = np.array([2.1,.60,.013,1.05,1.1,0.,0.,.2,0.,0.,0.,1.05])
AZIMUTH = np.deg2rad([45,90,135,225,270,315])
RADIAL = np.stack([np.cos(AZIMUTH), np.sin(AZIMUTH)], -1)
COXA = .086 * RADIAL
L1 = .078
V = np.array([.02394328259+.02394310446, -.1078466252])
V[0] /= np.sqrt(2)
L2 = float(np.linalg.norm(V))
A0 = float(np.deg2rad(15.))
B0 = float(np.arctan2(V[1], V[0])-A0)
R0 = float(.05 + L1*np.cos(A0) + L2*np.cos(A0+B0))
Z0 = float(L1*np.sin(A0) + L2*np.sin(A0+B0))
LIMITS = np.tile([np.deg2rad(20), np.deg2rad(15), np.deg2rad(15)], 6)


def smooth5(u):
    return u*u*u*(10+u*(-15+6*u))


def frequency(params, command, xp=np):
    speed = xp.sqrt(command[:,0]**2+command[:,1]**2+(.22*command[:,2])**2)
    return params[:,0]*(.55+.45*xp.clip(speed/.14,0,1.6))


def rotate(q, v, xp=np, inverse=False):
    w, u = q[:, :1], q[:, 1:]
    cross = xp.stack([u[:,1]*v[:,2]-u[:,2]*v[:,1],
                      u[:,2]*v[:,0]-u[:,0]*v[:,2],
                      u[:,0]*v[:,1]-u[:,1]*v[:,0]], -1)
    return (2*w*w-1)*v + 2*u*xp.sum(u*v, axis=-1, keepdims=True) + (-2 if inverse else 2)*w*cross


def feet(qpos, xp=np):
    q = qpos[:, 7:].reshape(-1,6,3)
    az = xp.array(AZIMUTH)[None,:]+q[:,:,0]
    a, b = A0+q[:,:,1], B0+q[:,:,2]
    r = .05+L1*xp.cos(a)+L2*xp.cos(a+b)
    z = L1*xp.sin(a)+L2*xp.sin(a+b)
    local = xp.stack([xp.array(COXA[:,0])[None,:]+r*xp.cos(az),
                      xp.array(COXA[:,1])[None,:]+r*xp.sin(az), z], -1)
    world = [rotate(qpos[:,3:7], local[:,i,:], xp)+qpos[:,:3] for i in range(6)]
    return xp.stack(world, axis=1)


def plan(params, phase, command, qpos, elapsed, xp=np, gyro=None, gravity=None):
    """Foot targets of the policy, before IK: x, y, z [N,6] in the body frame (hip-relative z),
    per-leg phase ph [N,6], swing progress u [N,6] (0 in stance), duty [N,1] and the lift part of z.
    gravity [N,3] (body frame, unit) replaces the orientation in qpos, e.g. from a real IMU."""
    p = params
    duty = p[:,1:2]
    offsets = xp.stack([p[:,8],p[:,9],p[:,10],p[:,10],p[:,9],p[:,8]], -1)
    ph = (phase[:,None]+xp.array([0.,.5,0.,.5,0.,.5])[None,:]+offsets) % 1
    u = xp.clip((ph-duty)/(1-duty),0,1)
    ratio = (1-duty)/duty
    travel = xp.where(ph < duty, .5-ph/duty, -.5-ratio*u+(1+ratio)*smooth5(u))
    speed = xp.sqrt(command[:,0:1]**2+command[:,1:2]**2+(.22*command[:,2:3])**2)
    lift_shape = xp.sin(np.pi*u)**4
    if p.shape[1] > 12:
        # Wider C2 swing clearance reduces late lift-off and early touchdown.
        # The optional blend preserves every existing 12-parameter policy.
        broad = 64*u**3*(1-u)**3
        lift_shape = (1-p[:,12:13])*lift_shape + p[:,12:13]*broad
    lift = p[:,2:3]*(.35+.65*xp.clip(speed/.14,0,1))*lift_shape
    ramp = smooth5(xp.clip(elapsed/1.2,0,1))
    radius = R0 + p[:,6:7]
    neutral = xp.array(COXA)[None,:,:]+radius[:,:,None]*xp.array(RADIAL)[None,:,:]
    vx = command[:,0:1]*p[:,3:4]-command[:,2:3]*p[:,4:5]*neutral[:,:,1]
    vy = command[:,1:2]*p[:,3:4]*p[:,11:12]+command[:,2:3]*p[:,4:5]*neutral[:,:,0]
    duration = duty / frequency(p,command,xp)[:,None]
    x = neutral[:,:,0]+travel*duration*vx*ramp
    y = neutral[:,:,1]+travel*duration*vy*ramp
    if gravity is None:
        gravity = rotate(qpos[:,3:7], xp.array([[0.,0.,-1.]])+xp.zeros((p.shape[0],3)), xp, True)
    if gyro is not None:
        # Gravity prediction g_dot = -omega cross g adds gyro damping without
        # observing absolute heading, global position or true linear velocity.
        cross = xp.stack([gyro[:,1]*gravity[:,2]-gyro[:,2]*gravity[:,1],
                          gyro[:,2]*gravity[:,0]-gyro[:,0]*gravity[:,2],
                          gyro[:,0]*gravity[:,1]-gyro[:,1]*gravity[:,0]], -1)
        gravity = gravity - .025*cross
    correction = p[:,7:8]*(gravity[:,0:1]*x + gravity[:,1:2]*y)
    activity = xp.clip(speed/.025,0,1)
    z = Z0-p[:,5:6] + lift*ramp*activity + xp.clip(correction,-.012,.012)
    return dict(x=x, y=y, z=z, ph=ph, u=xp.where(ph < duty, 0., u), duty=duty, lift=lift*ramp*activity)


def leg_ik(x, y, z, xp=np, low=None, high=None):
    """Geometric IK of the six legs: foot targets [N,6] -> joint angles [N,18], clipped to the
    original policy limits, or to low/high [18] when given."""
    dx, dy = x-xp.array(COXA[:,0])[None,:], y-xp.array(COXA[:,1])[None,:]
    yaw = xp.arctan2(dy,dx)-xp.array(AZIMUTH)[None,:]
    yaw = xp.arctan2(xp.sin(yaw),xp.cos(yaw))
    radial = xp.sqrt(dx*dx+dy*dy)-.05
    b = -xp.arccos(xp.clip((radial*radial+z*z-L1*L1-L2*L2)/(2*L1*L2),-.999999,.999999))
    a = xp.arctan2(z,radial)-xp.arctan2(L2*xp.sin(b),L1+L2*xp.cos(b))
    q = xp.stack([yaw,a-A0,b-B0],-1).reshape(-1,18)
    if low is None: return xp.clip(q,-xp.array(LIMITS)+.004,xp.array(LIMITS)-.004)
    return xp.clip(q,xp.array(low),xp.array(high))


def target(params, phase, command, qpos, elapsed, xp=np, gyro=None, gravity=None):
    """Batch inputs [N,P], [N], [N,3], [N,25]; returns [N,18]."""
    f = plan(params, phase, command, qpos, elapsed, xp, gyro, gravity)
    return leg_ik(f['x'], f['y'], f['z'], xp)


def save(path, params, **metadata):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(dict(algorithm='CEM episodic policy search',
        parameter_names=NAMES+(['broad_lift_blend'] if len(params)>12 else []), parameters=np.asarray(params).tolist(), **metadata), indent=2)+'\n')
    temp.replace(path)


def load(path):
    return np.array(json.loads(Path(path).read_text())['parameters'])
