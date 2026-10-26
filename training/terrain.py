"""Seeded terrain for the reference MuJoCo/BAM curriculum, in SI units.

No terrain data is passed to the controller. A flat spawn disk and smooth
entry prevent reset penetration. Heightfields retain the original robot and
collision proxies; the local fused Metal backend cannot simulate these fields.
"""
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from gait import ROOT, smooth5

EXTENT = 2.0
RESOLUTION = 401  # 10 mm cells across a 4 x 4 m patch.


def resolution(spec):
    # Cobblestone joints are narrower than a 10 mm cell: those use 5 mm cells.
    return spec.get('resolution',801 if spec['kind']=='cobbles' else RESOLUTION)


def cobbles(x,y,spec,rng):
    """Rounded stones (Voronoi cells of a jittered grid) with sunken joints between them."""
    from scipy.spatial import cKDTree
    size=spec.get('stone_m',.10);n=int(2*EXTENT/size)+3
    g=(np.arange(n)-n/2)*size
    centres=np.stack(np.meshgrid(g,g),-1).reshape(-1,2)+rng.uniform(-.3,.3,(n*n,2))*size
    tops=rng.uniform(-1,1,len(centres))*spec.get('height_m',.004)
    d,i=cKDTree(centres).query(np.stack([x.ravel(),y.ravel()],-1),k=2)
    edge=((d[:,1]-d[:,0])/2).reshape(x.shape)          # distance to the joint with the next stone
    dome=spec.get('dome_m',.004)*(1-np.clip(d[:,0].reshape(x.shape)/(.6*size),0,1)**2)
    joint=spec.get('joint_m',.015)/2;depth=spec.get('joint_depth_m',.008)
    # Stone edges are rounded down into the joint over 1 cm, the joint bottom is flat.
    z=tops[i[:,0]].reshape(x.shape)+dome-depth*(1-np.clip((edge-joint)/.01,0,1))**2
    return z-np.median(z)


def height_map(spec, seed):
    rng=np.random.default_rng(seed)
    xy=np.linspace(-EXTENT,EXTENT,resolution(spec))
    x,y=np.meshgrid(xy,xy)
    kind=spec['kind']; amplitude=spec.get('amplitude_m',0.)
    # Choose random directions and phases, including in validation.
    angle=rng.uniform(0,2*np.pi)
    a=np.cos(angle)*x+np.sin(angle)*y
    b=-np.sin(angle)*x+np.cos(angle)*y
    lengths=spec.get('wavelength_m',[.12,.24])
    waves=sum(np.sin(2*np.pi*(np.cos(t)*x+np.sin(t)*y)/rng.uniform(*lengths)+rng.uniform(0,2*np.pi))
              for t in rng.uniform(0,2*np.pi,5))/5
    if kind=='flat':z=np.zeros_like(x)
    elif kind=='bumps':z=amplitude*waves
    elif kind=='slope':
        # A finite smooth hill, rescaled below to also bound the entry slope.
        z=np.tan(np.deg2rad(spec['slope_deg']))*.55*np.tanh(a/.55)
    elif kind=='terraces':
        # Low raised plateaus, random orientation. Edges are one grid cell,
        # not mathematically vertical stair risers; heights are never enlarged.
        z=amplitude*(np.sin(2*np.pi*a/.24+rng.uniform(0,2*np.pi))>0)
    elif kind=='cobbles':z=cobbles(x,y,spec,rng)
    elif kind=='mixed':
        z=amplitude*waves + amplitude*.6*(np.sin(2*np.pi*a/.28)>0)
        z+=np.tan(np.deg2rad(spec.get('slope_deg',0)))*.4*np.tanh(b/.4)
    else:raise ValueError('Unknown terrain kind: '+kind)
    # All six nominal feet fit within radius .25 m. Transition outside .32 m.
    z*=smooth5(np.clip((np.sqrt(x*x+y*y)-.32)/.22,0,1))
    if kind=='slope':
        gradients=np.gradient(z,2*EXTENT/(resolution(spec)-1))
        grade=np.sqrt(gradients[0]**2+gradients[1]**2).max()
        z*=min(1.,np.tan(np.deg2rad(spec['slope_deg']))/max(grade,1e-12))
    return z.astype(np.float64)


def make_model(spec,seed):
    from obstacles import KINDS,make_model as make_obstacles
    if spec['kind'] in KINDS:return make_obstacles(spec,seed)
    tree=ET.parse(ROOT/'simulation/arachne.xml');root=tree.getroot()
    root.find('compiler').set('meshdir',str(ROOT/'simulation/meshes'))
    heights=height_map(spec,seed)
    if spec['kind']!='flat':
        lo=float(heights.min());scale=max(float(np.ptp(heights)),.001)
        ET.SubElement(root.find('asset'),'hfield',name='curriculum_terrain',
                      nrow=str(resolution(spec)),ncol=str(resolution(spec)),
                      size=f'{EXTENT} {EXTENT} {scale} .1')
        ground=root.find("worldbody/geom[@name='ground']")
        ground.attrib.pop('size',None)
        ground.attrib.update(type='hfield',hfield='curriculum_terrain',pos=f'0 0 {lo}')
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    if spec['kind']!='flat':
        model.hfield_data[:]=((heights-lo)/scale).ravel()
    return model,dict(spec=spec,seed=int(seed),extent_m=EXTENT,cell_size_m=2*EXTENT/(resolution(spec)-1),
                      height_min_m=float(heights.min()),height_max_m=float(heights.max()))


def height_below(model,data,x,y):
    """Exact collision-surface ray, used only for reward and diagnostics."""
    origin=np.array([x,y,2.]);direction=np.array([0.,0.,-1.])
    groups=np.array([1,0,0,0,0,0],dtype=np.uint8)
    geom_id=np.array([-1],dtype=np.int32)
    distance=mujoco.mj_ray(model,data,origin,direction,groups,True,-1,geom_id)
    if distance<0:raise RuntimeError('Terrain ray missed the ground')
    return 2.-distance
