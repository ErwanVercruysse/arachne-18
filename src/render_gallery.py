"""Render the actual exported CAD meshes; no generated geometry or image editing."""
from pathlib import Path
import json
import vtk
ROOT=Path(__file__).resolve().parent.parent
SIM=ROOT/'simulation'; OUT=ROOT/'assets'; OUT.mkdir(exist_ok=True)
items=json.loads((SIM/'cad_instances.json').read_text())
links=json.loads((SIM/'mass_properties.json').read_text())['links']

def render(filename,hide=(),camera=(440,-620,470),scale=285,exploded=False):
 ren=vtk.vtkRenderer();ren.SetBackground(.065,.10,.13);ren.SetBackground2(.21,.29,.32);ren.GradientBackgroundOn()
 for o in items:
  if o['name'] in hide:continue
  reader=vtk.vtkSTLReader();reader.SetFileName(str(SIM/'meshes'/(o['name']+'.stl')));reader.Update()
  normals=vtk.vtkPolyDataNormals();normals.SetInputConnection(reader.GetOutputPort());normals.SetFeatureAngle(45)
  mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(normals.GetOutputPort())
  actor=vtk.vtkActor();actor.SetMapper(mapper);actor.SetScale(1000)
  pos=[v*1000 for v in links[o['group']]['origin_neutral_m']]
  if exploded and o['name']=='05_SHELL':pos[2]+=95
  actor.SetPosition(*pos)
  prop=actor.GetProperty();prop.SetColor(*o['color']);prop.SetAmbient(.27);prop.SetDiffuse(.8);prop.SetSpecular(.3);prop.SetSpecularPower(50)
  ren.AddActor(actor)
 for pos,color,intensity in [((250,-450,650),(1,.97,.9),1.0),((-450,-200,100),(.55,.8,1),.65),((200,480,350),(1,.64,.3),.85)]:
  light=vtk.vtkLight();light.SetLightTypeToSceneLight();light.SetPosition(*pos);light.SetFocalPoint(0,0,20);light.SetColor(*color);light.SetIntensity(intensity);ren.AddLight(light)
 cam=ren.GetActiveCamera();cam.SetPosition(*camera);cam.SetFocalPoint(0,0,30 if exploded else 5);cam.SetViewUp(0,0,1);cam.ParallelProjectionOn();cam.SetParallelScale(scale)
 win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.AddRenderer(ren);win.SetSize(1800,1200);win.SetMultiSamples(8);ren.ResetCameraClippingRange();win.Render()
 grab=vtk.vtkWindowToImageFilter();grab.SetInput(win);grab.SetScale(1);grab.Update()
 writer=vtk.vtkPNGWriter();writer.SetFileName(str(OUT/filename));writer.SetInputConnection(grab.GetOutputPort());writer.Write();win.Finalize();print(filename,flush=True)
render('cad-overview.png')
render('cad-internals.png',hide=('05_SHELL',),camera=(350,-500,560))
render('cad-top.png',camera=(0,-.01,700),scale=282)
render('cad-service.png',camera=(400,-580,420),scale=300,exploded=True)
