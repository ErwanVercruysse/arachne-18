"""Render the four knee horn screws with their printed bearing eye."""
from pathlib import Path
import json,cadquery as cq,vtk,numpy as np
from vtk.util.numpy_support import numpy_to_vtk
root=Path(__file__).resolve().parent.parent;cache=root/'build/v4/.cad_cache'
meta=json.loads((cache/'placed.json').read_text());ren=vtk.vtkRenderer();ren.SetBackground(.92,.94,.95)
keep=('L2_tibia','L2_femur','L2_KNEE_STS3215','L2_knee_horn_-1','L2_knee_horn_1')
for o in meta:
 n=o['name']
 if n not in keep and not n.startswith('L2_knee_M3x6_'):continue
 s=cq.Shape.importBrep(str(cache/o['file']));vs,ts=s.tessellate(.15,.12)
 pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(np.array([v.toTuple() for v in vs]),deep=True));cs=vtk.vtkCellArray()
 for t in ts:
  cs.InsertNextCell(3)
  for v in t:cs.InsertCellPoint(v)
 poly=vtk.vtkPolyData();poly.SetPoints(pts);poly.SetPolys(cs);norm=vtk.vtkPolyDataNormals();norm.SetInputData(poly);norm.SetFeatureAngle(55)
 mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(norm.GetOutputPort());act=vtk.vtkActor();act.SetMapper(mapper)
 color=(.15,.55,.48) if 'tibia' in n else (.7,.4,.18) if 'femur' in n else (.18,.2,.22) if 'STS3215' in n else (.73,.75,.77)
 prop=act.GetProperty();prop.SetColor(*color);prop.SetAmbient(.25);prop.SetDiffuse(.7);prop.SetSpecular(.15);ren.AddActor(act)
for pos in [(160,120,200),(-150,300,150)]:
 l=vtk.vtkLight();l.SetPosition(*pos);l.SetFocalPoint(0,211,20);l.SetIntensity(.7);ren.AddLight(l)
cam=ren.GetActiveCamera();cam.SetPosition(250,215,95);cam.SetFocalPoint(0,211,20);cam.SetViewUp(0,0,1);cam.ParallelProjectionOn();cam.SetParallelScale(43)
win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.AddRenderer(ren);win.SetSize(1400,1000);ren.ResetCameraClippingRange();win.Render()
g=vtk.vtkWindowToImageFilter();g.SetInput(win);g.Update();w=vtk.vtkPNGWriter();w.SetFileName(str(root/'assets/cad-horn-detail.png'));w.SetInputConnection(g.GetOutputPort());w.Write();win.Finalize()
