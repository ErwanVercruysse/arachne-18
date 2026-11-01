"""Render a close, orthographic top view of one actual CAD leg for review."""
from pathlib import Path
import json
import cadquery as cq
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk

ROOT=Path(__file__).resolve().parent.parent
CACHE=ROOT/'build/v4/.cad_cache'
items=json.loads((CACHE/'placed.json').read_text())
renderer=vtk.vtkRenderer()
renderer.SetBackground(.055,.08,.11)
renderer.SetBackground2(.15,.21,.24)
renderer.GradientBackgroundOn()

for item in items:
    name=item['name']
    if not name.startswith('L2_') or item['kind'] not in ('printed','servo','hardware','fastener','factory_fastener','insert'):
        continue
    shape=cq.Shape.importBrep(str(CACHE/item['file']))
    vertices,triangles=shape.tessellate(.16,.13)
    points=vtk.vtkPoints()
    points.SetData(numpy_to_vtk(np.asarray([v.toTuple() for v in vertices]),deep=True))
    cells=vtk.vtkCellArray()
    for triangle in triangles:
        cells.InsertNextCell(3)
        for index in triangle:
            cells.InsertCellPoint(index)
    poly=vtk.vtkPolyData();poly.SetPoints(points);poly.SetPolys(cells)
    normals=vtk.vtkPolyDataNormals()
    normals.SetInputData(poly)
    normals.SetFeatureAngle(55)
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(normals.GetOutputPort())
    actor=vtk.vtkActor();actor.SetMapper(mapper)
    color=((.18,.20,.22) if item['kind']=='servo' else
           (.72,.75,.77) if item['kind']=='hardware' else
           (.14,.56,.47) if 'tibia' in name or 'foot' in name else
           (.68,.39,.20) if 'femur' in name or 'knee' in name else
           (.20,.44,.61))
    prop=actor.GetProperty()
    prop.SetColor(*item["color"])
    prop.SetAmbient(.28)
    prop.SetDiffuse(.68)
    prop.SetSpecular(.12)
    renderer.AddActor(actor)

for position,intensity in [((160,-140,450),.8),((-180,300,450),.65)]:
    light=vtk.vtkLight()
    light.SetLightTypeToSceneLight()
    light.SetPosition(*position)
    light.SetFocalPoint(0,155,0)
    light.SetIntensity(intensity)
    renderer.AddLight(light)

camera=renderer.GetActiveCamera()
camera.SetPosition(0,155,700)
camera.SetFocalPoint(0,155,0)
camera.SetViewUp(1,0,0)
camera.ParallelProjectionOn()
camera.SetParallelScale(70)
window=vtk.vtkRenderWindow()
window.SetOffScreenRendering(1)
window.AddRenderer(renderer)
window.SetSize(1800,1000)
window.SetMultiSamples(8)
renderer.ResetCameraClippingRange()
window.Render()
grab=vtk.vtkWindowToImageFilter();grab.SetInput(window);grab.Update()
output=ROOT/'assets/cad-leg-top.png'
writer=vtk.vtkPNGWriter();writer.SetFileName(str(output))
writer.SetInputConnection(grab.GetOutputPort());writer.Write()
window.Finalize()
print(output)
