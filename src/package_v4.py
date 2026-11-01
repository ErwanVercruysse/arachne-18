"""Publish the validated local v4 build into the repository's canonical folders."""
from pathlib import Path
import json,shutil,zipfile
ROOT=Path(__file__).resolve().parent.parent;BUILD=ROOT/'build/v4'
for report in ('integration_validation','leg_fit','body_fit','body_mount_sequence'):
 assert json.loads((BUILD/f'{report}.json').read_text())['passed'],report
for folder,pattern in [('print/stl','*.stl'),('cad/parts','*.step')]:
 target=ROOT/folder;target.mkdir(parents=True,exist_ok=True)
 for p in target.glob(pattern):p.unlink()
source_target=[('STL','print/stl'),('STEP_parts','cad/parts')]
for source,target in source_target:
 for p in (BUILD/source).iterdir():shutil.copy2(p,ROOT/target/p.name)
for source,target in [('ARACHNE_18_v4_assembly.step','cad/ARACHNE_18_assembly.step'),('ARACHNE_18_v4_parts.step','cad/ARACHNE_18_parts.step'),('parameters.json','cad/parameters.json'),('print_bom.csv','print/print_bom.csv'),('cad_validation.json','validation/cad_validation.json'),('integration_validation.json','validation/integration_v4.json'),('slicer_audit.json','validation/slicer_v4.json')]:shutil.copy2(BUILD/source,ROOT/target)
# Normalize the BOM so both Git and the ZIP use consistent line endings.
p=ROOT/'print/print_bom.csv';p.write_text(p.read_text())
if (BUILD/'leg_fit.json').exists():shutil.copy2(BUILD/'leg_fit.json',ROOT/'validation/leg_v4.json')
for name in ('body_fit','body_mount_sequence'):
 shutil.copy2(BUILD/f'{name}.json',ROOT/'validation'/f'{name}_v4.json')
# Only assets referenced by the current MJCF belong in the current mesh folder.
mesh_names={o['name']+'.stl' for o in json.loads((ROOT/'simulation/cad_instances.json').read_text())}
for p in (ROOT/'simulation/meshes').glob('*.stl'):
 if p.name not in mesh_names:p.unlink()
# Stable downloadable print bundle, without build caches or credentials.
with zipfile.ZipFile(ROOT/'print/ARACHNE_18_v4_print_pack.zip','w',zipfile.ZIP_DEFLATED) as z:
 for p in sorted((ROOT/'print/stl').glob('*.stl')):z.write(p,'print/stl/'+p.name)
 for rel in ('print/print_bom.csv','docs/build-guide.md','docs/hardware-bom.csv','assets/cad-horn-detail.png'):z.write(ROOT/rel,rel)
 z.writestr('START_HERE.txt','ARACHNE 18 v4 print pack\n\nRead docs/build-guide.md. Oriented millimetre STL files are in print/stl/.\nPrint quantities: print/print_bom.csv. Hardware: docs/hardware-bom.csv.\nReslice for the H2D; prototype one leg before printing the remaining five.\nProject: https://github.com/Tom-Michiels/arachne-18\n')
print('Published v4 CAD, print pack and validation reports.')
