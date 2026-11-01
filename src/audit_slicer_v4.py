"""Bambu Studio geometry audit using its single-extruder A1 0.4 mm profile.

This avoids H2D headless extruder-mapping errors. It is a support/geometry
proxy, NOT H2D G-code. Use the H2D profile for the physical print.
"""
from pathlib import Path
import subprocess
import json
import re
import hashlib

HERE=Path(__file__).resolve().parent.parent/'build/v4'
ROOT=Path('/Applications/BambuStudio.app/Contents/Resources/profiles/BBL')
APP='/Applications/BambuStudio.app/Contents/MacOS/BambuStudio'
def resolved(path,category):
    data=json.loads(path.read_text());result={}
    if data.get('inherits'):result.update(resolved(ROOT/category/(data['inherits']+'.json'),category))
    for inc in data.get('include',[]):result.update(resolved(ROOT/category/(inc+'.json'),category))
    result.update({k:v for k,v in data.items() if k not in ('inherits','include')})
    return result
report={"slicer":"Bambu Studio 02.08.02.61","printer_profile":"A1 0.4 mm — geometry proxy only",
        "target_printer":"Bambu H2D; reslice before printing","layer_height_mm":.2,
        "walls":5,"infill":"35% gyroid","parts":{}}
for stl in sorted((HERE/'STL').glob('0*.stl')):
    out=Path('/tmp/arachne-release-v4-slicer')/stl.stem;out.mkdir(parents=True,exist_ok=True)
    profile=resolved(Path(__file__).resolve().parent/'v4_slicer_profile.json','process')
    if 'tpu' in stl.stem:profile.update(enable_support='0',sparse_infill_density='20%')
    (out/'process.json').write_text(json.dumps(profile))
    filament='Generic TPU @BBL A1.json' if 'tpu' in stl.stem else 'Generic PLA @BBL A1.json'
    (out/'machine.json').write_text(json.dumps(resolved(ROOT/'machine/Bambu Lab A1 0.4 nozzle.json','machine')))
    (out/'filament.json').write_text(json.dumps(resolved(ROOT/'filament'/filament,'filament')))
    cmd=[APP,'--slice','0','--orient','0','--arrange','1','--load-settings',
         str(out/'machine.json')+';'+str(out/'process.json'),
         '--load-filaments',str(out/'filament.json'),'--outputdir',str(out),str(stl)]
    run=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (out/'log.txt').write_text(run.stdout)
    result=json.loads((out/'result.json').read_text())
    if run.returncode or result.get('return_code',0):raise RuntimeError(result)
    plates=result.get('sliced_plates',[])
    feature='';support_e=0.;total_e=0.
    for gcode in out.glob('*.gcode'):
        for line in gcode.read_text().splitlines():
            if line.startswith('; FEATURE: '):feature=line[11:]
            if line.startswith(('G0 ','G1 ')):
                m=re.search(r'(?:^| )E(-?(?:\d+(?:\.\d*)?|\.\d+))',line)
                if m and float(m[1])>0:
                    total_e+=float(m[1])
                    if feature.startswith('Support'):support_e+=float(m[1])
    report['parts'][stl.stem]={"sha256":hashlib.sha256(stl.read_bytes()).hexdigest(),
         "warning_message":[w for p in plates for w in p.get('warning_message',[])],
         "support_g_approx":round(support_e*.00298,2),"total_extrusion_g_approx":round(total_e*.00298,2),
         "material":"TPU" if 'tpu' in stl.stem else "PLA", "raw_result":result}
    print(stl.stem,report['parts'][stl.stem]['support_g_approx'],report['parts'][stl.stem]['warning_message'],flush=True)
(HERE/'slicer_audit.json').write_text(json.dumps(report,indent=2)+'\n')
