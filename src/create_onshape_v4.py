"""Create/resume a v4 assembly from its imported Part Studio; author mates separately."""
from pathlib import Path
import argparse,json
from onshape_api import Client
ROOT=Path(__file__).resolve().parent.parent
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--key-file',required=True);p.add_argument('--document',required=True);p.add_argument('--workspace',required=True);p.add_argument('--part-studio',required=True)
a=p.parse_args();c=Client(a.key_file)
state=ROOT/'build/onshape/v4_assembly.json';state.parent.mkdir(parents=True,exist_ok=True)
doc=a.document;work=a.workspace;ps=a.part_studio
parts=c.call('GET',f'/api/v17/parts/d/{doc}/w/{work}/e/{ps}')
meta=json.loads((ROOT/'simulation/cad_instances.json').read_text())
assert len(parts)==len(meta) and {o['name'] for o in parts}=={o['name'] for o in meta}
if state.exists():assembly=json.loads(state.read_text())
else:
 assembly=c.call('POST',f'/api/v17/assemblies/d/{doc}/w/{work}',json.dumps({'name':'ARACHNE 18 — v4 articulated assembly'}).encode())
 state.write_text(json.dumps(assembly,indent=2))
eid=assembly['id'];prefix=f'/api/v17/assemblies/d/{doc}/w/{work}/e/{eid}'
current=c.call('GET',prefix);existing={o['partId'] for o in current['rootAssembly']['instances']}
missing=[o for o in parts if o['partId'] not in existing]
for start in range(0,len(missing),40):
 batch=missing[start:start+40]
 body={'transformGroups':[{'transform':[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
  'instances':[dict(documentId=doc,elementId=ps,partId=o['partId'],isAssembly=False,isFixed=o['name']=='01_CHASSIS') for o in batch]}]}
 c.call('POST',prefix+'/transformedinstances',json.dumps(body).encode())
 print('Inserted',start+len(batch),'/',len(missing),flush=True)
current=c.call('GET',prefix);byid={o['partId']:o for o in parts}
instances={byid[o['partId']]['name']:dict(id=o['id'],partId=o['partId']) for o in current['rootAssembly']['instances']}
links=json.loads((ROOT/'simulation/mass_properties.json').read_text())['links'];joints=json.loads((ROOT/'simulation/joint_map.json').read_text())['joints']
anchors={g:v['parts'][0] for g,v in links.items()}
fastened=[dict(name='F_'+part,type='FASTENED',parent=anchors[g],child=part) for g,link in links.items() for part in link['parts'][1:] if part in instances]
revolute=[dict(name='R_'+j['name'],type='REVOLUTE',parent=anchors[j['parent']],child=anchors[j['child']],origin_m=j['origin_m'],axis=j['axis'],range_delta_deg=j['range_delta_deg']) for j in joints]
plan=dict(revision='v4',documentId=doc,workspaceId=work,elementId=eid,partStudioId=ps,fix='01_CHASSIS',instances=instances,fastened=fastened,revolute=revolute)
assert len(instances)==336 and len(fastened)==317 and len(revolute)==18
(ROOT/'simulation/onshape_mate_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
print('Assembly',eid,flush=True)
