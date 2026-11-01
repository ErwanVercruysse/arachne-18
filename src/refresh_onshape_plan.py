"""Refresh the native-mate plan from the current CAD and Onshape instances.

Requires the owner's external mode-0600 API key file; never copies credentials
into the repository. The assembly must already contain every CAD instance.
"""
from pathlib import Path
import argparse
import json
from onshape_mates import Client, ROOT, OUT, PLAN

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--key-file',type=Path,required=True)
args=parser.parse_args()
meta=json.loads((ROOT/'simulation/cad_instances.json').read_text())
links=json.loads((ROOT/'simulation/mass_properties.json').read_text())['links']
joints=json.loads((ROOT/'simulation/joint_map.json').read_text())['joints']
assembly=Client(args.key_file).call('GET')
instances={x['name'].rsplit(' <',1)[0]:x for x in assembly['rootAssembly']['instances']}
assert len(instances)==len(meta)
assert {x['name'] for x in meta}==set(instances)

anchors={group:link['parts'][0] for group,link in links.items()}
fastened=[dict(name='F_'+part,type='FASTENED',parent=anchor,child=part)
          for group,link in links.items()
          for anchor in [anchors[group]] for part in link['parts'][1:]
          if part in instances]
revolute=[dict(name='R_'+j['name'],type='REVOLUTE',
               parent=anchors[j['parent']],child=anchors[j['child']],
               origin_m=j['origin_m'],axis=j['axis'],
               range_delta_deg=j['range_delta_deg']) for j in joints]
plan=dict(documentId=PLAN['documentId'],workspaceId=PLAN['workspaceId'],
          elementId=PLAN['elementId'],fix=anchors['BODY'],
          instances={name:dict(id=item['id'],partId=item['partId'])
                     for name,item in instances.items()},
          fastened=fastened,revolute=revolute)
assert len(fastened)==len(meta)-len(links)
assert len(revolute)==18
path=OUT/'onshape_mate_plan.json'
path.write_text(json.dumps(plan,indent=2)+'\n')
print(f'{path}: {len(instances)} instances, {len(fastened)} fastened, {len(revolute)} revolute')
