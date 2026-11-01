"""Apply the reviewable ARACHNE mate plan via the official Onshape REST API.

Credentials are read from a separate mode-0600 JSON, never printed or exported.
Only operates on the document/workspace/assembly in onshape_mate_plan.json.
"""
from pathlib import Path
import json, argparse, copy, math, time, secrets, email.utils, hmac, hashlib, base64
import urllib.request, urllib.parse, urllib.error
import numpy as np

ROOT=Path(__file__).resolve().parent.parent
OUT=ROOT/'simulation'
WORK=ROOT/'build/onshape';WORK.mkdir(parents=True,exist_ok=True)
NAME_MAP=json.loads((ROOT/'cad/onshape_name_map.json').read_text())
def native_names(obj):
    if isinstance(obj,dict):return {native_names(k):native_names(v) for k,v in obj.items()}
    if isinstance(obj,list):return [native_names(v) for v in obj]
    if isinstance(obj,str):
        if obj in NAME_MAP:return NAME_MAP[obj]
        for prefix in ('MC_','F_'):
            if obj.startswith(prefix) and obj[len(prefix):] in NAME_MAP:
                return prefix+NAME_MAP[obj[len(prefix):]]
    return obj
PLAN=native_names(json.loads((OUT/'onshape_mate_plan.json').read_text()))
META=native_names(json.loads((OUT/'cad_instances.json').read_text()))
LINKS=native_names(json.loads((OUT/'mass_properties.json').read_text()))['links']
PREFIX=f"/api/v17/assemblies/d/{PLAN['documentId']}/w/{PLAN['workspaceId']}/e/{PLAN['elementId']}"

class Client:
    def __init__(self,keyfile):
        self.keys=json.loads(Path(keyfile).read_text())
    def call(self,method,suffix='',body=None,query=None):
        path=PREFIX+suffix;qs=urllib.parse.urlencode(query or {})
        url='https://cad.onshape.com'+path+('?' + qs if qs else '')
        nonce=secrets.token_hex(16);date=email.utils.formatdate(usegmt=True);ctype='application/json'
        sign='\n'.join([method,nonce,date,ctype,path,qs,'']).lower()
        mac=base64.b64encode(hmac.new(self.keys['secretKey'].encode(),sign.encode(),hashlib.sha256).digest()).decode()
        headers={'Date':date,'On-Nonce':nonce,'Content-Type':ctype,'Accept':'application/json',
                 'Authorization':f"On {self.keys['accessKey']}:HmacSHA256:{mac}"}
        req=urllib.request.Request(url,headers=headers,method=method,data=None if body is None else json.dumps(body).encode())
        try:
            with urllib.request.urlopen(req,timeout=90) as resp:
                raw=resp.read();return json.loads(raw) if raw else {}
        except urllib.error.HTTPError as e:
            raise RuntimeError(f'Onshape HTTP {e.code}: '+e.read().decode()[:1500]) from None

def enum(pid,value,name):return dict(btType='BTMParameterEnum-145',parameterId=pid,enumName=name,value=value)
def boolean(pid,value):return dict(btType='BTMParameterBoolean-144',parameterId=pid,value=value)
def quantity(pid,value,unit='m'):return dict(btType='BTMParameterQuantity-147',parameterId=pid,expression=f'{value:.12g} {unit}',isInteger=False)
def query(pid,queries):return dict(btType='BTMParameterQueryWithOccurrenceList-67',parameterId=pid,queries=queries)
def feature_query(fid):return dict(btType='BTMFeatureQueryWithOccurrence-157',path=[],featureId=fid,queryData='')
def origin_query(instance):return dict(btType='BTMInferenceQueryWithOccurrence-1083',inferenceType='PART_ORIGIN',path=[instance],deterministicIds=[''])

def connector(name,source,position=(0,0,0),rotation=0,rotation_axis='ABOUT_Z'):
    return dict(btType='BTMMateConnector-66',featureType='mateConnector',name=name,implicit=False,isHidden=True,
        suppressed=False,parameters=[enum('originType','ON_ENTITY','Origin type'),query('originQuery',[source]),
        query('originAdditionalQuery',[]),boolean('realign',False),query('primaryAxisQuery',[]),query('secondaryAxisQuery',[]),
        boolean('transform',True),*[quantity('translation'+k,v) for k,v in zip('XYZ',position)],
        enum('rotationType',rotation_axis,'Rotation axis'),quantity('rotation',rotation,'deg'),
        boolean('flipPrimary',False),enum('secondaryAxisType','PLUS_X','Reorient secondary axis')])

def mate(name,kind,first,second,limits=None):
    params=[enum('mateType',kind,'Mate type'),query('mateConnectorsQuery',[feature_query(first),feature_query(second)])]
    if limits:
        params.append(boolean('limitsEnabled',True))
        for pid,val in zip(['limitAxialZMin','limitAxialZMax'],limits):
            params.append(dict(btType='BTMParameterNullableQuantity-807',parameterId=pid,isInteger=False,isNull=False,expression=f'{val} deg'))
    return dict(btType='BTMMate-64',featureType='mate',name=name,suppressed=False,parameters=params)

def group_angle(group):return 0 if group=='BODY' else [45,90,135,225,270,315][int(group[1])-1]

class Installer:
    def __init__(self,client,limit=0):
        self.client=client;self.limit=limit;self.count=0
        self.snapshot=client.call('GET','/features')
        self.features={f['name']:f for f in self.snapshot['features']}
        self.states=self.snapshot['featureStates']
        self.log=[]
    def ensure(self,feature):
        name=feature['name']
        if name in self.features:
            f=self.features[name]
            state=self.states.get(f['featureId'],{})
            if state.get('featureStatus')!='OK':raise RuntimeError(f'Existing feature {name} is not OK: {state}')
            return f['featureId']
        if self.limit and self.count>=self.limit:raise StopIteration('Action limit reached; safely resumable')
        result=self.client.call('POST','/features',{'feature':feature})
        self.count+=1
        f=result['feature']; state=result['featureState'];self.features[name]=f;self.states[f['featureId']]=state
        record=dict(name=name,id=f['featureId'],status=state['featureStatus']);self.log.append(record)
        with (WORK/'install_log.jsonl').open('a') as stream:stream.write(json.dumps(record)+'\n')
        print(self.count,name,state['featureStatus'],flush=True)
        if state['featureStatus']!='OK':raise RuntimeError(f'Onshape failed to solve {name}')
        return f['featureId']
    def apply(self,stage='all'):
        # This is the one deliberately incomplete probe made during schema discovery.
        probe=self.features.get('F_BODY_02_RUGPLATFORM')
        if probe and probe['featureId']=='MwT6qk9xacQ5dpvLB' and self.states.get(probe['featureId'],{}).get('featureStatus')=='ERROR':
            self.client.call('DELETE','/features/featureid/'+urllib.parse.quote(probe['featureId'],safe=''))
            del self.features[probe['name']]
        mc={}
        for o in META:
            group=o['group'];n=o['name']
            mc[n]=self.ensure(connector('MC_'+n,origin_query(PLAN['instances'][n]['id']),LINKS[group]['origin_neutral_m'],group_angle(group)))
        if stage=='connectors':return
        for m in PLAN['fastened']:self.ensure(mate(m['name'],'FASTENED',mc[m['parent']],mc[m['child']]))
        if stage=='fastened':return
        for m in PLAN['revolute']:
            leg=int(m['name'][3]);kind=m['name'].split('_')[-1];angle=[45,90,135,225,270,315][leg-1]
            if kind=='yaw':
                parent_mc=self.ensure(connector('J_'+m['name']+'_parent',origin_query(PLAN['instances'][m['parent']]['id']),m['origin_m'],angle))
                child_mc=mc[m['child']]
            else:
                distance=[.062,0,-.00695] if kind=='hip' else [.078*math.cos(math.radians(20)),0,.078*math.sin(math.radians(20))]
                parent_mc=self.ensure(connector('J_'+m['name']+'_parent',feature_query(mc[m['parent']]),distance,90,'ABOUT_X'))
                child_mc=self.ensure(connector('J_'+m['name']+'_child',feature_query(mc[m['child']]),[0,0,0],90,'ABOUT_X'))
            self.ensure(mate(m['name'],'REVOLUTE',parent_mc,child_mc,m['range_delta_deg']))
    def verify(self):
        f=self.client.call('GET','/features');a=self.client.call('GET',query={'includeMateFeatures':'true','includeMateConnectors':'true'})
        (WORK/'features_after.json').write_text(json.dumps(f,indent=2))
        (WORK/'assembly_after.json').write_text(json.dumps(a,indent=2))
        mates=[x for x in f['features'] if x['featureType']=='mate']
        types=[next(p['value'] for p in x['parameters'] if p['parameterId']=='mateType') for x in mates]
        errors={fid:s for fid,s in f['featureStates'].items() if s.get('featureStatus')!='OK'}
        occ=a['rootAssembly']['occurrences'];base_id=PLAN['instances']['01_CHASSIS']['id']
        fixed=[o['path'] for o in occ if o.get('fixed')]
        drift=max(np.max(np.abs(np.array(o['transform']).reshape(4,4)-np.eye(4))) for o in occ)
        report=dict(fastened=types.count('FASTENED'),revolute=types.count('REVOLUTE'),errors=errors,fixed=fixed,
            max_neutral_transform_deviation=float(drift),instances=len(a['rootAssembly']['instances']),
            document_url=f"https://cad.onshape.com/documents/{PLAN['documentId']}/w/{PLAN['workspaceId']}/e/{PLAN['elementId']}")
        (OUT/'onshape_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
        assert types.count('FASTENED')==len(META)-len(LINKS) and types.count('REVOLUTE')==18
        assert len(a['rootAssembly']['instances'])==len(META)
        assert not errors and [base_id] in fixed and drift<1e-6

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--key-file',type=Path,required=True)
    p.add_argument('--limit',type=int,default=0);p.add_argument('--stage',choices=['connectors','fastened','all','verify'],default='all')
    args=p.parse_args();installer=Installer(Client(args.key_file),args.limit)
    try:
        if args.stage!='verify':installer.apply(args.stage)
        if args.stage in ['all','verify']:installer.verify()
    except StopIteration as e:print(e)
