"""Supplement: fixed screw heads, fork insertion and battery-strap routing."""
import math, json, hashlib
import cadquery as cq
from v4_body import ANCHORS, place, box, cylinder
from check_v4_leg import read_parts
from check_v4_body import intersects, OUT

def main():
    body=cq.importers.importStep(str(OUT/'STEP_parts/01_chassis.step')).val()
    coxa=read_parts()['01_coxa']
    heads=cq.Compound.makeCompound([cylinder((x,y,18.25),(0,0,1),2,1.6)
           for x in (-29,-8.3) for y in (-10.25,10.25)])
    report={'step_sha256':hashlib.sha256((OUT/'STEP_parts/01_chassis.step').read_bytes()).hexdigest(),
        'fixed_heads_to_moving_coxa_max_overlap_mm3':0.,'fork_insertion_to_body_max_overlap_mm3':0.,
        'case_screw_access_through_coxa_mm3':0.,'strap_corridor_overlap_mm3':0.}
    for yaw in range(-60,61,5):
        moved=coxa.rotate((0,0,0),(0,0,1),yaw)
        report['fixed_heads_to_moving_coxa_max_overlap_mm3']=max(report['fixed_heads_to_moving_coxa_max_overlap_mm3'],intersects(moved,heads))
    for i in (0,1):
        for dx in range(0,61,5):
            report['fork_insertion_to_body_max_overlap_mm3']=max(report['fork_insertion_to_body_max_overlap_mm3'],intersects(body,place(coxa.translate((dx,0,0)),ANCHORS[i])))
    for x in (-29,-8.3):
        for y in (-10.25,10.25):
            tool=cylinder((x,y,18.26),(0,0,1),2.35,70)
            report['case_screw_access_through_coxa_mm3']=max(report['case_screw_access_through_coxa_mm3'],intersects(coxa,tool))
    for x in (-30,30):
        # 10 mm strap, 1 mm thick: flat beneath the reinforcing spine and
        # vertical tails through the two deck slots, with an open corner path.
        corridor=box(x,0,20,10,50,1)
        for y in (-25,25):corridor=corridor.fuse(box(x,y,25.75,10,1,12.5))
        report['strap_corridor_overlap_mm3']=max(report['strap_corridor_overlap_mm3'],intersects(body,corridor))
    report['passed']=all(v<.05 for k,v in report.items() if k.endswith('mm3'))
    (OUT/'body_mount_sequence.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:raise SystemExit(1)

if __name__=='__main__':main()
