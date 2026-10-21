"""Pad-field centre of every footprint (board mm, KiCad axes) -> ../out/pad_centres.json.
JLC places a part on the centre of its pads, KiCad on the footprint origin (pin 1 for connectors, body centre for modules)."""
import json, os, sys
import pcbnew
HERE = os.path.dirname(os.path.abspath(__file__))
b = pcbnew.LoadBoard(os.path.join(HERE, '..', 'arachne_mini.kicad_pcb'))
out = {}
for f in b.GetFootprints():
    pads = [p for p in f.Pads() if p.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH]
    if not pads:
        continue
    bbs = [p.GetBoundingBox() for p in pads]                    # pad extents (big tabs count with their real size)
    x0 = min(pcbnew.ToMM(q.GetLeft()) for q in bbs); x1 = max(pcbnew.ToMM(q.GetRight()) for q in bbs)
    y0 = min(pcbnew.ToMM(q.GetTop()) for q in bbs); y1 = max(pcbnew.ToMM(q.GetBottom()) for q in bbs)
    o = f.GetPosition()
    out[f.GetReference()] = [(x0 + x1) / 2, (y0 + y1) / 2, pcbnew.ToMM(o.x), pcbnew.ToMM(o.y)]
json.dump(out, open(os.path.join(HERE, '..', 'out', 'pad_centres.json'), 'w'), indent=0)
print('pad centres', len(out)); sys.stdout.flush(); os._exit(0)
