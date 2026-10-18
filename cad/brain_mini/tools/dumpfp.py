import os
"""Dump courtyard boxes + pads of the board to JSON (run with KiCad python)."""
import json, sys, pcbnew
b = pcbnew.LoadBoard(sys.argv[1]); OX, OY = 150, 100
T = lambda v: (round(pcbnew.ToMM(v.x) - OX, 3), round(pcbnew.ToMM(v.y) - OY, 3))
out = []
for f in b.GetFootprints():
    cy = f.GetCourtyard(pcbnew.F_CrtYd)
    if cy.OutlineCount():
        bb = cy.BBox(); x0, y0 = T(bb.GetOrigin()); x1, y1 = T(bb.GetEnd())
    else:
        bb = f.GetBoundingBox(False); x0, y0 = T(bb.GetOrigin()); x1, y1 = T(bb.GetEnd())
    pads = []
    for p in f.Pads():
        bp = p.GetBoundingBox(); a = T(bp.GetOrigin()); c = T(bp.GetEnd())
        pads.append([p.GetNumber(), p.GetNetname(), a, c])
    out.append(dict(ref=f.GetReference(), box=[x0, y0, x1, y1], pos=T(f.GetPosition()), pads=pads))
json.dump(out, open(sys.argv[2], 'w'))
sys.stdout.flush(); os._exit(0)   # KiCad's SWIG objects can crash Python at shutdown
