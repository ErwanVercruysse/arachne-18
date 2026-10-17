"""Autoroute the board: export Specctra DSN -> Freerouting (headless) -> import SES -> GND pours -> fill.

    route.py export   writes ../out/board.dsn
    route.py import   reads  ../out/board.ses, adds GND pours, fills zones, saves the board
"""
import os, sys, subprocess, math
import pcbnew
import build_pcb as B

PRJ = B.PRJ
PCB = os.path.join(PRJ, B.NAME + '.kicad_pcb')
OUT = os.path.join(PRJ, 'out')
DSN, SES = os.path.join(OUT, 'board.dsn'), os.path.join(OUT, 'board.ses')


def export():
    b = pcbnew.LoadBoard(PCB)
    assert pcbnew.ExportSpecctraDSN(b, DSN), 'DSN export failed'
    print('dsn ok')


def add_gnd_pours(b):
    inner = B.OUTLINE['inner']
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(b); z.SetLayer(layer); z.SetNet(b.FindNet('GND'))
        ol = z.Outline(); ol.NewOutline()
        for p in inner:
            ol.Append(B.MM(B.OX + p[0]), B.MM(B.OY + p[1]))
        z.SetAssignedPriority(0); z.SetZoneName('GND pour')
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL); z.SetLocalClearance(B.MM(0.25))
        z.SetMinThickness(B.MM(0.2)); z.SetThermalReliefGap(B.MM(0.3)); z.SetThermalReliefSpokeWidth(B.MM(0.4))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        b.Add(z)


def stitch_gnd(b):
    """GND vias on a grid wherever there is room on all layers (ties the top/bottom pours to the plane)."""
    import itertools
    pads = []
    for f in b.GetFootprints():
        for p in f.Pads():
            bb = p.GetBoundingBox(); pads.append((bb.GetX(), bb.GetY(), bb.GetRight(), bb.GetBottom()))
    tracks = [(t.GetBoundingBox().GetX(), t.GetBoundingBox().GetY(), t.GetBoundingBox().GetRight(), t.GetBoundingBox().GetBottom())
              for t in b.GetTracks()]
    keep = []
    for z in b.Zones():
        if z.GetIsRuleArea() or (z.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu) and z.GetNetname() != 'GND'):
            bb = z.GetBoundingBox(); keep.append((bb.GetX(), bb.GetY(), bb.GetRight(), bb.GetBottom()))
    cy = []
    for f in b.GetFootprints():
        c = f.GetCourtyard(pcbnew.F_CrtYd)
        if c.OutlineCount():
            bb = c.BBox(); cy.append((bb.GetX(), bb.GetY(), bb.GetRight(), bb.GetBottom()))
    m = B.MM(0.75); n = 0
    poly = B.OUTLINE['edge']

    def inside(px, py, margin=1.2):
        c = False
        for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
            if (y1 > py) != (y2 > py) and px < (x2 - x1) * (py - y1) / (y2 - y1) + x1:
                c = not c
            # distance to the segment
            dx, dy = x2 - x1, y2 - y1; L = dx * dx + dy * dy or 1e-9
            t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / L))
            if (x1 + t * dx - px) ** 2 + (y1 + t * dy - py) ** 2 < margin ** 2:
                return False
        return c
    for gx, gy in itertools.product(range(-66, 67, 4), range(-33, 34, 4)):
        x, y = B.MM(B.OX + gx), B.MM(B.OY + gy)
        if not inside(gx, gy):
            continue

        def hit(lst, mm):
            return any(a - mm < x < c + mm and bb - mm < y < d + mm for a, bb, c, d in lst)
        if hit(pads, m) or hit(tracks, m) or hit(keep, B.MM(0.6)) or hit(cy, 0):
            continue
        B.via(b, 'GND', (gx, gy), d=0.6, drill=0.3); n += 1
    print('stitch vias', n)


def import_raw():
    """SES -> board, nothing else (so a second Freerouting pass can fill the last gaps)."""
    b = pcbnew.LoadBoard(PCB)
    assert pcbnew.ImportSpecctraSES(b, SES), 'SES import failed'
    pcbnew.SaveBoard(PCB, b)
    B.write_project()
    fix_vias(refill=False)
    print('raw import ok')


def island_vias(b):
    """Every top/bottom GND pour island gets at least one via to the GND plane."""
    gnd = [t for t in b.GetTracks() if t.GetClass() == 'PCB_VIA' and t.GetNetname() == 'GND']
    tht = [p for f in b.GetFootprints() for p in f.Pads() if p.GetNetname() == 'GND' and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
    n = 0
    for z in list(b.Zones()):
        if z.GetZoneName() != 'GND pour':
            continue
        l = z.GetLayer(); other = pcbnew.B_Cu if l == pcbnew.F_Cu else pcbnew.F_Cu
        fp = z.GetFilledPolysList(l)
        for i in range(fp.OutlineCount()):
            ol = fp.Outline(i)
            if any(ol.PointInside(v.GetPosition()) for v in gnd) or any(ol.PointInside(p.GetPosition()) for p in tht):
                continue
            bb = ol.BBox(); step = B.MM(0.25); done = False
            y = bb.GetY()
            while y < bb.GetBottom() and not done:
                x = bb.GetX()
                while x < bb.GetRight() and not done:
                    c = pcbnew.VECTOR2I(x, y)
                    ring = [pcbnew.VECTOR2I(int(x + B.MM(0.5) * math.cos(a)), int(y + B.MM(0.5) * math.sin(a))) for a in [k * math.pi / 4 for k in range(8)]]
                    if ol.PointInside(c) and all(ol.PointInside(q) for q in ring):
                        # opposite outer layer must be free of other-net copper here
                        clash = any(t.GetNetname() != 'GND' and t.IsOnLayer(other) and t.HitTest(c, B.MM(0.5)) for t in b.GetTracks()) or \
                            any(p.GetNetname() != 'GND' and p.IsOnLayer(other) and p.HitTest(c, B.MM(0.5)) for f in b.GetFootprints() for p in f.Pads()) or \
                            any(zz.GetNetname() != 'GND' and not zz.GetIsRuleArea() and zz.IsOnLayer(other) and zz.HitTestFilledArea(other, c, B.MM(0.5)) for zz in b.Zones())
                        if not clash:
                            B.via(b, 'GND', ((x / 1e6) - B.OX, (y / 1e6) - B.OY), d=0.45, drill=0.2)
                            gnd.append([t for t in b.GetTracks()][-1]); n += 1; done = True
                    x += step
                y += step
    return n


def do_import():
    import math as _m
    b = pcbnew.LoadBoard(PCB)
    stitch_gnd(b)
    add_gnd_pours(b)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    print('island vias:', island_vias(b))
    pcbnew.SaveBoard(PCB, b)
    fix_vias()
    print('imported + filled')


def fix_vias(refill=True):
    """Freerouting's 0.45 mm vias inherit a 0.3 mm drill from the net class: shrink the drill (ring 0.125 mm). Refill + save."""
    b = pcbnew.LoadBoard(PCB); n = 0
    for t in b.GetTracks():
        if t.GetClass() == 'PCB_VIA' and t.GetWidth(pcbnew.F_Cu) < B.MM(0.55) and t.GetDrill() > B.MM(0.2):
            t.SetDrill(B.MM(0.2)); n += 1
    if refill:
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b)
    B.write_project()
    print('vias fixed:', n)


if __name__ == '__main__':
    {'export': export, 'import_raw': import_raw, 'finish': do_import, 'fixvias': fix_vias}[sys.argv[1]]()
    sys.stdout.flush(); os._exit(0)   # KiCad's SWIG objects can crash Python at shutdown
