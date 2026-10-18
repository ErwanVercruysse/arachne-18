import sys
"""Place the ARACHNE brain board in pcbnew: outline, footprints, nets, planes, keep-outs, artwork.
Routing is done afterwards by route.py (Freerouting)."""
import os, json, math, uuid, shutil
import pcbnew
import design, libs

PRJ = libs.PRJ
NAME = 'arachne_mini'
OX, OY = 150.0, 100.0          # board centre on the KiCad page
MM = pcbnew.FromMM
W, H, RC = design.BOARD_W, design.BOARD_H, design.CORNER_R
X0, Y0, X1, Y1 = -W / 2, -H / 2, W / 2, H / 2


def V(x, y):
    return pcbnew.VECTOR2I(MM(OX + x), MM(OY + y))


def sym_uuid(ref, unit=1):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f'arachne-mini/{ref}/{unit}'))


ROOT_UUID = str(uuid.uuid5(uuid.NAMESPACE_URL, 'arachne-mini/root'))
OUTLINE = {k: [tuple(p) for p in v[:-1]] for k, v in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'outline.json'))).items()}


# ---------------------------------------------------------------- project file
def write_project():
    tpl = json.load(open(os.path.expanduser('~/Documents/GitHub/tiltbox-kicad/tiltbox.kicad_pro')))
    tpl['meta']['filename'] = NAME + '.kicad_pro'
    tpl['sheets'] = [[ROOT_UUID, 'Root']]
    tpl['boards'] = []
    ds = tpl['board']['design_settings']
    ds['rules'].update({'min_clearance': 0.127, 'min_track_width': 0.1, 'min_via_diameter': 0.45,
                        'min_through_hole_diameter': 0.2, 'min_via_annular_width': 0.1,
                        'min_copper_edge_clearance': 0.3, 'min_hole_clearance': 0.2, 'min_hole_to_hole': 0.25,
                        'min_silk_clearance': 0.0})
    ds['track_widths'] = [0.0, 0.15, 0.25, 0.5, 1.0, 1.5]
    ds['via_dimensions'] = [{'diameter': 0.0, 'drill': 0.0}, {'diameter': 0.45, 'drill': 0.2},
                            {'diameter': 0.6, 'drill': 0.3}, {'diameter': 0.8, 'drill': 0.4}]
    sev = ds.setdefault('rule_severities', {})
    sev.update({'silk_overlap': 'ignore', 'silk_over_copper': 'ignore', 'silk_edge_clearance': 'ignore',
                'lib_footprint_mismatch': 'ignore', 'lib_footprint_issues': 'ignore', 'text_height': 'ignore',
                'footprint_symbol_mismatch': 'warning'})
    base = dict(tpl['net_settings']['classes'][0])
    base.update({'name': 'Default', 'track_width': 0.15, 'clearance': 0.15, 'via_diameter': 0.45, 'via_drill': 0.2})
    classes = [base]
    pats = []
    for i, (cname, (nets, tw, cl)) in enumerate(design.NETCLASS.items()):
        c = dict(base); c.update({'name': cname, 'track_width': tw, 'clearance': cl, 'priority': i,
                                 'via_diameter': 0.6 if tw >= 0.5 else 0.45, 'via_drill': 0.3 if tw >= 0.5 else 0.2})
        classes.append(c)
        pats += [{'netclass': cname, 'pattern': n} for n in nets]
    tpl['net_settings']['classes'] = classes
    tpl['net_settings']['netclass_patterns'] = pats
    tpl['net_settings']['netclass_assignments'] = None
    json.dump(tpl, open(os.path.join(PRJ, NAME + '.kicad_pro'), 'w'), indent=2)


# ---------------------------------------------------------------- helpers
def shape_line(board, layer, a, b, w=0.15):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(V(*a)); s.SetEnd(V(*b)); s.SetLayer(layer); s.SetWidth(MM(w)); board.Add(s); return s


def shape_circle(board, layer, c, r, w=0.15, fill=False):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_CIRCLE)
    s.SetCenter(V(*c)); s.SetEnd(V(c[0] + r, c[1])); s.SetLayer(layer); s.SetWidth(MM(w))
    s.SetFilled(fill); board.Add(s); return s


def shape_poly(board, layer, pts, w=0.0, fill=True):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_POLY)
    s.SetPolyPoints([V(*p) for p in pts]); s.SetLayer(layer); s.SetWidth(MM(w)); s.SetFilled(fill)
    board.Add(s); return s


def text(board, layer, s, at, size=1.0, th=0.15, rot=0, mirror=False, bold=False):
    t = pcbnew.PCB_TEXT(board); t.SetText(s); t.SetPosition(V(*at)); t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(MM(size), MM(size))); t.SetTextThickness(MM(th)); t.SetTextAngleDegrees(rot)
    t.SetMirrored(mirror); t.SetBold(bold); board.Add(t); return t


def rounded_rect_pts(x0, y0, x1, y1, r, n=10):
    pts = []
    for cx, cy, a0 in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n); pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def zone(board, net, layer, pts, prio=0, name='', connect='thermal', clearance=0.25, minw=0.2):
    z = pcbnew.ZONE(board); z.SetLayer(layer)
    if net:
        z.SetNet(board.FindNet(net))
    ol = z.Outline(); ol.NewOutline()
    for p in pts:
        ol.Append(MM(OX + p[0]), MM(OY + p[1]))
    z.SetAssignedPriority(prio); z.SetZoneName(name)
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if connect == 'full' else pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetLocalClearance(MM(clearance)); z.SetMinThickness(MM(minw))
    z.SetThermalReliefGap(MM(0.3)); z.SetThermalReliefSpokeWidth(MM(0.5))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    board.Add(z); return z


def keepout(board, pts, layers, tracks=True, vias=True, pours=True, pads=False, fps=False, name=''):
    z = pcbnew.ZONE(board); z.SetIsRuleArea(True)
    ls = pcbnew.LSET()
    for l in layers:
        ls.AddLayer(l)
    z.SetLayerSet(ls)
    ol = z.Outline(); ol.NewOutline()
    for p in pts:
        ol.Append(MM(OX + p[0]), MM(OY + p[1]))
    z.SetDoNotAllowTracks(tracks); z.SetDoNotAllowVias(vias); z.SetDoNotAllowZoneFills(pours)
    z.SetDoNotAllowPads(pads); z.SetDoNotAllowFootprints(fps); z.SetZoneName(name)
    board.Add(z); return z


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def via(board, net, at, d=0.6, drill=0.3):
    v = pcbnew.PCB_VIA(board); v.SetPosition(V(*at)); v.SetDrill(MM(drill)); v.SetWidth(MM(d))
    v.SetNet(board.FindNet(net)); v.SetViaType(pcbnew.VIATYPE_THROUGH); board.Add(v); return v


def track(board, net, a, b, w, layer=pcbnew.F_Cu):
    t = pcbnew.PCB_TRACK(board); t.SetStart(V(*a)); t.SetEnd(V(*b)); t.SetWidth(MM(w)); t.SetLayer(layer)
    t.SetNet(board.FindNet(net)); board.Add(t); return t


def pad_pos(fp, num):
    out = []
    for p in fp.Pads():
        if p.GetNumber() == num:
            q = p.GetPosition(); out.append((pcbnew.ToMM(q.x) - OX, pcbnew.ToMM(q.y) - OY))
    return out


# ---------------------------------------------------------------- build
def build():
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(4)
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(MM(1.6))
    board.SetLayerName(pcbnew.In1_Cu, 'In1.Cu'); board.SetLayerName(pcbnew.In2_Cu, 'In2.Cu')
    # inner layers are solid planes (GND / 12 V): mark them 'power' so the autorouter keeps off them
    board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER); board.SetLayerType(pcbnew.In2_Cu, pcbnew.LT_POWER)

    for n in design.nets():
        board.Add(pcbnew.NETINFO_ITEM(board, n))

    fpdir = os.path.join(PRJ, 'arachne.pretty')
    fps = {}
    for p in design.parts:
        name = libs.fp_name(p['fp'])
        fp = pcbnew.FootprintLoad(fpdir, name)
        assert fp is not None, name
        fp.SetFPID(pcbnew.LIB_ID('arachne', name))
        fp.SetReference(p['ref']); fp.SetValue(p['value'])
        x, y, rot = p['at']
        fp.SetPosition(V(x, y)); fp.SetOrientationDegrees(rot)
        fp.SetPath(pcbnew.KIID_PATH('/' + sym_uuid(p['ref'], 1)))
        fp.SetProperties if False else None
        for prop, val in (('MPN', p['mpn']),):
            if val:
                f = pcbnew.PCB_FIELD(fp, pcbnew.FIELD_T_USER, prop) if hasattr(pcbnew, 'FIELD_T_USER') else None
                if f is not None:
                    f.SetText(val); f.SetVisible(False); f.SetLayer(pcbnew.F_Fab); fp.Add(f)
        board.Add(fp)
        for pad in fp.Pads():
            net = p['pins'].get(pad.GetNumber())
            if net:
                pad.SetNet(board.FindNet(net))
        fps[p['ref']] = fp
        # small, tidy reference text
        r = fp.Reference(); r.SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8))); r.SetTextThickness(MM(0.12))
    for i in range(1, 5):
        fps[f'H{i}'].Reference().SetVisible(False)

    # ---------------- outline: rounded rectangle
    edge = OUTLINE['edge']
    for a, b in zip(edge, edge[1:] + edge[:1]):
        if a != b:
            shape_line(board, pcbnew.Edge_Cuts, a, b, 0.1)
    inner = OUTLINE['inner']
    allcu = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]

    # ---------------- keep-outs: the ESP32 antenna hangs over the top edge; keep copper clear of the edge there
    ex = design.ESP[0]
    keepout(board, rect(ex - 10.5, Y0 - 1, ex + 10.5, Y0 + 1.0), allcu, name='ESP antenne-rand')

    # block offsets (the buck and the 12 V entry keep the layout of the big board)
    bx, by = design.BUCK
    ex_, ey_ = design.ENTRY
    sb = lambda pts: [(x + bx, y + by) for x, y in pts]
    se = lambda pts: [(x + ex_, y + ey_) for x, y in pts]
    rb = lambda x0, y0, x1, y1: sb(rect(x0, y0, x1, y1))
    re_ = lambda x0, y0, x1, y1: se(rect(x0, y0, x1, y1))

    # ---------------- planes
    zone(board, 'GND', pcbnew.In1_Cu, inner, prio=0, name='GND plane', connect='full')
    zone(board, 'V12', pcbnew.In2_Cu, inner, prio=0, name='12V servo plane', connect='full')
    # 5 V on the bottom layer: buck output caps -> '5V PI' pads (3 A for the Raspberry Pi 4)
    v5 = [(-31.6, -29.0), (7.2, -29.0), (7.2, -22.6), (-22.4, -22.6), (-22.4, -17.6), (-31.6, -17.6)]
    zone(board, 'V5', pcbnew.B_Cu, v5, prio=2, name='5V naar Pi (bottom)', connect='full')

    # local top-layer copper for the big currents
    zone(board, 'VIN_RAW', pcbnew.F_Cu, re_(-39.6, 26.4, -25.2, 34.3), prio=3, name='VIN_RAW', connect='full')
    zone(board, 'V12', pcbnew.F_Cu, re_(-46.0, 8.6, -20.9, 13.4), prio=2, name='12V caps', connect='full')
    zone(board, 'V12', pcbnew.F_Cu, re_(-24.3, 22.4, -20.9, 28.4), prio=2, name='12V Q1 source', connect='full')
    zone(board, 'V12', pcbnew.F_Cu, rb(-39.8, -20.4, -36.6, -12.9), prio=2, name='12V buck in', connect='full')
    zone(board, 'SW', pcbnew.F_Cu, sb([(-33.0, -29.5), (-28.6, -29.5), (-28.6, -18.9), (-21.2, -18.9), (-21.2, -14.6),
                                       (-28.6, -14.6), (-28.6, -14.55), (-33.0, -14.55)]), prio=2, name='SW', connect='full')
    zone(board, 'V5', pcbnew.F_Cu, rb(-43.9, -32.0, -37.6, -21.3), prio=2, name='5V out', connect='full')
    track(board, 'V12', (-36.8 + bx, -13.64 + by), (-35.56 + bx, -13.64 + by), 0.6)

    # ---------------- stitching vias for the high-current nets
    for x in (-45.2, -43.0, -38.6, -36.4, -34.2, -29.0, -26.8, -21.6):
        via(board, 'V12', (x + ex_, 9.4 + ey_))
    for x, y in ((-23.6, 23.2), (-21.6, 23.2), (-23.6, 24.6), (-21.6, 24.6), (-23.6, 26.0)):
        via(board, 'V12', (x + ex_, y + ey_))
    for y in (-19.0, -17.2, -15.4):
        via(board, 'V12', (-37.3 + bx, y + by))
    for x in (-42.8, -40.6, -38.4):                     # 5 V out -> bottom 5 V band
        via(board, 'V5', (x + bx, -22.1 + by))
    via(board, 'V5', (-38.2 + bx, -30.8 + by))
    for x, y in ((-47.2, -30.5), (-43.3, -13.6), (-43.3, -16.4), (-43.3, -19.4), (-23.0, -27.0), (-21.4, -27.0)):
        via(board, 'GND', (x + bx, y + by))
    for x, y in ((-45.0, 20.6), (-38.0, 20.6), (-35.0, 20.6), (-28.0, 20.6), (-21.3, 19.6), (-25.8, 4.8), (-40.5, 25.0)):
        via(board, 'GND', (x + ex_, y + ey_))

    # the autorouter must not cut the bottom 5 V band or the local power copper
    # only the long band is protected; the lower end (x < -22.4) stays open so the router can tap 5 V there
    keepout(board, rect(-22.4, -29.0, 7.2, -22.6), [pcbnew.B_Cu], tracks=True, vias=False, pours=False, name='5V band: geen sporen')
    for nm, pts in (('VIN_RAW', re_(-39.2, 26.8, -25.6, 33.9)), ('12V caps', re_(-45.6, 9.0, -21.3, 13.0)),
                    ('12V Q1', re_(-23.9, 22.8, -21.3, 28.0)), ('12V buck in', rb(-39.4, -20.0, -37.4, -13.3)),
                    ('SW', sb([(-32.6, -29.1), (-29.0, -29.1), (-29.0, -18.5), (-21.6, -18.5), (-21.6, -15.0), (-32.6, -15.0)])),
                    ('5V out', rb(-43.5, -31.6, -38.0, -21.7))):
        keepout(board, pts, [pcbnew.F_Cu], tracks=True, vias=False, pours=False, name=nm + ': geen vreemde sporen')

    fanout(board, {'GND', 'V12', 'V5'}, {'V5': v5})

    # ---------------- artwork
    F, B = pcbnew.F_SilkS, pcbnew.B_SilkS
    text(board, F, 'ARACHNE-18', (2.6, -16.5), size=1.5, th=0.26, bold=True)
    text(board, F, 'MINI', (2.6, -14.2), size=1.5, th=0.26, bold=True)
    text(board, F, 'ESP32-S3  v1.0', (2.6, -12.2), size=0.8, th=0.14)
    text(board, F, 'USB > PI 4', (9.5, 26.5), size=0.7)
    text(board, F, 'RST', (24.6, -9.6), size=0.6, )
    text(board, F, 'BOOT', (24.6, 1.3), size=0.6)
    text(board, F, 'IMU', (design.IM[0], design.IM[1] + 4.3), size=0.7)
    # back side: big name + spider
    back_art(board)
    silk_cleanup(board)
    board.SetFileName(os.path.join(PRJ, NAME + '.kicad_pcb'))
    pcbnew.SaveBoard(os.path.join(PRJ, NAME + '.kicad_pcb'), board)
    write_project()
    return board


# ---------------------------------------------------------------- plane fan-out
def _pt_in_poly(px, py, poly):
    c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > py) != (y2 > py) and px < (x2 - x1) * (py - y1) / (y2 - y1) + x1:
            c = not c
    return c


def _dist_seg(px, py, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; L = dx * dx + dy * dy or 1e-12
    t = max(0, min(1, ((px - a[0]) * dx + (py - a[1]) * dy) / L))
    return math.hypot(a[0] + t * dx - px, a[1] + t * dy - py)


def _edge_dist(px, py):
    e = OUTLINE['edge']
    return min(_dist_seg(px, py, a, b) for a, b in zip(e, e[1:] + e[:1]))


def _rect_seg_hit(r, a, b, m):
    """segment a-b within m of rectangle r=(x0,y0,x1,y1)?"""
    x0, y0, x1, y1 = r[0] - m, r[1] - m, r[2] + m, r[3] + m
    for i in range(21):
        t = i / 20; x = a[0] + (b[0] - a[0]) * t; y = a[1] + (b[1] - a[1]) * t
        if x0 < x < x1 and y0 < y < y1:
            return True
    return False


def fanout(board, plane_nets, regions):
    """Drop a via (+ short track) next to every SMD pad of a plane net that is not already on its own copper."""
    pads = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            bb = p.GetBoundingBox()
            r = (pcbnew.ToMM(bb.GetX()) - OX, pcbnew.ToMM(bb.GetY()) - OY, pcbnew.ToMM(bb.GetRight()) - OX, pcbnew.ToMM(bb.GetBottom()) - OY)
            pads.append((p, r, p.GetNetname(), p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD))
    vias = [(pcbnew.ToMM(v.GetPosition().x) - OX, pcbnew.ToMM(v.GetPosition().y) - OY, pcbnew.ToMM(v.GetWidth(pcbnew.F_Cu)) / 2, v.GetNetname())
            for v in board.GetTracks() if v.GetClass() == 'PCB_VIA']
    blocked = []                         # (poly, allowed_net)
    for z in board.Zones():
        if z.GetIsRuleArea() and z.GetDoNotAllowVias():
            blocked.append(([(pcbnew.ToMM(z.Outline().CVertex(i).x) - OX, pcbnew.ToMM(z.Outline().CVertex(i).y) - OY)
                             for i in range(z.Outline().TotalVertices())], None))
        elif not z.GetIsRuleArea() and z.GetLayer() == pcbnew.F_Cu:
            blocked.append(([(pcbnew.ToMM(z.Outline().CVertex(i).x) - OX, pcbnew.ToMM(z.Outline().CVertex(i).y) - OY)
                             for i in range(z.Outline().TotalVertices())], z.GetNetname()))
    fps_under = []                       # no vias under the CM5 connectors themselves
    n_ok, n_fail = 0, []
    for p, r, net, smd in pads:
        if net not in plane_nets or not smd or not p.IsOnLayer(pcbnew.F_Cu):
            continue
        cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
        if net in regions and not _pt_in_poly(cx, cy, regions[net]):
            continue
        if any(_pt_in_poly(cx, cy, poly) for poly, an in blocked if an == net):
            continue                     # pad sits on its own copper zone already
        ref = p.GetParentFootprint().GetReference()
        if (ref, p.GetNumber()) in (('U1', '2'),):
            continue
        big = net in ('V12', 'V5')
        vd, vdr, tw = (0.6, 0.3, 0.6) if big else (0.45, 0.2, 0.3)
        hw, hh = (r[2] - r[0]) / 2, (r[3] - r[1]) / 2
        cands = []
        for k in range(16):
            a = 2 * math.pi * k / 16
            for extra in (0.45, 0.75, 1.1, 1.6, 2.2):
                d = (abs(math.cos(a)) * hw + abs(math.sin(a)) * hh) + vd / 2 + extra
                cands.append((extra, k, cx + d * math.cos(a), cy + d * math.sin(a)))
        cands.sort()
        done = False
        for extra, k, vx, vy in cands:
            if _edge_dist(vx, vy) < vd / 2 + 0.5:
                continue
            if any(_pt_in_poly(vx, vy, poly) and an != net for poly, an in blocked):
                continue
            if any(math.hypot(vx - x, vy - y) < rr + vd / 2 + 0.2 for x, y, rr, nn in vias):
                continue
            bad = False
            for q, rq, nq, sq in pads:
                if q is p:
                    continue
                if nq == net and not big:
                    m = 0.05
                else:
                    m = 0.2
                # via body
                if rq[0] - m - vd / 2 < vx < rq[2] + m + vd / 2 and rq[1] - m - vd / 2 < vy < rq[3] + m + vd / 2:
                    bad = True; break
                if nq != net and _rect_seg_hit(rq, (cx, cy), (vx, vy), 0.18 + tw / 2):
                    bad = True; break
            if bad:
                continue
            track(board, net, (cx, cy), (vx, vy), tw)
            via(board, net, (vx, vy), d=vd, drill=vdr)
            vias.append((vx, vy, vd / 2, net)); n_ok += 1; done = True
            break
        if not done:
            n_fail.append((ref, p.GetNumber(), net))
    print('fan-out vias:', n_ok, 'failed:', n_fail)


def silk_cleanup(board):
    """Clean silkscreen: only connectors and buttons keep a visible reference (JLC places the rest);
    every leg connector gets a big L1..L6 label on its outer side."""
    import design
    legs = {p['ref']: p['value'].split()[0] for p in design.parts if p['value'].endswith('servo')}
    old = [t for t in board.Drawings() if t.GetClass() == 'PCB_TEXT' and t.GetText().startswith('BUS ')]
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        fp.Reference().SetVisible(ref[:1] == 'J' and ref not in legs)
        if ref in legs:
            x = pcbnew.ToMM(fp.GetPosition().x) - OX; y = pcbnew.ToMM(fp.GetPosition().y) - OY
            sx = 1 if x > 0 else -1
            cy = y - 2.5 if sx > 0 else y + 2.5
            text(board, pcbnew.F_SilkS, legs[ref], (x + sx * 4.1, cy), size=1.2, th=0.22, rot=90 * sx, bold=True)
    for t in old:                     # removing last: KiCad's SWIG proxies go stale after a Remove()
        board.Remove(t)


def back_art(board):
    """Name + spider on the bottom silkscreen."""
    B = pcbnew.B_SilkS
    text(board, B, 'ARACHNE-18', (17.0, 4.0), size=3.0, th=0.45, mirror=True, bold=True)
    text(board, B, 'mini brain  ·  ESP32-S3 + MPU-6050  ·  USB > Pi 4', (17.0, 8.0), size=1.0, th=0.16, mirror=True)
    text(board, B, 'Kobe Van Robaeys  ·  maakleerplek  ·  2026', (17.0, 10.5), size=0.9, th=0.15, mirror=True)
    spider(board, B, (17.0, -10.5), 0.5)


def spider(board, layer, c, s):
    cx, cy = c
    # body (thorax + abdomen) as filled polygons, 6 legs as polylines (mirrored left/right)
    def ell(x, y, rx, ry, n=36):
        return [(x + rx * math.cos(2 * math.pi * i / n), y + ry * math.sin(2 * math.pi * i / n)) for i in range(n)]
    shape_poly(board, layer, ell(cx, cy, 5 * s, 6.5 * s))
    shape_poly(board, layer, ell(cx, cy - 8.5 * s, 3.2 * s, 3.0 * s))
    legs = [((3.5, -4), (12, -9), (20, -5)), ((4.5, 0), (14, -1), (23, 4)), ((3.5, 4), (12, 9), (20, 15))]
    for sx in (1, -1):
        for a, b, d in legs:
            pts = [(cx + sx * a[0] * s, cy + a[1] * s), (cx + sx * b[0] * s, cy + b[1] * s), (cx + sx * d[0] * s, cy + d[1] * s)]
            shape_line(board, layer, pts[0], pts[1], 1.0 * s); shape_line(board, layer, pts[1], pts[2], 0.7 * s)


if __name__ == '__main__':
    build()
    print('board written')
    sys.stdout.flush(); os._exit(0)   # KiCad's SWIG objects can crash Python at shutdown
