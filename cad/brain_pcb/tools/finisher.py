"""Finish the last unrouted connections with a small 2-layer A* router (0.1 mm grid).

    KiCad python:  finisher.py dump   -> ../out/fin_dump.json  (copper, keep-outs, outline, open connections)
    venv python:   finisher.py route  -> ../out/fin_routes.json
    KiCad python:  finisher.py apply  -> adds tracks/vias, refills zones, saves the board
"""
import os, sys, json, re, math

HERE = os.path.dirname(os.path.abspath(__file__))
PRJ = os.path.dirname(HERE)
OUT = os.path.join(PRJ, 'out')
PCB = os.path.join(PRJ, 'arachne_brain.kicad_pcb')
OX, OY = 150.0, 100.0
CLR = 0.22                       # clearance used by the finisher (rules: 0.15, power 0.2) + grid snap margin


# ----------------------------------------------------------------------------- KiCad side
def dump():
    import pcbnew
    b = pcbnew.LoadBoard(PCB)
    T = lambda v: (pcbnew.ToMM(v.x) - OX, pcbnew.ToMM(v.y) - OY)
    L = {'F': pcbnew.F_Cu, 'B': pcbnew.B_Cu}

    def poly_pts(sps):
        out = []
        for i in range(sps.OutlineCount()):
            o = sps.Outline(i)
            out.append([T(o.CPoint(j)) for j in range(o.PointCount())])
        return out
    d = {'pads': [], 'tracks': [], 'vias': [], 'zones': [], 'keepouts': []}
    for f in b.GetFootprints():
        for p in f.Pads():
            for ln, l in L.items():
                if p.IsOnLayer(l):
                    d['pads'].append(dict(net=p.GetNetname(), layer=ln, poly=poly_pts(p.GetEffectivePolygon(l, pcbnew.ERROR_INSIDE)),
                                          ref=f.GetReference(), num=p.GetNumber()))
            if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                d['vias'].append(dict(net=p.GetNetname(), pos=T(p.GetPosition()), r=max(pcbnew.ToMM(p.GetDrillSize().x),
                                       pcbnew.ToMM(p.GetSize(pcbnew.F_Cu).x)) / 2))
    for t in b.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            d['vias'].append(dict(net=t.GetNetname(), pos=T(t.GetPosition()), r=pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)) / 2))
        else:
            for ln, l in L.items():
                if t.GetLayer() == l:
                    d['tracks'].append(dict(net=t.GetNetname(), layer=ln, a=T(t.GetStart()), b=T(t.GetEnd()), w=pcbnew.ToMM(t.GetWidth())))
    for z in b.Zones():
        for ln, l in L.items():
            if not z.IsOnLayer(l):
                continue
            if z.GetIsRuleArea():
                if z.GetDoNotAllowTracks():
                    o = z.Outline()
                    d['keepouts'].append(dict(layer=ln, poly=[T(o.CVertex(i)) for i in range(o.TotalVertices())]))
            elif z.GetNetname() != 'GND' or z.GetZoneName() != 'GND pour':
                d['zones'].append(dict(net=z.GetNetname(), layer=ln, poly=poly_pts(z.GetFilledPolysList(l))))
    d['outline'] = json.load(open(os.path.join(HERE, 'outline.json')))['edge']
    # open connections from the DRC report
    rpt = open(os.path.join(OUT, 'drc.rpt')).read()
    opens = []
    for blk in re.split(r'\n(?=\[)', rpt):
        if not blk.startswith('[unconnected_items]'):
            continue
        its = re.findall(r'@\(([\d.]+) mm, ([\d.]+) mm\): (.*)', blk)
        if len(its) != 2:
            continue
        ends = []
        for x, y, desc in its:
            net = re.search(r'\[([^\]]+)\]', desc).group(1)
            lay = 'FB' if ('Via' in desc or 'PTH' in desc) else ('B' if 'B.Cu' in desc else 'F')
            ends.append(dict(net=net, pos=(float(x) - OX, float(y) - OY), layers=lay, desc=desc))
        if ends[0]['net'] == ends[1]['net'] and 'Zone' not in ends[0]['desc'] + ends[1]['desc']:
            opens.append(ends)
    d['opens'] = opens
    json.dump(d, open(os.path.join(OUT, 'fin_dump.json'), 'w'))
    print('dumped; open connections:', [(o[0]['net'], o[0]['desc'][:30], o[1]['desc'][:30]) for o in opens])


def _match(t, pts, tol=0.002):
    import pcbnew
    T = lambda v: (pcbnew.ToMM(v.x) - OX, pcbnew.ToMM(v.y) - OY)
    return any(math.dist(T(t.GetStart()), a) < tol and math.dist(T(t.GetEnd()), b) < tol for a, b in pts)


def undo():
    """Remove what the last apply() added (matched by exact coordinates)."""
    import pcbnew
    b = pcbnew.LoadBoard(PCB)
    routes = json.load(open(os.path.join(OUT, 'fin_routes.json')))
    segs = [(tuple(s['a']), tuple(s['b'])) for r in routes for s in r['segs']]
    vias = [tuple(v) for r in routes for v in r['vias']]
    T = lambda v: (pcbnew.ToMM(v.x) - OX, pcbnew.ToMM(v.y) - OY)
    kill = [t for t in b.GetTracks() if (t.GetClass() == 'PCB_VIA' and any(math.dist(T(t.GetPosition()), v) < 0.002 for v in vias))
            or (t.GetClass() == 'PCB_TRACK' and _match(t, segs))]
    for t in kill:
        b.Remove(t)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones()); pcbnew.SaveBoard(PCB, b)
    print('removed', len(kill))


def drop_dangling(b, nets):
    """Delete vias of the given nets that no longer connect anything on one of their layers."""
    import pcbnew
    T = lambda v: (round(pcbnew.ToMM(v.x), 3), round(pcbnew.ToMM(v.y), 3))
    ends = {}
    for t in b.GetTracks():
        if t.GetClass() == 'PCB_TRACK' and t.GetNetname() in nets:
            for p in (t.GetStart(), t.GetEnd()):
                ends.setdefault(T(p), set()).add(t.GetLayer())
    n = 0
    for v in [t for t in b.GetTracks() if t.GetClass() == 'PCB_VIA' and t.GetNetname() in nets]:
        if len(ends.get(T(v.GetPosition()), ())) < 2 and v.GetNetname() not in ('GND', 'V12', 'V5'):
            b.Remove(v); n += 1
    # then track stubs with a loose end (repeat until stable); work on plain tuples, remove at the end
    pads = [(p, p.GetLayerSet()) for f in b.GetFootprints() for p in f.Pads() if p.GetNetname() in nets]
    zones = [z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetname() in nets]
    tr = []
    for t in b.GetTracks():
        if t.GetNetname() in nets:
            if t.GetClass() == 'PCB_VIA':
                tr.append(['V', t, T(t.GetPosition()), None, None, t.GetPosition()])
            else:
                tr.append(['T', t, T(t.GetStart()), T(t.GetEnd()), t.GetLayer(), (t.GetStart(), t.GetEnd())])
    dead = set()
    while True:
        cnt = {}
        for i, e in enumerate(tr):
            if i in dead:
                continue
            for q in ([e[2]] if e[0] == 'V' else [e[2], e[3]]):
                cnt[q] = cnt.get(q, 0) + 1
        new = []
        for i, e in enumerate(tr):
            if i in dead or e[0] != 'T':
                continue
            for q, vec in ((e[2], e[5][0]), (e[3], e[5][1])):
                if cnt.get(q, 0) < 2 and not any(pp.HitTest(vec) and ls.Contains(e[4]) for pp, ls in pads) \
                        and not any(z.IsOnLayer(e[4]) and z.HitTestFilledArea(e[4], vec) for z in zones):
                    new.append(i); break
        if not new:
            break
        dead |= set(new)
    for i in dead:
        b.Remove(tr[i][1]); n += 1
    return n


def apply():
    import pcbnew
    import build_pcb as B
    b = pcbnew.LoadBoard(PCB)
    routes = json.load(open(os.path.join(OUT, 'fin_routes.json')))
    n = 0
    for r in routes:
        for seg in r['segs']:
            B.track(b, r['net'], seg['a'], seg['b'], r['w'], pcbnew.F_Cu if seg['layer'] == 'F' else pcbnew.B_Cu); n += 1
        for v in r['vias']:
            B.via(b, r['net'], v, d=0.45, drill=0.2)
    print('dangling vias removed:', drop_dangling(b, {r['net'] for r in routes}))
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b)
    B.write_project()
    print('applied', n, 'segments')


# ----------------------------------------------------------------------------- router (numpy / shapely)
def route():
    import numpy as np, heapq
    import shapely
    from shapely.geometry import Polygon, Point, LineString
    from shapely.ops import unary_union
    d = json.load(open(os.path.join(OUT, 'fin_dump.json')))
    G = 0.1
    xs = np.arange(-70, 70.001, G); ys = np.arange(-35, 35.001, G)
    X, Y = np.meshgrid(xs, ys)
    board = Polygon(d['outline'])
    widths = {'V5': 0.4, 'V12': 0.5, 'GND': 0.3, '3V3': 0.3, 'CM_3V3': 0.3}
    results = []
    for ends in d['opens']:
        net = ends[0]['net']; w = widths.get(net, 0.15)
        inf = CLR + (0.05 if net in widths else 0) + w / 2   # inflate obstacles for the track centre line
        vinf = CLR + 0.225                                   # for a 0.45 mm via
        obs = {'F': [], 'B': []}; vobs = {'F': [], 'B': []}
        for p in d['pads']:
            if p['net'] != net:
                for ring in p['poly']:
                    if len(ring) > 2:
                        g = Polygon(ring); obs[p['layer']].append(g.buffer(inf)); vobs[p['layer']].append(g.buffer(vinf))
        for t in d['tracks']:
            if t['net'] != net:
                g = LineString([t['a'], t['b']]); obs[t['layer']].append(g.buffer(t['w'] / 2 + inf)); vobs[t['layer']].append(g.buffer(t['w'] / 2 + vinf))
        for v in d['vias']:
            if v['net'] != net:
                g = Point(v['pos']).buffer(v['r'])
                for l in 'FB':
                    obs[l].append(g.buffer(inf)); vobs[l].append(g.buffer(vinf))
        for z in d['zones']:
            if z['net'] != net:
                for ring in z['poly']:
                    if len(ring) > 2:
                        g = Polygon(ring).buffer(0); obs[z['layer']].append(g.buffer(inf)); vobs[z['layer']].append(g.buffer(vinf))
        for k in d['keepouts']:
            g = Polygon(k['poly']).buffer(0); obs[k['layer']].append(g.buffer(w / 2 + 0.08)); vobs[k['layer']].append(g.buffer(0.3))
        inside = board.buffer(-(0.3 + w / 2)); vinside = board.buffer(-(0.3 + 0.225))
        free = {}; vfree = None
        for l in 'FB':
            u = unary_union(obs[l]); free[l] = shapely.contains_xy(inside, X, Y) & ~shapely.contains_xy(u, X, Y)
            vu = unary_union(vobs[l]); vf = shapely.contains_xy(vinside, X, Y) & ~shapely.contains_xy(vu, X, Y)
            vfree = vf if vfree is None else (vfree & vf)
        H, W = X.shape
        idx = lambda p: (int(round((p[1] + 35) / G)), int(round((p[0] + 70) / G)))
        # start/goal: make a small free disc around each anchor (own copper there)
        starts, goals = [], []
        for e, lst in ((ends[0], starts), (ends[1], goals)):
            r0, c0 = idx(e['pos'])
            for l in e['layers']:
                for dr in range(-3, 4):
                    for dc in range(-3, 4):
                        if 0 <= r0 + dr < H and 0 <= c0 + dc < W:
                            free[l][r0 + dr, c0 + dc] = True
                lst.append((l, r0, c0))
        goalset = {(l, r, c) for l, r, c in goals}
        gr, gc = goals[0][1], goals[0][2]
        hfun = lambda r, c: math.hypot(r - gr, c - gc)
        openq = []; came = {}; gcost = {}
        for s in starts:
            gcost[s] = 0; heapq.heappush(openq, (hfun(s[1], s[2]), 0, s))
        nb = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        found = None; it = 0
        while openq:
            f, g, s = heapq.heappop(openq); it += 1
            if s in goalset:
                found = s; break
            if g > gcost.get(s, 1e18):
                continue
            l, r, c = s
            for dr, dc, cost in nb:
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and free[l][rr, cc]:
                    n2 = (l, rr, cc); g2 = g + cost + (0.3 if l == 'B' else 0)
                    if g2 < gcost.get(n2, 1e18):
                        gcost[n2] = g2; came[n2] = s; heapq.heappush(openq, (g2 + hfun(rr, cc), g2, n2))
            if vfree[r, c]:
                l2 = 'B' if l == 'F' else 'F'; n2 = (l2, r, c); g2 = g + 25
                if free[l2][r, c] and g2 < gcost.get(n2, 1e18):
                    gcost[n2] = g2; came[n2] = s; heapq.heappush(openq, (g2 + hfun(r, c), g2, n2))
        if not found:
            print('NO ROUTE', net, ends[0]['desc'][:40], '->', ends[1]['desc'][:40]); continue
        path = [found]
        while path[-1] in came:
            path.append(came[path[-1]])
        path.reverse()
        pt = lambda s: (round(-70 + s[2] * G, 3), round(-35 + s[1] * G, 3))

        def visible(a, c):                                  # straight line a->c stays in free cells?
            l = a[0]; n = int(max(abs(c[1] - a[1]), abs(c[2] - a[2])) * 2) + 1
            for k in range(n + 1):
                t = k / n; r = int(round(a[1] + (c[1] - a[1]) * t)); cc = int(round(a[2] + (c[2] - a[2]) * t))
                if not free[l][r, cc]:
                    return False
            return True
        segs, vias = [], []
        i = 0
        while i < len(path) - 1:
            if path[i + 1][0] != path[i][0]:
                vias.append(pt(path[i])); i += 1; continue
            j = i + 1
            for k in range(len(path) - 1, i, -1):          # farthest visible point on the same layer
                if all(p[0] == path[i][0] for p in path[i:k + 1]) and visible(path[i], path[k]):
                    j = k; break
            segs.append(dict(layer=path[i][0], a=pt(path[i]), b=pt(path[j]))); i = j
        # snap the very ends onto the real anchor points
        if segs:
            segs[0]['a'] = tuple(ends[0]['pos']) if segs[0]['layer'] in ends[0]['layers'] else segs[0]['a']
            segs[-1]['b'] = tuple(ends[1]['pos']) if segs[-1]['layer'] in ends[1]['layers'] else segs[-1]['b']
        print('routed', net, 'segments', len(segs), 'vias', len(vias), 'expanded', it)
        results.append(dict(net=net, w=w, segs=segs, vias=vias))
    json.dump(results, open(os.path.join(OUT, 'fin_routes.json'), 'w'))


if __name__ == '__main__':
    {'dump': dump, 'route': route, 'apply': apply, 'undo': undo}[sys.argv[1]]()
    sys.stdout.flush(); os._exit(0)   # KiCad's SWIG objects can crash Python at shutdown
