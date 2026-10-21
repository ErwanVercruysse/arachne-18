import sys
"""Generate the KiCad schematic (labels on every pin, grouped in titled blocks) from design.py.
Footprint links use the same UUIDs as build_pcb.py, so 'Update PCB from schematic' is clean."""
import os, uuid, copy, math
import design, libs, symlib
from sexp import parse, dump, find, find1, Sym
from build_pcb import sym_uuid, ROOT_UUID, NAME, PRJ

GRID = 1.27
S = Sym


def snap(v, g=2.54):
    return round(v / g) * g


def uid(*k):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, 'arachne-brain-sch/' + '/'.join(map(str, k))))


def font(size=1.27, bold=False):
    f = [S('font'), [S('size'), size, size]]
    if bold:
        f.append([S('bold'), S('yes')])
    return f


LIBSYMS = {s[1]: s for s in find(parse(open(os.path.join(PRJ, 'arachne.kicad_sym')).read()), 'symbol')}
PWR = symlib.get('power:PWR_FLAG')


def sym_geom(sym):
    """pins [(unit, num, name, x, y, ang)] and bbox (in symbol coords, y up) per unit."""
    pins = [(u, n, nm, x, y, a) for (u, n, nm, t, x, y, a, l) in symlib.pins(sym)]
    pts = {}

    def walk(node, unit):
        for e in node:
            if isinstance(e, list) and e:
                if e[0] in ('start', 'end', 'xy', 'center') and len(e) >= 3 and not isinstance(e[1], list):
                    pts.setdefault(unit, []).append((float(e[1]), float(e[2])))
                walk(e, unit)
    for sub in find(sym, 'symbol'):
        u = int(sub[1].rsplit('_', 2)[-2])
        walk([e for e in sub if not (isinstance(e, list) and e[0] == 'pin')], u)
    return pins, pts


def bbox(pins, pts, unit):
    xs, ys = [], []
    for u, n, nm, x, y, a in pins:
        if u in (0, unit):
            xs.append(x); ys.append(y)
    for u in (0, unit):
        for x, y in pts.get(u, []):
            xs.append(x); ys.append(y)
    return min(xs), min(ys), max(xs), max(ys)


def glabel(net, x, y, ang, u):
    """Global label: net names in the netlist match the PCB exactly (no '/' sheet prefix)."""
    just = S('right') if ang in (180, 270) else S('left')
    return [S('global_label'), net, [S('shape'), S('passive')], [S('at'), round(x, 3), round(y, 3), ang],
            [S('fields_autoplaced'), S('yes')], [S('effects'), font(1.0), [S('justify'), just]], [S('uuid'), u],
            [S('property'), 'Intersheetrefs', '${INTERSHEET_REFS}', [S('at'), round(x, 3), round(y, 3), 0],
             [S('effects'), font(1.0), [S('hide'), S('yes')]]]]


def label_len(net):
    return 1.0 * len(net) + 3


def build():
    items, libsyms_used = [], {}
    # ---------------------------------------------------------------- layout: groups -> blocks
    groups = []
    for p in design.parts:
        if p['group'] not in [g[0] for g in groups]:
            groups.append((p['group'], []))
        units = [1, 2] if p['ref'] == 'M1' else [1]
        for u in units:
            dict(groups)[p['group']].append((p, u))
    blocks = []
    for gname, members in groups:
        cells = []
        for p, u in members:
            sid = libs.sym_name(p['sym'])
            sym = LIBSYMS[sid]
            pins, pts = sym_geom(sym)
            x0, y0, x1, y1 = bbox(pins, pts, u)
            left = max([label_len(p['pins'].get(n, '')) for (uu, n, nm, x, y, a) in pins if uu in (0, u) and a == 0] or [0])
            right = max([label_len(p['pins'].get(n, '')) for (uu, n, nm, x, y, a) in pins if uu in (0, u) and a == 180] or [0])
            vert = 6 if any(a in (90, 270) for (uu, n, nm, x, y, a) in pins if uu in (0, u)) else 0
            w = (x1 - x0) + left + right + 8
            h = (y1 - y0) + 2 * vert + 9
            cells.append(dict(p=p, u=u, sym=sym, sid=sid, pins=pins, box=(x0, y0, x1, y1), left=left, vert=vert, w=w, h=h))
        maxw = 175 if gname != 'Raspberry Pi CM5' else 330
        # shelf-pack the cells
        x = y = rowh = 0; bw = 0
        for c in cells:
            if x + c['w'] > maxw and x > 0:
                x = 0; y += rowh; rowh = 0
            c['cx'], c['cy'] = x, y
            x += c['w']; rowh = max(rowh, c['h']); bw = max(bw, x)
        blocks.append(dict(name=gname, cells=cells, w=bw + 6, h=y + rowh + 12))
    # pack blocks into the sheet: columns
    PW, PH = 841, 594                        # A1 landscape
    colx, coly, colw = 15, 20, 0
    for bl in blocks:
        if coly + bl['h'] > PH - 40:
            colx += colw + 10; coly = 20; colw = 0
        bl['x'], bl['y'] = colx, coly
        coly += bl['h'] + 8; colw = max(colw, bl['w'])
    assert colx + colw < PW, 'sheet too small (%.0f)' % (colx + colw)

    # ---------------------------------------------------------------- emit
    for bl in blocks:
        bx, by = bl['x'], bl['y']
        items.append([S('rectangle'), [S('start'), bx, by], [S('end'), bx + bl['w'], by + bl['h']],
                      [S('stroke'), [S('width'), 0.35], [S('type'), S('dash')], [S('color'), 60, 90, 140, 1]],
                      [S('fill'), [S('type'), S('none')]], [S('uuid'), uid('blk', bl['name'])]])
        items.append([S('text'), bl['name'].upper(), [S('exclude_from_sim'), S('no')], [S('at'), bx + 2, by + 5, 0],
                      [S('effects'), font(2.5, True), [S('justify'), S('left'), S('bottom')]], [S('uuid'), uid('blkt', bl['name'])]])
        for c in bl['cells']:
            p, u = c['p'], c['u']
            x0, y0, x1, y1 = c['box']
            sx = snap(bx + 4 + c['cx'] + c['left'] - x0)
            sy = snap(by + 9 + c['cy'] + c['vert'] + 4 + y1)    # symbol y-up -> top of bbox at row top
            libid = 'arachne:' + c['sid']
            libsyms_used[libid] = c['sym']
            props = [('Reference', p['ref'], (sx, sy - y1 - 2.5), False),
                     ('Value', p['value'], (sx, sy - y0 + 2.5), False),
                     ('Footprint', 'arachne:' + libs.fp_name(p['fp']), (sx, sy), True),
                     ('Datasheet', '~', (sx, sy), True),
                     ] + ([('MPN', p['mpn'], (sx, sy), True)] if p['mpn'] else [])
            node = [S('symbol'), [S('lib_id'), libid], [S('at'), sx, sy, 0], [S('unit'), u],
                    [S('exclude_from_sim'), S('no')], [S('in_bom'), S('no') if p['ref'].startswith('H') else S('yes')],
                    [S('on_board'), S('yes')], [S('dnp'), S('yes') if p['dnp'] else S('no')], [S('uuid'), sym_uuid(p['ref'], u)]]
            for k, v, at, hide in props:
                eff = [S('effects'), font()]
                if hide:
                    eff.append([S('hide'), S('yes')])
                node.append([S('property'), k, v, [S('at'), at[0], at[1], 0], eff])
            for (uu, n, nm, px, py, a) in c['pins']:
                if uu in (0, u):
                    node.append([S('pin'), n, [S('uuid'), uid('pin', p['ref'], u, n)]])
            node.append([S('instances'), [S('project'), NAME, [S('path'), '/' + ROOT_UUID, [S('reference'), p['ref']], [S('unit'), u]]]])
            items.append(node)
            # labels / no-connects on every pin
            for (uu, n, nm, px, py, a) in c['pins']:
                if uu not in (0, u):
                    continue
                ax, ay = sx + px, sy - py
                net = p['pins'].get(n)
                if net:
                    la = {0: 180, 180: 0, 90: 270, 270: 90}[int(a)]
                    just = [S('justify'), S('right' if la in (180, 270) else 'left'), S('bottom')]
                    items.append(glabel(net, ax, ay, la, uid('lbl', p['ref'], u, n)))
                else:
                    items.append([S('no_connect'), [S('at'), round(ax, 3), round(ay, 3)], [S('uuid'), uid('nc', p['ref'], u, n)]])

    # ---------------------------------------------------------------- PWR_FLAGs on the supply nets (ERC)
    fx, fy = 15, 10
    libsyms_used['power:PWR_FLAG'] = PWR
    for i, net in enumerate(['GND', 'VIN_RAW', 'V12', 'V5', 'V5_ESP', 'VBUS_ESP', 'VBUS_CM']):
        x = 15.24 + 22.86 * i; y = 10.16
        node = [S('symbol'), [S('lib_id'), 'power:PWR_FLAG'], [S('at'), x, y, 0], [S('unit'), 1],
                [S('exclude_from_sim'), S('no')], [S('in_bom'), S('no')], [S('on_board'), S('no')], [S('dnp'), S('no')],
                [S('uuid'), uid('pf', net)],
                [S('property'), 'Reference', f'#FLG{i+1:02d}', [S('at'), x, y - 4, 0], [S('effects'), font(), [S('hide'), S('yes')]]],
                [S('property'), 'Value', 'PWR_FLAG', [S('at'), x, y - 3, 0], [S('effects'), font(1.0)]],
                [S('property'), 'Footprint', '', [S('at'), x, y, 0], [S('effects'), font(), [S('hide'), S('yes')]]],
                [S('property'), 'Datasheet', '~', [S('at'), x, y, 0], [S('effects'), font(), [S('hide'), S('yes')]]],
                [S('pin'), '1', [S('uuid'), uid('pfp', net)]],
                [S('instances'), [S('project'), NAME, [S('path'), '/' + ROOT_UUID, [S('reference'), f'#FLG{i+1:02d}'], [S('unit'), 1]]]]]
        items.append(node)
        items.append(glabel(net, x, y, 270, uid('pfl', net)))

    # ---------------------------------------------------------------- notes
    notes = [
        'ARACHNE-18 BRAIN v1.0 - Raspberry Pi CM5 + ESP32-S3 + MPU-6050 hexapod controller (generated by tools/build_sch.py from tools/design.py)',
        'Servo connectors: 1 = GND, 2 = +12 V, 3 = DATA (Feetech STS half-duplex bus, 1 Mbaud). CHECK the pin order against your cable before plugging in!',
        'Bus A = legs L1 L2 L3 (ESP32 UART1: TX IO17, RX IO18, DE IO16).  Bus B = legs L4 L5 L6 (UART2: TX IO10, RX IO11, DE IO9). DE = RTS in RS485 half-duplex mode.',
        'CM5 is never back-powered: ESP->CM5 UART through 74LVC1G34 powered by CM5_3V3, ESP reset/boot via 2N7002, USB switch disabled while the CM5 is off.',
        'ESP32 USB: USB-C plugged in -> ESP32 talks to the PC; otherwise it is wired to CM5 USB3-0 (USB2 pair) -> /dev/ttyACM* on the Pi.',
        'PI LINK header = pins 1-10 of a Raspberry Pi 40-pin header: a straight 10-way ribbon connects an external Pi 4 (5 V, GND, UART, GPIO4 = ESP reset) when no CM5 is fitted.',
        'Buck = TI TPS54560B datasheet design (fig. 32): 400 kHz, 5.0 V 5 A. UVLO start 6.5 V.',
    ]
    for i, t in enumerate(notes):
        items.append([S('text'), t, [S('exclude_from_sim'), S('no')], [S('at'), 15, PH - 42 + 4 * i, 0],
                      [S('effects'), font(2.0), [S('justify'), S('left'), S('bottom')]], [S('uuid'), uid('note', i)]])

    lib_symbols = [S('lib_symbols')]
    for libid, sym in libsyms_used.items():
        s = copy.deepcopy(sym); s[1] = libid
        lib_symbols.append(s)
    doc = [S('kicad_sch'), [S('version'), 20250114], [S('generator'), 'eeschema'], [S('generator_version'), '9.0'],
           [S('uuid'), ROOT_UUID], [S('paper'), 'A1'],
           [S('title_block'), [S('title'), 'ARACHNE-18 brain board'], [S('date'), '2026-10-17'], [S('rev'), '1.0'],
            [S('company'), 'Kobe Vanrobaeys - maakleerplek'], [S('comment'), 1, 'CM5 + ESP32-S3 + MPU-6050, 6 servo-bus legs']],
           lib_symbols] + items + [[S('sheet_instances'), [S('path'), '/', [S('page'), '1']]], [S('embedded_fonts'), S('no')]]
    open(os.path.join(PRJ, NAME + '.kicad_sch'), 'w').write(dump(doc) + '\n')
    print('schematic written: %d blocks' % len(blocks))


if __name__ == '__main__':
    build()
    sys.stdout.flush(); os._exit(0)   # KiCad's SWIG objects can crash Python at shutdown
