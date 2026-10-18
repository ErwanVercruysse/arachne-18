import sys
"""Build the self-contained project libraries: arachne.kicad_sym and arachne.pretty."""
import os, re, json, shutil, copy
import symlib
from sexp import parse, dump, find, find1, Sym
import design

HERE = os.path.dirname(os.path.abspath(__file__))
PRJ = os.path.dirname(HERE)
FPDIR = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/'
LIB = 'arachne'


def fp_name(fpid):
    return fpid.split(':')[1]


def sym_name(symid):
    lib, name = symid.split(':')
    return name


# ---------------------------------------------------------------- custom footprints
def fp_text(name, body, descr, attr='through_hole'):
    return f'''(footprint "{name}"
\t(version 20241229)
\t(generator "arachne_gen")
\t(layer "F.Cu")
\t(descr "{descr}")
\t(property "Reference" "REF**" (at 0 -4 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
\t(property "Value" "{name}" (at 0 4 0) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))
\t(attr {attr})
{body})
'''


def molex_5267():
    # stock KiCad Mini-SPOX 5267 footprint (chamfered body, real 3D model) + G V S labels
    src = open('/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/Connector_Molex.pretty/'
               'Molex_SPOX_5267-03A_1x03_P2.50mm_Vertical.kicad_mod').read()
    src = src.replace('(footprint "Molex_SPOX_5267-03A_1x03_P2.50mm_Vertical"', '(footprint "Molex_5267-03A_1x03_P2.50mm_Vertical"', 1)
    labels = ''.join(f'\t(fp_text user "{t}" (at {i*2.5} -4.3 0) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.15))))\n'
                     for i, t in enumerate('GVS'))
    src = src.replace('(at 2.5 -4.3 0)', '(at 2.5 4.0 0)', 1)      # move the reference text out of the way of the labels
    return src.rstrip()[:-1].rstrip() + '\n' + labels + ')\n'


def solder_pads():
    b = ''
    b += '\t(pad "1" thru_hole circle (at 0 0) (size 4.6 4.6) (drill 2.2) (layers "*.Cu" "*.Mask"))\n'
    b += '\t(pad "2" thru_hole roundrect (roundrect_rratio 0.2) (at 7 0) (size 4.6 4.6) (drill 2.2) (layers "*.Cu" "*.Mask"))\n'
    b += '\t(fp_text user "GND" (at 0 3.6 0) (layer "F.SilkS") (effects (font (size 1.1 1.1) (thickness 0.2))))\n'
    b += '\t(fp_text user "+12V" (at 7 3.6 0) (layer "F.SilkS") (effects (font (size 1.1 1.1) (thickness 0.2))))\n'
    b += '\t(fp_rect (start -3 -3) (end 10 4.8) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))\n'
    return fp_text('SolderWire_2pad_P7mm_D2.2mm', b, 'Two big solder pads for the 12 V battery leads (14-16 AWG)')


def mount_hole():
    b = '\t(pad "" np_thru_hole circle (at 0 0) (size 4.2 4.2) (drill 4.2) (layers "*.Cu" "*.Mask"))\n'
    b += '\t(fp_circle (center 0 0) (end 3.4 0) (stroke (width 0.15) (type solid)) (fill no) (layer "F.SilkS"))\n'
    b += '\t(fp_circle (center 0 0) (end 3.4 0) (stroke (width 0.15) (type solid)) (fill no) (layer "B.SilkS"))\n'
    b += '\t(fp_circle (center 0 0) (end 3.7 0) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))\n'
    return fp_text('MountingHole_4.2mm_M3_loose', b, 'M3 hole with 0.6 mm play: fits CAD (64,25) and measured (65.1,25.35) inserts',
                   attr='exclude_from_pos_files exclude_from_bom')


def solder_pads_5v():
    b = '\t(pad "1" thru_hole circle (at 0 0) (size 3.4 3.4) (drill 1.6) (layers "*.Cu" "*.Mask"))\n'
    b += '\t(pad "2" thru_hole roundrect (roundrect_rratio 0.2) (at 5 0) (size 3.4 3.4) (drill 1.6) (layers "*.Cu" "*.Mask"))\n'
    b += '\t(fp_text user "GND" (at 0 2.9 0) (layer "F.SilkS") (effects (font (size 0.9 0.9) (thickness 0.18))))\n'
    b += '\t(fp_text user "5V PI" (at 5 2.9 0) (layer "F.SilkS") (effects (font (size 0.9 0.9) (thickness 0.18))))\n'
    b += '\t(fp_rect (start -2.2 -2.2) (end 7.2 3.8) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))\n'
    return fp_text('SolderWire_2pad_P5mm_D1.6mm_5V', b, 'Two solder pads: 5 V 3 A out to the Raspberry Pi 4 (18-20 AWG)')


def mount_hole_m3():
    b = '\t(pad "" np_thru_hole circle (at 0 0) (size 3.2 3.2) (drill 3.2) (layers "*.Cu" "*.Mask"))\n'
    b += '\t(fp_circle (center 0 0) (end 3.0 0) (stroke (width 0.15) (type solid)) (fill no) (layer "F.SilkS"))\n'
    b += '\t(fp_circle (center 0 0) (end 3.0 0) (stroke (width 0.15) (type solid)) (fill no) (layer "B.SilkS"))\n'
    b += '\t(fp_circle (center 0 0) (end 3.3 0) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))\n'
    return fp_text('MountingHole_3.2mm_M3', b, 'M3 hole (3.2 mm) for spacers onto the acrylic plate', attr='exclude_from_pos_files exclude_from_bom')


def build_fp_lib():
    d = os.path.join(PRJ, LIB + '.pretty'); os.makedirs(d, exist_ok=True)
    for f in os.listdir(d):
        os.remove(os.path.join(d, f))
    custom = {'Molex_5267-03A_1x03_P2.50mm_Vertical': molex_5267(), 'SolderWire_2pad_P7mm_D2.2mm': solder_pads(),
              'MountingHole_3.2mm_M3': mount_hole_m3(), 'SolderWire_2pad_P5mm_D1.6mm_5V': solder_pads_5v()}
    for n, t in custom.items():
        open(os.path.join(d, n + '.kicad_mod'), 'w').write(t)
    for p in design.parts:
        lib, n = p['fp'].split(':')
        if lib == LIB:
            continue
        shutil.copy(os.path.join(FPDIR, lib + '.pretty', n + '.kicad_mod'), os.path.join(d, n + '.kicad_mod'))
    # sensor strip: three 1x19 rows 2.54 mm apart -> slim courtyard so the rows do not 'overlap'
    hp = os.path.join(d, 'PinHeader_1x19_P2.54mm_Vertical.kicad_mod')
    if os.path.exists(hp):
        t = open(hp).read()
        assert '(start -1.77 -1.78)\n\t\t(end 1.77 47.49)' in t
        t = t.replace('(start -1.77 -1.78)\n\t\t(end 1.77 47.49)', '(start -1.2 -1.5)\n\t\t(end 1.2 47.2)')
        open(hp, 'w').write(t)
    return d


# ---------------------------------------------------------------- symbols
def rename_sym(s, new):
    old = s[1]; s[1] = new
    for sub in find(s, 'symbol'):
        sub[1] = new + sub[1][len(old):]
    return s


def prop(name, val, at=(0, 0), hide=False, size=1.27):
    e = [Sym('effects'), [Sym('font'), [Sym('size'), size, size]]]
    if hide:
        e.append([Sym('hide'), Sym('yes')])
    return [Sym('property'), name, val, [Sym('at'), at[0], at[1], 0], e]


def pin(ptype, x, y, ang, num, name, length=5.08):
    f = [Sym('effects'), [Sym('font'), [Sym('size'), 1.27, 1.27]]]
    return [Sym('pin'), Sym(ptype), Sym('line'), [Sym('at'), x, y, ang], [Sym('length'), length],
            [Sym('name'), name, f], [Sym('number'), str(num), f]]


def cm5_symbol():
    names = {int(k): v for k, v in json.load(open(os.path.join(HERE, 'cm5_pins.json'))).items()}

    def ptype(n):
        if n in ('GND', '5V', 'GPIO_VREF'):
            return 'power_in'
        if n in ('CM5_3.3V', 'CM5_1.8V'):
            return 'power_out'
        return 'bidirectional'
    s = [Sym('symbol'), 'CM5', [Sym('pin_names'), [Sym('offset'), 1.016]],
         [Sym('exclude_from_sim'), Sym('no')], [Sym('in_bom'), Sym('yes')], [Sym('on_board'), Sym('yes')],
         prop('Reference', 'M', (0, 5.08)), prop('Value', 'CM5', (0, -132.08)),
         prop('Footprint', 'arachne:Raspberry-Pi-5-Compute-Module', hide=True),
         prop('Datasheet', 'https://datasheets.raspberrypi.com/cm5/cm5-datasheet.pdf', hide=True),
         prop('Description', 'Raspberry Pi Compute Module 5 (2x 100-pin DF40)', hide=True)]
    for unit in (1, 2):
        sub = [Sym('symbol'), f'CM5_{unit}_1',
               [Sym('rectangle'), [Sym('start'), -25.4, 2.54], [Sym('end'), 25.4, -127.0],
                [Sym('stroke'), [Sym('width'), 0.254], [Sym('type'), Sym('default')]],
                [Sym('fill'), [Sym('type'), Sym('background')]]],
               [Sym('text'), f'J{unit} pins {(unit-1)*100+1}-{unit*100}', [Sym('at'), 0, 0, 0],
                [Sym('effects'), [Sym('font'), [Sym('size'), 1.27, 1.27]]]]]
        seen = set()
        for i in range(50):
            for side, n in ((0, (unit - 1) * 100 + 2 * i + 1), (1, (unit - 1) * 100 + 2 * i + 2)):
                nm = names[n]; t = ptype(nm)
                if nm in seen and t.startswith('power'):
                    t = 'passive'
                seen.add(nm)
                y = -2.54 * i - 2.54
                sub.append(pin(t, -30.48 if side == 0 else 30.48, y, 0 if side == 0 else 180, n, nm))
        s.append(sub)
    s.append([Sym('embedded_fonts'), Sym('no')])
    return s


def build_sym_lib():
    out = {}
    for p in design.parts:
        sid = p['sym']; n = sym_name(sid)
        if n in out:
            continue
        if sid == 'arachne:CM5':
            out[n] = cm5_symbol()
        elif sid == 'arachne:Q_PMOS_GDS':
            s = rename_sym(symlib.get('Device:Q_PMOS'), 'Q_PMOS_GDS')
            m = {'G': '1', 'D': '2', 'S': '3'}
            for sub in find(s, 'symbol'):
                for pn in find(sub, 'pin'):
                    num = find1(pn, 'number'); num[1] = m[num[1]]
            out[n] = s
        else:
            out[n] = symlib.get(sid)
        # point the footprint field to our own lib
        fpid = LIB + ':' + fp_name(p['fp'])
        for e in out[n]:
            if isinstance(e, list) and e[0] == 'property' and e[1] == 'Footprint':
                e[2] = fpid if sid != 'Device:R' and sid != 'Device:C' else ''
    lib = [Sym('kicad_symbol_lib'), [Sym('version'), 20241209], [Sym('generator'), 'arachne_gen'],
           [Sym('generator_version'), '1.0']] + list(out.values())
    open(os.path.join(PRJ, LIB + '.kicad_sym'), 'w').write(dump(lib) + '\n')
    return out


def lib_tables():
    open(os.path.join(PRJ, 'sym-lib-table'), 'w').write(
        '(sym_lib_table\n\t(version 7)\n\t(lib (name "arachne")(type "KiCad")(uri "${KIPRJMOD}/arachne.kicad_sym")(options "")(descr "ARACHNE mini brain board"))\n)\n')
    open(os.path.join(PRJ, 'fp-lib-table'), 'w').write(
        '(fp_lib_table\n\t(version 7)\n\t(lib (name "arachne")(type "KiCad")(uri "${KIPRJMOD}/arachne.pretty")(options "")(descr "ARACHNE mini brain board"))\n)\n')


if __name__ == '__main__':
    build_fp_lib(); syms = build_sym_lib(); lib_tables()
    print('footprints + %d symbols written' % len(syms))
    sys.stdout.flush(); os._exit(0)   # KiCad's SWIG objects can crash Python at shutdown
