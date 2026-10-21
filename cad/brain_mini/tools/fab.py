"""Fabrication outputs for JLCPCB: gerbers + drill (zip), BOM and CPL (pick & place), PDF schematic, renders."""
import re, math, os, csv, subprocess, zipfile, shutil, collections
import design

PRJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
PCB = os.path.join(PRJ, 'arachne_mini.kicad_pcb')
SCH = os.path.join(PRJ, 'arachne_mini.kicad_sch')
FAB = os.path.join(PRJ, 'fab')


# JLCPCB / LCSC part numbers, checked for stock on 2026-10-17 (tools/jlc_stock.py)
LCSC = {'AOD4185': 'C400894', 'BZT52C15': 'C173427', 'SMBJ15CA': 'C78410', '330u 25V': 'C350008', '10u 25V': 'C9807',
        '100n': 'C14663', '442k': 'C23175', '90.9k': 'C23129', '243k': 'C23351', '6.8u': 'C20804570', 'B560C': 'C85100',
        '47u 10V': 'C84494', '53.6k': 'C23074', '10.2k': 'C22772', '16.9k': 'C25954', '4n7': 'C53987', '47p': 'C1671',
        'groen 5V': 'C12624', 'groen ACT': 'C12624', 'blauw ESP': 'C2288', '2k2': 'C4190', 'SS14': 'C2480',
        'AMS1117-3.3': 'C6186', '10u': 'C15850', '22u': 'C29277', 'ESP32-S3-WROOM-1-N16R8': 'C2913202', '10k': 'C25804',
        '1u': 'C15849', 'RESET': 'C72443', 'BOOT': 'C72443', '2N7002': 'C8545', '100k': 'C25803', '1k': 'C21190',
        '74AHCT1G126': 'C163712', '74LVC1G34': 'C840096', '47': 'C23182', 'MPU-6050': 'C24112', '2n2': 'C1604',
        '10n': 'C57112', '4k7': 'C23162', '15k': 'C22809', 'USB-C PI': 'C165948', '5k1': 'C23186',
        'USBLC6-2SC6': 'C2687116', '20k': 'C4184', 'FSUSB42MUX': 'C11355', '470': 'C23179', 'RPIBOOT': 'C2886898',
        'AAN/UIT': 'C2886898', 'FAN': 'C160404', 'PI LINK': 'C492422', 'GPIO': 'C41430886', 'SENSOR S': 'C27985198', 'SENSOR V': 'C27985198', 'SENSOR G': 'C27985198', 'I2C': 'C160404', 'TPS54560B': 'C1850354',
        'CM5 (wireless)': 'C6782225 x2 (connectors only)', '12V IN': '(solder pads, no part)', '5V PI': '(solder pads, no part)',
        # 2026-10-21 price pass: JLC basic parts wherever one exists
        '10u 25V': 'C15850', '47u 10V': 'C96123', '22u': 'C45783', 'LED 5V': 'C2286', 'LED ESP': 'C2286', 'SS54': 'C22452',
        '470k': 'C23178', '220k': 'C22961', '27k': 'C22967', 'LSM6DS3TR-C': 'C967633'}


def lcsc(value):
    return 'C45581067' if value.endswith('servo') else LCSC.get(value, '')


def placed(value):
    """False for footprints JLC does not fit (bare solder pads)"""
    return lcsc(value).startswith('C')


def run(*a):
    r = subprocess.run([KC, *a], capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr)
    return r


def main():
    shutil.rmtree(FAB, ignore_errors=True)
    g = os.path.join(FAB, 'gerbers'); os.makedirs(g)
    layers = 'F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts'
    run('pcb', 'export', 'gerbers', '--layers', layers, '--subtract-soldermask', '--no-protel-ext', '-o', g + '/', PCB)
    run('pcb', 'export', 'drill', '--format', 'excellon', '--excellon-separate-th', '--generate-map', '--map-format', 'gerberx2',
        '-o', g + '/', PCB)
    with zipfile.ZipFile(os.path.join(FAB, 'arachne_mini_gerbers_JLCPCB.zip'), 'w', zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(g)):
            z.write(os.path.join(g, f), f)
    # pick & place (only parts that are in the BOM)
    VAL = {p['ref']: p['value'] for p in design.parts}
    FP = {p['ref']: p['fp'].split(':')[1] for p in design.parts}
    ROT = [(r'^SOT-223', 180), (r'^SOT-23', 180), (r'^CP_Elec_', 180), (r'^Texas_R-PDSO-G8', 270), (r'^SOIC-', 270),
           (r'^QFN-', 90), (r'^JST_SH_SM', 180)]

    # JLC centres some parts on the middle of their pads instead of KiCad's origin (footprint-local mm, KiCad axes)
    OFFS = [(r'^ESP32-S3-WROOM-1', (0.0, 3.62))]          # module: pad field centre is 3.62 mm below the body centre

    def jlc_xy(ref, x, y, rot):
        for pat, (dx, dy) in OFFS:
            if re.match(pat, FP.get(ref, '')):
                a = math.radians(float(rot))                    # pos file: y up, rotation counter-clockwise
                lx, ly = dx, -dy
                return x + lx * math.cos(a) - ly * math.sin(a), y + lx * math.sin(a) + ly * math.cos(a)
        return x, y

    def jlc_rot(ref, rot):
        for pat, d in ROT:
            if re.match(pat, FP.get(ref, '')):
                return (float(rot) + d) % 360
        return float(rot) % 360
    pos = os.path.join(FAB, 'pos_raw.csv')
    run('pcb', 'export', 'pos', '--format', 'csv', '--units', 'mm', '--side', 'both', '--exclude-dnp', '-o', pos, PCB)
    with open(pos) as f, open(os.path.join(FAB, 'arachne_mini_CPL_JLCPCB.csv'), 'w', newline='') as o:
        w = csv.writer(o); w.writerow(['Designator', 'Mid X', 'Mid Y', 'Layer', 'Rotation'])
        for r in csv.DictReader(f):
            if r['Ref'].startswith('H') or r['Ref'] in design.DNP or not placed(VAL.get(r['Ref'], '')):
                continue
            x, y = jlc_xy(r['Ref'], float(r['PosX']), float(r['PosY']), r['Rot'])
            w.writerow([r['Ref'], f'{x:.4f}mm', f'{y:.4f}mm', 'Top' if r['Side'] == 'top' else 'Bottom', f"{jlc_rot(r['Ref'], r['Rot']):g}"])
    os.remove(pos)
    # BOM: one line per LCSC part (JLC warns when two lines match the same part); bare solder pads are left out
    groups = collections.OrderedDict()
    for p in design.parts:
        if p['ref'].startswith('H') or p['dnp'] or not placed(p['value']):
            continue
        g = groups.setdefault(lcsc(p['value']), dict(vals=[], refs=[], fp=p['fp'].split(':')[1], mpn=p['mpn']))
        g['refs'].append(p['ref'])
        if p['value'] not in g['vals']: g['vals'].append(p['value'])
    with open(os.path.join(FAB, 'arachne_mini_BOM_JLCPCB.csv'), 'w', newline='') as o:
        w = csv.writer(o); w.writerow(['Comment', 'Designator', 'Footprint', 'Quantity', 'MPN / note', 'LCSC Part #'])
        for code, g in groups.items():
            v = g['vals'][0] if len(g['vals']) == 1 else ' / '.join(g['vals'])
            w.writerow([v, ','.join(g['refs']), g['fp'], len(g['refs']), g['mpn'], code])
    run('sch', 'export', 'pdf', '-o', os.path.join(FAB, 'arachne_mini_schema.pdf'), SCH)
    for side in ('top', 'bottom'):
        run('pcb', 'render', '--side', side, '--width', '2400', '--height', '1400', '--quality', 'high',
            '--background', 'opaque', '-o', os.path.join(FAB, f'render_{side}.png'), PCB)
    run('pcb', 'render', '--side', 'top', '--rotate', '-35,0,25', '--zoom', '1.1', '--width', '2400', '--height', '1500',
        '--quality', 'high', '--background', 'opaque', '-o', os.path.join(FAB, 'render_3d.png'), PCB)
    print('fab outputs in', FAB, sorted(os.listdir(FAB)))


if __name__ == '__main__':
    main()
