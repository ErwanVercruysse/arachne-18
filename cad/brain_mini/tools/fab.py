"""Fabrication outputs for JLCPCB: gerbers + drill (zip), BOM and CPL (pick & place), PDF schematic, renders."""
import os, csv, subprocess, zipfile, shutil, collections
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
        'CM5 (wireless)': 'C6782225 x2 (connectors only)', '12V IN': '(solder pads, no part)', '5V PI': '(solder pads, no part)'}


def lcsc(value):
    return 'C45581067 (Molex 22-03-5035)' if value.endswith('servo') else LCSC.get(value, '')


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
    # pick & place
    pos = os.path.join(FAB, 'pos_raw.csv')
    run('pcb', 'export', 'pos', '--format', 'csv', '--units', 'mm', '--side', 'both', '--exclude-dnp', '-o', pos, PCB)
    with open(pos) as f, open(os.path.join(FAB, 'arachne_mini_CPL_JLCPCB.csv'), 'w', newline='') as o:
        w = csv.writer(o); w.writerow(['Designator', 'Mid X', 'Mid Y', 'Layer', 'Rotation'])
        for r in csv.DictReader(f):
            if r['Ref'].startswith('H') or r['Ref'] in design.DNP:
                continue
            w.writerow([r['Ref'], r['PosX'] + 'mm', r['PosY'] + 'mm', 'Top' if r['Side'] == 'top' else 'Bottom', r['Rot']])
    os.remove(pos)
    # BOM grouped by value + footprint + MPN
    groups = collections.OrderedDict()
    for p in design.parts:
        if p['ref'].startswith('H') or p['dnp']:
            continue
        k = (p['value'], p['fp'].split(':')[1], p['mpn'])
        groups.setdefault(k, []).append(p['ref'])
    with open(os.path.join(FAB, 'arachne_mini_BOM_JLCPCB.csv'), 'w', newline='') as o:
        w = csv.writer(o); w.writerow(['Comment', 'Designator', 'Footprint', 'Quantity', 'MPN / note', 'LCSC Part #'])
        for (v, fp, mpn), refs in groups.items():
            w.writerow([v, ','.join(refs), fp, len(refs), mpn, lcsc(v)])
    run('sch', 'export', 'pdf', '-o', os.path.join(FAB, 'arachne_mini_schema.pdf'), SCH)
    for side in ('top', 'bottom'):
        run('pcb', 'render', '--side', side, '--width', '2400', '--height', '1400', '--quality', 'high',
            '--background', 'opaque', '-o', os.path.join(FAB, f'render_{side}.png'), PCB)
    run('pcb', 'render', '--side', 'top', '--rotate', '-35,0,25', '--zoom', '1.1', '--width', '2400', '--height', '1500',
        '--quality', 'high', '--background', 'opaque', '-o', os.path.join(FAB, 'render_3d.png'), PCB)
    print('fab outputs in', FAB, sorted(os.listdir(FAB)))


if __name__ == '__main__':
    main()
