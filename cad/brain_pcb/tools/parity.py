"""Compare schematic netlist (kicad-cli export) with the PCB pads (run with KiCad python)."""
import subprocess, sys, os, xml.etree.ElementTree as ET
import pcbnew
PRJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
xmlp = os.path.join(PRJ, 'out', 'sch_net.xml')
subprocess.run([KC, 'sch', 'export', 'netlist', '--format', 'kicadxml', '-o', xmlp, os.path.join(PRJ, 'arachne_brain.kicad_sch')], capture_output=True)
r = ET.parse(xmlp).getroot()
sch = {}
for n in r.iter('net'):
    name = n.get('name').lstrip('/')
    for nd in n.iter('node'):
        sch[(nd.get('ref'), nd.get('pin'))] = name if not name.startswith('unconnected-') else ''
b = pcbnew.LoadBoard(os.path.join(PRJ, 'arachne_brain.kicad_pcb'))
pcb = {}
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetNumber():
            k = (f.GetReference(), p.GetNumber()); pcb[k] = p.GetNetname()
bad = []
for k, v in pcb.items():
    s = sch.get(k, None)
    if s is None:
        if v: bad.append((k, 'pcb', v, 'not in sch'))
    elif s != v and not (s == '' and (v == '' or v.startswith('unconnected-'))):
        bad.append((k, 'pcb', v, 'sch', s))
for k in sch:
    if k not in pcb: bad.append((k, 'missing on pcb'))
print('pads compared:', len(pcb), 'mismatches:', len(bad))
if len(sys.argv) > 1 and sys.argv[1] == 'fixnc':
    # give unused pads the 'unconnected-(...)' net names KiCad derives from the schematic
    raw = {}
    for n in r.iter('net'):
        if n.get('name').startswith('unconnected-'):
            for nd in n.iter('node'):
                raw[(nd.get('ref'), nd.get('pin'))] = n.get('name')
    k = 0
    for f in b.GetFootprints():
        for p in f.Pads():
            nm = raw.get((f.GetReference(), p.GetNumber()))
            if nm and not p.GetNetname():
                net = b.FindNet(nm)
                if net is None:
                    net = pcbnew.NETINFO_ITEM(b, nm); b.Add(net)
                p.SetNet(net); k += 1
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(os.path.join(PRJ, 'arachne_brain.kicad_pcb'), b)
    import build_pcb; build_pcb.write_project()
    print('unconnected nets set on', k, 'pads')
for x in bad[:30]: print(x)
sys.stdout.flush(); os._exit(0)   # KiCad's SWIG objects can crash Python at shutdown
