#!/bin/zsh
cd ${0:a:h}
PY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3
$PY build_pcb.py 2>&1 | grep -v assert | tail -1
$PY dumpfp.py ../arachne_brain.kicad_pcb ../out/fp.json 2>/dev/null
~/Documents/GitHub/arachne-18/.venv/bin/python plot.py ../out/fp.json ../out/plan.png
