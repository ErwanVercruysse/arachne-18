#!/bin/zsh
# Regenerate everything from design.py: libraries, board, schematic, routing, checks, fab outputs (~10 min).
set -e
cd ${0:a:h}
PY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3
VENV=~/Documents/GitHub/arachne-18/.venv/bin/python        # numpy + shapely for the finisher
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
[ -f freerouting.jar ] || curl -sL -o freerouting.jar https://github.com/freerouting/freerouting/releases/download/v2.4.1/freerouting-2.4.1.jar
mkdir -p ../out
python3 design.py
$PY libs.py 2>/dev/null
$PY build_pcb.py 2>/dev/null
$PY build_sch.py 2>/dev/null
./route.sh                                   # Freerouting + GND stitching/pours + DRC report
for i in 1 2 3; do                           # finish what Freerouting left open
  grep -q "Found 0 unconnected" <($KC pcb drc -o ../out/drc.rpt ../arachne_mini.kicad_pcb) && break
  $PY finisher.py dump 2>/dev/null; $VENV finisher.py route; $PY finisher.py apply 2>/dev/null
done
$PY parity.py fixnc 2>/dev/null
$KC pcb drc --schematic-parity -o ../out/drc.rpt ../arachne_mini.kicad_pcb
$KC sch erc -o ../out/erc.rpt ../arachne_mini.kicad_sch
python3 fab.py
