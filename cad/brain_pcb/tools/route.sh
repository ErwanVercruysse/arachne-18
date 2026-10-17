#!/bin/zsh
# Two Freerouting passes (second one only fills the gaps), then GND stitching + pours + DRC.
cd ${0:a:h}
PY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3
JAVA=$(brew --prefix openjdk)/bin/java
for pass in 1; do
  $PY route.py export 2>/dev/null
  (cd ../out && rm -f board.ses && $JAVA -jar ../tools/freerouting.jar -de board.dsn -do board.ses -mp $([ $pass = 1 ] && echo 100 || echo 40) -mt 8 --gui.enabled=false > fr$pass.log 2>&1)
  grep -E "Auto-routing stage completed" ../out/fr$pass.log | sed 's/.*final score/pass '$pass' score/'
  $PY route.py import_raw 2>/dev/null | grep -v Debug
done
$PY route.py finish 2>/dev/null | grep -v Debug
/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli pcb drc -o ../out/drc.rpt ../arachne_brain.kicad_pcb | grep Found
grep -A3 "^\[" ../out/drc.rpt | grep -E "^\[|@"
