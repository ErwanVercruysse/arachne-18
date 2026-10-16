#!/usr/bin/env bash
# Put the current code on the robot's Pi and restart the twin service, so the live twin
# (http://spin.local:5870) and the simulation (http://spin.local:5870/sim) are served to the whole wifi.
#
#   tools/deploy_pi.sh             # sync tools/ + simulation/ code, restart arachne-twin
#   tools/deploy_pi.sh --dry       # only show what would change
#
# calibration/ is NOT synced: the Pi's twin_map.json changes when someone presses ⇄/◎ there.
# sudo on the Pi needs the password: set ARACHNE_PI_PW, otherwise ssh asks for it.
set -euo pipefail
PI=${ARACHNE_PI:-spin@spin.local}
cd "$(dirname "$0")/.."
DRY=(); [[ "${1:-}" == "--dry" ]] && DRY=(--dry-run)

echo "== code naar $PI"
rsync -az --itemize-changes ${DRY[@]+"${DRY[@]}"} --exclude __pycache__ --exclude '*.pyc' tools/ "$PI:arachne-18/tools/"
rsync -az --itemize-changes ${DRY[@]+"${DRY[@]}"} --exclude __pycache__ --exclude meshes \
  simulation/arachne.xml simulation/arachne_fixed.xml simulation/joint_map.json simulation/bam_sts3215_12v_approx_m6.json \
  "$PI:arachne-18/simulation/"
rsync -az --ignore-existing ${DRY[@]+"${DRY[@]}"} simulation/meshes/ "$PI:arachne-18/simulation/meshes/"
# hub key: lets the Pi announce itself to the home server (spin.vanrol.eu) so it can pass commands on
if [[ -f ~/.config/arachne/hub_key ]]; then
  ssh "$PI" 'mkdir -p ~/.config/arachne && chmod 700 ~/.config/arachne'
  rsync -a --chmod=F600 ${DRY[@]+"${DRY[@]}"} ~/.config/arachne/hub_key "$PI:.config/arachne/hub_key"
fi
[[ ${#DRY[@]} -gt 0 ]] && exit 0

echo "== twin-service herstarten"
if [[ -n "${ARACHNE_PI_PW:-}" ]]; then
  ssh "$PI" "echo '$ARACHNE_PI_PW' | sudo -S -p '' systemctl restart arachne-twin"
else
  ssh -t "$PI" "sudo systemctl restart arachne-twin"
fi

echo "== wachten tot hij antwoordt"
HOST=${PI#*@}
for i in $(seq 40); do
  if curl -s -o /dev/null -m 2 "http://$HOST:5870/"; then break; fi; sleep 1
done
code=$(curl -s -o /dev/null -w '%{http_code}' -m 60 "http://$HOST:5870/sim/scene.json" || true)
echo "live twin: http://$HOST:5870   simulatie: http://$HOST:5870/sim   (scene: $code)"
