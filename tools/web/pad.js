// Game controller -> robot commands (Gamepad API, USB or Bluetooth, standard layout).
// Shared by the simulation page and the cockpit; `post(path, body)` sends to /sim/... or later /robot/...
export const GP_DO = {0: 'stand', 1: 'lie', 2: 'dance', 3: 'wave', 10: 'pushups', 11: 'look', 8: 'bow', 9: 'twist', 16: 'stop'};
const GAIT_ORDER = ['tripod', 'ripple', 'wave'], GAIT_NL = {tripod: 'tripod', ripple: 'ripple', wave: 'golf'};
const dz = v => Math.abs(v) < .15 ? 0 : (v - Math.sign(v)*.15)/.85;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
export const padName = p => p.id.replace(/\(.*\)/, '').trim() || 'controller';

// legend: <kbd data-gp="ls|rs|0..16"> elements get class "hit" while that control is used
export function highlight(root, p, ax) {
  if (!root) return;
  const down = i => !!p?.buttons[i]?.pressed, val = i => p?.buttons[i]?.value || 0;
  for (const k of root.querySelectorAll('kbd[data-gp]')) {
    const id = k.dataset.gp, on = !p ? false : id === 'ls' ? (ax[0] || ax[1]) && !down(4) : id === 'rs' ? (ax[2] || ax[3]) : id === '6' ? val(6) > .1 || val(7) > .1
      : id === '12' ? down(12) || down(13) : id === '14' ? down(14) || down(15) : down(+id);
    k.classList.toggle('hit', !!on);
  }
}

export function startPad({post, params, toast = () => {}, onStatus = () => {}, legend = null}) {
  const gp = {prev: [], pose: false, height: null, t: 0, name: '', drive: [0, 0, 0]};
  addEventListener('gamepadconnected', e => toast(`controller verbonden: ${padName(e.gamepad)}`));
  addEventListener('gamepaddisconnected', () => { gp.drive = [0, 0, 0]; toast('controller losgekoppeld', 'warn'); });
  setInterval(() => {
    const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(p => p && p.connected) : [];
    const p = pads.find(p => p.mapping === 'standard') || pads[0];
    if (!p) { gp.drive = [0, 0, 0]; if (gp.name) { gp.name = ''; onStatus(null); } highlight(legend, null, []); return; }
    if (gp.name !== p.id) { gp.name = p.id; onStatus(padName(p)); }
    const down = i => !!p.buttons[i]?.pressed, val = i => p.buttons[i]?.value || 0, edge = i => down(i) && !gp.prev[i];
    const ax = [0, 1, 2, 3].map(i => dz(p.axes[i] || 0));
    const prm = params() || {}, now = performance.now(), dt = gp.t ? (now - gp.t)/1000 : 0; gp.t = now;
    if (down(4)) {                                   // LB held: sticks move the body instead of walking
      gp.drive = [0, 0, 0]; gp.pose = true;
      if (!gp.poseT || now - gp.poseT > 90) { gp.poseT = now; post('/set', {roll: ax[0]*.22, pitch: -ax[1]*.22, yaw: -ax[2]*.28, x: -ax[3]*.03}); }
    } else {
      if (gp.pose) { gp.pose = false; post('/set', {roll: 0, pitch: 0, yaw: 0, x: 0}); }
      gp.drive = [-ax[1], -ax[0], -ax[2]];
    }
    for (const [i, what] of Object.entries(GP_DO)) if (edge(+i)) { post('/do', {what}); if (what === 'stop') post('/drive', {vx: 0, vy: 0, wz: 0}); }
    if (edge(5)) { const g = GAIT_ORDER[(GAIT_ORDER.indexOf(prm.gait) + 1) % 3]; post('/set', {gait: g}); toast(`gang: ${GAIT_NL[g]}`); }
    const step = (key, d, a, b, fmt) => { const v = +clamp((prm[key] ?? 0) + d, a, b).toFixed(3); post('/set', {[key]: v}); toast(fmt(v)); };
    if (edge(12)) step('speed', .1, .2, 1.5, v => `snelheid ${Math.round(v*100)} %`);
    if (edge(13)) step('speed', -.1, .2, 1.5, v => `snelheid ${Math.round(v*100)} %`);
    if (edge(14)) step('step_h', -.004, .008, .05, v => `staphoogte ${Math.round(v*1000)} mm`);
    if (edge(15)) step('step_h', .004, .008, .05, v => `staphoogte ${Math.round(v*1000)} mm`);
    const lift = val(7) - val(6);                    // triggers: body height, integrated while held
    if (Math.abs(lift) > .05) {
      if (gp.height == null) gp.height = prm.height ?? 0;
      gp.height = clamp(gp.height + lift*.03*dt, -.035, .025);
      if (!gp.hT || now - gp.hT > 90) { gp.hT = now; post('/set', {height: +gp.height.toFixed(4)}); }
    } else if (gp.height != null) { post('/set', {height: +gp.height.toFixed(4)}); gp.height = null; }
    highlight(legend, p, ax);
    gp.prev = p.buttons.map(b => b.pressed);
  }, 50);
  return gp;
}
