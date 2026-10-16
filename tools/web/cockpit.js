// ARACHNE cockpit: camera + 3D twin + controls on one screen (made for a landscape tablet).
// The buttons drive the simulation (/sim/...). The real robot is view-only until it serves the same API under /robot/.
import {startPad} from '/web/pad.js';

const $ = s => document.querySelector(s), $$ = s => [...document.querySelectorAll(s)];
const store = {get: (k, d) => { try { return JSON.parse(localStorage.getItem('ck.' + k)) ?? d; } catch { return d; } },
               set: (k, v) => { try { localStorage.setItem('ck.' + k, JSON.stringify(v)); } catch {} }};
const fmt = (v, n = 1) => (v ?? 0).toFixed(n);
let target = 'sim', last = null, robotApi = false;
const api = () => target === 'sim' ? '/sim' : '/robot';
const post = (path, body = {}) => target === 'robot' && !robotApi ? Promise.resolve({})
  : fetch(api() + path, {method: 'POST', body: JSON.stringify(body)}).then(r => r.json()).catch(() => ({}));

function toast(msg, level) {
  const t = document.createElement('div'); t.className = 'toast ' + (level || ''); t.textContent = msg;
  $('#toasts').appendChild(t); setTimeout(() => t.remove(), 2700);
}

// ------------------------------------------------------------------ views: camera and 3D, tap the small one to swap
const vCam = $('#v-cam'), v3d = $('#v-3d'), frame = $('#frame');
function layout(camBig) {
  vCam.classList.toggle('big', camBig); vCam.classList.toggle('small', !camBig);
  v3d.classList.toggle('big', !camBig); v3d.classList.toggle('small', camBig);
  store.set('camfirst', camBig); $('#camfirst').checked = camBig;
}
vCam.addEventListener('click', () => vCam.classList.contains('small') && layout(true));
v3d.addEventListener('click', () => v3d.classList.contains('small') && layout(false));
function load3d() {
  const cam = store.get('cam3d', 'follow');
  frame.src = target === 'sim' ? `/sim?embed&cam=${cam}` : '/?embed';
  $('#tag3d').textContent = target === 'sim' ? 'simulatie' : 'live twin';
}

// camera: an MJPEG stream (or a JPG that is refreshed); empty setting = the Pi's own /camera.mjpg
const img = $('#camimg'); let camTimer = null, camSnap = false;
let camIdx = store.get('camidx', 0), camCount = 0;
function camUrl() { return store.get('camurl', '') || `/camera.mjpg?cam=${camIdx}`; }
async function countCams() {
  try { camCount = (await (await fetch('/cameras', {cache: 'no-store'})).json()).count || 0; } catch { camCount = 0; }
  if (camIdx >= camCount) camIdx = 0;
  const multi = camCount > 1 && !store.get('camurl', '');
  $('#camnext').hidden = !multi; $('#camtag').textContent = multi ? `camera ${camIdx + 1}/${camCount}` : 'camera';
}
$('#camnext').addEventListener('click', e => { e.stopPropagation(); camIdx = (camIdx + 1) % Math.max(1, camCount); store.set('camidx', camIdx); countCams(); startCam(); });
function startCam() {
  clearTimeout(camTimer); vCam.classList.remove('live');
  const u = camUrl(); camSnap = /\.jpe?g($|\?)|snapshot|capture/i.test(u);
  img.onload = () => { vCam.classList.add('live'); if (camSnap) camTimer = setTimeout(() => { img.src = u + (u.includes('?') ? '&' : '?') + 't=' + Date.now(); }, 150); };
  img.onerror = () => {
    vCam.classList.remove('live');
    $('#camwhy').textContent = store.get('camurl', '') ? `Geen beeld van ${u}. Staat de camera aan en op hetzelfde wifi?`
      : 'Er is nog geen camerabeeld. Een Pi-camera wordt vanzelf gevonden; een ESP32-CAM of andere stream stel je in via ⚙.';
    camTimer = setTimeout(startCam, 5000);
  };
  img.src = u + (u.includes('?') ? '&' : '?') + 't=' + Date.now();
}

// ------------------------------------------------------------------ target: simulation or real robot
async function checkRobot() {
  try { const r = await fetch('/robot/info', {cache: 'no-store'}); robotApi = r.ok; } catch { robotApi = false; }
  $('#t-robot').title = robotApi ? 'de echte robot besturen' : 'nog niet: alleen kijken naar de live twin';
}
function setTarget(t) {
  target = t; store.set('target', t);
  $$('#target button').forEach(b => b.classList.toggle('on', b.dataset.t === t));
  const vo = t === 'robot' && !robotApi;
  document.body.classList.toggle('viewonly', vo); $('#viewonly').hidden = !vo;
  load3d();
}
$$('#target button').forEach(b => b.addEventListener('click', () => setTarget(b.dataset.t)));

// ------------------------------------------------------------------ buttons and sliders
$$('[data-do]').forEach(b => b.addEventListener('click', () => post('/do', {what: b.dataset.do})));
$('#b-stop').addEventListener('click', () => { post('/do', {what: 'stop'}); post('/drive', {vx: 0, vy: 0, wz: 0}); });
$$('#gait button').forEach(b => b.addEventListener('click', () => post('/set', {gait: b.dataset.gait})));
const SL = {speed: ['#r-speed', '#o-speed', v => `${Math.round(v*100)} %`], height: ['#r-height', '#o-height', v => `${v > 0 ? '+' : ''}${Math.round(v*1000)} mm`],
            step_h: ['#r-step', '#o-step', v => `${Math.round(v*1000)} mm`]};
const dragging = new Set();
for (const [k, [r, o, f]] of Object.entries(SL)) {
  const el = $(r), out = $(o), show = () => out.textContent = f(+el.value); let t = 0;
  el.addEventListener('pointerdown', () => dragging.add(k)); el.addEventListener('pointerup', () => setTimeout(() => dragging.delete(k), 400));
  el.addEventListener('input', () => { show(); const n = performance.now(); if (n - t > 70) { t = n; post('/set', {[k]: +el.value}); } });
  el.addEventListener('change', () => { show(); post('/set', {[k]: +el.value}); });
  show();
}

// ------------------------------------------------------------------ touch sticks + controller -> drive
const drive = {move: [0, 0], turn: 0, gp: [0, 0, 0], voice: [0, 0, 0], voiceUntil: 0, goal: null};
function stick(el, onMove, wide) {
  const knob = el.querySelector('.knob'); let id = null;
  const set = (x, y) => knob.style.transform = `translate(${x}px,${y}px)`;
  const move = e => {
    const r = el.getBoundingClientRect(); let x = e.clientX - (r.left + r.width/2), y = wide ? 0 : e.clientY - (r.top + r.height/2);
    const R = (wide ? r.width : r.width)/2 - 30;
    if (wide) x = Math.max(-R, Math.min(R, x)); else { const n = Math.hypot(x, y); if (n > R) { x *= R/n; y *= R/n; } }
    set(x, y); onMove(x/R, y/R);
  };
  el.addEventListener('pointerdown', e => { id = e.pointerId; el.setPointerCapture(id); el.classList.add('active'); move(e); });
  el.addEventListener('pointermove', e => e.pointerId === id && move(e));
  const end = e => { if (e.pointerId !== id) return; id = null; el.classList.remove('active'); set(0, 0); onMove(0, 0); };
  el.addEventListener('pointerup', end); el.addEventListener('pointercancel', end);
}
stick($('#s-move'), (x, y) => drive.move = [x, y]);
stick($('#s-turn'), x => drive.turn = x, true);
const pad = startPad({post, params: () => last?.motion?.params, toast, legend: $('#gpmap'), onStatus: name => {
  $('#c-pad').classList.toggle('ok', !!name); $('#c-pad span').textContent = name || 'geen controller';
}});
let sentZero = true;
setInterval(() => {
  const g = pad.drive, c = v => Math.max(-1, Math.min(1, v));
  if (drive.goal) goalStep();
  const vo = performance.now() < drive.voiceUntil ? drive.voice : [0, 0, 0];
  const vx = c(-drive.move[1] + g[0] + vo[0])*.07, vy = c(-drive.move[0] + g[1] + vo[1])*.05, wz = c(-drive.turn + g[2] + vo[2])*.5;
  const zero = !vx && !vy && !wz;
  if (zero && sentZero) return;
  post('/drive', {vx, vy, wz}); sentZero = zero;
}, 100);

// ------------------------------------------------------------------ status
function cls(el, c) { el.className = c || ''; }
function gaugeLabels([a, b]) { $('#g-speed').nextElementSibling.textContent = a; $('#g-head').nextElementSibling.textContent = b; }
async function poll() {
  try {
    if (target === 'sim') {
      const s = await (await fetch('/sim/state', {cache: 'no-store'})).json(); last = s;
      const b = s.base, mo = s.motion; gaugeLabels(['cm/s', 'koers']);
      $('#g-speed').textContent = fmt(Math.hypot(b.vx, b.vy)*100); $('#g-head').textContent = `${Math.round(b.yaw)}°`;
      $('#g-h').textContent = Math.round(b.z*1000);
      const gm = $('#g-margin'); gm.textContent = s.support.length >= 3 ? Math.round(s.margin*1000) : '–';
      cls(gm, s.support.length < 3 ? '' : s.margin < .01 ? 'bad' : s.margin < .025 ? 'warn' : 'ok');
      $('#g-v').textContent = fmt(s.supply.v); cls($('#g-v'), s.supply.v < 9 ? 'bad' : s.supply.v < 10.8 ? 'warn' : '');
      $('#g-a').textContent = fmt(s.supply.i); cls($('#g-a'), s.supply.i > s.supply.ilim*.95 ? 'bad' : s.supply.i > s.supply.ilim*.7 ? 'warn' : '');
      $('#c-mode').textContent = s.paused ? 'gepauzeerd' : mo.label; $('#c-mode').classList.toggle('live', mo.mode !== 'idle');
      $$('#gait button').forEach(x => x.classList.toggle('on', x.dataset.gait === mo.params.gait));
      $$('#side [data-do]').forEach(x => x.classList.toggle('run', mo.mode === 'show' && mo.label === x.textContent.toLowerCase()));
      for (const [k, [r, o, f]] of Object.entries(SL)) {
        const v = mo.params[k]; if (v == null || dragging.has(k)) continue;
        if (Math.abs(+$(r).value - v) > 1e-6) { $(r).value = v; $(o).textContent = f(v); }
      }
      linkOk('simulatie');
    } else {
      const s = await (await fetch('/state', {cache: 'no-store'})).json();
      const ad = Object.values(s.adapters || {}), ok = ad.filter(v => v === 'ok').length;
      $('#c-mode').textContent = ok ? `${Math.round(s.hz || 0)} Hz` : 'geen servo’s'; $('#c-mode').classList.toggle('live', !!ok);
      for (const id of ['#g-speed', '#g-head', '#g-h', '#g-margin', '#g-v', '#g-a']) { $(id).textContent = '–'; cls($(id)); }
      const volts = Object.values(s.joints || {}).map(j => j.volt).filter(v => v > 0);
      if (volts.length) { const v = volts.reduce((a, b) => a + b)/volts.length; $('#g-v').textContent = fmt(v); cls($('#g-v'), v < 9 ? 'bad' : v < 10.8 ? 'warn' : ''); }
      if (s.imu?.ok) { $('#g-speed').textContent = fmt(s.imu.roll); $('#g-head').textContent = fmt(s.imu.pitch); }
      gaugeLabels(s.imu?.ok ? ['roll °', 'pitch °'] : ['cm/s', 'koers']);
      linkOk(ok ? `${ok} adapter${ok > 1 ? 's' : ''}` : 'robot niet verbonden', !ok);
    }
  } catch { const c = $('#c-link'); c.className = 'chip bad'; c.querySelector('span').textContent = 'geen verbinding'; }
}
function linkOk(text, warn) { const c = $('#c-link'); c.className = 'chip ' + (warn ? '' : 'ok'); c.querySelector('span').textContent = text; }
setInterval(poll, 200);

// closed-loop moves for the voice assistant: turn N degrees / walk N metres, measured on the robot's own pose
function goalStep() {
  const g = drive.goal, s = last; if (!s) return;
  const now = performance.now();
  if (now > g.deadline) { drive.goal = null; drive.voiceUntil = 0; return; }
  if (g.kind === 'turn') {
    if (g.prev == null) g.prev = s.base.yaw;
    let d = s.base.yaw - g.prev; if (d > 180) d -= 360; if (d < -180) d += 360; g.done += d; g.prev = s.base.yaw;
    if (Math.abs(g.deg) - Math.abs(g.done) <= 3) { drive.goal = null; drive.voiceUntil = 0; return; }
    const k = Math.max(.35, Math.min(1, (Math.abs(g.deg) - Math.abs(g.done))/40));   // slow down near the end
    drive.voice = [0, 0, Math.sign(g.deg)*k]; drive.voiceUntil = now + 400;
  } else {
    if (g.d0 == null) g.d0 = s.dist || 0;
    const done = (s.dist || 0) - g.d0;
    if (done >= Math.abs(g.m) - .01) { drive.goal = null; drive.voiceUntil = 0; return; }
    drive.voice = [Math.sign(g.m)*(g.dir === 'y' ? 0 : 1), g.dir === 'y' ? Math.sign(g.m) : 0, 0]; drive.voiceUntil = now + 400;
  }
}

// ------------------------------------------------------------------ remote control from the app (voice assistant)
// The app shows this page in an iframe and sends {arachne: cmd, ...}; answers go back as {arachne: 'reply', id, ...}.
function summary() {
  if (!last) return {connected: false};
  const s = last, mo = s.motion;
  return {connected: true, target, doing: mo.label, mode: mo.mode, gait: mo.params.gait, speed: mo.params.speed, height_mm: Math.round(s.base.z*1000),
          volt: +s.supply.v.toFixed(2), amp: +s.supply.i.toFixed(2), stable_mm: Math.round(s.margin*1000), heading: Math.round(s.base.yaw),
          walked_m: +(s.dist || 0).toFixed(2), brownouts: s.supply.brownouts, controller: $('#c-pad span').textContent};
}
addEventListener('message', e => {
  const d = e.data; if (!d || !d.arachne) return;
  const reply = x => e.source?.postMessage({arachne: 'reply', id: d.id, ...x}, '*');
  if (d.arachne === 'do') { post('/do', {what: d.what}); if (d.what === 'stop') { drive.voiceUntil = 0; drive.goal = null; post('/drive', {vx: 0, vy: 0, wz: 0}); } }
  else if (d.arachne === 'turn') drive.goal = {kind: 'turn', deg: d.deg || 90, done: 0, prev: null, deadline: performance.now() + 30000};
  else if (d.arachne === 'walk') drive.goal = {kind: 'walk', m: d.m || .3, dir: d.dir || 'x', d0: null, deadline: performance.now() + 30000};
  else if (d.arachne === 'drive') { drive.voice = [d.vx || 0, d.vy || 0, d.wz || 0]; drive.voiceUntil = performance.now() + 1000*Math.min(15, d.secs || 2); }
  else if (d.arachne === 'set') post('/set', d.values || {});
  else if (d.arachne === 'reset') post('/reset', {pose: d.pose || 'lie'});
  else if (d.arachne === 'listening') document.body.classList.toggle('listening', !!d.on);
  reply({ok: true, state: summary()});
});

// ------------------------------------------------------------------ settings
$('#b-set').addEventListener('click', () => { $('#camurl').value = store.get('camurl', ''); $('#cam3d').value = store.get('cam3d', 'follow'); $('#settings').showModal(); });
$('#settings').addEventListener('close', () => {
  const u = $('#camurl').value.trim(); if (u !== store.get('camurl', '')) { store.set('camurl', u); startCam(); }
  layout($('#camfirst').checked);
  const c = $('#cam3d').value; if (c !== store.get('cam3d', 'follow')) { store.set('cam3d', c); frame.contentWindow?.postMessage({cam: c}, '*'); }
});

// ------------------------------------------------------------------ start
layout(store.get('camfirst', false));
await checkRobot();
setTarget(store.get('target', 'sim') === 'robot' ? 'robot' : 'sim');
await countCams(); startCam(); poll();
setInterval(countCams, 15000);
