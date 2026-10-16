// ARACHNE simulation page: renders the physics world streamed by live_twin.py (/sim/stream)
// and sends commands (/sim/do, /sim/drive, /sim/set). One shared world for every viewer.
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {startPad} from '/web/pad.js';

const $ = s => document.querySelector(s), $$ = s => [...document.querySelectorAll(s)];
const post = (path, body = {}) => fetch('/sim' + path, {method: 'POST', body: JSON.stringify(body)}).then(r => r.json()).catch(() => ({}));
const store = {get(k, d) { try { const v = localStorage.getItem('arachne-sim-' + k); return v == null ? d : JSON.parse(v); } catch { return d; } },
               set(k, v) { try { localStorage.setItem('arachne-sim-' + k, JSON.stringify(v)); } catch {} }};
const LEGS = ['L1', 'L2', 'L3', 'L4', 'L5', 'L6'];
const LEG_COL = {L1: 0x6aa7ff, L2: 0x5fc38a, L3: 0xf0b54a, L4: 0xc792ea, L5: 0x4fd1c5, L6: 0xef8a6b};

// ------------------------------------------------------------------ parts
const PARTS = [
  ['gebouwd', [['chassis', 'Chassis'], ['coxa', 'Coxa (heup)'], ['femur', 'Femur (dij)'], ['tibia', 'Tibia + voet'],
               ['servos', "Servo's STS3215"], ['clamps', 'Klemmen + schijven'], ['screws', 'Schroeven + inserts']]],
  ['nog niet gebouwd', [['deck', 'Rugdek'], ['shell', 'Schaal'], ['tray', 'Batterijbak'], ['ctrlplate', 'Controllerplaat'],
                        ['battery', '3S-batterij'], ['controller', 'Controller']]],
];
const ALL = PARTS.flatMap(g => g[1].map(p => p[0]));
const BUILT = PARTS[0][1].map(p => p[0]);
const PRESETS = [['Zoals gebouwd', BUILT], ['Volledig ontwerp', ALL], ['Skelet', ['chassis', 'coxa', 'femur', 'tibia']],
                 ["Servo's", ['chassis', 'servos']]];
const OPTIONAL = PARTS[1][1].map(p => p[0]);
let visible = new Set(store.get('parts', BUILT));
let massFollow = store.get('massfollow', true);

// ------------------------------------------------------------------ three.js scene
const view = $('#view');
const renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.05;
view.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x111517, 1.6, 4.5);
const cam = new THREE.PerspectiveCamera(38, 1, .01, 30);
cam.position.set(.78, .5, .7);
const controls = new OrbitControls(cam, renderer.domElement);
controls.target.set(0, .08, 0); controls.enableDamping = true; controls.dampingFactor = .12;
controls.maxPolarAngle = Math.PI*.495; controls.minDistance = .15; controls.maxDistance = 3;

scene.add(new THREE.HemisphereLight(0xdfe9ff, 0x26302f, 1.15));
const sun = new THREE.DirectionalLight(0xffffff, 2.1);
sun.position.set(.8, 1.6, .5); sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048); sun.shadow.bias = -.0004; sun.shadow.normalBias = .01;
Object.assign(sun.shadow.camera, {left: -.5, right: .5, top: .5, bottom: -.5, near: .1, far: 4});
scene.add(sun, sun.target);
const rim = new THREE.DirectionalLight(0x8fb6ff, .6); rim.position.set(-1, .6, -1); scene.add(rim);

// MuJoCo frame (z up) inside a y-up three.js world; slope tilts the whole frame
const world = new THREE.Group(); world.rotation.x = -Math.PI/2; scene.add(world);
const mj = new THREE.Group(); world.add(mj);

// floor: soft plane + fine/major grid lines
const floor = new THREE.Mesh(new THREE.PlaneGeometry(8, 8), new THREE.MeshStandardMaterial({color: 0x232b2d, roughness: .92, metalness: 0}));
floor.receiveShadow = true; mj.add(floor);
function gridLines(size, step, color, z) {
  const pts = [];
  for (let v = -size; v <= size + 1e-9; v += step) { pts.push(-size, v, z, size, v, z, v, -size, z, v, size, z); }
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  return new THREE.LineSegments(g, new THREE.LineBasicMaterial({color, transparent: true, opacity: .9}));
}
mj.add(gridLines(4, .05, 0x2b3538, .0003), gridLines(4, .5, 0x45524f, .0005));
// axis mark at the origin (start position)
const origin = new THREE.Mesh(new THREE.RingGeometry(.012, .016, 32), new THREE.MeshBasicMaterial({color: 0x3d4a4c}));
origin.position.z = .0006; mj.add(origin);

function resize() { const w = view.clientWidth, h = view.clientHeight; renderer.setSize(w, h); cam.aspect = w/h; cam.updateProjectionMatrix(); }
addEventListener('resize', resize); resize();

// ------------------------------------------------------------------ load model
const loadBar = $('#load-bar'), loadTxt = $('#load-txt');
const meta = await (await fetch('/sim/scene.json')).json();
async function fetchBin() {
  const r = await fetch('/sim/scene.bin');
  const total = meta.groups.reduce((s, g) => Math.max(s, g.f_off + g.f_len), 0);
  const reader = r.body.getReader(); const buf = new Uint8Array(total); let n = 0;
  for (;;) {
    const {done, value} = await reader.read(); if (done) break;
    buf.set(value, n); n += value.length;
    loadBar.style.width = (100*n/total).toFixed(0) + '%'; loadTxt.textContent = `model laden… ${(n/1e6).toFixed(1)} / ${(total/1e6).toFixed(1)} MB`;
  }
  return buf.buffer;
}
const setup = $('#setup');
if (!store.get('setupDone', false) && !/nosetup|embed/.test(location.search)) setup.hidden = false;
if (location.search.includes('embed')) document.body.classList.add('embed');
const bin = await fetchBin();
const bodies = {}, mats = [], meshesByCat = {};
for (const g of meta.groups) {
  if (!bodies[g.body]) { const grp = new THREE.Group(); grp.name = g.name; bodies[g.body] = grp; mj.add(grp); }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(bin, g.v_off, g.v_len/4), 3));
  geo.setIndex(new THREE.BufferAttribute(new Uint32Array(bin, g.f_off, g.f_len/4), 1));
  geo.computeVertexNormals();
  const base = new THREE.Color(g.rgba[0], g.rgba[1], g.rgba[2]);
  const metal = g.cat === 'screws' ? .7 : g.cat === 'servos' ? .2 : .05;
  const mat = new THREE.MeshStandardMaterial({color: base.clone(), roughness: g.cat === 'screws' ? .35 : .62, metalness: metal, transparent: true, opacity: 1});
  mat.userData = {base, cat: g.cat, leg: (g.name.match(/^L\d/) || [])[0], part: g.name};
  mats.push(mat);
  const mesh = new THREE.Mesh(geo, mat); mesh.castShadow = true; mesh.receiveShadow = true;
  bodies[g.body].add(mesh); (meshesByCat[g.cat] ||= []).push(mesh);
}
// collision primitives as wireframes
const collMat = new THREE.MeshBasicMaterial({color: 0xff9a4a, wireframe: true, transparent: true, opacity: .55});
const collMeshes = [];
for (const p of meta.prims) {
  let geo;
  const s = p.size;
  if (p.type === 'box') geo = new THREE.BoxGeometry(2*s[0], 2*s[1], 2*s[2]);
  else if (p.type === 'sphere') geo = new THREE.SphereGeometry(s[0], 12, 8);
  else if (p.type === 'capsule') { geo = new THREE.CapsuleGeometry(s[0], 2*s[1], 4, 10); geo.rotateX(Math.PI/2); }
  else if (p.type === 'cylinder') { geo = new THREE.CylinderGeometry(s[0], s[0], 2*s[1], 12); geo.rotateX(Math.PI/2); }
  else if (p.type === 'ellipsoid') { geo = new THREE.SphereGeometry(1, 16, 10); geo.scale(s[0], s[1], s[2]); }
  else continue;
  const m = new THREE.Mesh(geo, collMat); m.position.set(...p.pos); m.quaternion.set(p.quat[1], p.quat[2], p.quat[3], p.quat[0]);
  m.visible = false; collMeshes.push(m);
  (bodies[p.body] || mj).add(m);
}
$('#loading').style.opacity = 0; setTimeout(() => $('#loading').remove(), 450);

// ------------------------------------------------------------------ overlays in the world
const prints = new THREE.Group(); mj.add(prints);
const printGeo = new THREE.CircleGeometry(.0075, 18);
const printList = [];
const supportMat = new THREE.MeshBasicMaterial({color: 0x5fc38a, transparent: true, opacity: .14, depthWrite: false});
const support = new THREE.Mesh(new THREE.BufferGeometry(), supportMat); support.position.z = .0009; mj.add(support);
const supportLine = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color: 0x5fc38a, transparent: true, opacity: .8}));
supportLine.position.z = .001; mj.add(supportLine);
const comDot = new THREE.Mesh(new THREE.CircleGeometry(.007, 24), new THREE.MeshBasicMaterial({color: 0xffffff}));
comDot.position.z = .0012; mj.add(comDot);
const comLineGeo = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 3));
const comLine = new THREE.Line(comLineGeo, new THREE.LineDashedMaterial({color: 0xffffff, dashSize: .006, gapSize: .004, transparent: true, opacity: .6}));
mj.add(comLine);
const trailGeo = new THREE.BufferGeometry();
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({color: 0x6aa7ff, transparent: true, opacity: .55}));
trail.position.z = .0007; mj.add(trail);
const arrows = {};
for (const L of LEGS) { const a = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), .05, LEG_COL[L], .012, .008); a.visible = false; mj.add(a); arrows[L] = a; }

// ------------------------------------------------------------------ parts UI
function partMass(cat) { return meta.parts[cat] ? `${Math.round(meta.parts[cat]*1000)} g` : ''; }
function swatch(cat) {
  const m = mats.find(m => m.userData.cat === cat); return m ? '#' + m.userData.base.getHexString() : '#555';
}
function renderParts() {
  for (const el of $$('.partlist')) {
    el.innerHTML = '';
    for (const [grp, items] of PARTS) {
      el.insertAdjacentHTML('beforeend', `<div class="partgroup">${grp}</div>`);
      for (const [cat, name] of items) {
        if (!meshesByCat[cat]) continue;
        el.insertAdjacentHTML('beforeend', `<label class="part ${visible.has(cat) ? '' : 'off'}" data-cat="${cat}"><input type="checkbox" ${visible.has(cat) ? 'checked' : ''}>
          <span class="sw" style="background:${swatch(cat)}"></span><span class="nm">${name}</span><span class="kg">${partMass(cat)}</span></label>`);
      }
    }
  }
  for (const el of $$('.presets')) {
    el.innerHTML = PRESETS.map(([n, list], i) => `<button data-preset="${i}" class="${same(list) ? 'on' : ''}">${n}</button>`).join('');
  }
  for (const c of $$('.massfollow')) c.checked = massFollow;
}
const same = list => list.length === visible.size && list.every(c => visible.has(c));
function applyParts(sendMass = true) {
  for (const [cat, list] of Object.entries(meshesByCat)) for (const m of list) m.visible = visible.has(cat);
  store.set('parts', [...visible]); store.set('massfollow', massFollow);
  renderParts();
  if (sendMass) post('/set', {hidden_mass: massFollow ? OPTIONAL.filter(c => !visible.has(c)) : []});
}
document.addEventListener('click', e => {
  const part = e.target.closest('.part');
  if (part) { e.preventDefault(); const c = part.dataset.cat; visible.has(c) ? visible.delete(c) : visible.add(c); applyParts(); return; }
  const pre = e.target.closest('[data-preset]');
  if (pre) { visible = new Set(PRESETS[+pre.dataset.preset][1]); applyParts(); }
});
for (const c of $$('.massfollow')) c.addEventListener('change', () => { massFollow = c.checked; applyParts(); });
$('#setup-go').addEventListener('click', () => { setup.hidden = true; store.set('setupDone', true); });
applyParts();

// view toggles
const tog = (id, key, def, fn) => { const el = $(id); el.checked = store.get(key, def); const f = () => { store.set(key, el.checked); fn(el.checked); }; el.addEventListener('change', f); f(); return el; };
const xray = tog('#v-xray', 'xray', false, () => {});
const coll = tog('#v-coll', 'coll', false, on => collMeshes.forEach(m => m.visible = on));
const loadCol = tog('#v-load', 'load', false, () => {});
const vPrints = tog('#v-prints', 'prints', true, on => prints.visible = on);
const vSupport = tog('#v-support', 'support', true, on => { support.visible = supportLine.visible = comDot.visible = comLine.visible = on; });
const vForce = tog('#v-force', 'force', false, () => {});
const vTrail = tog('#v-trail', 'trail', true, on => trail.visible = on);
tog('#v-shadow', 'shadow', true, on => { renderer.shadowMap.enabled = on; sun.castShadow = on; mats.forEach(m => m.needsUpdate = true); });

// ------------------------------------------------------------------ tabs, buttons
for (const b of $$('#tabs button')) b.addEventListener('click', () => {
  $$('#tabs button').forEach(x => x.classList.toggle('on', x === b));
  $$('.tab').forEach(t => t.classList.toggle('on', t.id === 'tab-' + b.dataset.tab));
  store.set('tab', b.dataset.tab);
});
$(`#tabs [data-tab="${store.get('tab', 'move')}"]`)?.click();
for (const b of $$('[data-do]')) b.addEventListener('click', () => post('/do', {what: b.dataset.do}));
for (const b of $$('[data-script]')) b.addEventListener('click', () => post('/do', {what: b.classList.contains('run') ? 'stop' : 'script:' + b.dataset.script}));
$('#b-pause').addEventListener('click', () => post('/set', {paused: !(last && last.paused)}));
$('#b-reset').addEventListener('click', () => post('/reset', {pose: 'lie'}));
$('#b-reset-lie').addEventListener('click', () => post('/reset', {pose: 'lie'}));
$('#b-reset-stand').addEventListener('click', () => post('/reset', {pose: 'stand'}));
$('#b-push').addEventListener('click', () => post('/push'));
$('#b-share').addEventListener('click', async () => {
  const info = await (await fetch('/sim/info')).json();
  const url = info.urls[0] || location.href;
  $('#share-txt').innerHTML = info.shared
    ? `Zit je op hetzelfde wifi als <b>${info.host}</b>? Scan de code of typ het adres.`
    : `Deze server luistert nu alleen op deze computer (<code>${info.host}</code>). Op de Pi staat hij open voor het hele wifi; hier start je hem zo: <code>ARACHNE_HOST=0.0.0.0 .venv/bin/python tools/live_twin.py</code>.`;
  if (window.qrcode) { const q = qrcode(0, 'M'); q.addData(url); q.make(); $('#qr').innerHTML = q.createSvgTag({cellSize: 6, margin: 2, scalable: true}); }
  $('#share-urls').innerHTML = info.urls.map(u => `<a href="${u}">${u}</a>`).join('');
  $('#share').hidden = false;
});
$('#share-close').addEventListener('click', () => $('#share').hidden = true);
$('#share').addEventListener('click', e => { if (e.target.id === 'share') $('#share').hidden = true; });
for (const b of $$('#simspeed button')) b.addEventListener('click', () => post('/set', {speed_sim: +b.dataset.s}));
for (const b of $$('#gait button')) b.addEventListener('click', () => post('/set', {gait: b.dataset.gait}));
$('#supply').innerHTML = Object.entries(meta.supplies).map(([k, n]) => `<button data-sup="${k}" title="${n}">${n.replace('labvoeding ', 'lab ').replace(' (vol)', '')}</button>`).join('');
for (const b of $$('#supply button')) b.addEventListener('click', () => post('/set', {supply: b.dataset.sup}));

// sliders: send while dragging (throttled), follow the server when someone else changes them
const FMT = {speed: v => `${Math.round(v*100)} %`, step_h: v => `${Math.round(v*1000)} mm`, cycle: v => `${(+v).toFixed(2)} s`,
             height: v => `${v > 0 ? '+' : ''}${Math.round(v*1000)} mm`, roll: deg, pitch: deg, yaw: deg,
             x: v => `${v > 0 ? '+' : ''}${Math.round(v*1000)} mm`, y: v => `${v > 0 ? '+' : ''}${Math.round(v*1000)} mm`,
             slope: v => `${v}°`, friction: v => `μ ${(+v).toFixed(2)}`};
function deg(v) { const d = v*180/Math.PI; return `${d > 0 ? '+' : ''}${d.toFixed(1)}°`; }
const active = new Set();
function bindSlider(el, key, send) {
  const out = document.querySelector(`output[data-for="${key}"]`);
  const show = () => { if (out) out.textContent = FMT[key](+el.value); };
  let t = 0;
  el.addEventListener('pointerdown', () => active.add(el)); el.addEventListener('pointerup', () => setTimeout(() => active.delete(el), 400));
  el.addEventListener('input', () => { show(); const now = performance.now(); if (now - t > 60) { t = now; send(+el.value); } });
  el.addEventListener('change', () => { show(); send(+el.value); });
  show(); return {el, show};
}
const sliders = {};
for (const el of $$('input[data-p]')) sliders[el.dataset.p] = bindSlider(el, el.dataset.p, v => post('/set', {[el.dataset.p]: v}));
for (const el of $$('input[data-b]')) sliders[el.dataset.b] = bindSlider(el, el.dataset.b, v => post('/set', {[el.dataset.b]: v}));
sliders.slope = bindSlider($('#w-slope'), 'slope', v => post('/set', {slope: v}));
sliders.friction = bindSlider($('#w-fric'), 'friction', v => post('/set', {friction: v}));
function syncSlider(key, v) { const s = sliders[key]; if (!s || active.has(s.el) || v == null) return; if (Math.abs(+s.el.value - v) > 1e-6) { s.el.value = v; s.show(); } }
$('#pose-reset').addEventListener('click', () => post('/set', {roll: 0, pitch: 0, yaw: 0, x: 0, y: 0, height: 0}));

// ------------------------------------------------------------------ driving: pad, turn bar, keyboard
const drive = {vx: 0, vy: 0, wz: 0, pad: [0, 0], turn: 0, gp: [0, 0, 0], keys: new Set()};
function padControl(el, knob, onMove, axis = 'xy') {
  let id = null;
  const set = (x, y) => { knob.style.transform = axis === 'x' ? `translateX(${x}px)` : `translate(${x}px,${y}px)`; };
  const move = e => {
    const r = el.getBoundingClientRect();
    let x = e.clientX - (r.left + r.width/2), y = e.clientY - (r.top + r.height/2);
    const R = axis === 'x' ? r.width/2 - 19 : r.width/2 - 23;
    if (axis === 'x') { y = 0; x = Math.max(-R, Math.min(R, x)); }
    else { const n = Math.hypot(x, y); if (n > R) { x *= R/n; y *= R/n; } }
    set(x, y); onMove(x/R, y/R);
  };
  el.addEventListener('pointerdown', e => { id = e.pointerId; el.setPointerCapture(id); el.classList.add('active'); move(e); });
  el.addEventListener('pointermove', e => { if (e.pointerId === id) move(e); });
  const end = e => { if (e.pointerId !== id) return; id = null; el.classList.remove('active'); set(0, 0); onMove(0, 0); };
  el.addEventListener('pointerup', end); el.addEventListener('pointercancel', end);
}
padControl($('#pad'), $('#knob'), (x, y) => { drive.pad = [x, y]; });
padControl($('#turn'), $('#turn-knob'), x => { drive.turn = x; }, 'x');
let poseHold = false;
padControl($('#posepad'), $('#poseknob'), (x, y) => {
  if (x === 0 && y === 0 && !poseHold) return;
  poseHold = !(x === 0 && y === 0);
  post('/set', {roll: x*.22, pitch: -y*.22});
});
const KEYS = {KeyW: 1, ArrowUp: 1, KeyS: 1, ArrowDown: 1, KeyA: 1, ArrowLeft: 1, KeyD: 1, ArrowRight: 1, KeyQ: 1, KeyE: 1};
addEventListener('keydown', e => {
  if (e.target.matches?.('input, textarea')) return;
  if (KEYS[e.code]) { drive.keys.add(e.code); e.preventDefault(); }
  if (e.code === 'Space') { e.preventDefault(); drive.keys.clear(); post('/do', {what: 'stop'}); post('/drive', {vx: 0, vy: 0, wz: 0}); }
  if (e.code === 'KeyP') post('/set', {paused: !(last && last.paused)});
  if (e.code === 'KeyR' && !e.metaKey && !e.ctrlKey) post('/reset', {pose: 'lie'});
});
addEventListener('keyup', e => drive.keys.delete(e.code));
addEventListener('blur', () => drive.keys.clear());
let sentZero = true;
setInterval(() => {
  const k = drive.keys, has = c => k.has(c) ? 1 : 0;
  const fx = has('KeyW') + has('ArrowUp') - has('KeyS') - has('ArrowDown') - drive.pad[1] + drive.gp[0];
  const fy = has('KeyA') - has('KeyD') - drive.pad[0] + drive.gp[1];
  const fw = has('KeyQ') + has('ArrowLeft') - has('KeyE') - has('ArrowRight') - drive.turn + drive.gp[2];
  const vx = Math.max(-1, Math.min(1, fx))*.07, vy = Math.max(-1, Math.min(1, fy))*.05, wz = Math.max(-1, Math.min(1, fw))*.5;
  const zero = !vx && !vy && !wz;
  if (zero && sentZero) return;
  post('/drive', {vx, vy, wz}); sentZero = zero;
}, 100);

// ------------------------------------------------------------------ game controller (shared with the cockpit, see pad.js)
const EMBED = new URLSearchParams(location.search).has('embed');
if (!EMBED) {                                   // embedded in the cockpit: the cockpit reads the controller itself
  const gpst = $('#gp-status');
  const gpad = startPad({post, params: () => last?.motion?.params, toast, legend: $('#gpmap'), onStatus: name => {
    gpst.classList.toggle('on', !!name);
    gpst.querySelector('span').textContent = name || 'Sluit een controller aan (USB of Bluetooth) en druk op een knop';
  }});
  setInterval(() => { drive.gp = gpad.drive; }, 50);
}

// ------------------------------------------------------------------ gait diagram
const GAITS = {tripod: [{L1: 0, L3: 0, L5: 0, L2: .5, L4: .5, L6: .5}, .5],
               ripple: [{L3: 0, L2: 1/3, L1: 2/3, L4: .5, L5: 5/6, L6: 1/6}, 2/3],
               wave: [{L3: 0, L2: 1/6, L1: 2/6, L4: 3/6, L5: 4/6, L6: 5/6}, 5/6]};
let gaitShown = null; const gaitCur = [];
function drawGait(name) {
  if (gaitShown === name) return; gaitShown = name;
  const [ph, duty] = GAITS[name]; const el = $('#gaitviz'); el.innerHTML = ''; gaitCur.length = 0;
  for (const L of LEGS) {
    const a = (ph[L] + duty) % 1, w = 1 - duty;
    const segs = a + w <= 1 ? [[a, w]] : [[a, 1 - a], [0, a + w - 1]];
    el.insertAdjacentHTML('beforeend', `<div class="gl"><span>${L}</span><div class="tr">${segs.map(([s, w]) => `<i class="sw" style="left:${s*100}%;width:${w*100}%;background:#${LEG_COL[L].toString(16).padStart(6, '0')}"></i>`).join('')}<i class="cur"></i></div></div>`);
  }
  gaitCur.push(...el.querySelectorAll('.cur'));
}
let gaitPhase = 0;

// ------------------------------------------------------------------ measure panel
$('#heat').innerHTML = '<span></span><span class="h">yaw</span><span class="h">hip</span><span class="h">knie</span>' +
  LEGS.map(L => `<span class="lg">${L}</span>` + ['yaw', 'hip', 'knee'].map(p => `<span class="c" id="h-${L}_${p}"><b>–</b><small>–</small></span>`).join('')).join('');
$('#feet').innerHTML = LEGS.map(L => `<div class="f" id="f-${L}"><span>${L}</span><div class="b"><i style="height:0;background:#${LEG_COL[L].toString(16).padStart(6, '0')}"></i></div><small>0 N</small></div>`).join('');
const hist = {i: [], v: [], m: []};
function chart(cv, series, opts) {
  const dpr = devicePixelRatio || 1, W = cv.clientWidth, H = cv.height;
  if (cv.width !== W*dpr) { cv.width = W*dpr; cv.style.height = H + 'px'; cv.height = H*dpr; cv.dataset.h = H; }
  const h = +cv.dataset.h || H, ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, h);
  ctx.font = '10px ui-monospace,Menlo,monospace'; ctx.fillStyle = '#5d6865'; ctx.strokeStyle = '#242c2f'; ctx.lineWidth = 1;
  for (let k = 1; k < 4; k++) { const y = Math.round(h*k/4) + .5; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke(); }
  for (const s of series) {
    const n = s.data.length; if (n < 2) continue;
    if (s.limit != null) {
      const y = h - (s.limit - s.min)/(s.max - s.min)*h; ctx.setLineDash([3, 3]); ctx.strokeStyle = s.limitColor; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke(); ctx.setLineDash([]);
    }
    ctx.strokeStyle = s.color; ctx.lineWidth = 1.6; ctx.beginPath();
    s.data.forEach((v, i) => { const x = W*(i + 600 - n)/600, y = h - 2 - (Math.max(s.min, Math.min(s.max, v)) - s.min)/(s.max - s.min)*(h - 4); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke(); ctx.lineWidth = 1;
    if (s.label) { ctx.fillStyle = s.color; ctx.fillText(s.label, s.labelX ?? 6, s.labelY ?? 12); }
    if (s.zero != null) { const y = h - 2 - (s.zero - s.min)/(s.max - s.min)*(h - 4); ctx.strokeStyle = '#3a4548'; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke(); }
  }
}

// ------------------------------------------------------------------ stream
let last = null, prevContact = {}, shownEvents = new Set(), camMode = new URLSearchParams(location.search).get('cam') || store.get('cam', 'orbit');
addEventListener('message', e => { if (e.data?.cam) setCam(e.data.cam); });   // the cockpit switches the camera
for (const b of $$('#cam button')) { b.classList.toggle('on', b.dataset.cam === camMode); b.addEventListener('click', () => setCam(b.dataset.cam)); }
function setCam(m) {
  camMode = m; store.set('cam', m); $$('#cam button').forEach(b => b.classList.toggle('on', b.dataset.cam === m));
  if (m === 'top') { const t = controls.target; cam.position.set(t.x + .001, t.y + 1.1, t.z + .001); }
  if (m !== 'orbit' && last) baseFollow = null;
}
let baseFollow = null;
const tmpV = new THREE.Vector3();
function toast(msg, level) {
  const el = document.createElement('div'); el.className = 'toast ' + (level || ''); el.textContent = msg;
  $('#toasts').prepend(el); setTimeout(() => { el.style.opacity = 0; setTimeout(() => el.remove(), 450); }, level === 'bad' ? 9000 : 4000);
}
function fmt(v, d = 1) { return (v ?? 0).toFixed(d); }

function apply(s) {
  const first = !last; last = s;
  // bodies
  s.bodies.forEach((b, i) => { const g = bodies[i]; if (!g) return; g.position.set(b[0], b[1], b[2]); g.quaternion.set(b[4], b[5], b[6], b[3]); });
  // slope tilts the visual world (gravity is rotated in the physics)
  mj.rotation.y = -s.slope*Math.PI/180;
  // materials: x-ray / load colouring
  const xr = xray.checked, lc = loadCol.checked;
  for (const m of mats) {
    const u = m.userData; let op = 1;
    if (xr && u.cat !== 'servos') op = u.cat === 'screws' ? .25 : .22;
    m.opacity = op; m.depthWrite = op > .5;
    let l = 0;
    if (lc && u.leg) { const part = u.part.split('_')[1]; const j = `${u.leg}_${{COXA: 'yaw', FEMUR: 'hip', TIBIA: 'knee'}[part]}`; l = Math.min(1, (s.joints[j]?.load ?? 0)/700); }
    m.color.copy(u.base); if (l > .02) m.color.lerp(new THREE.Color(0xef4b3b), l);
  }
  // support polygon, COM
  const hull = s.support;
  if (hull.length >= 3) {
    const shape = new THREE.Shape(hull.map(p => new THREE.Vector2(p[0], p[1])));
    support.geometry.dispose(); support.geometry = new THREE.ShapeGeometry(shape);
    supportLine.geometry.dispose(); supportLine.geometry = new THREE.BufferGeometry().setFromPoints(hull.map(p => new THREE.Vector3(p[0], p[1], 0)));
    support.visible = supportLine.visible = vSupport.checked;
  } else { support.visible = supportLine.visible = false; }
  const mc = s.margin > .02 ? 0x5fc38a : s.margin > .008 ? 0xf0b54a : 0xef6b5b;
  supportMat.color.setHex(mc); supportLine.material.color.setHex(mc); comDot.material.color.setHex(mc);
  comDot.position.set(s.com[0], s.com[1], .0012);
  comLineGeo.attributes.position.array.set([s.com[0], s.com[1], .001, s.com[0], s.com[1], s.com[2]]); comLineGeo.attributes.position.needsUpdate = true; comLine.computeLineDistances();
  // footprints + force arrows
  const now = performance.now();
  for (const L of LEGS) {
    const f = s.feet[L], down = f.f > .3;
    if (down && !prevContact[L]) {
      const m = new THREE.Mesh(printGeo, new THREE.MeshBasicMaterial({color: LEG_COL[L], transparent: true, opacity: .7, depthWrite: false}));
      m.position.set(f.p[0], f.p[1], .0008); m.userData.t = now; prints.add(m); printList.push(m);
      if (printList.length > 180) { const o = printList.shift(); prints.remove(o); o.material.dispose(); }
    }
    prevContact[L] = down;
    const a = arrows[L]; a.visible = vForce.checked && down;
    if (a.visible) { a.position.set(f.p[0], f.p[1], f.p[2]); a.setLength(Math.min(.12, .004*f.f + .01), .012, .008); }
  }
  // body trail
  if (s.trail.length > 1) { trailGeo.dispose(); trailGeo.setFromPoints(s.trail.map(p => new THREE.Vector3(p[0], p[1], 0))); }
  // header + gauges
  const mo = s.motion;
  $('#c-mode').textContent = s.paused ? 'gepauzeerd' : mo.label; $('#c-mode').classList.toggle('live', mo.mode !== 'idle' || !!s.script);
  $('#c-time').textContent = `${fmt(s.t)} s`; $('#c-rtf').textContent = `rtf ${fmt(s.rtf, 2)}`;
  $('#c-view').textContent = `${s.viewers} ${s.viewers === 1 ? 'kijker' : 'kijkers'}`;
  $('#b-pause').textContent = s.paused ? '▶' : '❚❚';
  const b = s.base, sp = Math.hypot(b.vx, b.vy)*100;
  $('#g-speed').textContent = fmt(sp); $('#g-head').textContent = `${Math.round(b.yaw)}°`;
  $('#g-h').textContent = Math.round(b.z*1000);
  const gm = $('#g-margin'); gm.textContent = s.support.length >= 3 ? Math.round(s.margin*1000) : '–';
  gm.className = s.support.length < 3 ? 'bad' : s.margin > .02 ? 'ok' : s.margin > .008 ? 'warn' : 'bad';
  const gv = $('#g-v'); gv.textContent = fmt(s.supply.v); gv.className = s.supply.v < 9 ? 'bad' : s.supply.v < 10.8 ? 'warn' : '';
  const ga = $('#g-a'); ga.textContent = fmt(s.supply.i); ga.className = s.supply.i > s.supply.ilim*.95 ? 'bad' : s.supply.i > s.supply.ilim*.7 ? 'warn' : '';
  // buttons state
  $$('#gait button').forEach(x => x.classList.toggle('on', x.dataset.gait === mo.params.gait));
  $$('#simspeed button').forEach(x => x.classList.toggle('on', Math.abs(+x.dataset.s - s.speed) < 1e-6));
  $$('#supply button').forEach(x => x.classList.toggle('on', x.dataset.sup === s.supply.key));
  $$('[data-script]').forEach(x => x.classList.toggle('run', x.dataset.script === s.script));
  $$('.grid4 [data-do]').forEach(x => x.classList.toggle('run', mo.mode === 'show' && mo.label === x.textContent.toLowerCase()));
  for (const k of ['speed', 'step_h', 'cycle', 'height']) syncSlider(k, mo.params[k]);
  for (const k of ['roll', 'pitch', 'yaw', 'x', 'y']) syncSlider(k, mo.body[k]);
  syncSlider('slope', s.slope); syncSlider('friction', s.friction);
  $('#mass').textContent = `${(s.mass*1000).toFixed(0)} g`;
  $('#setup-mass').textContent = `massa in de fysica: ${(s.mass*1000).toFixed(0)} g`;
  drawGait(mo.params.gait);
  if (mo.label === 'lopen') gaitPhase = (gaitPhase + 1/30/mo.params.cycle) % 1;
  gaitCur.forEach(c => c.style.left = (gaitPhase*100) + '%');
  // measure tab
  hist.i.push(s.supply.i); hist.v.push(s.supply.v); hist.m.push(s.support.length >= 3 ? s.margin*1000 : -10);
  for (const k in hist) if (hist[k].length > 600) hist[k].shift();
  for (const [j, d] of Object.entries(s.joints)) {
    const el = document.getElementById('h-' + j); if (!el) continue;
    const pct = d.load/10; el.classList.toggle('off', !d.torque);
    el.firstChild.textContent = `${Math.round(pct)}%`; el.lastChild.textContent = `${d.amp.toFixed(2)}A`;
    const c = Math.min(1, pct/80); el.style.background = `rgba(${Math.round(60 + 179*c)},${Math.round(90 + 20*c)},${Math.round(80 - 20*c)},${.15 + .55*c})`;
  }
  for (const L of LEGS) {
    const f = s.feet[L], el = document.getElementById('f-' + L);
    el.classList.toggle('up', f.f <= .3); el.querySelector('i').style.height = Math.min(40, f.f*2.2) + 'px'; el.querySelector('small').textContent = `${f.f.toFixed(1)} N`;
  }
  $('#sup-name').textContent = s.supply.name;
  $('#m-peak').textContent = fmt(s.supply.peak); $('#m-brown').textContent = s.supply.brownouts; $('#m-dist').textContent = Math.round(s.dist*100);
  // events
  for (const e of s.events) { const k = e.t + e.msg; if (shownEvents.has(k)) continue; shownEvents.add(k); if (!first) toast(e.msg, e.level); }
}
setInterval(() => {
  if (!last || !$('#tab-meas').classList.contains('on')) return;
  const lim = last.supply.ilim;
  chart($('#ch-power'), [
    {data: hist.v, min: 0, max: 13, color: '#f0b54a', label: `${fmt(last.supply.v)} V`, labelX: 60},
    {data: hist.i, min: 0, max: Math.max(lim*1.15, 1), color: '#6aa7ff', limit: lim, limitColor: '#ef6b5b', label: `${fmt(last.supply.i)} A`}]);
  chart($('#ch-margin'), [{data: hist.m, min: -10, max: 60, color: '#5fc38a', zero: 0, label: `${last.support.length >= 3 ? Math.round(last.margin*1000) : '–'} mm`}]);
}, 100);

const es = new EventSource('/sim/stream');
es.onmessage = e => apply(JSON.parse(e.data));
es.onerror = () => { $('#c-mode').textContent = 'verbinding weg…'; };

// ------------------------------------------------------------------ render loop
renderer.setAnimationLoop(() => {
  const n = performance.now();
  for (const m of printList) m.material.opacity = Math.max(0, .7 - (n - m.userData.t)/30000);
  if (last && camMode !== 'orbit') {
    const b = last.bodies[1]; mj.localToWorld(tmpV.set(b[0], b[1], camMode === 'top' ? 0 : .06));
    if (baseFollow) { const d = tmpV.clone().sub(baseFollow); cam.position.add(d); controls.target.add(d); }
    else { const d = tmpV.clone().sub(controls.target); cam.position.add(d); controls.target.add(d); }
    baseFollow = tmpV.clone();
  }
  if (last) { const b = last.bodies[1]; mj.localToWorld(tmpV.set(b[0], b[1], 0)); sun.target.position.copy(tmpV); sun.position.set(tmpV.x + .8, 1.6, tmpV.z + .5); }
  controls.update(); renderer.render(scene, cam);
});
