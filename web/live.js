'use strict';
/* GARUDA OPS live mission control — talks to the drone bridge in server.py
   POST /api/command  {action,...}  ·  GET /api/telemetry?since=N  ·  GET /stream/<view> (MJPEG) */
const $ = id => document.getElementById(id);
const safe = s => String(s ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const fmtT = t => `${String(Math.floor(t / 60)).padStart(2, '0')}:${String(Math.floor(t % 60)).padStart(2, '0')}`;
let T = null, lastSeq = 0, events = [], reviews = {}, view = 'detection', online = false, cfgSynced = false;
let draftRoute = [], focusEvt = null, wasRec = false;

// ------------------------------------------------------------------ comms
async function cmd(body, quiet) {
  try {
    const r = await fetch('/api/command', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const j = await r.json();
    if (!j.ok) throw Error(j.error || 'rejected');
    if (!quiet) toast(label(body));
  } catch (e) { toast('Command failed: ' + e.message, true); }
}
function label(b) {
  if (b.action === 'mode') return 'Mode → ' + b.mode;
  if (b.action === 'goto') return `Flying to (${b.x.toFixed(0)}, ${b.y.toFixed(0)})`;
  if (b.action === 'set') return 'Updated ' + Object.keys(b.params).join(', ');
  return ({takeoff: 'Take-off commanded', land: 'Landing commanded', estop: 'EMERGENCY STOP', hold: 'Holding',
           reset: 'Simulation reset', record: 'Recording ' + b.state, patrol_set: 'Patrol route updated', clear_map: 'Map cleared'})[b.action] || b.action;
}
let toastTimer;
function toast(msg, err) {
  const t = $('toast'); t.textContent = msg; t.className = 'toast show' + (err ? ' err' : '');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.className = 'toast', msg.length > 60 ? 6000 : 1800);
}

async function poll() {
  try {
    const r = await fetch('/api/telemetry?since=' + lastSeq, {cache: 'no-store'});
    if (!r.ok) throw Error(r.status);
    T = await r.json();
    if (!online) { online = true; setView(view); }
    if (T.events.length) { events.push(...T.events); events = events.slice(-300); lastSeq = T.last_seq; renderIncidents(); }
    render();
  } catch (e) {
    online = false;
    $('linkText').textContent = 'BRIDGE OFFLINE'; $('linkBox').classList.add('off');
    $('bridgeText').textContent = 'Lost connection to server.py — is it still running?';
  }
  setTimeout(poll, 250);
}

// ------------------------------------------------------------------ render
const nf = (v, d = 1) => v == null ? '∞' : Number(v).toFixed(d);
function render() {
  const t = T;
  $('linkText').textContent = t.backend + ' LINKED'; $('linkBox').classList.remove('off');
  $('backendPill').textContent = '● ' + t.backend;
  $('modePill').textContent = t.mode;
  $('bridgeText').textContent = (t.backend.startsWith('MOCK')
    ? 'Built-in MOCK simulator (AirSim not detected). Start AirSim and restart server.py to fly the real simulation. '
    : 'Connected to Microsoft AirSim — commands below fly the drone in the simulator. ')
    + `Detections: ${t.detector}.` + (t.error ? ` ⚠ ${t.error}` : '');
  $('detPill').textContent = t.detector;
  $('mAlt').textContent = nf(t.pos.alt); $('mAltFoot').textContent = `target ${nf(t.cfg.altitude)} m`;
  $('mSpd').textContent = nf(t.vel.speed);
  $('mHdg').textContent = Math.round(t.heading) + '°'; $('mZone').textContent = t.zone;
  const dc = t.sectors.dc;
  $('mClr').textContent = dc == null ? 'CLEAR' : dc.toFixed(1) + ' m';
  $('mClr').style.color = t.sectors.center ? '#ff8f8f' : (dc != null && dc < t.cfg.look_ahead_threshold * 2 ? '#f3d48d' : '#e7edf7');
  $('mPpl').textContent = t.people; $('mVeh').textContent = `vehicles ${t.vehicles} · ${t.people_near} within ${t.cfg.crowd_range} m`;
  $('mCrowd').textContent = t.crowd_level;
  $('mCrowd').style.color = {LOW: '#59d8b5', MODERATE: '#f3d48d', HIGH: '#f2b06d', 'VERY HIGH': '#ff7d7d'}[t.crowd_level];
  $('armPill').textContent = t.armed ? (t.collided ? 'COLLISION' : 'AIRBORNE') : 'ON GROUND';
  $('armPill').className = 'pill ' + (t.armed ? 'cyan' : 'orange');
  $('fpsPill').textContent = `◉ ${t.fps} FPS`;
  $('hud').textContent = `ALT ${nf(t.pos.alt)} m | SPD ${nf(t.vel.speed)} m/s | HDG ${Math.round(t.heading)}° | ${t.gps.lat.toFixed(5)}, ${t.gps.lon.toFixed(5)}`;
  $('hudMode').textContent = `${t.mode}${t.target ? ' → WP ' + t.target.map(v => v.toFixed(0)).join(',') : ''}`;
  for (const [id, k, d] of [['secL', 'left', 'dl'], ['secC', 'center', 'dc'], ['secR', 'right', 'dr']]) {
    const el = $(id), v = t.sectors[d], max = 30;
    el.classList.toggle('blocked', t.sectors[k]);
    el.querySelector('b').textContent = v == null ? 'clear' : v.toFixed(1) + ' m';
    el.querySelector('i').style.width = (v == null ? 100 : Math.min(100, v / max * 100)) + '%';
  }
  const dEl = $('decision'); dEl.textContent = t.decision;
  dEl.parentElement.className = 'decision' + (/BLOCKED|COLLISION/.test(t.decision) ? ' bad' : /ADJUST|BIAS|ASSIST/.test(t.decision) ? ' warn' : '');
  document.querySelectorAll('#modeSeg button').forEach(b => b.classList.toggle('on', b.dataset.mode === t.mode));
  $('btnTakeoff').disabled = t.armed && t.mode !== 'LANDING';
  $('btnLand').disabled = !t.armed;
  // recording
  const rec = t.recording;
  $('recBadge').hidden = !rec; $('recBtn').classList.toggle('on', !!rec);
  $('recBtn').textContent = rec ? '■ Stop recording' : '● Record flight';
  if (rec) $('recTime').textContent = fmtT(rec.seconds);
  if (wasRec && !rec && t.last_manifest) {
    const m = t.last_manifest;
    toast(`Recording saved · ${m.frames} frames · SHA-256 ${String(m.video_sha256 || 'n/a').slice(0, 16)}…`);
  }
  wasRec = !!rec;
  if (!cfgSynced) syncCfg(t.cfg);
  // detections & log
  $('detBody').innerHTML = t.detections.length ? t.detections.map(d =>
    `<tr><td class="${safe(d.label)}">${safe(d.label.toUpperCase())}</td><td>${d.conf.toFixed(2)}</td><td>${d.dist == null ? '—' : d.dist.toFixed(1) + ' m'}</td><td>${safe(d.src)}</td></tr>`).join('')
    : '<tr><td colspan="4" style="color:#6c8296;padding:20px 12px">No detections in frame</td></tr>';
  const con = $('console'), atBottom = con.scrollTop + con.clientHeight >= con.scrollHeight - 8;
  con.innerHTML = t.log.map(l => `<div class="${safe(l[1])}">${safe(l[0])}  ${safe(l[1].padEnd(5))}  ${safe(l[2])}</div>`).join('');
  if (atBottom) con.scrollTop = con.scrollHeight;
  // summary
  const s = t.series;
  if (s.length) {
    const recent = s.filter(r => r[0] > s.at(-1)[0] - 120);
    const mins = recent.map(r => r[1]).filter(v => v != null);
    $('sMin').textContent = mins.length ? Math.min(...mins).toFixed(1) + ' m' : 'clear';
    $('sPeak').textContent = Math.max(...recent.map(r => r[2]));
  }
  $('sLoop').textContent = `${t.loop_ms} ms`;
  drawMap(); drawChart();
  renderAlerts(t);
}

// ------------------------------------------------------------------ emergency response
let shownAlert = null;
function beep() {
  try { const a = new (window.AudioContext || window.webkitAudioContext)(), o = a.createOscillator(), g = a.createGain();
    o.type = 'square'; o.frequency.value = 880; g.gain.value = 0.05; o.connect(g); g.connect(a.destination);
    o.start(); setTimeout(() => { o.frequency.value = 660; }, 180); setTimeout(() => { o.stop(); a.close(); }, 420); } catch (e) {}
}
function renderAlerts(t) {
  const near = t.people_near, lim = t.cfg.alert_limit;
  $('erGps').textContent = `${t.gps.lat.toFixed(6)}° N, ${t.gps.lon.toFixed(6)}° E`;
  $('erMaps').href = t.gps.maps_url;
  $('erNow').textContent = near; $('erNowFoot').textContent = `limit ${lim} · ${t.cfg.alert_enabled ? 'prompts ON' : 'prompts OFF'}`;
  $('erNow').parentElement.classList.toggle('over', near >= lim);
  const pa = t.pending_alert;
  $('erPill').textContent = pa ? '⚠ DECISION NEEDED' : (t.cfg.alert_enabled ? 'MONITORING' : 'PROMPTS OFF');
  $('erPill').className = 'pill' + (pa ? ' alarm' : '');
  $('erBody').innerHTML = t.alerts.length ? t.alerts.slice().reverse().map(a =>
    `<tr><td>${safe(a.id)}${a.test ? ' <small>(test)</small>' : ''}</td><td>${safe(a.created.slice(11, 23))}</td><td>${a.persons} / ${a.limit}</td>
     <td>${safe(a.status === 'DECLINED' ? '—' : (a.station || '—'))}</td>
     <td class="st-${safe(a.status)}" title="${safe(a.delivery || '')}${(a.notify || []).map(n => ' | ' + n.channel + ' ' + n.to + ': ' + n.status).join('')}">${safe(a.status)}${a.delivery_sec != null ? ' · ' + a.delivery_sec + ' s' : (a.decided ? ' · ' + a.response_sec + ' s' : '')}${mailTag(a)}${contactLinks(a)}</td></tr>`).join('')
    : '<tr><td colspan="5" style="color:#6c8296;padding:18px 12px">No alerts yet. A prompt appears when the crowd near the drone exceeds the limit for 3 s.</td></tr>';
  renderStations(t);
  if (t.delivery) $('erPill').textContent = '✈ DELIVERING ' + t.delivery.alert;
  const m = $('alertModal');
  if (!pa) { m.hidden = true; shownAlert = null; return; }
  if (shownAlert !== pa.id) {
    shownAlert = pa.id; m.hidden = false; beep();
    $('alertYes').disabled = $('alertNo').disabled = false; $('alertWait').textContent = 'Waiting for operator decision…';
    $('alertTest').hidden = !pa.test;
    $('alertMsg').innerHTML = `Crowd was too much in <b>${safe(pa.zone)}</b>: <b>${pa.persons}</b> persons detected near the drone (limit ${pa.limit}).`;
    $('alertId').textContent = pa.id; $('alertTime').textContent = pa.created;
    $('alertZone').textContent = `${pa.zone} · alt ${pa.alt} m`;
    $('alertGps').textContent = `${pa.latitude.toFixed(6)}, ${pa.longitude.toFixed(6)}`;
    $('alertMaps').href = pa.maps_url;
    $('alertSnap').removeAttribute('src');
    const live = id => (T.stations || []).find(s => s.id === id) || {};
    pa.station_options.forEach(o => { const s = live(o.id); o.email = s.email || ''; o.phone = s.phone || ''; });
    const ok = o => o.email || o.phone, first = pa.station_options.find(ok);
    pickedStation = first ? first.id : null;
    $('stationPick').innerHTML = pa.station_options.map((o, i) => `<label data-id="${safe(o.id)}" class="${o.id === pickedStation ? 'sel' : ''} ${ok(o) ? '' : 'disabled'}">
      <input type="radio" name="ps" value="${safe(o.id)}" ${o.id === pickedStation ? 'checked' : ''} ${ok(o) ? '' : 'disabled'}>
      <span><b>${safe(o.name)}</b>${i ? '' : ' <small>(nearest)</small>'}<br><small>${o.dist_m.toFixed(0)} m away · radio range ${o.range_m.toFixed(0)} m</small>
      <br><small class="contact">${ok(o) ? (o.email ? '✉ ' + safe(o.email) : '') + (o.email && o.phone ? ' · ' : '') + (o.phone ? '☎ ' + safe(o.phone) : '')
        : '⚠ no contact — add an e-mail or phone in the station card'}</small></span>
      <span class="${o.in_range ? 'in' : 'out'}">${o.in_range ? 'IN RANGE · instant' : `OUT OF RANGE · fly ${o.fly_m.toFixed(0)} m+ (≥${o.eta_sec} s)`}</span></label>`).join('');
    updateYes(pa);
  }
  if (!$('alertSnap').getAttribute('src') && pa.snapshot_sha256) $('alertSnap').src = `/api/alert/${pa.id}.jpg?h=${pa.snapshot_sha256.slice(0, 8)}`;
}
let pickedStation = null;
function contactLinks(a) {
  if (a.status !== 'DELIVERED') return '';
  return (a.station_phone ? ` <a href="#" data-open="wa" data-id="${safe(a.id)}">WhatsApp ↗</a>` : '') +
         (a.station_email && !(T && T.email_configured) ? ` <a href="#" data-open="mail" data-id="${safe(a.id)}">E-mail ↗</a>` : '');
}
function mailTag(a) {
  const n = (a.notify || []).find(x => x.channel === 'email');
  if (!n) return (a.notify || []).length ? '<br><small class="mail-OUT">☎ WhatsApp</small>' : '';
  if (n.status.startsWith('OPEN IN MAIL')) return '<br><small class="mail-OUT">✉ mail app</small>';
  const cls = n.status === 'SENT' ? 'SENT' : n.status.startsWith('FAILED') || n.status.startsWith('NOT') ? 'FAIL' : n.status.startsWith('SENDING') ? 'WAIT' : 'OUT';
  const txt = n.status === 'SENT' ? '✉ e-mailed' : cls === 'FAIL' ? '✉ failed' : cls === 'WAIT' ? '✉ sending…' : '✉ outbox';
  return `<br><small class="mail-${cls}">${txt}</small>`;
}
// ---- station contact boxes
const EMAIL_RE = /^[^@\s]+@[^@\s]+\.[A-Za-z]{2,}$/;
function normPhone(raw) {
  let d = String(raw || '').replace(/\D/g, '');
  if (!d) return '';
  if (d.length === 10) d = '91' + d; else if (d.length === 11 && d[0] === '0') d = '91' + d.slice(1);
  return d.length >= 10 && d.length <= 15 ? '+' + d : null;     // null = invalid
}
function renderStations(t) {
  const box = $('erStations'), list = t.stations || [], ids = list.map(p => p.id).join();
  if (box.dataset.ids !== ids) {
    box.dataset.ids = ids;
    box.innerHTML = list.map(p => `<div class="ps-card" data-id="${safe(p.id)}">
      <b class="ps-name"></b><div class="ps-dist"></div><small class="ps-gps"></small>
      <div class="ps-form">
        <label>✉ E-mail<input type="email" data-f="email" placeholder="abc@gmail.com" autocomplete="off"></label>
        <label>☎ Phone<input type="tel" data-f="phone" placeholder="9876543210" autocomplete="off"></label>
        <div class="ps-actions"><button class="ghost" data-save>Save contact</button><span class="ps-msg"></span></div>
      </div></div>`).join('');
  }
  for (const p of list) {
    const card = box.querySelector(`[data-id="${p.id}"]`); if (!card) continue;
    card.querySelector('.ps-name').textContent = '🛡 ' + p.name;
    card.querySelector('.ps-dist').innerHTML = `${p.dist_m.toFixed(0)} m from drone · range ${p.range_m.toFixed(0)} m · <span class="${p.in_range ? 'in' : 'out'}">${p.in_range ? 'IN RADIO RANGE' : 'OUT OF RANGE'}</span>`;
    card.querySelector('.ps-gps').textContent = `GPS ${p.latitude.toFixed(6)}, ${p.longitude.toFixed(6)}`;
    for (const inp of card.querySelectorAll('input')) {
      if (document.activeElement !== inp && !inp.dataset.dirty) inp.value = p[inp.dataset.f] || '';
    }
    card.classList.toggle('no-contact', !p.email && !p.phone);
    const msg = card.querySelector('.ps-msg');
    if (!msg.dataset.hold) msg.textContent = (!p.email && !p.phone) ? '⚠ add an e-mail or phone to receive alerts'
      : (p.email ? (t.email_configured ? '✉ e-mail sent automatically' : '✉ opens your mail app') : '') + (p.phone ? (p.email ? ' · ' : '') + '☎ WhatsApp message' : '');
  }
}
function saveStation(card) {
  const email = card.querySelector('[data-f=email]').value.trim(), rawPhone = card.querySelector('[data-f=phone]').value.trim();
  const msg = card.querySelector('.ps-msg'), phone = normPhone(rawPhone);
  const fail = text => { msg.textContent = '⚠ ' + text; msg.dataset.hold = 1; msg.className = 'ps-msg err'; setTimeout(() => { delete msg.dataset.hold; msg.className = 'ps-msg'; }, 3500); };
  if (!email && !rawPhone) return fail('Enter an e-mail or a phone number');
  if (email && !EMAIL_RE.test(email)) return fail('E-mail looks wrong (example: abc@gmail.com)');
  if (rawPhone && phone === null) return fail('Phone looks wrong (example: 9876543210)');
  card.querySelectorAll('input').forEach(i => delete i.dataset.dirty);
  cmd({action: 'station_update', id: card.dataset.id, email, phone: phone || ''}, true).then(() => {
    msg.textContent = '✓ Saved'; msg.dataset.hold = 1; msg.className = 'ps-msg ok';
    setTimeout(() => { delete msg.dataset.hold; msg.className = 'ps-msg'; }, 2500);
  });
}
$('erStations').addEventListener('input', e => { if (e.target.dataset.f) e.target.dataset.dirty = 1; });
$('erStations').addEventListener('keydown', e => { if (e.key === 'Enter' && e.target.dataset.f) saveStation(e.target.closest('.ps-card')); });
$('erStations').onclick = e => { const b = e.target.closest('[data-save]'); if (b) saveStation(b.closest('.ps-card')); };

// ---- alert message for e-mail / WhatsApp
function alertText(a) {
  return `🚨 CROWD ALERT ${a.id}${a.test ? ' (TEST)' : ''}\n${a.persons} persons detected near the drone in ${a.zone} (limit ${a.limit}).\n` +
    `Detected: ${a.created}\nGPS: ${a.latitude.toFixed(6)}, ${a.longitude.toFixed(6)}\nGoogle Maps: ${a.maps_url}\n` +
    `Drone altitude: ${a.alt} m\n— GARUDA OPS drone dashboard`;
}
function waLink(phone, a) { return `https://wa.me/${phone.replace(/\D/g, '')}?text=${encodeURIComponent(alertText(a))}`; }
function mailLink(email, a) {
  return `mailto:${email}?subject=${encodeURIComponent(`CROWD ALERT ${a.id}: ${a.persons} persons at ${a.zone}`)}&body=${encodeURIComponent(alertText(a))}`;
}
function openContacts(a, st) {           // called inside the operator's click, so the browser allows it
  if (st.phone) window.open(waLink(st.phone, a), '_blank', 'noopener');
  if (st.email && !(T && T.email_configured)) { const l = document.createElement('a'); l.href = mailLink(st.email, a); l.click(); }
}
$('erBody').addEventListener('click', e => {
  const b = e.target.closest('[data-open]'); if (!b || !T) return;
  const a = T.alerts.find(x => x.id === b.dataset.id); if (!a) return;
  const st = {email: a.station_email, phone: a.station_phone};
  if (b.dataset.open === 'wa') window.open(waLink(st.phone, a), '_blank', 'noopener');
  else { const l = document.createElement('a'); l.href = mailLink(st.email, a); l.click(); }
});
$('erCopy').onclick = e => { e.preventDefault(); if (T) navigator.clipboard.writeText(`${T.gps.lat.toFixed(6)}, ${T.gps.lon.toFixed(6)}`).then(() => toast('GPS copied')); };
function updateYes(pa) {
  const o = (pa.station_options || []).find(x => x.id === pickedStation);
  $('alertYes').disabled = !o;
  $('alertWait').textContent = o ? 'Waiting for operator decision…' : '⚠ No station has a contact. Add an e-mail or phone in the Emergency Response panel.';
  $('alertYes').textContent = !o ? '✔ YES, SEND ALERT' : o.in_range ? '✔ YES, SEND ALERT' : '✔ YES, FLY & DELIVER';
}
$('stationPick').onchange = e => {
  pickedStation = e.target.value;
  document.querySelectorAll('#stationPick label').forEach(l => l.classList.toggle('sel', l.dataset.id === pickedStation));
  if (T && T.pending_alert) updateYes(T.pending_alert);
};
function decide(decision) {
  if (!shownAlert) return;
  if (decision === 'send' && T && T.pending_alert) {
    const pa = T.pending_alert, o = (pa.station_options || []).find(x => x.id === pickedStation);
    if (!o) return;
    if (o.in_range) openContacts(pa, o);     // out of range: links appear in the table once delivered
  }
  $('alertYes').disabled = $('alertNo').disabled = true; $('alertWait').textContent = 'Sending…';
  cmd({action: 'alert_decision', id: shownAlert, decision, station: pickedStation}, true)
    .then(() => toast(decision === 'send' ? '🚨 Alert sent to ' + (($('stationPick').querySelector('label.sel b') || {}).textContent || 'police') : 'Alert declined'));
}
$('alertYes').onclick = () => decide('send');
$('alertNo').onclick = () => decide('decline');
$('erTest').onclick = () => { if (T && !T.armed) return toast('Take off first', true); cmd({action: 'alert_test'}, true); };

function syncCfg(c) {
  document.querySelectorAll('[data-param]').forEach(el => {
    const k = el.dataset.param;
    if (!(k in c)) return;
    if (el.type === 'checkbox') el.checked = !!c[k];
    else { el.value = c[k]; showVal(el); }
  });
  cfgSynced = true;
}
function showVal(el) { const o = el.parentElement.querySelector('output'); if (o) o.textContent = (+el.value).toFixed(el.step < 1 ? (el.step < 0.1 ? 2 : 1) : 0); }

// ------------------------------------------------------------------ views
function setView(v) {
  view = v;
  document.querySelectorAll('#viewTabs button').forEach(b => b.classList.toggle('on', b.dataset.view === v));
  const img = $('feed');
  img.onload = () => $('feedWait').style.display = 'none';
  img.onerror = () => { $('feedWait').style.display = ''; setTimeout(() => online && setView(view), 1500); };
  img.src = '/stream/' + v + '?t=' + Date.now();
}
$('viewTabs').onclick = e => { const b = e.target.closest('button'); if (b) setView(b.dataset.view); };

// ------------------------------------------------------------------ controls
$('btnTakeoff').onclick = () => cmd({action: 'takeoff'});
$('btnLand').onclick = () => cmd({action: 'land'});
$('btnStop').onclick = () => cmd({action: 'estop'});
$('btnReset').onclick = () => confirm('Reset the simulation? The drone returns to the start.') && cmd({action: 'reset'});
$('modeSeg').onclick = e => {
  const b = e.target.closest('button'); if (!b) return;
  if (T && !T.armed) return toast('Take off first', true);
  cmd({action: 'mode', mode: b.dataset.mode});
};
$('recBtn').onclick = () => cmd({action: 'record', state: T && T.recording ? 'stop' : 'start'});
document.querySelectorAll('[data-param]').forEach(el => {
  const send = () => {
    const p = {}; p[el.dataset.param] = el.type === 'checkbox' ? el.checked : +el.value;
    if (el.dataset.param === 'canny_lo') p.canny_hi = +el.value * 2.5;
    cmd({action: 'set', params: p}, true);
  };
  if (el.type === 'checkbox') el.onchange = send;
  else { el.oninput = () => showVal(el); el.onchange = () => { send(); toast(`${el.parentElement.firstChild.textContent.trim()} = ${el.value}`); }; }
});

// sticks: on-screen pad + keyboard. Values are re-sent every 100 ms while held (server has a 0.6 s dead-man).
const held = {pad: {}, key: {}};
const KEYS = {w: ['fwd', 1], s: ['fwd', -1], a: ['right', -1], d: ['right', 1], q: ['yaw', -1], e: ['yaw', 1], r: ['up', 1], f: ['up', -1]};
function stickVec() {
  const v = {fwd: 0, right: 0, up: 0, yaw: 0};
  for (const src of [held.pad, held.key]) for (const [axis, val] of Object.values(src)) v[axis] = Math.max(-1, Math.min(1, v[axis] + val));
  return v;
}
let wasActive = false;
setInterval(() => {
  const v = stickVec(), active = Object.values(v).some(x => x !== 0);
  if (active || wasActive) cmd(Object.assign({action: 'stick'}, v), true);
  wasActive = active;
}, 100);
document.querySelectorAll('.pad [data-stick]').forEach(b => {
  const [axis, val] = b.dataset.stick.split(':');
  const on = e => { e.preventDefault(); held.pad[b.dataset.stick] = [axis, +val]; b.classList.add('held'); if (T && T.mode !== 'MANUAL') toast('Manual override'); };
  const off = () => { delete held.pad[b.dataset.stick]; b.classList.remove('held'); };
  b.addEventListener('pointerdown', on); ['pointerup', 'pointerleave', 'pointercancel'].forEach(ev => b.addEventListener(ev, off));
});
document.querySelector('.pad [data-hover]').onclick = () => cmd({action: 'hold'});
const typing = e => /INPUT|TEXTAREA|SELECT/.test(e.target.tagName) && e.target.type !== 'checkbox' && e.target.type !== 'range';
window.addEventListener('keydown', e => {
  if (typing(e) || e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (KEYS[k]) { e.preventDefault(); if (!held.key[k] && T && T.mode !== 'MANUAL') toast('Manual override'); held.key[k] = KEYS[k]; return; }
  if (e.repeat) return;
  if (k === ' ') { e.preventDefault(); cmd({action: 'hold'}); }
  else if (k === 't') cmd({action: 'takeoff'});
  else if (k === 'l') cmd({action: 'land'});
  else if (k === 'x') cmd({action: 'estop'});
});
window.addEventListener('keyup', e => { delete held.key[e.key.toLowerCase()]; });
window.addEventListener('blur', () => { held.key = {}; held.pad = {}; });

// ------------------------------------------------------------------ map
const mapC = $('mapCanvas'), mctx = mapC.getContext('2d');
const mv = {cx: 30, cy: 10, scale: 3.2};    // world centre (north x, east y) and px per metre
const layer = k => document.querySelector(`[data-layer="${k}"]`).checked;
function w2s(x, y) { const W = mapC.clientWidth, H = mapC.clientHeight; return [W / 2 + (y - mv.cy) * mv.scale, H / 2 - (x - mv.cx) * mv.scale]; }
function s2w(px, py) { const W = mapC.clientWidth, H = mapC.clientHeight; return [mv.cx - (py - H / 2) / mv.scale, mv.cy + (px - W / 2) / mv.scale]; }
function heat(v) { // 0..1 -> jet-ish rgba
  const r = Math.min(255, Math.max(0, 255 * (1.5 - Math.abs(4 * v - 3)))), g = Math.min(255, Math.max(0, 255 * (1.5 - Math.abs(4 * v - 2)))), b = Math.min(255, Math.max(0, 255 * (1.5 - Math.abs(4 * v - 1))));
  return [r | 0, g | 0, b | 0];
}
function drawMap() {
  const dpr = window.devicePixelRatio || 1, W = mapC.clientWidth, H = mapC.clientHeight;
  if (mapC.width !== W * dpr || mapC.height !== H * dpr) { mapC.width = W * dpr; mapC.height = H * dpr; }
  const c = mctx; c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, W, H);
  if (!T) return;
  if (layer('follow') && !drag) { mv.cx += (T.pos.x - mv.cx) * 0.15; mv.cy += (T.pos.y - mv.cy) * 0.15; }
  // grid
  const step = mv.scale > 4 ? 10 : mv.scale > 1.5 ? 20 : 50;
  const [x0w, y0w] = s2w(0, H), [x1w, y1w] = s2w(W, 0);
  c.lineWidth = 1; c.strokeStyle = '#31516555'; c.fillStyle = '#5f7f92'; c.font = '9px monospace';
  for (let gy = Math.floor(y0w / step) * step; gy <= y1w; gy += step) { const [sx] = w2s(0, gy); c.beginPath(); c.moveTo(sx, 0); c.lineTo(sx, H); c.stroke(); }
  for (let gx = Math.floor(x0w / step) * step; gx <= x1w; gx += step) { const [, sy] = w2s(gx, 0); c.beginPath(); c.moveTo(0, sy); c.lineTo(W, sy); c.stroke(); }
  c.fillText(`grid ${step} m`, 10, H - 10); c.fillText('N ↑', W - 30, 22);
  // crowd heat
  if (layer('crowd')) {
    c.globalCompositeOperation = 'lighter';
    for (const [x, y, v] of T.crowd) {
      const [sx, sy] = w2s(x, y), n = Math.min(1, v / 3), rad = Math.max(10, 7 * mv.scale);
      const [r, g, b] = heat(0.25 + 0.75 * n);
      const gr = c.createRadialGradient(sx, sy, 0, sx, sy, rad);
      gr.addColorStop(0, `rgba(${r},${g},${b},${0.25 + 0.45 * n})`); gr.addColorStop(1, `rgba(${r},${g},${b},0)`);
      c.fillStyle = gr; c.fillRect(sx - rad, sy - rad, rad * 2, rad * 2);
    }
    c.globalCompositeOperation = 'source-over';
  }
  // depth-mapped obstacles
  if (layer('occ')) {
    const s = Math.max(2, 2 * mv.scale);
    for (const [x, y, n] of T.occ) { const [sx, sy] = w2s(x, y); c.fillStyle = `rgba(239,110,90,${0.18 + n * 0.07})`; c.fillRect(sx - s / 2, sy - s / 2, s, s); }
  }
  // patrol route (+ draft)
  const route = (pts, col, dash, loop) => {
    if (pts.length < 1) return;
    c.strokeStyle = col; c.setLineDash(dash); c.lineWidth = 1.6; c.beginPath();
    pts.forEach((p, i) => { const [sx, sy] = w2s(p[0], p[1]); i ? c.lineTo(sx, sy) : c.moveTo(sx, sy); });
    if (loop && pts.length > 2) c.closePath(); c.stroke(); c.setLineDash([]);
    pts.forEach((p, i) => { const [sx, sy] = w2s(p[0], p[1]); c.fillStyle = col; c.beginPath(); c.arc(sx, sy, 8, 0, 7); c.fill(); c.fillStyle = '#071a21'; c.font = 'bold 9px sans-serif'; c.textAlign = 'center'; c.fillText(i + 1, sx, sy + 3); c.textAlign = 'left'; });
  };
  if (layer('route')) route(T.patrol, T.mode === 'PATROL' ? '#42d9cecc' : '#42d9ce55', [6, 5], true);
  route(draftRoute, '#f0c181', [3, 4], false);
  // trail
  if (T.trail.length > 1) {
    c.strokeStyle = '#7ff0e2'; c.lineWidth = 2; c.beginPath();
    T.trail.forEach((p, i) => { const [sx, sy] = w2s(p[0], p[1]); i ? c.lineTo(sx, sy) : c.moveTo(sx, sy); }); c.stroke();
  }
  // home
  const [hx, hy] = w2s(T.home[0], T.home[1]);
  c.fillStyle = '#304b67'; c.strokeStyle = '#7f9bb6'; c.fillRect(hx - 11, hy - 11, 22, 22); c.strokeRect(hx - 11, hy - 11, 22, 22);
  c.fillStyle = '#fff'; c.font = '13px sans-serif'; c.textAlign = 'center'; c.fillText('⌂', hx, hy + 5); c.textAlign = 'left';
  // incidents
  for (const e of events) {
    if (e.severity === 'INFO') continue;
    const [sx, sy] = w2s(e.x, e.y);
    const col = {LOW: '#4fc7bf', MEDIUM: '#f3d48d', HIGH: '#f29a55', CRITICAL: '#ef5a5a'}[e.severity] || '#ccc';
    c.save(); c.translate(sx, sy); c.rotate(Math.PI / 4); c.fillStyle = col + (focusEvt === e.id ? '' : 'aa'); c.strokeStyle = '#071a21';
    const s = focusEvt === e.id ? 9 : 5; c.fillRect(-s, -s, s * 2, s * 2); c.strokeRect(-s, -s, s * 2, s * 2); c.restore();
  }
  // target
  if (T.target) {
    const [tx, ty] = w2s(T.target[0], T.target[1]), [dx, dy] = w2s(T.pos.x, T.pos.y);
    c.strokeStyle = '#f0c181'; c.setLineDash([4, 4]); c.beginPath(); c.moveTo(dx, dy); c.lineTo(tx, ty); c.stroke(); c.setLineDash([]);
    c.beginPath(); c.arc(tx, ty, 10, 0, 7); c.stroke(); c.beginPath(); c.moveTo(tx - 15, ty); c.lineTo(tx + 15, ty); c.moveTo(tx, ty - 15); c.lineTo(tx, ty + 15); c.stroke();
  }
  // police stations + radio range
  for (const p of (T.stations || [])) {
    const [px, py] = w2s(p.x, p.y), rr = p.range_m * mv.scale, deliv = T.delivery && T.delivery.station === p.id;
    c.strokeStyle = deliv ? '#ff8f8f' : '#7fb2ff88'; c.setLineDash([6, 6]); c.lineWidth = deliv ? 2 : 1.3;
    c.beginPath(); c.arc(px, py, rr, 0, 7); c.stroke(); c.setLineDash([]);
    c.fillStyle = '#1d4fa8'; c.strokeStyle = '#cfe0ff'; c.lineWidth = 2;
    c.beginPath(); c.arc(px, py, 12, 0, 7); c.fill(); c.stroke();
    c.fillStyle = '#fff'; c.font = 'bold 12px sans-serif'; c.textAlign = 'center'; c.fillText('P', px, py + 4);
    c.font = '10px sans-serif'; c.fillStyle = '#cfe0ff'; c.fillText(p.name, px, py + 27);
    c.fillStyle = p.in_range ? '#59d8b5' : '#f2b06d'; c.fillText(`${p.dist_m.toFixed(0)} m ${p.in_range ? '· in range' : '· out of range'}`, px, py + 40);
    c.textAlign = 'left';
    if (deliv) { const [dx, dy] = w2s(T.pos.x, T.pos.y); c.strokeStyle = '#ff8f8f'; c.setLineDash([4, 4]); c.beginPath(); c.moveTo(dx, dy); c.lineTo(px, py); c.stroke(); c.setLineDash([]); }
  }
  // drone + camera FOV
  const [sx, sy] = w2s(T.pos.x, T.pos.y), hd = T.heading * Math.PI / 180;
  c.save(); c.translate(sx, sy); c.rotate(hd);
  const fov = 30 * mv.scale;
  c.fillStyle = 'rgba(66,217,206,.10)'; c.beginPath(); c.moveTo(0, 0); c.arc(0, 0, fov, -Math.PI / 2 - Math.PI / 4, -Math.PI / 2 + Math.PI / 4); c.closePath(); c.fill();
  c.fillStyle = T.collided ? '#ef5a5a' : '#1c726f'; c.strokeStyle = '#5ef7e5'; c.lineWidth = 1.5;
  c.beginPath(); c.moveTo(0, -13); c.lineTo(9, 9); c.lineTo(0, 4); c.lineTo(-9, 9); c.closePath(); c.fill(); c.stroke();
  c.restore();
  c.strokeStyle = '#1ce4c233'; c.lineWidth = 6; c.beginPath(); c.arc(sx, sy, 18, 0, 7); c.stroke();
  $('coords').textContent = `${T.gps.lat.toFixed(6)}° N · ${T.gps.lon.toFixed(6)}° E · local (${T.pos.x.toFixed(1)}, ${T.pos.y.toFixed(1)}) m`;
}
let drag = null;
mapC.addEventListener('pointerdown', e => { drag = {x: e.offsetX, y: e.offsetY, cx: mv.cx, cy: mv.cy, moved: false}; mapC.setPointerCapture(e.pointerId); });
mapC.addEventListener('pointermove', e => {
  if (!drag) return;
  const dx = e.offsetX - drag.x, dy = e.offsetY - drag.y;
  if (Math.abs(dx) + Math.abs(dy) > 4) { drag.moved = true; document.querySelector('[data-layer="follow"]').checked = false; }
  if (drag.moved) { mv.cy = drag.cy - dx / mv.scale; mv.cx = drag.cx + dy / mv.scale; drawMap(); }
});
mapC.addEventListener('pointerup', e => {
  const d = drag; drag = null;
  if (!d || d.moved) return;
  const [x, y] = s2w(e.offsetX, e.offsetY);
  // incident pin?
  for (const ev of events) { const [sx, sy] = w2s(ev.x, ev.y); if (ev.severity !== 'INFO' && Math.hypot(sx - e.offsetX, sy - e.offsetY) < 9) return focusIncident(ev.id); }
  if (e.altKey) { cmd({action: 'station_move', x, y}, true).then(() => toast('Police station moved')); return; }
  if (e.shiftKey) { draftRoute.push([+x.toFixed(1), +y.toFixed(1)]); updRouteBtn(); drawMap(); return; }
  if (T && !T.armed) return toast('Take off first, then click the map to fly there', true);
  cmd({action: 'goto', x, y});
});
mapC.addEventListener('wheel', e => { e.preventDefault(); mv.scale = Math.max(0.5, Math.min(14, mv.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15))); drawMap(); }, {passive: false});
function updRouteBtn() { $('routeApply').disabled = draftRoute.length < 2; $('routeApply').textContent = `Use new route (${draftRoute.length})`; }
$('routeApply').onclick = () => { cmd({action: 'patrol_set', points: draftRoute}); draftRoute = []; updRouteBtn(); };
$('routeClear').onclick = () => { draftRoute = []; updRouteBtn(); drawMap(); };
$('mapClear').onclick = () => cmd({action: 'clear_map'});
document.querySelectorAll('[data-layer]').forEach(el => el.onchange = drawMap);

// ------------------------------------------------------------------ chart
const chC = $('chartCanvas'), cctx = chC.getContext('2d');
function drawChart() {
  const dpr = window.devicePixelRatio || 1, W = chC.clientWidth, H = chC.clientHeight;
  if (chC.width !== W * dpr || chC.height !== H * dpr) { chC.width = W * dpr; chC.height = H * dpr; }
  const c = cctx; c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, W, H);
  if (!T || !T.series.length) return;
  const s = T.series, tEnd = s.at(-1)[0], t0 = tEnd - 120, pl = 30, pr = 28, pt = 8, pb = 18;
  const X = t => pl + (t - t0) / 120 * (W - pl - pr), maxD = 30, Yd = d => pt + (1 - Math.min(d, maxD) / maxD) * (H - pt - pb);
  const maxP = Math.max(8, ...s.map(r => r[2])), Yp = p => H - pb - p / maxP * (H - pt - pb) * 0.85;
  c.font = '9px monospace'; c.fillStyle = '#6c8296'; c.strokeStyle = '#284458'; c.setLineDash([3, 6]);
  for (const d of [0, 10, 20, 30]) { const y = Yd(d); c.beginPath(); c.moveTo(pl, y); c.lineTo(W - pr, y); c.stroke(); c.fillText(d + 'm', 2, y + 3); }
  c.setLineDash([]);
  const pts = s.filter(r => r[0] >= t0), bw = Math.max(1.5, (W - pl - pr) / 1200 * 2);
  c.fillStyle = '#b08cf066';
  for (const r of pts) if (r[2]) { const y = Yp(r[2]); c.fillRect(X(r[0]) - bw / 2, y, bw, H - pb - y); }
  c.fillStyle = '#b08cf0'; c.fillText(maxP, W - pr + 4, Yp(maxP) + 3); c.fillText('ppl', W - pr + 4, H - pb);
  const th = T.cfg.look_ahead_threshold; c.strokeStyle = '#f0c181'; c.setLineDash([5, 5]); c.beginPath(); c.moveTo(pl, Yd(th)); c.lineTo(W - pr, Yd(th)); c.stroke(); c.setLineDash([]);
  const grd = c.createLinearGradient(0, pt, 0, H - pb); grd.addColorStop(0, '#39d4c544'); grd.addColorStop(1, '#39d4c500');
  c.beginPath(); pts.forEach((r, i) => { const x = X(r[0]), y = Yd(r[1] == null ? maxD : r[1]); i ? c.lineTo(x, y) : c.moveTo(x, y); });
  c.strokeStyle = '#42d9ce'; c.lineWidth = 2; c.stroke();
  c.lineTo(X(pts.at(-1)[0]), H - pb); c.lineTo(X(pts[0][0]), H - pb); c.closePath(); c.fillStyle = grd; c.fill();
  c.fillStyle = '#6c8296'; c.fillText('-2:00', pl, H - 4); c.fillText('now', W - pr - 18, H - 4);
}

// ------------------------------------------------------------------ incidents
function sevOn() { return new Set([...document.querySelectorAll('#sevFilter input:checked')].map(i => i.value)); }
function renderIncidents() {
  const on = sevOn(), list = events.filter(e => on.has(e.severity)).slice().reverse().slice(0, 80);
  $('incidentBadge').textContent = events.filter(e => e.severity !== 'INFO').length + ' EVENTS';
  $('incidentList').innerHTML = list.length ? list.map(x => {
    const st = reviews[x.id] || 'PENDING REVIEW';
    return `<div class="incident-card sev-${safe(x.severity)}" id="card-${safe(x.id)}"><div class="incident-top"><h3>${safe(x.type)}</h3><span class="severity">${safe(x.severity)}</span></div>
      <p>${safe(x.id)} · T+${fmtT(x.time_sec)} · ${safe(x.zone)} · (${x.x}, ${x.y}) m</p><p>${safe(x.description)}</p>
      <p>STATUS: <b style="color:#a4e5d6">${safe(st)}</b></p>
      <div class="incident-actions">${[['VERIFIED', 'Verify'], ['DISMISSED', 'Dismiss'], ['ASSIGNED', 'Assign']].map(([s, l]) => `<button data-id="${safe(x.id)}" data-status="${s}" class="${st === s ? 'picked' : ''}">${l}</button>`).join('')}
      <button data-id="${safe(x.id)}" data-locate>Locate</button></div></div>`;
  }).join('') : '<div class="empty">◌ No incidents yet.<br>Take off and fly — avoidance manoeuvres, crowd congestion and collisions appear here.</div>';
}
$('incidentList').onclick = e => {
  const b = e.target.closest('button'); if (!b) return;
  if (b.dataset.locate !== undefined) { const ev = events.find(x => x.id === b.dataset.id); if (ev) { mv.cx = ev.x; mv.cy = ev.y; document.querySelector('[data-layer="follow"]').checked = false; focusEvt = ev.id; drawMap(); } return; }
  reviews[b.dataset.id] = b.dataset.status; renderIncidents();
};
function focusIncident(id) {
  focusEvt = id; document.querySelectorAll('#sevFilter input').forEach(i => i.checked = true); renderIncidents();
  const el = $('card-' + id); if (el) { el.scrollIntoView({block: 'nearest', behavior: 'smooth'}); el.classList.add('flash'); }
  drawMap();
}
$('sevFilter').onchange = renderIncidents;
$('reportBtn').onclick = () => {
  const rep = {generated_at: new Date().toISOString(), backend: T && T.backend, detector: T && T.detector,
    incidents: events.map(x => ({...x, review_status: reviews[x.id] || 'PENDING REVIEW'}))};
  const u = URL.createObjectURL(new Blob([JSON.stringify(rep, null, 2)], {type: 'application/json'}));
  const a = document.createElement('a'); a.href = u; a.download = 'garuda_live_incident_report.json'; a.click(); setTimeout(() => URL.revokeObjectURL(u), 1000);
};

setInterval(() => { $('clock').textContent = new Date().toLocaleTimeString('en-IN', {hour12: false}) + ' IST'; }, 1000);
window.addEventListener('resize', () => { drawMap(); drawChart(); });
renderIncidents();
poll();
