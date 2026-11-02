// Browser ground station: Web Serial <-> telemetry radio <-> Cube (ArduPlane).
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const { PLANE_MODES, MODE_NUMBERS } = MAV;
  const VTOL_MODES = new Set(['QSTABILIZE', 'QHOVER', 'QLOITER', 'QLAND', 'QRTL', 'QACRO', 'QAUTOTUNE']);
  const ACK_RESULTS = ['accepted', 'temporarily rejected', 'denied', 'unsupported', 'failed', 'in progress', 'cancelled'];
  const COMMAND_NAMES = { 400: 'Arm/disarm', 176: 'Set mode', 192: 'Fly here', 512: 'Request home', 511: 'Message rate' };
  const TYPE_NAMES = { 1: 'fixed wing', 2: 'quadrotor', 13: 'hexarotor', 14: 'octorotor', 15: 'tricopter', 19: 'tailsitter', 20: 'tailsitter', 21: 'tilt-rotor', 22: 'VTOL', 23: 'tailsitter' };
  const COPTER_TYPES = new Set([2, 3, 4, 13, 14, 15]);

  // ---------- settings ----------
  const DEFAULTS = { m1: 1, m2: 2, m4: 3, tl: 5, tr: 6, p0: 1100, p90: 1900, cells: 4 };
  const settings = Object.assign({}, DEFAULTS, JSON.parse(localStorage.getItem('vtol-gcs-settings') || '{}'));
  const SETTING_INPUTS = { sM1: 'm1', sM2: 'm2', sM4: 'm4', sTL: 'tl', sTR: 'tr', sP0: 'p0', sP90: 'p90', sCells: 'cells' };
  for (const id in SETTING_INPUTS) {
    const key = SETTING_INPUTS[id];
    $(id).value = settings[key];
    $(id).addEventListener('change', () => {
      const v = Number($(id).value);
      if (Number.isFinite(v)) settings[key] = v;
      localStorage.setItem('vtol-gcs-settings', JSON.stringify(settings));
      dirty = true;
    });
  }

  // ---------- state ----------
  let link = null;          // { kind, write(bytes), close() }
  let parser = null;
  const enc = new MAV.Encoder(255, 190);
  let tlog = [];
  let s = freshState();
  let dirty = true;
  let gcsTimer = null;
  const pending = {};       // command id -> retry info

  function freshState() {
    return {
      target: null, lastHb: 0, connectedAt: 0, streamsRequested: false, homeAsked: 0,
      hb: null, att: null, pos: null, hud: null, sys: null, gps: null, servo: null, rc: null, radio: null, home: null,
      seq: null, rx: 0, lost: 0, armedSince: 0, flightMs: 0, lastArmed: false,
    };
  }

  // ---------- logging ----------
  function log(text, cls) {
    const li = document.createElement('li');
    if (cls) li.className = cls;
    const t = document.createElement('time');
    t.textContent = new Date().toLocaleTimeString([], { hour12: false });
    const span = document.createElement('span');
    span.textContent = text;
    li.append(t, span);
    const ul = $('log');
    ul.prepend(li);
    while (ul.children.length > 300) ul.lastChild.remove();
  }

  function recordFrame(raw) {
    const rec = new Uint8Array(8 + raw.length);
    new DataView(rec.buffer).setBigUint64(0, BigInt(Date.now()) * 1000n, false);
    rec.set(raw, 8);
    tlog.push(rec);
  }

  // ---------- link ----------
  function send(name, values) {
    if (!link) return false;
    const frame = enc.encode(name, values);
    recordFrame(frame);
    link.write(frame);
    return true;
  }

  function onMessage(m, hdr, raw) {
    recordFrame(raw);
    if (m._name === 'RADIO_STATUS') { s.radio = m; s.radioAt = Date.now(); dirty = true; return; }
    if (m._name === 'HEARTBEAT') {
      if (m.type === 6 || m.autopilot === 8) return; // another GCS, gimbal, etc.
      if (!s.target) {
        s.target = { sys: hdr.sysid, comp: hdr.compid };
        log(`Vehicle found: system ${hdr.sysid}, component ${hdr.compid}`, 'gcs');
        if (m.autopilot === 3 && COPTER_TYPES.has(m.type)) log('This looks like ArduCopter. This build needs ArduPlane with Q_ENABLE=1.', 's4');
      }
    }
    if (!s.target || hdr.sysid !== s.target.sys) return;

    if (hdr.compid === s.target.comp) {
      if (s.seq !== null) s.lost += (hdr.seq - s.seq - 1) & 0xff;
      s.seq = hdr.seq;
      s.rx++;
    }

    switch (m._name) {
      case 'HEARTBEAT': {
        if (hdr.compid !== s.target.comp) return;
        const now = Date.now();
        const wasLost = !s.lastHb || now - s.lastHb > 5000;
        const prev = s.hb;
        s.hb = m;
        s.lastHb = now;
        if (wasLost) {
          if (s.streamsRequested) log('Heartbeat back. Link restored.', 'gcs');
          requestStreams();
        }
        const armed = (m.base_mode & 128) !== 0;
        if (prev && armed !== ((prev.base_mode & 128) !== 0)) {
          log(armed ? 'ARMED' : 'DISARMED', armed ? 's4' : 'gcs');
          if (armed) requestHome();
        }
        if (prev && prev.custom_mode !== m.custom_mode) log(`Mode → ${modeName(m.custom_mode)}`, 'gcs');
        break;
      }
      case 'ATTITUDE': s.att = m; break;
      case 'GLOBAL_POSITION_INT': s.pos = m; addTrack(m); break;
      case 'VFR_HUD': s.hud = m; break;
      case 'SYS_STATUS': s.sys = m; break;
      case 'GPS_RAW_INT': s.gps = m; break;
      case 'SERVO_OUTPUT_RAW': if (m.port === 0) s.servo = m; break;
      case 'RC_CHANNELS': s.rc = m; break;
      case 'HOME_POSITION':
        s.home = { lat: m.latitude / 1e7, lon: m.longitude / 1e7 };
        updateHomeMarker();
        break;
      case 'STATUSTEXT': log(m.text, 's' + m.severity); break;
      case 'COMMAND_ACK': {
        const p = pending[m.command];
        if (m.result === 5) break; // in progress
        if (p) { clearTimeout(p.timer); delete pending[m.command]; }
        const what = (p && p.label) || COMMAND_NAMES[m.command] || 'Command ' + m.command;
        if (m.command === 512 || m.command === 511) break;
        log(`${what}: ${ACK_RESULTS[m.result] || 'result ' + m.result}`, m.result === 0 ? 'gcs' : 's4');
        break;
      }
    }
    dirty = true;
  }

  function requestStreams() {
    if (!s.target) return;
    // [stream id, Hz]: EXTENDED_STATUS, RC_CHANNELS, POSITION, EXTRA1 (attitude), EXTRA2 (VFR_HUD), EXTRA3
    for (const [id, hz] of [[2, 2], [3, 4], [6, 4], [10, 10], [11, 4], [12, 1]]) {
      send('REQUEST_DATA_STREAM', { req_message_rate: hz, target_system: s.target.sys, target_component: s.target.comp, req_stream_id: id, start_stop: 1 });
    }
    s.streamsRequested = true;
    requestHome();
  }

  function requestHome() {
    if (!s.target) return;
    s.homeAsked = Date.now();
    commandLong(512, [242], { quiet: true, retries: 0 });
  }

  function commandLong(command, params, opts) {
    opts = opts || {};
    if (!s.target) { log('No vehicle yet: wait for a heartbeat.', 's4'); return; }
    const p = params.concat([0, 0, 0, 0, 0, 0, 0]).slice(0, 7);
    const values = { command, target_system: s.target.sys, target_component: s.target.comp, confirmation: 0 };
    p.forEach((v, i) => { values['param' + (i + 1)] = v; });
    sendWithRetry(command, 'COMMAND_LONG', values, opts);
  }

  function sendWithRetry(command, name, values, opts) {
    const prev = pending[command];
    if (prev) clearTimeout(prev.timer);
    const entry = { label: opts.label, tries: 0 };
    pending[command] = entry;
    const attempt = () => {
      if (!link) return;
      if (name === 'COMMAND_LONG') values.confirmation = entry.tries;
      send(name, values);
      entry.tries++;
      entry.timer = setTimeout(() => {
        if (pending[command] !== entry) return;
        if (entry.tries <= (opts.retries === undefined ? 2 : opts.retries)) attempt();
        else {
          delete pending[command];
          if (!opts.quiet) log(`${opts.label || COMMAND_NAMES[command]}: no reply from the aircraft`, 's3');
        }
      }, 1500);
    };
    attempt();
  }

  function setMode(name) {
    const num = MODE_NUMBERS[name];
    if (num === undefined || !s.target) return;
    log(`Requesting ${name}…`, 'gcs');
    commandLong(176, [1, num], { label: `Mode ${name}` });
  }

  function arm(on) {
    log(on ? 'Requesting ARM…' : 'Requesting DISARM…', 'gcs');
    commandLong(400, [on ? 1 : 0], { label: on ? 'Arm' : 'Disarm' });
  }

  function flyTo(lat, lon, alt) {
    if (!s.target) return;
    const values = {
      param1: -1, param2: 1, param3: 0, param4: NaN,
      x: Math.round(lat * 1e7), y: Math.round(lon * 1e7), z: alt,
      command: 192, target_system: s.target.sys, target_component: s.target.comp,
      frame: 3, current: 0, autocontinue: 0,
    };
    log(`Fly here: ${lat.toFixed(6)}, ${lon.toFixed(6)} at ${alt} m (switches to GUIDED)`, 'gcs');
    sendWithRetry(192, 'COMMAND_INT', values, { label: 'Fly here' });
  }

  function startLink(l) {
    link = l;
    parser = new MAV.Parser(onMessage);
    s = freshState();
    s.connectedAt = Date.now();
    tlog = [];
    clearTrack();
    gcsTimer = setInterval(() => send('HEARTBEAT', { type: 6, autopilot: 8, base_mode: 0, custom_mode: 0, system_status: 4, mavlink_version: 3 }), 1000);
    document.body.classList.toggle('demo', l.kind === 'demo');
    $('btnSerial').disabled = $('btnDemo').disabled = $('baud').disabled = true;
    $('btnDisconnect').disabled = $('btnLog').disabled = false;
    follow = true;
    updateFollowBtn();
    dirty = true;
  }

  async function stopLink(reason) {
    if (!link) return;
    const l = link;
    link = null;
    clearInterval(gcsTimer);
    for (const k in pending) { clearTimeout(pending[k].timer); delete pending[k]; }
    try { await l.close(); } catch (e) { /* already gone */ }
    log(reason || 'Disconnected', 'gcs');
    $('btnSerial').disabled = !('serial' in navigator);
    $('btnDemo').disabled = $('baud').disabled = false;
    $('btnDisconnect').disabled = true;
    document.body.classList.remove('demo');
    dirty = true;
  }

  async function connectSerial() {
    let port;
    try {
      port = await navigator.serial.requestPort();
    } catch (e) { return; } // user closed the picker
    const baudRate = Number($('baud').value);
    try {
      await port.open({ baudRate, bufferSize: 8192 });
    } catch (e) {
      log(`Could not open the port: ${e.message}. Is Mission Planner or another app using it?`, 's3');
      return;
    }
    const writer = port.writable.getWriter();
    let reader = null;
    let closing = false;
    startLink({
      kind: 'serial',
      write: (bytes) => writer.write(bytes).catch(() => {}),
      close: async () => {
        closing = true;
        if (reader) await reader.cancel().catch(() => {});
        writer.releaseLock();
        await port.close();
      },
    });
    log(`Serial port open at ${baudRate} baud. Waiting for heartbeat…`, 'gcs');
    (async () => {
      while (port.readable && !closing) {
        reader = port.readable.getReader();
        try {
          for (;;) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value && parser) parser.feed(value);
          }
        } catch (e) {
          if (!closing) log(`Serial read error: ${e.message}`, 's4');
        } finally {
          reader.releaseLock();
        }
      }
      if (!closing) stopLink('Radio unplugged');
    })();
  }

  function connectDemo() {
    const sim = new SimVehicle((bytes) => { if (parser) parser.feed(bytes); });
    startLink({ kind: 'demo', write: (b) => sim.write(b), close: () => sim.stop() });
    log('Demo simulator started. Hold ARM to take off.', 'gcs');
    sim.start();
  }

  function saveTlog() {
    if (!tlog.length) return;
    const blob = new Blob(tlog, { type: 'application/octet-stream' });
    const a = document.createElement('a');
    const d = new Date(s.connectedAt || Date.now());
    const pad = (n) => String(n).padStart(2, '0');
    a.download = `vtol-${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}.tlog`;
    a.href = URL.createObjectURL(blob);
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  }

  // ---------- map ----------
  let map = null, acMarker = null, homeMarker = null, trackLine = null, tgtMarker = null;
  let follow = true, track = [], target = null;

  function initMap() {
    if (!window.L) { $('mapOffline').style.display = 'flex'; $('mapHint').textContent = 'Map unavailable'; return; }
    map = L.map('map', { zoomControl: true }).setView([20, 0], 2);
    const street = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19, attribution: '&copy; OpenStreetMap contributors',
    });
    const sat = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
      maxZoom: 19, attribution: 'Imagery &copy; Esri',
    });
    sat.addTo(map);
    L.control.layers({ Satellite: sat, Street: street }).addTo(map);
    trackLine = L.polyline([], { color: '#c2410c', weight: 2.5, opacity: 0.9 }).addTo(map);
    acMarker = L.marker([0, 0], {
      icon: L.divIcon({
        className: '', iconSize: [34, 34], iconAnchor: [17, 17],
        html: '<svg class="aircraft-icon" width="34" height="34" viewBox="-17 -17 34 34"><path d="M0,-15 L10,12 L0,6 L-10,12 Z" fill="#c2410c" stroke="#fff" stroke-width="2"/></svg>',
      }),
      interactive: false,
    });
    map.on('dragstart', () => { follow = false; updateFollowBtn(); });
    map.on('click', (e) => {
      target = e.latlng;
      if (!tgtMarker) tgtMarker = L.circleMarker(target, { radius: 7, color: '#16181d', weight: 2, fillColor: '#fff', fillOpacity: 1 }).addTo(map);
      tgtMarker.setLatLng(target);
      dirty = true;
    });
  }

  function addTrack(m) {
    if (!m.lat && !m.lon) return;
    const p = [m.lat / 1e7, m.lon / 1e7];
    const last = track[track.length - 1];
    if (last && distance(last[0], last[1], p[0], p[1]) < 1) return;
    track.push(p);
    if (track.length > 5000) track.shift();
    if (trackLine) trackLine.setLatLngs(track);
  }

  function clearTrack() { track = []; if (trackLine) trackLine.setLatLngs([]); }

  function updateHomeMarker() {
    if (!map || !s.home) return;
    if (!homeMarker) {
      homeMarker = L.marker([s.home.lat, s.home.lon], {
        icon: L.divIcon({ className: '', iconSize: [22, 22], iconAnchor: [11, 11], html: '<div style="width:22px;height:22px;border-radius:50%;background:#15803d;color:#fff;font:600 12px/22px IBM Plex Mono,monospace;text-align:center;border:2px solid #fff">H</div>' }),
      }).addTo(map);
    }
    homeMarker.setLatLng([s.home.lat, s.home.lon]);
  }

  function updateFollowBtn() { $('btnFollow').textContent = 'Follow aircraft: ' + (follow ? 'on' : 'off'); $('btnFollow').classList.toggle('on', follow); }

  function distance(lat1, lon1, lat2, lon2) {
    const r = Math.PI / 180;
    const a = Math.sin((lat2 - lat1) * r / 2) ** 2 + Math.cos(lat1 * r) * Math.cos(lat2 * r) * Math.sin((lon2 - lon1) * r / 2) ** 2;
    return 2 * 6371000 * Math.asin(Math.sqrt(a));
  }

  // ---------- drawing ----------
  function drawAdi() {
    const c = $('adi'), ctx = c.getContext('2d');
    const w = c.width, h = c.height, cx = w / 2, cy = h / 2;
    const roll = s.att ? s.att.roll : 0, pitch = s.att ? s.att.pitch * 180 / Math.PI : 0;
    const ppd = h / 50;
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    ctx.beginPath(); ctx.rect(0, 0, w, h); ctx.clip();
    ctx.translate(cx, cy);
    ctx.rotate(-roll);
    ctx.translate(0, pitch * ppd);
    ctx.fillStyle = s.att ? '#4f8fc7' : '#c9ced6'; ctx.fillRect(-w * 2, -h * 4, w * 4, h * 4);
    ctx.fillStyle = s.att ? '#8a6a45' : '#a9adb5'; ctx.fillRect(-w * 2, 0, w * 4, h * 4);
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(-w * 2, 0); ctx.lineTo(w * 2, 0); ctx.stroke();
    ctx.lineWidth = 2; ctx.fillStyle = '#fff'; ctx.font = '500 18px IBM Plex Mono, monospace'; ctx.textBaseline = 'middle';
    for (let d = -40; d <= 40; d += 5) {
      if (!d) continue;
      const y = -d * ppd, len = d % 10 ? 30 : 70;
      ctx.beginPath(); ctx.moveTo(-len, y); ctx.lineTo(len, y); ctx.stroke();
      if (!(d % 10)) { ctx.textAlign = 'left'; ctx.fillText(String(Math.abs(d)), len + 8, y); ctx.textAlign = 'right'; ctx.fillText(String(Math.abs(d)), -len - 8, y); }
    }
    ctx.restore();
    // roll scale
    ctx.save(); ctx.translate(cx, cy); ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
    const R = h * 0.42;
    for (const a of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
      const t = (a - 90) * Math.PI / 180, l = a % 30 ? 10 : 18;
      ctx.beginPath(); ctx.moveTo(Math.cos(t) * R, Math.sin(t) * R); ctx.lineTo(Math.cos(t) * (R - l), Math.sin(t) * (R - l)); ctx.stroke();
    }
    ctx.rotate(-roll); ctx.fillStyle = '#fbbf24';
    ctx.beginPath(); ctx.moveTo(0, -R + 2); ctx.lineTo(-9, -R + 18); ctx.lineTo(9, -R + 18); ctx.closePath(); ctx.fill();
    ctx.restore();
    // aircraft symbol
    ctx.save(); ctx.translate(cx, cy); ctx.strokeStyle = '#fbbf24'; ctx.lineWidth = 6; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(-110, 0); ctx.lineTo(-40, 0); ctx.lineTo(-20, 18); ctx.moveTo(110, 0); ctx.lineTo(40, 0); ctx.lineTo(20, 18); ctx.stroke();
    ctx.fillStyle = '#fbbf24'; ctx.beginPath(); ctx.arc(0, 0, 6, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
    if (!s.att) {
      ctx.fillStyle = 'rgba(22,24,29,.75)'; ctx.font = '500 22px IBM Plex Sans, sans-serif'; ctx.textAlign = 'center';
      ctx.fillText('No attitude data', cx, h - 30);
    }
  }

  function pwmToTilt(pwm) {
    if (!pwm) return null;
    return (pwm - settings.p0) / (settings.p90 - settings.p0) * 90;
  }

  function servoPwm(ch) { return s.servo ? s.servo['servo' + ch + '_raw'] : 0; }

  function drawTilt(left, right) {
    const c = $('tilt'), ctx = c.getContext('2d');
    const w = c.width, h = c.height;
    ctx.clearRect(0, 0, w, h);
    const pods = [['LEFT', left, w * 0.27], ['RIGHT', right, w * 0.73]];
    for (const [name, deg, x] of pods) {
      const y = h * 0.62, L = 74;
      // reference arc 0..90
      ctx.strokeStyle = '#dcdfe4'; ctx.lineWidth = 2; ctx.setLineDash([4, 5]);
      ctx.beginPath(); ctx.arc(x, y, L, -Math.PI / 2, 0); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = '#5d6370'; ctx.font = '500 22px IBM Plex Mono, monospace'; ctx.textAlign = 'center';
      ctx.fillText('0°', x, y - L - 10); ctx.textAlign = 'left'; ctx.fillText('90°', x + L + 6, y + 8);
      ctx.textAlign = 'center'; ctx.fillText(name, x, h - 8);
      // wing stub
      ctx.strokeStyle = '#16181d'; ctx.lineWidth = 6; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(x - 46, y + 10); ctx.lineTo(x + 20, y + 10); ctx.stroke();
      if (deg === null) continue;
      const a = (deg - 90) * Math.PI / 180; // 0° points up, 90° points forward (right)
      const ex = x + Math.cos(a) * L * 0.8, ey = y + Math.sin(a) * L * 0.8;
      ctx.strokeStyle = '#c2410c'; ctx.lineWidth = 8;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(ex, ey); ctx.stroke();
      // prop disc, perpendicular to the thrust axis
      const px = Math.cos(a + Math.PI / 2) * 36, py = Math.sin(a + Math.PI / 2) * 36;
      ctx.strokeStyle = '#16181d'; ctx.lineWidth = 4;
      ctx.beginPath(); ctx.moveTo(ex - px, ey - py); ctx.lineTo(ex + px, ey + py); ctx.stroke();
      ctx.fillStyle = '#16181d'; ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#16181d'; ctx.font = '600 30px IBM Plex Mono, monospace'; ctx.textAlign = 'right';
      ctx.fillText(deg.toFixed(0) + '°', x - 34, y - 40);
    }
  }

  function bars(el, rows) {
    if (el.children.length !== rows.length) {
      el.innerHTML = rows.map(() => '<div class="bar"><span></span><div class="track"><b></b></div><span class="num"></span></div>').join('');
    }
    rows.forEach((r, i) => {
      const row = el.children[i];
      row.children[0].textContent = r[0];
      row.children[1].firstChild.style.width = Math.max(0, Math.min(100, r[1])) + '%';
      row.children[2].textContent = r[2];
    });
  }

  function modeName(n) { return PLANE_MODES[n] || 'MODE ' + n; }

  function setText(id, html) { const el = $(id); if (el.innerHTML !== html) el.innerHTML = html; }

  function fmt(v, digits, unit) { return v === undefined || v === null || Number.isNaN(v) ? '—' : `${v.toFixed(digits)}<i>${unit}</i>`; }

  // ---------- render ----------
  function render() {
    const now = Date.now();
    const hbAge = s.lastHb ? (now - s.lastHb) / 1000 : null;
    const linked = link && hbAge !== null && hbAge < 3;

    // link pill
    const pill = $('linkPill');
    let cls = '', label = 'No link';
    if (link && !s.lastHb) { cls = 'warn'; label = 'Waiting for heartbeat'; }
    else if (link && linked) { cls = 'ok'; label = link.kind === 'demo' ? 'Demo link' : 'Telemetry OK'; }
    else if (link) { cls = 'bad'; label = `Link lost ${hbAge.toFixed(0)} s`; }
    pill.className = 'pill ' + cls;
    pill.textContent = label;

    // flight timer
    const armed = !!(s.hb && (s.hb.base_mode & 128));
    if (armed && !s.lastArmed) s.armedSince = now;
    if (!armed && s.lastArmed) s.flightMs += now - s.armedSince;
    s.lastArmed = armed;
    const ft = s.flightMs + (armed ? now - s.armedSince : 0);

    if (!dirty && link) { setText('kHb', hbAge === null ? '—' : `${hbAge.toFixed(1)}<i>s ago</i>`); }
    if (!dirty) return;
    dirty = false;

    drawAdi();
    if (s.att) {
      const d = (r) => (r * 180 / Math.PI).toFixed(0);
      setText('attText', `R ${d(s.att.roll)}° P ${d(s.att.pitch)}°`);
    } else setText('attText', '—');

    const hud = s.hud, pos = s.pos;
    setText('rAlt', pos ? fmt(pos.relative_alt / 1000, 1, 'm') : '—');
    setText('rClimb', hud ? fmt(hud.climb, 1, 'm/s') : '—');
    setText('rAs', hud ? fmt(hud.airspeed, 1, 'm/s') : '—');
    setText('rGs', hud ? fmt(hud.groundspeed, 1, 'm/s') : '—');
    setText('rHdg', hud ? fmt(hud.heading, 0, '°') : '—');
    setText('rThr', hud ? fmt(hud.throttle, 0, '%') : '—');
    const dHome = pos && s.home ? distance(s.home.lat, s.home.lon, pos.lat / 1e7, pos.lon / 1e7) : null;
    setText('rHome', dHome === null ? '—' : dHome >= 1000 ? fmt(dHome / 1000, 2, 'km') : fmt(dHome, 0, 'm'));
    const sec = Math.floor(ft / 1000);
    setText('rTime', s.hb ? `${Math.floor(sec / 60)}:${String(sec % 60).padStart(2, '0')}` : '—');

    // vehicle
    if (s.hb) {
      setText('armText', armed ? 'ARMED' : 'DISARMED');
      $('armBox').className = armed ? 'armed' : '';
      setText('modeText', modeName(s.hb.custom_mode));
      setText('vehText', `sys ${s.target.sys} · ${TYPE_NAMES[s.hb.type] || 'type ' + s.hb.type}`);
    } else {
      setText('armText', '—'); setText('modeText', '—'); setText('vehText', '—'); $('armBox').className = '';
    }
    const sys = s.sys;
    if (sys && sys.voltage_battery !== 65535) {
      const v = sys.voltage_battery / 1000, cell = v / settings.cells;
      const col = cell < 3.3 ? 'var(--bad)' : cell < 3.5 ? 'var(--warn)' : 'inherit';
      setText('kBatt', `<span style="color:${col}">${v.toFixed(1)}<i>V · ${cell.toFixed(2)}/cell</i></span>`);
    } else setText('kBatt', '—');
    setText('kCurr', sys && sys.current_battery >= 0 ? fmt(sys.current_battery / 100, 1, 'A') : '—');
    setText('kRem', sys && sys.battery_remaining >= 0 ? fmt(sys.battery_remaining, 0, '%') : '—');
    const FIX = ['No GPS', 'No fix', '2D', '3D', 'DGPS', 'RTK float', 'RTK fixed', 'Static', 'PPP'];
    setText('kGps', s.gps ? `${FIX[s.gps.fix_type] || s.gps.fix_type}<i>${s.gps.satellites_visible} sats · HDOP ${(s.gps.eph / 100).toFixed(1)}</i>` : '—');
    const r = s.radio && now - s.radioAt < 5000 ? s.radio : null;
    setText('kRssi', r ? `${r.rssi} / ${r.remrssi}` : '—');
    setText('kFade', r ? `${Math.min(r.rssi - r.noise, r.remrssi - r.remnoise)}<i>(${r.rssi - r.noise} / ${r.remrssi - r.remnoise})</i>` : '—');
    setText('kHb', hbAge === null ? '—' : `${hbAge.toFixed(1)}<i>s ago</i>`);
    const total = s.rx + s.lost;
    setText('kLoss', total > 20 ? fmt(100 * s.lost / total, 1, '%') : '—');

    // tilt + outputs
    const tl = pwmToTilt(servoPwm(settings.tl)), tr = pwmToTilt(servoPwm(settings.tr));
    drawTilt(tl, tr);
    setText('tiltText', tl === null ? '—' : `L ${tl.toFixed(0)}° · R ${tr === null ? '—' : tr.toFixed(0) + '°'}`);
    const motor = (label, ch) => { const p = servoPwm(ch); return [label, p ? (p - 1000) / 10 : 0, p ? Math.round(Math.max(0, (p - 1000) / 10)) + '%' : '—']; };
    const servo = (label, ch) => { const p = servoPwm(ch); return [label, p ? (p - 900) / 12 : 0, p || '—']; };
    bars($('outBars'), [motor('M1 FR', settings.m1), motor('M2 FL', settings.m2), motor('M4 rear', settings.m4), servo('Tilt L', settings.tl), servo('Tilt R', settings.tr)]);
    const rcRows = [];
    for (let i = 1; i <= 8; i++) { const p = s.rc ? s.rc['chan' + i + '_raw'] : 0; rcRows.push(['CH' + i, p ? (p - 900) / 12 : 0, p || '—']); }
    bars($('rcBars'), rcRows);

    // map
    if (map && pos && (pos.lat || pos.lon)) {
      const ll = [pos.lat / 1e7, pos.lon / 1e7];
      if (!map.hasLayer(acMarker)) { acMarker.addTo(map); map.setView(ll, 17); }
      acMarker.setLatLng(ll);
      const svg = acMarker.getElement() && acMarker.getElement().querySelector('svg');
      if (svg) svg.style.transform = `rotate(${pos.hdg === 65535 ? 0 : pos.hdg / 100}deg)`;
      if (follow) map.panTo(ll, { animate: false });
    }
    setText('tgtText', target ? `Target ${target.lat.toFixed(6)}, ${target.lng.toFixed(6)}${pos ? ' · ' + distance(pos.lat / 1e7, pos.lon / 1e7, target.lat, target.lng).toFixed(0) + ' m away' : ''}` : 'No target selected');

    // controls
    const canCmd = !!(link && s.target && linked);
    document.querySelectorAll('[data-mode], #btnArm, #btnDisarm').forEach((b) => { b.disabled = !canCmd; });
    $('btnFlyHere').disabled = !(canCmd && target);
    document.querySelectorAll('[data-mode]').forEach((b) => b.classList.toggle('on', !!(s.hb && modeName(s.hb.custom_mode) === b.dataset.mode)));
    setText('ctrlHint', !link ? 'Connect first' : !s.target ? 'Waiting for vehicle' : linked ? (link.kind === 'demo' ? 'Simulator' : 'Live') : 'Link lost: commands paused');
    const st = parser ? parser.stats : null;
    setText('pktStats', st ? `${st.frames} pkts · ${st.crcErrors} bad` : '—');
  }

  // ---------- controls ----------
  function buildModeButtons() {
    document.querySelectorAll('.modes').forEach((box) => {
      for (const m of box.dataset.modes.split(' ')) {
        const b = document.createElement('button');
        b.className = 'btn'; b.dataset.mode = m; b.textContent = m; b.disabled = true;
        box.appendChild(b);
      }
    });
    document.querySelectorAll('[data-mode]').forEach((b) => b.addEventListener('click', () => {
      const m = b.dataset.mode;
      const armed = s.hb && (s.hb.base_mode & 128);
      const inVtol = s.hb && VTOL_MODES.has(modeName(s.hb.custom_mode));
      if (armed && inVtol && !VTOL_MODES.has(m) && m !== 'RTL') {
        if (!confirm(`Switch to ${m}? From a VTOL mode this starts a TRANSITION to forward flight: the front motors tilt forward and the rear motor stops. Only do this with enough height and the pilot ready on the sticks.`)) return;
      }
      if (m === 'MANUAL' && armed && !confirm('MANUAL gives the pilot direct control with no stabilisation. Continue?')) return;
      setMode(m);
    }));
  }

  function holdButton(btn, action) {
    const fill = btn.querySelector('.fill');
    const ms = Number(btn.dataset.hold);
    let start = 0, raf = 0;
    const cancel = () => { cancelAnimationFrame(raf); start = 0; fill.style.width = '0'; };
    const tick = () => {
      const p = (performance.now() - start) / ms;
      fill.style.width = Math.min(100, p * 100) + '%';
      if (p >= 1) { cancel(); action(); return; }
      raf = requestAnimationFrame(tick);
    };
    btn.addEventListener('pointerdown', (e) => { if (btn.disabled) return; e.preventDefault(); start = performance.now(); raf = requestAnimationFrame(tick); });
    ['pointerup', 'pointerleave', 'pointercancel'].forEach((ev) => btn.addEventListener(ev, cancel));
    btn.addEventListener('keydown', (e) => { if (e.key === ' ' || e.key === 'Enter') e.preventDefault(); });
  }

  // ---------- wiring ----------
  buildModeButtons();
  holdButton($('btnArm'), () => arm(true));
  holdButton($('btnDisarm'), () => {
    const flying = s.pos && s.pos.relative_alt > 1000;
    if (flying && !confirm('The aircraft looks airborne. Disarming in the air stops the motors and it will fall. ArduPilot normally refuses this. Send disarm anyway?')) return;
    arm(false);
  });
  $('btnFlyHere').addEventListener('click', () => {
    if (!target) return;
    const alt = Number($('tgtAlt').value);
    if (!(alt >= 5)) { log('Pick a target altitude of at least 5 m.', 's4'); return; }
    const inVtol = s.hb && VTOL_MODES.has(modeName(s.hb.custom_mode));
    const warn = inVtol ? '\n\nThe aircraft is in a VTOL mode: GUIDED will start a TRANSITION to forward flight. Only do this with enough height.' : '';
    if (!confirm(`Send the aircraft to ${target.lat.toFixed(6)}, ${target.lng.toFixed(6)} at ${alt} m above home? It switches to GUIDED.${warn}`)) return;
    flyTo(target.lat, target.lng, alt);
  });
  $('btnFollow').addEventListener('click', () => { follow = !follow; updateFollowBtn(); dirty = true; });
  $('btnClearTrack').addEventListener('click', clearTrack);
  $('btnDemo').addEventListener('click', connectDemo);
  $('btnDisconnect').addEventListener('click', () => stopLink());
  $('btnLog').addEventListener('click', saveTlog);

  if ('serial' in navigator) {
    $('btnSerial').addEventListener('click', connectSerial);
  } else {
    $('btnSerial').disabled = true;
    $('btnSerial').title = 'Web Serial needs Chrome or Edge on a desktop/laptop';
    log('This browser has no Web Serial. Use Chrome or Edge on a laptop to connect the radio. Demo mode still works.', 's4');
  }
  window.addEventListener('beforeunload', (e) => { if (link && s.hb && (s.hb.base_mode & 128)) { e.preventDefault(); e.returnValue = ''; } });

  initMap();
  updateFollowBtn();
  setInterval(render, 100);
  render();
  log('Ready. Plug in the ground telemetry radio and press Connect, or try Demo.', 'gcs');
})();
