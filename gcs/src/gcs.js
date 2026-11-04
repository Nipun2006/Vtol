// Tri-Tilt GCS: a standalone ground station for a three-rotor tilt VTOL.
// Web Serial (USB radio or USB cable), WebSocket or a built-in simulator <-> MAVLink <-> PX4 or ArduPilot.
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const VS = { TO_FW: 1, TO_MC: 2, MC: 3, FW: 4 }; // MAV_VTOL_STATE
  const LS = { GROUND: 1, AIR: 2, TAKEOFF: 3, LANDING: 4 }; // MAV_LANDED_STATE
  const ACK = ['accepted', 'temporarily rejected', 'denied', 'unsupported', 'failed', 'in progress', 'cancelled'];
  const COMMAND_NAMES = { 400: 'Arm/disarm', 176: 'Mode change', 22: 'Takeoff', 3000: 'Transition', 192: 'Fly here', 20: 'Return', 21: 'Land', 512: 'Request', 511: 'Message rate' };
  const TYPE_NAMES = { 1: 'fixed wing', 2: 'quadrotor', 13: 'hexarotor', 14: 'octorotor', 15: 'tricopter', 19: 'VTOL tailsitter', 20: 'VTOL tailsitter', 21: 'VTOL tiltrotor', 22: 'VTOL', 23: 'VTOL tailsitter' };
  const AP_VTOL_MODES = new Set(['QSTABILIZE', 'QHOVER', 'QLOITER', 'QLAND', 'QRTL', 'QACRO', 'QAUTOTUNE', 'LOITER_ALT_QLAND']);
  const esc = (t) => String(t).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const deg = (r) => r * 180 / Math.PI;
  const angDiff = (a, b) => { let d = ((a - b) % 360 + 540) % 360 - 180; return Math.abs(d); };

  // ---------- settings ----------
  const SETTINGS_KEY = 'tri-tilt-gcs-v1';
  const DEFAULTS = {
    m1: 1, m2: 2, m3: 3, tl: 5, tr: 6, p0: 1100, p90: 1900,
    cells: 3, transAlt: 20, takeoffAlt: 20, flyAlt: 40, vStall: 8, vMin: 12, vCruise: 15,
    armA: 180, armB: 360, battG: 165, auwG: 1340,
    voice: true, night: false, kind: 'serial', baud: 57600, ws: 'ws://192.168.4.1:8765',
    camKind: 'none', camUrl: '', camDev: '', camMain: false, sat: true, view: null,
  };
  const settings = Object.assign({}, DEFAULTS, safeJson(localStorage.getItem(SETTINGS_KEY)));
  function safeJson(t) { try { return JSON.parse(t) || {}; } catch (e) { return {}; } }
  function saveSettings() { localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings)); }

  document.querySelectorAll('[data-setting]').forEach((el) => {
    const k = el.dataset.setting;
    el.value = settings[k];
    el.addEventListener('change', () => {
      const v = el.type === 'number' ? Number(el.value) : el.value.trim();
      if (el.type === 'number' && !Number.isFinite(v)) { el.value = settings[k]; return; }
      settings[k] = v;
      saveSettings();
      document.querySelectorAll(`[data-setting="${k}"]`).forEach((o) => { if (o !== el) o.value = v; });
      if (k === 'ws') $('wsUrl').value = v;
      dirty = true;
    });
  });

  // ---------- state ----------
  let link = null;          // { kind, write(bytes), close() }
  let parser = null;
  const enc = new MAV.Encoder(255, 190);
  let tlog = [];
  let s = freshState();
  let dirty = true;
  let timers = [];
  const pending = new Map(); // command id -> retry entry

  function freshState() {
    return {
      target: null, ap: null, version: null, lastHb: 0, connectedAt: 0, hb: null,
      att: null, pos: null, hud: null, wind: null, sys: null, gps: null, servo: null, rc: null,
      radio: null, radioAt: 0, home: null, ext: null, extAt: 0,
      seq: null, buckets: [], rtt: null, rttAt: 0, tsSent: new Map(),
      armedSince: 0, flightMs: 0, lastArmed: false,
      phase: 'nolink', phaseSince: 0, trans: null, wasLost: false,
      params: new Map(), paramMissing: new Set(), paramWrites: new Map(),
      balance: [], battWarn: 0, traffic: new Map(), seen: {},
    };
  }

  // ---------- log, voice, recording ----------
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
    while (ul.children.length > 400) ul.lastChild.remove();
  }

  function say(text) {
    if (!settings.voice || !('speechSynthesis' in window)) return;
    if (speechSynthesis.pending) speechSynthesis.cancel();
    const u = new SpeechSynthesisUtterance(text);
    u.rate = 1.1;
    speechSynthesis.speak(u);
  }

  function recordFrame(raw) {
    const rec = new Uint8Array(8 + raw.length);
    new DataView(rec.buffer).setBigUint64(0, BigInt(Date.now()) * 1000n, false);
    rec.set(raw, 8);
    tlog.push(rec);
  }

  function saveTlog() {
    if (!tlog.length) return;
    const blob = new Blob(tlog, { type: 'application/octet-stream' });
    const a = document.createElement('a');
    const d = new Date(s.connectedAt || Date.now());
    const p = (n) => String(n).padStart(2, '0');
    a.download = `tritilt-${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.tlog`;
    a.href = URL.createObjectURL(blob);
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  }

  // ---------- link ----------
  function send(name, values) {
    if (!link) return false;
    const frame = enc.encode(name, values);
    recordFrame(frame);
    link.write(frame);
    return true;
  }

  function feed(bytes) {
    if (!parser) return;
    bucket().bytes += bytes.length;
    parser.feed(bytes);
  }

  // One-second buckets for packet loss and data rate over the last 10 s.
  function bucket() {
    const sec = Math.floor(Date.now() / 1000);
    let b = s.buckets[s.buckets.length - 1];
    if (!b || b.sec !== sec) {
      b = { sec, rx: 0, lost: 0, bytes: 0 };
      s.buckets.push(b);
      while (s.buckets.length > 11) s.buckets.shift();
    }
    return b;
  }

  function linkStats() {
    const now = Math.floor(Date.now() / 1000);
    let rx = 0, lost = 0, bytes = 0, n = 0;
    for (const b of s.buckets) {
      if (b.sec === now || now - b.sec > 10) continue;
      rx += b.rx; lost += b.lost; bytes += b.bytes; n++;
    }
    return { loss: rx + lost > 20 ? 100 * lost / (rx + lost) : null, rate: n ? bytes / n : null };
  }

  function startLink(l) {
    link = l;
    parser = new MAV.Parser(onMessage);
    clearTraffic();
    s = freshState();
    s.connectedAt = Date.now();
    tlog = [];
    clearTrail();
    timers.push(setInterval(() => send('HEARTBEAT', { type: 6, autopilot: 8, base_mode: 0, custom_mode: 0, system_status: 4, mavlink_version: 3 }), 1000));
    timers.push(setInterval(pingTimesync, 1000));
    timers.push(setInterval(keepStreams, 4000));
    $('btnConnect').hidden = true;
    $('btnDisconnect').hidden = false;
    $('linkKind').disabled = $('baud').disabled = $('wsUrl').disabled = true;
    $('btnLog').disabled = false;
    follow = true;
    renderParamTable();
    dirty = true;
  }

  async function stopLink(reason) {
    if (!link) return;
    const l = link;
    link = null;
    timers.forEach(clearInterval);
    timers = [];
    for (const [, p] of pending) { clearTimeout(p.timer); p.resolve(null); }
    pending.clear();
    try { await l.close(); } catch (e) { /* already gone */ }
    log(reason || 'Disconnected', 'gcs');
    $('btnConnect').hidden = false;
    $('btnDisconnect').hidden = true;
    $('linkKind').disabled = $('baud').disabled = $('wsUrl').disabled = false;
    dirty = true;
  }

  async function connectSerial() {
    if (!('serial' in navigator)) { log('This browser cannot open USB ports. Use Chrome or Edge on a laptop.', 's3'); return; }
    let port;
    try { port = await navigator.serial.requestPort(); } catch (e) { return; } // picker closed
    const baudRate = Number($('baud').value);
    try {
      await port.open({ baudRate, bufferSize: 16384 });
    } catch (e) {
      log(`Could not open the port: ${e.message}. Close QGroundControl / Mission Planner first — only one app can use it.`, 's3');
      return;
    }
    const writer = port.writable.getWriter();
    let reader = null, closing = false;
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
    log(`Port open at ${baudRate} baud. Waiting for a heartbeat…`, 'gcs');
    (async () => {
      while (port.readable && !closing) {
        reader = port.readable.getReader();
        try {
          for (;;) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value) feed(value);
          }
        } catch (e) {
          if (!closing) log(`Serial read error: ${e.message}`, 's4');
        } finally {
          reader.releaseLock();
        }
      }
      if (!closing) stopLink('USB device unplugged');
    })();
  }

  function connectWs() {
    const url = $('wsUrl').value.trim();
    let ws;
    try { ws = new WebSocket(url); } catch (e) { log(`Bad WebSocket address: ${e.message}`, 's3'); return; }
    ws.binaryType = 'arraybuffer';
    log(`Connecting to ${url}…`, 'gcs');
    ws.onopen = () => {
      startLink({ kind: 'ws', write: (b) => { if (ws.readyState === 1) ws.send(b); }, close: () => ws.close() });
      log('WebSocket open. Waiting for a heartbeat…', 'gcs');
    };
    ws.onmessage = (e) => { if (e.data instanceof ArrayBuffer) feed(new Uint8Array(e.data)); };
    ws.onerror = () => { if (!link) log('WebSocket could not connect. Check the address and that the bridge is running.', 's3'); };
    ws.onclose = () => { if (link && link.kind === 'ws') stopLink('WebSocket closed'); };
  }

  // The demo adds a radio-like delay each way. One queue keeps frames in order
  // (separate timers with near-equal deadlines can fire out of order).
  function delayLine(fn) {
    const q = [];
    let timer = 0;
    const pump = () => {
      timer = 0;
      const now = performance.now();
      while (q.length && q[0].at <= now) fn(q.shift().bytes);
      if (q.length) timer = setTimeout(pump, Math.max(1, q[0].at - now));
    };
    return (bytes) => {
      const at = Math.max(q.length ? q[q.length - 1].at : 0, performance.now() + 28 + Math.random() * 18);
      q.push({ at, bytes });
      if (!timer) timer = setTimeout(pump, Math.max(1, at - performance.now()));
    };
  }

  function connectDemo(autopilot) {
    const toGcs = delayLine((b) => feed(b));
    const sim = new SimVehicle(toGcs, { autopilot });
    const toSim = delayLine((b) => sim.write(b));
    startLink({ kind: 'demo', write: toSim, close: () => sim.stop() });
    log(`Demo: ${autopilot === 'px4' ? 'PX4' : 'ArduPilot'} simulator with ~70 ms simulated radio round trip.`, 'gcs');
    sim.start();
  }

  function connect() {
    const k = $('linkKind').value;
    settings.kind = k;
    settings.baud = Number($('baud').value);
    saveSettings();
    if (k === 'serial') connectSerial();
    else if (k === 'ws') connectWs();
    else connectDemo(k === 'demo-px4' ? 'px4' : 'ardupilot');
  }

  // ---------- commands ----------
  const linked = () => !!(link && s.target && s.lastHb && Date.now() - s.lastHb < 3500);

  function cmd(command, params, opts) {
    opts = opts || {};
    return new Promise((resolve) => {
      if (!s.target || !link) { if (!opts.quiet) log('Not sent: no aircraft connected.', 's4'); resolve(null); return; }
      const p = params.concat([0, 0, 0, 0, 0, 0, 0]).slice(0, 7);
      const values = { command, target_system: s.target.sys, target_component: s.target.comp, confirmation: 0 };
      p.forEach((v, i) => { values['param' + (i + 1)] = v; });
      sendTracked(command, 'COMMAND_LONG', values, opts, resolve);
    });
  }

  function sendTracked(command, name, values, opts, resolve) {
    const prev = pending.get(command);
    if (prev) { clearTimeout(prev.timer); prev.resolve(null); }
    const entry = { label: opts.label || COMMAND_NAMES[command] || 'Command ' + command, tries: 0, quiet: opts.quiet, resolve };
    pending.set(command, entry);
    const retries = opts.retries === undefined ? 2 : opts.retries;
    const attempt = () => {
      if (!link) { pending.delete(command); resolve(null); return; }
      if (name === 'COMMAND_LONG') values.confirmation = Math.min(entry.tries, 255);
      send(name, values);
      entry.tries++;
      entry.timer = setTimeout(() => {
        if (pending.get(command) !== entry) return;
        if (entry.tries <= retries) attempt();
        else {
          pending.delete(command);
          if (!opts.quiet) log(`${entry.label}: no reply from the aircraft`, 's3');
          resolve(null);
        }
      }, 1500);
    };
    attempt();
  }

  function waitFor(pred, ms) {
    return new Promise((resolve) => {
      const t0 = Date.now();
      const iv = setInterval(() => {
        if (pred()) { clearInterval(iv); resolve(true); }
        else if (Date.now() - t0 > ms || !link) { clearInterval(iv); resolve(false); }
      }, 100);
    });
  }

  function pingTimesync() {
    if (!s.target) return;
    const ns = Math.round(performance.now() * 1e6);
    s.tsSent.set(ns, performance.now());
    for (const [k, t] of s.tsSent) if (performance.now() - t > 10000) s.tsSent.delete(k);
    send('TIMESYNC', { tc1: 0, ts1: ns });
  }

  // Ask for the messages this screen needs at sensible rates for a 57600-baud radio (about 1.5 kB/s).
  const STREAMS = [[30, 10], [33, 4], [74, 4], [1, 2], [24, 1], [36, 4], [65, 2], [245, 2], [242, 0.2]];
  function requestStreams() {
    if (!s.target) return;
    const extra = s.ap === 'px4' ? [[231, 1]] : [[168, 1]];
    let i = 0;
    for (const [id, hz] of STREAMS.concat(extra)) {
      setTimeout(() => cmd(511, [id, Math.round(1e6 / hz)], { quiet: true, retries: 0 }), 60 * i++);
    }
    if (s.ap === 'ardupilot') {
      // Older ArduPilot builds and SRx_ setups still listen to REQUEST_DATA_STREAM.
      for (const [id, hz] of [[2, 2], [3, 2], [6, 4], [10, 10], [11, 4], [12, 1]]) {
        send('REQUEST_DATA_STREAM', { req_message_rate: hz, target_system: s.target.sys, target_component: s.target.comp, req_stream_id: id, start_stop: 1 });
      }
    }
    s.streamsAt = Date.now();
  }

  function keepStreams() {
    if (!linked()) return;
    const now = Date.now();
    const missing = ['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'SYS_STATUS', 'SERVO_OUTPUT_RAW'].some((n) => !s.seen[n] || now - s.seen[n] > 5000);
    if (missing && now - (s.streamsAt || 0) > 8000) requestStreams();
    if (!s.home && now - (s.homeAsked || 0) > 5000) requestHome();
  }

  function requestHome() { s.homeAsked = Date.now(); cmd(512, [242], { quiet: true, retries: 0 }); }

  // ---------- autopilot adapters ----------
  const PX4 = {
    name: 'PX4',
    modeName: (cm) => MAV.px4Mode(cm),
    modes: [
      ['HOLD', 'Hold'], ['RTL', 'Return'], ['LAND', 'Land'],
      ['POSCTL', 'Position', 'rc'], ['ALTCTL', 'Altitude', 'rc'], ['STABILIZED', 'Stabilized', 'rc'],
      ['TAKEOFF', 'Takeoff'], ['MISSION', 'Mission'], ['ACRO', 'Acro', 'rc'],
    ],
    note: 'Position, Altitude, Stabilized and Acro fly from the RC sticks: PX4 refuses them (or fails safe) with no transmitter connected.',
    setMode(name) {
      const [main, sub] = MAV.PX4_MODES[name];
      log(`Requesting ${name}…`, 'gcs');
      return cmd(176, [1, main, sub], { label: `Mode ${name}` });
    },
    prepareTakeoff: async () => true,
    takeoff(alt) {
      // PX4 wants an altitude above sea level; NaN lets it use MIS_TAKEOFF_ALT.
      const amsl = s.home && Number.isFinite(s.home.amsl) ? s.home.amsl + alt : NaN;
      if (!Number.isFinite(amsl)) log('Home altitude unknown: PX4 will climb to its MIS_TAKEOFF_ALT instead.', 's4');
      return cmd(22, [0, 0, 0, NaN, NaN, NaN, amsl], { label: `Takeoff to ${alt} m` });
    },
    toPlane: () => cmd(3000, [VS.FW], { label: 'Transition to plane' }),
    toHover: () => cmd(3000, [VS.MC], { label: 'Back to hover' }),
    hold: () => PX4.setMode('HOLD'),
    rtl: () => PX4.setMode('RTL'),
    async land() {
      if ([VS.FW, VS.TO_FW].includes(vtolState())) {
        log('Back to hover first, then land.', 'gcs');
        await PX4.toHover();
        if (!(await waitFor(() => vtolState() === VS.MC, 15000))) { log('Still not in hover: land cancelled. Use Back to hover, then Land.', 's3'); return null; }
      }
      return PX4.setMode('LAND');
    },
    flyTo(lat, lon, rel) {
      const z = s.home && Number.isFinite(s.home.amsl) ? s.home.amsl + rel : NaN;
      return sendInt(192, [-1, 1, 0, NaN], lat, lon, z, 0, 'Fly here');
    },
    flyHereText: () => (vtolState() === VS.FW
      ? 'It flies there as a plane and circles the point (Hold mode).'
      : 'It flies there as a multicopter (about 5 m/s) and hovers there (Hold mode).'),
    params: [
      ['Transition without an airspeed sensor'],
      ['FW_USE_AIRSPD', 'Use an airspeed sensor (PX4 1.15+). 0 = no.', 0],
      ['FW_ARSP_MODE', 'Same thing on PX4 1.14: 1 = don\'t use airspeed.', 1],
      ['SYS_HAS_NUM_ASPD', 'Airspeed sensors needed to arm.', 0],
      ['VT_F_TR_OL_TM', 'Front transition time with no airspeed (s). PX4 finishes the transition on this timer.', 6],
      ['VT_TRANS_MIN_TM', 'Shortest front transition allowed (s).', 2],
      ['VT_TRANS_TIMEOUT', 'Give up and go back to hover after this long (s).', 15],
      ['VT_B_TRANS_DUR', 'Back transition time (s).', 4],
      ['VT_F_TRANS_DUR', 'Time to tilt fully forward at the end (s).', null],
      ['Tilt'],
      ['VT_TILT_MC', 'Tilt in hover (0 = straight up).', 0],
      ['VT_TILT_TRANS', 'Tilt held while speeding up (0.4 ≈ 36°).', 0.4],
      ['VT_TILT_FW', 'Tilt in plane mode (1 = 90°).', 1],
      ['VT_TILT_SPINUP', 'Tilt while spinning up on the ground.', 0],
      ['Speeds'],
      ['FW_AIRSPD_STALL', 'Stall speed (m/s).', 8],
      ['FW_AIRSPD_MIN', 'Slowest plane speed (m/s).', 12],
      ['FW_AIRSPD_TRIM', 'Cruise speed (m/s).', 15],
      ['Safety'],
      ['VT_FW_MIN_ALT', 'Quadchute (go back to hover) below this height in plane mode (m). 0 = off.', 10],
      ['VT_FW_QC_P', 'Quadchute at this pitch angle (°). 0 = off.', 30],
      ['VT_FW_QC_R', 'Quadchute at this roll angle (°). 0 = off.', 45],
      ['MIS_TAKEOFF_ALT', 'Default takeoff height (m).', 20],
      ['RTL_RETURN_ALT', 'Return-home height (m).', 40],
      ['RTL_DESCEND_ALT', 'Height to slow down at before landing (m).', 20],
      ['NAV_RCL_ACT', 'RC lost: 2 = Return.', 2],
      ['NAV_DLL_ACT', 'Telemetry lost: 0 = carry on (RC is primary).', 0],
      ['COM_DISARM_PRFLT', 'Auto-disarm if no takeoff within (s).', 10],
      ['BAT1_N_CELLS', 'Battery cells.', 3],
    ],
    speedParams: { stall: 'FW_AIRSPD_STALL', min: 'FW_AIRSPD_MIN', cruise: 'FW_AIRSPD_TRIM' },
    transTime() {
      const noAs = pv('FW_USE_AIRSPD') === 0 || pv('FW_ARSP_MODE') === 1 || (s.sys && !(s.sys.sensors_present & 16));
      if (!noAs) return null;
      return Math.max(pv('VT_F_TR_OL_TM') ?? 6, pv('VT_TRANS_MIN_TM') ?? 0);
    },
    backTime: () => pv('VT_B_TRANS_DUR') ?? 4,
    transTimeout: () => pv('VT_TRANS_TIMEOUT') ?? 15,
    autoDisarm: () => pv('COM_DISARM_PRFLT') ?? 10,
  };

  const AP = {
    name: 'ArduPilot',
    modeName: (cm) => MAV.PLANE_MODES[cm] || 'MODE ' + cm,
    modes: [
      ['QLOITER', 'QLoiter'], ['QHOVER', 'QHover', 'rc'], ['QSTABILIZE', 'QStabilize', 'rc'],
      ['QLAND', 'QLand'], ['QRTL', 'QRTL'], ['GUIDED', 'Guided'],
      ['LOITER', 'Loiter'], ['CRUISE', 'Cruise', 'rc'], ['FBWA', 'FBWA', 'rc'],
      ['AUTO', 'Auto'], ['RTL', 'RTL'], ['MANUAL', 'Manual', 'rc'],
    ],
    note: 'Q modes fly as a multicopter. Picking a plane mode while hovering starts a transition.',
    setMode(name) {
      log(`Requesting ${name}…`, 'gcs');
      return cmd(176, [1, MAV.MODE_NUMBERS[name]], { label: `Mode ${name}` });
    },
    async prepareTakeoff() {
      if (modeName() === 'GUIDED') return true;
      await AP.setMode('GUIDED');
      return waitFor(() => modeName() === 'GUIDED', 3000);
    },
    takeoff: (alt) => cmd(22, [0, 0, 0, 0, 0, 0, alt], { label: `Takeoff to ${alt} m` }),
    toPlane: () => AP.setMode('LOITER'),
    toHover: () => AP.setMode('QLOITER'),
    hold: () => AP.setMode([VS.FW, VS.TO_FW].includes(vtolState()) ? 'LOITER' : 'QLOITER'),
    rtl: () => AP.setMode('QRTL'),
    land: () => AP.setMode('QLAND'),
    flyTo: (lat, lon, rel) => sendInt(192, [-1, 1, 0, NaN], lat, lon, rel, 3, 'Fly here'),
    flyHereText: () => (vtolState() === VS.MC
      ? 'It switches to GUIDED, which TRANSITIONS to plane mode to get there (with Q_GUIDED_MODE=1 it hovers on arrival). Only with enough height.'
      : 'It switches to GUIDED and circles the point.'),
    params: [
      ['QuadPlane tilt'],
      ['Q_ENABLE', 'QuadPlane on.', 1], ['Q_FRAME_CLASS', '7 = tri.', 7], ['Q_TILT_MASK', 'Tilting motors (bitmask): 3 = motors 1+2.', 3],
      ['Q_TILT_TYPE', '2 = vectored yaw.', 2], ['Q_TILT_MAX', 'Tilt held while speeding up (°).', null],
      ['Q_TILT_RATE_UP', 'Tilt-up rate (°/s).', null], ['Q_TILT_RATE_DN', 'Tilt-down rate (°/s).', null],
      ['Transition without an airspeed sensor'],
      ['ARSPD_TYPE', '0 = no airspeed sensor.', 0], ['AIRSPEED_MIN', 'Speed that ends the transition (m/s).', 12],
      ['AIRSPEED_CRUISE', 'Cruise speed (m/s).', 15], ['Q_TRANSITION_MS', 'Blend time after reaching speed (ms).', null],
      ['Q_TRANS_FAIL', 'Give up the transition after (s). 0 = never.', 20],
      ['Safety'],
      ['Q_ASSIST_SPEED', 'Motors help below this speed (m/s).', null], ['Q_ASSIST_ANGLE', 'Motors help past this angle (°).', 30],
      ['Q_ASSIST_ALT', 'Motors help below this height (m).', 15], ['Q_RTL_ALT', 'QRTL height (m).', null],
      ['Q_GUIDED_MODE', 'Hover at GUIDED targets.', 1], ['FS_GCS_ENABL', 'Telemetry-loss failsafe (0 = off).', 0],
    ],
    speedParams: { stall: null, min: 'AIRSPEED_MIN', cruise: 'AIRSPEED_CRUISE' },
    transTime: () => null,
    backTime: () => 4,
    transTimeout: () => pv('Q_TRANS_FAIL') || null,
    autoDisarm: () => null,
  };

  const ad = () => (s.ap === 'ardupilot' ? AP : PX4);
  const pv = (name) => { const p = s.params.get(name); return p ? p.v : undefined; };

  function sendInt(command, p, lat, lon, z, frame, label) {
    return new Promise((resolve) => {
      if (!s.target) { resolve(null); return; }
      const values = {
        param1: p[0], param2: p[1], param3: p[2], param4: p[3],
        x: Math.round(lat * 1e7), y: Math.round(lon * 1e7), z,
        command, target_system: s.target.sys, target_component: s.target.comp, frame, current: 0, autocontinue: 0,
      };
      sendTracked(command, 'COMMAND_INT', values, { label }, resolve);
    });
  }

  // ---------- incoming ----------
  function onMessage(m, hdr, raw) {
    recordFrame(raw);
    if (m._name === 'RADIO_STATUS') { s.radio = m; s.radioAt = Date.now(); return; }
    if (m._name === 'HEARTBEAT') {
      if (m.type === 6 || m.autopilot === 8) return; // another GCS, a camera, the ADS-B receiver…
      if (!s.target) {
        s.target = { sys: hdr.sysid, comp: hdr.compid };
        s.ap = m.autopilot === 12 ? 'px4' : m.autopilot === 3 ? 'ardupilot' : 'other';
        log(`Found ${s.ap === 'px4' ? 'PX4' : s.ap === 'ardupilot' ? 'ArduPilot' : 'an autopilot'} (system ${hdr.sysid}, ${TYPE_NAMES[m.type] || 'type ' + m.type})`, 'gcs');
        if (s.ap === 'other') log('Unknown autopilot: treating it like PX4.', 's4');
        if (s.ap === 'ardupilot' && [2, 3, 4, 13, 14, 15].includes(m.type)) log('This looks like ArduCopter. A tilt VTOL needs ArduPlane with Q_ENABLE=1.', 's4');
        if (s.ap === 'px4' && m.type !== 21) log(`PX4 reports frame type ${TYPE_NAMES[m.type] || m.type}, not a tiltrotor. Check the airframe in QGroundControl.`, 's4');
        buildModeButtons();
        renderParamTable();
        setTimeout(() => cmd(512, [148], { quiet: true, retries: 1 }), 300);
        setTimeout(readParams, 2500);
      }
    }
    if (!s.target || hdr.sysid !== s.target.sys) return;

    if (hdr.compid === s.target.comp) {
      const b = bucket();
      if (s.seq !== null) b.lost += (hdr.seq - s.seq - 1) & 0xff;
      s.seq = hdr.seq;
      b.rx++;
    }
    if (m._name === 'ADSB_VEHICLE') { onTraffic(m); return; }
    s.seen[m._name] = Date.now();

    switch (m._name) {
      case 'HEARTBEAT': {
        if (hdr.compid !== s.target.comp) return;
        const now = Date.now();
        const fresh = !s.lastHb || now - s.lastHb > 5000;
        const prev = s.hb;
        s.hb = m;
        s.lastHb = now;
        if (fresh) {
          if (s.wasLost) { log('Telemetry back.', 'gcs'); say('Telemetry back'); s.wasLost = false; }
          requestStreams();
          requestHome();
        }
        const armed = (m.base_mode & 128) !== 0;
        if (prev && armed !== ((prev.base_mode & 128) !== 0)) {
          log(armed ? 'ARMED' : 'DISARMED', armed ? 's4' : 'gcs');
          say(armed ? 'Armed' : 'Disarmed');
          if (armed) { requestHome(); s.balance = []; }
        }
        if (prev && prev.custom_mode !== m.custom_mode) log(`Mode → ${modeName()}`, 'gcs');
        break;
      }
      case 'ATTITUDE': s.att = m; break;
      case 'GLOBAL_POSITION_INT': s.pos = m; addTrail(m); break;
      case 'VFR_HUD': s.hud = m; break;
      case 'SYS_STATUS': s.sys = m; break;
      case 'GPS_RAW_INT': s.gps = m; break;
      case 'SERVO_OUTPUT_RAW': if (m.port === 0) s.servo = m; break;
      case 'RC_CHANNELS': s.rc = m; break;
      case 'EXTENDED_SYS_STATE': s.ext = m; s.extAt = Date.now(); break;
      case 'WIND':
        s.wind = { from: ((m.direction % 360) + 360) % 360, speed: m.speed };
        break;
      case 'WIND_COV': {
        // wind_x / wind_y: the velocity the air moves with (north, east). "From" is the opposite way.
        if (!Number.isFinite(m.wind_x) || !Number.isFinite(m.wind_y)) { s.wind = null; break; }
        const sp = Math.hypot(m.wind_x, m.wind_y);
        s.wind = { from: (deg(Math.atan2(-m.wind_y, -m.wind_x)) + 360) % 360, speed: sp };
        break;
      }
      case 'HOME_POSITION':
        s.home = { lat: m.latitude / 1e7, lon: m.longitude / 1e7, amsl: m.altitude / 1000 };
        updateHomeMarker();
        break;
      case 'TIMESYNC': {
        if (m.tc1 === 0) break; // the autopilot asking us; not needed
        const sent = s.tsSent.get(m.ts1);
        if (sent === undefined) break;
        s.tsSent.delete(m.ts1);
        const rtt = performance.now() - sent;
        s.rtt = s.rtt === null ? rtt : s.rtt * 0.7 + rtt * 0.3;
        s.rttLast = rtt;
        s.rttAt = Date.now();
        break;
      }
      case 'AUTOPILOT_VERSION': {
        const v = m.flight_sw_version;
        s.version = `${(v >>> 24) & 255}.${(v >>> 16) & 255}.${(v >>> 8) & 255}`;
        log(`${ad().name} firmware ${s.version}`, 'gcs');
        break;
      }
      case 'PARAM_VALUE': onParam(m); break;
      case 'STATUSTEXT':
        log(m.text, 's' + m.severity);
        if (m.severity <= 2) say(m.text);
        break;
      case 'COMMAND_ACK': {
        const p = pending.get(m.command);
        if (m.result === 5) break; // in progress
        if (p) { clearTimeout(p.timer); pending.delete(m.command); p.resolve(m.result); }
        if (p && p.quiet && m.result === 0) break;
        if (!p && (m.command === 511 || m.command === 512)) break;
        const what = (p && p.label) || COMMAND_NAMES[m.command] || 'Command ' + m.command;
        log(`${what}: ${ACK[m.result] || 'result ' + m.result}`, m.result === 0 ? 'gcs' : 's4');
        if (m.result !== 0 && m.command !== 511 && m.command !== 512) say(`${what} ${ACK[m.result] || 'failed'}`);
        break;
      }
    }
    dirty = true;
  }

  // ---------- derived state ----------
  const isArmed = () => !!(s.hb && (s.hb.base_mode & 128));
  const modeName = () => (s.hb ? ad().modeName(s.hb.custom_mode) : '—');
  const relAlt = () => (s.pos ? s.pos.relative_alt / 1000 : null);
  const extFresh = () => s.ext && Date.now() - s.extAt < 5000;
  const servoPwm = (ch) => (s.servo ? s.servo['servo' + ch + '_raw'] || 0 : 0);
  const pwmToTilt = (pwm) => (pwm ? (pwm - settings.p0) / (settings.p90 - settings.p0) * 90 : null);
  const motorPct = (ch) => { const p = servoPwm(ch); return p ? clamp((p - 1000) / 10, 0, 100) : null; };
  function frontTilt() {
    const l = pwmToTilt(servoPwm(settings.tl)), r = pwmToTilt(servoPwm(settings.tr));
    if (l === null && r === null) return null;
    return l === null ? r : r === null ? l : (l + r) / 2;
  }

  function vtolState() {
    if (extFresh() && s.ext.vtol_state) return s.ext.vtol_state;
    if (!s.hb) return 0;
    if (s.ap === 'ardupilot') {
      const t = frontTilt();
      if (AP_VTOL_MODES.has(modeName())) return t !== null && t > 15 ? VS.TO_MC : VS.MC;
      return t !== null && t < 75 && (relAlt() || 0) > 1 ? VS.TO_FW : VS.FW;
    }
    return VS.MC;
  }

  function landedState() {
    if (extFresh() && s.ext.landed_state) return s.ext.landed_state;
    const a = relAlt();
    if (a === null) return 0;
    return a < 0.7 ? LS.GROUND : LS.AIR;
  }

  function computePhase() {
    if (!link) return 'nolink';
    if (!s.hb) return 'waiting';
    if (!isArmed()) return 'disarmed';
    const ls = landedState(), vs = vtolState();
    if (vs === VS.TO_FW) return 'to-plane';
    if (vs === VS.TO_MC) return 'to-hover';
    if (vs === VS.FW && ls !== LS.GROUND) return 'plane';
    if (ls === LS.GROUND) return 'armed';
    if (ls === LS.TAKEOFF) return 'takeoff';
    if (ls === LS.LANDING) return 'landing';
    return 'hover';
  }

  const PHASE_STEP = { nolink: 'ground', waiting: 'ground', disarmed: 'ground', armed: 'ground', takeoff: 'hover', hover: 'hover', landing: 'hover', 'to-plane': 'trans', 'to-hover': 'trans', plane: 'plane' };
  const PHASE_SAY = { hover: 'Hover', 'to-plane': 'Transition to plane', plane: 'Plane mode', 'to-hover': 'Transition to hover', landing: 'Landing', takeoff: 'Taking off' };
  const AIR = new Set(['takeoff', 'hover', 'landing', 'to-plane', 'to-hover', 'plane']);

  function updatePhase() {
    const p = computePhase();
    if (p === s.phase) return;
    const prev = s.phase;
    s.phase = p;
    s.phaseSince = Date.now();
    if (p === 'to-plane' || p === 'to-hover') {
      s.trans = { kind: p, t0: Date.now(), alt0: relAlt() || 0, warned: false };
    }
    if (PHASE_SAY[p] && prev !== 'nolink' && prev !== 'waiting') { say(PHASE_SAY[p]); log(`Phase: ${phaseLabel(p)}`, 'gcs'); }
    if (p === 'plane' && s.trans) log(`Transition took ${((Date.now() - s.trans.t0) / 1000).toFixed(1)} s`, 'gcs');
    newTrailSegment();
  }

  function phaseLabel(p) {
    return { nolink: 'NO LINK', waiting: 'WAITING', disarmed: 'ON GROUND', armed: 'ARMED', takeoff: 'TAKING OFF', hover: 'HOVER', landing: 'LANDING', 'to-plane': '→ PLANE', 'to-hover': '→ HOVER', plane: 'PLANE' }[p];
  }

  function speeds() {
    const sp = ad().speedParams;
    const get = (k, fb) => { const v = sp[k] && pv(sp[k]); return Number.isFinite(v) && v > 0 ? v : fb; };
    return { stall: get('stall', settings.vStall), min: get('min', settings.vMin), cruise: get('cruise', settings.vCruise) };
  }

  // ---------- actions ----------
  async function ask(title, bodyHtml, okLabel, dangerous) {
    const dlg = $('dlg');
    $('dlgTitle').textContent = title;
    $('dlgBody').innerHTML = bodyHtml;
    $('dlgOk').textContent = okLabel || 'OK';
    $('dlgOk').className = 'btn ' + (dangerous ? 'dangerous' : 'primary');
    return new Promise((resolve) => {
      dlg.addEventListener('close', () => resolve(dlg.returnValue === 'ok'), { once: true });
      dlg.returnValue = 'cancel';
      dlg.showModal();
      $('dlgCancel').focus();
    });
  }

  function checksHtml(list) {
    return '<ul class="checks">' + list.map((c) => `<li class="${c.st}"><span>${esc(c.label)}${c.detail ? `<small>${esc(c.detail)}</small>` : ''}</span></li>`).join('') + '</ul>';
  }

  async function doTakeoff() {
    const alt = clamp(Number(settings.takeoffAlt) || 20, 3, 120);
    const bad = groundChecks().filter((c) => c.st === 'bad');
    if (bad.length && !(await ask('Take off anyway?', `<p>These checks are failing:</p>${checksHtml(bad)}`, 'Take off anyway', true))) return;
    log(`Takeoff to ${alt} m…`, 'gcs');
    if (!(await ad().prepareTakeoff())) { log('Takeoff cancelled: could not switch mode.', 's3'); return; }
    if (!isArmed()) {
      await cmd(400, [1], { label: 'Arm' });
      if (!(await waitFor(isArmed, 3000))) { log('Takeoff cancelled: the aircraft did not arm (see messages above).', 's3'); return; }
    }
    await ad().takeoff(alt);
  }

  async function doToPlane() {
    const list = transChecks();
    const bad = list.filter((c) => c.st === 'bad').length;
    const T = ad().transTime();
    const sp = speeds();
    const body = `<p>The front motors tilt forward and the aircraft speeds up. ${T ? `${ad().name} finishes the transition on a ${T.toFixed(1)} s timer (no airspeed sensor), so it must be pointing into the wind with room ahead.` : `It finishes once the speed passes ${sp.min} m/s.`} The rear motor stops in plane mode.</p>${checksHtml(list)}<p><b>Abort</b> with <i>Back to hover</i> if it sinks or doesn't pick up speed.</p>`;
    if (!(await ask(bad ? 'Not all checks pass' : 'Transition to plane?', body, bad ? 'Transition anyway' : 'Start transition', !!bad))) return;
    ad().toPlane();
  }

  async function doToHover() {
    ad().toHover();
  }

  async function doFlyHere() {
    if (!tgt) return;
    const alt = clamp(Number(settings.flyAlt) || 40, 5, 400);
    const d = s.pos ? distance(s.pos.lat / 1e7, s.pos.lon / 1e7, tgt.lat, tgt.lng) : null;
    const body = `<p>${d !== null ? `${d.toFixed(0)} m away, ` : ''}at ${alt} m above home.</p><p>${esc(ad().flyHereText())}</p>`;
    if (!(await ask('Fly here?', body, 'Fly here', vtolState() === VS.MC && s.ap === 'ardupilot'))) return;
    log(`Fly here: ${tgt.lat.toFixed(6)}, ${tgt.lng.toFixed(6)} at ${alt} m`, 'gcs');
    ad().flyTo(tgt.lat, tgt.lng, alt);
  }

  async function doModeButton(name, needsRc) {
    const a = ad();
    if (s.ap === 'ardupilot' && isArmed() && vtolState() === VS.MC && !AP_VTOL_MODES.has(name) && name !== 'RTL' && AIR.has(s.phase)) {
      if (!(await ask(`Switch to ${name}?`, '<p>From a Q mode this starts a <b>transition</b> to plane mode. Only with enough height and room ahead.</p>', 'Switch', true))) return;
    } else if (needsRc && isArmed()) {
      if (!(await ask(`Switch to ${name}?`, '<p>This mode flies from the RC sticks. With no transmitter connected the autopilot refuses it or fails safe.</p>', 'Switch'))) return;
    }
    a.setMode(name);
  }

  async function doDisarm() {
    if (AIR.has(s.phase) && !(await ask('Disarm in the air?', '<p>The aircraft looks airborne. The autopilot normally refuses this. Use the emergency stop if you really need the motors off.</p>', 'Send disarm', true))) return;
    log('Requesting DISARM…', 'gcs');
    cmd(400, [0], { label: 'Disarm' });
  }

  async function doKill() {
    if (!(await ask('Emergency motor stop', '<p><b>All motors stop immediately.</b> If it is flying, it falls. Use this only to prevent injury or a flyaway.</p>', 'STOP MOTORS', true))) return;
    log('EMERGENCY STOP sent', 's2');
    cmd(400, [0, 21196], { label: 'Emergency stop' });
  }

  // Press-and-hold buttons: the action fires only after the hold time, so a stray tap does nothing.
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
    const begin = () => { if (btn.disabled || start) return; start = performance.now(); raf = requestAnimationFrame(tick); };
    btn.addEventListener('pointerdown', (e) => { e.preventDefault(); begin(); });
    ['pointerup', 'pointerleave', 'pointercancel', 'blur'].forEach((ev) => btn.addEventListener(ev, cancel));
    btn.addEventListener('keydown', (e) => { if ((e.key === ' ' || e.key === 'Enter') && !e.repeat) { e.preventDefault(); begin(); } else if (e.key === ' ' || e.key === 'Enter') e.preventDefault(); });
    btn.addEventListener('keyup', (e) => { if (e.key === ' ' || e.key === 'Enter') cancel(); });
    btn.addEventListener('click', (e) => e.preventDefault());
  }

  function buildModeButtons() {
    const a = ad();
    const box = $('modeGrid');
    box.innerHTML = '';
    for (const [name, label, rc] of a.modes) {
      const b = document.createElement('button');
      b.className = 'btn';
      b.dataset.mode = name;
      b.textContent = label;
      b.title = name + (rc ? ' — needs RC sticks' : '');
      b.addEventListener('click', () => doModeButton(name, !!rc));
      box.appendChild(b);
    }
    $('modeNote').textContent = a.note;
  }

  // ---------- readiness checks ----------
  function cellV() {
    if (!s.sys || s.sys.voltage_battery === 65535 || !s.sys.voltage_battery) return null;
    return s.sys.voltage_battery / 1000 / settings.cells;
  }

  function common() {
    const out = [];
    const hbAge = s.lastHb ? (Date.now() - s.lastHb) / 1000 : null;
    const st = linkStats();
    out.push({ label: 'Telemetry link', st: hbAge === null ? 'bad' : hbAge > 3 ? 'bad' : (st.loss || 0) > 15 || (s.rtt || 0) > 800 ? 'warn' : 'ok', detail: hbAge === null ? 'no heartbeat' : `${s.rtt !== null ? s.rtt.toFixed(0) + ' ms round trip, ' : ''}${st.loss !== null ? st.loss.toFixed(1) + '% loss' : ''}` });
    const g = s.gps;
    out.push({ label: 'GPS', st: !g ? 'bad' : g.fix_type < 3 ? 'bad' : g.satellites_visible < 8 ? 'warn' : 'ok', detail: g ? `${['no GPS', 'no fix', '2D', '3D', 'DGPS', 'RTK float', 'RTK fixed'][g.fix_type] || g.fix_type} · ${g.satellites_visible} satellites` : 'no data' });
    return out;
  }

  function battCheck(rest) {
    const c = cellV();
    const ok = rest ? 3.75 : 3.6, warn = rest ? 3.6 : 3.45;
    return { label: 'Battery', st: c === null ? 'warn' : c >= ok ? 'ok' : c >= warn ? 'warn' : 'bad', detail: c === null ? 'no voltage reading' : `${c.toFixed(2)} V per cell${rest ? ' (resting)' : ' (under load)'}` };
  }

  function groundChecks() {
    const out = common();
    out.push({ label: 'Home position set', st: s.home ? 'ok' : 'bad', detail: s.home ? '' : 'waits for a GPS fix' });
    out.push(battCheck(true));
    if (s.att) {
      const tilt = Math.max(Math.abs(deg(s.att.roll)), Math.abs(deg(s.att.pitch)));
      out.push({ label: 'Sitting level', st: tilt < 8 ? 'ok' : 'warn', detail: `${tilt.toFixed(0)}° off level` });
    }
    const t = frontTilt();
    if (t !== null && isArmed()) out.push({ label: 'Front motors pointing up', st: Math.abs(t) < 10 ? 'ok' : 'bad', detail: `tilt ${t.toFixed(0)}°` });
    if (s.wind) out.push({ label: 'Wind', st: s.wind.speed < 6 ? 'ok' : s.wind.speed < 9 ? 'warn' : 'bad', detail: `${s.wind.speed.toFixed(1)} m/s` });
    return out;
  }

  function transChecks() {
    const out = [];
    const a = relAlt();
    out.push({ label: `Height ≥ ${settings.transAlt} m`, st: a === null ? 'bad' : a >= settings.transAlt ? 'ok' : 'bad', detail: a === null ? '' : `now ${a.toFixed(0)} m` });
    out.push(...common());
    out.push(battCheck(false));
    if (s.att) {
      const tilt = Math.max(Math.abs(deg(s.att.roll)), Math.abs(deg(s.att.pitch)));
      const climb = s.hud ? Math.abs(s.hud.climb) : 0;
      out.push({ label: 'Steady hover', st: tilt < 12 && climb < 1.2 ? 'ok' : 'warn', detail: `${tilt.toFixed(0)}° lean, ${climb.toFixed(1)} m/s vertical` });
    }
    if (s.wind && s.wind.speed > 1.5 && s.hud) {
      const off = angDiff(s.hud.heading, s.wind.from);
      out.push({ label: 'Nose into the wind', st: off < 45 ? 'ok' : off < 100 ? 'warn' : 'bad', detail: off < 45 ? `${off.toFixed(0)}° off the wind` : `turn to face ${s.wind.from.toFixed(0)}° (${compass(s.wind.from)})` });
    } else {
      out.push({ label: 'Nose into the wind', st: 'warn', detail: 'wind not known yet: point it into the wind yourself' });
    }
    const T = ad().transTime() || 6, sp = speeds();
    out.push({ label: `Clear ahead: about ${Math.round((0.5 * T + 3) * sp.cruise / 10) * 10} m`, st: '', detail: 'distance covered while speeding up and settling' });
    return out;
  }

  function planeChecks() {
    const out = [];
    const sp = speeds();
    const v = s.hud ? s.hud.airspeed : null;
    out.push({ label: `Speed above ${sp.min} m/s`, st: v === null ? 'warn' : v >= sp.min ? 'ok' : v >= sp.stall ? 'warn' : 'bad', detail: v === null ? '' : `${v.toFixed(1)} m/s${s.sys && !(s.sys.sensors_present & 16) ? ' (estimated)' : ''}` });
    const a = relAlt();
    out.push({ label: `Height ≥ ${settings.transAlt} m`, st: a === null ? 'warn' : a >= settings.transAlt ? 'ok' : 'warn', detail: a === null ? '' : `${a.toFixed(0)} m` });
    out.push(...common(), battCheck(false));
    return out;
  }

  // ---------- params ----------
  function readParams() {
    if (!s.target || !link) return;
    const names = ad().params.filter((p) => p.length > 1).map((p) => p[0]);
    s.paramMissing.clear();
    $('paramTx').textContent = 'Reading…';
    names.forEach((n, i) => setTimeout(() => requestParam(n), i * 45));
    // Ask again for anything that didn't answer, then mark the rest as not on this firmware.
    setTimeout(() => names.filter((n) => !s.params.has(n)).forEach((n, i) => setTimeout(() => requestParam(n), i * 45)), names.length * 45 + 2500);
    setTimeout(() => {
      names.filter((n) => !s.params.has(n)).forEach((n) => s.paramMissing.add(n));
      $('paramTx').textContent = `${names.filter((n) => s.params.has(n)).length} of ${names.length} read`;
      renderParamTable();
    }, names.length * 90 + 6000);
  }

  function requestParam(name) {
    send('PARAM_REQUEST_READ', { param_index: -1, target_system: s.target.sys, target_component: s.target.comp, param_id: name });
  }

  function onParam(m) {
    const name = m.param_id;
    const v = MAV.paramFromWire(m.param_value, m.param_type, s.ap === 'px4');
    const val = m.param_type === 9 ? Math.round(v * 1e5) / 1e5 : v;
    s.params.set(name, { v: val, t: m.param_type });
    s.paramMissing.delete(name);
    const w = s.paramWrites.get(name);
    if (w) {
      clearTimeout(w.timer);
      s.paramWrites.delete(name);
      const ok = Math.abs(val - w.value) < 1e-4 * Math.max(1, Math.abs(w.value));
      log(ok ? `${name} = ${val} saved` : `${name} is ${val} (asked for ${w.value})`, ok ? 'gcs' : 's4');
    }
    updateParamRow(name);
  }

  async function writeParam(name, value) {
    const p = s.params.get(name);
    if (!p) { log(`Read ${name} first.`, 's4'); return; }
    if (isArmed() && !(await ask(`Change ${name} while armed?`, '<p>Changing parameters while armed can upset the aircraft. Better on the ground, disarmed.</p>', 'Change it', true))) return;
    const value2 = p.t === 9 ? value : Math.round(value);
    const attempt = (n) => {
      send('PARAM_SET', { param_value: MAV.paramToWire(value2, p.t, s.ap === 'px4'), target_system: s.target.sys, target_component: s.target.comp, param_id: name, param_type: p.t });
      const w = { value: value2, timer: setTimeout(() => { if (s.paramWrites.get(name) !== w) return; if (n < 2) attempt(n + 1); else { s.paramWrites.delete(name); log(`${name}: no reply`, 's3'); } }, 1500) };
      s.paramWrites.set(name, w);
    };
    attempt(0);
  }

  function renderParamTable() {
    const box = $('paramTable');
    if (!s.ap) { box.innerHTML = '<p class="note">The list depends on the autopilot: connect first.</p>'; return; }
    let html = '<table class="ptable">';
    for (const p of ad().params) {
      if (p.length === 1) { html += `<tr><th colspan="4">${esc(p[0])}</th></tr>`; continue; }
      const [name, desc, sug] = p;
      html += `<tr data-p="${name}"><td>${name}<small>${esc(desc)}</small></td><td class="cur"></td><td>${sug === null ? '' : `<span class="sug" title="Suggested for this aircraft. Click to use.">${sug}</span>`}</td><td><button class="btn" disabled>Set</button></td></tr>`;
    }
    box.innerHTML = html + '</table>';
    box.querySelectorAll('tr[data-p]').forEach((tr) => {
      const name = tr.dataset.p;
      tr.querySelector('.btn').addEventListener('click', () => {
        const inp = tr.querySelector('input');
        const v = Number(inp && inp.value);
        if (inp && Number.isFinite(v)) writeParam(name, v);
      });
      const sug = tr.querySelector('.sug');
      if (sug) sug.addEventListener('click', () => { const inp = tr.querySelector('input'); if (inp) { inp.value = sug.textContent; inp.dispatchEvent(new Event('input')); } });
      updateParamRow(name);
    });
  }

  function updateParamRow(name) {
    const tr = $('paramTable').querySelector(`tr[data-p="${name}"]`);
    if (!tr) return;
    const cur = tr.querySelector('.cur'), btn = tr.querySelector('.btn'), sug = tr.querySelector('.sug');
    const p = s.params.get(name);
    if (!p) {
      cur.innerHTML = s.paramMissing.has(name) ? '<span class="missing">not on this firmware</span>' : '—';
      btn.disabled = true;
      return;
    }
    let inp = cur.querySelector('input');
    if (!inp) {
      cur.innerHTML = '<input type="number" step="any">';
      inp = cur.querySelector('input');
      inp.addEventListener('input', () => {
        const q = s.params.get(name);
        const changed = q && Number(inp.value) !== q.v;
        inp.classList.toggle('changed', changed);
        btn.disabled = !changed || !linked();
      });
    }
    if (document.activeElement !== inp) { inp.value = p.v; inp.classList.remove('changed'); btn.disabled = true; }
    if (sug) sug.classList.toggle('match', Number(sug.textContent) === p.v);
  }

  // ---------- map ----------
  let map = null, acMarker = null, homeMarker = null, tgtMarker = null, satLayer = null, streetLayer = null;
  let follow = true, tgt = null, trail = [], trailPts = 0;
  let tileCache = null;
  const PHASE_COLOR = () => {
    const css = getComputedStyle(document.documentElement);
    const v = (n) => css.getPropertyValue(n).trim();
    return { hover: v('--hover'), takeoff: v('--hover'), landing: v('--hover'), 'to-plane': v('--trans'), 'to-hover': v('--trans'), plane: v('--plane') };
  };

  async function openTileCache() {
    try { tileCache = await caches.open('tri-tilt-tiles-v1'); } catch (e) { tileCache = null; }
  }

  // Tiles come from the saved copy first, so the map works at a field with no signal.
  function cachedLayer(url, opts) {
    const Layer = L.TileLayer.extend({
      createTile(coords, done) {
        const img = document.createElement('img');
        img.alt = '';
        img.setAttribute('role', 'presentation');
        const src = this.getTileUrl(coords);
        const show = (u, revoke) => {
          img.onload = () => { if (revoke) URL.revokeObjectURL(u); done(null, img); };
          img.onerror = (e) => { if (revoke) URL.revokeObjectURL(u); done(e, img); };
          img.src = u;
        };
        (async () => {
          if (!tileCache) return show(src);
          try {
            const hit = await tileCache.match(src);
            if (hit) return show(URL.createObjectURL(await hit.blob()), true);
            const res = await fetch(src, { mode: 'cors' });
            if (!res.ok) return show(src);
            tileCache.put(src, res.clone()).catch(() => {});
            return show(URL.createObjectURL(await res.blob()), true);
          } catch (e) { return show(src); }
        })();
        return img;
      },
    });
    return new Layer(url, opts);
  }

  function initMap() {
    if (!window.L) return;
    const v = settings.view;
    map = L.map('map', { zoomControl: true, attributionControl: true }).setView(v ? [v.lat, v.lon] : [22.5, 79], v ? v.z : 5);
    satLayer = cachedLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', { maxZoom: 19, attribution: 'Imagery &copy; Esri' });
    streetLayer = cachedLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '&copy; OpenStreetMap contributors' });
    (settings.sat ? satLayer : streetLayer).addTo(map);
    let errors = 0;
    [satLayer, streetLayer].forEach((l) => {
      l.on('tileerror', () => { if (++errors > 6) $('mapOffline').hidden = false; });
      l.on('tileload', () => { errors = 0; $('mapOffline').hidden = true; });
    });
    acMarker = L.marker([0, 0], {
      icon: L.divIcon({
        className: '', iconSize: [40, 40], iconAnchor: [20, 20],
        html: '<svg class="ac-icon" width="40" height="40" viewBox="-20 -20 40 40"><path d="M0,-17 L3,-6 L17,2 L17,5 L3,2 L2,11 L7,15 L7,17 L0,15 L-7,17 L-7,15 L-2,11 L-3,2 L-17,5 L-17,2 L-3,-6 Z" fill="#c2410c" stroke="#fff" stroke-width="1.6" stroke-linejoin="round"/></svg>',
      }),
      interactive: false, zIndexOffset: 1000,
    });
    map.on('dragstart', () => { follow = false; updateFollowBtn(); });
    map.on('moveend', () => {
      const c = map.getCenter();
      settings.view = { lat: +c.lat.toFixed(6), lon: +c.lng.toFixed(6), z: map.getZoom() };
      saveSettings();
    });
    map.on('click', (e) => {
      tgt = e.latlng;
      if (!tgtMarker) tgtMarker = L.circleMarker(tgt, { radius: 9, color: '#121418', weight: 3, fillColor: '#fff', fillOpacity: 1 }).addTo(map);
      tgtMarker.setLatLng(tgt);
      dirty = true;
    });
    updateLayerBtn();
  }

  function newTrailSegment() {
    if (!map) return;
    const col = PHASE_COLOR()[s.phase];
    if (!col) return;
    const last = trail[trail.length - 1];
    const start = last ? last.getLatLngs().slice(-1) : [];
    trail.push(L.polyline(start, { color: col, weight: 3, opacity: 0.95 }).addTo(map));
  }

  function addTrail(m) {
    if (!map || (!m.lat && !m.lon) || !AIR.has(s.phase)) return;
    if (!trail.length) newTrailSegment();
    const seg = trail[trail.length - 1];
    if (!seg) return;
    const p = L.latLng(m.lat / 1e7, m.lon / 1e7);
    const pts = seg.getLatLngs();
    if (pts.length && pts[pts.length - 1].distanceTo(p) < 1.5) return;
    seg.addLatLng(p);
    if (++trailPts > 8000 && trail.length > 1) { trailPts -= trail[0].getLatLngs().length; trail.shift().remove(); }
  }

  function clearTrail() { trail.forEach((t) => t.remove()); trail = []; trailPts = 0; }

  function updateHomeMarker() {
    if (!map || !s.home) return;
    if (!homeMarker) {
      homeMarker = L.marker([s.home.lat, s.home.lon], {
        icon: L.divIcon({ className: '', iconSize: [24, 24], iconAnchor: [12, 12], html: '<div style="width:24px;height:24px;border-radius:50%;background:#15803d;color:#fff;font:700 12px/20px ui-monospace,Menlo,monospace;text-align:center;border:2px solid #fff;box-shadow:0 1px 3px rgba(0,0,0,.5)">H</div>' }),
        interactive: false,
      }).addTo(map);
    }
    homeMarker.setLatLng([s.home.lat, s.home.lon]);
  }

  function onTraffic(m) {
    if (!map || !(m.flags & 1)) return;
    const id = m.ICAO_address;
    const ll = [m.lat / 1e7, m.lon / 1e7];
    const label = `${(m.callsign || id.toString(16)).trim()} ${(m.altitude / 1000).toFixed(0)}m`;
    let t = s.traffic.get(id);
    if (!t) {
      t = { marker: L.marker(ll, { icon: L.divIcon({ className: '', iconSize: [90, 22], iconAnchor: [9, 11], html: '' }), interactive: false }).addTo(map) };
      s.traffic.set(id, t);
      if (s.pos) {
        const d = distance(s.pos.lat / 1e7, s.pos.lon / 1e7, ll[0], ll[1]);
        if (d < 5000) log(`ADS-B traffic: ${label}, ${(d / 1000).toFixed(1)} km away`, 's4');
      }
    }
    t.at = Date.now();
    t.marker.setLatLng(ll);
    t.marker.setIcon(L.divIcon({ className: '', iconSize: [90, 22], iconAnchor: [9, 11], html: `<svg width="18" height="18" viewBox="-9 -9 18 18" style="transform:rotate(${m.heading / 100}deg)"><path d="M0,-8 L6,7 L0,3 L-6,7 Z" fill="#7c3aed" stroke="#fff"/></svg><span class="traffic">${esc(label)}</span>` }));
  }

  function clearTraffic() { if (s && s.traffic) s.traffic.forEach((t) => t.marker.remove()); }

  function updateFollowBtn() { $('btnFollow').classList.toggle('on', follow); $('btnFollow').textContent = follow ? 'Following' : 'Follow'; }
  function updateLayerBtn() { $('btnLayer').textContent = settings.sat ? 'Street map' : 'Satellite'; }

  function distance(lat1, lon1, lat2, lon2) {
    const r = Math.PI / 180;
    const a = Math.sin((lat2 - lat1) * r / 2) ** 2 + Math.cos(lat1 * r) * Math.cos(lat2 * r) * Math.sin((lon2 - lon1) * r / 2) ** 2;
    return 2 * 6371000 * Math.asin(Math.sqrt(a));
  }
  function bearing(lat1, lon1, lat2, lon2) {
    const r = Math.PI / 180;
    const y = Math.sin((lon2 - lon1) * r) * Math.cos(lat2 * r);
    const x = Math.cos(lat1 * r) * Math.sin(lat2 * r) - Math.sin(lat1 * r) * Math.cos(lat2 * r) * Math.cos((lon2 - lon1) * r);
    return (deg(Math.atan2(y, x)) + 360) % 360;
  }
  const compass = (d) => ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'][Math.round(d / 45) % 8];

  // ---------- instruments ----------
  function drawAdi() {
    const c = $('adi'), ctx = c.getContext('2d');
    const w = c.width, h = c.height, cx = w / 2, cy = h / 2;
    const roll = s.att ? s.att.roll : 0, pitch = s.att ? deg(s.att.pitch) : 0;
    const ppd = h / 50;
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-roll);
    ctx.translate(0, pitch * ppd);
    ctx.fillStyle = s.att ? '#3b82c4' : '#8d939c'; ctx.fillRect(-w * 2, -h * 4, w * 4, h * 4);
    ctx.fillStyle = s.att ? '#7a5a36' : '#6b7079'; ctx.fillRect(-w * 2, 0, w * 4, h * 4);
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(-w * 2, 0); ctx.lineTo(w * 2, 0); ctx.stroke();
    ctx.lineWidth = 1.5; ctx.fillStyle = '#fff'; ctx.font = '600 13px ui-monospace,Menlo,monospace'; ctx.textBaseline = 'middle';
    for (let d = -40; d <= 40; d += 5) {
      if (!d) continue;
      const y = -d * ppd, len = d % 10 ? w * 0.05 : w * 0.12;
      ctx.beginPath(); ctx.moveTo(-len, y); ctx.lineTo(len, y); ctx.stroke();
      if (!(d % 10)) { ctx.textAlign = 'left'; ctx.fillText(String(Math.abs(d)), len + 5, y); ctx.textAlign = 'right'; ctx.fillText(String(Math.abs(d)), -len - 5, y); }
    }
    ctx.restore();
    ctx.save(); ctx.translate(cx, cy); ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
    const R = h * 0.42;
    for (const a of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
      const t = (a - 90) * Math.PI / 180, l = a % 30 ? 7 : 13;
      ctx.beginPath(); ctx.moveTo(Math.cos(t) * R, Math.sin(t) * R); ctx.lineTo(Math.cos(t) * (R - l), Math.sin(t) * (R - l)); ctx.stroke();
    }
    ctx.rotate(-roll); ctx.fillStyle = '#fbbf24';
    ctx.beginPath(); ctx.moveTo(0, -R + 2); ctx.lineTo(-7, -R + 14); ctx.lineTo(7, -R + 14); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.save(); ctx.translate(cx, cy); ctx.strokeStyle = '#fbbf24'; ctx.lineWidth = 5; ctx.lineCap = 'round';
    const k = w * 0.2;
    ctx.beginPath(); ctx.moveTo(-k, 0); ctx.lineTo(-k * 0.35, 0); ctx.lineTo(-k * 0.18, 13); ctx.moveTo(k, 0); ctx.lineTo(k * 0.35, 0); ctx.lineTo(k * 0.18, 13); ctx.stroke();
    ctx.fillStyle = '#fbbf24'; ctx.beginPath(); ctx.arc(0, 0, 5, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
  }

  function drawTape() {
    const c = $('tape'), ctx = c.getContext('2d');
    const w = c.width, h = c.height;
    const sp = speeds();
    const vmax = Math.max(25, sp.cruise * 1.6);
    const top = 34, bot = h - 34;
    const y = (v) => bot - (clamp(v, 0, vmax) / vmax) * (bot - top);
    ctx.clearRect(0, 0, w, h);
    const css = getComputedStyle(document.documentElement);
    const col = (n) => css.getPropertyValue(n).trim();
    const bx = 52, bw = 18;
    ctx.fillStyle = col('--bad'); ctx.fillRect(bx, y(sp.stall), bw, bot - y(sp.stall));
    ctx.fillStyle = col('--warn'); ctx.fillRect(bx, y(sp.min), bw, y(sp.stall) - y(sp.min));
    ctx.fillStyle = col('--ok'); ctx.fillRect(bx, y(sp.cruise * 1.4), bw, y(sp.min) - y(sp.cruise * 1.4));
    ctx.fillStyle = col('--muted'); ctx.font = '600 15px ui-monospace,Menlo,monospace'; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (let v = 0; v <= vmax; v += 5) { ctx.fillText(String(v), bx - 6, y(v)); ctx.fillRect(bx - 3, y(v) - 0.5, 3, 1); }
    ctx.strokeStyle = col('--ink'); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(bx + bw / 2, y(sp.cruise) - 6); ctx.lineTo(bx + bw / 2, y(sp.cruise) + 6); ctx.stroke();
    const as = s.hud ? s.hud.airspeed : null, gs = s.hud ? s.hud.groundspeed : null;
    if (gs !== null) {
      ctx.strokeStyle = col('--muted'); ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(bx + bw + 2, y(gs)); ctx.lineTo(bx + bw + 16, y(gs) - 7); ctx.lineTo(bx + bw + 16, y(gs) + 7); ctx.closePath(); ctx.stroke();
    }
    if (as !== null) {
      ctx.fillStyle = col('--ink');
      ctx.beginPath(); ctx.moveTo(bx + bw + 2, y(as)); ctx.lineTo(bx + bw + 22, y(as) - 10); ctx.lineTo(bx + bw + 22, y(as) + 10); ctx.closePath(); ctx.fill();
    }
    ctx.textAlign = 'center'; ctx.fillStyle = col('--ink'); ctx.font = '700 22px ui-monospace,Menlo,monospace';
    ctx.fillText(as === null ? '—' : as.toFixed(1), w / 2, 15);
    ctx.font = '600 13px ui-monospace,Menlo,monospace'; ctx.fillStyle = col('--muted');
    ctx.fillText(gs === null ? 'GS —' : `GS ${gs.toFixed(1)}`, w / 2, h - 13);
  }

  // Top view of the airframe: prop discs foreshorten as the front motors tilt forward.
  function buildTri() {
    const P = 127;
    $('tri').innerHTML = `
      <rect x="-600" y="0" width="1200" height="250" rx="8" fill="var(--panel)" stroke="var(--muted)" stroke-width="5"/>
      <rect x="-14" y="250" width="28" height="420" fill="var(--muted)"/>
      <rect x="-165" y="600" width="330" height="80" rx="6" fill="var(--panel)" stroke="var(--muted)" stroke-width="5"/>
      <rect x="-48" y="-230" width="96" height="480" rx="44" fill="var(--card)" stroke="var(--muted)" stroke-width="5"/>
      <g stroke="var(--muted)" stroke-width="9"><line x1="-230" y1="-118" x2="-230" y2="60"/><line x1="230" y1="-118" x2="230" y2="60"/></g>
      <g transform="translate(0 62)"><circle r="20" fill="var(--card)" stroke="var(--ink)" stroke-width="4"/><path d="M0,-20 A20,20 0 0 1 20,0 L0,0 Z M0,20 A20,20 0 0 1 -20,0 L0,0 Z" fill="var(--ink)"/></g>
      ${['L', 'R', 'B'].map((k) => `<g id="tri${k}"><ellipse class="disc" rx="${P}" ry="${P}" fill="var(--accent)" fill-opacity=".15" stroke="var(--accent)" stroke-width="7"/><circle r="16" fill="var(--ink)"/><text class="pct" y="${k === 'B' ? 72 : -32}" text-anchor="middle" font-size="64" font-weight="700" fill="var(--ink)">—</text></g>`).join('')}
      <text id="triTL" x="-400" y="-200" text-anchor="middle" font-size="54" font-weight="700" fill="var(--trans)"></text>
      <text id="triTR" x="400" y="-200" text-anchor="middle" font-size="54" font-weight="700" fill="var(--trans)"></text>
      <text x="-400" y="-260" text-anchor="middle" font-size="36" fill="var(--muted)">LEFT tilt</text>
      <text x="400" y="-260" text-anchor="middle" font-size="36" fill="var(--muted)">RIGHT tilt</text>
      <text id="triYaw" x="0" y="-270" text-anchor="middle" font-size="36" fill="var(--muted)"></text>
      <text x="160" y="435" font-size="36" fill="var(--muted)">rear (fixed)</text>`;
    $('triL').setAttribute('transform', 'translate(-230 -118)');
    $('triR').setAttribute('transform', 'translate(230 -118)');
    $('triB').setAttribute('transform', 'translate(0 422)');
  }

  function updateTri() {
    const tl = pwmToTilt(servoPwm(settings.tl)), tr = pwmToTilt(servoPwm(settings.tr));
    const set = (k, pct, tilt) => {
      const g = $('tri' + k);
      const disc = g.querySelector('.disc');
      disc.setAttribute('ry', (127 * Math.max(0.12, Math.cos(clamp(tilt || 0, 0, 90) * Math.PI / 180))).toFixed(1));
      disc.setAttribute('fill-opacity', pct === null ? 0.08 : (0.1 + 0.8 * pct / 100).toFixed(2));
      g.querySelector('.pct').textContent = pct === null ? '—' : pct < 1 && isArmed() ? 'OFF' : pct.toFixed(0) + '%';
    };
    set('L', motorPct(settings.m2), tl);
    set('R', motorPct(settings.m1), tr);
    set('B', motorPct(settings.m3), 0);
    $('triTL').textContent = tl === null ? '—' : tl.toFixed(0) + '°';
    $('triTR').textContent = tr === null ? '—' : tr.toFixed(0) + '°';
    $('triYaw').textContent = tl !== null && tr !== null ? `yaw tilt ${(tl - tr) >= 0 ? '+' : ''}${(tl - tr).toFixed(1)}°` : '';
  }

  // In a steady hover the motor split tells you where the CG really is.
  function updateBalance() {
    const box = $('balance');
    const steady = s.phase === 'hover' && s.att && Math.abs(deg(s.att.roll)) < 8 && Math.abs(deg(s.att.pitch)) < 8 && s.hud && Math.abs(s.hud.climb) < 0.6 && s.hud.groundspeed < 3;
    const fr = motorPct(settings.m1), fl = motorPct(settings.m2), rr = motorPct(settings.m3);
    if (steady && fr && fl && rr) {
      s.balance.push({ fr, fl, rr, t: Date.now() });
      while (s.balance.length && Date.now() - s.balance[0].t > 5000) s.balance.shift();
    }
    if (s.balance.length < 12) {
      box.className = 'balance';
      setText('balTx', s.phase === 'hover' ? 'Collecting… hold a steady hover for a few seconds.' : 'Shown once hovering steadily.');
      return;
    }
    const avg = (k) => s.balance.reduce((a, b) => a + b[k], 0) / s.balance.length;
    const FR = avg('fr'), FL = avg('fl'), RR = avg('rr');
    // Thrust grows faster than throttle; ~1.5 power is a fair guess for small props.
    const tF = ((FR + FL) / 2) ** 1.5, tR = RR ** 1.5;
    const r = tR / tF;
    const a = settings.armA, b = settings.armB;
    const d = (r * b - 2 * a) / (2 + r); // + = CG aft of the design point
    const lr = (FR - FL) / ((FR + FL) / 2) * 100;
    const msgs = [];
    let cls = 'ok';
    if (Math.abs(d) < 6) msgs.push(`Front ${((FR + FL) / 2).toFixed(0)}% · rear ${RR.toFixed(0)}%: balanced.`);
    else {
      cls = 'warn';
      const move = Math.abs(d) * settings.auwG / settings.battG;
      msgs.push(`Rear motor working ${d > 0 ? 'harder' : 'less'} (front ${((FR + FL) / 2).toFixed(0)}% · rear ${RR.toFixed(0)}%): CG roughly ${Math.abs(d).toFixed(0)} mm ${d > 0 ? 'aft' : 'forward'}. Move the battery about ${Math.round(move / 5) * 5} mm ${d > 0 ? 'forward' : 'back'}.`);
    }
    if (Math.abs(lr) > 8) { cls = 'warn'; msgs.push(`${lr > 0 ? 'Right' : 'Left'} front motor working ${Math.abs(lr).toFixed(0)}% harder: CG to that side, or a weaker motor/prop there.`); }
    box.className = 'balance ' + cls;
    setText('balTx', esc(msgs.join(' ')));
  }

  function bars(el, rows) {
    if (el.children.length !== rows.length) el.innerHTML = rows.map(() => '<div class="bar"><span></span><div class="track"><b></b></div><span class="num"></span></div>').join('');
    rows.forEach((r, i) => {
      const row = el.children[i];
      row.children[0].textContent = r[0];
      row.children[1].firstChild.style.width = clamp(r[1], 0, 100) + '%';
      row.children[2].textContent = r[2];
    });
  }

  function setText(id, html) { const el = $(id); if (el && el.innerHTML !== html) el.innerHTML = html; }
  const fmt = (v, d, unit) => (v === undefined || v === null || Number.isNaN(v) ? '—' : `${v.toFixed(d)}<i>${unit}</i>`);
  function setStat(id, cls) { $(id).className = 'stat ' + (cls || ''); }

  // ---------- render ----------
  function render() {
    const now = Date.now();
    const hbAge = s.lastHb ? (now - s.lastHb) / 1000 : null;
    const live = linked();
    updatePhase();

    // Link lost bookkeeping and banner.
    if (link && s.lastHb && hbAge > 3.5 && !s.wasLost) { s.wasLost = true; log('Telemetry lost.', 's3'); say('Telemetry lost'); }
    const banner = $('banner');
    let bt = '', bcls = '';
    if (link && s.wasLost) { bt = `TELEMETRY LOST ${hbAge.toFixed(0)} s — the aircraft carries on with what it was doing. Fly it with the RC transmitter.`; bcls = 'bad'; }
    else if (link && link.kind === 'demo') bt = 'DEMO — simulated aircraft. Nothing here is real telemetry.';
    else if (!('serial' in navigator) && $('linkKind').value === 'serial') bt = 'This browser can\'t open USB ports. Use Chrome or Edge on a laptop (desktop, not phone).';
    banner.hidden = !bt; banner.textContent = bt; banner.className = 'banner ' + bcls;

    // Header stats.
    const st = linkStats();
    if (!link) { setText('sLink', '—'); setText('sLinkSub', 'not connected'); setStat('stLink'); }
    else if (!s.lastHb) { setText('sLink', 'WAIT'); setText('sLinkSub', 'no heartbeat yet'); setStat('stLink', 'warn'); }
    else if (!live) { setText('sLink', 'LOST'); setText('sLinkSub', `${hbAge.toFixed(0)} s ago`); setStat('stLink', 'bad'); }
    else {
      const rtt = s.rtt !== null && now - s.rttAt < 5000 ? s.rtt : null;
      setText('sLink', rtt === null ? 'LIVE' : `${rtt.toFixed(0)} ms`);
      setText('sLinkSub', `${st.loss === null ? '—' : st.loss.toFixed(1) + '%'} loss · ${st.rate === null ? '—' : (st.rate / 1024).toFixed(1) + ' kB/s'}`);
      setStat('stLink', (st.loss || 0) > 10 || (rtt || 0) > 600 ? 'warn' : 'ok');
    }
    const c = cellV();
    if (c !== null) {
      setText('sBatt', `${(s.sys.voltage_battery / 1000).toFixed(1)} V`);
      setText('sBattSub', `${c.toFixed(2)}/cell${s.sys.battery_remaining >= 0 ? ' · ' + s.sys.battery_remaining + '%' : ''}`);
      setStat('stBatt', c < 3.4 ? 'bad' : c < 3.55 ? 'warn' : 'ok');
      if (isArmed() && AIR.has(s.phase)) {
        if (c < 3.4 && s.battWarn < 2) { s.battWarn = 2; say('Battery critical. Land now'); log('Battery critical: land now.', 's2'); }
        else if (c < 3.55 && s.battWarn < 1) { s.battWarn = 1; say('Battery low'); log('Battery low: plan to land.', 's4'); }
      }
    } else { setText('sBatt', '—'); setText('sBattSub', '&nbsp;'); setStat('stBatt'); }
    if (s.gps) {
      const FIX = ['NONE', 'NO FIX', '2D', '3D', 'DGPS', 'RTK~', 'RTK'];
      setText('sGps', `${FIX[s.gps.fix_type] || s.gps.fix_type} · ${s.gps.satellites_visible}`);
      setText('sGpsSub', `HDOP ${(s.gps.eph / 100).toFixed(1)}`);
      setStat('stGps', s.gps.fix_type >= 3 ? (s.gps.satellites_visible >= 8 ? 'ok' : 'warn') : 'bad');
    } else { setText('sGps', '—'); setText('sGpsSub', '&nbsp;'); setStat('stGps'); }
    const armed = isArmed();
    if (armed && !s.lastArmed) s.armedSince = now;
    if (!armed && s.lastArmed) s.flightMs += now - s.armedSince;
    s.lastArmed = armed;
    const ft = Math.floor((s.flightMs + (armed ? now - s.armedSince : 0)) / 1000);
    setText('sTime', s.hb ? `${Math.floor(ft / 60)}:${String(ft % 60).padStart(2, '0')}` : '—');
    setText('sTimeSub', armed ? 'armed' : s.hb ? 'disarmed' : '&nbsp;');
    setText('apText', s.ap ? `${ad().name}${s.version ? ' ' + s.version : ''} · sys ${s.target.sys}` : link ? 'Waiting for heartbeat' : 'No vehicle');

    renderPhase(hbAge);
    renderControls(live);
    renderReady();

    // Instruments.
    drawAdi();
    setText('attTx', s.att ? `R ${deg(s.att.roll).toFixed(0)}° P ${deg(s.att.pitch).toFixed(0)}°` : '—');
    drawTape();
    const hud = s.hud, pos = s.pos;
    setText('rAlt', pos ? fmt(pos.relative_alt / 1000, 1, 'm') : '—');
    setText('rClimb', hud ? fmt(hud.climb, 1, 'm/s') : '—');
    setText('asLabel', s.sys && !(s.sys.sensors_present & 16) ? 'Airspeed (est.)' : 'Airspeed');
    setText('rAs', hud ? fmt(hud.airspeed, 1, 'm/s') : '—');
    setText('rGs', hud ? fmt(hud.groundspeed, 1, 'm/s') : '—');
    setText('rHdg', hud ? fmt(hud.heading, 0, '°') : '—');
    setText('rThr', hud ? fmt(hud.throttle, 0, '%') : '—');
    if (s.wind) {
      const w = s.wind;
      setText('rWind', `${w.speed.toFixed(1)}<i>m/s ${compass(w.from)}</i>`);
      $('windRose').hidden = false;
      $('windArrow').setAttribute('transform', `rotate(${(w.from + 180).toFixed(0)})`);
      setText('windTx', `${w.speed.toFixed(1)} m/s<br><small>from ${compass(w.from)}</small>`);
      if (hud) {
        const head = w.speed * Math.cos((w.from - hud.heading) * Math.PI / 180);
        const col = head < -2 ? 'var(--bad)' : head < 0 ? 'var(--warn)' : 'var(--ok)';
        setText('rHead', `<span style="color:${col}">${Math.abs(head).toFixed(1)}<i>m/s ${head >= 0 ? 'head' : 'tail'}</i></span>`);
      } else setText('rHead', '—');
    } else { setText('rWind', '—'); setText('rHead', '—'); $('windRose').hidden = true; }
    if (pos && s.home) {
      const la = pos.lat / 1e7, lo = pos.lon / 1e7;
      const d = distance(s.home.lat, s.home.lon, la, lo);
      setText('rHome', `${d >= 1000 ? (d / 1000).toFixed(2) : d.toFixed(0)}<i>${d >= 1000 ? 'km' : 'm'} ${d > 5 ? compass(bearing(la, lo, s.home.lat, s.home.lon)) : ''}</i>`);
    } else setText('rHome', '—');

    // Aircraft tab.
    updateTri();
    updateBalance();
    const r = s.radio && now - s.radioAt < 5000 ? s.radio : null;
    setText('kRssi', r ? `${r.rssi} / ${r.remrssi}` : link && link.kind === 'serial' ? '<i>USB cable or no SiK</i>' : '—');
    setText('kFade', r ? `${Math.min(r.rssi - r.noise, r.remrssi - r.remnoise)}<i>dB-ish</i>` : '—');
    setText('kRtt', s.rtt !== null && now - s.rttAt < 5000 ? `${s.rtt.toFixed(0)}<i>ms (~${(s.rtt / 2).toFixed(0)} one way)</i>` : '—');
    setText('kLoss', st.loss === null ? '—' : fmt(st.loss, 1, '%'));
    setText('kRate', st.rate === null ? '—' : `${(st.rate / 1024).toFixed(2)}<i>kB/s</i>`);
    setText('kCurr', s.sys && s.sys.current_battery >= 0 ? fmt(s.sys.current_battery / 100, 1, 'A') : '—');
    if ($('outBars').offsetParent) {
      const motor = (label, ch) => { const p = motorPct(ch); return [label, p || 0, p === null ? '—' : p.toFixed(0) + '%']; };
      const servo = (label, ch) => { const p = servoPwm(ch); return [label, p ? (p - 900) / 12 : 0, p || '—']; };
      bars($('outBars'), [motor('Motor FR', settings.m1), motor('Motor FL', settings.m2), motor('Motor rear', settings.m3), servo('Tilt L', settings.tl), servo('Tilt R', settings.tr)]);
      const rc = [];
      for (let i = 1; i <= 8; i++) { const p = s.rc ? s.rc['chan' + i + '_raw'] : 0; rc.push(['RC ' + i, p ? (p - 900) / 12 : 0, p || '—']); }
      bars($('rcBars'), rc);
    }

    // Map.
    if (map && pos && (pos.lat || pos.lon)) {
      const ll = [pos.lat / 1e7, pos.lon / 1e7];
      if (!map.hasLayer(acMarker)) { acMarker.addTo(map); map.setView(ll, 17); }
      acMarker.setLatLng(ll);
      const svg = acMarker.getElement() && acMarker.getElement().querySelector('svg');
      const yaw = s.att ? deg(s.att.yaw) : pos.hdg !== 65535 ? pos.hdg / 100 : 0;
      if (svg) svg.style.transform = `rotate(${yaw}deg)`;
      if (follow) map.panTo(ll, { animate: false });
    }
    for (const [id, t] of s.traffic) if (now - t.at > 30000) { t.marker.remove(); s.traffic.delete(id); }
    $('targetBar').hidden = !tgt;
    if (tgt) setText('tgtTx', `Target${pos ? ' · ' + distance(pos.lat / 1e7, pos.lon / 1e7, tgt.lat, tgt.lng).toFixed(0) + ' m' : ''}`);
    $('btnFlyHere').disabled = !(live && AIR.has(s.phase));
    $('btnFlyHere').title = AIR.has(s.phase) ? '' : 'Take off first';

    const ps = parser ? parser.stats : null;
    setText('pktTx', ps ? `${ps.frames} pkts · ${ps.crcErrors} bad` : '—');
  }

  function renderPhase(hbAge) {
    const card = $('phaseCard');
    const p = s.phase;
    card.dataset.phase = p;
    const step = PHASE_STEP[p];
    card.querySelectorAll('.steps li').forEach((li) => li.classList.toggle('on', li.dataset.step === step && p !== 'nolink'));
    let big = phaseLabel(p), sub = '', prog = null;
    const a = relAlt();
    switch (p) {
      case 'nolink': sub = 'Connect the radio (or USB cable) to start.'; break;
      case 'waiting': sub = 'Port open, no heartbeat yet. Check the baud rate, that the radios are paired (solid green LED), and that nothing else has the port.'; break;
      case 'disarmed': sub = 'Disarmed. Hold “Take off” when the checks below are green.'; break;
      case 'armed': {
        sub = 'PROPS ARE LIVE.';
        const ad2 = ad().autoDisarm();
        if (ad2) { const left = Math.max(0, ad2 - (Date.now() - s.armedSince) / 1000); sub += ` ${ad().name} disarms itself in ${left.toFixed(0)} s without a takeoff.`; prog = left / ad2; }
        break;
      }
      case 'takeoff': sub = `Climbing · ${a === null ? '—' : a.toFixed(1)} m`; break;
      case 'hover': sub = `Multicopter, motors up · ${a === null ? '—' : a.toFixed(0)} m`; break;
      case 'landing': sub = `Descending · ${a === null ? '—' : a.toFixed(1)} m`; break;
      case 'plane': sub = `Wing flying, rear motor off · ${s.hud ? s.hud.airspeed.toFixed(0) : '—'} m/s · ${a === null ? '—' : a.toFixed(0)} m`; break;
      case 'to-plane': case 'to-hover': {
        const el = s.trans ? (Date.now() - s.trans.t0) / 1000 : 0;
        const T = p === 'to-plane' ? ad().transTime() : ad().backTime();
        sub = p === 'to-plane' ? `Tilting forward, speeding up · ${el.toFixed(1)} s${T ? ' of ' + T.toFixed(1) : ''}` : `Slowing down, tilting up · ${el.toFixed(1)} s`;
        if (T) prog = Math.min(1, el / T);
        else if (p === 'to-plane' && s.hud) prog = Math.min(1, s.hud.airspeed / speeds().min);
        break;
      }
    }
    if (link && s.wasLost) sub = `Telemetry lost ${hbAge.toFixed(0)} s ago. Last known: ${big}.`;
    setText('phaseBig', esc(big));
    setText('phaseSub', esc(sub));
    $('phaseProg').hidden = prog === null;
    if (prog !== null) $('phaseProg').firstChild.style.width = (prog * 100).toFixed(1) + '%';
    const armed = isArmed();
    $('armBadge').textContent = s.hb ? (armed ? 'ARMED' : 'DISARMED') : '—';
    $('armBadge').className = 'badge' + (armed ? ' armed' : '');
    $('modeBadge').textContent = s.hb ? `${ad().name} · ${modeName()}` : '—';
  }

  function renderControls(live) {
    const p = s.phase;
    const show = {
      ground: ['nolink', 'waiting', 'disarmed', 'armed'].includes(p),
      'ground-disarmed': ['nolink', 'waiting', 'disarmed'].includes(p),
      'ground-armed': p === 'armed',
      hover: p === 'hover' || p === 'takeoff' || p === 'landing',
      plane: p === 'plane',
      trans: p === 'to-plane',
      air: AIR.has(p),
    };
    document.querySelectorAll('.acts [data-show]').forEach((el) => {
      el.hidden = !el.dataset.show.split(' ').some((k) => show[k]);
    });
    ['btnTakeoff', 'btnArm', 'btnToPlane', 'btnToHover', 'btnHold', 'btnRtl', 'btnLand', 'btnDisarm'].forEach((id) => { $(id).disabled = !live; });
    $('btnToPlane').disabled = !live || p !== 'hover';
    $('btnKill').disabled = !(live && isArmed());
    document.querySelectorAll('#modeGrid [data-mode]').forEach((b) => {
      b.disabled = !live;
      b.classList.toggle('on', !!s.hb && modeName() === b.dataset.mode);
    });
    setText('ctrlHint', !link ? 'Connect first' : !s.target ? 'Waiting for vehicle' : live ? (link.kind === 'demo' ? 'Simulator' : 'Live') : 'Link lost: commands paused');
  }

  function renderReady() {
    const p = s.phase;
    const inTrans = p === 'to-plane' || p === 'to-hover';
    $('readyList').hidden = inTrans;
    $('transLive').hidden = !inTrans;
    let title, list;
    if (inTrans) {
      title = p === 'to-plane' ? 'Transition to plane' : 'Transition to hover';
      renderTransLive();
      setText('readyTitle', esc(title));
      return;
    }
    if (p === 'hover') { title = 'Ready to transition?'; list = transChecks(); }
    else if (p === 'plane') { title = 'Plane flight'; list = planeChecks(); }
    else if (AIR.has(p)) { title = 'Flight'; list = common().concat([battCheck(false)]); }
    else { title = 'Ready to take off?'; list = s.hb ? groundChecks() : []; }
    const bad = list.filter((c) => c.st === 'bad').length, warn = list.filter((c) => c.st === 'warn').length;
    const sum = !list.length ? '' : bad ? `<span style="color:var(--bad)">${bad} failing</span>` : warn ? `<span style="color:var(--warn)">${warn} to check</span>` : '<span style="color:var(--ok)">all good</span>';
    $('readyTitle').innerHTML = `${esc(title)} <span>${sum}</span>`;
    const html = list.length ? checksHtml(list).replace(/^<ul class="checks">|<\/ul>$/g, '') : '<li><span class="note">Connect to see checks.</span></li>';
    if ($('readyList').innerHTML !== html) $('readyList').innerHTML = html;
  }

  function renderTransLive() {
    const t = s.trans;
    if (!t) return;
    const el = (Date.now() - t.t0) / 1000;
    const toPlane = t.kind === 'to-plane';
    const T = toPlane ? ad().transTime() : ad().backTime();
    const timeout = toPlane ? ad().transTimeout() : null;
    const span = T || timeout || 15;
    $('tlTime').style.width = clamp(el / span * 100, 0, 100) + '%';
    setText('tlTimeTx', `${el.toFixed(1)}${T ? '/' + T.toFixed(0) : ''} s`);
    const sp = speeds();
    const v = s.hud ? s.hud.airspeed : 0;
    const vmax = sp.cruise * 1.3;
    $('tlSpd').style.width = clamp(v / vmax * 100, 0, 100) + '%';
    $('tlSpdMin').style.left = (sp.min / vmax * 100) + '%';
    setText('tlSpdTx', `${v.toFixed(1)} m/s`);
    const tilt = frontTilt();
    $('tlTilt').style.width = tilt === null ? '0' : clamp(tilt / 90 * 100, 0, 100) + '%';
    setText('tlTiltTx', tilt === null ? '—' : `${tilt.toFixed(0)}°`);
    const dAlt = (relAlt() || 0) - t.alt0;
    const altBar = $('tlAlt');
    const wpct = clamp(Math.abs(dAlt) / 10 * 50, 0, 50);
    altBar.style.width = wpct + '%';
    altBar.style.left = dAlt >= 0 ? '50%' : (50 - wpct) + '%';
    altBar.style.background = dAlt < -3 ? 'var(--bad)' : dAlt < -1.5 ? 'var(--warn)' : 'var(--ok)';
    setText('tlAltTx', `${dAlt >= 0 ? '+' : ''}${dAlt.toFixed(1)} m`);
    let note = toPlane ? `Done when ${T ? `the ${T.toFixed(0)} s timer runs out` : `speed passes ${sp.min} m/s`}.${timeout ? ` Auto-abort after ${timeout} s.` : ''}` : 'Speed bleeds off, then the motors tilt up and it hovers.';
    if (toPlane && dAlt < -3) {
      note = `<b style="color:var(--bad)">Losing height (${dAlt.toFixed(1)} m). If it keeps sinking: Back to hover.</b>`;
      if (!t.warned) { t.warned = true; say('Losing height'); }
    }
    setText('tlNote', note);
  }

  // ---------- camera ----------
  const cam = { stream: null, pc: null };
  function camUi() {
    const k = $('camKind').value;
    $('camDevRow').hidden = k !== 'usb';
    $('camUrlRow').hidden = !(k === 'mjpeg' || k === 'whep');
    $('camUrl').placeholder = k === 'whep' ? 'http://192.168.4.1:8889/cam/whep' : 'http://192.168.4.1:8080/stream.mjpg';
  }
  async function listCams() {
    try {
      const devs = (await navigator.mediaDevices.enumerateDevices()).filter((d) => d.kind === 'videoinput');
      const sel = $('camDev');
      sel.innerHTML = devs.map((d, i) => `<option value="${esc(d.deviceId)}">${esc(d.label || 'Video device ' + (i + 1))}</option>`).join('') || '<option value="">No video devices found</option>';
      if (settings.camDev) sel.value = settings.camDev;
    } catch (e) { /* no media devices */ }
  }
  function stopCamera() {
    if (cam.stream) cam.stream.getTracks().forEach((t) => t.stop());
    if (cam.pc) cam.pc.close();
    cam.stream = cam.pc = null;
    $('camVideo').srcObject = null;
    $('camImg').removeAttribute('src');
    $('camBox').hidden = true;
    $('btnCamToggle').hidden = true;
  }
  async function startCamera() {
    stopCamera();
    const k = $('camKind').value, url = $('camUrl').value.trim();
    settings.camKind = k; settings.camUrl = url; settings.camDev = $('camDev').value;
    saveSettings();
    if (k === 'none') return;
    const video = $('camVideo'), img = $('camImg');
    try {
      if (k === 'usb') {
        cam.stream = await navigator.mediaDevices.getUserMedia({ video: settings.camDev ? { deviceId: { exact: settings.camDev } } : true, audio: false });
        video.srcObject = cam.stream;
        listCams();
      } else if (k === 'mjpeg') {
        img.src = url;
      } else if (k === 'whep') {
        const pc = new RTCPeerConnection();
        cam.pc = pc;
        pc.addTransceiver('video', { direction: 'recvonly' });
        pc.ontrack = (e) => { video.srcObject = e.streams[0] || new MediaStream([e.track]); };
        await pc.setLocalDescription(await pc.createOffer());
        await new Promise((r) => { if (pc.iceGatheringState === 'complete') r(); else { pc.onicegatheringstatechange = () => pc.iceGatheringState === 'complete' && r(); setTimeout(r, 1500); } });
        const res = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/sdp' }, body: pc.localDescription.sdp });
        if (!res.ok) throw new Error(`server said ${res.status}`);
        await pc.setRemoteDescription({ type: 'answer', sdp: await res.text() });
      }
      video.hidden = k === 'mjpeg';
      img.hidden = k !== 'mjpeg';
      $('camBox').hidden = false;
      $('camBox').classList.toggle('main', settings.camMain);
      $('btnCamToggle').hidden = false;
      setText('camTx', k === 'usb' ? 'USB video' : k === 'whep' ? 'WebRTC' : 'Stream');
      log('Video started.', 'gcs');
    } catch (e) {
      stopCamera();
      log(`Video failed: ${e.message}`, 's3');
    }
  }

  // ---------- wiring ----------
  function selectTab(name) {
    document.querySelectorAll('.tabs [data-tab]').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.tab === name)));
    document.querySelectorAll('.tab[data-panel]').forEach((p) => { p.hidden = p.dataset.panel !== name; });
    if (name === 'setup' && tileCache) tileCache.keys().then((k) => { $('tileTx').textContent = `${k.length} tiles saved.`; });
    if (name === 'cam') listCams();
    dirty = true;
  }

  function applyTheme() {
    document.documentElement.classList.toggle('night', settings.night);
    $('btnTheme').textContent = settings.night ? 'Day' : 'Night';
    $('btnVoice').classList.toggle('on', settings.voice);
    $('btnVoice').textContent = settings.voice ? 'Voice on' : 'Voice off';
    if (s) newTrailSegment();
  }

  function init() {
    $('linkKind').value = settings.kind;
    $('baud').value = String(settings.baud);
    $('wsUrl').value = settings.ws;
    const kindUi = () => {
      const k = $('linkKind').value;
      $('baud').hidden = k !== 'serial';
      $('wsUrl').hidden = k !== 'ws';
    };
    $('linkKind').addEventListener('change', kindUi);
    kindUi();
    $('btnConnect').addEventListener('click', connect);
    $('btnDisconnect').addEventListener('click', () => stopLink());
    $('btnLog').addEventListener('click', saveTlog);
    $('btnVoice').addEventListener('click', () => { settings.voice = !settings.voice; saveSettings(); applyTheme(); if (settings.voice) say('Voice on'); });
    $('btnTheme').addEventListener('click', () => { settings.night = !settings.night; saveSettings(); applyTheme(); });
    holdButton($('btnTakeoff'), doTakeoff);
    holdButton($('btnArm'), () => { log('Requesting ARM…', 'gcs'); cmd(400, [1], { label: 'Arm' }); });
    holdButton($('btnDisarm'), doDisarm);
    holdButton($('btnKill'), doKill);
    $('btnToPlane').addEventListener('click', doToPlane);
    $('btnToHover').addEventListener('click', doToHover);
    $('btnHold').addEventListener('click', () => ad().hold());
    $('btnRtl').addEventListener('click', async () => { if (await ask('Return home?', `<p>${s.ap === 'ardupilot' ? 'QRTL: flies home and lands vertically.' : 'Return: climbs to RTL_RETURN_ALT, flies home, transitions to hover and lands.'}</p>`, 'Return home')) ad().rtl(); });
    $('btnLand').addEventListener('click', async () => { if (await ask('Land here?', `<p>${[VS.FW, VS.TO_FW].includes(vtolState()) ? 'It goes back to hover first, then lands straight down where it is.' : 'It lands straight down where it is.'}</p>`, 'Land')) ad().land(); });
    $('btnFlyHere').addEventListener('click', doFlyHere);
    $('btnTgtCancel').addEventListener('click', () => { tgt = null; if (tgtMarker) { tgtMarker.remove(); tgtMarker = null; } dirty = true; });
    $('btnFollow').addEventListener('click', () => { follow = !follow; updateFollowBtn(); });
    $('btnLayer').addEventListener('click', () => {
      if (!map) return;
      settings.sat = !settings.sat; saveSettings();
      map.removeLayer(settings.sat ? streetLayer : satLayer);
      (settings.sat ? satLayer : streetLayer).addTo(map);
      updateLayerBtn();
    });
    $('btnClear').addEventListener('click', clearTrail);
    $('btnCamToggle').addEventListener('click', () => { $('camBox').hidden = !$('camBox').hidden; });
    $('btnCamSize').addEventListener('click', () => { settings.camMain = !settings.camMain; saveSettings(); $('camBox').classList.toggle('main', settings.camMain); });
    document.querySelectorAll('.tabs [data-tab]').forEach((b) => b.addEventListener('click', () => selectTab(b.dataset.tab)));
    $('btnParamRead').addEventListener('click', readParams);
    $('camKind').value = settings.camKind;
    $('camUrl').value = settings.camUrl;
    $('camKind').addEventListener('change', camUi);
    camUi();
    $('btnCamStart').addEventListener('click', startCamera);
    $('btnCamStop').addEventListener('click', () => { settings.camKind = 'none'; saveSettings(); stopCamera(); $('camKind').value = 'none'; camUi(); });
    $('btnTileClear').addEventListener('click', async () => { if (tileCache) { await caches.delete('tri-tilt-tiles-v1'); await openTileCache(); $('tileTx').textContent = '0 tiles saved.'; } });
    if ('serial' in navigator) navigator.serial.addEventListener('disconnect', () => { if (link && link.kind === 'serial') stopLink('USB device unplugged'); });
    window.addEventListener('beforeunload', (e) => { if (link && isArmed()) { e.preventDefault(); e.returnValue = ''; } });

    applyTheme();
    buildTri();
    buildModeButtons();
    renderParamTable();
    initMap();
    updateFollowBtn();
    openTileCache();
    setInterval(render, 80);
    render();
    log('Ready. Plug in the telemetry radio (or the Cube by USB) and press Connect, or pick a demo.', 'gcs');
    if (settings.camKind !== 'none' && settings.camKind !== 'usb') startCamera();
  }

  init();
})();
