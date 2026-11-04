// Demo vehicle: a very rough tri tilt-rotor that speaks MAVLink like PX4 or ArduPlane would.
// It exists so the dashboards can be tried without hardware. It is not a flight model.
(function (root) {
  'use strict';
  const MAV = root.MAV || (typeof require !== 'undefined' ? require('./mavlink.js') : null);
  const M = MAV.MODE_NUMBERS;
  const HOME = { lat: 47.397742, lon: 8.545594, amsl: 488 };
  const R_EARTH = 6378137;
  const MC = 3, FW = 4, TO_FW = 1, TO_MC = 2; // MAV_VTOL_STATE

  // ArduPlane: which modes fly as a multicopter, and what each mode does.
  const AP_VTOL = new Set([M.QSTABILIZE, M.QHOVER, M.QLOITER, M.QLAND, M.QRTL, M.QACRO]);
  const AP_ARMABLE = new Set([M.QSTABILIZE, M.QHOVER, M.QLOITER, M.QACRO, M.FBWA, M.MANUAL, M.STABILIZE, M.GUIDED]);
  const PX4_ARMABLE = new Set(['MANUAL', 'STABILIZED', 'ALTCTL', 'POSCTL', 'HOLD', 'TAKEOFF', 'ACRO']);

  // [name, value, MAV_PARAM_TYPE] — 9 = float, 6 = int32
  const PX4_PARAMS = [
    ['VT_TYPE', 1, 6], ['VT_F_TR_OL_TM', 6, 9], ['VT_F_TRANS_DUR', 5, 9], ['VT_B_TRANS_DUR', 4, 9],
    ['VT_TRANS_MIN_TM', 2, 9], ['VT_TRANS_TIMEOUT', 15, 9], ['VT_ARSP_TRANS', 10, 9],
    ['VT_TILT_MC', 0, 9], ['VT_TILT_TRANS', 0.4, 9], ['VT_TILT_FW', 1, 9], ['VT_TILT_SPINUP', 0, 9],
    ['VT_FW_MIN_ALT', 0, 9], ['VT_FW_QC_P', 0, 6], ['VT_FW_QC_R', 0, 6],
    ['FW_AIRSPD_STALL', 8, 9], ['FW_AIRSPD_MIN', 12, 9], ['FW_AIRSPD_TRIM', 15, 9], ['FW_USE_AIRSPD', 0, 6],
    ['SYS_HAS_NUM_ASPD', 0, 6], ['MIS_TAKEOFF_ALT', 2.5, 9], ['RTL_RETURN_ALT', 60, 9], ['RTL_DESCEND_ALT', 30, 9],
    ['NAV_DLL_ACT', 0, 6], ['COM_DL_LOSS_T', 10, 6], ['NAV_RCL_ACT', 2, 6], ['COM_DISARM_PRFLT', 10, 9],
    ['BAT1_N_CELLS', 3, 6], ['BAT_LOW_THR', 0.15, 9],
  ];
  const AP_PARAMS = [
    ['Q_ENABLE', 1, 2], ['Q_FRAME_CLASS', 7, 2], ['Q_TILT_MASK', 3, 6], ['Q_TILT_TYPE', 2, 2],
    ['Q_TILT_RATE_UP', 40, 4], ['Q_TILT_RATE_DN', 40, 4], ['Q_TILT_MAX', 45, 4], ['Q_TRANSITION_MS', 5000, 4],
    ['Q_TRANS_FAIL', 20, 4], ['Q_ASSIST_SPEED', 11, 9], ['Q_ASSIST_ANGLE', 30, 2], ['Q_ASSIST_ALT', 15, 4],
    ['AIRSPEED_MIN', 12, 9], ['AIRSPEED_CRUISE', 15, 9], ['ARSPD_TYPE', 0, 2], ['Q_RTL_ALT', 15, 4],
    ['Q_GUIDED_MODE', 1, 2], ['FS_GCS_ENABL', 0, 2], ['BATT_LOW_VOLT', 10.5, 9],
  ];

  class SimVehicle {
    constructor(onBytes, opts) {
      this.onBytes = onBytes;
      this.px4 = !!(opts && opts.autopilot === 'px4');
      this.enc = new MAV.Encoder(1, 1);
      this.radioEnc = new MAV.Encoder(51, 68); // SiK radios report RADIO_STATUS as sysid 51
      this.parser = new MAV.Parser((m) => this.handle(m));
      this.params = new Map((this.px4 ? PX4_PARAMS : AP_PARAMS).map(([k, v, t]) => [k, { v, t }]));
      this.reset();
    }

    reset() {
      Object.assign(this, {
        t: 0, armed: false, armedAt: 0, mode: this.px4 ? 'POSCTL' : M.QHOVER,
        vtol: MC, transT: 0, landed: true, landedAt: 0, takingOff: false,
        n: 0, e: 0, alt: 0, hdg: 0.6, speed: 0, climb: 0,
        roll: 0, pitch: 0, tilt: 0, yawTilt: 0,
        mah: 0, current: 0.8, cells: 3, capacity: 5000,
        hold: { n: 0, e: 0 }, holdAlt: 0, takeoffAlt: 20, afterTransition: null,
        windFrom: 250, windSpeed: 4.5,
      });
    }

    start() {
      this.timer = setInterval(() => this.step(0.05), 50);
      if (this.px4) this.text(6, 'PX4 v1.15.4 (demo simulator), CubeOrange+ — no hardware connected');
      else {
        this.text(6, 'ArduPlane V4.5.x (demo simulator)');
        this.text(6, 'CubeOrange+ — simulated, no hardware connected');
      }
    }

    stop() { clearInterval(this.timer); }

    write(bytes) { this.parser.feed(bytes); }

    send(name, values, enc) { this.onBytes((enc || this.enc).encode(name, values)); }

    text(severity, text) { this.send('STATUSTEXT', { severity, text }); }

    ack(command, result) { this.send('COMMAND_ACK', { command, result }); }

    param(name) { const p = this.params.get(name); return p ? p.v : 0; }

    get flying() { return this.alt > 0.3; }

    // ---------- modes ----------
    modeName() { return this.px4 ? this.mode : MAV.PLANE_MODES[this.mode]; }

    setMode(mode) {
      if (this.px4) {
        if (!MAV.PX4_MODES[mode]) return false;
        if (mode === 'MISSION') { this.text(4, 'Mission rejected: no mission uploaded'); return false; }
        if (mode === 'OFFBOARD') { this.text(4, 'Offboard: no setpoints from a companion computer'); return false; }
        if (mode === 'TAKEOFF' && !this.armed) { this.text(4, 'Takeoff denied: arm first'); return false; }
      } else if (MAV.PLANE_MODES[mode] === undefined) return false;
      if (mode === this.mode) return true;
      this.mode = mode;
      this.takingOff = false;
      if (['HOLD', M.GUIDED, M.LOITER, M.QLOITER, M.QHOVER].includes(mode)) this.setHoldHere();
      if (mode === 'TAKEOFF') { this.takeoffAlt = this.param('MIS_TAKEOFF_ALT'); this.takingOff = true; }
      if (!this.px4 && this.armed && this.flying) {
        // ArduPlane: the mode decides the airframe state.
        const wantFw = !AP_VTOL.has(mode);
        if (wantFw && (this.vtol === MC || this.vtol === TO_MC)) this.startTransition(FW);
        if (!wantFw && (this.vtol === FW || this.vtol === TO_FW)) this.startTransition(MC);
      }
      if (this.px4 && (mode === 'LAND' || mode === 'RTL') && this.vtol === TO_FW) this.startTransition(MC);
      return true;
    }

    setHoldHere() { this.hold = { n: this.n, e: this.e }; this.holdAlt = Math.max(this.alt, this.flying ? 5 : 0); }

    startTransition(to) {
      if (to === FW) {
        this.vtol = TO_FW; this.transT = 0;
        if (!this.px4) this.text(6, 'Transition started airspeed ' + this.speed.toFixed(1));
      } else {
        this.vtol = TO_MC; this.transT = 0;
        if (!this.px4) this.text(6, 'Transition VTOL');
      }
    }

    // ---------- MAVLink in ----------
    handle(m) {
      switch (m._name) {
        case 'COMMAND_LONG': return this.commandLong(m);
        case 'COMMAND_INT': return this.commandInt(m);
        case 'SET_MODE': this.setMode(m.custom_mode); return;
        case 'TIMESYNC':
          if (m.tc1 === 0) this.send('TIMESYNC', { tc1: Math.round(this.t * 1e9) + 1, ts1: m.ts1 });
          return;
        case 'PARAM_REQUEST_READ': {
          const p = this.params.get(m.param_id);
          if (p) this.sendParam(m.param_id);
          return;
        }
        case 'PARAM_SET': {
          const p = this.params.get(m.param_id);
          if (!p) return;
          p.v = MAV.paramFromWire(m.param_value, p.t, this.px4);
          if (p.t === 9) p.v = Math.round(p.v * 1e4) / 1e4;
          this.sendParam(m.param_id);
          return;
        }
      }
    }

    sendParam(id) {
      const p = this.params.get(id);
      const keys = [...this.params.keys()];
      this.send('PARAM_VALUE', {
        param_value: MAV.paramToWire(p.v, p.t, this.px4), param_count: keys.length,
        param_index: keys.indexOf(id), param_id: id, param_type: p.t,
      });
    }

    commandLong(m) {
      switch (m.command) {
        case 400: { // arm / disarm
          if (m.param1 === 1) {
            if (this.armed) return this.ack(400, 0);
            const ok = this.px4 ? PX4_ARMABLE.has(this.mode) : AP_ARMABLE.has(this.mode);
            if (!ok) {
              this.text(3, this.px4 ? 'Arming denied: switch to a manual or hold mode' : 'Arm: Mode not armable');
              return this.ack(400, 4);
            }
            this.armed = true; this.armedAt = this.t; this.landed = !this.flying;
            this.setHoldHere();
            this.text(6, this.px4 ? 'Armed by external command' : 'Arming motors');
            return this.ack(400, 0);
          }
          if (this.flying && Math.round(m.param2) !== 21196) {
            this.text(3, this.px4 ? 'Disarming denied: not landed' : 'Disarm refused: vehicle is flying');
            return this.ack(400, 4);
          }
          this.disarm(this.px4 ? 'Disarmed by external command' : 'Disarming motors');
          return this.ack(400, 0);
        }
        case 176: {
          let mode;
          if (this.px4) {
            const main = Math.round(m.param2), sub = Math.round(m.param3);
            mode = MAV.px4Mode(((main << 16) | (sub << 24)) >>> 0);
          } else mode = Math.round(m.param2);
          return this.ack(176, this.setMode(mode) ? 0 : 4);
        }
        case 22: { // takeoff
          if (!this.armed) { this.text(4, 'Takeoff denied: disarmed'); return this.ack(22, 4); }
          if (this.flying) { this.text(4, 'Takeoff denied: already flying'); return this.ack(22, 4); }
          // PX4 takes an AMSL altitude in param7, ArduPlane a height above home.
          let alt = this.px4 ? m.param7 - HOME.amsl : m.param7;
          if (!Number.isFinite(alt) || alt < 1) alt = this.px4 ? this.param('MIS_TAKEOFF_ALT') : 10;
          if (!this.px4 && this.mode !== M.GUIDED) { this.text(4, 'User Takeoff only in GUIDED mode'); return this.ack(22, 4); }
          if (this.px4) this.mode = 'TAKEOFF';
          this.takeoffAlt = alt;
          this.takingOff = true;
          this.setHoldHere();
          this.text(6, `Takeoff to ${alt.toFixed(1)} m above home`);
          return this.ack(22, 0);
        }
        case 3000: { // DO_VTOL_TRANSITION
          const to = Math.round(m.param1);
          if (to !== FW && to !== MC) return this.ack(3000, 2);
          if (!this.px4 && this.mode !== M.AUTO) { this.text(5, 'VTOL transition only in AUTO'); return this.ack(3000, 4); }
          if (!this.armed || !this.flying) { this.text(4, 'Transition denied: vehicle landed'); return this.ack(3000, 4); }
          if (to === FW && (this.vtol === FW || this.vtol === TO_FW)) return this.ack(3000, 0);
          if (to === MC && (this.vtol === MC || this.vtol === TO_MC)) return this.ack(3000, 0);
          this.startTransition(to);
          if (to === FW) this.hold = { n: this.n + 400 * Math.cos(this.hdg), e: this.e + 400 * Math.sin(this.hdg) };
          else this.setHoldHere();
          return this.ack(3000, 0);
        }
        case 20: return this.ack(20, this.setMode(this.px4 ? 'RTL' : M.QRTL) ? 0 : 4);
        case 21: return this.ack(21, this.setMode(this.px4 ? 'LAND' : M.QLAND) ? 0 : 4);
        case 512:
          if (Math.round(m.param1) === 242) this.sendHome();
          if (Math.round(m.param1) === 148) this.sendVersion();
          return this.ack(512, 0);
        case 511: return this.ack(511, 0);
        default: return this.ack(m.command, 3);
      }
    }

    commandInt(m) {
      if (m.command !== 192) return this.ack(m.command, 3);
      if (!this.armed || !this.flying) {
        this.text(4, 'Reposition rejected: not flying');
        return this.ack(192, 4);
      }
      const lat = m.x / 1e7, lon = m.y / 1e7;
      this.hold = {
        n: (lat - HOME.lat) * Math.PI / 180 * R_EARTH,
        e: (lon - HOME.lon) * Math.PI / 180 * R_EARTH * Math.cos(HOME.lat * Math.PI / 180),
      };
      if (Number.isFinite(m.z)) this.holdAlt = m.frame === 0 ? m.z - HOME.amsl : m.z;
      const hold = this.hold, holdAlt = this.holdAlt;
      this.setMode(this.px4 ? 'HOLD' : M.GUIDED);
      this.hold = hold; this.holdAlt = holdAlt;
      return this.ack(192, 0);
    }

    disarm(why) {
      this.armed = false;
      this.takingOff = false;
      this.speed = 0; this.climb = 0; this.tilt = 0; this.vtol = MC;
      this.text(6, why);
    }

    // ---------- motion ----------
    // Turn towards a point and fly at up to `speed`; returns the distance left.
    steerTo(p, speed, dt, turnRate) {
      const dn = p.n - this.n, de = p.e - this.e;
      const dist = Math.hypot(dn, de);
      const want = dist < 2 ? this.hdg : Math.atan2(de, dn);
      let err = want - this.hdg;
      while (err > Math.PI) err -= 2 * Math.PI;
      while (err < -Math.PI) err += 2 * Math.PI;
      const turn = Math.max(-turnRate, Math.min(turnRate, err * 1.5));
      this.hdg += turn * dt;
      this.accelTo(Math.min(speed, dist * 0.6), dt);
      this.roll = this.vtol === FW ? Math.atan(this.speed * turn / 9.81) : 0;
      return dist;
    }

    accelTo(v, dt, rate) { const a = (rate || 2.5) * dt; this.speed += Math.max(-a, Math.min(a, v - this.speed)); }

    orbit(c, radius, speed, dt) {
      const dn = this.n - c.n, de = this.e - c.e;
      const dist = Math.hypot(dn, de);
      if (dist > radius * 1.3) return this.steerTo(c, speed, dt, 0.5);
      const ang = Math.atan2(de, dn) + 0.4;
      this.steerTo({ n: c.n + radius * Math.cos(ang), e: c.e + radius * Math.sin(ang) }, speed, dt, 0.5);
      this.speed = Math.max(this.speed, speed * 0.95);
      return dist;
    }

    climbTo(alt, dt) {
      const up = this.vtol === FW ? 3 : 2, down = this.alt > 5 ? 1.5 : 0.6;
      this.climb = Math.max(-down, Math.min(up, (alt - this.alt) * 0.8));
    }

    // What the current mode wants, as a multicopter or as a plane.
    behaviour() {
      const m = this.modeName();
      if (['LAND', 'QLAND'].includes(m)) return 'land';
      if (['RTL', 'QRTL'].includes(m)) return 'rtl';
      if (this.takingOff) return 'takeoff';
      return 'hold';
    }

    step(dt) {
      this.t += dt;
      const home = { n: 0, e: 0 };
      const fwSpeed = this.px4 ? this.param('FW_AIRSPD_TRIM') : this.param('AIRSPEED_CRUISE');

      // Transition state machine.
      if (this.vtol === TO_FW) {
        this.transT += dt;
        const done = this.px4
          ? this.transT >= Math.max(this.param('VT_F_TR_OL_TM'), this.param('VT_TRANS_MIN_TM'))
          : this.speed >= this.param('AIRSPEED_MIN') && this.tilt >= this.param('Q_TILT_MAX') - 1;
        if (done) {
          this.vtol = FW;
          if (!this.px4) this.text(6, 'Transition done');
          if (this.px4) { this.hold = { n: this.n + 150 * Math.cos(this.hdg), e: this.e + 150 * Math.sin(this.hdg) }; }
        } else if (this.px4 && this.transT > this.param('VT_TRANS_TIMEOUT')) {
          this.text(2, 'Quadchute: transition timeout');
          this.startTransition(MC); this.mode = 'HOLD'; this.setHoldHere();
        }
      } else if (this.vtol === TO_MC) {
        this.transT += dt;
        const dur = this.px4 ? this.param('VT_B_TRANS_DUR') : 4;
        if (this.transT >= dur || this.speed < 3) {
          this.vtol = MC;
          if (!this.px4) this.text(6, 'VTOL position reached');
          if (this.behaviour() === 'hold') this.setHoldHere();
        }
      }

      if (this.armed) {
        const b = this.behaviour();
        if (this.vtol === MC) {
          switch (b) {
            case 'takeoff':
              this.accelTo(0, dt);
              this.climbTo(this.takeoffAlt, dt);
              if (this.alt >= this.takeoffAlt - 0.3) {
                this.takingOff = false;
                if (this.px4) this.mode = 'HOLD';
                this.setHoldHere();
                this.holdAlt = this.takeoffAlt;
              }
              break;
            case 'land':
              this.accelTo(0, dt, 1.5);
              this.climb = this.alt > 5 ? -1.5 : -0.6;
              break;
            case 'rtl': {
              const rtlAlt = this.px4 ? this.param('RTL_RETURN_ALT') : this.param('Q_RTL_ALT');
              const d = Math.hypot(this.n, this.e);
              if (d > 3) {
                this.climbTo(Math.max(this.alt, Math.min(rtlAlt, 40)), dt);
                if (this.alt > Math.min(rtlAlt, 40) - 2 || this.alt > 25) this.steerTo(home, 5, dt, 0.8);
              } else {
                this.accelTo(0, dt);
                this.climb = this.alt > 5 ? -1.5 : -0.6;
              }
              break;
            }
            default: {
              if (this.landed) { this.climb = 0; break; } // on the ground until a takeoff is commanded
              this.steerTo(this.hold, 5, dt, 0.8);
              this.climbTo(this.holdAlt, dt);
              this.roll = 0.03 * Math.sin(this.t * 1.3);
            }
          }
        } else if (this.vtol === TO_FW) {
          this.accelTo(fwSpeed, dt, 2.2);
          this.climbTo(this.holdAlt, dt);
          this.roll = 0;
        } else if (this.vtol === TO_MC) {
          this.accelTo(0, dt, 2.5);
          this.climbTo(this.holdAlt, dt);
          this.roll = 0;
        } else { // FW
          if (b === 'land' || (b === 'rtl' && this.px4 && Math.hypot(this.n, this.e) < 90)) this.startTransition(MC);
          else if (b === 'rtl') { this.orbit(home, 80, fwSpeed, dt); this.climbTo(Math.max(this.alt, 30), dt); }
          else { this.orbit(this.hold, 80, fwSpeed, dt); this.climbTo(this.holdAlt, dt); }
        }
      }

      // Tilt: 0° up, 90° forward. PX4 parks at VT_TILT_TRANS during the transition.
      let tiltWant = 0;
      if (this.armed && this.flying) {
        if (this.vtol === FW) tiltWant = 90;
        else if (this.vtol === TO_FW) tiltWant = this.px4 ? this.param('VT_TILT_TRANS') * 90 : Math.min(90, this.param('Q_TILT_MAX') + (this.speed >= this.param('AIRSPEED_MIN') ? 45 : 0));
      }
      this.tilt += Math.max(-dt * 60, Math.min(dt * 60, tiltWant - this.tilt));
      this.yawTilt = this.armed && this.flying && this.vtol === MC ? 3 * Math.sin(this.t * 0.9) : 0;

      this.alt = Math.max(0, this.alt + this.climb * dt);
      if (this.flying && this.landed) this.landed = false;
      if (this.armed && !this.landed && this.alt === 0 && this.climb <= 0) {
        this.landed = true; this.landedAt = this.t;
        this.text(6, this.px4 ? 'Landing detected' : 'Land complete');
        if (!this.px4) { this.disarm('Disarming motors'); this.mode = M.QHOVER; }
      }
      if (this.px4 && this.armed && this.landed) {
        const since = this.t - Math.max(this.armedAt, this.landedAt);
        const limit = this.landedAt > this.armedAt ? 2 : this.param('COM_DISARM_PRFLT');
        if (limit > 0 && since > limit && this.behaviour() !== 'takeoff') {
          this.disarm(this.landedAt > this.armedAt ? 'Disarmed by landing' : 'Disarmed by auto preflight disarming');
          this.mode = 'POSCTL';
        }
      }
      if (!this.armed) this.climb = 0;
      this.n += this.speed * Math.cos(this.hdg) * dt;
      this.e += this.speed * Math.sin(this.hdg) * dt;
      this.pitch = this.vtol === FW ? 0.05 + this.climb * 0.03 : -this.speed * 0.02;

      const hover = this.armed && this.flying ? 1 - this.tilt / 90 : 0;
      this.current = !this.armed ? 0.8 : this.flying ? 10 + 22 * hover + Math.random() : 3;
      this.mah += this.current * dt / 3.6;

      const tick = Math.round(this.t / dt);
      if (tick % 20 === 0) this.sendSlow();
      if (tick % 100 === 50) this.sendHome();
      if (this.px4 && tick % 100 === 25) this.send('TIMESYNC', { tc1: 0, ts1: Math.round(this.t * 1e9) }); // PX4 asks too
      if (tick % 2 === 0) this.send('ATTITUDE', {
        time_boot_ms: this.t * 1000, roll: this.roll, pitch: this.pitch, yaw: this.hdg,
        rollspeed: 0, pitchspeed: 0, yawspeed: 0,
      });
      if (tick % 4 === 0) this.sendFast();
    }

    // ---------- MAVLink out ----------
    latLon() {
      const lat = HOME.lat + (this.n / R_EARTH) * 180 / Math.PI;
      const lon = HOME.lon + (this.e / (R_EARTH * Math.cos(HOME.lat * Math.PI / 180))) * 180 / Math.PI;
      return { lat, lon };
    }

    sendHome() {
      this.send('HOME_POSITION', {
        latitude: Math.round(HOME.lat * 1e7), longitude: Math.round(HOME.lon * 1e7),
        altitude: HOME.amsl * 1000, q: [1, 0, 0, 0],
      });
    }

    sendVersion() {
      const v = this.px4 ? [1, 15, 4] : [4, 5, 7];
      this.send('AUTOPILOT_VERSION', {
        capabilities: this.px4 ? 0xe4ef : 0xfbef, uid: 0,
        flight_sw_version: ((v[0] << 24) | (v[1] << 16) | (v[2] << 8) | 255) >>> 0,
        vendor_id: 0x2dae, product_id: 0x1058,
      });
    }

    sendFast() {
      const { lat, lon } = this.latLon();
      const hdgDeg = ((this.hdg * 180 / Math.PI) % 360 + 360) % 360;
      const vn = this.speed * Math.cos(this.hdg), ve = this.speed * Math.sin(this.hdg);
      this.send('GLOBAL_POSITION_INT', {
        time_boot_ms: this.t * 1000, lat: Math.round(lat * 1e7), lon: Math.round(lon * 1e7),
        alt: Math.round((HOME.amsl + this.alt) * 1000), relative_alt: Math.round(this.alt * 1000),
        vx: Math.round(vn * 100), vy: Math.round(ve * 100), vz: Math.round(-this.climb * 100),
        hdg: Math.round(hdgDeg * 100),
      });
      const fw = this.vtol === FW;
      const thr = !this.armed ? 0 : this.flying ? Math.round(fw ? 48 : 40 + 14 * (1 - this.tilt / 90)) : 10;
      // Air-relative velocity = ground velocity - wind velocity (wind blows towards windFrom + 180).
      const wf = this.windFrom * Math.PI / 180, ws = this.windSpeed + 0.6 * Math.sin(this.t * 0.4);
      const airspeed = Math.hypot(vn + ws * Math.cos(wf), ve + ws * Math.sin(wf));
      this.send('VFR_HUD', {
        airspeed: this.flying ? airspeed : 0,
        groundspeed: this.speed, alt: HOME.amsl + this.alt, climb: this.climb,
        heading: Math.round(hdgDeg), throttle: thr,
      });
      // Outputs as wired on the Cube: MAIN1 front-right, MAIN2 front-left, MAIN3 rear, MAIN5/6 tilt L/R, MAIN7/8 ailerons.
      // The rear motor runs a little harder than the front pair, as it would with the CG slightly aft.
      const wob = (k) => Math.round(18 * Math.sin(this.t * 7 + k));
      const idle = this.armed ? 1100 : 1000;
      const hoverPwm = this.flying ? 1540 : idle;
      const front = !this.armed ? 1000 : fw ? 1430 : this.vtol === TO_FW ? 1700 : hoverPwm;
      const rear = !this.armed ? 1000 : fw ? 1000 : this.vtol === TO_FW ? Math.round(hoverPwm - 300 * this.tilt / 90) : this.flying ? hoverPwm + 45 : idle;
      const tiltPwm = (deg) => Math.round(1100 + (Math.max(-8, Math.min(90, deg)) / 90) * 800);
      const ail = (s) => Math.round(1500 + s * 200 * this.roll);
      this.send('SERVO_OUTPUT_RAW', {
        time_usec: this.t * 1e6,
        servo1_raw: front > 1000 ? front + wob(0) : front,
        servo2_raw: front > 1000 ? front + wob(2) : front,
        servo3_raw: rear > 1000 ? rear + wob(4) : rear,
        servo4_raw: 0,
        servo5_raw: tiltPwm(this.tilt + this.yawTilt),
        servo6_raw: tiltPwm(this.tilt - this.yawTilt),
        servo7_raw: ail(1),
        servo8_raw: ail(-1),
      });
    }

    sendSlow() {
      const used = this.mah / this.capacity;
      const volts = this.cells * (4.2 - 0.75 * used) - this.current * 0.012;
      let customMode, baseMode = 1 | (this.armed ? 128 : 0);
      if (this.px4) {
        customMode = MAV.px4CustomMode(this.mode);
        baseMode |= ['HOLD', 'TAKEOFF', 'MISSION', 'RTL', 'LAND', 'POSCTL'].includes(this.mode) ? 16 | 8 | 4 : 16 | 64;
      } else {
        customMode = this.mode;
        baseMode |= 4 | 16;
      }
      this.send('HEARTBEAT', {
        custom_mode: customMode, type: this.px4 ? 21 : 21, autopilot: this.px4 ? 12 : 3, base_mode: baseMode,
        system_status: this.armed ? 4 : 3, mavlink_version: 3,
      });
      this.send('EXTENDED_SYS_STATE', {
        vtol_state: this.vtol,
        landed_state: !this.flying ? 1 : this.behaviour() === 'takeoff' ? 3 : this.climb < -0.3 && ['land', 'rtl'].includes(this.behaviour()) ? 4 : 2,
      });
      this.send('SYS_STATUS', {
        sensors_present: 0x0020002f, sensors_enabled: 0x0020002f, sensors_health: 0x0020002f,
        load: 350, voltage_battery: Math.round(volts * 1000), current_battery: Math.round(this.current * 100),
        battery_remaining: Math.max(0, Math.round(100 * (1 - used))),
      });
      const wd = this.windFrom + 4 * Math.sin(this.t * 0.2), wsp = this.windSpeed + 0.6 * Math.sin(this.t * 0.4);
      if (this.px4) {
        // WIND_COV carries the velocity the air moves with (towards), north/east.
        const to = (wd + 180) * Math.PI / 180;
        this.send('WIND_COV', { time_usec: this.t * 1e6, wind_x: wsp * Math.cos(to), wind_y: wsp * Math.sin(to), wind_z: 0, var_horiz: 0.4, var_vert: 0.1, wind_alt: HOME.amsl + this.alt, horiz_accuracy: 0, vert_accuracy: 0 });
      } else {
        this.send('WIND', { direction: wd, speed: wsp, speed_z: 0 });
      }
      const { lat, lon } = this.latLon();
      this.send('GPS_RAW_INT', {
        time_usec: this.t * 1e6, lat: Math.round(lat * 1e7), lon: Math.round(lon * 1e7),
        alt: Math.round((HOME.amsl + this.alt) * 1000), eph: 70, epv: 110,
        vel: Math.round(this.speed * 100), cog: 0, fix_type: 3, satellites_visible: 17,
      });
      const ch = { time_boot_ms: this.t * 1000, chancount: 12, rssi: 230 };
      for (let i = 1; i <= 18; i++) ch['chan' + i + '_raw'] = i <= 12 ? 1500 : 0;
      ch.chan3_raw = this.armed ? 1500 : 1000;
      ch.chan5_raw = this.vtol === FW ? 1900 : 1500;
      this.send('RC_CHANNELS', ch);
      const dist = Math.hypot(this.n, this.e);
      const rssi = Math.max(60, Math.round(210 - dist / 8 + 4 * Math.sin(this.t)));
      this.send('RADIO_STATUS', {
        rssi, remrssi: rssi - 6, txbuf: 95, noise: 38, remnoise: 41, rxerrors: 0, fixed: 0,
      }, this.radioEnc);
      // A bit of traffic so the ADS-B layer has something to show.
      const a = this.t * 0.01;
      this.send('ADSB_VEHICLE', {
        ICAO_address: 0x800abc, lat: Math.round((HOME.lat + 0.03 * Math.cos(a)) * 1e7), lon: Math.round((HOME.lon + 0.045 * Math.sin(a)) * 1e7),
        altitude: 1200000, heading: Math.round((((a * 180 / Math.PI) + 90) % 360) * 100), hor_velocity: 6000, ver_velocity: 0,
        flags: 0x1f, squawk: 7000, altitude_type: 0, callsign: 'VT-DMO', emitter_type: 1, tslc: 1,
      });
    }
  }

  if (typeof module !== 'undefined' && module.exports) module.exports = SimVehicle;
  else root.SimVehicle = SimVehicle;
})(typeof window !== 'undefined' ? window : globalThis);
