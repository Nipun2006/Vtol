// Demo vehicle: a very rough tri tilt-rotor that speaks MAVLink like ArduPlane would.
// It exists so the dashboard can be tried without hardware. It is not a flight model.
(function (root) {
  'use strict';
  const MAV = root.MAV || (typeof require !== 'undefined' ? require('./mavlink.js') : null);
  const M = MAV.MODE_NUMBERS;
  const VTOL_MODES = new Set([M.QSTABILIZE, M.QHOVER, M.QLOITER, M.QLAND, M.QRTL, M.QACRO]);
  const ARMABLE = new Set([M.QSTABILIZE, M.QHOVER, M.QLOITER, M.QACRO, M.FBWA, M.MANUAL, M.STABILIZE]);
  const HOME = { lat: 47.397742, lon: 8.545594, amsl: 488 };
  const R_EARTH = 6378137;

  class SimVehicle {
    constructor(onBytes) {
      this.onBytes = onBytes;
      this.enc = new MAV.Encoder(1, 1);
      this.radioEnc = new MAV.Encoder(51, 68); // SiK radios report RADIO_STATUS as sysid 51
      this.parser = new MAV.Parser((m) => this.handle(m));
      this.reset();
    }

    reset() {
      Object.assign(this, {
        t: 0, armed: false, mode: M.QHOVER,
        n: 0, e: 0, alt: 0, hdg: 0, speed: 0, climb: 0,
        roll: 0, pitch: 0, tilt: 0, yawTilt: 0,
        mah: 0, current: 0.8, cells: 4, capacity: 5000,
        target: null, targetAlt: 30, holdAlt: 15,
      });
    }

    start() {
      this.timer = setInterval(() => this.step(0.05), 50);
      this.text(6, 'ArduPlane V4.5.x (demo simulator)');
      this.text(6, 'CubeOrange+ — simulated, no hardware connected');
    }

    stop() { clearInterval(this.timer); }

    write(bytes) { this.parser.feed(bytes); }

    send(name, values, enc) { this.onBytes((enc || this.enc).encode(name, values)); }

    text(severity, text) { this.send('STATUSTEXT', { severity, text }); }

    ack(command, result) { this.send('COMMAND_ACK', { command, result }); }

    get flying() { return this.alt > 0.3; }
    get forward() { return !VTOL_MODES.has(this.mode); }

    setMode(mode) {
      if (MAV.PLANE_MODES[mode] === undefined) return false;
      if (mode === this.mode) return true;
      const wasForward = this.forward;
      this.mode = mode;
      if (mode === M.GUIDED && !this.target) this.target = { n: this.n, e: this.e };
      if (this.armed && this.flying) {
        if (!wasForward && this.forward) this.text(6, 'Transition started');
        if (wasForward && !this.forward) this.text(6, 'Transition to VTOL');
      }
      return true;
    }

    handle(m) {
      switch (m._name) {
        case 'COMMAND_LONG': return this.commandLong(m);
        case 'COMMAND_INT': return this.commandInt(m);
        case 'SET_MODE': this.setMode(m.custom_mode); return;
      }
    }

    commandLong(m) {
      switch (m.command) {
        case 400: { // arm / disarm
          if (m.param1 === 1) {
            if (this.armed) return this.ack(400, 0);
            if (!ARMABLE.has(this.mode)) {
              this.text(3, 'Arm: Mode not armable');
              return this.ack(400, 4);
            }
            this.armed = true;
            this.text(6, 'Arming motors');
            return this.ack(400, 0);
          }
          if (this.flying && Math.round(m.param2) !== 21196) {
            this.text(3, 'Disarm refused: vehicle is flying');
            return this.ack(400, 4);
          }
          this.disarm();
          return this.ack(400, 0);
        }
        case 176: return this.ack(176, this.setMode(Math.round(m.param2)) ? 0 : 2);
        case 512:
          if (Math.round(m.param1) === 242) this.sendHome();
          return this.ack(512, 0);
        case 511: return this.ack(511, 0);
        default: return this.ack(m.command, 3);
      }
    }

    commandInt(m) {
      if (m.command !== 192) return this.ack(m.command, 3);
      if (!this.armed || !this.flying) {
        this.text(4, 'Reposition rejected: not flying');
        return this.ack(192, 2);
      }
      const lat = m.x / 1e7, lon = m.y / 1e7;
      this.target = {
        n: (lat - HOME.lat) * Math.PI / 180 * R_EARTH,
        e: (lon - HOME.lon) * Math.PI / 180 * R_EARTH * Math.cos(HOME.lat * Math.PI / 180),
      };
      if (m.z > 0) this.targetAlt = m.z;
      this.setMode(M.GUIDED);
      return this.ack(192, 0);
    }

    disarm() {
      this.armed = false;
      this.speed = 0; this.climb = 0; this.tilt = 0;
      this.text(6, 'Disarming motors');
    }

    // Fly towards a point at the given speed; returns the distance left.
    steerTo(p, speed, dt, turnRate) {
      const dn = p.n - this.n, de = p.e - this.e;
      const dist = Math.hypot(dn, de);
      const want = Math.atan2(de, dn);
      let err = want - this.hdg;
      while (err > Math.PI) err -= 2 * Math.PI;
      while (err < -Math.PI) err += 2 * Math.PI;
      const turn = Math.max(-turnRate, Math.min(turnRate, err * 1.5));
      this.hdg += turn * dt;
      this.speed += (Math.min(speed, dist) - this.speed) * Math.min(1, dt * 0.8);
      this.roll = this.forward ? Math.atan(this.speed * turn / 9.81) : 0;
      return dist;
    }

    orbit(c, radius, speed, dt) {
      const dn = this.n - c.n, de = this.e - c.e;
      const dist = Math.hypot(dn, de);
      if (dist > radius * 1.3) return this.steerTo(c, speed, dt, 0.5);
      const ang = Math.atan2(de, dn) + 0.4;
      return this.steerTo({ n: c.n + radius * Math.cos(ang), e: c.e + radius * Math.sin(ang) }, speed, dt, 0.5);
    }

    climbTo(alt, rate, dt) {
      const d = alt - this.alt;
      this.climb = Math.max(-rate, Math.min(rate, d));
    }

    step(dt) {
      this.t += dt;
      const home = { n: 0, e: 0 };
      if (this.armed) {
        switch (this.mode) {
          case M.QSTABILIZE: case M.QHOVER: case M.QLOITER: case M.QACRO:
            this.climbTo(this.holdAlt, 1.5, dt);
            this.speed *= 1 - dt * 0.8;
            this.roll = 0.03 * Math.sin(this.t * 1.3);
            break;
          case M.QLAND:
            this.speed *= 1 - dt;
            this.climb = this.alt > 5 ? -1.5 : -0.5;
            break;
          case M.QRTL: {
            const d = this.steerTo(home, 6, dt, 0.8);
            this.climbTo(Math.max(this.alt, 15), 2, dt);
            if (d < 3) this.climb = this.alt > 5 ? -1.5 : -0.5;
            break;
          }
          case M.RTL:
            this.orbit(home, 80, 18, dt);
            this.climbTo(this.targetAlt, 3, dt);
            break;
          case M.GUIDED:
            this.orbit(this.target || home, 60, 18, dt);
            this.climbTo(this.targetAlt, 3, dt);
            break;
          default: // FBWA, CRUISE, AUTO, LOITER...
            if (this.alt < 1) { this.climbTo(this.holdAlt, 1.5, dt); break; }
            this.orbit({ n: 150, e: 0 }, 120, 18, dt);
            this.climbTo(this.targetAlt, 2, dt);
        }
      }

      // Tilt follows the mode: vertical in VTOL, forward once a forward-flight mode is in.
      const tiltWant = this.armed && this.flying && this.forward ? 90 : 0;
      const prevTilt = this.tilt;
      this.tilt += Math.max(-dt * 40, Math.min(dt * 40, tiltWant - this.tilt));
      if (prevTilt < 90 && this.tilt >= 90) this.text(6, 'Transition done');
      if (prevTilt > 0 && this.tilt <= 0 && this.flying) this.text(6, 'VTOL position reached');
      this.yawTilt = this.armed && this.flying && this.tilt < 30 ? 3 * Math.sin(this.t * 0.9) : 0;

      this.alt = Math.max(0, this.alt + this.climb * dt);
      if (this.armed && this.alt === 0 && this.climb < 0 && (this.mode === M.QLAND || this.mode === M.QRTL)) {
        this.text(6, 'Land complete');
        this.disarm();
        this.mode = M.QHOVER;
      }
      this.n += this.speed * Math.cos(this.hdg) * dt;
      this.e += this.speed * Math.sin(this.hdg) * dt;
      this.pitch = this.forward && this.flying ? 0.05 + this.climb * 0.03 : -this.speed * 0.02;

      const hover = this.armed && this.flying ? 1 - this.tilt / 90 : 0;
      this.current = !this.armed ? 0.8 : this.flying ? 10 + 22 * hover + Math.random() : 3;
      this.mah += this.current * dt / 3.6;

      const tick = Math.round(this.t / dt);
      if (tick % 20 === 0) this.sendSlow();
      if (tick % 2 === 0) this.send('ATTITUDE', {
        time_boot_ms: this.t * 1000, roll: this.roll, pitch: this.pitch, yaw: this.hdg,
        rollspeed: 0, pitchspeed: 0, yawspeed: 0,
      });
      if (tick % 4 === 0) this.sendFast();
    }

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
      const thr = !this.armed ? 0 : this.flying ? Math.round(45 + 12 * (1 - this.tilt / 90)) : 10;
      this.send('VFR_HUD', {
        airspeed: this.forward ? this.speed + 0.5 * Math.sin(this.t) : this.speed * 0.6,
        groundspeed: this.speed, alt: HOME.amsl + this.alt, climb: this.climb,
        heading: Math.round(hdgDeg), throttle: thr,
      });
      const motor = (on, base) => (!this.armed ? 1000 : on ? Math.round(base + 25 * Math.sin(this.t * 7)) : 1000);
      const hoverPwm = this.flying ? 1560 : 1150;
      const tiltPwm = (deg) => Math.round(1100 + (Math.max(-8, Math.min(90, deg)) / 90) * 800);
      const elevon = (s) => Math.round(1500 + s * 200 * this.roll);
      this.send('SERVO_OUTPUT_RAW', {
        time_usec: this.t * 1e6,
        servo1_raw: motor(true, this.tilt > 80 ? 1450 : hoverPwm),
        servo2_raw: motor(true, this.tilt > 80 ? 1450 : hoverPwm),
        servo3_raw: motor(this.tilt < 85, hoverPwm),
        servo4_raw: 0,
        servo5_raw: tiltPwm(this.tilt + this.yawTilt),
        servo6_raw: tiltPwm(this.tilt - this.yawTilt),
        servo7_raw: elevon(1),
        servo8_raw: elevon(-1),
      });
    }

    sendSlow() {
      const used = this.mah / this.capacity;
      const volts = this.cells * (4.2 - 0.75 * used) - this.current * 0.012;
      const modeFlags = 1 | (this.armed ? 128 : 0) | 4 | 16;
      this.send('HEARTBEAT', {
        custom_mode: this.mode, type: 21, autopilot: 3, base_mode: modeFlags,
        system_status: this.armed ? 4 : 3, mavlink_version: 3,
      });
      this.send('SYS_STATUS', {
        load: 350, voltage_battery: Math.round(volts * 1000), current_battery: Math.round(this.current * 100),
        battery_remaining: Math.max(0, Math.round(100 * (1 - used))),
      });
      const { lat, lon } = this.latLon();
      this.send('GPS_RAW_INT', {
        time_usec: this.t * 1e6, lat: Math.round(lat * 1e7), lon: Math.round(lon * 1e7),
        alt: Math.round((HOME.amsl + this.alt) * 1000), eph: 70, epv: 110,
        vel: Math.round(this.speed * 100), cog: 0, fix_type: 3, satellites_visible: 17,
      });
      const ch = { time_boot_ms: this.t * 1000, chancount: 12, rssi: 230 };
      for (let i = 1; i <= 18; i++) ch['chan' + i + '_raw'] = i <= 12 ? 1500 : 0;
      ch.chan3_raw = this.armed ? 1500 : 1000;
      ch.chan5_raw = this.forward ? 1900 : 1500;
      this.send('RC_CHANNELS', ch);
      const dist = Math.hypot(this.n, this.e);
      const rssi = Math.max(60, Math.round(210 - dist / 8 + 4 * Math.sin(this.t)));
      this.send('RADIO_STATUS', {
        rssi, remrssi: rssi - 6, txbuf: 95, noise: 38, remnoise: 41, rxerrors: 0, fixed: 0,
      }, this.radioEnc);
    }
  }

  if (typeof module !== 'undefined' && module.exports) module.exports = SimVehicle;
  else root.SimVehicle = SimVehicle;
})(typeof window !== 'undefined' ? window : globalThis);
