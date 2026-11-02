// Minimal MAVLink v1/v2 codec for the messages the dashboard uses.
// Field layouts and CRC extras match pymavlink's common/ardupilotmega dialect.
(function (root) {
  'use strict';

  // name: [id, crcExtra, structFormat, fieldNames]
  const DEFS = {
    HEARTBEAT: [0, 50, 'IBBBBB', ['custom_mode', 'type', 'autopilot', 'base_mode', 'system_status', 'mavlink_version']],
    SYS_STATUS: [1, 124, 'IIIHHhHHHHHHb', ['sensors_present', 'sensors_enabled', 'sensors_health', 'load', 'voltage_battery', 'current_battery', 'drop_rate_comm', 'errors_comm', 'errors_count1', 'errors_count2', 'errors_count3', 'errors_count4', 'battery_remaining']],
    SET_MODE: [11, 89, 'IBB', ['custom_mode', 'target_system', 'base_mode']],
    GPS_RAW_INT: [24, 24, 'QiiiHHHHBB', ['time_usec', 'lat', 'lon', 'alt', 'eph', 'epv', 'vel', 'cog', 'fix_type', 'satellites_visible']],
    ATTITUDE: [30, 39, 'Iffffff', ['time_boot_ms', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed']],
    GLOBAL_POSITION_INT: [33, 104, 'IiiiihhhH', ['time_boot_ms', 'lat', 'lon', 'alt', 'relative_alt', 'vx', 'vy', 'vz', 'hdg']],
    SERVO_OUTPUT_RAW: [36, 222, 'IHHHHHHHHBHHHHHHHH', ['time_usec', 'servo1_raw', 'servo2_raw', 'servo3_raw', 'servo4_raw', 'servo5_raw', 'servo6_raw', 'servo7_raw', 'servo8_raw', 'port', 'servo9_raw', 'servo10_raw', 'servo11_raw', 'servo12_raw', 'servo13_raw', 'servo14_raw', 'servo15_raw', 'servo16_raw']],
    RC_CHANNELS: [65, 118, 'IHHHHHHHHHHHHHHHHHHBB', ['time_boot_ms', 'chan1_raw', 'chan2_raw', 'chan3_raw', 'chan4_raw', 'chan5_raw', 'chan6_raw', 'chan7_raw', 'chan8_raw', 'chan9_raw', 'chan10_raw', 'chan11_raw', 'chan12_raw', 'chan13_raw', 'chan14_raw', 'chan15_raw', 'chan16_raw', 'chan17_raw', 'chan18_raw', 'chancount', 'rssi']],
    REQUEST_DATA_STREAM: [66, 148, 'HBBBB', ['req_message_rate', 'target_system', 'target_component', 'req_stream_id', 'start_stop']],
    VFR_HUD: [74, 20, 'ffffhH', ['airspeed', 'groundspeed', 'alt', 'climb', 'heading', 'throttle']],
    COMMAND_INT: [75, 158, 'ffffiifHBBBBB', ['param1', 'param2', 'param3', 'param4', 'x', 'y', 'z', 'command', 'target_system', 'target_component', 'frame', 'current', 'autocontinue']],
    COMMAND_LONG: [76, 152, 'fffffffHBBB', ['param1', 'param2', 'param3', 'param4', 'param5', 'param6', 'param7', 'command', 'target_system', 'target_component', 'confirmation']],
    COMMAND_ACK: [77, 143, 'HB', ['command', 'result']],
    RADIO_STATUS: [109, 185, 'HHBBBBB', ['rxerrors', 'fixed', 'rssi', 'remrssi', 'txbuf', 'noise', 'remnoise']],
    HOME_POSITION: [242, 104, 'iiifff4ffff', ['latitude', 'longitude', 'altitude', 'x', 'y', 'z', 'q', 'approach_x', 'approach_y', 'approach_z']],
    STATUSTEXT: [253, 83, 'B50s', ['severity', 'text']],
  };

  // CRC extras for every ardupilotmega message, so frames we don't decode can still be
  // validated before they are skipped (otherwise a stray 0xFD byte can swallow good frames).
  const CRC_EXTRA = {0:50,1:124,2:137,4:237,5:217,6:104,7:119,11:89,20:214,21:159,22:220,23:168,24:24,25:23,26:170,27:144,28:67,29:115,30:39,31:246,32:185,33:104,34:237,35:244,36:222,37:212,38:9,39:254,40:230,41:28,42:28,43:132,44:221,45:232,46:11,47:153,48:41,49:39,50:78,51:196,54:15,55:3,61:167,62:183,63:119,64:191,65:118,66:148,67:21,69:243,70:124,73:38,74:20,75:158,76:152,77:143,81:106,82:49,83:22,84:143,85:140,86:5,87:150,89:231,90:183,91:63,92:54,93:47,100:175,101:102,102:158,103:208,104:56,105:93,106:138,107:108,108:32,109:185,110:84,111:34,112:174,113:124,114:237,115:4,116:76,117:128,118:56,119:116,120:134,121:237,122:203,123:250,124:87,125:203,126:220,127:25,128:226,129:46,130:29,131:223,132:85,133:6,134:229,135:203,136:1,137:195,138:109,139:168,140:181,141:47,142:72,143:131,144:127,146:103,147:154,148:178,149:200,150:134,151:219,152:208,153:188,154:84,155:22,156:19,157:21,158:134,160:78,161:68,162:189,163:127,164:154,165:21,166:21,167:144,168:1,169:234,170:73,171:181,172:22,173:83,174:167,175:138,176:234,177:240,178:47,179:189,180:52,181:174,182:229,183:85,184:159,185:186,186:72,191:92,192:36,193:71,194:98,195:120,200:134,201:205,214:69,215:101,216:50,217:202,218:17,219:162,225:208,226:207,230:163,231:105,232:151,233:35,234:150,235:179,241:90,242:104,243:85,244:95,245:130,246:184,247:81,248:8,249:204,250:49,251:170,252:44,253:83,254:46,256:71,257:131,258:187,259:92,260:146,261:179,262:12,263:133,264:49,265:26,266:193,267:35,268:14,269:109,270:59,271:22,275:126,276:18,277:62,280:70,281:48,282:123,283:74,284:99,285:137,286:210,287:1,288:20,299:19,301:243,310:28,311:95,320:243,321:88,322:243,323:78,324:132,330:23,331:91,332:236,333:231,335:225,339:199,340:99,350:232,370:75,373:117,375:251,376:199,385:147,386:132,387:4,388:8,9000:113,9005:117,10001:209,10002:186,10003:4,10004:133,10005:103,10006:193,10007:71,10008:240,10151:195,11000:134,11001:15,11002:234,11003:64,11004:11,11005:93,11010:46,11011:106,11020:205,11030:144,11031:133,11032:85,11033:195,11034:79,11035:128,11036:177,11037:130,11038:47,11039:142,11040:132,11041:208,11042:201,11043:193,11044:189,12900:114,12901:254,12902:140,12903:249,12904:77,12905:49,12915:94,12918:139,12919:7,12920:20,42000:227,42001:239,50001:246,50002:181,50003:62,50004:240,50005:152,52000:13,52001:239};

  const SIZES = { B: 1, b: 1, H: 2, h: 2, I: 4, i: 4, f: 4, Q: 8, s: 1 };

  function parseFormat(fmt) {
    const out = [];
    const re = /(\d*)([BbHhIifQs])/g;
    let m, off = 0;
    while ((m = re.exec(fmt))) {
      const n = m[1] ? parseInt(m[1], 10) : 1;
      out.push({ t: m[2], n, off });
      off += SIZES[m[2]] * n;
    }
    return { fields: out, size: off };
  }

  const BY_ID = {};
  const BY_NAME = {};
  for (const name in DEFS) {
    const [id, crc, fmt, names] = DEFS[name];
    const { fields, size } = parseFormat(fmt);
    const def = { name, id, crc, fields, names, size };
    BY_ID[id] = def;
    BY_NAME[name] = def;
  }

  function readOne(dv, t, off) {
    switch (t) {
      case 'B': return dv.getUint8(off);
      case 'b': return dv.getInt8(off);
      case 'H': return dv.getUint16(off, true);
      case 'h': return dv.getInt16(off, true);
      case 'I': return dv.getUint32(off, true);
      case 'i': return dv.getInt32(off, true);
      case 'f': return dv.getFloat32(off, true);
      case 'Q': return Number(dv.getBigUint64(off, true));
    }
  }

  function writeOne(dv, t, off, v) {
    switch (t) {
      case 'B': dv.setUint8(off, v); break;
      case 'b': dv.setInt8(off, v); break;
      case 'H': dv.setUint16(off, v, true); break;
      case 'h': dv.setInt16(off, v, true); break;
      case 'I': dv.setUint32(off, v >>> 0, true); break;
      case 'i': dv.setInt32(off, v, true); break;
      case 'f': dv.setFloat32(off, v, true); break;
      case 'Q': dv.setBigUint64(off, BigInt(Math.round(v)), true); break;
    }
  }

  function decodePayload(def, bytes) {
    // MAVLink 2 trims trailing zero bytes, so pad back to the full length.
    const buf = new Uint8Array(Math.max(def.size, bytes.length));
    buf.set(bytes);
    const dv = new DataView(buf.buffer);
    const msg = { _name: def.name, _id: def.id };
    def.fields.forEach((f, i) => {
      const key = def.names[i];
      if (f.t === 's') {
        let s = '';
        for (let k = 0; k < f.n; k++) {
          const c = buf[f.off + k];
          if (!c) break;
          s += String.fromCharCode(c);
        }
        msg[key] = s;
      } else if (f.n > 1) {
        msg[key] = Array.from({ length: f.n }, (_, k) => readOne(dv, f.t, f.off + k * SIZES[f.t]));
      } else {
        msg[key] = readOne(dv, f.t, f.off);
      }
    });
    return msg;
  }

  function encodePayload(def, values) {
    const buf = new Uint8Array(def.size);
    const dv = new DataView(buf.buffer);
    def.fields.forEach((f, i) => {
      const v = values[def.names[i]];
      if (f.t === 's') {
        const s = String(v || '');
        for (let k = 0; k < Math.min(s.length, f.n); k++) buf[f.off + k] = s.charCodeAt(k) & 0xff;
      } else if (f.n > 1) {
        for (let k = 0; k < f.n; k++) writeOne(dv, f.t, f.off + k * SIZES[f.t], (v && v[k]) || 0);
      } else {
        writeOne(dv, f.t, f.off, v === undefined ? 0 : v);
      }
    });
    return buf;
  }

  // CRC-16/MCRF4XX (X.25) as used by MAVLink.
  function crcAccumulate(b, crc) {
    let tmp = (b ^ (crc & 0xff)) & 0xff;
    tmp = (tmp ^ (tmp << 4)) & 0xff;
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
  }

  function crcCalc(bytes, start, end, extra) {
    let crc = 0xffff;
    for (let i = start; i < end; i++) crc = crcAccumulate(bytes[i], crc);
    return crcAccumulate(extra, crc);
  }

  class Encoder {
    constructor(sysid, compid) {
      this.sysid = sysid;
      this.compid = compid;
      this.seq = 0;
    }
    encode(name, values) {
      const def = BY_NAME[name];
      if (!def) throw new Error('Unknown MAVLink message ' + name);
      let payload = encodePayload(def, values);
      let len = payload.length;
      while (len > 1 && payload[len - 1] === 0) len--;
      payload = payload.subarray(0, len);
      const frame = new Uint8Array(10 + len + 2);
      frame[0] = 0xfd;
      frame[1] = len;
      frame[2] = 0;
      frame[3] = 0;
      frame[4] = this.seq;
      this.seq = (this.seq + 1) & 0xff;
      frame[5] = this.sysid;
      frame[6] = this.compid;
      frame[7] = def.id & 0xff;
      frame[8] = (def.id >> 8) & 0xff;
      frame[9] = (def.id >> 16) & 0xff;
      frame.set(payload, 10);
      const crc = crcCalc(frame, 1, 10 + len, def.crc);
      frame[10 + len] = crc & 0xff;
      frame[11 + len] = crc >> 8;
      return frame;
    }
  }

  // Streaming parser. onMessage(msg, header, rawFrameBytes) for every frame with a valid CRC.
  class Parser {
    constructor(onMessage) {
      this.onMessage = onMessage;
      this.buf = new Uint8Array(0);
      this.stats = { frames: 0, crcErrors: 0, unknown: 0 };
    }
    feed(chunk) {
      const merged = new Uint8Array(this.buf.length + chunk.length);
      merged.set(this.buf);
      merged.set(chunk, this.buf.length);
      let b = merged;
      let i = 0;
      while (i < b.length) {
        const magic = b[i];
        if (magic !== 0xfd && magic !== 0xfe) { i++; continue; }
        if (b.length - i < 3) break;
        const len = b[i + 1];
        let hdrLen, total, msgid, sysid, compid, seq;
        if (magic === 0xfd) {
          hdrLen = 10;
          const signed = (b[i + 2] & 0x01) !== 0;
          total = hdrLen + len + 2 + (signed ? 13 : 0);
          if (b.length - i < total) break;
          seq = b[i + 4]; sysid = b[i + 5]; compid = b[i + 6];
          msgid = b[i + 7] | (b[i + 8] << 8) | (b[i + 9] << 16);
        } else {
          hdrLen = 6;
          total = hdrLen + len + 2;
          if (b.length - i < total) break;
          seq = b[i + 2]; sysid = b[i + 3]; compid = b[i + 4];
          msgid = b[i + 5];
        }
        const def = BY_ID[msgid];
        const extra = def ? def.crc : CRC_EXTRA[msgid];
        if (extra === undefined) { i++; continue; }
        const crcPos = i + hdrLen + len;
        const crc = crcCalc(b, i + 1, crcPos, extra);
        if ((b[crcPos] | (b[crcPos + 1] << 8)) !== crc) {
          this.stats.crcErrors++;
          i++;
          continue;
        }
        this.stats.frames++;
        if (!def) {
          this.stats.unknown++;
          i += total;
          continue;
        }
        const msg = decodePayload(def, b.subarray(i + hdrLen, crcPos));
        const raw = b.slice(i, i + total);
        try {
          this.onMessage(msg, { sysid, compid, seq, msgid, v2: magic === 0xfd }, raw);
        } catch (e) {
          console.error(e);
        }
        i += total;
      }
      this.buf = b.slice(i);
      if (this.buf.length > 4096) this.buf = this.buf.slice(-280);
    }
  }

  // ArduPlane flight modes (custom_mode numbers).
  const PLANE_MODES = {
    0: 'MANUAL', 1: 'CIRCLE', 2: 'STABILIZE', 3: 'TRAINING', 4: 'ACRO', 5: 'FBWA', 6: 'FBWB',
    7: 'CRUISE', 8: 'AUTOTUNE', 10: 'AUTO', 11: 'RTL', 12: 'LOITER', 13: 'TAKEOFF', 14: 'AVOID_ADSB',
    15: 'GUIDED', 17: 'QSTABILIZE', 18: 'QHOVER', 19: 'QLOITER', 20: 'QLAND', 21: 'QRTL',
    22: 'QAUTOTUNE', 23: 'QACRO', 24: 'THERMAL', 25: 'LOITER_ALT_QLAND',
  };
  const MODE_NUMBERS = {};
  for (const k in PLANE_MODES) MODE_NUMBERS[PLANE_MODES[k]] = Number(k);

  const api = { DEFS, BY_ID, BY_NAME, Encoder, Parser, crcCalc, PLANE_MODES, MODE_NUMBERS };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.MAV = api;
})(typeof window !== 'undefined' ? window : globalThis);
