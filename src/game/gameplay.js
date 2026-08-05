// =============================================================================
// gameplay.js — DESCENT run director.  CONTRACT §8.
//
// Owns: the run state machine, the run clock, checkpoint splits, the persisted
// personal best, the best-run ghost (record + replay + render), style scoring,
// the countdown, and crash → respawn with a time penalty.
//
// -----------------------------------------------------------------------------
// CONTRACT-NOTE: `bike.js` already emits `run:crash` at crash onset (postfx
//   subscribes to it — ADDENDUM §C) and its own header asks gameplay not to
//   re-emit. The contract asks gameplay to emit it. Both are satisfied here by a
//   de-duplicator: we listen for `run:crash`, and emit our own (with the contract
//   payload `{ position, severity }`) only if nobody else emitted one within the
//   last third of a second — i.e. within the same crash episode. So exactly one
//   `run:crash` reaches every listener whether or not the physics module sends it.
// CONTRACT-NOTE: respawn is owned here, not by the bike. bike.js self-respawns
//   only if nobody reset it during the tumble (its TUMBLE_MAX is 3.6 s); our
//   crash beat is shorter, so in practice gameplay always drives it. If the bike
//   does get there first we detect it (`bike:respawn`, or `state.crashed`
//   clearing) and close the beat out immediately — the run never stalls.
// CONTRACT-NOTE: additive events emitted here, beyond the five required ones:
//   `run:countdown` { index, count, label }, `run:pause` { paused, state },
//   `run:respawned` { position, checkpointIndex, penalty, crashCount },
//   `run:state` { from, to }, `run:ready` { best }. HUD/menu/audio may use them;
//   nothing depends on them existing.
// CONTRACT-NOTE: the ghost is a *silhouette*, not a second full rider. The bike
//   is a one-off clone of `ctx.bikeModel.group` (rigid hierarchy, materials
//   swapped for one shared translucent material); the rider is a merged
//   procedural capsule figure built here. Cloning `rider.group` is deliberately
//   avoided: it may be a SkinnedMesh whose clone would freeze in bind pose.
// CONTRACT-NOTE: event payload objects are pre-allocated and REUSED (same policy
//   as bike.js) except `run:finish`, which fires once per run and hands over
//   fresh arrays that the menu/HUD are expected to retain.
// CONTRACT-NOTE: the run clock accumulates the clamped `dt` main.js hands every
//   system, NOT wall-clock time. main.js clamps dt to 1/20 s, so on a machine
//   running at 10 fps the simulated world advances at half real time — a
//   wall-sourced timer would then tick at double speed and no two recorded times
//   would be comparable. Accumulated dt is still performance.now()-derived (via
//   THREE.Clock), so the resolution is sub-millisecond, and it is immune to a
//   backgrounded tab where rAF stops but the wall clock does not.
// CONTRACT-NOTE: `ctx.menu` is a stub in the scaffold. To avoid the game booting
//   into a frozen title screen that nothing can dismiss, the 'menu' state
//   auto-starts after a short delay unless a menu declares that it owns the
//   screen (`menu.open === true`, `menu.isOpen === true`, or it called
//   `gameplay.hold()`). `?menu=1` forces the title hold, `?menu=0` skips it.
// =============================================================================

import * as THREE from 'three';
import { clamp, clamp01, lerp } from '../core/rng.js';
import { t as tr } from '../i18n/i18n.js';

// -----------------------------------------------------------------------------
// Tuning
// -----------------------------------------------------------------------------

const COUNTDOWN_BEAT = 0.85;        // s per "3" / "2" / "1"
const COUNTDOWN_GO_HOLD = 0.28;     // s the GO caption sits before the timer runs
const MENU_AUTOSTART = 1.1;         // s before an unclaimed menu starts the run
const CRASH_BEAT = 1.25;            // s of wreckage before we respawn them
const RESPAWN_SETTLE = 0.45;        // s of grace after a respawn (no crash re-arm)

const PENALTY_BASE = 3.0;           // s added on the first crash of a segment
const PENALTY_STEP = 0.75;          // s added per repeat crash in the same segment
const PENALTY_MAX = 6.0;

// Style scoring.
const AIR_MIN = 0.38;               // s before airtime scores at all
const AIR_RATE = 62;                // points per second of air (^AIR_EXP)
const AIR_EXP = 1.25;
const WHIP_MIN = 0.28;              // rad of chassis yaw-off-travel to register
const WHIP_RATE = 210;
const ROT_FULL = 5.6;               // rad — a complete flip
const LAND_CLEAN = 0.72;            // landing quality thresholds
const LAND_PERFECT = 0.90;
const MANUAL_MIN = 0.65;            // s
const MANUAL_RATE = 48;             // points per second
const NEARMISS_RADIUS = 1.75;       // m, horizontal, trunk centre to chassis
const NEARMISS_MIN_SPEED = 8.5;     // m/s
const NEARMISS_COOLDOWN = 0.22;     // s between scored near-misses
const COMBO_GAIN = 0.35;
const COMBO_MAX = 6.0;
const COMBO_HOLD = 3.2;             // s at full value before it starts falling
const COMBO_DECAY = 1.1;            // multiplier units per second

// Ghost recording.
const GHOST_HZ = 30;
const GHOST_DT = 1 / GHOST_HZ;
const GHOST_STRIDE = 20;
const GHOST_CAP0 = 30 * 200;        // 200 s of run before the buffer grows
const GHOST_MAX_SAMPLES = 30 * 900; // 15 min hard cap; a run longer than this is a lost cause
const G = {                         // channel layout
  T: 0, PX: 1, PY: 2, PZ: 3, QX: 4, QY: 5, QZ: 6, QW: 7,
  LEAN: 8, STEER: 9, SPINF: 10, SPINR: 11, FORK: 12, SHOCK: 13,
  FLAGS: 14, SPEED: 15, WHIP: 16, WHEELIE: 17, TRAILT: 18,
};
const F_AIR = 1, F_CRASH = 2, F_BRAKE = 4, F_MANUAL = 8;

// Persistence.
const STORE_VERSION = 1;
const STORE_PREFIX = 'descent.v1';
const GHOST_OPACITY = 0.32;         // base alpha of the ghost silhouette
const GHOST_MAX_CHARS = 1400000;    // ~2.8 MB of localStorage quota (UTF-16). Above this we decimate.

// -----------------------------------------------------------------------------
// Module-scope scratch — nothing in update() allocates.
// -----------------------------------------------------------------------------

const _v3a = new THREE.Vector3();
const _v3b = new THREE.Vector3();
const _v3c = new THREE.Vector3();
const _qa = new THREE.Quaternion();
const _qb = new THREE.Quaternion();
const _UP = new THREE.Vector3(0, 1, 0);
const _ONE = new THREE.Vector3(1, 1, 1);

const num = (v, d) => (typeof v === 'number' && isFinite(v) ? v : d);

// =============================================================================
// Persistence — namespaced, versioned, and paranoid about what comes back out.
// =============================================================================

function storageAvailable() {
  try {
    const k = STORE_PREFIX + '.probe';
    window.localStorage.setItem(k, '1');
    window.localStorage.removeItem(k);
    return true;
  } catch (e) {
    return false;
  }
}

function createStore(seed) {
  const ok = storageAvailable();
  const recordKey = `${STORE_PREFIX}.record.${seed >>> 0}`;
  const ghostKey = `${STORE_PREFIX}.ghost.${seed >>> 0}`;

  function readJson(key) {
    if (!ok) return null;
    let raw = null;
    try { raw = window.localStorage.getItem(key); } catch (e) { return null; }
    if (!raw || typeof raw !== 'string') return null;
    try {
      const obj = JSON.parse(raw);
      return (obj && typeof obj === 'object') ? obj : null;
    } catch (e) {
      // Corrupt entry — bin it so it cannot keep costing us a parse every boot.
      try { window.localStorage.removeItem(key); } catch (e2) { /* ignore */ }
      return null;
    }
  }

  function writeJson(key, obj) {
    if (!ok) return false;
    try {
      window.localStorage.setItem(key, JSON.stringify(obj));
      return true;
    } catch (e) {
      return false;   // quota, private mode, whatever — never fatal
    }
  }

  return {
    available: ok,
    loadRecord() {
      const o = readJson(recordKey);
      if (!o || o.v !== STORE_VERSION) return null;
      if (!isFinite(o.time) || o.time <= 0 || o.time > 36000) return null;
      const splits = Array.isArray(o.splits)
        ? o.splits.filter((s) => typeof s === 'number' && isFinite(s) && s >= 0)
        : [];
      return {
        time: o.time,
        splits,
        score: num(o.score, 0),
        crashes: num(o.crashes, 0),
        date: typeof o.date === 'string' ? o.date : '',
        seed: seed >>> 0,
      };
    },
    saveRecord(rec) {
      return writeJson(recordKey, {
        v: STORE_VERSION, seed: seed >>> 0,
        time: rec.time, splits: rec.splits, score: rec.score,
        crashes: rec.crashes, date: new Date().toISOString(),
      });
    },
    loadGhost() { return readJson(ghostKey); },
    saveGhost(payload) { return writeJson(ghostKey, payload); },
    clearGhost() {
      if (!ok) return;
      try { window.localStorage.removeItem(ghostKey); } catch (e) { /* ignore */ }
    },
    clearAll() {
      if (!ok) return;
      try {
        window.localStorage.removeItem(recordKey);
        window.localStorage.removeItem(ghostKey);
      } catch (e) { /* ignore */ }
    },
  };
}

// =============================================================================
// Ghost recording buffer + codec
// =============================================================================

function createRecording(capacity) {
  return {
    data: new Float32Array(Math.max(64, capacity) * GHOST_STRIDE),
    count: 0,
    duration: 0,
    cursor: 0,          // playback hint, monotonic
    deltaCursor: 0,     // separate hint for the trailT→time lookup
    meta: null,
  };
}

function recReset(rec) {
  rec.count = 0;
  rec.duration = 0;
  rec.cursor = 0;
  rec.deltaCursor = 0;
}

/** Returns the offset of a new sample slot, growing the buffer if needed. */
function recAlloc(rec) {
  if (rec.count >= GHOST_MAX_SAMPLES) return -1;
  const need = (rec.count + 1) * GHOST_STRIDE;
  if (need > rec.data.length) {
    const grown = new Float32Array(Math.min(
      GHOST_MAX_SAMPLES * GHOST_STRIDE,
      Math.max(need, Math.ceil(rec.data.length * 1.7))));
    grown.set(rec.data);
    rec.data = grown;
  }
  const off = rec.count * GHOST_STRIDE;
  rec.count++;
  return off;
}

/** Copy a recording into another (used to promote the live run to "best"). */
function recCopyInto(src, dst) {
  const need = src.count * GHOST_STRIDE;
  if (dst.data.length < need) dst.data = new Float32Array(need);
  dst.data.set(src.data.subarray(0, need));
  dst.count = src.count;
  dst.duration = src.duration;
  dst.cursor = 0;
  dst.deltaCursor = 0;
  dst.meta = src.meta;
}

/**
 * Interpolated sample at run-time `t`. Advances rec.cursor, which makes normal
 * forward playback O(1); a backwards seek costs one binary search.
 * Writes into `out` and returns it. No allocation.
 */
function recSample(rec, t, out) {
  out.valid = false;
  const n = rec.count;
  if (n === 0) return out;
  const d = rec.data;
  if (t <= d[G.T]) {
    readSample(d, 0, out);
    out.valid = true;
    out.finished = false;
    return out;
  }
  const lastOff = (n - 1) * GHOST_STRIDE;
  if (t >= d[lastOff + G.T]) {
    readSample(d, n - 1, out);
    out.valid = true;
    out.finished = true;
    return out;
  }
  out.finished = false;

  let i = rec.cursor;
  if (i < 0 || i >= n - 1 || d[i * GHOST_STRIDE + G.T] > t) {
    // Seek: binary search rather than rewinding one sample at a time.
    let lo = 0, hi = n - 1;
    while (lo + 1 < hi) {
      const mid = (lo + hi) >> 1;
      if (d[mid * GHOST_STRIDE + G.T] <= t) lo = mid; else hi = mid;
    }
    i = lo;
  } else {
    while (i < n - 2 && d[(i + 1) * GHOST_STRIDE + G.T] <= t) i++;
  }
  rec.cursor = i;

  const a = i * GHOST_STRIDE, b = (i + 1) * GHOST_STRIDE;
  const ta = d[a + G.T], tb = d[b + G.T];
  const f = tb > ta ? clamp01((t - ta) / (tb - ta)) : 0;

  out.position.set(
    lerp(d[a + G.PX], d[b + G.PX], f),
    lerp(d[a + G.PY], d[b + G.PY], f),
    lerp(d[a + G.PZ], d[b + G.PZ], f));
  _qa.set(d[a + G.QX], d[a + G.QY], d[a + G.QZ], d[a + G.QW]);
  _qb.set(d[b + G.QX], d[b + G.QY], d[b + G.QZ], d[b + G.QW]);
  out.quaternion.copy(_qa).slerp(_qb, f);
  out.lean = lerp(d[a + G.LEAN], d[b + G.LEAN], f);
  out.steer = lerp(d[a + G.STEER], d[b + G.STEER], f);
  out.spinF = lerp(d[a + G.SPINF], d[b + G.SPINF], f);
  out.spinR = lerp(d[a + G.SPINR], d[b + G.SPINR], f);
  out.fork = lerp(d[a + G.FORK], d[b + G.FORK], f);
  out.shock = lerp(d[a + G.SHOCK], d[b + G.SHOCK], f);
  out.speed = lerp(d[a + G.SPEED], d[b + G.SPEED], f);
  out.whip = lerp(d[a + G.WHIP], d[b + G.WHIP], f);
  out.wheelie = lerp(d[a + G.WHEELIE], d[b + G.WHEELIE], f);
  out.trailT = lerp(d[a + G.TRAILT], d[b + G.TRAILT], f);
  const flags = d[(f < 0.5 ? a : b) + G.FLAGS] | 0;
  out.airborne = (flags & F_AIR) !== 0;
  out.crashed = (flags & F_CRASH) !== 0;
  out.time = t;
  out.valid = true;
  return out;
}

function readSample(d, index, out) {
  const o = index * GHOST_STRIDE;
  out.position.set(d[o + G.PX], d[o + G.PY], d[o + G.PZ]);
  out.quaternion.set(d[o + G.QX], d[o + G.QY], d[o + G.QZ], d[o + G.QW]);
  out.lean = d[o + G.LEAN];
  out.steer = d[o + G.STEER];
  out.spinF = d[o + G.SPINF];
  out.spinR = d[o + G.SPINR];
  out.fork = d[o + G.FORK];
  out.shock = d[o + G.SHOCK];
  out.speed = d[o + G.SPEED];
  out.whip = d[o + G.WHIP];
  out.wheelie = d[o + G.WHEELIE];
  out.trailT = d[o + G.TRAILT];
  const flags = d[o + G.FLAGS] | 0;
  out.airborne = (flags & F_AIR) !== 0;
  out.crashed = (flags & F_CRASH) !== 0;
  out.time = d[o + G.T];
}

/**
 * Run time at which the ghost reached trail progress `tt`. Monotonic cursor,
 * so the per-frame cost is a couple of comparisons. Returns NaN if unknown.
 */
function recTimeAtProgress(rec, tt) {
  const n = rec.count;
  if (n < 2) return NaN;
  const d = rec.data;
  let i = rec.deltaCursor;
  if (i < 0 || i >= n - 1 || d[i * GHOST_STRIDE + G.TRAILT] > tt) {
    let lo = 0, hi = n - 1;
    if (d[G.TRAILT] > tt) return d[G.T];
    while (lo + 1 < hi) {
      const mid = (lo + hi) >> 1;
      if (d[mid * GHOST_STRIDE + G.TRAILT] <= tt) lo = mid; else hi = mid;
    }
    i = lo;
  } else {
    while (i < n - 2 && d[(i + 1) * GHOST_STRIDE + G.TRAILT] <= tt) i++;
  }
  rec.deltaCursor = i;
  const a = i * GHOST_STRIDE, b = (i + 1) * GHOST_STRIDE;
  const pa = d[a + G.TRAILT], pb = d[b + G.TRAILT];
  if (pb <= pa) return d[a + G.T];
  const f = clamp01((tt - pa) / (pb - pa));
  return lerp(d[a + G.T], d[b + G.T], f);
}

// ---- base64 <-> bytes -------------------------------------------------------

function bytesToBase64(u8) {
  let s = '';
  const CH = 0x2000;   // 8 k arguments per apply() — safely under the stack limit
  for (let i = 0; i < u8.length; i += CH) {
    s += String.fromCharCode.apply(null, u8.subarray(i, Math.min(i + CH, u8.length)));
  }
  return btoa(s);
}

function base64ToBytes(b64) {
  const bin = atob(b64);
  const u8 = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return u8;
}

/**
 * Quantise to uint16 with a per-channel min/max. Generic — no hand-tuned scales
 * to get wrong — and on a 2.6 km course it lands ~2 cm of position precision,
 * which is well under what you can see on a translucent ghost.
 */
function encodeRecording(rec, decimate) {
  const step = Math.max(1, decimate | 0);
  const n = Math.ceil(rec.count / step);
  if (n < 2) return null;
  const src = rec.data;
  const lo = new Float64Array(GHOST_STRIDE).fill(Infinity);
  const hi = new Float64Array(GHOST_STRIDE).fill(-Infinity);
  for (let s = 0, i = 0; i < rec.count; i += step, s++) {
    const o = i * GHOST_STRIDE;
    for (let c = 0; c < GHOST_STRIDE; c++) {
      const v = src[o + c];
      if (!isFinite(v)) continue;
      if (v < lo[c]) lo[c] = v;
      if (v > hi[c]) hi[c] = v;
    }
  }
  const q = new Uint16Array(n * GHOST_STRIDE);
  for (let s = 0, i = 0; s < n; s++, i += step) {
    const o = Math.min(i, rec.count - 1) * GHOST_STRIDE;
    const p = s * GHOST_STRIDE;
    for (let c = 0; c < GHOST_STRIDE; c++) {
      const range = hi[c] - lo[c];
      const v = src[o + c];
      q[p + c] = range > 1e-9 && isFinite(v)
        ? Math.round(clamp01((v - lo[c]) / range) * 65535)
        : 0;
    }
  }
  return {
    v: STORE_VERSION,
    n, stride: GHOST_STRIDE, hz: GHOST_HZ / step,
    dur: rec.duration,
    lo: Array.from(lo, (x) => (isFinite(x) ? x : 0)),
    hi: Array.from(hi, (x) => (isFinite(x) ? x : 0)),
    b64: bytesToBase64(new Uint8Array(q.buffer, q.byteOffset, q.byteLength)),
  };
}

function decodeRecording(payload) {
  if (!payload || payload.v !== STORE_VERSION) return null;
  const n = payload.n | 0;
  const stride = payload.stride | 0;
  if (n < 2 || stride !== GHOST_STRIDE || n > GHOST_MAX_SAMPLES) return null;
  if (!Array.isArray(payload.lo) || !Array.isArray(payload.hi)) return null;
  if (payload.lo.length !== stride || payload.hi.length !== stride) return null;
  if (typeof payload.b64 !== 'string' || payload.b64.length < 8) return null;
  let u8;
  try { u8 = base64ToBytes(payload.b64); } catch (e) { return null; }
  if (u8.length !== n * stride * 2) return null;
  const q = new Uint16Array(u8.buffer, 0, n * stride);
  const rec = createRecording(n);
  rec.count = n;
  const d = rec.data;
  for (let s = 0; s < n; s++) {
    const o = s * stride;
    for (let c = 0; c < stride; c++) {
      const l = num(payload.lo[c], 0), h = num(payload.hi[c], 0);
      d[o + c] = h > l ? l + (q[o + c] / 65535) * (h - l) : l;
    }
  }
  // A ghost whose time channel is not monotonic would break playback outright.
  for (let s = 1; s < n; s++) {
    if (d[s * stride + G.T] < d[(s - 1) * stride + G.T]) return null;
  }
  rec.duration = num(payload.dur, d[(n - 1) * stride + G.T]);
  // Re-normalise the quaternion channel: quantisation pulls it off unit length.
  for (let s = 0; s < n; s++) {
    const o = s * stride;
    const x = d[o + G.QX], y = d[o + G.QY], z = d[o + G.QZ], w = d[o + G.QW];
    const len = Math.sqrt(x * x + y * y + z * z + w * w);
    if (len > 1e-6) {
      d[o + G.QX] = x / len; d[o + G.QY] = y / len;
      d[o + G.QZ] = z / len; d[o + G.QW] = w / len;
    } else {
      d[o + G.QW] = 1;
    }
  }
  return rec;
}

// =============================================================================
// Ghost visuals — a translucent silhouette, built once, lazily.
// =============================================================================

/** Merge a list of { geo, matrix } into one non-indexed position+normal buffer. */
function mergeGeoms(list) {
  const prepped = [];
  let total = 0;
  for (const item of list) {
    if (!item || !item.geo) continue;
    const g = item.geo.index ? item.geo.toNonIndexed() : item.geo.clone();
    if (item.matrix) g.applyMatrix4(item.matrix);
    prepped.push(g);
    total += g.attributes.position.count;
    item.geo.dispose();
  }
  if (!total) return null;
  const pos = new Float32Array(total * 3);
  const nrm = new Float32Array(total * 3);
  let off = 0;
  for (const g of prepped) {
    pos.set(g.attributes.position.array, off * 3);
    if (g.attributes.normal) nrm.set(g.attributes.normal.array, off * 3);
    off += g.attributes.position.count;
    g.dispose();
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  out.setAttribute('normal', new THREE.BufferAttribute(nrm, 3));
  out.computeBoundingSphere();
  return out;
}

function capsuleBetween(ax, ay, az, bx, by, bz, r) {
  _v3a.set(ax, ay, az);
  _v3b.set(bx, by, bz);
  const len = _v3a.distanceTo(_v3b);
  const geo = new THREE.CapsuleGeometry(r, Math.max(0.02, len - r * 2), 3, 8);
  _v3c.addVectors(_v3a, _v3b).multiplyScalar(0.5);
  _v3b.sub(_v3a).normalize();
  _qa.setFromUnitVectors(_UP, _v3b);
  const m = new THREE.Matrix4().compose(_v3c, _qa, _ONE);
  return { geo, matrix: m };
}

/**
 * A rider in the attack position, in bike-local space (origin = wheelbase
 * midpoint at axle height, forward = −Z, up = +Y). Read as a silhouette at
 * 30 % opacity, which is all a ghost needs to be.
 */
function buildRiderSilhouette() {
  const parts = [];
  // Feet on the pedals, knees bent, hips back — standing, weight neutral.
  parts.push(capsuleBetween(-0.10, 0.02, 0.06, -0.15, 0.44, 0.00, 0.075));   // L shin
  parts.push(capsuleBetween(0.10, 0.02, 0.06, 0.15, 0.44, 0.00, 0.075));     // R shin
  parts.push(capsuleBetween(-0.15, 0.44, 0.00, -0.11, 0.76, 0.12, 0.085));   // L thigh
  parts.push(capsuleBetween(0.15, 0.44, 0.00, 0.11, 0.76, 0.12, 0.085));     // R thigh
  parts.push(capsuleBetween(0.00, 0.74, 0.12, 0.00, 1.13, -0.20, 0.135));    // torso
  parts.push(capsuleBetween(-0.17, 1.10, -0.20, -0.33, 1.03, -0.44, 0.055)); // L upper arm
  parts.push(capsuleBetween(0.17, 1.10, -0.20, 0.33, 1.03, -0.44, 0.055));   // R upper arm
  parts.push(capsuleBetween(-0.33, 1.03, -0.44, -0.36, 0.95, -0.66, 0.048)); // L forearm
  parts.push(capsuleBetween(0.33, 1.03, -0.44, 0.36, 0.95, -0.66, 0.048));   // R forearm
  // Helmet, pushed forward over the bars.
  const head = new THREE.SphereGeometry(0.125, 12, 9);
  parts.push({ geo: head, matrix: new THREE.Matrix4().makeTranslation(0, 1.27, -0.29) });
  return mergeGeoms(parts);
}

/** Minimal bike stand-in, used only when ctx.bikeModel is unavailable. */
function buildFallbackBike(wheelR, wheelbase) {
  const parts = [];
  const half = wheelbase * 0.5;
  for (const z of [-half, half]) {
    const w = new THREE.TorusGeometry(wheelR, 0.055, 6, 20);
    parts.push({ geo: w, matrix: new THREE.Matrix4().compose(
      _v3a.set(0, 0, z), _qa.setFromAxisAngle(_UP, Math.PI / 2), _ONE) });
  }
  parts.push(capsuleBetween(0, 0.0, half, 0, 0.30, 0.10, 0.035));    // down tube-ish
  parts.push(capsuleBetween(0, 0.30, 0.10, 0, 0.05, -half, 0.033));  // top tube to head
  parts.push(capsuleBetween(0, 0.05, -half, 0, 0.55, -half - 0.08, 0.04)); // fork/steerer
  parts.push(capsuleBetween(-0.38, 0.58, -half - 0.06, 0.38, 0.58, -half - 0.06, 0.024)); // bars
  return mergeGeoms(parts);
}

// =============================================================================
// createGameplay
// =============================================================================

export function createGameplay(ctx) {
  const seed = (ctx && ctx.seed) >>> 0;
  const store = createStore(seed);
  const params = (typeof location !== 'undefined' && location.search)
    ? new URLSearchParams(location.search) : null;
  const menuParam = params ? params.get('menu') : null;

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------

  let state = 'menu';
  let prevState = 'menu';
  let stateTime = 0;                 // s in the current state
  let holdRequested = menuParam === '1';
  const forceNoMenu = menuParam === '0';

  // Run clock. It accumulates the SAME clamped dt the physics integrates rather
  // than sampling the wall clock. That matters: main.js clamps dt to 1/20 s, so
  // on a machine dropping to 10 fps the world advances half as fast as the wall
  // clock, and a wall-sourced timer would run at double speed and make every
  // recorded time frame-rate dependent. dt itself comes from THREE.Clock, i.e.
  // performance.now(), so this is still sub-millisecond resolution — and it is
  // immune to a backgrounded tab, where rAF stops but the wall clock does not.
  let clockRunning = false;
  let runTime = 0;
  let penaltyTotal = 0;

  let countdownIndex = -1;           // 3,2,1 → 0,1,2 ; GO → 3
  let goHold = 0;

  let checkpointsTaken = 0;
  let nextCheckpoint = 0;
  let lastCheckpointIndex = -1;
  let crashCount = 0;
  let crashesThisSegment = 0;
  let crashBeat = 0;
  let respawnGrace = 0;
  let finished = false;
  let isBest = false;
  let manualCameraOverride = false;

  const splits = [];                 // [{ index, t, time, delta, best }]
  let best = store.loadRecord();
  let liveDelta = NaN;
  let ghostDelta = NaN;

  // ---------------------------------------------------------------------------
  // Trail-derived constants (re-read each run; trail may build after us in a
  // degraded boot).
  // ---------------------------------------------------------------------------

  const checkpoints = [];            // shallow copies: { t, index, position }
  let finishT = 0.985;

  function syncTrail() {
    checkpoints.length = 0;
    const trail = ctx.trail;
    if (trail && Array.isArray(trail.checkpoints)) {
      for (let i = 0; i < trail.checkpoints.length; i++) {
        const cp = trail.checkpoints[i];
        if (!cp || !isFinite(cp.t)) continue;
        checkpoints.push(cp);
      }
      checkpoints.sort((a, b) => a.t - b.t);
    }
    finishT = num(trail && trail.finishT, 0.985);
  }
  syncTrail();

  // ---------------------------------------------------------------------------
  // Events — pre-allocated payloads (bike.js policy: listeners read synchronously)
  // ---------------------------------------------------------------------------

  const evCheckpoint = { index: 0, time: 0, delta: 0, best: 0, position: new THREE.Vector3() };
  const evCrash = { position: new THREE.Vector3(), severity: 0, reason: '', count: 0 };
  const evTrick = { name: '', points: 0, multiplier: 1, kind: '', total: 0 };
  const evCountdown = { index: 0, count: 0, label: '' };
  const evPause = { paused: false, state: 'paused' };
  const evRespawned = { position: new THREE.Vector3(), checkpointIndex: -1, penalty: 0, crashCount: 0 };
  const evStateChange = { from: '', to: '' };

  const unsubs = [];
  function on(name, fn) {
    if (!ctx.events || !ctx.events.on) return;
    const off = ctx.events.on(name, fn);
    unsubs.push(typeof off === 'function' ? off : () => ctx.events.off(name, fn));
  }
  function emit(name, payload) {
    if (ctx.events && ctx.events.emit) ctx.events.emit(name, payload);
  }

  // De-duplicator for run:crash (see the CONTRACT-NOTE at the top). bike.js emits
  // `bike:crash` FIRST and `run:crash` immediately after, both inside bike.update()
  // — i.e. both land before gameplay.update() runs. So the decision to emit our own
  // cannot be taken in the crash handler (we would always beat bike's run:crash to
  // it); it is deferred to the end of our update, by which point we know. The test
  // is a short time window rather than frame equality, so it still holds if the
  // other emitter fires from a different point in the frame.
  let externalCrashTime = -1e9;
  let emittingOwnCrash = false;
  let pendingCrashEmit = false;

  // ---------------------------------------------------------------------------
  // Clock
  // ---------------------------------------------------------------------------

  function tickClock(dt) {
    if (!clockRunning) return;
    const step = num(dt, 0);
    if (step > 0) runTime += step;
  }
  function setClock(on_) {
    clockRunning = on_;
  }

  // ---------------------------------------------------------------------------
  // State machine
  // ---------------------------------------------------------------------------

  function setState(next) {
    if (next === state) return;
    prevState = state;
    state = next;
    stateTime = 0;
    api.state = next;
    evStateChange.from = prevState;
    evStateChange.to = next;
    emit('run:state', evStateChange);
    // The clock runs while riding and while wrecked — crashing costs real time
    // on top of the explicit penalty; that is the point of it.
    setClock(next === 'running' || next === 'crashed');
    applyCameraMode(next);
  }

  function applyCameraMode(s) {
    const cam = ctx.chaseCamera;
    if (!cam || typeof cam.setMode !== 'function' || manualCameraOverride) return;
    try {
      if (s === 'menu') cam.setMode('cinematic');
      else if (s === 'finished') cam.setMode('chaseFar');
      else if (s === 'running' || s === 'countdown' || s === 'crashed') cam.setMode('chase');
    } catch (e) { /* camera module is someone else's problem */ }
  }

  function menuOwnsScreen() {
    if (forceNoMenu) return false;
    if (holdRequested) return true;
    const m = ctx.menu;
    if (!m) return false;
    if (typeof m.isOpen === 'boolean') return m.isOpen;
    if (typeof m.open === 'boolean') return m.open;
    return false;
  }

  // ---------------------------------------------------------------------------
  // Run lifecycle
  // ---------------------------------------------------------------------------

  function placeAtStart() {
    const bike = ctx.bike;
    if (!bike || typeof bike.reset !== 'function') return;
    const t = ctx.trail && ctx.trail.startTransform;
    try { bike.reset(t || null); } catch (e) { /* bike is mid-build */ }
  }

  function resetRunData() {
    syncTrail();
    runTime = 0;
    penaltyTotal = 0;
    splits.length = 0;
    checkpointsTaken = 0;
    nextCheckpoint = 0;
    lastCheckpointIndex = -1;
    crashCount = 0;
    crashesThisSegment = 0;
    crashBeat = 0;
    respawnGrace = 0;
    finished = false;
    isBest = false;
    pendingCrashEmit = false;
    liveDelta = NaN;
    ghostDelta = NaN;
    api.score = 0;
    api.checkpointIndex = 0;
    style.reset();
    recReset(liveRec);
    nextGhostSample = 0;
    if (bestRec) { bestRec.cursor = 0; bestRec.deltaCursor = 0; }
    ghost.playing = false;
    ghost.time = 0;
    ghost.finished = false;
    nmScoredCount = 0;
    nmScoredHead = 0;
    nmCount = 0;
    nmTimer = 0;
    nmLastX = 1e9; nmLastZ = 1e9;
  }

  /** Begin a run: countdown on the gate, then GO. */
  function start() {
    resetRunData();
    placeAtStart();
    countdownIndex = -1;
    goHold = 0;
    setState('countdown');
    return api;
  }

  function restart() {
    // Wherever we were — mid-air, wrecked, finished, paused — this always lands
    // back on the gate with a fresh countdown.
    start();
    return api;
  }

  function pause(force) {
    if (state === 'menu') return api;
    const wantPaused = force === undefined ? state !== 'paused' : !!force;
    if (wantPaused && state !== 'paused') {
      setState('paused');
      evPause.paused = true;
      evPause.state = prevState;
      emit('run:pause', evPause);
    } else if (!wantPaused && state === 'paused') {
      const back = (prevState === 'paused' || !prevState) ? 'running' : prevState;
      setState(back);
      evPause.paused = false;
      evPause.state = back;
      emit('run:pause', evPause);
    }
    return api;
  }

  function beginRunning() {
    setState('running');
    api.countdown = 0;
    api.countdownLabel = '';
    emit('run:start', { seed, best, checkpoints: checkpoints.length });
  }

  function finishRun() {
    if (finished) return;
    finished = true;
    setState('finished');

    const splitTimes = splits.map((s) => s.time);
    const requiredCps = Math.max(0, checkpoints.length - 1);
    const valid = checkpointsTaken >= requiredCps;
    isBest = valid && (!best || runTime < best.time - 1e-4);

    if (isBest) {
      const record = {
        time: runTime, splits: splitTimes, score: api.score,
        crashes: crashCount, date: new Date().toISOString(), seed,
      };
      best = record;
      store.saveRecord(record);
      promoteGhost();
    }
    api.best = best;

    emit('run:finish', {
      time: runTime,
      splits: splits.map((s) => ({ index: s.index, time: s.time, delta: s.delta, best: s.best })),
      splitTimes,
      isBest,
      best,
      score: api.score,
      crashes: crashCount,
      penalty: penaltyTotal,
      valid,
    });
  }

  // ---------------------------------------------------------------------------
  // Checkpoints
  // ---------------------------------------------------------------------------

  function bestSplit(i) {
    if (!best || !best.splits || i >= best.splits.length) return NaN;
    const v = best.splits[i];
    return (typeof v === 'number' && isFinite(v)) ? v : NaN;
  }

  function takeCheckpoint(i) {
    const cp = checkpoints[i];
    const pb = bestSplit(i);
    const delta = isFinite(pb) ? runTime - pb : NaN;
    const entry = { index: i, t: cp.t, time: runTime, delta, best: pb };
    splits.push(entry);
    checkpointsTaken++;
    lastCheckpointIndex = i;
    crashesThisSegment = 0;
    api.checkpointIndex = i + 1;
    api.lastSplit = entry;

    evCheckpoint.index = i;
    evCheckpoint.time = runTime;
    evCheckpoint.delta = isFinite(delta) ? delta : 0;
    evCheckpoint.best = isFinite(pb) ? pb : 0;
    if (cp.position) evCheckpoint.position.copy(cp.position);
    emit('run:checkpoint', evCheckpoint);
  }

  /** Live pace vs the PB, interpolated between checkpoints by trail progress. */
  function updateLiveDelta(tt) {
    if (!best || !best.splits || !best.splits.length || !checkpoints.length) {
      liveDelta = NaN;
      return;
    }
    // Find the bracketing checkpoints for the current progress.
    let i = 0;
    while (i < checkpoints.length && checkpoints[i].t <= tt) i++;
    const t1 = i < checkpoints.length ? checkpoints[i].t : finishT;
    const time1 = i < checkpoints.length ? bestSplit(i) : best.time;
    const t0 = i > 0 ? checkpoints[i - 1].t : 0;
    const time0 = i > 0 ? bestSplit(i - 1) : 0;
    if (!isFinite(time1) || !isFinite(time0) || t1 <= t0) { liveDelta = NaN; return; }
    const f = clamp01((tt - t0) / (t1 - t0));
    liveDelta = runTime - lerp(time0, time1, f);
  }

  // ---------------------------------------------------------------------------
  // Crash / respawn
  // ---------------------------------------------------------------------------

  function onCrash(severity, position, reason) {
    if (state !== 'running') return;              // already wrecked, or not riding
    if (respawnGrace > 0) return;                 // do not re-arm during the drop-in
    crashCount++;
    crashesThisSegment++;
    crashBeat = 0;
    style.onCrash();

    if (position) evCrash.position.copy(position);
    else if (ctx.bike && ctx.bike.state) evCrash.position.copy(ctx.bike.state.position);
    evCrash.severity = clamp01(num(severity, 0.6));
    evCrash.reason = reason || '';
    evCrash.count = crashCount;

    setState('crashed');
    pendingCrashEmit = true;      // flushed at the end of update(), if nobody beat us
  }

  function flushCrashEmit() {
    if (!pendingCrashEmit) return;
    pendingCrashEmit = false;
    // Anything within a third of a second is the same crash episode.
    if (num(ctx.time, 0) - externalCrashTime < 0.35) return;   // already emitted
    emittingOwnCrash = true;
    emit('run:crash', evCrash);
    emittingOwnCrash = false;
  }

  function doRespawn() {
    const bike = ctx.bike;
    const penalty = Math.min(PENALTY_MAX, PENALTY_BASE + PENALTY_STEP * (crashesThisSegment - 1));
    penaltyTotal += penalty;
    runTime += penalty;

    const fallbackTarget = () => (
      (lastCheckpointIndex >= 0 && checkpoints[lastCheckpointIndex])
        ? checkpoints[lastCheckpointIndex]
        : (ctx.trail && ctx.trail.startTransform) || null);

    let idx = lastCheckpointIndex;
    if (bike) {
      try {
        if (typeof bike.respawn === 'function') {
          // bike.respawn() picks the last checkpoint it passed and gives a rolling
          // entry speed — much kinder than a dead stop on a 20 % chute.
          bike.respawn();
          const bs = bike.state;
          if (bs && typeof bs.checkpointIndex === 'number' && bs.checkpointIndex >= 0) {
            idx = bs.checkpointIndex;
          }
        } else if (typeof bike.reset === 'function') {
          bike.reset(fallbackTarget());
        }
      } catch (e) { /* keep the run alive regardless */ }
      // Belt and braces: if the bike is somehow still down, hard-reset it. Without
      // this the run could return to 'running' around an un-rideable bike.
      if (bike.state && bike.state.crashed && typeof bike.reset === 'function') {
        try { bike.reset(fallbackTarget()); } catch (e) { /* ignore */ }
      }
    }

    respawnGrace = RESPAWN_SETTLE;
    crashBeat = 0;
    style.reset(true);
    if (ctx.bike && ctx.bike.state) evRespawned.position.copy(ctx.bike.state.position);
    evRespawned.checkpointIndex = idx;
    evRespawned.penalty = penalty;
    evRespawned.crashCount = crashCount;
    api.penalty = penaltyTotal;
    setState('running');
    emit('run:respawned', evRespawned);
  }

  // ---------------------------------------------------------------------------
  // Style scoring
  // ---------------------------------------------------------------------------

  const style = (() => {
    let combo = 1;
    let comboTimer = 0;
    let wasAir = false;
    let airStart = 0;
    let maxWhip = 0;
    let maxRot = 0;
    let peakHeight = 0;
    let manualTime = 0;
    let manualKind = 0;       // 1 = manual, 2 = endo
    let nearMissCooldown = 0;
    let lastTrickName = '';
    let lastTrickPoints = 0;
    let lastTrickAt = -99;
    let tricks = 0;
    let airTotal = 0;

    // Landing data arrives via bike's `bike:landed` event, which is emitted
    // inside the physics step — cache it and consume on the airborne falling edge.
    let pendingLanding = null;
    const landingCache = { airTime: 0, quality: 1, vNormal: 0, whip: 0, cased: false, frame: -1 };

    function award(name, raw, kind) {
      if (!(raw > 0)) return 0;
      const pts = Math.round(raw * combo);
      api.score += pts;
      tricks++;
      combo = Math.min(COMBO_MAX, combo + COMBO_GAIN);
      comboTimer = COMBO_HOLD;
      lastTrickName = name;
      lastTrickPoints = pts;
      lastTrickAt = ctx.time;
      evTrick.name = name;
      evTrick.points = pts;
      evTrick.multiplier = combo;
      evTrick.kind = kind || '';
      evTrick.total = api.score;
      emit('trick:landed', evTrick);
      return pts;
    }

    // Composite name for a jump: rotation + whip + air + landing, joined.
    const nameParts = ['', '', '', ''];

    function landAir(airTime, quality, cased, landY) {
      let raw = 0;
      let np = 0;
      nameParts[0] = nameParts[1] = nameParts[2] = nameParts[3] = '';

      const rot = Math.abs(maxRot);
      if (rot >= ROT_FULL && !cased) {
        nameParts[np++] = maxRot > 0 ? tr('trick.frontflip') : tr('trick.backflip');
        raw += 460 * Math.floor(rot / ROT_FULL);
      } else if (rot >= ROT_FULL * 0.55) {
        nameParts[np++] = tr('trick.suicideNoHander');   // half rotation held and pulled back
        raw += 120;
      }

      if (maxWhip >= WHIP_MIN) {
        nameParts[np++] = maxWhip > 0.95 ? tr('trick.bigWhip') : (maxWhip > 0.55 ? tr('trick.whip') : tr('trick.scrub'));
        raw += WHIP_RATE * (maxWhip - WHIP_MIN * 0.5);
      }

      if (airTime >= AIR_MIN) {
        const airPts = AIR_RATE * Math.pow(airTime, AIR_EXP);
        raw += airPts;
        airTotal += airTime;
        if (airTime > 1.8) nameParts[np++] = tr('trick.hugeAir');
        else if (airTime > 1.0 && np === 0) nameParts[np++] = tr('trick.bigAir');
        else if (np === 0) nameParts[np++] = tr('trick.air');
      }

      // Vertical drop from the top of the arc to the landing. A 4 m huck off a
      // cliff band earns very little airtime but is the gutsiest thing on the
      // mountain, so it is scored on its own axis.
      const drop = peakHeight - num(landY, peakHeight);
      if (drop > 2.5) {
        raw += 22 * (drop - 2.0);
        if (np === 0 || drop > 6) nameParts[np++] = drop > 6 ? tr('trick.huck') : tr('trick.drop');
      }

      if (cased) {
        // A case kills the multiplier and the points for the whole jump.
        combo = Math.max(1, 1 + (combo - 1) * 0.35);
        comboTimer = COMBO_HOLD * 0.4;
        if (raw > 0) {
          evTrick.name = 'Cased';
          evTrick.points = 0;
          evTrick.multiplier = combo;
          evTrick.kind = 'case';
          evTrick.total = api.score;
          emit('trick:landed', evTrick);
        }
        return;
      }

      if (raw <= 0) return;

      if (quality >= LAND_PERFECT) { nameParts[np++] = tr('trick.perfectLanding'); raw += 85; }
      else if (quality >= LAND_CLEAN) { nameParts[np++] = tr('trick.cleanLanding'); raw += 40; }

      let name = nameParts[0];
      for (let i = 1; i < np; i++) name += ' + ' + nameParts[i];
      award(name, raw, 'air');
    }

    return {
      get combo() { return combo; },
      get comboTimer() { return comboTimer; },
      get tricks() { return tricks; },
      get airTotal() { return airTotal; },
      get lastTrickName() { return lastTrickName; },
      get lastTrickPoints() { return lastTrickPoints; },
      get lastTrickAt() { return lastTrickAt; },
      get airborneTime() { return wasAir ? Math.max(0, ctx.time - airStart) : 0; },
      get meter() { return clamp01((combo - 1) / (COMBO_MAX - 1)); },

      noteLanding(e) {
        landingCache.airTime = num(e && e.airTime, 0);
        landingCache.quality = clamp01(num(e && e.quality, 0.7));
        landingCache.vNormal = num(e && e.vNormal, 0);
        landingCache.whip = Math.abs(num(e && e.whip, 0));
        landingCache.cased = !!(e && e.cased);
        landingCache.frame = ctx.frame;
        pendingLanding = landingCache;
      },

      onCrash() {
        combo = 1;
        comboTimer = 0;
        wasAir = false;
        maxWhip = 0;
        maxRot = 0;
        manualTime = 0;
        manualKind = 0;
        pendingLanding = null;
      },

      reset(keepScore) {
        combo = 1; comboTimer = 0;
        wasAir = false; airStart = 0; maxWhip = 0; maxRot = 0; peakHeight = 0;
        manualTime = 0; manualKind = 0; nearMissCooldown = 0;
        pendingLanding = null;
        if (!keepScore) { tricks = 0; airTotal = 0; lastTrickName = ''; lastTrickPoints = 0; }
      },

      update(dt, bs) {
        // Combo decay — hold at full value briefly, then bleed back to 1×.
        if (comboTimer > 0) comboTimer -= dt;
        else if (combo > 1) combo = Math.max(1, combo - COMBO_DECAY * dt);
        if (nearMissCooldown > 0) nearMissCooldown -= dt;
        if (!bs) return;

        const air = !!bs.airborne && !bs.crashed;

        if (air) {
          if (!wasAir) { airStart = ctx.time; maxWhip = 0; maxRot = 0; peakHeight = bs.position.y; }
          const w = Math.abs(num(bs.whip, 0));
          if (w > maxWhip) maxWhip = w;
          const r = num(bs.airRotation, 0);
          if (Math.abs(r) > Math.abs(maxRot)) maxRot = r;
          if (bs.position.y > peakHeight) peakHeight = bs.position.y;
          manualTime = 0;
          manualKind = 0;
        } else if (wasAir) {
          // Landing: prefer bike's own evaluation, fall back to state.lastLanding.
          let airTime, quality, cased;
          if (pendingLanding && ctx.frame - pendingLanding.frame <= 1) {
            airTime = pendingLanding.airTime;
            quality = pendingLanding.quality;
            cased = pendingLanding.cased;
          } else {
            const ll = bs.lastLanding;
            airTime = num(ll && ll.airTime, ctx.time - airStart);
            quality = clamp01(num(ll && ll.quality, num(bs.landQuality, 0.7)));
            cased = !!(ll && ll.cased);
          }
          pendingLanding = null;
          if (!bs.crashed) landAir(airTime, quality, cased, bs.position.y);
          maxWhip = 0; maxRot = 0; peakHeight = bs.position.y;
        }
        wasAir = air;

        // Manuals / endos on the ground.
        if (!air && !bs.crashed) {
          const wheelie = clamp01(num(bs.wheelie, 0));
          const endo = clamp01(num(bs.endo, 0));
          const kind = wheelie > 0.34 ? 1 : (endo > 0.42 ? 2 : 0);
          if (kind && (manualKind === 0 || manualKind === kind)) {
            manualKind = kind;
            manualTime += dt;
          } else if (manualTime > 0) {
            if (manualTime >= MANUAL_MIN) {
              const nm = manualKind === 2
                ? (manualTime > 2.0 ? tr('trick.longNoseManual') : tr('trick.noseManual'))
                : (manualTime > 2.5 ? tr('trick.longManual') : tr('trick.manual'));
              award(nm, MANUAL_RATE * manualTime, 'manual');
            }
            manualTime = 0;
            manualKind = 0;
          }
        } else if (manualTime > 0) {
          // Popped into the air out of a manual — still counts.
          if (manualTime >= MANUAL_MIN) {
            award(manualKind === 2 ? tr('trick.noseManual') : tr('trick.manual'), MANUAL_RATE * manualTime, 'manual');
          }
          manualTime = 0;
          manualKind = 0;
        }
      },

      nearMiss(distance, speed) {
        if (nearMissCooldown > 0) return;
        nearMissCooldown = NEARMISS_COOLDOWN;
        const closeness = clamp01(1 - distance / NEARMISS_RADIUS);
        const speedK = clamp01((speed - NEARMISS_MIN_SPEED) / 9);
        const raw = 26 + 44 * closeness * (0.5 + 0.5 * speedK);
        award(closeness > 0.65 ? tr('trick.treeSkimmer') : tr('trick.nearMiss'), raw, 'nearmiss');
      },

      get manualTime() { return manualTime; },
    };
  })();

  // ---------------------------------------------------------------------------
  // Near-miss detection against the instanced forest.
  //
  // vegetation.js does not publish a spatial query, so we read the near tier's
  // instance matrices directly (translation lives at elements 12/13/14) into a
  // small candidate list, refreshed on a timer / on movement rather than per
  // frame. Everything is preallocated.
  // ---------------------------------------------------------------------------

  const NM_MAX = 128;
  const nmPos = new Float32Array(NM_MAX * 3);
  let nmCount = 0;
  let nmTimer = 0;
  let nmLastX = 1e9, nmLastZ = 1e9;
  const treeMeshes = [];
  let treeScanAttempts = 0;
  let treeScanCooldown = 0;

  // Ring of recently-scored trunk positions so one tree scores once.
  const NM_RING = 12;
  const nmScored = new Float32Array(NM_RING * 2);
  let nmScoredHead = 0;
  let nmScoredCount = 0;

  function visitTreeMesh(o) {
    if (!o || !o.isInstancedMesh) return;
    const n = o.name || '';
    if (n.indexOf('trees-tier0') === 0 || n.indexOf('trees-tier1') === 0) treeMeshes.push(o);
  }
  function visitAnyTreeMesh(o) {
    if (!o || !o.isInstancedMesh) return;
    if (/tree/i.test(o.name || '')) treeMeshes.push(o);
  }

  function collectTreeMeshes() {
    treeMeshes.length = 0;
    const g = ctx.vegetation && ctx.vegetation.group;
    if (!g || typeof g.traverse !== 'function') return;
    g.traverse(visitTreeMesh);
    if (!treeMeshes.length) g.traverse(visitAnyTreeMesh);
  }

  function refreshTreeCandidates(px, py, pz) {
    nmCount = 0;
    if (!treeMeshes.length) return;
    const R = 26, R2 = R * R;
    for (let m = 0; m < treeMeshes.length && nmCount < NM_MAX; m++) {
      const mesh = treeMeshes[m];
      const arr = mesh.instanceMatrix && mesh.instanceMatrix.array;
      const n = Math.min(mesh.count | 0, arr ? (arr.length / 16) | 0 : 0);
      for (let i = 0; i < n; i++) {
        const o = i * 16;
        const x = arr[o + 12], y = arr[o + 13], z = arr[o + 14];
        const dx = x - px, dz = z - pz;
        if (dx * dx + dz * dz > R2) continue;
        if (Math.abs(y - py) > 9) continue;          // ledge above / below us
        const w = nmCount * 3;
        nmPos[w] = x; nmPos[w + 1] = y; nmPos[w + 2] = z;
        if (++nmCount >= NM_MAX) break;
      }
    }
  }

  function alreadyScored(x, z) {
    for (let i = 0; i < nmScoredCount; i++) {
      const dx = nmScored[i * 2] - x, dz = nmScored[i * 2 + 1] - z;
      if (dx * dx + dz * dz < 2.25) return true;     // within 1.5 m = same tree
    }
    return false;
  }
  function markScored(x, z) {
    nmScored[nmScoredHead * 2] = x;
    nmScored[nmScoredHead * 2 + 1] = z;
    nmScoredHead = (nmScoredHead + 1) % NM_RING;
    if (nmScoredCount < NM_RING) nmScoredCount++;
  }

  function updateNearMisses(dt, bs) {
    if (!bs || bs.crashed) return;
    const speed = num(bs.speed, 0);
    if (speed < NEARMISS_MIN_SPEED) return;

    if (!treeMeshes.length) {
      treeScanCooldown -= dt;
      if (treeScanCooldown <= 0 && treeScanAttempts < 12) {
        treeScanCooldown = 2.0;
        treeScanAttempts++;
        collectTreeMeshes();
      }
      if (!treeMeshes.length) return;
    }

    const px = bs.position.x, py = bs.position.y, pz = bs.position.z;
    nmTimer -= dt;
    const moved = (px - nmLastX) * (px - nmLastX) + (pz - nmLastZ) * (pz - nmLastZ);
    if (nmTimer <= 0 || moved > 100) {
      nmTimer = 0.3;
      nmLastX = px; nmLastZ = pz;
      refreshTreeCandidates(px, py, pz);
    }
    if (!nmCount) return;

    // Only trees that are alongside us right now — a trunk 8 m ahead is not a
    // near miss, it is a plan.
    const fx = bs.forward ? bs.forward.x : 0;
    const fz = bs.forward ? bs.forward.z : -1;
    const fl = Math.sqrt(fx * fx + fz * fz) || 1;
    const nfx = fx / fl, nfz = fz / fl;

    let bestD = Infinity, bx = 0, bz = 0;
    const R2 = NEARMISS_RADIUS * NEARMISS_RADIUS;
    for (let i = 0; i < nmCount; i++) {
      const o = i * 3;
      const dx = nmPos[o] - px, dz = nmPos[o + 2] - pz;
      const along = dx * nfx + dz * nfz;
      if (along > 1.1 || along < -1.6) continue;       // not abeam
      const d2 = dx * dx + dz * dz;
      if (d2 > R2) continue;
      if (Math.abs(nmPos[o + 1] - py) > 6) continue;
      if (d2 < bestD) { bestD = d2; bx = nmPos[o]; bz = nmPos[o + 2]; }
    }
    if (bestD === Infinity) return;
    if (alreadyScored(bx, bz)) return;
    markScored(bx, bz);
    style.nearMiss(Math.sqrt(bestD), speed);
  }

  // ---------------------------------------------------------------------------
  // Ghost — recorder, player, renderer
  // ---------------------------------------------------------------------------

  const liveRec = createRecording(GHOST_CAP0);
  let bestRec = null;
  let nextGhostSample = 0;

  const ghostSample = {
    position: new THREE.Vector3(),
    quaternion: new THREE.Quaternion(),
    lean: 0, steer: 0, spinF: 0, spinR: 0, fork: 0, shock: 0,
    speed: 0, whip: 0, wheelie: 0, trailT: 0, time: 0,
    airborne: false, crashed: false, valid: false, finished: false,
  };

  let ghostGroup = null;
  let ghostMaterial = null;
  let ghostBuilt = false;
  const ghostOwnedGeoms = [];
  const ghostWheels = [];       // nodes in the clone we can spin
  let ghostWheelAngle = 0;

  function makeGhostMaterial() {
    const mat = new THREE.MeshStandardMaterial({
      color: 0x74c8ff,
      emissive: 0x0d3a55,
      emissiveIntensity: 1.0,
      roughness: 0.55,
      metalness: 0.0,
      transparent: true,
      opacity: GHOST_OPACITY,
      depthWrite: false,
      side: THREE.FrontSide,
      fog: true,                 // MeshStandardMaterial picks up the patched height fog
      dithering: true,
      toneMapped: true,
    });
    mat.name = 'ghost';
    // A rim term so the silhouette reads against both sky and dirt. This is a
    // patched standard material, so fog/lighting/tonemapping all still apply
    // (ADDENDUM §B only bites for a hand-written ShaderMaterial).
    mat.onBeforeCompile = (shader) => {
      const token = '#include <emissivemap_fragment>';
      if (shader.fragmentShader.indexOf(token) < 0) return;
      shader.fragmentShader = shader.fragmentShader.replace(token, `${token}
        {
          vec3 vDir = normalize(vViewPosition);
          float rim = pow(1.0 - clamp(abs(dot(vDir, normal)), 0.0, 1.0), 2.2);
          totalEmissiveRadiance += vec3(0.16, 0.42, 0.62) * rim * 1.6;
          diffuseColor.a *= 0.55 + 0.75 * rim;
        }`);
    };
    return mat;
  }

  /**
   * bikeModel.group carries the bike's WORLD transform (on itself or on its
   * `chassis` child). The clone has to sit at the origin of our own ghost group,
   * so neutralise anything obviously world-space.
   */
  function neutraliseWorldTransforms(root, chassisName) {
    root.position.set(0, 0, 0);
    root.quaternion.identity();
    root.updateMatrix();
    for (const child of root.children) {
      const isChassis = chassisName && child.name === chassisName;
      // No legitimate bike part sits more than 5 m from the bike origin, so a
      // large offset can only be a world-space placement.
      if (isChassis || child.position.lengthSq() > 25) {
        child.position.set(0, 0, 0);
        child.quaternion.identity();
        child.updateMatrix();
      }
    }
  }

  function buildGhostVisual() {
    if (ghostBuilt) return;
    ghostBuilt = true;
    if (!ctx.scene) return;

    ghostMaterial = makeGhostMaterial();
    ghostGroup = new THREE.Group();
    ghostGroup.name = 'ghost';
    ghostGroup.visible = false;
    ghostGroup.matrixAutoUpdate = true;
    ghostGroup.renderOrder = 6;

    let haveBike = false;
    const bm = ctx.bikeModel;
    if (bm && bm.group && typeof bm.group.clone === 'function' && bm.group.children.length) {
      try {
        const clone = bm.group.clone(true);
        const chassisName = (bm.chassis && bm.chassis !== bm.group) ? bm.chassis.name : '';
        neutraliseWorldTransforms(clone, chassisName);
        clone.traverse((o) => {
          if (o.isMesh || o.isInstancedMesh || o.isLine || o.isPoints) {
            o.material = ghostMaterial;
            o.castShadow = false;
            o.receiveShadow = false;
            o.customDepthMaterial = undefined;
            o.customDistanceMaterial = undefined;
            o.renderOrder = 6;
          }
          if (/wheel/i.test(o.name || '') && ghostWheels.length < 4) ghostWheels.push(o);
        });
        ghostGroup.add(clone);
        haveBike = true;
      } catch (e) {
        haveBike = false;
      }
    }

    if (!haveBike) {
      const wr = num(ctx.bike && ctx.bike.WHEEL_RADIUS, 0.37);
      const wb = num(ctx.bike && ctx.bike.WHEELBASE, 1.25);
      const geo = buildFallbackBike(wr, wb);
      if (geo) {
        ghostOwnedGeoms.push(geo);
        const mesh = new THREE.Mesh(geo, ghostMaterial);
        mesh.name = 'ghost-bike';
        mesh.renderOrder = 6;
        ghostGroup.add(mesh);
      }
    }

    const riderGeo = buildRiderSilhouette();
    if (riderGeo) {
      ghostOwnedGeoms.push(riderGeo);
      const riderMesh = new THREE.Mesh(riderGeo, ghostMaterial);
      riderMesh.name = 'ghost-rider';
      riderMesh.renderOrder = 6;
      ghostGroup.add(riderMesh);
    }

    ctx.scene.add(ghostGroup);
    ghost.object = ghostGroup;
  }

  function recordGhostSample(bs) {
    if (!bs) return;
    const off = recAlloc(liveRec);
    if (off < 0) return;
    const d = liveRec.data;
    d[off + G.T] = runTime;
    d[off + G.PX] = bs.position.x;
    d[off + G.PY] = bs.position.y;
    d[off + G.PZ] = bs.position.z;
    d[off + G.QX] = bs.quaternion.x;
    d[off + G.QY] = bs.quaternion.y;
    d[off + G.QZ] = bs.quaternion.z;
    d[off + G.QW] = bs.quaternion.w;
    d[off + G.LEAN] = num(bs.lean, 0);
    d[off + G.STEER] = num(bs.steer, 0);
    const wf = bs.wheels && bs.wheels[0];
    const wr = bs.wheels && bs.wheels[1];
    d[off + G.SPINF] = num(wf && wf.spinRate, 0);
    d[off + G.SPINR] = num(wr && wr.spinRate, 0);
    const susp = bs.suspension;
    d[off + G.FORK] = num(susp && susp.fork && susp.fork.travel, 0);
    d[off + G.SHOCK] = num(susp && susp.shock && susp.shock.travel, 0);
    let flags = 0;
    if (bs.airborne) flags |= F_AIR;
    if (bs.crashed) flags |= F_CRASH;
    if (num(bs.brakeRear, 0) > 0.25 || num(bs.brakeFront, 0) > 0.25) flags |= F_BRAKE;
    if (num(bs.wheelie, 0) > 0.3) flags |= F_MANUAL;
    d[off + G.FLAGS] = flags;
    d[off + G.SPEED] = num(bs.speed, 0);
    d[off + G.WHIP] = num(bs.whip, 0);
    d[off + G.WHEELIE] = num(bs.wheelie, 0);
    d[off + G.TRAILT] = clamp01(num(bs.trailT, 0));
    liveRec.duration = runTime;
  }

  /** The run just became the best — keep its recording and persist it. */
  function promoteGhost() {
    if (liveRec.count < 8) return;
    if (!bestRec) bestRec = createRecording(liveRec.count);
    recCopyInto(liveRec, bestRec);
    bestRec.meta = { time: runTime, seed };
    ghost.available = true;
    ghost.duration = bestRec.duration;
    ghost.sampleCount = bestRec.count;

    // Persist, decimating if the encoded string would be antisocial.
    for (let decim = 1; decim <= 8; decim *= 2) {
      const payload = encodeRecording(bestRec, decim);
      if (!payload) return;
      if (payload.b64.length > GHOST_MAX_CHARS) {
        if (decim < 8) continue;
        break;                       // still too fat at 3.75 Hz — do not store it
      }
      payload.time = runTime;
      payload.seed = seed;
      if (store.saveGhost(payload)) return;
    }
    // Everything failed (quota) — drop any stale ghost rather than leaving a
    // recording from a slower run on disk.
    store.clearGhost();
  }

  function loadGhost() {
    const payload = store.loadGhost();
    if (!payload) return;
    if (payload.seed !== undefined && (payload.seed >>> 0) !== seed) return;
    let rec = null;
    try { rec = decodeRecording(payload); } catch (e) { rec = null; }
    if (!rec) { store.clearGhost(); return; }
    bestRec = rec;
    bestRec.meta = { time: num(payload.time, rec.duration), seed };
    ghost.available = true;
    ghost.duration = rec.duration;
    ghost.sampleCount = rec.count;
  }

  function updateGhost(dt) {
    const show = !!(bestRec && bestRec.count > 1 &&
      (state === 'running' || state === 'crashed' || state === 'countdown'));
    if (!show) {
      ghost.playing = false;
      if (ghostGroup) ghostGroup.visible = false;
      return;
    }

    const t = state === 'countdown' ? 0 : runTime;
    ghost.time = t;
    recSample(bestRec, t, ghostSample);
    ghost.finished = !!ghostSample.finished;
    ghost.playing = ghostSample.valid && !ghostSample.finished;

    if (!ghostSample.valid) {
      if (ghostGroup) ghostGroup.visible = false;
      return;
    }

    ghost.position.copy(ghostSample.position);
    ghost.quaternion.copy(ghostSample.quaternion);
    ghost.speed = ghostSample.speed;
    ghost.lean = ghostSample.lean;
    ghost.airborne = ghostSample.airborne;
    ghost.trailT = ghostSample.trailT;

    // Progress delta: how far ahead/behind the ghost we are right now.
    const bs = ctx.bike && ctx.bike.state;
    if (bs && state === 'running') {
      const gt = recTimeAtProgress(bestRec, clamp01(num(bs.trailT, 0)));
      ghostDelta = isFinite(gt) ? runTime - gt : NaN;
    }
    ghost.delta = ghostDelta;

    if (!ghostBuilt) buildGhostVisual();
    if (!ghostGroup) return;

    // Proximity fade rather than a hard cut. Riding wheel-to-wheel with the ghost
    // is the whole point of it, and a binary hide would pop exactly then; instead
    // it thins out as it overlaps your own bike, which is where it would otherwise
    // just be visual noise on top of the rider.
    let opacity = GHOST_OPACITY;
    if (bs) {
      const d = ghostSample.position.distanceTo(bs.position);
      opacity *= clamp01((d - 0.7) / 1.6);
    }
    // And fade out over the last second of the recording so it does not vanish
    // mid-stride when the ghost's run ends before yours.
    const remaining = bestRec.duration - t;
    if (remaining < 1.0) opacity *= clamp01(remaining);

    const vis = !ghostSample.finished && opacity > 0.012;
    ghostGroup.visible = vis;
    if (!vis) return;
    if (ghostMaterial) ghostMaterial.opacity = opacity;

    ghostGroup.position.copy(ghostSample.position);
    ghostGroup.quaternion.copy(ghostSample.quaternion);

    if (ghostWheels.length) {
      ghostWheelAngle -= ghostSample.spinR * dt;
      if (ghostWheelAngle > 1e5 || ghostWheelAngle < -1e5) ghostWheelAngle = 0;
      for (let i = 0; i < ghostWheels.length; i++) ghostWheels[i].rotation.x = ghostWheelAngle;
    }
  }

  // ---------------------------------------------------------------------------
  // Public object
  // ---------------------------------------------------------------------------

  const ghost = {
    available: false,
    playing: false,
    finished: false,
    time: 0,
    duration: 0,
    sampleCount: 0,
    delta: NaN,
    speed: 0,
    lean: 0,
    trailT: 0,
    airborne: false,
    position: new THREE.Vector3(),
    quaternion: new THREE.Quaternion(),
    object: null,
    /** Sample the stored best run at an arbitrary time. Returns null if none. */
    sample(t, out) {
      if (!bestRec || bestRec.count < 1) return null;
      return recSample(bestRec, num(t, 0), out || ghostSample);
    },
    setVisible(v) { if (ghostGroup) ghostGroup.visible = !!v; },
    clear() {
      bestRec = null;
      ghost.available = false;
      ghost.sampleCount = 0;
      ghost.duration = 0;
      if (ghostGroup) ghostGroup.visible = false;
      store.clearGhost();
    },
  };

  const api = {
    // --- CONTRACT §8 -------------------------------------------------------
    state,
    time: 0,
    splits,
    best,
    score: 0,
    start,
    restart,
    pause,

    // --- additive ----------------------------------------------------------
    ghost,
    checkpointIndex: 0,
    checkpointCount: checkpoints.length,
    lastSplit: null,
    penalty: 0,
    crashes: 0,
    progress: 0,             // 0..1 along the trail
    countdown: 0,            // seconds remaining in the countdown
    countdownLabel: '',
    isBest: false,
    liveDelta: NaN,          // + = behind PB pace
    ghostDelta: NaN,
    style: {
      multiplier: 1, meter: 0, tricks: 0, airTime: 0, airTotal: 0,
      lastTrick: '', lastTrickPoints: 0, lastTrickAt: -99, manualTime: 0,
    },
    seed,
    storageAvailable: store.available,

    /** A menu can claim the title screen so the run does not auto-start. */
    hold() { holdRequested = true; if (state !== 'menu') setState('menu'); return api; },
    release() { holdRequested = false; return api; },
    resume() { return pause(false); },
    toMenu() {
      resetRunData();
      placeAtStart();
      holdRequested = true;
      setState('menu');
      return api;
    },
    clearRecords() {
      store.clearAll();
      best = null;
      api.best = null;
      ghost.clear();
      return api;
    },
    formatTime,
  };

  api.state = state;

  // ---------------------------------------------------------------------------
  // Event wiring
  // ---------------------------------------------------------------------------

  on('run:crash', () => {
    if (!emittingOwnCrash) externalCrashTime = num(ctx.time, 0);
  });
  on('bike:crash', (e) => {
    onCrash(e && e.severity, e && e.position, e && e.reason);
  });
  on('bike:landed', (e) => { style.noteLanding(e); });
  on('bike:respawn', () => {
    // The bike beat our crash beat to it (its own TUMBLE_MAX). Close the beat
    // out now so the run never sits in 'crashed' with a rideable bike.
    if (state === 'crashed') crashBeat = CRASH_BEAT;
  });

  // ---------------------------------------------------------------------------
  // Frame
  // ---------------------------------------------------------------------------

  let edgePause = false;
  let edgeReset = false;

  function update(dt) {
    const d = clamp(num(dt, 0), 0, 1 / 20);
    stateTime += d;
    if (respawnGrace > 0) respawnGrace -= d;

    const input = ctx.input && ctx.input.state;
    const bs = ctx.bike && ctx.bike.state;

    // ---- discrete input (edge-triggered by input.js, one frame each) -------
    if (input) {
      if (input.cameraCycle) manualCameraOverride = true;
      // R — always a full restart, whatever state we are in. bike.js consumes
      // the same edge and resets itself to the gate, so the two agree.
      if (input.reset && !edgeReset) restart();
      edgeReset = !!input.reset;

      if (input.pause && !edgePause) {
        if (state === 'menu') { if (!menuOwnsScreen()) start(); }
        else if (state === 'finished') restart();
        else pause();
      }
      edgePause = !!input.pause;
    }

    tickClock(d);
    api.time = runTime;
    api.penalty = penaltyTotal;
    api.crashes = crashCount;
    api.checkpointCount = checkpoints.length;

    switch (state) {
      case 'menu': {
        // Never let a stub menu strand the player on a frozen mountainside.
        if (!menuOwnsScreen() &&
            (stateTime > MENU_AUTOSTART || (input && input.anyPressed && stateTime > 0.25))) {
          start();
        }
        break;
      }

      case 'countdown': {
        const total = COUNTDOWN_BEAT * 3;
        const idx = Math.min(3, Math.floor(stateTime / COUNTDOWN_BEAT));
        if (idx !== countdownIndex) {
          countdownIndex = idx;
          evCountdown.index = idx;
          evCountdown.count = Math.max(0, 3 - idx);
          evCountdown.label = idx >= 3 ? 'GO' : String(3 - idx);
          emit('run:countdown', evCountdown);
        }
        api.countdown = Math.max(0, total - stateTime);
        api.countdownLabel = idx >= 3 ? 'GO' : String(3 - idx);
        if (stateTime >= total) {
          goHold += d;
          if (goHold >= COUNTDOWN_GO_HOLD || (input && (input.pedal > 0.1 || input.pump > 0.1))) {
            beginRunning();
          }
        }
        break;
      }

      case 'running': {
        api.countdown = 0;
        if (bs) {
          const tt = clamp01(num(bs.trailT, 0));
          api.progress = tt;

          // Checkpoints. Sequential, and tolerant of a teleport that skips one
          // (we simply take them all) so the sequence can never wedge.
          while (nextCheckpoint < checkpoints.length && tt >= checkpoints[nextCheckpoint].t) {
            takeCheckpoint(nextCheckpoint);
            nextCheckpoint++;
          }

          updateLiveDelta(tt);
          api.liveDelta = liveDelta;

          // Finish. Require at least one checkpoint so a glitch at the gate
          // cannot end the run before it starts.
          if (tt >= finishT && (checkpointsTaken > 0 || checkpoints.length === 0)) {
            finishRun();
            break;
          }

          // Crash fallback: a crashed bike while we think we are riding means the
          // event was missed (or a respawn did not take). Level-triggered, not
          // edge-triggered, so it self-heals rather than latching.
          if (bs.crashed && respawnGrace <= 0) onCrash(0.7, bs.position, 'state');
        }
        break;
      }

      case 'crashed': {
        crashBeat += d;
        // Either the bike put itself back on the trail during the tumble (its own
        // TUMBLE_MAX beat us to it) or our beat has run — either way, one exit.
        const settled = bs ? !bs.crashed : true;
        if ((settled && crashBeat > 0.35) || crashBeat >= CRASH_BEAT) doRespawn();
        break;
      }

      case 'finished': {
        api.countdown = 0;
        api.progress = 1;
        break;
      }

      case 'paused':
      default:
        break;
    }

    // ---- style + ghost ----------------------------------------------------
    if (state === 'running') {
      style.update(d, bs);
      updateNearMisses(d, bs);

      // Fixed-rate ghost capture. A time jump (crash penalty) re-bases the
      // schedule instead of firing a burst of samples.
      if (bs) {
        if (runTime >= nextGhostSample) {
          recordGhostSample(bs);
          nextGhostSample += GHOST_DT;
          if (runTime > nextGhostSample + GHOST_DT * 4) nextGhostSample = runTime + GHOST_DT;
        }
      }
    } else if (state === 'crashed') {
      style.update(d, bs);
    }

    updateGhost(d);
    api.ghostDelta = ghostDelta;

    api.style.multiplier = style.combo;
    api.style.meter = style.meter;
    api.style.tricks = style.tricks;
    api.style.airTime = style.airborneTime;
    api.style.airTotal = style.airTotal;
    api.style.lastTrick = style.lastTrickName;
    api.style.lastTrickPoints = style.lastTrickPoints;
    api.style.lastTrickAt = style.lastTrickAt;
    api.style.manualTime = style.manualTime;
    api.isBest = isBest;
    api.best = best;

    flushCrashEmit();
  }

  function init() {
    syncTrail();
    api.checkpointCount = checkpoints.length;
    loadGhost();
    api.best = best;
    collectTreeMeshes();

    // Always open on 'menu'. If nothing claims the screen, update() rolls straight
    // into the countdown a beat later, so a stub menu can never strand the player.
    setState('menu');
    placeAtStart();
    emit('run:ready', { best, seed, checkpoints: checkpoints.length, ghost: ghost.available });
  }

  function dispose() {
    for (const off of unsubs) { try { off(); } catch (e) { /* ignore */ } }
    unsubs.length = 0;
    if (ghostGroup) {
      if (ctx.scene) ctx.scene.remove(ghostGroup);
      ghostGroup.clear();
      ghostGroup = null;
    }
    for (const g of ghostOwnedGeoms) { try { g.dispose(); } catch (e) { /* ignore */ } }
    ghostOwnedGeoms.length = 0;
    if (ghostMaterial) { ghostMaterial.dispose(); ghostMaterial = null; }
    treeMeshes.length = 0;
    ghost.object = null;
  }

  api.init = init;
  api.update = update;
  api.dispose = dispose;

  return api;
}

// -----------------------------------------------------------------------------
// Shared formatter — the HUD and the menu should show identical strings.
// -----------------------------------------------------------------------------
export function formatTime(seconds, decimals = 2) {
  const s = Math.max(0, num(seconds, 0));
  const m = Math.floor(s / 60);
  const rem = s - m * 60;
  const pad = rem < 10 ? '0' : '';
  return `${m}:${pad}${rem.toFixed(decimals)}`;
}

export function formatDelta(delta, decimals = 2) {
  if (!isFinite(delta)) return '';
  const sign = delta >= 0 ? '+' : '−';
  return `${sign}${Math.abs(delta).toFixed(decimals)}`;
}
