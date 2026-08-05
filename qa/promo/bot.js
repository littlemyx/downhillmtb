// Promo-capture bot ("bot v4"). Self-contained; loaded on demand from the page console
// of a dev-server session (the hidden browser pane works — see qa/promo/README.md):
//
//   const bot = await import('/qa/promo/bot.js');
//   await bot.runAll();                    // all four proven segments
//   await bot.recordSegment(bot.SEGMENTS[1], { slow: 4 });   // one segment, 4x slow-mo
//
// Frames are POSTed to /qa/shot (the qaCapture middleware in vite.config.js) and land in
// qa/shots/ as `<seg>_NNNN.png` — the files CONTAIN JPEG despite the extension; rename to
// .jpg before ffmpeg. Assembly commands are in qa/promo/README.md.
//
// This file is orchestration-owned, like qa/harness.js. Game modules must not import it.
//
// WHY MANUAL TICKING: in a hidden browser pane rAF is frozen but promises/fetch still run.
// The game's own loop (main.js tick()) never fires, so the bot drives the frame itself:
// update()+lateUpdate() over ORDER_REC below, then postfx.render(), then a canvas readback.
// This also makes capture deterministic — one physics tick per captured frame, wall-clock
// independent.

const g = () => {
  const d = window.__DESCENT__;
  if (!d) throw new Error('game not booted');
  return d;
};

// Same wave order as ORDER in src/main.js, minus:
//  - hud/menu: DOM overlays — invisible to canvas.toDataURL, pure waste;
//  - audio: AudioContext is suspended in a hidden pane, ticking it is useless.
// chaseCamera stays in the list but is frozen (see freezeCamera) — the bot drives its own
// camera, because the stock chase camera is tuned for play, not for footage.
const ORDER_REC = ['input', 'terrain', 'trail', 'sky', 'water', 'vegetation',
                   'collision', 'bike', 'bikeModel', 'rider', 'particles',
                   'chaseCamera', 'gameplay', 'postfx'];

// =============================================================================
// Small vector helpers (constructor borrowed from live objects — no three import)
// =============================================================================
const V3 = (x = 0, y = 0, z = 0) => new (g().camera.position.constructor)(x, y, z);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const clamp01 = (v) => clamp(v, 0, 1);

/** Stop the game's own camera driver so the bot camera sticks. Same trick as qa/harness.js. */
export function freezeCamera(on = true) {
  const cc = g().chaseCamera;
  if (!cc) return;
  if (on) {
    if (!cc.__qaSaved) cc.__qaSaved = { update: cc.update, lateUpdate: cc.lateUpdate };
    cc.update = () => {};
    cc.lateUpdate = () => {};
  } else if (cc.__qaSaved) {
    cc.update = cc.__qaSaved.update;
    cc.lateUpdate = cc.__qaSaved.lateUpdate;
    delete cc.__qaSaved;
  }
}

// =============================================================================
// Game loop, driven by hand
// =============================================================================

/**
 * One full game tick. `drive` is injected immediately AFTER input.update() so the
 * autopilot's writes to input.state are not low-pass-filtered back toward the
 * keyboard's zeros (input.update() rate-limits state toward held keys every tick).
 */
export function tickGame(dt, drive = null) {
  const d = g();
  d.time += dt;
  d.frame++;
  for (const k of ORDER_REC) {
    const s = d[k];
    if (s && typeof s.update === 'function') {
      try { s.update(dt, d); } catch (e) { /* one bad system must not stop the take */ }
    }
    if (k === 'input' && drive) drive(dt);
  }
  for (const k of ORDER_REC) {
    const s = d[k];
    if (s && typeof s.lateUpdate === 'function') {
      try { s.lateUpdate(dt, d); } catch (e) { /* keep the take alive */ }
    }
  }
}

/**
 * Full restart, then sit through the countdown WITHOUT letting the bike run.
 *
 * CRITICAL, learned the hard way: after gameplay.restart() the warm-up must tick ONLY
 * ['gameplay'] until it reaches 'running'. If the whole ORDER_REC is ticked here, the bot
 * rides from the gate — and the start descent is unrideable (see README), so the bike
 * crashes, gameplay arms its delayed respawn (CRASH_BEAT), and that pending respawn then
 * teleports the bike back mid-take, ruining the NEXT segment's teleport.
 * Countdown is 3 x 0.85 s + 0.28 s GO-hold (~2.83 s -> ~180 ticks at 1/60).
 */
export function restartToRunning(maxTicks = 400) {
  const d = g();
  d.gameplay.restart();
  for (let i = 0; i < maxTicks; i++) {
    d.time += 1 / 60; d.frame++;
    try { d.gameplay.update(1 / 60, d); } catch (e) { /* ignore */ }
    if (d.gameplay.state === 'running') return true;
  }
  return d.gameplay.state === 'running';
}

/**
 * Put the bike on the trail at parameter `t`, rolling at `v0` m/s.
 *
 * bike.reset({t}) is called FIRST — not for the transform (reset's own transform handling
 * is not trusted: it snaps height, discards roll/pitch and zeroes velocity — session-
 * verified that relying on it produces bad spawns) — but because it clears every
 * controller filter/integrator AND seeds state.trailT, which nearestT() uses as its
 * search hint; a stale hint can snap the bike to the wrong leg of a switchback.
 * The real transform then goes in through the supported mutators
 * (setPosition/setOrientation/setVelocity — bike.state itself is read-only).
 *
 * Height: state.position is the chassis origin at the AXLE MIDPOINT at axle height
 * (bike.geometry.chassisOrigin), so ground + wheelRadius + a small clearance.
 */
export function teleport(t, v0 = 9.5) {
  const d = g();
  const s = d.trail.sampleAt(t);              // NOTE: sampleAt returns a REUSED object —
  const pos = s.position.clone();             // clone everything before the next call.
  const tan = s.tangent.clone().normalize();
  d.bike.reset({ t });

  const wheelR = (d.bike.geometry && d.bike.geometry.wheelRadius) || 0.34;
  pos.y = d.terrain.sampleHeight(pos.x, pos.z) + wheelR + 0.06;
  d.bike.setPosition(pos);

  // Yaw-only orientation facing down the trail. bike forward is (0,0,-1) rotated by Q;
  // rotating (0,0,-1) about +Y by theta gives (-sin, 0, -cos) => theta = atan2(-tx, -tz).
  const q = new (d.camera.quaternion.constructor)();
  q.setFromAxisAngle(V3(0, 1, 0), Math.atan2(-tan.x, -tan.z));
  d.bike.setOrientation(q);
  d.bike.setVelocity(tan.clone().multiplyScalar(v0));

  // A few settle ticks with neutral input so the suspension finds its sag before frame 0.
  for (let i = 0; i < 5; i++) tickGame(1 / 60, () => neutralInput());
  return pos;
}

// =============================================================================
// Autopilot
// =============================================================================
const GRAV = 9.81;
const AP = {
  lookaheadK: 0.7,      // lookahead metres per (m/s) of speed
  lookaheadMin: 4, lookaheadMax: 18,
  steerGain: 2.2,       // steer = clamp(gain * signed angle to lookahead point)
  scanAheadM: 45,       // corner-speed scan window
  scanStepM: 3,
  leanAvail: 0.42,      // rad of lean margin on top of built bank for the corner limit
  aBrake: 5.5,          // m/s^2 assumed decel for the braking-distance relaxation
  brakeGain: 0.55,      // brake per (m/s) over the limit
  pedalOn: 0.8,
};

function neutralInput() {
  const s = g().input.state;
  s.steer = 0; s.pitch = 0; s.roll = 0; s.pump = 0;
  s.pedal = 0; s.brakeFront = 0; s.brakeRear = 0;
}

/**
 * Corner speed limit at trail parameter `t`, from XZ-projected curvature.
 *
 * IMPORTANT: the tangent's vertical component must be EXCLUDED before measuring the
 * heading change — with it included, every pitch change (a roller, a grade break) reads
 * as a "corner" and the bot brakes on straights. Only the XZ turn is a turn.
 * Bank raises the limit: standard banked-corner v^2 = g * tan(bank + lean) / kappa.
 */
function cornerLimitAt(t) {
  const d = g();
  const L = d.trail.length;
  const dm = 4;                                 // curvature baseline, metres
  const s0 = d.trail.sampleAt(t);
  const h0 = Math.atan2(s0.tangent.x, s0.tangent.z);   // XZ heading only
  const bank = Math.abs(s0.bank || 0);
  const s1 = d.trail.sampleAt(Math.min(0.999, t + dm / L));
  const h1 = Math.atan2(s1.tangent.x, s1.tangent.z);
  let dh = h1 - h0;
  if (dh > Math.PI) dh -= 2 * Math.PI;
  if (dh < -Math.PI) dh += 2 * Math.PI;
  const kappa = Math.abs(dh) / dm;              // rad/m of XZ turn
  if (kappa < 1e-4) return Infinity;
  const aLat = GRAV * Math.tan(Math.min(bank + AP.leanAvail, 1.2));
  return Math.sqrt(Math.max(1, aLat / kappa));
}

/**
 * Drive one tick: pursuit steering toward a speed-scaled lookahead point, and a speed
 * governor that scans the trail ahead and brakes early enough (braking-distance model)
 * instead of panicking at the apex.
 */
function makeDriver(cap) {
  return function drive() {
    const d = g();
    const b = d.bike.state;
    if (!b || b.crashed) return;
    const s = d.input.state;
    const t = b.trailT;
    const L = d.trail.length;
    const speed = b.speed || 0;

    // ---- steering: signed XZ angle from bike forward to the lookahead point ----
    const la = clamp(speed * AP.lookaheadK, AP.lookaheadMin, AP.lookaheadMax);
    const target = d.trail.sampleAt(Math.min(0.999, t + la / L)).position.clone();
    const fwd = V3(0, 0, -1).applyQuaternion(b.quaternion);
    const tx = target.x - b.position.x, tz = target.z - b.position.z;
    // positive = target on the rider's right = steer right (+1). Verified convention:
    // input.steer +1 is KeyD/ArrowRight.
    const ang = Math.atan2(fwd.x * tz - fwd.z * tx, fwd.x * tx + fwd.z * tz);
    s.steer = clamp(ang * AP.steerGain, -1, 1);
    s.roll = 0; s.pump = 0;

    // ---- speed governor: min over the scan window, relaxed by braking distance ----
    let vmax = cap;
    for (let m = 0; m <= AP.scanAheadM; m += AP.scanStepM) {
      const lim = cornerLimitAt(Math.min(0.999, t + m / L));
      if (lim === Infinity) continue;
      // allowed-now speed so that aBrake gets us down to `lim` after `m` metres
      const allowed = Math.sqrt(lim * lim + 2 * AP.aBrake * m);
      if (allowed < vmax) vmax = allowed;
    }

    const over = speed - vmax;
    const grad = -((d.trail.sampleAt(t).tangent.y) || 0);   // + = descending
    if (over > 0.3) {
      const brake = clamp01(over * AP.brakeGain);
      // Front brake fades out with gradient — full front brake on a steep pitch is an
      // instant endo (input.pitch +1 is nose-DOWN, so weight goes BACK: pitch < 0).
      const frontScale = clamp01(0.55 - 1.4 * grad);
      s.brakeFront = brake * frontScale;
      s.brakeRear = brake;
      s.pedal = 0;
      s.pitch = -clamp01(0.35 + 0.5 * brake);   // weight back while braking
    } else {
      s.brakeFront = 0; s.brakeRear = 0;
      s.pedal = speed < vmax - 0.8 ? AP.pedalOn : 0;
      s.pitch = -clamp01(grad * 1.2) * 0.4;     // a little back on steeps even off-brake
    }
  };
}

// =============================================================================
// Bot camera — smoothed follow, aimed along velocity, never inside the hill
// =============================================================================
const CAM = { back: 6.5, up: 2.3, lookAhead: 9, lookUp: 0.9, lerp: 0.14, fov: 66, clearance: 0.5 };

function makeCamera() {
  const d = g();
  const dir = V3(0, 0, -1);
  let first = true;
  if (d.camera.fov !== CAM.fov) { d.camera.fov = CAM.fov; d.camera.updateProjectionMatrix(); }
  return function updateCam() {
    const b = d.bike.state;
    if (!b) return;
    // direction from velocity (XZ), falling back to the chassis forward at low speed
    const v = b.velocity;
    if (v && (v.x * v.x + v.z * v.z) > 4) dir.set(v.x, 0, v.z).normalize();
    else { dir.set(0, 0, -1).applyQuaternion(b.quaternion); dir.y = 0; dir.normalize(); }
    const desired = V3(
      b.position.x - dir.x * CAM.back,
      b.position.y + CAM.up,
      b.position.z - dir.z * CAM.back,
    );
    const gy = d.terrain.sampleHeight(desired.x, desired.z);
    if (Number.isFinite(gy) && desired.y < gy + CAM.clearance) desired.y = gy + CAM.clearance;
    if (first) { d.camera.position.copy(desired); first = false; }
    else d.camera.position.lerp(desired, CAM.lerp);
    d.camera.lookAt(
      b.position.x + dir.x * CAM.lookAhead,
      b.position.y + CAM.lookUp,
      b.position.z + dir.z * CAM.lookAhead,
    );
  };
}

// =============================================================================
// Capture
// =============================================================================

/**
 * Pin the backbuffer to the capture size. Called EVERY tick, not once: the engine's
 * pixel-ratio governor and the main.js resize handler both fight the capture size when
 * the pane reports 0x0, and postfx keeps its own render targets (postfx.resize).
 */
function makePin(w, h) {
  const d = g();
  return function pin() {
    d.renderer.setPixelRatio(1);
    d.renderer.setSize(w, h, false);
    if (d.camera.aspect !== w / h) { d.camera.aspect = w / h; d.camera.updateProjectionMatrix(); }
    if (d.postfx && d.postfx.resize) d.postfx.resize(w, h);
  };
}

function drawFrame(dt) {
  const d = g();
  if (d.postfx && d.postfx.render) d.postfx.render(dt);
  else d.renderer.render(d.scene, d.camera);
}

async function postFrame(name, quality = 0.92) {
  const d = g();
  // JPEG, not PNG: a 1080p PNG readback+POST per frame is 5-10x slower and the middleware
  // stores whatever bytes it gets. The saved file gets a .png EXTENSION but JPEG CONTENT —
  // rename to .jpg before ffmpeg (see README).
  const dataUrl = d.renderer.domElement.toDataURL('image/jpeg', quality);
  const r = await fetch('/qa/shot', {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ name, dataUrl }),
  });
  return r.json();
}

// =============================================================================
// Takes
// =============================================================================

/**
 * Record one take. Returns { ok, frames, crashedAt }.
 *  - `slow`: 1 = real time (dt 1/60, capture every 2nd tick, play at 30 fps);
 *            4 = honest 4x slow-mo (dt 1/120, capture EVERY tick, play at 30 fps).
 *    Slow-mo is done by ticking physics finer, never by duplicating frames.
 *  - Success criterion: ZERO crashes inside the recorded window. Any crash aborts the
 *    take immediately (a crashed frame is not promo footage, and gameplay's delayed
 *    respawn would fire mid-take).
 */
export async function recordTake({ t, name, frames = 240, v0 = 9.5, cap = 12.5,
                                   slow = 1, w = 1920, h = 1080 } = {}) {
  const d = g();
  const gov = d.engine && typeof d.engine.setGovernorEnabled === 'function';
  if (gov) { try { d.engine.setGovernorEnabled(false); } catch (e) { /* older engine */ } }
  freezeCamera(true);

  if (!restartToRunning()) { console.warn('[bot] countdown never finished'); }
  teleport(t, v0);

  const drive = makeDriver(cap);
  const cam = makeCamera();
  const pin = makePin(w, h);
  const dt = slow > 1 ? 1 / (30 * slow) : 1 / 60;
  const ticksPerFrame = slow > 1 ? 1 : 2;

  // warm frames: TAA / adaptive exposure / LOD settle before frame 0000
  for (let i = 0; i < 10; i++) { tickGame(dt, drive); cam(); pin(); drawFrame(dt); }

  let crashedAt = -1;
  for (let f = 0; f < frames; f++) {
    for (let k = 0; k < ticksPerFrame; k++) tickGame(dt, drive);
    if (d.bike.state.crashed) { crashedAt = f; break; }
    cam();
    pin();
    drawFrame(dt * ticksPerFrame);
    await postFrame(`${name}_${String(f).padStart(4, '0')}`);
    if (d.bike.state.crashed) { crashedAt = f; break; }
  }

  neutralInput();
  if (gov) { try { d.engine.setGovernorEnabled(true); } catch (e) { /* ignore */ } }
  return { ok: crashedAt < 0, frames: crashedAt < 0 ? frames : crashedAt, crashedAt, name };
}

/**
 * Record a segment with retries. A failed attempt leaves partial frames on disk under the
 * same names; the succeeding attempt overwrites every one of them, so no cleanup between
 * attempts is needed. Between attempts a FULL restart happens inside recordTake — that is
 * what clears gameplay's pending crash/respawn state.
 */
export async function recordSegment(seg, overrides = {}) {
  const opts = { ...seg, ...overrides };
  const retries = opts.retries ?? 4;
  for (let a = 0; a <= retries; a++) {
    const r = await recordTake(opts);
    if (r.ok) { console.log(`[bot] ${opts.name}: OK in attempt ${a + 1} (${r.frames} frames)`); return r; }
    console.warn(`[bot] ${opts.name}: crash at frame ${r.crashedAt}, attempt ${a + 1}/${retries + 1}`);
  }
  return { ok: false, name: opts.name };
}

// =============================================================================
// Proven segments
// =============================================================================
// The trail START (t 0.05-0.19) is UNRIDEABLE for the bot — the corner-berming pass runs
// in only 3 of 8 corner phases there (known issue, see the project README) and the bot
// cannot survive unbermed switchbacks. Do not add takes in that window.
// These four were verified clean over multiple takes at the caps/speeds given:
export const SEGMENTS = [
  { name: 'seg-berms',  t: 0.29, cap: 12.5, v0: 9.5,  frames: 240 },  // banked flow corners
  { name: 'seg-jumps',  t: 0.44, cap: 13.5, v0: 10.0, frames: 240 },  // jump line
  { name: 'seg-forest', t: 0.75, cap: 12.0, v0: 9.0,  frames: 240 },  // forest loam
  { name: 'seg-finish', t: 0.95, cap: 13.5, v0: 10.0, frames: 240 },  // final sprint
];

export async function runAll(overrides = {}) {
  const out = [];
  for (const seg of SEGMENTS) out.push(await recordSegment(seg, overrides));
  console.table(out);
  return out;
}

window.__PROMO_BOT__ = { runAll, recordSegment, recordTake, teleport, restartToRunning,
                         tickGame, freezeCamera, SEGMENTS, AP, CAM };
