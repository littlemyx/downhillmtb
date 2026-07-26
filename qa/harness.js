// Visual-QA harness. Loaded on demand from the page console:
//   const qa = await import('/qa/harness.js');  await qa.runShotSet();
// Renders a fixed set of camera setups at full resolution and POSTs each PNG to
// /qa/shot, which the vite dev middleware writes into qa/shots/.
//
// This file is orchestration-owned. Game modules must not import it.

const V = (x, y, z) => new (window.__DESCENT__.camera.position.constructor)(x, y, z);

function ctx() {
  const d = window.__DESCENT__;
  if (!d) throw new Error('game not booted');
  return d;
}

/** Stop the game's own camera driver so a QA pose sticks. */
export function freezeCamera(on = true) {
  const d = ctx();
  const cc = d.chaseCamera;
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

/**
 * Put the bike (and therefore the rider) on the trail at `t`, facing down the trail.
 * The bike and rider are the two most-scrutinised assets in a bike game and must appear
 * in the review set — without this the whole set is landscape photography.
 * Also anchors the depth-of-field focus target, which reads the bike position.
 */
export function placeBike(t) {
  const d = ctx();
  if (!d.bike) return null;
  const s = d.trail.sampleAt(t);
  const tan = s.tangent.clone().normalize();
  const q = new (d.camera.quaternion.constructor)();
  const m = new (d.camera.matrixWorld.constructor)();
  const up = V(0, 1, 0);
  const eye = s.position.clone();
  m.lookAt(V(0, 0, 0), tan.clone().negate(), up);
  q.setFromRotationMatrix(m);
  const gy = d.terrain.sampleHeight(eye.x, eye.z);
  eye.y = Math.max(eye.y, gy) + 0.05;
  try {
    if (typeof d.bike.reset === 'function') d.bike.reset({ position: eye, quaternion: q });
    const st = d.bike.state;
    if (st) {
      if (st.position && st.position.copy) st.position.copy(eye);
      if (st.quaternion && st.quaternion.copy) st.quaternion.copy(q);
      st.trailT = t;
      // a plausible mid-run speed so speed-driven effects and rider pose are not at rest
      st.speed = 11.5;
      if (st.velocity && st.velocity.copy) st.velocity.copy(tan).multiplyScalar(11.5);
    }
  } catch (e) { /* a stub bike must not stop the capture */ }
  return { t, pos: eye.toArray() };
}

/** Keep a camera position above the ground with clearance — never pose inside the hill. */
export function clampAboveTerrain(pos, clearance = 1.8) {
  const d = ctx();
  if (!d.terrain || !d.terrain.sampleHeight) return pos;
  const g = d.terrain.sampleHeight(pos.x, pos.z);
  if (Number.isFinite(g) && pos.y < g + clearance) pos.y = g + clearance;
  return pos;
}

/** Place the camera relative to a point on the trail. */
export function poseOnTrail(t, { back = 7, up = 2.6, side = 0, ahead = 14, fov = 64, pitch = 0, bike = true } = {}) {
  const d = ctx();
  if (bike) placeBike(t);
  const s = d.trail.sampleAt(t);
  const p = s.position;
  const tan = s.tangent.clone().normalize();
  const right = V(0, 1, 0).cross(tan).normalize().multiplyScalar(-1);
  d.camera.position.set(
    p.x - tan.x * back + right.x * side,
    p.y + up,
    p.z - tan.z * back + right.z * side,
  );
  clampAboveTerrain(d.camera.position, 1.8);
  const la = d.trail.sampleAt(Math.min(0.999, t + ahead / d.trail.length));
  d.camera.lookAt(la.position.x, la.position.y + 1.2 + pitch, la.position.z);
  if (d.camera.fov !== fov) { d.camera.fov = fov; d.camera.updateProjectionMatrix(); }
  return { t, cam: d.camera.position.toArray(), trailY: p.y };
}

/**
 * Drive the world systems (LOD, instancing, wind, clouds) without letting the bike or the
 * game state run. Needed because the QA camera teleports and because rAF is throttled
 * while the browser pane is not composited.
 */
export function tickWorld(n = 8, dt = 1 / 60) {
  const d = ctx();
  // bikeModel/rider are included so the bike and rider follow a QA-placed bike state.
  // `bike` itself is NOT ticked — its physics would drive it away from the shot.
  const WORLD = ['terrain', 'trail', 'sky', 'water', 'vegetation', 'particles',
                 'bikeModel', 'rider'];
  for (let i = 0; i < n; i++) {
    d.time += dt;
    d.frame++;
    for (const k of WORLD) {
      const s = d[k];
      if (s && typeof s.update === 'function') {
        try { s.update(dt, d); } catch (e) { /* one bad system must not stop the capture */ }
      }
    }
  }
}

/** Free camera at an absolute position looking at an absolute target. */
export function poseFree(pos, target, fov = 55) {
  const d = ctx();
  d.camera.position.set(pos[0], pos[1], pos[2]);
  d.camera.lookAt(target[0], target[1], target[2]);
  if (d.camera.fov !== fov) { d.camera.fov = fov; d.camera.updateProjectionMatrix(); }
}

/**
 * Render one frame at `w`x`h` and POST it to disk.
 * Renders and reads back inside a single task so preserveDrawingBuffer isn't needed.
 */
export async function shoot(name, { w = 1920, h = 1080, settle = 6 } = {}) {
  const d = ctx();
  const el = d.renderer.domElement;
  const prevW = el.width, prevH = el.height;
  const prevPR = d.renderer.getPixelRatio();

  // The engine has a frame-time pixel-ratio governor and main.js has a resize handler; when the
  // browser pane is not composited the container reports 0x0 and either will drag the backbuffer
  // to 1x1 mid-capture. Pin the size, suspend the governor, and re-assert immediately before
  // the readback.
  const gov = d.engine && typeof d.engine.setGovernorEnabled === 'function';
  if (gov) { try { d.engine.setGovernorEnabled(false); } catch (e) { /* older engine */ } }
  const pin = () => {
    d.renderer.setPixelRatio(1);
    d.renderer.setSize(w, h, false);
    d.camera.aspect = w / h;
    d.camera.updateProjectionMatrix();
    if (d.postfx && d.postfx.resize) d.postfx.resize(w, h);
  };
  pin();

  const draw = () => {
    if (d.postfx && d.postfx.render) d.postfx.render(1 / 60);
    else d.renderer.render(d.scene, d.camera);
  };
  // CRITICAL: the world systems must be ticked after a camera teleport or LOD/instance
  // tier selection is still packed for the OLD camera position — which shows up as an
  // empty forest. The page's own rAF loop is throttled when the tab is not composited,
  // so we drive the world systems here explicitly.
  tickWorld(12);
  // A few warm frames: TAA/adaptive exposure/LOD need to settle before the capture.
  pin();
  for (let i = 0; i < settle; i++) draw();
  pin();
  draw();
  const dataUrl = el.toDataURL('image/png');

  // Measure BEFORE restoring, or the telemetry describes the restored backbuffer rather than
  // the frame that was just captured.
  const capturedW = el.width, capturedH = el.height;
  // Build a sky mask: re-render the same frame with the sky dome and background hidden, so the
  // variance checks below can tell "flat shading on a surface" from "a cloudless sky".
  const skyMask = buildSkyMask(draw, el);
  const metrics = analyse(el, 320, 180, skyMask);
  if (capturedW !== w || capturedH !== h) {
    metrics.warnings.push(`WRONG_SIZE captured ${capturedW}x${capturedH}, expected ${w}x${h}`);
  }
  if (metrics.meanRGB[0] + metrics.meanRGB[1] + metrics.meanRGB[2] < 3) {
    metrics.warnings.push('DEGENERATE frame is essentially empty/black — do not review this shot');
  }

  // restore
  if (gov) { try { d.engine.setGovernorEnabled(true); } catch (e) { /* ignore */ } }
  d.renderer.setPixelRatio(prevPR);
  d.renderer.setSize(Math.max(1, prevW / prevPR), Math.max(1, prevH / prevPR), false);
  d.camera.aspect = prevW / Math.max(1, prevH);
  d.camera.updateProjectionMatrix();
  if (d.postfx && d.postfx.resize) d.postfx.resize(prevW, prevH);

  const r = await fetch('/qa/shot', {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ name, dataUrl }),
  });
  const j = await r.json();
  return { ...j, name, metrics };
}

let _probe = null, _maskCanvas = null;

/**
 * Render the frame once with the sky hidden against magenta, and return a boolean array marking
 * which downsampled pixels are sky. Without this, any "flat bright region" test is really just a
 * clear-sky detector — measured: 12.0% and 11.1% of two frames flagged, zero terrain pixels.
 */
export function buildSkyMask(draw, el, w = 320, h = 180) {
  const d = ctx();
  const scene = d.scene;
  let skyObj = null;
  scene.traverse((o) => { if (!skyObj && o.name === 'sky') skyObj = o; });
  const prevVis = skyObj ? skyObj.visible : null;
  const prevBg = scene.background;
  const prevFog = scene.fog;
  // Key colour: anything the scene cannot produce. Magenta with no green.
  const Color = (prevBg && prevBg.isColor) ? prevBg.constructor
              : (d.sun && d.sun.color && d.sun.color.constructor) || null;
  if (skyObj) skyObj.visible = false;
  scene.fog = null;                       // fog would tint the key toward the fog colour
  scene.background = Color ? new Color(1, 0, 1) : null;
  draw();
  if (!_maskCanvas) { _maskCanvas = document.createElement('canvas'); _maskCanvas.width = w; _maskCanvas.height = h; }
  const g = _maskCanvas.getContext('2d', { willReadFrequently: true });
  g.drawImage(el, 0, 0, w, h);
  const px = g.getImageData(0, 0, w, h).data;
  const mask = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) {
    const r = px[i * 4], gg = px[i * 4 + 1], b = px[i * 4 + 2];
    // magenta-dominant => nothing was drawn there => sky
    mask[i] = (r > 90 && b > 90 && gg < Math.min(r, b) * 0.65) ? 1 : 0;
  }
  if (skyObj) skyObj.visible = prevVis;
  scene.background = prevBg;
  scene.fog = prevFog;
  draw();                       // restore the real frame before the caller reads pixels
  return mask;
}
/**
 * Objective per-shot telemetry, so the review loop has measurements and not only opinions.
 * Catches the failure modes the round-2 review had to find by eye: crushed shadows, clipped
 * highlights, and large flat fills (camera underground, or a colourless below-horizon band).
 */
export function analyse(canvas, w = 320, h = 180, skyMask = null) {
  if (!_probe) {
    _probe = document.createElement('canvas');
    _probe.width = w; _probe.height = h;
  }
  const g = _probe.getContext('2d', { willReadFrequently: true });
  g.drawImage(canvas, 0, 0, w, h);
  const px = g.getImageData(0, 0, w, h).data;
  const n = w * h;
  const lum = new Float32Array(n);
  let sumR = 0, sumG = 0, sumB = 0;
  for (let i = 0; i < n; i++) {
    const r = px[i * 4], gg = px[i * 4 + 1], b = px[i * 4 + 2];
    lum[i] = 0.2126 * r + 0.7152 * gg + 0.0722 * b;
    sumR += r; sumG += gg; sumB += b;
  }
  const sorted = Float32Array.from(lum).sort();
  const median = sorted[n >> 1];
  const p01 = sorted[Math.floor(n * 0.01)], p99 = sorted[Math.floor(n * 0.99)];
  let crushed = 0, clipped = 0, flat = 0;
  for (let i = 0; i < n; i++) { if (lum[i] < 6) crushed++; if (lum[i] > 250) clipped++; }
  // local flatness: 3x3 luminance range below 1.5 => a fill, not a surface
  for (let y = 1; y < h - 1; y++) {
    for (let x = 1; x < w - 1; x++) {
      const c = y * w + x;
      let lo = 255, hi = 0;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        const v = lum[c + dy * w + dx];
        if (v < lo) lo = v; if (v > hi) hi = v;
      }
      if (hi - lo < 1.5) flat++;
    }
  }
  const flatFrac = flat / ((w - 2) * (h - 2));
  const warnings = [];
  if (flatFrac > 0.25) warnings.push(`FLAT_FILL ${(flatFrac * 100).toFixed(0)}% of frame is a featureless fill (camera underground, or a colourless sky/horizon band)`);
  if (crushed / n > 0.05) warnings.push(`CRUSHED ${((crushed / n) * 100).toFixed(1)}% of pixels below L=6 — no shadow detail`);
  if (clipped / n > 0.05) warnings.push(`CLIPPED ${((clipped / n) * 100).toFixed(1)}% of pixels above L=250`);
  if (median < 25) warnings.push(`DARK median luminance ${median.toFixed(0)}/255`);
  // The clipping metric got optimised into its own opposite: after the exposure fix NO pixel in
  // any shot exceeded L=242. Real sunlit photography clips somewhere — a set with no true white,
  // no sun glint and no specular hit is a different failure, not a pass.
  if (p99 < 200) warnings.push(`NO_HIGHLIGHTS p99 luminance only ${p99.toFixed(0)} — no specular hit or sun glint anywhere`);
  // "clipped 0.0%" is arithmetically true of a structureless white plane. Catch flat bright
  // regions by their lack of within-surface variance, not by their level.
  //
  // CORRECTION: the first version of this check flagged CLEAR SKY — measured at 12.0% and 11.1%
  // of two frames with zero terrain pixels in either. A cloudless sky is legitimately smooth, so
  // a variance test that includes it is just a sky detector. `mask` (below) is a render of the
  // same frame with the sky hidden; only pixels that contain geometry are eligible.
  {
    let brightFlat = 0, bright = 0;
    for (let y = 1; y < h - 1; y++) {
      for (let x = 1; x < w - 1; x++) {
        const c = y * w + x;
        if (lum[c] < 150) continue;
        if (skyMask && skyMask[c]) continue;   // sky pixel — not a surface, cannot be "flat shading"
        bright++;
        let lo = 255, hi = 0;
        for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
          const v = lum[c + dy * w + dx];
          if (v < lo) lo = v; if (v > hi) hi = v;
        }
        if (hi - lo < 2.5) brightFlat++;
      }
    }
    const frac = bright > 0 ? brightFlat / n : 0;
    if (frac > 0.10) warnings.push(`FLAT_BRIGHT ${(frac * 100).toFixed(0)}% of frame is a structureless bright plane (reads as a broken shader even though it is not clipped)`);
  }
  return {
    medianL: +median.toFixed(1), p01: +p01.toFixed(1), p99: +p99.toFixed(1),
    meanRGB: [sumR / n, sumG / n, sumB / n].map((v) => +v.toFixed(1)),
    crushedPct: +((crushed / n) * 100).toFixed(1),
    clippedPct: +((clipped / n) * 100).toFixed(1),
    flatPct: +(flatFrac * 100).toFixed(1),
    warnings,
  };
}

/** The standard review set — the shots every critic pass looks at. */
export const SHOTS = [
  ['01-start-gate',    { t: 0.005, back: 9,  up: 3.0, fov: 62 }],
  ['02-steep-tech',    { t: 0.14,  back: 6,  up: 2.2, fov: 68 }],
  ['03-berm-flow',     { t: 0.30,  back: 6.5,up: 2.0, fov: 70, side: 2.5 }],
  ['04-jump-line',     { t: 0.45,  back: 8,  up: 2.4, fov: 72 }],
  ['05-rock-chute',    { t: 0.57,  back: 6,  up: 2.6, fov: 66 }],
  ['06-creek',         { t: 0.66,  back: 7,  up: 1.8, fov: 66 }],
  ['07-forest-loam',   { t: 0.76,  back: 5.5,up: 2.0, fov: 70 }],
  ['08-rock-garden',   { t: 0.86,  back: 6,  up: 2.2, fov: 68 }],
  ['09-final-sprint',  { t: 0.96,  back: 8,  up: 2.6, fov: 74 }],
  ['10-riders-eye',    { t: 0.33,  back: 1.2,up: 1.55,fov: 84 }],
  ['11-low-wide',      { t: 0.50,  back: 12, up: 1.6, fov: 50 }],
  ['12-trailside',     { t: 0.22,  back: 4,  up: 1.4, fov: 58, side: 7 }],
  // The bike and rider get the most viewer scrutiny in a bike game — frame them properly.
  ['13-bike-side',     { t: 0.40,  back: 0.2, up: 1.1, fov: 42, side: 3.4, ahead: 3 }],
  ['14-rider-three-q', { t: 0.72,  back: 3.0, up: 1.7, fov: 46, side: 2.2, ahead: 4 }],
  ['15-bike-low-rear', { t: 0.58,  back: 3.2, up: 0.7, fov: 52, ahead: 5 }],
];

export async function runShotSet(tag = '') {
  const d = ctx();
  freezeCamera(true);
  const out = [];
  for (const [name, opts] of SHOTS) {
    const { t, ...rest } = opts;
    poseOnTrail(t, rest);
    const r = await shoot(tag ? `${tag}_${name}` : name);
    out.push({ name, ok: r.ok, metrics: r.metrics });
  }
  // one scenic wide of the whole mountain, from above the start
  const b = d.terrain.bounds;
  const s0 = d.trail.sampleAt(0.0).position, s1 = d.trail.sampleAt(1.0).position;
  poseFree([s0.x + 320, s0.y + 180, s0.z + 340], [s1.x, s1.y, s1.z], 48);
  out.push({ name: 'wide', ...(await shoot(tag ? `${tag}_00-establishing` : '00-establishing')) });
  freezeCamera(false);
  return out;
}

/** Honest frame-time measurement: many renders, then a GPU sync via a pixel readback. */
/**
 * Measure a REAL frame, not just the composer.
 *
 * The previous version called only postfx.render(), which excluded all seventeen module
 * update()/lateUpdate() calls that main.js runs every frame — physics, collision, vegetation
 * LOD, terrain quadtree, shadow refit, particles, audio, HUD, chase camera. It also held the
 * camera perfectly still, so vegetation's chunk re-pack (the documented expensive path) never
 * fired once, and never called postfx.update(), so the gated passes — including motion blur,
 * the most expensive per-pixel pass in the chain — were very likely OFF in the measurement and
 * ON in the real game. Every optimisation decision this project made was taken against that
 * number. This version drives the full frame with the bike actually moving.
 */
export async function measureRealFrame(frames = 90, w = 1920, h = 1080) {
  const d = ctx();
  const ORDER = ['input', 'terrain', 'trail', 'sky', 'water', 'vegetation', 'collision', 'bike',
                 'bikeModel', 'rider', 'particles', 'chaseCamera', 'audio', 'gameplay', 'hud',
                 'menu', 'postfx'];
  const prevPR = d.renderer.getPixelRatio();
  const prevW = d.renderer.domElement.width, prevH = d.renderer.domElement.height;
  const gov = d.engine && typeof d.engine.setGovernorEnabled === 'function';
  if (gov) { try { d.engine.setGovernorEnabled(false); } catch (e) {} }
  d.renderer.setPixelRatio(1);
  d.renderer.setSize(w, h, false);
  d.camera.aspect = w / h; d.camera.updateProjectionMatrix();
  if (d.postfx && d.postfx.resize) d.postfx.resize(w, h);
  freezeCamera(false);                      // let the real chase camera drive
  if (d.bike && d.bike.reset && d.trail) {
    try { d.bike.reset(d.trail.startTransform); } catch (e) {}
  }

  const gl = d.renderer.getContext();
  const px = new Uint8Array(4);
  const sync = () => gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px);
  const dt = 1 / 60;
  const step = () => {
    // a rider actually descending, so LOD/instancing/particles do their real work
    const b = d.bike && d.bike.state;
    if (b && d.trail && d.input) {
      const s = d.input.state;
      s.pedal = b.speed < 11 ? 0.6 : 0; s.brakeRear = b.speed > 17 ? 0.3 : 0;
      s.steer = 0; s.pitch = 0; s.roll = 0; s.pump = 0;
    }
    d.time += dt; d.frame++;
    for (const k of ORDER) { const s = d[k]; if (s && s.update) { try { s.update(dt, d); } catch (e) {} } }
    for (const k of ORDER) { const s = d[k]; if (s && s.lateUpdate) { try { s.lateUpdate(dt, d); } catch (e) {} } }
    if (d.postfx && d.postfx.render) d.postfx.render(dt); else d.renderer.render(d.scene, d.camera);
  };
  for (let i = 0; i < 30; i++) step();       // warm: let LOD settle and the bike get moving
  sync();
  d.renderer.info.autoReset = false; d.renderer.info.reset();
  const t0 = performance.now();
  for (let i = 0; i < frames; i++) step();
  sync();
  const ms = (performance.now() - t0) / frames;
  const out = {
    mode: 'REAL FRAME (all systems + moving bike)',
    msPerFrame: +ms.toFixed(2), fps: +(1000 / ms).toFixed(1),
    renderPixels: w * h,
    drawCalls: Math.round(d.renderer.info.render.calls / frames),
    triangles: Math.round(d.renderer.info.render.triangles / frames),
    bikeSpeedKmh: d.bike && d.bike.state ? +(d.bike.state.speed * 3.6).toFixed(1) : null,
    shipping: shippingEstimate(w, h),
  };
  d.renderer.info.autoReset = true;
  if (gov) { try { d.engine.setGovernorEnabled(true); } catch (e) {} }
  d.renderer.setPixelRatio(prevPR);
  d.renderer.setSize(Math.max(1, prevW / prevPR), Math.max(1, prevH / prevPR), false);
  return out;
}

/**
 * What the SHIPPING path would actually render at this CSS size.
 * Reads the engine's own effective ratio rather than duplicating its budget table — the
 * previous version hardcoded a copy of PIXEL_BUDGET that the engine later changed
 * (9.2e6 -> 4.20e6 at `high`), so it reported a ratio ~1.48x too high, i.e. ~2.2x too few pixels.
 */
export function shippingEstimate(w = 1920, h = 1080) {
  const d = ctx();
  const st = (d.engine && d.engine.stats) || {};
  const live = (d.engine && d.engine.pixelRatio) || st.pixelRatio || null;
  return {
    engineEffectivePixelRatio: live,
    engineReportedMPix: st.renderMPix != null ? st.renderMPix : null,
    engineDrawSize: (st.drawWidth && st.drawHeight) ? [st.drawWidth, st.drawHeight] : null,
    note: 'read from engine.stats — do not duplicate engine PIXEL_BUDGET here',
  };
}

export async function measure(frames = 60, w = 1920, h = 1080) {
  const d = ctx();
  // Pin the backbuffer. Without this, measure() runs at whatever the live canvas happens to
  // be — and on a Retina panel engine.js's pixel budget resolves DPR to 2, so the number
  // would describe an 8.3 MPix image while the review shots are 2.07 MPix.
  const prevPR = d.renderer.getPixelRatio();
  const prevW = d.renderer.domElement.width, prevH = d.renderer.domElement.height;
  d.renderer.setPixelRatio(1);
  d.renderer.setSize(w, h, false);
  d.camera.aspect = w / h;
  d.camera.updateProjectionMatrix();
  if (d.postfx && d.postfx.resize) d.postfx.resize(w, h);
  const gl = d.renderer.getContext();
  const px = new Uint8Array(4);
  const draw = () => {
    if (d.postfx && d.postfx.render) d.postfx.render(1 / 60);
    else d.renderer.render(d.scene, d.camera);
  };
  const sync = () => { gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px); };
  draw(); sync();
  const t0 = performance.now();
  for (let i = 0; i < frames; i++) draw();
  sync();
  const ms = (performance.now() - t0) / frames;
  d.renderer.info.autoReset = false;
  d.renderer.info.reset();
  draw(); sync();
  const out = {
    renderPixels: w * h,
    cssSize: [d.container.clientWidth, d.container.clientHeight],
    shipping: shippingEstimate(w, h),
    WARNING: 'composer-only timing — excludes all module update()/lateUpdate() and holds the ' +
             'camera static. Use measureRealFrame() for a number that means anything.',
    msPerFrame: +ms.toFixed(2),
    fps: +(1000 / ms).toFixed(1),
    drawCalls: d.renderer.info.render.calls,
    triangles: d.renderer.info.render.triangles,
    programs: d.renderer.info.programs.length,
    textures: d.renderer.info.memory.textures,
    geometries: d.renderer.info.memory.geometries,
    size: [d.renderer.domElement.width, d.renderer.domElement.height],
  };
  d.renderer.info.autoReset = true;
  // restore whatever the page was using
  d.renderer.setPixelRatio(prevPR);
  d.renderer.setSize(prevW / prevPR, prevH / prevPR, false);
  d.camera.aspect = prevW / prevH;
  d.camera.updateProjectionMatrix();
  if (d.postfx && d.postfx.resize) d.postfx.resize(prevW, prevH);
  return out;
}

export function health() {
  const d = ctx();
  const b = d.bike && d.bike.state;
  return {
    errors: window.__DESCENT_ERRORS__ || [],
    frame: d.frame,
    modules: ['terrain','trail','sky','water','vegetation','collision','bike','bikeModel',
              'rider','particles','chaseCamera','audio','gameplay','hud','menu','postfx']
      .reduce((a, k) => (a[k] = !!d[k], a), {}),
    bike: b ? {
      pos: b.position && b.position.toArray().map(n => +n.toFixed(2)),
      speed: +(b.speed || 0).toFixed(2),
      airborne: !!b.airborne, crashed: !!b.crashed,
      surface: b.surface, trailT: +(b.trailT || 0).toFixed(4),
      nan: !!(b.position && [b.position.x, b.position.y, b.position.z].some(Number.isNaN)),
    } : null,
  };
}

window.__QA__ = { freezeCamera, poseOnTrail, poseFree, shoot, runShotSet, measure, health, SHOTS };
