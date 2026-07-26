// engine.js — the render device. Owns the WebGL context, the scene root, the camera,
// colour management, the shadow configuration and viewport/quality plumbing.
// Sets ctx.renderer / ctx.scene / ctx.camera per CONTRACT §2.
//
// Colour policy (CONTRACT §0): THREE.ColorManagement is ON, the renderer outputs sRGB and
// tonemaps with ACES Filmic at exposure ~1.0. Therefore *every* colour and albedo texture
// authored elsewhere must be tagged SRGBColorSpace, and every data map (normal / rough /
// AO / height / splat) must be NoColorSpace. Getting that wrong is the single most common
// cause of a washed-out, "web-demo" looking frame.
//
// CONTRACT-NOTE: the contract does not name an owner for renderer-level tone mapping when
// postfx runs its own ACES pass. This module therefore exposes
// `engine.setToneMappingEnabled(false)` — postfx should call it during init so ACES is not
// applied twice. `renderer.toneMappingExposure` is left populated with the authored
// exposure even while renderer-side tonemapping is off, so postfx can read it as the
// scene's intended exposure.
//
// CONTRACT-NOTE: `ctx.settings.shadowMapSize` is consumed by whoever creates the lights
// (sky.js). The renderer has no such property, so `engine.applyShadowSettings()` walks
// ctx.scene and re-sizes every light shadow it finds, and re-runs on 'quality:changed'.
// Light modules should simply read ctx.settings.shadowMapSize when they build.
//
// CONTRACT-NOTE: logarithmicDepthBuffer is OFF by design (it breaks the pmndrs depth-based
// effects), and three 0.185's `reversedDepthBuffer` is also OFF: N8AO / DoF / motion blur
// all linearise the depth texture assuming a conventional [0,1] near→far range, and a
// reversed range silently corrupts them. Depth precision is instead bought with a tight
// near plane budget (0.1 / 8000 ≈ 0.6 m of depth resolution at 1 km, ~9 m at 4 km — fine
// for terrain, and distant geometry is fogged out by sky.js long before that matters).

import * as THREE from 'three';
import { clamp } from './rng.js';

// ---------------------------------------------------------------------------
// Module-scope scratch — no per-frame allocation anywhere below.
// ---------------------------------------------------------------------------
const _size = new THREE.Vector2();
const _drawSize = new THREE.Vector2();

const MIN_PIXEL_RATIO = 0.5;
const MAX_PIXEL_RATIO = 2.0;

// Device-pixel budget per quality tier. `ctx.settings.pixelRatio` is authored as
// min(devicePixelRatio, 2) with no knowledge of the display size, so on a 5K panel it
// would ask for ~33 M pixels and tank the frame. We clamp the *effective* ratio so the
// backbuffer never exceeds this many device pixels.
//
// R3/K2 — these were 3.2 / 6.0 / 9.2 / 13.5 M, which is far too generous. At the tier
// that matters, `high`, a 1920×1080 CSS window (2.0736 MPix) on any Retina panel resolved
// the effective ratio to the full 2.0 and rendered an **8.29 MPix** backbuffer — 4× the
// image the review screenshots were captured at, so the shipping frame rate at default
// settings was nothing like the measured one. The budgets below are keyed to an explicit
// effective ratio at 1080p, which is the honest way to author them:
//
//   low    1.50e6 → DPR 0.85  (sub-native, SMAA cleans the upscale)
//   medium 2.70e6 → DPR 1.14
//   high   4.20e6 → DPR 1.42  (all the extra sharpness SMAA at high can actually resolve;
//                              beyond ~1.4 the SMAA blend weights are already sub-pixel)
//   ultra  6.20e6 → DPR 1.73  (ultra also pays for MSAA + alphaToCoverage)
//
// (The 1/16 snap below rounds to the nearest grid cell, so the applied ratio can sit up
// to half a cell above the budget-derived one — 1.4375 rather than 1.423 at `high`, a
// 1.9% overshoot. That is deliberate: the budget is a target, the snap grid protects the
// postfx target allocations, and the governor is what enforces the real ceiling.)
//
// The tier-to-tier ratios of the old table (0.35 / 0.65 / 1 / 1.47 against `high`) are
// preserved, so nothing that keyed off the *relative* spacing of the tiers changes.
// Backbuffer size is additionally governed at runtime by frame time — see the ratio
// governor in §5. A 1920×1080 @1x display still never engages the budget at any tier.
const PIXEL_BUDGET = {
  low: 1.50e6,
  medium: 2.70e6,
  high: 4.20e6,
  ultra: 6.20e6,
};

// --- frame-time ratio governor (R3/K2) -------------------------------------
// A static budget cannot know what the GPU is. These sample the real frame time and
// shed backbuffer resolution one snap step (1/16) at a time when the machine cannot
// hold the target, recovering it when the load drops. The shed/recover thresholds are
// deliberately far apart, a decision consumes a whole fresh window, and recovery needs
// several consecutive good windows (a count that only ever grows) — so it cannot pump.
const GOV_WINDOW = 30;              // frames per decision window
const GOV_SHED_MS = 15.0;           // mean above this → shed a step (≈66 fps)
const GOV_RECOVER_MS = 12.5;        // mean below this → a step back (≈80 fps); 2.5 ms band
const GOV_STEP = 1 / 16;            // exactly one snap increment of the ratio grid
const GOV_MAX_TRIM = 0.75;          // never shed more than 0.75 of ratio in total
const GOV_RECOVER_WINDOWS = 2;      // consecutive good windows needed for the first recovery
const GOV_RECOVER_WINDOWS_MAX = 8;  // …rising by one per shed, so oscillation dies out

const CAMERA_FOV = 62;     // chase camera drives this at runtime (speed FOV kick)
const CAMERA_NEAR = 0.1;
const CAMERA_FAR = 8000;

const OVERLAY_STYLE_ID = 'descent-engine-overlay-style';

// ---------------------------------------------------------------------------
// Capability detection
// ---------------------------------------------------------------------------

/**
 * Probe for a usable WebGL2 context without keeping it alive.
 * three 0.185 is WebGL2-only, so this is a hard requirement, not a nicety.
 * @returns {{ ok: boolean, reason: string, renderer: string }}
 */
function detectWebGL2() {
  if (typeof WebGL2RenderingContext === 'undefined') {
    return { ok: false, reason: 'This browser has no WebGL 2 implementation.', renderer: '' };
  }
  let canvas = null;
  let gl = null;
  try {
    canvas = document.createElement('canvas');
    gl = canvas.getContext('webgl2', { failIfMajorPerformanceCaveat: false });
  } catch (err) {
    return { ok: false, reason: `Creating a WebGL 2 context threw: ${err && err.message}`, renderer: '' };
  }
  if (!gl) {
    return {
      ok: false,
      reason: 'A WebGL 2 context could not be created. Hardware acceleration may be disabled.',
      renderer: '',
    };
  }
  let name = '';
  try {
    const ext = gl.getExtension('WEBGL_debug_renderer_info');
    if (ext) name = String(gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) || '');
  } catch (err) { /* privacy-restricted; not important */ }
  // Release the probe context immediately — contexts are a scarce resource.
  try { gl.getExtension('WEBGL_lose_context')?.loseContext(); } catch (err) { /* ignore */ }
  canvas = null;
  return { ok: true, reason: '', renderer: name };
}

// ---------------------------------------------------------------------------
// On-screen messaging (unsupported hardware / context loss)
// ---------------------------------------------------------------------------

function ensureOverlayStyles() {
  if (document.getElementById(OVERLAY_STYLE_ID)) return;
  const style = document.createElement('style');
  style.id = OVERLAY_STYLE_ID;
  style.textContent = `
.descent-overlay {
  position: fixed; inset: 0; z-index: 10000;
  display: flex; flex-direction: column; align-items: center; justify-content: center;
  gap: 18px; padding: 8vmin; box-sizing: border-box; text-align: center;
  background:
    radial-gradient(120% 90% at 50% 8%, #16202e 0%, #0a0e15 55%, #05070b 100%);
  color: #e8edf3;
  font: 400 15px/1.6 ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif;
  -webkit-font-smoothing: antialiased;
  letter-spacing: 0.01em;
}
.descent-overlay__mark {
  font: 700 13px/1 ui-sans-serif, system-ui, sans-serif;
  letter-spacing: 0.52em; text-indent: 0.52em;
  color: #7d93ad; text-transform: uppercase;
}
.descent-overlay__rule {
  width: 56px; height: 1px; background: linear-gradient(90deg, transparent, #3d5570, transparent);
}
.descent-overlay__title {
  font: 600 clamp(20px, 3.4vw, 30px)/1.2 ui-sans-serif, system-ui, sans-serif;
  letter-spacing: -0.015em; color: #f4f7fa; margin: 0;
}
.descent-overlay__body { max-width: 54ch; color: #9fb0c4; margin: 0; }
.descent-overlay__list {
  margin: 4px 0 0; padding: 0; list-style: none;
  color: #7f93a9; font: 400 13px/1.9 ui-monospace, SFMono-Regular, Menlo, monospace;
}
.descent-overlay__list li::before { content: '— '; color: #4c637d; }
.descent-overlay--toast {
  inset: auto 0 32px 0; background: none; z-index: 10001;
  pointer-events: none; padding: 0;
}
.descent-overlay--toast .descent-overlay__chip {
  display: inline-flex; align-items: center; gap: 10px;
  padding: 10px 18px; border-radius: 999px;
  background: rgba(9, 13, 20, 0.78); border: 1px solid rgba(125, 147, 173, 0.28);
  backdrop-filter: blur(14px) saturate(1.2);
  font: 500 13px/1 ui-sans-serif, system-ui, sans-serif; letter-spacing: 0.02em;
  color: #dbe5ef; box-shadow: 0 12px 40px rgba(0, 0, 0, 0.5);
}
.descent-overlay__dot {
  width: 7px; height: 7px; border-radius: 50%; background: #ffb454;
  box-shadow: 0 0 12px #ffb454; animation: descent-pulse 1.1s ease-in-out infinite;
}
@keyframes descent-pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.25; } }
`;
  document.head.appendChild(style);
}

/**
 * Full-screen "we cannot run here" panel. Deliberately styled — a raw browser error
 * page is part of the "web demo" look we are avoiding.
 */
function showFatalOverlay(container, title, body, hints) {
  ensureOverlayStyles();
  const host = container && container.ownerDocument ? container : document.body;
  const el = document.createElement('div');
  el.className = 'descent-overlay';
  el.setAttribute('role', 'alert');

  const mark = document.createElement('div');
  mark.className = 'descent-overlay__mark';
  mark.textContent = 'Descent';

  const rule = document.createElement('div');
  rule.className = 'descent-overlay__rule';

  const h = document.createElement('h1');
  h.className = 'descent-overlay__title';
  h.textContent = title;

  const p = document.createElement('p');
  p.className = 'descent-overlay__body';
  p.textContent = body;

  el.appendChild(mark);
  el.appendChild(rule);
  el.appendChild(h);
  el.appendChild(p);

  if (hints && hints.length) {
    const ul = document.createElement('ul');
    ul.className = 'descent-overlay__list';
    for (const hint of hints) {
      const li = document.createElement('li');
      li.textContent = hint;
      ul.appendChild(li);
    }
    el.appendChild(ul);
  }

  (host.parentNode ? host.parentNode : document.body).appendChild(el);
  return el;
}

/** Small bottom-centre chip used while the GL context is being restored. */
function makeToast(text) {
  ensureOverlayStyles();
  const el = document.createElement('div');
  el.className = 'descent-overlay descent-overlay--toast';
  const chip = document.createElement('div');
  chip.className = 'descent-overlay__chip';
  const dot = document.createElement('span');
  dot.className = 'descent-overlay__dot';
  const label = document.createElement('span');
  label.textContent = text;
  chip.appendChild(dot);
  chip.appendChild(label);
  el.appendChild(chip);
  return el;
}

// ---------------------------------------------------------------------------
// Fallback sky
// ---------------------------------------------------------------------------

/**
 * A procedurally-painted equirectangular gradient used as the scene background and as a
 * baseline IBL until sky.js installs the real Rayleigh/Mie sky. Golden-hour palette per
 * CONTRACT §6, so a screenshot taken with no other module finished still reads as a
 * mountain evening rather than a grey void.
 *
 * Painted in sRGB (canvas is sRGB by definition) and tagged as such.
 */
function makeFallbackSkyTexture() {
  const W = 512;
  const H = 256;
  const canvas = document.createElement('canvas');
  canvas.width = W;
  canvas.height = H;
  const g = canvas.getContext('2d');

  // v=1 (canvas row 0) is the zenith once flipY is applied, so paint top-down.
  const grad = g.createLinearGradient(0, 0, 0, H);
  grad.addColorStop(0.00, '#17324f'); // zenith, deep evening blue
  grad.addColorStop(0.22, '#2c5578');
  grad.addColorStop(0.40, '#6b90ac');
  grad.addColorStop(0.47, '#bcb4a4'); // haze band just above the horizon
  grad.addColorStop(0.50, '#d8bc94'); // horizon, warm
  grad.addColorStop(0.53, '#6d6154'); // ground haze
  grad.addColorStop(0.72, '#3c372f');
  grad.addColorStop(1.00, '#221f1b'); // nadir
  g.fillStyle = grad;
  g.fillRect(0, 0, W, H);

  // Warm scattering lobe around where sky.js will put the low sun.
  const sunU = 0.62;
  const sunV = 0.435;
  g.globalCompositeOperation = 'lighter';
  const glow = g.createRadialGradient(sunU * W, sunV * H, 0, sunU * W, sunV * H, H * 0.62);
  glow.addColorStop(0.00, 'rgba(255, 226, 176, 0.85)');
  glow.addColorStop(0.14, 'rgba(255, 196, 132, 0.36)');
  glow.addColorStop(0.42, 'rgba(196, 150, 110, 0.13)');
  glow.addColorStop(1.00, 'rgba(120, 130, 150, 0.0)');
  g.fillStyle = glow;
  g.fillRect(0, 0, W, H);

  // Opposite horizon gets a faint cool counter-glow so the sphere is not symmetric.
  const anti = g.createRadialGradient((sunU + 0.5) % 1 * W, sunV * H, 0, (sunU + 0.5) % 1 * W, sunV * H, H * 0.45);
  anti.addColorStop(0.0, 'rgba(120, 150, 190, 0.16)');
  anti.addColorStop(1.0, 'rgba(120, 150, 190, 0.0)');
  g.fillStyle = anti;
  g.fillRect(0, 0, W, H);
  g.globalCompositeOperation = 'source-over';

  const tex = new THREE.CanvasTexture(canvas);
  tex.name = 'engine.fallbackSky';
  tex.mapping = THREE.EquirectangularReflectionMapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.minFilter = THREE.LinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.generateMipmaps = false;
  tex.needsUpdate = true;
  return tex;
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

export function createEngine(ctx) {
  const container = ctx.container || document.body;
  const settings = ctx.settings;

  // --- 0. hardware check -------------------------------------------------
  const caps = detectWebGL2();
  if (!caps.ok) {
    showFatalOverlay(
      container,
      'This device cannot run DESCENT',
      `${caps.reason} DESCENT needs WebGL 2, which every current desktop browser supports when hardware acceleration is switched on.`,
      [
        'Chrome / Edge: Settings → System → "Use graphics acceleration when available"',
        'Safari: Develop → Experimental Features → WebGL 2.0',
        'Firefox: about:config → webgl.force-enabled = true',
        'Check chrome://gpu for a blocklisted driver',
      ],
    );
    throw new Error(`[engine] WebGL2 unavailable: ${caps.reason}`);
  }

  // --- 1. colour management ----------------------------------------------
  // Must be set before any Color/Texture is constructed by another module.
  THREE.ColorManagement.enabled = true;

  // --- 2. renderer --------------------------------------------------------
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({
      // SMAA runs in postfx; MSAA on the default framebuffer would be paid for and then
      // thrown away, because the composer renders the scene into its own targets.
      antialias: false,
      alpha: false,
      // No stencil work anywhere in the pipeline — dropping it lets the driver pick a
      // packed 24-bit depth format instead of D24S8 on some GPUs.
      stencil: false,
      depth: true,
      powerPreference: 'high-performance',
      premultipliedAlpha: true,
      // Screenshots go through the QA harness's own readback path; keeping the drawing
      // buffer alive costs bandwidth every frame for nothing.
      preserveDrawingBuffer: false,
      failIfMajorPerformanceCaveat: false,
      // See the CONTRACT-NOTE at the top: reversed depth breaks pmndrs depth effects.
      reversedDepthBuffer: false,
    });
  } catch (err) {
    showFatalOverlay(
      container,
      'The graphics context could not be created',
      'The browser reported WebGL 2 support but refused to create a rendering context. This usually means the GPU process crashed or the driver is blocklisted.',
      [String((err && err.message) || err)],
    );
    throw err;
  }

  const canvas = renderer.domElement;
  canvas.style.display = 'block';
  canvas.style.width = '100%';
  canvas.style.height = '100%';
  canvas.style.outline = 'none';
  canvas.style.touchAction = 'none';   // stop mobile scroll-gesture stealing input
  canvas.tabIndex = 0;                 // keyboard focus target for input.js
  canvas.setAttribute('aria-label', 'DESCENT game viewport');
  container.appendChild(canvas);

  // Output + tonemapping. Kept in local state so setToneMappingEnabled can flip the
  // renderer side off without losing the authored values.
  let toneMappingMode = THREE.ACESFilmicToneMapping;
  let toneMappingEnabled = true;
  let exposure = 1.0;

  function applyToneMapping() {
    renderer.toneMapping = toneMappingEnabled ? toneMappingMode : THREE.NoToneMapping;
    // Always publish the authored exposure — three ignores it under NoToneMapping, and
    // postfx reads it as "the exposure the scene was lit for".
    renderer.toneMappingExposure = exposure;
  }

  renderer.outputColorSpace = THREE.SRGBColorSpace;
  applyToneMapping();

  // three >= r165 is physically-lit by default; the guard keeps the intent explicit and
  // is harmless if the property has been removed.
  if ('useLegacyLights' in renderer) renderer.useLegacyLights = false;

  renderer.autoClear = true;
  renderer.setClearColor(0x0b1220, 1);
  renderer.sortObjects = true;
  renderer.info.autoReset = false;   // we aggregate across all postfx passes ourselves
  renderer.debug.checkShaderErrors = true; // many hand-written shaders in this project

  // Shadows (CONTRACT §9 / brief): PCF, auto-updating, size from settings.
  //
  // R3/K3 — this was PCFSoftShadowMap. PCFSoft derives its filter radius per fragment
  // from screen-space derivatives of the shadow coordinate; at the texel-snapped ~0.15 m
  // density sky.js fits, that radius lands *below one texel* over the whole gameplay
  // range, so the 5×5 soft kernel degenerates to the same tap pattern as plain PCF. The
  // two filters were indistinguishable in all 16 review shots. PCFShadowMap does the same
  // 2×2-tap-per-corner work without the derivative maths and the extra fetches, and the
  // recovered time is better spent on `ctx.settings.shadowMapSize` — which is authored in
  // ctx.js and consumed by sky.js (see the CONTRACT-NOTE above), both outside this file.
  renderer.shadowMap.enabled = settings.shadows !== false;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  renderer.shadowMap.autoUpdate = true;

  // Anisotropy default: terrain and trail tread are viewed at grazing angles almost
  // every frame, and isotropic filtering turns them to mush at distance. 8 is the
  // sweet spot on Apple silicon; going to 16 costs bandwidth for little gain.
  const maxAnisotropy = renderer.capabilities.getMaxAnisotropy();
  THREE.Texture.DEFAULT_ANISOTROPY = Math.min(8, maxAnisotropy || 1);

  // --- 3. scene -----------------------------------------------------------
  const scene = new THREE.Scene();
  scene.name = 'DESCENT';

  const fallbackSky = makeFallbackSkyTexture();
  scene.background = fallbackSky;
  scene.backgroundIntensity = 1.0;

  // Baseline IBL so PBR materials are lit before sky.js runs its own PMREM. Without
  // this, every metal reads pure black in an early screenshot. One-off cost, ~3 ms.
  let fallbackEnv = null;
  try {
    const pmrem = new THREE.PMREMGenerator(renderer);
    pmrem.compileEquirectangularShader();
    fallbackEnv = pmrem.fromEquirectangular(fallbackSky).texture;
    fallbackEnv.name = 'engine.fallbackEnv';
    scene.environment = fallbackEnv;
    scene.environmentIntensity = 1.0;
    pmrem.dispose();
  } catch (err) {
    console.warn('[engine] fallback environment map failed; materials may read dark', err);
  }

  // --- 4. camera ----------------------------------------------------------
  const startW = Math.max(1, container.clientWidth || window.innerWidth || 1);
  const startH = Math.max(1, container.clientHeight || window.innerHeight || 1);

  const camera = new THREE.PerspectiveCamera(CAMERA_FOV, startW / startH, CAMERA_NEAR, CAMERA_FAR);
  camera.name = 'chaseCamera';
  // A sane default pose so the very first frame (before chaseCamera exists) looks down
  // the -Z fall line rather than into the ground.
  camera.position.set(0, 4.2, 7.5);
  camera.lookAt(0, 1.4, -6);
  camera.updateProjectionMatrix();
  // Adding the camera to the scene means anything parented to it (audio listener,
  // first-person rider parts, screen-space props) is transformed correctly.
  scene.add(camera);

  ctx.renderer = renderer;
  ctx.scene = scene;
  ctx.camera = camera;

  // --- 5. pixel ratio / sizing -------------------------------------------
  let viewW = startW;
  let viewH = startH;
  let appliedPixelRatio = 1;

  // Governor state. All module-lifetime; the sample ring is a typed array so the
  // per-frame path below allocates nothing.
  const govSamples = new Float64Array(GOV_WINDOW);
  let govFill = 0;
  let govIdx = 0;
  let govSum = 0;
  let govTrim = 0;                              // ratio subtracted by the governor
  let govRecoverStreak = 0;
  let govRecoverNeeded = GOV_RECOVER_WINDOWS;
  let govEnabled = true;
  let govPending = false;
  let govMeanMs = 0;
  let disposed = false;

  function govClearWindow() {
    govSamples.fill(0);
    govFill = 0;
    govIdx = 0;
    govSum = 0;
  }

  /**
   * Clamp the requested pixel ratio into [MIN, MAX], then against the quality tier's
   * device-pixel budget (so a huge display cannot silently blow the frame time out),
   * then against whatever the frame-time governor has decided to shed.
   */
  function effectivePixelRatio(w, h) {
    const deviceMax = Math.min(window.devicePixelRatio || 1, MAX_PIXEL_RATIO);
    let pr = clamp(settings.pixelRatio || 1, MIN_PIXEL_RATIO, Math.max(MIN_PIXEL_RATIO, deviceMax));
    const budget = PIXEL_BUDGET[ctx.quality] || PIXEL_BUDGET.high;
    const area = Math.max(1, w * h);
    const maxRatio = Math.sqrt(budget / area);
    if (pr > maxRatio) pr = Math.max(MIN_PIXEL_RATIO, maxRatio);
    // Runtime governor trim. Applied last so it composes with, rather than fights, the
    // authored ratio and the tier budget.
    if (govTrim > 0) pr = Math.max(MIN_PIXEL_RATIO, pr - govTrim);
    // Snap to 1/16ths: micro-jitter in the ratio forces every render target in the
    // postfx chain to be reallocated, which stutters. GOV_STEP is exactly one such
    // increment, so a governor step always moves the applied ratio by one grid cell.
    return Math.round(pr * 16) / 16;
  }

  function applySize(w, h, notify) {
    viewW = Math.max(1, Math.floor(w));
    viewH = Math.max(1, Math.floor(h));

    const pr = effectivePixelRatio(viewW, viewH);
    const ratioChanged = Math.abs(pr - appliedPixelRatio) > 1e-4;
    appliedPixelRatio = pr;

    if (ratioChanged) renderer.setPixelRatio(pr);
    // updateStyle=false: the canvas is already stretched to the container by CSS, and
    // letting three write inline px sizes fights the layout on DPR changes.
    renderer.setSize(viewW, viewH, false);
    canvas.style.width = '100%';
    canvas.style.height = '100%';

    camera.aspect = viewW / viewH;
    camera.updateProjectionMatrix();

    if (notify && ctx.events) {
      ctx.events.emit('engine:resize', { width: viewW, height: viewH, pixelRatio: pr });
    }
    return ratioChanged;
  }

  applySize(startW, startH, false);

  /**
   * Re-apply the size with the governor's new trim. Deliberately deferred out of the
   * render path: the frame-time sample is taken from inside the guarded `render()`,
   * which on a postfx frame is a call made *by the composer mid-chain* — resizing its
   * targets from there would resize between two passes of the same frame. A rAF
   * scheduled during frame N runs after the already-registered tick for frame N+1, i.e.
   * at a real frame boundary.
   */
  function applyGovernorNow() {
    govPending = false;
    if (disposed || contextLost) return;
    const changed = applySize(viewW, viewH, true);
    if (changed && ctx.postfx && typeof ctx.postfx.resize === 'function') {
      try { ctx.postfx.resize(viewW, viewH); } catch (err) { /* postfx logs its own */ }
    }
    if (ctx.debug && ctx.debug.enabled) {
      ctx.debug.log('engine.governor', {
        meanMs: +govMeanMs.toFixed(2),
        trim: govTrim,
        pixelRatio: appliedPixelRatio,
        mpix: +((viewW * appliedPixelRatio) * (viewH * appliedPixelRatio) / 1e6).toFixed(2),
      });
    }
  }

  function scheduleGovernorApply() {
    if (govPending) return;
    govPending = true;
    if (typeof requestAnimationFrame === 'function') requestAnimationFrame(applyGovernorNow);
    else setTimeout(applyGovernorNow, 0);
  }

  /**
   * One frame-time sample. Sheds 1/16 of the pixel ratio when the 30-frame mean exceeds
   * GOV_SHED_MS and gives it back when the mean falls below GOV_RECOVER_MS, with three
   * independent anti-oscillation guards: a 2.5 ms dead band between the two thresholds,
   * a full fresh window after every decision (so a change is always judged on frames
   * rendered at the new resolution), and a consecutive-good-window requirement for
   * recovery that increases by one on every shed and is never reduced.
   * No allocation.
   */
  function sampleGovernor(deltaMs) {
    if (!govEnabled || contextLost) return;
    // Reject the first frame, hitches, tab-switch gaps and debugger pauses — a governor
    // that reacts to a 400 ms stall would drop three steps for one GC.
    if (!(deltaMs > 0.5) || deltaMs > 120) return;
    // If something else has pinned the backbuffer (the QA harness's shoot()/measure()
    // both do exactly that), stay out of its way entirely.
    if (Math.abs(renderer.getPixelRatio() - appliedPixelRatio) > 1e-4) return;

    govSum += deltaMs - govSamples[govIdx];
    govSamples[govIdx] = deltaMs;
    govIdx = (govIdx + 1) % GOV_WINDOW;
    if (govFill < GOV_WINDOW) { govFill++; return; }

    govMeanMs = govSum / GOV_WINDOW;
    stats.frameMeanMs = govMeanMs;

    if (govMeanMs > GOV_SHED_MS) {
      if (govTrim < GOV_MAX_TRIM - 1e-6) {
        govTrim = Math.min(GOV_MAX_TRIM, govTrim + GOV_STEP);
        govRecoverNeeded = Math.min(GOV_RECOVER_WINDOWS_MAX, govRecoverNeeded + 1);
        scheduleGovernorApply();
      }
      govRecoverStreak = 0;
      govClearWindow();
      return;
    }

    if (govMeanMs < GOV_RECOVER_MS && govTrim > 0) {
      govRecoverStreak++;
      if (govRecoverStreak >= govRecoverNeeded) {
        govTrim = Math.max(0, govTrim - GOV_STEP);
        govRecoverStreak = 0;
        scheduleGovernorApply();
      }
      govClearWindow();
      return;
    }

    // In the dead band (or already at full resolution): hold, and start a fresh window.
    govRecoverStreak = 0;
    govClearWindow();
  }

  // A DPR change (dragging the window between a Retina and a 1x display) fires no
  // resize event on some browsers, so watch the media query too.
  let dprQuery = null;
  const onDprChange = () => {
    const w = container.clientWidth || window.innerWidth || viewW;
    const h = container.clientHeight || window.innerHeight || viewH;
    api.resize(w, h);
    watchDpr();
  };
  function watchDpr() {
    if (dprQuery && dprQuery.removeEventListener) dprQuery.removeEventListener('change', onDprChange);
    if (typeof window.matchMedia !== 'function') return;
    const dpr = window.devicePixelRatio || 1;
    dprQuery = window.matchMedia(`(resolution: ${dpr}dppx)`);
    if (dprQuery.addEventListener) dprQuery.addEventListener('change', onDprChange);
  }
  watchDpr();

  // --- 6. shadow settings live-apply -------------------------------------
  /**
   * Push ctx.settings.shadows / shadowMapSize onto the renderer and onto every light
   * currently in the scene. Disposing shadow.map forces three to reallocate at the new
   * resolution on the next shadow pass.
   */
  function applyShadowSettings() {
    const wanted = settings.shadows !== false;
    if (renderer.shadowMap.enabled !== wanted) {
      renderer.shadowMap.enabled = wanted;
      renderer.shadowMap.needsUpdate = true;
    }
    const s = Math.max(256, Math.min(8192, settings.shadowMapSize | 0 || 2048));
    scene.traverse((obj) => {
      if (!obj.isLight || !obj.shadow) return;
      const ms = obj.shadow.mapSize;
      if (ms.x === s && ms.y === s) return;
      ms.set(s, s);
      if (obj.shadow.map) {
        obj.shadow.map.dispose();
        obj.shadow.map = null;
      }
      if (obj.shadow.mapPass) {
        obj.shadow.mapPass.dispose();
        obj.shadow.mapPass = null;
      }
      obj.shadow.needsUpdate = true;
    });
    renderer.shadowMap.needsUpdate = true;
  }

  // --- 7. stats -----------------------------------------------------------
  // renderer.info resets on every render() call, and the postfx chain issues several per
  // frame, so autoReset is off (above) and we aggregate: on the first render of a new
  // ctx.frame we publish the previous frame's totals and zero the counters.
  const stats = {
    drawCalls: 0,
    triangles: 0,
    points: 0,
    lines: 0,
    programs: 0,
    geometries: 0,
    textures: 0,
    passes: 0,
    fps: 0,
    frameTimeMs: 0,
    frameMeanMs: 0,          // 30-frame mean the governor decides on (0 until a full window)
    gpuFrames: 0,
    pixelRatio: appliedPixelRatio,        // the *effective* ratio actually applied (K2)
    pixelRatioTrim: 0,       // how much of it the governor is currently withholding
    width: viewW,            // CSS pixels
    height: viewH,
    drawWidth: 0,            // device pixels actually rendered
    drawHeight: 0,
    renderMPix: 0,           // …and their product, which is the number that costs money
  };

  let statsFrame = -1;
  let passCount = 0;
  let lastFrameStamp = (typeof performance !== 'undefined' ? performance.now() : Date.now());
  let smoothedFrameMs = 16.7;

  function rollFrameStats() {
    const info = renderer.info;
    stats.drawCalls = info.render.calls;
    stats.triangles = info.render.triangles;
    stats.points = info.render.points;
    stats.lines = info.render.lines;
    stats.passes = passCount;
    stats.geometries = info.memory.geometries;
    stats.textures = info.memory.textures;
    stats.programs = info.programs ? info.programs.length : 0;
    stats.pixelRatio = appliedPixelRatio;
    stats.pixelRatioTrim = govTrim;
    stats.width = viewW;
    stats.height = viewH;
    stats.drawWidth = Math.floor(viewW * appliedPixelRatio);
    stats.drawHeight = Math.floor(viewH * appliedPixelRatio);
    stats.renderMPix = (stats.drawWidth * stats.drawHeight) / 1e6;
    stats.gpuFrames = info.render.frame;

    const now = (typeof performance !== 'undefined' ? performance.now() : Date.now());
    const delta = now - lastFrameStamp;
    lastFrameStamp = now;
    // EMA over ~0.4 s; a raw 1/dt readout is unreadable noise in a HUD.
    if (delta > 0 && delta < 1000) {
      smoothedFrameMs += (delta - smoothedFrameMs) * 0.09;
      stats.frameTimeMs = smoothedFrameMs;
      stats.fps = smoothedFrameMs > 0 ? 1000 / smoothedFrameMs : 0;
    }
    // Governor runs off the raw delta, not the EMA: the EMA's ~0.4 s tail would stack a
    // second lag on top of the 30-frame window and make the loop under-damped.
    sampleGovernor(delta);

    info.reset();
    passCount = 0;
  }

  // --- 8. context loss ----------------------------------------------------
  let contextLost = false;
  let lostToast = null;
  let renderErrorLogged = false;

  // Wrap render() so a lost context cannot throw inside main.js's frame loop (that
  // call site is not fully guarded, and one throw there kills the game permanently).
  const rawRender = renderer.render.bind(renderer);
  renderer.render = function guardedRender(sceneArg, cameraArg) {
    if (contextLost) return;
    if (ctx.frame !== statsFrame) {
      rollFrameStats();
      statsFrame = ctx.frame;
    }
    passCount++;
    try {
      rawRender(sceneArg, cameraArg);
    } catch (err) {
      if (!renderErrorLogged) {
        renderErrorLogged = true;
        console.error('[engine] render threw', err);
      }
    }
  };

  function onContextLost(event) {
    event.preventDefault(); // required, or the browser will never restore the context
    contextLost = true;
    console.warn('[engine] WebGL context lost — pausing rendering');
    if (!lostToast) {
      lostToast = makeToast('Restoring graphics…');
      document.body.appendChild(lostToast);
    }
    if (ctx.events) ctx.events.emit('engine:contextlost', null);
  }

  function onContextRestored() {
    contextLost = false;
    console.warn('[engine] WebGL context restored — reinitialising render state');
    if (lostToast && lostToast.parentNode) lostToast.parentNode.removeChild(lostToast);
    lostToast = null;

    // three re-initialises its GL state itself; we only need to re-push our own policy.
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    applyToneMapping();
    renderer.shadowMap.type = THREE.PCFShadowMap;   // keep in step with construction (K3)
    renderer.shadowMap.autoUpdate = true;
    renderer.info.autoReset = false;
    applyShadowSettings();
    renderer.setPixelRatio(appliedPixelRatio);
    renderer.setSize(viewW, viewH, false);
    renderErrorLogged = false;

    if (ctx.events) ctx.events.emit('engine:contextrestored', null);
  }

  canvas.addEventListener('webglcontextlost', onContextLost, false);
  canvas.addEventListener('webglcontextrestored', onContextRestored, false);

  // --- 9. quality ---------------------------------------------------------
  function applyQuality() {
    applyShadowSettings();
    // A tier change moves the budget and the whole cost curve; anything the governor
    // learned about the old tier is worthless. Start it clean at full resolution.
    govTrim = 0;
    govRecoverStreak = 0;
    govRecoverNeeded = GOV_RECOVER_WINDOWS;
    govClearWindow();
    const ratioChanged = applySize(viewW, viewH, true);
    // The postfx render targets are sized from the drawing buffer, so a pixel-ratio
    // change without a window resize would leave the composer stale.
    if (ratioChanged && ctx.postfx && typeof ctx.postfx.resize === 'function') {
      try { ctx.postfx.resize(viewW, viewH); } catch (err) { /* postfx logs its own */ }
    }
  }

  // ctx.setQuality() may also be called directly by the menu; stay in sync either way.
  const offQuality = ctx.events ? ctx.events.on('quality:changed', applyQuality) : null;

  // --- 10. public API -----------------------------------------------------
  const api = {
    renderer,
    scene,
    camera,
    canvas,
    stats,

    /** Reported GPU string when the browser allows it — useful in bug reports. */
    gpu: caps.renderer,

    capabilities: {
      maxAnisotropy,
      maxTextureSize: renderer.capabilities.maxTextureSize,
      maxSamples: renderer.capabilities.maxSamples,
      precision: renderer.capabilities.precision,
      floatRenderTargets: renderer.extensions.has('EXT_color_buffer_float'),
      floatBlend: renderer.extensions.has('EXT_float_blend'),
      floatLinearFilter: renderer.extensions.has('OES_texture_float_linear'),
    },

    get contextLost() { return contextLost; },
    get pixelRatio() { return appliedPixelRatio; },
    /** Ratio the frame-time governor is currently withholding (0 = full resolution). */
    get pixelRatioTrim() { return govTrim; },
    get frameMeanMs() { return govMeanMs; },
    get width() { return viewW; },
    get height() { return viewH; },

    /** Robust viewport resize. Falls back to the container/window when handed junk. */
    resize(w, h) {
      let width = Number.isFinite(w) ? Math.floor(w) : 0;
      let height = Number.isFinite(h) ? Math.floor(h) : 0;
      if (width <= 0) width = container.clientWidth || window.innerWidth || viewW;
      if (height <= 0) height = container.clientHeight || window.innerHeight || viewH;
      if (width === viewW && height === viewH) {
        // Still re-evaluate the ratio: DPR may have changed under a same-size viewport.
        const pr = effectivePixelRatio(viewW, viewH);
        if (Math.abs(pr - appliedPixelRatio) > 1e-4) applySize(width, height, true);
        return;
      }
      // The window's frames are about to cost a different amount; don't let a decision
      // be made on a mix of pre- and post-resize samples.
      govClearWindow();
      govRecoverStreak = 0;
      applySize(width, height, true);
    },

    /**
     * Apply a quality preset. Accepts a preset name (delegates to ctx.setQuality so the
     * whole settings block moves together) or nothing (re-apply current settings).
     */
    setQuality(q) {
      if (typeof q === 'string' && q !== ctx.quality && typeof ctx.setQuality === 'function') {
        ctx.setQuality(q);   // emits 'quality:changed' → applyQuality()
        return;
      }
      applyQuality();
    },

    /** Override the requested pixel ratio (still clamped and budgeted). */
    setPixelRatio(pr) {
      if (!Number.isFinite(pr)) return;
      settings.pixelRatio = clamp(pr, MIN_PIXEL_RATIO, MAX_PIXEL_RATIO);
      const changed = applySize(viewW, viewH, true);
      if (changed && ctx.postfx && typeof ctx.postfx.resize === 'function') {
        try { ctx.postfx.resize(viewW, viewH); } catch (err) { /* ignore */ }
      }
    },

    /**
     * Turn the frame-time ratio governor on or off. Off restores full resolution
     * immediately — QA capture and any benchmark that needs a pinned backbuffer should
     * disable it, though the governor also stands down on its own whenever it sees the
     * renderer's pixel ratio differ from the one it applied.
     */
    setGovernorEnabled(enabled) {
      govEnabled = !!enabled;
      govClearWindow();
      govRecoverStreak = 0;
      if (!govEnabled && govTrim > 0) {
        govTrim = 0;
        scheduleGovernorApply();
      }
      return govEnabled;
    },

    /** Drop any accumulated trim and start metering again from full resolution. */
    resetGovernor() {
      govRecoverStreak = 0;
      govRecoverNeeded = GOV_RECOVER_WINDOWS;
      govClearWindow();
      if (govTrim > 0) {
        govTrim = 0;
        scheduleGovernorApply();
      }
    },

    /**
     * postfx calls this with `false` when it applies its own ACES/AgX pass, so the
     * image is not tonemapped twice (which crushes highlights and desaturates).
     */
    setToneMappingEnabled(enabled) {
      toneMappingEnabled = !!enabled;
      applyToneMapping();
      return toneMappingEnabled;
    },

    /** Change the renderer-side tonemapping operator (ignored while disabled). */
    setToneMapping(mode) {
      if (typeof mode !== 'number') return;
      toneMappingMode = mode;
      applyToneMapping();
    },

    /** Scene exposure in stops-ish multiplier form. ~1.0 with physical sun intensities. */
    setExposure(value) {
      if (!Number.isFinite(value)) return;
      exposure = clamp(value, 0.05, 8);
      applyToneMapping();
    },

    get exposure() { return exposure; },
    get toneMappingEnabled() { return toneMappingEnabled; },

    /** Adjust the clip planes (first-person camera may want a tighter near plane). */
    setClipPlanes(near, far) {
      if (Number.isFinite(near) && near > 0) camera.near = near;
      if (Number.isFinite(far) && far > camera.near) camera.far = far;
      camera.updateProjectionMatrix();
    },

    applyShadowSettings,

    /** Drawing-buffer size in device pixels, written into `out` (no allocation). */
    getDrawingBufferSize(out) {
      renderer.getDrawingBufferSize(_drawSize);
      if (out) return out.set(_drawSize.x, _drawSize.y);
      return _drawSize;
    },

    /** CSS-pixel viewport size, written into `out` (no allocation). */
    getSize(out) {
      _size.set(viewW, viewH);
      if (out) return out.set(viewW, viewH);
      return _size;
    },

    dispose() {
      disposed = true;   // a governor apply may still be queued on a rAF
      if (offQuality) offQuality();
      canvas.removeEventListener('webglcontextlost', onContextLost);
      canvas.removeEventListener('webglcontextrestored', onContextRestored);
      if (dprQuery && dprQuery.removeEventListener) dprQuery.removeEventListener('change', onDprChange);
      if (lostToast && lostToast.parentNode) lostToast.parentNode.removeChild(lostToast);
      if (fallbackEnv) fallbackEnv.dispose();
      fallbackSky.dispose();
      renderer.render = rawRender;
      renderer.dispose();
      if (canvas.parentNode) canvas.parentNode.removeChild(canvas);
    },
  };

  if (ctx.debug && ctx.debug.enabled) {
    ctx.debug.log('engine', {
      gpu: caps.renderer || '(hidden)',
      pixelRatio: appliedPixelRatio,
      size: `${viewW}×${viewH}`,
      backbuffer: `${Math.floor(viewW * appliedPixelRatio)}×${Math.floor(viewH * appliedPixelRatio)}`
        + ` (${((viewW * appliedPixelRatio * viewH * appliedPixelRatio) / 1e6).toFixed(2)} MPix)`,
      pixelBudgetMPix: (PIXEL_BUDGET[ctx.quality] || PIXEL_BUDGET.high) / 1e6,
      maxAnisotropy,
      shadowMapSize: settings.shadowMapSize,
    });
  }

  return api;
}

export default createEngine;
