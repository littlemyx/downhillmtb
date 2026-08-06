// DESCENT — bootstrap and frame loop.
// Owned by the integration layer. Implementers must NOT edit this file; it wires the
// modules described in CONTRACT.md together in the fixed wave order.

import * as THREE from 'three';
import { createContext } from './core/ctx.js';

import { createEngine } from './core/engine.js';
import { createInput } from './game/input.js';
import { createTerrain } from './world/terrain.js';
import { createTrail } from './world/trail.js';
import { createSky } from './world/sky.js';
import { createWater } from './world/water.js';
import { createVegetation } from './world/vegetation.js';
import { createCollision } from './physics/collision.js';
import { createBike } from './physics/bike.js';
import { createBikeModel } from './entities/bikeModel.js';
import { createRider } from './entities/rider.js';
import { createParticles } from './game/particles.js';
import { createChaseCamera } from './camera/chaseCamera.js';
import { createAudio } from './audio/audio.js';
import { createMusic } from './audio/music/music.js';
import { createGameplay } from './game/gameplay.js';
import { createHud } from './ui/hud.js';
import { createMenu } from './ui/menu.js';
import { createPostFX } from './core/postfx.js';
import { createPlatform } from './platform/yandex.js';
import { setLang } from './i18n/i18n.js';
import { beginStage, report, yieldFrame } from './core/bootProgress.js';

const MAX_DT = 1 / 20;

/** Guard so one bad module cannot blank the whole game. */
function safe(name, fn, fallback = null) {
  try {
    return fn();
  } catch (err) {
    console.error(`[boot] ${name} failed:`, err);
    window.__DESCENT_ERRORS__ = window.__DESCENT_ERRORS__ || [];
    window.__DESCENT_ERRORS__.push({ module: name, message: String(err && err.message || err), stack: err && err.stack });
    return fallback;
  }
}

async function safeAsync(name, fn) {
  try {
    return await fn();
  } catch (err) {
    console.error(`[boot] ${name} failed:`, err);
    window.__DESCENT_ERRORS__ = window.__DESCENT_ERRORS__ || [];
    window.__DESCENT_ERRORS__.push({ module: name, message: String(err && err.message || err), stack: err && err.stack });
    return null;
  }
}

async function boot() {
  const container = document.getElementById('app');
  const params = new URLSearchParams(location.search);
  const ctx = createContext(container, {
    seed: params.has('seed') ? Number(params.get('seed')) : undefined,
    quality: params.get('quality') || 'medium',
  });
  window.__DESCENT__ = ctx;
  window.__DESCENT_ERRORS__ = [];

  // Platform init runs alongside terrain generation — by the time the UI waves
  // need a language, it has long since settled. It never rejects.
  ctx.platform = safe('platform', () => createPlatform());
  const platformInit = ctx.platform ? ctx.platform.init({ timeoutMs: 3000 }) : null;

  // ---- wave 1: engine + input -------------------------------------------
  beginStage(0.01, 0.03, 'boot');
  const engine = safe('engine', () => createEngine(ctx));
  ctx.engine = engine;
  if (engine?.init) await safeAsync('engine.init', () => engine.init());
  if (!ctx.renderer || !ctx.scene || !ctx.camera) {
    throw new Error('engine did not populate ctx.renderer/scene/camera');
  }

  ctx.input = safe('input', () => createInput(ctx));
  if (ctx.input?.init) await safeAsync('input.init', () => ctx.input.init());

  // ---- wave 2/3: terrain then trail (trail carves, terrain commits) ------
  // The build steps are async solely so they can yield to the compositor and
  // move the boot progress bar; their work stays synchronous between yields.
  beginStage(0.03, 0.38, 'world');
  await yieldFrame();
  ctx.terrain = safe('terrain', () => createTerrain(ctx));
  if (ctx.terrain?.init) await safeAsync('terrain.init', () => ctx.terrain.init());
  await safeAsync('terrain.buildBase', () => ctx.terrain?.buildBase());

  beginStage(0.38, 0.63, 'trail');
  ctx.trail = safe('trail', () => createTrail(ctx));
  if (ctx.trail?.init) await safeAsync('trail.init', () => ctx.trail.init());
  await safeAsync('trail.build', () => ctx.trail?.build(ctx.terrain));
  beginStage(0.63, 0.71, 'carve');
  await safeAsync('terrain.applyCarve', () => {
    const stamps = ctx.trail?.getCarveStamps?.() || [];
    return ctx.terrain?.applyCarve(stamps);
  });
  beginStage(0.71, 0.78, 'chunks');
  await safeAsync('terrain.commit', () => ctx.terrain?.commit());
  beginStage(0.78, 0.83, 'detail');
  await safeAsync('trail.finalize', () => ctx.trail?.finalize(ctx.terrain));

  // ---- wave 4: world dressing -------------------------------------------
  beginStage(0.83, 0.91, 'dress');
  const wave4 = [
    ['sky', createSky], ['water', createWater], ['vegetation', createVegetation],
  ];
  let wave4Done = 0;
  for (const [name, factory] of wave4) {
    await yieldFrame();
    ctx[name] = safe(name, () => factory(ctx));
    if (ctx[name]?.init) await safeAsync(`${name}.init`, () => ctx[name].init());
    report(++wave4Done / wave4.length);
  }

  // ---- platform: language and cloud record, both before any UI is built ----
  // The language has to be in place before the HUD/menu factories run (they build
  // their DOM eagerly), and the cloud personal best has to be in localStorage
  // before gameplay reads it at construction.
  const platformState = await safeAsync('platform.init', () => platformInit);
  safe('i18n', () => setLang(platformState ? platformState.lang : navigator.language));
  if (platformState && platformState.sdk) {
    await safeAsync('platform.mergeCloudBest', () => ctx.platform.mergeCloudBest(ctx.seed));
  }

  // ---- wave 5..8 ---------------------------------------------------------
  beginStage(0.91, 0.95, 'ui');
  await yieldFrame();
  const rest = [
    ['collision', createCollision], ['bike', createBike],
    ['bikeModel', createBikeModel], ['rider', createRider], ['particles', createParticles],
    ['chaseCamera', createChaseCamera], ['audio', createAudio], ['music', createMusic],
    ['gameplay', createGameplay], ['hud', createHud], ['menu', createMenu],
  ];
  let restDone = 0;
  for (const [name, factory] of rest) {
    ctx[name] = safe(name, () => factory(ctx));
    if (ctx[name]?.init) await safeAsync(`${name}.init`, () => ctx[name].init());
    report(++restDone / rest.length);
  }

  // Gameplay marks, ad pauses and score submission all hang off ctx.events.
  safe('platform.bind', () => ctx.platform?.bind(ctx));

  // ---- wave 9: post-processing (needs the finished scene) ----------------
  beginStage(0.95, 0.99, 'fx');
  await yieldFrame();
  ctx.postfx = safe('postfx', () => createPostFX(ctx));
  if (ctx.postfx?.init) await safeAsync('postfx.init', () => ctx.postfx.init());
  beginStage(0.99, 1, 'fx');

  // Ordered update lists, built once — no per-frame array churn.
  const updatables = [];
  const lateUpdatables = [];
  const resizables = [];
  const ORDER = [
    'input', 'terrain', 'trail', 'sky', 'water', 'vegetation',
    'collision', 'bike', 'bikeModel', 'rider', 'particles',
    'chaseCamera', 'audio', 'music', 'gameplay', 'hud', 'menu', 'postfx',
  ];
  for (const name of ORDER) {
    const s = ctx[name];
    if (!s) continue;
    if (typeof s.update === 'function') updatables.push([name, s]);
    if (typeof s.lateUpdate === 'function') lateUpdatables.push([name, s]);
    if (typeof s.resize === 'function') resizables.push([name, s]);
  }

  // While the run is held, everything that animates off bike.state must hold with
  // it. bike.js freezes its sim on pause, but it publishes the LAST LIVE state —
  // speed, wheel spinRate, slip — so any module integrating that state with real
  // dt keeps moving: wheels spin in place, the camera keeps its speed shake, and
  // particles roost from the frozen contact patch for the whole pause. Input,
  // gameplay (it un-pauses), UI and audio (it ducks itself) stay live.
  const HELD_WHILE_PAUSED = new Set(['bike', 'bikeModel', 'rider', 'particles', 'chaseCamera']);

  function onResize() {
    const w = container.clientWidth || window.innerWidth;
    const h = container.clientHeight || window.innerHeight;
    if (engine?.resize) engine.resize(w, h);
    for (const [, s] of resizables) {
      try { s.resize(w, h); } catch (e) { /* keep the frame alive */ }
    }
  }
  window.addEventListener('resize', onResize);
  onResize();

  // Errors after boot get recorded too, so the visual-QA pass can see them.
  window.addEventListener('error', (e) => {
    window.__DESCENT_ERRORS__.push({ module: 'runtime', message: e.message });
  });

  let frameErrLogged = new Set();
  // Frame cap: on high-refresh displays rAF fires at 120 Hz+; skipped ticks return
  // before touching the clock, so the skipped time simply folds into the next dt.
  // Paced off the previous deadline (not "now") so vsync jitter doesn't drift the
  // rate; the 0.5 ms tolerance keeps a 60 cap from slipping to 40 on a 120 Hz panel.
  let capDeadline = 0;
  function tick() {
    requestAnimationFrame(tick);
    const cap = ctx.settings.fpsCap | 0;
    if (cap > 0) {
      const now = performance.now();
      const interval = 1000 / cap;
      if (now - capDeadline < interval - 0.5) return;
      capDeadline = Math.max(capDeadline + interval, now - interval);
    }
    const dt = Math.min(ctx.clock.getDelta(), MAX_DT);
    ctx.dt = dt;
    ctx.time += dt;
    ctx.frame++;

    const held = !!(ctx.gameplay && ctx.gameplay.state === 'paused');
    for (const [name, s] of updatables) {
      if (held && HELD_WHILE_PAUSED.has(name)) continue;
      try { s.update(dt, ctx); } catch (e) {
        if (!frameErrLogged.has(name)) { frameErrLogged.add(name); console.error(`[update] ${name}`, e); window.__DESCENT_ERRORS__.push({ module: `${name}.update`, message: String(e && e.message || e) }); }
      }
    }
    for (const [name, s] of lateUpdatables) {
      if (held && HELD_WHILE_PAUSED.has(name)) continue;
      try { s.lateUpdate(dt, ctx); } catch (e) {
        if (!frameErrLogged.has(name + ':late')) { frameErrLogged.add(name + ':late'); console.error(`[lateUpdate] ${name}`, e); window.__DESCENT_ERRORS__.push({ module: `${name}.lateUpdate`, message: String(e && e.message || e) }); }
      }
    }

    if (ctx.postfx && typeof ctx.postfx.render === 'function') {
      try { ctx.postfx.render(dt); }
      catch (e) {
        if (!frameErrLogged.has('postfx')) { frameErrLogged.add('postfx'); console.error('[render] postfx', e); }
        ctx.renderer.render(ctx.scene, ctx.camera);
      }
    } else {
      ctx.renderer.render(ctx.scene, ctx.camera);
    }

    // Signal for the automated visual-QA pass: the first fully rendered frame.
    // The same moment is what the platform means by "loaded" (requirement 1.19).
    if (ctx.frame === 3) {
      document.body.dataset.descentReady = '1';
      ctx.platform?.markReady();
      window.__DESCENT_BOOT__?.done();
    }
  }
  tick();

  return ctx;
}

boot().catch((err) => {
  console.error('[boot] fatal', err);
  window.__DESCENT_BOOT__?.fail();
  const el = document.createElement('pre');
  el.style.cssText = 'position:fixed;inset:0;padding:24px;color:#ff6b6b;background:#05070b;font:12px ui-monospace,monospace;white-space:pre-wrap;z-index:9999';
  el.textContent = 'DESCENT failed to start:\n\n' + (err && err.stack || err);
  document.body.appendChild(el);
});
