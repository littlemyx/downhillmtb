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
import { createGameplay } from './game/gameplay.js';
import { createHud } from './ui/hud.js';
import { createMenu } from './ui/menu.js';
import { createPostFX } from './core/postfx.js';

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
    quality: params.get('quality') || 'high',
  });
  window.__DESCENT__ = ctx;
  window.__DESCENT_ERRORS__ = [];

  // ---- wave 1: engine + input -------------------------------------------
  const engine = safe('engine', () => createEngine(ctx));
  ctx.engine = engine;
  if (engine?.init) await safeAsync('engine.init', () => engine.init());
  if (!ctx.renderer || !ctx.scene || !ctx.camera) {
    throw new Error('engine did not populate ctx.renderer/scene/camera');
  }

  ctx.input = safe('input', () => createInput(ctx));
  if (ctx.input?.init) await safeAsync('input.init', () => ctx.input.init());

  // ---- wave 2/3: terrain then trail (trail carves, terrain commits) ------
  ctx.terrain = safe('terrain', () => createTerrain(ctx));
  if (ctx.terrain?.init) await safeAsync('terrain.init', () => ctx.terrain.init());
  safe('terrain.buildBase', () => ctx.terrain?.buildBase());

  ctx.trail = safe('trail', () => createTrail(ctx));
  if (ctx.trail?.init) await safeAsync('trail.init', () => ctx.trail.init());
  safe('trail.build', () => ctx.trail?.build(ctx.terrain));
  safe('terrain.applyCarve', () => {
    const stamps = ctx.trail?.getCarveStamps?.() || [];
    ctx.terrain?.applyCarve(stamps);
  });
  safe('terrain.commit', () => ctx.terrain?.commit());
  safe('trail.finalize', () => ctx.trail?.finalize(ctx.terrain));

  // ---- wave 4: world dressing -------------------------------------------
  const wave4 = [
    ['sky', createSky], ['water', createWater], ['vegetation', createVegetation],
  ];
  for (const [name, factory] of wave4) {
    ctx[name] = safe(name, () => factory(ctx));
    if (ctx[name]?.init) await safeAsync(`${name}.init`, () => ctx[name].init());
  }

  // ---- wave 5..8 ---------------------------------------------------------
  const rest = [
    ['collision', createCollision], ['bike', createBike],
    ['bikeModel', createBikeModel], ['rider', createRider], ['particles', createParticles],
    ['chaseCamera', createChaseCamera], ['audio', createAudio],
    ['gameplay', createGameplay], ['hud', createHud], ['menu', createMenu],
  ];
  for (const [name, factory] of rest) {
    ctx[name] = safe(name, () => factory(ctx));
    if (ctx[name]?.init) await safeAsync(`${name}.init`, () => ctx[name].init());
  }

  // ---- wave 9: post-processing (needs the finished scene) ----------------
  ctx.postfx = safe('postfx', () => createPostFX(ctx));
  if (ctx.postfx?.init) await safeAsync('postfx.init', () => ctx.postfx.init());

  // Ordered update lists, built once — no per-frame array churn.
  const updatables = [];
  const lateUpdatables = [];
  const resizables = [];
  const ORDER = [
    'input', 'terrain', 'trail', 'sky', 'water', 'vegetation',
    'collision', 'bike', 'bikeModel', 'rider', 'particles',
    'chaseCamera', 'audio', 'gameplay', 'hud', 'menu', 'postfx',
  ];
  for (const name of ORDER) {
    const s = ctx[name];
    if (!s) continue;
    if (typeof s.update === 'function') updatables.push([name, s]);
    if (typeof s.lateUpdate === 'function') lateUpdatables.push([name, s]);
    if (typeof s.resize === 'function') resizables.push([name, s]);
  }

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
  function tick() {
    requestAnimationFrame(tick);
    const dt = Math.min(ctx.clock.getDelta(), MAX_DT);
    ctx.dt = dt;
    ctx.time += dt;
    ctx.frame++;

    for (const [name, s] of updatables) {
      try { s.update(dt, ctx); } catch (e) {
        if (!frameErrLogged.has(name)) { frameErrLogged.add(name); console.error(`[update] ${name}`, e); window.__DESCENT_ERRORS__.push({ module: `${name}.update`, message: String(e && e.message || e) }); }
      }
    }
    for (const [name, s] of lateUpdatables) {
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
    if (ctx.frame === 3) document.body.dataset.descentReady = '1';
  }
  tick();

  return ctx;
}

boot().catch((err) => {
  console.error('[boot] fatal', err);
  const el = document.createElement('pre');
  el.style.cssText = 'position:fixed;inset:0;padding:24px;color:#ff6b6b;background:#05070b;font:12px ui-monospace,monospace;white-space:pre-wrap;z-index:9999';
  el.textContent = 'DESCENT failed to start:\n\n' + (err && err.stack || err);
  document.body.appendChild(el);
});
