// =============================================================================
// Sample-based ambient music system. CONTRACT-NOTE: this module is the one
// deliberate exception to "all sound synthesised in-code" — it lazily loads
// pre-produced stems from public/music/ strictly after platform ready, and the
// game is fully functional if those assets never arrive (state 'disabled',
// zero errors surfaced).
//
// Wiring: loader (fetch+decode) -> conductor (layers/phrases/stingers) into a
// lowpass -> trim chain that feeds audio.js's music bus, which inherits the
// master volume, limiter, mute, ad-mute and blur handling. tension.js turns
// trail design + riding + events into the scalar the conductor mixes by.
// =============================================================================

import { createMusicLoader } from './loader.js';
import { createTension } from './tension.js';
import { createConductor } from './conductor.js';

const CRASH_DEDUP_S = 0.4;       // run:crash and bike:crash double-emit
const LP_OPEN = 18000, LP_MENU = 2500, LP_PAUSE = 1200;
const TRIM_MENU = 0.5, TRIM_PAUSE = 0.4;  // ~-6 dB / ~-8 dB

export function createMusic(ctx) {
  // 'idle' -> 'loading' -> 'playing' | 'disabled'
  let state = 'idle';
  let loader = null;
  let tension = null;
  let conductor = null;
  let bus = null;                // { input, duck } from audio.js
  let lowpass = null, trim = null;
  let lastGs = '';
  let crashCool = 0;
  let crashSlam = 0;             // seconds the crash duck owns bus.duck
  let duckLevel = 1;
  const offs = [];

  function init() {
    tension = createTension(ctx);
    tension.init();
    wireEvents();
  }

  function wireEvents() {
    if (!ctx.events) return;
    const on = (name, fn) => {
      const o = ctx.events.on(name, fn);
      offs.push(typeof o === 'function' ? o : () => ctx.events.off(name, fn));
    };
    on('run:start', (p) => {
      if (conductor) conductor.newRun((p && p.seed) || ctx.seed);
    });
    const onCrash = (p) => {
      if (crashCool > 0) return;
      crashCool = CRASH_DEDUP_S;
      tension.crash();
      if (conductor && bus) {
        conductor.stinger('crash', false);
        // Slam the music down under the crash, rebuild slowly.
        const g = bus.duck.gain, t = ctx.audio.context.currentTime;
        g.cancelScheduledValues(t);
        g.setTargetAtTime(0.15, t, 0.02);
        g.setTargetAtTime(1.0, t + 0.5, 0.8);
        crashSlam = 3.0;
      }
    };
    on('run:crash', onCrash);
    on('bike:crash', onCrash);
    on('run:respawned', () => tension.respawn());
    on('run:finish', () => { if (conductor) conductor.stinger('finish', true); });
    on('trick:landed', () => tension.trickLanded());
  }

  function buildGraph() {
    const ac = ctx.audio.context;
    bus = ctx.audio.getMusicBus();
    if (!bus) return;
    lowpass = ac.createBiquadFilter();
    lowpass.type = 'lowpass';
    lowpass.frequency.value = LP_MENU;
    lowpass.Q.value = 0.5;
    trim = ac.createGain();
    trim.gain.value = TRIM_MENU;
    lowpass.connect(trim);
    trim.connect(bus.input);

    conductor = createConductor(ac, lowpass);
    conductor.setManifest(loader.manifest);
    for (const [id, buf] of loader.buffers) {
      const asset = loader.manifest.assets.find((a) => a.id === id);
      if (asset) conductor.addBuffer(asset, buf);
    }
    loader.onBuffer((asset, buf) => conductor.addBuffer(asset, buf));
    conductor.start(ctx.seed);
    lastGs = '';
    state = 'playing';
  }

  /** Menu/pause voicing: one filter + one trim, retargeted on state change. */
  function applyGameState(gs) {
    if (gs === lastGs || !bus) return;
    lastGs = gs;
    const t = ctx.audio.context.currentTime;
    let lp = LP_OPEN, tr = 1.0, tc = 0.5;
    if (gs === 'menu') { lp = LP_MENU; tr = TRIM_MENU; }
    else if (gs === 'paused') { lp = LP_PAUSE; tr = TRIM_PAUSE; tc = 0.25; }
    else if (gs === 'countdown') { tc = 0.7; } // the slow open into the run
    lowpass.frequency.setTargetAtTime(lp, t, tc);
    trim.gain.setTargetAtTime(tr, t, tc);
  }

  function realUpdate(dt) {
    if (state === 'idle') {
      // Strictly after platform ready (markReady fires on frame 3) so the
      // first-ever network fetch of this project can't touch time-to-ready.
      if (ctx.frame > 3) {
        loader = createMusicLoader('./music/');
        loader.start();
        state = 'loading';
      }
      return;
    }
    if (state === 'disabled') return;

    const audio = ctx.audio;
    if (loader && audio && audio.started) loader.drainDecodes(audio.context);

    if (state === 'loading') {
      if (loader.state === 'disabled') { state = 'disabled'; return; }
      if (loader.manifest && audio && audio.started && loader.buffers.size > 0) buildGraph();
      return;
    }

    // ---- playing -----------------------------------------------------------
    tension.update(dt);
    const gs = ctx.gameplay ? ctx.gameplay.state : 'menu';
    applyGameState(gs);
    if (crashCool > 0) crashCool -= dt;
    if (crashSlam > 0) crashSlam -= dt;
    else {
      // Sit slightly under heavy SFX moments (skid, lockup) — read straight
      // off audio.js's already-smoothed state, nothing recomputed here.
      const s = audio.debugState;
      const target = 1 - 0.25 * Math.max(s.skid, s.lockup);
      if (Math.abs(target - duckLevel) > 0.01) {
        duckLevel = target;
        bus.duck.gain.setTargetAtTime(target, audio.context.currentTime, 0.3);
      }
    }
    conductor.update(dt, tension.tension, tension.band);
  }

  const api = {
    init,
    update(dt) {
      if (state === 'disabled') return;
      try { realUpdate(dt); }
      catch (e) {
        // Music must never cost a frame or a run: first failure turns it off.
        state = 'disabled';
        console.warn('[music] disabled:', e);
      }
    },
    dispose() {
      for (let i = 0; i < offs.length; i++) { try { offs[i](); } catch (e) { /* ignore */ } }
      offs.length = 0;
      if (conductor) conductor.dispose();
      if (loader) loader.dispose();
      try { if (lowpass) lowpass.disconnect(); if (trim) trim.disconnect(); } catch (e) { /* ignore */ }
      conductor = null; loader = null; bus = null;
      state = 'disabled';
    },

    // --- debug / QA ---------------------------------------------------------
    setTensionOverride(v) { if (tension) tension.setOverride(v); },
    get debugState() {
      return {
        state,
        tension: tension ? tension.tension : 0,
        band: tension ? tension.band : 0,
        override: tension ? tension.override : null,
        barIndex: conductor ? conductor.barIndex : -1,
        layerGains: conductor ? conductor.layerGains : [],
        lastDecisions: conductor ? conductor.lastDecisions : [],
        loaded: loader ? { state: loader.state, fetched: loader.fetched, failed: loader.failed, decoded: loader.buffers.size } : null,
      };
    },
  };
  return api;
}

export default createMusic;
