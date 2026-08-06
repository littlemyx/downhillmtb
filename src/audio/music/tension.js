// =============================================================================
// Tension model — one smoothed scalar in [0,1] plus a discrete band 0..3.
//
// Three terms:
//   section  — the trail's designed technicality at (slightly ahead of) the
//              rider, so the music swells before the rock garden, not in it;
//   perf     — how hard the rider is actually going: pace vs the design speed
//              profile, combo meter, airtime, g-force;
//   spike    — short event lifts (tricks) and the post-crash clamp.
//
// Continuous `tension` drives layer gains; `band` (with hysteresis) gates the
// discrete decisions — phrase density and register pools.
// =============================================================================

import { clamp01, damp } from '../../core/rng.js';

// trail.phases publishes { id, tStart, tEnd, surface } only; the technicality
// numbers live in trail.js's private PHASES table and are mirrored here by id
// (values, not code — keep in sync with src/world/trail.js PHASES).
const PHASE_TECH = {
  start: 0.05, roots: 0.90, flow: 0.15, jumps: 0.20, slab: 0.85,
  creek: 0.35, loam: 0.55, rocks: 0.95, sprint: 0.10,
};

const LOOKUP_N = 256;
const LOOKAHEAD_S = 2.5;         // seconds of travel the section term anticipates
const BAND_EDGES = [0.25, 0.5, 0.75];
const BAND_HYST = 0.06;
const SPIKE_DECAY = 0.25 / 4;    // a trick's +0.25 decays over ~4 s
const CRASH_HOLD_S = 3.0;
const CRASH_CEIL = 0.1;

export function createTension(ctx) {
  const section = new Float32Array(LOOKUP_N);
  let trailLen = 2600;
  let tension = 0.05;
  let band = 0;
  let spike = 0;
  let crashHold = 0;
  let override = null;

  function init() {
    const phases = ctx.trail && ctx.trail.phases;
    if (ctx.trail && ctx.trail.length > 0) trailLen = ctx.trail.length;
    section.fill(0.3);
    if (phases && phases.length) {
      for (let i = 0; i < LOOKUP_N; i++) {
        const t = i / (LOOKUP_N - 1);
        for (let p = 0; p < phases.length; p++) {
          if (t >= phases[p].tStart && t <= phases[p].tEnd) {
            const tech = PHASE_TECH[phases[p].id];
            section[i] = tech === undefined ? 0.3 : tech;
            break;
          }
        }
      }
      // Box-smooth so phase boundaries are ramps the music can ride, not steps.
      // Radius 8 of 256 ≈ 3% of the run ≈ 80 m — about a berm-to-berm breath.
      for (let pass = 0; pass < 3; pass++) {
        let acc = 0;
        const r = 8, w = 2 * r + 1;
        const src = Float32Array.from(section);
        for (let i = -r; i <= r; i++) acc += src[clampIdx(i)];
        for (let i = 0; i < LOOKUP_N; i++) {
          section[i] = acc / w;
          acc += src[clampIdx(i + r + 1)] - src[clampIdx(i - r)];
        }
        function clampIdx(i) { return i < 0 ? 0 : i >= LOOKUP_N ? LOOKUP_N - 1 : i; }
      }
    }
  }

  function updateBand() {
    while (band < 3 && tension > BAND_EDGES[band] + BAND_HYST) band++;
    while (band > 0 && tension < BAND_EDGES[band - 1] - BAND_HYST) band--;
  }

  function update(dt) {
    if (crashHold > 0) crashHold -= dt;
    if (spike > 0) spike = Math.max(0, spike - SPIKE_DECAY * dt);

    const st = ctx.bike && ctx.bike.state;
    const gp = ctx.gameplay;
    const gs = gp ? gp.state : 'menu';

    let target = 0.05; // menu floor
    if (st && (gs === 'running' || gs === 'paused' || gs === 'crashed' || gs === 'finished' || gs === 'countdown')) {
      const t = clamp01(st.trailT || 0);
      const ahead = clamp01(t + (st.speed || 0) * LOOKAHEAD_S / trailLen);
      const sec = section[(ahead * (LOOKUP_N - 1)) | 0];

      const design = ctx.trail && ctx.trail.speedAt ? ctx.trail.speedAt(t) : 10;
      const pace = clamp01(((st.speed || 0) / Math.max(1, design) - 0.55) / 0.6);
      const meter = gp.style ? clamp01(gp.style.meter || 0) : 0;
      const air = st.airborne ? clamp01((st.airTime || 0) / 1.2) : 0;
      const g = clamp01(((st.gForce || 1) - 1.2) / 2.0);
      const perf = 0.45 * pace + 0.3 * meter + 0.15 * air + 0.1 * g;

      target = clamp01(0.5 * sec + 0.5 * perf + spike);
      if (gs === 'countdown') target = Math.min(target, 0.3);
      if (gs === 'finished') target = Math.min(target, 0.25);
    }
    if (crashHold > 0) {
      target = Math.min(target, CRASH_CEIL);
      // The crash itself is the one deliberately instant drop.
      if (tension > CRASH_CEIL) tension = CRASH_CEIL;
    }

    // Fast up, slow down: energy arrives with the action and lingers after it.
    tension = damp(tension, target, target > tension ? 2.0 : 0.35, dt);
    if (override !== null) tension = override;
    updateBand();
  }

  return {
    init,
    update,
    trickLanded() { spike = Math.min(0.35, spike + 0.25); },
    crash() { crashHold = CRASH_HOLD_S; },
    respawn() { crashHold = 0; },
    setOverride(v) { override = (v === null || v === undefined) ? null : clamp01(v); },
    get override() { return override; },
    get tension() { return tension; },
    get band() { return band; },
  };
}
