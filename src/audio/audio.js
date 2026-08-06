// =============================================================================
// audio.js — DESCENT. Fully synthesised WebAudio engine (CONTRACT §8).
//
// Almost nothing here is loaded from disk or network: every buffer, impulse
// response and waveform is generated in code at start-up from the deterministic
// PRNG in core/rng.js. The one exception (CONTRACT amendment) is a small set of
// pre-produced Suno foley samples (impact/crash/slide/wind, public/sfx/) that
// replace the harshest synthesised one-shots when decoded — with the synthesis
// kept as the fallback. The graph is only built after a real user gesture, so
// the page can never be autoplay-blocked and no AudioContext exists until the
// player has touched something.
//
// CONTRACT-NOTE: "no per-frame allocations in update()" cannot be taken
// literally for WebAudio one-shots — AudioBufferSourceNode and OscillatorNode
// are single-use by specification, so a granular/impulse layer must construct
// one node per event. Everything that CAN be reused is pooled: every filter,
// gain, panner and waveshaper for the grain, thunk, clank, bell and vocal
// voices is allocated once at build time and re-triggered. Only the source node
// itself is new, and the whole one-shot system is behind a hard voice cap
// (MAX_VOICES) plus per-family rate limits. No JS objects, arrays, vectors or
// closures are allocated in update().
//
// CONTRACT-NOTE: ctx.settings has no volume field, so master volume lives here
// and is persisted to localStorage under 'descent.audio'. menu.js should call
// audio.setMasterVolume(0..1) / audio.setMuted(bool); both are safe before the
// graph exists.
//
// CONTRACT-NOTE: additive `setExternalMute(bool)` silences the master for the
// host platform (a fullscreen ad is up). It is a separate flag from `muted` and
// is never persisted, so releasing it restores whatever the player had chosen
// rather than un-muting them.
//
// CONTRACT-NOTE: this module reads several additive fields that bike.js
// publishes beyond CONTRACT §5 (state.skid, state.roughness, state.yawRate,
// state.lastLanding) and the additive events 'bike:landed' / 'bike:crash'.
// Every one of them is optional — the engine degrades to speed-and-surface-only
// behaviour if the bike is still a stub.
//
// Signal layout
// -------------
//   busBike   tyre roll, grains, suspension, chain, freehub, brakes, skid,
//             impacts, crashes, rider breathing and grunts
//   busWind   wind noise (dry only — wind is not a reverberant source)
//   busAmb    forest bed, birds, creek           -> ambDuck -> busSfx
//   busUI     run-event tones
//
//   each bus -> (dry gain -> busSfx) + (send gain -> revSend)
//
// busSfx is the effects volume stage: every non-music path (dry buses, reverb
// returns) sums into it before master, so setSfxVolume scales effects only.
//
// Everything on busBike additionally carries a PER-VOICE reverb send, set at
// trigger time, because the families sharing that bus want wildly different
// space: a tyre grain wants almost none (it happens under you), a crash wants
// a lot (it should sound like the whole valley heard it).
//   revSend  -> convolverForest -> returnForest -\
//            -> convolverOpen   -> returnOpen   --> busSfx -> master
//   master   -> limiter (DynamicsCompressor, brick-ish) -> destination
// =============================================================================

import { makeRng, subSeed, clamp, clamp01, lerp, damp } from '../core/rng.js';
import { Surface } from '../world/terrain.js';
import { createSfxLoader } from './sfx/loader.js';

// -----------------------------------------------------------------------------
// Tuning constants
// -----------------------------------------------------------------------------

const STORAGE_KEY = 'descent.audio';

// Hard cap on simultaneous one-shot sources. Measured peak during a heavy
// section (gravel at speed + a crash) is ~50, so this leaves real headroom; the
// tyre-grain layer additionally refuses to start above GRAIN_BUDGET of the cap
// so a granular storm can never starve an impact or a run tone.
const MAX_VOICES = 96;
const GRAIN_BUDGET = 0.55;
const GRAIN_VOICES = 24;        // pooled tyre-grain channels
const CLICK_VOICES = 10;        // pooled metallic/percussive channels
const BODY_VOICES = 8;          // pooled tonal (sine/FM) channels
const NOISE_SECONDS = 2.6;      // length of each looping noise bed

// Per-voice reverb send levels for the families that share busBike.
const SEND_GRAIN = 0.09;        // tyre grit — happens under you, barely any space
const SEND_MECH = 0.18;         // suspension, chain
const SEND_HIT = 0.50;          // impacts
const SEND_CRASH = 0.62;        // crash — the whole valley heard it
const SEND_RIDER = 0.22;        // breathing, grunts

const REF_SPEED = 22.0;         // m/s that counts as "flat out" for scaling
const WHEEL_RADIUS = 0.37;      // CONTRACT §0 scale reference
const FREEHUB_TEETH = 44;       // ratchet engagements per wheel revolution

// Surface voicing table. Indexed by the Surface enum in world/terrain.js:
// DIRT 0, LOAM 1, ROCK 2, GRAVEL 3, GRASS 4, ROOT 5, MUD 6, SNOW 7.
//
// Every field here is a continuous scalar, which is what makes the surface
// "crossfade" work: instead of switching between parallel voices (clicky, and
// three times the CPU) the live parameters are exponentially damped towards the
// new surface's targets over ~0.3 s. A blend of two surfaces is a genuine
// intermediate timbre, not a gain crossfade between two fixed ones.
const SURF = [
  { // 0 DIRT — the default hardpack. Mid, dry, a bit of body.
    brown: 0.55, pink: 0.62, white: 0.20,
    lp: 1350, hp: 95, peakF: 900, peakG: 4.0, peakQ: 1.7,
    level: 1.00, rate: 1.00,
    grain: 24, grainF: 1500, grainSpread: 0.95, grainDec: 0.045, grainQ: 3.0,
    squelch: 0.00, squelchHz: 2.2,
    skidF: 1400, skidQ: 2.2, skidLevel: 1.00, roll: 1.0,
  },
  { // 1 LOAM — dark, damp, low. The "hero dirt" thud.
    brown: 0.88, pink: 0.44, white: 0.05,
    lp: 640, hp: 70, peakF: 420, peakG: 4.5, peakQ: 1.3,
    level: 0.94, rate: 0.92,
    grain: 10, grainF: 700, grainSpread: 0.7, grainDec: 0.065, grainQ: 2.0,
    squelch: 0.14, squelchHz: 1.7,
    skidF: 820, skidQ: 1.8, skidLevel: 0.85, roll: 1.05,
  },
  { // 2 ROCK — hard and ringing, a high-Q peak that sings under the tyre.
    brown: 0.18, pink: 0.52, white: 0.58,
    lp: 5400, hp: 160, peakF: 2400, peakG: 9.0, peakQ: 7.0,
    level: 1.12, rate: 1.10,
    grain: 42, grainF: 3200, grainSpread: 1.35, grainDec: 0.028, grainQ: 8.0,
    squelch: 0.00, squelchHz: 3.0,
    skidF: 2700, skidQ: 4.2, skidLevel: 1.10, roll: 0.88,
  },
  { // 3 GRAVEL — bright and grainy, a constant rain of stone hits.
    brown: 0.30, pink: 0.66, white: 0.52,
    lp: 3500, hp: 130, peakF: 1900, peakG: 5.5, peakQ: 2.5,
    level: 1.14, rate: 1.05,
    grain: 74, grainF: 2600, grainSpread: 1.25, grainDec: 0.022, grainQ: 5.5,
    squelch: 0.00, squelchHz: 3.0,
    skidF: 2100, skidQ: 2.7, skidLevel: 1.18, roll: 1.15,
  },
  { // 4 GRASS — soft hiss, almost no impulses, low level.
    brown: 0.14, pink: 0.34, white: 0.76,
    lp: 5800, hp: 260, peakF: 4300, peakG: 2.0, peakQ: 1.0,
    level: 0.76, rate: 1.02,
    grain: 5, grainF: 3900, grainSpread: 1.0, grainDec: 0.05, grainQ: 1.6,
    squelch: 0.00, squelchHz: 2.5,
    skidF: 3300, skidQ: 1.4, skidLevel: 0.72, roll: 1.25,
  },
  { // 5 ROOT — dirt-like bed, but heavy low-pitched periodic knocks.
    brown: 0.64, pink: 0.56, white: 0.20,
    lp: 1150, hp: 80, peakF: 760, peakG: 5.5, peakQ: 2.3,
    level: 1.06, rate: 0.97,
    grain: 17, grainF: 520, grainSpread: 0.6, grainDec: 0.08, grainQ: 4.5,
    squelch: 0.05, squelchHz: 2.0,
    skidF: 1100, skidQ: 2.0, skidLevel: 0.95, roll: 1.1,
  },
  { // 6 MUD — wet, thick, squelching. Heavy LFO sweep on the lowpass.
    brown: 0.96, pink: 0.50, white: 0.10,
    lp: 420, hp: 60, peakF: 330, peakG: 6.5, peakQ: 3.4,
    level: 1.02, rate: 0.86,
    grain: 14, grainF: 480, grainSpread: 0.8, grainDec: 0.095, grainQ: 2.2,
    squelch: 0.90, squelchHz: 1.35,
    skidF: 620, skidQ: 1.6, skidLevel: 0.80, roll: 1.45,
  },
  { // 7 SNOW — muffled crunch.
    brown: 0.36, pink: 0.46, white: 0.62,
    lp: 3000, hp: 150, peakF: 1500, peakG: 2.5, peakQ: 1.5,
    level: 0.84, rate: 0.95,
    grain: 30, grainF: 2200, grainSpread: 1.1, grainDec: 0.035, grainQ: 3.5,
    squelch: 0.00, squelchHz: 2.5,
    skidF: 1800, skidQ: 1.2, skidLevel: 0.86, roll: 1.3,
  },
];
const SURF_DEFAULT = SURF[0];

// A-minor pentatonic, the friendliest scale for "tasteful, not arcade" UI tones.
const PENT = [220.00, 261.63, 293.66, 329.63, 392.00, 440.00, 523.25, 587.33, 659.25, 784.00, 880.00];

// Bird songs. Each entry is a flat note list so playback allocates nothing:
// [f0, f1, dur, gap, fmRatio, fmIndex] repeated. f0/f1 are the start/end pitch
// of the note (a glide), fmIndex is in units of carrier frequency.
const BIRD_SONGS = [
  // 0 — long rising whistle, twice. Thrush-ish.
  { gain: 0.85, bright: 1.0, notes: [2150, 3350, 0.30, 0.42, 2.0, 0.06, 2200, 3500, 0.26, 0.0, 2.0, 0.06] },
  // 1 — fast descending trill.
  { gain: 0.7, bright: 1.1, notes: [
    3700, 3450, 0.055, 0.022, 3.1, 0.35, 3650, 3400, 0.05, 0.022, 3.1, 0.35,
    3600, 3350, 0.05, 0.022, 3.1, 0.35, 3520, 3280, 0.05, 0.022, 3.1, 0.35,
    3450, 3200, 0.05, 0.022, 3.1, 0.35, 3380, 3050, 0.06, 0.0, 3.1, 0.35] },
  // 2 — two-tone call, the classic "tee-too".
  { gain: 0.95, bright: 0.9, notes: [3150, 3200, 0.12, 0.085, 2.0, 0.05, 2380, 2300, 0.20, 0.0, 2.0, 0.05] },
  // 3 — dry chatter / alarm rattle.
  { gain: 0.6, bright: 1.2, notes: [
    4200, 4100, 0.028, 0.032, 1.41, 1.5, 4300, 4150, 0.028, 0.032, 1.41, 1.5,
    4150, 4000, 0.028, 0.032, 1.41, 1.5, 4350, 4200, 0.028, 0.032, 1.41, 1.5,
    4100, 3950, 0.028, 0.032, 1.41, 1.5, 4250, 4100, 0.028, 0.032, 1.41, 1.5,
    4180, 3900, 0.034, 0.0, 1.41, 1.5] },
  // 4 — low dove coo, three soft notes with vibrato-ish FM.
  { gain: 1.0, bright: 0.45, notes: [640, 610, 0.30, 0.16, 1.0, 0.02, 600, 580, 0.34, 0.18, 1.0, 0.02, 590, 555, 0.40, 0.0, 1.0, 0.02] },
  // 5 — single high peep.
  { gain: 0.65, bright: 1.15, notes: [5300, 4600, 0.07, 0.0, 2.0, 0.1] },
  // 6 — raven croak, harsh and inharmonic. Belongs above the treeline.
  { gain: 1.1, bright: 0.6, notes: [420, 360, 0.26, 0.34, 2.71, 2.6, 400, 340, 0.30, 0.0, 2.71, 2.6] },
  // 7 — woodpecker drum: pure impulse rhythm, almost no pitch.
  { gain: 0.8, bright: 1.0, notes: [
    1500, 1400, 0.02, 0.038, 1.0, 3.0, 1500, 1400, 0.02, 0.038, 1.0, 3.0,
    1500, 1400, 0.02, 0.038, 1.0, 3.0, 1500, 1400, 0.02, 0.038, 1.0, 3.0,
    1500, 1400, 0.02, 0.038, 1.0, 3.0, 1480, 1360, 0.024, 0.0, 1.0, 3.0] },
];
// How likely each song is when the terrain is forested vs. exposed.
const BIRD_FOREST_WEIGHT = [1.0, 1.0, 1.0, 0.8, 0.9, 0.7, 0.05, 0.6];
const BIRD_OPEN_WEIGHT = [0.35, 0.25, 0.3, 0.25, 0.05, 0.5, 1.0, 0.0];

// -----------------------------------------------------------------------------
// Module-scope scratch. Nothing below is reallocated per frame.
// -----------------------------------------------------------------------------

const _pan = { pan: 0, dist: 1, gain: 1 };   // written by spatialise()
let _voices = 0;                              // live one-shot source count

// =============================================================================
// Buffer / waveform generators (run once, at graph build)
// =============================================================================

/**
 * Two-channel noise beds. Both channels are independent streams so any pair of
 * sources reading the same buffer at different offsets is genuinely decorrelated
 * — that is what makes the wind and forest beds sound wide rather than mono-in-
 * the-middle.
 *
 * kind: 'white' | 'pink' | 'brown'
 *   pink  — Paul Kellet's economy filter (−3 dB/oct, cheap and accurate enough)
 *   brown — leaky integrator (−6 dB/oct), the body of every low rumble here
 */
function makeNoiseBuffer(ac, rng, kind, seconds) {
  const sr = ac.sampleRate;
  const len = Math.max(1024, Math.floor(sr * seconds));
  const buf = ac.createBuffer(2, len, sr);
  for (let ch = 0; ch < 2; ch++) {
    const d = buf.getChannelData(ch);
    if (kind === 'white') {
      for (let i = 0; i < len; i++) d[i] = rng() * 2 - 1;
    } else if (kind === 'pink') {
      let b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
      for (let i = 0; i < len; i++) {
        const w = rng() * 2 - 1;
        b0 = 0.99886 * b0 + w * 0.0555179;
        b1 = 0.99332 * b1 + w * 0.0750759;
        b2 = 0.96900 * b2 + w * 0.1538520;
        b3 = 0.86650 * b3 + w * 0.3104856;
        b4 = 0.55000 * b4 + w * 0.5329522;
        b5 = -0.7616 * b5 - w * 0.0168980;
        d[i] = (b0 + b1 + b2 + b3 + b4 + b5 + b6 + w * 0.5362) * 0.11;
        b6 = w * 0.115926;
      }
    } else {
      let last = 0;
      for (let i = 0; i < len; i++) {
        const w = rng() * 2 - 1;
        last = (last + 0.019 * w) / 1.019;
        d[i] = last * 3.4;
      }
    }
    // Loop-point conditioning: cross-fade the last 40 ms into the head so the
    // seam is inaudible on an infinitely looping source.
    const xf = Math.min(Math.floor(sr * 0.04), len >> 3);
    for (let i = 0; i < xf; i++) {
      const t = i / xf;
      const a = d[i], b = d[len - xf + i];
      d[i] = a * t + b * (1 - t);
    }
    // Normalise so every kind arrives at the mixer at a comparable level.
    let peak = 1e-6;
    for (let i = 0; i < len; i++) { const a = Math.abs(d[i]); if (a > peak) peak = a; }
    const k = 0.92 / peak;
    for (let i = 0; i < len; i++) d[i] *= k;
  }
  return buf;
}

/**
 * Procedural reverb impulse response.
 *
 * Structure, in the order a real room produces it:
 *   1. pre-delay of silence (distance to the first surface),
 *   2. a handful of discrete early reflections with slightly different L/R
 *      arrival times — this is what gives a space a *size*,
 *   3. a diffuse noise tail under an exponential decay, progressively
 *      low-passed to model air absorption (high frequencies die first).
 *
 * opts.decay is the exponential rate (higher = shorter), opts.brightness is the
 * starting one-pole coefficient of the darkening filter.
 */
function makeImpulseResponse(ac, rng, opts) {
  const sr = ac.sampleRate;
  const dur = opts.duration;
  const len = Math.max(256, Math.floor(sr * dur));
  const buf = ac.createBuffer(2, len, sr);
  const preDelay = Math.floor(sr * opts.preDelay);

  // Both exponentials are advanced by a constant per-sample ratio rather than
  // recomputed — two Math.exp calls per sample dominated the build cost, and
  // the incremental form is exact for a fixed step.
  const kDamp = Math.exp(-opts.damping / sr);
  const kDecay = Math.exp(-opts.decay / sr);
  const buildN = Math.max(1, Math.floor(sr * 0.025));
  const invLen = 1 / len;

  for (let ch = 0; ch < 2; ch++) {
    const d = buf.getChannelData(ch);
    let lp = 0;
    let aE = opts.brightness;    // brightness * exp(-t * damping)
    let dE = 1;                  // exp(-t * decay)
    for (let i = preDelay; i < len; i++) {
      const w = rng() * 2 - 1;
      // Air absorption: the filter closes as the tail ages.
      const a = aE < 0.015 ? 0.015 : (aE > 0.995 ? 0.995 : aE);
      lp += a * (w - lp);
      // Build-up over the first ~25 ms so the tail swells rather than starting
      // at full density, then exponential decay with a linear tail-off so the
      // very end of the buffer reaches exactly zero.
      const n = i - preDelay;
      const build = n < buildN ? n / buildN : 1;
      d[i] = lp * build * dE * (1 - i * invLen);
      aE *= kDamp;
      dE *= kDecay;
    }
    // Early reflections. Deterministic times, per-channel jitter for width.
    const erBase = opts.erSpread;
    for (let k = 0; k < opts.erCount; k++) {
      const jitter = 1 + (rng() - 0.5) * 0.34;
      const tRef = erBase * (0.22 + k * 0.55 + rng() * 0.3) * jitter;
      const idx = preDelay + Math.floor(tRef * sr);
      if (idx >= len - 3) continue;
      const amp = opts.erGain * Math.exp(-k * 0.42) * (0.6 + rng() * 0.7) * (rng() < 0.5 ? -1 : 1);
      // Smear each reflection over a few samples so it reads as a surface, not a tick.
      d[idx] += amp;
      d[idx + 1] += amp * 0.55;
      d[idx + 2] += amp * 0.22;
    }
  }
  // Normalise to a fixed RMS so swapping IRs does not change the wet level.
  let sum = 0, n = 0;
  for (let ch = 0; ch < 2; ch++) {
    const d = buf.getChannelData(ch);
    for (let i = 0; i < len; i += 7) { sum += d[i] * d[i]; n++; }
  }
  const rms = Math.sqrt(sum / Math.max(1, n)) || 1e-6;
  const gain = 0.055 / rms;
  for (let ch = 0; ch < 2; ch++) {
    const d = buf.getChannelData(ch);
    for (let i = 0; i < len; i++) d[i] *= gain;
  }
  return buf;
}

/**
 * WaveShaper curve that turns a band-limited sawtooth into a narrow pulse
 * train. Used for the freehub: an impulse train excites the ratchet resonances
 * exactly the way a pawl does, and unlike a PeriodicWave it keeps its full
 * bright spectrum at low click rates. Run the shaper at 4x oversampling.
 */
function makePulseCurve(width) {
  const n = 4096;
  const c = new Float32Array(n);
  const centre = 0.72;         // where in the saw ramp the pulse fires
  for (let i = 0; i < n; i++) {
    const x = (i / (n - 1)) * 2 - 1;
    const u = (x - centre) / width;
    // Unipolar Gaussian spike: near-zero for most of the ramp. The residual DC
    // is removed by the highpass that follows the shaper.
    c[i] = Math.exp(-u * u);
  }
  return c;
}

/** Gentle saturation for the master bus — catches transients before the limiter. */
function makeSoftClipCurve() {
  const n = 2048;
  const c = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    const x = (i / (n - 1)) * 2 - 1;
    c[i] = Math.tanh(x * 1.35) / Math.tanh(1.35);
  }
  return c;
}

// =============================================================================
// createAudio
// =============================================================================

export function createAudio(ctx) {
  // ---------------------------------------------------------------------------
  // Persisted / externally settable state. Valid before the graph exists.
  // ---------------------------------------------------------------------------
  let masterVolume = 0.85;
  let musicVolume = 0.7;
  let sfxVolume = 1.0;
  let muted = false;
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const p = JSON.parse(raw);
      if (typeof p.volume === 'number') masterVolume = clamp01(p.volume);
      if (typeof p.musicVolume === 'number') musicVolume = clamp01(p.musicVolume);
      if (typeof p.sfxVolume === 'number') sfxVolume = clamp01(p.sfxVolume);
      if (typeof p.muted === 'boolean') muted = p.muted;
    }
  } catch (e) { /* private mode / disabled storage — defaults are fine */ }

  function persist() {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify({ volume: masterVolume, musicVolume, sfxVolume, muted })); }
    catch (e) { /* ignore */ }
  }

  let ac = null;
  let started = false;
  let failed = false;
  let windowBlurred = false;
  let docHidden = false;
  // Silenced by the host platform (an ad is on screen). Deliberately separate
  // from `muted`, which is the player's own setting and is persisted.
  let externalMuted = false;

  const rng = makeRng(subSeed((ctx && ctx.seed) || 1, 'audio'));
  const erng = makeRng(subSeed((ctx && ctx.seed) || 1, 'audio:events'));

  // ---------------------------------------------------------------------------
  // Live, JS-smoothed parameter block. Damped here rather than in AudioParams so
  // one frame of a stuttering rAF can never produce a click, and so nothing has
  // to be allocated to hold intermediate values.
  // ---------------------------------------------------------------------------
  const S = {
    // derived from the bike
    speed: 0, speedN: 0, load: 0, airborne: 0, contact: 1,
    rearSpin: 0, frontSpin: 0, skid: 0, rough: 0.25, wetness: 0,
    brakeF: 0, brakeR: 0, lockup: 0, pedal: 0, coast: 0,
    gForce: 1, yaw: 0, drift: 0,
    // surface voicing, exponentially chasing SURF[current]
    brown: SURF_DEFAULT.brown, pink: SURF_DEFAULT.pink, white: SURF_DEFAULT.white,
    lp: SURF_DEFAULT.lp, hp: SURF_DEFAULT.hp,
    peakF: SURF_DEFAULT.peakF, peakG: SURF_DEFAULT.peakG, peakQ: SURF_DEFAULT.peakQ,
    level: SURF_DEFAULT.level, rate: SURF_DEFAULT.rate,
    squelch: 0, squelchHz: 2.2, grainRate: 0, grainF: 1500, grainSpread: 1,
    grainDec: 0.045, grainQ: 3, skidF: 1400, skidQ: 2.2, skidLevel: 1, roll: 1,
    // environment
    openness: 0.35, creek: 0, creekPan: 0, altitude: 0,
    // rider
    effort: 0.15, breathPhase: 0, stamina: 1,
    // mix
    pauseGain: 1,
  };

  // Suspension trackers (previous travel, for a velocity we can trust even if
  // bike.js does not publish one).
  let prevFork = 0, prevShock = 0, forkDeep = 0, shockDeep = 0;
  let thunkCool = 0, reboundCool = 0, slapCool = 0;
  let grainAcc = 0, birdTimer = 3.5, bloopTimer = 2.0;
  let envTimer = 0, creekTimer = 0, crashCool = 0;
  let runTime = 0;
  let lastCheckpoint = 0;

  // ---------------------------------------------------------------------------
  // Graph handles. All null until build().
  // ---------------------------------------------------------------------------
  let master, limiter, shaper;
  let busSfx;
  let busBike, busWind, busAmb, busUI, ambDuck;
  let busMusic, musicDuck;
  let revSend, revForestGain, revOpenGain;
  let nWhite, nPink, nBrown;                       // shared noise buffers

  // tyre
  let tyBrown, tyPink, tyWhite, tyHP, tyLP, tyPeak, tyGain, tySquelchOsc, tySquelchAmt;
  let tySrcBrown, tySrcPink, tySrcWhite;
  // wind
  let wdSrcL, wdSrcR, wdBpL, wdBpR, wdGainL, wdGainR, wdPanL, wdPanR, wdLowSrc, wdLowLP, wdLowGain;
  // Suno-sampled SFX (public/sfx/). Loader fetches lazily; every pick() has a
  // procedural fallback, so none of this is load-bearing.
  let sfxLoader = null;
  let wdSmpSrc = null, wdSmpGain = null, wdSmpLP = null;
  // While a sampled crash take is playing, impact() must not add its own
  // recorded hit — two foley thuds 20 ms apart read as a flam/echo. The
  // synthesised low body drop still fires; only the sample is suppressed.
  // The cooldown also mutes the metallic one-shots (bottom-out clank, chain
  // slap, pings) that read as chimes on top of a recording.
  let impactSmpCool = 0;
  // The last started impact take: a run:crash arriving in the same physics
  // step stops it and substitutes the crash take, killing the comb/flam.
  let lastImpactSmp = null, lastImpactSmpT = -9;
  // freehub
  let fhOsc, fhShape, fhHP, fhBpA, fhBpB, fhBpC, fhGain, fhJitterSrc, fhJitterLP, fhJitterAmt;
  // brakes
  let brSrc, brBp, brPeak, brGain, brLfo, brLfoAmt;
  let sqOscA, sqOscB, sqNoiseSrc, sqBp, sqGain, sqVib, sqVibAmt;
  // skid
  let skSrc, skBp, skPeak, skGain;
  // ambience
  let amWindSrcL, amWindSrcR, amWindBpL, amWindBpR, amWindGainL, amWindGainR;
  let amLfoA, amLfoAAmt, amLfoB, amLfoBAmt, amPanL, amPanR;
  let ckSrc, ckHP, ckBpA, ckBpB, ckBpC, ckSum, ckPan, ckGain;
  let ckLfoA, ckLfoAAmt, ckLfoB, ckLfoBAmt, ckLfoC, ckLfoCAmt;

  // pooled one-shot voices
  const grainPool = [];
  const clickPool = [];
  const bodyPool = [];
  let grainIdx = 0, clickIdx = 0, bodyIdx = 0;

  // ---------------------------------------------------------------------------
  // Small helpers
  // ---------------------------------------------------------------------------
  const now = () => (ac ? ac.currentTime : 0);

  /** Smoothed gain write. Time constant is short enough to track, long enough not to zip. */
  function gset(param, v, tc) {
    param.setTargetAtTime(v, ac.currentTime, tc || 0.02);
  }
  /** Immediate write for filter coefficients (already damped in JS). */
  function pset(param, v) { param.value = v; }

  function mkGain(v, dest) {
    const g = ac.createGain();
    g.gain.value = v;
    if (dest) g.connect(dest);
    return g;
  }
  function mkFilter(type, freq, q, gainDb, dest) {
    const f = ac.createBiquadFilter();
    f.type = type;
    f.frequency.value = freq;
    if (q !== undefined && q !== null) f.Q.value = q;
    if (gainDb !== undefined && gainDb !== null) f.gain.value = gainDb;
    if (dest) f.connect(dest);
    return f;
  }
  /** A permanently-running looping noise source. */
  function mkNoise(buffer, rate, dest) {
    const s = ac.createBufferSource();
    s.buffer = buffer;
    s.loop = true;
    s.playbackRate.value = rate || 1;
    if (dest) s.connect(dest);
    s.start(0, rng() * (buffer.duration - 0.05));
    return s;
  }
  /** LFO: oscillator -> depth gain, ready to be connected to an AudioParam. */
  function mkLfo(freq, depth, type) {
    const o = ac.createOscillator();
    o.type = type || 'sine';
    o.frequency.value = freq;
    const g = mkGain(depth);
    o.connect(g);
    o.start();
    return [o, g];
  }

  /**
   * Camera-relative pan and distance attenuation. Writes into the shared _pan
   * scratch — no allocation. Falls back to centred/unattenuated if there is no
   * camera yet.
   */
  function spatialise(x, y, z, refDist) {
    const cam = ctx && ctx.camera;
    if (!cam || !cam.matrixWorld) { _pan.pan = 0; _pan.dist = 0; _pan.gain = 1; return _pan; }
    const e = cam.matrixWorld.elements;
    const dx = x - e[12], dy = y - e[13], dz = z - e[14];
    const d = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1e-3;
    // Camera right axis is the first basis column of the world matrix.
    const p = (dx * e[0] + dy * e[1] + dz * e[2]) / d;
    _pan.pan = clamp(p * 1.25, -1, 1);
    _pan.dist = d;
    const r = refDist || 6;
    _pan.gain = clamp01(r / (r + d * d * 0.06));
    return _pan;
  }

  /** Voice budget. Every one-shot goes through this. */
  function claimVoice(src) {
    if (_voices >= MAX_VOICES) return false;
    _voices++;
    src.onended = releaseVoice;
    return true;
  }
  function releaseVoice() { if (_voices > 0) _voices--; this.onended = null; }

  // ===========================================================================
  // Graph construction
  // ===========================================================================
  function build() {
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) { failed = true; return; }
    try {
      ac = new AC({ latencyHint: 'interactive' });
    } catch (e) {
      failed = true;
      return;
    }

    // ---- master ------------------------------------------------------------
    // Order is master -> limiter -> soft clip -> destination. The limiter is a
    // true safety net, not a glue compressor: at -3 dB it should be doing
    // nothing at all during normal riding and only catch a crash landing.
    // Putting the shaper *after* it matters, because a WaveShaper clamps its
    // input to [-1, 1] — feeding it an un-limited peak would hard-clip.
    limiter = ac.createDynamicsCompressor();
    limiter.threshold.value = -3.0;
    limiter.knee.value = 1.5;
    limiter.ratio.value = 20.0;
    limiter.attack.value = 0.002;
    limiter.release.value = 0.13;

    shaper = ac.createWaveShaper();
    shaper.curve = makeSoftClipCurve();
    shaper.oversample = '2x';

    // Starts silent; applyMaster() at the end of start() ramps it up with a
    // 50 ms time constant, so the first gesture never produces a DC click.
    master = mkGain(0.0001);
    master.connect(limiter);
    limiter.connect(shaper);
    shaper.connect(ac.destination);

    // ---- reverb ------------------------------------------------------------
    revSend = mkGain(1.0);

    const irRng = makeRng(subSeed((ctx && ctx.seed) || 1, 'audio:ir'));
    const convForest = ac.createConvolver();
    convForest.normalize = false;
    convForest.buffer = makeImpulseResponse(ac, irRng, {
      // Dense conifer forest: short, dark, heavily diffused, very few discrete
      // reflections (trunks scatter rather than reflect).
      duration: 1.05, preDelay: 0.004, decay: 5.6, damping: 5.5,
      brightness: 0.30, erCount: 5, erSpread: 0.016, erGain: 0.10,
    });
    const convOpen = ac.createConvolver();
    convOpen.normalize = false;
    // The open-terrain IR is by far the most expensive buffer here (2.6 s
    // stereo). Generating it on the next task keeps the frame that handles the
    // player's very first gesture inside budget — that gesture can land
    // mid-run, and a 20 ms hitch there is a visible stutter. Its return gain
    // starts at 0 and update() ramps it with a 0.5 s time constant, so the one
    // frame where the convolver has no buffer is inaudible.
    setTimeout(() => {
      if (!ac || !started) return;
      try {
        convOpen.buffer = makeImpulseResponse(ac, irRng, {
          // Open rock cirque: long, bright, with real slapback off the walls.
          duration: 2.6, preDelay: 0.019, decay: 1.7, damping: 1.05,
          brightness: 0.78, erCount: 9, erSpread: 0.055, erGain: 0.26,
        });
      } catch (e) { /* context died; the forest IR still covers us */ }
    }, 0);

    // Tame the send so the reverb never gets fizzy or muddy.
    const revPre = mkFilter('highpass', 190, 0.7, null, null);
    const revPre2 = mkFilter('lowpass', 7200, 0.7, null, null);
    revSend.connect(revPre);
    revPre.connect(revPre2);
    // Every non-music path sums into busSfx before master, so one gain scales
    // effects (bike, wind, UI, ambience, reverb returns) without touching the
    // music. Perceptual (v^2), same convention as busMusic.
    busSfx = mkGain(sfxVolume * sfxVolume, master);

    revForestGain = mkGain(0.7, busSfx);
    revOpenGain = mkGain(0.0, busSfx);
    revPre2.connect(convForest);
    revPre2.connect(convOpen);
    convForest.connect(revForestGain);
    convOpen.connect(revOpenGain);

    // ---- buses -------------------------------------------------------------
    // Each bus is a summing gain with a fixed dry path and a fixed reverb send.
    function mkBus(dry, send) {
      const b = mkGain(1.0);
      const d = mkGain(dry, busSfx);
      b.connect(d);
      if (send > 0) { const s = mkGain(send, revSend); b.connect(s); }
      return b;
    }
    // busBike has no fixed send of its own — its pooled voices each carry one.
    busBike = mkBus(1.0, 0.0);
    busWind = mkBus(1.0, 0.0);
    busUI = mkBus(1.0, 0.5);
    // Music (src/audio/music/) sums here. No reverb send — the stems arrive
    // with their own space baked in, and the game's convolvers would smear
    // them. Volume is perceptual (v^2) and lives on the bus itself; the duck
    // stage is driven by the music system (SFX activity, crash stingers).
    musicDuck = mkGain(1.0, master);
    busMusic = mkGain(musicVolume * musicVolume, musicDuck);
    // Ambience runs through a duck stage before its own bus split.
    ambDuck = mkGain(1.0);
    busAmb = mkGain(1.0, ambDuck);
    const ambDry = mkGain(0.9, busSfx);
    const ambSend = mkGain(0.42, revSend);
    ambDuck.connect(ambDry);
    ambDuck.connect(ambSend);

    // ---- shared noise beds --------------------------------------------------
    nWhite = makeNoiseBuffer(ac, rng, 'white', NOISE_SECONDS);
    nPink = makeNoiseBuffer(ac, rng, 'pink', NOISE_SECONDS);
    nBrown = makeNoiseBuffer(ac, rng, 'brown', NOISE_SECONDS);

    buildTyre();
    buildGrainPool();
    buildClickPool();
    buildBodyPool();
    buildFreehub();
    buildBrakes();
    buildSkid();
    buildWind();
    buildAmbience();

    started = true;
  }

  // ---------------------------------------------------------------------------
  // Tyre roll
  // ---------------------------------------------------------------------------
  function buildTyre() {
    tyGain = mkGain(0.0, busBike);
    // peaking resonance -> lowpass -> highpass is deliberate: the resonance is a
    // property of the carcass and rides *inside* the surface's brightness band.
    tyHP = mkFilter('highpass', SURF_DEFAULT.hp, 0.7, null, tyGain);
    tyLP = mkFilter('lowpass', SURF_DEFAULT.lp, 0.9, null, tyHP);
    tyPeak = mkFilter('peaking', SURF_DEFAULT.peakF, SURF_DEFAULT.peakQ, SURF_DEFAULT.peakG, tyLP);

    tyBrown = mkGain(SURF_DEFAULT.brown, tyPeak);
    tyPink = mkGain(SURF_DEFAULT.pink, tyPeak);
    tyWhite = mkGain(SURF_DEFAULT.white, tyPeak);

    tySrcBrown = mkNoise(nBrown, 1.0, tyBrown);
    tySrcPink = mkNoise(nPink, 1.0, tyPink);
    tySrcWhite = mkNoise(nWhite, 1.0, tyWhite);

    // Squelch: an LFO on the lowpass cutoff. On mud this is deep and slow and is
    // exactly the "sucking" character of a wet rut; on everything else the depth
    // is zero and the LFO is silent.
    const [osc, amt] = mkLfo(2.2, 0);
    tySquelchOsc = osc;
    tySquelchAmt = amt;
    tySquelchAmt.connect(tyLP.frequency);
  }

  // ---------------------------------------------------------------------------
  // Pooled voices
  // ---------------------------------------------------------------------------
  /**
   * A pooled voice's output stage: pan -> busBike (dry), and pan -> send ->
   * revSend. The send level is set per trigger, because the families sharing
   * this bus want completely different amounts of space — a tyre grain happens
   * under you and wants almost none, a crash should sound like the whole valley
   * heard it.
   */
  function mkVoiceOut() {
    const pan = ac.createStereoPanner();
    pan.connect(busBike);
    const send = mkGain(SEND_GRAIN, revSend);
    pan.connect(send);
    return [pan, send];
  }

  function buildGrainPool() {
    for (let i = 0; i < GRAIN_VOICES; i++) {
      const io = mkVoiceOut();
      const g = mkGain(0, io[0]);
      const bp = mkFilter('bandpass', 1500, 3, null, g);
      grainPool.push({ bp, g, pan: io[0], send: io[1], until: 0 });
    }
  }
  // Struck-metal partials are pure sines whose timbre IS the partial set, so
  // this pool deliberately has no filter — just an envelope and a pan.
  function buildClickPool() {
    for (let i = 0; i < CLICK_VOICES; i++) {
      const io = mkVoiceOut();
      const g = mkGain(0, io[0]);
      clickPool.push({ g, pan: io[0], send: io[1], until: 0 });
    }
  }
  function buildBodyPool() {
    for (let i = 0; i < BODY_VOICES; i++) {
      const io = mkVoiceOut();
      const g = mkGain(0, io[0]);
      const lp = mkFilter('lowpass', 900, 1.0, null, g);
      bodyPool.push({ lp, g, pan: io[0], send: io[1], until: 0 });
    }
  }
  function takeGrain(t) {
    for (let i = 0; i < GRAIN_VOICES; i++) {
      grainIdx = (grainIdx + 1) % GRAIN_VOICES;
      const v = grainPool[grainIdx];
      if (v.until <= t) return v;
    }
    return null;
  }
  function takeClick(t) {
    for (let i = 0; i < CLICK_VOICES; i++) {
      clickIdx = (clickIdx + 1) % CLICK_VOICES;
      const v = clickPool[clickIdx];
      if (v.until <= t) return v;
    }
    return null;
  }
  function takeBody(t) {
    for (let i = 0; i < BODY_VOICES; i++) {
      bodyIdx = (bodyIdx + 1) % BODY_VOICES;
      const v = bodyPool[bodyIdx];
      if (v.until <= t) return v;
    }
    return null;
  }

  // ---------------------------------------------------------------------------
  // Freehub + chain
  // ---------------------------------------------------------------------------
  function buildFreehub() {
    fhGain = mkGain(0.0, busBike);

    // Three resonances: the pawl click itself (bright), the ratchet ring, and a
    // lower body tone from the hub shell. Together these are what makes a
    // freehub identifiable rather than "a buzzy oscillator".
    fhBpA = mkFilter('bandpass', 3100, 7.5, null, fhGain);
    fhBpB = mkFilter('bandpass', 4900, 12.0, null, fhGain);
    fhBpC = mkFilter('bandpass', 1750, 4.0, null, fhGain);
    const fhBpCg = mkGain(0.55);
    fhBpC.disconnect();
    fhBpC.connect(fhBpCg);
    fhBpCg.connect(fhGain);

    fhHP = mkFilter('highpass', 700, 0.7, null, null);
    fhHP.connect(fhBpA);
    fhHP.connect(fhBpB);
    fhHP.connect(fhBpC);

    fhShape = ac.createWaveShaper();
    fhShape.curve = makePulseCurve(0.055);
    fhShape.oversample = '4x';
    fhShape.connect(fhHP);

    fhOsc = ac.createOscillator();
    fhOsc.type = 'sawtooth';
    fhOsc.frequency.value = 40;
    fhOsc.connect(fhShape);
    fhOsc.start();

    // Real ratchets are not metronomic: machining tolerance and wheel flex give
    // a few percent of jitter, and without it this reads as a synth buzz.
    fhJitterSrc = mkNoise(nBrown, 0.35, null);
    fhJitterLP = mkFilter('lowpass', 22, 0.7, null, null);
    fhJitterAmt = mkGain(0);
    fhJitterSrc.connect(fhJitterLP);
    fhJitterLP.connect(fhJitterAmt);
    fhJitterAmt.connect(fhOsc.frequency);
  }

  // ---------------------------------------------------------------------------
  // Brakes: rotor rub + squeal
  // ---------------------------------------------------------------------------
  function buildBrakes() {
    brGain = mkGain(0.0, busBike);
    brPeak = mkFilter('peaking', 2600, 5.5, 7, brGain);
    brBp = mkFilter('bandpass', 1500, 3.2, null, brPeak);
    brSrc = mkNoise(nWhite, 1.0, brBp);
    // Rotor warp: the pad contact modulates once per wheel revolution. This LFO
    // frequency is set from the live wheel spin rate each frame.
    const [lo, la] = mkLfo(2.0, 0.45);
    brLfo = lo; brLfoAmt = la;
    brLfoAmt.connect(brGain.gain);

    // Squeal: a very high-Q resonance excited by both noise (stick-slip grit)
    // and two slightly detuned oscillators (the tonal howl).
    sqGain = mkGain(0.0, busBike);
    sqBp = mkFilter('bandpass', 2800, 26, null, sqGain);
    const sqMix = mkGain(1.0, sqBp);
    sqOscA = ac.createOscillator(); sqOscA.type = 'sawtooth'; sqOscA.frequency.value = 2800;
    sqOscB = ac.createOscillator(); sqOscB.type = 'triangle'; sqOscB.frequency.value = 2807;
    const oa = mkGain(0.22, sqMix), ob = mkGain(0.16, sqMix);
    sqOscA.connect(oa); sqOscB.connect(ob);
    sqOscA.start(); sqOscB.start();
    sqNoiseSrc = mkNoise(nWhite, 1.0, mkGain(0.6, sqMix));
    // Slight vibrato so the howl wavers like a real squeal instead of sitting flat.
    const [vo, va] = mkLfo(7.3, 26);
    sqVib = vo; sqVibAmt = va;
    sqVibAmt.connect(sqOscA.frequency);
    sqVibAmt.connect(sqBp.frequency);
  }

  function buildSkid() {
    skGain = mkGain(0.0, busBike);
    skPeak = mkFilter('peaking', 1400, 2.2, 9, skGain);
    skBp = mkFilter('bandpass', 1400, 1.1, null, skPeak);
    skSrc = mkNoise(nPink, 1.0, skBp);
  }

  // ---------------------------------------------------------------------------
  // Wind
  // ---------------------------------------------------------------------------
  function buildWind() {
    wdPanL = ac.createStereoPanner(); wdPanL.pan.value = -0.4; wdPanL.connect(busWind);
    wdPanR = ac.createStereoPanner(); wdPanR.pan.value = 0.4; wdPanR.connect(busWind);
    wdGainL = mkGain(0, wdPanL);
    wdGainR = mkGain(0, wdPanR);
    wdBpL = mkFilter('bandpass', 500, 0.75, null, wdGainL);
    wdBpR = mkFilter('bandpass', 520, 0.75, null, wdGainR);
    // Two sources on the same buffer at different rates and offsets — fully
    // decorrelated, which is what makes the width real rather than a haas trick.
    wdSrcL = mkNoise(nWhite, 1.0, wdBpL);
    wdSrcR = mkNoise(nWhite, 1.03, wdBpR);

    wdLowGain = mkGain(0, busWind);
    wdLowLP = mkFilter('lowpass', 190, 1.1, null, wdLowGain);
    wdLowSrc = mkNoise(nBrown, 1.0, wdLowLP);

    // The sampled wind-rush loop belongs to the previous context after a
    // dispose/rebuild; update() lazily recreates it once a buffer is decoded.
    wdSmpSrc = wdSmpGain = wdSmpLP = null;
  }

  // ---------------------------------------------------------------------------
  // Ambience: forest bed, birds, creek
  // ---------------------------------------------------------------------------
  function buildAmbience() {
    amPanL = ac.createStereoPanner(); amPanL.pan.value = -0.75; amPanL.connect(busAmb);
    amPanR = ac.createStereoPanner(); amPanR.pan.value = 0.75; amPanR.connect(busAmb);
    amWindGainL = mkGain(0.0, amPanL);
    amWindGainR = mkGain(0.0, amPanR);
    amWindBpL = mkFilter('bandpass', 620, 0.85, null, amWindGainL);
    amWindBpR = mkFilter('bandpass', 700, 0.8, null, amWindGainR);
    amWindSrcL = mkNoise(nPink, 0.85, amWindBpL);
    amWindSrcR = mkNoise(nPink, 0.79, amWindBpR);

    // Two incommensurate LFOs on the band centres: gusts that never obviously
    // repeat. Depth is large — the movement is most of what sells "trees".
    const [la, laa] = mkLfo(0.071, 240);
    amLfoA = la; amLfoAAmt = laa;
    amLfoAAmt.connect(amWindBpL.frequency);
    const [lb, lba] = mkLfo(0.113, 210);
    amLfoB = lb; amLfoBAmt = lba;
    amLfoBAmt.connect(amWindBpR.frequency);
    // The same LFOs, scaled, drive the gains so each gust swells and fades. The
    // depth is deliberately comparable to the bed level itself — a forest bed
    // that does not breathe reads as tape hiss.
    const gA = mkGain(0.018); amLfoA.connect(gA); gA.connect(amWindGainL.gain);
    const gB = mkGain(0.016); amLfoB.connect(gB); gB.connect(amWindGainR.gain);

    // ---- creek -------------------------------------------------------------
    ckGain = mkGain(0.0, busAmb);
    ckPan = ac.createStereoPanner(); ckPan.pan.value = 0; ckPan.connect(ckGain);
    ckSum = mkGain(1.0, ckPan);
    // Moving water is broadband noise shaped by a small number of wandering
    // resonances (the standing waves around each rock). Three is enough.
    ckBpA = mkFilter('bandpass', 900, 1.8, null, mkGain(0.9, ckSum));
    ckBpB = mkFilter('bandpass', 1900, 2.4, null, mkGain(0.7, ckSum));
    ckBpC = mkFilter('bandpass', 3400, 3.0, null, mkGain(0.45, ckSum));
    ckHP = mkFilter('highpass', 260, 0.7, null, null);
    ckHP.connect(ckBpA); ckHP.connect(ckBpB); ckHP.connect(ckBpC);
    ckSrc = mkNoise(nPink, 1.0, ckHP);
    const [ca, caa] = mkLfo(0.107, 220); ckLfoA = ca; ckLfoAAmt = caa; ckLfoAAmt.connect(ckBpA.frequency);
    const [cb, cba] = mkLfo(0.163, 430); ckLfoB = cb; ckLfoBAmt = cba; ckLfoBAmt.connect(ckBpB.frequency);
    const [cc, cca] = mkLfo(0.231, 700); ckLfoC = cc; ckLfoCAmt = cca; ckLfoCAmt.connect(ckBpC.frequency);
  }

  // ===========================================================================
  // One-shot voices
  // ===========================================================================

  // Last sfxPick outcome, surfaced via api.sfxSamples — the one-line answer to
  // "is the recording actually playing, or am I hearing the fallback?"
  let dbgLastSfx = 'none';

  /** Random decoded Suno sample of `kind`, or null (procedural fallback). */
  function sfxPick(kind) {
    const buf = sfxLoader ? sfxLoader.pick(kind, erng()) : null;
    dbgLastSfx = (buf ? 'sample:' : 'fallback:') + kind;
    return buf;
  }

  /**
   * One-shot playback of a decoded sample. Impacts and crashes fire a few
   * times a run at most, so the small node chain is allocated per event and
   * torn down in onended — unlike the pooled grain/click families this can
   * never run at granular rates. Counts against the same voice cap.
   */
  function sample(t, buf, amp, rate, pan, send, lpHz) {
    if (!buf || !started) return false;
    const src = ac.createBufferSource();
    src.buffer = buf;
    src.playbackRate.value = rate || 1;
    if (!claimVoice(src)) return false;
    const g = mkGain(amp);
    const p = ac.createStereoPanner();
    p.pan.value = clamp(pan || 0, -1, 1);
    let head = src;
    let lp = null;
    if (lpHz) { lp = mkFilter('lowpass', lpHz, 0.7, null, null); src.connect(lp); head = lp; }
    head.connect(g);
    g.connect(p);
    p.connect(busBike);
    let s = null;
    if (send > 0) { s = mkGain(send, revSend); p.connect(s); }
    src.onended = () => {
      releaseVoice.call(src);
      try {
        g.disconnect(); p.disconnect();
        if (lp) lp.disconnect();
        if (s) s.disconnect();
      } catch (e) { /* context died first */ }
    };
    src.start(t);
    return src;
  }

  /**
   * A short filtered noise click. The tyre grain layer, the chain slap
   * transient, the rock chips and the freehub-adjacent ticks all come from here.
   */
  function grain(t, freq, q, amp, dur, pan, rateScale, send) {
    if (!started) return;
    const v = takeGrain(t);
    if (!v) return;
    v.send.gain.value = send === undefined ? SEND_GRAIN : send;
    const src = ac.createBufferSource();
    src.buffer = nWhite;
    src.playbackRate.value = rateScale || (0.6 + rng() * 1.6);
    if (!claimVoice(src)) return;
    src.connect(v.bp);
    // The grain pool is shared with rebound()/crash(), which schedule sweeps on
    // this same AudioParam. A plain .value write is ignored while an automation
    // event list is non-empty, so clear it explicitly.
    v.bp.frequency.cancelScheduledValues(t);
    v.bp.frequency.setValueAtTime(clamp(freq, 40, 16000), t);
    pset(v.bp.Q, clamp(q, 0.4, 26));
    v.pan.pan.value = clamp(pan || 0, -1, 1);
    const g = v.g.gain;
    g.cancelScheduledValues(t);
    g.setValueAtTime(0.0001, t);
    g.linearRampToValueAtTime(amp, t + 0.0015);
    g.exponentialRampToValueAtTime(0.0001, t + dur);
    // Playing back faster consumes more of the buffer, so the random read
    // offset has to leave room for rate * duration or the grain truncates.
    const span = Math.max(0, nWhite.duration - (dur + 0.03) * src.playbackRate.value);
    const off = rng() * span;
    src.start(t, off, dur + 0.01);
    src.stop(t + dur + 0.02);
    v.until = t + dur + 0.02;
  }


  /**
   * Suspension compression thunk. A low body tone whose pitch drops as the
   * spring loads, plus a damped noise burst for the oil/bushing noise.
   * `hard` adds the bottom-out clank: bright, inharmonic, metal-on-metal.
   */
  function thunk(t, amp, lowHz, hard, pan) {
    if (!started || amp < 0.008) return;
    const b = takeBody(t);
    if (b) {
      const osc = ac.createOscillator();
      osc.type = 'triangle';
      if (claimVoice(osc)) {
        osc.connect(b.lp);
        b.send.gain.value = SEND_MECH;
        pset(b.lp.frequency, lowHz * 6);
        b.pan.pan.value = clamp(pan || 0, -1, 1);
        osc.frequency.setValueAtTime(lowHz * 1.9, t);
        osc.frequency.exponentialRampToValueAtTime(Math.max(24, lowHz * 0.82), t + 0.085);
        const g = b.g.gain;
        g.cancelScheduledValues(t);
        g.setValueAtTime(0.0001, t);
        g.linearRampToValueAtTime(amp, t + 0.004);
        g.exponentialRampToValueAtTime(0.0001, t + 0.22);
        osc.start(t);
        osc.stop(t + 0.24);
        b.until = t + 0.24;
      }
    }
    // Air/oil rush through the damper.
    grain(t, 260 + rng() * 160, 1.1, amp * 0.55, 0.10, pan, 0.55 + rng() * 0.3, SEND_MECH);
    if (hard > 0.02) {
      // Bottom-out: a recorded damped clank when decoded — a real mechanical
      // thump that layers fine with the impact foley, unlike the synthesised
      // pings, which stay fallback-only and never over a recording.
      const cb = sfxPick('clank');
      if (cb) {
        sample(t + 0.002, cb, clamp(amp * hard * 1.6, 0, 0.5), 0.9 + rng() * 0.2, pan, 0);
      } else if (impactSmpCool <= 0) {
        // Three inharmonic partials plus a bright transient. Inharmonic
        // ratios (not 2:3:4) are what make it read as struck alloy rather than a note.
        const a = amp * hard;
        metalPing(t, 1180, a * 0.5, 0.13, pan, SEND_MECH);
        metalPing(t + 0.002, 1830, a * 0.34, 0.10, pan, SEND_MECH);
        metalPing(t + 0.004, 2870, a * 0.22, 0.075, pan, SEND_MECH);
        grain(t, 3600, 2.0, a * 0.5, 0.045, pan, 1.6, SEND_MECH);
      }
    }
  }

  /** Suspension rebound: a soft downward-sweeping air rush. */
  function rebound(t, amp, pan) {
    if (!started || amp < 0.006) return;
    const v = takeGrain(t);
    if (!v) return;
    const src = ac.createBufferSource();
    src.buffer = nPink;
    src.playbackRate.value = 0.85 + rng() * 0.3;
    if (!claimVoice(src)) return;
    src.connect(v.bp);
    v.send.gain.value = SEND_MECH;
    v.pan.pan.value = clamp(pan || 0, -1, 1);
    pset(v.bp.Q, 1.3);
    v.bp.frequency.cancelScheduledValues(t);
    v.bp.frequency.setValueAtTime(950, t);
    v.bp.frequency.exponentialRampToValueAtTime(280, t + 0.26);
    const g = v.g.gain;
    g.cancelScheduledValues(t);
    g.setValueAtTime(0.0001, t);
    g.linearRampToValueAtTime(amp, t + 0.03);
    g.exponentialRampToValueAtTime(0.0001, t + 0.30);
    src.start(t, rng() * 2.0, 0.34);
    src.stop(t + 0.34);
    v.until = t + 0.34;
  }

  /** A single struck-metal partial. Sine through a fast exponential decay. */
  function metalPing(t, freq, amp, dur, pan, send) {
    if (!started || amp < 0.004) return;
    const v = takeClick(t);
    if (!v) return;
    v.send.gain.value = send === undefined ? SEND_MECH : send;
    const osc = ac.createOscillator();
    osc.type = 'sine';
    osc.frequency.value = freq * (0.985 + rng() * 0.03);
    if (!claimVoice(osc)) return;
    osc.connect(v.g);
    v.pan.pan.value = clamp(pan || 0, -1, 1);
    const g = v.g.gain;
    g.cancelScheduledValues(t);
    g.setValueAtTime(0.0001, t);
    g.linearRampToValueAtTime(amp, t + 0.002);
    g.exponentialRampToValueAtTime(0.0001, t + dur);
    osc.start(t);
    osc.stop(t + dur + 0.01);
    v.until = t + dur + 0.02;
  }

  /**
   * Chain slap — the chain lifting off the guide and hitting the stay. Short,
   * bright, inharmonic, with a noise transient. Randomised so repeats over a
   * rock garden never sound looped.
   */
  function chainSlap(t, amp, pan) {
    if (!started || amp < 0.01) return;
    // A recorded chain-against-frame snap when decoded; the synthesised ping
    // stack below is the fallback (its sine partials read as chimes).
    const b = sfxPick('chain_slap');
    if (b) {
      sample(t, b, clamp(amp * 2.4, 0, 0.5), 0.88 + rng() * 0.28, pan, 0);
      return;
    }
    const detune = 0.92 + rng() * 0.18;
    metalPing(t, 1420 * detune, amp * 0.55, 0.075, pan, SEND_MECH);
    metalPing(t + 0.0015, 2130 * detune, amp * 0.42, 0.055, pan, SEND_MECH);
    metalPing(t + 0.003, 3170 * detune, amp * 0.28, 0.04, pan, SEND_MECH);
    metalPing(t + 0.004, 4310 * detune, amp * 0.16, 0.03, pan, SEND_MECH);
    grain(t, 2800 + rng() * 1400, 1.4, amp * 0.7, 0.028, pan, 1.4 + rng() * 0.8, SEND_MECH);
    // Occasionally a double-slap, which is what actually happens on a big hit.
    if (rng() < 0.32) {
      const t2 = t + 0.035 + rng() * 0.03;
      metalPing(t2, 1960 * detune, amp * 0.3, 0.05, pan, SEND_MECH);
      grain(t2, 3200, 1.6, amp * 0.35, 0.022, pan, 1.6, SEND_MECH);
    }
  }

  /**
   * Layered impact. severity 0..1.
   *   low   — a sine dropping from ~130 Hz to ~42 Hz: the mass of rider+bike
   *   noise — the ground itself, filtered by the surface under the wheel
   *   body   — inharmonic partials: frame, rims, pads rattling in the calipers
   */
  function impact(t, severity, surfIdx, x, y, z) {
    if (!started) return;
    const sev = clamp01(severity);
    if (sev < 0.02) return;
    const surf = SURF[surfIdx | 0] || SURF_DEFAULT;
    let pan = 0, atten = 1;
    if (x !== undefined) { spatialise(x, y, z, 9); pan = _pan.pan * 0.5; atten = lerp(0.6, 1.0, _pan.gain); }
    const amp = (0.10 + 0.5 * sev * sev) * atten;

    // When a foley sample is available it IS the impact; the synthesised
    // layers drop to a thin support bed (and lose most of their reverb send),
    // otherwise they drown the recording in the old "bucket" sound.
    // impactSmpCool > 0 means a crash take is already carrying this hit — no
    // extra sample, but the support bed still ducks under the recording.
    const smpActive = impactSmpCool > 0;
    const smp = (sev > 0.12 && !smpActive)
      ? sfxPick(sev > 0.45 ? 'impact_hard' : 'impact_soft') : null;
    const sup = (smp || smpActive) ? 0.3 : 1;
    const sendHit = (smp || smpActive) ? SEND_HIT * 0.1 : SEND_HIT;

    // --- low body drop -------------------------------------------------------
    const b = takeBody(t);
    if (b) {
      const osc = ac.createOscillator();
      osc.type = 'sine';
      if (claimVoice(osc)) {
        osc.connect(b.lp);
        b.send.gain.value = sendHit;
        pset(b.lp.frequency, 600);
        b.pan.pan.value = pan * 0.3;
        const f0 = lerp(95, 165, sev);
        osc.frequency.setValueAtTime(f0, t);
        osc.frequency.exponentialRampToValueAtTime(lerp(52, 38, sev), t + 0.13);
        const g = b.g.gain;
        g.cancelScheduledValues(t);
        g.setValueAtTime(0.0001, t);
        g.linearRampToValueAtTime(amp * 1.15 * sup, t + 0.004);
        g.exponentialRampToValueAtTime(0.0001, t + 0.28 + sev * 0.2);
        osc.start(t);
        osc.stop(t + 0.52);
        b.until = t + 0.52;
      }
    }
    // --- ground noise burst, coloured by the surface -------------------------
    const nf = lerp(surf.lp * 0.45, surf.lp * 0.9, sev);
    grain(t, clamp(nf, 120, 6000), 0.9, amp * 0.85 * sup, 0.10 + sev * 0.11, pan, 0.6 + rng() * 0.5, sendHit);
    grain(t + 0.006, clamp(nf * 1.9, 180, 9000), 1.4, amp * 0.4 * sup, 0.055, pan, 1.2, sendHit);
    // --- bike body -----------------------------------------------------------
    // The sampled landing foley, played at full level and full bandwidth (it
    // is already EQ'd); the synthesised metal pings read as falling pipes, so
    // they are only the not-yet-loaded fallback.
    if (smp) {
      // Completely dry: the take carries its own space, any convolver share
      // reads as "hit in a distant pot". The chain slap is also skipped — its
      // sine partials chime over a recording.
      const s = sample(t + 0.002, smp, clamp(amp * 1.8, 0, 1.0), 0.92 + rng() * 0.16, pan, 0);
      if (s) {
        impactSmpCool = Math.max(impactSmpCool, 0.25);
        lastImpactSmp = s;
        lastImpactSmpT = t;
      }
    } else if (sev > 0.12 && !smpActive) {
      const m = amp * (0.20 + sev * 0.45);
      metalPing(t + 0.004, 385 * (0.95 + rng() * 0.1), m * 0.6, 0.20, pan, SEND_HIT);
      metalPing(t + 0.006, 613 * (0.95 + rng() * 0.1), m * 0.42, 0.16, pan, SEND_HIT);
      metalPing(t + 0.008, 971 * (0.95 + rng() * 0.1), m * 0.26, 0.12, pan, SEND_HIT);
      if (sev > 0.35) chainSlap(t + 0.012, m * 0.9, pan);
    }
    duck(0.35 + sev * 0.45, 0.06, 0.45 + sev * 0.5);
  }

  /** Full crash: a big impact, a tumble of clanks, and a long dirt scrape. */
  function crash(severity, x, y, z) {
    if (!started) return;
    const t = now();
    const sev = clamp01(severity === undefined ? 0.7 : severity);
    // Pick the crash take BEFORE the impact layer: when one exists, it carries
    // the hit itself, so impact() must skip its own recorded thud (and any
    // bike:impact arriving in the same tumble stays sample-free too).
    const cs = sfxPick('crash');
    if (cs) impactSmpCool = 0.7;
    impact(t, Math.max(0.55, sev), (ctx.bike && ctx.bike.state && ctx.bike.state.surface) | 0, x, y, z);
    let pan = 0;
    if (x !== undefined) { spatialise(x, y, z, 9); pan = _pan.pan * 0.5; }

    // Tumble + slide. Sampled foley when decoded (a real crash take plus a
    // dirt-scrape tail); otherwise the synthesised clank sequence.
    if (cs) {
      // A bike:impact from the same slam may have scheduled its own take a few
      // milliseconds ago — stop it; two recordings 30 ms apart comb into a
      // metallic "distant pot". The crash take owns the hit.
      if (lastImpactSmp && t - lastImpactSmpT < 0.12) {
        try { lastImpactSmp.stop(t); } catch (e) { /* already ended */ }
        lastImpactSmp = null;
      }
      // Completely dry: the takes carry their own recorded space, and any
      // convolver share turns them into a washed-out bucket.
      sample(t + 0.02, cs, 0.6 + sev * 0.35, 0.94 + erng() * 0.12, pan, 0);
      const sl = sfxPick('slide');
      // The slide take starts as the tumble dies down, slower for big crashes.
      if (sl) sample(t + 0.28 + erng() * 0.15, sl, 0.32 + sev * 0.2, 0.86 + erng() * 0.2, pan, 0);
    } else {
      // A decaying sequence of frame/wheel clanks at irregular intervals.
      let tt = t + 0.09;
      const n = 4 + Math.floor(erng() * 5);
      for (let i = 0; i < n; i++) {
        const a = (0.16 + sev * 0.2) * Math.pow(0.72, i) * (0.6 + erng() * 0.8);
        metalPing(tt, (520 + erng() * 900) * (1 - i * 0.05), a, 0.09 + erng() * 0.1, pan + (erng() - 0.5) * 0.4, SEND_CRASH);
        metalPing(tt + 0.003, (1400 + erng() * 1800), a * 0.5, 0.06, pan, SEND_CRASH);
        grain(tt, 300 + erng() * 900, 0.9, a * 0.9, 0.09, pan, 0.5 + erng(), SEND_CRASH);
        tt += 0.055 + erng() * 0.16;
      }
      // Body and bike sliding through dirt: broadband noise sweeping down.
      const v = takeGrain(t);
      if (v) {
        const src = ac.createBufferSource();
        src.buffer = nPink;
        src.playbackRate.value = 0.9;
        if (claimVoice(src)) {
          src.connect(v.bp);
          v.send.gain.value = SEND_CRASH;
          v.pan.pan.value = pan;
          pset(v.bp.Q, 0.8);
          v.bp.frequency.cancelScheduledValues(t);
          v.bp.frequency.setValueAtTime(1500, t + 0.05);
          v.bp.frequency.exponentialRampToValueAtTime(320, t + 1.0);
          const g = v.g.gain;
          g.cancelScheduledValues(t);
          g.setValueAtTime(0.0001, t + 0.04);
          g.linearRampToValueAtTime(0.16 + sev * 0.16, t + 0.12);
          g.exponentialRampToValueAtTime(0.0001, t + 1.15);
          src.start(t + 0.04, erng() * 2.0, 1.2);
          src.stop(t + 1.25);
          v.until = t + 1.25;
        }
      }
    }
    grunt(t + 0.02, 0.85 + erng() * 0.15, true);
    duck(0.75, 0.05, 1.6);
    // "You lost the run" cue: the recorded descending notes, half a second
    // after the hit so they read as a comment, not as part of the crash. The
    // old synthesised dyad (chimes-with-reverb over the foley) stays
    // fallback-only.
    const uf = sfxPick('ui_fall');
    if (uf) sample(t + 0.5, uf, 0.2, 1, 0, 0);
    else if (!cs) uiFall(t + 0.10);
  }

  /**
   * Rider vocal. A sawtooth glottal source through three formant bandpasses —
   * the cheapest thing that actually reads as a human voice rather than a
   * synth. `hard` opens the vowel and pushes the pitch up (a real grunt).
   */
  function grunt(t, amp, hard) {
    if (!started || amp < 0.02) return;
    const v = takeBody(t);
    if (!v) return;
    const osc = ac.createOscillator();
    osc.type = 'sawtooth';
    if (!claimVoice(osc)) return;

    // Formants for a mid-central /ʌ/ ("uh"), which is what effort sounds like.
    const f1 = mkFilter('bandpass', hard ? 700 : 610, 9, null, null);
    // Pool constraint: the body voice only has one filter, so build the vocal
    // tract on the fly. This happens at most a few times a second and only on
    // landings, so it is well inside budget.
    const f2 = mkFilter('bandpass', hard ? 1290 : 1150, 11, null, null);
    const f3 = mkFilter('bandpass', 2480, 13, null, null);
    const m1 = mkGain(1.0, v.g), m2 = mkGain(0.5, v.g), m3 = mkGain(0.18, v.g);
    f1.connect(m1); f2.connect(m2); f3.connect(m3);
    osc.connect(f1); osc.connect(f2); osc.connect(f3);
    v.send.gain.value = SEND_RIDER;
    v.pan.pan.value = 0;

    const p0 = (hard ? 148 : 116) * (0.94 + rng() * 0.12);
    osc.frequency.setValueAtTime(p0 * 1.06, t);
    osc.frequency.exponentialRampToValueAtTime(p0 * 0.72, t + (hard ? 0.28 : 0.34));
    const dur = hard ? 0.30 : 0.40;
    const g = v.g.gain;
    g.cancelScheduledValues(t);
    g.setValueAtTime(0.0001, t);
    g.linearRampToValueAtTime(amp * 0.16, t + 0.018);
    g.exponentialRampToValueAtTime(0.0001, t + dur);
    osc.start(t);
    osc.stop(t + dur + 0.02);
    v.until = t + dur + 0.03;
    // Breath noise rides on top; without it a grunt sounds like a buzzer.
    grain(t + 0.005, 780, 0.8, amp * 0.09, dur * 0.7, 0, 0.7, SEND_RIDER);
    osc.onended = () => {
      releaseVoice.call(osc);
      f1.disconnect(); f2.disconnect(); f3.disconnect();
      m1.disconnect(); m2.disconnect(); m3.disconnect();
    };
  }

  /** Breathing. Inhale is brighter and shorter; exhale is dark and longer. */
  function breath(t, inhale, amp) {
    if (!started || amp < 0.006) return;
    const v = takeGrain(t);
    if (!v) return;
    const src = ac.createBufferSource();
    src.buffer = nPink;
    src.playbackRate.value = inhale ? 1.1 : 0.85;
    if (!claimVoice(src)) return;
    src.connect(v.bp);
    v.send.gain.value = SEND_RIDER;
    v.pan.pan.value = 0;
    pset(v.bp.Q, inhale ? 1.3 : 0.9);
    const f0 = inhale ? 560 : 380;
    const f1 = inhale ? 700 : 260;
    const dur = inhale ? 0.32 : 0.46;
    v.bp.frequency.cancelScheduledValues(t);
    v.bp.frequency.setValueAtTime(f0, t);
    v.bp.frequency.exponentialRampToValueAtTime(f1, t + dur);
    const g = v.g.gain;
    g.cancelScheduledValues(t);
    g.setValueAtTime(0.0001, t);
    g.linearRampToValueAtTime(amp, t + dur * 0.32);
    g.exponentialRampToValueAtTime(0.0001, t + dur);
    src.start(t, rng() * 2.0, dur + 0.05);
    src.stop(t + dur + 0.06);
    v.until = t + dur + 0.06;
  }

  /**
   * Chowning FM bell/mallet — the voice for every run-event tone. A modest
   * modulation index with an inharmonic ratio gives a struck-metal attack that
   * settles into a clean sine, which is what "tasteful UI tone" means here.
   */
  function bell(t, freq, amp, dur, ratio, index, pan) {
    if (!started || amp < 0.002) return;
    const car = ac.createOscillator();
    const mod = ac.createOscillator();
    car.type = 'sine'; mod.type = 'sine';
    if (!claimVoice(car)) return;
    car.frequency.value = freq;
    mod.frequency.value = freq * (ratio || 2.0);
    const modGain = mkGain(freq * (index || 1.4));
    const amp1 = mkGain(0.0001);
    const p = ac.createStereoPanner();
    p.pan.value = clamp(pan || 0, -1, 1);
    mod.connect(modGain);
    modGain.connect(car.frequency);
    car.connect(amp1);
    amp1.connect(p);
    p.connect(busUI);
    // Modulation index decays much faster than amplitude: bright strike, pure tail.
    modGain.gain.setValueAtTime(freq * (index || 1.4), t);
    modGain.gain.exponentialRampToValueAtTime(freq * 0.02, t + dur * 0.28);
    const g = amp1.gain;
    g.setValueAtTime(0.0001, t);
    g.linearRampToValueAtTime(amp, t + 0.006);
    g.exponentialRampToValueAtTime(0.0001, t + dur);
    car.start(t); mod.start(t);
    car.stop(t + dur + 0.03); mod.stop(t + dur + 0.03);
    car.onended = () => {
      releaseVoice.call(car);
      amp1.disconnect(); modGain.disconnect(); p.disconnect(); mod.disconnect();
    };
  }

  /** Warm low sine pad under the run tones — gives the UI weight without a thud. */
  function pad(t, freq, amp, dur) {
    if (!started) return;
    const osc = ac.createOscillator();
    osc.type = 'sine';
    osc.frequency.value = freq;
    if (!claimVoice(osc)) return;
    const g = mkGain(0.0001, busUI);
    osc.connect(g);
    g.gain.setValueAtTime(0.0001, t);
    g.gain.linearRampToValueAtTime(amp, t + dur * 0.25);
    g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
    osc.start(t);
    osc.stop(t + dur + 0.03);
    osc.onended = () => { releaseVoice.call(osc); g.disconnect(); };
  }

  /** Descending detuned dyad for the crash — a "you lost the run" cue, not a buzzer. */
  function uiFall(t) {
    if (!started) return;
    for (let i = 0; i < 2; i++) {
      const osc = ac.createOscillator();
      osc.type = 'triangle';
      if (!claimVoice(osc)) return;
      const f = i === 0 ? 146.83 : 110.0;          // D3 and A2
      const g = mkGain(0.0001, busUI);
      osc.connect(g);
      osc.frequency.setValueAtTime(f * 1.02, t);
      osc.frequency.exponentialRampToValueAtTime(f * 0.72, t + 0.85);
      g.gain.setValueAtTime(0.0001, t);
      g.gain.linearRampToValueAtTime(0.075, t + 0.05);
      g.gain.exponentialRampToValueAtTime(0.0001, t + 0.95);
      osc.start(t); osc.stop(t + 1.0);
      osc.onended = () => { releaseVoice.call(osc); g.disconnect(); };
    }
  }

  /** A single bird note: FM carrier with a pitch glide. */
  function birdNote(t, f0, f1, dur, ratio, index, amp, pan, bright) {
    if (!started || amp < 0.002) return;
    const car = ac.createOscillator();
    const mod = ac.createOscillator();
    car.type = 'sine'; mod.type = 'sine';
    if (!claimVoice(car)) return;
    const modGain = mkGain(f0 * index);
    const amp1 = mkGain(0.0001);
    const tone = mkFilter('bandpass', f0 * (bright || 1), 1.1, null, null);
    const p = ac.createStereoPanner();
    p.pan.value = clamp(pan, -1, 1);
    mod.connect(modGain);
    modGain.connect(car.frequency);
    car.connect(amp1);
    amp1.connect(tone);
    tone.connect(p);
    p.connect(busAmb);

    car.frequency.setValueAtTime(f0, t);
    car.frequency.exponentialRampToValueAtTime(Math.max(40, f1), t + dur);
    mod.frequency.setValueAtTime(f0 * ratio, t);
    mod.frequency.exponentialRampToValueAtTime(Math.max(40, f1 * ratio), t + dur);
    modGain.gain.setValueAtTime(f0 * index, t);
    modGain.gain.exponentialRampToValueAtTime(Math.max(1, f0 * index * 0.25), t + dur);
    tone.frequency.setValueAtTime(f0 * (bright || 1), t);
    tone.frequency.exponentialRampToValueAtTime(Math.max(80, f1 * (bright || 1)), t + dur);

    const atk = Math.min(0.012, dur * 0.3);
    amp1.gain.setValueAtTime(0.0001, t);
    amp1.gain.linearRampToValueAtTime(amp, t + atk);
    amp1.gain.setValueAtTime(amp, t + dur * 0.7);
    amp1.gain.exponentialRampToValueAtTime(0.0001, t + dur);
    car.start(t); mod.start(t);
    car.stop(t + dur + 0.02); mod.stop(t + dur + 0.02);
    car.onended = () => {
      releaseVoice.call(car);
      amp1.disconnect(); modGain.disconnect(); tone.disconnect(); p.disconnect(); mod.disconnect();
    };
  }

  /** Pick and schedule a whole bird song. Everything is scheduled ahead — no per-frame cost. */
  function birdSong() {
    if (!started) return;
    // Weighted pick by how exposed the terrain is: no ravens in the trees, no
    // doves above the treeline.
    const open = S.openness;
    let total = 0;
    for (let i = 0; i < BIRD_SONGS.length; i++) {
      total += lerp(BIRD_FOREST_WEIGHT[i], BIRD_OPEN_WEIGHT[i], open);
    }
    let r = erng() * total, pick = 0;
    for (let i = 0; i < BIRD_SONGS.length; i++) {
      r -= lerp(BIRD_FOREST_WEIGHT[i], BIRD_OPEN_WEIGHT[i], open);
      if (r <= 0) { pick = i; break; }
    }
    const song = BIRD_SONGS[pick];
    // Distance: most birds are far away and reverberant; a few are close.
    const far = erng();
    const dist = lerp(0.28, 1.0, far * far);
    const transpose = 0.9 + erng() * 0.22;
    const pan = (erng() * 2 - 1) * 0.85;
    // Birds go quiet when you are on the gas — masking, and it keeps the mix clean.
    const speedMask = lerp(1.0, 0.22, clamp01(S.speedN * 1.4));
    const amp = 0.052 * song.gain * dist * speedMask * lerp(0.45, 1.0, 1 - open * 0.55);
    if (amp < 0.003) return;

    let t = now() + 0.03;
    const n = song.notes;
    for (let i = 0; i + 5 < n.length; i += 6) {
      birdNote(t, n[i] * transpose, n[i + 1] * transpose, n[i + 2], n[i + 4], n[i + 5],
        amp * (0.85 + erng() * 0.3), pan + (erng() - 0.5) * 0.12, song.bright);
      t += n[i + 2] + n[i + 3];
    }
  }

  /** A creek "bloop" — a bubble collapsing. Downward pitch sweep, very short. */
  function bloop(t, amp, pan) {
    if (!started || amp < 0.002) return;
    const osc = ac.createOscillator();
    osc.type = 'sine';
    if (!claimVoice(osc)) return;
    const g = mkGain(0.0001);
    const p = ac.createStereoPanner();
    p.pan.value = clamp(pan, -1, 1);
    osc.connect(g); g.connect(p); p.connect(busAmb);
    const f = 420 + erng() * 900;
    osc.frequency.setValueAtTime(f, t);
    osc.frequency.exponentialRampToValueAtTime(f * 0.42, t + 0.055);
    g.gain.setValueAtTime(0.0001, t);
    g.gain.linearRampToValueAtTime(amp, t + 0.004);
    g.gain.exponentialRampToValueAtTime(0.0001, t + 0.07);
    osc.start(t); osc.stop(t + 0.09);
    osc.onended = () => { releaseVoice.call(osc); g.disconnect(); p.disconnect(); };
  }

  /** Duck the ambience under a transient. */
  function duck(amount, attack, release) {
    if (!started) return;
    const t = now();
    const floor = clamp01(1 - amount);
    ambDuck.gain.cancelScheduledValues(t);
    ambDuck.gain.setTargetAtTime(floor, t, Math.max(0.01, attack));
    ambDuck.gain.setTargetAtTime(1.0, t + attack * 3 + 0.02, Math.max(0.05, release * 0.4));
  }

  // ===========================================================================
  // Event wiring
  // ===========================================================================
  const offs = [];
  function wireEvents() {
    if (!ctx || !ctx.events) return;
    const on = (name, fn) => { const o = ctx.events.on(name, fn); if (typeof o === 'function') offs.push(o); else offs.push(() => ctx.events.off(name, fn)); };

    on('bike:impact', (p) => {
      if (!started || !p) return;
      const pos = p.position;
      impact(now(), p.severity || 0, p.surface | 0, pos && pos.x, pos && pos.y, pos && pos.z);
    });

    on('bike:skid', (p) => {
      // The continuous skid layer is driven from state.skid in update(); this
      // event only nudges it so a short chirp still registers between frames.
      if (!p) return;
      const i = clamp01(p.intensity || 0);
      if (i > S.skid) S.skid = i;
    });

    on('bike:landed', (p) => {
      if (!started || !p) return;
      const air = p.airTime || 0;
      const q = p.quality === undefined ? 1 : clamp01(p.quality);
      // A grunt on anything meaningful; harder and louder if it was cased.
      if (air > 0.75 || p.cased) {
        grunt(now() + 0.015, clamp01(0.35 + air * 0.35 + (1 - q) * 0.5), air > 1.2 || p.cased);
      }
      S.effort = clamp01(S.effort + air * 0.18 + (1 - q) * 0.22);
    });

    on('run:crash', (p) => {
      // bike.js emits both 'bike:crash' and 'run:crash', and gameplay may emit
      // its own — collapse anything inside 0.4 s into one crash.
      if (!started || crashCool > 0) return;
      crashCool = 0.4;
      const pos = p && p.position;
      crash(p && p.severity, pos && pos.x, pos && pos.y, pos && pos.z);
      S.effort = 1;
    });

    on('run:start', () => {
      if (!started) return;
      runTime = 0; lastCheckpoint = 0;
      S.effort = 0.18; S.stamina = 1;
      const t = now() + 0.02;
      // Low swell, then a clean two-note lift. Deliberately understated.
      pad(t, 110, 0.05, 0.7);
      bell(t, 293.66, 0.10, 0.55, 2.0, 1.1, -0.12);        // D4
      bell(t + 0.13, 440.0, 0.115, 0.9, 2.0, 1.0, 0.12);   // A4
    });

    on('run:checkpoint', (p) => {
      if (!started) return;
      const idx = (p && (p.index !== undefined ? p.index : p.checkpoint)) || lastCheckpoint;
      lastCheckpoint = idx + 1;
      const note = PENT[clamp(3 + (idx % 6), 0, PENT.length - 1)];
      const t = now() + 0.01;
      bell(t, note, 0.085, 0.5, 3.0, 0.9, (erng() - 0.5) * 0.3);
      bell(t + 0.004, note * 2, 0.03, 0.35, 2.0, 0.6, 0);
    });

    on('run:finish', () => {
      if (!started) return;
      const t = now() + 0.02;
      pad(t, 110, 0.055, 1.6);
      bell(t + 0.00, 440.00, 0.10, 0.8, 2.0, 1.0, -0.18);  // A4
      bell(t + 0.11, 523.25, 0.10, 0.9, 2.0, 1.0, 0.06);   // C5
      bell(t + 0.22, 659.25, 0.115, 1.9, 2.0, 0.95, 0.18); // E5
      bell(t + 0.24, 880.00, 0.045, 2.2, 3.0, 0.7, -0.1);  // A5 shimmer
      duck(0.3, 0.08, 1.2);
    });

    on('trick:landed', (p) => {
      if (!started) return;
      const q = clamp01((p && (p.quality !== undefined ? p.quality : p.score / 500)) || 0.6);
      // Recorded UI pluck only. NO synth fallback: the old pentatonic bell
      // landed in the same instant as the impact foley and read as a metallic
      // "pot" overtone on every jump.
      const b = sfxPick('ui_trick');
      if (b) sample(now() + 0.01, b, 0.16 + q * 0.1, 0.96 + q * 0.08, 0, 0);
    });

    on('water:splash', (p) => {
      if (!started || !p) return;
      const pos = p.position;
      const v = p.velocity;
      const sp = v ? Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) : 6;
      const a = clamp01(sp / 14) * 0.35;
      let pan = 0;
      if (pos) { spatialise(pos.x, pos.y, pos.z, 8); pan = _pan.pan * 0.5; }
      const t = now();
      grain(t, 900, 0.7, a, 0.18, pan, 0.7, SEND_HIT);
      grain(t + 0.01, 2600, 1.1, a * 0.7, 0.12, pan, 1.4, SEND_HIT);
      for (let i = 0; i < 4; i++) bloop(t + 0.02 + erng() * 0.12, a * 0.5 * erng(), pan + (erng() - 0.5) * 0.4);
    });

    on('quality:changed', () => { /* nothing quality-dependent in the audio graph */ });
  }

  // ===========================================================================
  // Gesture / focus handling
  // ===========================================================================
  function start() {
    if (started || failed) return;
    build();
    if (!started) return;
    if (ac.state === 'suspended' && ac.resume) ac.resume().catch(() => {});
    applyMaster();
  }

  const onGesture = () => { start(); tryResume(); };
  window.addEventListener('pointerdown', onGesture);
  window.addEventListener('keydown', onGesture);
  window.addEventListener('touchstart', onGesture, { passive: true });

  // Browsers auto-suspend an AudioContext on a hidden tab. Coming back is not a
  // fresh user gesture, but resume() is allowed because the context was already
  // unlocked once — without this the game returns from a background tab silent.
  function tryResume() {
    if (ac && ac.state === 'suspended' && ac.resume) ac.resume().catch(() => {});
  }
  const onBlur = () => { windowBlurred = true; applyMaster(); };
  const onFocus = () => { windowBlurred = false; tryResume(); applyMaster(); };
  const onVisibility = () => {
    docHidden = document.hidden;
    if (!docHidden) tryResume();
    applyMaster();
  };
  window.addEventListener('blur', onBlur);
  window.addEventListener('focus', onFocus);
  document.addEventListener('visibilitychange', onVisibility);

  function targetMaster() {
    if (muted || externalMuted || windowBlurred || docHidden) return 0.0;
    // Perceptual curve — a linear slider on a linear gain feels wrong at the top.
    return Math.pow(masterVolume, 1.6) * 0.9 * S.pauseGain;
  }
  function applyMaster() {
    if (!started || !ac) return;
    const v = targetMaster();
    master.gain.setTargetAtTime(Math.max(0.00005, v), ac.currentTime, 0.05);
  }

  wireEvents();

  // ===========================================================================
  // Environment probes (throttled — these are heightfield lookups, not free)
  // ===========================================================================
  function probeEnvironment(x, y, z) {
    const terrain = ctx && ctx.terrain;
    if (!terrain || typeof terrain.sampleMaterial !== 'function') return;

    // Openness: how much sky and rock is around the rider. Above the treeline is
    // fully exposed; dense conifers below it are the tightest space on the hill.
    let open = 0.3;
    if (typeof terrain.treelineAt === 'function') {
      const tl = terrain.treelineAt(x, z);
      open = clamp01((y - (tl - 45)) / 95);
    }
    const mat = terrain.sampleMaterial(x, z) | 0;
    if (mat === Surface.ROCK) open = Math.max(open, 0.60);    // cliff bands ring
    if (mat === Surface.SNOW) open = Math.max(open, 0.85);    // always above the trees
    if (mat === Surface.GRAVEL) open = Math.max(open, 0.42);  // talus fan
    if (mat === Surface.GRASS) open = Math.max(open, 0.30);   // meadow
    if (typeof terrain.sampleSlope === 'function') {
      // A steep slope means the trees around you are below eye level.
      open = Math.max(open, clamp01((terrain.sampleSlope(x, z) - 0.6) / 0.55) * 0.7);
    }
    S.openness = damp(S.openness, clamp01(open), 0.8, 0.2);

    if (typeof terrain.sampleWetness === 'function') {
      S.wetness = damp(S.wetness, clamp01(terrain.sampleWetness(x, z)), 2.5, 0.2);
    }
    if (terrain.bounds) {
      const span = Math.max(1, (terrain.bounds.maxY || 400) - (terrain.bounds.minY || 0));
      S.altitude = clamp01((y - (terrain.bounds.minY || 0)) / span);
    }
  }

  /**
   * Nearest point on the creek centreline. water.creekPath is a flat
   * [x,y,z,...] Float32Array, so this is a strided scan with no allocation.
   * Throttled to a few times a second; the creek does not move.
   */
  function probeCreek(x, y, z) {
    const water = ctx && ctx.water;
    let best = 1e9, bx = 0, bz = 0;
    const path = water && water.creekPath;
    if (path && path.length >= 6) {
      const stride = 3 * (path.length > 900 ? 2 : 1);
      for (let i = 0; i < path.length - 2; i += stride) {
        const dx = path[i] - x, dz = path[i + 2] - z;
        const d = dx * dx + dz * dz;
        if (d < best) { best = d; bx = path[i]; bz = path[i + 2]; }
      }
      best = Math.sqrt(best);
    } else if (water && typeof water.depthAt === 'function') {
      best = water.depthAt(x, z) > 0.02 ? 2 : 1e9;
      bx = x; bz = z;
    }
    // Audible from ~55 m, loud within ~6 m.
    const near = clamp01(1 - (best - 5) / 55);
    S.creek = damp(S.creek, near * near, 1.2, 0.25);
    if (best < 1e8) {
      spatialise(bx, y, bz, 20);
      S.creekPan = damp(S.creekPan, _pan.pan * 0.75, 2.0, 0.25);
    }
  }

  // ===========================================================================
  // update()
  // ===========================================================================
  function update(dt, c) {
    const context = c || ctx;
    const d = clamp(dt || 0, 0, 1 / 20);

    // Pause / menu state gates the master even before the graph exists.
    const gp = context.gameplay;
    const paused = !!(gp && (gp.state === 'paused' || gp.state === 'menu'));
    const pauseTarget = paused ? (gp && gp.state === 'menu' ? 0.65 : 0.0) : 1.0;
    const prevPause = S.pauseGain;
    S.pauseGain = damp(S.pauseGain, pauseTarget, 6, d);
    if (started && Math.abs(S.pauseGain - prevPause) > 1e-4) applyMaster();

    // SFX samples: fetch strictly after platform ready (frame 3, same rule as
    // music) — needs no AudioContext; decode drains once the graph exists.
    if (!sfxLoader && context.frame > 3) {
      sfxLoader = createSfxLoader('./sfx/');
      sfxLoader.start();
    }
    if (!started) return;
    if (ac.state === 'suspended') return;   // resumed by the next gesture
    if (sfxLoader) sfxLoader.drainDecodes(ac);

    const t = ac.currentTime;
    const bike = context.bike;
    const st = bike && bike.state;
    if (crashCool > 0) crashCool -= d;
    if (impactSmpCool > 0) impactSmpCool -= d;

    // ---- read the bike, defensively ---------------------------------------
    const speed = st ? (st.speed || 0) : 0;
    const speedN = clamp01(speed / REF_SPEED);
    S.speed = damp(S.speed, speed, 14, d);
    S.speedN = damp(S.speedN, speedN, 10, d);

    const wheels = st && st.wheels;
    const wf = wheels && wheels[0];
    const wr = wheels && wheels[1];
    const contactN = (wf && wf.contact ? 0.5 : 0) + (wr && wr.contact ? 0.5 : 0);
    S.contact = damp(S.contact, st ? contactN : 1, 12, d);
    S.airborne = damp(S.airborne, st && st.airborne ? 1 : 0, 11, d);

    const loadRaw = wr ? clamp01((wr.load || 0) / 1400) : 0.55;
    S.load = damp(S.load, loadRaw, 6, d);
    S.rearSpin = damp(S.rearSpin, wr ? Math.abs(wr.spinRate || 0) : speed / WHEEL_RADIUS, 12, d);
    S.frontSpin = damp(S.frontSpin, wf ? Math.abs(wf.spinRate || 0) : S.rearSpin, 12, d);
    S.gForce = damp(S.gForce, st ? (st.gForce || 1) : 1, 8, d);
    S.rough = damp(S.rough, st && st.roughness !== undefined ? clamp01(st.roughness) : 0.3, 3, d);
    S.brakeF = damp(S.brakeF, st ? clamp01(st.brakeFront || 0) : 0, 14, d);
    S.brakeR = damp(S.brakeR, st ? clamp01(st.brakeRear || 0) : 0, 14, d);
    S.pedal = damp(S.pedal, st ? clamp01(st.pedalling || 0) : 0, 8, d);
    S.yaw = damp(S.yaw, st && st.yawRate !== undefined ? st.yawRate : 0, 9, d);

    // Skid: prefer the bike's own scalar; fall back to slip ratios.
    let skidRaw = 0;
    if (st && st.skid !== undefined) skidRaw = clamp01(st.skid);
    else if (wr) skidRaw = clamp01(Math.abs(wr.slipRatio || 0) * 1.3 + Math.abs(wr.slipAngle || 0) * 1.1);
    S.skid = damp(S.skid, Math.max(skidRaw, S.skid * 0.6), 9, d);

    // Lockup: heavy brake with the wheel turning far slower than the ground.
    let lock = 0;
    if (wr && speed > 2) {
      const rolling = speed / WHEEL_RADIUS;
      lock = clamp01(1 - Math.abs(wr.spinRate || 0) / Math.max(1e-3, rolling));
    }
    const brakeMax = Math.max(S.brakeF, S.brakeR);
    S.lockup = damp(S.lockup, lock * brakeMax, 7, d);

    // Coasting: not pedalling, rear wheel turning, still upright.
    const coastRaw = (1 - S.pedal) * clamp01(S.rearSpin / 5) * (st && st.crashed ? 0.25 : 1);
    S.coast = damp(S.coast, coastRaw, 7, d);

    // ---- surface voicing crossfade ----------------------------------------
    const surfIdx = st ? ((st.surface | 0) % SURF.length) : 0;
    const sf = SURF[surfIdx] || SURF_DEFAULT;
    // ~0.28 s to swap character: fast enough to track a rock garden, slow enough
    // that a one-frame material flicker at a boundary is inaudible.
    const R = 3.6;
    S.brown = damp(S.brown, sf.brown, R, d);
    S.pink = damp(S.pink, sf.pink, R, d);
    S.white = damp(S.white, sf.white + S.wetness * 0.25, R, d);
    S.lp = damp(S.lp, sf.lp, R, d);
    S.hp = damp(S.hp, sf.hp, R, d);
    S.peakF = damp(S.peakF, sf.peakF, R, d);
    S.peakG = damp(S.peakG, sf.peakG, R, d);
    S.peakQ = damp(S.peakQ, sf.peakQ, R, d);
    S.level = damp(S.level, sf.level, R, d);
    S.rate = damp(S.rate, sf.rate, R, d);
    S.squelch = damp(S.squelch, sf.squelch, R, d);
    S.squelchHz = damp(S.squelchHz, sf.squelchHz, R, d);
    S.grainF = damp(S.grainF, sf.grainF, R, d);
    S.grainSpread = damp(S.grainSpread, sf.grainSpread, R, d);
    S.grainDec = damp(S.grainDec, sf.grainDec, R, d);
    S.skidF = damp(S.skidF, sf.skidF, R, d);
    S.skidQ = damp(S.skidQ, sf.skidQ, R, d);
    S.skidLevel = damp(S.skidLevel, sf.skidLevel, R, d);
    S.roll = damp(S.roll, sf.roll, R, d);

    // ---- tyre roll ---------------------------------------------------------
    // Centre frequency and resonance both climb with speed: a tyre at 40 km/h
    // has a much higher tread-block passing frequency than one at walking pace.
    const rollBright = 0.55 + 0.75 * S.speedN;
    const grounded = (1 - S.airborne) * (0.35 + 0.65 * S.contact);
    const tyreAmp = clamp01(S.speed / 3.2) * (0.30 + 0.70 * S.load) * S.level * grounded * S.roll * 0.30;
    gset(tyGain.gain, tyreAmp, 0.03);
    pset(tyLP.frequency, clamp(S.lp * rollBright, 90, 17000));
    pset(tyHP.frequency, clamp(S.hp, 20, 1200));
    // The carcass resonance rises with the tread passing rate.
    pset(tyPeak.frequency, clamp(S.peakF * (0.7 + 0.55 * S.speedN), 60, 12000));
    pset(tyPeak.Q, clamp(S.peakQ * (0.8 + 0.8 * S.speedN), 0.3, 22));
    pset(tyPeak.gain, S.peakG * (0.6 + 0.6 * S.speedN));
    gset(tyBrown.gain, S.brown, 0.05);
    gset(tyPink.gain, S.pink, 0.05);
    gset(tyWhite.gain, S.white, 0.05);
    // Noise playback rate carries a little of the pitch: pure filter movement
    // reads as a filter sweep, not as going faster.
    const tyRate = clamp(S.rate * (0.80 + 0.55 * S.speedN), 0.3, 3.2);
    pset(tySrcBrown.playbackRate, tyRate);
    pset(tySrcPink.playbackRate, tyRate * 1.03);
    pset(tySrcWhite.playbackRate, tyRate * 0.97);
    // Squelch LFO: only mud has depth, and it sweeps faster the faster you go.
    pset(tySquelchOsc.frequency, clamp(S.squelchHz * (0.7 + 1.6 * S.speedN), 0.2, 20));
    gset(tySquelchAmt.gain, S.squelch * S.lp * 0.55 * clamp01(S.speed / 4), 0.06);

    // ---- granular impulse layer -------------------------------------------
    // Rate ∝ speed × terrain roughness × the surface's own stone density.
    const grainRate = sf.grain * clamp01(S.speed / 11) * (0.35 + 1.35 * S.rough)
      * grounded * (1 + S.skid * 0.6);
    S.grainRate = damp(S.grainRate, Math.min(grainRate, 78), 6, d);
    grainAcc += S.grainRate * d;
    let fired = 0;
    const grainCap = _voices < MAX_VOICES * GRAIN_BUDGET ? 6 : 0;
    while (grainAcc >= 1 && fired < grainCap) {
      grainAcc -= 1;
      fired++;
      // Poisson-ish jitter: without it a fixed rate reads as a machine.
      const jitter = rng() * d * 0.9;
      const f = S.grainF * (0.45 + rng() * rng() * 2.1 * S.grainSpread);
      const a = (0.020 + rng() * rng() * 0.075) * (0.4 + 0.9 * S.speedN) * S.level;
      grain(t + jitter, clamp(f, 90, 15000), sf.grainQ * (0.7 + rng() * 0.8),
        a, S.grainDec * (0.6 + rng() * 0.9), (rng() * 2 - 1) * 0.45, 0.5 + rng() * 1.7);
    }
    if (grainAcc > 4) grainAcc = 4;   // never build a backlog after a frame spike

    // ---- suspension --------------------------------------------------------
    const susp = st && st.suspension;
    thunkCool -= d; reboundCool -= d; slapCool -= d;
    if (susp) {
      const fk = susp.fork, sh = susp.shock;
      const fkMax = (fk && fk.max) || 0.17;
      const shMax = (sh && sh.max) || 0.165;
      const fkT = fk ? (fk.travel || 0) : 0;
      const shT = sh ? (sh.travel || 0) : 0;
      // Prefer the published velocity; derive it if the bike does not publish one.
      const fkV = fk && fk.velocity !== undefined ? fk.velocity : (fkT - prevFork) / Math.max(1e-4, d);
      const shV = sh && sh.velocity !== undefined ? sh.velocity : (shT - prevShock) / Math.max(1e-4, d);
      prevFork = fkT; prevShock = shT;

      // Compression thunk. Threshold is on velocity, not travel, so a slow
      // squat is silent and a square-edge hit is loud.
      if (thunkCool <= 0) {
        const v = Math.max(fkV, shV);
        if (v > 0.45) {
          const amp = clamp01((v - 0.4) / 2.6) * 0.30;
          const isFork = fkV >= shV;
          const depth = isFork ? fkT / fkMax : shT / shMax;
          // Bottom-out: deep AND still moving fast.
          const hard = clamp01((depth - 0.86) / 0.14) * clamp01((v - 1.1) / 2.0);
          thunk(t, amp, isFork ? 78 : 62, hard, isFork ? -0.14 : 0.10);
          thunkCool = 0.055 + rng() * 0.03;
          // A hard compression always throws the chain — a recorded slap via
          // chainSlap() (which falls back to synth pings only when nothing is
          // decoded). The old separate frame-ring pings stay gone: the clank
          // take inside thunk() covers the bottom-out.
          if (amp > 0.11 && slapCool <= 0) {
            chainSlap(t + 0.008, amp * 0.85, 0.18);
            slapCool = 0.075 + rng() * 0.06;
          }
        }
      }
      // Rebound whoosh: the shock extending fast out of a deep stroke.
      if (reboundCool <= 0) {
        const rv = Math.max(-fkV, -shV);
        const wasDeep = Math.max(forkDeep, shockDeep);
        if (rv > 0.55 && wasDeep > 0.45) {
          rebound(t, clamp01((rv - 0.5) / 2.2) * 0.13 * wasDeep, (fkV < shV ? -0.12 : 0.12));
          reboundCool = 0.16 + rng() * 0.1;
        }
      }
      forkDeep = Math.max(fkT / fkMax, forkDeep - d * 1.6);
      shockDeep = Math.max(shT / shMax, shockDeep - d * 1.6);
    }

    // ---- chain slap from terrain chatter -----------------------------------
    // Independent of the suspension thunk: on chattery ground the chain slaps
    // continuously even without a big hit.
    if (slapCool <= 0 && grounded > 0.4) {
      const chatter = S.rough * S.speedN * (1 - S.pedal * 0.8);
      if (rng() < chatter * d * 9) {
        chainSlap(t, 0.035 + rng() * 0.055 * chatter, (rng() - 0.5) * 0.4);
        slapCool = 0.05 + rng() * 0.09;
      }
    }
    // Airborne: the chain rattles freely as the bike unloads. Recorded rattle
    // snippets only — the old synthesised slap pings read as loose chimes, so
    // with nothing decoded the flight stays wind-and-freehub.
    if (S.airborne > 0.6 && slapCool <= 0 && rng() < d * 3.5) {
      const rb = sfxPick('rattle');
      if (rb) sample(t, rb, 0.10 + rng() * 0.08, 0.85 + rng() * 0.3, (rng() - 0.5) * 0.5, 0);
      slapCool = 0.12 + rng() * 0.15;
    }

    // ---- freehub -----------------------------------------------------------
    // Click rate = ratchet engagements per second. Real hubs sit between about
    // 40 Hz (walking pace) and 350 Hz (flat out) — that whole range has to be
    // audible as a rate change, not just a volume change.
    const revsPerSec = S.rearSpin / (Math.PI * 2);
    const clickHz = clamp(revsPerSec * FREEHUB_TEETH, 8, 420);
    pset(fhOsc.frequency, clickHz);
    gset(fhJitterAmt.gain, clickHz * 0.022, 0.08);
    // Only when coasting: any pedal input engages the hub and the buzz stops
    // dead, which is exactly the cue a rider listens for.
    const fhAmp = S.coast * clamp01((S.rearSpin - 1.5) / 5) * (1 - S.pedal) * 0.115
      * (0.55 + 0.45 * clamp01(S.rearSpin / 22));
    gset(fhGain.gain, fhAmp, 0.035);
    pset(fhBpA.frequency, 2950 + 420 * S.speedN);
    pset(fhBpB.frequency, 4750 + 500 * S.speedN);

    // ---- brakes ------------------------------------------------------------
    const rub = brakeMax * clamp01(S.speed / 2.5) * (0.35 + 0.65 * S.contact);
    gset(brGain.gain, rub * 0.055, 0.03);
    pset(brBp.frequency, clamp(1200 + 1500 * brakeMax + 400 * S.speedN, 200, 8000));
    pset(brPeak.frequency, clamp(2400 + 900 * S.speedN, 400, 9000));
    // Rotor warp modulates once per revolution.
    pset(brLfo.frequency, clamp(revsPerSec, 0.1, 60));
    gset(brLfoAmt.gain, rub * 0.03, 0.05);

    // Squeal: needs pressure AND slip. Pitch rises with pressure — stiffer
    // contact, higher stick-slip frequency.
    const squealDrive = clamp01((brakeMax - 0.45) / 0.55) * clamp01((S.lockup - 0.12) / 0.6)
      * clamp01(S.speed / 4) * (0.4 + 0.6 * S.skid);
    const squealHz = clamp(1900 + 1500 * brakeMax + 500 * S.speedN + S.lockup * 400, 600, 7000);
    pset(sqBp.frequency, squealHz);
    pset(sqOscA.frequency, squealHz * 0.998);
    pset(sqOscB.frequency, squealHz * 1.0035);
    pset(sqBp.Q, clamp(14 + 26 * squealDrive, 6, 44));
    pset(sqVib.frequency, 5.5 + 6 * squealDrive);
    gset(sqVibAmt.gain, squealHz * 0.008 * squealDrive, 0.06);
    gset(sqGain.gain, squealDrive * squealDrive * 0.075, 0.045);

    // ---- skid --------------------------------------------------------------
    const skidAmp = S.skid * clamp01(S.speed / 3.5) * S.skidLevel * grounded * 0.16;
    gset(skGain.gain, skidAmp, 0.03);
    pset(skBp.frequency, clamp(S.skidF * (0.7 + 0.6 * S.speedN), 120, 9000));
    pset(skBp.Q, clamp(S.skidQ * 0.5, 0.3, 14));
    pset(skPeak.frequency, clamp(S.skidF * 1.6, 200, 12000));
    pset(skPeak.Q, clamp(S.skidQ, 0.3, 18));
    pset(skSrc.playbackRate, clamp(0.8 + 0.5 * S.speedN, 0.3, 2.5));

    // ---- wind --------------------------------------------------------------
    // Gain and brightness both rise with speed; the stereo image opens as well,
    // so at speed the wind wraps around you instead of sitting in the middle.
    const windAmp = Math.pow(S.speedN, 1.7) * 0.30 + S.speedN * 0.045;
    const windHz = 320 + 1250 * S.speedN;
    // Sampled wind-rush loop: created once its buffer decodes, then driven by
    // the same speed curve. The synthesised bandpass wind stays underneath at
    // reduced level — it carries the doppler drift and the stereo movement.
    if (!wdSmpSrc && sfxLoader) {
      const wb = sfxLoader.pick('wind', 0);
      if (wb) {
        wdSmpGain = mkGain(0, busWind);
        wdSmpLP = mkFilter('lowpass', 900, 0.7, null, wdSmpGain);
        wdSmpSrc = ac.createBufferSource();
        wdSmpSrc.buffer = wb;
        wdSmpSrc.loop = true;
        wdSmpSrc.connect(wdSmpLP);
        wdSmpSrc.start(t + 0.05);
      }
    }
    const wProc = wdSmpSrc ? 0.45 : 1.0;
    if (wdSmpSrc) {
      gset(wdSmpGain.gain, windAmp * 1.15, 0.06);
      pset(wdSmpLP.frequency, clamp(500 + 5200 * S.speedN, 300, 12000));
      pset(wdSmpSrc.playbackRate, clamp(0.82 + 0.4 * S.speedN + S.drift * 0.6, 0.5, 1.6));
    }
    gset(wdGainL.gain, windAmp * wProc, 0.05);
    gset(wdGainR.gain, windAmp * 0.97 * wProc, 0.05);
    pset(wdBpL.frequency, windHz);
    pset(wdBpR.frequency, windHz * 1.09);
    pset(wdBpL.Q, 0.6 + 0.5 * S.speedN);
    pset(wdBpR.Q, 0.62 + 0.5 * S.speedN);
    const width = 0.28 + 0.62 * S.speedN;
    pset(wdPanL.pan, -width);
    pset(wdPanR.pan, width);
    gset(wdLowGain.gain, Math.pow(S.speedN, 2.4) * 0.20, 0.06);
    pset(wdLowLP.frequency, 130 + 160 * S.speedN);
    // Doppler-ish drift. Swinging the bars hard shifts the apparent wind pitch;
    // it is not physically a doppler shift, but it is the cue that sells a turn.
    S.drift = damp(S.drift, clamp(-S.yaw * 0.05, -0.22, 0.22), 5, d);
    pset(wdSrcL.playbackRate, 1 + S.drift);
    pset(wdSrcR.playbackRate, 1.03 - S.drift);

    // ---- environment probes (throttled) ------------------------------------
    const pos = st && st.position;
    const px = pos ? pos.x : 0, py = pos ? pos.y : 0, pz = pos ? pos.z : 0;
    envTimer -= d;
    if (envTimer <= 0) { envTimer = 0.2; probeEnvironment(px, py, pz); }
    creekTimer -= d;
    if (creekTimer <= 0) { creekTimer = 0.25; probeCreek(px, py, pz); }

    // ---- ambience ----------------------------------------------------------
    // Trees rustle less above the treeline but the wind itself is stronger, so
    // the bed does not simply fade out — it changes colour.
    const forest = 1 - S.openness;
    const bedAmp = 0.048 * (0.45 + 0.75 * forest) * (0.7 + 0.5 * S.altitude);
    gset(amWindGainL.gain, bedAmp, 0.4);
    gset(amWindGainR.gain, bedAmp * 0.95, 0.4);
    pset(amWindBpL.frequency, 520 + 420 * S.openness);
    pset(amWindBpR.frequency, 590 + 460 * S.openness);
    pset(amPanL.pan, -0.6 - 0.3 * forest);
    pset(amPanR.pan, 0.6 + 0.3 * forest);

    gset(ckGain.gain, S.creek * 0.15, 0.25);
    pset(ckPan.pan, clamp(S.creekPan, -1, 1));
    pset(ckHP.frequency, 220 + 240 * S.creek);

    // Reverb character: open terrain gets the long bright tail, dense forest the
    // short dark one, and the total wet level opens up in the open.
    const openMix = S.openness;
    gset(revForestGain.gain, (1 - openMix) * 0.85 + 0.1, 0.5);
    gset(revOpenGain.gain, openMix * 0.9, 0.5);

    // Birds. Denser in forest, sparse in the open, and they stop entirely when
    // the rider is flat out (nothing sings next to a bike at 50 km/h).
    birdTimer -= d;
    if (birdTimer <= 0) {
      const activity = lerp(0.35, 1.0, forest) * lerp(1.0, 0.3, S.speedN);
      birdTimer = (1.6 + erng() * 6.5) / Math.max(0.15, activity);
      if (!paused) birdSong();
    }
    // Creek bloops when close.
    bloopTimer -= d;
    if (bloopTimer <= 0) {
      bloopTimer = 0.25 + erng() * 0.9;
      if (S.creek > 0.35) bloop(t + erng() * 0.1, S.creek * 0.045 * erng(), S.creekPan + (erng() - 0.5) * 0.5);
    }

    // ---- rider effort and breathing ----------------------------------------
    if (!paused && st && !st.crashed) runTime += d;
    // Effort builds with speed, g-force, pedalling and time in the run, and
    // recovers slowly when the trail lets up. `fatigue` is the pure time term —
    // four minutes in, the rider is breathing hard even on a fire road.
    const fatigue = clamp01(runTime / 260);
    const effortTarget = clamp01(
      0.12 + 0.42 * S.speedN + 0.30 * clamp01((S.gForce - 1.1) / 2.2) +
      0.32 * S.pedal + 0.22 * S.rough * S.speedN + 0.20 * (1 - S.stamina) + 0.16 * fatigue
    );
    S.effort = damp(S.effort, effortTarget, effortTarget > S.effort ? 1.1 : 0.28, d);
    // Stamina drains with effort; it only feeds the breathing rate, never pitch.
    S.stamina = clamp01(S.stamina - d * (0.004 + S.effort * 0.011));

    const breathHz = lerp(0.30, 1.30, S.effort) * lerp(1.0, 1.3, fatigue * 0.6 + (1 - S.stamina) * 0.4);
    S.breathPhase += breathHz * d;
    if (S.breathPhase >= 1) {
      S.breathPhase -= 1;
      const amp = (0.014 + S.effort * 0.055) * (1 - S.airborne * 0.4);
      breath(t, true, amp);
      // Exhale follows at ~45% of the cycle.
      breath(t + 0.45 / Math.max(0.2, breathHz), false, amp * 0.85);
      // Heavy effort adds a voiced edge to the exhale.
      if (S.effort > 0.72 && rng() < 0.22) {
        grunt(t + 0.45 / Math.max(0.2, breathHz), (S.effort - 0.7) * 0.9, false);
      }
    }
  }

  // ===========================================================================
  // Public API
  // ===========================================================================
  const api = {
    get started() { return started; },
    get context() { return ac; },
    get masterVolume() { return masterVolume; },
    get muted() { return muted; },

    start,

    setMasterVolume(v) {
      masterVolume = clamp01(typeof v === 'number' ? v : 0.85);
      persist();
      applyMaster();
    },
    setMusicVolume(v) {
      musicVolume = clamp01(typeof v === 'number' ? v : 0.7);
      persist();
      if (started) gset(busMusic.gain, musicVolume * musicVolume, 0.05);
    },
    get musicVolume() { return musicVolume; },

    setSfxVolume(v) {
      sfxVolume = clamp01(typeof v === 'number' ? v : 1.0);
      persist();
      if (started) gset(busSfx.gain, sfxVolume * sfxVolume, 0.05);
    },
    get sfxVolume() { return sfxVolume; },

    /**
     * Summing point for the sample-based music system. Null until the first
     * user gesture builds the graph. `input` inherits master volume, mute,
     * external (ad) mute and blur-zeroing; `duck` is the music system's own
     * ducking stage (unity by default).
     */
    getMusicBus() {
      return started ? { input: busMusic, duck: musicDuck } : null;
    },

    setMuted(m) {
      muted = !!m;
      persist();
      applyMaster();
    },
    toggleMute() { api.setMuted(!muted); return muted; },

    /**
     * Silence for something outside the game — a fullscreen ad. Not persisted and
     * does not touch the player's own mute setting, so restoring it cannot
     * un-mute someone who chose silence.
     */
    setExternalMute(m) {
      externalMuted = !!m;
      applyMaster();
    },

    /**
     * Fire a named sound directly. Used by the UI and available for debugging.
     * opts: { volume, pan, x, y, z, severity, freq }
     */
    play(name, opts) {
      if (!started) return;
      const o = opts || {};
      const t = now() + 0.01;
      const vol = o.volume === undefined ? 1 : clamp01(o.volume);
      switch (name) {
        case 'impact': impact(t, o.severity === undefined ? 0.5 : o.severity, o.surface | 0, o.x, o.y, o.z); break;
        case 'crash': crash(o.severity, o.x, o.y, o.z); break;
        case 'thunk': thunk(t, 0.25 * vol, 72, o.severity || 0, o.pan || 0); break;
        case 'chain': chainSlap(t, 0.09 * vol, o.pan || 0); break;
        case 'grunt': grunt(t, vol, !!o.hard); break;
        case 'bird': birdSong(); break;
        case 'ui':
        case 'tone': bell(t, o.freq || 440, 0.09 * vol, o.dur || 0.6, 2.0, 1.0, o.pan || 0); break;
        case 'hover': bell(t, 587.33, 0.035 * vol, 0.18, 3.0, 0.7, o.pan || 0); break;
        case 'select': bell(t, 880.0, 0.06 * vol, 0.32, 2.0, 0.9, o.pan || 0); break;
        case 'back': bell(t, 329.63, 0.05 * vol, 0.30, 2.0, 0.8, o.pan || 0); break;
        default: break;
      }
    },

    /** Manual duck, for cutscene/menu transitions. */
    duck,

    update,

    resize() { /* nothing view-dependent */ },

    dispose() {
      window.removeEventListener('pointerdown', onGesture);
      window.removeEventListener('keydown', onGesture);
      window.removeEventListener('touchstart', onGesture);
      window.removeEventListener('blur', onBlur);
      window.removeEventListener('focus', onFocus);
      document.removeEventListener('visibilitychange', onVisibility);
      for (let i = 0; i < offs.length; i++) { try { offs[i](); } catch (e) { /* ignore */ } }
      offs.length = 0;
      if (sfxLoader) { sfxLoader.dispose(); sfxLoader = null; }
      wdSmpSrc = wdSmpGain = wdSmpLP = null;
      if (ac) {
        try {
          master.gain.cancelScheduledValues(ac.currentTime);
          master.gain.setTargetAtTime(0.0001, ac.currentTime, 0.03);
          const dead = ac;
          setTimeout(() => { try { dead.close(); } catch (e) { /* ignore */ } }, 200);
        } catch (e) { /* ignore */ }
      }
      ac = null;
      started = false;
    },

    // --- debug aids ---------------------------------------------------------
    get debugState() { return S; },
    /** Sampled-SFX status for the debug overlay / harness. */
    get sfxSamples() {
      return {
        state: sfxLoader ? sfxLoader.state : 'off',
        decoded: sfxLoader ? sfxLoader.decodedCount : 0,
        last: dbgLastSfx,
      };
    },
    get voices() { return _voices; },
  };

  return api;
}

export default createAudio;
