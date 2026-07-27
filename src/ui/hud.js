// hud.js — DESCENT run HUD. DOM overlay per CONTRACT §8.
//
// Design intent: a broadcast-telemetry look — one accent, tabular numerals, heavy
// negative space, everything animated on transform/opacity only. The whole overlay
// is built once; per-frame work is limited to a handful of style writes and text
// updates that are gated on the value having actually changed.
//
// CONTRACT-NOTE: several things this module needs are not pinned down by the contract,
// so it reads them defensively and falls back:
//   * `run:checkpoint` payload shape is unspecified. We accept
//     { index|checkpoint|i, time|split, delta|deltaBest|diff, best } in any combination,
//     and compute the delta ourselves from `gameplay.best.splits[i]` when it is absent.
//   * `gameplay.best` may be a number (total time) or an object { time, splits[] }.
//     Both are handled.
//   * `gameplay.countdown` (seconds remaining) is not in the contract. When
//     `gameplay.state === 'countdown'` and no numeric countdown field exists we run our
//     own 3-2-1 clock so the display is never blank.
//   * Photo-mode ownership is unspecified. `input.state.photoMode` is an edge flag, so
//     the HUD keeps its own toggle, but defers to `ctx.photoMode` if any other module
//     defines it as a boolean. The HUD never writes to ctx.
//   * Style/combo scoring: if `gameplay.score` is ever a finite non-zero number we track
//     it; until then the HUD keeps a local decaying combo so the meter is not dead during
//     development. `trick:landed` wins over `bike:landed` once it has fired even once.
//   * `ctx.engine.stats` is used for the debug block (engine is not in CONTRACT §2 but is
//     assigned by main.js and documented in ADDENDUM §G's spirit).
//   * R9 balance cue: the key glyph is the only place the HUD hard-codes a binding.
//     CONTRACT §7 does not publish a name→key map, so it is derived from the shipped
//     input.js mapping (W = pitch +1 = weight forward, S = pitch −1 = weight back;
//     stick Y is inverted into the same convention) and from `input.hasGamepad`.
//     If input.js ever publishes a binding table, read it instead. Note that
//     input.js's own inline comment on the stick axis states the OPPOSITE sign
//     convention to its code and to bike.js — the code and bike.js agree, the
//     comment is stale, and this module follows the code.

import { clamp, clamp01, damp } from '../core/rng.js';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const SURFACE_NAMES = ['DIRT', 'LOAM', 'ROCK', 'GRAVEL', 'GRASS', 'ROOT', 'MUD', 'SNOW'];

/** Speed gauge range. A DH bike tops out around 75 km/h on this course. */
const GAUGE_MAX_KMH = 90;
const GAUGE_REDLINE_KMH = 66;

/** Gauge arc geometry, in SVG user units. */
const GA = { cx: 150, cy: 140, r: 116, a0: 196, a1: 344, w: 300, h: 124 };

const GEAR_COUNT = 12;
/** Pseudo-cassette: km/h at which each gear "engages". Non-linear, like a real 1x12. */
const GEAR_SPEEDS = [0, 5, 9, 13, 17.5, 22, 27, 32.5, 38.5, 45, 52.5, 61];

// ---------------------------------------------------------------------------
// R9 — rider balance.
//
// The rear load fraction is taken from `bike.state.wheels[i].load`, which
// bike.js publishes in newtons. Nothing here re-derives it from grade, from the
// input axis, or from a free-body diagram: the whole point of the affordance is
// that it shows what the tyres are actually carrying.
//
// Where the dot sits: for a two-wheeled body, the front load fraction IS the
// centre of mass's position along the wheelbase measured from the rear axle
// (N_front/W = b/L, b = CG-to-rear-axle). So one measured number places the dot,
// with no model in between.
//
// Calibration and its provenance, stated so it is not re-litigated. bike.js's
// static split is FRONT_BIAS 0.45, i.e. a rear fraction of 0.55 on the flat.
// The one published measurement of this quantity on steep ground (round 8, 45%
// grade) puts an un-shifted rider at ~0.13 rear and a shifted one at ~0.33. So
// BAL_LO sits just above the value that was measured to be unrideable, and
// BAL_HI between the two — the caution arrives before the state does, which is
// the entire job. Those are static free-body figures and the live loads will
// not match them exactly, so nothing here is a hard trip: the dot ramps
// continuously against the band and only the cue is discrete — and the cue is
// hysteretic, gated on grade, and suppressed once the player is already at full
// shift, because at that point it has nothing left to tell them.
const BAL_LO = 0.16;           // rear fraction — the state that precedes an OTB
const BAL_HI = 0.25;           // rear fraction — caution
const BAL_FRONT_LO = 0.18;     // front fraction below this = looping out
const BAL_CUE_GRADE = 0.16;    // no cue below this grade; balance is not the story
const BAL_CUE_HOLD = 0.60;     // s the cue holds after the state clears
const BAL_X0 = 0.17, BAL_X1 = 0.83;   // rail extent, as fractions of the box
// bike.js publishes `state.riderFore` in metres (+ = forward) and T.RIDER_SHIFT
// is its full-deflection value. This is only a scale, not a re-derivation.
const RIDER_SHIFT = 0.30;

const AIR_MIN_SHOW = 0.16;     // s airborne before the counter appears
const AIR_HOLD = 1.05;         // s the final value is held after touchdown
const DELTA_HOLD = 3.4;        // s the split delta stays up before fading
const SHAKE_DECAY = 7.5;       // 1/s
const DAMAGE_DECAY = 0.62;     // 1/s

let uidCounter = 0;

// ---------------------------------------------------------------------------
// Small pure helpers (module scope — no per-frame allocation from closures)
// ---------------------------------------------------------------------------

function polarX(cx, r, deg) { return cx + r * Math.cos(deg * Math.PI / 180); }
function polarY(cy, r, deg) { return cy + r * Math.sin(deg * Math.PI / 180); }

/** SVG arc path between two angles on a circle. */
function arcPath(cx, cy, r, a0, a1) {
  const x0 = polarX(cx, r, a0), y0 = polarY(cy, r, a0);
  const x1 = polarX(cx, r, a1), y1 = polarY(cy, r, a1);
  const large = Math.abs(a1 - a0) > 180 ? 1 : 0;
  const sweep = a1 > a0 ? 1 : 0;
  return `M ${x0.toFixed(2)} ${y0.toFixed(2)} A ${r} ${r} 0 ${large} ${sweep} ${x1.toFixed(2)} ${y1.toFixed(2)}`;
}

/** Angle on the gauge for a speed in km/h. */
function gaugeAngle(kmh) {
  const f = clamp01(kmh / GAUGE_MAX_KMH);
  return GA.a0 + (GA.a1 - GA.a0) * f;
}

/** "1:23" — the part of the clock that changes at most once a second. */
function fmtClockMain(t) {
  if (!Number.isFinite(t) || t < 0) t = 0;
  const total = Math.floor(t);
  const m = Math.floor(total / 60);
  const s = total - m * 60;
  return m + ':' + (s < 10 ? '0' : '') + s;
}

/** ".45" — hundredths, kept in its own (smaller, dimmer) span. */
function fmtClockFrac(t) {
  if (!Number.isFinite(t) || t < 0) t = 0;
  const h = Math.floor((t - Math.floor(t)) * 100);
  return '.' + (h < 10 ? '0' : '') + h;
}

/** Full "1:23.45" for one-shot displays (best time, finish card). */
function fmtClock(t) {
  if (!Number.isFinite(t) || t < 0) return '--:--.--';
  return fmtClockMain(t) + fmtClockFrac(t);
}

/** "+1.24" / "-0.87" — signed, always two decimals, minutes only when needed. */
function fmtDelta(d) {
  if (!Number.isFinite(d)) return '';
  const sign = d < 0 ? '−' : '+';   // U+2212 minus reads better than hyphen
  const a = Math.abs(d);
  if (a >= 60) {
    const m = Math.floor(a / 60);
    const s = a - m * 60;
    return sign + m + ':' + (s < 10 ? '0' : '') + s.toFixed(2);
  }
  return sign + a.toFixed(2);
}

/** textContent write, gated on change. Cheaper than trusting the browser to diff. */
function setText(el, v) {
  if (!el) return;
  if (el.__v !== v) { el.__v = v; el.textContent = v; }
}

/** classList write, gated on change (contains() is cheap and allocation-free). */
function setFlag(el, cls, on) {
  if (!el) return;
  if (el.classList.contains(cls) !== !!on) el.classList.toggle(cls, !!on);
}

/** Element factory. */
function h(tag, cls, parent, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) { e.textContent = text; e.__v = text; }
  if (parent) parent.appendChild(e);
  return e;
}

function svgEl(tag, parent, attrs) {
  const e = document.createElementNS('http://www.w3.org/2000/svg', tag);
  if (attrs) for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}

/** Web Animations one-shot; silently no-ops where unsupported. */
function play(el, frames, opts) {
  if (!el || typeof el.animate !== 'function') return null;
  try { return el.animate(frames, opts); } catch (e) { return null; }
}

function pickNum(...vals) {
  for (let i = 0; i < vals.length; i++) {
    if (Number.isFinite(vals[i])) return vals[i];
  }
  return undefined;
}

// ---------------------------------------------------------------------------
// Stylesheet — one <style>, design tokens at the top.
// ---------------------------------------------------------------------------

const CSS = `
#dsc-hud {
  /* ---- scale: one unit drives the whole layout, 1280 -> 3840 ------------- */
  --u: clamp(11px, min(0.80vw, 1.72vh), 24px);

  /* ---- type scale (multiples of u) --------------------------------------- */
  --t-micro: calc(var(--u) * 0.62);
  --t-small: calc(var(--u) * 0.82);
  --t-body:  calc(var(--u) * 1.00);
  --t-mid:   calc(var(--u) * 1.55);
  --t-large: calc(var(--u) * 2.55);
  --t-huge:  calc(var(--u) * 4.35);
  --t-mega:  calc(var(--u) * 7.20);

  /* ---- spacing scale ------------------------------------------------------ */
  --sp1: calc(var(--u) * 0.35);
  --sp2: calc(var(--u) * 0.75);
  --sp3: calc(var(--u) * 1.40);
  --sp4: calc(var(--u) * 2.40);
  --sp5: calc(var(--u) * 4.00);

  /* ---- colour: 5 neutrals + one accent + two semantics -------------------- */
  --paper:  #F2F7FC;
  --dim:    rgba(230, 240, 250, 0.60);
  --mute:   rgba(214, 228, 242, 0.32);
  --hair:   rgba(226, 240, 252, 0.16);
  --ink:    rgba(4, 7, 11, 0.46);
  --accent: #7DEEFF;
  --hot:    #FF6A2B;
  --good:   #46E08C;
  --bad:    #FF4A5E;

  /* ---- motion ------------------------------------------------------------- */
  --e-out:  cubic-bezier(0.16, 1, 0.30, 1);
  --e-soft: cubic-bezier(0.33, 1, 0.68, 1);
  --e-both: cubic-bezier(0.65, 0, 0.35, 1);
  --e-snap: cubic-bezier(0.22, 1.35, 0.36, 1);

  --f-ui: -apple-system, BlinkMacSystemFont, "SF Pro Display", "Segoe UI Variable Display",
          "Segoe UI", system-ui, ui-sans-serif, "Helvetica Neue", Arial, sans-serif;
  --f-num: ui-monospace, "SF Mono", SFMono-Regular, Menlo, "Cascadia Mono", "Roboto Mono",
           "DejaVu Sans Mono", Consolas, monospace;

  --shadow: 0 1px 2px rgba(0,0,0,0.55), 0 0 14px rgba(0,0,0,0.28);

  position: fixed;
  inset: 0;
  z-index: 20;
  pointer-events: none;
  font-family: var(--f-ui);
  color: var(--paper);
  -webkit-font-smoothing: antialiased;
  text-rendering: optimizeLegibility;
  contain: layout style;
  opacity: 1;
  will-change: opacity;
  transition: opacity 420ms var(--e-out);
}
#dsc-hud.is-hidden { opacity: 0; }
#dsc-hud.is-dimmed { opacity: 0.30; }
#dsc-hud * { box-sizing: border-box; margin: 0; padding: 0; }

/* Shake layer. Only the framed content moves — the full-bleed layers stay put so
   the shake can never reveal an edge. */
#dsc-hud .shake {
  position: absolute;
  inset: 0;
  will-change: transform;
  transform: translate3d(0,0,0);
}
#dsc-hud .frame {
  position: absolute;
  inset: 0;
  padding:
    calc(var(--sp4) + env(safe-area-inset-top, 0px))
    calc(var(--sp4) + env(safe-area-inset-right, 0px))
    calc(var(--sp4) + env(safe-area-inset-bottom, 0px))
    calc(var(--sp4) + env(safe-area-inset-left, 0px));
}

/* ---- shared bits --------------------------------------------------------- */
#dsc-hud .lbl {
  font-size: var(--t-micro);
  font-weight: 650;
  letter-spacing: 0.22em;
  text-transform: uppercase;
  color: var(--mute);
  text-shadow: var(--shadow);
  white-space: nowrap;
}
#dsc-hud .num {
  font-variant-numeric: tabular-nums lining-nums;
  font-feature-settings: "tnum" 1, "lnum" 1;
  letter-spacing: -0.012em;
}
#dsc-hud .cl { position: absolute; }

/* Corner scrims: a whisper of darkness so white numerals hold up against the sky.
   Purely decorative, no layout cost. */
#dsc-hud .scrim {
  position: absolute;
  pointer-events: none;
  opacity: 0.85;
}
#dsc-hud .scrim-tl {
  top: 0; left: 0; width: 46%; height: 34%;
  background: radial-gradient(120% 120% at 0% 0%, rgba(3,6,10,0.52), rgba(3,6,10,0) 68%);
}
#dsc-hud .scrim-bl {
  bottom: 0; left: 0; width: 52%; height: 40%;
  background: radial-gradient(120% 120% at 0% 100%, rgba(3,6,10,0.50), rgba(3,6,10,0) 68%);
}
#dsc-hud .scrim-br {
  bottom: 0; right: 0; width: 46%; height: 46%;
  background: radial-gradient(120% 120% at 100% 100%, rgba(3,6,10,0.58), rgba(3,6,10,0) 70%);
}
#dsc-hud .scrim-r {
  top: 0; right: 0; width: 22%; height: 100%;
  background: linear-gradient(270deg, rgba(3,6,10,0.34), rgba(3,6,10,0) 72%);
}

/* ---- timer cluster ------------------------------------------------------- */
#dsc-hud .cl-timer {
  top: calc(var(--sp4) + env(safe-area-inset-top, 0px));
  left: calc(var(--sp4) + env(safe-area-inset-left, 0px));
}
#dsc-hud .timer-rule {
  width: calc(var(--u) * 2.6);
  height: 2px;
  background: var(--accent);
  opacity: 0.9;
  margin-bottom: var(--sp2);
  transform-origin: left center;
  box-shadow: 0 0 10px rgba(125,238,255,0.45);
}
#dsc-hud .timer {
  display: flex;
  align-items: baseline;
  font-family: var(--f-num);
  font-weight: 600;
  line-height: 0.92;
  text-shadow: var(--shadow);
  margin-top: calc(var(--u) * 0.12);
}
#dsc-hud .t-main { font-size: var(--t-large); }
#dsc-hud .t-frac { font-size: var(--t-mid); color: var(--dim); margin-left: 0.04em; }
#dsc-hud .timer-sub {
  display: flex;
  gap: var(--sp2);
  align-items: baseline;
  margin-top: var(--sp2);
}
#dsc-hud .best {
  font-family: var(--f-num);
  font-size: var(--t-small);
  color: var(--dim);
  text-shadow: var(--shadow);
}
#dsc-hud .delta {
  margin-top: var(--sp3);
  font-family: var(--f-num);
  font-size: var(--t-mid);
  font-weight: 650;
  line-height: 1;
  padding: calc(var(--u) * 0.24) calc(var(--u) * 0.6);
  border-radius: calc(var(--u) * 0.28);
  display: inline-block;
  background: rgba(4,7,11,0.44);
  border-left: 2px solid currentColor;
  text-shadow: var(--shadow);
  opacity: 0;
  transform: translate3d(calc(var(--u) * -0.5), 0, 0);
  will-change: transform, opacity;
  transition: opacity 620ms var(--e-out), transform 620ms var(--e-out);
}
#dsc-hud .delta.on { opacity: 1; transform: translate3d(0,0,0); }
#dsc-hud .delta.good { color: var(--good); }
#dsc-hud .delta.bad  { color: var(--bad); }

/* ---- trail progress (right edge) ----------------------------------------- */
#dsc-hud .cl-prog {
  --track: calc(var(--u) * 21);
  top: 50%;
  right: calc(var(--sp4) + env(safe-area-inset-right, 0px));
  transform: translateY(-56%);
  display: flex;
  flex-direction: column;
  align-items: flex-end;
  gap: var(--sp2);
}
#dsc-hud .prog-track {
  position: relative;
  width: calc(var(--u) * 0.34);
  height: var(--track);
  background: rgba(6,10,15,0.46);
  border-radius: 999px;
  overflow: visible;
  box-shadow: inset 0 0 0 1px var(--hair);
}
#dsc-hud .prog-fill {
  position: absolute;
  inset: 0;
  border-radius: 999px;
  background: linear-gradient(180deg, var(--accent), rgba(125,238,255,0.35));
  transform-origin: top center;
  transform: scaleY(var(--p, 0));
  will-change: transform;
  box-shadow: 0 0 12px rgba(125,238,255,0.30);
}
#dsc-hud .prog-pip {
  position: absolute;
  left: 50%;
  width: calc(var(--u) * 0.95);
  height: 2px;
  margin-left: calc(var(--u) * -0.475);
  margin-top: -1px;
  background: var(--mute);
  border-radius: 1px;
  transition: background-color 340ms var(--e-out), width 0ms;
}
#dsc-hud .prog-pip.done { background: var(--paper); }
#dsc-hud .prog-marker {
  position: absolute;
  left: 50%;
  top: 0;
  width: calc(var(--u) * 1.6);
  height: calc(var(--u) * 1.6);
  margin-left: calc(var(--u) * -0.8);
  margin-top: calc(var(--u) * -0.8);
  transform: translate3d(0, calc(var(--p, 0) * var(--track)), 0);
  will-change: transform;
}
#dsc-hud .prog-marker::before {
  content: "";
  position: absolute;
  left: 50%; top: 50%;
  width: calc(var(--u) * 0.52);
  height: calc(var(--u) * 0.52);
  margin: calc(var(--u) * -0.26) 0 0 calc(var(--u) * -0.26);
  background: var(--paper);
  border-radius: 999px;
  box-shadow: 0 0 0 calc(var(--u) * 0.14) rgba(4,7,11,0.55), 0 0 14px rgba(255,255,255,0.55);
}
#dsc-hud .prog-ghost {
  position: absolute;
  left: 50%;
  top: 0;
  width: calc(var(--u) * 0.9);
  height: calc(var(--u) * 0.9);
  margin-left: calc(var(--u) * -0.45);
  margin-top: calc(var(--u) * -0.45);
  border: 1.5px solid var(--hot);
  border-radius: 999px;
  opacity: 0;
  transform: translate3d(0, calc(var(--g, 0) * var(--track)), 0);
  will-change: transform, opacity;
}
#dsc-hud .prog-ghost.on { opacity: 0.85; }
#dsc-hud .prog-cap { display: flex; flex-direction: column; align-items: flex-end; gap: 2px; }
#dsc-hud .prog-pct {
  font-family: var(--f-num);
  font-size: var(--t-small);
  color: var(--dim);
  text-shadow: var(--shadow);
}

/* Centre-screen callouts sit on bare sky, which is the brightest thing in the
   frame. Each carries its own soft scrim so white numerals never lose contrast.
   isolation:isolate keeps the z-index:-1 scrim inside its own element. */
#dsc-hud .cl-air, #dsc-hud .banner, #dsc-hud .count { isolation: isolate; }
#dsc-hud .cl-air::before,
#dsc-hud .banner::before,
#dsc-hud .count::before {
  content: "";
  position: absolute;
  left: 50%;
  top: 50%;
  width: calc(var(--u) * 40);
  height: calc(var(--u) * 14);
  transform: translate(-50%, -50%);
  /* closest-side reaches zero exactly at the box edge, so there is no visible
     ellipse rim — the scrim has to be felt, not seen. */
  background: radial-gradient(closest-side,
    rgba(3,6,10,0.34), rgba(3,6,10,0.20) 38%, rgba(3,6,10,0.07) 68%, rgba(3,6,10,0) 100%);
  z-index: -1;
  pointer-events: none;
}
#dsc-hud .count::before { width: calc(var(--u) * 30); height: calc(var(--u) * 20); }

/* ---- airtime ------------------------------------------------------------- */
#dsc-hud .cl-air {
  top: 16%;
  left: 50%;
  transform: translate3d(-50%, calc(var(--u) * 0.7), 0) scale(0.94);
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: calc(var(--u) * 0.1);
  opacity: 0;
  will-change: transform, opacity;
  transition: opacity 260ms var(--e-out), transform 380ms var(--e-out);
}
#dsc-hud .cl-air.on {
  opacity: 1;
  transform: translate3d(-50%, 0, 0) scale(1);
}
#dsc-hud .air-row { display: flex; align-items: baseline; }
#dsc-hud .air-val {
  font-family: var(--f-num);
  font-size: calc(var(--u) * 3.2);
  font-weight: 650;
  line-height: 1;
  letter-spacing: -0.02em;
  text-shadow: 0 2px 22px rgba(0,0,0,0.55), 0 1px 3px rgba(0,0,0,0.6);
  color: var(--paper);
  transition: color 220ms var(--e-out);
}
#dsc-hud .cl-air.big .air-val { color: var(--accent); }
#dsc-hud .cl-air.huge .air-val { color: var(--hot); }
#dsc-hud .air-unit {
  font-family: var(--f-num);
  font-size: var(--t-body);
  color: var(--dim);
  margin-left: 0.16em;
}

/* ---- style / combo meter ------------------------------------------------- */
#dsc-hud .cl-style {
  bottom: calc(var(--sp4) + env(safe-area-inset-bottom, 0px));
  left: 50%;
  transform: translateX(-50%);
  width: calc(var(--u) * 17);
  display: flex;
  flex-direction: column;
  gap: var(--sp1);
  opacity: 0;
  transition: opacity 420ms var(--e-out);
}
#dsc-hud .cl-style.on { opacity: 1; }
#dsc-hud .style-head {
  display: flex;
  align-items: baseline;
  justify-content: space-between;
}
#dsc-hud .style-mult {
  font-family: var(--f-num);
  font-size: var(--t-body);
  font-weight: 650;
  color: var(--accent);
  text-shadow: var(--shadow);
}
#dsc-hud .style-bar {
  position: relative;
  height: calc(var(--u) * 0.28);
  background: rgba(6,10,15,0.48);
  border-radius: 999px;
  overflow: hidden;
  box-shadow: inset 0 0 0 1px var(--hair);
}
#dsc-hud .style-fill {
  position: absolute;
  inset: 0;
  border-radius: 999px;
  background: linear-gradient(90deg, var(--accent), #FFFFFF);
  transform-origin: left center;
  transform: scaleX(var(--v, 0));
  will-change: transform;
  box-shadow: 0 0 12px rgba(125,238,255,0.4);
}
#dsc-hud .style-score {
  align-self: flex-end;
  font-family: var(--f-num);
  font-size: var(--t-small);
  color: var(--dim);
  text-shadow: var(--shadow);
}
#dsc-hud .chips {
  position: absolute;
  bottom: calc(var(--u) * 3.4);
  left: 0;
  right: 0;
  height: calc(var(--u) * 5);
  pointer-events: none;
}
#dsc-hud .chip {
  position: absolute;
  bottom: 0;
  left: 50%;
  transform: translate3d(-50%, 0, 0);
  white-space: nowrap;
  font-size: var(--t-small);
  font-weight: 700;
  letter-spacing: 0.10em;
  text-transform: uppercase;
  padding: calc(var(--u) * 0.2) calc(var(--u) * 0.62);
  border-radius: 999px;
  background: rgba(4,7,11,0.52);
  box-shadow: inset 0 0 0 1px var(--hair);
  color: var(--paper);
  text-shadow: var(--shadow);
  opacity: 0;
  will-change: transform, opacity;
}
#dsc-hud .chip b {
  font-family: var(--f-num);
  color: var(--accent);
  margin-left: 0.5em;
  font-weight: 700;
}

/* ---- telemetry strip (bottom-left) --------------------------------------- */
#dsc-hud .cl-tele {
  bottom: calc(var(--sp4) + env(safe-area-inset-bottom, 0px));
  left: calc(var(--sp4) + env(safe-area-inset-left, 0px));
  display: flex;
  align-items: flex-end;
  gap: calc(var(--u) * 2.05);
}
#dsc-hud .mod { display: flex; flex-direction: column; gap: var(--sp1); }
/* A fixed body height keeps the three micro-labels on one optical baseline no
   matter how tall each module's internals are. */
#dsc-hud .mod-body {
  display: flex;
  align-items: center;
  gap: var(--sp2);
  height: calc(var(--u) * 3.6);
}

/* inclinometer */
#dsc-hud .incl {
  position: relative;
  width: calc(var(--u) * 3.6);
  height: calc(var(--u) * 3.6);
  border-radius: calc(var(--u) * 0.5);
  overflow: hidden;
  background: rgba(5,9,14,0.50);
  box-shadow: inset 0 0 0 1px var(--hair);
}
#dsc-hud .incl-rot {
  position: absolute;
  left: -50%;
  top: 0;
  width: 200%;
  height: 100%;
  transform: rotate(var(--a, 0deg));
  transform-origin: 50% 50%;
  will-change: transform;
}
#dsc-hud .incl-ground {
  position: absolute;
  left: 0; top: 50%;
  width: 100%; height: 100%;
  background: linear-gradient(180deg, rgba(125,238,255,0.20), rgba(125,238,255,0.02) 60%);
  border-top: 1.5px solid var(--accent);
}
#dsc-hud .incl.steep .incl-ground {
  background: linear-gradient(180deg, rgba(255,106,43,0.22), rgba(255,106,43,0.02) 60%);
  border-top-color: var(--hot);
}
#dsc-hud .incl-hub {
  position: absolute;
  left: 50%; top: 50%;
  width: calc(var(--u) * 0.3);
  height: calc(var(--u) * 0.3);
  margin: calc(var(--u) * -0.15) 0 0 calc(var(--u) * -0.15);
  border-radius: 999px;
  background: var(--paper);
  box-shadow: 0 0 0 2px rgba(4,7,11,0.6);
}
#dsc-hud .grade-num {
  font-family: var(--f-num);
  font-size: var(--t-mid);
  font-weight: 600;
  line-height: 1;
  text-shadow: var(--shadow);
  min-width: calc(var(--u) * 3.4);
  transition: color 260ms var(--e-out);
}
#dsc-hud .incl.steep + .grade-num, #dsc-hud .grade-num.steep { color: var(--hot); }

/* ---- balance (R9) --------------------------------------------------------
   A side elevation of the wheelbase, tilted by the same grade the inclinometer
   shows, with the rider's measured centre of mass sliding along it. On a steep
   the dot runs toward the front axle on its own; the only thing that pulls it
   back is the player. Fades out when balance is not the story. */
#dsc-hud .mod-bal {
  position: relative;
  opacity: 0.26;
  transition: opacity 380ms var(--e-out);
}
#dsc-hud .mod-bal.live { opacity: 1; }
#dsc-hud .bal {
  /* One box width drives every horizontal position below, so the JS can work
     in unitless 0..1 fractions of the wheelbase and never touch layout. */
  --bw: calc(var(--u) * 6.1);
  position: relative;
  width: var(--bw);
  height: calc(var(--u) * 3.6);
  border-radius: calc(var(--u) * 0.5);
  overflow: hidden;
  background: rgba(5,9,14,0.50);
  box-shadow: inset 0 0 0 1px var(--hair);
}
#dsc-hud .bal-rot {
  position: absolute;
  left: 0; top: 0; width: 100%; height: 100%;
  transform: rotate(var(--a, 0deg));
  transform-origin: 50% 50%;
  will-change: transform;
}
/* The rail runs rear (left) -> front (right); +grade rotates it nose-down to
   the right, which is the direction of travel. */
#dsc-hud .bal-rail {
  position: absolute;
  /* 17% inset, not 14%: at the ±46° clamp the rail end plus the dot radius
     then still clears the box, so nothing is ever clipped by the overflow. */
  left: 17%; right: 17%; top: 50%;
  height: calc(var(--u) * 0.16);
  margin-top: calc(var(--u) * -0.08);
  border-radius: 999px;
  background: rgba(214,228,242,0.24);
}
#dsc-hud .bal-axle {
  position: absolute;
  top: 50%;
  width: calc(var(--u) * 0.34);
  height: calc(var(--u) * 0.34);
  margin: calc(var(--u) * -0.17) 0 0 calc(var(--u) * -0.17);
  border-radius: 999px;
  box-shadow: inset 0 0 0 1.5px rgba(214,228,242,0.42);
}
#dsc-hud .bal-axle.rear { left: 17%; }
#dsc-hud .bal-axle.front { left: 83%; }
/* Safe band: where the rear tyre still has enough load to steer. */
#dsc-hud .bal-zone {
  position: absolute;
  top: 50%;
  left: calc(var(--z0, 0.29) * var(--bw));
  width: calc(var(--zw, 0.38) * var(--bw));
  height: calc(var(--u) * 0.86);
  margin-top: calc(var(--u) * -0.43);
  border-radius: calc(var(--u) * 0.2);
  background: rgba(125,238,255,0.13);
  box-shadow: inset 0 0 0 1px rgba(125,238,255,0.30);
  transition: background-color 260ms var(--e-out), box-shadow 260ms var(--e-out);
}
#dsc-hud .mod-bal.warn .bal-zone {
  background: rgba(255,106,43,0.15);
  box-shadow: inset 0 0 0 1px rgba(255,106,43,0.46);
}
/* The commanded shift — a caret under the rail. This is the player's own
   input, shown separately from its result so the control reads as connected. */
#dsc-hud .bal-cmd {
  position: absolute;
  top: 50%;
  left: 0;
  width: calc(var(--u) * 0.52);
  height: calc(var(--u) * 0.30);
  margin: calc(var(--u) * 0.52) 0 0 calc(var(--u) * -0.26);
  transform: translate3d(calc(var(--x, 0.5) * var(--bw)), 0, 0);
  will-change: transform;
  background: var(--dim);
  clip-path: polygon(50% 0, 100% 100%, 0 100%);
  opacity: 0.0;
  transition: opacity 220ms var(--e-out), background-color 220ms var(--e-out);
}
#dsc-hud .mod-bal.cmd .bal-cmd { opacity: 0.95; }
#dsc-hud .mod-bal.cmd.shifting .bal-cmd { background: var(--accent); }
/* The result — where the mass actually is, from the measured wheel loads. */
#dsc-hud .bal-dot {
  position: absolute;
  top: 50%;
  left: 0;
  width: calc(var(--u) * 0.62);
  height: calc(var(--u) * 0.62);
  margin: calc(var(--u) * -0.31) 0 0 calc(var(--u) * -0.31);
  border-radius: 999px;
  background: var(--paper);
  box-shadow: 0 0 0 calc(var(--u) * 0.12) rgba(4,7,11,0.62), 0 0 10px rgba(255,255,255,0.35);
  transform: translate3d(calc(var(--x, 0.47) * var(--bw)), 0, 0);
  will-change: transform;
  transition: background-color 240ms var(--e-out);
}
#dsc-hud .mod-bal.warn .bal-dot { background: var(--hot); }
#dsc-hud .mod-bal.air .bal { opacity: 0.34; }
/* The cue. Sits above the strip on its own line so it can never reflow the
   telemetry row, and it states the condition rather than scolding. */
#dsc-hud .bal-cue {
  position: absolute;
  bottom: 100%;
  left: 0;
  margin-bottom: calc(var(--u) * 0.34);
  display: flex;
  align-items: center;
  gap: calc(var(--u) * 0.34);
  white-space: nowrap;
  opacity: 0;
  transform: translate3d(calc(var(--u) * 0.5), 0, 0);
  transition: opacity 200ms var(--e-out), transform 320ms var(--e-out);
  will-change: transform, opacity;
}
#dsc-hud .mod-bal.cue .bal-cue { opacity: 1; transform: translate3d(0,0,0); }
#dsc-hud .bal-cue-txt {
  font-size: var(--t-micro);
  font-weight: 700;
  letter-spacing: 0.22em;
  text-transform: uppercase;
  color: var(--hot);
  text-shadow: var(--shadow);
}
#dsc-hud .bal-key {
  font-family: var(--f-num);
  font-size: var(--t-micro);
  font-weight: 700;
  line-height: 1;
  color: var(--paper);
  padding: calc(var(--u) * 0.16) calc(var(--u) * 0.30);
  border-radius: calc(var(--u) * 0.2);
  background: rgba(4,7,11,0.55);
  box-shadow: inset 0 0 0 1px rgba(255,106,43,0.55);
}

/* brakes */
#dsc-hud .brk-set { display: flex; gap: calc(var(--u) * 0.42); align-items: flex-end; }
#dsc-hud .brk { display: flex; flex-direction: column; align-items: center; gap: calc(var(--u) * 0.22); }
#dsc-hud .brk-track {
  position: relative;
  width: calc(var(--u) * 0.5);
  height: calc(var(--u) * 2.65);
  border-radius: calc(var(--u) * 0.25);
  background: rgba(5,9,14,0.50);
  overflow: hidden;
  box-shadow: inset 0 0 0 1px var(--hair);
}
#dsc-hud .brk-fill {
  position: absolute;
  inset: 0;
  border-radius: calc(var(--u) * 0.25);
  background: linear-gradient(0deg, var(--hot), #FFB37A);
  transform-origin: bottom center;
  transform: scaleY(var(--v, 0));
  will-change: transform;
}
#dsc-hud .brk-key {
  font-size: var(--t-micro);
  font-weight: 700;
  letter-spacing: 0.08em;
  color: var(--mute);
  text-shadow: var(--shadow);
}

/* gear */
#dsc-hud .gear-num {
  font-family: var(--f-num);
  font-size: var(--t-mid);
  font-weight: 650;
  line-height: 1;
  text-shadow: var(--shadow);
  min-width: calc(var(--u) * 1.5);
  text-align: right;
}
#dsc-hud .gear-pips {
  display: flex;
  flex-direction: column-reverse;
  gap: calc(var(--u) * 0.1);
  height: calc(var(--u) * 2.9);
  justify-content: space-between;
}
#dsc-hud .gear-pip {
  width: calc(var(--u) * 0.52);
  height: calc(var(--u) * 0.1);
  background: rgba(214,228,242,0.26);
  border-radius: 1px;
  transform-origin: left center;
  transition: background-color 180ms var(--e-out), transform 220ms var(--e-out);
}
#dsc-hud .gear-pip.on { background: var(--paper); transform: scaleX(1.55); }
#dsc-hud .mod-gear.pedal .gear-num { color: var(--accent); }

/* ---- speed cluster (bottom-right) ---------------------------------------- */
#dsc-hud .cl-speed {
  bottom: calc(var(--sp4) + env(safe-area-inset-bottom, 0px));
  right: calc(var(--sp4) + env(safe-area-inset-right, 0px));
  width: calc(var(--u) * 17);
}
#dsc-hud .gauge-wrap { position: relative; width: 100%; }
#dsc-hud .gauge { display: block; width: 100%; height: auto; overflow: visible; }
#dsc-hud .g-track { fill: none; stroke: rgba(8,13,19,0.55); stroke-width: 9; stroke-linecap: round; }
#dsc-hud .g-red   { fill: none; stroke: rgba(255,106,43,0.34); stroke-width: 9; stroke-linecap: round; }
#dsc-hud .g-ticks { stroke: rgba(226,240,252,0.30); stroke-width: 1.6; stroke-linecap: round; }
#dsc-hud .g-ticks.major { stroke: rgba(226,240,252,0.55); stroke-width: 2.2; }
#dsc-hud .g-fill {
  fill: none;
  stroke-width: 9;
  stroke-linecap: round;
  filter: drop-shadow(0 0 6px rgba(125,238,255,0.45));
  will-change: stroke-dashoffset;
}
#dsc-hud .g-peak { stroke: var(--paper); stroke-width: 2.4; stroke-linecap: round; opacity: 0.8; }
#dsc-hud .sp-read {
  position: absolute;
  left: 50%;
  bottom: calc(var(--u) * 0.15);
  transform: translateX(-50%);
  display: flex;
  align-items: baseline;
  gap: calc(var(--u) * 0.28);
}
#dsc-hud .sp-val {
  font-size: var(--t-huge);
  font-weight: 700;
  line-height: 0.86;
  letter-spacing: -0.035em;
  text-shadow: var(--shadow);
  font-variant-numeric: tabular-nums lining-nums;
  transition: color 260ms var(--e-out);
}
#dsc-hud .cl-speed.redline .sp-val { color: #FFD9C4; }
#dsc-hud .sp-unit {
  font-size: var(--t-micro);
  font-weight: 700;
  letter-spacing: 0.20em;
  color: var(--dim);
  text-shadow: var(--shadow);
  padding-bottom: calc(var(--u) * 0.25);
}
#dsc-hud .sp-foot {
  display: flex;
  justify-content: flex-end;
  align-items: baseline;
  gap: var(--sp2);
  margin-top: calc(var(--u) * -0.1);
}
#dsc-hud .sp-peak-val {
  font-family: var(--f-num);
  font-size: var(--t-small);
  color: var(--dim);
  text-shadow: var(--shadow);
}

/* ---- full-bleed feedback layers ------------------------------------------ */
#dsc-hud .layer {
  position: absolute;
  inset: 0;
  pointer-events: none;
}
#dsc-hud .vig {
  opacity: 0;
  will-change: opacity;
  background:
    radial-gradient(120% 88% at 50% 50%, rgba(160,0,12,0) 42%, rgba(150,6,16,0.62) 88%, rgba(96,2,8,0.86) 100%);
  mix-blend-mode: normal;
}
#dsc-hud .flash {
  opacity: 0;
  will-change: opacity;
  background: linear-gradient(180deg, rgba(125,238,255,0.10), rgba(125,238,255,0) 34%);
}
#dsc-hud .banner {
  position: absolute;
  top: 34%;
  left: 50%;
  transform: translate3d(-50%, 0, 0);
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: var(--sp1);
  opacity: 0;
  will-change: transform, opacity;
}
#dsc-hud .banner-rule {
  width: calc(var(--u) * 13);
  height: 2px;
  background: var(--accent);
  box-shadow: 0 0 14px rgba(125,238,255,0.7);
  transform-origin: center;
}
#dsc-hud .banner.bad .banner-rule { background: var(--bad); box-shadow: 0 0 14px rgba(255,74,94,0.7); }
#dsc-hud .banner.good .banner-rule { background: var(--good); box-shadow: 0 0 14px rgba(70,224,140,0.7); }
#dsc-hud .banner-ttl {
  font-size: var(--t-body);
  font-weight: 700;
  letter-spacing: 0.34em;
  text-transform: uppercase;
  text-indent: 0.34em;
  text-shadow: 0 2px 20px rgba(0,0,0,0.6), 0 1px 3px rgba(0,0,0,0.6);
}
#dsc-hud .banner-val {
  font-family: var(--f-num);
  font-size: var(--t-large);
  font-weight: 650;
  line-height: 1;
  text-shadow: 0 2px 22px rgba(0,0,0,0.55), 0 1px 3px rgba(0,0,0,0.6);
}
#dsc-hud .banner.bad .banner-val { color: var(--bad); }
#dsc-hud .banner.good .banner-val { color: var(--good); }

#dsc-hud .count {
  position: absolute;
  top: 44%;
  left: 50%;
  transform: translate3d(-50%, -50%, 0);
  display: flex;
  flex-direction: column;
  align-items: center;
  opacity: 0;
  will-change: transform, opacity;
  transition: opacity 240ms var(--e-out);
}
#dsc-hud .count.on { opacity: 1; }
#dsc-hud .count-n {
  font-size: var(--t-mega);
  font-weight: 700;
  line-height: 0.84;
  letter-spacing: -0.04em;
  font-variant-numeric: tabular-nums;
  text-shadow: 0 2px 30px rgba(0,0,0,0.6), 0 0 2px rgba(0,0,0,0.5);
  will-change: transform, opacity;
}
#dsc-hud .count-lbl { margin-top: var(--sp2); }

/* ---- debug --------------------------------------------------------------- */
#dsc-hud .dbg {
  position: absolute;
  top: calc(var(--sp4) + env(safe-area-inset-top, 0px));
  right: calc(var(--sp4) + env(safe-area-inset-right, 0px));
  display: none;
  flex-direction: column;
  gap: 1px;
  font-family: var(--f-num);
  font-size: calc(var(--u) * 0.70);
  line-height: 1.45;
  color: rgba(226,240,252,0.72);
  background: rgba(4,7,11,0.55);
  padding: calc(var(--u) * 0.5) calc(var(--u) * 0.7);
  border-radius: calc(var(--u) * 0.3);
  box-shadow: inset 0 0 0 1px var(--hair);
  text-align: right;
  min-width: calc(var(--u) * 11);
}
#dsc-hud.dbg-on .dbg { display: flex; }
#dsc-hud .dbg-row { display: flex; justify-content: space-between; gap: var(--sp3); }
#dsc-hud .dbg-k { color: var(--mute); letter-spacing: 0.08em; }
#dsc-hud .dbg-v { color: var(--paper); }
#dsc-hud .dbg-v.warn { color: var(--hot); }

/* Narrow/short viewports: shed the least important furniture. */
#dsc-hud.compact .cl-prog { display: none; }
#dsc-hud.compact .sp-foot { display: none; }

@media (prefers-reduced-motion: reduce) {
  #dsc-hud .shake { transform: none !important; }
  #dsc-hud * { transition-duration: 1ms !important; }
}
`;

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

export function createHud(ctx) {
  const uid = 'dsc' + (uidCounter++);

  // ---- fallback state so every read is safe before other systems exist ----
  const NULL_V = { x: 0, y: 0, z: 0 };
  const NULL_BIKE = {
    speed: 0, velocity: NULL_V, forward: { x: 0, y: 0, z: -1 }, up: { x: 0, y: 1, z: 0 },
    airborne: false, airTime: 0, lastAirTime: 0, crashed: false, crashTimer: 0,
    brakeFront: 0, brakeRear: 0, pedalling: 0, gForce: 1, surface: 0,
    trailT: 0, distance: 0, lean: 0, offTrail: false, whip: 0, skid: 0,
    riderFore: 0, wheels: null,
  };

  // =========================================================================
  // DOM
  // =========================================================================

  const style = document.createElement('style');
  style.id = uid + '-style';
  style.textContent = CSS;
  document.head.appendChild(style);

  const root = document.createElement('div');
  root.id = 'dsc-hud';

  const shake = h('div', 'shake', root);
  const frame = h('div', 'frame', shake);

  h('div', 'scrim scrim-tl', frame);
  h('div', 'scrim scrim-bl', frame);
  h('div', 'scrim scrim-br', frame);
  h('div', 'scrim scrim-r', frame);

  // ---- timer -------------------------------------------------------------
  const clTimer = h('div', 'cl cl-timer', frame);
  const timerRule = h('div', 'timer-rule', clTimer);
  h('div', 'lbl', clTimer, 'Run Time');
  const elTimer = h('div', 'timer num', clTimer);
  const elTMain = h('span', 't-main', elTimer, '0:00');
  const elTFrac = h('span', 't-frac', elTimer, '.00');
  const timerSub = h('div', 'timer-sub', clTimer);
  h('div', 'lbl', timerSub, 'Best');
  const elBest = h('div', 'best num', timerSub, '--:--.--');
  const elDelta = h('div', 'delta num', clTimer, '+0.00');

  // ---- trail progress ----------------------------------------------------
  const clProg = h('div', 'cl cl-prog', frame);
  h('div', 'lbl', clProg, 'Start');
  const progTrack = h('div', 'prog-track', clProg);
  h('div', 'prog-fill', progTrack);
  const progGhost = h('div', 'prog-ghost', progTrack);
  const progMarker = h('div', 'prog-marker', progTrack);
  const progCap = h('div', 'prog-cap', clProg);
  h('div', 'lbl', progCap, 'Finish');
  const elProgPct = h('div', 'prog-pct num', progCap, '0%');
  let progPips = [];

  // ---- airtime -----------------------------------------------------------
  const clAir = h('div', 'cl cl-air', frame);
  const airRow = h('div', 'air-row', clAir);
  const elAirVal = h('div', 'air-val num', airRow, '0.00');
  h('div', 'air-unit', airRow, 's');
  h('div', 'lbl', clAir, 'Airtime');

  // ---- style meter -------------------------------------------------------
  const clStyle = h('div', 'cl cl-style', frame);
  const chips = h('div', 'chips', clStyle);
  const chipPool = [];
  for (let i = 0; i < 5; i++) {
    const c = h('div', 'chip', chips);
    const cl = h('span', null, c, 'TRICK');
    const cv = h('b', null, c, '+0');
    chipPool.push({ el: c, lbl: cl, val: cv, anim: null });
  }
  const styleHead = h('div', 'style-head', clStyle);
  h('div', 'lbl', styleHead, 'Style');
  const elMult = h('div', 'style-mult num', styleHead, '×1.0');
  const styleBar = h('div', 'style-bar', clStyle);
  h('div', 'style-fill', styleBar);
  const elScore = h('div', 'style-score num', clStyle, '0');

  // ---- telemetry strip ---------------------------------------------------
  const clTele = h('div', 'cl cl-tele', frame);

  const modGrade = h('div', 'mod mod-grade', clTele);
  h('div', 'lbl', modGrade, 'Grade');
  const gradeBody = h('div', 'mod-body', modGrade);
  const inclBox = h('div', 'incl', gradeBody);
  const inclRot = h('div', 'incl-rot', inclBox);
  h('div', 'incl-ground', inclRot);
  h('div', 'incl-hub', inclBox);
  const elGrade = h('div', 'grade-num num', gradeBody, '0%');

  // ---- balance (R9) ------------------------------------------------------
  const modBal = h('div', 'mod mod-bal', clTele);
  h('div', 'lbl', modBal, 'Balance');
  const balBody = h('div', 'mod-body', modBal);
  const balBox = h('div', 'bal', balBody);
  const balRot = h('div', 'bal-rot', balBox);
  h('div', 'bal-rail', balRot);
  h('div', 'bal-axle rear', balRot);
  h('div', 'bal-axle front', balRot);
  const balZone = h('div', 'bal-zone', balRot);
  const balCmdEl = h('div', 'bal-cmd', balRot);
  const balDot = h('div', 'bal-dot', balRot);
  const balCue = h('div', 'bal-cue', modBal);
  const elBalCue = h('div', 'bal-cue-txt', balCue, 'Rear Light');
  const elBalKey = h('div', 'bal-key', balCue, 'S');
  // The band is fixed in wheelbase terms — it is the dot that moves — so it is
  // written once, here, from the same constants the warning states use.
  balZone.style.setProperty('--z0',
    (BAL_X0 + (BAL_X1 - BAL_X0) * BAL_FRONT_LO).toFixed(4));
  balZone.style.setProperty('--zw',
    ((BAL_X1 - BAL_X0) * (1 - BAL_HI - BAL_FRONT_LO)).toFixed(4));

  const modBrake = h('div', 'mod mod-brake', clTele);
  h('div', 'lbl', modBrake, 'Brakes');
  const brakeBody = h('div', 'mod-body', modBrake);
  const brkSet = h('div', 'brk-set', brakeBody);
  function makeBrake(key) {
    const b = h('div', 'brk', brkSet);
    const track = h('div', 'brk-track', b);
    const fill = h('div', 'brk-fill', track);
    h('div', 'brk-key', b, key);
    return fill;
  }
  const brkFrontFill = makeBrake('F');
  const brkRearFill = makeBrake('R');

  const modGear = h('div', 'mod mod-gear', clTele);
  h('div', 'lbl', modGear, 'Gear');
  const gearBody = h('div', 'mod-body', modGear);
  const elGear = h('div', 'gear-num num', gearBody, '1');
  const gearPipsWrap = h('div', 'gear-pips', gearBody);
  const gearPips = [];
  for (let i = 0; i < GEAR_COUNT; i++) gearPips.push(h('div', 'gear-pip', gearPipsWrap));

  // ---- speed cluster -----------------------------------------------------
  const clSpeed = h('div', 'cl cl-speed', frame);
  const gaugeWrap = h('div', 'gauge-wrap', clSpeed);
  const gauge = svgEl('svg', gaugeWrap, {
    class: 'gauge',
    viewBox: `0 0 ${GA.w} ${GA.h}`,
    preserveAspectRatio: 'xMidYMax meet',
    'aria-hidden': 'true',
  });
  {
    const defs = svgEl('defs', gauge);
    const grad = svgEl('linearGradient', defs, {
      id: uid + '-g', x1: '0', y1: '1', x2: '1', y2: '0',
    });
    svgEl('stop', grad, { offset: '0', 'stop-color': '#7DEEFF' });
    svgEl('stop', grad, { offset: '0.62', 'stop-color': '#EAF7FF' });
    svgEl('stop', grad, { offset: '1', 'stop-color': '#FF6A2B' });
  }
  const fullArc = arcPath(GA.cx, GA.cy, GA.r, GA.a0, GA.a1);
  svgEl('path', gauge, { class: 'g-track', d: fullArc });
  svgEl('path', gauge, {
    class: 'g-red',
    d: arcPath(GA.cx, GA.cy, GA.r, gaugeAngle(GAUGE_REDLINE_KMH), GA.a1),
  });
  // Ticks: two paths so minor/major can carry different weights, both built once.
  {
    let minor = '', major = '';
    for (let v = 0; v <= GAUGE_MAX_KMH; v += 10) {
      const a = gaugeAngle(v);
      const isMajor = (v % 30) === 0;
      const rIn = GA.r - (isMajor ? 21 : 15);
      const rOut = GA.r - 7;
      const seg = `M ${polarX(GA.cx, rIn, a).toFixed(2)} ${polarY(GA.cy, rIn, a).toFixed(2)} ` +
                  `L ${polarX(GA.cx, rOut, a).toFixed(2)} ${polarY(GA.cy, rOut, a).toFixed(2)} `;
      if (isMajor) major += seg; else minor += seg;
    }
    svgEl('path', gauge, { class: 'g-ticks', d: minor, fill: 'none' });
    svgEl('path', gauge, { class: 'g-ticks major', d: major, fill: 'none' });
  }
  const gFill = svgEl('path', gauge, {
    class: 'g-fill',
    d: fullArc,
    pathLength: '1000',
    'stroke-dasharray': '1000 1000',
    'stroke-dashoffset': '1000',
    stroke: `url(#${uid}-g)`,
  });
  const gPeak = svgEl('line', gauge, {
    class: 'g-peak',
    x1: GA.cx, y1: GA.cy - GA.r - 16,
    x2: GA.cx, y2: GA.cy - GA.r - 7,
    transform: `rotate(${(GA.a0 - 270).toFixed(2)} ${GA.cx} ${GA.cy})`,
  });
  const spRead = h('div', 'sp-read', gaugeWrap);
  const elSpeed = h('div', 'sp-val num', spRead, '0');
  h('div', 'sp-unit', spRead, 'km/h');
  const spFoot = h('div', 'sp-foot', clSpeed);
  h('div', 'lbl', spFoot, 'Peak');
  const elPeak = h('div', 'sp-peak-val num', spFoot, '0');

  // ---- full-bleed layers (outside .shake) --------------------------------
  const elFlash = h('div', 'layer flash', root);
  const elVig = h('div', 'layer vig', root);
  const banner = h('div', 'banner', root);
  const bannerRule = h('div', 'banner-rule', banner);
  const elBannerTtl = h('div', 'banner-ttl', banner, 'Checkpoint');
  const elBannerVal = h('div', 'banner-val num', banner, '');
  const countBox = h('div', 'count', root);
  const elCountN = h('div', 'count-n', countBox, '3');
  h('div', 'lbl count-lbl', countBox, 'Get Ready');

  // ---- debug -------------------------------------------------------------
  const dbg = h('div', 'dbg', frame);
  function dbgRow(key) {
    const r = h('div', 'dbg-row', dbg);
    h('span', 'dbg-k', r, key);
    return h('span', 'dbg-v', r, '—');
  }
  const dFps = dbgRow('FPS');
  const dMs = dbgRow('MS');
  const dCalls = dbgRow('DRAW');
  const dTris = dbgRow('TRIS');
  const dSpeed = dbgRow('SPD');
  const dSurf = dbgRow('SURF');
  const dTrail = dbgRow('T');
  const dAir = dbgRow('AIR');

  (ctx && ctx.container ? ctx.container : document.body).appendChild(root);

  // =========================================================================
  // Mutable HUD state
  // =========================================================================

  const last = {
    speedInt: -1, peakInt: -1, gaugeF: -1, peakAngle: -999,
    tMain: '', tFrac: '', best: -1,
    progress: -1, ghost: -2, pct: -1,
    grade: -999, gradeDeg: -999, gradeTxt: '', gear: -1,
    balDot: -9, balCmd: -9,
    brakeF: -1, brakeR: -1,
    styleV: -1, mult: -1, score: -1,
    air: '', vig: -1, shakeOn: false,
    countN: '', dbgAt: -1,
  };

  let dispSpeed = 0;        // damped km/h driving the arc (the digits use raw)
  let peakSpeed = 0;
  let dispGrade = 0;        // damped, signed: + = descending
  let groundGrade = 0;      // last on-the-ground reading, held through flights

  // R9 balance. `balRear` holds its last grounded value through a flight, where
  // both wheel loads are zero and the fraction has no meaning.
  let balRear = 1 - 0.45;   // bike.js FRONT_BIAS — the static split, as a seed
  let balCueT = 0;
  let balCueTxt = 'Rear Light';
  let balCueKey = 'S';

  let airVisible = false;
  let airHold = 0;
  let airValue = 0;
  let wasAirborne = false;

  let deltaHold = 0;

  let styleCharge = 0;
  let localScore = 0;
  let gpScoreActive = false;
  let trickEventSeen = false;
  let chipCursor = 0;

  let damage = 0;
  let shakeAmp = 0;
  let shakeT = 0;

  let hidden = false;
  let localPhoto = false;
  let prevPhotoEdge = false;

  let localCountdown = 0;
  let prevState = '';

  let cpDone = 0;
  let bestSplits = null;

  let compact = false;
  let viewW = window.innerWidth || 1920;
  let viewH = window.innerHeight || 1080;

  const reduceMotion = typeof window.matchMedia === 'function'
    ? window.matchMedia('(prefers-reduced-motion: reduce)').matches
    : false;

  // =========================================================================
  // Derived reads (all defensive)
  // =========================================================================

  function bikeState() {
    const b = ctx && ctx.bike;
    const s = b && b.state;
    return s && typeof s.speed === 'number' ? s : NULL_BIKE;
  }

  function gameplay() {
    return (ctx && ctx.gameplay) || null;
  }

  /** Best total time, whether `best` is a number or an object. */
  function bestTime(gp) {
    if (!gp) return NaN;
    const b = gp.best;
    if (Number.isFinite(b)) return b;
    if (b && typeof b === 'object') {
      const t = pickNum(b.time, b.total, b.totalTime, b.duration);
      if (t !== undefined) return t;
    }
    const t2 = pickNum(gp.bestTime);
    return t2 === undefined ? NaN : t2;
  }

  /** Best per-checkpoint split array, if the gameplay module exposes one. */
  function refreshBestSplits(gp) {
    if (!gp) { bestSplits = null; return; }
    const b = gp.best;
    if (b && Array.isArray(b.splits)) { bestSplits = b.splits; return; }
    if (Array.isArray(gp.bestSplits)) { bestSplits = gp.bestSplits; return; }
    bestSplits = null;
  }

  function bestSplitAt(i) {
    if (!bestSplits || i < 0 || i >= bestSplits.length) return NaN;
    const v = bestSplits[i];
    if (Number.isFinite(v)) return v;
    if (v && typeof v === 'object') {
      const t = pickNum(v.time, v.t, v.split);
      if (t !== undefined) return t;
    }
    return NaN;
  }

  // =========================================================================
  // Event feedback
  // =========================================================================

  function showBanner(title, value, tone, holdMs) {
    setText(elBannerTtl, title);
    setText(elBannerVal, value);
    setFlag(banner, 'good', tone === 'good');
    setFlag(banner, 'bad', tone === 'bad');
    const d = holdMs || 1500;
    play(bannerRule, [
      { transform: 'scaleX(0.06)', opacity: 0 },
      { transform: 'scaleX(1)', opacity: 1, offset: 0.16 },
      { transform: 'scaleX(1)', opacity: 1, offset: 0.74 },
      { transform: 'scaleX(1.04)', opacity: 0 },
    ], { duration: d, easing: 'cubic-bezier(0.16,1,0.30,1)' });
    play(banner, [
      { opacity: 0, transform: 'translate3d(-50%, 14px, 0)' },
      { opacity: 1, transform: 'translate3d(-50%, 0, 0)', offset: 0.14 },
      { opacity: 1, transform: 'translate3d(-50%, 0, 0)', offset: 0.72 },
      { opacity: 0, transform: 'translate3d(-50%, -12px, 0)' },
    ], { duration: d, easing: 'cubic-bezier(0.16,1,0.30,1)' });
  }

  function screenFlash(strength, tint) {
    if (tint) elFlash.style.background = tint;
    play(elFlash, [
      { opacity: 0 },
      { opacity: clamp01(strength), offset: 0.10 },
      { opacity: 0 },
    ], { duration: 640, easing: 'cubic-bezier(0.16,1,0.30,1)' });
  }

  function pushChip(label, points) {
    const c = chipPool[chipCursor];
    chipCursor = (chipCursor + 1) % chipPool.length;
    setText(c.lbl, label);
    setText(c.val, points > 0 ? '+' + Math.round(points) : '');
    // Fan the chips out a little so a rapid burst does not stack into one blob.
    const jitter = ((chipCursor * 37) % 40) - 20;
    if (c.anim) { try { c.anim.cancel(); } catch (e) { /* ignore */ } }
    c.anim = play(c.el, [
      { opacity: 0, transform: `translate3d(calc(-50% + ${jitter * 0.25}px), 16px, 0) scale(0.9)` },
      { opacity: 1, transform: `translate3d(calc(-50% + ${jitter * 0.5}px), 0px, 0) scale(1)`, offset: 0.13 },
      { opacity: 1, transform: `translate3d(calc(-50% + ${jitter * 0.8}px), -14px, 0) scale(1)`, offset: 0.66 },
      { opacity: 0, transform: `translate3d(calc(-50% + ${jitter}px), -40px, 0) scale(0.97)` },
    ], { duration: 1650, easing: 'cubic-bezier(0.16,1,0.30,1)', fill: 'both' });
  }

  function addShake(amount) {
    if (reduceMotion) return;
    shakeAmp = Math.min(1.25, shakeAmp + clamp01(amount));
  }

  // ---- subscriptions -----------------------------------------------------

  const unsubs = [];
  function on(name, fn) {
    if (ctx && ctx.events && ctx.events.on) unsubs.push(ctx.events.on(name, fn));
  }

  on('run:start', () => {
    cpDone = 0;
    deltaHold = 0;
    setFlag(elDelta, 'on', false);
    localScore = 0;
    styleCharge = 0;
    peakSpeed = 0;
    damage = 0;
    balRear = 1 - 0.45;
    balCueT = 0;
    for (const p of progPips) setFlag(p, 'done', false);
    countBox.classList.remove('on');
    showBanner('Go', '', 'good', 1100);
  });

  on('run:checkpoint', (p) => {
    const gp = gameplay();
    refreshBestSplits(gp);
    let idx = pickNum(p && p.index, p && p.checkpoint, p && p.i);
    if (idx === undefined) idx = cpDone;
    const now = pickNum(
      p && p.time, p && p.split, p && p.elapsed,
      gp && gp.time,
    );
    let delta = pickNum(p && p.delta, p && p.deltaBest, p && p.diff, p && p.gap);
    if (delta === undefined) {
      const ref = bestSplitAt(idx);
      if (Number.isFinite(ref) && now !== undefined) delta = now - ref;
    }

    cpDone = Math.max(cpDone, idx + 1);
    for (let i = 0; i < progPips.length; i++) setFlag(progPips[i], 'done', i < cpDone);

    if (delta !== undefined && Number.isFinite(delta)) {
      const good = delta <= 0;
      setText(elDelta, fmtDelta(delta));
      setFlag(elDelta, 'good', good);
      setFlag(elDelta, 'bad', !good);
      setFlag(elDelta, 'on', true);
      deltaHold = DELTA_HOLD;
      play(elDelta, [
        { transform: 'translate3d(0,0,0) scale(1.16)' },
        { transform: 'translate3d(0,0,0) scale(1)' },
      ], { duration: 520, easing: 'cubic-bezier(0.22,1.35,0.36,1)' });
      showBanner('Split ' + (idx + 1), fmtDelta(delta), good ? 'good' : 'bad', 1500);
      screenFlash(good ? 0.55 : 0.4,
        good ? 'linear-gradient(180deg, rgba(70,224,140,0.16), rgba(70,224,140,0) 36%)'
             : 'linear-gradient(180deg, rgba(255,74,94,0.14), rgba(255,74,94,0) 36%)');
    } else {
      showBanner('Split ' + (idx + 1), now !== undefined ? fmtClock(now) : '', null, 1400);
      screenFlash(0.5, 'linear-gradient(180deg, rgba(125,238,255,0.14), rgba(125,238,255,0) 36%)');
    }
  });

  on('run:finish', (p) => {
    const gp = gameplay();
    const t = pickNum(p && p.time, p && p.total, gp && gp.time);
    const isBest = !!(p && (p.isBest || p.newBest || p.record));
    showBanner(isBest ? 'New Best' : 'Finish', t !== undefined ? fmtClock(t) : '',
      isBest ? 'good' : null, 2600);
    screenFlash(0.7, 'linear-gradient(180deg, rgba(125,238,255,0.18), rgba(125,238,255,0) 40%)');
    deltaHold = 0;
  });

  on('run:crash', (p) => {
    // `severity` is not on a pinned scale — bike.js reports an impact velocity, other
    // callers may pass 0..1. Normalise anything > 1.5 as m/s over a ~12 m/s range.
    const raw = pickNum(p && p.severity, 0.8);
    const sev = raw > 1.5 ? clamp01(raw / 12) : clamp01(raw);
    damage = Math.min(1, damage + 0.52 + sev * 0.40);
    addShake(0.55 + sev * 0.55);
    showBanner('Crash', '', 'bad', 1400);
  });

  on('bike:impact', (p) => {
    // bike.js reports severity as an impact normal velocity in m/s.
    const v = pickNum(p && p.severity, 0);
    if (v > 4.2) {
      const k = clamp01((v - 4.2) / 9);
      addShake(0.18 + k * 0.75);
      if (v > 9) damage = Math.min(1, damage + k * 0.30);
    }
  });

  on('bike:landed', (p) => {
    const at = pickNum(p && p.airTime, 0);
    const cased = !!(p && p.cased);
    const q = pickNum(p && p.quality, 1);
    if (at > 0.35) addShake(cased ? 0.7 : clamp01(0.12 + at * 0.16) * (1.2 - q * 0.5));
    if (cased) damage = Math.min(1, damage + 0.22);
    // Only self-score when the gameplay layer has never scored a trick.
    if (!trickEventSeen && at > 0.55) {
      const pts = Math.round(at * 140 * (0.55 + q * 0.75));
      localScore += pts;
      styleCharge = clamp01(styleCharge + 0.20 + at * 0.10);
      pushChip(at > 1.6 ? 'Huge Air' : 'Air', pts);
    }
  });

  on('trick:landed', (p) => {
    trickEventSeen = true;
    const name = (p && (p.type || p.name || p.trick)) || 'Trick';
    const pts = pickNum(p && p.points, p && p.score, p && p.value, 0);
    localScore += pts;
    styleCharge = clamp01(styleCharge + 0.18 + clamp01(pts / 900) * 0.3);
    pushChip(String(name), pts);
  });

  on('bike:respawn', () => {
    damage = Math.min(damage, 0.25);
    styleCharge = 0;
    // A respawn must not inherit the balance state that caused the crash.
    balRear = 1 - 0.45;
    balCueT = 0;
  });

  // =========================================================================
  // Lazy build: checkpoint pips need ctx.trail
  // =========================================================================

  let pipsBuilt = false;
  function ensurePips() {
    if (pipsBuilt) return;
    const trail = ctx && ctx.trail;
    const cps = trail && trail.checkpoints;
    if (!cps || !cps.length) return;
    const finishT = Number.isFinite(trail.finishT) ? trail.finishT : 1;
    for (let i = 0; i < cps.length; i++) {
      const t = clamp01((cps[i] && cps[i].t) / (finishT || 1));
      const pip = h('div', 'prog-pip', progTrack);
      pip.style.top = (t * 100).toFixed(2) + '%';
      progPips.push(pip);
    }
    // Marker and ghost must paint above the pips.
    progTrack.appendChild(progGhost);
    progTrack.appendChild(progMarker);
    pipsBuilt = true;
  }

  // =========================================================================
  // Frame update
  // =========================================================================

  function lateUpdate(dt) {
    const d = Math.min(Number.isFinite(dt) ? dt : 0, 1 / 20);
    const bs = bikeState();
    const gp = gameplay();
    const state = (gp && typeof gp.state === 'string') ? gp.state : 'running';

    ensurePips();

    // ---- photo mode / visibility -----------------------------------------
    const inputState = ctx && ctx.input && ctx.input.state;
    const edge = !!(inputState && inputState.photoMode);
    if (edge && !prevPhotoEdge) localPhoto = !localPhoto;
    prevPhotoEdge = edge;
    const photo = (typeof ctx.photoMode === 'boolean') ? ctx.photoMode : localPhoto;

    const shouldHide = hidden || photo || state === 'menu';
    setFlag(root, 'is-hidden', shouldHide);
    setFlag(root, 'is-dimmed', !shouldHide && state === 'paused');
    setFlag(root, 'dbg-on', !!(ctx.debug && ctx.debug.enabled));

    // Nothing below is visible; skip the work entirely.
    if (shouldHide) {
      if (shakeAmp > 0) { shakeAmp = 0; shake.style.transform = 'translate3d(0,0,0)'; }
      return;
    }

    // ---- speed ------------------------------------------------------------
    const kmh = Math.max(0, (Number.isFinite(bs.speed) ? bs.speed : 0) * 3.6);
    // Digits track raw speed (immediate), the arc lags very slightly so it sweeps
    // rather than twitches — the difference is what makes a gauge feel mechanical.
    dispSpeed = damp(dispSpeed, kmh, 13, d);
    if (kmh > peakSpeed) peakSpeed = kmh;

    const speedInt = Math.round(kmh);
    if (speedInt !== last.speedInt) {
      last.speedInt = speedInt;
      setText(elSpeed, String(speedInt));
    }
    const peakInt = Math.round(peakSpeed);
    if (peakInt !== last.peakInt) {
      last.peakInt = peakInt;
      setText(elPeak, String(peakInt));
    }
    setFlag(clSpeed, 'redline', kmh >= GAUGE_REDLINE_KMH);

    const gf = clamp01(dispSpeed / GAUGE_MAX_KMH);
    if (Math.abs(gf - last.gaugeF) > 0.0012) {
      last.gaugeF = gf;
      gFill.style.strokeDashoffset = (1000 - gf * 1000).toFixed(1);
    }
    const pa = gaugeAngle(peakSpeed) - 270;
    if (Math.abs(pa - last.peakAngle) > 0.25) {
      last.peakAngle = pa;
      gPeak.setAttribute('transform', `rotate(${pa.toFixed(2)} ${GA.cx} ${GA.cy})`);
    }

    // ---- timer ------------------------------------------------------------
    const runTime = Number.isFinite(gp && gp.time) ? gp.time : 0;
    const tm = fmtClockMain(runTime);
    if (tm !== last.tMain) { last.tMain = tm; setText(elTMain, tm); }
    const tf = fmtClockFrac(runTime);
    if (tf !== last.tFrac) { last.tFrac = tf; setText(elTFrac, tf); }

    const bt = bestTime(gp);
    const btKey = Number.isFinite(bt) ? bt : -1;
    if (btKey !== last.best) {
      last.best = btKey;
      setText(elBest, Number.isFinite(bt) ? fmtClock(bt) : '--:--.--');
    }

    if (deltaHold > 0) {
      deltaHold -= d;
      if (deltaHold <= 0) setFlag(elDelta, 'on', false);
    }

    // ---- trail progress ---------------------------------------------------
    const trail = ctx && ctx.trail;
    const finishT = (trail && Number.isFinite(trail.finishT)) ? trail.finishT : 1;
    const rawT = Number.isFinite(bs.trailT) ? bs.trailT : 0;
    const prog = clamp01(rawT / (finishT || 1));
    if (Math.abs(prog - last.progress) > 0.0008) {
      last.progress = prog;
      clProg.style.setProperty('--p', prog.toFixed(4));
    }
    const pct = Math.round(prog * 100);
    if (pct !== last.pct) { last.pct = pct; setText(elProgPct, pct + '%'); }

    const gt = pickNum(gp && gp.ghostT, gp && gp.ghostProgress);
    const hasGhost = gt !== undefined;
    setFlag(progGhost, 'on', hasGhost);
    if (hasGhost) {
      const gv = clamp01(gt / (finishT || 1));
      if (Math.abs(gv - last.ghost) > 0.0012) {
        last.ghost = gv;
        clProg.style.setProperty('--g', gv.toFixed(4));
      }
    }

    // ---- gradient / inclinometer -----------------------------------------
    // Chassis forward pitch is the honest read of the slope under the bike and it
    // stays valid at zero speed; while airborne we hold the last ground value so
    // the needle does not swing with the bike's rotation.
    const fw = bs.forward || NULL_BIKE.forward;
    const fh = Math.hypot(fw.x || 0, fw.z || 0);
    if (!bs.airborne && fh > 1e-3) {
      groundGrade = clamp(-(fw.y || 0) / fh, -1.2, 1.2);
    }
    dispGrade = damp(dispGrade, groundGrade, bs.airborne ? 1.2 : 4.5, d);

    // Horizon rotates on a fine threshold (it is a continuous analogue element);
    // the numeral only changes on whole percent, so it is not restyled at 60 Hz.
    const deg = clamp(Math.atan(dispGrade) * 180 / Math.PI, -46, 46);
    if (Math.abs(deg - last.gradeDeg) > 0.12) {
      last.gradeDeg = deg;
      const a = deg.toFixed(2) + 'deg';
      inclRot.style.setProperty('--a', a);
      // The balance rail is the same slope seen from the side, so it must use
      // the same number — a descent tilts it nose-down to the right, which is
      // the direction of travel.
      balRot.style.setProperty('--a', a);
    }
    const gradePct = Math.round(dispGrade * 100);
    if (gradePct !== last.grade) {
      last.grade = gradePct;
      // Cycling convention: a descent reads negative.
      const txt = (gradePct > 0 ? '−' : gradePct < 0 ? '+' : '') + Math.abs(gradePct) + '%';
      if (txt !== last.gradeTxt) { last.gradeTxt = txt; setText(elGrade, txt); }
      const steep = Math.abs(gradePct) > 24;
      setFlag(inclBox, 'steep', steep);
      setFlag(elGrade, 'steep', steep);
    }

    // ---- balance (R9) -----------------------------------------------------
    // Weight shift decides whether the steep sections are rideable at all, and
    // nothing else on screen says so. This teaches it the way a racing game
    // teaches: by making the consequence visible while it is still avoidable.
    const wheels = bs.wheels;
    const wF = wheels && wheels[0], wR = wheels && wheels[1];
    const loadF = wF && Number.isFinite(wF.load) ? Math.max(0, wF.load) : 0;
    const loadR = wR && Number.isFinite(wR.load) ? Math.max(0, wR.load) : 0;
    const loadSum = loadF + loadR;
    // 40 N is well under the noise floor of a wheel that is genuinely carrying
    // something; below it (or airborne, or crashed) the fraction is meaningless
    // and the module holds its last reading rather than snapping to centre.
    const grounded = loadSum > 40 && !bs.airborne && !bs.crashed;
    if (grounded) balRear = damp(balRear, loadR / loadSum, 7, d);
    const frontFrac = clamp01(1 - balRear);

    const riderFore = Number.isFinite(bs.riderFore) ? bs.riderFore : 0;
    const balCmd = clamp(riderFore / RIDER_SHIFT, -1, 1);

    const balSteep = Math.abs(dispGrade) > BAL_CUE_GRADE;
    const rearLight = balRear < BAL_HI;
    const rearCrit = balRear < BAL_LO;
    const frontLight = frontFrac < BAL_FRONT_LO;
    const balWarn = grounded && (rearLight || frontLight);

    // The cue names the state; it does not scold, and it goes quiet the moment
    // the player is already doing everything the control can do.
    let cueTxt = '';
    if (grounded && balSteep && rearCrit && balCmd > -0.75) cueTxt = 'rear';
    else if (grounded && frontLight && balCmd < 0.75) cueTxt = 'front';
    if (cueTxt) {
      const pad = !!(ctx && ctx.input && ctx.input.hasGamepad);
      balCueTxt = cueTxt === 'rear' ? 'Rear Light' : 'Front Light';
      balCueKey = cueTxt === 'rear' ? (pad ? '↓' : 'S') : (pad ? '↑' : 'W');
      balCueT = BAL_CUE_HOLD;
    } else if (balCueT > 0) {
      balCueT = Math.max(0, balCueT - d);
    }
    const cueOn = balCueT > 0 && !bs.crashed;
    if (cueOn) { setText(elBalCue, balCueTxt); setText(elBalKey, balCueKey); }

    setFlag(modBal, 'live',
      balSteep || balWarn || cueOn || Math.abs(balCmd) > 0.12);
    setFlag(modBal, 'warn', balWarn);
    setFlag(modBal, 'air', !grounded);
    setFlag(modBal, 'cmd', Math.abs(balCmd) > 0.06);
    setFlag(modBal, 'shifting', balCmd < -0.25);
    setFlag(modBal, 'cue', cueOn);

    // Front load fraction is the CG's position along the wheelbase from the
    // rear axle, so it places the dot directly.
    const dotF = BAL_X0 + (BAL_X1 - BAL_X0) * frontFrac;
    if (Math.abs(dotF - last.balDot) > 0.0015) {
      last.balDot = dotF;
      balDot.style.setProperty('--x', dotF.toFixed(4));
    }
    // The caret is the raw command, on its own small scale about the centre of
    // the rail — it is the input, not a second claim about where the mass is.
    const cmdF = 0.5 + balCmd * 0.20;
    if (Math.abs(cmdF - last.balCmd) > 0.0015) {
      last.balCmd = cmdF;
      balCmdEl.style.setProperty('--x', cmdF.toFixed(4));
    }

    // ---- brakes -----------------------------------------------------------
    const bf = clamp01(bs.brakeFront || 0);
    const br = clamp01(bs.brakeRear || 0);
    if (Math.abs(bf - last.brakeF) > 0.004) {
      last.brakeF = bf;
      brkFrontFill.style.setProperty('--v', bf.toFixed(3));
    }
    if (Math.abs(br - last.brakeR) > 0.004) {
      last.brakeR = br;
      brkRearFill.style.setProperty('--v', br.toFixed(3));
    }

    // ---- gear -------------------------------------------------------------
    let gear = 1;
    for (let i = GEAR_COUNT - 1; i >= 0; i--) {
      if (kmh >= GEAR_SPEEDS[i]) { gear = i + 1; break; }
    }
    if (gear !== last.gear) {
      last.gear = gear;
      setText(elGear, String(gear));
      for (let i = 0; i < gearPips.length; i++) setFlag(gearPips[i], 'on', i < gear);
    }
    setFlag(modGear, 'pedal', (bs.pedalling || 0) > 0.12);

    // ---- airtime ----------------------------------------------------------
    const airborne = !!bs.airborne;
    const at = Number.isFinite(bs.airTime) ? bs.airTime : 0;
    if (airborne && at >= AIR_MIN_SHOW) {
      airValue = at;
      airHold = AIR_HOLD;
      airVisible = true;
    } else if (airHold > 0) {
      airHold -= d;
      if (airHold <= 0) airVisible = false;
    } else {
      airVisible = false;
    }
    if (wasAirborne && !airborne) {
      const fin = Number.isFinite(bs.lastAirTime) && bs.lastAirTime > 0 ? bs.lastAirTime : airValue;
      airValue = fin;
      if (fin >= AIR_MIN_SHOW) {
        airHold = AIR_HOLD;
        airVisible = true;
        play(clAir, [
          { transform: 'translate3d(-50%, 0, 0) scale(1.20)' },
          { transform: 'translate3d(-50%, 0, 0) scale(1)' },
        ], { duration: 560, easing: 'cubic-bezier(0.22,1.35,0.36,1)' });
      }
    }
    wasAirborne = airborne;

    setFlag(clAir, 'on', airVisible);
    if (airVisible) {
      const s = airValue.toFixed(2);
      if (s !== last.air) { last.air = s; setText(elAirVal, s); }
      setFlag(clAir, 'big', airValue >= 1.1 && airValue < 2.0);
      setFlag(clAir, 'huge', airValue >= 2.0);
    }

    // ---- style / combo ----------------------------------------------------
    // Charge builds a little while airborne (commitment), bleeds off on the ground.
    if (airborne) styleCharge = clamp01(styleCharge + d * 0.16);
    else styleCharge = Math.max(0, styleCharge - d * (0.20 + styleCharge * 0.16));

    if (gp && Number.isFinite(gp.score) && gp.score !== 0) gpScoreActive = true;
    const score = (gpScoreActive && gp && Number.isFinite(gp.score))
      ? Math.round(gp.score) : Math.round(localScore);

    const mult = 1 + Math.floor(styleCharge * 6) * 0.5;
    const shownMult = pickNum(gp && gp.multiplier, gp && gp.combo, mult);

    setFlag(clStyle, 'on', score > 0 || styleCharge > 0.01);
    if (Math.abs(styleCharge - last.styleV) > 0.004) {
      last.styleV = styleCharge;
      styleBar.style.setProperty('--v', styleCharge.toFixed(3));
    }
    const mKey = Math.round(shownMult * 10);
    if (mKey !== last.mult) {
      last.mult = mKey;
      setText(elMult, '×' + (mKey / 10).toFixed(1));
    }
    if (score !== last.score) {
      last.score = score;
      setText(elScore, score.toLocaleString('en-US'));
    }

    // ---- countdown --------------------------------------------------------
    if (state === 'countdown') {
      let cv = pickNum(gp && gp.countdown, gp && gp.countdownTime, gp && gp.timeToStart);
      if (cv === undefined) {
        if (prevState !== 'countdown') localCountdown = 3.0;
        localCountdown = Math.max(0, localCountdown - d);
        cv = localCountdown;
      }
      const n = cv <= 0.02 ? 'Go' : String(Math.ceil(cv));
      countBox.classList.add('on');
      if (n !== last.countN) {
        last.countN = n;
        setText(elCountN, n);
        play(elCountN, [
          { opacity: 0, transform: 'scale(1.55)' },
          { opacity: 1, transform: 'scale(1)', offset: 0.28 },
          { opacity: 1, transform: 'scale(1)', offset: 0.7 },
          { opacity: 0.0, transform: 'scale(0.94)' },
        ], { duration: 940, easing: 'cubic-bezier(0.16,1,0.30,1)' });
      }
    } else if (countBox.classList.contains('on')) {
      countBox.classList.remove('on');
      last.countN = '';
    }
    prevState = state;

    // ---- damage vignette --------------------------------------------------
    const crashed = !!bs.crashed;
    if (crashed) damage = Math.min(1, damage + d * 1.6);
    else damage = Math.max(0, damage - d * DAMAGE_DECAY);
    // A gentle pulse while the vignette is up reads as "hurt", not "broken UI".
    const pulse = damage > 0.02 ? 1 + Math.sin(ctx.time * 6.0) * 0.07 * damage : 1;
    const vigOpacity = clamp01(damage * 0.92 * pulse);
    if (Math.abs(vigOpacity - last.vig) > 0.004) {
      last.vig = vigOpacity;
      elVig.style.opacity = vigOpacity.toFixed(3);
    }

    // ---- impact shake -----------------------------------------------------
    if (shakeAmp > 0.0008) {
      shakeAmp *= Math.exp(-SHAKE_DECAY * d);
      shakeT += d;
      const a = shakeAmp;
      // Two incommensurate frequencies per axis so it never reads as a sine wobble.
      const x = (Math.sin(shakeT * 61.3) * 0.65 + Math.sin(shakeT * 37.1) * 0.35) * a * 9;
      const y = (Math.sin(shakeT * 53.7 + 1.7) * 0.6 + Math.sin(shakeT * 29.3) * 0.4) * a * 7;
      const r = Math.sin(shakeT * 41.9 + 0.6) * a * 0.42;
      shake.style.transform =
        'translate3d(' + x.toFixed(2) + 'px,' + y.toFixed(2) + 'px,0) rotate(' + r.toFixed(3) + 'deg)';
      last.shakeOn = true;
    } else if (last.shakeOn) {
      last.shakeOn = false;
      shakeAmp = 0;
      shake.style.transform = 'translate3d(0,0,0)';
    }

    // ---- debug block (5 Hz) -----------------------------------------------
    if (ctx.debug && ctx.debug.enabled) {
      if (ctx.time - last.dbgAt > 0.2) {
        last.dbgAt = ctx.time;
        const st = (ctx.engine && ctx.engine.stats) || null;
        const fps = st && Number.isFinite(st.fps) ? st.fps : 0;
        setText(dFps, fps.toFixed(0));
        setFlag(dFps, 'warn', fps > 0 && fps < 50);
        setText(dMs, st ? (st.frameTimeMs || 0).toFixed(1) : '—');
        const calls = st ? st.drawCalls : 0;
        setText(dCalls, st ? String(calls) : '—');
        setFlag(dCalls, 'warn', calls > 400);
        setText(dTris, st ? ((st.triangles || 0) / 1000).toFixed(0) + 'k' : '—');
        setText(dSpeed, kmh.toFixed(1));
        const sid = Number.isFinite(bs.surface) ? bs.surface : 0;
        setText(dSurf, SURFACE_NAMES[sid] || String(sid));
        setText(dTrail, rawT.toFixed(3));
        setText(dAir, airborne ? at.toFixed(2) : '—');
      }
    }
  }

  // =========================================================================
  // Public API
  // =========================================================================

  function resize(w, hgt) {
    viewW = Number.isFinite(w) && w > 0 ? w : (window.innerWidth || viewW);
    viewH = Number.isFinite(hgt) && hgt > 0 ? hgt : (window.innerHeight || viewH);
    const nextCompact = viewW < 1080 || viewH < 620;
    if (nextCompact !== compact) {
      compact = nextCompact;
      setFlag(root, 'compact', compact);
    }
  }
  resize(viewW, viewH);

  return {
    root,

    show() { hidden = false; },
    hide() { hidden = true; },
    get photoMode() { return localPhoto; },
    setPhotoMode(v) { localPhoto = !!v; },

    /** Exposed so other systems can punch the HUD without going through events. */
    shake: addShake,
    flash: screenFlash,
    banner: showBanner,

    lateUpdate,
    resize,

    dispose() {
      for (const off of unsubs) { try { off && off(); } catch (e) { /* ignore */ } }
      unsubs.length = 0;
      for (const c of chipPool) { if (c.anim) { try { c.anim.cancel(); } catch (e) { /* ignore */ } } }
      if (root.parentNode) root.parentNode.removeChild(root);
      if (style.parentNode) style.parentNode.removeChild(style);
      progPips.length = 0;
    },
  };
}
