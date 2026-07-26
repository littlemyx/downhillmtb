// =============================================================================
// DESCENT — src/ui/menu.js
// Title screen, pause overlay, run-summary card, settings panel and controls
// reference. A DOM overlay (CONTRACT §8), injecting its own <style>, sharing the
// HUD's design language: confident typography, restrained colour, expo easing.
//
// Design notes (the "why"):
//  - The render is never fully hidden. Every panel sits on a directional scrim
//    (dark bottom-left → clear top-right) plus a backdrop blur, so type stays
//    legible while the mountain keeps playing behind it. That is what separates a
//    title screen from a modal dialog.
//  - Motion is all cubic-bezier(0.16, 1, 0.3, 1) — a long, decelerating expo out.
//    Cheap UI uses linear/ease; expensive UI settles.
//  - Every animated element is staggered off a --i custom property set once at
//    build time, so nothing is animated from JS per frame.
//
// =============================================================================
// CONTRACT-NOTE: `src/game/input.js` does not export its binding table (it only
//   exports `createInput`), so the Controls reference cannot be *derived* from the
//   real bindings as the brief asks. This file therefore:
//     1. prefers `ctx.input.bindings` if the input module ever exposes one,
//     2. then the module-level export `BINDINGS` / `KEY_BINDINGS` (imported via a
//        namespace import so it costs nothing if absent),
//     3. and finally falls back to KEYMAP below, which mirrors input.js exactly as
//        of this build (keydown codes + the standard-mapping button indices used in
//        its update()).
//   Please have input.js export `BINDINGS` in the shape documented above KEYMAP and
//   this fallback becomes dead code.
//
// CONTRACT-NOTE: `gameplay` owns run state. This module never mutates
//   `gameplay.time`/`splits`/`best`. It does set `gameplay.state = 'menu'` while the
//   title screen is up and `gameplay.mode = 'free' | 'timed'` when a mode is chosen
//   (plus an event, `run:mode` `{ mode }`) because CONTRACT §8 has a 'menu' state but
//   no API to enter it and no free-ride entry point. If gameplay would rather own
//   that, expose `gameplay.toMenu()` / `gameplay.start({ mode })` and this module
//   will prefer them — both are probed before the property fallback.
//
// CONTRACT-NOTE: `run:finish` has no documented payload shape. This module accepts a
//   wide, tolerant range (see normaliseResult) and independently tracks top speed,
//   biggest air and crash count off `ctx.bike.state` + the `run:crash` event, using
//   its own numbers ONLY where the payload/gameplay does not supply them. It also
//   keeps a localStorage personal best (`descent.best.v1`) as a fallback for the
//   "new personal best" treatment when `gameplay.best` is absent.
//
// CONTRACT-NOTE: settings this module adds to `ctx.settings`, all optional for other
//   systems to honour: `fov` (degrees — base FOV before the camera's speed kick),
//   `masterVolume` (0..1), `invertLook` (bool), `photoMode` (bool). Each change also
//   emits `settings:changed` `{ key, value }`, plus `audio:volume` and `photo:mode`
//   `{ enabled }`. chaseCamera should read `ctx.settings.fov` as its base FOV; audio
//   should honour `ctx.settings.masterVolume` (a `setMasterVolume`/`setVolume` method
//   is called directly if present).
// =============================================================================

import * as InputModule from '../game/input.js';

// ---------------------------------------------------------------------------
// Bindings (see CONTRACT-NOTE above). Shape:
//   [{ group, items: [{ action, note, keys: [[cap, ...], ...], pad }] }]
// where `keys` is a list of alternative chord-groups: [['A','D'], ['←','→']]
// renders as   A / D   or   ← / →.
// ---------------------------------------------------------------------------
const KEYMAP = [
  {
    group: 'Riding',
    items: [
      { action: 'Steer', note: 'lean the bike into the turn', keys: [['A', 'D'], ['←', '→']], pad: 'Left stick ←→' },
      { action: 'Weight fore / aft', note: 'attack position, manual, air pitch', keys: [['W', 'S'], ['↑', '↓']], pad: 'Left stick ↑↓' },
      { action: 'Roll / whip', note: 'in the air only', keys: [['Q', 'E']], pad: 'Right stick ←→' },
      { action: 'Pump', note: 'hold through the compression, release over the crest', keys: [['Space']], pad: 'A / ✕' },
      { action: 'Manual', note: 'hold to find the balance point', keys: [['M']], pad: 'X / ▢' },
    ],
  },
  {
    group: 'Bike',
    items: [
      { action: 'Front brake', note: 'analogue — modulate it', keys: [['J']], pad: 'L2 (analogue)' },
      { action: 'Rear brake', note: 'lock it up to scrub or drift', keys: [['K']], pad: 'R2 (analogue)' },
      { action: 'Pedal', note: 'sprint out of the flat corners', keys: [['Shift']], pad: 'R1' },
      { action: 'Reset to checkpoint', keys: [['R']], pad: 'B / ○' },
    ],
  },
  {
    group: 'System',
    items: [
      { action: 'Pause', keys: [['Esc'], ['P']], pad: 'Start' },
      { action: 'Cycle camera', note: 'chase · wide · first person · cinematic', keys: [['C']], pad: 'Y / △' },
      { action: 'Photo mode', note: 'hides all interface', keys: [['F']], pad: 'Select / Share' },
    ],
  },
];

const QUALITY_OPTIONS = [
  { value: 'low', label: 'Low' },
  { value: 'medium', label: 'Medium' },
  { value: 'high', label: 'High' },
  { value: 'ultra', label: 'Ultra' },
];

const CAMERA_OPTIONS = [
  { value: 'chase', label: 'Chase' },
  { value: 'chaseFar', label: 'Wide' },
  { value: 'firstPerson', label: 'First person' },
  { value: 'cinematic', label: 'Cinematic' },
];

const ROMAN = [null, 'I', 'II', 'III', 'IV', 'V'];

// F1..F12 — never swallowed, so devtools/reload still work with a menu open.
const FN_KEY = /^F\d{1,2}$/;

const SETTINGS_KEY = 'descent.settings.v1';
const BEST_KEY = 'descent.best.v1';

// Nav repeat timing for a held stick (seconds).
const NAV_REPEAT_DELAY = 0.38;
const NAV_REPEAT_RATE = 0.11;

// ---------------------------------------------------------------------------
// Stylesheet. One injected <style>; everything is namespaced under .dm- so it can
// never leak into the HUD's DOM.
// ---------------------------------------------------------------------------
const CSS = `
.dm-root {
  --ink: 5 7 11;
  --paper: 255 255 255;
  --accent: 255 178 82;
  --gold: 255 209 122;
  --good: 118 240 168;
  --bad: 255 108 96;
  --ease: cubic-bezier(0.16, 1, 0.3, 1);
  --ease-soft: cubic-bezier(0.33, 1, 0.68, 1);
  --pad: clamp(26px, 4.4vw, 84px);

  position: fixed;
  inset: 0;
  z-index: 60;
  pointer-events: none;
  color: rgb(var(--paper));
  font-family: ui-sans-serif, system-ui, -apple-system, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
  font-synthesis-weight: none;
  -webkit-font-smoothing: antialiased;
  text-rendering: optimizeLegibility;
  user-select: none;
  -webkit-user-select: none;
}
.dm-root.dm-photo { opacity: 0; visibility: hidden; transition: opacity .28s var(--ease-soft); }

.dm-root *,
.dm-root *::before,
.dm-root *::after { box-sizing: border-box; }

/* :where() keeps the reset at .dm-root's specificity (0,1,0) so the component
   classes below — which are also 0,1,0 but come later — still win. A plain
   ".dm-root button" selector is 0,1,1 and would silently zero every padding and
   border set further down. */
.dm-root :where(button) { font: inherit; color: inherit; background: none; border: 0; margin: 0; padding: 0; cursor: pointer; }
.dm-root :where(button):focus { outline: none; }
.dm-root :where(kbd) { font: inherit; }

/* --- shared atoms -------------------------------------------------------- */
.dm-num {
  font-family: ui-monospace, "SF Mono", SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace;
  font-variant-numeric: tabular-nums;
  font-feature-settings: "tnum" 1, "zero" 1;
  letter-spacing: -0.01em;
}
.dm-eyebrow {
  font-size: clamp(9px, 0.72vw, 11px);
  font-weight: 600;
  letter-spacing: 0.42em;
  text-transform: uppercase;
  color: rgb(var(--paper) / 0.42);
  white-space: nowrap;
}
.dm-rule { height: 1px; background: linear-gradient(90deg, rgb(var(--paper) / 0.26), rgb(var(--paper) / 0)); }

/* --- scrim over the render ----------------------------------------------- */
.dm-scrim {
  position: absolute; inset: 0;
  opacity: 0;
  transition: opacity .55s var(--ease-soft), backdrop-filter .55s var(--ease-soft);
  background:
    /* Directional key light for the type: heaviest under the wordmark and menu
       column, clearing towards the top right so the mountain still reads. */
    radial-gradient(115% 92% at 4% 96%, rgb(var(--ink) / 0.94) 0%, rgb(var(--ink) / 0.62) 34%, rgb(var(--ink) / 0) 70%),
    linear-gradient(100deg, rgb(var(--ink) / 0.62) 0%, rgb(var(--ink) / 0.16) 42%, rgb(var(--ink) / 0) 68%),
    linear-gradient(to top, rgb(var(--ink) / 0.8) 0%, rgb(var(--ink) / 0.12) 48%, rgb(var(--ink) / 0.38) 100%);
}
.dm-scrim::after {
  content: ''; position: absolute; inset: 0;
  background: radial-gradient(78% 62% at 50% 46%, rgb(var(--ink) / 0) 42%, rgb(var(--ink) / 0.6) 100%);
}
.dm-root[data-view="title"] .dm-scrim { opacity: 1; }
.dm-root[data-view="pause"] .dm-scrim,
.dm-root[data-view="settings"] .dm-scrim,
.dm-root[data-view="controls"] .dm-scrim {
  opacity: 1;
  backdrop-filter: blur(16px) saturate(0.78) brightness(0.66);
  -webkit-backdrop-filter: blur(16px) saturate(0.78) brightness(0.66);
}
.dm-root[data-view="summary"] .dm-scrim {
  opacity: 1;
  backdrop-filter: blur(7px) saturate(0.9) brightness(0.78);
  -webkit-backdrop-filter: blur(7px) saturate(0.9) brightness(0.78);
}

/* Fine grain over the scrim so large flat darks do not band. */
.dm-grain {
  position: absolute; inset: 0; opacity: 0; pointer-events: none;
  mix-blend-mode: overlay;
  transition: opacity .8s var(--ease-soft);
  background-repeat: repeat;
  background-size: 128px 128px;
}
.dm-root:not([data-view="none"]) .dm-grain { opacity: 0.5; }

/* --- view container ------------------------------------------------------ */
.dm-view { position: absolute; inset: 0; display: none; pointer-events: none; }
.dm-view.dm-live { display: block; }
.dm-view.dm-open { pointer-events: auto; }
.dm-view.dm-leaving { pointer-events: none; }

/* ========================================================================== */
/* TITLE                                                                      */
/* ========================================================================== */
.dm-title-wrap {
  position: absolute; inset: 0;
  display: flex; flex-direction: column;
  padding: var(--pad);
  padding-bottom: clamp(24px, 3.4vw, 60px);
}
.dm-title-top { display: flex; align-items: flex-start; justify-content: space-between; gap: 24px; }
.dm-mark { display: flex; align-items: center; gap: 12px; }
.dm-mark-glyph {
  width: 20px; height: 20px; flex: none;
  border: 1.5px solid rgb(var(--accent) / 0.9);
  border-radius: 3px;
  transform: rotate(45deg);
  position: relative;
}
.dm-mark-glyph::after {
  content: ''; position: absolute; inset: 4px;
  background: rgb(var(--accent) / 0.9);
  border-radius: 1px;
}
.dm-meta { text-align: right; display: grid; gap: 5px; }
.dm-meta span { display: block; font-size: 10px; letter-spacing: 0.26em; text-transform: uppercase; color: rgb(var(--paper) / 0.34); }

.dm-title-main { margin-top: auto; max-width: min(1080px, 82vw); }

.dm-wordmark {
  display: block;
  margin: 0 0 clamp(14px, 1.6vw, 26px);
  font-size: clamp(52px, 11.6vw, 178px);
  font-weight: 800;
  line-height: 0.86;
  letter-spacing: 0.255em;
  text-transform: uppercase;
  white-space: nowrap;
  filter: drop-shadow(0 22px 48px rgb(var(--ink) / 0.7));
}
.dm-wm-l {
  display: inline-block;
  background-image: linear-gradient(174deg,
    rgb(255 255 255) 4%,
    rgb(246 249 255) 40%,
    rgb(198 210 228) 82%,
    rgb(162 176 200) 100%);
  -webkit-background-clip: text; background-clip: text;
  color: transparent;
  opacity: 0;
  transform: translate3d(0, 0.34em, 0);
  animation: dm-letter 1.15s var(--ease) forwards;
  animation-delay: calc(140ms + var(--i) * 52ms);
}
@keyframes dm-letter {
  from { opacity: 0; transform: translate3d(0, 0.34em, 0); filter: blur(9px); }
  to   { opacity: 1; transform: translate3d(0, 0, 0); filter: blur(0); }
}

.dm-sub { display: flex; align-items: center; gap: 18px; margin-bottom: clamp(28px, 3.6vw, 58px); }
.dm-sub-rule {
  width: clamp(34px, 5vw, 78px); height: 1px; flex: none;
  background: rgb(var(--accent) / 0.85);
  transform-origin: left center;
  transform: scaleX(0);
  animation: dm-rule-in 1.1s var(--ease) 620ms forwards;
}
@keyframes dm-rule-in { to { transform: scaleX(1); } }
.dm-sub-text {
  font-size: clamp(10px, 0.86vw, 13px);
  font-weight: 500;
  letter-spacing: 0.34em;
  text-transform: uppercase;
  color: rgb(var(--paper) / 0.6);
}

.dm-fade {
  opacity: 0; transform: translate3d(0, 14px, 0);
  animation: dm-fade-up 1.05s var(--ease) forwards;
  animation-delay: calc(var(--i) * 90ms + 700ms);
}
@keyframes dm-fade-up { to { opacity: 1; transform: translate3d(0, 0, 0); } }

/* --- the menu list ------------------------------------------------------- */
.dm-list { display: flex; flex-direction: column; gap: 2px; margin: 0; padding: 0; list-style: none; }
.dm-item {
  position: relative;
  display: grid;
  grid-template-columns: clamp(34px, 3.4vw, 52px) minmax(0, 1fr);
  align-items: baseline;
  gap: 4px;
  padding: clamp(9px, 0.85vw, 14px) 0;
  width: min(580px, 66vw);
  text-align: left;
  transition: padding-left .5s var(--ease);
}
.dm-item::before {
  content: '';
  position: absolute; left: 0; top: 50%;
  width: clamp(14px, 1.5vw, 24px); height: 1px;
  transform: translateY(-50%) scaleX(0);
  transform-origin: left center;
  background: rgb(var(--accent));
  transition: transform .48s var(--ease);
}
.dm-item-idx {
  font-size: clamp(9px, 0.7vw, 11px);
  letter-spacing: 0.14em;
  color: rgb(var(--paper) / 0.26);
  transition: color .4s var(--ease-soft);
}
.dm-item-label {
  font-size: clamp(15px, 1.34vw, 22px);
  font-weight: 600;
  letter-spacing: 0.2em;
  text-transform: uppercase;
  color: rgb(var(--paper) / 0.62);
  transition: color .38s var(--ease-soft), letter-spacing .55s var(--ease), text-shadow .4s var(--ease-soft);
}
.dm-item-desc {
  grid-column: 2;
  margin-top: 0;
  max-width: 46ch;
  font-size: clamp(10px, 0.78vw, 12.5px);
  line-height: 1.5;
  letter-spacing: 0.055em;
  color: rgb(var(--paper) / 0.34);
  opacity: 0;
  max-height: 0;
  overflow: hidden;
  transform: translate3d(0, -4px, 0);
  transition: opacity .42s var(--ease-soft), max-height .5s var(--ease), transform .5s var(--ease), margin-top .5s var(--ease);
}
.dm-item.dm-on { padding-left: clamp(30px, 2.9vw, 44px); }
.dm-item.dm-on::before { transform: translateY(-50%) scaleX(1); }
.dm-item.dm-on .dm-item-idx { color: rgb(var(--accent) / 0.85); }
.dm-item.dm-on .dm-item-label {
  color: rgb(var(--paper));
  letter-spacing: 0.245em;
  text-shadow: 0 0 30px rgb(var(--accent) / 0.28);
}
.dm-item.dm-on .dm-item-desc { opacity: 1; max-height: 62px; margin-top: 5px; transform: none; }

.dm-title-foot {
  margin-top: clamp(30px, 4vw, 62px);
  display: flex; align-items: flex-end; justify-content: space-between; gap: 24px;
}
.dm-hint { display: flex; flex-wrap: wrap; gap: clamp(10px, 1.2vw, 22px); }
.dm-hint-i {
  display: inline-flex; align-items: center; gap: 7px;
  font-size: 10.5px; letter-spacing: 0.16em; text-transform: uppercase;
  color: rgb(var(--paper) / 0.32);
}

.dm-key {
  display: inline-flex; align-items: center; justify-content: center;
  min-width: 20px; height: 20px; padding: 0 6px;
  border-radius: 4px;
  border: 1px solid rgb(var(--paper) / 0.16);
  background: linear-gradient(180deg, rgb(var(--paper) / 0.1), rgb(var(--paper) / 0.02));
  box-shadow: inset 0 1px 0 rgb(var(--paper) / 0.1), 0 1px 2px rgb(var(--ink) / 0.5);
  font-family: ui-monospace, "SF Mono", SFMono-Regular, Menlo, Consolas, monospace;
  font-size: 10px; font-weight: 600; letter-spacing: 0.04em;
  color: rgb(var(--paper) / 0.78);
  white-space: nowrap;
}
.dm-key-sep { color: rgb(var(--paper) / 0.22); font-size: 10px; padding: 0 1px; }

/* ========================================================================== */
/* PANELS (settings / controls / summary)                                     */
/* ========================================================================== */
.dm-centre { position: absolute; inset: 0; display: flex; align-items: center; justify-content: center; padding: clamp(18px, 3vw, 54px); }
.dm-panel {
  position: relative;
  width: min(920px, 92vw);
  max-height: min(88vh, 900px);
  display: flex; flex-direction: column;
  padding: clamp(22px, 2.6vw, 44px);
  border-radius: 5px;
  border: 1px solid rgb(var(--paper) / 0.085);
  background:
    linear-gradient(180deg, rgb(var(--paper) / 0.045) 0%, rgb(var(--paper) / 0) 26%),
    rgb(var(--ink) / 0.68);
  backdrop-filter: blur(26px) saturate(1.25);
  -webkit-backdrop-filter: blur(26px) saturate(1.25);
  box-shadow:
    0 42px 120px -28px rgb(0 0 0 / 0.9),
    0 2px 0 -1px rgb(var(--paper) / 0.09) inset;
  opacity: 0;
  transform: translate3d(0, 22px, 0) scale(0.985);
}
.dm-view.dm-open .dm-panel { animation: dm-panel-in .72s var(--ease) forwards; }
.dm-view.dm-leaving .dm-panel { animation: dm-panel-out .26s var(--ease-soft) forwards; }
@keyframes dm-panel-in { to { opacity: 1; transform: none; } }
@keyframes dm-panel-out { from { opacity: 1; transform: none; } to { opacity: 0; transform: translate3d(0, 10px, 0) scale(0.99); } }

.dm-panel-head {
  display: flex; align-items: flex-start; justify-content: space-between; gap: 20px;
  padding-bottom: clamp(14px, 1.5vw, 22px);
  border-bottom: 1px solid rgb(var(--paper) / 0.08);
}
.dm-panel-title { margin: 7px 0 0; font-size: clamp(19px, 1.9vw, 30px); font-weight: 700; letter-spacing: 0.16em; text-transform: uppercase; line-height: 1; }
.dm-panel-body {
  overflow-y: auto; overscroll-behavior: contain;
  padding: clamp(14px, 1.6vw, 26px) 8px clamp(6px, 1vw, 14px) 2px;
  margin-right: -6px;
}
.dm-panel-body::-webkit-scrollbar { width: 4px; }
.dm-panel-body::-webkit-scrollbar-thumb { background: rgb(var(--paper) / 0.16); border-radius: 4px; }
.dm-panel-body::-webkit-scrollbar-track { background: transparent; }
.dm-panel-foot {
  display: flex; align-items: center; justify-content: space-between; gap: 18px; flex-wrap: wrap;
  padding-top: clamp(14px, 1.5vw, 22px);
  border-top: 1px solid rgb(var(--paper) / 0.08);
}

.dm-stagger { opacity: 0; transform: translate3d(0, 12px, 0); }
.dm-view.dm-open .dm-stagger { animation: dm-fade-up .78s var(--ease) forwards; animation-delay: calc(180ms + var(--i) * 42ms); }

/* --- buttons ------------------------------------------------------------- */
.dm-btns { display: flex; gap: 10px; flex-wrap: wrap; }
.dm-btn {
  position: relative;
  display: inline-flex; align-items: center; gap: 10px;
  padding: 11px 22px;
  border-radius: 3px;
  border: 1px solid rgb(var(--paper) / 0.16);
  font-size: 11.5px; font-weight: 650; letter-spacing: 0.2em; text-transform: uppercase;
  color: rgb(var(--paper) / 0.74);
  background: rgb(var(--paper) / 0.03);
  transition: color .3s var(--ease-soft), background .3s var(--ease-soft),
              border-color .3s var(--ease-soft), transform .45s var(--ease),
              box-shadow .4s var(--ease-soft);
}
.dm-btn.dm-on {
  color: rgb(var(--ink));
  background: rgb(var(--paper) / 0.92);
  border-color: rgb(var(--paper) / 0.92);
  transform: translateY(-1px);
  box-shadow: 0 10px 30px -10px rgb(var(--paper) / 0.36);
}
.dm-btn.dm-primary { border-color: rgb(var(--accent) / 0.55); color: rgb(var(--accent)); background: rgb(var(--accent) / 0.08); }
.dm-btn.dm-primary.dm-on { color: rgb(var(--ink)); background: rgb(var(--accent)); border-color: rgb(var(--accent)); box-shadow: 0 12px 34px -10px rgb(var(--accent) / 0.6); }
.dm-btn-key { opacity: 0.6; font-size: 9.5px; letter-spacing: 0.1em; }

/* --- settings rows ------------------------------------------------------- */
.dm-rows { display: flex; flex-direction: column; }
.dm-row {
  position: relative;
  display: grid;
  grid-template-columns: minmax(0, 1fr) minmax(190px, 44%);
  align-items: center;
  gap: clamp(14px, 2vw, 40px);
  width: 100%;
  padding: clamp(11px, 1.15vw, 16px) 14px clamp(11px, 1.15vw, 16px) 16px;
  text-align: left;
  border-radius: 3px;
  border-bottom: 1px solid rgb(var(--paper) / 0.055);
  transition: background .34s var(--ease-soft);
}
.dm-row::before {
  content: ''; position: absolute; left: 0; top: 14%; bottom: 14%;
  width: 2px; border-radius: 2px;
  background: rgb(var(--accent));
  transform: scaleY(0); transform-origin: center;
  transition: transform .42s var(--ease);
}
.dm-row.dm-on { background: rgb(var(--paper) / 0.045); }
.dm-row.dm-on::before { transform: scaleY(1); }
.dm-row-label { font-size: clamp(11.5px, 1.02vw, 14px); font-weight: 600; letter-spacing: 0.13em; text-transform: uppercase; color: rgb(var(--paper) / 0.86); }
.dm-row-note { margin-top: 4px; font-size: clamp(10px, 0.76vw, 11.5px); line-height: 1.45; letter-spacing: 0.04em; color: rgb(var(--paper) / 0.36); }
.dm-row-ctl { display: flex; align-items: center; justify-content: flex-end; gap: 10px; }

/* segmented */
.dm-seg { display: flex; border: 1px solid rgb(var(--paper) / 0.12); border-radius: 3px; overflow: hidden; background: rgb(var(--ink) / 0.4); }
.dm-seg-o {
  padding: 7px clamp(8px, 1vw, 15px);
  font-size: 10px; font-weight: 600; letter-spacing: 0.14em; text-transform: uppercase;
  color: rgb(var(--paper) / 0.44);
  transition: color .28s var(--ease-soft), background .28s var(--ease-soft);
  white-space: nowrap;
}
.dm-seg-o + .dm-seg-o { border-left: 1px solid rgb(var(--paper) / 0.09); }
.dm-seg-o:hover { color: rgb(var(--paper) / 0.8); }
.dm-seg-o.dm-sel { color: rgb(var(--ink)); background: rgb(var(--paper) / 0.88); }
.dm-row.dm-on .dm-seg-o.dm-sel { background: rgb(var(--accent)); }

/* toggle */
.dm-tog {
  position: relative; width: 46px; height: 24px; flex: none;
  border-radius: 999px; border: 1px solid rgb(var(--paper) / 0.16);
  background: rgb(var(--ink) / 0.55);
  transition: background .34s var(--ease-soft), border-color .34s var(--ease-soft);
}
.dm-tog::after {
  content: ''; position: absolute; top: 50%; left: 3px;
  width: 16px; height: 16px; margin-top: -8px;
  border-radius: 999px; background: rgb(var(--paper) / 0.6);
  transition: transform .44s var(--ease), background .34s var(--ease-soft);
}
.dm-tog.dm-sel { background: rgb(var(--accent) / 0.28); border-color: rgb(var(--accent) / 0.7); }
.dm-tog.dm-sel::after { transform: translateX(22px); background: rgb(var(--accent)); }
.dm-tog-txt { font-size: 10px; letter-spacing: 0.18em; text-transform: uppercase; color: rgb(var(--paper) / 0.42); min-width: 26px; text-align: right; }

/* slider */
.dm-sl { position: relative; flex: 1; max-width: 230px; height: 22px; display: flex; align-items: center; cursor: ew-resize; touch-action: none; }
.dm-sl-track { position: absolute; left: 0; right: 0; height: 2px; border-radius: 2px; background: rgb(var(--paper) / 0.14); overflow: hidden; }
.dm-sl-fill { position: absolute; inset: 0 auto 0 0; background: rgb(var(--paper) / 0.62); transition: background .3s var(--ease-soft); }
.dm-row.dm-on .dm-sl-fill { background: rgb(var(--accent)); }
.dm-sl-knob { position: absolute; top: 50%; width: 11px; height: 11px; margin: -5.5px 0 0 -5.5px; border-radius: 999px; background: rgb(var(--paper)); box-shadow: 0 2px 8px rgb(var(--ink) / 0.8); transition: transform .3s var(--ease); }
.dm-row.dm-on .dm-sl-knob { transform: scale(1.22); }
.dm-sl-val { min-width: 44px; text-align: right; font-size: 12px; color: rgb(var(--paper) / 0.7); }

/* ========================================================================== */
/* CONTROLS                                                                   */
/* ========================================================================== */
.dm-ctl-grp + .dm-ctl-grp { margin-top: clamp(18px, 2vw, 30px); }
.dm-ctl-grp-h { display: flex; align-items: center; gap: 14px; margin-bottom: 10px; }
.dm-ctl-grp-h .dm-rule { flex: 1; }
.dm-ctl-head, .dm-ctl-row {
  display: grid;
  grid-template-columns: minmax(0, 1.25fr) minmax(130px, 0.95fr) minmax(100px, 0.8fr);
  gap: clamp(10px, 1.5vw, 26px);
  align-items: center;
}
.dm-ctl-head { padding: 4px 8px 8px; border-bottom: 1px solid rgb(var(--paper) / 0.07); }
.dm-ctl-head span { font-size: 9px; letter-spacing: 0.28em; text-transform: uppercase; color: rgb(var(--paper) / 0.3); }
.dm-ctl-row { padding: 9px 8px; border-bottom: 1px solid rgb(var(--paper) / 0.045); }
.dm-ctl-row:hover { background: rgb(var(--paper) / 0.03); }
.dm-ctl-a { font-size: clamp(11.5px, 1vw, 13.5px); font-weight: 600; letter-spacing: 0.1em; color: rgb(var(--paper) / 0.9); }
.dm-ctl-n { margin-top: 3px; font-size: 10.5px; line-height: 1.4; letter-spacing: 0.03em; color: rgb(var(--paper) / 0.33); }
.dm-ctl-k { display: flex; align-items: center; flex-wrap: wrap; gap: 5px; }
.dm-ctl-or { font-size: 9px; letter-spacing: 0.16em; text-transform: uppercase; color: rgb(var(--paper) / 0.24); padding: 0 3px; }
.dm-ctl-p { font-size: 11px; letter-spacing: 0.08em; color: rgb(var(--paper) / 0.52); }
.dm-ctl-src { font-size: 10px; letter-spacing: 0.16em; text-transform: uppercase; color: rgb(var(--paper) / 0.26); text-align: right; }

/* ========================================================================== */
/* PAUSE                                                                      */
/* ========================================================================== */
.dm-pause-wrap { position: absolute; inset: 0; display: flex; flex-direction: column; justify-content: center; padding: var(--pad); }
.dm-pause-inner { opacity: 0; transform: translate3d(0, 16px, 0); max-width: min(900px, 86vw); }
.dm-view.dm-open .dm-pause-inner { animation: dm-fade-up .68s var(--ease) 60ms forwards; }
.dm-pause-h { font-size: clamp(30px, 4.6vw, 66px); font-weight: 800; letter-spacing: 0.3em; text-transform: uppercase; line-height: 1; margin: 12px 0 clamp(10px, 1.4vw, 22px); }
.dm-pause-meta { display: flex; gap: clamp(18px, 2.4vw, 44px); margin-bottom: clamp(24px, 3vw, 46px); flex-wrap: wrap; }
.dm-pause-meta-i { display: grid; gap: 7px; }
.dm-pause-meta-v { font-size: clamp(14px, 1.4vw, 20px); font-weight: 600; color: rgb(var(--paper) / 0.9); }

/* ========================================================================== */
/* SUMMARY                                                                    */
/* ========================================================================== */
.dm-sum { width: min(1020px, 94vw); }
.dm-sum.dm-pb {
  border-color: rgb(var(--gold) / 0.34);
  box-shadow:
    0 42px 130px -30px rgb(0 0 0 / 0.92),
    0 0 90px -40px rgb(var(--gold) / 0.55),
    0 2px 0 -1px rgb(var(--gold) / 0.22) inset;
}
.dm-sum-eyebrow { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }
.dm-pb-badge {
  position: relative; overflow: hidden;
  display: inline-flex; align-items: center; gap: 8px;
  padding: 5px 12px 5px 10px;
  border-radius: 2px;
  border: 1px solid rgb(var(--gold) / 0.5);
  background: rgb(var(--gold) / 0.1);
  font-size: 9.5px; font-weight: 700; letter-spacing: 0.3em; text-transform: uppercase;
  color: rgb(var(--gold));
}
.dm-pb-badge::after {
  content: ''; position: absolute; top: 0; bottom: 0; width: 55%;
  background: linear-gradient(100deg, transparent, rgb(255 255 255 / 0.42), transparent);
  transform: translateX(-160%);
  animation: dm-sweep 2.4s var(--ease-soft) 700ms 2;
}
@keyframes dm-sweep { to { transform: translateX(340%); } }
.dm-pb-dot { width: 5px; height: 5px; border-radius: 999px; background: rgb(var(--gold)); box-shadow: 0 0 12px rgb(var(--gold)); }

.dm-sum-time { display: flex; align-items: baseline; gap: clamp(12px, 1.6vw, 26px); flex-wrap: wrap; padding: clamp(14px, 1.6vw, 24px) 0 clamp(4px, 0.6vw, 10px); }
.dm-sum-time-v { font-size: clamp(38px, 5.4vw, 78px); font-weight: 700; letter-spacing: -0.02em; line-height: 0.92; }
.dm-sum.dm-pb .dm-sum-time-v { color: rgb(var(--gold)); text-shadow: 0 0 60px rgb(var(--gold) / 0.34); }
.dm-delta { font-size: clamp(13px, 1.3vw, 18px); font-weight: 600; }
.dm-up { color: rgb(var(--good)); }
.dm-down { color: rgb(var(--bad)); }
.dm-flat { color: rgb(var(--paper) / 0.4); }
.dm-delta-note { margin-top: 5px; font-size: 10px; letter-spacing: 0.24em; text-transform: uppercase; color: rgb(var(--paper) / 0.32); }

.dm-sum-grid { display: grid; grid-template-columns: minmax(0, 1.32fr) minmax(0, 1fr); gap: clamp(20px, 2.6vw, 46px); align-items: start; }
@media (max-width: 860px) { .dm-sum-grid { grid-template-columns: minmax(0, 1fr); } }

.dm-splits { width: 100%; min-width: 0; }
.dm-split-head, .dm-split-row {
  display: grid;
  grid-template-columns: 26px minmax(0, 1fr) 84px 78px;
  gap: 10px; align-items: center;
}
.dm-split-head { padding: 0 6px 8px; border-bottom: 1px solid rgb(var(--paper) / 0.08); }
.dm-split-head span { font-size: 9px; letter-spacing: 0.26em; text-transform: uppercase; color: rgb(var(--paper) / 0.28); }
.dm-split-head span:nth-child(n+3) { text-align: right; }
.dm-split-row {
  padding: 8px 6px;
  border-bottom: 1px solid rgb(var(--paper) / 0.04);
  opacity: 0; transform: translate3d(0, 10px, 0);
}
.dm-view.dm-open .dm-split-row { animation: dm-fade-up .62s var(--ease) forwards; animation-delay: calc(300ms + var(--i) * 46ms); }
.dm-split-row.dm-final { border-bottom: 0; margin-top: 4px; border-top: 1px solid rgb(var(--paper) / 0.14); padding-top: 12px; }
.dm-split-i { font-size: 10px; color: rgb(var(--paper) / 0.28); }
.dm-split-n { font-size: 11.5px; letter-spacing: 0.1em; text-transform: uppercase; color: rgb(var(--paper) / 0.72); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.dm-split-row.dm-final .dm-split-n { color: rgb(var(--paper)); font-weight: 600; }
.dm-split-t { text-align: right; font-size: 12.5px; color: rgb(var(--paper) / 0.92); }
.dm-split-d { text-align: right; font-size: 11.5px; }

.dm-stats { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 1px; background: rgb(var(--paper) / 0.06); border: 1px solid rgb(var(--paper) / 0.06); border-radius: 3px; overflow: hidden; }
.dm-stat { padding: clamp(12px, 1.3vw, 18px); background: rgb(var(--ink) / 0.5); display: grid; gap: 7px; align-content: start; }
.dm-stat-v { font-size: clamp(19px, 2.1vw, 30px); font-weight: 650; line-height: 1; color: rgb(var(--paper) / 0.95); }
.dm-stat-u { font-size: 0.48em; font-weight: 500; letter-spacing: 0.1em; color: rgb(var(--paper) / 0.4); margin-left: 4px; }
.dm-stat-sub { font-size: 10px; letter-spacing: 0.05em; line-height: 1.35; color: rgb(var(--paper) / 0.3); }
.dm-stat.dm-hot .dm-stat-v { color: rgb(var(--accent)); }

/* --- short viewports ------------------------------------------------------
   vw-based type ramps say nothing about height, and a 1024x640 window is exactly
   where a nine-row split table stops fitting. Tighten the vertical rhythm rather
   than let the body scroll. */
@media (max-height: 760px) {
  .dm-panel { padding: clamp(16px, 2vw, 30px); max-height: 94vh; }
  .dm-panel-body { padding-top: clamp(10px, 1.2vw, 18px); }
  .dm-panel-head { padding-bottom: 12px; }
  .dm-panel-foot { padding-top: 12px; }
  .dm-sum-time { padding: 12px 0 2px; }
  .dm-sum-time-v { font-size: clamp(30px, 4.6vw, 56px); }
  .dm-split-row { padding-top: 6px; padding-bottom: 6px; }
  .dm-row { padding-top: 9px; padding-bottom: 9px; }
  .dm-row-note { margin-top: 2px; }
  .dm-stat { padding: 11px 12px; gap: 5px; }
  .dm-wordmark { font-size: clamp(46px, 9.4vw, 132px); }
  .dm-sub { margin-bottom: clamp(18px, 2.4vw, 34px); }
  .dm-title-foot { margin-top: clamp(18px, 2.4vw, 34px); }
  .dm-item { padding-top: 8px; padding-bottom: 8px; }
}

/* --- reduced motion ------------------------------------------------------ */
@media (prefers-reduced-motion: reduce) {
  .dm-root *, .dm-root *::before, .dm-root *::after {
    animation-duration: 0.001ms !important;
    animation-delay: 0ms !important;
    transition-duration: 0.001ms !important;
  }
  .dm-wm-l, .dm-fade, .dm-stagger, .dm-split-row, .dm-pause-inner, .dm-panel {
    opacity: 1; transform: none; filter: none;
  }
}
`;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
function el(tag, cls, text) {
  const node = document.createElement(tag);
  if (cls) node.className = cls;
  if (text != null) node.textContent = text;
  return node;
}

function clear(node) { while (node.firstChild) node.removeChild(node.firstChild); }

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

/** m:ss.mmm, dropping the minutes field under a minute (a race clock, not a stopwatch). */
function fmtTime(s) {
  if (typeof s !== 'number' || !isFinite(s) || s < 0) return '—:——.———';
  const m = Math.floor(s / 60);
  const rem = s - m * 60;
  const r = rem.toFixed(3);
  return m > 0 ? `${m}:${rem < 10 ? '0' : ''}${r}` : r;
}

/** Signed delta with a true minus sign (U+2212) so + and − share a glyph width. */
function fmtDelta(d) {
  if (typeof d !== 'number' || !isFinite(d)) return '—';
  if (Math.abs(d) < 0.0005) return '±0.000';
  return (d > 0 ? '+' : '−') + Math.abs(d).toFixed(3);
}

function deltaClass(d) {
  if (typeof d !== 'number' || !isFinite(d)) return 'dm-flat';
  if (d < -0.0005) return 'dm-up';
  if (d > 0.0005) return 'dm-down';
  return 'dm-flat';
}

/** 128×128 tiling luminance noise, drawn once — no downloaded texture. */
function makeGrainDataURL() {
  try {
    const c = document.createElement('canvas');
    c.width = 128; c.height = 128;
    const g = c.getContext('2d');
    if (!g) return '';
    const img = g.createImageData(128, 128);
    const d = img.data;
    for (let i = 0; i < d.length; i += 4) {
      // Low-amplitude noise around mid grey; `overlay` blending turns it into grain.
      const n = 118 + ((Math.random() * 22) | 0);
      d[i] = n; d[i + 1] = n; d[i + 2] = n; d[i + 3] = 26;
    }
    g.putImageData(img, 0, 0);
    return c.toDataURL('image/png');
  } catch (e) {
    return '';
  }
}

// ---------------------------------------------------------------------------
export function createMenu(ctx) {
  const reduced = (() => {
    try { return window.matchMedia('(prefers-reduced-motion: reduce)').matches; }
    catch (e) { return false; }
  })();

  // -------------------------------------------------------------------------
  // Persisted settings
  // -------------------------------------------------------------------------
  const settings = {
    quality: (ctx && ctx.quality) || 'high',
    camera: 'chase',
    invertLook: false,
    fov: 62,
    volume: 0.8,
    photoMode: false,
  };
  try {
    const raw = localStorage.getItem(SETTINGS_KEY);
    if (raw) {
      const saved = JSON.parse(raw);
      if (saved && typeof saved === 'object') {
        if (QUALITY_OPTIONS.some((o) => o.value === saved.quality)) settings.quality = saved.quality;
        if (CAMERA_OPTIONS.some((o) => o.value === saved.camera)) settings.camera = saved.camera;
        if (typeof saved.invertLook === 'boolean') settings.invertLook = saved.invertLook;
        if (typeof saved.fov === 'number') settings.fov = clamp(saved.fov, 55, 95);
        if (typeof saved.volume === 'number') settings.volume = clamp(saved.volume, 0, 1);
      }
    }
  } catch (e) { /* private mode / storage disabled — defaults are fine */ }

  function saveSettings() {
    try {
      localStorage.setItem(SETTINGS_KEY, JSON.stringify({
        quality: settings.quality, camera: settings.camera,
        invertLook: settings.invertLook, fov: settings.fov, volume: settings.volume,
      }));
    } catch (e) { /* ignore */ }
  }

  function emit(name, payload) {
    if (ctx && ctx.events && typeof ctx.events.emit === 'function') ctx.events.emit(name, payload);
  }

  function applySetting(key, value, opts) {
    const persist = !opts || opts.persist !== false;
    settings[key] = value;
    switch (key) {
      case 'quality':
        if (ctx && typeof ctx.setQuality === 'function') ctx.setQuality(value);
        break;
      case 'camera':
        if (ctx && ctx.chaseCamera && typeof ctx.chaseCamera.setMode === 'function') ctx.chaseCamera.setMode(value);
        break;
      case 'invertLook':
        if (ctx && ctx.settings) ctx.settings.invertLook = value;
        break;
      case 'fov':
        if (ctx && ctx.settings) { ctx.settings.fov = value; ctx.settings.fovBase = value; }
        // Immediate feedback if nothing else owns the FOV this frame.
        if (ctx && ctx.camera && typeof ctx.camera.fov === 'number' &&
            !(ctx.chaseCamera && typeof ctx.chaseCamera.setMode === 'function')) {
          ctx.camera.fov = value;
          if (typeof ctx.camera.updateProjectionMatrix === 'function') ctx.camera.updateProjectionMatrix();
        }
        break;
      case 'volume':
        if (ctx && ctx.settings) ctx.settings.masterVolume = value;
        if (ctx && ctx.audio) {
          if (typeof ctx.audio.setMasterVolume === 'function') ctx.audio.setMasterVolume(value);
          else if (typeof ctx.audio.setVolume === 'function') ctx.audio.setVolume(value);
        }
        emit('audio:volume', value);
        break;
      case 'photoMode':
        if (ctx && ctx.settings) ctx.settings.photoMode = value;
        if (ctx && ctx.hud) {
          if (typeof ctx.hud.setVisible === 'function') ctx.hud.setVisible(!value);
          else if (value && typeof ctx.hud.hide === 'function') ctx.hud.hide();
          else if (!value && typeof ctx.hud.show === 'function') ctx.hud.show();
        }
        emit('photo:mode', { enabled: value });
        break;
      default: break;
    }
    emit('settings:changed', { key, value });
    if (persist && key !== 'photoMode') saveSettings();
  }

  // Photo mode closes whatever was open and restores it on the way out, so
  // ducking into a shot from the title screen does not dump you into the game.
  let photoRestoreView = 'none';
  function setPhotoMode(on) {
    const want = !!on;
    if (want === !!settings.photoMode) return;
    if (want) {
      photoRestoreView = view;
      applySetting('photoMode', true, { persist: false });
      setView('none');
    } else {
      applySetting('photoMode', false, { persist: false });
      const restore = photoRestoreView;
      photoRestoreView = 'none';
      if (restore && restore !== 'none') setView(restore);
    }
  }

  // -------------------------------------------------------------------------
  // Root DOM + stylesheet
  // -------------------------------------------------------------------------
  const styleEl = el('style');
  styleEl.id = 'descent-menu-style';
  const grainURL = makeGrainDataURL();
  styleEl.textContent = CSS + (grainURL ? `\n.dm-grain { background-image: url("${grainURL}"); }\n` : '');
  document.head.appendChild(styleEl);

  const root = el('div', 'dm-root');
  root.id = 'descent-menu';
  root.dataset.view = 'none';
  root.setAttribute('role', 'presentation');

  const scrim = el('div', 'dm-scrim');
  const grain = el('div', 'dm-grain');
  root.appendChild(scrim);
  root.appendChild(grain);

  const host = (ctx && ctx.container) || document.body;
  host.appendChild(root);

  // -------------------------------------------------------------------------
  // Navigation model.
  // A "view" is { name, el, items[], onEnter, onExit }. Items expose activate()
  // and optionally adjust(dir) for left/right.
  // -------------------------------------------------------------------------
  const views = new Map();
  let view = 'none';
  let prevView = 'none';
  let leaveTimer = 0;
  let focusIndex = 0;

  function currentView() { return views.get(view) || null; }

  function registerView(name, node) {
    node.classList.add('dm-view');
    node.dataset.dmView = name;
    node.setAttribute('aria-hidden', 'true');
    root.appendChild(node);
    const v = { name, el: node, items: [], onEnter: null, onExit: null, lastFocus: 0 };
    views.set(name, v);
    return v;
  }

  function setFocus(i, opts) {
    const scroll = !(opts && opts.scroll === false);
    const v = currentView();
    if (!v || !v.items.length) return;
    const n = v.items.length;
    let idx = ((i % n) + n) % n;
    // Skip disabled entries in the direction of travel.
    let guard = 0;
    while (v.items[idx] && v.items[idx].disabled && guard++ < n) idx = (idx + 1) % n;
    focusIndex = idx;
    v.lastFocus = idx;
    for (let k = 0; k < n; k++) {
      const it = v.items[k];
      if (!it || !it.el) continue;
      const on = k === idx;
      it.el.classList.toggle('dm-on', on);
      it.el.tabIndex = on ? 0 : -1;
      if (on) {
        try { it.el.focus({ preventScroll: !scroll }); }
        catch (e) { try { it.el.focus(); } catch (e2) { /* ignore */ } }
      }
    }
  }

  function moveFocus(dir) {
    const v = currentView();
    if (!v || !v.items.length) return;
    setFocus(focusIndex + dir);
  }

  function activateFocused() {
    const v = currentView();
    if (!v) return;
    const it = v.items[focusIndex];
    if (it && !it.disabled && typeof it.activate === 'function') it.activate();
  }

  function adjustFocused(dir) {
    const v = currentView();
    if (!v) return;
    const it = v.items[focusIndex];
    if (it && !it.disabled && typeof it.adjust === 'function') it.adjust(dir);
  }

  function setView(name, opts) {
    const focus = opts && opts.focus !== undefined ? opts.focus : 0;
    const remember = !(opts && opts.remember === false);
    if (view === name) return;

    const old = views.get(view);
    if (old) {
      if (typeof old.onExit === 'function') old.onExit();
      old.el.classList.remove('dm-open');
      old.el.classList.add('dm-leaving');
      old.el.setAttribute('aria-hidden', 'true');
      const dying = old;
      clearTimeout(leaveTimer);
      leaveTimer = setTimeout(() => {
        dying.el.classList.remove('dm-leaving', 'dm-live');
      }, reduced ? 0 : 260);
    }
    if (remember) prevView = view;
    view = name;
    root.dataset.view = name;

    const next = views.get(name);
    if (!next) {
      if (document.activeElement && root.contains(document.activeElement)) document.activeElement.blur();
      return;
    }
    // Force a reflow between removing and re-adding the animation classes so a
    // re-opened panel replays its entrance instead of snapping in.
    next.el.classList.remove('dm-leaving');
    next.el.classList.add('dm-live');
    void next.el.offsetWidth;
    next.el.classList.add('dm-open');
    next.el.setAttribute('aria-hidden', 'false');
    if (typeof next.onEnter === 'function') next.onEnter();
    focusIndex = 0;
    setFocus(focus === 'last' ? next.lastFocus : (focus | 0), { scroll: false });
  }

  function isBlocking() { return view !== 'none'; }

  // -------------------------------------------------------------------------
  // Game-side actions
  // -------------------------------------------------------------------------
  function gp() { return ctx && ctx.gameplay ? ctx.gameplay : null; }

  function setCameraMode(mode) {
    if (ctx && ctx.chaseCamera && typeof ctx.chaseCamera.setMode === 'function') ctx.chaseCamera.setMode(mode);
  }

  function enterMenuState() {
    const g = gp();
    if (!g) return;
    if (typeof g.toMenu === 'function') { try { g.toMenu(); } catch (e) { /* ignore */ } return; }
    try { g.state = 'menu'; } catch (e) { /* read-only implementation — fine */ }
  }

  function beginRun(mode) {
    const g = gp();
    resetTelemetry();
    emit('run:mode', { mode });
    if (g) {
      try { g.mode = mode; g.freeRide = (mode === 'free'); } catch (e) { /* ignore */ }
      if (typeof g.restart === 'function') {
        try { g.restart({ mode }); } catch (e) { try { g.restart(); } catch (e2) { /* ignore */ } }
      } else if (typeof g.start === 'function') {
        try { g.start({ mode }); } catch (e) { try { g.start(); } catch (e2) { /* ignore */ } }
      }
    } else if (ctx && ctx.bike && typeof ctx.bike.reset === 'function') {
      try { ctx.bike.reset(ctx.trail && ctx.trail.startTransform); } catch (e) { /* ignore */ }
    }
    setCameraMode(settings.camera === 'cinematic' ? 'chase' : settings.camera);
    wantPaused = false;
    setView('none');
  }

  function resumeRun() {
    const g = gp();
    wantPaused = false;
    if (g && g.state === 'paused') {
      if (typeof g.resume === 'function') { try { g.resume(); } catch (e) { /* ignore */ } }
      else if (typeof g.pause === 'function') { try { g.pause(); } catch (e) { /* ignore */ } }
    }
    setView('none');
  }

  function quitToTitle() {
    wantPaused = false;
    enterMenuState();
    setCameraMode('cinematic');
    showTitle();
  }

  // -------------------------------------------------------------------------
  // TITLE
  // -------------------------------------------------------------------------
  const titleView = registerView('title', el('div'));
  let seedLine = null;
  {
    const wrap = el('div', 'dm-title-wrap');

    // top bar --------------------------------------------------------------
    const top = el('div', 'dm-title-top');
    const mark = el('div', 'dm-mark dm-fade');
    mark.style.setProperty('--i', '0');
    mark.appendChild(el('div', 'dm-mark-glyph'));
    mark.appendChild(el('div', 'dm-eyebrow', 'Descent'));
    const meta = el('div', 'dm-meta dm-fade');
    meta.style.setProperty('--i', '0');
    seedLine = el('span', 'dm-num');
    meta.appendChild(seedLine);
    meta.appendChild(el('span', null, 'Alpine massif · golden hour'));
    top.appendChild(mark);
    top.appendChild(meta);

    // wordmark -------------------------------------------------------------
    const main = el('div', 'dm-title-main');
    const wm = el('h1', 'dm-wordmark');
    wm.setAttribute('aria-label', 'Descent');
    const WORD = 'DESCENT';
    for (let i = 0; i < WORD.length; i++) {
      const s = el('span', 'dm-wm-l', WORD[i]);
      s.style.setProperty('--i', String(i));
      s.setAttribute('aria-hidden', 'true');
      wm.appendChild(s);
    }
    main.appendChild(wm);

    const sub = el('div', 'dm-sub');
    sub.appendChild(el('div', 'dm-sub-rule'));
    const subText = el('div', 'dm-sub-text dm-fade', 'Two point six kilometres · four hundred metres of fall line');
    subText.style.setProperty('--i', '1');
    sub.appendChild(subText);
    main.appendChild(sub);

    // menu list ------------------------------------------------------------
    const list = el('ul', 'dm-list');
    list.setAttribute('role', 'menu');
    const TITLE_ITEMS = [
      { label: 'Start Run', desc: 'Timed descent. Eight splits, one clock, your personal best on the line.', run: () => beginRun('timed') },
      { label: 'Free Ride', desc: 'No clock, no gates. Session the jump line until it is dialled.', run: () => beginRun('free') },
      { label: 'Settings', desc: 'Quality, camera, field of view, volume, photo mode.', run: () => setView('settings') },
      { label: 'Controls', desc: 'Keyboard and gamepad reference.', run: () => setView('controls') },
    ];
    TITLE_ITEMS.forEach((entry, i) => {
      const li = el('li');
      li.setAttribute('role', 'none');
      const btn = el('button', 'dm-item dm-fade');
      btn.type = 'button';
      btn.setAttribute('role', 'menuitem');
      btn.style.setProperty('--i', String(i + 2));
      btn.tabIndex = -1;
      btn.appendChild(el('span', 'dm-item-idx dm-num', String(i + 1).padStart(2, '0')));
      btn.appendChild(el('span', 'dm-item-label', entry.label));
      btn.appendChild(el('span', 'dm-item-desc', entry.desc));
      btn.addEventListener('click', entry.run);
      btn.addEventListener('pointerenter', () => setFocus(i, { scroll: false }));
      li.appendChild(btn);
      list.appendChild(li);
      titleView.items.push({ el: btn, activate: entry.run });
    });
    main.appendChild(list);

    // foot -----------------------------------------------------------------
    const foot = el('div', 'dm-title-foot dm-fade');
    foot.style.setProperty('--i', '6');
    const hint = el('div', 'dm-hint');
    const hintSpecs = [
      { keys: ['↑', '↓'], text: 'Navigate' },
      { keys: ['⏎'], text: 'Select' },
      { keys: ['Esc'], text: 'Back' },
    ];
    for (const h of hintSpecs) {
      const i = el('span', 'dm-hint-i');
      for (const k of h.keys) i.appendChild(el('kbd', 'dm-key', k));
      i.appendChild(document.createTextNode(h.text));
      hint.appendChild(i);
    }
    const padHint = el('span', 'dm-hint-i');
    padHint.appendChild(el('kbd', 'dm-key', 'Pad'));
    padHint.appendChild(document.createTextNode('Supported'));
    hint.appendChild(padHint);
    foot.appendChild(hint);
    foot.appendChild(el('div', 'dm-eyebrow', 'Pre-release build'));

    wrap.appendChild(top);
    wrap.appendChild(main);
    wrap.appendChild(foot);
    titleView.el.appendChild(wrap);

    titleView.onEnter = () => {
      setCameraMode('cinematic');
      seedLine.textContent = `Seed ${(ctx && ctx.seed != null) ? ctx.seed : 0}`;
      // Replay the intro sequence every time the title is shown.
      if (!reduced) {
        const animated = titleView.el.querySelectorAll('.dm-wm-l, .dm-fade, .dm-sub-rule');
        for (let i = 0; i < animated.length; i++) {
          const n = animated[i];
          n.style.animation = 'none';
          void n.offsetWidth;
          n.style.animation = '';
        }
      }
    };
  }

  function showTitle() { setView('title'); }

  // -------------------------------------------------------------------------
  // Reusable panel scaffold
  // -------------------------------------------------------------------------
  function buildPanel(v, opts) {
    const centre = el('div', 'dm-centre');
    const panel = el('div', 'dm-panel' + (opts.wide ? ' dm-sum' : ''));
    panel.setAttribute('role', 'dialog');
    panel.setAttribute('aria-modal', 'true');
    panel.setAttribute('aria-label', opts.title);

    const head = el('div', 'dm-panel-head');
    const headL = el('div');
    const eyeRow = el('div', 'dm-sum-eyebrow');
    const eyeEl = el('div', 'dm-eyebrow', opts.eyebrow);
    eyeRow.appendChild(eyeEl);
    headL.appendChild(eyeRow);
    const titleEl = el('h2', 'dm-panel-title', opts.title);
    headL.appendChild(titleEl);
    const headR = el('div');
    head.appendChild(headL);
    head.appendChild(headR);

    const body = el('div', 'dm-panel-body');
    const foot = el('div', 'dm-panel-foot');

    panel.appendChild(head);
    panel.appendChild(body);
    panel.appendChild(foot);
    centre.appendChild(panel);
    v.el.appendChild(centre);
    return { panel, head, headL, headR, eyeRow, eyeEl, titleEl, body, foot };
  }

  function makeButton(v, label, onClick, opts) {
    const o = opts || {};
    const b = el('button', 'dm-btn' + (o.primary ? ' dm-primary' : ''));
    b.type = 'button';
    b.tabIndex = -1;
    b.appendChild(document.createTextNode(label));
    if (o.keyHint) b.appendChild(el('span', 'dm-btn-key', o.keyHint));
    b.addEventListener('click', onClick);
    b.addEventListener('pointerenter', () => {
      for (let i = 0; i < v.items.length; i++) {
        if (v.items[i].el === b) { setFocus(i, { scroll: false }); break; }
      }
    });
    v.items.push({ el: b, activate: onClick });
    return b;
  }

  // -------------------------------------------------------------------------
  // SETTINGS
  // -------------------------------------------------------------------------
  const settingsView = registerView('settings', el('div'));
  const settingsSync = [];   // functions that push model → DOM
  {
    const P = buildPanel(settingsView, { eyebrow: 'Configuration', title: 'Settings' });
    const rows = el('div', 'dm-rows');
    P.body.appendChild(rows);

    let rowIndex = 0;
    function addRow(label, note, buildCtl) {
      const row = el('button', 'dm-row dm-stagger');
      row.type = 'button';
      row.tabIndex = -1;
      row.style.setProperty('--i', String(rowIndex++));
      const left = el('div');
      left.appendChild(el('div', 'dm-row-label', label));
      if (note) left.appendChild(el('div', 'dm-row-note', note));
      const ctl = el('div', 'dm-row-ctl');
      row.appendChild(left);
      row.appendChild(ctl);
      rows.appendChild(row);
      const nav = { el: row };
      settingsView.items.push(nav);
      row.addEventListener('pointerenter', () => {
        for (let i = 0; i < settingsView.items.length; i++) {
          if (settingsView.items[i].el === row) { setFocus(i, { scroll: false }); break; }
        }
      });
      buildCtl(ctl, row, nav);
      return row;
    }

    // -- segmented control -------------------------------------------------
    function addSegmented(label, note, options, get, set) {
      addRow(label, note, (ctl, row, nav) => {
        const seg = el('div', 'dm-seg');
        const btns = [];
        const sync = () => {
          const cur = get();
          for (let i = 0; i < options.length; i++) btns[i].classList.toggle('dm-sel', options[i].value === cur);
        };
        options.forEach((opt) => {
          const b = el('button', 'dm-seg-o', opt.label);
          b.type = 'button';
          b.tabIndex = -1;
          b.addEventListener('click', (e) => { e.stopPropagation(); set(opt.value); sync(); });
          seg.appendChild(b);
          btns.push(b);
        });
        ctl.appendChild(seg);
        settingsSync.push(sync);
        nav.adjust = (dir) => {
          const n = options.length;
          let cur = -1;
          for (let i = 0; i < n; i++) if (options[i].value === get()) { cur = i; break; }
          const next = ((cur < 0 ? 0 : cur) + dir + n) % n;
          set(options[next].value);
          sync();
        };
        nav.activate = () => nav.adjust(1);
        // Clicking the row body (rather than a segment) advances too.
        row.addEventListener('click', (e) => { if (!seg.contains(e.target)) nav.adjust(1); });
        sync();
      });
    }

    // -- toggle ------------------------------------------------------------
    function addToggle(label, note, get, set) {
      addRow(label, note, (ctl, row, nav) => {
        const txt = el('span', 'dm-tog-txt', 'Off');
        const tog = el('span', 'dm-tog');
        ctl.appendChild(txt);
        ctl.appendChild(tog);
        const sync = () => {
          const on = !!get();
          tog.classList.toggle('dm-sel', on);
          txt.textContent = on ? 'On' : 'Off';
          row.setAttribute('aria-pressed', on ? 'true' : 'false');
        };
        settingsSync.push(sync);
        nav.activate = () => { set(!get()); sync(); };
        nav.adjust = (dir) => { set(dir > 0); sync(); };
        row.addEventListener('click', () => nav.activate());
        sync();
      });
    }

    // -- slider ------------------------------------------------------------
    function addSlider(label, note, min, max, step, get, set, format) {
      addRow(label, note, (ctl, row, nav) => {
        const sl = el('div', 'dm-sl');
        sl.setAttribute('role', 'slider');
        sl.setAttribute('aria-label', label);
        const track = el('div', 'dm-sl-track');
        const fill = el('div', 'dm-sl-fill');
        track.appendChild(fill);
        const knob = el('div', 'dm-sl-knob');
        sl.appendChild(track);
        sl.appendChild(knob);
        const val = el('span', 'dm-sl-val dm-num');
        ctl.appendChild(sl);
        ctl.appendChild(val);

        const sync = () => {
          const v = clamp(get(), min, max);
          const f = (v - min) / (max - min);
          fill.style.right = `${(1 - f) * 100}%`;
          knob.style.left = `${f * 100}%`;
          val.textContent = format(v);
          sl.setAttribute('aria-valuenow', String(Math.round(v * 100) / 100));
          sl.setAttribute('aria-valuemin', String(min));
          sl.setAttribute('aria-valuemax', String(max));
        };
        settingsSync.push(sync);

        const setFromClientX = (clientX) => {
          const r = track.getBoundingClientRect();
          if (r.width <= 0) return;
          const f = clamp((clientX - r.left) / r.width, 0, 1);
          const raw = min + f * (max - min);
          // Snap to the step grid, then re-clamp — floating point drift on a
          // 0.05 step otherwise leaves 0.9500000000000001 in localStorage.
          const snapped = Math.round(raw / step) * step;
          set(clamp(Math.round(snapped * 1000) / 1000, min, max));
          sync();
        };
        let dragging = false;
        sl.addEventListener('pointerdown', (e) => {
          e.stopPropagation();
          dragging = true;
          try { sl.setPointerCapture(e.pointerId); } catch (err) { /* ignore */ }
          setFromClientX(e.clientX);
        });
        sl.addEventListener('pointermove', (e) => { if (dragging) setFromClientX(e.clientX); });
        const stop = (e) => {
          if (!dragging) return;
          dragging = false;
          try { sl.releasePointerCapture(e.pointerId); } catch (err) { /* ignore */ }
        };
        sl.addEventListener('pointerup', stop);
        sl.addEventListener('pointercancel', stop);
        sl.addEventListener('click', (e) => e.stopPropagation());

        nav.adjust = (dir) => {
          const next = clamp(get() + dir * step, min, max);
          set(Math.round(next * 1000) / 1000);
          sync();
        };
        nav.activate = () => { /* sliders are adjusted, not activated */ };
        sync();
      });
    }

    addSegmented('Quality', 'Shadows, ambient occlusion, cloud and the post chain.',
      QUALITY_OPTIONS, () => settings.quality, (v) => applySetting('quality', v));

    addSegmented('Camera', 'Also cycles in-game with C or triangle.',
      CAMERA_OPTIONS, () => settings.camera, (v) => applySetting('camera', v));

    addToggle('Invert look', 'Flips fore/aft weight shift and air pitch.',
      () => settings.invertLook, (v) => applySetting('invertLook', v));

    addSlider('Field of view', 'Base FOV. The camera still adds its speed kick on top.',
      55, 95, 1, () => settings.fov, (v) => applySetting('fov', Math.round(v)), (v) => `${Math.round(v)}°`);

    addSlider('Master volume', 'Tyre roll, suspension, chain, wind and ambience.',
      0, 1, 0.05, () => settings.volume, (v) => applySetting('volume', v), (v) => `${Math.round(v * 100)}%`);

    addToggle('Photo mode', 'Hides the interface. F or Select brings it back.',
      () => settings.photoMode, (v) => setPhotoMode(v));

    P.foot.appendChild(el('div', 'dm-eyebrow', '← → adjust · ⏎ toggle'));
    const sFoot = el('div', 'dm-btns');
    sFoot.appendChild(makeButton(settingsView, 'Back', () => back(), { keyHint: 'Esc' }));
    P.foot.appendChild(sFoot);

    settingsView.onEnter = () => {
      // The camera mode can be cycled in-game behind our back; re-read it — but
      // never adopt 'cinematic', which the title screen forces on and which would
      // otherwise overwrite the player's chosen riding camera.
      const live = ctx && ctx.chaseCamera && ctx.chaseCamera.mode;
      if (typeof live === 'string' && live !== 'cinematic') {
        for (const o of CAMERA_OPTIONS) if (o.value === live) settings.camera = o.value;
      }
      if (ctx && ctx.quality) settings.quality = ctx.quality;
      for (const fn of settingsSync) fn();
    };
  }

  // -------------------------------------------------------------------------
  // CONTROLS
  // -------------------------------------------------------------------------
  function resolveBindings() {
    const fromInstance = ctx && ctx.input && ctx.input.bindings;
    const fromModule = InputModule && (InputModule.BINDINGS || InputModule.KEY_BINDINGS || InputModule.bindings);
    const raw = fromInstance || fromModule;
    if (Array.isArray(raw) && raw.length && raw[0] && Array.isArray(raw[0].items)) {
      return { groups: raw, source: 'from input.js' };
    }
    return { groups: KEYMAP, source: 'mirrors input.js' };
  }

  const controlsView = registerView('controls', el('div'));
  {
    const P = buildPanel(controlsView, { eyebrow: 'Reference', title: 'Controls' });
    const srcEl = el('div', 'dm-ctl-src');
    P.headR.appendChild(srcEl);

    const listHost = el('div');
    P.body.appendChild(listHost);

    function renderKeys(target, groups) {
      for (let g = 0; g < groups.length; g++) {
        if (g > 0) target.appendChild(el('span', 'dm-ctl-or', 'or'));
        const chord = groups[g];
        for (let k = 0; k < chord.length; k++) {
          if (k > 0) target.appendChild(el('span', 'dm-key-sep', '/'));
          target.appendChild(el('kbd', 'dm-key', chord[k]));
        }
      }
    }

    function build() {
      clear(listHost);
      const resolved = resolveBindings();
      srcEl.textContent = resolved.source;
      let stagger = 0;
      for (const grp of resolved.groups) {
        const sect = el('div', 'dm-ctl-grp dm-stagger');
        sect.style.setProperty('--i', String(stagger++));
        const h = el('div', 'dm-ctl-grp-h');
        h.appendChild(el('div', 'dm-eyebrow', grp.group || ''));
        h.appendChild(el('div', 'dm-rule'));
        sect.appendChild(h);

        const head = el('div', 'dm-ctl-head');
        head.appendChild(el('span', null, 'Action'));
        head.appendChild(el('span', null, 'Keyboard'));
        head.appendChild(el('span', null, 'Gamepad'));
        sect.appendChild(head);

        const items = grp.items || [];
        for (const item of items) {
          const row = el('div', 'dm-ctl-row');
          const a = el('div');
          a.appendChild(el('div', 'dm-ctl-a', item.action || ''));
          if (item.note) a.appendChild(el('div', 'dm-ctl-n', item.note));
          const kc = el('div', 'dm-ctl-k');
          renderKeys(kc, item.keys || []);
          row.appendChild(a);
          row.appendChild(kc);
          row.appendChild(el('div', 'dm-ctl-p', item.pad || '—'));
          sect.appendChild(row);
        }
        listHost.appendChild(sect);
      }
    }

    P.foot.appendChild(el('div', 'dm-eyebrow', 'Brakes and triggers are analogue'));
    const cFoot = el('div', 'dm-btns');
    cFoot.appendChild(makeButton(controlsView, 'Back', () => back(), { keyHint: 'Esc' }));
    P.foot.appendChild(cFoot);

    controlsView.onEnter = build;
  }

  // -------------------------------------------------------------------------
  // PAUSE
  // -------------------------------------------------------------------------
  const pauseView = registerView('pause', el('div'));
  let pauseTimeEl = null;
  let pauseSectionEl = null;
  let pauseProgressEl = null;
  {
    const wrap = el('div', 'dm-pause-wrap');
    const inner = el('div', 'dm-pause-inner');

    inner.appendChild(el('div', 'dm-eyebrow', 'Run held'));
    inner.appendChild(el('h2', 'dm-pause-h', 'Paused'));

    const meta = el('div', 'dm-pause-meta');
    function metaItem(label, mono) {
      const i = el('div', 'dm-pause-meta-i');
      i.appendChild(el('div', 'dm-eyebrow', label));
      const v = el('div', 'dm-pause-meta-v' + (mono ? ' dm-num' : ''), '—');
      i.appendChild(v);
      meta.appendChild(i);
      return v;
    }
    pauseTimeEl = metaItem('Elapsed', true);
    pauseSectionEl = metaItem('Section', false);
    pauseProgressEl = metaItem('Descended', true);
    inner.appendChild(meta);

    const btns = el('div', 'dm-btns');
    btns.appendChild(makeButton(pauseView, 'Resume', resumeRun, { primary: true, keyHint: 'Esc' }));
    btns.appendChild(makeButton(pauseView, 'Restart', () => beginRun(currentMode()), { keyHint: 'R' }));
    btns.appendChild(makeButton(pauseView, 'Settings', () => setView('settings')));
    btns.appendChild(makeButton(pauseView, 'Controls', () => setView('controls')));
    btns.appendChild(makeButton(pauseView, 'Quit to title', quitToTitle));
    inner.appendChild(btns);

    wrap.appendChild(inner);
    pauseView.el.appendChild(wrap);

    pauseView.onEnter = refreshPauseMeta;
  }

  function currentMode() {
    const g = gp();
    return (g && g.mode) || 'timed';
  }

  function refreshPauseMeta() {
    const g = gp();
    if (pauseTimeEl) pauseTimeEl.textContent = (g && typeof g.time === 'number') ? fmtTime(g.time) : '—';
    const bs = ctx && ctx.bike && ctx.bike.state;
    const t = bs && typeof bs.trailT === 'number' ? bs.trailT : null;
    if (pauseSectionEl) pauseSectionEl.textContent = t == null ? '—' : sectionNameAt(t);
    if (pauseProgressEl) {
      let dist = bs && typeof bs.distance === 'number' ? bs.distance : null;
      if (dist == null && t != null && ctx && ctx.trail && typeof ctx.trail.length === 'number') {
        dist = t * ctx.trail.length;
      }
      pauseProgressEl.textContent = dist == null ? '—' : `${Math.round(dist)} m`;
    }
  }

  /** Name of the trail phase containing t — used for split labels. */
  function sectionNameAt(t) {
    const phases = ctx && ctx.trail && ctx.trail.phases;
    if (Array.isArray(phases) && phases.length) {
      for (let i = 0; i < phases.length; i++) {
        const p = phases[i];
        if (t >= p.tStart && t <= p.tEnd) return p.name || p.id || 'Section';
      }
      const last = phases[phases.length - 1];
      if (t > last.tEnd) return last.name || last.id || 'Section';
      return phases[0].name || phases[0].id || 'Section';
    }
    return 'Section';
  }

  // -------------------------------------------------------------------------
  // SUMMARY
  // -------------------------------------------------------------------------
  const summaryView = registerView('summary', el('div'));
  let sumEyebrow = null, sumTitle = null, sumTimeEl = null, sumDeltaEl = null, sumDeltaNote = null;
  let sumPanel = null, sumSplitsHost = null, sumStatsHost = null, sumBadgeHost = null, sumMetaEl = null;
  {
    const P = buildPanel(summaryView, { eyebrow: 'Run complete', title: 'Summary', wide: true });
    sumPanel = P.panel;
    sumEyebrow = P.eyeEl;
    sumTitle = P.titleEl;
    sumBadgeHost = P.eyeRow;
    sumMetaEl = el('div', 'dm-eyebrow');
    P.headR.appendChild(sumMetaEl);

    const timeRow = el('div', 'dm-sum-time dm-stagger');
    timeRow.style.setProperty('--i', '0');
    sumTimeEl = el('div', 'dm-sum-time-v dm-num', '—');
    sumDeltaEl = el('div', 'dm-delta dm-num', '');
    sumDeltaNote = el('div', 'dm-delta-note', '');
    const dwrap = el('div');
    dwrap.appendChild(sumDeltaEl);
    dwrap.appendChild(sumDeltaNote);
    timeRow.appendChild(sumTimeEl);
    timeRow.appendChild(dwrap);
    P.panel.insertBefore(timeRow, P.body);

    const grid = el('div', 'dm-sum-grid');
    sumSplitsHost = el('div', 'dm-splits');
    sumStatsHost = el('div');
    grid.appendChild(sumSplitsHost);
    grid.appendChild(sumStatsHost);
    P.body.appendChild(grid);

    P.foot.appendChild(el('div', 'dm-eyebrow', '⏎ retry · Esc menu'));
    const fbtns = el('div', 'dm-btns');
    fbtns.appendChild(makeButton(summaryView, 'Retry', () => beginRun(currentMode()), { primary: true, keyHint: 'R' }));
    fbtns.appendChild(makeButton(summaryView, 'Menu', quitToTitle, { keyHint: 'Esc' }));
    P.foot.appendChild(fbtns);
  }

  // --- telemetry the summary falls back on ---------------------------------
  const telem = { topSpeed: 0, biggestAir: 0, crashes: 0, wasAirborne: false };
  function resetTelemetry() {
    telem.topSpeed = 0; telem.biggestAir = 0; telem.crashes = 0; telem.wasAirborne = false;
  }

  function readBestStore() {
    try {
      const raw = localStorage.getItem(BEST_KEY);
      if (!raw) return null;
      const b = JSON.parse(raw);
      if (b && typeof b.time === 'number' && isFinite(b.time)) {
        // A best on a different mountain is meaningless.
        if (b.seed != null && ctx && ctx.seed != null && b.seed !== ctx.seed) return null;
        return b;
      }
    } catch (e) { /* ignore */ }
    return null;
  }

  function writeBestStore(time, splits) {
    try {
      const flat = [];
      if (Array.isArray(splits)) {
        for (const s of splits) flat.push(typeof s === 'number' ? s : (s && s.time));
      }
      localStorage.setItem(BEST_KEY, JSON.stringify({
        time, splits: flat, seed: ctx && ctx.seed, at: Date.now(),
      }));
    } catch (e) { /* ignore */ }
  }

  /** First finite number among the candidates, else null. */
  function num() {
    for (let i = 0; i < arguments.length; i++) {
      const c = arguments[i];
      if (typeof c === 'number' && isFinite(c)) return c;
    }
    return null;
  }

  /** Coerce whatever gameplay hands us into a stable shape (see CONTRACT-NOTE). */
  function normaliseResult(payload) {
    const p = (payload && typeof payload === 'object') ? payload : {};
    const g = gp() || {};

    const time = num(p.time, p.totalTime, p.finalTime, p.elapsed, g.time);

    const rawSplits = Array.isArray(p.splits) ? p.splits : (Array.isArray(g.splits) ? g.splits : []);
    const cps = (ctx && ctx.trail && ctx.trail.checkpoints) || [];

    const bestObj = (p.best && typeof p.best === 'object') ? p.best
      : ((g.best && typeof g.best === 'object') ? g.best : null);
    const storedBest = readBestStore();
    const bestTime = num(
      p.bestTime,
      bestObj ? num(bestObj.time, bestObj.total) : null,
      typeof p.best === 'number' ? p.best : null,
      typeof g.best === 'number' ? g.best : null,
      storedBest ? storedBest.time : null,
    );
    const bestSplitsRaw = (bestObj && Array.isArray(bestObj.splits)) ? bestObj.splits
      : (Array.isArray(p.bestSplits) ? p.bestSplits
      : ((storedBest && Array.isArray(storedBest.splits)) ? storedBest.splits : []));
    const bestSplitTime = (i) => {
      const s = bestSplitsRaw[i];
      if (typeof s === 'number' && isFinite(s)) return s;
      if (s && typeof s === 'object') return num(s.time, s.total, s.cumulative);
      return null;
    };

    const splits = [];
    for (let i = 0; i < rawSplits.length; i++) {
      const s = rawSplits[i];
      let st = null, delta = null, t = null, name = null;
      if (typeof s === 'number') {
        st = s;
      } else if (s && typeof s === 'object') {
        st = num(s.time, s.total, s.cumulative, s.at, s.elapsed);
        delta = num(s.delta, s.diff);
        t = num(s.t, s.trailT);
        name = typeof s.name === 'string' ? s.name : (typeof s.label === 'string' ? s.label : null);
      }
      if (delta == null) {
        const b = bestSplitTime(i);
        if (b != null && st != null) delta = st - b;
      }
      if (t == null && cps[i] && typeof cps[i].t === 'number') t = cps[i].t;
      if (!name) name = (t != null) ? sectionNameAt(t) : `Split ${i + 1}`;
      splits.push({ index: i, time: st, delta, name });
    }

    let isPB = p.isPersonalBest;
    if (typeof isPB !== 'boolean') isPB = p.personalBest;
    if (typeof isPB !== 'boolean') isPB = p.newBest;
    if (typeof isPB !== 'boolean') isPB = p.isBest;
    if (typeof isPB !== 'boolean') {
      isPB = (time != null) && (bestTime == null || time < bestTime - 0.0005);
    }
    const deltaTotal = (time != null && bestTime != null) ? time - bestTime : null;

    const score = num(p.score, p.style, p.styleScore, g.score) || 0;
    let topSpeed = num(p.topSpeed, p.maxSpeed, g.topSpeed);
    if (topSpeed == null) topSpeed = telem.topSpeed;

    let airTime = num(p.biggestAir, p.bestAir, p.maxAirTime, g.biggestAir);
    let airDist = num(p.biggestAirDistance, p.maxAirDistance);
    if (airTime == null && p.biggestAir && typeof p.biggestAir === 'object') {
      airTime = num(p.biggestAir.time, p.biggestAir.airTime);
      if (airDist == null) airDist = num(p.biggestAir.distance);
    }
    if (airTime == null) airTime = telem.biggestAir;

    let crashes = num(p.crashes, p.crashCount, g.crashes, g.crashCount);
    if (crashes == null) crashes = telem.crashes;

    const mode = p.mode || g.mode || 'timed';

    return { time, splits, bestTime, isPB, deltaTotal, score, topSpeed, airTime, airDist, crashes, mode };
  }

  function statTile(label, value, unit, sub, hot) {
    const t = el('div', 'dm-stat' + (hot ? ' dm-hot' : ''));
    t.appendChild(el('div', 'dm-eyebrow', label));
    const v = el('div', 'dm-stat-v dm-num');
    v.appendChild(document.createTextNode(value));
    if (unit) v.appendChild(el('span', 'dm-stat-u', unit));
    t.appendChild(v);
    t.appendChild(el('div', 'dm-stat-sub', sub || ''));
    return t;
  }

  function showSummary(payload) {
    const r = normaliseResult(payload);
    const free = r.mode === 'free';

    // --- header ------------------------------------------------------------
    sumPanel.classList.toggle('dm-pb', !!r.isPB && !free);
    sumEyebrow.textContent = free ? 'Session ended' : 'Run complete';
    sumTitle.textContent = free ? 'Free ride' : 'Summary';

    const oldBadge = sumBadgeHost.querySelector('.dm-pb-badge');
    if (oldBadge) sumBadgeHost.removeChild(oldBadge);
    if (r.isPB && !free) {
      const badge = el('div', 'dm-pb-badge');
      badge.appendChild(el('span', 'dm-pb-dot'));
      badge.appendChild(document.createTextNode('Personal best'));
      sumBadgeHost.appendChild(badge);
    }
    sumMetaEl.textContent = `Seed ${(ctx && ctx.seed != null) ? ctx.seed : '—'}`;

    sumTimeEl.textContent = free ? '—' : fmtTime(r.time);
    if (!free && r.deltaTotal != null) {
      sumDeltaEl.textContent = fmtDelta(r.deltaTotal);
      sumDeltaEl.className = 'dm-delta dm-num ' + deltaClass(r.deltaTotal);
      sumDeltaNote.textContent = (r.isPB ? 'previous best ' : 'against best ') + fmtTime(r.bestTime);
    } else if (!free && r.isPB && r.time != null) {
      sumDeltaEl.textContent = 'First run';
      sumDeltaEl.className = 'dm-delta dm-num dm-up';
      sumDeltaNote.textContent = 'benchmark set';
    } else {
      sumDeltaEl.textContent = '';
      sumDeltaEl.className = 'dm-delta dm-num dm-flat';
      sumDeltaNote.textContent = '';
    }

    // --- splits ------------------------------------------------------------
    clear(sumSplitsHost);
    const head = el('div', 'dm-split-head');
    head.appendChild(el('span', null, '#'));
    head.appendChild(el('span', null, 'Section'));
    head.appendChild(el('span', null, 'Split'));
    head.appendChild(el('span', null, 'Delta'));
    sumSplitsHost.appendChild(head);

    let rowI = 0;
    const cps = (ctx && ctx.trail && ctx.trail.checkpoints) || [];
    const rowCount = Math.max(r.splits.length, free ? 0 : Math.min(cps.length, 8));
    // Eight gates over nine phases means two gates can land in the same section.
    // Number the repeats rather than printing "The Flow" twice.
    const seen = Object.create(null);
    for (let i = 0; i < rowCount; i++) {
      const s = r.splits[i] || {
        time: null,
        delta: null,
        name: (cps[i] && typeof cps[i].t === 'number') ? sectionNameAt(cps[i].t) : `Split ${i + 1}`,
      };
      const n = seen[s.name] = (seen[s.name] || 0) + 1;
      const label = n > 1 ? `${s.name} ${ROMAN[n] || n}` : s.name;
      const row = el('div', 'dm-split-row');
      row.style.setProperty('--i', String(rowI++));
      row.appendChild(el('div', 'dm-split-i dm-num', String(i + 1).padStart(2, '0')));
      row.appendChild(el('div', 'dm-split-n', label));
      row.appendChild(el('div', 'dm-split-t dm-num', s.time == null ? '—' : fmtTime(s.time)));
      row.appendChild(el('div', 'dm-split-d dm-num ' + deltaClass(s.delta), s.delta == null ? '—' : fmtDelta(s.delta)));
      sumSplitsHost.appendChild(row);
    }
    if (!free) {
      const fin = el('div', 'dm-split-row dm-final');
      fin.style.setProperty('--i', String(rowI++));
      fin.appendChild(el('div', 'dm-split-i dm-num', '✓'));
      fin.appendChild(el('div', 'dm-split-n', 'Finish'));
      fin.appendChild(el('div', 'dm-split-t dm-num', fmtTime(r.time)));
      fin.appendChild(el('div', 'dm-split-d dm-num ' + deltaClass(r.deltaTotal),
        r.deltaTotal == null ? '—' : fmtDelta(r.deltaTotal)));
      sumSplitsHost.appendChild(fin);
    }

    // --- stats -------------------------------------------------------------
    clear(sumStatsHost);
    const stats = el('div', 'dm-stats dm-stagger');
    stats.style.setProperty('--i', '2');
    const kmh = (r.topSpeed || 0) * 3.6;
    stats.appendChild(statTile('Style', String(Math.round(r.score)), 'pts',
      r.score > 0 ? 'air, whips, manuals, clean landings' : 'nothing scored this run', r.score > 0));
    stats.appendChild(statTile('Top speed', kmh.toFixed(1), 'km/h',
      `${(r.topSpeed || 0).toFixed(1)} m/s`, kmh > 60));
    stats.appendChild(statTile('Biggest air', (r.airTime || 0).toFixed(2), 's',
      (r.airDist != null) ? `${r.airDist.toFixed(1)} m carried` : 'hang time', (r.airTime || 0) > 1.2));
    stats.appendChild(statTile('Crashes', String(Math.round(r.crashes)), '',
      r.crashes === 0 ? 'clean run' : 'respawned at the last gate', false));
    sumStatsHost.appendChild(stats);

    // Keep our own fallback best in step.
    if (!free && r.time != null && r.isPB) writeBestStore(r.time, r.splits);

    setView('summary', { focus: 0 });
  }

  // -------------------------------------------------------------------------
  // Keyboard. Registered in the CAPTURE phase on window so that, while a menu is
  // open, the keystroke never reaches input.js — otherwise arrowing through the
  // menu would also steer the bike.
  // -------------------------------------------------------------------------
  function back() {
    switch (view) {
      case 'settings':
      case 'controls':
        // Return to whatever opened us, defaulting to the title screen.
        setView((prevView === 'pause') ? 'pause' : 'title', { remember: false });
        break;
      case 'pause':
        resumeRun();
        break;
      case 'summary':
        quitToTitle();
        break;
      case 'title':
        setFocus(0);   // nowhere above the root menu
        break;
      default:
        break;
    }
  }

  function onKeyDown(e) {
    if (settings.photoMode) {
      if (e.code === 'KeyF' || e.code === 'Escape') {
        setPhotoMode(false);
        e.preventDefault();
        e.stopPropagation();
      }
      return;
    }

    if (!isBlocking()) {
      // Not modal: claim only the pause key, and deliberately let it through to
      // input.js so gameplay can pause itself (we reconcile in lateUpdate).
      if ((e.code === 'Escape' || e.code === 'KeyP') && !e.repeat) openPause();
      return;
    }

    // Never eat the browser's own shortcuts (reload, devtools, tab switching).
    if (e.metaKey || e.ctrlKey || e.altKey || FN_KEY.test(e.code)) return;

    switch (e.code) {
      case 'ArrowUp': case 'KeyW': moveFocus(-1); break;
      case 'ArrowDown': case 'KeyS': moveFocus(1); break;
      case 'ArrowLeft': case 'KeyA': adjustFocused(-1); break;
      case 'ArrowRight': case 'KeyD': adjustFocused(1); break;
      case 'Enter': case 'NumpadEnter': case 'Space': activateFocused(); break;
      case 'Escape': case 'Backspace': back(); break;
      case 'Tab': moveFocus(e.shiftKey ? -1 : 1); break;
      case 'Home': setFocus(0); break;
      case 'End': setFocus(-1); break;
      case 'KeyR':
        if (view === 'summary' || view === 'pause') beginRun(currentMode());
        break;
      case 'KeyF':
        setPhotoMode(true);
        break;
      default:
        if (view === 'title' && e.code.length === 6 && e.code.indexOf('Digit') === 0) {
          const n = Number(e.code.slice(5)) - 1;
          const v = currentView();
          if (v && n >= 0 && n < v.items.length) { setFocus(n); activateFocused(); }
        }
        break;
    }
    // Swallow everything while modal so no stray key reaches the bike.
    e.preventDefault();
    e.stopPropagation();
  }

  // NOTE: keyup is deliberately NOT intercepted. If the player is holding W when
  // the menu opens and releases it while modal, input.js must see that keyup or
  // the key stays in its held set forever and the bike leans back on resume.

  window.addEventListener('keydown', onKeyDown, { capture: true });
  root.addEventListener('contextmenu', (e) => { if (isBlocking()) e.preventDefault(); });

  // -------------------------------------------------------------------------
  // Pause reconciliation
  // -------------------------------------------------------------------------
  let wantPaused = false;
  // The Escape keydown that opens the pause overlay is deliberately NOT swallowed
  // (gameplay needs to see it). input.js therefore raises state.pause on the very
  // next frame — by which time we are modal, and padNav would read it as "Start
  // pressed in a menu" and immediately resume. Hold the guard until we have seen
  // one frame with state.pause low, which consumes exactly that stale edge and no
  // more, so a genuine Start press a moment later still works.
  let padPauseGuard = false;

  function openPause() {
    if (settings.photoMode) return;
    if (view === 'title' || view === 'summary') return;
    if (view === 'settings' || view === 'controls') { back(); return; }
    wantPaused = true;
    padPauseGuard = true;
    setView('pause');
  }

  // -------------------------------------------------------------------------
  // Events
  // -------------------------------------------------------------------------
  const unsubs = [];
  function on(name, fn) {
    if (!ctx || !ctx.events || typeof ctx.events.on !== 'function') return;
    const off = ctx.events.on(name, fn);
    unsubs.push(typeof off === 'function' ? off : () => ctx.events.off(name, fn));
  }

  on('run:finish', (payload) => showSummary(payload));
  on('run:start', () => {
    resetTelemetry();
    wantPaused = false;
    if (view !== 'none') setView('none');
  });
  on('run:crash', () => { telem.crashes++; });
  on('pause', () => openPause());
  on('run:pause', () => openPause());
  on('resume', () => { if (view === 'pause') { wantPaused = false; setView('none'); } });
  on('run:resume', () => { if (view === 'pause') { wantPaused = false; setView('none'); } });
  on('quality:changed', (q) => {
    const v = typeof q === 'string' ? q : (q && q.quality);
    if (v && v !== settings.quality) {
      settings.quality = v;
      if (view === 'settings') for (const fn of settingsSync) fn();
    }
  });

  // -------------------------------------------------------------------------
  // Gamepad navigation — via ctx.input.state only, no direct Gamepad polling.
  //   pitch  = left stick Y   (input.js already flips it: up-stick is +)
  //   steer  = left stick X
  //   pump   = A / ✕          (level, so we edge-detect it here)
  //   reset  = B / ○          (already edge-triggered by input.js)
  //   pause  = Start          (already edge-triggered)
  // -------------------------------------------------------------------------
  let navDir = 0, navHold = 0, navRepeatAt = 0;
  let adjDir = 0, adjHold = 0, adjRepeatAt = 0;
  let prevPump = 0;
  let prevPhoto = false;
  let lastGameplayState = '';

  function padNav(dt) {
    const s = ctx && ctx.input && ctx.input.state;
    if (!s) return;

    const dv = s.pitch > 0.55 ? -1 : (s.pitch < -0.55 ? 1 : 0);
    if (dv !== navDir) {
      navDir = dv; navHold = 0; navRepeatAt = NAV_REPEAT_DELAY;
      if (dv !== 0) moveFocus(dv);
    } else if (dv !== 0) {
      navHold += dt;
      if (navHold >= navRepeatAt) { moveFocus(dv); navRepeatAt += NAV_REPEAT_RATE; }
    }

    const dh = s.steer > 0.55 ? 1 : (s.steer < -0.55 ? -1 : 0);
    if (dh !== adjDir) {
      adjDir = dh; adjHold = 0; adjRepeatAt = NAV_REPEAT_DELAY;
      if (dh !== 0) adjustFocused(dh);
    } else if (dh !== 0) {
      adjHold += dt;
      if (adjHold >= adjRepeatAt) { adjustFocused(dh); adjRepeatAt += NAV_REPEAT_RATE; }
    }

    if (s.pump > 0.5 && prevPump <= 0.5) activateFocused();
    if (s.reset) back();
    if (s.pause && !padPauseGuard) {
      if (view === 'pause') resumeRun();
      else back();
    }
  }

  // -------------------------------------------------------------------------
  // Frame. Allocation-free: only numeric comparisons and, on state changes,
  // DOM writes.
  // -------------------------------------------------------------------------
  function lateUpdate(dt) {
    const d = Math.min(dt || 0, 1 / 20);
    const s = ctx && ctx.input && ctx.input.state;

    // --- photo mode toggles from anywhere ---------------------------------
    if (s) {
      if (s.photoMode && !prevPhoto) setPhotoMode(!settings.photoMode);
      prevPhoto = !!s.photoMode;
    }
    const photoActive = !!(settings.photoMode || (ctx && ctx.settings && ctx.settings.photoMode));
    if (photoActive !== root.classList.contains('dm-photo')) {
      root.classList.toggle('dm-photo', photoActive);
      settings.photoMode = photoActive;
    }
    if (photoActive) { prevPump = s ? s.pump : 0; return; }

    // --- telemetry ---------------------------------------------------------
    const g = gp();
    const bs = ctx && ctx.bike && ctx.bike.state;
    if (bs && (!g || g.state === 'running' || g.state === 'crashed')) {
      if (typeof bs.speed === 'number' && bs.speed > telem.topSpeed) telem.topSpeed = bs.speed;
      if (bs.airborne) {
        if (typeof bs.airTime === 'number' && bs.airTime > telem.biggestAir) telem.biggestAir = bs.airTime;
        telem.wasAirborne = true;
      } else if (telem.wasAirborne) {
        telem.wasAirborne = false;
        if (typeof bs.lastAirTime === 'number' && bs.lastAirTime > telem.biggestAir) telem.biggestAir = bs.lastAirTime;
      }
    }

    // --- reconcile with gameplay's own state -------------------------------
    if (g && typeof g.state === 'string') {
      if (g.state !== lastGameplayState) {
        if (g.state === 'paused' && view === 'none') { wantPaused = true; setView('pause'); }
        else if (g.state !== 'paused' && view === 'pause' && !wantPaused) setView('none');
        else if (g.state === 'menu' && view === 'none') showTitle();
        lastGameplayState = g.state;
      }
      // We asked for a pause and gameplay has not taken it — ask once.
      if (wantPaused && view === 'pause' && g.state !== 'paused' && g.state !== 'menu' &&
          g.state !== 'finished' && typeof g.pause === 'function') {
        try { g.pause(); } catch (e) { /* ignore */ }
        lastGameplayState = typeof g.state === 'string' ? g.state : lastGameplayState;
      }
    }

    if (view === 'pause') refreshPauseMeta();

    // --- gamepad -----------------------------------------------------------
    if (padPauseGuard && s && !s.pause) padPauseGuard = false;
    if (isBlocking()) padNav(d);
    else if (s && s.pause) openPause();     // Start with no menu up

    prevPump = s ? s.pump : 0;
  }

  // -------------------------------------------------------------------------
  // Public surface
  // -------------------------------------------------------------------------
  return {
    root,

    get open() { return isBlocking(); },
    get view() { return view; },

    init() {
      // Push the persisted settings into the world before the first frame.
      applySetting('invertLook', settings.invertLook, { persist: false });
      applySetting('fov', settings.fov, { persist: false });
      applySetting('volume', settings.volume, { persist: false });
      if (ctx && ctx.quality !== settings.quality) applySetting('quality', settings.quality, { persist: false });
      enterMenuState();
      lastGameplayState = (gp() && typeof gp().state === 'string') ? gp().state : '';
      setCameraMode('cinematic');
      showTitle();
    },

    /** show() with no argument opens the title screen. */
    show(which) { setView(which && views.has(which) ? which : 'title'); },
    hide() { setView('none'); },
    showTitle,
    showSummary,
    pause: openPause,
    resume: resumeRun,

    lateUpdate,

    resize() { /* the layout is fluid (clamp/vw); nothing to recompute */ },

    dispose() {
      window.removeEventListener('keydown', onKeyDown, { capture: true });
      clearTimeout(leaveTimer);
      for (const off of unsubs) { try { off(); } catch (e) { /* ignore */ } }
      unsubs.length = 0;
      if (root.parentNode) root.parentNode.removeChild(root);
      if (styleEl.parentNode) styleEl.parentNode.removeChild(styleEl);
      views.clear();
    },
  };
}
