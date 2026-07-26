// postfx.js — the pmndrs `postprocessing` chain for DESCENT.
//
// This module owns everything that happens between "the scene has been rasterised" and
// "pixels hit the screen": ambient occlusion, depth of field, motion blur, bloom,
// exposure, tone mapping, colour grading, lens artefacts, grain, vignette and
// anti-aliasing. main.js calls `postfx.render(dt)` instead of `renderer.render()`.
//
// Chain (CONTRACT §6, order matters):
//
//   RenderPass                                              scene → HDR half-float buffer
//   N8AOPostPass                                            contact-scale AO
//   EffectPass[ DepthOfField ]                              FAR FIELD ONLY, > 30 m
//   EffectPass[ MotionBlur ]                                camera reprojection + speed radial
//   EffectPass[ AdaptiveExposure, ToneMapping, LUT3D ]      HDR → graded LDR
//   EffectPass[ ChromaticAberration ]                       edge-weighted fringing (ultra only)
//   EffectPass[ LensDistortion, Bloom, Film ]               barrel + bloom + shoulder + grain
//   EffectPass[ SMAA ]                                      anti-aliasing, last
//
// ---------------------------------------------------------------------------------
// CONTRACT-NOTE: `postprocessing` 6.39.3 does NOT ship MotionBlurEffect or
// VelocityDepthNormalPass (they were removed after v6.28 and have not returned). This
// module therefore implements motion blur itself, as a custom `Effect`: per-pixel camera
// reprojection (reconstruct world position from the depth buffer, project it with the
// previous frame's view-projection matrix, blur along the resulting screen-space
// velocity) plus a speed-driven radial component. That gives correct camera translation
// *and* rotation blur for all static geometry. True per-object velocity would require a
// velocity G-buffer, which means overriding materials on meshes owned by other modules —
// out of scope for this file. The rider/bike is explicitly protected by a world-space
// sphere mask (`riderSphere`) rather than being smeared as if it were static geometry.
//
// CONTRACT-NOTE: `ToneMappingEffect` only supports adaptive luminance for its
// Reinhard2 operator, not for ACES/AgX. Adaptive exposure is therefore implemented as a
// separate `AdaptiveExposureEffect` placed *before* tone mapping, reusing the library's
// LuminancePass + AdaptiveLuminancePass (GPU-side; no readback stall) and driving a
// clamped exposure multiplier. Tone mapping itself stays ACES Filmic.
//
// CONTRACT-NOTE: this module calls `ctx.engine.setToneMappingEnabled(false)` during
// init so ACES is not applied twice (engine.js exposes exactly that hook). If the engine
// ever stops providing it we fall back to setting `renderer.toneMapping = NoToneMapping`
// directly, because a double tone map is catastrophic, not cosmetic.
//
// CONTRACT-NOTE: §9 lists no `postfx`-specific settings beyond the effect toggles, so
// internal quality (AO sample count, bloom mip levels, DoF resolution scale, motion-blur
// taps, SMAA preset, MSAA) is derived from `ctx.quality` here.
//
// CONTRACT-NOTE (r2 review, P1-c): the **god-ray pass has been deleted for this
// milestone**. `GodRaysEffect`'s visibility term is a pure facing test with no occlusion
// test, so the 60-sample radial march ran at full price with the sun behind a ridge and
// composited shafts *over* the ridge that was blocking it. At the authored 0.5 resolution
// scale and KernelSize.SMALL it produced no shaft structure in any of the fourteen review
// frames — only a milky wash — for ~1 ms. To restore it: give it its own EffectPass, gate
// the blend weight on an actual depth-buffer occlusion test at the sun's projected screen
// position (not on `camFwd · sunDir`), and drop the sample count to ~40. `sky.js` already
// draws a real sun disc (`uShowSunDisc`), so no light-source proxy mesh is needed here.
//
// CONTRACT-NOTE (r2 review, P1-b): §6 lists bloom *before* tone mapping. It is now
// **after** the tone map and grade, merged into the lens pass. `BloomEffect` thresholds on
// the luminance of the pass input buffer; pre-tonemap that buffer is unbounded HDR where a
// daylight sky sits many units above 1.0, so a 0.85 threshold admitted the entire sky at
// full weight and the mip chain smeared it over the whole frame. Post-tonemap the input is
// bounded to [0,1] and the threshold means what it says. Consequence: bloom now happens
// after chromatic aberration rather than before it, which is a wash visually.
//
// CONTRACT-NOTE (r2 review P0-a, REVISED by r3 review I2): DoF is back on during gameplay,
// but it is now **structurally incapable** of doing what r2 deleted it for. r2's defect was a
// half-resolution *near-field* blur over the whole frame, focused on a bike that was not in
// front of the lens, destroying the near-field surface detail the terrain shader pays 7 ms to
// synthesise. What runs now is far field only:
//   * `CircleOfConfusionMaterial`'s fragment shader is patched so the near channel is a hard
//     `0.0`. Not "small" — zero. Nothing between the lens and the focus plane can ever be
//     defocused, at any setting, by any code path.
//   * the far ramp starts at `DOF_FAR_START` (30 m, pushed further out if the subject is
//     further than that) and reaches its maximum at `DOF_FAR_END` (120 m).
//   * the maximum circle of confusion is `DOF_MAX_COC_PX` (3.5) *pixels*. `bokehScale` is in
//     drawing-buffer pixels for this library (`step = texelSize * coc * scale`), so this is a
//     measured cap, not a taste knob. r2 capped it at 0.6, i.e. sub-pixel, i.e. nothing.
// If the shader patch does not apply (a library upgrade changed the source), DoF is dropped
// entirely rather than falling back to the near-field behaviour r2 removed.
//
// CONTRACT-NOTE (r3 review, I1): the **highlight shoulder** lives in `FilmEffect`, not in the
// grading LUT, because it has to be scene-adaptive and a 33³ cube cannot be. It is driven from
// the *same* adapted-luminance texture the exposure controller already computes on the GPU —
// no readback, no CPU cost, and it fails to identity when the metering signal is missing.
//
// CONTRACT-NOTE (r5 review): **where the highlight range is actually decided.** r3 put a
// curve in `FilmEffect` and called it a shoulder. It is not, and it cannot be, because by the
// time `FilmEffect` runs the tone map has already `saturate()`d everything: ACES (three's
// `RRTAndODTFit`, which `postprocessing` compiles in) reaches exactly 1.0 at an input of 25.7,
// and the r5 set measured a sky sitting ~70x the ground radiance — so the entire sky arrived
// at the display stage as one identical clipped value with nothing left to roll off. Worse,
// the r3 curve at its OPEN settings (knee 0.90, white 0.985) *expanded* the 0.90–1.00 band and
// pushed a large slab of every open frame the rest of the way to 255.
//
// So there are now two curves, in the two places they can each do their job:
//   1. an **HDR roll-off** inside `AdaptiveExposureEffect`, applied immediately after the
//      exposure multiply and before the tone map — logarithmic above a knee at 0.65,
//      ratio-preserving on the max channel, exact identity below. This is what stops the sky
//      clipping and what lets four stops of it arrive as four distinguishable values.
//   2. a **display roll-off** in `FilmEffect`, which now sits *below* the identity line
//      through the shoulder band instead of above it, and which switches to a bounded lift
//      (never a hard remap onto 1.0) for frames that genuinely have no highlight in them.
// The exposure controller's metering mask changed in the same pass: r3's hard upper-25% crop
// became a centre-weighted field with the sky down-weighted rather than excluded, plus an
// explicit bounded highlight term. See the constants for the measured derivation of each.
//
// CONTRACT-NOTE (r6 review): **the r5 roll-off had no white point, and that is its own
// failure.** Measured across all sixteen 1920x1080 review shots: 0.000% crushed, 0.000%
// clipped — and no pixel above L=242, no channel reaching 249. There is no true white, no sun
// glint and no specular hit anywhere in the set. Walking the whole chain numerically, the r5
// constants require 3,174 units of post-exposure radiance to exceed L=250 and 2.8e9 to reach
// 255; the brightest pixel in sixteen frames is 50. The logarithm is unbounded but grows far
// too slowly to ever reach ACES's saturation point, and raising its softness makes the sky
// brighter *and* pushes the white point further away. Fixed by an additive specular-escape
// term inside the same HDR curve — zero below the knee, C1 as it leaves it, linear far above,
// so the sun's own disc and a mirror-angle glint saturate while the 7-15 unit sky moves by a
// tenth of a level. See HL_SPEC_GAIN for the full derivation and the before/after table.
// Two other r6 items live in this file: `TIERS.high` finally gets `msaa: 2` (the line
// vegetation.js specified three rounds ago), and `init()` now precompiles the whole pipeline
// behind the loading screen with `checkShaderErrors` gated — see `precompile()`.
// ---------------------------------------------------------------------------------
//
// Everything is procedural — the grading LUT is generated in code at boot. No files,
// no network.

import * as THREE from 'three';
import {
  EffectComposer,
  RenderPass,
  EffectPass,
  Effect,
  EffectAttribute,
  BlendFunction,
  BloomEffect,
  DepthOfFieldEffect,
  ToneMappingEffect,
  ToneMappingMode,
  LUT3DEffect,
  LookupTexture,
  ChromaticAberrationEffect,
  LensDistortionEffect,
  SMAAEffect,
  SMAAPreset,
  EdgeDetectionMode,
  LuminancePass,
  AdaptiveLuminancePass,
} from 'postprocessing';
import { N8AOPostPass } from 'n8ao';

import { clamp, clamp01, lerp, smoothstep, damp } from './rng.js';

// ---------------------------------------------------------------------------
// Module-scope scratch. Nothing below allocates per frame.
// ---------------------------------------------------------------------------
const _viewProj = new THREE.Matrix4();
const _invViewProj = new THREE.Matrix4();
const _camWorld = new THREE.Vector3();
const _camFwd = new THREE.Vector3();
const _focusPoint = new THREE.Vector3();
const _focusCand = new THREE.Vector3();
const _tmpV3 = new THREE.Vector3();
// 1x1 readback scratch for `debugExposure()`. Module scope so the diagnostic itself never
// allocates, even if a harness calls it once per shot across a long run.
const _exposureReadback = new Uint8Array(4);

// Speed response window. 7 m/s ≈ 25 km/h (rolling), 27 m/s ≈ 97 km/h (pinned).
const SPEED_LOW = 7.0;
const SPEED_HIGH = 27.0;

// --- focus ---------------------------------------------------------------
// Where the eye goes on a descent: not at the bike, at the piece of trail you are about
// to be on. 15 m at 12 m/s is a little over a second of lead, which is what a rider
// actually looks at.
const FOCUS_AHEAD_M = 15.0;
// Beyond this the camera is nowhere near the trail (an establishing wide, a QA free
// pose), and the spline is no longer a sane focus source — fall back to the lens axis.
const FOCUS_TRAIL_MAX_DIST = 45.0;
// Rider chest height above the tread.
const FOCUS_HEIGHT = 1.05;

// --- metering (r5 review) --------------------------------------------------
// r3 gave the controller a hard crop that threw away the top 25% of the frame. That did
// stop it metering *on* the sky, and it also made it structurally blind to the sky
// clipping — which is how r5 arrived at 38% of pixels above L=250 with the controller
// reporting a perfectly healthy frame. What replaces it is two terms in one statistic:
//
//   LEVEL     a weighted mean of scene luminance. The weight is centre-weighted and the
//             sky is weighted *down*, not out. It has to stay ground-dominated: measured
//             on the r5 set the sky sits ~70x the ground radiance, so any metering that
//             gives the sky real weight exposes for the sky and returns the ground to the
//             mud r2 spent a round digging it out of. Each pixel's contribution is also
//             capped, so no single region can run away with the mean.
//   HIGHLIGHT the fraction of the frame inside the top stop of the display range, measured
//             *at the exposure this frame will actually be rendered at* (the metering
//             shader recomputes the controller's own transfer function from last frame's
//             adapted value). This is the term that "pulls exposure down when the top
//             percentile clips", and unlike the level term it is bounded — it can move the
//             metered average by at most METER_HI_GAIN / meterNorm, i.e. ~0.5 stop.
//
// Both are summed into the single 8-bit channel `AdaptiveLuminancePass` reads, which is why
// they have to fit inside [0,1] together: METER_LEVEL_CAP + METER_HI_GAIN < 1.
const METER_SKY_WEIGHT = 0.035;   // weight at the top of the frame. NOT zero.
const METER_SKY_LO = 0.55;        // uv.y (0 = bottom) where the sky ramp starts
const METER_SKY_HI = 0.82;        // uv.y where it reaches METER_SKY_WEIGHT
const METER_EDGE_WEIGHT = 0.60;   // weight at the left/right frame edge
const METER_EDGE_LO = 0.28;       // |uv.x - 0.5| where the edge ramp starts
const METER_EDGE_HI = 0.50;
const METER_LEVEL_CAP = 0.60;     // per-pixel cap on the level term
// Post-exposure linear radiance. ACES reaches display 0.94 at 1.79 and 0.98 at 4.28, so
// this window is "the top stop and a half of the display range".
const METER_HI_KNEE = 3.0;
const METER_HI_TOP = 12.0;
const METER_HI_GAIN = 0.10;

// --- HDR highlight roll-off (r5 review) ------------------------------------
// The regression is not that the tone curve is wrong, it is that the sky arrives at the
// tone curve ~70x above the ground. ACES saturates at an input of 25.7 (measured against
// three's RRTAndODTFit, which is what `postprocessing` compiles in), so everything above
// that is *already* exactly 1.0 by the time any display-space curve can see it. A
// display-space shoulder cannot roll off information that has been clamped upstream of it;
// it can only avoid adding to the pile. So the roll-off runs here, in HDR, immediately
// after the exposure multiply and before the tone map:
//
//   y(m) = K + S * ln(1 + (m - K) / S)      for m > K,   y(m) = m otherwise
//
// y(K) = K and y'(K) = 1, so it is exactly identity below the knee and C1 across it — the
// shadow floor and the entire midtone range are bit-untouched, which is the r2 fix that
// must not be re-opened. Above the knee it is logarithmic rather than asymptotic: it never
// stops separating, so four stops of sky still arrive at the tone map as four *distinguishable*
// values instead of one flat white. Applied ratio-preserving on the max channel, so a blue
// sky stays blue rather than being desaturated on the way down.
//
// K = 0.65: measured sunlit ground on the r5 set sits at 0.10-0.25 post-exposure linear and
// snow/white-lambertian at ~0.7-0.8, so the knee is above everything that is legitimately
// *surface* and below everything that is sky or specular.
// S = 0.90: walked end to end (roll-off -> ACES -> display shoulder -> sRGB), post-exposure
// radiance now maps 0.2 -> L135, 0.55 -> 202, 1.2 -> 221, 3 -> 234, 7 -> 241, 15 -> 245,
// 50 -> 249, 400 -> 252. Everything at and below 0.55 is bit-identical to before.
// The number that matters: a pixel needed **2.22** units of post-exposure radiance to exceed
// L=250 through the r5 chain, and needs far more through this one. The r5 sky measures 7-15.
// That single ratio is the regression and its fix.
const HL_KNEE = 0.65;
const HL_SOFT = 0.90;

// --- specular escape (r6 review, item 2) -----------------------------------
// The r5 roll-off above is correct and stays. What it did not have is a *white point*.
//
// The r6 measurement: across all sixteen 1920x1080 review shots no pixel exceeds L=242 and
// no channel reaches 249. That is not a pass, it is the opposite failure — there is no true
// white, no sun glint and no specular hit anywhere in the set. Real sunlit photography clips
// somewhere.
//
// The cause is the shape of the curve, not its tuning, and it is worth being exact about
// because "raise S" is the obvious answer and it is wrong. Walking the whole chain
// numerically (roll-off -> ACES RRTAndODTFit -> the grading cube -> FilmEffect's OPEN
// shoulder -> sRGB) the r5 constants require:
//     L = 242  at   50 units of post-exposure radiance   <- exactly the measured peak
//     L = 245  at  132
//     L = 250  at  3,174
//     L = 255  at  2.8e9                                 <- i.e. never
// The logarithm is the problem. `K + S*ln(1 + over/S)` is unbounded but it grows so slowly
// that ACES's saturation point (an input of 25.7) sits at m = 2.5e12. Raising S does not fix
// it and makes it worse in the only band that matters: S = 3.0 would lift the 7-15 sky by
// ~5 levels *and* push the white point out to m = 1.3e4.
//
// So the fix is a second, additive term that is asleep everywhere the r5 curve was doing its
// job and wakes up only where a genuine specular lives:
//
//     y += G * over * over / (over + Q)
//
//   * exactly 0 below the knee (`over` is already clamped at 0 there), so the shadow floor,
//     the whole midtone range and the r2 fix are still bit-identical;
//   * quadratic at small `over` with zero slope at 0, so it is C1 where it joins — no
//     contour ring at the hand-off, which a `max(m - SPEC, 0)` linear escape would give;
//   * asymptotically linear with slope G, so it *does* reach ACES saturation at a finite,
//     physically reachable radiance;
//   * monotonic everywhere (derivative G*(over^2 + 2*over*Q)/(over+Q)^2 > 0) — verified
//     numerically over m in [0, 5000] at 0.01 steps, 0 non-monotonic samples.
//
// G = 0.035, Q = 300 solved so that paper white lands at m = 595. Where that number comes
// from: sky.js clamps its solar disc at 2200 linear (sky.js:1100) precisely so it cannot
// write Inf into this buffer, so the sun's own radiance arrives here at ~2530 at the
// measured working exposure. 595 is 4.2x below that — i.e. anything within about two stops
// of a mirror reflection of the sun (a chrome stanchion at the mirror angle, water glitter,
// a wet-rock or rotor glint) reaches 255, and a 4% dielectric reflection of the sun does not.
// That is the correct discrimination and it is made in radiance, not by taste.
//
// The whole re-tune, measured end to end:
//              m=7    m=15   m=50   m=100  m=200  m=400  m=600  m=2530 (sun disc)
//   r5 curve   230.3  236.1  242.0  244.3  246.0  247.3  248.0  249.8
//   with this  230.4  236.2  243.0  246.6  250.3  253.4  254.5  255.0
// The measured peak of the entire r6 set is m = 50, so the set moves by ONE level at its
// brightest pixel and by less than 0.3 of a level anywhere below m = 25. Nothing in the set
// clips: it takes 595 units and the brightest thing in sixteen frames is 50. The proof that
// the sun disc is in none of them is in the review's own figure — the disc renders at L=249.8
// through the r5 curve, and no channel in the set reaches 249.
const HL_SPEC_GAIN = 0.035;
const HL_SPEC_SOFT = 300.0;

// --- depth of field, far field only (r3 review, I2) ------------------------
// Nothing nearer than this is ever defocused. The chase rigs sit 5–14 m from the rider, so
// the hero, the bike, the tread under the front wheel and the next feature are all inside
// the sharp zone at every camera distance the game presents.
const DOF_FAR_START = 30.0;
// Maximum circle of confusion is reached here and held beyond.
const DOF_FAR_END = 120.0;
// Peak CoC radius, in drawing-buffer pixels. `bokehScale` is literally this number for this
// library: `step = texelSize * coc * scale`, kernel radius <= 1.
const DOF_MAX_COC_PX = 3.5;
// If the subject (the bike) is further away than DOF_FAR_START — a long-lens replay camera
// across a valley — the ramp start is pushed out so the subject can never enter the far
// field. This is r2's RC-4 defect expressed as an invariant rather than a hope.
const DOF_SUBJECT_MARGIN = 1.6;   // multiplier on subject distance
const DOF_SUBJECT_PAD = 8.0;      // metres added on top
// A far-field defocus is a *subject* effect: it exists to separate a hero from a background.
// With no hero in frame — the establishing wide, a free QA pose, the title-screen crane over
// the valley — there is nothing to separate, and putting a 3.5 px blur on an entire mountain
// is precisely the "smeared clouds at infinity" defect r2 removed. So the pass is gated on a
// real subject being in front of the lens and close enough for the shot to be about it.
// Hysteresis, because this gate rebuilds the output pass when it flips.
const DOF_SUBJECT_ON = 55.0;
const DOF_SUBJECT_OFF = 70.0;

// --- highlight shoulder (r3 review I1, REWRITTEN by r5 review) -------------
// Applied in FilmEffect, in the sqrt-approximated display space the grain already uses, so
// these numbers are directly comparable to the review's measured display percentiles.
//
// What was here was NOT a shoulder. `f(x) = x + smoothstep(knee, white, x) * (1 - x)` with
// (0.90, 0.985) is a highlight *expander*: it maps display 0.94 to 0.967 and everything at
// or above 0.985 to exactly 1.0. On the r5 set, where a large fraction of every open frame
// already sat in the 0.90-1.00 band, that curve was itself manufacturing clipped pixels —
// it is the second-largest contributor to the 24-38% measured in the twelve open shots,
// behind the sky arriving over ACES's clip point in the first place.
//
// It is replaced by two curves, chosen per frame from the same adapted-luminance signal:
//
//   OPEN (a frame with light in it) — a genuine roll-off. Display input is bounded to 1.0
//   by the tone map, so a concave curve through the knee can never reach 1.0 at input 1.0;
//   the only shape that both sits *below* identity through the shoulder band and still
//   passes through (1,1) is a dip: f(x) = x - D * B(t)^2 with B(t) = 4t(1 - t) and
//   t = (x - knee)/(1 - knee). B and its square vanish with zero slope at both ends, so the
//   curve is C1 at the knee and at white, and it is monotonic for D < (1-knee)/3.08
//   (max|B B'| = 1.540, so f' >= 1 - 3.08 D/(1-knee); at 0.80/0.035 the flattest point is
//   f' = 0.46, checked numerically over 2000 samples).
//   Everything in the shoulder band comes *down* (0.90 -> 0.865, 0.96 -> 0.946) and only a
//   true 1.0 stays 1.0.
//
//   FLAT (a forest interior with no specular in it) — the r3 I1 lift, but bounded. The old
//   version mapped everything above its white point to exactly 1.0, which is a hard clip
//   wearing a shoulder's clothes; with r5_07's canopy holes landing at display ~0.95 it
//   would have held that shot at 7% clipped on its own. `gain < 1` gives f(1) = 1 and
//   f(x) < 1 for every x < 1, so it can add separation to a flat frame without inventing a
//   clipped pixel anywhere.
const SHOULDER_KNEE_OPEN = 0.80;
const SHOULDER_ROLLOFF_OPEN = 0.035;   // peak depth of the dip, at display 0.90
// Solved rather than tasted: with the HDR roll-off in front of it, (0.64, 0.30) is the
// strongest lift that still keeps a real margin between the canopy holes and the top of the
// range. Re-derived numerically through the whole chain for r6, because the figure that used
// to be quoted here was computed without FilmEffect's own curve in the walk and was wrong by
// two orders of magnitude. On the FLAT branch a pixel needs **85** units of post-exposure
// radiance to exceed L=250 (174 before the specular escape landed) and **538** to reach 255.
// The canopy holes in 07-forest-loam sit at 7-15 units and land at L=242-246 either way, so
// the shot that has to come in under 2% clipped comes in at 0% even when the metering
// classifies it as flat.
const SHOULDER_KNEE_FLAT = 0.64;
const SHOULDER_LIFT_FLAT = 0.30;       // < 1, so nothing below 1.0 is pushed to 1.0
// Adapted-scene-luminance window that selects between them, in true (normalised) metered
// units. Measured on the r5 set the weighted mean runs ~0.016 in the enclosed shots
// (13-bike-side, 15-bike-low-rear: p50 L=60/47, 0.00% clipped, no white point) and 0.14-0.22
// in the open ones. This window puts the enclosed shots fully on the lift and every shot
// that is currently clipping fully on the roll-off.
const SHOULDER_METER_LO = 0.030;
const SHOULDER_METER_HI = 0.140;
// Ordered dither at the final 8-bit stage, in LSBs (r3 review, I5). Half a level either way.
const DITHER_LSB = 0.5;

// --- exposure controller (r5 review) ---------------------------------------
// The r3 clamp was [0.35, 3.00]. Re-derived against the r5 set by inverting ACES on each
// shot's measured median: the controller runs ~1.15 in 00-establishing and pins hard at the
// ceiling (solved compensation 3.7 and 4.3) in 13-bike-side and 15-bike-low-rear. So the
// *floor* has never been within 0.6 stop of anything the scene asks for — it is 1.4 stops
// of unexamined range whose only function is to hide a metering pathology, and hiding one is
// how r5 happened. The ceiling, by contrast, is doing real work: it is what stops the two
// enclosed shots going black, and lowering it far would re-open r2's shadow-floor P0.
//   floor 0.35 -> 0.55  (still 0.8 stop below the brightest frame's requirement)
//   ceiling 3.00 -> 2.75 (0.13 stop off the two frames that reach it; both measure 0.00%
//                         crushed at p50 L=60/47, so the cost is a couple of levels)
// A 2.32-stop window instead of a 3.10-stop one, centred a quarter of a stop above the
// exposure the scene is authored for.
const EXPOSURE_MIN = 0.55;
const EXPOSURE_MAX = 2.75;
const EXPOSURE_KEY = 0.30;
const EXPOSURE_STRENGTH = 0.45;
// ~2 s time constant. Slow, like an eye.
const ADAPT_RATE = 0.5;

// --- camera cuts -----------------------------------------------------------
// An eye adapts over seconds; a *cut* does not, because the eye never saw the old frame.
// Without this, every hard camera change — replay angle, photo mode, respawn, and every
// pose in the QA harness — renders at the previous shot's exposure. Measured consequence on
// the r5 set: `shoot()` gives the composer 7 frames at dt = 1/60, and at ADAPT_RATE the
// adapted luminance moves 5.7% in that time, so all sixteen shots were captured at
// substantially ONE exposure. That is why the same build produced 38% clipped in
// 01-start-gate and a p50 of 47 in 15-bike-low-rear.
const CUT_DISTANCE = 20.0;     // metres of camera translation in a single frame
const CUT_DOT = 0.5;           // view-direction dot below this is a cut (~60 degrees)
const ADAPT_RATE_CUT = 60.0;   // ~1/60 s time constant: converged inside ~7 frames
const CUT_HOLD = 0.20;         // seconds to hold the fast rate after a cut

// ---------------------------------------------------------------------------
// Quality tiers. `high` is the default and must look stunning (CONTRACT §9).
// ---------------------------------------------------------------------------
//
// AO budget note (r2 review, P2): the AO settings below are deliberately cheap. The
// instrumented A/B put N8AO at 5.44 ms of a 23.4 ms frame for an effect that could not be
// found in any of the fourteen review shots. Half-res + 16 samples + a single 4-tap
// denoise iteration recovers ~4.8 ms of that. `ultra` keeps the expensive settings.
//
// DoF resolution scale (r3 review, I2): the field is now far-only and therefore
// low-frequency by construction — beyond 30 m there is nothing in this scene with detail
// finer than the blur kernel itself. r2's objection to 0.5 was about a *near* field over
// the whole frame, where the bokeh kernel ended up wider than the buffer it sampled. Only
// the intermediate bokeh target is scaled (`renderTargetFar` and the CoC targets stay full
// resolution), so this halves the two convolution passes and nothing else.
const TIERS = {
  low: {
    aoSamples: 8, aoDenoise: 4, aoDenoiseIterations: 1, aoHalfRes: true,
    bloomLevels: 4,
    dofScale: 0.5,
    mbSamples: 6,
    smaa: SMAAPreset.LOW,
    msaa: 0,
    lutSize: 17,
    tetrahedralLUT: false,
  },
  medium: {
    aoSamples: 12, aoDenoise: 4, aoDenoiseIterations: 1, aoHalfRes: true,
    bloomLevels: 5,
    dofScale: 0.5,
    mbSamples: 8,
    smaa: SMAAPreset.MEDIUM,
    msaa: 0,
    lutSize: 25,
    tetrahedralLUT: false,
  },
  high: {
    aoSamples: 16, aoDenoise: 4, aoDenoiseIterations: 1, aoHalfRes: true,
    bloomLevels: 5,
    dofScale: 0.5,
    // r3 review, I4: 12 taps at full res, each doing a colour fetch AND a depth fetch, is 24
    // dependent fetches per pixel over the whole frame for an effect that is by construction
    // low-frequency. 10 taps, depth on every second one, and a per-pixel early-out keyed to
    // the actual streak length in pixels.
    mbSamples: 10,
    smaa: SMAAPreset.HIGH,
    // r6 review, item 1 — 2x MSAA at `high`, the line vegetation.js asked for three rounds
    // ago (vegetation.js:3751-3758) and nothing here ever gave it.
    //
    // Why SMAA cannot cover this and MSAA can. SMAA is *morphological*: it re-shapes edges
    // it can find in the final image, it runs last (after grain and chromatic aberration),
    // and its `edgeDetectionThreshold` had to be raised to 0.06 so it would not fire on the
    // film grain sitting in front of it. That threshold is a 6%-contrast floor on every edge
    // in the frame, and it makes the pass structurally blind to the thing that is actually
    // aliasing here: an alpha-tested needle silhouette against sky, where the *shading* on
    // either side of the edge differs by far less than 6% but the coverage goes 0 -> 1 in
    // one pixel. Morphological AA also cannot invent the sub-pixel coverage a 5.7 px needle
    // stroke needs; it can only smooth the staircase it already has.
    //
    // MSAA solves exactly that case and only that case: the fragment shader still runs once
    // per pixel (so this is NOT 2x the shading cost), and with `alphaToCoverage` on the
    // foliage materials the alpha test resolves into 2 sub-samples instead of a binary
    // keep/discard. vegetation.js:4792-4815 re-reads `composer.multisampling` on every
    // quality change and flips `alphaToCoverage` on the five foliage materials by itself —
    // that wiring has been finished and idle since r3, so this constant is the whole change.
    //
    // Cost, honestly, and checked against the library rather than assumed. I wanted to write
    // "only the scene target is multisampled"; it is not true. `EffectComposer`'s constructor
    // builds `inputBuffer` with the requested sample count and then does
    // `outputBuffer = inputBuffer.clone()` (postprocessing/build/index.js, EffectComposer
    // ctor), so BOTH ping-pong buffers carry it and every pass transition in the chain ends
    // up resolving one. At 1920x1080 RGBA16F + depth that is:
    //   memory   +16.6 MB colour and +8.3 MB depth per buffer for the multisampled
    //            attachments — call it +50 MB across the pair.
    //   time     the scene pass rasterises into a 2x target. Fragment shading is NOT doubled
    //            (MSAA multiplies coverage samples, not shader invocations) so the terrain
    //            shader's ~7 ms is untouched; what grows is colour/depth store bandwidth on
    //            the one geometry pass, plus a full-target resolve at each of the 5-6 pass
    //            transitions at roughly 0.2 ms each. Honest estimate: **+1.5 to +2.5 ms** at
    //            1080p, i.e. 6-12% of the 20-25 ms shipping frame the final verdict derives
    //            in §6 — not of the harness's 11.11 ms, which times `draw()` alone.
    // That is a real price and it is worth stating plainly: this is the most expensive of the
    // three changes in this file by an order of magnitude. It is still the right trade —
    // foliage occupies 30-50% of pixels in nine of sixteen shots and its silhouette is the
    // single most-visible aliasing in the set — but if the target machine cannot afford it,
    // the knob is this constant and nothing else. There is no cheaper half: decoupling the
    // scene target's sample count from the chain's would be a change to `postprocessing`,
    // not to a number here.
    // `ultra` already ran msaa: 4 through this exact path, N8AO's depth consumer included, so
    // the multisampled-target-with-depth-texture case is exercised, not new.
    msaa: 2,
    lutSize: 33,
    tetrahedralLUT: false,
  },
  ultra: {
    aoSamples: 32, aoDenoise: 8, aoDenoiseIterations: 2, aoHalfRes: false,
    bloomLevels: 6,
    dofScale: 1.0,
    mbSamples: 18,
    smaa: SMAAPreset.ULTRA,
    msaa: 4,
    lutSize: 33,
    tetrahedralLUT: true,
  },
};

// ===========================================================================
// 1. Procedural film grading LUT
// ===========================================================================
//
// Authored in *display* space (the LUT3DEffect converts the linear working colour to
// sRGB before the lookup and back afterwards), because tonal curves, lift/gamma/gain and
// split toning are all defined perceptually. Every step is deliberately small: a grade
// that reads as "a grade" is a grade that has gone too far.

// ASC-CDL style slope / offset / power. Warm gain, cool lifted shadows.
//
// r2 review, P0-c — WHITE POINT. Measured across the review set: red reached 255 while
// blue topped out at 248–253, i.e. *no pixel in fourteen 1080p frames was white*. Two
// causes, both fixed here:
//   (i)  the CDL was applied un-normalised, so `1.0` in mapped to `slope + offset` out —
//        1.020 on red (clipped a stop early by the old sCurve's clamp01) and 0.9924 on
//        blue (which can therefore never reach 1.0 no matter what follows).
//   (ii) the amber highlight split-tone was applied at *full* strength at the white point,
//        so highlights diverged to yellow instead of converging to white — the exact
//        inverse of what every film-emulation curve is built to do.
// The CDL is therefore normalised by its own value at white: `((x*S + O) / (S + O))^P`.
// That keeps the whole point of the grade — a cool lifted toe, a warm shoulder — while
// making `f(1) == 1` exactly, per channel, by construction. Verified by
// `verifyGradingLUTWhitePoint()` below, which runs at boot.
const CDL_SLOPE = [1.016, 1.000, 0.980];
const CDL_OFFSET = [0.0040, 0.0062, 0.0125];
const CDL_POWER = [1.000, 0.992, 1.018];
// Per-channel value of `slope * 1 + offset`. Dividing by this is what pins white.
const CDL_WHITE = [
  CDL_SLOPE[0] + CDL_OFFSET[0],
  CDL_SLOPE[1] + CDL_OFFSET[1],
  CDL_SLOPE[2] + CDL_OFFSET[2],
];

// Split tone. Shadows drift teal-green (the contract asks for lifted greens in shadow),
// highlights drift amber. Magnitudes are in display units — 0.012 is ~3/255, which is
// where a split tone should live. All shadow components stay positive so no channel is
// driven to a hard zero, which would clip detail out of the darks.
//
// r2 review, P1-d: the negative blue term is gone. It was pulling every bright sky pixel
// toward yellow, and the measured skies came back (225,233,237)–(245,243,240): white, not
// blue. The highlight tint is now weighted by a *band* that dies at both ends (see
// `hiBand` below), so it colours the shoulder and leaves the white point alone.
const SHADOW_TINT = [0.002, 0.006, 0.011];
const HIGHLIGHT_TINT = [0.010, 0.003, 0.000];

// r2 review, P1-a: `S_CURVE_AMOUNT = 0.26` is DELETED. ACES already owns the tonal curve;
// stacking a second smoothstep on top of its toe is half of why `r2_07` measured 90.8%
// below 20 IRE with 13% of pixels under L=6. What replaces it is a toe *lift*, which is
// the half of a film curve the image was actually missing: a print black is not zero.
// `SHADOW_LIFT` is the display-space value that maps to input 0. 0.030 ≈ 7.6/255.
const SHADOW_LIFT = 0.030;

const MID_SAT_BOOST = 0.16;    // midtone saturation
const GLOBAL_SAT = 1.03;
// r2 review, P1-d: 0.30 with a 0.78 onset was bleaching the entire upper sky to white.
// 0.10 with a 0.90 onset confines it to genuine speculars and the sun's surround.
const HIGHLIGHT_BLEACH = 0.10;
const BLEACH_ONSET = 0.90;

/**
 * Normalised ASC-CDL. `f(0) = (O/(S+O))^P` (the lifted, tinted toe) and `f(1) = 1`
 * exactly — the division and the pow are both exact at x = 1.
 */
function cdl(x, i) {
  const v = Math.max(x * CDL_SLOPE[i] + CDL_OFFSET[i], 0) / CDL_WHITE[i];
  return CDL_POWER[i] === 1 ? v : Math.pow(v, CDL_POWER[i]);
}

/**
 * Filmic toe lift. Raises the darks toward a print black without touching the top:
 * `f(0) = LIFT`, `f(1) = 1` exactly, monotonic for LIFT < 1/3. Values above 1 (grading
 * headroom) pass through untouched, which is why the `max(0, ...)` is there.
 */
function toeLift(x, lift) {
  const t = Math.max(0, 1 - x);
  return x + lift * t * t * t;
}

/**
 * The grade itself, one display-referred RGB triple in, one out. Split out of the cube
 * builder so the white-point assertion can evaluate the exact same code path.
 * Returns into `out` (a 3-element array) to stay allocation-free.
 */
function gradeRGB(r0, g0, b0, out) {
  // --- 1. lift / gamma / gain (normalised, so white stays white) ---------
  let r = cdl(r0, 0);
  let g = cdl(g0, 1);
  let b = cdl(b0, 2);

  // --- 2. toe lift (replaces the stacked S-curve) ------------------------
  r = toeLift(r, SHADOW_LIFT);
  g = toeLift(g, SHADOW_LIFT);
  b = toeLift(b, SHADOW_LIFT);

  // --- 3. split tone ----------------------------------------------------
  let luma = 0.2126 * r + 0.7152 * g + 0.0722 * b;
  const shadowW = 1 - smoothstep(0.0, 0.42, luma);
  const highW = smoothstep(0.48, 1.0, luma);
  // Highlight tint as a band, not a ramp: `4·w·(1-w)` peaks at exactly 1.0 in the middle
  // of the shoulder (luma ≈ 0.74) and is exactly 0 at the white point, so the authored
  // magnitude still means what it says but the tint cannot survive to 1.0.
  const hiBand = 4 * highW * (1 - highW);
  r += SHADOW_TINT[0] * shadowW + HIGHLIGHT_TINT[0] * hiBand;
  g += SHADOW_TINT[1] * shadowW + HIGHLIGHT_TINT[1] * hiBand;
  b += SHADOW_TINT[2] * shadowW + HIGHLIGHT_TINT[2] * hiBand;

  // --- 4. saturation, biased to the midtones ----------------------------
  //
  // r5 review — the "does the LUT clip a stop early?" audit. The white point is exactly
  // (1,1,1) and the CDL is normalised, but saturation > 1 raises the *max channel* of every
  // colour it touches: a sky at (0.55, 0.75, 0.95) came out of this step at b = 0.9825, i.e.
  // a channel driven 8 levels closer to 255 by the grade alone, and a fraction of a stop
  // higher and it would have been clamped by the `clamp01` at the cube write — clipped a
  // stop early, exactly as suspected. The fix is not to drop the saturation (it is doing
  // real work in the midtones) but to forbid it from raising the maximum: after saturating,
  // rescale the deviation from luma so the largest channel lands no higher than the largest
  // channel that came in. Below the highlights this is inert — the max only rises when the
  // colour is already close to the top.
  luma = 0.2126 * r + 0.7152 * g + 0.0722 * b;
  const maxIn = Math.max(r, Math.max(g, b));
  const midW = 1 - Math.pow(Math.abs(clamp01(luma) * 2 - 1), 1.5);
  const sat = GLOBAL_SAT + MID_SAT_BOOST * midW;
  r = luma + (r - luma) * sat;
  g = luma + (g - luma) * sat;
  b = luma + (b - luma) * sat;
  const maxOut = Math.max(r, Math.max(g, b));
  // Ramped in over 0.60→0.92 so the midtone boost keeps its full authority where it belongs
  // and loses it entirely as the colour approaches the top of the range.
  const guard = smoothstep(0.60, 0.92, maxIn);
  const maxAllowed = lerp(maxOut, maxIn, guard);
  if (maxOut > maxAllowed && maxOut > luma) {
    const k = (maxAllowed - luma) / (maxOut - luma);
    r = luma + (r - luma) * k;
    g = luma + (g - luma) * k;
    b = luma + (b - luma) * k;
  }

  // --- 5. highlight bleach ----------------------------------------------
  const bleach = HIGHLIGHT_BLEACH * smoothstep(BLEACH_ONSET, 1.0, luma);
  out[0] = lerp(r, luma, bleach);
  out[1] = lerp(g, luma, bleach);
  out[2] = lerp(b, luma, bleach);
  return out;
}

const _gradeScratch = [0, 0, 0];

/**
 * Build the 33³ (or smaller) grading cube. Values in and out are display-referred.
 * Runs once at boot; ~36 k iterations of cheap maths, well under a frame.
 */
function buildGradingLUT(size) {
  const data = new Float32Array(size * size * size * 4);
  const sizeSq = size * size;
  const last = size - 1;

  for (let bi = 0; bi < size; bi++) {
    for (let gi = 0; gi < size; gi++) {
      for (let ri = 0; ri < size; ri++) {
        // Divide rather than multiply by a precomputed step: `last / last` is exactly
        // 1.0 for every size, `last * (1/last)` is not (try size = 25).
        const out = gradeRGB(ri / last, gi / last, bi / last, _gradeScratch);
        const i4 = (ri + gi * size + bi * sizeSq) * 4;
        data[i4 + 0] = clamp01(out[0]);
        data[i4 + 1] = clamp01(out[1]);
        data[i4 + 2] = clamp01(out[2]);
        data[i4 + 3] = 1;
      }
    }
  }

  const lut = new LookupTexture(data, size);
  lut.name = 'descent.filmGrade';
  lut.userData.whitePoint = verifyGradingLUTWhitePoint(lut, size);
  return lut;
}

/**
 * P0-c assertion. The cube's top corner — the entry at (size-1, size-1, size-1), i.e. the
 * one every white pixel in the frame lands on — must be exactly (1, 1, 1). If it is not,
 * nothing in the game can ever render a white pixel, and the whole image inherits a tint
 * that is invisible on a waveform and obvious in a screenshot.
 *
 * Reads the built Float32Array, i.e. the bytes the GPU will actually sample, not an
 * idealised recomputation. Exported so a unit test can call it directly:
 *
 *   import { verifyGradingLUTWhitePoint, buildGradingLUTForTest } from 'src/core/postfx.js';
 *   const { lut, size } = buildGradingLUTForTest(33);
 *   assert.deepEqual(verifyGradingLUTWhitePoint(lut, size).rgb, [1, 1, 1]);
 *
 * @param {LookupTexture} lut - a cube built by `buildGradingLUT`.
 * @param {Number} size - the cube's edge length.
 * @return {{ok: Boolean, rgb: Number[]}}
 */
export function verifyGradingLUTWhitePoint(lut, size) {
  const data = lut && lut.image && lut.image.data;
  if (!data) return { ok: false, rgb: [NaN, NaN, NaN] };
  const last = size - 1;
  const i4 = (last + last * size + last * size * size) * 4;
  const rgb = [data[i4 + 0], data[i4 + 1], data[i4 + 2]];
  const ok = rgb[0] === 1 && rgb[1] === 1 && rgb[2] === 1;
  if (!ok) {
    console.error(
      `[postfx] grading LUT white point is not white: (${rgb[0]}, ${rgb[1]}, ${rgb[2]}). ` +
      'No pixel in the game can render as 255,255,255. Check CDL normalisation and the ' +
      'highlight split-tone band in gradeRGB().');
  }
  return { ok, rgb };
}

/** Test seam: build a cube without touching the renderer or the composer. */
export function buildGradingLUTForTest(size = 33) {
  return { lut: buildGradingLUT(size), size };
}

// ===========================================================================
// 2. Custom effects
// ===========================================================================
//
// (The procedural sun-sprite texture and its light-source mesh lived here. They existed
// only to feed `GodRaysEffect`, which is deleted for this milestone — see the CONTRACT-NOTE
// at the top of the file. `sky.js` draws the visible sun disc.)

// --- 2a. Adaptive exposure -------------------------------------------------

const exposureFrag = /* glsl */`
uniform lowp sampler2D luminanceBuffer;
uniform vec4 exposureRange;   // x = base, y = key, z = min, w = max
uniform float adaptStrength;
uniform float meterNorm;      // E[w] of the metering weight field; turns the raw mean into
                              // a true weighted mean (see computeMeterNorm)
uniform vec4 rolloff;         // x = knee, y = softness, z = specular gain, w = specular soft
                              // (all in post-exposure linear radiance)

void mainImage(const in vec4 inputColor, const in vec2 uv, out vec4 outputColor) {

	float e = exposureRange.x;

	#ifdef ADAPTIVE
		// The adapted luminance is packed into RGBA by AdaptiveLuminancePass and is
		// therefore limited to [0,1] — which is fine, and in fact required: the metering
		// shader packs a level term and a bounded highlight term into this one channel.
		float avg = unpackRGBAToFloat(texture2D(luminanceBuffer, vec2(0.5))) / meterNorm;
		avg = max(avg, 0.012);
		// Partial compensation (adaptStrength < 1) so a dark forest still reads dark and
		// an open sunlit face still reads bright — the eye does not fully normalise.
		float comp = pow(exposureRange.y / avg, adaptStrength);
		e = clamp(exposureRange.x * comp, exposureRange.z, exposureRange.w);
	#endif

	vec3 c = inputColor.rgb * e;

	// --- HDR highlight roll-off (r5 review) -----------------------------
	// y = K + S*ln(1 + (m - K)/S): identity below the knee, C1 across it, logarithmic
	// above. Ratio-preserving on the max channel so chromaticity survives the compression
	// — a blue sky comes down as blue, not as white. This is what has to happen *here*
	// rather than after the tone map: ACES saturates at an input of 25.7 and everything
	// above that is one identical clipped value by the time any display curve can see it.
	// The specular escape (rolloff.z) is what gives the frame a reachable white point: it is
	// exactly zero below the knee, quadratic-with-zero-slope as it leaves it, and linear far
	// above, so the sun's disc and a mirror-angle glint saturate ACES while the sky at 7-15
	// units moves by a tenth of a level. Written as over * (over / (over + Q)) rather than the
	// algebraically identical over*over / (over + Q) deliberately: the second form squares a
	// radiance that runs to ~2500 here, which is 6.3e6 and overflows a mediump float. This
	// form never holds an intermediate larger than over itself.
	if(rolloff.y > 0.0) {
		float m = max(max(c.r, c.g), c.b);
		float over = max(m - rolloff.x, 0.0);
		float y = m - over + rolloff.y * log(1.0 + over / rolloff.y);
		y += rolloff.z * over * (over / (over + rolloff.w));
		c *= y / max(m, 1e-5);
	}

	outputColor = vec4(c, inputColor.a);

}`;

// Metering (r5 review). Replaces `LuminanceMaterial`'s fragment shader outright.
//
// r3 cropped the sampled region to the lower 75% of the frame by rewriting the *vertex*
// shader, on the correct reasoning that a fragment mask writing zeros would drag the
// mip-chained mean toward black rather than excluding the region. The reasoning is right and
// the conclusion was wrong: the way to weight a mean without biasing it is to divide by the
// mean of the weight field, which is a constant for a fixed field and is computed once on the
// CPU by `computeMeterNorm()`. So the crop becomes a weight, the sky comes back into the
// controller's view at low weight instead of being invisible, and a second, bounded term
// reports how much of the frame is actually in danger of clipping.
//
// The exposure transfer function is recomputed here from last frame's adapted value so the
// highlight term can be evaluated on *exposed* radiance. That closes the loop (more exposure
// -> more highlight -> less exposure) with a loop gain well under unity and a 2 s time
// constant on top, so it settles rather than oscillates.
const meteringFrag = /* glsl */`
#include <common>
#include <packing>
#define unpackRGBAToFloat(v) unpackRGBAToDepth(v)
// mediump unconditionally, not the library's FRAMEBUFFER_PRECISION_HIGH switch. LuminancePass
// has no initialize(), so that define is never set and the library's own shader takes the
// lowp sampler2D branch — and ES3 only guarantees lowp a range of [-2, 2]. The scene buffer
// is HalfFloatType and the highlight term below has to read radiances up to ~15, so a sampler
// that saturates at 2 would silently blind the very term that exists to see the sky.
uniform mediump sampler2D inputBuffer;
uniform mediump sampler2D adaptedLuminance;
uniform vec4 exposureRange;    // shared, by reference, with AdaptiveExposureEffect
uniform float adaptStrength;
uniform float meterNorm;
uniform vec4 meterWeights;     // x = sky weight, y = edge weight, z = level cap, w unused
uniform vec4 meterShape;       // x,y = sky ramp over uv.y   z,w = edge ramp over |uv.x-0.5|
uniform vec3 meterHighlight;   // x = knee, y = top, z = gain   (post-exposure linear)
varying vec2 vUv;

void main() {

	float l = luminance(texture2D(inputBuffer, vUv).rgb);

	// Centre-weighted, sky down-weighted but never zero. uv.y = 0 is the bottom.
	float sky = smoothstep(meterShape.x, meterShape.y, vUv.y);
	float w = mix(1.0, meterWeights.x, sky);
	float edge = smoothstep(meterShape.z, meterShape.w, abs(vUv.x - 0.5));
	w *= mix(1.0, meterWeights.y, edge);

	// Level: capped per pixel so no one region can run away with the mean.
	float level = w * min(l, meterWeights.z);

	// Highlight: what fraction of the frame is inside the top stop of the display range at
	// the exposure this frame is about to be rendered at.
	float avgPrev = max(unpackRGBAToFloat(texture2D(adaptedLuminance, vec2(0.5))) / meterNorm, 0.012);
	float e = clamp(exposureRange.x * pow(exposureRange.y / avgPrev, adaptStrength),
	                exposureRange.z, exposureRange.w);
	float over = meterHighlight.z * smoothstep(meterHighlight.x, meterHighlight.y, l * e);

	// The target is 8-bit; the two terms are sized so their sum cannot reach 1.0.
	gl_FragColor = vec4(level + over);

}`;

/**
 * E[w] over the metering weight field, i.e. the constant that turns the raw mip-chained mean
 * of `w * l` into a true weighted mean of `l`. Integrated numerically from the same constants
 * the shader uses, so retuning the field cannot silently rescale the exposure.
 *
 * @return {Number}
 */
function computeMeterNorm() {
  const N = 128;
  let sum = 0;
  for (let j = 0; j < N; j++) {
    const y = (j + 0.5) / N;
    const wy = lerp(1, METER_SKY_WEIGHT, smoothstep(METER_SKY_LO, METER_SKY_HI, y));
    for (let i = 0; i < N; i++) {
      const x = (i + 0.5) / N;
      const wx = lerp(1, METER_EDGE_WEIGHT, smoothstep(METER_EDGE_LO, METER_EDGE_HI, Math.abs(x - 0.5)));
      sum += wy * wx;
    }
  }
  return sum / (N * N);
}

const METER_NORM = computeMeterNorm();

class AdaptiveExposureEffect extends Effect {
  constructor({ base = 1.0, key = EXPOSURE_KEY, min = EXPOSURE_MIN, max = EXPOSURE_MAX,
                strength = EXPOSURE_STRENGTH, adaptationRate = ADAPT_RATE, resolution = 256,
                adaptive = true, weightedMetering = true,
                rolloffKnee = HL_KNEE, rolloffSoft = HL_SOFT,
                rolloffSpecGain = HL_SPEC_GAIN, rolloffSpecSoft = HL_SPEC_SOFT } = {}) {
    super('AdaptiveExposureEffect', exposureFrag, {
      blendFunction: BlendFunction.SRC,
      uniforms: new Map([
        ['luminanceBuffer', new THREE.Uniform(null)],
        ['exposureRange', new THREE.Uniform(new THREE.Vector4(base, key, min, max))],
        ['adaptStrength', new THREE.Uniform(strength)],
        ['meterNorm', new THREE.Uniform(1)],
        ['rolloff', new THREE.Uniform(new THREE.Vector4(
          rolloffKnee, rolloffSoft, rolloffSpecGain, Math.max(rolloffSpecSoft, 1e-3)))],
      ]),
    });

    // 256×256 luminance, mip-chained down to 1×1, then temporally adapted.
    this.renderTargetLuminance = new THREE.WebGLRenderTarget(1, 1, {
      minFilter: THREE.LinearMipmapLinearFilter,
      magFilter: THREE.LinearFilter,
      depthBuffer: false,
    });
    this.renderTargetLuminance.texture.generateMipmaps = true;
    this.renderTargetLuminance.texture.name = 'Exposure.Luminance';

    this.luminancePass = new LuminancePass({ renderTarget: this.renderTargetLuminance });

    this.adaptiveLuminancePass = new AdaptiveLuminancePass(this.luminancePass.texture, {
      minLuminance: 0.012,
      adaptationRate,
    });
    this.uniforms.get('luminanceBuffer').value = this.adaptiveLuminancePass.texture;

    // r5 review: weighted, highlight-aware metering. Replaces r3's hard upper-25% crop.
    // Guarded — if a future `postprocessing` reshapes LuminanceMaterial this degrades to the
    // library's own unweighted full-frame mean (meterNorm stays 1, so the exposure is still
    // correctly scaled) rather than producing a black frame.
    this.weightedMetering = false;
    if (weightedMetering) {
      try {
        const lm = this.luminancePass.fullscreenMaterial;
        // Shared *by reference* with this effect's own uniforms, so the metering shader
        // always evaluates the exact transfer function the exposure shader will apply —
        // including the crash dip, which writes exposureRange.x every frame.
        lm.uniforms.exposureRange = this.uniforms.get('exposureRange');
        lm.uniforms.adaptStrength = this.uniforms.get('adaptStrength');
        lm.uniforms.meterNorm = this.uniforms.get('meterNorm');
        lm.uniforms.adaptedLuminance = new THREE.Uniform(this.adaptiveLuminancePass.texture);
        lm.uniforms.meterWeights = new THREE.Uniform(new THREE.Vector4(
          METER_SKY_WEIGHT, METER_EDGE_WEIGHT, METER_LEVEL_CAP, 0));
        lm.uniforms.meterShape = new THREE.Uniform(new THREE.Vector4(
          METER_SKY_LO, METER_SKY_HI, METER_EDGE_LO, METER_EDGE_HI));
        lm.uniforms.meterHighlight = new THREE.Uniform(new THREE.Vector3(
          METER_HI_KNEE, METER_HI_TOP, METER_HI_GAIN));
        lm.fragmentShader = meteringFrag;
        lm.needsUpdate = true;
        this.uniforms.get('meterNorm').value = METER_NORM;
        this.weightedMetering = true;
      } catch (err) {
        console.warn('[postfx] weighted metering unavailable, metering full frame:', err);
        this.uniforms.get('meterNorm').value = 1;
      }
    }

    const exponent = Math.max(0, Math.ceil(Math.log2(resolution)));
    this.luminancePass.resolution.setPreferredSize(2 ** exponent, 2 ** exponent);
    this.adaptiveLuminancePass.fullscreenMaterial.mipLevel1x1 = exponent;

    this.adaptive = adaptive;
  }

  get adaptive() { return this.defines.has('ADAPTIVE'); }

  set adaptive(value) {
    if (this.adaptive === !!value) return;
    if (value) this.defines.set('ADAPTIVE', '1');
    else this.defines.delete('ADAPTIVE');
    this.adaptiveLuminancePass.enabled = !!value;
    this.setChanged();
  }

  /** Base exposure, i.e. the exposure with a perfectly "average" frame. */
  get exposure() { return this.uniforms.get('exposureRange').value.x; }
  set exposure(v) { this.uniforms.get('exposureRange').value.x = v; }

  /** (base, key, min, max) — shared by reference with the metering material. */
  get range() { return this.uniforms.get('exposureRange').value; }
  /** E[w] of the metering weight field. 1 when the weighted-metering patch did not apply. */
  get meterNorm() { return this.uniforms.get('meterNorm').value; }
  /**
   * HDR highlight roll-off, as (knee, softness, specular gain, specular softness).
   * Set y to 0 to disable the whole block — note that also disables the specular escape,
   * which restores hard ACES clipping and is the intended meaning of "off".
   */
  get rolloff() { return this.uniforms.get('rolloff').value; }

  get adaptationRate() { return this.adaptiveLuminancePass.fullscreenMaterial.adaptationRate; }
  set adaptationRate(v) { this.adaptiveLuminancePass.fullscreenMaterial.adaptationRate = v; }

  update(renderer, inputBuffer, deltaTime) {
    if (!this.adaptiveLuminancePass.enabled) return;
    this.luminancePass.render(renderer, inputBuffer);
    this.adaptiveLuminancePass.render(renderer, null, null, deltaTime);
  }

  setSize(width, height) {
    this.luminancePass.setSize(width, height);
  }

  initialize(renderer, alpha, frameBufferType) {
    this.adaptiveLuminancePass.initialize(renderer, alpha, frameBufferType);
  }

  dispose() {
    this.renderTargetLuminance.dispose();
    this.luminancePass.dispose();
    this.adaptiveLuminancePass.dispose();
    super.dispose();
  }
}

// --- 2b. Motion blur -------------------------------------------------------

const motionBlurFrag = /* glsl */`
uniform mat4 prevViewProj;
uniform mat4 invViewProj;
uniform vec4 blurConfig;    // x = shutter scale, y = max blur (uv), z = radial, w = radial start
uniform vec4 riderSphere;   // xyz = world centre of the bike, w = protect radius

float mbNoise(const in vec2 p) {
	return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

void mainImage(const in vec4 inputColor, const in vec2 uv, const in float depth, out vec4 outputColor) {

	// Depth exactly at the far plane reconstructs to w≈0; nudge it in so the sky still
	// gets a well-conditioned (rotation-dominated) velocity instead of an infinity.
	float d = min(depth, 0.99995);

	vec4 ndc = vec4(uv * 2.0 - 1.0, d * 2.0 - 1.0, 1.0);
	vec4 world = invViewProj * ndc;
	world /= world.w;

	// The bike and rider move *with* the camera, so treating them as static geometry
	// would smear them — mask them out in world space instead.
	float riderMask = smoothstep(riderSphere.w, riderSphere.w * 1.7, distance(world.xyz, riderSphere.xyz));

	vec2 velocity = vec2(0.0);
	vec4 prevClip = prevViewProj * world;
	if(prevClip.w > 1e-4) {
		vec2 prevUv = (prevClip.xy / prevClip.w) * 0.5 + 0.5;
		velocity = (uv - prevUv) * blurConfig.x;
	}
	velocity *= riderMask;

	// Speed-driven radial streak, weighted to the edges of the frame.
	vec2 fromCentre = (uv - 0.5) * vec2(aspect, 1.0);
	float radius = length(fromCentre);
	float edge = smoothstep(blurConfig.w, 1.05, radius);
	vec2 radial = fromCentre * (blurConfig.z * edge) * riderMask;
	radial.x /= aspect;

	vec2 offset = velocity + radial;
	float len = length(offset);

	if(len < 1e-5) {
		outputColor = inputColor;
		return;
	}

	if(len > blurConfig.y) {
		offset *= blurConfig.y / len;
	}

	// Per-pixel jitter breaks the ghosting that a fixed tap pattern produces.
	float jitter = mbNoise(gl_FragCoord.xy + vec2(fract(time) * 91.7)) - 0.5;

	// r3 review, I4 — tap count follows the streak, not the tier. The blur is clamped to
	// blurConfig.y (<= 0.05 uv), so over most of the frame the streak is only a few pixels
	// long and the extra taps land on the same texels. One tap per pixel of streak is the
	// Nyquist limit of the thing being reconstructed; anything more is a duplicate fetch.
	float taps = clamp(ceil(len / max(texelSize.y, 1e-6)), 2.0, float(MB_SAMPLES));

	float centreDist = -getViewZ(d);
	vec4 sum = inputColor;
	float total = 1.0;
	// Depth is sampled on every second tap and the weight is reused for its partner. The
	// rejection weight exists to stop a sharp foreground bleeding outwards; it is a
	// low-frequency function of screen position, so half the depth fetches reconstruct it
	// to well inside a quantisation step. This removes MB_SAMPLES/2 dependent fetches and
	// MB_SAMPLES/2 getViewZ evaluations per pixel.
	float w = 1.0;

	for(int i = 0; i < MB_SAMPLES; ++i) {

		if(float(i) >= taps) { break; }

		float t = (float(i) + 0.5 + jitter) / taps - 0.5;
		vec2 suv = clamp(uv + offset * t, vec2(0.0), vec2(1.0));

		if(mod(float(i), 2.0) < 0.5) {
			float sd = -getViewZ(min(readDepth(suv), 0.99995));
			w = clamp((sd - centreDist) / max(centreDist * 0.25, 0.4) + 1.0, 0.0, 1.0);
		}

		sum += texture2D(inputBuffer, suv) * w;
		total += w;

	}

	outputColor = sum / total;

}`;

class MotionBlurEffect extends Effect {
  constructor({ samples = 12, shutter = 0.85, maxBlur = 0.045 } = {}) {
    super('MotionBlurEffect', motionBlurFrag, {
      blendFunction: BlendFunction.SRC,
      attributes: EffectAttribute.CONVOLUTION | EffectAttribute.DEPTH,
      defines: new Map([['MB_SAMPLES', Math.max(2, samples | 0).toFixed(0)]]),
      uniforms: new Map([
        ['prevViewProj', new THREE.Uniform(new THREE.Matrix4())],
        ['invViewProj', new THREE.Uniform(new THREE.Matrix4())],
        ['blurConfig', new THREE.Uniform(new THREE.Vector4(shutter, maxBlur, 0, 0.35))],
        ['riderSphere', new THREE.Uniform(new THREE.Vector4(0, 0, 0, 1.6))],
      ]),
    });
    this._samples = samples;
  }

  get samples() { return this._samples; }

  set samples(value) {
    const v = Math.max(2, value | 0);
    if (v === this._samples) return;
    this._samples = v;
    this.defines.set('MB_SAMPLES', v.toFixed(0));
    this.setChanged();
  }

  get config() { return this.uniforms.get('blurConfig').value; }
  get riderSphere() { return this.uniforms.get('riderSphere').value; }

  /** Push the current and previous camera matrices. Called once per frame by postfx. */
  setCameraMatrices(invViewProjection, prevViewProjection) {
    this.uniforms.get('invViewProj').value.copy(invViewProjection);
    this.uniforms.get('prevViewProj').value.copy(prevViewProjection);
  }
}

// --- 2c. Film: grain, vignette, mood --------------------------------------

const filmFrag = /* glsl */`
uniform vec4 grainConfig;     // x = amount, y = cell size (px), z = chroma, w = seed
uniform vec4 vignetteConfig;  // x = strength, y = inner, z = outer, w = edge desat
uniform vec4 moodConfig;      // x = saturation, y = crash, z = speed, w = exposure trim
uniform vec4 shoulderConfig;  // x = knee(open), y = rolloff depth(open), z = knee(flat), w = lift(flat)
uniform vec4 shoulderMeter;   // x = meter lo, y = meter hi, z = dither (LSB), w = shoulder gain
uniform vec3 crashTint;
#ifdef SHOULDER_ADAPTIVE
uniform lowp sampler2D adaptedLuminance;
uniform float shoulderMeterNorm;
#endif

float filmHash(const in vec3 p) {
	vec3 q = fract(p * vec3(0.1031, 0.1030, 0.0973));
	q += dot(q, q.yzx + 33.33);
	return fract((q.x + q.y) * q.z);
}

void mainImage(const in vec4 inputColor, const in vec2 uv, out vec4 outputColor) {

	vec3 c = max(inputColor.rgb, vec3(0.0));

	// Exposure trim: the dip on impact, and a hair of extra light at speed.
	c *= moodConfig.w;

	const vec3 LUMA = vec3(0.2126, 0.7152, 0.0722);
	float l = dot(c, LUMA);

	// Saturation drains slightly with speed and heavily on a crash.
	c = mix(vec3(l), c, moodConfig.x);
	c = mix(c, vec3(l) * crashTint, moodConfig.y * 0.55);

	// --- vignette -------------------------------------------------------
	// A wide, soft exposure falloff with a touch of edge desaturation, the way a fast
	// lens actually behaves. Deliberately not a hard black donut: the radius is
	// normalised so r == 1 lands exactly on the corner at any aspect ratio, and the
	// outer edge of the ramp sits *past* the corner so the falloff never bottoms out.
	vec2 p = (uv - 0.5) * vec2(aspect, 1.0);
	float r = length(p) / length(vec2(aspect, 1.0) * 0.5);
	float v = 1.0 - smoothstep(vignetteConfig.y, vignetteConfig.z, r);
	float edge = 1.0 - v;
	c = mix(c, vec3(dot(c, LUMA)), edge * vignetteConfig.w);
	c *= mix(1.0, v, vignetteConfig.x);

	// Everything below works in an approximate display space (sqrt ≈ gamma 2.0). The frame is
	// re-linearised at the end. Both the shoulder and the grain belong here: they are print
	// behaviours, and authoring them against linear values puts their knees in the wrong place.
	vec3 disp = sqrt(c);

	// --- highlight shoulder (r3 review I1, rewritten r5) ----------------
	// Two curves, blended by how flat the frame is. Both are exact identity below their
	// knee, both pass through (1, 1), and NEITHER maps any input below 1.0 onto 1.0 — which
	// is the property the r3 version lacked and the reason it was manufacturing clipped
	// pixels in every open frame.
	{
		float flat01 = 0.0;
		#ifdef SHOULDER_ADAPTIVE
			float sceneLum = unpackRGBAToFloat(texture2D(adaptedLuminance, vec2(0.5)))
			               / max(shoulderMeterNorm, 1e-4);
			flat01 = (1.0 - smoothstep(shoulderMeter.x, shoulderMeter.y, sceneLum)) * shoulderMeter.w;
		#endif
		float dl = dot(disp, LUMA);

		// OPEN — roll-off. A symmetric dip through the shoulder band: B = 4t(1-t) and its
		// square both vanish with zero slope at t = 0 and t = 1, so the curve leaves the
		// identity line at the knee and rejoins it at 1.0 without a contour at either end,
		// and everything in between comes down.
		float tO = clamp((dl - shoulderConfig.x) / max(1.0 - shoulderConfig.x, 1e-3), 0.0, 1.0);
		float b = 4.0 * tO * (1.0 - tO);
		float openL = dl - shoulderConfig.y * b * b;

		// FLAT — bounded lift. The gain is < 1, so f(1) = 1 and f(x) < 1 for all x < 1: a
		// forest interior gets its top decile stretched toward white without any value
		// below white being remapped onto it.
		float flatL = dl + shoulderConfig.w * smoothstep(shoulderConfig.z, 1.0, dl) * (1.0 - dl);

		float shaped = mix(openL, flatL, flat01);
		// Scale the triple rather than curving each channel, so the shoulder cannot rotate
		// hue on a saturated highlight.
		disp *= shaped / max(dl, 1e-4);
		disp = min(disp, vec3(1.0));
	}

	// --- grain ----------------------------------------------------------
	vec2 cell = floor(gl_FragCoord.xy / max(grainConfig.y, 1.0));
	vec3 g = vec3(
		filmHash(vec3(cell, grainConfig.w)),
		filmHash(vec3(cell, grainConfig.w + 17.13)),
		filmHash(vec3(cell, grainConfig.w + 41.77))
	) - 0.5;
	g = mix(vec3((g.x + g.y + g.z) * 0.3333), g, grainConfig.z);
	float ld = dot(disp, LUMA);
	// Real grain peaks in the midtones and all but vanishes in clean blacks and whites.
	float response = 1.0 - 0.82 * abs(ld * 2.0 - 1.0);
	disp += g * grainConfig.x * response;

	// --- ordered dither, last (r3 review, I5) ---------------------------
	// +/- half a least-significant bit at 8 bits, in display space, immediately before the
	// frame is handed back for the sRGB write. Interleaved-gradient noise rather than a
	// hash: it is spatially uniform at every scale, so it breaks a gradient's contour
	// without adding the low-frequency clumping a hash produces. This is deliberately done
	// BEFORE sky.js re-saturates the sky (r3 review E2) — the same quantisation spread over
	// a wider colour distance is much harder to hide.
	if(shoulderMeter.z > 0.0) {
		float dth = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
		disp += (dth - 0.5) * shoulderMeter.z * (1.0 / 255.0);
	}

	outputColor = vec4(disp * disp, inputColor.a);

}`;

class FilmEffect extends Effect {
  /**
   * @param {Texture|null} adaptedLuminance - the 1x1 adapted-luminance target produced by
   *   `AdaptiveExposureEffect`. When it is missing the shoulder compiles out entirely and
   *   the curve degrades to the OPEN (near-identity) end, which is the safe direction.
   */
  constructor(adaptedLuminance = null, meterNorm = 1) {
    super('FilmEffect', filmFrag, {
      blendFunction: BlendFunction.SRC,
      defines: adaptedLuminance ? new Map([['SHOULDER_ADAPTIVE', '1']]) : new Map(),
      uniforms: new Map([
        // r3 review, I2: 3.0% grain, not 1–2%, was also feeding SMAA's edge detector — see
        // the threshold note at the SMAA construction site.
        ['grainConfig', new THREE.Uniform(new THREE.Vector4(0.022, 1, 0.35, 0))],
        ['vignetteConfig', new THREE.Uniform(new THREE.Vector4(0.24, 0.45, 1.25, 0.14))],
        ['moodConfig', new THREE.Uniform(new THREE.Vector4(1, 0, 0, 1))],
        ['shoulderConfig', new THREE.Uniform(new THREE.Vector4(
          SHOULDER_KNEE_OPEN, SHOULDER_ROLLOFF_OPEN, SHOULDER_KNEE_FLAT, SHOULDER_LIFT_FLAT))],
        ['shoulderMeter', new THREE.Uniform(new THREE.Vector4(
          SHOULDER_METER_LO, SHOULDER_METER_HI, DITHER_LSB, 1.0))],
        ['adaptedLuminance', new THREE.Uniform(adaptedLuminance)],
        // The metering statistic is a weighted mean; dividing by E[w] here is what makes
        // shoulderMeter.x/.y comparable with the review's measured scene luminances.
        ['shoulderMeterNorm', new THREE.Uniform(meterNorm > 0 ? meterNorm : 1)],
        ['crashTint', new THREE.Uniform(new THREE.Color(1.06, 0.86, 0.82))],
      ]),
    });
  }

  get grain() { return this.uniforms.get('grainConfig').value; }
  get vignette() { return this.uniforms.get('vignetteConfig').value; }
  get mood() { return this.uniforms.get('moodConfig').value; }
  get shoulder() { return this.uniforms.get('shoulderConfig').value; }
  get shoulderMeter() { return this.uniforms.get('shoulderMeter').value; }
}

// --- 2d. Far-field-only circle of confusion --------------------------------

// The exact tail of `postprocessing`'s circle-of-confusion fragment shader. Matched as a
// regex rather than a literal so a whitespace change in a patch release does not silently
// turn the far-field guarantee back into a near-field blur.
const COC_NEAR_FAR_RE = /gl_FragColor\s*\.\s*rg\s*=\s*magnitude\s*\*\s*vec2\s*\([^;]*\)\s*;/;

// Far channel only. `distance` is the material's own local (it shadows the GLSL builtin, as
// the library wrote it); `focusDistance` and `focusRange` are its existing uniforms, here
// re-purposed as the start and the width of the far ramp. `max(focusRange, 1.0)` keeps
// smoothstep's edges ordered no matter what is written to the uniform.
const COC_FAR_ONLY =
  'gl_FragColor.rg=vec2(0.0,smoothstep(focusDistance,focusDistance+max(focusRange,1.0),distance));';

/**
 * Rewrite a `DepthOfFieldEffect`'s CoC material so it can only ever produce far-field blur.
 * Returns false if the library's shader no longer looks the way this patch expects, in which
 * case the caller must drop depth of field rather than ship a near-field blur.
 *
 * @param {DepthOfFieldEffect} dof
 * @return {Boolean} whether the patch applied.
 */
function patchCoCFarFieldOnly(dof) {
  try {
    const mat = dof.cocMaterial;
    const src = mat && mat.fragmentShader;
    if (typeof src !== 'string' || !COC_NEAR_FAR_RE.test(src)) {
      console.warn('[postfx] circle-of-confusion shader does not match the expected form; ' +
        'depth of field disabled rather than risk a near-field blur.');
      return false;
    }
    mat.fragmentShader = src.replace(COC_NEAR_FAR_RE, COC_FAR_ONLY);
    mat.needsUpdate = true;
    return true;
  } catch (err) {
    console.warn('[postfx] could not patch the circle-of-confusion shader:', err);
    return false;
  }
}

// ===========================================================================
// 3. Factory
// ===========================================================================

export function createPostFX(ctx) {
  const renderer = ctx.renderer;
  const scene = ctx.scene;
  const camera = ctx.camera;
  const settings = ctx.settings || {};

  // --- state -------------------------------------------------------------
  let composer = null;
  let broken = false;              // hard fallback to renderer.render()
  let brokenLogged = false;
  let tierName = TIERS[ctx.quality] ? ctx.quality : 'high';
  let tier = TIERS[tierName];

  let speed01 = 0;                 // smoothed 0..1 speed response
  let speedTarget = 0;
  let crash = 0;                   // 0..1 crash pulse envelope
  let impact = 0;                  // short g-force flinch
  let grainSeed = 0;
  let havePrevMatrices = false;
  let focusHintT = 0;              // last trail t under the camera, for a local search
  let subjectDist = 0;             // metres, camera → bike; 0 when there is no subject
  let haveSubject = false;         // is there a hero in front of the lens? (gates DoF)
  // Tracked so `enabled` is only written (and refreshOutputPass only run) on a change.
  let dofOn = null;
  let motionBlurOn = null;
  // Per-instance, not module scope: this is *state* (last frame's camera), not scratch.
  const prevViewProj = new THREE.Matrix4();
  // Camera-cut detection (r5 review). Preallocated; `render()` allocates nothing.
  const cutPrevPos = new THREE.Vector3();
  const cutPrevFwd = new THREE.Vector3();
  let haveCutRef = false;
  let cutHold = 0;             // seconds of fast exposure adaptation still owed
  let adaptRateApplied = -1;   // last value written to the effect, so we only write on change

  const passes = {};
  const fx = {};
  const disposables = [];
  const unsubscribes = [];

  /** Constructing an effect must never take the whole frame down. */
  function tryBuild(name, fn) {
    try {
      return fn();
    } catch (err) {
      console.warn(`[postfx] ${name} unavailable, skipping:`, err && err.message ? err.message : err);
      return null;
    }
  }

  // -----------------------------------------------------------------------
  // 3a. Grading LUT (built once, reused across quality changes)
  // -----------------------------------------------------------------------
  let lutTexture = tryBuild('grading LUT', () => buildGradingLUT(tier.lutSize));
  if (lutTexture) disposables.push(lutTexture);

  /**
   * Hardware linear filtering of a FLOAT 3D texture needs OES_texture_float_linear even on
   * WebGL2. Without it the LUT must stay on the tetrahedral path (which switches the
   * texture to NEAREST and interpolates in the shader), so the P2 "drop tetrahedral at
   * high" saving is only taken where it is actually safe.
   */
  function canFilterFloatLUT() {
    try {
      const gl = renderer.getContext && renderer.getContext();
      return !!(gl && gl.getExtension('OES_texture_float_linear'));
    } catch (e) {
      return false;
    }
  }

  // -----------------------------------------------------------------------
  // 3b. Chain construction
  // -----------------------------------------------------------------------
  function buildChain() {
    const s = ctx.settings || {};
    tier = TIERS[tierName] || TIERS.high;

    composer = new EffectComposer(renderer, {
      frameBufferType: THREE.HalfFloatType,
      depthBuffer: true,
      stencilBuffer: false,
      // 2x at `high`, 4x at `ultra`, off below (r6 review item 1 — see TIERS.high). This is
      // also the value vegetation.js polls to decide whether `alphaToCoverage` is worth
      // enabling on the foliage materials, so the clamp matters: on a device that reports
      // maxSamples 0 the coverage stays off rather than costing a per-draw state change for
      // nothing.
      multisampling: Math.min(tier.msaa, renderer.capabilities.maxSamples || 0),
    });

    // -- 1. scene ---------------------------------------------------------
    passes.render = new RenderPass(scene, camera);
    composer.addPass(passes.render);

    // -- 2. ambient occlusion --------------------------------------------
    if (s.ao !== false) {
      const ao = tryBuild('N8AO', () => {
        const size = renderer.getDrawingBufferSize(new THREE.Vector2());
        const pass = new N8AOPostPass(scene, camera, size.x, size.y);
        const cfg = pass.configuration;
        // r2 review, P0-b. This is a full-screen pass over the *composite*, so whatever it
        // does, it does to direct sunlight as well as to ambient. It therefore has to be a
        // contact term and nothing more. The previous settings — radius 1.6 m (a GI
        // radius), intensity 2.8, occlusion colour (0.03, 0.045, 0.07) — multiplied a
        // moderately occluded pixel by 0.37 whether or not the sun was hitting it, and are
        // the single largest contributor to the 9x shadow-floor collapse between r1 and r2.
        //
        // The compositor is `mix(scene, color * scene, 1 - pow(texel.r, intensity))`, so
        // `color` is the multiplier applied at *full* occlusion. At (0.10, 0.13, 0.18) a
        // fully occluded pixel keeps ~13% of its light and resolves toward the sky fill
        // rather than toward zero; at intensity 1.1 the same 0.7 raw occlusion that used to
        // land on a 0.37 multiplier now lands on 0.72.
        //
        // 0.55 m: under a wheel, in the gap between two rocks, in the crease where a trunk
        // meets the ground. Anything wider is sky occlusion and belongs to the IBL.
        cfg.aoRadius = 0.55;
        // n8ao's world-space range check is `aoRadius * distanceFalloff * 0.2` metres.
        // 2.2 at this radius gives ~24 cm of depth tolerance — enough to seat contacts,
        // tight enough that a distant surface behind a near one is not treated as touching.
        cfg.distanceFalloff = 2.2;
        cfg.intensity = 1.1;
        cfg.aoSamples = tier.aoSamples;
        cfg.denoiseSamples = tier.aoDenoise;
        cfg.denoiseRadius = 10;
        cfg.denoiseIterations = tier.aoDenoiseIterations;
        cfg.halfRes = tier.aoHalfRes;
        cfg.depthAwareUpsampling = true;
        cfg.screenSpaceRadius = false;
        cfg.colorMultiply = true;
        cfg.accumulate = false;       // the camera never stops moving; accumulation ghosts
        cfg.transparencyAware = false;
        // The proxy only disables auto-detection when the value actually changes, and a
        // per-frame scene.traverse() over a 30 k-instance forest is not acceptable.
        pass.autoDetectTransparency = false;
        // Occluded pixels resolve toward the sky fill, not toward zero. Ambient occlusion
        // is a loss of *sky* light, not of all light, and a crevice on a sunlit mountain
        // still receives bounce from the rock either side of it.
        cfg.color = new THREE.Color(0.10, 0.13, 0.18);
        return pass;
      });
      if (ao) {
        passes.ao = ao;
        composer.addPass(ao);
      }
    }

    // -- 3. depth of field, FAR FIELD ONLY --------------------------------
    // r3 review, I2. See the CONTRACT-NOTE at the head of the file for why this is back and
    // why it cannot become r2's defect again.
    //
    // The library's CoC material writes `magnitude * vec2(step(sd,0), step(0,sd))` — near
    // CoC in .r, far CoC in .g, both derived from |distance - focusDistance|. That shape
    // cannot express "sharp from the lens to 30 m, then ramp": the near field always exists
    // between the camera and the focus plane. So the last line of that shader is replaced
    // with a far-only ramp, which also re-purposes the two existing uniforms:
    //     focusDistance -> the distance at which the far field starts
    //     focusRange    -> the width of the ramp to maximum CoC
    // The near channel becomes a literal 0.0, so `mix(result, colorNear, cocNear)` in the
    // combine shader is an exact no-op no matter what anything else does.
    if (s.dof !== false) {
      fx.dof = tryBuild('DepthOfFieldEffect', () => {
        const dof = new DepthOfFieldEffect(camera, {
          focusDistance: DOF_FAR_START,
          focusRange: DOF_FAR_END - DOF_FAR_START,
          bokehScale: DOF_MAX_COC_PX,
          resolutionScale: tier.dofScale,
        });
        if (!patchCoCFarFieldOnly(dof)) {
          // Fail closed. A near-field blur focused by a library default is exactly the
          // thing r2 deleted; better no depth of field at all.
          try { dof.dispose(); } catch (e) { /* nothing to clean up */ }
          throw new Error('circle-of-confusion shader patch did not apply');
        }
        // NOT `dof.target = _focusPoint`. The library's auto-focus overwrites
        // `focusDistance` with `camera.distanceTo(target)` every frame, and after the patch
        // that uniform means "where the far field starts" — letting it be driven by a point
        // in front of the lens would put the ramp start at 15 m and defocus the trail ahead.
        // `updateEffectParams()` sets it from the subject distance instead.
        dof.target = null;
        return dof;
      });
      if (fx.dof) {
        passes.dof = new EffectPass(camera, fx.dof);
        passes.dof.enabled = false;      // syncGatedPasses() owns it from here
        dofOn = false;
        composer.addPass(passes.dof);
      }
    }

    // -- 4. motion blur ---------------------------------------------------
    if (s.motionBlur !== false) {
      fx.motionBlur = tryBuild('MotionBlurEffect', () => new MotionBlurEffect({
        samples: tier.mbSamples,
        shutter: 0.9,
        maxBlur: 0.05,
      }));
      if (fx.motionBlur) {
        passes.motionBlur = new EffectPass(camera, fx.motionBlur);
        // r2 review, P2: a stationary or barely-rolling camera produces a velocity of zero,
        // so the pass is a full-resolution round trip that returns its input unchanged.
        // Gated on speed01 with hysteresis by `syncGatedPasses()`.
        passes.motionBlur.enabled = false;
        motionBlurOn = false;
        composer.addPass(passes.motionBlur);
      }
    }

    // -- 5. adaptive exposure ---------------------------------------------
    // r3 review, I3: this used to have its own `EffectPass`, on the stated ground that
    // "everything after it must see the exposed image". True, and satisfied by merging:
    // `EffectPass` chains the effects' `mainImage` calls in order and feeds each one the
    // previous blended result, this effect declares no `EffectAttribute` and blends with
    // `SRC`, so putting it first in the tone-map pass is byte-identical to running it in a
    // pass of its own — minus a full-resolution RGBA16F read+write and a composer
    // ping-pong for one scalar multiply. Its `update()` still receives the same
    // `inputBuffer` (the composer's input to this pass is the same buffer the standalone
    // exposure pass used to read), so the metering is unchanged as well.
    fx.exposure = tryBuild('AdaptiveExposureEffect', () => new AdaptiveExposureEffect({
      base: baseExposure(),
      key: EXPOSURE_KEY,
      // r5 review. r3 widened this to [0.35, 3.0] as headroom for a controller that was
      // pinned at a 1.50 ceiling, and asked for the effective compensation to be read back
      // on the re-shoot. Read back (by inverting ACES on the measured medians, and now also
      // directly via `postfx.debugExposure()`): ~1.15 in the establishing wide, pinned at the
      // ceiling in the two enclosed shots. The floor was never within 0.6 stop of anything
      // the scene asked for. See EXPOSURE_MIN / EXPOSURE_MAX for the re-derivation.
      min: EXPOSURE_MIN,
      max: EXPOSURE_MAX,
      // Partial compensation (< 1) so a dark forest still reads dark and an open sunlit
      // face still reads bright — the eye does not fully normalise.
      strength: EXPOSURE_STRENGTH,
      adaptationRate: ADAPT_RATE,
      resolution: 256,
      // r5 review: centre-weighted metering with the sky down-weighted rather than cropped
      // out, plus a bounded highlight term. r3's hard upper-25% crop is what left the
      // controller unable to see the sky clipping at all.
      weightedMetering: true,
      rolloffKnee: HL_KNEE,
      rolloffSoft: HL_SOFT,
      // r6 review item 2: the additive term that gives the chain a reachable white point.
      // See the derivation at HL_SPEC_GAIN.
      rolloffSpecGain: HL_SPEC_GAIN,
      rolloffSpecSoft: HL_SPEC_SOFT,
    }));

    // -- 6. exposure + tone map + grade -----------------------------------
    // (God rays used to sit at the head of this pass. Deleted — see the CONTRACT-NOTE.
    //  Bloom used to sit here too and has moved after the grade — see P1-b below.)
    const hdrEffects = [];
    // Must be first: everything downstream has to see the exposed image.
    if (fx.exposure) hdrEffects.push(fx.exposure);

    fx.toneMapping = tryBuild('ToneMappingEffect', () => new ToneMappingEffect({
      blendFunction: BlendFunction.SRC,
      mode: ToneMappingMode.ACES_FILMIC,
    }));
    if (fx.toneMapping) hdrEffects.push(fx.toneMapping);

    if (lutTexture) {
      // r2 review, P2: tetrahedral sampling is exact but costs a branchy shader. Hardware
      // trilinear is indistinguishable on a 33³ cube with a grade this gentle — but it
      // switches the float LUT to LinearFilter, which needs OES_texture_float_linear. Keep
      // tetrahedral wherever that extension is missing, and at ultra.
      const tetra = tier.tetrahedralLUT || !canFilterFloatLUT();
      fx.lut = tryBuild('LUT3DEffect', () => new LUT3DEffect(lutTexture, {
        blendFunction: BlendFunction.SRC,
        tetrahedralInterpolation: tetra,
      }));
      if (fx.lut) hdrEffects.push(fx.lut);
    }

    if (hdrEffects.length > 0) {
      passes.hdr = new EffectPass(camera, ...hdrEffects);
      composer.addPass(passes.hdr);
    }

    // -- 7. chromatic aberration -----------------------------------------
    // Convolution effect: has to live in its own pass, and cannot share a pass with the
    // UV-transforming lens distortion.
    //
    // r2 review, P2: cut below ultra. At the authored offset the fringe is 0.86 x 0.24
    // pixels at 1080p — under a pixel, i.e. invisible — and it was paying for a full-res
    // RGBA16F round trip to produce it. At ultra it stays, because it is what the speed
    // ramp modulates and at ultra we can afford the buffer.
    if (s.chromaticAberration !== false && tierName === 'ultra') {
      fx.chromaticAberration = tryBuild('ChromaticAberrationEffect', () =>
        new ChromaticAberrationEffect({
          offset: new THREE.Vector2(0.00045, 0.00022),
          radialModulation: true,
          modulationOffset: 0.22,   // dead-centre stays perfectly clean
        }));
      if (fx.chromaticAberration) {
        passes.chromaticAberration = new EffectPass(camera, fx.chromaticAberration);
        composer.addPass(passes.chromaticAberration);
      }
    }

    // -- 8. lens distortion + bloom + grain + vignette --------------------
    const lensEffects = [];
    fx.lensDistortion = tryBuild('LensDistortionEffect', () => new LensDistortionEffect({
      // Negative k = barrel. The sampled radius shrinks toward the centre, so the frame
      // never samples outside [0,1] and there are no black corners.
      distortion: new THREE.Vector2(-0.022, -0.022),
      focalLength: new THREE.Vector2(1, 1),
      principalPoint: new THREE.Vector2(0, 0),
    }));
    if (fx.lensDistortion) lensEffects.push(fx.lensDistortion);

    // r2 review, P1-b. Bloom rides in this pass, i.e. it thresholds the *tone-mapped and
    // graded* image. `BloomEffect.update()` always reads its owning pass's input buffer, so
    // its position within the effect list does not change what it sources — only the pass
    // it belongs to does. In the old HDR pass the buffer was unbounded, a daylight sky sat
    // several units above 1.0, and a 0.85 threshold therefore admitted the entire sky at
    // full weight; 7–8 mip levels at radius 0.78 then spread that across the whole frame as
    // the milky wash every critic described. Here the input is bounded to [0,1], so 0.85
    // means "the top 15% of the display range" — the sun's surround, snow, wet-rock
    // speculars, brake-rotor glints — and 5 levels at radius 0.60 keep the halo local.
    if (s.bloom !== false) {
      fx.bloom = tryBuild('BloomEffect', () => new BloomEffect({
        blendFunction: BlendFunction.SCREEN,
        // r5 review. With the HDR roll-off in place the sky lands at ~0.91-0.93 linear in
        // this pass's input buffer — i.e. it was sitting *above* a 0.85 threshold, over the
        // largest uniform region in the frame, and a SCREEN blend at 0.80 was adding another
        // ~0.02 linear to it. Bloom was therefore manufacturing part of the >L250 slab it
        // was being blamed for spreading. At 0.92 the sky falls out of the bloom source and
        // genuine speculars (wet rock, rotor glints, the sun's surround, snow) stay in.
        luminanceThreshold: 0.92,
        luminanceSmoothing: 0.3,
        intensity: 0.72,
        mipmapBlur: true,
        radius: 0.60,
        levels: tier.bloomLevels,
      }));
      if (fx.bloom) lensEffects.push(fx.bloom);
    }

    // The shoulder reads the adapted-luminance target the exposure controller renders
    // earlier in the frame, so it is always one pass fresh. If the exposure effect failed to
    // build there is no metering signal and the shoulder compiles out (see FilmEffect).
    const adaptedLum = (fx.exposure && fx.exposure.adaptiveLuminancePass)
      ? fx.exposure.adaptiveLuminancePass.texture
      : null;
    const adaptedNorm = fx.exposure ? fx.exposure.meterNorm : 1;
    fx.film = tryBuild('FilmEffect', () => new FilmEffect(adaptedLum, adaptedNorm));
    if (fx.film) {
      if (s.filmGrain === false) fx.film.grain.x = 0;
      lensEffects.push(fx.film);
    }

    if (lensEffects.length > 0) {
      passes.lens = new EffectPass(camera, ...lensEffects);
      composer.addPass(passes.lens);
    }

    // -- 9. anti-aliasing, last ------------------------------------------
    if (s.antialias !== 'none') {
      fx.smaa = tryBuild('SMAAEffect', () => new SMAAEffect({
        preset: tier.smaa,
        edgeDetectionMode: EdgeDetectionMode.COLOR,
      }));
      if (fx.smaa) {
        // Raise the edge threshold above the grain amplitude so film grain does not get
        // mistaken for geometry edges (the classic "SMAA after grain" artefact). Grain is
        // now 0.022 rather than 0.030 and the ordered dither is 1/255, so 0.06 has more
        // headroom than it did — but it is left alone here deliberately: lowering it is a
        // foliage-aliasing decision that belongs with the alpha-to-coverage work in
        // vegetation.js (r3 review D2), not with a grain change.
        try { fx.smaa.edgeDetectionMaterial.edgeDetectionThreshold = 0.06; } catch (e) { /* older API */ }
        passes.smaa = new EffectPass(camera, fx.smaa);
        composer.addPass(passes.smaa);
      }
    }

    refreshOutputPass();
    applyPixelRatioToGrain();
  }

  /** The engine authors the intended scene exposure; honour it. */
  function baseExposure() {
    const e = ctx.engine && typeof ctx.engine.exposure === 'number'
      ? ctx.engine.exposure
      : (renderer.toneMappingExposure || 1.0);
    return clamp(e, 0.05, 8);
  }

  /**
   * Exactly one pass may render to the screen, and it must be the last *enabled* one —
   * otherwise toggling an effect off at runtime would leave a black frame.
   */
  function refreshOutputPass() {
    if (!composer) return;
    composer.autoRenderToScreen = false;
    let last = null;
    for (const p of composer.passes) if (p.enabled) last = p;
    for (const p of composer.passes) p.renderToScreen = (p === last);
  }

  /**
   * Camera-cut detection (r5 review).
   *
   * The exposure controller is deliberately slow — ADAPT_RATE gives it a ~2 s time constant,
   * which is what an eye does and what stops the frame pumping as the rider passes under a
   * tree. An eye does not adapt across a *cut*, though, because it never saw the outgoing
   * frame: a replay angle change, a photo-mode jump, a respawn or a QA pose should re-expose
   * immediately. Without that, whatever the previous shot metered is what the next one is
   * rendered at — measured on the r5 set, `shoot()` gives the composer 7 frames at dt = 1/60,
   * over which the adapted luminance moves 5.7%, so all sixteen review frames were captured
   * at substantially one exposure. That single fact explains both ends of the r5 spread: the
   * open shots clipped 24-38% and the enclosed ones sat at a median of L=47.
   *
   * Runs in `render()` rather than `update()` on purpose — `render()` is the one entry point
   * every caller uses, including the QA harness, which drives the composer directly and never
   * calls `update()`.
   *
   * Also re-seeds the motion-blur reprojection: a teleport otherwise produces exactly one
   * frame of full-screen smear, because `prevViewProj` describes a camera somewhere else.
   *
   * @param {Number} dt - seconds since the last render.
   */
  function detectCameraCut(dt) {
    if (!camera) return;
    camera.getWorldPosition(_camWorld);
    camera.getWorldDirection(_camFwd);

    let cut = false;
    if (!haveCutRef) {
      cut = true;                     // first frame: adopt the scene's exposure at once
      haveCutRef = true;
    } else {
      cut = _camWorld.distanceTo(cutPrevPos) > CUT_DISTANCE
        || _camFwd.dot(cutPrevFwd) < CUT_DOT;
    }
    cutPrevPos.copy(_camWorld);
    cutPrevFwd.copy(_camFwd);

    if (cut) {
      cutHold = CUT_HOLD;
      havePrevMatrices = false;       // no smear across the cut
    } else if (cutHold > 0) {
      cutHold = Math.max(0, cutHold - dt);
    }

    if (fx.exposure) {
      const want = cutHold > 0 ? ADAPT_RATE_CUT : ADAPT_RATE;
      if (want !== adaptRateApplied) {
        fx.exposure.adaptationRate = want;
        adaptRateApplied = want;
      }
    }
  }

  function applyPixelRatioToGrain() {
    if (!fx.film) return;
    // Keep the grain cell at roughly one CSS pixel regardless of DPR, so it does not
    // dissolve into invisibility on a Retina panel.
    const pr = renderer.getPixelRatio ? renderer.getPixelRatio() : 1;
    fx.film.grain.y = Math.max(1, Math.round(pr));
  }

  // -----------------------------------------------------------------------
  // 3b-ii. Shader precompile (r6 review, item 3)
  // -----------------------------------------------------------------------
  //
  // The defect: `engine.js:415` sets `renderer.debug.checkShaderErrors = true` and nothing
  // anywhere calls `compile()`. So every material in a 22-module, ~200k-instance world
  // compiles *lazily, on its first draw*, and each of those compiles is followed by a
  // synchronous `getProgramParameter(LINK_STATUS)` + `getShaderInfoLog()` — a full CPU/GPU
  // pipeline flush, per program, in the middle of the first seconds of a run. That is the
  // exact inverse of what a console build does, and it lands on the frames where the rider
  // is accelerating away from the gate.
  //
  // The fix has to live here rather than in engine.js for two reasons, and they are also why
  // main.js's `postfx.init()` hook is the right place to hang it:
  //   * postfx is wave 9, i.e. the first moment at which the scene is complete AND the
  //     composer exists. Anything earlier would compile half a world.
  //   * main.js `await`s `postfx.init()` before it starts the frame loop, so all of this
  //     happens during boot, behind the menu overlay, and every millisecond it costs is a
  //     millisecond the run does not spend linking programs.
  //
  // What gets compiled: `compileAsync()` covers every material reachable from the scene
  // graph (three builds the real light state for it, so these are the shipping permutations,
  // not stand-ins), and one throwaway composer render covers the shadow-depth variants and
  // every pass material in the chain — *including* the two passes that are gated off at rest
  // and would otherwise compile mid-run the first time the rider hits 8.6 m/s (motion blur)
  // or a subject enters the DoF gate.
  const nowMs = () => ((typeof performance !== 'undefined' && performance.now)
    ? performance.now() : Date.now());

  const precompileStats = {
    ran: false, ms: 0, sceneMs: 0, chainMs: 0,
    programsBefore: 0, programsAfter: 0, compiled: 0,
    parallel: false, gatedCheckShaderErrors: false, devBuild: false, error: null,
  };

  /**
   * Is this a development build? Vite substitutes `import.meta.env.DEV` at build time; a
   * plain `<script type="module">` load leaves `import.meta.env` undefined, and undefined
   * means "assume production", which is the conservative answer for a *diagnostic* flag.
   */
  function isDevBuild() {
    try {
      return !!(import.meta && import.meta.env && import.meta.env.DEV);
    } catch (e) {
      return false;
    }
  }

  /**
   * Compile the whole pipeline up front. Idempotent; never throws.
   *
   * @return {Promise<Object>} `precompileStats`, also readable later via `debugPrecompile()`.
   */
  async function precompile() {
    if (precompileStats.ran) return precompileStats;
    precompileStats.ran = true;
    precompileStats.devBuild = isDevBuild();
    if (broken || !composer) return precompileStats;

    const t0 = nowMs();
    const info = renderer.info;
    precompileStats.programsBefore = (info && info.programs) ? info.programs.length : 0;

    // Gate the validation query. In a dev build it stays on — this project is mostly
    // hand-written GLSL and a silent link failure there costs an afternoon — and the stall
    // is paid here, once, during boot, instead of during the run. In a production
    // build it goes off and STAYS off: the whole point is that the shipping binary does not
    // flush the pipeline once per program, and by this point there is nothing left to
    // validate anyway.
    const prevCheck = renderer.debug ? renderer.debug.checkShaderErrors : false;
    if (renderer.debug && !precompileStats.devBuild) {
      renderer.debug.checkShaderErrors = false;
      precompileStats.gatedCheckShaderErrors = true;
    }

    try {
      // --- 1. every material in the scene ---------------------------------
      // `compileAsync` uses KHR_parallel_shader_compile where the driver has it, so the
      // link status is polled instead of blocked on; `compile` is the synchronous fallback.
      const tScene = nowMs();
      if (typeof renderer.compileAsync === 'function') {
        precompileStats.parallel = true;
        await renderer.compileAsync(scene, camera);
      } else if (typeof renderer.compile === 'function') {
        renderer.compile(scene, camera);
      }
      precompileStats.sceneMs = nowMs() - tScene;

      // --- 2. every pass in the chain, gated ones included ----------------
      // The `renderToScreen` flags are deliberately left EXACTLY as they are. It is tempting
      // to force them all false so this render never touches the canvas, and it would be
      // wrong: `Pass`'s setter does `material.needsUpdate = true` on every *change* of that
      // flag, because the output pass compiles a different variant (it owns the sRGB
      // conversion). Forcing it off and back on would precompile the variant that never runs
      // and then throw away the one that does — the exact opposite of the point. One frame
      // appears on the canvas instead, at the end of boot, behind the menu overlay.
      //
      // Only the two gated passes are toggled, and toggling `enabled` compiles nothing it
      // should not: neither of them is ever the last enabled pass (SMAA is), so neither one's
      // `renderToScreen` moves, and this compiles them in the configuration they will
      // actually run in the first time the rider crosses 8.6 m/s or a subject enters the DoF
      // gate — mid-run, at speed, which is the worst possible moment to link a program.
      const tChain = nowMs();
      const wasDof = passes.dof ? passes.dof.enabled : false;
      const wasMotionBlur = passes.motionBlur ? passes.motionBlur.enabled : false;
      if (passes.dof) passes.dof.enabled = true;
      if (passes.motionBlur) passes.motionBlur.enabled = true;
      try {
        composer.render(1 / 60);
      } finally {
        if (passes.dof) passes.dof.enabled = wasDof;
        if (passes.motionBlur) passes.motionBlur.enabled = wasMotionBlur;
        refreshOutputPass();
      }
      precompileStats.chainMs = nowMs() - tChain;
    } catch (err) {
      precompileStats.error = String((err && err.message) || err);
      console.warn('[postfx] precompile did not complete; the run will compile lazily:', err);
    } finally {
      if (renderer.debug) {
        renderer.debug.checkShaderErrors = precompileStats.devBuild ? prevCheck : false;
      }
      // The precompile frame is a throwaway drawn from wherever the camera happened to be
      // parked at the end of boot. Nothing about it may leak into the first real frame:
      // `prevViewProj` would describe a camera somewhere else (one full-screen smear), and
      // the camera-cut reference would make the first real frame adapt slowly from a pose
      // that was never on screen — which is precisely the r5 defect this file already fixed
      // once. Reset both, so frame 1 is treated as a cut and re-exposes immediately.
      havePrevMatrices = false;
      haveCutRef = false;
      cutHold = 0;
      adaptRateApplied = -1;
    }

    precompileStats.programsAfter = (info && info.programs) ? info.programs.length : 0;
    precompileStats.compiled = precompileStats.programsAfter - precompileStats.programsBefore;
    precompileStats.ms = nowMs() - t0;
    console.info(
      `[postfx] precompiled ${precompileStats.compiled} programs in ` +
      `${precompileStats.ms.toFixed(1)} ms (scene ${precompileStats.sceneMs.toFixed(1)}, ` +
      `chain ${precompileStats.chainMs.toFixed(1)}, ` +
      `parallel=${precompileStats.parallel}, ` +
      `checkShaderErrors=${renderer.debug ? renderer.debug.checkShaderErrors : 'n/a'})`);
    return precompileStats;
  }

  function teardownChain() {
    if (!composer) return;
    try { composer.dispose(); } catch (e) { /* already gone */ }
    composer = null;
    for (const k of Object.keys(passes)) delete passes[k];
    for (const k of Object.keys(fx)) delete fx[k];
    havePrevMatrices = false;
    dofOn = null;
    motionBlurOn = null;
    haveCutRef = false;
    cutHold = 0;
    adaptRateApplied = -1;
  }

  // -----------------------------------------------------------------------
  // 3c. Frame-to-frame parameter animation
  // -----------------------------------------------------------------------

  /** Speed response, in m/s. Called from update(), and available to other modules. */
  function setSpeed(v) {
    const s = Number.isFinite(v) ? Math.abs(v) : 0;
    speedTarget = clamp01((s - SPEED_LOW) / (SPEED_HIGH - SPEED_LOW));
  }

  /** Crash / impact hook — a brief chromatic, desaturation and vignette punch. */
  function pulse(strength = 1) {
    crash = Math.max(crash, clamp01(strength));
  }

  /**
   * Is depth of field wanted right now? (r2 review P0-a, revised by r3 review I2.)
   *
   * r2's answer was "never during gameplay", and the reasoning was sound for the effect it
   * was answering about: a near-field blur you cannot read the trail through, fighting every
   * pixel of near-field detail the terrain shader synthesises. The effect that runs now is
   * far field only and cannot touch anything inside 30 m — the rider, the bike, the tread,
   * the next feature and the trailside geometry are all sharp by construction — so that
   * objection no longer applies, and the thing it *does* fix is a gameplay problem: in
   * r3_07 the highest-contrast, busiest pixels in the frame are the aliased leaf edges
   * behind the rider, which is the opposite of where a viewer's eye should go.
   *
   * Still off at `low`, where the two extra convolution passes are not affordable, still
   * respects `settings.dof === false`, and — new — off whenever there is no subject in
   * frame to separate from the background (see DOF_SUBJECT_ON).
   */
  function wantDepthOfField() {
    if ((ctx.settings || {}).dof === false) return false;
    if (tierName === 'low') return false;
    return haveSubject;
  }

  /**
   * Enable/disable the two gated passes. Only writes `enabled` (and only re-derives the
   * output pass) when a gate actually flips, because `refreshOutputPass` dirties pass
   * materials.
   */
  function syncGatedPasses() {
    let changed = false;

    if (passes.dof) {
      const want = wantDepthOfField();
      if (want !== dofOn) { dofOn = want; passes.dof.enabled = want; changed = true; }
    }

    if (passes.motionBlur) {
      // Hysteresis: on above 0.11, off below 0.08, so a rider hovering around the
      // threshold does not rebuild the output pass every frame. speed01 = 0.08 is about
      // 8.6 m/s (31 km/h) — below that the per-frame camera delta is under a pixel.
      const want = motionBlurOn ? speed01 > 0.08 : speed01 > 0.11;
      if (want !== motionBlurOn) {
        motionBlurOn = want;
        passes.motionBlur.enabled = want;
        changed = true;
      }
    }

    if (changed) refreshOutputPass();
  }

  /**
   * Subject point (r2 review P0-a / RC-4; role narrowed by r3 review I2).
   *
   * The library's own auto-focus is no longer used — `dof.target` is null and the far-ramp
   * start is written directly — so this is now only the *fallback* subject for frames where
   * there is no bike (the menu, a free QA pose). It still matters: the far ramp is pushed
   * out past the subject, so a wildly wrong subject would defocus the thing being looked at.
   * Also refreshes `_camWorld`/`_camFwd`, which `updateEffectParams()` reads.
   *
   * Preferred source is the trail spline `FOCUS_AHEAD_M` ahead of whatever point of the
   * centreline the camera is nearest to. That is where a rider is looking, it is stable
   * across camera modes, and it does not care where the bike is. Two guards, either of
   * which drops us back to a point on the lens axis:
   *   - the camera is more than FOCUS_TRAIL_MAX_DIST from the centreline (an establishing
   *     wide, a free QA pose), so the spline says nothing about this shot;
   *   - the resulting point is behind the lens (camera facing back up the hill).
   *
   * Allocation-free: `nearestT` and `sampleAt` both return trail-owned scratch objects.
   */
  /**
   * Is the hero in frame, and how far away? Gates the far-field DoF and sets its ramp start.
   * Runs before `syncGatedPasses()`, which reads `haveSubject`.
   */
  function updateSubject() {
    const bike = ctx.bike && ctx.bike.state;
    const pos = bike && bike.position;
    if (!pos || !Number.isFinite(pos.x)) { haveSubject = false; subjectDist = 0; return; }
    _tmpV3.copy(pos).sub(_camWorld);
    const d = _tmpV3.length();
    // Behind the lens is not "in frame". The dot test is against the raw delta, so it is a
    // sign test and needs no normalise.
    const inFront = _tmpV3.dot(_camFwd) > 0;
    const limit = haveSubject ? DOF_SUBJECT_OFF : DOF_SUBJECT_ON;
    haveSubject = inFront && d > 0.5 && d < limit;
    subjectDist = haveSubject ? d : 0;
  }

  function updateFocus() {
    if (!camera) return;
    camera.getWorldPosition(_camWorld);
    camera.getWorldDirection(_camFwd);
    updateSubject();

    const trail = ctx.trail;
    if (trail && typeof trail.nearestT === 'function' && typeof trail.sampleAt === 'function'
        && Number.isFinite(trail.length) && trail.length > 1) {
      const near = trail.nearestT(_camWorld, focusHintT);
      if (near && Number.isFinite(near.t) && near.distance < FOCUS_TRAIL_MAX_DIST) {
        focusHintT = near.t;
        const tAhead = Math.min(1, near.t + FOCUS_AHEAD_M / trail.length);
        const s = trail.sampleAt(tAhead);
        if (s && s.position) {
          _focusCand.copy(s.position);
          _focusCand.y += FOCUS_HEIGHT;
          // Far enough in front of the lens to be a sane focus plane? A few metres is the
          // floor — a focus distance of 1 m would put the entire frame in the near field.
          // This also rejects the degenerate case at the finish gate, where t saturates at
          // 1 and the "ahead" point stops moving away from the camera.
          _tmpV3.copy(_focusCand).sub(_camWorld);
          if (_tmpV3.dot(_camFwd) > 3.0) {
            _focusPoint.copy(_focusCand);
            return;
          }
        }
      }
    }

    // Fallback: a point on the lens axis at the same lead distance. Always in front, always
    // finite, and it keeps the CoC plane stable rather than snapping to the far plane.
    _focusPoint.copy(_camWorld).addScaledVector(_camFwd, FOCUS_AHEAD_M);
  }

  function updateEffectParams(dt) {
    const p = speed01;
    const c = crash;
    const shock = clamp01(c * 1.25);
    const flinch = impact;

    // --- depth of field, far field only ----------------------------------
    // r3 review, I2. After the CoC shader patch these two uniforms mean "where the far
    // field starts" and "how wide the ramp to maximum blur is", both in metres.
    if (fx.dof && dofOn) {
      const cocMat = fx.dof.cocMaterial;
      if (cocMat) {
        // The subject must never enter the far field. On a chase camera this is a no-op
        // (the rider is 5–14 m away and the floor is 30 m); on a 40 m replay long lens it
        // pushes the ramp start out to ~72 m, which is r2's RC-4 failure expressed as an
        // invariant instead of a hope.
        const subject = subjectDist > 0.1 ? subjectDist : _camWorld.distanceTo(_focusPoint);
        const start = Math.max(DOF_FAR_START, subject * DOF_SUBJECT_MARGIN + DOF_SUBJECT_PAD);
        cocMat.focusDistance = start;
        // Hold the ramp's shape as the start moves, so a long-lens shot gets the same
        // gradual falloff rather than snapping to maximum blur just past the subject.
        cocMat.focusRange = (DOF_FAR_END - DOF_FAR_START) * (start / DOF_FAR_START);
      }
      // bokehScale is the peak CoC radius in *drawing-buffer* pixels, so it has to scale
      // with DPR or the blur halves on a Retina panel. Written only on a change: the setter
      // touches five material uniforms.
      const pr = renderer.getPixelRatio ? renderer.getPixelRatio() : 1;
      const wantScale = DOF_MAX_COC_PX * Math.max(1, pr);
      if (Math.abs(fx.dof.bokehScale - wantScale) > 1e-3) fx.dof.bokehScale = wantScale;
    }

    // --- motion blur -----------------------------------------------------
    if (fx.motionBlur) {
      const cfg = fx.motionBlur.config;
      cfg.x = lerp(0.82, 1.0, p);                       // shutter
      cfg.z = lerp(0.0, 0.020, p * p) + 0.016 * shock;  // radial streak
      cfg.w = lerp(0.42, 0.24, p);                      // radial start radius
      const bike = ctx.bike && ctx.bike.state;
      const sphere = fx.motionBlur.riderSphere;
      if (bike && bike.position) {
        sphere.set(bike.position.x, bike.position.y + 0.6, bike.position.z, 1.75);
      } else {
        // No bike yet: park the protect sphere far below the world so it masks nothing.
        sphere.set(0, -1e6, 0, 0.001);
      }
    }

    // --- exposure --------------------------------------------------------
    if (fx.exposure) {
      // Crash dips the exposure; the "eye" then recovers through the adaptation curve.
      fx.exposure.exposure = baseExposure() * (1.0 - 0.28 * shock);
    }

    // --- bloom -----------------------------------------------------------
    if (fx.bloom) {
      // A touch more bloom at speed reads as air glare, without lifting the black point.
      fx.bloom.intensity = lerp(0.72, 0.88, p);
    }

    // --- chromatic aberration -------------------------------------------
    if (fx.chromaticAberration) {
      const o = fx.chromaticAberration.offset;
      const mag = 0.00045 + 0.00135 * p * p + 0.0055 * shock + 0.0012 * flinch;
      o.set(mag, mag * 0.48);
    }

    // --- lens distortion -------------------------------------------------
    if (fx.lensDistortion) {
      const k = -(0.022 + 0.052 * p * p + 0.030 * shock);
      fx.lensDistortion.distortion.set(k, k);
    }

    // --- film ------------------------------------------------------------
    if (fx.film) {
      const grain = fx.film.grain;
      if ((ctx.settings || {}).filmGrain !== false) {
        // r3 review, I2 asks for 1–2%. 2.2% base, because the grain is also doing the
        // dithering work in the midtones where the ordered dither is smallest relative to
        // the local gradient. Still well under SMAA's 0.06 edge threshold.
        grain.x = 0.022 + 0.010 * shock;
        // Wrap the seed so the hash never loses precision in a long session.
        grainSeed = (grainSeed + dt * 60) % 1024;
        grain.w = Math.floor(grainSeed);
      } else {
        grain.x = 0;
      }

      const vig = fx.film.vignette;
      vig.x = 0.24 + 0.09 * p + 0.45 * shock + 0.04 * flinch;   // strength
      vig.y = lerp(0.45, 0.26, Math.max(p * 0.5, shock));       // inner radius
      vig.w = 0.14 + 0.12 * p + 0.30 * shock;                   // edge desaturation

      const mood = fx.film.mood;
      mood.x = 1.0 - 0.13 * p - 0.62 * shock;   // saturation
      mood.y = shock;
      mood.z = p;
      mood.w = (1.0 + 0.03 * p) * (1.0 - 0.22 * shock);
    }
  }

  // -----------------------------------------------------------------------
  // 3d. Events
  // -----------------------------------------------------------------------
  if (ctx.events) {
    unsubscribes.push(ctx.events.on('run:crash', () => pulse(1)));
    unsubscribes.push(ctx.events.on('quality:changed', () => {
      try {
        if (TIERS[ctx.quality]) tierName = ctx.quality;
        teardownChain();
        buildChain();
        const w = (ctx.container && ctx.container.clientWidth) || window.innerWidth;
        const h = (ctx.container && ctx.container.clientHeight) || window.innerHeight;
        if (composer) composer.setSize(w, h);
      } catch (err) {
        console.error('[postfx] rebuild after quality change failed:', err);
        broken = true;
      }
    }));
  }

  // -----------------------------------------------------------------------
  // 3e. Build
  // -----------------------------------------------------------------------
  try {
    // Take ownership of tone mapping before anything renders, or ACES is applied twice
    // (once by the renderer, once by us) and the image goes flat and milky.
    if (ctx.engine && typeof ctx.engine.setToneMappingEnabled === 'function') {
      ctx.engine.setToneMappingEnabled(false);
    } else {
      renderer.toneMapping = THREE.NoToneMapping;
    }
    buildChain();
  } catch (err) {
    console.error('[postfx] composer construction failed, falling back to direct render:', err);
    broken = true;
    // If we cannot post-process, the renderer must tonemap again or the frame is raw HDR.
    try {
      if (ctx.engine && typeof ctx.engine.setToneMappingEnabled === 'function') {
        ctx.engine.setToneMappingEnabled(true);
      } else {
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
      }
    } catch (e) { /* nothing more we can do */ }
  }

  // -----------------------------------------------------------------------
  // 3f. Public system
  // -----------------------------------------------------------------------
  const system = {
    get composer() { return composer; },
    passes,
    effects: fx,

    /**
     * main.js awaits this in wave 9, before the frame loop starts and while the loading
     * screen is still up — which is the only window in the whole run where a multi-hundred-
     * millisecond stall is free. See `precompile()` for what it does and why it lives here.
     */
    async init() {
      return precompile();
    },

    /** Diagnostic: what the precompile actually did. For the QA harness. */
    debugPrecompile() { return precompileStats; },

    /** CPU-side animation. main.js calls this before render(). */
    update(dt, c) {
      const context = c || ctx;
      const bike = context.bike && context.bike.state;

      setSpeed(bike ? bike.speed : 0);
      speed01 = damp(speed01, speedTarget, 3.2, dt);

      // Crash envelope: fast attack (set to 1 by the event), ~0.85 s release.
      if (crash > 0) crash = Math.max(0, crash - dt / 0.85);

      // A light flinch from ordinary hard landings, independent of an actual crash.
      const g = bike && Number.isFinite(bike.gForce) ? bike.gForce : 0;
      impact = Math.max(damp(impact, 0, 6.0, dt), clamp01((g - 3.0) / 7.0));

      updateFocus();
      syncGatedPasses();
      updateEffectParams(dt);
    },

    /** Called by main.js instead of renderer.render(). Must always produce a frame. */
    render(dt) {
      const delta = Number.isFinite(dt) ? dt : 0.016;

      if (broken || !composer) {
        renderer.render(scene, camera);
        return;
      }

      // Must run before the composer so the fast adaptation rate is in place for the very
      // frame the cut happens on, not the one after it.
      detectCameraCut(delta);

      // Camera reprojection matrices for motion blur. Done here, after chaseCamera's
      // lateUpdate, so the matrices describe the frame we are about to draw.
      //
      // The view-projection is tracked every frame *even while the pass is gated off*,
      // because `prevViewProj` has to describe the immediately preceding frame. Letting it
      // go stale while the rider is slow would produce one enormous smear on the frame the
      // gate re-opens. Only the inverse and the uniform upload are skipped.
      if (fx.motionBlur) {
        // three only refreshes matrixWorldInverse inside renderer.render(), which has not
        // run yet this frame — so derive it here or the reprojection lags by a frame and
        // disagrees with the depth buffer it is sampling.
        camera.updateMatrixWorld();
        camera.matrixWorldInverse.copy(camera.matrixWorld).invert();
        _viewProj.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
        if (!havePrevMatrices) {
          prevViewProj.copy(_viewProj);
          havePrevMatrices = true;
        }
        if (motionBlurOn) {
          _invViewProj.copy(_viewProj).invert();
          fx.motionBlur.setCameraMatrices(_invViewProj, prevViewProj);
        }
      }

      try {
        composer.render(delta);
      } catch (err) {
        if (!brokenLogged) {
          brokenLogged = true;
          broken = true;
          console.error('[postfx] render failed, falling back to direct render:', err);
        }
        renderer.render(scene, camera);
        return;
      }

      if (fx.motionBlur) prevViewProj.copy(_viewProj);
    },

    resize(w, h) {
      const width = Math.max(1, w | 0) || window.innerWidth;
      const height = Math.max(1, h | 0) || window.innerHeight;
      // composer.setSize() takes CSS pixels and forwards the drawing-buffer size to every
      // pass, N8AO included, so nothing else needs resizing by hand.
      if (composer) {
        try { composer.setSize(width, height); } catch (e) { /* keep the frame alive */ }
      }
      applyPixelRatioToGrain();
    },

    /**
     * Quality is normally driven by ctx.setQuality() → 'quality:changed' → full rebuild.
     * This entry point exists for the contract; a string simply re-reads the tier.
     */
    setQuality(q) {
      if (typeof q === 'string' && TIERS[q]) tierName = q;
      if (broken) return;
      try {
        teardownChain();
        buildChain();
        const w = (ctx.container && ctx.container.clientWidth) || window.innerWidth;
        const h = (ctx.container && ctx.container.clientHeight) || window.innerHeight;
        if (composer) composer.setSize(w, h);
      } catch (err) {
        console.error('[postfx] setQuality rebuild failed:', err);
        broken = true;
      }
    },

    setSpeed,
    pulse,

    /**
     * Toggle a single pass by name, e.g. setEnabled('dof', false).
     * Note: `dof` and `motionBlur` are re-derived every frame by `syncGatedPasses()`, so a
     * manual override of those two only lasts until the next update — set `settings.dof`
     * or `settings.motionBlur` to false instead if you want it to stick.
     */
    setEnabled(name, enabled) {
      const p = passes[name];
      if (!p) return false;
      p.enabled = !!enabled;
      if (name === 'dof') dofOn = !!enabled;
      if (name === 'motionBlur') motionBlurOn = !!enabled;
      refreshOutputPass();
      return true;
    },

    /**
     * Tune the adaptive display-space shoulder without a rebuild. The one number in it that
     * is not derived from a measurement is the adapted-luminance window that separates
     * "forest interior" from "open and sunlit"; everything else came off the r5 percentiles.
     *
     * Pass any subset: { kneeOpen, rolloffOpen, kneeFlat, liftFlat, meterLo, meterHi,
     * gain (0 pins the curve to the OPEN roll-off), dither (in LSBs, 0 disables) }.
     * `whiteOpen`/`whiteFlat` are accepted as aliases for the r3 spelling of the first and
     * fourth fields, but note the *meaning* changed in r5: the second field is now the depth
     * of a roll-off, not a white point, and the fourth is a bounded lift gain.
     */
    setShoulder(o) {
      if (!fx.film || !o) return false;
      const s = fx.film.shoulder;
      const m = fx.film.shoulderMeter;
      if (Number.isFinite(o.kneeOpen)) s.x = o.kneeOpen;
      if (Number.isFinite(o.rolloffOpen)) s.y = o.rolloffOpen;
      if (Number.isFinite(o.kneeFlat)) s.z = o.kneeFlat;
      if (Number.isFinite(o.liftFlat)) s.w = clamp(o.liftFlat, 0, 1);
      if (Number.isFinite(o.meterLo)) m.x = o.meterLo;
      if (Number.isFinite(o.meterHi)) m.y = o.meterHi;
      if (Number.isFinite(o.dither)) m.z = o.dither;
      if (Number.isFinite(o.gain)) m.w = clamp(o.gain, 0, 1);
      return true;
    },

    /**
     * Tune the HDR highlight roll-off — the curve that actually decides how much of the
     * frame can clip AND whether anything in it can be white, because it runs before ACES
     * saturates. All four fields are in post-exposure linear radiance:
     *
     *   knee     where the roll-off starts. Below it the frame is bit-identical.
     *   soft     the logarithmic softness above the knee. `soft: 0` disables the entire
     *            block, specular escape included, and restores hard ACES clipping.
     *   specGain slope the escape term approaches far above the knee. 0 removes the white
     *            point again and reproduces the r5 curve exactly.
     *   specSoft how far above the knee the escape takes to reach that slope.
     *
     * A QA sweep of the white point is `setHighlightRolloff({ specGain })` — at 0.035 paper
     * white sits at 595 units, at 0.062 it sits at 400, at 0.017 at 1000.
     */
    setHighlightRolloff(o) {
      if (!fx.exposure || !o) return false;
      const r = fx.exposure.rolloff;
      if (Number.isFinite(o.knee)) r.x = Math.max(0, o.knee);
      if (Number.isFinite(o.soft)) r.y = Math.max(0, o.soft);
      if (Number.isFinite(o.specGain)) r.z = Math.max(0, o.specGain);
      // Clamped away from zero: it is a denominator.
      if (Number.isFinite(o.specSoft)) r.w = Math.max(1e-3, o.specSoft);
      return true;
    },

    /**
     * Read the exposure controller back (r3 review's own open question, and the thing that
     * would have caught the r5 regression a round earlier).
     *
     * Costs a 1x1 `readRenderTargetPixels`, i.e. a pipeline stall — call it from the QA
     * harness once per shot, never per frame. Returns the metered scene luminance, the
     * compensation the controller derived from it, and the exposure actually applied, so a
     * review can say "the controller is pinned at its ceiling" instead of inferring it by
     * inverting ACES on a histogram.
     *
     * @return {{ok:Boolean, metered:Number, comp:Number, exposure:Number,
     *           clamped:String, base:Number, key:Number, min:Number, max:Number,
     *           meterNorm:Number, weightedMetering:Boolean}}
     */
    debugExposure() {
      const e = fx.exposure;
      const out = {
        ok: false, metered: NaN, comp: NaN, exposure: NaN, clamped: 'none',
        base: NaN, key: NaN, min: NaN, max: NaN, meterNorm: NaN, weightedMetering: false,
      };
      if (!e) return out;
      const r = e.range;
      out.base = r.x; out.key = r.y; out.min = r.z; out.max = r.w;
      out.meterNorm = e.meterNorm;
      out.weightedMetering = !!e.weightedMetering;
      try {
        // AdaptiveLuminancePass packs its scalar with three's `packDepthToRGBA`, whose
        // inverse is `dot(v, UnpackFactors4)` with
        // UnpackFactors4 = (255/256, (255/256)/256, (255/256)/65536, 1/16777216)
        // and v the bytes normalised to [0,1]. Kept explicit rather than reconstructed from
        // a shader chunk so a three revision that changes the packing shows up as a wrong
        // number in the QA log rather than as a silent one.
        const px = _exposureReadback;
        renderer.readRenderTargetPixels(e.adaptiveLuminancePass.renderTargetAdapted, 0, 0, 1, 1, px);
        const D = 255 / 256;
        const packed = (px[0] / 255) * D
          + (px[1] / 255) * (D / 256)
          + (px[2] / 255) * (D / 65536)
          + (px[3] / 255) * (1 / 16777216);
        const metered = Math.max(packed / (e.meterNorm || 1), 0.012);
        const comp = Math.pow(r.y / metered, e.uniforms.get('adaptStrength').value);
        const raw = r.x * comp;
        out.metered = metered;
        out.comp = comp;
        out.exposure = clamp(raw, r.z, r.w);
        out.clamped = raw < r.z ? 'floor' : (raw > r.w ? 'ceiling' : 'none');
        out.ok = true;
      } catch (err) {
        out.error = String((err && err.message) || err);
      }
      return out;
    },

    /** Diagnostic: the built cube's top corner, for the QA harness / a unit test. */
    gradingWhitePoint() {
      return lutTexture && lutTexture.userData
        ? lutTexture.userData.whitePoint
        : { ok: false, rgb: [NaN, NaN, NaN] };
    },

    dispose() {
      for (const off of unsubscribes) { try { off(); } catch (e) { /* ignore */ } }
      unsubscribes.length = 0;
      teardownChain();
      for (const d of disposables) {
        try { if (d && typeof d.dispose === 'function') d.dispose(); } catch (e) { /* ignore */ }
      }
      disposables.length = 0;
      lutTexture = null;
    },
  };

  return system;
}

export default createPostFX;
