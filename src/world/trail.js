// =============================================================================
// DESCENT — src/world/trail.js
// The downhill run: route-finding down the fall line, trail-design rules on top,
// a physically-derived speed profile, features (berms/jumps/rock/creek), the carve
// stamps that reshape the mountain, and the visible race course.
//
// CONTRACT §4 interface is implemented exactly. Extra, purely additive fields are
// documented in the CONTRACT-NOTEs below.
// =============================================================================
//
// CONTRACT-NOTE: `trail` exposes some additive read-only fields beyond §4 that other
//   modules may find useful (all optional to consume, nothing in §4 changed):
//     trail.cornerAudit -> the lean-identity audit, one row per corner, with a
//                          replay of the pre-fix speed/bank rule on the same
//                          geometry as the "before" column. See
//                          auditCornerIdentity() and replayOldRule().
//     trail.routeAudit  -> one row per route variant the march tried, with the
//                          corridor-protectability numbers it was scored on and
//                          which one was chosen. See marchRoute()/scoreRoute().
//     trail.safety.legBefore / legAfter
//                       -> the two-leg conflict audit: stretches where two parts
//                          of the route, more than LEG_ARC_MIN of arc apart,
//                          contest the same ground. `legBefore` is measured on
//                          the final stations before the pinch, `legAfter` after
//                          it. See auditLegConflicts() and legClearanceCost().
//     trail.safety.crestStage
//                       -> crest violations on the FINAL profile after each pass
//                          that touches it, with authored relief masked. This is
//                          NOT safety.crestViolAfter, which is measured at
//                          capCrestCurvature's own exit, four passes upstream of
//                          the profile that ships. See countCrestViol().
//     trail.splits      -> [{ id, tStart, tEnd, side, offset, mainLine:'A'|'B', label }]
//     trail.phases      -> [{ id, name, tStart, tEnd, surface }]   pacing sections
//     trail.speedAt(t)  -> expected rider speed (m/s) from the design speed profile
//     trail.surfaceAt(t)-> SurfaceId of the tread at t
//     trail.finishT     -> t of the finish arch (0.985); ~40 m of run-out follows
//     trail.group       -> THREE.Group holding all trail-owned meshes
//
// CONTRACT-NOTE: carve stamps rely on terrain honouring `stamp.bank` (radians of
//   cross-slope, + = right side high) to build the banked cross-section. To be robust
//   against a terrain implementation that ignores `bank`, every station also emits
//   laterally-offset stamps whose `targetHeight` ALREADY includes the banked/crowned
//   cross-section (targetHeight = centre + tan(bank)*offset + crossProfile). Both
//   implementations therefore converge on the same surface — the offset stamps are
//   redundant, never additive, so berms cannot be double-banked.
//
// CONTRACT-NOTE: jump ballistics — computed takeoff speeds and landing distances.
//   Every lip/landing pair is solved from the design speed profile at that station
//   (see solveJump()). Measured against the shipped terrain at the default seed
//   (20260726), the jump line builds as:
//
//     feature        takeoff        lip          landing  gap    air    window
//     tabletop       15.5 m/s 56 km/h  0.92 m @ 10°  12.2 m  6.7 m  0.87 s  7.3–20.5 m/s
//     double-1       14.0 m/s 51 km/h  0.98 m @ 15°  13.1 m  5.6 m  1.05 s  10.4–19.4
//     double-2       13.0 m/s 47 km/h  0.98 m @ 24°  15.0 m 11.1 m  1.36 s  12.2–16.6
//     road gap (A)   11.7 m/s 42 km/h  1.06 m @ 30°  15.6 m 12.2 m  1.52 s  11.0–15.2
//     step-down      14.0 m/s 50 km/h  0.72 m @ 15°  16.9 m 13.9 m  1.33 s  13.2–20.7
//     rock double    10.9 m/s 39 km/h  0.66 m @ 12°   7.5 m  4.1 m  0.76 s  9.6–14.8
//     finish booter  17.5 m/s 63 km/h  0.72 m @ 16°  19.8 m 15.6 m  1.25 s  16.8–21.3
//
//   (Re-measured this round. The table above had drifted from the shipped build
//   by up to 3 m of landing distance; the numbers here are what solveJump()
//   actually emits on seed 20260726, and they are bit-identical before and after
//   every change made this round — the jump line is untouched.)
//
//   "landing" is the horizontal distance from the lip to where the arc meets the
//   base grade; "gap" is lip-to-knuckle; "window" is the range of takeoff speeds
//   that still touch down on the landing transition. Design touchdown is 61% down
//   the ramp with a 2.5–3.8 m/s surface-normal impact (a firm landing, not a case).
//   Run with ?debug to print the table for any seed.
//
//   The landing ramp is the CHORD from the knuckle to the touchdown point, not the
//   tangent: the trajectory is concave, so a tangent line sits *above* the arc
//   everywhere except its contact point and would meet the rider at the knuckle.
//
// CONTRACT-NOTE: the run is a designed 2.6-2.7 km with a designed ~430 m drop, and
//   the route-finder stops when it has spent that altitude. On a mountain with more
//   relief than that the finish arch therefore sits part-way down the hillside
//   rather than on the valley floor — deliberate, and normal for a real DH course.
//   `trail.curve` still ends ~40 m past the arch as run-out.
//
// CONTRACT-NOTE (for the integration pass — cross-module, terrain owner should see it):
//   The tread ribbon is a decal laid on terrain.sampleHeight(), but the terrain
//   RENDERS through a quadtree whose leaf vertex spacing grows with camera
//   distance (49x49 per node, node size ~ dist/2.05 => ~1 m at 100 m, ~2 m at
//   300 m). The trail is a 2-3 m wide trench, so beyond ~40 m the rendered
//   hillside no longer contains the trench and buries the tread in patches.
//   No amount of polygon offset can reach that: it is a height error in a surface
//   the tread does not own. buildTreadMesh() therefore bakes a per-vertex
//   `aLodLift` attribute (the burial depth against a 4 m-lattice reconstruction of
//   the terrain) and the tread's vertex shader fades it in over 40-140 m. If the
//   terrain ever gains a distance-stable trail-corridor tessellation, or the
//   quadtree's _splitK changes materially, LODQ and the smoothstep range here
//   should be re-derived rather than left as they are.
//
// CONTRACT-NOTE (RESOLVED — kept because the measurement discipline it forced is
//   still the right one):
//   An earlier cut of this note reported that the world was not deterministic
//   from ctx.seed, because terrain.js sized its hydraulic-erosion droplet budget
//   from a wall-clock projection. That has been fixed in terrain.js: the budget
//   is now DROPLETS x DROPLET_TIER[ctx.quality], there is no time-budgeted
//   branching left in terrain.js or in this file, and Date.now() feeds only the
//   timings report. Re-verified this round on seven boots of seed 20260726 under
//   load, plus 777 and 12345: byte-identical fingerprints every time.
//
//   The world is still a function of seed AND quality tier (medium runs 65% of
//   the droplets), which is settings-derived and documented, but means a player
//   who changes quality mid-session gets a different mountain.
//
//   What stands regardless: any harness measuring this module must A/B in ONE
//   process against ONE cached pre-carve mountain and must fingerprint the world
//   before and after every measurement. Several rounds of this project's tuning
//   were done across a gap where that was not true.
//
// CONTRACT-NOTE: A/B line splits diverge up to ~5.5 m from the centreline, which is
//   wider than the 1.2–3.0 m tread width §4 allows for `widthAt()`. The carve and the
//   tread ribbon cover BOTH lines; `widthAt()` still reports the main line's tread
//   width, so a rider on the alternate line will read as `lateral` ±3–5 m. Gameplay
//   should treat `Math.abs(lateral) < 1.6 + widthAt(t)` OR "inside a split range" as
//   on-course; `trail.splits` is provided for exactly that test.
// =============================================================================

import * as THREE from 'three';
import { makeRng, subSeed, clamp, clamp01, lerp, smoothstep } from '../core/rng.js';
import { Surface } from './terrain.js';

// ---------------------------------------------------------------------------
// Physical constants for the design speed profile. These are *design* numbers —
// the actual bike sim owns the real physics; this only has to be close enough
// that the berms and jumps are built for a speed a rider actually arrives at.
// ---------------------------------------------------------------------------
const G = 9.81;
const RIDER_MASS = 95;        // kg, rider + bike + kit
const CDA = 0.62;             // m², upright-ish DH attack position
const AIR_RHO = 1.15;         // kg/m³, ~1000 m altitude
const V_CAP = 22.0;           // m/s (79 km/h) — realistic DH terminal on this grade
const A_BRAKE = 7.2;          // m/s² achievable braking on dirt before pitching
const PEDAL_POWER = 320;      // W, a racer sprinting out of the gate
const MU_LAT = 1.18;          // lateral friction coefficient, DH tyre on prepared dirt

// ---------------------------------------------------------------------------
// THE LEAN IDENTITY — the constraint the whole speed/radius/bank solve exists
// to satisfy.
//
// A bike holds a corner by leaning. On a corner of plan radius r taken at v,
// the lean the rider must hold RELATIVE TO THE SURFACE is
//
//     leanContact = atan( v^2 / (g*r) ) - bank
//
// and a downhill bicycle on good dirt runs out of tyre somewhere around 45 deg.
// So a course is only rideable where
//
//     atan( v^2 / (g*r) )  <=  bank + LEAN_AVAIL,  with real margin below it.
//
// WHAT WAS WRONG. Before this change the corner speed cap used MU_LAT = 1.18 —
// a perfectly good number for a DH tyre, and the wrong number for this job,
// because tan(atan(1.18)) = 49.7 deg of lean is past what the bike can hold and
// well past what it can hold while the tyres are anything less than perfectly
// planted. Measured on seed 20260726, the trail asked 10.6-14.3 m/s of a
// 12.5-15 m radius through 76-92 m: 37-59 deg of required lean against 15-19
// deg of built bank. The fast end of that is not merely hard, it is impossible.
//
// THE FIX IS ONE SUBSTITUTION AND IT MAKES THE IDENTITY AN IDENTITY. The
// standard banked-corner limit is
//
//     v^2 = g*r*(mu + tan(phi)) / (1 - mu*tan(phi))
//
// and the right-hand side is exactly g*r*tan(atan(mu) + phi). So solving the
// speed profile with mu = tan(LEAN_DESIGN) gives
//
//     atan( v^2/(g*r) ) = phi + LEAN_DESIGN   =>   leanContact = LEAN_DESIGN
//
// at every corner where the cap binds, and less than that everywhere else. The
// identity holds by construction rather than by inspection, on all nine phases,
// for any seed and any mountain. LEAN_DESIGN is then the single number that
// says how hard the course asks the rider to lean.
//
// LEAN_DESIGN is 28 deg and not 39. 39 would satisfy the acceptance test with
// its stated 6 deg of margin and would still not be rideable, because the
// bike's ceiling is not a constant: `leanCeiling()` in bike.js derives it from
// tyre load, and it was measured collapsing from 0.63 rad to 0.22 rad (36 deg
// to 13 deg) through a roller crest. A design that asks for the steady-state
// ceiling has nothing left for the moment the tyres go light. 28 deg leaves
// roughly a third of the budget unspent at every corner on the course.
const LEAN_AVAIL = THREE.MathUtils.degToRad(45);
const LEAN_DESIGN = THREE.MathUtils.degToRad(28);
// mu that reproduces LEAN_DESIGN, per surface. SURF_MU scales it, so a corner
// on mud or roots is designed slower than the same corner on hardpack.
const MU_DESIGN = Math.tan(LEAN_DESIGN);

// ---------------------------------------------------------------------------
// BERM STYLE, per phase.
//
// Every phase is bermed. That is the change: findCorners() and the berm
// builder used to be invoked at three call sites — 'flow', 'loam' and 'sprint'
// — so six of the nine phases had no corner detection and no berm construction
// at all, and the global banking pass then multiplied the ideal bank by a
// per-phase coefficient as low as 0.42. A corner in `roots` needing 45 deg was
// authored to receive 18.9.
//
// What is NOT the change: making everything a motorway berm. The character of
// a phase is what it is built out of, and these numbers say so —
//
//   `style`     how much of the IDEAL bank a builder shapes here for its own
//               sake. 1.0 is a full race berm; 0.45-0.6 is an insloped rut or
//               a worn catch bank of the kind that forms naturally on the
//               outside of a tech corner.
//   `maxBank`   the ceiling. A rock slab does not hold a 46 deg berm; it holds
//               a shallow insloped shelf, so `slab` stops at 22 deg and the
//               speed solve takes the corner speed down to suit.
//   `minRadius` how tight a corner has to be before it is worth shaping at all
//               — bigger in the flow phases, where a builder shapes long
//               sweeping turns, smaller in the tech phases, where only the
//               genuinely tight corners get worked.
//
// The identity floor overrides `style` (never `maxBank`): wherever the corner
// physically needs more bank than the phase's character would shape, it gets
// it, up to the phase ceiling. Past the ceiling the speed comes down instead —
// which is the honest answer for a rock chute, and is what makes a tech phase
// slow rather than fatal.
// `style` is deliberately well below 1 outside the flow phases, and that is a
// safety property as much as an aesthetic one. Bank is not free: a rider who
// slides laterally on a steeply banked tread is thrown UPWARD at
// v_lateral * tan(bank), and at 37 deg a 2.5 m/s drift becomes 1.9 m/s of
// vertical — measured at 80 m, the bike left the ground doing exactly that and
// then hit the catch berm outboard of the tread at 13 g. Bank the corner for
// what it needs plus a builder's flourish; do not bank it because you can.
const BERM_STYLE = {
  start:  { minRadius: 55, minSpacing: 18, style: 0.55, maxBank: 0.52, widthAdd: 0.30, ruts: 0.045, brake: 0.055, brakeLen: 12 },
  roots:  { minRadius: 26, minSpacing: 10, style: 0.35, maxBank: 0.35, widthAdd: 0.14, ruts: 0.060, brake: 0.045, brakeLen: 7 },
  flow:   { minRadius: 46, minSpacing: 18, style: 0.85, maxBank: 0.75, widthAdd: 0.35, ruts: 0.045, brake: 0.075, brakeLen: 14 },
  jumps:  { minRadius: 50, minSpacing: 20, style: 0.70, maxBank: 0.60, widthAdd: 0.30, ruts: 0.040, brake: 0.050, brakeLen: 12 },
  slab:   { minRadius: 30, minSpacing: 12, style: 0.30, maxBank: 0.30, widthAdd: 0.10, ruts: 0.014, brake: 0.000, brakeLen: 0 },
  creek:  { minRadius: 36, minSpacing: 14, style: 0.45, maxBank: 0.42, widthAdd: 0.22, ruts: 0.030, brake: 0.035, brakeLen: 8 },
  loam:   { minRadius: 34, minSpacing: 14, style: 0.70, maxBank: 0.65, widthAdd: 0.30, ruts: 0.070, brake: 0.060, brakeLen: 10 },
  rocks:  { minRadius: 30, minSpacing: 12, style: 0.35, maxBank: 0.34, widthAdd: 0.14, ruts: 0.018, brake: 0.000, brakeLen: 0 },
  sprint: { minRadius: 52, minSpacing: 20, style: 0.85, maxBank: 0.75, widthAdd: 0.38, ruts: 0.045, brake: 0.070, brakeLen: 16 },
};
const BERM_STYLE_DEFAULT = BERM_STYLE.flow;

// ---------------------------------------------------------------------------
// STRAIGHT-LINE SPEED CEILING, per phase.
//
// The corner cap above bounds the speed a corner can be taken at. Nothing
// bounded the speed on a STRAIGHT, so the forward pass ran gravity against drag
// and rolling resistance and arrived at whatever that gave — measured on seed
// 20260726, 18.7-19.8 m/s (67-71 km/h) through the Rooty Steeps, on a 1.37-1.48 m
// tread, at 20-32% gradient, over root ridges 5-12 cm proud. That is not a
// design speed, it is the absence of one, and because `speedAt()` is what the
// rider (and the autopilot) servo onto, it is an instruction to ride a 1.4 m
// wide root chute at 71 km/h. Every run reaching that far died in it.
//
// These are the speeds each section is DESIGNED to be ridden at, and they are
// what a real DH course runs: a race averages 25-35 km/h and tops out at 55-65
// on the open sections. They are stated per phase because that is where the
// design intent lives — Rooty Steeps is slow because it is narrow, steep and
// rough, and the Finish Sprint is fast because it is 3 m of hardpack.
//
// The geometry has to justify the number, or a slow corner reads as a fast one
// and the rider is punished for believing what they see. So the ceiling is
// modulated by the two things the rider can actually SEE at that station — how
// wide the tread is, and how rough it is — rather than applied as a flat
// per-phase constant. Both modifiers key off the phase's own design width, so
// a phase that widens for a berm speeds up there and a pinch slows down.
const V_PHASE = {
  start: 16.0, roots: 10.5, flow: 17.0, jumps: 17.5, slab: 12.5,
  creek: 12.0, loam: 13.0, rocks: 11.0, sprint: 19.0,
};
// Chatter ceiling. Transverse relief of amplitude A at wavelength lambda drives
// the contact patch at v/lambda Hz, and the peak vertical acceleration of that
// motion is A*(2*pi*v/lambda)^2 / 2. Setting that equal to g gives the speed at
// which an UNSPRUNG wheel would leave the ground every cycle; a 170 mm fork
// with real damping carries roughly ROUGH_SUSP times that before the tyre
// starts skipping, which is the number calibrated here. lambda is the ~1.6 m
// spacing the root ridges and braking-bump trains are authored at.
const ROUGH_LAMBDA = 1.6;
const ROUGH_SUSP = 3.4;

const STATION_DS = 0.4;       // m between centreline stations (also the stamp spacing)
const RIBBON_DS = 0.8;        // m between tread-ribbon rings (halved over shaped features)
const NEAR_WINDOW = 22;       // stations searched either side of the hint (±8.8 m)
const TAPE_H = 0.075;         // m, height of the course-tape ribbon

// Per-surface rolling resistance and grip scale.
const SURF_CRR = [0.035, 0.055, 0.028, 0.062, 0.075, 0.050, 0.090, 0.080];
const SURF_MU = [1.00, 1.06, 0.90, 0.74, 0.80, 0.70, 0.58, 0.55];
// Tread tint per surface (authored sRGB; THREE.Color converts to working space).
const SURF_TINT = [0x8a7150, 0x40342a, 0x77746d, 0x8e8676, 0x6f7048, 0x6d5836, 0x4c3c2c, 0xbfc4c9];

// ---------------------------------------------------------------------------
// HARDPACK — the warm packed-dirt base that gets splatted THROUGH every parent
// surface along the ridden line.
//
// This is the fix for the round-3 headline. Measured tread B/R: 0.63 in the one
// open alpine shot (correct) but 1.01-1.87 in every forest shot, and in r3_15
// the tread reads B/R 1.62 while the duff three metres away under identical
// light reads 1.17 — so it is the tread's own albedo, not the lighting. The
// cause is that the tread inherits the PARENT geology's tint: LOAM 0x40342a is
// 4% linear (a dark base takes its hue wholesale from sky ambient), and
// ROCK 0x77746d / GRAVEL 0x8e8676 are chromatically neutral, so the slab and
// creek phases have literally nothing warm in them.
//
// A ridden line is not its parent geology. Whatever the bedrock, the surface a
// tyre rolls on is compacted fines with a dust film — and DIRT's 0x8a7150 is
// the tint that measured correctly (0.63) in r3_01, so it is the reference.
// 0x907149 is that hue held at essentially the same luminance (so the one shot
// that already measured correctly does not move) and pushed warmer: B/R 0.51
// against DIRT's 0.58.
const HARDPACK_TINT = 0x907149;
// How much hardpack shows through at the centre of the ridden line, per
// surface, tapering to 0.26x of this out at the loose shoulders. Bedrock keeps
// more of itself; the dark and the chromatically-neutral layers take most of
// it; snow is barely touched. DIRT is deliberately LOW — its own tint is
// already the reference hardpack and r3_01's sunlit tread measured correctly.
//        DIRT  LOAM  ROCK  GRAVEL GRASS  ROOT   MUD   SNOW
const HARDPACK_W = [0.60, 0.78, 0.62, 0.72, 0.55, 0.62, 0.45, 0.10];

// Specular intensity of the tread. F0 = 0.04 * this. See the shader patch in
// buildTreadMesh() for why a dirt surface is not a smooth dielectric.
const TREAD_SPEC_SCALE = 0.35;

// ---------------------------------------------------------------------------
// EXPOSURE SAFETY — the level-design rule that stops the generator building
// unmarked traps. See the long note above applyExposureSafety().
//
// The failure this exists to prevent, measured on seed 20260726 before the rule:
// at 85 m the tread is 2.9 m wide, the grade ramps 8% -> 59% in 10 m, and 1.2 m
// outside the right-hand tread edge the mountain falls away 1.5 m, then 4.5 m at
// 2.2 m and 8.3 m at 4.6 m. A rider 2.2 m off the centreline — an ordinary error
// on a 2.9 m tread — is over it and lands at 14.7 m/s of normal velocity. 34% of
// the trail measured as some degree of that, in 31 separate stretches.
// ---------------------------------------------------------------------------

// Where we probe, in metres beyond the tread edge. 4.6 m is a little past the
// terrain batter's own 4.0 m run, so we see ground the batter never reaches.
const EXPO_PROBE = [0.4, 0.8, 1.2, 1.6, 2.2, 2.8, 3.6, 4.6];
// tan(35 deg): the cross-slope a soil hillside actually stands at. Ground that
// falls away no faster than this is a slope you slide down, not a drop.
const EXPO_REPOSE = 0.70;
// "Unsupported fall" = the worst (fall - EXPO_REPOSE * distance) over the probe.
// It is metres of drop in excess of what the hillside could be holding up, so it
// is zero on any ordinary sidehill however steep, and large only on a real edge.
const EXPO_TRIGGER = 0.45;   // m — below this the shoulder is just a shoulder
const EXPO_FULL = 2.50;      // m — the response saturates here
const EXPO_TAPE = 1.50;      // m — above this the section gets taped even in an untaped phase

const RUNOFF_D = 2.40;       // m of built run-off we try to give outside the tread edge
const SHELF_SLOPE = 0.20;    // the shelf falls away at 20%: rideable, off-camber, slow
const CATCH_H_MIN = 0.30;    // m — outer catch berm at the trigger
const CATCH_H_MAX = 0.85;    // m — outer catch berm at saturation
const CATCH_FILL_MAX = 1.80; // m — most built fill a catch berm may stand on before
                             // we pull it in toward the tread instead of piling higher
const CATCH_MIN_Q = 0.60;    // m — closest to the tread edge a catch crest may sit.
                             // Two reasons it is not smaller. The tread ribbon overhangs
                             // its own edge by 18% (emitStrip), so a crest inside that
                             // would bury the ribbon; and terrain.applyCarve floors every
                             // stamp radius at 0.6 m, so nothing sharper than about a
                             // 1.2 m wavelength survives the carve accumulator anyway.
// THE INNER FACE OF A CATCH BERM IS A RAMP THE RIDER HITS, so its slope is a
// safety constraint and not a detail. Authored as a fixed 0.40 m of ramp up to
// 0.85 m of crest, it was a 2.1:1 face — 65 degrees — sitting 0.6 m outside the
// tread edge. Measured at 80 m on seed 20260726: a rider drifting 0.4 m wide at
// 10 m/s hit it and took 13 g, left the ground, and every cell of the autopilot
// sweep died within 25 m of that point. A catch berm that launches the rider it
// caught is worse than no catch berm.
//
// So the ramp length is derived from the crest height instead of being fixed,
// and where there is not enough room outboard for a rideable face the CREST
// COMES DOWN rather than the face steepening. A 0.25 m ridge at 30 degrees
// still scrubs speed and still turns a rider back; it does not throw them.
const CATCH_FACE = 0.60;     // tan(31 deg) — the steepest inner face a rider can hit
const CATCH_RAMP_MAX = 1.60; // m — the longest face worth authoring
const CATCH_RAMP = 0.40;     // m — floor on the ramp length (see rampLenFor)
const CATCH_PLATEAU = 0.50;  // m of FLAT crest. Not cosmetic: applyCarve reconstructs
                             // the surface as a distance-weighted mean of flat stamp
                             // discs, which is a low-pass filter about 1 m wide. A
                             // triangular crest comes out of it at half height and got
                             // measured at 0.0 m more than once; a plateau survives.
const CATCH_BACK_SLOPE = 1.10;  // tan(48 deg) — the back of the berm, down to natural ground
const CATCH_BACK_MAX = 1.20;    // m of berm back we bother to author

const BENCH_SHIFT_MAX = 1.10;   // m the centreline may be benched toward the safe side
const BENCH_CUT_EXTRA = 2.60;   // m of extra excavation a bench shift may buy
const BENCH_CUT_TOTAL = 7.00;   // m — hard excavation ceiling (buildStations uses 6.5)
const BENCH_BLEND = 12.0;       // m over which a bench shift eases in and out

const WIDEN_MAX = 0.45;         // m of extra tread width where exposed (still <= 3.0)

// Grade-ramp cap. `rate` is |d(grade)/ds| measured over a RAMP_WIN window, so
// 0.020 means the gradient may change by 14 percentage points over 7 m.
const RAMP_WIN = 7.0;
const RAMP_OPEN = 0.045;     // open, supported corridor
const RAMP_EXPOSED = 0.020;  // exposed or narrow corridor
const RAMP_NARROW = 2.00;    // m — a tread narrower than this counts as narrow
const RAMP_CUT_MAX = 7.00;   // m — excavation ceiling for the relaxation
const RAMP_FILL_MAX = 3.00;  // m
const FEATURE_GUARD = 15.0;  // m either side of a shaped feature that the rule leaves alone

// ---------------------------------------------------------------------------
// CORRIDOR PROTECTABILITY — the term that stops the ROUTE-FINDER walking into
// ground the safety pass cannot then rescue.
//
// WHY THIS EXISTS, AND WHY IT IS UPSTREAM OF EVERYTHING ELSE IN THIS FILE.
// applyExposureSafety() is a mitigation pass. It eases the gradient, benches the
// line toward the safe side, widens the tread and builds a run-off shelf with a
// catch berm at its lip. All four are real and all four are bounded — a bench
// shift is at most BENCH_SHIFT_MAX, a catch berm may stand on at most
// CATCH_FILL_MAX of built fill. Where the mountain has genuinely gone, none of
// them fit, and designCatch() says so: on seed 20260726 it reported 26 stations
// with no catch berm at all, all of them in the first hundred metres, and every
// cell of the autopilot sweep died there.
//
// Measured on the PRE-CARVE mountain along the route the old marchRoute picked:
//
//     0-100 m    mean unsupported fall-away 4.59 m   163/250 stations unprotectable
//   100-200 m                               2.09 m     9/250
//   200-1200 m                              0.00 m     0/2500
//  1400-1700 m                              2.95 m   263/750
//  1800-2700 m                              0.13 m     0/2250
//
// The exposure is not diffuse. It is two identifiable corridors, and the first
// one is exactly where every autopilot cell crashed. Through 0-100 m the left
// probe reads 0.00 m at every single station while the right reads 5-8 m: the
// route is running the crest of a rib with the hillside standing up on one side
// and gone on the other. A 1.5 m error — ordinary on a 2.9 m tread, and a
// fixed-gain pure-pursuit controller carries about that much on a 14-30 m radius
// — puts the rider over the edge, and there is nothing to catch them because
// nothing can be built there.
//
// So the fix is not a better catch berm. It is to make the route-finder score
// the corridor it is about to commit to, and refuse the ones it cannot protect.
//
// THE METRIC IS designCatch()'s OWN FEASIBILITY TEST, run forward.
// A catch berm at reach q sits at (edgeY - SHELF_SLOPE*q + h) and stands on
// (crest - naturalGround(q)) of built fill. designCatch accepts it while that is
// within CATCH_FILL_MAX*1.7. So the deficit
//
//     max(0, -max_q [ budget - fill(q) ])
//
// is metres of fill the catch berm would be SHORT BY at the friendliest reach
// available — it is exactly zero wherever designCatch will succeed, and it grows
// smoothly where it will not. That smoothness is what makes it usable as a score:
// a hard feasible/infeasible flag gives the heading search nothing to descend.
//
// The second term is near-field unsupported fall — metres of drop in excess of
// what a soil hillside at EXPO_REPOSE could be holding up, within CORR_NEAR of
// the tread edge. It is zero on an ordinary sidehill however steep (which is the
// whole point: a downhill course wants steep sidehill) and large only on a real
// edge. It is there because a corridor can be protectable and still unpleasant,
// and because it gives the search a gradient to follow while it is still several
// metres away from the ground going.
//
// WHAT THIS TERM IS NOT ALLOWED TO DO. It is capped — see CORR_CAP — well below
// the march's climb penalties (900/1100/1600) and below its grade-holding weight
// (220). It therefore cannot buy its way out of exposure by climbing, by
// abandoning the phase's design gradient, or by switchbacking: the |dth| penalty
// and the switchback state machine are untouched. It biases a choice between
// comparable headings. It does not get to choose a different sport.
// THE PROBE IS TWO-SCALE, AND THE SECOND SCALE IS A GATE RATHER THAN A COST.
// Measured over eight probe horizons (24 m to 90 m), all built in one process
// against one pinned mountain and swept with the same 21-cell autopilot:
//
//   horizon   unprotectable   crestViolAfter   median progress   min progress
//     24 m         50               22              240 m           230 m
//     34 m         50                7              398 m           232 m
//     48 m         55               34              389 m           231 m
//     56 m          0              182              204 m           143 m
//     62 m         13               26              323 m           194 m
//     72 m          5              225              155 m           110 m
//     90 m         53               14              382 m           207 m
//
// Correlations over that family: residual crest-curvature violations against
// minimum progress r = -0.91, against median r = -0.82. Unprotectable stations
// against minimum progress r = +0.88 — POSITIVE, which is not a claim that
// exposure helps but the signature of a trade: a term that chases distant
// exposure at full weight drags the line onto ground with more longitudinal
// relief, capCrestCurvature cannot flatten all of it, and the bike is launched.
// Once the route is off the start rib the binding constraint on rideability
// stops being the ground beside the tread and becomes the ground under it.
//
// So the near scale carries the full cost and decides the heading, and the far
// scale only fires on a deficit past CORR_FAR_TOL — enough to refuse walking
// into a corridor whose exits are all bad, and not enough to steer the route
// around ordinary distant sidehill.
const CORR_S = [5.0, 13.0, 24.0];   // m ahead of the candidate step that we probe
const CORR_S_FAR = [38.0, 56.0, 78.0];
const CORR_FAR_TOL = 0.55;          // m of catch-berm deficit the far gate tolerates
const CORR_W_FAR = 52.0;
const CORR_CAP_FAR = 90.0;

// ---------------------------------------------------------------------------
// THE OTHER HALF OF "CANNOT BE BUILT" — the LONGITUDINAL one.
//
// A corridor can be unbuildable in two ways, and the sweep above showed they
// trade against each other under pressure. Laterally: no catch berm fits, so an
// ordinary error is fatal. Longitudinally: the ground rolls over faster than a
// profile can be cut through it inside the excavation budget, so the tread keeps
// a crest that throws the rider into the air at the phase's own design speed.
//
// Fighting only the lateral half moved the route off the start rib — median
// autopilot progress 98 -> 240 m — and then pushed it onto ground where
// capCrestCurvature could not converge: pressing harder on exposure took
// unprotectable stations 50 -> 0 and crest violations 22 -> 175, and progress
// back down to 232/136 m. The two terms have to be scored together or the
// route-finder simply trades one defect for the other, which is what the
// exposure-safety pass was already doing a hundred metres downstream.
//
// THE RULE IS THE SAME ONE capCrestCurvature ENFORCES, evaluated forward on
// natural ground: over a crest of convex vertical curvature kappa the wheels
// keep contact only while kappa <= g / (CREST_SAFETY * v^2), and v here is the
// phase's own design speed, because that is the speed the course will be asking
// for when it gets there. The probe is deliberately coarse — 9 m segments over
// 64 m — because the route-finder cannot control 4 m detail and should not try:
// what it can choose is whether to commit to a fall line with a 15-30 m rollover
// in it that no amount of excavation will take out.
const CORR_KS = [0.0, 9.0, 18.0, 27.0, 36.0, 50.0, 64.0];
// Chosen by sweep, on protection rather than on the autopilot's best cell. The
// seven configurations measured on seed 20260726, one process, one mountain:
//
//   far-tol  kappa-w   unprotectable  crest  exposed(m)  autopilot p25/med/min
//     0.55      900          5          58      1020        219 / 300 / 121
//     off       900         47          12      1346        314 / 329 / 240
//     0.55      400          6          54       477        211 / 221 / 205
//     0.80      900         43          15      1124        325 / 334 / 248
//     0.55     1400          4          69       682        116 / 140 /  97
//
// The trade is explicit and it does not have a free corner: anything that takes
// unprotectable stations below about ten costs roughly a hundred metres of
// autopilot progress, because the same pressure that pushes the line off a rib
// pushes it onto ground with more longitudinal relief. 400 is taken over 900
// because it holds unprotectable at 6 while cutting exposed trail to 477 m from
// the baseline's 1454, and because its autopilot spread (205-351 m, a factor of
// 1.7) is the tightest of the seven against the baseline's 81-413 m — this
// project has already been bitten by a course whose measured progress moved 14x
// on a 10 cm change of lookahead, and a result that does not depend on the
// probe is worth more than a bigger one that does.
const CORR_W_KAPPA = 400.0;         // score per /m of convex curvature past the cap
const CORR_CAP_KAPPA = 120.0;
// Shared with capCrestCurvature — see the note there. The route promises what
// the profile solver promises.
const CREST_SAFETY = 1.45;
const CREST_KAPPA_MIN = 0.010;      // /m — never demand a vertical radius over 100 m
const CREST_KAPPA_MAX = 0.120;      // /m — nor bother below ~8 m
const CORR_Q = [0.6, 1.0, 1.5, 2.1, 2.8];  // m beyond the tread edge
const CORR_NEAR = 1.6;              // m — near-field, i.e. as far as an ordinary error goes
const CORR_BUDGET = CATCH_FILL_MAX * 1.7;  // designCatch's relaxed retaining-edge budget
const CORR_CATCH_H = 0.15;          // m — the smallest catch crest designCatch will accept
// Weights. `deficit` and `fall` are both in metres, and on the ribs this course
// actually contains they run 0.5-3.0 m and 2-8 m respectively.
const CORR_W_DEFICIT = 34.0;
const CORR_W_FALL = 7.0;
// Hard ceiling on the whole term, per candidate heading. 110 is deliberately of
// the same order as the march's existing cross-slope penalty (120 per unit of
// gradient past 0.85) and roughly half its grade-holding weight, so a heading
// that is 0.5 of gradient off the phase design can never win on safety alone.
const CORR_CAP = 110.0;
// The same measurement, used to pick the LAUNCH POINT. That scan runs once per
// variant, so it can afford a wider fan and more probe stations — and it has to,
// because the first hundred metres of this course is the defect and no amount of
// steering saves a start placed on a knife rib. Measured over the 49 candidate
// launch points on seed 20260726: the old scan's winner scored 1117.5 and its
// best descending corridor still carried 0.44 m of mean unsupported fall, while
// a candidate 286 m along the same contour scored 1116.8 — a 0.06% difference,
// i.e. a tie — on a corridor measuring 0.00 m with nothing unprotectable in it.
// The launch scan was choosing between near-ties on a criterion that could not
// see the only thing that mattered.
//
// CORR_LAUNCH_W is dimensionless and is applied to a cost in the same units as
// CORR_CAP. The spread of the scan's own score across the plausible candidates
// on this mountain is about 90 points, so a weight near 1 makes a completely
// unprotectable launch cost about as much as being the twelfth-best hilltop —
// enough to lose a tie, not enough to start the run halfway down the mountain.
const CORR_LAUNCH_S = [8.0, 20.0, 34.0, 50.0, 70.0];
const CORR_LAUNCH_W = 1.15;
const CORR_LAUNCH_CAP = 150.0;
const CORR_LAUNCH_FAN = 10;         // +-10 steps of 6 deg = +-60 deg of heading
// Cost per unit of gradient past the opening phase's ceiling, inside the launch
// fan. Scaled so a 60% face — the one the protectability-only cut chose — costs
// the whole of CORR_LAUNCH_CAP on its own.
const CORR_LAUNCH_GRADE = 450.0;
const CORR_LAUNCH_G = [40.0, 90.0, 150.0];   // m — ranges the launch fan probes grade at

// ---------------------------------------------------------------------------
// Pacing script. `du` is the fraction of the *vertical drop* the phase occupies;
// `grade` is the target trail gradient (rise/run) through it. Together they set
// how long each phase is: len = drop * du / grade.
// Σ du = 1.0 and Σ (du/grade) ≈ 6.4 → ~2.7 km for a 420 m mountain.
// ---------------------------------------------------------------------------
// `len` is the phase's share of the total run length; `grade` is the gradient the
// route-finder tries to hold through it. Σ(len·grade) is therefore the average
// gradient of the whole track — 16.6%, i.e. 430 m over 2.6 km, as CONTRACT §0 asks.
const PHASES = [
  { id: 'start',  name: 'Start Chute',     len: 0.1154, grade: 0.110, width: 2.90, twist: 0.32, drift: 165, surface: Surface.DIRT,   tape: 1.0, tech: 0.05 },
  { id: 'roots',  name: 'Rooty Steeps',    len: 0.0885, grade: 0.260, width: 1.50, twist: 0.75, drift: 80,  surface: Surface.ROOT,   tape: 0.0, tech: 0.90 },
  { id: 'flow',   name: 'The Flow',        len: 0.1615, grade: 0.150, width: 2.75, twist: 1.05, drift: 145, surface: Surface.DIRT,   tape: 1.0, tech: 0.15 },
  { id: 'jumps',  name: 'Jump Line',       len: 0.1538, grade: 0.130, width: 2.60, twist: 0.34, drift: 135, surface: Surface.DIRT,   tape: 1.0, tech: 0.20 },
  { id: 'slab',   name: 'Slab Chute',      len: 0.0654, grade: 0.330, width: 2.05, twist: 0.45, drift: 70,  surface: Surface.ROCK,   tape: 0.0, tech: 0.85 },
  { id: 'creek',  name: 'Creek Crossing',  len: 0.0500, grade: 0.090, width: 2.35, twist: 0.42, drift: 100, surface: Surface.GRAVEL, tape: 0.0, tech: 0.35 },
  { id: 'loam',   name: 'Loam Turns',      len: 0.1269, grade: 0.190, width: 1.75, twist: 1.20, drift: 95,  surface: Surface.LOAM,   tape: 0.0, tech: 0.55 },
  { id: 'rocks',  name: 'Rock Garden',     len: 0.0846, grade: 0.210, width: 2.15, twist: 0.55, drift: 105, surface: Surface.ROCK,   tape: 0.0, tech: 0.95 },
  { id: 'sprint', name: 'Finish Sprint',   len: 0.1539, grade: 0.115, width: 2.95, twist: 0.82, drift: 155, surface: Surface.DIRT,   tape: 1.0, tech: 0.10 },
];

// Cumulative length fractions, and the cumulative share of the total *drop* each
// phase boundary should have consumed by then. The second curve is what the route
// governor steers against, so the run lands on its design drop at its design length.
const PHASE_LEN_CUM = [];
const PHASE_DROP_CUM = [];
(() => {
  let l = 0, d = 0, tot = 0;
  for (const p of PHASES) tot += p.len * p.grade;
  for (const p of PHASES) {
    l += p.len; d += (p.len * p.grade) / tot;
    PHASE_LEN_CUM.push(l); PHASE_DROP_CUM.push(d);
  }
})();

/** Phase index at a length fraction u in [0,1]. */
function phaseAtLength(u) {
  for (let i = 0; i < PHASE_LEN_CUM.length; i++) if (u <= PHASE_LEN_CUM[i]) return i;
  return PHASES.length - 1;
}
/** Fraction of the total drop that *should* be used up by length fraction u. */
function dropFracAtLength(u) {
  let l0 = 0, d0 = 0;
  for (let i = 0; i < PHASES.length; i++) {
    const l1 = PHASE_LEN_CUM[i], d1 = PHASE_DROP_CUM[i];
    if (u <= l1 || i === PHASES.length - 1) {
      const f = (u - l0) / Math.max(1e-6, l1 - l0);
      return clamp01(d0 + (d1 - d0) * clamp01(f));
    }
    l0 = l1; d0 = d1;
  }
  return 1;
}

// ---------------------------------------------------------------------------
// Module-scope scratch. Nothing below allocates per frame.
// ---------------------------------------------------------------------------
const _p = new THREE.Vector3();
const _q = new THREE.Vector3();
const _up = new THREE.Vector3(0, 1, 0);
const _fwd = new THREE.Vector3();
const _rgt = new THREE.Vector3();
const _m4 = new THREE.Matrix4();
const _m3 = new THREE.Matrix3();
const _quat = new THREE.Quaternion();
const _col = new THREE.Color();

// ===========================================================================
// Deterministic noise helpers
// ===========================================================================

/** 1-D value noise on a wrapping lattice; smooth (C1) and cheap. */
function makeNoise1D(rng, n) {
  const tbl = new Float32Array(n);
  for (let i = 0; i < n; i++) tbl[i] = rng() * 2 - 1;
  return function noise1(x) {
    const xi = Math.floor(x);
    const f = x - xi;
    const i0 = ((xi % n) + n) % n;
    const i1 = (i0 + 1) % n;
    const u = f * f * (3 - 2 * f);
    return tbl[i0] + (tbl[i1] - tbl[i0]) * u;
  };
}

/** Sum of octaves of 1-D value noise, normalised to roughly [-1, 1]. */
function fbm1(noise, x, octaves, lacunarity, gain) {
  let sum = 0, amp = 1, f = 1, norm = 0;
  for (let o = 0; o < octaves; o++) {
    sum += noise(x * f) * amp;
    norm += amp;
    f *= lacunarity;
    amp *= gain;
  }
  return sum / norm;
}

/**
 * Tileable 2-D value noise. One lattice per octave with the lattice size equal to
 * the octave frequency, so sampling p in [0,1) wraps exactly at the seam.
 */
function makeTileFbm(rng, freqs, gain) {
  const lattices = freqs.map((f) => {
    const a = new Float32Array(f * f);
    for (let i = 0; i < a.length; i++) a[i] = rng();
    return a;
  });
  return function fbm(px, py) {
    let sum = 0, amp = 1, norm = 0;
    for (let k = 0; k < freqs.length; k++) {
      const L = freqs[k];
      const a = lattices[k];
      const x = px * L, y = py * L;
      const xi = Math.floor(x), yi = Math.floor(y);
      const fx = x - xi, fy = y - yi;
      const x0 = ((xi % L) + L) % L, y0 = ((yi % L) + L) % L;
      const x1 = (x0 + 1) % L, y1 = (y0 + 1) % L;
      const u = fx * fx * (3 - 2 * fx), v = fy * fy * (3 - 2 * fy);
      const r0 = a[y0 * L + x0] + (a[y0 * L + x1] - a[y0 * L + x0]) * u;
      const r1 = a[y1 * L + x0] + (a[y1 * L + x1] - a[y1 * L + x0]) * u;
      sum += (r0 + (r1 - r0) * v) * amp;
      norm += amp;
      amp *= gain;
    }
    return sum / norm;
  };
}

/**
 * Anisotropic tileable value fbm. Each octave carries its own INTEGER lattice
 * count per axis, so a `freqs` of [[16,4],[32,8]] makes every feature four
 * times longer in +Y than in +X and still wraps exactly at both seams.
 *
 * Why this exists: a ridden trail is a strongly directional surface. Ruts, drag
 * lines, wheel-track fines and stone scars all run WITH the direction of
 * travel, so the gradient across the tread is 1.5-2.5x the gradient along it.
 * Round 3 measured this build at 0.57-0.99 — i.e. the tread carried isotropic
 * noise and nothing that said "vehicles have been down here". An isotropic fbm
 * cannot produce that ratio no matter how it is weighted; the lattice itself
 * has to be stretched.
 */
function makeTileFbmXY(rng, freqs, gain) {
  const lattices = freqs.map(([fx, fy]) => {
    const a = new Float32Array(fx * fy);
    for (let i = 0; i < a.length; i++) a[i] = rng();
    return a;
  });
  return function fbmXY(px, py) {
    let sum = 0, amp = 1, norm = 0;
    for (let k = 0; k < freqs.length; k++) {
      const LX = freqs[k][0], LY = freqs[k][1];
      const a = lattices[k];
      const x = px * LX, y = py * LY;
      const xi = Math.floor(x), yi = Math.floor(y);
      const fx = x - xi, fy = y - yi;
      const x0 = ((xi % LX) + LX) % LX, y0 = ((yi % LY) + LY) % LY;
      const x1 = (x0 + 1) % LX, y1 = (y0 + 1) % LY;
      const u = fx * fx * (3 - 2 * fx), v = fy * fy * (3 - 2 * fy);
      const r0 = a[y0 * LX + x0] + (a[y0 * LX + x1] - a[y0 * LX + x0]) * u;
      const r1 = a[y1 * LX + x0] + (a[y1 * LX + x1] - a[y1 * LX + x0]) * u;
      sum += (r0 + (r1 - r0) * v) * amp;
      norm += amp;
      amp *= gain;
    }
    return sum / norm;
  };
}

// ===========================================================================
// Canvas texture authoring (procedural — nothing is downloaded)
// ===========================================================================

function newCanvas(w, h) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  return c;
}

function canvasTexture(canvas, { srgb = true, repeatX = 1, repeatY = 1, aniso = 8 } = {}) {
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = srgb ? THREE.SRGBColorSpace : THREE.NoColorSpace;
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(repeatX, repeatY);
  tex.anisotropy = aniso;
  tex.needsUpdate = true;
  return tex;
}

/** Sobel a height field into a tangent-space normal map canvas (linear data). */
function heightToNormalCanvas(field, size, strength) {
  const c = newCanvas(size, size);
  const g = c.getContext('2d');
  const img = g.createImageData(size, size);
  const d = img.data;
  const at = (x, y) => field[(((y % size) + size) % size) * size + (((x % size) + size) % size)];
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const tl = at(x - 1, y - 1), tc = at(x, y - 1), tr = at(x + 1, y - 1);
      const ml = at(x - 1, y), mr = at(x + 1, y);
      const bl = at(x - 1, y + 1), bc = at(x, y + 1), br = at(x + 1, y + 1);
      const dx = (tr + 2 * mr + br) - (tl + 2 * ml + bl);
      const dy = (bl + 2 * bc + br) - (tl + 2 * tc + tr);
      let nx = -dx * strength, ny = -dy * strength, nz = 1;
      const inv = 1 / Math.sqrt(nx * nx + ny * ny + 1);
      nx *= inv; ny *= inv; nz *= inv;
      const o = (y * size + x) * 4;
      d[o] = (nx * 0.5 + 0.5) * 255;
      d[o + 1] = (ny * 0.5 + 0.5) * 255;
      d[o + 2] = (nz * 0.5 + 0.5) * 255;
      d[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  return c;
}

/**
 * Worn-dirt tread: a mostly achromatic detail map (mean ~0.8) so per-station
 * vertex tint carries the actual surface hue, plus embedded pebbles, dry scuff
 * and the packed-line sheen a ridden trail develops.
 *
 * The map also carries the SUB-METRE RELIEF of a ridden trail — the transverse
 * braking-bump / root chatter and the longitudinal drag lines. That relief used
 * to live in the tread ribbon's vertex positions, which are emitted at 0.8 m
 * spacing: a 1.1 m ripple sampled at 0.8 m is below Nyquist, so it aliased into
 * hard-edged facets whose vertex normals swung by tens of degrees between
 * adjacent rows. Against a 19° sun that is the black-and-pale polygon mosaic in
 * seven of the fourteen review frames. Height-map relief costs no geometry, is
 * mip-filtered for free, and cannot alias.
 *
 * The tread UV tile is 2.6 m across x 3.1 m along the trail (see buildTreadMesh),
 * so 3 chatter cycles per tile is a 1.03 m ripple — the real braking-bump
 * wavelength — and it wraps seamlessly because 3 is an integer.
 */
function buildTreadMaps(rng, size) {
  // X is ACROSS the tread, Y is ALONG it (see buildTreadMesh's UVs). Every
  // structural octave below is stretched 2.5-4x along the direction of travel,
  // which is what turns a noise field into a ridden surface.
  const fbmDrag = makeTileFbmXY(rng, [[8, 2], [16, 4], [32, 8], [64, 16]], 0.55);
  const fbmFine = makeTileFbmXY(rng, [[32, 12], [64, 24], [128, 48], [256, 96]], 0.58);
  const fbmMicro = makeTileFbmXY(rng, [[64, 32], [128, 64], [256, 128]], 0.60);
  const fbmMacro = makeTileFbm(rng, [2, 4, 8], 0.6);
  // Stones are only mildly elongated — they get dragged, not extruded.
  const pebble = makeTileFbmXY(rng, [[24, 16], [48, 32]], 0.5);

  const h = new Float32Array(size * size);
  const rBase = new Float32Array(size * size);
  const alb = newCanvas(size, size);
  const rgh = newCanvas(size, size);
  const ga = alb.getContext('2d');
  const gr = rgh.getContext('2d');
  const ia = ga.createImageData(size, size);
  const ir = gr.createImageData(size, size);
  const da = ia.data, dr = ir.data;

  const TAU = Math.PI * 2;
  // The map is authored at 1024 over a 2.6 m tile (2.5 mm/texel), i.e. twice
  // the r3 sampling density, so the Sobel of the same relief is half as steep
  // and the normal strength has to rise to match.
  const NRM_STRENGTH = 5.0;

  for (let y = 0; y < size; y++) {
    const py = y / size;
    for (let x = 0; x < size; x++) {
      const px = x / size;
      const drag = fbmDrag(px, py);
      const fine = fbmFine(px, py);
      const micro = fbmMicro(px, py);
      const macro = fbmMacro(px, py);
      const peb = pebble(px, py);

      // Drag lines: the ridges of fines a tyre pushes out either side of its
      // contact patch, plus the scars a dragged stone leaves. Strictly a
      // function of the ACROSS coordinate, so 100% of its gradient is
      // across-track. Two scales — 0.29 m and 0.11 m at the 2.6 m tile.
      const lines = 0.5 + 0.5 * (
        Math.sin(px * TAU * 9 + drag * 5.0) * 0.60 +
        Math.sin(px * TAU * 23 + fine * 4.0) * 0.40);

      // Transverse chatter — braking bumps and root ripple, ~1.03 m wavelength.
      // Kept (it is real) but reduced: it is the one term that works AGAINST the
      // anisotropy the tread needs, and Lane B3 is restoring the geometric
      // version of it at 0.06-0.12 m where it belongs.
      const chatter = Math.sin((py * 3) * TAU + drag * 3.4) * 0.5 + 0.5;
      const chatterAmp = 0.18 * smoothstep(0.36, 0.76, macro * 0.7 + drag * 0.3);

      // Stone mask — embedded pebbles poking through the packed dirt.
      const stone = smoothstep(0.60, 0.76, peb * 0.7 + fine * 0.3);
      // Loose grit clusters, driven off the two finest fields so the grain sits
      // where the eye looks for it (near-field, 3-10 screen px).
      const grit = smoothstep(0.52, 0.86, fine * 0.6 + micro * 0.4);

      let height = drag * 0.42 + fine * 0.30 + micro * 0.16 + macro * 0.12;
      height += stone * 0.26 - grit * 0.05 + (lines - 0.5) * 0.34;
      height += (chatter - 0.5) * chatterAmp;
      h[y * size + x] = height;

      // Near-achromatic DETAIL albedo. It is deliberately bright (mean ~0.88 sRGB
      // ≈ 0.75 linear) because the per-station vertex tint carries the actual
      // surface colour; tint × map then lands at ~0.17 linear for dirt, which is
      // the real-world albedo of packed trail dirt. Author this map at a "natural"
      // mid-brown instead and everything ends up three times too dark once the
      // tint multiplies in.
      //
      // The energy has moved DOWN in scale relative to r3: measured fine/coarse
      // std ratio was 1.4-1.8 where real dirt reads 3-5, so `fine` and `micro`
      // now carry 0.58 of the amplitude between them and `macro` only 0.12.
      let v = 0.88
        + (drag - 0.5) * 0.20
        + (fine - 0.5) * 0.42
        + (micro - 0.5) * 0.30
        + (macro - 0.5) * 0.12;
      v += (lines - 0.5) * 0.24 + (chatter - 0.5) * chatterAmp * 0.20;
      v = clamp(v, 0.30, 1.22);
      // Stones read cooler and lighter, grit warmer and lighter.
      const rC = clamp(v * (1 + grit * 0.10) + stone * 0.085, 0, 1.10);
      const gC = clamp(v * (1 + grit * 0.05) + stone * 0.100, 0, 1.10);
      const bC = clamp(v * (1 - grit * 0.06) + stone * 0.125, 0, 1.10);

      const o = (y * size + x) * 4;
      // A DUST bias in the detail map itself: everything that has been ridden
      // on carries a film of its own fines, and fines are warm. 250/244/234
      // puts a B/R factor of 0.936 sRGB (0.837 linear) on top of the surface
      // tint. Deliberately modest — r3_01's sunlit tread already measured 0.63
      // and was the one reading the review called correct, so this must not
      // drag the DIRT phases past it while it rescues the forest phases.
      // Channel means after this: (0.754, 0.709, 0.631) linear — the pivot the
      // wear-contrast term in buildTreadMesh's shader patch uses.
      da[o] = Math.min(255, rC * 250);
      da[o + 1] = Math.min(255, gC * 244);
      da[o + 2] = Math.min(255, bC * 234);
      da[o + 3] = 255;

      // Roughness: packed dirt is very rough; stones are polished by tyres.
      rBase[y * size + x] =
        0.94 - stone * 0.18 - (macro - 0.5) * 0.08 + (fine - 0.5) * 0.05;
    }
  }

  // --- second pass: Toksvig-equivalent roughness from normal variance -------
  // The tread had NO specular filtering of any kind and a 0.42 roughness floor.
  // Against a bright sky PMREM that is enough indirect specular to dominate a
  // 4%-albedo forest tread: measured, the tread in r3_15 came out BLUER than a
  // white tape ribbon under the same light (B/R 1.62 vs 1.37), which a diffuse
  // surface with a warm albedo simply cannot do. Sub-texel normal variance has
  // to raise roughness, exactly as terrainMaterial.js already does.
  const at = (a, x, y) => a[(((y % size) + size) % size) * size + (((x % size) + size) % size)];
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const gx = at(h, x + 1, y) - at(h, x - 1, y);
      const gy = at(h, x, y + 1) - at(h, x, y - 1);
      const slope = Math.hypot(gx, gy) * NRM_STRENGTH;
      // Soft-saturating so a busy texel cannot pin the whole map at 1.0 and
      // erase the polished-stone sheen the map is also carrying.
      const tok = 0.28 * slope / (slope + 1.2);
      const rough = clamp(rBase[y * size + x] + tok, 0.66, 1.0);
      const ro = (y * size + x) * 4;
      dr[ro] = dr[ro + 1] = dr[ro + 2] = rough * 255;
      dr[ro + 3] = 255;
    }
  }

  ga.putImageData(ia, 0, 0);
  gr.putImageData(ir, 0, 0);
  const nrm = heightToNormalCanvas(h, size, NRM_STRENGTH);
  return { albedo: alb, rough: rgh, normal: nrm };
}

/**
 * Woven polypropylene — ballast bags at the gate feet and the inflatable arch.
 * These were untextured white boxes; at the review exposure an albedo-0.42 flat
 * box with no map is a blown-out faceted blob (r2_08).
 */
function buildFabricMaps(rng, size) {
  const soil = makeTileFbm(rng, [4, 8, 16, 32], 0.55);
  const fuzz = makeTileFbm(rng, [16, 32, 64], 0.5);
  const c = newCanvas(size, size);
  const g = c.getContext('2d');
  const img = g.createImageData(size, size);
  const d = img.data;
  const h = new Float32Array(size * size);
  const WEAVE = 40;                    // threads per tile — integer so it wraps
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const px = x / size, py = y / size;
      // Plain weave: warp and weft alternate which one is proud.
      const warp = Math.sin(px * WEAVE * Math.PI * 2) * 0.5 + 0.5;
      const weft = Math.sin(py * WEAVE * Math.PI * 2) * 0.5 + 0.5;
      const over = ((Math.floor(px * WEAVE) + Math.floor(py * WEAVE)) & 1) ? warp : weft;
      const dirt = soil(px, py) * 0.5 + fuzz(px * 2, py * 2) * 0.5;
      h[y * size + x] = over * 0.7 + dirt * 0.3;
      // Dusty sand-bag hessian, not white vinyl.
      const v = 0.42 + over * 0.16 + (dirt - 0.5) * 0.26;
      const o = (y * size + x) * 4;
      d[o] = clamp(v * 1.06, 0, 1) * 255;
      d[o + 1] = clamp(v * 0.99, 0, 1) * 255;
      d[o + 2] = clamp(v * 0.86, 0, 1) * 255;
      d[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  return { albedo: c, normal: heightToNormalCanvas(h, size, 1.5) };
}

/** Rough sawn timber for stakes, gate posts and signage backs. */
function buildWoodMaps(rng, size) {
  const grain = makeTileFbm(rng, [4, 8, 16, 64], 0.55);
  const knots = makeTileFbm(rng, [4, 8], 0.5);
  const c = newCanvas(size, size);
  const g = c.getContext('2d');
  const img = g.createImageData(size, size);
  const d = img.data;
  const h = new Float32Array(size * size);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const px = x / size, py = y / size;
      // Rings run along the post's length (v).
      const rings = Math.sin((py * 26 + grain(px, py) * 5.5) * Math.PI) * 0.5 + 0.5;
      const k = smoothstep(0.66, 0.84, knots(px, py));
      const v = 0.44 + rings * 0.24 + grain(px * 2, py * 2) * 0.2 - k * 0.28;
      h[y * size + x] = rings * 0.6 + grain(px * 3, py * 3) * 0.4;
      const o = (y * size + x) * 4;
      d[o] = clamp(v * 1.30, 0, 1) * 255;
      d[o + 1] = clamp(v * 1.02, 0, 1) * 255;
      d[o + 2] = clamp(v * 0.74, 0, 1) * 255;
      d[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  return { albedo: c, normal: heightToNormalCanvas(h, size, 1.6) };
}

/** Course tape: printed chevrons on thin polythene, with alpha for the perforations. */
function buildTapeTexture(colorA, colorB) {
  const w = 256, h = 64;
  const c = newCanvas(w, h);
  const g = c.getContext('2d');
  g.clearRect(0, 0, w, h);
  g.fillStyle = colorA;
  g.fillRect(0, 0, w, h);
  g.fillStyle = colorB;
  const skew = 26;
  for (let i = -2; i < 10; i++) {
    const x = i * 52;
    g.beginPath();
    g.moveTo(x, 0);
    g.lineTo(x + 26, 0);
    g.lineTo(x + 26 - skew, h);
    g.lineTo(x - skew, h);
    g.closePath();
    g.fill();
  }
  // Printed edge lines + a little wear.
  g.globalAlpha = 0.35;
  g.fillStyle = '#000000';
  g.fillRect(0, 0, w, 3);
  g.fillRect(0, h - 3, w, 3);
  g.globalAlpha = 1;
  return c;
}

/**
 * Banner / signage atlas. 4x4 cells at `cell` px. Cells:
 *  0..7  checkpoint banners 1..8      8 start banner     9 finish banner
 *  10 A-LINE  11 B-LINE  12 arrow-left  13 arrow-right  14 caution  15 sponsor
 */
function buildBannerAtlas(cell) {
  const size = cell * 4;
  const c = newCanvas(size, size);
  const g = c.getContext('2d');
  g.clearRect(0, 0, size, size);

  const cx = (i) => (i % 4) * cell;
  const cy = (i) => Math.floor(i / 4) * cell;

  function panel(i, bg, accent) {
    const x = cx(i), y = cy(i);
    g.save();
    g.beginPath(); g.rect(x, y, cell, cell); g.clip();
    g.fillStyle = bg; g.fillRect(x, y, cell, cell);
    // subtle vinyl weave
    g.globalAlpha = 0.06; g.fillStyle = '#ffffff';
    for (let k = 0; k < cell; k += 4) g.fillRect(x, y + k, cell, 1);
    g.globalAlpha = 1;
    if (accent) {
      g.fillStyle = accent;
      g.fillRect(x, y, cell, Math.round(cell * 0.10));
      g.fillRect(x, y + cell - Math.round(cell * 0.10), cell, Math.round(cell * 0.10));
    }
    g.restore();
  }
  function text(i, str, px, color, dy = 0.5, weight = '900') {
    const x = cx(i), y = cy(i);
    g.save();
    g.beginPath(); g.rect(x, y, cell, cell); g.clip();
    g.fillStyle = color;
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    g.font = `${weight} ${Math.round(cell * px)}px "Helvetica Neue", Helvetica, Arial, sans-serif`;
    g.fillText(str, x + cell / 2, y + cell * dy);
    g.restore();
  }

  // Checkpoint banners — dark charcoal with a hot accent, the number huge.
  // Charcoal, not black: #14181d is 2% linear, which at distance reads as a
  // pure-black slab cut out of the hillside (r2_00). Real printed PVC bottoms
  // out around 4-5% reflectance.
  for (let i = 0; i < 8; i++) {
    panel(i, '#23282e', i % 2 ? '#e2432f' : '#f2b134');
    text(i, `CP ${i + 1}`, 0.40, '#f4f5f2', 0.52);
    g.save();
    g.globalAlpha = 0.5;
    text(i, 'DESCENT', 0.11, '#8f979f', 0.84, '700');
    g.restore();
  }
  // Start banner
  panel(8, '#1c222a', '#4fb477');
  text(8, 'START', 0.34, '#f4f5f2', 0.44);
  text(8, 'DESCENT  DH  ·  2.6 km  ·  420 m', 0.085, '#9aa3ab', 0.74, '600');
  // Finish banner
  panel(9, '#1c222a', '#e2432f');
  text(9, 'FINISH', 0.32, '#f4f5f2', 0.46);
  text(9, '· · · · · · · · · ·', 0.13, '#e2432f', 0.76, '700');
  // A / B line plates
  panel(10, '#e2432f', null);
  text(10, 'A', 0.55, '#ffffff', 0.42);
  text(10, 'PRO LINE', 0.13, '#ffe9e5', 0.78, '800');
  panel(11, '#2f6fe2', null);
  text(11, 'B', 0.55, '#ffffff', 0.42);
  text(11, 'RIDE AROUND', 0.105, '#e2ecff', 0.78, '800');
  // Direction arrows
  for (const [i, dir] of [[12, -1], [13, 1]]) {
    panel(i, '#f2b134', null);
    const x = cx(i), y = cy(i);
    g.save();
    g.translate(x + cell / 2, y + cell / 2);
    g.scale(dir, 1);
    g.fillStyle = '#181b1f';
    g.beginPath();
    g.moveTo(cell * 0.30, 0);
    g.lineTo(-cell * 0.02, -cell * 0.26);
    g.lineTo(-cell * 0.02, -cell * 0.10);
    g.lineTo(-cell * 0.32, -cell * 0.10);
    g.lineTo(-cell * 0.32, cell * 0.10);
    g.lineTo(-cell * 0.02, cell * 0.10);
    g.lineTo(-cell * 0.02, cell * 0.26);
    g.closePath();
    g.fill();
    g.restore();
  }
  // Caution plate (chevrons)
  panel(14, '#f2b134', null);
  g.save();
  g.beginPath(); g.rect(cx(14), cy(14), cell, cell); g.clip();
  g.fillStyle = '#181b1f';
  for (let k = -4; k < 8; k++) {
    g.beginPath();
    const x = cx(14) + k * cell * 0.26;
    g.moveTo(x, cy(14));
    g.lineTo(x + cell * 0.13, cy(14));
    g.lineTo(x + cell * 0.13 - cell * 0.3, cy(14) + cell);
    g.lineTo(x - cell * 0.3, cy(14) + cell);
    g.closePath(); g.fill();
  }
  g.restore();
  // Sponsor / neutral banner
  panel(15, '#1b2027', '#f4f5f2');
  text(15, 'DESCENT', 0.22, '#f4f5f2', 0.5);

  return c;
}

/** UV rect for atlas cell i in a 4x4 atlas. */
function atlasRect(i) {
  const u = (i % 4) / 4, v = 1 - (Math.floor(i / 4) + 1) / 4;
  return [u + 0.002, v + 0.002, 0.25 - 0.004, 0.25 - 0.004];
}

// ===========================================================================
// Geometry accumulation — merge primitives into a handful of draw calls.
// ===========================================================================

function newBuilder(opts = {}) {
  return {
    pos: [], nor: [], uv: [], idx: [],
    col: opts.color ? [] : null,
    flu: opts.flutter ? [] : null,
    // 0 = lower edge of a ribbon, 1 = upper edge. Only the course tape needs
    // it: it is what lets the vertex shader hold a minimum screen width.
    edg: opts.edge ? [] : null,
    vcount: 0,
  };
}

/**
 * Append `geo` transformed by `m`, optionally tinted (RGBA) and given a per-vertex
 * flutter weight (a function of the local position, used by the cloth shader).
 */
function appendGeometry(b, geo, m, rgba, flutterFn, uvRect) {
  const pos = geo.attributes.position;
  const nor = geo.attributes.normal;
  const uv = geo.attributes.uv;
  const index = geo.index;
  const base = b.vcount;
  _m3.getNormalMatrix(m);
  for (let i = 0; i < pos.count; i++) {
    _p.fromBufferAttribute(pos, i);
    const lx = _p.x, ly = _p.y, lz = _p.z;
    _p.applyMatrix4(m);
    b.pos.push(_p.x, _p.y, _p.z);
    if (nor) {
      _q.fromBufferAttribute(nor, i).applyMatrix3(_m3).normalize();
      b.nor.push(_q.x, _q.y, _q.z);
    } else b.nor.push(0, 1, 0);
    let u = uv ? uv.getX(i) : 0;
    let v = uv ? uv.getY(i) : 0;
    if (uvRect) { u = uvRect[0] + u * uvRect[2]; v = uvRect[1] + v * uvRect[3]; }
    b.uv.push(u, v);
    if (b.col) {
      if (rgba) b.col.push(rgba[0], rgba[1], rgba[2], rgba[3]);
      else b.col.push(1, 1, 1, 1);
    }
    if (b.flu) b.flu.push(flutterFn ? flutterFn(lx, ly, lz) : 0);
    if (b.edg) b.edg.push(0.5);            // primitives never widen
  }
  if (index) {
    for (let i = 0; i < index.count; i++) b.idx.push(base + index.getX(i));
  } else {
    for (let i = 0; i < pos.count; i++) b.idx.push(base + i);
  }
  b.vcount += pos.count;
}

function builderToGeometry(b) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(b.pos, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(b.nor, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(b.uv, 2));
  if (b.col) g.setAttribute('color', new THREE.Float32BufferAttribute(b.col, 4));
  if (b.flu) g.setAttribute('aFlutter', new THREE.Float32BufferAttribute(b.flu, 1));
  if (b.edg) g.setAttribute('aEdge', new THREE.Float32BufferAttribute(b.edg, 1));
  g.setIndex(b.idx);
  g.computeBoundingSphere();
  return g;
}

// ===========================================================================
// Ballistics — a jump you cannot land is a bug.
// ===========================================================================

/**
 * Solve a lip + landing pair. A jump you cannot land is a bug, so the landing is
 * not guessed — it is *constructed from the trajectory*.
 *
 * Coordinates: x is horizontal distance from the takeoff lip along the trail; all
 * heights are measured relative to the landing-side base grade, which falls at
 * `alpha` radians (and sits `stepDown` below the takeoff-side grade).
 *
 *   psi  = theta - alpha            launch angle above horizontal
 *   rel(x) = H + tan(alpha)x + (vy/vx)x - (g/2vx²)x²      H = lipH + stepDown
 *
 * rel(0) = H (the lip top) and rel(D) = 0 (where the arc meets the base grade).
 *
 * The landing transition is the TANGENT to the trajectory at the chosen touchdown
 * point xT, so a rider at the design speed touches down with (almost) zero
 * surface-normal velocity. The knuckle is wherever that tangent, extended back up
 * the hill, reaches the knuckle height — which makes the gap a *consequence* of
 * the physics rather than a number someone made up. The ramp is then extended past
 * xT (so the touchdown sits ~60% down it) and curved back out to the trail.
 */
function solveJump(v, theta, alpha, lipH, stepDown, opt = {}) {
  const psi = theta - alpha;
  const vx = Math.max(0.5, v * Math.cos(psi));
  const vy = v * Math.sin(psi);
  const ta = Math.tan(alpha);
  const H = lipH + stepDown;
  const a2 = G / (2 * vx * vx);
  const b1 = ta + vy / vx;

  const rel = (x) => H + b1 * x - a2 * x * x;

  // Flat-landing distance: rel(D) = 0.
  const D = (b1 + Math.sqrt(b1 * b1 + 4 * a2 * H)) / (2 * a2);

  // Touch down a little past D so the landing transition is dug in below the base
  // grade — which is exactly how a real landing is built.
  const touchOver = opt.touchOver === undefined ? 1.10 : opt.touchOver;
  const xT = D * touchOver;
  const yT = rel(xT);                 // negative: below base grade

  // Knuckle height, and where the arc passes `clearance` above it on the way down.
  // Solving for that point (rather than picking a gap length) is what guarantees
  // the rider actually clears the knuckle.
  const yK = clamp(lipH * (opt.knuckle === undefined ? 0.92 : opt.knuckle), 0.35, 1.6);
  const clearance = opt.clearance === undefined ? 0.55 : opt.clearance;
  const disc = b1 * b1 - 4 * a2 * (yK + clearance - H);
  let xK = disc > 0 ? (b1 + Math.sqrt(disc)) / (2 * a2) : D * 0.55;
  xK = clamp(xK, 1.5, xT - 1.5);

  // The landing ramp is the CHORD from the knuckle to the touchdown point — not
  // the tangent. The trajectory is concave, so a tangent line lies *above* the
  // arc everywhere except its one contact point, which would put the ramp into
  // the rider's face at the knuckle. A chord between two points that are on or
  // below the arc stays below it the whole way, so the flight is clear and
  // touchdown happens exactly where it was designed to.
  const m = (yT - yK) / (xT - xK);
  const rampRun = (xT - xK) * 1.65;   // design touchdown at ~61% of the ramp
  const rampEnd = xK + rampRun;
  const yEnd = yK + m * rampRun;      // depth of the landing pit, below grade

  // Climb back out of the pit at no more than 20% relative to grade.
  const outLen = clamp(Math.abs(yEnd - stepDown) / 0.20, 4, 26);

  // Ground height (relative to landing-side grade) at x. Before the knuckle that
  // is the deck of a tabletop or the floor of a dug gap — getting this right is
  // what makes the "can a slower rider case it?" test meaningful.
  const deckY = opt.deckY === undefined ? 0 : opt.deckY;
  const surf = (x) => {
    if (x <= xK) return deckY;
    if (x <= rampEnd) return yK + m * (x - xK);
    const f = clamp01((x - rampEnd) / outLen);
    return yEnd * (1 - f * f * (3 - 2 * f));
  };

  // Absolute landing angle below horizontal (the ramp falls at m, plus the grade).
  const delta = Math.atan(-(m - ta));
  const tTouch = xT / vx;
  const vTouch = Math.hypot(vx, vy - G * tTouch);
  const gammaTouch = Math.atan2(G * tTouch - vy, vx);

  if (opt.quick) {
    return { psi, D, xK, yK, xT, yT, m, rampRun, rampEnd, yEnd, outLen, delta, surf };
  }

  // Numerically find the speed window that still lands on the transition.
  function touchdownFor(vv) {
    const px = Math.max(0.5, vv * Math.cos(psi));
    const py = vv * Math.sin(psi);
    const aa = G / (2 * px * px);
    const bb = ta + py / px;
    const far = rampEnd + outLen;
    for (let x = 0.25; x <= far; x += 0.25) {
      const r = H + bb * x - aa * x * x;
      const s = surf(x);
      if (r <= s) return x - 0.125;
    }
    return far + 1;
  }
  let vMin = v, vMax = v;
  for (let k = 1; k <= 45; k++) {
    const vv = v * (1 - k * 0.02);
    if (vv < 2) break;
    const x = touchdownFor(vv);
    // Coming up short on a tabletop means landing on the deck, which is the whole
    // point of a tabletop; on a double or a gap it means casing the knuckle.
    if (x < xK + 0.6) { if (opt.deckY > 0.2) vMin = vv * 0.55; break; }
    vMin = vv;
  }
  for (let k = 1; k <= 45; k++) {
    const vv = v * (1 + k * 0.02);
    const x = touchdownFor(vv);
    if (x > rampEnd + outLen * 0.55) break;   // would land on the flat run-out
    vMax = vv;
  }

  return {
    psi, D, xK, yK, xT, yT, m, rampRun, rampEnd, yEnd, outLen, delta, surf,
    touchFrac: (xT - xK) / rampRun,
    airTime: tTouch, vTouch,
    gammaTouch,
    // Speed into the landing surface: the number that decides whether it hurts.
    // A chord landing gives a small, deliberate compression rather than zero.
    vImpact: Math.abs(vTouch * Math.sin(gammaTouch - delta)),
    peakH: H + (vy * vy) / (2 * G),
    vMin, vMax,
  };
}

/**
 * Choose the takeoff angle that produces a given landing distance at the design
 * speed. Building jumps this way round means a jump line has a *rhythm* the
 * builder chose (9 m, 11 m, 13 m…) instead of whatever the local gradient
 * happens to throw out — on a 30% pitch a 23° lip at 20 m/s launches you 40 m.
 * D is monotone in theta over the usable range, so bisection is exact enough.
 */
function lipAngleForDistance(v, alpha, lipH, stepDown, targetD, loDeg, hiDeg) {
  let lo = THREE.MathUtils.degToRad(loDeg);
  let hi = THREE.MathUtils.degToRad(hiDeg);
  if (solveJump(v, lo, alpha, lipH, stepDown, { quick: true }).D >= targetD) return lo;
  if (solveJump(v, hi, alpha, lipH, stepDown, { quick: true }).D <= targetD) return hi;
  for (let it = 0; it < 22; it++) {
    const mid = (lo + hi) * 0.5;
    if (solveJump(v, mid, alpha, lipH, stepDown, { quick: true }).D < targetD) lo = mid;
    else hi = mid;
  }
  return (lo + hi) * 0.5;
}

// ===========================================================================
// createTrail
// ===========================================================================

export function createTrail(ctx) {
  const routeRng = makeRng(subSeed(ctx.seed, 'trail:route'));
  const featRng = makeRng(subSeed(ctx.seed, 'trail:features'));
  const dressRng = makeRng(subSeed(ctx.seed, 'trail:dressing'));
  const texRng = makeRng(subSeed(ctx.seed, 'trail:textures'));

  const wanderN = makeNoise1D(routeRng, 512);
  const microN = makeNoise1D(featRng, 1024);
  const edgeN = makeNoise1D(dressRng, 512);

  // ---- station table (all Float32Array, built once) -----------------------
  let N = 0;
  let S = null;                 // station arrays
  let curve = null;
  let length = 1;
  let stamps = [];
  const features = [];
  const checkpoints = [];
  const splits = [];
  const phaseSpans = [];
  const jumpLog = [];
  const cornerAudit = [];   // see auditCornerIdentity()
  const routeAudit = [];    // one row per route variant; see build() and scoreRoute()
  let crestMask = null;     // authored-relief mask; see countCrestViol()
  const group = new THREE.Group();
  group.name = 'trail';

  const startTransform = {
    position: new THREE.Vector3(),
    quaternion: new THREE.Quaternion(),
  };

  // Spatial index over stations for the global nearestT fallback.
  let gridCell = 24, gridW = 1, gridH = 1, gridMinX = 0, gridMinZ = 0;
  let cellStart = null, cellItems = null;

  // Reusable return objects — nearestT/sampleAt are called every frame.
  const _sample = {
    position: new THREE.Vector3(),
    tangent: new THREE.Vector3(0, 0, -1),
    normal: new THREE.Vector3(0, 1, 0),
    binormal: new THREE.Vector3(1, 0, 0),
    width: 2.2, bank: 0, gradient: 0, feature: null,
  };
  const _near = {
    t: 0, lateral: 0, distance: 0,
    position: new THREE.Vector3(),
    tangent: new THREE.Vector3(0, 0, -1),
  };

  // Flutter shader uniforms, shared by tape and banners. Updated in update().
  const flutterUniforms = {
    uTime: { value: 0 },
    uWindDir: { value: new THREE.Vector3(0.82, 0, 0.57) },
    uWindAmp: { value: 0.11 },
    // Drawing-buffer height in pixels, for the tape's minimum-screen-width
    // term. Seeded from the renderer and refreshed by resize().
    uViewportH: { value: 1080 },
  };
  const _resizeV2 = new THREE.Vector2();
  if (ctx.renderer && ctx.renderer.getDrawingBufferSize) {
    ctx.renderer.getDrawingBufferSize(_resizeV2);
    if (_resizeV2.y > 1) flutterUniforms.uViewportH.value = _resizeV2.y;
  }

  // =========================================================================
  // 1. ROUTE — march down the fall line with trail-design rules on top
  // =========================================================================

  function terrainH(terrain, x, z) {
    return terrain.sampleHeight(
      clamp(x, terrain.bounds.minX + 2, terrain.bounds.maxX - 2),
      clamp(z, terrain.bounds.minZ + 2, terrain.bounds.maxZ - 2));
  }

  /** Central-difference terrain gradient (dh/dx, dh/dz) over `e` metres. */
  function gradAt(terrain, x, z, e, out) {
    const hL = terrainH(terrain, x - e, z), hR = terrainH(terrain, x + e, z);
    const hD = terrainH(terrain, x, z - e), hU = terrainH(terrain, x, z + e);
    out.gx = (hR - hL) / (2 * e);
    out.gz = (hU - hD) / (2 * e);
    out.mag = Math.hypot(out.gx, out.gz);
    return out;
  }

  // Scratch for the march's inner loop.
  const gTmp = { gx: 0, gz: 0, mag: 0 };

  /**
   * Corridor protectability at one prospective station, in metres.
   *
   * (cx, cz) is the centre of a tread of half-width `half` whose direction of
   * travel is the unit vector (sx, sz); the corridor is probed CORR_Q metres
   * beyond each tread edge. See the long note at CORR_S for the derivation.
   *
   * Returns the WORSE of the two sides, because a rib is only gone on one side
   * and that is quite enough. `out` is a caller-owned scratch object so this is
   * allocation-free — it is called about half a million times per build.
   */
  function corridorAt(terrain, cx, cz, sx, sz, half, out) {
    const y0 = terrainH(terrain, cx, cz);
    // right = (dir x up) in XZ
    const rx = -sz, rz = sx;
    let worstDef = 0, worstFall = 0;
    for (let si = 0; si < 2; si++) {
      const sgn = si === 0 ? -1 : 1;
      let bestSlack = -1e9, fall = 0;
      for (let k = 0; k < CORR_Q.length; k++) {
        const q = CORR_Q[k];
        const d = half + q;
        const nat = terrainH(terrain, cx + rx * sgn * d, cz + rz * sgn * d);
        // Metres of drop in excess of what a soil hillside could hold up. Zero on
        // any ordinary sidehill however steep; the tread half-width is inside the
        // repose allowance because the tread itself is a bench cut into that slope.
        if (q <= CORR_NEAR + 1e-6) {
          const e = (y0 - nat) - EXPO_REPOSE * d;
          if (e > fall) fall = e;
        }
        // designCatch()'s feasibility test, run forward on natural ground.
        const slack = CORR_BUDGET - ((y0 - SHELF_SLOPE * q + CORR_CATCH_H) - nat);
        if (slack > bestSlack) bestSlack = slack;
      }
      const def = bestSlack < 0 ? -bestSlack : 0;
      if (def > worstDef) worstDef = def;
      if (fall > worstFall) worstFall = fall;
    }
    out.deficit = worstDef;
    out.fall = worstFall;
    out.cost = worstDef * CORR_W_DEFICIT + worstFall * CORR_W_FALL;
    return out;
  }

  const cTmp = { deficit: 0, fall: 0, cost: 0 };

  /**
   * Mean corridor cost over `stations` metres ahead of (x, z) on heading `hd`.
   * Straight-line probe: the march re-scores every 6 m, so a curved probe would
   * only be re-deciding ground it is about to look at again anyway.
   */
  function corridorCost(terrain, x, z, hd, half, stations, cap) {
    const sx = Math.sin(hd), sz = -Math.cos(hd);
    let sum = 0;
    for (let k = 0; k < stations.length; k++) {
      const s = stations[k];
      corridorAt(terrain, x + sx * s, z + sz * s, sx, sz, half, cTmp);
      sum += cTmp.cost;
    }
    const c = sum / stations.length;
    return c > cap ? cap : c;
  }

  /**
   * The far gate. Deficit only, above a tolerance: this is the term that refuses
   * to walk into a corridor whose exits are all bad, without steering the route
   * around ordinary distant sidehill. See the note at CORR_S_FAR for the
   * measurement that made it a gate rather than a cost.
   */
  function corridorGate(terrain, x, z, hd, half) {
    const sx = Math.sin(hd), sz = -Math.cos(hd);
    let sum = 0;
    for (let k = 0; k < CORR_S_FAR.length; k++) {
      const s = CORR_S_FAR[k];
      corridorAt(terrain, x + sx * s, z + sz * s, sx, sz, half, cTmp);
      const over = cTmp.deficit - CORR_FAR_TOL;
      if (over > 0) sum += over * CORR_W_FAR;
    }
    const c = sum / CORR_S_FAR.length;
    return c > CORR_CAP_FAR ? CORR_CAP_FAR : c;
  }

  /**
   * Longitudinal buildability of a candidate heading: the worst convex vertical
   * curvature of the natural ground ahead, scored against the crest cap at the
   * phase's design speed. See the note at CORR_KS.
   *
   * `vDesign` is the phase's design speed, not a solved one — at route-finding
   * time there is no speed profile yet, and the design speed is the honest
   * statement of what the course will ask for here.
   */
  function corridorProfile(terrain, x, z, hd, vDesign) {
    const sx = Math.sin(hd), sz = -Math.cos(hd);
    const v = Math.max(4, vDesign);
    const cap = clamp(G / (CREST_SAFETY * v * v), CREST_KAPPA_MIN, CREST_KAPPA_MAX);
    let yPrev = terrainH(terrain, x, z);
    let prevG = 0, worst = 0;
    for (let k = 1; k < CORR_KS.length; k++) {
      const s = CORR_KS[k];
      const y = terrainH(terrain, x + sx * s, z + sz * s);
      const seg = s - CORR_KS[k - 1];
      const g = (yPrev - y) / seg;           // + = descending
      if (k > 1) {
        // d(grade)/ds over the two segments straddling station k-1. Positive is
        // convex — getting steeper down the hill — which is the launching sign.
        const span = 0.5 * (s - CORR_KS[k - 2]);
        const kap = (g - prevG) / span;
        if (kap > worst) worst = kap;
      }
      prevG = g; yPrev = y;
    }
    const over = worst - cap;
    if (over <= 0) return 0;
    const c = over * CORR_W_KAPPA;
    return c > CORR_CAP_KAPPA ? CORR_CAP_KAPPA : c;
  }

  // ---------------------------------------------------------------------------
  // LEG CLEARANCE — the route must not lay a second leg on ground an earlier leg
  // has already claimed.
  //
  // WHAT GOES WRONG WITHOUT IT, measured on the shipped world at seed 777: the
  // trail passes arc 342-345 and comes back at arc 423-426 with 1.38 m of PLAN
  // separation and 5.82 m of height between the two limbs — a 4.2:1 face where a
  // hillside stands at about 0.7:1. 16 such stretches, 85.6 m in total. Terrain's
  // accumulator then has to serve both from one heightfield: its plane fit admits
  // a neighbour while |dh| <= 0.8*d + 0.3, so at that spacing the two limbs are
  // fitted TOGETHER, and the committed cross-slope under the tread came out at
  // -71.4 deg where the trail declared +1.2. That is not a carve defect and no
  // accumulator can do better — the trail asked for two surfaces in one place.
  //
  // THE CLEARANCE A PAIR OF LIMBS NEEDS IS A PROPERTY OF THE STAMP CLOUD, not of
  // the drop between them, and getting that wrong is worse than not having the
  // rule. A first cut required dh / EXPO_REPOSE of separation — the slope a
  // hillside stands at — which sounds right and is not: two limbs of an ordinary
  // switchback stack sit 10-14 m apart with 6-8 m between them, that rule wanted
  // 12 m, and the term stopped being zero on courses that had nothing wrong with
  // them (seed 20260726 lost four features and overran its design drop by 28 m).
  //
  // What actually breaks is CROSS-VOTING: terrain's accumulator lets a stamp vote
  // out to RHO_MAX (1.25) normalised radii, widened up to 14% by the carve-width
  // jitter, so a station's stamps influence ground out to
  //     legReach(half) = 1.22*half            (the outermost lateral's offset)
  //                    + 1.4*(0.50*half+0.30) (that lateral's voting radius)
  // from its own centreline. Two limbs closer than the sum of their reaches are
  // voting on each other's tread, whatever the height between them — and where
  // the height IS large, the votes are metres apart and the mean is a wall. Once
  // the reaches clear, the only thing left between the limbs is the cut/fill
  // batter, whose target is clamped against natural ground and whose weight
  // terrain already floored at 5e-4 for exactly this case.
  //
  // THE TERM IS EXACTLY ZERO WHEN THE ROUTE IS CLEAN. That is deliberate and it
  // is checkable: on seeds 20260726 and 12345, which have no fold-back conflicts,
  // the marched polyline is bit-identical with and without this term. A safety
  // rule that silently reshapes a course it has nothing to say about is a rule
  // nobody can measure.
  //
  // The weight and cap are set to the same scale as the corridor-protectability
  // term (CORR_CAP 110) and therefore, like it, sit well below the march's climb
  // penalties (900/1100/1600) and below its grade-holding weight (220): this can
  // decide between comparable headings and it cannot buy its way out by climbing,
  // by abandoning the phase gradient, or by refusing to switchback at all.
  // 45 m of travel: a 6.5 m-radius switchback — the tightest the march builds —
  // covers pi*6.5 = 20 m of arc through the apex and leaves its limbs 13 m apart
  // in plan, comfortably outside any legReach() pair, so the rule has nothing to
  // say about a switchback and everything to say about a route that comes back
  // across its own line a hundred metres later.
  const LEG_ARC_MIN = 45.0;     // m of travel before a point counts as another leg
  const LEG_SEARCH = 9.0;       // m — past 2*legReach(1.5) = 6.6 m no pair conflicts
  const LEG_W = 13.0;           // score per metre of missing clearance
  const LEG_CAP = 115.0;
  const LEG_CELL = 12.0;
  // ...AND A GATE ON TOP OF THE COST, for the same reason the corridor term has
  // one. A capped bias decides between comparable headings; it loses to `aim`
  // (up to 220 near the finish) and to grade holding (220), and on seed 777 it
  // did: the route still closed a 360 deg loop back onto its own line 80 m later
  // and 5.7 m lower, because every heading out of that bowl was penalised and
  // the cheapest one still went round. Laying tread on top of an earlier leg is
  // not a trade — it is a trail that cannot be built — so past LEG_HARD_TOL of
  // OVERLAP the penalty leaves the tie-breaking range and joins the class of the
  // march's structural refusals (climb 900, dead end 1600).
  const LEG_HARD_TOL = 0.75;    // m of deficit tolerated before the gate fires
  const LEG_W_HARD = 900.0;
  // Measured NOT to bind: 300, 600 and 1500 give byte-identical worlds on all
  // three seeds. What decides the route is that the gate exists at all, not how
  // far past it the penalty is allowed to run — so this is a guard rail rather
  // than a tuning knob, and nothing here should be read as tuned.
  const LEG_CAP_HARD = 1500.0;

  /** How far a station's stamps vote from its own centreline. See LEG note. */
  function legReach(half) {
    return 1.22 * half + 1.4 * (0.50 * half + 0.30);
  }

  /**
   * Missing leg clearance at a candidate point, in metres, summed over the
   * earlier legs it would contest. `hist` is the march's own grid of committed
   * points; see marchRoute(). Returns 0 — exactly — when nothing is contested.
   */
  function legClearanceCost(hist, nx, nz, travelledNow, half) {
    const bx = Math.floor(nx / LEG_CELL), bz = Math.floor(nz / LEG_CELL);
    let deficit = 0;
    const R = Math.ceil(LEG_SEARCH / LEG_CELL);
    for (let a = -R; a <= R; a++) {
      for (let b = -R; b <= R; b++) {
        const list = hist.cells.get((bz + b) * 8192 + (bx + a));
        if (!list) continue;
        for (let m = 0; m < list.length; m++) {
          const k = list[m];
          if (travelledNow - hist.s[k] < LEG_ARC_MIN) continue;
          const dx = nx - hist.x[k], dz = nz - hist.z[k];
          const d = Math.sqrt(dx * dx + dz * dz);
          if (d >= LEG_SEARCH) continue;
          const need = legReach(half) + legReach(hist.half[k]);
          if (d >= need) continue;
          deficit += need - d;
        }
      }
    }
    if (deficit <= 0) return 0;
    let c = deficit * LEG_W;
    if (c > LEG_CAP) c = LEG_CAP;
    if (deficit > LEG_HARD_TOL) {
      const g = (deficit - LEG_HARD_TOL) * LEG_W_HARD;
      c += g > LEG_CAP_HARD ? LEG_CAP_HARD : g;
    }
    return c;
  }

  function marchRoute(terrain, variant) {
    // Each variant is an independent, deterministic attempt at the route: a
    // different corridor target and a different wander stream. build() runs
    // several and keeps the best, because on some mountains the first corridor
    // you pick walks into a wall and no amount of local steering saves it.
    const vrng = makeRng(subSeed(ctx.seed, `trail:route:${variant}`));
    const vWander = makeNoise1D(vrng, 512);
    const B = terrain.bounds;
    const margin = 160;
    const zStart = B.maxZ - 210;

    // Pick a launch point: high, with real fall beneath it, and — the addition —
    // standing at the head of a corridor that can actually be built and protected.
    //
    // The per-variant jitter used to be applied AFTER the scan, which meant the
    // point that was scored and the point that was used were up to 35 m apart.
    // That was survivable while the score was only "high and steep"; it is not
    // survivable once the score is measuring the ground either side of the tread,
    // because 35 m along a contour is the difference between a bench and a rib.
    // The jitter now offsets the whole scan grid instead, so each variant still
    // searches its own set of candidates and the winner is the point that was
    // measured.
    const launchJitter = (vrng() - 0.5) * 70;
    const startHalf = PHASES[0].width * 0.5;
    let startX = 0, bestScore = -Infinity;
    for (let k = 0; k <= 48; k++) {
      const x = clamp(lerp(B.minX + margin, B.maxX - margin, k / 48) + launchJitter,
        B.minX + margin, B.maxX - margin);
      const h = terrainH(terrain, x, zStart);
      const hAhead = terrainH(terrain, x, zStart - 220);
      const hSide = Math.abs(terrainH(terrain, x + 60, zStart) - terrainH(terrain, x - 60, zStart));
      // High + steep ahead + not on a knife-edge ridge crest.
      let s = h * 0.55 + (h - hAhead) * 1.6 - hSide * 0.35;
      // ... and the best USABLE corridor leaving it, where usable means two
      // things at once: protectable, and at a gradient the opening phase can
      // actually be built at.
      //
      // Both halves are load-bearing and the second was learnt the hard way. A
      // first cut of this scored protectability alone. It duly found a launch
      // point whose corridor measured 0.00 m of unsupported fall for the whole
      // first hundred metres — and put it on the front of a face running 42-60%,
      // in the phase whose design gradient is 11% and whose design speed is
      // 16 m/s. The route was safe and unrideable: the autopilot went airborne at
      // 75 m doing 17.4 m/s down a 57% pitch. Trading exposure for gradient is
      // not a fix, it is a different defect. So the fan penalises any heading
      // past the march's own grade ceiling for that phase, and the launch point
      // is scored on the best corridor that is BOTH.
      const gTarget = PHASES[0].grade;
      const gMax = Math.min(0.46, gTarget * 1.9 + 0.06);
      let bestCorr = CORR_LAUNCH_CAP;
      for (let j = -CORR_LAUNCH_FAN; j <= CORR_LAUNCH_FAN; j++) {
        // Heading convention, and it is worth stating because getting it wrong
        // costs a silent no-op rather than an error: the march's direction of
        // travel is (sin h, -cos h), so h = 0 is -Z, which CONTRACT §0 says is
        // down the mountain. A fan centred on PI points back up the hill, fails
        // the descent filter at every candidate, and leaves the term a constant.
        const hd = j * (6 * Math.PI / 180);
        const dsx = Math.sin(hd), dsz = -Math.cos(hd);
        // The grade is probed at three ranges and judged on the WORST, because a
        // corridor that leaves at 27% and turns into a 60% face ninety metres in
        // is a 60% face. A single 60 m probe reported 0.27 at the launch point
        // that produced the 42-60% Start Chute — right at the ceiling, so it
        // attracted no penalty at all, and the defect was a hundred metres past
        // where the scan was looking.
        let gWorst = 0, gMean = 0;
        for (let q = 0; q < CORR_LAUNCH_G.length; q++) {
          const s = CORR_LAUNCH_G[q];
          const g = (h - terrainH(terrain, x + dsx * s, zStart + dsz * s)) / s;
          if (g > gWorst) gWorst = g;
          gMean += g;
        }
        gMean /= CORR_LAUNCH_G.length;
        if (gMean < gTarget * 0.55) continue;     // traverses or climbs: not a start
        const gPen = gWorst > gMax ? (gWorst - gMax) * CORR_LAUNCH_GRADE : 0;
        const c = corridorCost(terrain, x, zStart, hd, startHalf, CORR_LAUNCH_S, CORR_LAUNCH_CAP)
          + gPen;
        if (c < bestCorr) bestCorr = c;
      }
      s -= Math.min(CORR_LAUNCH_CAP, bestCorr) * CORR_LAUNCH_W;
      if (s > bestScore) { bestScore = s; startX = x; }
    }
    const yStart = terrainH(terrain, startX, zStart);

    // A race track is a *fixed vertical drop*, not "all the way to the bottom".
    // Find where a straight probe down the corridor has given up TARGET_DROP of
    // altitude and finish there; that keeps the run near the 2.6 km / 420 m spec
    // whatever the mountain happens to be doing.
    const TARGET_DROP = 430;
    const rawFinX = clamp(startX + (vrng() - 0.5) * 520, B.minX + margin, B.maxX - margin);
    let finishX = rawFinX, zFinish = B.minZ + 170;
    for (let k = 1; k <= 140; k++) {
      const f = k / 140;
      const zz = lerp(zStart, B.minZ + 170, f);
      const xx = lerp(startX, rawFinX, f);
      if (terrainH(terrain, xx, zz) <= yStart - TARGET_DROP) { finishX = xx; zFinish = zz; break; }
    }
    const yFinish = terrainH(terrain, finishX, zFinish);
    const drop = clamp(yStart - yFinish, 160, TARGET_DROP + 30);
    const targetLength = clamp(drop / 0.165, 1400, 3000);

    // Corridor: the straight line start -> finish. Lateral drift from it is what
    // triggers a switchback, exactly like a real trail builder constrained by a
    // corridor easement.
    const corLen = Math.hypot(finishX - startX, zFinish - zStart);
    const corDX = (finishX - startX) / corLen, corDZ = (zFinish - zStart) / corLen;
    // corridor "right" = (corDir x up) in XZ = (-dz, dx)
    const corRX = -corDZ, corRZ = corDX;

    const px = [startX], pz = [zStart], pph = [0];
    let x = startX, z = zStart, y = yStart;
    let head = Math.atan2(corDX, -corDZ);     // dir = (sin h, -cos h)
    let travelled = 0;
    // Committed-point history for the leg-clearance term, bucketed on a LEG_CELL
    // grid so a candidate heading costs O(1) to score. See legClearanceCost().
    const hist = { x: [], z: [], s: [], half: [], cells: new Map() };
    const histPush = (hx, hz, hs, hh) => {
      const idx = hist.x.length;
      hist.x.push(hx); hist.z.push(hz); hist.s.push(hs); hist.half.push(hh);
      const key = Math.floor(hz / LEG_CELL) * 8192 + Math.floor(hx / LEG_CELL);
      let l = hist.cells.get(key);
      if (!l) { l = []; hist.cells.set(key, l); }
      l.push(idx);
    };
    histPush(startX, zStart, 0, startHalf);
    let mode = 0;                              // 0 = ride, 1 = switchback
    let sbSign = 1, sbSteps = 0, lastSwitchback = -999;
    const g = { gx: 0, gz: 0, mag: 0 };

    const MAX_STEPS = 3000;
    for (let step = 0; step < MAX_STEPS; step++) {
      // Pacing runs on LENGTH: the rhythm of the run has to map onto the ground
      // the rider covers, whatever altitude the mountain happens to offer.
      const uLen = clamp01(travelled / targetLength);
      const uElev = clamp01((yStart - y) / drop);
      const phIdx = phaseAtLength(uLen);
      const ph = PHASES[phIdx];

      // Altitude governor: steepen if we're behind the design descent curve,
      // flatten if we're burning altitude faster than the script wants.
      const govern = clamp(1 + 1.5 * (dropFracAtLength(uLen) - uElev), 0.55, 2.0);
      const targetGrade = ph.grade * govern;
      const maxGrade = Math.min(0.46, targetGrade * 1.9 + 0.06);
      // Near the end, or if we've somehow overrun, head decisively for the finish.
      const overrun = clamp01((travelled / targetLength - 0.92) * 6);

      gradAt(terrain, x, z, 9, g);
      const fallSteep = g.mag;

      // Signed drift from the corridor.
      const drift = (x - startX) * corRX + (z - zStart) * corRZ;
      const dirX = Math.sin(head), dirZ = -Math.cos(head);
      const driftRate = dirX * corRX + dirZ * corRZ;

      // --- switchback state machine ---------------------------------------
      if (mode === 0) {
        const limit = ph.drift;
        const outbound = Math.sign(drift) === Math.sign(driftRate) && Math.abs(driftRate) > 0.25;
        if (Math.abs(drift) > limit && outbound && fallSteep > 0.26 &&
            travelled - lastSwitchback > 110 && z > zFinish + 220) {
          mode = 1;
          sbSign = -Math.sign(drift) || 1;
          sbSteps = 0;
        }
      }

      let ds, dHead;
      if (mode === 1) {
        // Tight ~6.5 m radius reversal; keep the grade gentle through the apex so
        // the inside of the turn isn't a step.
        ds = 3.0;
        dHead = sbSign * 0.46;                       // 26.4° per 3 m ≈ 6.5 m radius
        sbSteps++;
        // Exit once we're heading firmly back across the hill.
        const nowRate = Math.sin(head) * corRX + (-Math.cos(head)) * corRZ;
        const heading = Math.sin(head) * corDX + (-Math.cos(head)) * corDZ;
        if ((nowRate * sbSign > 0.72 && heading > -0.15) || sbSteps > 26) {
          mode = 0;
          lastSwitchback = travelled;
        }
      } else {
        // --- scored heading search ----------------------------------------
        ds = 6.0;
        const twist = ph.twist;
        const wander = fbm1(vWander, travelled * 0.0085, 3, 2.1, 0.55) * twist;
        // Turn authority: enough to shape the trail, and more when the fall line
        // is steeper than we want — that is how a builder bails onto a traverse
        // instead of dropping straight off the front of a face.
        const steepNeed = clamp01((fallSteep - maxGrade) * 2.5);
        const maxTurn = 0.18 + twist * 0.14 + steepNeed * 0.24;
        const K = 9;
        let bestS = -Infinity, bestD = 0;
        for (let k = -K; k <= K; k++) {
          const dth = (k / K) * maxTurn;
          const h2 = head + dth;
          const nx = x + Math.sin(h2) * ds;
          const nz = z - Math.cos(h2) * ds;
          let sc = 0;
          // Turn away from the map edge before we reach it. Deliberately NOT the
          // +Z edge: the start sits close to it, and penalising that side would
          // bias every candidate at the top of the hill straight down the fall
          // line instead of letting the trail traverse.
          const edge = Math.min(nx - B.minX, B.maxX - nx, nz - B.minZ);
          if (edge < 170) sc -= (170 - edge) * (170 - edge) * 0.035;
          const ny = terrainH(terrain, nx, nz);
          const grade = (y - ny) / ds;               // + = descending
          // Judging a heading on one 6 m step walks the trail into hollows it then
          // has to climb out of, so probe 24 m and 66 m ahead as well. The far
          // probe is a "does this direction go anywhere" gate, not a grade target.
          const s2 = Math.sin(h2), c2 = -Math.cos(h2);
          const gradeMid = (y - terrainH(terrain, x + s2 * 24, z + c2 * 24)) / 24;
          const gradeFar = (y - terrainH(terrain, x + s2 * 66, z + c2 * 66)) / 66;
          const grd = grade * 0.40 + gradeMid * 0.35 + gradeFar * 0.25;
          // Holding the design gradient is the dominant term: this is what makes
          // the trail a *trail* rather than a beeline to the bottom.
          sc -= Math.abs(grd - targetGrade) * 220;
          if (grade < 0.015) sc -= (0.015 - grade) * 900;      // never climb
          if (gradeMid < 0.012) sc -= (0.012 - gradeMid) * 1100;
          if (gradeFar < 0.010) sc -= (0.010 - gradeFar) * 1600; // no dead ends
          if (grd > maxGrade) sc -= (grd - maxGrade) * 320;
          sc -= Math.abs(dth) * 42;                  // prefer smooth arcs
          sc += dth * wander * 95;                   // natural sinuosity
          // Corridor keeping: soft, quadratic beyond the phase's drift allowance.
          // It must not be able to trap the route in a flat corridor — the
          // switchback machinery is what actually bounds the drift.
          const nDrift = (nx - startX) * corRX + (nz - zStart) * corRZ;
          const over = Math.max(0, Math.abs(nDrift) - ph.drift);
          sc -= over * over * 0.012;
          // Head for the finish: weak while there's mountain left, strong once
          // we're close or the run is getting long.
          const toFinX = finishX - nx, toFinZ = zFinish - nz;
          const toFinLen = Math.max(1, Math.hypot(toFinX, toFinZ));
          const aim = (Math.sin(h2) * toFinX + (-Math.cos(h2)) * toFinZ) / toFinLen;
          sc += aim * (10 + 90 * smoothstep(450, 140, toFinLen) + 120 * overrun);
          // Avoid benching an absurd cross-slope (it would need a huge cut).
          gradAt(terrain, nx, nz, 7, gTmp);
          if (gTmp.mag > 0.85) sc -= (gTmp.mag - 0.85) * 120;
          // CORRIDOR PROTECTABILITY. The term this whole file's safety pass was
          // being asked to compensate for after the fact. Probed over the next
          // 24 m at the phase's own design width, capped at CORR_CAP so it can
          // bias a choice between comparable headings and nothing more — see the
          // long note at CORR_S.
          sc -= corridorCost(terrain, nx, nz, h2, ph.width * 0.5, CORR_S, CORR_CAP);
          sc -= corridorGate(terrain, nx, nz, h2, ph.width * 0.5);
          sc -= corridorProfile(terrain, nx, nz, h2, V_PHASE[ph.id]);
          // LEG CLEARANCE. Exactly zero unless this heading would lay tread on
          // ground an earlier leg has already claimed — see legClearanceCost().
          sc -= legClearanceCost(hist, nx, nz, travelled + ds, ph.width * 0.5);
          if (sc > bestS) { bestS = sc; bestD = dth; }
        }
        dHead = bestD;
      }

      head += dHead;
      x += Math.sin(head) * ds;
      z -= Math.cos(head) * ds;
      x = clamp(x, B.minX + 40, B.maxX - 40);
      z = clamp(z, B.minZ + 30, B.maxZ - 10);
      y = terrainH(terrain, x, z);
      travelled += ds;
      px.push(x); pz.push(z); pph.push(phIdx);
      histPush(x, z, travelled, ph.width * 0.5);

      // The run is a designed length. Stop there, or if we run out of mountain.
      if (travelled >= targetLength) break;
      if (z < B.minZ + 120) break;
    }

    // Run-out past the finish: 45 m, straightening onto the corridor direction.
    for (let k = 0; k < 15; k++) {
      const blend = 0.25;
      head = head * (1 - blend) + Math.atan2(corDX, -corDZ) * blend;
      x += Math.sin(head) * 3.0;
      z -= Math.cos(head) * 3.0;
      px.push(x); pz.push(z); pph.push(PHASES.length - 1);
    }
    // The march's own phase assignment is authoritative — it is what the route was
    // actually shaped for. Monotonise it and hand it to the station builder.
    for (let k = 1; k < pph.length; k++) if (pph[k] < pph[k - 1]) pph[k] = pph[k - 1];

    return {
      px, pz, pph, startX, zStart, finishX, zFinish, drop, targetLength, variant,
      startY: yStart,
    };
  }

  /**
   * Grade a candidate route the way a trail builder would walk it: does it climb,
   * is it the length we wanted, and did it use up the altitude it was supposed to?
   * Higher is better.
   */
  function scoreRoute(terrain, route) {
    const n = route.px.length;
    if (n < 40) return -1e9;
    let travelled = 0, climb = 0, low = Infinity;
    let y0 = 0, yLast = 0;
    for (let k = 0; k < n; k++) {
      const y = terrainH(terrain, route.px[k], route.pz[k]);
      if (k === 0) { y0 = y; low = y; }
      if (k > 0) travelled += Math.hypot(route.px[k] - route.px[k - 1], route.pz[k] - route.pz[k - 1]);
      if (y - low > climb) climb = y - low;
      if (y < low) low = y;
      yLast = y;
    }
    const dropGot = y0 - yLast;
    const lenErr = Math.abs(travelled - route.targetLength) / Math.max(1, route.targetLength);
    const dropErr = Math.abs(dropGot - route.drop) / Math.max(1, route.drop);
    // ---- corridor protectability, averaged over the whole candidate ---------
    // The heading search is local: it can only choose between the headings
    // available from where it already is, so it can be walked into a corridor
    // whose exits are all bad several hundred metres earlier. Scoring the
    // finished candidate is the only place that shows up, and marchRoute()
    // already runs seven independent variants precisely so that one of them can
    // be thrown away. This makes "seven attempts, keep the best" mean something
    // about safety as well as about length and drop.
    //
    // Sampled every 4th polyline point (24 m) rather than at every one: the term
    // is a property of a stretch of trail, and the march is re-scored every 6 m
    // anyway. `route.exposed` is reported by build() so the acceptance numbers
    // can be read off the route that was actually chosen.
    let corr = 0, corrN = 0, corrWorst = 0, corrBad = 0;
    for (let k = 4; k < n - 4; k += 4) {
      const dx = route.px[k + 1] - route.px[k - 1], dz = route.pz[k + 1] - route.pz[k - 1];
      const l = Math.max(1e-4, Math.hypot(dx, dz));
      const ph = PHASES[route.pph ? route.pph[k] : 0];
      corridorAt(terrain, route.px[k], route.pz[k], dx / l, dz / l, ph.width * 0.5, cTmp);
      const c = cTmp.cost > CORR_CAP ? CORR_CAP : cTmp.cost;
      corr += c; corrN++;
      if (cTmp.deficit > 0) corrBad++;
      if (cTmp.deficit > corrWorst) corrWorst = cTmp.deficit;
    }
    route.corrMean = corrN ? corr / corrN : 0;
    route.corrWorst = corrWorst;
    route.corrBad = corrBad;
    route.marchedLength = travelled;
    route.marchedDrop = dropGot;
    // Gradient of the first 90 m of the marched line — the Start Chute's own
    // fall line, and the number that showed the launch scan had traded exposure
    // for a face the opening phase cannot use. See the launch scan in marchRoute.
    {
      const k = Math.min(n - 1, 15);
      let d = 0;
      for (let j = 1; j <= k; j++) d += Math.hypot(route.px[j] - route.px[j - 1], route.pz[j] - route.pz[j - 1]);
      route.openGrade = d > 1 ? (terrainH(terrain, route.px[0], route.pz[0])
        - terrainH(terrain, route.px[k], route.pz[k])) / d : 0;
    }
    // Climbing is the disqualifier; the rest is fine tuning.
    // The corridor weight is set so that a route averaging the full CORR_CAP
    // gives up about as much as a 25% length error — a big number, because a
    // route that is unprotectable along its length is not a course, and a small
    // enough one that it cannot win against a route that climbs.
    return -climb * 9 - lenErr * 90 - dropErr * 70 - route.corrMean * 0.20;
  }

  /** Laplacian smoothing of the marched polyline in XZ, endpoints pinned. */
  function smoothPolyline(px, pz, passes, weight) {
    const n = px.length;
    const ax = Float64Array.from(px), az = Float64Array.from(pz);
    const bx = new Float64Array(n), bz = new Float64Array(n);
    for (let p = 0; p < passes; p++) {
      bx[0] = ax[0]; bz[0] = az[0];
      bx[n - 1] = ax[n - 1]; bz[n - 1] = az[n - 1];
      for (let i = 1; i < n - 1; i++) {
        bx[i] = ax[i] + ((ax[i - 1] + ax[i + 1]) * 0.5 - ax[i]) * weight;
        bz[i] = az[i] + ((az[i - 1] + az[i + 1]) * 0.5 - az[i]) * weight;
      }
      ax.set(bx); az.set(bz);
    }
    return { x: ax, z: az };
  }

  // =========================================================================
  // 2. STATIONS — resample, elevation shaping, curvature
  // =========================================================================

  function gaussianSmooth(src, radius, sigma) {
    const n = src.length;
    const k = new Float32Array(radius * 2 + 1);
    let sum = 0;
    for (let i = -radius; i <= radius; i++) {
      const w = Math.exp(-(i * i) / (2 * sigma * sigma));
      k[i + radius] = w; sum += w;
    }
    for (let i = 0; i < k.length; i++) k[i] /= sum;
    const out = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      let acc = 0;
      for (let j = -radius; j <= radius; j++) {
        const idx = clamp(i + j, 0, n - 1);
        acc += src[idx] * k[j + radius];
      }
      out[i] = acc;
    }
    return out;
  }

  /**
   * Enforce the trail-design descent rule: never climb more than ~12 m in a run,
   * and never at more than ~4%. Anything steeper uphill gets cut down to grade.
   */
  /**
   * CONTRACT §4: "the trail must never climb for more than ~15 m". Measured the
   * way a rider would feel it — how far above the lowest point already passed you
   * can ever be — not as a sum of every little rise. Two rules:
   *   1. no single step may rise faster than `maxUpGrade`;
   *   2. no point may sit more than `maxClimb` above the running low.
   */
  function enforceDescent(y, ds, raw, maxCut) {
    const n = y.length;
    const maxClimb = 7.0;
    const maxUpGrade = 0.035;
    let low = y[0];
    for (let i = 1; i < n; i++) {
      let v = y[i];
      const cap = y[i - 1] + maxUpGrade * ds;
      if (v > cap) v = cap;
      if (v > low + maxClimb) v = low + maxClimb;
      // Excavation limit wins the tie. Without it, a route that climbs 80 m would
      // be "fixed" by cutting an 80 m trench through the mountain, which is far
      // worse than the trail going over a rise. Route selection is what actually
      // keeps the climb down; this is the backstop.
      if (raw && v < raw[i] - maxCut) v = raw[i] - maxCut;
      y[i] = v;
      if (v < low) low = v;
    }
  }

  // -------------------------------------------------------------------------
  // LOCAL GRADE DISCIPLINE
  //
  // THE DEFECT. `PHASES` gives every section a design gradient, and the route
  // march steers against it — but only on AVERAGE, over probes 6/24/66 m long,
  // with an altitude governor allowed to double the target. Measured on seed
  // 20260726 before this pass, the Start Chute (design 11%, tech 0.05, i.e. the
  // easy opening) ran 0.4% at 60 m to 54% at 88 m and back to 4% at 103 m. A
  // phase's design gradient was a long-run average and nothing else.
  //
  // That is not a cosmetic complaint. A 0.4% -> 54% -> 4% profile is a convex
  // crest followed by a concave compression, and the bike's own lean ceiling is
  // a function of tyre load: over the crest the wheels go light, the ceiling
  // collapses (measured 0.63 rad -> 0.26 rad through exactly this stretch), and
  // the rider cannot command the lean the corner underneath needs. Every cell
  // of the autopilot sweep died between 79 and 98 m.
  //
  // THE RULE. Within each phase, redistribute the drop so the LOCAL gradient
  // stays inside a ceiling derived from the phase's own design value, while
  // preserving that phase's total drop to the millimetre. Capping the peaks
  // therefore steepens the flats: the pitch at 88 m comes down and the 0.4%
  // section at 60 m picks up the altitude it gave away. Nothing is flattened
  // overall — the run's descent, and every phase's descent, is bit-preserved.
  //
  // The ceiling is the larger of "the design gradient plus room to breathe" and
  // "a bit more than this phase's own mean", because a phase whose route came
  // out steeper than designed cannot be held to the design number without an
  // excavation nobody would dig. And the whole thing is clamped against the
  // same cut/fill budget as everything else, so on ground that genuinely falls
  // away faster than the budget can follow the rule simply does what it can.
  // -------------------------------------------------------------------------
  const GRADE_LOCAL_K = 1.55;     // × the phase design grade ...
  const GRADE_LOCAL_ADD = 0.045;  // ... plus this = the local ceiling
  const GRADE_MEAN_K = 1.15;      // never tighter than this × the phase's own mean
  const GRADE_MEAN_ADD = 0.020;
  const GRADE_HARD_MAX = 0.46;    // the steepest pitch the course may ever hold
  const GRADE_FLOOR = 0.004;      // a station may be near-flat, never uphill
  const GRADE_WIN = 6.0;          // m — the window the local gradient is measured over
  // THE FILL BUDGET IS NOT THE ELEVATION SOLVE'S FILL BUDGET, and this is the
  // most important constant in the pass.
  //
  // Redistributing drop inside a phase means cutting the steep parts and
  // RAISING the flat parts, and a raised tread is an embankment. The elevation
  // solve allows 3.0 m of fill — but terrain.applyCarve() does not build it:
  // its batter caps fill at FILL_MAX 0.7 m and refuses entirely by 1.4 m,
  // deliberately, because a bench across a fall-away is built full-cut and not
  // by throwing an embankment over the void. So a design line sitting 2 m above
  // natural ground is not a trail, it is a request the terrain declines, and
  // what gets committed is a hole where the tread was drawn.
  //
  // Measured with the shaping pass on the elevation solve's own 3.0 m budget:
  // 1056 of 6602 stations (16.0%) committed more than the fork's 170 mm of
  // travel BELOW the design line, against 168 (2.5%) before the pass existed,
  // and bucketing the error against every candidate driver put all of it on
  // one — stations needing more than 1 m of fill averaged 0.188 m of shortfall
  // with a p95 of 0.593 m, while bank, plan radius and shelf presence showed
  // nothing. The pass was drawing embankments and the mountain was declining
  // to build them, and the rider was riding into the difference.
  //
  // 0.65 m is inside what the batter honours at full strength. Cutting is
  // unaffected: a cut is always granted, so the ceiling on the steep half of
  // the redistribution stays the full excavation budget.
  const GRADE_FILL_MAX = 0.65;

  /** Box-smooth a per-station drop array over `r` stations, edges clamped. */
  function smoothDrop(src, r) {
    const n = src.length;
    const out = new Float64Array(n);
    const w = 2 * r + 1;
    for (let i = 0; i < n; i++) {
      let acc = 0;
      for (let j = -r; j <= r; j++) acc += src[clamp(i + j, 0, n - 1)];
      out[i] = acc / w;
    }
    return out;
  }

  /**
   * Cap the local gradient of `y` inside every phase, preserving each phase's
   * total drop exactly. `raw` is natural ground; the result stays inside the
   * same cut/fill budget as the rest of the elevation solve.
   */
  function shapePhaseGrade(y, raw, phase, maxCut, maxFill) {
    const n = y.length;
    const ds = STATION_DS;
    const r = Math.max(1, Math.round(GRADE_WIN / (2 * ds)));
    // phase spans
    let a = 0;
    while (a < n - 1) {
      let b = a;
      while (b + 1 < n && phase[b + 1] === phase[a]) b++;
      const span = b - a;
      if (span >= 24) {
        const ph = PHASES[Math.min(phase[a], PHASES.length - 1)];
        const total = y[a] - y[b];
        const meanG = total / (span * ds);
        // The ceiling. Never below the phase's own mean or the problem has no
        // solution; never above GRADE_HARD_MAX whatever the mean says.
        const cap = clamp(
          Math.max(ph.grade * GRADE_LOCAL_K + GRADE_LOCAL_ADD,
            meanG * GRADE_MEAN_K + GRADE_MEAN_ADD),
          meanG * 1.02 + 1e-4, GRADE_HARD_MAX);
        const dy = new Float64Array(span);
        for (let k = 0; k < span; k++) dy[k] = y[a + k] - y[a + k + 1];

        for (let outer = 0; outer < 6; outer++) {
          for (let pass = 0; pass < 40; pass++) {
            const g = smoothDrop(dy, r);
            let moved = 0;
            for (let k = 0; k < span; k++) {
              const gk = g[k] / ds;
              if (gk > cap) { const f = cap / gk; dy[k] *= f; moved = 1; }
              else if (gk < GRADE_FLOOR) { dy[k] += (GRADE_FLOOR - gk) * ds; moved = 1; }
            }
            // Restore the phase's total drop exactly, spreading the correction
            // onto whichever stations have headroom in the right direction.
            let sum = 0;
            for (let k = 0; k < span; k++) sum += dy[k];
            const deficit = total - sum;
            if (Math.abs(deficit) > 1e-9) {
              let head = 0;
              const h = new Float64Array(span);
              for (let k = 0; k < span; k++) {
                h[k] = deficit > 0 ? Math.max(0, cap * ds - dy[k])
                  : Math.max(0, dy[k] - GRADE_FLOOR * ds);
                head += h[k];
              }
              if (head > 1e-9) {
                for (let k = 0; k < span; k++) dy[k] += deficit * (h[k] / head);
              } else {
                for (let k = 0; k < span; k++) dy[k] += deficit / span;
              }
              moved = 1;
            }
            if (!moved) break;
          }
          // Integrate the drops back into a profile, then clamp each station
          // against the excavation budget.
          //
          // THE INTEGRATION IS OPEN-LOOP AND THAT IS THE WHOLE POINT. The
          // obvious implementation — walk forward, clamp, carry the clamped
          // value into the next step — feeds every clamp into everything
          // downstream, so the drop a clamped station could not take piles up
          // and lands on the phase's LAST segment, which has nowhere to pass it
          // on. Measured that way: a 6.2 m vertical step appeared at 2558 m,
          // and the sprint phase's worst local gradient went 39.9% -> 88.8% —
          // the pass manufacturing the exact defect it exists to remove.
          //
          // Integrating from the phase's start height with the UNCLAMPED drops
          // and clamping each station independently keeps the target profile
          // exact (its last value is y[b] by construction, since the drops sum
          // to the phase total) and confines a budget violation to the station
          // that has it. Re-reading `dy` afterwards lets the next pass's
          // water-fill spread that station's shortfall over its neighbours.
          let acc = y[a];
          for (let k = 0; k < span - 1; k++) {
            acc -= dy[k];
            y[a + k + 1] = clamp(acc, raw[a + k + 1] - maxCut, raw[a + k + 1] + maxFill);
          }
          for (let k = 0; k < span; k++) dy[k] = y[a + k] - y[a + k + 1];
        }
      }
      a = b + 1;
    }
  }

  // -------------------------------------------------------------------------
  // PLAN RADIUS DISCIPLINE
  //
  // The third lever on the lean identity, after bank and speed: open the
  // corner. The route march produces its heading in 6 m steps with a turn
  // authority of up to 0.56 rad per step, and its switchback mode is an
  // explicit 6.5 m-radius reversal — so the resampled centreline routinely
  // carries 13-16 m plan radii, including in the Start Chute, whose design
  // gradient is 11% and whose `tech` weighting is 0.05, i.e. the easy opening.
  //
  // A 14 m radius is not by itself a defect; a bicycle turns inside that. Two
  // things make it one here. It drives the required lean up as 1/r, so it is
  // the term that puts a corner past 45 deg in the first place. And it is
  // below what any lookahead-based rider — human or controller — can hold a
  // line through: overshoot on corner exit was measured at 3-5 m against a
  // 1.5 m tread half-width, which on the exposed side of a rib is the whole
  // failure.
  //
  // So: relax the plan alignment wherever the radius is under the phase's
  // minimum, with a hard cap on how far the line may move from the route the
  // march chose. The cap is what keeps this a trail-design rule rather than a
  // rewrite of the route — the march's corridor keeping, switchback placement
  // and fall-line reading all survive; the corners it turns get widened by up
  // to PLAN_SHIFT_MAX and no further. Where that is not enough, the speed
  // solve takes the corner speed down instead, which is the honest answer.
  //
  // Per-phase minima are the character knob. `roots`, `loam`, `slab` and
  // `rocks` keep genuinely tight turns — they are meant to be technical and
  // they are ridden slowly. The open, fast phases get radii a rider can carry
  // speed through.
  const PLAN_MIN_R = {
    start: 27, roots: 13, flow: 23, jumps: 27, slab: 15,
    creek: 17, loam: 14, rocks: 15, sprint: 25,
  };
  const PLAN_SHIFT_MAX = 3.6;    // m the alignment may move from the marched line
  const PLAN_STRIDE = 5.0;       // m — the stride the plan curvature is measured over
  // Below this the corner is one of the march's authored switchbacks (its
  // reversal mode is an explicit 6.5 m radius) and the relaxation leaves it
  // alone. See the hairpin lock in openTightCorners().
  const PLAN_HAIRPIN_R = 8.5;

  /**
   * Widen plan corners tighter than their phase's minimum radius.
   *
   * Laplacian relaxation over a stencil the width of the curvature stride,
   * driven only where the radius is short, displacement-limited against the
   * incoming line, endpoints pinned. Operates in place on px/pz.
   */
  function openTightCorners(px, pz, phase, n) {
    const ox = Float64Array.from(px.subarray(0, n));
    const oz = Float64Array.from(pz.subarray(0, n));
    const m = Math.max(2, Math.round(PLAN_STRIDE / STATION_DS));
    const mS = Math.max(2, Math.round(1.6 / STATION_DS));
    const wgt = new Float32Array(n);
    const planR = (i) => {
      const ax = px[i - m] - px[i], az = pz[i - m] - pz[i];
      const bx = px[i + m] - px[i], bz = pz[i + m] - pz[i];
      const la = Math.hypot(ax, az), lb = Math.hypot(bx, bz);
      const lc = Math.hypot(px[i + m] - px[i - m], pz[i + m] - pz[i - m]);
      const area2 = Math.abs(ax * bz - az * bx);
      if (area2 < 1e-6 || la < 1e-6 || lb < 1e-6) return 4000;
      return (la * lb * lc) / (2 * area2);
    };
    // HAIRPIN LOCK. A switchback is not a defect — the march builds them
    // deliberately, as explicit 6.5 m-radius reversals, and they are the reason
    // the route can hold its design gradient across a steep face at all. This
    // pass must not touch them, for two separate reasons.
    //
    // The first is intent: a 6.5 m switchback ridden at the speed the lean
    // identity allows there is a legitimate, slow, technical corner.
    //
    // The second is that the relaxation is actively WRONG on one. The Laplacian
    // target is the midpoint of the two stencil ends, and past about 100 deg of
    // turn inside the stencil that midpoint has crossed to the inside of the
    // arc, near its centre of curvature — so relaxing toward it sharpens the
    // hairpin instead of opening it. Measured without the lock, the worst plan
    // radius on the course went 5.53 m -> 3.31 m at the same station (277 m):
    // the pass made the one corner it should have left alone materially worse.
    //
    // A per-station guard is not enough either. The violation cone is a full
    // stencil wide, so shoulder stations that pass the guard still drag the
    // apex, and the apex's own measured radius depends on ground a stencil away
    // in each direction. The lock is therefore dilated over two stencil widths.
    // With it: worst radius 5.528 m before, 5.529 m after — the switchback comes
    // through untouched, and the count of stations under their phase minimum
    // still falls 227 -> 114.
    const locked = new Uint8Array(n);
    for (let i = m; i < n - m; i++) {
      if (planR(i) >= PLAN_HAIRPIN_R) continue;
      for (let j = Math.max(0, i - 2 * m); j <= Math.min(n - 1, i + 2 * m); j++) locked[j] = 1;
    }
    for (let i = 0; i < n; i++) if (locked[i]) safety.planLocked++;
    measurePlanRadii(px, pz, phase, n, safety, 'planTightBefore', 'planWorstBefore', 'planWorstAtBefore');
    for (let pass = 0; pass < 500; pass++) {
      wgt.fill(0);
      let any = false;
      for (let i = m; i < n - m; i++) {
        const rMin = PLAN_MIN_R[PHASES[phase[i]].id] || 20;
        // Menger-style radius from the three points a stride apart.
        const ax = px[i - m] - px[i], az = pz[i - m] - pz[i];
        const bx = px[i + m] - px[i], bz = pz[i + m] - pz[i];
        const la = Math.hypot(ax, az), lb = Math.hypot(bx, bz);
        const lc = Math.hypot(px[i + m] - px[i - m], pz[i + m] - pz[i - m]);
        const area2 = Math.abs(ax * bz - az * bx);
        if (area2 < 1e-6 || la < 1e-6 || lb < 1e-6) continue;
        const r = (la * lb * lc) / (2 * area2);
        if (r >= rMin || locked[i]) continue;
        any = true;
        const v = clamp01((rMin - r) / rMin) * 0.75 + 0.25;
        for (let j = Math.max(0, i - m); j <= Math.min(n - 1, i + m); j++) {
          if (locked[j]) continue;
          const w = v * (1 - Math.abs(j - i) / (m + 1));
          if (w > wgt[j]) wgt[j] = w;
        }
      }
      if (!any) break;
      for (let i = 1; i < n - 1; i++) {
        const w = wgt[i];
        if (w <= 0 || locked[i]) continue;
        // The relaxation stencil is deliberately much shorter than the
        // detection stencil (1.6 m against 5 m): a Laplacian step toward the
        // midpoint of two points 5 m away is a large, badly-conditioned move on
        // anything that turns appreciably inside that span. Short steps, many
        // passes — the same operator, run as a diffusion rather than a jump.
        const lo = Math.max(0, i - mS), hi = Math.min(n - 1, i + mS);
        const tx2 = (px[lo] + px[hi]) * 0.5, tz2 = (pz[lo] + pz[hi]) * 0.5;
        let stepX = (tx2 - px[i]) * w * 0.22;
        let stepZ = (tz2 - pz[i]) * w * 0.22;
        // Displacement budget against the marched line.
        //
        // The step is TRUNCATED, never snapped. Snapping a station that has run
        // out of budget onto the boundary circle is what the obvious
        // implementation does, and it is wrong: adjacent stations hit the
        // boundary travelling in different directions, so the projection leaves
        // a crease exactly where the relaxation was working hardest. Measured
        // that way, the pass took the count of under-minimum stations 227 -> 78
        // and the WORST plan radius on the course 5.53 m -> 3.34 m, i.e. it
        // fixed the average by manufacturing a sharper corner than any it
        // removed. Shrinking the step to whatever budget is left instead brings
        // a station smoothly to rest at the cap.
        const d = Math.hypot(px[i] - ox[i], pz[i] - oz[i]);
        const brake = smoothstep(PLAN_SHIFT_MAX, PLAN_SHIFT_MAX * 0.55, d);
        if (brake <= 0) continue;
        stepX *= brake; stepZ *= brake;
        const remain = PLAN_SHIFT_MAX - d;
        const sl = Math.hypot(stepX, stepZ);
        if (sl > remain) { const k = remain / sl; stepX *= k; stepZ *= k; }
        px[i] += stepX; pz[i] += stepZ;
        safety.planShiftMax = Math.max(safety.planShiftMax, Math.hypot(px[i] - ox[i], pz[i] - oz[i]));
      }
    }
  }

  /**
   * Count stations under their phase's minimum plan radius. Written as its own
   * function because the "after" figure MUST be taken on the re-spaced line:
   * openTightCorners() shortens the arcs it relaxes, so the stations inside a
   * corner it has just widened sit closer together than STATION_DS, and a
   * three-point radius taken at a fixed station stride then spans less ground
   * and reads SMALLER. Measured that way the pass appeared to take the worst
   * radius on the course from 5.53 m to 3.31 m while in fact opening it.
   */
  function measurePlanRadii(px, pz, phase, n, into, keyCount, keyWorst, keyAt) {
    const m = Math.max(2, Math.round(PLAN_STRIDE / STATION_DS));
    for (let i = m; i < n - m; i++) {
      const rMin = PLAN_MIN_R[PHASES[phase[i]].id] || 20;
      const ax = px[i - m] - px[i], az = pz[i - m] - pz[i];
      const bx = px[i + m] - px[i], bz = pz[i + m] - pz[i];
      const la = Math.hypot(ax, az), lb = Math.hypot(bx, bz);
      const lc = Math.hypot(px[i + m] - px[i - m], pz[i + m] - pz[i - m]);
      const area2 = Math.abs(ax * bz - az * bx);
      if (area2 < 1e-6 || la < 1e-6 || lb < 1e-6) continue;
      const r = (la * lb * lc) / (2 * area2);
      if (r < rMin) into[keyCount]++;
      if (r < into[keyWorst]) { into[keyWorst] = r; into[keyAt] = i * STATION_DS; }
    }
  }

  /**
   * Re-space a polyline to exactly STATION_DS between stations, in place.
   * openTightCorners() shortens the arcs it relaxes, and every downstream
   * consumer — curvature, the speed solve, `length`, the stamp spacing —
   * assumes uniform spacing. Returns the new station count.
   */
  function respaceXZ(px, pz, n) {
    const cum = new Float64Array(n);
    for (let i = 1; i < n; i++) cum[i] = cum[i - 1] + Math.hypot(px[i] - px[i - 1], pz[i] - pz[i - 1]);
    const total = cum[n - 1];
    const m = Math.max(64, Math.floor(total / STATION_DS) + 1);
    const qx = new Float64Array(m), qz = new Float64Array(m);
    let k = 0;
    for (let i = 0; i < m; i++) {
      const s = Math.min(total, i * STATION_DS);
      while (k < n - 2 && cum[k + 1] < s) k++;
      const seg = Math.max(1e-9, cum[k + 1] - cum[k]);
      const f = clamp01((s - cum[k]) / seg);
      qx[i] = px[k] + (px[k + 1] - px[k]) * f;
      qz[i] = pz[k] + (pz[k + 1] - pz[k]) * f;
    }
    for (let i = 0; i < m; i++) { px[i] = qx[i]; pz[i] = qz[i]; }
    return m;
  }

  /** Phase index per station, mapped by arc length along the marched polyline. */
  function assignPhases(route, n) {
    const phase = new Uint8Array(n);
    const mp = route.pph || [];
    const cum = new Float64Array(route.px.length);
    for (let k = 1; k < route.px.length; k++) {
      cum[k] = cum[k - 1] + Math.hypot(route.px[k] - route.px[k - 1], route.pz[k] - route.pz[k - 1]);
    }
    const total = Math.max(1e-6, cum[cum.length - 1]);
    let k = 0;
    for (let i = 0; i < n; i++) {
      const target = (i / Math.max(1, n - 1)) * total;
      while (k < cum.length - 1 && cum[k + 1] < target) k++;
      phase[i] = mp.length ? mp[Math.min(k, mp.length - 1)] : 0;
    }
    for (let i = 1; i < n; i++) if (phase[i] < phase[i - 1]) phase[i] = phase[i - 1];
    return phase;
  }

  function buildStations(terrain, route) {
    const sm = smoothPolyline(route.px, route.pz, 4, 0.30);
    const ctrl = [];
    for (let i = 0; i < sm.x.length; i++) ctrl.push(new THREE.Vector3(sm.x[i], 0, sm.z[i]));
    const flat = new THREE.CatmullRomCurve3(ctrl, false, 'centripetal', 0.5);
    flat.arcLengthDivisions = Math.max(2000, ctrl.length * 12);
    const flatLen = flat.getLength();

    N = Math.max(64, Math.round(flatLen / STATION_DS) + 1);
    // Over-allocate: openTightCorners() shortens the line, so respaceXZ() only
    // ever returns fewer stations than it was handed.
    const px = new Float32Array(N), pz = new Float32Array(N);
    const pts = flat.getSpacedPoints(N - 1);
    for (let i = 0; i < N; i++) { px[i] = pts[i].x; pz[i] = pts[i].z; }

    // --- plan radius discipline (see openTightCorners) ---------------------
    openTightCorners(px, pz, assignPhases(route, N), N);
    N = respaceXZ(px, pz, N);
    measurePlanRadii(px, pz, assignPhases(route, N), N, safety,
      'planTightAfter', 'planWorstAfter', 'planWorstAtAfter');
    // Trim to the new station count: S.px/S.pz are read by other modules and a
    // trailing tail of stale coordinates past S.n would be a live foot-gun.
    const sx = px.slice(0, N), sz = pz.slice(0, N);

    // --- elevation: sample, bench-smooth, enforce descent, limit cut/fill ---
    let by = new Float32Array(N);
    for (let i = 0; i < N; i++) by[i] = terrainH(terrain, sx[i], sz[i]);
    const raw = Float32Array.from(by);
    // THE EXCAVATION LIMIT IS APPLIED AGAINST A SMOOTHED NATURAL PROFILE, NOT
    // THE SAMPLED ONE, and that distinction turned out to matter more than any
    // other line in the elevation solve.
    //
    // `raw` is the mountain sampled every 0.4 m, so it carries the full
    // roughness of the ground — tenths of a metre at metre wavelengths. Clamp
    // the design line to `raw - MAX_CUT` and, anywhere the clamp binds, the
    // tread inherits that roughness verbatim: the line stops being a designed
    // profile and becomes a rigid offset copy of a rocky face.
    //
    // Measured at 70-95 m on seed 20260726, where holding a sane gradient needs
    // more than the 7 m excavation ceiling: the design line sat pinned at
    // raw-7.0 and its gradient oscillated 0% / 54% / 3% / 49% / 10% / 43%
    // station to station, a washboard at a 1-2 m wavelength. The bike hopped
    // clean off it at 10 m/s — every cell of the sweep left the tread there.
    //
    // An excavation ceiling is a statement about how much material a builder
    // will move, which is a property of a stretch of trail and not of one
    // 0.4 m sample. Measuring it against the ground smoothed over ~10 m says
    // exactly that, and leaves the clamp surface smooth enough to be a trail.
    const rawS = gaussianSmooth(raw, 25, 9);

    // Bench smoothing (a cut trail rides through small terrain noise), then the
    // descent rule, then a cut/fill limit so we never end up in a trench. The
    // order matters: enforceDescent only ever *lowers* the line, so the cut limit
    // has to come after it — and then we re-run the descent rule on the clamped
    // line, iterating until both hold. Four passes is enough to converge.
    const MAX_CUT = 6.5, MAX_FILL = 3.0;
    by = gaussianSmooth(by, 22, 8);       // ~9 m bench smoothing
    for (let pass = 0; pass < 4; pass++) {
      enforceDescent(by, STATION_DS, rawS, MAX_CUT);
      for (let i = 0; i < N; i++) by[i] = clamp(by[i], rawS[i] - MAX_CUT, rawS[i] + MAX_FILL);
      by = gaussianSmooth(by, 8, 3.0);
    }
    enforceDescent(by, STATION_DS, rawS, MAX_CUT);
    by = gaussianSmooth(by, 4, 1.6);
    enforceDescent(by, STATION_DS, rawS, MAX_CUT + 0.6);

    // --- phase assignment: inherited from the march ------------------------
    // Hoisted above the elevation shaping: shapePhaseGrade() needs to know which
    // phase each station belongs to, and the assignment depends only on arc
    // length along the marched polyline, never on height.
    const phase = assignPhases(route, N);

    // --- local grade discipline (see shapePhaseGrade) ----------------------
    shapePhaseGrade(by, rawS, phase, MAX_CUT, GRADE_FILL_MAX);
    for (let i = 0; i < N; i++) by[i] = clamp(by[i], rawS[i] - MAX_CUT, rawS[i] + GRADE_FILL_MAX);
    by = gaussianSmooth(by, 5, 2.0);
    shapePhaseGrade(by, rawS, phase, MAX_CUT, GRADE_FILL_MAX);
    for (let i = 0; i < N; i++) by[i] = clamp(by[i], rawS[i] - MAX_CUT, rawS[i] + GRADE_FILL_MAX);
    enforceDescent(by, STATION_DS, rawS, MAX_CUT + 0.6);

    // --- tangents, right vectors, grade, curvature -------------------------
    const tx = new Float32Array(N), ty = new Float32Array(N), tz = new Float32Array(N);
    const rx = new Float32Array(N), rz = new Float32Array(N);
    const grade = new Float32Array(N);
    const curv = new Float32Array(N);
    const radius = new Float32Array(N);

    for (let i = 0; i < N; i++) {
      const a = Math.max(0, i - 1), b = Math.min(N - 1, i + 1);
      let dx = sx[b] - sx[a], dz = sz[b] - sz[a], dy = by[b] - by[a];
      const l = Math.max(1e-5, Math.hypot(dx, dy, dz));
      tx[i] = dx / l; ty[i] = dy / l; tz[i] = dz / l;
      const hl = Math.max(1e-5, Math.hypot(tx[i], tz[i]));
      rx[i] = -tz[i] / hl; rz[i] = tx[i] / hl;
      grade[i] = -ty[i];                      // + = descending
    }
    // Signed curvature via dTangent · right, measured over a 5 m stride so tiny
    // resampling wobble doesn't create phantom berms.
    const stride = Math.max(2, Math.round(5 / STATION_DS));
    for (let i = 0; i < N; i++) {
      const a = clamp(i - stride, 0, N - 1), b = clamp(i + stride, 0, N - 1);
      const ds = (b - a) * STATION_DS;
      if (ds < 1e-3) { curv[i] = 0; radius[i] = 1e4; continue; }
      const dtx = tx[b] - tx[a], dtz = tz[b] - tz[a];
      const k = (dtx * rx[i] + dtz * rz[i]) / ds;
      curv[i] = k;
      radius[i] = 1 / Math.max(1e-4, Math.abs(k));
    }
    const curvS = gaussianSmooth(curv, 10, 4);
    for (let i = 0; i < N; i++) {
      curv[i] = curvS[i];
      radius[i] = clamp(1 / Math.max(1e-4, Math.abs(curvS[i])), 4, 4000);
    }

    // The phase array was assigned before the elevation shaping (above) because
    // shapePhaseGrade() is written against it.
    const totalLen = (N - 1) * STATION_DS;

    S = {
      n: N, ds: STATION_DS,
      px: sx, py: Float32Array.from(by), pz: sz, by,
      raw,                             // natural ground under the centreline, pre-shaping
      rawS,                            // ... smoothed over ~10 m; the excavation limits are measured against THIS
      tx, ty, tz, rx, rz,
      grade, curv, radius, phase,
      width: new Float32Array(N),
      bank: new Float32Array(N),
      speed: new Float32Array(N),
      surface: new Uint8Array(N),
      hOff: new Float32Array(N),       // feature height offset above the base grade
      rut: new Float32Array(N),        // rut depth (m)
      crown: new Float32Array(N),      // tread crown (m)
      rough: new Float32Array(N),      // micro-roughness amplitude (m)
      bumps: new Float32Array(N),      // transverse ridge amplitude (roots/brake bumps)
      feat: new Int16Array(N),
      wallride: new Float32Array(N),   // extra wall height on the outside of the turn
      // ---- exposure safety (applyExposureSafety) --------------------------
      expoL: new Float32Array(N),      // m of unsupported fall outside the left edge
      expoR: new Float32Array(N),      //   ... and the right
      shelfL: new Float32Array(N),     // m of built run-off shelf on the left
      shelfR: new Float32Array(N),
      catchL: new Float32Array(N),     // m of catch berm at the outer lip of that shelf
      catchR: new Float32Array(N),
      benchShift: new Float32Array(N), // m the line was benched (+ = toward rider's right)
      exposed: new Uint8Array(N),      // 1 where the rule fired at all
      taped: new Uint8Array(N),        // 1 where exposure forces tape in an untaped phase
    };
    S.feat.fill(-1);
    length = totalLen;

    // Base per-station width/surface/wear from the phase, with organic variation.
    for (let i = 0; i < N; i++) {
      const ph = PHASES[phase[i]];
      const wob = fbm1(microN, i * STATION_DS * 0.05, 3, 2.0, 0.5);
      S.width[i] = clamp(ph.width + wob * 0.28, 1.2, 3.0);
      S.surface[i] = ph.surface;
      S.crown[i] = 0.035 + wob * 0.012;
      S.rut[i] = clamp(0.030 + 0.05 * ph.tech + wob * 0.02, 0.012, 0.10);
      S.rough[i] = 0.012 + 0.05 * ph.tech;
    }

    // Blend phase transitions over 25 m so width/surface don't step.
    const wS = gaussianSmooth(S.width, 30, 12);
    S.width.set(wS);

    // Build the CatmullRom curve other modules consume (decimated for speed).
    const decim = Math.max(1, Math.round(4 / STATION_DS));
    const cpts = [];
    for (let i = 0; i < N; i += decim) cpts.push(new THREE.Vector3(sx[i], S.py[i], sz[i]));
    const lastV = new THREE.Vector3(sx[N - 1], S.py[N - 1], sz[N - 1]);
    if (cpts[cpts.length - 1].distanceTo(lastV) > 0.05) cpts.push(lastV);
    curve = new THREE.CatmullRomCurve3(cpts, false, 'centripetal', 0.5);
    curve.arcLengthDivisions = Math.max(2000, cpts.length * 8);
  }

  // =========================================================================
  // 3. SPEED PROFILE — what the rider actually arrives at
  // =========================================================================

  /**
   * Three-pass racing-line speed solve:
   *   (a) corner speed cap from radius + bank + surface grip,
   *   (b) forward pass with gravity/drag/rolling-resistance/pedalling,
   *   (c) backward pass limited by braking.
   * Run twice, because the berm bank depends on speed and speed depends on bank.
   */
  /**
   * The component of the bank at station i that actually helps the corner.
   *
   * `S.bank` is signed (+ = right side high) and is set to -sign(curv)·mag, so
   * for a correctly-banked corner this returns +mag. It returns a NEGATIVE
   * number where the tread is off-camber — after the bank smoothing runs across
   * a curvature sign change, or where the straight-section outslope tilts the
   * wrong way for a gentle bend — and that is the point: an adverse camber must
   * cost the speed solve exactly as much as a berm gains it. Using |bank| here
   * (which is what the old cap did) credits an off-camber corner with the bank
   * it is losing.
   */
  function helpfulBank(i) {
    const c = S.curv[i];
    if (c === 0) return 0;
    return -Math.sign(c) * S.bank[i];
  }

  /** Berm style for the phase at station i. */
  function styleAt(i) {
    return BERM_STYLE[PHASES[S.phase[i]].id] || BERM_STYLE_DEFAULT;
  }

  /**
   * The banked-corner speed limit that makes the lean identity hold.
   * See the LEAN_DESIGN note at the head of the file for the derivation:
   * with mu = tan(LEAN_DESIGN) this returns the speed at which the rider's
   * lean relative to the tread is exactly LEAN_DESIGN.
   */
  function cornerSpeedCap(i) {
    const mu = clamp(MU_DESIGN * SURF_MU[S.surface[i]], 0.22, 1.0);
    // Clamp the bank contribution: past ~50 deg the formula's denominator runs
    // away, and no rider is relying on a berm steeper than that anyway.
    const tb = Math.tan(clamp(helpfulBank(i), -0.55, 0.87));
    const denom = Math.max(0.20, 1 - mu * tb);
    const num = Math.max(0.02, mu + tb);
    return Math.min(V_CAP, Math.sqrt(Math.max(1, G * S.radius[i] * num / denom)));
  }

  /**
   * The design speed of the section itself, independent of any corner: what a
   * rider carries down a straight of this width, on this surface, at this
   * roughness. See the V_PHASE note.
   */
  function techSpeedCap(i) {
    const ph = PHASES[S.phase[i]];
    const base = V_PHASE[ph.id] === undefined ? 17.0 : V_PHASE[ph.id];
    // Tread width, relative to the phase's own design width. A pinch is slower
    // than the section it is in; a corner that has been widened is faster.
    const wf = clamp(0.74 + 0.30 * (S.width[i] / Math.max(0.8, ph.width)), 0.70, 1.14);
    // Chatter.
    const amp = Math.max(0.015, S.bumps[i] + S.rough[i] * 1.5);
    const rough = ROUGH_SUSP * (ROUGH_LAMBDA / (2 * Math.PI)) * Math.sqrt((2 * G) / amp);
    return Math.min(base * wf, rough);
  }

  /** Clamp the solved speed to the corner and section caps, then re-brake. */
  function applyCornerSpeedCap() {
    const n = S.n, ds = S.ds, v = S.speed;
    for (let i = 0; i < n; i++) {
      const cap = Math.min(cornerSpeedCap(i), techSpeedCap(i));
      if (v[i] > cap) v[i] = cap;
    }
    for (let i = n - 2; i >= 0; i--) {
      const lim = Math.sqrt(v[i + 1] * v[i + 1] + 2 * A_BRAKE * ds);
      if (v[i] > lim) v[i] = lim;
    }
  }

  /**
   * The audit CONTRACT-NOTE'd as `trail.cornerAudit`.
   *
   * One row per corner: the geometry the route-finder produced, the bank and
   * speed the solve settled on, the lean the rider is therefore asked to hold
   * against the tread, and — for every corner that would have VIOLATED the
   * identity had it been banked and paced the old way — what changed.
   *
   * The "before" column is not a guess: it re-runs the old rule on the same
   * geometry. `MU_LAT` 1.18 for the corner cap, and the old per-phase build
   * coefficient (1.0 for flow/sprint/jumps, 0.85 loam, 0.5 creek, 0.42 for
   * everything else) applied to atan(v^2/(g*r)), with no identity floor and no
   * bank at all past a 220 m radius.
   */
  /**
   * Replay the OLD speed/bank rule on the current geometry: the pre-fix corner
   * cap (MU_LAT 1.18), the pre-fix per-phase build coefficient, no identity
   * floor, no bank past a 220 m radius, and NO section speed ceiling — so the
   * forward pass runs gravity against drag exactly as it used to. Returns the
   * speed profile that rule produces.
   *
   * This exists so the audit's "before" column is a measurement rather than an
   * assertion. Taking the current solved speed as the "before" would understate
   * the defect enormously, because the current speed has already been brought
   * down by the very fix being audited.
   */
  function replayOldRule() {
    const n = S.n, ds = S.ds;
    const v = new Float32Array(n);
    const b = new Float32Array(n);
    const vmax = new Float32Array(n);
    for (let iter = 0; iter < 2; iter++) {
      for (let i = 0; i < n; i++) {
        const mu = MU_LAT * SURF_MU[S.surface[i]];
        const tb = Math.tan(Math.abs(b[i]));
        vmax[i] = Math.min(V_CAP, Math.sqrt(Math.max(1,
          G * S.radius[i] * (mu + tb) / Math.max(0.15, 1 - mu * tb))));
      }
      v[0] = 3.0;
      for (let i = 1; i < n; i++) {
        const gI = clamp(S.grade[i - 1], -0.6, 0.6);
        const sinT = gI / Math.sqrt(1 + gI * gI);
        const cosT = 1 / Math.sqrt(1 + gI * gI);
        const vp = Math.max(1.0, v[i - 1]);
        const crr = SURF_CRR[S.surface[i - 1]];
        const ph = PHASES[S.phase[i - 1]];
        const pedal = (ph.id === 'start' || ph.id === 'sprint') && vp < 15
          ? PEDAL_POWER / (RIDER_MASS * vp) : 0;
        const drag = 0.5 * AIR_RHO * CDA * vp * vp / RIDER_MASS;
        const a = G * sinT - crr * G * cosT - drag + pedal;
        v[i] = Math.sqrt(Math.max(1.0, vp * vp + 2 * a * ds));
        if (v[i] > vmax[i]) v[i] = vmax[i];
      }
      for (let i = n - 2; i >= 0; i--) {
        const lim = Math.sqrt(v[i + 1] * v[i + 1] + 2 * A_BRAKE * ds);
        if (v[i] > lim) v[i] = lim;
        if (v[i] > vmax[i]) v[i] = vmax[i];
      }
      for (let i = 0; i < n; i++) {
        const ph = PHASES[S.phase[i]];
        const r = S.radius[i];
        if (r > 220) { b[i] = 0; continue; }
        const build = ph.id === 'flow' || ph.id === 'sprint' || ph.id === 'jumps' ? 1.0
          : ph.id === 'loam' ? 0.85 : ph.id === 'creek' ? 0.5 : 0.42;
        b[i] = clamp(Math.atan((v[i] * v[i]) / (G * r)) * build, 0, THREE.MathUtils.degToRad(46));
      }
      const bS = gaussianSmooth(b, 16, 6);
      b.set(bS);
    }
    return { v, b };
  }

  function auditCornerIdentity() {
    cornerAudit.length = 0;
    const n = S.n;
    const old = replayOldRule();
    const seen = [];
    // Local curvature maxima, the same net the acceptance harness casts.
    let i = 0;
    const gap = M(8);
    while (i < n) {
      if (S.radius[i] < 90) {
        let j = i, best = i;
        while (j < n && S.radius[j] < 90 * 1.35 && Math.sign(S.curv[j]) === Math.sign(S.curv[i])) {
          if (S.radius[j] < S.radius[best]) best = j;
          j++;
        }
        seen.push(best);
        i = Math.max(j, best + gap);
      } else i++;
    }
    for (const k of seen) {
      const ph = PHASES[S.phase[k]];
      const r = S.radius[k], v = S.speed[k];
      const bank = helpfulBank(k);
      const req = Math.atan((v * v) / (G * r));
      const vB = old.v[k];
      const bOld = old.b[k];
      const reqB = Math.atan((vB * vB) / (G * r));
      cornerAudit.push({
        s: +(k * S.ds).toFixed(1),
        phase: ph.id,
        radius: +r.toFixed(1),
        before: {
          speed: +vB.toFixed(2),
          bankDeg: +THREE.MathUtils.radToDeg(bOld).toFixed(1),
          requiredLeanDeg: +THREE.MathUtils.radToDeg(reqB).toFixed(1),
          contactLeanDeg: +THREE.MathUtils.radToDeg(reqB - bOld).toFixed(1),
          violated: (reqB - bOld) > LEAN_AVAIL - THREE.MathUtils.degToRad(6),
          impossible: (reqB - bOld) > LEAN_AVAIL,
        },
        after: {
          speed: +v.toFixed(2),
          bankDeg: +THREE.MathUtils.radToDeg(bank).toFixed(1),
          requiredLeanDeg: +THREE.MathUtils.radToDeg(req).toFixed(1),
          contactLeanDeg: +THREE.MathUtils.radToDeg(req - bank).toFixed(1),
          violated: (req - bank) > LEAN_AVAIL - THREE.MathUtils.degToRad(6),
        },
        // What the fix actually did here.
        change: {
          bankDeg: +THREE.MathUtils.radToDeg(bank - bOld).toFixed(1),
          speed: +(v - vB).toFixed(2),
          leanRelieved: +THREE.MathUtils.radToDeg((reqB - bOld) - (req - bank)).toFixed(1),
        },
      });
    }
  }

  function solveSpeeds(rebank = true) {
    const n = S.n, ds = S.ds;
    const vmax = new Float32Array(n);
    const v = S.speed;
    // Three iterations, not two. The bank depends on the speed and the speed
    // depends on the bank, and raising a corner's bank raises its speed cap,
    // which raises the ideal bank again; two passes left a measurable residual
    // on the tightest corners.
    const iters = rebank ? 3 : 1;

    for (let iter = 0; iter < iters; iter++) {
      for (let i = 0; i < n; i++) vmax[i] = Math.min(cornerSpeedCap(i), techSpeedCap(i));
      // Forward — physics-limited acceleration.
      v[0] = 3.0;
      for (let i = 1; i < n; i++) {
        const gI = clamp(S.grade[i - 1], -0.6, 0.6);
        const sinT = gI / Math.sqrt(1 + gI * gI);
        const cosT = 1 / Math.sqrt(1 + gI * gI);
        const vp = Math.max(1.0, v[i - 1]);
        const crr = SURF_CRR[S.surface[i - 1]];
        const ph = PHASES[S.phase[i - 1]];
        const pedal = (ph.id === 'start' || ph.id === 'sprint') && vp < 15
          ? PEDAL_POWER / (RIDER_MASS * vp) : 0;
        const drag = 0.5 * AIR_RHO * CDA * vp * vp / RIDER_MASS;
        const a = G * sinT - crr * G * cosT - drag + pedal;
        v[i] = Math.sqrt(Math.max(1.0, vp * vp + 2 * a * ds));
        if (v[i] > vmax[i]) v[i] = vmax[i];
      }
      // Backward — braking-limited entry speeds.
      for (let i = n - 2; i >= 0; i--) {
        const lim = Math.sqrt(v[i + 1] * v[i + 1] + 2 * A_BRAKE * ds);
        if (v[i] > lim) v[i] = lim;
        if (v[i] > vmax[i]) v[i] = vmax[i];
      }
      // Bank the corners for the speed we just solved. Skipped on the second solve,
      // which runs after the features have deliberately flattened jump lips and
      // over-banked the wallride.
      if (!rebank) continue;
      // ---- bank every corner, in every phase -----------------------------
      for (let i = 0; i < n; i++) {
        const st = styleAt(i);
        const r = S.radius[i];
        // atan(v²/(g·r)) is the bank that would make the corner feel flat.
        const ideal = Math.atan((v[i] * v[i]) / (G * r));
        // The identity floor: the bank this corner NEEDS so the rider is not
        // asked for more than LEAN_DESIGN of lean against the tread.
        const need = ideal - LEAN_DESIGN;
        // A genuinely straight section is left flat (the outslope pass below
        // gives it its drainage tilt). "Straight" now means BOTH slack in the
        // identity and slacker than the phase's own berm threshold — the old
        // test was a bare `r > 220`, which left a 240 m-radius bend taken at
        // 20 m/s with no bank at all and 24 deg of lean to find from nowhere.
        if (need <= 0.015 && r > st.minRadius * 4.5) { S.bank[i] = 0; continue; }
        const mag = clamp(Math.max(ideal * st.style, need), 0, st.maxBank);
        S.bank[i] = -Math.sign(S.curv[i]) * mag;
      }
      // Smooth the bank along the trail so a berm builds and releases instead
      // of switching on at one station, THEN re-solve the speed against the
      // smoothed bank on the next iteration. The final speed clamp after the
      // loop closes the gap for the last iteration.
      const bS = gaussianSmooth(S.bank, 16, 6);
      S.bank.set(bS);
    }

    if (!rebank) { applyCornerSpeedCap(); auditCornerIdentity(); return; }
    // Straight sections get a builder's outslope for drainage: always tilted to
    // whichever side the hill falls away.
    for (let i = 0; i < S.n; i++) {
      if (Math.abs(S.bank[i]) > 0.05) continue;
      const e = 4;
      const hR = terrainSampleCache(S.px[i] + S.rx[i] * e, S.pz[i] + S.rz[i] * e);
      const hL = terrainSampleCache(S.px[i] - S.rx[i] * e, S.pz[i] - S.rz[i] * e);
      const csRight = (hR - hL) / (2 * e);
      const out = Math.sign(csRight) * 0.030;
      S.bank[i] = S.bank[i] * 0.4 + out * 0.6;
    }
    // The outslope pass and the bank smoothing both run AFTER the last speed
    // solve, and either can leave a station with less helpful bank than the
    // speed was solved for. Close the loop: nothing below this line may raise
    // a corner's speed, so the identity holds against the bank that is
    // actually built.
    applyCornerSpeedCap();
  }

  // Terrain reference captured during build (only used inside build()).
  let _terrainRef = null;
  function terrainSampleCache(x, z) {
    return _terrainRef ? terrainH(_terrainRef, x, z) : 0;
  }

  // =========================================================================
  // 4. FEATURES
  // =========================================================================

  /**
   * Station span of a phase, or null if the route was too short to reach it.
   * Callers must skip on null — a degenerate span would stack every feature of
   * the run on top of each other at t=0.
   */
  function stationRangeForPhase(id) {
    let a = -1, b = -1;
    const pi = PHASES.findIndex((p) => p.id === id);
    for (let i = 0; i < S.n; i++) {
      if (S.phase[i] === pi) { if (a < 0) a = i; b = i; }
    }
    if (a < 0 || (b - a) < M(24)) return null;
    return [a, b];
  }

  const M = (m) => Math.max(1, Math.round(m / STATION_DS));   // metres -> stations

  function addFeature(type, i0, i1, params, force) {
    const a = clamp(Math.min(i0, i1), 0, S.n - 1);
    const b = clamp(Math.max(i0, i1), 0, S.n - 1);
    const f = {
      id: `${type}-${features.length + 1}`,
      type,
      tStart: a / (S.n - 1),
      tEnd: b / (S.n - 1),
      position: new THREE.Vector3(
        S.px[(a + b) >> 1], S.py[(a + b) >> 1], S.pz[(a + b) >> 1]),
      params: params || {},
      i0: a, i1: b,
    };
    features.push(f);
    const fi = features.length - 1;
    // Shaped features (jumps, drops) claim their stations from any broad feature
    // that already covers them — the stamp `kind` depends on it.
    for (let i = a; i <= b; i++) if (force || S.feat[i] < 0) S.feat[i] = fi;
    return f;
  }

  /** Straightest, fastest station within `win` of `i0` — where a jump belongs. */
  function straightestNear(i0, win, lo, hi) {
    let best = clamp(i0, lo, hi), bestScore = -Infinity;
    for (let d = -win; d <= win; d++) {
      const i = clamp(i0 + d, lo, hi);
      // Straight, fast, and not already occupied by another shaped feature.
      let sc = Math.min(S.radius[i], 300) * 0.06 + S.speed[i] * 2.2 - Math.abs(d) * 0.02;
      if (S.feat[i] >= 0) sc -= 400;
      if (sc > bestScore) { bestScore = sc; best = i; }
    }
    return best;
  }

  /** Local curvature maxima -> the corners a builder would berm. */
  function findCorners(i0, i1, minRadius, minSpacing) {
    const out = [];
    let i = i0;
    const gap = M(minSpacing);
    while (i <= i1) {
      if (S.radius[i] < minRadius) {
        let j = i, best = i;
        while (j <= i1 && S.radius[j] < minRadius * 1.35) {
          if (S.radius[j] < S.radius[best]) best = j;
          j++;
        }
        // Extend the berm over the whole arc, not just the apex.
        let a = best, b = best;
        while (a > i0 && S.radius[a - 1] < minRadius * 2.0 && Math.sign(S.curv[a - 1]) === Math.sign(S.curv[best])) a--;
        while (b < i1 && S.radius[b + 1] < minRadius * 2.0 && Math.sign(S.curv[b + 1]) === Math.sign(S.curv[best])) b++;
        out.push({ apex: best, i0: a, i1: b });
        i = Math.max(j, best + gap);
      } else i++;
    }
    return out;
  }

  /** Write a smooth height offset ramp into S.hOff over [i0,i1]. */
  function blendOffset(i0, i1, fn) {
    for (let i = Math.max(0, i0); i <= Math.min(S.n - 1, i1); i++) {
      const f = (i1 === i0) ? 0 : (i - i0) / (i1 - i0);
      S.hOff[i] += fn(f, i);
    }
  }

  /**
   * Detect the corners in one phase and BUILD them: register the berm feature,
   * widen the tread through the arc, deepen the ridden rut, and lay a braking
   * bump train on the run-in.
   *
   * THIS IS THE CALL SITE THAT USED TO EXIST THREE TIMES. findCorners() — and
   * therefore every berm on the course — was invoked only inside `flow`, `loam`
   * and `sprint`. Six of the nine phases (start, roots, jumps, slab, creek,
   * rocks) had no corner detection and no berm construction at all, however
   * tight the route-finder's corners were there, and the global banking pass
   * then multiplied the ideal bank by a per-phase coefficient as low as 0.42.
   * The corners in those phases were not "less bermed"; they were not built.
   *
   * The per-phase numbers live in BERM_STYLE. A tech phase gets tight, modest,
   * worn catch banks and a short braking-bump train; a flow phase gets long
   * sweeping berms. The BANK itself is not set here — solveSpeeds() owns it,
   * because bank, radius and speed have to be solved together for the lean
   * identity to hold — so what this adds is the rest of what a built corner is:
   * the width, the rut, and the chatter on the approach.
   *
   * @param {string} id      phase id
   * @param {object} opt     { margin0, margin1, skipShaped, onCorner }
   * @returns the corners it found, so a phase can pick one out for a wallride.
   */
  function buildPhaseBerms(id, opt = {}) {
    const span = stationRangeForPhase(id);
    if (!span) return [];
    const st = BERM_STYLE[id] || BERM_STYLE_DEFAULT;
    const [a, b] = span;
    const i0 = a + M(opt.margin0 === undefined ? 10 : opt.margin0);
    const i1 = b - M(opt.margin1 === undefined ? 12 : opt.margin1);
    if (i1 - i0 < M(12)) return [];
    const corners = findCorners(i0, i1, st.minRadius, st.minSpacing);
    const out = [];
    for (const c of corners) {
      // A corner that lands on authored relief is left to the feature that owns
      // it: a berm inside a jump's landing transition is not a berm.
      if (opt.skipShaped) {
        let clash = false;
        for (let i = c.i0; i <= c.i1 && !clash; i++) {
          const fi = S.feat[i];
          if (fi >= 0) {
            const t = features[fi].type;
            if (t === 'jump' || t === 'doubles' || t === 'gap' || t === 'stepDown' || t === 'drop') clash = true;
          }
        }
        if (clash) continue;
      }
      if (opt.onCorner && opt.onCorner(c) === false) continue;
      const bank = Math.abs(S.bank[c.apex]);
      addFeature('berm', c.i0, c.i1, {
        radius: S.radius[c.apex],
        bank,
        entrySpeed: S.speed[c.apex],
        // The berm's own height above the tread centre at the outer edge.
        height: clamp(Math.tan(bank) * S.width[c.apex] * 0.5, 0.18, 1.6),
        phase: id,
        style: st.style,
      });
      for (let i = c.i0; i <= c.i1; i++) {
        S.width[i] = clamp(S.width[i] + st.widthAdd, 1.2, 3.0);
        S.rut[i] = Math.max(S.rut[i], st.ruts);
      }
      if (st.brake > 0) addBrakingBumps(c.i0 - M(st.brakeLen), c.i0 - M(1), st.brake);
      out.push(c);
    }
    return out;
  }

  function buildFeatures() {
    // ---------------- phase 1: open pedally start (rollers) ----------------
    phStart: {
      const span = stationRangeForPhase('start');
      if (!span) break phStart;
      const [a, b] = span;
      buildPhaseBerms('start');
      addFeature('rollers', a + M(45), b - M(30), { count: 5, amplitude: 0.32 });
      const r0 = a + M(45), r1 = b - M(30);
      const n = 5;
      const sp = (r1 - r0) / n;
      for (let k = 0; k < n; k++) {
        const c = Math.round(r0 + sp * (k + 0.5));
        const half = Math.round(sp * 0.42);
        const amp = 0.26 + featRng() * 0.14;
        blendOffset(c - half, c + half, (f) => amp * (0.5 - 0.5 * Math.cos(f * Math.PI * 2)));
      }
      for (let i = a; i <= b; i++) S.width[i] = clamp(S.width[i] + 0.15, 1.2, 3.0);
    }

    // ---------------- phase 2: steep rooty tech ----------------------------
    phRoots: {
      const span = stationRangeForPhase('roots');
      if (!span) break phRoots;
      const [a, b] = span;
      buildPhaseBerms('roots', { margin0: 6, margin1: 8 });
      const seg = Math.round((b - a) / 3);
      addFeature('roots', a + M(8), a + seg, { density: 0.9, height: 0.11 });
      addFeature('chute', a + seg, a + seg * 2, { grade: 0.32 });
      addFeature('roots', a + seg * 2, b - M(10), { density: 0.75, height: 0.09 });
      // Root ridges: transverse, clustered, biggest where it's steepest.
      for (let i = a; i <= b; i++) {
        const cluster = fbm1(microN, i * STATION_DS * 0.14 + 40, 2, 2.0, 0.5) * 0.5 + 0.5;
        const s = Math.sin(i * STATION_DS * 2.6 + microN(i * 0.05) * 3.0);
        S.bumps[i] = Math.max(0, s) * (0.055 + 0.07 * cluster) * smoothstep(0.25, 0.6, cluster);
        S.surface[i] = (S.bumps[i] > 0.045) ? Surface.ROOT : Surface.DIRT;
        S.rough[i] = 0.05 + 0.03 * cluster;
        S.width[i] = clamp(S.width[i] - 0.12, 1.2, 3.0);
      }
      // A committing drop halfway down the tech: the trail is built up onto a lip,
      // then falls away vertically back to the natural grade.
      const dI = a + Math.round((b - a) * 0.55);
      const dh = 1.05 + featRng() * 0.5;
      addFeature('drop', dI - M(3), dI + M(6), { height: dh, landing: 'natural' }, true);
      blendOffset(dI - M(6), dI, (f) => dh * smoothstep(0, 1, f));
      blendOffset(dI + 1, dI + M(1.2), (f) => dh * (1 - smoothstep(0, 1, f)));
      for (let i = dI - M(6); i <= dI + M(6); i++) {
        if (i >= 0 && i < S.n) { S.bumps[i] = 0; S.rough[i] = 0.015; }
      }
    }

    // ---------------- phase 3: flowing bermed section ----------------------
    phFlow: {
      const span = stationRangeForPhase('flow');
      if (!span) break phFlow;
      buildPhaseBerms('flow', { margin0: 10, margin1: 10 });
    }

    // ---------------- phase 4: the jump line -------------------------------
    // Jumps first, so the berm pass can see which stations are already claimed
    // by authored relief and leave them alone.
    buildJumpLine();
    buildPhaseBerms('jumps', { margin0: 8, margin1: 12, skipShaped: true });

    // ---------------- phase 5: exposed rock slab chute ---------------------
    phSlab: {
      const span = stationRangeForPhase('slab');
      if (!span) break phSlab;
      const [a, b] = span;
      buildPhaseBerms('slab', { margin0: 6, margin1: 22 });
      addFeature('chute', a + M(6), b - M(20), { grade: 0.34, exposure: 1.0 });
      for (let i = a; i <= b; i++) {
        S.surface[i] = Surface.ROCK;
        S.rough[i] = 0.022;
        S.rut[i] = 0.012;
        S.crown[i] = 0.012;
        S.width[i] = clamp(S.width[i] - 0.08, 1.2, 3.0);
        // The slab is bedrock: it undulates in long low waves, not dirt chatter.
        S.hOff[i] += fbm1(microN, i * STATION_DS * 0.055 + 120, 3, 2.2, 0.5) * 0.16;
      }
      // A rock roll / drop off the bottom lip of the slab.
      const dI = b - M(24);
      const dh = 1.5 + featRng() * 0.7;
      addFeature('drop', dI - M(8), dI + M(8), { height: dh, surface: 'rock' }, true);
      blendOffset(dI - M(8), dI, (f) => dh * smoothstep(0, 1, f));
      blendOffset(dI + 1, dI + M(1.6), (f) => dh * (1 - smoothstep(0, 1, f)));
    }

    // ---------------- phase 6: creek crossing ------------------------------
    phCreek: {
      const span = stationRangeForPhase('creek');
      if (!span) break phCreek;
      const [a, b] = span;
      buildPhaseBerms('creek', { margin0: 6, margin1: 8 });
      const c = Math.round((a + b) / 2);
      const halfW = M(7);
      addFeature('creekCrossing', c - halfW * 2, c + halfW * 2, {
        width: halfW * 2 * STATION_DS, depth: 0.55,
      });
      // Dish the trail down into the creek bed and back out: a real ford.
      blendOffset(c - halfW * 2, c + halfW * 2, (f) => {
        const s = Math.sin(f * Math.PI);
        return -0.55 * s * s;
      });
      for (let i = c - halfW * 2; i <= c + halfW * 2; i++) {
        if (i < 0 || i >= S.n) continue;
        const d = Math.abs(i - c) / halfW;
        S.surface[i] = d < 1 ? Surface.MUD : Surface.GRAVEL;
        S.rough[i] = 0.05 - 0.02 * d;
        S.width[i] = clamp(S.width[i] + 0.5 * (1 - d), 1.2, 3.0);
        S.rut[i] = 0.02;
      }
      addFeature('rollers', b - M(30), b - M(4), { count: 3, amplitude: 0.22 });
      const r0 = b - M(30), r1 = b - M(4), n = 3, sp = (r1 - r0) / n;
      for (let k = 0; k < n; k++) {
        const cc = Math.round(r0 + sp * (k + 0.5));
        const half = Math.round(sp * 0.45);
        blendOffset(cc - half, cc + half, (f) => 0.22 * (0.5 - 0.5 * Math.cos(f * Math.PI * 2)));
      }
    }

    // ---------------- phase 7: dark loamy forest turns ---------------------
    phLoam: {
      const span = stationRangeForPhase('loam');
      if (!span) break phLoam;
      const [a, b] = span;
      // The tightest corner in the loam becomes a natural wallride up the cut
      // bank — picked up front so the section always has one, and skipped by
      // the berm builder so the two do not both shape it.
      const scan = findCorners(a + M(8), b - M(14), BERM_STYLE.loam.minRadius, BERM_STYLE.loam.minSpacing);
      let wallAt = -1, wallR = Infinity;
      for (const c of scan) {
        if (c.apex < a + M(35)) continue;
        if (S.radius[c.apex] < wallR) { wallR = S.radius[c.apex]; wallAt = c.apex; }
      }
      if (wallR > 26) wallAt = -1;
      let wallDone = false;
      buildPhaseBerms('loam', {
        margin0: 8,
        margin1: 14,
        onCorner: (c) => {
          if (wallDone || c.apex !== wallAt) return true;
          wallDone = true;
          addFeature('wallride', c.i0, c.i1, {
            height: 2.1, radius: S.radius[c.apex], entrySpeed: S.speed[c.apex],
          });
          for (let i = c.i0; i <= c.i1; i++) {
            const f = (i - c.i0) / Math.max(1, c.i1 - c.i0);
            S.wallride[i] = Math.sin(f * Math.PI) * 2.1;
            S.bank[i] = -Math.sign(S.curv[c.apex]) * Math.max(Math.abs(S.bank[i]),
              THREE.MathUtils.degToRad(52) * Math.sin(f * Math.PI));
            S.width[i] = clamp(S.width[i] + 0.5, 1.2, 3.0);
          }
          return false;
        },
      });
      for (let i = a; i <= b; i++) {
        S.surface[i] = Surface.LOAM;
        S.rut[i] = clamp(S.rut[i] + 0.035, 0.02, 0.13);   // loam ruts up deep
        S.rough[i] = 0.03;
      }
    }

    // ---------------- phase 8: rock garden ---------------------------------
    phRocks: {
      const span = stationRangeForPhase('rocks');
      if (!span) break phRocks;
      const [a, b] = span;
      buildPhaseBerms('rocks', { margin0: 6, margin1: 14 });
      addFeature('rockGarden', a + M(6), b - M(12), { size: 0.45, density: 0.8 });
      for (let i = a; i <= b; i++) {
        S.surface[i] = Surface.ROCK;
        const chaos = fbm1(microN, i * STATION_DS * 0.35 + 300, 3, 2.3, 0.55);
        S.rough[i] = 0.085 + chaos * 0.05;
        S.hOff[i] += chaos * 0.14;
        S.crown[i] = 0.01;
        S.rut[i] = 0.015;
      }
      // Two-thirds down, a shaped dirt double for anyone carrying speed through
      // the rocks — the fast line, if you can hold it together to get there.
      const jI = straightestNear(a + Math.round((b - a) * 0.62), M(14), a, b);
      buildJump(jI, {
        type: 'doubles', name: 'rock-garden double',
        targetD: 7.5, minDeg: 12, lipHeight: 0.66, table: 0.15, stepDown: 0,
      });
    }

    // ---------------- phase 9: final bermed sprint -------------------------
    phSprint: {
      const span = stationRangeForPhase('sprint');
      if (!span) break phSprint;
      const [a, b] = span;
      buildPhaseBerms('sprint', { margin0: 10, margin1: 45 });
      // A last hip/booter before the arch — the money shot for the finish camera.
      // Kept well clear of the finish structure (which sits at t = 0.985).
      const jI = straightestNear(b - M(125), M(22), a + M(20), b - M(90));
      buildJump(jI, {
        type: 'jump', name: 'finish booter',
        targetD: 12.0, minDeg: 16, lipHeight: 0.72, table: 0.40, stepDown: 0,
      });
      for (let i = b - M(40); i <= b; i++) S.width[i] = clamp(S.width[i] + 0.3, 1.2, 3.0);
    }
  }

  /** Braking bumps: transverse chatter that builds up before a hard corner. */
  function addBrakingBumps(i0, i1, amp) {
    for (let i = Math.max(0, i0); i <= Math.min(S.n - 1, i1); i++) {
      const f = (i - i0) / Math.max(1, i1 - i0);
      // Wavelength ~1.1 m, growing towards the corner.
      const phase = i * STATION_DS * (2 * Math.PI / 1.1);
      const env = smoothstep(0, 0.35, f) * (0.35 + 0.65 * f);
      S.bumps[i] = Math.max(S.bumps[i], (Math.sin(phase) * 0.5 + 0.5) * amp * env);
      S.rut[i] = Math.max(S.rut[i], 0.05 * env);
    }
  }

  /**
   * Build one lip + landing at station `i` from the design speed there.
   * Writes the lip transition, the deck/gap and the landing transition into hOff,
   * registers the feature, and records the numbers for the CONTRACT-NOTE.
   */
  function buildJump(i, opt) {
    const n = S.n;
    if (i < M(25) || i > n - M(60)) return null;
    const alpha = Math.atan(clamp(S.grade[i], 0.02, 0.30));
    const vApproach = S.speed[i];
    const lipH = opt.lipHeight;
    const step = opt.stepDown || 0;
    const table = opt.table === undefined ? 1 : opt.table;
    // Climbing the lip costs kinetic energy (85% of it — you pump the transition).
    const vTO = Math.sqrt(Math.max(9, vApproach * vApproach - 2 * G * lipH * 0.85));
    // Ground level between the lip and the knuckle: tabletop deck, or a dug gap.
    const gapDepth = 0.35 + step * 0.15;
    const deckY = lipH * 0.97 * table - (1 - table) * gapDepth;
    // Too slow for this to be anything but a case: don't build it.
    if (vTO < 9) return null;
    // Size the lip for the landing distance the jump line calls for, but never
    // below `minDeg` — on a steep grade a 7 deg lip reaches the target distance
    // with no pop at all, and a jump with no pop is a speed bump.
    const theta = lipAngleForDistance(vTO, alpha, lipH, step, opt.targetD,
      opt.minDeg === undefined ? 14 : opt.minDeg, 30);
    const sol = solveJump(vTO, theta, alpha, lipH, step, {
      knuckle: opt.knuckle, touchOver: opt.touchOver, clearance: opt.clearance, deckY,
    });
    // Too slow to clear anything: don't build a jump the rider will case.
    if (sol.D < 3.5) return null;

    // Station geometry. solveJump works in horizontal metres; station spacing runs
    // along the slope, so convert with cos(alpha).
    const cosA = Math.max(0.5, Math.cos(alpha));
    const sx = (metres) => M(metres / cosA);

    const transLen = clamp(lipH / Math.max(0.14, Math.tan(theta)) * 2.1, 2.5, 9);
    const iTransStart = i - sx(transLen);
    const iLip = i;
    const iKnuckle = i + sx(sol.xK);
    const iRise = Math.max(iLip + 2, iKnuckle - sx(1.4));   // knuckle face
    const iRampEnd = i + sx(sol.rampEnd);
    const iTouch = i + sx(sol.xT);
    const iOutEnd = Math.min(n - 1, i + sx(sol.rampEnd + sol.outLen));
    if (iRampEnd >= n - 4) return null;

    // --- lip: circular transition blended into a short straight kicker ------
    blendOffset(iTransStart, iLip, (f) => {
      const arc = (1 - Math.cos(f * theta)) / Math.max(1e-3, 1 - Math.cos(theta));
      const kick = Math.pow(f, 1.7);
      return lipH * (arc * 0.70 + kick * 0.30);
    });

    // --- deck (tabletop) or dug gap (double / road gap) --------------------
    const deckAt = (f) => {
      const s = Math.sin(f * Math.PI);
      return lipH * 0.97 * table * (1 - 0.20 * s) - (1 - table) * gapDepth * s;
    };
    if (iRise > iLip + 1) {
      const span = iRise - (iLip + 1);
      blendOffset(iLip + 1, iRise, (f) => deckAt(f * (span / Math.max(1, span))));
    }

    // --- knuckle face: rise out of the gap onto the landing lip ------------
    const yKnuckle = sol.yK - step;
    const yAtRise = deckAt(1);
    blendOffset(iRise + 1, iKnuckle, (f) => lerp(yAtRise, yKnuckle, smoothstep(0, 1, f)));

    // --- landing transition, straight at the trajectory tangent ------------
    blendOffset(iKnuckle + 1, iRampEnd, (f) =>
      yKnuckle + sol.m * (f * sol.rampRun));

    // --- climb back out of the landing pit onto the trail ------------------
    const yEnd = sol.yEnd - step;
    blendOffset(iRampEnd + 1, iOutEnd, (f) => yEnd * (1 - smoothstep(0, 1, f)));

    // --- tread treatment ---------------------------------------------------
    for (let k = Math.max(0, iTransStart); k <= iOutEnd; k++) {
      S.width[k] = clamp(S.width[k] + 0.35, 1.2, 3.0);
      S.rough[k] = Math.min(S.rough[k], 0.010);
      S.rut[k] = Math.min(S.rut[k], 0.028);
      S.crown[k] = 0.02;
      S.bumps[k] = 0;
      S.surface[k] = Surface.DIRT;
    }
    // A shaped takeoff is never banked.
    for (let k = Math.max(0, iTransStart); k <= iKnuckle; k++) S.bank[k] *= 0.2;
    // Braking bumps build up on the run-in to a big jump.
    addBrakingBumps(iTransStart - M(16), iTransStart - M(3), 0.045);

    const f = addFeature(opt.type, iTransStart, iOutEnd, {
      name: opt.name,
      takeoffSpeed: vTO,
      approachSpeed: vApproach,
      lipHeight: lipH,
      lipAngle: theta,
      launchAngle: sol.psi,
      landingDistance: sol.D,
      gap: sol.xK,
      knuckleHeight: sol.yK,
      landingAngle: sol.delta,
      touchdown: sol.xT,
      touchFrac: sol.touchFrac,
      pitDepth: -sol.yEnd,
      airTime: sol.airTime,
      peakHeight: sol.peakH,
      impactSpeed: sol.vImpact,
      speedWindow: [sol.vMin, sol.vMax],
      stepDown: step,
      iLip, iKnuckle, iTouch, iRampEnd, iOutEnd,
    }, true);
    jumpLog.push({
      name: opt.name,
      takeoff: +vTO.toFixed(2),
      kmh: +(vTO * 3.6).toFixed(1),
      lipH: +lipH.toFixed(2),
      lipDeg: +THREE.MathUtils.radToDeg(theta).toFixed(0),
      landing: +sol.D.toFixed(1),
      gap: +sol.xK.toFixed(1),
      rampDeg: +THREE.MathUtils.radToDeg(sol.delta).toFixed(0),
      pit: +(-sol.yEnd).toFixed(2),
      touchFrac: +sol.touchFrac.toFixed(2),
      air: +sol.airTime.toFixed(2),
      peak: +sol.peakH.toFixed(2),
      window: [+sol.vMin.toFixed(1), +sol.vMax.toFixed(1)],
      impact: +sol.vImpact.toFixed(2),
    });
    return f;
  }

  /** The jump line: tabletop -> double -> double -> gap (A-line) -> step-down. */
  function buildJumpLine() {
    const range = stationRangeForPhase('jumps');
    if (!range) return;
    const [a, b] = range;
    const span = b - a;
    if (span < M(120)) return;

    // Straighten the jump line: jumps need a consistent run-in, so kill the bank
    // and pick the straightest stations available.
    // Rhythm: build up 9 -> 11 -> 13 m, the big A-line gap, then a step-down.
    const slots = [0.14, 0.34, 0.50, 0.68, 0.86];
    const kinds = [
      { type: 'jump', name: 'tabletop', targetD: 11.0, minDeg: 10, lipHeight: 0.92, table: 1.0, stepDown: 0 },
      { type: 'doubles', name: 'double-1', targetD: 13.0, minDeg: 15, clearance: 0.75, lipHeight: 0.98, table: 0.15, stepDown: 0 },
      { type: 'doubles', name: 'double-2', targetD: 15.0, minDeg: 16, clearance: 0.80, lipHeight: 0.98, table: 0.10, stepDown: 0 },
      { type: 'gap', name: 'road gap (A)', targetD: 19.0, minDeg: 18, clearance: 0.85, lipHeight: 1.06, table: 0.0, stepDown: 0 },
      { type: 'stepDown', name: 'step-down', targetD: 16.0, minDeg: 15, clearance: 0.85, lipHeight: 0.72, table: 0.0, stepDown: 2.0 },
    ];
    for (let k = 0; k < slots.length; k++) {
      // Nudge to the straightest station within ±8 m of the nominal slot.
      const best = straightestNear(a + Math.round(span * slots[k]), M(9), a, b);
      buildJump(best, kinds[k]);
    }
  }

  // =========================================================================
  // 5. A/B LINE SPLITS
  // =========================================================================

  function buildSplits() {
    const defs = [
      { phase: 'flow', at: 0.42, len: 58, side: -1, off: 4.6, mainLine: 'A', label: 'inside pump line', kind: 'inside' },
      { phase: 'jumps', at: 0.68, len: 74, side: 1, off: 5.5, mainLine: 'A', label: 'gap ride-around', kind: 'around' },
      { phase: 'slab', at: 0.46, len: 66, side: -1, off: 5.2, mainLine: 'A', label: 'slab or switchback', kind: 'around' },
      { phase: 'loam', at: 0.30, len: 46, side: 1, off: 4.0, mainLine: 'B', label: 'rooty inside cut', kind: 'inside' },
      { phase: 'rocks', at: 0.40, len: 62, side: -1, off: 4.8, mainLine: 'A', label: 'rock chute or bypass', kind: 'around' },
      { phase: 'sprint', at: 0.30, len: 52, side: 1, off: 4.2, mainLine: 'B', label: 'high line', kind: 'inside' },
    ];
    for (const d of defs) {
      const range = stationRangeForPhase(d.phase);
      if (!range) continue;
      const [a, b] = range;
      const c = a + Math.round((b - a) * d.at);
      const half = M(d.len * 0.5);
      const i0 = clamp(c - half, 2, S.n - 3);
      const i1 = clamp(c + half, 2, S.n - 3);
      if (i1 - i0 < M(18)) continue;
      splits.push({
        id: `split-${splits.length + 1}`,
        tStart: i0 / (S.n - 1),
        tEnd: i1 / (S.n - 1),
        i0, i1,
        side: d.side,
        offset: d.off,
        mainLine: d.mainLine,
        label: d.label,
        kind: d.kind,
        // The branch is slower/safer if the main line is the A-line.
        branchIsB: d.mainLine === 'A',
      });
      // The corridor opens up where the lines split.
      for (let i = i0; i <= i1; i++) {
        const f = (i - i0) / (i1 - i0);
        S.width[i] = clamp(S.width[i] + 0.45 * Math.sin(f * Math.PI), 1.2, 3.0);
      }
    }
  }

  /** Lateral offset of a split branch at station i (0 outside the split). */
  function branchOffset(sp, i) {
    if (i < sp.i0 || i > sp.i1) return 0;
    const f = (i - sp.i0) / (sp.i1 - sp.i0);
    const s = Math.sin(f * Math.PI);
    return sp.side * sp.offset * s * s;
  }

  // ---------------------------------------------------------------------------
  // A SPLIT IS ONE SURFACE THAT FORKS, not a second trail dropped beside the
  // first — and this is the one function that says what its height is, for the
  // carve stamps and the tread ribbon alike.
  //
  // WHAT IT REPLACES, measured on the shipped world (seed 20260726, split-1, arc
  // 728-732): the main line is a 3.0 m tread on 43 deg of bank, so its left edge
  // stands 1.20 m above the centreline; the branch at that station sat 1.84 m out
  // on the same side, with a 1.15 m stamp (1.31 m of reach after terrain's
  // carve-width jitter), taking its height from natural ground up to 1.30 m BELOW
  // the centreline. Two stamps covering the same ground declared heights 2.5 m
  // apart — the largest single disagreement in the emitted cloud after the rut
  // stamps — and the accumulator committed something between them, which turns
  // the berm into a ramp and the branch into a ditch. The tread ribbon and the
  // carve also disagreed about the branch height, because each had its own copy
  // of the rule.
  //
  //   q    = |o| - mainHalf              metres outside the main tread edge
  //   edge = surfaceHeight(i, +-mainHalf)  the height BOTH lines share there
  //   w    = smoothstep(0, BRANCH_JOIN, q)
  //   y    = edge + clamp(w * (own - edge), +-BRANCH_SLOPE * q)
  //
  // The clamp is what makes the ground between the lines rideable rather than a
  // step: whatever the B-line's own base wants to be, the connecting ground never
  // departs from the shared edge faster than BRANCH_SLOPE. Which is also what a
  // real fork looks like — you can put a wheel down anywhere between the lines.
  const BRANCH_JOIN = 3.0;      // m of clearance over which the lines separate
  const BRANCH_SLOPE = 0.42;    // tan(23 deg) — the connecting ground's limit
  const BRANCH_HALF = 0.85;     // branches are narrower than the main line
  function branchBaseY(sp, i, terrain) {
    const o = branchOffset(sp, i);
    const sgn = Math.sign(o) || 1;
    const mainHalf = Math.max(0.6, S.width[i] * 0.5);
    const x = S.px[i] + S.rx[i] * o, z = S.pz[i] + S.rz[i] * o;
    const edgeY = surfaceHeight(i, sgn * mainHalf);
    // The B-line rolls the natural ground rather than the main line's shaped
    // height, so it reads as the slower way round; the A-line branch stays on the
    // main line's own banked plane.
    const nat = terrain ? terrain.sampleHeight(x, z) : terrainSampleCache(x, z);
    const ownY = sp.branchIsB
      ? clamp(nat - 0.05, S.py[i] - 1.3, S.py[i] + 0.35)
      : S.py[i] + Math.tan(S.bank[i]) * o;
    const q = Math.abs(o) - mainHalf;
    if (q <= 0) return edgeY;
    const lim = BRANCH_SLOPE * q;
    return edgeY + clamp(smoothstep(0, BRANCH_JOIN, q) * (ownY - edgeY), -lim, lim);
  }

  // =========================================================================
  // 6. CROSS-SECTION + CARVE STAMPS
  // =========================================================================

  /**
   * The worn cross-section of a ridden tread, in metres relative to the
   * centreline surface. v is the normalised lateral position (-1..1 across the
   * tread). Crowned in the middle, two shallow tyre ruts, a windrow of pushed
   * dirt at the edges — never a flat cut.
   */
  function crossProfile(i, v) {
    const av = Math.abs(v);
    const crown = S.crown[i] * (1 - av * av);
    // Two ruts either side of centre. exp() bumps, wandering slightly with s.
    const wob = microN(i * 0.02) * 0.10;
    const r1 = Math.exp(-Math.pow((v - (0.30 + wob)) / 0.20, 2));
    const r2 = Math.exp(-Math.pow((v + (0.30 - wob)) / 0.20, 2));
    const ruts = -S.rut[i] * Math.max(r1, r2);
    // Edge windrow: loose dirt pushed out by years of tyres.
    const edge = smoothstep(0.72, 1.0, av) * (0.045 + 0.05 * S.rut[i] * 8) *
      (0.6 + 0.4 * (edgeN(i * 0.03 + av * 7) * 0.5 + 0.5));
    return crown + ruts + edge;
  }

  /**
   * The RENDER cross-section: the same shape as crossProfile(), band-limited so
   * that nothing below ~1.0 m of wavelength survives into vertex positions.
   *
   * Two changes, both forced by the sampling rate of the tread ribbon (0.8 m
   * longitudinally, ~0.27 m laterally):
   *   - the rut Gaussian widens from sigma 0.20 to 0.34 of the half-width
   *     (~0.26 m -> ~0.45 m of actual channel), which puts >3 lateral samples
   *     across it instead of ~1.4. Depth is scaled by 0.75 so the widened
   *     channel removes the same volume of dirt and still reads as a rut.
   *   - the windrow's `edgeN(i*0.03 + av*7)` term is a *per-vertex* random: av*7
   *     advances the noise lattice by ~1.6 units between adjacent lateral
   *     samples, i.e. white noise on the ribbon edge. It becomes a function of
   *     station only, so the windrow undulates along the trail as it should
   *     instead of sawtoothing across it.
   * The relief that leaves here is not lost — it is in the tread normal/height
   * map (see buildTreadMaps) and in the carve stamps that physics samples.
   */
  function crossProfileRender(i, v) {
    const av = Math.abs(v);
    const crown = S.crown[i] * (1 - av * av);
    const wob = microN(i * 0.02) * 0.10;
    const SIG = 0.34;
    const r1 = Math.exp(-Math.pow((v - (0.30 + wob)) / SIG, 2));
    const r2 = Math.exp(-Math.pow((v + (0.30 - wob)) / SIG, 2));
    const ruts = -S.rut[i] * 0.75 * Math.max(r1, r2);
    const edge = smoothstep(0.72, 1.0, av) * (0.045 + 0.05 * S.rut[i] * 8) *
      (0.6 + 0.4 * (edgeN(i * 0.03) * 0.5 + 0.5));
    return crown + ruts + edge;
  }

  /**
   * Micro-detail noise: chatter, embedded rock, transverse ridges.
   *
   * BAND-LIMITED TO WHAT A 0.37 m WHEEL CAN ACTUALLY RIDE, and the wavelength is
   * not a taste call — two things in this file already fix it and the emission
   * disagreed with both.
   *
   * (1) techSpeedCap() bounds the design speed with a chatter model whose
   *     wavelength is ROUGH_LAMBDA = 1.6 m: "the spacing the root ridges and
   *     braking-bump trains are authored at". The fbm was authored at
   *     s * 1.9, and makeNoise1D's lattice spacing is 1, so its fundamental was
   *     1/1.9 = 0.53 m and its second octave (lacunarity 2.4) was 0.22 m. The
   *     speed ceiling was therefore calibrated against relief the trail does not
   *     emit, and the relief the trail does emit had no speed ceiling at all.
   *
   * (2) surfaceHeightRender() already refuses to put anything under ~1 m of
   *     wavelength into vertex positions, on the grounds that the ribbon cannot
   *     reconstruct it. The carve kept it, with the note that "the wheel keeps
   *     feeling the chatter the eye is no longer being shown". A 0.37 m wheel
   *     does not feel 0.22 m chatter; it bridges everything under ~2R = 0.74 m
   *     and is thrown by whatever is sharper than its own radius. Dilating the
   *     committed profile by a wheel disc and asking where v^2 kappa/g >= 1 at
   *     design speed, an 0.22 m ripple of ONE MILLIMETRE clears that threshold
   *     by two orders of magnitude — so any amplitude at all reads as a launch,
   *     which is exactly what 33% of stations were measuring.
   *
   * So the fundamental is placed at ROUGH_LAMBDA * 2.4 = 3.84 m and the second
   * octave lands on ROUGH_LAMBDA itself: the ceiling is now calibrated against
   * the relief that exists, both octaves are longer than the wheel, and the
   * amplitude (S.rough) is untouched — the trail is exactly as rough, at a
   * wavelength a bicycle can ride instead of one it can only be launched by.
   *
   * The lateral term is cut from v*3.1 to v*0.85 for the same reason in the
   * other axis: v is normalised, so 3.1 advanced the lattice by 6.2 units across
   * the tread — six independent draws over 2.5 m, i.e. white noise across the
   * line, which is also what put a 0.4 m-wavelength ripple into the cross-slope
   * and pulled the low-bank stations' realised bank down to 0.42x of declared.
   * Trail relief is anisotropic: it runs with the line, not across it.
   */
  function microDetail(i, v) {
    const s = i * STATION_DS;
    let h = fbm1(microN, s / (ROUGH_LAMBDA * 2.4) + v * 0.85, 2, 2.4, 0.5) * S.rough[i];
    // Transverse ridges (roots / braking bumps) run across the trail, so they are
    // a function of s only, with a little lateral wander.
    if (S.bumps[i] > 0.001) {
      h += S.bumps[i] * (0.8 + 0.2 * Math.cos(v * 2.6 + microN(i * 0.01) * 3));
    }
    return h;
  }

  // Fraction of the half-width over which the cross-slope is held at its full
  // value. Outside it the slope eases to zero by the tread edge.
  const BANK_FULL = 0.72;

  /**
   * The banked cross-section's rise at lateral offset `o`, with the cross-slope
   * EASED TO ZERO at the tread edge instead of running off it at full tilt.
   *
   * Why: outside the tread the surface is either the run-off shelf (falling at
   * SHELF_SLOPE) or natural ground. A tread that is still rising at tan(bank)
   * when it meets a shelf that is falling at 0.20 has a slope discontinuity of
   * tan(bank) + 0.20 — a crease at the tread edge, 0.78 of slope at a 30 deg
   * bank. The rider spends a great deal of time within a few tens of
   * centimetres of that edge (the autopilot's lateral error reaches the full
   * half-width on this course), and crossing a crease at 10 m/s throws the bike
   * off the ground: measured at 76 m, ground curvature along the bike's path of
   * -0.62 /m, six times the threshold at which the wheels leave, followed by
   * 6.7 g and 1.8 m of air.
   *
   * Easing the slope over the outer 28% of the half-width turns the crease into
   * a rolled lip, which is also what a built berm actually looks like. The
   * centre 72% of the tread — where the rider rides and where the lean identity
   * is evaluated — keeps the cross-slope the speed solve was built for, so the
   * identity is unaffected.
   */
  function bankRise(i, o, half) {
    const t = Math.tan(S.bank[i]);
    const v = o / half;
    const a = Math.abs(v);
    if (a <= BANK_FULL) return t * o;
    const W = 1 - BANK_FULL;
    const x = Math.min(1, (a - BANK_FULL) / W);
    // Integral of (1 - smoothstep(x)) dx = x - x^3 + x^4/2, so the rise is C1
    // at both ends and the slope is exactly zero at the tread edge.
    const extra = W * (x - x * x * x + 0.5 * x * x * x * x);
    return Math.sign(v) * t * half * (BANK_FULL + extra);
  }

  function treadHeight(i, o) {
    // o = lateral offset in metres, + = rider's right
    const half = Math.max(0.6, S.width[i] * 0.5);
    const v = clamp(o / half, -1.6, 1.6);
    let y = S.py[i] + bankRise(i, o, half) + crossProfile(i, v) + microDetail(i, v);
    if (S.wallride[i] > 0.01) {
      // The wall kicks up on the outside of the turn: flat tread out to 0.8 of the
      // half-width, then a quadratic ramp to full wall height at the outer edge.
      const outSign = -Math.sign(S.bank[i]) || 1;
      const t = clamp01(((o * outSign) / half - 0.80) / 0.62);
      y += S.wallride[i] * t * t;
    }
    return y;
  }

  /**
   * The run-off shelf / catch berm, as a height RELATIVE to the tread edge on
   * that side. `null` where the station has no shelf on that side, or where `o`
   * is inside the tread, or past the crest (the back of the berm is authored by
   * the stamp emitter, which has the terrain to run down to).
   *
   * See applyExposureSafety() for what sets the shelf and catch arrays.
   */
  function shelfRel(i, o) {
    const run = o < 0 ? S.shelfL[i] : S.shelfR[i];
    if (run < CATCH_MIN_Q * 0.8) return null;
    const half = Math.max(0.6, S.width[i] * 0.5);
    const q = (o < 0 ? -o : o) - half;
    if (q <= 0 || q > run + CATCH_PLATEAU) return null;
    const h = o < 0 ? S.catchL[i] : S.catchR[i];
    const ramp = rampLenFor(run, h);
    if (q >= run) return -SHELF_SLOPE * run + h;          // flat crest
    return -SHELF_SLOPE * q + h * smoothstep(run - ramp, run, q);
  }

  /**
   * How much lateral run the inner face of a catch berm of height `h` gets, on
   * a shelf of length `run`. See the CATCH_FACE note: the face is a ramp the
   * rider hits, so it is sized from the crest height at a rideable slope,
   * bounded by the room actually available.
   */
  function rampLenFor(run, h) {
    return clamp(h / CATCH_FACE, CATCH_RAMP, Math.min(CATCH_RAMP_MAX, Math.max(CATCH_RAMP, run * 0.85)));
  }

  /**
   * The designed ground surface at lateral offset `o`.
   *
   * CRITICAL — this is the single cross-section the whole carve agrees on.
   * terrain.applyCarve() reconstructs the surface as a distance-weighted MEAN of
   * every stamp reaching a cell, so a feature that only some stamps know about
   * gets averaged away by the ones that do not. An earlier cut of the catch berm
   * emitted it as its own stamps on top of an unchanged tread cross-section, and
   * the tread's own lateral stamps out-voted it 1.5:1 — an authored 0.85 m berm
   * arrived on the heightfield as 0.0 m, i.e. painted and not built. Putting it
   * here instead is the same discipline the file's opening CONTRACT-NOTE already
   * applies to bank and crown: every stamp at every offset derives its target
   * from one function, so they cannot disagree and cannot cancel.
   */
  function surfaceHeight(i, o) {
    const rel = shelfRel(i, o);
    if (rel === null) return treadHeight(i, o);
    const half = Math.max(0.6, S.width[i] * 0.5);
    return treadHeight(i, o < 0 ? -half : half) + rel;
  }

  /**
   * surfaceHeight() for the RENDER ribbon: identical macro shape (grade, bank,
   * crown, wallride) with all sub-metre relief removed. microDetail() is a
   * 0.5 m-wavelength fbm of amplitude up to +/-6 cm and a 1.1 m braking-bump
   * train — both far below the ribbon's 0.8 m Nyquist limit, and both now
   * carried by the tread normal map instead. Collision is unaffected: the carve
   * stamps still call surfaceHeight(), so the wheel keeps feeling the chatter
   * the eye is no longer being shown as facets.
   */
  function surfaceHeightRender(i, o) {
    const half = Math.max(0.6, S.width[i] * 0.5);
    const v = clamp(o / half, -1.6, 1.6);
    let y = S.py[i] + bankRise(i, o, half) + crossProfileRender(i, v);
    if (S.wallride[i] > 0.01) {
      const outSign = -Math.sign(S.bank[i]) || 1;
      const t = clamp01(((o * outSign) / half - 0.80) / 0.62);
      y += S.wallride[i] * t * t;
    }
    return y;
  }

  // =========================================================================
  // 6. EXPOSURE SAFETY — detect and mitigate unmarked traps
  // =========================================================================
  //
  // THE DEFECT THIS EXISTS FOR
  //
  // The route-finder picks a line down the fall line and the station solver
  // benches it into whatever ground that line crosses. Neither of them looks
  // sideways. So the generator will happily lay a 2.9 m tread along the crest of
  // a rib with the mountain falling away 2.7 m in 1.1 m of travel two metres off
  // the racing line, at the point of maximum grade — and it did, at 85 m, where
  // every one of a 21-cell autopilot sweep died with an `impact` crash.
  //
  // That is not a hard feature. A hard feature is one you can see, judge and
  // fail at proportionately. A 2.7 m unprotected drop 2 m from the fall line at
  // 53% grade is a trap: the ordinary error of running a metre wide is fatal,
  // and there is nothing on the trail that tells you so beforehand.
  //
  // Real courses do not leave that. On an exposed traverse a builder does one of
  // four things, and usually several: bench the tread further into the hillside,
  // build a catch berm or an armoured outside edge, leave graded run-off outside
  // the tread, or ease the gradient so the rider is not being accelerated into
  // the exposure while trying to turn away from it. This pass does all four,
  // driven by a measurement rather than by hand-placed exceptions.
  //
  // THE MEASUREMENT — "unsupported fall"
  //
  // At each station, on each side, we probe the natural ground at EXPO_PROBE
  // metres beyond the tread edge and take
  //
  //     expo = max over d of ( fall(d) - EXPO_REPOSE * d ),  clamped at 0
  //
  // where fall(d) is how far the ground has dropped below the tread edge. The
  // EXPO_REPOSE term is what makes this a defect detector rather than a slope
  // detector: an ordinary sidehill, however steep the trail's own gradient, sits
  // at or below the angle of repose and scores zero. Only ground that falls away
  // faster than a hillside can stand — i.e. an edge — scores.
  //
  // WHY THE TERRAIN'S OWN BATTER DOES NOT ALREADY FIX IT
  //
  // terrain.applyCarve() does taper the cut and fill outside the tread, but its
  // fill side is deliberately capped (FILL_MAX 0.7 m, refusing entirely by 1.4 m)
  // because a bench across a fall-away is built full-cut, not by throwing an
  // embankment over the void. That is the right rule for a batter. It means the
  // batter can do nothing at all about a 2.7 m drop, and the trail is the only
  // module that can, because the trail is the thing that chose to go there.
  //
  // WHAT IT DOES NOT DO
  //
  // It does not flatten the run. The grade cap is a cap on the RATE the gradient
  // changes, not on the gradient — a 60% pitch is untouched as long as it is
  // entered over more than a couple of bike lengths. Shaped features (jumps,
  // drops, berms, wallrides) and 15 m either side of them are excluded outright,
  // so the jump line's ballistics are bit-identical. And the run-off it builds is
  // deliberately unpleasant: it falls away at 20%, and it is painted GRAVEL, the
  // second-loosest surface on the course (mu 0.74 against dirt's 1.00). Running
  // wide costs you the corner and a lot of speed. It just does not kill you.
  // =========================================================================

  /**
   * Build-time safety audit; also exposed as `trail.safety`. All "before"
   * numbers describe the trail as the route-finder and the feature pass left it,
   * i.e. the defect inventory. Whether the mitigation worked is a question for a
   * measurement of the CARVED terrain, not for this object.
   */
  const safety = {
    exposedStations: 0, exposedMetres: 0, stretches: 0,   // the audit
    worstExpoBefore: 0,
    // response A: `rampViol*` count stations over their cap; `rampWorst*` is the
    // single worst rate. The max barely moves because the cut-only rule declines
    // to fix a violation whose only fix is fill — the counts are the honest read.
    rampCapped: 0, rampViolBefore: 0, rampViolAfter: 0,
    rampWorstBefore: 0, rampWorstAfter: 0,
    benched: 0, benchMax: 0,                              // response B
    widened: 0,                                           // response C
    catchBerms: 0, shelfMetres: 0, unprotectable: 0,      // response D
    tapedMetres: 0,                                       // response E
    // ---- two-leg conflicts (auditLegConflicts) ----
    // `legBefore` is the count on the stations as the route, feature and safety
    // passes left them; `legAfter` is the count after the pinch. Both are on the
    // FINAL line, which is the only place the property can honestly be claimed.
    legBefore: null, legAfter: null,
    // ---- plan radius discipline (openTightCorners) ----
    planTightBefore: 0, planTightAfter: 0,
    planWorstBefore: 1e9, planWorstAfter: 1e9, planShiftMax: 0,
    planWorstAtBefore: 0, planWorstAtAfter: 0, planLocked: 0,
    // ---- crest curvature (capCrestCurvature) ----
    crestViolBefore: 0, crestViolAfter: 0,
    crestWorstBefore: 0, crestWorstAfter: 0,
    crestWorstAtBefore: 0, crestWorstAtAfter: 0,
    // Crest violations on the FINAL profile after each later pass — see
    // countCrestViol(). crestViolAfter is the cap's own exit, not the shipped
    // profile, and the two are not the same number.
    crestStage: { afterCap: 0, afterFeatures: 0, afterRecap: 0, afterSafety: 0 },
  };

  /**
   * Metres of unsupported fall outside one tread edge. See the note above.
   * `half` is the tread half-width; the reference height is the DESIGN tread
   * edge (where the rider actually is), the probe heights are natural ground
   * (applyCarve has not run yet, and outside the tread it barely will).
   */
  function exposureAt(terrain, i, sgn, half) {
    const ey = surfaceHeight(i, sgn * half);
    const bx = S.px[i] + S.rx[i] * sgn * half;
    const bz = S.pz[i] + S.rz[i] * sgn * half;
    const dx = S.rx[i] * sgn, dz = S.rz[i] * sgn;
    let worst = 0;
    for (let k = 0; k < EXPO_PROBE.length; k++) {
      const d = EXPO_PROBE[k];
      const e = (ey - terrainH(terrain, bx + dx * d, bz + dz * d)) - EXPO_REPOSE * d;
      if (e > worst) worst = e;
    }
    return worst;
  }

  /**
   * Stations whose height profile is AUTHORED and whose ballistics must come out
   * bit-identical: jump lips and landings, the committing drops, the wallride.
   *
   * NOTE what is deliberately NOT here. An earlier cut of this also protected
   * any station banked past 12 deg, on the theory that a bank means a built
   * berm. On this course that mask swallowed the entire 68-110 m switchback —
   * i.e. the exact defect this pass exists to fix — because solveSpeeds() banks
   * every corner tighter than a 220 m radius, built or not. Bank is a property
   * of the speed solve, not evidence of authored geometry. Berms are handled the
   * other way round: their flanks get the catch treatment like anything else,
   * because a bermed corner with the mountain missing off the outside is still a
   * trap, and a catch berm outboard of a race berm is standard course furniture.
   *
   * @param {string[]} types   feature types to protect.
   * @param {number}   guardM  metres of margin dilated around each feature.
   */
  function buildAuthoredMask(types, guardM) {
    const n = S.n;
    const m = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const fi = S.feat[i];
      const t = fi >= 0 ? features[fi].type : null;
      if ((t && types.indexOf(t) >= 0) || S.wallride[i] > 0.05) m[i] = 1;
    }
    const g = M(guardM);
    const out = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      if (!m[i]) continue;
      for (let j = Math.max(0, i - g); j <= Math.min(n - 1, i + g); j++) out[j] = 1;
    }
    // No endpoint pin. startTransform and the checkpoints are all derived from
    // S.px/py/pz AFTER this pass runs, so benching the first few metres moves the
    // gate with the trail rather than away from it — and on seed 20260726 the
    // start gate sits on a spur with a 6 m drop 1.5 m to the right of it, which
    // is the last place to refuse the rule on principle.
    return out;
  }

  /**
   * Cap how fast the gradient may ramp.
   *
   * Enforced as a constraint on the SECOND derivative of the vertical profile,
   * relaxed iteratively with a wide stencil (+/- half the RAMP_WIN window)
   * because that is the wavelength the constraint is written at — a one-station
   * Laplacian only attacks 0.8 m wobble and would do nothing to a 7 m ramp.
   *
   * The correction is written into S.hOff, not S.by, so the natural bench stays
   * on record; it is clamped against the excavation limits every pass; it is
   * zero on protected stations, which therefore act as fixed boundary
   * conditions; and the endpoints never move, so the run's total descent is
   * unchanged to the last millimetre.
   *
   * CUT-ONLY ON EXPOSED GROUND. Easing a pitch has two halves: cutting the crest
   * where the gradient is picking up, and filling the runout where it eases off.
   * The fill half RAISES the tread — and on a rib, raising the tread by a metre
   * makes the drop off the side a metre deeper, which is the opposite of the
   * point. Measured on seed 20260726 the first draft of this function lifted the
   * 94-100 m section 1.0-1.3 m and gave back most of what the catch berm bought.
   * So on any exposed station the relaxation may only ever lower the line. It
   * fixes fewer violations that way; it never trades one defect for another.
   */
  function capGradeRamp(vertMask, exposedMask) {
    const n = S.n, ds = S.ds;
    const m = Math.max(2, Math.round(RAMP_WIN / (2 * ds)));
    const y = new Float64Array(n);
    for (let i = 0; i < n; i++) y[i] = S.by[i] + S.hOff[i];
    const y0 = Float64Array.from(y);
    const viol = new Float32Array(n);
    const wgt = new Float32Array(n);
    const touched = new Uint8Array(n);

    const rateAt = (arr, i) => {
      const gA = (arr[i - m] - arr[i]) / (m * ds);
      const gB = (arr[i] - arr[i + m]) / (m * ds);
      return Math.abs(gB - gA) / (m * ds);
    };
    const capAt = (i) => (exposedMask[i] || S.width[i] < RAMP_NARROW ? RAMP_EXPOSED : RAMP_OPEN);

    for (let i = m; i < n - m; i++) {
      if (vertMask[i]) continue;
      const r = rateAt(y0, i);
      if (r > safety.rampWorstBefore) safety.rampWorstBefore = r;
      if (r > capAt(i)) { touched[i] = 1; safety.rampViolBefore++; }
    }

    const RELAX = 0.35;
    for (let pass = 0; pass < 80; pass++) {
      viol.fill(0);
      let any = false;
      for (let i = m; i < n - m; i++) {
        if (vertMask[i]) continue;
        const cap = capAt(i);
        const r = rateAt(y, i);
        if (r <= cap) continue;
        any = true;
        viol[i] = clamp01((r - cap) / cap) * 0.65 + 0.35;
        touched[i] = 1;
      }
      if (!any) break;
      // Cone dilation: spread each violation over the window it was measured on,
      // so the correction is a smooth arc rather than a spike at the worst cell.
      wgt.fill(0);
      for (let i = 0; i < n; i++) {
        const v = viol[i];
        if (v <= 0) continue;
        for (let j = Math.max(0, i - m); j <= Math.min(n - 1, i + m); j++) {
          const w = v * (1 - Math.abs(j - i) / (m + 1));
          if (w > wgt[j]) wgt[j] = w;
        }
      }
      for (let i = m; i < n - m; i++) {
        const w = wgt[i];
        if (w <= 0 || vertMask[i]) continue;
        y[i] += w * RELAX * (0.5 * (y[i - m] + y[i + m]) - y[i]);
        if (exposedMask[i] && y[i] > y0[i]) y[i] = y0[i];   // cut-only: see above
        const lo = S.rawS[i] - RAMP_CUT_MAX, hi = S.rawS[i] + GRADE_FILL_MAX;
        if (y[i] < lo) y[i] = lo; else if (y[i] > hi) y[i] = hi;
      }
    }

    // Descent guard. Smoothing an already-descending profile very rarely makes
    // it climb, but "very rarely" is not "never" and CONTRACT §4 is absolute
    // about it. Only ever lowers, and only stations the relaxation touched, so
    // it cannot reach a feature.
    // The excavation limit wins the tie, exactly as it does in enforceDescent():
    // without it this guard cascades — station i pulled down by i-1, i+1 pulled
    // down by i — and on seed 12345 it dug the worst cut on the course from 8.0 m
    // to 17.2 m. A trail that goes over a small rise is a far smaller defect than
    // a trail in a seventeen-metre trench.
    for (let i = 1; i < n; i++) {
      if (!touched[i] || vertMask[i]) continue;
      const cap = y[i - 1] + 0.035 * ds;
      if (y[i] > cap) y[i] = Math.max(cap, S.rawS[i] - RAMP_CUT_MAX);
    }

    for (let i = 0; i < n; i++) {
      if (touched[i]) safety.rampCapped++;
      S.hOff[i] += y[i] - y0[i];
      S.py[i] = S.by[i] + S.hOff[i];
    }
    for (let i = m; i < n - m; i++) {
      if (vertMask[i]) continue;
      const r = rateAt(y, i);
      if (r > safety.rampWorstAfter) safety.rampWorstAfter = r;
      if (r > capAt(i)) safety.rampViolAfter++;
    }
  }

  // -------------------------------------------------------------------------
  // CREST CURVATURE — the bound that stops the tread launching the rider.
  //
  // THE MEASUREMENT THAT MOTIVATES IT. Every cell of the 21-cell autopilot
  // sweep died between 81 and 101 m, and every single one was AIRBORNE at the
  // moment of the crash. The trace shows why: the Start Chute ran 0.4% of
  // gradient at 60 m and 49% at 84 m, so the profile rolls over a crest whose
  // vertical radius is about 12 m, and a bike arriving at the design speed of
  // 12.7 m/s needs v^2/R = 13.4 m/s^2 of centripetal acceleration to stay on
  // it against the 9.81 that gravity provides. It cannot. It leaves the ground,
  // lands 15-40 m off line, and goes over the edge.
  //
  // THE RULE. Over a crest of vertical radius R the wheels keep contact only
  // while v^2 <= g*R. Requiring a margin — the rider should still have real
  // weight on the tyres, not be at the point of flight — gives
  //
  //     R  >=  CREST_SAFETY * v^2 / g          i.e.  kappa <= g / (CREST_SAFETY * v^2)
  //
  // with kappa the convex vertical curvature d(grade)/ds. At CREST_SAFETY 1.45
  // the wheels still carry ~31% of static load over the worst crest on the
  // course, which is enough tyre to steer with.
  //
  // WHAT IT IS NOT. It is not a cap on gradient. A 46% pitch is untouched as
  // long as it is ENTERED over enough distance, which is exactly the
  // distinction between a steep trail and a jump. Concave compressions are
  // bounded separately and much more loosely: a compression pushes the rider
  // INTO the ground, so it costs suspension travel rather than contact, and
  // the fork has 170 mm to spend.
  //
  // The correction is cut-biased for the same reason everything else here is:
  // lowering a crest is an excavation the mountain always grants, whereas
  // raising a compression is an embankment terrain.applyCarve() only builds to
  // about 0.7 m. Authored relief — jump lips, landings, the committing drops —
  // is excluded outright, so the ballistics come through bit-identical.
  // -------------------------------------------------------------------------
  // CREST_SAFETY / CREST_KAPPA_MIN / CREST_KAPPA_MAX are at module scope with the
  // corridor constants: the route-finder and this solver have to promise the same
  // number, or the route commits to profiles the solver then cannot deliver.
  const CREST_WIN = 4.0;          // m — the span a launch happens over
  const CREST_RELAX_WIN = 11.0;   // m — the span the correction is spread over
  const COMPRESS_KAPPA_MAX = 0.10;

  /**
   * Crest violations on the CURRENT profile, at the current speed profile.
   *
   * This exists because `crestViolAfter` is measured at capCrestCurvature's own
   * exit, and four passes run after it — buildFeatures(), the final re-solve,
   * capGradeRamp() and benchLine() — every one of which moves S.py or S.hOff.
   * Reporting the cap's own exit and calling the profile safe is exactly the
   * "verified the change, not the consumers" failure: measured on seed 20260726
   * the cap left 144 violations and the FINAL design profile carried 244.
   * safety.crestStage records the count after each pass so the difference is
   * visible instead of inferred.
   *
   * `mask` MUST be passed once features exist. A jump lip is convex past the
   * crest cap BY CONSTRUCTION — that is what a jump is — so an unmasked count
   * reports the jump line as 155 defects. The first cut of this did exactly
   * that and made buildFeatures() look like it was adding 180 launch points
   * when 155 of them were the features themselves.
   */
  function countCrestViol(mask) {
    if (!S) return 0;
    const n = S.n, ds = S.ds;
    const m = Math.max(2, Math.round(CREST_WIN / (2 * ds)));
    let c = 0;
    for (let i = m; i < n - m; i++) {
      if (mask && mask[i]) continue;
      const v = Math.max(4, S.speed[i]);
      const cap = clamp(G / (CREST_SAFETY * v * v), CREST_KAPPA_MIN, CREST_KAPPA_MAX);
      const gA = (S.py[i - m] - S.py[i]) / (m * ds);
      const gB = (S.py[i] - S.py[i + m]) / (m * ds);
      if ((gB - gA) / (m * ds) > cap) c++;
    }
    return c;
  }

  /**
   * Relax the vertical profile until every crest is flat enough to hold the
   * wheels down at that station's design speed. Writes into S.hOff, leaves
   * S.by alone, and never moves a station carrying authored relief.
   */
  function capCrestCurvature(vertMask) {
    const n = S.n, ds = S.ds;
    const m = Math.max(2, Math.round(CREST_WIN / (2 * ds)));
    const mR = Math.max(m + 1, Math.round(CREST_RELAX_WIN / (2 * ds)));
    const y = new Float64Array(n);
    for (let i = 0; i < n; i++) y[i] = S.by[i] + S.hOff[i];
    const y0 = Float64Array.from(y);
    const wgt = new Float32Array(n);
    const capOf = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const v = Math.max(4, S.speed[i]);
      capOf[i] = clamp(G / (CREST_SAFETY * v * v), CREST_KAPPA_MIN, CREST_KAPPA_MAX);
    }
    // Positive = convex (getting steeper down the hill), i.e. a crest.
    const kappaAt = (arr, i) => {
      const gA = (arr[i - m] - arr[i]) / (m * ds);
      const gB = (arr[i] - arr[i + m]) / (m * ds);
      return (gB - gA) / (m * ds);
    };

    for (let i = m; i < n - m; i++) {
      if (vertMask[i]) continue;
      const k = kappaAt(y0, i);
      if (k > safety.crestWorstBefore) { safety.crestWorstBefore = k; safety.crestWorstAtBefore = i * ds; }
      if (k > capOf[i]) safety.crestViolBefore++;
    }

    const RELAX = 0.40;
    for (let pass = 0; pass < 300; pass++) {
      wgt.fill(0);
      let any = false;
      for (let i = m; i < n - m; i++) {
        if (vertMask[i]) continue;
        const k = kappaAt(y, i);
        const over = k > capOf[i] ? (k - capOf[i]) / capOf[i]
          : (-k > COMPRESS_KAPPA_MAX ? (-k - COMPRESS_KAPPA_MAX) / COMPRESS_KAPPA_MAX : 0);
        if (over <= 0) continue;
        any = true;
        const v = clamp01(over) * 0.65 + 0.35;
        for (let j = Math.max(0, i - mR); j <= Math.min(n - 1, i + mR); j++) {
          const w = v * (1 - Math.abs(j - i) / (mR + 1));
          if (w > wgt[j]) wgt[j] = w;
        }
      }
      if (!any) break;
      for (let i = mR; i < n - mR; i++) {
        const w = wgt[i];
        if (w <= 0 || vertMask[i]) continue;
        // The RELAXATION stencil is wider than the MEASUREMENT stencil, and
        // they are separate on purpose. The launch happens over ~4 m — that is
        // the span the wheels and the fork see, so that is where the curvature
        // must be measured. But the geometry that produces it is a 10-15 m
        // rollover, and a Laplacian over ±2 m attacks 2 m wobble and leaves a
        // 12 m crest almost untouched: measured with the two stencils equal,
        // the pass took the worst crest curvature only 0.66 -> 0.25 /m against
        // a 0.042 cap, i.e. it barely moved the defect it had correctly found.
        const step = w * RELAX * (0.5 * (y[i - mR] + y[i + mR]) - y[i]);
        y[i] += step;
        // Cut freely, fill only as far as the terrain will actually build.
        const lo = S.rawS[i] - RAMP_CUT_MAX, hi = S.rawS[i] + GRADE_FILL_MAX;
        if (y[i] < lo) y[i] = lo; else if (y[i] > hi) y[i] = hi;
      }
    }

    // Descent guard — the same one capGradeRamp uses, and for the same reason:
    // smoothing a descending profile very rarely makes it climb, and CONTRACT
    // §4 is absolute about it.
    for (let i = 1; i < n; i++) {
      if (vertMask[i]) continue;
      const cap = y[i - 1] + 0.035 * ds;
      if (y[i] > cap) y[i] = Math.max(cap, S.rawS[i] - RAMP_CUT_MAX);
    }

    for (let i = 0; i < n; i++) {
      S.hOff[i] += y[i] - y0[i];
      S.py[i] = S.by[i] + S.hOff[i];
    }
    for (let i = m; i < n - m; i++) {
      if (vertMask[i]) continue;
      const k = kappaAt(y, i);
      if (k > safety.crestWorstAfter) { safety.crestWorstAfter = k; safety.crestWorstAtAfter = i * ds; }
      if (k > capOf[i]) safety.crestViolAfter++;
    }
  }

  /** Recompute tangents, right vectors and grade from the current px/py/pz. */
  function refreshBasis() {
    const n = S.n;
    for (let i = 0; i < n; i++) {
      const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
      const dx = S.px[b] - S.px[a], dz = S.pz[b] - S.pz[a], dy = S.py[b] - S.py[a];
      const l = Math.max(1e-5, Math.hypot(dx, dy, dz));
      S.tx[i] = dx / l; S.ty[i] = dy / l; S.tz[i] = dz / l;
      const hl = Math.max(1e-5, Math.hypot(S.tx[i], S.tz[i]));
      S.rx[i] = -S.tz[i] / hl; S.rz[i] = S.tx[i] / hl;
      S.grade[i] = -S.ty[i];
    }
  }

  /**
   * Bench the line toward the safe side.
   *
   * This is the cheapest and most honest of the four responses — it is what a
   * builder does first, and it costs nothing but excavation. Only applied where
   * exactly one side is exposed (on a rib with both sides gone there is nowhere
   * to go), never onto ground that would need more than BENCH_CUT_EXTRA of extra
   * cut, and eased in over BENCH_BLEND metres so the racing line stays smooth.
   * Curvature and bank are deliberately NOT re-solved afterwards: the shift is
   * sub-metre and smoothed over 12 m, so the berms stay built for the corners
   * they were solved for.
   */
  function benchLine(terrain, lineMask) {
    const n = S.n;
    const want = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      if (lineMask[i]) continue;
      const half = Math.max(0.6, S.width[i] * 0.5);
      const eL = exposureAt(terrain, i, -1, half);
      const eR = exposureAt(terrain, i, 1, half);
      const bad = Math.max(eL, eR);
      if (bad < EXPO_TRIGGER) continue;
      if (Math.min(eL, eR) > EXPO_TRIGGER * 0.8) continue;   // a rib: nowhere to go
      const sgn = eR > eL ? -1 : 1;                          // toward the SAFE side
      const need = clamp01((bad - EXPO_TRIGGER) / (EXPO_FULL - EXPO_TRIGGER));
      const y = S.py[i];
      const cut0 = Math.max(0, terrainH(terrain, S.px[i], S.pz[i]) - y);
      const budget = Math.min(BENCH_CUT_TOTAL, cut0 + BENCH_CUT_EXTRA);
      let s = need * BENCH_SHIFT_MAX;
      while (s > 0.10) {
        const cut = terrainH(terrain, S.px[i] + S.rx[i] * sgn * s,
          S.pz[i] + S.rz[i] * sgn * s) - y;
        if (cut <= budget) break;
        s -= 0.15;
      }
      want[i] = s > 0.10 ? sgn * s : 0;
    }
    const rad = M(BENCH_BLEND);
    const sm = gaussianSmooth(want, rad, rad * 0.42);
    // Re-check affordability AFTER the blend. Smoothing spreads a station's shift
    // onto its neighbours, and a neighbour standing under a steeper bank cannot
    // afford the same shift: on seed 7 the un-rechecked blend pushed the worst
    // excavation on the whole course from 4.84 m to 7.99 m. Pulling back is done
    // in the same 0.15 m steps and then re-blended over a short kernel, so the
    // line stays smooth.
    const fit = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      let s = lineMask[i] ? 0 : sm[i];
      if (Math.abs(s) < 0.02) { fit[i] = 0; continue; }
      const sgn = s < 0 ? -1 : 1;
      const y = S.py[i];
      const budget = Math.min(BENCH_CUT_TOTAL,
        Math.max(0, terrainH(terrain, S.px[i], S.pz[i]) - y) + BENCH_CUT_EXTRA);
      let a = Math.abs(s);
      while (a > 0.02) {
        const cut = terrainH(terrain, S.px[i] + S.rx[i] * sgn * a,
          S.pz[i] + S.rz[i] * sgn * a) - y;
        if (cut <= budget) break;
        a -= 0.15;
      }
      fit[i] = a > 0.02 ? sgn * a : 0;
    }
    const rad2 = M(4.0);
    const sm2 = gaussianSmooth(fit, rad2, rad2 * 0.45);
    for (let i = 0; i < n; i++) {
      // The blend can only give back shift the clamp took away, so take whichever
      // of the two is smaller in magnitude and keep the clamp's guarantee.
      let s = lineMask[i] ? 0 : sm2[i];
      if (Math.abs(s) > Math.abs(fit[i])) s = fit[i];
      if (Math.abs(s) < 0.02) continue;
      S.benchShift[i] = s;
      S.px[i] += S.rx[i] * s;
      S.pz[i] += S.rz[i] * s;
      safety.benched++;
      if (Math.abs(s) > safety.benchMax) safety.benchMax = Math.abs(s);
    }
    refreshBasis();
    for (let i = 0; i < n; i++) S.raw[i] = terrainH(terrain, S.px[i], S.pz[i]);
  }

  /**
   * Design the run-off shelf and its catch berm on each exposed side.
   *
   * The shelf runs from the tread edge out to `shelfRun` metres, falling away at
   * SHELF_SLOPE, and finishes in a berm `catchH` above the tread edge. `shelfRun`
   * is chosen as the LARGEST reach whose berm crest still stands on no more than
   * CATCH_FILL_MAX of built fill — i.e. where the ground has gone, the catch
   * comes in closer to the tread rather than the embankment growing taller. That
   * is the difference between a catch berm and a viaduct.
   *
   * Stored per station; buildStamps() turns it into carve stamps that genuinely
   * move the heightfield, so collision, the bike and the camera all see it.
   */
  function designCatch(terrain, flankMask) {
    const n = S.n;
    for (let i = 0; i < n; i++) {
      const half = Math.max(0.6, S.width[i] * 0.5);
      for (const sgn of [-1, 1]) {
        const expo = exposureAt(terrain, i, sgn, half);
        if (sgn < 0) S.expoL[i] = expo; else S.expoR[i] = expo;
        if (expo < EXPO_TRIGGER) continue;
        S.exposed[i] = 1;
        if (expo >= EXPO_TAPE) S.taped[i] = 1;
        if (flankMask[i]) continue;       // a jump or a wallride authors its own flanks
        const need = clamp01((expo - EXPO_TRIGGER) / (EXPO_FULL - EXPO_TRIGGER));
        let catchH = lerp(CATCH_H_MIN, CATCH_H_MAX, need);
        const ey = surfaceHeight(i, sgn * half);
        const bx = S.px[i] + S.rx[i] * sgn * half;
        const bz = S.pz[i] + S.rz[i] * sgn * half;
        const dx = S.rx[i] * sgn, dz = S.rz[i] * sgn;
        // Longest reach whose crest still stands on no more than CATCH_FILL_MAX
        // of built fill — i.e. where the mountain has gone, the catch comes in
        // toward the tread rather than the embankment growing taller. That is the
        // difference between a catch berm and a viaduct. Walked all the way in to
        // CATCH_MIN_Q the shelf degenerates into a raised, armoured outside edge,
        // which is the other thing builders do on an exposed traverse and is
        // still the difference between being bounced back onto the trail and
        // being launched off it.
        let run = 0;
        for (let q = RUNOFF_D; q >= CATCH_MIN_Q - 1e-6; q -= 0.15) {
          const crest = ey - SHELF_SLOPE * q + catchH;
          if (crest - terrainH(terrain, bx + dx * q, bz + dz * q) <= CATCH_FILL_MAX) {
            run = q; break;
          }
        }
        if (run <= 0) {
          // Nothing fits inside the ordinary fill budget. A short edge is a small
          // volume of material, so it gets a bigger one — this is a retaining
          // edge, and real courses build them out of rock and crib. Search again
          // over the short reaches only.
          for (let q = CATCH_MIN_Q * 2; q >= CATCH_MIN_Q - 1e-6; q -= 0.15) {
            const crest = ey - SHELF_SLOPE * q + catchH;
            if (crest - terrainH(terrain, bx + dx * q, bz + dz * q) <= CATCH_FILL_MAX * 1.7) {
              run = q; break;
            }
          }
        }
        if (run <= 0) {
          // Still nothing: drop the crest to whatever the budget does buy at the
          // minimum reach rather than leave the edge completely unmarked.
          run = CATCH_MIN_Q;
          const nat = terrainH(terrain, bx + dx * run, bz + dz * run);
          catchH = Math.min(catchH, (nat + CATCH_FILL_MAX * 1.7) - (ey - SHELF_SLOPE * run));
          if (catchH < 0.15) { safety.unprotectable++; continue; }
        }
        // Rideable inner face. The crest may never be taller than the run
        // available to ramp up to it at CATCH_FACE — see the note there.
        catchH = Math.min(catchH, CATCH_FACE * Math.min(CATCH_RAMP_MAX, run * 0.85));
        if (catchH < 0.12) { safety.unprotectable++; continue; }
        if (sgn < 0) { S.shelfL[i] = run; S.catchL[i] = catchH; }
        else { S.shelfR[i] = run; S.catchR[i] = catchH; }
      }
    }
    // Smooth the shelf and the berm along the trail so they grow and die away
    // over ~5 m instead of switching on at one station.
    const r = M(5.0);
    for (const k of ['shelfL', 'shelfR', 'catchL', 'catchR']) {
      S[k].set(gaussianSmooth(S[k], r, r * 0.45));
    }
    for (let i = 0; i < n; i++) {
      if (S.shelfL[i] >= CATCH_MIN_Q * 0.8 || S.shelfR[i] >= CATCH_MIN_Q * 0.8) {
        safety.catchBerms++;
        safety.shelfMetres += S.ds;
      }
    }
  }

  /**
   * The whole rule, in order: measure -> ease the gradient -> bench the line ->
   * widen -> build the catch. Each step re-measures, because each one changes
   * what the next one sees.
   */
  function applyExposureSafety(terrain) {
    if (!S || !terrain) return;
    const n = S.n;
    // Three masks, because the three responses threaten three different things.
    //   vert  — the height profile. Everything with authored relief goes in, the
    //           committing drops included: a 88% step over 3 m IS the feature and
    //           the ramp cap would sand it flat.
    //   line  — the plan alignment. Drops are NOT in it: a drop is a step down
    //           the trail, so sliding its approach 1 m sideways over 12 m costs
    //           it nothing, and refusing to bench there left a 5 m fall-away
    //           beside the 1499-1515 m drop with no response available at all.
    //   flank — the ground outboard of the tread edge. Only the wallride, whose
    //           outside IS an authored wall. Jumps are deliberately NOT excluded
    //           here: shelfRel() returns null inside the tread, so every stamp a
    //           jump emits is bit-identical, and the shelf's innermost band
    //           carries about 0.12 of carve weight back over the tread edge
    //           against the tread family's 2+ — about a centimetre. The measured
    //           jump table is unchanged (see the jumps/ballistics check), and a
    //           3.5 m unprotected drop beside the 997-1030 m landing is exactly
    //           the class of defect this pass exists to remove.
    const BALLISTIC = ['jump', 'doubles', 'gap', 'stepDown', 'wallride'];
    const vertMask = buildAuthoredMask(BALLISTIC.concat(['drop']), FEATURE_GUARD);
    const lineMask = buildAuthoredMask(BALLISTIC, FEATURE_GUARD);
    const flankMask = buildAuthoredMask(['wallride'], 2.0);

    // ---- pass 1: measure, and find the corridors the rule must protect ----
    const exposedMask = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const half = Math.max(0.6, S.width[i] * 0.5);
      const e = Math.max(exposureAt(terrain, i, -1, half), exposureAt(terrain, i, 1, half));
      if (e > safety.worstExpoBefore) safety.worstExpoBefore = e;
      if (e >= EXPO_TRIGGER) {
        exposedMask[i] = 1;
        safety.exposedStations++;
      }
    }
    safety.exposedMetres = safety.exposedStations * S.ds;
    for (let i = 0; i < n; i++) {
      if (exposedMask[i] && (i === 0 || !exposedMask[i - 1])) safety.stretches++;
    }
    // Dilate the exposed mask by the ramp window: the gradient that matters is
    // the one you are ALREADY carrying when you arrive at the exposure.
    {
      const g = M(RAMP_WIN * 1.5);
      const d = new Uint8Array(n);
      for (let i = 0; i < n; i++) {
        if (!exposedMask[i]) continue;
        for (let j = Math.max(0, i - g); j <= Math.min(n - 1, i + g); j++) d[j] = 1;
      }
      exposedMask.set(d);
    }

    // ---- pass 2: cap the grade ramp ---------------------------------------
    capGradeRamp(vertMask, exposedMask);
    refreshBasis();

    // ---- pass 3: bench the line into the hillside -------------------------
    benchLine(terrain, lineMask);

    // ---- pass 4: widen (still inside CONTRACT §0's 1.2-3.0 m) -------------
    {
      const add = new Float32Array(n);
      for (let i = 0; i < n; i++) {
        if (lineMask[i]) continue;
        const half = Math.max(0.6, S.width[i] * 0.5);
        const e = Math.max(exposureAt(terrain, i, -1, half), exposureAt(terrain, i, 1, half));
        if (e < EXPO_TRIGGER) continue;
        add[i] = clamp01((e - EXPO_TRIGGER) / (EXPO_FULL - EXPO_TRIGGER)) * WIDEN_MAX;
      }
      const r = M(8.0);
      const sm = gaussianSmooth(add, r, r * 0.45);
      for (let i = 0; i < n; i++) {
        if (sm[i] > 0.02 && S.width[i] < 3.0) safety.widened++;
        S.width[i] = clamp(S.width[i] + sm[i], 1.2, 3.0);
      }
    }

    // ---- pass 5: design the catch berm and run-off shelf ------------------
    designCatch(terrain, flankMask);
    for (let i = 0; i < n; i++) if (S.taped[i]) safety.tapedMetres += S.ds;
  }

  /**
   * TWO-LEG CONFLICT AUDIT AND RESIDUAL RESOLUTION — the downstream half of the
   * leg-clearance rule (the upstream half is legClearanceCost(), in the march).
   *
   * The march can only promise clearance on the polyline it committed. Three
   * later passes move the line without knowing about the other leg:
   * openTightCorners() shifts stations up to ~3.2 m laterally to open a corner,
   * benchLine() shifts up to BENCH_SHIFT_MAX toward the safe side, and the
   * feature and safety passes widen the tread by up to WIDEN_MAX + a berm's
   * widthAdd. Any of them can eat clearance the march paid for. So the property
   * is re-measured on the FINAL stations, which is the only place it can honestly
   * be claimed, and what is left is resolved rather than reported.
   *
   * Resolution is the narrow one, deliberately. Where two limbs contest ground,
   * a real builder does not move the trail — the corner is where it is — he cuts
   * a narrower bench on each and armours between them. Narrowing is bounded by
   * the CONTRACT §4 floor of 1.2 m and it is self-consistent: techSpeedCap()
   * reads the width, so a pinched limb is also a slower one. Where even the floor
   * cannot clear the pair the station is counted as unresolved, which is a fact
   * about the mountain and belongs in the audit, not hidden.
   *
   * `mode` 'audit' measures without touching anything.
   */
  function auditLegConflicts(mode) {
    const n = S.n;
    const CELL = 8.0;
    const bins = new Map();
    for (let i = 0; i < n; i++) {
      const key = Math.floor(S.pz[i] / CELL) * 8192 + Math.floor(S.px[i] / CELL);
      let l = bins.get(key);
      if (!l) { l = []; bins.set(key, l); }
      l.push(i);
    }
    const minArc = Math.round(LEG_ARC_MIN / STATION_DS);
    const half = (i) => Math.max(0.6, S.width[i] * 0.5);
    let changed = false;
    // Two sweeps: the first narrows, the second measures what is left.
    for (let pass = 0; pass < (mode === 'audit' ? 1 : 2); pass++) {
      const measuring = mode === 'audit' || pass === 1;
      let count = 0, metres = 0, worst = 0, worstAt = 0, worstGap = 0, unresolved = 0;
      let inRun = false;
      for (let i = 0; i < n; i++) {
        const bx = Math.floor(S.px[i] / CELL), bz = Math.floor(S.pz[i] / CELL);
        let deficit = 0, dh = 0, gap = 0;
        for (let a = -1; a <= 1; a++) {
          for (let b = -1; b <= 1; b++) {
            const l = bins.get((bz + b) * 8192 + (bx + a));
            if (!l) continue;
            for (let m = 0; m < l.length; m++) {
              const j = l[m];
              if (Math.abs(j - i) < minArc) continue;
              const d = Math.hypot(S.px[j] - S.px[i], S.pz[j] - S.pz[i]);
              const need = legReach(half(i)) + legReach(half(j));
              if (d >= need) continue;
              if (!measuring) {
                // Narrow both limbs, sharing the correction, down to the floor.
                const over = need - d;
                for (const k of [i, j]) {
                  const w = clamp(S.width[k] - over * 0.55, 1.2, 3.0);
                  if (w < S.width[k] - 1e-4) { S.width[k] = w; changed = true; }
                }
              }
              if (need - d > deficit) {
                deficit = need - d; gap = d;
                dh = Math.abs(S.py[j] - S.py[i]);
              }
            }
          }
        }
        if (measuring && deficit > 0) {
          metres += STATION_DS;
          if (!inRun) { count++; inRun = true; }
          if (dh > worst) { worst = dh; worstAt = i; worstGap = gap; }
          if (S.width[i] <= 1.2 + 1e-4) unresolved++;
        } else if (measuring) inRun = false;
      }
      if (measuring) {
        const rec = {
          count, metres: +metres.toFixed(1), unresolvedStations: unresolved,
          worstHeightGap: +worst.toFixed(2), worstPlanGap: +worstGap.toFixed(2),
          worstAtArc: +(worstAt * STATION_DS).toFixed(1),
        };
        if (mode === 'audit') safety.legBefore = rec; else safety.legAfter = rec;
      }
    }
    return changed;
  }

  function pushStamp(x, z, radius, targetHeight, falloff, material, bank, kind) {
    stamps.push({ x, z, radius, targetHeight, falloff, material, bank, kind });
  }

  /**
   * The cross-slope of the DESIGNED surface at lateral offset `o`, as an angle
   * in the same convention `stamp.bank` uses (+ = rider's right side high).
   *
   * Every stamp is a disc whose target height varies across its own footprint
   * as `s * tan(stamp.bank)` (terrain.js stampTarget). If that bank is not the
   * slope of the surface the stamp is supposed to be building, the disc is
   * TILTED WRONG over its whole influence radius, and applyCarve averages the
   * error into the committed heightfield.
   *
   * That was happening at every station carrying a run-off shelf. The shelf and
   * catch-berm bands were emitted with `bank: 0` — flat discs — while the tread
   * they abut is banked up to 30 deg, and their influence radius (1.9 x 0.36 m)
   * reaches 0.68 m back inside the tread edge. A flat disc laid across a 30 deg
   * cross-slope is wrong by 0.39 m at the edge of its own reach.
   *
   * Measured on the committed heightfield at 70-100 m before this: the tread
   * surface deviated from the emitted design plane by up to 0.42 m, swinging
   * +0.15 / -0.28 m at a ~3 m wavelength INSIDE the tread. That is what threw
   * the bike off at 81 m — and it threw it off at 45% of the design speed too,
   * which is how we know it was geometry and not pace.
   *
   * Sampling the design surface either side of the stamp centre and handing the
   * stamp its own local slope makes each disc tangent to what it is building,
   * so overlapping discs agree instead of fighting.
   */
  function localCross(i, o) {
    const e = 0.18;
    const d = (surfaceHeight(i, o + e) - surfaceHeight(i, o - e)) / (2 * e);
    return Math.atan(clamp(d, -1.4, 1.4));
  }

  function buildStamps() {
    stamps = [];
    const n = S.n;
    for (let i = 0; i < n; i++) {
      const half = Math.max(0.6, S.width[i] * 0.5);
      const x = S.px[i], z = S.pz[i];
      const rxi = S.rx[i], rzi = S.rz[i];
      const bank = S.bank[i];
      const mat = S.surface[i];
      const isBerm = Math.abs(bank) > THREE.MathUtils.degToRad(12);
      const isWall = S.wallride[i] > 0.05;
      const feat = S.feat[i] >= 0 ? features[S.feat[i]] : null;
      const ftype = feat ? feat.type : null;

      let kind = 'tread';
      if (isBerm || isWall) kind = 'berm';
      if (ftype === 'jump' || ftype === 'doubles' || ftype === 'gap' || ftype === 'stepDown') {
        if (i <= (feat.params.iLip || 0)) kind = 'lip';
        else if (i >= (feat.params.iKnuckle || 0)) kind = 'landing';
        else kind = 'lip';
      }
      if (ftype === 'drop') kind = 'drop';

      // Sharper stamps where the shape must stay crisp (lips, knuckles, drops).
      const crisp = (kind === 'lip' || kind === 'landing' || kind === 'drop');
      const falloff = crisp ? 1.35 : 2.1;
      // MEASURED AND REJECTED: shrinking the discs where the cross-slope is
      // steep. A stamp declares a PLANE, so its radius is a claim about how far
      // the surface stays planar, and on a 43 deg berm across a 3.0 m tread it
      // does not — the largest residual disagreements left in the cloud are one
      // 1.58 m disc extrapolated over a doubly curved berm (seed 12345, arc
      // 2417, 3.66 m of spread). Scaling the radius by 1/(1+|tan(bank)|*0.45)
      // does reduce that: the >0.50 m band goes 1.57/2.44/2.55% -> 1.05/1.92/
      // 1.69% across the three seeds and p99 0.59 -> 0.51 m. It also makes the
      // COMMITTED surface very slightly rougher (wheel-launch 26.80 -> 27.15%,
      // 23.99 -> 24.16%, 23.35 -> 23.89%) because the along-track overlap
      // between consecutive stations shrinks with the radius, and it leaves the
      // committed-vs-emitted residual completely unmoved (p99 0.115 m either
      // way). It trades the metric that matters for the one being reported, so
      // it is not taken.
      const rScale = crisp ? 0.62 : 1.0;

      // ---- who may invite the cut/fill batter --------------------------------
      //
      // terrain.applyCarve() batters the ground outside any stamp of kind 'tread'
      // or 'rut' back to an angle of repose, CLAMPED AGAINST NATURAL GROUND. On a
      // fall-away that clamp means the batter votes for the void, and it votes
      // from a long way off (BATTER_RUN is 4 m). Measured on seed 20260726, the
      // batter votes from one station's own tread stamps summed to ~0.35 of carve
      // weight at a target 4.5 m down, which is enough to cancel a designed 0.66 m
      // catch berm outright — the berm arrived on the heightfield at 0.18 m BELOW
      // the tread edge. That is precisely the "paints it but does not build it"
      // failure mode.
      //
      // The rule that resolves it: where the trail has BUILT ground outside a
      // tread edge, there is nothing out there for a batter to taper, so no stamp
      // on that side may ask for one. The outermost stamp on a side WITHOUT a
      // shelf keeps its 'tread' kind, so the cut bank above the trail still gets
      // its repose taper and the r3 vertical-wall defect does not come back.
      const shelfL = S.shelfL[i] >= CATCH_MIN_Q * 0.8;
      const shelfR = S.shelfR[i] >= CATCH_MIN_Q * 0.8;
      const anyShelf = shelfL || shelfR;
      /** @param {number} side -1 left, +1 right, 0 centreline. */
      const kindFor = (base, side, outermost) => {
        if (!anyShelf || (base !== 'tread' && base !== 'rut')) return base;
        const sideBuilt = side < 0 ? shelfL : side > 0 ? shelfR : true;
        if (!sideBuilt && outermost) return base;      // keep the cut-bank batter
        return 'berm';
      };

      // Centre stamp.
      pushStamp(x, z, (half * 0.80 + 0.38) * rScale, surfaceHeight(i, 0), falloff,
        mat, localCross(i, 0), kindFor(kind, 0, false));

      // Lateral stamps. Their targetHeight already contains the banked/crowned
      // cross-section, so a terrain that ignores `bank` still gets the right shape.
      const offs = isWall ? [-0.55, 0.55, -0.9, 0.9, -1.18, 1.18, -1.45, 1.45]
        : isBerm ? [-0.55, 0.55, -0.92, 0.92, -1.22, 1.22]
          : [-0.58, 0.58, -1.0, 1.0];
      const outerMag = Math.abs(offs[offs.length - 1]);
      for (let k = 0; k < offs.length; k++) {
        const o = offs[k] * half;
        const base = Math.abs(offs[k]) > 1.05
          ? (isBerm || isWall ? 'berm' : 'tread') : kind;
        pushStamp(
          x + rxi * o, z + rzi * o,
          (half * 0.50 + 0.30) * rScale,
          surfaceHeight(i, o),
          falloff, mat, localCross(i, o),
          kindFor(base, Math.sign(offs[k]), Math.abs(offs[k]) >= outerMag - 1e-6));
      }

      // Ruts as their own stamps so the terrain PAINTS them as worn channels.
      //
      // THEY MUST NOT DIG. Measured on the shipped world (seed 20260726): these
      // stamps carried `- 0.012 - 0.24` below the shared cross-section wherever
      // the station also had a run-off shelf (which demotes the kind to 'berm'),
      // so at every such station a 0.34 m stamp — floored to 0.6 m and widened to
      // 0.68 m of reach by terrain.applyCarve — declared a surface a quarter of a
      // metre below the one every tread stamp at the same point declared. That is
      // the single largest systematic contradiction in the emitted cloud, and it
      // is a straight violation of the rule stated over surfaceHeight(): every
      // stamp at every offset derives its target from ONE function.
      //
      // The 0.24 m was compensating for terrain.stampTarget()'s old kind-specific
      // 'rut' section, which no longer exists — terrain adds no trough of its own
      // now, so the compensation is not merely inconsistent, it is stale. The rut
      // is already in crossProfile(); if it needs to be deeper, S.rut is the knob,
      // and every stamp sees it.
      //
      // The radius is emitted at the 0.6 m floor terrain will apply anyway, so the
      // declared footprint is the real one rather than a fiction 1.8x smaller.
      if (S.rut[i] > 0.045 && !crisp && (i % 2) === 0) {
        for (const sgn of [-1, 1]) {
          const o = sgn * half * (0.30 + microN(i * 0.02) * 0.1);
          pushStamp(x + rxi * o, z + rzi * o, 0.6,
            surfaceHeight(i, o), 1.4, mat, localCross(i, o), kindFor('rut', sgn, false));
        }
      }

      // Rock garden: scattered proud boulders inside the corridor.
      //
      // Emitted at the true footprint (terrain floors every radius at 0.6 m, so a
      // 0.22 m stamp was declaring a boulder over three times its stated width),
      // and the proud height is bounded by what that footprint can support at a
      // slope a wheel can climb: a 0.6 m-radius cap standing `rise` proud has a
      // mean face slope of rise/0.6, and past ~0.55 of that it stops being a rock
      // in the trail and becomes a step the front wheel hits square.
      if (S.surface[i] === Surface.ROCK && S.rough[i] > 0.07 && (i % 5) === 0) {
        const r = featRng();
        if (r < 0.55) {
          const o = (featRng() * 2 - 1) * half * 0.95;
          const rad = 0.6 + featRng() * 0.34;
          const rise = Math.min(0.30, rad * 0.42);
          pushStamp(x + rxi * o, z + rzi * o, rad,
            surfaceHeight(i, o) + rise, 1.1, Surface.ROCK, localCross(i, o), 'tread');
        }
      }
    }

    // --- exposure protection: run-off shelf and catch berm -----------------
    //
    // These are the stamps that make applyExposureSafety() real. A stamp writes
    // a height target into the heightfield, so this genuinely raises ground that
    // was not there — it is not a paint pass and it is not a decal. Two details
    // matter and are easy to get wrong:
    //
    //  - kind 'berm', NOT 'tread'. terrain.applyCarve() only batters plain tread
    //    back to an angle of repose; anything else it treats as constructed and
    //    leaves alone. A catch berm emitted as 'tread' would be recut to repose
    //    and cease to exist, which is exactly the "paints it but does not build
    //    it" failure.
    //  - emitted band by band, each band walking the whole trail in station
    //    order, because prepareStamps() takes a stamp's along-track reach from
    //    its ARRAY neighbours. Emitted station-major the bands would zigzag
    //    across the trail and every stamp would be stretched sideways.
    //
    // Every second station: the stamps stretch along their own tangent to cover
    // the gap to their array neighbour, so 0.8 m spacing still reads continuous,
    // and halving the count keeps the carve inside its build-time budget.
    //
    // MAX_BANDS bands cover shelf + crest at ~0.28 m, then up to 3 more walk the
    // BACK of the berm down at CATCH_BACK_SLOPE until it meets natural ground.
    // The back matters: without stamps outboard of the crest, the carve's mean
    // has nothing on that side to average against and it rounds the crest off.
    const BAND_DS = 0.28;
    const MAX_BANDS = 9;
    for (const sgn of [-1, 1]) {
      const runArr = sgn < 0 ? S.shelfL : S.shelfR;
      // Bands are emitted band-index-major so that each band's array neighbours
      // run ALONG the trail. Station-major they would zigzag across it and
      // prepareStamps() would stretch every stamp sideways.
      for (let b = 1; b <= MAX_BANDS + 3; b++) {
        for (let i = 0; i < n; i += 2) {
          const run = runArr[i];
          if (run < CATCH_MIN_Q * 0.8) continue;
          const half = Math.max(0.6, S.width[i] * 0.5);
          const outer = run + CATCH_PLATEAU;             // outer edge of the flat crest
          const nb = clamp(Math.round(outer / BAND_DS), 3, MAX_BANDS);
          const edgeY = surfaceHeight(i, sgn * half);
          const crestY = edgeY + (sgn < 0 ? S.catchL[i] : S.catchR[i]) - SHELF_SLOPE * run;
          let q, y;
          if (b <= nb) {
            q = outer * (b / nb);
            y = surfaceHeight(i, sgn * (half + q));      // shelf + ramp + flat crest
          } else {
            q = outer + (b - nb) * BAND_DS * 1.4;
            if (q > outer + CATCH_BACK_MAX) continue;
            y = crestY - CATCH_BACK_SLOPE * (q - outer);
            const nat = terrainSampleCache(
              S.px[i] + S.rx[i] * sgn * (half + q), S.pz[i] + S.rz[i] * sgn * (half + q));
            if (y <= nat + 0.05) continue;               // the back has daylighted
          }
          const o = sgn * (half + q);
          pushStamp(
            S.px[i] + S.rx[i] * o, S.pz[i] + S.rz[i] * o,
            0.36, y, 1.9, Surface.GRAVEL, localCross(i, o), 'berm',
          );
        }
      }
    }

    // --- split branches ----------------------------------------------------
    //
    // Height comes from branchBaseY() — see the note there for what it replaces
    // and why the tread ribbon now reads the same function.
    //
    // WHAT IS EMITTED WHERE. A branch stamp may not vote on the main line's
    // surface, so each one is admitted only if its own footprint stays outboard
    // of the main tread edge, and its radius is cut to the room available:
    // terrain floors every radius at 0.6 m and widens it by up to 14%, so a
    // stamp whose axis sits `d` metres outside the edge may have a radius of at
    // most d/1.14, and below 0.684 m of room there is no admissible stamp at
    // all. The branch therefore grows out of the tread edge as the lines
    // separate instead of appearing on top of the main line — which is both the
    // correct geometry and what the ribbon draws.
    for (const sp of splits) {
      for (let i = sp.i0; i <= sp.i1; i++) {
        const o = branchOffset(sp, i);
        if (o === 0) continue;
        const sgn = Math.sign(o);
        const mainHalf = Math.max(0.6, S.width[i] * 0.5);
        const baseY = branchBaseY(sp, i);
        const bankB = sp.branchIsB ? S.bank[i] * 0.4 : S.bank[i] * 0.8;
        // Safer line, looser surface: the B-line trades grip for exposure.
        const matB = sp.branchIsB && S.surface[i] !== Surface.MUD
          ? Surface.GRAVEL : S.surface[i];
        for (const k of [0, -0.7, 0.7, -1.05, 1.05]) {
          const oo = k * BRANCH_HALF;
          const lat = o + oo;                       // offset from the centreline
          const room = Math.abs(lat) - mainHalf;    // ...outside the main tread edge
          if (Math.sign(lat) !== sgn || room <= 0.6 * 1.14) continue;
          const rad = clamp(room / 1.14, 0.6, k === 0 ? 1.15 : 0.7);
          pushStamp(
            S.px[i] + S.rx[i] * lat, S.pz[i] + S.rz[i] * lat, rad,
            baseY + Math.tan(bankB) * oo + (Math.abs(k) > 0.9 ? 0.05 : Math.abs(k) > 0 ? 0.02 : 0),
            2.0, matB, bankB, 'tread',
          );
        }
      }
    }
  }

  // =========================================================================
  // 7. SPATIAL INDEX for the global nearestT fallback
  // =========================================================================

  function buildIndex(terrain) {
    const B = terrain.bounds;
    gridMinX = B.minX; gridMinZ = B.minZ;
    gridW = Math.max(1, Math.ceil((B.maxX - B.minX) / gridCell));
    gridH = Math.max(1, Math.ceil((B.maxZ - B.minZ) / gridCell));
    const cells = gridW * gridH;
    const counts = new Int32Array(cells + 1);
    const cellOf = new Int32Array(S.n);
    for (let i = 0; i < S.n; i++) {
      const cx = clamp(Math.floor((S.px[i] - gridMinX) / gridCell), 0, gridW - 1);
      const cz = clamp(Math.floor((S.pz[i] - gridMinZ) / gridCell), 0, gridH - 1);
      const c = cz * gridW + cx;
      cellOf[i] = c;
      counts[c + 1]++;
    }
    for (let c = 0; c < cells; c++) counts[c + 1] += counts[c];
    cellStart = counts;
    cellItems = new Int32Array(S.n);
    const cursor = Int32Array.from(counts.subarray(0, cells));
    for (let i = 0; i < S.n; i++) cellItems[cursor[cellOf[i]]++] = i;
  }

  // =========================================================================
  // BUILD
  // =========================================================================

  function build(terrain) {
    if (!terrain) return [];
    _terrainRef = terrain;

    // Route-find several times and keep the best line down the hill.
    let route = null, bestScore = -Infinity;
    routeAudit.length = 0;
    for (let v = 0; v < 7; v++) {
      const cand = marchRoute(terrain, v);
      const sc = scoreRoute(terrain, cand);
      routeAudit.push({
        variant: v, score: +sc.toFixed(2),
        startX: +cand.startX.toFixed(1), startY: +cand.startY.toFixed(1),
        corrMean: +(cand.corrMean || 0).toFixed(2),
        corrWorst: +(cand.corrWorst || 0).toFixed(2),
        corrBad: cand.corrBad || 0,
        len: +(cand.marchedLength || 0).toFixed(0),
        drop: +(cand.marchedDrop || 0).toFixed(0),
        openGrade: +(cand.openGrade || 0).toFixed(3),
      });
      if (sc > bestScore) { bestScore = sc; route = cand; }
    }
    for (const r of routeAudit) r.chosen = r.variant === route.variant;
    buildStations(terrain, route);
    solveSpeeds();
    // Bound the crest curvature against the speed the profile itself asks for,
    // BEFORE any feature is placed — so the jump line, the drops and the
    // rollers are authored on top of a profile that already holds the wheels
    // down, and nothing this pass does can touch their ballistics afterwards.
    // Twice, because the cap is a function of speed and the speed is a function
    // of the gradient this pass just changed.
    {
      const empty = new Uint8Array(S.n);
      for (let k = 0; k < 2; k++) {
        capCrestCurvature(empty);
        refreshBasis();
        solveSpeeds();
      }
    }
    safety.crestStage.afterCap = countCrestViol();
    buildFeatures();
    buildSplits();
    // Features changed the surfaces and widths, so re-solve the speed profile with
    // the final numbers — but do NOT re-bank, or it would undo the flattened jump
    // lips and the over-banked wallride.
    solveSpeeds(false);

    // Final surface Y includes the feature offsets.
    for (let i = 0; i < S.n; i++) S.py[i] = S.by[i] + S.hOff[i];

    // ---- the crest cap has to run AGAIN, on the profile the features left ----
    //
    // The pass above deliberately runs BEFORE buildFeatures() so that nothing it
    // does can touch a jump's ballistics. That is right, and it is not enough:
    // measured on seed 20260726, the profile leaving the cap carried 72 crest
    // violations and the profile leaving buildFeatures() carried 252. The
    // features add 180 launch points of their own, on the ground BETWEEN the
    // authored relief — roller crests, berm entries, the shoulders either side
    // of a chute — and nothing re-measured it. The reported crestViolAfter came
    // from the cap's own exit, so the number that was signed off described a
    // profile four passes upstream of the one that ships.
    //
    // Running it a second time under the SAME authored-relief mask
    // applyExposureSafety uses keeps the original guarantee exactly: every
    // station carrying a lip, a landing, a committing drop or the wallride is
    // masked, so the jump table is unchanged to the millimetre, and only the
    // ordinary ground either side of them is flattened back under the cap.
    {
      const vm = buildAuthoredMask(
        ['jump', 'doubles', 'gap', 'stepDown', 'wallride', 'drop'], FEATURE_GUARD);
      safety.crestStage.afterFeatures = countCrestViol(vm);
      capCrestCurvature(vm);
      refreshBasis();
      solveSpeeds(false);
      for (let i = 0; i < S.n; i++) S.py[i] = S.by[i] + S.hOff[i];
      safety.crestStage.afterRecap = countCrestViol(vm);
      crestMask = vm;
    }

    // Level-design safety audit and mitigation. Must run AFTER the features have
    // set the final width/bank/height (it measures the real cross-section) and
    // BEFORE the curve rebuild and buildStamps() (it moves px/pz/py and adds the
    // shelf geometry). See the long note above applyExposureSafety().
    applyExposureSafety(terrain);
    safety.crestStage.afterSafety = countCrestViol(crestMask);

    // Two-leg conflicts, measured on the FINAL stations and resolved by pinching
    // the contesting limbs. See auditLegConflicts(). The speed profile is re-run
    // when a width actually moved, because techSpeedCap() reads it.
    auditLegConflicts('audit');
    if (auditLegConflicts('resolve')) solveSpeeds(false);

    for (const f of features) {
      const mid = (f.i0 + f.i1) >> 1;
      f.position.set(S.px[mid], S.py[mid], S.pz[mid]);
    }
    // Recompute tangents/grades against the final surface so sampleAt is honest.
    for (let i = 0; i < S.n; i++) {
      const a = Math.max(0, i - 1), b = Math.min(S.n - 1, i + 1);
      const dx = S.px[b] - S.px[a], dz = S.pz[b] - S.pz[a], dy = S.py[b] - S.py[a];
      const l = Math.max(1e-5, Math.hypot(dx, dy, dz));
      S.tx[i] = dx / l; S.ty[i] = dy / l; S.tz[i] = dz / l;
      S.grade[i] = -S.ty[i];
    }
    // Rebuild the exposed curve against the final surface.
    {
      const decim = Math.max(1, Math.round(4 / STATION_DS));
      const cpts = [];
      for (let i = 0; i < S.n; i += decim) cpts.push(new THREE.Vector3(S.px[i], S.py[i], S.pz[i]));
      const tail = new THREE.Vector3(S.px[S.n - 1], S.py[S.n - 1], S.pz[S.n - 1]);
      // A duplicated final control point gives CatmullRom a zero-length segment
      // and NaN tangents, which would poison every consumer of trail.curve.
      if (cpts[cpts.length - 1].distanceTo(tail) > 0.05) cpts.push(tail);
      curve = new THREE.CatmullRomCurve3(cpts, false, 'centripetal', 0.5);
      curve.arcLengthDivisions = Math.max(2000, cpts.length * 8);
      length = curve.getLength();
    }

    buildStamps();
    buildIndex(terrain);

    // ---- phase spans (additive info) --------------------------------------
    phaseSpans.length = 0;
    for (let p = 0; p < PHASES.length; p++) {
      let a = -1, b = -1;
      for (let i = 0; i < S.n; i++) if (S.phase[i] === p) { if (a < 0) a = i; b = i; }
      if (a < 0) continue;
      phaseSpans.push({
        id: PHASES[p].id, name: PHASES[p].name,
        tStart: a / (S.n - 1), tEnd: b / (S.n - 1),
        surface: PHASES[p].surface,
      });
    }

    // ---- checkpoints: 8, spread, snapped to calm stations ------------------
    checkpoints.length = 0;
    for (let k = 1; k <= 8; k++) {
      const nominal = Math.round((S.n - 1) * (k / 9) * 0.98 + (S.n - 1) * 0.02);
      let best = nominal, bestScore = -Infinity;
      for (let d = -M(20); d <= M(20); d++) {
        const i = clamp(nominal + d, M(12), S.n - 1 - M(30));
        // Prefer straight, unbanked, un-featured stations for a gate.
        let sc = Math.min(S.radius[i], 200) * 0.05 - Math.abs(S.bank[i]) * 30 -
          Math.abs(S.hOff[i]) * 6 - Math.abs(d) * 0.004;
        if (S.feat[i] >= 0) {
          const ft = features[S.feat[i]].type;
          if (ft === 'jump' || ft === 'doubles' || ft === 'gap' || ft === 'stepDown' || ft === 'drop') sc -= 100;
        }
        if (sc > bestScore) { bestScore = sc; best = i; }
      }
      checkpoints.push({
        t: best / (S.n - 1),
        position: new THREE.Vector3(S.px[best], S.py[best], S.pz[best]),
        index: k - 1,
        station: best,
      });
    }

    // ---- start transform ---------------------------------------------------
    {
      const i = M(3);
      _fwd.set(S.tx[i], S.ty[i], S.tz[i]).normalize();
      _rgt.set(S.rx[i], 0, S.rz[i]).normalize();
      _q.crossVectors(_rgt, _fwd).normalize();        // up
      _m4.makeBasis(_rgt, _q, _p.copy(_fwd).negate());
      startTransform.quaternion.setFromRotationMatrix(_m4);
      startTransform.position.set(S.px[i], S.py[i] + 0.55, S.pz[i]);
    }

    if (ctx.debug && ctx.debug.enabled) {
      ctx.debug.log('trail', {
        length: +length.toFixed(1),
        drop: +(S.py[0] - S.py[S.n - 1]).toFixed(1),
        avgGrade: +((S.py[0] - S.py[S.n - 1]) / length).toFixed(4),
        stations: S.n, stamps: stamps.length,
        features: features.length, splits: splits.length,
      });
      ctx.debug.log('trail.safety', safety);
      // eslint-disable-next-line no-console
      console.table(jumpLog);
    }

    return stamps;
  }

  function getCarveStamps() { return stamps; }

  // =========================================================================
  // QUERIES
  // =========================================================================

  function sampleAt(t) {
    if (!S) return _sample;
    const u = clamp01(t) * (S.n - 1);
    const i = Math.min(S.n - 2, u | 0);
    const f = u - i;
    const j = i + 1;
    _sample.position.set(
      S.px[i] + (S.px[j] - S.px[i]) * f,
      S.py[i] + (S.py[j] - S.py[i]) * f,
      S.pz[i] + (S.pz[j] - S.pz[i]) * f);
    _sample.tangent.set(
      S.tx[i] + (S.tx[j] - S.tx[i]) * f,
      S.ty[i] + (S.ty[j] - S.ty[i]) * f,
      S.tz[i] + (S.tz[j] - S.tz[i]) * f).normalize();
    const bank = S.bank[i] + (S.bank[j] - S.bank[i]) * f;
    // Binormal = rider's right, in the banked plane.
    _sample.binormal.set(S.rx[i] + (S.rx[j] - S.rx[i]) * f, 0, S.rz[i] + (S.rz[j] - S.rz[i]) * f).normalize();
    _sample.normal.crossVectors(_sample.binormal, _sample.tangent).normalize();
    // Roll both by the bank around the tangent.
    _quat.setFromAxisAngle(_sample.tangent, -bank);
    _sample.normal.applyQuaternion(_quat).normalize();
    _sample.binormal.applyQuaternion(_quat).normalize();
    _sample.width = S.width[i] + (S.width[j] - S.width[i]) * f;
    _sample.bank = bank;
    _sample.gradient = -_sample.tangent.y;
    const fi = S.feat[f < 0.5 ? i : j];
    _sample.feature = fi >= 0 ? features[fi] : null;
    return _sample;
  }

  function widthAt(t) {
    if (!S) return 2.2;
    const u = clamp01(t) * (S.n - 1);
    const i = Math.min(S.n - 2, u | 0);
    const f = u - i;
    return S.width[i] + (S.width[i + 1] - S.width[i]) * f;
  }

  function speedAt(t) {
    if (!S) return 0;
    const u = clamp01(t) * (S.n - 1);
    const i = Math.min(S.n - 2, u | 0);
    const f = u - i;
    return S.speed[i] + (S.speed[i + 1] - S.speed[i]) * f;
  }

  function surfaceAt(t) {
    if (!S) return Surface.DIRT;
    return S.surface[clamp(Math.round(clamp01(t) * (S.n - 1)), 0, S.n - 1)];
  }

  // Distance² from a point to station i.
  function d2(i, x, y, z) {
    const dx = S.px[i] - x, dy = S.py[i] - y, dz = S.pz[i] - z;
    return dx * dx + dy * dy + dz * dz;
  }

  /**
   * Nearest point on the centreline. Local window search around `hint` (called
   * every frame), escalating to the spatial index only when the hint is stale.
   */
  function nearestT(position, hint) {
    if (!S) return _near;
    const x = position.x, y = position.y, z = position.z;
    let best = -1, bestD = Infinity;

    if (typeof hint === 'number' && isFinite(hint)) {
      const c = clamp(Math.round(hint * (S.n - 1)), 0, S.n - 1);
      const lo = Math.max(0, c - NEAR_WINDOW);
      const hi = Math.min(S.n - 1, c + NEAR_WINDOW);
      for (let i = lo; i <= hi; i++) {
        const d = d2(i, x, y, z);
        if (d < bestD) { bestD = d; best = i; }
      }
      // Stale hint if we landed on the window edge or we're a long way off.
      if (best === lo && lo > 0) best = -1;
      else if (best === hi && hi < S.n - 1) best = -1;
      else if (bestD > 400) best = -1;
    }

    if (best < 0) {
      bestD = Infinity;
      // Spatial index: widen the ring until something is found.
      const cx = Math.floor((x - gridMinX) / gridCell);
      const cz = Math.floor((z - gridMinZ) / gridCell);
      // Always sweep out to ring 2 (±72 m). Stopping at the first ring that holds
      // *any* station is wrong where switchbacks stack — the leg above can be in
      // a nearer cell than the leg you are actually standing on.
      for (let ring = 0; ring <= 8; ring++) {
        if (ring > 2 && best >= 0) break;
        for (let dz = -ring; dz <= ring; dz++) {
          for (let dx = -ring; dx <= ring; dx++) {
            if (ring > 0 && Math.abs(dx) !== ring && Math.abs(dz) !== ring) continue;
            const gx = cx + dx, gz = cz + dz;
            if (gx < 0 || gz < 0 || gx >= gridW || gz >= gridH) continue;
            const c = gz * gridW + gx;
            const s0 = cellStart[c], s1 = cellStart[c + 1];
            for (let k = s0; k < s1; k++) {
              const i = cellItems[k];
              const d = d2(i, x, y, z);
              if (d < bestD) { bestD = d; best = i; }
            }
          }
        }
      }
      if (best < 0) {
        // Absolute fallback: coarse global scan, then refine.
        const stride = 16;
        for (let i = 0; i < S.n; i += stride) {
          const d = d2(i, x, y, z);
          if (d < bestD) { bestD = d; best = i; }
        }
        const lo = Math.max(0, best - stride), hi = Math.min(S.n - 1, best + stride);
        for (let i = lo; i <= hi; i++) {
          const d = d2(i, x, y, z);
          if (d < bestD) { bestD = d; best = i; }
        }
      } else {
        // Refine around the indexed hit (cells are 24 m, stations 0.4 m).
        const lo = Math.max(0, best - 130), hi = Math.min(S.n - 1, best + 130);
        for (let i = lo; i <= hi; i++) {
          const d = d2(i, x, y, z);
          if (d < bestD) { bestD = d; best = i; }
        }
      }
    }

    // Sub-station refinement: project onto the two adjacent segments.
    let bi = best, bf = 0, bd = bestD;
    for (const s of [-1, 0]) {
      const a = best + s, b = a + 1;
      if (a < 0 || b > S.n - 1) continue;
      const ax = S.px[a], ay = S.py[a], az = S.pz[a];
      const ex = S.px[b] - ax, ey = S.py[b] - ay, ez = S.pz[b] - az;
      const len2 = ex * ex + ey * ey + ez * ez;
      if (len2 < 1e-9) continue;
      let f = ((x - ax) * ex + (y - ay) * ey + (z - az) * ez) / len2;
      f = f < 0 ? 0 : f > 1 ? 1 : f;
      const qx = ax + ex * f, qy = ay + ey * f, qz = az + ez * f;
      const dx = x - qx, dy = y - qy, dz = z - qz;
      const d = dx * dx + dy * dy + dz * dz;
      if (d < bd) { bd = d; bi = a; bf = f; }
    }

    const i = bi, j = Math.min(S.n - 1, bi + 1), f = bf;
    _near.position.set(
      S.px[i] + (S.px[j] - S.px[i]) * f,
      S.py[i] + (S.py[j] - S.py[i]) * f,
      S.pz[i] + (S.pz[j] - S.pz[i]) * f);
    _near.tangent.set(
      S.tx[i] + (S.tx[j] - S.tx[i]) * f,
      S.ty[i] + (S.ty[j] - S.ty[i]) * f,
      S.tz[i] + (S.tz[j] - S.tz[i]) * f).normalize();
    _near.t = (i + f) / (S.n - 1);
    const rX = S.rx[i] + (S.rx[j] - S.rx[i]) * f;
    const rZ = S.rz[i] + (S.rz[j] - S.rz[i]) * f;
    const rl = Math.max(1e-6, Math.hypot(rX, rZ));
    _near.lateral = ((x - _near.position.x) * rX + (z - _near.position.z) * rZ) / rl;
    _near.distance = Math.sqrt(bd);
    return _near;
  }

  // =========================================================================
  // 8. FINALIZE — the visible race course
  // =========================================================================

  let treadMesh = null;
  let postsMesh = null;
  // The tread's own detail albedo, reused by the contact mounds so disturbed
  // soil at the foot of a post is made of the same dirt as the trail.
  let treadAlbedoTex = null;
  const disposables = [];

  function reg(obj) { disposables.push(obj); return obj; }

  /**
   * Cloth/flutter vertex patch shared by tape and banners.
   *
   * `opts.minPx` additionally holds a ribbon at a minimum screen width. A
   * course tape is 75 mm tall, which is 3.5 px at 40 m and 0.9 px at 150 m: it
   * drops below one pixel, the printed red stripe aliases away, and the ribbon
   * both flickers in motion and reads as a bare white thread in a still. The
   * expansion is done in clip space along the ribbon's own edge attribute, so
   * near tape (already wider than the minimum) is untouched.
   *
   * IT IS ALSO BOUNDED, and it was not. `minNdc` is a screen-space target with
   * no relationship to the object's real size, so the amount of geometry it
   * manufactures grows without limit as the ribbon recedes: at 400 m a 75 mm
   * ribbon is 0.34 px and the term was widening it 5x, at 1 km 12x, and the
   * whole 3.3 km of course tape is inside the draw distance of an establishing
   * shot. That is how a 75 mm ribbon was measured occupying 12.5% of the frame
   * as a pale sheet — not a geometry bug, an unbounded screen-space term. It
   * also depends on a `uViewportH` uniform this module cannot verify (the
   * drawing buffer moves under the resolution governor between resize() calls),
   * so an unbounded form has no worst case at all.
   *
   * MINPX_MAX_GROW caps the drawn height at a fixed multiple of the ribbon's
   * TRUE projected height. Inside ~150 m that is the anti-aliasing floor doing
   * its job; past it the ribbon simply gets smaller, the way a 75 mm strip of
   * polythene at 400 m should. The bound holds whatever uViewportH says.
   */
  const MINPX_MAX_GROW = 2.0;
  function applyFlutter(material, amp, opts) {
    const minPx = opts && opts.minPx ? opts.minPx : 0;
    const worldH = opts && opts.worldH ? opts.worldH : 1;
    material.onBeforeCompile = (shader) => {
      shader.uniforms.uTime = flutterUniforms.uTime;
      shader.uniforms.uWindDir = flutterUniforms.uWindDir;
      shader.uniforms.uWindAmp = { value: amp };
      shader.vertexShader = `
        attribute float aFlutter;
        uniform float uTime;
        uniform vec3 uWindDir;
        uniform float uWindAmp;
      ` + shader.vertexShader.replace('#include <begin_vertex>', `
        #include <begin_vertex>
        {
          float fw = aFlutter;
          if (fw > 0.001) {
            vec3 wp = (modelMatrix * vec4(transformed, 1.0)).xyz;
            float ph = wp.x * 0.55 + wp.z * 0.75 + uTime * 5.1;
            float wave = sin(ph) * 0.6 + sin(ph * 2.37 + 1.7) * 0.28 + sin(ph * 4.1 + 0.4) * 0.12;
            transformed += uWindDir * (wave * uWindAmp * fw);
            transformed.y += cos(ph * 1.63) * uWindAmp * 0.45 * fw;
          }
        }
      `);
      if (minPx > 0) {
        shader.uniforms.uViewportH = flutterUniforms.uViewportH;
        shader.vertexShader = `
          attribute float aEdge;
          uniform float uViewportH;
        ` + shader.vertexShader.replace('#include <project_vertex>', `
          #include <project_vertex>
          {
            float wClip = max( gl_Position.w, 1e-4 );
            float hNdc  = ( ${worldH.toFixed(4)} * projectionMatrix[1][1] ) / wClip;
            float minNdc = ${minPx.toFixed(3)} * 2.0 / max( uViewportH, 1.0 );
            // Never draw the ribbon more than MINPX_MAX_GROW times its true
            // projected height, whatever the viewport uniform claims.
            float target = min( minNdc, hNdc * ${MINPX_MAX_GROW.toFixed(2)} );
            float grow = max( 0.0, target - hNdc ) * 0.5;
            gl_Position.y += ( aEdge - 0.5 ) * 2.0 * grow * wClip;
          }
        `);
      }
    };
    // The min-width block bakes worldH and minPx into the shader source as
    // literals, so two materials with different values must not share a compiled
    // program. Encode them in the key rather than relying on there only ever
    // being one ribbon type.
    material.customProgramCacheKey = () => (minPx > 0
      ? `descent-flutter-minpx-${minPx.toFixed(3)}-${worldH.toFixed(4)}`
      : 'descent-flutter');
    return material;
  }

  /**
   * Smooth, monotone, C-infinity replacement for clamp(u, 0, 1). Maps the whole
   * real line into (0,1) with unit slope at u = 0.5 and no corner at either end,
   * so a height built through it has no C1 discontinuity to turn into a normal
   * break. This is the specific fix for the tread mosaic: see buildTreadMesh().
   */
  function softClamp01(u) {
    const w = (u - 0.5) * 2;
    return 0.5 + 0.5 * (w / Math.sqrt(1 + w * w));
  }

  function buildTreadMesh(terrain) {
    // 512 -> 1024. The tile is 2.6 m across, so a 512 map is 5.1 mm/texel; at
    // the 1.5 m the near tread is actually presented at that is 6.3 screen px
    // per texel, and the finest thing the map could author (a 4-texel feature)
    // landed at 25 px. Measured pixel-scale/32-px-scale std ratio was 1.43 where
    // dirt reads 3-5 — the grain was not missing, it was authored too coarse to
    // exist. 1024 halves it to 2.5 mm/texel.
    const maps = buildTreadMaps(texRng, 1024);
    const map = reg(canvasTexture(maps.albedo, { srgb: true, repeatX: 1, repeatY: 1, aniso: 16 }));
    const nrm = reg(canvasTexture(maps.normal, { srgb: false, aniso: 16 }));
    const rgh = reg(canvasTexture(maps.rough, { srgb: false, aniso: 8 }));
    treadAlbedoTex = map;

    const mat = reg(new THREE.MeshStandardMaterial({
      map, normalMap: nrm, roughnessMap: rgh,
      // 1.25 -> 1.05: the map now carries twice the high-frequency content, and
      // the normal is the channel that aliases.
      normalScale: new THREE.Vector2(1.05, 1.05),
      roughness: 1.0, metalness: 0.0,
      vertexColors: true,
      transparent: true,
      depthWrite: false,
      // NOTE: polygonOffset is NOT the tread's problem and must stay exactly as
      // it is. The mosaic was never z-fighting — it was a clamped vertex height
      // saturating at both ends against a heightfield sampled below Nyquist.
      polygonOffset: true,
      polygonOffsetFactor: -4,
      polygonOffsetUnits: -4,
      side: THREE.FrontSide,
      dithering: true,
    }));
    mat.name = 'trail-tread';

    // ---------------------------------------------------------------------
    // Distance-graded LOD lift.
    //
    // The terrain renders through a quadtree whose leaf vertex spacing grows
    // with camera distance (49x49 vertices per node, node size ~ dist/2.05, so
    // ~1 m at 100 m and ~2 m at 300 m). The trail is a 2-3 m wide trench carved
    // into that surface, so beyond ~40 m the rendered hillside no longer
    // contains the trench at all and swallows the tread in patches — the three
    // disconnected grey parallelograms in r2_02 and the grey slab in r2_01.
    //
    // That is a HEIGHT error in a surface the tread does not own, not a depth
    // precision error, so no amount of polygon offset can reach it. aLodLift is
    // the measured burial depth per vertex (how far a 4 m-lattice reconstruction
    // of the terrain sits above this vertex) and the vertex shader fades it in
    // between 40 m and 140 m, where the ribbon is thin enough that lifting it
    // by up to ~1 m moves it a handful of pixels.
    // ---------------------------------------------------------------------
    //
    // The same patch also carries the WEAR and SKY-OCCLUSION terms (H2), and
    // knocks the tread's indirect specular down to something a porous soil
    // aggregate actually reflects. See the aWear/aSkyOcc bake in emitStrip().
    mat.onBeforeCompile = (shader) => {
      shader.uniforms.uSpecScale = { value: TREAD_SPEC_SCALE };
      shader.vertexShader = `
        attribute float aLodLift;
        attribute float aWear;
        attribute float aSkyOcc;
        varying float vWear;
        varying float vSkyOcc;
      ` + shader.vertexShader.replace(
        '#include <begin_vertex>',
        `#include <begin_vertex>
        vWear = aWear;
        vSkyOcc = aSkyOcc;
        {
          vec3 wLift = ( modelMatrix * vec4( transformed, 1.0 ) ).xyz;
          float camDist = distance( cameraPosition, wLift );
          transformed.y += aLodLift * smoothstep( 40.0, 140.0, camDist );
        }`);

      shader.fragmentShader = `
        uniform float uSpecScale;
        varying float vWear;
        varying float vSkyOcc;
      ` + shader.fragmentShader
        // --- H2, albedo half: a buffed line has had its texture polished off.
        // Pulling the detail map toward its own mean flattens the ridden
        // centre; pushing away from it roughens the loose shoulders. This is
        // the "lower-contrast over the centre 40%, looser grit toward the
        // edges" half of the wear gradient. The pivot is the map's per-channel
        // MEAN in linear space (0.75 / 0.71 / 0.63, computed in
        // buildTreadMaps) — a scalar pivot would have shifted the buffed
        // centre ~10% towards blue, the exact thing this lane is removing.
        .replace('#include <map_fragment>', `#include <map_fragment>
        diffuseColor.rgb = max( vec3( 0.0 ),
          mix( vec3( 0.75, 0.71, 0.63 ), diffuseColor.rgb, mix( 1.20, 0.60, vWear ) ) );`)
        // --- H2, roughness half: hardpack under a tyre is burnished; the
        // shoulders are dry loose grit. Floor of 0.55 so nothing on the tread
        // can ever go glossy again.
        .replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
        roughnessFactor = clamp( roughnessFactor * mix( 1.05, 0.82, vWear ), 0.55, 1.0 );`)
        // --- the blue tread, part 2. F0 = 0.04 / F90 = 1.0 is a smooth
        // dielectric; dry soil is a porous scattering aggregate whose specular
        // lobe is largely absorbed by its own microstructure. Measured, the
        // untreated tread reflected enough sky to come out BLUER than a white
        // ribbon under the same light — the specular was carrying most of the
        // pixel. 0.35 corresponds to F0 ~0.014 (IOR ~1.26), which is where
        // soil and dust are normally authored.
        .replace('#include <lights_physical_fragment>', `#include <lights_physical_fragment>
        material.specularColor *= uSpecScale;
        material.specularColorBlended *= uSpecScale;
        material.specularF90 *= 0.55;`)
        // --- baked sky occlusion. R10 is explicit that the screen-space AO
        // must NOT be re-raised (it was correctly weakened in r2 to save the
        // shadow floor) and that grounding is to be bought from baked terms per
        // asset. A trail is a trench: its horizon is the cut bank either side,
        // and it has been receiving 100% of the sky's (blue) irradiance while
        // the terrain around it is occluded. This is that missing term, and it
        // attenuates indirect specular as well as indirect diffuse.
        .replace('#include <aomap_fragment>', `{
          float treadAO = clamp( vSkyOcc, 0.0, 1.0 );
          reflectedLight.indirectDiffuse *= treadAO;
          float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
          reflectedLight.indirectSpecular *=
            computeSpecularOcclusion( dotNV, treadAO, material.roughness );
        }`);
    };
    mat.customProgramCacheKey = () => 'descent-tread-wear-v2';

    const LAT = 13;                       // lateral samples across the ribbon
    const coarse = Math.max(1, Math.round(RIBBON_DS / STATION_DS));
    const pos = [], nor = [], uv = [], col = [], idx = [], lift = [];
    const wearA = [], occA = [];          // H2 wear gradient + baked sky occlusion
    const hillN = [];                     // band-limited hillside normal per vertex
    let vcount = 0;

    const tintCache = [];
    for (let s = 0; s < SURF_TINT.length; s++) tintCache.push(new THREE.Color(SURF_TINT[s]));
    const hardpack = new THREE.Color(HARDPACK_TINT);
    const _tintScratch = new THREE.Color();

    // ---------------------------------------------------------------------
    // Baked sky occlusion (the tread's half of R10's "buy grounding from baked
    // terms, not from SSAO").
    //
    // For a horizon at elevation angle e in one azimuth wedge, the fraction of
    // the cosine-weighted hemisphere still open in that wedge is cos²(e) =
    // 1/(1+tan²e). Averaging that over six azimuths against the CARVED
    // heightfield (finalize() runs after terrain.commit(), so the trench is
    // already in it) gives a physically-shaped occlusion for ~18 height samples
    // per vertex: ~0.85-0.95 on an open bench, ~0.45-0.65 inside a 4 m carve.
    // ---------------------------------------------------------------------
    const OCC_DIRS = 6;
    const OCC_CS = new Float64Array(OCC_DIRS * 2);
    for (let d = 0; d < OCC_DIRS; d++) {
      const a = (d / OCC_DIRS) * Math.PI * 2;
      OCC_CS[d * 2] = Math.cos(a);
      OCC_CS[d * 2 + 1] = Math.sin(a);
    }
    const OCC_R = [1.3, 3.4, 8.0];
    // Occlusion is smooth across a 3 m tread, so it is probed at five lateral
    // stations per row and interpolated between them: 90 height samples per
    // row instead of 234, which is the difference between ~900 k and ~300 k
    // probes over the whole 2.6 km ribbon.
    const OCC_K = [0, 3, 6, 9, 12];
    const occRow = new Float64Array(LAT);
    function skyOcclusion(x, z, y) {
      if (!terrain) return 1;
      let sum = 0;
      for (let d = 0; d < OCC_DIRS; d++) {
        const cx = OCC_CS[d * 2], cz = OCC_CS[d * 2 + 1];
        let tanMax = 0;
        for (let k = 0; k < OCC_R.length; k++) {
          const r = OCC_R[k];
          const t = (terrain.sampleHeight(x + cx * r, z + cz * r) - y) / r;
          if (t > tanMax) tanMax = t;
        }
        sum += 1 / (1 + tanMax * tanMax);
      }
      return sum / OCC_DIRS;
    }

    // Shaped features (lips, knuckles, drops) need twice the longitudinal density
    // or the ribbon rounds the takeoff off into a mound.
    const SHAPED = { jump: 1, doubles: 1, gap: 1, stepDown: 1, drop: 1 };
    function stepAt(i) {
      const fi = S.feat[i];
      if (fi >= 0 && SHAPED[features[fi].type]) return 1;
      return coarse;
    }
    function isShaped(i) {
      const fi = S.feat[i];
      return fi >= 0 && SHAPED[features[fi].type] ? 1 : 0;
    }

    /**
     * What a 4 m-lattice mesh would report as the ground height here. Q = 4 m is
     * deliberately coarser than the terrain LOD actually uses inside 300 m, so
     * the lift is conservative — it errs towards floating rather than burying.
     */
    const LODQ = 4.0;
    function coarseHeight(x, z) {
      if (!terrain) return 0;
      const gx = Math.floor(x / LODQ) * LODQ, gz = Math.floor(z / LODQ) * LODQ;
      const fx = (x - gx) / LODQ, fz = (z - gz) / LODQ;
      const h00 = terrain.sampleHeight(gx, gz);
      const h10 = terrain.sampleHeight(gx + LODQ, gz);
      const h01 = terrain.sampleHeight(gx, gz + LODQ);
      const h11 = terrain.sampleHeight(gx + LODQ, gz + LODQ);
      return lerp(lerp(h00, h10, fx), lerp(h01, h11, fx), fz);
    }

    const TREAD_CLEAR = 0.035;            // metres proud of the carved ground
    const TREAD_RIDE = 0.085;             // metres the tread may ride above it

    function emitStrip(sampleFn, i0, i1, widthFn, tintScale, alphaFn) {
      // ---- pass 1: gather the ribbon lattice -----------------------------
      const rowsI = [];
      for (let i = i0; i <= i1; i += stepAt(i)) rowsI.push(i);
      if (rowsI.length < 2) return;
      const R = rowsI.length;
      const NG = R * LAT;
      const gX = new Float64Array(NG), gZ = new Float64Array(NG);
      const gIdeal = new Float64Array(NG), gT = new Float64Array(NG);
      const shaped = new Uint8Array(R);
      const cwArr = new Float64Array(R);
      const dists = new Float64Array(R);

      let dist = 0, prevX = 0, prevZ = 0, first = true;
      for (let r = 0; r < R; r++) {
        const i = rowsI[r];
        shaped[r] = isShaped(i);
        const cw = widthFn(i);
        cwArr[r] = cw;
        const half = cw * 0.5;
        const cx = sampleFn.x(i), cz = sampleFn.z(i);
        if (!first) dist += Math.hypot(cx - prevX, cz - prevZ);
        prevX = cx; prevZ = cz; first = false;
        dists[r] = dist;
        for (let k = 0; k < LAT; k++) {
          const v = (k / (LAT - 1)) * 2 - 1;          // -1..1
          const o = v * half * 1.18;                  // overhang for the fade
          const px = cx + S.rx[i] * o;
          const pz = cz + S.rz[i] * o;
          const g = r * LAT + k;
          gX[g] = px; gZ[g] = pz;
          gIdeal[g] = sampleFn.y(i, o);
          gT[g] = terrain ? terrain.sampleHeight(px, pz) : gIdeal[g];
        }
      }

      // ---- pass 2: band-limit the sampled terrain height -----------------
      // terrain.sampleHeight() blends a 0.35 m detail overlay over the 2.0 m
      // global field, so it carries relief four times finer than the ribbon can
      // represent. Sampling it raw at 0.8 m is what beat the ribbon against the
      // heightfield and produced the straight-sided facets. Two [1,2,1] passes
      // along the ribbon put an exact null at 1.6 m and -6 dB at 3.2 m, which
      // removes everything the ribbon cannot carry and leaves the bench, the
      // rollers and the berms untouched. Shaped rows (lips, knuckles, drops) are
      // emitted at 0.4 m and take one pass only, so a takeoff stays crisp.
      const tmp = new Float64Array(NG);
      const gTraw = Float64Array.from(gT);
      for (let pass = 0; pass < 2; pass++) {
        for (let r = 0; r < R; r++) {
          const rm = r > 0 ? r - 1 : 0, rp = r < R - 1 ? r + 1 : R - 1;
          for (let k = 0; k < LAT; k++) {
            tmp[r * LAT + k] = 0.25 * gT[rm * LAT + k] + 0.5 * gT[r * LAT + k] + 0.25 * gT[rp * LAT + k];
          }
        }
        for (let r = 0; r < R; r++) {
          if (pass > 0 && shaped[r]) continue;
          for (let k = 0; k < LAT; k++) gT[r * LAT + k] = tmp[r * LAT + k];
        }
      }
      // One lateral pass as well: the carve stamps are 0.3-1.2 m discs, so the
      // heightfield is lumpy across the tread too. A symmetric kernel is exact on
      // the bank (linear in o) and costs the crown < 1 mm.
      for (let r = 0; r < R; r++) {
        const b = r * LAT;
        for (let k = 0; k < LAT; k++) {
          const km = k > 0 ? k - 1 : 0, kp = k < LAT - 1 ? k + 1 : LAT - 1;
          tmp[b + k] = 0.25 * gT[b + km] + 0.5 * gT[b + k] + 0.25 * gT[b + kp];
        }
        for (let k = 0; k < LAT; k++) gT[b + k] = tmp[b + k];
      }

      // ---- pass 2b: smooth clearance over the relief we just removed -----
      // The low-pass deliberately deletes relief the RENDERED terrain still has
      // — its near leaves sample at ~0.5 m — so a smoothed ribbon can end up
      // underneath a local bump and let the hillside poke through it, which
      // would look exactly like the mosaic we are removing. Lift the ribbon by
      // the DILATED positive deviation of the fine field over the smoothed one:
      // the max filter guarantees the lift covers the peak, and the [1,2,1]
      // passes after it restore C1 continuity, so this cannot put an edge back.
      const exc = new Float64Array(NG);
      for (let g = 0; g < NG; g++) exc[g] = Math.max(0, gTraw[g] - gT[g]);
      for (let r = 0; r < R; r++) {                       // dilate +/-3 rows
        for (let k = 0; k < LAT; k++) {
          let m = 0;
          for (let o = -3; o <= 3; o++) {
            const rr = clamp(r + o, 0, R - 1);
            const e = exc[rr * LAT + k];
            if (e > m) m = e;
          }
          tmp[r * LAT + k] = m;
        }
      }
      for (let r = 0; r < R; r++) {                       // dilate +/-1 lateral
        const b = r * LAT;
        for (let k = 0; k < LAT; k++) {
          const km = k > 0 ? k - 1 : 0, kp = k < LAT - 1 ? k + 1 : LAT - 1;
          exc[b + k] = Math.max(tmp[b + km], Math.max(tmp[b + k], tmp[b + kp]));
        }
      }
      for (let r = 0; r < R; r++) {                       // re-smooth, and cap
        const rm = r > 0 ? r - 1 : 0, rp = r < R - 1 ? r + 1 : R - 1;
        for (let k = 0; k < LAT; k++) {
          tmp[r * LAT + k] = Math.min(0.20,
            0.25 * exc[rm * LAT + k] + 0.5 * exc[r * LAT + k] + 0.25 * exc[rp * LAT + k]);
        }
      }
      for (let r = 0; r < R; r++) {
        const b = r * LAT;
        for (let k = 0; k < LAT; k++) {
          const km = k > 0 ? k - 1 : 0, kp = k < LAT - 1 ? k + 1 : LAT - 1;
          exc[b + k] = 0.25 * tmp[b + km] + 0.5 * tmp[b + k] + 0.25 * tmp[b + kp];
        }
      }

      // ---- pass 3: solve the tread height, then emit ---------------------
      // The old form was `tH + 0.035 + clamp(ideal - tH, 0, 0.085)`. A clamp
      // saturating at BOTH ends along an irregular contour is a C1 break every
      // time the argument crosses either limit, and computeVertexNormals() turns
      // every one of those into a hard normal edge. softClamp01 has the same
      // envelope (never below the ground, never more than 12 cm above it) with a
      // continuous derivative everywhere, so there is no edge to shade.
      const gY = new Float64Array(NG);
      for (let g = 0; g < NG; g++) {
        const d = gIdeal[g] - gT[g];
        gY[g] = gT[g] + TREAD_CLEAR + exc[g] + TREAD_RIDE * softClamp01(d / TREAD_RIDE);
      }

      for (let r = 0; r < R; r++) {
        const i = rowsI[r];
        const rowBase = vcount;
        const cw = cwArr[r];
        const tint = tintCache[S.surface[i]] || tintCache[0];
        // Macro tonal variation so the ribbon never looks like one flat decal.
        // Two scales: ~20 m of wear banding, plus a ~150 m field that survives
        // the mip chain and stops the far tread reading as one flat grey.
        const s = i * STATION_DS;
        const macro = 0.72
          + 0.24 * (fbm1(microN, s * 0.02 + 900, 3, 2.1, 0.55) * 0.5 + 0.5)
          + 0.13 * (fbm1(microN, s * 0.0045 + 311, 2, 2.0, 0.5) * 0.5 + 0.5)
          // A third scale at ~3.6 m, deliberately close to but not equal to the
          // 3.1 m longitudinal UV tile, so the two beat against each other and
          // the repeat stops being findable down a 2.6 km ribbon.
          + 0.11 * (microN(s * 0.278 + 57) * 0.5 + 0.5);
        const rm = r > 0 ? r - 1 : 0, rp = r < R - 1 ? r + 1 : R - 1;

        // ---- where the ridden line actually is --------------------------
        // Riders do not track the centreline. They ride the outside of a
        // bermed corner and drift back to the middle on the straights, and the
        // line wanders over tens of metres. `wc` is that line, in the same
        // -1..1 lateral coordinate as `v`.
        const hpW = HARDPACK_W[S.surface[i]] !== undefined
          ? HARDPACK_W[S.surface[i]] : HARDPACK_W[0];
        const wc = clamp(-S.curv[i] * 5.0 + edgeN(i * 0.011 + 77) * 0.22, -0.34, 0.34);
        // A technical, rocky or rooty section is not buffed smooth; a flow
        // section is polished to a ribbon.
        const wearGain = 0.55 + 0.45 * (1 - clamp01(PHASES[S.phase[i]].tech));

        // Sparse sky-occlusion probe for this row, then fill by interpolation.
        for (let q = 0; q < OCC_K.length; q++) {
          const kk = OCC_K[q];
          const gg = r * LAT + kk;
          occRow[kk] = terrain ? skyOcclusion(gX[gg], gZ[gg], gY[gg]) : 1;
        }
        for (let q = 0; q + 1 < OCC_K.length; q++) {
          const k0 = OCC_K[q], k1 = OCC_K[q + 1];
          for (let kk = k0 + 1; kk < k1; kk++) {
            occRow[kk] = lerp(occRow[k0], occRow[k1], (kk - k0) / (k1 - k0));
          }
        }
        for (let k = 0; k < LAT; k++) {
          const g = r * LAT + k;
          const v = (k / (LAT - 1)) * 2 - 1;
          const px = gX[g], pz = gZ[g], py = gY[g];
          pos.push(px, py, pz);
          nor.push(0, 1, 0);
          uv.push((v * 0.5 + 0.5) * (cw / 2.6), dists[r] / 3.1);

          // Band-limited hillside normal, taken from the SMOOTHED terrain grid
          // rather than terrain.sampleNormal() — sampleNormal is the analytic
          // gradient of the same 0.35 m field we just low-passed away, so using
          // it would put the noise straight back into the shading.
          const km = k > 0 ? k - 1 : 0, kp = k < LAT - 1 ? k + 1 : LAT - 1;
          const ax = gX[rp * LAT + k] - gX[rm * LAT + k];
          const ay = gT[rp * LAT + k] - gT[rm * LAT + k];
          const az = gZ[rp * LAT + k] - gZ[rm * LAT + k];
          const bx = gX[r * LAT + kp] - gX[r * LAT + km];
          const by = gT[r * LAT + kp] - gT[r * LAT + km];
          const bz = gZ[r * LAT + kp] - gZ[r * LAT + km];
          let nx = by * az - bz * ay;
          let ny = bz * ax - bx * az;
          let nz = bx * ay - by * ax;
          const nl = Math.hypot(nx, ny, nz);
          if (nl > 1e-9) { nx /= nl; ny /= nl; nz /= nl; }
          else { nx = 0; ny = 1; nz = 0; }
          if (ny < 0) { nx = -nx; ny = -ny; nz = -nz; }
          hillN.push(nx, ny, nz);

          // Burial depth against a coarse reconstruction of the terrain.
          lift.push(terrain ? clamp(coarseHeight(px, pz) - py + 0.10, 0, 2.5) : 0);

          // Edge alpha with an irregular, but band-limited, boundary. The old
          // `k * 3.7` term advanced the noise lattice ~3.7 units between adjacent
          // lateral samples, i.e. white noise on the ribbon edge.
          const wob = edgeN(i * 0.05 + v * 0.9) * 0.14;
          const av = Math.abs(v);
          let a = 1 - smoothstep(0.60 + wob, 1.0, av);
          if (alphaFn) a *= alphaFn(i);

          // ---- H2: the wear gradient -----------------------------------
          // 1 on the ridden line, 0 out at the loose shoulders. `dv` is the
          // distance from the WANDERING line, not from the centreline, which
          // is what stops the gradient reading as a painted stripe.
          const dv = Math.abs(v - wc);
          const wear = clamp01((1 - smoothstep(0.16, 0.62, dv)) * wearGain);
          wearA.push(wear);

          // Baked sky occlusion, with a small extra term at the shoulders:
          // there is always duff, litter and trailside growth right at the
          // edge of a tread that the heightfield alone does not know about.
          occA.push(clamp(occRow[k] * (1 - 0.16 * smoothstep(0.35, 1.0, av)), 0.30, 1.0));

          // ---- the warm hardpack base, splatted through the parent ------
          // Blended in LINEAR space (THREE.Color is already in working space),
          // which is the physically right way to mix two materials.
          _tintScratch.copy(tint).lerp(hardpack, hpW * (0.26 + 0.74 * wear));
          // The ridden line is darker (compacted, fines driven in); the
          // shoulders are lighter (dry loose grit sitting proud).
          const shade = macro * (1.06 - 0.24 * wear) * tintScale;
          col.push(_tintScratch.r * shade, _tintScratch.g * shade, _tintScratch.b * shade, a);
          vcount++;
        }
        if (r + 1 < R) {
          // WINDING (P0, and the reason the tread never appeared in any review
          // frame): rows advance downhill and k advances to the rider's right, so
          // the old order (a+k, b+k, a+k+1) is forward x right = DOWN. Every
          // triangle in the ribbon faced the ground. With side: FrontSide that is
          // a back face from any camera above the surface, so the whole mesh was
          // culled — measured 101,064 of 101,064 triangles facing down, and
          // computeVertexNormals() returning a mean normal.y of -0.95. What the
          // review shots show as "the tread" is the terrain's carved corridor;
          // the tread material genuinely was not being drawn anywhere, which is
          // exactly the "untextured flat mid-grey / missing material" signal.
          const a = rowBase, b = rowBase + LAT;
          for (let k = 0; k + 1 < LAT; k++) {
            idx.push(a + k, a + k + 1, b + k);
            idx.push(a + k + 1, b + k + 1, b + k);
          }
        }
      }
    }

    // Main line.
    emitStrip({
      x: (i) => S.px[i], z: (i) => S.pz[i],
      y: (i, o) => surfaceHeightRender(i, o),
    }, 0, S.n - 1, (i) => S.width[i], 1.0);

    // Split branches.
    for (const sp of splits) {
      const bw = 1.75;
      emitStrip({
        x: (i) => S.px[i] + S.rx[i] * branchOffset(sp, i),
        z: (i) => S.pz[i] + S.rz[i] * branchOffset(sp, i),
        // ONE height rule, shared with the carve — see branchBaseY(). The ribbon
        // used to carry its own copy (`min(py + 0.05, natural + 0.02)`), so the
        // decal and the ground it is laid on were solving different equations
        // and the branch read as floating or buried wherever they diverged.
        y: (i, o) => branchBaseY(sp, i, terrain)
          + crossProfileRender(i, clamp(o / (bw * 0.5), -1.4, 1.4)) * 0.8,
      }, sp.i0, sp.i1, () => bw, 0.94, (i) => {
        // Fade the branch out where the two lines merge, so the braid reads as one
        // tread splitting rather than two decals stacked on top of each other.
        const f = (i - sp.i0) / Math.max(1, sp.i1 - sp.i0);
        return smoothstep(0.02, 0.22, f) * smoothstep(0.98, 0.78, f);
      });
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
    geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
    geo.setAttribute('color', new THREE.Float32BufferAttribute(col, 4));
    geo.setAttribute('aLodLift', new THREE.Float32BufferAttribute(lift, 1));
    geo.setAttribute('aWear', new THREE.Float32BufferAttribute(wearA, 1));
    geo.setAttribute('aSkyOcc', new THREE.Float32BufferAttribute(occA, 1));
    geo.setIndex(idx);
    geo.computeVertexNormals();

    // Blend the tread's shading normal half-way to the hillside it sits on. The
    // ribbon is a 3 m wide strip laid over a mountainside: whatever its own
    // cross-section does, it must not disagree with the slope around it, or it
    // reads as a separate flat object stuck to the hill (the cold blue-grey
    // slabs in r2_02, which are the ribbon shading off-sun while the terrain
    // either side of it shades on-sun).
    {
      const na = geo.attributes.normal.array;
      const HILL_W = 0.5;
      for (let v3 = 0; v3 < na.length; v3 += 3) {
        const nx = na[v3] * (1 - HILL_W) + hillN[v3] * HILL_W;
        const ny = na[v3 + 1] * (1 - HILL_W) + hillN[v3 + 1] * HILL_W;
        const nz = na[v3 + 2] * (1 - HILL_W) + hillN[v3 + 2] * HILL_W;
        const l = Math.hypot(nx, ny, nz) || 1;
        na[v3] = nx / l; na[v3 + 1] = ny / l; na[v3 + 2] = nz / l;
      }
      geo.attributes.normal.needsUpdate = true;
    }

    geo.computeBoundingSphere();
    // The vertex shader lifts far vertices by up to 2.5 m; grow the culling
    // sphere so a lifted section can never be frustum-rejected while on screen.
    if (geo.boundingSphere) geo.boundingSphere.radius += 2.5;
    reg(geo);

    treadMesh = new THREE.Mesh(geo, mat);
    treadMesh.name = 'trail-tread';
    treadMesh.receiveShadow = true;
    treadMesh.castShadow = false;
    treadMesh.renderOrder = 1;
    treadMesh.matrixAutoUpdate = false;
    treadMesh.updateMatrix();
    group.add(treadMesh);
  }

  /**
   * Course furniture: tape on stakes down the fast sections, arrow signage at the
   * junctions, checkpoint gates, the start gate and the finish arch.
   */
  function buildCourseFurniture(terrain) {
    const woodMaps = buildWoodMaps(texRng, 256);
    const woodMap = reg(canvasTexture(woodMaps.albedo, { srgb: true, repeatY: 1 }));
    const woodNrm = reg(canvasTexture(woodMaps.normal, { srgb: false }));
    const woodMat = reg(new THREE.MeshStandardMaterial({
      map: woodMap, normalMap: woodNrm, roughness: 0.93, metalness: 0.0,
      vertexColors: true,
    }));
    woodMat.name = 'trail-wood';

    const metalMat = reg(new THREE.MeshStandardMaterial({
      color: new THREE.Color(0xa6acb2), roughness: 0.38, metalness: 0.85,
      vertexColors: true,
    }));
    metalMat.name = 'trail-metal';

    // ---------------------------------------------------------------------
    // H1. The black bars.
    //
    // Round 3 read the tape posts as unlit black rectangles and blamed
    // MeshBasicMaterial; the triage corrected that to "a dark metal at
    // metalness 0.85 with vertex colour 0.22, which has essentially no diffuse
    // term and therefore no terminator". Both are close, and the real cause is
    // narrower and worse: the tape posts are an InstancedMesh over `gStake`,
    // and `gStake` is a bare CylinderGeometry with NO `color` attribute — but
    // it was being drawn with `woodMat`, which sets `vertexColors: true`.
    // three emits USE_COLOR straight off the material (WebGLPrograms never
    // consults the geometry), so the shader runs `vColor.rgb *= color` against
    // an unbound attribute, WebGL supplies the generic default (0,0,0,1), and
    // every stake multiplies its albedo by zero. Instanced colour is applied
    // after that, so it could not rescue it either. The posts were not dark
    // metal; they were albedo ZERO, which is why they have no terminator, no
    // rim, and no response to the sun anywhere in sixteen frames.
    //
    // They now get their own dielectric: metalness 0, roughness 0.72, NO
    // vertexColors (instance colour alone does the tinting), tinted to a
    // creosoted course stake at ~0.06-0.09 linear — the albedo band the work
    // order asks for, and also what a real weathered course pin measures.
    const stakeMat = reg(new THREE.MeshStandardMaterial({
      map: woodMap, normalMap: woodNrm,
      color: new THREE.Color(0xffffff),
      roughness: 0.72, metalness: 0.0,
      vertexColors: false,
    }));
    stakeMat.name = 'trail-stake';

    // Dielectric for the dark course furniture that was going into metalMat at
    // vertex colour 0.22 (timing-beam housings, gantry feet). Painted steel is
    // still a coated dielectric as far as its diffuse term is concerned.
    const postMat = reg(new THREE.MeshStandardMaterial({
      color: new THREE.Color(0xffffff), roughness: 0.70, metalness: 0.0,
      vertexColors: true,
    }));
    postMat.name = 'trail-post';

    // Loose soil for the contact mounds at the foot of every planted object.
    // A post standing on a flat cut ellipse is the single clearest "dropped in,
    // not planted" tell there is, and R10 rules out buying it back from SSAO.
    const soilMat = reg(new THREE.MeshStandardMaterial({
      map: treadAlbedoTex, color: new THREE.Color(0xffffff),
      roughness: 1.0, metalness: 0.0,
      vertexColors: true,
      transparent: true, depthWrite: false,
      polygonOffset: true, polygonOffsetFactor: -3, polygonOffsetUnits: -3,
      side: THREE.FrontSide, dithering: true,
    }));
    soilMat.name = 'trail-soil';

    // Inflatable arch / sandbags / soft goods. This used to be an untextured
    // white MeshStandardMaterial, which at the review exposure turned every
    // ballast bag into a blown-out faceted white box (r2_08). Woven
    // polypropylene, procedurally generated like everything else.
    const fabMaps = buildFabricMaps(texRng, 256);
    const fabMap = reg(canvasTexture(fabMaps.albedo, { srgb: true, repeatX: 2, repeatY: 2 }));
    const fabNrm = reg(canvasTexture(fabMaps.normal, { srgb: false, repeatX: 2, repeatY: 2 }));
    const fabricMat = reg(new THREE.MeshStandardMaterial({
      map: fabMap, normalMap: fabNrm,
      normalScale: new THREE.Vector2(0.8, 0.8),
      color: new THREE.Color(0xffffff), roughness: 0.86, metalness: 0.0,
      vertexColors: true,
    }));
    fabricMat.name = 'trail-fabric';

    const atlas = reg(canvasTexture(buildBannerAtlas(256), { srgb: true, aniso: 8 }));
    atlas.wrapS = atlas.wrapT = THREE.ClampToEdgeWrapping;
    const bannerMat = reg(applyFlutter(new THREE.MeshStandardMaterial({
      map: atlas, roughness: 0.76, metalness: 0.0,
      side: THREE.DoubleSide, vertexColors: true,
      transparent: false, alphaTest: 0.02,
    }), 0.055));
    bannerMat.name = 'trail-banner';

    // THE WHITE IS UNDER THE BLOOM THRESHOLD, deliberately. ADDENDUM C: postfx
    // blooms above scene luminance ~0.85, and warns against blowing out large
    // white surfaces. A lambertian albedo `a` under the sun (DirectionalLight
    // intensity 3.4) leaves radiance a/pi * 3.4 * NdotL, so 0xf2f3f1 — linear
    // 0.888 — peaks at 0.96 and blooms wherever the ribbon faces the sun at
    // NdotL > 0.885. Three kilometres of ribbon pinned to a minimum screen width
    // and carrying a constant world-space normal meant a large fraction of it
    // did exactly that at once, which is the "vast pale sheet" the establishing
    // shot was measured on. 0xdcdcd6 is linear 0.712 and peaks at 0.770: still
    // unmistakably white tape against 0.2-albedo hardpack, and it cannot bloom
    // at any angle. The two geometric halves of the same defect are the widening
    // bound in applyFlutter() and the real ribbon normals below.
    const tapeTex = reg(canvasTexture(buildTapeTexture('#dcdcd6', '#e0452c'),
      { srgb: true, aniso: 8 }));
    // H4. `transparent: true` TOGETHER WITH `alphaTest` is a three.js foot-gun:
    // it puts a fully alpha-tested surface into the depth-sorted transparent
    // pass, where it renders after the opaque geometry, sorts per-object against
    // a ribbon that spans hundreds of metres, and gains nothing at all — the
    // alphaTest has already resolved every fragment to opaque or discarded.
    // transparent:false puts it back in the opaque pass where it belongs.
    // (Round 3's "tape mips to clipped white 255,255,255" is a separate claim
    // and was measured wrong — max 208-211, and near tape in shade is 45/55/57.
    // What is actually happening is a sub-pixel ribbon losing its red stripe to
    // aliasing, which is what the minimum-screen-width term below fixes.)
    const tapeMat = reg(applyFlutter(new THREE.MeshStandardMaterial({
      map: tapeTex, roughness: 0.52, metalness: 0.0,
      side: THREE.DoubleSide, vertexColors: true,
      transparent: false, alphaTest: 0.35, depthWrite: true,
    }), 0.10, { minPx: 1.7, worldH: TAPE_H }));
    tapeMat.name = 'trail-tape';

    const wood = newBuilder({ color: true });
    const metal = newBuilder({ color: true });
    const post = newBuilder({ color: true });
    const soil = newBuilder({ color: true });
    const fabric = newBuilder({ color: true });
    const banners = newBuilder({ color: true, flutter: true });
    const tape = newBuilder({ color: true, flutter: true, edge: true });

    const WOOD_RGBA = [0.92, 0.88, 0.82, 1];
    const WHITE = [1, 1, 1, 1];

    // --- shared primitives (disposed at the end of this function) ----------
    // 26-38 mm was a real course pin, and at the 20-40 m a sign or a tape line is
    // actually seen from it is 1-2 px wide and disappears entirely — which is why
    // the tape and the sign plate in r2_04 read as floating with nothing under
    // them. 38-52 mm is a split-timber stake: still slender, but it resolves.
    const gStake = new THREE.CylinderGeometry(0.038, 0.052, 1, 7, 1);
    gStake.translate(0, 0.5, 0);
    const gPost = new THREE.CylinderGeometry(0.055, 0.075, 1, 9, 1);
    gPost.translate(0, 0.5, 0);
    const gTube = new THREE.CylinderGeometry(0.05, 0.05, 1, 8, 1);
    gTube.translate(0, 0.5, 0);
    const gPlane = new THREE.PlaneGeometry(1, 1, 1, 1);
    const gBox = new THREE.BoxGeometry(1, 1, 1);
    // A ballast bag is a slumped sack, not a cube. A unit sphere squashed by the
    // placement scale gives a real shading gradient and silhouette for 96 tris.
    const gBag = new THREE.SphereGeometry(0.5, 12, 7);

    /**
     * H5. A hanging banner with a real catenary baked in.
     *
     * Every banner on the course was a 1x1 PlaneGeometry — a rigid rectangle
     * with a ruled bottom edge, which is the single most obvious thing wrong
     * with a piece of printed PVC hung between two points. `sag` is in LOCAL
     * units, so a 0.10 sag on a panel scaled to 0.88 m tall drops 88 mm at
     * mid-span. The free lower edge sags further than the laced top edge and
     * the whole panel bellies out of plane, which is what gives the print a
     * curved terminator instead of one flat value.
     */
    const saggedCache = new Map();
    function saggedPanel(sag) {
      const key = sag.toFixed(3);
      const hit = saggedCache.get(key);
      if (hit) return hit;
      const g = new THREE.PlaneGeometry(1, 1, 16, 4);
      const p = g.attributes.position;
      for (let k = 0; k < p.count; k++) {
        const lx = p.getX(k), ly = p.getY(k);
        const bow = Math.max(0, 1 - 4 * lx * lx);          // 1 mid-span, 0 at the ties
        const free = 1 + (0.5 - ly) * 0.7;                 // lower edge hangs further
        p.setY(k, ly - sag * bow * free);
        p.setZ(k, p.getZ(k) + sag * bow * 0.35);
      }
      p.needsUpdate = true;
      g.computeVertexNormals();
      saggedCache.set(key, g);
      return g;
    }

    const groundY = (x, z) => (terrain ? terrain.sampleHeight(x, z) : 0);
    // ADDENDUM §A forbids raycasting terrain.group (its chunk meshes toggle
    // `visible` with the LOD cut, so a hit depends on where the camera is).
    // sampleNormal() IS the analytic answer to a downward raycast, so that is
    // what "ground-project every instance" means here.
    const _gn = new THREE.Vector3(0, 1, 0);
    const groundN = (x, z) => {
      if (terrain && terrain.sampleNormal) terrain.sampleNormal(x, z, _gn);
      else _gn.set(0, 1, 0);
      return _gn;
    };
    /**
     * Ground height over a post's whole FOOTPRINT, not just its axis. The carve
     * builds a fill shoulder on the outboard side of a bench, so a post planted
     * at the single sample under its centre can have the rim of its base 0.7 m
     * inside that shoulder — measured at the start gate before this was added.
     */
    const groundYFoot = (i, x, z, r) => {
      let h = groundY(x, z);
      const hl = Math.max(1e-6, Math.hypot(S.tx[i], S.tz[i]));
      for (const s of [-1, 1]) {
        h = Math.max(h, groundY(x + S.rx[i] * s * r, z + S.rz[i] * s * r));
        h = Math.max(h, groundY(x + (S.tx[i] / hl) * s * r, z + (S.tz[i] / hl) * s * r));
      }
      return h;
    };

    // ---------------------------------------------------------------------
    // H3. Where a prop's base actually has to be.
    //
    // groundYFoot() takes the MAXIMUM over the footprint, which is right for
    // deciding how TALL something must be so its head rail clears the bank. It
    // is exactly wrong for deciding where its FOOT goes: the r3_04 gantry pole
    // ends in mid air with rock visible under the cut, and the black post beside
    // it floats, because the base was seated on the highest sample nearby and
    // then sunk by only 12 cm.
    //
    // Worse, terrain.sampleHeight() is the 0.35 m detail field, but the terrain
    // RENDERS through a quadtree whose leaf spacing grows with camera distance —
    // the same mismatch the tread's aLodLift attribute exists to absorb. At the
    // 30-60 m a gantry is seen from, the drawn hillside can sit up to ~1 m below
    // the height the props were seated against. So the seat is the MINIMUM over
    // the footprint of both the fine field AND a 4 m-lattice reconstruction of
    // it, and every post extends POST_BURY below that.
    const PROP_LODQ = 4.0;
    const coarseGroundY = (x, z) => {
      if (!terrain) return 0;
      const gx = Math.floor(x / PROP_LODQ) * PROP_LODQ, gz = Math.floor(z / PROP_LODQ) * PROP_LODQ;
      const fx = (x - gx) / PROP_LODQ, fz = (z - gz) / PROP_LODQ;
      return lerp(
        lerp(groundY(gx, gz), groundY(gx + PROP_LODQ, gz), fx),
        lerp(groundY(gx, gz + PROP_LODQ), groundY(gx + PROP_LODQ, gz + PROP_LODQ), fx), fz);
    };
    const POST_BURY = 0.30;                 // m of post below the seat
    const groundYSeat = (i, x, z, r) => {
      let h = Math.min(groundY(x, z), coarseGroundY(x, z));
      const hl = Math.max(1e-6, Math.hypot(S.tx[i], S.tz[i]));
      for (const s of [-1, 1]) {
        const ax = x + S.rx[i] * s * r, az = z + S.rz[i] * s * r;
        const bx = x + (S.tx[i] / hl) * s * r, bz = z + (S.tz[i] / hl) * s * r;
        h = Math.min(h, groundY(ax, az), coarseGroundY(ax, az));
        h = Math.min(h, groundY(bx, bz), coarseGroundY(bx, bz));
      }
      return h;
    };

    /**
     * Contact mound + darkening skirt at the foot of a planted object. Nothing
     * in the build is grounded (round 3, honest verdict, item 3): no contact
     * shadow, no soil disturbance, no litter mound anywhere in sixteen frames,
     * and the screen-space AO cannot be turned back up without re-opening the
     * r2 shadow-floor P0. This is the per-asset baked substitute — a low cone
     * of disturbed soil with a vertex-alpha skirt that fades into the ground.
     * ~16 triangles, merged into one draw call for the whole course.
     */
    function contactMound(i, x, z, radius, rise, tone) {
      if (!terrain) return;
      const SEG = 8;
      const base = soil.vcount;
      const gy = groundY(x, z);
      soil.pos.push(x, gy + rise, z);
      soil.nor.push(0, 1, 0);
      soil.uv.push(0.5, 0.5);
      soil.col.push(tone * 0.62, tone * 0.55, tone * 0.44, 0.92);
      soil.vcount++;
      for (let ring = 0; ring < 2; ring++) {
        const rr = radius * (ring ? 1.0 : 0.45);
        const hh = ring ? 0.012 : rise * 0.5;
        const al = ring ? 0.0 : 0.62;
        for (let k = 0; k < SEG; k++) {
          const a = (k / SEG) * Math.PI * 2;
          const wob = 0.72 + 0.56 * edgeN(x * 0.7 + z * 0.3 + k * 1.7 + ring * 11);
          const ex = x + Math.cos(a) * rr * wob, ez = z + Math.sin(a) * rr * wob;
          const n = groundN(ex, ez);
          soil.pos.push(ex, groundY(ex, ez) + hh, ez);
          soil.nor.push(n.x, n.y, n.z);
          soil.uv.push(0.5 + Math.cos(a) * 0.5, 0.5 + Math.sin(a) * 0.5);
          soil.col.push(tone * 0.55, tone * 0.48, tone * 0.38, al);
          soil.vcount++;
        }
      }
      // Winding: +Y up. cross(a1-c, a0-c).y = r² sin(dθ) > 0, so the cap is
      // (centre, a1, a0) and the skirt band is (a0, a1, b1) / (a0, b1, b0).
      for (let k = 0; k < SEG; k++) {
        const a0 = base + 1 + k, a1 = base + 1 + ((k + 1) % SEG);
        const b0 = base + 1 + SEG + k, b1 = base + 1 + SEG + ((k + 1) % SEG);
        soil.idx.push(base, a1, a0);
        soil.idx.push(a0, a1, b1, a0, b1, b0);
      }
    }

    // --- orientation helpers ------------------------------------------------
    // Objects are placed with explicit orthonormal bases rather than Euler angles,
    // so "facing back up the trail" is unambiguous.
    const _ax = new THREE.Vector3();
    const _ay = new THREE.Vector3(0, 1, 0);
    const _az = new THREE.Vector3();
    const _mb = new THREE.Matrix4();

    /** Upright: local +X = rider's right, +Y = up, +Z = back up the trail. */
    function qUpright(i, outQ) {
      _ax.set(S.rx[i], 0, S.rz[i]).normalize();
      const hl = Math.max(1e-6, Math.hypot(S.tx[i], S.tz[i]));
      _az.set(-S.tx[i] / hl, 0, -S.tz[i] / hl);
      _mb.makeBasis(_ax, _ay, _az);
      return outQ.setFromRotationMatrix(_mb);
    }
    /** Rail: local +Y = rider's right (so a Y-cylinder spans the trail). */
    function qRail(i, outQ) {
      _ax.set(0, 1, 0);
      const hl = Math.max(1e-6, Math.hypot(S.tx[i], S.tz[i]));
      _ay.set(S.rx[i], 0, S.rz[i]).normalize();
      _az.set(S.tx[i] / hl, 0, S.tz[i] / hl);
      _mb.makeBasis(_ax, _ay, _az);
      _ay.set(0, 1, 0);
      return outQ.setFromRotationMatrix(_mb);
    }
    // (qFlat, the world-Y-up "lie on the ground" basis, is gone: every user of it
    // was a rigid slab that half-sank on a cross-slope. Use qFlatN for small
    // objects and fitGroundPlane for slabs.)
    /**
     * Flat on the ground and TILTED INTO IT: local +X = right, +Y = along the
     * trail, +Z = the terrain normal. Anything that lies on the hillside (decks,
     * ballast bags, painted lines) must use this, not qFlat — a world-Y-up box on
     * a 20% slope buries one end and floats the other, which is the "half-sunk
     * untextured box" in r2_08.
     */
    const _t3 = new THREE.Vector3();
    /**
     * Least-squares plane through the four corners of a rigid ground-lying slab
     * (deck, platform) of half-extents ex across the trail and ez along it.
     * Writes the basis into outQ (+X across, +Y along, +Z the plane normal) and
     * returns the plane's height at the centre. A slab this big cannot use the
     * point normal at its centre: on the 11% start chute, using the tread's own
     * normal for a 3.0 x 2.4 m deck still drives a corner 0.7 m into the cut bank.
     */
    function fitGroundPlane(i, cx, cz, ex, ez, outQ) {
      const rx = S.rx[i], rz = S.rz[i];
      const hl = Math.max(1e-6, Math.hypot(S.tx[i], S.tz[i]));
      const tx = S.tx[i] / hl, tz = S.tz[i] / hl;
      const h = [];
      for (const a of [-1, 1]) {
        for (const b of [-1, 1]) {
          h.push(groundY(cx + rx * a * ex + tx * b * ez,
            cz + rz * a * ex + tz * b * ez));
        }
      }
      const [hmm, hmp, hpm, hpp] = h;                 // (a,b) = (-,-) (-,+) (+,-) (+,+)
      const gAcross = ((hpm + hpp) - (hmm + hmp)) / (4 * ex);
      const gAlong = ((hmp + hpp) - (hmm + hpm)) / (4 * ez);
      // Surface y = h(a,b) => normal ∝ (-dh/da, 1, -dh/db) in the (right, up, along) frame.
      _az.set(
        -gAcross * rx - gAlong * tx,
        1,
        -gAcross * rz - gAlong * tz).normalize();
      _ax.set(rx, 0, rz);
      _ax.addScaledVector(_az, -_ax.dot(_az));
      if (_ax.lengthSq() < 1e-8) _ax.set(1, 0, 0);
      _ax.normalize();
      _t3.crossVectors(_az, _ax);
      _mb.makeBasis(_ax, _t3, _az);
      outQ.setFromRotationMatrix(_mb);
      return (hmm + hmp + hpm + hpp) * 0.25;
    }
    function qFlatN(i, x, z, outQ) {
      _az.copy(groundN(x, z)).normalize();
      _ax.set(S.rx[i], 0, S.rz[i]);
      _ax.addScaledVector(_az, -_ax.dot(_az));            // Gram-Schmidt
      if (_ax.lengthSq() < 1e-8) _ax.set(1, 0, 0);
      _ax.normalize();
      _t3.crossVectors(_az, _ax);
      _mb.makeBasis(_ax, _t3, _az);
      return outQ.setFromRotationMatrix(_mb);
    }

    // Instanced stakes: collect transforms, build one InstancedMesh at the end.
    const stakeM = [];
    const stakeC = [];
    const _euler = new THREE.Euler();

    /**
     * Plant a tape stake. Returns the world Y its base was seated at, so the
     * tape ribbon hangs off the same number the post was built from.
     * H1: sunk 9 cm below a seat that already errs low against the rendered
     * (LOD-coarsened) hillside, plus a mound of disturbed soil, because a stake
     * standing on a flat cut ellipse is not planted, it is placed.
     */
    function placeStake(i, x, z, h, lx, ly, lz, tint) {
      const y = groundYSeat(i, x, z, 0.06);
      _euler.set(lx, ly, lz, 'ZXY');
      _quat.setFromEuler(_euler);
      _m4.compose(_p.set(x, y - 0.09, z), _quat, _q.set(1, h + 0.09, 1));
      stakeM.push(_m4.clone());
      stakeC.push(tint);
      contactMound(i, x, z, 0.17 + dressRng() * 0.07, 0.035, 0.85 + dressRng() * 0.25);
      return y;
    }

    // ======================= course tape ===================================
    const tapeSpans = [];
    {
      let cur = null;
      for (let i = 0; i < S.n; i++) {
        // Phase script, OR the exposure rule forcing tape onto a stretch that a
        // raw tech phase would otherwise leave unmarked. A real course marshal
        // tapes an exposed traverse whatever the terrain is made of; S.taped is
        // only set where the drop outside the tread exceeded EXPO_TAPE, and the
        // span builder still discards anything shorter than 30 m, so this cannot
        // quietly tape the whole of the roots or the rock garden.
        const taped = PHASES[S.phase[i]].tape > 0.5 || S.taped[i] === 1;
        if (taped && !cur) cur = { i0: i, i1: i };
        else if (taped && cur) cur.i1 = i;
        else if (cur) { if (cur.i1 - cur.i0 > M(30)) tapeSpans.push(cur); cur = null; }
      }
      if (cur && cur.i1 - cur.i0 > M(30)) tapeSpans.push(cur);
    }

    // The tape used to be a straight chord between posts 6.2 m apart. On this
    // course the switchbacks run to a 6.5 m radius, where a 6.2 m chord cuts
    // ~0.75 m inside the arc — most of a 2.6 m tread. That is why the tape
    // crosses the racing line at chest height in r2_03, r2_04 and r2_08, and any
    // rider would be clotheslined by it. Both the post spacing and the ribbon now
    // follow the OFFSET CENTRELINE, so the tape is parallel to the trail edge
    // everywhere by construction and can never enter the corridor.
    const TAPE_CLEAR = 0.34;     // metres of daylight the sag must keep
    const _tp = new THREE.Vector3();

    /**
     * Clearance of a candidate tape point from the nearest part of the WHOLE
     * course, not just the station it was offset from. On a hairpin the two limbs
     * sit closer together than 2 x (halfWidth + 0.85), so tape hung on the
     * outside of one limb lands squarely on the other limb's tread.
     */
    const courseClearance = (x, z, y) => {
      _tp.set(x, y, z);
      const nt = nearestT(_tp);
      const idx = clamp(Math.round(nt.t * (S.n - 1)), 0, S.n - 1);
      return Math.abs(nt.lateral) - S.width[idx] * 0.5;
    };

    for (const span of tapeSpans) {
      for (const side of [-1, 1]) {
        // Lateral offset of the tape line. Capped inside the local turn radius so
        // the offset polyline can never fold back on itself on the inside of a
        // turn, then pulled in towards the trail edge if the nominal offset would
        // land it on another limb of the course. Memoised: this runs a global
        // spatial query and the ribbon asks for the same stations repeatedly.
        const offCache = new Map();
        const offAt = (i) => {
          const hit = offCache.get(i);
          if (hit !== undefined) return hit;
          const half = S.width[i] * 0.5;
          const inside = (Math.sign(S.curv[i]) || 1) === side;
          const cap = inside ? Math.max(0.9, S.radius[i] * 0.8) : Infinity;
          let o = side * Math.min(half + 0.26, cap);
          for (const extra of [0.85, 0.62, 0.42]) {
            const cand = side * Math.min(half + extra, cap);
            const x = S.px[i] + S.rx[i] * cand, z = S.pz[i] + S.rz[i] * cand;
            if (courseClearance(x, z, S.py[i]) > 0.20) { o = cand; break; }
          }
          offCache.set(i, o);
          return o;
        };
        /** Can this side be taped at all here? False on a hairpin whose two limbs
         *  are closer together than the tape needs — there the honest answer is
         *  no tape, not tape across someone's racing line. */
        const okAt = (i) => {
          const o = offAt(i);
          return courseClearance(S.px[i] + S.rx[i] * o, S.pz[i] + S.rz[i] * o, S.py[i]) > 0.15;
        };

        // Split the span wherever the course doubles back on itself.
        const runs = [];
        {
          const stride = M(1.5);
          let cur = null;
          for (let i = span.i0; i <= span.i1; i += stride) {
            if (okAt(i)) { if (!cur) cur = { i0: i, i1: i }; else cur.i1 = i; }
            else if (cur) { if (cur.i1 - cur.i0 > M(12)) runs.push(cur); cur = null; }
          }
          if (cur && cur.i1 - cur.i0 > M(12)) runs.push(cur);
        }

        for (const run of runs) {
        // Post stations: 3 m through the tightest corners, 6 m on the straights.
        const postIdx = [];
        for (let i = run.i0; i <= run.i1;) {
          postIdx.push(i);
          i += M(clamp(S.radius[i] * 0.55, 3.0, 6.0));
        }
        if (postIdx[postIdx.length - 1] !== run.i1) postIdx.push(run.i1);
        if (postIdx.length < 2) continue;

        // Extra stakes wherever the ground falls away under a span. A fixed
        // spacing hangs the tape across every gulley on the course with nothing
        // holding it up — which is the tape floating in mid-air in r2_04. A real
        // course marshal puts another stake in the dip; so do we. Iterated, so a
        // deep gulley gets several.
        for (let pass = 0; pass < 4; pass++) {
          let inserted = false;
          for (let p = 0; p + 1 < postIdx.length; p++) {
            const iA = postIdx[p], iB = postIdx[p + 1];
            if (iB - iA < M(2.0)) continue;
            const iM = (iA + iB) >> 1;
            const oA = offAt(iA), oB = offAt(iB), oM = offAt(iM);
            const yA = groundY(S.px[iA] + S.rx[iA] * oA, S.pz[iA] + S.rz[iA] * oA);
            const yB = groundY(S.px[iB] + S.rx[iB] * oB, S.pz[iB] + S.rz[iB] * oB);
            const yM = groundY(S.px[iM] + S.rx[iM] * oM, S.pz[iM] + S.rz[iM] * oM);
            if ((yA + yB) * 0.5 - yM > 0.55) {
              postIdx.splice(p + 1, 0, iM);
              inserted = true;
              p++;
            }
          }
          if (!inserted) break;
        }

        const postY = [];
        for (const i of postIdx) {
          const o = offAt(i);
          const x = S.px[i] + S.rx[i] * o;
          const z = S.pz[i] + S.rz[i] * o;
          const h = 1.02 + dressRng() * 0.18;
          // H5: per-post lean widened from ±3.7° to ±7°. Every post on a real
          // course has been leaned by somebody's shoulder or a winter of frost.
          const baseY = placeStake(i, x, z, h,
            (dressRng() - 0.5) * 0.25, dressRng() * Math.PI, (dressRng() - 0.5) * 0.25,
            0.80 + dressRng() * 0.34);
          postY.push(baseY + h * (0.83 + dressRng() * 0.09));
        }

        let dist = 0;
        const rows = [];
        let prevX = 0, prevZ = 0, firstRow = true;
        for (let p = 0; p + 1 < postIdx.length; p++) {
          const iA = postIdx[p], iB = postIdx[p + 1];
          const seg = Math.max(1, iB - iA);
          const spanLen = seg * STATION_DS;
          // Parabolic (small-sag) solution of the catenary between the two post
          // tops, then floored against the ground so the belly of a span crossing
          // a gulley can never dip into the hillside.
          //
          // H5: the sag was a pure function of span length, so every span
          // between two equally-spaced posts sagged by exactly the same amount
          // and the tape read as an identical mechanical zigzag from one end of
          // the course to the other. A marshal pulls each span by hand: some are
          // drum tight, some are slack. 0.55-1.75x, plus a lateral swing so the
          // ribbon does not hang on a perfectly straight plan-view chord, plus a
          // slow twist about its own axis.
          const sagK = 0.55 + dressRng() * 1.20;
          const sagMax = (0.035 + spanLen * 0.016) * sagK;
          const swing = (dressRng() - 0.5) * 0.16 * Math.min(1, spanLen / 4);
          const twist = (dressRng() - 0.5) * 0.9;
          const subs = Math.max(2, Math.round(spanLen / 0.7));
          for (let s = 0; s <= subs; s++) {
            if (p > 0 && s === 0) continue;              // shared vertex row
            const f = s / subs;
            const ii = clamp(Math.round(iA + seg * f), 0, S.n - 1);
            const o = offAt(ii);
            const bow = 4 * f * (1 - f);
            const x = S.px[ii] + S.rx[ii] * (o + swing * bow);
            const z = S.pz[ii] + S.rz[ii] * (o + swing * bow);
            if (!firstRow) dist += Math.hypot(x - prevX, z - prevZ);
            prevX = x; prevZ = z; firstRow = false;
            const chord = lerp(postY[p], postY[p + 1], f);
            const sag = sagMax * bow;
            const y = Math.max(chord - sag, groundY(x, z) + TAPE_CLEAR);
            const row = tape.vcount;
            const flut = Math.sin(f * Math.PI);
            // Twist: the ribbon rolls about its own long axis mid-span, so the
            // top edge leans out and the printed face turns towards the light.
            const tw = twist * bow;
            const twX = S.rx[ii] * TAPE_H * Math.sin(tw);
            const twZ = S.rz[ii] * TAPE_H * Math.sin(tw);
            // THE RIBBON NEEDS A REAL NORMAL. Every tape vertex on the course was
            // authored (0,0,1) — a constant world-space +Z — so a 3.3 km ribbon
            // that turns through every heading on the mountain was lit as one
            // flat facing plane: no falloff round a corner, no difference between
            // the sunlit and shaded sides of a switchback, and a uniform pale
            // value wherever the sun happened to be near +Z. Combined with the
            // unbounded widening above, that is what read as a sheet.
            //
            // The face normal is perpendicular to both the ribbon's own long axis
            // (the trail tangent) and its edge (up, rolled by the twist).
            const eX = twX, eY = TAPE_H * Math.cos(tw), eZ = twZ;
            const tX = S.tx[ii], tY = S.ty[ii], tZ = S.tz[ii];
            let nX = eY * tZ - eZ * tY;
            let nY = eZ * tX - eX * tZ;
            let nZ = eX * tY - eY * tX;
            const nL = Math.max(1e-6, Math.hypot(nX, nY, nZ));
            nX /= nL; nY /= nL; nZ /= nL;
            for (let k = 0; k < 2; k++) {
              tape.pos.push(x + (k ? twX : 0), y + (k ? TAPE_H * Math.cos(tw) : 0), z + (k ? twZ : 0));
              tape.nor.push(nX, nY, nZ);
              tape.uv.push(dist / 0.62, k);
              tape.col.push(1, 1, 1, 1);
              tape.flu.push(flut * (k ? 1 : 0.5));
              tape.edg.push(k);
              tape.vcount++;
            }
            rows.push(row);
          }
        }
        for (let r = 0; r + 1 < rows.length; r++) {
          const a = rows[r], b = rows[r + 1];
          tape.idx.push(a, a + 1, b);
          tape.idx.push(a + 1, b + 1, b);
        }
        }   // run
      }
    }

    // ======================= signage =======================================
    /**
     * A sign is a plate on a post. Two things were wrong with it in r2_04:
     *  - the "post" was the 26-38 mm tape stake, which is 1-2 px wide at the
     *    distance a sign is actually read from and simply vanishes, leaving the
     *    plate hanging in mid-air. It is now a 68-93 mm round timber post in the
     *    merged wood mesh, with a ballast chock at the foot.
     *  - the plate was planted at a fixed lateral offset, which on a benched
     *    trail puts it out over the downhill edge where the ground falls away.
     *    The offset is now walked back in until the post's foot is on ground
     *    within 0.45 m of the tread, i.e. the sign is on the bench, not the drop.
     */
    function addSign(i, lateral, cellIndex, height) {
      const treadY = S.py[i];
      let off = lateral * (S.width[i] * 0.5 + 0.95);
      let x = S.px[i] + S.rx[i] * off;
      let z = S.pz[i] + S.rz[i] * off;
      let y = groundYFoot(i, x, z, 0.07);
      for (let s = 1; s <= 5 && y < treadY - 0.45; s++) {
        const o = off * (1 - s * 0.14);
        x = S.px[i] + S.rx[i] * o;
        z = S.pz[i] + S.rz[i] * o;
        y = groundYFoot(i, x, z, 0.07);
        off = o;
      }
      // H3: the visible base sits on the LOWEST plausible rendered ground, not
      // the highest sampled one, and the post runs POST_BURY below it.
      const yBase = Math.min(y, groundYSeat(i, x, z, 0.09));
      qUpright(i, _quat);
      _m4.compose(_p.set(x, yBase - POST_BURY, z), _quat,
        _q.set(0.62, height + 0.40 + POST_BURY, 0.62));
      appendGeometry(wood, gPost, _m4, WOOD_RGBA, null, null);
      // Chock at the foot so it reads as planted rather than intersecting.
      qFlatN(i, x, z, _quat);
      _m4.compose(_p.set(x, y + 0.05, z), _quat, _q.set(0.34, 0.30, 0.10));
      appendGeometry(fabric, gBag, _m4, [0.30, 0.28, 0.24, 1], null, null);
      contactMound(i, x, z, 0.30, 0.05, 0.9);

      // H5. The board.
      //
      // This is the "crossed planks that read as grave markers". Two faults,
      // and the second one is why the sign has never been legible in any review
      // frame: the printed plate was a zero-thickness plane at local z = 0 and
      // the timber backing board was a 25 mm box centred on the SAME point, so
      // the plate sat exactly inside the board and was occluded from both
      // sides. Every sign on the course has been rendering as a blank plank
      // nailed across a post — i.e. a cross.
      //
      // The board is now behind the plate, the plate is proud of it by 8 mm,
      // and the whole thing is a landscape arrow board (0.86 x 0.40) rather
      // than the near-square plate that made the cross read as a cross. It is
      // also offset up the post rather than centred on it.
      qUpright(i, _quat);
      // qUpright's local +Z is "back up the trail", i.e. towards the rider, so
      // that is the direction the printed face has to be pushed along.
      const hlS = Math.max(1e-6, Math.hypot(S.tx[i], S.tz[i]));
      const fxS = -S.tx[i] / hlS, fzS = -S.tz[i] / hlS;
      const bw = 0.86, bh = 0.40;
      _m4.compose(_p.set(x - fxS * 0.030, y + height, z - fzS * 0.030),
        _quat, _q.set(bw + 0.05, bh + 0.05, 0.030));
      appendGeometry(wood, gBox, _m4, WOOD_RGBA, null, null);
      _m4.compose(_p.set(x + fxS * 0.008, y + height, z + fzS * 0.008),
        _quat, _q.set(bw, bh, 1));
      appendGeometry(banners, gPlane, _m4, WHITE, () => 0, atlasRect(cellIndex));
    }

    for (const sp of splits) {
      const iEntry = clamp(sp.i0 - M(5), 0, S.n - 1);
      addSign(iEntry, -sp.side, sp.mainLine === 'A' ? 10 : 11, 1.18);
      addSign(iEntry, sp.side, sp.mainLine === 'A' ? 11 : 10, 1.18);
      addSign(clamp(sp.i0 + M(7), 0, S.n - 1), sp.side, 14, 0.95);
    }
    {
      const used = [];
      for (const f of features) {
        if (f.type !== 'berm') continue;
        const i = f.i0;
        if (used.some((u) => Math.abs(u - i) < M(45))) continue;
        used.push(i);
        const dir = Math.sign(S.curv[(f.i0 + f.i1) >> 1]) || 1;
        addSign(clamp(i - M(7), 0, S.n - 1), -dir, dir > 0 ? 13 : 12, 1.05);
      }
      // Caution plates before the big shaped features.
      for (const f of features) {
        if (f.type !== 'gap' && f.type !== 'stepDown' && f.type !== 'drop') continue;
        addSign(clamp(f.i0 - M(12), 0, S.n - 1), -1, 14, 1.0);
      }
    }

    // ======================= checkpoint gates ==============================
    for (const cp of checkpoints) {
      const i = cp.station;
      const w = Math.max(2.5, S.width[i] * 0.5 + 1.3);
      qUpright(i, _quat);
      // The crossbar and banner span 5-6 m, so they must clear the HIGHER foot,
      // not the centreline. On a benched trail the uphill foot stands 2-4 m above
      // the tread, and a bar hung off the centre height simply disappears into
      // the cut bank.
      // Sampled right across the span, not just at the two feet: the bank between
      // them can be higher than either, and the banner is as wide as the gate.
      let footTop = -Infinity;
      for (let s = -1; s <= 1.0001; s += 0.25) {
        footTop = Math.max(footTop, groundYFoot(i, S.px[i] + S.rx[i] * s * w,
          S.pz[i] + S.rz[i] * s * w, 0.09));
      }
      for (const side of [-1, 1]) {
        const x = S.px[i] + S.rx[i] * side * w;
        const z = S.pz[i] + S.rz[i] * side * w;
        // H3. The foot goes on the LOWEST plausible rendered ground and the
        // leg runs 30 cm below it, so a coarser terrain LOD cannot expose the
        // cut end — which is exactly what r3_04's CP4 support pole does now.
        const y = Math.min(groundYFoot(i, x, z, 0.09), groundYSeat(i, x, z, 0.16));
        _m4.compose(_p.set(x, y - POST_BURY, z), _quat,
          _q.set(1, 3.05 + (footTop - y) + POST_BURY, 1));
        appendGeometry(metal, gPost, _m4, [0.9, 0.91, 0.93, 1], null, null);
        // Ballast bag. It used to take the POST's ground height while sitting
        // 0.42 m further out, so on any cross-slope it half-sank or floated —
        // the untextured white box in r2_08. Sampled and tilted at its own spot.
        const bx = x + S.rx[i] * side * 0.42, bz = z + S.rz[i] * side * 0.42;
        const by = groundY(bx, bz);
        qFlatN(i, bx, bz, _quat);
        _m4.compose(_p.set(bx, by + 0.11, bz), _quat, _q.set(0.66, 0.44, 0.23));
        appendGeometry(fabric, gBag, _m4, [0.30, 0.28, 0.24, 1], null, null);
        qUpright(i, _quat);
        contactMound(i, x, z, 0.46, 0.06, 0.9);
      }
      const cxp = S.px[i], czp = S.pz[i];
      const cy = Math.max(groundY(cxp, czp) + 2.5, footTop + 1.6);

      // ---- H5: a real truss, not one tube -----------------------------
      // A checkpoint gantry that is two poles and a single 100 mm tube reads as
      // scaffolding poles leaned together. Every real race gantry is a lattice:
      // parallel top and bottom chords with a zig-zag web between them. It is
      // ~14 merged cylinders per gate and it changes the object completely.
      const trussH = 0.44;
      // The legs stand footTop + 3.05 tall, so the top chord has to be clamped
      // under that or the truss floats off the ends of its own posts wherever
      // the 2.5 m centreline clearance is the binding constraint.
      const railY = Math.min(cy + 0.52, footTop + 3.05 - trussH - 0.12);
      qRail(i, _quat);
      for (const dy of [0, trussH]) {
        _m4.compose(_p.set(cxp - S.rx[i] * w, railY + dy, czp - S.rz[i] * w),
          _quat, _q.set(0.86, w * 2, 0.86));
        appendGeometry(metal, gTube, _m4, [0.84, 0.85, 0.87, 1], null, null);
      }
      qUpright(i, _quat);
      {
        // Web members. Each is a tube from (o0, low) to (o1, high) in the plane
        // of the gate, so it needs its own basis: +Y along the member.
        const bays = Math.max(4, Math.round((w * 2) / 0.85));
        for (let b = 0; b < bays; b++) {
          const o0 = -w + (b / bays) * w * 2;
          const o1 = -w + ((b + 1) / bays) * w * 2;
          const up = (b % 2) === 0;
          const ax0 = cxp + S.rx[i] * o0, az0 = czp + S.rz[i] * o0;
          const ax1 = cxp + S.rx[i] * o1, az1 = czp + S.rz[i] * o1;
          const ay0 = railY + (up ? 0 : trussH), ay1 = railY + (up ? trussH : 0);
          _ax.set(ax1 - ax0, ay1 - ay0, az1 - az0);
          const len = Math.max(0.05, _ax.length());
          _ax.multiplyScalar(1 / len);                 // member axis -> local +Y
          _az.set(-S.tx[i], 0, -S.tz[i]).normalize();
          _t3.crossVectors(_ax, _az);
          if (_t3.lengthSq() < 1e-8) _t3.set(1, 0, 0);
          _t3.normalize();
          _az.crossVectors(_t3, _ax).normalize();
          _mb.makeBasis(_t3, _ax, _az);
          _quat.setFromRotationMatrix(_mb);
          _m4.compose(_p.set(ax0, ay0, az0), _quat, _q.set(0.62, len, 0.62));
          appendGeometry(metal, gTube, _m4, [0.80, 0.81, 0.84, 1], null, null);
        }
        qUpright(i, _quat);
      }
      // Banner slung UNDER the truss with a real catenary, double-sided. Its
      // top corners are laced to the bottom chord, so it hangs off the truss
      // rather than floating in the gap under it.
      _m4.compose(_p.set(cxp, railY - 0.44, czp), _quat, _q.set(w * 2, 0.88, 1));
      appendGeometry(banners, saggedPanel(0.10), _m4, WHITE,
        (lx, ly) => clamp01((0.5 - Math.abs(lx)) * 2.2) * (0.5 - ly), atlasRect(cp.index));
    }

    // ======================= start gate ====================================
    {
      const i = M(2);
      const w = 2.45;
      qUpright(i, _quat);
      const cx = S.px[i], cz = S.pz[i];
      let cy = groundY(cx, cz);
      let footTop = cy;
      for (let s = -1; s <= 1.0001; s += 0.25) {
        footTop = Math.max(footTop, groundYFoot(i, S.px[i] + S.rx[i] * s * w,
          S.pz[i] + S.rz[i] * s * w, 0.13));
      }
      cy = Math.max(cy, footTop - 1.15);          // head rail clears the higher foot
      for (const side of [-1, 1]) {
        const x = S.px[i] + S.rx[i] * side * w;
        const z = S.pz[i] + S.rz[i] * side * w;
        const y = Math.min(groundYFoot(i, x, z, 0.13), groundYSeat(i, x, z, 0.20));
        _m4.compose(_p.set(x, y - POST_BURY, z), _quat,
          _q.set(1.5, 3.55 + (footTop - y) + POST_BURY, 1.5));
        appendGeometry(wood, gPost, _m4, WOOD_RGBA, null, null);
        contactMound(i, x, z, 0.55, 0.07, 0.9);
      }
      // Head rail.
      qRail(i, _quat);
      _m4.compose(_p.set(cx - S.rx[i] * (w + 0.2), cy + 3.42, cz - S.rz[i] * (w + 0.2)),
        _quat, _q.set(1.5, (w + 0.2) * 2, 1.5));
      appendGeometry(wood, gTube, _m4, WOOD_RGBA, null, null);
      // Banner slung beneath it.
      qUpright(i, _quat);
      _m4.compose(_p.set(cx, cy + 2.8, cz), _quat, _q.set(w * 2.05, 1.2, 1));
      appendGeometry(banners, saggedPanel(0.085), _m4, WHITE,
        (lx, ly) => clamp01((0.5 - Math.abs(lx)) * 2.2) * (0.5 - ly), atlasRect(8));
      // Timber start deck the rider rolls off. Laid on the terrain normal and
      // seated on the MEAN of its four corner heights — a world-Y-up 3.0 x 2.4 m
      // slab on the 11% start-chute grade buries one end by 0.13 m and floats the
      // other by as much.
      {
        const deckY = fitGroundPlane(i, cx, cz, 1.3, 1.2, _quat);
        _m4.compose(_p.set(cx, deckY + 0.08, cz), _quat, _q.set(2.6, 2.4, 0.14));
        appendGeometry(wood, gBox, _m4, [0.72, 0.68, 0.62, 1], null, null);
      }
      // Marshal flags on the uprights.
      qUpright(i, _quat);
      for (const side of [-1, 1]) {
        const x = cx + S.rx[i] * side * (w + 0.42);
        const z = cz + S.rz[i] * side * (w + 0.42);
        _m4.compose(_p.set(x, cy + 2.15, z), _quat, _q.set(0.52 * side, 0.74, 1));
        appendGeometry(banners, gPlane, _m4, WHITE,
          (lx) => clamp01(lx + 0.5), atlasRect(15));
      }
    }

    // ======================= finish arch ===================================
    {
      const i = clamp(Math.round((S.n - 1) * 0.985), 0, S.n - 1);
      const cx = S.px[i], cz = S.pz[i];
      const cy = groundY(cx, cz);
      qUpright(i, _quat);

      // Inflatable-style arch: a tube swept along a rounded profile authored in
      // local space (X across the trail, Y up), then placed with the basis above.
      // The two feet are 7.4 m apart, so on any cross-slope a rigid profile plants
      // one leg in the hill and hangs the other in the air. Each control point's
      // height is biased by the ground under its own x, tapering out towards the
      // crown so the arch stays an arch.
      const half = 3.7;
      const footL = groundY(cx - S.rx[i] * half, cz - S.rz[i] * half) - cy;
      const footR = groundY(cx + S.rx[i] * half, cz + S.rz[i] * half) - cy;
      const foot = (x) => {
        const u = clamp(x / half, -1, 1);
        return (u < 0 ? footL * -u : footR * u);
      };
      const archPts = [
        new THREE.Vector3(-half, 0.02 + foot(-half), 0),
        new THREE.Vector3(-half - 0.05, 1.9 + foot(-half) * 0.75, 0),
        new THREE.Vector3(-half + 0.30, 3.45 + foot(-half) * 0.45, 0),
        new THREE.Vector3(-half * 0.52, 4.3 + foot(-half * 0.52) * 0.25, 0),
        new THREE.Vector3(0, 4.52 + (footL + footR) * 0.09, 0),
        new THREE.Vector3(half * 0.52, 4.3 + foot(half * 0.52) * 0.25, 0),
        new THREE.Vector3(half - 0.30, 3.45 + foot(half) * 0.45, 0),
        new THREE.Vector3(half + 0.05, 1.9 + foot(half) * 0.75, 0),
        new THREE.Vector3(half, 0.02 + foot(half), 0),
      ];
      const archCurve = new THREE.CatmullRomCurve3(archPts, false, 'centripetal', 0.5);
      const gArch = new THREE.TubeGeometry(archCurve, 80, 0.31, 12, false);
      _m4.compose(_p.set(cx, cy, cz), _quat, _q.set(1, 1, 1));
      appendGeometry(fabric, gArch, _m4, [0.78, 0.20, 0.13, 1], null, null);
      gArch.dispose();

      // Banner panel across the arch.
      _m4.compose(_p.set(cx, cy + 3.35, cz), _quat, _q.set(half * 1.75, 1.35, 1));
      appendGeometry(banners, saggedPanel(0.075), _m4, WHITE,
        (lx, ly) => clamp01((0.5 - Math.abs(lx)) * 2.4) * (0.5 - ly), atlasRect(9));

      // Sandbag feet.
      for (const side of [-1, 1]) {
        for (let k = 0; k < 3; k++) {
          const off = side * (half - 0.02) + (dressRng() - 0.5) * 0.22;
          const along = (k - 1) * 0.44;
          const fx = cx + S.rx[i] * off + S.tx[i] * along;
          const fz = cz + S.rz[i] * off + S.tz[i] * along;
          const fy = groundY(fx, fz);
          qFlatN(i, fx, fz, _quat);
          _m4.compose(_p.set(fx, fy + 0.12, fz), _quat, _q.set(0.70, 0.46, 0.25));
          appendGeometry(fabric, gBag, _m4, [0.30, 0.28, 0.24, 1], null, null);
        }
        qUpright(i, _quat);
      }

      // Painted finish line across the tread — ground-projected in 10 spans
      // rather than one rigid quad, so it cannot half-sink on a cambered tread.
      {
        const halfW = S.width[i] * 0.62;
        const NSEG = 10, LEN = 0.5;
        const rect = atlasRect(14);
        const base = banners.vcount;
        for (let s = 0; s <= NSEG; s++) {
          const o = (s / NSEG * 2 - 1) * halfW;
          for (let e = 0; e < 2; e++) {
            const along = (e ? 0.5 : -0.5) * LEN;
            const fx = cx + S.rx[i] * o + S.tx[i] * along;
            const fz = cz + S.rz[i] * o + S.tz[i] * along;
            const fy = groundY(fx, fz) + 0.055;
            const n = groundN(fx, fz);
            banners.pos.push(fx, fy, fz);
            banners.nor.push(n.x, n.y, n.z);
            banners.uv.push(rect[0] + (s / NSEG) * rect[2], rect[1] + e * rect[3]);
            banners.col.push(1, 1, 1, 1);
            banners.flu.push(0);
            banners.vcount++;
          }
        }
        for (let s = 0; s < NSEG; s++) {
          const a = base + s * 2, b = a + 2;
          banners.idx.push(a, a + 1, b, a + 1, b + 1, b);
        }
      }

      // Timing beam housings either side of the line.
      //
      // H1. These were the vertex-colour-0.22 cylinders going into `metalMat`
      // at metalness 0.85 — a dark metal with essentially no diffuse term and
      // therefore no terminator, which is why they read as flat black bars. A
      // painted steel housing is a coated dielectric: metalness 0, roughness
      // 0.70, albedo 0.062 linear (0.28 sRGB), which is inside the 0.05-0.08
      // band the work order asks for and does have a lit side and a dark side.
      qUpright(i, _quat);
      for (const side of [-1, 1]) {
        const x = cx + S.rx[i] * side * (S.width[i] * 0.5 + 0.72);
        const z = cz + S.rz[i] * side * (S.width[i] * 0.5 + 0.72);
        const y = Math.min(groundYFoot(i, x, z, 0.07), groundYSeat(i, x, z, 0.09));
        _m4.compose(_p.set(x, y - POST_BURY, z), _quat, _q.set(0.85, 1.22 + POST_BURY, 0.85));
        appendGeometry(post, gPost, _m4, [0.28, 0.29, 0.31, 1], null, null);
        _m4.compose(_p.set(x, y + 1.2, z), _quat, _q.set(0.16, 0.24, 0.16));
        appendGeometry(metal, gBox, _m4, [0.9, 0.55, 0.15, 1], null, null);
        contactMound(i, x, z, 0.28, 0.045, 0.9);
      }
    }

    // ======================= commit ========================================
    if (stakeM.length) {
      // stakeMat, NOT woodMat: woodMat sets vertexColors:true and gStake has no
      // `color` attribute, so every stake was multiplying its albedo by the
      // unbound-attribute default of (0,0,0). See the stakeMat comment above —
      // this one line is the whole of the "black bars" defect.
      const inst = new THREE.InstancedMesh(gStake, stakeMat, stakeM.length);
      for (let k = 0; k < stakeM.length; k++) {
        inst.setMatrixAt(k, stakeM[k]);
        const c = stakeC[k];
        // Creosoted split-timber course pin: ~0.28 sRGB / 0.065 linear, warm.
        _col.setRGB(c * 0.355, c * 0.315, c * 0.262, THREE.SRGBColorSpace);
        inst.setColorAt(k, _col);
      }
      inst.instanceMatrix.needsUpdate = true;
      if (inst.instanceColor) inst.instanceColor.needsUpdate = true;
      inst.castShadow = !!(ctx.settings && ctx.settings.shadows);
      inst.receiveShadow = true;
      inst.name = 'trail-stakes';
      inst.frustumCulled = false;
      group.add(inst);
      postsMesh = inst;
      reg(gStake);
    } else gStake.dispose();

    function commit(builder, material, name, cast) {
      if (!builder.pos.length) return null;
      const g = reg(builderToGeometry(builder));
      const m = new THREE.Mesh(g, material);
      m.name = name;
      m.castShadow = !!cast && !!(ctx.settings && ctx.settings.shadows);
      m.receiveShadow = true;
      m.matrixAutoUpdate = false;
      m.updateMatrix();
      group.add(m);
      return m;
    }
    commit(wood, woodMat, 'trail-woodwork', true);
    commit(metal, metalMat, 'trail-metalwork', true);
    commit(post, postMat, 'trail-postwork', true);
    commit(fabric, fabricMat, 'trail-fabric', true);
    // Tape and banners cast: a course tape casting no shadow at 2 m from the lens
    // is one of the clearest "this is a render" tells there is, and both are
    // alpha-tested so the auto-generated depth material handles them correctly.
    commit(banners, bannerMat, 'trail-banners', true);
    commit(tape, tapeMat, 'trail-tape', true);
    // The contact mounds are a ground decal: they receive, they never cast, and
    // they render after the tread so the disturbed soil sits on top of it.
    const soilMesh = commit(soil, soilMat, 'trail-soil', false);
    if (soilMesh) soilMesh.renderOrder = 2;

    gPost.dispose(); gTube.dispose(); gPlane.dispose(); gBox.dispose(); gBag.dispose();
    for (const g of saggedCache.values()) g.dispose();
    saggedCache.clear();
  }

  function finalize(terrain) {
    if (!S || !curve) return;
    // Re-seat the start on the carved surface so the bike doesn't spawn buried.
    if (terrain) {
      const i = M(3);
      const h = terrain.sampleHeight(S.px[i], S.pz[i]);
      startTransform.position.set(S.px[i], Math.max(S.py[i], h) + 0.55, S.pz[i]);
      for (const cp of checkpoints) {
        cp.position.y = Math.max(cp.position.y, terrain.sampleHeight(cp.position.x, cp.position.z));
      }
    }
    try {
      buildTreadMesh(terrain);
    } catch (e) { console.error('[trail] tread mesh failed', e); }
    try {
      buildCourseFurniture(terrain);
    } catch (e) { console.error('[trail] course furniture failed', e); }
    if (ctx.scene) ctx.scene.add(group);
  }

  function update(dt) {
    // Only secondary motion: cloth flutter. No allocation.
    flutterUniforms.uTime.value = ctx.time;
    // Gusts: slow amplitude modulation so the tape isn't a metronome.
    flutterUniforms.uWindAmp.value = 0.09 + 0.05 * Math.sin(ctx.time * 0.37) * Math.sin(ctx.time * 0.13);
  }

  /** The tape's minimum-screen-width term needs the drawing-buffer height. */
  function resize() {
    if (!ctx.renderer || !ctx.renderer.getDrawingBufferSize) return;
    const s = ctx.renderer.getDrawingBufferSize(_resizeV2);
    if (s.y > 1) flutterUniforms.uViewportH.value = s.y;
  }

  function dispose() {
    if (ctx.scene) ctx.scene.remove(group);
    group.traverse((o) => {
      if (o.isMesh || o.isInstancedMesh) {
        if (o.geometry) o.geometry.dispose();
      }
    });
    for (const d of disposables) { if (d && d.dispose) d.dispose(); }
    disposables.length = 0;
    group.clear();
    treadMesh = null;
    postsMesh = null;
  }

  return {
    // --- CONTRACT §4 ---
    get curve() { return curve; },
    get length() { return length; },
    features, checkpoints, startTransform,
    build, getCarveStamps, finalize,
    sampleAt, nearestT, widthAt,
    // --- additive (see CONTRACT-NOTE) ---
    splits, phases: phaseSpans, jumps: jumpLog,
    cornerAudit,                  // lean-identity audit; see auditCornerIdentity()
    routeAudit,                   // route-variant audit; see build() and scoreRoute()
    safety,                       // exposure-safety audit; see applyExposureSafety()
    speedAt, surfaceAt,
    finishT: 0.985,
    group,
    get stations() { return S; },
    get treadMesh() { return treadMesh; },
    get postsMesh() { return postsMesh; },
    update, resize, dispose,
  };
}
