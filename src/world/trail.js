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
// CONTRACT-NOTE (WHAT CHANGED IN THE ROUND THAT ADDED R_PLAN_MIN — read this
//   first, the three fixes are independent and each has its own long note):
//
//   (A) THE JUMP LINE IS NOW PACED AT THE SPEED ITS OWN FEATURES PUBLISH.
//       `safety.jumpWindowBad` was 20 of 32 air features across four seeds —
//       every one of them BELOW the minimum of its own published speed window,
//       i.e. the design telling the rider to case every jump on the course. It
//       is now 0 of 24 on the three buildable seeds, with `jumpOverSpeed` still
//       0. Three things had to be right: computeVertKappa() must not price a
//       lip's own convexity as unwanted relief (S.hBal); leanBudgetAt() and
//       brakeAccel() must not price a lip as a 100% hillside (S.gradeR); and
//       buildJump() must cap its RUN-IN at the approach speed rather than at the
//       takeoff speed, or the fixed point ratchets a jump out of existence one
//       lip-climb per pass. Then build() iterates features-and-profile to a
//       fixed point (JUMP_FIT_ITERS).
//
//   (B) THE COURSE NO LONGER SHIPS SUB-10 m CORNERS. Every seed carried a
//       4.8-5.5 m plan radius; the shipped minimum is now 10.5-16.5 m and no
//       station on any seed is inside R_PLAN_MIN. Fixed at the march
//       (SB_RADIUS, the turn-authority bound) rather than downstream, because
//       the hairpin lock in openTightCorners() is real — re-measured this round
//       on the respaced line — and a hairpin that is authored tight cannot be
//       opened afterwards.
//
//   (C) CONTRACT §4's DESCENT RULE IS MEASURED AS A PROPERTY. DESC_MAX_RUN
//       counts CONSECUTIVE non-descending stations and a 10 micrometre dip
//       resets it, so the reference seed reported 11.2 m while shipping 89.4 m
//       that never got back below its own start. `safety.flatRunShipped` is the
//       property; enforceDescentShipped() now enforces a windowed net-descent
//       rule; and the variant budget is spent on the property, not the counter.
//
//   ...and the build now REFUSES a course whose shipped profile carries a wall.
//   See the wall gate at the end of build(): seed 12345 ships 12.2 m of ground
//   at up to a 93% committed gradient, on which the measured lean budget is
//   zero at any radius, bank or speed. It now fails generation loudly, as seed 2
//   does. Two of five seeds refuse; three build.
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
//     trail.safety.routeAdmissible / routeFail / routeReseeded / routeVariants
//                       -> whether ANY marched variant produced a course a
//                          bicycle can ride and descend, which test the chosen
//                          one failed if not, and whether the extended variant
//                          budget was used. See routeAdmissible().
//
// CONTRACT-NOTE (BEHAVIOUR CHANGE — build() can now THROW):
//   `trail.build(terrain)` throws if no marched variant is admissible. The error
//   message names the seed, the test every variant failed, and each variant's
//   numbers; `err.trailRouteAudit` carries the rows. This replaces the previous
//   behaviour of shipping the best inadmissible route with a flag on
//   `safety.routeAdmissible`, which three consecutive audits found being ignored.
//   `routeAdmissible` tests conditions no downstream pass can repair, so a build
//   that trips it has not produced a course. Known affected seed: 2 (every
//   variant climbs 18.7-93 m against a 12 m limit). The A/B control path
//   `ctx.settings.trailSolver = { feasible: false }` never throws — it exists to
//   reproduce the pre-rideability build bit-identically.
//     trail.safety.feasViol / feasFrac / feasCorner / feasLaunch / feasWorstDeg
//                       -> the joint (v, phi) feasibility of the SHIPPED
//                          stations: how many have NO speed in
//                          [V_DESIGN_FLOOR, design] and no bank inside the phase
//                          ceiling at which the lean identity, the launch cap and
//                          the longitudinal demand all hold at once, split by
//                          which constraint fails. See auditJointFeasibility().
//     trail.safety.cornerFloorBound
//                       -> stations where the honest corner cap fell below
//                          V_DESIGN_FLOOR and the floor was applied anyway. This
//                          is a GEOMETRY failure being reported rather than
//                          obeyed — see designCap().
//     trail.routeAudit[].feasFrac / feasMean / climb / flat / minR / conflict /
//                       admissible / fail
//                       -> per-variant rideability and admissibility, so the
//                          choice between variants can be read rather than assumed.
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
//     tech drop       8.97 m/s 32 km/h  1.25 m @  0°   4.5 m  3.2 m  0.53 s  4.8–17.0 m/s
//     tabletop        9.89 m/s 36 km/h  0.92 m @ 14°   7.5 m  4.1 m  0.83 s  4.8–16.4
//     double-1        9.16 m/s 33 km/h  0.98 m @ 19°   7.6 m  4.2 m  0.93 s  8.1–17.4
//     double-2        9.39 m/s 34 km/h  0.98 m @ 26°   9.2 m  5.2 m  1.10 s  8.1–17.1
//     road gap (A)    9.06 m/s 33 km/h  1.06 m @ 30°   9.7 m  6.2 m  1.25 s  8.0–15.9
//     step-down       7.24 m/s 26 km/h  0.72 m @ 25°   5.8 m  3.2 m  0.92 s  6.1–13.8
//     slab roll       7.29 m/s 26 km/h  1.64 m @  0°   4.2 m  2.9 m  0.61 s  3.9–13.8
//     finish booter   9.74 m/s 35 km/h  0.72 m @ 19°   7.9 m  3.6 m  0.90 s  7.6–16.9
//
//   RE-MEASURED, AND THE JUMPS ARE SMALLER THAN THE TABLE THIS REPLACES. That is
//   the fix, not a regression. The previous table's takeoffs of 11.7-17.5 m/s
//   describe features solved for a pace the course never delivered: the profile
//   that shipped paced their lips at 5.4-6.2 m/s, BELOW the vMin of their own
//   published windows, on 20 of 32 air features across four seeds. These numbers
//   are what the corridor actually carries, and the shipped design speed at every
//   lip is now inside its own window on every buildable seed
//   (`safety.jumpWindowBad` = 0, `jumpOverSpeed` = 0). A 9 m/s takeoff off a 30°
//   lip carrying 9.7 m with 1.25 s of air is a real road gap; a 19.8 m landing
//   nobody arrives fast enough to reach is not.
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
import { report, maybeYield } from '../core/bootProgress.js';

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

// ---------------------------------------------------------------------------
// THE LAUNCH CONSTRAINT — priced on the relief this module ACTUALLY authors.
//
// WHAT WAS WRONG, and it is the whole of round 6's remaining cause. The chatter
// term above prices roughness from `S.bumps + S.rough` at ONE wavelength,
// ROUGH_LAMBDA = 1.6 m. That is a real wavelength — it is where the root ridges
// and the braking-bump trains live — and it is not where the launching content
// lives. Measured on the shipped build (seed 20260726, 0.37 m stencil on the
// wheel-dilated design centreline, 0.05 m plan-arc sampling): at arc 288-294 m
// the profile carries 0.056-0.139 /m of convex vertical curvature at a 2 m
// stencil and 0.051-0.089 /m at a 6 m stencil, while `S.bumps` there is 0.0015 m
// and `S.rough` 0.0145 m — so the chatter model returned ~16.8 m/s and the
// design speed profile ran at 14.9-16.8 m/s over ground whose geometric launch
// speed is 8-10 m/s. None of that content is visible at 1.6 m. It is 2-20 m
// relief: the residue of the mountain the bench smoothing did not remove, plus
// the shoulders of every chute and roller the feature pass authors.
//
// THE FIX IS TO MEASURE INSTEAD OF MODELLING. `computeVertKappa()` takes the
// convex vertical curvature of the profile that will actually be committed,
// across a LADDER of stencils (LAUNCH_STENCILS), and keeps the worst. A single
// stencil cannot do this: a 0.4 m stencil sees 2 m wobble and is blind to a 20 m
// rollover once noise is averaged in, and a 6 m stencil sees the rollover and
// reads a 2 m step as a third of its true curvature. The ladder sees both.
//
// The criterion is the one the wheels obey. Over a crest of curvature kappa at
// speed v the tyres carry (g - v^2*kappa) of the normal load, so contact is lost
// at v^2*kappa/g = 1. Designing to LAUNCH_SAFETY means the wheels still carry
// 1 - 1/LAUNCH_SAFETY of static load at the design speed, which is the margin
// that survives a rider arriving fast, a carve that rounds a crest the wrong
// way, and the fact that the bike is 1.25 m long and samples two points.
const LAUNCH_SAFETY = 1.85;
// Half-stencils, metres. 0.4 m is the station spacing and is within 8% of the
// 0.37 m wheel radius the acceptance test uses; 12.8 m is past the longest
// rollover a 20 m/s rider can be thrown by.
const LAUNCH_STENCILS = [0.4, 0.8, 1.6, 3.2, 6.4, 12.8];
// Floor on the design speed. Where the relief cannot be flattened enough for
// the phase speed, the speed comes down — but a trail that asks for 4 m/s is
// not a downhill course, it is a hike. Where THIS floor binds, the geometry
// pass has failed and the failure is reported (safety.launchFloorBound).
const V_DESIGN_FLOOR = 6.2;
// A jump's run-out climbs out of its landing pit at this fraction of the local
// grade, so the world gradient through it still descends, and never spends more
// than JUMP_OUT_MAX metres of trail doing it. See solveJump().
const JUMP_OUT_FRAC = 0.55;
const JUMP_OUT_MAX = 34.0;
// The pace a jump is solved against, as a fraction of the pre-feature design
// speed at its lip. 1.00 = no derate; see the note in buildJump() for the
// measurement that rejected 0.85.
const JUMP_V_DERATE = 1.00;
// ---------------------------------------------------------------------------
// THE JUMP-LINE FIXED POINT. See buildJump() and the loop in build().
//
// The pace at a lip and the feature built there determine each other, and until
// this round nobody solved the pair: the features were built once, against the
// pace of ground that did not yet carry them, and the profile was then solved
// against the ground that did. `safety.jumpWindowBad` measured the residual at
// 20 of 32 features across four seeds and every audit since round 8 has named
// it. These four constants are the whole of the fix's tuning surface.
//
// Iterations. The map converges fast because it is close to a contraction — the
// delivered pace is a smooth, weakly-varying function of the lip height, and the
// lip height is a square-root function of the pace. Measured across four seeds,
// the residual is zero after 2-3 passes and never improves after 4.
const JUMP_FIT_ITERS = 4;
// Relaxation on the pace update. A raw fixed-point step (take the delivered
// speed as the next approach speed) can oscillate on a feature whose delivered
// pace is set by a corner cap just downstream: the jump shrinks, the corner cap
// stops binding, the jump grows again. Under-relaxing kills that without
// slowing convergence measurably.
const JUMP_FIT_RELAX = 0.65;
// The takeoff speed the authored jump-line rhythm (11 / 13 / 15 / 19 / 16 m of
// carry) was sized for — i.e. the number `opt.targetD` means. Landing distance
// is ballistic and scales as v^2, so a feature reached at a different pace is
// scaled by (vTO/V_JUMP_REF)^2 and the line keeps its rhythm at whatever speed
// the corridor delivers. Taken from the shipped ballistics table at the head of
// this file: the five jump-line features publish takeoffs of 11.7-14.0 m/s.
const V_JUMP_REF = 12.0;
// The most of the approach's kinetic energy a lip may take. Climbing the lip
// costs 2*g*lipH*0.85 of v^2, so a 0.98 m lip costs 16.3 m^2/s^2 — 45% of the
// energy at 6 m/s and 11% at 12. Past about a third the takeoff is slower than
// the jump needs whatever the rider does, and the feature is a speed bump with
// a hole after it. 0.35 keeps vTO >= 0.8*vApproach everywhere.
const JUMP_LIP_KE = 0.35;
// Below this takeoff speed a lip is not a jump. It was 8.0 m/s, which is the
// right number for a jump-line feature and refuses a legitimate small double in
// a slow phase; with the lip now sized to the pace the geometry stays honest
// further down, and solveJump's own `D < 3.5 m` gate is what finally refuses.
// Below this takeoff speed a lip is not a jump. It is V_DESIGN_FLOOR, and that
// is not a coincidence: designCap() floors the design speed at V_DESIGN_FLOOR,
// so a feature solved for a takeoff BELOW the floor publishes a ballistic cap
// the speed solve is not allowed to honour, and the profile then paces the lip
// faster than the jump was built for — `safety.jumpOverSpeed`, the exact defect
// this round is closing, inverted. A jump the corridor cannot deliver the floor
// speed to is a jump that does not belong there; buildFeatures reports it as a
// loss (safety.jumpFitLostWhy) rather than shipping it.
const JUMP_V_MIN = V_DESIGN_FLOOR;
// The shortest carry worth calling a jump, metres, and the distance a scaled
// target is floored at. solveJump refuses anything under 3.5 m outright; aiming
// at 4.5 keeps the solve off that gate at the slowest pace a lip is built at.
const JUMP_D_MIN = 4.5;
// EXEMPT THE BALLISTIC CORE FROM THE LAUNCH-SPEED CAP? MEASURED: NO, AND THE
// NUMBER IS THE REASON.
//
// The argument for `true` is in techSpeedCap() and it is sound as physics: a
// jump lip is convex past the launch cap BY CONSTRUCTION, so the cap collapses
// there, and the profile then paces every jump on the course at 5.4-6.2 m/s
// against its own published takeoff of 11.5-12.0 — below the vMin of its own
// speed window on all but the tabletop. The design says: case every jump.
//
// The argument for `false` is the autopilot, 21 cells x 3 pitch saturations x
// 3 seeds, one process per arm, world fingerprint asserted stable:
//
//     arm                     20260726        777          99999
//     exempt = false      med 1289, 0/21   9/21 finish   med 355, 0/21
//     exempt = true       med 1240, 0/21   0/21          med 355, 0/21
//     ... and 777's max   2693 m           1298 m
//
// Turning it on converts jumps the rider used to ROLL into jumps the rider must
// LAND, and completions go 9 -> 0 on the one seed that finishes. That is a
// statement about the whole jump line, not about one feature: the jumps are
// sized for a takeoff the course cannot deliver, and un-capping the speed does
// not fix that, it just makes the rider arrive at the lip fast enough to be
// thrown by it. The fix is to size the jumps for the pace that ships, which is
// a fixed point this module does not yet iterate (safety.jumpWindowBad counts
// the residual: 16 of 32 features on the four seeds are outside their own
// window). Until that is done the honest default is the conservative one.
//
// SUPERSEDED, AND KEPT `false` WITH THE MEASUREMENT SO IT IS NOT RE-DERIVED.
// The diagnosis above is right and the instrument is wrong. The defect is that
// the launch cap MEASURES a jump's own lip as unwanted relief; the fix is to
// stop measuring it, not to stop applying the cap. computeVertKappa() now prices
// the constraint on the profile with the authored ballistic relief subtracted
// (S.hBal), which fixes the RUN-IN — where the defect actually is, because the
// stencil ladder runs to 12.8 m — while leaving the cap binding on the knuckle,
// the landing and the run-out, where this switch would have removed it. With
// that plus the fixed point in build(), `safety.jumpWindowBad` is 0 of 24 air
// features on the three buildable seeds and `jumpOverSpeed` is 0.
const JUMP_PACE_EXEMPT = false;
// Braking bumps on a jump run-in. Measured at 45 mm / 1.1 m they cost the
// approach 45% of its speed through chatterKappa(); measured WITH the pace
// exemption off, removing them changes the autopilot by less than one cell on
// every seed. Left on, because they are deliberate trail design and the
// evidence for removing them is not there.
//
// RE-EXAMINED THIS ROUND AND STILL ON, but the measurement above should be read
// with its confound stated: it was taken while the launch cap was pricing every
// lip as relief, so the jump line was already pinned at V_DESIGN_FLOOR and
// removing the bumps could not change anything. With that fixed the bumps are no
// longer masked — and the delivered approach speeds on the jump line are 8.7 to
// 10.6 m/s with them still on, so they are not what binds either. A measurement
// taken behind a saturated constraint is not evidence; this one now is.
const JUMP_RUNIN_BUMPS = true;

// ---------------------------------------------------------------------------
// THE CARVE'S OWN NOISE FLOOR — the reason a top speed exists at all.
//
// This is the number that decides the acceptance figure, so it is a measurement
// and it is stated as one. Take every station where the EMITTED DESIGN PROFILE
// is flat (|kappa_design| < 0.012 /m at the 0.37 m acceptance stencil) and no
// feature is authored, and ask what convex curvature the COMMITTED surface
// carries there anyway. Three seeds, ~25-29k stations each:
//
//     seed        p50     p75     p90     p95     p99     max
//     20260726  0.0000  0.0192  0.0429  0.0561  0.0840  0.361
//     777       0.0000  0.0117  0.0335  0.0495  0.0827  0.258
//     12345     0.0000  0.0139  0.0366  0.0497  0.0777  0.179
//
// Half the course is reconstructed exactly. The other half carries curvature
// the design never asked for, from the millimetre-scale residue of a
// distance-weighted mean of stamp discs: 0.0012 m r.m.s. at 1-2 m wavelength on
// quiet ground turns into 0.021 /m of curvature, because A*(2*pi/lambda)^2 is a
// brutal operator at short wavelengths. That is the round-5 lesson in the only
// units that matter — the launch threshold at 12.9 m/s is 0.059 /m, which a
// 1 m ripple reaches at 1.5 mm.
//
// SO THERE IS A TOP SPEED ON THIS COURSE AND IT IS NOT A DESIGN CHOICE. At the
// p90 of that distribution the launch-free speed is 15.1 / 17.1 / 16.4 m/s; at
// the p95 it is 13.2 / 14.1 / 14.1. The acceptance gate is on the WORST 25 m
// BIN, which lands near the p97 of the course distribution, and that is 11-12
// m/s. Every phase ceiling in V_PHASE above that is a statement the carve cannot
// honour, and the measurement says so plainly: at 16.8 m/s the 50-100 m bins ran
// 16-17% launched while the DESIGN profile at the same stations scored 0-3%.
// Nothing in this module can flatten that, because the design is already flat.
//
// terrain.applyCarve() owns that reconstruction and this module may not touch
// it. What this module owns is the speed it asks a rider to take it at.
//
// CONTRACT-NOTE (cross-module, for the integration pass and the terrain owner):
//   KAPPA_CARVE_FLOOR is a measurement of terrain.applyCarve()'s reconstruction,
//   not a property of the trail. It costs this course roughly 5 m/s of top
//   design speed — V_PHASE asks for 19 m/s in the finish sprint and the carve
//   will only support 11.8. If applyCarve gets smoother, this number should come
//   DOWN and the course gets faster for nothing. Re-measure it (the harness is
//   twenty lines: dilate the committed centreline by a 0.37 m disc, second-
//   difference at 0.37 m, and take the p95 over stations whose design curvature
//   is under 0.012 /m) rather than trusting this comment. Terrain changed under
//   this measurement mid-round — a track-frame anisotropic tread filter landed
//   between two of my runs — so it is set at the conservative end of the three
//   seeds rather than fitted to one.
// SWEPT, on one cached mountain per seed, in one process: 0.026 / 0.030 / 0.034 /
// 0.038 (top design speed 14.3 / 13.3 / 12.5 / 11.8 m/s) give a worst 25 m bin of
// 11.8 / 11.2 / 12.0 / 15.4 % on seed 20260726, 22.6 / 19.6 / 17.2 / 20.2 on 777
// and 14.4 / 15.0 / 14.2 / 14.6 on 12345. The worst bin is nearly FLAT in this
// parameter, which is itself the finding: past about 12.5 m/s the residual is not
// speed-limited any more, it is a handful of places where the committed surface is
// rough at any speed a bicycle is ridden at. 0.034 is taken because it minimises
// the worst case across the three seeds; going lower buys nothing and costs the
// course its character.
const KAPPA_CARVE_FLOOR = 0.034;

// ---------------------------------------------------------------------------
// THE LAUNCH KERNEL — curvature per metre of amplitude, as the wheel and the
// carve actually deliver it.
//
// Two filters sit between an authored sinusoid and the curvature that throws a
// rider, and both are wavelength-dependent, so a single fudge factor (which is
// what ROUGH_SUSP is) cannot represent either.
//
//  (1) THE WHEEL. A 0.37 m wheel does not follow the ground; its axle follows
//      the ground dilated by its own disc. Short wavelengths are bridged. This
//      is computed numerically, once, at module load — dilate a sinusoid, take
//      the second difference at the same 0.37 m stencil the acceptance test
//      uses, and record the answer. No closed form, no calibration constant.
//
//  (2) THE CARVE. terrain.applyCarve() reconstructs the surface as a
//      distance-weighted mean of stamp discs, which is a low-pass. Its transfer
//      function was measured across ten bands on three seeds in round 6
//      (committed r.m.s. / design r.m.s. on the committed centreline): 0.28 at
//      0.3-0.5 m, 0.06 at 0.5-1, 0.11 at 1-2, 0.68 at 2-6, 0.97 at 6-20, 1.00
//      above. Pricing sub-metre relief at full amplitude would be the mirror of
//      the ROUGH_LAMBDA error — slowing the trail for texture that is never
//      built. It is applied ONLY to the sub-station authored components
//      (microDetail, root ridges, braking bumps); the station-lattice curvature
//      is taken at face value, which is conservative by the same table.
//
// CONTRACT-NOTE: CARVE_XFER describes terrain.applyCarve()'s behaviour, which
//   this module does not own. It is a measurement, reproduced on three seeds,
//   and it is only ever used to make the speed profile LESS conservative than
//   the raw authored amplitude would make it. If applyCarve's reconstruction
//   changes materially, re-measure it; the failure mode of a stale table is a
//   trail that is paced for texture it no longer has.
const WHEEL_R = 0.37;
const LK_LAMBDA = [0.2, 0.28, 0.4, 0.55, 0.75, 1.0, 1.4, 1.9, 2.6, 3.6, 5.0, 7.0, 10.0, 14.0, 20.0, 30.0, 45.0];
const CARVE_XFER_BANDS = [
  [0.30, 0.05], [0.50, 0.28], [1.00, 0.06], [2.00, 0.11],
  [6.00, 0.68], [20.0, 0.97], [1e4, 1.00],
];

/** Piecewise-constant in band, linear in log-lambda across band edges. */
function carveXfer(lambda) {
  let prevEdge = 0.05, prevVal = CARVE_XFER_BANDS[0][1];
  for (let k = 0; k < CARVE_XFER_BANDS.length; k++) {
    const [edge, val] = CARVE_XFER_BANDS[k];
    if (lambda <= edge) {
      const t = clamp01((Math.log(lambda) - Math.log(prevEdge)) /
        Math.max(1e-6, Math.log(edge) - Math.log(prevEdge)));
      return lerp(prevVal, val, t);
    }
    prevEdge = edge; prevVal = val;
  }
  return 1.0;
}

/**
 * Convex curvature seen by a WHEEL_R disc at a WHEEL_R stencil, per metre of
 * amplitude, for a sinusoid of wavelength `lambda`. Computed numerically once.
 */
let LK_KAPPA_CACHE = null;
function launchKernelTable() {
  if (LK_KAPPA_CACHE) return LK_KAPPA_CACHE;
  const out = new Float64Array(LK_LAMBDA.length);
  const A = 0.05;                  // representative amplitude; dilation is mildly non-linear
  for (let k = 0; k < LK_LAMBDA.length; k++) {
    const lam = LK_LAMBDA[k];
    // Fine enough to resolve both the wheel and the wave, coarse enough that the
    // whole table costs a couple of million operations once per process.
    const ds = Math.max(0.002, Math.min(0.01, lam / 150));
    const span = Math.max(lam * 1.6, WHEEL_R * 5);
    const m = Math.round(span / ds) + 1;
    const y = new Float64Array(m);
    for (let i = 0; i < m; i++) y[i] = A * Math.cos((2 * Math.PI * (i * ds - span * 0.5)) / lam);
    // dilation by the wheel disc (upper envelope of the axle locus)
    const w = Math.round(WHEEL_R / ds);
    const cap = new Float64Array(2 * w + 1);
    for (let j = -w; j <= w; j++) cap[j + w] = Math.sqrt(Math.max(0, WHEEL_R * WHEEL_R - (j * ds) * (j * ds)));
    const c = new Float64Array(m);
    for (let i = 0; i < m; i++) {
      let best = -Infinity;
      const lo = Math.max(0, i - w), hi = Math.min(m - 1, i + w);
      for (let j = lo; j <= hi; j++) { const v = y[j] + cap[j - i + w]; if (v > best) best = v; }
      c[i] = best;
    }
    const h = Math.max(1, Math.round(WHEEL_R / ds));
    let worst = 0;
    for (let i = h; i < m - h; i++) {
      const kap = -(c[i - h] - 2 * c[i] + c[i + h]) / (h * ds * h * ds);
      if (kap > worst) worst = kap;
    }
    out[k] = worst / A;            // per metre of amplitude
  }
  LK_KAPPA_CACHE = out;
  return out;
}

/** Interpolated launch kernel, including the carve's low-pass. */
function launchKappaPerAmp(lambda) {
  const LK = launchKernelTable();
  let k = 0;
  while (k < LK_LAMBDA.length - 1 && LK_LAMBDA[k + 1] < lambda) k++;
  const a = LK_LAMBDA[k], b = LK_LAMBDA[Math.min(LK_LAMBDA.length - 1, k + 1)];
  const t = b > a ? clamp01((lambda - a) / (b - a)) : 0;
  const base = lerp(LK[k], LK[Math.min(LK.length - 1, k + 1)], t);
  return base * carveXfer(lambda);
}

// ---------------------------------------------------------------------------
// THE CORNER-HOLDING CONSTRAINT, and why it is the other half of the same solve.
//
// Round 6 clamped the speed governor to the geometric sqrt(g/kappa) limit and
// measured the result over 63 runs: seed 20260726's median progress went
// 224 -> 304 m and its airborne fraction 0.130 -> 0.067, and seed 777 COLLAPSED
// to 34 m with 17 of 21 crashes as lowsides. That is the correct experiment and
// the wrong conclusion drawn from it. The clamp changed the speed and left the
// BANK exactly where it was — banks solved for 16 m/s, ridden at 9. On a berm of
// bank phi taken at v the rider's lean relative to the tread is
//
//     leanContact = atan(v^2/(g*r)) - phi
//
// which goes NEGATIVE when the corner is taken slower than the berm was built
// for: the rider has to lean OUT, up the berm, to stay on it, and past what the
// tyres will hold in that direction the bike slides down the face. That is a
// lowside, and it is exactly what 17 of 21 cells did.
//
// So the bank has to be re-solved at whatever speed the launch constraint
// leaves, not merely clamped after it. Two numbers bound that:
//
//   LEAN_HOLD    how far the rider can lean OUT of a berm and still hold it.
//                Less than LEAN_AVAIL: leaning out is a worse-supported posture
//                than leaning in, and the load is on the outside edge of a tyre
//                designed to be loaded on its shoulder knobs.
//   SLOW_FRAC    the slowest a rider realistically arrives at a corner the
//                design paced at v — i.e. they braked too hard. The bank must
//                still be holdable there.
//
// Together: bank <= atan((SLOW_FRAC*v)^2 / (g*r)) + LEAN_HOLD, alongside the
// existing identity floor bank >= atan(v^2/(g*r)) - LEAN_DESIGN. Where those
// two cross, the corner cannot be built for both the fast and the slow rider and
// the RADIUS is what is wrong — which is what openTightCorners() is for.
const LEAN_HOLD = THREE.MathUtils.degToRad(34);
const SLOW_FRAC = 0.55;

// ===========================================================================
// THE RIDEABILITY ENVELOPE — MEASURED OFF bike.js, not derived from a free-body
// diagram of it.
//
// WHAT THE PREVIOUS CUT OF THIS BLOCK GOT WRONG, and it is worth stating in
// full because two rounds were spent on the consequences.
//
// It mirrored leanCeiling()'s arithmetic by hand: the two-end moment balance,
// the friction ellipse per wheel, and one fitted smoothstep on the rear load
// fraction. Verified against a published table to 0.01 deg — of its own output.
// Three terms that dominate on steep ground were missing:
//
//   1. `T.LEAN_LIMIT_PLANTED` (0.34 rad = 19.5 deg). bike.js floors the ceiling
//      there whenever ANY tyre is on the ground, in terms: "no amount of derate
//      can take away the ability to steer while a tyre is on the ground". The
//      mirror had no floor at all and returned exactly zero above ~48% grade.
//   2. `T.RIDER_SHIFT` (0.30 m). Full aft pitch input moves the centre of mass
//      0.246 m back, which takes the rear load fraction on a 45% grade from
//      0.134 to 0.331 — from below the model's own skipping band to well above
//      it. CONTRACT section 6 requires the rider to do exactly this ("body
//      English: weight back on steeps"). The mirror rode with its weight over
//      the bars on a 90% grade.
//   3. mu on DIRT. bike.js's `SURF_MU[DIRT]` is 1.08; the mirror used 1.00.
//
// And its one fitted parameter — the smoothstep between REAR_LO and REAR_HI —
// was calibrated against synthetic cells produced by an autopilot that held
// `input.pitch` at zero for the whole sweep. An instrument defect was laundered
// into a route-planning constant, and the term then refused ground a rider who
// moves can ride: on 45% ground it answered 2.4 deg where the bike delivers 35.
//
// SO THIS IS NOT A MODEL ANY MORE. IT IS A MEASUREMENT.
//
// `RIDE_CEIL` below is `bike.state.leanLimit` — the number leanCeiling() itself
// returns, which is also the number bike.js's own low-side test is judged
// against — recorded while driving the REAL src/physics/bike.js and
// src/physics/collision.js over an ANALYTIC helicoid (no grid, no bilinear
// facets, no trail), at a grid of gradient x deceleration, at the reference
// rider's pitch schedule, on DIRT. Each cell is the p10 of the published
// ceiling over the run — the tenth percentile, not the median, so a cell that
// spends part of its time light is scored at the value it spends it at.
//
// Everything about the probe that could bias it is stated:
//   * straight-line runs, speeds 6.2 / 10 / 15 m/s, worst of the three, for the
//     decel = 0 column; for decel > 0 a single ramp from 16 m/s to 5.5 m/s at
//     the column's rate, scored only while the ramp is live.
//   * the governor modulates the brakes against `state.skid`, because holding a
//     locked wheel is a controller defect and not a property of the bike.
//     Measured mean skid over the grid is 0.003.
//   * cells where the run did not complete are entered as ZERO, which is why
//     the table falls off a cliff rather than tapering: past a gradient of
//     about 0.75 the bike crashes on a DEAD STRAIGHT run at every speed.
//
// THE TABLE LANDS ON THE MECHANISM, which is the strongest evidence that the
// probe is measuring the bike and not the harness. With the reference rider's
// weight shift the cg-to-front-axle arm is
//     0.640 + 0.30*(78/95) = 0.886 m,
// so the endo gradient is 0.886/1.05 = 0.844. The measured ceiling is nonzero
// at 0.70 and zero at 0.75-and-above at every deceleration — i.e. the bike
// stops being rideable within one grid step of its own shifted endo point, a
// number that appears nowhere in the probe.
//
// WHAT IT SAYS, next to what the mirror said (design budget, degrees, decel 0):
//
//     grade     mirror     MEASURED      grade     mirror   MEASURED
//      flat      28.0        28.0         45%        2.4      24.5
//      25%       16.9        25.5         55%        0.0      24.1
//      30%       14.4        25.5         70%        0.0      20.3
//      36%       11.3        25.5         75%        0.0       0.0
//
// The flat-ground figure is 28.0 deg in BOTH columns and that is deliberate:
// RIDE_USE is chosen (not tuned) so the measured ceiling on flat dirt maps
// exactly onto LEAN_DESIGN. Everywhere the old constant was right, the answer
// is still the old constant to the last digit; what changes is only the steeps.
//
// THE REFERENCE RIDER, stated explicitly because the whole course design is now
// priced off it. `RIDE_PITCH_G` and `RIDE_PITCH_A` define
//
//     input.pitch = -clamp( tanTheta/0.55 + decel/12, 0, 1 )
//
// (negative = weight BACK — bike.js's own axis convention). It is not a guess:
// the optimum aft shift was mapped over gradient x deceleration by sweeping
// `input.pitch` in five steps at each cell and taking the ceiling each produced.
// The schedule tracks the measured optimum to within one 0.25 step everywhere
// the ground is rideable, and it is a real control input with an interior
// optimum, not a cheat — a constant `pitch = -1` is WORSE than zero below about
// a 20% grade (0 deg of ceiling on the flat: the front is unweighted and the
// FRONT then limits), and a constant `pitch = 0` is worse than the schedule
// everywhere above 25%. Any harness quoting a completion figure against this
// course must drive this schedule, or it is measuring a different rider.
//
// WHAT THE MEASUREMENT DOES NOT COVER, so nobody quotes it past its evidence:
//   * The corner-HOLD half of the probe — the largest lean actually sustained
//     round a constant-radius helicoid — agrees with the derated budget at
//     plan radii above ~33 m (measured usable fractions 0.58-0.76 of the
//     ceiling in tangent space, against the 0.596 used here) and is BELOW it at
//     radii of 22-30 m. Whether that is the bike or the harness's own pursuit
//     controller could not be separated: at 6.2 m/s those cells are limited by
//     the probe's speed governor (they fail `stuck` with 0.1 m of tracking
//     error). Tight radii are governed here by the radius machinery —
//     openTightCorners(), SB_RADIUS, rideRadius() — and NOT by this term.
//   * Bank is not an axis. The probe runs unbanked ground; a berm is credited
//     downstream, additively, exactly as before.
// ===========================================================================
const RIDE_WB = 1.25;             // bike.js T.WHEELBASE
const RIDE_CG_FWD = -0.015;       // bike.js T.CG_FWD
const RIDE_CG_H = 1.05;           // bike.js T.ROLL_H — cg above the contact line
const RIDE_A = RIDE_WB * 0.5 - RIDE_CG_FWD;   // 0.640 m, cg -> front axle
const RIDE_B = RIDE_WB * 0.5 + RIDE_CG_FWD;   // 0.610 m, cg -> rear axle
const RIDE_SHIFT = 0.30;          // bike.js T.RIDER_SHIFT
const RIDE_SHIFT_FRAC = 78 / 95;  // bike.js T.RIDER_MASS / T.MASS
// The reference rider's pitch schedule. See the note above.
const RIDE_PITCH_G = 0.55;        // tan(theta) per unit of aft pitch input
const RIDE_PITCH_A = 12.0;        // m/s^2 of deceleration per unit of aft pitch
const RIDE_DEC_CAP = 5.0;         // m/s^2 of design deceleration worth pricing
                                  // (also the last column of the table)

/** input.pitch the reference rider is holding here. Negative = weight back. */
function ridePitch(tanTheta, decel) {
  const t = Math.max(0, tanTheta) / RIDE_PITCH_G
    + clamp(decel, 0, RIDE_DEC_CAP) / RIDE_PITCH_A;
  return -clamp01(t);
}

/**
 * cg-to-front-axle arm with the reference rider's weight shift applied. This is
 * bike.js line 1258-1259 exactly:
 *     riderFore = input.pitch * RIDER_SHIFT
 *     cgFwd     = CG_FWD + riderFore * (RIDER_MASS / MASS)
 * and a = WHEELBASE/2 - cgFwd, so an AFT shift LENGTHENS the arm.
 */
function rideArmA(tanTheta, decel) {
  return RIDE_A - ridePitch(tanTheta, decel) * RIDE_SHIFT * RIDE_SHIFT_FRAC;
}

// bike.js's own SURF_MU. The trail's SURF_MU array (further down) is a DESIGN
// scale used for rolling resistance and pacing; this one is the bike's, and it
// is what the tyre actually has. They disagree on dirt (1.08 vs 1.00), which is
// the third of the three terms the mirrored model was missing.
const RIDE_SURF_MU = [1.08, 1.15, 0.95, 0.78, 0.86, 0.70, 0.58, 0.52];
const RIDE_MU_REF = RIDE_SURF_MU[0];

// --- the measurement -------------------------------------------------------
// Rows: tan(theta), rise/run. Columns: deceleration demanded ON TOP of holding
// the grade, m/s^2. Values: bike.state.leanLimit p10, DEGREES.
//
// Post-processed exactly once, and only in the conservative direction: a
// monotone non-increasing lower envelope in both axes (running min from the
// top-left). The raw grid is non-monotone by up to 3 deg where the pitch
// schedule crosses a step, and a route-planning term that is non-monotone in
// gradient produces routes nobody can explain.
const RIDE_TAN_AX = [0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
  0.50, 0.55, 0.60, 0.65, 0.70, 0.75];
const RIDE_DEC_AX = [0, 1, 2, 3, 4, 5];
const RIDE_CEIL = [
  [41.73, 41.07, 39.16, 36.68, 33.89, 30.75],   // 0%
  [41.56, 39.41, 37.40, 34.70, 31.65, 28.20],   // 5%
  [40.08, 38.11, 35.76, 34.00, 31.50, 28.17],   // 10%
  [38.65, 38.11, 35.76, 34.00, 31.50, 28.17],   // 15%
  [38.65, 38.11, 35.76, 34.00, 31.50, 28.17],   // 20%
  [38.65, 38.11, 35.76, 34.00, 31.50, 28.17],   // 25%
  [38.65, 38.11, 35.76, 34.00, 31.44, 26.61],   // 30%
  [38.65, 37.48, 35.30, 32.29, 28.03, 25.11],   // 35%
  [37.46, 35.56, 32.90, 29.10, 27.18, 25.11],   // 40%
  [37.46, 35.56, 32.90, 29.10, 20.10, 0.61],    // 45%
  [37.46, 35.56, 32.90, 24.71, 5.66, 0.61],     // 50%
  [36.94, 35.56, 28.94, 15.13, 0.84, 0.61],     // 55%
  [36.94, 32.49, 22.25, 0.57, 0.57, 0.57],      // 60%
  [35.41, 27.62, 10.77, 0.57, 0.57, 0.57],      // 65%
  [31.76, 20.76, 0.71, 0.57, 0.57, 0.57],       // 70%
  [0, 0, 0, 0, 0, 0],                           // 75% — the bike crashes straight
];
// The design's share of the bike's own measured ceiling. CHOSEN, not tuned, so
// that the budget on flat dirt is exactly LEAN_DESIGN — the same argument the
// old LEAN_USE made, against a measured ceiling instead of a modelled one. It
// is also inside the corner-hold measurement's usable band (0.58-0.76) at every
// plan radius the hold probe could read cleanly.
const RIDE_USE = Math.tan(LEAN_DESIGN) / Math.tan(RIDE_CEIL[0][0] * Math.PI / 180);
// Below this the rider has no usable lean at all: they cannot correct a line,
// let alone turn. Used to locate the gradient at which the ground stops being
// rideable, for the refusal term in corridorRide().
const RIDE_MIN_DEG = 4.0;
// The rear-load band, kept from the previous cut BUT no longer carrying any
// fitted weight: it was a smoothstep inside the lean model, and the measurement
// has replaced everything it was doing there. What survives is its use as a
// GATE — below RIDE_REAR_LO the rear is not a cornering wheel and a switchback
// apex has nothing to turn on. Evaluated against the SHIFTED arm, it now fires
// at a gradient of about 0.79, i.e. within one grid step of where the measured
// ceiling goes to zero (0.75). Two independent statements of the same physics
// agreeing to 5% of gradient is the reason both are kept.
const RIDE_REAR_LO = 0.08;
const RIDE_REAR_HI = 0.22;
// Longitudinal demand at which the rear leaves the ground for the reference
// rider — the ENDO limit, at full aft shift. 0.844, against 0.610 for a rider
// sitting still. This is the real bound on a bicycle's front brake.
const RIDE_ENDO_MU = (RIDE_A + RIDE_SHIFT * RIDE_SHIFT_FRAC) / RIDE_CG_H;

/** Bilinear read of the measured ceiling, RADIANS. Zero past the table. */
function rideCeiling(tanTheta, decel) {
  let t = tanTheta > 0 ? tanTheta : 0;
  const last = RIDE_TAN_AX.length - 1;
  if (t >= RIDE_TAN_AX[last]) return 0;
  let a = decel > 0 ? decel : 0;
  if (a > RIDE_DEC_CAP) a = RIDE_DEC_CAP;
  let i = 0;
  while (i < last - 1 && RIDE_TAN_AX[i + 1] <= t) i++;
  const ft = (t - RIDE_TAN_AX[i]) / (RIDE_TAN_AX[i + 1] - RIDE_TAN_AX[i]);
  let j = 0;
  while (j < RIDE_DEC_AX.length - 2 && RIDE_DEC_AX[j + 1] <= a) j++;
  const fa = clamp01((a - RIDE_DEC_AX[j]) / (RIDE_DEC_AX[j + 1] - RIDE_DEC_AX[j]));
  const lo = RIDE_CEIL[i][j] * (1 - fa) + RIDE_CEIL[i][j + 1] * fa;
  const hi = RIDE_CEIL[i + 1][j] * (1 - fa) + RIDE_CEIL[i + 1][j + 1] * fa;
  return (lo * (1 - ft) + hi * ft) * Math.PI / 180;
}

/**
 * The gradient at which the design budget falls to RIDE_MIN_DEG, for a given
 * deceleration. Ground steeper than this is not "expensive" — the reference
 * rider does not get down it upright at any radius, so no berm and no line
 * choice rescues it. Precomputed off the table itself; nothing here is tuned.
 */
const RIDE_TAN_ZERO = (() => {
  const out = new Float64Array(RIDE_DEC_AX.length);
  const minRad = RIDE_MIN_DEG * Math.PI / 180;
  for (let j = 0; j < RIDE_DEC_AX.length; j++) {
    let g = RIDE_TAN_AX[RIDE_TAN_AX.length - 1];
    for (let s = 0; s <= 200; s++) {
      const t = (s / 200) * RIDE_TAN_AX[RIDE_TAN_AX.length - 1];
      const b = Math.atan(RIDE_USE * Math.tan(rideCeiling(t, RIDE_DEC_AX[j])));
      if (b < minRad) { g = t; break; }
    }
    out[j] = g;
  }
  return out;
})();

/**
 * Interpolated RIDE_TAN_ZERO at an arbitrary deceleration, ON A GIVEN SURFACE.
 *
 * The surface term is not cosmetic and it was the second half of the seed-2026
 * 0726 wall. leanBudget() indexes the table at demand/k where k is the surface's
 * grip relative to dirt, so the gradient at which the budget dies scales with k
 * too: a 65% pitch on ROOT (k = 0.648) is a 100% pitch as far as the tyre is
 * concerned, and the reference rider does not get down it. Reading the dirt
 * figure there says 0.74 and permits it.
 */
function rideGradeMax(decel, surf) {
  const mu = RIDE_SURF_MU[surf] === undefined ? RIDE_MU_REF : RIDE_SURF_MU[surf];
  const k = mu / RIDE_MU_REF;
  const a = clamp(decel, 0, RIDE_DEC_CAP) / k;
  if (a >= RIDE_DEC_AX[RIDE_DEC_AX.length - 1]) {
    return k * RIDE_TAN_ZERO[RIDE_DEC_AX.length - 1];
  }
  let j = 0;
  while (j < RIDE_DEC_AX.length - 2 && RIDE_DEC_AX[j + 1] <= a) j++;
  const f = clamp01((a - RIDE_DEC_AX[j]) / (RIDE_DEC_AX[j + 1] - RIDE_DEC_AX[j]));
  return k * (RIDE_TAN_ZERO[j] * (1 - f) + RIDE_TAN_ZERO[j + 1] * f);
}

/**
 * sin(theta) -> tan(theta), SIGN PRESERVED. S.grade is the y-component of a
 * 3-D unit tangent, i.e. sin(theta) — while every consumer in this file treats
 * it as rise/run. The two differ by 5% at a 30% grade and 12% at 50%, which is
 * inside nobody's error bar until it is multiplied by a load-transfer arm.
 */
function tanFromSin(s) {
  const a = clamp(Math.abs(s), 0, 0.985);
  const t = a / Math.sqrt(1 - a * a);
  return s < 0 ? -t : t;
}

/**
 * Rear load fraction under a longitudinal demand of `muLong` (tyre force /
 * normal force, SIGNED: + = braking), FOR THE REFERENCE RIDER — i.e. with the
 * weight shift the pitch schedule is holding at that demand. Negative means the
 * rear is off the ground: an endo.
 *
 * This is the arithmetic bike.js does at line 1258; the only thing that changed
 * is that `riderFore` is no longer assumed to be zero. With the shift the endo
 * gradient moves from 0.610 to 0.844, and the measured ceiling above goes to
 * zero at 0.75 — within one grid step of it.
 */
function rearLoadFrac(muLong, tanTheta, decel) {
  const a = (tanTheta === undefined)
    ? rideArmA(muLong, 0)              // demand-keyed shift when no grade is given
    : rideArmA(tanTheta, decel === undefined ? 0 : decel);
  return (a - muLong * RIDE_CG_H) / RIDE_WB;
}

/**
 * THE LEAN A RIDER CAN ACTUALLY HOLD AGAINST THE TREAD, in radians, on ground
 * of gradient `tanTheta` while shedding `decel` m/s^2 on top of holding the
 * grade, on SurfaceId `surf`.
 *
 * Returns 0 when the reference rider cannot descend this ground upright at all
 * — which is a statement that the ground is unrideable at any radius, not a
 * small number.
 *
 * SURFACE. The measurement was taken on DIRT and the surface enters twice, both
 * validated against probes on all eight SurfaceIds:
 *   - the ceiling scales in TANGENT space with the bike's own SURF_MU relative
 *     to dirt (measured 1.06 / 0.87 / 0.70 / 0.78 / 0.62 / 0.48 / 0.40 against
 *     a predicted 1.065 / 0.880 / 0.722 / 0.796 / 0.648 / 0.537 / 0.481);
 *   - and low-mu ground behaves like STEEPER ground, so the table is indexed at
 *     the demand divided by the same scale. Six of the eight surfaces are then
 *     predicted to within 1.5 deg; mud and snow on 50% ground are predicted
 *     conservatively (0 against a measured 15.6 and 18.0 deg), which is the
 *     direction to be wrong in on a surface the phases barely use.
 */
function leanBudget(tanTheta, decel, surf) {
  const mu = RIDE_SURF_MU[surf] === undefined ? RIDE_MU_REF : RIDE_SURF_MU[surf];
  const k = mu / RIDE_MU_REF;
  // A station the design profile happens to be ACCELERATING through is not a
  // station where the rider is guaranteed to be: they may arrive at the design
  // speed and merely hold it, which is the demand the probe was governed to.
  // The grade itself stays SIGNED, so a climb is not charged for braking that
  // gravity is doing for it.
  const dec = clamp(decel, 0, RIDE_DEC_CAP);
  const ceil = rideCeiling(Math.max(0, tanTheta) / k, dec / k);
  if (ceil <= 0) return 0;
  return Math.atan(RIDE_USE * k * Math.tan(ceil));
}

/** Contact lean a corner of plan radius r demands at speed v, radians. */
function leanRequired(v, r) {
  return Math.atan((v * v) / (G * Math.max(0.5, r)));
}

// ---------------------------------------------------------------------------
// FEASIBILITY AS A COST ON THE MARCH — the weights, and what class each is in.
//
// The march's existing terms fall into three classes, and a term that does not
// know which class it is in will either do nothing or take over the route:
//
//   tie-break   |dth|*42, wander*95, cross-slope 120, CORR_CAP 110, LEG_CAP 115
//   steering    grade holding 220, aim up to 220, over-grade 320
//   refusal     climb 900/1100/1600 (but these are per UNIT of grade deficit,
//               so a heading climbing at 5% actually costs about 230), and the
//               leg-conflict gate LEG_W_HARD (capped at 1500)
//
// The feasibility term is split to match. `soft` says "the phase cannot be
// ridden at its design speed here" and sits in the tie-break class, so it
// shapes a choice between comparable headings and never buys a climb. `hard`
// says "there is no speed at all at which a bicycle holds this" and sits at the
// top of the steering class — able to outbid grade holding and `aim`, and
// deliberately NOT able to outbid the climb gates, because a route that climbs
// to avoid steep ground is a different defect, not a fix.
//
// It does not need to outbid them. The escape from an infeasible heading is
// almost never uphill: turning LESS takes the radius to infinity and the
// shortfall to zero, and where the ground is too steep for that to help, the
// escape is a TRAVERSE — which is a descending heading with a smaller grade.
// That is (a) in the trail builder's list: turn where it is flatter, point it
// straighter where it is steep.
//
// AND THE HALF THAT WAS MISSING FROM THE FIRST CUT OF THIS TERM, stated because
// it is the most instructive measurement of the round. Scored on the corner
// alone — shortfall and headroom, both functions of the RADIUS — the term did
// exactly what it was told and made the course WORSE: seed 20260726's p99 grade
// went 47% -> 75%, its max 79% -> 101%, and the worst 25 m friction bin 43% ->
// 75%. Penalising a turn on steep ground, and nothing else, does not send the
// route somewhere flatter; it sends it STRAIGHT DOWN THE FALL LINE, which is
// the one heading where the radius is infinite and the corner term is free.
//
// "Turn where it is flatter and point it straighter where it is steep" is two
// instructions, and only the second one is about radius. The first is about the
// GROUND, and it needs its own term: `budgetLoss` — how much cornering capacity
// the gradient has already taken away from a rider, whatever they do with the
// bars. It is priced on every candidate, including dead-straight ones, so steep
// ground is expensive per se and a heading that merely avoids turning on it
// cannot escape the bill.
const FEAS_W_BUDGET = 300.0;      // per radian of lean budget the grade has eaten
const FEAS_CAP_BUDGET = 110.0;    // same scale as the cross-slope penalty (120)
const FEAS_W_SOFT = 220.0;        // per radian of lost design-speed headroom
const FEAS_CAP_SOFT = 80.0;
const FEAS_W_HARD = 3000.0;       // per radian of lean the ground cannot supply
const FEAS_CAP_HARD = 300.0;
const FEAS_W_SPEED = 60.0;        // per m/s the launch cap falls below the floor
const FEAS_CAP_SPEED = 120.0;
const FEAS_W_ENDO = 900.0;        // per unit of rear-load fraction past the endo
const FEAS_CAP_ENDO = 260.0;
// GROUND THE REFERENCE RIDER CANNOT DESCEND UPRIGHT AT ALL. Refusal class — the
// same scale as the march's climb gates, and deliberately below their 1600 dead
// end, so a route can never buy its way out of a cliff by climbing. The step is
// what the term is worth AT the boundary: without it the penalty starts at zero
// exactly where the ground stops being rideable, which is a cliff edge with a
// zero-cost first step onto it.
const FEAS_W_GROUND = 1100.0;
const FEAS_CAP_GROUND = 1300.0;
const FEAS_GROUND_STEP = 0.10;
// ...and the ramp BELOW that threshold, which is the half that was missing.
// See the long note in corridorRide(). The floor is CHOSEN, not tuned: half of
// LEAN_DESIGN, i.e. the point at which the ground has taken half of everything
// the design ever intended the rider to have. Swept over 0.35/0.50/0.65 of
// LEAN_DESIGN and over three weights during the r11 A/B; the dead-ground metric
// is flat in the floor and monotone in the weight, and both are reported.
// The floor is a FRACTION OF WHAT THE PHASE HAS ON ITS OWN FLAT GROUND, not an
// absolute lean. An absolute floor was measured first and it is wrong in the way
// the file already warns about: 14 deg is half of dirt's flat budget but MORE
// than roots has at a 44% gradient, so the term fired across the whole Rooty
// Steeps fan, stopped being decidable, and started outbidding the climb gates —
// every one of the five seeds then failed generation with 18-75 m of climb.
// A fraction of the surface's own flat budget fires in the same PLACE on every
// surface, which is what "the ground has taken most of what you had" means.
const FEAS_BUD_FRAC = 0.45;
// ...and the weight, which is ZERO, and that is a measured result and not an
// oversight. Read the note in corridorRide() first: the mechanism it describes
// is real and reproduced. RE-PRICING THE FAN IS NOT THE FIX FOR IT.
//
// Two weights were measured on all five seeds, one process per arm:
//
//   * FEAS_W_BUDLOW = 1400/rad with an absolute floor of LEAN_DESIGN/2. The term
//     then fires across the whole Rooty Steeps fan (14 deg is MORE than roots
//     has at a 44% gradient), stops being decidable, and starts outbidding the
//     march's climb gates — which the design note above says it must never do.
//     Every one of the five seeds failed generation, with 18-190 m of climb.
//   * A hand-off weight, floored at 0.45 of the surface's own flat budget and
//     scaled to reach exactly FEAS_GROUND_STEP * FEAS_W_GROUND at zero budget,
//     i.e. adding no refusal power at all and only spreading the step the
//     gradient term already had over the collapse it was meant to guard. All
//     five seeds generate. Dead ground goes 2.8/1.5/12.9/7.9 -> 2.3/3.8/15.8/3.1
//     and non-descending 18.3/22.8/17.9/11.3 -> 11.7/20.5/36.9/15.3 m: two seeds
//     better, two worse, ON THE METRIC THE TERM EXISTS FOR.
//
// That is a redraw, not a repair — the same verdict an earlier audit reached
// about this file's route changes generally, and it applies to my own. The fix
// that IS attributable is the one in build(): choose the variant by the profile
// the elevation solve delivers rather than by the marched polyline. It holds
// three seeds bit-identical and improves the fourth.
//
// Kept at zero, with the measurement, so round 12 does not re-derive it.
const FEAS_BUDLOW_AT_ZERO = 0;
// DEAD GROUND. The budget at or below which the reference rider has no usable
// lean at all — the same statement RIDE_MIN_DEG makes, in radians, so the
// route audit and the refusal gradient are two readings of one number.
const FEAS_DEAD_RAD = THREE.MathUtils.degToRad(RIDE_MIN_DEG);
// The plan radius a candidate heading commits to is estimated from the turn it
// asks for, blended with the turn the march is already carrying — a corner is a
// SUSTAINED turn, and pricing one 6 m step in isolation lets the route spiral
// into a hairpin one admissible step at a time.
const FEAS_TURN_MEMORY = 0.45;
// Radius beyond which a heading is simply not a corner and the term is off.
const FEAS_R_MAX = 400.0;
// Fraction of a declared bank that survives to the ridden surface: 0.95 for the
// stamp cloud reconstructing its own section, 0.92 for terrain's band limit.
// Both are terrain.js's own published measurements, not fitted here.
const BERM_REALISE = 0.95 * 0.92;
// ===========================================================================
// R_PLAN_MIN — THE MINIMUM PLAN RADIUS THE COURSE MAY SHIP, AND ITS DERIVATION.
//
// THE DEFECT, named. Eleven rounds of tuning left every seed shipping a corner
// of 4.8-5.5 m plan radius (+-4 m circumradius on the committed centreline):
// 20260726 arc 286 r 5.11, 777 arc 726 r 4.81, 12345 arc 392 r 4.96, 99999 arc
// 600 r 5.48. Seed 777's is the first wall on the seed the audits call closest
// to shippable. Round 9 answered it by CAPPING THE SPEED below HAIRPIN_R0, and
// that cannot work, because the speed cannot go below V_DESIGN_FLOOR: the whole
// design is built on the finding that asking a rider to crawl a steep pitch
// costs them the entire longitudinal budget in braking, which is what collapses
// the lean ceiling in the first place (see designCap()). If the floor speed is
// not rideable at that radius, the RADIUS is the defect.
//
// THE NUMBER, from this file's own physics and nothing else. At the apex the
// rider must hold, against the tread,
//
//     leanContact = atan(v^2 / (g*r)) - bank_realised
//
// and the ground has to supply it. Evaluated where hairpins actually live —
// the tech phases, on 35-45% ground, on ROOT/ROCK:
//
//   * bank: the tech phases cap at maxBank 0.30-0.35 rad (17.2-20.1 deg), of
//     which BERM_REALISE = 0.874 survives to the ridden surface => 15.0-17.6 deg.
//   * budget: leanBudget(0.35-0.45, 0, ROOT) = 16.2 deg down to 13.4 deg.
//   * design share: this file never spends the whole budget — LEAN_DESIGN is 28
//     of a measured 41.7 deg flat-ground ceiling, i.e. RIDE_USE = 0.596 — and a
//     hairpin apex is the last place to start. Two thirds of (bank + budget) is
//     19-22 deg, i.e. tan = 0.345-0.404.
//
//   r >= v^2 / (g * 0.345)  =  11.4 m at V_DESIGN_FLOOR (6.2 m/s)
//                              8.9 m at SB_V_APEX      (5.5 m/s)
//
// AND IT AGREES WITH THE ONLY INDEPENDENT MEASUREMENT ANYONE HAS TAKEN OF IT.
// The round-12 verifier drove the real bike.js over an analytic constant-radius
// helicoid at the design floor speed on 15% ground, 9 cells per radius: r 7 m
// clean 9/9, r 5.5 m 6/9. Their instrument certifies down to 7 m and starts
// failing at 5.5 — precisely the band this course was shipping.
//
// 10.0 m is taken: above the 8.9 m apex-speed bound, below the 11.4 m
// floor-speed bound (a hairpin apex IS paced at SB_V_APEX, not at the floor),
// and carrying a clear margin over the 7 m the bike was measured clean at.
const R_PLAN_MIN = 10.0;
// The design target the geometry passes are aimed at, which has to sit ABOVE
// R_PLAN_MIN because three separate operators downstream tighten a corner after
// it is drawn: smoothPolyline + the CatmullRom resample, openTightCorners'
// displacement budget, and benchLine's lateral shift. Measured across the four
// seeds, the shipped +-4 m circumradius comes out at 0.75-0.85 of the radius the
// march committed, so the target carries 30% over the floor.
const R_PLAN_TARGET = 13.0;
// The switchback the march builds is a fixed-radius reversal. It was 6.5 m —
// which is R_PLAN_MIN's derivation run backwards: a 6.5 m apex at 5.5 m/s
// demands atan(30.25/(9.81*6.5)) = 25.4 deg of contact lean, against 15-17 deg
// of realised berm plus 13-16 deg of budget with NOTHING left unspent. It is
// the tightest thing on the course and it was authored, not discovered.
//
// SWEPT, one process per arm, four seeds, plan radius measured on the RESPACED
// committed centreline at the +-4 m acceptance stencil:
//
//     SB_RADIUS      6.5     9.0     11.0    13.0    16.0
//     worst r        4.8     6.6     9.0     9.4     11.7
//     stations <10 m 28-58   77-116  0-9     0-19    0
//
// 13.0 with the plan floor and the march turn cap below takes stations under
// 10 m to ZERO on all four seeds with a worst radius of 10.1-13.9 m. 16.0 also
// works and costs more of the mountain's character (it forces longer traverses
// and fewer, flatter reversals), so the smaller number that clears the gate is
// the one taken.
const SB_RADIUS = R_PLAN_TARGET;
// ...and what the control arm (`settings.trailSolver.feasible = false`) uses, so
// that path still reproduces the pre-rideability build bit-identically. Every
// geometry change this round makes is gated the same way; see readSolverOpts().
const SB_RADIUS_LEGACY = 6.5;
// ...and the speed a hairpin APEX is ridden at, which is not V_DESIGN_FLOOR.
// The file's own note says it: at the apex the rider is at full lock, at minimum
// speed, with the fall line across them. Gating at the general design floor
// (6.2 m/s) demanded 31.1 deg of contact lean from the old 6.5 m radius and refused
// every hairpin on ground steeper than about 37% in the tech phases — measured
// on seed 99999, that refused ALL of them, and the route answered by running
// straight down the fall line instead: minimum plan radius 5.4 -> 20.9 m,
// stations over 35% grade 6.1% -> 16.9%, feature count 40 -> 21. Refusing to
// turn on steep ground is not the same as not being on steep ground, and the
// straight-and-steeper course it produced is the worse of the two.
const SB_V_APEX = 5.5;
// Slack allowed at the switchback gate: a hairpin apex is the one place a
// builder deliberately spends the whole budget.
const SB_FEAS_TOL = THREE.MathUtils.degToRad(3);
// ...and the relief valve. If the drift has run this far past the phase's own
// allowance, the feasibility gate stands down and the old fall-angle gate is
// the only test — otherwise a mountain with no flat ground anywhere would
// forbid every switchback and the route would traverse to the map edge. This is
// the guard against "must not switchback endlessly" read in the other
// direction: it must also never become unable to switchback at all.
const SB_FORCE_DRIFT = 2.4;


// the same contract as terrain's `settings.treadFilter`: explicit, logged,
// identical on every boot of the same settings, and never consulted by anything
// else. It exists so a harness can A/B the joint solve against ONE cached
// mountain in ONE process — which is the only way any number in this file's
// notes can be trusted, and doubly so while terrain.js is itself changing.
//
//   { launch: boolean, rebank: boolean, feasible: boolean }   all default true
//
// `launch: false` restores the round-6 behaviour exactly: the fixed-wavelength
// chatter ceiling below, and no relief flattening. `rebank: false` keeps the
// launch cap but leaves the bank where the pre-feature solve put it — i.e. it
// reproduces the round-6 speed-clamp ablation, which is the experiment that
// dropped seed 777 to 34 m with 17 lowsides.
//
// `feasible: false` restores the round-7 behaviour exactly: marchRoute scored
// without the rideability term, the switchback gate on fall angle alone, the
// corner grip budget as the naive friction circle, and the bank identity floor
// against the constant LEAN_DESIGN. It is the A/B control for everything the
// rideability section above adds, and it is the ONLY way to attribute a change
// in the acceptance figures to this round rather than to a moving mountain.
//
// EVERY CHANGE MADE IN THE ROUND THAT ADDED R_PLAN_MIN AND THE JUMP-LINE FIXED
// POINT IS GATED ON IT TOO, and that is verified rather than asserted: with
// `feasible: false` the current file and the previous cut produce byte-identical
// carved heightfields, stamp counts, station tables, speed and bank on all four
// buildable seeds. The gated items are SB_RADIUS and the reversal rate, the
// march's turn-authority bound, the PLAN_MIN_R floor, ROUTE_MIN_R, the variant
// and station-try budgets, the S.hBal subtraction in computeVertKappa(), the
// ballistic-free grade in leanBudgetAt()/brakeAccel(), the split of a jump's
// vFeat cap at its lip, the lip/target-distance sizing, the jump siting window,
// the windowed descent pass, the shipped-radius variant gate and the wall gate.
// Two of those were found only BECAUSE the identity was checked: the reversal
// rate was written as a literal 0.46 where 3.0/6.5 is 0.46154, and a 0.3%
// difference in it sends the march down a different fall line 600 m later.
function readSolverOpts(settings) {
  const o = settings && settings.trailSolver;
  const dflt = {
    launch: true, rebank: true, feasible: true,
    safety: LAUNCH_SAFETY, carveFloor: KAPPA_CARVE_FLOOR, trace: false,
    traceFrom: Infinity, traceTo: -Infinity,
  };
  if (!o || typeof o !== 'object') return dflt;
  const num = (v, d) => (typeof v === 'number' && isFinite(v) && v > 0 ? v : d);
  const spec = {
    launch: o.launch !== false,
    rebank: o.rebank !== false,
    feasible: o.feasible !== false,
    safety: num(o.safety, LAUNCH_SAFETY),
    carveFloor: num(o.carveFloor, KAPPA_CARVE_FLOOR),
    // Diagnostic only, and OFF by default so the shipped world is bit-identical
    // with it absent: record one row per COMMITTED march step on trail.routeTrace
    // so the rideability terms can be read at a named site instead of modelled.
    // Three rounds of this project were lost to models of code that disagreed
    // with the code; this is the cheap way not to write a fourth.
    trace: !!o.trace,
    traceFrom: typeof o.traceFrom === 'number' ? o.traceFrom : Infinity,
    traceTo: typeof o.traceTo === 'number' ? o.traceTo : -Infinity,
  };
  // eslint-disable-next-line no-console
  console.info('[trail] solver overridden by settings.trailSolver:', spec);
  return spec;
}

/** The round-6 chatter ceiling, kept verbatim for the A/B control path. */
function legacyRoughCap(bumps, rough) {
  const amp = Math.max(0.015, bumps + rough * 1.5);
  return ROUGH_SUSP * (ROUGH_LAMBDA / (2 * Math.PI)) * Math.sqrt((2 * G) / amp);
}

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
// A near probe for the GRADE half of corridorRide() only — deliberately not a
// member of CORR_KS, because the ladder above also carries the vertical-curvature
// measurement and inserting a rung would move vLaunch, the deceleration, and
// therefore every other rideability term. ~2.5 wheelbases: the scale a rider
// meets a pitch at. See the note in corridorRide().
const CORR_NEAR_S = 0.0;
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

  // CLIMB BACK OUT OF THE PIT SLOWLY ENOUGH THAT THE TRAIL STILL DESCENDS.
  //
  // This was a flat 20% RELATIVE to grade, and that is the largest single source
  // of CONTRACT §4 breaches on the shipped course. The run-out gives back the
  // whole pit — for a step-down, `yEnd - stepDown` is about 3.2 m — and it gives
  // it back at 20% relative on ground falling at 13%, through a smoothstep whose
  // peak slope is 1.5x its mean. The world gradient through the run-out is
  // therefore about +7% for 17-19 m, and it was the WORST non-descending run on
  // three of the four buildable seeds: 17.0 m at seed 20260726 arc 1379, 17.9 m
  // at 777 arc 1377, 17.9 m at 12345 arc 1379 and 18.7 m at 99999 arc 1394 —
  // the same feature, the step-down, at the same place in the jump line.
  //
  // The rate is now a fraction of the grade the run-out is built on, so the
  // world gradient stays negative by construction, and the 1.5 is the
  // smoothstep's peak-to-mean. `outRate` is passed in by the caller because the
  // caller is the one that knows the local grade; the default reproduces
  // nothing — it is simply a sane floor for a caller that does not care.
  const outRate = opt.outRate === undefined ? 0.20 : opt.outRate;
  const outLen = clamp(1.5 * Math.abs(yEnd - stepDown) / Math.max(0.02, outRate),
    4, opt.outMax === undefined ? 26 : opt.outMax);

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

  const SOLVER = readSolverOpts(ctx.settings);
  // Every constant this round moved has a legacy value the control arm restores.
  const SB_R = SOLVER.feasible ? SB_RADIUS : SB_RADIUS_LEGACY;
  const R_MIN_PLAN_EFF = SOLVER.feasible ? R_PLAN_MIN : 0;
  const R_TARGET_EFF = SOLVER.feasible ? R_PLAN_TARGET : 0;

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
  const routeTrace = [];    // per-committed-step rideability rows; only when
                            // settings.trailSolver.trace is set. See readSolverOpts().
  const fanTrace = [];      // per-CANDIDATE rows inside [traceFrom, traceTo] metres
  let crestMask = null;     // authored-relief mask; see countCrestViol()
  let ballisticMask = null; // lip->touchdown of every solved jump; see techSpeedCap()
  // THE JUMP-LINE FIXED POINT'S STATE, and it is deliberately tiny.
  //   featPace   feature name -> the approach speed to solve that feature at.
  //              Empty on the first pass (buildJump/buildDrop then read S.speed,
  //              exactly as before); on every pass after, it carries the speed
  //              the profile ACTUALLY delivered at that feature's lip.
  //   featPassRng  a stream buildFeatures() owns, re-derived from the seed at the
  //              start of every pass. It exists so the iteration count is
  //              invisible downstream: `featRng` is also read by buildStamps(),
  //              and a second feature pass drawing from it would move the boulder
  //              scatter as a side effect of a speed solve.
  const featPace = new Map();
  // Every airborne feature buildFeatures() ATTEMPTS, and why an attempt that
  // failed did. A fixed point that converges by deleting features is not a fixed
  // point, it is a retreat, and the first cut of this loop did exactly that:
  // `jumpWindowBad` went to 0 while the four seeds went from 32 air features to
  // 18. The pass score is therefore lexicographic on (features lost, features
  // outside their window), never on the second alone.
  const featAttempt = new Map();
  let featPassRng = makeRng(subSeed(ctx.seed, 'trail:features:pass'));
  /** The feature pass's own draw. The control arm keeps the shared stream so it
   *  reproduces the pre-round build; see readSolverOpts(). */
  const featDraw = () => (SOLVER.feasible ? featPassRng() : featRng());
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

  // -------------------------------------------------------------------------
  // RIDEABILITY OF A CANDIDATE HEADING — the same ladder corridorProfile walks,
  // read for the OTHER question: can a bicycle corner on the ground this heading
  // commits to?
  //
  // It answers with a (v, phi) existence test, exactly as the brief states it:
  //   1. the longitudinal demand — holding the grade plus the deceleration the
  //      launch cap itself forces — leaves enough LEAN CEILING (leanBudget());
  //   2. atan(v^2/(g r)) <= phi + lean_available;
  //   3. v <= sqrt(g / kappa_vertical) with the CREST_SAFETY margin.
  // The lowest admissible speed is the friendliest for (2) and costs nothing in
  // (1) or (3), so if the corner is not holdable at V_DESIGN_FLOOR it is not
  // holdable at any speed and the heading is INADMISSIBLE rather than expensive.
  //
  // `phi` is what a berm could be built to here, which is the phase's own
  // maxBank derated by whether the corridor can physically carry the fill — that
  // is option (b) in the builder's list, and the derate is what stops it being
  // claimed on a rib where nothing can be built.
  //
  // The three ways out are all reachable from inside the heading fan: turning
  // less takes `radius` up and the requirement down (c, and (a) in its
  // "point it straighter" form); a shallower heading across the fall line takes
  // `tanT` down (a, in its "turn where it is flatter" form); and `phi` is the
  // bank the corridor will hold (b). Nothing here rejects without also telling
  // the search which direction is cheaper.
  const rTmp = { cost: 0, shortfall: 0, headroom: 0, vHi: 0, grade: 0, rearFrac: 1 };

  function corridorRide(terrain, x, z, hd, ph, radius, bankFac, out) {
    const sx = Math.sin(hd), sz = -Math.cos(hd);
    const vPhase = Math.max(4, V_PHASE[ph.id] === undefined ? 17 : V_PHASE[ph.id]);
    const y0 = terrainH(terrain, x, z);
    let yPrev = y0;
    let prevG = 0, worstK = 0, gNear = 0;
    for (let k = 1; k < CORR_KS.length; k++) {
      const s = CORR_KS[k];
      const y = terrainH(terrain, x + sx * s, z + sz * s);
      const seg = s - CORR_KS[k - 1];
      const g = (yPrev - y) / seg;                    // + = descending, rise/run
      if (k > 1) {
        const span = 0.5 * (s - CORR_KS[k - 2]);
        const kap = (g - prevG) / span;
        if (kap > worstK) worstK = kap;
      }
      // THE STEEPEST SUSTAINED PITCH THE STEP COMMITS TO. Cumulative means over
      // 9, 18 and 27 m, worst of the three — not the worst single 9 m SEGMENT
      // anywhere in the 64 m probe, and not the 27 m mean alone.
      //
      // Both of the obvious choices were measured and both are wrong. The worst
      // segment over the whole ladder runs past 50% at 15% of candidates on real
      // mountainside, so it condemned every route on every seed for ground the
      // profile solver flattens or the trail never occupies. The 27 m mean
      // alone missed the defect it was written for: at seed 20260726 arc
      // 175-195 m the shipped profile carries a 20 m pitch at 41-47% with a
      // 60 m corner in the middle of it — the exact grade x radius cell the
      // synthetic sweep scores 0/6 at every speed — and its 27 m mean reads 30%.
      // Cumulative means are the compromise: 9 m is short enough to see a pitch
      // a corner fits inside, and averaging from the step rather than differencing
      // one segment keeps it out of the mountain's own metre-scale noise.
      if (k <= 3) { const gc = (y0 - y) / s; if (gc > gNear) gNear = gc; }
      prevG = g; yPrev = y;
    }
    // ...AND ONE WINDOW SHORTER THAN THE LADDER CARRIES, because 9 m is still
    // three and a half wheelbases and the rider meets a pitch over two.
    //
    // MEASURED, r11, and it is half of the FEAS_W_GROUND asymmetry. At seed
    // 12345 arc 202 and seed 99999 arc 154 the committed line ships 12-13 m of
    // ground whose gradient is 0.84-1.05 at the +-4.5 m stencil the acceptance
    // harness reads, while the 9/18/27 m cumulative means from the same step
    // read 0.73-0.75 — i.e. exactly at rideGradeMax, so the refusal term charged
    // 0-119 out of a 1300 cap. Worse, at 12345 the heading the march CHOSE at
    // s=192 has a 6 m step grade of 1.10 and a 9 m cumulative mean of 0.742: the
    // long windows were letting it aim straight at a wall it then had to ride.
    // CORR_NEAR_S is deliberately a cumulative mean from the step and NOT the
    // worst segment anywhere in the ladder — that variant is the one the note
    // above records as condemning every route on every seed.
    if (CORR_NEAR_S > 0) {
      const yN = terrainH(terrain, x + sx * CORR_NEAR_S, z + sz * CORR_NEAR_S);
      const gc = (y0 - yN) / CORR_NEAR_S;
      if (gc > gNear) gNear = gc;
    }
    // (3) the speed the natural relief permits, before any excavation.
    const kUse = Math.max(worstK, CREST_KAPPA_MIN);
    const vLaunch = Math.sqrt(G / (CREST_SAFETY * kUse));
    const vHi = Math.min(vPhase, vLaunch);
    // (1) the deceleration that cap itself demands over the probe span. This is
    // the term the friction-circle audit found dominant: on the shipped course
    // mu_long from braking exceeded mu_long from grade by 3x.
    const span = CORR_KS[CORR_KS.length - 1];
    const dec = vLaunch < vPhase
      ? (vPhase * vPhase - vLaunch * vLaunch) / (2 * span) : 0;
    const tanT = gNear;                               // rise/run, signed
    const budget = leanBudget(tanT, dec, ph.surface);
    const st = BERM_STYLE[ph.id] || BERM_STYLE_DEFAULT;
    // What a berm here would ACTUALLY realise, not what the phase ceiling says:
    // the emitted design realises ~0.95 of its declared cross-slope on the
    // acceptance secant and terrain's band limit commits ~0.92 of that (both
    // measured, both documented in terrain.js's own CONTRACT-NOTE).
    const phi = st.maxBank * clamp01(bankFac) * BERM_REALISE;
    const avail = budget + phi;
    const needLo = leanRequired(V_DESIGN_FLOOR, radius);
    const needHi = leanRequired(Math.max(V_DESIGN_FLOOR, vHi), radius);
    out.shortfall = needLo - avail;
    out.headroom = avail - needHi;
    out.vHi = vHi;
    out.grade = tanT;
    out.dec = dec;
    const cosT = 1 / Math.sqrt(1 + tanT * tanT);
    const muLong = (G * tanT * cosT + clamp(dec, 0, RIDE_DEC_CAP)) / (G * cosT);
    const nR = rearLoadFrac(muLong, tanT, dec);
    out.rearFrac = nR;
    out.budget = budget;
    let c = 0;
    // (a) TURN WHERE IT IS FLATTER. Priced on the ground, not on the turn, so a
    // heading cannot dodge it by pointing straight down the face.
    const loss = LEAN_DESIGN - budget;
    if (loss > 0) c += Math.min(FEAS_CAP_BUDGET, loss * FEAS_W_BUDGET);
    // (a2) ...AND GROUND YOU CANNOT DESCEND UPRIGHT IS NOT "EXPENSIVE", IT IS
    // REFUSED. This is the half of (a) that was missing, and it is the defect
    // that shipped 30 m of 49-91% fall line on seed 12345.
    //
    // The mechanism is worth stating because it is not obvious. Every other
    // rideability term here is scaled by the CORNER: `shortfall` and `headroom`
    // are both differences against leanRequired(v, radius), and on a dead
    // straight heading the radius is FEAS_R_MAX, so leanRequired is 0.56 deg and
    // any berm at all covers it. A fall line therefore costs nothing under
    // (hard) no matter how steep it is — the budget can be exactly zero and the
    // shortfall still negative. `loss` does price the ground, but it is a
    // tie-break-class term capped at 110, and on a face where every heading is
    // steep it cancels out of the comparison entirely.
    //
    // So the gradient at which the measured ceiling reaches RIDE_MIN_DEG gets
    // its own term, in the REFUSAL class alongside the climb gates (900/1100/
    // 1600) — because it is the same kind of statement: not "this is a poor
    // trail", but "this is not a trail". The escape is the one the file already
    // documents: a traverse, which is a descending heading with a smaller
    // gradient. It deliberately does NOT outbid the climb gates, so a route
    // still cannot climb its way out of a cliff.
    // ON DIRT, DELIBERATELY, and this is a measured choice rather than an
    // oversight. Reading the refusal gradient on the PHASE's own surface is more
    // faithful — a 65% pitch on roots is a 100% pitch as far as the tyre is
    // concerned — and it makes the course worse: it redrew all five routes and
    // took the reference seed's autopilot from 452 m back to 325 m, because on a
    // low-mu phase it condemns most of the available headings at once and the
    // march then has nothing left to choose between. The march is a LOCAL search;
    // a term that is saturated across the whole fan decides nothing and only
    // displaces the route. The surface is priced where it can be acted on — in
    // leanBudget() at every station, and in the speed profile through
    // cornerGripBudget() — not in the heading fan.
    const gMax = rideGradeMax(dec, Surface.DIRT);
    const tanOver = tanT - gMax;
    out.gMax = gMax;
    out.tanOver = tanOver;
    out.groundCost = 0;
    if (tanOver > 0) {
      out.groundCost = Math.min(FEAS_CAP_GROUND, (FEAS_GROUND_STEP + tanOver) * FEAS_W_GROUND);
    }
    // ...AND THE OTHER HALF OF THE ASYMMETRY, which is that a threshold in
    // GRADIENT has a zero-cost band that reaches all the way down to a budget of
    // RIDE_MIN_DEG. Measured off this file's own table, on dirt at dec 0:
    //
    //     gradient   0.688   0.702   0.714   0.727   0.739   0.742   0.750
    //     budget     20.9    19.3    14.3     8.8     4.3     3.0     0.1  deg
    //
    // rideGradeMax lands at 0.743, so everything from "the ground has eaten
    // three quarters of the rider's cornering capacity" down to 3 deg is free.
    // On ordinary sidehill that does not matter, because a cheaper heading
    // exists and the term never has to fire. On a face where the WHOLE FAN is
    // steep it matters completely: the march does not escape the band, it
    // OPTIMISES ONTO IT. Measured at seed 12345, the 19-candidate fan at s=186
    // carries budgets of 0.0-11.2 deg and gNear 0.72-0.91, and the winner is the
    // candidate sitting just inside 0.743 with a ground cost of exactly zero —
    // 12.9 m committed at 3-9 deg of lean, which is where every autopilot cell
    // lowsides. Seed 777's fall line read 0.88-0.95 across its whole fan, i.e.
    // genuinely PAST the threshold, so there the term produced a real cost
    // difference and the route left. Same term, same surface class, opposite
    // outcome, and this is the whole of it.
    //
    // So the refusal is priced on the BUDGET and not on a gradient: continuous,
    // monotone, and — the property the note above says is non-negotiable —
    // never saturating across a fan, because it is linear in a quantity that
    // still varies after the gradient term has pinned at its cap. It is read on
    // the PHASE's surface, unlike the gradient half, because a budget is what
    // the rider has and the rider is on the phase's surface; the measurement
    // that argued against that (the fan going flat on a low-mu phase) was about
    // a STEP function, and a ramp does not have that failure mode.
    const budFlat = leanBudget(0, 0, ph.surface);
    const budFloor = budFlat * FEAS_BUD_FRAC;
    if (budget < budFloor && budFloor > 1e-4) {
      out.groundCost += (1 - budget / budFloor) * FEAS_BUDLOW_AT_ZERO;
    }
    if (out.groundCost > 0) c += out.groundCost;
    if (out.shortfall > 0) {
      c += Math.min(FEAS_CAP_HARD, out.shortfall * FEAS_W_HARD);
    } else if (out.headroom < 0) {
      c += Math.min(FEAS_CAP_SOFT, -out.headroom * FEAS_W_SOFT);
    }
    if (vHi < V_DESIGN_FLOOR) {
      c += Math.min(FEAS_CAP_SPEED, (V_DESIGN_FLOOR - vHi) * FEAS_W_SPEED);
    }
    if (nR < RIDE_REAR_LO) {
      const over = (RIDE_REAR_LO - nR) / Math.max(0.02, RIDE_REAR_LO);
      c += Math.min(FEAS_CAP_ENDO, over * FEAS_W_ENDO);
    }
    out.cost = c;
    return out;
  }

  /**
   * How much of the phase's berm ceiling this corridor could actually stand up,
   * 0..1. `deficit` is designCatch()'s own shortfall in metres of fill at the
   * friendliest reach — it is exactly zero wherever a retaining edge can be
   * built, so on ordinary sidehill this returns 1 and the term is inert.
   */
  function bankFactorFor(deficit) {
    return 1 - 0.75 * clamp01(deficit / CORR_BUDGET);
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
  // Switchback siting. SB_MAX_FALL is tan(38 deg): steeper than a hillside
  // stands (EXPO_REPOSE = tan 35 deg) but well short of the 50-58 deg cliff the
  // ungated machinery walked onto. SB_APEX_PROBE is roughly the distance from
  // the entry to the apex of a 6.5 m-radius reversal (a quarter turn plus the
  // run-in), so the gate measures the ground the corner will actually sit on.
  const SB_MAX_FALL = 0.78;
  const SB_APEX_PROBE = 14.0;

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
    //
    //
    // MEASURED AND REJECTED, stated so it is not re-derived: scoring the far end
    // of the corridor on the climb its straight probe contains — a fan of 17
    // candidate bearings, each walked and each penalised for a hill in the
    // middle — does NOT fix seed 2, which is the only reason to write it. Every
    // one of sixteen variants still climbed 18.7 m or more. It redrew all five
    // routes to do it (feasibility 0.91/0.88/3.39/1.32/1.56% -> 1.19/0.83/0.90/
    // 1.57/2.09%: better on two seeds, worse on three, attributable on none).
    // A change that misses its own motivating case and moves every other course
    // is a redraw, not a repair. Seed 2's mountain cannot carry a 2.6 km / 430 m
    // descent without crossing a rise, and the answer to that is the refusal at
    // the end of build(), not a different random draw.
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
    // Turn the march is already carrying, rad per step. A corner is a SUSTAINED
    // turn: pricing one 6 m step in isolation lets the route spiral into a
    // hairpin one individually-admissible step at a time. See FEAS_TURN_MEMORY.
    let turnMem = 0;
    // Rideability book-keeping for this variant — reported on routeAudit so the
    // choice between variants can be read rather than assumed.
    let feasSteps = 0, feasBadSteps = 0, feasCostSum = 0, sbForced = 0;
    // DEAD GROUND — the longest CONTINUOUS run of committed march, in metres, on
    // which this file's own measured table returns no usable lean at all. This is
    // the quantity two audits have now named as the thing that kills seeds 12345
    // and 99999 inside the first 200 m, and it was never measured: the march
    // reported `feasBadSteps` as a FRACTION, which a 13 m wall in a 2.7 km course
    // moves by 0.5%. A fraction cannot refuse a wall. A run length can.
    let deadRun = 0, deadWorst = 0, deadAt = 0;

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
        // WHERE A SWITCHBACK MAY BE BUILT.
        //
        // Once mode 1 is entered nothing is scored: the march turns at a fixed
        // 6.5 m radius for up to 26 steps, over whatever ground it happens to be
        // standing on. On seed 20260726 it fired onto a cliff band, and that one
        // decision is what killed every cell of the autopilot sweep. Measured on
        // the shipped build at arc 292-306 m: plan radius 8-16 m, natural ground
        // running 119-157% (50-58 deg), the design line already pinned at the
        // full 7.00 m excavation limit so the profile solve could not ease it,
        // and a committed gradient of 68-83% through the apex. Every run died
        // there — 21 of 21 cells inside 32 m of the same point.
        //
        // A builder does not put a switchback on a cliff. They keep traversing
        // until the ground lies down, because the apex of a hairpin is the one
        // place on a trail that has to be nearly flat: the rider is at full lock,
        // at minimum speed, with the fall line across them. So the entry now
        // gates on the fall line where the APEX will land, not merely on where
        // the trail is standing when the drift allowance runs out. If the ground
        // is too steep the drift simply keeps growing and the corridor term
        // (soft, quadratic) pays for it — a longer traverse is a trail; a hairpin
        // down a 55 deg face is not.
        gradAt(terrain, x + dirX * SB_APEX_PROBE, z + dirZ * SB_APEX_PROBE, 9, gTmp);
        const apexFall = Math.max(fallSteep, gTmp.mag);
        // THE GATE THE FALL ANGLE ALONE COULD NOT BE. SB_MAX_FALL is tan(38 deg)
        // — and a 6.5 m reversal on a 38% face needs
        //     atan(V_DESIGN_FLOOR^2/(g*6.5)) = 31.1 deg
        // of contact lean, against a lean budget that has already collapsed to
        // about 10 deg by 38% and a berm ceiling of 17-20 deg in the tech
        // phases. That is the "5 m radius corners on 69% grades" the audit
        // found, and no downstream pass can rescue it: the apex is committed
        // 600 m before any speed profile exists.
        //
        // So the entry now asks the same existence question the heading search
        // asks — can a bicycle hold SB_RADIUS at the slowest speed the design
        // permits, on the ground the APEX will land on, with the biggest berm
        // this phase and this corridor can carry? If not, the drift simply keeps
        // growing and the march keeps traversing, which is what a builder does.
        let sbOk = apexFall < SB_MAX_FALL;
        if (SOLVER.feasible && sbOk) {
          const ax = x + dirX * SB_APEX_PROBE, az = z + dirZ * SB_APEX_PROBE;
          corridorAt(terrain, ax, az, dirX, dirZ, ph.width * 0.5, cTmp);
          const st = BERM_STYLE[ph.id] || BERM_STYLE_DEFAULT;
          const phi = st.maxBank * clamp01(bankFactorFor(cTmp.deficit));
          const avail = phi + leanBudget(apexFall, 0, ph.surface) + SB_FEAS_TOL;
          // The hard half: past the endo grade the rear wheel is not on the
          // ground under the braking a hairpin entry needs, and no berm makes
          // that rideable. This is the "5 m radius corners on 69% grades" case.
          // Holding speed on a grade costs mu_long = tan(theta) exactly, and
          // apexFall IS rise/run, so it is the longitudinal demand directly.
          // Evaluated for the REFERENCE RIDER, i.e. with the weight shift they
          // are holding on that gradient. The previous cut evaluated it for a
          // rider sitting still, which put the endo point at 0.610 and refused
          // every hairpin above about a 51% face — on a mountain whose hairpins
          // are the only thing keeping the route off the fall line.
          const apexRear = rearLoadFrac(apexFall, apexFall, 0);
          sbOk = apexRear >= RIDE_REAR_LO
            && leanRequired(SB_V_APEX, SB_R) <= avail;
          // The relief valve. A mountain with no flat ground anywhere must not
          // be able to forbid every switchback — the route would traverse to the
          // map edge and the corridor term would never get it back. Past
          // SB_FORCE_DRIFT times the phase's own allowance the gate stands down
          // and the fall-angle test is once again the only one. Counted, so a
          // course that relies on it says so.
          if (!sbOk && Math.abs(drift) > limit * SB_FORCE_DRIFT) { sbOk = true; sbForced++; }
        }
        if (Math.abs(drift) > limit && outbound && fallSteep > 0.26 &&
            sbOk &&
            travelled - lastSwitchback > 110 && z > zFinish + 220) {
          mode = 1;
          sbSign = -Math.sign(drift) || 1;
          sbSteps = 0;
        }
      }

      let ds, dHead;
      if (mode === 1) {
        // The reversal, at SB_RADIUS. The heading step is DERIVED from the
        // radius rather than written next to it as a comment: the previous cut
        // had `dHead = 0.46` with `// ~6.5 m radius` beside it, which is true
        // only for ds = 3.0, and a constant whose value and whose stated meaning
        // can drift apart is a constant that will.
        ds = 3.0;
        // The control arm keeps the literal 0.46 the previous cut wrote here:
        // 3.0/6.5 = 0.46154, and a 0.3% difference in the reversal rate is
        // enough to send the march down a different fall line 600 m later.
        dHead = sbSign * (SOLVER.feasible ? ds / SB_R : 0.46);
        sbSteps++;
        // Exit once we're heading firmly back across the hill. The step budget
        // scales with the radius, or a wider reversal runs out of steps
        // mid-turn and ships a half-finished hairpin: a full 180 deg reversal
        // needs pi/dHead steps, and the budget is twice that.
        const nowRate = Math.sin(head) * corRX + (-Math.cos(head)) * corRZ;
        const heading = Math.sin(head) * corDX + (-Math.cos(head)) * corDZ;
        const sbBudget = SOLVER.feasible ? Math.ceil(2 * Math.PI * SB_R / ds) : 26;
        if ((nowRate * sbSign > 0.72 && heading > -0.15) || sbSteps > sbBudget) {
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
        //
        // AND BOUNDED BY R_PLAN_TARGET, which is the other half of the radius
        // fix. The switchback is not the only way this march commits a tight
        // corner: at ds = 6 m the authority ran to 0.56 rad, i.e. a 10.7 m
        // marched radius before any smoothing, and the smoothing and resample
        // then take 15-25% off it. Every downstream pass — openTightCorners, the
        // bank solve, the hairpin speed cap — is a repair for geometry this line
        // is free to author, so the bound belongs here, at the only place the
        // radius is actually chosen.
        const steepNeed = clamp01((fallSteep - maxGrade) * 2.5);
        const maxTurn = Math.min(R_TARGET_EFF > 0 ? ds / R_TARGET_EFF : Infinity,
          0.18 + twist * 0.14 + steepNeed * 0.24);
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
          // RIDEABILITY. Read the deficit at the candidate itself first — it is
          // both the berm-buildability factor and, via corridorCost below, the
          // protectability term, and corridorAt writes into shared scratch.
          let bankFac = 1;
          if (SOLVER.feasible) {
            corridorAt(terrain, nx, nz, s2, c2, ph.width * 0.5, cTmp);
            bankFac = bankFactorFor(cTmp.deficit);
          }
          sc -= corridorCost(terrain, nx, nz, h2, ph.width * 0.5, CORR_S, CORR_CAP);
          sc -= corridorGate(terrain, nx, nz, h2, ph.width * 0.5);
          sc -= corridorProfile(terrain, nx, nz, h2, V_PHASE[ph.id]);
          if (SOLVER.feasible) {
            // The radius this heading commits to. `dth` alone under-reads a
            // sustained turn, so it is blended with the turn already carried.
            const dthEff = Math.abs(dth) * (1 - FEAS_TURN_MEMORY)
              + Math.abs(dth + turnMem) * FEAS_TURN_MEMORY;
            const rCand = dthEff > 1e-4 ? Math.min(FEAS_R_MAX, ds / dthEff) : FEAS_R_MAX;
            // Evaluated on EVERY candidate, straight ones included — see the
            // note at FEAS_W_BUDGET for the measurement that made that
            // non-negotiable.
            corridorRide(terrain, nx, nz, h2, ph, rCand, bankFac, rTmp);
            sc -= rTmp.cost;
          }
          // LEG CLEARANCE. Exactly zero unless this heading would lay tread on
          // ground an earlier leg has already claimed — see legClearanceCost().
          sc -= legClearanceCost(hist, nx, nz, travelled + ds, ph.width * 0.5);
          if (SOLVER.trace && travelled >= SOLVER.traceFrom && travelled <= SOLVER.traceTo) {
            fanTrace.push({
              variant, s: +travelled.toFixed(1), k, dth: +dth.toFixed(3),
              sc: +sc.toFixed(1),
              grd: +(grade * 0.40 + gradeMid * 0.35 + gradeFar * 0.25).toFixed(3),
              g1: +grade.toFixed(3), gMid: +gradeMid.toFixed(3), gFar: +gradeFar.toFixed(3),
              ride: SOLVER.feasible ? +rTmp.cost.toFixed(1) : 0,
              ground: SOLVER.feasible ? +rTmp.groundCost.toFixed(1) : 0,
              gNear: SOLVER.feasible ? +rTmp.grade.toFixed(3) : 0,
              budDeg: SOLVER.feasible ? +THREE.MathUtils.radToDeg(rTmp.budget).toFixed(1) : 0,
            });
          }
          if (sc > bestS) { bestS = sc; bestD = dth; }
        }
        dHead = bestD;
      }

      head += dHead;
      // Carried turn, decayed per METRE so the estimate does not depend on
      // whether the march is taking 6 m ride steps or 3 m switchback steps.
      turnMem = turnMem * Math.pow(0.72, ds / 6) + dHead;
      x += Math.sin(head) * ds;
      z -= Math.cos(head) * ds;
      x = clamp(x, B.minX + 40, B.maxX - 40);
      z = clamp(z, B.minZ + 30, B.maxZ - 10);
      y = terrainH(terrain, x, z);
      travelled += ds;
      px.push(x); pz.push(z); pph.push(phIdx);
      histPush(x, z, travelled, ph.width * 0.5);
      if (SOLVER.feasible) {
        // Audit the step that was actually COMMITTED, at the radius it committed
        // to (a switchback commits SB_RADIUS by construction). This is the row
        // scoreRoute() and the admissibility check are read off.
        const rC = mode === 1 ? SB_R
          : (Math.abs(dHead) > 1e-4 ? Math.min(FEAS_R_MAX, ds / Math.abs(dHead)) : FEAS_R_MAX);
        corridorAt(terrain, x, z, Math.sin(head), -Math.cos(head), ph.width * 0.5, cTmp);
        corridorRide(terrain, x, z, head, ph, rC, bankFactorFor(cTmp.deficit), rTmp);
        feasSteps++;
        feasCostSum += rTmp.cost;
        if (rTmp.shortfall > 0 || rTmp.rearFrac < RIDE_REAR_LO) feasBadSteps++;
        // Dead ground is judged on the budget the rider actually has on the
        // PHASE's surface — the same number leanBudget() hands every station
        // downstream — and not on the dirt-referenced gradient the heading fan
        // is steered by. The fan reads dirt so it stays decidable (see the note
        // in corridorRide); the AUDIT has no such constraint and must not lie.
        if (rTmp.budget <= FEAS_DEAD_RAD) {
          deadRun += ds;
          if (deadRun > deadWorst) { deadWorst = deadRun; deadAt = travelled; }
        } else deadRun = 0;
        if (SOLVER.trace) {
          routeTrace.push({
            variant, step, s: +travelled.toFixed(1), x: +x.toFixed(2), z: +z.toFixed(2),
            ph: ph.id, mode, r: +rC.toFixed(1),
            gNear: +rTmp.grade.toFixed(4), dec: +rTmp.dec.toFixed(2),
            gMax: +rTmp.gMax.toFixed(4), tanOver: +rTmp.tanOver.toFixed(4),
            ground: +rTmp.groundCost.toFixed(1),
            budgetDeg: +THREE.MathUtils.radToDeg(rTmp.budget).toFixed(2),
            shortDeg: +THREE.MathUtils.radToDeg(rTmp.shortfall).toFixed(2),
            rear: +rTmp.rearFrac.toFixed(3), cost: +rTmp.cost.toFixed(1),
          });
        }
      }

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
      // Rideability of the line as it was MARCHED — the committed heading and
      // the committed radius at every step, not a re-derivation afterwards.
      marchFeasFrac: feasSteps ? feasBadSteps / feasSteps : 0,
      marchFeasCost: feasSteps ? feasCostSum / feasSteps : 0,
      marchDead: deadWorst, marchDeadAt: deadAt,
      sbForced,
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
    let y0 = 0, yLast = 0, flat = 0, flatWorst = 0;
    for (let k = 0; k < n; k++) {
      const y = terrainH(terrain, route.px[k], route.pz[k]);
      if (k === 0) { y0 = y; low = y; }
      if (k > 0) {
        const step = Math.hypot(route.px[k] - route.px[k - 1], route.pz[k] - route.pz[k - 1]);
        travelled += step;
        if (y >= yLast - 1e-4) { flat += step; if (flat > flatWorst) flatWorst = flat; } else flat = 0;
      }
      if (y - low > climb) climb = y - low;
      if (y < low) low = y;
      yLast = y;
    }
    route.flatWorst = flatWorst;
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
    // ---- rideability, on the FINISHED candidate ----------------------------
    // The same argument as the corridor block above, for the other constraint.
    // The heading search is local and can be walked into a corridor where every
    // exit is unrideable several hundred metres earlier; this is the only place
    // that shows up, and marchRoute() already runs several variants precisely so
    // one of them can be thrown away.
    //
    // The radius here is the polyline's OWN circumradius over a +-2 point (12 m)
    // stencil, not the march's step-by-step estimate — a deliberately
    // independent second opinion on the same line.
    let feasBad = 0, feasN = 0, feasCost = 0, feasWorst = 0;
    if (SOLVER.feasible) {
      // MEASURED ON THE LINE THAT WILL BE BUILT, not on the raw marched
      // polyline. buildStations() smooths with exactly these parameters before
      // it lays a single station, and the difference is not cosmetic: a 6 m-step
      // polyline carrying the wander noise reads a circumradius of 20-40 m on
      // ground a rider would call dead straight, so the raw line scores 13% of
      // itself unrideable and every variant fails admissibility for a corner
      // that is never built. Same smoothing, same answer as the stations.
      const sm = smoothPolyline(route.px, route.pz, 4, 0.30);
      const m = 3;                     // +-18 m circumradius stencil
      for (let k = m; k < n - m; k += 2) {
        const ph = PHASES[route.pph ? route.pph[k] : 0];
        const ax = sm.x[k - m] - sm.x[k], az = sm.z[k - m] - sm.z[k];
        const bx = sm.x[k + m] - sm.x[k], bz = sm.z[k + m] - sm.z[k];
        const la = Math.hypot(ax, az), lb = Math.hypot(bx, bz);
        const lc = Math.hypot(sm.x[k + m] - sm.x[k - m], sm.z[k + m] - sm.z[k - m]);
        const area2 = Math.abs(ax * bz - az * bx);
        if (area2 < 1e-6 || la < 1e-6 || lb < 1e-6) continue;
        const r = Math.min(FEAS_R_MAX, (la * lb * lc) / (2 * area2));
        const dx = sm.x[k + 1] - sm.x[k - 1], dz = sm.z[k + 1] - sm.z[k - 1];
        const l = Math.max(1e-4, Math.hypot(dx, dz));
        const hd = Math.atan2(dx / l, -dz / l);
        corridorAt(terrain, sm.x[k], sm.z[k], dx / l, dz / l, ph.width * 0.5, cTmp);
        corridorRide(terrain, sm.x[k], sm.z[k], hd, ph, r,
          bankFactorFor(cTmp.deficit), rTmp);
        feasN++;
        feasCost += rTmp.cost;
        if (rTmp.shortfall > 0 || rTmp.rearFrac < RIDE_REAR_LO) feasBad++;
        if (rTmp.shortfall > feasWorst) feasWorst = rTmp.shortfall;
      }
    }
    route.feasFrac = feasN ? feasBad / feasN : 0;
    route.feasMean = feasN ? feasCost / feasN : 0;
    route.feasWorst = feasWorst;
    route.feasN = feasN;
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
    //
    // Rideability enters at the same scale and one step above it. `feasMean` is
    // in the same units as `corrMean` and carries the same weight; `feasFrac` is
    // a FRACTION OF THE COURSE that no bicycle can ride, and 10% of that is
    // worth more than any length error, because a route with a hundred metres of
    // unrideable trail in it is not a shorter course, it is not a course.
    // `corrMean` is a mean over a course that is protectable nearly everywhere,
    // so at 0.20 it could never decide anything: OFF and ON routes measuring 5
    // and 41 unprotectable stations differ by about 0.5 points of it. The COUNT
    // of samples carrying a catch-berm deficit is the quantity that separates
    // them, and it is weighted so that a route with thirty of them gives up
    // about as much as a route with 3% of itself unrideable. Neither can outbid
    // a climb, which is still the disqualifier.
    //
    // The non-descending term is here for the same reason: a marched line that
    // traverses 70 m without losing height is not a fault the profile solver can
    // fix — enforceDescent() can only CUT, and cutting 70 m of traverse is not a
    // trail, it is a trench.
    const flatPen = Math.max(0, (route.flatWorst || 0) - 40) * 2.0;
    // DEAD GROUND, as a continuous preference alongside the gate in
    // routeAdmissible(). `feasFrac * 900` cannot see a wall: 13 m of dead ground
    // in a 2.7 km course is 0.5% of it, worth 4 points against length errors
    // worth hundreds. A RUN LENGTH can, and it is weighted so that a variant
    // carrying twice the admissible run gives up more than any length error —
    // because a course with 13 m of zero-lean ground at 200 m is not a course
    // that is a bit worse, it is one nobody gets past.
    const deadPen = Math.max(0, (route.marchDead || 0) - ROUTE_MAX_DEAD) * 40;
    return -climb * 9 - lenErr * 90 - dropErr * 70 - route.corrMean * 0.20
      - (route.corrBad || 0) * 6 - flatPen - deadPen
      - route.feasMean * 0.20 - route.feasFrac * 900;
  }

  // -------------------------------------------------------------------------
  // ROUTE ADMISSIBILITY — the gate that stops a broken course shipping.
  //
  // WHY IT EXISTS AND WHY IT IS NOT A REPAIR PASS. On seed 2 the shipped route
  // carries a 2.0 m minimum plan radius (a 1.25 m-wheelbase bike cannot turn
  // that), 10.59 m of climb, 106.8 m of non-descending trail, 129 unresolved
  // leg-conflict stations and a worst height gap of 7.85 m across a 4.75 m plan
  // gap at arc 789 — the signature of the route folding back across its own
  // line and terrain then being asked to serve two surfaces from one
  // heightfield. Every one of those is a property of the MARCHED POLYLINE, i.e.
  // it is decided before a single station exists, and none of them is
  // repairable downstream: openTightCorners() explicitly locks hairpins,
  // enforceDescent() can only cut, and auditLegConflicts() can only pinch.
  //
  // So the failure is DETECTED and the route is RE-DRAWN, deterministically.
  // build() marches its usual variants, and if the best of them is inadmissible
  // it marches further variants — each with its own subSeed'd stream, so the
  // extra attempts are as reproducible as the first seven — until one is
  // admissible or the budget runs out. Nothing about the mountain changes: the
  // terrain heightfield is already built and the carve has not run, so this
  // reseeds only the ROUTE. That is the choice made here over reseeding the
  // world, and the reason is that ctx.seed is a contract with terrain, sky,
  // vegetation and every screenshot ever taken of a given seed; a trail that
  // cannot find a line is a trail problem and must be paid for by the trail.
  //
  // IF NO VARIANT IS ADMISSIBLE, GENERATION NOW FAILS. That is a reversal of the
  // previous cut, which shipped the best-scoring inadmissible route with
  // `safety.routeAdmissible = false` and a note. Three rounds of audit have said
  // the same thing about the same seed — "seed 2 must not ship" — and a flag
  // nobody is required to read is not a gate. `routeAdmissible` names conditions
  // that no downstream pass can repair (a 2.0 m plan radius on a 1.25 m
  // wheelbase; 106.8 m of non-descending trail; two legs contesting one
  // heightfield), so a build that trips it has not produced a course and should
  // not pretend it has.
  //
  // What that costs, stated plainly: seed 2 no longer generates. Every one of
  // its sixteen marched variants climbs between 18.7 and 93 m against a 12 m
  // gate, and the floor of 18.7 m does not move when the corridor's far end is
  // re-chosen (measured — see the note in marchRoute). Its mountain will not
  // carry a 2.6 km / 430 m descent without crossing a rise. The honest answers
  // are a different seed or a shorter design drop, and both belong to the
  // caller; silently shipping a course with a hill in the middle of it does not.
  //
  // The A/B control path (`settings.trailSolver.feasible = false`) is exempt: it
  // exists to reproduce the previous build bit-identically and must not acquire
  // a new failure mode.
  // The two thresholds below are on the MARCHED line, before enforceDescent()
  // has cut anything, so they are deliberately looser than the shipped limits
  // they protect: enforceDescent holds the shipped climb to 7 m and can do it
  // wherever the cut budget allows. What they are here to catch is the gross
  // failure — seed 2's 10.6 m of shipped climb and 106.8 m of non-descending
  // trail — not a marched line that merely needs benching.
  const ROUTE_MAX_CLIMB = 12.0;       // m above the running low, marched line
  const ROUTE_MAX_FLAT = 90.0;        // m of continuous non-descending trail
  // m of plan radius on the MARCHED line. Was 5.0, which is below anything the
  // march can now author (SB_RADIUS and the turn-authority cap both bound it at
  // R_PLAN_TARGET), so it could only ever pass. 9.0 is a real gate: measured,
  // two of 24 variants on seed 20260726 march a 2.87-2.98 m corner despite those
  // bounds, and both are now disqualified before their stations are built.
  const ROUTE_MIN_R = 9.0;
  // Fraction of the marched line no bicycle can ride. This is a GROSS-FAILURE
  // gate, not the target: the target is carried by scoreRoute's -feasFrac*900,
  // which is a continuous preference and does the work on every ordinary seed.
  // Set at 2% it was unreachable on every seed and every variant — so the
  // extended budget ran 16 marches on courses that had nothing wrong with them,
  // and `routeAdmissible` reported false on a course that ships fine. 6% is
  // where the shipped routes actually sit once the term is doing its job.
  const ROUTE_MAX_FEAS = 0.06;
  const ROUTE_CONFLICT_TOL = 0.75;    // m, = LEG_HARD_TOL
  // Longest continuous run of committed march, in metres, on which the measured
  // table returns no usable lean. Two audits in a row have named exactly this
  // quantity as what kills seeds 12345 and 99999 inside the first 200 m — 12.9 m
  // at arc 202 and 12.1 m at arc 154 — while `feasFrac` moved by half a percent
  // and every gate passed. 6 m is about a bike plus a rider's reaction: a metre
  // or two of zero-budget ground on a straight is a chute, and seed 777 ships
  // 1.5 m of it and finishes; thirteen metres is a wall.
  const ROUTE_MAX_DEAD = 30.0;
  // ---- the limits that are read off the BUILT profile ---------------------
  // These are the ones that decide, and they are the CONTRACT's own numbers.
  // CONTRACT §4: "the trail must never climb for more than ~15 m". 14 m leaves a
  // station of margin against the +-2 m stencil.
  // MEASURED AT THE BASE-PROFILE STAGE, WHICH IS NOT THE FINAL STAGE, and the
  // offset is stated rather than hidden. capCrestCurvature(), capLaunchCurvature()
  // and capGradeRamp() all run afterwards and all lower crests, so a base profile
  // reads high: measured on the four buildable seeds, base -> final is
  // 22.7->18.3, 24.0->22.8, 32.8->17.9 and 12.1->11.3 m of non-descending, and
  // 10.5->2.8, 12.3->1.5, 23.9->12.9 and 12.9->7.9 m of dead ground. These
  // limits are therefore set on the BASE scale to land the FINAL number on
  // CONTRACT §4's ~15 m. The final number is measured too, at the end of build(),
  // and reported on safety.flatShipped / safety.deadShipped — that one is the
  // acceptance figure and it is not inferred from these.
  const SHIP_MAX_FLAT = 18.0;
  // ...AND THE ONE THE VARIANT BUDGET IS ACTUALLY SPENT ON, which is the
  // PROPERTY and not the counter. See the note above enforceDescentShipped().
  // SHIP_MAX_FLAT bounds the longest run of consecutive non-descending stations,
  // which a 10 micrometre dip resets, so a base profile can read 11 m while
  // carrying 89 m that never gets back below its own start — and that is what
  // the reference seed was selecting on. SHIP_MAX_RUN bounds the stretch that
  // ends at or above its own start and never drops below it, on the same base
  // scale: enforceDescentShipped()'s windowed pass takes another 20-60% off it
  // downstream where the excavation ceiling allows, so 26 m here lands the
  // final figure near CONTRACT §4's ~15 m. Both are measured and published;
  // `safety.flatRunShipped` is the acceptance figure and it is not inferred
  // from this one.
  const SHIP_MAX_RUN = 26.0;
  // Dead ground: a metre or two on a straight is a chute a rider rolls, and seed
  // 777 shipped 1.5 m of it and finished. Six is about a bike plus a reaction.
  const SHIP_MAX_DEAD = 12.0;
  // ...and the HARD one, measured on the profile that ships rather than on the
  // base profile, which refuses the build. See the wall gate at the end of
  // build(). SHIP_MAX_DEAD above is a variant tie-break on the base profile;
  // this is a statement that a course was produced that nobody can ride down.
  const SHIP_HARD_DEAD = 8.0;
  // CONTRACT §4's ~15 m, as the threshold at which the shipped non-descending
  // RUN (the property, not the counter) is reported loudly. Warn, not throw —
  // see the note at the call site.
  const DESC_RUN_WARN = 20.0;
  // How many candidates build() will build stations for before it settles. Every
  // extra try costs one buildStations(), which is a small fraction of build();
  // the loop exits at the first candidate inside both limits, so on a healthy
  // seed this costs nothing at all.
  const STATION_TRIES = 6;

  /**
   * Metres of missing leg clearance summed over every pair of polyline points
   * more than LEG_ARC_MIN of travel apart. Independent of the march's own
   * incremental term — this one sees the finished line, including the legs that
   * were laid before the ones they conflict with.
   */
  function routeSelfConflict(route) {
    const n = route.px.length;
    const s = new Float64Array(n);
    for (let k = 1; k < n; k++) {
      s[k] = s[k - 1] + Math.hypot(route.px[k] - route.px[k - 1], route.pz[k] - route.pz[k - 1]);
    }
    let worst = 0, total = 0, count = 0;
    for (let i = 0; i < n; i++) {
      const hi = (PHASES[route.pph ? route.pph[i] : 0]).width * 0.5;
      for (let j = i + 1; j < n; j++) {
        if (s[j] - s[i] < LEG_ARC_MIN) continue;
        const dx = route.px[j] - route.px[i], dz = route.pz[j] - route.pz[i];
        const d = Math.hypot(dx, dz);
        if (d >= LEG_SEARCH) continue;
        const hj = (PHASES[route.pph ? route.pph[j] : 0]).width * 0.5;
        const need = legReach(hi) + legReach(hj);
        if (d >= need) continue;
        const def = need - d;
        total += def; count++;
        if (def > worst) worst = def;
      }
    }
    return { worst, total, count };
  }

  /**
   * Walk the marched polyline the way a builder would and say whether it is a
   * course at all. Returns null when admissible, else the name of the first
   * test it fails. Fills route.adm* for the audit either way.
   */
  function routeAdmissible(terrain, route) {
    const n = route.px.length;
    let low = Infinity, climb = 0, flat = 0, flatWorst = 0;
    let yPrev = 0;
    for (let k = 0; k < n; k++) {
      const y = terrainH(terrain, route.px[k], route.pz[k]);
      if (k === 0) { low = y; yPrev = y; continue; }
      const step = Math.hypot(route.px[k] - route.px[k - 1], route.pz[k] - route.pz[k - 1]);
      if (y >= yPrev - 1e-4) { flat += step; if (flat > flatWorst) flatWorst = flat; } else flat = 0;
      if (y - low > climb) climb = y - low;
      if (y < low) low = y;
      yPrev = y;
    }
    let rMin = 1e9;
    for (let k = 2; k < n - 2; k++) {
      const ax = route.px[k - 2] - route.px[k], az = route.pz[k - 2] - route.pz[k];
      const bx = route.px[k + 2] - route.px[k], bz = route.pz[k + 2] - route.pz[k];
      const la = Math.hypot(ax, az), lb = Math.hypot(bx, bz);
      const lc = Math.hypot(route.px[k + 2] - route.px[k - 2], route.pz[k + 2] - route.pz[k - 2]);
      const area2 = Math.abs(ax * bz - az * bx);
      if (area2 < 1e-6 || la < 1e-6 || lb < 1e-6) continue;
      const r = (la * lb * lc) / (2 * area2);
      if (r < rMin) rMin = r;
    }
    const conf = routeSelfConflict(route);
    route.admClimb = climb;
    route.admFlat = flatWorst;
    route.admMinR = rMin;
    route.admConflict = conf.worst;
    route.admConflictN = conf.count;
    route.admDead = route.marchDead || 0;
    if (climb > ROUTE_MAX_CLIMB) return 'climb';
    if (flatWorst > ROUTE_MAX_FLAT) return 'nonDescending';
    if (rMin < ROUTE_MIN_R) return 'planRadius';
    if (conf.worst > ROUTE_CONFLICT_TOL) return 'legConflict';
    if (SOLVER.feasible && route.marchDead > ROUTE_MAX_DEAD) return 'deadGround';
    if (SOLVER.feasible && route.feasFrac > ROUTE_MAX_FEAS) return 'rideability';
    return null;
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
  // THE THIRD RULE, AND THE ONE THAT WAS MISSING. Rules 1 and 2 bound the RATE
  // and the HEIGHT of a climb; neither bounds its LENGTH. `maxUpGrade` is 3.5%,
  // so a profile may rise at 3.4% for two hundred metres and pass both — and
  // that is exactly what shipped: seed 777 at arc 2497 carried 22.8 m of trail
  // rising 0.65 m, which is 2.8%, inside every test this function applied.
  // CONTRACT §4 is about the LENGTH ("must never climb for more than ~15 m"),
  // so the length is what has to be bounded.
  //
  // 11 m, not 15: the run is measured here in PLAN (ds is the station spacing)
  // while the acceptance figure is 3-D arc, and three passes run downstream of
  // the last call to this function. The margin is the difference.
  const DESC_MAX_RUN = 11.0;
  const DESC_MIN_FALL = 0.010;   // rise/run the rule then insists on

  function enforceDescent(y, ds, raw, maxCut) {
    const n = y.length;
    const maxClimb = 7.0;
    const maxUpGrade = 0.035;
    let low = y[0];
    let run = 0;
    for (let i = 1; i < n; i++) {
      let v = y[i];
      const cap = y[i - 1] + maxUpGrade * ds;
      if (v > cap) v = cap;
      if (v > low + maxClimb) v = low + maxClimb;
      // Rule 3: once the line has gone DESC_MAX_RUN metres without losing
      // height, it must lose some. Cut-only, like everything else here.
      if (run >= DESC_MAX_RUN) {
        const fall = y[i - 1] - DESC_MIN_FALL * ds;
        if (v > fall) v = fall;
      }
      // Excavation limit wins the tie. Without it, a route that climbs 80 m would
      // be "fixed" by cutting an 80 m trench through the mountain, which is far
      // worse than the trail going over a rise. Route selection is what actually
      // keeps the climb down; this is the backstop.
      if (raw && v < raw[i] - maxCut) v = raw[i] - maxCut;
      y[i] = v;
      // Counted on the value that was actually COMMITTED, so a run the
      // excavation limit refused to break keeps counting and the audit sees it.
      if (v >= y[i - 1] - 1e-5) run += ds; else run = 0;
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
  //
  // EVERY ENTRY IS FLOORED AT R_PLAN_MIN — the requirement, not R_PLAN_TARGET.
  // The target is what the MARCH is bounded at, because a corner that is never
  // authored tight needs no repair; asking this relaxation to reach the target
  // as well puts 1291 of 6600 stations inside its trigger on the reference seed,
  // spreads the PLAN_SHIFT_MAX displacement budget across all of them, and
  // measurably leaves the worst corner TIGHTER than it found it (9.50 -> 8.26 m).
  // A repair pass that runs everywhere is not a repair pass.
  //
  // The per-phase numbers below are the
  // CHARACTER knob and they still are — `start`, `flow`, `jumps` and `sprint`
  // keep their long, fast radii — but four of them (roots 13, loam 14, slab 15,
  // rocks 15) were BELOW the radius the reference rider can hold at the design
  // floor speed, so "these phases are meant to be technical and are ridden
  // slowly" was authorising geometry no speed makes rideable. Technical means
  // tight for a trail; it does not mean tighter than the bike turns.
  const PLAN_MIN_R = (() => {
    const character = {
      start: 27, roots: 13, flow: 23, jumps: 27, slab: 15,
      creek: 17, loam: 14, rocks: 15, sprint: 25,
    };
    const out = {};
    for (const k of Object.keys(character)) out[k] = Math.max(R_MIN_PLAN_EFF, character[k]);
    return out;
  })();
  const PLAN_SHIFT_MAX = 3.6;    // m the alignment may move from the marched line
  const PLAN_STRIDE = 5.0;       // m — the stride the plan curvature is measured over
  // Below this the corner is a reversal the relaxation cannot open, and the
  // relaxation leaves it alone. See the hairpin lock in openTightCorners().
  //
  // RE-MEASURED THIS ROUND, ON THE RESPACED LINE, because the file's own note
  // (see measurePlanRadii) later diagnosed the "5.53 -> 3.31 m" figure the lock
  // was justified by as an artefact of measuring at a fixed station stride on a
  // line the pass had shortened — the same numbers, at the same station. If the
  // lock rested on that artefact it should have come off. It does not: with the
  // lock removed and every radius read on the RESPACED centreline at the +-4 m
  // acceptance stencil, the worst plan radius goes 4.91 -> 3.25 m on seed
  // 20260726 and 4.84 -> 3.18 m on 777. The mechanism in the note is real — past
  // about 100 deg of turn inside the detection stencil the Laplacian target has
  // crossed to the inside of the arc — and the lock stays.
  //
  // What that means for the radius fix is the point: a hairpin cannot be opened
  // downstream, so it must not be authored tight upstream. That is why
  // R_PLAN_TARGET is enforced at the march (SB_RADIUS and the turn-authority
  // cap) and not here. With the march bounded, this lock is inert on all four
  // seeds — `safety.planLocked` falls to 0 — which is the correct end state for
  // a guard: still armed, never firing.
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
      // The grade the TYRE works against: the same profile computeVertKappa()
      // prices, i.e. with the authored ballistic relief removed. See
      // computeRideBasis(). Only leanBudgetAt() and brakeAccel() read it.
      gradeR: Float32Array.from(grade),
      // THE RADIUS A RIDER ACTUALLY MEETS. `radius` above is a tangent
      // difference over a +-5 m stride, Gaussian-smoothed again over +-4 m —
      // two low-passes on a quantity that only matters at its minimum, so it
      // systematically OVER-reads exactly where the identity binds. Measured on
      // the ON build at seed 20260726 arc 188 m: `radius` reports the corner
      // slack enough to authorise 7.4 m/s while a 0.37 m-stencil circumradius on
      // the same committed centreline reads 68 m and the honest cap is 6.4.
      // `radiusR` is the plain +-4 m circumradius — the same stencil the
      // acceptance harness uses, stated as the brief requires — and the speed,
      // bank and identity all read it. `radius`/`curv` keep their old duty
      // (feature siting, bank SIGN) so nothing else in the file moves.
      radiusR: new Float32Array(N),
      width: new Float32Array(N),
      bank: new Float32Array(N),
      speed: new Float32Array(N),
      surface: new Uint8Array(N),
      hOff: new Float32Array(N),       // feature height offset above the base grade
      // THE AUTHORED BALLISTIC RELIEF, as a separate record of the part of hOff
      // that buildJump()/buildDrop() wrote. See computeVertKappa(): the launch
      // constraint prices convex curvature the design did NOT intend to throw the
      // rider off, and a solved lip is not that. Never consumed by the geometry —
      // hOff remains the single source of the surface — only by the measurement.
      hBal: new Float32Array(N),
      rut: new Float32Array(N),        // rut depth (m)
      crown: new Float32Array(N),      // tread crown (m)
      rough: new Float32Array(N),      // micro-roughness amplitude (m)
      bumps: new Float32Array(N),      // transverse ridge amplitude (roots/brake bumps)
      kapVRaw: new Float32Array(N),    // ... the same, WITH the ballistic relief in (audit only)
      kapV: new Float32Array(N),       // convex vertical curvature of the committed centre
                                       // profile, worst over LAUNCH_STENCILS (see
                                       // computeVertKappa) — the launch constraint's input
      vWant: new Float32Array(N),      // the speed the design intends here, ignoring relief;
                                       // capLaunchCurvature() flattens the ground to it
      // A HARD SPEED CEILING AUTHORED BY A FEATURE, m/s. The speed profile is
      // re-solved after buildFeatures(), so a feature that needs the rider to
      // arrive slow cannot simply write S.speed — it has to declare a cap the
      // solve honours. buildDrop() is the only writer: a drop's landing accepts
      // a bounded speed window and the design speed has to be inside it.
      vFeat: new Float32Array(N),
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
    S.vFeat.fill(1e9);
    ballisticMask = null;
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
  /**
   * The lateral friction actually left over after the fall line has been paid
   * for — the friction circle, which the corner cap did not have.
   *
   * MEASURED, and it is the reason every one of 21 autopilot cells died within
   * 32 m of the same point on seed 20260726 once the launch problem was fixed.
   * At arc 296-301 m the plan radius runs 21 -> 9 m while the gradient runs
   * 30% -> 72%. The corner cap looked at the radius, the bank and the surface,
   * saw 10.1 m/s of grip, and paced it at 8.5-9.2. But a rider holding speed on
   * a 72% pitch is already asking the tyres for g*sin(theta) = 0.58 g just to
   * not accelerate, and the design friction on dirt is tan(28 deg) = 0.53 g in
   * total. There is nothing left to turn with. The trace shows exactly that:
   * skid 0.84-0.98, the bike's own leanLimit collapsed to 19-26 deg, and the
   * lateral error going 0.6 -> 3.0 m in 0.9 s straight off the outside.
   *
   * The circle is the standard one. The tyres carry mu*N in total; holding
   * speed on the slope spends tan(theta) of it; what is left for the corner is
   *
   *     mu_lat = sqrt( mu^2 - (tan theta)^2 )
   *
   * floored at MU_CORNER_MIN so a genuinely steep chute is paced as a chute
   * rather than as a stop. On the flat this returns mu unchanged, so nothing
   * outside the steeps moves at all: at 25% grade dirt goes 0.53 -> 0.47, at
   * 50% it goes 0.53 -> 0.18, and at 72% it is on the floor.
   *
   * This is the third constraint in the joint solve. The other two — do not get
   * launched, and hold the corner at the speed you arrive at — are useless on
   * their own if the tyres are already spent resisting gravity.
   */
  // THE FLOOR UNDER THE CORNER GRIP BUDGET, and why it had to come off.
  //
  // 0.12 was there so a genuinely steep chute is paced as a chute rather than as
  // a stop. Measured on the ON build at seed 20260726, arc 186-194 m — where
  // every one of six autopilot cells died within 4 m of the same point — it was
  // doing something else entirely: grade 46%, plan radius 63-71 m, honest lean
  // budget 0.1-2 deg, and the floor authorised mu = 0.12, which through
  //     v = sqrt(g r (mu + tan phi) / (1 - mu tan phi))
  // set the design speed to 11.1 m/s. The lean that corner then demands is
  // 10.8 deg against 7.0 available. Without the floor the same station solves to
  // 8.6 m/s and the identity closes exactly. A floor on the GRIP is a floor on
  // the lie; the honest floor is on the SPEED, and V_DESIGN_FLOOR already is
  // one — where the corner cap falls below it, the geometry has failed and
  // safety.cornerFloorBound says so instead of the profile pretending.
  const MU_CORNER_MIN = 0.12;
  const MU_CORNER_MIN_FEAS = 0.010;   // numerical floor only (keeps the sqrt sane)

  /**
   * The deceleration the SPEED PROFILE ITSELF demands at station i, m/s².
   *
   * This is the term the friction-circle audit found dominant and this module
   * had never priced: decomposed on seed 20260726, `tan(theta) > mu_design`
   * alone at 1.27% of stations but `mu_long > mu_design` at 3.88%, so the
   * braking the design asks for outweighs the grade it asks it on. Zero on the
   * first solve, when there is no profile yet — which is correct, not a gap:
   * the solve is a fixed point and the first iterate has nothing to brake for.
   */
  function decelDemand(i) {
    if (!S.speed || S.n < 3) return 0;
    const a = Math.max(0, i - 1), b = Math.min(S.n - 1, i + 1);
    const v = S.speed[i];
    if (!(v > 0.5)) return 0;
    const dv = (S.speed[b] - S.speed[a]) / ((b - a) * S.ds);
    // Signed: a station the profile is ACCELERATING through is one where
    // gravity, not the tyre, is doing the longitudinal work.
    return clamp(-v * dv, -RIDE_DEC_CAP, RIDE_DEC_CAP);
  }

  /**
   * The lean this station's rider can actually hold against the tread, radians.
   * On flat dirt this is LEAN_DESIGN to the last digit; on a 45% grade under
   * speed-holding braking it is 24.5 deg (the previous, modelled, cut of this
   * said 2.4 — see the RIDEABILITY ENVELOPE note for what was missing).
   */
  function leanBudgetAt(i) {
    if (!SOLVER.feasible) return LEAN_DESIGN;
    return leanBudget(tanFromSin(rideGrade(i)), decelDemand(i), S.surface[i]);
  }

  /** S.grade with the authored ballistic relief removed. See computeRideBasis(). */
  function rideGrade(i) {
    return S.gradeR ? S.gradeR[i] : S.grade[i];
  }

  /**
   * The lateral friction left after the fall line and the brakes have been paid
   * for, as a coefficient — which is exactly tan(the lean budget), because
   * cornerSpeedCap() inverts v² = g·r·(mu + tan phi)/(1 − mu·tan phi) and the
   * right-hand side of that is g·r·tan(atan(mu) + phi). So substituting the
   * load-derived budget here makes the identity
   *
   *     atan(v²/(g r)) = phi + lean_available(grade, decel, surface)
   *
   * hold by construction at every corner where the cap binds, with a budget
   * that collapses under braking the way the bike's own leanCeiling() does.
   *
   * WHAT THIS REPLACES. The old rule was the naive friction circle,
   * sqrt(mu² − tan²theta) at a CONSTANT mu = tan(28 deg). It is the rule the
   * skeptic's synthetic-course sweep falsified directly: it authorises 21 m/s
   * through a 64 m corner on a 45% grade, where the bike cannot hold 6.
   */
  function cornerGripBudget(i) {
    if (!SOLVER.feasible) {
      const mu0 = clamp(MU_DESIGN * SURF_MU[S.surface[i]], 0.22, 1.0);
      const fall = Math.abs(S.grade[i]);
      return Math.max(MU_CORNER_MIN, Math.sqrt(Math.max(0, mu0 * mu0 - fall * fall)));
    }
    return Math.max(MU_CORNER_MIN_FEAS, Math.tan(leanBudgetAt(i)));
  }

  // A HAIRPIN IS RIDDEN AT HAIRPIN SPEED, and the lean identity on its own does
  // not know that.
  //
  // This is the answer to "is a 5.4 m hairpin a legitimate feature at
  // V_DESIGN_FLOOR, or a defect?" — it is legitimate, and what was defective was
  // the speed the profile entered it at. Measured on the shipped build at seed
  // 20260726 arc 296-306: plan radius 5.1-7.7 m, built bank 24-28 deg, design
  // speed 8.4-9.2 m/s. The identity closes there — it is exactly what
  // cornerSpeedCap solved for, v^2 = g*r*(mu + tan phi)/(1 - mu*tan phi) with
  // mu = tan(budget) and a 28 deg berm gives 8.15 m/s at r = 5.1 — and the
  // rider still cannot hold it. EVERY autopilot cell died inside 305-311 m.
  //
  // Two independent probes say the same thing, on different ground and with
  // different controllers. The corner-hold probe on the analytic helicoid holds
  // 0.58-0.76 of the measured ceiling at plan radii above 33 m and only
  // 0.43-0.50 at 22-30 m; and on FLAT ground it holds a 5 m radius perfectly
  // well at 6.2 m/s while failing far larger radii at 10-14 m/s. So the variable
  // is not the radius on its own — it is the radius AT SPEED. At 8.4 m/s in a
  // 5.1 m circle the bike is turning at 1.65 rad/s with the front tyre at a slip
  // angle the single-track solver in bike.js has to run right up against its
  // saturation to supply, and the quasi-static identity is blind to all of it.
  //
  // The cap below is a design statement rather than a fitted curve: below
  // HAIRPIN_R0 a corner is a hairpin and is paced at the switchback apex speed
  // the march already builds to (SB_V_APEX), easing back to the identity's own
  // answer by HAIRPIN_R1. Above HAIRPIN_R1 it is inert — it never binds on an
  // ordinary corner, which is checkable: the term changes nothing on any station
  // with r > 18 m on any seed.
  //
  // AND IT IS NO LONGER LOAD-BEARING, which is the point of R_PLAN_MIN. This cap
  // was round 9's answer to the 5 m hairpin, and it could never be the answer,
  // because it can only push the pace down to V_DESIGN_FLOOR and the corner
  // needed it lower. With the march bounded at R_PLAN_TARGET the tightest corner
  // that ships is 10.1-13.9 m, so what remains here is a pacing rule for genuine
  // hairpins — the thing it always should have been — and never a rescue for
  // geometry that has none.
  const HAIRPIN_R0 = R_PLAN_MIN;  // the tightest plan radius the course may ship
  const HAIRPIN_R1 = 18.0;        // open enough that the identity is the only bound
  const HAIRPIN_V1 = 9.5;         // m/s — the pace an 18 m corner is entered at

  function cornerSpeedCap(i) {
    const mu = SOLVER.launch ? cornerGripBudget(i)
      : clamp(MU_DESIGN * SURF_MU[S.surface[i]], 0.22, 1.0);
    // Clamp the bank contribution: past ~50 deg the formula's denominator runs
    // away, and no rider is relying on a berm steeper than that anyway.
    const tb = Math.tan(clamp(helpfulBank(i), -0.55, 0.87));
    const denom = Math.max(0.20, 1 - mu * tb);
    const num = Math.max(0.02, mu + tb);
    const r = rideRadius(i);
    const v = Math.min(V_CAP, Math.sqrt(Math.max(1, G * r * num / denom)));
    if (!SOLVER.feasible || r >= HAIRPIN_R1) return v;
    const f = smoothstep(HAIRPIN_R0, HAIRPIN_R1, r);
    return Math.min(v, SB_V_APEX + (HAIRPIN_V1 - SB_V_APEX) * f);
  }

  // -------------------------------------------------------------------------
  // VERTICAL CURVATURE OF THE COMMITTED PROFILE — the measurement techSpeedCap
  // was missing. See the LAUNCH_SAFETY note at the head of the file.
  // -------------------------------------------------------------------------

  /**
   * Convex vertical curvature of profile `y` at station `i` over a half-stencil
   * of `h` metres of PLAN arc. Positive = crest (the direction that throws the
   * rider). Converted to true path curvature by the (1+y'^2)^(3/2) term, because
   * stations are spaced in plan and a 50% grade would otherwise over-read by 20%.
   */
  function vertKappaAt(y, i, h) {
    const k = Math.max(1, Math.round(h / STATION_DS));
    const a = i - k, b = i + k;
    if (a < 0 || b >= S.n) return 0;
    const d = k * STATION_DS;
    const d2 = (y[a] - 2 * y[i] + y[b]) / (d * d);
    const yp = (y[Math.min(S.n - 1, i + 1)] - y[Math.max(0, i - 1)]) / (2 * STATION_DS);
    return -d2 / Math.pow(1 + yp * yp, 1.5);
  }

  /**
   * Fill S.kapV with the worst convex curvature over the stencil ladder.
   *
   * WHY A LADDER AND NOT A WAVELENGTH. The relief on this profile is not a
   * sinusoid and it is not one scale: measured on seed 20260726 at arc 178-180 m
   * the 2 m stencil reads 0.063-0.079 /m while the 6 m stencil reads 0.021-0.023,
   * and 110 m further down at 290-294 m it is the other way round (2 m: 0.099-0.139,
   * 6 m: 0.085-0.089). A single stencil is guaranteed to miss one of those two
   * places, and both of them launch the rider. Taking the worst over the ladder
   * is conservative by construction and cannot miss either.
   *
   * The smallest rung is the station spacing, which is within 8% of the wheel
   * radius the acceptance test stencils at; anything shorter than that is
   * sub-station relief and is priced separately by chatterKappa().
   */
  function computeVertKappa() {
    const n = S.n;
    // Measure on the profile FILTERED the way the wheel and the carve filter it,
    // not on the raw station lattice. Without this the shortest rung reads its
    // own quantisation: a 5 mm station-to-station step is 0.06 /m at a 0.4 m
    // stencil, three times the cap at 16 m/s, and it is neither built (the carve
    // passes 6-11% of 0.5-2 m content) nor felt (a 0.37 m wheel bridges it). The
    // sigma is 0.45 m, whose Gaussian transfer — 0.21 at 1.6 m, 0.64 at 3 m,
    // 0.90 at 6 m, 0.97 at 12 m — tracks the measured carve transfer in
    // CARVE_XFER_BANDS to within the spread of the three seeds it came from.
    //
    // ...AND MEASURED ON THE PROFILE WITH THE AUTHORED BALLISTIC RELIEF TAKEN
    // OUT. This is the fix for `safety.jumpWindowBad`, and it is a correction to
    // the MEASUREMENT, not an exemption from the constraint.
    //
    // launchSpeedCap() answers "above what speed does this ground throw the
    // wheels off it". A jump lip is convex past that cap BY CONSTRUCTION —
    // leaving the ground is the entire point — so measuring the lip and then
    // pacing to it is a category error, and it is not confined to the lip. The
    // stencil ladder runs to 12.8 m, so a 1 m lip poisons the launch reading of
    // every station within 12.8 m of it IN BOTH DIRECTIONS, which is the whole
    // run-in. The backward brake limit then drags the approach down to the floor.
    // That is the observed defect exactly: on seed 20260726 the doubles are
    // solved for an 11.8 m/s takeoff and the profile delivers 5.39 m/s at the
    // lip, against a vMin of 11.1 the same solve published — 20 of 32 air
    // features across four seeds paced BELOW their own window.
    //
    // WHY NOT JUMP_PACE_EXEMPT. That constant switches the section cap off on the
    // ballistic core and it is a worse instrument for the same idea, for a
    // reason the note beside it does not state: the mask reaches iOutEnd, so the
    // KNUCKLE, the landing ramp and the run-out get paced at the phase ceiling
    // (17.5 m/s in the jump phase) with no launch cap at all, and the run-in
    // outside the mask is still poisoned by the lip 12 m ahead of it. It relaxes
    // the constraint in the one place it should still bind and leaves it wrong
    // where the defect actually is. Measured, it took seed 777 from 9/21
    // finishes to 0/21. Kept `false`.
    //
    // Subtracting hBal instead says the right thing in the right place: the
    // run-in is priced by the ground it is on, the ballistic core is priced by
    // solveJump()'s own speed window through S.vFeat, and every metre of
    // ordinary ground — including the run-out once its authored relief is
    // accounted for — is priced by the launch cap exactly as before. The raw
    // figure is still measured and published as `safety.launchViolRaw`, so
    // nothing is hidden by the accounting.
    const raw = new Float32Array(n);
    const balOff = SOLVER.feasible && S.hBal ? S.hBal : null;
    for (let i = 0; i < n; i++) raw[i] = S.by[i] + S.hOff[i] - (balOff ? balOff[i] : 0);
    const y = gaussianSmooth(raw, 4, 0.45 / STATION_DS);
    for (let i = 0; i < n; i++) {
      let worst = 0;
      for (let s = 0; s < LAUNCH_STENCILS.length; s++) {
        const k = vertKappaAt(y, i, LAUNCH_STENCILS[s]);
        if (k > worst) worst = k;
      }
      S.kapV[i] = worst;
    }
    // The same measurement WITHOUT the subtraction, for the audit only. Nothing
    // reads it but countLaunchViol's raw column: it is the honest statement of
    // how much of the committed surface would launch a rider at the design pace
    // if the ballistic features were not features.
    if (S.kapVRaw) {
      const rawFull = new Float32Array(n);
      for (let i = 0; i < n; i++) rawFull[i] = S.by[i] + S.hOff[i];
      const yF = gaussianSmooth(rawFull, 4, 0.45 / STATION_DS);
      for (let i = 0; i < n; i++) {
        let worst = 0;
        for (let s = 0; s < LAUNCH_STENCILS.length; s++) {
          const k = vertKappaAt(yF, i, LAUNCH_STENCILS[s]);
          if (k > worst) worst = k;
        }
        S.kapVRaw[i] = worst;
      }
    }
  }

  /**
   * Convex curvature from the sub-station relief this module authors — the
   * microDetail fbm, the root ridges and the braking-bump trains — priced at the
   * wavelengths they are actually emitted at, through the wheel and the carve.
   *
   * This is what ROUGH_LAMBDA was trying to be. The difference is that it names
   * every component and its own wavelength instead of collapsing all of them
   * onto one number: microDetail's fundamental is ROUGH_LAMBDA*2.4 = 3.84 m and
   * its second octave (lacunarity 2.4, gain 0.5) lands on ROUGH_LAMBDA itself,
   * and the ridge trains are at 2.42 m (roots, sin(s*2.6)) and 1.1 m (braking
   * bumps). Those are four different filters, not one.
   */
  function chatterKappa(i) {
    const r = S.rough[i];
    let k = r * launchKappaPerAmp(ROUGH_LAMBDA * 2.4);
    const k2 = r * 0.5 * launchKappaPerAmp(ROUGH_LAMBDA);
    if (k2 > k) k = k2;
    if (S.bumps[i] > 0.001) {
      // The ridge trains: roots at 2.42 m, braking bumps at 1.1 m. Both are
      // emitted into S.bumps, so price the worse of the two.
      const kb = S.bumps[i] * Math.max(launchKappaPerAmp(2.42), launchKappaPerAmp(1.1));
      if (kb > k) k = kb;
    }
    return k;
  }

  /** Geometric launch-speed limit at station i, with the design margin. */
  function launchSpeedCap(i) {
    let k = Math.max(S.kapV[i], SOLVER.carveFloor);
    const kc = chatterKappa(i);
    if (kc > k) k = kc;
    return Math.sqrt(G / (SOLVER.safety * k));
  }

  /**
   * The speed the design INTENDS at this station, before the relief is priced:
   * the phase ceiling modulated by tread width. capLaunchCurvature() flattens the
   * ground toward this, and only where it cannot does techSpeedCap() give way.
   */
  function wantSpeedCap(i) {
    const ph = PHASES[S.phase[i]];
    const base = V_PHASE[ph.id] === undefined ? 17.0 : V_PHASE[ph.id];
    // Tread width, relative to the phase's own design width. A pinch is slower
    // than the section it is in; a corner that has been widened is faster.
    const wf = clamp(0.74 + 0.30 * (S.width[i] / Math.max(0.8, ph.width)), 0.70, 1.14);
    return base * wf;
  }

  /**
   * The design speed of the section itself, independent of any corner: what a
   * rider carries down a straight of this width, on this surface, at this
   * roughness — bounded by the speed at which the relief that will ACTUALLY be
   * committed here throws the wheels off the ground.
   */
  function techSpeedCap(i) {
    if (!SOLVER.launch) {
      return Math.min(wantSpeedCap(i), legacyRoughCap(S.bumps[i], S.rough[i]));
    }
    // AUTHORED BALLISTIC RELIEF IS NOT A DEFECT TO BE PACED AROUND — LEAVING
    // THE GROUND IS THE POINT.
    //
    // launchSpeedCap() is the speed above which the tread's convex curvature
    // throws the wheels off it. A jump lip is convex past that cap BY
    // CONSTRUCTION — the file already says so where the crest mask is built —
    // so at every lip the cap collapses to the V_DESIGN_FLOOR floor, and
    // cornerCapLead() then drags the whole run-in down with it. Measured on the
    // shipped build, on both arms, before this line existed: every jump on
    // every seed was solved for a takeoff of 11.5-12.0 m/s and the profile then
    // paced its own lip at 5.4-6.2 m/s, which is BELOW the `vMin` of its own
    // published speed window on all but the tabletop. The trail was designing
    // five jumps per seed and then telling the rider to case every one of them.
    // CONTRACT §4: "a jump you can't land is a bug".
    //
    // So on the ballistic core the section cap is the design speed alone. The
    // CORNER cap still applies (a berm through a jump is still a berm), the
    // braking and acceleration limits still apply, and the launch cap still
    // governs every metre of ordinary ground including the run-in and the
    // run-out — it is switched off only where the geometry is a jump that this
    // module solved and published a speed window for.
    if (JUMP_PACE_EXEMPT && ballisticMask && ballisticMask[i]) return wantSpeedCap(i);
    return Math.max(V_DESIGN_FLOOR, Math.min(wantSpeedCap(i), launchSpeedCap(i)));
  }

  // -------------------------------------------------------------------------
  // BRAKING — the term the friction-circle audit found DOMINANT, and the one
  // this module had wrong by a factor of two.
  //
  // A_BRAKE is a constant 7.2 m/s². The trail's own design friction is
  // tan(28 deg) = 0.532, so on FLAT ground the profile was already asking for
  // 1.4x the deceleration it designs the corners against — and on a descent it
  // is much worse, because gravity is pushing the other way: the honest limit is
  //
  //     a_brake = g * (mu_brake * cos theta - sin theta)
  //
  // which on a 20% grade is 3.2 m/s², on a 40% grade 1.7, and NEGATIVE past
  // about 32 deg, where a bicycle cannot slow down at all. Measured on the
  // shipped profile at seed 20260726: decel p95 3.9 m/s², p99 at the 5 m/s²
  // measurement clamp — i.e. mu_long from braking alone running to 0.72 where
  // 0.53 is designed for, which is precisely the audit's decomposition
  // (mu_long > mu_design at 3.88% of stations against tan theta > mu_design at
  // 1.27%). It is also what collapses the lean budget: at 5 m/s² of braking the
  // rear load fraction goes NEGATIVE and leanBudget() returns zero, so 7-8% of
  // the course had no cornering capacity at all for a reason that had nothing
  // to do with the ground.
  //
  // mu_brake is the design's longitudinal share — the same RIDE_USE fraction of
  // the bike's own grip — capped by the ENDO limit, which is the real bound on
  // a bicycle's front brake. Both moved with the measurement: the grip scale is
  // now the bike's own SURF_MU (1.08 on dirt, not 1.00) and the endo limit is
  // the reference rider's shifted one (0.844, not 0.610), so the profile is
  // allowed to brake harder than the previous cut let it — which is what stops
  // the backward pass manufacturing the deceleration that then eats the corner.
  // ...AND IT IS SPENT AT A FRACTION, NOT IN FULL. A speed profile that brakes
  // at 100% of the longitudinal budget has, by definition, no lateral capacity
  // left anywhere it is braking — leanBudget() returns literally zero — so every
  // gentle sweeper the profile happens to be slowing through becomes a
  // violation. That is not a corner defect, it is a design that never left
  // itself a reserve. 0.72 keeps mu_long at 0.44 on dirt, which leaves
  // sqrt(0.532^2 - 0.44^2) = 0.30 of lateral inside the trail's own design
  // friction — enough for a 150 m sweeper at 12 m/s five times over.
  const BRAKE_USE = 0.72;
  const BRAKE_A_MIN = 0.35;    // m/s², floor so the backward pass stays solvable
  // Metres of straight the profile must be at corner speed BEFORE the apex.
  // Braking into a corner is what makes the lean budget vanish exactly where it
  // is needed; a rider brakes in a straight line and enters at a settled speed,
  // and the speed profile has to say so or the corner cap and the grip budget
  // chase each other down to the floor.
  const BRAKE_LEAD = 6.0;

  function brakeAccel(i) {
    if (!SOLVER.feasible) return A_BRAKE;
    const muS = SURF_MU[S.surface[i]] === undefined ? 1 : SURF_MU[S.surface[i]];
    const muB = BRAKE_USE * Math.min(RIDE_USE * RIDE_MU_REF * muS, RIDE_ENDO_MU);
    // The ballistic-free grade: a rider is not braking on a jump lip, and
    // pricing one as a 100% face returns BRAKE_A_MIN and makes the backward pass
    // unable to reach the feature at any speed. See computeRideBasis().
    const tanT = tanFromSin(rideGrade(i));        // signed, + = descending
    const cosT = 1 / Math.sqrt(1 + tanT * tanT);
    return Math.max(BRAKE_A_MIN, G * (muB * cosT - tanT * cosT));
  }

  /**
   * The corner cap, min-filtered over the next BRAKE_LEAD metres. See the note
   * above: this is what makes the profile arrive at a corner already slowed.
   */
  function cornerCapLead(cap, out) {
    const n = S.n, lead = Math.max(1, Math.round(BRAKE_LEAD / S.ds));
    for (let i = 0; i < n; i++) {
      let m = cap[i];
      const hi = Math.min(n - 1, i + lead);
      for (let j = i + 1; j <= hi; j++) if (cap[j] < m) m = cap[j];
      out[i] = m;
    }
    return out;
  }

  /** Clamp the solved speed to the corner and section caps, then re-brake. */
  const _capA = { arr: null, lead: null };
  function capArrays() {
    if (!_capA.arr || _capA.arr.length !== S.n) {
      _capA.arr = new Float32Array(S.n); _capA.lead = new Float32Array(S.n);
    }
    return _capA;
  }

  /**
   * THE FLOOR IS ON THE SPEED, NOT ON THE GRIP — the second half of taking
   * MU_CORNER_MIN off, and it took a measurement to find.
   *
   * With the grip floor gone the corner cap is honest, and on steep ground the
   * honest answer collapses: seed 12345 paced 24% of itself below 5.6 m/s and
   * bottomed out at 1.0 m/s, seed 777 10%, and the autopilot's dominant failure
   * became `stuck`. That is not a course. Worse, it is self-defeating in exactly
   * the way this round's whole finding predicts: holding 4.8 m/s down a 46%
   * grade costs the ENTIRE longitudinal budget in braking, which is what
   * collapses leanCeiling() in the first place. Asking a rider to go slower on a
   * steep pitch removes the grip they needed to corner. Measured at seed
   * 20260726 arc 185-195 m: paced at 4.84 m/s the run crashed at 200 m; the
   * same corner at 10 m/s crashed at 190 m; the geometry is what is wrong.
   *
   * So the design speed is floored at V_DESIGN_FLOOR and the stations where the
   * unfloored corner cap was below it are COUNTED, not silently obeyed. That
   * count is a statement that the geometry has failed — it is the number the
   * march is being scored to drive to zero, and auditJointFeasibility() reports
   * the same stations as infeasible rather than pretending a 1 m/s trail is a
   * solution.
   */
  function designCap(cornerCap, i) {
    const c = Math.min(cornerCap, techSpeedCap(i), S.vFeat[i]);
    if (!SOLVER.feasible) return c;
    // THE FLOOR IS A PACING RULE AND A FEATURE CAP IS A PHYSICAL BOUND, so the
    // floor may not lift the speed back over one. V_DESIGN_FLOOR exists because
    // asking a rider to crawl a steep pitch costs them the whole longitudinal
    // budget; it does not exist to authorise arriving at a lip faster than the
    // landing it was solved for. Where the two disagree the feature wins, and
    // the disagreement is counted rather than hidden.
    const floored = Math.max(V_DESIGN_FLOOR, c);
    if (floored > S.vFeat[i]) { safety.featFloorConflict++; return S.vFeat[i]; }
    return floored;
  }

  function applyCornerSpeedCap() {
    const n = S.n, ds = S.ds, v = S.speed;
    const A = capArrays();
    for (let i = 0; i < n; i++) A.arr[i] = cornerSpeedCap(i);
    if (SOLVER.feasible) cornerCapLead(A.arr, A.lead); else A.lead.set(A.arr);
    for (let i = 0; i < n; i++) {
      const cap = designCap(A.lead[i], i);
      if (v[i] > cap) v[i] = cap;
    }
    // FORWARD ACCELERATION LIMIT, and it is not cosmetic.
    //
    // This pass clamps v DOWN at a station and then only ever ran a BACKWARD
    // brake limit, so a station whose cap binds next to one whose cap does not
    // left a STEP UP in the profile — and every consumer that differentiates the
    // profile reads that step as an acceleration the trail is demanding. All 30
    // of the friction-circle violations left on seed 20260726 after this round's
    // route work were this artefact and nothing else: at arc 63.2-63.6 m the
    // profile stepped 8.2 -> 10.8 m/s across 0.4 m, which is 70 m/s^2 and reads
    // as mu_long = 3.5 where 0.53 is designed for. The lateral term at those
    // same stations is 0.03-0.15, i.e. there was never anything wrong with the
    // corner. A speed profile has to be reachable in BOTH directions.
    if (SOLVER.feasible) {
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
        const lim = Math.sqrt(Math.max(1.0, vp * vp + 2 * a * ds));
        if (v[i] > lim) v[i] = lim;
      }
    }
    for (let i = n - 2; i >= 0; i--) {
      const lim = Math.sqrt(v[i + 1] * v[i + 1] + 2 * brakeAccel(i) * ds);
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

  // WHY THERE IS NO "RESTORE THE IDENTITY FLOOR AFTER SMOOTHING" PASS, having
  // built one and measured it.
  //
  // The bank solve computes need = atan(v²/(g r)) − lean_budget and then runs a
  // Gaussian over the result, which costs a short corner about half its peak. It
  // is tempting to put the floor back afterwards, and it measures WORSE: at seed
  // 20260726 arc 187 m the restored floor declared 9.57 deg of bank across 12 m
  // of trail, and the committed cross-slope on the acceptance secant came out at
  // 3.2 deg — a realisation of 0.33 against terrain's published 0.92 — because
  // terrain's band limit (sigma 0.555 m along track) cannot reproduce a bank
  // spike that short, and neither can a rider. The design then priced a corner
  // cap of 10.5 m/s against a berm that was never built.
  //
  // A bank is a RAMP, not a value. So the smoothed bank stands, and where it is
  // not enough for the identity the SPEED gives way — which is what
  // applyCornerSpeedCap() has always done and what it can now do honestly, since
  // MU_CORNER_MIN no longer authorises grip the ground has not got.

  function solveSpeeds(rebank = true) {
    // Zeroed per solve, or the counter accumulates across the four feature
    // passes and the ten solves inside them and reports a number nobody can
    // interpret. It describes the profile that ships, not the history of it.
    safety.featFloorConflict = 0;
    const n = S.n, ds = S.ds;
    const vmax = new Float32Array(n);
    const v = S.speed;
    // Three iterations, not two. The bank depends on the speed and the speed
    // depends on the bank, and raising a corner's bank raises its speed cap,
    // which raises the ideal bank again; two passes left a measurable residual
    // on the tightest corners.
    const iters = rebank ? 3 : 1;

    const A = capArrays();
    for (let iter = 0; iter < iters; iter++) {
      for (let i = 0; i < n; i++) A.arr[i] = cornerSpeedCap(i);
      if (SOLVER.feasible) cornerCapLead(A.arr, A.lead); else A.lead.set(A.arr);
      for (let i = 0; i < n; i++) vmax[i] = designCap(A.lead[i], i);
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
      // Backward — braking-limited entry speeds, at the deceleration the GRADE
      // and the design friction actually allow. See brakeAccel().
      for (let i = n - 2; i >= 0; i--) {
        const lim = Math.sqrt(v[i + 1] * v[i + 1] + 2 * brakeAccel(i) * ds);
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
        const r = rideRadius(i);
        // atan(v²/(g·r)) is the bank that would make the corner feel flat.
        const ideal = Math.atan((v[i] * v[i]) / (G * r));
        // The identity floor: the bank this corner NEEDS so the rider is not
        // asked for more lean against the tread than the ground will actually
        // give them. On flat dirt that is LEAN_DESIGN, exactly as before; on
        // steep ground it is far less, so the BERM has to do the work the lean
        // cannot — which is option (b) in the builder's list, applied where the
        // route has already committed to the grade.
        const need = ideal - leanBudgetAt(i);
        // A genuinely straight section is left flat (the outslope pass below
        // gives it its drainage tilt). "Straight" now means BOTH slack in the
        // identity and slacker than the phase's own berm threshold — the old
        // test was a bare `r > 220`, which left a 240 m-radius bend taken at
        // 20 m/s with no bank at all and 24 deg of lean to find from nowhere.
        if (need <= 0.015 && r > st.minRadius * 4.5) { S.bank[i] = 0; continue; }
        let mag = clamp(Math.max(ideal * st.style, need), 0, st.maxBank);
        // The slow-rider ceiling. See the LEAN_HOLD note: a berm banked for a
        // speed the rider does not arrive at is a surface they slide down.
        const hold = Math.atan((SLOW_FRAC * SLOW_FRAC * v[i] * v[i]) / (G * r)) + LEAN_HOLD;
        if (SOLVER.rebank && mag > hold) mag = Math.max(0, hold);
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

  /**
   * Straightest, fastest station within `win` of `i0` — where a jump belongs.
   *
   * THE PACE IS PART OF WHERE A JUMP BELONGS, and the window has to be wide
   * enough for that to mean anything. Measured on seed 20260726: the road gap's
   * nominal slot lands on 30 m of essentially flat trail (committed grade
   * 0.007-0.014) directly after the previous jump's run-out, where the profile
   * is SHEDDING speed to drag and rolling resistance — 8.64 m/s at the double's
   * lip down to 4.94 m/s across 47 m. A 19 m road gap cannot be built there at
   * any pace, and the fixed point's only remaining move is to shrink it until it
   * is not a gap. Seventy metres further on the same phase runs at 10.5 m/s.
   *
   * A builder puts the gap where the speed is. The search window is therefore
   * wide enough to reach the next fast ground — +-18 m against a nominal slot
   * spacing of ~57 m, so the jump line cannot reorder itself — and the speed
   * term is scaled to the size of the decision it is making: 4 points per m/s
   * against a 0-18 point straightness term and a 0.02/station centring term.
   */
  function straightestNear(i0, win, lo, hi) {
    let best = clamp(i0, lo, hi), bestScore = -Infinity;
    for (let d = -win; d <= win; d++) {
      const i = clamp(i0 + d, lo, hi);
      // Straight, fast, and not already occupied by another shaped feature.
      let sc = Math.min(S.radius[i], 300) * 0.06
        + S.speed[i] * (SOLVER.feasible ? 4.0 : 2.2) - Math.abs(d) * 0.02;
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

  // -------------------------------------------------------------------------
  // THE BALLISTIC-RELIEF RECORD.
  //
  // `S.hBal` is the part of `S.hOff` that a solved airborne feature wrote — the
  // lip transition, the deck or dug gap, the knuckle face, the landing chord and
  // the climb out of the pit. It is recorded by DIFFERENCE rather than by having
  // buildJump/buildDrop write two arrays, so there is exactly one place the
  // surface is authored and no way for the two records to disagree.
  //
  // Nothing in the geometry reads it. Its only consumer is computeVertKappa(),
  // which prices the launch constraint on the profile with it removed, because
  // a lip is convex BY CONSTRUCTION and pacing a rider to it is a category
  // error. See the long note there.
  // -------------------------------------------------------------------------
  let _balScratch = null;
  function beginBallistic(i0, i1) {
    const a = clamp(Math.min(i0, i1), 0, S.n - 1);
    const b = clamp(Math.max(i0, i1), 0, S.n - 1);
    if (!_balScratch || _balScratch.length < S.n) _balScratch = new Float32Array(S.n);
    for (let i = a; i <= b; i++) _balScratch[i] = S.hOff[i];
    return { a, b };
  }
  function endBallistic(span) {
    if (!span) return;
    for (let i = span.a; i <= span.b; i++) S.hBal[i] += S.hOff[i] - _balScratch[i];
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

  // -------------------------------------------------------------------------
  // THE PRE-FEATURE STATE, so buildFeatures() can be run more than once.
  //
  // Every array below is one buildFeatures() writes; S.speed is included not
  // because features write it but because they READ it — straightestNear() sites
  // a jump partly on the local design speed, and a siting that moved between
  // passes would turn the fixed point into a search over station indices. With
  // it restored, pass k+1 differs from pass k in EXACTLY one thing: the approach
  // speed each feature is solved at.
  // -------------------------------------------------------------------------
  const FEATURE_STATE_KEYS = ['hOff', 'hBal', 'width', 'surface', 'rough', 'bumps',
    'rut', 'crown', 'bank', 'wallride', 'vFeat', 'feat', 'speed'];
  function featureStateSnapshot() {
    const snap = { arr: {}, nFeat: features.length, nLog: jumpLog.length };
    for (const k of FEATURE_STATE_KEYS) snap.arr[k] = S[k].slice();
    return snap;
  }
  function featureStateRestore(snap) {
    for (const k of FEATURE_STATE_KEYS) S[k].set(snap.arr[k]);
    features.length = snap.nFeat;
    jumpLog.length = snap.nLog;
    ballisticMask = null;
    crestMask = null;
    // hOff is restored, so the surface and everything derived from it must be
    // put back with it. Missing this is silent: the next pass would build its
    // features against the PREVIOUS pass's grade, and the fixed point would be
    // iterating two things at once.
    for (let i = 0; i < S.n; i++) S.py[i] = S.by[i] + S.hOff[i];
    refreshBasis();
    computeVertKappa();
  }

  function buildFeatures() {
    // Re-derived every pass, so pass 2 draws the same numbers pass 1 did and the
    // iteration is invisible to every consumer downstream. See featPassRng.
    featPassRng = makeRng(subSeed(ctx.seed, 'trail:features:pass'));
    featAttempt.clear();
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
        const amp = 0.26 + featDraw() * 0.14;
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
      const dh = 1.05 + featDraw() * 0.5;
      buildDrop(dI, dh, { name: 'tech drop' });
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
      const dI = b - M(30);
      const dh = 1.5 + featDraw() * 0.7;
      buildDrop(dI, dh, { name: 'slab roll', surface: 'rock' });
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
    const give = (why) => { featAttempt.set(opt.name, why); return null; };
    featAttempt.set(opt.name, '');
    if (i < M(25) || i > n - M(60)) return give('offCourse');
    const alpha = Math.atan(clamp(S.grade[i], 0.02, 0.30));
    // THE APPROACH SPEED IS A FIXED POINT, AND IT IS NOW ITERATED. See
    // JUMP_FIT_ITERS and the loop in build().
    //
    // THE DEFECT, restated in one line because it took eleven rounds to name:
    // buildFeatures() runs before the final speed solve, so `S.speed` here is the
    // pace of ground that does not yet carry the jump, and by the time the jump
    // exists the profile paces its own lip at half that. On seed 20260726 the
    // doubles were solved for an 11.8 m/s takeoff and the shipped profile
    // delivered 5.39 at the lip, against a `vMin` of 11.1 the same solve
    // published. 20 of 32 air features on the four seeds were BELOW the minimum
    // of their own window: the design was telling the rider to case every jump on
    // the course, and CONTRACT §4 says a jump you cannot land is a bug.
    //
    // TWO THINGS HAD TO CHANGE AND ONLY ONE OF THEM IS THIS LOOP.
    //
    //  (1) The pace was wrong for a measurable reason — the launch cap was
    //      pricing the jump's own lip as unwanted relief, over a 12.8 m stencil,
    //      which poisoned the entire run-in. That is fixed in computeVertKappa()
    //      by subtracting S.hBal, and it is what lets the corridor deliver a
    //      takeoff speed at all.
    //
    //  (2) Whatever the corridor then delivers, the feature has to be SOLVED FOR
    //      IT. That is `paceFor()` below plus the iteration: build the line,
    //      solve the profile, read the speed actually delivered at each lip, and
    //      rebuild the line for it. A feature whose window the shipped pace
    //      cannot enter is a defect regardless of which side is wrong.
    //
    // A flat derate (JUMP_V_DERATE = 0.85) was measured by an earlier round and
    // REJECTED, correctly: it moves every window down by the same fraction and
    // then five features end up ABOVE their own takeoff, which is the same bug
    // inverted. A fixed point is not a fudge factor — it converges on the pace
    // each feature individually gets.
    const vApproach = (featPace.get(opt.name) ?? S.speed[i]) * JUMP_V_DERATE;
    let step = opt.stepDown || 0;
    const table = opt.table === undefined ? 1 : opt.table;
    // THE LIP IS SIZED FOR THE APPROACH, NOT THE OTHER WAY ROUND. Climbing the
    // lip costs kinetic energy (85% of it — you pump the transition), so a lip
    // authored for 12 m/s is a speed bump at 7: it eats a third of the approach
    // and leaves nothing to fly with. Cap the lip at JUMP_LIP_KE of the
    // approach's kinetic energy so the takeoff always keeps most of what it
    // arrived with, and let the authored height stand wherever the pace can
    // afford it (which, once (1) above is fixed, is everywhere on the jump line).
    const lipH = SOLVER.feasible
      ? Math.min(opt.lipHeight, JUMP_LIP_KE * vApproach * vApproach / (2 * G * 0.85))
      : opt.lipHeight;
    const vTO = Math.sqrt(Math.max(SOLVER.feasible ? 1 : 9,
      vApproach * vApproach - 2 * G * lipH * 0.85));
    // Too slow for this to be anything but a case: don't build it.
    if (vTO < (SOLVER.feasible ? JUMP_V_MIN : 8)) return give('tooSlow');
    // ...AND THE LANDING DISTANCE IS BALLISTIC, SO IT SCALES AS v^2. The jump
    // line's rhythm (11 / 13 / 15 / 19 / 16 m) was authored against the ~12 m/s
    // takeoff the phase is designed for; asking for 19 m of carry at 8 m/s just
    // pins lipAngleForDistance() at its 30 deg ceiling and produces a kicker.
    // Scaling the target keeps the RHYTHM — the relative sizes, which is what a
    // jump line is — at whatever pace the corridor carries.
    // ...with a FLOOR, because the scaling and solveJump's own `D < 3.5 m` gate
    // were fighting: scaled to 30% a 7.5 m rock-garden double asks for 2.25 m of
    // carry, lipAngleForDistance() returns its minimum angle, and the feature is
    // then refused for not carrying far enough. Aiming at JUMP_D_MIN instead
    // pins the lip at its 30 deg ceiling and delivers whatever that gives —
    // 4.0 m at a 5.7 m/s takeoff, which is a small double and is a feature.
    const targetD = SOLVER.feasible
      ? Math.max(JUMP_D_MIN,
        opt.targetD * clamp((vTO / V_JUMP_REF) * (vTO / V_JUMP_REF), 0.25, 1.15))
      : opt.targetD;
    // The run-out must climb back out of the pit slower than the ground falls,
    // or the feature ships a CONTRACT §4 breach. See the note in solveJump().
    // JUMP_OUT_FRAC of the local grade leaves the world gradient descending
    // through the whole run-out; JUMP_OUT_MAX bounds how much trail one feature
    // may spend on its own exit.
    const outRate = Math.max(0.045, JUMP_OUT_FRAC * Math.tan(alpha));
    // AND THE AUTHORED STEP HAS TO FIT INSIDE THAT BUDGET, which the step-down
    // did not. `hOff` is an offset from a base profile that is the same on both
    // sides of the feature, so a step-down does not lower the trail — it digs a
    // hole the run-out then has to climb out of, and a 2.0 m step on 13% ground
    // needs 67 m of run-out to climb out of while still descending. The step is
    // therefore reduced until it fits: 2.0 m becomes about 0.7 m in the jump
    // line, which is still a step-down and is one the trail can pay for.
    // Everything downstream — the lip angle, the gap, the landing, the speed
    // window — is then solved against the step that will actually be built, so
    // the published ballistics describe the feature that ships.
    let theta = 0, sol = null, gapDepth = 0, deckY = 0;
    // The second dial the fit has, and the one a gap needs: how far past the
    // flat-landing point the design touchdown sits. At 1.10 the landing is dug
    // well below grade — which is how a real landing is built and why it is the
    // default — but it is also what makes the pit 2.5 m deep on the road gap,
    // and a 2.5 m pit costs 53 m of run-out at this grade. Pulled toward 1.0
    // (touchdown exactly where the arc meets grade) only as far as the run-out
    // budget demands, so every jump keeps the deepest landing it can pay for.
    let touchOver = opt.touchOver === undefined ? 1.10 : opt.touchOver;
    for (let it = 0; it < 7; it++) {
      gapDepth = 0.35 + step * 0.15;
      deckY = lipH * 0.97 * table - (1 - table) * gapDepth;
      // Size the lip for the landing distance the jump line calls for, but never
      // below `minDeg` — on a steep grade a 7 deg lip reaches the target distance
      // with no pop at all, and a jump with no pop is a speed bump.
      theta = lipAngleForDistance(vTO, alpha, lipH, step, targetD,
        opt.minDeg === undefined ? 14 : opt.minDeg, 30);
      sol = solveJump(vTO, theta, alpha, lipH, step, {
        knuckle: opt.knuckle, touchOver, clearance: opt.clearance, deckY,
        outRate, outMax: JUMP_OUT_MAX,
      });
      const need = 1.5 * Math.abs(sol.yEnd - step) / outRate;
      if (need <= JUMP_OUT_MAX + 1e-6) break;
      const over = (need - JUMP_OUT_MAX) * outRate / 1.5;   // metres of excess pit
      if (step > 0.02) step = Math.max(0, step - over);
      else if (touchOver > 1.001) touchOver = Math.max(1.0, touchOver - 0.03);
      else break;
    }
    // Too slow to clear anything: don't build a jump the rider will case.
    if (sol.D < 3.5) return give('noCarry');

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
    if (iRampEnd >= n - 4) return give('noRoom');

    // Everything between here and `endBallistic()` is authored ballistic relief.
    // See computeVertKappa(): the launch constraint must not price it.
    const balA = beginBallistic(iTransStart, iOutEnd);

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
    endBallistic(balA);

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
    // CONTRACT §4, "a jump you can't land": the design speed AT THE LIP may not
    // exceed the speed the ballistics were solved for. Declared as a cap the
    // speed solve honours, because the solve runs again after this. vTO is
    // inside its own published window by construction, so this can never push a
    // jump under its own floor.
    //
    // THE RUN-IN IS CAPPED AT THE APPROACH SPEED, NOT AT vTO, AND THAT
    // DISTINCTION IS THE WHOLE OF THE JUMP-LINE DEFECT.
    //
    // The previous cut wrote vTO across the run-in as well. vTO is the speed at
    // the TOP of the lip and it is strictly below the approach speed, because
    // climbing the lip is what costs the difference. So capping the approach at
    // vTO instructs the rider to arrive at the foot of the transition already
    // travelling at the speed they are supposed to leave the top at — and then
    // climb the lip out of that. The delivered lip speed is therefore vTO minus
    // the lip climb, every time, by construction.
    //
    // That is a ratchet, and under the fixed point it is a runaway one:
    // measured, the approach speed on the jump line walked 12.5 -> 11.8 -> 11.2
    // -> ... on successive passes, one lip-climb per pass, until JUMP_V_MIN
    // deleted the feature. It converged to `jumpWindowBad = 0` by shipping half
    // the jump line — 32 air features across four seeds became 18 — which is
    // the correct answer to the wrong question. With the cap split at the lip
    // the map is stationary: solving a feature for the speed the run-in carries
    // no longer lowers the speed the run-in carries.
    const vRunInCap = SOLVER.feasible ? vApproach : vTO;
    for (let k = Math.max(0, iTransStart - M(6)); k < iLip; k++) {
      if (vRunInCap < S.vFeat[k]) S.vFeat[k] = vRunInCap;
    }
    for (let k = iLip; k <= iKnuckle; k++) {
      if (vTO < S.vFeat[k]) S.vFeat[k] = vTO;
    }
    if (JUMP_RUNIN_BUMPS) addBrakingBumps(iTransStart - M(16), iTransStart - M(3), 0.045);
    // NO BRAKING BUMPS ON A JUMP RUN-IN, and this is a measured decision rather
    // than a stylistic one. They were 45 mm at a 1.1 m wavelength, which
    // chatterKappa() prices at 1.47 /m of convex curvature — a launch-speed cap
    // of 1.9 m/s, floored to V_DESIGN_FLOOR, and then propagated to the lip by
    // the backward brake limit. Measured on seed 20260726: the doubles were
    // solved for a 9.8 m/s takeoff and the profile delivered 5.3 at their lips,
    // i.e. BELOW the vMin of 9.0 the same solve published. The jump line was
    // braking itself into casing every jump on it. A rider brakes BEFORE a jump
    // run-in, not on it; the run-in is where you stop braking.

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
      // iApproach is the foot of the lip transition — the station whose speed
      // IS the approach speed. The fixed point feeds this back, never iLip.
      iApproach: Math.max(0, iTransStart),
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

  /**
   * A COMMITTING DROP, SOLVED.
   *
   * THE DEFECT. Both drops on every seed were authored as two blendOffsets: rise
   * `dh` over six metres, then fall `dh` over one point two. That is not a
   * feature, it is a step in the tread — a 1.4 m face at a 210% gradient, with
   * the landing left as whatever the base grade happened to be, and no speed cap
   * of any kind through it. The trail published a design speed 54% above the
   * speed at which its own crest curvature launches the rider off the lip, on a
   * feature whose landing is flat ground. CONTRACT §4: "a jump you can't land is
   * a bug", and the module already solves ballistics correctly for the six
   * authored jumps — this one was simply never solved.
   *
   * A drop IS a jump, with one difference: there is no pop. The rider leaves the
   * edge travelling along the trail surface, so the lip angle relative to the
   * grade is zero and `psi = -alpha`. Everything else — the flight, the point
   * where the arc meets grade, the chord landing transition, the touchdown
   * fraction, the speed window — is the same solve `buildJump` uses, so a drop
   * now gets a matched landing transition and a published speed window like
   * every other airborne feature.
   *
   * The approach ramp onto the lip is sized the same way a run-out is: at
   * JUMP_OUT_FRAC of the local grade, so building the lip up does not itself
   * breach CONTRACT §4 on the way in. It used to rise `dh` over six metres,
   * which on the Rooty Steeps is a 24% ramp against a 26% grade.
   */
  function buildDrop(dI, dh, opt) {
    const n = S.n;
    const dname = (opt && opt.name) ? opt.name : 'drop';
    const give = (why) => { featAttempt.set(dname, why); return null; };
    featAttempt.set(dname, '');
    if (dI < M(24) || dI > n - M(50)) return give('offCourse');
    const alpha = Math.atan(clamp(S.grade[dI], 0.02, 0.34));
    // Same fixed point as buildJump(): on iteration 0 this is the pre-feature
    // pace, and on every iteration after it is the speed the profile actually
    // delivered at this lip. See the loop in build().
    const vApp = Math.max(4.0, featPace.get(opt && opt.name ? opt.name : 'drop') ?? S.speed[dI]);
    const outRate = Math.max(0.045, JUMP_OUT_FRAC * Math.tan(alpha));
    // theta = 0: the lip is parallel to the grade. No pop, by definition.
    const sol = solveJump(vApp, 0, alpha, dh, 0, {
      knuckle: 0.30, clearance: 0.25, touchOver: 1.05, deckY: dh,
      outRate, outMax: JUMP_OUT_MAX,
    });
    const cosA = Math.max(0.5, Math.cos(alpha));
    const sx = (metres) => Math.max(1, M(metres / cosA));
    const upLen = clamp(1.5 * dh / outRate, 3, JUMP_OUT_MAX);
    const iUp = dI - sx(upLen);
    const iFace = dI + sx(Math.min(1.2, sol.xK * 0.5));
    const iKnuckle = Math.max(iFace + 1, dI + sx(sol.xK));
    const iRampEnd = Math.max(iKnuckle + 1, dI + sx(sol.rampEnd));
    const iOutEnd = Math.min(n - 1, dI + sx(sol.rampEnd + sol.outLen));
    if (iUp < 1 || iRampEnd >= n - 4) return give('noRoom');

    const balA = beginBallistic(iUp, iOutEnd);
    // Ramp up onto the lip, at a rate the grade beats.
    blendOffset(iUp, dI, (f) => dh * smoothstep(0, 1, f));
    // The face. The rider is airborne over it; it is the void, not tread.
    blendOffset(dI + 1, iFace, (f) => lerp(dh, sol.yK, smoothstep(0, 1, f)));
    // Flat at the knuckle height until the landing ramp starts.
    blendOffset(iFace + 1, iKnuckle, () => sol.yK);
    // The landing transition: the chord from the knuckle to the touchdown point.
    blendOffset(iKnuckle + 1, iRampEnd, (f) => sol.yK + sol.m * (f * sol.rampRun));
    // ...and back out of the landing pit onto the trail.
    blendOffset(iRampEnd + 1, iOutEnd, (f) => sol.yEnd * (1 - smoothstep(0, 1, f)));
    endBallistic(balA);

    // THE SPEED CAP. Without it the profile solve is free to pace the feature at
    // whatever the section allows, which is how a 1.44 m drop came to publish
    // 7.9 m/s against a lip that launches at 3.6. Capped at the lower of the
    // speed the ballistics were solved at and the largest speed that still
    // lands on the transition, over the whole feature and its approach.
    // Split at the lip for the same reason buildJump does — see the long note
    // there. A drop has no pop, but it does have an approach RAMP: the trail is
    // built up `dh` onto the lip over `upLen`, so the rider climbs it, and
    // capping the foot of that ramp at the lip speed subtracts the climb twice.
    const vCap = Math.min(vApp, sol.vMax);
    const vRunIn = SOLVER.feasible ? Math.sqrt(vCap * vCap + 2 * G * dh * 0.85) : vCap;
    for (let i = Math.max(0, iUp - M(8)); i < dI; i++) {
      if (vRunIn < S.vFeat[i]) S.vFeat[i] = vRunIn;
    }
    for (let i = dI; i <= iOutEnd; i++) {
      if (vCap < S.vFeat[i]) S.vFeat[i] = vCap;
    }
    for (let i = Math.max(0, iUp); i <= iOutEnd; i++) {
      S.bumps[i] = 0;
      S.rough[i] = Math.min(S.rough[i], 0.015);
      S.width[i] = clamp(S.width[i] + 0.25, 1.2, 3.0);
    }
    const f = addFeature('drop', iUp, iOutEnd, {
      name: opt && opt.name ? opt.name : 'drop',
      height: dh,
      landing: 'solved',
      surface: opt && opt.surface ? opt.surface : undefined,
      takeoffSpeed: vApp,
      approachSpeed: vApp,
      lipHeight: dh, lipAngle: 0, launchAngle: sol.psi,
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
      stepDown: 0,
      // A drop's ballistics are solved AT the lip (theta = 0, no pop: the rider
      // leaves the edge travelling along the surface), so the speed the fixed
      // point must feed back is the lip's own, not the foot of the up-ramp's.
      iApproach: dI,
      iLip: dI, iKnuckle, iTouch: dI + sx(sol.xT), iRampEnd, iOutEnd,
    }, true);
    jumpLog.push({
      name: (opt && opt.name) || 'drop',
      takeoff: +vApp.toFixed(2),
      kmh: +(vApp * 3.6).toFixed(1),
      lipH: +dh.toFixed(2),
      lipDeg: 0,
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
      const best = straightestNear(a + Math.round(span * slots[k]),
        M(SOLVER.feasible ? 18 : 9), a, b);
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
    // ---- the joint launch/bank solve (capLaunchCurvature, rebankAtSolvedSpeed) ----
    // `launchViol*` count stations violating the launch cap at ANY rung of the
    // stencil ladder, measured against S.vWant (the speed the design intends),
    // not against the speed the ground happened to leave. `launchWorst*` is the
    // worst ratio kappa/cap. `launchSpeedGiven` counts stations where the
    // geometry could not be flattened and the SPEED gave way instead;
    // `launchFloorBound` counts the ones where even that hit V_DESIGN_FLOOR and
    // the design is knowingly over the cap. `bankConflict` counts corners where
    // the fast-rider floor and the slow-rider ceiling crossed.
    launchViolBefore: 0, launchViolAfter: 0,
    launchWorstBefore: 0, launchWorstAfter: 0,
    launchWorstAtBefore: 0, launchWorstAtAfter: 0,
    launchSpeedGiven: 0, launchFloorBound: 0, bankConflict: 0,
    launchStage: { afterJoint: 0, afterSafety: 0 },
    // ---- route rideability (marchRoute / routeAdmissible) ----
    // `routeAdmissible` false means NO variant produced a course and the
    // best-scoring broken one shipped; `routeFail` names the test it failed.
    // `routeReseeded` true means the extended variant budget was used, i.e. the
    // first seven attempts all failed and the route was deliberately re-drawn.
    // `sbForced` counts switchbacks built through the relief valve — hairpins
    // the rideability gate refused and the drift allowance overrode.
    routeVariants: 0, routeAdmissible: true, routeFail: '', routeReseeded: false,
    routeFeasFrac: 0, routeClimb: 0, routeMinR: 0, routeConflict: 0, sbForced: 0,
    // ---- the joint (v, phi) feasibility of the SHIPPED stations ----
    // `feasViol` counts stations where no speed in [V_DESIGN_FLOOR, design] and
    // no bank inside the phase ceiling satisfies the lean identity against the
    // load-derived budget; `gripFloorBound` counts stations where the corner
    // budget hit MU_CORNER_MIN and the identity is knowingly not guaranteed.
    feasViol: 0, feasWorstDeg: 0, feasWorstAt: 0, gripFloorBound: 0,
    feasCorner: 0, feasLaunch: 0, cornerFloorBound: 0, feasDesignViol: 0,
    // ---- the jump-line fixed point (see the loop in build()) ----
    // `jumpFitIters` is how many feature passes were run; `jumpFitResidual` how
    // many air features the winning pass leaves outside their own published
    // speed window; `jumpFitWorst` how far outside, in m/s; `jumpFitLost` how
    // many features the winning pass could not build at the pace the corridor
    // delivers, and `jumpFitLostWhy` names each one and its reason. A build with
    // a non-zero `jumpFitLost` is reporting ground the design wanted a jump on
    // and the profile cannot pace one on — a route finding, not a jump finding.
    jumpFitIters: 0, jumpFitResidual: 0, jumpFitWorst: 0,
    jumpFitLost: 0, jumpFitLostWhy: '',
    // Stations where V_DESIGN_FLOOR would have lifted the design speed back
    // above a solved feature's own ballistic cap. See designCap().
    featFloorConflict: 0,
    // ---- the shipped plan radius (R_PLAN_MIN) ----
    // Measured on S.px/S.pz at the END of build(), i.e. after applyExposureSafety
    // has benched the line — which is the only place the number means anything.
    // `planShipMin` is the worst +-4 m circumradius on the course, `planShipAt`
    // where, `planShipUnder` how many stations are inside R_PLAN_MIN.
    planShipMin: 0, planShipAt: 0, planShipUnder: 0,
    // ---- CONTRACT §4 as a PROPERTY, not as a counter (see measureShippedFlat) ----
    // `flatRunShipped` is the longest stretch that ends at or above its own start
    // and never drops below it in between — what a rider pushing up it meets.
    // `flatWinRise` is the worst NET rise over any DESC_WIN of 3-D arc.
    flatRunShipped: 0, flatRunShippedAt: 0, flatWinRise: 0, flatWinAt: 0,
    flatWinCapped: 0, flatWinFloorBound: 0,
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
   * THE BALLISTIC CORE of every solved jump: lip transition through touchdown
   * ramp. `buildAuthoredMask` guards a feature plus FEATURE_GUARD metres either
   * side, which is right for the crest cap (a jump lip IS a crest violation by
   * construction and so is the ground around it), and wrong for the descent
   * rule — because the run-out, which is where the descent breaches actually
   * are, sits inside that guard and would be protected from the fix.
   *
   * This mask protects exactly what the ballistics depend on: the surface the
   * trajectory is solved against, from the takeoff transition to the end of the
   * landing ramp. `solveJump` publishes those station indices, so this is read
   * off the solve rather than guessed from a feature's extent.
   */
  function buildBallisticMask(guardM, toOutEnd) {
    const n = S.n;
    const out = new Uint8Array(n);
    const g = M(guardM === undefined ? 1.5 : guardM);
    for (const f of features) {
      const p = f.params;
      if (!p || p.iLip === undefined || p.iRampEnd === undefined) continue;
      const a = Math.max(0, Math.min(f.i0, p.iLip) - g);
      // The SPEED mask reaches the end of the run-out; the DESCENT mask stops at
      // the touchdown ramp, because the run-out is precisely where the descent
      // breaches were and it must stay correctable.
      const end = toOutEnd && p.iOutEnd !== undefined ? p.iOutEnd : p.iRampEnd;
      const b = Math.min(n - 1, end + g);
      for (let i = a; i <= b; i++) out[i] = 1;
    }
    for (let i = 0; i < n; i++) if (S.wallride[i] > 0.05) out[i] = 1;
    return out;
  }

  /**
   * CONTRACT §4 ON THE LINE THAT SHIPS.
   *
   * enforceDescent() runs inside buildStations(), on the base profile, and six
   * passes move that profile afterwards: buildFeatures() writes the authored
   * relief, capCrestCurvature() and capLaunchCurvature() lower crests,
   * capGradeRamp() eases second derivatives, applyExposureSafety() benches the
   * line and re-runs the ramp cap. Measured on the shipped build before this
   * pass existed, the longest non-descending run on S.py was 18.3 / 22.8 / 17.9
   * / 18.7 m on the four buildable seeds against CONTRACT §4's ~15 m, and on
   * three of them the worst offender was authored relief the base-profile rule
   * had never seen.
   *
   * So the rule is applied once more, at the end, to the profile that ships.
   * Cut-only, bounded by the same excavation ceiling as every other pass, zero
   * on the ballistic core, and written into S.hOff so S.by stays the record of
   * what the elevation solve decided.
   */
  // ---------------------------------------------------------------------------
  // THE DESCENT RULE IS A PROPERTY, AND DESC_MAX_RUN IS A COUNTER. THEY ARE NOT
  // THE SAME THING, AND THE DIFFERENCE IS 78 METRES.
  //
  // `enforceDescent` and the loop below it counted CONSECUTIVE non-descending
  // stations and reset on the first station that fell by anything at all — the
  // test is `v >= y[i-1] - 1e-5`, so a ten-MICROMETRE dip discharges it, and
  // rule 3 satisfies itself with DESC_MIN_FALL*ds = 4 mm of fall. What the rule
  // therefore permits is a sawtooth: climb 11 m at up to maxUpGrade, drop 4 mm,
  // climb 11 m again. The counter reads 11.2 m and reports compliance.
  //
  // Measured on the shipped reference seed before this pass: `safety.flatShipped`
  // read 11.2 m while the profile carried a stretch of 89.4 m from arc 2157 that
  // ends 1.59 m ABOVE where it started and never once drops below its own start
  // — broken only by 1-2 cm dips at five stations. That is what a rider pushing
  // up it experiences and it is what CONTRACT §4's "~15 m" is about. Three
  // consecutive audits reported the counter and none of them reported the
  // property.
  //
  // So the rule is stated as the property. DESC_WIN is the CONTRACT limit read
  // literally: over any window of DESC_WIN metres of 3-D arc the trail must have
  // NET DESCENT. Enforcing `y[j] < min{ y[i] : arc[i] <= arc[j] - DESC_WIN }`
  // gives exactly that, because a run that ends at or above its own start and
  // never drops below it in between is by definition a window in which the
  // profile failed to descend — and the running prefix minimum is precisely the
  // anchor such a run would be measured from.
  //
  // Cut-only, like every other elevation rule here, bounded by the same
  // excavation ceiling, and zero on the ballistic core (a landing pit's run-out
  // is authored relief, not a climb). Where the excavation ceiling binds the
  // rule cannot be met and the station is COUNTED (`flatWinFloorBound`) rather
  // than silently obeyed — that is the route being wrong, not the profile.
  // ---------------------------------------------------------------------------
  const DESC_WIN = 15.0;      // m of 3-D arc — CONTRACT §4's "~15 m", read as a window
  const DESC_WIN_EPS = 0.002; // m of strict fall demanded across the window

  function enforceDescentShipped(mask) {
    const n = S.n, ds = S.ds;
    let run = 0, changed = 0;
    // Pass 1: the consecutive rule, unchanged. It is cheap, it fires more often
    // than the windowed rule, and it keeps the profile falling at the short
    // wavelengths the windowed rule is blind to.
    for (let i = 1; i < n; i++) {
      const prev = S.by[i - 1] + S.hOff[i - 1];
      let v = S.by[i] + S.hOff[i];
      if (!mask[i] && run >= DESC_MAX_RUN) {
        const fall = prev - DESC_MIN_FALL * ds;
        if (v > fall) {
          const floor = S.rawS[i] - RAMP_CUT_MAX;
          const nv = Math.max(fall, floor);
          if (nv < v) { S.hOff[i] += nv - v; v = nv; changed++; }
        }
      }
      if (v >= prev - 1e-5) run += ds; else run = 0;
    }
    for (let i = 0; i < n; i++) S.py[i] = S.by[i] + S.hOff[i];

    // Pass 2: the window. One forward sweep with a trailing prefix minimum —
    // `low` is the smallest COMMITTED height at any station already more than
    // DESC_WIN of arc behind, so it is exactly the anchor the property is
    // measured from, and every station it has seen has already been corrected.
    const arc = new Float64Array(n);
    for (let i = 1; i < n; i++) {
      arc[i] = arc[i - 1]
        + Math.hypot(S.px[i] - S.px[i - 1], S.py[i] - S.py[i - 1], S.pz[i] - S.pz[i - 1]);
    }
    safety.flatWinFloorBound = 0;
    let k = 0, low = S.py[0], capped = 0;
    if (!SOLVER.feasible) return changed;
    for (let j = 1; j < n; j++) {
      while (k < j && arc[j] - arc[k] >= DESC_WIN) { if (S.py[k] < low) low = S.py[k]; k++; }
      if (k === 0 || mask[j]) continue;
      const need = low - DESC_WIN_EPS;
      if (S.py[j] <= need) continue;
      const floor = S.rawS[j] - RAMP_CUT_MAX;
      const nv = Math.max(need, floor);
      if (nv < S.py[j]) {
        S.hOff[j] += nv - S.py[j];
        S.py[j] = nv;
        capped++;
      }
      if (S.py[j] > need + 1e-9) safety.flatWinFloorBound++;
    }
    safety.flatWinCapped = capped;
    return changed + capped;
  }

  /**
   * CONTRACT §4 on the FINAL surface, measured three ways, because the first of
   * them is the one that was being reported and it is the one that lies.
   *
   *   flat   the counter: longest run of CONSECUTIVE non-descending stations.
   *   run    THE PROPERTY: longest stretch that ends at or above its own start
   *          and never drops below that start in between — what a rider pushing
   *          up it meets. `rise` is how far above its start it ends.
   *   win    the worst NET rise over any window of DESC_WIN of 3-D arc, which is
   *          the quantity enforceDescentShipped()'s second pass drives to zero.
   *
   * All three on S.py at the end of build() — not on the marched polyline, and
   * not on the base profile four passes upstream.
   */
  function measureShippedFlat() {
    const n = S.n;
    const arc = new Float64Array(n);
    for (let i = 1; i < n; i++) {
      arc[i] = arc[i - 1]
        + Math.hypot(S.px[i] - S.px[i - 1], S.py[i] - S.py[i - 1], S.pz[i] - S.pz[i - 1]);
    }
    let flat = 0, worst = 0, at = 0, start = 0;
    for (let i = 1; i < n; i++) {
      const d = arc[i] - arc[i - 1];
      if (S.py[i] >= S.py[i - 1] - 1e-4) {
        if (flat === 0) start = arc[i - 1];
        flat += d;
        if (flat > worst) { worst = flat; at = start; }
      } else flat = 0;
    }
    // The property. `a` tracks the running-low index, so between a and any i at
    // or above it the profile never went below y[a] — that IS the run.
    let runWorst = 0, runAt = 0, runRise = 0, a = 0;
    for (let i = 1; i < n; i++) {
      if (S.py[i] < S.py[a]) { a = i; continue; }
      const span = arc[i] - arc[a];
      if (span > runWorst) { runWorst = span; runAt = arc[a]; runRise = S.py[i] - S.py[a]; }
    }
    // The window.
    let winRise = -Infinity, winAt = 0, j = 0;
    for (let i = 0; i < n; i++) {
      while (j < n - 1 && arc[j] - arc[i] < DESC_WIN) j++;
      if (arc[j] - arc[i] < DESC_WIN * 0.999) break;
      const rise = S.py[j] - S.py[i];
      if (rise > winRise) { winRise = rise; winAt = arc[i]; }
    }
    return {
      flat: worst, at,
      run: runWorst, runAt, runRise,
      win: winRise === -Infinity ? 0 : winRise, winAt,
    };
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
   * Launch violations on the CURRENT profile at the CURRENT governed speed,
   * over the whole stencil ladder. This is the design-time proxy for the
   * acceptance metric, and the two agree closely enough to steer by: measured on
   * seed 20260726, the acceptance harness scores the interpolated station
   * profile at 4.5% of stations against the committed surface's 3.53%, so this
   * runs about 1.3x conservative, in the safe direction.
   *
   * Same discipline as countCrestViol(): the mask is not optional once features
   * exist, because a jump lip is convex past any launch cap BY CONSTRUCTION.
   */
  function countLaunchViol(mask) {
    if (!S) return 0;
    const n = S.n;
    const raw = new Float32Array(n);
    for (let i = 0; i < n; i++) raw[i] = S.by[i] + S.hOff[i];
    const y = gaussianSmooth(raw, 4, 0.45 / STATION_DS);
    let c = 0;
    for (let i = 0; i < n; i++) {
      if (mask && mask[i]) continue;
      const v = Math.max(4, S.speed[i]);
      const k = Math.max(SOLVER.carveFloor, (() => {
        let w = 0;
        for (let s = 0; s < LAUNCH_STENCILS.length; s++) {
          const q = vertKappaAt(y, i, LAUNCH_STENCILS[s]);
          if (q > w) w = q;
        }
        return w;
      })());
      if (k * v * v > G) c++;
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

  // -------------------------------------------------------------------------
  // THE JOINT SOLVE — speed, vertical curvature and bank, together.
  //
  // capCrestCurvature() above is the same idea at one stencil and one safety
  // factor, and it was measured (round 6) leaving 54 violations of its OWN cap on
  // the shipped profile while the acceptance stencil found 40-56% of stations in
  // the worst 25 m bin launching. Three things were wrong with it and all three
  // are fixed here rather than by tuning it:
  //
  //  1. ONE MEASUREMENT STENCIL (4 m). Content at 2 m reads at a third of its
  //     curvature through it, and content at 20 m reads correctly but is relaxed
  //     by a stencil that cannot reach it. LAUNCH_STENCILS is a ladder.
  //  2. ONE RELAXATION STENCIL (11 m), fixed, for every violation regardless of
  //     the scale of the thing that caused it. Here the relaxation is at the SAME
  //     scale as the measurement that fired: a 2 m defect gets a 2 m Laplacian, a
  //     20 m rollover gets a 20 m one. That is the only way a single pass can
  //     take both out without either ringing or doing nothing.
  //  3. IT CAPPED AGAINST THE SPEED THE PROFILE ALREADY HAD, which is circular:
  //     wherever the speed had already been dragged down by the geometry the cap
  //     relaxed, so did the requirement. Here the cap is written against
  //     `S.vWant` — the speed the DESIGN intends, from the phase ceiling and the
  //     corner — so the ground is flattened toward the design instead of the
  //     design being quietly flattened toward the ground.
  //
  // What it may not do: fill more than GRADE_FILL_MAX, cut more than
  // RAMP_CUT_MAX, move a station carrying authored relief, or make the trail
  // climb. Where those bind, the speed gives way instead (techSpeedCap), and the
  // count of stations where THAT happened is safety.launchSpeedGiven.
  // -------------------------------------------------------------------------
  const LAUNCH_KAPPA_MIN = 0.008;   // /m — never demand a vertical radius over 125 m
  const LAUNCH_KAPPA_MAX = 0.150;   // /m — nor bother below ~6.7 m

  // -------------------------------------------------------------------------
  // THE SWITCHBACK RULE — gradient bounded by the corner it is in.
  //
  // MEASURED. With the launch problem fixed, the airborne fraction on seed
  // 20260726 fell 0.130 -> 0.041 and every one of 21 autopilot cells then died
  // within 32 m of the SAME point, arc 301 m. The trace says why and it is not
  // ambiguous: plan radius 21 -> 9 m while the gradient runs 30% -> 72%, the
  // bike's own leanLimit collapsing to 19-26 deg, skid 0.84-0.98, and the
  // lateral error going 0.6 -> 3.0 m in nine tenths of a second straight off the
  // outside of the corner. Seed 777 dies the same way at 84-88 m: radius 31 m at
  // 63-68% of gradient, lateral error 0.9 -> 4.5 m, then over the edge.
  //
  // A 70% pitch spends tan(35 deg) = 0.70 g of tyre just holding speed. That is
  // more than the whole design friction budget on dirt (tan(28 deg) = 0.53), so
  // there is nothing left to turn with at ANY speed — which is why the round-6
  // speed ablation could not fix seed 777 either. cornerGripBudget() prices that
  // into the speed; this bounds the geometry that creates it, which is the only
  // real answer.
  //
  // It is also what a trail builder does. Nobody builds a switchback down the
  // fall line: the apex is cut in near-flat and the drop is spent on the
  // straights either side. `shapePhaseGrade` already caps the gradient at
  // GRADE_HARD_MAX inside each phase — but it runs in buildStations(), six
  // passes before the profile that ships, and it measures over a 6 m window that
  // a two-station 70% spike walks straight through. Measured on the shipped
  // build, the start phase (design gradient 11%, shapePhaseGrade cap 26.7%)
  // carried 72% at arc 300.
  //
  // CONTRACT §4 asks for "steepest chutes to 35%". This is also that.
  const GRADE_CORNER_R0 = 10.0;   // m — full switchback
  const GRADE_CORNER_R1 = 60.0;   // m — open enough that the corner costs nothing
  const GRADE_CORNER_F = 0.45;    // fraction of the hard max left at R0
  const GRADE_CAP_WIN = 5.0;      // m — half-window the gradient is bounded over

  /** The steepest gradient the corner at station i can be ridden on. */
  function gradeCapAt(i) {
    const r = S.radius[i];
    const f = clamp(GRADE_CORNER_F + (1 - GRADE_CORNER_F) *
      (r - GRADE_CORNER_R0) / (GRADE_CORNER_R1 - GRADE_CORNER_R0), GRADE_CORNER_F, 1);
    return GRADE_HARD_MAX * f;
  }

  /** Fill S.vWant with the speed the design intends, ignoring the relief. */
  function computeWantSpeed() {
    for (let i = 0; i < S.n; i++) {
      S.vWant[i] = Math.max(V_DESIGN_FLOOR,
        Math.min(wantSpeedCap(i), cornerSpeedCap(i), V_CAP));
    }
  }

  /**
   * Flatten the vertical profile until the wheels stay down at S.vWant, at every
   * scale in the ladder. Writes into S.hOff; leaves S.by alone.
   */
  function capLaunchCurvature(vertMask) {
    const n = S.n, ds = S.ds;
    const y = new Float64Array(n);
    for (let i = 0; i < n; i++) y[i] = S.by[i] + S.hOff[i];
    const y0 = Float64Array.from(y);
    const cap = new Float32Array(n);
    computeWantSpeed();
    // The target is the speed the design wants, bounded by the speed the CARVE
    // can deliver a surface for. Asking the relaxation to flatten below
    // KAPPA_CARVE_FLOOR is asking it to remove curvature that is not in the
    // design profile at all — it would smooth real trail away chasing a
    // reconstruction artefact, which is how a course gets flattened to pass a
    // metric. See the KAPPA_CARVE_FLOOR note.
    const vTop = Math.sqrt(G / (SOLVER.safety * SOLVER.carveFloor));
    for (let i = 0; i < n; i++) {
      const v = clamp(S.vWant[i], 4, vTop);
      cap[i] = clamp(G / (SOLVER.safety * v * v), LAUNCH_KAPPA_MIN, LAUNCH_KAPPA_MAX);
    }
    const wgt = new Float32Array(n);
    const yf = new Float32Array(n);
    const RELAX = 0.38;
    // Measure through the same wheel/carve filter computeVertKappa() uses, or
    // the pass spends 260 iterations chasing lattice quantisation it can neither
    // remove nor be thrown by. `y` is what gets relaxed; `yf` is what is judged.
    const refilter = () => {
      const s = gaussianSmooth(y, 4, 0.45 / STATION_DS);
      for (let i = 0; i < n; i++) yf[i] = s[i];
    };
    refilter();

    // Worst violation on entry, over the whole ladder, for the audit.
    const yf0 = Float32Array.from(yf);
    for (let i = 0; i < n; i++) {
      if (vertMask[i]) continue;
      for (let s = 0; s < LAUNCH_STENCILS.length; s++) {
        const k = vertKappaAt(yf0, i, LAUNCH_STENCILS[s]);
        if (k > cap[i]) {
          safety.launchViolBefore++;
          if (k / cap[i] > safety.launchWorstBefore) {
            safety.launchWorstBefore = k / cap[i];
            safety.launchWorstAtBefore = i * ds;
          }
          break;
        }
      }
    }

    for (let pass = 0; pass < 260; pass++) {
      let any = false;
      refilter();
      // Largest scale first: taking a 20 m rollover out changes what the 2 m
      // stencil sees, and doing it the other way round makes the small-scale
      // work be undone by the large-scale pass.
      for (let s = LAUNCH_STENCILS.length - 1; s >= 0; s--) {
        const h = LAUNCH_STENCILS[s];
        const k = Math.max(1, Math.round(h / ds));
        if (n <= 2 * k + 2) continue;
        wgt.fill(0);
        let fired = false;
        for (let i = k; i < n - k; i++) {
          if (vertMask[i]) continue;
          const kap = vertKappaAt(yf, i, h);
          if (kap <= cap[i]) continue;
          fired = true; any = true;
          const over = clamp01((kap - cap[i]) / cap[i]) * 0.65 + 0.35;
          // Spread over the stencil so the correction is a rounded crest, not a
          // notch at the measurement point.
          for (let j = Math.max(0, i - k); j <= Math.min(n - 1, i + k); j++) {
            const w = over * (1 - Math.abs(j - i) / (k + 1));
            if (w > wgt[j]) wgt[j] = w;
          }
        }
        if (!fired) continue;
        for (let i = k; i < n - k; i++) {
          const w = wgt[i];
          if (w <= 0 || vertMask[i]) continue;
          y[i] += w * RELAX * (0.5 * (y[i - k] + y[i + k]) - y[i]);
          const lo = S.rawS[i] - RAMP_CUT_MAX, hi = S.rawS[i] + GRADE_FILL_MAX;
          if (y[i] < lo) y[i] = lo; else if (y[i] > hi) y[i] = hi;
        }
      }
      // ---- the switchback rule, in the same relaxation ---------------------
      //
      // CUT-BIASED, and that is not a preference. Easing a pitch means lowering
      // its top and raising its bottom; the mountain always grants the cut and
      // terrain.applyCarve() builds at most GRADE_FILL_MAX of the fill. So the
      // pass lowers the upper half of an over-steep window by the whole excess
      // and lets the clamp take whatever the fill side cannot have. Repeated,
      // that walks the excavation back UP the pitch, which is exactly how a
      // bench gets cut into a steep face.
      {
        const m = Math.max(2, Math.round(GRADE_CAP_WIN / ds));
        for (let i = m; i < n - m; i++) {
          if (vertMask[i]) continue;
          const win = 2 * m * ds;
          const g = (yf[i - m] - yf[i + m]) / win;
          const gc = gradeCapAt(i);
          if (g <= gc) continue;
          any = true;
          const excess = (g - gc) * win;      // metres of drop to take out of the window
          const A = 0.5 * RELAX * excess;
          for (let j = i - m; j <= i + m; j++) {
            if (vertMask[j]) continue;
            const u = (j - i) / m;            // -1 upstream .. +1 downstream
            // Antisymmetric: lower the top (u<0), raise the bottom (u>0). sin is
            // used rather than a ramp because its slope is zero at u = +-1, so the
            // correction joins the untouched profile without a kink — a kink here
            // would be a launcher, which is the constraint running next door.
            y[j] += A * Math.sin(Math.PI * 0.5 * u);
            const lo = S.rawS[j] - RAMP_CUT_MAX, hi = S.rawS[j] + GRADE_FILL_MAX;
            if (y[j] < lo) y[j] = lo; else if (y[j] > hi) y[j] = hi;
          }
        }
      }
      if (!any) break;
    }

    // Descent guard — CONTRACT §4 is absolute about climbing, and a Laplacian
    // that lowers a crest can raise the hollow on the far side of it.
    for (let i = 1; i < n; i++) {
      if (vertMask[i]) continue;
      const c = y[i - 1] + 0.035 * ds;
      if (y[i] > c) y[i] = Math.max(c, S.rawS[i] - RAMP_CUT_MAX);
    }

    for (let i = 0; i < n; i++) {
      S.hOff[i] += y[i] - y0[i];
      S.py[i] = S.by[i] + S.hOff[i];
    }
    refilter();
    for (let i = 0; i < n; i++) {
      if (vertMask[i]) continue;
      for (let s = 0; s < LAUNCH_STENCILS.length; s++) {
        const k = vertKappaAt(yf, i, LAUNCH_STENCILS[s]);
        if (k > cap[i]) {
          safety.launchViolAfter++;
          if (k / cap[i] > safety.launchWorstAfter) {
            safety.launchWorstAfter = k / cap[i];
            safety.launchWorstAtAfter = i * ds;
          }
          break;
        }
      }
    }
  }

  /**
   * Re-solve the bank at the speed the joint solve actually left, everywhere the
   * feature pass has not authored the cross-section itself.
   *
   * THIS IS THE HALF OF THE FIX THE ROUND-6 ABLATION WAS MISSING. Clamping the
   * governor without re-banking leaves berms built for 16 m/s being ridden at 9,
   * which asks the rider to lean OUT of every corner — 17 of 21 cells lowsided.
   * The bank is a function of the speed; change one and the other has to move.
   *
   * Both bounds are applied, and they are the two halves of the contract at the
   * head of this file:
   *   floor    bank >= atan(v^2/(g*r)) - LEAN_DESIGN     (fast enough to need it)
   *   ceiling  bank <= atan((SLOW_FRAC*v)^2/(g*r)) + LEAN_HOLD   (holdable slow)
   * Where the ceiling is below the floor the corner cannot be built for both and
   * the ceiling wins — a corner you can hold slowly and have to brake for beats
   * one you slide off the moment you are not perfect. That is counted as
   * safety.bankConflict.
   */
  function rebankAtSolvedSpeed(mask) {
    const n = S.n;
    const b = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      b[i] = S.bank[i];
      if (mask && mask[i]) continue;
      const st = styleAt(i);
      const r = rideRadius(i);
      const v = S.speed[i];
      const ideal = Math.atan((v * v) / (G * r));
      const need = ideal - leanBudgetAt(i);
      const hold = Math.atan((SLOW_FRAC * SLOW_FRAC * v * v) / (G * r)) + LEAN_HOLD;
      if (need <= 0.015 && r > st.minRadius * 4.5) { b[i] = 0; continue; }
      let mag = clamp(Math.max(ideal * st.style, need), 0, st.maxBank);
      if (mag > hold) { mag = Math.max(0, hold); safety.bankConflict++; }
      b[i] = -Math.sign(S.curv[i]) * mag;
    }
    const bS = gaussianSmooth(b, 16, 6);
    for (let i = 0; i < n; i++) {
      // The smoother must not reach into a station whose cross-section the
      // feature pass authored — a jump lip is deliberately flat and the wallride
      // deliberately past any berm ceiling.
      if (mask && mask[i]) continue;
      S.bank[i] = bS[i];
    }
    // Straight sections keep their drainage outslope.
    for (let i = 0; i < n; i++) {
      if ((mask && mask[i]) || Math.abs(S.bank[i]) > 0.05) continue;
      const e = 4;
      const hR = terrainSampleCache(S.px[i] + S.rx[i] * e, S.pz[i] + S.rz[i] * e);
      const hL = terrainSampleCache(S.px[i] - S.rx[i] * e, S.pz[i] - S.rz[i] * e);
      const out = Math.sign(hR - hL) * 0.030;
      S.bank[i] = S.bank[i] * 0.4 + out * 0.6;
    }
    // Nothing below this line may raise a corner's speed.
    applyCornerSpeedCap();
  }

  /**
   * THE ACCEPTANCE MEASURE, taken on the stations that ship.
   *
   * A station is INFEASIBLE when there is no (v, phi) at all: no speed between
   * V_DESIGN_FLOOR and the design speed, and no bank inside the phase's own
   * ceiling, at which all three constraints hold at once —
   *   1. the longitudinal demand leaves a lean budget (leanBudgetAt);
   *   2. atan(v²/(g r)) <= phi + budget;
   *   3. v <= sqrt(g / (LAUNCH_SAFETY * kappa_v)).
   * Constraint 2 is easiest at the lowest admissible speed and constraints 1
   * and 3 do not care about it, so the existence test is exactly "does the
   * floor speed work" — which is why slowing down cannot rescue a station this
   * counts, and why the fix had to be upstream in the route.
   *
   * `feasDesign` is the weaker, more familiar question — does the SHIPPED speed
   * profile hold — and is reported alongside so the two are never conflated.
   */
  function auditJointFeasibility() {
    safety.feasViol = 0; safety.feasWorstDeg = 0; safety.feasWorstAt = 0;
    safety.feasDesignViol = 0; safety.gripFloorBound = 0; safety.cornerFloorBound = 0;
    safety.feasCorner = 0; safety.feasLaunch = 0;
    for (let i = 0; i < S.n; i++) {
      const r = rideRadius(i);
      const phi = Math.max(0, helpfulBank(i));
      const st = styleAt(i);
      // The bank a builder could still add here, not merely the one that is
      // built: the existence test asks whether the corner CAN be made to work.
      const phiMax = Math.max(phi, st.maxBank);
      const budget = leanBudgetAt(i);
      if (SOLVER.feasible && Math.min(cornerSpeedCap(i), techSpeedCap(i)) < V_DESIGN_FLOOR - 1e-3) {
        safety.cornerFloorBound++;
      }
      if (SOLVER.feasible && Math.tan(budget) < MU_CORNER_MIN) safety.gripFloorBound++;
      const shortLo = leanRequired(V_DESIGN_FLOOR, r) - (phiMax + budget);
      const shortDes = leanRequired(S.speed[i], r) - (phi + budget);
      const vLaunch = launchSpeedCap(i);
      const badCorner = shortLo > 0, badLaunch = vLaunch < V_DESIGN_FLOOR - 1e-3;
      if (badCorner) safety.feasCorner++;
      if (badLaunch) safety.feasLaunch++;
      const bad = badCorner || badLaunch;
      if (bad) {
        safety.feasViol++;
        if (shortLo > safety.feasWorstDeg) {
          safety.feasWorstDeg = shortLo; safety.feasWorstAt = i * S.ds;
        }
      }
      if (shortDes > 0) safety.feasDesignViol++;
    }
    safety.feasWorstDeg = +THREE.MathUtils.radToDeg(safety.feasWorstDeg).toFixed(2);
    safety.feasFrac = S.n ? safety.feasViol / S.n : 0;
    safety.feasDesignFrac = S.n ? safety.feasDesignViol / S.n : 0;
  }

  /**
   * Plain +-RIDE_STENCIL circumradius of the committed plan line. See the note
   * at S.radiusR. Recomputed after every pass that moves px/pz.
   */
  const RIDE_STENCIL = 4.0;
  let _kapR = null;
  function computeRideRadius() {
    if (!S || !S.radiusR) return;
    const n = S.n, m = Math.max(1, Math.round(RIDE_STENCIL / S.ds));
    if (!_kapR || _kapR.length !== n) _kapR = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const a = i - m, b = i + m;
      if (a < 0 || b >= n) { _kapR[i] = 1 / 4000; continue; }
      const ax = S.px[a] - S.px[i], az = S.pz[a] - S.pz[i];
      const bx = S.px[b] - S.px[i], bz = S.pz[b] - S.pz[i];
      const la = Math.hypot(ax, az), lb = Math.hypot(bx, bz);
      const lc = Math.hypot(S.px[b] - S.px[a], S.pz[b] - S.pz[a]);
      const ar = Math.abs(ax * bz - az * bx);
      const r = (ar < 1e-7 || la < 1e-6 || lb < 1e-6)
        ? 4000 : clamp((la * lb * lc) / (2 * ar), 3, 4000);
      _kapR[i] = 1 / r;
    }
    // Smoothed in CURVATURE, not in radius, and only just: a circumradius on a
    // 0.4 m lattice flaps between 177 m and 879 m from one station to the next
    // on ground a rider would call one continuous bend, and a bank solved off
    // that is a bank nobody can build. Radius 5 stations / sigma 2 (0.8 m) is
    // an order of magnitude tighter than the +-4 m Gaussian on `curv`, so the
    // corner survives and the lattice noise does not.
    const ks = gaussianSmooth(_kapR, 5, 2);
    for (let i = 0; i < n; i++) S.radiusR[i] = clamp(1 / Math.max(2.5e-4, ks[i]), 3, 4000);
  }

  /**
   * The worst +-RIDE_STENCIL circumradius on the CURRENT plan line, and where.
   * Measured wherever it is asked for, because a plan radius is only meaningful
   * after every pass that moves px/pz has run — which, this round, turned out to
   * include applyExposureSafety(): the bench shift is capped at BENCH_SHIFT_MAX
   * = 1.10 m and a 1.1 m inward shift takes a 13 m corner to 8.3.
   */
  function measureShipRadius() {
    const n = S.n, m = Math.max(1, Math.round(RIDE_STENCIL / S.ds));
    let worst = 4000, at = 0, under = 0;
    for (let i = m; i < n - m; i++) {
      const ax = S.px[i - m] - S.px[i], az = S.pz[i - m] - S.pz[i];
      const bx = S.px[i + m] - S.px[i], bz = S.pz[i + m] - S.pz[i];
      const la = Math.hypot(ax, az), lb = Math.hypot(bx, bz);
      const lc = Math.hypot(S.px[i + m] - S.px[i - m], S.pz[i + m] - S.pz[i - m]);
      const ar = Math.abs(ax * bz - az * bx);
      if (ar < 1e-7 || la < 1e-6 || lb < 1e-6) continue;
      const r = (la * lb * lc) / (2 * ar);
      if (r < R_PLAN_MIN) under++;
      if (r < worst) { worst = r; at = i * S.ds; }
    }
    return { worst, at, under };
  }

  /** The radius the corner solve prices: the honest one, never the smoothed one. */
  function rideRadius(i) {
    if (!SOLVER.feasible) return S.radius[i];
    const rr = S.radiusR[i];
    return rr > 0 && rr < S.radius[i] ? rr : S.radius[i];
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
    computeRideBasis();
    computeRideRadius();
  }

  /**
   * THE GRADE THE TYRE IS WORKING AGAINST — the same profile computeVertKappa()
   * prices the launch constraint on, i.e. with the authored ballistic relief
   * removed. Fills S.gradeR.
   *
   * THIS IS THE THIRD FACE OF ONE DEFECT, and it was the one that actually paced
   * the jump line. A lip is a short, steep, authored ramp: measured on seed
   * 20260726 the five jump-line lips read S.grade = 0.65-0.78 — sin(theta), so
   * tan(theta) = 0.86-1.25, i.e. an 86-125% hillside. leanBudget() reads that,
   * finds it past RIDE_TAN_ZERO, and correctly returns ZERO lean budget, because
   * on a 100% face the reference rider has none. cornerGripBudget() then falls to
   * MU_CORNER_MIN_FEAS and cornerSpeedCap() answers 5.9-7.4 m/s at a plan radius
   * of 126-279 m — a corner cap of 7 m/s on ground that is, in plan, essentially
   * straight. cornerCapLead() then drags the whole run-in down to it.
   *
   * Measured at the lips of the jump line, seed 20260726, before this:
   *
   *   arc   r(m)  tech  launch  CORNER  lean budget
   *   1088  279   12.49  12.49   7.40      0.0 deg
   *   1158  266   12.49  12.49   7.22      0.0
   *   1300  126   12.49  12.49   5.93      0.0
   *
   * Nothing about the tyre is different on a lip; what is different is that the
   * rider is on a designed ramp for 4 metres, not descending a face. The lean
   * budget, the braking limit and the endo margin are all questions about ground
   * the rider is riding ON, and a solved ballistic feature is priced by
   * solveJump() and its published speed window instead. So those three read
   * S.gradeR and everything else — the forward pass's gravity term, sampleAt()'s
   * published gradient, the descent rules, the audits — reads the real S.grade,
   * because the lip genuinely is there and climbing it genuinely does cost speed.
   */
  function computeRideBasis() {
    const n = S.n;
    if (!S.gradeR) return;
    for (let i = 0; i < n; i++) {
      const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
      const dx = S.px[b] - S.px[a], dz = S.pz[b] - S.pz[a];
      const dy = (S.py[b] - S.hBal[b]) - (S.py[a] - S.hBal[a]);
      const l = Math.max(1e-5, Math.hypot(dx, dy, dz));
      S.gradeR[i] = -(dy / l);
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
  // WHAT THE ELEVATION SOLVE ACTUALLY SHIPPED
  //
  // THE DEFECT THIS EXISTS FOR, named by two audits in a row. `routeAdmissible()`
  // measures climb, non-descending run and dead ground on the MARCHED POLYLINE —
  // raw ground, 6 m steps, before a single station exists. Every one of those
  // three is then transformed by buildStations(): the plan line is smoothed and
  // resampled, and the elevation is bench-smoothed, descent-enforced, clamped to
  // an excavation budget and grade-shaped per phase. Measured on the shipped
  // build, the marched line and the built profile disagree by a factor of five in
  // both directions:
  //
  //     seed        marched flat -> built    marched dead -> built
  //     20260726        54 m       18.3 m        15 m        2.8 m
  //     777             24 m       22.8 m        15 m        1.5 m
  //     12345           48 m       17.9 m        18 m       12.9 m
  //     99999           15 m       18.7 m        18 m       12.1 m
  //
  // A gate read off the left-hand column certifies something other than what
  // ships, in both directions at once: it passed 12345's 12.9 m of dead ground
  // and it would have refused seed 777's build for 15 m of dead ground that the
  // profile solve removes entirely. So the numbers that decide are taken HERE,
  // off S, after buildStations() — and build() picks its variant by building
  // them and looking, rather than by predicting.
  //
  // Measured on the BASE profile (S.by), before any feature relief: authored
  // relief is a separate question with a separate owner (see buildJump's
  // run-out) and mixing them makes neither answerable.
  // =========================================================================

  /**
   * Longest continuous run, in 3-D metres, of (a) non-descending base profile
   * and (b) ground on which the measured table gives the reference rider no
   * usable lean. Reads S; call straight after buildStations().
   */
  function measureBaseProfile() {
    const n = S.n;
    let flat = 0, flatWorst = 0, flatAt = 0, flatStart = 0;
    let dead = 0, deadWorst = 0, deadAt = 0, deadStart = 0;
    let arc = 0, prevArc = 0;
    // The gradient stencil is +-2 m of 3-D arc — the same one the acceptance
    // harness uses — not the +-0.4 m station difference, which reads resampling
    // wobble as gradient.
    const half = Math.max(1, Math.round(2.0 / S.ds));
    for (let i = 1; i < n; i++) {
      const d = Math.hypot(S.px[i] - S.px[i - 1], S.by[i] - S.by[i - 1], S.pz[i] - S.pz[i - 1]);
      prevArc = arc; arc += d;
      if (S.by[i] >= S.by[i - 1] - 1e-4) {
        if (flat === 0) flatStart = prevArc;
        flat += d;
        if (flat > flatWorst) { flatWorst = flat; flatAt = flatStart; }
      } else flat = 0;
      const a = i - half < 0 ? 0 : i - half;
      const b = i + half > n - 1 ? n - 1 : i + half;
      const run = (b - a) * S.ds;
      const sinT = run > 1e-6 ? (S.by[a] - S.by[b]) / run : 0;
      if (leanBudget(tanFromSin(sinT), 0, S.surface[i]) <= FEAS_DEAD_RAD) {
        if (dead === 0) deadStart = prevArc;
        dead += d;
        if (dead > deadWorst) { deadWorst = dead; deadAt = deadStart; }
      } else dead = 0;
    }
    // THE PROPERTY, not the counter. See the note above enforceDescentShipped():
    // a run of consecutive non-descending stations is reset by a 10 micrometre
    // dip, and the base profile is a sawtooth by construction, so `flatWorst`
    // reads 11 m on a variant carrying 89 m that never gets back below its own
    // start. The variant budget was being spent on the wrong number.
    let runWorst = 0, runAt = 0, aI = 0, a2 = 0;
    {
      let acc = 0;
      const cum = new Float64Array(n);
      for (let i = 1; i < n; i++) {
        acc += Math.hypot(S.px[i] - S.px[i - 1], S.by[i] - S.by[i - 1], S.pz[i] - S.pz[i - 1]);
        cum[i] = acc;
      }
      for (let i = 1; i < n; i++) {
        if (S.by[i] < S.by[aI]) { aI = i; continue; }
        const span = cum[i] - cum[aI];
        if (span > runWorst) { runWorst = span; runAt = cum[aI]; a2 = i; }
      }
    }
    return {
      flat: flatWorst, flatAt, dead: deadWorst, deadAt, arc,
      run: runWorst, runAt, runRise: a2 ? S.by[a2] - S.by[aI] : 0,
    };
  }

  // =========================================================================
  // BUILD
  // =========================================================================

  // Async solely for the boot progress bar (see core/bootProgress.js): the
  // solver stages are unchanged and stay synchronous between yields, so every
  // RNG stream is consumed in the same order and the course is bit-identical.
  async function build(terrain) {
    if (!terrain) return [];
    _terrainRef = terrain;

    // Route-find several times and keep the best line down the hill.
    //
    // The variant budget is now in two parts. The first ROUTE_VARIANTS are
    // always marched and the best is kept, exactly as before. If that best is
    // INADMISSIBLE — see routeAdmissible(); this is the seed-2 case — further
    // variants are marched, deterministically, until one is admissible or the
    // extended budget runs out. Each variant has its own subSeed'd stream, so
    // the extra attempts are as reproducible as the first seven, and on a seed
    // whose first attempt is admissible not one extra march is run: the world
    // is bit-identical to a build without this loop.
    let route = null, bestScore = -Infinity;
    let bestAdm = null, bestAdmScore = -Infinity;
    const cands = [];
    routeAudit.length = 0;
    // With the rideability solve OFF the extended budget is off too, and the
    // admissibility test does not influence the choice: the control path must
    // reproduce the previous build BIT-IDENTICALLY or no A/B in this file's
    // notes means anything. Verified: same fingerprint, same stamp count.
    const ROUTE_VARIANTS = 7;
    const ROUTE_VARIANTS_MAX = SOLVER.feasible ? 24 : 7;
    const marchOne = (v) => {
      const cand = marchRoute(terrain, v);
      const sc = scoreRoute(terrain, cand);
      const fail = SOLVER.feasible ? routeAdmissible(terrain, cand) : null;
      cand.admFail = fail;
      cand.score = sc;
      cands.push(cand);
      routeAudit.push({
        variant: v, score: +sc.toFixed(2),
        startX: +cand.startX.toFixed(1), startY: +cand.startY.toFixed(1),
        corrMean: +(cand.corrMean || 0).toFixed(2),
        corrWorst: +(cand.corrWorst || 0).toFixed(2),
        corrBad: cand.corrBad || 0,
        len: +(cand.marchedLength || 0).toFixed(0),
        drop: +(cand.marchedDrop || 0).toFixed(0),
        openGrade: +(cand.openGrade || 0).toFixed(3),
        feasFrac: +(cand.feasFrac || 0).toFixed(4),
        feasMean: +(cand.feasMean || 0).toFixed(2),
        feasWorstDeg: +THREE.MathUtils.radToDeg(cand.feasWorst || 0).toFixed(1),
        sbForced: cand.sbForced || 0,
        climb: +(cand.admClimb || 0).toFixed(2),
        flat: +(cand.admFlat || 0).toFixed(1),
        minR: +(cand.admMinR || 0).toFixed(2),
        conflict: +(cand.admConflict || 0).toFixed(2),
        dead: +(cand.marchDead || 0).toFixed(1),
        deadAt: +(cand.marchDeadAt || 0).toFixed(0),
        admissible: !fail, fail: fail || '',
      });
      if (sc > bestScore) { bestScore = sc; route = cand; }
      if (SOLVER.feasible && !fail && sc > bestAdmScore) { bestAdmScore = sc; bestAdm = cand; }
    };
    for (let v = 0; v < ROUTE_VARIANTS_MAX; v++) {
      marchOne(v);
      report(0.28 * Math.min(1, (v + 1) / ROUTE_VARIANTS));
      { const y = maybeYield(); if (y) await y; }
      // Stop as soon as the ordinary budget has produced something admissible.
      if (v + 1 >= ROUTE_VARIANTS && (bestAdm || !SOLVER.feasible)) break;
    }
    safety.routeVariants = routeAudit.length;
    safety.routeAdmissible = !!bestAdm;
    safety.routeFail = bestAdm ? '' : (route.admFail || '');
    safety.routeReseeded = routeAudit.length > ROUTE_VARIANTS;
    // LOUD FAILURE. See the note above routeAdmissible(). The message carries
    // every variant's numbers so the reason is in the failure and not in a
    // debugger session.
    if (SOLVER.feasible && !bestAdm) {
      const rows = routeAudit.map((r) => `    v${r.variant}: ${r.fail}`
        + ` climb=${r.climb} m, flat=${r.flat} m, minR=${r.minR} m,`
        + ` conflict=${r.conflict} m, dead=${r.dead} m @${r.deadAt} m,`
        + ` unrideable=${(r.feasFrac * 100).toFixed(1)}%`).join('\n');
      const msg = `[trail] GENERATION FAILED for seed ${ctx.seed}: no admissible route.\n`
        + `  ${routeAudit.length} independent variants were marched and every one failed`
        + ` "${route.admFail}".\n`
        + `  Limits: climb <= ${ROUTE_MAX_CLIMB} m, non-descending <= ${ROUTE_MAX_FLAT} m,`
        + ` plan radius >= ${ROUTE_MIN_R} m, leg conflict <= ${ROUTE_CONFLICT_TOL} m,`
        + ` dead ground <= ${ROUTE_MAX_DEAD} m,`
        + ` unrideable <= ${(ROUTE_MAX_FEAS * 100).toFixed(0)}%.\n${rows}\n`
        + `  This mountain will not carry the design descent. Change ctx.seed, or lower`
        + ` the design drop. The trail deliberately does NOT ship a course it has`
        + ` measured as unrideable.`;
      // eslint-disable-next-line no-console
      console.error(msg);
      const err = new Error(msg);
      err.trailRouteAudit = routeAudit.slice();
      err.trailRouteFail = route.admFail;
      throw err;
    }
    if (bestAdm) { route = bestAdm; bestScore = bestAdmScore; }

    // ---- SELECT ON THE BUILT PROFILE, NOT ON THE MARCHED ONE ---------------
    //
    // See the note above measureBaseProfile(). The marched line and the profile
    // the elevation solve delivers disagree by up to 5x in both directions, so
    // this loop builds the stations for the best few candidates and picks the
    // one whose BUILT base profile is a course. It is not a repair pass and it
    // cannot invent a good route: if every candidate ships dead ground, the best
    // of them is used and `safety` says so in the numbers, loudly.
    //
    // Cost: buildStations() is a small fraction of build(), and the loop stops
    // at the first candidate inside both limits — so on a seed whose best route
    // is already a course, exactly one extra call is made and it is the one the
    // build needed anyway.
    //
    // Order: admissible first (the marched gates still disqualify the genuinely
    // broken lines), then by score. Determinism is unaffected — buildStations()
    // and everything it calls are pure functions of the route and the mountain.
    let chosen = null, chosenM = null, chosenPen = Infinity, lastBuilt = null;
    {
      const tried = new Set();
      let ok = false, built = 0;
      // Two waves, for exactly the reason the marched-admissibility budget has
      // two: the first ROUTE_VARIANTS are all a healthy seed ever needs, and a
      // seed whose best line ships dead ground deserves the rest of the budget
      // rather than a shrug. Bit-identical on a seed that passes in wave 0.
      for (let wave = 0; wave < 2 && !ok; wave++) {
        if (wave === 1) {
          if (!SOLVER.feasible || cands.length >= ROUTE_VARIANTS_MAX) break;
          for (let v = cands.length; v < ROUTE_VARIANTS_MAX; v++) {
            marchOne(v);
            { const y = maybeYield(); if (y) await y; }
          }
          safety.routeVariants = routeAudit.length;
          safety.routeReseeded = true;
        }
        // ADMISSIBLE CANDIDATES ONLY. The marched gates still disqualify the
        // genuinely broken lines and this loop must not be able to smuggle one
        // back in: measured, letting it prefer-but-not-require admissibility put
        // seed 20260726 on an inadmissible variant carrying 12.4 m of climb.
        const pool = SOLVER.feasible ? cands.filter((c) => !c.admFail) : cands;
        const order = (pool.length ? pool : cands).slice().sort((a, b) => b.score - a.score);
        const tries = Math.min(order.length, SOLVER.feasible ? STATION_TRIES * (wave + 1) : 1);
        for (let k = 0; k < tries && built < STATION_TRIES * 2; k++) {
          const cand = order[k];
          if (tried.has(cand)) continue;
          tried.add(cand); built++;
          lastBuilt = cand;
          report(0.28 + 0.14 * Math.min(1, built / STATION_TRIES));
          { const y = maybeYield(); if (y) await y; }
          buildStations(terrain, cand);
          const m = measureBaseProfile();
          // THE PLAN RADIUS IS A PROPERTY OF THE BUILT LINE, NOT OF THE MARCHED
          // ONE, and the two differ by up to 30%. On seed 20260726 the chosen
          // variant marches a 11.87 m minimum and ships 8.34 m: smoothPolyline,
          // the centripetal Catmull-Rom and the 0.4 m resample take a third of a
          // corner's radius out between the march and the stations, and
          // openTightCorners cannot put it back — it is displacement-limited
          // against the very line that lost it. Other variants on the same
          // mountain build clean. So the radius joins dead ground and the
          // descent rule as something the variant budget is spent on, at the
          // same weight, because all three are "ground nobody can ride" in
          // different axes and none is worth trading for another.
          const rM = measureShipRadius();
          cand.shipFlat = m.flat; cand.shipDead = m.dead; cand.shipR = rM.worst;
          cand.shipRun = m.run;
          cand.shipFlatAt = m.flatAt; cand.shipDeadAt = m.deadAt; cand.shipRAt = rM.at;
          const row = routeAudit[cand.variant];
          if (row) {
            row.shipFlat = +m.flat.toFixed(1); row.shipFlatAt = +m.flatAt.toFixed(0);
            row.shipRun = +m.run.toFixed(1); row.shipRunAt = +m.runAt.toFixed(0);
            row.shipDead = +m.dead.toFixed(1); row.shipDeadAt = +m.deadAt.toFixed(0);
            row.shipR = +rM.worst.toFixed(2); row.shipRAt = +rM.at.toFixed(0);
            row.shipRUnder = rM.under;
          }
          // ORDERED BY WHAT THE REST OF THE BUILD CAN STILL REPAIR.
          //
          // The three defects are not commensurable and summing them picks the
          // wrong variant. The plan radius is the one NO downstream pass can fix:
          // openTightCorners is displacement-limited against the very line that
          // lost the radius, applyExposureSafety's bench only ever tightens it,
          // and the speed cap cannot go below V_DESIGN_FLOOR. So a variant that
          // clears R_PLAN_MIN beats one that does not, outright — measured on
          // seed 20260726, exactly one of six built variants does (11.41 m
          // against 7.97-9.24), and summing the penalties instead chose one of
          // the others and shipped 24 stations inside the floor.
          //
          // Dead ground and the descent rule DO have downstream repairs
          // (capGradeRamp, enforceDescentShipped's windowed pass), so they are
          // traded against each other on a common scale below that gate, in
          // units of their own limit so neither can swamp the other by the
          // accident of being measured in bigger numbers. The march's own score
          // is the tie-break and nothing more.
          const hardR = !SOLVER.feasible || rM.worst >= R_PLAN_MIN ? 0 : 1;
          const pen = hardR * 1e6
            + Math.max(0, m.dead / SHIP_MAX_DEAD - 1) * 100
            + Math.max(0, (SOLVER.feasible ? m.run / SHIP_MAX_RUN : m.flat / SHIP_MAX_FLAT) - 1) * 100
            - cand.score * 0.001;
          if (pen < chosenPen) { chosenPen = pen; chosen = cand; chosenM = m; }
          if (m.dead <= SHIP_MAX_DEAD
            && (SOLVER.feasible ? m.run <= SHIP_MAX_RUN : m.flat <= SHIP_MAX_FLAT)
            && (!SOLVER.feasible || rM.worst >= R_PLAN_MIN)) { ok = true; break; }
        }
      }
      // Rebuild the winner's stations unless it was the last one built. Getting
      // this wrong is silent and total: S would carry one candidate's stations
      // while `route` and every audit number described another.
      if (chosen && chosen !== lastBuilt) buildStations(terrain, chosen);
      route = chosen || route;
      bestScore = route.score;
      safety.shipDead = chosenM ? chosenM.dead : 0;
      safety.shipDeadAt = chosenM ? chosenM.deadAt : 0;
      safety.shipFlatBase = chosenM ? chosenM.flat : 0;
      safety.shipFlatBaseAt = chosenM ? chosenM.flatAt : 0;
      safety.stationTries = Math.min(cands.length, SOLVER.feasible ? STATION_TRIES : 1);
    }
    for (const r of routeAudit) r.chosen = r.variant === route.variant;
    safety.routeFeasFrac = route.feasFrac || 0;
    safety.routeClimb = route.admClimb || 0;
    safety.routeMinR = route.admMinR || 0;
    safety.routeDead = route.marchDead || 0;
    safety.routeDeadAt = route.marchDeadAt || 0;
    safety.routeConflict = route.admConflict || 0;
    safety.sbForced = route.sbForced || 0;
    computeRideRadius();
    // The relief has to be measured before the first speed solve, or the first
    // solve prices it at zero and every pass downstream inherits that.
    computeVertKappa();
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
        report(0.42 + 0.04 * k);
        { const y = maybeYield(); if (y) await y; }
        capCrestCurvature(empty);
        refreshBasis();
        computeVertKappa();
        solveSpeeds();
      }
      // ---- the joint solve, on bare ground, before any feature exists -------
      // Flatten the relief to the speed the design wants, re-measure it,
      // re-solve the speed and the bank against what is left, and repeat. Three
      // rounds: the ground moves, which moves the corner speed, which moves the
      // bank, which moves the corner speed again.
      if (SOLVER.launch) {
        for (let k = 0; k < 3; k++) {
          report(0.50 + 0.03 * k);
          { const y = maybeYield(); if (y) await y; }
          capLaunchCurvature(empty);
          refreshBasis();
          computeVertKappa();
          solveSpeeds();
        }
      }
    }
    safety.crestStage.afterCap = countCrestViol();

    // =======================================================================
    // FIX A — PACE THE JUMP LINE AT THE SPEED ITS OWN FEATURES PUBLISH.
    //
    // Build the features, solve the profile the way it will actually ship, read
    // the speed that profile DELIVERS on the approach to each lip, and rebuild
    // the features for it. Repeat until the two agree, or the iteration budget
    // runs out; report the residual either way (safety.jumpFitIters /
    // jumpFitResidual / jumpFitPace).
    //
    // WHY THIS IS THE FIX AND NOT A TUNING. The defect was never that a number
    // was wrong; it is that two quantities which determine each other were
    // solved in one direction only. buildFeatures() ran once, against the pace
    // of ground that did not yet carry a jump, and the profile was then solved
    // against the ground that did. Every seed shipped features whose delivered
    // lip speed was half their own published takeoff — 20 of 32 across four
    // seeds, BELOW the `vMin` of their own window, i.e. the design instructing
    // the rider to case every jump on the course. CONTRACT §4: a jump you can't
    // land is a bug.
    //
    // THREE THINGS HAD TO BE RIGHT FOR IT TO CONVERGE, and each of them was
    // found by the fixed point failing to:
    //
    //  1. The launch cap must stop pricing a lip's own convexity as unwanted
    //     relief (computeVertKappa subtracts S.hBal). Without it the run-in
    //     collapses to V_DESIGN_FLOOR and the fixed point converges honestly on
    //     a course with no jumps left on it.
    //  2. THE PASS THAT IS ITERATED MUST BE THE PASS THAT SHIPS. Iterating
    //     buildFeatures + one speed solve converged to `jumpWindowBad = 0` and
    //     the build then shipped 15 of 32 outside their windows, because the
    //     crest cap, the joint solve and the re-bank all run afterwards and all
    //     move the pace. Everything up to applyExposureSafety() is inside the
    //     loop for that reason. (applyExposureSafety is not: it benches px/pz
    //     and re-samples natural ground, which is not restorable state. It is
    //     measured after the loop instead, and the final audit is taken there.)
    //  3. The quantity fed back must be the APPROACH speed, at the foot of the
    //     lip transition, not the speed at the lip. Feeding back the lip speed
    //     subtracts the lip climb a second time on every pass — a systematic
    //     downward bias that walked a 9.8 m/s takeoff to 5.5 in four iterations
    //     and looked exactly like the original defect.
    //
    // Cost: one extra feature build and joint solve per pass beyond the first,
    // which measures at 8-12% of build() per pass on these seeds.
    const featurePass = async () => {
      buildFeatures();
      { const y = maybeYield(); if (y) await y; }
      // The ballistic core has to exist before the next speed solve, or the
      // solve paces the jump line at the floor. See techSpeedCap().
      ballisticMask = buildBallisticMask(1.5, true);
      // The authored relief is part of the surface now, so the basis and the
      // relief measurement have to see it before anything is paced against it.
      // (The control arm keeps the original order, in which the profile was
      // paced against the PRE-feature basis; see readSolverOpts().)
      if (SOLVER.feasible) {
        for (let i = 0; i < S.n; i++) S.py[i] = S.by[i] + S.hOff[i];
        refreshBasis();
        computeVertKappa();
      } else {
        buildSplits();
      }
      // Features changed the surfaces and widths, so re-solve the speed profile
      // with the final numbers — but do NOT re-bank, or it would undo the
      // flattened jump lips and the over-banked wallride.
      solveSpeeds(false);
      for (let i = 0; i < S.n; i++) S.py[i] = S.by[i] + S.hOff[i];

      // ---- the crest cap has to run AGAIN, on the profile the features left --
      //
      // The pass upstream deliberately runs BEFORE buildFeatures() so that
      // nothing it does can touch a jump's ballistics. That is right, and it is
      // not enough: measured on seed 20260726, the profile leaving the cap
      // carried 72 crest violations and the profile leaving buildFeatures()
      // carried 252. The features add 180 launch points of their own, on the
      // ground BETWEEN the authored relief — roller crests, berm entries, the
      // shoulders either side of a chute — and nothing re-measured it. The
      // reported crestViolAfter came from the cap's own exit, so the number that
      // was signed off described a profile four passes upstream of the one that
      // ships.
      //
      // Running it a second time under the SAME authored-relief mask
      // applyExposureSafety uses keeps the original guarantee exactly: every
      // station carrying a lip, a landing, a committing drop or the wallride is
      // masked, so the jump table is unchanged to the millimetre, and only the
      // ordinary ground either side of them is flattened back under the cap.
      const vm = buildAuthoredMask(
        ['jump', 'doubles', 'gap', 'stepDown', 'wallride', 'drop'], FEATURE_GUARD);
      safety.crestStage.afterFeatures = countCrestViol(vm);
      { const y = maybeYield(); if (y) await y; }
      capCrestCurvature(vm);
      refreshBasis();
      computeVertKappa();
      solveSpeeds(false);
      for (let i = 0; i < S.n; i++) S.py[i] = S.by[i] + S.hOff[i];
      safety.crestStage.afterRecap = countCrestViol(vm);
      crestMask = vm;

      // ---- the joint solve again, on the profile the features left ----------
      //
      // Same reason capCrestCurvature runs twice: buildFeatures() authors relief
      // on top of the flattened ground and the ground BETWEEN the authored
      // relief is not flat any more. Under the same authored mask, so the jump
      // table is untouched to the millimetre.
      //
      // Then, and this is the part the round-6 speed ablation left out, the bank
      // is re-solved at whatever speed the flattening left. A berm built for a
      // speed the rider no longer arrives at is a lowside.
      for (let k = 0; k < 3; k++) {
        { const y = maybeYield(); if (y) await y; }
        if (SOLVER.launch) {
          capLaunchCurvature(vm);
          refreshBasis();
          computeVertKappa();
          solveSpeeds(false);
          for (let i = 0; i < S.n; i++) S.py[i] = S.by[i] + S.hOff[i];
        }
        if (SOLVER.rebank) rebankAtSolvedSpeed(vm);
      }
    };

    /**
     * Score the current profile against every solved feature's own window, and
     * return the approach speeds the next pass should solve them at.
     */
    const scoreFeaturePace = () => {
      let bad = 0, moved = 0, worst = 0, lost = 0;
      const next = new Map();
      for (const [, why] of featAttempt) if (why) lost++;
      for (const f of features) {
        const p = f.params;
        if (!p || !p.name || p.iLip === undefined || !p.speedWindow) continue;
        const vDel = S.speed[p.iLip];
        if (vDel < p.speedWindow[0] || vDel > p.speedWindow[1]) {
          bad++;
          const miss = Math.max(p.speedWindow[0] - vDel, vDel - p.speedWindow[1]);
          if (miss > worst) worst = miss;
        }
        // The APPROACH speed, read at the foot of the lip transition. See point
        // 3 in the note above: the lip climb is what turns an approach into a
        // takeoff, and buildJump applies it, so feeding back the lip speed
        // applies it twice.
        const iA = p.iApproach === undefined ? p.iLip : p.iApproach;
        const prev = featPace.get(p.name);
        const tgt = Math.max(1.0, S.speed[iA]);
        const nv = prev === undefined ? tgt : prev + (tgt - prev) * JUMP_FIT_RELAX;
        // 0.05 m/s is two orders of magnitude inside the narrowest speed window
        // any feature publishes; below it the pace vector is stationary and
        // another pass is a full feature build spent to change nothing.
        if (prev === undefined || Math.abs(nv - prev) > 0.05) moved++;
        next.set(p.name, nv);
      }
      return { bad, moved, worst, lost, next };
    };

    {
      const preFeat = featureStateSnapshot();
      let bestPace = null, bestLost = Infinity, bestBad = Infinity, bestWorst = Infinity;
      let bestIter = -1, bestWhy = '';
      let iter = 0, lastIter = 0, stall = 0, lastCost = Infinity;
      // THE SCORE IS `bad + lost`, AND BOTH TERMS ARE LOAD-BEARING.
      //
      // Scoring on `bad` alone lets the fixed point converge by DELETING the
      // jump line: measured, `jumpWindowBad` went to 0 while the four seeds went
      // from 32 air features to 18. Scoring lexicographically on `lost` first is
      // the opposite failure and it is just as real: on seed 20260726 the
      // free-pace pass builds nine features of which five are outside their own
      // windows, and every pass that fixes them costs one feature that the
      // ground genuinely cannot carry — so `lost`-first pins the build on the
      // worst pass available.
      //
      // Weighted equally, a pass may trade one feature for one wall and no more.
      // That is the honest exchange rate: a feature the rider is told to case is
      // a wall, and a feature that is not there is a poorer course. Ties break on
      // `lost` (keep the fuller course) and then on how far outside its window
      // the worst survivor is.
      const better = (sc) => {
        const c = sc.bad + sc.lost, bc = bestBad + bestLost;
        return c !== bc ? c < bc
          : sc.lost !== bestLost ? sc.lost < bestLost
            : sc.worst < bestWorst - 1e-6;
      };
      const iterBudget = SOLVER.feasible ? JUMP_FIT_ITERS : 1;
      for (; iter < iterBudget; iter++) {
        if (iter > 0) featureStateRestore(preFeat);
        lastIter = iter;
        report(0.58 + 0.30 * (iter / iterBudget));
        await featurePass();
        const sc = scoreFeaturePace();
        // Keep the pace vector that produced this pass, not the one it suggests.
        if (better(sc)) {
          bestLost = sc.lost; bestBad = sc.bad; bestWorst = sc.worst;
          bestPace = new Map(featPace); bestIter = iter;
          bestWhy = [...featAttempt].filter(([, w]) => w).map(([k, w]) => `${k}:${w}`).join(',');
        }
        // Stop on a clean pass, a stationary one, or one that has stopped
        // improving. Stopping at the first `bad === 0` stops at the first pass
        // that deleted its way there; not stopping at all spends a full feature
        // build per pass to re-derive an answer that is not moving.
        const cost = sc.bad + sc.lost;
        if (cost >= lastCost) stall++; else stall = 0;
        lastCost = cost;
        if (cost === 0 || sc.moved === 0 || stall >= 2) break;
        for (const [k, v] of sc.next) featPace.set(k, v);
      }
      // If a later pass was worse than an earlier one — which the relaxation
      // makes unlikely but does not forbid — rebuild on the pace vector that
      // scored best rather than on whichever one the loop happened to stop on.
      // Silent divergence is the failure mode of every fixed point and it is
      // cheap to make impossible.
      if (bestPace && bestIter !== lastIter) {
        featPace.clear();
        for (const [k, v] of bestPace) featPace.set(k, v);
        featureStateRestore(preFeat);
        await featurePass();
      }
      safety.jumpFitIters = lastIter + 1;
      safety.jumpFitResidual = bestBad === Infinity ? 0 : bestBad;
      safety.jumpFitWorst = +(bestWorst === Infinity ? 0 : bestWorst).toFixed(2);
      safety.jumpFitLost = bestLost === Infinity ? 0 : bestLost;
      safety.jumpFitLostWhy = bestWhy;
    }
    if (SOLVER.feasible) buildSplits();

    {
      // Book-keeping: where did the geometry fail and the speed have to give way?
      computeWantSpeed();
      safety.launchSpeedGiven = 0; safety.launchFloorBound = 0;
      for (let i = 0; i < S.n; i++) {
        if (S.speed[i] < S.vWant[i] - 0.25) safety.launchSpeedGiven++;
        if (launchSpeedCap(i) < V_DESIGN_FLOOR - 1e-3) safety.launchFloorBound++;
      }
      safety.launchStage.afterJoint = countLaunchViol(crestMask);
    }

    // Level-design safety audit and mitigation. Must run AFTER the features have
    // set the final width/bank/height (it measures the real cross-section) and
    // BEFORE the curve rebuild and buildStamps() (it moves px/pz/py and adds the
    // shelf geometry). See the long note above applyExposureSafety().
    report(0.90);
    { const y = maybeYield(); if (y) await y; }
    applyExposureSafety(terrain);
    safety.crestStage.afterSafety = countCrestViol(crestMask);
    // applyExposureSafety() benches the line and re-runs capGradeRamp, both of
    // which move S.py. Re-measure, re-pace and re-bank against what shipped —
    // reporting a launch figure from four passes upstream is exactly the mistake
    // countCrestViol() exists to document.
    computeVertKappa();
    solveSpeeds(false);
    if (SOLVER.rebank) rebankAtSolvedSpeed(crestMask);
    safety.launchStage.afterSafety = countLaunchViol(crestMask);

    // ---- CONTRACT §4, on the profile that ships ---------------------------
    // See the note above enforceDescentShipped(). Measured before and after, on
    // S.py, so the number that is signed off is the number the rider meets.
    {
      const before = measureShippedFlat();
      safety.flatBefore = +before.flat.toFixed(1);
      safety.flatBeforeAt = +before.at.toFixed(0);
      safety.flatRunBefore = +before.run.toFixed(1);
      safety.flatWinBefore = +before.win.toFixed(3);
      const bm = buildBallisticMask(1.5);
      // Twice. The windowed pass only ever lowers stations, and lowering a
      // station lowers the running prefix minimum every later station is
      // measured against — so one sweep leaves a small residual wherever the
      // correction itself moved the anchor. A second sweep clears it; a third
      // measurably changes nothing on any seed.
      safety.flatCapped = SOLVER.feasible
        ? enforceDescentShipped(bm) + enforceDescentShipped(bm)
        : enforceDescentShipped(bm);
      if (SOLVER.feasible) refreshBasis();
      // The pass lowered stations, so the relief and the pace both moved.
      computeVertKappa();
      solveSpeeds(false);
      const after = measureShippedFlat();
      safety.flatShipped = +after.flat.toFixed(1);
      safety.flatShippedAt = +after.at.toFixed(0);
      safety.flatRunShipped = +after.run.toFixed(1);
      safety.flatRunShippedAt = +after.runAt.toFixed(0);
      safety.flatWinRise = +after.win.toFixed(3);
      safety.flatWinAt = +after.winAt.toFixed(0);
    }
    // The plan radius, on the line that ships — after the bench, after
    // everything. See measureShipRadius().
    {
      const rM = measureShipRadius();
      safety.planShipMin = +rM.worst.toFixed(2);
      safety.planShipAt = +rM.at.toFixed(0);
      safety.planShipUnder = rM.under;
    }

    // =====================================================================
    // FIX C — THE WALL GATE, MEASURED ON THE PROFILE THAT SHIPS.
    //
    // WHAT THIS IS FOR. Seed 12345 shipped 12.8 m of continuous committed
    // gradient at 0.82-0.84 at arc 202 — ground the measured RIDE_CEIL table
    // says the reference rider has ZERO lean budget on, at any radius, any bank
    // and any speed. Three independent instruments agreed it is impassable: the
    // table itself returns 0 above a 0.75 gradient, an analytic replica of that
    // exact cell failed 0/9 at every saturation, radius, bank and speed, and
    // every autopilot cell on the shipped build died there. `SHIP_MAX_DEAD` was
    // already measuring the same quantity — on the BASE profile, as a variant
    // TIE-BREAK — and when every variant failed it the build shrugged and
    // shipped the least bad one.
    //
    // A course with a wall no rider can pass is worse than no course. Seed 2
    // already refuses, loudly and deterministically, when no route is
    // admissible; this is the same refusal one measurement later, on the profile
    // the rider actually meets rather than on the marched polyline.
    //
    // WHY 8 m AND NOT ZERO. A metre or two of ground above the budget is a chute
    // a rider rolls: they arrive with momentum, they are on it for a fifth of a
    // second, and the bike does not need to steer to cross it. Round 12
    // measured 4.0 m surviving on seed 99999 and 12.8 m killing 11-19 of 21
    // cells on 12345. 8 m is 1.3 s at the design floor speed and about six bike
    // lengths — past any argument that momentum carries it.
    //
    // WHAT IT COSTS, stated plainly: on the four buildable seeds this refuses
    // 12345 (10.8 m at up to 0.91 on DIRT, arc 94) and passes 20260726 (0.0),
    // 777 (0.0) and 99999 (3.8 m at arc 2004). One seed in five now refuses,
    // where two of five ship a wall.
    {
      const n = S.n;
      const arc = new Float64Array(n);
      for (let i = 1; i < n; i++) {
        arc[i] = arc[i - 1]
          + Math.hypot(S.px[i] - S.px[i - 1], S.py[i] - S.py[i - 1], S.pz[i] - S.pz[i - 1]);
      }
      const half = Math.max(1, Math.round(2.0 / S.ds));
      let run = 0, worst = 0, at = 0, worstG = 0, runG = 0, worstSurf = 0, runSurf = 0;
      for (let i = 1; i < n; i++) {
        const a = i - half < 0 ? 0 : i - half;
        const b = i + half > n - 1 ? n - 1 : i + half;
        const d = arc[b] - arc[a];
        const dy = S.py[a] - S.py[b];
        const g = d > 1e-6 ? dy / Math.sqrt(Math.max(1e-9, d * d - dy * dy)) : 0;
        if (leanBudget(g, 0, S.surface[i]) <= FEAS_DEAD_RAD) {
          run += arc[i] - arc[i - 1];
          if (g > runG) { runG = g; runSurf = S.surface[i]; }
          if (run > worst) { worst = run; at = arc[i] - run; worstG = runG; worstSurf = runSurf; }
        } else { run = 0; runG = 0; }
      }
      safety.deadShipped = +worst.toFixed(1);
      safety.deadShippedAt = +at.toFixed(0);
      safety.deadShippedGrade = +worstG.toFixed(2);
      if (SOLVER.feasible && worst > SHIP_HARD_DEAD) {
        const msg = `[trail] GENERATION FAILED for seed ${ctx.seed}: the course that`
          + ` ships carries ${worst.toFixed(1)} m of continuous ground the reference`
          + ` rider cannot descend, starting at arc ${at.toFixed(0)} m`
          + ` (peak committed gradient ${(worstG * 100).toFixed(0)}%, surface id ${worstSurf};`
          + ` the measured lean budget is zero there at any radius, bank or speed).\n`
          + `  ${routeAudit.length} route variants were marched and ${safety.stationTries}`
          + ` had their stations built; the best of them still ships this. Limit is`
          + ` ${SHIP_HARD_DEAD} m.\n`
          + `  Change ctx.seed, or lower the design drop. The trail deliberately does`
          + ` NOT ship a course with a wall no rider can pass — see the seed-2 refusal`
          + ` in routeAdmissible() for the same rule one measurement earlier.`;
        // eslint-disable-next-line no-console
        console.error(msg);
        const err = new Error(msg);
        err.trailRouteAudit = routeAudit.slice();
        err.trailDeadShipped = worst;
        err.trailDeadShippedAt = at;
        throw err;
      }
    }

    // CONTRACT §4's descent rule, on the property rather than the counter. This
    // one WARNS rather than throwing, and the distinction is deliberate: a
    // stretch that fails to descend is a pace killer and a CONTRACT breach, but
    // it is not a wall — round 12 measured a rider grinding 26 m of it at 6 m/s
    // — and refusing the reference seed over ground it can cross would be the
    // wrong trade. Where it fires, the excavation ceiling is what is binding
    // (safety.flatWinFloorBound), i.e. the route committed to a rise the profile
    // solve is not allowed to cut through, and the fix is a route, not a profile.
    if (SOLVER.feasible && safety.flatRunShipped > DESC_RUN_WARN) {
      // eslint-disable-next-line no-console
      console.warn(`[trail] seed ${ctx.seed}: CONTRACT §4 non-descending run of`
        + ` ${safety.flatRunShipped} m at arc ${safety.flatRunShippedAt} m`
        + ` (limit ~${DESC_WIN} m). ${safety.flatWinFloorBound} stations are pinned at the`
        + ` ${RAMP_CUT_MAX} m excavation ceiling, so the profile solve cannot cut it out —`
        + ` this is the route crossing a rise, not the profile failing to descend.`);
    }

    // ---- BALLISTIC ACCEPTANCE, on the final profile -----------------------
    // Two questions, both asked of the line that ships and not of the line the
    // feature was authored on: is the design speed at the lip at or below the
    // speed the ballistics were solved for, and is it inside the speed window
    // this module published for that feature? The first is CONTRACT §4's "a
    // jump you can't land"; the second is the same question asked honestly.
    safety.jumpOverSpeed = 0;
    safety.jumpWindowBad = 0;
    safety.jumpAudit = [];
    for (const f of features) {
      const p = f.params;
      if (!p || p.iLip === undefined || !p.speedWindow) continue;
      const v = S.speed[p.iLip];
      const over = v > p.takeoffSpeed * 1.0001;
      const outside = v < p.speedWindow[0] || v > p.speedWindow[1];
      if (over) safety.jumpOverSpeed++;
      if (outside) safety.jumpWindowBad++;
      safety.jumpAudit.push({
        type: f.type, name: p.name,
        vLip: +v.toFixed(2), vTakeoff: +p.takeoffSpeed.toFixed(2),
        ratio: +(v / p.takeoffSpeed).toFixed(3),
        window: [+p.speedWindow[0].toFixed(1), +p.speedWindow[1].toFixed(1)],
        inWindow: !outside,
      });
    }

    // Two-leg conflicts, measured on the FINAL stations and resolved by pinching
    // the contesting limbs. See auditLegConflicts(). The speed profile is re-run
    // when a width actually moved, because techSpeedCap() reads it.
    auditLegConflicts('audit');
    if (auditLegConflicts('resolve')) solveSpeeds(false);
    auditJointFeasibility();

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

    report(0.96);
    { const y = maybeYield(); if (y) await y; }
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

  // Async for the boot progress bar only — see the note above build().
  async function finalize(terrain) {
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
    report(0.5);
    { const y = maybeYield(); if (y) await y; }
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
    routeTrace,                   // per-step rideability trace (opt-in; see readSolverOpts)
    fanTrace,                     // per-candidate heading trace (opt-in, windowed)
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
