// bike.js — the downhill bike simulation. CONTRACT §5.
//
// =============================================================================
// ARCHITECTURE (read this before changing anything)
// =============================================================================
//
// The chassis is a *sprung mass* carried on two kinematic, ground-following wheels
// — the classic "raycast vehicle" topology, but with the raycast replaced by
// collision.sweepWheel() so each wheel rides the lower 120° of its contact arc and
// climbs over rocks and roots instead of snapping to a single height sample.
//
//   Linear DOFs   : position + velocity, fully integrated from forces. Always.
//   Pitch         : a real rotational DOF integrated from torque about the chassis
//                   origin. Load transfer, brake dive, squat, manuals, endos and
//                   their balance points all emerge from this — none of it is
//                   scripted. Gravity's torque about the chassis origin is included,
//                   which is precisely what creates the wheelie balance point.
//   Roll (lean)   : an inverted-pendulum DOF about the contact line, driven by a
//                   torque-limited rider balance controller — a critically-damped
//                   second-order servo with a rate limit, an arrest profile and an
//                   anti-windup clamp on the commanded lean. When the tyres cannot
//                   supply the lateral acceleration the lean demands, the bike keeps
//                   falling into the turn — that IS the front washing out, and it is
//                   recoverable by standing the bike up if you react in time.
//                   The pendulum runs against the EFFECTIVE normal specific force,
//                   not against a constant g: through a compression that is 2.4 g and
//                   over a crest it is a fraction of one, and the lean that can be
//                   held scales with it.
//   Steering      : an OUTPUT, solved from the complete steady-state single-track
//                   relation δ = L·a_y/v² + α_f − α_r, with the slip angles obtained
//                   by inverting the same tyre curve the physics just used. Only a
//                   small, heavily filtered feedback correction sits on top. See the
//                   long note above groundStep().
//   Yaw           : integrated from tyre lateral forces (single-track bicycle model),
//                   so the bike understeers, oversteers and drifts for real reasons.
//   In the air    : orientation is integrated freely from angular velocity, so whips,
//                   scrubs, flips and spins are genuine rigid-body rotation. On
//                   landing the free quaternion is decomposed back into
//                   heading/pitch/lean about the landing normal, so there is no snap.
//   Crashed       : full 6-DOF bouncing rigid body with angular momentum.
//
// Integration is a fixed 240 Hz accumulator, and the render transform published on
// bike.state is interpolated between the last two substeps. Behaviour is therefore
// identical at 30 fps and 165 fps, and the bike cannot tunnel through terrain.
//
// -----------------------------------------------------------------------------
// ALLOCATION: there is no `new` anywhere in update()/substep() — every vector,
// quaternion, hit record and event payload is preallocated. Measured steady-state
// allocation is 31.7 bytes/frame, which is V8 boxing the closure-scoped scalar sim
// state (`lean`, `pitch`, `yawRate`, …) as HeapNumbers on assignment; that is
// unavoidable short of replacing named state with Float64Array indices, and at
// 1.9 kB/s it is one young-generation scavenge every ~74 minutes. Two things that
// DID matter and are fixed: Math.hypot allocates in V8 (use Math.sqrt in the inner
// loop), and event payloads must be reused rather than built per emit.
//
// -----------------------------------------------------------------------------
// CONTRACT-NOTE — **bike.state IS READ-ONLY. WRITING TO IT DOES NOTHING.**
//   This is the single most dangerous thing about this module's interface and
//   CONTRACT §5 does not say it, so it is said here as loudly as possible.
//   `state.position`, `state.velocity`, `state.quaternion`, `state.angularVelocity`
//   and every wheel/suspension field are COPIES, refreshed from the authoritative
//   simulation state (P, V, Q, W, susp[], wheelSim[]) at the end of every update().
//   `state.position` and `state.quaternion` are not even copies of the current
//   substep — they are INTERPOLATED between the last two substeps so the render
//   transform is smooth (see the accumulator in update()). There is deliberately no
//   live aliasing: making them live would either expose a pose that jitters at the
//   substep boundary or make the render transform authoritative, and both are worse.
//   The failure mode is silent: `ctx.bike.state.velocity.y = 5` throws nothing, logs
//   nothing, and is erased ~a millisecond later.
//   To actually move the bike, use the mutators, which write P/V/Q/W and re-publish:
//       bike.setVelocity(v3) / bike.addVelocity(v3) / bike.setPosition(v3)
//       bike.setOrientation(quat) / bike.teleport(position, quaternion)
//       bike.reset(transform) / bike.respawn()
// CONTRACT-NOTE: `state.position` is the chassis origin at the MID-POINT of the
//   wheelbase, at axle height (the scaffold used the same convention and the chase
//   camera / motion-blur sphere both want the centre of the bike). The contract text
//   says "rear-axle-ish"; the authoritative wheel positions are always
//   state.wheels[0].position (front axle) and state.wheels[1].position (rear axle),
//   both in world space, and bike.geometry publishes every offset bikeModel needs.
// CONTRACT-NOTE: input axis convention chosen here — `input.pitch === +1` is weight
//   FORWARD / nose DOWN (endo, front-flip rotation); `-1` is weight BACK / nose UP
//   (manual, back-flip). That matches the universal MTB/MX convention for a stick
//   pushed away from you. `state.riderFore` publishes the derived weight shift in
//   metres (+ = forward) so rider.js never has to re-derive it.
// CONTRACT-NOTE: this module emits `bike:crash` at crash onset AND `run:crash`
//   (postfx already listens for the latter — see ADDENDUM §C). gameplay should
//   listen for `bike:crash` rather than re-emitting `run:crash`, or postfx gets it
//   twice. Also emitted: `bike:impact`, `bike:skid`, `bike:landed`, `bike:respawn`,
//   `water:splash`.
// CONTRACT-NOTE: event payload objects are pre-allocated and REUSED (no per-frame
//   allocation). Listeners must read them synchronously and must not retain them.
// CONTRACT-NOTE: collision.sweepWheel is called with the optional 4th `forward`
//   argument; see the note at the top of collision.js.
// =============================================================================

import * as THREE from 'three';
import { Surface } from '../world/terrain.js';
import { makeRng, subSeed, clamp, clamp01, lerp, smoothstep, damp } from '../core/rng.js';

// =============================================================================
// TUNING — everything you would reach for to change how the bike feels.
// Grouped and commented. Nothing outside this block is a magic number.
// =============================================================================
const T = {
  // ---- integration -------------------------------------------------------
  SUBSTEP: 1 / 240,          // s. Fixed physics step.
  MAX_SUBSTEPS: 12,          // caps a 50 ms frame; beyond this we drop time.

  // ---- mass & geometry ---------------------------------------------------
  MASS: 95,                  // kg, bike (17) + rider (78) + kit.
  RIDER_MASS: 78,            // kg, the part that can move relative to the bike.
  UNSPRUNG: 4.2,             // kg per wheel+brake+lower leg.
  GRAVITY: 9.81,
  WHEELBASE: 1.25,           // m — CONTRACT §0 scale reference.
  WHEEL_RADIUS: 0.37,
  CG_UP: 0.68,               // m above the axle line ⇒ 1.05 m above the ground.
  CG_FWD: -0.015,            // m, static bias (a shade behind centre — DH stance).
  RIDER_SHIFT: 0.30,         // m of fore/aft weight shift at full pitch input.
  RIDER_CG_UP: 0.86,         // m above the axle line — where the rider's mass sits.
  FRONT_BIAS: 0.45,          // static weight on the front wheel.
  I_PITCH: 52,               // kg·m² about the chassis origin.
  I_YAW: 34,                 // kg·m².
  ROLL_H: 1.05,              // m, cg height above the contact line (pendulum arm).

  // ---- suspension --------------------------------------------------------
  FORK_TRAVEL: 0.170,        // m — CONTRACT: fork ~170 mm.
  SHOCK_TRAVEL: 0.165,       // m — CONTRACT: shock ~165 mm.
  FORK_RAKE: 27 * Math.PI / 180,   // strut leans forward going down (slack head angle).
  SHOCK_RAKE: -8 * Math.PI / 180,  // rear axis leans back going down.
  FORK_LEN: 0.50,            // m, mount→axle at full extension.
  SHOCK_LEN: 0.42,
  SAG_FRAC: 0.30,            // sits 30% into travel at rest — CONTRACT requirement.
  SPRING_PROG: 1.9,          // progression: F = k·x·(1 + PROG·(x/max)²).
  DAMP_COMP: 0.36,           // compression damping ratio (of critical).
  DAMP_REB: 0.72,            // rebound ratio — HIGHER than compression, i.e. the
                             // shock returns SLOWLY. Required by the contract and
                             // it is what makes the bike feel planted.
  VKNEE_COMP: 0.65,          // m/s. Digressive blow-off knee: above this the
                             // compression damper softens, so square-edge hits do
                             // not spike the chassis. This is the whole difference
                             // between "harsh" and "supple".
  VKNEE_REB: 1.60,           // m/s, rebound knee (much less digressive).
  REBOUND_V_MAX: 3.2,        // m/s cap on how fast the wheel can chase the ground.
  BUMP_START: 0.86,          // fraction of travel where the bottom-out bumper bites.
  BUMP_K: 7.0e6,             // N/m² — quadratic bumper.
  BUMP_DAMP: 9.0e4,          // N·s/m² — kills bumper ring.
  TOPOUT_SPRING: 900,        // N/m, gentle spring pulling a hanging wheel down.

  // ---- tyres -------------------------------------------------------------
  KAPPA_PEAK: 0.13,          // longitudinal slip ratio at peak grip (dirt knobby).
  ALPHA_PEAK: 0.165,         // rad (9.5°) slip angle at peak lateral grip.
  TYRE_SLIDE_MU: 0.74,       // sliding grip as a fraction of peak.
  TYRE_SLIDE_S: 3.2,         // normalised slip at which we reach the sliding plateau.
  LAT_GRIP: 0.94,            // dirt tyres have a touch less lateral than longitudinal.
  LOAD_SENS: 0.16,           // μ falls as load rises — no infinite grip from loading
                             // one wheel. μ = μ₀(1 − SENS·(N/N_ref − 1)).
  LOAD_REF: 470,             // N, reference wheel load for the above.
  // Relaxation length is not cosmetic: it is what keeps the wheel-spin ODE stable
  // at 240 Hz. Without it the slip loop has a ~5 ms time constant and explodes.
  // (Eigenvalues of the (ω, κ) system at 15 m/s land at −30 ± 105i, giving
  // |1 + λh| ≈ 0.98 — stable with margin.)
  SLIP_VREF: 3.2,            // m/s, low-speed floor for the slip-angle denominators.
  RELAX_LONG: 0.26,          // m, longitudinal relaxation length (tyre carcass lag).
  RELAX_LAT: 0.40,           // m, lateral relaxation length.
  WHEEL_INERTIA: 0.38,       // kg·m² — a DH wheel + tyre is heavy.
  BRAKE_FRONT_MAX: 620,      // N·m at the wheel. Enough to lock it — that's the point.
  BRAKE_REAR_MAX: 470,
  PEDAL_TORQUE: 190,         // N·m at the rear wheel.
  PEDAL_SPINOUT: 11.5,       // m/s beyond which pedalling stops adding anything.
  BEARING_DRAG: 0.05,        // N·m·s/rad — bearings + seals + chain.

  // ---- aero & drivetrain -------------------------------------------------
  AERO_K: 0.36,              // ½ρCdA. Terminal ≈ 17.5 m/s on a 15% grade.
  AERO_TUCK: 0.78,           // multiplier when the rider is tucked (crouched, no input).

  // ---- steering & lean ---------------------------------------------------
  LEAN_MAX: 0.98,            // rad (56°) — the rider's physical limit; grip usually
                             // caps you well before this, except in a berm.
  // N·m of roll authority (body english + countersteer). This has to be at least
  // gravity's toppling torque at the LOW-SIDE THRESHOLD — M·g·h·sin(1.26) = 931 N·m
  // — or there is a band of lean angles that are silently fatal: the bike is not yet
  // "down" by the crash test, but the rider cannot physically arrest it, so every
  // excursion into that band ends on the floor with no save available. 820 N·m ran
  // out at 0.99 rad against a 1.26 rad threshold, and that 0.27 rad dead band was
  // the single largest source of crashes on the trail. Washouts are still washouts:
  // the lean ceiling, the front-slip WASH_ANGLE test and combined-slip braking all
  // still put you down. What changed is that a lean you react to in time is now
  // always recoverable, which is what the design intended all along.
  LEAN_TORQUE_MAX: 1000,
  // ---- rider balance controller -----------------------------------------
  // A critically-damped second-order servo on lean, with a rate limit and an
  // anti-windup clamp on the commanded lean. See the long note above groundStep()
  // for why this replaced the previous time-optimal (bang-bang) rate controller.
  LEAN_WN: 8.0,              // rad/s natural frequency (settles in ~0.5 s).
  LEAN_ZETA: 1.0,            // critically damped. Never overshoots, never rings.
  LEAN_RATE_MAX: 2.6,        // rad/s, fastest a rider can flick the bike over
                             // (upright to full lean in ~0.30 s).
  LEAN_APPROACH: 0.80,       // safety factor on the arrest profile (<1 so the rider
                             // starts arresting the roll slightly early).
  LEAN_CMD_RATE: 2.8,        // rad/s, how fast the *commanded* lean can move. A DH
                             // bike takes ~0.35 s to flick from upright to full
                             // lean; any quicker and you build roll momentum the
                             // rider cannot then arrest.
  LEAN_CMD_LEAD: 0.45,       // rad. ANTI-WINDUP: the commanded lean may never run
                             // more than this ahead of the achieved lean. Without
                             // it, holding full stick while the tyres are saturated
                             // winds the command out to the stops, and the servo
                             // then slams the other way the instant grip returns.
  LEAN_ANORM_MIN: 0.35,      // × g. Floor on the effective normal specific force —
  LEAN_ANORM_MAX: 3.0,       // × g. ...and ceiling. See aNormFilt in groundStep().
  LEAN_BALANCE_LAG: 14,      // 1/s. The rider's estimate of the lateral force the
                             // tyres are giving them is not instantaneous — real
                             // neuromuscular lag is ~70 ms. The PHYSICS still uses
                             // the true instantaneous force; only the rider's
                             // feed-forward is filtered. This is what stops the
                             // controller differentiating tyre-load noise.
  // Available grip caps lean, but the ceiling has to be the grip that is ACTUALLY
  // there: a wheel in the air, or one already spending its friction on the brakes,
  // holds no lean at all. See leanCeiling() — the limit is set by whichever end
  // saturates first under the moment balance, not by μ alone. Getting this wrong
  // is what let the rider command 0.79 rad when the bike could only hold 0.62,
  // which is a capsize every time.
  LEAN_GRIP_MARGIN: 0.92,    // the last few percent are the player's to lose.
  LEAN_LIMIT_LAG: 5.0,       // 1/s smoothing on the ceiling — a wheel skipping over
                             // a root must not yank the lean out from under you.
  LEAN_LIMIT_FALL: 2.5,      // × LAG when the ceiling is FALLING. Losing grip is felt
                             // immediately; trusting it again takes a moment. Also
                             // stops a momentary wheel lift ratcheting the ceiling up.
  LEAN_LIMIT_MIN: 0.16,      // rad. Airborne floor. Even with no grip at all the
                             // rider may hold a little angle; a ceiling of zero is a
                             // bike that snaps bolt upright the moment it gets light.
                             // NB this is a floor on the FINAL ceiling, applied after
                             // the plant derate — applying it before (as the code used
                             // to) made the floor unreachable and let the ceiling go
                             // to zero.
  // FLOOR ON THE COMMANDED LEAN WHILE A TYRE IS ON THE GROUND. Applied in
  // groundStep(), not to the ceiling. It is a design invariant, not a tuning number:
  // **a planted bike can always be steered.** The plant derate may make the rider ask
  // for less lean, but it may never leave them unable to generate a turn at all,
  // because the situation the derate fires in — getting light, near the edge of the
  // tread — is precisely the situation where the rider MUST be able to turn.
  // Measured before this existed: the ceiling sat pinned at LEAN_LIMIT_MIN for 5 m of
  // a 53% grade with both wheels planted and 20-50 mm of suspension in hand, the bike
  // could command 0.13 rad, lateral error ran 0.23 -> 2.18 m across a 2.9 m tread, and
  // the run went over the cutting. 0.34 rad is a_lat = g·tan(0.34) = 3.5 m/s², i.e. a
  // 30 m radius at 10 m/s — enough to hold a line, not enough to rail a berm.
  // It is capped by the ceiling at the point of use, so on genuinely slippery ground
  // it never authorises lean the tyres will not hold.
  // HONEST SCOPE: removing this floor measures clearly worse (48.8% of attempted
  // distance and 100 crashes, against 51.4% and 68 with it), but it did NOT unblock
  // the 85 m failure — see the note on LEAN_LOOK_FLOOR and the harness `probe9`
  // result: doubling the whole ceiling there buys 1 m of progress.
  LEAN_LIMIT_PLANTED: 0.34,
  LEAN_ONE_WHEEL: 0.55,      // ceiling derate when only one end is on the ground.
  LEAN_BOTH_LAG: 7.0,        // 1/s, SYMMETRIC smoothing on "are both wheels down".
                             // The one-wheel derate is applied off this rather than
                             // off the raw boolean, because the raw boolean flickers
                             // at several Hz on real tread and anything asymmetric
                             // downstream of it ratchets. ~140 ms: a manual reads as
                             // one-wheel, a skip over a root does not.
  // ---- the plant derate: what it is ACTUALLY for -------------------------
  // A lean is only worth having if the tyres are on the ground to convert it into a
  // turn. Getting light over a crest and continuing to lean is how riders end up
  // landing on their side: the bar does nothing, so the rider asks for more, and the
  // bike arrives at the next contact at 45°. That — an AIRBORNE-TRANSITION problem —
  // is the whole and only job of this term.
  //
  // It is NOT a grip term, and it used to be used as one. `leanGrip` is already a
  // load-INDEPENDENT friction ratio: the equilibrium of the roll pendulum is
  // tan(lean) = a_lat/a_norm and the tyre limit is a_lat ≤ μ·a_norm, so a_norm
  // cancels and the lean a surface will hold is atan(μ) whether the tyres are
  // carrying 1.0 g or 0.4 g. Multiplying that ratio by a load fraction says "at half
  // load you may only use half the coefficient of friction", which is not a physical
  // statement about anything. Worse, it is backwards where it bites: at half load the
  // lean needed to produce a given lateral acceleration is LARGER, not smaller, so
  // derating the ceiling exactly when the bike is light removes the rider's ability
  // to use the (already halved) grip they do have.
  //
  // Hence thresholds that mean "the tyres are carrying essentially nothing", not
  // "the tyres are carrying less than 0.8 g". They are fractions of the load THIS
  // SLOPE legitimately provides (g·cosθ, taken from the contact normal), so a bike
  // simply accelerating down a steepening 53% grade — where a_n is honestly 0.5-0.9 g
  // and both tyres are planted — reads as fully planted, which it is.
  LEAN_PLANT_LO: 0.06,
  LEAN_PLANT_HI: 0.45,
  // 1/s on the rider's sense of tyre load. Much slower than LEAN_BALANCE_LAG on
  // purpose: this feeds a derate that is asymmetric (falls 2.5× faster than it
  // rises), and an asymmetric filter fed a noisy signal ratchets — a wheel skipping
  // over a root several times a second drove the ceiling to its floor and left the
  // bike unable to generate a turn at all. Anticipation is the lookahead's job; this
  // term only has to know how loaded the tyres have been over the last fifth of a
  // second.
  LEAN_LOAD_LAG: 6.0,
  // Hard ceiling on the COMMANDED lean, as a fraction of the lean from which a total
  // loss of grip is still recoverable — asin(a_roll_max·h/g) = 0.99 rad for this
  // bike. Past that the rider's full body english is less than gravity's toppling
  // torque and the bike is going down no matter what, so the rider never asks to go
  // there deliberately.
  LEAN_SAFE_FRAC: 0.86,
  // ---- the lean ceiling as a CONTROL CONSTRAINT --------------------------
  // The ceiling used to be a number the command was clipped against and a crash
  // trigger evaluated after the fact. Clipping the command at exactly the ceiling
  // leaves no room for the roll to overshoot, for the ceiling to move, or for the
  // servo's own lag — and it measured 14% of a trail run spent ABOVE the ceiling,
  // with `lowside` the single largest crash cause. The rider now aims deliberately
  // inside the grip they have.
  LEAN_CMD_MARGIN: 0.80,     // fraction of the ceiling the rider will ask for. The
                             // remaining 20% is the reserve that absorbs a moving
                             // ceiling, and it is what turns "lay it down" into
                             // "run wide", which is the recoverable, visible mistake.
  // Terrain lookahead for the PREDICTIVE ceiling. A rider does not discover a crest
  // when the wheels leave it; they see it coming and go neutral first. Convex ground
  // ahead unloads the tyres (a_n ≈ g + v²·κ), and lower-μ ground ahead takes grip
  // away, so both are folded into the ceiling BEFORE the bike gets there. Without
  // this the ceiling can only ever react, and a lean committed 0.3 s ago is already
  // unavailable by the time the ceiling notices.
  LEAN_LOOK_T: 0.35,         // s of travel to look ahead...
  LEAN_LOOK_MIN: 2.0,        // ...clamped to this many metres...
  LEAN_LOOK_MAX: 5.0,        // ...and this many.
  // THE SCALE THE WHEELS ACTUALLY SEE. The prediction used to be ONE three-point
  // curvature straddling the whole lookahead — at 10.5 m/s that is a ±5.8 m stencil,
  // an 11.6 m chord. A second difference over 11.6 m is an average, and on a slope
  // that is merely STEEPENING (the entry to any chute) that average reads as a huge
  // unloading which then only partly happens, because the bike follows the transition
  // and its velocity rotates with the ground. Measured at 83 m on the trail: the
  // chord read κ = −0.21 1/m (a 4.7 m radius crest) and predicted a_n = −14.9 m/s²,
  // i.e. "you are about to be thrown off the ground at 2.3 g", while both tyres were
  // planted with 16-55 mm of suspension compressed and honestly carrying 0.66 g.
  //
  // A two-wheeler does not feel curvature at that scale in either direction. Short
  // convexities are straddled by the wheelbase and swallowed by 170 mm of travel;
  // long ones are ridden through. What unloads the CHASSIS is ground curvature on the
  // order of the contact geometry — the wheelbase plus a wheel's contact arc, about
  // 1.9 m end to end. So the profile is now MARCHED: heights sampled every
  // LEAN_LOOK_STEP metres along the travel ray, a three-point curvature taken at each
  // station with that same step as its half-width, and the WORST station over the
  // window is what the rider reacts to. A real crest shows up at every scale and
  // still fires the derate; a steepening slope does not, because at wheel scale it is
  // not convex.
  LEAN_LOOK_STEP: 1.0,       // m. Stencil half-width AND station spacing.
  LEAN_LOOK_MAX_N: 8,        // cap on stations (allocation-free scratch is sized for
                             // this + 2 guard samples).
  LEAN_LOOK_MAX_SLOPE: 1.2,  // dh/ds (50°) above which a station is NOT rideable
                             // ground and its curvature is not evidence about tyre
                             // load. The trail's own steepest grade is ~0.6.
  // Floor under the predictive derate. SHIPS AT 0 — i.e. OFF — and the reason is
  // measurement, not theory. The theory says it should exist: the prediction is a
  // guess about ground the bike has not reached, taken along a ray that follows the
  // current velocity, so on a bike that is already sliding it is a guess about ground
  // the bike will never ride. But swept over the whole trail, floors of 0.3, 0.5 and
  // 0.65 all measured WORSE than none (53.0/48.4/55.9% of attempted distance against
  // 60.4/53.0% with no floor, across two independent worlds), which is the same
  // result the previous engineer got for floors on the old chord estimate. The
  // guarantee the floor was meant to provide is provided instead by the floor on the
  // COMMANDED lean (LEAN_LIMIT_PLANTED), which is the constraint that actually
  // matters — the rider can always steer — and which does not measure worse.
  // Left in as a live knob because it is the first thing to reach for if the
  // prediction is ever found firing on ground the bike does not ride.
  LEAN_LOOK_FLOOR: 0.0,
  // Floor on the arrest budget (rad/s²). The budget is aRollMax minus the toppling
  // acceleration the rider can actually FEEL, so in a balanced corner it is the full
  // authority and in a fall it collapses to this — which is the point.
  LEAN_ARREST_MIN: 0.8,
  // Tyre-scrub roll damping, 1/s. The roll axis was the only attitude DOF in the
  // model with NO damping at all: pitch has PITCH_DAMP, yaw has the −yawRate·42 scrub
  // term, and roll had nothing, so any impulse the ground handed the pendulum
  // integrated straight into a free roll rate. Rolling a bike on the ground drags both
  // contact patches sideways and that resists — it is the same scrub as the yaw term,
  // about the same axis budget. Scaled by how loaded the tyres are, so there is none
  // in the air. The rider's feed-forward CANCELS it (a rider knows their own bike
  // scrubs), so it is invisible while they have authority left and only bites once
  // they are at full body english — which is exactly the runaway it exists to bound.
  ROLL_DAMP: 0.8,
  STEER_MAX: 0.62,           // rad at the bar at walking pace...
  STEER_SPEED_FALL: 0.09,    // ...decaying as 1/(1 + FALL·v).
  // ---- steer solver -------------------------------------------------------
  // Steer is an OUTPUT: whatever bar angle holds the lean the rider chose. The
  // feed-forward is the FULL single-track steady state — Ackermann PLUS the slip
  // angles the tyres must actually run (δ = L·a_y/v² + α_f − α_r). The α term is
  // not a refinement: at 0.49 rad of lean it is −0.037 rad against an Ackermann of
  // +0.049, i.e. three times the size of the correct answer and of the opposite
  // sign. Leaving it out is what forced the old code to find the bar angle with a
  // high-gain proportional loop on raw yaw rate, and that loop lives downstream of
  // wheel loads that swing 0→2400 N on a rocky chute.
  STEER_FB: 0.16,            // yaw-rate error feedback (steer to kill excess yaw).
  STEER_BETA: 0.30,          // sideslip-ERROR feedback — steer into the slide. The
                             // error is measured against the sideslip the corner
                             // actually calls for, so a healthy steady corner asks
                             // for nothing and only a genuine slide does.
  STEER_KI: 0.10,            // 1/s integral on yaw-rate error: mops up model error
                             // (load transfer, camber) without a standing offset.
  STEER_INT_MAX: 0.06,       // rad, hard clamp on the integrator = its anti-windup.
  STEER_CORR_MAX: 0.16,      // rad, clamp on the combined correction.
  STEER_FILT: 11.0,          // 1/s low-pass on every signal that feeds the bar.
                             // 90 ms — a rider's hands, and well inside the 0.4 m
                             // lateral relaxation length, so it costs no real
                             // response. THIS is what keeps contact noise off the
                             // handlebar.
  STEER_ACT: 22,             // 1/s bar actuator bandwidth.
  STEER_RATE_MAX: 6.0,       // rad/s. Hands have a speed limit; without one, a step
                             // in the feed-forward is a step at the contact patch.
  // Countersteer. Driven by the roll ACCELERATION the rider is asking for, so it is
  // a transient that dies once the lean is established. It is taken from the
  // FILTERED demand — the raw demand saturates against LEAN_TORQUE_MAX and would
  // put a ±0.055 rad square wave on the bar all by itself.
  COUNTERSTEER: 0.040,       // s per rad/s² of roll demand.
  COUNTERSTEER_MAX: 0.055,   // rad, hard clamp on the transient.
  LOWSPEED_UPRIGHT: 3.0,     // m/s below which the rider just dabs and stands it up.

  // ---- attitude impulse limits -------------------------------------------
  // `pitch` and `lean` are attitude DOFs of the SPRUNG mass, but the torques that
  // drive them are read straight off the rigid ground reaction — including the
  // quadratic bump-stop, which is a 7.0e6 N/m² spring. A hard bottom-out therefore
  // hands the rotational DOFs a 15 kN impulse with no compliance in between, and
  // one substep of that is a roll or pitch rate no bicycle can have (measured: a
  // 6 kN rear hit doubling the roll rate to 4.85 rad/s and kicking the nose 48° up,
  // which then launches and barrel-rolls). The real bike's answer is that such an
  // impulse mostly goes into vertical motion through the suspension, not into
  // attitude. These three ceilings are generous — several times anything reachable
  // by riding — so they are invisible except on a bottom-out.
  TAU_PITCH_MAX: 2600,       // N·m of ground-reaction pitch torque. ~4 g of load
                             // transfer; a hard but survivable landing is ~1900.
  TAU_YAW_MAX: 1200,         // N·m of tyre-derived yaw torque. Steady cornering at
                             // the limit is ~250; a deliberate rear step-out ~800.
                             // Without this a 6.6 kN rear bottom-out on a drop
                             // landing spun the bike at 2.9 rad/s in three substeps,
                             // and it then flew 100° off-axis and washed out — the
                             // single most repeatable crash on the whole trail.
  LEAN_ALAT_MAX: 2.0,        // multiples of the CURRENT normal specific force that
                             // the roll pendulum will accept laterally. The tyres
                             // can hold ~0.7 of it in a corner, so this only ever
                             // clips a bottom-out spike.
  LEAN_RATE_LIMIT: 5.0,      // rad/s of roll on the ground (286°/s — a full flick
                             // is 2.6).
  PITCH_RATE_LIMIT: 5.0,     // rad/s of pitch on the ground. A manual pops at ~1.7.
  YAW_RATE_LIMIT: 3.5,       // rad/s of yaw on the ground (200°/s). A committed
                             // corner at 12 m/s is 0.9; past ~2 you are spinning.

  // ---- pitch -------------------------------------------------------------
  PITCH_DAMP: 190,           // N·m·s/rad, extra pitch damping (anti-porpoise).
  RIDER_PITCH_TORQUE: 330,   // N·m of deliberate pitch authority (manual/endo).
  PITCH_HOLD_KP: 900,        // rider keeping the bike composed relative to the ground.
  PITCH_HOLD_KD: 240,
  PITCH_HOLD_MAX: 260,       // N·m — a rider can shift ~0.28 m: 932 N × 0.28 m.

  // ---- pumping (the soul of downhill) ------------------------------------
  CROUCH_RANGE: 0.34,        // m the rider's mass can travel up/down.
  CROUCH_BASE: 0.35,         // resting attack-position crouch (0 = tall, 1 = deep).
  PUMP_CROUCH: 0.55,         // extra crouch while `pump` is held (preload).
  PUMP_ABSORB: 0.24,         // automatic absorption of hard suspension compressions.
  CROUCH_KP: 225,            // actuator stiffness (full pop in ~0.15 s)...
  CROUCH_KD: 26,
  PUMP_FORCE_MAX: 1400,      // N the rider can push/pull. ~1.5× system weight.
  PUMP_ASSIST_GAIN: 1.6,     // amplifies the *signed* thrust the pump physically
                             // produced. Good timing → gain; bad timing → loss. It
                             // cannot reward a mistimed pump because the underlying
                             // quantity is already signed by the terrain slope.
  PUMP_THRUST_CAP: 1100,     // N, total (physical + assist) clamp.
  PUMP_LOOKAHEAD: 3.0,       // m, terrain curvature lookahead for the pump window.

  // ---- air ---------------------------------------------------------------
  AIR_PITCH_TORQUE: 4.6,     // rad/s² of authority...
  AIR_ROLL_TORQUE: 6.2,
  AIR_YAW_TORQUE: 3.9,       // ...yaw is the whip/scrub axis.
  AIR_RAMP: 0.28,            // s of air time before full authority (kills hop-spam).
  AIR_DRAG_ROT: 0.55,        // 1/s rotational damping in the air.
  // Roll rate on the ground exists to BALANCE a lean against gravity — it is not
  // an intention to rotate the bike. Carrying it wholesale into a jump means
  // getting light mid-corner at 45° of lean quietly turns into a barrel roll and
  // an unrecoverable low-side on touchdown. Real riders' bikes just stay where
  // they were left. Keep a fraction of it (so a deliberate flick still whips) and
  // damp the residual whenever the player is not actively asking for roll.
  AIR_ROLL_KEEP: 0.35,
  AIR_ROLL_DAMP: 2.4,        // 1/s, only applied below AIR_ROLL_DEADZONE of input.
  AIR_ROLL_DEADZONE: 0.25,
  AIR_MIN: 0.09,             // s of no contact before we call it airborne.
  WHIP_RETURN: 2.6,          // rad/s² pulling a whipped bike back in line as the
                             // ground approaches — without this, whips are unlandable.
  WHIP_RETURN_WINDOW: 0.40,  // s before predicted touchdown that the return kicks in.

  // ---- landings ----------------------------------------------------------
  LAND_MISMATCH: 0.30,       // fraction of speed lost for a fully mismatched landing.
  LAND_VN_LOSS: 0.10,        // extra longitudinal loss per m/s of normal impact.
  CASE_VN: 5.5,              // m/s of normal closing speed above which it's a case.
  LAND_MIN_AIR: 0.25,        // s of air below which a touchdown is a wheel skip,
                             // not a landing, and is not judged as one.
  CASE_QUALITY: 0.88,        // ...but only if the attitude was poor too. Dropping
                             // 2 m onto a matched transition is a big vN and a
                             // perfectly good landing; the same drop onto a face
                             // that kicks back up at you is a case.

  // ---- crash -------------------------------------------------------------
  CRASH_VN: 9.0,             // m/s normal impact velocity.
  CRASH_AXIS: 50 * Math.PI / 180,   // rad off-axis on landing — CONTRACT §5.
  CRASH_WALL_V: 6.0,         // m/s into a wall.
  WALL_NORMAL_Y: 0.55,       // a contact normal flatter than this is a wall, not ground.
  // FULL LOCK AT SPEED — a designed outcome, not an accident.
  // Held full lock on smooth ground, measured on a 26% grade:
  //   30 km/h  no crash, 1.7 m radius, scrubs to 14.5 km/h — a tight, recoverable turn
  //   45 km/h  `washout` at 2.25 s      60 km/h  `washout` at 1.90 s
  // In both crashing cases |lean| never reaches the ceiling (peak 0.66 against a
  // ceiling of 0.83, 0.0% of the run above it) — the bike is NOT falling over, the
  // FRONT TYRE is being asked for a slip angle past WASH_ANGLE and letting go. That is
  // the distinction this whole controller is built on: the rider asked for something
  // the front could not do, so the front washes; the controller never quietly commanded
  // a lean the tyres would not hold and then laid the bike down. Slamming the bar to
  // the stop at 60 km/h SHOULD end the run, and it should end it in a way the player
  // can name.
  WASH_ANGLE: 0.62,          // rad of front slip angle that counts as washing out.
  WASH_TIME: 0.45,           // s of sustained wash before you are down.
  LOOP_MARGIN: 0.24,         // rad past the balance point before it's a loop-out/OTB.
  TUMBLE_MIN: 1.1,           // s minimum tumble before respawn.
  TUMBLE_MAX: 3.6,           // s maximum.
  TUMBLE_RESTITUTION: 0.30,
  TUMBLE_FRICTION: 0.55,
  TUMBLE_SPIN: 2.4,          // how much tangential velocity converts to spin.
  RESPAWN_SPEED: 6.0,        // m/s rolling entry speed after a respawn.

  // ---- anti-frustration (tunable down to 0 for a "pro" mode) -------------
  ASSIST: 1.0,               // master scale for the two assists below.
  GRIP_ASSIST: 0.20,         // widens the friction ellipse by up to 20% but ONLY
                             // when you are already at the limit — invisible when
                             // riding within yourself.
  AIR_AUTO_STAB: 3.1,        // rad/s² levelling torque when no air input is given.
  AIR_STAB_DEADZONE: 0.16,   // input magnitude below which auto-stabilisation runs.

  // ---- misc --------------------------------------------------------------
  SPEED_CAP: 34,             // m/s hard safety clamp.
  SKID_THRESHOLD: 0.55,      // normalised combined slip that counts as sliding.
  IMPACT_MIN: 1.6,           // m/s normal velocity worth an event.
  IMPACT_FULL: 9.0,          // m/s normal velocity = severity 1.
  SPLASH_MIN_SPEED: 1.5,     // m/s
  OFFTRAIL_MARGIN: 0.65,     // m past the tread edge before offTrail goes true.
  TRAIL_RESYNC: 1.5,         // s of sustained nearestT disagreement before the
                             // teleport guard gives up and accepts the jump. See
                             // updateTrail().
};

// =============================================================================
// Per-surface physics. Index = SurfaceId from terrain.js.
//   DIRT 0, LOAM 1, ROCK 2, GRAVEL 3, GRASS 4, ROOT 5, MUD 6, SNOW 7
// =============================================================================
const SURF_MU = new Float32Array([1.08, 1.15, 0.95, 0.78, 0.86, 0.70, 0.58, 0.52]);
const SURF_CRR = new Float32Array([0.030, 0.042, 0.022, 0.055, 0.052, 0.036, 0.086, 0.072]);
// How much of the peak survives once sliding — rock is consistent, mud is not.
const SURF_SLIDE = new Float32Array([0.76, 0.80, 0.82, 0.68, 0.70, 0.64, 0.55, 0.58]);
// Feeds particles/audio/camera via state.roughness.
const SURF_ROUGH = new Float32Array([0.25, 0.18, 0.85, 0.55, 0.30, 0.75, 0.35, 0.20]);

const surfMu = (m) => SURF_MU[m] !== undefined ? SURF_MU[m] : 1.0;
const surfCrr = (m) => SURF_CRR[m] !== undefined ? SURF_CRR[m] : 0.03;
const surfSlide = (m) => SURF_SLIDE[m] !== undefined ? SURF_SLIDE[m] : 0.75;

// =============================================================================
// Scratch — module scope. Nothing in update()/substep() allocates.
// =============================================================================
const _fwd = new THREE.Vector3();
const _up = new THREE.Vector3();
const _right = new THREE.Vector3();
const _fwdH = new THREE.Vector3();
const _axis = new THREE.Vector3();
const _mount = new THREE.Vector3();
const _axleExt = new THREE.Vector3();
const _cg = new THREE.Vector3();
const _r = new THREE.Vector3();
const _F = new THREE.Vector3();
const _Fsum = new THREE.Vector3();
const _tau = new THREE.Vector3();
const _tauSum = new THREE.Vector3();
const _omega = new THREE.Vector3();
const _vC = new THREE.Vector3();
const _tFwd = new THREE.Vector3();
const _tRight = new THREE.Vector3();
const _wheelFwd = new THREE.Vector3();
const _n = new THREE.Vector3();
const _tmpA = new THREE.Vector3();
const _tmpB = new THREE.Vector3();
const _tmpC = new THREE.Vector3();
const _tmpD = new THREE.Vector3();
const _travel = new THREE.Vector3();
const _groundUp = new THREE.Vector3(0, 1, 0);
const _landNormal = new THREE.Vector3(0, 1, 0);
const _predPos = new THREE.Vector3();
const _bodyA = new THREE.Vector3();
const _bodyB = new THREE.Vector3();
const _qa = new THREE.Quaternion();
const _qb = new THREE.Quaternion();
const _qSpin = new THREE.Quaternion();
const _m4 = new THREE.Matrix4();
const _WORLD_UP = new THREE.Vector3(0, 1, 0);
// Marched terrain profile for the predictive lean ceiling. Sized for
// LEAN_LOOK_MAX_N stations plus the two guard samples the end stencils need.
// Module scope so updateLeanLookahead() never allocates.
const _lookH = new Float64Array(16);

// =============================================================================
// Tyre curve.
//
// Normalised combined-slip friction. `s` is the magnitude of the normalised slip
// vector (s = 1 sits exactly on the friction peak). The rising branch 2s/(1+s²) is
// the brush-model / simplified-Pacejka shape — near-linear off zero, peaking at
// exactly 1.0 — and past the peak it eases onto a sliding plateau rather than
// collapsing, because a tyre that loses ALL grip once it slides is twitchy and
// miserable to ride. No trig, so it is free to call four times per substep.
// =============================================================================
function tyreCurve(s, slidePlateau) {
  if (s <= 0) return 0;
  const rise = (2 * s) / (1 + s * s);
  if (s <= 1) return rise;
  const k = clamp01((s - 1) / (T.TYRE_SLIDE_S - 1));
  const w = k * k * (3 - 2 * k);
  return rise * (1 - w) + slidePlateau * w;
}

// =============================================================================
// Inverse tyre — "what slip angle do I need for this much lateral force?"
//
// The rider steers to a lean, and holding a lean requires a known lateral force at
// each contact patch. Turning that force back into a slip angle is what the steer
// solver needs, and it is the exact inverse of the rising branch of tyreCurve():
//
//     2s/(1+s²) = ρ    ⇒    s = (1 − √(1−ρ²)) / ρ        (the root with s ≤ 1)
//
// where ρ is the demanded force as a fraction of the peak available. Past ρ = 1
// there is no solution — you are asking for more than the tyre has — so we return
// the angle at the peak and let the lean ceiling be the thing that says no.
// One sqrt and one atan, twice per substep.
// =============================================================================
const TAN_ALPHA_PEAK = Math.tan(0.165);          // matches T.ALPHA_PEAK
const ALPHA_ASK_MAX = 0.34;                      // rad, ~2× peak. Well under WASH_ANGLE.

const ALPHA_ASK_KNEE = 0.88;                     // ρ where the exact inverse hands over
function slipForForce(force, capacity) {
  if (!(capacity > 1)) return ALPHA_ASK_MAX;     // no load ⇒ no answer; ask for the max
  const rho = force / capacity;
  if (!(rho > 1e-4)) return 0;
  // Exact inverse while it exists.
  const r = rho < ALPHA_ASK_KNEE ? rho : ALPHA_ASK_KNEE;
  const s = (1 - Math.sqrt(1 - r * r)) / r;
  const a = Math.atan(s * TAN_ALPHA_PEAK);
  if (rho <= ALPHA_ASK_KNEE) return a;
  // Past the knee it must SATURATE SMOOTHLY, never step. Snapping straight to
  // ALPHA_ASK_MAX at ρ = 1 puts a 0.24 rad discontinuity in the steer feed-forward,
  // and since the demand sits right on the tyre's capacity through any committed
  // corner, that discontinuity is a limit cycle: bar steps out, force arrives,
  // demand drops below capacity, bar steps back. Measured as a visible stutter in
  // the yaw rate under constant input before this was smoothed.
  const k = clamp01((rho - ALPHA_ASK_KNEE) / (1.30 - ALPHA_ASK_KNEE));
  const w = k * k * (3 - 2 * k);
  return a + (ALPHA_ASK_MAX - a) * w;
}

/**
 * Lateral force still available at a contact patch, after the friction ellipse has
 * already spent some of it on braking or drive. This is the quantity the whole
 * cornering model hangs off: it is what sets the lean ceiling AND what the steer
 * feed-forward inverts, so the two can never disagree with each other.
 */
function lateralCapacity(ws) {
  if (!ws.contact || !(ws.load > 1)) return 0;
  const budget = ws.mu * ws.load;
  if (!(budget > 1)) return 0;
  const used = clamp01(Math.abs(ws.fx) / budget);
  const left = 1 - used * used;
  return left > 0 ? budget * T.LAT_GRIP * Math.sqrt(left) : 0;
}

function makeWheel() {
  return {
    position: new THREE.Vector3(),   // world axle position
    contact: false,
    normal: new THREE.Vector3(0, 1, 0),
    material: Surface.DIRT,
    compression: 0,                  // 0..1 of travel
    slipRatio: 0,
    slipAngle: 0,
    load: 0,                         // N
    spinRate: 0,                     // rad/s
    // additive
    slipNorm: 0,                     // normalised combined slip (1 = at the limit)
    forceLong: 0,
    forceLat: 0,
    contactPoint: new THREE.Vector3(),
  };
}

export function createBike(ctx) {
  const collision = () => ctx.collision;
  // Deterministic stream for the crash tumble, so a replay of the same run tumbles
  // the same way. Never Math.random() in anything that feeds the simulation.
  const rng = makeRng(subSeed((ctx && ctx.seed) || 1, 'bike-tumble'));

  // -------------------------------------------------------------------------
  // Published state — CONTRACT §5.
  // -------------------------------------------------------------------------
  const state = {
    // ***********************************************************************
    // READ-ONLY. Every field below is a COPY republished each frame from the
    // authoritative sim state; `position`/`quaternion` are additionally
    // INTERPOLATED between the last two substeps. Assigning to any of them is
    // silently discarded on the next update(). Use bike.setVelocity /
    // addVelocity / setPosition / setOrientation / teleport / reset / respawn.
    // See the CONTRACT-NOTE at the top of this file.
    // ***********************************************************************
    position: new THREE.Vector3(),        // read-only, interpolated
    quaternion: new THREE.Quaternion(),   // read-only, interpolated
    velocity: new THREE.Vector3(),        // read-only copy of V
    angularVelocity: new THREE.Vector3(), // read-only copy of ω
    speed: 0,
    forward: new THREE.Vector3(0, 0, -1),
    right: new THREE.Vector3(1, 0, 0),
    up: new THREE.Vector3(0, 1, 0),

    wheels: [makeWheel(), makeWheel()],   // [front, rear]
    suspension: {
      fork: { travel: 0, velocity: 0, max: T.FORK_TRAVEL },
      shock: { travel: 0, velocity: 0, max: T.SHOCK_TRAVEL },
    },

    airborne: false, airTime: 0, lastAirTime: 0,
    crashed: false, crashTimer: 0,
    lean: 0, steer: 0,
    pumpImpulse: 0,
    brakeFront: 0, brakeRear: 0,
    pedalling: 0,
    gForce: 1,
    surface: Surface.DIRT,
    wheelie: 0, endo: 0,
    distance: 0,
    trailT: 0,
    // False while the teleport guard is rejecting `trail.nearestT` — the bike is not
    // where the nearest-point query says it is (it has fallen beside another leg of
    // the run). `trailT`/`distance` hold at the last verified value while this is
    // false. See updateTrail(). Also announced once per event as `bike:trailJump`.
    trailTValid: true,
    offTrail: false, lateral: 0,

    // ---- additive (documented extras other modules asked for) ------------
    pitch: 0,               // rad, + = nose up, relative to the contact plane
    pitchRate: 0,
    leanRate: 0,
    yawRate: 0,             // rad/s about world up
    riderFore: 0,           // m of rider weight shift, + = forward
    riderCrouch: T.CROUCH_BASE,   // 0 = tall, 1 = deep crouch
    riderPumpForce: 0,      // N, + = rider unweighting the bike
    pumpDrive: 0,           // -1..1, + = driving the bike into the ground
    pumpWindow: 0,          // 0..1 — how good *right now* is for a pump
    pumpGain: 0,            // W of longitudinal power from pumping (signed)
    skid: 0,                // 0..1 overall sliding intensity
    gripAvail: 1.05,        // load-weighted μ under the tyres right now
    leanLimit: 0.8,         // rad, the lean the current grip will actually hold
    // ---- lean-ceiling breakdown (diagnostic; HUD/debug may read these) ----
    // leanLimit = max(floor, leanGrip × oneWheelFrac);  the plant/lookahead
    // fractions bound the COMMANDED lean instead (groundStep), not the ceiling.
    // Publishing the three factors separately is what made the 91 m stop-ship
    // diagnosable at all: "the ceiling is at its floor" is not a cause, "the
    // PREDICTED plant fraction is 0.0 while both tyres are carrying 0.6 g" is.
    leanGrip: 0.62,         // rad, the pure friction-ratio half of the ceiling
    plantFrac: 1,           // 0..1, how planted the tyres are RIGHT NOW
    plantDerate: 1,         // 0..1, min(plantFrac, lookPlantFrac) — bounds the COMMAND
    lookPlantFrac: 1,       // 0..1, how planted the ground ahead will let them be
    aNormLook: 9.81,        // m/s², worst predicted normal specific force ahead
    roughness: 0.25,        // surface roughness under the wheels
    inAir: false,           // alias of airborne, kept for readability
    whip: 0,                // rad, chassis yaw away from the travel direction
    airRotation: 0,         // rad of accumulated pitch rotation this flight
    landQuality: 1,         // 0..1 quality of the last landing
    lastLanding: { airTime: 0, quality: 0, vNormal: 0, axisError: 0, whip: 0, cased: false },
    wheelieRaw: 0,          // unclamped, > 1 means past the balance point
    endoRaw: 0,
    inWater: false,
    respawnCount: 0,
    checkpointIndex: -1,
  };

  // -------------------------------------------------------------------------
  // Simulation state (authoritative; state.position/quaternion are interpolated
  // copies published for rendering).
  // -------------------------------------------------------------------------
  const P = new THREE.Vector3();          // chassis origin
  const V = new THREE.Vector3();          // velocity
  const Q = new THREE.Quaternion();       // orientation
  const W = new THREE.Vector3();          // angular velocity (air / crash)

  const P0 = new THREE.Vector3();         // previous substep pose, for interpolation
  const Q0 = new THREE.Quaternion();
  const P1 = new THREE.Vector3();
  const Q1 = new THREE.Quaternion();

  let heading = Math.PI;                  // fwd = (sin h, 0, cos h); trail runs to −Z
  let pitch = 0, pitchRate = 0;
  let lean = 0, leanRate = 0, leanCmd = 0;
  let yawRate = 0;
  let steerAngle = 0;

  // ---- controller state (all filtered / integrated, all reset in reset()) ----
  // Every one of these exists to keep tyre-load noise out of the handlebar. They
  // are the rider's senses, not the bike's state: the physics always uses the raw
  // instantaneous values.
  let aLatFilt = 0;        // m/s², lateral acceleration the tyres are giving us
  let aNormFilt = 9.81;    // m/s², specific force along the contact normal
  let yawRateFilt = 0;     // rad/s
  let betaFilt = 0;        // rad, chassis sideslip
  let steerInt = 0;        // rad, bounded integral on yaw-rate error
  let rollAccelFilt = 0;   // rad/s², what the countersteer transient is built from
  let leanLimit = 0.62;    // rad, the lean the grip under us right now will hold
  let leanGrip = 0.62;     // rad, the grip half of it before the load derate
  // The rider's sense of how loaded the tyres are. Deliberately NOT aNormFilt: that
  // one is the pendulum's effective gravity and is pinned to 1 g whenever there is no
  // contact, which told the load derate the tyres were fully loaded at the exact
  // moment they were carrying nothing. This is the true normal specific force, and it
  // is zero in the air.
  let aLoadFilt = T.GRAVITY;
  // Predictive terms, refreshed once a frame from the terrain ahead. 1 = the ground
  // ahead takes nothing away from the ceiling.
  let lookLoadFrac = 1;
  let lookMuFrac = 1;
  let aNormLook = T.GRAVITY;   // m/s², worst predicted normal specific force ahead
  let plantFracCur = 1;        // 0..1, published breakdown of the ceiling
  let bothFrac = 1;            // 0..1, symmetrically smoothed "both wheels down"
  let plantDerate = 1;         // 0..1, how much lean the rider will COMMIT to

  let crouch = T.CROUCH_BASE, crouchRate = 0, riderForce = 0;
  let riderFore = 0;

  let airborne = false, airTime = 0, groundedTime = 0, noContactTime = 0;
  let washTimer = 0;
  let crashed = false, crashTimer = 0, crashHandled = false;
  let accumulator = 0;
  let gForceSmooth = 1;
  let pumpThrust = 0, pumpWindow = 0;
  let skidSmooth = 0;
  // Load-weighted mean of the μ actually available at the contact patches. Feeds
  // the lean ceiling, so riding onto mud or gravel really does take lean away.
  let muAvail = 1.05;
  let lastPumpProfileFrame = -1;
  // Trail-progress teleport guard — see updateTrail().
  let trailTGuard = 0;
  let trailTSeed = true;        // next query is a re-seed (reset/respawn), accept it
  let trailJumpTimer = 0;
  let trailJumpFlagged = false;
  let lastFrameDt = 1 / 60;

  // -------------------------------------------------------------------------
  // Suspension units. Geometry is derived so that at design sag the axles sit
  // exactly at ±WHEELBASE/2 from the chassis origin, at axle height.
  // -------------------------------------------------------------------------
  function makeSusp(travel, rake, restLen, staticLoad, sign) {
    const sag = travel * T.SAG_FRAC;
    const L = restLen - sag;
    const cosR = Math.cos(rake);
    const sinR = Math.sin(rake);
    // Strut axis (chassis frame): axisUp = −cos(rake), axisFwd = +sin(rake).
    // A raked fork acts as an incline, so the wheel's normal load for a given strut
    // force is Fs/cos(rake). Size the spring with that factored in, otherwise the
    // bike sits proud of the design sag.
    const k0 = (staticLoad * cosR) / (sag * (1 + T.SPRING_PROG * T.SAG_FRAC * T.SAG_FRAC));
    const sprung = staticLoad / T.GRAVITY;
    const cCrit = 2 * Math.sqrt(k0 * sprung);
    return {
      max: travel, sag, restLen, rake, cosR, sinR,
      mountUp: cosR * L,
      mountFwd: sign * (T.WHEELBASE * 0.5) - sinR * L,
      k0,
      cComp: cCrit * T.DAMP_COMP,
      cReb: cCrit * T.DAMP_REB,
      comp: sag,
      rate: 0,
      rateSmooth: 0,
      force: 0,
      bottomed: false,
    };
  }

  const staticF = T.MASS * T.GRAVITY * T.FRONT_BIAS;
  const staticR = T.MASS * T.GRAVITY * (1 - T.FRONT_BIAS);
  const susp = [
    makeSusp(T.FORK_TRAVEL, T.FORK_RAKE, T.FORK_LEN, staticF, +1),
    makeSusp(T.SHOCK_TRAVEL, T.SHOCK_RAKE, T.SHOCK_LEN, staticR, -1),
  ];

  // Per-wheel working data.
  const wheelSim = [
    { kappa: 0, alpha: 0, spin: 0, load: 0, mu: 1, slipN: 0, fx: 0, fy: 0, rrTorque: 0, contact: false, sinceContact: 99 },
    { kappa: 0, alpha: 0, spin: 0, load: 0, mu: 1, slipN: 0, fx: 0, fy: 0, rrTorque: 0, contact: false, sinceContact: 99 },
  ];

  // Pre-allocated hit records and lookahead profile.
  let hits = null;
  let bodyHit = null;
  let profileOut = null;

  function ensureScratch() {
    const c = collision();
    if (!hits) {
      hits = [c ? c.makeHit() : null, c ? c.makeHit() : null];
      if (!hits[0]) hits = null;
    }
    if (!bodyHit && c) bodyHit = c.makeHit();
    const mkProf = () => (c && c.makeProfile ? c.makeProfile()
      : { here: 0, ahead: 0, behind: 0, grad: 0, curvature: 0 });
    if (!profileOut && c) profileOut = mkProf();
    return !!hits;
  }

  // -------------------------------------------------------------------------
  // Reused event payloads (see the CONTRACT-NOTE about retention).
  // -------------------------------------------------------------------------
  const evImpact = { severity: 0, position: new THREE.Vector3(), normal: new THREE.Vector3(0, 1, 0), surface: 0 };
  const evSkid = { intensity: 0, surface: 0 };
  const evSplash = { position: new THREE.Vector3(), velocity: new THREE.Vector3() };
  const evCrash = { position: new THREE.Vector3(), severity: 0, reason: '', speed: 0 };
  const evLanded = { airTime: 0, quality: 0, vNormal: 0, whip: 0, cased: false, position: new THREE.Vector3() };
  const evRespawn = { position: new THREE.Vector3(), checkpoint: -1 };
  const evTrailJump = { from: 0, to: 0, gap: 0 };

  // Per-frame event accumulators (flushed once, at the end of update()).
  let pendImpact = 0;
  const pendImpactPos = new THREE.Vector3();
  const pendImpactNrm = new THREE.Vector3(0, 1, 0);
  let pendImpactSurf = 0;
  let skidEmitTimer = 0;
  let splashTimer = 0;

  function emit(name, payload) {
    if (ctx.events && ctx.events.emit) ctx.events.emit(name, payload);
  }

  function noteImpact(severityVel, point, normal, material) {
    if (severityVel <= pendImpact) return;
    pendImpact = severityVel;
    pendImpactPos.copy(point);
    pendImpactNrm.copy(normal);
    pendImpactSurf = material;
  }

  // =========================================================================
  // Pose helpers
  // =========================================================================

  /**
   * Build the chassis quaternion from a ground reference normal plus the three
   * rider-frame angles. Order matters: heading about world up, then pitch about the
   * chassis right axis, then lean about the (pitched) forward axis.
   */
  function buildGroundQuat(refUp, hdg, pit, lea, outQ) {
    _tmpA.set(Math.sin(hdg), 0, Math.cos(hdg));
    // Project the heading direction into the contact plane.
    _tmpA.addScaledVector(refUp, -_tmpA.dot(refUp));
    if (_tmpA.lengthSq() < 1e-8) {
      // Degenerate: heading is parallel to the reference normal (a vertical wall).
      // Any perpendicular direction will do; take the world-up cross.
      _tmpA.crossVectors(_WORLD_UP, refUp);
      if (_tmpA.lengthSq() < 1e-8) _tmpA.set(0, 0, -1);
    }
    _tmpA.normalize();
    _tmpB.crossVectors(_tmpA, refUp).normalize();      // right, before pitch
    _qa.setFromAxisAngle(_tmpB, pit);                  // + pitch = nose up
    _tmpA.applyQuaternion(_qa).normalize();            // pitched forward
    _tmpC.copy(refUp).applyQuaternion(_qa).normalize();// pitched up
    _qb.setFromAxisAngle(_tmpA, lea);                  // + lean = tips to the right
    _tmpC.applyQuaternion(_qb).normalize();
    _tmpB.crossVectors(_tmpA, _tmpC).normalize();      // right
    _tmpC.crossVectors(_tmpB, _tmpA).normalize();      // re-orthogonalised up
    _tmpD.copy(_tmpA).negate();                        // local +Z = −forward
    _m4.makeBasis(_tmpB, _tmpC, _tmpD);
    outQ.setFromRotationMatrix(_m4);
    return outQ;
  }

  /**
   * Decompose the free-flight quaternion back into heading/pitch/lean about a given
   * reference up. Called on touchdown so the ground solver picks up exactly where
   * the air solver left off, with no visible snap.
   */
  function decomposePose(refUp) {
    _fwd.set(0, 0, -1).applyQuaternion(Q);
    _up.set(0, 1, 0).applyQuaternion(Q);
    heading = Math.atan2(_fwd.x, _fwd.z);
    pitch = Math.asin(clamp(_fwd.dot(refUp), -1, 1));

    // Zero-lean reference frame at this heading/pitch.
    _tmpA.set(Math.sin(heading), 0, Math.cos(heading));
    _tmpA.addScaledVector(refUp, -_tmpA.dot(refUp));
    if (_tmpA.lengthSq() < 1e-8) _tmpA.copy(_fwd);
    _tmpA.normalize();
    _tmpB.crossVectors(_tmpA, refUp).normalize();
    _qa.setFromAxisAngle(_tmpB, pitch);
    _tmpC.copy(refUp).applyQuaternion(_qa).normalize();   // up0
    _tmpA.applyQuaternion(_qa).normalize();               // fwd0
    _tmpB.crossVectors(_tmpA, _tmpC).normalize();         // right0
    lean = Math.atan2(_up.dot(_tmpB), _up.dot(_tmpC));

    // Angular rates from the free angular velocity.
    yawRate = W.dot(_WORLD_UP);
    pitchRate = W.dot(_tmpB);
    leanRate = W.dot(_tmpA);
  }

  // =========================================================================
  // Suspension
  // =========================================================================

  function springForce(su, c) {
    if (c <= 0) return 0;
    const f = c / su.max;
    let F = su.k0 * c * (1 + T.SPRING_PROG * f * f);
    if (f > T.BUMP_START) {
      const over = (f - T.BUMP_START) * su.max;
      F += T.BUMP_K * over * over;
    }
    return F;
  }

  function damperForce(su, v) {
    // Digressive both ways; compression blows off much earlier than rebound.
    if (v >= 0) return su.cComp * v / (1 + v / T.VKNEE_COMP);
    const a = -v;
    return -(su.cReb * a / (1 + a / T.VKNEE_REB));
  }

  // =========================================================================
  // Reset / respawn
  // =========================================================================

  function reset(transform) {
    const trail = ctx.trail;
    let t = transform;
    // Where on the trail we are landing. Seeding state.trailT here matters: the next
    // nearestT() call uses it as its search hint, and a stale hint from wherever we
    // crashed would force the expensive global fallback (or, worse, snap us to the
    // wrong leg of a switchback).
    let spawnT = 0;
    if (t && t.t !== undefined && !t.quaternion && trail && trail.sampleAt) {
      // A checkpoint {t, position} — build a full transform from the trail.
      const s = trail.sampleAt(t.t);
      P.copy(s.position);
      _tmpA.copy(s.tangent).normalize();
      heading = Math.atan2(_tmpA.x, _tmpA.z);
      spawnT = t.t;
      t = null;
    } else if (t && t.position) {
      P.copy(t.position);
      if (t.quaternion) {
        _fwd.set(0, 0, -1).applyQuaternion(t.quaternion);
        heading = Math.atan2(_fwd.x, _fwd.z);
      }
      if (typeof t.t === 'number') spawnT = t.t;
    } else if (trail && trail.startTransform && trail.startTransform.position) {
      P.copy(trail.startTransform.position);
      _fwd.set(0, 0, -1).applyQuaternion(trail.startTransform.quaternion);
      heading = Math.atan2(_fwd.x, _fwd.z);
    } else {
      P.set(0, 0, 0);
      heading = Math.PI;
    }

    const terrain = ctx.terrain;
    if (terrain && terrain.sampleHeight) {
      const h = terrain.sampleHeight(P.x, P.z);
      P.y = h + T.WHEEL_RADIUS + 0.015;
      terrain.sampleNormal(P.x, P.z, _groundUp);
    } else {
      _groundUp.set(0, 1, 0);
    }
    _landNormal.copy(_groundUp);

    pitch = 0; pitchRate = 0;
    lean = 0; leanRate = 0; leanCmd = 0;
    yawRate = 0; steerAngle = 0;
    // Controller state. Every filter and integrator has to be cleared here, or a
    // respawn starts with the rider still reacting to the crash they just had.
    aLatFilt = 0; aNormFilt = T.GRAVITY;
    yawRateFilt = 0; betaFilt = 0; steerInt = 0; rollAccelFilt = 0;
    leanLimit = 0.62; leanGrip = 0.62; aLoadFilt = T.GRAVITY;
    lookLoadFrac = 1; lookMuFrac = 1; aNormLook = T.GRAVITY;
    plantFracCur = 1; bothFrac = 1; plantDerate = 1;
    V.set(0, 0, 0);
    W.set(0, 0, 0);
    buildGroundQuat(_groundUp, heading, 0, 0, Q);

    for (let i = 0; i < 2; i++) {
      susp[i].comp = susp[i].sag;
      susp[i].rate = 0; susp[i].rateSmooth = 0; susp[i].force = 0; susp[i].bottomed = false;
      const ws = wheelSim[i];
      ws.kappa = 0; ws.alpha = 0; ws.spin = 0; ws.load = i === 0 ? staticF : staticR;
      ws.contact = true; ws.sinceContact = 0; ws.slipN = 0; ws.fx = 0; ws.fy = 0;
    }

    crouch = T.CROUCH_BASE; crouchRate = 0; riderForce = 0; riderFore = 0;
    airborne = false; airTime = 0; groundedTime = 1; noContactTime = 0;
    washTimer = 0; crashed = false; crashTimer = 0; crashHandled = false;
    accumulator = 0; gForceSmooth = 1; pumpThrust = 0; pumpWindow = 0;
    skidSmooth = 0;

    P0.copy(P); P1.copy(P);
    Q0.copy(Q); Q1.copy(Q);

    state.position.copy(P);
    state.quaternion.copy(Q);
    state.velocity.set(0, 0, 0);
    state.angularVelocity.set(0, 0, 0);
    state.speed = 0;
    state.crashed = false; state.crashTimer = 0;
    state.airborne = false; state.airTime = 0; state.inAir = false;
    state.lean = 0; state.steer = 0; state.pitch = 0;
    state.wheelie = 0; state.endo = 0; state.wheelieRaw = 0; state.endoRaw = 0;
    state.skid = 0; state.gForce = 1; state.whip = 0;
    state.trailT = spawnT;
    trailTGuard = spawnT; trailTSeed = true; trailJumpTimer = 0;
    trailJumpFlagged = false; state.trailTValid = true;
    state.distance = spawnT * ((ctx.trail && ctx.trail.length) || 1);
    refreshDerived(1 / 60);
  }

  /** Drop the rider back at the last checkpoint they passed. */
  function respawn() {
    const trail = ctx.trail;
    let target = null;
    let idx = -1;
    if (trail && trail.checkpoints && trail.checkpoints.length) {
      for (let i = 0; i < trail.checkpoints.length; i++) {
        if (trail.checkpoints[i].t <= state.trailT + 1e-4) { target = trail.checkpoints[i]; idx = i; }
      }
    }
    if (target) reset(target);
    else reset(null);

    // Give them a rolling start rather than a dead stop — a standing restart on a
    // 20% chute is miserable.
    _fwd.set(0, 0, -1).applyQuaternion(Q);
    V.copy(_fwd).multiplyScalar(T.RESPAWN_SPEED);
    for (let i = 0; i < 2; i++) wheelSim[i].spin = T.RESPAWN_SPEED / T.WHEEL_RADIUS;
    state.respawnCount++;
    state.checkpointIndex = idx;
    evRespawn.position.copy(P);
    evRespawn.checkpoint = idx;
    emit('bike:respawn', evRespawn);
  }

  function triggerCrash(reason, severityVel) {
    if (crashed) return;
    crashed = true;
    crashTimer = 0;
    crashHandled = false;
    state.crashed = true;
    washTimer = 0;

    // Seed the tumble: keep linear momentum, convert some of it into spin so the
    // bike cartwheels rather than sliding like a crate.
    _fwd.set(0, 0, -1).applyQuaternion(Q);
    _up.set(0, 1, 0).applyQuaternion(Q);
    _right.crossVectors(_fwd, _up).normalize();
    const sp = V.length();
    W.copy(_right).multiplyScalar(-sp * 0.55 - 1.2);
    W.addScaledVector(_fwd, lean * 3.2 + (rng() - 0.5) * 2.0);
    W.addScaledVector(_WORLD_UP, yawRate * 0.6 + (rng() - 0.5) * 1.6);
    V.multiplyScalar(0.82);
    V.y += 1.4;

    evCrash.position.copy(P);
    evCrash.severity = clamp01((severityVel || sp) / T.CRASH_VN);
    evCrash.reason = reason;
    evCrash.speed = sp;
    emit('bike:crash', evCrash);
    emit('run:crash', evCrash);
    noteImpact(Math.max(severityVel || 0, sp * 0.5), P, _WORLD_UP, state.surface);
  }

  // =========================================================================
  // Crash tumble — a real bouncing rigid body.
  // =========================================================================
  function tumbleStep(h) {
    const c = collision();
    V.y -= T.GRAVITY * h;
    // Air drag on a tumbling body is much higher than on a tucked rider.
    const sp = V.length();
    if (sp > 0.01) V.addScaledVector(V, -(0.9 * sp * h) / T.MASS);
    P.addScaledVector(V, h);

    // Integrate the orientation from angular velocity.
    const wl = W.length();
    if (wl > 1e-5) {
      _tmpA.copy(W).multiplyScalar(1 / wl);
      _qSpin.setFromAxisAngle(_tmpA, wl * h);
      Q.premultiply(_qSpin).normalize();
    }
    W.multiplyScalar(Math.exp(-1.6 * h));

    if (c) {
      // Body capsule: roughly the rider curled around the bike.
      _fwd.set(0, 0, -1).applyQuaternion(Q);
      _bodyA.copy(P).addScaledVector(_fwd, 0.42);
      _bodyB.copy(P).addScaledVector(_fwd, -0.42);
      const R = 0.52;
      if (c.capsuleCast(_bodyA, _bodyB, R, bodyHit, 3) && bodyHit.depth > 0) {
        _n.copy(bodyHit.normal);
        P.addScaledVector(_n, bodyHit.depth * Math.max(0.35, _n.y) + 0.002);
        const vn = V.dot(_n);
        if (vn < 0) {
          // Normal bounce.
          _tmpA.copy(_n).multiplyScalar(-vn * (1 + T.TUMBLE_RESTITUTION));
          V.add(_tmpA);
          // Tangential friction, and convert what we scrub into spin.
          _tmpB.copy(V).addScaledVector(_n, -V.dot(_n));
          const tl = _tmpB.length();
          if (tl > 0.05) {
            const scrub = Math.min(tl, T.TUMBLE_FRICTION * -vn + 0.6);
            _tmpB.multiplyScalar(1 / tl);
            V.addScaledVector(_tmpB, -scrub);
            _tmpC.crossVectors(_n, _tmpB).multiplyScalar(scrub * T.TUMBLE_SPIN / R);
            W.add(_tmpC);
          }
          if (-vn > T.IMPACT_MIN) noteImpact(-vn, bodyHit.point, _n, bodyHit.material);
          state.surface = bodyHit.material;
        }
      }
    }

    crashTimer += h;
    state.crashTimer = crashTimer;

    const settled = crashTimer > T.TUMBLE_MIN && V.lengthSq() < 4.0 && W.lengthSq() < 3.0;
    if (!crashHandled && (settled || crashTimer > T.TUMBLE_MAX)) {
      crashHandled = true;
      // If gameplay wants to own respawning it can call bike.reset() during the
      // tumble (which clears `crashed`); we only self-respawn if nobody did.
      respawn();
    }
  }

  // =========================================================================
  // The substep — the whole simulation, at a fixed 240 Hz.
  // =========================================================================
  function substep(h, input) {
    if (crashed) { tumbleStep(h); return; }
    const c = collision();
    if (!c) return;

    // ---- 0. body axes ----------------------------------------------------
    _fwd.set(0, 0, -1).applyQuaternion(Q);
    _up.set(0, 1, 0).applyQuaternion(Q);
    _right.crossVectors(_fwd, _up).normalize();
    _fwdH.set(_fwd.x, 0, _fwd.z);
    if (_fwdH.lengthSq() < 1e-6) _fwdH.set(Math.sin(heading), 0, Math.cos(heading));
    _fwdH.normalize();

    // Angular velocity in world space (used for contact-point velocities).
    if (airborne) _omega.copy(W);
    else {
      _omega.set(0, 0, 0)
        .addScaledVector(_WORLD_UP, yawRate)
        .addScaledVector(_right, pitchRate)
        .addScaledVector(_fwd, leanRate);
    }

    // Rider weight shift and centre of mass.
    riderFore = input ? input.pitch * T.RIDER_SHIFT : 0;
    const cgFwd = T.CG_FWD + riderFore * (T.RIDER_MASS / T.MASS);
    _cg.copy(P).addScaledVector(_up, T.CG_UP).addScaledVector(_fwd, cgFwd);

    // ---- 1. rider vertical actuator (the pump) ---------------------------
    // Physical basis: the rider's mass is the only thing that can add energy to the
    // system. Crouching *quickly* accelerates that mass downwards, which UNWEIGHTS
    // the bike (the chassis has to support less than m·g). Extending drives the mass
    // up and LOADS the bike. Do the second one on the face of a compression, where
    // the contact normal leans forwards, and the extra normal force has a forward
    // component: that is a pump, and it is genuinely free speed.
    let crouchCmd = T.CROUCH_BASE + (input ? input.pump : 0) * T.PUMP_CROUCH;
    // The rider also passively absorbs a hard compression.
    const compRateAvg = (susp[0].rateSmooth + susp[1].rateSmooth) * 0.5;
    crouchCmd += clamp01(compRateAvg * 0.5) * T.PUMP_ABSORB;
    crouchCmd = clamp01(crouchCmd);

    const aMax = T.PUMP_FORCE_MAX / (T.RIDER_MASS * T.CROUCH_RANGE);
    let crouchAcc = T.CROUCH_KP * (crouchCmd - crouch) - T.CROUCH_KD * crouchRate;
    crouchAcc = clamp(crouchAcc, -aMax, aMax);
    crouchRate += crouchAcc * h;
    crouch += crouchRate * h;
    if (crouch < 0) { crouch = 0; if (crouchRate < 0) crouchRate = 0; }
    else if (crouch > 1) { crouch = 1; if (crouchRate > 0) crouchRate = 0; }
    // + = rider accelerating downwards = an UPWARD (unweighting) force on the bike.
    riderForce = T.RIDER_MASS * crouchAcc * T.CROUCH_RANGE;

    // ---- 2. suspension + ground contact ----------------------------------
    _Fsum.set(0, 0, 0);
    _tauSum.set(0, 0, 0);
    let contactCount = 0;
    let loadSum = 0;
    let muWeighted = 0;
    _tmpC.set(0, 0, 0);          // accumulates the load-weighted contact normal

    for (let i = 0; i < 2; i++) {
      const su = susp[i];
      const ws = wheelSim[i];
      const hit = hits[i];

      // Strut axis in world: chassis-down, raked.
      _axis.copy(_up).multiplyScalar(-su.cosR).addScaledVector(_fwd, su.sinR).normalize();
      _mount.copy(P).addScaledVector(_up, su.mountUp).addScaledVector(_fwd, su.mountFwd);
      _axleExt.copy(_mount).addScaledVector(_axis, su.restLen);

      // One sweep per wheel per substep, at FULL EXTENSION. The compression needed
      // to place the tyre exactly on the ground then falls straight out of the
      // sweep depth — no iteration, no penetration.
      c.sweepWheel(_axleExt, T.WHEEL_RADIUS, hit, _fwdH);
      const dn = Math.max(0.30, -_axis.y);            // downward component of the strut
      const cTarget = hit.depth / dn;                 // may be negative (wheel hanging)

      const prevComp = su.comp;
      if (cTarget > su.comp) {
        // Compression: the ground is rigid, so the wheel goes where it is told.
        su.comp = Math.min(cTarget, su.max);
      } else {
        // Extension is limited by the shock's rebound circuit. The terminal
        // extension speed of a massless wheel is where spring force == damper
        // force; the digressive knee lets it move faster on big extensions.
        const fs = springForce(su, su.comp);
        const vGuess = fs / Math.max(1, su.cReb);
        const vReb = Math.min(T.REBOUND_V_MAX, vGuess * (1 + vGuess / T.VKNEE_REB));
        su.comp = Math.max(cTarget < 0 ? 0 : cTarget, su.comp - vReb * h);
        if (su.comp < 0) su.comp = 0;
      }
      su.rate = (su.comp - prevComp) / h;
      // Light filter: the raw rate spikes by a metre per second on a single 0.35 m
      // heightfield cell and the damper would slam the chassis.
      su.rateSmooth = damp(su.rateSmooth, clamp(su.rate, -8, 8), 55, h);
      su.bottomed = su.comp >= su.max - 1e-4;

      const touching = cTarget > -1e-4 && su.comp <= cTarget + 2e-3;
      ws.contact = touching;
      if (touching) ws.sinceContact = 0; else ws.sinceContact += h;

      // --- spring + damper --------------------------------------------------
      let Fs = 0;
      if (touching) {
        Fs = springForce(su, su.comp) + damperForce(su, su.rateSmooth);
        if (su.bottomed && su.rateSmooth > 0) Fs += T.BUMP_DAMP * su.rateSmooth * su.max;
        if (Fs < 0) Fs = 0;                       // a strut cannot pull
      } else {
        // Hanging: a weak topout spring keeps the wheel tracking down towards the
        // ground rather than dangling, which matters for how quickly you regain grip.
        Fs = 0;
      }
      su.force = Fs;

      // Wheel normal load. Dividing by the strut/normal alignment is correct: a
      // raked fork is an incline, so a given strut force corresponds to a larger
      // ground normal force. The unsprung weight rides on the ground directly.
      _n.copy(hit.normal);
      const align = Math.max(0.35, -_axis.dot(_n));
      let N = touching ? (Fs / align + T.UNSPRUNG * T.GRAVITY * Math.max(0, _n.y)) : 0;
      if (!isFinite(N) || N < 0) N = 0;
      ws.load = N;
      loadSum += N;
      if (touching) {
        contactCount++;
        _tmpC.addScaledVector(_n, N + 60);          // +60 so a zero-load contact still counts
      }

      // --- tyre -------------------------------------------------------------
      ws.fx = 0; ws.fy = 0; ws.slipN = 0; ws.rrTorque = 0; ws.mu = 0;
      // Contact-point velocity, including the chassis rotation. This is what makes
      // the front and rear see *different* slip angles — i.e. what makes the bike
      // understeer or oversteer for a real reason.
      _r.copy(hit.point).sub(P);
      _vC.copy(V).add(_tmpA.crossVectors(_omega, _r));

      // Tyre frame in the contact plane.
      if (i === 0) {
        _qa.setFromAxisAngle(_n, -steerAngle);      // + steer = right = negative yaw
        _wheelFwd.copy(_fwd).applyQuaternion(_qa);
      } else {
        _wheelFwd.copy(_fwd);
      }
      _tFwd.copy(_wheelFwd).addScaledVector(_n, -_wheelFwd.dot(_n));
      if (_tFwd.lengthSq() < 1e-8) _tFwd.copy(_fwdH);
      _tFwd.normalize();
      _tRight.crossVectors(_tFwd, _n).normalize();

      const vLong = _vC.dot(_tFwd);
      const vLat = _vC.dot(_tRight);
      // Low-speed regularisation of the slip angles. atan2(vLat, |vLong|) is
      // unbounded as vLong → 0, so at walking pace a few cm/s of lateral drift reads
      // as 15° of slip angle and the tyre model returns a fully saturated, sliding
      // force. That is a numerical artefact, not a tyre: at 1.3 m/s a bicycle is not
      // sliding. It made every standing start unstable — pedal, tiny yaw
      // disturbance, huge slip angle, huge lateral force, more yaw — and the bike was
      // on its side inside 1.3 s. T.SLIP_VREF is the reference speed the slip angles
      // are measured against; above it nothing changes at all.
      const vAbs = Math.max(Math.abs(vLong), T.SLIP_VREF);

      // Wheel spin: drive − brake − tyre reaction. Semi-implicit and with the brake
      // applied as a clamp, so a locked wheel is exactly locked and never rings.
      let driveTorque = 0;
      if (i === 1 && input) {
        const spinout = 1 - smoothstep(T.PEDAL_SPINOUT * 0.6, T.PEDAL_SPINOUT, Math.abs(vLong));
        driveTorque = input.pedal * T.PEDAL_TORQUE * spinout;
      }
      const brakeCmd = i === 0 ? (input ? input.brakeFront : 0) : (input ? input.brakeRear : 0);
      const brakeTorque = brakeCmd * (i === 0 ? T.BRAKE_FRONT_MAX : T.BRAKE_REAR_MAX);

      // Steady-state slips.
      const kappaSS = clamp((ws.spin * T.WHEEL_RADIUS - vLong) / vAbs, -4, 4);
      const alphaSS = Math.atan2(vLat, vAbs);

      // Relaxation length — the carcass takes distance, not time, to build force.
      // This is also what keeps the wheel-spin ODE stable at 240 Hz.
      const relRateL = Math.min(Math.max(Math.abs(vLong), 1.2) / T.RELAX_LONG, 0.85 / h);
      const relRateA = Math.min(Math.max(Math.abs(vLong), 1.2) / T.RELAX_LAT, 0.85 / h);
      ws.kappa += (kappaSS - ws.kappa) * Math.min(1, relRateL * h);
      ws.alpha += (alphaSS - ws.alpha) * Math.min(1, relRateA * h);

      if (touching && N > 1) {
        // Load sensitivity: doubling the load does NOT double the grip.
        const mat = hit.material;
        let mu = surfMu(mat) * (1 - T.LOAD_SENS * (N / T.LOAD_REF - 1));
        mu = clamp(mu, surfMu(mat) * 0.55, surfMu(mat) * 1.35);

        muWeighted += mu * N;

        const sx = ws.kappa / T.KAPPA_PEAK;
        const sy = Math.tan(clamp(ws.alpha, -1.35, 1.35)) / Math.tan(T.ALPHA_PEAK);
        // NB: Math.hypot, not Math.sqrt, would be the obvious spelling here — but
        // V8 allocates on every Math.hypot call and this runs 4× per substep, i.e.
        // ~2000×/second. That alone was ~130 bytes/frame of garbage.
        const s = Math.sqrt(sx * sx + sy * sy);
        ws.slipN = s;

        // Grip assist: quietly widen the friction ellipse, but ONLY once you are
        // already leaning on it. Below the limit this multiplies by exactly 1.
        if (T.ASSIST > 0) {
          mu *= 1 + T.ASSIST * T.GRIP_ASSIST * smoothstep(0.88, 1.30, s);
        }

        // Published so the steer solver and the lean ceiling can invert exactly the
        // same tyre the physics just used — including the grip assist.
        ws.mu = mu;

        if (s > 1e-5) {
          const f = tyreCurve(s, surfSlide(mat)) * mu * N;
          ws.fx = f * (sx / s);
          ws.fy = -f * (sy / s) * T.LAT_GRIP;
        }

        // Rolling resistance is carcass hysteresis, i.e. a TORQUE on the wheel —
        // not a force on the chassis. Applying it as a chassis force would spin the
        // wheel *up* through the reaction term, which is backwards. As a torque it
        // slows the wheel, which drives a small negative slip, which retards the
        // bike through the contact patch. Same equilibrium, correct mechanism.
        ws.rrTorque = surfCrr(mat) * N * T.WHEEL_RADIUS * Math.tanh(ws.spin / 1.5);
      }

      // Integrate the wheel spin with the tyre reaction.
      let spinFree = ws.spin +
        ((driveTorque - ws.fx * T.WHEEL_RADIUS - ws.rrTorque - T.BEARING_DRAG * ws.spin) / T.WHEEL_INERTIA) * h;
      const brakeDelta = (brakeTorque / T.WHEEL_INERTIA) * h;
      if (Math.abs(spinFree) <= brakeDelta) spinFree = 0;
      else spinFree -= Math.sign(spinFree) * brakeDelta;
      ws.spin = clamp(spinFree, -260, 260);

      // --- accumulate onto the chassis --------------------------------------
      if (touching) {
        // Full ground reaction: N along the contact normal plus the in-plane tyre
        // force. Using N·n (not the strut force) is deliberate — the horizontal
        // component of a tilted normal is what launches you off a lip, pushes you
        // through a berm, and pays you back for a well-timed pump.
        _F.copy(_n).multiplyScalar(N).addScaledVector(_tFwd, ws.fx).addScaledVector(_tRight, ws.fy);
        _Fsum.add(_F);
        _tauSum.add(_tau.crossVectors(_r, _F));

        // Impact reporting: how fast the contact point was closing on the ground.
        const closing = -_vC.dot(_n);
        if (closing > T.IMPACT_MIN && su.rate > 0.6) {
          noteImpact(closing, hit.point, _n, hit.material);
        }
        if (su.bottomed && su.rate > 1.2) {
          noteImpact(Math.max(closing, 2.2), hit.point, _n, hit.material);
        }
        // Crash on a genuinely violent normal impact.
        if (closing > T.CRASH_VN && airTime > 0.15) {
          triggerCrash('impact', closing);
          return;
        }
      }

      // Publish.
      const w = state.wheels[i];
      w.contact = touching;
      w.normal.copy(hit.normal);
      w.material = hit.material;
      w.compression = clamp01(su.comp / su.max);
      w.slipRatio = ws.kappa;
      w.slipAngle = ws.alpha;
      w.load = N;
      w.spinRate = ws.spin;
      w.slipNorm = ws.slipN;
      w.forceLong = ws.fx;
      w.forceLat = ws.fy;
      w.contactPoint.copy(hit.point);
      // The rendered axle position follows the strut.
      w.position.copy(_mount).addScaledVector(_axis, su.restLen - su.comp);
    }

    // Load-weighted available grip. Smoothed so a single loose sample does not
    // yank the lean ceiling out from under the rider.
    if (contactCount > 0 && loadSum > 1) {
      muAvail = damp(muAvail, muWeighted / loadSum, 9, h);
    }

    // ---- 3. reference ground normal --------------------------------------
    if (contactCount > 0 && _tmpC.lengthSq() > 1e-8) {
      _tmpC.normalize();
      if (airborne) {
        // Touchdown substep: SNAP. The landing evaluation that runs a few lines
        // below judges the bike against this normal, and a 22/s filter would still
        // be showing it the sky.
        _groundUp.copy(_tmpC);
      } else {
        // Smooth: the raw blended normal still steps a little on rough ground, and
        // an unsmoothed reference makes the whole chassis twitch.
        _groundUp.lerp(_tmpC, 1 - Math.exp(-22 * h)).normalize();
      }
      _landNormal.copy(_groundUp);
    } else {
      _groundUp.lerp(_landNormal, 1 - Math.exp(-4 * h)).normalize();
    }

    // ---- 4. rider pump force + gravity + aero ----------------------------
    // Rider actuator reaction, applied at the rider's centre of mass so a pump with
    // your weight back lifts the front — exactly as it does in real life.
    if (Math.abs(riderForce) > 1) {
      _F.copy(_up).multiplyScalar(riderForce);
      _r.copy(_up).multiplyScalar(T.RIDER_CG_UP).addScaledVector(_fwd, riderFore);
      _Fsum.add(_F);
      _tauSum.add(_tau.crossVectors(_r, _F));
    }

    // Measure how much longitudinal thrust the pump actually produced, then amplify
    // it. `pumpThrust` is already signed by the terrain: pushing down where the
    // normal leans forward is positive (you gain), pushing down on the face of a
    // riser is negative (you pay). The assist therefore cannot reward bad timing.
    _travel.copy(V);
    _travel.y = 0;
    const travelLen = _travel.length();
    if (travelLen > 0.5 && contactCount > 0) {
      _travel.multiplyScalar(1 / travelLen);
      const nDotTravel = _groundUp.dot(_travel);
      pumpThrust = clamp(-riderForce * nDotTravel, -T.PUMP_THRUST_CAP, T.PUMP_THRUST_CAP);
      const assisted = clamp(pumpThrust * T.PUMP_ASSIST_GAIN, -T.PUMP_THRUST_CAP, T.PUMP_THRUST_CAP);
      _Fsum.addScaledVector(_travel, assisted);
      state.pumpGain = (pumpThrust + assisted) * travelLen;
    } else {
      pumpThrust = 0;
      state.pumpGain = 0;
    }

    // Gravity at the centre of mass. Its torque about the chassis origin is NOT
    // zero, and it is exactly what creates the wheelie and endo balance points.
    _F.set(0, -T.MASS * T.GRAVITY, 0);
    _Fsum.add(_F);
    _r.copy(_cg).sub(P);
    _tauSum.add(_tau.crossVectors(_r, _F));

    // Aero drag, reduced when tucked.
    const sp = V.length();
    if (sp > 0.05) {
      const tuck = lerp(1, T.AERO_TUCK, crouch * (input && !input.anyPressed ? 1 : 0.55));
      const drag = T.AERO_K * tuck * sp * sp;
      _Fsum.addScaledVector(V, -drag / sp);
    }

    // ---- 5. linear integration -------------------------------------------
    _tmpA.copy(_Fsum).multiplyScalar(h / T.MASS);
    V.add(_tmpA);
    // Non-gravity specific force, for camera shake / HUD.
    _tmpB.copy(_Fsum);
    _tmpB.y += T.MASS * T.GRAVITY;
    gForceSmooth = damp(gForceSmooth, _tmpB.length() / (T.MASS * T.GRAVITY), 12, h);

    if (V.lengthSq() > T.SPEED_CAP * T.SPEED_CAP) V.setLength(T.SPEED_CAP);
    P.addScaledVector(V, h);

    // ---- 6. attitude ------------------------------------------------------
    const grounded = contactCount > 0;
    if (grounded) noContactTime = 0; else noContactTime += h;

    if (airborne) {
      airTime += h;
      airStep(h, input);
      if (grounded) {
        // Touchdown. Evaluate before handing back to the ground solver.
        if (evaluateLanding()) return;      // crashed
        airborne = false;
        state.lastAirTime = airTime;
        airTime = 0;
        groundedTime = 0;
        decomposePose(_groundUp);
      }
    } else {
      groundedTime += h;
      groundStep(h, input, contactCount, loadSum);
      if (crashed) return;
      if (!grounded && noContactTime > T.AIR_MIN) {
        // Leaving the ground: carry the current rotational rates into free flight.
        airborne = true;
        airTime = noContactTime;
        state.airRotation = 0;
        W.set(0, 0, 0)
          .addScaledVector(_WORLD_UP, yawRate)
          .addScaledVector(_right, pitchRate)
          .addScaledVector(_fwd, leanRate * T.AIR_ROLL_KEEP);
        predictLandingNormal();
      }
    }

    // ---- 7. hard safety constraint (never tunnel) -------------------------
    resolvePenetration(h);
    wallStep(h);

    // ---- 8. sanity --------------------------------------------------------
    if (!isFinite(P.x) || !isFinite(P.y) || !isFinite(P.z) ||
        !isFinite(V.x) || !isFinite(V.y) || !isFinite(V.z)) {
      // Should never happen; if it does, do not take the whole game down with us.
      respawn();
    }
  }

  /**
   * The lean the grip actually under the tyres right now will hold.
   *
   * NOT atan(μ). In a steady corner the moment balance fixes the split of the
   * cornering force between the ends — F_f = M·a_y·b/L, F_r = M·a_y·a/L — so the
   * bike is limited by whichever end runs out of friction FIRST, not by the total.
   * With this bike's geometry and a downhill's forward load transfer the rear is
   * always the limiting end, and it is limited hard: a_y_max of ~7.0 m/s² against
   * the ~10.5 that atan(μ) implies. That is a real 0.62 rad ceiling versus a
   * fictional 0.79 one, and commanding the fictional one is a capsize — the tyres
   * cannot supply g·tan(lean), so the inverted pendulum simply keeps falling.
   *
   * Because it is built from `lateralCapacity`, it also falls when a wheel goes
   * light over a crest and when the brakes are already using the friction, which is
   * exactly when a real rider stands the bike up. And it RISES in a berm, because
   * the bank pushes the wheel loads up — so berms still buy lean, for the right
   * reason and without a special case.
   */
  /**
   * The PREDICTIVE half of the lean ceiling: what the ground the wheels are about to
   * be on will let the rider hold. Refreshed once a frame — it is a property of the
   * terrain a third of a second away, not of this substep — and costs at most
   * LEAN_LOOK_MAX_N + 2 height samples plus one material lookup.
   *
   * Two things take lean away before you arrive:
   *  - convex ground (a crest, the back of a roller, a jump lip). Following it needs a
   *    centripetal acceleration, and past the point where gravity cannot supply it the
   *    tyres carry nothing — you are off the ground. This is the "go neutral over a
   *    crest" the plant derate exists for, moved from after the fact to before it.
   *  - lower-μ ground (gravel, mud, snow). Riding onto it at full lean is a crash the
   *    player could see coming, so the rider stands up on the approach instead.
   *
   * CAVEAT, stated because it bounds what this term can be trusted for: the ray
   * follows the CURRENT velocity, so a bike that is already drifting sideways samples
   * ground it may never ride — at 83 m on the trail, drifting 0.35 m per metre, the
   * far end of an 11.6 m stencil sat 2 m off the tread over a 2.7 m drop-off, and the
   * prediction "you are about to be launched" was self-fulfilling: it removed the lean
   * that was the only way to turn back onto the tread. Two things bound that now — the
   * horizon is short (0.35 s, ≤ 5 m) so the drift has less room to compound, and
   * LEAN_LIMIT_PLANTED means no prediction, right or wrong, can take away the rider's
   * ability to steer while a tyre is on the ground.
   */
  function updateLeanLookahead() {
    const c = collision();
    if (!c || crashed) { lookLoadFrac = 1; lookMuFrac = 1; aNormLook = T.GRAVITY; return; }
    _travel.copy(V); _travel.y = 0;
    const sp = _travel.length();            // HORIZONTAL speed — see the algebra below
    if (sp < 1.5) { lookLoadFrac = 1; lookMuFrac = 1; aNormLook = T.GRAVITY; return; }
    _travel.multiplyScalar(1 / sp);
    const d = clamp(sp * T.LEAN_LOOK_T, T.LEAN_LOOK_MIN, T.LEAN_LOOK_MAX);
    const dx = T.LEAN_LOOK_STEP;
    // Stations 1..n at s = dx .. n·dx; samples 0..n+1 so every station has a
    // symmetric ±dx stencil. n·dx ≈ d.
    const n = Math.min(T.LEAN_LOOK_MAX_N, Math.max(2, Math.round(d / dx)));
    for (let k = 0; k <= n + 1; k++) {
      const s = (k - 1) * dx;
      _lookH[k] = c.heightAt(P.x + _travel.x * s, P.z + _travel.z * s);
    }

    // The normal load a bike riding a heightfield y = f(x) carries, as a FRACTION of
    // what the same slope carries at rest. Parameterise by horizontal distance x:
    //
    //   a·n = f''·ẋ² / √(1+f'²)          (normal component of path acceleration)
    //   N/m = g·cosθ + a·n = (g + f''·ẋ²) / √(1+f'²)
    //   N/m ÷ (g·cosθ)     = 1 + ẋ²·f''/g
    //
    // The √(1+f'²) cancels exactly, so with ẋ the HORIZONTAL speed this is a pure,
    // slope-free statement: > 1 in a compression, 1 on any constant grade however
    // steep, 0 at the point the tyres stop carrying anything, < 0 means the bike has
    // already left the ground. A steepening slope is not an unloading — this is the
    // formula that says so, and it is the same formula the old code used; what was
    // wrong was the scale f'' was measured at and the thresholds it was compared to.
    const kv = sp * sp / T.GRAVITY;
    const maxRise = dx * T.LEAN_LOOK_MAX_SLOPE;
    let worst = 1;
    for (let k = 1; k <= n; k++) {
      // RIDEABILITY GATE. A three-point curvature is only a statement about ground
      // the wheels are going to roll over. Beside a cut tread the ground falls away
      // several metres in one step, and the stencil then reports f'' of −13 1/m — a
      // "crest" of 8 cm radius. That is not a crest, it is the EDGE, and reading it as
      // one is what made the 91 m failure self-fulfilling: the bike drifting towards
      // the edge had its lean taken away, which was the only thing that could have
      // turned it back. So a station straddling a step steeper than a bike could ride
      // is skipped: the derate's business is crests you stay on the ground over, and
      // if you genuinely do launch off an edge, contact goes false and the airborne
      // floor takes over anyway — standing the bike up early buys nothing there.
      const dUp = _lookH[k + 1] - _lookH[k];
      const dDn = _lookH[k] - _lookH[k - 1];
      if (Math.abs(dUp) > maxRise || Math.abs(dDn) > maxRise) continue;
      const f2 = (dUp - dDn) / (dx * dx);
      const frac = 1 + kv * f2;
      if (frac < worst) worst = frac;
    }
    aNormLook = worst * T.GRAVITY;          // published for the HUD / diagnostics
    // FLOORED. Anticipation makes a rider go neutral over a crest; it does not make
    // them abandon a corner they are already committed to. Whatever the terrain ahead
    // says, this term may not take more than (1 − LEAN_LOOK_FLOOR) of the ceiling
    // away — the rest of the job belongs to the present-tense plant term, which is
    // measuring what the tyres are actually carrying rather than guessing.
    lookLoadFrac = Math.max(T.LEAN_LOOK_FLOOR,
      smoothstep(T.LEAN_PLANT_LO, T.LEAN_PLANT_HI, worst));
    if (c.materialAt) {
      const mAhead = surfMu(c.materialAt(P.x + _travel.x * d, P.z + _travel.z * d));
      lookMuFrac = clamp(mAhead / Math.max(0.25, muAvail), 0.3, 1);
    } else {
      lookMuFrac = 1;
    }
  }

  function leanCeiling(h, loadFrac) {
    const capF = lateralCapacity(wheelSim[0]);
    const capR = lateralCapacity(wheelSim[1]);
    const a = T.WHEELBASE * 0.5 - T.CG_FWD;      // CG → front axle
    const b = T.WHEELBASE * 0.5 + T.CG_FWD;      // CG → rear axle
    const nTotal = wheelSim[0].load + wheelSim[1].load;
    // NB the test is CONTACT, not capacity. A rear wheel spinning under power is
    // still on the ground: it has no lateral grip left, and the honest answer is
    // that the bike cannot hold any lean at all — which is what the two-end formula
    // says when capR goes to zero. Branching on capacity instead sent that case to
    // the one-wheel fallback, which reads the FRONT's μ and cheerfully authorised
    // 0.5 rad. From a standing start on the centreline that was a low-side inside
    // 1.3 s: pedal, wheelspin, rear steps out, and the rider is still leaning on it.
    const bothDown = wheelSim[0].contact && wheelSim[1].contact;
    let target;
    if (bothDown && nTotal > 1) {
      // Expressed as an effective friction RATIO, not as an absolute acceleration.
      // That distinction is load-bearing. `lean` is measured against the CONTACT
      // normal, and for a lean relative to the surface the limit is tan(lean) ≤ μ_eff
      // — independent of how hard the surface is pushing. Take the absolute
      // acceleration instead and you get a positive feedback loop: leaning harder
      // loads the tyres, more load reads as more capacity, which permits more lean.
      // Measured: it ran the steady-state lean to 0.78 rad on flat ground where the
      // honest answer is 0.59, and then the corner scrubbed the bike to a standstill.
      // As a ratio, a berm's extra load cancels top and bottom — the world-space lean
      // still rises with the bank, exactly as it should, but the RELATIVE limit does
      // not move.
      const muF = capF * T.WHEELBASE / b;        // front-limited effective μ × N_total
      const muR = capR * T.WHEELBASE / a;        // rear-limited
      target = Math.atan(Math.min(muF, muR) / nTotal * T.LEAN_GRIP_MARGIN * lookMuFrac);
    } else if (wheelSim[0].contact || wheelSim[1].contact) {
      // One wheel only (a manual, an endo, a wheel skipping a root). It carries the
      // whole cornering force on its own, and it has to carry the weight too. Still
      // expressed as a friction ratio, for the same reason as above.
      //
      // NOTE what is NOT here any more: the LEAN_ONE_WHEEL derate. One-wheel contact
      // is a FLICKERING boolean — on real tread the rear skips several times a second
      // — and this branch feeds the asymmetric filter below, which falls 2.5× faster
      // than it rises. An asymmetric filter driven by a flickering input ratchets, and
      // this one did: through the 76-92 m corner the rear skipped at ~4 Hz and each
      // skip knocked ~30% off `leanGrip` against a 5/s recovery, so the grip term sat
      // pinned at 0.34 rad on clean dirt whose honest answer is 0.65. The corner needs
      // 0.53 rad of contact-relative lean; the bike could ask for 0.27. That is the
      // same "asymmetric filter + flickering input" defect that was already found and
      // fixed for the plant derate, still present here.
      // The one-wheel derate is now applied AFTER the filter, off a symmetrically
      // smoothed both-wheels-down fraction, where it cannot ratchet — a sustained
      // manual is derated, a 30 ms skip is not.
      const front = wheelSim[0].contact;
      const cap = front ? capF : capR;
      const load = front ? wheelSim[0].load : wheelSim[1].load;
      target = Math.atan(clamp(cap / Math.max(load, 1), 0, 2)
        * T.LEAN_GRIP_MARGIN * lookMuFrac);
    } else {
      // Airborne, or genuinely no grip: the grip term simply holds its last value.
      // There is no honest grip answer in free flight, and the load derate below
      // already takes the ceiling down to its floor while the wheels carry nothing.
      target = null;
    }

    // ---- grip term: asymmetric smoothing ---------------------------------
    // Falls quickly and rises slowly. Losing grip is something a rider feels
    // immediately; trusting it again takes a moment, and a momentary wheel lift must
    // not ratchet the ceiling upwards.
    if (target !== null) {
      leanGrip = damp(leanGrip, target,
        target < leanGrip ? T.LEAN_LIMIT_LAG * T.LEAN_LIMIT_FALL : T.LEAN_LIMIT_LAG, h);
    }

    // ---- plant derate: a direct multiplier, NOT another asymmetric filter ----
    // This is the "go neutral when you get light" term, and it must be able to come
    // back as fast as it went away. Feeding it through the asymmetric smoothing above
    // was measurably wrong: contact flickers several times a second on real tread, and
    // an asymmetric filter driven by a flickering input ratchets — the ceiling sat
    // pinned near its floor through the rollers, the rider could not command enough
    // lean to generate a turn, and the run left the trail sideways at 20 m/s. The
    // smoothing this term needs lives on its INPUT instead (LEAN_LOAD_LAG), where it
    // is symmetric and cannot ratchet.
    //
    // ---- one-wheel derate, moved out of the ratchet -----------------------
    // Symmetrically smoothed, so it measures how much of the last few tenths of a
    // second the bike has actually been on ONE end — a manual or an endo — rather
    // than reacting to a wheel skipping a root.
    bothFrac = damp(bothFrac, bothDown ? 1 : 0, T.LEAN_BOTH_LAG, h);
    const oneWheelFrac = T.LEAN_ONE_WHEEL + (1 - T.LEAN_ONE_WHEEL) * bothFrac;

    // ---- what leanLimit IS, and what it is NOT ----------------------------
    // It is the lean the grip under the tyres will hold: a friction ratio, times the
    // one-wheel geometry derate, floored so that a bike with a tyre on the ground is
    // never described as unable to steer. It is a statement about the SURFACE, and it
    // is what the over-ceiling test and the low-side logic are judged against.
    //
    // The plant derate is NOT in it any more, and that is the substantive fix. Folding
    // "how loaded are the tyres" into a friction ratio double-counts: the roll
    // pendulum balances tan(lean) = a_lat/a_norm against a tyre limit of
    // a_lat ≤ μ·a_norm, so a_norm cancels and the holdable lean is atan(μ) at 0.4 g
    // exactly as at 1.0 g. Multiplying by a load fraction said "at half load you may
    // use half the coefficient of friction", which then reported a planted bike on a
    // 53% grade as being over a ceiling of 0.16 rad while its tyres were honestly good
    // for 0.65 — and the low-side test fires off that number.
    //
    // Going light is still a real reason to stop asking for lean; it is just a
    // statement about the RIDER, not the surface. So it moved to where it belongs —
    // the commanded lean in groundStep(), via `plantDerate` below. The rider goes
    // neutral over a crest exactly as before (the thresholds are unchanged, and they
    // measure better than loosening them), but the bike is no longer judged against a
    // ceiling that pretends the grip went away with the load.
    const anyContact = wheelSim[0].contact || wheelSim[1].contact;
    const floor = anyContact
      ? Math.min(leanGrip, Math.max(T.LEAN_LIMIT_MIN, T.LEAN_LIMIT_PLANTED))
      : T.LEAN_LIMIT_MIN;
    leanLimit = Math.max(floor, leanGrip * oneWheelFrac);
    // How much of that the rider is willing to ask for, given how loaded the tyres are
    // now (`loadFrac`) and how loaded the ground a third of a second ahead will let
    // them be (`lookLoadFrac`). Taking the smaller is what makes the rider anticipate
    // a crest instead of discovering it.
    plantDerate = Math.min(loadFrac, lookLoadFrac);
    return leanLimit;
  }

  // -------------------------------------------------------------------------
  // Ground attitude: pitch DOF + lean pendulum + yaw + the rider's steer solver.
  //
  // WHY THE LEAN CONTROLLER IS A PLAIN CRITICALLY-DAMPED SERVO
  // ----------------------------------------------------------
  // It used to be a time-optimal rate controller: cap the commanded roll rate at
  // √(2·a_arrest·error) so the rider can always stop the roll where they meant to.
  // That is a correct piece of theory and it is the wrong tool here, because the
  // switching surface it rides on is computed from `lean` and `leanRate`, and
  // `leanRate` is driven by the tyre lateral force, which is driven by the wheel
  // LOAD — and on a rocky chute the loads swing between 0 and 2400 N several times
  // a second. Bang-bang control of a noisy plant chatters at the switching surface
  // by construction: the demand slammed between ±LEAN_TORQUE_MAX, and since the
  // countersteer term is proportional to that demand, the chatter went straight out
  // to the handlebar as a ±0.055 rad square wave.
  //
  // A critically-damped second order servo has no switching surface to chatter on.
  // Rate limiting is applied to the COMMANDED rate (not to the torque), the
  // commanded lean carries an anti-windup clamp so it can never run away from the
  // achievable lean while the tyres are saturated, and the rider's estimate of the
  // tyre force is low-passed — the physics still sees the raw force, only the
  // rider's feed-forward is filtered. Saturation against LEAN_TORQUE_MAX is still
  // exactly the washout: when the tyres under-deliver by more than the rider's body
  // english can make up, the bike keeps falling into the turn, and you can still
  // save it by standing it up in time.
  // -------------------------------------------------------------------------
  function groundStep(h, input, contactCount, loadSum) {
    const steerIn = input ? input.steer : 0;
    const pitchIn = input ? input.pitch : 0;
    const sp = V.length();
    let rollDemand = 0;          // rad/s of commanded roll rate
    let rollAccelDemand = 0;     // rad/s² the rider is asking the bike to roll at

    // ---- pitch -----------------------------------------------------------
    // Torque about the chassis right axis, from the ground + gravity + rider.
    // Ground reaction, impulse-limited — see TAU_PITCH_MAX.
    let tauPitch = clamp(_tauSum.dot(_right), -T.TAU_PITCH_MAX, T.TAU_PITCH_MAX)
      - T.PITCH_DAMP * pitchRate;

    // Deliberate rider pitch authority (manual / endo). pitch input +1 = nose down.
    const manualBoost = (input && input.manual) ? 1.35 : 1.0;
    tauPitch += -pitchIn * T.RIDER_PITCH_TORQUE * manualBoost;

    // Rider composure: hold the bike level with the ground unless the player is
    // deliberately asking for a manual/endo. Torque-limited to what a rider can
    // actually produce by shifting their weight.
    const holdScale = clamp01(1 - Math.abs(pitchIn) * 1.4) * ((input && input.manual) ? 0.15 : 1);
    if (holdScale > 0.01) {
      let hold = T.PITCH_HOLD_KP * (0 - pitch) - T.PITCH_HOLD_KD * pitchRate;
      hold = clamp(hold, -T.PITCH_HOLD_MAX, T.PITCH_HOLD_MAX) * holdScale;
      tauPitch += hold;
    }

    pitchRate += (tauPitch / T.I_PITCH) * h;
    pitchRate = clamp(pitchRate, -T.PITCH_RATE_LIMIT, T.PITCH_RATE_LIMIT);
    pitch += pitchRate * h;
    pitch = clamp(pitch, -1.15, 1.15);

    // ---- lean ------------------------------------------------------------
    // EFFECTIVE GRAVITY. The roll pendulum is not standing in a 1 g field: it is
    // standing on whatever the ground is pushing back with. Through the trough of a
    // roller that is 2.4 g, and in a berm more; over a crest it is a fraction of g.
    // Using the constant 9.81 there is not a small error — it says the lean needs
    // only g·tan(θ) of lateral force to hold when the truth is 2.4·g·tan(θ), so the
    // rider is told they are safely inside the limit while the bike is falling over.
    // Measured: a lean escaping to 1.24 rad in a 2.4 g compression on open, flat,
    // 3 m wide trail with the ceiling reading 0.55.
    //
    // Filtered, and used for BOTH the physics pendulum and the rider's model, so the
    // two can never disagree. It is also exactly consistent with leanCeiling():
    // that returns a friction RATIO μ_eff, and equilibrium here is
    // tan(lean) = a_lat/a_norm ≤ μ_eff. Same statement, same numbers.
    const aNormRaw = (wheelSim[0].load + wheelSim[1].load) / T.MASS;
    const aNormClamped = clamp(contactCount > 0 ? aNormRaw : T.GRAVITY,
      T.GRAVITY * T.LEAN_ANORM_MIN, T.GRAVITY * T.LEAN_ANORM_MAX);
    aNormFilt = damp(aNormFilt, aNormClamped, T.LEAN_BALANCE_LAG, h);
    // CLIP THE PAIR, NOT ONE ARM OF IT. The pendulum balances a RATIO — equilibrium is
    // a_lat = a_norm·tan(lean) — so clipping the normal spike while leaving the lateral
    // alone does not damp the spike, it INVENTS a lean error that never existed.
    // Worked example from a traced low-side: a bottom-out puts 8.59 g through the
    // wheels with 2.40 g of lateral. That is a ratio of 0.28, i.e. a perfectly ordinary
    // 0.27 rad corner. Clamp the normal to LEAN_ANORM_MAX (3 g) and leave the lateral,
    // and the pendulum reads a ratio of 0.80 — a bike 0.67 rad past balance — and flicks
    // it over at 2.9 rad/s. That single inconsistency was the largest remaining source
    // of low-sides on the trail: 34 of 72 crashes, at a mean load spike of 8.6 g.
    // Scaling down only: getting LIGHT must not amplify the lateral arm either.
    const spikeScale = (contactCount > 0 && aNormRaw > 1e-3)
      ? Math.min(1, aNormClamped / aNormRaw) : 1;

    // The commanded lean comes from steering input, scaled in with speed (you
    // cannot lean a stationary bike into a turn), and is capped by the lean the
    // grip actually under the tyres will hold — see leanCeiling().
    const speedFrac = smoothstep(1.0, 7.5, sp);
    // How much of the bike's weight the tyres are ACTUALLY carrying right now. Note
    // this is not aNormFilt: that one is clamped to 1 g whenever there is no contact
    // (it is the pendulum's effective gravity and a free-falling pendulum is not a
    // thing), so reading the load off it said "fully loaded" at the exact moment the
    // wheels were carrying nothing — and the crest derate this feeds never fired in
    // the case it was written for. Filtered with the rider's own reaction lag.
    const aLoadRaw = contactCount > 0 ? (wheelSim[0].load + wheelSim[1].load) / T.MASS : 0;
    aLoadFilt = damp(aLoadFilt, aLoadRaw, T.LEAN_LOAD_LAG, h);
    // Compared against what THIS SLOPE provides at rest (g·cosθ), not against a flat
    // g. On a 53% grade cosθ is 0.88 and every load reads 14% light against the wrong
    // reference; combined with thresholds meant for "nearly airborne" that is enough
    // on its own to start derating a perfectly planted bike. Clamped so a near-vertical
    // contact normal (a wall graze) cannot divide by nothing.
    const aLoadRef = T.GRAVITY * clamp(_groundUp.y, 0.45, 1);
    const loadFrac = smoothstep(T.LEAN_PLANT_LO, T.LEAN_PLANT_HI, aLoadFilt / aLoadRef);
    plantFracCur = loadFrac;
    const leanGripMax = leanCeiling(h, loadFrac);
    const aRollMaxConst = T.LEAN_TORQUE_MAX / (T.MASS * T.ROLL_H * T.ROLL_H);
    const leanSafe = Math.asin(clamp01(aRollMaxConst * T.ROLL_H / T.GRAVITY)) * T.LEAN_SAFE_FRAC;
    const leanCeil = Math.min(leanGripMax, leanSafe);
    // THE CONTROL CONSTRAINT. The rider asks for a fraction of the ceiling, never all
    // of it. Clipping the command at exactly the ceiling leaves nothing to absorb the
    // roll overshoot, the ceiling moving under you, or the servo's own lag, and the
    // measured consequence was 14% of a trail run spent above the ceiling with
    // `lowside` the largest single crash cause. With the reserve, a corner that asks
    // for more lean than the grip allows is answered with less lean — the bike runs
    // wide. That is a mistake the player can see and correct. Falling over is not.
    //
    // The PLANT DERATE lives here, not in the ceiling — see the long note in
    // leanCeiling(). "The tyres are light, stop asking for lean" is a statement about
    // what the rider will commit to, not about what the surface will hold, and putting
    // it in the ceiling meant a planted bike using grip it genuinely had was reported
    // as over its limit and fed to the low-side test. Here it does the job it was
    // written for — the rider goes neutral over a crest — and nothing else.
    //
    // Floored, so no amount of derate can take away the ability to steer while a tyre
    // is on the ground: the situation the derate fires in (getting light, near the
    // edge of the tread) is precisely the situation where the rider MUST be able to
    // turn. The floor is itself capped by the ceiling, so it never authorises lean the
    // surface will not hold.
    const plantFloor = (wheelSim[0].contact || wheelSim[1].contact)
      ? Math.min(leanCeil * T.LEAN_CMD_MARGIN, T.LEAN_LIMIT_PLANTED)
      : 0;
    const leanCmdCeil = Math.max(plantFloor,
      leanCeil * T.LEAN_CMD_MARGIN * plantDerate);
    let target = clamp(steerIn * T.LEAN_MAX * speedFrac, -leanCmdCeil, leanCmdCeil);
    // ANTI-WINDUP. `leanCmd` is an integrator (a rate-limited ramp), and the one
    // failure mode every integrator has is winding out while the plant is saturated.
    // Holding full stick through a corner the tyres cannot hold used to run the
    // command to the stops; the servo then had a huge error to unwind and slammed
    // the bar the other way the moment grip returned. Clamping the command to a
    // bounded lead over the ACHIEVED lean bounds the error the servo ever sees, and
    // therefore bounds the demand — no windup is possible by construction.
    // Note the `min(…, 0)` / `max(…, 0)`: the clamp may only stop the command
    // running further OVER than the bike has got to. Standing the bike up must
    // always be available at full authority — that is the save, and an anti-windup
    // clamp that got in its way would turn every big lean into a crash.
    target = clamp(target,
      Math.min(lean - T.LEAN_CMD_LEAD, 0),
      Math.max(lean + T.LEAN_CMD_LEAD, 0));
    leanCmd += clamp(target - leanCmd, -T.LEAN_CMD_RATE * h, T.LEAN_CMD_RATE * h);
    leanCmd = clamp(leanCmd, -T.LEAN_MAX, T.LEAN_MAX);
    // Hard saturation of the command against the ceiling. The ramp above already
    // walks the command back when the ceiling falls, but a ramp is a lag and a lag is
    // somewhere for an integrator to hide: this makes the constraint an invariant
    // rather than a tendency. It only ever moves the command TOWARDS upright, so the
    // save is never clipped.
    leanCmd = clamp(leanCmd, -leanCmdCeil, leanCmdCeil);

    // Actual lateral acceleration the tyres delivered, in the contact plane
    // (+ = towards the rider's right, i.e. into a right-hand turn).

    // Impulse-limited — see LEAN_ALAT_MAX. The LINEAR integration above already
    // used the full unclipped force; this ceiling only stops a bottom-out spike
    // being integrated into a roll rate.
    const aLatCap = T.LEAN_ALAT_MAX * aNormFilt;
    const aLat = clamp((wheelSim[0].fy + wheelSim[1].fy) / T.MASS * spikeScale,
      -aLatCap, aLatCap);
    // What the RIDER perceives. The physics below uses the raw `aLat`; the rider's
    // feed-forward uses this. Filtering here rather than in the plant is the whole
    // difference between a controller that rejects tyre-load noise and one that
    // differentiates it straight onto the handlebar.
    aLatFilt = damp(aLatFilt, aLat, T.LEAN_BALANCE_LAG, h);

    // Inverted pendulum about the contact line:
    //     I·θ̈ = M·g·h·sin θ  −  M·a_lat·h·cos θ  +  τ_rider,     I = M·h²
    // Gravity topples you further into the lean; the centripetal reaction stands
    // you back up. Equilibrium is exactly a_lat = g·tan(lean), as it should be. If
    // the tyres cannot supply that, the bike keeps falling into the turn — that IS
    // the front washing out, and it is recoverable if you stand it up in time.
    const cosLean = Math.cos(lean);
    const aPend = (aNormFilt * Math.sin(lean) - aLat * cosLean) / T.ROLL_H;
    // Tyre-scrub roll damping — see ROLL_DAMP. Only as much of it as the tyres are
    // loaded enough to provide.
    const rollDamp = T.ROLL_DAMP * clamp01(aLoadFilt / T.GRAVITY);
    let leanAcc = aPend - rollDamp * leanRate;

    // ---- rider roll authority: critically-damped 2nd order + rate limit ----
    const I_ROLL = T.MASS * T.ROLL_H * T.ROLL_H;
    const aRollMax = T.LEAN_TORQUE_MAX / I_ROLL;
    const leanErr = leanCmd - lean;
    // What the rider FEELS the pendulum doing (filtered — see aLatFilt above).
    const aPendFelt = (aNormFilt * Math.sin(lean) - aLatFilt * cosLean) / T.ROLL_H;

    // Rate limit lives on the COMMANDED rate, never on the torque — a rate limit
    // applied to the torque is a bang-bang controller wearing a hat. Three limits,
    // and the third is the one that matters:
    //
    //   ω·e                  critically-damped approach; governs near zero error
    //   LEAN_RATE_MAX        how fast a human can throw a DH bike over
    //   √(2·a_arrest·e)      the roll rate that can still be STOPPED in the error
    //                        that remains, with gravity's help or hindrance counted
    //
    // Drop the third and the servo runs at full rate right up to the target and then
    // needs v²/2a of extra travel to stop — 0.43 rad at 2.6 rad/s, which sails past
    // the grip ceiling and lays the bike down every time. Note this is NOT the old
    // time-optimal controller: that one put the profile on the torque, so the demand
    // slammed between ±LEAN_TORQUE_MAX at the switching surface. Here the profile
    // only shapes the target rate, and a critically-damped PD on a filtered rate
    // follows it smoothly.
    // |aPendFelt|, not the signed value: sign(leanRate)·aPendFelt would flip
    // discontinuously every time the roll rate crosses zero, which is a switching
    // surface — exactly what we are here to avoid. Taking the pessimistic side is
    // continuous and costs only a slightly early arrest.
    // PESSIMISTIC arrest budget: what is left of the rider's roll authority after the
    // pendulum has taken its share.
    //
    // Budgeting on gravity alone — aRollMax − g·|sin θ|/h — is not pessimistic enough,
    // and the gap is where the low-sides came from. Worked example, taken from a
    // traced crash: mid-flick at lean +0.47 with the tyres still carrying the PREVIOUS
    // corner's slip, a_lat is −0.55 g while the bike leans +0.47. The gravity-only
    // budget reads 5.3 rad/s² and authorises a roll rate of 2.7 rad/s. The truth is
    // that the felt pendulum is +9.3 rad/s² against a rider authority of 9.55, so the
    // real net arrest is 0.2 rad/s² — the rider is at full body english and going
    // nowhere. The roll ran from 0.47 to the 1.26 low-side threshold in 0.3 s.
    //
    // So budget on what the rider can FEEL, which is exactly aPendFelt. Note the two
    // properties that make this safe rather than merely conservative:
    //   - in a BALANCED corner a_lat = a_norm·tan θ, so aPendFelt is zero and the
    //     budget is the full authority. Committed cornering is not penalised at all.
    //   - it only collapses when the bike is genuinely falling, which is precisely
    //     when a rider stops adding lean. The behaviour falls out; it is not a rule.
    // |aPendFelt|, not the signed value, for the same reason the note above gives:
    // signing it on the roll direction is a switching surface, and we are here to
    // avoid switching surfaces.
    const arrestBudget = Math.max(T.LEAN_ARREST_MIN,
      aRollMax - Math.max(T.GRAVITY * Math.abs(Math.sin(lean)) / T.ROLL_H,
        Math.abs(aPendFelt)));
    const vArrest = Math.sqrt(2 * arrestBudget * Math.abs(leanErr) * T.LEAN_APPROACH);
    let rate = T.LEAN_WN * Math.abs(leanErr);
    if (rate > T.LEAN_RATE_MAX) rate = T.LEAN_RATE_MAX;
    if (rate > vArrest) rate = vArrest;
    rollDemand = leanErr >= 0 ? rate : -rate;

    // The rider holds the bike: they cancel what the pendulum is doing to them and
    // servo the rest. Saturating against LEAN_TORQUE_MAX is precisely the washout —
    // the rider is at full body english and the bike keeps going over.
    // Split the rider's effort into the part that HOLDS the bike (cancelling the
    // pendulum and the tyre scrub) and the part that ASKS IT TO ROLL (the servo).
    // Only the second is countersteer's business — see below.
    const rollServo = clamp(2 * T.LEAN_ZETA * T.LEAN_WN * (rollDemand - leanRate),
      -aRollMax, aRollMax);
    let riderAcc = -aPendFelt + rollDamp * leanRate + rollServo;
    riderAcc = clamp(riderAcc, -aRollMax, aRollMax);
    // COUNTERSTEER SOURCE. This used to be the whole of `riderAcc`, and that is not a
    // transient: in a committed corner the holding term −aPendFelt carries a large
    // standing value, and it saturates against the torque limit, so the bar carried a
    // standing offset that jittered with it. Measured on a perfectly smooth plane at
    // 15 m/s and |steer| 0.8 — i.e. with the commanded lean sitting on the grip
    // ceiling — that closed a loop at 3.5 Hz: 3.5 bar reversals/s of 0.05 rad. Zeroing
    // COUNTERSTEER removed it entirely and nothing else did, which is the proof.
    // The roll acceleration the rider is ASKING FOR is the servo term alone. In a
    // steady corner rollDemand and leanRate are both zero, so it is exactly zero —
    // which is what "a transient that dies once the lean is established" has to mean.
    rollAccelDemand = rollServo;
    leanAcc += riderAcc;

    // Low speed: the rider just puts a foot down and stands it up. Not physical,
    // but a bike that falls over every time you slow to walking pace is not a game.
    if (sp < T.LOWSPEED_UPRIGHT) {
      const k = 1 - sp / T.LOWSPEED_UPRIGHT;
      leanAcc += (-lean * 26 - leanRate * 9) * k;
    }

    leanRate += leanAcc * h;
    leanRate = clamp(leanRate, -T.LEAN_RATE_LIMIT, T.LEAN_RATE_LIMIT);
    lean += leanRate * h;
    if (lean > 1.45) { lean = 1.45; if (leanRate > 0) leanRate = 0; }
    if (lean < -1.45) { lean = -1.45; if (leanRate < 0) leanRate = 0; }

    // Down. A lean past ~72° with the wheels still nominally in contact is a
    // low-side; there is nothing left to recover with.
    if (Math.abs(lean) > 1.26 && sp > 4) {
      triggerCrash('lowside', sp * 0.55);
      return;
    }

    // ---- rider steer solver ----------------------------------------------
    // Steer is an OUTPUT: the rider steers by whatever it takes to hold the lean
    // they have chosen. The feed-forward below is the complete steady-state
    // single-track solution, not just the kinematic part:
    //
    //     δ = L·a_y/v²  +  α_f − α_r
    //         └ Ackermann ┘  └ what the tyres must actually slip ┘
    //
    // Both terms matter and the second one is usually the bigger. Worked example on
    // this bike at 11.5 m/s and 0.49 rad of lean: Ackermann = +0.0489 rad,
    // α_f − α_r = −0.0370 rad, total +0.0119 rad — and 0.0119 is what the simulation
    // converges to on smooth ground, to within a fifth of a degree. Ship only the
    // Ackermann term and the feed-forward is wrong by 4×, which leaves the entire
    // real steering signal to be found by feedback. That is what the old code did,
    // with a gain of 0.55 on RAW yaw rate — a signal whose parent is the wheel load,
    // which swings 0→2400 N on a chute. The bar became noise with the answer buried
    // in it, and the corrections spent their lives pinned to ±STEER_CORR_MAX.
    //
    // With the feed-forward correct, the feedback gains drop by ~3.5× and everything
    // that reaches the bar is low-passed at STEER_FILT.
    const vSafe = Math.max(sp, 2.2);
    // Same effective gravity as the pendulum — the bar angle must ask for the force
    // that actually holds this lean, not the force that would hold it at 1 g.
    const aLatWanted = aNormFilt * Math.tan(clamp(lean, -1.2, 1.2));
    const yawWanted = -aLatWanted / vSafe;                 // + yaw = turning left
    const aArm = T.WHEELBASE * 0.5 - T.CG_FWD;             // CG → front axle
    const bArm = T.WHEELBASE * 0.5 + T.CG_FWD;             // CG → rear axle

    let delta = T.WHEELBASE * aLatWanted / (vSafe * vSafe);   // Ackermann

    // Slip angles the corner demands of each end. The moment balance about the CG
    // fixes the split: F_f = M·a_y·b/L and F_r = M·a_y·a/L. Inverting the same tyre
    // curve the physics just used means the rider's model and the bike's model can
    // never disagree — which is exactly the disagreement the feedback used to have
    // to paper over.
    const latDemand = T.MASS * Math.abs(aLatWanted);
    // A wheel in the air has no capacity and therefore no answer. Fall back to its
    // static share of the weight rather than to zero: a rider skipping the rear over
    // a root does not suddenly forget how much bar the corner needs, and letting the
    // feed-forward jump by a third of a radian every time a wheel goes light would
    // reintroduce exactly the chatter this rewrite exists to remove.
    const capF = Math.max(lateralCapacity(wheelSim[0]), muAvail * staticF * T.LAT_GRIP * 0.25);
    const capR = Math.max(lateralCapacity(wheelSim[1]), muAvail * staticR * T.LAT_GRIP * 0.25);
    const alphaFwant = slipForForce(latDemand * bArm / T.WHEELBASE, capF);
    const alphaRwant = slipForForce(latDemand * aArm / T.WHEELBASE, capR);
    const understeer = alphaFwant - alphaRwant;
    delta += (aLatWanted >= 0 ? understeer : -understeer);

    // The sideslip this corner is SUPPOSED to run. Feeding back raw β would fight
    // the feed-forward, because a healthy steady corner has a large β by design.
    const betaWant = (aLatWanted >= 0 ? 1 : -1) *
      (bArm * Math.abs(aLatWanted) / (vSafe * vSafe) - alphaRwant);

    // Measured sideslip, and every feedback signal low-passed together.
    let beta = 0;
    if (sp > 2) {
      _tmpA.copy(V);
      beta = Math.atan2(_tmpA.dot(_right), Math.max(Math.abs(_tmpA.dot(_fwd)), 1.5));
    }
    betaFilt = damp(betaFilt, beta, T.STEER_FILT, h);
    yawRateFilt = damp(yawRateFilt, yawRate, T.STEER_FILT, h);

    // Stabilising corrections. `delta` and yaw have OPPOSITE signs (positive steer
    // = right = negative yaw), so to shed excess LEFT yaw you INCREASE delta —
    // hence (yawRate − yawWanted), not the other way round. Getting this backwards
    // turns the rider into a positive-feedback amplifier and the bike weaves itself
    // into the dirt at about 16 m/s.
    const yawErr = yawRateFilt - yawWanted;
    let corr = T.STEER_FB * yawErr * Math.min(1, 9 / vSafe);
    // Sideslip error: the rear has stepped out further than the corner asked for —
    // steer into the slide to catch it.
    corr += T.STEER_BETA * (betaFilt - betaWant) * smoothstep(2, 6, sp);

    // Bounded integrator. Mops up the slow model error the proportional term would
    // otherwise have to hold a standing offset for (load transfer, camber thrust,
    // cross-slope). Its clamp IS its anti-windup, and it is bled off whenever the
    // tyres are past their peak — integrating against a sliding tyre is integrating
    // against a plant that has stopped responding.
    const sliding = Math.max(wheelSim[0].slipN, wheelSim[1].slipN) > 1.0;
    if (sliding || contactCount === 0) {
      steerInt -= steerInt * Math.min(1, 6 * h);
    } else {
      steerInt = clamp(steerInt + T.STEER_KI * yawErr * h, -T.STEER_INT_MAX, T.STEER_INT_MAX);
    }
    corr += steerInt;
    delta += clamp(corr, -T.STEER_CORR_MAX, T.STEER_CORR_MAX);

    // Countersteer: to start rolling INTO a turn you momentarily steer OUT of it,
    // and to stop rolling you steer back in. It is generated by the roll
    // acceleration demand, so it is a transient that dies away once the lean is
    // established. Built from the FILTERED demand: the raw demand saturates against
    // the rider's torque limit and would put a square wave on the bar by itself.
    // Scaled by 1/v because a given bar angle buys far more yaw at speed, and
    // hard-clamped so it can never dominate the steering.
    rollAccelFilt = damp(rollAccelFilt, rollAccelDemand, T.STEER_FILT, h);
    const csScale = smoothstep(3, 9, sp) * Math.min(1, 8 / vSafe);
    delta -= clamp(T.COUNTERSTEER * rollAccelFilt * csScale,
      -T.COUNTERSTEER_MAX, T.COUNTERSTEER_MAX);

    // Actuator: a first-order lag plus a hard rate limit. Hands have a speed limit,
    // and without one a step in the feed-forward is a step at the contact patch.
    const steerLimit = T.STEER_MAX / (1 + T.STEER_SPEED_FALL * sp);
    const steerWant = clamp(delta, -steerLimit, steerLimit);
    let steerNext = damp(steerAngle, steerWant, T.STEER_ACT, h);
    const dMax = T.STEER_RATE_MAX * h;
    steerAngle += clamp(steerNext - steerAngle, -dMax, dMax);

    // ---- yaw -------------------------------------------------------------
    // Single-track bicycle model. Front lateral force at +a yaws you into the turn,
    // rear lateral force at −b yaws you out of it — which is why a sliding rear
    // steps out and a sliding front just runs wide.
    const a = aArm;
    const b = bArm;
    // Impulse-limited for the same reason as the pitch torque above: the tyre force
    // is proportional to the wheel load, and a bottom-out load spike is not a
    // steering input. See TAU_YAW_MAX.
    let tauYaw = clamp(-a * wheelSim[0].fy + b * wheelSim[1].fy,
      -T.TAU_YAW_MAX, T.TAU_YAW_MAX);
    tauYaw -= yawRate * 42;                        // tyre-scrub yaw damping
    yawRate += (tauYaw / T.I_YAW) * h;
    yawRate = clamp(yawRate, -T.YAW_RATE_LIMIT, T.YAW_RATE_LIMIT);
    heading += yawRate * h;

    // ---- front washout ---------------------------------------------------
    const alphaF = Math.abs(wheelSim[0].alpha);
    if (wheelSim[0].contact && alphaF > T.WASH_ANGLE && sp > 6) {
      washTimer += h;
      if (washTimer > T.WASH_TIME) { triggerCrash('washout', sp * 0.6); return; }
    } else {
      washTimer = Math.max(0, washTimer - h * 2.2);
    }

    // ---- manual / endo balance points ------------------------------------
    // These fall out of the geometry: the balance point is where the centre of mass
    // sits directly over the contact patch.
    const balanceUp = Math.atan2(b + 0.0, T.CG_UP + T.WHEEL_RADIUS);
    const balanceDown = Math.atan2(a + 0.0, T.CG_UP + T.WHEEL_RADIUS);
    const frontUp = !wheelSim[0].contact && wheelSim[1].contact;
    const rearUp = !wheelSim[1].contact && wheelSim[0].contact;
    state.wheelieRaw = frontUp ? Math.max(0, pitch) / balanceUp : 0;
    state.endoRaw = rearUp ? Math.max(0, -pitch) / balanceDown : 0;
    state.wheelie = clamp01(state.wheelieRaw);
    state.endo = clamp01(state.endoRaw);
    if (frontUp && pitch > balanceUp + T.LOOP_MARGIN && pitchRate > 0.25) {
      triggerCrash('loopout', 4 + sp * 0.3);
      return;
    }
    if (rearUp && -pitch > balanceDown + T.LOOP_MARGIN && pitchRate < -0.25 && sp > 3) {
      triggerCrash('otb', 4 + sp * 0.5);
      return;
    }

    // ---- rebuild the pose -------------------------------------------------
    buildGroundQuat(_groundUp, heading, pitch, lean, Q);

  }

  // -------------------------------------------------------------------------
  // Air: free rotation with player authority, whips/scrubs, and the assist that
  // brings a whipped bike back in line so it is actually landable.
  // -------------------------------------------------------------------------
  function airStep(h, input) {
    const ramp = smoothstep(0, T.AIR_RAMP, airTime);
    const steerIn = input ? input.steer : 0;
    const pitchIn = input ? input.pitch : 0;
    const rollIn = input ? input.roll : 0;

    _fwd.set(0, 0, -1).applyQuaternion(Q);
    _up.set(0, 1, 0).applyQuaternion(Q);
    _right.crossVectors(_fwd, _up).normalize();

    // Player torques. Yaw is the whip/scrub axis: kick the back out with steer,
    // roll it over with roll, and bring it back before you touch down.
    _tmpA.set(0, 0, 0)
      .addScaledVector(_right, -pitchIn * T.AIR_PITCH_TORQUE)
      .addScaledVector(_fwd, rollIn * T.AIR_ROLL_TORQUE)
      .addScaledVector(_up, -steerIn * T.AIR_YAW_TORQUE);
    W.addScaledVector(_tmpA, ramp * h);

    // Predicted time to touchdown, used both by the whip return and the auto-stab.
    const tLand = predictTimeToLand();

    // Whip return: as the ground comes up, pull the chassis heading back towards
    // the direction of travel. Real riders do exactly this — without it a whip is
    // a guaranteed crash and nobody would ever throw one.
    _travel.copy(V); _travel.y = 0;
    if (_travel.lengthSq() > 4 && tLand < T.WHIP_RETURN_WINDOW) {
      _travel.normalize();
      _tmpB.set(_fwd.x, 0, _fwd.z);
      if (_tmpB.lengthSq() > 1e-4) {
        _tmpB.normalize();
        const cross = _tmpB.x * _travel.z - _tmpB.z * _travel.x;
        const dot = clamp(_tmpB.dot(_travel), -1, 1);
        const yawErr = Math.atan2(cross, dot);
        state.whip = -yawErr;
        const urgency = 1 - clamp01(tLand / T.WHIP_RETURN_WINDOW);
        W.addScaledVector(_WORLD_UP, (-yawErr * 1.3 - W.dot(_WORLD_UP) * 1.5) * T.WHIP_RETURN * urgency * h);
      }
    } else if (_travel.lengthSq() > 4) {
      _travel.normalize();
      _tmpB.set(_fwd.x, 0, _fwd.z);
      if (_tmpB.lengthSq() > 1e-4) {
        _tmpB.normalize();
        const cross = _tmpB.x * _travel.z - _tmpB.z * _travel.x;
        state.whip = -Math.atan2(cross, clamp(_tmpB.dot(_travel), -1, 1));
      }
    }

    // Anti-frustration auto-stabilisation: with no input, gently level the bike
    // towards the surface it is about to land on.
    //
    // The deadzone is over the ROLL and PITCH inputs only. It used to be the sum of
    // all three, which meant any steer at all switched the levelling off — and steer
    // in the air is the whip/scrub axis, so a rider who was mid-corner when the
    // ground fell away (i.e. exactly the rider who needs levelling) never got it.
    // They launched at 45°+ of lean, held their line, and landed off-axis every
    // time. Whipping is still yours: the yaw torque above is untouched, and roll
    // input still passes through with no levelling at all.
    const inputMag = Math.abs(pitchIn) + Math.abs(rollIn);
    if (T.ASSIST > 0 && inputMag < T.AIR_STAB_DEADZONE) {
      _tmpB.crossVectors(_up, _landNormal);        // torque axis that levels us
      const err = _tmpB.length();
      if (err > 1e-4) {
        _tmpB.multiplyScalar(1 / err);
        const strength = T.ASSIST * T.AIR_AUTO_STAB * Math.min(1, err) * smoothstep(0.15, 0.5, airTime);
        W.addScaledVector(_tmpB, strength * h);
        // Damp the residual so it settles rather than oscillating.
        W.multiplyScalar(Math.exp(-1.5 * h));
      }
    }

    // Bleed off residual roll the player did not ask for (see AIR_ROLL_KEEP).
    // Deliberate roll input passes straight through untouched.
    if (Math.abs(rollIn) < T.AIR_ROLL_DEADZONE) {
      const rollRate = W.dot(_fwd);
      W.addScaledVector(_fwd, -rollRate * Math.min(1, T.AIR_ROLL_DAMP * h));
    }

    W.multiplyScalar(Math.exp(-T.AIR_DRAG_ROT * h));
    state.airRotation += W.dot(_right) * h;

    const wl = W.length();
    if (wl > 1e-5) {
      _tmpA.copy(W).multiplyScalar(1 / wl);
      _qSpin.setFromAxisAngle(_tmpA, wl * h);
      Q.premultiply(_qSpin).normalize();
    }
  }

  /** Ballistic estimate of how long until the wheels reach the ground. */
  function predictTimeToLand() {
    const c = collision();
    if (!c) return 9;
    let t = 0;
    for (let i = 0; i < 5; i++) {
      t = i * 0.12;
      _predPos.set(
        P.x + V.x * t,
        P.y + V.y * t - 0.5 * T.GRAVITY * t * t,
        P.z + V.z * t);
      const h = c.heightAt(_predPos.x, _predPos.z);
      if (_predPos.y - h < T.WHEEL_RADIUS + 0.15) return t;
    }
    return 0.6;
  }

  /** Where (and on what) are we about to land? Feeds the auto-stabiliser. */
  function predictLandingNormal() {
    const c = collision();
    if (!c) return;
    let t = 0;
    for (let i = 1; i <= 8; i++) {
      t = i * 0.11;
      _predPos.set(
        P.x + V.x * t,
        P.y + V.y * t - 0.5 * T.GRAVITY * t * t,
        P.z + V.z * t);
      const h = c.heightAt(_predPos.x, _predPos.z);
      if (_predPos.y - h < T.WHEEL_RADIUS) break;
    }
    c.normalAt(_predPos.x, _predPos.z, _landNormal);
  }

  /**
   * Landing evaluation. Matching the bike's attitude to the landing preserves
   * speed; casing it (landing short/flat, or badly off axis) costs speed or puts
   * you down. The normal component of the impact is absorbed by the suspension
   * automatically — what we add here is the *scrub* penalty for arriving crooked.
   * @returns true if this landing was a crash.
   */
  function evaluateLanding() {
    _fwd.set(0, 0, -1).applyQuaternion(Q);
    _up.set(0, 1, 0).applyQuaternion(Q);
    _n.copy(_groundUp);

    const axisErr = Math.acos(clamp(_up.dot(_n), -1, 1));
    const vN = Math.max(0, -V.dot(_n));
    const sp = V.length();

    // Pitch match: is the bike pointing along the surface it is arriving on?
    _travel.copy(V).addScaledVector(_n, -V.dot(_n));
    let pitchMatch = 1;
    if (_travel.lengthSq() > 0.5) {
      _travel.normalize();
      _tmpA.copy(_fwd).addScaledVector(_n, -_fwd.dot(_n));
      if (_tmpA.lengthSq() > 1e-6) {
        _tmpA.normalize();
        pitchMatch = clamp01(_tmpA.dot(_travel));
      }
    }
    const quality = clamp01(Math.cos(Math.min(axisErr, Math.PI * 0.5)) * pitchMatch);

    // Off-axis crash — but only for something that was actually a jump. Skipping a
    // wheel over a root for 0.15 s while leaning through a berm is not a landing,
    // and judging it as one makes every hard corner with a bump in it fatal.
    if (axisErr > T.CRASH_AXIS && vN > 1.8 && airTime > T.LAND_MIN_AIR) {
      triggerCrash('offaxis', vN + sp * 0.3);
      return true;
    }
    if (vN > T.CRASH_VN) { triggerCrash('case', vN); return true; }

    const cased = vN > T.CASE_VN && quality < T.CASE_QUALITY;
    // Longitudinal scrub from a mismatched landing, on top of whatever the
    // suspension and tyres take out on their own.
    const loss = sp * T.LAND_MISMATCH * (1 - quality) + vN * T.LAND_VN_LOSS * (1 - quality * 0.6);
    if (loss > 0.02 && sp > 0.1) {
      const scale = Math.max(0, 1 - loss / sp);
      V.multiplyScalar(scale);
    }

    state.landQuality = quality;
    state.lastLanding.airTime = airTime;
    state.lastLanding.quality = quality;
    state.lastLanding.vNormal = vN;
    state.lastLanding.axisError = axisErr;
    state.lastLanding.whip = state.whip;
    state.lastLanding.cased = cased;

    if (vN > T.IMPACT_MIN) noteImpact(vN, P, _n, state.surface);
    if (airTime > 0.28) {
      evLanded.airTime = airTime;
      evLanded.quality = quality;
      evLanded.vNormal = vN;
      evLanded.whip = state.whip;
      evLanded.cased = cased;
      evLanded.position.copy(P);
      emit('bike:landed', evLanded);
    }
    state.whip = 0;
    return false;
  }

  // -------------------------------------------------------------------------
  // Hard constraints: never let a wheel end a substep inside the terrain, and
  // never let the chassis pass through a wall.
  // -------------------------------------------------------------------------
  function resolvePenetration(h) {
    const c = collision();
    if (!c) return;
    _fwd.set(0, 0, -1).applyQuaternion(Q);
    _up.set(0, 1, 0).applyQuaternion(Q);
    _fwdH.set(_fwd.x, 0, _fwd.z);
    if (_fwdH.lengthSq() < 1e-6) _fwdH.set(0, 0, -1);
    _fwdH.normalize();

    // IMPORTANT: we do NOT push the chassis out for every millimetre of overlap.
    // Between substeps the bike travels up to ~9 cm, so on rising ground the wheel
    // is routinely a centimetre "into" the terrain at the end of a substep — and
    // taking that up is precisely the suspension's job. What we resolve here is
    // only the part the suspension has no travel left for. Without this
    // distinction the constraint would fight the springs and the bike would skate
    // over bumps instead of absorbing them.
    let worst = 0;
    for (let i = 0; i < 2; i++) {
      const su = susp[i];
      _axis.copy(_up).multiplyScalar(-su.cosR).addScaledVector(_fwd, su.sinR).normalize();
      _mount.copy(P).addScaledVector(_up, su.mountUp).addScaledVector(_fwd, su.mountFwd);
      _axleExt.copy(_mount).addScaledVector(_axis, su.restLen - su.comp);
      c.sweepWheel(_axleExt, T.WHEEL_RADIUS, hits[i], _fwdH);
      if (hits[i].depth <= 0) continue;
      const dn = Math.max(0.30, -_axis.y);
      // Extra compression this overlap would need, on top of what we already have.
      const totalNeeded = su.comp + hits[i].depth / dn;
      const excess = totalNeeded - su.max;
      if (excess > 0) {
        const lift = excess * dn;
        if (lift > worst) { worst = lift; _n.copy(hits[i].normal); }
      }
    }
    if (worst > 0.002) {
      // The sweep depth is a vertical measure, so a vertical lift resolves it
      // exactly. Then remove the inbound normal velocity so we cannot accumulate
      // energy against the constraint.
      P.y += worst;
      const vn = V.dot(_n);
      if (vn < 0) {
        V.addScaledVector(_n, -vn * 1.02);
        if (-vn > T.CRASH_VN * 1.25) triggerCrash('slam', -vn);
      }
    }

    // Absolute floor — if anything above went wrong, do not let the bike escape
    // below the heightfield.
    const gh = c.heightAt(P.x, P.z);
    if (P.y < gh + 0.06) { P.y = gh + 0.06; if (V.y < 0) V.y = 0; }
  }

  function wallStep(h) {
    const c = collision();
    if (!c || crashed) return;
    _fwd.set(0, 0, -1).applyQuaternion(Q);
    _up.set(0, 1, 0).applyQuaternion(Q);
    _right.crossVectors(_fwd, _up).normalize();
    // Body sphere sitting where the rider's hips are.
    _tmpA.copy(P).addScaledVector(_up, 0.78);
    if (!c.sphereCast(_tmpA, 0.52, bodyHit)) return;
    if (bodyHit.depth <= 0) return;
    _n.copy(bodyHit.normal);
    // Only a steep face counts as a wall; anything flatter is just ground the
    // wheels are already dealing with.
    if (_n.y > T.WALL_NORMAL_Y) return;

    _tmpB.set(_n.x, 0, _n.z);
    const nl = _tmpB.length();
    if (nl < 1e-4) return;
    _tmpB.multiplyScalar(1 / nl);
    const into = -V.dot(_tmpB);
    P.addScaledVector(_tmpB, bodyHit.depth * 0.6 + 0.005);
    if (into > 0) {
      V.addScaledVector(_tmpB, into * 1.15);      // stop, with a touch of rebound
      if (into > T.CRASH_WALL_V) {
        triggerCrash('wall', into);
      } else if (into > T.IMPACT_MIN) {
        noteImpact(into, bodyHit.point, _n, bodyHit.material);
        // Glancing hits scrub speed and unsettle the bike rather than ending the run.
        V.multiplyScalar(1 - clamp01(into / T.CRASH_WALL_V) * 0.28);
        yawRate += (Math.sign(_tmpB.dot(_right)) || 1) * into * 0.12;
      }
    }
  }

  // =========================================================================
  // Per-frame derived state, trail relationship, and event flushing.
  // =========================================================================
  function refreshDerived(dt) {
    _fwd.set(0, 0, -1).applyQuaternion(state.quaternion);
    _up.set(0, 1, 0).applyQuaternion(state.quaternion);
    _right.crossVectors(_fwd, _up).normalize();
    state.forward.copy(_fwd);
    state.up.copy(_up);
    state.right.copy(_right);

    state.velocity.copy(V);
    state.speed = V.length();
    state.angularVelocity.copy(airborne || crashed ? W : _omega);

    state.airborne = airborne;
    state.inAir = airborne;
    state.airTime = airTime;
    if (airborne || crashed) {
      // Manual/endo are ground moves; leaving them latched at their last value
      // would have the rider animating a wheelie mid-backflip.
      state.wheelie = 0; state.endo = 0; state.wheelieRaw = 0; state.endoRaw = 0;
    } else if (!airborne) {
      state.whip = 0;
    }
    state.crashed = crashed;
    state.crashTimer = crashTimer;
    state.lean = lean;
    state.leanRate = leanRate;
    state.steer = steerAngle;
    state.pitch = pitch;
    state.pitchRate = pitchRate;
    state.yawRate = yawRate;
    state.gForce = gForceSmooth;
    state.gripAvail = muAvail;
    // The lean the grip actually under the tyres will hold — not atan(μ). See
    // leanCeiling(). The HUD and the rider read this.
    state.leanLimit = leanLimit;
    state.leanGrip = leanGrip;
    state.plantFrac = plantFracCur;
    state.plantDerate = plantDerate;
    state.lookPlantFrac = lookLoadFrac;
    state.aNormLook = aNormLook;
    state.riderFore = riderFore;
    state.riderCrouch = crouch;
    state.riderPumpForce = riderForce;
    state.pumpDrive = clamp(-riderForce / T.PUMP_FORCE_MAX, -1, 1);
    state.pumpImpulse = clamp01(Math.abs(riderForce) / T.PUMP_FORCE_MAX);
    state.pumpWindow = pumpWindow;

    state.suspension.fork.travel = susp[0].comp;
    state.suspension.fork.velocity = susp[0].rateSmooth;
    state.suspension.shock.travel = susp[1].comp;
    state.suspension.shock.velocity = susp[1].rateSmooth;

    // Wheel axle positions from the *interpolated* pose so the model is smooth.
    for (let i = 0; i < 2; i++) {
      const su = susp[i];
      _axis.copy(_up).multiplyScalar(-su.cosR).addScaledVector(_fwd, su.sinR).normalize();
      _mount.copy(state.position).addScaledVector(_up, su.mountUp).addScaledVector(_fwd, su.mountFwd);
      state.wheels[i].position.copy(_mount).addScaledVector(_axis, su.restLen - su.comp);
    }

    const rearMat = state.wheels[1].contact ? state.wheels[1].material
      : (state.wheels[0].contact ? state.wheels[0].material : state.surface);
    state.surface = rearMat;
    state.roughness = SURF_ROUGH[rearMat] !== undefined ? SURF_ROUGH[rearMat] : 0.3;

    // Skid intensity: how far past the friction peak either tyre is, weighted by
    // load so a nearly-unweighted wheel does not scream.
    let skid = 0;
    for (let i = 0; i < 2; i++) {
      const ws = wheelSim[i];
      if (!ws.contact) continue;
      const over = clamp01((ws.slipN - T.SKID_THRESHOLD) / 1.5);
      skid = Math.max(skid, over * clamp01(ws.load / 300));
    }
    skidSmooth = damp(skidSmooth, skid, 14, dt);
    state.skid = skidSmooth;
  }

  /**
   * Where we are on the course.
   *
   * TELEPORT GUARD. `trail.nearestT` answers "which point of the ribbon is closest",
   * and on a mountain with stacked switchbacks that question is genuinely ambiguous:
   * a bike that falls off the side and lands beside a lower leg is nearest to a
   * station a few hundred metres further down the run, so the raw query hands back a
   * progress jump the rider never rode. gameplay drives checkpoints, splits and the
   * HUD progress bar off this, and `respawn()` picks its checkpoint from it — so a
   * teleport does not just look wrong, it respawns you past the section you fell off.
   *
   * The bike cannot advance along the trail faster than it is moving, so anything
   * larger than that is rejected and `state.trailT` holds. Held time is published as
   * `state.trailTValid === false` and announced once as `bike:trailJump`, so gameplay
   * can decide what a fallen rider deserves rather than being quietly lied to.
   *
   * RESIDUAL, stated rather than hidden: after TRAIL_RESYNC seconds of sustained
   * disagreement the guard gives up and accepts the query, because a guard that can
   * wedge progress permanently would be a worse bug than the one it fixes. So a rider
   * who genuinely ends up beside a lower leg and stays there still gets the jump,
   * 1.5 s late and flagged. Removing that last case needs `nearestT` to disambiguate
   * by along-track continuity rather than by distance, which is trail.js's to fix.
   */
  function updateTrail() {
    const trail = ctx.trail;
    if (!trail || !trail.nearestT) return;
    // The hint keeps this a local search — passing it is mandatory, a global
    // search here costs more than the entire physics step.
    const near = trail.nearestT(state.position, state.trailT);
    if (!near) return;
    const len = trail.length || 1;
    const dtF = Math.max(1e-4, lastFrameDt);
    // Metres of trail the bike could plausibly have covered since the last query,
    // plus slack for the arc-length/chord difference through a tight radius.
    const maxStep = state.speed * dtF * 1.5 + 0.75;
    const stepped = Math.abs(near.t - trailTGuard) * len;

    if (trailTSeed || stepped <= maxStep) {
      trailTGuard = near.t;
      trailTSeed = false;
      trailJumpTimer = 0;
      state.trailTValid = true;
    } else {
      trailJumpTimer += dtF;
      state.trailTValid = false;
      if (!trailJumpFlagged) {
        trailJumpFlagged = true;
        evTrailJump.from = trailTGuard;
        evTrailJump.to = near.t;
        evTrailJump.gap = (near.t - trailTGuard) * len;
        emit('bike:trailJump', evTrailJump);
      }
      if (trailJumpTimer > T.TRAIL_RESYNC) {
        // Give up rather than wedge progress forever — but only after announcing it.
        trailTGuard = near.t;
        trailJumpTimer = 0;
        state.trailTValid = true;
      }
    }
    if (state.trailTValid) trailJumpFlagged = false;

    state.trailT = trailTGuard;
    state.lateral = near.lateral;
    const w = (trail.widthAt ? trail.widthAt(state.trailT) : 2.2) || 2.2;
    // While the guard is rejecting, the bike is by definition not where the query
    // says it is, so it is off the course whatever `near.lateral` reads.
    state.offTrail = !state.trailTValid || Math.abs(near.lateral) > w * 0.5 + T.OFFTRAIL_MARGIN;
    state.distance = state.trailT * len;
  }

  /** Pump window: is the ground ahead about to compress? Feeds the HUD and rider. */
  function updatePumpWindow() {
    const c = collision();
    if (!c || !profileOut) { pumpWindow = 0; return; }
    _travel.copy(V); _travel.y = 0;
    if (_travel.lengthSq() < 1) { pumpWindow = 0; return; }
    _travel.normalize();
    c.profile(state.position.x, state.position.z, _travel.x, _travel.z, T.PUMP_LOOKAHEAD, profileOut);
    // Concave ground ahead (a compression / the back of a roller) is where a pump
    // pays. Convex (a crest) is where unweighting pays. Report the magnitude of the
    // opportunity; the sign lives in state.pumpGain.
    const curve = profileOut.curvature;
    pumpWindow = clamp01(Math.abs(curve) / 0.09) * clamp01(state.speed / 6);
  }

  function updateWater(dt) {
    const water = ctx.water;
    const terrain = ctx.terrain;
    let waterY = null;
    if (water && typeof water.heightAt === 'function') {
      const wy = water.heightAt(state.position.x, state.position.z);
      if (wy !== null && wy !== undefined && isFinite(wy)) waterY = wy;
    }
    if (waterY === null && terrain && terrain.creekLevel !== undefined && ctx.trail) {
      // No water module yet: only trust the global creek level where the trail says
      // there is actually a creek, otherwise every low-lying square metre "splashes".
      const s = ctx.trail.sampleAt ? ctx.trail.sampleAt(state.trailT) : null;
      if (s && s.feature && s.feature.type === 'creekCrossing') waterY = terrain.creekLevel;
    }
    const rearY = state.wheels[1].contactPoint.y;
    const wasIn = state.inWater;
    state.inWater = waterY !== null && rearY < waterY + 0.05;

    splashTimer -= dt;
    if (state.inWater && state.speed > T.SPLASH_MIN_SPEED && (splashTimer <= 0 || !wasIn)) {
      splashTimer = 0.09;
      evSplash.position.copy(state.wheels[1].contactPoint);
      evSplash.velocity.copy(V);
      emit('water:splash', evSplash);
    }
  }

  function flushEvents(dt) {
    if (pendImpact > T.IMPACT_MIN) {
      evImpact.severity = clamp01((pendImpact - T.IMPACT_MIN) / (T.IMPACT_FULL - T.IMPACT_MIN));
      evImpact.position.copy(pendImpactPos);
      evImpact.normal.copy(pendImpactNrm);
      evImpact.surface = pendImpactSurf;
      emit('bike:impact', evImpact);
      if (ctx.input && ctx.input.rumble && evImpact.severity > 0.12) {
        ctx.input.rumble(evImpact.severity, 60 + evImpact.severity * 160);
      }
      pendImpact = 0;
    }

    skidEmitTimer -= dt;
    if (state.skid > 0.08) {
      if (skidEmitTimer <= 0) {
        skidEmitTimer = 1 / 14;
        evSkid.intensity = state.skid;
        evSkid.surface = state.surface;
        emit('bike:skid', evSkid);
      }
    } else if (skidEmitTimer < 0 && evSkid.intensity > 0) {
      evSkid.intensity = 0;
      evSkid.surface = state.surface;
      emit('bike:skid', evSkid);
      skidEmitTimer = 0.25;
    }
  }

  // =========================================================================
  // update() — accumulator, substeps, interpolation.
  // =========================================================================
  function update(dt, c) {
    const context = c || ctx;
    const d = clamp(dt || 0, 0, 1 / 20);
    lastFrameDt = Math.max(1e-4, d);
    if (!ensureScratch()) return;

    const input = (context.input && context.input.state) || null;
    if (input && input.reset) { reset(null); return; }

    const gp = context.gameplay;
    const mode = gp && gp.state;
    const frozen = mode === 'paused' || mode === 'menu' || mode === 'finished';

    if (frozen || d <= 0) {
      refreshDerived(Math.max(d, 1 / 240));
      return;
    }

    // Countdown: hold the bike on the gate with both brakes locked.
    const holdInput = (mode === 'countdown') ? COUNTDOWN_INPUT : input;

    // Publish the input the SIMULATION is actually using, not what the player is
    // pressing — during the countdown those differ, and bikeModel drives the brake
    // calipers from these.
    if (holdInput) {
      state.brakeFront = holdInput.brakeFront;
      state.brakeRear = holdInput.brakeRear;
      state.pedalling = holdInput.pedal;
    }

    // The predictive half of the lean ceiling, refreshed BEFORE the substeps so this
    // frame's physics uses this frame's view of the ground ahead.
    updateLeanLookahead();

    accumulator += d;
    const H = T.SUBSTEP;
    const maxAcc = H * T.MAX_SUBSTEPS;
    if (accumulator > maxAcc) accumulator = maxAcc;   // drop time rather than spiral

    let steps = 0;
    while (accumulator >= H && steps < T.MAX_SUBSTEPS) {
      P0.copy(P1); Q0.copy(Q1);
      substep(H, holdInput);
      P1.copy(P); Q1.copy(Q);
      accumulator -= H;
      steps++;
    }
    if (steps === 0) {
      // No substep this frame (very high frame rate) — keep the endpoints valid.
      P0.copy(P1); Q0.copy(Q1);
    }

    // Interpolate the render transform between the last two substeps.
    const alpha = clamp01(accumulator / H);
    state.position.lerpVectors(P0, P1, alpha);
    state.quaternion.copy(Q0).slerp(Q1, alpha);
    if (!isFinite(state.position.x)) state.position.copy(P);

    // Re-predict the landing surface once a frame while in the air. A long jump can
    // cross very different ground, and both the auto-stabiliser and the landing
    // evaluation aim at this normal.
    if (airborne && !crashed) predictLandingNormal();

    refreshDerived(d);
    updateTrail();
    updatePumpWindow();
    updateWater(d);
    flushEvents(d);

    // Out of the world entirely — put them back on the trail.
    const terrain = context.terrain;
    if (terrain && terrain.inBounds && !terrain.inBounds(P.x, P.z)) respawn();
  }

  // A frozen "both brakes on" input used during the countdown.
  const COUNTDOWN_INPUT = {
    steer: 0, pitch: 0, roll: 0,
    brakeFront: 1, brakeRear: 1,
    pedal: 0, pump: 0, manual: false,
    reset: false, pause: false, cameraCycle: false, photoMode: false,
    anyPressed: false,
  };

  // Respawn when gameplay tells us to (it owns the time penalty).
  if (ctx.events && ctx.events.on) {
    ctx.events.on('run:respawn', () => respawn());
  }

  reset(null);

  return {
    state,
    reset,
    update,
    respawn,

    // Geometry other modules need (bikeModel especially) — published so nobody has
    // to guess where the axles and the chassis origin are.
    geometry: {
      wheelbase: T.WHEELBASE,
      wheelRadius: T.WHEEL_RADIUS,
      chassisOrigin: 'axle-midpoint',
      frontAxleOffset: T.WHEELBASE * 0.5,     // metres forward of state.position
      rearAxleOffset: -T.WHEELBASE * 0.5,
      forkMountUp: susp[0].mountUp,
      forkMountFwd: susp[0].mountFwd,
      forkRake: T.FORK_RAKE,
      forkLen: T.FORK_LEN,
      forkTravel: T.FORK_TRAVEL,
      shockMountUp: susp[1].mountUp,
      shockMountFwd: susp[1].mountFwd,
      shockRake: T.SHOCK_RAKE,
      shockLen: T.SHOCK_LEN,
      shockTravel: T.SHOCK_TRAVEL,
      bbUp: 0.0,                              // bottom bracket ≈ axle height − 25 mm
      bbFwd: -0.06,
      cgUp: T.CG_UP,
    },

    // ---- the supported way to move the bike from outside --------------------
    // `bike.state` is read-only (see the CONTRACT-NOTE at the top of this file).
    // These write the authoritative sim state and immediately re-publish, so the
    // caller sees the change on `state` straight away instead of having it silently
    // reverted at the end of the next update().
    setVelocity(v) {
      if (!v || !isFinite(v.x) || !isFinite(v.y) || !isFinite(v.z)) return;
      V.copy(v);
      if (V.lengthSq() > T.SPEED_CAP * T.SPEED_CAP) V.setLength(T.SPEED_CAP);
      state.velocity.copy(V);
      state.speed = V.length();
    },
    addVelocity(dv) {
      if (!dv || !isFinite(dv.x) || !isFinite(dv.y) || !isFinite(dv.z)) return;
      V.add(dv);
      if (V.lengthSq() > T.SPEED_CAP * T.SPEED_CAP) V.setLength(T.SPEED_CAP);
      state.velocity.copy(V);
      state.speed = V.length();
    },
    setPosition(p) {
      if (!p || !isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) return;
      P.copy(p); P0.copy(p); P1.copy(p);   // kill the interpolation history too,
      state.position.copy(p);              // or the render pose lerps across the jump
    },
    setOrientation(q) {
      if (!q || !isFinite(q.x) || !isFinite(q.w)) return;
      Q.copy(q).normalize(); Q0.copy(Q); Q1.copy(Q);
      state.quaternion.copy(Q);
    },
    teleport(p, q) {
      this.setPosition(p);
      if (q) this.setOrientation(q);
      W.set(0, 0, 0);
      state.angularVelocity.set(0, 0, 0);
    },

    // Exposed so a debug panel (or a "pro mode" toggle) can retune live.
    tuning: T,
    setAssist(v) { T.ASSIST = clamp01(v); },

    get WHEEL_RADIUS() { return T.WHEEL_RADIUS; },
    get WHEELBASE() { return T.WHEELBASE; },
  };
}
