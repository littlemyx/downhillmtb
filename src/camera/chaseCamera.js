// =============================================================================
// DESCENT — chaseCamera.js   (CONTRACT §6 "src/camera/chaseCamera.js")
//
// The camera is the whole game as far as the player is concerned, so this module
// is deliberately not a lerp-and-lookAt. The structure is:
//
//   1. Every mode is a *stateless solver* that writes a "pose" — a desired eye
//      position, a desired look-at point, a roll angle, a FOV, and the spring
//      constants that pose wants to be followed with.
//   2. Mode changes blend two solved poses with an eased weight (~0.6 s), so all
//      the follow state lives in exactly one set of springs and there is never a
//      hand-off discontinuity to debug.
//   3. The blended pose is constrained against the terrain (pushed out along the
//      surface normal, and the bike→camera segment swept so a berm or a rock
//      never gets between the player and the bike).
//   4. One critically-damped spring per axis then follows it, with target-velocity
//      feed-forward so the camera does not rubber-band under acceleration.
//   5. Handheld noise, impact shake and roll are layered on last, in that order,
//      so they never feed back into the follow solution.
//
// CONTRACT-NOTE (camera → menu/gameplay): `setMode()` accepts the five contract
//   modes. `ctx.chaseCamera.mode` is a live getter (menu.js reads it). The
//   in-game cycle key walks only the three *rideable* cameras
//   ('chase' → 'chaseFar' → 'firstPerson'); 'cinematic' and 'replay' are
//   director cameras that gameplay/menu select explicitly — cycling into them
//   mid-run would take control away from the player. Cycling out of a director
//   camera returns to 'chase'.
//
// CONTRACT-NOTE (camera → hud/menu): photo mode is owned here, because the free-fly
//   camera *is* photo mode. On toggle this module sets `ctx.photoMode` (hud.js
//   already defers to that boolean), mirrors it into `ctx.settings.photoMode` and
//   emits `photo:mode` `{ enabled }` — the same event menu.js emits — so the HUD
//   hides either way. It also listens for that event so the menu's own photo
//   toggle stays in sync, and honours `ctx.settings.invertLook`.
//
// CONTRACT-NOTE (camera → vegetation): there is no contractual query for "is there
//   a tree trunk here", so trunk occlusion is solved with `terrain.sampleCarve(x,z)`
//   — vegetation excludes trees wherever the trail carved the ground, so carve is a
//   free, exact, allocation-free proxy for "this spot is inside the cleared
//   corridor". If vegetation ever grows an occluder query, swap the body of
//   `corridorClearance()`; nothing else needs to change.
//
// CONTRACT-NOTE (camera → qa/harness.js, r3 review Lane G): the review set is *not* shot
//   through this module — `qa/harness.js:freezeCamera()` stubs out `lateUpdate` and poses
//   `ctx.camera` directly, so none of the framing work below can reach a review screenshot
//   on its own. The framing solver is therefore also exposed as two pure functions that
//   take no ownership of the camera and mutate nothing:
//
//     ctx.chaseCamera.solveHeroShot({ t, back, up, side, ahead, fov, azimuth, fill })
//       -> { position, lookAt, fov, pushedBack, liftedOverCrest, heroScreen }
//     ctx.chaseCamera.solveEstablishingShot({ fov })
//       -> { position, lookAt, fov }
//
//   Both return a shared, reused object — copy out of it immediately. The harness owner
//   (Lane K) can adopt them with three lines per shot:
//       const p = d.chaseCamera.solveHeroShot({ t: 0.40, fov: 42, azimuth: 22, fill: 0.62 });
//       d.camera.position.copy(p.position);
//       d.camera.lookAt(p.lookAt);  d.camera.fov = p.fov; d.camera.updateProjectionMatrix();
//   `solveHeroShot` runs the identical containment, crest-lift and lead code the gameplay
//   camera runs, so a shot that passes in the harness passes in the game and vice versa.
//
// CONTRACT-NOTE (camera → bike): reads the additive `state.roughness`, `state.whip`,
//   `state.lean`, `state.gForce`, `state.airTime`, `state.trailT`, `state.lateral`
//   and `state.offTrail` fields that bike.js publishes. All are read defensively;
//   the camera degrades to a plain follow if the bike is missing entirely.
//
// Allocation discipline: every vector, quaternion and matrix used per frame is
// preallocated at module scope or inside the closure. Nothing in lateUpdate()
// allocates. `trail.sampleAt()`/`nearestT()` return *shared* objects, so their
// results are copied out immediately (see readTrail()).
// =============================================================================

import * as THREE from 'three';
import { createNoise2D } from 'simplex-noise';
import { makeRng, subSeed, clamp, clamp01, lerp, smoothstep, damp } from '../core/rng.js';

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------

const MAX_DT = 1 / 20;

// Speed used to normalise everything that "scales with speed". A DH bike on this
// trail tops out around 22–24 m/s (≈ 82 km/h), so 23 puts the top of every curve
// exactly where the fast sections live rather than somewhere you never reach.
const SPEED_REF = 23;

const MODE_BLEND = 0.60;        // seconds, CONTRACT §6
const CRASH_BLEND_IN = 0.45;
const CRASH_BLEND_OUT = 0.55;

// Ground clearance the camera is never allowed inside, per mode family.
const CLEAR_CHASE = 1.15;
const CLEAR_WIDE = 1.6;

// Trunk-occlusion thresholds. vegetation.js clears trees wherever the trail
// carved the ground (carve > 0.16) or within 3.2–4.7 m of the centreline, so
// these two numbers are read straight off that module's placement rules.
const CARVE_CLEAR = 0.16;
const CORRIDOR_TRIGGER = 4.2;   // metres out before we believe there is a trunk
const CORRIDOR_TARGET = 3.6;    // metres out we pull back to — inside the clear band

const DEG2RAD = Math.PI / 180;

// ---------------------------------------------------------------------------
// Hero framing (r3 review, Lane G)
//
// G1: "the hero cameras cannot photograph the hero" — four of sixteen review frames either
// amputate the bike at a frame edge, crop the front wheel at x = 0, bury the whole bike
// behind a terrain lip, or hide the bike behind its own rider. G2: nine of sixteen put the
// rider within ~3% of frame centre with the vanishing point and the horizon there too,
// which reads as a debug camera.
//
// The answer is a solver, not a bigger boom: project the hero's bounds, guarantee they sit
// inside a safe rect, and *deliberately* place them off centre inside it.
// ---------------------------------------------------------------------------

// Bounding sphere of bike + rider in world units. `bike.state.position` is the rear-axle-ish
// chassis origin, one wheel radius (0.37 m) above the tread; the helmet crown is ~1.85 m
// above the tread and the wheelbase is 1.25 m. A sphere centred 0.72 m above the origin with
// a 1.35 m radius contains the front contact patch (1.26 m from the centre) and the helmet
// (0.76 m) with room left for 170 mm of fork travel and a whip.
const HERO_UP = 0.72;
const HERO_R = 1.35;
// The bottom of those bounds, measured down from `pose.anchor` (= bike origin + HERO_UP).
// The contact patch is one wheel radius below the origin, so 0.72 + 0.37.
const HERO_FOOT_BELOW_ANCHOR = 1.09;

// The safe rect the bounds must sit inside, as a fraction of the true half-frame:
// the central 90% of WIDTH (the 5% margins G1 asks for) and the central 70% of HEIGHT.
const SAFE_H = 0.90;
const SAFE_V = 0.70;
// The bounds may fill this much of the *tighter* safe half-extent before the camera is
// pushed back. The remainder is headroom for the composition offset below.
const HERO_FILL_MAX = 0.85;
// Never push back further than this in one solve — past it the shot is wrong for a reason
// the framing solver cannot fix.
const HERO_PUSH_MAX = 7.0;

// Composition offset, as a share of the safe half-extent. 0.30 of the safe half-width is
// ~13.5% of frame width off centre at 62° / 16:9, i.e. essentially the rule-of-thirds line;
// the corner term adds to it so the rider ends up on the *inside* and the apex opens up.
const LEAD_BASE = 0.30;
const LEAD_CURVE = 0.55;
const LEAD_MAX = 0.55;
// Vertical. On a steep descent the hero rides high in frame and the horizon is pushed above
// the midline; on flat ground it drops below centre and the horizon comes down with it.
// Either way it is never *at* the midline, which is what nine of sixteen r3 frames were.
const LEAD_V_STEEP = 0.45;
const LEAD_V_FLAT = -0.22;
const LEAD_V_GRAD = 0.45;       // gradient at which the steep value is reached

// Azimuth off dead astern. A camera on the bike's own centreline photographs the rider's
// back and nothing else — r3_15 is a shot briefed "bike, low rear" in which the bike is
// entirely hidden behind its rider. 0.20 rad = 11.5°, which is enough to bring the frame,
// the fork crowns and the far grip back into shot and small enough that the trail ahead is
// still square to the lens. The QA hero presets use the 15–25° G1 asks for; a gameplay
// camera does not want to sit that far off axis for two and a half kilometres.
const SIDE_FLIP = 0.25;         // |curve| at which the held side commits to the new hand
const SIDE_RATE = 1.15;         // /s — how fast azimuth and lead follow the held side

// Crest occlusion. r3_11: 12 m back over a convex rollover, and the rider is buried to the
// chest by the tread itself. Solve the sight line to the *bottom* of the hero bounds.
// Metres the sight line must clear the ground by, at the camera end, ramping to zero at the
// wheels. Ramped rather than flat for the same reason the occlusion sweep ramps its own
// clearance: at the hero end the "ground" being sampled is the ground the bike is standing
// on, so a flat requirement is a false positive that lifts the camera on level trail. The
// margin also has to cover the trail tread ribbon, which sits 0.035–0.12 m proud of the
// heightfield `groundAt` samples.
const HERO_SIGHT_MARGIN = 0.22;
const HERO_LIFT_MAX = 3.2;        // metres
const HERO_SIGHT_STEPS = 7;
const HERO_SIGHT_NEAR = 0.18;     // start sampling this far along, hero → camera

// --- per-mode rig definitions ----------------------------------------------
// dist/height are metres at rest; *Speed terms are added at SPEED_REF.
// flow      = how much of the camera's yaw comes from the trail tangent ahead
//             rather than from the bike itself (the single most important number
//             in this file — see solveChase).
// lookMix   = how much the look-at point is pulled onto the trail ahead.
// fovOff    = offset applied to the rest FOV relative to ctx.settings.fov - 4.
// fovSpan   = degrees added between rest and SPEED_REF.
// omega*    = spring frequencies (rad/s). Critically damped, so settling time is
//             roughly 4/omega seconds.
const RIGS = {
  chase: {
    dist: 5.35, distSpeed: 2.55, height: 1.92, heightSpeed: 0.52,
    flow: 0.55, flowAir: 0.20, lookMix: 0.28, lookMixSpeed: 0.24,
    side: 1.15, lookSide: 0.55, roll: 0.30,
    azOff: 0.20, azCurve: 0.10,
    fovOff: 0, fovSpan: 20,
    omegaPos: 7.4, omegaY: 4.6, omegaLook: 11.5, ff: 0.72,
    shake: 1.0, noise: 1.0, clearance: CLEAR_CHASE, avoid: true, sweep: true, treeCap: 0.55,
    frameHero: true, leadScale: 1.0,
  },
  chaseFar: {
    dist: 8.90, distSpeed: 3.40, height: 3.05, heightSpeed: 0.80,
    flow: 0.66, flowAir: 0.16, lookMix: 0.34, lookMixSpeed: 0.24,
    side: 1.75, lookSide: 0.80, roll: 0.22,
    azOff: 0.16, azCurve: 0.08,
    fovOff: -3, fovSpan: 15,
    omegaPos: 5.6, omegaY: 3.7, omegaLook: 9.0, ff: 0.80,
    shake: 0.66, noise: 0.72, clearance: CLEAR_WIDE, avoid: true, sweep: true, treeCap: 0.32,
    frameHero: true, leadScale: 0.85,
  },
  firstPerson: {
    dist: 0, distSpeed: 0, height: 0, heightSpeed: 0,
    flow: 0.30, flowAir: 0.10, lookMix: 0.36, lookMixSpeed: 0.20,
    side: 0, lookSide: 0.9, roll: 0.40,
    azOff: 0, azCurve: 0,
    // GoPro-wide on purpose: a wide lens is also what keeps the macro-range
    // gloves small at the frame corners instead of filling the screen.
    fovOff: 16, fovSpan: 10,
    // Stiffer than chase on purpose: any lag between the head and the eye
    // reads as swimming, and the authored bob shouldn't fight a slow spring.
    omegaPos: 30, omegaY: 27, omegaLook: 17, ff: 1.0,
    // The eye sits at zero lever arm from the impacts — raw impulses that read
    // as "energy" three metres back read as nausea here.
    shake: 0.85, noise: 0.95, clearance: 0.35, avoid: false, sweep: false,
    // How much the mount is pulled off the animated rider head onto the rigid
    // chassis frame. The rider rig absorbs bike motion through arms and spine,
    // so a pure head mount floats on the body; a share of chassis keeps the
    // camera coupled to the front end.
    mount: 0.55,
    // Direct fork-compression dip (fraction of travel) and the share of steer
    // angle the view yaws with — both are what "bolted near the bars" reads as.
    forkDip: 0.35, steerYaw: 0.20,
    // Keep the bar centre no lower than this fraction of the lower half-FOV.
    // The trail-pull lifts the gaze at speed and quietly pushes the whole
    // cockpit out the bottom of frame; this is the floor under that.
    barFrame: 0.85,
    // There is no hero to frame from inside the hero's own helmet.
    frameHero: false, leadScale: 0,
  },
  // Director cameras keep the ground push-out (nobody wants a shot from inside
  // the mountain) but NOT the bike→camera sweep: a long lens looking across a
  // valley is supposed to have terrain between the lens and the subject.
  cinematic: {
    fovOff: 0, fovSpan: 0,
    omegaPos: 13, omegaY: 13, omegaLook: 9, ff: 1.0,
    shake: 0.18, noise: 0.55, clearance: 2.2, avoid: true, sweep: false, roll: 0,
    // Director shots are authored setups; the framing solver would fight the shot list.
    frameHero: false, leadScale: 0,
  },
  replay: {
    fovOff: 0, fovSpan: 0,
    omegaPos: 9, omegaY: 9, omegaLook: 4.6, ff: 0.9,
    shake: 0.45, noise: 0.85, clearance: 1.4, avoid: true, sweep: false, roll: 0,
    // Containment only — a broadcast operator on a long lens leads the subject, but gently,
    // and the tripod position is fixed so there is no boom to push back along.
    frameHero: true, leadScale: 0.5,
  },
};

// The cameras the C key walks through. Director cameras are excluded on purpose.
const CYCLE = ['chase', 'chaseFar', 'firstPerson'];

// ---------------------------------------------------------------------------
// Module-scope scratch. Nothing below allocates per frame.
// ---------------------------------------------------------------------------

const _v0 = new THREE.Vector3();
const _v1 = new THREE.Vector3();
const _v2 = new THREE.Vector3();
const _v3 = new THREE.Vector3();
const _v4 = new THREE.Vector3();
const _up = new THREE.Vector3(0, 1, 0);
const _fwd = new THREE.Vector3();
const _boom = new THREE.Vector3();
const _right = new THREE.Vector3();
const _camUp = new THREE.Vector3();
const _q0 = new THREE.Quaternion();
const _m4 = new THREE.Matrix4();
const _eul = new THREE.Euler(0, 0, 0, 'YXZ');

// Trail read-back buffers (trail.sampleAt returns a shared object — copy at once).
const _trPos = new THREE.Vector3();
const _trTan = new THREE.Vector3(0, 0, -1);
const _trPos2 = new THREE.Vector3();
const _trTan2 = new THREE.Vector3(0, 0, -1);

// ---------------------------------------------------------------------------
// Maths helpers
// ---------------------------------------------------------------------------

/** 5th-order smoothstep — zero 1st *and* 2nd derivative at both ends. Mode blends
 *  use this because a plain smoothstep still shows a velocity kink at the join. */
function smootherstep(x) {
  const t = clamp01(x);
  return t * t * t * (t * (t * 6 - 15) + 10);
}

// Exact critically-damped spring (the closed-form solution, not an Euler step),
// so behaviour is identical at 30, 60 and 144 fps. Results come back through two
// module-scope slots to avoid allocating a return object.
let _sx = 0, _sv = 0;
function critDamp(x, v, target, omega, dt) {
  if (!(omega > 0) || dt <= 0) { _sx = x; _sv = v; return; }
  const e = Math.exp(-omega * dt);
  const c = x - target;
  const t2 = (v + omega * c) * dt;
  _sx = target + (c + t2) * e;
  _sv = (v - omega * t2) * e;
}

/** Under-damped oscillator relaxing to zero — used for shake, which should ring
 *  a couple of times rather than ooze back like a critically damped spring. */
function oscDecay(x, v, omega, zeta, dt) {
  if (dt <= 0) { _sx = x; _sv = v; return; }
  const wd = omega * Math.sqrt(Math.max(1e-4, 1 - zeta * zeta));
  const e = Math.exp(-zeta * omega * dt);
  const c = Math.cos(wd * dt), s = Math.sin(wd * dt);
  const a = x;
  const b = (v + zeta * omega * x) / wd;
  _sx = e * (a * c + b * s);
  _sv = e * ((-zeta * omega) * (a * c + b * s) + wd * (b * c - a * s));
}

/** Normalised XZ blend of two directions. Falls back to `a` when they are close
 *  to opposite, where the linear blend degenerates. */
function blendDirXZ(a, b, w, out) {
  const d = a.x * b.x + a.z * b.z;
  if (d < -0.9) { out.set(a.x, 0, a.z); }
  else out.set(a.x + (b.x - a.x) * w, 0, a.z + (b.z - a.z) * w);
  const l = Math.sqrt(out.x * out.x + out.z * out.z);
  if (l < 1e-5) out.set(a.x, 0, a.z).normalize();
  else { out.x /= l; out.z /= l; }
  return out;
}

/**
 * Rotate a direction about world Y, in place. Positive `a` swings the vector to the LEFT of
 * where it points (facing -Z, +a moves it toward -X), which means the camera hung off the
 * *back* of that vector swings to the rider's right. Sign checked against the air-drift
 * rotation below, which uses the same form.
 */
function rotateXZ(v, a) {
  if (!(Math.abs(a) > 1e-5)) return v;
  const c = Math.cos(a), s = Math.sin(a);
  const x = v.x * c + v.z * s;
  const z = -v.x * s + v.z * c;
  v.x = x; v.z = z;
  return v;
}

/**
 * Signed yaw from `a` to `b` in the XZ plane, radians, + = to the rider's right.
 * Check: facing -Z, right is +X; a turn toward +X must come back positive, which
 * is why the cross term is (a.x*b.z - a.z*b.x) and not the other way round.
 */
function yawDelta(a, b) {
  const cross = a.x * b.z - a.z * b.x;
  const dot = a.x * b.x + a.z * b.z;
  return Math.atan2(cross, dot);
}

// ---------------------------------------------------------------------------
// Pose — the thing every mode solver produces
// ---------------------------------------------------------------------------

function makePose() {
  return {
    pos: new THREE.Vector3(),
    look: new THREE.Vector3(0, 0, -1),
    roll: 0,
    fov: 62,
    omegaPos: 7.4, omegaY: 4.6, omegaLook: 11.5, ff: 0.7,
    shake: 1, noise: 1,
    clearance: CLEAR_CHASE,
    avoid: true,                   // push out of the ground along the normal
    sweep: true,                   // sweep the bike→camera segment for occluders
    treeCap: 0.55,                 // most of the arm the trunk response may eat
    anchor: new THREE.Vector3(),   // hero bounds centre; the bike→camera sweep starts here
    // --- hero framing (r3 Lane G) ---
    frameHero: true,               // run the containment / composition solver at all
    heroR: HERO_R,                 // radius of the hero bounding sphere, metres
    leadX: 0,                      // where to place the hero horizontally, share of safe half
    leadY: 0,                      // ...and vertically
    fillMin: 0,                    // 0 = no minimum apparent size (gameplay rigs)
  };
}

function copyPose(dst, src) {
  dst.pos.copy(src.pos); dst.look.copy(src.look); dst.anchor.copy(src.anchor);
  dst.roll = src.roll; dst.fov = src.fov;
  dst.omegaPos = src.omegaPos; dst.omegaY = src.omegaY;
  dst.omegaLook = src.omegaLook; dst.ff = src.ff;
  dst.shake = src.shake; dst.noise = src.noise;
  dst.clearance = src.clearance; dst.avoid = src.avoid; dst.sweep = src.sweep;
  dst.treeCap = src.treeCap;
  dst.frameHero = src.frameHero; dst.heroR = src.heroR;
  dst.leadX = src.leadX; dst.leadY = src.leadY; dst.fillMin = src.fillMin;
}

function blendPose(a, b, w, out) {
  out.pos.lerpVectors(a.pos, b.pos, w);
  out.look.lerpVectors(a.look, b.look, w);
  out.anchor.lerpVectors(a.anchor, b.anchor, w);
  out.roll = lerp(a.roll, b.roll, w);
  out.fov = lerp(a.fov, b.fov, w);
  out.omegaPos = lerp(a.omegaPos, b.omegaPos, w);
  out.omegaY = lerp(a.omegaY, b.omegaY, w);
  out.omegaLook = lerp(a.omegaLook, b.omegaLook, w);
  out.ff = lerp(a.ff, b.ff, w);
  out.shake = lerp(a.shake, b.shake, w);
  out.noise = lerp(a.noise, b.noise, w);
  out.clearance = lerp(a.clearance, b.clearance, w);
  out.treeCap = lerp(a.treeCap, b.treeCap, w);
  out.heroR = lerp(a.heroR, b.heroR, w);
  out.leadX = lerp(a.leadX, b.leadX, w);
  out.leadY = lerp(a.leadY, b.leadY, w);
  out.fillMin = lerp(a.fillMin, b.fillMin, w);
  // Constraint flags cannot be interpolated; take the incoming pose's rules as
  // soon as it owns the majority of the blend.
  out.avoid = w < 0.5 ? a.avoid : b.avoid;
  out.sweep = w < 0.5 ? a.sweep : b.sweep;
  // ...except hero framing, which is a containment guarantee: honour it if *either* side of
  // the blend wants it, so the hero cannot fall out of frame mid-transition. The lead terms
  // interpolate to zero on their own for the modes that do not want composition.
  out.frameHero = a.frameHero || b.frameHero;
}

// =============================================================================
export function createChaseCamera(ctx) {
  const cam = ctx.camera;

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------
  let mode = 'chase';
  let prevMode = 'chase';
  let blendT = 1;                 // 1 = settled in `mode`
  let firstFrame = true;
  // Set by whichever solver decides an intentional hard cut is due (a replay
  // hand-off, a cinematic shot change, entering a director camera).
  let requestSnap = false;
  // True only while the *active* mode is being solved. During a mode blend the
  // outgoing mode is solved too, and it must not be allowed to fire cuts.
  let solvingActive = true;

  const poseA = makePose();       // solver output for `prevMode`
  const poseB = makePose();       // solver output for `mode`
  const pose = makePose();        // blended + constrained target

  // Follow springs.
  const spPos = new THREE.Vector3();
  const spVel = new THREE.Vector3();
  const spLook = new THREE.Vector3();
  const spLookVel = new THREE.Vector3();
  const prevTargetPos = new THREE.Vector3();
  const prevTargetLook = new THREE.Vector3();
  const tgtVel = new THREE.Vector3();       // smoothed d(target)/dt, for feed-forward
  const tgtLookVel = new THREE.Vector3();
  let haveHistory = false;
  let skipVelFrame = false;

  let rollNow = 0, rollVel = 0;
  let fovNow = 62, fovVel = 0;
  let fovKick = 0, fovKickVel = 0;
  let lastSpeed = 0;

  // Shake: world-space positional oscillator + camera-local angular oscillator.
  const shakePos = new THREE.Vector3();
  const shakeVel = new THREE.Vector3();
  let shYaw = 0, shYawV = 0, shPitch = 0, shPitchV = 0, shRoll = 0, shRollV = 0;
  // Impulses arrive during update(); they are consumed in lateUpdate().
  const pendImpulse = new THREE.Vector3();
  let pendPitch = 0, pendRollImp = 0, pendYawImp = 0, pendCount = 0;

  // Terrain-avoidance memory (fast in, slow out — a snap back is worse than the clip).
  let avoidLift = 0;
  let avoidShorten = 0;
  let treeShorten = 0;
  // Crest-occlusion memory, same discipline: rise immediately, release over ~1.3 s.
  let heroLift = 0;

  // Composition memory. `sideHold` is a sticky ±1 (+1 = camera off the rider's right);
  // `azNow` and `leadNow`/`leadVNow` are its smoothed consequences, so a slalom does not
  // make the frame slosh from side to side at the corner-detection threshold.
  let sideHold = 1;
  let azNow = 0;
  let leadNow = 0;
  let leadVNow = 0;

  // Crash orbit.
  let crashW = 0;
  const crashAnchor = new THREE.Vector3();
  let crashAz = 0;
  let wasCrashed = false;

  // Air drift.
  let airAz = 0;

  // Trail bookkeeping.
  let trailHint = 0;
  let trailLen = 2600;
  let bikeOnTrail = true;

  // Deterministic noise for handheld drift and for the director cameras.
  const rng = makeRng(subSeed(ctx.seed || 0, 'camera'));
  const noise2 = createNoise2D(makeRng(subSeed(ctx.seed || 0, 'camera:handheld')));

  // -------------------------------------------------------------------------
  // Photo mode (free-fly). Owned here; see the CONTRACT-NOTE at the top.
  // -------------------------------------------------------------------------
  let photo = false;
  let photoEdge = false;
  let cycleEdge = false;
  const phPos = new THREE.Vector3();
  const phVel = new THREE.Vector3();
  let phYaw = 0, phPitch = 0, phYawT = 0, phPitchT = 0;
  let phFov = 55, phFovT = 55;
  let phRoll = 0;
  let pointerLocked = false;
  let dragging = false;
  const phKeys = new Set();
  // Publish it immediately so hud.js defers to this module from frame one rather
  // than running its own local toggle (see the CONTRACT-NOTE at the top).
  ctx.photoMode = false;
  if (ctx.settings) ctx.settings.photoMode = false;

  // -------------------------------------------------------------------------
  // Small terrain/trail accessors, all allocation free and all defensive: this
  // module must never be the reason a frame throws.
  // -------------------------------------------------------------------------

  function groundAt(x, z) {
    const t = ctx.terrain;
    if (!t || !isFinite(x) || !isFinite(z)) return 0;
    if (t.inBounds && !t.inBounds(x, z)) return t.valleyY !== undefined ? t.valleyY : 0;
    const h = t.sampleHeight(x, z);
    return isFinite(h) ? h : 0;
  }

  function normalAt(x, z, out) {
    const c = ctx.collision;
    if (c && c.normalAt) { c.normalAt(x, z, out); if (isFinite(out.x)) return out; }
    const t = ctx.terrain;
    if (t && t.sampleNormal) { t.sampleNormal(x, z, out); if (isFinite(out.x)) return out; }
    return out.set(0, 1, 0);
  }

  /**
   * Signed metres from the trail centreline at `pos`. Keeps its own search hint
   * (the camera is up to 14 m behind the bike, which is more than trail.js's
   * ±8.8 m local window) so the query stays on the cheap local path.
   */
  let camHint = 0;
  function corridorLateral(pos) {
    const tr = ctx.trail;
    if (!tr || !tr.nearestT) return 0;
    const n = tr.nearestT(pos, camHint);
    if (!n || !isFinite(n.t)) return 0;
    camHint = n.t;
    // A stale/far answer is not evidence of anything; treat it as "in the clear".
    if (n.distance > 60) return 0;
    return n.lateral;
  }

  /** 0 = untouched mountain (trees live here), 1 = trail tread. */
  function corridorClearance(x, z) {
    const veg = ctx.vegetation;
    // Preferred path if vegetation ever exposes a real occluder query.
    if (veg && typeof veg.corridorClearance === 'function') {
      const v = veg.corridorClearance(x, z);
      if (isFinite(v)) return clamp01(v);
    }
    const t = ctx.terrain;
    if (t && typeof t.sampleCarve === 'function') {
      const v = t.sampleCarve(x, z);
      return isFinite(v) ? clamp01(v) : 1;
    }
    return 1;
  }

  /** Copy a trail sample out of the shared object trail.sampleAt() returns. */
  function readTrail(t, outPos, outTan) {
    const tr = ctx.trail;
    if (!tr || !tr.sampleAt) return false;
    const s = tr.sampleAt(clamp01(t));
    if (!s || !isFinite(s.position.x)) return false;
    outPos.copy(s.position);
    outTan.copy(s.tangent);
    if (Math.abs(outTan.x) + Math.abs(outTan.z) < 1e-4) outTan.set(0, outTan.y, -1);
    return true;
  }

  function trailTFor(s) {
    const tr = ctx.trail;
    if (!tr) return 0;
    if (s && isFinite(s.trailT) && s.trailT > 0) { trailHint = s.trailT; return s.trailT; }
    if (s && tr.nearestT) {
      const n = tr.nearestT(s.position, trailHint);
      if (n && isFinite(n.t)) { trailHint = n.t; return n.t; }
    }
    return trailHint;
  }

  // -------------------------------------------------------------------------
  // Bike fallback — the camera must still frame *something* if bike.js is down.
  // -------------------------------------------------------------------------
  const fbState = {
    position: new THREE.Vector3(),
    velocity: new THREE.Vector3(),
    forward: new THREE.Vector3(0, 0, -1),
    right: new THREE.Vector3(1, 0, 0),
    up: new THREE.Vector3(0, 1, 0),
    speed: 0, lean: 0, gForce: 1, roughness: 0.2,
    airborne: false, airTime: 0, crashed: false,
    trailT: 0, lateral: 0, offTrail: false, whip: 0,
    brakeFront: 0, brakeRear: 0,
  };
  let fbReady = false;

  function bikeState() {
    const s = ctx.bike && ctx.bike.state;
    if (s && s.position && isFinite(s.position.x)) return s;
    if (!fbReady) {
      const st = ctx.trail && ctx.trail.startTransform;
      if (st && st.position) {
        fbState.position.copy(st.position);
        fbState.forward.set(0, 0, -1).applyQuaternion(st.quaternion).setY(0).normalize();
        fbState.right.crossVectors(fbState.forward, _up).normalize();
        fbReady = true;
      }
    }
    return fbState;
  }

  // =========================================================================
  // MODE SOLVERS
  // =========================================================================

  /**
   * Chase / chaseFar.
   *
   * The important idea: the camera's yaw is NOT the bike's yaw. It is a blend of
   *   (a) where the bike is actually travelling, and
   *   (b) where the *trail* is going, sampled a speed-scaled distance ahead.
   * That is what lets you see a corner before you are in it, and it lets the bike
   * whip, drift and scrub inside the frame instead of dragging the world around
   * with it. Rigidly bolting the camera to chassis yaw is the single most common
   * reason a chase camera reads as amateur.
   */
  function solveChase(p, s, rig, dt) {
    const speed = Math.max(0, s.speed || 0);
    const sn = clamp01(speed / SPEED_REF);
    const air = clamp01((s.airTime || 0) / 0.75) * (s.airborne ? 1 : 0);

    // ---- direction the bike is committed to ------------------------------
    _v0.copy(s.forward); _v0.y = 0;
    if (_v0.lengthSq() < 1e-6) _v0.set(0, 0, -1); else _v0.normalize();
    if (speed > 2.0) {
      _v1.copy(s.velocity); _v1.y = 0;
      if (_v1.lengthSq() > 1e-6) {
        _v1.normalize();
        // Follow the travel direction a little more than the heading, so a drift
        // shows the bike sideways in frame rather than pointing the camera into
        // the trees the front wheel happens to face.
        blendDirXZ(_v0, _v1, 0.40 * clamp01((speed - 2) / 5), _v0);
      }
    }

    // ---- direction the trail is going ------------------------------------
    const t0 = trailTFor(s);
    const lookAheadM = lerp(9, 34, sn);
    let haveTrail = false;
    let curve = 0;
    if (trailLen > 1 && readTrail(t0 + lookAheadM / trailLen, _trPos, _trTan)) {
      haveTrail = true;
      // A second, nearer sample keeps the flow direction from snapping when the
      // look-ahead point crosses a switchback.
      if (readTrail(t0 + (lookAheadM * 0.42) / trailLen, _trPos2, _trTan2)) {
        blendDirXZ(_trTan2, _trTan, 0.62, _v2);
      } else {
        _v2.set(_trTan.x, 0, _trTan.z).normalize();
      }
      // Signed curvature over the look-ahead: drives the outside-of-corner offset.
      _v3.set(_trTan.x, 0, _trTan.z).normalize();
      curve = clamp(yawDelta(_v0, _v3) / 0.9, -1, 1);
    } else {
      _v2.copy(_v0);
    }

    // How much to trust the trail: not at all if we are a long way off it, and
    // less if the rider is deep off-line (B-lines, crashes, exploring).
    let trust = haveTrail ? 1 : 0;
    if (s.offTrail) trust *= 0.45;
    const lat = Math.abs(s.lateral || 0);
    trust *= 1 - smoothstep(4.0, 14.0, lat);
    if (s.crashed) trust *= 0.3;

    const flow = (rig.flow + (s.airborne ? rig.flowAir : 0)) * trust;
    blendDirXZ(_v0, _v2, flow, _fwd);

    // Airborne drift: rotate the rig slowly around the bike while it is in the
    // air. Costs nothing, and it is what makes hang time read as long.
    const whipSign = (s.whip || 0) !== 0 ? Math.sign(s.whip) : 1;
    const airAzT = air * 0.20 * whipSign;
    airAz = damp(airAz, airAzT, s.airborne ? 1.6 : 4.5, dt);
    if (Math.abs(airAz) > 1e-4) {
      const c = Math.cos(airAz), sa = Math.sin(airAz);
      const fx = _fwd.x * c + _fwd.z * sa;
      const fz = -_fwd.x * sa + _fwd.z * c;
      _fwd.set(fx, 0, fz).normalize();
    }

    _right.crossVectors(_fwd, _up).normalize();

    // ---- rig geometry ------------------------------------------------------
    const brake = clamp01(Math.max(s.brakeFront || 0, s.brakeRear || 0));
    let dist = rig.dist + rig.distSpeed * sn + air * 1.7 - brake * 0.45 * sn;
    let height = rig.height + rig.heightSpeed * sn + air * 0.95;
    // Steep ground: lift the rig so the trail below stays in frame instead of the
    // camera burying its nose in the slope you just rode off. The lift has to be
    // proportional to the ARM, not a constant — the ground behind rises by
    // grad*dist, so a fixed 1 m works for the 6 m chase rig and leaves the 12 m
    // wide rig underground, where terrain avoidance then eats 4 m of its length
    // and it stops being a wide camera at all. (Measured: chaseFar's median arm
    // was 7.9 m instead of 12 before this was made proportional.)
    const grad = clamp(-(s.forward ? s.forward.y : 0), -0.7, 0.7);
    const gUp = clamp(grad, 0, 0.7);
    height += Math.min(4.2, gUp * dist * 0.62);
    dist -= gUp * dist * 0.10;

    // ---- azimuth off dead astern + composition side (r3 G1/G2) ------------
    // Sticky, so a rhythm section does not flip the frame from side to side; smoothed, so
    // the flip that does happen takes a second rather than a frame. Advanced only while the
    // *active* mode is being solved — during a mode blend this function runs twice.
    if (solvingActive) {
      if (curve > SIDE_FLIP) sideHold = -1;
      else if (curve < -SIDE_FLIP) sideHold = 1;
      const azT = (rig.azOff || 0) * sideHold - curve * (rig.azCurve || 0);
      // The rider goes to the side OPPOSITE the way the camera swung and opposite the way
      // the trail turns, so the space in frame is where the trail is about to be.
      const leadT = clamp(-(LEAD_BASE * sideHold + LEAD_CURVE * curve), -LEAD_MAX, LEAD_MAX);
      const leadVT = lerp(LEAD_V_FLAT, LEAD_V_STEEP, clamp01(gUp / LEAD_V_GRAD));
      azNow = damp(azNow, azT, SIDE_RATE, dt);
      leadNow = damp(leadNow, leadT, SIDE_RATE, dt);
      leadVNow = damp(leadVNow, leadVT, 2.2, dt);
    }
    // The boom is the flow direction swung off the bike's axis; `_fwd` itself is left alone
    // because the corner swing and the aim bias below are both expressed in its frame.
    _boom.copy(_fwd);
    rotateXZ(_boom, azNow);

    p.pos.copy(s.position);
    p.pos.addScaledVector(_boom, -dist);
    p.pos.y += height;
    // Swing to the OUTSIDE of the upcoming corner: the bike sits off-centre in
    // frame on entry and the apex opens up. 1.1–1.7 m is enough to feel, small
    // enough not to look like a mistake.
    p.pos.addScaledVector(_right, -curve * rig.side);

    // ---- look target -------------------------------------------------------
    // Mixing a bike-anchored point with a point on the trail ahead is what
    // automatically pitches the camera down into a chute and up out of a
    // compression, without a single explicit pitch rule.
    p.look.copy(s.position);
    p.look.y += 1.15;
    if (trust > 0.01) {
      const lookM = lerp(6, 21, sn);
      if (readTrail(t0 + lookM / trailLen, _trPos2, _trTan2)) {
        _v3.copy(_trPos2); _v3.y += 1.30;
        p.look.lerp(_v3, (rig.lookMix + rig.lookMixSpeed * sn) * trust);
      }
    }
    // Aim slightly to the INSIDE while the rig sits outside — that pair is what
    // frames an apex properly.
    p.look.addScaledVector(_right, curve * rig.lookSide);
    // Airborne: drop the aim so the landing is visible, not just sky.
    p.look.y -= air * 0.85;

    // The anchor is the centre of the hero bounding sphere: it is both what the occlusion
    // sweep starts from and what the framing solver keeps inside the safe rect.
    p.anchor.copy(s.position); p.anchor.y += HERO_UP;
    p.frameHero = rig.frameHero !== false;
    p.heroR = HERO_R;
    p.leadX = leadNow * (rig.leadScale !== undefined ? rig.leadScale : 1);
    p.leadY = leadVNow * (rig.leadScale !== undefined ? rig.leadScale : 1);
    p.fillMin = 0;

    // ---- roll --------------------------------------------------------------
    // 25–35% of the bike's lean, per the contract. 30% keeps berms feeling banked
    // without the horizon swinging enough to make anyone queasy.
    p.roll = clamp((s.lean || 0) * rig.roll, -0.34, 0.34);

    // ---- FOV ---------------------------------------------------------------
    const base = baseFov();
    const rest = base - 4 + rig.fovOff;
    // Slightly super-linear so the last third of the speed range still gives a
    // visible widening — a linear ramp reads as "nothing happening" up top.
    p.fov = rest + Math.pow(sn, 1.15) * rig.fovSpan;

    p.omegaPos = rig.omegaPos; p.omegaY = rig.omegaY;
    p.omegaLook = rig.omegaLook; p.ff = rig.ff;
    p.shake = rig.shake; p.noise = rig.noise;
    p.clearance = rig.clearance; p.avoid = rig.avoid; p.sweep = !!rig.sweep;
    p.treeCap = rig.treeCap !== undefined ? rig.treeCap : 0.5;

    // ---- crash orbit -------------------------------------------------------
    if (crashW > 0.001) applyCrashOrbit(p, s, dt);
  }

  /**
   * Crash beat: a wider, slower orbit around the point of the wreck so the reset
   * has a moment. Blended over the chase pose rather than replacing it, so the
   * entry and exit are both continuous.
   */
  function applyCrashOrbit(p, s, dt) {
    crashAz += dt * 0.42;
    const r = 8.4;
    const cx = crashAnchor.x + Math.sin(crashAz) * r;
    const cz = crashAnchor.z + Math.cos(crashAz) * r;
    _v0.set(cx, crashAnchor.y + 3.5, cz);
    const g = groundAt(cx, cz) + 2.4;
    if (_v0.y < g) _v0.y = g;
    _v1.copy(crashAnchor); _v1.y += 0.85;
    // Drift the aim toward wherever the ragdoll ended up.
    _v1.lerp(s.position, 0.35);

    const w = smootherstep(crashW);
    p.pos.lerp(_v0, w);
    p.look.lerp(_v1, w);
    p.roll = lerp(p.roll, 0, w);
    p.fov = lerp(p.fov, baseFov() - 9, w);
    p.omegaPos = lerp(p.omegaPos, 2.6, w);
    p.omegaY = lerp(p.omegaY, 2.2, w);
    p.omegaLook = lerp(p.omegaLook, 3.2, w);
    p.ff = lerp(p.ff, 0.2, w);
    p.shake = lerp(p.shake, 0.35, w);
    p.clearance = lerp(p.clearance, 2.0, w);
    // Let the orbit be wide. The trunk-avoidance pull is tuned for a chase arm
    // that has to stay in the corridor; applied to an 8.4 m orbit it collapsed the
    // radius to 3.7 m (measured) and there was no crash beat left. Terrain
    // avoidance still applies — the shot may be wide, but not underground.
    p.treeCap = lerp(p.treeCap, 0.08, w);
    // Drop the composition lead: a wreck is a director beat, and an orbit that also swings
    // the subject across the frame reads as a camera that has lost the plot. Containment
    // stays on — the ragdoll must not leave frame.
    p.leadX = lerp(p.leadX, 0, w);
    p.leadY = lerp(p.leadY, 0, w);
  }

  // ---- first person -------------------------------------------------------
  let bobPhase = 0;
  // Landing dip: the rider's knees, not the camera spring. A sprung offset that
  // takes a downward kick on the airborne→contact edge and recovers in ~0.4 s.
  let fpWasAir = false, fpAirT = 0, dipY = 0, dipV = 0;
  // Slow-tracked head offset in the chassis frame (up/forward/right components).
  // This is what the rig.mount blend anchors to — see solveFirstPerson.
  let fpMntU = 0, fpMntF = 0, fpMntR = 0, fpMntOk = false;
  const headPos = new THREE.Vector3();

  function solveFirstPerson(p, s, rig, dt) {
    const speed = Math.max(0, s.speed || 0);
    const sn = clamp01(speed / SPEED_REF);

    // Prefer the rider's actual head (rider.js updates in the wave before us) —
    // but only if it is actually attached to this bike. A rider that has not been
    // updated yet, or a ragdoll that has been flung, reports a head hundreds of
    // metres away, and a helmet cam that stays behind at the start gate is a much
    // worse failure than one that is 5 cm off the anatomical position.
    let gotHead = false;
    if (ctx.rider && typeof ctx.rider.getHeadPosition === 'function') {
      ctx.rider.getHeadPosition(headPos);
      if (isFinite(headPos.x) && headPos.distanceToSquared(s.position) < 9) gotHead = true;
    }
    if (!gotHead) {
      // Approximates the head; the chest-mount offset below is applied on top.
      headPos.copy(s.position);
      headPos.addScaledVector(s.up, 1.28);
      headPos.addScaledVector(s.forward, 0.12);
    } else if (rig.mount) {
      // Pull the mount partway off the animated head onto the chassis frame —
      // anchored to a SLOW-TRACKED copy of the head's chassis-frame offset, not
      // a fixed point. A fixed point cannot match both the gate stance and the
      // attack tuck (the head moves ~15 cm between them in the chassis frame),
      // and the static error walks the eye into the hollow sleeve meshes.
      // Sustained pose passes through; only the fast divergence — arms and
      // spine soaking up chassis motion — is suppressed by the blend.
      _v3.copy(headPos).sub(s.position);
      const hu = _v3.dot(s.up), hf = _v3.dot(s.forward);
      const hr = s.right ? _v3.dot(s.right) : 0;
      if (!fpMntOk || Math.abs(hu - fpMntU) + Math.abs(hf - fpMntF) > 0.6) {
        fpMntU = hu; fpMntF = hf; fpMntR = hr; fpMntOk = true;
      } else {
        const k = 1 - Math.exp(-dt / 0.35);
        fpMntU += (hu - fpMntU) * k;
        fpMntF += (hf - fpMntF) * k;
        fpMntR += (hr - fpMntR) * k;
      }
      _v3.copy(s.position).addScaledVector(s.up, fpMntU).addScaledVector(s.forward, fpMntF);
      if (s.right) _v3.addScaledVector(s.right, fpMntR);
      if (isFinite(_v3.x)) headPos.lerp(_v3, rig.mount);
    }

    // Head bob: cadence from ground speed, amplitude from surface roughness and
    // suspension activity. Riders are not walking, so this is small and mostly
    // vertical — the wrong call here is a seasickness machine.
    const rough = clamp01(s.roughness !== undefined ? s.roughness : 0.25);
    const contact = s.airborne ? 0 : 1;
    bobPhase += dt * (2.2 + speed * 0.55);
    const bobAmp = contact * (0.007 + rough * 0.022) * (0.35 + sn * 0.9);
    const bobY = Math.sin(bobPhase) * bobAmp;
    const bobX = Math.sin(bobPhase * 0.5 + 1.1) * bobAmp * 0.6;

    if (s.airborne) fpAirT = Math.max(fpAirT, s.airTime || 0);
    if (fpWasAir && !s.airborne) {
      dipV -= clamp(fpAirT / 0.9, 0.25, 1.0) * 1.6;
      fpAirT = 0;
    }
    fpWasAir = !!s.airborne;
    oscDecay(dipY, dipV, 16, 0.60, dt); dipY = _sx; dipV = _sv;

    _v0.copy(s.forward); _v0.y = 0;
    if (_v0.lengthSq() < 1e-6) _v0.set(0, 0, -1); else _v0.normalize();
    // A bar-mounted camera yaws with the steering; a chest cam does not. Take a
    // share so the cockpit leads into the turn. Sign matches the physics:
    // + steer = right = negative yaw (bike.js steer solver).
    if (rig.steerYaw) _v0.applyAxisAngle(_up, -(s.steer || 0) * rig.steerYaw);
    _right.crossVectors(_v0, _up).normalize();

    p.pos.copy(headPos);
    p.pos.y += bobY + dipY;
    // Fork compression reaches the eye directly — this is the high-frequency
    // "front wheel just hit something" signal the rider rig otherwise absorbs.
    const fork = s.suspension && s.suspension.fork;
    if (fork && rig.forkDip) p.pos.addScaledVector(s.up, -(fork.travel || 0) * rig.forkDip);
    p.pos.addScaledVector(_right, bobX);
    // Chest mount, not helmet mount. In the attack position the head hangs
    // directly over the grips (measured: bars 0.03 m BEHIND the eyes, 0.54 m
    // below) — no FOV shows the cockpit from up there. From the sternum the
    // bars sit ~43° below the lens axis and enter the bottom of frame. The
    // offset lives in the BIKE frame so pitching up a steep face keeps the
    // lens on the sternum instead of drifting behind the rider's back.
    p.pos.addScaledVector(s.forward, -0.34);
    p.pos.addScaledVector(s.up, -0.22);

    // Riders look through the corner, so the helmet cam gets a share of the
    // trail tangent too — just less than the chase rig.
    const t0 = trailTFor(s);
    let curve = 0;
    let trust = 0;
    if (trailLen > 1 && readTrail(t0 + lerp(11, 30, sn) / trailLen, _trPos, _trTan)) {
      trust = s.offTrail ? 0.5 : 1;
      trust *= 1 - smoothstep(4, 12, Math.abs(s.lateral || 0));
      _v3.set(_trTan.x, 0, _trTan.z).normalize();
      curve = clamp(yawDelta(_v0, _v3) / 0.9, -1, 1);
      blendDirXZ(_v0, _v3, rig.flow * trust, _fwd);
    } else {
      _fwd.copy(_v0);
    }

    p.look.copy(p.pos).addScaledVector(_fwd, 12);
    // Look down the hill, not at the horizon: a fixed drop plus the trail's own
    // gradient. This is what puts the bars in shot without faking geometry.
    p.look.y -= 2.6 + clamp(-(s.forward ? s.forward.y : 0), -0.4, 0.8) * 5.0;
    if (trust > 0.01) {
      _v3.copy(_trPos); _v3.y += 1.1;
      p.look.lerp(_v3, (rig.lookMix + rig.lookMixSpeed * sn) * trust * 0.7);
    }
    p.look.addScaledVector(_right, curve * rig.lookSide);

    const base = baseFov();
    p.fov = base - 4 + rig.fovOff + Math.pow(sn, 1.1) * rig.fovSpan;

    // The whole point of a helmet cam is that the bike is in shot. The
    // trail-pull above lifts the gaze with speed, and past ~8 m/s it walks the
    // entire cockpit out the bottom of frame — so after all the look shaping,
    // pitch the gaze back down just enough to hold the bar centre at
    // rig.barFrame of the lower half-FOV. Down only: at the gate the bars sit
    // well inside the frame and this must not touch the shot.
    const anch = rig.barFrame && ctx.bikeModel && ctx.bikeModel.anchors;
    if (anch && anch.bar) {
      anch.bar.getWorldPosition(_v3);
      if (isFinite(_v3.x) && _v3.distanceToSquared(s.position) < 9) {
        _v3.sub(p.pos);
        const dB = _v3.length();
        if (dB > 0.2) {
          const pitchBar = Math.asin(clamp(_v3.y / dB, -1, 1));
          _v3.copy(p.look).sub(p.pos);
          const dL = _v3.length() || 1;
          const pitchLook = Math.asin(clamp(_v3.y / dL, -1, 1));
          const dpMax = Math.atan(Math.tan(p.fov * 0.5 * Math.PI / 180) * rig.barFrame);
          const over = (pitchLook - pitchBar) - dpMax;
          if (over > 0) p.look.y -= Math.tan(over) * dL;
        }
      }
    }

    p.anchor.copy(p.pos);
    p.frameHero = false; p.heroR = HERO_R; p.leadX = 0; p.leadY = 0; p.fillMin = 0;
    // The helmet is bolted to the rider, so it takes more roll than the chase
    // rig — but still not all of it.
    p.roll = clamp((s.lean || 0) * rig.roll, -0.38, 0.38);

    p.omegaPos = rig.omegaPos; p.omegaY = rig.omegaY;
    p.omegaLook = rig.omegaLook; p.ff = rig.ff;
    p.shake = rig.shake; p.noise = rig.noise;
    p.clearance = rig.clearance; p.avoid = rig.avoid; p.sweep = !!rig.sweep;
    p.treeCap = rig.treeCap !== undefined ? rig.treeCap : 0.5;

    if (crashW > 0.001) applyCrashOrbit(p, s, dt);
  }

  // =========================================================================
  // CINEMATIC — the title screen. This is the first thing anyone sees, so it is
  // a proper little shot list rather than a spin.
  // =========================================================================
  let shots = null;
  let shotIndex = 0;
  let shotTime = 0;
  const shotAnchor = new THREE.Vector3();
  const shotLook = new THREE.Vector3();

  /**
   * Shot list.
   *
   * Placement invariant, and it is the whole trick to a cinematic camera in a
   * forest: every setup is EITHER inside the cleared trail corridor (within ~4 m
   * of the centreline, where vegetation.js has removed the trees) OR above the
   * canopy (> ~26 m over local ground). Anything between those two is a shot of
   * tree bark. The one shot that has to break it — the crane — starts in the
   * corridor and travels straight up through the band.
   */
  function buildShots() {
    const tr = ctx.trail;
    const out = [];
    // Shot 0 is always the hero: a slow arc on the bike at the gate with the
    // mountain behind it. This is the title-screen frame, so it gets the longest
    // hold. The arc is an ELLIPSE elongated along the trail — along-trail is
    // inside the cleared corridor, across-trail is in the trees.
    out.push({
      type: 'hero', t: 0.004, along: 7.4, across: 3.1, height: 2.05,
      az: 2.25, azSpeed: 0.050, fov: 40, fov2: 37, dur: 13,
    });

    // Then a spread of shots hung off real features, so the menu is advertising
    // the trail you are about to ride.
    const wanted = ['jump', 'chute', 'berm', 'drop', 'rockGarden', 'wallride', 'gap'];
    const feats = [];
    if (tr && tr.features) {
      for (const f of tr.features) {
        if (!f || !f.position || !isFinite(f.position.x)) continue;
        if (wanted.indexOf(f.type) < 0) continue;
        feats.push(f);
      }
    }
    // Sample evenly across the run so the reel walks down the mountain.
    const kinds = ['dolly', 'orbit', 'push', 'crane', 'orbit', 'dolly', 'push'];
    for (let i = 0; i < 7; i++) {
      const want = (i + 0.5) / 7;
      let pick = null, bestD = Infinity;
      for (const f of feats) {
        const ft = (f.tStart + f.tEnd) * 0.5;
        const d = Math.abs(ft - want);
        if (d < bestD) { bestD = d; pick = f; }
      }
      const t = pick ? clamp01((pick.tStart + pick.tEnd) * 0.5) : want;
      const side = rng() < 0.5 ? -1 : 1;
      const kind = kinds[i];
      if (kind === 'dolly') {
        // Camera-car shot: in the corridor, tracking down the trail.
        out.push({ type: 'dolly', t, side, lateral: 2.6 + rng() * 1.5, height: 2.1 + rng() * 1.5,
          ahead: 46 + rng() * 26, speed: 2.6 + rng() * 1.8, fov: 44, fov2: 40, dur: 10 });
      } else if (kind === 'orbit') {
        // Aerial: above the canopy, looking down onto the feature.
        out.push({ type: 'orbit', t, radius: 18 + rng() * 10, height: 27 + rng() * 11,
          az: rng() * Math.PI * 2, azSpeed: side * (0.042 + rng() * 0.026), fov: 40, fov2: 46, dur: 11 });
      } else if (kind === 'push') {
        // Long lens across the valley, well clear of everything.
        out.push({ type: 'push', t, side, lateral: 42 + rng() * 26, height: 30 + rng() * 16,
          closeBy: 9 + rng() * 6, fov: 27, fov2: 23, dur: 10 });
      } else {
        // Reveal: starts at the tape, cranes up through the canopy.
        out.push({ type: 'crane', t, side, lateral: 2.8 + rng() * 1.4, height: 2.2,
          rise: 26 + rng() * 8, ahead: 60, fov: 38, fov2: 45, dur: 12 });
      }
    }
    return out;
  }

  function solveCinematic(p, s, rig, dt) {
    if (!shots) { shots = buildShots(); shotIndex = 0; shotTime = 0; }
    const shot = shots[shotIndex % shots.length];
    shotTime += dt;
    const u = clamp01(shotTime / shot.dur);

    const haveTrail = trailLen > 1 && readTrail(shot.t, _trPos, _trTan);
    if (haveTrail) { shotAnchor.copy(_trPos); }
    else { shotAnchor.copy(s.position); _trTan.set(0, 0, -1); }

    _v0.set(_trTan.x, 0, _trTan.z);
    if (_v0.lengthSq() < 1e-6) _v0.set(0, 0, -1); else _v0.normalize();
    _right.crossVectors(_v0, _up).normalize();

    shotLook.copy(shotAnchor); shotLook.y += 1.4;

    switch (shot.type) {
      case 'hero': {
        // Arc around whatever the bike is (the gate at boot, the rider mid-run),
        // on an ellipse stretched along the trail so it stays in the clear.
        shotAnchor.copy(s.position);
        const az = shot.az + shot.azSpeed * shotTime;
        const alongK = Math.cos(az) * shot.along;
        const acrossK = Math.sin(az) * shot.across;
        p.pos.copy(shotAnchor);
        p.pos.addScaledVector(_v0, alongK);
        p.pos.addScaledVector(_right, acrossK);
        p.pos.y += shot.height + Math.sin(shotTime * 0.13) * 0.45;
        const gh = groundAt(p.pos.x, p.pos.z) + 1.5;
        if (p.pos.y < gh) p.pos.y = gh;
        shotLook.copy(shotAnchor); shotLook.y += 1.05;
        break;
      }
      case 'orbit': {
        const az = shot.az + shot.azSpeed * shotTime;
        const ox = shotAnchor.x + Math.sin(az) * shot.radius;
        const oz = shotAnchor.z + Math.cos(az) * shot.radius;
        // Above the canopy relative to the ground it is actually over, not to the
        // trail — on a mountainside those differ by tens of metres.
        p.pos.set(ox, Math.max(shotAnchor.y, groundAt(ox, oz)) + shot.height, oz);
        break;
      }
      case 'dolly': {
        // Track alongside the trail, looking down it. The look point runs ahead at
        // the same rate, so the parallax reads like a camera car.
        //
        // The track FOLLOWS the centreline rather than running straight along the
        // opening tangent: on a straight line the camera walks out of the cleared
        // corridor within a few seconds and finishes the shot inside a tree
        // (measured — it was the only remaining source of bad cinematic frames).
        const travel = shot.speed * shotTime;
        if (readTrail(shot.t + travel / trailLen, _trPos2, _trTan2)) {
          _v1.set(_trTan2.x, 0, _trTan2.z);
          if (_v1.lengthSq() < 1e-6) _v1.copy(_v0); else _v1.normalize();
          _v2.crossVectors(_v1, _up).normalize();
          p.pos.copy(_trPos2).addScaledVector(_v2, shot.side * shot.lateral);
        } else {
          p.pos.copy(shotAnchor).addScaledVector(_v0, travel).addScaledVector(_right, shot.side * shot.lateral);
        }
        p.pos.y = groundAt(p.pos.x, p.pos.z) + shot.height;
        if (readTrail(shot.t + (travel + shot.ahead) / trailLen, _trPos, _trTan)) {
          shotLook.copy(_trPos); shotLook.y += 1.5;
        }
        break;
      }
      case 'push': {
        // Long lens, high and off to the side, easing in over the shot.
        const d = shot.lateral - shot.closeBy * smootherstep(u);
        _v1.copy(shotAnchor).addScaledVector(_right, shot.side * d);
        _v1.y = Math.max(groundAt(_v1.x, _v1.z), shotAnchor.y) + shot.height;
        p.pos.copy(_v1);
        break;
      }
      default: { // crane
        const rise = shot.rise * smootherstep(u);
        _v1.copy(shotAnchor).addScaledVector(_right, shot.side * shot.lateral);
        _v1.y = groundAt(_v1.x, _v1.z) + shot.height + rise;
        p.pos.copy(_v1);
        if (readTrail(shot.t + shot.ahead / trailLen, _trPos2, _trTan2)) {
          shotLook.copy(_trPos2); shotLook.y += 1.4;
        }
        break;
      }
    }

    p.look.copy(shotLook);
    p.anchor.copy(shotAnchor); p.anchor.y += 1.0;
    p.frameHero = false; p.heroR = HERO_R; p.leadX = 0; p.leadY = 0; p.fillMin = 0;
    p.roll = 0;
    p.fov = lerp(shot.fov, shot.fov2, smootherstep(u));
    p.omegaPos = rig.omegaPos; p.omegaY = rig.omegaY;
    p.omegaLook = rig.omegaLook; p.ff = rig.ff;
    p.shake = rig.shake; p.noise = rig.noise;
    p.clearance = rig.clearance; p.avoid = rig.avoid; p.sweep = !!rig.sweep;
    p.treeCap = rig.treeCap !== undefined ? rig.treeCap : 0.5;

    if (shotTime >= shot.dur) {
      shotIndex = (shotIndex + 1) % shots.length;
      shotTime = 0;
      if (solvingActive) requestSnap = true;   // a showreel cuts; it does not fly between setups
    }
  }

  // =========================================================================
  // REPLAY — trackside cameras that hand off down the trail, with a hard cut at
  // each hand-off (the one place the contract explicitly allows one).
  // =========================================================================
  let replayCams = null;
  let replayIndex = -1;
  let replayTime = 0;
  const replayLook = new THREE.Vector3();
  const replayLookVel = new THREE.Vector3();
  let replayHaveLook = false;

  function buildReplayCams() {
    const out = [];
    const N = 14;
    for (let i = 0; i < N; i++) {
      const t0 = (i + 0.5) / N;
      if (!readTrail(t0, _trPos, _trTan)) continue;
      _v0.set(_trTan.x, 0, _trTan.z);
      if (_v0.lengthSq() < 1e-6) _v0.set(0, 0, -1); else _v0.normalize();
      _v1.crossVectors(_v0, _up).normalize();

      // Try both sides and a couple of stand-off distances; keep the one with the
      // best sight line to the trail (highest above the intervening ground).
      let best = null, bestScore = -Infinity;
      for (let k = 0; k < 6; k++) {
        const side = k < 3 ? 1 : -1;
        const lat = 6.5 + (k % 3) * 4.5;
        const px = _trPos.x + _v1.x * side * lat;
        const pz = _trPos.z + _v1.z * side * lat;
        const g = groundAt(px, pz);
        const h = g + 2.1 + (k % 3) * 0.8;
        // Sight line: march to the trail point and find the worst intrusion.
        let worst = 0;
        for (let m = 1; m < 6; m++) {
          const f = m / 6;
          const sx = px + (_trPos.x - px) * f;
          const sz = pz + (_trPos.z - pz) * f;
          const sy = h + (_trPos.y + 1.2 - h) * f;
          worst = Math.max(worst, groundAt(sx, sz) + 0.4 - sy);
        }
        // Prefer a clear line, then some elevation over the trail, then closeness.
        const score = -worst * 6 + (h - _trPos.y) * 0.9 - lat * 0.05;
        if (score > bestScore) {
          bestScore = score;
          best = { x: px, y: h, z: pz, dirX: _v0.x, dirZ: _v0.z, sideX: _v1.x * side, sideZ: _v1.z * side };
        }
      }
      if (!best) continue;
      best.tStart = i / N;
      best.tEnd = (i + 1) / N;
      out.push(best);
    }
    return out;
  }

  function solveReplay(p, s, rig, dt) {
    if (!replayCams) { replayCams = buildReplayCams(); replayIndex = -1; }
    if (!replayCams.length) { solveChase(p, s, RIGS.chaseFar, dt); return; }

    const t = trailTFor(s);
    let want = replayIndex;
    if (want < 0 || t < replayCams[want].tStart || t > replayCams[want].tEnd) {
      want = 0;
      for (let i = 0; i < replayCams.length; i++) {
        if (t >= replayCams[i].tStart) want = i;
      }
    }
    if (want !== replayIndex) {
      replayIndex = want;
      replayTime = 0;
      replayHaveLook = false;
      if (solvingActive) requestSnap = true;   // hand-off: hard cut, like a broadcast switch
    }
    replayTime += dt;
    const c = replayCams[replayIndex];

    // A trackside camera is on a tripod, but a human is holding it — a metre of
    // slow lateral drift over the pass keeps it from looking like a security cam.
    const drift = Math.sin(replayTime * 0.28) * 0.55;
    p.pos.set(c.x + c.sideX * drift, c.y + Math.sin(replayTime * 0.19) * 0.12, c.z + c.sideZ * drift);

    // Operator lag: the aim chases the bike with a lead, and never perfectly.
    _v0.copy(s.position); _v0.y += 1.0;
    _v0.addScaledVector(s.velocity, 0.12);
    if (!replayHaveLook) { replayLook.copy(_v0); replayLookVel.set(0, 0, 0); replayHaveLook = true; }
    const omega = 6.0;
    critDamp(replayLook.x, replayLookVel.x, _v0.x, omega, dt); replayLook.x = _sx; replayLookVel.x = _sv;
    critDamp(replayLook.y, replayLookVel.y, _v0.y, omega * 0.8, dt); replayLook.y = _sx; replayLookVel.y = _sv;
    critDamp(replayLook.z, replayLookVel.z, _v0.z, omega, dt); replayLook.z = _sx; replayLookVel.z = _sv;
    p.look.copy(replayLook);

    // Long lens when the subject is far: keeps the rider a constant size in frame
    // and gives the compressed, telephoto look that broadcast DH has. The 5 m
    // half-height frames the rider at roughly a fifth of frame height; the 22°
    // floor stops it going so long that tracking error and shake dominate.
    const d = p.pos.distanceTo(s.position);
    const fov = clamp(THREE.MathUtils.radToDeg(2 * Math.atan(5.0 / Math.max(4, d))), 22, 55);
    p.fov = fov;

    p.anchor.copy(s.position); p.anchor.y += HERO_UP;
    // Containment plus a gentle broadcast lead. `fillMin` is left at zero: the tripod is
    // placed, not solved, so there is no boom to push back along — the long lens is what
    // controls apparent size here (see the fov solve above).
    p.frameHero = true; p.heroR = HERO_R; p.fillMin = 0;
    p.leadX = leadNow * 0.5;
    p.leadY = leadVNow * 0.5;
    p.roll = 0;
    p.omegaPos = rig.omegaPos; p.omegaY = rig.omegaY;
    p.omegaLook = 60;            // the look point is already damped above
    p.ff = rig.ff;
    // Long lenses magnify shake; that is exactly why it reads as a real camera.
    p.shake = rig.shake * (1 + (56 - fov) / 56);
    p.noise = rig.noise * (1 + (56 - fov) / 40);
    p.clearance = rig.clearance; p.avoid = true; p.sweep = false;  // placed, not solved
    p.treeCap = 0;
  }

  // =========================================================================
  // TERRAIN / OCCLUSION CONSTRAINTS
  // =========================================================================

  /**
   * Push the desired position out of the mountain, then sweep the bike→camera
   * segment. Both responses are held in state that rises immediately and decays
   * slowly, so leaving a berm behind does not snap the camera back.
   */
  function constrain(p, dt) {
    // The framing solver is not a terrain constraint and must run even when there is no
    // terrain and even for modes that want neither the push-out nor the sweep — containment
    // is a guarantee, not a nicety.
    if (!ctx.terrain || (!p.avoid && !p.sweep)) {
      avoidShorten = 0; treeShorten = 0;
      heroLift = damp(heroLift, 0, 1.6, dt);
      composeHero(p);
      return;
    }

    // ---- 1. sweep the segment from the bike to the camera -------------------
    // _v0 is normalised unconditionally: the decaying pull below is persistent
    // state, so a mode change to a no-sweep camera would otherwise spend the next
    // two seconds displacing the camera along a segLen-long "unit" vector.
    _v0.subVectors(p.pos, p.anchor);
    const segLen = _v0.length();
    if (segLen > 1e-4) _v0.multiplyScalar(1 / segLen); else _v0.set(0, 0, 1);
    if (!p.sweep) { avoidShorten = 0; treeShorten = 0; }

    let needShorten = 0;
    if (p.sweep && segLen > 0.5) {
      const STEPS = 8;
      // Two details that matter more than the sweep itself:
      //   * start a quarter of the way out. The anchor is 0.85 m above the tread,
      //     so the first samples are always "too close to the ground" — they are
      //     the ground the bike is standing on.
      //   * ramp the required clearance from near-zero at the bike to the full
      //     value at the camera. A flat requirement made the arm rise out of the
      //     ground steeply enough to trip its own test: measured, it collapsed the
      //     8.4 m crash orbit to 3 m and had the 12 m wide rig one centimetre from
      //     doing the same.
      for (let i = 2; i <= STEPS; i++) {
        const f = i / STEPS;
        const x = p.anchor.x + _v0.x * segLen * f;
        const y = p.anchor.y + _v0.y * segLen * f;
        const z = p.anchor.z + _v0.z * segLen * f;
        const req = lerp(0.30, p.clearance * 0.85, f);
        if (y < groundAt(x, z) + req) {
          // Everything from here out is inside the hill: pull the camera in to
          // just before the intrusion.
          needShorten = Math.max(needShorten, segLen * (1 - (f - 1 / STEPS)));
          break;
        }
      }
    }

    // ---- 2. trunk occlusion -------------------------------------------------
    // vegetation.js clears trees where the trail carved the ground OR within
    // ~3.2–4.7 m of the centreline, so "am I behind a trunk" reduces exactly to
    // "am I outside the cleared corridor" — no tree query needed, and no
    // per-frame scene traversal. Only meaningful when the bike itself is on the
    // trail; off-piste there is no corridor to retreat into and pulling the
    // camera in would just look nervous.
    let needTree = 0;
    if (p.sweep && segLen > 1.0 && bikeOnTrail &&
        corridorClearance(p.pos.x, p.pos.z) < CARVE_CLEAR) {
      const lat = Math.abs(corridorLateral(p.pos));
      if (lat > CORRIDOR_TRIGGER) {
        // The bike is on the centreline, so lateral offset falls off roughly
        // linearly with arm length: to land at CORRIDOR_TARGET metres out, the
        // arm has to come in by this much. Solving it directly beats marching
        // samples back down the arm and costs one nearestT for the whole frame.
        needTree = Math.min(segLen * p.treeCap, segLen * (1 - CORRIDOR_TARGET / lat));
      }
    }

    // Rise instantly, recover slowly (1.4–1.8 /s ≈ 2 s to fully release).
    avoidShorten = needShorten > avoidShorten ? needShorten : damp(avoidShorten, needShorten, 1.8, dt);
    treeShorten = needTree > treeShorten ? needTree : damp(treeShorten, needTree, 1.4, dt);
    const pull = Math.min(segLen * 0.72, avoidShorten + treeShorten);
    if (pull > 0.01 && segLen > 0.5) p.pos.addScaledVector(_v0, -pull);

    // ---- 3. push out along the surface normal ------------------------------
    // Done after the sweep, because shortening the arm can itself drop the camera
    // into a berm face.
    if (p.avoid) {
      const g = groundAt(p.pos.x, p.pos.z);
      const pen = (g + p.clearance) - p.pos.y;
      const need = pen > 0 ? pen : 0;
      avoidLift = need > avoidLift ? need : damp(avoidLift, need, 2.2, dt);
      if (avoidLift > 0.005) {
        normalAt(p.pos.x, p.pos.z, _v1);
        // Along the normal, not straight up: on a steep face that slides the camera
        // out of the slope instead of climbing it, which keeps the framing.
        const ny = Math.max(0.35, _v1.y);
        p.pos.addScaledVector(_v1, avoidLift / ny);
      }
    }

    // ---- 4. crest occlusion, then framing, last -----------------------------
    if (p.frameHero) {
      const need = heroSightNeed(p);
      heroLift = need > heroLift ? need : damp(heroLift, need, 1.5, dt);
      if (heroLift > 0.01) p.pos.y += heroLift;
    } else {
      heroLift = damp(heroLift, 0, 1.6, dt);
    }
    composeHero(p);
  }

  /**
   * How much higher does the camera have to be for the *bottom* of the hero bounds to be
   * visible over the ground between the lens and the bike? Returns metres, never negative.
   *
   * r3_11 is the case: a 12 m boom at 1.6 m over a convex tread rollover, and the rider is
   * buried to the chest by the trail itself. The terrain sweep in step 1 cannot catch this —
   * it tests whether the *camera* is inside the hill, and the camera is fine; what is
   * blocked is the sight line to the wheels.
   *
   * The solve is exact rather than iterative. Parametrise the horizontal path from the hero
   * to the camera by f in (0, 1]. The sight line's height at f is `by + (H - by) * f`. For it
   * to clear a ground sample `g` there we need `H >= by + (g - by) / f`, so the required
   * camera height is a closed form per sample and the answer is the maximum over samples.
   * Sampling starts at HERO_SIGHT_NEAR because below that the divisor is small and the
   * ground being sampled is the ground the bike is standing on.
   */
  function heroSightNeed(p) {
    if (!ctx.terrain) return 0;
    const by = p.anchor.y - HERO_FOOT_BELOW_ANCHOR;
    const dx = p.pos.x - p.anchor.x;
    const dz = p.pos.z - p.anchor.z;
    if (dx * dx + dz * dz < 1.0) return 0;      // helmet-cam distance; nothing to occlude
    const H = p.pos.y;
    let need = 0;
    for (let i = 1; i <= HERO_SIGHT_STEPS; i++) {
      const f = HERO_SIGHT_NEAR + (1 - HERO_SIGHT_NEAR) * (i / (HERO_SIGHT_STEPS + 1));
      const g = groundAt(p.anchor.x + dx * f, p.anchor.z + dz * f) + HERO_SIGHT_MARGIN * f;
      if (g <= by) continue;
      const req = by + (g - by) / f;
      if (req - H > need) need = req - H;
    }
    return clamp(need, 0, HERO_LIFT_MAX);
  }

  /**
   * Hero framing solver (r3 review, G1 + G2). Replaces the old `frameGuard`, which limited
   * the bike's angular offset from the view axis and therefore actively *enforced* the
   * dead-centre composition G2 objects to.
   *
   * Two jobs, in this order:
   *
   *   1. CONTAINMENT. The hero's bounding sphere must fit inside a safe rect — the central
   *      90% of frame width, the central 70% of frame height — with the whole sphere inside,
   *      not just its centre. If it is too big for that rect the camera is pushed back along
   *      its own boom until it fits; if it is outside the rect the aim is rotated until it is
   *      inside. That is the fix for r3_13 (7% of frame, amputated by two edges), r3_12
   *      (front wheel cropped at x = 0) and r3_15.
   *
   *   2. COMPOSITION. Inside that rect, the hero is placed *deliberately* off centre, by
   *      `leadX`/`leadY` shares of the safe half-extent. Horizontally that comes from the
   *      corner and the held camera side; vertically from the fall-line gradient, which is
   *      also what moves the horizon off the midline.
   *
   * All of it runs on the *desired* pose, so the follow springs smooth the result and none
   * of it is ever visible as a snap. It is not a feedback loop: the solvers rebuild `look`
   * from the bike and the trail every frame, so the bias below is applied to a fresh aim
   * rather than compounding on last frame's corrected one.
   *
   * Tolerance, measured rather than assumed: the bounds' angular half-size is evaluated
   * against the *pre-rotation* forward distance, which under-reads it by 1/cos(offset) —
   * about 6% at the widest placement this produces. The safe rect carries 10% of frame width
   * and 30% of frame height of margin, so the worst case over the five reference framings
   * puts the bounding edge at 0.885 of the half-width and 0.717 of the half-height, i.e.
   * still comfortably inside the frame. Nothing is amputated, which is the criterion.
   */
  function composeHero(p) {
    if (!p.frameHero) return;

    // Tangent-space half-extents of the safe rect at this FOV. Working in tangents rather
    // than angles keeps the whole solve to one atan at the end and makes the aspect-ratio
    // term a plain multiply.
    const tanV = Math.tan(clamp(p.fov, 5, 150) * DEG2RAD * 0.5);
    const asp = (cam && isFinite(cam.aspect) && cam.aspect > 0.2) ? cam.aspect : (16 / 9);
    const halfH = tanV * asp * SAFE_H;
    const halfV = tanV * SAFE_V;
    const tight = Math.min(halfH, halfV);

    // ---- 1a. apparent size --------------------------------------------------
    _v3.subVectors(p.anchor, p.pos);
    let d = _v3.length();
    if (d < 1e-3) return;

    // Distance at which the bounds exactly fill the allowed share of the safe rect. Closer
    // than `dForMaxFill` the hero is too big to be contained; further than `dForMinFill`
    // (only set by the QA hero presets) it is too small to be the subject of the shot.
    const dForMaxFill = p.heroR / (HERO_FILL_MAX * tight);
    const dForMinFill = p.fillMin > 0 ? p.heroR / (p.fillMin * tight) : 0;
    let want = d;
    if (d < dForMaxFill) want = dForMaxFill;
    else if (dForMinFill > 0 && d > dForMinFill) want = dForMinFill;
    if (Math.abs(want - d) > 0.02) {
      const move = clamp(want - d, -HERO_PUSH_MAX, HERO_PUSH_MAX);
      // Along the hero→camera direction, so the framing angle is preserved as it moves.
      p.pos.addScaledVector(_v3, -move / d);
      // Backing off can drop the lens into a berm behind it; the ground clamp is cheap and
      // this is the only position change that happens after the main push-out.
      const g = groundAt(p.pos.x, p.pos.z) + p.clearance;
      if (p.pos.y < g) p.pos.y = g;
      _v3.subVectors(p.anchor, p.pos);
      d = _v3.length();
      if (d < 1e-3) return;
    }

    // ---- 1b. view basis -----------------------------------------------------
    _v0.subVectors(p.look, p.pos);
    const dLook = _v0.length();
    if (dLook < 0.05) return;
    _v0.multiplyScalar(1 / dLook);                 // view axis
    _v1.crossVectors(_v0, _up);
    if (_v1.lengthSq() < 1e-8) return;             // straight up or down; leave it alone
    _v1.normalize();                               // camera right
    _v2.crossVectors(_v1, _v0).normalize();        // camera up

    const fwdD = _v3.dot(_v0);
    // Behind the lens, or so far off axis that the tangent parametrisation is meaningless.
    // The springs are mid-transition; the output-side guard in lateUpdate() is the backstop.
    if (fwdD < 0.35) return;

    // Angular half-size of the bounds, in the same tangent units.
    const hs = p.heroR / fwdD;
    const limX = Math.max(0.01, halfH - hs);
    const limY = Math.max(0.01, halfV - hs);

    // ---- 2. placement -------------------------------------------------------
    // Target tangent coordinates: this frame's own coordinates, biased by the composition
    // lead, then clamped so the whole sphere stays inside the safe rect. The lead is a bias
    // on a *freshly solved* aim, not an accumulator — the mode solvers rebuild `look` from
    // the bike and the trail every frame, so there is no feedback path from last frame's
    // corrected aim into this one.
    const X0 = _v3.dot(_v1) / fwdD;
    const Y0 = _v3.dot(_v2) / fwdD;
    const Xt = clamp(X0 + p.leadX * halfH, -limX, limX);
    const Yt = clamp(Y0 + p.leadY * halfV, -limY, limY);
    if (Math.abs(Xt - X0) < 1e-4 && Math.abs(Yt - Y0) < 1e-4) return;

    // Rotate the axis toward the hero by exactly the error. To first order that moves the
    // hero to the target; the basis itself rotates with the axis, so a single step leaves a
    // residual of a few percent of the offset — enough, at a 30° offset on the tightest QA
    // preset, to push the bounding edge back outside the 5% margin this solver exists to
    // guarantee. The second step takes it under a tenth of a percent, i.e. inside a pixel.
    for (let it = 0; it < 2; it++) {
      const fd = _v3.dot(_v0);
      if (fd < 0.35) break;
      const X = _v3.dot(_v1) / fd;
      const Y = _v3.dot(_v2) / fd;
      _v4.copy(_v0).addScaledVector(_v1, X - Xt).addScaledVector(_v2, Y - Yt);
      if (_v4.lengthSq() < 1e-8) break;
      _v0.copy(_v4).normalize();
      _v1.crossVectors(_v0, _up);
      if (_v1.lengthSq() < 1e-8) break;
      _v1.normalize();
      _v2.crossVectors(_v1, _v0).normalize();
    }
    p.look.copy(p.pos).addScaledVector(_v0, dLook);
  }

  // =========================================================================
  // PHOTO MODE
  // =========================================================================

  function onPhotoKey(e) {
    if (!photo) return;
    const el = document.activeElement;
    if (el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.isContentEditable)) return;
    if (e.type === 'keydown') phKeys.add(e.code); else phKeys.delete(e.code);
  }
  function onPhotoBlur() { phKeys.clear(); dragging = false; }
  function onPointerDown(e) {
    if (!photo || e.button !== 0) return;
    dragging = true;
    const el = ctx.container || ctx.renderer?.domElement;
    if (el && el.requestPointerLock && !pointerLocked) {
      try { el.requestPointerLock(); } catch (err) { /* user gesture rules; drag still works */ }
    }
  }
  function onPointerUp() { dragging = false; }
  function onPointerMove(e) {
    if (!photo) return;
    if (!pointerLocked && !dragging) return;
    const inv = (ctx.settings && ctx.settings.invertLook) ? -1 : 1;
    const k = 0.0022;
    phYawT -= (e.movementX || 0) * k;
    phPitchT -= (e.movementY || 0) * k * inv;
    phPitchT = clamp(phPitchT, -1.45, 1.45);
  }
  function onPointerLockChange() {
    const el = ctx.container || ctx.renderer?.domElement;
    pointerLocked = !!document.pointerLockElement &&
      (document.pointerLockElement === el || (el && el.contains && el.contains(document.pointerLockElement)));
  }
  function onWheel(e) {
    if (!photo) return;
    phFovT = clamp(phFovT + Math.sign(e.deltaY) * 2.0, 14, 100);
    e.preventDefault();
  }

  function enterPhoto() {
    phPos.copy(cam.position);
    phVel.set(0, 0, 0);
    _eul.setFromQuaternion(cam.quaternion, 'YXZ');
    phYaw = phYawT = _eul.y;
    phPitch = phPitchT = _eul.x;
    phRoll = 0;
    phFov = phFovT = cam.fov;
    phKeys.clear();
  }

  function exitPhoto() {
    if (document.pointerLockElement) { try { document.exitPointerLock(); } catch (e) { /* ignore */ } }
    phKeys.clear();
    dragging = false;
    // Fly back to the game camera on the springs, starting from wherever the shot
    // was framed — spPos/spLook were parked there every frame of photo mode.
    spVel.set(0, 0, 0); spLookVel.set(0, 0, 0);
    tgtVel.set(0, 0, 0); tgtLookVel.set(0, 0, 0);
    // The target-velocity estimate is a finite difference against last frame's
    // pose, which is now meaningless; skip it for one frame or the feed-forward
    // term throws the camera several metres.
    skipVelFrame = true;
    // ...but if the shot wandered off across the valley, a half-second flight back
    // at 300 m/s is far worse than an honest cut.
    const bs = ctx.bike && ctx.bike.state;
    if (!bs || !bs.position || spPos.distanceToSquared(bs.position) > 25 * 25) requestSnap = true;
  }

  function setPhoto(on, emitEvent) {
    const want = !!on;
    if (want === photo) return;
    photo = want;
    if (want) enterPhoto(); else exitPhoto();
    ctx.photoMode = photo;
    if (ctx.settings) ctx.settings.photoMode = photo;
    if (emitEvent !== false && ctx.events) ctx.events.emit('photo:mode', { enabled: photo });
  }

  const PH_KEY = {
    fwd: ['KeyW', 'ArrowUp'], back: ['KeyS', 'ArrowDown'],
    left: ['KeyA', 'ArrowLeft'], right: ['KeyD', 'ArrowRight'],
    up: ['Space', 'KeyE'], down: ['KeyQ', 'ControlLeft'],
  };
  const anyKey = (list) => { for (let i = 0; i < list.length; i++) if (phKeys.has(list[i])) return true; return false; };

  function updatePhoto(dt) {
    // Orientation: damped toward the accumulated mouse target so even a flicked
    // mouse produces a smooth arc.
    phYaw = damp(phYaw, phYawT, 13, dt);
    phPitch = damp(phPitch, phPitchT, 13, dt);
    phFov = damp(phFov, phFovT, 9, dt);

    const boost = (phKeys.has('ShiftLeft') || phKeys.has('ShiftRight')) ? 4.0
      : (phKeys.has('AltLeft') || phKeys.has('AltRight')) ? 0.22 : 1.0;
    const spd = 7.5 * boost;

    _eul.set(phPitch, phYaw, 0, 'YXZ');
    _q0.setFromEuler(_eul);
    _fwd.set(0, 0, -1).applyQuaternion(_q0);
    _right.set(1, 0, 0).applyQuaternion(_q0);

    _v0.set(0, 0, 0);
    if (anyKey(PH_KEY.fwd)) _v0.add(_fwd);
    if (anyKey(PH_KEY.back)) _v0.sub(_fwd);
    if (anyKey(PH_KEY.right)) _v0.add(_right);
    if (anyKey(PH_KEY.left)) _v0.sub(_right);
    if (anyKey(PH_KEY.up)) _v0.y += 1;
    if (anyKey(PH_KEY.down)) _v0.y -= 1;
    if (_v0.lengthSq() > 1e-6) _v0.normalize().multiplyScalar(spd); else _v0.set(0, 0, 0);

    // Heavily damped velocity — a photo camera should glide, never twitch.
    phVel.x = damp(phVel.x, _v0.x, 4.5, dt);
    phVel.y = damp(phVel.y, _v0.y, 4.5, dt);
    phVel.z = damp(phVel.z, _v0.z, 4.5, dt);
    phPos.addScaledVector(phVel, dt);

    // Stay above the mountain and inside the world.
    const t = ctx.terrain;
    if (t) {
      if (t.bounds) {
        phPos.x = clamp(phPos.x, t.bounds.minX + 4, t.bounds.maxX - 4);
        phPos.z = clamp(phPos.z, t.bounds.minZ + 4, t.bounds.maxZ - 4);
      }
      const g = groundAt(phPos.x, phPos.z) + 0.55;
      if (phPos.y < g) { phPos.y = g; if (phVel.y < 0) phVel.y = 0; }
    }

    cam.position.copy(phPos);
    cam.quaternion.copy(_q0);
    if (Math.abs(phRoll) > 1e-5) {
      _q0.setFromAxisAngle(_v1.set(0, 0, 1), phRoll);
      cam.quaternion.multiply(_q0);
    }
    applyFov(phFov);
    // Keep the follow springs parked on the camera so leaving photo mode starts
    // from where the shot was, not from wherever the bike dragged them.
    spPos.copy(phPos);
    spLook.copy(phPos).addScaledVector(_fwd, 10);
    rollNow = 0; rollVel = 0;
    cam.updateMatrixWorld();
  }

  // =========================================================================
  // FOV plumbing — updateProjectionMatrix only when it actually changes.
  // =========================================================================
  function baseFov() {
    const f = ctx.settings && ctx.settings.fov;
    return (typeof f === 'number' && f > 20 && f < 140) ? f : 62;
  }
  function applyFov(v) {
    const f = clamp(v, 12, 120);
    // First person puts the lens centimetres from the rider's own forearms, and
    // the stock 0.1 m near plane saws visible slices off them. Pull it in for
    // that rig only — tightening it globally would cost depth precision across
    // an 8 km world for a problem only the helmet cam has.
    const near = mode === 'firstPerson' ? 0.02 : 0.1;
    if (Math.abs(f - cam.fov) > 0.004 || cam.near !== near) {
      cam.fov = f;
      cam.near = near;
      cam.updateProjectionMatrix();
    }
  }

  // =========================================================================
  // Events
  // =========================================================================
  const offs = [];
  if (ctx.events) {
    offs.push(ctx.events.on('bike:impact', (e) => {
      if (!e) return;
      const sev = clamp01(e.severity || 0);
      if (sev <= 0.02) return;
      // Directional: the ground drives the bike along +normal, so the camera
      // (which is trailing a sprung mass) lurches the other way first.
      const n = e.normal;
      const mag = 2.7 * Math.pow(sev, 1.15);
      if (n && isFinite(n.x)) {
        pendImpulse.x -= n.x * mag;
        pendImpulse.y -= n.y * mag;
        pendImpulse.z -= n.z * mag;
      } else {
        pendImpulse.y -= mag;
      }
      // Angular kick: pitch scales with how head-on the hit was, roll/yaw take a
      // deterministic-ish sign so repeated hits do not resonate.
      const vert = n && isFinite(n.y) ? clamp01(Math.abs(n.y)) : 1;
      pendPitch += 1.55 * sev * (0.45 + vert * 0.55);
      const sgn = (rng() < 0.5) ? -1 : 1;
      pendRollImp += sgn * 0.95 * sev;
      pendYawImp += -sgn * 0.60 * sev;
      pendCount++;
    }));

    offs.push(ctx.events.on('run:crash', (e) => {
      if (e && e.position && isFinite(e.position.x)) crashAnchor.copy(e.position);
      else {
        const s = ctx.bike && ctx.bike.state;
        if (s) crashAnchor.copy(s.position);
      }
      crashAz = Math.atan2(cam.position.x - crashAnchor.x, cam.position.z - crashAnchor.z);
      wasCrashed = true;
    }));

    // The menu owns a photo toggle too; keep both in step without a feedback loop.
    offs.push(ctx.events.on('photo:mode', (e) => {
      if (!e) return;
      const want = !!e.enabled;
      if (want !== photo) setPhoto(want, false);
    }));

    offs.push(ctx.events.on('run:start', () => {
      // A fresh run should not inherit the menu's shot list state.
      shotTime = 0; shotIndex = 0;
      replayIndex = -1;
      crashW = 0; wasCrashed = false;
    }));
  }

  window.addEventListener('keydown', onPhotoKey);
  window.addEventListener('keyup', onPhotoKey);
  window.addEventListener('blur', onPhotoBlur);
  window.addEventListener('pointerdown', onPointerDown);
  window.addEventListener('pointerup', onPointerUp);
  window.addEventListener('pointermove', onPointerMove);
  window.addEventListener('wheel', onWheel, { passive: false });
  document.addEventListener('pointerlockchange', onPointerLockChange);

  // =========================================================================
  // MODE MANAGEMENT
  // =========================================================================
  function setMode(m, opts) {
    if (!RIGS[m] || m === mode) return;
    // Freeze the outgoing pose so the blend has something stable to leave from.
    copyPose(poseA, poseB);
    prevMode = mode;
    mode = m;
    blendT = 0;
    // Nothing has been solved yet (boot-time setMode from gameplay/menu): there
    // is no outgoing pose to blend from, so take the new one whole.
    if (firstFrame) { blendT = 1; requestSnap = true; }
    // Replay is a broadcast switch, not a camera move — cut both into and out of
    // it. Blending out of a 22° lens 40 m from the trail would be a rocket flight.
    if (m === 'replay' || prevMode === 'replay' || (opts && opts.cut)) {
      blendT = 1; requestSnap = true;
    }
    if (m === 'cinematic') { shotTime = 0; }
    if (m === 'replay') { replayIndex = -1; replayHaveLook = false; }
  }

  function solveFor(m, p, s, dt) {
    switch (m) {
      case 'firstPerson': solveFirstPerson(p, s, RIGS.firstPerson, dt); break;
      case 'cinematic': solveCinematic(p, s, RIGS.cinematic, dt); break;
      case 'replay': solveReplay(p, s, RIGS.replay, dt); break;
      case 'chaseFar': solveChase(p, s, RIGS.chaseFar, dt); break;
      default: solveChase(p, s, RIGS.chase, dt); break;
    }
  }

  function snap() {
    spPos.copy(pose.pos); spVel.set(0, 0, 0);
    spLook.copy(pose.look); spLookVel.set(0, 0, 0);
    prevTargetPos.copy(pose.pos); prevTargetLook.copy(pose.look);
    tgtVel.set(0, 0, 0); tgtLookVel.set(0, 0, 0);
    rollNow = pose.roll; rollVel = 0;
    fovNow = pose.fov; fovVel = 0; fovKick = 0; fovKickVel = 0;
    haveHistory = true;
  }

  // =========================================================================
  // FRAME
  // =========================================================================
  // The rider hides its own head when the lens gets close (first person, or a
  // photo-mode fly-through). Distance is the whole criterion: it covers the
  // 0.6 s mode blend, the crash orbit pulling out to show the ragdoll, and
  // hard cuts — all without knowing which mode asked. Runs after the camera is
  // final for the frame, so a cut never flashes the helmet interior.
  function publishHeadProximity() {
    const r = ctx.rider;
    if (!r || typeof r.setHeadProximity !== 'function' ||
        typeof r.getHeadPosition !== 'function') return;
    r.getHeadPosition(_v0);
    r.setHeadProximity(isFinite(_v0.x) ? cam.position.distanceTo(_v0) : Infinity);
  }

  function lateUpdate(rawDt, c) {
    if (!cam) return;
    const dt = clamp(rawDt || 0, 0, MAX_DT);
    const time = (c && c.time) || 0;

    if (ctx.trail && isFinite(ctx.trail.length) && ctx.trail.length > 1) trailLen = ctx.trail.length;

    const inp = (c && c.input && c.input.state) || null;

    // ---- discrete input ----------------------------------------------------
    const pEdge = !!(inp && inp.photoMode);
    if (pEdge && !photoEdge) setPhoto(!photo, true);
    photoEdge = pEdge;

    const cEdge = !!(inp && inp.cameraCycle);
    // No cycling out of the title cinematic: it is not in CYCLE, and a stray C
    // on the menu screen should not replace the shot list with a chase cam.
    if (cEdge && !cycleEdge && !photo && mode !== 'cinematic') {
      const i = CYCLE.indexOf(mode);
      setMode(i < 0 ? CYCLE[0] : CYCLE[(i + 1) % CYCLE.length]);
    }
    cycleEdge = cEdge;

    if (photo) { updatePhoto(dt); publishHeadProximity(); return; }

    const s = bikeState();
    // Cached for constrain(): the trunk response only makes sense when there is a
    // corridor for the camera to retreat into.
    bikeOnTrail = Math.abs(s.lateral || 0) < 5.0 &&
      corridorClearance(s.position.x, s.position.z) > 0.05;

    // ---- crash weight ------------------------------------------------------
    const crashed = !!s.crashed;
    if (crashed && !wasCrashed) {
      crashAnchor.copy(s.position);
      crashAz = Math.atan2(cam.position.x - crashAnchor.x, cam.position.z - crashAnchor.z);
    }
    wasCrashed = crashed;
    const crashTarget = (crashed && mode !== 'cinematic' && mode !== 'replay') ? 1 : 0;
    crashW = crashTarget > crashW
      ? Math.min(1, crashW + dt / CRASH_BLEND_IN)
      : Math.max(0, crashW - dt / CRASH_BLEND_OUT);

    // ---- solve the target pose --------------------------------------------
    if (blendT < 1) {
      blendT = Math.min(1, blendT + dt / MODE_BLEND);
      solvingActive = false;
      solveFor(prevMode, poseA, s, dt);
      solvingActive = true;
      solveFor(mode, poseB, s, dt);
      blendPose(poseA, poseB, smootherstep(blendT), pose);
    } else {
      solvingActive = true;
      solveFor(mode, poseB, s, dt);
      copyPose(pose, poseB);
    }

    constrain(pose, dt);

    // ---- follow springs ----------------------------------------------------
    if (firstFrame || !haveHistory) { snap(); firstFrame = false; }
    if (requestSnap) { snap(); requestSnap = false; }

    // Target velocity, finite-differenced and smoothed. A critically damped
    // spring lags a ramp input by 2*v/omega; feeding a fraction of the target's
    // own velocity forward cancels that lag, which is exactly the difference
    // between "follows the bike" and "rubber-bands behind the bike".
    if (dt > 1e-5 && !skipVelFrame) {
      const inv = 1 / dt;
      _v0.subVectors(pose.pos, prevTargetPos).multiplyScalar(inv);
      if (_v0.lengthSq() > 45 * 45) _v0.setLength(45);
      tgtVel.lerp(_v0, 1 - Math.exp(-14 * dt));
      _v1.subVectors(pose.look, prevTargetLook).multiplyScalar(inv);
      if (_v1.lengthSq() > 60 * 60) _v1.setLength(60);
      tgtLookVel.lerp(_v1, 1 - Math.exp(-14 * dt));
    }
    skipVelFrame = false;
    prevTargetPos.copy(pose.pos);
    prevTargetLook.copy(pose.look);

    const wp = pose.omegaPos, wy = pose.omegaY, wl = pose.omegaLook;
    const ffP = pose.ff * 2;
    _v2.set(
      pose.pos.x + tgtVel.x * (ffP / Math.max(0.5, wp)),
      pose.pos.y + tgtVel.y * (ffP / Math.max(0.5, wy)),
      pose.pos.z + tgtVel.z * (ffP / Math.max(0.5, wp)));

    critDamp(spPos.x, spVel.x, _v2.x, wp, dt); spPos.x = _sx; spVel.x = _sv;
    critDamp(spPos.y, spVel.y, _v2.y, wy, dt); spPos.y = _sx; spVel.y = _sv;
    critDamp(spPos.z, spVel.z, _v2.z, wp, dt); spPos.z = _sx; spVel.z = _sv;

    _v3.set(
      pose.look.x + tgtLookVel.x * (ffP / Math.max(0.5, wl)),
      pose.look.y + tgtLookVel.y * (ffP / Math.max(0.5, wl)),
      pose.look.z + tgtLookVel.z * (ffP / Math.max(0.5, wl)));
    critDamp(spLook.x, spLookVel.x, _v3.x, wl, dt); spLook.x = _sx; spLookVel.x = _sv;
    critDamp(spLook.y, spLookVel.y, _v3.y, wl, dt); spLook.y = _sx; spLookVel.y = _sv;
    critDamp(spLook.z, spLookVel.z, _v3.z, wl, dt); spLook.z = _sx; spLookVel.z = _sv;

    // ---- hard floor --------------------------------------------------------
    // The spring can undershoot into a rise between frames. Clamp the *output*
    // and write the clamp back into the spring so it does not fight the surface.
    if (ctx.terrain && mode !== 'firstPerson') {
      const floor = groundAt(spPos.x, spPos.z) + Math.max(0.45, pose.clearance * 0.5);
      if (spPos.y < floor) {
        spPos.y = floor;
        if (spVel.y < 0) spVel.y = 0;
      }
    }

    // ---- roll --------------------------------------------------------------
    critDamp(rollNow, rollVel, pose.roll, 5.4, dt); rollNow = _sx; rollVel = _sv;

    // ---- orientation -------------------------------------------------------
    _fwd.subVectors(spLook, spPos);
    if (_fwd.lengthSq() < 1e-8) _fwd.set(0, 0, -1);
    _fwd.normalize();

    // Output-side containment backstop. composeHero() places the hero inside the safe rect
    // on the *desired* pose, but on a tight corner at 22 m/s the position and look springs
    // lag by different amounts and the bounds can still cross a frame edge — measured at
    // 0.6% of frames on a full-length synthetic run before a guard was added here.
    //
    // The limit is the safe rect's own corner rather than a fixed angle. That matters now:
    // the old constant (min(0.42, fov*0.0074) ≈ 24° at 62°) was *tighter* than the safe
    // rect it is meant to protect (≈37° at the same FOV with the hero 6 m out), so keeping
    // it would have quietly cancelled the composition offset every frame — i.e. it would
    // have re-created exactly the dead-centre framing G2 objects to.
    if (pose.frameHero) {
      _v0.subVectors(pose.anchor, spPos);
      const dB = _v0.length();
      if (dB > 0.75) {
        _v0.multiplyScalar(1 / dB);
        const tanV = Math.tan(clamp(fovNow, 5, 150) * DEG2RAD * 0.5);
        const asp = (isFinite(cam.aspect) && cam.aspect > 0.2) ? cam.aspect : (16 / 9);
        const hs = pose.heroR / dB;
        const limX = Math.max(0.02, tanV * asp * SAFE_H - hs);
        const limY = Math.max(0.02, tanV * SAFE_V - hs);
        // Corner of the safe rect, plus ~1.7° of slack so the backstop only ever catches
        // genuine overshoot and never argues with the solver about the last degree.
        const maxOut = Math.atan(Math.sqrt(limX * limX + limY * limY)) + 0.03;
        const ang = Math.acos(clamp(_v0.dot(_fwd), -1, 1));
        if (ang > maxOut) {
          _fwd.lerp(_v0, 1 - maxOut / ang);
          if (_fwd.lengthSq() > 1e-8) _fwd.normalize(); else _fwd.copy(_v0);
        }
      }
    }

    _right.crossVectors(_fwd, _up);
    if (_right.lengthSq() < 1e-8) _right.set(1, 0, 0); else _right.normalize();
    _camUp.crossVectors(_right, _fwd).normalize();
    if (Math.abs(rollNow) > 1e-5) {
      const cr = Math.cos(rollNow), sr = Math.sin(rollNow);
      _camUp.set(
        _camUp.x * cr + _right.x * sr,
        _camUp.y * cr + _right.y * sr,
        _camUp.z * cr + _right.z * sr).normalize();
    }
    _v4.copy(spPos).add(_fwd);
    _m4.lookAt(spPos, _v4, _camUp);
    cam.quaternion.setFromRotationMatrix(_m4);
    cam.position.copy(spPos);

    // ---- handheld noise + impact shake -------------------------------------
    applyShake(dt, time, s, pose);

    // ---- FOV ---------------------------------------------------------------
    updateFov(dt, s, pose);

    publishHeadProximity();

    cam.updateMatrixWorld();
  }

  // -------------------------------------------------------------------------
  // Shake: layered low-frequency drift (scaled by what the wheels are doing)
  // plus a sprung, directional impulse from impacts.
  // -------------------------------------------------------------------------
  function applyShake(dt, time, s, p) {
    // ---- consume queued impulses ------------------------------------------
    if (pendCount > 0) {
      const k = p.shake;
      shakeVel.addScaledVector(pendImpulse, k);
      shPitchV += pendPitch * k;
      shRollV += pendRollImp * k;
      shYawV += pendYawImp * k;
      pendImpulse.set(0, 0, 0);
      pendPitch = pendRollImp = pendYawImp = 0;
      pendCount = 0;
      // Clamp so a pile-up of hits cannot launch the camera.
      if (shakeVel.lengthSq() > 25) shakeVel.setLength(5);
      shPitchV = clamp(shPitchV, -4, 4);
      shRollV = clamp(shRollV, -3, 3);
      shYawV = clamp(shYawV, -3, 3);
    }

    // ---- integrate the oscillators ----------------------------------------
    // zeta ~0.4 gives two visible rings then gone: a thump, not a wobble.
    oscDecay(shakePos.x, shakeVel.x, 33, 0.40, dt); shakePos.x = _sx; shakeVel.x = _sv;
    oscDecay(shakePos.y, shakeVel.y, 33, 0.40, dt); shakePos.y = _sx; shakeVel.y = _sv;
    oscDecay(shakePos.z, shakeVel.z, 33, 0.40, dt); shakePos.z = _sx; shakeVel.z = _sv;
    oscDecay(shPitch, shPitchV, 27, 0.42, dt); shPitch = _sx; shPitchV = _sv;
    oscDecay(shRoll, shRollV, 25, 0.45, dt); shRoll = _sx; shRollV = _sv;
    oscDecay(shYaw, shYawV, 29, 0.48, dt); shYaw = _sx; shYawV = _sv;

    // ---- handheld perlin drift --------------------------------------------
    const speed = Math.max(0, s.speed || 0);
    // Hard requirement: zero shake when stationary.
    const moving = smoothstep(0.4, 3.2, speed);
    const rough = clamp01(s.roughness !== undefined ? s.roughness : 0.25) * (s.airborne ? 0.25 : 1);
    const gEx = clamp01(Math.abs((s.gForce !== undefined ? s.gForce : 1) - 1) / 2.2);
    const sn = clamp01(speed / SPEED_REF);
    const amp = p.noise * moving * (0.0016 + rough * 0.0068 + gEx * 0.0060 + sn * 0.0026);

    let nYaw = 0, nPitch = 0, nRoll = 0, nx = 0, ny = 0;
    if (amp > 1e-6) {
      // Three octaves: a slow float, a mid sway, and a fine tremor. The slow one
      // carries most of the amplitude — that is what makes it read as a person
      // holding a camera rather than a vibration.
      const t1 = time * 0.23, t2 = time * 0.79, t3 = time * 2.10;
      nYaw = noise2(t1, 11.3) * 1.00 + noise2(t2, 12.7) * 0.50 + noise2(t3, 13.9) * 0.18;
      nPitch = noise2(t1, 47.9) * 1.00 + noise2(t2, 49.1) * 0.50 + noise2(t3, 50.3) * 0.18;
      nRoll = noise2(t1, 83.1) * 1.00 + noise2(t2, 84.3) * 0.42;
      nx = noise2(t1 * 0.8, 131.7) * 1.0 + noise2(t2, 133.1) * 0.4;
      ny = noise2(t1 * 0.8, 167.3) * 1.0 + noise2(t2, 168.9) * 0.4;
      nYaw *= amp; nPitch *= amp * 0.85; nRoll *= amp * 1.25;
      nx *= amp * 6.5; ny *= amp * 5.0;
    }

    // ---- apply -------------------------------------------------------------
    const yaw = shYaw + nYaw;
    const pitch = shPitch + nPitch;
    const roll = shRoll + nRoll;
    if (Math.abs(yaw) + Math.abs(pitch) + Math.abs(roll) > 1e-6) {
      _eul.set(pitch, yaw, roll, 'YXZ');
      _q0.setFromEuler(_eul);
      cam.quaternion.multiply(_q0);          // local space: shake rides the camera
    }

    // Positional shake stays in world space (the oscillator is world space), the
    // perlin offset is camera-local so it never pushes the camera into the hill.
    if (shakePos.lengthSq() > 1e-8) cam.position.add(shakePos);
    if (nx !== 0 || ny !== 0) {
      _v0.set(nx, ny, 0).applyQuaternion(cam.quaternion);
      cam.position.add(_v0);
    }
  }

  // -------------------------------------------------------------------------
  // FOV: a lagged spring plus an acceleration kick, so speed *changes* read as
  // fast rather than just steady-state speed reading as wide.
  // -------------------------------------------------------------------------
  function updateFov(dt, s, p) {
    const speed = Math.max(0, s.speed || 0);
    const accel = dt > 1e-5 ? (speed - lastSpeed) / dt : 0;
    lastSpeed = speed;

    if (accel > 1.5) fovKickV(accel);
    oscDecay(fovKick, fovKickVel, 6.2, 0.85, dt); fovKick = _sx; fovKickVel = _sv;

    // Asymmetric: widen quickly under power, close slowly on the brakes. Closing
    // fast makes braking feel like a stall; closing slowly makes it feel like
    // you carried the speed.
    const rising = p.fov > fovNow;
    const w = (mode === 'cinematic' || mode === 'replay') ? 6.0 : (rising ? 2.9 : 1.55);
    critDamp(fovNow, fovVel, p.fov, w, dt); fovNow = _sx; fovVel = _sv;

    applyFov(fovNow + clamp(fovKick, -3, 5));
  }
  function fovKickV(accel) {
    fovKickVel += Math.min(accel, 14) * 0.20;
    if (fovKickVel > 9) fovKickVel = 9;
  }

  // =========================================================================
  // SHOT SOLVERS — pure, camera-free (r3 review G1 + G3)
  //
  // These exist because the review set is posed by `qa/harness.js`, which freezes this
  // module. They run the same containment, crest-lift and composition code the gameplay
  // camera runs, so the framing guarantees hold for a QA capture as well as for a frame the
  // player sees. Neither touches `ctx.camera`, the follow springs or any follow state.
  // =========================================================================
  const shotPose = makePose();
  const shotResult = {
    position: new THREE.Vector3(),
    lookAt: new THREE.Vector3(),
    fov: 55,
    pushedBack: 0,          // metres the containment solver had to back the camera off
    liftedOverCrest: 0,     // metres it had to climb to see the wheels
    // Where the hero bounds ended up, in NDC (-1..1, +x right, +y up), and their angular
    // half-size as a share of the half-frame height. The harness can assert on these.
    heroScreen: { x: 0, y: 0, halfSize: 0 },
  };

  const _estA = new THREE.Vector3();
  const _estB = new THREE.Vector3();
  const _estM = new THREE.Vector3();
  const _estFall = new THREE.Vector3();
  const _estSide = new THREE.Vector3();
  const _estDir = new THREE.Vector3();

  function aspectNow() {
    return (cam && isFinite(cam.aspect) && cam.aspect > 0.2) ? cam.aspect : (16 / 9);
  }

  /** Fill in shotResult.heroScreen from the solved pose. */
  function measureHeroScreen(p) {
    const hs = shotResult.heroScreen;
    hs.x = 0; hs.y = 0; hs.halfSize = 0;
    _v0.subVectors(p.look, p.pos);
    if (_v0.lengthSq() < 1e-8) return;
    _v0.normalize();
    _v1.crossVectors(_v0, _up);
    if (_v1.lengthSq() < 1e-8) return;
    _v1.normalize();
    _v2.crossVectors(_v1, _v0).normalize();
    _v3.subVectors(p.anchor, p.pos);
    const fwdD = _v3.dot(_v0);
    if (fwdD < 1e-3) return;
    const tanV = Math.tan(clamp(p.fov, 5, 150) * DEG2RAD * 0.5);
    hs.x = (_v3.dot(_v1) / fwdD) / (tanV * aspectNow());
    hs.y = (_v3.dot(_v2) / fwdD) / tanV;
    hs.halfSize = (p.heroR / fwdD) / tanV;
  }

  /**
   * Solve a hero shot: a camera that is actually able to photograph the bike and rider.
   *
   * Options (all optional): `t` along the trail, `back`/`up`/`side` in metres, `ahead` in
   * metres for the aim, `fov` in degrees, `azimuth` in degrees off dead astern (+ = the
   * camera swings to the rider's right; G1 asks for 15–25° on the low-rear preset), `fill`
   * = the smallest share of the safe half-height the hero bounds are allowed to occupy
   * (0.55–0.70 for a portrait, 0 to leave the framing to the boom length), and `leadX`/
   * `leadY` to override the composition offset.
   *
   * `back`/`up`/`side` are a *starting suggestion*, not a result: the solver will back the
   * camera off if the bounds cannot be contained, pull it in if they are smaller than
   * `fill`, and lift it over a crest that hides the wheels.
   */
  function solveHeroShot(opts) {
    const o = opts || {};
    const t = clamp01(o.t !== undefined ? o.t : 0.4);
    const back = o.back !== undefined ? o.back : 6.5;
    const upM = o.up !== undefined ? o.up : 1.9;
    const side = o.side || 0;
    const aheadM = o.ahead !== undefined ? o.ahead : 14;
    const fov = clamp(o.fov !== undefined ? o.fov : 55, 12, 120);
    const azDeg = o.azimuth !== undefined ? o.azimuth : 0;

    shotResult.fov = fov;
    shotResult.pushedBack = 0;
    shotResult.liftedOverCrest = 0;

    const haveTrail = trailLen > 1 && readTrail(t, _trPos, _trTan);
    if (!haveTrail) {
      // No trail: frame whatever the bike is, from behind and slightly off axis.
      const s = bikeState();
      _trPos.copy(s.position);
      _trTan.copy(s.forward);
    }
    _v0.set(_trTan.x, 0, _trTan.z);
    if (_v0.lengthSq() < 1e-6) _v0.set(0, 0, -1); else _v0.normalize();
    _right.crossVectors(_v0, _up).normalize();

    // The harness seats the bike on the trail sample or the terrain, whichever is higher.
    const baseY = Math.max(_trPos.y, groundAt(_trPos.x, _trPos.z)) + 0.05;

    shotPose.anchor.set(_trPos.x, baseY + HERO_UP, _trPos.z);
    shotPose.heroR = HERO_R;
    shotPose.frameHero = true;
    shotPose.fov = fov;
    shotPose.clearance = CLEAR_CHASE;
    shotPose.avoid = true;
    shotPose.sweep = true;
    shotPose.treeCap = 0;
    shotPose.fillMin = clamp(o.fill !== undefined ? o.fill : 0, 0, 0.95);
    // Default composition: the hero goes to the side the camera did *not* swing to, and sits
    // a little above centre so the trail below it fills the bottom of frame.
    // Half the gameplay lead by default: a QA preset already aims down the trail from an
    // authored offset, so a good part of the off-centre placement is in the preset already
    // and the full gameplay bias on top of it lands the hero near the frame edge.
    shotPose.leadX = clamp(o.leadX !== undefined ? o.leadX
      : (azDeg >= 0 ? -LEAD_BASE * 0.5 : LEAD_BASE * 0.5), -LEAD_MAX, LEAD_MAX);
    shotPose.leadY = clamp(o.leadY !== undefined ? o.leadY : 0.14, -0.6, 0.6);

    // Boom, swung off the trail tangent by the requested azimuth.
    _boom.copy(_v0);
    rotateXZ(_boom, azDeg * DEG2RAD);
    shotPose.pos.set(_trPos.x, baseY + upM, _trPos.z)
      .addScaledVector(_boom, -back)
      .addScaledVector(_right, side);
    const gFloor = groundAt(shotPose.pos.x, shotPose.pos.z) + CLEAR_CHASE;
    if (shotPose.pos.y < gFloor) shotPose.pos.y = gFloor;

    // Aim down the trail; the framing solver then rotates this until the hero is placed.
    if (haveTrail && readTrail(t + aheadM / trailLen, _trPos2, _trTan2)) {
      shotPose.look.copy(_trPos2); shotPose.look.y += 1.2;
    } else {
      shotPose.look.copy(shotPose.anchor).addScaledVector(_v0, Math.max(4, aheadM));
    }

    const d0 = shotPose.pos.distanceTo(shotPose.anchor);
    const lift = heroSightNeed(shotPose);
    shotPose.pos.y += lift;
    shotResult.liftedOverCrest = lift;

    composeHero(shotPose);

    shotResult.pushedBack = shotPose.pos.distanceTo(shotPose.anchor) - d0;
    shotResult.position.copy(shotPose.pos);
    shotResult.lookAt.copy(shotPose.look);
    measureHeroScreen(shotPose);
    return shotResult;
  }

  /**
   * Solve the establishing shot (r3 review, G3).
   *
   * The r3 camera was placed at `start + (320, 180, 340)` aimed at the finish, which points
   * down and out to sea: a flat forested shelf against a featureless blue plane with the
   * world edge visible. The run itself is a smudge and nothing is in silhouette.
   *
   * What this does instead, and it is three decisions:
   *   1. Stand ACROSS the fall line, not on it, so the descent runs *through* the frame
   *      instead of away from the lens. The stand-off is proportional to the run's own
   *      length, so the whole run is in shot at any FOV.
   *   2. Pick the side of the run whose OPPOSITE side has the higher ground, because that
   *      is the side with a ridgeline behind the trail to silhouette against the sky.
   *   3. Tilt down by exactly enough to put the horizon on the upper third rather than the
   *      midline: the horizon of a level camera sits `tan(pitch)` above the view axis, so
   *      solving `tan(theta) = 0.40 * tan(fov/2)` places it at 20% of frame height above
   *      centre. That is one line of trigonometry and it is the whole difference between a
   *      landscape and a survey photograph.
   */
  function solveEstablishingShot(opts) {
    const o = opts || {};
    const fov = clamp(o.fov !== undefined ? o.fov : 46, 12, 120);
    shotResult.fov = fov;
    shotResult.pushedBack = 0;
    shotResult.liftedOverCrest = 0;
    shotResult.heroScreen.x = 0;
    shotResult.heroScreen.y = 0;
    shotResult.heroScreen.halfSize = 0;

    const ok = trailLen > 1
      && readTrail(0.012, _estA, _trTan)
      && readTrail(0.985, _estB, _trTan2)
      && readTrail(0.46, _estM, _trTan2);
    if (!ok) {
      // Nothing to establish. Hand back a sane high wide rather than a NaN.
      const s = bikeState();
      shotResult.position.copy(s.position).add(_v0.set(240, 180, 240));
      shotResult.lookAt.copy(s.position);
      return shotResult;
    }

    _estFall.set(_estB.x - _estA.x, 0, _estB.z - _estA.z);
    const span = _estFall.length();
    if (span < 1) _estFall.set(0, 0, -1); else _estFall.multiplyScalar(1 / span);
    _estSide.crossVectors(_estFall, _up).normalize();

    // Which side to stand on: the one whose far side is higher, so the run has a ridge
    // behind it rather than sky or ocean.
    const probe = clamp(span * 0.30, 120, 700);
    const hRight = groundAt(_estM.x + _estSide.x * probe, _estM.z + _estSide.z * probe);
    const hLeft = groundAt(_estM.x - _estSide.x * probe, _estM.z - _estSide.z * probe);
    let sideSign = hLeft > hRight ? 1 : -1;

    const D = clamp(span * 0.55, 380, 1100);
    const place = (sgn) => {
      shotResult.position.set(
        _estM.x + _estSide.x * sgn * D - _estFall.x * span * 0.10, 0,
        _estM.z + _estSide.z * sgn * D - _estFall.z * span * 0.10);
    };
    place(sideSign);
    // If that lands outside the world, the far side is the only side.
    const b = ctx.terrain && ctx.terrain.bounds;
    if (b) {
      const M = 80;
      const outside = shotResult.position.x < b.minX + M || shotResult.position.x > b.maxX - M
        || shotResult.position.z < b.minZ + M || shotResult.position.z > b.maxZ - M;
      if (outside) {
        place(-sideSign);
        sideSign = -sideSign;
        shotResult.position.x = clamp(shotResult.position.x, b.minX + M, b.maxX - M);
        shotResult.position.z = clamp(shotResult.position.z, b.minZ + M, b.maxZ - M);
      }
    }

    const camGround = groundAt(shotResult.position.x, shotResult.position.z);
    // High enough to see over the intervening ground, low enough that this is a landscape
    // and not a map. Referenced to the START, which is the top of the run.
    shotResult.position.y = Math.max(camGround, _estA.y) + clamp(D * 0.22, 90, 300);

    _estDir.set(_estM.x - shotResult.position.x, 0, _estM.z - shotResult.position.z);
    if (_estDir.lengthSq() < 1e-6) _estDir.copy(_estFall); else _estDir.normalize();
    const tanV = Math.tan(fov * DEG2RAD * 0.5);
    const theta = Math.atan(0.40 * tanV);
    const ct = Math.cos(theta), st = Math.sin(theta);
    _estDir.set(_estDir.x * ct, -st, _estDir.z * ct).normalize();
    shotResult.lookAt.copy(shotResult.position).addScaledVector(_estDir, D);
    return shotResult;
  }

  // =========================================================================
  return {
    get mode() { return mode; },
    get isPhotoMode() { return photo; },
    get blending() { return blendT < 1; },

    setMode,

    /** Cut instead of blend — used by anything that wants a deliberate hard cut. */
    cutTo(m) { setMode(m, { cut: true }); },

    setPhotoMode(on) { setPhoto(on, true); },

    /** Force the follow springs onto the current target (after a respawn, say). */
    snapToTarget() { requestSnap = true; },

    // Shot solvers (r3 review G1/G3). Pure — see the CONTRACT-NOTE at the head of the file.
    // The returned object is shared and reused; copy out of it before the next call.
    solveHeroShot,
    solveEstablishingShot,

    lateUpdate,

    dispose() {
      for (const off of offs) { try { off(); } catch (e) { /* ignore */ } }
      offs.length = 0;
      window.removeEventListener('keydown', onPhotoKey);
      window.removeEventListener('keyup', onPhotoKey);
      window.removeEventListener('blur', onPhotoBlur);
      window.removeEventListener('pointerdown', onPointerDown);
      window.removeEventListener('pointerup', onPointerUp);
      window.removeEventListener('pointermove', onPointerMove);
      window.removeEventListener('wheel', onWheel);
      document.removeEventListener('pointerlockchange', onPointerLockChange);
      if (photo) { ctx.photoMode = false; if (ctx.settings) ctx.settings.photoMode = false; }
    },
  };
}
