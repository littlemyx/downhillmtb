// rider.js — procedurally generated, fully rigged DH rider. CONTRACT §6, ADDENDUM §D.
//
// The rider is a single SkinnedMesh with 23 bones and four material groups
// (skin / kit / helmet shell / goggle lens) => 4 draw calls, inside the ≤ 8 budget.
// Everything — geometry, normal maps, colour — is generated in code. No assets.
//
// Design notes worth knowing before you edit anything here:
//
//  * ALL bones have an identity rest rotation. Bone rest positions are authored as
//    absolute points in rider-local space and the local offset is just
//    (restPos[child] − restPos[parent]). Because every rest rotation is identity, a
//    bone's *world* (rider-local) rotation IS its delta-from-rest, which makes the
//    IK, the look-at and the ragdoll retarget all trivially expressible as
//    "rotate the bone's rest child-direction onto this new direction". That single
//    decision is what keeps the animation code short and impossible to break.
//
//  * The rig is evaluated by our own forward-kinematics pass into preallocated
//    world position/quaternion arrays, then written down into the THREE.Bone
//    objects once per frame. We never call updateMatrixWorld() mid-frame, so
//    ordering bugs (pose the spine, then read the shoulder for arm IK) cannot happen.
//
//  * Rider-local space == the bike chassis frame: origin at the axle midpoint,
//    forward = −Z, up = +Y, rider's right = +X, ground at y = −0.37.
//    (bike.js publishes `geometry.chassisOrigin = 'axle-midpoint'`.)
//
// CONTRACT-NOTE: bikeModel.anchors (ADDENDUM §D) is read defensively — a missing
//   ctx.bikeModel, a missing anchor, or an anchor that reports a non-finite or
//   absurd position (> 3 m from the chassis origin, which is what a half-built
//   bikeModel looks like) silently falls back to hard-coded DH cockpit geometry so
//   the IK always has a valid target and never throws or snaps.
// CONTRACT-NOTE: additive API beyond `group`/`update` — `rider.mesh`, `rider.bones`,
//   `rider.skeleton`, `rider.getHeadPosition(out)` (first-person camera hook),
//   `rider.isRagdoll()`. Nothing else depends on them.
// CONTRACT-NOTE: the rider consumes several documented-additive fields on
//   ctx.bike.state (riderCrouch, riderFore, pumpDrive, whip, lean, gForce,
//   lastLanding, pitch) and degrades to sane defaults when they are absent.
//   `state.riderFore` is the ONE that carries a scale (metres of weight shift,
//   + = forward); `RIDER_SHIFT` below is its unit and the only number here that
//   mirrors a bike.js constant. The shift is never re-derived from input.pitch.
// CONTRACT-NOTE: the rider owns TWO UV channels on its own geometry.
//   `uv`  (channel 0) = the baked garment atlas — unique per surface patch.
//   `uv1` (channel 1) = the tiling weave/rubber normal UV; the kit/skin/helmet
//   materials therefore set `normalMap.channel = 1`. Nothing outside this file
//   touches that geometry, so the extra channel is self-contained.
// CONTRACT-NOTE: additive API `rider.setPhotoMode(bool)` — swaps the cheap
//   gameplay materials for MeshPhysicalMaterial variants (clearcoat on the
//   helmet, sheen on skin, iridescence on the lens). Off by default; the
//   gameplay set is 1 physical + 3 standard programs (round-3 work order C8).
//
// LANE C (round-3 work order) implementation notes:
//   C1  the helmet is FrontSide with a real inward-normalled liner + cheek pads,
//       so the eye port can no longer show the interior of the shell.
//   C2  every emitted patch registers a *chart*; charts are packed into a single
//       albedo atlas and painted at texel rate (hems, seams, stitch, sole tread,
//       logos, wear) instead of being interpolated across a 40-80 px triangle.
//   C6  the same atlas carries hemispherical AO baked against a capsule proxy of
//       the body + bar + pedals + saddle, so contact darkening exists at all.
//   C3  hands are a palm + four wrapped fingers + a thumb around the bar axis;
//       shoes are a flat sole slab + upper + strap; knee pads are a hard cap.
//   C7  the whole stack is ~10% shorter and the limb radii are well down.
//
// LANE R9 (round-9 work order) — make the weight shift VISIBLE, because the
// game now depends on the player performing it:
//   R9a `state.riderFore` owns the fore/aft travel. The old driver moved the
//       rider aft on GRADE with more authority (0.80) than the player's own
//       command had (0.70), so on a 45% grade two thirds of the shift happened
//       for free and the player-attributable hip travel collapsed to 74 mm —
//       the animation was performing the input it was meant to be teaching.
//       Grade now amplifies the command instead of substituting for it, and
//       player-owned hip travel runs 0.220 m (flat) to 0.332 m (45%).
//   R9b Getting back is hips back AND DOWN with the chest dropping and the
//       arms straightening: 0.31 m of hip travel, 0.16 m of hip drop and a
//       0.36 m head-height swing on a 45% grade, at 98% arm extension.
//   R9c The arms are now a real limit (`ARM_REACH`, closed-form clamp after
//       the spine FK). Before R9 the driver envelope exceeded the arm by up to
//       75 mm, which `solveLimb` absorbed by silently stretching the forearm.

import * as THREE from 'three';
import { makeRng, subSeed, clamp, clamp01, lerp, smoothstep, damp } from '../core/rng.js';

// ===========================================================================
// 0. Constants
// ===========================================================================

const GROUND_Y = -0.37;              // ground plane in rider-local space

// Material group indices (== order of the material array).
const G_SKIN = 0, G_KIT = 1, G_HELMET = 2, G_LENS = 3;

// Bone indices. Parents must always come first (topological order).
const B_PELVIS = 0, B_SPINE1 = 1, B_SPINE2 = 2, B_SPINE3 = 3,
      B_NECK = 4, B_HEAD = 5, B_PEAK = 6,
      B_CLAVL = 7, B_UARML = 8, B_LARML = 9, B_HANDL = 10,
      B_CLAVR = 11, B_UARMR = 12, B_LARMR = 13, B_HANDR = 14,
      B_THIGHL = 15, B_SHINL = 16, B_FOOTL = 17, B_TOEL = 18,
      B_THIGHR = 19, B_SHINR = 20, B_FOOTR = 21, B_TOER = 22;
const NBONES = 23;

// Segment lengths used by the IK and by the rest-pose solve.
// C7: the r3 measurement put helmet-crown-to-pedal at 1.536 m against a 0.74 m
// wheel, i.e. a 1.53 m rider. Everything below is scaled so the same measurement
// lands at ~1.385 m, and the arms are shortened enough that the elbow sits at
// ~110 deg instead of the 159 deg (near-locked) that read as a commuter perch.
const L_UPPERARM = 0.292, L_FOREARM = 0.256;
const L_THIGH = 0.412, L_SHIN = 0.404;

// Cockpit fallbacks (rider-local metres), derived from CONTRACT §0 scale:
// BB is 0.06 m behind the axle midpoint and 25 mm below the axle line; a DH
// cockpit puts the grips ~0.46 m ahead of the BB and 0.68 m above it, bar 760 wide.
const FB_BB = [0, -0.025, 0.060];
const FB_GRIP_L = [-0.380, 0.655, -0.400];
const FB_GRIP_R = [0.380, 0.655, -0.400];
const CRANK_R = 0.1725, Q_HALF = 0.085;
const FB_PEDAL_L = [-Q_HALF, FB_BB[1], FB_BB[2] + CRANK_R];   // left foot trailing
const FB_PEDAL_R = [Q_HALF, FB_BB[1], FB_BB[2] - CRANK_R];    // right foot leading

// C3 — grip geometry. The hand bone is the PALM centre, which sits HAND_UP above
// the bar axis and HAND_IN inboard of the grip's outer end; the fingers are then
// modelled as arcs of radius (BAR_R + finger radius) about the bar axis, so the
// bar is enclosed by geometry rather than passing through an ellipsoid.
const BAR_R = 0.0185;
const HAND_UP = 0.030;
const HAND_IN = 0.052;
// Ankle relative to the pedal spindle (a flat pedal platform is ~8 mm proud).
const ANKLE_UP = 0.088, ANKLE_BACK = 0.042;
// Shoe sole plane, measured down from the ankle joint in the foot's own frame.
const SOLE_DROP = 0.080, SOLE_THICK = 0.016;

// R9 — the fore/aft weight shift.
//
// `bike.js` publishes `state.riderFore` = `input.pitch * T.RIDER_SHIFT`, in
// metres, + = forward. This is the ONLY thing here that knows the scale; the
// weight shift itself is never re-derived. See the R9 block in update().
const RIDER_SHIFT = 0.30;
// A rider gets behind the saddle until they run out of arm, and so does this
// rig. `solveTwoBone` clamps the ELBOW to (l1+l2)*0.998 but `solveLimb` still
// snaps the hand onto the grip regardless of distance, so an over-commanded
// shift renders as a *stretched forearm* rather than as a rider at full
// extension — it fails silently and it looks wrong. Measured over the whole
// driver envelope, the code before R9 exceeded the arm by up to 75 mm.
// 0.975 leaves the clavicle shrug (applied after this clamp, ~4 mm at full
// crouch) and the lateral lean their margin; it costs 4 mm of rearward travel.
const ARM_REACH = (L_UPPERARM + L_FOREARM) * 0.975;

// ===========================================================================
// 1. Module-scope scratch — nothing in update() may allocate.
// ===========================================================================

const _v0 = new THREE.Vector3(), _v1 = new THREE.Vector3(), _v2 = new THREE.Vector3(),
      _v3 = new THREE.Vector3(), _v4 = new THREE.Vector3(), _v5 = new THREE.Vector3(),
      _v6 = new THREE.Vector3(), _v7 = new THREE.Vector3(), _v8 = new THREE.Vector3(),
      _v9 = new THREE.Vector3(), _v10 = new THREE.Vector3(), _v11 = new THREE.Vector3(),
      _s0 = new THREE.Vector3();
const _q0 = new THREE.Quaternion(), _q1 = new THREE.Quaternion(),
      _qa = new THREE.Quaternion(), _qb = new THREE.Quaternion(),
      _qc = new THREE.Quaternion(), _qd = new THREE.Quaternion();
const _e0 = new THREE.Euler();
const _m0 = new THREE.Matrix4(), _m1 = new THREE.Matrix4();
const _col = new THREE.Color();

// ===========================================================================
// 2. Procedural textures
// ===========================================================================

/** Seamless value noise on a `period × period` lattice, wrapped — tiles perfectly. */
function makeTileNoise(rng, period) {
  const g = new Float32Array(period * period);
  for (let i = 0; i < g.length; i++) g[i] = rng();
  return function noise(x, y) {
    const xi = Math.floor(x), yi = Math.floor(y);
    const fx = x - xi, fy = y - yi;
    const u = fx * fx * (3 - 2 * fx), v = fy * fy * (3 - 2 * fy);
    const x0 = ((xi % period) + period) % period, x1 = (x0 + 1) % period;
    const y0 = ((yi % period) + period) % period, y1 = (y0 + 1) % period;
    const a = g[y0 * period + x0], b = g[y0 * period + x1];
    const c = g[y1 * period + x0], d = g[y1 * period + x1];
    return lerp(lerp(a, b, u), lerp(c, d, u), v);
  };
}

/** Tileable fbm over [0,1)² using octave lattices that all divide evenly. */
function makeTileFbm(rng, octaves, basePeriod) {
  const layers = [];
  let p = basePeriod, amp = 1, norm = 0;
  for (let o = 0; o < octaves; o++) {
    layers.push({ n: makeTileNoise(rng, p), p, amp });
    norm += amp; p *= 2; amp *= 0.5;
  }
  return function fbm(u, v) {
    let s = 0;
    for (let i = 0; i < layers.length; i++) {
      const L = layers[i];
      s += L.n(u * L.p, v * L.p) * L.amp;
    }
    return s / norm;
  };
}

function heightToNormal(h, size, strength, out, off) {
  const idx = (x, y) => ((y + size) % size) * size + ((x + size) % size);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const dx = (h[idx(x + 1, y)] - h[idx(x - 1, y)]) * strength;
      const dy = (h[idx(x, y + 1)] - h[idx(x, y - 1)]) * strength;
      // Tangent-space normal from a height gradient; z is reconstructed in-shader
      // for the packed map, so only xy are stored.
      const inv = 1 / Math.sqrt(dx * dx + dy * dy + 1);
      const p = (y * size + x) * 4;
      out[p + off] = Math.max(0, Math.min(255, ((-dx * inv) * 0.5 + 0.5) * 255)) | 0;
      out[p + off + 1] = Math.max(0, Math.min(255, ((-dy * inv) * 0.5 + 0.5) * 255)) | 0;
    }
  }
}

/** Full RGB tangent normal (for maps consumed by the stock three normal path). */
function heightToNormalRGB(h, size, strength, out) {
  const idx = (x, y) => ((y + size) % size) * size + ((x + size) % size);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const dx = (h[idx(x + 1, y)] - h[idx(x - 1, y)]) * strength;
      const dy = (h[idx(x, y + 1)] - h[idx(x, y - 1)]) * strength;
      const inv = 1 / Math.sqrt(dx * dx + dy * dy + 1);
      const p = (y * size + x) * 4;
      out[p] = Math.max(0, Math.min(255, ((-dx * inv) * 0.5 + 0.5) * 255)) | 0;
      out[p + 1] = Math.max(0, Math.min(255, ((-dy * inv) * 0.5 + 0.5) * 255)) | 0;
      out[p + 2] = Math.max(0, Math.min(255, (inv * 0.5 + 0.5) * 255)) | 0;
      out[p + 3] = 255;
    }
  }
}

function dataTex(data, size, aniso) {
  const t = new THREE.DataTexture(data, size, size, THREE.RGBAFormat);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.NoColorSpace;          // data map, never sRGB
  t.magFilter = THREE.LinearFilter;
  t.minFilter = THREE.LinearMipmapLinearFilter;
  t.generateMipmaps = true;
  t.anisotropy = aniso;
  t.needsUpdate = true;
  return t;
}

/**
 * One 512² RGBA map carrying TWO tangent normals:
 *   RG = jersey/short fabric,  BA = moulded rubber (gloves, pads, shoes).
 * Packing them together means the whole kit needs a single sampler and the shader
 * can crossfade between the two by a per-vertex `aSurf` flag.
 *
 * C5 — the round-3 measurement is that this map was authored *below Nyquist*: at
 * the old TILE of 0.085 m a 9-px thread was 1.5 mm on the garment, i.e. ~0.5
 * screen pixels at 3 m, which is exactly the condition that produces moiré. The
 * fix is twofold: the tile is now 0.28 m and a thread is 16 px (8.8 mm ≈ 3 px at
 * 3 m, and the twill rib it forms is ~7 px), and — more importantly — the
 * *dominant* feature in the height field is no longer the weave at all but a
 * ridged fold field at 40–90 mm, which is the scale a viewer actually reads
 * cloth from. The weave survives only as a low-amplitude term that mips away.
 */
function makeKitNormal(rng, aniso) {
  const S = 512;
  const weave = new Float32Array(S * S);
  const rubber = new Float32Array(S * S);
  const foldN = makeTileFbm(rng, 3, 4);     // 1/4 tile => ~70 mm creases
  const gatherN = makeTileFbm(rng, 2, 9);   // ~31 mm gathers
  const fineN = makeTileFbm(rng, 3, 48);
  const grainN = makeTileFbm(rng, 4, 16);
  const cellN = makeTileNoise(rng, 36);     // ~7.8 mm silicone/rubber pebbles

  const THREADS = 16;                       // px per thread — 8.8 mm at a 0.28 m tile
  for (let y = 0; y < S; y++) {
    for (let x = 0; x < S; x++) {
      const i = y * S + x;
      const u = x / S, v = y / S;

      // Ridged fold field: |2f-1| inverted and sharpened turns blobby fbm into
      // creases (lines), which is what fabric actually does.
      const f = foldN(u, v);
      const crease = Math.pow(1 - Math.abs(f * 2 - 1), 2.6);
      const gather = Math.pow(1 - Math.abs(gatherN(u, v) * 2 - 1), 3.2);

      // Twill rib: a diagonal every two thread cells. This is the highest
      // frequency that is allowed to carry real amplitude.
      const rib = 0.5 + 0.5 * Math.cos(((x + y * 0.5) / (THREADS * 2)) * Math.PI * 2);
      const cx = Math.floor(x / THREADS), cy = Math.floor(y / THREADS);
      const fx = (x % THREADS) / THREADS, fy = (y % THREADS) / THREADS;
      const over = ((cx + cy) & 1) === 0;
      const thread = (over ? Math.sin(Math.PI * fy) : Math.sin(Math.PI * fx)) * 0.5 + 0.5;

      weave[i] = crease * 1.00 + gather * 0.42 + rib * 0.20
               + thread * 0.085 + fineN(u, v) * 0.05;

      // Rubber: pebbled cells + fine grain + a moulded rib every ~40 px (22 mm)
      // so glove palms and shoe soles read as moulded rather than merely noisy.
      const c = cellN(u * 36, v * 36);
      const mould = Math.pow(Math.abs(Math.sin((x + c * 10) * Math.PI / 40)), 5) * 0.55;
      rubber[i] = c * 0.62 + grainN(u, v) * 0.28 + mould;
    }
  }
  const data = new Uint8Array(S * S * 4);
  heightToNormal(weave, S, 20, data, 0);
  heightToNormal(rubber, S, 30, data, 2);
  return dataTex(data, S, aniso);
}

/** Skin: pores + a little sub-dermal unevenness. Only the jaw and neck show. */
function makeSkinNormal(rng, aniso) {
  const S = 256;
  const h = new Float32Array(S * S);
  const pores = makeTileFbm(rng, 4, 64);    // ~1 mm at the 0.28 m tile
  const broad = makeTileFbm(rng, 3, 8);
  for (let y = 0; y < S; y++) {
    for (let x = 0; x < S; x++) {
      h[y * S + x] = pores(x / S, y / S) * 0.55 + broad(x / S, y / S) * 0.45;
    }
  }
  const data = new Uint8Array(S * S * 4);
  heightToNormalRGB(h, S, 5, data);
  return dataTex(data, S, aniso);
}

/** Helmet: shallow orange-peel in the clear coat + fine flake. Very subtle. */
function makeHelmetNormal(rng, aniso) {
  const S = 256;
  const h = new Float32Array(S * S);
  const peel = makeTileFbm(rng, 3, 64);     // ~1.1 mm orange peel at a 0.28 m tile
  const flake = makeTileFbm(rng, 2, 128);
  for (let y = 0; y < S; y++) {
    for (let x = 0; x < S; x++) {
      h[y * S + x] = peel(x / S, y / S) * 0.8 + flake(x / S, y / S) * 0.2;
    }
  }
  const data = new Uint8Array(S * S * 4);
  heightToNormalRGB(h, S, 3, data);
  return dataTex(data, S, aniso);
}

// ===========================================================================
// 3. Palette (authored sRGB, converted to the linear working space by Color)
// ===========================================================================

function rgb(hex) {
  _col.setHex(hex);                     // sRGB → linear working space
  return [_col.r, _col.g, _col.b];
}

const PAL = {
  skin: rgb(0x9c6d52), skinDark: rgb(0x6d4936), skinShade: rgb(0x4a3020),
  lip: rgb(0x7d4a3c), brow: rgb(0x2d1e15),
  jersey: rgb(0x156a78), jerseyDark: rgb(0x0d3d47), jerseyAccent: rgb(0xd9541f),
  jerseyPale: rgb(0xa8c4c6), jerseyInk: rgb(0x08252c),
  shorts: rgb(0x22252b), shortsPanel: rgb(0x12444d), shortsInk: rgb(0x101216),
  sock: rgb(0x2b2f36), sockBand: rgb(0xd9541f),
  glove: rgb(0x15171b), gloveGrip: rgb(0x2c3037), gloveCuff: rgb(0x1e5c66),
  pad: rgb(0x1c1f24), padCap: rgb(0x3a4048), padSleeve: rgb(0x15181d),
  shoe: rgb(0x1e2126), shoeSole: rgb(0x5d4026), shoeTrim: rgb(0xd9541f),
  shoeToe: rgb(0x2c3036), lace: rgb(0x8e969c),
  helmet: rgb(0x2b2e34), helmetStripe: rgb(0xd9541f), helmetPale: rgb(0xc9cdd2),
  helmetIn: rgb(0x0e1013),
  liner: rgb(0x141619), linerPad: rgb(0x23262b),
  goggleFrame: rgb(0x121418), strap: rgb(0x16181c), strapBand: rgb(0xd9541f),
  stitch: rgb(0xb9bcc0), grime: rgb(0x4a3a26),
};

// sRGB <-> linear, needed because the atlas is written as 8-bit sRGB bytes but
// every colour function above works in the linear space THREE.Color produced.
function lin2srgb(x) {
  if (x <= 0) return 0;
  if (x >= 1) return 255;
  const s = x <= 0.0031308 ? x * 12.92 : 1.055 * Math.pow(x, 1 / 2.4) - 0.055;
  return Math.max(0, Math.min(255, (s * 255 + 0.5) | 0));
}

/** Cheap deterministic 2-D hash in [0,1) — used for grime, stipple and dust. */
function hash2(x, y) {
  let h = Math.sin(x * 127.1 + y * 311.7) * 43758.5453;
  return h - Math.floor(h);
}

/** A soft-edged band centred on `c` of half-width `w`, edge softness `s`. */
function band(x, c, w, s) {
  const d = Math.abs(x - c);
  return 1 - smoothstep(w, w + s, d);
}

/** A hard line pair — the signature of a stitched seam. */
function seamLine(x, c, w) {
  return band(x, c, w, 0.0016);
}

// ===========================================================================
// 4. Mesh builder
//
// Two primitives cover the whole body:
//   emitTube  — a swept ring loft (limbs, torso, helmet shell, chin bar, straps)
//   emitSheet — a parametric double-sided plate with a rim (peak, goggle lens)
// Everything lands in one interleaved vertex pool per material group; the groups
// are concatenated in order at the end so each becomes one contiguous draw call.
// All of this runs once at construction, so allocating freely here is fine.
// ===========================================================================

function makeBuilder(nGroups) {
  const G = [];
  const charts = [];
  for (let i = 0; i < nGroups; i++) {
    G.push({
      index: i, charts,
      pos: [], uv: [], col: [], tint: [], surf: [], flut: [], si: [], sw: [], idx: [],
    });
  }
  G.charts = charts;
  return G;
}

/** Push one vertex. bones/weights are up to two influences. */
function pushVert(g, x, y, z, u, v, c, surf, flut, b0, w0, b1, tint) {
  g.pos.push(x, y, z);
  g.uv.push(u, v);
  g.col.push(c[0], c[1], c[2]);
  g.tint.push(tint === undefined ? 1 : tint);
  g.surf.push(surf);
  g.flut.push(flut);
  const w1 = 1 - w0;
  g.si.push(b0, b1, 0, 0);
  g.sw.push(w0, w1, 0, 0);
  return (g.pos.length / 3) - 1;
}

/** Superellipse profile: pw = 2 is a plain ellipse, higher is boxier (ribcage). */
function shapeCos(t, pw) {
  const c = Math.cos(t);
  return c < 0 ? -Math.pow(-c, 2 / pw) : Math.pow(c, 2 / pw);
}
function shapeSin(t, pw) {
  const s = Math.sin(t);
  return s < 0 ? -Math.pow(-s, 2 / pw) : Math.pow(s, 2 / pw);
}

/**
 * Parallel-transport frames along a polyline. Returns per-point {ax, side, up}.
 * Parallel transport (rather than a fixed up-vector) is what stops limb tubes
 * from twisting when the chain bends through more than ~60°.
 */
function buildFrames(pts, upHint, closed) {
  const n = pts.length;
  const ax = [], side = [], up = [];
  const t = new THREE.Vector3(), a = new THREE.Vector3(), b = new THREE.Vector3();
  for (let i = 0; i < n; i++) {
    const p0 = pts[closed ? (i - 1 + n) % n : Math.max(0, i - 1)];
    const p1 = pts[closed ? (i + 1) % n : Math.min(n - 1, i + 1)];
    t.set(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    if (t.lengthSq() < 1e-12) t.set(0, 1, 0);
    t.normalize();
    ax.push(t.clone());
  }
  // Seed the first frame from the up hint, then transport.
  a.set(upHint[0], upHint[1], upHint[2]);
  if (Math.abs(a.dot(ax[0])) > 0.97) a.set(1, 0, 0);
  b.crossVectors(a, ax[0]).normalize();
  side.push(b.clone());
  up.push(new THREE.Vector3().crossVectors(ax[0], b).normalize());
  for (let i = 1; i < n; i++) {
    // Project the previous side vector into the new ring plane.
    b.copy(side[i - 1]);
    b.addScaledVector(ax[i], -b.dot(ax[i]));
    if (b.lengthSq() < 1e-10) { b.set(1, 0, 0).addScaledVector(ax[i], -ax[i].x); }
    b.normalize();
    side.push(b.clone());
    up.push(new THREE.Vector3().crossVectors(ax[i], b).normalize());
  }
  return { ax, side, up };
}

/**
 * Sweep a closed cross-section along a ring list.
 *
 * ring = { c:[x,y,z], side:Vector3, up:Vector3, rx, ry, pw, b0, b1, w0,
 *          col:[r,g,b]|fn(k,radial,ring), surf, flut, uRep, v, tint }
 * opts = { capStart, capEnd, closed, reverse, warp(ri,k,radial,ring,p3),
 *          mask(ri,k,x,y,z)->bool, kind, detail, chart:false }
 *
 * The ring is emitted with `radial + 1` columns — the last is a positional
 * duplicate of the first carrying u = 1 instead of u = 0. Without it the wrap
 * quad runs the whole UV range backwards across a single column of triangles,
 * which both smears the tiling normal map and makes an atlas chart impossible.
 */
function emitTube(g, rings, radial, opts) {
  opts = opts || {};
  const nR = rings.length;
  const base = g.pos.length / 3;
  const NC = radial + 1;
  const masked = opts.mask ? new Uint8Array(nR * NC) : null;
  const tmp = [0, 0, 0], wp = [0, 0, 0];

  for (let i = 0; i < nR; i++) {
    const r = rings[i];
    for (let k = 0; k < NC; k++) {
      const kk = k === radial ? 0 : k;
      const t = (kk / radial) * Math.PI * 2;
      const cs = shapeCos(t, r.pw), sn = shapeSin(t, r.pw);
      wp[0] = r.c[0] + r.side.x * (r.rx * cs) + r.up.x * (r.ry * sn);
      wp[1] = r.c[1] + r.side.y * (r.rx * cs) + r.up.y * (r.ry * sn);
      wp[2] = r.c[2] + r.side.z * (r.rx * cs) + r.up.z * (r.ry * sn);
      if (opts.warp) opts.warp(i, kk, radial, r, wp);
      let c = r.col;
      if (typeof c === 'function') { c = c(kk, radial, r, tmp) || tmp; }
      pushVert(g, wp[0], wp[1], wp[2], (k / radial) * r.uRep, r.v, c,
        r.surf, r.flut, r.b0, r.w0, r.b1, r.tint);
      if (masked) masked[i * NC + k] = opts.mask(i, kk, wp[0], wp[1], wp[2]) ? 1 : 0;
    }
  }

  const rev = !!opts.reverse;
  const nSeg = opts.closed ? nR : nR - 1;
  for (let i = 0; i < nSeg; i++) {
    const i1 = (i + 1) % nR;
    for (let k = 0; k < radial; k++) {
      const k1 = k + 1;
      if (masked && (masked[i * NC + k] || masked[i * NC + k1] ||
                     masked[i1 * NC + k] || masked[i1 * NC + k1])) continue;
      const a = base + i * NC + k, b = base + i * NC + k1;
      const c = base + i1 * NC + k, d = base + i1 * NC + k1;
      if (rev) g.idx.push(a, c, b, b, c, d);
      else g.idx.push(a, b, c, b, d, c);
    }
  }

  const extras = [];
  if (opts.capStart) {
    extras.push({ vi: capRing(g, rings[0], radial, base, !rev), u: 0.5, v: 0 });
  }
  if (opts.capEnd) {
    extras.push({ vi: capRing(g, rings[nR - 1], radial, base + (nR - 1) * NC, rev), u: 0.5, v: 1 });
  }
  if (opts.chart !== false) {
    g.charts.push({
      gi: g.index, base, nu: NC, nv: nR, radial, rings, extras, alias: -1,
      kind: opts.kind || 'kit', detail: opts.detail || null, sheet: false,
    });
  }
  return base;
}

function capRing(g, r, radial, ringBase, reversed) {
  let c = r.col;
  if (typeof c === 'function') c = c(0, radial, r, [0, 0, 0]) || [0, 0, 0];
  const ci = pushVert(g, r.c[0], r.c[1], r.c[2], 0.5, r.v, c,
    r.surf, r.flut, r.b0, r.w0, r.b1, r.tint);
  for (let k = 0; k < radial; k++) {
    const k1 = k + 1;
    if (reversed) g.idx.push(ci, ringBase + k1, ringBase + k);
    else g.idx.push(ci, ringBase + k, ringBase + k1);
  }
  return ci;
}

/**
 * Parametric plate with thickness. fn(i, j, out[3]) fills a grid point; the plate
 * is emitted as a front shell, a back shell and a rim so it is watertight and
 * never shows a backface.
 */
function emitSheet(g, nu, nv, fn, opts) {
  const thickFn = typeof opts.thickness === 'function'
    ? opts.thickness : () => opts.thickness;
  const wrapU = !!opts.wrapU;
  const P = new Float32Array(nu * nv * 3);
  const N = new Float32Array(nu * nv * 3);
  const t = [0, 0, 0];
  for (let j = 0; j < nv; j++) {
    for (let i = 0; i < nu; i++) {
      fn(i, j, t);
      const p = (j * nu + i) * 3;
      P[p] = t[0]; P[p + 1] = t[1]; P[p + 2] = t[2];
    }
  }
  const gp = (i, j, o) => {
    const ii = wrapU ? ((i % nu) + nu) % nu : clamp(i, 0, nu - 1);
    const jj = clamp(j, 0, nv - 1);
    const p = (jj * nu + ii) * 3;
    o.set(P[p], P[p + 1], P[p + 2]);
    return o;
  };
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(),
        d = new THREE.Vector3(), n = new THREE.Vector3();
  for (let j = 0; j < nv; j++) {
    for (let i = 0; i < nu; i++) {
      gp(i + 1, j, a); gp(i - 1, j, b); gp(i, j + 1, c); gp(i, j - 1, d);
      a.sub(b); c.sub(d);
      n.crossVectors(a, c);
      if (n.lengthSq() < 1e-14) n.set(0, 0, 1); else n.normalize();
      const p = (j * nu + i) * 3;
      N[p] = n.x; N[p + 1] = n.y; N[p + 2] = n.z;
    }
  }
  if (opts.flipNormals) for (let i = 0; i < N.length; i++) N[i] = -N[i];

  const base = g.pos.length / 3;
  const col = opts.col, tmp = [0, 0, 0];
  const tF = opts.tint === undefined ? 1 : opts.tint;
  const tB = opts.tintBack === undefined ? tF : opts.tintBack;
  for (let layer = 0; layer < 2; layer++) {
    for (let j = 0; j < nv; j++) {
      for (let i = 0; i < nu; i++) {
        const p = (j * nu + i) * 3;
        const uu = i / (nu - 1), vv = j / Math.max(1, nv - 1);
        const s = (layer === 0 ? 0.5 : -0.5) * thickFn(uu, vv);
        let cc = col;
        if (typeof cc === 'function') cc = cc(uu, vv, layer, tmp) || tmp;
        pushVert(g, P[p] + N[p] * s, P[p + 1] + N[p + 1] * s, P[p + 2] + N[p + 2] * s,
          uu * opts.uRep, vv * opts.vRep,
          cc, opts.surf, opts.flut, opts.b0, opts.w0, opts.b1,
          layer === 0 ? tF : tB);
      }
    }
  }
  const L0 = base, L1 = base + nu * nv;
  const iMax = wrapU ? nu : nu - 1;
  for (let j = 0; j < nv - 1; j++) {
    for (let i = 0; i < iMax; i++) {
      const i1 = (i + 1) % nu;
      const a0 = L0 + j * nu + i, b0 = L0 + j * nu + i1;
      const c0 = L0 + (j + 1) * nu + i, d0 = L0 + (j + 1) * nu + i1;
      g.idx.push(a0, c0, b0, b0, c0, d0);
      const a1 = L1 + j * nu + i, b1 = L1 + j * nu + i1;
      const c1 = L1 + (j + 1) * nu + i, d1 = L1 + (j + 1) * nu + i1;
      g.idx.push(a1, b1, c1, b1, d1, c1);
    }
  }
  // Rim: j = 0 and j = nv-1 edges (and the i edges when not wrapped).
  for (let i = 0; i < iMax; i++) {
    const i1 = (i + 1) % nu;
    const f0 = L0 + i, f1 = L0 + i1, k0 = L1 + i, k1 = L1 + i1;
    g.idx.push(f0, f1, k0, f1, k1, k0);
    const j = nv - 1;
    const g0 = L0 + j * nu + i, g1 = L0 + j * nu + i1, h0 = L1 + j * nu + i, h1 = L1 + j * nu + i1;
    g.idx.push(g0, h0, g1, g1, h0, h1);
  }
  if (!wrapU) {
    for (let j = 0; j < nv - 1; j++) {
      const a0 = L0 + j * nu, a1 = L0 + (j + 1) * nu, b0 = L1 + j * nu, b1 = L1 + (j + 1) * nu;
      g.idx.push(a0, b0, a1, a1, b0, b1);
      const e = nu - 1;
      const c0 = L0 + j * nu + e, c1 = L0 + (j + 1) * nu + e,
            d0 = L1 + j * nu + e, d1 = L1 + (j + 1) * nu + e;
      g.idx.push(c0, c1, d0, c1, d1, d0);
    }
  }
  if (opts.chart !== false) {
    g.charts.push({
      gi: g.index, base, nu, nv, radial: 0, rings: null, extras: [],
      alias: base + nu * nv, kind: opts.kind || 'kit',
      detail: opts.detail || null, sheet: true, col,
    });
  }
  return base;
}

/** Concatenate the material groups into one indexed BufferGeometry with groups. */
function finishGeometry(G) {
  let nv = 0, ni = 0;
  for (const g of G) { nv += g.pos.length / 3; ni += g.idx.length; }
  const pos = new Float32Array(nv * 3), uv = new Float32Array(nv * 2),
        uvA = new Float32Array(nv * 2),
        col = new Float32Array(nv * 3), nrm = new Float32Array(nv * 3),
        tint = new Float32Array(nv),
        surf = new Float32Array(nv), flut = new Float32Array(nv),
        si = new Uint16Array(nv * 4), sw = new Float32Array(nv * 4);
  const idx = nv > 65535 ? new Uint32Array(ni) : new Uint16Array(ni);
  const geo = new THREE.BufferGeometry();
  const offsets = new Int32Array(G.length);
  const ranges = [];

  let vo = 0, io = 0;
  for (let gi = 0; gi < G.length; gi++) {
    const g = G[gi];
    const count = g.pos.length / 3;
    offsets[gi] = vo;
    ranges.push({ start: vo, end: vo + count });
    pos.set(g.pos, vo * 3); uv.set(g.uv, vo * 2); col.set(g.col, vo * 3);
    tint.set(g.tint, vo);
    surf.set(g.surf, vo); flut.set(g.flut, vo);
    si.set(g.si, vo * 4); sw.set(g.sw, vo * 4);
    for (let i = 0; i < g.idx.length; i++) idx[io + i] = g.idx[i] + vo;
    geo.addGroup(io, g.idx.length, gi);
    vo += count; io += g.idx.length;
  }

  // Area-weighted vertex normals (accumulate the un-normalised face normals).
  for (let i = 0; i < ni; i += 3) {
    const a = idx[i] * 3, b = idx[i + 1] * 3, c = idx[i + 2] * 3;
    const ax = pos[b] - pos[a], ay = pos[b + 1] - pos[a + 1], az = pos[b + 2] - pos[a + 2];
    const bx = pos[c] - pos[a], by = pos[c + 1] - pos[a + 1], bz = pos[c + 2] - pos[a + 2];
    const nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    nrm[a] += nx; nrm[a + 1] += ny; nrm[a + 2] += nz;
    nrm[b] += nx; nrm[b + 1] += ny; nrm[b + 2] += nz;
    nrm[c] += nx; nrm[c + 1] += ny; nrm[c + 2] += nz;
  }
  for (let i = 0; i < nv; i++) {
    const p = i * 3;
    const l = Math.hypot(nrm[p], nrm[p + 1], nrm[p + 2]);
    if (l > 1e-9) { nrm[p] /= l; nrm[p + 1] /= l; nrm[p + 2] /= l; }
    else { nrm[p] = 0; nrm[p + 1] = 1; nrm[p + 2] = 0; }   // degenerate fan centre
  }

  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('normal', new THREE.BufferAttribute(nrm, 3));
  // uv  (channel 0) = the baked garment atlas, filled in by bakeRiderAtlas().
  // uv1 (channel 1) = the tiling weave/rubber normal-map UV built above.
  geo.setAttribute('uv', new THREE.BufferAttribute(uvA, 2));
  geo.setAttribute('uv1', new THREE.BufferAttribute(uv, 2));
  geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  geo.setAttribute('aSurf', new THREE.BufferAttribute(surf, 1));
  geo.setAttribute('aFlutter', new THREE.BufferAttribute(flut, 1));
  geo.setAttribute('skinIndex', new THREE.BufferAttribute(si, 4));
  geo.setAttribute('skinWeight', new THREE.BufferAttribute(sw, 4));
  geo.setIndex(new THREE.BufferAttribute(idx, 1));
  // Generous manual bounds: skinning + ragdoll move vertices well outside the
  // rest-pose box and a stale bounding sphere would pop the shadow caster out.
  geo.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0.70, 0.06), 1.75);
  return { geo, offsets, ranges, tint, count: nv, charts: G.charts };
}

// ===========================================================================
// 4b. Baked ambient occlusion  (work order C6, and the asset-side half of R10)
//
// R10 is explicit that grounding must NOT be bought by re-raising N8AO — that
// re-opens round 2's shadow-floor P0. So the rider carries its own occlusion,
// baked once at construction against a capsule proxy of the body *and* of the
// three things it is in contact with: the bar, the pedals and the saddle. That
// is the contact darkening the review says is missing everywhere in 16 frames.
// ===========================================================================

/** 18 cosine-ish directions on the +Z hemisphere (a Fibonacci spiral). */
const AO_DIRS = (function buildAoDirs() {
  const N = 18, out = new Float32Array(N * 3);
  const ga = Math.PI * (3 - Math.sqrt(5));
  for (let i = 0; i < N; i++) {
    // z in (0,1]: bias the set toward the pole so the horizon ring is not
    // over-weighted; the cosine term is applied again at accumulation time.
    const z = Math.sqrt((i + 0.5) / N);
    const r = Math.sqrt(Math.max(0, 1 - z * z));
    const a = i * ga;
    out[i * 3] = Math.cos(a) * r; out[i * 3 + 1] = Math.sin(a) * r; out[i * 3 + 2] = z;
  }
  return out;
})();

/** Closest-approach parameter of a ray against a capsule; -1 if it misses. */
function rayCapsule(ox, oy, oz, dx, dy, dz, c, maxT) {
  const ux = c.bx - c.ax, uy = c.by - c.ay, uz = c.bz - c.az;
  const wx = ox - c.ax, wy = oy - c.ay, wz = oz - c.az;
  const b = dx * ux + dy * uy + dz * uz;
  const cc = ux * ux + uy * uy + uz * uz;
  const d = dx * wx + dy * wy + dz * wz;
  const e = ux * wx + uy * wy + uz * wz;
  const D = cc - b * b;                        // |d| == 1
  let s;
  if (D < 1e-9 || cc < 1e-12) s = 0;
  else s = (e - b * d) / D;
  s = s < 0 ? 0 : (s > 1 ? 1 : s);
  // Re-solve t for the clamped s (one Gauss-Seidel step is plenty for AO).
  let t = dx * (c.ax + ux * s - ox) + dy * (c.ay + uy * s - oy) + dz * (c.az + uz * s - oz);
  if (t < 0) t = 0; else if (t > maxT) t = maxT;
  const px = ox + dx * t - (c.ax + ux * s);
  const py = oy + dy * t - (c.ay + uy * s);
  const pz = oz + dz * t - (c.az + uz * s);
  return (px * px + py * py + pz * pz) <= c.r * c.r ? t : -1;
}

/** Squared distance from a point to a capsule axis. */
function pointCapsuleD2(px, py, pz, c) {
  const ux = c.bx - c.ax, uy = c.by - c.ay, uz = c.bz - c.az;
  const cc = ux * ux + uy * uy + uz * uz;
  let s = cc < 1e-12 ? 0 : ((px - c.ax) * ux + (py - c.ay) * uy + (pz - c.az) * uz) / cc;
  s = s < 0 ? 0 : (s > 1 ? 1 : s);
  const dx = px - (c.ax + ux * s), dy = py - (c.ay + uy * s), dz = pz - (c.az + uz * s);
  return dx * dx + dy * dy + dz * dz;
}

/**
 * Capsule proxy of the rider PLUS the three things it is in contact with — the
 * bar, the pedal platforms and the saddle nose. Contact darkening is what tells
 * a viewer an object has weight, and the review found none anywhere in 16 frames.
 */
function bodyCapsules(rig) {
  const P = rig.restPos;
  const C = [];
  const push = (a, b, r) => C.push({
    ax: a[0], ay: a[1], az: a[2], bx: b[0], by: b[1], bz: b[2], r,
  });
  const p = (i) => [P[i].x, P[i].y, P[i].z];

  push(p(B_PELVIS), p(B_SPINE2), 0.148);
  push(p(B_SPINE2), p(B_SPINE3), 0.158);
  push(p(B_SPINE3), p(B_NECK), 0.108);
  push(p(B_THIGHL), p(B_THIGHR), 0.128);                     // hip block
  push(p(B_NECK), p(B_HEAD), 0.060);
  // Helmet: a ball at the shell centre. Head/liner/shell vertices sit inside it
  // and are skipped by the "inside" test; what it darkens is the collar, the
  // shoulders and the tops of the goggle strap, which is correct.
  const sc = [P[B_HEAD].x, P[B_HEAD].y + 0.086, P[B_HEAD].z - 0.008];
  push(sc, sc, 0.138);

  push(p(B_UARML), p(B_LARML), 0.076); push(p(B_LARML), p(B_HANDL), 0.050);
  push(p(B_UARMR), p(B_LARMR), 0.076); push(p(B_LARMR), p(B_HANDR), 0.050);
  push(p(B_THIGHL), p(B_SHINL), 0.106); push(p(B_SHINL), p(B_FOOTL), 0.070);
  push(p(B_FOOTL), p(B_TOEL), 0.048);
  push(p(B_THIGHR), p(B_SHINR), 0.106); push(p(B_SHINR), p(B_FOOTR), 0.070);
  push(p(B_FOOTR), p(B_TOER), 0.048);

  // Contact partners.
  push(FB_GRIP_L, FB_GRIP_R, BAR_R + 0.005);
  for (const pd of [FB_PEDAL_L, FB_PEDAL_R]) {
    push([pd[0] - 0.052, pd[1], pd[2]], [pd[0] + 0.052, pd[1], pd[2]], 0.042);
  }
  push([0, 0.638, 0.250], [0, 0.638, 0.372], 0.072);         // saddle nose/rails
  return C;
}

/**
 * Per-vertex AO in [0,1]. `skip` marks vertices (the goggle lens) that must stay
 * unoccluded — a mirror lens with AO on it reads as a dirty lens, not a mirror.
 */
function bakeVertexAO(pos, nrm, count, caps, skipStart, skipEnd) {
  const ao = new Float32Array(count);
  const MAXT = 0.42;                 // contact radius; beyond this it is sky
  const near = [];
  const NDIR = AO_DIRS.length / 3;
  for (let v = 0; v < count; v++) {
    if (v >= skipStart && v < skipEnd) { ao[v] = 1; continue; }
    const p = v * 3;
    const nx = nrm[p], ny = nrm[p + 1], nz = nrm[p + 2];
    const ox = pos[p] + nx * 0.006, oy = pos[p + 1] + ny * 0.006, oz = pos[p + 2] + nz * 0.006;

    // Prune: only capsules whose swept volume can reach this point matter, and
    // any capsule we are already inside is our own body part.
    near.length = 0;
    for (let ci = 0; ci < caps.length; ci++) {
      const c = caps[ci];
      const d2 = pointCapsuleD2(ox, oy, oz, c);
      const reach = c.r + MAXT;
      if (d2 > reach * reach) continue;
      if (d2 < c.r * c.r * 1.0201) continue;         // we are inside it
      near.push(c);
    }
    if (near.length === 0) { ao[v] = 1; continue; }

    // Tangent frame about the normal.
    let tx, ty, tz;
    if (Math.abs(ny) < 0.9) { tx = -nz; ty = 0; tz = nx; } else { tx = 0; ty = -nz; tz = ny; }
    let tl = Math.hypot(tx, ty, tz); if (tl < 1e-6) { tx = 1; ty = 0; tz = 0; tl = 1; }
    tx /= tl; ty /= tl; tz /= tl;
    const bx = ny * tz - nz * ty, by = nz * tx - nx * tz, bz = nx * ty - ny * tx;

    let occ = 0, wsum = 0;
    for (let i = 0; i < NDIR; i++) {
      const ax = AO_DIRS[i * 3], ay = AO_DIRS[i * 3 + 1], az = AO_DIRS[i * 3 + 2];
      const dx = tx * ax + bx * ay + nx * az;
      const dy = ty * ax + by * ay + ny * az;
      const dz = tz * ax + bz * ay + nz * az;
      const w = az;                                  // cosine weight
      wsum += w;
      let best = -1;
      for (let ci = 0; ci < near.length; ci++) {
        const t = rayCapsule(ox, oy, oz, dx, dy, dz, near[ci], MAXT);
        if (t >= 0.004 && (best < 0 || t < best)) best = t;
      }
      if (best >= 0) occ += w * (1 - smoothstep(0.05, MAXT, best));
    }
    ao[v] = clamp01(1 - (occ / Math.max(1e-4, wsum)) * 0.92);
  }
  return ao;
}

// ===========================================================================
// 4c. Garment atlas  (work order C2)
//
// Every emitted patch is a regular (nu × nv) grid, so it is already trivially
// parameterised — no unwrap is needed, only a packer. Each chart gets a rect in
// one shared texture and is painted per texel: the broad colour comes from the
// same authored colour function the vertices used to carry, and on top of it go
// the things a 2 mm feature on a swept loft can never express — hems, cover
// stitch, panel seams, cuffs, sole tread, laces, print, wear and baked AO.
// ===========================================================================

const ATLAS_PAD = 2;

function chartMetrics(chart, pos, off) {
  const { nu, nv } = chart;
  const gb = off + chart.base;
  let uLen = 0, vLen = 0;
  const rows = Math.min(nv, 5), cols = Math.min(nu, 5);
  for (let jj = 0; jj < rows; jj++) {
    const j = Math.round((jj / Math.max(1, rows - 1)) * (nv - 1));
    let s = 0;
    for (let i = 0; i < nu - 1; i++) {
      const a = (gb + j * nu + i) * 3, b = (gb + j * nu + i + 1) * 3;
      s += Math.hypot(pos[b] - pos[a], pos[b + 1] - pos[a + 1], pos[b + 2] - pos[a + 2]);
    }
    uLen += s;
  }
  for (let ii = 0; ii < cols; ii++) {
    const i = Math.round((ii / Math.max(1, cols - 1)) * (nu - 1));
    let s = 0;
    for (let j = 0; j < nv - 1; j++) {
      const a = (gb + j * nu + i) * 3, b = (gb + (j + 1) * nu + i) * 3;
      s += Math.hypot(pos[b] - pos[a], pos[b + 1] - pos[a + 1], pos[b + 2] - pos[a + 2]);
    }
    vLen += s;
  }
  chart.mU = uLen / rows;
  chart.mV = vLen / cols;
}

/** Shelf packer. Returns true and writes chart.rect when everything fits. */
function packCharts(charts, size, texelM) {
  const cap = Math.floor(size * 0.40);
  for (const c of charts) {
    c.wPx = Math.max(6, Math.min(cap, Math.ceil(c.mU / texelM)));
    c.hPx = Math.max(6, Math.min(cap, Math.ceil(c.mV / texelM)));
  }
  const order = charts.slice().sort((a, b) => b.hPx - a.hPx);
  let x = 0, y = 0, shelf = 0;
  for (const c of order) {
    const w = c.wPx + ATLAS_PAD * 2, h = c.hPx + ATLAS_PAD * 2;
    if (w > size || h > size) return false;
    if (x + w > size) { x = 0; y += shelf; shelf = 0; }
    if (y + h > size) return false;
    c.rx = x + ATLAS_PAD; c.ry = y + ATLAS_PAD;
    x += w; if (h > shelf) shelf = h;
  }
  return true;
}

const _paint = [0, 0, 0, 1, 0];
const _ringScratch = { U: 0 };

function sampleChartColour(chart, u, v, out) {
  if (chart.sheet) {
    out[4] = v;
    const c = chart.col;
    if (typeof c === 'function') { const r = c(u, v, 0, out) || out; if (r !== out) { out[0] = r[0]; out[1] = r[1]; out[2] = r[2]; } }
    else if (c) { out[0] = c[0]; out[1] = c[1]; out[2] = c[2]; }
    else { out[0] = out[1] = out[2] = 1; }
    return;
  }
  const rings = chart.rings, nR = chart.nv;
  const fi = v * (nR - 1);
  let i0 = Math.floor(fi); if (i0 < 0) i0 = 0; if (i0 > nR - 1) i0 = nR - 1;
  const i1 = Math.min(nR - 1, i0 + 1), tt = fi - i0;
  const u0 = rings[i0].U !== undefined ? rings[i0].U : i0 / Math.max(1, nR - 1);
  const u1 = rings[i1].U !== undefined ? rings[i1].U : i1 / Math.max(1, nR - 1);
  _ringScratch.U = u0 + (u1 - u0) * tt;
  out[4] = _ringScratch.U;
  const c = rings[i0].col;
  if (typeof c === 'function') {
    const r = c(u * chart.radial, chart.radial, _ringScratch, out) || out;
    if (r !== out) { out[0] = r[0]; out[1] = r[1]; out[2] = r[2]; }
  } else if (c) { out[0] = c[0]; out[1] = c[1]; out[2] = c[2]; }
  else { out[0] = out[1] = out[2] = 1; }
}

/**
 * Pack, paint and upload. Writes geo.attributes.uv (channel 0) in place and
 * returns the DataTexture.
 */
function bakeRiderAtlas(charts, geo, offsets, ao, tintArr, size, texelMin, aniso) {
  const pos = geo.attributes.position.array;
  const uvA = geo.attributes.uv.array;
  const colA = geo.attributes.color.array;
  // A charted vertex takes its albedo from the atlas, so its vertex colour drops
  // to a scalar tint. Anything NOT charted (the goggle lens) keeps the authored
  // colour it was built with, so a missing island can never turn the rider white.
  const setTint = (vi) => {
    const t = tintArr[vi];
    colA[vi * 3] = t; colA[vi * 3 + 1] = t; colA[vi * 3 + 2] = t;
  };
  for (const c of charts) chartMetrics(c, pos, offsets[c.gi]);

  // Largest texel that still fits everything, but never finer than texelMin —
  // there is no point paying for detail the screen cannot resolve.
  let lo = texelMin, hi = 0.05, best = hi;
  for (let it = 0; it < 16; it++) {
    const mid = (lo + hi) * 0.5;
    if (packCharts(charts, size, mid)) { best = mid; hi = mid; } else { lo = mid; }
    if (hi - lo < 1e-5) break;
  }
  if (!packCharts(charts, size, best)) packCharts(charts, size, best * 1.6);

  const data = new Uint8Array(size * size * 4);
  data.fill(255);
  const inv = 1 / size;

  for (const chart of charts) {
    const { nu, nv, wPx, hPx, rx, ry } = chart;
    const gb = offsets[chart.gi] + chart.base;
    const ab = chart.alias >= 0 ? offsets[chart.gi] + chart.alias : -1;

    // --- UVs: vertex (i,j) lands exactly on a texel centre inside the island.
    for (let j = 0; j < nv; j++) {
      for (let i = 0; i < nu; i++) {
        const px = rx + (nu > 1 ? (i * (wPx - 1)) / (nu - 1) : 0) + 0.5;
        const py = ry + (nv > 1 ? (j * (hPx - 1)) / (nv - 1) : 0) + 0.5;
        const vg = gb + j * nu + i;
        uvA[vg * 2] = px * inv; uvA[vg * 2 + 1] = py * inv;
        setTint(vg);
        if (ab >= 0) {
          const vj = ab + j * nu + i;
          uvA[vj * 2] = px * inv; uvA[vj * 2 + 1] = py * inv;
          setTint(vj);
        }
      }
    }
    for (const e of chart.extras) {
      // Cap-fan centres are not on the grid; park them on the island edge the
      // fan belongs to so the cap samples the same colour as its rim.
      const px = rx + e.u * (wPx - 1) + 0.5, py = ry + e.v * (hPx - 1) + 0.5;
      const vi = offsets[chart.gi] + e.vi;
      uvA[vi * 2] = px * inv; uvA[vi * 2 + 1] = py * inv;
      setTint(vi);
    }

    // --- AO grid for this chart (bilinear source).
    const aog = new Float32Array(nu * nv);
    for (let j = 0; j < nv; j++) {
      for (let i = 0; i < nu; i++) {
        let a = ao[gb + j * nu + i];
        if (ab >= 0) a = Math.min(a, ao[ab + j * nu + i]);
        aog[j * nu + i] = a;
      }
    }

    // --- Paint.
    const detail = chart.detail;
    for (let py = -ATLAS_PAD; py < hPx + ATLAS_PAD; py++) {
      const v = clamp01(py / Math.max(1, hPx - 1));
      const fj = v * (nv - 1);
      const j0 = Math.min(nv - 1, Math.floor(fj)), j1 = Math.min(nv - 1, j0 + 1);
      const tj = fj - j0;
      const row = ((ry + Math.max(0, Math.min(size - 1, py))) * size) * 4;
      for (let px = -ATLAS_PAD; px < wPx + ATLAS_PAD; px++) {
        const u = clamp01(px / Math.max(1, wPx - 1));
        _paint[0] = 1; _paint[1] = 1; _paint[2] = 1; _paint[3] = 1; _paint[4] = v;
        sampleChartColour(chart, u, v, _paint);
        if (detail) detail(u, v, _paint);

        const fi2 = u * (nu - 1);
        const i0 = Math.min(nu - 1, Math.floor(fi2)), i1 = Math.min(nu - 1, i0 + 1);
        const ti = fi2 - i0;
        const a0 = aog[j0 * nu + i0] * (1 - ti) + aog[j0 * nu + i1] * ti;
        const a1 = aog[j1 * nu + i0] * (1 - ti) + aog[j1 * nu + i1] * ti;
        const aoV = (a0 * (1 - tj) + a1 * tj) * _paint[3];

        const o = row + (rx + Math.max(0, Math.min(size - 1, px))) * 4;
        data[o] = lin2srgb(_paint[0] * aoV);
        data[o + 1] = lin2srgb(_paint[1] * aoV);
        data[o + 2] = lin2srgb(_paint[2] * aoV);
        data[o + 3] = 255;
      }
    }
  }

  geo.attributes.uv.needsUpdate = true;
  geo.attributes.color.needsUpdate = true;
  const t = new THREE.DataTexture(data, size, size, THREE.RGBAFormat);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping;
  t.magFilter = THREE.LinearFilter;
  t.minFilter = THREE.LinearMipmapLinearFilter;
  t.generateMipmaps = true;
  t.anisotropy = aniso;
  t.needsUpdate = true;
  return t;
}

// ===========================================================================
// 5. Rig
// ===========================================================================

/**
 * Analytic two-bone IK. Writes the mid-joint into `outMid` and the (perpendicular,
 * unit) bend direction into `outBend`. `pole` is a world-ish point the joint is
 * pulled towards — that is what keeps elbows out and knees forward instead of
 * flipping about randomly, which is the single most common way rider IK looks bad.
 */
function solveTwoBone(root, l1, l2, target, pole, outMid, outBend) {
  // Dedicated scratch: callers routinely pass _v0/_v1 as target/pole.
  const axis = _s0.copy(target).sub(root);
  let d = axis.length();
  const dMin = Math.abs(l1 - l2) + 1e-3;
  const dMax = (l1 + l2) * 0.998;         // never fully lock the joint
  if (d < 1e-5) { axis.set(0, -1, 0); d = dMin; }
  else axis.multiplyScalar(1 / d);
  d = clamp(d, dMin, dMax);

  const bend = outBend.copy(pole).sub(root);
  bend.addScaledVector(axis, -bend.dot(axis));
  if (bend.lengthSq() < 1e-8) {
    // Degenerate pole (pole on the bone axis) — pick any stable perpendicular.
    bend.set(0, 1, 0).addScaledVector(axis, -axis.y);
    if (bend.lengthSq() < 1e-8) bend.set(1, 0, 0).addScaledVector(axis, -axis.x);
  }
  bend.normalize();

  const cosA = clamp((l1 * l1 + d * d - l2 * l2) / (2 * l1 * d), -1, 1);
  const sinA = Math.sqrt(Math.max(0, 1 - cosA * cosA));
  outMid.copy(root).addScaledVector(axis, l1 * cosA).addScaledVector(bend, l1 * sinA);
  return d;
}

/** Bone table. Positions are absolute rider-local; rest rotations are all identity. */
function buildRig() {
  const P = new Array(NBONES);
  const parent = new Int8Array(NBONES);
  const name = new Array(NBONES);

  const set = (i, n, p, x, y, z) => { name[i] = n; parent[i] = p; P[i] = new THREE.Vector3(x, y, z); };

  // C4/C7 — DH attack position at ~1.385 m crown-to-pedal.
  //  * hips low and back over the rear of the cockpit, pelvis carried in
  //    posterior tilt (spine1 sits forward of the pelvis, not stacked above it),
  //  * thoracic spine flattened: the pelvis->spine3 run is a straight 33 deg
  //    ramp rather than the old 38 deg hunch,
  //  * shoulders lowered 115 mm and brought forward, which together with the
  //    shorter humerus/forearm puts the elbow at 110 deg instead of 159 deg.
  //  * eyeline (goggle centre, y = 1.245) is 0.59 m above the bar.
  //    idx        name          parent      x       y       z
  set(B_PELVIS, 'pelvis', -1, 0.000, 0.756, 0.336);
  set(B_SPINE1, 'spine1', B_PELVIS, 0.000, 0.826, 0.232);
  set(B_SPINE2, 'spine2', B_SPINE1, 0.000, 0.917, 0.080);
  set(B_SPINE3, 'spine3', B_SPINE2, 0.000, 1.024, -0.080);
  set(B_NECK, 'neck', B_SPINE3, 0.000, 1.083, -0.128);
  set(B_HEAD, 'head', B_NECK, 0.000, 1.147, -0.164);
  set(B_PEAK, 'peak', B_HEAD, 0.000, 1.279, -0.289);   // visor mount, on the brow

  set(B_CLAVL, 'clavicleL', B_SPINE3, -0.054, 1.050, -0.092);
  set(B_UARML, 'upperArmL', B_CLAVL, -0.170, 1.020, -0.140);
  set(B_LARML, 'lowerArmL', B_UARML, 0, 0, 0);          // solved below
  set(B_HANDL, 'handL', B_LARML, -0.328, 0.685, -0.400);

  set(B_CLAVR, 'clavicleR', B_SPINE3, 0.054, 1.050, -0.092);
  set(B_UARMR, 'upperArmR', B_CLAVR, 0.170, 1.020, -0.140);
  set(B_LARMR, 'lowerArmR', B_UARMR, 0, 0, 0);
  set(B_HANDR, 'handR', B_LARMR, 0.328, 0.685, -0.400);

  set(B_THIGHL, 'thighL', B_PELVIS, -0.092, 0.736, 0.320);
  set(B_SHINL, 'shinL', B_THIGHL, 0, 0, 0);
  set(B_FOOTL, 'footL', B_SHINL, -Q_HALF, FB_PEDAL_L[1] + ANKLE_UP, FB_PEDAL_L[2] + ANKLE_BACK);
  // Toe forward and very slightly UP: the rest foot must agree with the runtime
  // foot direction (footPitch ~0.05-0.15 rad) or the flat sole tilts off the pedal.
  set(B_TOEL, 'toeL', B_FOOTL, -Q_HALF, FB_PEDAL_L[1] + ANKLE_UP + 0.016,
    FB_PEDAL_L[2] + ANKLE_BACK - 0.160);

  set(B_THIGHR, 'thighR', B_PELVIS, 0.092, 0.736, 0.320);
  set(B_SHINR, 'shinR', B_THIGHR, 0, 0, 0);
  set(B_FOOTR, 'footR', B_SHINR, Q_HALF, FB_PEDAL_R[1] + ANKLE_UP, FB_PEDAL_R[2] + ANKLE_BACK);
  set(B_TOER, 'toeR', B_FOOTR, Q_HALF, FB_PEDAL_R[1] + ANKLE_UP + 0.016,
    FB_PEDAL_R[2] + ANKLE_BACK - 0.160);

  // Solve the rest elbows/knees with the same IK the runtime uses, so the rest
  // pose and the animated pose agree exactly and skinning never has to stretch.
  const mid = new THREE.Vector3(), bend = new THREE.Vector3(), pole = new THREE.Vector3();
  const restBend = new Array(NBONES);

  for (const s of [-1, 1]) {
    const iU = s < 0 ? B_UARML : B_UARMR, iL = s < 0 ? B_LARML : B_LARMR,
          iH = s < 0 ? B_HANDL : B_HANDR;
    // C4 — elbows WIDE and up. The pole is dominated by the lateral term so the
    // upper arm sits outboard of the torso silhouette instead of pinned to it.
    pole.copy(P[iU]).add(_v1.set(s * 0.95, 0.46, 0.44).normalize().multiplyScalar(0.72));
    solveTwoBone(P[iU], L_UPPERARM, L_FOREARM, P[iH], pole, mid, bend);
    P[iL].copy(mid);
    restBend[iU] = bend.clone(); restBend[iL] = bend.clone();

    const iT = s < 0 ? B_THIGHL : B_THIGHR, iS = s < 0 ? B_SHINL : B_SHINR,
          iF = s < 0 ? B_FOOTL : B_FOOTR;
    // Knees track forward and out, following the widened elbows.
    pole.copy(P[iT]).add(_v1.set(s * 0.34, 0.08, -0.92).normalize().multiplyScalar(0.72));
    solveTwoBone(P[iT], L_THIGH, L_SHIN, P[iF], pole, mid, bend);
    P[iS].copy(mid);
    restBend[iT] = bend.clone(); restBend[iS] = bend.clone();
  }

  // Leaf bones need an explicit tip so they still have a rest direction.
  const tip = new Array(NBONES);
  tip[B_HANDL] = P[B_HANDL].clone().add(new THREE.Vector3(0.100, -0.010, 0.006));
  tip[B_HANDR] = P[B_HANDR].clone().add(new THREE.Vector3(-0.100, -0.010, 0.006));
  tip[B_TOEL] = P[B_TOEL].clone().add(new THREE.Vector3(0, 0.004, -0.055));
  tip[B_TOER] = P[B_TOER].clone().add(new THREE.Vector3(0, 0.004, -0.055));
  tip[B_PEAK] = P[B_PEAK].clone().add(new THREE.Vector3(0, 0.022, -0.086));

  // Primary child per bone (defines restDir). Branch points pick the spine.
  const child = new Int8Array(NBONES).fill(-1);
  for (let i = 0; i < NBONES; i++) {
    const p = parent[i];
    if (p >= 0 && child[p] < 0) child[p] = i;
  }
  child[B_SPINE3] = B_NECK;               // not the left clavicle
  child[B_PELVIS] = B_SPINE1;             // not the left thigh

  const restLocal = new Array(NBONES);
  const restDir = new Array(NBONES);
  const restRef = new Array(NBONES);
  const restLen = new Float32Array(NBONES);

  const refHint = new Array(NBONES);
  for (let i = 0; i < NBONES; i++) refHint[i] = new THREE.Vector3(0, 0, -1);
  refHint[B_CLAVL].set(0, 1, 0); refHint[B_CLAVR].set(0, 1, 0);
  refHint[B_FOOTL].set(0, 1, 0); refHint[B_FOOTR].set(0, 1, 0);
  refHint[B_HANDL].set(0, 1, 0); refHint[B_HANDR].set(0, 1, 0);

  for (let i = 0; i < NBONES; i++) {
    const p = parent[i];
    restLocal[i] = p < 0 ? P[i].clone() : P[i].clone().sub(P[p]);
    const c = child[i];
    const t = c >= 0 ? P[c] : tip[i];
    const d = (t ? t.clone().sub(P[i]) : new THREE.Vector3(0, 1, 0));
    restLen[i] = d.length();
    if (restLen[i] < 1e-6) d.set(0, 1, 0); else d.multiplyScalar(1 / restLen[i]);
    restDir[i] = d;
    // Reference axis, perpendicularised against restDir.
    const r = (restBend[i] ? restBend[i].clone() : refHint[i].clone());
    r.addScaledVector(d, -r.dot(d));
    if (r.lengthSq() < 1e-8) {
      r.set(0, 1, 0).addScaledVector(d, -d.y);
      if (r.lengthSq() < 1e-8) r.set(1, 0, 0).addScaledVector(d, -d.x);
    }
    restRef[i] = r.normalize();
  }

  // Precomputed inverse rest basis per bone, so orientBone is one matrix multiply.
  const restBasisT = new Array(NBONES);
  for (let i = 0; i < NBONES; i++) {
    const a1 = restDir[i], a2 = restRef[i];
    const a3 = new THREE.Vector3().crossVectors(a1, a2);
    restBasisT[i] = new THREE.Matrix4().makeBasis(a1, a2, a3).transpose();
  }

  // THREE.Bone hierarchy in the rest pose.
  const bones = new Array(NBONES);
  for (let i = 0; i < NBONES; i++) {
    const b = new THREE.Bone();
    b.name = name[i];
    b.position.copy(restLocal[i]);
    bones[i] = b;
  }
  for (let i = 0; i < NBONES; i++) if (parent[i] >= 0) bones[parent[i]].add(bones[i]);

  // Runtime pose buffers.
  const localQuat = [], localPos = [], worldQuat = [], worldPos = [], ragQuat = [];
  for (let i = 0; i < NBONES; i++) {
    localQuat.push(new THREE.Quaternion());
    ragQuat.push(new THREE.Quaternion());
    localPos.push(restLocal[i].clone());
    worldQuat.push(new THREE.Quaternion());
    worldPos.push(new THREE.Vector3());
  }

  return {
    name, parent, child, restPos: P, restLocal, restDir, restRef, restLen, restBasisT,
    bones, localQuat, localPos, worldQuat, worldPos, ragQuat,
  };
}

/** Forward kinematics over the whole rig (23 bones — cheap, run it freely). */
function fk(rig) {
  const { parent, localQuat, localPos, worldQuat, worldPos } = rig;
  for (let i = 0; i < NBONES; i++) {
    const p = parent[i];
    if (p < 0) {
      worldQuat[i].copy(localQuat[i]);
      worldPos[i].copy(localPos[i]);
    } else {
      worldQuat[i].multiplyQuaternions(worldQuat[p], localQuat[i]);
      worldPos[i].copy(localPos[i]).applyQuaternion(worldQuat[p]).add(worldPos[p]);
    }
  }
}

/**
 * Rotate bone `i` so its rest child-direction points along `dir`, with the roll
 * about that axis chosen so the bone's rest reference axis lands on `ref`.
 * Writes the resulting world rotation into `outWorld` (and derives the local).
 */
function orientBone(rig, i, dir, ref, outWorldQuat) {
  const b1 = _v6.copy(dir);
  if (b1.lengthSq() < 1e-12) b1.copy(rig.restDir[i]); else b1.normalize();
  const b2 = _v7.copy(ref);
  b2.addScaledVector(b1, -b2.dot(b1));
  if (b2.lengthSq() < 1e-9) {
    b2.set(0, 1, 0).addScaledVector(b1, -b1.y);
    if (b2.lengthSq() < 1e-9) b2.set(1, 0, 0).addScaledVector(b1, -b1.x);
  }
  b2.normalize();
  const b3 = _v8.crossVectors(b1, b2);
  _m0.makeBasis(b1, b2, b3);
  _m0.multiply(rig.restBasisT[i]);
  outWorldQuat.setFromRotationMatrix(_m0);
}

/** Orient a bone by an explicit forward/up (used for the head look-at). */
function orientBoneFwdUp(dir, up, outWorldQuat) {
  const z = _v6.copy(dir).multiplyScalar(-1);
  if (z.lengthSq() < 1e-12) z.set(0, 0, 1); else z.normalize();
  const y = _v7.copy(up);
  y.addScaledVector(z, -y.dot(z));
  if (y.lengthSq() < 1e-9) y.set(0, 1, 0).addScaledVector(z, -z.y);
  y.normalize();
  const x = _v8.crossVectors(y, z);
  _m0.makeBasis(x, y, z);
  outWorldQuat.setFromRotationMatrix(_m0);
}

/** worldQuat -> localQuat given the parent's already-computed world rotation. */
function worldToLocalQuat(rig, i, worldQ, outLocal) {
  const p = rig.parent[i];
  if (p < 0) { outLocal.copy(worldQ); return; }
  _q0.copy(rig.worldQuat[p]).invert();
  outLocal.multiplyQuaternions(_q0, worldQ);
}

// ===========================================================================
// 6. Body construction
// ===========================================================================

// C5 — metres per normal-map tile. Was 0.085, which put the weave 4-8x below
// Nyquist at gameplay distance; see makeKitNormal for the full derivation.
const TILE = 0.28;

/**
 * Ring list along a bone polyline. Skin weights are assigned analytically from the
 * parametric position along each segment (50/50 exactly at a joint, falling to a
 * single influence `blend` of the way into the segment), which gives far cleaner
 * deformation than any distance-based auto-weighting.
 */
function chainRings(pts, boneIds, steps, opts) {
  const nSeg = pts.length - 1;
  const raw = [];
  for (let s = 0; s < nSeg; s++) {
    const last = s === nSeg - 1;
    const kMax = last ? steps : steps - 1;
    for (let k = 0; k <= kMax; k++) {
      const t = k / steps;
      raw.push({
        p: [lerp(pts[s][0], pts[s + 1][0], t), lerp(pts[s][1], pts[s + 1][1], t),
            lerp(pts[s][2], pts[s + 1][2], t)],
        s, t,
      });
    }
  }
  // Arclength parameterisation.
  let total = 0;
  const cum = [0];
  for (let i = 1; i < raw.length; i++) {
    total += Math.hypot(raw[i].p[0] - raw[i - 1].p[0], raw[i].p[1] - raw[i - 1].p[1],
                        raw[i].p[2] - raw[i - 1].p[2]);
    cum.push(total);
  }
  const frames = buildFrames(raw.map((r) => r.p), opts.upHint || [0, 0, -1], false);

  const blend = opts.blend !== undefined ? opts.blend : 0.42;
  const rings = [];
  const prof = [0, 0];
  for (let i = 0; i < raw.length; i++) {
    const r = raw[i];
    const U = total > 1e-6 ? cum[i] / total : 0;
    let b0 = boneIds[r.s], b1 = boneIds[r.s], w0 = 1;
    if (r.t > 1 - blend && r.s + 1 < boneIds.length) {
      b1 = boneIds[r.s + 1];
      w0 = 1 - 0.5 * smoothstep(1 - blend, 1, r.t);
    } else if (r.t < blend && r.s > 0) {
      b1 = boneIds[r.s - 1];
      w0 = 0.5 + 0.5 * smoothstep(0, blend, r.t);
    }
    opts.profile(U, prof);
    const rx = prof[0], ry = prof[1];
    rings.push({
      c: r.p, side: frames.side[i], up: frames.up[i], ax: frames.ax[i],
      rx, ry, pw: opts.pw ? opts.pw(U) : 2.0,
      b0, b1, w0, U,
      col: opts.col, surf: opts.surf ? opts.surf(U) : 0,
      flut: opts.flut ? opts.flut(U) : 0,
      tint: opts.tint === undefined ? 1 : opts.tint,
      uRep: Math.max(0.4, (Math.PI * (rx + ry)) / TILE),
      v: cum[i] / TILE,
    });
  }
  return rings;
}

/** Piecewise-linear lookup over [[u, value], ...] stops. */
function stops(table, u) {
  if (u <= table[0][0]) return table[0][1];
  for (let i = 1; i < table.length; i++) {
    if (u <= table[i][0]) {
      const t = (u - table[i - 1][0]) / Math.max(1e-6, table[i][0] - table[i - 1][0]);
      return lerp(table[i - 1][1], table[i][1], t);
    }
  }
  return table[table.length - 1][1];
}

function mixCol(a, b, t, out) {
  out[0] = a[0] + (b[0] - a[0]) * t;
  out[1] = a[1] + (b[1] - a[1]) * t;
  out[2] = a[2] + (b[2] - a[2]) * t;
  return out;
}

// ---------------------------------------------------------------------------
// 6a. Atlas painters.
//
// These are the whole point of C2. Each runs per atlas texel with
//   u  = fraction around the sweep (0 == +side, 0.25 == front, 0.75 == back),
//   v  = fraction along the chart grid,
//   o  = [r, g, b, aoMul, U]  — U is the arclength parameter the vertex colour
//        function saw, so a painted feature lands exactly on its colour band.
// At the packed density (~1.3 mm/texel) a 12 mm hem is nine texels wide with a
// step, not a 60 px interpolated gradient. That is the entire finding.
// ---------------------------------------------------------------------------

function mixInto(o, c, t) {
  if (t <= 0) return;
  if (t > 1) t = 1;
  o[0] += (c[0] - o[0]) * t; o[1] += (c[1] - o[1]) * t; o[2] += (c[2] - o[2]) * t;
}

/** Rounded rect coverage in chart space. */
function rrect(u, v, cu, cv, hu, hv, soft) {
  const du = Math.max(0, Math.abs(u - cu) - hu);
  const dv = Math.max(0, Math.abs(v - cv) - hv);
  const d = Math.sqrt(du * du + dv * dv);
  return 1 - smoothstep(0, soft, d);
}

/** Cover-stitch: a dark seam furrow flanked by two pale thread lines. */
function coverStitch(o, x, c, w) {
  const s = seamLine(x, c, w);
  if (s > 0) { mixInto(o, PAL.jerseyInk, s * 0.50); o[3] *= 1 - s * 0.28; }
  const th = seamLine(x, c - w * 2.4, w * 0.40) + seamLine(x, c + w * 2.4, w * 0.40);
  if (th > 0) mixInto(o, PAL.stitch, th * 0.16);
}

/** Seven-segment glyph, used for the back number board.
 *  bits: 1=top 2=top-right 4=bottom-right 8=bottom 16=bottom-left 32=top-left 64=middle */
const SEG_DIGITS = [0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f];
function digitAt(u, v, d) {
  const m = SEG_DIGITS[d] | 0;
  const T = 0.16, H = 0.5;                       // stroke thickness, half height
  const inX = u > -0.5 - T && u < 0.5 + T;
  if (!inX || v < -H - T || v > H + T) return 0;
  const hor = (yc) => (Math.abs(v - yc) < T * 0.5 && Math.abs(u) < 0.5 - T * 0.3) ? 1 : 0;
  const ver = (xc, yc) => (Math.abs(u - xc) < T * 0.5 && Math.abs(v - yc) < H * 0.5 - T * 0.2) ? 1 : 0;
  let a = 0;
  if (m & 1) a = Math.max(a, hor(H));
  if (m & 2) a = Math.max(a, ver(0.5, H * 0.5));
  if (m & 4) a = Math.max(a, ver(0.5, -H * 0.5));
  if (m & 8) a = Math.max(a, hor(-H));
  if (m & 16) a = Math.max(a, ver(-0.5, -H * 0.5));
  if (m & 32) a = Math.max(a, ver(-0.5, H * 0.5));
  if (m & 64) a = Math.max(a, hor(0));
  return a;
}

/** Weave-scale value break so a flat panel is never algebraically flat. */
function heather(o, u, v, amt) {
  const n = hash2(Math.floor(u * 900), Math.floor(v * 900)) * 0.6
          + hash2(Math.floor(u * 190) + 7, Math.floor(v * 190) + 3) * 0.4;
  const k = 1 + (n - 0.5) * amt;
  o[0] *= k; o[1] *= k; o[2] *= k;
}

/** Mud/dust that collects low on a garment and in its creases. */
function grime(o, u, v, strength) {
  if (strength <= 0) return;
  const sp = hash2(Math.floor(u * 240) * 1.7, Math.floor(v * 260) * 1.3);
  const blob = Math.pow(hash2(Math.floor(u * 64), Math.floor(v * 72)), 3.0);
  mixInto(o, PAL.grime, clamp01(strength) * (sp * 0.26 + blob * 0.30));
}

// --- jersey / torso ---------------------------------------------------------
function dJersey(u, v, o) {
  const U = o[4];
  // Side seams down the lateral extremes, plus the shoulder yoke.
  coverStitch(o, u, 0.0, 0.0035); coverStitch(o, u, 1.0, 0.0035);
  coverStitch(o, u, 0.5, 0.0035);
  coverStitch(o, U, 0.815, 0.0032);
  // Hem cover stitch — the real hem lip is separate geometry sitting below this.
  coverStitch(o, U, 0.055, 0.0030);
  // Raglan sleeve seam sweeping off the yoke toward the armpit.
  const rag = seamLine(U, 0.705 + 0.075 * Math.cos((u - 0.25) * Math.PI * 2), 0.0028);
  if (rag > 0) { mixInto(o, PAL.jerseyInk, rag * 0.30); o[3] *= 1 - rag * 0.18; }

  // Chest print block and a sponsor bar beneath it (front, u ~ 0.25).
  const chest = rrect(u, U, 0.25, 0.585, 0.055, 0.030, 0.010);
  if (chest > 0) {
    mixInto(o, PAL.jerseyPale, chest * 0.92);
    const ink = rrect(u, U, 0.25, 0.585, 0.040, 0.008, 0.004);
    mixInto(o, PAL.jerseyInk, ink * 0.85);
  }
  const bar = rrect(u, U, 0.25, 0.495, 0.070, 0.008, 0.005);
  mixInto(o, PAL.jerseyAccent, bar * 0.85);

  // Back number board. (The board, the print blocks and the digits are the code
  // path an authored decal atlas would feed; PART 4 puts the painted art itself
  // out of engineering scope, so this is the procedural stand-in for it.)
  const board = rrect(u, U, 0.75, 0.520, 0.088, 0.105, 0.008);
  if (board > 0) {
    mixInto(o, PAL.jerseyPale, board * 0.95);
    const gu = (u - 0.75) / 0.064, gv = (U - 0.520) / 0.160;
    const d = digitAt(gu + 0.70, gv, 2) + digitAt(gu - 0.70, gv, 7);
    mixInto(o, PAL.jerseyInk, clamp01(d) * board * 0.94);
  }

  heather(o, u, v, 0.055);
  grime(o, u, v, smoothstep(0.34, 0.02, U) * 0.85);
  // Crease darkening in the fold field the normal map is also using.
  const cr = Math.pow(1 - Math.abs(hash2(Math.floor(u * 30), Math.floor(v * 34)) * 2 - 1), 3);
  o[3] *= 1 - cr * 0.12;
}

/** Sleeve: the jersey's seams and wear, but none of its print blocks. */
function dSleeve(u, v, o) {
  const U = o[4];
  coverStitch(o, u, 0.0, 0.0032); coverStitch(o, u, 1.0, 0.0032);
  coverStitch(o, u, 0.5, 0.0032);
  coverStitch(o, U, 0.055, 0.0030);          // shoulder seam into the deltoid
  coverStitch(o, U, 0.885, 0.0030);          // cuff seam
  heather(o, u, v, 0.055);
  grime(o, u, v, smoothstep(0.60, 0.95, U) * 0.7);
  const cr = Math.pow(1 - Math.abs(hash2(Math.floor(u * 30), Math.floor(v * 34)) * 2 - 1), 3);
  o[3] *= 1 - cr * 0.12;
}

// --- shorts + sock ----------------------------------------------------------
function dLeg(u, v, o) {
  const U = o[4];
  coverStitch(o, u, 0.0, 0.0035); coverStitch(o, u, 1.0, 0.0035);
  coverStitch(o, u, 0.5, 0.0035);
  if (U < 0.44) {
    // Thigh vent panel with a stitched outline, and the short's hem stitch.
    const panel = rrect(u, U, 0.12, 0.20, 0.055, 0.075, 0.006);
    const inner = rrect(u, U, 0.12, 0.20, 0.049, 0.069, 0.004);
    mixInto(o, PAL.shortsPanel, (panel - inner) * 0.9);
    mixInto(o, PAL.shortsInk, inner * 0.22);
    coverStitch(o, U, 0.408, 0.0030);
    grime(o, u, v, smoothstep(0.44, 0.20, U) * 0.7);
  } else {
    // Sock ribbing: fine vertical ribs, hard-edged at texel rate.
    const rib = 0.5 + 0.5 * Math.cos(u * Math.PI * 2 * 34);
    const k = 1 - Math.pow(rib, 3) * 0.16;
    o[0] *= k; o[1] *= k; o[2] *= k;
    o[3] *= 1 - Math.pow(rib, 4) * 0.10;
    coverStitch(o, U, 0.575, 0.0026);
  }
  heather(o, u, v, 0.05);
}

// --- knee pad ---------------------------------------------------------------
function dPad(u, v, o) {
  const U = o[4];
  // Hard-shell panel splits, front-centred at u = 0.25.
  const du = Math.abs(u - 0.25);
  const split = seamLine(U, 0.32, 0.006) + seamLine(U, 0.66, 0.006);
  const onCap = (1 - smoothstep(0.20, 0.235, du)) * smoothstep(0.10, 0.14, U)
              * (1 - smoothstep(0.86, 0.92, U));
  if (split * onCap > 0) { mixInto(o, PAL.shortsInk, split * onCap * 0.7); o[3] *= 1 - split * onCap * 0.4; }
  // Edge binding round the shell, then an elastic sleeve outside it.
  const edge = (1 - smoothstep(0.185, 0.215, du)) - (1 - smoothstep(0.205, 0.232, du));
  mixInto(o, PAL.padCap, clamp01(edge) * onCap * 0.55);
  const sleeve = 1 - onCap;
  if (sleeve > 0.5) {
    const rib = 0.5 + 0.5 * Math.cos(U * Math.PI * 2 * 46);
    const k = 1 - Math.pow(rib, 3) * 0.20;
    o[0] *= k; o[1] *= k; o[2] *= k;
    mixInto(o, PAL.padSleeve, 0.35);
  }
  // Moulded brand block on the cap.
  const logo = rrect(u, U, 0.25, 0.50, 0.030, 0.012, 0.006);
  mixInto(o, PAL.padCap, logo * 0.75);
  heather(o, u, v, 0.04);
  grime(o, u, v, 0.35);
}

// --- glove ------------------------------------------------------------------
function dGlovePalm(u, v, o) {
  // Silicone print on the palm side (u ~ 0.75 == the bar side, see buildHand),
  // a knuckle panel seam on the back, and a stitched thumb gusset.
  const palm = clamp01(Math.cos((u - 0.75) * Math.PI * 2) * 1.1);
  if (palm > 0.02) {
    const gx = u * 46, gy = v * 26;
    const dx = gx - Math.floor(gx) - 0.5, dy = gy - Math.floor(gy) - 0.5;
    const dot = 1 - smoothstep(0.20, 0.30, Math.sqrt(dx * dx + dy * dy));
    mixInto(o, PAL.gloveGrip, dot * palm * 0.85);
    o[3] *= 1 - dot * palm * 0.10;
  }
  const back = clamp01(-Math.cos((u - 0.75) * Math.PI * 2) * 1.2);
  const kn = seamLine(v, 0.34, 0.006) + seamLine(v, 0.62, 0.006);
  if (kn * back > 0) { mixInto(o, PAL.shortsInk, kn * back * 0.6); o[3] *= 1 - kn * back * 0.35; }
  coverStitch(o, u, 0.0, 0.003); coverStitch(o, u, 1.0, 0.003);
  heather(o, u, v, 0.05);
  grime(o, u, v, 0.5);
}

function dGloveFinger(u, v, o) {
  // Knuckle creases across the finger, and a seam down the side of it.
  const cr = seamLine(v, 0.30, 0.030) + seamLine(v, 0.62, 0.026);
  o[3] *= 1 - cr * 0.30;
  mixInto(o, PAL.shortsInk, cr * 0.20);
  coverStitch(o, u, 0.25, 0.010);
  grime(o, u, v, 0.5);
}

// --- shoe -------------------------------------------------------------------
function dSoleTread(u, v, o) {
  // Directional lug blocks. The groove is a hard step, which is what makes a
  // sole read as a sole rather than a brown ellipse.
  const su = u * 6.0, sv = v * 17.0 + u * 1.6;
  const fu = su - Math.floor(su), fv = sv - Math.floor(sv);
  const lug = (1 - smoothstep(0.30, 0.40, Math.abs(fu - 0.5)))
            * (1 - smoothstep(0.30, 0.40, Math.abs(fv - 0.5)));
  const groove = 1 - lug;
  mixInto(o, PAL.shortsInk, groove * 0.55);
  o[3] *= 1 - groove * 0.45;
  // Toe and heel rand in a lighter rubber.
  mixInto(o, PAL.shoeToe, (smoothstep(0.90, 0.99, v) + smoothstep(0.10, 0.01, v)) * 0.5);
  grime(o, u, v, 0.85);
}

function dShoeUpper(u, v, o) {
  // Lace panel with eyelets down the instep (u ~ 0.25 == the top of the foot).
  const top = clamp01(Math.cos((u - 0.25) * Math.PI * 2) * 1.4);
  const panel = top * smoothstep(0.10, 0.20, v) * (1 - smoothstep(0.58, 0.70, v));
  mixInto(o, PAL.shortsInk, panel * 0.30);
  const rows = 5;
  for (let i = 0; i < rows; i++) {
    const lv = 0.18 + i * 0.088;
    const e = rrect(u, v, 0.25 - 0.030, lv, 0.002, 0.002, 0.004)
            + rrect(u, v, 0.25 + 0.030, lv, 0.002, 0.002, 0.004);
    mixInto(o, PAL.lace, e * 0.7);
    o[3] *= 1 - e * 0.4;
    const cross = seamLine(u, 0.25, 0.028) * band(v, lv + 0.044, 0.004, 0.004);
    mixInto(o, PAL.lace, cross * 0.65);
  }
  // Toe cap and heel counter seams.
  coverStitch(o, v, 0.735, 0.004);
  coverStitch(o, v, 0.115, 0.004);
  // Sole rand line where the upper meets the slab.
  const rand = clamp01(Math.cos((u - 0.75) * Math.PI * 2) * 1.6);
  mixInto(o, PAL.shoeSole, rand * 0.35);
  heather(o, u, v, 0.05);
  grime(o, u, v, 0.9);
}

// --- helmet -----------------------------------------------------------------
function dHelmetShell(u, v, o) {
  const U = o[4];
  const du = Math.min(Math.abs(u - 0.25), Math.abs(u - 0.75));   // 0 at front/back centre
  // The shell's radius collapses toward the crown, so a rect in (u, U) is a
  // trapezoid on the surface. Undo that here. (Mirrors buildBody's sFac: rings
  // run dy = 0.122 -> -0.133 over U, SRY = 0.127.)
  const dy = 0.122 - 0.255 * U;
  const rf = Math.max(0.34, dy > 0
    ? Math.sqrt(Math.max(0, 1 - (dy / 0.127) * (dy / 0.127)))
    : 1 - 0.20 * (dy / 0.136) * (dy / 0.136));
  const w = (mm) => mm / rf;      // u half-width for a given proportional width

  // Brow vents: three slots ABOVE the eye-port brow line (the port is masked out
  // below U ~ 0.227, so anything lower would be painted into the hole).
  let vent = 0;
  for (let i = 0; i < 3; i++) {
    const vv = 0.075 + i * 0.062;
    vent += rrect(u, U, 0.25 - w(0.048), vv, w(0.011), 0.020, 0.004)
          + rrect(u, U, 0.25 + w(0.048), vv, w(0.011), 0.020, 0.004);
  }
  // Rear exhausts.
  vent += rrect(u, U, 0.75 - w(0.036), 0.30, w(0.013), 0.034, 0.005)
        + rrect(u, U, 0.75 + w(0.036), 0.30, w(0.013), 0.034, 0.005);
  vent = clamp01(vent);
  if (vent > 0) { mixInto(o, PAL.helmetIn, vent * 0.94); o[3] *= 1 - vent * 0.55; }

  // A crisp graphic wing instead of a soft interpolated stripe.
  const wing = (1 - smoothstep(w(0.030), w(0.038), du)) * smoothstep(0.05, 0.09, U);
  mixInto(o, PAL.helmetStripe, wing * 0.85);
  const pin = band(du, w(0.049), w(0.0026), w(0.0016)) * smoothstep(0.05, 0.09, U);
  mixInto(o, PAL.helmetPale, pin * 0.8);
  // Side flash: a swept graphic across the temple, which is what a helmet is
  // read by in profile — the round-3 shell was one flat value there.
  const side = Math.min(Math.abs(u - 0.0), Math.abs(u - 0.5), Math.abs(u - 1.0));
  const flash = (1 - smoothstep(0.055, 0.085, side))
              * band(U, 0.50 + 0.10 * smoothstep(0.02, 0.12, side), 0.055, 0.035);
  mixInto(o, PAL.helmetStripe, flash * 0.55);
  mixInto(o, PAL.helmetPale, band(side, 0.092, 0.004, 0.004) * band(U, 0.53, 0.075, 0.04) * 0.6);

  // Shell edge trim round the bottom of the skirt.
  const trim = smoothstep(0.965, 0.995, U);
  mixInto(o, PAL.helmetIn, trim * 0.8);
  // Fine flake + a low-frequency clearcoat value break.
  heather(o, u, v, 0.035);
  o[3] *= 1 - smoothstep(0.86, 1.0, U) * 0.16;     // shading under the skirt
}

function dFace(u, v, o) {
  const U = o[4];
  // Brow, eye sockets and a nose shadow — the head is nearly always seen through
  // the eye port, so it has to read as a face rather than a pink ellipsoid.
  const front = Math.sin(u * Math.PI * 2);
  const f = clamp01(front);
  const brow = band(U, 0.700, 0.014, 0.020) * f;
  mixInto(o, PAL.brow, brow * 0.55);
  const eye = (rrect(u, U, 0.25 - 0.055, 0.655, 0.026, 0.012, 0.012)
             + rrect(u, U, 0.25 + 0.055, 0.655, 0.026, 0.012, 0.012));
  mixInto(o, PAL.skinShade, clamp01(eye) * 0.75);
  o[3] *= 1 - clamp01(eye) * 0.30;
  const nose = rrect(u, U, 0.25, 0.545, 0.016, 0.055, 0.030) * f;
  o[3] *= 1 - nose * 0.10;
  const nostril = (rrect(u, U, 0.25 - 0.016, 0.492, 0.005, 0.004, 0.006)
                 + rrect(u, U, 0.25 + 0.016, 0.492, 0.005, 0.004, 0.006));
  mixInto(o, PAL.skinShade, clamp01(nostril) * 0.8);
  const mouth = rrect(u, U, 0.25, 0.395, 0.036, 0.010, 0.010) * f;
  mixInto(o, PAL.lip, mouth * 0.6);
  const line = rrect(u, U, 0.25, 0.395, 0.034, 0.0015, 0.003) * f;
  mixInto(o, PAL.skinShade, line * 0.7);
  // Stubble low on the jaw.
  const st = hash2(Math.floor(u * 700), Math.floor(v * 700));
  mixInto(o, PAL.brow, smoothstep(0.42, 0.16, U) * f * st * 0.30);
  heather(o, u, v, 0.04);
  // The head is inside a full-face helmet, so it is deeply occluded everywhere
  // except the small solid angle the eye port subtends. This is the other half
  // of C1: without it the face is the brightest thing on the character.
  const openness = f * (1 - smoothstep(0.70, 0.94, U));
  o[3] *= lerp(0.26, 0.84, openness);
}

function dNeck(u, v, o) {
  heather(o, u, v, 0.05);
  o[3] *= lerp(0.52, 0.80, clamp01(Math.sin(u * Math.PI * 2) * 0.5 + 0.5));
}

/** Generic soft goods: side seams + a hem stitch + wear. */
function dTrim(u, v, o) {
  coverStitch(o, u, 0.0, 0.004); coverStitch(o, u, 1.0, 0.004);
  coverStitch(o, u, 0.5, 0.004);
  o[3] *= 1 - smoothstep(0.75, 1.0, v) * 0.18;
  heather(o, u, v, 0.05);
  grime(o, u, v, 0.4);
}

/** Helmet liner: EPS + comfort foam, seen through the eye port. */
function dLiner(u, v, o) {
  const cell = hash2(Math.floor(u * 150), Math.floor(v * 150));
  mixInto(o, PAL.linerPad, cell * 0.35);
  // Channelled foam pads: broad soft blocks with dark gaps.
  const bu = Math.abs((u * 7) % 1 - 0.5), bv = Math.abs((v * 5) % 1 - 0.5);
  const gap = smoothstep(0.40, 0.48, Math.max(bu, bv));
  o[3] *= 1 - gap * 0.35;
  o[3] *= 0.72;                                     // it is inside a helmet
}

function buildBody(rig, quality) {
  const lo = quality === 'low';
  // Silhouette quality matters more than triangle count here — the rider is on
  // screen 100% of the time and a faceted forearm is an instant tell.
  const R_TORSO = lo ? 12 : 22, R_LIMB = lo ? 10 : 16, R_ARM = lo ? 10 : 14,
        R_HEAD = lo ? 12 : 20, R_HELM = lo ? 16 : 30, R_SMALL = lo ? 8 : 14,
        R_TINY = lo ? 6 : 9;
  const STEP = lo ? 4 : 7;

  const G = makeBuilder(4);
  const P = rig.restPos;
  const gS = G[G_SKIN], gK = G[G_KIT], gH = G[G_HELMET], gL = G[G_LENS];
  const v3 = (p) => [p.x, p.y, p.z];
  const along = (p, d, s) => [p[0] + d.x * s, p[1] + d.y * s, p[2] + d.z * s];

  // ---------------------------------------------------------------- torso ---
  // Runs from the jersey hem below the pelvis up to the collar, so the fabric
  // silhouette (not the body) is what you actually see. C7: every radius here
  // is ~12% down on round 3, where the torso read as a barrel wider than the bar.
  const T_RX = [[0, 0.150], [0.16, 0.156], [0.30, 0.146], [0.50, 0.140], [0.72, 0.170],
                [0.88, 0.162], [1, 0.113]];
  const T_RY = [[0, 0.126], [0.16, 0.126], [0.30, 0.118], [0.50, 0.111], [0.72, 0.123],
                [0.88, 0.117], [1, 0.092]];
  const hemPt = [P[B_PELVIS].x, P[B_PELVIS].y - 0.100, P[B_PELVIS].z + 0.046];
  {
    const top = [P[B_SPINE3].x, P[B_SPINE3].y + 0.050, P[B_SPINE3].z - 0.026];
    const pts = [hemPt, v3(P[B_PELVIS]), v3(P[B_SPINE1]), v3(P[B_SPINE2]), v3(P[B_SPINE3]), top];
    const ids = [B_PELVIS, B_PELVIS, B_SPINE1, B_SPINE2, B_SPINE3, B_SPINE3];
    const rings = chainRings(pts, ids, STEP, {
      upHint: [0, 0, -1],
      profile: (u, o) => { o[0] = stops(T_RX, u); o[1] = stops(T_RY, u); },
      pw: (u) => lerp(2.15, 2.7, smoothstep(0.2, 0.75, u)),
      surf: () => 0,
      flut: (u) => stops([[0, 0.95], [0.18, 0.70], [0.36, 0.30], [0.6, 0.10], [1, 0.04]], u),
      col: (k, radial, ring, out) => {
        const t = (k / radial) * Math.PI * 2;
        const front = Math.sin(t);          // +1 = chest, −1 = back
        const u = ring.U;
        // Base jersey with a darker back panel and side panels.
        mixCol(PAL.jersey, PAL.jerseyDark, 0.55 - front * 0.45, out);
        // Chest/shoulder accent band; at atlas rate this is now a crisp edge.
        const band2 = smoothstep(0.615, 0.628, u) * (1 - smoothstep(0.757, 0.770, u));
        mixCol(out, PAL.jerseyAccent, band2 * (0.35 + 0.45 * clamp01(front + 0.5)), out);
        const pin = smoothstep(0.788, 0.793, u) * (1 - smoothstep(0.806, 0.811, u));
        mixCol(out, PAL.jerseyPale, pin * 0.85, out);
        mixCol(out, PAL.jerseyDark, smoothstep(0.16, 0, u) * 0.5, out);
        return out;
      },
    });
    emitTube(gK, rings, R_TORSO, {
      capStart: true, capEnd: true, kind: 'jersey', detail: dJersey,
      // C5 — macro cloth. A jersey is read from its gathers, not its weave: a
      // waist gather where the fabric bunches over the shorts, and slack folds
      // under the arms. These are geometric so they survive to any distance.
      warp: (ri, k, radial, r, p) => {
        const U = r.U, t = (k / radial) * Math.PI * 2;
        const gather = Math.pow(Math.max(0, 1 - Math.abs(U - 0.26) / 0.17), 2);
        const w = Math.sin(t * 5.0 + U * 11.0) * 0.0060 * gather;
        const slackW = Math.pow(Math.max(0, 1 - Math.abs(U - 0.655) / 0.15), 2);
        const slack = slackW * Math.abs(Math.cos(t)) * Math.sin(t * 3.0 + 1.1) * 0.0045;
        const cx = p[0] - r.c[0], cy = p[1] - r.c[1], cz = p[2] - r.c[2];
        const l = Math.sqrt(cx * cx + cy * cy + cz * cz) || 1;
        const s = (w + slack) / l;
        p[0] += cx * s; p[1] += cy * s; p[2] += cz * s;
      },
    });

    // Jersey hem — a real lip with a fold-back, so a hem EXISTS in silhouette.
    // (C2: on the round-3 build there was no hem, seam, cuff or sole at any
    // distance because every one of them was a per-vertex gradient.)
    const hb = [
      [hemPt[0], hemPt[1] + 0.034, hemPt[2] - 0.004],
      [hemPt[0], hemPt[1] + 0.012, hemPt[2] - 0.001],
      [hemPt[0], hemPt[1] - 0.002, hemPt[2] + 0.001],
      [hemPt[0], hemPt[1] - 0.008, hemPt[2] + 0.002],
    ];
    const hr = chainRings(hb, [B_PELVIS, B_PELVIS, B_PELVIS, B_PELVIS], 1, {
      upHint: [0, 0, -1], blend: 0.5,
      profile: (u, o) => {
        const k = stops([[0, 1.000], [0.34, 1.055], [0.70, 1.050], [1, 0.965]], u);
        o[0] = stops(T_RX, 0.02) * k; o[1] = stops(T_RY, 0.02) * k;
      },
      pw: () => 2.2, surf: () => 0, flut: () => 1.0,
      col: (k, radial, ring, out) => {
        mixCol(PAL.jerseyDark, PAL.jersey, 0.35, out);
        return out;
      },
    });
    emitTube(gK, hr, R_TORSO, { kind: 'hem', detail: dTrim });
  }

  // ------------------------------------------------------------- hip block ---
  // Fills the crotch/seat between the two thigh tubes.
  {
    const a = [P[B_PELVIS].x, P[B_PELVIS].y + 0.028, P[B_PELVIS].z];
    const b = [P[B_PELVIS].x, P[B_PELVIS].y - 0.094, P[B_PELVIS].z - 0.018];
    const rings = chainRings([a, b], [B_PELVIS, B_PELVIS], 3, {
      upHint: [0, 0, -1],
      // Kept strictly inside the jersey's local radius (T_RX ~0.146 at this
      // height) so the shorts cannot punch through the jersey shell.
      profile: (u, o) => { o[0] = 0.138 - u * 0.024; o[1] = 0.112 - u * 0.012; },
      pw: () => 2.4,
      surf: () => 0,
      flut: (u) => u * 0.35,
      col: PAL.shorts,
    });
    emitTube(gK, rings, R_TORSO, { capStart: false, capEnd: true, kind: 'shorts', detail: dTrim });

    // Shorts waistband — a proud elasticated band with a ratchet closure.
    const wb = [
      [a[0], a[1] + 0.030, a[2] + 0.002], [a[0], a[1] + 0.010, a[2] + 0.001],
      [a[0], a[1] - 0.012, a[2]], [a[0], a[1] - 0.026, a[2] - 0.002],
    ];
    const wr = chainRings(wb, [B_PELVIS, B_PELVIS, B_PELVIS, B_PELVIS], 1, {
      upHint: [0, 0, -1], blend: 0.5,
      profile: (u, o) => {
        const k = stops([[0, 0.965], [0.30, 1.035], [0.78, 1.030], [1, 0.980]], u);
        o[0] = 0.134 * k; o[1] = 0.109 * k;
      },
      pw: () => 2.4, surf: () => 0, flut: () => 0,
      col: (k, radial, ring, out) => {
        const t = (k / radial) * Math.PI * 2;
        mixCol(PAL.shortsInk, PAL.shorts, 0.5 + Math.sin(t) * 0.2, out);
        return out;
      },
    });
    emitTube(gK, wr, R_TORSO, { kind: 'band', detail: dTrim });
  }

  // --------------------------------------------------------------- legs ------
  // C7 — thigh 224 mm across the baggy short (was 284 mm), calf 152 mm.
  const L_RX = [[0, 0.112], [0.18, 0.107], [0.36, 0.112], [0.44, 0.088], [0.505, 0.064],
                [0.60, 0.076], [0.74, 0.068], [0.90, 0.047], [1, 0.040]];
  for (const s of [-1, 1]) {
    const iT = s < 0 ? B_THIGHL : B_THIGHR, iS = s < 0 ? B_SHINL : B_SHINR,
          iF = s < 0 ? B_FOOTL : B_FOOTR, iToe = s < 0 ? B_TOEL : B_TOER;
    const pts = [v3(P[iT]), v3(P[iS]), v3(P[iF])];
    const ids = [iT, iS, iF];
    const rings = chainRings(pts, ids, STEP + 2, {
      upHint: [0, 0, -1],
      profile: (u, o) => {
        const r = stops(L_RX, u);
        o[0] = r; o[1] = r * (u < 0.42 ? 1.04 : 0.95);
      },
      pw: (u) => lerp(2.3, 2.05, smoothstep(0.35, 0.6, u)),
      surf: (u) => (u < 0.42 ? 0 : 0.35),
      flut: (u) => stops([[0, 0.25], [0.18, 0.45], [0.34, 0.72], [0.44, 0.05], [1, 0]], u),
      col: (k, radial, ring, out) => {
        const t = (k / radial) * Math.PI * 2;
        const front = Math.sin(t), side = Math.abs(Math.cos(t));
        const u = ring.U;
        if (u < 0.43) {
          mixCol(PAL.shorts, PAL.shortsPanel, clamp01(side - 0.55) * 1.6, out);
          mixCol(out, PAL.jerseyAccent,
            smoothstep(0.316, 0.325, u) * (1 - smoothstep(0.348, 0.357, u)) * 0.75, out);
        } else {
          mixCol(PAL.sock, PAL.pad, clamp01(front) * 0.25, out);
          const band2 = smoothstep(0.556, 0.564, u) * (1 - smoothstep(0.598, 0.606, u));
          mixCol(out, PAL.sockBand, band2 * 0.9, out);
        }
        return out;
      },
    });
    emitTube(gK, rings, R_LIMB, {
      capStart: true, capEnd: false, kind: 'leg', detail: dLeg,
      warp: (ri, k, radial, r, p) => {
        // Baggy-short slack: vertical folds that die out at the hem.
        const U = r.U;
        if (U > 0.40) return;
        const t = (k / radial) * Math.PI * 2;
        const w = Math.sin(t * 4.0 + U * 7.0) * 0.0055
                * Math.pow(Math.max(0, 1 - Math.abs(U - 0.22) / 0.20), 1.6);
        const cx = p[0] - r.c[0], cy = p[1] - r.c[1], cz = p[2] - r.c[2];
        const l = Math.sqrt(cx * cx + cy * cy + cz * cz) || 1;
        p[0] += cx * w / l; p[1] += cy * w / l; p[2] += cz * w / l;
      },
    });

    // Short leg-opening hem, at the U where the profile steps down to the knee.
    {
      const hip = P[iT], knee = P[iS];
      const d = new THREE.Vector3().subVectors(knee, hip).normalize();
      const base = along(v3(hip), d, 0.300);
      const hb = [along(base, d, -0.024), along(base, d, -0.004),
                  along(base, d, 0.010), along(base, d, 0.019)];
      const hr = chainRings(hb, [iT, iT, iT, iT], 1, {
        upHint: [0, 0, -1], blend: 0.5,
        profile: (u, o) => {
          const k = stops([[0, 1.00], [0.34, 1.075], [0.72, 1.065], [1, 0.97]], u);
          o[0] = 0.113 * k; o[1] = 0.117 * k;
        },
        pw: () => 2.25, surf: () => 0, flut: () => 0.65,  // matches the short at this U
        col: (k, radial, ring, out) => { mixCol(PAL.shorts, PAL.shortsInk, 0.30, out); return out; },
      });
      emitTube(gK, hr, R_LIMB, { kind: 'hem', detail: dTrim });
    }

    // ------------------------------------------------------------ knee pad ---
    // C3 — this used to be the same knobbly ellipsoid as the glove and the shoe.
    // It is now a hard shield sitting proud of a ribbed elastic sleeve, with a
    // hard-edged rim you can see in silhouette.
    {
      const hip = P[iT], knee = P[iS], ankle = P[iF];
      const dUp = new THREE.Vector3().subVectors(hip, knee).normalize();
      const dDn = new THREE.Vector3().subVectors(ankle, knee).normalize();
      const a = along(v3(knee), dUp, 0.112);
      const m = v3(knee);
      const b = along(v3(knee), dDn, 0.130);
      const rings = chainRings([a, m, b], [iT, iS, iS], 3, {
        upHint: [0, 0, -1], blend: 0.5,
        profile: (u, o) => {
          const r = 0.074 + 0.020 * Math.sin(Math.PI * clamp01(u));
          o[0] = r * 0.96; o[1] = r;
        },
        pw: () => 2.35,
        surf: () => 1,
        flut: () => 0,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          const front = Math.sin(t);
          mixCol(PAL.padSleeve, PAL.pad, clamp01(front + 0.2) * 0.9, out);
          mixCol(out, PAL.padCap, clamp01(front - 0.25) * 1.1 *
            (1 - Math.abs(ring.U - 0.45) * 1.4), out);
          return out;
        },
      });
      emitTube(gK, rings, R_LIMB, {
        kind: 'pad', detail: dPad,
        warp: (ri, k, radial, r, p) => {
          // Shield: a hard-edged proud cap over the front 230 deg of the knee,
          // segmented into three moulded plates.
          const t = (k / radial) * Math.PI * 2;
          const U = r.U;
          const az = Math.abs(Math.atan2(Math.cos(t), Math.sin(t)));   // 0 at front
          const lat = 1 - smoothstep(1.05, 1.20, az);
          const lon = smoothstep(0.09, 0.14, U) * (1 - smoothstep(0.86, 0.92, U));
          let amp = lat * lon * 0.0145;
          // Plate splits: two grooves across the cap.
          amp -= lat * lon * 0.0055 *
            (band(U, 0.32, 0.010, 0.012) + band(U, 0.66, 0.010, 0.012));
          const cx = p[0] - r.c[0], cy = p[1] - r.c[1], cz = p[2] - r.c[2];
          const l = Math.sqrt(cx * cx + cy * cy + cz * cz) || 1;
          p[0] += cx * amp / l; p[1] += cy * amp / l; p[2] += cz * amp / l;
        },
      });
    }

    // ---------------------------------------------------------------- shoe ---
    // C3 — a flat-soled shoe whose sole plane mates with the pedal cage, built
    // in the foot's own frame so it stays flat when footPitch drives the ankle.
    {
      const ank = P[iF], toe = P[iToe];
      const fwd = new THREE.Vector3().subVectors(toe, ank).normalize();
      const sideV = new THREE.Vector3(1, 0, 0);
      sideV.addScaledVector(fwd, -sideV.dot(fwd)).normalize();
      const upV = new THREE.Vector3().crossVectors(sideV, fwd).normalize();
      const pt = (a, h) => [
        ank.x + fwd.x * a + upV.x * h,
        ank.y + fwd.y * a + upV.y * h,
        ank.z + fwd.z * a + upV.z * h,
      ];

      // Sole slab. pw 5 plus a hard clamp on the underside gives a true plane.
      const soleH = -(SOLE_DROP - SOLE_THICK * 0.5);
      const spts = [pt(-0.070, soleH), pt(-0.030, soleH), pt(0.030, soleH),
                    pt(0.100, soleH), pt(0.155, soleH), pt(0.190, soleH)];
      const sids = spts.map(() => iF);
      const srings = chainRings(spts, sids, 1, {
        upHint: [upV.x, upV.y, upV.z], blend: 0.5,
        profile: (u, o) => {
          o[0] = stops([[0, 0.036], [0.18, 0.046], [0.45, 0.049], [0.74, 0.045],
                        [0.90, 0.038], [1, 0.024]], u);
          o[1] = SOLE_THICK * 0.5;
        },
        pw: () => 5.0, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          const down = -Math.sin(t);
          mixCol(PAL.shoeToe, PAL.shoeSole, clamp01(down * 1.4 + 0.15), out);
          return out;
        },
      });
      emitTube(gK, srings, R_SMALL, {
        capStart: true, capEnd: true, kind: 'sole', detail: dSoleTread,
        warp: (ri, k, radial, r, p) => {
          const dy = (p[0] - r.c[0]) * upV.x + (p[1] - r.c[1]) * upV.y + (p[2] - r.c[2]) * upV.z;
          const flat = -SOLE_THICK * 0.5;
          if (dy < flat) {
            const d = flat - dy;
            p[0] += upV.x * d; p[1] += upV.y * d; p[2] += upV.z * d;
          }
        },
      });

      // Upper: heel counter, ankle collar, instep, toe box.
      const uA = [-0.062, -0.025, 0.025, 0.085, 0.140, 0.180];
      const uRy = [0.050, 0.052, 0.047, 0.038, 0.027, 0.015];
      const upts = uA.map((a, i) => pt(a, -0.064 + uRy[i]));
      const urings = chainRings(upts, upts.map(() => iF), 1, {
        upHint: [upV.x, upV.y, upV.z], blend: 0.5,
        profile: (u, o) => {
          o[0] = stops([[0, 0.034], [0.152, 0.045], [0.357, 0.047], [0.603, 0.044],
                        [0.833, 0.036], [1, 0.020]], u);
          o[1] = stops([[0, 0.050], [0.152, 0.052], [0.357, 0.047], [0.603, 0.038],
                        [0.833, 0.027], [1, 0.015]], u);
        },
        pw: () => 3.0, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          const u = ring.U;
          mixCol(PAL.shoe, PAL.shoeToe, clamp01(Math.sin(t) * 0.4 + 0.25) * 0.5, out);
          mixCol(out, PAL.shoeToe, smoothstep(0.80, 0.94, u) * 0.7, out);
          mixCol(out, PAL.shoeTrim,
            smoothstep(0.118, 0.128, u) * (1 - smoothstep(0.150, 0.160, u)) * 0.65, out);
          return out;
        },
      });
      emitTube(gK, urings, R_SMALL, {
        capStart: true, capEnd: true, kind: 'shoe', detail: dShoeUpper,
      });

      // Instep ratchet strap — the single cheapest read of "this is a shoe".
      {
        const sp = [];
        const NS = lo ? 5 : 8;
        for (let i = 0; i <= NS; i++) {
          const th = lerp(-1.45, 1.45, i / NS);
          const rx2 = 0.049, ry2 = 0.048;
          const c = pt(0.062, -0.020);
          sp.push([c[0] + sideV.x * Math.sin(th) * rx2 + upV.x * Math.cos(th) * ry2,
                   c[1] + sideV.y * Math.sin(th) * rx2 + upV.y * Math.cos(th) * ry2,
                   c[2] + sideV.z * Math.sin(th) * rx2 + upV.z * Math.cos(th) * ry2]);
        }
        const sr = chainRings(sp, sp.map(() => iF), 1, {
          upHint: [fwd.x, fwd.y, fwd.z], blend: 0.5,
          profile: (u, o) => { o[0] = 0.016; o[1] = 0.005; },
          pw: () => 3.2, surf: () => 1, flut: () => 0,
          col: (k, radial, ring, out) => {
            mixCol(PAL.strap, PAL.shoeTrim,
              smoothstep(0.44, 0.48, ring.U) * (1 - smoothstep(0.54, 0.58, ring.U)) * 0.7, out);
            return out;
          },
        });
        emitTube(gK, sr, R_TINY, { capStart: true, capEnd: true, kind: 'band', detail: dTrim });
      }
    }
  }

  // --------------------------------------------------------------- arms ------
  for (const s of [-1, 1]) {
    const iC = s < 0 ? B_CLAVL : B_CLAVR, iU = s < 0 ? B_UARML : B_UARMR,
          iL = s < 0 ? B_LARML : B_LARMR, iH = s < 0 ? B_HANDL : B_HANDR;

    // Deltoid cap. C6: this used to start at the clavicle at full deltoid radius
    // and intersect the torso as a visible boolean seam along the shoulder line.
    // It now starts small and deep inside the chest so the union fairs into a
    // fillet instead, and it carries the jersey colour at the join.
    {
      const deep = [
        lerp(P[B_SPINE3].x, P[iC].x, 0.30),
        lerp(P[B_SPINE3].y, P[iC].y, 0.30) + 0.010,
        lerp(P[B_SPINE3].z, P[iC].z, 0.30),
      ];
      const rings = chainRings([deep, v3(P[iC]), v3(P[iU])], [B_SPINE3, iC, iU], 2, {
        upHint: [0, 0, -1], blend: 0.5,
        profile: (u, o) => {
          const r = stops([[0, 0.046], [0.34, 0.068], [0.70, 0.077], [1, 0.079]], u);
          o[0] = r; o[1] = r * 0.94;
        },
        pw: () => 2.3, surf: () => 0, flut: () => 0.05,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          mixCol(PAL.jersey, PAL.jerseyDark, 0.45 - Math.sin(t) * 0.35, out);
          mixCol(out, PAL.jerseyAccent,
            clamp01(Math.sin(t) * 0.5 + 0.2) * smoothstep(0.42, 0.62, ring.U) * 0.55, out);
          return out;
        },
      });
      emitTube(gK, rings, R_ARM, { capStart: true, capEnd: false, kind: 'jersey', detail: dTrim });
    }

    const A_RX = [[0, 0.084], [0.12, 0.076], [0.34, 0.063], [0.53, 0.050],
                  [0.70, 0.053], [0.88, 0.041], [1, 0.033]];
    const rings = chainRings([v3(P[iU]), v3(P[iL]), v3(P[iH])], [iU, iL, iH], STEP, {
      upHint: [0, 0, -1],
      profile: (u, o) => { const r = stops(A_RX, u); o[0] = r; o[1] = r * 0.97; },
      pw: () => 2.15,
      surf: () => 0,
      flut: (u) => stops([[0, 0.30], [0.25, 0.50], [0.5, 0.34], [0.8, 0.12], [1, 0.03]], u),
      col: (k, radial, ring, out) => {
        const t = (k / radial) * Math.PI * 2;
        const u = ring.U;
        mixCol(PAL.jersey, PAL.jerseyDark, 0.45 - Math.sin(t) * 0.35, out);
        const outer = clamp01(-Math.cos(t) * s * 1.4);
        mixCol(out, PAL.jerseyAccent,
          outer * smoothstep(0.05, 0.20, u) * (1 - smoothstep(0.70, 0.90, u)) * 0.75, out);
        mixCol(out, PAL.jerseyDark, smoothstep(0.86, 1.0, u) * 0.8, out);
        return out;
      },
    });
    emitTube(gK, rings, R_ARM, {
      kind: 'jersey', detail: dSleeve,
      warp: (ri, k, radial, r, p) => {
        // C5 — fabric bunching in the elbow crook, which is where a viewer
        // looks for cloth behaviour on a bent arm.
        const U = r.U;
        const t = (k / radial) * Math.PI * 2;
        const crook = Math.pow(Math.max(0, 1 - Math.abs(U - 0.50) / 0.16), 1.5);
        const w = Math.sin(U * 46.0) * 0.0045 * crook * clamp01(0.55 + Math.sin(t) * 0.55);
        const cx = p[0] - r.c[0], cy = p[1] - r.c[1], cz = p[2] - r.c[2];
        const l = Math.sqrt(cx * cx + cy * cy + cz * cz) || 1;
        p[0] += cx * w / l; p[1] += cy * w / l; p[2] += cz * w / l;
      },
    });

    // Sleeve cuff at the end of the jersey sleeve (mid-forearm on a DH jersey).
    {
      const e = new THREE.Vector3().subVectors(P[iH], P[iL]).normalize();
      const c0 = along(v3(P[iL]), e, 0.108);
      const cb = [along(c0, e, -0.020), along(c0, e, -0.002),
                  along(c0, e, 0.010), along(c0, e, 0.018)];
      const cr = chainRings(cb, [iL, iL, iL, iL], 1, {
        upHint: [0, 0, -1], blend: 0.5,
        profile: (u, o) => {
          const k = stops([[0, 1.00], [0.34, 1.10], [0.72, 1.09], [1, 0.96]], u);
          o[0] = 0.050 * k; o[1] = 0.049 * k;
        },
        pw: () => 2.2, surf: () => 0, flut: () => 0.18,   // matches the sleeve at this U
        col: (k, radial, ring, out) => { mixCol(PAL.jerseyDark, PAL.jerseyInk, 0.45, out); return out; },
      });
      emitTube(gK, cr, R_ARM, { kind: 'hem', detail: dTrim });
    }

    buildHand(s, iH, iL);
  }

  /**
   * C3 — the hand. Round 3: "the bar passes through the glove volume and the
   * grip re-emerges outboard", because the glove was one ellipsoid of radius
   * 56 mm centred on the bar axis. It is now a palm sitting HAND_UP above the
   * axis with four fingers and a thumb swept as arcs of radius (BAR_R + r)
   * about that axis, so the bar is enclosed rather than intersected.
   */
  function buildHand(s, iH, iL) {
    const w = P[iH];
    const inb = new THREE.Vector3(-s * 0.100, -0.010, 0.006).normalize();
    const fwd = new THREE.Vector3(0, 0, -1);
    fwd.addScaledVector(inb, -fwd.dot(inb)).normalize();
    const upV = new THREE.Vector3().crossVectors(inb, fwd).normalize();
    if (upV.y < 0) upV.multiplyScalar(-1);

    // Point at (bar-axis offset a, polar angle phi, radius rc) about the bar.
    const arcPt = (a, phi, rc) => {
      const bx = w.x + inb.x * a - upV.x * HAND_UP;
      const by = w.y + inb.y * a - upV.y * HAND_UP;
      const bz = w.z + inb.z * a - upV.z * HAND_UP;
      const sp = Math.sin(phi), cp = Math.cos(phi);
      return [bx + fwd.x * sp * rc + upV.x * cp * rc,
              by + fwd.y * sp * rc + upV.y * cp * rc,
              bz + fwd.z * sp * rc + upV.z * cp * rc];
    };

    // --- palm: a flattened block lying along the bar, resting on top of it.
    {
      const pa = [-0.046, -0.020, 0.010, 0.038, 0.050];
      const pts = pa.map((a) => [w.x + inb.x * a - fwd.x * 0.004,
                                 w.y + inb.y * a - fwd.y * 0.004,
                                 w.z + inb.z * a - fwd.z * 0.004]);
      const rings = chainRings(pts, pts.map(() => iH), 1, {
        upHint: [upV.x, upV.y, upV.z], blend: 0.5,
        profile: (u, o) => {
          o[0] = stops([[0, 0.026], [0.22, 0.033], [0.62, 0.034], [0.88, 0.030], [1, 0.021]], u);
          o[1] = stops([[0, 0.019], [0.22, 0.024], [0.62, 0.024], [0.88, 0.021], [1, 0.015]], u);
        },
        pw: () => 2.9, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          mixCol(PAL.glove, PAL.gloveGrip, clamp01(-Math.sin(t) - 0.1) * 0.9, out);
          mixCol(out, PAL.jerseyAccent,
            smoothstep(0.30, 0.34, ring.U) * (1 - smoothstep(0.46, 0.50, ring.U)) *
            clamp01(Math.sin(t)) * 0.55, out);
          return out;
        },
      });
      emitTube(gK, rings, R_SMALL, {
        capStart: true, capEnd: true, kind: 'glove', detail: dGlovePalm,
        warp: (ri, k, radial, r, p) => {
          // Knuckle ridge across the back of the hand.
          const t = (k / radial) * Math.PI * 2;
          const backTop = clamp01(Math.cos(t) * 0.55 + Math.sin(t) * 0.75);
          const kn = Math.pow(Math.max(0, Math.sin(r.U * Math.PI * 4.6)), 6) * backTop;
          const cx = p[0] - r.c[0], cy = p[1] - r.c[1], cz = p[2] - r.c[2];
          const l = Math.sqrt(cx * cx + cy * cy + cz * cz) || 1;
          const amp = kn * 0.0035;
          p[0] += cx * amp / l; p[1] += cy * amp / l; p[2] += cz * amp / l;
        },
      });
    }

    // --- four fingers, wrapping from the front of the bar round underneath.
    const FOFF = [0.030, 0.010, -0.010, -0.030];
    const FRAD = [0.0098, 0.0100, 0.0094, 0.0084];
    const PHI = [0.95, 1.55, 2.15, 2.75, 3.35, 3.90];
    const RC = [0.0330, 0.0300, 0.0285, 0.0280, 0.0280, 0.0285];
    for (let f = 0; f < 4; f++) {
      const pts = PHI.map((phi, i) => arcPt(FOFF[f], phi, RC[i]));
      const r0 = FRAD[f];
      const rings = chainRings(pts, pts.map(() => iH), 1, {
        upHint: [inb.x, inb.y, inb.z], blend: 0.5,
        profile: (u, o) => {
          const r = r0 * stops([[0, 1.10], [0.20, 1.02], [0.55, 0.96], [0.85, 0.90], [1, 0.80]], u);
          o[0] = r; o[1] = r * 1.06;
        },
        pw: () => 2.4, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          mixCol(PAL.glove, PAL.gloveGrip, clamp01(Math.sin(t) * 0.5 + 0.2) * 0.55, out);
          return out;
        },
      });
      emitTube(gK, rings, R_TINY, { capStart: false, capEnd: true, kind: 'finger', detail: dGloveFinger });
    }

    // --- thumb, coming round the back of the bar to meet the fingertips.
    {
      const tp = [-0.35, -0.80, -1.25, -1.62, -1.95];
      const trc = [0.0340, 0.0325, 0.0310, 0.0300, 0.0298];
      const pts = tp.map((phi, i) => arcPt(0.043, phi, trc[i]));
      const rings = chainRings(pts, pts.map(() => iH), 1, {
        upHint: [inb.x, inb.y, inb.z], blend: 0.5,
        profile: (u, o) => {
          const r = stops([[0, 0.0130], [0.35, 0.0118], [0.72, 0.0104], [1, 0.0088]], u);
          o[0] = r; o[1] = r * 1.05;
        },
        pw: () => 2.4, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          mixCol(PAL.glove, PAL.gloveGrip, clamp01(Math.sin(t) * 0.5 + 0.2) * 0.55, out);
          return out;
        },
      });
      emitTube(gK, rings, R_TINY, { capStart: false, capEnd: true, kind: 'finger', detail: dGloveFinger });
    }

    // --- gauntlet cuff over the wrist, proud of the sleeve.
    {
      const e = new THREE.Vector3().subVectors(P[iL], P[iH]).normalize();
      const cb = [along(v3(w), e, 0.012), along(v3(w), e, 0.030),
                  along(v3(w), e, 0.050), along(v3(w), e, 0.062)];
      const cr = chainRings(cb, [iH, iH, iH, iH], 1, {
        upHint: [upV.x, upV.y, upV.z], blend: 0.5,
        profile: (u, o) => {
          const r = stops([[0, 0.036], [0.30, 0.042], [0.74, 0.041], [1, 0.033]], u);
          o[0] = r; o[1] = r * 0.94;
        },
        pw: () => 2.5, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          mixCol(PAL.gloveCuff, PAL.glove, smoothstep(0.55, 1.0, ring.U) * 0.8, out);
          return out;
        },
      });
      emitTube(gK, cr, R_SMALL, { capEnd: true, kind: 'band', detail: dTrim });
    }
  }

  // --------------------------------------------------------------- neck ------
  {
    const a = [P[B_SPINE3].x, P[B_SPINE3].y + 0.018, P[B_SPINE3].z - 0.009];
    const b = [P[B_HEAD].x, P[B_HEAD].y - 0.018, P[B_HEAD].z + 0.007];
    const rings = chainRings([a, v3(P[B_NECK]), b], [B_SPINE3, B_NECK, B_HEAD], 2, {
      upHint: [0, 0, -1], blend: 0.5,
      profile: (u, o) => { const r = 0.056 - u * 0.009; o[0] = r; o[1] = r * 1.05; },
      pw: () => 2.1, surf: () => 0, flut: () => 0,
      col: (k, radial, ring, out) => {
        const t = (k / radial) * Math.PI * 2;
        mixCol(PAL.skin, PAL.skinDark, 0.35 - Math.sin(t) * 0.3, out);
        return out;
      },
    });
    emitTube(gS, rings, R_SMALL, { kind: 'skin', detail: dNeck });

    // Jersey collar sitting on top of it, with a proud bound edge.
    const cr = chainRings(
      [[a[0], a[1] - 0.013, a[2]], [a[0], a[1] + 0.010, a[2] - 0.004],
       [a[0], a[1] + 0.030, a[2] - 0.007], [a[0], a[1] + 0.040, a[2] - 0.008]],
      [B_SPINE3, B_SPINE3, B_NECK, B_NECK], 1, {
        upHint: [0, 0, -1], blend: 0.5,
        profile: (u, o) => {
          const r = stops([[0, 0.078], [0.34, 0.073], [0.76, 0.070], [1, 0.075]], u);
          o[0] = r; o[1] = r * 1.04;
        },
        pw: () => 2.2, surf: () => 0, flut: (u) => u * 0.25,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          mixCol(PAL.jerseyDark, PAL.jersey, clamp01(Math.sin(t) * 0.5 + 0.5) * 0.6, out);
          mixCol(out, PAL.jerseyInk, smoothstep(0.80, 1.0, ring.U) * 0.7, out);
          return out;
        },
      });
    emitTube(gK, cr, R_TORSO, { kind: 'hem', detail: dTrim });
  }

  // --------------------------------------------------------------- head ------
  // C7 — head + helmet scaled to 0.94, which with the shortened spine puts the
  // crown 1.385 m above the pedal spindle (was 1.540 m).
  const HC = [P[B_HEAD].x, P[B_HEAD].y + 0.073, P[B_HEAD].z - 0.010];
  const SC = [HC[0], HC[1] + 0.013, HC[2] + 0.002];
  const SRX = 0.110, SRY = 0.127, SRZ = 0.136;   // helmet ellipsoid semi-axes
  const sFac = (dy) => (dy > 0
    ? Math.sqrt(Math.max(0, 1 - (dy / SRY) * (dy / SRY)))
    : 1 - 0.20 * (dy / 0.136) * (dy / 0.136));
  const shellPt = (dy, az, inflate, out) => {
    const f = sFac(dy) * (inflate || 1);
    out[0] = SC[0] + SRX * f * Math.sin(az);
    out[1] = SC[1] + dy;
    out[2] = SC[2] - SRZ * f * Math.cos(az);
    return out;
  };
  const PORT_AZ = 0.95, PORT_TOP = 0.064;   // eye-port half-angle and brow line

  {
    const pts = [
      [HC[0], HC[1] - 0.106, HC[2] - 0.005],
      [HC[0], HC[1] - 0.068, HC[2] - 0.013],
      [HC[0], HC[1] - 0.027, HC[2] - 0.011],
      [HC[0], HC[1] + 0.009, HC[2] + 0.000],
      [HC[0], HC[1] + 0.050, HC[2] + 0.005],
      [HC[0], HC[1] + 0.086, HC[2] + 0.011],
      [HC[0], HC[1] + 0.110, HC[2] + 0.014],
    ];
    const RX = [[0, 0.043], [0.18, 0.063], [0.36, 0.071], [0.54, 0.077], [0.72, 0.079],
                [0.90, 0.068], [1, 0.027]];
    const rings = chainRings(pts, pts.map(() => B_HEAD), 1, {
      upHint: [0, 0, -1], blend: 0.5,
      profile: (u, o) => { const r = stops(RX, u); o[0] = r; o[1] = r * 1.14; },
      pw: () => 2.2, surf: () => 0, flut: () => 0,
      col: (k, radial, ring, out) => {
        const t = (k / radial) * Math.PI * 2;
        const front = Math.sin(t);
        mixCol(PAL.skin, PAL.skinDark, 0.40 - front * 0.34, out);
        mixCol(out, PAL.skinDark, smoothstep(0.34, 0.10, ring.U) * clamp01(front) * 0.55, out);
        return out;
      },
    });
    emitTube(gS, rings, R_HEAD, {
      capStart: true, capEnd: true, kind: 'face', detail: dFace,
      warp: (ri, k, radial, r, p) => {
        // A nose and a brow ridge. The eye port is a real opening, so the head
        // is on screen whenever the rider is — it cannot be a bare ellipsoid.
        const t = (k / radial) * Math.PI * 2;
        const f = clamp01(Math.sin(t));
        const U = r.U;
        const nose = Math.pow(Math.max(0, 1 - Math.abs(U - 0.545) / 0.11), 1.4)
                   * Math.pow(f, 6) * 0.016;
        const brow = Math.pow(Math.max(0, 1 - Math.abs(U - 0.705) / 0.045), 1.2)
                   * Math.pow(f, 2) * 0.005;
        const chin = Math.pow(Math.max(0, 1 - Math.abs(U - 0.290) / 0.10), 1.5)
                   * Math.pow(f, 3) * 0.006;
        const cx = p[0] - r.c[0], cy = p[1] - r.c[1], cz = p[2] - r.c[2];
        const l = Math.sqrt(cx * cx + cy * cy + cz * cz) || 1;
        const amp = nose + brow + chin;
        p[0] += cx * amp / l; p[1] += cy * amp / l; p[2] += cz * amp / l;
      },
    });
  }

  // ------------------------------------------------------- helmet shell ------
  {
    const pts = [];
    const nR = lo ? 11 : 17;
    for (let i = 0; i < nR; i++) {
      const dy = lerp(0.122, -0.133, i / (nR - 1));
      pts.push([SC[0], SC[1] + dy, SC[2]]);
    }
    const rings = chainRings(pts, pts.map(() => B_HEAD), 1, {
      upHint: [0, 0, -1], blend: 0.5,
      profile: (u, o) => {
        const dy = lerp(0.122, -0.133, u);
        const f = sFac(dy);
        o[0] = SRX * f; o[1] = SRZ * f;
      },
      pw: () => 2.05, surf: () => 0, flut: () => 0,
      col: (k, radial, ring, out) => {
        const t = (k / radial) * Math.PI * 2;
        const front = Math.sin(t), lat = Math.abs(Math.cos(t));
        const dy = lerp(0.122, -0.133, ring.U);
        mixCol(PAL.helmet, PAL.helmetIn, clamp01(0.25 - front * 0.25), out);
        const stripe = (1 - smoothstep(0.16, 0.34, lat)) * smoothstep(-0.02, 0.05, dy);
        mixCol(out, PAL.helmetStripe, stripe * 0.9, out);
        mixCol(out, PAL.helmetPale, smoothstep(0.62, 0.94, lat) *
          smoothstep(-0.03, -0.10, dy) * 0.26, out);
        return out;
      },
    });
    emitTube(gH, rings, R_HELM, {
      capStart: true, capEnd: false, kind: 'helmet', detail: dHelmetShell,
      mask: (ri, k, x, y, z) => {
        const dy = y - SC[1];
        if (dy > PORT_TOP) return false;
        const az = Math.atan2((x - SC[0]) / SRX, -(z - SC[2]) / SRZ);
        return Math.abs(az) < PORT_AZ;
      },
    });

    // ---------------------------------------------------------------- C1 ----
    // THE HOLE THROUGH THE PROTAGONIST'S HEAD.
    //
    // Round 3 shipped `side: THREE.DoubleSide` on the shell with the comment
    // "the eye port is a real hole; you see the liner" — but there was no liner.
    // What the chase camera actually saw through the port was the un-normalled
    // interior of the far side of the shell: the brightest element on the
    // character, in every gameplay frame. Three reviewers filed it as a blank
    // face with atlas bleed at the crown.
    //
    // The shell is now FrontSide (see buildMaterials) and this is the liner:
    // a reverse-wound EPS shell whose normals face INWARD, so the far side of it
    // is the front-facing surface you see through the port, and the near side is
    // correctly back-face culled. Verify on a full 360 turntable, not from the
    // front — from behind you must still see an opaque shell.
    {
      const lp = [];
      const nL = lo ? 9 : 13;
      for (let i = 0; i < nL; i++) {
        const dy = lerp(0.116, -0.146, i / (nL - 1));
        lp.push([SC[0], SC[1] + dy, SC[2]]);
      }
      const lr = chainRings(lp, lp.map(() => B_HEAD), 1, {
        upHint: [0, 0, -1], blend: 0.5,
        profile: (u, o) => {
          const dy = lerp(0.116, -0.146, u);
          const f = sFac(dy) * 0.900;
          o[0] = SRX * f; o[1] = SRZ * f;
        },
        pw: () => 2.05, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          mixCol(PAL.liner, PAL.linerPad, clamp01(0.35 + Math.sin(t) * 0.25), out);
          return out;
        },
      });
      emitTube(gH, lr, lo ? 12 : 20, {
        reverse: true, capStart: true, capEnd: false, kind: 'liner', detail: dLiner,
      });

      // Cheek pads, which are what you actually see either side of the face.
      for (const cs of [-1, 1]) {
        const pp = [];
        for (let i = 0; i <= 3; i++) {
          const dy = lerp(-0.020, -0.108, i / 3);
          const o3 = [0, 0, 0];
          shellPt(dy, cs * (PORT_AZ - 0.13), 0.845, o3);
          pp.push(o3);
        }
        const pr = chainRings(pp, pp.map(() => B_HEAD), 1, {
          upHint: [0, 0, -1], blend: 0.5,
          profile: (u, o) => {
            const r = stops([[0, 0.014], [0.45, 0.021], [1, 0.013]], u);
            o[0] = r; o[1] = r * 1.5;
          },
          pw: () => 2.6, surf: () => 1, flut: () => 0,
          col: PAL.linerPad,
        });
        emitTube(gH, pr, R_TINY, { capStart: true, capEnd: true, kind: 'liner', detail: dLiner });
      }
    }

    // Port lip — real helmets have a visible 8 mm edge round the opening, and it
    // is what stops the shell reading as paper-thin from the chase camera.
    {
      const path = [];
      const N1 = lo ? 5 : 8, N2 = lo ? 7 : 12;
      for (let i = 0; i <= N1; i++) path.push([-0.133 + (PORT_TOP + 0.133) * (i / N1), -PORT_AZ]);
      for (let i = 1; i <= N2; i++) path.push([PORT_TOP, -PORT_AZ + 2 * PORT_AZ * (i / N2)]);
      for (let i = 1; i <= N1; i++) path.push([PORT_TOP - (PORT_TOP + 0.133) * (i / N1), PORT_AZ]);
      const pts = path.map((q) => { const o = [0, 0, 0]; shellPt(q[0], q[1], 1.0, o); return o; });
      for (let pass = 0; pass < 2; pass++) {
        const src = pts.map((p) => p.slice());
        for (let i = 1; i < pts.length - 1; i++) {
          for (let c = 0; c < 3; c++) pts[i][c] = (src[i - 1][c] + src[i][c] * 2 + src[i + 1][c]) * 0.25;
        }
      }
      const rings = chainRings(pts, pts.map(() => B_HEAD), 1, {
        upHint: [0, 1, 0], blend: 0.5,
        profile: (u, o) => { o[0] = 0.012; o[1] = 0.014; },
        pw: () => 2.4, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          mixCol(PAL.helmetIn, PAL.helmet, clamp01(Math.sin(t)) * 0.55, out);
          return out;
        },
      });
      emitTube(gH, rings, lo ? 5 : 8, { capStart: true, capEnd: true, kind: 'liner', detail: dLiner });
    }

    // Chin bar: a quadratic sweep from cheek to cheek across the mouth.
    {
      const a = shellPt(-0.094, -PORT_AZ, 1.0, [0, 0, 0]);
      const b = shellPt(-0.094, PORT_AZ, 1.0, [0, 0, 0]);
      const cP = [SC[0], SC[1] - 0.141, SC[2] - SRZ * 1.30];
      const pts = [];
      const N = lo ? 9 : 15;
      for (let i = 0; i <= N; i++) {
        const t = i / N, it = 1 - t;
        pts.push([
          it * it * a[0] + 2 * it * t * cP[0] + t * t * b[0],
          it * it * a[1] + 2 * it * t * cP[1] + t * t * b[1],
          it * it * a[2] + 2 * it * t * cP[2] + t * t * b[2],
        ]);
      }
      const rings = chainRings(pts, pts.map(() => B_HEAD), 1, {
        upHint: [0, 1, 0], blend: 0.5,
        profile: (u, o) => {
          const bulge = 1 + 0.22 * Math.sin(Math.PI * u);
          o[0] = 0.028 * bulge; o[1] = 0.034 * bulge;
        },
        pw: () => 2.7, surf: () => 0, flut: () => 0,
        col: (k, radial, ring, out) => {
          const t = (k / radial) * Math.PI * 2;
          mixCol(PAL.helmet, PAL.helmetIn, clamp01(-Math.sin(t)) * 0.75, out);
          const vent = Math.pow(Math.abs(Math.sin(ring.U * Math.PI * 7)), 8);
          mixCol(out, PAL.helmetIn, vent * clamp01(0.6 - Math.sin(t)) * 0.9, out);
          mixCol(out, PAL.helmetStripe,
            smoothstep(0.40, 0.46, ring.U) * (1 - smoothstep(0.54, 0.60, ring.U)) * 0.55, out);
          return out;
        },
      });
      emitTube(gH, rings, lo ? 8 : 12, {
        capStart: true, capEnd: true, kind: 'helmet', detail: dHelmetShell,
      });
    }

    // Peak / visor, skinned to its own bone so it can wobble on a spring.
    // C9 — real thickness that tapers from 6 mm at the mount to 2.5 mm at the
    // tip, so the rim reads as a rolled edge rather than a zero-thickness plate.
    {
      const nu = lo ? 11 : 15, nv = 5;
      const mount = [0, 0, 0];
      emitSheet(gH, nu, nv, (i, j, out) => {
        const u = (i / (nu - 1)) * 2 - 1;
        const v = j / (nv - 1);
        const az = u * 1.02;
        shellPt(0.045, az, 1.015, mount);
        const ext = 0.122 * (1 - 0.34 * u * u);
        const fx = Math.sin(az) * 0.30, fz = -Math.cos(az);
        out[0] = mount[0] + fx * ext * v;
        out[1] = mount[1] + ext * v * 0.30 - (ext * v) * (ext * v) * 3.2 - u * u * 0.011;
        out[2] = mount[2] + fz * ext * v;
      }, {
        thickness: (u, v) => 0.0060 - 0.0035 * smoothstep(0.35, 1.0, v)
          - 0.0018 * smoothstep(0.72, 1.0, Math.abs(u * 2 - 1)),
        wrapU: false, uRep: 2.4, vRep: 1.2,
        surf: 0, flut: 0, b0: B_PEAK, w0: 1, b1: B_PEAK,
        tint: 1, tintBack: 0.42,
        kind: 'helmet',
        detail: (u, v, o) => {
          // Underside gets the moulded rib pattern; topside a graphic edge.
          const rib = 0.5 + 0.5 * Math.cos(u * Math.PI * 2 * 9);
          o[3] *= 1 - Math.pow(rib, 4) * 0.10;
          mixInto(o, PAL.helmetStripe, smoothstep(0.80, 0.94, v) * 0.75);
          mixInto(o, PAL.helmetIn, smoothstep(0.955, 1.0, v) * 0.85);
          heather(o, u, v, 0.035);
        },
        col: (u, v, layer, out) => {
          mixCol(PAL.helmet, PAL.helmetIn, layer === 1 ? 0.55 : 0, out);
          return out;
        },
      });
    }

    // Chin strap + buckle. The strap is what makes a helmet look WORN rather
    // than dropped on, and it is six triangles' worth of geometry.
    for (const cs of [-1, 1]) {
      const top = shellPt(-0.078, cs * 1.48, 1.005, [0, 0, 0]);
      const mid = [SC[0] + cs * 0.052, SC[1] - 0.120, SC[2] - 0.040];
      const chin = [SC[0] + cs * 0.012, SC[1] - 0.148, SC[2] - 0.062];
      const pts = [top, mid, chin];
      const rings = chainRings(pts, pts.map(() => B_HEAD), 2, {
        upHint: [0, 1, 0], blend: 0.5,
        profile: (u, o) => { o[0] = 0.004; o[1] = 0.013; },
        pw: () => 3.0, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          mixCol(PAL.strap, PAL.strapBand,
            smoothstep(0.60, 0.66, ring.U) * (1 - smoothstep(0.72, 0.78, ring.U)) * 0.7, out);
          return out;
        },
      });
      emitTube(gK, rings, R_TINY, { capStart: true, capEnd: true, kind: 'band', detail: dTrim });
    }
  }

  // ------------------------------------------------------------ goggles ------
  {
    const GC = [SC[0], SC[1] + 0.011, SC[2] - SRZ * 0.905];
    const outline = (th, out) => {
      const ox = 0.095 * Math.cos(th) * (1 + 0.05 * Math.cos(2 * th));
      const oy = 0.047 * Math.sin(th) + 0.007 * Math.cos(2 * th) - 0.005;
      const oz = 0.034 * (ox / 0.095) * (ox / 0.095);
      out[0] = GC[0] + ox; out[1] = GC[1] + oy; out[2] = GC[2] + oz;
      return out;
    };
    const NF = lo ? 16 : 26;
    const fpts = [];
    for (let i = 0; i < NF; i++) fpts.push(outline((i / NF) * Math.PI * 2, [0, 0, 0]));

    // Frame
    {
      const rings = chainRings(fpts.concat([fpts[0]]), fpts.map(() => B_HEAD).concat([B_HEAD]), 1, {
        upHint: [0, 0, -1], blend: 0.5,
        profile: (u, o) => { o[0] = 0.013; o[1] = 0.016; },
        pw: () => 2.5, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          mixCol(PAL.goggleFrame, PAL.strapBand,
            smoothstep(0.20, 0.26, ring.U) * (1 - smoothstep(0.30, 0.36, ring.U)) * 0.8, out);
          return out;
        },
      });
      emitTube(gK, rings, lo ? 5 : 7, { kind: 'band', detail: dTrim });
    }

    // Lens — a curved shell that bulges forward at the bridge. No atlas: it is a
    // mirror, and a mirror with baked AO on it reads as a dirty lens.
    {
      const nu = lo ? 16 : 26, nv = 4;
      const o3 = [0, 0, 0];
      emitSheet(gL, nu, nv, (i, j, out) => {
        const th = (i / nu) * Math.PI * 2;
        const s2 = j / (nv - 1);
        outline(th, o3);
        out[0] = GC[0] + (o3[0] - GC[0]) * s2 * 0.95;
        out[1] = GC[1] + (o3[1] - GC[1]) * s2 * 0.95;
        out[2] = GC[2] + (o3[2] - GC[2]) * s2 * s2 - 0.019 * (1 - s2 * s2);
      }, {
        thickness: 0.0032, wrapU: true, uRep: 1, vRep: 1,
        surf: 0, flut: 0, b0: B_HEAD, w0: 1, b1: B_HEAD,
        chart: false,
        col: () => [1, 1, 1],
      });
    }

    // Strap: leaves the frame at both edges and wraps the back of the shell.
    {
      const spts = [];
      const eL = outline(Math.PI, [0, 0, 0]), eR = outline(0, [0, 0, 0]);
      spts.push([eR[0] + 0.010, eR[1], eR[2] + 0.012]);
      const NS = lo ? 12 : 20;
      for (let i = 0; i <= NS; i++) {
        const az = lerp(1.16, Math.PI * 2 - 1.16, i / NS);
        spts.push(shellPt(0.009 + Math.sin(i / NS * Math.PI) * 0.010, az, 1.030, [0, 0, 0]));
      }
      spts.push([eL[0] - 0.010, eL[1], eL[2] + 0.012]);
      const rings = chainRings(spts, spts.map(() => B_HEAD), 1, {
        upHint: [0, 1, 0], blend: 0.5,
        profile: (u, o) => { o[0] = 0.006; o[1] = 0.024; },
        pw: () => 3.0, surf: () => 1, flut: () => 0,
        col: (k, radial, ring, out) => {
          const band2 = Math.pow(Math.abs(Math.sin(ring.U * Math.PI * 5.5)), 12);
          mixCol(PAL.strap, PAL.strapBand, band2 * 0.85, out);
          return out;
        },
      });
      emitTube(gK, rings, lo ? 5 : 7, { capStart: true, capEnd: true, kind: 'band', detail: dTrim });
    }
  }

  return finishGeometry(G);
}

// ===========================================================================
// 7. Materials
// ===========================================================================

function buildMaterials(tex, ctx) {
  const windU = {
    uTime: { value: 0 },
    uWindAmp: { value: 0.008 },
    uGust: { value: 0 },
  };

  // C8 — four MeshPhysicalMaterial programs collapse to one physical + three
  // standard. Sheen at 0.15 on skin and clearcoat at 0.35 on the helmet are not
  // resolvable at chase distance, and `lens` iridescence is the most expensive
  // branch in three's physical shader, paid for a ~600 px mirror. The physical
  // variants are rebuilt on demand by setPhotoMode().
  const skin = new THREE.MeshStandardMaterial({
    color: 0xffffff, vertexColors: true,
    roughness: 0.56, metalness: 0.0,
    map: tex.atlas,
    normalMap: tex.skin, normalScale: new THREE.Vector2(0.40, 0.40),
  });
  skin.name = 'riderSkin';

  // --- kit (jersey / shorts / gloves / pads / shoes / straps) ----------------
  // One material, two surface behaviours: aSurf = 0 is woven fabric with sheen,
  // aSurf = 1 is moulded rubber. Both normals live in one packed texture so the
  // whole kit still costs a single sampler and a single draw call.
  const kit = new THREE.MeshPhysicalMaterial({
    color: 0xffffff, vertexColors: true,
    roughness: 0.88, metalness: 0.0,
    map: tex.atlas,
    // C5 — normalScale down from 1.0 now that the map's dominant term is a fold
    // field rather than a sub-pixel weave; 1.0 on the old map was the moiré.
    normalMap: tex.kit, normalScale: new THREE.Vector2(0.62, 0.62),
    sheen: 0.55, sheenRoughness: 0.72, sheenColor: new THREE.Color(0xbfc9cc),
  });
  kit.name = 'riderKit';
  kit.onBeforeCompile = (sh) => {
    sh.uniforms.uTime = windU.uTime;
    sh.uniforms.uWindAmp = windU.uWindAmp;
    sh.uniforms.uGust = windU.uGust;
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', `#include <common>
        attribute float aSurf;
        attribute float aFlutter;
        varying float vSurf;
        uniform float uTime;
        uniform float uWindAmp;
        uniform float uGust;`)
      .replace('#include <begin_vertex>', `#include <begin_vertex>
        vSurf = aSurf;`)
      // Applied AFTER skinning so the flutter is in object space (== the bike
      // frame), which is exactly where the apparent wind comes from: straight
      // back along +Z at the rider's speed.
      .replace('#include <skinning_vertex>', `#include <skinning_vertex>
        {
          float fw = aFlutter;
          if (fw > 0.002) {
            float ph  = uTime * (12.0 + fw * 7.0) + transformed.y * 9.5
                      + transformed.x * 6.5 + transformed.z * 4.5;
            float ph2 = uTime * 7.3 + transformed.y * 5.0 - transformed.z * 6.5;
            vec3 wob = vec3(sin(ph) * 0.55 + sin(ph2 * 1.7) * 0.22,
                            sin(ph * 1.31 + 1.7) * 0.42,
                            cos(ph * 0.87 + 0.4) * 0.80 + cos(ph2) * 0.20);
            vec3 back = vec3(0.0, 0.10, 1.0);
            float g = 0.7 + 0.3 * uGust;
            transformed += (back * (1.35 * g) + wob) * (fw * fw) * uWindAmp;
          }
        }`);
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', '#include <common>\nvarying float vSurf;')
      .replace('#include <normal_fragment_maps>', `
        vec4 nPack = texture2D( normalMap, vNormalMapUv );
        vec2 nFab = nPack.rg * 2.0 - 1.0;
        vec2 nRub = nPack.ba * 2.0 - 1.0;
        vec2 nXY = mix( nFab, nRub * 1.30, vSurf ) * normalScale;
        vec3 mapN = vec3( nXY, sqrt( max( 1.0e-4, 1.0 - dot( nXY, nXY ) ) ) );
        normal = normalize( tbn * mapN );`)
      .replace('#include <lights_physical_fragment>', `#include <lights_physical_fragment>
        material.sheenColor *= ( 1.0 - vSurf );
        material.roughness = clamp( material.roughness * mix( 1.0, 0.66, vSurf ), 0.04, 1.0 );`);
  };
  // Force a distinct program key from any other MeshPhysicalMaterial in the scene.
  kit.customProgramCacheKey = () => 'riderKit';

  // --- helmet shell ---------------------------------------------------------
  // C1 — FrontSide. `DoubleSide` here is what produced "a hole through the
  // protagonist's head": the eye port is a genuine opening in the shell, so a
  // double-sided shell renders its own interior straight down the barrel of the
  // chase camera. The interior is now the reverse-wound liner built in
  // buildBody(), which is what a viewer should see through the port.
  const helmet = new THREE.MeshStandardMaterial({
    color: 0xffffff, vertexColors: true,
    roughness: 0.40, metalness: 0.0,
    map: tex.atlas,
    envMapIntensity: 1.15,
    normalMap: tex.helmet, normalScale: new THREE.Vector2(0.42, 0.42),
    side: THREE.FrontSide,
  });
  helmet.name = 'riderHelmet';

  // --- goggle lens ----------------------------------------------------------
  // A real mirrored DH lens, lit entirely by scene.environment (owned by sky).
  // Kept opaque so it stays out of the transparent pass — you cannot see through
  // a mirror lens anyway. C8: metal + low roughness, no iridescence branch.
  const lens = new THREE.MeshStandardMaterial({
    color: new THREE.Color(0x5e3f21),
    metalness: 1.0, roughness: 0.070,
    envMapIntensity: 1.9,
    side: THREE.FrontSide,
  });
  lens.name = 'riderLens';

  // The atlas lives on UV channel 0; the tiling weave/rubber map on channel 1.
  for (const m of [skin, kit, helmet]) {
    if (m.normalMap) m.normalMap.channel = 1;
    if (m.map) m.map.channel = 0;
  }

  return { mats: [skin, kit, helmet, lens], windU };
}

/**
 * Photo-mode material set (C8 keeps this available). Built lazily the first
 * time it is asked for, so gameplay never pays for the extra shader programs.
 */
function buildPhotoMaterials(tex, gameplay) {
  const skin = new THREE.MeshPhysicalMaterial({
    color: 0xffffff, vertexColors: true,
    roughness: 0.58, metalness: 0.0, map: tex.atlas,
    normalMap: tex.skin, normalScale: new THREE.Vector2(0.55, 0.55),
    sheen: 0.15, sheenRoughness: 0.5, sheenColor: new THREE.Color(0xffd9c0),
  });
  const helmet = new THREE.MeshPhysicalMaterial({
    color: 0xffffff, vertexColors: true,
    roughness: 0.40, metalness: 0.0, map: tex.atlas,
    clearcoat: 0.55, clearcoatRoughness: 0.22,
    normalMap: tex.helmet, normalScale: new THREE.Vector2(0.42, 0.42),
    side: THREE.FrontSide,
  });
  const lens = new THREE.MeshPhysicalMaterial({
    color: new THREE.Color(0x5e3f21),
    metalness: 1.0, roughness: 0.070, envMapIntensity: 1.9,
    iridescence: 0.7, iridescenceIOR: 1.6,
    iridescenceThicknessRange: [180, 620],
    side: THREE.FrontSide,
  });
  skin.name = 'riderSkinPhoto'; helmet.name = 'riderHelmetPhoto'; lens.name = 'riderLensPhoto';
  for (const m of [skin, helmet]) {
    if (m.normalMap) m.normalMap.channel = 1;
    if (m.map) m.map.channel = 0;
  }
  // The kit material is shared with the gameplay set — it is already the one
  // MeshPhysicalMaterial we keep, so there is nothing to upgrade.
  return [skin, gameplay[1], helmet, lens];
}

// ===========================================================================
// 8. Ragdoll
//
// 16 Verlet particles + distance constraints. It is deliberately not a full
// rigid-body solver: what sells a bike crash is the limbs trailing and the body
// tumbling and sliding, and a constraint net does that with no allocation and no
// chance of exploding into the sky mid-run.
// ===========================================================================

const P_PELVIS = 0, P_SPINEMID = 1, P_CHEST = 2, P_HEAD = 3,
      P_SHLDL = 4, P_ELBL = 5, P_HANDL = 6,
      P_SHLDR = 7, P_ELBR = 8, P_HANDR = 9,
      P_HIPL = 10, P_KNEEL = 11, P_FOOTL = 12,
      P_HIPR = 13, P_KNEER = 14, P_FOOTR = 15;
const NPTS = 16;

const RD_SEED = [
  B_PELVIS, B_SPINE2, B_SPINE3, B_HEAD,
  B_UARML, B_LARML, B_HANDL,
  B_UARMR, B_LARMR, B_HANDR,
  B_THIGHL, B_SHINL, B_FOOTL,
  B_THIGHR, B_SHINR, B_FOOTR,
];
// C7 — scaled with the body (~0.90).
const RD_RADIUS = [0.117, 0.117, 0.126, 0.128, 0.080, 0.062, 0.054,
                   0.080, 0.062, 0.054, 0.090, 0.072, 0.058, 0.090, 0.072, 0.058];

// [a, b, stiffness, mode] — mode 0 exact, 1 max-only (a limit, not a rod).
const RD_LINKS = [
  [P_PELVIS, P_SPINEMID, 1.0, 0], [P_SPINEMID, P_CHEST, 1.0, 0],
  [P_CHEST, P_HEAD, 1.0, 0],
  [P_CHEST, P_SHLDL, 1.0, 0], [P_CHEST, P_SHLDR, 1.0, 0],
  [P_SHLDL, P_ELBL, 1.0, 0], [P_ELBL, P_HANDL, 1.0, 0],
  [P_SHLDR, P_ELBR, 1.0, 0], [P_ELBR, P_HANDR, 1.0, 0],
  [P_PELVIS, P_HIPL, 1.0, 0], [P_PELVIS, P_HIPR, 1.0, 0],
  [P_HIPL, P_KNEEL, 1.0, 0], [P_KNEEL, P_FOOTL, 1.0, 0],
  [P_HIPR, P_KNEER, 1.0, 0], [P_KNEER, P_FOOTR, 1.0, 0],
  // Bracing — keeps the torso a body and not a chain of beads.
  [P_PELVIS, P_CHEST, 0.75, 0], [P_SHLDL, P_SHLDR, 0.9, 0],
  [P_HIPL, P_HIPR, 0.9, 0], [P_PELVIS, P_SHLDL, 0.5, 0],
  [P_PELVIS, P_SHLDR, 0.5, 0], [P_CHEST, P_HIPL, 0.5, 0],
  [P_CHEST, P_HIPR, 0.5, 0], [P_HEAD, P_SHLDL, 0.4, 0], [P_HEAD, P_SHLDR, 0.4, 0],
  [P_SPINEMID, P_SHLDL, 0.35, 0], [P_SPINEMID, P_SHLDR, 0.35, 0],
  // Joint limits: stop elbows and knees hyper-extending into a straight line.
  [P_SHLDL, P_HANDL, 0.8, 1], [P_SHLDR, P_HANDR, 0.8, 1],
  [P_HIPL, P_FOOTL, 0.8, 1], [P_HIPR, P_FOOTR, 0.8, 1],
];

// bone, dirFrom, dirTo, refA, refB, refC (-1 => ref is refA−refB, else
// refA − midpoint(refB, refC), i.e. the bend direction of a two-bone limb).
const RD_ORIENT = [
  [B_PELVIS, P_PELVIS, P_SPINEMID, P_HIPR, P_HIPL, -1],
  [B_SPINE1, P_PELVIS, P_SPINEMID, P_HIPR, P_HIPL, -1],
  [B_SPINE2, P_SPINEMID, P_CHEST, P_SHLDR, P_SHLDL, -1],
  [B_SPINE3, P_SPINEMID, P_CHEST, P_SHLDR, P_SHLDL, -1],
  [B_NECK, P_CHEST, P_HEAD, P_SHLDR, P_SHLDL, -1],
  [B_UARML, P_SHLDL, P_ELBL, P_ELBL, P_SHLDL, P_HANDL],
  [B_LARML, P_ELBL, P_HANDL, P_ELBL, P_SHLDL, P_HANDL],
  [B_UARMR, P_SHLDR, P_ELBR, P_ELBR, P_SHLDR, P_HANDR],
  [B_LARMR, P_ELBR, P_HANDR, P_ELBR, P_SHLDR, P_HANDR],
  [B_THIGHL, P_HIPL, P_KNEEL, P_KNEEL, P_HIPL, P_FOOTL],
  [B_SHINL, P_KNEEL, P_FOOTL, P_KNEEL, P_HIPL, P_FOOTL],
  [B_THIGHR, P_HIPR, P_KNEER, P_KNEER, P_HIPR, P_FOOTR],
  [B_SHINR, P_KNEER, P_FOOTR, P_KNEER, P_HIPR, P_FOOTR],
];

function makeRagdoll(rig) {
  const pos = new Float32Array(NPTS * 3);
  const prev = new Float32Array(NPTS * 3);
  const gy = new Float32Array(NPTS);
  const gn = new Float32Array(NPTS * 3);
  const links = RD_LINKS.map((l) => {
    const a = rig.restPos[RD_SEED[l[0]]], b = rig.restPos[RD_SEED[l[1]]];
    return { a: l[0], b: l[1], k: l[2], mode: l[3], rest: a.distanceTo(b) };
  });
  const worldQ = [];
  for (let i = 0; i < NBONES; i++) worldQ.push(new THREE.Quaternion());
  const hit = { height: 0, normal: new THREE.Vector3(0, 1, 0), material: 0, slope: 0 };
  // Which bones get an explicit orientation from the particles (the rest inherit).
  const oriented = new Uint8Array(NBONES);
  oriented[B_HEAD] = 1;
  for (const E of RD_ORIENT) oriented[E[0]] = 1;
  const gp = (i, out) => out.set(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);

  return {
    pos, prev, active: false,
    /** Seed from the current driven pose (world space) plus the bike's motion. */
    seed(rig2, groupPos, groupQuat, vel, angVel, rand) {
      for (let i = 0; i < NPTS; i++) {
        const w = _v0.copy(rig2.worldPos[RD_SEED[i]]).applyQuaternion(groupQuat).add(groupPos);
        pos[i * 3] = w.x; pos[i * 3 + 1] = w.y; pos[i * 3 + 2] = w.z;
        // v = v_cm + ω × r, plus a little scatter so both sides do not mirror.
        _v1.copy(w).sub(groupPos);
        _v2.crossVectors(angVel, _v1).add(vel);
        _v2.x += (rand() - 0.5) * 1.4;
        _v2.y += rand() * 1.1 + 0.5;
        _v2.z += (rand() - 0.5) * 1.4;
        prev[i * 3] = w.x - _v2.x * (1 / 60);
        prev[i * 3 + 1] = w.y - _v2.y * (1 / 60);
        prev[i * 3 + 2] = w.z - _v2.z * (1 / 60);
        gy[i] = -1e9;
      }
      this.active = true;
    },

    step(dt, ctx) {
      const h = Math.min(dt, 1 / 45) * 0.5;      // two substeps
      const g = -9.81;
      // Ground sample once per frame per point; the terrain is not moving.
      const coll = ctx.collision;
      for (let i = 0; i < NPTS; i++) {
        const x = pos[i * 3], z = pos[i * 3 + 2];
        let hgt = -1e9, nx = 0, ny = 1, nz = 0;
        if (coll && coll.probe) {
          coll.probe(x, z, hit);
          hgt = hit.height; nx = hit.normal.x; ny = hit.normal.y; nz = hit.normal.z;
        } else if (ctx.terrain && ctx.terrain.sampleHeight) {
          hgt = ctx.terrain.sampleHeight(x, z);
        }
        gy[i] = hgt; gn[i * 3] = nx; gn[i * 3 + 1] = ny; gn[i * 3 + 2] = nz;
      }

      for (let sub = 0; sub < 2; sub++) {
        for (let i = 0; i < NPTS; i++) {
          const p = i * 3;
          for (let c = 0; c < 3; c++) {
            const cur = pos[p + c];
            let v = (cur - prev[p + c]) * 0.994;
            if (v > 1.2) v = 1.2; else if (v < -1.2) v = -1.2;   // anti-explosion
            prev[p + c] = cur;
            pos[p + c] = cur + v + (c === 1 ? g * h * h : 0);
          }
        }
        for (let it = 0; it < 7; it++) {
          for (let li = 0; li < links.length; li++) {
            const L = links[li];
            const a = L.a * 3, b = L.b * 3;
            let dx = pos[b] - pos[a], dy = pos[b + 1] - pos[a + 1], dz = pos[b + 2] - pos[a + 2];
            const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (d < 1e-6) continue;
            if (L.mode === 1 && d <= L.rest) continue;
            const diff = ((d - L.rest) / d) * 0.5 * L.k;
            dx *= diff; dy *= diff; dz *= diff;
            pos[a] += dx; pos[a + 1] += dy; pos[a + 2] += dz;
            pos[b] -= dx; pos[b + 1] -= dy; pos[b + 2] -= dz;
          }
          // Ground: push out along the surface normal and scrub tangential speed.
          for (let i = 0; i < NPTS; i++) {
            const p = i * 3;
            const floor = gy[i] + RD_RADIUS[i];
            if (pos[p + 1] < floor) {
              const pen = floor - pos[p + 1];
              pos[p] += gn[p] * pen * 0.35;
              pos[p + 1] = floor;
              pos[p + 2] += gn[p + 2] * pen * 0.35;
              // Friction + a little bounce loss, applied through the Verlet history.
              const vx = pos[p] - prev[p], vy = pos[p + 1] - prev[p + 1], vz = pos[p + 2] - prev[p + 2];
              prev[p] = pos[p] - vx * 0.45;
              prev[p + 1] = pos[p + 1] - vy * 0.15;
              prev[p + 2] = pos[p + 2] - vz * 0.45;
            }
          }
        }
      }
    },

    /** Retarget the particle cloud onto bone rotations, in group-local space. */
    pose(rigRef, invGroupQuat, groupPos, outLocal) {
      for (let i = 0; i < NBONES; i++) worldQ[i].identity();

      for (let e = 0; e < RD_ORIENT.length; e++) {
        const E = RD_ORIENT[e];
        gp(E[1], _v0); gp(E[2], _v1);
        _v2.copy(_v1).sub(_v0).applyQuaternion(invGroupQuat);
        gp(E[3], _v3);
        if (E[5] < 0) { gp(E[4], _v4); _v5.copy(_v3).sub(_v4); }
        else { gp(E[4], _v4); gp(E[5], _v9); _v5.copy(_v3).sub(_v4.add(_v9).multiplyScalar(0.5)); }
        _v5.applyQuaternion(invGroupQuat);
        orientBone(rigRef, E[0], _v2, _v5, worldQ[E[0]]);
      }
      // Head: face along the body's forward, level with the shoulder line.
      gp(P_CHEST, _v0); gp(P_HEAD, _v1); gp(P_SHLDL, _v3); gp(P_SHLDR, _v4);
      _v2.copy(_v1).sub(_v0).applyQuaternion(invGroupQuat).normalize();   // up
      _v5.copy(_v4).sub(_v3).applyQuaternion(invGroupQuat).normalize();   // right
      _v6.crossVectors(_v2, _v5);                                          // forward
      orientBoneFwdUp(_v6, _v2, worldQ[B_HEAD]);

      // Bones with no particle of their own simply inherit their parent.
      for (let i = 0; i < NBONES; i++) {
        const p = rigRef.parent[i];
        if (!oriented[i] && p >= 0) worldQ[i].copy(worldQ[p]);
        if (p < 0) outLocal[i].copy(worldQ[i]);
        else { _q0.copy(worldQ[p]).invert(); outLocal[i].multiplyQuaternions(_q0, worldQ[i]); }
      }
      // Root translation.
      gp(P_PELVIS, _v0);
      return _v0.sub(groupPos).applyQuaternion(invGroupQuat);
    },
  };
}

// ===========================================================================
// 9. createRider
// ===========================================================================

const SPINE_BONES = [B_SPINE1, B_SPINE2, B_SPINE3];
const SPINE_SHARE = [0.30, 0.34, 0.36];

const EMPTY_INPUT = {
  steer: 0, pitch: 0, roll: 0, brakeFront: 0, brakeRear: 0,
  pedal: 0, pump: 0, manual: false,
};

export function createRider(ctx) {
  const rng = makeRng(subSeed(ctx.seed || 1, 'rider'));
  const caps = ctx.renderer && ctx.renderer.capabilities;
  const aniso = caps && caps.getMaxAnisotropy ? Math.min(8, caps.getMaxAnisotropy()) : 4;

  const tex = {
    kit: makeKitNormal(rng, aniso),
    skin: makeSkinNormal(rng, aniso),
    helmet: makeHelmetNormal(rng, aniso),
    atlas: null,
  };
  const rig = buildRig();
  const built = buildBody(rig, ctx.quality);
  const geo = built.geo;

  // C2 + C6 — bake the garment atlas. Order matters: AO needs the final welded
  // normals, and the atlas needs the AO, so both run after finishGeometry().
  const lowTier = ctx.quality === 'low' || ctx.quality === 'medium';
  const ATLAS_SIZE = lowTier ? 1024 : 2048;
  // 1.3 mm/texel is ~2x oversampled against the 2.9 mm/px the rider occupies at
  // 3 m on a 1080p frame, which is where the reviewer's crops were taken.
  const TEXEL_MIN = lowTier ? 0.0026 : 0.0013;
  {
    const lensRange = built.ranges[G_LENS];
    const ao = bakeVertexAO(
      geo.attributes.position.array, geo.attributes.normal.array, built.count,
      bodyCapsules(rig), lensRange.start, lensRange.end);
    tex.atlas = bakeRiderAtlas(built.charts, geo, built.offsets, ao, built.tint,
      ATLAS_SIZE, TEXEL_MIN, aniso);
  }

  const { mats, windU } = buildMaterials(tex, ctx);
  let photoMats = null, photoOn = false;

  // ---- first-person head hide ---------------------------------------------
  // One shared uniform across every rider material (photo set included): the
  // camera publishes its distance to the head each frame, and any vertex whose
  // skin weight is dominated by neck/head/visor collapses to a point at the
  // base of the neck. Collapsing in the *visible* materials only keeps the
  // stock depth material untouched, so the helmet still casts its shadow —
  // which is exactly what a real POV rig shows on the ground ahead.
  const headHideU = { value: 0 };
  function injectHeadHide(mat) {
    const prev = mat.onBeforeCompile;
    mat.onBeforeCompile = (sh, renderer) => {
      if (prev) prev(sh, renderer);
      sh.uniforms.uHeadHide = headHideU;
      sh.vertexShader = sh.vertexShader
        .replace('#include <common>', `#include <common>
        uniform float uHeadHide;
        varying float vArmHide;`)
        .replace('#include <skinning_vertex>', `#include <skinning_vertex>
        vArmHide = 0.0;
        #ifdef USE_SKINNING
        if (uHeadHide > 0.001) {
          // Bones 0..8 and 11,12: trunk (pelvis and full spine), neck, head,
          // visor, clavicles and upper arms. The FP near plane sits at 0.02 m
          // and the lens is INSIDE the spine2 jersey, so the trunk has to go
          // bone-by-bone. The upper arms go with it because the rig-follow in
          // update() parks the ELBOWS just outside the bottom of frame — the
          // discard isoline at the elbow is therefore off-screen, and the
          // visible forearms (9,13) + hands (10,14) read as whole arms coming
          // up from behind the camera. Hidden in the FRAGMENT stage only, and
          // never by moving vertices — a partial vertex collapse renders as a
          // crumpled jersey balloon right in front of the lens.
          vec4 hb = vec4(1.0) - step(vec4(8.5), skinIndex)
                  + step(vec4(10.5), skinIndex) - step(vec4(12.5), skinIndex);
          vArmHide = uHeadHide * dot(max(hb, vec4(0.0)), skinWeight);
        }
        #endif`);
      sh.fragmentShader = sh.fragmentShader
        .replace('#include <common>', '#include <common>\nvarying float vArmHide;')
        .replace('#include <clipping_planes_fragment>',
          'if (vArmHide > 0.5) discard;\n#include <clipping_planes_fragment>');
    };
    const prevKey = mat.customProgramCacheKey;
    mat.customProgramCacheKey = prevKey
      ? () => prevKey.call(mat) + '|headHide'
      : () => mat.name + '|headHide';
  }
  for (const m of mats) injectHeadHide(m);

  // Binary with hysteresis: hidden when the lens closes inside 1.0 m, shown
  // again beyond 1.25 m. The discard is a hard cut, so the uniform must be a
  // hard state too — a mid-fade value just slides the cut isoline through the
  // shoulders. The FP chest camera sits ~0.5 m from the head, far inside the
  // band, so impact shake never flickers it.
  let headHidden = false;
  function setHeadProximity(d) {
    if (!isFinite(d)) headHidden = false;
    else if (d < 1.0) headHidden = true;
    else if (d > 1.25) headHidden = false;
    headHideU.value = headHidden ? 1 : 0;
    // The FP crop opens the hollow sleeve/short tubes at the discard isoline —
    // with front-face culling those cuts read as holes straight through the
    // arm. Render the kit's interior while cropped so the cut shows cloth
    // lining instead. FrontSide again the moment the crop lifts; the helmet
    // stays FrontSide always (see buildMaterials C1).
    const side = headHidden ? THREE.DoubleSide : THREE.FrontSide;
    for (const m of mats) if (m.name === 'riderKit' && m.side !== side) m.side = side;
  }

  const group = new THREE.Group();
  group.name = 'rider';

  const mesh = new THREE.SkinnedMesh(geo, mats);
  mesh.name = 'riderBody';
  mesh.castShadow = !(ctx.settings && ctx.settings.shadows === false);
  mesh.receiveShadow = true;
  mesh.frustumCulled = false;        // always on screen; skinning invalidates bounds

  group.add(rig.bones[0]);
  group.add(mesh);
  // Bind while the whole hierarchy is at the origin so bindMatrix is identity and
  // the 'attached' bind mode cancels the group transform out cleanly every frame.
  group.updateMatrixWorld(true);
  const skeleton = new THREE.Skeleton(rig.bones);
  mesh.bind(skeleton);

  // ---- first-person shadow twin -------------------------------------------
  // In first person the visible skeleton is dragged to the lens (rig follow in
  // update()), and a stretched puppet shadow on the trail would give the trick
  // away. This twin holds the UNMOVED pose and exists only for the shadow map:
  // colorWrite/depthWrite are off, so the colour pass rasterises nothing.
  const shadowBones = rig.bones.map((b) => { const n = new THREE.Bone(); n.name = b.name + 'Shadow'; return n; });
  for (let i = 0; i < shadowBones.length; i++) {
    if (rig.parent[i] >= 0) shadowBones[rig.parent[i]].add(shadowBones[i]);
    shadowBones[i].position.copy(rig.bones[i].position);
  }
  const shadowMat = new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: false });
  shadowMat.name = 'riderShadowProxy';
  const shadowMesh = new THREE.SkinnedMesh(geo, shadowMat);
  shadowMesh.name = 'riderShadow';
  shadowMesh.castShadow = true;
  shadowMesh.receiveShadow = false;
  shadowMesh.frustumCulled = false;
  shadowMesh.visible = false;
  group.add(shadowBones[0]);
  group.add(shadowMesh);
  group.updateMatrixWorld(true);
  const shadowSkeleton = new THREE.Skeleton(shadowBones);
  shadowMesh.bind(shadowSkeleton);

  if (ctx.scene) ctx.scene.add(group);

  // Respect the shadow flag, including live quality changes.
  const applySettings = () => {
    mesh.castShadow = !(ctx.settings && ctx.settings.shadows === false);
  };
  applySettings();
  let offQuality = null;
  if (ctx.events && ctx.events.on) offQuality = ctx.events.on('quality:changed', applySettings);

  const rag = makeRagdoll(rig);

  // First-person rig follow tuning. Head target relative to the LENS: slightly
  // behind (-fwd) and above, which parks the shoulders behind the camera so the
  // arms recede off-frame toward the bars. Each FP_LIMB is [root, mid, end]:
  // root and mid travel rigidly with the trunk, end stays pinned to its anchor
  // (grip / pedal) and the mid→end segment is re-aimed and stretched to reach.
  const FP_BODY_FWD = -0.06;
  const FP_BODY_UP = 0.30;
  // Elbow parking spot in LENS space: behind the lens plane (invisible), below
  // it and out to the side, so each forearm enters the frame from its lower
  // corner and runs to the grip as one straight stretched tube. Keeping the
  // bend angle small is what keeps linear-blend skinning from crumpling.
  // Deep enough below/behind the frame that the elbow blend seam (hidden upper
  // arm meets stretched forearm) never peeks into the bottom corners.
  const FP_ELBOW_BACK = 0.24, FP_ELBOW_DOWN = 0.36, FP_ELBOW_OUT = 0.46;
  const FP_LEGS = [
    [B_THIGHL, B_SHINL, B_FOOTL], [B_THIGHR, B_SHINR, B_FOOTR],
  ];
  const fpCamL = new THREE.Vector3(), fpFwdL = new THREE.Vector3(),
        fpRightL = new THREE.Vector3(), fpUpL = new THREE.Vector3();
  // Forearm stretch plumbing: the forearm BONE is scaled along its own bind
  // axis by writing the bone matrix directly (T*R*S with S an axial scale), so
  // the sleeve genuinely elongates as one tube instead of leaving a skinning
  // blend gap. The hand pre-multiplies the inverse scale so the glove keeps
  // its shape. fpForeDir* are the bind-space forearm axes (bind rotations are
  // identity, so bone-local == rider-local here).
  const fpForeDir = [
    new THREE.Vector3().subVectors(rig.restPos[B_HANDL], rig.restPos[B_LARML]).normalize(),
    new THREE.Vector3().subVectors(rig.restPos[B_HANDR], rig.restPos[B_LARMR]).normalize(),
  ];
  const fpForeLen = [
    rig.restPos[B_HANDL].distanceTo(rig.restPos[B_LARML]),
    rig.restPos[B_HANDR].distanceTo(rig.restPos[B_LARMR]),
  ];
  const fpM0 = new THREE.Matrix4(), fpM1 = new THREE.Matrix4();
  const fpOne = new THREE.Vector3(1, 1, 1);
  const FP_SCALED_BONES = [B_LARML, B_HANDL, B_LARMR, B_HANDR];
  function axialScale(out, d, k) {
    const a = k - 1;
    out.set(
      1 + a * d.x * d.x, a * d.x * d.y, a * d.x * d.z, 0,
      a * d.y * d.x, 1 + a * d.y * d.y, a * d.y * d.z, 0,
      a * d.z * d.x, a * d.z * d.y, 1 + a * d.z * d.z, 0,
      0, 0, 0, 1);
    return out;
  }

  // ---- persistent animation state (all preallocated) ----------------------
  const aGripL = new THREE.Vector3(), aGripR = new THREE.Vector3();
  const aPedalL = new THREE.Vector3(), aPedalR = new THREE.Vector3();
  // Hand IK targets and the inboard bar axis, derived from the grips once per
  // frame: the reach clamp needs them before the arms are solved, and the arm
  // solve then reuses them instead of rebuilding the same vectors.
  const aHandL = new THREE.Vector3(), aHandR = new THREE.Vector3();
  const aBarL = new THREE.Vector3(), aBarR = new THREE.Vector3();
  const gazeWorld = new THREE.Vector3();
  const gazeLocal = new THREE.Vector3();
  const lastBikePos = new THREE.Vector3();
  const invQ = new THREE.Quaternion();
  const zeroV = new THREE.Vector3();
  const hitScratch = { height: 0, normal: new THREE.Vector3(0, 1, 0), material: 0, slope: 0 };

  const D = {
    crouch: 0, fore: 0, aft: 0, fwd: 0, roll: 0, twist: 0, tuck: 0,
    brace: 0, elbowOut: 1,
    comp: 0.3, compRate: 0, hipX: 0, land: 0, headRollX: 0,
  };
  let spineX = 0, spineV = 0;          // spine bend spring (overshoot = weight)
  let hipY = 0, hipYV = 0;             // vertical hip spring
  let peakX = 0, peakV = 0, peakZ = 0, peakZV = 0;
  let prevHeadFwdY = 0, prevHeadRightY = 0;
  let compPrev = 0.3;
  let braceTimer = 0, braceMag = 0;
  let wasAirborne = false;
  let ragBlend = 0;
  let gazeInit = false;
  let started = false;

  // ---- helpers ------------------------------------------------------------

  /** World-space bikeModel anchor -> rider-local, with a hard sanity gate. */
  function readAnchor(nm, out, fb) {
    const bm = ctx.bikeModel;
    const a = bm && bm.anchors ? bm.anchors[nm] : null;
    if (a && typeof a.getWorldPosition === 'function') {
      a.getWorldPosition(_v10);
      if (isFinite(_v10.x) && isFinite(_v10.y) && isFinite(_v10.z)) {
        _v10.sub(group.position).applyQuaternion(invQ);
        // A half-built bikeModel reports anchors at the origin or miles away.
        if (_v10.lengthSq() < 6.25 && _v10.lengthSq() > 1e-4) { out.copy(_v10); return true; }
      }
    }
    out.set(fb[0], fb[1], fb[2]);
    return false;
  }

  function setEuler(q, x, y, z) {
    _e0.set(x, y, z, 'YXZ');
    q.setFromEuler(_e0);
  }

  /** Two-bone IK onto a target, then orient the three bones of the chain. */
  function solveLimb(iRoot, iMid, iEnd, target, pole, endDir, endRef) {
    const root = rig.worldPos[iRoot];
    const l1 = rig.restLen[iRoot], l2 = rig.restLen[iMid];
    solveTwoBone(root, l1, l2, target, pole, _v4, _v5);

    _v3.copy(_v4).sub(root);
    orientBone(rig, iRoot, _v3, _v5, _qa);
    worldToLocalQuat(rig, iRoot, _qa, rig.localQuat[iRoot]);
    rig.worldQuat[iRoot].copy(_qa);

    _v3.copy(target).sub(_v4);
    orientBone(rig, iMid, _v3, _v5, _qb);
    _q1.copy(_qa).invert();
    rig.localQuat[iMid].multiplyQuaternions(_q1, _qb);
    rig.worldQuat[iMid].copy(_qb);
    rig.worldPos[iMid].copy(_v4);

    // endRef === null means "use the lower segment's direction" (wrist follows
    // the forearm); _v3 still holds mid→target at this point.
    orientBone(rig, iEnd, endDir, endRef || _v3, _qc);
    _q1.copy(_qb).invert();
    rig.localQuat[iEnd].multiplyQuaternions(_q1, _qc);
    rig.worldQuat[iEnd].copy(_qc);
    rig.worldPos[iEnd].copy(target);
  }

  function restPose() {
    for (let i = 0; i < NBONES; i++) {
      rig.localQuat[i].identity();
      rig.localPos[i].copy(rig.restLocal[i]);
    }
  }

  function writeBones() {
    for (let i = 0; i < NBONES; i++) {
      rig.bones[i].position.copy(rig.localPos[i]);
      rig.bones[i].quaternion.copy(rig.localQuat[i]);
    }
  }

  // ---- update -------------------------------------------------------------

  function update(dt, c) {
    const s = c.bike && c.bike.state;
    const inp = (c.input && c.input.state) || EMPTY_INPUT;
    windU.uTime.value = c.time;

    if (!s) {
      restPose();
      writeBones();
      return;
    }
    if (!started) { lastBikePos.copy(s.position); started = true; }

    const dtc = Math.max(1e-4, Math.min(dt, 1 / 20));
    const speed = s.speed || 0;
    const brake = Math.max(s.brakeFront || 0, s.brakeRear || 0);
    const airborne = !!s.airborne;
    const crashed = !!s.crashed;

    // ---- crash / respawn bookkeeping --------------------------------------
    const jumped = lastBikePos.distanceToSquared(s.position) > 16;   // teleport
    lastBikePos.copy(s.position);
    if (crashed && !rag.active && !jumped) {
      fk(rig);
      rag.seed(rig, group.position, group.quaternion,
        s.velocity || zeroV, s.angularVelocity || zeroV, rng);
      ragBlend = 0;
    }
    if (jumped) { rag.active = false; ragBlend = 0; }
    if (rag.active) {
      rag.step(dtc, c);
      ragBlend = damp(ragBlend, crashed ? 1 : 0, crashed ? 16 : 5.5, dtc);
      if (!crashed && ragBlend < 0.02) { rag.active = false; ragBlend = 0; }
    }

    // ---- group transform ---------------------------------------------------
    if (ragBlend > 0.001) {
      _v0.set(rag.pos[P_PELVIS * 3], rag.pos[P_PELVIS * 3 + 1], rag.pos[P_PELVIS * 3 + 2]);
      group.position.copy(s.position).lerp(_v0, ragBlend);
      _q0.identity();
      group.quaternion.copy(s.quaternion).slerp(_q0, ragBlend);
    } else {
      group.position.copy(s.position);
      group.quaternion.copy(s.quaternion);
    }
    invQ.copy(group.quaternion).invert();

    // ---- anchors -----------------------------------------------------------
    readAnchor('gripL', aGripL, FB_GRIP_L);
    readAnchor('gripR', aGripR, FB_GRIP_R);
    readAnchor('pedalL', aPedalL, FB_PEDAL_L);
    readAnchor('pedalR', aPedalR, FB_PEDAL_R);

    // C3 — palm centre, not bar centre: HAND_IN inboard of the grip's outer end
    // so the glove sits WITHIN the grip's length, and HAND_UP above the bar axis
    // so the palm rests on top of it and the fingers (built as arcs about that
    // axis) close underneath.
    for (let side = 0; side < 2; side++) {
      const grip = side === 0 ? aGripL : aGripR;
      const other = side === 0 ? aGripR : aGripL;
      const bar = side === 0 ? aBarL : aBarR;
      const hand = side === 0 ? aHandL : aHandR;
      bar.copy(other).sub(grip);
      if (bar.lengthSq() < 1e-6) bar.set(side === 0 ? 1 : -1, 0, 0);
      bar.normalize();                              // inboard along the bar
      hand.copy(grip).addScaledVector(bar, HAND_IN);
      hand.y += HAND_UP;
    }

    // ---- drivers -----------------------------------------------------------
    const fork = s.suspension && s.suspension.fork;
    const shock = s.suspension && s.suspension.shock;
    const compF = fork && fork.max > 0 ? clamp01(fork.travel / fork.max) : 0.3;
    const compR = shock && shock.max > 0 ? clamp01(shock.travel / shock.max) : 0.3;
    const comp = (compF + compR) * 0.5;
    const rawRate = (comp - compPrev) / dtc;
    compPrev = comp;
    D.comp = comp;
    D.compRate = damp(D.compRate, clamp(rawRate, -14, 14), 18, dtc);
    const compRel = comp - 0.30;                        // 30% sag is "neutral"

    // Landing edge -> brace impulse.
    if (wasAirborne && !airborne) {
      const ll = s.lastLanding;
      const sev = ll && isFinite(ll.vNormal) ? clamp01(Math.abs(ll.vNormal) / 8) : clamp01((s.gForce || 1) - 1);
      braceTimer = 0.40; braceMag = 0.35 + sev * 0.75;
    }
    wasAirborne = airborne;
    braceTimer = Math.max(0, braceTimer - dtc);
    // Sharp attack, slower release — a rider takes the hit then stands back up.
    const braceT = braceTimer / 0.40;
    const brace = braceMag * (braceT > 0.8 ? (1 - braceT) * 5 : braceT / 0.8);

    // Height above the ground, used to pre-load the legs before touchdown.
    let clearance = 99;
    if (airborne) {
      let gh = -1e9;
      if (c.collision && c.collision.probe) {
        c.collision.probe(s.position.x, s.position.z, hitScratch);
        gh = hitScratch.height;
      } else if (c.terrain && c.terrain.sampleHeight) {
        gh = c.terrain.sampleHeight(s.position.x, s.position.z);
      }
      if (gh > -1e8) clearance = s.position.y - 0.37 - gh;
    }
    const falling = (s.velocity && s.velocity.y < -0.5);
    const landPrep = airborne && falling ? smoothstep(1.7, 0.35, clearance) : 0;
    D.land = damp(D.land, landPrep, 14, dtc);

    // Tuck: fast, straight, off the brakes.
    const steerMag = Math.min(1, Math.abs(inp.steer || 0) + Math.abs(s.lean || 0) * 1.5);
    const tuckT = smoothstep(10, 18.5, speed) * (1 - clamp01(steerMag * 1.7)) *
      (1 - brake) * (airborne ? 0.35 : 1);
    D.tuck = damp(D.tuck, tuckT, 3.2, dtc);

    // Crouch: 0 = the built attack stance, + = compressed, − = extended tall.
    let crouchT = compRel * 1.05;
    crouchT += clamp01((s.gForce || 1) - 1) * 0.26;
    crouchT += brake * 0.18;
    crouchT += D.tuck * 0.34;
    crouchT -= (s.pumpDrive || 0) * 0.34;             // driving down = legs extend
    crouchT += (s.pumpImpulse || 0) * 0.10;
    crouchT += (isFinite(s.riderCrouch) ? (s.riderCrouch - 0.35) : 0) * 0.55;
    if (airborne) {
      const at = s.airTime || 0;
      crouchT += smoothstep(0.34, 0.04, at) * 0.42;   // suck up the lip
      crouchT -= smoothstep(0.22, 0.75, at) * 0.34 * (1 - D.land);
      crouchT += D.land * 0.60;
    }
    crouchT += brace * 0.80;
    D.crouch = damp(D.crouch, clamp(crouchT, -0.55, 1.30), 15, dtc);

    // ---- R9: fore/aft weight — the player's, not the animator's ------------
    //
    // Round 8 established that the fore/aft shift is not decoration: full pitch
    // moves the CG 0.246 m aft, which on a 45% grade takes the rear load
    // fraction from ~0.13 to ~0.33 and the lean ceiling with it. A player who
    // does not know to do it finds the game unridable. So the animation must
    // TELL them — which means the rider must look meaningfully different when
    // they shift and when they do not.
    //
    // The code here used to do the opposite. `smoothstep(0.12, 0.46, gradient)
    // * 0.80` moved the rider aft on grade ALONE, with more authority (0.80)
    // than the player's own command had (0.70). Evaluating this module's own
    // arithmetic across the grade range:
    //
    //   grade   D.fore, no input   D.fore, full S   player-owned hip travel
    //     0%         0.000            -0.700              0.130 m
    //    30%        -0.435            -1.135              0.130 m
    //    45%        -0.798            -1.200              0.074 m   <- worst
    //
    // i.e. on the ground where the mechanic decides whether you finish the run,
    // two thirds of the shift happened for free and the player's contribution
    // shrank to 74 mm — the least legible signal exactly where it mattered
    // most. The animation was quietly performing the input it was supposed to
    // be teaching.
    //
    // Now: `state.riderFore` owns the travel, and grade AMPLIFIES the player's
    // expression instead of substituting for it (a rider getting back on a 45%
    // chute goes much further behind the saddle than one doing it on a fire
    // road). Same measurement, after:
    //
    //   grade   D.fore, no input   D.fore, full S   player-owned hip travel
    //     0%        +0.000            -1.000              0.220 m
    //    30%        +0.120            -1.200              0.284 m
    //    45%        +0.230            -1.350              0.332 m   <- best
    //
    // and the silhouette at the two ends of the 45% row: hips 0.31 m back and
    // 0.16 m down, head 0.36 m lower, arms 74% -> 98% extended.
    //
    const gradient = -(s.forward ? s.forward.y : 0);      // + = descending
    const steep = smoothstep(0.10, 0.50, gradient);
    // The command, in units of RIDER_SHIFT. bike.js derives this from
    // input.pitch; we never re-derive it, we only rescale it to ±1.
    const foreCmd = clamp((s.riderFore || 0) / RIDER_SHIFT, -1, 1);
    let foreT = foreCmd * (1 + steep * 0.40);
    // Un-commanded on a steep, the rider is stacked over the bars — high, hips
    // over the saddle, nothing in reserve. Kept small (a few cm) so the neutral
    // pose never reads as a bug, but deliberately on the WRONG side of neutral:
    // it must not look like the shift the player has not made.
    foreT += steep * (1 - clamp01(-foreCmd)) * 0.24;
    foreT += (s.pedalling || 0) * 0.40;
    // Braking is a brace, not a weight shift — `brace`/`crouch` already carry
    // it. The old -0.38 here was a third automatic term competing with the
    // player for the same axis.
    foreT -= brake * 0.10;
    foreT -= clamp01(s.endo || 0) * 0.55;
    foreT += clamp01(s.wheelie || 0) * 0.30;
    D.fore = damp(D.fore, clamp(foreT, -1.35, 0.9), 8, dtc);

    // Split, because the two directions are not the same movement: getting back
    // is hips back AND down with the chest dropping onto the top tube, going
    // forward is a smaller move over the stem.
    const aft = Math.max(0, -D.fore);
    const fwd = Math.max(0, D.fore);
    D.aft = aft; D.fwd = fwd;

    // Lean separation: the rider stays more upright than the bike, and the gap
    // widens with lean angle and speed. This is the thing that reads as "skilled".
    const bikeLean = s.lean || 0;
    const kSep = clamp(0.64 - 0.22 * clamp01(Math.abs(bikeLean) / 0.75)
      - 0.09 * clamp01((speed - 6) / 14), 0.28, 0.75);
    const rollT = (1 - kSep) * bikeLean;
    D.roll = damp(D.roll, rollT, 12, dtc);
    D.headRollX = (kSep - 0.22) * bikeLean;
    D.hipX = damp(D.hipX, -bikeLean * 0.055, 9, dtc);

    // Hips counter-rotate against a whip; on the ground a touch of lead into the turn.
    let twistT = -clamp(s.whip || 0, -1.1, 1.1) * 0.55 - (inp.steer || 0) * 0.07;
    D.twist = damp(D.twist, clamp(twistT, -0.6, 0.6), 10, dtc);
    // Elbows come in as the arms straighten — a rider at full extension behind
    // the saddle is not chicken-winging, they are hanging off the bar.
    D.elbowOut = damp(D.elbowOut,
      1 - D.tuck * 0.55 + clamp01(D.crouch) * 0.20 - aft * 0.22, 6, dtc);
    D.brace = brace;

    // ---- pose: root + spine ------------------------------------------------
    restPose();

    // Vertical hip spring: mass has inertia, so the hips lag the bike a little.
    // R9: getting back is also getting LOW — the seat passes under the rider's
    // chest, not behind their hips. The drop is what makes the shift read from
    // the chase camera, and it is nearly free against the arm budget (dropping
    // the shoulder shortens the vertical leg of the shoulder→grip triangle, so
    // it buys back most of what the rearward translation spends).
    const hipTargetY = -D.crouch * 0.148 - aft * 0.095;   // C7: scaled with the body
    const hipAcc = (hipTargetY - hipY) * 300 - hipYV * 24;
    hipYV += hipAcc * dtc; hipY += hipYV * dtc;
    hipY = clamp(hipY, -0.234, 0.126);

    const rp = rig.localPos[B_PELVIS];
    rp.copy(rig.restLocal[B_PELVIS]);
    rp.y += hipY;
    // Asymmetric: 0.22 m of rearward travel (against bike.js's own 0.30 m of
    // CG shift) versus 0.17 m forward. The reach clamp below decides how much
    // of the rearward demand the arms will actually allow.
    rp.z += D.crouch * 0.076 + aft * 0.22 - fwd * 0.17;
    rp.x += D.hipX;

    // Spine bend spring — slight overshoot so the torso has weight.
    // R9: chest down as the hips go back (the flat-back steep-chute shape), and
    // no back-ARCH on the forward side — a rider moving over the stem is not
    // leaning away from it, and that term was what pushed the tall-and-forward
    // corner 56 mm past the end of the arm.
    const spineTarget = 0.30 * D.crouch + 0.22 * brake + 0.42 * D.tuck + aft * 0.14
      + D.land * 0.18;
    const spineAcc = (spineTarget - spineX) * 165 - spineV * 17;
    spineV += spineAcc * dtc; spineX += spineV * dtc;
    spineX = clamp(spineX, -0.45, 0.95);

    // Rotation about +X is "look up", so bending forward is negative X.
    // R9: the extra `aft * 0.11` is anterior pelvic tilt — the hips rotate
    // forward as they translate back, which is what puts the tailbone behind
    // the saddle instead of merely sliding the whole rider rearwards.
    setEuler(rig.localQuat[B_PELVIS],
      -(D.crouch * 0.14 - D.fore * 0.10 + aft * 0.11), D.twist * 0.55, D.roll * 0.40);
    for (let i = 0; i < 3; i++) {
      setEuler(rig.localQuat[SPINE_BONES[i]],
        -spineX * SPINE_SHARE[i],
        -D.twist * 0.90 * SPINE_SHARE[i],
        D.roll * 0.20);
    }

    fk(rig);

    // ---- R9: the arms are the limit on how far back a rider can get --------
    //
    // This is a real constraint, not a safety net: arms-length behind the
    // saddle is exactly where a downhill rider runs out of travel, and it is
    // why the pose bottoms out into "arms extended, chest down" rather than
    // continuing to slide backwards. Without it the rig does not fail loudly —
    // `solveLimb` snaps the hand onto the grip whatever the distance, so the
    // forearm simply renders longer than it is.
    //
    // Closed form, not iteration: the correction is a pure translation of the
    // root along z, the spine rotation is already fixed at this point, so the
    // shoulder translates by exactly the same z while the lateral and vertical
    // separations from the grip do not move at all. Solving
    // `dx² + dy² + (dz − fix)² = ARM_REACH²` for `fix` is therefore exact.
    {
      let fix = 0;
      for (let side = 0; side < 2; side++) {
        const sh = rig.worldPos[side === 0 ? B_UARML : B_UARMR];
        const hand = side === 0 ? aHandL : aHandR;
        const dx = hand.x - sh.x, dy = hand.y - sh.y, dz = hand.z - sh.z;
        const rem = ARM_REACH * ARM_REACH - dx * dx - dy * dy;
        // rem <= 0 means the grip is out of reach on the lateral/vertical legs
        // alone; the best we can do is put the shoulder level with it in z.
        const m = rem > 0 ? Math.sqrt(rem) : 0;
        let f = 0;
        if (dz < -m) f = dz + m;            // too far back  -> pull forward
        else if (dz > m) f = dz - m;        // too far forward -> push back
        if (Math.abs(f) > Math.abs(fix)) fix = f;
      }
      if (Math.abs(fix) > 1e-4) {
        rig.localPos[B_PELVIS].z += fix;
        fk(rig);
      }
    }

    // ---- gaze --------------------------------------------------------------
    if (!crashed) {
      const trail = c.trail;
      if (trail && typeof trail.sampleAt === 'function' && trail.length > 1) {
        const look = (13 + speed * 1.15) / trail.length;
        const sm = trail.sampleAt(clamp01((s.trailT || 0) + look));
        _v0.copy(sm.position);
        _v0.y += 1.30;
        // Riders look through the turn, not at the apex they are already on.
        if (sm.binormal) _v0.addScaledVector(sm.binormal, (inp.steer || 0) * 0.7);
      } else {
        _v0.copy(s.position).addScaledVector(s.forward || FB_UNIT_FWD, 16);
        _v0.y += 0.9;
      }
      if (!gazeInit) { gazeWorld.copy(_v0); gazeInit = true; }
      else gazeWorld.lerp(_v0, 1 - Math.exp(-6.5 * dtc));
    }
    gazeLocal.copy(gazeWorld).sub(group.position).applyQuaternion(invQ);

    // Head/neck look-at, clamped so the neck never breaks.
    _v1.copy(gazeLocal).sub(rig.worldPos[B_HEAD]);
    if (_v1.lengthSq() < 1e-6) _v1.set(0, 0, -1);
    _v1.normalize();
    _q0.copy(rig.worldQuat[B_SPINE3]).invert();
    _v1.applyQuaternion(_q0);
    let yaw = Math.atan2(-_v1.x, -_v1.z);
    let pitch = Math.asin(clamp(_v1.y, -1, 1));
    if (!isFinite(yaw)) yaw = 0;
    // C4 — the round-3 chase shot has the head yawed ~65 deg off the fall line,
    // which is most of why the pose reads wrong. A rider looking through a turn
    // turns the head ~30-35 deg and no more; beyond that the eyes do the work.
    yaw = clamp(yaw, -0.62, 0.62);
    pitch = clamp(pitch, -0.40, 0.72);
    setEuler(rig.localQuat[B_NECK], pitch * 0.35, yaw * 0.35, D.headRollX * 0.35);
    setEuler(rig.localQuat[B_HEAD], pitch * 0.65, yaw * 0.65, D.headRollX * 0.65);

    fk(rig);

    // ---- arms --------------------------------------------------------------
    for (let side = 0; side < 2; side++) {
      const left = side === 0;
      const iC = left ? B_CLAVL : B_CLAVR, iU = left ? B_UARML : B_UARMR,
            iL = left ? B_LARML : B_LARMR, iH = left ? B_HANDL : B_HANDR;
      const hand = left ? aHandL : aHandR, bar = left ? aBarL : aBarR;
      const sgn = left ? -1 : 1;

      // Shrug the shoulders a little when braced or crouched.
      setEuler(rig.localQuat[iC], -D.crouch * 0.06 - D.brace * 0.10, 0, sgn * D.crouch * 0.05);
      // Recompute this clavicle's world transform only (cheap, keeps FK honest).
      rig.worldQuat[iC].multiplyQuaternions(rig.worldQuat[B_SPINE3], rig.localQuat[iC]);
      rig.worldPos[iC].copy(rig.restLocal[iC]).applyQuaternion(rig.worldQuat[B_SPINE3])
        .add(rig.worldPos[B_SPINE3]);
      rig.worldQuat[iU].multiplyQuaternions(rig.worldQuat[iC], rig.localQuat[iU]);
      rig.worldPos[iU].copy(rig.restLocal[iU]).applyQuaternion(rig.worldQuat[iC])
        .add(rig.worldPos[iC]);

      // C3 — the palm centre (`aHandL/R`) and the inboard bar axis (`aBarL/R`)
      // were built from the grips at the top of update(), because the reach
      // clamp needs them before the spine is committed. Round 3: "the bar
      // passes through the glove volume and the grip re-emerges outboard".
      _v0.copy(hand);

      // Pole: C4 — elbows OUTBOARD of the torso silhouette. The lateral term now
      // dominates (0.98 vs 0.58) and the rearward term is cut, which is what
      // turns a commuter perch into an attack position.
      _v1.set(sgn * 0.98 * D.elbowOut, 0.46 + D.brace * 0.14, 0.42).normalize()
        .applyQuaternion(rig.worldQuat[B_SPINE3]);
      _v2.copy(rig.worldPos[iU]).addScaledVector(_v1, 0.72);

      // Hand: fingers wrap the bar (long axis inboard). The roll reference must
      // be the frame the glove was AUTHORED in — bike-up (the hand's restRef is
      // +Y) — not the forearm direction: the forearm meets the wrist at ~140°
      // to vertical, and using it rolled the whole grip top-and-backwards. Real
      // hands stay wrapped around the bar however the elbows move.
      _v11.copy(bar);
      _v9.set(0, 1, 0);
      solveLimb(iU, iL, iH, _v0, _v2, _v11, _v9);
    }

    // ---- legs --------------------------------------------------------------
    const footPitch = 0.05 + brake * 0.30 + D.brace * 0.26 - (s.pedalling || 0) * 0.14
      + clamp01(D.crouch) * 0.10;
    for (let side = 0; side < 2; side++) {
      const left = side === 0;
      const iT = left ? B_THIGHL : B_THIGHR, iS = left ? B_SHINL : B_SHINR,
            iF = left ? B_FOOTL : B_FOOTR;
      const pedal = left ? aPedalL : aPedalR;
      const sgn = left ? -1 : 1;

      // Ankle sits above and slightly behind the pedal spindle. These are the
      // same constants the rest pose was authored from, so the flat sole plane
      // built in the foot's frame lands on the pedal platform.
      _v0.copy(pedal);
      _v0.y += ANKLE_UP;
      _v0.z += ANKLE_BACK;

      // Knees track forward, splaying out as the rider gets low.
      _v1.set(sgn * (0.32 + clamp01(D.crouch) * 0.28), 0.06, -0.94).normalize();
      _v2.copy(rig.worldPos[iT]).addScaledVector(_v1, 0.74);

      // Foot: heels drop under braking and on impact — the classic DH tell.
      _v11.set(0, Math.sin(footPitch), -Math.cos(footPitch));
      _v9.set(Math.sin(D.roll * 0.30) * sgn * 0.35, Math.cos(D.roll * 0.30), 0);
      solveLimb(iT, iS, iF, _v0, _v2, _v11, _v9);
    }

    // ---- helmet peak spring ------------------------------------------------
    _v0.set(0, 0, -1).applyQuaternion(rig.worldQuat[B_HEAD]);
    _v1.set(1, 0, 0).applyQuaternion(rig.worldQuat[B_HEAD]);
    const dPitch = (_v0.y - prevHeadFwdY) / dtc;
    const dRoll = (_v1.y - prevHeadRightY) / dtc;
    prevHeadFwdY = _v0.y; prevHeadRightY = _v1.y;
    const vertJolt = D.compRate * 0.012 + (s.gForce || 1) - 1;
    peakV += (-peakX * 260 - peakV * 17 + clamp(dPitch, -25, 25) * 0.22 + clamp(vertJolt, -4, 4) * 0.9) * dtc;
    peakX = clamp(peakX + peakV * dtc, -0.17, 0.17);
    peakZV += (-peakZ * 300 - peakZV * 19 + clamp(dRoll, -25, 25) * 0.16) * dtc;
    peakZ = clamp(peakZ + peakZV * dtc, -0.13, 0.13);
    setEuler(rig.localQuat[B_PEAK], peakX, 0, peakZ);

    // ---- ragdoll blend -----------------------------------------------------
    if (ragBlend > 0.001) {
      fk(rig);
      const rootLocal = rag.pose(rig, invQ, group.position, rig.ragQuat);
      const b = smoothstep(0, 1, ragBlend);
      for (let i = 0; i < NBONES; i++) rig.localQuat[i].slerp(rig.ragQuat[i], b);
      rig.localPos[B_PELVIS].lerp(rootLocal, b);
    }

    writeBones();

    // ---- first-person rig follow -------------------------------------------
    // FP: the trunk is BOLTED to the lens. The pose above stays untouched in
    // rig.worldPos/worldQuat — it is what the shadow twin shows and what
    // getHeadPosition() feeds the camera, so there is no feedback loop between
    // the camera spring and the body that follows it. The delta between the
    // lens-mounted head target and the solved head is added to the THREE bones
    // after writeBones(); hands and feet are counter-pinned to the bars and
    // pedals, so the shoulders travel with the trunk and only the forearms and
    // shins stretch — deliberately, joints be damned.
    const cc = c.chaseCamera;
    let followOn = false;
    if (!photoOn && ragBlend < 0.01 && !rag.active && c.camera && cc &&
        cc.mode === 'firstPerson' && !cc.blending) {
      _v0.set(0, 0, -1).applyQuaternion(c.camera.quaternion);
      _v0.y = 0;
      if (_v0.lengthSq() > 1e-6) {
        _v0.normalize();
        // Head target in lens space: just behind and above the lens, so the
        // shoulders sit BEHIND the camera and the arms recede out of frame
        // instead of hanging in front of it.
        _v1.copy(c.camera.position).addScaledVector(_v0, FP_BODY_FWD);
        _v1.y += FP_BODY_UP;
        getHeadPosition(_v2);
        _v1.sub(_v2);                                  // world-space delta
        if (_v1.lengthSq() < 4) {                      // sanity: < 2 m
          followOn = true;
          // Shadow twin freezes the unmoved pose before the drag.
          for (let i = 0; i < NBONES; i++) {
            shadowBones[i].position.copy(rig.bones[i].position);
            shadowBones[i].quaternion.copy(rig.bones[i].quaternion);
          }
          _v1.applyQuaternion(invQ);                   // rider-local delta
          rig.bones[B_PELVIS].position.add(_v1);
          // Camera frame in rider-local space, for the elbow parking spots.
          fpCamL.copy(c.camera.position).sub(group.position).applyQuaternion(invQ);
          fpFwdL.copy(_v0).applyQuaternion(invQ);      // _v0 = lens fwd, horiz
          _v2.set(0, 1, 0);
          fpUpL.copy(_v2).applyQuaternion(invQ);
          _v2.crossVectors(_v0, _v2).normalize();      // lens right, horiz
          fpRightL.copy(_v2).applyQuaternion(invQ);
          // Arms: shoulder rides with the trunk, elbow parks in LENS space just
          // behind/below the frame corner, hand stays pinned to the grip. Both
          // segments are re-aimed along their new directions so the stretch
          // runs along the bone axis and the sleeve stays a clean tube.
          for (let k = 0; k < 2; k++) {
            const iC = k ? B_CLAVR : B_CLAVL, iU = k ? B_UARMR : B_UARML,
                  iL = k ? B_LARMR : B_LARML, iH = k ? B_HANDR : B_HANDL;
            _v2.copy(rig.worldPos[iU]).add(_v1);       // shoulder, moved
            _v4.copy(rig.worldPos[iH]);                // hand, pinned
            _v3.copy(fpCamL)
              .addScaledVector(fpFwdL, -FP_ELBOW_BACK)
              .addScaledVector(fpUpL, -FP_ELBOW_DOWN)
              .addScaledVector(fpRightL, (k ? 1 : -1) * FP_ELBOW_OUT); // elbow target
            // upper arm: aim shoulder -> elbow
            _v5.copy(rig.worldPos[iL]).sub(rig.worldPos[iU]);
            _v6.copy(_v3).sub(_v2);
            if (_v5.lengthSq() < 1e-8 || _v6.lengthSq() < 1e-8) continue;
            _qb.setFromUnitVectors(_v5.normalize(), _v6.normalize()).multiply(rig.worldQuat[iU]);
            _q0.copy(rig.worldQuat[iC]).invert();
            rig.bones[iU].quaternion.multiplyQuaternions(_q0, _qb);
            _q0.copy(_qb).invert();
            _v5.copy(_v3).sub(_v2).applyQuaternion(_q0);
            rig.bones[iL].position.copy(_v5);
            // forearm: aim elbow -> hand
            _v5.copy(rig.worldPos[iH]).sub(rig.worldPos[iL]);
            _v6.copy(_v4).sub(_v3);
            if (_v5.lengthSq() < 1e-8 || _v6.lengthSq() < 1e-8) continue;
            _qc.setFromUnitVectors(_v5.normalize(), _v6.normalize()).multiply(rig.worldQuat[iL]);
            _q0.copy(_qb).invert();
            rig.bones[iL].quaternion.multiplyQuaternions(_q0, _qc);
            _q0.copy(_qc).invert();
            _v5.copy(_v4).sub(_v3).applyQuaternion(_q0);
            rig.bones[iH].position.copy(_v5);
            rig.bones[iH].quaternion.multiplyQuaternions(_q0, rig.worldQuat[iH]);
            // Stretch the forearm BONE along its bind axis so the sleeve mesh
            // itself elongates to span elbow->hand; the glove pre-multiplies
            // the inverse scale and keeps its shape.
            const kS = Math.max(0.2, _v5.length() / Math.max(1e-4, fpForeLen[k]));
            const bL = rig.bones[iL], bH = rig.bones[iH];
            bL.matrixAutoUpdate = false; bH.matrixAutoUpdate = false;
            fpM0.compose(bL.position, bL.quaternion, fpOne);
            bL.matrix.multiplyMatrices(fpM0, axialScale(fpM1, fpForeDir[k], kS));
            fpM0.compose(bH.position, bH.quaternion, fpOne);
            bH.matrix.multiplyMatrices(axialScale(fpM1, fpForeDir[k], 1 / kS), fpM0);
          }
          // Legs: knee rides with the trunk, foot stays on the pedal; the shin
          // is re-aimed at the pinned foot. Any blend crumple sits at the
          // pedals, below the bottom edge of the FP frame.
          for (let k = 0; k < FP_LEGS.length; k++) {
            const iR = FP_LEGS[k][0], iM = FP_LEGS[k][1], iE = FP_LEGS[k][2];
            _v2.copy(rig.worldPos[iM]).add(_v1);       // knee, moved
            _v3.copy(rig.worldPos[iE]);                // foot, pinned
            _v4.copy(_v3).sub(rig.worldPos[iM]);
            _v5.copy(_v3).sub(_v2);
            if (_v4.lengthSq() < 1e-8 || _v5.lengthSq() < 1e-8) continue;
            _q1.setFromUnitVectors(_v4.normalize(), _v5.normalize()).multiply(rig.worldQuat[iM]);
            _q0.copy(rig.worldQuat[iR]).invert();
            rig.bones[iM].quaternion.multiplyQuaternions(_q0, _q1);
            _q0.copy(_q1).invert();
            _v3.sub(_v2).applyQuaternion(_q0);
            rig.bones[iE].position.copy(_v3);
            rig.bones[iE].quaternion.multiplyQuaternions(_q0, rig.worldQuat[iE]);
          }
        }
      }
    }
    if (!followOn) {
      for (let k = 0; k < FP_SCALED_BONES.length; k++) {
        const b = rig.bones[FP_SCALED_BONES[k]];
        if (!b.matrixAutoUpdate) b.matrixAutoUpdate = true;
      }
    }
    const shadowsOn = !(c.settings && c.settings.shadows === false);
    mesh.castShadow = followOn ? false : shadowsOn;
    shadowMesh.visible = followOn && shadowsOn;

    // ---- cloth wind --------------------------------------------------------
    const windAmp = (0.006 + clamp01(speed / 24) * 0.032) * (1 - ragBlend * 0.5);
    windU.uWindAmp.value = damp(windU.uWindAmp.value, windAmp, 5, dtc);
    windU.uGust.value = 0.5 + 0.5 * Math.sin(c.time * 1.9) * Math.sin(c.time * 0.61 + 1.3);
  }

  // ---- misc API -----------------------------------------------------------

  function getHeadPosition(out) {
    out.copy(rig.worldPos[B_HEAD]);
    out.y += 0.092;                                // eyeline, behind the lens
    out.applyQuaternion(group.quaternion).add(group.position);
    return out;
  }

  /** C8 — swap in the expensive material set for photo mode, and back again. */
  function setPhotoMode(on) {
    const want = !!on;
    if (want === photoOn) return;
    photoOn = want;
    if (want && !photoMats) {
      photoMats = buildPhotoMaterials(tex, mats);
      // The kit material is shared with the gameplay set and already injected.
      for (const m of photoMats) if (mats.indexOf(m) < 0) injectHeadHide(m);
    }
    mesh.material = want ? photoMats : mats;
  }

  function dispose() {
    if (offQuality) offQuality();
    geo.dispose();
    for (const m of mats) m.dispose();
    if (photoMats) for (const m of photoMats) if (mats.indexOf(m) < 0) m.dispose();
    for (const k in tex) if (tex[k] && tex[k].dispose) tex[k].dispose();
    if (skeleton.dispose) skeleton.dispose();
    if (shadowSkeleton.dispose) shadowSkeleton.dispose();
    shadowMat.dispose();
    if (ctx.scene) ctx.scene.remove(group);
  }

  // Sensible pose before the first update (menu / cinematic camera).
  restPose();
  fk(rig);
  writeBones();

  return {
    group, mesh, skeleton, bones: rig.bones, materials: mats,
    update, dispose, getHeadPosition, setHeadProximity, setPhotoMode,
    isRagdoll: () => rag.active,
    drivers: D,
  };
}

const FB_UNIT_FWD = new THREE.Vector3(0, 0, -1);
