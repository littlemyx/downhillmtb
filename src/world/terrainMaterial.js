// terrainMaterial.js — DESCENT ground shading.
//
// createTerrainMaterial(ctx, opts) -> THREE.MeshStandardMaterial patched via
// onBeforeCompile.  Everything Three gives us (shadows, IBL/PMREM environment,
// fog, tonemapping, dithering) is preserved; we only replace the parts of the
// physical shader that decide albedo / roughness / normal / AO.
//
// What it does, per CONTRACT §3:
//   * 8 procedurally-synthesised surfaces (dirt, loam, rock, gravel, grass,
//     root, mud, snow) packed into three GPU texture ARRAYS:
//       uAlbedoArray  RGB = albedo (sRGB internal format)  A = baked AO (linear)
//       uNrhArray     RG  = tangent normal xy              B = roughness  A = height
//       uDetailArray  RG  = micro normal xy                B = albedo mod A = rough mod
//     Every one of those channels derives from a real generated height field —
//     the normal maps are the gradient of that field, never a flat blue plate.
//   * Height-map splat blending between up to 4 surfaces per fragment
//     (per-layer height + weight, sharp "who pokes through" transition).
//   * Triplanar projection faded in above ~26-47 deg so cliffs are not smeared,
//     using the whiteout normal blend so flat normal maps still reproduce the
//     geometric normal exactly.
//   * Stochastic (triangle-grid) tile-break with per-cell rotation + offset,
//     blended by height so contrast survives — this is what kills the visible
//     repetition at distance.  All array fetches use textureGrad with gradients
//     taken from the *continuous* uv, so the discontinuous stochastic uv can
//     never produce mip seams.
//   * Two-scale detail: per-layer macro tile (4-16 m) x micro tile (~0.5-1.1 m),
//     plus three octaves of large-scale macro-variation noise (~210 m / 62 m /
//     13 m) modulating brightness, hue and the surface mix itself.
//   * Distance AND screen-footprint detail fade, so the micro layer never
//     aliases into sparkle at range.
//   * Curvature/cavity darkening derived from the world-normal derivative,
//     gated by the on-screen SIZE OF THE TRIANGLE (see the R0 note below).
//   * Wetness uniform: darkens albedo, collapses roughness, flattens normals,
//     and biases the splat toward MUD in drainage lines.
//   * Snow uniform: accumulates by altitude, slope and cavity.
//
// CONTRACT-NOTE: the material can read the terrain's painted SurfaceId in two
// optional ways; both default OFF so the material is correct on its own.
//   1. opts.materialIds — a Uint8Array of SurfaceId (0..7) laid out row-major
//      [ j * res + i ] over the terrain bounds, with opts.materialRes and
//      opts.bounds.  That is exactly terrain.js's `materials` array; passing it
//      is what makes the carved trail read as trail.  Sampled with a manual
//      4-tap bilinear over IDs (an id value is never lerped).
//   2. opts.surfaceAttribute — set true if the chunk geometry carries a float
//      attribute named `aSurface` (per-vertex SurfaceId).  If the define were on
//      while the attribute is missing the whole mountain would read as DIRT, so
//      it is strictly opt-in.
// CONTRACT-NOTE: world position/normal are derived with `modelMatrix`, so terrain
// chunk meshes must not use non-uniform scale (uniform scale + translation are
// fine).  InstancedMesh / BatchedMesh terrain chunks are not supported.
// CONTRACT-NOTE: the generated texture set is cached at module scope per
// (seed, size), so calling createTerrainMaterial() once per LOD chunk is cheap.
// The uniform bundle is shared per ctx, so setWetness()/setSnow()/setTime() on
// any returned material moves every terrain chunk at once.
// CONTRACT-NOTE (terrainMaterial -> terrain, PERF + CORRECTNESS, ACTION REQUIRED):
//   This fragment shader is the largest single line item in the frame.  The
//   per-LOD material variants the r2/r3 work orders ask for now exist, but
//   terrain.js still assigns ONE material to every chunk.  To claim both the
//   far-variant saving and the exact cavity guard, in terrain.js buildChunk():
//       const mat = terrainMat.userData.materialForDepth( depth );
//       const mesh = new THREE.Mesh( geo, mat );
//   `materialForDepth(d)` returns:
//       d >= 7   the near material (full shader, cavity on)
//       d == 6   a "mid" variant — near shader with the cavity term compiled OUT
//       d <= 5   a "far" variant — no tile-break, no micro-detail, no cavity,
//                maxLayers 2, but TRIPLANAR IS KEPT (r3 finding R15: the far
//                band starts around 110 m and a 150 m cliff must not go back
//                to planar smearing)
//   All variants share one texture set and one uniform bundle, so every
//   setWetness()/setSnow()/setTime() call still moves the whole mountain, and
//   nothing is built until it is first asked for.
//
//   The FIRST call to materialForDepth() also tells this module that terrain.js
//   is depth-aware, which lets the base material use its exact 0.5 m vertex
//   spacing for the cavity guard.  Until that call happens the base material
//   assumes the CONSERVATIVE 2.0 m spacing (depth 5) — see the R0 note.
//
//   Measured caveat carried over from r2: a depth split ALONE only returns
//   ~0.5 ms because the near chunks own the pixels — the near variant has
//   therefore also been brought down (tile-break 3 taps -> 2, maxLayers 4 -> 3
//   at high, layer-skip threshold 0.018 -> 0.06), which needs no change in
//   terrain.js.
// CONTRACT-NOTE (r3 R0 — the "translucent ghost quads"):
//   Four r3 lenses filed a critical against three other files for large
//   translucent quads over the terrain.  There is no quad.  It was THIS file:
//   `terrCavity`/`terrCrest` are built from dFdx of a smooth-shaded varying, so
//   they are CONSTANT ACROSS A TRIANGLE, and the old guard faded them by CAMERA
//   distance while terrain.js keys LOD to CORRIDOR distance.  A bank 8 m from
//   the lens but 30 m from the trail spline is a 2 m-per-vertex triangle with
//   the guard sitting at exactly 1.0, so a whole triangle took a flat 55% AO
//   knockdown and a +-16% albedo step.  Measured multiply 0.567 with chroma
//   unchanged — a hue-neutral multiply, not a shadow.
//   The guard is now the ON-SCREEN SIZE OF THE TRIANGLE, which is the quantity
//   that actually decides whether a per-triangle-constant term reads as
//   curvature or as faceting.  Do not put a distance-based guard back.
// CONTRACT-NOTE (specular aliasing): roughness is now floored and widened by a
//   screen-space normal-variance term (Kaplanyan/Toksvig-equivalent), so
//   sub-pixel detail raises roughness instead of aliasing into sparkle.  Do not
//   re-introduce a near-mirror roughness floor for dry ground.
// CONTRACT-NOTE (terrainMaterial -> terrain, uWetness): terrain.js commit()
//   passes `wetness: 0.13` in matOpts (terrain.js:2728), which overrides this
//   module's 0.0 default.  That is the reconciliation for the r4 note that the
//   r3 report claimed the default was 0.0 while the live material read 0.13 —
//   the default change DID take, the caller overrides it.  0.13 is terrain.js's
//   call to make and is not overridden back; instead the wetness block now
//   distinguishes DAMP from a water FILM (see terrWetFilm), so a small value
//   buys drainage-weighted albedo darkening only and can no longer put gloss or
//   a flattened normal on dry ground.
// CONTRACT-NOTE (r5 A8 — the "directional corduroy"):
//   MEASURED, not assumed. Structure-tensor orientation + coherence on
//   high-pass bands of r5_02 / r5_01, against r3_02 as the control:
//     * The artefact is a directional LOW-PASS, not an added pattern: the
//       surface is smooth ALONG the streaks and sharp ACROSS them (r5_02
//       x200-400 y820-1020, 1-3 px band: coherence 0.83; 10-24 px band: 0.36).
//     * Its orientation is VIEW-dependent, not world-dependent. Fitting a
//       radial focus separately to the left slope, the tread and the right
//       slope gives (1360,200), (960,640) and (1040,640) — three surfaces,
//       three foci, each on that surface's own depth direction. A world-locked
//       texture would give three constant angles; a full-screen effect
//       (motion blur, radial blur) would give ONE focus for all three.
//     * It is not motion blur. In r5_01 the foreground course tape is razor
//       sharp at the same depth as ground that is heavily striated.
//     * Lane A6 did not create it, it made it legible: r3_02 carries the same
//       orientation (136-138 deg vs r5's 143-146) at a fine-band RMS of 3.8
//       against r5's 31.3.
//   The only view-dependent directional filter in this material is the
//   sampling footprint itself, and at the incidence angles a downhill chase
//   camera runs at it is a needle (ratio 10-30:1 on a mid-field slope). See
//   terrClampAniso(). Relief amplitude is deliberately NOT reduced: the near
//   tread sits at 20-40 deg of incidence, i.e. ratio 1.6-3, so the clamp does
//   not touch it and the A6 near-field gain is preserved intact.
// CONTRACT-NOTE (terrainMaterial -> vegetation, r4 "pale blue glossy slabs"):
//   The r4 pale-blue faceted slabs (r4_07 right of frame, r4_13 centre, and ~15
//   of them across the hillside in r4_12 / r4_02) are NOT this material and NOT
//   the terrain surface.  They are vegetation.js's boulder instances —
//   buildBoulder() (vegetation.js:2214), a noise-displaced IcosahedronGeometry
//   pushed as K_BOULDER into rockLayer.  Evidence: (1) the facets are an
//   irregular triangulation with a low-poly convex silhouette, not the regular
//   49x49 quad lattice of a depth-7 chunk — at 21-41 m a depth-7 terrain
//   triangle is 8-16 px, the slab's facets are 40-160 px; (2) the slab carries
//   ZERO albedo texture while terrain 2 m away in the same frame at the same
//   distance is fully textured, and no variant of this shader removes all
//   texture (the far variant starts at depth <= 5, i.e. > 110 m, and still
//   samples the albedo array); (3) r4_13's silhouette is a convex dome with a
//   flattened base standing proud of the hillside; (4) boulder placement peaks
//   at exactly the measured 28.6-31.6 deg (vegetation.js:3973,
//   smoothstep(0.10, 0.52, slope) in radians) and LOAM carries dens 0.13, so
//   they do occur off-trail on loam.  The QA raycast missed them because
//   vegetation re-packs mesh.count and the instanceMatrix per frame per chunk
//   and never resets InstancedMesh.boundingSphere, so an out-of-render
//   raycast tests a stale sphere against a stale packing.
//   The regression IS a consequence of Lane A3 — the terrain rock palette went
//   cLight 0x9a978f -> 0x6f6c66 and cQuartz 0xb9b6ad -> 0x8e8b84 (~0.10-0.27
//   linear) and the frame re-exposed down with it — but the boulders did not
//   move: buildBoulder's granite band is lerp(0.30, 0.46) linear, x an
//   instance tint of 0.82-1.22 (vegetation.js:3985), i.e. up to 0.56 linear
//   albedo, ~5x the LOAM it sits on.  A neutral 0.5-albedo body lit mainly by
//   the sky dome reads exactly as measured: 70/110/145 sRGB, B/R 2.09, against
//   ground at 43/51/40, B/R 0.93.
//   FIX BELONGS IN vegetation.js, NOT HERE.  Do not re-brighten the terrain
//   rock to hide it — that re-opens A3/R7.  Suggested: scale the granite band
//   to ~lerp(0.125, 0.235) and the lichen/dirt targets by the same ~0.45, and
//   narrow the instance luminance tint to 0.78 + rng()*0.28.  Separately the
//   boulders have no albedo variation at all at 20-40 m (uRockNormal perturbs
//   the normal only), which is why they read as flat plates.

import * as THREE from 'three';
import { makeRng, subSeed, clamp, clamp01, smoothstep } from '../core/rng.js';

// ---------------------------------------------------------------------------
// Surface table
// ---------------------------------------------------------------------------

export const SurfaceIds = {
  DIRT: 0, LOAM: 1, ROCK: 2, GRAVEL: 3, GRASS: 4, ROOT: 5, MUD: 6, SNOW: 7,
};
const LAYER_COUNT = 8;

// Per-layer art direction. `macro` / `detail` are metres per tile.
const SURFACE_DEFS = [
  { name: 'dirt',   macro: 9.5,  detail: 0.78, heightK: 1.05, normalK: 1.00, roughK: 1.00, tint: 0xffffff },
  { name: 'loam',   macro: 7.5,  detail: 0.62, heightK: 0.92, normalK: 1.05, roughK: 1.00, tint: 0xffffff },
  { name: 'rock',   macro: 16.5, detail: 1.10, heightK: 1.70, normalK: 1.35, roughK: 1.00, tint: 0xffffff },
  { name: 'gravel', macro: 5.2,  detail: 0.52, heightK: 1.55, normalK: 1.25, roughK: 1.00, tint: 0xffffff },
  { name: 'grass',  macro: 4.4,  detail: 0.46, heightK: 1.15, normalK: 0.95, roughK: 1.00, tint: 0xffffff },
  { name: 'root',   macro: 6.2,  detail: 0.58, heightK: 1.40, normalK: 1.20, roughK: 1.00, tint: 0xffffff },
  { name: 'mud',    macro: 8.0,  detail: 0.70, heightK: 0.80, normalK: 0.90, roughK: 1.00, tint: 0xffffff },
  { name: 'snow',   macro: 15.0, detail: 0.95, heightK: 0.50, normalK: 0.65, roughK: 1.00, tint: 0xffffff },
];

// A6 — HEIGHT -> NORMAL, CALIBRATED IN METRES.
// -----------------------------------------------------------------------------
// This used to be a bare `BUMP_SCALE` scalar folded into a magic 0.02, which
// meant a layer's apparent relief depended on its tile size: grass (4.4 m tile)
// and rock (16.5 m tile) with similar bump numbers ended up with world slopes
// almost 4x apart, and nobody could say what any of the numbers meant.
//
// The height fields are 0..1, so the physically correct gradient is
//     dH/dX = dh_per_texel * TEXELS_PER_TILE / TILE_METRES * RELIEF_METRES
// i.e. the only free parameter is RELIEF_METRES — the peak-to-peak world
// amplitude of that surface's height field.  Authored explicitly below.
//
// r3 A6 measured near-field relief GETTING WORSE as the camera closes (pixel /
// 32-px std ratio 1.43 at 1.5 m vs 1.69 at 10 m).  At 1.5 m the macro texture
// is magnified ~13x and carries nothing, so all near-field relief comes from
// the micro-detail array; both tables are lifted ~15-45% and uDetailAlbedo goes
// 0.22 -> 0.35 to match.
//
// OBSERVATION for a later round, not fixed here: measured on the packed set,
// GRASS's macro gradient averages |dH/dX| ~ 18, i.e. 86 deg, because genGrass
// lays blade strokes one texel wide and a one-texel step is a vertical wall to
// a central-difference gradient. That channel is already saturated against
// terrDecodeN's 0.995 clamp, so this table's value for grass is very nearly a
// no-op either way. The fix is wider strokes or a finer grass tile in the
// generator, not a smaller amplitude here — shrinking the amplitude to hide it
// would mis-calibrate the one table that is now supposed to mean metres.
//                        dirt  loam  rock  grav  grass root  mud   snow
const MACRO_RELIEF_M  = [ 0.70, 0.52, 1.75, 0.46, 0.32, 0.62, 0.48, 0.50 ];
const DETAIL_RELIEF_M = [ 0.054, 0.038, 0.105, 0.048, 0.024, 0.042, 0.030, 0.024 ];
// AO derivation strength per surface.
const AO_SCALE = [1.5, 1.6, 2.0, 2.2, 2.0, 1.7, 1.3, 0.9];

// Per-surface roughness FLOOR. Measured dry mineral ground is 0.85-0.95; the
// r2 review found a sunlit gravel bank blowing past L=230 because nothing here
// stopped the splat, the micro-detail modulation and the wetness branch from
// dragging it into near-mirror territory. Mud and snow are allowed lower
// because a puddle and an ice crust genuinely are smoother, but neither is
// allowed to be plastic.
//                       dirt  loam  rock  gravel grass root  mud   snow
const ROUGH_MIN =      [ 0.82, 0.85, 0.55, 0.82,  0.80, 0.58, 0.18, 0.22 ];

// P3 — LOAM ALBEDO LIFT.
// -----------------------------------------------------------------------------
// The r2 review measured the forest floor at a median luminance of 11-16/255,
// i.e. ~0.3% linear, where dry duff should be 10-15% albedo. It also warned
// that the layer may have been PRE-DARKENED to compensate for the RC-1 lighting
// collapse (hemisphere light at 0.14 against a sun at 3.51, N8AO multiplying
// direct sun), which another engineer is fixing in the same wave. Raising the
// albedo the whole way now would therefore overshoot once the indirect term
// comes back.
//
// This is that single re-tunable constant. It is an ADDITIVE lift in linear
// albedo, applied only to the LOAM layer, tinted the colour of dry duff.
// Measured mean loam albedo is ~4% linear; +0.030 puts it at ~6.8%, which is
// deliberately about half way to the 10-15% target.
//
// RE-TUNED FOR r3 (work order A4). RC-1 landed in r2 and was verified: 0.00%
// crushed pixels in all 16 shots, controlled highlight end, indirect chain
// restored. The forest floor is still the darkest thing in frame and it is the
// mechanism behind THREE separate r3 findings — the near-black forest floor,
// the cold blue tread (r3 A5/R13: a LOAM base at ~4% linear takes its hue
// wholesale from sky ambient), and the acid-green-grass-on-void value break.
// Raised the whole way to the authored target: 0.060 puts LOAM at ~10% linear
// albedo, the bottom of the 10-15% measured range for dry duff.
// Runtime hook: material.userData.setLayerLift( SurfaceIds.LOAM, v ).
const LOAM_ALBEDO_LIFT = 0.060;

// Four authored geological hues for the world-space macro-albedo field. These
// are LINEAR multipliers (not colours), chosen roughly luminance-preserving so
// the field re-hues the mountain without re-exposing it.
const GEO_TINTS = [
  [1.13, 0.98, 0.78],   // iron-stained ochre
  [0.90, 0.93, 1.00],   // cold grey schist
  [0.95, 1.03, 0.84],   // olive / lichen-shot
  [1.16, 0.92, 0.79],   // rust-red bedrock
];

// ---------------------------------------------------------------------------
// Colour helpers — author in sRGB hex, work in linear, encode back to sRGB
// bytes at pack time.  Doing it by hand (rather than via THREE.Color) keeps the
// result independent of the global ColorManagement flag.
// ---------------------------------------------------------------------------

function srgbDecode(c) {
  return c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4);
}
function srgbEncode(c) {
  if (c <= 0) return 0;
  if (c >= 1) return 1;
  return c <= 0.0031308 ? c * 12.92 : 1.055 * Math.pow(c, 1 / 2.4) - 0.055;
}
/** sRGB hex -> linear [r, g, b] */
function C(hex) {
  return [
    srgbDecode(((hex >> 16) & 255) / 255),
    srgbDecode(((hex >> 8) & 255) / 255),
    srgbDecode((hex & 255) / 255),
  ];
}

// ---------------------------------------------------------------------------
// Deterministic, *tileable* noise.
//
// Periodic Perlin: the lattice wraps at an integer period, so an fbm built with
// an integer base frequency and lacunarity 2 tiles perfectly in [0,1)^2.  That
// is non-negotiable — a seam in a 5 m tile is visible from 30 m away.
// ---------------------------------------------------------------------------

const PERM_SIZE = 1024;
const PERM_MASK = PERM_SIZE - 1;

function makePerlin(seed) {
  const rng = makeRng(seed);
  const perm = new Uint16Array(PERM_SIZE);
  for (let i = 0; i < PERM_SIZE; i++) perm[i] = i;
  for (let i = PERM_SIZE - 1; i > 0; i--) {
    const j = (rng() * (i + 1)) | 0;
    const t = perm[i]; perm[i] = perm[j]; perm[j] = t;
  }
  // 16 evenly spaced gradients — far fewer axis artefacts than the classic 4.
  const GX = new Float32Array(16), GY = new Float32Array(16);
  for (let i = 0; i < 16; i++) {
    const a = (i / 16) * Math.PI * 2;
    GX[i] = Math.cos(a); GY[i] = Math.sin(a);
  }

  return function perlin(x, y, px, py) {
    let ix = Math.floor(x), iy = Math.floor(y);
    const fx = x - ix, fy = y - iy;
    ix = ((ix % px) + px) % px;
    iy = ((iy % py) + py) % py;
    const ix1 = (ix + 1) % px, iy1 = (iy + 1) % py;

    const py0 = perm[iy & PERM_MASK], py1 = perm[iy1 & PERM_MASK];
    const h00 = perm[(ix + py0) & PERM_MASK] & 15;
    const h10 = perm[(ix1 + py0) & PERM_MASK] & 15;
    const h01 = perm[(ix + py1) & PERM_MASK] & 15;
    const h11 = perm[(ix1 + py1) & PERM_MASK] & 15;

    const n00 = GX[h00] * fx + GY[h00] * fy;
    const n10 = GX[h10] * (fx - 1) + GY[h10] * fy;
    const n01 = GX[h01] * fx + GY[h01] * (fy - 1);
    const n11 = GX[h11] * (fx - 1) + GY[h11] * (fy - 1);

    const u = fx * fx * fx * (fx * (fx * 6 - 15) + 10);
    const v = fy * fy * fy * (fy * (fy * 6 - 15) + 10);
    const a = n00 + (n10 - n00) * u;
    const b = n01 + (n11 - n01) * u;
    return (a + (b - a) * v) * 1.4; // roughly normalised to [-1, 1]
  };
}

/** fbm over the periodic lattice. u, v in tile space [0,1). Returns ~[-1,1]. */
function fbm(noise, u, v, freq, octaves, gain) {
  let sum = 0, amp = 1, norm = 0, f = freq;
  for (let o = 0; o < octaves; o++) {
    sum += noise(u * f, v * f, f, f) * amp;
    norm += amp;
    amp *= gain; f *= 2;
  }
  return sum / norm;
}

/** Anisotropic fbm: independent integer periods in u and v (still tileable). */
function fbmAniso(noise, u, v, fu, fv, octaves, gain) {
  let sum = 0, amp = 1, norm = 0, a = fu, b = fv;
  for (let o = 0; o < octaves; o++) {
    sum += noise(u * a, v * b, a, b) * amp;
    norm += amp;
    amp *= gain; a *= 2; b *= 2;
  }
  return sum / norm;
}

/** Ridged multifractal — the sharp-crested variant, used for rock. Returns 0..1. */
function ridged(noise, u, v, freq, octaves, gain) {
  let sum = 0, amp = 1, norm = 0, f = freq;
  for (let o = 0; o < octaves; o++) {
    let n = noise(u * f, v * f, f, f);
    n = 1 - Math.abs(n);
    n *= n;
    sum += n * amp;
    norm += amp;
    amp *= gain; f *= 2;
  }
  return sum / norm;
}

/** Periodic Worley/cellular. Fills and returns [f1, f2, cellIndex, cellRandom]. */
function makeWorley(seed, cells) {
  const rng = makeRng(seed);
  const n = cells * cells;
  const pts = new Float32Array(n * 2);
  const rnd = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    pts[i * 2] = 0.12 + rng() * 0.76;
    pts[i * 2 + 1] = 0.12 + rng() * 0.76;
    rnd[i] = rng();
  }
  const out = [0, 0, 0, 0];
  return function worley(u, v) {
    const fx = u * cells, fy = v * cells;
    const cx = Math.floor(fx), cy = Math.floor(fy);
    let f1 = 1e9, f2 = 1e9, id = 0;
    for (let dy = -1; dy <= 1; dy++) {
      const gy = cy + dy;
      const wy = ((gy % cells) + cells) % cells;
      for (let dx = -1; dx <= 1; dx++) {
        const gx = cx + dx;
        const wx = ((gx % cells) + cells) % cells;
        const k = wy * cells + wx;
        const ex = gx + pts[k * 2] - fx;
        const ey = gy + pts[k * 2 + 1] - fy;
        const d = Math.sqrt(ex * ex + ey * ey);
        if (d < f1) { f2 = f1; f1 = d; id = k; }
        else if (d < f2) { f2 = d; }
      }
    }
    out[0] = f1; out[1] = f2; out[2] = id; out[3] = rnd[id];
    return out;
  };
}

/**
 * Anisotropic stone scatter on a periodic lattice.
 *
 * Plain Worley measures an isotropic distance, so every site becomes a circle
 * of the same shape and the octave reads as a dot lattice — which is exactly
 * what the r2 autocorrelation found (a clean peak at ~20 cm pitch with a single
 * harmonic).  Here each cell carries its own rotation and an area-preserving
 * flatten, and the distance is measured in THAT cell's frame, so every stone is
 * a differently-oriented ellipse.
 *
 * Returns the shared array [ d, cellIndex, rndA, rndB ] where `d` is the
 * normalised distance in cell units (0 at the stone centre).
 */
function makeStones(seed, cells) {
  const rng = makeRng(seed);
  const n = cells * cells;
  const px = new Float32Array(n), py = new Float32Array(n);
  const ca = new Float32Array(n), sa = new Float32Array(n);
  const ax = new Float32Array(n), ay = new Float32Array(n);
  const r0 = new Float32Array(n), r1 = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    // Jitter is independent per octave because each octave gets its own seed —
    // superimposing octaves that share a jitter re-creates the lattice.
    px[i] = 0.10 + rng() * 0.80;
    py[i] = 0.10 + rng() * 0.80;
    const a = rng() * Math.PI;
    ca[i] = Math.cos(a); sa[i] = Math.sin(a);
    const e = 1 + rng() * 1.15;          // aspect 1.0 .. 2.15
    ax[i] = Math.sqrt(e); ay[i] = 1 / Math.sqrt(e);   // area preserving
    r0[i] = rng(); r1[i] = rng();
  }
  const out = [0, 0, 0, 0];
  return function stones(u, v) {
    const fx = u * cells, fy = v * cells;
    const cx = Math.floor(fx), cy = Math.floor(fy);
    let best = 1e9, id = 0;
    for (let dy = -1; dy <= 1; dy++) {
      const gy = cy + dy;
      const wy = ((gy % cells) + cells) % cells;
      for (let dx = -1; dx <= 1; dx++) {
        const gx = cx + dx;
        const wx = ((gx % cells) + cells) % cells;
        const k = wy * cells + wx;
        const ex = gx + px[k] - fx;
        const ey = gy + py[k] - fy;
        const rx = (ex * ca[k] + ey * sa[k]) / ax[k];
        const ry = (-ex * sa[k] + ey * ca[k]) / ay[k];
        const d = Math.sqrt(rx * rx + ry * ry);
        if (d < best) { best = d; id = k; }
      }
    }
    out[0] = best; out[1] = id; out[2] = r0[id]; out[3] = r1[id];
    return out;
  };
}

/**
 * Combine several scatter octaves by UNION (max), never by sum.
 *
 * Summing octaves makes the small grit ride on top of the large stones, so
 * every stone ends up the same apparent size and the surface is monodisperse.
 * Taking the max means a large stone genuinely occludes the grit under it,
 * which is what produces a real size distribution.  Each stone also gets its
 * own random burial depth, so the exposed cap size varies independently of the
 * stone size — that is what stops the scatter reading as one repeated pebble.
 *
 * `octs`: [{ fn, radius, amp, flat, bury }]  radius/amp in cell units / height.
 * `out` : [ height, coverage 0..1, cellIndex, rnd, octaveIndex ] — reused.
 */
function stoneUnion(octs, u, v, out) {
  let bh = -1e9, bc = 0, bid = 0, br = 0, bo = -1;
  for (let o = 0; o < octs.length; o++) {
    const oc = octs[o];
    const s = oc.fn(u, v);
    const r = oc.radius * (0.55 + s[3] * 0.90);
    if (s[0] >= r) continue;
    const q = 1 - s[0] / r;
    // Flattened dome — angular chips lying flat, not marbles.
    const dome = Math.pow(Math.sqrt(Math.max(0, 1 - (1 - q) * (1 - q))), oc.flat);
    const h = oc.amp * (dome - oc.bury * s[2]);
    if (h > bh) { bh = h; bc = Math.min(1, q * 3.0); bid = s[1]; br = s[2]; bo = o; }
  }
  out[0] = bh; out[1] = bc; out[2] = bid; out[3] = br; out[4] = bo;
  return out;
}

// ---------------------------------------------------------------------------
// Raster utilities on wrapped Float32 fields
// ---------------------------------------------------------------------------

/** Separable box blur with wrap-around. `tmp` must be a scratch of the same size. */
function boxBlurWrap(src, dst, tmp, size, radius) {
  const inv = 1 / (radius * 2 + 1);
  for (let y = 0; y < size; y++) {
    const row = y * size;
    let acc = 0;
    for (let k = -radius; k <= radius; k++) acc += src[row + (((k % size) + size) % size)];
    for (let x = 0; x < size; x++) {
      tmp[row + x] = acc * inv;
      const out = row + ((((x - radius) % size) + size) % size);
      const inn = row + ((((x + radius + 1) % size) + size) % size);
      acc += src[inn] - src[out];
    }
  }
  for (let x = 0; x < size; x++) {
    let acc = 0;
    for (let k = -radius; k <= radius; k++) acc += tmp[((((k % size) + size) % size) * size) + x];
    for (let y = 0; y < size; y++) {
      dst[y * size + x] = acc * inv;
      const out = ((((y - radius) % size) + size) % size) * size + x;
      const inn = ((((y + radius + 1) % size) + size) % size) * size + x;
      acc += tmp[inn] - tmp[out];
    }
  }
}

/**
 * Rasterise a soft tapered stroke with wrap-around, calling
 * cb(index, coverage 0..1, tAlongStroke 0..1, distFromCentre 0..1).
 * Used for grass blades, pine needles, roots and mud ruts.
 */
function stroke(size, x0, y0, x1, y1, w0, w1, cb) {
  const dx = x1 - x0, dy = y1 - y0;
  const len = Math.sqrt(dx * dx + dy * dy);
  const steps = Math.max(1, Math.ceil(len * 1.5));
  for (let s = 0; s <= steps; s++) {
    const t = s / steps;
    const cx = x0 + dx * t, cy = y0 + dy * t;
    const w = w0 + (w1 - w0) * t;
    if (w <= 0.05) continue;
    const ri = Math.ceil(w);
    const bx = Math.round(cx), by = Math.round(cy);
    for (let oy = -ri; oy <= ri; oy++) {
      const iy = (((by + oy) % size) + size) % size;
      const rowo = iy * size;
      for (let ox = -ri; ox <= ri; ox++) {
        const d = Math.sqrt(ox * ox + oy * oy) / w;
        if (d > 1) continue;
        const ix = (((bx + ox) % size) + size) % size;
        cb(rowo + ix, 1 - d * d, t, d);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Surface synthesis
//
// Each generator fills the shared working fields:
//   F.h    height     0..1
//   F.r/g/b           linear albedo
//   F.ro   roughness  0..1
//   F.ao   authored occlusion (multiplied into the height-derived AO)
// ---------------------------------------------------------------------------

function newFields(N) {
  return {
    h: new Float32Array(N), r: new Float32Array(N), g: new Float32Array(N),
    b: new Float32Array(N), ro: new Float32Array(N), ao: new Float32Array(N),
    t0: new Float32Array(N), t1: new Float32Array(N), t2: new Float32Array(N),
    t3: new Float32Array(N),
  };
}

function setCol(F, i, col, k) {
  F.r[i] = col[0] * k; F.g[i] = col[1] * k; F.b[i] = col[2] * k;
}
function mixCol(F, i, col, t, k) {
  F.r[i] += (col[0] * k - F.r[i]) * t;
  F.g[i] += (col[1] * k - F.g[i]) * t;
  F.b[i] += (col[2] * k - F.b[i]) * t;
}

// --- 0: DIRT — dry compacted trail hardpack with embedded stones ------------
function genDirt(S, F, seed) {
  const n = makePerlin(subSeed(seed, 'dirt-n'));
  const n2 = makePerlin(subSeed(seed, 'dirt-n2'));
  const cracks = makeWorley(subSeed(seed, 'dirt-crack'), 9);

  // Three superimposed scatter octaves instead of one 26-cell dot lattice.
  // Dirt tiles at 9.5 m, so cells -> stone pitch:  38 = 25 cm, 118 = 8 cm,
  // 264 = 3.6 cm.  The third octave deliberately stops at 3.6 cm rather than
  // the 2.5 cm the work order asks for: at 512 texels over a 9.5 m tile,
  // 2.5 cm is 1.3 texels, i.e. below Nyquist, so it would alias rather than
  // resolve.  Everything below 3 cm is carried by the grain fbm here and by
  // the micro-detail array, which tiles at 0.78 m (1.5 mm/texel).
  const stoneOct = [
    { fn: makeStones(subSeed(seed, 'dirt-s25'), 38),  radius: 0.26, amp: 0.30, flat: 0.55, bury: 0.60 },
    { fn: makeStones(subSeed(seed, 'dirt-s08'), 118), radius: 0.34, amp: 0.145, flat: 0.68, bury: 0.62 },
    { fn: makeStones(subSeed(seed, 'dirt-s03'), 264), radius: 0.42, amp: 0.065, flat: 0.80, bury: 0.66 },
  ];
  const sOut = [0, 0, 0, 0, 0];

  const cLow = C(0x3b2c1d), cMid = C(0x6b5238), cHigh = C(0x8f7351);
  // A3: the STONES embedded in the hardpack are rock and came down with the rest
  // of the rock (x0.62 linear). The soil ramp (cLow/cMid/cHigh) is deliberately
  // NOT touched — DIRT is the warm hardpack the tread is made of, and A4/A5 want
  // that warmer and lighter, not darker.
  const cStone = C(0x6f685e), cStoneDark = C(0x4b473f), cIron = C(0x7a4f2c);

  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = y * S + x;

      // Domain-warped fbm gives lumpy, non-gridded relief. `bed` is the smooth
      // soil datum the stones are measured from; `fines` is that datum plus its
      // own grain.
      const wx = fbm(n2, u, v, 3, 3, 0.5) * 0.055;
      const wy = fbm(n2, u + 0.31, v + 0.17, 3, 3, 0.5) * 0.055;
      const bed = fbm(n, u + wx, v + wy, 6, 6, 0.5) * 0.5 + 0.5;
      const grain = fbm(n2, u, v, 48, 3, 0.55) * 0.5 + 0.5;
      const fines = bed * 0.78 + grain * 0.22;

      // Stones sitting proud of the surface: UNION of three size octaves
      // against the fines, so a 25 cm stone buries the 8 cm grit beside it and
      // a deeply-buried stone disappears into the soil entirely.
      stoneUnion(stoneOct, u, v, sOut);
      const stoneTop = bed + sOut[0];
      let h = fines, stoneM = 0, stoneRnd = 0;
      if (sOut[0] > -1e8 && stoneTop > fines) {
        h = stoneTop;
        // Fade the stone's albedo in over the last 1.5 cm of its emergence, so
        // a barely-proud stone is a stain in the dirt, not a hard-edged disc.
        stoneM = sOut[1] * clamp01((stoneTop - fines) / 0.015);
        stoneRnd = sOut[3];
      }

      // Dried shrinkage cracks along the cell boundaries.
      const cw = cracks(u, v);
      const crack = clamp01(1 - (cw[1] - cw[0]) / 0.055);
      const crackM = crack * crack * (0.55 + 0.45 * (fbm(n2, u, v, 12, 2, 0.5) * 0.5 + 0.5));
      h -= crackM * 0.16;

      h = clamp01(h);
      F.h[i] = h;

      // Albedo: dark in the pits, warm and dusty on the crests.
      const t = smoothstep(0.30, 0.78, h);
      setCol(F, i, cLow, 1);
      mixCol(F, i, cMid, smoothstep(0.18, 0.62, h), 1);
      mixCol(F, i, cHigh, t * 0.85, 1);
      // Iron staining at low frequency, so big areas are never one flat brown.
      const iron = clamp01(fbm(n2, u + 0.7, v + 0.2, 4, 3, 0.5) * 0.5 + 0.5);
      mixCol(F, i, cIron, smoothstep(0.62, 0.95, iron) * 0.35, 1);
      if (stoneM > 0) {
        const sr = stoneRnd;
        mixCol(F, i, sr > 0.5 ? cStone : cStoneDark, stoneM * (0.55 + sr * 0.4), 0.85 + sr * 0.35);
      }
      mixCol(F, i, cLow, crackM * 0.8, 0.55);

      // Dry hardpack. Stones read slightly smoother than the fines around them
      // but never anywhere near glossy — see uLayerRoughMin[DIRT] = 0.82.
      F.ro[i] = 0.95 - stoneM * 0.10 - t * 0.05 + crackM * 0.03;
      F.ao[i] = 1 - crackM * 0.35;
    }
  }
}

// --- 1: LOAM — dark forest floor, needle litter, moss ----------------------
function genLoam(S, F, seed) {
  const n = makePerlin(subSeed(seed, 'loam-n'));
  const n2 = makePerlin(subSeed(seed, 'loam-n2'));
  const rng = makeRng(subSeed(seed, 'loam-litter'));

  const cSoil = C(0x241a12), cSoil2 = C(0x3d2c1c), cHumus = C(0x533c24);
  const cMoss = C(0x3f4c22), cMossLt = C(0x5d6b31);
  const cNeedle = C(0x6a5030), cNeedle2 = C(0x4a3a22), cNeedleGreen = C(0x4e5a2c);

  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = y * S + x;
      const wx = fbm(n2, u, v, 2, 3, 0.5) * 0.09;
      let h = fbm(n, u + wx, v, 4, 6, 0.52) * 0.5 + 0.5;
      h = h * 0.8 + (fbm(n2, u, v, 40, 3, 0.5) * 0.5 + 0.5) * 0.2;
      F.h[i] = clamp01(h);

      const t = smoothstep(0.25, 0.8, h);
      setCol(F, i, cSoil, 1);
      mixCol(F, i, cSoil2, smoothstep(0.15, 0.6, h), 1);
      mixCol(F, i, cHumus, t * 0.7, 1);

      // Moss creeping in low-frequency patches.
      const moss = clamp01(fbm(n2, u + 0.4, v + 0.9, 5, 3, 0.5) * 0.5 + 0.5);
      const mossM = smoothstep(0.55, 0.82, moss);
      mixCol(F, i, cMoss, mossM * 0.75, 1);
      mixCol(F, i, cMossLt, mossM * t * 0.4, 1);
      F.h[i] = clamp01(F.h[i] + mossM * 0.06);

      F.ro[i] = 0.97 - mossM * 0.07;
      F.ao[i] = 1;
    }
  }

  // Pine needles: thousands of short strokes, direction from a flow field.
  const needles = Math.round(S * S / 230);
  for (let k = 0; k < needles; k++) {
    const cx = rng() * S, cy = rng() * S;
    const flow = fbm(n2, cx / S, cy / S, 3, 2, 0.5) * Math.PI * 2;
    const a = flow + (rng() - 0.5) * 2.0;
    const len = (0.030 + rng() * 0.042) * S;
    const w = Math.max(0.75, S * 0.0032);
    const roll = rng();
    const col = roll < 0.18 ? cNeedleGreen : (roll < 0.6 ? cNeedle : cNeedle2);
    const shade = 0.7 + rng() * 0.6;
    stroke(S, cx, cy, cx + Math.cos(a) * len, cy + Math.sin(a) * len, w, w * 0.5,
      (i, cov) => {
        const c = cov * 0.9;
        mixCol(F, i, col, c, shade);
        F.h[i] = clamp01(F.h[i] + c * 0.10);
        F.ro[i] -= c * 0.10;
      });
  }
  // A handful of twigs for larger-scale interest.
  for (let k = 0; k < Math.round(S / 12); k++) {
    const cx = rng() * S, cy = rng() * S;
    const a = rng() * Math.PI * 2;
    const len = (0.10 + rng() * 0.16) * S;
    const w = Math.max(1.2, S * 0.006);
    stroke(S, cx, cy, cx + Math.cos(a) * len, cy + Math.sin(a) * len, w, w * 0.6,
      (i, cov) => {
        mixCol(F, i, cNeedle, cov * 0.85, 0.8);
        F.h[i] = clamp01(F.h[i] + cov * 0.22);
      });
  }
}

// --- 2: ROCK — jointed alpine granite/schist with lichen -------------------
function genRock(S, F, seed) {
  const n = makePerlin(subSeed(seed, 'rock-n'));
  const n2 = makePerlin(subSeed(seed, 'rock-n2'));
  const n3 = makePerlin(subSeed(seed, 'rock-n3'));
  // Joint spacing: rock tiles at 16.5 m, so 11 cells ~= 1.5 m blocks and 24
  // cells ~= 0.7 m. Any coarser and the polygon network becomes a signature
  // you can spot repeating across the mountain.
  const joints = makeWorley(subSeed(seed, 'rock-joint'), 11);
  const joints2 = makeWorley(subSeed(seed, 'rock-joint2'), 24);

  // A3 — ROCK ALBEDO, HALVED.
  // r3 measured the foreground rock/scree in r3_02 at 201/184/168 = 79% sRGB,
  // brighter than most snow references. That single error is why the adaptive
  // exposure is dragged up and the far peak reads dark — a composition lens
  // filed it as "inverted aerial perspective" and wanted the shadow cascade and
  // the fog curve rewritten (rejected as R7; the far peak is fine, the near
  // rock is 2x too bright).
  // The whole ramp comes down by ~2x in LINEAR albedo, dark end included, so
  // internal contrast is held: cLight/cDark was 11.9:1, it is now 10.2:1.
  // Bulk sunlit rock now sits at 0.36-0.44 in sRGB units (was 0.60-0.73).
  const cDark = C(0x232220), cMid = C(0x494741), cLight = C(0x6f6c66);
  const cWarm = C(0x594834), cLichen = C(0x656d49), cLichenLt = C(0x8b9079);
  const cQuartz = C(0x8e8b84);

  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = y * S + x;

      // Warped ridged noise reads as fractured rock rather than clouds.
      const wx = fbm(n3, u, v, 2, 3, 0.5) * 0.10;
      const wy = fbm(n3, u + 0.53, v + 0.29, 2, 3, 0.5) * 0.10;
      let h = ridged(n, u + wx, v + wy, 3, 6, 0.55);
      h = Math.pow(h, 1.25);

      // Bedding planes / stratification.
      const strat = Math.sin((v * 7 + fbm(n2, u, v, 3, 3, 0.5) * 1.6) * Math.PI * 2);
      h += strat * 0.045;

      // Two scales of joint (crack) network cut into the surface. Kept narrow
      // and shallow — real granite is jointed, but the joints are hairlines,
      // not a dried-mud pattern.
      const j1 = joints(u, v);
      const c1 = clamp01(1 - (j1[1] - j1[0]) / 0.045);
      const j2 = joints2(u, v);
      const c2 = clamp01(1 - (j2[1] - j2[0]) / 0.032);
      const crack = clamp01(c1 * c1 * 0.85 + c2 * c2 * 0.45);
      h -= crack * 0.19;

      // Micro chipping.
      h += fbm(n2, u, v, 40, 3, 0.55) * 0.055;
      h = clamp01(h * 0.55 + 0.35);
      F.h[i] = h;

      const t = smoothstep(0.28, 0.78, h);
      setCol(F, i, cDark, 1);
      mixCol(F, i, cMid, smoothstep(0.12, 0.62, h), 1);
      mixCol(F, i, cLight, t * t * 0.85, 1);

      // Broad tonal blotching so a cliff is never one flat sheet of concrete.
      const blot = fbm(n3, u * 0.7 + 0.11, v * 0.7 + 0.44, 3, 4, 0.55);
      const bk = 1 + blot * 0.34;
      F.r[i] *= bk; F.g[i] *= bk; F.b[i] *= bk;

      // Mineral banding: warm iron on some strata, pale quartz on others.
      const band = clamp01(fbm(n2, u * 0.5, v, 4, 3, 0.5) * 0.5 + 0.5);
      mixCol(F, i, cWarm, smoothstep(0.52, 0.90, band) * 0.55, 1);
      const qz = clamp01(fbm(n3, u + 0.8, v + 0.4, 16, 2, 0.5) * 0.5 + 0.5);
      mixCol(F, i, cQuartz, smoothstep(0.80, 0.97, qz) * 0.5, 1);

      // Lichen — the single biggest cue that rock is real rock.
      const li = clamp01(fbm(n3, u + 0.15, v + 0.62, 7, 4, 0.55) * 0.5 + 0.5);
      const liM = smoothstep(0.56, 0.72, li) * (1 - crack * 0.7) * smoothstep(0.35, 0.7, h);
      mixCol(F, i, cLichen, liM * 0.8, 1);
      const liSpeck = clamp01(fbm(n2, u * 1.3 + 0.2, v * 1.3, 34, 2, 0.5) * 0.5 + 0.5);
      mixCol(F, i, cLichenLt, liM * smoothstep(0.62, 0.9, liSpeck) * 0.7, 1);

      mixCol(F, i, cDark, crack * 0.55, 0.5);

      F.ro[i] = 0.58 + crack * 0.20 + liM * 0.34 - t * 0.06;
      F.ao[i] = 1 - crack * 0.40;
    }
  }
}

// --- 3: GRAVEL — loose scree at three pebble scales ------------------------
function genGravel(S, F, seed) {
  const n = makePerlin(subSeed(seed, 'grv-n'));
  // Scree tiles at 5.2 m, so cells -> stone pitch: 21 = 25 cm, 65 = 8 cm,
  // 208 = 2.5 cm (2.5 texels/cell at 512 — right at the resolvable limit).
  // These are the three octaves the r2 autocorrelation asked for.  They are
  // combined by UNION, never by sum: the previous code added a tiny, a small
  // and a big Worley dome together, which is precisely what produced one
  // monodisperse dot lattice at ~20 cm with a single harmonic.
  const stoneOct = [
    { fn: makeStones(subSeed(seed, 'grv-s25'), 21),  radius: 0.40, amp: 0.34, flat: 0.50, bury: 0.45 },
    { fn: makeStones(subSeed(seed, 'grv-s08'), 65),  radius: 0.44, amp: 0.165, flat: 0.62, bury: 0.50 },
    { fn: makeStones(subSeed(seed, 'grv-s02'), 208), radius: 0.46, amp: 0.075, flat: 0.78, bury: 0.55 },
  ];
  const sOut = [0, 0, 0, 0, 0];

  // A3, continued. The r3 measurement that produced "rock albedo is ~2x too
  // high" was taken on FOREGROUND SCREE, which is this layer, not genLoam's
  // neighbour and not genRock. The work order names genRock's constants, so
  // both are brought down by the same factor — halving genRock alone would have
  // left the actual measured pixels untouched and produced a scree field
  // brighter than the cliff above it.
  // Linear albedo halved throughout; the palette now runs 0.24-0.47 sRGB with
  // the brightest (rarest) chip at 0.47, against a target of 0.35-0.45 for
  // sunlit rock.
  const fines = C(0x282521);
  const palette = [C(0x646058), C(0x4e4a43), C(0x70695f), C(0x3d3a35), C(0x644f39), C(0x79766c)];
  const PN = palette.length;

  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = y * S + x;

      // The bed of fines the stones are half-buried in.
      const bed = 0.16 + (fbm(n, u, v, 6, 4, 0.5) * 0.5 + 0.5) * 0.14;
      setCol(F, i, fines, 0.9 + (fbm(n, u, v, 30, 2, 0.5) * 0.5 + 0.5) * 0.3);
      F.ro[i] = 0.96;
      let occ = 1;
      let h = bed;

      stoneUnion(stoneOct, u, v, sOut);
      const top = bed + sOut[0];
      if (sOut[0] > -1e8 && top > bed) {
        h = top;
        const oct = sOut[4];
        const rnd = sOut[3];
        // Emergence: how far this stone's cap stands above the fines. It drives
        // the albedo blend, so stones bury at every depth and the visible size
        // distribution is broader than the geometric one.
        const emerge = clamp01((top - bed) / (0.020 + 0.035 * oct));
        const cov = Math.min(1, sOut[1] * 1.15) * emerge;
        const pal = palette[((sOut[2] * (oct === 0 ? 5 : oct === 1 ? 3 : 7)) % PN) | 0];
        mixCol(F, i, pal, cov, 0.70 + rnd * 0.60);
        // Per-stone face texture, keyed to the stone so neighbours differ.
        const speck = fbm(n, u, v, 60, 2, 0.5) * 0.5 + 0.5;
        const k = (0.92 + speck * 0.16) * (0.94 + rnd * 0.12);
        F.r[i] += (F.r[i] * k - F.r[i]) * cov;
        F.g[i] += (F.g[i] * k - F.g[i]) * cov;
        F.b[i] += (F.b[i] * k - F.b[i]) * cov;
        h += (speck - 0.5) * 0.02 * cov;
        // Dry scree: a chip face is a little less rough than the dust between
        // the chips, but nothing here is remotely glossy.
        F.ro[i] = 0.96 - cov * (0.10 + rnd * 0.06);
        occ = Math.min(occ, 1.0 - cov * (0.30 + 0.25 * (2 - oct) / 2));
      }

      F.h[i] = clamp01(h);
      F.ao[i] = occ;
    }
  }
}

// --- 4: GRASS — alpine tussock over dark soil ------------------------------
function genGrass(S, F, seed) {
  const n = makePerlin(subSeed(seed, 'grass-n'));
  const n2 = makePerlin(subSeed(seed, 'grass-n2'));
  const rng = makeRng(subSeed(seed, 'grass-blades'));

  const cSoil = C(0x241f16), cSoil2 = C(0x362e20);
  const cBase = C(0x2c3a1a), cMid = C(0x4e6224), cTip = C(0x7d8f3c);
  const cDry = C(0x9c8f45), cDry2 = C(0xb0a05a), cRust = C(0x7a5c2c);

  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = y * S + x;
      const clump = clamp01(fbm(n, u, v, 6, 4, 0.5) * 0.5 + 0.5);
      F.h[i] = clump * 0.30;
      setCol(F, i, cSoil, 1);
      mixCol(F, i, cSoil2, clamp01(fbm(n2, u, v, 18, 2, 0.5) * 0.5 + 0.5), 1);
      // A base wash of green so bare soil never dominates between blades.
      mixCol(F, i, cBase, smoothstep(0.35, 0.75, clump) * 0.55, 1);
      F.ro[i] = 0.94;
      F.ao[i] = 1 - smoothstep(0.30, 0.0, clump) * 0.15;
    }
  }

  // Blades. Direction follows a coherent flow field; dryness is driven by the
  // same low-frequency noise so colour variation is spatially organised rather
  // than salt-and-pepper.
  const blades = Math.round(S * S / 42);
  for (let k = 0; k < blades; k++) {
    const cx = rng() * S, cy = rng() * S;
    const u = cx / S, v = cy / S;
    const clump = clamp01(fbm(n, u, v, 6, 4, 0.5) * 0.5 + 0.5);
    if (rng() > 0.25 + clump * 0.9) continue;
    const dryness = clamp01(fbm(n2, u + 0.5, v + 0.3, 4, 3, 0.5) * 0.5 + 0.5);
    const flow = fbm(n2, u, v, 3, 2, 0.5) * Math.PI * 1.6;
    const a = flow + (rng() - 0.5) * 1.5;
    const len = (0.022 + rng() * 0.045) * S * (0.6 + clump * 0.8);
    const bend = (rng() - 0.5) * 0.9;
    const w = Math.max(0.8, S * 0.0038) * (0.7 + rng() * 0.7);
    const dry = rng() < dryness * 0.85;
    const tipCol = dry ? (rng() < 0.4 ? cDry2 : cDry) : cTip;
    const midCol = dry ? cRust : cMid;
    const shade = 0.72 + rng() * 0.6;
    // Two segments so the blade curves instead of being a stick.
    const mx = cx + Math.cos(a) * len * 0.55, my = cy + Math.sin(a) * len * 0.55;
    const ex = mx + Math.cos(a + bend) * len * 0.5, ey = my + Math.sin(a + bend) * len * 0.5;
    stroke(S, cx, cy, mx, my, w, w * 0.75, (i, cov, t) => {
      const c = cov * 0.95;
      mixCol(F, i, midCol, c * (0.35 + t * 0.5), shade * (0.55 + t * 0.35));
      F.h[i] = clamp01(F.h[i] + c * 0.16 * (0.4 + t * 0.6));
      F.ro[i] -= c * 0.10;
      F.ao[i] -= c * 0.08;
    });
    stroke(S, mx, my, ex, ey, w * 0.75, w * 0.18, (i, cov, t) => {
      const c = cov * 0.95;
      mixCol(F, i, tipCol, c * (0.55 + t * 0.4), shade * (0.85 + t * 0.35));
      F.h[i] = clamp01(F.h[i] + c * 0.24 * (0.6 + t * 0.4));
      F.ro[i] -= c * 0.14;
    });
  }

  // A scatter of tiny alpine flowers. Reads as life, not as noise.
  const cFlower = [C(0xd9d3c0), C(0xc9b8d8), C(0xd8c46a)];
  for (let k = 0; k < Math.round(S / 6); k++) {
    const cx = rng() * S, cy = rng() * S;
    const col = cFlower[(rng() * cFlower.length) | 0];
    const r = Math.max(1.0, S * 0.0035) * (0.8 + rng() * 0.8);
    stroke(S, cx, cy, cx + 0.2, cy + 0.2, r, r, (i, cov) => {
      mixCol(F, i, col, cov * 0.9, 1);
      F.ro[i] += cov * 0.05;
    });
  }
}

// --- 5: ROOT — exposed root network polished by tyres ----------------------
function genRoot(S, F, seed) {
  const n = makePerlin(subSeed(seed, 'root-n'));
  const n2 = makePerlin(subSeed(seed, 'root-n2'));
  const rng = makeRng(subSeed(seed, 'root-net'));

  const cSoil = C(0x2a2016), cSoil2 = C(0x40301e);
  const cBark = C(0x4a3822), cBarkLt = C(0x6d5535), cWorn = C(0x9a7f57), cWet = C(0x33241a);

  // Soil bed.
  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = y * S + x;
      let h = fbm(n, u, v, 5, 5, 0.5) * 0.5 + 0.5;
      h = h * 0.72 + (fbm(n2, u, v, 44, 3, 0.5) * 0.5 + 0.5) * 0.28;
      F.h[i] = h * 0.30;
      setCol(F, i, cSoil, 1);
      mixCol(F, i, cSoil2, smoothstep(0.2, 0.8, h), 1);
      F.ro[i] = 0.96;
      F.ao[i] = 1;
    }
  }

  // Root strands: smooth random walks that cross the tile and wrap.
  const rootMask = F.t0;
  rootMask.fill(0);
  const strands = 9;
  for (let s = 0; s < strands; s++) {
    let px = rng() * S, py = rng() * S;
    let a = rng() * Math.PI * 2;
    let turn = 0;
    const baseW = (0.016 + rng() * 0.028) * S;
    const segs = 40 + ((rng() * 24) | 0);
    const segLen = S * 0.055;
    const wearAmt = rng();
    for (let k = 0; k < segs; k++) {
      turn += (rng() - 0.5) * 0.32;
      turn *= 0.86;
      a += turn;
      const nx = px + Math.cos(a) * segLen;
      const ny = py + Math.sin(a) * segLen;
      const tp = k / segs;
      const w0 = baseW * (0.55 + 0.45 * Math.sin(tp * Math.PI));
      const w1 = baseW * (0.55 + 0.45 * Math.sin(((k + 1) / segs) * Math.PI));
      stroke(S, px, py, nx, ny, w0, w1, (i, cov, t, d) => {
        // Rounded cross-section: full height at the centre, down to the soil.
        const prof = Math.sqrt(Math.max(0, 1 - d * d));
        if (prof <= rootMask[i]) return;
        rootMask[i] = prof;
        F.h[i] = 0.28 + prof * 0.62;
        const wear = clamp01((prof - 0.55) / 0.45) * (0.35 + wearAmt * 0.65);
        setCol(F, i, cBark, 1);
        mixCol(F, i, cBarkLt, prof * 0.6, 1);
        mixCol(F, i, cWorn, wear * 0.75, 1);
        F.ro[i] = 0.88 - wear * 0.34;
      });
      px = nx; py = ny;
    }
  }

  // Bark grain: longitudinal striations, only where a root exists.
  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const i = y * S + x;
      const m = rootMask[i];
      if (m <= 0) continue;
      const u = x / S;
      const grain = fbmAniso(n2, u, v, 8, 64, 3, 0.5);
      const fine = fbm(n, u, v, 70, 2, 0.5);
      F.h[i] = clamp01(F.h[i] + grain * 0.05 * m + fine * 0.02);
      const k = 1 + grain * 0.18 + fine * 0.06;
      F.r[i] *= k; F.g[i] *= k; F.b[i] *= k;
    }
  }

  // Occlusion + damp soil in the crevice where root meets ground.
  boxBlurWrap(rootMask, F.t2, F.t1, S, Math.max(1, (S * 0.012) | 0));
  for (let i = 0; i < S * S; i++) {
    const shadow = clamp01(F.t2[i] - rootMask[i]);
    F.ao[i] = clamp01(1 - shadow * 2.2);
    if (rootMask[i] <= 0) mixCol(F, i, cWet, clamp01(shadow * 1.4), 1);
  }
}

// --- 6: MUD — churned, wet, puddled ---------------------------------------
function genMud(S, F, seed) {
  const n = makePerlin(subSeed(seed, 'mud-n'));
  const n2 = makePerlin(subSeed(seed, 'mud-n2'));
  const rng = makeRng(subSeed(seed, 'mud-ruts'));

  const cWet = C(0x1d1610), cMid = C(0x3a2b1d), cDry = C(0x6d573c), cSheen = C(0x2a2419);

  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = y * S + x;
      const wx = fbm(n2, u, v, 2, 3, 0.5) * 0.12;
      // Big smooth basins (where water pools) plus churned lumps on top.
      const basin = fbm(n, u + wx, v, 3, 3, 0.5) * 0.5 + 0.5;
      const churn = fbm(n2, u, v, 11, 5, 0.55) * 0.5 + 0.5;
      let h = basin * 0.62 + churn * 0.38;
      h += (fbm(n, u, v, 50, 2, 0.5) * 0.5 + 0.5) * 0.06;
      h = clamp01(h);
      F.h[i] = h;

      const puddle = 1 - smoothstep(0.30, 0.46, h);
      const dryT = smoothstep(0.55, 0.85, h);

      setCol(F, i, cMid, 1);
      mixCol(F, i, cDry, dryT * 0.8, 1);
      mixCol(F, i, cWet, puddle * 0.9, 1);
      mixCol(F, i, cSheen, puddle * 0.4, 1);

      // Roughness is the whole story for mud: near-mirror in the puddles.
      F.ro[i] = 0.72 - puddle * 0.62 + dryT * 0.20;
      F.ao[i] = 1 - puddle * 0.18;
    }
  }

  // Tyre ruts pressed through the mud, with raised lips.
  for (let k = 0; k < Math.round(S / 9); k++) {
    const cx = rng() * S, cy = rng() * S;
    const a = Math.PI * 0.5 + (rng() - 0.5) * 0.9;
    const len = (0.2 + rng() * 0.5) * S;
    const w = Math.max(1.5, S * 0.012) * (0.6 + rng() * 0.9);
    const deep = 0.10 + rng() * 0.16;
    stroke(S, cx, cy, cx + Math.cos(a) * len, cy + Math.sin(a) * len, w, w * 0.8,
      (i, cov, t, d) => {
        const prof = Math.cos(d * Math.PI) * 0.5 + 0.5;
        const lip = Math.max(0, Math.sin(d * Math.PI) - 0.4);
        F.h[i] = clamp01(F.h[i] - prof * deep + lip * deep * 0.5);
        mixCol(F, i, cWet, prof * 0.5, 1);
        F.ro[i] -= prof * 0.25;
      });
  }
  for (let i = 0; i < S * S; i++) F.ro[i] = clamp(F.ro[i], 0.05, 1);
}

// --- 7: SNOW — wind-packed alpine snow ------------------------------------
function genSnow(S, F, seed) {
  const n = makePerlin(subSeed(seed, 'snow-n'));
  const n2 = makePerlin(subSeed(seed, 'snow-n2'));
  const rng = makeRng(subSeed(seed, 'snow-sparkle'));

  const cWhite = C(0xe8ebee), cWarm = C(0xf2eee6), cBlue = C(0xb6c3d6), cDeep = C(0x8fa0ba);

  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = y * S + x;
      // Sastrugi: wind-carved ridges, anisotropic.
      // A8 (r5): was 12:2, i.e. 6:1. Sastrugi genuinely are directional, but
      // farProfile() compiles TERRAIN_TILE_BREAK out, so beyond ~110 m there is
      // no per-cell rotation left and a 6:1 ridge field becomes one world-Z
      // comb across the whole distant mountain — which is exactly what the
      // acceptance criterion forbids. 2.75:1 keeps the wind read at close
      // range, where the rotation is still there to decorrelate it.
      const sast = fbmAniso(n, u, v, 11, 4, 4, 0.5);
      const drift = fbm(n2, u, v, 3, 4, 0.5);
      const grain = fbm(n, u, v, 46, 3, 0.5);
      let h = 0.5 + sast * 0.26 + drift * 0.22 + grain * 0.05;
      // Crust fractures.
      const crust = Math.abs(fbm(n2, u + 0.4, v + 0.7, 9, 3, 0.5));
      const crustM = smoothstep(0.02, 0.0, crust);
      h -= crustM * 0.08;
      h = clamp01(h);
      F.h[i] = h;

      const t = smoothstep(0.32, 0.78, h);
      setCol(F, i, cDeep, 1);
      mixCol(F, i, cBlue, smoothstep(0.15, 0.55, h), 1);
      mixCol(F, i, cWhite, t, 1);
      mixCol(F, i, cWarm, t * t * 0.55, 1);

      F.ro[i] = 0.46 - t * 0.14 + crustM * 0.15;
      F.ao[i] = 1;
    }
  }
  // Sparkle: sparse very smooth micro-facets. Under IBL these catch the sun
  // and read as ice crystals rather than as noise.
  const sparkles = Math.round(S * S / 900);
  for (let k = 0; k < sparkles; k++) {
    const i = (rng() * S * S) | 0;
    F.ro[i] = 0.06;
    F.r[i] *= 1.15; F.g[i] *= 1.15; F.b[i] *= 1.18;
  }
}

const GENERATORS = [genDirt, genLoam, genRock, genGravel, genGrass, genRoot, genMud, genSnow];

// ---------------------------------------------------------------------------
// Packing into DataArrayTextures
// ---------------------------------------------------------------------------

/**
 * Percentile-normalise a height field to [0.02, 0.98] with a mean pulled to
 * 0.5.  This matters far more than it looks: the splat blend compares layer
 * heights against each other, so if (say) rock's field averages 0.62 and
 * grass's averages 0.18, rock wins every contested fragment regardless of the
 * coverage weights.  Normalising makes the per-layer contrast knob
 * (uLayerHeightK) behave as contrast rather than as a hidden priority.
 */
function normaliseHeight(src, dst, N) {
  const BINS = 256;
  const hist = new Uint32Array(BINS);
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < N; i++) {
    const v = src[i];
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  const span = Math.max(1e-5, hi - lo);
  for (let i = 0; i < N; i++) {
    let b = ((src[i] - lo) / span * (BINS - 1)) | 0;
    if (b < 0) b = 0; else if (b >= BINS) b = BINS - 1;
    hist[b]++;
  }
  // 0.5 / 99.5 percentile clip so a handful of stroke spikes cannot squash
  // the whole range.
  const cut = Math.max(1, (N * 0.005) | 0);
  let acc = 0, bLo = 0, bHi = BINS - 1;
  for (let b = 0; b < BINS; b++) { acc += hist[b]; if (acc >= cut) { bLo = b; break; } }
  acc = 0;
  for (let b = BINS - 1; b >= 0; b--) { acc += hist[b]; if (acc >= cut) { bHi = b; break; } }
  if (bHi <= bLo) { bLo = 0; bHi = BINS - 1; }
  const pLo = lo + (bLo / (BINS - 1)) * span;
  const pHi = lo + (bHi / (BINS - 1)) * span;
  const inv = 1 / Math.max(1e-5, pHi - pLo);

  let mean = 0;
  for (let i = 0; i < N; i++) {
    const v = clamp01((src[i] - pLo) * inv);
    dst[i] = v;
    mean += v;
  }
  mean /= N;
  // Gamma so the mean lands on 0.5 — the neutral point of the splat compare.
  if (mean > 0.02 && mean < 0.98) {
    const g = Math.log(0.5) / Math.log(mean);
    if (g > 0.25 && g < 4) {
      for (let i = 0; i < N; i++) dst[i] = 0.02 + Math.pow(dst[i], g) * 0.96;
      return;
    }
  }
  for (let i = 0; i < N; i++) dst[i] = 0.02 + dst[i] * 0.96;
}

/**
 * @param slopeK  d(worldHeight)/d(tileFraction) per unit of the 0..1 height
 *                field, i.e. RELIEF_METRES / TILE_METRES. See MACRO_RELIEF_M.
 */
function deriveNormalAndPack(S, F, slopeK, aoK, albedoData, nrhData, layer) {
  const N = S * S;
  const off = layer * N * 4;

  // AO from the difference between the height field and two blurred copies:
  // a wide radius for large cavities, a tight one for creases.
  const wide = F.t0, tight = F.t1, tmp = F.t2;
  boxBlurWrap(F.h, wide, tmp, S, Math.max(2, (S * 0.055) | 0));
  boxBlurWrap(F.h, tight, tmp, S, Math.max(1, (S * 0.013) | 0));

  // The packed height channel is normalised; normals and AO stay on the raw
  // field so each surface keeps its authored relief strength.
  const hPack = F.t3;
  normaliseHeight(F.h, hPack, N);

  for (let y = 0; y < S; y++) {
    const ym = ((y - 1) + S) % S, yp = (y + 1) % S;
    const rowY = y * S, rowM = ym * S, rowP = yp * S;
    for (let x = 0; x < S; x++) {
      const i = rowY + x;
      const xm = ((x - 1) + S) % S, xp = (x + 1) % S;

      // Central-difference gradient of the *generated* height field, with the
      // diagonal taps mixed in to smooth single-texel spikes from the strokes.
      const dhdu = (F.h[rowY + xp] - F.h[rowY + xm]) * 0.5;
      const dhdv = (F.h[rowP + x] - F.h[rowM + x]) * 0.5;
      const dhdu2 = (F.h[rowP + xp] - F.h[rowP + xm] + F.h[rowM + xp] - F.h[rowM + xm]) * 0.25;
      const dhdv2 = (F.h[rowP + xp] - F.h[rowM + xp] + F.h[rowP + xm] - F.h[rowM + xm]) * 0.25;
      // dh is per texel; S texels span one tile; slopeK converts a unit of the
      // 0..1 field to world metres per tile. So this is a true dH/dX.
      const gu = ((dhdu * 2 + dhdu2) / 3) * S * slopeK;
      const gv = ((dhdv * 2 + dhdv2) / 3) * S * slopeK;

      let nx = -gu, ny = -gv;
      const len = Math.sqrt(nx * nx + ny * ny + 1);
      nx /= len; ny /= len;

      const aoWide = clamp01(1 - (wide[i] - F.h[i]) * aoK * 1.6);
      const aoTight = clamp01(1 - (tight[i] - F.h[i]) * aoK * 3.4);
      const ao = clamp01(Math.min(aoWide, aoTight) * 0.55 + aoWide * aoTight * 0.45) * clamp01(F.ao[i]);

      const o = off + i * 4;
      albedoData[o] = (srgbEncode(F.r[i]) * 255 + 0.5) | 0;
      albedoData[o + 1] = (srgbEncode(F.g[i]) * 255 + 0.5) | 0;
      albedoData[o + 2] = (srgbEncode(F.b[i]) * 255 + 0.5) | 0;
      albedoData[o + 3] = (ao * 255 + 0.5) | 0;

      nrhData[o] = ((nx * 0.5 + 0.5) * 255 + 0.5) | 0;
      nrhData[o + 1] = ((ny * 0.5 + 0.5) * 255 + 0.5) | 0;
      nrhData[o + 2] = (clamp01(F.ro[i]) * 255 + 0.5) | 0;
      nrhData[o + 3] = (clamp01(hPack[i]) * 255 + 0.5) | 0;
    }
  }
}

// Per-surface micro-detail character:
// [ freqU, freqV, octaves, contrast, speckle ]
// Relief amplitude is no longer a free scalar here either — see DETAIL_RELIEF_M.
//
// A8 (r5) — THE ASPECT RATIOS HERE ARE WORLD-AXIS LOCKED. terrDetail() samples
// this array at bare `gWPos.xz` with NO tile-break and therefore NO per-cell
// rotation (the comment there says none is needed at sub-metre scale, which is
// true of repetition but not of direction). So a 4.7:1 grass tile and a 6:1
// root tile were not "fibres" — they were one comb, aligned to world +X, over
// the entire mountain, on every fragment where those layers won the splat.
// Brought down to a hint of grain (1.4:1 / 1.7:1). The real directional
// character of grass is the coherent blade flow field in genGrass(), which is
// in the MACRO array and therefore does get per-cell rotation.
const DETAIL_DEFS = [
  [40, 40, 4, 1.00, 0.10], // dirt   — granular
  [34, 34, 4, 0.90, 0.06], // loam   — soft fibrous
  [30, 30, 5, 1.35, 0.16], // rock   — crystalline chipping
  [46, 46, 4, 1.25, 0.20], // gravel — grit
  [28, 40, 4, 1.00, 0.05], // grass  — fibrous, faintly aligned (was 12x56)
  [26, 44, 4, 1.05, 0.05], // root   — bark grain (was 10x60)
  [36, 36, 4, 0.85, 0.04], // mud    — wet speckle
  [44, 44, 3, 0.70, 0.22], // snow   — crystal sparkle
];

function buildDetailLayer(S, layer, seed, data) {
  const def = DETAIL_DEFS[layer];
  const fu = def[0], fv = def[1], oct = def[2], contrast = def[3];
  const speckle = def[4];
  // Same calibration as the macro layer: metres of relief over metres of tile.
  const slopeK = DETAIL_RELIEF_M[layer] / SURFACE_DEFS[layer].detail;

  const n = makePerlin(subSeed(seed, 'det' + layer));
  const n2 = makePerlin(subSeed(seed, 'det2' + layer));
  const rng = makeRng(subSeed(seed, 'detspk' + layer));

  const N = S * S;
  const h = new Float32Array(N);
  const bright = new Float32Array(N);
  const rough = new Float32Array(N);

  const lo = Math.max(4, fu >> 2), mid = Math.max(6, fu >> 1);
  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = y * S + x;
      let e = fbmAniso(n, u, v, fu, fv, oct, 0.55);
      e = Math.sign(e) * Math.pow(Math.abs(e), 1 / contrast);
      h[i] = e * 0.5 + 0.5;
      bright[i] = clamp01(0.5 + e * 0.55 + fbm(n2, u, v, lo, 2, 0.5) * 0.18);
      rough[i] = clamp01(0.5 - e * 0.34 + fbm(n2, u + 0.3, v + 0.6, mid, 2, 0.5) * 0.22);
    }
  }
  // Sparse bright/smooth specks (mica, quartz grit, ice crystal).
  const specks = Math.round(N * speckle * 0.02);
  for (let k = 0; k < specks; k++) {
    const i = (rng() * N) | 0;
    bright[i] = clamp01(bright[i] + 0.45);
    rough[i] = clamp01(rough[i] - 0.35);
    h[i] = clamp01(h[i] + 0.25);
  }

  const off = layer * N * 4;
  for (let y = 0; y < S; y++) {
    const ym = ((y - 1) + S) % S, yp = (y + 1) % S;
    const rowY = y * S, rowM = ym * S, rowP = yp * S;
    for (let x = 0; x < S; x++) {
      const i = rowY + x;
      const xm = ((x - 1) + S) % S, xp = (x + 1) % S;
      const gu = (h[rowY + xp] - h[rowY + xm]) * 0.5 * S * slopeK;
      const gv = (h[rowP + x] - h[rowM + x]) * 0.5 * S * slopeK;
      let nx = -gu, ny = -gv;
      const len = Math.sqrt(nx * nx + ny * ny + 1);
      nx /= len; ny /= len;
      const o = off + i * 4;
      data[o] = ((nx * 0.5 + 0.5) * 255 + 0.5) | 0;
      data[o + 1] = ((ny * 0.5 + 0.5) * 255 + 0.5) | 0;
      data[o + 2] = (bright[i] * 255 + 0.5) | 0;
      data[o + 3] = (rough[i] * 255 + 0.5) | 0;
    }
  }
}

function buildMacroNoise(S, seed) {
  const n1 = makePerlin(subSeed(seed, 'macro1'));
  const n2 = makePerlin(subSeed(seed, 'macro2'));
  const n3 = makePerlin(subSeed(seed, 'macro3'));
  const n4 = makePerlin(subSeed(seed, 'macro4'));
  const data = new Uint8Array(S * S * 4);
  for (let y = 0; y < S; y++) {
    const v = y / S;
    for (let x = 0; x < S; x++) {
      const u = x / S;
      const i = (y * S + x) * 4;
      // R: broad brightness / dryness.
      const r = clamp01(fbm(n1, u, v, 3, 5, 0.55) * 0.55 + 0.5);
      // G: warm/cool hue drift, domain-warped so it does not correlate with R.
      const wx = fbm(n3, u, v, 2, 2, 0.5) * 0.2;
      const g = clamp01(fbm(n2, u + wx, v, 2, 4, 0.55) * 0.6 + 0.5);
      // B: patch mask for surface-mix variation.
      const b = clamp01(ridged(n3, u, v, 4, 4, 0.5) * 1.25);
      // A: moisture — valley-shaped, so it concentrates in lines not blobs.
      const a = clamp01(1 - Math.abs(fbm(n4, u, v, 3, 4, 0.5)) * 1.7);
      data[i] = (r * 255) | 0;
      data[i + 1] = (g * 255) | 0;
      data[i + 2] = (b * 255) | 0;
      data[i + 3] = (a * 255) | 0;
    }
  }
  const tex = new THREE.DataTexture(data, S, S, THREE.RGBAFormat, THREE.UnsignedByteType);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.generateMipmaps = true;
  tex.colorSpace = THREE.NoColorSpace;
  tex.needsUpdate = true;
  tex.name = 'terrain-macro-variation';
  return tex;
}

// ---------------------------------------------------------------------------
// Texture set cache (shared by every terrain chunk)
// ---------------------------------------------------------------------------

const TEXTURE_CACHE = new Map();

function buildTextureSet(seed, size, detailSize, anisotropy) {
  const key = seed + '|' + size + '|' + detailSize;
  const cached = TEXTURE_CACHE.get(key);
  if (cached) {
    if (anisotropy > cached.anisotropy) {
      cached.anisotropy = anisotropy;
      for (const t of [cached.albedo, cached.nrh, cached.detail]) {
        t.anisotropy = anisotropy;
        t.needsUpdate = true;
      }
    }
    return cached;
  }

  const t0 = (typeof performance !== 'undefined' ? performance.now() : 0);
  const N = size * size;
  const albedoData = new Uint8Array(N * 4 * LAYER_COUNT);
  const nrhData = new Uint8Array(N * 4 * LAYER_COUNT);
  const F = newFields(N);

  for (let layer = 0; layer < LAYER_COUNT; layer++) {
    F.h.fill(0); F.r.fill(0); F.g.fill(0); F.b.fill(0);
    F.ro.fill(0.9); F.ao.fill(1);
    GENERATORS[layer](size, F, subSeed(seed, 'surf' + layer));
    deriveNormalAndPack(size, F, MACRO_RELIEF_M[layer] / SURFACE_DEFS[layer].macro,
      AO_SCALE[layer], albedoData, nrhData, layer);
  }

  const detailData = new Uint8Array(detailSize * detailSize * 4 * LAYER_COUNT);
  for (let layer = 0; layer < LAYER_COUNT; layer++) {
    buildDetailLayer(detailSize, layer, subSeed(seed, 'detail'), detailData);
  }

  const mkArray = (data, w, h, colorSpace, name) => {
    const tex = new THREE.DataArrayTexture(data, w, h, LAYER_COUNT);
    tex.format = THREE.RGBAFormat;
    tex.type = THREE.UnsignedByteType;
    tex.colorSpace = colorSpace;
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.minFilter = THREE.LinearMipmapLinearFilter;
    tex.magFilter = THREE.LinearFilter;
    tex.generateMipmaps = true;
    tex.anisotropy = anisotropy;
    tex.name = name;
    tex.needsUpdate = true;
    return tex;
  };

  const set = {
    // sRGB colour space => three uploads as SRGB8_ALPHA8, so the GPU linearises
    // RGB for free. Alpha (our AO) is never sRGB-encoded, per the GL spec.
    albedo: mkArray(albedoData, size, size, THREE.SRGBColorSpace, 'terrain-albedo-array'),
    nrh: mkArray(nrhData, size, size, THREE.NoColorSpace, 'terrain-nrh-array'),
    detail: mkArray(detailData, detailSize, detailSize, THREE.NoColorSpace, 'terrain-detail-array'),
    macro: buildMacroNoise(256, subSeed(seed, 'macronoise')),
    anisotropy,
    size, detailSize, key,
    buildMs: (typeof performance !== 'undefined' ? performance.now() : 0) - t0,
  };

  TEXTURE_CACHE.set(key, set);
  return set;
}

// ---------------------------------------------------------------------------
// Optional SurfaceId map (from terrain.js's `materials` array)
// ---------------------------------------------------------------------------

function buildIdTexture(ids, res) {
  const data = new Uint8Array(res * res);
  data.set(ids.subarray(0, res * res));
  const tex = new THREE.DataTexture(data, res, res, THREE.RedFormat, THREE.UnsignedByteType);
  tex.wrapS = tex.wrapT = THREE.ClampToEdgeWrapping;
  tex.minFilter = THREE.NearestFilter;
  tex.magFilter = THREE.NearestFilter;
  tex.generateMipmaps = false;
  tex.colorSpace = THREE.NoColorSpace;
  tex.unpackAlignment = 1;
  tex.needsUpdate = true;
  tex.name = 'terrain-surface-ids';
  return tex;
}

// ---------------------------------------------------------------------------
// GLSL
// ---------------------------------------------------------------------------

const VERT_PARS = /* glsl */`
varying vec3 vTerrWPos;
varying vec3 vTerrWNrm;
#ifdef TERRAIN_SURFACE_ATTR
attribute float aSurface;
varying float vTerrSurf;
#endif
`;

const VERT_BODY = /* glsl */`
vTerrWPos = ( modelMatrix * vec4( transformed, 1.0 ) ).xyz;
vTerrWNrm = normalize( mat3( modelMatrix ) * objectNormal );
#ifdef TERRAIN_SURFACE_ATTR
vTerrSurf = aSurface;
#endif
`;

const FRAG_PARS = /* glsl */`
varying vec3 vTerrWPos;
varying vec3 vTerrWNrm;
#ifdef TERRAIN_SURFACE_ATTR
varying float vTerrSurf;
uniform float uSurfaceAttrWeight;
#endif

uniform sampler2DArray uAlbedoArray;   // rgb albedo (sRGB), a = baked AO
uniform sampler2DArray uNrhArray;      // rg normal.xy, b roughness, a height
uniform sampler2DArray uDetailArray;   // rg normal.xy, b brightness, a roughness
uniform sampler2D      uMacroNoise;    // r bright, g hue, b patch, a moisture

#ifdef TERRAIN_ID_MAP
uniform sampler2D uIdMap;
uniform vec4  uIdMapRect;    // xy = (minX, minZ), zw = 1/(sizeX, sizeZ)
uniform float uIdMapTexel;   // 1 / resolution
uniform float uIdWeight;
#endif

uniform float uLayerMacro[8];
uniform float uLayerDetail[8];
uniform float uLayerHeightK[8];
uniform float uLayerNormalK[8];
uniform float uLayerRoughK[8];
uniform float uLayerRoughMin[8];   // dry ground is NOT allowed to be glossy
uniform float uLayerLift[8];       // additive albedo floor, see LOAM_ALBEDO_LIFT
uniform vec3  uLayerTint[8];

uniform float uTime;
uniform float uWetness;
uniform float uSnowAmount;
uniform float uSnowLine;
uniform float uSnowBlend;
uniform float uDetailStrength;
uniform float uDetailAlbedo;
uniform float uDetailFadeStart;
uniform float uDetailFadeEnd;
uniform vec3  uVarScales;      // macro / mid / patch, in 1/metres
uniform float uMacroContrast;
uniform float uTriStart;
uniform float uTriEnd;
uniform float uTriSharpness;
uniform float uSplatSharpness;
uniform float uNormalStrength;
uniform float uCavityStrength;
uniform float uAOStrength;
uniform float uTileRotate;
uniform float uRoughnessScale;
uniform float uRainRipple;
uniform float uRoughFloor;      // global dry-ground roughness floor
uniform float uSpecAAVar;       // Kaplanyan sigma
uniform float uSpecAAClamp;     // Kaplanyan kappa, in alpha (= roughness^2) units
uniform float uSplatContrast;   // height-splat seam width, in score units
uniform float uLayerSkip;       // below this normalised weight a layer is not sampled
uniform float uAnisoClamp;      // A8/r5: max major:minor of the sampling footprint

// R0/A1 — the ONE number the cavity guard needs: the world-space distance
// between two adjacent vertices of THIS chunk's mesh, in metres. Per material,
// NOT part of the shared bundle: it is the only value that differs between the
// per-LOD material variants. See materialForDepth().
uniform float uVertexSpacing;
uniform vec2  uCavityTriPx;     // x = fully on below this many px, y = fully off above

uniform float uHardpackBlend;   // A5: warm mineral base kept under painted rock/gravel
uniform vec2  uWindDir;         // A7: prevailing wind, world xz, unit length
uniform float uSnowEdgeScale;   // A7: 1/metres of the snow-margin raggedness

// Analytic (texture-free) macro fields — see the terrMacroTint block.
uniform vec3  uGeoTint[4];
uniform vec3  uGeoScale;        // 1/metres for the three analytic octaves
uniform vec2  uAltRange;        // world Y of the bottom / top of the mountain
uniform float uMacroTintStrength;
uniform float uMacroValueStrength;
uniform float uDrainStain;

#define TERR_BOMB_FREQ 0.37

// --- fragment-scope globals, filled once at the top of the body -------------
vec3 gWPos;
vec3 gGeoN;
vec3 gTriW;
vec2 gDTopX, gDTopY;   // d(world.xz)/dx, /dy  — metres per pixel
vec2 gDSxX,  gDSxY;    // d(world.zy)
vec2 gDSzX,  gDSzY;    // d(world.xy)
float gDetailFade;

// Hash without sin() — stable at the world-space magnitudes we index with.
vec2 terrHash2( vec2 p ) {
  vec3 p3 = fract( vec3( p.xyx ) * vec3( 0.1031, 0.1030, 0.0973 ) );
  p3 += dot( p3, p3.yzx + 33.33 );
  return fract( ( p3.xx + p3.yz ) * p3.zy );
}

mat2 terrRot( float a ) {
  float s = sin( a ), c = cos( a );
  return mat2( c, - s, s, c );
}

// A8 (r5) — SCREEN-FOOTPRINT ANISOTROPY CLAMP.  See the r5 CONTRACT-NOTE.
// ---------------------------------------------------------------------------
// Every array fetch in this shader is a textureGrad with the true per-pixel
// world derivatives.  On a hillside seen edge-on — which is most of the ground
// in a downhill chase camera — that footprint is a needle: tens of metres long
// along the view ray, centimetres across it.  A sampler handed a needle applies
// a needle-shaped filter: it low-passes the surface HARD along the view
// direction and leaves it sharp across, which turns isotropic grain into
// parallel dashes.  That is the r5 "corduroy", and it is why the striations run
// down the fall line, follow the camera, and appear at every distance.
//
// The fix is to widen the short axis until the ellipse is at worst
// maxRatio*sqrt(1 + 1/maxRatio^2) : 1  (3.16:1 at the default cap of 3 — the
// long axis grows a little too, because the growth is isotropic).  Growing an
// axis only ever ADDS blur, so this can never
// introduce aliasing, and it is self-targeting: ground the camera looks down on
// (the near tread, incidence 20-40 deg, ratio 1.6-3) is untouched, while the
// mid-field slope at 3 deg of incidence (ratio ~19) is fully clamped.
//
// Closed form, no eigenvectors.  The footprint ellipse of M = [gx gy] is
// described by A = M*M^T (symmetric 2x2), whose eigenvalues are the squared
// semi-axes.  Adding r^2*I to A grows the ellipse isotropically by r; picking
// r^2 = want - minor^2 lifts the short axis to 'want' and leaves the long axis
// essentially where it was (r <= major / maxRatio).  M' is then recovered as
// the symmetric square root of A', which is the same ellipse — and the ellipse
// is all the sampler reads.
void terrClampAniso( inout vec2 gx, inout vec2 gy, float maxRatio ) {
  float mr = max( maxRatio, 1.0 );
  float a = gx.x * gx.x + gy.x * gy.x;
  float b = gx.x * gx.y + gy.x * gy.y;
  float c = gx.y * gx.y + gy.y * gy.y;
  float t = 0.5 * ( a + c );
  float d = sqrt( max( t * t - ( a * c - b * b ), 0.0 ) );
  float major2 = t + d;
  float minor2 = max( t - d, 0.0 );
  float want2 = major2 / ( mr * mr );
  if ( major2 < 1e-12 || minor2 >= want2 ) return;
  float r2 = want2 - minor2;
  a += r2; c += r2;
  // sqrt of a symmetric PSD 2x2:  ( A + sqrt(det) I ) / sqrt( tr + 2 sqrt(det) )
  float s = sqrt( max( a * c - b * b, 0.0 ) );
  float den = sqrt( max( a + c + 2.0 * s, 1e-12 ) );
  gx = vec2( a + s, b ) / den;
  gy = vec2( b, c + s ) / den;
}

// Analytic value noise in WORLD METRES.  Deliberately not a texture fetch: the
// macro-variation map is mipped, so its top two octaves are gone by ~300 m and
// the far half of an establishing shot collapses to a single flat brown.  This
// survives to the horizon and cannot alias at the scales it is used at (a 200 m
// feature is still ~100 px at 3 km).
float terrVN( vec2 p ) {
  vec2 i = floor( p ), f = fract( p );
  vec2 s = f * f * ( 3.0 - 2.0 * f );
  float a = terrHash2( i ).x;
  float b = terrHash2( i + vec2( 1.0, 0.0 ) ).x;
  float c = terrHash2( i + vec2( 0.0, 1.0 ) ).x;
  float d = terrHash2( i + vec2( 1.0, 1.0 ) ).x;
  return mix( mix( a, b, s.x ), mix( c, d, s.x ), s.y );
}
float terrVN2( vec2 p ) {
  return terrVN( p ) * 0.66 + terrVN( p * 2.17 + 11.31 ) * 0.34;
}

// Heitz/Neyret triangular grid: three overlapping randomised copies of the
// texture with barycentric weights. Unlike naive per-tile randomisation this
// has no cell seams at all.
void terrTriGrid( vec2 uv, out vec3 w, out vec2 v1, out vec2 v2, out vec2 v3 ) {
  const mat2 toSkewed = mat2( 1.0, 0.0, - 0.57735027, 1.15470054 );
  vec2 sk = toSkewed * uv;
  vec2 base = floor( sk );
  vec3 t = vec3( fract( sk ), 0.0 );
  t.z = 1.0 - t.x - t.y;
  if ( t.z > 0.0 ) {
    w = vec3( t.z, t.y, t.x );
    v1 = base;
    v2 = base + vec2( 0.0, 1.0 );
    v3 = base + vec2( 1.0, 0.0 );
  } else {
    w = vec3( - t.z, 1.0 - t.y, 1.0 - t.x );
    v1 = base + vec2( 1.0, 1.0 );
    v2 = base + vec2( 1.0, 0.0 );
    v3 = base + vec2( 0.0, 1.0 );
  }
}

vec2 terrTileUV( vec2 uv, vec2 cell, float rotAmt, out mat2 m ) {
  vec2 h = terrHash2( cell + vec2( 0.37, 0.11 ) );
  float a = ( h.x - 0.5 ) * 6.2831853 * rotAmt;
  m = terrRot( a );
  return m * uv + h * 23.17;
}

// One projection of one layer, with stochastic tile-break. Gradients always
// come from the continuous uv (transformed by the same rotation), so the
// discontinuous stochastic uv can never produce a mip seam.
void terrProj( float layer, vec2 uv, vec2 ddx, vec2 ddy, out vec4 nrh, out vec4 alb ) {
  #ifdef TERRAIN_TILE_BREAK
    vec3 bw; vec2 c1, c2, c3;
    terrTriGrid( uv * TERR_BOMB_FREQ, bw, c1, c2, c3 );

    #if TERRAIN_TILE_BREAK_TAPS == 2
      // PERF: two copies instead of three.  The third corner of the triangle
      // always carries the smallest barycentric weight, and a two-copy
      // height-blend still kills the repetition grid completely — this is a
      // straight 33% cut of the single most expensive thing in the frame
      // (2 arrays x up to 3 projections x maxLayers fetches each).
      // Selecting the two heaviest corners is pure scalar compare work in
      // uniform control flow; the compiler lowers it to selects, so no
      // divergence is introduced.
      vec2 cA = c1, cB = c2, cC = c3;
      float wA = bw.x, wB = bw.y, wC = bw.z;
      if ( wB > wA ) { vec2 tc = cA; cA = cB; cB = tc; float tw = wA; wA = wB; wB = tw; }
      if ( wC > wA ) { cB = cA; wB = wA; cA = cC; wA = wC; }
      else if ( wC > wB ) { cB = cC; wB = wC; }

      mat2 m1, m2;
      vec2 u1 = terrTileUV( uv, cA, uTileRotate, m1 );
      vec2 u2 = terrTileUV( uv, cB, uTileRotate, m2 );

      vec4 n1 = textureGrad( uNrhArray, vec3( u1, layer ), m1 * ddx, m1 * ddy );
      vec4 n2 = textureGrad( uNrhArray, vec3( u2, layer ), m2 * ddx, m2 * ddy );

      // Height-aware blend: whichever copy pokes higher owns the fragment.
      // A plain weighted average washes the contrast out and reads as blur.
      vec2 hb = vec2( n1.a, n2.a ) * 0.65 + vec2( wA, wB );
      float mx = max( hb.x, hb.y );
      vec2 b = max( hb - ( mx - 0.30 ), 0.0 ) * step( 0.0015, vec2( wA, wB ) );
      b /= max( b.x + b.y, 1e-4 );

      nrh = n1 * b.x + n2 * b.y;
      alb = textureGrad( uAlbedoArray, vec3( u1, layer ), m1 * ddx, m1 * ddy ) * b.x
          + textureGrad( uAlbedoArray, vec3( u2, layer ), m2 * ddx, m2 * ddy ) * b.y;
    #else
      mat2 m1, m2, m3;
      vec2 u1 = terrTileUV( uv, c1, uTileRotate, m1 );
      vec2 u2 = terrTileUV( uv, c2, uTileRotate, m2 );
      vec2 u3 = terrTileUV( uv, c3, uTileRotate, m3 );

      vec4 n1 = textureGrad( uNrhArray, vec3( u1, layer ), m1 * ddx, m1 * ddy );
      vec4 n2 = textureGrad( uNrhArray, vec3( u2, layer ), m2 * ddx, m2 * ddy );
      vec4 n3 = textureGrad( uNrhArray, vec3( u3, layer ), m3 * ddx, m3 * ddy );

      vec3 hb = vec3( n1.a, n2.a, n3.a ) * 0.65 + bw;
      float mx = max( hb.x, max( hb.y, hb.z ) );
      vec3 b = max( hb - ( mx - 0.30 ), 0.0 ) * step( 0.0015, bw );
      b /= max( b.x + b.y + b.z, 1e-4 );

      nrh = n1 * b.x + n2 * b.y + n3 * b.z;
      alb = textureGrad( uAlbedoArray, vec3( u1, layer ), m1 * ddx, m1 * ddy ) * b.x
          + textureGrad( uAlbedoArray, vec3( u2, layer ), m2 * ddx, m2 * ddy ) * b.y
          + textureGrad( uAlbedoArray, vec3( u3, layer ), m3 * ddx, m3 * ddy ) * b.z;
    #endif
  #else
    nrh = textureGrad( uNrhArray, vec3( uv, layer ), ddx, ddy );
    alb = textureGrad( uAlbedoArray, vec3( uv, layer ), ddx, ddy );
  #endif
}

vec3 terrDecodeN( vec2 rg, float k ) {
  vec2 xy = ( rg * 2.0 - 1.0 ) * k;
  float d = min( dot( xy, xy ), 0.995 );
  return vec3( xy, sqrt( max( 1e-5, 1.0 - d ) ) );
}

// Sample one surface layer with the triplanar blend applied.
void terrSampleLayer( int li, out vec3 albedo, out float ao, out vec3 nrmW,
                      out float rough, out float height ) {
  float layer = float( li );
  float inv = 1.0 / max( uLayerMacro[ li ], 0.05 );
  float nk = uLayerNormalK[ li ] * uNormalStrength;

  vec4 nY, aY;
  terrProj( layer, gWPos.xz * inv, gDTopX * inv, gDTopY * inv, nY, aY );
  vec3 tY = terrDecodeN( nY.rg, nk );

  vec3 accA = aY.rgb * gTriW.y;
  float accAO = aY.a * gTriW.y;
  float accR = nY.b * gTriW.y;
  float accH = nY.a * gTriW.y;
  float wsum = gTriW.y;

  vec3 N = gGeoN;

  #ifdef TERRAIN_TRIPLANAR
    vec3 tX = vec3( 0.0, 0.0, 1.0 );
    vec3 tZ = vec3( 0.0, 0.0, 1.0 );
    if ( gTriW.x > 0.004 ) {
      vec4 nn, aa;
      terrProj( layer, gWPos.zy * inv, gDSxX * inv, gDSxY * inv, nn, aa );
      tX = terrDecodeN( nn.rg, nk );
      accA += aa.rgb * gTriW.x; accAO += aa.a * gTriW.x;
      accR += nn.b * gTriW.x;   accH += nn.a * gTriW.x;
      wsum += gTriW.x;
    }
    if ( gTriW.z > 0.004 ) {
      vec4 nn, aa;
      terrProj( layer, gWPos.xy * inv, gDSzX * inv, gDSzY * inv, nn, aa );
      tZ = terrDecodeN( nn.rg, nk );
      accA += aa.rgb * gTriW.z; accAO += aa.a * gTriW.z;
      accR += nn.b * gTriW.z;   accH += nn.a * gTriW.z;
      wsum += gTriW.z;
    }
    // Whiteout blend (Golus): reproduces the geometric normal exactly when the
    // maps are flat, unlike the naive swizzle-and-add triplanar.
    vec3 wx = vec3( tX.xy + N.zy, abs( tX.z ) * N.x );
    vec3 wy = vec3( tY.xy + N.xz, abs( tY.z ) * N.y );
    vec3 wz = vec3( tZ.xy + N.xy, abs( tZ.z ) * N.z );
    nrmW = normalize( wx.zyx * gTriW.x + wy.xzy * gTriW.y + wz.xyz * gTriW.z );
  #else
    vec3 wy = vec3( tY.xy + N.xz, abs( tY.z ) * N.y );
    nrmW = normalize( wy.xzy );
  #endif

  float iw = 1.0 / max( wsum, 1e-4 );
  albedo = accA * iw * uLayerTint[ li ];
  // Additive albedo floor. Only LOAM uses it (see LOAM_ALBEDO_LIFT).
  // A4/R13: the tint used to be ( 1.0, 0.86, 0.66 ), which is a pale cream with
  // a linear B/R of 0.66 — HIGHER than the loam it is lifting (~0.44). So the
  // old lift raised the forest floor's value and made it BLUER at the same
  // time, which is the opposite of what the cold-tread finding needs. Retuned
  // to a dry-duff brown at B/R 0.38, so the same amount of lift now warms the
  // layer as it raises it. See LOAM_ALBEDO_LIFT for the magnitude.
  albedo += uLayerLift[ li ] * vec3( 1.0, 0.76, 0.38 );
  ao = accAO * iw;
  // Floored per layer. Dry mineral ground has a measured roughness of
  // 0.85-0.95; anything below ~0.8 turns a sunlit gravel bank into wet
  // plastic under a 3.5-intensity sun.
  rough = clamp( accR * iw * uLayerRoughK[ li ], uLayerRoughMin[ li ], 1.0 );
  height = accH * iw;
}

// Micro-detail layer: one fetch per active projection. No tile-break needed at
// sub-metre scale; faded out by screen footprint so it never aliases.
void terrDetail( int li, out vec3 pert, out float bright, out float roughMod ) {
  float layer = float( li );
  float inv = 1.0 / max( uLayerDetail[ li ], 0.02 );

  vec4 dY = textureGrad( uDetailArray, vec3( gWPos.xz * inv, layer ), gDTopX * inv, gDTopY * inv );
  vec2 pY = dY.rg * 2.0 - 1.0;
  pert = vec3( pY.x, 0.0, pY.y ) * gTriW.y;
  bright = dY.b * gTriW.y;
  roughMod = dY.a * gTriW.y;
  float wsum = gTriW.y;

  #ifdef TERRAIN_TRIPLANAR
    if ( gTriW.x > 0.004 ) {
      vec4 d = textureGrad( uDetailArray, vec3( gWPos.zy * inv, layer ), gDSxX * inv, gDSxY * inv );
      vec2 p = d.rg * 2.0 - 1.0;
      pert += vec3( 0.0, p.y, p.x ) * gTriW.x;
      bright += d.b * gTriW.x; roughMod += d.a * gTriW.x; wsum += gTriW.x;
    }
    if ( gTriW.z > 0.004 ) {
      vec4 d = textureGrad( uDetailArray, vec3( gWPos.xy * inv, layer ), gDSzX * inv, gDSzY * inv );
      vec2 p = d.rg * 2.0 - 1.0;
      pert += vec3( p.x, p.y, 0.0 ) * gTriW.z;
      bright += d.b * gTriW.z; roughMod += d.a * gTriW.z; wsum += gTriW.z;
    }
  #endif

  float iw = 1.0 / max( wsum, 1e-4 );
  pert *= iw; bright *= iw; roughMod *= iw;
}
`;

const FRAG_BODY = /* glsl */`
// ==========================================================================
// DESCENT terrain surface
// ==========================================================================
gWPos = vTerrWPos;
gGeoN = normalize( vTerrWNrm );

// Screen-space world derivatives for every projection, taken here in uniform
// control flow and reused as explicit gradients everywhere below.
vec3 dPx = dFdx( gWPos );
vec3 dPy = dFdy( gWPos );
gDTopX = dPx.xz; gDTopY = dPy.xz;
gDSxX  = dPx.zy; gDSxY  = dPy.zy;
gDSzX  = dPx.xy; gDSzY  = dPy.xy;

// A8 (r5) — clamp the footprint's aspect ratio BEFORE anything samples with it,
// so the macro array, the micro-detail array and every triplanar projection all
// see the same, bounded ellipse. Done once per fragment, not per layer.
terrClampAniso( gDTopX, gDTopY, uAnisoClamp );
terrClampAniso( gDSxX,  gDSxY,  uAnisoClamp );
terrClampAniso( gDSzX,  gDSzY,  uAnisoClamp );

float terrViewDist = length( vViewPosition );
float terrMPP = max( length( gDTopX ), length( gDTopY ) );   // metres per pixel

// Two independent fades: distance (art control) and footprint (anti-alias).
gDetailFade = min(
  1.0 - smoothstep( uDetailFadeStart, uDetailFadeEnd, terrViewDist ),
  1.0 - smoothstep( 0.09, 0.42, terrMPP )
);

// --- triplanar weights: planar on flat ground, blending in on slopes --------
{
  vec3 an = abs( gGeoN );
  vec3 bw = pow( an, vec3( uTriSharpness ) );
  bw /= max( bw.x + bw.y + bw.z, 1e-5 );
  float sT = smoothstep( uTriStart, uTriEnd, 1.0 - gGeoN.y );
  gTriW = vec3( bw.x * sT, mix( 1.0, bw.y, sT ), bw.z * sT );
  gTriW /= max( gTriW.x + gTriW.y + gTriW.z, 1e-5 );
}

// --- curvature / cavity ----------------------------------------------------
// R0. THE defect of round 3, filed by four lenses against three other files as
// "translucent ghost quads". Nothing was being drawn. dFdx of a smooth-shaded
// varying is CONSTANT ACROSS A TRIANGLE, so everything below is a per-triangle
// constant, and it lands on the AO, the albedo, the splat weights and the
// drainage stain. The old code diagnosed that correctly in its own comment and
// then guarded against the wrong variable: it faded by CAMERA distance, while
// terrain.js keys LOD to CORRIDOR distance. A bank 8 m from the lens and 30 m
// from the spline is a 2 m-per-vertex triangle with the old guard at exactly
// 1.0 — ~225 screen px of flat 55%-AO knockdown with a hard triangle edge.
//
// The guard is now the quantity that actually decides whether this term reads
// as curvature or as faceting: the SIZE OF THE TRIANGLE ON SCREEN.
//
//   triPx = vertexSpacing / (world metres per screen pixel)
//
// min( length(dPx), length(dPy) ) is the footprint, because it yields the LARGER of
// the triangle's two screen extents, which is the conservative one. It is also
// correct for grazing and near-vertical surfaces, where the xz-only footprint
// used by the detail fade collapses.
//
// Cavity survives only in the 4..24 px band. Above 24 px the derivative is
// straddling real geometry and turns into a lattice (the two triangles of a
// grid quad take OPPOSITE signs, which is what made r3_04 read as alternating
// light/dark masonry). Below ~4 px it is sub-pixel noise being multiplied into
// albedo, which aliases at range.
float terrCavity = 0.0;
float terrCrest = 0.0;
#ifdef TERRAIN_CAVITY
{
  // Taken in uniform control flow: a derivative inside a branch is undefined
  // behaviour in GLSL ES 3.00 even when the branch is quad-uniform, which this
  // one is (dPx/dPy are themselves quad constants).
  vec3 dNx = dFdx( vTerrWNrm );
  vec3 dNy = dFdy( vTerrWNrm );
  float terrTriPx = uVertexSpacing / max( min( length( dPx ), length( dPy ) ), 1e-5 );
  float cf = ( 1.0 - smoothstep( uCavityTriPx.x, uCavityTriPx.y, terrTriPx ) )
           * smoothstep( 1.5, 4.0, terrTriPx );
  if ( cf > 0.002 ) {
    float lx = max( dot( dPx, dPx ), 1e-6 );
    float ly = max( dot( dPy, dPy ), 1e-6 );
    // Concave where the normal rotates against the direction of travel.
    float conc = - ( dot( dNx, dPx ) / lx + dot( dNy, dPy ) / ly ) * 0.5;
    terrCavity = clamp( conc * 14.0, 0.0, 1.0 ) * cf;
    terrCrest = clamp( - conc * 14.0, 0.0, 1.0 ) * cf;
  }
}
#endif

// --- macro variation (three decorrelated octaves of the same noise map) -----
vec2 terrMv = gWPos.xz;
vec4 mA = texture( uMacroNoise, terrMv * uVarScales.x );
vec4 mB = texture( uMacroNoise, ( terrMv * uVarScales.y ) * mat2( 0.8, 0.6, - 0.6, 0.8 ) + 0.371 );
vec4 mC = texture( uMacroNoise, ( terrMv * uVarScales.z ) * mat2( - 0.28, 0.96, - 0.96, - 0.28 ) + 0.713 );

float terrSlope01 = 1.0 - clamp( gGeoN.y, 0.0, 1.0 );   // 0 flat .. 1 vertical
float terrAlt = gWPos.y;

// --- drainage / wetness ----------------------------------------------------
// R0 consumer 4/5: this feeds wetness AND the gully staining in the analytic
// geology block (up to a -13.5% albedo multiply), so a per-triangle cavity put
// a hard-edged stain wedge on the hillside. Cavity's share is reduced and the
// noise fields carry proportionally more.
float terrDrain = clamp( terrCavity * 0.95 + ( mA.a - 0.42 ) * 1.20 + ( mC.a - 0.5 ) * 0.40, 0.0, 1.0 );
terrDrain *= 1.0 - smoothstep( 0.24, 0.55, terrSlope01 );   // water does not sit on cliffs
float terrWet = clamp( uWetness * ( 0.30 + 1.05 * terrDrain ), 0.0, 1.0 );

// --- snow mask -------------------------------------------------------------
// A7. r3 filed the snow patches as "smooth flat ellipses". They were never
// decals — the mask was already slope + altitude + noise — but every term in it
// was low-frequency and isotropic, so the margin could only ever be a smooth
// oval. Three things fix that, all analytic and fetch-free:
//   * a 3.5 m / 1.6 m raggedness octave on the altitude threshold, so the
//     snowline breaks up at the scale a viewer reads snow margins at;
//   * a WIND ASPECT term — snow scours off windward faces and drifts deep on
//     the lee side, which is the single strongest real-world cue and the reason
//     alpine snow patches are asymmetric rather than elliptical;
//   * the splat's own height blend then finishes the edge at stone scale,
//     because snow's weight now crosses its neighbours' inside the margin
//     rather than fading uniformly through it.
float terrSnow = 0.0;
if ( uSnowAmount > 0.001 ) {
  float edge = terrVN2( gWPos.xz * uSnowEdgeScale );
  float altBias = ( mB.r - 0.5 ) * uSnowBlend * 1.35
                + ( edge - 0.5 ) * uSnowBlend * 1.10;
  float alt = smoothstep( uSnowLine - uSnowBlend, uSnowLine + uSnowBlend, terrAlt + altBias );
  float lay = smoothstep( 0.30, 0.72, gGeoN.y );
  // gGeoN.xz is the horizontal component of the surface normal: it points at
  // the direction the slope FACES. +1 = facing straight into the wind.
  float aspect = dot( normalize( gGeoN.xz + vec2( 1e-4 ) ), uWindDir );
  float wind = mix( 1.30, 0.55, smoothstep( - 0.65, 0.65, aspect ) );
  terrSnow = clamp( uSnowAmount * alt * lay * wind
                    * ( 0.80 + terrCavity * 0.6 ) * ( 0.60 + mC.b * 0.8 ), 0.0, 1.0 );
}

// --- surface weights -------------------------------------------------------
float wArr[8];
{
  // NB: 'patch' is a reserved word in GLSL ES 3.00 — do not name anything that.
  float patchN = mC.b;
  float dryness = mA.r;
  float band = mB.b;

  // Slope is expressed as 1 - cos(theta):
  //   20 deg = 0.060   30 deg = 0.134   40 deg = 0.234
  //   50 deg = 0.357   60 deg = 0.500   70 deg = 0.658

  // ROCK: outcrops from ~40 deg, dominant past ~58 deg, plus scattered bedrock
  // knuckles poking through gentle ground.
  float rockSlope = smoothstep( 0.235, 0.50, terrSlope01 + ( patchN - 0.5 ) * 0.14 );
  float rockPatch = smoothstep( 0.76, 0.94, band ) * ( 1.0 - smoothstep( 0.05, 0.20, terrSlope01 ) ) * 0.5;
  wArr[2] = max( rockSlope, rockPatch ) * ( 0.90 + patchN * 0.35 ) * 1.25;

  // GRAVEL / scree: the transitional band below the cliffs, deposited in fans.
  // Deliberately wide and strong — a bare alpine face is mostly loose rock,
  // not solid bedrock, and it is what stops the mountain reading as one slab.
  float scree = smoothstep( 0.055, 0.20, terrSlope01 ) * ( 1.0 - smoothstep( 0.33, 0.58, terrSlope01 ) );
  wArr[3] = scree * ( 0.55 + smoothstep( 0.35, 0.78, band ) * 1.15 ) * ( 0.65 + dryness * 0.75 ) * 1.15;

  // GRASS: gentle, moist, away from the bedrock patches.
  float gentle = 1.0 - smoothstep( 0.07, 0.30, terrSlope01 );
  wArr[4] = gentle * ( 0.45 + ( 1.0 - dryness ) * 1.15 ) * ( 0.55 + mA.a * 0.9 )
            * ( 1.0 - smoothstep( 0.55, 0.95, band ) * 0.6 ) * 1.2;

  // LOAM: sheltered forest floor — gentle-to-moderate, concave, damp.
  // R0 consumer 3: the cavity share of this is what put a DIFFERENT MATERIAL on
  // alternate triangles — the "straight-jointed masonry" on the r3_04 cliff. It
  // is a legitimate rule (duff does collect in hollows) so it stays, but the
  // moisture field now carries more of it than the curvature estimate does.
  wArr[1] = ( 1.0 - smoothstep( 0.13, 0.38, terrSlope01 ) )
            * ( 0.34 + terrCavity * 0.65 + mA.a * 0.95 )
            * ( 0.5 + ( 1.0 - dryness ) * 0.8 );

  // DIRT: the default hardpack. Never zero, so the ground always has a base.
  wArr[0] = 0.34 + ( 1.0 - smoothstep( 0.24, 0.48, terrSlope01 ) ) * ( 0.40 + dryness * 1.0 )
            + terrCrest * 0.20;

  // ROOT: small patches, inside the loamy zones only.
  wArr[5] = smoothstep( 0.70, 0.90, mC.g ) * ( 1.0 - smoothstep( 0.16, 0.32, terrSlope01 ) )
            * ( 0.4 + mA.a * 0.9 ) * 1.15;

  // MUD: drainage lines, scaled by the wetness uniform.
  wArr[6] = terrWet * ( 0.35 + terrDrain * 1.5 ) * ( 1.0 - smoothstep( 0.16, 0.36, terrSlope01 ) ) * 1.6;

  // SNOW.
  wArr[7] = terrSnow * 2.2;

  // A5 / R13 — WARM HARDPACK UNDER THE TREAD.
  // The materials lens measured the tread at B/R 1.91 in the r3_07 forest and
  // called it a material-assignment bug. It is not: r3_07 is the 'loam' phase on
  // Surface.LOAM, and a LOAM base at ~4% linear takes its hue wholesale from sky
  // ambient (fixed by A4's lift to ~10%). But r3_05 ('slab'/ROCK) and r3_06
  // ('creek'/GRAVEL) DO genuinely hand the tread to neutral mineral layers, and
  // those need a warm hardpack base splatted THROUGH them, not a replacement.
  // So wherever the trail has painted ROCK or GRAVEL on ground gentle enough to
  // actually ride, DIRT keeps a proportional share of the mix and wins the
  // height splat in the low spots. Gated on slope so a painted rock cliff — the
  // other thing ROCK gets painted on — is untouched. 1-cos: 0.16 = 33 deg,
  // 0.38 = 51 deg, so a bermed corner keeps its hardpack and a cliff gets none.
  float terrRideable = 1.0 - smoothstep( 0.16, 0.38, terrSlope01 );
  float terrHardpack = uHardpackBlend * terrRideable;

  #ifdef TERRAIN_ID_MAP
  {
    // Manual 4-tap bilinear over IDs. Never interpolate an id value itself —
    // that would invent surfaces which were never painted.
    vec2 iuv = ( gWPos.xz - uIdMapRect.xy ) * uIdMapRect.zw;
    vec2 tc = iuv / uIdMapTexel - 0.5;
    vec2 tb = floor( tc );
    vec2 tf = tc - tb;
    for ( int oy = 0; oy < 2; oy ++ ) {
      for ( int ox = 0; ox < 2; ox ++ ) {
        float fw = ( ox == 0 ? 1.0 - tf.x : tf.x ) * ( oy == 0 ? 1.0 - tf.y : tf.y );
        if ( fw < 0.002 ) continue;
        vec2 st = ( tb + vec2( float( ox ), float( oy ) ) + 0.5 ) * uIdMapTexel;
        float sid = texture( uIdMap, clamp( st, 0.0, 1.0 ) ).r * 255.0;
        int si = clamp( int( sid + 0.5 ), 0, 7 );
        float idw = fw * uIdWeight;
        wArr[ si ] += idw;
        if ( si == 2 || si == 3 ) wArr[ 0 ] += idw * terrHardpack;
      }
    }
  }
  #endif

  #ifdef TERRAIN_SURFACE_ATTR
  {
    float sid = clamp( vTerrSurf, 0.0, 7.0 );
    int i0 = int( floor( sid ) );
    int i1 = min( i0 + 1, 7 );
    float fr = fract( sid );
    float w0 = uSurfaceAttrWeight * ( 1.0 - fr );
    float w1 = uSurfaceAttrWeight * fr;
    wArr[ i0 ] += w0;
    wArr[ i1 ] += w1;
    if ( i0 == 2 || i0 == 3 ) wArr[ 0 ] += w0 * terrHardpack;
    if ( i1 == 2 || i1 == 3 ) wArr[ 0 ] += w1 * terrHardpack;
  }
  #endif

  wArr[0] = max( wArr[0], 0.06 );
}

// --- pick the strongest N layers ------------------------------------------
int terrSel[4];
float terrSelW[4];
float terrWTotal = 0.0;
for ( int k = 0; k < TERRAIN_MAX_LAYERS; k ++ ) {
  int best = 0;
  float bestW = -1.0;
  for ( int i = 0; i < 8; i ++ ) {
    if ( wArr[ i ] > bestW ) { bestW = wArr[ i ]; best = i; }
  }
  terrSel[ k ] = best;
  terrSelW[ k ] = max( bestW, 0.0 );
  terrWTotal += terrSelW[ k ];
  wArr[ best ] = -1.0;
}
{
  float invW = 1.0 / max( terrWTotal, 1e-4 );
  for ( int k = 0; k < TERRAIN_MAX_LAYERS; k ++ ) terrSelW[ k ] *= invW;
}

// --- sample + height-blend -------------------------------------------------
vec3 terrLA[4];
vec3 terrLN[4];
vec3 terrLM[4];   // x = ao, y = rough, z = height
float terrHS[4];

for ( int k = 0; k < TERRAIN_MAX_LAYERS; k ++ ) {
  terrLA[ k ] = vec3( 0.0 );
  terrLN[ k ] = gGeoN;
  terrLM[ k ] = vec3( 1.0, 1.0, 0.0 );
  terrHS[ k ] = -20.0;
  // PERF: a layer below this normalised weight contributes under 1/255 after
  // tonemapping but costs a full set of array fetches. Was 0.018.
  if ( terrSelW[ k ] < uLayerSkip ) continue;
  vec3 alb, nrm; float ao, rgh, hgt;
  terrSampleLayer( terrSel[ k ], alb, ao, nrm, rgh, hgt );
  terrLA[ k ] = alb;
  terrLN[ k ] = nrm;
  terrLM[ k ] = vec3( ao, rgh, hgt );
  // Height-map splat, PRODUCT form:  score = height * weight.
  //   The previous form added the coverage weight to the height (w * 1.45
  //   against a contrast of 0.15), so the weight ramp — which varies over
  //   metres — set the transition width and every surface boundary became a
  //   constant-width crossfade, i.e. the ~30 cm muddy smear running the length
  //   of the mountain. Multiplying instead means the seam width is set by the
  //   layer's own baked relief, so it goes ragged at stone scale and the
  //   proud material genuinely wins.
  float hk = clamp( ( hgt - 0.5 ) * uLayerHeightK[ terrSel[ k ] ] + 0.5, 0.02, 1.0 );
  terrHS[ k ] = hk * ( 0.12 + terrSelW[ k ] * 1.30 );
}

float terrHMax = -1e9;
for ( int k = 0; k < TERRAIN_MAX_LAYERS; k ++ ) terrHMax = max( terrHMax, terrHS[ k ] );

vec3 terrAlbedo = vec3( 0.0 );
vec3 terrNrmW = vec3( 0.0 );
float terrAOTex = 0.0;
float terrRough = 0.0;
// R4 — the per-layer roughness floor, carried out of terrSampleLayer() as a
// BLENDED value so it can be re-applied at the very end. Inside
// terrSampleLayer() the floor is clamped onto the layer's own sample and then
// three later stages are free to walk straight back through it: the micro
// detail (+-0.15), the damp term (a mix toward terrWetGloss) and, on a splat
// boundary, the blend itself. ROUGH_MIN's comment says dry mineral ground is
// 0.85-0.95 and that below ~0.8 a sunlit bank turns to wet plastic — that is a
// statement about the SHADED pixel, not about one intermediate fetch, so the
// floor has to survive to the shaded pixel.
float terrRoughMinB = 0.0;
{
  float bsum = 0.0;
  float bw[4];
  for ( int k = 0; k < TERRAIN_MAX_LAYERS; k ++ ) {
    bw[ k ] = max( terrHS[ k ] - ( terrHMax - uSplatContrast ), 0.0 );
    bsum += bw[ k ];
  }
  float invB = 1.0 / max( bsum, 1e-5 );
  for ( int k = 0; k < TERRAIN_MAX_LAYERS; k ++ ) {
    float b = bw[ k ] * invB;
    terrAlbedo += terrLA[ k ] * b;
    terrNrmW += terrLN[ k ] * b;
    terrAOTex += terrLM[ k ].x * b;
    terrRough += terrLM[ k ].y * b;
    terrRoughMinB += uLayerRoughMin[ terrSel[ k ] ] * b;
  }
  terrNrmW = normalize( terrNrmW );
}

// --- micro detail (dominant layer) ----------------------------------------
#ifdef TERRAIN_DETAIL
if ( gDetailFade > 0.004 ) {
  vec3 pert; float dBright, dRough;
  terrDetail( terrSel[ 0 ], pert, dBright, dRough );
  terrNrmW = normalize( terrNrmW + pert * ( uDetailStrength * gDetailFade ) );
  terrAlbedo *= 1.0 + ( dBright - 0.5 ) * 2.0 * uDetailAlbedo * gDetailFade;
  terrRough = clamp( terrRough + ( dRough - 0.5 ) * 0.30 * gDetailFade, 0.0, 1.0 );
}
#endif

// --- macro variation: brightness + hue so the ground is never one colour ---
// Near-field term, from the mipped macro-variation map. Good detail, but its
// top octaves are gone by ~300 m, which is why the analytic block below exists.
{
  float bright = 1.0
    + ( mA.r - 0.5 ) * uMacroContrast
    + ( mB.r - 0.5 ) * uMacroContrast * 0.55
    + ( mC.r - 0.5 ) * uMacroContrast * 0.22;
  terrAlbedo *= max( bright, 0.25 );
  // Warm/cool drift, roughly luminance preserving.
  vec3 warmT = vec3( 1.075, 1.0, 0.885 );
  vec3 coolT = vec3( 0.915, 0.985, 1.10 );
  terrAlbedo *= mix( coolT, warmT, clamp( mA.g * 0.75 + mB.g * 0.25, 0.0, 1.0 ) );
  // Sun-bleached crests, damp hollows.
  // R0 consumer 2: this was +-16%, and because the two triangles of a grid quad
  // take OPPOSITE signs of 'conc', it produced the alternating light/dark
  // lattice in r3_04 — the largest single component of the measured 0.567
  // multiply. Halved, on top of the triangle-size guard, so that even a
  // surviving triangle boundary cannot step by more than ~7%.
  terrAlbedo *= 1.0 + terrCrest * 0.05 - terrCavity * 0.08;
}

// --- macro ALBEDO: world-space analytic geology ----------------------------
// The r2 review measured the whole lit slope as one warm brown (~150/115/80)
// modulated only by N.L, with the far half of the establishing shot carrying no
// material information at all. Everything here is evaluated analytically in
// world metres so it survives to the horizon instead of mipping away.
{
  vec2 gp = gWPos.xz;
  float gA = terrVN2( gp * uGeoScale.x );                 // ~380 m rock-type field
  float gB = terrVN(  gp * uGeoScale.y + 31.7 );          // ~210 m, decorrelated
  float gV = terrVN(  gp * uGeoScale.z + 7.31 );          // ~55 m value field

  // Four authored geological hues, picked on two decorrelated fields so a
  // hillside carries visible banding rather than one flat tone.
  vec3 geo = mix(
    mix( uGeoTint[ 0 ], uGeoTint[ 1 ], smoothstep( 0.34, 0.70, gA ) ),
    mix( uGeoTint[ 2 ], uGeoTint[ 3 ], smoothstep( 0.30, 0.68, gA ) ),
    smoothstep( 0.38, 0.74, gB ) );
  geo = mix( vec3( 1.0 ), geo, uMacroTintStrength );

  // Altitude ramp: humic warm ground in the valley, cold scoured pale rock up
  // top. This is the cue that tells a viewer how high the camera is.
  float altT = clamp( ( terrAlt - uAltRange.x ) / max( 1.0, uAltRange.y - uAltRange.x ), 0.0, 1.0 );
  geo *= mix( vec3( 1.055, 1.0, 0.905 ), vec3( 0.945, 0.975, 1.055 ),
              smoothstep( 0.22, 0.86, altT ) );

  // Value field, +/- uMacroValueStrength.
  geo *= 1.0 + ( gV - 0.5 ) * 2.0 * uMacroValueStrength;

  // Gully staining. terrDrain is the near-field drainage term (it fades with
  // the curvature estimate past ~95 m), so it is backed by an analytic
  // ridge-line field at ~100 m spacing that keeps the staining alive at range.
  float gully = 1.0 - abs( terrVN( gp * uGeoScale.z * 0.55 + 19.13 ) * 2.0 - 1.0 );
  float terrStain = clamp( terrDrain * 0.7 + smoothstep( 0.62, 0.94, gully ) * 0.65, 0.0, 1.0 );
  geo *= mix( vec3( 1.0 ), vec3( 0.70, 0.735, 0.70 ), terrStain * uDrainStain );

  terrAlbedo *= geo;
}

// --- wetness ---------------------------------------------------------------
// The roughness a wet surface collapses TO now scales with how wet it actually
// is. Damp ground (uWetness ~ 0.1, which is what a dry sunny course carries)
// must stay matte; only a real water film — rain, a creek crossing — is
// allowed near the gloss end. Previously every value of uWetness drove the same
// 0.075 target, which is a near-mirror, and uWetness defaulted to 0.18 for no
// stated reason.
float terrWetGloss = mix( 0.62, 0.09, smoothstep( 0.30, 0.85, uWetness ) );
// R4 — DAMP IS NOT WET. THE FILM THRESHOLD.
// The r3 report recorded uWetness's DEFAULT being set to 0.0 while the live
// material reads 0.13. Both are true and neither is a bug in this file: the
// default here is 0.0, and terrain.js's commit() passes wetness: 0.13 in
// matOpts (terrain.js:2728), which overrides it. 0.13 is a defensible "the
// soil is not bone dry" number and it is terrain.js's call to make, so it is
// not overridden back — but the SHAPE of what a small value buys was wrong.
//
// At 0.13 the block below used to pull roughness 15% of the way toward 0.62
// and flatten the shading normal 10% of the way to the geometric normal, on
// ground that is visibly dry, in full sun. That is precisely the "specular
// sheen inconsistent with the flat ground beside it" this file is measured
// against, and it is worst on a 25-40 deg off-trail slope because terrDrain
// is still ungated there (its slope cut-off is smoothstep(0.24,0.55), i.e.
// 40-60 deg) so terrWet is at its maximum.
//
// So the two terms that make a surface read WET rather than DAMP — the
// roughness collapse and the normal flattening — are gated behind a film
// threshold. Below ~0.18 the ground stays exactly as rough and as detailed as
// the dry splat made it, and the only thing a damp uniform still buys is the
// albedo darkening, which is what damp soil genuinely does, drainage-weighted
// (at 0.13 that is a 2% darkening on a ridge and 10% in a gully). A real
// water film — rain, a creek crossing, setWetness(0.6) — is unaffected.
float terrWetFilm = smoothstep( 0.18, 0.55, uWetness );
if ( terrWet > 0.002 ) {
  float wamt = terrWet * ( 1.0 - terrSnow );
  float wfilm = wamt * terrWetFilm;
  // Wet ground: diffuse albedo drops (light is trapped in the water film),
  // roughness collapses, and the micro-relief fills in.
  terrAlbedo *= mix( 1.0, 0.42, wamt );
  terrRough = mix( terrRough, terrWetGloss, wfilm * 0.85 );
  terrNrmW = normalize( mix( terrNrmW, gGeoN, wfilm * 0.55 ) );
  if ( uRainRipple > 0.001 ) {
    vec2 rp = gWPos.xz * 5.7;
    float rip = sin( rp.x * 3.0 + uTime * 6.1 ) * sin( rp.y * 3.7 - uTime * 5.3 )
              + sin( ( rp.x + rp.y ) * 2.3 - uTime * 7.7 ) * 0.6;
    terrNrmW = normalize( terrNrmW + vec3( rip, 0.0, rip * 0.7 ) * uRainRipple * wfilm * 0.03 );
  }
}

// --- occlusion -------------------------------------------------------------
// R0 consumer 1: a 55% FLAT AO knockdown per triangle. conc*14 saturates at
// a curvature of 0.071/m, i.e. a 14 m radius, so an ordinary broad hollow took
// the full knockdown — and per triangle, with a hard edge. Reduced to 0.35 and
// gated by triangle size. The micro-scale occlusion this was visually standing
// in for is already carried per-pixel and mip-filtered by terrAOTex (the baked
// AO in the albedo array's alpha), which is why the near field loses nothing
// when the geometric term goes to zero close to the camera.
float terrAO = clamp( terrAOTex, 0.0, 1.0 );
terrAO *= 1.0 - terrCavity * uCavityStrength;
terrAO = clamp( mix( 1.0, terrAO, uAOStrength ), 0.0, 1.0 );

// A little of the cavity is folded into albedo too — real ground creases read
// darker in direct light as well, not only in ambient.
terrAlbedo *= mix( 1.0, terrAO, 0.30 );

// --- specular anti-aliasing (Toksvig / LEAN equivalent) --------------------
// The normal maps here store xy only and reconstruct z, so they are unit length
// by construction and a texture-space Toksvig factor has nothing to read. The
// screen-space form (Kaplanyan et al. 2016, as shipped in UE4 and Frostbite)
// measures the same quantity where it actually matters: the variance of the
// SHADING normal across the pixel footprint, after tile-break, triplanar and
// micro-detail have all had their say. That variance is converted to extra NDF
// width, so sub-pixel detail raises roughness instead of aliasing into
// sparkle. The per-layer floors above are what stop the gravel bank measuring
// above L=230 in a still frame; this is what stops it crawling in motion, and
// it is the half of the fix a screenshot cannot show you.
{
  vec3 dNx = dFdx( terrNrmW );
  vec3 dNy = dFdy( terrNrmW );
  float variance = uSpecAAVar * ( dot( dNx, dNx ) + dot( dNy, dNy ) );
  float kernel = min( 2.0 * variance, uSpecAAClamp );
  float alpha = terrRough * terrRough;
  terrRough = sqrt( clamp( alpha + kernel, 0.0, 1.0 ) );
}

// Roughness floor. 0.045 was near-mirror and applied to everything. Dry ground
// now gets the LARGER of the global floor (uRoughFloor 0.25) and the blended
// per-layer floor from ROUGH_MIN — dirt/loam/gravel/grass are all 0.80-0.85
// there, so dry forest floor can no longer be walked down to 0.25 by the micro
// detail, and a specular sheen that the flat ground beside it does not have is
// no longer reachable. 0.94 leaves the micro-detail roughness modulation a
// little downward range so the surface is not perfectly uniform.
// Only an explicit water FILM (not ambient dampness — see terrWetFilm) is
// allowed below that.
float terrDryFloor = max( uRoughFloor, terrRoughMinB * 0.94 );
float terrRoughFloor = mix( terrDryFloor, 0.045,
  smoothstep( 0.25, 0.70, terrWet ) * terrWetFilm );

diffuseColor.rgb *= max( terrAlbedo, vec3( 0.0 ) );
`;

// ---------------------------------------------------------------------------
// Uniform bundle (shared per ctx so every chunk moves together)
// ---------------------------------------------------------------------------

const UNIFORM_CACHE = new WeakMap();

function buildUniforms(ctx, textures, opts) {
  const macro = new Float32Array(LAYER_COUNT);
  const detail = new Float32Array(LAYER_COUNT);
  const heightK = new Float32Array(LAYER_COUNT);
  const normalK = new Float32Array(LAYER_COUNT);
  const roughK = new Float32Array(LAYER_COUNT);
  const roughMin = new Float32Array(ROUGH_MIN);
  const lift = new Float32Array(LAYER_COUNT);
  // P3: the only layer with an albedo lift. See LOAM_ALBEDO_LIFT.
  lift[SurfaceIds.LOAM] = opts.loamLift !== undefined ? opts.loamLift : LOAM_ALBEDO_LIFT;
  const tints = [];
  for (let i = 0; i < LAYER_COUNT; i++) {
    const d = SURFACE_DEFS[i];
    macro[i] = d.macro; detail[i] = d.detail;
    heightK[i] = d.heightK; normalK[i] = d.normalK; roughK[i] = d.roughK;
    const c = C(d.tint);
    tints.push(new THREE.Vector3(c[0], c[1], c[2]));
  }

  const bounds = opts.bounds || (ctx && ctx.terrain && ctx.terrain.bounds) || null;
  const minY = bounds && isFinite(bounds.minY) ? bounds.minY : 0;
  const maxY = bounds && isFinite(bounds.maxY) ? bounds.maxY : 480;
  const span = Math.max(1, maxY - minY);
  // Only the very top of the mountain wears snow by default — the run starts
  // high, and a dusted ridgeline is what sells the altitude without burying
  // the trail.
  const snowLine = opts.snowLine !== undefined ? opts.snowLine : minY + span * 0.93;

  const pick = (v, d) => (v === undefined ? d : v);

  return {
    uAlbedoArray: { value: textures.albedo },
    uNrhArray: { value: textures.nrh },
    uDetailArray: { value: textures.detail },
    uMacroNoise: { value: textures.macro },

    uLayerMacro: { value: macro },
    uLayerDetail: { value: detail },
    uLayerHeightK: { value: heightK },
    uLayerNormalK: { value: normalK },
    uLayerRoughK: { value: roughK },
    uLayerRoughMin: { value: roughMin },
    uLayerLift: { value: lift },
    uLayerTint: { value: tints },

    uTime: { value: 0 },
    // Default 0. Wetness is a weather/creek state, not a look: it used to
    // default to 0.18 and drag every dry surface toward a water film.
    uWetness: { value: pick(opts.wetness, 0.0) },
    uSnowAmount: { value: pick(opts.snowAmount, 0.5) },
    uSnowLine: { value: snowLine },
    uSnowBlend: { value: pick(opts.snowBlend, Math.max(12, span * 0.045)) },

    uDetailStrength: { value: pick(opts.detailStrength, 0.85) },
    // A6: 0.22 -> 0.35. Between ~1 m and ~4 m the macro texture is magnified
    // 5-15x and carries no high-frequency information at all, so the micro
    // layer is the ONLY thing writing pixel-scale variance in the near field.
    uDetailAlbedo: { value: pick(opts.detailAlbedo, 0.35) },
    uDetailFadeStart: { value: pick(opts.detailFadeStart, 22) },
    uDetailFadeEnd: { value: pick(opts.detailFadeEnd, 78) },

    uVarScales: { value: new THREE.Vector3(1 / 210, 1 / 62, 1 / 13) },
    uMacroContrast: { value: pick(opts.macroContrast, 0.30) },

    // 1 - cos(theta):  26 deg -> 0.101,  47 deg -> 0.318
    uTriStart: { value: pick(opts.triStart, 0.101) },
    uTriEnd: { value: pick(opts.triEnd, 0.318) },
    uTriSharpness: { value: pick(opts.triSharpness, 5.0) },

    uSplatSharpness: { value: pick(opts.splatSharpness, 0.15) },
    // Seam width of the height splat, in score units (score = height * weight,
    // both 0..1). 0.07 is roughly one stone of ragged edge; larger values walk
    // back toward the old constant-width crossfade.
    uSplatContrast: { value: pick(opts.splatContrast, 0.07) },
    uNormalStrength: { value: pick(opts.normalStrength, 1.0) },
    // R0: 0.55 -> 0.35. See the occlusion block.
    uCavityStrength: { value: pick(opts.cavityStrength, 0.35) },
    uAOStrength: { value: pick(opts.aoStrength, 1.0) },
    uTileRotate: { value: pick(opts.tileRotate, 0.38) },
    uRoughnessScale: { value: 1.0 },
    uRainRipple: { value: pick(opts.rainRipple, 0.0) },
    uRoughFloor: { value: pick(opts.roughFloor, 0.25) },
    // Kaplanyan specular-AA constants. sigma widens the NDF per unit of normal
    // variance; kappa caps it so a genuine geometric edge inside the 2x2
    // derivative quad cannot blow the surface out to fully rough.
    uSpecAAVar: { value: pick(opts.specAAVar, 0.60) },
    uSpecAAClamp: { value: pick(opts.specAAClamp, 0.20) },
    uLayerSkip: { value: pick(opts.layerSkip, 0.06) },
    // A8 (r5): the largest major:minor the sampling footprint is allowed to
    // reach. 3.0 is the smallest value that still lets genuine foreshortening
    // read on a bank; it is also comfortably under the 8-16 the hardware
    // sampler is configured for, so the clamp — not the driver — decides how
    // directional the filter kernel gets. Raise it and the corduroy returns;
    // drop it toward 1.0 and the mid field goes to flat colour.
    uAnisoClamp: { value: pick(opts.anisoClamp, 3.0) },

    uGeoTint: { value: GEO_TINTS.map((c) => new THREE.Vector3(c[0], c[1], c[2])) },
    uGeoScale: { value: new THREE.Vector3(1 / 380, 1 / 210, 1 / 55) },
    uAltRange: { value: new THREE.Vector2(minY, maxY) },
    uMacroTintStrength: { value: pick(opts.macroTintStrength, 0.85) },
    uMacroValueStrength: { value: pick(opts.macroValueStrength, 0.15) },
    uDrainStain: { value: pick(opts.drainStain, 0.45) },

    // R0/A1: the screen-pixel window the per-triangle curvature term is allowed
    // to live in. Fully on below 24 px of triangle edge, fully off above 64 px.
    uCavityTriPx: { value: new THREE.Vector2(
      pick(opts.cavityTriPxOn, 24), pick(opts.cavityTriPxOff, 64)) },
    // A5. Solved against the splat, not guessed. terrain.js paints via aSurface
    // at uSurfaceAttrWeight 1.5, and flat ground already gives DIRT ~1.24. At
    // blend 0.45 DIRT reaches 1.92 against ROCK's 1.50, whose normalised score
    // gap (0.124) exceeds uSplatContrast (0.07) — DIRT would win outright and
    // REPLACE the rock tread instead of showing through it. At 0.30 the gap is
    // 0.059, just inside the contrast band, so the two layers genuinely
    // compete and the winner is decided by their own baked relief: ROCK's
    // heightK is 1.70 against DIRT's 1.05, so proud rock keeps the crests and
    // warm hardpack fills the joints. That is the look the finding asks for.
    uHardpackBlend: { value: pick(opts.hardpackBlend, 0.30) },
    uWindDir: { value: (opts.windDir
      ? new THREE.Vector2(opts.windDir.x, opts.windDir.y).normalize()
      : new THREE.Vector2(0.82, 0.57).normalize()) },
    uSnowEdgeScale: { value: 1 / pick(opts.snowEdgeMetres, 3.5) },

    uSurfaceAttrWeight: { value: pick(opts.surfaceAttrWeight, 1.5) },
    uIdMap: { value: null },
    uIdMapRect: { value: new THREE.Vector4(0, 0, 1, 1) },
    uIdMapTexel: { value: 1 / 256 },
    uIdWeight: { value: pick(opts.idWeight, 1.6) },
  };
}

// ---------------------------------------------------------------------------
// Quality
// ---------------------------------------------------------------------------

// `tileBreakTaps` is the number of stochastic copies blended per fetch.
// 3 is the textbook Heitz/Neyret triangle grid; 2 drops the corner that always
// carries the least barycentric weight and still removes the repetition grid
// entirely, for a third fewer texture fetches on the hottest shader in the
// frame. Only ultra pays for the third tap.
function qualityProfile(ctx, opts) {
  const q = opts.quality || (ctx && ctx.quality) || 'high';
  switch (q) {
    case 'low':
      return { size: 256, detailSize: 128, maxLayers: 2, triplanar: false, tileBreak: false, tileBreakTaps: 2, detail: false, cavity: false };
    case 'medium':
      return { size: 256, detailSize: 256, maxLayers: 3, triplanar: true, tileBreak: true, tileBreakTaps: 2, detail: true, cavity: false };
    case 'ultra':
      return { size: 512, detailSize: 256, maxLayers: 4, triplanar: true, tileBreak: true, tileBreakTaps: 3, detail: true, cavity: true };
    case 'high':
    default:
      // maxLayers 4 -> 3: the fourth layer is measured at 2.31 ms and, with the
      // layer-skip threshold at 0.06, almost never carries a visible weight.
      return { size: 512, detailSize: 256, maxLayers: 3, triplanar: true, tileBreak: true, tileBreakTaps: 2, detail: true, cavity: true };
  }
}

// The FAR variant, for quadtree chunks the camera is never close to. Shares the
// texture set and the uniform bundle with the near material; only the defines
// differ, so it is a second program over the same data.
//
// A2 / R15 — TRIPLANAR STAYS ON. This function previously dropped it, and the
// perf lens was right to flag its own recommendation: the far band begins
// around 110 m, so a 150 m cliff would have gone straight back to planar
// smearing, which is the r2 defect that "triplanar is already engaged" (r2's
// own rejected finding) was proved against. It is also close to free out there:
// the `gTriW.x > 0.004` / `gTriW.z > 0.004` branches in terrSampleLayer skip
// BOTH extra projections on flat ground, and flat ground is what the far band
// is mostly made of. Only tileBreak, detail, cavity and maxLayers come off.
// `triplanar` is inherited rather than forced so the `low` preset, which has no
// triplanar to begin with, is unaffected.
function farProfile(prof) {
  return {
    size: prof.size, detailSize: prof.detailSize,
    maxLayers: 2, triplanar: prof.triplanar, tileBreak: false, tileBreakTaps: 2,
    detail: false, cavity: false,
  };
}

// The MID variant — the full near shader with the per-triangle curvature term
// compiled out. R0/A1 asks for cavity off at every depth <= 6 as belt and
// braces behind the triangle-size guard; depth 6 is 1.0 m per vertex, so its
// triangles only fall under the 24 px threshold beyond ~37 m, and at exactly
// that boundary the two triangles of a grid quad can still disagree in sign.
// Cheaper to delete it than to defend it.
function midProfile(prof) {
  return Object.assign({}, prof, { cavity: false });
}

function profileForVariant(prof, variant) {
  if (variant === 'far') return farProfile(prof);
  if (variant === 'mid') return midProfile(prof);
  return prof;
}

// terrain.js quadtree geometry, for the cavity guard's vertex spacing.
// WORLD = 3072 m, nodes are WORLD / 2^depth, and every chunk is a G x G vertex
// grid with G = 49, i.e. 48 cells.  depth 7 -> 0.5 m, 6 -> 1.0, 5 -> 2.0,
// 4 -> 4.0, 3 -> 8.0 m per vertex.
// NOTE: the r3 work order quotes 0.375 / 0.75 for depths 7 and 6. Those come
// from a stale comment in terrain.js desiredDepth() which assumes a 65-vertex
// grid; the geometry actually built in buildChunk() is `size / (G - 1)` with
// G = 49. The measured 2.0 / 4.0 / 8.0 for depths 5-3 agree exactly.
const CHUNK_WORLD_M = 3072;
const CHUNK_GRID = 49;
const MAX_CHUNK_DEPTH = 7;
// What the base material assumes about its own triangles until terrain.js calls
// materialForDepth(). It has to be CONSERVATIVE (coarse), because if terrain.js
// is not depth-aware the base material shades every chunk in the world,
// including the depth-5 banks 8 m from the lens that produced R0. Guessing
// 0.5 m here would under-estimate triPx by 4x and let the artefact back in
// between roughly 20 and 75 m.
const DEFAULT_VERTEX_SPACING = 2.0;

function vertexSpacingForDepth(depth, opts) {
  const world = (opts && opts.worldSize) || CHUNK_WORLD_M;
  const grid = (opts && opts.chunkGrid) || CHUNK_GRID;
  const d = Math.max(0, Math.min(MAX_CHUNK_DEPTH, depth | 0));
  return (world / Math.pow(2, d)) / (grid - 1);
}

function applyDefines(material, prof, flags) {
  const d = material.defines;
  d.TERRAIN_MAX_LAYERS = prof.maxLayers;
  d.TERRAIN_TILE_BREAK_TAPS = prof.tileBreakTaps || 3;
  if (prof.triplanar) d.TERRAIN_TRIPLANAR = ''; else delete d.TERRAIN_TRIPLANAR;
  if (prof.tileBreak) d.TERRAIN_TILE_BREAK = ''; else delete d.TERRAIN_TILE_BREAK;
  if (prof.detail) d.TERRAIN_DETAIL = ''; else delete d.TERRAIN_DETAIL;
  if (prof.cavity) d.TERRAIN_CAVITY = ''; else delete d.TERRAIN_CAVITY;
  if (flags.idMap) d.TERRAIN_ID_MAP = ''; else delete d.TERRAIN_ID_MAP;
  if (flags.surfaceAttribute) d.TERRAIN_SURFACE_ATTR = ''; else delete d.TERRAIN_SURFACE_ATTR;
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

/**
 * @param {object} ctx   shared game context (may be null for offline tests)
 * @param {object} [opts]
 *   quality            override ctx.quality
 *   bounds             { minX, maxX, minZ, maxZ, minY, maxY }
 *   materialIds        Uint8Array of SurfaceId, row-major [ j * res + i ]
 *   materialRes        resolution of that array
 *   surfaceAttribute   true if the geometry carries a float `aSurface`
 *   wetness            0..1 (default 0.0 — a weather state, not a look)
 *   snowAmount         0..1 (default 0.5)
 *   variant            undefined | 'mid' | 'far' — set by materialForDepth()
 *   vertexSpacing      metres between adjacent mesh vertices; drives the R0
 *                      cavity guard. Default 2.0 (conservative).
 *   worldSize          terrain.js WORLD, for vertexSpacingForDepth (3072)
 *   chunkGrid          terrain.js G_FINE/G_COARSE, ditto (49)
 *   anisoClamp         max major:minor of the texture sampling footprint
 *                      (default 3.0 — see the r5 A8 CONTRACT-NOTE)
 *   windDir            {x, y} prevailing wind in world xz, for the snow margin
 *   wireframe          debug
 *   ...plus the tuning knobs enumerated in buildUniforms()
 * @returns {THREE.MeshStandardMaterial}
 */
export function createTerrainMaterial(ctx, opts = {}) {
  const prof = profileForVariant(qualityProfile(ctx, opts), opts.variant);
  const seed = ((opts.seed !== undefined ? opts.seed : (ctx && ctx.seed) || 20260726)) >>> 0;

  // PER-MATERIAL, deliberately NOT in the shared bundle: this is the only value
  // that differs between the near / mid / far LOD variants. See the R0 note.
  const uVertexSpacing = {
    value: opts.vertexSpacing !== undefined ? opts.vertexSpacing : DEFAULT_VERTEX_SPACING,
  };
  const texSeed = subSeed(seed, 'terrain-surfaces');

  let aniso = 8;
  try {
    const caps = ctx && ctx.renderer && ctx.renderer.capabilities;
    const max = caps && caps.getMaxAnisotropy ? caps.getMaxAnisotropy() : 0;
    if (max) aniso = Math.min(max, prof.size >= 512 ? 16 : 8);
  } catch (e) { /* renderer not ready — 8 is a safe default */ }

  const textures = buildTextureSet(texSeed, prof.size, prof.detailSize, aniso);
  if (ctx && ctx.debug && ctx.debug.log) {
    ctx.debug.log('terrainMaterial textures', textures.key, (textures.buildMs | 0) + ' ms');
  }

  // Uniforms are shared across every chunk material built for this ctx, so a
  // single setWetness()/setSnow() call moves the whole mountain.
  let bundle = ctx ? UNIFORM_CACHE.get(ctx) : null;
  if (!bundle) {
    bundle = { uniforms: buildUniforms(ctx, textures, opts), materials: new Set(), textures };
    if (ctx) UNIFORM_CACHE.set(ctx, bundle);
  }
  const uniforms = bundle.uniforms;

  // Optional painted-id map, built once per ctx.
  let hasIdMap = !!uniforms.uIdMap.value;
  if (!hasIdMap && opts.materialIds && opts.materialRes) {
    const res = opts.materialRes | 0;
    const b = opts.bounds || (ctx && ctx.terrain && ctx.terrain.bounds);
    if (opts.materialIds.length >= res * res && b && isFinite(b.minX) && isFinite(b.maxX)) {
      uniforms.uIdMap.value = buildIdTexture(opts.materialIds, res);
      uniforms.uIdMapRect.value.set(
        b.minX, b.minZ,
        1 / Math.max(1e-3, b.maxX - b.minX),
        1 / Math.max(1e-3, b.maxZ - b.minZ),
      );
      uniforms.uIdMapTexel.value = 1 / res;
      hasIdMap = true;
    }
  }

  const flags = { idMap: hasIdMap, surfaceAttribute: !!opts.surfaceAttribute };

  // If terrain.js supplies BOTH a painted id map and a per-vertex id, they
  // would double-count and completely bury the procedural rules (which supply
  // all the sub-metre variation). Split the authority between them.
  if (flags.idMap && flags.surfaceAttribute) {
    if (opts.idWeight === undefined) uniforms.uIdWeight.value = 0.95;
    if (opts.surfaceAttrWeight === undefined) uniforms.uSurfaceAttrWeight.value = 0.85;
  }

  const material = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    roughness: 1.0,
    metalness: 0.0,
    dithering: true,
    side: THREE.FrontSide,
    fog: true,
    envMapIntensity: opts.envMapIntensity !== undefined ? opts.envMapIntensity : 1.0,
    wireframe: !!opts.wireframe,
  });
  material.name = opts.variant ? 'terrain-' + opts.variant : 'terrain';
  material.defines = {};
  applyDefines(material, prof, flags);

  material.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, uniforms);
    // After the bundle, so a shared-bundle key can never shadow it.
    shader.uniforms.uVertexSpacing = uVertexSpacing;

    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\n' + VERT_PARS)
      .replace('#include <worldpos_vertex>', '#include <worldpos_vertex>\n' + VERT_BODY);

    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', '#include <common>\n' + FRAG_PARS)
      .replace('#include <map_fragment>', FRAG_BODY)
      // Roughness comes from the splat, not from a roughnessMap.
      .replace('#include <roughnessmap_fragment>',
        'float roughnessFactor = clamp( terrRough * uRoughnessScale * roughness, terrRoughFloor, 1.0 );')
      // World-space perturbed normal -> view space. `viewMatrix` is one of
      // three's default fragment-stage uniforms.
      .replace('#include <normal_fragment_maps>',
        'normal = normalize( mat3( viewMatrix ) * terrNrmW );')
      // Feed our AO into indirect lighting exactly the way three's aoMap does.
      .replace('#include <aomap_fragment>', /* glsl */`
      {
        float ambientOcclusion = terrAO;
        reflectedLight.indirectDiffuse *= ambientOcclusion;
        #if defined( USE_ENVMAP ) && defined( STANDARD )
          float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
          reflectedLight.indirectSpecular *= computeSpecularOcclusion( dotNV, ambientOcclusion, material.roughness );
        #endif
      }
      `);

    material.userData.shader = shader;
  };

  material.customProgramCacheKey = () => {
    const d = material.defines;
    return 'descent-terrain|' + [
      d.TERRAIN_MAX_LAYERS,
      d.TERRAIN_TILE_BREAK_TAPS,
      d.TERRAIN_TRIPLANAR !== undefined ? 1 : 0,
      d.TERRAIN_TILE_BREAK !== undefined ? 1 : 0,
      d.TERRAIN_DETAIL !== undefined ? 1 : 0,
      d.TERRAIN_CAVITY !== undefined ? 1 : 0,
      d.TERRAIN_ID_MAP !== undefined ? 1 : 0,
      d.TERRAIN_SURFACE_ATTR !== undefined ? 1 : 0,
    ].join(',');
  };

  // ---- public surface -----------------------------------------------------
  const ud = material.userData;
  ud.isTerrainMaterial = true;
  ud.uniforms = uniforms;
  ud.textures = textures;
  ud.profile = prof;
  ud.surfaceDefs = SURFACE_DEFS;

  ud.setTime = (t) => { uniforms.uTime.value = t; };
  ud.setWetness = (w) => { uniforms.uWetness.value = clamp01(w); };
  ud.setSnow = (amount, line, blend) => {
    if (amount !== undefined) uniforms.uSnowAmount.value = clamp01(amount);
    if (line !== undefined) uniforms.uSnowLine.value = line;
    if (blend !== undefined) uniforms.uSnowBlend.value = blend;
  };
  ud.setDetailStrength = (s) => { uniforms.uDetailStrength.value = Math.max(0, s); };
  ud.setRainRipple = (s) => { uniforms.uRainRipple.value = Math.max(0, s); };
  /** Additive linear-albedo lift for one layer. See LOAM_ALBEDO_LIFT. */
  ud.setLayerLift = (id, v) => {
    if (id < 0 || id >= LAYER_COUNT) return;
    uniforms.uLayerLift.value[id] = Math.max(0, v);
  };
  /** Per-layer roughness floor, for re-tuning without touching the tables. */
  ud.setLayerRoughMin = (id, v) => {
    if (id < 0 || id >= LAYER_COUNT) return;
    uniforms.uLayerRoughMin.value[id] = clamp01(v);
  };

  /**
   * The world distance between two adjacent vertices of the mesh this material
   * shades, in metres. This is the ONLY input to the R0 cavity guard, so a
   * caller that builds terrain some other way should set it rather than let the
   * conservative default suppress the term everywhere.
   */
  ud.setVertexSpacing = (m) => {
    if (m > 0) uVertexSpacing.value = m;
  };
  ud.getVertexSpacing = () => uVertexSpacing.value;
  ud.setWindDir = (x, z) => {
    const l = Math.hypot(x, z);
    if (l > 1e-4) uniforms.uWindDir.value.set(x / l, z / l);
  };

  // ---- per-LOD chunk variants (see the CONTRACT-NOTE at the top) -----------
  // Nothing here is built until it is first requested, so a caller that never
  // asks pays nothing — not even a shader compile.
  let midMat = null;
  let farMat = null;
  if (!opts.variant) {
    let depthAware = false;
    /**
     * The material a quadtree chunk at `depth` should use.
     *   depth >= MAX_CHUNK_DEPTH (7)  the near material, cavity on
     *   depth >= `nearDepth` (6)      the mid variant, cavity compiled out
     *   otherwise                     the far variant
     * At most three materials and three programs exist for the whole mountain,
     * all sharing one texture set and one uniform bundle.
     */
    ud.materialForDepth = (depth, nearDepth) => {
      if (!depthAware) {
        // terrain.js is telling us the LOD depth of every chunk, so the base
        // material only ever shades the finest band and can stop assuming the
        // conservative DEFAULT_VERTEX_SPACING.
        depthAware = true;
        uVertexSpacing.value = vertexSpacingForDepth(MAX_CHUNK_DEPTH, opts);
      }
      let d = depth | 0;
      if (!(d >= 0)) d = MAX_CHUNK_DEPTH;
      if (d >= MAX_CHUNK_DEPTH) return material;

      // Nothing left to drop (the `low` preset already has no tile-break, no
      // triplanar and no detail), so a second program would buy nothing.
      const degenerate = !prof.tileBreak && !prof.triplanar && !prof.detail && !prof.cavity;
      if (degenerate) return material;

      const cut = nearDepth === undefined ? 6 : nearDepth;
      const variant = d >= cut ? 'mid' : 'far';
      const spacing = vertexSpacingForDepth(d, opts);
      if (variant === 'mid') {
        // The mid variant exists ONLY to drop the cavity term. If the quality
        // preset never had it (medium and below), mid and near would compile to
        // the same program — hand back the base material instead of doubling
        // the material count for nothing.
        if (!prof.cavity) return material;
        if (!midMat) {
          midMat = createTerrainMaterial(ctx, Object.assign({}, opts,
            { variant: 'mid', vertexSpacing: spacing }));
          ud.midMaterial = midMat;
        }
        return midMat;
      }
      if (!farMat) {
        farMat = createTerrainMaterial(ctx, Object.assign({}, opts,
          { variant: 'far', vertexSpacing: spacing }));
        ud.farMaterial = farMat;
      }
      return farMat;
    };
    ud.midMaterial = null;
    ud.farMaterial = null;
  }
  ud.setLayerScale = (id, macroM, detailM) => {
    if (id < 0 || id >= LAYER_COUNT) return;
    if (macroM) uniforms.uLayerMacro.value[id] = macroM;
    if (detailM) uniforms.uLayerDetail.value[id] = detailM;
  };
  ud.setLayerTint = (id, hex) => {
    if (id < 0 || id >= LAYER_COUNT) return;
    const c = C(hex);
    uniforms.uLayerTint.value[id].set(c[0], c[1], c[2]);
  };
  /** Re-read a quality name and flip the shader features (no texture rebuild). */
  ud.applyQuality = (quality) => {
    const q = quality || (ctx && ctx.quality);
    const p = profileForVariant(qualityProfile(ctx, { quality: q }), opts.variant);
    p.size = prof.size; p.detailSize = prof.detailSize;
    Object.assign(prof, p);
    applyDefines(material, prof, flags);
    material.needsUpdate = true;
    if (midMat && midMat.userData.applyQuality) midMat.userData.applyQuality(q);
    if (farMat && farMat.userData.applyQuality) farMat.userData.applyQuality(q);
  };
  /** Toggle an individual feature: 'triplanar' | 'tileBreak' | 'detail' | 'cavity'. */
  ud.setFeature = (name, on) => {
    if (!(name in prof)) return;
    prof[name] = !!on;
    applyDefines(material, prof, flags);
    material.needsUpdate = true;
  };

  bundle.materials.add(material);

  const baseDispose = material.dispose.bind(material);
  material.dispose = () => {
    // Variants first, so the shared texture set is torn down exactly once, by
    // whichever material happens to empty bundle.materials.
    if (midMat) { const m = midMat; midMat = null; ud.midMaterial = null; m.dispose(); }
    if (farMat) { const f = farMat; farMat = null; ud.farMaterial = null; f.dispose(); }
    bundle.materials.delete(material);
    // Textures are shared; only tear them down when the last chunk goes away.
    if (bundle.materials.size === 0) {
      textures.albedo.dispose();
      textures.nrh.dispose();
      textures.detail.dispose();
      textures.macro.dispose();
      if (uniforms.uIdMap.value) uniforms.uIdMap.value.dispose();
      TEXTURE_CACHE.delete(textures.key);
      if (ctx) UNIFORM_CACHE.delete(ctx);
    }
    baseDispose();
  };

  return material;
}

export default createTerrainMaterial;
