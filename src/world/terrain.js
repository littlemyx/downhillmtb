// DESCENT — terrain.js
// Heightfield-backed alpine mountainside. See CONTRACT.md §3.
//
// CONTRACT-NOTE (terrain → terrainMaterial): `createTerrainMaterial(ctx, opts)` is called
//   once from commit(). `opts` describes the vertex attributes every terrain chunk carries:
//     position  vec3   world space (chunks have an identity matrix)
//     normal    vec3   analytic, from the interpolated heightfield (crack-free across LODs)
//     uv        vec2   world XZ / 32 m  (1 uv unit = 32 m)
//     color     vec3   FULL BASE ALBEDO in LINEAR-sRGB working space (not a multiplier)
//     aSplat    vec4   normalised blend weights [rock, scree, soil, grass], sums to ~1
//     aTerrainExtra vec4 normalised [snow, wetness, trailCarve, curvatureAO]
//     aSurface  float  SurfaceId (0..7) of the dominant material, non-normalised ubyte
//   If the material sets `material.userData.descentTerrainMaterial = true` terrain leaves
//   `vertexColors` alone (the material owns the decision). Otherwise terrain switches
//   `vertexColors` on and whitens `material.color` so the stub still renders a correctly
//   coloured mountain on its own (CONTRACT §10).
//
// CONTRACT-NOTE (terrain → everyone): extra, non-contractual but stable helpers are exposed
//   for sky/water/vegetation: `sampleWetness(x,z)`, `sampleSnow(x,z)`, `sampleCarve(x,z)`,
//   `treelineAt(x,z)`, `snowlineAt(x,z)`, `valleyY`, `creekLevel`, `group`, `chunks`.
//
// CONTRACT-NOTE (terrain → vegetation/water/anyone raycasting): the terrain is a
//   quadtree whose chunk meshes toggle `visible` every frame as the cut moves, and
//   three.js skips invisible objects when raycasting. Do NOT raycast the terrain
//   group — the answer depends on where the camera happens to be. Use
//   `terrain.sampleHeight(x, z)` / `sampleNormal` / `collision.rayDown()`, which
//   read the heightfield directly, are exact, and are far cheaper.
//
// CONTRACT-NOTE (terrain → trail): CarveStamp has no tangent, but berms need a cross-slope
//   direction. Terrain derives a per-stamp tangent by 2-D PCA over neighbouring stamps within
//   9 m and orients it downhill using neighbouring `targetHeight`s, so `bank` (+ = right side
//   high) is applied correctly regardless of the order stamps arrive in.

import * as THREE from 'three';
import { createNoise2D } from 'simplex-noise';
import { makeRng, subSeed, clamp01, lerp, smoothstep } from '../core/rng.js';
import { createTerrainMaterial } from './terrainMaterial.js';

export const Surface = {
  DIRT: 0, LOAM: 1, ROCK: 2, GRAVEL: 3, GRASS: 4, ROOT: 5, MUD: 6, SNOW: 7,
};

// ---------------------------------------------------------------------------
// World tunables
// ---------------------------------------------------------------------------

const WORLD = 3072;          // metres, square. Power-of-two friendly for the quadtree.
const RES = 1537;            // global heightfield samples per axis → 2.0 m / sample
const CELL = WORLD / (RES - 1);
const HALF = WORLD / 2;

const BASE_Y = 1140;         // valley floor datum, metres ASL
const RELIEF = 720;          // vertical scale of the elevation profile

const TREELINE_Y = 1570;     // ± noise
const SNOWLINE_Y = 1700;     // ± noise

// ---------------------------------------------------------------------------
// Far ring — the world continues past the playable square
//
// WORLD is 3072 m across and used to simply stop, so an establishing shot ended
// in a ruled straight line with sky above it. The ring is a coarse polar sheet
// of the *same* generator (baseHeight + the same ridged multifractal) laid from
// the playable square's own edge out to FAR_RING_R, giving a 10.8 km horizon.
//
// FAR_RING_R is bounded by engine.js's CAMERA_FAR = 8000 m: the ring is centred
// on the world origin, a camera can be at most WORLD*sqrt(2)/2 = 2172 m from
// that origin, so the furthest ring vertex any camera can see is
// 5400 + 2172 = 7572 m away — inside the far plane with 400 m of margin. The
// outermost FAR_FADE_BAND metres are additionally forced to the fog colour, so
// the ring's own outer edge dissolves into the horizon rather than terminating.
// ---------------------------------------------------------------------------
const FAR_RING_R = 5400;     // metres, outer radius from the world origin
const FAR_RING_SEGS = 448;   // angular divisions (multiple of 8 so the square's
                             // corners land on vertices). ~19-38 m of spacing
                             // along the seam with the playable edge.
const FAR_RING_RINGS = 22;   // radial divisions
const FAR_BLEND = 300;       // metres over which the far field takes over from
                             // the real (eroded, carved) heightfield
const FAR_SETTLE = 2200;     // metres for the far-field terms to reach full
const FAR_FADE_BAND = 500;   // the last 500 m dissolve into the horizon colour
const FAR_SKIRT = 70;        // downward curtain at the seam with the playable
                             // edge, so a sampling mismatch cannot open a crack

// Detail overlay: 0.35 m/sample, stored as sparse tiles over the trail corridor only.
const DSTEP = 0.35;
const DT_INNER = 63;                 // interior cells owned by one tile
const DTS = DT_INNER + 3;            // 66 samples/tile: 1-sample bicubic border each side
const DGRID = Math.ceil(WORLD / DSTEP);      // global detail sample count per axis
const DTILES = Math.floor(DGRID / DT_INNER) + 1;
const MAX_DETAIL_TILES = 2600;       // hard safety cap (~46 MB)

const DETAIL_R_IN = 16.0;    // corridor radius with full detail
const DETAIL_R_OUT = 26.0;   // corridor radius where detail has faded to zero

// Quadtree
const ROOT_DEPTH = 3;        // 384 m nodes at the coarsest
const MAX_DEPTH = 7;         // 24 m nodes at the finest
// One vertex grid for every depth keeps the LOD ladder a clean 2x (0.5 m/vertex on
// the trail corridor up to 8 m/vertex on the far flanks) and lets every chunk in the
// world share exactly three index buffers. 0.5 m/vertex meets CONTRACT §3; the 0.35 m
// detail heightfield underneath still drives physics and normals at full fidelity,
// and trail.finalize() lays its own tread mesh over the top.
const G_FINE = 49;           // vertex grid for depth 6..7
const G_COARSE = 49;         // vertex grid for depth 3..5

// ---------------------------------------------------------------------------
// LOD ladder — derived from screen-space triangle size, not from trail proximity
//
// The r3 review measured triangles 8 m from the lens carrying 2 m per vertex
// (~225 screen px of edge). That is the structural half of the "translucent
// ghost quad" root cause: terrainMaterial.js's cavity/crest term is a dFdx of a
// smooth-shaded varying, so it is *constant across a triangle* and only means
// anything while the triangle is small on screen. Its guard fades on camera
// distance; the LOD that decides the triangle size did not.
//
// Two things were wrong here and both are fixed:
//
//  1. desiredDepth() point-sampled the corridor distance field at the node
//     centre and its four corners and took the minimum of five values. On a
//     96 m node the trail can run along an edge midpoint, 48 m from every one
//     of those five samples — so the node reported "48 m from the corridor",
//     returned depth 5, and never subdivided. Two metres per vertex with the
//     trail running straight through it, at any camera distance. The field is
//     now minimised over the node's whole footprint.
//
//  2. The bands themselves were authored as trail-proximity radii
//     (7.5 / 22 / 150 / 560 m) with no relation to how large the resulting
//     triangle is on screen. They are now derived: refine to depth D wherever
//     the NEXT COARSER level's vertex spacing would exceed LOD_TARGET_PX.
//
// The camera never leaves the trail corridor, but it is not *on* the centreline
// — the chase boom and the QA poses put it up to ~8 m laterally and a couple of
// metres up — so a point at corridor distance `dc` can be as close as
// `dc - CAM_CORRIDOR_REACH` to the lens. That term is what converts a corridor
// radius (which is all we can know at build time, because the tree is built
// once) into a bound on camera distance.
//
// The runtime cut in selectNode() is already screen-space correct: with
// splitK = 1.15 the drawn node size is ~dist/1.15, so a G=49 patch is a
// constant 900/(48*1.15) = 16 px per vertex edge at every distance. It could
// simply never get there, because the *built* tree bottomed out at depth 5 off
// the corridor and selectNode() has nothing finer to descend into.
// ---------------------------------------------------------------------------
const LOD_FOCAL_PX = 900;      // 1080 / (2 tan(62/2 deg)) — engine.js CAMERA_FOV at 1080p
const LOD_TARGET_PX = 32;      // longest tolerated screen-space vertex edge
const CAM_CORRIDOR_REACH = 12; // metres the camera can stand off the centreline

/** Metres between adjacent vertices of a chunk at `depth`. */
function vertexSpacing(depth) {
  const G = depth >= 6 ? G_FINE : G_COARSE;
  return (WORLD / (1 << depth)) / (G - 1);
}

/**
 * Corridor radius inside which a node must refine to `depth`: the radius at
 * which the next coarser level's vertex spacing would exceed LOD_TARGET_PX on
 * screen for a camera sitting on the corridor.
 *
 * With G = 49 throughout, vertex spacing is 8 / 4 / 2 / 1 / 0.5 m at depths
 * 3..7, so the bands come out at:
 *     depth 4 inside 237 m, 5 inside 124 m, 6 inside 68 m, 7 inside 40 m.
 * (Previously 560 / 150 / 22 / 7.5 m — the two fine bands were 3-5x too tight
 * and the two coarse ones 2-3x too loose.)
 */
function lodBand(depth) {
  return vertexSpacing(depth - 1) * (LOD_FOCAL_PX / LOD_TARGET_PX) + CAM_CORRIDOR_REACH;
}

const LOD_BANDS = (() => {
  const a = new Float64Array(MAX_DEPTH + 1);
  for (let d = ROOT_DEPTH + 1; d <= MAX_DEPTH; d++) a[d] = lodBand(d);
  return a;
})();

// Corridor distance field
const DF_CELL = 8;
const DFN = Math.floor(WORLD / DF_CELL) + 1;
const DF_FAR = 4096;

// Erosion
const DROPLETS = 135000;
const DROPLET_LIFE = 42;
const EROSION_BUDGET_MS = 2000;

// ---------------------------------------------------------------------------
// Colour — everything authored sRGB, stored linear (CONTRACT §0)
// ---------------------------------------------------------------------------

function srgbToLinear(c) {
  return c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4);
}
function linRGB(hex) {
  return [
    srgbToLinear(((hex >> 16) & 255) / 255),
    srgbToLinear(((hex >> 8) & 255) / 255),
    srgbToLinear((hex & 255) / 255),
  ];
}

// Splat basis colours: weathered alpine schist, dry scree, damp forest soil, alpine turf.
const C_ROCK = linRGB(0x6f6b63);
const C_SCREE = linRGB(0x8e8574);
const C_SOIL = linRGB(0x6b5439);
const C_GRASS = linRGB(0x56682e);
const C_SNOW = linRGB(0xe8eff8);
// Far-field only: beyond the playable square there is no vegetation module, so
// the sub-treeline flanks are tinted toward a dark conifer mass by hand or the
// far ring reads as bare turf against a forested foreground.
const C_FOREST = linRGB(0x33402b);

// Canonical splat for each SurfaceId — used where the trail has painted a material.
const SURFACE_SPLAT = [
  [0.04, 0.14, 0.82, 0.00], // DIRT   — trail tread
  [0.02, 0.05, 0.80, 0.13], // LOAM   — forest floor
  [0.92, 0.08, 0.00, 0.00], // ROCK
  [0.18, 0.78, 0.04, 0.00], // GRAVEL — talus / scree
  [0.02, 0.03, 0.15, 0.80], // GRASS
  [0.02, 0.05, 0.81, 0.12], // ROOT
  [0.02, 0.06, 0.92, 0.00], // MUD
  [0.14, 0.16, 0.10, 0.60], // SNOW   — the snow itself rides in aTerrainExtra.x
];

// ---------------------------------------------------------------------------
// Small maths helpers (module scope — no per-call allocation)
// ---------------------------------------------------------------------------

/** Catmull-Rom value at t∈[0,1] between p1 and p2. */
function cr(p0, p1, p2, p3, t) {
  const b = p2 - p0;
  const c = 2 * p0 - 5 * p1 + 4 * p2 - p3;
  const d = -p0 + 3 * p1 - 3 * p2 + p3;
  return 0.5 * (2 * p1 + b * t + c * t * t + d * t * t * t);
}
/** d/dt of the above. */
function crd(p0, p1, p2, p3, t) {
  const b = p2 - p0;
  const c = 2 * p0 - 5 * p1 + 4 * p2 - p3;
  const d = -p0 + 3 * p1 - 3 * p2 + p3;
  return 0.5 * (b + 2 * c * t + 3 * d * t * t);
}

/**
 * Monotone cubic (Fritsch–Carlson) interpolant. Used for the mountain's elevation
 * profile so the valley floor, face and summit shoulder join with continuous slope
 * and *without* the overshoot a plain Catmull-Rom would give (an overshoot here
 * would be a hill in the middle of the descent).
 */
function monotoneSpline(xs, ys) {
  const n = xs.length;
  const dk = new Float64Array(n - 1);
  for (let i = 0; i < n - 1; i++) dk[i] = (ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]);
  const m = new Float64Array(n);
  m[0] = dk[0];
  m[n - 1] = dk[n - 2];
  for (let i = 1; i < n - 1; i++) m[i] = (dk[i - 1] + dk[i]) * 0.5;
  for (let i = 0; i < n - 1; i++) {
    if (dk[i] === 0) { m[i] = 0; m[i + 1] = 0; continue; }
    const a = m[i] / dk[i];
    const b = m[i + 1] / dk[i];
    const s = a * a + b * b;
    if (s > 9) {
      const t = 3 / Math.sqrt(s);
      m[i] = t * a * dk[i];
      m[i + 1] = t * b * dk[i];
    }
  }
  return function evalSpline(x) {
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (xs[mid] <= x) lo = mid; else hi = mid; }
    const h = xs[hi] - xs[lo];
    const t = (x - xs[lo]) / h;
    const t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * ys[lo] + (t3 - 2 * t2 + t) * h * m[lo]
         + (-2 * t3 + 3 * t2) * ys[hi] + (t3 - t2) * h * m[hi];
  };
}

// Elevation profile along +Z (u=0 at the valley end, u=1 at the summit end).
// Chosen so the usable band u∈[0.17, 0.88] — 2130 m of Z — drops ~458 m, which a
// meandering ~2.6 km trail rides out at roughly 420 m of descent (CONTRACT §0).
const PROFILE = monotoneSpline(
  [0.00, 0.08, 0.17, 0.30, 0.45, 0.60, 0.75, 0.88, 1.00],
  [0.000, 0.012, 0.045, 0.150, 0.290, 0.420, 0.560, 0.680, 0.820],
);

// Module-scope scratch — sampleField writes here so the query API allocates nothing.
let _fh = 0, _fgx = 0, _fgz = 0;
const _vScratch = new THREE.Vector3();

// ---------------------------------------------------------------------------

export function createTerrain(ctx) {
  const settings = (ctx && ctx.settings) || {};
  const lodScale = typeof settings.terrainLOD === 'number' ? settings.terrainLOD : 1.0;
  const lowSpec = lodScale < 0.75;

  // Independent noise fields. Order of construction is part of the seed contract.
  const nWarpA = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-warpA')));
  const nWarpB = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-warpB')));
  const nSwell = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-swell')));
  const nRidge = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-ridge')));
  const nRidge2 = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-ridge2')));
  const nRough = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-rough')));
  const nBandP = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-bandP')));
  const nBandQ = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-bandQ')));
  const nOutcrop = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-outcrop')));
  const nMat = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-mat')));
  const nMat2 = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-mat2')));
  const nLine = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-line')));
  const nMacro = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-macro')));
  const nMicro = createNoise2D(makeRng(subSeed(ctx.seed, 'terr-micro')));
  const dropRng = makeRng(subSeed(ctx.seed, 'terr-droplets'));

  const minX = -HALF, maxX = HALF, minZ = -HALF, maxZ = HALF;

  // ---- global field storage ----
  const heights = new Float32Array(RES * RES);
  const materials = new Uint8Array(RES * RES);
  const splatU8 = new Uint8Array(RES * RES * 4);   // rock, scree, soil, grass
  const extraU8 = new Uint8Array(RES * RES * 2);   // snow, wetness
  const hardnessU8 = new Uint8Array(RES * RES);    // resistance to hydraulic erosion
  const talusU8 = new Uint8Array(RES * RES);       // thermal deposition (scree fans)
  const aoU8 = new Uint8Array(RES * RES);          // curvature AO
  const carveU8 = new Uint8Array(RES * RES);       // how strongly the trail reshaped this cell

  const bounds = { minX, maxX, minZ, maxZ, minY: BASE_Y, maxY: BASE_Y + RELIEF };
  const timings = {};
  let creekLevel = BASE_Y + 4;

  const idx = (i, j) => j * RES + i;

  // -------------------------------------------------------------------------
  // Noise primitives
  // -------------------------------------------------------------------------

  function fbm(n, x, z, oct, gain) {
    let sum = 0, amp = 1, f = 1, norm = 0;
    for (let o = 0; o < oct; o++) {
      sum += n(x * f, z * f) * amp;
      norm += amp;
      amp *= gain;
      f *= 2.037;   // irrational-ish lacunarity avoids octave alignment artefacts
    }
    return sum / norm;
  }

  /**
   * Musgrave ridged multifractal, remapped to 0..1. `offset - |n|` folds the noise
   * so zero-crossings become sharp crests; squaring sharpens them further, and the
   * running `weight` makes higher octaves only bite where lower ones are already
   * high — that is what produces branching ridge systems rather than uniform bumps.
   */
  function ridged(n, x, z, oct, lac, gain) {
    let sum = 0, freq = 1, amp = 0.5, weight = 1, norm = 0;
    for (let o = 0; o < oct; o++) {
      let s = 1.0 - Math.abs(n(x * freq, z * freq));
      s *= s;
      s *= weight;
      weight = s * 2.0;
      if (weight > 1) weight = 1; else if (weight < 0) weight = 0;
      sum += s * amp;
      norm += amp;
      freq *= lac;
      amp *= gain;
    }
    return sum / norm;
  }

  function treelineAt(x, z) {
    return TREELINE_Y + nLine(x * 0.00085, z * 0.00085) * 52 + nLine(x * 0.0035 + 9.1, z * 0.0035 - 4.3) * 14;
  }
  function snowlineAt(x, z) {
    return SNOWLINE_Y + nLine(x * 0.00062 + 21.7, z * 0.00062 - 13.4) * 68;
  }

  // -------------------------------------------------------------------------
  // Pass A — base elevation
  // -------------------------------------------------------------------------

  function baseHeight(x, z) {
    const u = (z - minZ) / WORLD;
    const prof = PROFILE(u);

    // Domain warp: rotates and stretches the ridge system so nothing lines up with
    // the sampling grid, and gives the spurs a natural sinuous plan-form.
    const wx = x + nWarpA(x * 0.00042, z * 0.00042) * 175;
    const wz = z + nWarpB(x * 0.00042 + 31.7, z * 0.00042 - 11.3) * 175;

    let h = BASE_Y + prof * RELIEF;

    // Broad swell — cirques, shoulders, the fact that a mountainside is not a plane.
    // Damped near the valley floor so the runout stays flat.
    h += fbm(nSwell, wx * 0.00055, wz * 0.00031, 3, 0.52) * 78 * (0.30 + 0.70 * prof);

    // Primary spur ridges and gullies. Z frequency is roughly half the X frequency,
    // so features are elongated downhill — spurs run down the fall line, as they do
    // on a real face, instead of forming a blobby egg-carton.
    const r1 = ridged(nRidge, wx * 0.00118, wz * 0.00054, 6, 2.06, 0.52);
    const ridgeAmp = 124 * smoothstep(0.03, 0.34, prof);
    h += (r1 - 0.40) * ridgeAmp;

    // Secondary gully network, same anisotropy, an octave band up.
    const r2 = ridged(nRidge2, wx * 0.0034, wz * 0.0016, 3, 2.11, 0.5);
    h += (r2 - 0.46) * 24 * smoothstep(0.05, 0.30, prof);

    // Slope-scale roughness (the micro-roughness octave is added at full grid
    // resolution in passA — everything here is band-limited well below 4 m).
    h += fbm(nRough, wx * 0.0061, wz * 0.0061, 3, 0.5) * 5.9;

    // Floodplain: pull the last few percent of the profile toward a near-flat plain
    // with a gentle meander, so the run has a believable runout and a creek line.
    const floorMask = smoothstep(0.115, 0.020, prof);
    if (floorMask > 0) {
      const plain = BASE_Y + prof * RELIEF
        + nSwell(x * 0.00085 + 55.0, z * 0.00085) * 7.0
        + fbm(nRough, x * 0.006 + 3.0, z * 0.006, 3, 0.5) * 1.4;
      h = lerp(h, plain, floorMask * 0.85);
    }
    return h;
  }

  /**
   * Every term in baseHeight() is band-limited well below 4 m, so it is evaluated
   * on a half-resolution lattice and reconstructed with the same Catmull-Rom
   * bicubic the query path uses (C1, so the upsample introduces no creasing).
   * Only the micro-roughness octave — the one term with structure below 4 m — is
   * evaluated per full-resolution cell. That is a ~3x saving on the single most
   * expensive pass, with no visible difference in the result.
   */
  const CRES = (RES + 1) >> 1;             // 769
  const CCELL = WORLD / (CRES - 1);        // 4.0 m

  function passA() {
    const coarse = new Float32Array(CRES * CRES);
    for (let j = 0; j < CRES; j++) {
      const z = minZ + j * CCELL;
      const row = j * CRES;
      for (let i = 0; i < CRES; i++) coarse[row + i] = baseHeight(minX + i * CCELL, z);
    }

    let lo = Infinity, hi = -Infinity;
    for (let j = 0; j < RES; j++) {
      const z = minZ + j * CELL;
      const row = j * RES;
      // Full-res sample j maps to coarse coordinate j/2 exactly.
      let fz = j * 0.5;
      if (fz < 1) fz = 1; else if (fz > CRES - 3) fz = CRES - 3;
      const cj = fz | 0;
      const tz = fz - cj;
      for (let i = 0; i < RES; i++) {
        const x = minX + i * CELL;
        let fx = i * 0.5;
        if (fx < 1) fx = 1; else if (fx > CRES - 3) fx = CRES - 3;
        const ci = fx | 0;
        const tx = fx - ci;
        const b = (cj - 1) * CRES + (ci - 1);
        const r0 = b, r1 = b + CRES, r2 = b + 2 * CRES, r3 = b + 3 * CRES;
        const v0 = cr(coarse[r0], coarse[r0 + 1], coarse[r0 + 2], coarse[r0 + 3], tx);
        const v1 = cr(coarse[r1], coarse[r1 + 1], coarse[r1 + 2], coarse[r1 + 3], tx);
        const v2 = cr(coarse[r2], coarse[r2 + 1], coarse[r2 + 2], coarse[r2 + 3], tx);
        const v3 = cr(coarse[r3], coarse[r3 + 1], coarse[r3 + 2], coarse[r3 + 3], tx);

        const h = cr(v0, v1, v2, v3, tz)
          + fbm(nRough, x * 0.0295 + 7.3, z * 0.0295 - 2.1, 2, 0.5) * 0.95;
        heights[row + i] = h;
        if (h < lo) lo = h;
        if (h > hi) hi = h;
      }
    }
    bounds.minY = lo;
    bounds.maxY = hi;
  }

  // -------------------------------------------------------------------------
  // Pass B — cliff bands + rock hardness
  //
  // Sedimentary/metamorphic faces band horizontally. We terrace the height toward
  // multiples of a locally-varying strata period, but only where the ground is
  // already steep, so gentle slopes stay smooth and steep faces break into ledges
  // and risers. The strata are given a slight dip so the bands are not dead level.
  // -------------------------------------------------------------------------

  function passB(tmp) {
    for (let j = 0; j < RES; j++) {
      const z = minZ + j * CELL;
      const row = j * RES;
      for (let i = 0; i < RES; i++) {
        const k = row + i;
        const x = minX + i * CELL;
        const h = heights[k];

        const iL = i > 0 ? k - 1 : k, iR = i < RES - 1 ? k + 1 : k;
        const jD = j > 0 ? k - RES : k, jU = j < RES - 1 ? k + RES : k;
        const gx = (heights[iR] - heights[iL]) / ((iR - iL) * CELL || CELL);
        const gz = (heights[jU] - heights[jD]) / (((jU - jD) / RES) * CELL || CELL);
        const st = Math.sqrt(gx * gx + gz * gz);   // tan(slope)

        // Rock outcrop field: independent of slope, so crags appear on moderate
        // ground too. Combined with steepness it gives the hardness map that both
        // resists erosion and drives the ROCK material.
        const outc = nOutcrop(x * 0.00115, z * 0.00115) * 0.55
                   + nOutcrop(x * 0.0046 + 17.0, z * 0.0046 + 5.0) * 0.30
                   + nOutcrop(x * 0.017 - 4.0, z * 0.017 + 11.0) * 0.15;
        const outcropN = clamp01(outc * 0.5 + 0.5);
        const steepN = smoothstep(0.42, 1.05, st);
        let hard = clamp01(steepN * 0.62 + smoothstep(0.52, 0.86, outcropN) * 0.55);
        // Bedrock is exposed near the summit; the valley is alluvium.
        hard *= 0.55 + 0.45 * smoothstep(0.10, 0.70, (h - BASE_Y) / RELIEF);
        hardnessU8[k] = (clamp01(hard) * 255) | 0;

        // Terracing.
        //
        // The phase used to be a single ~310 m plan-view octave, so the terrace
        // boundary was, to within a very slow drift, a level curve of h: the
        // r3 review read that off the left slope of r3_11 as a contour map. It
        // is not a precision artefact — `heights` is Float32 and always has
        // been — it is that nothing broke the contour. Two more plan-view
        // octaves (~53 m and ~13 m) push the phase around laterally so the
        // riser wanders across the fall line, and a fourth field varies the
        // ledge/riser ratio so no two bands have the same section.
        const period = 11.5 + 12.5 * (nBandP(x * 0.00072, z * 0.00072) * 0.5 + 0.5);
        const dip = x * 0.055 + z * 0.032;          // ~3° regional dip
        const phase = nBandQ(x * 0.0032, z * 0.0032) * 0.42
                    + nBandQ(x * 0.019 + 5.0, z * 0.019 - 8.0) * 0.17
                    + nMicro(x * 0.075 + 2.0, z * 0.075 + 9.0) * 0.08;
        const t = (h + dip) / period + phase;
        const f = Math.floor(t);
        const fr = t - f;
        // Ledge (flat) then riser (steep): the smoothstep width sets the ratio.
        const rww = nBandP(x * 0.024 - 3.0, z * 0.024 + 6.0) * 0.10;
        const stepped = smoothstep(0.30 + rww, 0.66 - rww, fr);
        const terr = (f + stepped) * period - dip;

        const cliffMask = smoothstep(0.55, 1.10, st) * (0.30 + 0.70 * outcropN);
        tmp[k] = lerp(h, terr, cliffMask * 0.58);
      }
    }
    heights.set(tmp);
  }

  // -------------------------------------------------------------------------
  // Pass C — hydraulic (droplet) erosion
  //
  // Classic particle model: a droplet follows the gradient, picks up sediment while
  // it accelerates downhill and drops it where it slows or the slope reverses. The
  // erosion rate is scaled by (1 - hardness), so soft ground gullies out while rock
  // ribs survive — that contrast is most of what makes erosion read as geological
  // rather than as a blur filter.
  // -------------------------------------------------------------------------

  const INERTIA = 0.055;
  const CAPACITY = 0.95;
  const MIN_SLOPE = 0.012;
  const ERODE_SPEED = 0.26;
  const DEPOSIT_SPEED = 0.24;
  const EVAPORATE = 0.0165;
  const GRAVITY = 5.0;
  const BRUSH_R = 3;

  function passC(flowAcc) {
    // Precompute the erosion brush (a cone kernel) so material is removed from a
    // patch rather than a single cell — single-cell erosion makes spiky noise.
    const bx = [], bz = [], bw = [];
    let wsum = 0;
    for (let dz = -BRUSH_R; dz <= BRUSH_R; dz++) {
      for (let dx = -BRUSH_R; dx <= BRUSH_R; dx++) {
        const d = Math.sqrt(dx * dx + dz * dz);
        if (d > BRUSH_R) continue;
        const w = 1 - d / BRUSH_R;
        bx.push(dx); bz.push(dz); bw.push(w); wsum += w;
      }
    }
    const bn = bw.length;
    for (let b = 0; b < bn; b++) bw[b] /= wsum;

    let total = Math.round(DROPLETS * (lowSpec ? 0.45 : 1));
    const t0 = Date.now();
    let checked = false;

    for (let d = 0; d < total; d++) {
      // Deterministic time guard: if this machine is far slower than expected we
      // drop to a fixed fraction of the droplet budget rather than an arbitrary
      // count, so the result stays reproducible in tiers.
      if (!checked && d === 20000) {
        checked = true;
        const projected = (Date.now() - t0) * (total / 20000);
        if (projected > EROSION_BUDGET_MS) {
          total = projected > EROSION_BUDGET_MS * 2 ? Math.round(total * 0.25) : Math.round(total * 0.5);
          if (total < 20000) total = 20000;
        }
      }

      let px = 2 + dropRng() * (RES - 5);
      let pz = 2 + dropRng() * (RES - 5);
      let dx = 0, dz = 0, speed = 1, water = 1, sed = 0;

      for (let life = 0; life < DROPLET_LIFE; life++) {
        const ix = px | 0, iz = pz | 0;
        if (ix < 1 || iz < 1 || ix >= RES - 2 || iz >= RES - 2) break;
        const fx = px - ix, fz = pz - iz;
        const k = iz * RES + ix;
        const h00 = heights[k], h10 = heights[k + 1];
        const h01 = heights[k + RES], h11 = heights[k + RES + 1];

        const gx = (h10 - h00) * (1 - fz) + (h11 - h01) * fz;
        const gz = (h01 - h00) * (1 - fx) + (h11 - h10) * fx;
        const hOld = h00 * (1 - fx) * (1 - fz) + h10 * fx * (1 - fz)
                   + h01 * (1 - fx) * fz + h11 * fx * fz;

        dx = dx * INERTIA - gx * (1 - INERTIA);
        dz = dz * INERTIA - gz * (1 - INERTIA);
        const dl = Math.sqrt(dx * dx + dz * dz);
        if (dl < 1e-6) break;
        dx /= dl; dz /= dl;
        px += dx; pz += dz;

        const nx = px | 0, nz = pz | 0;
        if (nx < 1 || nz < 1 || nx >= RES - 2 || nz >= RES - 2) break;
        const nfx = px - nx, nfz = pz - nz;
        const nk = nz * RES + nx;
        const hNew = heights[nk] * (1 - nfx) * (1 - nfz) + heights[nk + 1] * nfx * (1 - nfz)
                   + heights[nk + RES] * (1 - nfx) * nfz + heights[nk + RES + 1] * nfx * nfz;
        const dh = hNew - hOld;

        // Drainage record — where water repeatedly passes is where mud and creeks go.
        flowAcc[k] += water * (0.35 + speed);

        const cap = Math.max(-dh, MIN_SLOPE) * speed * water * CAPACITY;

        if (sed > cap || dh > 0) {
          // Deposit. Uphill steps get back-filled (up to the step height) which is
          // what carves flat-bottomed valleys and builds alluvial fans.
          const amt = dh > 0 ? Math.min(dh, sed) : (sed - cap) * DEPOSIT_SPEED;
          sed -= amt;
          heights[k] += amt * (1 - fx) * (1 - fz);
          heights[k + 1] += amt * fx * (1 - fz);
          heights[k + RES] += amt * (1 - fx) * fz;
          heights[k + RES + 1] += amt * fx * fz;
        } else {
          const soft = 1 - (hardnessU8[k] / 255) * 0.82;
          let amt = Math.min((cap - sed) * ERODE_SPEED * soft, -dh);
          if (amt > 0.35) amt = 0.35;           // never punch a hole in one step
          if (amt > 0) {
            for (let b = 0; b < bn; b++) {
              const cx = ix + bx[b], cz = iz + bz[b];
              if (cx < 0 || cz < 0 || cx >= RES || cz >= RES) continue;
              heights[cz * RES + cx] -= bw[b] * amt;
            }
            sed += amt;
          }
        }

        speed = Math.sqrt(Math.max(0, speed * speed - dh * GRAVITY));
        water *= 1 - EVAPORATE;
        if (water < 0.02) break;
      }
    }
    timings.droplets = total;
  }

  // -------------------------------------------------------------------------
  // Pass D — thermal erosion (angle of repose)
  //
  // Anything steeper than its repose angle slumps. The repose angle is taken from
  // the hardness map (rock stands at ~62°, soil at ~34°), so cliff bands survive
  // while everything below them collects the debris — which is exactly how talus
  // fans form. The moved volume is recorded and becomes the GRAVEL material.
  // -------------------------------------------------------------------------

  function passD(delta, passes) {
    const talus = new Float32Array(RES * RES);
    const diag = Math.SQRT1_2;
    for (let p = 0; p < passes; p++) {
      delta.fill(0);
      for (let j = 1; j < RES - 1; j++) {
        const row = j * RES;
        for (let i = 1; i < RES - 1; i++) {
          const k = row + i;
          const h = heights[k];
          const hard = hardnessU8[k] / 255;
          // tan(34°)=0.675 … tan(62°)=1.88
          const maxDrop = (0.675 + 1.205 * hard) * CELL;

          let bestK = -1, bestDrop = 0, bestScale = 1;
          for (let o = 0; o < 8; o++) {
            const ox = (o === 0 || o === 4 || o === 5) ? -1 : (o === 2 || o === 6 || o === 7) ? 1 : 0;
            const oz = (o === 1 || o === 4 || o === 7) ? -1 : (o === 3 || o === 5 || o === 6) ? 1 : 0;
            const nk = k + oz * RES + ox;
            const sc = (ox && oz) ? diag : 1;
            const drop = (h - heights[nk]) * sc;
            if (drop > bestDrop) { bestDrop = drop; bestK = nk; bestScale = sc; }
          }
          if (bestK >= 0 && bestDrop > maxDrop * bestScale) {
            const move = (bestDrop - maxDrop * bestScale) * 0.42;
            delta[k] -= move;
            delta[bestK] += move;
          }
        }
      }
      for (let k = 0; k < heights.length; k++) {
        const dv = delta[k];
        if (dv !== 0) {
          heights[k] += dv;
          if (dv > 0) talus[k] += dv;
        }
      }
    }
    // Normalise deposition to 0..255. Talus depths of ~1.5 m read as a full fan.
    for (let k = 0; k < talus.length; k++) {
      talusU8[k] = (clamp01(talus[k] / 1.5) * 255) | 0;
    }
  }

  // -------------------------------------------------------------------------
  // Pass E — selective smoothing
  //
  // Droplet erosion leaves single-cell speckle on soft ground. A hardness-weighted
  // 3×3 blur removes it without touching rock.
  // -------------------------------------------------------------------------

  function passE(tmp) {
    tmp.set(heights);
    for (let j = 1; j < RES - 1; j++) {
      const row = j * RES;
      for (let i = 1; i < RES - 1; i++) {
        const k = row + i;
        const w = (1 - hardnessU8[k] / 255) * 0.55;
        if (w <= 0.02) continue;
        const avg = (tmp[k - 1] + tmp[k + 1] + tmp[k - RES] + tmp[k + RES]) * 0.15
                  + (tmp[k - RES - 1] + tmp[k - RES + 1] + tmp[k + RES - 1] + tmp[k + RES + 1]) * 0.06
                  + tmp[k] * 0.16;
        heights[k] = lerp(tmp[k], avg, w);
      }
    }
  }

  // -------------------------------------------------------------------------
  // Pass G — mesostructure: benches, rock steps, boulder swells, runnels
  //
  // Everything above this pass is either regional (the ridge system, and passB's
  // cliff bands at a 12–24 m strata period) or sub-metre (the micro-roughness
  // octave). Between the two there was nothing at all, which is why fourteen
  // review frames contained no bench, no rock step, no talus apron and no
  // boulder over 0.4 m: every feature at human-to-vehicle scale was band-limited
  // out of existence before the mesh was ever built.
  //
  // Three fields that this generator was already computing had no consumer
  // whatsoever — `hardnessU8` (only erosion read it), `talusU8` (nothing read
  // it) and the droplet drainage accumulator (only the material pass read it).
  // This pass consumes all three and puts 0.4–1.2 m of landform back into the
  // 6–16 m band, which is the finest the 2.0 m global grid can carry without
  // aliasing. Anything shorter than ~6 m lives in the 0.35 m detail overlay
  // instead (see initDetailTiles), so a rock step there always belongs to a
  // bench here.
  //
  // Runs after the smoothing pass (which would otherwise erase it) and before
  // the material pass (so slope-driven materials see the new relief).
  // -------------------------------------------------------------------------

  function passG(flowAcc, tmp) {
    // Same mean-relative soft knee passF uses, so "flow" means the same thing
    // in both passes.
    let sum = 0, n = 0;
    for (let k = 0; k < flowAcc.length; k += 7) { const v = flowAcc[k]; if (v > 0) { sum += v; n++; } }
    const K = Math.max(1e-3, (n > 0 ? sum / n : 1) * 5.0);

    tmp.set(heights);
    for (let j = 1; j < RES - 1; j++) {
      const z = minZ + j * CELL;
      const row = j * RES;
      for (let i = 1; i < RES - 1; i++) {
        const k = row + i;
        const x = minX + i * CELL;
        const h = tmp[k];

        const gx = (tmp[k + 1] - tmp[k - 1]) / (2 * CELL);
        const gz = (tmp[k + RES] - tmp[k - RES]) / (2 * CELL);
        const st = Math.sqrt(gx * gx + gz * gz);   // tan(slope)

        const hard = hardnessU8[k] / 255;
        const tal = talusU8[k] / 255;
        const flow = flowAcc[k] / (flowAcc[k] + K);

        let d = 0;

        // --- benches and rock steps on hard, steep ground -------------------
        // Strata-aligned exactly like passB's cliff bands (same regional dip) so
        // the two read as the same rock, but an order of magnitude finer: a 7–15
        // m riser carrying 0.4–1.2 m of relief. `(smoothstep - 0.5)` pushes the
        // surface onto the nearest tread, which flattens ledges and steepens
        // risers — a terrace, not a wobble.
        const stepMask = smoothstep(0.34, 0.72, hard) * smoothstep(0.30, 0.80, st);
        if (stepMask > 0.01) {
          const period = 7.0 + 8.0 * (nBandP(x * 0.0016 + 4.0, z * 0.0016 - 9.0) * 0.5 + 0.5);
          const dip = x * 0.055 + z * 0.032;
          // Same fix as passB: a plan-view octave at ~19 m so the bench edge
          // wanders across the fall line instead of tracing a height contour.
          const ph = nBandQ(x * 0.0075 + 12.0, z * 0.0075 + 3.0) * 0.5
                   + nMicro(x * 0.052 - 21.0, z * 0.052 + 33.0) * 0.22;
          const t = (h + dip) / period + ph;
          const fr = t - Math.floor(t);
          d += (smoothstep(0.28, 0.72, fr) - 0.5) * period * 0.12 * stepMask;
        }

        // --- talus swells ---------------------------------------------------
        // The thermal pass already worked out where the debris aprons are; give
        // them lumps you can see instead of a smooth ramp. Positive lobes are
        // squared (proud boulder mounds), negative lobes are damped (a talus fan
        // does not have pits in it).
        const talMask = smoothstep(0.12, 0.46, tal) * (0.45 + 0.55 * smoothstep(0.20, 0.70, st));
        if (talMask > 0.01) {
          const b1 = nMicro(x * 0.090 + 61.0, z * 0.090 - 29.0);   // ~11 m
          const b2 = nMicro(x * 0.160 - 17.0, z * 0.160 + 44.0);   // ~6.3 m
          let dome = b1 * 0.68 + b2 * 0.32;
          dome = dome > 0 ? dome * dome * 1.8 : dome * 0.35;
          d += dome * 1.15 * talMask;
        }

        // --- drainage runnels ------------------------------------------------
        // Where the droplets actually ran, incised in soft ground and not in
        // rock. This is what makes a gully read as cut rather than as a fold.
        d -= smoothstep(0.30, 0.80, flow) * (1 - hard * 0.75)
           * smoothstep(0.06, 0.30, st) * 0.85;

        // The runout has to stay rideable and the creek has to stay in its bed,
        // so the whole pass fades out on the valley floor.
        heights[k] = h + d * smoothstep(0.02, 0.10, (h - BASE_Y) / RELIEF);
      }
    }
  }

  // -------------------------------------------------------------------------
  // Pass F — material, splat weights, wetness, snow, curvature AO
  //
  // These four channels are what physics, audio, particles and the terrain shader
  // all read, so the rules have to be geologically motivated rather than arbitrary:
  //   rock   — slope over ~38° or exposed bedrock
  //   scree  — thermal deposition (below cliffs) and loose steep ground above treeline
  //   grass  — gentle ground below the snowline
  //   soil   — everything else, richest below the treeline
  //   mud    — high drainage accumulation on gentle ground
  //   snow   — above the snowline, slope-limited, collecting in hollows
  // -------------------------------------------------------------------------

  function passF(flowAcc) {
    // Normalise drainage accumulation. A mean-relative soft knee keeps a handful of
    // huge trunk streams from flattening every tributary to zero.
    let sum = 0, n = 0;
    for (let k = 0; k < flowAcc.length; k += 7) { const v = flowAcc[k]; if (v > 0) { sum += v; n++; } }
    const meanFlow = n > 0 ? sum / n : 1;
    const K = Math.max(1e-3, meanFlow * 5.0);

    let creekSum = 0, creekN = 0;

    for (let j = 0; j < RES; j++) {
      const z = minZ + j * CELL;
      const row = j * RES;
      for (let i = 0; i < RES; i++) {
        const k = row + i;
        const x = minX + i * CELL;
        const h = heights[k];

        const iL = i > 0 ? k - 1 : k, iR = i < RES - 1 ? k + 1 : k;
        const jD = j > 0 ? k - RES : k, jU = j < RES - 1 ? k + RES : k;
        const gx = (heights[iR] - heights[iL]) / (Math.max(1, iR - iL) * CELL);
        const gz = (heights[jU] - heights[jD]) / (Math.max(1, (jU - jD) / RES) * CELL);
        const st = Math.sqrt(gx * gx + gz * gz);

        // Discrete Laplacian → local concavity, used for snow loading.
        const lap = (heights[iL] + heights[iR] + heights[jD] + heights[jU] - 4 * h) / (CELL * CELL);
        const conc = clamp01(0.5 + lap * 26);

        // Ambient occlusion wants concavity at the scale you can *see* — a gully or
        // the inside of a ledge, not 2 m noise — so it compares the height against a
        // neighbourhood 12 m out rather than against the immediate 4-neighbours.
        const S = 6;
        const wi = i >= S ? -S : S, wI = i < RES - S ? S : -S;
        const wj = j >= S ? -S * RES : S * RES, wJ = j < RES - S ? S * RES : -S * RES;
        const wide = (heights[k + wi] + heights[k + wI] + heights[k + wj] + heights[k + wJ]) * 0.25 - h;
        aoU8[k] = (clamp01(0.5 + wide * 0.10) * 255) | 0;

        const hard = hardnessU8[k] / 255;
        const tal = talusU8[k] / 255;
        const flow = flowAcc[k] / (flowAcc[k] + K);   // 0..1 soft knee
        // Single-octave treeline/snowline here (this pass runs 2.4 M times); the
        // public treelineAt/snowlineAt keep the extra octave for placement queries.
        const tl = TREELINE_Y + nLine(x * 0.00085, z * 0.00085) * 52;
        const sl = SNOWLINE_Y + nLine(x * 0.00062 + 21.7, z * 0.00062 - 13.4) * 68;

        // Organic boundary jitter so material edges are ragged, never contour lines.
        const j1 = nMat(x * 0.0135, z * 0.0135) * 0.5 + 0.5;
        const j2 = nMat2(x * 0.0042, z * 0.0042) * 0.5 + 0.5;
        const j3 = 1 - j1 * 0.55 - j2 * 0.45 + nMat2(x * 0.052 + 60, z * 0.052 - 22) * 0.5;

        // Rock: past ~28° the soil mantle can no longer hold, and bedrock ribs
        // show through anywhere the hardness field is high.
        let wRock = smoothstep(0.52, 0.96, st) * (0.40 + 0.60 * hard) * 1.45
                  + smoothstep(0.52, 0.86, hard) * smoothstep(0.26, 0.58, st) * 1.20;
        wRock *= 0.80 + 0.40 * j1;

        // Scree: thermal deposition below cliffs, loose steep soft ground, and the
        // frost-shattered debris above the treeline.
        let wScree = smoothstep(0.10, 0.42, tal) * 1.55
                   + smoothstep(0.40, 0.74, st) * (1 - hard) * 0.95
                   + smoothstep(tl + 10, tl + 130, h) * smoothstep(0.14, 0.46, st) * 1.0;
        wScree *= 0.78 + 0.44 * j2;

        let wGrass = (smoothstep(0.80, 0.24, st) * 1.45)
                   * smoothstep(sl + 30, sl - 170, h)
                   * (0.50 + 0.50 * j2);
        // Alpine turf carries well above the treeline, then gives out to scree.
        wGrass *= 0.42 + 0.58 * smoothstep(tl + 270, tl - 70, h);

        let wSoil = 0.09
                  + smoothstep(1.00, 0.30, st) * 0.42
                  + smoothstep(tl + 60, tl - 210, h) * 0.52
                  + flow * 0.55;
        wSoil *= 0.82 + 0.36 * j3;

        if (wRock < 0) wRock = 0;
        if (wScree < 0) wScree = 0;
        if (wGrass < 0) wGrass = 0;
        if (wSoil < 0) wSoil = 0;

        // Rock and scree suppress the softer covers rather than merely out-weighing
        // them — a cliff face is bare, not 40% grass.
        const bare = clamp01(wRock * 0.9 + wScree * 0.5);
        wGrass *= 1 - bare * 0.92;
        wSoil *= 1 - bare * 0.80;

        const tot = wRock + wScree + wSoil + wGrass || 1;
        const sr = wRock / tot, sc = wScree / tot, ss = wSoil / tot, sg = wGrass / tot;

        const s4 = k * 4;
        splatU8[s4] = (sr * 255) | 0;
        splatU8[s4 + 1] = (sc * 255) | 0;
        splatU8[s4 + 2] = (ss * 255) | 0;
        splatU8[s4 + 3] = (sg * 255) | 0;

        // Snow: above the line, slope-limited (it slides off anything steep) and
        // wind-loaded into hollows.
        let snow = smoothstep(sl - 70, sl + 40, h)
                 * (1 - smoothstep(0.66, 1.05, st))
                 * (0.55 + 0.75 * conc);
        snow = clamp01(snow * (0.80 + 0.40 * j2));

        // Wetness: drainage lines, damp ground, and the valley floor.
        const wet = clamp01(flow * (1.25 - 0.6 * smoothstep(0.15, 0.6, st))
                  + smoothstep(0.09, 0.015, (h - BASE_Y) / RELIEF) * 0.35);

        const e2 = k * 2;
        extraU8[e2] = (snow * 255) | 0;
        extraU8[e2 + 1] = (wet * 255) | 0;

        // Dominant surface id, with the physical overrides layered on top.
        let mat;
        if (sr >= sc && sr >= ss && sr >= sg) mat = Surface.ROCK;
        else if (sc >= ss && sc >= sg) mat = Surface.GRAVEL;
        else if (sg >= ss) mat = Surface.GRASS;
        else mat = (h < tl + 20) ? Surface.LOAM : Surface.DIRT;

        if (wet > 0.66 && st < 0.34 && mat !== Surface.ROCK) mat = Surface.MUD;
        if (snow > 0.55) mat = Surface.SNOW;
        // Root mats under the forest canopy — small, noisy patches on gentle ground.
        if (mat === Surface.LOAM && st < 0.50 && j3 > 0.79 && h < tl) mat = Surface.ROOT;
        materials[k] = mat;

        if (wet > 0.75 && st < 0.12) { creekSum += h; creekN++; }
      }
    }
    if (creekN > 0) creekLevel = creekSum / creekN;
  }

  // -------------------------------------------------------------------------
  // buildBase — orchestration
  // -------------------------------------------------------------------------

  let baseBuilt = false;

  function buildBase() {
    if (baseBuilt) return;
    const t0 = Date.now();
    passA();
    const t1 = Date.now();

    const tmp = new Float32Array(RES * RES);
    passB(tmp);
    const t2 = Date.now();

    const flowAcc = new Float32Array(RES * RES);
    passC(flowAcc);
    const t3 = Date.now();

    passD(tmp, lowSpec ? 2 : 3);
    const t4 = Date.now();

    passE(tmp);
    const t5 = Date.now();

    passG(flowAcc, tmp);
    const t5b = Date.now();

    passF(flowAcc);
    const t6 = Date.now();

    // Refresh the vertical bounds after erosion moved material around.
    let lo = Infinity, hi = -Infinity;
    for (let k = 0; k < heights.length; k++) {
      const h = heights[k];
      if (h < lo) lo = h;
      if (h > hi) hi = h;
    }
    bounds.minY = lo;
    bounds.maxY = hi;

    timings.base = t1 - t0;
    timings.cliffs = t2 - t1;
    timings.hydraulic = t3 - t2;
    timings.thermal = t4 - t3;
    timings.smooth = t5 - t4;
    timings.mesostructure = t5b - t5;
    timings.materials = t6 - t5b;
    timings.buildBase = t6 - t0;

    baseBuilt = true;
    if (ctx.debug) ctx.debug.log('terrain.buildBase', timings, bounds);
  }

  // -------------------------------------------------------------------------
  // Detail overlay storage (sparse 0.35 m tiles over the trail corridor)
  // -------------------------------------------------------------------------

  const tileIndex = new Int32Array(DTILES * DTILES).fill(-1);
  let dHeights = null;     // Float32Array(nTiles * DTS * DTS)
  let dMaterial = null;    // Uint8Array
  let dWeight = null;      // Uint8Array — corridor blend weight
  let dCarve = null;       // Uint8Array — trail carve strength
  let nTiles = 0;
  let detailReady = false;

  // Corridor distance field (metres to the trail centreline), 8 m grid.
  let corridorDF = null;

  // -------------------------------------------------------------------------
  // Sampling — allocation-free. sampleField() writes _fh/_fgx/_fgz.
  //
  // The global grid is reconstructed with a Catmull-Rom bicubic rather than
  // bilinear: bilinear is only C0, and the resulting derivative discontinuities
  // show up as a diamond-quilt pattern once chunk geometry is finer than the
  // heightfield, and as a 2 m ticking through the bike's suspension. Bicubic gives
  // a C1 field, so normals are smooth and free.
  // -------------------------------------------------------------------------

  function globalField(x, z) {
    let fx = (x - minX) / CELL;
    let fz = (z - minZ) / CELL;
    if (fx < 1) fx = 1; else if (fx > RES - 3) fx = RES - 3;
    if (fz < 1) fz = 1; else if (fz > RES - 3) fz = RES - 3;
    const i = fx | 0, j = fz | 0;
    const tx = fx - i, tz = fz - j;
    const b = (j - 1) * RES + (i - 1);

    const r0 = b, r1 = b + RES, r2 = b + 2 * RES, r3 = b + 3 * RES;
    const a0 = heights[r0], a1 = heights[r0 + 1], a2 = heights[r0 + 2], a3 = heights[r0 + 3];
    const b0 = heights[r1], b1 = heights[r1 + 1], b2 = heights[r1 + 2], b3 = heights[r1 + 3];
    const c0 = heights[r2], c1 = heights[r2 + 1], c2 = heights[r2 + 2], c3 = heights[r2 + 3];
    const d0 = heights[r3], d1 = heights[r3 + 1], d2 = heights[r3 + 2], d3 = heights[r3 + 3];

    const v0 = cr(a0, a1, a2, a3, tx), v1 = cr(b0, b1, b2, b3, tx);
    const v2 = cr(c0, c1, c2, c3, tx), v3 = cr(d0, d1, d2, d3, tx);
    _fh = cr(v0, v1, v2, v3, tz);

    const g0 = crd(a0, a1, a2, a3, tx), g1 = crd(b0, b1, b2, b3, tx);
    const g2 = crd(c0, c1, c2, c3, tx), g3 = crd(d0, d1, d2, d3, tx);
    _fgx = cr(g0, g1, g2, g3, tz) / CELL;
    _fgz = crd(v0, v1, v2, v3, tz) / CELL;
  }

  /** Tile lookup for a world position. Returns the tile slot or -1. */
  function tileAt(gx, gz) {
    const ti = Math.floor(gx / DT_INNER);
    const tj = Math.floor(gz / DT_INNER);
    if (ti < 0 || tj < 0 || ti >= DTILES || tj >= DTILES) return -1;
    return tileIndex[tj * DTILES + ti];
  }

  // Detail sampling results, written by detailField().
  let _dh = 0, _dgx = 0, _dgz = 0, _dw = 0, _dwx = 0, _dwz = 0;

  function detailField(x, z) {
    const gx = (x - minX) / DSTEP;
    const gz = (z - minZ) / DSTEP;
    const ti = Math.floor(gx / DT_INNER);
    const tj = Math.floor(gz / DT_INNER);
    if (ti < 0 || tj < 0 || ti >= DTILES || tj >= DTILES) { _dw = 0; return false; }
    const slot = tileIndex[tj * DTILES + ti];
    if (slot < 0) { _dw = 0; return false; }

    // Local coordinates inside the tile; the tile owns [ti*DT_INNER-1 .. +DTS-2].
    const lx = gx - (ti * DT_INNER - 1);
    const lz = gz - (tj * DT_INNER - 1);
    const i = lx | 0, j = lz | 0;
    const tx = lx - i, tz = lz - j;

    const base = slot * DTS * DTS;
    const w00 = dWeight[base + j * DTS + i] / 255;
    const w10 = dWeight[base + j * DTS + i + 1] / 255;
    const w01 = dWeight[base + (j + 1) * DTS + i] / 255;
    const w11 = dWeight[base + (j + 1) * DTS + i + 1] / 255;
    const wa = w00 + (w10 - w00) * tx;
    const wb = w01 + (w11 - w01) * tx;
    _dw = wa + (wb - wa) * tz;
    if (_dw <= 0.0005) { _dw = 0; return false; }
    // Gradient of the blend weight — needed so the normal stays continuous where
    // the detail overlay fades back into the global field.
    _dwx = (((w10 - w00) * (1 - tz)) + ((w11 - w01) * tz)) / DSTEP;
    _dwz = (((w01 - w00) * (1 - tx)) + ((w11 - w10) * tx)) / DSTEP;

    const r0 = base + (j - 1) * DTS + (i - 1);
    const r1 = r0 + DTS, r2 = r0 + 2 * DTS, r3 = r0 + 3 * DTS;
    const a0 = dHeights[r0], a1 = dHeights[r0 + 1], a2 = dHeights[r0 + 2], a3 = dHeights[r0 + 3];
    const b0 = dHeights[r1], b1 = dHeights[r1 + 1], b2 = dHeights[r1 + 2], b3 = dHeights[r1 + 3];
    const c0 = dHeights[r2], c1 = dHeights[r2 + 1], c2 = dHeights[r2 + 2], c3 = dHeights[r2 + 3];
    const d0 = dHeights[r3], d1 = dHeights[r3 + 1], d2 = dHeights[r3 + 2], d3 = dHeights[r3 + 3];

    const v0 = cr(a0, a1, a2, a3, tx), v1 = cr(b0, b1, b2, b3, tx);
    const v2 = cr(c0, c1, c2, c3, tx), v3 = cr(d0, d1, d2, d3, tx);
    _dh = cr(v0, v1, v2, v3, tz);

    const g0 = crd(a0, a1, a2, a3, tx), g1 = crd(b0, b1, b2, b3, tx);
    const g2 = crd(c0, c1, c2, c3, tx), g3 = crd(d0, d1, d2, d3, tx);
    _dgx = cr(g0, g1, g2, g3, tz) / DSTEP;
    _dgz = crd(v0, v1, v2, v3, tz) / DSTEP;
    return true;
  }

  /** Blended height + analytic gradient. Result in _fh/_fgx/_fgz. */
  function sampleField(x, z) {
    globalField(x, z);
    if (!detailReady) return;
    const hg = _fh, ggx = _fgx, ggz = _fgz;
    if (!detailField(x, z)) return;
    const w = _dw;
    const dh = _dh - hg;
    _fh = hg + dh * w;
    // d/dx[(1-w)hg + w hd] = ggx + (dgx-ggx)w + (hd-hg)dw/dx
    _fgx = ggx + (_dgx - ggx) * w + dh * _dwx;
    _fgz = ggz + (_dgz - ggz) * w + dh * _dwz;
  }

  function sampleHeight(x, z) {
    sampleField(x, z);
    return _fh;
  }

  function sampleNormal(x, z, out) {
    const o = out || _vScratch;
    sampleField(x, z);
    // Surface y = h(x,z) ⇒ n ∝ (-∂h/∂x, 1, -∂h/∂z)
    const nx = -_fgx, nz = -_fgz;
    const inv = 1 / Math.sqrt(nx * nx + 1 + nz * nz);
    o.x = nx * inv; o.y = inv; o.z = nz * inv;
    return o;
  }

  function sampleSlope(x, z) {
    sampleField(x, z);
    return Math.atan(Math.sqrt(_fgx * _fgx + _fgz * _fgz));
  }

  function globalCellIndex(x, z) {
    let i = Math.round((x - minX) / CELL);
    let j = Math.round((z - minZ) / CELL);
    if (i < 0) i = 0; else if (i > RES - 1) i = RES - 1;
    if (j < 0) j = 0; else if (j > RES - 1) j = RES - 1;
    return j * RES + i;
  }

  function sampleMaterial(x, z) {
    if (detailReady) {
      const gx = (x - minX) / DSTEP;
      const gz = (z - minZ) / DSTEP;
      const ti = Math.floor(gx / DT_INNER);
      const tj = Math.floor(gz / DT_INNER);
      if (ti >= 0 && tj >= 0 && ti < DTILES && tj < DTILES) {
        const slot = tileIndex[tj * DTILES + ti];
        if (slot >= 0) {
          const i = Math.round(gx - (ti * DT_INNER - 1));
          const j = Math.round(gz - (tj * DT_INNER - 1));
          if (i >= 0 && j >= 0 && i < DTS && j < DTS) {
            const p = slot * DTS * DTS + j * DTS + i;
            if (dWeight[p] > 128) return dMaterial[p];
          }
        }
      }
    }
    return materials[globalCellIndex(x, z)];
  }

  /** 0..1 — how strongly the trail carve reshaped this point. */
  function sampleCarve(x, z) {
    if (detailReady) {
      const gx = (x - minX) / DSTEP;
      const gz = (z - minZ) / DSTEP;
      const ti = Math.floor(gx / DT_INNER);
      const tj = Math.floor(gz / DT_INNER);
      if (ti >= 0 && tj >= 0 && ti < DTILES && tj < DTILES) {
        const slot = tileIndex[tj * DTILES + ti];
        if (slot >= 0) {
          const lx = gx - (ti * DT_INNER - 1);
          const lz = gz - (tj * DT_INNER - 1);
          const i = lx | 0, j = lz | 0;
          const tx = lx - i, tz = lz - j;
          const b = slot * DTS * DTS + j * DTS + i;
          const c00 = dCarve[b], c10 = dCarve[b + 1];
          const c01 = dCarve[b + DTS], c11 = dCarve[b + DTS + 1];
          const ca = c00 + (c10 - c00) * tx;
          const cb = c01 + (c11 - c01) * tx;
          return (ca + (cb - ca) * tz) / 255;
        }
      }
    }
    return carveU8[globalCellIndex(x, z)] / 255;
  }

  /** Bilinear fetch from one of the global Uint8 channel arrays. */
  function bilinearU8(arr, stride, off, x, z) {
    let fx = (x - minX) / CELL;
    let fz = (z - minZ) / CELL;
    if (fx < 0) fx = 0; else if (fx > RES - 1.001) fx = RES - 1.001;
    if (fz < 0) fz = 0; else if (fz > RES - 1.001) fz = RES - 1.001;
    const i = fx | 0, j = fz | 0;
    const tx = fx - i, tz = fz - j;
    const k = (j * RES + i) * stride + off;
    const v00 = arr[k], v10 = arr[k + stride];
    const v01 = arr[k + RES * stride], v11 = arr[k + RES * stride + stride];
    const a = v00 + (v10 - v00) * tx;
    const b = v01 + (v11 - v01) * tx;
    return (a + (b - a) * tz) / 255;
  }

  function sampleWetness(x, z) { return bilinearU8(extraU8, 2, 1, x, z); }
  function sampleSnow(x, z) { return bilinearU8(extraU8, 2, 0, x, z); }

  function inBounds(x, z) {
    return x >= minX && x <= maxX && z >= minZ && z <= maxZ;
  }

  // -------------------------------------------------------------------------
  // applyCarve — the trail reshapes the mountain
  // -------------------------------------------------------------------------

  /**
   * Radial shaping weight. A trail tread is a flat bench, not a crown, so the core
   * is flat and only the outer part of the radius ramps. Beyond the tread radius a
   * weaker, wider "shoulder" term eases the bench into the hillside — that is what
   * produces the cut bank above the trail and the fill slope below it.
   */
  function carveShape(dn, falloff) {
    if (dn >= 1.9) return 0;
    let core = 0;
    if (dn < 1) {
      const a = clamp01((1 - dn) * 2);
      core = a * a * (3 - 2 * a);
      const f = falloff > 0 ? Math.min(4, falloff) : 2;
      if (f !== 2) core = Math.pow(core, f * 0.5);
    }
    const sa = clamp01((1.9 - dn) / 1.4);
    const shoulder = sa * sa * (3 - 2 * sa) * 0.40;
    return core > shoulder ? core : shoulder;
  }

  /**
   * Cross-section of a stamp at lateral offset `s` (metres, + = rider's right).
   * `bank` is the cross-slope in radians; berms additionally get a parabolic outer
   * wall so the section is genuinely banked and supportive, not a tilted plane.
   */
  function stampTarget(st, sRaw, r) {
    // Clamp just outside the tread: past that the shoulder term is easing the bench
    // into the hillside, and a berm's parabola must not keep climbing out there.
    const lim = r * 1.15;
    const s = sRaw > lim ? lim : (sRaw < -lim ? -lim : sRaw);
    const bank = st.bank || 0;
    const tb = Math.tan(bank);
    let y = st.targetHeight;
    switch (st.kind) {
      case 'berm': {
        y += s * tb;
        const outside = bank >= 0 ? (s > 0 ? s : 0) : (s < 0 ? s : 0);
        // Rise curves up toward the outside; scaled by how hard the turn is banked.
        y += outside * outside * 0.085 * clamp01(Math.abs(bank) * 3.2);
        break;
      }
      case 'rut':
        y += s * tb;
        y -= 0.24 * Math.exp(-(s * s) / 0.55);
        break;
      case 'wallride':
        y += s * tb;
        y += Math.max(0, s) * Math.max(0, s) * 0.22;
        break;
      case 'lip':
      case 'landing':
      case 'drop':
      case 'tread':
      default:
        y += s * tb * 0.9;
        y -= Math.abs(s) * 0.022;   // slight outslope so the tread sheds water
        break;
    }
    return y;
  }

  // ---- lateral noise on the carve width -----------------------------------
  // One world-space octave at ~7.4 m, +/-13%. Deliberately one octave and not
  // two: this is evaluated per (stamp, cell) in the 0.35 m detail loop, which is
  // tens of millions of calls, and a second octave at ~2 m barely survives the
  // 2 m global grid anyway.
  const RW_MAX = 1.14;           // upper bound of carveWidth(), used to size the loops
  function carveWidth(x, z) {
    return 1 + nMat(x * 0.135 + 311.0, z * 0.135 - 77.0) * 0.13;
  }

  // ---- cut/fill batter ----------------------------------------------------
  // A carve stamp pulled the ground straight to the tread height out to ~1.9
  // radii and then stopped. On a steep face that is a boolean subtraction: the
  // r3 review measured 4-5 m near-vertical walls with a razor-straight top edge
  // tracking the spline (r3_08, r3_09, r3_11). A real bench does not do that —
  // the cut face stands at the material's angle of repose, its top rounds over,
  // and the debris that came off it collects as a wedge at the foot.
  // The work order asks for "taper to angle of repose (35-40 deg soil)". That is
  // the ANGLE OF THE RESULT, not the asymptote to author: a hyperbola asymptotic
  // to tan(37 deg) cut into a 31 deg hillside does not daylight for 8 m, and into
  // a 45 deg hillside never. Cross-sections simulated over the real stamp layout
  // (centre + 4 laterals at 0.4 m stations) give the measured face angle as:
  //     hillside 24 deg: was 69 deg at 1.25 m -> 38 deg at 2.45 m
  //     hillside 31 deg: was 74 deg at 1.25 m -> 46 deg at 3.10 m
  //     hillside 45 deg: was 80 deg at 1.25 m -> 60 deg at 4.25 m
  // i.e. the asked-for 35-40 deg on moderate ground, and on genuinely steep
  // ground a 60 deg bank pushed 4 m out from the tread instead of an 80 deg one
  // rising straight off it. A short cohesive cut standing steeper than the
  // ambient hillside is also what a real full-bench trail looks like.
  const REPOSE_CUT = 1.30;       // tan(52 deg) asymptote — see above
  const REPOSE_FILL = 0.62;      // tan(31.8 deg) — loose fill stands shallower
  const SHOULDER_ROUND = 0.85;   // metres over which the top of the cut rounds off
  const TALUS_WEDGE = 0.30;      // max metres of debris at the foot of the cut
  const FILL_MAX = 0.7;          // max metres of fill before a bench goes full-cut
  const BATTER_RUN = 4.0;        // metres of cut/fill slope beyond the tread edge
  const BATTER_RELEASE = 2.8;    // metres over which it eases into the hillside
  // The batter's weight in the target MEAN. The blend STRENGTH out on the batter
  // has to reach 1 (see batterWeight), but if it also carried weight 1 in the
  // mean then a stamp battering across a switchback's other leg would out-vote
  // that leg's own tread. 0.12 lets the batter own cells no tread stamp reaches
  // while leaving a real tread stamp an ~8:1 majority wherever they overlap.
  const BATTER_MEAN_W = 0.12;

  /**
   * Batter the carve target outside the tread edge.
   *
   * `over` is metres beyond the tread edge, `hNat` the natural (pre-carve)
   * ground there, `y` the stamp's tread target. The profile is a hyperbola
   * tangent to the bench at over = 0 and asymptotic to the repose slope, so the
   * shoulder is rounded over ~0.85 m rather than creased, and it is clamped
   * against the natural ground so a carve can never raise a cut or lower a
   * fill. Depends only on (position, stamp), so it stays independent of the
   * order stamps arrive in — which is the property the whole accumulator is
   * built around.
   */
  function batter(y, hNat, over) {
    if (over <= 0) return y;
    const round = Math.sqrt(over * over + SHOULDER_ROUND * SHOULDER_ROUND) - SHOULDER_ROUND;
    if (hNat > y) {
      const cut = hNat - y;
      // Talus wedge: material that has slumped off the batter and piled at its
      // foot. Peaks ~0.5 m out from the tread edge and scales with cut height.
      const d = over - 0.5;
      const wedge = Math.min(TALUS_WEDGE, cut * 0.10) * Math.exp(-(d * d) / 0.30);
      const t = y + REPOSE_CUT * round + wedge;
      return t < hNat ? t : hNat;
    }
    // Fill side. A bench cut across a steep fall-away is built full-cut, not by
    // throwing a five-metre embankment out over the void, so the fill is limited
    // to FILL_MAX and refuses entirely (continuously, no step) by twice that.
    const t = y - REPOSE_FILL * round;
    const depth = t - hNat;
    if (depth <= 0) return hNat;
    if (depth >= FILL_MAX * 2) return hNat;
    return hNat + depth * (depth < FILL_MAX ? 1 : (FILL_MAX * 2 - depth) / FILL_MAX);
  }

  /**
   * Blend strength for the batter region, as a function of metres outside the
   * tread edge and the stamp's own along-track coordinate.
   *
   * The batter target is clamped against the natural ground, so applying it at
   * full strength is self-limiting: where the cut has already daylighted, the
   * target IS the natural ground and full strength changes nothing. That is what
   * makes it safe to override `carveShape`'s outer ramp here — and it has to be
   * overridden, because that ramp is what produced the wall. carveShape hands
   * the shoulder only 0.28 of the way to its target at one radius and 0.08 at
   * 1.5, so with the old flat target the ground out there stayed within a few
   * percent of the untouched hillside and the bench simply ended in a face.
   *
   * The release is deliberately wide (2.8 m of a 4.0 m run): it is a lerp toward
   * the natural ground, so whatever cut has not daylighted by then is spread
   * over 2.8 m rather than dumped in one step. Narrowing it to 1.1 m moved the
   * 45 deg-hillside face back from 60 deg to 78 deg — the release width is what
   * does most of the work on steep ground, not the repose constant.
   *
   * Extending BATTER_RUN further is a straight trade against carve build time:
   * the stamp loop's footprint grows with its square.
   */
  function batterWeight(over, uaAbs) {
    if (over > BATTER_RUN || uaAbs >= 1.9) return 0;
    let t = over <= BATTER_RUN - BATTER_RELEASE ? 1 : (BATTER_RUN - over) / BATTER_RELEASE;
    const a = (1.9 - uaAbs) / 0.55;
    if (a < t) t = a;
    if (t <= 0) return 0;
    if (t > 1) t = 1;
    return t * t * (3 - 2 * t);
  }

  // Per-cell stamp evaluation, shared by the global and detail carve loops so
  // the two fields cannot drift apart. Results go into module-scope scalars —
  // this runs tens of millions of times and must not allocate.
  let _scTarget = 0, _scW = 0, _scWH = 0, _scWM = 0, _scRW = 0;

  /**
   * Evaluate one stamp at one cell. Returns false if the cell is outside this
   * stamp's influence, otherwise writes _scTarget / _scW / _scWH / _scWM / _scRW.
   *   _scW  — carve shape weight: material paint, carveU8, micro-relief gates
   *   _scWM — weight in the height target MEAN
   *   _scWH — blend STRENGTH (max over stamps)
   */
  function stampCell(st, lat, along, r, rl, x, z, batterKind, hNat) {
    const uaAbs = (along < 0 ? -along : along) / rl;
    if (uaAbs >= 1.9) return false;
    const latAbs = lat < 0 ? -lat : lat;
    const rMax = r * RW_MAX;
    const ur = lat / rMax;
    // Cheap reject. `ur` uses the widest the jittered tread can be, so
    // ur^2 + ua^2 >= 3.61 implies dn >= 1.9 and cannot false-reject.
    if (ur * ur + uaAbs * uaAbs >= 3.61
        && !(batterKind && latAbs <= rMax + BATTER_RUN)) return false;

    // Lateral noise on the cut width: the carve radius was a constant per stamp,
    // so the trench ran the whole trail at a dead-uniform width with its top
    // edge exactly offset from the spline. Faded out before the batter's own
    // run so the batter geometry cannot inherit a step where the fade ends, and
    // a function of world position only, so overlapping stamps agree on it and
    // the accumulator stays independent of the order stamps arrive in.
    let rw = r;
    if (latAbs < r * 2.2) {
      const jf = latAbs < r * 1.4 ? 1 : (r * 2.2 - latAbs) / (r * 0.8);
      rw = r * (1 + (carveWidth(x, z) - 1) * jf);
    }

    const ul = lat / rw, ua = along / rl;
    const dn = Math.sqrt(ul * ul + ua * ua);
    const w = carveShape(dn, st.falloff);
    let target = stampTarget(st, lat, rw);
    let wh = w, wm = w;
    if (batterKind) {
      target = batter(target, hNat, latAbs - rw);
      const bw = batterWeight(latAbs - rw, uaAbs);
      if (bw > wh) wh = bw;
      wm += bw * BATTER_MEAN_W;
    }
    if (wh <= 0.002) return false;
    _scTarget = target; _scW = w; _scWH = wh; _scWM = wm; _scRW = rw;
    return true;
  }

  // ---- ridden-tread microrelief -------------------------------------------
  // r2 authored the ruts 0.028 m deep at sigma 0.13 m on a 0.35 m detail grid —
  // roughly one sample wide, so the reconstruction filter erased them before
  // they ever reached a vertex. The r3 review measured the tread varying LESS
  // across the track than down it (0.57-0.99, where a ridden line reads
  // 1.5-2.5), i.e. isotropic noise rather than a line.
  const RUT_DEPTH = 0.11;        // metres (work order: 0.08-0.15)
  // sigma 0.33 m rather than the work order's 0.35: FWHM 0.78 m is 2.2 detail
  // samples and 1.6 render vertices, still comfortably reconstructable, and the
  // narrower profile leaves a 72 mm crown between the two ruts where 0.35 leaves
  // 54 mm — a wider sigma at this separation starts merging the pair into one
  // broad trough, which is the opposite of a ridden line.
  const RUT_SIG2 = 2 * 0.33 * 0.33;
  const RUT_HALF = 0.62;         // half the track separation: 1.24 m apart
  const WINDROW = 0.075;         // berm of displaced material at the tread margin
  const WINDROW_SIG2 = 2 * 0.22 * 0.22;
  const BRAKE_BUMP = 0.085;      // metres (work order: 0.06-0.12)

  /** Per-stamp working data: unit tangent (downhill), arc length, local grade. */
  function prepareStamps(stamps) {
    const n = stamps.length;
    const tanX = new Float32Array(n);
    const tanZ = new Float32Array(n);
    const arc = new Float32Array(n);
    const grade = new Float32Array(n);
    const spacing = new Float32Array(n);

    // Spatial bins so the PCA neighbourhood search stays O(n). The trail can hand
    // us tens of thousands of stamps, so the per-stamp neighbourhood is also capped:
    // 20 samples is far more than a principal axis needs, and without the cap a
    // densely-stamped trail turns this into an O(n * density) blow-up.
    const MAXN = 20;
    const nbX = new Float64Array(MAXN);
    const nbZ = new Float64Array(MAXN);
    const nbH = new Float64Array(MAXN);
    const BIN = 8;
    const bins = new Map();
    const binKey = (x, z) => (Math.floor((z - minZ) / BIN) * 4096 + Math.floor((x - minX) / BIN));
    for (let i = 0; i < n; i++) {
      const st = stamps[i];
      const key = binKey(st.x, st.z);
      let list = bins.get(key);
      if (!list) { list = []; bins.set(key, list); }
      list.push(i);
    }

    // 6.5 m: wide enough to fix a tangent, tight enough that a switchback's other
    // leg does not contaminate the principal axis.
    const R2 = 6.5 * 6.5;
    for (let i = 0; i < n; i++) {
      const st = stamps[i];
      const bx = Math.floor((st.x - minX) / BIN);
      const bz = Math.floor((st.z - minZ) / BIN);
      let sxx = 0, sxz = 0, szz = 0, count = 0;
      let gLo = st.targetHeight, gHi = st.targetHeight, gDist = 1;
      let nearest2 = Infinity;

      outer:
      for (let oz = -1; oz <= 1; oz++) {
        for (let ox = -1; ox <= 1; ox++) {
          const list = bins.get((bz + oz) * 4096 + (bx + ox));
          if (!list) continue;
          // Stride rather than truncate: bin lists are in stamp order, so an evenly
          // spread sample spans the trail segment in that bin and gives a stable
          // principal axis. Taking the first N would bias the axis to one side.
          const stride = list.length > 7 ? Math.ceil(list.length / 7) : 1;
          for (let m = 0; m < list.length; m += stride) {
            const jj = list[m];
            if (jj === i) continue;
            const o = stamps[jj];
            const dx = o.x - st.x, dz = o.z - st.z;
            const d2 = dx * dx + dz * dz;
            if (d2 > R2 || d2 < 1e-6) continue;
            if (d2 < nearest2) nearest2 = d2;
            sxx += dx * dx; sxz += dx * dz; szz += dz * dz;
            nbX[count] = dx; nbZ[count] = dz; nbH[count] = o.targetHeight;
            count++;
            if (o.targetHeight < gLo) { gLo = o.targetHeight; gDist = Math.sqrt(d2); }
            if (o.targetHeight > gHi) gHi = o.targetHeight;
            if (count >= MAXN) break outer;
          }
        }
      }

      let tx, tz;
      if (count >= 2) {
        // Principal axis of a 2x2 symmetric covariance matrix.
        const theta = 0.5 * Math.atan2(2 * sxz, sxx - szz);
        tx = Math.cos(theta); tz = Math.sin(theta);
        // Orient downhill: sum of (this - neighbour height) * (offset . tangent).
        let orient = 0;
        for (let m = 0; m < count; m++) {
          orient += (st.targetHeight - nbH[m]) * (nbX[m] * tx + nbZ[m] * tz);
        }
        if (orient < 0) { tx = -tx; tz = -tz; }
      } else {
        // Isolated stamp: fall back to the local fall line.
        sampleField(st.x, st.z);
        const gl = Math.sqrt(_fgx * _fgx + _fgz * _fgz);
        if (gl > 1e-4) { tx = -_fgx / gl; tz = -_fgz / gl; } else { tx = 0; tz = -1; }
      }
      tanX[i] = tx; tanZ[i] = tz;
      grade[i] = clamp01((gHi - gLo) / Math.max(1, gDist));
      // Along-trail reach. If the trail hands us stamps further apart than their
      // radius, circular stamps would leave a plateau-and-riser staircase in the
      // tread; stretching each stamp along its tangent to cover the gap makes
      // consecutive targets blend into a continuous ramp instead.
      // Array-order neighbours give the exact gap for sequentially-emitted stamps;
      // the (strided, so approximate) spatial nearest is only the fallback.
      let gap = Infinity;
      if (i > 0) {
        const d = Math.hypot(stamps[i - 1].x - st.x, stamps[i - 1].z - st.z);
        if (d > 1e-4 && d < 30) gap = d;
      }
      if (i < n - 1) {
        const d = Math.hypot(stamps[i + 1].x - st.x, stamps[i + 1].z - st.z);
        if (d > 1e-4 && d < 30 && d < gap) gap = d;
      }
      if (gap === Infinity) gap = nearest2 < Infinity ? Math.sqrt(nearest2) : 0;
      spacing[i] = gap;
    }

    // Arc length by array order, restarting whenever the trail jumps.
    let a = 0;
    for (let i = 0; i < n; i++) {
      if (i > 0) {
        const dx = stamps[i].x - stamps[i - 1].x;
        const dz = stamps[i].z - stamps[i - 1].z;
        const d = Math.sqrt(dx * dx + dz * dz);
        a = d < 30 ? a + d : 0;
      }
      arc[i] = a;
    }
    return { tanX, tanZ, arc, grade, spacing };
  }

  /** Chamfer distance transform → metres from the trail centreline, on an 8 m grid. */
  function buildCorridorField(stamps) {
    const df = new Float32Array(DFN * DFN).fill(DF_FAR);
    for (let s = 0; s < stamps.length; s++) {
      const st = stamps[s];
      if (!st || !isFinite(st.x) || !isFinite(st.z)) continue;
      const ci = Math.round((st.x - minX) / DF_CELL);
      const cj = Math.round((st.z - minZ) / DF_CELL);
      for (let oj = -2; oj <= 2; oj++) {
        const j = cj + oj;
        if (j < 0 || j >= DFN) continue;
        for (let oi = -2; oi <= 2; oi++) {
          const i = ci + oi;
          if (i < 0 || i >= DFN) continue;
          const dx = (minX + i * DF_CELL) - st.x;
          const dz = (minZ + j * DF_CELL) - st.z;
          const d = Math.sqrt(dx * dx + dz * dz);
          const k = j * DFN + i;
          if (d < df[k]) df[k] = d;
        }
      }
    }
    const A = DF_CELL, B = DF_CELL * Math.SQRT2;
    for (let j = 0; j < DFN; j++) {
      for (let i = 0; i < DFN; i++) {
        const k = j * DFN + i;
        let v = df[k];
        if (i > 0 && df[k - 1] + A < v) v = df[k - 1] + A;
        if (j > 0) {
          if (df[k - DFN] + A < v) v = df[k - DFN] + A;
          if (i > 0 && df[k - DFN - 1] + B < v) v = df[k - DFN - 1] + B;
          if (i < DFN - 1 && df[k - DFN + 1] + B < v) v = df[k - DFN + 1] + B;
        }
        df[k] = v;
      }
    }
    for (let j = DFN - 1; j >= 0; j--) {
      for (let i = DFN - 1; i >= 0; i--) {
        const k = j * DFN + i;
        let v = df[k];
        if (i < DFN - 1 && df[k + 1] + A < v) v = df[k + 1] + A;
        if (j < DFN - 1) {
          if (df[k + DFN] + A < v) v = df[k + DFN] + A;
          if (i < DFN - 1 && df[k + DFN + 1] + B < v) v = df[k + DFN + 1] + B;
          if (i > 0 && df[k + DFN - 1] + B < v) v = df[k + DFN - 1] + B;
        }
        df[k] = v;
      }
    }
    return df;
  }

  function corridorDist(x, z) {
    if (!corridorDF) return DF_FAR;
    let fx = (x - minX) / DF_CELL;
    let fz = (z - minZ) / DF_CELL;
    if (fx < 0) fx = 0; else if (fx > DFN - 1.001) fx = DFN - 1.001;
    if (fz < 0) fz = 0; else if (fz > DFN - 1.001) fz = DFN - 1.001;
    const i = fx | 0, j = fz | 0;
    const tx = fx - i, tz = fz - j;
    const k = j * DFN + i;
    const a = corridorDF[k] + (corridorDF[k + 1] - corridorDF[k]) * tx;
    const b = corridorDF[k + DFN] + (corridorDF[k + DFN + 1] - corridorDF[k + DFN]) * tx;
    return a + (b - a) * tz;
  }

  /** Micro-relief amplitude (metres) for the detail overlay, by surface. */
  function microAmp(mat) {
    switch (mat) {
      case Surface.ROCK: return 0.115;
      case Surface.GRAVEL: return 0.070;
      case Surface.ROOT: return 0.055;
      case Surface.GRASS: return 0.038;
      case Surface.LOAM: return 0.034;
      case Surface.MUD: return 0.026;
      case Surface.SNOW: return 0.018;
      default: return 0.030;
    }
  }

  function allocateDetailTiles(stamps) {
    const touched = new Set();
    for (let s = 0; s < stamps.length; s++) {
      const st = stamps[s];
      if (!st || !isFinite(st.x) || !isFinite(st.z)) continue;
      const R = Math.max(DETAIL_R_OUT + 2, (st.radius || 2) * 2.0 + 4);
      const gi0 = Math.floor((st.x - R - minX) / DSTEP / DT_INNER);
      const gi1 = Math.floor((st.x + R - minX) / DSTEP / DT_INNER);
      const gj0 = Math.floor((st.z - R - minZ) / DSTEP / DT_INNER);
      const gj1 = Math.floor((st.z + R - minZ) / DSTEP / DT_INNER);
      for (let tj = gj0; tj <= gj1; tj++) {
        if (tj < 0 || tj >= DTILES) continue;
        for (let ti = gi0; ti <= gi1; ti++) {
          if (ti < 0 || ti >= DTILES) continue;
          touched.add(tj * DTILES + ti);
        }
      }
    }
    const keys = Array.from(touched).sort((a, b) => a - b);   // deterministic order
    nTiles = Math.min(keys.length, MAX_DETAIL_TILES);
    if (nTiles === 0) return;
    const cells = nTiles * DTS * DTS;
    dHeights = new Float32Array(cells);
    dMaterial = new Uint8Array(cells);
    dWeight = new Uint8Array(cells);
    dCarve = new Uint8Array(cells);
    for (let t = 0; t < nTiles; t++) tileIndex[keys[t]] = t;
  }

  function initDetailTiles() {
    for (let key = 0; key < tileIndex.length; key++) {
      const slot = tileIndex[key];
      if (slot < 0) continue;
      const ti = key % DTILES;
      const tj = (key / DTILES) | 0;
      const gx0 = ti * DT_INNER - 1;
      const gz0 = tj * DT_INNER - 1;
      const base = slot * DTS * DTS;

      for (let j = 0; j < DTS; j++) {
        const z = minZ + (gz0 + j) * DSTEP;
        for (let i = 0; i < DTS; i++) {
          const x = minX + (gx0 + i) * DSTEP;
          const p = base + j * DTS + i;

          const dc = corridorDist(x, z);
          const w = smoothstep(DETAIL_R_OUT, DETAIL_R_IN, dc);
          dWeight[p] = (clamp01(w) * 255) | 0;

          // Every sample gets a real height even where the blend weight is zero:
          // the bicubic reconstruction reads a 4x4 window, so a single unwritten
          // sample just outside the fade would poison every query near it.
          globalField(x, z);
          const mat = materials[globalCellIndex(x, z)];
          dMaterial[p] = mat;
          if (w <= 0) { dHeights[p] = _fh; continue; }

          // Sub-2-metre relief the global grid cannot hold: stones, tussocks, the
          // pebbly texture of scree. Three octaves at very different scales.
          const amp = microAmp(mat);
          const m1 = nMicro(x * 0.62, z * 0.62);
          const m2 = nMicro(x * 2.35 + 41.0, z * 2.35 - 17.0);
          const m3 = nMicro(x * 7.1 - 8.0, z * 7.1 + 3.0);
          let dh = _fh + (m1 * 0.62 + m2 * 0.28 + m3 * 0.10) * amp;

          // --- landform at human scale ---------------------------------------
          // The corridor overlay samples at 0.35 m, so it is the only field in
          // this generator that can hold a 1–3 m feature: a rock step you would
          // have to ride off, a boulder you would have to go round. Driven off
          // the same hardness / talus fields as passG, so a step here is always
          // part of a bench there. Everything below is gated so the flat, soft,
          // grassy ground the trail mostly runs on is left alone.
          const gk = globalCellIndex(x, z);
          const hardD = hardnessU8[gk] / 255;
          const talD = talusU8[gk] / 255;
          const stD = Math.sqrt(_fgx * _fgx + _fgz * _fgz);
          const rocky = mat === Surface.ROCK ? 1 : (mat === Surface.GRAVEL ? 0.6 : 0);
          // Keep the tread and its immediate margin clean. `dc` is the corridor
          // distance already computed above for the blend weight; the trail's own
          // carve would flatten most of this anyway, but relying on that would
          // put metre-scale rock inside the racing line on any stamp gap.
          const clear = smoothstep(4.5, 11.0, dc);

          // Bedrock ribs: a 1.7–3.2 m strata period carrying 0.6–1.1 m steps.
          const stepMask = clear * smoothstep(0.40, 0.80, hardD) * smoothstep(0.26, 0.75, stD);
          if (stepMask > 0.01) {
            const period = 1.7 + 1.5 * (nBandP(x * 0.010 + 7.0, z * 0.010 - 2.0) * 0.5 + 0.5);
            const t = (_fh + x * 0.055 + z * 0.032) / period + nBandQ(x * 0.06, z * 0.06) * 0.5;
            const fr = t - Math.floor(t);
            dh += (smoothstep(0.30, 0.70, fr) - 0.5) * period * 0.34 * stepMask;
          }

          // Boulder field: only the upper tail of the noise becomes a rock, so
          // the ground between the boulders stays ground rather than turning
          // into an egg carton. Peak dome height 0.72 m at ~2.5 m spacing.
          const boulderMask = clear * clamp01(
            smoothstep(0.18, 0.55, talD) + rocky * 0.55 * smoothstep(0.18, 0.55, stD));
          if (boulderMask > 0.01) {
            const bn = nMicro(x * 0.40 + 91.0, z * 0.40 - 37.0) * 0.68
                     + nMicro(x * 0.93 - 55.0, z * 0.93 + 21.0) * 0.32;
            let dome = (bn - 0.22) / 0.78;
            if (dome > 0) {
              if (dome > 1) dome = 1;
              dh += dome * dome * (3 - 2 * dome) * 0.72 * boulderMask;
            }
            // Cobble course between the boulders. 0.9 => ~1.1 m wavelength,
            // still three samples per cycle at DSTEP, so it does not alias.
            const cob = nMicro(x * 0.90 + 12.0, z * 0.90 - 8.0);
            if (cob > 0.35) dh += (cob - 0.35) * 0.26 * boulderMask;
          }

          dHeights[p] = dh;
        }
      }
    }
  }

  function applyCarve(stamps) {
    if (!baseBuilt) buildBase();
    const list = Array.isArray(stamps) ? stamps.filter(
      (s) => s && isFinite(s.x) && isFinite(s.z) && isFinite(s.targetHeight),
    ) : [];

    const t0 = Date.now();
    tileIndex.fill(-1);
    nTiles = 0;
    detailReady = false;
    corridorDF = buildCorridorField(list);
    if (list.length === 0) { detailReady = false; return; }

    const prep = prepareStamps(list);
    allocateDetailTiles(list);
    if (nTiles === 0) { detailReady = false; return; }
    initDetailTiles();
    const t1 = Date.now();

    // ---- global grid ------------------------------------------------------
    // Two accumulators make the result independent of the order stamps arrive in:
    // the carve *strength* is the maximum any stamp asks for (so the cross-section
    // shape is preserved exactly), while the carve *target* is the weighted mean of
    // every stamp reaching the cell (so the tread interpolates smoothly along the
    // trail instead of stepping from one stamp's plateau to the next).
    const accW = new Float32Array(RES * RES);
    const accWT = new Float32Array(RES * RES);
    const accMax = new Float32Array(RES * RES);

    for (let s = 0; s < list.length; s++) {
      const st = list[s];
      const r = Math.max(0.6, st.radius || 2);
      const rl = Math.min(r * 3, Math.max(r, prep.spacing[s] * 0.85));
      // Only plain benched tread gets the batter. Berms, wallrides, lips,
      // landings and drops are *constructed* features whose flanks are authored
      // by trail.js, and re-cutting them to an angle of repose would flatten the
      // very geometry they exist to make.
      const batterKind = st.kind === undefined || st.kind === 'tread' || st.kind === 'rut';
      // The footprint must cover the carve ellipse, the along-track reach AND
      // the batter's own lateral run.
      const reach = Math.max(r * RW_MAX * 1.9, rl * 1.9,
        batterKind ? r * RW_MAX + BATTER_RUN : 0);
      const tx = prep.tanX[s], tz = prep.tanZ[s];
      const rx = -tz, rz = tx;                      // rider's right

      const i0 = Math.max(0, Math.floor((st.x - reach - minX) / CELL));
      const i1 = Math.min(RES - 1, Math.ceil((st.x + reach - minX) / CELL));
      const j0 = Math.max(0, Math.floor((st.z - reach - minZ) / CELL));
      const j1 = Math.min(RES - 1, Math.ceil((st.z + reach - minZ) / CELL));

      for (let j = j0; j <= j1; j++) {
        const z = minZ + j * CELL;
        const dz = z - st.z;
        for (let i = i0; i <= i1; i++) {
          const x = minX + i * CELL;
          const dx = x - st.x;
          const lat = dx * rx + dz * rz;
          const along = dx * tx + dz * tz;
          const k = j * RES + i;
          // `heights[k]` is still the natural surface: the accumulator is only
          // applied after every stamp has been visited.
          if (!stampCell(st, lat, along, r, rl, x, z, batterKind, heights[k])) continue;
          const w = _scW;
          accW[k] += _scWM;
          accWT[k] += _scWM * _scTarget;
          if (_scWH > accMax[k]) accMax[k] = _scWH;
          // carveU8 feeds sampleCarve(), which vegetation uses to stay off the
          // tread and buildChunk uses to drive the splat toward trail material.
          // It follows the tread weight, not the batter strength, or a 4 m band
          // of hillside would read as trail.
          const cw = (w * 255) | 0;
          if (cw > carveU8[k]) carveU8[k] = cw;

          if (w > 0.5 && st.material !== undefined && st.material !== null) {
            materials[k] = st.material;
            const cs = SURFACE_SPLAT[st.material] || SURFACE_SPLAT[0];
            const s4 = k * 4;
            const blend = Math.min(1, (w - 0.5) * 2.4);
            splatU8[s4] = lerp(splatU8[s4], cs[0] * 255, blend) | 0;
            splatU8[s4 + 1] = lerp(splatU8[s4 + 1], cs[1] * 255, blend) | 0;
            splatU8[s4 + 2] = lerp(splatU8[s4 + 2], cs[2] * 255, blend) | 0;
            splatU8[s4 + 3] = lerp(splatU8[s4 + 3], cs[3] * 255, blend) | 0;
            // A ridden tread is packed and dry.
            const e2 = k * 2;
            extraU8[e2] = (extraU8[e2] * (1 - blend)) | 0;
            extraU8[e2 + 1] = (extraU8[e2 + 1] * (1 - blend * 0.7)) | 0;
          }
        }
      }
    }
    for (let k = 0; k < accW.length; k++) {
      const wsum = accW[k];
      if (wsum === 0) continue;
      heights[k] += (accWT[k] / wsum - heights[k]) * accMax[k];
    }
    const t2 = Date.now();

    // ---- detail overlay ---------------------------------------------------
    const dCells = nTiles * DTS * DTS;
    const dAccW = new Float32Array(dCells);
    const dAccWT = new Float32Array(dCells);
    const dAccMax = new Float32Array(dCells);

    for (let s = 0; s < list.length; s++) {
      const st = list[s];
      const r = Math.max(0.6, st.radius || 2);
      const rl = Math.min(r * 3, Math.max(r, prep.spacing[s] * 0.85));
      const tx = prep.tanX[s], tz = prep.tanZ[s];
      const rx = -tz, rz = tx;
      const arc0 = prep.arc[s];
      const grade = prep.grade[s];
      const isTread = st.kind === 'tread' || st.kind === 'rut' || st.kind === undefined;
      const brake = isTread ? smoothstep(0.11, 0.30, grade) : 0;
      // Only plain benched tread is battered — see the global loop above.
      const batterKind = isTread;
      const reach = Math.max(r * RW_MAX * 1.9, rl * 1.9,
        batterKind ? r * RW_MAX + BATTER_RUN : 0);
      // Edge breakdown is not symmetric on a real trail: the outside of a turn
      // gets scuffed wide by riders running out of it, and the shoulder above a
      // braking zone gets chewed. `bank` is + when the RIGHT side is high, i.e.
      // the right side is the outside of the turn.
      const bank = st.bank || 0;
      const bankSide = bank > 0 ? 1 : (bank < 0 ? -1 : 0);
      const bankStr = clamp01(Math.abs(bank) * 3.0);
      const brakeBias = 0.16 * brake;

      const gi0 = Math.floor((st.x - reach - minX) / DSTEP);
      const gi1 = Math.ceil((st.x + reach - minX) / DSTEP);
      const gj0 = Math.floor((st.z - reach - minZ) / DSTEP);
      const gj1 = Math.ceil((st.z + reach - minZ) / DSTEP);
      const ti0 = Math.floor(gi0 / DT_INNER), ti1 = Math.floor(gi1 / DT_INNER);
      const tj0 = Math.floor(gj0 / DT_INNER), tj1 = Math.floor(gj1 / DT_INNER);

      for (let tj = tj0; tj <= tj1; tj++) {
        if (tj < 0 || tj >= DTILES) continue;
        for (let ti = ti0; ti <= ti1; ti++) {
          if (ti < 0 || ti >= DTILES) continue;
          const slot = tileIndex[tj * DTILES + ti];
          if (slot < 0) continue;
          const ox = ti * DT_INNER - 1;
          const oz = tj * DT_INNER - 1;
          const base = slot * DTS * DTS;
          const li0 = Math.max(0, gi0 - ox), li1 = Math.min(DTS - 1, gi1 - ox);
          const lj0 = Math.max(0, gj0 - oz), lj1 = Math.min(DTS - 1, gj1 - oz);

          for (let j = lj0; j <= lj1; j++) {
            const z = minZ + (oz + j) * DSTEP;
            const dz = z - st.z;
            for (let i = li0; i <= li1; i++) {
              const x = minX + (ox + i) * DSTEP;
              const dx = x - st.x;
              const lat = dx * rx + dz * rz;
              const along = dx * tx + dz * tz;
              const p = base + j * DTS + i;
              // dHeights is still the natural surface here — the accumulator is
              // only applied after every stamp has been visited.
              if (!stampCell(st, lat, along, r, rl, x, z, batterKind, dHeights[p])) continue;
              const w = _scW;
              const rw = _scRW;
              let target = _scTarget;
              const sArc = arc0 + along;

              const cw = (w * 255) | 0;
              if (cw > dCarve[p]) dCarve[p] = cw;

              // Windrow — the material the tyres have pushed out, sitting as a
              // low berm along both tread margins. Nothing was producing this at
              // all, and it is the single largest across-track gradient on a
              // ridden trail. It has to live OUTSIDE the `w > 0.35` core gate:
              // at the tread margin that gate is worth ~0.09, which would have
              // reduced a 75 mm berm to 7 mm and clipped its outer half.
              if (isTread) {
                const e = Math.abs(lat) - rw * 1.05;
                target += WINDROW * Math.exp(-(e * e) / WINDROW_SIG2) * clamp01(w * 2.4);
              }

              if (w > 0.35) {
                const inner = (w - 0.35) / 0.65;
                // Two tyre ruts. Amplitude 0.028 -> 0.11 m and sigma 0.13 ->
                // 0.33 m: below that the feature is one detail sample wide and
                // is filtered away before it reaches a vertex, which is why the
                // tread measured as isotropic noise (across/along gradient
                // 0.57-0.99 where a ridden line reads 1.5-2.5). The pair is
                // widened to 1.24 m apart so the crown between them survives the
                // wider sigma, and the line wanders +/-0.22 m over a ~74 m
                // wavelength so it is a ridden line and not a machined groove.
                if (isTread) {
                  const wander = nMicro(sArc * 0.085, st.x * 0.031) * 0.22;
                  const dl = lat - (RUT_HALF + wander);
                  const dr = lat + (RUT_HALF - wander);
                  const rutL = Math.exp(-(dl * dl) / RUT_SIG2);
                  const rutR = Math.exp(-(dr * dr) / RUT_SIG2);
                  target -= RUT_DEPTH * (rutL + rutR) * inner;
                }
                // Braking bumps: transverse ripples that build on steep entries.
                if (brake > 0) {
                  const jitter = nMicro(sArc * 0.09, st.x * 0.05) * 0.5;
                  const ripple = Math.sin((sArc / 1.15 + jitter) * Math.PI * 2) * 0.5 + 0.5;
                  const across = 1 - clamp01(Math.abs(lat) / (rw * 0.8));
                  target -= BRAKE_BUMP * brake * ripple * across * inner;
                }
                // Loose rock / roots keep their chatter even after the cut.
                const mm = st.material;
                if (mm === Surface.ROCK || mm === Surface.GRAVEL || mm === Surface.ROOT) {
                  const chat = nMicro(x * 1.9 + 3.0, z * 1.9 - 6.0) * 0.6
                             + nMicro(x * 5.3 - 12.0, z * 5.3 + 8.0) * 0.4;
                  target += chat * (mm === Surface.ROCK ? 0.075 : 0.045) * inner;
                }
              }

              dAccW[p] += _scWM;
              dAccWT[p] += _scWM * target;
              if (_scWH > dAccMax[p]) dAccMax[p] = _scWH;

              if (st.material !== undefined && st.material !== null) {
                if (w > 0.50) {
                  dMaterial[p] = st.material;
                } else if (w > 0.10) {
                  // Ragged, scuffed trail edge rather than a clean painted ring.
                  // One octave at nMat(x*0.55) is a ~1.8 m wavelength, which
                  // draws a gentle sine down the trail edge, not raggedness.
                  // Three octaves at ~1.8 m / ~0.4 m / ~0.12 m: the finest is
                  // below the 0.35 m grid, but this is a binary paint decision
                  // per sample rather than a resampled continuous signal, so it
                  // reads as grain on the edge and cannot alias.
                  const n = 0.5 + 0.30 * nMat(x * 0.55, z * 0.55)
                                + 0.14 * nMat2(x * 2.5 + 63.0, z * 2.5 - 21.0)
                                + 0.06 * nMicro(x * 8.3 - 14.0, z * 8.3 + 45.0);
                  const outside = (bankSide !== 0 && (lat > 0 ? 1 : -1) === bankSide)
                    ? 0.20 * bankStr : 0;
                  const t = (w - 0.10) / 0.40 + outside + brakeBias;
                  const m0 = dMaterial[p];
                  if (n < t && (m0 === Surface.GRASS || m0 === Surface.LOAM || m0 === Surface.ROOT)) {
                    dMaterial[p] = Surface.DIRT;
                  }
                }
              }
            }
          }
        }
      }
    }

    for (let p = 0; p < dCells; p++) {
      const wsum = dAccW[p];
      if (wsum === 0) continue;
      dHeights[p] += (dAccWT[p] / wsum - dHeights[p]) * dAccMax[p];
    }

    detailReady = true;

    let lo = Infinity, hi = -Infinity;
    for (let k = 0; k < heights.length; k++) {
      const h = heights[k];
      if (h < lo) lo = h;
      if (h > hi) hi = h;
    }
    bounds.minY = lo;
    bounds.maxY = hi;

    timings.carveInit = t1 - t0;
    timings.carveGlobal = t2 - t1;
    timings.carveDetail = Date.now() - t2;
    timings.detailTiles = nTiles;
    timings.stamps = list.length;
    if (ctx.debug) ctx.debug.log('terrain.applyCarve', timings);
  }

  // -------------------------------------------------------------------------
  // commit — chunked quadtree LOD meshes
  // -------------------------------------------------------------------------

  const group = new THREE.Group();
  group.name = 'terrain';
  group.matrixAutoUpdate = false;
  const chunks = [];
  const roots = [];
  let terrainMat = null;
  let useVertexColorAlbedo = false;

  // Shared index buffers: topology depends only on (gridSize, stride), so every
  // chunk of a given class points at the same GL buffer.
  const indexCache = new Map();

  function buildIndex(G, stride) {
    const key = G * 16 + stride;
    let attr = indexCache.get(key);
    if (attr) return attr;

    const quads = (G - 1) / stride;
    const tris = quads * quads * 2 + quads * 4 * 2;
    const arr = new Uint16Array(tris * 3);
    let p = 0;
    // Surface
    for (let j = 0; j < G - 1; j += stride) {
      for (let i = 0; i < G - 1; i += stride) {
        const a = j * G + i;
        const b = (j + stride) * G + i;
        const c = j * G + i + stride;
        const d = (j + stride) * G + i + stride;
        arr[p++] = a; arr[p++] = b; arr[p++] = c;
        arr[p++] = c; arr[p++] = b; arr[p++] = d;
      }
    }
    // Skirts. Ordering of the four edge blocks matches buildChunk() below.
    const S0 = G * G;          // j = 0      (-Z)
    const S1 = S0 + G;         // j = G-1    (+Z)
    const S2 = S1 + G;         // i = 0      (-X)
    const S3 = S2 + G;         // i = G-1    (+X)
    for (let i = 0; i < G - 1; i += stride) {
      const i2 = i + stride;
      // -Z edge: outward normal -Z
      arr[p++] = i; arr[p++] = i2; arr[p++] = S0 + i;
      arr[p++] = i2; arr[p++] = S0 + i2; arr[p++] = S0 + i;
      // +Z edge: outward normal +Z
      const t0 = (G - 1) * G + i, t1 = (G - 1) * G + i2;
      arr[p++] = t0; arr[p++] = S1 + i; arr[p++] = t1;
      arr[p++] = t1; arr[p++] = S1 + i; arr[p++] = S1 + i2;
    }
    for (let j = 0; j < G - 1; j += stride) {
      const j2 = j + stride;
      // -X edge
      arr[p++] = j * G; arr[p++] = S2 + j; arr[p++] = j2 * G;
      arr[p++] = j2 * G; arr[p++] = S2 + j; arr[p++] = S2 + j2;
      // +X edge
      const u0 = j * G + G - 1, u1 = j2 * G + G - 1;
      arr[p++] = u0; arr[p++] = u1; arr[p++] = S3 + j;
      arr[p++] = u1; arr[p++] = S3 + j2; arr[p++] = S3 + j;
    }
    attr = new THREE.BufferAttribute(arr.subarray(0, p), 1);
    indexCache.set(key, attr);
    return attr;
  }

  // Per-vertex surface data scratch.
  let _vR = 0, _vC = 0, _vS = 0, _vG = 0, _vSnow = 0, _vWet = 0, _vAo = 0;

  function sampleVertexData(x, z) {
    _vR = bilinearU8(splatU8, 4, 0, x, z);
    _vC = bilinearU8(splatU8, 4, 1, x, z);
    _vS = bilinearU8(splatU8, 4, 2, x, z);
    _vG = bilinearU8(splatU8, 4, 3, x, z);
    _vSnow = bilinearU8(extraU8, 2, 0, x, z);
    _vWet = bilinearU8(extraU8, 2, 1, x, z);
    _vAo = bilinearU8(aoU8, 1, 0, x, z);
  }

  function buildChunk(x0, z0, size, depth) {
    const G = depth >= 6 ? G_FINE : G_COARSE;
    const step = size / (G - 1);
    const nv = G * G + 4 * G;

    const pos = new Float32Array(nv * 3);
    const nrm = new Float32Array(nv * 3);
    const uv = new Float32Array(nv * 2);
    // The vertex albedo is only needed when we are shading from vertex colours;
    // a real terrain material ignores it, so we skip ~20 MB of upload.
    const col = useVertexColorAlbedo ? new Float32Array(nv * 3) : null;
    const splat = new Uint8Array(nv * 4);
    const extra = new Uint8Array(nv * 4);
    const surf = new Uint8Array(nv);

    let minY = Infinity, maxY = -Infinity;

    for (let j = 0; j < G; j++) {
      const z = z0 + j * step;
      for (let i = 0; i < G; i++) {
        const x = x0 + i * step;
        const v = j * G + i;

        sampleField(x, z);
        const h = _fh;
        if (h < minY) minY = h;
        if (h > maxY) maxY = h;

        pos[v * 3] = x; pos[v * 3 + 1] = h; pos[v * 3 + 2] = z;
        const inv = 1 / Math.sqrt(_fgx * _fgx + 1 + _fgz * _fgz);
        nrm[v * 3] = -_fgx * inv; nrm[v * 3 + 1] = inv; nrm[v * 3 + 2] = -_fgz * inv;
        uv[v * 2] = x * 0.03125; uv[v * 2 + 1] = z * 0.03125;

        sampleVertexData(x, z);
        const matId = sampleMaterial(x, z);
        const carve = sampleCarve(x, z);

        let sr = _vR, sc = _vC, ss = _vS, sg = _vG;
        if (carve > 0.02) {
          // Where the trail has been cut, drive the blend to that surface's mix.
          const cs = SURFACE_SPLAT[matId] || SURFACE_SPLAT[0];
          const b = Math.min(1, carve * 1.15);
          sr = lerp(sr, cs[0], b); sc = lerp(sc, cs[1], b);
          ss = lerp(ss, cs[2], b); sg = lerp(sg, cs[3], b);
        }
        const tot = sr + sc + ss + sg || 1;
        sr /= tot; sc /= tot; ss /= tot; sg /= tot;

        const wet = _vWet * (1 - carve * 0.6);
        const snow = _vSnow;
        // Curvature AO: gullies and the insides of ledges darken.
        const ao = clamp01(0.55 + (_vAo - 0.5) * 1.25);

        if (col) {
          // ---- albedo -----------------------------------------------------
          // A ridden tread is dust-dry and lighter than the forest floor around it.
          const dry = 1 + carve * 0.30;
          let r = C_ROCK[0] * sr + C_SCREE[0] * sc + C_SOIL[0] * ss * dry + C_GRASS[0] * sg;
          let g = C_ROCK[1] * sr + C_SCREE[1] * sc + C_SOIL[1] * ss * dry + C_GRASS[1] * sg;
          let b2 = C_ROCK[2] * sr + C_SCREE[2] * sc + C_SOIL[2] * ss * dry + C_GRASS[2] * sg;

          // Wet ground darkens and cools.
          const wk = 1 - 0.44 * wet;
          r *= wk; g *= wk * 1.01; b2 *= wk * 1.05;

          if (snow > 0.002) {
            r = lerp(r, C_SNOW[0], snow); g = lerp(g, C_SNOW[1], snow); b2 = lerp(b2, C_SNOW[2], snow);
          }

          // Macro variation at two very different scales kills the "one big painted
          // surface" look long before any texture is involved.
          const m1 = nMacro(x * 0.0031, z * 0.0031);
          const m2 = nMacro(x * 0.021 + 13.0, z * 0.021 - 5.0);
          const macro = 1 + (m1 * 0.085 + m2 * 0.045) * (1 - snow * 0.7);
          r *= macro; g *= macro * (1 + m1 * 0.018); b2 *= macro * (1 - m1 * 0.02);

          const aoK = 0.70 + 0.30 * ao;
          col[v * 3] = r * aoK; col[v * 3 + 1] = g * aoK; col[v * 3 + 2] = b2 * aoK;
        }

        const s4 = v * 4;
        splat[s4] = (sr * 255) | 0; splat[s4 + 1] = (sc * 255) | 0;
        splat[s4 + 2] = (ss * 255) | 0; splat[s4 + 3] = (sg * 255) | 0;
        extra[s4] = (snow * 255) | 0;
        extra[s4 + 1] = (wet * 255) | 0;
        extra[s4 + 2] = (clamp01(carve) * 255) | 0;
        extra[s4 + 3] = (ao * 255) | 0;
        surf[v] = matId;
      }
    }

    // ---- skirts ---------------------------------------------------------
    // Depth scales with chunk size so it always covers the worst-case height error
    // when this chunk drops to a coarser index stride, or meets a finer neighbour.
    const skirt = Math.max(1.2, size * 0.07);
    let w = G * G;
    // The skirt is the only part of a chunk that can be seen from underneath or
    // through an LOD crack, and it used to copy the surface vertex wholesale —
    // ground-up normal, ground splat, ground UVs. Planar UVs on a vertical wall
    // is the one place in this terrain where the projection genuinely runs
    // downhill, and it read as smeared dirt.
    //
    // Two changes make it bedrock without a second material (a second material
    // would mean a second geometry group and would double the terrain's draw
    // calls): the skirt vertex gets a horizontal *outward* normal, which the
    // terrain material's triplanar blend — smoothstep(uTriStart=0.101,
    // uTriEnd=0.318, 1 - N.y) — reads as N.y = 0, i.e. a full world-space
    // projection with no planar UVs involved; and its splat is driven to pure
    // rock with the snow/wet/carve channels cleared and the curvature AO pinned
    // dark, so that projection resolves as shaded bedrock.
    //
    // Only the skirt's lower row is changed; the quad's upper row is still the
    // surface vertex itself, so a thin LOD crack exposes ground-coloured
    // geometry and only a wide one exposes rock. That is the correct ordering:
    // the crack-hiding job the skirt was built for is unaffected.
    const edge = (v, nx, nz) => {
      const o = w++;
      pos[o * 3] = pos[v * 3]; pos[o * 3 + 1] = pos[v * 3 + 1] - skirt; pos[o * 3 + 2] = pos[v * 3 + 2];
      nrm[o * 3] = nx; nrm[o * 3 + 1] = 0; nrm[o * 3 + 2] = nz;
      uv[o * 2] = uv[v * 2]; uv[o * 2 + 1] = uv[v * 2 + 1];
      if (col) {
        col[o * 3] = C_ROCK[0] * 0.45; col[o * 3 + 1] = C_ROCK[1] * 0.45; col[o * 3 + 2] = C_ROCK[2] * 0.45;
      }
      splat[o * 4] = 255; splat[o * 4 + 1] = 0; splat[o * 4 + 2] = 0; splat[o * 4 + 3] = 0;
      extra[o * 4] = 0; extra[o * 4 + 1] = 0; extra[o * 4 + 2] = 0; extra[o * 4 + 3] = 64;
      surf[o] = Surface.ROCK;
    };
    for (let i = 0; i < G; i++) edge(i, 0, -1);                      // -Z
    for (let i = 0; i < G; i++) edge((G - 1) * G + i, 0, 1);         // +Z
    for (let j = 0; j < G; j++) edge(j * G, -1, 0);                  // -X
    for (let j = 0; j < G; j++) edge(j * G + G - 1, 1, 0);           // +X

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('normal', new THREE.BufferAttribute(nrm, 3));
    geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    if (col) geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    geo.setAttribute('aSplat', new THREE.BufferAttribute(splat, 4, true));
    geo.setAttribute('aTerrainExtra', new THREE.BufferAttribute(extra, 4, true));
    geo.setAttribute('aSurface', new THREE.BufferAttribute(surf, 1, false));

    const strides = ((G - 1) % 4 === 0) ? [1, 2, 4] : [1, 2];
    const idxSets = strides.map((s) => buildIndex(G, s));
    geo.setIndex(idxSets[0]);

    const cx = x0 + size * 0.5, cz = z0 + size * 0.5;
    const cy = (minY + maxY) * 0.5;
    const halfY = (maxY - minY) * 0.5 + skirt;
    // 1e-3 of padding: exact-fit spheres lose corner vertices to float rounding
    // during frustum culling, which pops a chunk out at the screen edge.
    const rad = Math.sqrt(size * size * 0.5 + halfY * halfY) * 1.001 + 0.01;
    geo.boundingSphere = new THREE.Sphere(new THREE.Vector3(cx, cy, cz), rad);
    geo.boundingBox = new THREE.Box3(
      new THREE.Vector3(x0, minY - skirt, z0),
      new THREE.Vector3(x0 + size, maxY, z0 + size),
    );

    // ---- one material per LOD depth --------------------------------------
    // `materialForDepth` has existed in terrainMaterial.js since r2 behind an
    // ACTION REQUIRED contract note and had never been called, so every chunk on
    // the mountain — including the 8 m/vertex ones 1.5 km out — ran the full
    // near shader (tile-break, triplanar, micro-detail, 3-4 splat layers).
    // Feature-detected, because terrain must still render against a stub
    // material (CONTRACT §10).
    //
    // It is also the hook Lane A needs: the variant is chosen by depth, and
    // depth now *is* a distance band with a known vertex spacing, so the cavity
    // guard can be baked per variant as a screen-space triangle-size fade
    // instead of the view-distance fade that produced the faceting.
    const mud = (terrainMat && terrainMat.userData) || null;
    const chunkMat = (mud && typeof mud.materialForDepth === 'function')
      ? (mud.materialForDepth(depth) || terrainMat)
      : terrainMat;

    const mesh = new THREE.Mesh(geo, chunkMat);
    mesh.name = `terrain-d${depth}-${Math.round(x0 / size)}-${Math.round(z0 / size)}`;
    mesh.matrixAutoUpdate = false;
    mesh.updateMatrix();
    mesh.visible = false;   // the quadtree cut in update() decides each frame
    // Terrain self-shadowing (ridge lines, cliff bands casting onto the face
    // below) is a big part of the alpine read, but casting was gated on
    // `settings.cascades > 1` — a feature that does not exist. sky.js implements
    // exactly ONE cascade fitted to `sky.shadowRange` and says so in its own
    // contract note, so `cascades: 3` at high meant ~100+ chunks and ~460 k
    // triangles, including depth-3 nodes 1.5 km out, were vertex-shaded into a
    // 150 m shadow volume and clipped. Casting is now decided per frame in
    // selectNode() against the slice sky.js has actually fitted this frame.
    const shadows = settings.shadows !== false;
    mesh.castShadow = false;
    mesh.receiveShadow = shadows;
    group.add(mesh);

    const node = { mesh, geo, idxSets, tier: 0, cx, cy, cz, size, depth, children: null, split: false };
    chunks.push(node);
    return node;
  }

  /**
   * True minimum of the corridor distance field over a node's whole footprint.
   *
   * This replaces a five-point sample (centre + corners) whose worst case on a
   * 96 m node overstated the distance by 48 m — enough to leave a chunk at
   * 2 m/vertex with the trail running along its edge. See the LOD_BANDS block.
   *
   * The field is a chamfer distance transform on an 8 m grid and is therefore
   * 1-Lipschitz, so `min over the grid points in the footprint, minus half a
   * cell diagonal` is a valid *lower* bound: the error can only ever
   * over-refine, never under-refine. Cost is bounded by the field's own
   * resolution — a 384 m root reads 49x49 cells, and the whole tree reads about
   * 5 x (WORLD/DF_CELL)^2 ~= 0.7 M values once, at commit.
   */
  function minCorridorDist(x0, z0, size) {
    if (!corridorDF) return DF_FAR;
    let i0 = Math.floor((x0 - minX) / DF_CELL);
    let i1 = Math.ceil((x0 + size - minX) / DF_CELL);
    let j0 = Math.floor((z0 - minZ) / DF_CELL);
    let j1 = Math.ceil((z0 + size - minZ) / DF_CELL);
    if (i0 < 0) i0 = 0;
    if (j0 < 0) j0 = 0;
    if (i1 > DFN - 1) i1 = DFN - 1;
    if (j1 > DFN - 1) j1 = DFN - 1;
    if (i1 < i0 || j1 < j0) return DF_FAR;
    let m = DF_FAR;
    for (let j = j0; j <= j1; j++) {
      const row = j * DFN;
      for (let i = i0; i <= i1; i++) {
        const v = corridorDF[row + i];
        if (v < m) m = v;
      }
    }
    m -= DF_CELL * 0.71;          // half a cell diagonal: the continuous minimum
    return m > 0 ? m : 0;         // can sit between four sampled grid points
  }

  /**
   * Quadtree refinement. The band a node falls in is decided by the closest the
   * corridor gets to ANY part of its footprint, and the bands themselves are
   * derived from screen-space vertex spacing (see LOD_BANDS at module scope).
   *
   * Resulting spacing ladder, and the camera distance at which each band starts:
   *   depth 7 — 24 m chunk, 0.5 m/vertex, corridor < 40 m  (camera >= 28 m)
   *   depth 6 — 48 m chunk, 1.0 m/vertex, corridor < 68 m  (camera >= 56 m)
   *   depth 5 — 96 m chunk, 2.0 m/vertex, corridor < 124 m (camera >= 112 m)
   *   depth 4 — 192 m chunk, 4.0 m/vertex, corridor < 237 m (camera >= 225 m)
   *   depth 3 — 384 m chunk, 8.0 m/vertex, everything beyond
   * so no visible triangle outside the near field exceeds ~32 px of edge, and
   * inside it the floor is MAX_DEPTH's 0.5 m — which is also the resolution of
   * the 0.35 m detail heightfield feeding it, so refining further would only
   * resample the same signal.
   */
  function desiredDepth(x0, z0, size) {
    const d = minCorridorDist(x0, z0, size);
    const L = lodScale > 0 ? lodScale : 1;
    for (let depth = MAX_DEPTH; depth > ROOT_DEPTH; depth--) {
      if (d < LOD_BANDS[depth] * L) return depth;
    }
    return ROOT_DEPTH;
  }

  /**
   * Builds geometry at EVERY level of the tree, not only the leaves. That is what
   * makes the runtime cut possible: a distant branch can be drawn by one 384 m
   * parent instead of its 256 great-grandchildren. The extra levels cost about a
   * third more vertices (1 + 1/4 + 1/16 + ...) and take the visible chunk count
   * from "every chunk on the mountain" to a bounded ring of ~80.
   */
  function buildTree(x0, z0, size, depth) {
    const node = buildChunk(x0, z0, size, depth);
    const want = Math.min(MAX_DEPTH, desiredDepth(x0, z0, size));
    if (depth < want) {
      const h = size * 0.5;
      node.children = [
        buildTree(x0, z0, h, depth + 1),
        buildTree(x0 + h, z0, h, depth + 1),
        buildTree(x0, z0 + h, h, depth + 1),
        buildTree(x0 + h, z0 + h, h, depth + 1),
      ];
    }
    return node;
  }

  // -------------------------------------------------------------------------
  // Far ring — the world beyond the playable square
  //
  // One static mesh, one draw call, ~14.7 k triangles, no LOD and no per-frame
  // work of any kind. It is not in `chunks`, so the quadtree cut never touches
  // it and it is never toggled.
  // -------------------------------------------------------------------------

  let farRingMesh = null;
  let farRingMat = null;
  let farRingFadePatched = true;

  /** Metres by which (x,z) lies outside the playable square. 0 inside it. */
  function outsideDist(x, z) {
    const ox = Math.abs(x) - HALF;
    const oz = Math.abs(z) - HALF;
    const a = ox > 0 ? ox : 0;
    const b = oz > 0 ? oz : 0;
    return Math.sqrt(a * a + b * b);
  }

  /**
   * Height of the continued heightfield outside the playable square.
   *
   * `baseHeight` keeps producing ridges out here for free — the noise is a
   * function of world x/z and does not know about WORLD — but `PROFILE` clamps
   * at both ends, so left alone the far field would be a plateau beyond the
   * summit and a dead-flat plain beyond the valley. Two distance-driven terms
   * fix that:
   *   - a much broader ridged multifractal (the *same* Musgrave `ridged()` the
   *     mountain is built from, an octave band down) that only bites where the
   *     profile is already high, so the horizon behind the summit is a range of
   *     distant peaks rather than a table;
   *   - a gentle rise on the low ground, so the valley floor broadens into
   *     distant flats. That rise is load-bearing: it guarantees the far field
   *     climbs back out of water.js's valley body, so the water's own offshore
   *     apron can terminate *underneath* dry land and never show an edge.
   *
   * `sampleHeight` clamps its lattice coordinates, so outside the square it
   * returns the edge value — an exact extrusion of the real, eroded, carved
   * heightfield. Easing out of that over FAR_BLEND metres is what makes the map
   * boundary have no step in it.
   */
  function farHeight(x, z) {
    const hEdge = sampleHeight(x, z);
    const dOut = outsideDist(x, z);
    if (dOut <= 0) return hEdge;

    const t = clamp01(dOut / FAR_SETTLE);
    const prof = PROFILE(clamp01((z - minZ) / WORLD));
    const highland = smoothstep(0.10, 0.45, prof);

    let h = baseHeight(x, z);
    // Four octaves, not six: the ring's outer vertex spacing is 200-310 m, so a
    // fifth octave at ~190 m would alias into the silhouette rather than shape it.
    const fr = ridged(nRidge2, x * 0.00033 + 71.0, z * 0.00033 - 23.0, 4, 2.07, 0.55);
    h += (fr - 0.38) * 340 * t * highland;
    h += 26 * t * (1 - highland);

    return lerp(hEdge, h, smoothstep(0, FAR_BLEND, dOut));
  }

  /**
   * Far-field albedo. Same rules as passF, evaluated analytically: rock on
   * steep ground, snow above the line, alpine turf below it, scree between —
   * plus a conifer tint under the treeline, because vegetation.js stops at the
   * map edge and without it the far flanks read as bare turf against a
   * forested foreground.
   */
  function farColor(x, z, h, st, out) {
    const tl = treelineAt(x, z);
    const sl = snowlineAt(x, z);

    const jit = nMat(x * 0.0021 + 5.0, z * 0.0021 - 3.0) * 0.5 + 0.5;

    let wRock = smoothstep(0.46, 0.98, st) * (0.75 + 0.5 * jit);
    let wScree = smoothstep(0.22, 0.60, st) * smoothstep(tl - 60, tl + 170, h) * 1.1;
    let wGrass = smoothstep(0.85, 0.24, st) * smoothstep(tl + 260, tl - 140, h) * 1.0;
    let wSoil = 0.22 + smoothstep(1.0, 0.30, st) * 0.35;
    const bare = clamp01(wRock * 0.9 + wScree * 0.5);
    wGrass *= 1 - bare * 0.92;
    wSoil *= 1 - bare * 0.80;

    const tot = wRock + wScree + wSoil + wGrass || 1;
    wRock /= tot; wScree /= tot; wSoil /= tot; wGrass /= tot;

    let r = C_ROCK[0] * wRock + C_SCREE[0] * wScree + C_SOIL[0] * wSoil + C_GRASS[0] * wGrass;
    let g = C_ROCK[1] * wRock + C_SCREE[1] * wScree + C_SOIL[1] * wSoil + C_GRASS[1] * wGrass;
    let b = C_ROCK[2] * wRock + C_SCREE[2] * wScree + C_SOIL[2] * wSoil + C_GRASS[2] * wGrass;

    // Forest mass under the treeline, thinning on anything too steep to hold it.
    const forest = smoothstep(tl + 30, tl - 150, h) * smoothstep(0.95, 0.40, st) * 0.78;
    if (forest > 0) {
      r = lerp(r, C_FOREST[0], forest); g = lerp(g, C_FOREST[1], forest); b = lerp(b, C_FOREST[2], forest);
    }

    const snow = clamp01(smoothstep(sl - 90, sl + 50, h) * (1 - smoothstep(0.62, 1.05, st)));
    if (snow > 0) {
      r = lerp(r, C_SNOW[0], snow); g = lerp(g, C_SNOW[1], snow); b = lerp(b, C_SNOW[2], snow);
    }

    // Macro variation at the only scale that survives out here.
    const m = 1 + nMacro(x * 0.00085 + 41.0, z * 0.00085 - 17.0) * 0.10;
    out[0] = r * m; out[1] = g * m; out[2] = b * m;
  }

  const _farCol = [0, 0, 0];

  function buildFarRing() {
    const NA = FAR_RING_SEGS;
    const NR = FAR_RING_RINGS;
    const rows = NR + 1;
    // + TWO extra rows for the inner skirt. The skirt's top row is a *duplicate*
    // of row 0 rather than row 0 itself: sharing the vertices would let
    // computeVertexNormals() average the vertical curtain into the surface and
    // paint a shaded ring right along the map boundary — exactly the seam this
    // whole ring exists to remove.
    const nv = NA * rows + NA * 2;
    const pos = new Float32Array(nv * 3);
    const col = new Float32Array(nv * 3);
    const fade = new Float32Array(nv);

    const cosT = new Float64Array(NA);
    const sinT = new Float64Array(NA);
    const rIn = new Float64Array(NA);
    for (let a = 0; a < NA; a++) {
      const th = (a / NA) * Math.PI * 2;
      const c = Math.cos(th), s = Math.sin(th);
      cosT[a] = c; sinT[a] = s;
      // Radius at which this ray leaves the playable square, so row 0 sits
      // exactly on the map boundary.
      const m = Math.max(Math.abs(c), Math.abs(s));
      rIn[a] = HALF / (m > 1e-9 ? m : 1e-9);
    }

    for (let row = 0; row < rows; row++) {
      // s^1.8 keeps the rings dense where the seam is and coarse out at the
      // horizon: the first step out from the boundary is ~20 m, the last ~300 m.
      const ease = Math.pow(row / NR, 1.8);
      for (let a = 0; a < NA; a++) {
        const r = rIn[a] + (FAR_RING_R - rIn[a]) * ease;
        const x = cosT[a] * r;
        const z = sinT[a] * r;
        const v = row * NA + a;

        const h = farHeight(x, z);
        pos[v * 3] = x; pos[v * 3 + 1] = h; pos[v * 3 + 2] = z;

        // Slope from an 8 m forward difference. All it drives is the albedo mix,
        // and 8 m is the right scale to ask "is this a cliff" at — the ring's own
        // vertex spacing (20-300 m) would answer a different question entirely.
        const D = 8;
        const gx = (farHeight(x + D, z) - h) / D;
        const gz = (farHeight(x, z + D) - h) / D;
        farColor(x, z, h, Math.sqrt(gx * gx + gz * gz), _farCol);
        col[v * 3] = _farCol[0]; col[v * 3 + 1] = _farCol[1]; col[v * 3 + 2] = _farCol[2];

        // Carries the vertex's radius, not a fade value: the dissolve is a
        // smoothstep in the fragment shader, so it cannot band on a ring
        // boundary the way a per-vertex fade over 300 m spacing would.
        fade[v] = r;
      }
    }

    // Skirt row: the inner ring dropped by FAR_SKIRT. Row 0 samples the boundary
    // every ~38 m while the edge chunks carry a vertex every 8 m, so the two
    // polylines can disagree by a metre or two on a rough edge; the curtain
    // means that can never open a hole to the sky.
    const ST = rows * NA;          // skirt top row (duplicate of row 0)
    const SB = ST + NA;            // skirt bottom row
    for (let a = 0; a < NA; a++) {
      const v = a;
      const t = ST + a, o = SB + a;
      pos[t * 3] = pos[v * 3]; pos[t * 3 + 1] = pos[v * 3 + 1]; pos[t * 3 + 2] = pos[v * 3 + 2];
      pos[o * 3] = pos[v * 3]; pos[o * 3 + 1] = pos[v * 3 + 1] - FAR_SKIRT; pos[o * 3 + 2] = pos[v * 3 + 2];
      for (let c = 0; c < 3; c++) {
        col[t * 3 + c] = C_ROCK[c] * 0.62;
        col[o * 3 + c] = C_ROCK[c] * 0.45;
      }
      fade[t] = fade[v]; fade[o] = fade[v];
    }

    const tris = NA * NR * 2 + NA * 2;
    const idx = new Uint16Array(tris * 3);
    let p = 0;
    for (let row = 0; row < NR; row++) {
      const b0 = row * NA, b1 = (row + 1) * NA;
      for (let a = 0; a < NA; a++) {
        const a2 = (a + 1) % NA;
        const A = b0 + a, B = b0 + a2, C = b1 + a, D = b1 + a2;
        // cross( B-A, C-A ) = cross( +theta, +radial ) = +Y, so this winding
        // gives an upward-facing surface.
        idx[p++] = A; idx[p++] = B; idx[p++] = C;
        idx[p++] = B; idx[p++] = D; idx[p++] = C;
      }
    }
    for (let a = 0; a < NA; a++) {
      const a2 = (a + 1) % NA;
      const A = ST + a, B = ST + a2, C = SB + a, D = SB + a2;
      idx[p++] = A; idx[p++] = B; idx[p++] = C;
      idx[p++] = B; idx[p++] = D; idx[p++] = C;
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    geo.setAttribute('aFarRadius', new THREE.BufferAttribute(fade, 1));
    geo.setIndex(new THREE.BufferAttribute(idx, 1));
    geo.computeVertexNormals();

    let lo = Infinity, hi = -Infinity;
    for (let v = 0; v < nv; v++) {
      const y = pos[v * 3 + 1];
      if (y < lo) lo = y;
      if (y > hi) hi = y;
    }
    const cy = (lo + hi) * 0.5;
    geo.boundingSphere = new THREE.Sphere(
      new THREE.Vector3(0, cy, 0),
      Math.sqrt(FAR_RING_R * FAR_RING_R + (hi - cy) * (hi - cy)) * 1.01);

    farRingMat = new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 0.97,
      metalness: 0.0,
      dithering: true,
      side: THREE.DoubleSide,
      fog: true,
    });
    farRingMat.name = 'terrain:farRing';
    // Force the outermost FAR_FADE_BAND metres to the fog colour on top of
    // whatever aerial perspective sky.js is running, so the ring's own outer
    // edge dissolves into the horizon instead of terminating in a silhouette.
    // sky.js owns the fog chunks (it replaces THREE.ShaderChunk.fog_fragment
    // wholesale), so both the patched and the stock composite line are handled
    // and a miss degrades to "the ring just fogs normally" rather than to a
    // shader compile error.
    const F0 = (FAR_RING_R - FAR_FADE_BAND).toFixed(1);
    const F1 = (FAR_RING_R - 30).toFixed(1);
    const FADE = `smoothstep( ${F0}, ${F1}, vFarRadius )`;
    farRingMat.onBeforeCompile = (shader) => {
      shader.vertexShader = 'attribute float aFarRadius;\nvarying float vFarRadius;\n'
        + shader.vertexShader.replace('void main() {', 'void main() {\n\tvFarRadius = aFarRadius;');
      const HFOG_LINE = 'gl_FragColor.rgb = mix( gl_FragColor.rgb, hfogFinal, clamp( fogFactor, 0.0, 1.0 ) );';
      const STOCK_LINE = 'gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );';
      let frag = shader.fragmentShader;
      if (frag.indexOf(HFOG_LINE) >= 0) {
        frag = frag.replace(HFOG_LINE,
          `gl_FragColor.rgb = mix( gl_FragColor.rgb, hfogFinal, clamp( max( fogFactor, ${FADE} ), 0.0, 1.0 ) );`);
      } else if (frag.indexOf(STOCK_LINE) >= 0) {
        frag = frag.replace(STOCK_LINE,
          `gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, clamp( max( fogFactor, ${FADE} ), 0.0, 1.0 ) );`);
      } else {
        farRingFadePatched = false;
      }
      shader.fragmentShader = 'varying float vFarRadius;\n' + frag;
    };

    farRingMesh = new THREE.Mesh(geo, farRingMat);
    farRingMesh.name = 'terrain-far-ring';
    farRingMesh.matrixAutoUpdate = false;
    farRingMesh.updateMatrix();
    farRingMesh.castShadow = false;      // far outside every shadow cascade
    farRingMesh.receiveShadow = false;
    // renderOrder is deliberately left at 0: three sorts opaque draws
    // front-to-back, so the near chunks lay depth down first and the ring's
    // large, mostly-occluded footprint is killed by early-Z.
    group.add(farRingMesh);

    timings.farRingTris = tris;
    timings.farRingVerts = nv;
  }

  function commit() {
    if (!baseBuilt) buildBase();
    if (chunks.length) return;
    const t0 = Date.now();

    const matOpts = {
      seed: ctx.seed,
      quality: ctx.quality,
      bounds,
      // The painted SurfaceId field: this is what makes the carved trail read as a
      // trail rather than as generic dirt-coloured ground.
      materialIds: materials,
      materialRes: RES,
      surfaceAttribute: true,
      snowLine: SNOWLINE_Y,
      snowBlend: 46,
      snowAmount: 0.85,
      wetness: 0.13,
      // Descriptive extras (ignored by materials that do not want them).
      worldSize: WORLD,
      cellSize: CELL,
      Surface,
      surfaceSplat: SURFACE_SPLAT,
      treeline: TREELINE_Y,
      valleyY: BASE_Y,
      uvScale: 32,
      // MEASURED metres between adjacent chunk vertices, indexed by LOD depth.
      // Lane A's cavity guard needs this to fade the per-triangle cavity/crest
      // term on screen-space triangle size. Read it — do NOT re-derive it from
      // a comment: the r3 work order's table (depth 7 = 0.375, 6 = 0.75) came
      // from a stale comment in this file and is wrong by 1.33x, because
      // G_FINE == G_COARSE == 49 so the ladder is a clean 2x at 8/4/2/1/0.5 m.
      // Note also that selectNode()'s second LOD stage swaps to a 2x or 4x
      // index stride past `size*4.5` and `size*11`, so the effective spacing of
      // a DRAWN chunk can be 2x or 4x this — at which range terrMPP is large
      // enough that the guard has already closed.
      vertexSpacingByDepth: (() => {
        const t = {};
        for (let d = ROOT_DEPTH; d <= MAX_DEPTH; d++) t[d] = vertexSpacing(d);
        return t;
      })(),
      lodBandsByDepth: (() => {
        const t = {};
        for (let d = ROOT_DEPTH + 1; d <= MAX_DEPTH; d++) t[d] = LOD_BANDS[d] * lodScale;
        return t;
      })(),
      vertexColorMeaning: 'linear-albedo',
      attributes: {
        color: 'vec3 linear-sRGB base albedo (present only in the fallback path)',
        aSplat: 'vec4 normalised weights [rock, scree, soil, grass]',
        aTerrainExtra: 'vec4 normalised [snow, wetness, trailCarve, curvatureAO]',
        aSurface: 'float SurfaceId 0..7',
        uv: 'world XZ / 32 m',
      },
    };
    try {
      terrainMat = createTerrainMaterial(ctx, matOpts);
    } catch (e) {
      console.error('[terrain] createTerrainMaterial failed, using fallback', e);
      terrainMat = null;
    }
    if (!terrainMat) {
      terrainMat = new THREE.MeshStandardMaterial({ roughness: 0.95, metalness: 0 });
    }
    // If the material has not declared itself terrain-aware it is a stub, so fall
    // back to shading from our per-vertex albedo — the mountain must read correctly
    // on its own (CONTRACT §10). A real terrain material owns that decision itself
    // and must NOT have vertexColors forced on underneath it.
    const ud = terrainMat.userData || {};
    if (!(ud.isTerrainMaterial || ud.descentTerrainMaterial)) {
      useVertexColorAlbedo = true;
      terrainMat.vertexColors = true;
      if (terrainMat.color) terrainMat.color.setRGB(1, 1, 1);
      if (terrainMat.roughness !== undefined) terrainMat.roughness = 0.94;
      if (terrainMat.metalness !== undefined) terrainMat.metalness = 0.0;
      terrainMat.dithering = true;
      terrainMat.needsUpdate = true;
    }

    if (!corridorDF) corridorDF = new Float32Array(DFN * DFN).fill(DF_FAR);
    // depth d ⇒ node size WORLD / 2^d. The tree starts at ROOT_DEPTH (384 m nodes,
    // 8x8 of them) because anything coarser is never worth drawing.
    const rootSize = WORLD / (1 << ROOT_DEPTH);
    const rootN = 1 << ROOT_DEPTH;
    for (let j = 0; j < rootN; j++) {
      for (let i = 0; i < rootN; i++) {
        roots.push(buildTree(minX + i * rootSize, minZ + j * rootSize, rootSize, ROOT_DEPTH));
      }
    }

    // The world does not stop at the map boundary any more.
    buildFarRing();

    group.updateMatrix();
    if (ctx.scene) ctx.scene.add(group);
    // Pick an initial cut so the terrain is populated before the first update().
    update(0, ctx);

    let verts = 0;
    for (const c of chunks) verts += c.geo.attributes.position.count;
    timings.commit = Date.now() - t0;
    timings.chunks = chunks.length;
    timings.vertices = verts;
    // Measured, not asserted from a comment (see the splitK derivation below).
    timings.visibleChunks = lodStats.visible;
    timings.splitK = _splitK;
    // The derived LOD ladder, so a reviewer can check the numbers against the
    // frame instead of against the comment above them.
    timings.lodBands = Array.from(LOD_BANDS).map((v, d) => (d > ROOT_DEPTH
      ? `d${d}<${v.toFixed(0)}m @${vertexSpacing(d).toFixed(3)}m/vtx` : null)).filter(Boolean);
    timings.farMaterial = !!(terrainMat && terrainMat.userData
      && typeof terrainMat.userData.materialForDepth === 'function');
    if (ctx.debug) ctx.debug.log('terrain.commit', timings);
  }

  // -------------------------------------------------------------------------
  // update — per-frame LOD selection
  //
  // Geometry is never rebuilt at runtime. Two stages, both pointer work:
  //   1. the quadtree cut — walk down from the 8x8 roots and stop at the first node
  //      further away than `splitK` of its own size, drawing that node instead of
  //      its whole subtree. This is what keeps the visible chunk count bounded
  //      instead of "every chunk on the mountain" (~500+).
  //   2. within the chosen node, swap to a coarser prebuilt index buffer over the
  //      same vertices when even its own tessellation is past what the screen
  //      resolves. Index buffers are shared by every chunk of the same grid size,
  //      so this is three GL buffers for the whole world.
  // Both stages carry a dead band so a node on a threshold cannot flip each frame,
  // and the skirts built in buildChunk() cover the seams either stage opens up.
  //
  // ---- the split constant, derived rather than guessed ---------------------
  // A node of size S is DRAWN when its parent split (dist < 2*S*K) and it did
  // not (dist >= S*K), so each depth occupies the annulus [S*K, 2*S*K) and
  // contributes about pi*((2SK)^2 - (SK)^2)/S^2 = 3*pi*K^2 chunks, independent
  // of S. With ROOT_DEPTH = 3 there are also 64 root nodes, of which only those
  // inside 384*K split. So:
  //
  //     visible ~= 64 - pi*K^2  +  3*pi*K^2 * (number of interior depths)
  //
  // K = 2.05 gives 51 + 3*39.6 + the depth-7 band ~= 190-220. The r2 review
  // measured 217 in the running build against a comment in this file claiming
  // "~60-100", i.e. the tuning had never been checked. The work order asks for
  // 2.6; that is the wrong direction — the test is `dist < size * K`, so a
  // LARGER K splits MORE and 2.6 would take the count to roughly 300.
  // K = 1.15 is the value that delivers the work order's intent:
  //     51 + 3*(3*pi*1.32) + ~12  ~=  100-120 visible chunks,
  // roughly halving the terrain's share of the 400-draw-call budget.
  //
  // It also fixes the thing the work order actually cares about — that the
  // depth bands mean what terrainMaterial.js's near/far variant split assumes.
  // At K = 1.15 the bands are: depth 7 inside 55 m, depth 6 in 55-110 m,
  // depth 5 in 110-221 m, depth 4 in 221-442 m, depth 3 beyond. At K = 2.05
  // depth 6 reached 197 m, so "NEAR = depth 6-7 = inside ~50 m" was false by
  // a factor of four.
  //
  // Cost of the change: at the far edge of a band a G=49 patch is
  // 1080/(2*tan(31deg)) / (48*K) = 20.8/K pixels per vertex edge, so K = 1.15
  // is 18 px/edge against 10 px/edge before. Terrain silhouettes do not need
  // 10 px vertices; the 0.35 m detail heightfield and the material's own normal
  // detail carry everything below that. The measured geometry cost either way
  // is ~0.3 ms, so this is a draw-call fix, not a fill-rate one.
  //
  // `lodStats` is exported so the debug HUD can assert the number rather than
  // trusting this comment ever again.
  //
  // ---- interaction with the r3 LOD-band fix --------------------------------
  // The derivation above assumes the tree HAS a node at every depth the cut
  // wants. Before the LOD_BANDS fix it usually did not: off the corridor the
  // tree bottomed out at depth 5, so the cut stopped early and drew one big
  // chunk where the arithmetic predicts four small ones. The measured 217 was
  // an over-count from a different cause (K = 2.05), and the count is now
  // expected to land close to the derived figure: 64 roots less those that
  // split, plus 3*pi*K^2 = 12.5 per interior depth, plus the depth-7 leaf disc
  // (pi*(2*24*1.15)^2 / 24^2 = 17), i.e. roughly 110-120. Still inside the
  // 40-170 assertion band below.
  // -------------------------------------------------------------------------

  // Camera position for the current cut, kept as scalars so the walk allocates nothing.
  let _cpx = 0, _cpy = 0, _cpz = 0, _splitK = 1.15, _lodNow = 1;
  // Shadow slice for the current frame — see the castShadow note in buildChunk().
  let _castShadows = settings.shadows !== false;
  let _shadowReach = 150;

  // Preallocated: update() must not allocate.
  const _lodByDepth = new Int32Array(MAX_DEPTH + 1);
  const lodStats = { visible: 0, splitK: _splitK, byDepth: _lodByDepth, warned: false };
  let _lodVisible = 0;
  let _lodFrames = 0;

  /**
   * Distance from the camera to the node's footprint (not its centre) — using the
   * centre would keep a 384 m node subdivided long after you have flown past it,
   * and would pop the node you are standing on.
   */
  function nodeDistance(n) {
    const h = n.size * 0.5;
    let dx = Math.abs(_cpx - n.cx) - h; if (dx < 0) dx = 0;
    let dz = Math.abs(_cpz - n.cz) - h; if (dz < 0) dz = 0;
    const dy = _cpy - n.cy;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  function selectNode(n) {
    const dist = nodeDistance(n);
    // Hysteresis on the split test: 8% of dead band, so a node sitting exactly on
    // the boundary cannot toggle its whole subtree on and off every frame.
    const split = n.size * _splitK * _lodNow * (n.split ? 1.08 : 1.0);
    if (n.children && dist < split) {
      n.split = true;
      for (let i = 0; i < 4; i++) selectNode(n.children[i]);
      return;
    }
    n.split = false;
    n.mesh.visible = true;
    // A caster outside the fitted ortho volume is clipped, so submitting it costs
    // a draw call and a full vertex pass for nothing. `+ n.size` keeps a chunk
    // that only straddles the boundary in the pass.
    n.mesh.castShadow = _castShadows && dist < _shadowReach + n.size;
    _lodVisible++;
    _lodByDepth[n.depth]++;
    // Second LOD stage inside the chosen node: drop to a coarser prebuilt index
    // when even this node's own tessellation is beyond what the screen resolves.
    const nsets = n.idxSets.length;
    let tier = 0;
    // At 4.5 node-sizes away a G=49 patch is ~5 px per vertex edge at 1080p/55deg,
    // so halving the tessellation there is still well above the resolve limit.
    if (dist > n.size * 11 * _lodNow) tier = 2;
    else if (dist > n.size * 4.5 * _lodNow) tier = 1;
    if (tier >= nsets) tier = nsets - 1;
    if (tier !== n.tier) {
      n.tier = tier;
      n.geo.setIndex(n.idxSets[tier]);
    }
  }

  function update(dt, c) {
    const cam = (c && c.camera) || ctx.camera;
    if (!cam || roots.length === 0) return;
    _cpx = cam.position.x; _cpy = cam.position.y; _cpz = cam.position.z;
    _lodNow = (c && c.settings && typeof c.settings.terrainLOD === 'number')
      ? c.settings.terrainLOD : lodScale;

    // sky.js owns the shadow rig and refits it every frame; `shadowRange` is the
    // range it is ACTUALLY fitted to (the adaptive aerial branch grows it up to
    // 8x), so reading it rather than a local copy is what stops the two numbers
    // drifting apart. terrain updates before sky in the wave order, so this is
    // last frame's fit — the value is quantised on a 1.25x ladder and moves
    // slowly, and the `+ n.size` margin absorbs a step.
    const cs = (c && c.settings) || settings;
    _castShadows = cs.shadows !== false;
    const sky = (c && c.sky) || ctx.sky;
    const reach = sky && (sky.shadowRange || sky.shadowDistance);
    _shadowReach = (typeof reach === 'number' && reach > 0) ? reach : 150;

    for (let i = 0; i < chunks.length; i++) {
      const m = chunks[i].mesh;
      m.visible = false;
      m.castShadow = false;
    }
    _lodVisible = 0;
    _lodByDepth.fill(0);
    for (let i = 0; i < roots.length; i++) selectNode(roots[i]);
    lodStats.visible = _lodVisible;
    // One-shot: shout if the derivation above stops matching the running build.
    // (Deferred past the first frames so the initial cut from commit() — taken
    // before the camera has been placed — cannot trip it.)
    _lodFrames++;
    if (!lodStats.warned && _lodFrames > 90 && (_lodVisible < 40 || _lodVisible > 170)) {
      lodStats.warned = true;
      console.warn('[terrain] visible chunk count', _lodVisible,
        'is outside the derived 40-170 band for splitK', _splitK, 'byDepth', Array.from(_lodByDepth));
    }
  }

  function dispose() {
    for (let i = 0; i < chunks.length; i++) chunks[i].geo.dispose();
    chunks.length = 0;
    roots.length = 0;
    indexCache.clear();
    // The far LOD variant is a second material sharing one texture bundle; the
    // bundle only tears down when the LAST material goes, so dispose it first.
    const fud = (terrainMat && terrainMat.userData) || null;
    if (fud && fud.farMaterial && fud.farMaterial.dispose) fud.farMaterial.dispose();
    if (terrainMat && terrainMat.dispose) terrainMat.dispose();
    if (farRingMesh) {
      group.remove(farRingMesh);
      farRingMesh.geometry.dispose();
      farRingMesh = null;
    }
    if (farRingMat) { farRingMat.dispose(); farRingMat = null; }
    if (ctx.scene) ctx.scene.remove(group);
  }

  return {
    Surface,

    // Contract §3
    buildBase,
    applyCarve,
    commit,
    sampleHeight,
    sampleNormal,
    sampleMaterial,
    sampleSlope,
    inBounds,
    bounds,
    size: { width: WORLD, depth: WORLD },
    resolution: RES,
    update,
    dispose,

    // Raw fields — physics/debug may read these, but must not resize them.
    heights,
    materials,
    ao: aoU8,
    splat: splatU8,
    extra: extraU8,

    // Scene objects
    group,
    get mesh() { return group; },
    chunks,
    get material() { return terrainMat; },
    get usingVertexColorAlbedo() { return useVertexColorAlbedo; },

    // Extras (see the CONTRACT-NOTE at the top of this file)
    sampleWetness,
    sampleSnow,
    sampleCarve,
    corridorDistance: corridorDist,
    treelineAt,
    snowlineAt,
    detailStep: DSTEP,
    valleyY: BASE_Y,
    get creekLevel() { return creekLevel; },
    get timings() { return timings; },

    // LOD instrumentation — the debug HUD asserts against this rather than
    // against a comment. `byDepth` is a live preallocated Int32Array.
    lodStats,

    // The continued world beyond the playable square (one static draw call).
    get farRing() { return farRingMesh; },
    farRingRadius: FAR_RING_R,
    get farRingFadePatched() { return farRingFadePatched; },
  };
}
