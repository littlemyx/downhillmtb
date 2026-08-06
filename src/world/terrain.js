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
// CONTRACT-NOTE (terrain → trail, NEW API FOR marchRoute): four directional queries, added
//   for the route-finder's heading search. All are allocation-free and all are valid before
//   applyCarve() — which is when the march runs — as well as after.
//
//     terrain.sampleGrade(x, z, dirX, dirZ)             -> dY per metre ALONG the heading.
//                                                          SIGNED: descending is NEGATIVE.
//     terrain.sampleCrossSlope(x, z, dirX, dirZ)        -> dY per metre to the rider's RIGHT
//                                                          of that heading (+ = right high,
//                                                          the same sign as CarveStamp.bank).
//     terrain.sampleCurvatureAlong(x, z, dirX, dirZ, half)
//                                                       -> d2Y/ds2 along the heading, by a
//                                                          central difference at HALF-STENCIL
//                                                          `half` metres. Convex (a crest) is
//                                                          NEGATIVE. `half` is mandatory: a
//                                                          curvature quoted without its
//                                                          stencil is not a measurement, and
//                                                          five rounds were lost to that.
//     terrain.probeCorridor(x, z, dirX, dirZ, half, out)-> all of the above in ONE pass that
//                                                          shares its samples, plus the
//                                                          corridor probe. See probeCorridor()
//                                                          for the full field list; the short
//                                                          version is `planarL/planarR`
//                                                          (metres out to which the hillside
//                                                          stays a plane — the honest corridor
//                                                          width), `fallL/fallR` (signed
//                                                          departure at 5 m, - = exposure),
//                                                          `cutL/cutR` (the cut a 2 m half
//                                                          bench would have to make) and
//                                                          `benchable` (0..1, one scalar for a
//                                                          cost function).
//
//   `dirX/dirZ` need not be unit — they are normalised inside, because a heading sweep
//   usually builds them from a sin/cos pair. `out` may be omitted, in which case a
//   module-scope object is reused; keep a result across another call and you must pass your
//   own. `probeReach`, `probeBand`, `probeFallAt` are exported so a caller can report the
//   geometry its numbers came from.
//
//   COST, measured on this machine, so the march can budget rather than guess:
//   sampleGrade / sampleCrossSlope are ~90 ns (one height sample; the gradient is analytic
//   and comes free with it). sampleCurvatureAlong is ~3x that (a second derivative genuinely
//   needs a stencil). probeCorridor is ~2.5 us, because its lateral walk is up to 40 height
//   samples. SHORTLIST ON THE CHEAP ONES AND PROBE THE SURVIVORS: 64 headings x 6800 stations
//   is 39 ms of sampleGrade and 1.1 s of probeCorridor.
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
//
// CONTRACT-NOTE (terrain → trail, THE DESIGN SURFACE): CarveStamp describes a *plane*, not a
//   plateau — `targetHeight` is the tread height AT THE STAMP CENTRE and `bank` its cross-slope.
//   Terrain now also derives the stamp's ALONG-track gradient (from its neighbours' target
//   heights) and evaluates each stamp as a plane at the query point. Two consequences the trail
//   engineer should know about:
//     * `bank` is committed at 1.0x on every kind. It used to be 0.9x on tread/lip/landing/drop
//       and, because the blend strength collapsed outside half a tread radius, ~0.22x on berms.
//       The lean identity atan(v^2/(g r)) <= bank + lean was therefore satisfied in the emitted
//       design and violated in the built world.
//     * `falloff` no longer shapes the tread core (the bench must be committed at full strength
//       or the committed cross-section cannot equal the emitted one). It still shapes the ease
//       outside the tread edge and the paint/carve mask.
//   `terrain.sampleDesign(x, z, out)` returns the design surface at a point so this can be
//   measured rather than asserted; see the "committed vs design" note above applyCarve().
//
// CONTRACT-NOTE (terrain → trail, THE BAND LIMIT): the committed tread is band-limited
//   before it is written to the heightfield — see commitDetail(). Terrain will not
//   reproduce anything a 0.37 m wheel cannot roll over ALONG the track, and a stamp that
//   asks for it is not honoured. Two emissions currently fall in that class and arrive
//   softened rather than as authored: the rut stamps (r 0.34 m, 0.012-0.252 m below the
//   tread, alternate stations) and the rock-garden boulder stamps (r 0.22-0.56 m, 0.17-0.42 m
//   proud). WHY: at the p50 design speed of 12.9 m/s a wheel leaves the ground above
//   kappa = g/v^2 = 0.059 /m, which a 1 m-wavelength ripple reaches at an amplitude of
//   1.5 mm. Committing the emitted cloud exactly threw the wheel off the ground at 37% of
//   stations against 4% for the trail's own centreline. If a feature is meant to be ridden
//   over rather than filtered, trail.js must emit it at a scale a wheel can follow — the
//   place to do that is the stamp radius and station spacing, not a deeper targetHeight on
//   a small stamp.
//
//   THE FILTER IS ANISOTROPIC AND THIS IS THE NUMBER THE TRAIL ENGINEER NEEDS. It is a
//   Gaussian in the TRACK FRAME: sigma 0.35 m ACROSS the track everywhere, and 0.555 m
//   ALONG it where the design authors curvature, widening to 1.10 m where the design is
//   flat (see the ADAPTIVE block in commitDetail()). The quantity to calibrate a banking
//   solve against is the committed cross-slope as a fraction of tan(bank), measured on the
//   +-0.36 x width secant. THE ADAPTIVE WIDENING IS ALONG-TRACK ONLY and therefore does not
//   change that answer materially: 0.917/0.888/0.921 -> 0.913/0.884/0.921 across the three
//   seeds, and on the steepest berms it does not move at all (0.928 -> 0.928).
//
//   THE ANISOTROPY QUESTION, SETTLED WITH NUMBERS. A/B in one process against one cached
//   route and one cached base heightfield per seed, pre-carve heights hash identical across
//   every variant; committed cross-slope / tan(bank), p50, seeds 20260726 / 777 / 12345:
//
//                                  all      by |tan bank| 0.05-0.15 / 0.15-0.35 / 0.35-0.70 / >0.70
//     SYMMETRIC (iso, sigma 0.555) 0.876    0.886 / 0.871 / 0.871 / 0.881
//                                  0.840    0.809 / 0.871 / 0.865 / -
//                                  0.875    0.879 / 0.874 / 0.865 / -
//     ANISOTROPIC (0.555 / 0.35)   0.917    0.907 / 0.919 / 0.932 / 0.928
//                                  0.888    0.849 / 0.920 / 0.924 / -
//                                  0.921    0.908 / 0.944 / 0.927 / -
//     SHIPPED (adaptive + aniso)   0.913    0.899 / 0.917 / 0.928 / 0.928
//                                  0.884    0.847 / 0.917 / 0.923 / -
//                                  0.921    0.907 / 0.940 / 0.923 / -
//
//   The anisotropic kernel is worth 4.1-4.8 points of bank overall, and — the part that
//   decides it — the symmetric kernel is flat or FALLING across the bank bands (0.886 ->
//   0.871, 0.879 -> 0.865) while the anisotropic one RISES with them (0.907 -> 0.932,
//   0.908 -> 0.944). Bank matters most where the berm is steepest, which is exactly where
//   the symmetric kernel is relatively worst. Its price is 0.0022-0.0029 /m of flat-ground
//   residual p90; the whole-course wheel test moves by +0.01 to +0.02 percentage points and
//   the worst 25 m bin by -0.8 to 0.0, i.e. the wheel test does not separate them at all,
//   and shaped-feature retention agrees to 0.005. SHIP THE ANISOTROPIC ONE — and with the
//   adaptive along-track sigma now taking 37-41% off the residual p90, its residual cost is
//   repaid many times over. (Reported on the wheel test and the cross-slope pair, never on
//   band r.m.s. — a band r.m.s. cannot see a curvature, which is the round-6 lesson.)
//
//   THE CEILING IS NOT 1.0. The EMITTED design surface itself only realises
//   0.962 / 0.949 / 0.953 of its own declared bank on the same secant (0.956 / 0.893 /
//   0.934 on the steepest berms), because five stamps per station reconstructed by a
//   scattered-data interpolant do not reproduce a section exactly. So of the ~13% that
//   was missing, ~4-6% is the stamp cloud and terrain cannot reach it from here. If the
//   solve needs more bank than that, author more bank.
//
//   The consequence for `sampleDesign`: |sampleHeight - sampleDesign| at the emitted stamps
//   is p50 0.028 / p99 0.178-0.262 / max 0.62-1.04 m, where the symmetric kernel gave
//   p99 0.258-0.312 and max 0.85-1.13. That residual is the band limit doing its job, not
//   the accumulator failing; it is measured at the stamps, which are exactly the points a
//   low-pass is expected to miss.
//
// CONTRACT-NOTE (terrain → trail, WHAT THE COMMITTED SURFACE COSTS IN TOP SPEED, and what
//   it does NOT): the residual convex curvature the commit leaves on ground the design asked
//   to be FLAT — |kappa_design| < 0.012 /m at the 0.37 m stencil, no feature authored — is
//   now, per seed 20260726 / 777 / 12345, against the r7 kernel on the same route:
//         p90   0.0227 -> 0.0134     0.0217 -> 0.0137     0.0242 -> 0.0151   /m
//         p95   0.0331 -> 0.0170     0.0352 -> 0.0172     0.0364 -> 0.0193
//         p99   0.0557 -> 0.0315     0.0594 -> 0.0286     0.0571 -> 0.0326
//         max   0.0725 -> 0.0660     0.1234 -> 0.1149     0.0918 -> 0.0630
//   i.e. 37-41% off the p90, 47-51% off the p95, 43-52% off the p99. The launch-free speed
//   at p95, sqrt(g / kappa_p95), rises from 16.4-17.2 m/s to 22.6-24.1 m/s.
//   THE PRICE, stated where it can be found: the shaped-feature peak-to-peak mean falls
//   0.932/0.927/0.938 -> 0.924/0.916/0.933 and its MINIMUM 0.808/0.795/0.812 ->
//   0.808/0.761/0.811 — one feature on seed 777 loses an extra 3.4% of its lip; the 2-6 m
//   band ratio falls 0.728/0.707/0.733 -> 0.716/0.689/0.712; the committed cross-slope p50
//   falls 0.917/0.888/0.921 -> 0.913/0.884/0.921; and the carve costs 0-3% more.
//   Stencil, reference and sampling for every one of those figures:
//   plan path = centripetal Catmull-Rom through the emitted station plan points,
//   arc-length resampled at 0.05 m; wheel-centre locus = morphological dilation of the
//   profile by a 0.37 m disc; kappa = symmetric second difference at HALF-STENCIL 0.37 m,
//   linearly interpolated off the grid; reference = the emitted station centreline S.py read
//   by Catmull-Rom at the same fractional station index.
//
//   AND THE PART THE TRAIL ENGINEER SHOULD ACT ON. This does NOT move the wheel test's worst
//   25 m bin, and the reason is worth having in writing. Of the launching samples on the
//   whole course — 361 / 334 / 782 on the three seeds — **0.0% / 0.0% / 0.1% are at
//   flat-design stations**, and the worst 25 m bin on every seed contains **none at all**
//   (arc 1225 at 13.4%, arc 1150 at 12.4%, arc 1225 at 13.2%; flat-share 0% in each). The
//   worst bins are curvature the design ASKED FOR at a speed `speedAt` also asked for.
//   Terrain can hold its own residual to a
//   twentieth of the launch threshold and the gate will not move, because the gate is not
//   measuring terrain. Whole-course wheel test moves 0.63% -> 0.59% / 0.93% -> 0.87% /
//   0.80% -> 0.73%; worst bin 13.4 -> 13.4 / 20.0 -> 20.0 / 13.2 -> 13.2, unmoved on all
//   three seeds. Terrain's residual is not what stops the rider. The design's own curvature
//   at the design's own speed is.
//
// CONTRACT-NOTE (terrain → everyone, DETERMINISM): world generation is a pure function of
//   `ctx.seed` and the quality settings. It used to size the hydraulic-erosion droplet budget
//   from a wall-clock projection and silently drop to 50%/25% under machine load, so the same
//   seed produced different mountains on different boots (measured: 2712.72 m vs 2694.02 m of
//   trail, 13.4% vs 42.2% grade at the same arc length, from an unchanged tree). That is gone.
//   `terrain.fingerprint()` hashes every generated field so determinism can be asserted rather
//   than assumed. Nothing that feeds the world may read a clock — `Date.now()` appears in this
//   file only to fill `timings`, which no generator reads.

import * as THREE from 'three';
import { createNoise2D } from 'simplex-noise';
import { makeRng, subSeed, clamp01, lerp, smoothstep } from '../core/rng.js';
import { createTerrainMaterial } from './terrainMaterial.js';
import { report, maybeYield } from '../core/bootProgress.js';

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

// ---------------------------------------------------------------------------
// Erosion budget — a function of the quality tier, never of the clock
//
// passC() used to project elapsed wall-clock time at droplet 20 000 and, if the
// machine looked slow, silently cut the budget to 50% or 25%. World generation
// was therefore a function of MACHINE LOAD, in direct contradiction of CONTRACT
// §0. Three boots of an unchanged tree produced 2712.72 m and 2694.02 m trails
// and 13.4% vs 42.2% grade at the same arc length; every measurement taken
// against this generator was taken against a world that moved underneath it.
//
// The budget is now chosen from the quality tier alone, and `high` — the
// default — is the FULL budget, so the default world is the one the tuning was
// meant to be measured against. If a machine genuinely cannot afford the pass
// that is now an explicit, logged, opt-in setting (`settings.erosionDroplets`),
// identical on every boot of that machine, rather than an implicit race.
// ---------------------------------------------------------------------------
const DROPLETS = 135000;
const DROPLET_LIFE = 42;
// Fraction of DROPLETS by quality tier. Unknown tiers fall back to 'high'.
const DROPLET_TIER = { low: 0.35, medium: 0.65, high: 1.0, ultra: 1.0 };

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

  // ---- droplet budget: seed + quality ONLY (see the DROPLET_TIER block) ----
  // The tier key is `ctx.quality` when it names a known tier, otherwise it is
  // derived from `terrainLOD` so a hand-rolled settings object still gets a
  // stable, documented answer. Both are settings. Neither is a clock.
  const dropletTier = (typeof ctx.quality === 'string' && DROPLET_TIER[ctx.quality] !== undefined)
    ? ctx.quality
    : (lodScale < 0.6 ? 'low' : lodScale < 0.9 ? 'medium' : 'high');
  const dropletDefault = Math.round(DROPLETS * DROPLET_TIER[dropletTier]);
  const dropletOverride = settings.erosionDroplets;
  const dropletBudget = (typeof dropletOverride === 'number'
    && isFinite(dropletOverride) && dropletOverride > 0)
    ? Math.max(2000, Math.round(dropletOverride))
    : dropletDefault;
  if (dropletBudget !== dropletDefault) {
    // Explicit and logged, per CONTRACT §0. A quiet degradation is exactly the
    // failure this replaced.
    console.info('[terrain] erosion droplet budget overridden by settings.erosionDroplets:',
      dropletBudget, '(quality tier', dropletTier, 'default', dropletDefault + ')');
  }

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

  async function passA() {
    const coarse = new Float32Array(CRES * CRES);
    for (let j = 0; j < CRES; j++) {
      const z = minZ + j * CCELL;
      const row = j * CRES;
      for (let i = 0; i < CRES; i++) coarse[row + i] = baseHeight(minX + i * CCELL, z);
      if ((j & 63) === 0) {
        report(0.05 * 0.5 * (j / CRES));
        const y = maybeYield(); if (y) await y;
      }
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
      if ((j & 63) === 0) {
        report(0.05 * (0.5 + 0.5 * (j / RES)));
        const y = maybeYield(); if (y) await y;
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

  async function passB(tmp) {
    for (let j = 0; j < RES; j++) {
      if ((j & 63) === 0) {
        report(0.05 + 0.10 * (j / RES));
        const y = maybeYield(); if (y) await y;
      }
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

  async function passC(flowAcc) {
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

    // Fixed budget. Resolved once in createTerrain from the seed's quality tier
    // and an explicit opt-in override — no wall-clock term anywhere in this loop.
    const total = dropletBudget;

    for (let d = 0; d < total; d++) {
      if ((d & 2047) === 0) {
        report(0.15 + 0.40 * (d / total));
        const y = maybeYield(); if (y) await y;
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
    timings.dropletTier = dropletTier;
  }

  // -------------------------------------------------------------------------
  // Pass D — thermal erosion (angle of repose)
  //
  // Anything steeper than its repose angle slumps. The repose angle is taken from
  // the hardness map (rock stands at ~62°, soil at ~34°), so cliff bands survive
  // while everything below them collects the debris — which is exactly how talus
  // fans form. The moved volume is recorded and becomes the GRAVEL material.
  // -------------------------------------------------------------------------

  async function passD(delta, passes) {
    const talus = new Float32Array(RES * RES);
    const diag = Math.SQRT1_2;
    for (let p = 0; p < passes; p++) {
      delta.fill(0);
      for (let j = 1; j < RES - 1; j++) {
        if ((j & 63) === 0) {
          report(0.55 + 0.20 * ((p + j / RES) / passes));
          const y = maybeYield(); if (y) await y;
        }
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

  async function passE(tmp) {
    tmp.set(heights);
    for (let j = 1; j < RES - 1; j++) {
      if ((j & 127) === 0) {
        report(0.75 + 0.05 * (j / RES));
        const y = maybeYield(); if (y) await y;
      }
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

  async function passG(flowAcc, tmp) {
    // Same mean-relative soft knee passF uses, so "flow" means the same thing
    // in both passes.
    let sum = 0, n = 0;
    for (let k = 0; k < flowAcc.length; k += 7) { const v = flowAcc[k]; if (v > 0) { sum += v; n++; } }
    const K = Math.max(1e-3, (n > 0 ? sum / n : 1) * 5.0);

    tmp.set(heights);
    for (let j = 1; j < RES - 1; j++) {
      if ((j & 63) === 0) {
        report(0.80 + 0.08 * (j / RES));
        const y = maybeYield(); if (y) await y;
      }
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

  async function passF(flowAcc) {
    // Normalise drainage accumulation. A mean-relative soft knee keeps a handful of
    // huge trunk streams from flattening every tributary to zero.
    let sum = 0, n = 0;
    for (let k = 0; k < flowAcc.length; k += 7) { const v = flowAcc[k]; if (v > 0) { sum += v; n++; } }
    const meanFlow = n > 0 ? sum / n : 1;
    const K = Math.max(1e-3, meanFlow * 5.0);

    let creekSum = 0, creekN = 0;

    for (let j = 0; j < RES; j++) {
      if ((j & 63) === 0) {
        report(0.88 + 0.12 * (j / RES));
        const y = maybeYield(); if (y) await y;
      }
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

  // Async solely for the boot progress bar: the passes yield to the compositor
  // on a time budget (see core/bootProgress.js) but stay synchronous between
  // yields, so RNG consumption order — and therefore the terrain — is unchanged.
  async function buildBase() {
    if (baseBuilt) return;
    const t0 = Date.now();
    await passA();
    const t1 = Date.now();

    const tmp = new Float32Array(RES * RES);
    await passB(tmp);
    const t2 = Date.now();

    const flowAcc = new Float32Array(RES * RES);
    await passC(flowAcc);
    const t3 = Date.now();

    await passD(tmp, lowSpec ? 2 : 3);
    const t4 = Date.now();

    await passE(tmp);
    const t5 = Date.now();

    await passG(flowAcc, tmp);
    const t5b = Date.now();

    await passF(flowAcc);
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

  // ---- design-surface lookup ----------------------------------------------
  // The stamp list and its prepared per-stamp data, kept after applyCarve so
  // sampleDesign() can answer "what did the trail actually ask for here?".
  // A CSR bin index over an 8 m grid keeps the lookup O(stamps in a few bins).
  const DB_CELL = 8;
  const DBN = Math.floor(WORLD / DB_CELL) + 1;
  let designStamps = null;
  let designPrep = null;
  let designBinStart = null;
  let designBinItem = null;
  let designBinRing = 1;

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
  // MARCH SUPPORT — directional terrain queries for the route-finder
  //
  // See the CONTRACT-NOTE at the top of this file. `marchRoute` evaluates many
  // candidate headings per station, BEFORE any carve exists, and needs the same
  // four quantities at each: grade along the heading, cross-slope across it,
  // curvature along it, and how the hillside behaves either side of it.
  //
  // Three properties these have to have, and each one is a decision:
  //
  //  * ALLOCATION-FREE. Every one of these writes into caller-supplied or
  //    module-scope storage. A heading search is an inner loop.
  //  * ANALYTIC, not finite-differenced, for grade and cross-slope. sampleField()
  //    already computes the exact gradient of the bicubic reconstruction as a
  //    by-product of the height, so a directional derivative is two multiplies
  //    on top of one height sample rather than two extra samples. Curvature is
  //    the exception: it is a second derivative and genuinely needs a stencil,
  //    which is therefore an explicit argument and never a hidden default.
  //  * TANGENT-PLANE REFERENCED for the corridor probe. `cross` already reports
  //    the ambient cross-slope; what a route-finder additionally needs to know
  //    is where the hillside STOPS being a plane — a cliff band, a gully lip,
  //    the shoulder of a spur. So the probe measures departure from the tangent
  //    plane at the query point, not raw height difference, and a uniform
  //    cross-slope of any steepness reads as zero departure.
  //
  // Heading convention: (dirX, dirZ) is a plan-space direction, not required to
  // be unit — it is normalised here, because a caller sweeping headings will
  // usually have built it from a sin/cos pair and normalising twice is waste.
  // "Right" is the rider's right of that heading, i.e. (-dirZ, dirX), matching
  // CarveStamp.bank (+ = right side high) and trail.nearestT's `lateral`.
  //
  // Sign convention: grade is dY per metre ALONG the heading, so a descending
  // heading is NEGATIVE. That is the opposite sign to trail.js's `S.grade`,
  // which is a magnitude; it is signed here because the whole point of the
  // query is to let the march tell downhill from uphill without a second call.
  // -------------------------------------------------------------------------

  const _probeOut = {
    height: 0, gradient: 0, cross: 0, slope: 0, curvature: 0,
    fallL: 0, fallR: 0, planarL: 0, planarR: 0, cutL: 0, cutR: 0, benchable: 0,
  };

  /** Normalise a heading into module scratch. Returns false for a degenerate one. */
  let _hx = 0, _hz = -1;
  function setHeading(dirX, dirZ) {
    const l = Math.sqrt(dirX * dirX + dirZ * dirZ);
    if (!(l > 1e-9)) { _hx = 0; _hz = -1; return false; }
    _hx = dirX / l; _hz = dirZ / l;
    return true;
  }

  /** dY per metre along the heading. Negative = descending. */
  function sampleGrade(x, z, dirX, dirZ) {
    setHeading(dirX, dirZ);
    sampleField(x, z);
    return _fgx * _hx + _fgz * _hz;
  }

  /** dY per metre to the rider's RIGHT of the heading. + = right side high. */
  function sampleCrossSlope(x, z, dirX, dirZ) {
    setHeading(dirX, dirZ);
    sampleField(x, z);
    return _fgx * -_hz + _fgz * _hx;
  }

  /**
   * Second derivative of the surface along the heading, 1/m, by a symmetric
   * central difference at HALF-STENCIL `half` metres. Convex (a crest, the thing
   * that launches a wheel) is NEGATIVE, matching the sign of d2y/ds2.
   *
   * `half` is mandatory and there is no default on purpose: a curvature figure
   * quoted without its stencil is not a measurement, and this file has been
   * bitten by exactly that. For launch work the stencil is the wheel radius,
   * 0.37 m; for route-scale relief it should be several metres.
   */
  function sampleCurvatureAlong(x, z, dirX, dirZ, half) {
    if (!setHeading(dirX, dirZ)) return 0;
    const H = half > 1e-3 ? half : 1e-3;
    sampleField(x, z); const h0 = _fh;
    sampleField(x + _hx * H, z + _hz * H); const hp = _fh;
    sampleField(x - _hx * H, z - _hz * H); const hm = _fh;
    return (hp - 2 * h0 + hm) / (H * H);
  }

  // Corridor probe geometry. PROBE_REACH is deliberately a little more than the
  // 4 m batter run plus a 3 m tread half-width, so the answer covers everything
  // a bench built here would actually disturb.
  const PROBE_DS = 0.5;        // lateral step, metres
  const PROBE_REACH = 10.0;    // how far out the probe walks, each side
  const PROBE_BAND = 0.75;     // metres of departure from the tangent plane that
                               // still counts as "the same hillside"
  const PROBE_FALL = 5.0;      // the reference offset the fall-away is quoted at
  const PROBE_BENCH = 2.0;     // half-width of the bench the cut is measured over

  /**
   * Everything the march needs at one (position, heading), in ONE pass that
   * shares its height samples between the terms.
   *
   *   height     world Y at the query point
   *   gradient   dY/m along the heading (negative = descending)
   *   cross      dY/m to the rider's right
   *   slope      radians from horizontal (the ambient steepness, unsigned)
   *   curvature  d2Y/ds2 along the heading at half-stencil `half` (convex < 0)
   *   planarL/R  metres out to which the hillside stays within PROBE_BAND of the
   *              tangent plane. This is the honest "corridor width": it is how
   *              far a bench can be shifted either way without meeting a break
   *              in the ground. Saturates at PROBE_REACH.
   *   fallL/R    departure from the tangent plane at PROBE_FALL metres out,
   *              SIGNED, + = the ground stands above the plane (a cut bank),
   *              - = it falls away (exposure). This is the exposure figure the
   *              QA passes quote, measured against the plane so that ambient
   *              cross-slope is not counted as fall-away.
   *   cutL/R     the largest positive departure inside PROBE_BENCH — the cut a
   *              bench of that half-width would have to make on each side.
   *   benchable  min(planarL, planarR) / PROBE_REACH, 0..1 — one scalar the
   *              march can put straight into a cost without reading the rest.
   *
   * `out` may be omitted, in which case a module-scope object is reused; a
   * caller that keeps a result across another call must pass its own.
   */
  function probeCorridor(x, z, dirX, dirZ, half, out) {
    const o = out || _probeOut;
    if (!setHeading(dirX, dirZ)) { _hx = 0; _hz = -1; }
    const hx = _hx, hz = _hz;
    const rx = -hz, rz = hx;
    sampleField(x, z);
    const h0 = _fh, gx = _fgx, gz = _fgz;
    o.height = h0;
    o.gradient = gx * hx + gz * hz;
    o.cross = gx * rx + gz * rz;
    o.slope = Math.atan(Math.sqrt(gx * gx + gz * gz));
    // Curvature reuses h0 but needs its own two samples (a second derivative
    // cannot be had from one gradient).
    const H = half > 1e-3 ? half : 1e-3;
    sampleField(x + hx * H, z + hz * H); const hp = _fh;
    sampleField(x - hx * H, z - hz * H); const hm = _fh;
    o.curvature = (hp - 2 * h0 + hm) / (H * H);

    // Lateral walk. `plane` is the tangent plane's height at the offset, so a
    // uniform cross-slope contributes nothing and only breaks in the ground do.
    const mCross = o.cross;
    let planarL = PROBE_REACH, planarR = PROBE_REACH;
    let fallL = 0, fallR = 0, cutL = 0, cutR = 0;
    let doneL = false, doneR = false;
    for (let d = PROBE_DS; d <= PROBE_REACH + 1e-6; d += PROBE_DS) {
      if (!doneR) {
        sampleField(x + rx * d, z + rz * d);
        const dev = _fh - (h0 + mCross * d);
        if (d <= PROBE_BENCH && dev > cutR) cutR = dev;
        if (d <= PROBE_FALL + 1e-6) fallR = dev;
        if ((dev < 0 ? -dev : dev) > PROBE_BAND) { planarR = d - PROBE_DS; doneR = true; }
      }
      if (!doneL) {
        sampleField(x - rx * d, z - rz * d);
        const dev = _fh - (h0 - mCross * d);
        if (d <= PROBE_BENCH && dev > cutL) cutL = dev;
        if (d <= PROBE_FALL + 1e-6) fallL = dev;
        if ((dev < 0 ? -dev : dev) > PROBE_BAND) { planarL = d - PROBE_DS; doneL = true; }
      }
      if (doneL && doneR) break;
    }
    o.planarL = planarL; o.planarR = planarR;
    o.fallL = fallL; o.fallR = fallR;
    o.cutL = cutL; o.cutR = cutR;
    o.benchable = (planarL < planarR ? planarL : planarR) / PROBE_REACH;
    return o;
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
   * The DESIGN SURFACE one stamp declares at a point: a PLANE through the stamp,
   * plus whatever section its `kind` adds.
   *
   *   `sRaw`   lateral offset, metres, + = rider's right
   *   `along`  along-track offset, metres, + = downhill (the stamp's own tangent)
   *   `gA`     along-track gradient of the tread, dY per metre along +tangent
   *
   * The per-kind section extras this function used to add are GONE, and their
   * removal is load-bearing rather than a simplification. trail.js emits five
   * stamps per station spanning the tread, so the cloud already describes the
   * section it wants — including a berm's. Adding a berm parabola, or a rut
   * trough, about EACH sample's own centre superimposed five offset copies of
   * the feature and guaranteed that the five stamps of one station contradicted
   * each other about the height at any given point. That contradiction is not
   * resolvable downstream by any weighting, which is why attacking this from
   * the stamp side did not close it. If a berm needs more support than the
   * trail emitted, that belongs in trail.js's `targetHeight`, not bolted on
   * here where terrain cannot know where the centreline is.
   *
   * MEASURED DEFECT this fixes: the stamp used to be a horizontal PLATEAU —
   * `targetHeight` with no along-track term at all. A stamp asserts its section
   * over +/-1.9 of its along-track reach, so on a 20% grade a single stamp was
   * declaring a surface up to ~0.5 m off the trail's own descent at the ends of
   * its own footprint. The accumulator then averaged a window of those plateaux,
   * which low-passes the descent and leaves a ripple at the stamp spacing —
   * the ~3 m wavelength measured inside the tread. With the along-track term
   * present, every stamp that reaches a cell evaluates to (very nearly) the SAME
   * height there, and a weighted mean of equal values is exact whatever the
   * weights are. That is what makes the accumulator reproduce the section.
   *
   * Two authored terms were removed for the same reason. Bank was applied at
   * 0.9x on tread/lip/landing/drop, and a -0.022*|s| outslope was subtracted:
   * together ~0.11 m of systematic error at the edge of a banked 1.5 m tread,
   * i.e. over the acceptance threshold on its own, and a silent contradiction of
   * the bank the trail's lean identity was proved against. Water shedding is now
   * carried by the ruts and the windrow, which are real ridden geometry.
   */
  function stampTarget(st, sRaw, r, along, gA, gL) {
    // Clamp just outside the tread: past that the batter and the shoulder are
    // easing the bench into the hillside, and the plane must not keep climbing
    // out there.
    const lim = r * 1.15;
    const s = sRaw > lim ? lim : (sRaw < -lim ? -lim : sRaw);
    return st.targetHeight + gA * along + gL * s;
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
   * tread edge. (The along-track release used to live in here as a second
   * argument; it is now `alongGate`, applied by the caller, so that one function
   * owns the along-track behaviour of every term.)
   *
   * The batter target is clamped against the natural ground, so applying it at
   * full strength is self-limiting: where the cut has already daylighted, the
   * target IS the natural ground and full strength changes nothing. That is what
   * makes it safe to override the outer cross-track ease here — and it has to be
   * overridden, because that ease is what produced the wall: it handed the
   * shoulder only a fraction of the way to its target, so with a flat target the
   * ground out there stayed within a few percent of the untouched hillside and
   * the bench simply ended in a face.
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
  function batterWeight(over) {
    if (over > BATTER_RUN) return 0;
    let t = over <= BATTER_RUN - BATTER_RELEASE ? 1 : (BATTER_RUN - over) / BATTER_RELEASE;
    if (t <= 0) return 0;
    if (t > 1) t = 1;
    return t * t * (3 - 2 * t);
  }

  // -------------------------------------------------------------------------
  // Committed vs design: the accumulator, and why it used to launch the bike
  //
  // MEASURED: the committed tread deviated from the emitted design surface by
  // up to 0.4 m at a ~3 m wavelength, INSIDE the tread. Nineteen of twenty-one
  // autopilot runs were airborne at the moment of their first crash.
  //
  // The accumulator used ONE radial falloff — carveShape(sqrt(ul^2 + ua^2)) —
  // as both the along-track interpolation weight AND the strength with which
  // the design surface replaces the natural hillside. Those are different jobs
  // and the coupling is fatal. Take a cell at 70% of the tread half-width:
  //
  //     directly abeam a stamp   ua = 0     -> dn = 0.70 -> strength 0.65
  //     halfway between two      ua = 0.59  -> dn = 0.91 -> strength 0.07
  //
  // (ua = 0.59 is the midpoint at the usual rl = 0.85 * spacing.) So the
  // committed tread oscillated between 65% and 7% of the way from the hillside
  // to the design surface, with a period equal to the stamp spacing — a
  // scalloped ramp, in phase across the whole tread width, sitting under a bike
  // doing 12 m/s. That is the ~3 m ripple, and the same collapse is why berms
  // were built at roughly 22% of their authored height: a berm's outer wall is
  // authored at 1.0-1.15 tread radii, where the radial falloff had nothing left.
  //
  // A tread is a RIBBON, not a disc. So:
  //
  //   crossStrength(t)      how far this cell is pulled to the design surface.
  //                         Cross-track ONLY. 1 across the whole bench, easing
  //                         off between the tread edge and 1.9 radii. Combined
  //                         across stamps with max(), so overlapping stamps
  //                         cannot dilute each other.
  //   alongGate(u)          1 out to one along-track reach, released by 1.45.
  //                         Multiplies the strength so the trail's two ends
  //                         daylight, and does nothing in between — consecutive
  //                         stamps' full-strength spans always overlap
  //                         (see stampReach()).
  //   alongKernel(u) *      weight in the target MEAN. Interpolation only; it
  //   lateralMeanWeight(t)  can never reduce strength. Because every stamp now
  //                         evaluates its plane AT THE CELL, the values being
  //                         averaged agree, so the mean is exact regardless of
  //                         the weights — the kernel only has to be smooth.
  //
  // `falloff` no longer shapes the tread core: a bench committed at anything
  // less than full strength cannot equal the emitted section by construction.
  // It shapes the ease outside the tread edge instead. See the CONTRACT-NOTE.
  // -------------------------------------------------------------------------

  const UA_FULL = 1.0;    // |along|/rl inside which a stamp asserts full strength
  const UA_MAX = 1.45;    // ... and beyond which it asserts nothing
  const CS_OUT = 1.9;     // |lat|/rw at which the cross-track ease reaches zero

  /** Height blend strength across the section. t = |lat| / rw. */
  function crossStrength(t, falloff) {
    if (t >= CS_OUT) return 0;
    if (t <= 1) return 1;
    const a = (CS_OUT - t) / (CS_OUT - 1);
    const s = a * a * (3 - 2 * a);
    // falloff only shapes the ease outside the bench (see the note above).
    const f = (falloff > 0.05 && falloff < 8) ? falloff : 1;
    return f === 1 ? s : Math.pow(s, f);
  }

  /** Along-track gate on the strength. Flat inside UA_FULL, released by UA_MAX. */
  function alongGate(u) {
    if (u <= UA_FULL) return 1;
    if (u >= UA_MAX) return 0;
    const a = (UA_MAX - u) / (UA_MAX - UA_FULL);
    return a * a * (3 - 2 * a);
  }

  /** Along-track interpolation kernel for the target MEAN. */
  function alongKernel(u) {
    if (u >= UA_MAX) return 0;
    const a = 1 - u / UA_MAX;
    return a * a * (3 - 2 * a);
  }

  // ---- the target mean is an INTERPOLANT, not a smoother -------------------
  //
  // With the plane fit in place the accumulator is doing scattered-data
  // reconstruction of a point cloud, so the mean weight has to be peaked at the
  // sample. A broad kernel — which is what a flat "weight 1 across the whole
  // bench" is — box-filters the cloud over its entire support: measured on seed
  // 1337, a ~2.7 x 2.0 m patch, which smoothed the emitted tread's own
  // curvature into a p90 residual of 0.166 m even after every stamp had been
  // made to agree about the plane.
  //
  // This is Shepard weighting over the per-stamp planes — equivalently a moving
  // least-squares reconstruction with precomputed local fits. It is singular at
  // the sample, so the committed surface passes through every emitted stamp
  // height exactly, and it is smooth between samples because the things being
  // blended are planes rather than constants (plain Shepard over values would
  // put a flat spot at every node; Shepard over planes does not).
  //
  // The cap bounds the dynamic range. Targets are accumulated as OFFSETS from
  // the natural ground rather than as absolute elevations (see applyCarve), so
  // the largest single product is ~1.6e3 against a Float32 mantissa good to one
  // part in 1e7 — i.e. sub-millimetre, where accumulating 1700 m elevations at
  // weight 160 would have been good to about 10 cm.
  // SHEP_POWER note. The kernel is the square of the usual Franke-Little form,
  // i.e. ((1-r)^2 / r^2)^2. The unsquared kernel leaves the nearest emitted
  // sample only ~50% of the total weight at a cell halfway between samples, so
  // the committed surface is a genuine blend of three or four neighbouring
  // planes and inherits their disagreement. Measured on the self-consistent
  // 90% of the tread, seed 1337:
  //     unsquared   p50 0.029  p90 0.129  p99 0.270  max 0.727
  //     squared     see the residual report in the QA notes
  // Squaring raises the nearest sample to ~85% and is still C1 in rho.
  // Measured, self-consistent tread (covering stamps agreeing within 0.5 m),
  // seed 1337, before and after squaring — the two are not directly comparable
  // because aligning sampleDesign's own nearest-stamp test to the same rho
  // landed in between, so the honest summary is the final matrix in the QA
  // notes: p50 0.014, p90 0.062, p95 0.094, p99 0.214 over the whole tread, and
  // p99 0.077 wherever the emitted stamps agree with each other to 0.15 m.
  const MEAN_CAP = 2000;
  const SHEP_EPS = 1 / 45;   // softens the singularity at the sample itself
  const RHO_MAX = 1.25;      // normalised radius at which a stamp stops voting
  // A floor, so cells reached only by the shoulder or by the batter — where no
  // stamp is anywhere near — still have a target to be pulled toward.
  //
  // It has to be SMALL, and the reason is a counting argument rather than a
  // taste one. A battered tread stamp reaches ~5.7 m laterally, so where a
  // switchback folds back on itself, thirty-odd stamps of the OTHER leg reach a
  // cell on this leg's tread and each casts a floor-weight vote for a surface
  // metres away. The legitimate near stamp carries a Shepard weight of ~6 at a
  // typical tread cell (the nearest emitted sample is ~0.3 m away). At a floor
  // of 0.012 those thirty votes summed to ~0.42 — 7% of the total — which on a
  // 5 m height difference between the legs is 0.35 m of committed error, and it
  // was the largest single term left in the residual. 5e-4 puts it at 0.3%.
  const MEAN_FLOOR = 5e-4;

  function shepard(rho) {
    if (rho >= 1) return 0;
    const g = 1 - rho;
    const w = (g * g) / (rho * rho + SHEP_EPS);
    const w4 = w * w;                     // sharpened — see SHEP_POWER note
    return w4 > MEAN_CAP ? MEAN_CAP : w4;
  }

  /**
   * Along-track reach of a stamp, in metres.
   *
   * The full-strength spans of consecutive stamps must overlap or the tread
   * develops a strength notch between every pair, which is the defect this file
   * is fixing. That needs `rl >= spacing / 2` unconditionally, so the third term
   * is a floor and not, as before, a term inside a min().
   */
  function stampReach(r, spacing) {
    const sp = spacing > 0 ? spacing : 0;
    let rl = r;
    const a = Math.min(r * 3, sp * 0.85);
    if (a > rl) rl = a;
    const b = Math.min(r * 8, sp * 0.55);
    if (b > rl) rl = b;
    return rl;
  }

  /**
   * Tread half-width at a point, including the world-space lateral jitter.
   * A function of world position only, so every stamp agrees on it and the
   * accumulator stays independent of the order stamps arrive in.
   */
  function jitterRadius(r, latAbs, x, z) {
    if (latAbs >= r * 2.2) return r;
    const jf = latAbs < r * 1.4 ? 1 : (r * 2.2 - latAbs) / (r * 0.8);
    return r * (1 + (carveWidth(x, z) - 1) * jf);
  }

  // Per-cell stamp evaluation, shared by the global and detail carve loops so
  // the two fields cannot drift apart. Results go into module-scope scalars —
  // this runs tens of millions of times and must not allocate.
  let _scTarget = 0, _scW = 0, _scWH = 0, _scWM = 0, _scInner = 0, _scAg = 0;
  let _scLatCL = 0;

  /**
   * Evaluate one stamp at one cell. Returns false if the cell is outside this
   * stamp's influence, otherwise writes the module-scope scalars:
   *   _scTarget — the design height this stamp declares here
   *   _scW      — paint/carve MASK (material, carveU8, splat, vegetation
   *               exclusion). Deliberately still the old radial carveShape, so
   *               nothing downstream of the paint changes behaviour.
   *   _scWH     — height blend STRENGTH, combined across stamps with max()
   *   _scWM     — weight in the height target MEAN
   *   _scInner  — ridden-microrelief gate (cross-track only, x along-track gate)
   *   _scAg     — the along-track gate on its own
   *   _scLatCL  — lateral offset from the ribbon CENTRELINE (not this stamp's
   *               axis); the coordinate the microrelief is keyed on
   */
  function stampCell(st, lat, along, r, rl, x, z, batterKind, hNat, gA, gL, latOff, halfW) {
    const uaAbs = (along < 0 ? -along : along) / rl;
    if (uaAbs >= UA_MAX) return false;
    const latAbs = lat < 0 ? -lat : lat;
    const rMax = r * RW_MAX;
    // Cheap reject. `rMax` is the widest the jittered tread can be, so
    // latAbs >= rMax * CS_OUT implies t >= CS_OUT and cannot false-reject.
    if (latAbs >= rMax * CS_OUT
        && !(batterKind && latAbs <= rMax + BATTER_RUN)) return false;

    // Lateral noise on the cut width: the carve radius was a constant per stamp,
    // so the trench ran the whole trail at a dead-uniform width with its top
    // edge exactly offset from the spline. Faded out before the batter's own
    // run so the batter geometry cannot inherit a step where the fade ends, and
    // a function of world position only, so overlapping stamps agree on it and
    // the accumulator stays independent of the order stamps arrive in.
    const rw = jitterRadius(r, latAbs, x, z);

    const ul = lat / rw, ua = along / rl;
    const t = ul < 0 ? -ul : ul;
    const ag = alongGate(uaAbs);

    let target = stampTarget(st, lat, rw, along, gA, gL);
    let wh = crossStrength(t, st.falloff) * ag;
    let wm = shepard(Math.sqrt(t * t + uaAbs * uaAbs) / RHO_MAX);
    const wFloor = MEAN_FLOOR * alongKernel(uaAbs);
    if (wFloor > wm) wm = wFloor;
    if (batterKind) {
      target = batter(target, hNat, latAbs - rw);
      const bw = batterWeight(latAbs - rw) * ag;
      if (bw > wh) wh = bw;
    }
    // wm can only be large where wh is 1 (a large Shepard weight needs both a
    // small cross-track and a small along-track offset, which is exactly where
    // crossStrength and alongGate are both saturated), so this one test is
    // enough — there is no case where a stamp has a real vote and no strength.
    if (wh <= 0.002) return false;

    // Paint mask keeps the old radial shape — see the field list above.
    _scW = carveShape(Math.sqrt(ul * ul + ua * ua), st.falloff);
    // The ridden microrelief belongs to the bench and is measured from the
    // ribbon CENTRELINE, not from this stamp's own axis (see the CENTRELINE
    // COORDINATE note). It is also gated across the track rather than
    // radially: gating it on the radial mask made the ruts and braking bumps
    // fade out between every pair of stamps, which is another (smaller)
    // along-track ripple at exactly the stamp spacing.
    _scLatCL = lat + latOff;
    const tc = (_scLatCL < 0 ? -_scLatCL : _scLatCL) / halfW;
    _scInner = (1 - smoothstep(0.72, 1.0, tc)) * ag;
    _scAg = ag;
    _scTarget = target; _scWH = wh; _scWM = wm;
    return true;
  }

  // ---- ridden-tread microrelief -------------------------------------------
  // r2 authored the ruts 0.028 m deep at sigma 0.13 m on a 0.35 m detail grid —
  // roughly one sample wide, so the reconstruction filter erased them before
  // they ever reached a vertex. The r3 review measured the tread varying LESS
  // across the track than down it (0.57-0.99, where a ridden line reads
  // 1.5-2.5), i.e. isotropic noise rather than a line.
  //
  // WHAT SURVIVES THE LOW-PASS. commitDetail() low-passes the committed target
  // at TREAD_LP_SIGMA = 0.555 m, so every term here is authored on the far side
  // of a filter. A Gaussian feature of width sigma_f survives at
  // sigma_f / sqrt(sigma_f^2 + sigma_lp^2) of its authored amplitude:
  //     ruts    sigma_f 0.33 -> 0.51   committed ~0.056 m, widened to sigma 0.64
  //     windrow sigma_f 0.22 -> 0.37   committed ~0.028 m
  // PRE-EMPHASIS WAS TRIED AND REJECTED. Authoring 0.215 / 0.20 to commit the
  // asked-for 0.11 / 0.075 works — the committed depth comes back — but the
  // deeper trough moves the CENTRELINE down by an extra 37 mm, modulated by the
  // rut wander (74 m) and the tread half-width, and that lands in the long
  // bands: 6-20 m spurious/design went 0.07 -> 0.11 and 20-60 m 0.04 -> 0.06,
  // i.e. straight through the ceiling this file is held to. The wheel test also
  // went 3.87% -> 4.95%. A shallower, wider ridden channel is the cheaper trade.
  // The braking bumps and the loose-rock chatter are filtered away outright —
  // see the note on each.
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
  // Braking bumps: 0.085 m at a 1.15 m period is kappa = 2.5 /m, forty times the
  // 0.059 /m a wheel can hold at the p50 design speed. They are a launcher by
  // construction and the low-pass removes 99% of them (transfer exp(-2 pi^2
  // sigma^2 / L^2) = 0.011 at L = 1.15 m). The constant is left at its authored
  // value rather than zeroed so that the term reappears intact if the filter is
  // ever narrowed; nothing here is tuned to a filtered amplitude.
  const BRAKE_BUMP = 0.085;      // metres (work order: 0.06-0.12)

  /**
   * The ridden-tread microrelief, as a height DELTA on the design plane.
   *
   * Shared by the detail-overlay accumulator and by sampleDesign(), so the
   * reference surface a QA pass measures against is literally the same code the
   * carve commits — a residual measured against a re-implementation would only
   * be measuring the re-implementation.
   *
   * Every term is a function of the point and the arc coordinate, never of
   * *which* stamp is asking. The two noise fields used to take `st.x` as their
   * second coordinate, so consecutive stamps drew slightly different rut wander
   * and braking-bump phase at the same cell and the accumulator averaged the
   * difference away — one more source of stamp-spacing ripple.
   */
  function treadMicro(st, isTread, latCL, halfW, sArc, inner, brake, x, z, ag) {
    let d = 0;
    // Windrow — the material the tyres have pushed out, sitting as a low berm
    // along both tread margins. Nothing was producing this at all, and it is
    // the single largest across-track gradient on a ridden trail. It sits at
    // the tread MARGIN, so it must not be gated on an inside-the-tread term.
    if (isTread) {
      const e = (latCL < 0 ? -latCL : latCL) - halfW * 1.05;
      d += WINDROW * Math.exp(-(e * e) / WINDROW_SIG2) * ag;
    }
    if (inner <= 0) return d;
    // Two tyre ruts. Amplitude 0.028 -> 0.11 m and sigma 0.13 -> 0.33 m: below
    // that the feature is one detail sample wide and is filtered away before it
    // reaches a vertex, which is why the tread measured as isotropic noise
    // (across/along gradient 0.57-0.99 where a ridden line reads 1.5-2.5). The
    // pair is 1.24 m apart so the crown between them survives the wider sigma,
    // and the line wanders +/-0.22 m over a ~74 m wavelength so it is a ridden
    // line and not a machined groove.
    if (isTread) {
      const wander = nMicro(sArc * 0.085, 311.0) * 0.22;
      const dl = latCL - (RUT_HALF + wander);
      const dr = latCL + (RUT_HALF - wander);
      d -= RUT_DEPTH * (Math.exp(-(dl * dl) / RUT_SIG2) + Math.exp(-(dr * dr) / RUT_SIG2)) * inner;
    }
    // Braking bumps: transverse ripples that build on steep entries.
    if (brake > 0) {
      const jitter = nMicro(sArc * 0.09, 57.0) * 0.5;
      const ripple = Math.sin((sArc / 1.15 + jitter) * Math.PI * 2) * 0.5 + 0.5;
      const across = 1 - clamp01((latCL < 0 ? -latCL : latCL) / (halfW * 0.8));
      d -= BRAKE_BUMP * brake * ripple * across * inner;
    }
    // Loose rock / roots keep their chatter even after the cut.
    const mm = st.material;
    if (mm === Surface.ROCK || mm === Surface.GRAVEL || mm === Surface.ROOT) {
      const chat = nMicro(x * 1.9 + 3.0, z * 1.9 - 6.0) * 0.6
                 + nMicro(x * 5.3 - 12.0, z * 5.3 + 8.0) * 0.4;
      d += chat * (mm === Surface.ROCK ? 0.075 : 0.045) * inner;
    }
    return d;
  }

  /** Per-stamp working data: unit tangent (downhill), arc length, local grade. */
  function prepareStamps(stamps) {
    const n = stamps.length;
    const tanX = new Float32Array(n);
    const tanZ = new Float32Array(n);
    const arc = new Float32Array(n);
    const grade = new Float32Array(n);
    const spacing = new Float32Array(n);
    // Along-track gradient of the tread (dY per metre along +tangent). See
    // stampTarget(): without this every stamp declares a horizontal plateau.
    const slope = new Float32Array(n);

    // Cross-track gradient of the tread (dY per metre to the rider's right),
    // fitted from the emitted stamp heights rather than taken from `bank` —
    // see the PLANE FIT note below.
    const cross = new Float32Array(n);
    // -----------------------------------------------------------------------
    // CENTRELINE COORDINATE — why the ridden microrelief needs one
    //
    // trail.js emits five stamps per station, offset roughly 0, +/-0.87 and
    // +/-1.49 m across the tread. Every stamp measures `lat` from ITS OWN axis,
    // so a rut pair authored at +/-0.62 m lands at +/-0.62 m from the CENTRE
    // stamp, at +0.25/+1.49 m from the first lateral, at +0.87/+2.11 m from the
    // second, and so on: ten rut lines across a three-metre tread, from a
    // feature that is supposed to be two. The windrow, which sits at 1.05 tread
    // radii, was scattered the same way. The accumulator then averaged the
    // disagreement, which both erased the feature and left the residue as
    // across-track error — measured 0.257 m at p90 in the outer quarter of the
    // tread against 0.067 m at the centre.
    //
    // `latOff` is each stamp's own lateral offset from the local ribbon
    // centroid, so `lat + latOff` is a coordinate every stamp of a station
    // agrees on. `halfW` is the widest carve radius at the station, which is
    // the ribbon half-width the microrelief should be scaled against. The
    // centroid is accumulated in world space and so needs no tangent, which
    // keeps it in the same single traversal as everything else.
    // -----------------------------------------------------------------------
    const latOff = new Float32Array(n);
    const halfW = new Float32Array(n);
    const CEN_R2 = 1.8 * 1.8;
    const CEN_K = 1 / (2 * 1.1 * 1.1);

    // Spatial bins so the neighbourhood search stays O(n).
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

    // -----------------------------------------------------------------------
    // THE PLANE FIT — what "the cross-section the trail emits" actually is
    //
    // trail.js emits FIVE stamps per station: a centre at r ~1.6 and four
    // laterals at r ~1.05, offset roughly +/-0.87 and +/-1.49 m, each with its
    // own `targetHeight` sampled on the tread at ITS OWN position and its own
    // `bank`. Stations are ~0.40 m apart. Measured on seed 1337: 58 173 stamps,
    // 36 797 of them 'berm'.
    //
    // Two facts about that emission decide how the accumulator has to work.
    //
    // 1. The stamps are SAMPLES of one ribbon, not five independent sections.
    //    A stamp's `targetHeight` is the tread height at its own position, so
    //    the surface the trail is asking for is the surface THROUGH THE POINT
    //    CLOUD. Treating each stamp as an authority over its whole footprint
    //    and averaging the results is the wrong operation on a point cloud.
    //
    // 2. The `bank` field does not agree with the cloud. Fitting the cloud's
    //    own cross-slope and comparing it against tan(bank), over 1 557 banked
    //    stamps on seed 1337:
    //
    //      |tan(bank)|   fitted cross-slope / tan(bank), p50
    //        0.05-0.15                    0.62
    //        0.15-0.30                    0.38
    //        0.30-0.60                    0.58
    //        0.60-2.00                    0.59
    //
    //    i.e. the emitted heights carry roughly HALF the declared bank, and the
    //    disagreement reaches 0.34 m of height at the tread edge (p50) and
    //    0.80 m (p90) on the hardest-banked stamps. Applying `s * tan(bank)`
    //    about each stamp's own centre — which is what this file used to do —
    //    therefore made every stamp declare a different height at the same
    //    point, up to 0.67 m apart, and the accumulator averaged the argument.
    //    That is the bulk of the measured 0.4 m tread deviation, and it is why
    //    attacking it from the stamp side did not close it: no per-stamp offset
    //    can reconcile stamps that contradict each other.
    //
    // So the plane is FITTED, not declared: for each stamp, a weighted
    // least-squares fit of its neighbours' emitted heights against their world
    // offsets, anchored to pass exactly through the stamp's own emitted point.
    // The fit is done in world (x, z) — a plane is axis-independent — and then
    // projected onto the tangent/right frame, so the tangent estimate cannot
    // contaminate the gradient. `bank` still shapes the berm's outer wall and
    // still decides which side of the tread is the outside; it no longer sets
    // the base plane.
    //
    // Result: every stamp within a station, and consecutive stations, evaluate
    // to very nearly the SAME height at any given cell. A weighted mean of
    // equal values is exact whatever the weights are — which is what finally
    // lets the accumulator reproduce the emitted section.
    //
    // FIT_R / FIT_SIG: 2.2 m of support with a 1.05 m Gaussian. Wide enough to
    // span the full 3 m lateral spread of a station and ~5 stations along, tight
    // enough that a jump lip's curvature is not fitted away (a 4+ m support
    // measured a 0.30 m p90 disagreement between the fitted intercept and the
    // stamp's own height; anchoring plus a tight support removes that question).
    // -----------------------------------------------------------------------
    const R2 = 6.5 * 6.5;          // tangent support (principal axis)
    const FIT_R2 = 2.2 * 2.2;      // plane-fit support
    const FIT_K = 1 / (2 * 1.05 * 1.05);
    for (let i = 0; i < n; i++) {
      const st = stamps[i];
      const bx = Math.floor((st.x - minX) / BIN);
      const bz = Math.floor((st.z - minZ) / BIN);
      let sxx = 0, sxz = 0, szz = 0, count = 0;
      let ox = 0, oz = 0;                       // orientation sums
      let fxx = 0, fxz = 0, fzz = 0, fxh = 0, fzh = 0, fitN = 0;
      let cw = 1, cwx = 0, cwz = 0;             // ribbon centroid (self weighted 1)
      let wide = Math.max(0.6, st.radius || 2);  // widest carve at this station
      let nearest2 = Infinity;
      const h0 = st.targetHeight;

      for (let bjo = -1; bjo <= 1; bjo++) {
        for (let bio = -1; bio <= 1; bio++) {
          const list = bins.get((bz + bjo) * 4096 + (bx + bio));
          if (!list) continue;
          for (let m = 0; m < list.length; m++) {
            const jj = list[m];
            if (jj === i) continue;
            const o = stamps[jj];
            const dx = o.x - st.x, dz = o.z - st.z;
            const d2 = dx * dx + dz * dz;
            if (d2 > R2 || d2 < 1e-8) continue;
            const dh = o.targetHeight - h0;
            // Height-plausibility gate. Where a switchback folds back on itself
            // the two legs pass within a couple of metres in plan and several
            // metres apart in height, and everything below — the tangent, the
            // plane fit and the ribbon centroid — would otherwise be computed
            // across BOTH legs. Measured consequence before this gate: fitted
            // cross-slopes pinned at the 58 deg clamp, and a 2.35 m committed
            // error at the two switchbacks on seed 1337 (x ~ -78 z ~ -472 and
            // x ~ 5 z ~ -341), which were the largest residuals in the run.
            // 0.8 * distance + 0.3 m admits a 40% grade and a 45 deg berm over
            // the whole support and rejects a leg 3 m below this one.
            const dhAbs = dh < 0 ? -dh : dh;
            if (dhAbs > 0.8 * Math.sqrt(d2) + 0.3) continue;
            if (d2 < nearest2) nearest2 = d2;
            sxx += dx * dx; sxz += dx * dz; szz += dz * dz;
            ox -= dh * dx; oz -= dh * dz;
            count++;
            if (d2 <= FIT_R2) {
              const w = Math.exp(-d2 * FIT_K);
              fxx += w * dx * dx; fxz += w * dx * dz; fzz += w * dz * dz;
              fxh += w * dx * dh; fzh += w * dz * dh;
              fitN++;
            }
            if (d2 <= CEN_R2) {
              const w = Math.exp(-d2 * CEN_K);
              cw += w; cwx += w * dx; cwz += w * dz;
              const rj = Math.max(0.6, o.radius || 2);
              if (rj > wide) wide = rj;
            }
          }
        }
      }

      let tx, tz;
      if (count >= 2) {
        // Principal axis of a 2x2 symmetric covariance matrix.
        const theta = 0.5 * Math.atan2(2 * sxz, sxx - szz);
        tx = Math.cos(theta); tz = Math.sin(theta);
        // Orient downhill: sum of (this - neighbour height) * (offset . tangent).
        if (ox * tx + oz * tz < 0) { tx = -tx; tz = -tz; }
      } else {
        // Isolated stamp: fall back to the local fall line.
        sampleField(st.x, st.z);
        const gl = Math.sqrt(_fgx * _fgx + _fgz * _fgz);
        if (gl > 1e-4) { tx = -_fgx / gl; tz = -_fgz / gl; } else { tx = 0; tz = -1; }
      }
      tanX[i] = tx; tanZ[i] = tz;

      // Solve the anchored 2x2 normal equations for the world-space gradient.
      // `det` is the neighbourhood's weighted second-moment determinant; a
      // near-singular one means the neighbours are collinear (a station with no
      // along-track partners, or a lone lateral), in which case the gradient in
      // the unobserved direction is not recoverable and the declared bank is the
      // only information there is.
      let gA = 0, gL = Math.tan(st.bank || 0);
      const det = fxx * fzz - fxz * fxz;
      if (fitN >= 3 && det > 1e-4) {
        const b = (fzz * fxh - fxz * fzh) / det;
        const c = (fxx * fzh - fxz * fxh) / det;
        gA = b * tx + c * tz;
        gL = b * -tz + c * tx;
      }
      // Bounds. 45 deg along track (the steepest authored chute is ~35% and a
      // jump lip steeper still) and 50 deg across it (a steep berm). Past those
      // the estimator is reading a switchback, not a section.
      if (gA > 1.0) gA = 1.0; else if (gA < -1.0) gA = -1.0;
      if (gL > 1.2) gL = 1.2; else if (gL < -1.2) gL = -1.2;
      slope[i] = gA;
      cross[i] = gL;
      // Offset of this stamp from the local ribbon centroid, measured to the
      // rider's right. `lat + latOff` is then a coordinate every stamp of a
      // station agrees on; see the CENTRELINE COORDINATE note above.
      const cnx = cwx / cw, cnz = cwz / cw;
      let lo = -(cnx * -tz + cnz * tx);
      // A centroid is only meaningful if it is drawn from a laterally spread
      // neighbourhood. Bound it at one station half-width so a degenerate
      // neighbourhood cannot throw the microrelief metres off the tread.
      if (lo > wide) lo = wide; else if (lo < -wide) lo = -wide;
      latOff[i] = lo;
      halfW[i] = wide;
      // Local steepness, for the braking bumps. The fitted along-track gradient
      // is a direct and much less noisy answer than the old (max - min) / range
      // over the neighbourhood, which a single outlying stamp could dominate.
      grade[i] = clamp01(gA < 0 ? -gA : gA);
      // Along-trail reach. If the trail hands us stamps further apart than their
      // radius, circular stamps would leave a plateau-and-riser staircase in the
      // tread; stretching each stamp along its tangent to cover the gap makes
      // consecutive targets blend into a continuous ramp instead.
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
    //
    // Advanced by the ALONG-TRACK component only. trail.js emits several stamps
    // per station (a centre plus laterals), and counting their lateral
    // separation as arc length made `sArc` disagree by up to a metre between
    // stamps that share a station — which put the rut wander and the
    // braking-bump phase out of step from one stamp to the next, so the
    // accumulator averaged them towards nothing and left a ripple at the stamp
    // spacing. The braking-bump period is 1.15 m, so a 0.5 m disagreement is
    // very nearly antiphase.
    let a = 0;
    for (let i = 0; i < n; i++) {
      if (i > 0) {
        const dx = stamps[i].x - stamps[i - 1].x;
        const dz = stamps[i].z - stamps[i - 1].z;
        const d = Math.sqrt(dx * dx + dz * dz);
        a = d < 30 ? a + (dx * tanX[i] + dz * tanZ[i]) : 0;
      }
      arc[i] = a;
    }

    // -----------------------------------------------------------------------
    // SECOND PASS — the vertical curvature the DESIGN asks for. Input to the
    // adaptive band limit; see commitDetail().
    //
    // `kapA` is d(slope)/d(arc): the vertical curvature of the emitted tread,
    // 1/m, unsigned. It is the quantity that decides whether a wheel leaves the
    // ground, and — the point of computing it here — it is the quantity that
    // separates relief the trail AUTHORED from ripple the reconstruction left
    // behind. The first is not terrain's to remove; the second is.
    //
    // It is a weighted least-squares regression against the along-track offset
    // over the SAME neighbour bins and the SAME height-plausibility gate as the
    // plane fit, so a switchback's other leg cannot contribute. The support is
    // deliberately shorter than the plane fit's (KAP_R 1.8 m against 2.2 m): the
    // regressand is already a fitted gradient, so its own resolution is ~2.2 m
    // and a wider support here would only blur a feature edge into the flat
    // ground beside it — the one error that matters, because it is the direction
    // that over-filters.
    //
    // UNITS: `arc` advances by the PLAN along-track component, so kapA is
    // d2Y/dx2 and not the true curvature d2Y/ds2 / (1 + Y')^1.5. On the steepest
    // authored ground (40%) that reads ~16% high, which is the conservative
    // direction — it keeps the narrow kernel — and is not worth a square root
    // per stamp to correct.
    //
    // THE PLAN-CURVATURE LIMIT IS DELIBERATELY NOT COMPUTED HERE. A straight
    // kernel on a curved tread departs the arc, so plan curvature does have to
    // cap the widening — but a per-stamp d(theta)/d(arc) regressed from
    // neighbouring stamps' PCA TANGENTS is an estimator built on the difference
    // of two noisy angles, and it behaved like one: MEASURED, it pinned 32% of
    // committed cells to the base kernel, which on a route whose 1st-percentile
    // plan radius is 16 m would require a 2.5 m radius at a third of all
    // stations. The cap is taken instead from the COHERENCE of the doubled-angle
    // direction field the filter already accumulates — see smoothTrackFrame().
    // That is a weighted vector sum rather than a difference of angles, it is
    // measured over exactly the neighbourhood the kernel gathers from, and it
    // falls for both of the reasons that warrant a narrow kernel: a bending
    // tread, and two legs crossing.
    // -----------------------------------------------------------------------
    const kapA = new Float32Array(n);
    const KAP_R2 = 1.8 * 1.8;
    const KAP_K = 1 / (2 * 0.9 * 0.9);
    for (let i = 0; i < n; i++) {
      const st = stamps[i];
      const bx = Math.floor((st.x - minX) / BIN);
      const bz = Math.floor((st.z - minZ) / BIN);
      const tx = tanX[i], tz = tanZ[i];
      const h0 = st.targetHeight;
      let saa = 0, sav = 0;
      for (let bjo = -1; bjo <= 1; bjo++) {
        for (let bio = -1; bio <= 1; bio++) {
          const list = bins.get((bz + bjo) * 4096 + (bx + bio));
          if (!list) continue;
          for (let m = 0; m < list.length; m++) {
            const jj = list[m];
            if (jj === i) continue;
            const o = stamps[jj];
            const dx = o.x - st.x, dz = o.z - st.z;
            const d2 = dx * dx + dz * dz;
            if (d2 > KAP_R2 || d2 < 1e-8) continue;
            const dh = o.targetHeight - h0;
            const dhAbs = dh < 0 ? -dh : dh;
            if (dhAbs > 0.8 * Math.sqrt(d2) + 0.3) continue;
            const al = dx * tx + dz * tz;
            if ((al < 0 ? -al : al) < 0.05) continue;   // no along-track leverage
            const w = Math.exp(-d2 * KAP_K);
            saa += w * al * al;
            sav += w * al * (slope[jj] - slope[i]);
          }
        }
      }
      if (saa > 1e-6 && isFinite(sav / saa)) {
        const ka = sav / saa;
        kapA[i] = ka < 0 ? -ka : ka;
      } else {
        // A stamp with no along-track partners tells us nothing about curvature.
        // Report it as fully curved: that keeps the NARROW kernel, which is the
        // conservative answer everywhere. An unknown is never a licence to
        // filter harder.
        kapA[i] = 1;
      }
    }

    return { tanX, tanZ, arc, grade, spacing, slope, cross, latOff, halfW, kapA };
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

  /**
   * CSR bin index over the stamp list, so sampleDesign() does not have to scan
   * tens of thousands of stamps per query.
   */
  function buildDesignIndex(list, prep) {
    const cells = DBN * DBN;
    const start = new Int32Array(cells + 1);
    const cellOf = new Int32Array(list.length);
    let maxReach = DB_CELL;
    for (let s = 0; s < list.length; s++) {
      const st = list[s];
      let i = Math.floor((st.x - minX) / DB_CELL);
      let j = Math.floor((st.z - minZ) / DB_CELL);
      if (i < 0) i = 0; else if (i > DBN - 1) i = DBN - 1;
      if (j < 0) j = 0; else if (j > DBN - 1) j = DBN - 1;
      const c = j * DBN + i;
      cellOf[s] = c;
      start[c + 1]++;
      const r = Math.max(0.6, st.radius || 2);
      const reach = Math.max(r * RW_MAX, stampReach(r, prep.spacing[s]) * UA_FULL);
      if (reach > maxReach) maxReach = reach;
    }
    for (let c = 0; c < cells; c++) start[c + 1] += start[c];
    const item = new Int32Array(list.length);
    const cursor = new Int32Array(cells);
    for (let s = 0; s < list.length; s++) {
      const c = cellOf[s];
      item[start[c] + cursor[c]] = s;
      cursor[c]++;
    }
    designBinStart = start;
    designBinItem = item;
    designBinRing = Math.max(1, Math.ceil(maxReach / DB_CELL));
    designStamps = list;
    designPrep = prep;
  }

  // Stable result object so sampleDesign() allocates nothing when reused.
  const _designOut = {
    ok: false, height: 0, plane: 0, micro: 0, lateral: 0, treadFrac: 0, along: 0,
    width: 0, stamp: -1, kind: null, bank: 0, material: -1, spread: 0,
  };

  /**
   * The design surface the trail emitted at (x, z) — the reference the
   * committed heightfield is measured against.
   *
   * Returns `out.ok === false` for any point that is not inside some stamp's
   * tread (|lat| <= the jittered half-width, |along| within one along-track
   * reach). Where it is true, `out.height` is the design tread height there:
   * the emitting stamp's plane, its kind's section, and the ridden microrelief
   * — produced by exactly the same functions the accumulator commits, so a
   * residual measured against it is a measurement of the ACCUMULATOR and not of
   * a re-implementation of the design.
   *
   * Nearest-station selection: on a self-consistent trail every stamp covering
   * a cell evaluates to the same height there, so which one answers matters
   * only to the extent that the trail disagrees with itself — which is a useful
   * thing for a residual to expose rather than to hide.
   */
  function sampleDesign(x, z, out) {
    const o = out || _designOut;
    o.ok = false; o.height = 0; o.plane = 0; o.micro = 0;
    o.lateral = 0; o.treadFrac = 0; o.along = 0;
    o.width = 0; o.stamp = -1; o.kind = null; o.bank = 0; o.material = -1;
    o.spread = 0;
    if (!designStamps || designStamps.length === 0 || !designBinStart) return o;

    const bi = Math.floor((x - minX) / DB_CELL);
    const bj = Math.floor((z - minZ) / DB_CELL);
    const R = designBinRing;
    let best = -1, bestKey = Infinity;
    let bestLat = 0, bestAlong = 0, bestRw = 0, bestT = 0;
    let spreadLo = Infinity, spreadHi = -Infinity;

    for (let oj = -R; oj <= R; oj++) {
      const j = bj + oj;
      if (j < 0 || j >= DBN) continue;
      for (let oi = -R; oi <= R; oi++) {
        const i = bi + oi;
        if (i < 0 || i >= DBN) continue;
        const c = j * DBN + i;
        for (let q = designBinStart[c]; q < designBinStart[c + 1]; q++) {
          const s = designBinItem[q];
          const st = designStamps[s];
          const tx = designPrep.tanX[s], tz = designPrep.tanZ[s];
          const dx = x - st.x, dz = z - st.z;
          const lat = dx * -tz + dz * tx;
          const along = dx * tx + dz * tz;
          const r = Math.max(0.6, st.radius || 2);
          const rl = stampReach(r, designPrep.spacing[s]);
          const uaAbs = (along < 0 ? -along : along) / rl;
          if (uaAbs > UA_FULL) continue;
          const latAbs = lat < 0 ? -lat : lat;
          const rw = jitterRadius(r, latAbs, x, z);
          const t = latAbs / rw;
          if (t > 1) continue;
          // Spread of the design heights the COVERING stamps declare here. On a
          // self-consistent trail this is a few centimetres. Where a switchback
          // folds back on itself and two legs claim the same ground at
          // different elevations it is metres, and no accumulator can satisfy
          // both — see the `spread` field on the result.
          const py = stampTarget(st, lat, rw, along, designPrep.slope[s], designPrep.cross[s]);
          if (py < spreadLo) spreadLo = py;
          if (py > spreadHi) spreadHi = py;
          // Selected on the SAME normalised radius the accumulator's Shepard
          // weight uses, so "the design here" means the section declared by the
          // sample that actually governs here. Selecting on |along| instead —
          // which is the obvious choice and was the first one tried — picks the
          // centre stamp for a cell that the accumulator is correctly letting a
          // lateral stamp govern, and then reports the difference as residual.
          const key = t * t + uaAbs * uaAbs;
          if (key < bestKey) {
            bestKey = key; best = s;
            bestLat = lat; bestAlong = along; bestRw = rw; bestT = t;
          }
        }
      }
    }
    if (best < 0) return o;

    const st = designStamps[best];
    const r = Math.max(0.6, st.radius || 2);
    const rl = stampReach(r, designPrep.spacing[best]);
    const uaAbs = (bestAlong < 0 ? -bestAlong : bestAlong) / rl;
    const ag = alongGate(uaAbs);
    const isTread = st.kind === 'tread' || st.kind === 'rut' || st.kind === undefined;
    const brake = isTread ? smoothstep(0.11, 0.30, designPrep.grade[best]) : 0;
    const halfW = designPrep.halfW[best];
    const latCL = bestLat + designPrep.latOff[best];
    const tc = (latCL < 0 ? -latCL : latCL) / halfW;
    const inner = (1 - smoothstep(0.72, 1.0, tc)) * ag;
    const sArc = designPrep.arc[best] + bestAlong;

    // No batter term: inside the tread `over` is negative and batter() is the
    // identity there by construction.
    const plane = stampTarget(st, bestLat, bestRw, bestAlong,
      designPrep.slope[best], designPrep.cross[best]);
    const micro = treadMicro(st, isTread, latCL, halfW, sArc, inner, brake, x, z, ag);

    o.ok = true;
    o.height = plane + micro;
    // The ridden microrelief on its own, so a QA pass can subtract it and check
    // the committed tread against the RAW emitted stamp heights — a test that
    // shares no code with this module's idea of what the design surface is.
    o.micro = micro;
    o.plane = plane;
    o.lateral = latCL;          // from the ribbon centreline, not the stamp axis
    o.along = bestAlong;
    o.treadFrac = tc;           // |lateral| / ribbon half-width
    o.width = halfW;
    o.stamp = best;
    o.kind = st.kind || 'tread';
    o.bank = st.bank || 0;
    o.material = st.material === undefined || st.material === null ? -1 : st.material;
    // How badly the emitted stamps covering this point disagree with each other.
    o.spread = spreadHi - spreadLo;
    return o;
  }

  // -------------------------------------------------------------------------
  // commitDetail — band-limit the committed tread to what a wheel can follow
  //
  // WHY THIS EXISTS. The accumulator above is deliberately an INTERPOLANT: the
  // Shepard kernel is singular at the sample, so the committed surface passes
  // exactly through every emitted stamp height. That is the right answer when
  // the point cloud is a sampling of one smooth ribbon. It is the wrong answer
  // for the cloud trail.js actually emits, because that cloud contains features
  // finer than the thing that has to roll over them:
  //
  //   * rut stamps  — radius 0.34 m, targetHeight 0.012-0.252 m BELOW the tread,
  //                   emitted on alternate stations, i.e. every 0.8 m;
  //   * rock-garden boulder stamps — radius 0.22-0.56 m, 0.17-0.42 m PROUD,
  //                   emitted on every fifth station of a rock section.
  //
  // Interpolated exactly, those become a 0.8 m corrugation down the middle of
  // the tread. MEASURED on the committed centreline at arc 405-520 m, seed
  // 20260726: +-0.10 to 0.25 m of height swing at 0.4-0.8 m wavelength, at
  // lateral offsets -0.25..+0.25 m and nowhere else across the section.
  //
  // WHY IT MATTERS, in numbers rather than adjectives. A wheel leaves the ground
  // when v^2 * kappa >= g. At the p50 design speed of 12.9 m/s that is
  // kappa = 0.059 /m — a 17 m crest radius. A sinusoid of wavelength L reaches
  // that curvature at an amplitude of only kappa*L^2/(4*pi^2):
  //     L = 1 m -> 1.5 mm      L = 2 m -> 6 mm      L = 4 m -> 24 mm
  // So sub-metre content of a few millimetres is a launcher, and no amount of
  // r.m.s. agreement with the design protects against it. That is exactly why
  // the previous round closed the 2-6 m band and the failure simply moved down.
  //
  // WHAT WAS TRIED AND REJECTED (measured, seed 20260726, wheel-test % of
  // stations where v^2*kappa/g >= 1 at the design speed; the design centreline
  // itself scores 4.25%):
  //     baseline                                             43.64%
  //     along-track mean support floor (metres, not radii)    39.58%
  //     ... plus no tread microrelief at all                  24.29%
  //     detail lattice halved to 0.175 m                      45.06%   (so it
  //         is NOT lattice aliasing: a finer lattice is worse, not better)
  //     Shepard singularity softened / kernel unsquared       36-40%
  //     no stamp may assert finer than a wheel (support floor
  //         in metres on BOTH axes)                           44.29%   (worse:
  //         a widened rut stamp drags a wider trough)
  //     low-pass of the committed carve DELTA                 35.97%   (wrong:
  //         the carve cancels the natural ground exactly only when the delta is
  //         applied unfiltered; filtering the delta re-injects the mountain's
  //         own sub-metre roughness under the tread)
  //     low-pass of the accumulated TARGET SURFACE, 0.55 m     4.07%
  //
  // So the fix is the last of those: reconstruct the target as before, then
  // low-pass the target SURFACE (not the delta, not the stamps) at the scale a
  // 0.37 m wheel integrates over, and commit that. Everything the design asks
  // for above ~2 m survives; everything below it, which the design does not ask
  // for and a wheel cannot follow, does not.
  //
  // COST OF THE SYMMETRIC KERNEL, and why it is no longer the default.
  //
  // A world-axis-separable kernel low-passes ACROSS the track exactly as hard as
  // it does along it. Along the track that is the whole point — a 0.37 m wheel
  // cannot follow anything shorter, so nothing a rider can feel is lost. Across
  // the track it is pure damage: the tread is only 2.4-3.0 m wide, so a sigma
  // 0.555 m kernel sampled at the +-0.36 x width secant reaches 1.7 m out, past
  // the tread edge, and mixes the batter and the untouched hillside into the
  // berm wall. A wheel has no reason to pay for that.
  //
  // THE KERNEL IS THEREFORE ANISOTROPIC, IN THE TRACK FRAME: sigma_along
  // unchanged at 1.5 wheel radii, sigma_cross at one detail cell. The track
  // bearing is not assumed — it is accumulated per detail cell as a
  // doubled-angle structure tensor (the dDirA/dDirB/dDirW triple in applyCarve),
  // so two legs of a switchback running opposite ways REINFORCE (same line, same
  // filter) and two crossing at right angles CANCEL, and a cell whose bearing
  // cancels falls back to the isotropic kernel. An unknown bearing is a reason
  // to be conservative, not a reason to guess.
  //
  // MEASURED, round 7. A/B in one process against one cached world, pre-carve
  // fingerprint identical across every variant; seeds 20260726 / 777 / 12345;
  // `sigmaCross: 0.555` reproduces the isotropic kernel to three decimals on
  // every metric below, which is what says the gather is a faithful
  // generalisation rather than a different filter.
  //
  //                              committed cross-slope / tan(bank), p50
  //     sigma_cross   0.555 (iso)   0.868 / 0.864 / 0.873
  //                   0.44          0.897 / 0.898 / 0.903
  //                   0.35  SHIPPED 0.921 / 0.922 / 0.926
  //                   0.26          0.942 / 0.943 / 0.946
  //                   0.175         0.956 / 0.957 / 0.961
  //     no filter at all            0.966 / 0.970 / 0.972
  //     the DESIGN surface itself   0.962 / 0.949 / 0.953   <- the real ceiling
  //
  // and on the steepest berms alone (|tan bank| > 0.70), where it binds:
  //     0.555 (iso)  0.87 / 0.60 / 0.74      0.35  0.92 / 0.78 / 0.86
  //     0.26         0.94 / 0.85 / 0.91      0.175 0.95 / 0.92 / 0.95
  //
  // RE-CONFIRMED IN ROUND 8 UNDER THE ADAPTIVE KERNEL, seed 20260726, one
  // process, one route, everything else identical — so the cross decision does
  // not have to be taken on trust from a superseded configuration:
  //     sigma_cross 0.35  SHIPPED  x-slope p50 0.909, steepest berms 0.926,
  //                                residual p90 0.0131
  //     sigma_cross 0.555 (iso)    x-slope p50 0.868, steepest berms 0.879,
  //                                residual p90 0.0117
  // 4.1 points of bank overall and 4.7 on the steepest berms, for 0.0014 /m of
  // residual p90. Same trade, same answer.
  //
  // WHY 0.35 AND NOT 0.175, which retains more. Because below one lattice cell
  // the discrete kernel stops being a low-pass: cross-track content survives at
  // and above the 0.35 m lattice's Nyquist (0.7 m), and the bicubic
  // reconstruction in detailField() turns it into ripple ALONG any line that is
  // not exactly lattice-aligned. Wheel test, worst 25 m bin at the 0.05 m
  // stencil (the sub-metre-sensitive one), same three seeds:
  //     0.555 (iso)  11.6 / 10.0 / 15.8      0.35  11.4 / 10.0 / 11.4
  //     0.26         17.6 / 17.8 / 21.2      0.175 34.4 / 38.4 / 37.6
  // and 0.3-0.6 m band r.m.s. 7.5e-5 / 9.6e-5 / 7.8e-5 for iso, 9.7e-5 / 1.1e-4
  // / 9.7e-5 at 0.35, 3.6e-4 / 3.4e-4 / 4.0e-4 at 0.175. Two points of bank is
  // 0.6 deg of lean on a 30 deg berm; the 0.175 setting buys it with a 4.5x
  // increase in exactly the content the last two rounds died on. It is not a
  // trade worth making, and the fact that it looks free on band r.m.s. above
  // 2 m is the round-6 lesson restated.
  //
  // WHAT IT BUYS THE RIDER, which is the only reason any of this matters. At the
  // binding corner of each seed — the tightest banked corner at its governed
  // speed — the lean the rider must still carry once the COMMITTED bank has done
  // its share, against a 45 deg ceiling:
  //     20260726 arc 297  r 7.9  v 9.7   31.3 -> 30.3 deg   (margin 13.7 -> 14.7)
  //     777      arc 1867 r 9.3  v 11.8  32.8 -> 26.3 deg   (margin 12.2 -> 18.7)
  //     12345    arc 1843 r 9.5  v 11.8  35.7 -> 28.9 deg   (margin  9.3 -> 16.1)
  // i.e. 6.6 and 6.8 more degrees of committed bank at the two corners that were
  // closest to washing out. That is the lowside failure mode, not the launch one.
  //
  // AN INVARIANT WORTH KEEPING: swapping the kernel changes `detail.heights` and
  // NOTHING ELSE — `detail.material`, `detail.weight` and `detail.carve` hash
  // identically under both. The filter touches the height field only; paint,
  // corridor blend and the vegetation-exclusion mask are untouched by
  // construction.
  //
  // Two things measured as making NO difference and were therefore removed
  // rather than kept as knobs: weighting the bearing tensor by the Shepard mean
  // weight instead of the blend strength (identical to three decimals on all
  // three seeds), and pre-smoothing the tensor at sigma 1.05 m (identical, and
  // 2.4x the carve time).
  //
  // CAVEAT ON THE ABSOLUTE FIGURES. trail.js was being rewritten in parallel
  // while these were taken and changed four times during the session, so every
  // absolute number above belongs to one trail revision. The A/B was re-run on
  // each and the iso-vs-track deltas reproduced every time; the cross-slope
  // figures quoted are from trail.js df2b1b8ec7f9, and 8107cc05a0ef gives
  // 0.870/0.865/0.873 -> 0.926/0.922/0.928 for the same swap.
  //
  // AND IT HAPPENED AGAIN IN ROUND 8, WORSE, SO HERE IS THE RULE. Two consecutive
  // boots of the same seed 20 s apart produced 2718.68 m / 58 800 stamps and
  // 2718.04 m / 58 860 stamps: trail.js was being saved between them. The route
  // is therefore NOT a fixed quantity across processes and no absolute figure in
  // this file may be compared against one taken in another process. What IS
  // fixed, and was verified byte-identical across five concurrent processes at
  // load average 4.3 on all three seeds, is everything terrain owns — the base
  // heightfield and every field derived from it (see fingerprint()). Every
  // measurement quoted in this file is therefore an A/B inside ONE process
  // against ONE cached route and ONE cached base, with the pre-carve heights
  // hash asserted equal across every variant of the comparison. That is the only
  // form of measurement that survives a module being rewritten underneath it.
  const WHEEL_R = 0.37;             // contact patch the profile is integrated over
  // 1.5 wheel radii. Measured on the wheel test, seed 20260726 / 777 / 12345:
  //   1.0 R (4 passes)  4.73 / 5.93 / 6.34 %      1.5 R (5 passes) 4.07 / 5.29 / 5.48 %
  // The 4-pass setting misses the 6% acceptance on seed 12345, so the margin is
  // bought here rather than hoped for.
  const TREAD_LP_SIGMA = WHEEL_R * 1.5;
  // Across the track: exactly one detail cell — see the Nyquist argument above.
  const TREAD_LP_SIGMA_CROSS = DSTEP;

  // -------------------------------------------------------------------------
  // THE ALONG-TRACK SIGMA IS ADAPTIVE, AND THIS IS THE ROUND-8 CHANGE
  //
  // THE DEFECT. At stations where the emitted design profile is FLAT — |kappa|
  // below 0.012 /m at the 0.37 m acceptance stencil, with no feature authored —
  // the committed surface still carried convex curvature the design never asked
  // for. Measured across seeds 20260726 / 777 / 12345, one process and one
  // cached route each (plan path = centripetal Catmull-Rom through the emitted
  // station plan points arc-resampled at 0.05 m, wheel-centre locus = dilation
  // by a 0.37 m disc, kappa by second difference at half-stencil 0.37 m,
  // reference = the emitted station centreline S.py):
  //     shipped kernel  p90 0.0198 / 0.0227 / 0.0198   p99 0.0583 / 0.0581 / 0.0535
  // That p99 is very nearly the whole 0.059 /m a wheel can hold at the 12.9 m/s
  // p50 design speed, arriving on ground the trail asked to be flat.
  //
  // WHY A FIXED SIGMA CANNOT FIX IT. The kernel's job is stated as a length —
  // "1.5 wheel radii" — but the constraint it is being held to is a CURVATURE.
  // A residual sinusoid of amplitude A and wavelength L passes the Gaussian at
  // T(L) = exp(-2 pi^2 sigma^2 / L^2) and arrives carrying A*T(L)*(2 pi / L)^2
  // of curvature. Maximise over L: the worst case is at L = pi*sigma*sqrt(2)
  // (2.47 m at the shipped sigma) and its size is 2A/(e*sigma^2). The gain the
  // rider feels therefore falls as 1/sigma^2 — which is why widening works.
  //
  // THE WHOLE LADDER, ONE PROCESS, ONE CACHED ROUTE, ONE CACHED BASE (seed
  // 20260726; pre-carve heights hash identical across every row):
  //
  //   sigma_along  residual at flat stations      wheel test      shaped feature
  //                p90     p95     max            course  worst   p2p mean / min  2-6 m
  //     0.555 r7   0.0226  0.0335  0.074           0.61%  13.4%   0.932 / 0.808   0.728
  //     0.75       0.0161  0.0220  0.053           0.53%  13.2%   0.906 / 0.739   0.607
  //     1.00       0.0137  0.0169  0.067           0.24%   9.4%   0.870 / 0.642   0.489
  //     1.30       0.0129  0.0164  0.145           0.10%   3.4%   0.825 / 0.527   0.406
  //     1.40       0.0129  0.0168  0.206           0.09%   3.4%   0.811 / 0.490   0.390
  //     2.50       0.0177  0.0250  1.251           0.39%  13.0%   0.674 / 0.187   0.393
  //     4.00       0.0313  0.0480  2.543           1.03%  26.8%   0.559 / 0.042   0.581
  //     ADAPTIVE   0.0133  0.0168  0.066           0.58%  13.4%   0.924 / 0.808   0.716
  //
  // Two things in that table decide the design.
  //
  // First, the fixed-sigma family turns back on itself: past ~1.4 m the residual
  // rises again and its MAXIMUM explodes (0.06 -> 2.1 -> 3.4 /m), because a
  // kernel wider than the corner it sits on stops filtering along the tread and
  // starts smearing across it (see PLAN CURVATURE below). There is no fixed
  // sigma that is safe and also strong.
  //
  // Second, and decisively: WIDENING FLATTENS WHAT THE TRAIL AUTHORED. A jump lip
  // at 49% of its emitted amplitude (sigma 1.4) or 21% (sigma 2.5) is the
  // "flattened into a fire road" failure this file exists to avoid. A global
  // widen trades one acceptance criterion for another and is not a fix.
  //
  // The adaptive row reaches the fixed family's best residual — p90 0.0133 and
  // p95 0.0168, against 0.0129 / 0.0164 at sigma 1.30 — while holding the
  // shaped-feature minimum at the r7 kernel's own figure to the last digit
  // (0.808 against 0.808), where sigma 1.30 loses 35% of it. It also has the
  // lowest MAXIMUM residual of any row in the table (0.066 against 0.145 at
  // sigma 1.30 and 1.25 at 2.50), because the coherence cap withdraws exactly
  // where a fixed wide kernel does its damage. That is the entire claim of this
  // block.
  //
  // The one thing it does NOT do is move the wheel test: 0.61% -> 0.58% course,
  // worst 25 m bin 13.4% -> 13.4%. See the CONTRACT-NOTE on top speed — 1.1% of
  // the launching samples on this course are at flat-design stations, so that
  // gate is not measuring terrain and no amount of this work will move it.
  //
  // THE FIX. Widen the kernel ONLY where the design asked for nothing, and there
  // the widening costs nothing BY CONSTRUCTION — there is no authored relief at
  // a flat station to lose. sigma_along is selected per detail cell from the
  // design's own curvature, carried by the stamps (prepareStamps' kapA):
  //
  //   kapA <= KAPPA_DESIGN_FLAT     -> sigma_along = TREAD_LP_SIGMA_FLAT
  //   kapA >= KAPPA_DESIGN_FEATURE  -> sigma_along = TREAD_LP_SIGMA (unchanged)
  //   between                        -> smoothstep, quantised to LP_NLEV levels
  //
  // TREAD_LP_SIGMA_FLAT = 1.10 is the KNEE of that curve, not its minimum, and
  // the knee is the right choice because past it the residual is flat and the
  // only thing a wider kernel can still do is damage. Sweeping it alone, same
  // process and route as the ladder above:
  //     flat sigma  residual p90 / p95 / p99    feature mean / min   x-slope p50 / steep
  //       0.555 r7    0.0226 / 0.0335 / 0.0558    0.932 / 0.808       0.917 / 0.926
  //       0.90        0.0143 / 0.0184 / 0.0336    0.927 / 0.808       0.914 / 0.926
  //       1.10 SHIP   0.0133 / 0.0168 / 0.0300    0.924 / 0.808       0.912 / 0.926
  //       1.30        0.0131 / 0.0170 / 0.0314    0.920 / 0.808       0.909 / 0.926
  //       1.60        0.0136 / 0.0182 / 0.0336    0.916 / 0.789       0.904 / 0.926
  //       2.20        0.0173 / 0.0249 / 0.0445    0.910 / 0.736       0.894 / 0.926
  // The shaped-feature MINIMUM — the jump lip that would be flattened — is
  // bit-for-bit the r7 figure up to 1.30 and starts falling at 1.60, and the
  // committed cross-slope on the steepest berms does not move at all across the
  // whole sweep (0.926, every row), because the widening runs ALONG the berm.
  // 1.10 and 1.30 measure the same; 1.10 is taken because filtering less for the
  // same answer is the error this file would rather make.
  //
  // KAPPA_DESIGN_FLAT is the acceptance criterion's own flatness threshold, so
  // the widened kernel is applied exactly on the set the residual is measured
  // over and nowhere else. KAPPA_DESIGN_FEATURE is g / v^2 at the p50 design
  // speed: at or above it the trail is authoring launch-scale relief, which is
  // its decision to make and not terrain's to filter away.
  //
  // PLAN CURVATURE IS A HARD CAP ON THE WIDENING, and it is what stops the
  // non-monotone blow-up above. The kernel's "along the track" is one straight
  // bearing per cell, so on a corner of plan radius r a tap half-span L departs
  // the arc by L^2/(2r) at the ends; past a fraction of the tread half-width
  // that stops being a low-pass along the tread and becomes a smear across it,
  // which is exactly what a berm cannot afford.
  //
  // The cap is taken from the COHERENCE of the doubled-angle direction field
  // this filter already accumulates, C = |sum w e^(2 i theta)| / sum w, and NOT
  // from a per-stamp plan-curvature estimate. The estimate was tried and
  // measured first: regressing d(theta)/d(arc) from neighbouring stamps' PCA
  // tangents pinned 32% of committed cells to the base kernel on a route whose
  // 1st-percentile plan radius is 16 m — it was reading its own angle noise.
  // Coherence is a weighted vector sum instead of a difference of two noisy
  // angles, it is evaluated over the very neighbourhood the kernel gathers
  // from, and it falls for both of the reasons that warrant a narrow kernel:
  // the tread is bending, or two legs are crossing.
  //
  // And it converts to a departure in closed form, with no threshold to invent.
  // C = <cos 2 delta> = 1 - 2<delta^2> + O(delta^4), so the r.m.s. angle between
  // the bearing the kernel assumes and the bearing the tread actually has is
  // delta = sqrt((1 - C)/2). A kernel of half-span L applied at that error walks
  // L*delta off the tread, so requiring the 2-sigma span to stay inside PLAN_TOL
  // gives, exactly,
  //         sigma <= PLAN_TOL / sqrt(2 * (1 - C)).
  // MEASURED coherence over the committed cells of seed 20260726, one process,
  // one cached route: p10 0.914, p50 0.992. That is 1.98 m of allowance at the
  // median (no cap at all, the design gate binds first) and 0.60 m at the tenth
  // percentile (straight back to the base kernel), which is the behaviour asked
  // for: unrestricted on the open trail, withdrawn on the switchback stacks and
  // the tightest berms.
  //
  // CONSERVATISM, AND WHERE IT IS AND IS NOT TAKEN. The design curvature is
  // accumulated per cell as a BROAD weighted mean of the stamps reaching it —
  // weighted by the height-blend STRENGTH, which is ~1 for every stamp across
  // the whole bench, so it genuinely averages the neighbourhood. Two other
  // estimators were tried and both are worse. Three-way A/B, ONE process, ONE
  // cached route, ONE cached base, everything else identical:
  //
  //     accumulation of kapA per cell   residual p90 / p95 / p99 / max    v95
  //       MAX over the stamps           0.0167 / 0.0287 / 0.0520 / 0.075  18.5
  //       SHEPARD-weighted mean         0.0139 / 0.0186 / 0.0457 / 0.145  22.9
  //       STRENGTH-weighted mean SHIP   0.0131 / 0.0170 / 0.0314 / 0.072  24.0
  //
  // The MAX is the obvious "conservative" choice and it is the worst of the
  // three, which is worth understanding rather than just noting: ~30 stamps
  // reach a typical tread cell, each carrying a regression's worth of noise, so
  // a max over them is an order statistic on the noise tail. It does not produce
  // a narrower kernel — its median sigma is the HIGHEST of the three, 1.108
  // against 0.992 — it produces an INCOHERENT one, flipping between wide and
  // narrow cell to cell as the noise tail moves, and a spatially incoherent
  // kernel width is itself a source of curvature.
  //
  // The SHEPARD weight fails for a different reason: that kernel is singular at
  // the sample and hands the nearest stamp ~85% of the total, so a
  // "Shepard-weighted mean" of a per-stamp quantity is very nearly the nearest
  // stamp's own value and averages almost nothing. Its p99 and max give it away
  // (0.046 and 0.145 against 0.031 and 0.072).
  //
  // Conservatism is taken SPATIALLY instead, which is where it belongs: the
  // per-cell sigma field is eroded (running minimum) by LP_SIG_ERODE cells on
  // each axis, so no cell inside a feature's own reconstruction footprint can
  // inherit a flat neighbour's wide kernel. A mean averages the noise down; the
  // erosion still guarantees the only error it can make is filtering too little.
  // An unknown — an incoherent bearing, a stamp with no along-track partners —
  // is always resolved toward the shipped kernel, never away from it.
  //
  // WHAT IS DELIBERATELY UNCHANGED: sigma_cross. It stays at one detail cell at
  // every level, so the committed cross-slope — and therefore the bank a berm
  // realises, which is the dominant crash cause on two seeds — is bit-for-bit
  // the shipped answer wherever the design has curvature and within measurement
  // noise of it everywhere else. The anisotropy question and the residual
  // question are settled independently, on purpose.
  // -------------------------------------------------------------------------
  const TREAD_LP_SIGMA_FLAT = 1.10;      // sigma_along on ground the design left flat
  const KAPPA_DESIGN_FLAT = 0.012;       // 1/m — the acceptance criterion's own threshold
  const KAPPA_DESIGN_FEATURE = 0.060;    // 1/m — g / (12.9 m/s)^2, the launch scale
  const PLAN_TOL = 0.25;                 // m the kernel may depart the tread by
  const LP_NLEV = 12;                    // sigma_along quantisation levels
  // Detail cells of min-erosion on the sigma field. MEASURED across 0/2/4/6 on
  // seed 20260726, one process, one route: the shaped-feature minimum moves
  // 0.783 -> 0.785 and the residual p95 0.0174 -> 0.0185, i.e. the erosion is
  // very nearly free in both directions on this route. It is kept at a 1.4 m
  // guard band — the same order as the widest kernel's one-sigma — as insurance
  // for a route where a feature DOES sit against flat ground, which this route
  // cannot prove never happens, and because its only possible error is
  // filtering too little.
  const LP_SIG_ERODE = 4;
  // A [1,2,1]/4 pass has variance DSTEP^2/2, so N passes give sigma = DSTEP*sqrt(N/2).
  // Only the 'iso' mode uses this; it is kept so the shipped round-5/6 surface can
  // still be reproduced exactly for an A/B.
  const TREAD_LP_PASSES = Math.max(0, Math.round(2 * (TREAD_LP_SIGMA / DSTEP) * (TREAD_LP_SIGMA / DSTEP)));

  // ---- filter selection ----------------------------------------------------
  // `settings.treadFilter` is a QA/diagnostic override with the same contract as
  // `settings.erosionDroplets`: explicit, logged, identical on every boot of the
  // same settings, and never consulted by anything else. It exists so a harness
  // can A/B two kernels against ONE cached world in ONE process, which is the
  // only way any of the numbers above can be trusted.
  //   { mode: 'track' | 'iso' | 'none', sigmaAlong, sigmaCross, sigmaAlongFlat,
  //     kappaFlat, kappaFeature, planTol, erodeCells }
  const TREAD_FILTER_DEFAULT = Object.freeze({
    mode: 'track',
    sigmaAlong: TREAD_LP_SIGMA,
    sigmaCross: TREAD_LP_SIGMA_CROSS,
    // `sigmaAlongFlat === sigmaAlong` disables the adaptive widening entirely
    // and reproduces the round-7 surface exactly, which is what makes the A/B
    // above runnable from this file alone.
    sigmaAlongFlat: TREAD_LP_SIGMA_FLAT,
    kappaFlat: KAPPA_DESIGN_FLAT,
    kappaFeature: KAPPA_DESIGN_FEATURE,
    planTol: PLAN_TOL,
    erodeCells: LP_SIG_ERODE,
  });
  const treadFilter = (() => {
    const o = settings.treadFilter;
    if (!o || typeof o !== 'object') return TREAD_FILTER_DEFAULT;
    const mode = (o.mode === 'iso' || o.mode === 'none' || o.mode === 'track')
      ? o.mode : TREAD_FILTER_DEFAULT.mode;
    const num = (v, d, min) => (typeof v === 'number' && isFinite(v) && v >= min ? v : d);
    const sa = num(o.sigmaAlong, TREAD_FILTER_DEFAULT.sigmaAlong, 1e-6);
    // Floored at half a lattice cell: below that the sampled Gaussian is
    // narrower than the lattice it lives on and is a comb, not a filter.
    const sc = Math.max(DSTEP * 0.5, num(o.sigmaCross, TREAD_FILTER_DEFAULT.sigmaCross, 0));
    // The flat-ground sigma can never be NARROWER than the base one: the ladder
    // runs from `sigmaAlong` upward, and inverting it would filter features
    // harder than flat ground, which is the opposite of the whole argument.
    const saf = Math.max(sa, num(o.sigmaAlongFlat, TREAD_FILTER_DEFAULT.sigmaAlongFlat, 1e-6));
    const kf = num(o.kappaFlat, TREAD_FILTER_DEFAULT.kappaFlat, 0);
    const kx = Math.max(kf + 1e-6, num(o.kappaFeature, TREAD_FILTER_DEFAULT.kappaFeature, 0));
    const pt = num(o.planTol, TREAD_FILTER_DEFAULT.planTol, 1e-4);
    const ec = Math.round(num(o.erodeCells, TREAD_FILTER_DEFAULT.erodeCells, 0));
    const spec = Object.freeze({
      mode, sigmaAlong: sa, sigmaCross: sc, sigmaAlongFlat: saf,
      kappaFlat: kf, kappaFeature: kx, planTol: pt, erodeCells: ec,
    });
    console.info('[terrain] tread filter overridden by settings.treadFilter:', spec,
      '(default', TREAD_FILTER_DEFAULT, ')');
    return spec;
  })();

  // Direction bins for the anisotropic kernel. The kernel is symmetric under
  // t -> -t, so the ring covers [0, pi) only. 64 bins is 2.8 deg of quantisation,
  // i.e. at most 0.1% of misalignment in the projected sigma — far below the
  // 5% the direction field itself is good to.
  const LP_NDIR = 64;
  let lpTaps = null;

  /**
   * Precompute the tap list of the anisotropic kernel for every direction bin at
   * every sigma_along LEVEL, plus one isotropic set per level used wherever the
   * track direction is unknown.
   *
   * Bin layout is `level * (LP_NDIR + 1) + dir`, with `dir === LP_NDIR` the
   * isotropic fallback. THE FALLBACK IS AT THE BASE SIGMA AT EVERY LEVEL: an
   * unknown bearing is a reason to filter conservatively, and a widened
   * ISOTROPIC kernel would low-pass across the track as hard as along it, which
   * is precisely the cross-slope damage the anisotropic kernel exists to avoid.
   *
   * Level 0 is the base sigma and reproduces the previous single-kernel tables
   * exactly, so `sigmaAlongFlat === sigmaAlong` is a bit-identical no-op.
   *
   * Taps are stored CSR-style (start[] into offI[]/offJ[]/wt[]) and weights are
   * normalised per bin, so a gather is a plain weighted mean with no per-cell
   * transcendentals. Nothing here reads a clock or an rng: the tables are a pure
   * function of the sigma ladder.
   */
  function buildLpTaps(sa, saFlat, sc) {
    const nLev = saFlat > sa * (1 + 1e-9) ? LP_NLEV : 1;
    const perLev = LP_NDIR + 1;
    const nBins = nLev * perLev;
    const start = new Int32Array(nBins + 1);
    const oi = [], oj = [], wv = [];
    let Rmax = 1;
    for (let L = 0; L < nLev; L++) {
      const saL = nLev === 1 ? sa : sa + ((saFlat - sa) * L) / (nLev - 1);
      // Each level is sized on its own sigma, so the wide levels do not make the
      // narrow ones pay for taps that weigh nothing.
      const R = Math.max(1, Math.ceil((3 * Math.max(saL, sc)) / DSTEP));
      if (R > Rmax) Rmax = R;
      const iaL = 1 / (2 * saL * saL);
      const iaBase = 1 / (2 * sa * sa);
      const ic = 1 / (2 * sc * sc);
      for (let d = 0; d < perLev; d++) {
        const bin = L * perLev + d;
        start[bin] = wv.length;
        const iso = d === LP_NDIR;
        const th = ((d + 0.5) * Math.PI) / LP_NDIR;
        const tx = Math.cos(th), tz = Math.sin(th);
        // The isotropic fallback is built at the BASE sigma and therefore needs
        // only the base radius.
        const RR = iso ? Math.max(1, Math.ceil((3 * Math.max(sa, sc)) / DSTEP)) : R;
        let sum = 0;
        const first = wv.length;
        for (let dj = -RR; dj <= RR; dj++) {
          for (let di = -RR; di <= RR; di++) {
            const px = di * DSTEP, pz = dj * DSTEP;
            let w;
            if (iso) {
              w = Math.exp(-(px * px + pz * pz) * iaBase);
            } else {
              const a = px * tx + pz * tz;
              const c = -px * tz + pz * tx;
              w = Math.exp(-a * a * iaL - c * c * ic);
            }
            if (w < 1e-4) continue;
            oi.push(di); oj.push(dj); wv.push(w); sum += w;
          }
        }
        for (let t = first; t < wv.length; t++) wv[t] /= sum;
      }
    }
    start[nBins] = wv.length;
    lpTaps = {
      R: Rmax, nLev, perLev, nBins, start,
      // Offsets can exceed +-127 only if a sigma above ~14 m were configured;
      // Int16 removes the question at the cost of one byte per tap.
      offI: Int16Array.from(oi),
      offJ: Int16Array.from(oj),
      wt: Float32Array.from(wv),
    };
    return lpTaps;
  }

  /**
   * The sigma_along a stamp permits from the DESIGN'S OWN VERTICAL CURVATURE, in
   * metres. The plan-curvature cap is applied separately, per cell, from the
   * direction field's coherence — see the ADAPTIVE block and smoothTrackFrame().
   */
  function sigmaAllowed(kapA) {
    const f = treadFilter;
    let t = (kapA - f.kappaFlat) / (f.kappaFeature - f.kappaFlat);
    if (t < 0) t = 0; else if (t > 1) t = 1;
    const ease = t * t * (3 - 2 * t);            // 0 at flat, 1 at feature scale
    const s = f.sigmaAlongFlat + (f.sigmaAlong - f.sigmaAlongFlat) * ease;
    return s < f.sigmaAlong ? f.sigmaAlong : s;
  }

  // Coherence below which a cell's accumulated bearing is not trusted and the
  // isotropic kernel is used instead. |sum w e^(2 i theta)| / sum w is 1 for a
  // single consistent tangent and 0 for two legs crossing at right angles.
  const LP_DIR_COHERENCE = 0.5;

  /** Detail-lattice cell index for a GLOBAL lattice coordinate, or -1. */
  function detailCellAt(gi, gj) {
    const ti = Math.floor(gi / DT_INNER);
    const tj = Math.floor(gj / DT_INNER);
    if (ti < 0 || tj < 0 || ti >= DTILES || tj >= DTILES) return -1;
    const slot = tileIndex[tj * DTILES + ti];
    if (slot < 0) return -1;
    const li = gi - (ti * DT_INNER - 1);
    const lj = gj - (tj * DT_INNER - 1);
    if (li < 0 || lj < 0 || li >= DTS || lj >= DTS) return -1;
    return slot * DTS * DTS + lj * DTS + li;
  }

  /** Slot -> tileIndex key, so a filter can walk tiles in slot order. */
  function slotKeys() {
    const k = new Int32Array(nTiles);
    for (let key = 0; key < tileIndex.length; key++) {
      const s = tileIndex[key];
      if (s >= 0) k[s] = key;
    }
    return k;
  }

  /**
   * Separable [1,2,1]/4 over the detail lattice, `passes` per WORLD axis.
   * Reads through the global lattice addressing so a sample two tiles share is
   * computed from the same neighbourhood in both and the copies stay
   * bit-identical — a one-ulp disagreement there would show as a seam in the
   * normals. Returns whichever buffer holds the result.
   *
   * This is the shipped round-5/6 height kernel — isotropic, and therefore as
   * hard across the track as along it. It is retained only for `mode: 'iso'`, so
   * that the A/B in the note above can be reproduced from this file alone.
   */
  function smoothSeparable(srcIn, dCells, passes) {
    let src = srcIn;
    if (passes <= 0 || nTiles === 0) return src;
    const keys = slotKeys();
    let dst = new Float32Array(dCells);
    for (let pass = 0; pass < passes * 2; pass++) {
      const alongX = (pass & 1) === 0;
      for (let s = 0; s < nTiles; s++) {
        const key = keys[s];
        const ti = key % DTILES;
        const tj = (key / DTILES) | 0;
        const gi0 = ti * DT_INNER - 1;
        const gj0 = tj * DT_INNER - 1;
        const base = s * DTS * DTS;
        for (let j = 0; j < DTS; j++) {
          for (let i = 0; i < DTS; i++) {
            const p = base + j * DTS + i;
            const c = src[p];
            const pa = alongX ? detailCellAt(gi0 + i - 1, gj0 + j)
              : detailCellAt(gi0 + i, gj0 + j - 1);
            const pb = alongX ? detailCellAt(gi0 + i + 1, gj0 + j)
              : detailCellAt(gi0 + i, gj0 + j + 1);
            // A missing neighbour reflects the centre, so the filter is
            // mass-preserving at the edge of the allocated corridor instead
            // of pulling the surface toward zero there.
            dst[p] = ((pa < 0 ? c : src[pa]) + 2 * c + (pb < 0 ? c : src[pb])) * 0.25;
          }
        }
      }
      const t = src; src = dst; dst = t;
    }
    return src;
  }

  /**
   * The anisotropic kernel, evaluated in the TRACK FRAME of each cell.
   *
   * One gather per cell with precomputed, per-direction-bin, pre-normalised
   * weights — not a cascade of resampled passes. That matters: every resampling
   * of an off-axis 1-D pass drags a bilinear footprint across the track, so a
   * cascade's effective cross-track sigma depends on the trail's bearing
   * relative to the lattice. A single gather with lattice-aligned taps has
   * exactly the cross-track sigma it says it has, at every bearing.
   *
   * Cells no stamp reached are not filtered at all — they are never committed,
   * and skipping them is most of the cost. They are still READ as taps, which is
   * correct: they hold the natural ground the tread has to daylight into.
   *
   * Reads go through the same global lattice addressing the isotropic path uses,
   * so a sample two tiles share is computed from the same neighbourhood in both
   * and the copies stay bit-identical.
   */
  /**
   * Erode the per-cell sigma_along field by `cells` samples on each axis, so a
   * cell can never be filtered wider than the narrowest requirement in its own
   * neighbourhood. A separable running minimum — the morphological dual of the
   * [1,2,1] smoother above, and the conservative direction by construction: the
   * only error it can make is filtering too little.
   *
   * Reads through the same global lattice addressing as everything else, so the
   * duplicated border samples two tiles share stay bit-identical.
   */
  function erodeSigma(sigIn, dCells, cells) {
    if (cells <= 0 || nTiles === 0) return sigIn;
    let src = sigIn;
    let dst = new Float32Array(dCells);
    const keys = slotKeys();
    for (let pass = 0; pass < 2; pass++) {
      const alongX = pass === 0;
      for (let s = 0; s < nTiles; s++) {
        const key = keys[s];
        const ti = key % DTILES;
        const tj = (key / DTILES) | 0;
        const gi0 = ti * DT_INNER - 1;
        const gj0 = tj * DT_INNER - 1;
        const base = s * DTS * DTS;
        for (let j = 0; j < DTS; j++) {
          for (let i = 0; i < DTS; i++) {
            let m = src[base + j * DTS + i];
            for (let k = -cells; k <= cells; k++) {
              if (k === 0) continue;
              const q = alongX ? detailCellAt(gi0 + i + k, gj0 + j)
                : detailCellAt(gi0 + i, gj0 + j + k);
              // A missing neighbour is outside the allocated corridor and holds
              // no tread; it must not pull the sigma down.
              if (q < 0) continue;
              if (src[q] < m) m = src[q];
            }
            dst[base + j * DTS + i] = m;
          }
        }
      }
      const t = src; src = dst; dst = t;
    }
    return src;
  }

  function smoothTrackFrame(src, dCells, accW, dirA, dirB, dirW, sigA) {
    if (nTiles === 0) return src;
    const taps = lpTaps
      || buildLpTaps(treadFilter.sigmaAlong, treadFilter.sigmaAlongFlat, treadFilter.sigmaCross);
    const start = taps.start, offI = taps.offI, offJ = taps.offJ, wt = taps.wt;
    const perLev = taps.perLev, nLev = taps.nLev;
    const isoBin = LP_NDIR;
    // Level lookup: 0 at the base sigma, nLev-1 at the flat-ground sigma.
    const sa0 = treadFilter.sigmaAlong;
    const levScale = nLev > 1 ? (nLev - 1) / (treadFilter.sigmaAlongFlat - sa0) : 0;
    const dst = new Float32Array(dCells);
    const keys = slotKeys();
    const binScale = LP_NDIR / Math.PI;
    // Coherence -> plan-curvature cap on the widening, in closed form: the
    // kernel may not walk more than planTol off the tread at its 2-sigma span,
    // and the r.m.s. bearing error is sqrt((1 - coherence)/2). See the ADAPTIVE
    // block for the derivation and the measured coherence distribution.
    const planTol = treadFilter.planTol;
    // Instrumentation: what the adaptive band limit ACTUALLY did, per committed
    // cell, after both the design-curvature gate and the coherence cap. Not a
    // knob — "the widening never reached the flat ground" is otherwise invisible
    // from outside this file, and it is the failure mode a per-stamp plan
    // curvature estimate already produced once.
    const NB = 64;
    const hist = new Int32Array(NB);
    const cHist = new Int32Array(NB);
    let hN = 0, hSum = 0;
    const hLo = sa0, hSpan = treadFilter.sigmaAlongFlat > sa0
      ? treadFilter.sigmaAlongFlat - sa0 : 1;
    for (let s = 0; s < nTiles; s++) {
      const key = keys[s];
      const ti = key % DTILES;
      const tj = (key / DTILES) | 0;
      const gi0 = ti * DT_INNER - 1;
      const gj0 = tj * DT_INNER - 1;
      const base = s * DTS * DTS;
      for (let j = 0; j < DTS; j++) {
        for (let i = 0; i < DTS; i++) {
          const p = base + j * DTS + i;
          // Cells no stamp reached are never committed; skipping them is most of
          // the cost. They are still read as taps, from `src`, which is correct.
          if (accW[p] === 0) { dst[p] = src[p]; continue; }
          const wc = dirW[p];

          // Bearing of the tread here, from the doubled-angle structure tensor.
          let bin = isoBin;
          const a = dirA[p], b = dirB[p];
          const mag = Math.sqrt(a * a + b * b);
          const coher = wc > 0 ? mag / wc : 0;
          // Deliberately the same multiply-form test as before rather than
          // `coher > LP_DIR_COHERENCE`: the two are algebraically identical but
          // not bit-identical, and this branch decides the bearing of the
          // shipped surface.
          if (wc > 0 && mag > wc * LP_DIR_COHERENCE) {
            let th = Math.atan2(b, a) * 0.5;      // (-pi/2, pi/2]
            if (th < 0) th += Math.PI;
            bin = (th * binScale) | 0;
            if (bin >= LP_NDIR) bin = LP_NDIR - 1;
            else if (bin < 0) bin = 0;
          }

          // sigma_along level from the design's own curvature, capped by the
          // coherence of the bearing this kernel is about to assume (see the
          // ADAPTIVE block). The isotropic fallback bin stays at the base sigma
          // at every level, so an unknown bearing is never widened.
          let sigUsed = sa0;
          if (nLev > 1 && bin !== isoBin) {
            const d2 = 2 * (1 - coher);
            const cap = d2 > 1e-9 ? planTol / Math.sqrt(d2) : Infinity;
            sigUsed = sigA[p] < cap ? sigA[p] : cap;
            if (sigUsed < sa0) sigUsed = sa0;
            let lev = Math.round((sigUsed - sa0) * levScale);
            if (lev < 0) lev = 0; else if (lev > nLev - 1) lev = nLev - 1;
            bin += lev * perLev;
          }
          hN++; hSum += sigUsed;
          let hb = Math.floor(((sigUsed - hLo) / hSpan) * NB);
          if (hb < 0) hb = 0; else if (hb > NB - 1) hb = NB - 1;
          hist[hb]++;
          let cb = Math.floor(coher * NB);
          if (cb < 0) cb = 0; else if (cb > NB - 1) cb = NB - 1;
          cHist[cb]++;

          const t0 = start[bin], t1 = start[bin + 1];
          let acc = 0, wsum = 0;
          for (let t = t0; t < t1; t++) {
            const li = i + offI[t], lj = j + offJ[t];
            let q;
            if (li >= 0 && li < DTS && lj >= 0 && lj < DTS) {
              // Fast path: the tap is inside this tile. Same cell, same value
              // and same address the global path would have produced.
              q = base + lj * DTS + li;
            } else {
              q = detailCellAt(gi0 + li, gj0 + lj);
              if (q < 0) continue;              // outside the allocated corridor
            }
            const w = wt[t];
            acc += src[q] * w;
            wsum += w;
          }
          // Renormalising by the weight actually used keeps the filter DC-exact
          // at the edge of the corridor instead of pulling the surface toward
          // zero there.
          dst[p] = wsum > 0 ? acc / wsum : src[p];
        }
      }
    }
    const quant = (h, f, lo, span, nb) => {
      const want = f * hN;
      let acc = 0;
      for (let b = 0; b < nb; b++) { acc += h[b]; if (acc >= want) return lo + ((b + 0.5) / nb) * span; }
      return lo + span;
    };
    timings.lpSigma = hN === 0 ? null : {
      cells: hN,
      mean: hSum / hN,
      p10: quant(hist, 0.10, hLo, hSpan, NB),
      p50: quant(hist, 0.50, hLo, hSpan, NB),
      p90: quant(hist, 0.90, hLo, hSpan, NB),
      baseFrac: hist[0] / hN,
      wideFrac: hist[NB - 1] / hN,
      coherP10: quant(cHist, 0.10, 0, 1, NB),
      coherP50: quant(cHist, 0.50, 0, 1, NB),
    };
    return dst;
  }

  /**
   * Build the target surface from the accumulators, band-limit it, and commit it.
   *
   * Buffers are Float32 offsets from BASE_Y rather than absolute elevations:
   * the world spans ~700 m of relief, so the worst resolution is 4e-5 m against
   * an error budget of 0.10 m. Absolute 1700 m elevations in Float32 would be
   * good to 2e-4 m, which is still fine, but the datum costs nothing.
   */
  function commitDetail(accW, accWT, accMax, dCells, dirA, dirB, dirW, kapA) {
    let src = new Float32Array(dCells);
    for (let p = 0; p < dCells; p++) {
      const w = accW[p];
      src[p] = (w === 0 ? dHeights[p] : dHeights[p] + accWT[p] / w) - BASE_Y;
    }
    // Cleared here rather than left from a previous applyCarve on the same
    // instance: only the 'track' path produces it, and a stale one would be
    // read as a live measurement.
    timings.lpSigma = null;
    if (treadFilter.mode === 'iso') src = smoothSeparable(src, dCells, TREAD_LP_PASSES);
    else if (treadFilter.mode === 'track') {
      // Design curvature -> permitted sigma_along, in place (kapA is scratch and
      // is not read again), then eroded so the widening cannot creep onto a
      // feature from the flat ground beside it.
      //
      // A cell no stamp reached carries the FLAT sigma, not the base one. It is
      // never committed, so its own value is irrelevant; the only thing it does
      // is act as an input to the erosion, and there is no authored curvature
      // out there to protect. Giving it the base sigma instead makes the
      // erosion pull every tread cell within LP_SIG_ERODE of the corridor's
      // edge down to the base kernel for no reason at all.
      for (let p = 0; p < dCells; p++) {
        const w = dirW[p];
        kapA[p] = w > 0 ? sigmaAllowed(kapA[p] / w) : treadFilter.sigmaAlongFlat;
      }
      const sig = erodeSigma(kapA, dCells, treadFilter.erodeCells);
      src = smoothTrackFrame(src, dCells, accW, dirA, dirB, dirW, sig);
    }
    for (let p = 0; p < dCells; p++) {
      if (accW[p] === 0) continue;
      dHeights[p] += (src[p] + BASE_Y - dHeights[p]) * accMax[p];
    }
  }

  async function applyCarve(stamps) {
    if (!baseBuilt) await buildBase();
    const list = Array.isArray(stamps) ? stamps.filter(
      (s) => s && isFinite(s.x) && isFinite(s.z) && isFinite(s.targetHeight),
    ) : [];

    const t0 = Date.now();
    tileIndex.fill(-1);
    nTiles = 0;
    detailReady = false;
    designStamps = null; designPrep = null; designBinStart = null; designBinItem = null;
    corridorDF = buildCorridorField(list);
    if (list.length === 0) { detailReady = false; return; }

    const prep = prepareStamps(list);
    buildDesignIndex(list, prep);
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
      if ((s & 255) === 0) {
        report(0.55 * (s / list.length));
        const y = maybeYield(); if (y) await y;
      }
      const st = list[s];
      const r = Math.max(0.6, st.radius || 2);
      const rl = stampReach(r, prep.spacing[s]);
      const gA = prep.slope[s];
      const gL = prep.cross[s];
      const latOff = prep.latOff[s];
      const halfW = prep.halfW[s];
      // Only plain benched tread gets the batter. Berms, wallrides, lips,
      // landings and drops are *constructed* features whose flanks are authored
      // by trail.js, and re-cutting them to an angle of repose would flatten the
      // very geometry they exist to make.
      const batterKind = st.kind === undefined || st.kind === 'tread' || st.kind === 'rut';
      // The footprint must cover the whole region stampCell() can accept, which
      // is now a rotated RECTANGLE (cross-track and along-track are independent)
      // rather than the old ellipse — so it is the diagonal, not the larger
      // side. Costs ~4% more cells on a battered tread stamp; clipping a corner
      // instead would put a step in the surface at the corner.
      const reachLat = r * RW_MAX * CS_OUT + (batterKind ? BATTER_RUN : 0);
      const reach = Math.sqrt(reachLat * reachLat + (rl * UA_MAX) * (rl * UA_MAX));
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
          if (!stampCell(st, lat, along, r, rl, x, z, batterKind, heights[k], gA, gL,
            latOff, halfW)) continue;
          const w = _scW;
          // Accumulated as an OFFSET from the natural ground, never as an
          // absolute elevation: with Shepard weights up to MEAN_CAP, summing
          // 1700 m elevations would be good to only ~0.1 m in Float32, which is
          // the whole error budget. `heights[k]` is still the natural surface —
          // the accumulator is applied after every stamp has been visited.
          accW[k] += _scWM;
          accWT[k] += _scWM * (_scTarget - heights[k]);
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
      heights[k] += (accWT[k] / wsum) * accMax[k];
    }
    const t2 = Date.now();

    // ---- detail overlay ---------------------------------------------------
    const dCells = nTiles * DTS * DTS;
    const dAccW = new Float32Array(dCells);
    const dAccWT = new Float32Array(dCells);
    const dAccMax = new Float32Array(dCells);
    // Track bearing per detail cell, as a DOUBLED-ANGLE structure tensor
    // (sum w*cos2th, sum w*sin2th) using the same Shepard weights as the height
    // mean. Doubled angle because the kernel is symmetric under t -> -t: the two
    // legs of a switchback that reach the same cell running opposite ways must
    // REINFORCE (same line, same filter), while two legs crossing at right
    // angles must CANCEL, so that the cell is filtered isotropically rather than
    // smeared along a bearing that is only one of the two answers. A plain
    // vector sum gets both of those cases backwards.
    //
    // The weight is the stamp's height-blend STRENGTH, which is ~1 across the
    // whole bench for every stamp that reaches the cell, so the bearing is the
    // average bearing of the neighbourhood rather than that of whichever stamp
    // happens to be nearest. (The Shepard mean weight was tried and measures
    // identically to three decimals on all three seeds — the doubled-angle sum
    // is robust either way. Strength is kept because it is the cheaper claim to
    // defend.) dDirW is the matching weight total: it is NOT accW, which sums
    // Shepard weights, and the coherence test needs the denominator that goes
    // with the numerator.
    const dDirA = new Float32Array(dCells);
    const dDirB = new Float32Array(dCells);
    const dDirW = new Float32Array(dCells);
    // The design's own vertical curvature per detail cell, accumulated with the
    // height-blend STRENGTH as its weight — the same summand as dDirW, which is
    // therefore its denominator. A broad mean, deliberately not the Shepard one:
    // see the CONSERVATISM paragraph in the ADAPTIVE block. Converted to a
    // sigma_along and eroded in commitDetail.
    const dKapA = new Float32Array(dCells);

    for (let s = 0; s < list.length; s++) {
      if ((s & 255) === 0) {
        report(0.55 + 0.45 * (s / list.length));
        const y = maybeYield(); if (y) await y;
      }
      const st = list[s];
      const r = Math.max(0.6, st.radius || 2);
      const rl = stampReach(r, prep.spacing[s]);
      const gA = prep.slope[s];
      const gL = prep.cross[s];
      const latOff = prep.latOff[s];
      const halfW = prep.halfW[s];
      const tx = prep.tanX[s], tz = prep.tanZ[s];
      const rx = -tz, rz = tx;
      // Constant over this stamp's footprint, so it is hoisted out of the cell
      // loop exactly like the doubled-angle bearing below.
      const kapAs = prep.kapA[s];
      // Doubled-angle form of this stamp's bearing; constant over its footprint.
      const dir2a = tx * tx - tz * tz, dir2b = 2 * tx * tz;
      const arc0 = prep.arc[s];
      const grade = prep.grade[s];
      const isTread = st.kind === 'tread' || st.kind === 'rut' || st.kind === undefined;
      const brake = isTread ? smoothstep(0.11, 0.30, grade) : 0;
      // Only plain benched tread is battered — see the global loop above.
      const batterKind = isTread;
      const reachLat = r * RW_MAX * CS_OUT + (batterKind ? BATTER_RUN : 0);
      const reach = Math.sqrt(reachLat * reachLat + (rl * UA_MAX) * (rl * UA_MAX));
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
              if (!stampCell(st, lat, along, r, rl, x, z, batterKind, dHeights[p], gA, gL,
                latOff, halfW)) continue;
              const w = _scW;
              const sArc = arc0 + along;

              const cw = (w * 255) | 0;
              if (cw > dCarve[p]) dCarve[p] = cw;

              // The ridden microrelief. Identical code to the one sampleDesign()
              // reports, so "committed vs design" measures the accumulator and
              // nothing else.
              const target = _scTarget
                + treadMicro(st, isTread, _scLatCL, halfW, sArc, _scInner, brake, x, z, _scAg);

              dAccW[p] += _scWM;
              dAccWT[p] += _scWM * (target - dHeights[p]);
              dDirA[p] += _scWH * dir2a;
              dDirB[p] += _scWH * dir2b;
              dDirW[p] += _scWH;
              if (_scWH > dAccMax[p]) dAccMax[p] = _scWH;
              dKapA[p] += _scWH * kapAs;

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

    commitDetail(dAccW, dAccWT, dAccMax, dCells, dDirA, dDirB, dDirW, dKapA);

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

  async function commit() {
    if (!baseBuilt) await buildBase();
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
        report((j * rootN + i + 1) / (rootN * rootN) * 0.9);
        const y = maybeYield(); if (y) await y;
      }
    }

    // The world does not stop at the map boundary any more.
    report(0.9);
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

  // -------------------------------------------------------------------------
  // fingerprint — determinism, asserted rather than assumed
  //
  // CONTRACT §0 promises the world is a pure function of ctx.seed. It was not:
  // the erosion pass sized its droplet budget from a wall-clock projection and
  // degraded silently under load, so the same seed produced different mountains
  // on different boots and every measurement taken this session was taken
  // against a moving target. That is fixed at the source (see DROPLET_TIER);
  // this is the check that stops it coming back.
  //
  // FNV-1a over the raw bytes of every generated field. Hashing the Float32
  // bit patterns rather than rounded values is deliberate: a tolerance would
  // hide exactly the kind of small, load-dependent drift that caused the
  // problem. Byte order is the host's, so these hashes compare boots on one
  // machine — which is the claim being made.
  // -------------------------------------------------------------------------

  function fnv1a(h, view) {
    if (!view) return Math.imul(h ^ 0xff, 0x01000193) >>> 0;
    const u8 = new Uint8Array(view.buffer, view.byteOffset, view.byteLength);
    let a = h >>> 0;
    for (let i = 0; i < u8.length; i++) {
      a = Math.imul(a ^ u8[i], 0x01000193) >>> 0;
    }
    return a >>> 0;
  }
  const hex8 = (v) => (v >>> 0).toString(16).padStart(8, '0');

  /**
   * Deterministic hash of everything this module generates. Cheap enough to
   * call from a QA harness or a debug key; not called on any hot path.
   */
  function fingerprint() {
    const H0 = 0x811c9dc5;
    const detail = detailReady && dHeights
      ? {
        tiles: nTiles,
        heights: hex8(fnv1a(H0, dHeights)),
        material: hex8(fnv1a(H0, dMaterial)),
        weight: hex8(fnv1a(H0, dWeight)),
        carve: hex8(fnv1a(H0, dCarve)),
      }
      : { tiles: 0, heights: null, material: null, weight: null, carve: null };
    let all = H0;
    all = fnv1a(all, heights);
    all = fnv1a(all, materials);
    all = fnv1a(all, splatU8);
    all = fnv1a(all, extraU8);
    all = fnv1a(all, hardnessU8);
    all = fnv1a(all, talusU8);
    all = fnv1a(all, aoU8);
    all = fnv1a(all, carveU8);
    if (detailReady && dHeights) {
      all = fnv1a(all, dHeights);
      all = fnv1a(all, dMaterial);
      all = fnv1a(all, dWeight);
      all = fnv1a(all, dCarve);
    }
    return {
      seed: ctx.seed,
      quality: ctx.quality,
      resolution: RES,
      droplets: timings.droplets || 0,
      dropletTier,
      treadFilter,
      world: hex8(all),
      heights: hex8(fnv1a(H0, heights)),
      materials: hex8(fnv1a(H0, materials)),
      splat: hex8(fnv1a(H0, splatU8)),
      extra: hex8(fnv1a(H0, extraU8)),
      hardness: hex8(fnv1a(H0, hardnessU8)),
      talus: hex8(fnv1a(H0, talusU8)),
      ao: hex8(fnv1a(H0, aoU8)),
      carve: hex8(fnv1a(H0, carveU8)),
      detail,
      minY: bounds.minY,
      maxY: bounds.maxY,
      creekLevel,
    };
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

    // --- march support: directional queries, allocation-free (see §MARCH) ---
    sampleGrade,
    sampleCrossSlope,
    sampleCurvatureAlong,
    probeCorridor,
    probeReach: PROBE_REACH,
    probeBand: PROBE_BAND,
    probeFallAt: PROBE_FALL,
    // The trail's emitted design surface at a point (see sampleDesign()). QA
    // measures the committed heightfield against this; nothing on a hot path
    // reads it.
    sampleDesign,
    // The tread band-limit kernel actually in force (see commitDetail()). Read
    // only; changing it means changing `settings.treadFilter` before build.
    treadFilter,
    // Determinism check (see fingerprint()).
    fingerprint,
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
