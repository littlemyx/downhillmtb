// bikeModel.js — procedural downhill mountain bike. CONTRACT §6, ADDENDUM §D.
//
// =============================================================================
// ARCHITECTURE
// =============================================================================
// Everything is generated in code: no meshes, no textures, no models loaded. The
// bike is built once at construction into ~19 merged meshes and thereafter only
// transforms (and three morph-target influences) change per frame.
//
// The single trick that makes "≤ 20 draw calls with a dozen distinct PBR finishes"
// possible is a per-vertex material attribute: every geometry carries `color`
// (albedo, linear) and `aPbr` (roughness, metalness) attributes, and the shared
// MeshStandardMaterial is patched in onBeforeCompile to read roughness/metalness
// from `aPbr`. So anodised alu, raw alu, chrome stanchions, black rubber, steel
// rotors and matte carbon can all live in ONE merged mesh with correct, distinct
// finishes. Only two materials exist: MAT_HARD (metal micro-scratch normal) and
// MAT_CARBON (twill weave normal) — the difference is the normal map, because
// that is the only thing a vertex attribute cannot carry.
//
// Geometry is real, not implied:
//   * tubes are swept along Catmull-Rom curves with per-station elliptical
//     sections, so the downtube is round at the head tube and a wide flat oval at
//     the BB — the hydroformed look. No uniform cylinders anywhere structural.
//   * the rear end is a genuine four-bar Horst-link, solved every frame by
//     circle-circle intersection. The rear axle position is dictated by the
//     physics (bike.js publishes shock travel); the swingarm rotates to follow it,
//     the Horst pivot rides the swingarm, the seatstay and rocker are solved from
//     it, and the shock eye-to-eye falls out of the rocker angle. Measured at
//     build time: 251 mm eye-to-eye at full extension, 64 mm of stroke for 165 mm
//     of wheel travel — a 2.6:1 leverage ratio, which is a real DH number.
//   * 32 spokes per wheel in true 3-cross lacing (67.5° hub-to-rim offset, which
//     is n·720/N for n=3, N=32), alternating leading/trailing, both flanges.
//   * tyre knobs are displaced geometry — they break the silhouette, which a
//     normal map cannot do.
//   * rotors are cut through: THREE.Shape with real holes, extruded.
//   * the chain is solved as a belt over four circles (chainring, cassette, both
//     derailleur pulleys) using external-tangent geometry, then instanced link by
//     link along the arc-length parameterisation.
//
// =============================================================================
// CONTRACT-NOTE: `bike.geometry.bbUp` is published as 0.0 but its own comment says
//   "bottom bracket ≈ axle height − 25 mm". The comment is right and the value is
//   a placeholder, so this module uses −0.028 m (BB 342 mm above the ground with a
//   370 mm wheel, which is a correct DH bottom-bracket height). Everything else in
//   `bike.geometry` is used verbatim.
// CONTRACT-NOTE: the contract asks for a dropper post. A DH bike does not have one
//   (fixed post, saddle slammed) and animating it would cost a draw call out of a
//   budget that is already at 19/20, so the post is modelled dropper-style but is
//   static. Say the word and it becomes an object with a 60 mm range.
// CONTRACT-NOTE: derailleur pulleys do not spin (they are merged into the
//   derailleur body to stay inside the draw-call budget). The chain, chainring and
//   cassette all move correctly, which is what reads at speed.
// CONTRACT-NOTE: all textures are DataTextures generated from a seeded PRNG, so
//   this module builds head-less (no `document`) and is deterministic from
//   ctx.seed, per CONTRACT §0.
// CONTRACT-NOTE: `bike.geometry` is read once, at construction. If bike.js ever
//   re-tunes FORK_RAKE / SHOCK_LEN / WHEELBASE at runtime, this model will not
//   follow — call createBikeModel again, or tell me and I will make it re-derive.
// CONTRACT-NOTE: the rider's IK targets are the anchors listed in ADDENDUM §D and
//   nothing else. gripL/gripR are at the OUTER END of each grip (x = ∓0.376 in
//   chassis-local, 1.05 m above the ground at sag); pedalL/pedalR orbit the BB on
//   a 165 mm crank and are level with the chassis, not with the world.
//
// =============================================================================
// ROUND-3 REVISION (LANE F) — what changed and why
// =============================================================================
// F1 VALUE HIERARCHY. Measured on r3_12 the fork stanchions peaked at L=232 and
//   the fork lowers at L=163 against a sunlit trail whose p95 was L=66 — the two
//   least important parts of the bike were the brightest objects in the frame.
//   Root cause was not just the authored numbers: `roughnessFactor` arrives from
//   the roughness MAP (mean ≈ 0.72) and was then multiplied by the authored
//   roughness, so every finish was shading ~0.72x rougher^-1 than its own table
//   said — a "0.045" stanchion was really 0.032, a mirror. The map is now a
//   modulation centred on 1.0 (see patchBikeMaterial), so the numbers in F below
//   are the ACTUAL shading roughness. Lowers and rims are re-authored per the
//   work order; stanchions went with them because the measurement says they, not
//   the lowers, were the worst offender.
// F2 IDENTITY. The frame was `metalness 1.0`, and a metal has no diffuse term —
//   that is precisely why it read as "flat red with one broad terminator". Frame
//   paint is now a dielectric (metalness 0) with a per-vertex CLEARCOAT layer, so
//   it has a real N·L terminator under a tight lacquer highlight. On top of that
//   there is a full decal path: a procedurally generated 4-cell decal atlas, a
//   per-vertex decal-rect attribute, and a shader that composites it into albedo
//   and clearcoat. Weld beads, gussets and a raised head badge are real geometry.
//   AUTHORED-ART HOOK: replace `buildDecalAtlas()` with a loaded 4-cell RGBA atlas
//   (linear, straight alpha, cell order: downtube / toptube / head badge / stay)
//   and everything downstream keeps working unchanged.
// F3 The brake hoses now terminate on the caliper banjos, and both master
//   cylinders now grow a hose that hands over ON THE STEER AXIS (x=0, z=0 in the
//   steer frame) so the join is gap-free at every steering angle. Lever blades
//   were present but authored 6.4 mm fore/aft x 21 mm tall — a fin standing on
//   edge, in bright ALU_MACH, invisible against the bar. They are now real
//   paddles with a reach curve, a dimpled tip and a dark blade.
// F4 Sub-pixel geometry (spokes, chain plates) carries its own half-width in
//   metres in `aDecal.z`, and the vertex shader expands it in VIEW space until it
//   covers ~1.5 px. That is also the spoke LOD: at 15 m a 1.05 mm spoke expands
//   to ~12 mm and 32 of them merge into a disc, which is what the alpha-textured
//   disc in the work order would have produced — without adding an alpha test.
// F6 castShadow is now on the 8 meshes that own the silhouette, not all 19, and
//   the chain drops out past 18 m. Contact grounding is bought with a build-time
//   vertex-AO bake (occupancy grid + hemisphere march, zero runtime cost) plus a
//   two-instance contact-shadow blob under the wheels — NOT by re-raising N8AO
//   (round-3 rejection R10) and not by touching the shadow rig (Lane E owns it).
// =============================================================================

import * as THREE from 'three';
import { makeRng, subSeed, clamp, clamp01, lerp, damp } from '../core/rng.js';

// =============================================================================
// TUNING / GEOMETRY TABLE
// All lengths in metres, in CHASSIS-LOCAL space:
//   +X = rider's right, +Y = up, +Z = BACKWARD (forward is −Z).
//   Origin = the axle midpoint at axle height (bike.js `chassisOrigin`), so the
//   ground sits at y = −0.37 and every number below can be read against a real
//   bike spec sheet.
// =============================================================================
const G = {
  wheelbase: 1.25,
  wheelR: 0.370,

  // --- fork (mirrors bike.js T.FORK_*; overridden from bike.geometry if present)
  forkRake: 27 * Math.PI / 180,   // 63° head angle
  forkLen: 0.50,
  forkTravel: 0.170,
  forkSag: 0.30,
  forkOffset: 0.048,              // steerer sits this far BEHIND the leg axis
  crownLower: 0.136,              // lower crown, along the steer axis above the mount
  crownUpper: 0.281,
  barAxis: 0.316,                 // bar centre along the steer axis
  legX: 0.066,                    // half-spacing of the stanchions
  stanchionR: 0.0200,             // 40 mm stanchions
  lowerTop: 0.420,                // top of the fork lowers, above the axle
  lowerFlare: 0.014,              // lowers splay outboard toward the dropouts, which
                                  // is the only thing that lets the rotor and a 2.5"
                                  // tyre both pass between them
  stanchionLen: 0.410,

  // --- shock / rear end (mirrors bike.js T.SHOCK_*)
  shockRake: -8 * Math.PI / 180,
  shockLen: 0.42,
  shockTravel: 0.165,
  shockSag: 0.30,

  // --- four-bar. See the design log: solved and non-singular over the whole
  //     travel range, monotonic shock stroke, 2.6:1 leverage.
  mainPivot: { y: 0.045, z: 0.055 },
  horstBack: -0.075,              // Horst pivot, forward of the axle
  horstDown: -0.055,              // ...and below it
  rockerPivot: { y: 0.400, z: 0.140 },
  seatstayTop: { y: 0.420, z: 0.330 },   // rocker↔seatstay pivot at rest
  shockEye: { y: 0.3431, z: 0.0431 },    // rocker↔shock pivot at rest
  shockMount: { y: 0.120, z: 0.100 },    // frame↔shock pivot (fixed)

  // --- front triangle
  bb: { y: -0.028, z: 0.055 },
  seatTubeTop: { y: 0.400, z: 0.140 },
  saddle: { y: 0.470, z: 0.150 },
  saddleLen: 0.252,

  // --- cockpit
  barWidth: 0.800,
  barRise: 0.028,
  barSweep: 0.055,
  gripInner: 0.242,
  gripOuter: 0.376,

  // --- drivetrain
  chainline: 0.049,
  chainringTeeth: 34,
  chainPitch: 0.0127,
  crankLen: 0.165,
  crankQ: 0.0855,                 // half the Q-factor (171 mm)
  cogTeeth: [11, 13, 15, 17, 20, 24, 28],
  cogX: [0.0500, 0.0562, 0.0624, 0.0686, 0.0748, 0.0810, 0.0872],
  driveCog: 4,                    // index of the cog the chain sits on (20t)
  pulleyTeeth: 11,

  // --- brakes
  rotorR: 0.100,                  // 200 mm floating disc
  rotorX: 0.0455,                 // ISO rotor plane; rotors live on the LEFT (−X)
};

// Teeth → pitch radius for a roller chain: r = p / (2·sin(π/N)).
const pitchRadius = (teeth) => G.chainPitch / (2 * Math.sin(Math.PI / teeth));

// =============================================================================
// FINISHES — { c: sRGB hex, r: roughness, m: metalness, cc: clearcoat 0..1 }
// Authored to be ACES tone-mapped (ADDENDUM §C): no pre-darkening.
//
// `r` IS THE SHADING ROUGHNESS. Before round 3 it was multiplied by the roughness
// map (mean 0.72) and every finish shaded ~30 % smoother than it claimed; the map
// is now a ±13 % modulation centred on 1.0 (patchBikeMaterial), so these numbers
// mean what they say. If you re-tune one, re-tune it against the measurement that
// matters: NOTHING on the bike may out-luminate sunlit trail tread. On r3_12 the
// sunlit tread p95 was L=66; the stanchion peaked at 232 and the lowers at 163.
//
// `cc` > 0 selects the clearcoat lobe, which is what makes paint read as paint.
// A painted tube is a DIELECTRIC (m = 0) — the old m = 1.0 removed the diffuse
// term entirely, and that missing terminator is the "red plastic straw" finding.
// =============================================================================
const F = {
  // --- painted frame surfaces: dielectric base + lacquer ---------------------
  PAINT:      { c: 0x8f2d15, r: 0.52, m: 0.00, cc: 0.90 },  // brand red
  PAINT_DEEP: { c: 0x5b1c0d, r: 0.56, m: 0.00, cc: 0.85 },  // shadowed panels, gussets
  PAINT_BLK:  { c: 0x1b1e22, r: 0.55, m: 0.00, cc: 0.78 },  // second colourway
  // --- anodised / machined metal --------------------------------------------
  ANO:        { c: 0x86331a, r: 0.36, m: 1.00 },  // colour-anodised machined alu
  ANO_DEEP:   { c: 0x4e1f0e, r: 0.42, m: 1.00 },
  ANO_GOLD:   { c: 0x7a5f28, r: 0.36, m: 1.00 },  // small gold accents only
  LOWER:      { c: 0x575046, r: 0.40, m: 1.00 },  // fork lower castings. Was
                                                  // ANO_GOLD 0xa8802f/0.28: HSV
                                                  // sat 0.72 val 0.66 → 0.29/0.35,
                                                  // i.e. desaturated off brass and
                                                  // dropped a stop and a half.
  ALU_RAW:    { c: 0x8b9198, r: 0.46, m: 1.00 },  // raw / brushed aluminium
  RIM:        { c: 0x2c2f34, r: 0.42, m: 1.00 },  // matte dark-anodised rim — a
                                                  // bright rim turns the wheels
                                                  // into a pair of mirror hoops
  ALU_MACH:   { c: 0xa3aab2, r: 0.34, m: 1.00 },  // machined + lightly polished alu
  STANCHION:  { c: 0xb9bfc6, r: 0.115, m: 1.00 }, // hard-anodised stanchion. Still
                                                  // the shiniest thing on the bike,
                                                  // but its specular lobe is ~6x
                                                  // wider than the old 0.045 mirror
                                                  // so it no longer clips.
  WELD:       { c: 0x9aa0a6, r: 0.58, m: 1.00 },  // heat-tinted weld bead
  BLACK_ANO:  { c: 0x22262a, r: 0.36, m: 1.00 },  // black-anodised hardware
  BLACK_MATT: { c: 0x1b1e22, r: 0.66, m: 0.55 },  // matte black machined
  STEEL:      { c: 0x9aa0a6, r: 0.40, m: 1.00 },  // spokes, axles, bolts
  CHAIN:      { c: 0x565c62, r: 0.50, m: 1.00 },  // oily chain plates — a chain is
                                                  // NOT bright steel, and rendering it
                                                  // that way makes it a glowing rope
  ROTOR:      { c: 0x969ca2, r: 0.30, m: 1.00 },  // rotor braking surface
  ROTOR_ARM:  { c: 0x6e3319, r: 0.34, m: 1.00 },  // anodised rotor carrier
  RUBBER:     { c: 0x121417, r: 0.95, m: 0.00 },  // tyre carcass
  RUBBER_KNOB:{ c: 0x171a1e, r: 0.88, m: 0.00 },  // knobs catch a touch more light
  GRIP:       { c: 0x1c2025, r: 0.85, m: 0.00 },
  HOSE:       { c: 0x0f1114, r: 0.60, m: 0.00 },  // braided hose, slightly glossy
  SADDLE:     { c: 0x191c21, r: 0.68, m: 0.00, cc: 0.25 },
  CARBON:     { c: 0x1d2126, r: 0.42, m: 0.00, cc: 0.90 },
  CARBON_GLS: { c: 0x15181c, r: 0.34, m: 0.00, cc: 1.00 },  // lacquered carbon
  SEAL:       { c: 0x191c20, r: 0.55, m: 0.00 },  // fork wiper seals
  OIL:        { c: 0x2a2f36, r: 0.34, m: 0.90 },  // shock body / damper
};

// Atlas cell ids for `aPbr.w`. 0 = no decal; 1..4 select a 2x2 cell.
const DECAL = { NONE: 0, DOWNTUBE: 1, TOPTUBE: 2, BADGE: 3, STAY: 4 };

// =============================================================================
// PROCEDURAL TEXTURES — DataTexture only, so this builds without a DOM.
// =============================================================================

/** Value-noise lattice sampled with a smootherstep, tileable on `period`. */
function makeNoise2D(rng, period) {
  const g = new Float32Array(period * period);
  for (let i = 0; i < g.length; i++) g[i] = rng();
  const fade = (t) => t * t * t * (t * (t * 6 - 15) + 10);
  return (x, y) => {
    const xi = Math.floor(x), yi = Math.floor(y);
    const xf = x - xi, yf = y - yi;
    const x0 = ((xi % period) + period) % period, x1 = (x0 + 1) % period;
    const y0 = ((yi % period) + period) % period, y1 = (y0 + 1) % period;
    const u = fade(xf), v = fade(yf);
    const a = lerp(g[y0 * period + x0], g[y0 * period + x1], u);
    const b = lerp(g[y1 * period + x0], g[y1 * period + x1], u);
    return lerp(a, b, v);
  };
}

/** Central-difference a tileable height field into an RGBA normal map. */
function heightToNormalTexture(height, size, strength) {
  const data = new Uint8Array(size * size * 4);
  const wrap = (i) => ((i % size) + size) % size;
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const hl = height[y * size + wrap(x - 1)], hr = height[y * size + wrap(x + 1)];
      const hd = height[wrap(y - 1) * size + x], hu = height[wrap(y + 1) * size + x];
      // Gradient → normal. The 2/size factor keeps `strength` resolution-independent.
      let nx = (hl - hr) * strength, ny = (hd - hu) * strength, nz = 1.0;
      const inv = 1 / Math.sqrt(nx * nx + ny * ny + 1);
      nx *= inv; ny *= inv; nz *= inv;
      const o = (y * size + x) * 4;
      data[o] = (nx * 0.5 + 0.5) * 255;
      data[o + 1] = (ny * 0.5 + 0.5) * 255;
      data[o + 2] = (nz * 0.5 + 0.5) * 255;
      data[o + 3] = 255;
    }
  }
  const tex = new THREE.DataTexture(data, size, size, THREE.RGBAFormat);
  tex.colorSpace = THREE.NoColorSpace;            // data map, never sRGB
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.magFilter = THREE.LinearFilter;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.generateMipmaps = true;
  tex.needsUpdate = true;
  return tex;
}

/** Greyscale RGBA from a 0..1 field — used as a roughness variation map. */
function fieldToTexture(field, size) {
  const data = new Uint8Array(size * size * 4);
  for (let i = 0; i < size * size; i++) {
    const v = clamp01(field[i]) * 255;
    data[i * 4] = data[i * 4 + 1] = data[i * 4 + 2] = v;
    data[i * 4 + 3] = 255;
  }
  const tex = new THREE.DataTexture(data, size, size, THREE.RGBAFormat);
  tex.colorSpace = THREE.NoColorSpace;
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.magFilter = THREE.LinearFilter;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.generateMipmaps = true;
  tex.needsUpdate = true;
  return tex;
}

// =============================================================================
// DECAL ATLAS
// =============================================================================
// A bike in a bike game has to carry a brand, and the brand has to survive being
// read at 120 px. This builds a 2x2 RGBA atlas — down tube wordmark, top tube
// model mark, head badge emblem, chainstay strip — from a stroke font and a
// handful of filled polygons, and hands it to the shader through `uDecalMap`.
//
// AUTHORED-ART SEAM: swap this function for a loader and nothing downstream
// changes. Requirements on the replacement: RGBA8, square, 2x2 cells in the order
// above, **linear** colour (NOT sRGB — the sampler is a custom uniform and three
// will not decode it for you), straight (non-premultiplied) alpha, and RGB
// flooded into the transparent margin so bilinear filtering cannot pull a dark
// fringe out of it.
// =============================================================================

/** sRGB byte → linear float, the exact transfer function three uses. */
function srgbToLinear(u8) {
  const c = u8 / 255;
  return c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4);
}

// Single-stroke letterforms on a 0..1 box (x right, y up), authored at a
// condensed industrial proportion. Each glyph is a list of polylines; the
// rasteriser gives them round caps and joins, which is what turns a stroke font
// into something that reads as a real heavy sans rather than a plotter font.
const GLYPHS = {
  'A': [[[0, 0], [0.38, 1], [0.76, 0]], [[0.13, 0.34], [0.63, 0.34]]],
  'B': [[[0, 0], [0, 1], [0.52, 1], [0.72, 0.84], [0.72, 0.66], [0.50, 0.53], [0, 0.53]],
    [[0.50, 0.53], [0.74, 0.38], [0.74, 0.15], [0.52, 0], [0, 0]]],
  'F': [[[0.74, 1], [0, 1], [0, 0]], [[0, 0.52], [0.58, 0.52]]],
  'J': [[[0.62, 1], [0.62, 0.22], [0.44, 0], [0.18, 0], [0, 0.20]]],
  'K': [[[0, 0], [0, 1]], [[0.72, 1], [0.05, 0.44]], [[0.26, 0.60], [0.76, 0]]],
  'Q': [[[0.20, 1], [0.54, 1], [0.74, 0.76], [0.74, 0.24], [0.54, 0], [0.20, 0], [0, 0.24], [0, 0.76], [0.20, 1]],
    [[0.46, 0.26], [0.82, -0.08]]],
  'Z': [[[0, 1], [0.76, 1], [0, 0], [0.76, 0]]],
  'C': [[[0.80, 0.84], [0.58, 1], [0.22, 1], [0, 0.74], [0, 0.26], [0.22, 0], [0.58, 0], [0.80, 0.16]]],
  'D': [[[0, 0], [0, 1], [0.46, 1], [0.78, 0.74], [0.78, 0.26], [0.46, 0], [0, 0]]],
  'E': [[[0.74, 1], [0, 1], [0, 0], [0.74, 0]], [[0, 0.5], [0.60, 0.5]]],
  'G': [[[0.78, 0.84], [0.56, 1], [0.22, 1], [0, 0.74], [0, 0.26], [0.22, 0], [0.58, 0], [0.78, 0.20], [0.78, 0.44], [0.46, 0.44]]],
  'H': [[[0, 0], [0, 1]], [[0.74, 0], [0.74, 1]], [[0, 0.52], [0.74, 0.52]]],
  'I': [[[0.30, 0], [0.30, 1]]],
  'L': [[[0, 1], [0, 0], [0.70, 0]]],
  'M': [[[0, 0], [0, 1], [0.42, 0.42], [0.84, 1], [0.84, 0]]],
  'N': [[[0, 0], [0, 1], [0.76, 0], [0.76, 1]]],
  'O': [[[0.22, 1], [0.60, 1], [0.82, 0.74], [0.82, 0.26], [0.60, 0], [0.22, 0], [0, 0.26], [0, 0.74], [0.22, 1]]],
  'P': [[[0, 0], [0, 1], [0.52, 1], [0.76, 0.82], [0.76, 0.62], [0.52, 0.46], [0, 0.46]]],
  'R': [[[0, 0], [0, 1], [0.52, 1], [0.76, 0.82], [0.76, 0.64], [0.50, 0.48], [0, 0.48]], [[0.36, 0.48], [0.78, 0]]],
  'S': [[[0.80, 0.86], [0.56, 1], [0.20, 1], [0, 0.82], [0.06, 0.62], [0.70, 0.44], [0.78, 0.22], [0.58, 0], [0.20, 0], [0, 0.14]]],
  'T': [[[0, 1], [0.78, 1]], [[0.39, 1], [0.39, 0]]],
  'U': [[[0, 1], [0, 0.24], [0.20, 0], [0.56, 0], [0.76, 0.24], [0.76, 1]]],
  'V': [[[0, 1], [0.38, 0], [0.76, 1]]],
  'W': [[[0, 1], [0.18, 0], [0.48, 0.66], [0.78, 0], [0.96, 1]]],
  'X': [[[0, 1], [0.76, 0]], [[0, 0], [0.76, 1]]],
  'Y': [[[0, 1], [0.38, 0.48], [0.76, 1]], [[0.38, 0.48], [0.38, 0]]],
  '0': [[[0.20, 1], [0.54, 1], [0.74, 0.76], [0.74, 0.24], [0.54, 0], [0.20, 0], [0, 0.24], [0, 0.76], [0.20, 1]]],
  '1': [[[0.06, 0.80], [0.36, 1], [0.36, 0]], [[0.04, 0], [0.68, 0]]],
  '2': [[[0, 0.82], [0.20, 1], [0.56, 1], [0.74, 0.80], [0.68, 0.58], [0, 0], [0.74, 0]]],
  '3': [[[0, 0.92], [0.24, 1], [0.56, 1], [0.74, 0.80], [0.56, 0.56], [0.20, 0.56]], [[0.56, 0.56], [0.76, 0.32], [0.56, 0.04], [0.22, 0.04], [0, 0.14]]],
  '4': [[[0.56, 0], [0.56, 1], [0, 0.30], [0.78, 0.30]]],
  '5': [[[0.74, 1], [0.06, 1], [0, 0.56], [0.44, 0.62], [0.74, 0.42], [0.66, 0.12], [0.34, 0], [0.04, 0.10]]],
  '6': [[[0.72, 0.92], [0.48, 1], [0.18, 0.90], [0, 0.50], [0.02, 0.20], [0.28, 0], [0.58, 0.02], [0.74, 0.26], [0.58, 0.50], [0.22, 0.52], [0.02, 0.36]]],
  '7': [[[0, 1], [0.78, 1], [0.28, 0]]],
  '8': [[[0.38, 0.54], [0.14, 0.66], [0.14, 0.88], [0.38, 1], [0.62, 0.88], [0.62, 0.66], [0.38, 0.54], [0.08, 0.38], [0.08, 0.14], [0.38, 0], [0.68, 0.14], [0.68, 0.38], [0.38, 0.54]]],
  '9': [[[0.04, 0.08], [0.28, 0], [0.58, 0.10], [0.76, 0.50], [0.74, 0.80], [0.48, 1], [0.18, 0.98], [0.02, 0.74], [0.18, 0.50], [0.54, 0.48], [0.74, 0.64]]],
  '-': [[[0, 0.48], [0.68, 0.48]]],
  '.': [[[0.16, 0.03], [0.22, 0.03]]],
  ' ': [],
};
const GLYPH_ADV = 0.94;    // advance as a fraction of cap height, plus tracking

/**
 * Where each decal lands, and — critically — how it is PROPORTIONED.
 *
 * A square atlas cell projected onto a band 250 mm long and 45 mm tall is
 * stretched 5.6:1, so artwork authored square comes out grotesquely wide. Every
 * cell therefore carries `W`, the aspect of the band it will be projected onto,
 * and the rasteriser draws in a virtual space W units wide and 1 unit tall in
 * which x and y are PHYSICALLY EQUAL. Author here, get isotropic letterforms on
 * the bike. W is derived from the tube it goes on:
 *   W = (vSpan x tubeArcLength) / (uSpan x tubeCircumference)
 * Recompute it if you move a control point; it is the difference between a
 * wordmark and a smear.
 */
const DECAL_RECT = {
  // down tube: 0.647 m arc, ~0.226 m around ⇒ 252 x 45 mm band
  DOWNTUBE: { cell: DECAL.DOWNTUBE, vCentre: 0.46, vSpan: 0.39, uSpan: 0.20, W: 5.58 },
  // top tube: 0.428 m arc, ~0.151 m around ⇒ 171 x 33 mm band
  TOPTUBE: { cell: DECAL.TOPTUBE, vCentre: 0.52, vSpan: 0.40, uSpan: 0.22, W: 5.15 },
  // chainstay: 0.503 m arc, ~0.138 m around ⇒ 171 x 25 mm band
  STAY: { cell: DECAL.STAY, vCentre: 0.52, vSpan: 0.34, uSpan: 0.18, W: 6.89 },
  // head badge: an explicit 28 x 55 mm plate, so it is TALLER than it is wide
  BADGE: { cell: DECAL.BADGE, W: 0.518 },
};

/**
 * A tiny software rasteriser over one atlas cell. Everything is drawn as either
 * a thick polyline (round caps, exact distance field, so it antialiases properly
 * and mips down cleanly) or a convex filled polygon.
 *
 * Coordinates are VIRTUAL: x runs 0..W, y runs 0..1, and one unit of x is the
 * same physical length as one unit of y once the decal is on the bike.
 */
class DecalCell {
  constructor(size, W) {
    this.n = size;
    this.W = W || 1;
    this.a = new Float32Array(size * size);           // coverage
    this.rgb = new Float32Array(size * size * 3);     // LINEAR ink colour
  }
  /** Flood the whole cell with an ink colour so the transparent margin cannot
   *  bleed dark under bilinear filtering / mip reduction. */
  flood(hex) {
    const r = srgbToLinear((hex >> 16) & 255), g = srgbToLinear((hex >> 8) & 255), b = srgbToLinear(hex & 255);
    for (let i = 0; i < this.n * this.n; i++) { this.rgb[i * 3] = r; this.rgb[i * 3 + 1] = g; this.rgb[i * 3 + 2] = b; }
  }
  /** Source-OVER with straight alpha. The new ink is on top. */
  _paint(x, y, cov, r, g, b) {
    if (cov <= 0) return;
    const i = y * this.n + x;
    const ad = this.a[i];
    const ao = cov + ad * (1 - cov);
    if (ao <= 1e-6) return;
    const w = cov / ao;
    this.rgb[i * 3] += (r - this.rgb[i * 3]) * w;
    this.rgb[i * 3 + 1] += (g - this.rgb[i * 3 + 1]) * w;
    this.rgb[i * 3 + 2] += (b - this.rgb[i * 3 + 2]) * w;
    this.a[i] = ao;
  }
  /** Thick polyline in VIRTUAL coords (x 0..W, y 0..1). `w` is HALF-width. */
  stroke(pts, w, hex) {
    if (pts.length < 2) return;
    const N = this.n, W = this.W;
    const r = srgbToLinear((hex >> 16) & 255), g = srgbToLinear((hex >> 8) & 255), b = srgbToLinear(hex & 255);
    // One texel, measured in virtual units, along the worst of the two axes.
    const aa = Math.hypot(W, 1) / N * 0.5;
    for (let s = 0; s < pts.length - 1; s++) {
      const ax = pts[s][0], ay = pts[s][1], bx = pts[s + 1][0], by = pts[s + 1][1];
      const x0 = Math.max(0, Math.floor(((Math.min(ax, bx) - w - aa) / W) * N));
      const x1 = Math.min(N - 1, Math.ceil(((Math.max(ax, bx) + w + aa) / W) * N));
      const y0 = Math.max(0, Math.floor((Math.min(ay, by) - w - aa) * N));
      const y1 = Math.min(N - 1, Math.ceil((Math.max(ay, by) + w + aa) * N));
      const ex = bx - ax, ey = by - ay;
      const el = ex * ex + ey * ey;
      for (let py = y0; py <= y1; py++) {
        const fy = (py + 0.5) / N;
        for (let px = x0; px <= x1; px++) {
          const fx = ((px + 0.5) / N) * W;
          let t = el > 1e-12 ? ((fx - ax) * ex + (fy - ay) * ey) / el : 0;
          t = t < 0 ? 0 : t > 1 ? 1 : t;
          const dx = fx - (ax + ex * t), dy = fy - (ay + ey * t);
          const d = Math.sqrt(dx * dx + dy * dy);
          const k = clamp01((w + aa - d) / (2 * aa));
          this._paint(px, py, k * k * (3 - 2 * k), r, g, b);
        }
      }
    }
  }
  /** Filled polygon (even-odd), 4x4 supersampled so edges are clean. */
  fill(poly, hex) {
    const N = this.n, W = this.W;
    const r = srgbToLinear((hex >> 16) & 255), g = srgbToLinear((hex >> 8) & 255), b = srgbToLinear(hex & 255);
    let minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
    for (const p of poly) {
      if (p[0] < minx) minx = p[0]; if (p[0] > maxx) maxx = p[0];
      if (p[1] < miny) miny = p[1]; if (p[1] > maxy) maxy = p[1];
    }
    const x0 = Math.max(0, Math.floor((minx / W) * N)), x1 = Math.min(N - 1, Math.ceil((maxx / W) * N));
    const y0 = Math.max(0, Math.floor(miny * N)), y1 = Math.min(N - 1, Math.ceil(maxy * N));
    const M = poly.length;
    for (let py = y0; py <= y1; py++) {
      for (let px = x0; px <= x1; px++) {
        let hits = 0;
        for (let sy = 0; sy < 4; sy++) {
          const fy = (py + (sy + 0.5) / 4) / N;
          for (let sx = 0; sx < 4; sx++) {
            const fx = (((px + (sx + 0.5) / 4) / N)) * W;
            let inside = false;
            for (let i = 0, j = M - 1; i < M; j = i++) {
              const yi = poly[i][1], yj = poly[j][1];
              if ((yi > fy) !== (yj > fy)) {
                const xx = poly[i][0] + ((fy - yi) / (yj - yi)) * (poly[j][0] - poly[i][0]);
                if (fx < xx) inside = !inside;
              }
            }
            if (inside) hits++;
          }
        }
        this._paint(px, py, hits / 16, r, g, b);
      }
    }
  }
  /** Lay out a string. Returns the advance width actually used. */
  text(str, x, y, capH, weight, hex, align) {
    let w = 0;
    for (const ch of str) w += (GLYPHS[ch] ? GLYPH_ADV : 0.55) * capH;
    const ox = align === 'c' ? x - w * 0.5 : align === 'r' ? x - w : x;
    let cx = ox;
    for (const ch of str) {
      const g = GLYPHS[ch];
      if (g) {
        for (const poly of g) {
          const p = [];
          for (const v of poly) p.push([cx + v[0] * capH, y + v[1] * capH]);
          this.stroke(p, weight * capH, hex);
        }
      }
      cx += (g ? GLYPH_ADV : 0.55) * capH;
    }
    return w;
  }
}

/**
 * Compose the 2x2 atlas. Cell order matches the DECAL enum:
 *   1 DOWNTUBE  2 TOPTUBE
 *   3 BADGE     4 STAY
 * Cell (0,0) of the texture is the BOTTOM-LEFT, which is where cell 1 lands, so
 * the shader's `org = (mod(k,2), floor(k/2)) * 0.5` addresses them in that order.
 */
function buildDecalAtlas(size) {
  const px = size >> 1;
  const cells = [
    new DecalCell(px, DECAL_RECT.DOWNTUBE.W),
    new DecalCell(px, DECAL_RECT.TOPTUBE.W),
    new DecalCell(px, DECAL_RECT.BADGE.W),
    new DecalCell(px, DECAL_RECT.STAY.W),
  ];
  const INK = 0xf2f4f7;          // off-white vinyl
  const INK2 = 0xc6ccd3;
  const ACC = 0xe8622a;          // accent orange — the one saturated thing allowed

  // --- 1. down tube: 252 x 45 mm, so one virtual unit is 45 mm --------------
  {
    const c = cells[0], W = c.W;
    c.flood(INK);
    // Leading double chevron, then the wordmark at a 27 mm cap height, then a
    // rule. Everything is expressed against `W` so the margins survive a retune.
    const cv = (x0) => [[x0, 0.50], [x0 + 0.30, 0.86], [x0 + 0.52, 0.86], [x0 + 0.22, 0.50],
      [x0 + 0.52, 0.14], [x0 + 0.30, 0.14]];
    c.fill(cv(0.18), ACC);
    c.fill(cv(0.52), INK);
    const cap = 0.60;
    c.text('DESCENT', 1.22, 0.30, cap, 0.048, INK, 'l');
    c.stroke([[1.22, 0.175], [W - 0.22, 0.175]], 0.020, ACC);
  }
  // --- 2. top tube: 171 x 33 mm --------------------------------------------
  {
    const c = cells[1], W = c.W;
    c.flood(INK);
    c.text('V10', 0.30, 0.30, 0.58, 0.050, INK, 'l');
    c.stroke([[2.20, 0.20], [2.20, 0.86]], 0.022, ACC);
    c.text('CARBON', 2.50, 0.56, 0.24, 0.075, INK2, 'l');
    c.text('210 MM', 2.50, 0.20, 0.24, 0.075, INK2, 'l');
    void W;
  }
  // --- 3. head badge: 28 x 55 mm, i.e. TALLER than it is wide ---------------
  // Pure geometry, no type, so it survives being read at 12 px.
  {
    const c = cells[2], W = c.W;                 // W ≈ 0.518
    c.flood(INK);
    const X = (t) => t * W;                      // 0..1 across the badge
    const shield = [[X(0.06), 0.955], [X(0.94), 0.955], [X(0.94), 0.30], [X(0.50), 0.045], [X(0.06), 0.30]];
    c.fill(shield, 0x14171a);
    c.stroke(shield.concat([shield[0]]), 0.010, 0xc9a24a);
    // Sky band, snow peak, then a gold plinth.
    c.fill([[X(0.13), 0.90], [X(0.87), 0.90], [X(0.87), 0.70], [X(0.13), 0.70]], 0x2f4a63);
    c.fill([[X(0.13), 0.70], [X(0.34), 0.70], [X(0.50), 0.885], [X(0.66), 0.70], [X(0.87), 0.70],
      [X(0.87), 0.48], [X(0.13), 0.48]], 0xdfe6ec);
    c.fill([[X(0.13), 0.48], [X(0.87), 0.48], [X(0.87), 0.42], [X(0.13), 0.42]], 0xc9a24a);
    // Three speed bars in the tail of the shield.
    for (let i = 0; i < 3; i++) {
      const y = 0.345 - i * 0.070;
      const inset = 0.06 + i * 0.10;
      c.stroke([[X(0.14 + inset), y], [X(0.86 - inset), y]], 0.016, 0xc9a24a);
    }
  }
  // --- 4. chainstay strip: 171 x 25 mm --------------------------------------
  {
    const c = cells[3], W = c.W;
    c.flood(INK);
    c.stroke([[0.20, 0.80], [W - 0.20, 0.80]], 0.026, ACC);
    c.text('DESCENT', 0.20, 0.24, 0.42, 0.058, INK, 'l');
    c.text('SERIES 27', W - 0.20, 0.28, 0.20, 0.078, INK2, 'r');
  }

  // --- pack ------------------------------------------------------------------
  const data = new Uint8Array(size * size * 4);
  const enc = (v) => Math.max(0, Math.min(255, Math.round(v * 255)));
  for (let k = 0; k < 4; k++) {
    const ox = (k % 2) * px, oy = ((k / 2) | 0) * px;
    const c = cells[k];
    for (let y = 0; y < px; y++) {
      for (let x = 0; x < px; x++) {
        const si = y * px + x;
        const di = ((oy + y) * size + (ox + x)) * 4;
        // RGB is written everywhere (see flood()), alpha carries the shape.
        data[di] = enc(c.rgb[si * 3]);
        data[di + 1] = enc(c.rgb[si * 3 + 1]);
        data[di + 2] = enc(c.rgb[si * 3 + 2]);
        data[di + 3] = enc(c.a[si]);
      }
    }
  }
  const tex = new THREE.DataTexture(data, size, size, THREE.RGBAFormat);
  tex.colorSpace = THREE.NoColorSpace;   // authored linear: it is a custom uniform,
                                         // so three will not decode sRGB for us
  tex.wrapS = tex.wrapT = THREE.ClampToEdgeWrapping;
  tex.magFilter = THREE.LinearFilter;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.generateMipmaps = true;
  tex.needsUpdate = true;
  return tex;
}

/**
 * Brushed/anodised metal: fine circumferential grain (stretched hard along X so
 * it reads as a lathe-turned or extruded finish), a scatter of deeper scratches,
 * and a low-frequency mottle so large flat panels are not perfectly uniform.
 */
function buildMetalMaps(rng, size) {
  const n1 = makeNoise2D(rng, 64);
  const n2 = makeNoise2D(rng, 16);
  const h = new Float32Array(size * size);
  const rough = new Float32Array(size * size);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const u = x / size, v = y / size;
      // 24:1 anisotropy — grain runs along U (around a tube / around a hub).
      let g = n1(u * 20, v * 480) * 0.55 + n1(u * 40, v * 960) * 0.28;
      g += n2(u * 6, v * 96) * 0.30;
      const i = y * size + x;
      h[i] = g;
      // Roughness varies with the grain and picks up a slow mottle.
      rough[i] = 0.72 + (g - 0.55) * 0.55 + (n2(u * 4, v * 4) - 0.5) * 0.22;
    }
  }
  // Sparse deeper scratches, drawn as thin lines with a soft falloff.
  for (let s = 0; s < 90; s++) {
    const y0 = Math.floor(rng() * size);
    const len = 20 + rng() * (size * 0.7);
    const x0 = rng() * size;
    const slope = (rng() - 0.5) * 0.06;
    const depth = 0.12 + rng() * 0.22;
    for (let t = 0; t < len; t++) {
      const x = Math.floor(x0 + t) % size;
      const y = Math.floor(y0 + t * slope + size) % size;
      const i = y * size + x;
      h[i] -= depth;
      rough[i] += depth * 0.5;
    }
  }
  return { normal: heightToNormalTexture(h, size, size * 0.010), rough: fieldToTexture(rough, size) };
}

/**
 * 2×2 twill carbon weave. Tows are raised where they pass over, sunk where they
 * pass under; the twill offset is what gives carbon its diagonal ribbing.
 */
function buildCarbonMaps(rng, size) {
  const n = makeNoise2D(rng, 32);
  const h = new Float32Array(size * size);
  const rough = new Float32Array(size * size);
  const tows = 16;                     // tows across the tile
  const cell = size / tows;
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const tx = Math.floor(x / cell), ty = Math.floor(y / cell);
      const fx = (x / cell) - tx, fy = (y / cell) - ty;
      // 2×2 twill: a tow is "over" when ((tx - ty) mod 4) < 2.
      const over = (((tx - ty) % 4) + 4) % 4 < 2;
      // Cross-section of the tow itself (a shallow arch across its width).
      const arch = Math.sin(Math.PI * (over ? fy : fx));
      let v = over ? 0.62 + arch * 0.26 : 0.30 + arch * 0.14;
      // Individual filaments within the tow.
      v += Math.sin((over ? fx : fy) * Math.PI * 9) * 0.028;
      v += (n(x / size * 24, y / size * 24) - 0.5) * 0.05;
      const i = y * size + x;
      h[i] = v;
      // Resin pools in the valleys and reads glossier there.
      rough[i] = over ? 0.78 : 0.58;
    }
  }
  return { normal: heightToNormalTexture(h, size, size * 0.0045), rough: fieldToTexture(rough, size) };
}

// =============================================================================
// MATERIAL — one MeshPhysicalMaterial, everything else per-vertex.
// =============================================================================
// Two programs carry a dozen distinct finishes:
//   aPbr   = ( roughness, metalness, clearcoat, decalCell )
//   aDecal = ( decalRectU, decalRectV, thinHalfWidthMetres )
// and the patch below turns those into (a) correct PBR per finish, (b) a decal
// composited into albedo + clearcoat, and (c) a view-space expansion that stops
// spokes and chain plates falling below ~1.5 px. `shared` carries the uniform
// OBJECTS so a single write per frame updates both materials.
function patchBikeMaterial(mat, shared) {
  mat.onBeforeCompile = (shader) => {
    shader.uniforms.uDecalMap = shared.uDecalMap;
    shader.uniforms.uPx = shared.uPx;
    shader.uniforms.uThinPx = shared.uThinPx;

    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', `#include <common>
attribute vec4 aPbr;
attribute vec3 aDecal;
varying vec4 vPbr;
varying vec2 vDecal;
uniform float uPx;        // pixels per metre at one metre of view depth
uniform float uThinPx;    // minimum rendered HALF-width, in pixels`)
      .replace('#include <begin_vertex>', `#include <begin_vertex>
	vPbr = aPbr;
	vDecal = aDecal.xy;`)
      // Rebuild <project_vertex> so a sub-pixel feature can be widened in view
      // space. `transformedNormal` already exists here (defaultnormal_vertex runs
      // before begin_vertex) and it already carries the instanceMatrix rotation,
      // so this is correct for the instanced chain as well as the spokes.
      .replace('#include <project_vertex>', `
	vec4 mvPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		mvPosition = batchingMatrix * mvPosition;
	#endif
	#ifdef USE_INSTANCING
		mvPosition = instanceMatrix * mvPosition;
	#endif
	mvPosition = modelViewMatrix * mvPosition;
	if ( aDecal.z > 1e-5 ) {
		float vDepth = max( - mvPosition.z, 0.05 );
		float pxPerM = uPx / vDepth;
		float halfPx = aDecal.z * pxPerM;
		if ( halfPx < uThinPx ) {
			// Grow to the minimum, but never by more than 7x — past that the
			// spoke set has already merged into a disc and more would only
			// start eating the rim.
			float grow = min( ( uThinPx - halfPx ) / pxPerM, aDecal.z * 7.0 );
			mvPosition.xyz += normalize( transformedNormal ) * grow;
		}
	}
	gl_Position = projectionMatrix * mvPosition;`);

    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `#include <common>
varying vec4 vPbr;
varying vec2 vDecal;
uniform sampler2D uDecalMap;`)
      // Decal, composited straight into albedo. The fetch is unconditional (a
      // texture2D inside non-uniform control flow has undefined derivatives) and
      // the cell select is arithmetic, so there is no branch at all.
      .replace('#include <color_fragment>', `#include <color_fragment>
	float gDecalA = 0.0;
	{
		float dOn = step( 0.5, vPbr.w );
		float dK = max( vPbr.w - 1.0, 0.0 );
		vec2 dOrg = vec2( mod( dK, 2.0 ), floor( dK * 0.5 ) ) * 0.5;
		vec2 dEdge = fwidth( vDecal ) + 1e-4;
		vec2 dIn = smoothstep( vec2( 0.0 ), dEdge, vDecal ) *
		           smoothstep( vec2( 0.0 ), dEdge, vec2( 1.0 ) - vDecal );
		// 0.4995 + a half-texel inset keeps bilinear taps inside their own cell.
		vec4 dTex = texture2D( uDecalMap, dOrg + clamp( vDecal, 0.0, 1.0 ) * 0.4990 + 0.0005 );
		gDecalA = dTex.a * dIn.x * dIn.y * dOn;
		diffuseColor.rgb = mix( diffuseColor.rgb, dTex.rgb, gDecalA );
	}`)
      // roughnessmap_fragment leaves roughnessFactor = material.roughness (1.0)
      // x map.g. The map's mean is ~0.72, so using it as a multiplier silently
      // made every authored roughness ~30 % smoother. Re-centre it on 1.0 and let
      // the per-vertex value be the truth. Vinyl is smoother than paint.
      .replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
	roughnessFactor = clamp( vPbr.x * ( 0.74 + roughnessFactor * 0.36 ) * ( 1.0 - gDecalA * 0.35 ), 0.02, 1.0 );`)
      // metalness is authoritative per-vertex — no map involved. A decal is vinyl,
      // i.e. a dielectric, even when it is stuck to a metal.
      .replace('#include <metalnessmap_fragment>', `#include <metalnessmap_fragment>
	metalnessFactor = vPbr.y * ( 1.0 - gDecalA );`)
      // Per-vertex clearcoat. lights_physical_fragment has already resolved
      // material.clearcoatRoughness (uniform + geometryRoughness); only the
      // strength is per-vertex, and a decal is lacquered over.
      .replace('#include <lights_physical_fragment>', `#include <lights_physical_fragment>
	#ifdef USE_CLEARCOAT
		material.clearcoat = saturate( vPbr.z + gDecalA * 0.45 );
	#endif`);

    mat.userData.shader = shader;
  };
  // Without this, three cannot tell a patched program from an unpatched one.
  mat.customProgramCacheKey = () => 'descent-bike-pbr-v3';
  return mat;
}

function makeFinishMaterial(maps, normalScale, shared) {
  const mat = new THREE.MeshPhysicalMaterial({
    color: 0xffffff,
    vertexColors: true,
    // roughness/metalness are placeholders; the real values arrive per-vertex.
    roughness: 1.0,
    metalness: 1.0,
    // clearcoat must be non-zero at compile time or three will not emit
    // USE_CLEARCOAT and the per-vertex override has nothing to write into.
    clearcoat: 1.0,
    clearcoatRoughness: 0.10,
    normalMap: maps ? maps.normal : null,
    roughnessMap: maps ? maps.rough : null,
    envMapIntensity: 1.0,
    dithering: true,
  });
  if (mat.normalScale) mat.normalScale.set(normalScale, normalScale);
  return patchBikeMaterial(mat, shared);
}

// =============================================================================
// GEOMETRY HELPERS
// =============================================================================

const _m4 = new THREE.Matrix4();
const _m4b = new THREE.Matrix4();
const _v = new THREE.Vector3();
const _v2 = new THREE.Vector3();
const _q = new THREE.Quaternion();

/** Guarantee position/normal/uv/index so everything can be merged blindly. */
function normaliseGeo(geo) {
  if (!geo.getAttribute('normal')) geo.computeVertexNormals();
  if (!geo.getAttribute('uv')) {
    const n = geo.getAttribute('position').count;
    geo.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(n * 2), 2));
  }
  if (!geo.index) {
    const n = geo.getAttribute('position').count;
    const idx = n > 65535 ? new Uint32Array(n) : new Uint16Array(n);
    for (let i = 0; i < n; i++) idx[i] = i;
    geo.setIndex(new THREE.BufferAttribute(idx, 1));
  }
  // Drop anything we do not merge, so the attribute sets always line up.
  for (const key of Object.keys(geo.attributes)) {
    if (key !== 'position' && key !== 'normal' && key !== 'uv' &&
        key !== 'color' && key !== 'aPbr' && key !== 'aDecal') geo.deleteAttribute(key);
  }
  return geo;
}

/** Multiply the UVs in place — controls the physical scale of the grain. */
function scaleUV(geo, su, sv) {
  const uv = geo.getAttribute('uv');
  for (let i = 0; i < uv.count; i++) uv.setXY(i, uv.getX(i) * su, uv.getY(i) * sv);
  uv.needsUpdate = true;
  return geo;
}

/**
 * Revolve a 2D profile about the X axis. `profile` is a flat [r0,x0, r1,x1, …]
 * list; pass `closed` for a solid section (rim, tyre, shock body). Normals are
 * derived from the profile tangent rather than face-averaged, so hard corners
 * stay hard and the seam of a closed profile does not smear.
 */
function latheX(profile, segments, closed, uRepeat, vRepeat) {
  const pts = profile.length / 2;
  const rows = closed ? pts + 1 : pts;
  const cols = segments + 1;
  const pos = new Float32Array(rows * cols * 3);
  const nor = new Float32Array(rows * cols * 3);
  const uv = new Float32Array(rows * cols * 2);
  // Arc length along the profile, for a non-stretched V.
  const vArr = new Float32Array(rows);
  let total = 0;
  for (let i = 1; i < rows; i++) {
    const a = (i - 1) % pts, b = i % pts;
    total += Math.hypot(profile[b * 2] - profile[a * 2], profile[b * 2 + 1] - profile[a * 2 + 1]);
    vArr[i] = total;
  }
  if (total <= 0) total = 1;

  for (let i = 0; i < rows; i++) {
    const k = i % pts;
    const r = profile[k * 2], x = profile[k * 2 + 1];
    // Profile tangent from the neighbours (one-sided at open ends).
    const kp = closed ? (k - 1 + pts) % pts : Math.max(0, k - 1);
    const kn = closed ? (k + 1) % pts : Math.min(pts - 1, k + 1);
    let tr = profile[kn * 2] - profile[kp * 2];
    let tx = profile[kn * 2 + 1] - profile[kp * 2 + 1];
    const tl = Math.hypot(tr, tx) || 1;
    tr /= tl; tx /= tl;
    // Outward normal in the (x, r) plane is the tangent rotated +90°.
    const nr = tx, nx = -tr;
    for (let j = 0; j < cols; j++) {
      const a = (j / segments) * Math.PI * 2;
      const ca = Math.cos(a), sa = Math.sin(a);
      const o = (i * cols + j) * 3;
      pos[o] = x; pos[o + 1] = r * ca; pos[o + 2] = r * sa;
      nor[o] = nx; nor[o + 1] = nr * ca; nor[o + 2] = nr * sa;
      const p = (i * cols + j) * 2;
      uv[p] = (j / segments) * (uRepeat || 1);
      uv[p + 1] = (vArr[i] / total) * (vRepeat || 1);
    }
  }
  const idx = [];
  for (let i = 0; i < rows - 1; i++) {
    for (let j = 0; j < segments; j++) {
      const a = i * cols + j, b = a + 1, c = a + cols, d = c + 1;
      idx.push(a, c, b, b, c, d);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  geo.setIndex(idx);
  return geo;
}

/**
 * Sweep an elliptical section along a Catmull-Rom curve using parallel-transport
 * frames. `stations` is [{ t, rx, ry }] — this is what gives the frame tubes a
 * varying cross-section (round at the head tube, wide flat oval at the BB)
 * instead of the dead-uniform cylinders that shout "placeholder".
 *
 * FRAME CONVENTION — read this before tuning any section. rx/ry are measured in
 * the transported frame, not in world axes, and which world axis each lands on
 * depends on the curve's direction:
 *   curve running in the YZ plane (frame tubes, stays, rocker):
 *       rx = in-plane depth,  ry = LATERAL (±X) half-width
 *   curve running along ±X (bars, arches, bridges):
 *       rx = fore/aft (±Z),   ry = vertical (±Y)
 *   curve running along ±Z (crank arms, chain links):
 *       rx = vertical (±Y),   ry = LATERAL (±X)
 * Get it backwards and you get a 50 mm-thick crank arm that looks fine in a
 * side-on screenshot and absurd from anywhere else.
 */
function sweptTube(points, stations, sides, steps, opts) {
  const o = opts || {};
  const curve = new THREE.CatmullRomCurve3(points, false, 'centripetal', 0.5);
  const N = steps;
  const P = [], T = [], NRM = [], BIN = [];
  for (let i = 0; i <= N; i++) {
    const t = i / N;
    P.push(curve.getPointAt(t));
    T.push(curve.getTangentAt(t).normalize());
  }
  // Seed the frame with the world axis least aligned with the first tangent.
  const t0 = T[0];
  const seed = Math.abs(t0.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
  let n0 = new THREE.Vector3().crossVectors(t0, seed).normalize();
  if (o.up) {
    n0 = new THREE.Vector3().crossVectors(t0, o.up).normalize();
    if (n0.lengthSq() < 1e-8) n0 = new THREE.Vector3().crossVectors(t0, seed).normalize();
  }
  NRM.push(n0);
  BIN.push(new THREE.Vector3().crossVectors(t0, n0).normalize());
  for (let i = 1; i <= N; i++) {
    const q = new THREE.Quaternion().setFromUnitVectors(T[i - 1], T[i]);
    const n = NRM[i - 1].clone().applyQuaternion(q).normalize();
    NRM.push(n);
    BIN.push(new THREE.Vector3().crossVectors(T[i], n).normalize());
  }

  const sectionAt = (t) => {
    let a = stations[0], b = stations[stations.length - 1];
    for (let i = 0; i < stations.length - 1; i++) {
      if (t >= stations[i].t && t <= stations[i + 1].t) { a = stations[i]; b = stations[i + 1]; break; }
    }
    const span = Math.max(1e-6, b.t - a.t);
    const k = clamp01((t - a.t) / span);
    const s = k * k * (3 - 2 * k);      // smooth, so section changes look formed
    return [lerp(a.rx, b.rx, s), lerp(a.ry, b.ry, s)];
  };

  const cols = sides + 1;
  const pos = new Float32Array((N + 1) * cols * 3);
  const uv = new Float32Array((N + 1) * cols * 2);
  let arc = 0;
  const arcs = new Float32Array(N + 1);
  for (let i = 1; i <= N; i++) { arc += P[i].distanceTo(P[i - 1]); arcs[i] = arc; }
  const total = arc || 1;
  for (let i = 0; i <= N; i++) {
    const t = i / N;
    const [rx, ry] = sectionAt(t);
    for (let j = 0; j <= sides; j++) {
      const a = (j / sides) * Math.PI * 2;
      const ca = Math.cos(a), sa = Math.sin(a);
      const oi = (i * cols + j) * 3;
      pos[oi] = P[i].x + NRM[i].x * rx * ca + BIN[i].x * ry * sa;
      pos[oi + 1] = P[i].y + NRM[i].y * rx * ca + BIN[i].y * ry * sa;
      pos[oi + 2] = P[i].z + NRM[i].z * rx * ca + BIN[i].z * ry * sa;
      const ui = (i * cols + j) * 2;
      uv[ui] = j / sides;
      uv[ui + 1] = arcs[i] / total;
    }
  }
  const idx = [];
  for (let i = 0; i < N; i++) {
    for (let j = 0; j < sides; j++) {
      const a = i * cols + j, b = a + 1, c = a + cols, d = c + 1;
      idx.push(a, c, b, b, c, d);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();

  // Caps as separate fans so their normals stay flat.
  const caps = [];
  const makeCap = (i, dir) => {
    const [rx, ry] = sectionAt(i / N);
    const cp = new Float32Array((sides + 2) * 3);
    const cn = new Float32Array((sides + 2) * 3);
    const cu = new Float32Array((sides + 2) * 2);
    cp[0] = P[i].x; cp[1] = P[i].y; cp[2] = P[i].z;
    for (let j = 0; j <= sides; j++) {
      const a = (j / sides) * Math.PI * 2;
      const ca = Math.cos(a), sa = Math.sin(a);
      const oi = (j + 1) * 3;
      cp[oi] = P[i].x + NRM[i].x * rx * ca + BIN[i].x * ry * sa;
      cp[oi + 1] = P[i].y + NRM[i].y * rx * ca + BIN[i].y * ry * sa;
      cp[oi + 2] = P[i].z + NRM[i].z * rx * ca + BIN[i].z * ry * sa;
      cu[(j + 1) * 2] = 0.5 + ca * 0.5; cu[(j + 1) * 2 + 1] = 0.5 + sa * 0.5;
    }
    for (let j = 0; j < sides + 2; j++) {
      cn[j * 3] = T[i].x * dir; cn[j * 3 + 1] = T[i].y * dir; cn[j * 3 + 2] = T[i].z * dir;
    }
    const ci = [];
    for (let j = 0; j < sides; j++) {
      if (dir > 0) ci.push(0, j + 1, j + 2); else ci.push(0, j + 2, j + 1);
    }
    const cg = new THREE.BufferGeometry();
    cg.setAttribute('position', new THREE.BufferAttribute(cp, 3));
    cg.setAttribute('normal', new THREE.BufferAttribute(cn, 3));
    cg.setAttribute('uv', new THREE.BufferAttribute(cu, 2));
    cg.setIndex(ci);
    caps.push(cg);
  };
  if (o.capStart !== false) makeCap(0, -1);
  if (o.capEnd !== false) makeCap(N, 1);

  return caps.length ? mergeRaw([geo].concat(caps)) : geo;
}

/**
 * A toothed disc — chainring, cassette cog, derailleur pulley. The tooth profile
 * is generated directly rather than extruded from a Shape, which keeps the
 * triangle count honest (a 34 t ring is ~800 triangles).
 */
function gearGeo(rBore, rRoot, rTip, teeth, thickness) {
  const outline = [];
  const step = (Math.PI * 2) / teeth;
  for (let i = 0; i < teeth; i++) {
    const a = i * step;
    outline.push([rRoot, a]);
    outline.push([rTip, a + step * 0.17]);
    outline.push([rTip, a + step * 0.33]);
    outline.push([rRoot, a + step * 0.50]);
  }
  const n = outline.length;
  const hx = thickness * 0.5;
  const pos = [], nor = [], uv = [], idx = [];
  // Two annulus faces (bore ring + outline ring), then the rim wall.
  for (let side = 0; side < 2; side++) {
    const x = side ? hx : -hx;
    const nx = side ? 1 : -1;
    const base = pos.length / 3;
    for (let i = 0; i < n; i++) {
      const a = outline[i][1];
      pos.push(x, rBore * Math.cos(a), rBore * Math.sin(a));
      nor.push(nx, 0, 0);
      uv.push(i / n * 6, 0);
      pos.push(x, outline[i][0] * Math.cos(a), outline[i][0] * Math.sin(a));
      nor.push(nx, 0, 0);
      uv.push(i / n * 6, 1);
    }
    for (let i = 0; i < n; i++) {
      const a0 = base + i * 2, a1 = base + ((i + 1) % n) * 2;
      if (side) idx.push(a0, a0 + 1, a1, a1, a0 + 1, a1 + 1);
      else idx.push(a0, a1, a0 + 1, a1, a1 + 1, a0 + 1);
    }
  }
  const base = pos.length / 3;
  for (let i = 0; i < n; i++) {
    const a = outline[i][1], r = outline[i][0];
    const ca = Math.cos(a), sa = Math.sin(a);
    pos.push(-hx, r * ca, r * sa); nor.push(0, ca, sa); uv.push(i / n * 8, 0);
    pos.push(hx, r * ca, r * sa); nor.push(0, ca, sa); uv.push(i / n * 8, 1);
  }
  for (let i = 0; i < n; i++) {
    const a0 = base + i * 2, a1 = base + ((i + 1) % n) * 2;
    idx.push(a0, a1, a0 + 1, a1, a1 + 1, a0 + 1);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  return geo;
}

/** A tapered box — the shape of every tyre knob and most small brackets. */
function taperBox(w, h, d, taper, topScaleD) {
  const tw = w * taper, td = d * (topScaleD === undefined ? taper : topScaleD);
  const hw = w * 0.5, hd = d * 0.5, thw = tw * 0.5, thd = td * 0.5;
  const v = [
    -hw, 0, -hd, hw, 0, -hd, hw, 0, hd, -hw, 0, hd,          // bottom 0-3
    -thw, h, -thd, thw, h, -thd, thw, h, thd, -thw, h, thd,   // top 4-7
  ];
  const f = [
    [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [4, 5, 6, 7], [3, 2, 1, 0],
  ];
  const pos = [], uv = [], idx = [];
  for (const q of f) {
    const b = pos.length / 3;
    for (let i = 0; i < 4; i++) {
      pos.push(v[q[i] * 3], v[q[i] * 3 + 1], v[q[i] * 3 + 2]);
      uv.push(i === 1 || i === 2 ? 1 : 0, i >= 2 ? 1 : 0);
    }
    idx.push(b, b + 1, b + 2, b, b + 2, b + 3);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  return geo;
}

/** Raw merge with no per-vertex material data (used inside builders). */
function mergeRaw(list) {
  let vc = 0, ic = 0;
  for (const g of list) {
    normaliseGeo(g);
    vc += g.getAttribute('position').count;
    ic += g.index.count;
  }
  const pos = new Float32Array(vc * 3), nor = new Float32Array(vc * 3), uv = new Float32Array(vc * 2);
  const idx = vc > 65535 ? new Uint32Array(ic) : new Uint16Array(ic);
  let vo = 0, io = 0;
  for (const g of list) {
    const p = g.getAttribute('position'), nn = g.getAttribute('normal'), u = g.getAttribute('uv');
    pos.set(p.array.subarray(0, p.count * 3), vo * 3);
    nor.set(nn.array.subarray(0, nn.count * 3), vo * 3);
    uv.set(u.array.subarray(0, u.count * 2), vo * 2);
    const gi = g.index.array;
    for (let i = 0; i < g.index.count; i++) idx[io + i] = gi[i] + vo;
    vo += p.count; io += g.index.count;
    g.dispose();
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  geo.setIndex(new THREE.BufferAttribute(idx, 1));
  return geo;
}

/**
 * Accumulates finished parts and merges them into one geometry, remembering each
 * part's vertex range so morph targets can be built afterwards.
 */
class Parts {
  constructor() { this.list = []; this.total = 0; }
  /**
   * @param {object} [opts] extras:
   *   opts.thin   half-width in metres of a sub-pixel feature (spoke, chain plate)
   *   opts.decal  { cell, N, sides, vCentre, vSpan, uSpan } — project an atlas cell
   *               onto a sweptTube's flanks. See tubeDecalRect().
   *   opts.decalUV { cell, fn(x,y,z,out2) } — explicit rect coords, for plates.
   * @returns {{start:number,count:number}} the merged-geometry vertex range.
   */
  add(geo, fin, matrix, uvScale, opts) {
    normaliseGeo(geo);
    if (matrix) geo.applyMatrix4(matrix);
    if (uvScale) scaleUV(geo, uvScale[0], uvScale[1]);
    const n = geo.getAttribute('position').count;
    const col = new Float32Array(n * 3);
    const pbr = new Float32Array(n * 4);
    const dec = new Float32Array(n * 3);
    const c = new THREE.Color(fin.c);          // sRGB hex → linear working space
    const o = opts || {};
    const cell = o.decal ? o.decal.cell : (o.decalUV ? o.decalUV.cell : DECAL.NONE);
    const thin = o.thin || 0;
    for (let i = 0; i < n; i++) {
      col[i * 3] = c.r; col[i * 3 + 1] = c.g; col[i * 3 + 2] = c.b;
      pbr[i * 4] = fin.r; pbr[i * 4 + 1] = fin.m;
      pbr[i * 4 + 2] = fin.cc || 0; pbr[i * 4 + 3] = cell;
      // Default the decal rect far outside [0,1] so nothing shows by accident.
      dec[i * 3] = -8; dec[i * 3 + 1] = -8; dec[i * 3 + 2] = thin;
    }
    if (o.decal) tubeDecalRect(geo, o.decal, dec);
    if (o.decalUV) {
      const p = geo.getAttribute('position');
      const out = [0, 0];
      for (let i = 0; i < n; i++) {
        o.decalUV.fn(p.getX(i), p.getY(i), p.getZ(i), out);
        dec[i * 3] = out[0]; dec[i * 3 + 1] = out[1];
      }
    }
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    geo.setAttribute('aPbr', new THREE.BufferAttribute(pbr, 4));
    geo.setAttribute('aDecal', new THREE.BufferAttribute(dec, 3));
    const range = { start: this.total, count: n };
    this.total += n;
    this.list.push(geo);
    return range;
  }
  build() {
    if (!this.list.length) return new THREE.BufferGeometry();
    let vc = 0, ic = 0;
    for (const g of this.list) { vc += g.getAttribute('position').count; ic += g.index.count; }
    const pos = new Float32Array(vc * 3), nor = new Float32Array(vc * 3);
    const uv = new Float32Array(vc * 2), col = new Float32Array(vc * 3);
    const pbr = new Float32Array(vc * 4), dec = new Float32Array(vc * 3);
    const idx = vc > 65535 ? new Uint32Array(ic) : new Uint16Array(ic);
    let vo = 0, io = 0;
    for (const g of this.list) {
      const p = g.getAttribute('position');
      pos.set(p.array.subarray(0, p.count * 3), vo * 3);
      nor.set(g.getAttribute('normal').array.subarray(0, p.count * 3), vo * 3);
      uv.set(g.getAttribute('uv').array.subarray(0, p.count * 2), vo * 2);
      col.set(g.getAttribute('color').array.subarray(0, p.count * 3), vo * 3);
      pbr.set(g.getAttribute('aPbr').array.subarray(0, p.count * 4), vo * 4);
      dec.set(g.getAttribute('aDecal').array.subarray(0, p.count * 3), vo * 3);
      const gi = g.index.array;
      for (let i = 0; i < g.index.count; i++) idx[io + i] = gi[i] + vo;
      vo += p.count; io += g.index.count;
      g.dispose();
    }
    this.list.length = 0;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
    geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    geo.setAttribute('aPbr', new THREE.BufferAttribute(pbr, 4));
    geo.setAttribute('aDecal', new THREE.BufferAttribute(dec, 3));
    geo.setIndex(new THREE.BufferAttribute(idx, 1));
    geo.computeBoundingSphere();
    return geo;
  }
}

/**
 * Project a decal rect onto the two flanks of a sweptTube.
 *
 * sweptTube lays its shell out as (N+1) rings x (sides+1) columns, so I know
 * exactly which vertex is where. The only genuinely fiddly part is ORIENTATION:
 * a decal on the −X flank of a tube is seen from the opposite side, so the same
 * texture mapping reads mirrored there. Rather than guess, both axes take their
 * sign from the geometry itself:
 *
 *   screenRight(flank) = up x n      (n = outward surface normal)
 *   screenUp(flank)    = up projected into the surface tangent plane
 *
 * and the along-tube / around-tube parameter directions are dotted against those.
 * Signs only degenerate where n is near ±Y — the top and bottom of the tube — and
 * that is 0.25 in u away from the flank centre, i.e. well outside any band we
 * ever ask for (uSpan is kept ≤ 0.22). So the wordmark reads correctly from both
 * sides of the bike with no per-call bookkeeping.
 */
function tubeDecalRect(geo, cfg, out) {
  const p = geo.getAttribute('position');
  const N = cfg.N, sides = cfg.sides;
  const cols = sides + 1;
  const need = (N + 1) * cols;
  if (p.count < need) return;               // not a tube shell — leave defaults
  const vc = cfg.vCentre, vSpan = cfg.vSpan, uSpan = cfg.uSpan;
  const cy = new Float64Array(N + 1), cyz = new Float64Array(N + 1), cyx = new Float64Array(N + 1);
  for (let i = 0; i <= N; i++) {
    let sx = 0, sy = 0, sz = 0;
    for (let j = 0; j < sides; j++) {
      const k = i * cols + j;
      sx += p.getX(k); sy += p.getY(k); sz += p.getZ(k);
    }
    cyx[i] = sx / sides; cy[i] = sy / sides; cyz[i] = sz / sides;
  }
  for (let i = 0; i <= N; i++) {
    const ia = Math.max(0, i - 1), ib = Math.min(N, i + 1);
    const tx = cyx[ib] - cyx[ia], ty = cy[ib] - cy[ia], tz = cyz[ib] - cyz[ia];
    const v = i / N;
    for (let j = 0; j <= sides; j++) {
      const k = i * cols + j;
      const u = j / sides;
      // Only the two flanks carry the decal; everything else stays out of range.
      const du = u <= 0.5 ? u - 0.25 : u - 0.75;
      if (Math.abs(du) > uSpan * 0.75) continue;
      // Outward normal from the ring centre.
      let nx = p.getX(k) - cyx[i], ny = p.getY(k) - cy[i], nz = p.getZ(k) - cyz[i];
      const nl = Math.hypot(nx, ny, nz) || 1;
      nx /= nl; ny /= nl; nz /= nl;
      // screenRight = up x n
      const rx = 1 * nz - 0 * ny, ry = 0 * nx - 0 * nz, rz = 0 * ny - 1 * nx;
      void ry;
      const dv = tx * rx + ty * 0 + tz * rz;
      // screenUp = up − n (n·up)
      const dot = ny;
      let ux = -nx * dot, uy = 1 - ny * dot, uz = -nz * dot;
      const ul = Math.hypot(ux, uy, uz) || 1;
      ux /= ul; uy /= ul; uz /= ul;
      // Direction of increasing u along the ring.
      const ja = (j - 1 + sides) % sides, jb = (j + 1) % sides;
      const ka = i * cols + ja, kb = i * cols + jb;
      const gx = p.getX(kb) - p.getX(ka), gy = p.getY(kb) - p.getY(ka), gz = p.getZ(kb) - p.getZ(ka);
      const dua = gx * ux + gy * uy + gz * uz;
      const sV = dv >= 0 ? 1 : -1;
      const sU = dua >= 0 ? 1 : -1;
      out[k * 3] = 0.5 + sV * (v - vc) / vSpan;
      out[k * 3 + 1] = 0.5 + sU * du / uSpan;
    }
  }
}

/**
 * A weld bead: an elliptical ring of overlapping "dimes". A TIG bead on an alloy
 * frame is a stack of crescents, and the ripple is the whole reason a weld reads
 * as a weld rather than as a fillet — so the section radius is modulated round
 * the ring instead of being constant. Built in its own XY plane about +Z.
 */
function weldRing(rx, ry, beadR, ripples, segs, sides) {
  const cols = sides + 1;
  const pos = new Float32Array((segs + 1) * cols * 3);
  const nor = new Float32Array((segs + 1) * cols * 3);
  const uv = new Float32Array((segs + 1) * cols * 2);
  for (let i = 0; i <= segs; i++) {
    const a = (i / segs) * Math.PI * 2;
    const ca = Math.cos(a), sa = Math.sin(a);
    const px = rx * ca, py = ry * sa;
    // Ellipse tangent (unnormalised is fine, we only need a direction).
    let tx = -rx * sa, ty = ry * ca;
    const tl = Math.hypot(tx, ty) || 1;
    tx /= tl; ty /= tl;
    // In-plane outward normal of the ring path.
    const ox = ty, oy = -tx;
    const r = beadR * (0.80 + 0.20 * Math.abs(Math.sin(ripples * a)));
    for (let j = 0; j <= sides; j++) {
      const b = (j / sides) * Math.PI * 2;
      const cb = Math.cos(b), sb = Math.sin(b);
      const k = (i * cols + j) * 3;
      pos[k] = px + ox * r * cb;
      pos[k + 1] = py + oy * r * cb;
      pos[k + 2] = r * sb;
      nor[k] = ox * cb; nor[k + 1] = oy * cb; nor[k + 2] = sb;
      const m = (i * cols + j) * 2;
      uv[m] = (i / segs) * 8; uv[m + 1] = j / sides;
    }
  }
  const idx = [];
  for (let i = 0; i < segs; i++) {
    for (let j = 0; j < sides; j++) {
      const a = i * cols + j, b = a + 1, c = a + cols, d = c + 1;
      idx.push(a, c, b, b, c, d);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  geo.setIndex(idx);
  return geo;
}

/**
 * Orient a weld ring so its plane is perpendicular to `axis` and its centre sits
 * at `c`. `axis` is the direction of the tube the bead wraps.
 */
function weldAt(c, axis, rx, ry, beadR, ripples, q) {
  const geo = weldRing(rx, ry, beadR, ripples, q >= 2 ? 40 : 24, q >= 2 ? 8 : 6);
  const z = _v.copy(axis).normalize();
  _q.setFromUnitVectors(new THREE.Vector3(0, 0, 1), z);
  _m4.compose(c, _q, new THREE.Vector3(1, 1, 1));
  geo.applyMatrix4(_m4);
  return geo;
}

/**
 * A gusset: a triangular plate of real thickness, spanning three points. Two DH
 * frames in ten are gusseted at the head tube and the BB and it is one of the
 * few pieces of structure a viewer can name.
 */
function gussetPlate(a, b, c, thick) {
  // Plate normal from the triangle, thickness split either side.
  const ab = new THREE.Vector3().subVectors(b, a);
  const ac = new THREE.Vector3().subVectors(c, a);
  const n = new THREE.Vector3().crossVectors(ab, ac).normalize().multiplyScalar(thick * 0.5);
  const v = [];
  for (const p of [a, b, c]) v.push(p.x - n.x, p.y - n.y, p.z - n.z);
  for (const p of [a, b, c]) v.push(p.x + n.x, p.y + n.y, p.z + n.z);
  const idx = [
    0, 2, 1, 3, 4, 5,                    // the two faces
    0, 1, 4, 0, 4, 3,                    // three edge walls
    1, 2, 5, 1, 5, 4,
    2, 0, 3, 2, 3, 5,
  ];
  const uv = [];
  for (let i = 0; i < 6; i++) uv.push((i % 3) * 0.5, (i / 3) | 0);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  return geo;
}

/**
 * Build a morph target from a per-vertex displacement function. Morph targets are
 * how the brake pads squeeze, the levers pull and the frame flexes without any of
 * them costing a draw call.
 */
function addMorph(geo, fn) {
  const p = geo.getAttribute('position');
  const n = p.count;
  const target = new Float32Array(n * 3);
  const out = new THREE.Vector3();
  for (let i = 0; i < n; i++) {
    out.set(p.getX(i), p.getY(i), p.getZ(i));
    fn(out, i);
    target[i * 3] = out.x; target[i * 3 + 1] = out.y; target[i * 3 + 2] = out.z;
  }
  if (!geo.morphAttributes.position) geo.morphAttributes.position = [];
  geo.morphAttributes.position.push(new THREE.BufferAttribute(target, 3));
  geo.morphTargetsRelative = false;
  return geo.morphAttributes.position.length - 1;
}

// =============================================================================
// SOLVERS — allocation-free, called every frame.
// =============================================================================

/**
 * Circle–circle intersection in a 2D plane, choosing the branch nearest a
 * reference point. This is the whole four-bar: given where the Horst pivot has
 * been carried to by the swingarm, and the fixed rocker pivot, it finds the one
 * place the seatstay/rocker joint can be.
 */
const _cc = { y: 0, z: 0, ok: false };
function circleIntersect(c0y, c0z, r0, c1y, c1z, r1, refY, refZ) {
  const dy = c1y - c0y, dz = c1z - c0z;
  const d = Math.sqrt(dy * dy + dz * dz);
  _cc.ok = false;
  if (d < 1e-9 || d > r0 + r1 || d < Math.abs(r0 - r1)) return _cc;
  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const hs = r0 * r0 - a * a;
  if (hs < 0) return _cc;
  const h = Math.sqrt(hs);
  const my = c0y + (a * dy) / d, mz = c0z + (a * dz) / d;
  const py = -dz / d, pz = dy / d;
  const ay = my + h * py, az = mz + h * pz;
  const by = my - h * py, bz = mz - h * pz;
  const da = (ay - refY) * (ay - refY) + (az - refZ) * (az - refZ);
  const db = (by - refY) * (by - refY) + (bz - refZ) * (bz - refZ);
  if (da <= db) { _cc.y = ay; _cc.z = az; } else { _cc.y = by; _cc.z = bz; }
  _cc.ok = true;
  return _cc;
}

/**
 * Belt-over-pulleys solver for the chain.
 *
 * The chain is a convex loop around four circles — chainring, cassette cog and
 * both derailleur pulleys — all wrapped the same way, so every connecting span is
 * an external tangent. (That is not a simplification: on a real bike the chain
 * runs round the *outside* of both pulleys and the cage sits inside it. The "S"
 * you see is the cage, not the chain.)
 *
 * Direction of travel, which is worth stating because it is counter-intuitive:
 * the taut top run moves FORWARD, from the cassette to the chainring. The chain
 * leaves the chainring at the bottom, runs rearward as the slack return, and the
 * derailleur — which lives on the slack side, as it must — feeds it back up.
 *
 * Working plane is (a = −z, b = y): a points forward, b points up, i.e. exactly
 * what you see looking at the drive side of the bike, so senses read naturally.
 */
class Belt {
  constructor(n) {
    this.n = n;
    this.cx = new Float64Array(n);      // circle centre a
    this.cy = new Float64Array(n);      // circle centre b
    this.r = new Float64Array(n);
    // Per circle: arrival normal angle, departure normal angle, wrap (negative =
    // clockwise, which is the sense the whole loop runs in).
    this.aIn = new Float64Array(n);
    this.aOut = new Float64Array(n);
    this.wrap = new Float64Array(n);
    // Per span: start point, unit direction, length.
    this.sx = new Float64Array(n);
    this.sy = new Float64Array(n);
    this.dx = new Float64Array(n);
    this.dy = new Float64Array(n);
    this.len = new Float64Array(n);
    this.cum = new Float64Array(2 * n + 1);   // cumulative arc length, arc/span
    this.total = 0;
    this.valid = false;
  }

  /**
   * Solve the tangents and wraps. Every circle is wrapped clockwise, so for two
   * circles the common normal n satisfies d·n = r0 − r1, and the departure
   * tangent is −perp(n).
   */
  solve() {
    const n = this.n;
    // Tangent normals, one per span i → i+1.
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      const dx = this.cx[j] - this.cx[i], dy = this.cy[j] - this.cy[i];
      const D = Math.sqrt(dx * dx + dy * dy);
      const k = (this.r[i] - this.r[j]) / (D || 1);
      if (D < 1e-6 || Math.abs(k) > 1) { this.valid = false; return false; }
      const ux = dx / D, uy = dy / D;
      const wx = -uy, wy = ux;                 // perpendicular (CCW) of u
      const b = Math.sqrt(Math.max(0, 1 - k * k));   // + branch ⇒ clockwise wrap
      const nx = k * ux + b * wx, ny = k * uy + b * wy;
      // Departure point on i, arrival point on j, both offset along the same n.
      const px = this.cx[i] + this.r[i] * nx, py = this.cy[i] + this.r[i] * ny;
      const qx = this.cx[j] + this.r[j] * nx, qy = this.cy[j] + this.r[j] * ny;
      const ex = qx - px, ey = qy - py;
      const L = Math.sqrt(ex * ex + ey * ey) || 1e-6;
      this.sx[i] = px; this.sy[i] = py;
      this.dx[i] = ex / L; this.dy[i] = ey / L;
      this.len[i] = L;
      this.aOut[i] = Math.atan2(ny, nx);
      this.aIn[j] = Math.atan2(ny, nx);
    }
    // Wrap angles, taken clockwise (negative) from arrival to departure.
    this.total = 0;
    for (let i = 0; i < n; i++) {
      let w = this.aOut[i] - this.aIn[i];
      while (w > 0) w -= Math.PI * 2;
      while (w < -Math.PI * 2) w += Math.PI * 2;
      this.wrap[i] = w;
      this.cum[i * 2] = this.total;
      this.total += Math.abs(w) * this.r[i];
      this.cum[i * 2 + 1] = this.total;
      this.total += this.len[i];
    }
    this.cum[2 * n] = this.total;
    this.valid = isFinite(this.total) && this.total > 0.1;
    return this.valid;
  }

  /**
   * Position + tangent at arc length s (wrapped). Writes into `out` as
   * { a, b, ta, tb }. No allocation.
   */
  sample(s, out) {
    const S = ((s % this.total) + this.total) % this.total;
    const n = this.n;
    let seg = 0;
    for (let i = 0; i < 2 * n; i++) { if (S >= this.cum[i] && S < this.cum[i + 1]) { seg = i; break; } }
    const i = seg >> 1;
    if ((seg & 1) === 0) {
      // On the arc of circle i.
      const local = S - this.cum[seg];
      const ang = this.aIn[i] + Math.sign(this.wrap[i]) * (local / (this.r[i] || 1e-6));
      const ca = Math.cos(ang), sa = Math.sin(ang);
      out.a = this.cx[i] + this.r[i] * ca;
      out.b = this.cy[i] + this.r[i] * sa;
      // Clockwise motion ⇒ tangent is −perp(normal).
      out.ta = sa; out.tb = -ca;
    } else {
      const local = S - this.cum[seg];
      out.a = this.sx[i] + this.dx[i] * local;
      out.b = this.sy[i] + this.dy[i] * local;
      out.ta = this.dx[i]; out.tb = this.dy[i];
    }
    return out;
  }
}

// =============================================================================
// PART BUILDERS
// =============================================================================

/**
 * A complete wheel: box-section rim, 32 spokes in true 3-cross lacing, hub with
 * flanges, knobby tyre with displaced tread, 200 mm rotor, and (for the rear) the
 * cassette. Local frame: axle along X, wheel in the YZ plane.
 */
function buildWheel(rear, q, rng) {
  const P = new Parts();
  const seg = q >= 2 ? 72 : 48;
  const rimOuter = 0.3120, rimBed = 0.2835;

  // ---- rim: a real box section with bead hooks -----------------------------
  // (radius, x) traversed so that the outer surface runs +x ⇒ outward normals.
  const rimProfile = [
    0.2800, -0.0125,  0.2782, -0.0158,  0.2860, -0.0186,  0.3000, -0.0190,
    0.3095, -0.0164,  0.3120, -0.0122,  0.3062, -0.0104,  0.3062, 0.0104,
    0.3120, 0.0122,  0.3095, 0.0164,  0.3000, 0.0190,  0.2860, 0.0186,
    0.2782, 0.0158,  0.2800, 0.0125,  0.2835, 0.0090,  0.2835, -0.0090,
  ];
  P.add(latheX(rimProfile, seg, true, 26, 1), F.RIM, null, [1, 1]);
  // Rim tape / inner floor so the spoke bed does not read as a hole.
  P.add(latheX([0.2828, -0.0095, 0.2828, 0.0095], seg, false, 22, 1), F.BLACK_MATT);

  // ---- hub -----------------------------------------------------------------
  const flangeR = 0.0298;
  const flangeXL = -0.0290;
  const flangeXR = rear ? 0.0225 : 0.0290;   // rear hub is dished for the cassette
  const hub = [
    0.0165, -0.0500, 0.0215, -0.0500, 0.0215, flangeXL - 0.0030,
    flangeR, flangeXL - 0.0030, flangeR, flangeXL + 0.0030, 0.0215, flangeXL + 0.0030,
    0.0215, flangeXR - 0.0030, flangeR, flangeXR - 0.0030, flangeR, flangeXR + 0.0030,
    0.0215, flangeXR + 0.0030, 0.0215, 0.0500, 0.0165, 0.0500,
  ];
  P.add(latheX(hub, q >= 2 ? 32 : 20, true, 8, 1), F.BLACK_ANO);
  // Axle through the hub, plus end caps.
  P.add(latheX([0.0100, -0.0800, 0.0100, 0.0800], q >= 2 ? 20 : 12, false, 6, 1), F.STEEL);

  // ---- 32 spokes, 3-cross --------------------------------------------------
  // The rim hole for a spoke sits n·720/N degrees around from its hub hole:
  // for n = 3 crossings on a 32 h wheel that is 67.5°, and it is that specific
  // angle that makes a 3-cross wheel instantly recognisable.
  const CROSS_ANGLE = (3 * 720 / 32) * Math.PI / 180;
  const spokeR = 0.00105;
  const spokes = [];
  for (let side = 0; side < 2; side++) {
    const fx = side ? flangeXR : flangeXL;
    const base = side ? Math.PI / 16 : 0;          // flanges are clocked half a hole
    for (let k = 0; k < 16; k++) {
      const hubA = base + k * (Math.PI / 8);
      const dir = (k % 2 === 0) ? 1 : -1;           // alternating leading/trailing
      const rimA = hubA + dir * CROSS_ANGLE;
      const hy = flangeR * Math.cos(hubA), hz = flangeR * Math.sin(hubA);
      const ry = rimBed * Math.cos(rimA), rz = rimBed * Math.sin(rimA);
      // J-bend: the spoke leaves the flange face, not its centre.
      const ex = fx + (side ? -0.0032 : 0.0032);
      const dxv = 0 - ex, dyv = ry - hy, dzv = rz - hz;
      const len = Math.sqrt(dxv * dxv + dyv * dyv + dzv * dzv);
      const g = new THREE.CylinderGeometry(spokeR, spokeR * 0.92, len, 5, 1, true);
      // Cylinder is +Y aligned; aim it down the spoke.
      _v.set(dxv / len, dyv / len, dzv / len);
      _q.setFromUnitVectors(new THREE.Vector3(0, 1, 0), _v);
      _m4.compose(new THREE.Vector3(ex + dxv * 0.5, hy + dyv * 0.5, hz + dzv * 0.5), _q, new THREE.Vector3(1, 1, 1));
      g.applyMatrix4(_m4);
      spokes.push(g);
      // Nipple at the rim, pointing back down the spoke.
      const nip = new THREE.CylinderGeometry(0.0025, 0.0019, 0.011, 6, 1);
      _v.set(0, -Math.cos(rimA), -Math.sin(rimA));
      _q.setFromUnitVectors(new THREE.Vector3(0, 1, 0), _v);
      _m4.compose(new THREE.Vector3(0, rimBed * Math.cos(rimA) * 1.005, rimBed * Math.sin(rimA) * 1.005), _q, new THREE.Vector3(1, 1, 1));
      nip.applyMatrix4(_m4);
      spokes.push(nip);
    }
  }
  // A 1.05 mm spoke is 0.066 px wide at 15 m. `thin` hands its real half-width to
  // the vertex shader, which widens it in view space until it covers ~1.5 px —
  // that is what stops the wheel breaking into a dotted sparkle, and at range the
  // 32 widened spokes merge into a disc, which is the LOD the work order asked
  // for without adding an alpha test to the frame.
  P.add(mergeRaw(spokes), F.STEEL, null, [1, 3], { thin: spokeR });

  // ---- tyre ----------------------------------------------------------------
  // 2.5" DH casing. Outer carcass radius 0.3565; knobs take it to 0.370.
  const tyreProfile = [
    0.3098, -0.0128, 0.3230, -0.0232, 0.3400, -0.0300, 0.3505, -0.0292,
    0.3556, -0.0182, 0.3568, 0.0000, 0.3556, 0.0182, 0.3505, 0.0292,
    0.3400, 0.0300, 0.3230, 0.0232, 0.3098, 0.0128,
    0.3135, 0.0100, 0.3150, 0.0000, 0.3135, -0.0100,
  ];
  P.add(latheX(tyreProfile, seg, true, 30, 2), F.RUBBER, null, [1, 1]);

  // Knobs: two centre rows, an intermediate row and side knobs that stand proud
  // of the shoulder so they break the silhouette — the whole point of modelling
  // them rather than faking them in a normal map.
  const knobRows = q >= 2 ? 34 : 24;
  const knobs = [];
  const knobAt = (ang, x, w, l, h, tilt) => {
    // Built pointing +Y, then aimed radially outward at `ang` and tilted onto
    // the shoulder for the side knobs.
    const g = taperBox(w, h, l, 0.72, 0.80);
    const dirY = Math.cos(ang), dirZ = Math.sin(ang);
    const nx = Math.sin(tilt), ny = Math.cos(tilt) * dirY, nz = Math.cos(tilt) * dirZ;
    _v.set(nx, ny, nz).normalize();
    _q.setFromUnitVectors(new THREE.Vector3(0, 1, 0), _v);
    // Sit the knob base slightly inside the carcass so there is never a gap.
    const r = 0.3520 - Math.abs(x) * 0.28;
    _m4.compose(new THREE.Vector3(x, r * dirY, r * dirZ), _q, new THREE.Vector3(1, 1, 1));
    g.applyMatrix4(_m4);
    knobs.push(g);
  };
  for (let i = 0; i < knobRows; i++) {
    const a = (i / knobRows) * Math.PI * 2;
    const alt = i % 2 === 0;
    // Centre tread: paired blocks, alternating with a single wide block.
    if (alt) {
      knobAt(a, -0.0072, 0.0125, 0.0180, 0.0150, 0);
      knobAt(a, 0.0072, 0.0125, 0.0180, 0.0150, 0);
    } else {
      knobAt(a, 0.0000, 0.0180, 0.0130, 0.0142, 0);
    }
    // Intermediate + side knobs, tilted onto the shoulder.
    const s = alt ? 1 : -1;
    knobAt(a + 0.045, s * 0.0215, 0.0135, 0.0175, 0.0140, s * 0.30);
    knobAt(a - 0.045, -s * 0.0215, 0.0135, 0.0175, 0.0140, -s * 0.30);
    knobAt(a + 0.09, s * 0.0292, 0.0130, 0.0200, 0.0128, s * 0.72);
    knobAt(a - 0.09, -s * 0.0292, 0.0130, 0.0200, 0.0128, -s * 0.72);
  }
  P.add(mergeRaw(knobs), F.RUBBER_KNOB, null, [2, 2]);

  // ---- 200 mm floating rotor, vented through --------------------------------
  P.add(buildRotor(q), F.ROTOR, new THREE.Matrix4().makeTranslation(-G.rotorX, 0, 0));
  // Carrier + rivets are a different finish, so they are added by buildRotor's
  // companion below.
  P.add(buildRotorCarrier(q), F.ROTOR_ARM, new THREE.Matrix4().makeTranslation(-G.rotorX, 0, 0));

  // ---- cassette (rear only) -------------------------------------------------
  if (rear) {
    const cogs = [];
    for (let i = 0; i < G.cogTeeth.length; i++) {
      const t = G.cogTeeth[i];
      const rp = pitchRadius(t);
      const g = gearGeo(Math.min(0.0215, rp - 0.010), rp - 0.0042, rp + 0.0038, t, 0.0018);
      g.applyMatrix4(new THREE.Matrix4().makeTranslation(G.cogX[i], 0, 0));
      cogs.push(g);
    }
    // Freehub body.
    cogs.push(latheX([0.0180, 0.0430, 0.0180, 0.0920, 0.0150, 0.0920], 20, false, 6, 1));
    P.add(mergeRaw(cogs), F.STEEL, null, [3, 1]);
  }

  const geo = P.build();
  return geo;
}

/** The braking surface: an annulus with vent slots and drillings cut clean through. */
function buildRotor(q) {
  const shape = new THREE.Shape();
  shape.absarc(0, 0, G.rotorR, 0, Math.PI * 2, false);
  const inner = new THREE.Path();
  inner.absarc(0, 0, 0.0735, 0, Math.PI * 2, true);
  shape.holes.push(inner);
  const nSlot = q >= 2 ? 12 : 8;
  for (let i = 0; i < nSlot; i++) {
    const a = (i / nSlot) * Math.PI * 2;
    // Curved vent slot, drawn as a thin annular sector.
    const p = new THREE.Path();
    const r0 = 0.0800, r1 = 0.0935, half = 0.085;
    p.moveTo(Math.cos(a - half) * r0, Math.sin(a - half) * r0);
    p.absarc(0, 0, r0, a - half, a + half, false);
    p.lineTo(Math.cos(a + half) * r1, Math.sin(a + half) * r1);
    p.absarc(0, 0, r1, a + half, a - half, true);
    p.closePath();
    shape.holes.push(p);
  }
  const nDrill = q >= 2 ? 24 : 12;
  for (let i = 0; i < nDrill; i++) {
    const a = ((i + 0.5) / nDrill) * Math.PI * 2;
    const r = i % 2 ? 0.0885 : 0.0785;
    const p = new THREE.Path();
    p.absarc(Math.cos(a) * r, Math.sin(a) * r, 0.0042, 0, Math.PI * 2, true);
    shape.holes.push(p);
  }
  const geo = new THREE.ExtrudeGeometry(shape, { depth: 0.0019, bevelEnabled: false, curveSegments: q >= 2 ? 14 : 8 });
  geo.translate(0, 0, -0.00095);
  geo.rotateY(Math.PI / 2);      // extruded along +Z; rotors live in the YZ plane
  return normaliseGeo(geo);
}

/** The anodised spider the rotor floats on, plus its rivets. */
function buildRotorCarrier(q) {
  const shape = new THREE.Shape();
  shape.absarc(0, 0, 0.0755, 0, Math.PI * 2, false);
  const bore = new THREE.Path();
  bore.absarc(0, 0, 0.0290, 0, Math.PI * 2, true);
  shape.holes.push(bore);
  for (let i = 0; i < 5; i++) {
    const a = (i / 5) * Math.PI * 2;
    const p = new THREE.Path();
    const r0 = 0.0400, r1 = 0.0650, half = 0.42;
    p.moveTo(Math.cos(a - half) * r0, Math.sin(a - half) * r0);
    p.absarc(0, 0, r0, a - half, a + half, false);
    p.lineTo(Math.cos(a + half) * r1, Math.sin(a + half) * r1);
    p.absarc(0, 0, r1, a + half, a - half, true);
    p.closePath();
    shape.holes.push(p);
  }
  // Six-bolt mounting holes.
  for (let i = 0; i < 6; i++) {
    const a = (i / 6) * Math.PI * 2;
    const p = new THREE.Path();
    p.absarc(Math.cos(a) * 0.0222, Math.sin(a) * 0.0222, 0.0026, 0, Math.PI * 2, true);
    shape.holes.push(p);
  }
  const carrier = new THREE.ExtrudeGeometry(shape, { depth: 0.0022, bevelEnabled: false, curveSegments: q >= 2 ? 14 : 8 });
  carrier.translate(0, 0, -0.0011);
  carrier.rotateY(Math.PI / 2);
  const parts = [normaliseGeo(carrier)];
  for (let i = 0; i < 10; i++) {
    const a = ((i + 0.5) / 10) * Math.PI * 2;
    const rv = new THREE.CylinderGeometry(0.0032, 0.0032, 0.0052, 8, 1);
    rv.rotateZ(Math.PI / 2);
    rv.translate(0, Math.cos(a) * 0.0745, Math.sin(a) * 0.0745);
    parts.push(rv);
  }
  return mergeRaw(parts);
}

// =============================================================================
// DERIVED SUSPENSION GEOMETRY
// Mirrors the maths in bike.js exactly (and prefers bike.geometry when it is
// there) so the drawn axles sit precisely where the simulation says they are.
// =============================================================================
function deriveGeometry(bg) {
  const g = {};
  const mk = (rake, len, travel, sag, sign, mountUp, mountFwd) => {
    const cosR = Math.cos(rake), sinR = Math.sin(rake);
    const L = len - travel * sag;
    return {
      rake, cosR, sinR, len, travel, sagT: travel * sag,
      mountUp: mountUp !== undefined ? mountUp : cosR * L,
      mountFwd: mountFwd !== undefined ? mountFwd : sign * (G.wheelbase * 0.5) - sinR * L,
    };
  };
  g.fork = mk(
    bg && bg.forkRake !== undefined ? bg.forkRake : G.forkRake,
    bg && bg.forkLen !== undefined ? bg.forkLen : G.forkLen,
    bg && bg.forkTravel !== undefined ? bg.forkTravel : G.forkTravel,
    G.forkSag, +1,
    bg ? bg.forkMountUp : undefined, bg ? bg.forkMountFwd : undefined,
  );
  g.shock = mk(
    bg && bg.shockRake !== undefined ? bg.shockRake : G.shockRake,
    bg && bg.shockLen !== undefined ? bg.shockLen : G.shockLen,
    bg && bg.shockTravel !== undefined ? bg.shockTravel : G.shockTravel,
    G.shockSag, -1,
    bg ? bg.shockMountUp : undefined, bg ? bg.shockMountFwd : undefined,
  );
  // Axle position for a given compression, in chassis-local (y, z).
  g.forkAxle = (t, out) => {
    const s = g.fork.len - t;
    out.y = g.fork.mountUp - g.fork.cosR * s;
    out.z = -g.fork.mountFwd - g.fork.sinR * s;
    return out;
  };
  g.rearAxle = (t, out) => {
    const s = g.shock.len - t;
    out.y = g.shock.mountUp - g.shock.cosR * s;
    out.z = -g.shock.mountFwd - g.shock.sinR * s;
    return out;
  };
  // The fork strut axis is also the steering axis. Its origin is the chassis-side
  // mount; +Y in this frame runs up the steerer, +Z runs "backward" (which is
  // where the fork offset puts the head tube relative to the legs).
  g.steerOrigin = new THREE.Vector3(0, g.fork.mountUp, -g.fork.mountFwd);
  g.steerToChassis = new THREE.Matrix4()
    .makeRotationX(g.fork.rake)
    .setPosition(g.steerOrigin);
  return g;
}

// =============================================================================
// FRAME — front triangle. Hydroformed swept tubes, not cylinders.
// =============================================================================
function buildFrameAlu(SG, q, rng) {
  const P = new Parts();
  const sides = q >= 2 ? 16 : 10;
  const steps = q >= 2 ? 18 : 10;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);

  // ---- head tube, on the steerer line (offset behind the leg axis) ----------
  const ht = new Parts();
  const htGeo = latheX([
    0.0290, -0.062, 0.0300, -0.055, 0.0272, -0.048, 0.0272, 0.040,
    0.0300, 0.048, 0.0312, 0.058, 0.0290, 0.064,
  ], sides + 4, false, 8, 1);
  // latheX revolves about X; the head tube runs along the steer axis (+Y here).
  htGeo.rotateZ(Math.PI / 2);
  htGeo.translate(0, G.crownLower + 0.072, G.forkOffset);
  ht.add(htGeo, F.PAINT);
  // Tapered headset cups: 1.5" bottom, 1⅛" top.
  const cupLo = latheX([0.0300, -0.010, 0.0320, -0.006, 0.0320, 0.006, 0.0295, 0.010], sides + 4, false, 8, 1);
  cupLo.rotateZ(Math.PI / 2); cupLo.translate(0, G.crownLower + 0.020, G.forkOffset);
  ht.add(cupLo, F.BLACK_ANO);
  const cupHi = latheX([0.0245, -0.009, 0.0265, -0.005, 0.0265, 0.005, 0.0240, 0.009], sides + 4, false, 8, 1);
  cupHi.rotateZ(Math.PI / 2); cupHi.translate(0, G.crownLower + 0.128, G.forkOffset);
  ht.add(cupHi, F.ALU_MACH);
  // ---- head badge ----------------------------------------------------------
  // A raised shield on the front face of the head tube, between the crowns. This
  // is the one place on a bike a viewer looks for a maker's mark, and it is the
  // cheapest identity in the model: 1.2 mm of proud plate and one atlas cell.
  {
    // 29 x 55 mm. Deliberately narrower than the 54 mm head tube: a flat plate on
    // a round tube has to stay inside the chord or its edges float clear of it.
    // The width is DERIVED from the atlas cell's aspect so the two cannot drift.
    const bh = 0.027, bw = bh * DECAL_RECT.BADGE.W;
    const bz = G.forkOffset - 0.0272;                           // front face of the HT
    const badgeY = G.crownLower + 0.076;
    // Rounded-shield outline (x right, y up in the badge's own plane).
    const outline = [
      [-1.00, 0.92], [-0.72, 1.00], [0.72, 1.00], [1.00, 0.92],
      [1.00, -0.22], [0.62, -0.72], [0.00, -1.00], [-0.62, -0.72], [-1.00, -0.22],
    ];
    const pos = [], uvb = [], idxb = [];
    // Front face (slightly domed) + a rim wall back to the head tube.
    pos.push(0, badgeY, bz - 0.0022); uvb.push(0.5, 0.5);
    for (const o of outline) {
      const x = o[0] * bw, y = badgeY + o[1] * bh;
      const dome = 0.0022 * (1 - 0.55 * (o[0] * o[0] + o[1] * o[1]));
      pos.push(x, y, bz - dome); uvb.push(0.5 + o[0] * 0.5, 0.5 + o[1] * 0.5);
    }
    for (const o of outline) {
      const x = o[0] * bw, y = badgeY + o[1] * bh;
      pos.push(x, y, bz + 0.011); uvb.push(0.5 + o[0] * 0.5, 0.5 + o[1] * 0.5);
    }
    const M = outline.length;
    for (let i = 0; i < M; i++) {
      const a = 1 + i, b = 1 + ((i + 1) % M);
      idxb.push(0, b, a);                              // face, normal toward −Z
      const ra = 1 + M + i, rb = 1 + M + ((i + 1) % M);
      idxb.push(a, b, rb, a, rb, ra);                  // rim wall
    }
    const badge = new THREE.BufferGeometry();
    badge.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    badge.setAttribute('uv', new THREE.Float32BufferAttribute(uvb, 2));
    badge.setIndex(idxb);
    badge.computeVertexNormals();
    // Explicit decal rect: the badge's own plane maps 1:1 onto the atlas cell.
    ht.add(badge, F.PAINT_BLK, null, [1, 1], {
      decalUV: {
        cell: DECAL.BADGE,
        fn: (x, y, z, out) => { out[0] = 0.5 + x / (bw * 2.05); out[1] = 0.5 + (y - badgeY) / (bh * 2.05); void z; },
      },
    });
  }
  const htMerged = ht.build();
  htMerged.applyMatrix4(SG.steerToChassis);
  // Re-add as a single part so the steer-frame transform is baked in.
  P.list.push(htMerged); P.total += htMerged.getAttribute('position').count;

  // Anchor points for the tube ends, in chassis space.
  const htLow = new THREE.Vector3(0, G.crownLower + 0.020, G.forkOffset).applyMatrix4(SG.steerToChassis);
  const htMid = new THREE.Vector3(0, G.crownLower + 0.070, G.forkOffset).applyMatrix4(SG.steerToChassis);
  const htTop = new THREE.Vector3(0, G.crownLower + 0.126, G.forkOffset).applyMatrix4(SG.steerToChassis);

  // ---- bottom bracket shell -------------------------------------------------
  const bbShell = latheX([
    0.0210, -0.0480, 0.0245, -0.0430, 0.0245, 0.0430, 0.0210, 0.0480,
  ], sides + 4, false, 6, 1);
  bbShell.translate(0, G.bb.y, G.bb.z);
  P.add(bbShell, F.PAINT_DEEP, null, [1, 1]);

  // ---- seat tube: BB → rocker pivot -----------------------------------------
  P.add(sweptTube([
    V(0, G.bb.y + 0.010, G.bb.z + 0.004),
    V(0, 0.110, G.bb.z + 0.026),
    V(0, 0.280, G.seatTubeTop.z - 0.014),
    V(0, G.seatTubeTop.y + 0.030, G.seatTubeTop.z),
  ], [
    { t: 0.00, rx: 0.0245, ry: 0.0245 },
    { t: 0.35, rx: 0.0205, ry: 0.0225 },
    { t: 1.00, rx: 0.0192, ry: 0.0192 },
  ], sides, steps, { capStart: false }), F.PAINT, null, [3, 6]);

  // ---- seatpost + saddle ----------------------------------------------------
  // Stub post: a DH bike has the saddle slammed, and the linkage needs the room.
  const post = latheX([0.0157, -0.006, 0.0157, 0.026, 0.0150, 0.030], sides, false, 5, 2);
  post.rotateZ(Math.PI / 2);
  post.rotateX(0.22);                    // continues the seat tube's rearward lean
  post.translate(0, G.seatTubeTop.y + 0.016, G.seatTubeTop.z + 0.006);
  P.add(post, F.BLACK_ANO, null, [1, 2]);
  P.add(buildSaddle(q), F.SADDLE, new THREE.Matrix4().makeTranslation(0, G.saddle.y, G.saddle.z), [2, 2]);
  // Saddle rails and clamp.
  const rails = [];
  for (const s of [-1, 1]) {
    const r = new THREE.CylinderGeometry(0.0035, 0.0035, 0.150, 6, 1);
    r.rotateX(Math.PI / 2);
    r.translate(s * 0.021, G.saddle.y - 0.024, G.saddle.z + 0.004);
    rails.push(r);
  }
  const clamp1 = new THREE.CylinderGeometry(0.017, 0.019, 0.030, 10, 1);
  clamp1.rotateZ(Math.PI / 2);
  clamp1.rotateY(0.0);
  clamp1.translate(0, G.saddle.y - 0.028, G.saddle.z + 0.006);
  rails.push(clamp1);
  P.add(mergeRaw(rails), F.ALU_MACH);

  // ---- shock lower mount (frame side) + yoke --------------------------------
  const yoke = sweptTube([
    V(0, G.bb.y + 0.030, G.bb.z + 0.012),
    V(0, G.shockMount.y - 0.030, G.shockMount.z - 0.006),
    V(0, G.shockMount.y, G.shockMount.z),
  ], [
    { t: 0, rx: 0.030, ry: 0.020 },
    { t: 1, rx: 0.022, ry: 0.016 },
  ], sides, 8, {});
  P.add(yoke, F.PAINT_DEEP);
  const mountEye = latheX([0.0075, -0.0180, 0.0135, -0.0180, 0.0135, 0.0180, 0.0075, 0.0180], sides, true, 4, 1);
  mountEye.translate(0, G.shockMount.y, G.shockMount.z);
  P.add(mountEye, F.ALU_MACH);

  // ---- pivot bosses ---------------------------------------------------------
  const bosses = [];
  const boss = (y, z, r, w) => {
    const b = latheX([r * 0.45, -w, r, -w * 0.8, r, w * 0.8, r * 0.45, w], sides, true, 4, 1);
    b.translate(0, y, z);
    bosses.push(b);
  };
  boss(G.mainPivot.y, G.mainPivot.z, 0.030, 0.062);
  boss(G.rockerPivot.y, G.rockerPivot.z, 0.024, 0.030);
  P.add(mergeRaw(bosses), F.ALU_MACH);

  // ---- welds and gussets ----------------------------------------------------
  // F2: a frame with no welds is a frame with no manufacturing process. Beads go
  // where an alloy DH frame is actually welded, and their planes are set from the
  // direction of the tube each one wraps so they read as joints, not as rings.
  {
    const welds = [];
    const dtAxis = new THREE.Vector3(0, -0.62, 0.79);      // down tube leaving the HT
    const ttAxis = new THREE.Vector3(0, -0.24, 0.97);      // top tube leaving the HT
    // head tube ↔ down tube, head tube ↔ top tube
    welds.push(weldAt(V(0, htLow.y - 0.014, htLow.z + 0.028), dtAxis, 0.0335, 0.0320, 0.0038, 15, q));
    welds.push(weldAt(V(0, htTop.y - 0.006, htTop.z + 0.026), ttAxis, 0.0270, 0.0255, 0.0034, 13, q));
    // down tube ↔ BB shell, seat tube ↔ BB shell
    welds.push(weldAt(V(0, G.bb.y + 0.020, G.bb.z - 0.020), new THREE.Vector3(0, -0.68, 0.73), 0.0400, 0.0310, 0.0038, 16, q));
    welds.push(weldAt(V(0, G.bb.y + 0.036, G.bb.z + 0.014), new THREE.Vector3(0, 0.97, 0.24), 0.0260, 0.0260, 0.0034, 13, q));
    // top tube ↔ seat tube, and the rocker-pivot boss into the seat tube
    welds.push(weldAt(V(0, G.seatTubeTop.y + 0.022, G.seatTubeTop.z - 0.016), new THREE.Vector3(0, -0.30, -0.95), 0.0225, 0.0210, 0.0030, 12, q));
    for (const s2 of [-1, 1]) {
      welds.push(weldAt(V(s2 * 0.026, G.rockerPivot.y, G.rockerPivot.z), new THREE.Vector3(1, 0, 0), 0.0258, 0.0258, 0.0024, 12, q));
    }
    // Pivot-boss collars, on the OUTBOARD FACE of each boss — a bead at a boss's
    // mid-plane is buried inside the frame and reads as a machining step, not a weld.
    for (const s2 of [-1, 1]) {
      welds.push(weldAt(V(s2 * 0.056, G.mainPivot.y, G.mainPivot.z), new THREE.Vector3(1, 0, 0), 0.0320, 0.0320, 0.0028, 14, q));
    }
    P.add(mergeRaw(welds), F.WELD, null, [3, 1]);
  }
  {
    // Head-tube gusset, as a real triangular plate between the HT and the down
    // tube, and a BB gusset under the seat-tube junction.
    const gussets = [];
    gussets.push(gussetPlate(
      V(0, htLow.y + 0.006, htLow.z + 0.030),
      V(0, htLow.y - 0.096, htLow.z + 0.098),
      V(0, htLow.y - 0.060, htLow.z + 0.014), 0.020));
    gussets.push(gussetPlate(
      V(0, G.bb.y + 0.050, G.bb.z + 0.006),
      V(0, G.bb.y + 0.012, G.bb.z + 0.048),
      V(0, G.bb.y + 0.008, G.bb.z - 0.004), 0.028));
    P.add(mergeRaw(gussets), F.PAINT_DEEP, null, [1, 1]);
  }

  // ---- rear brake hose ------------------------------------------------------
  // It starts ON THE STEER AXIS (the cockpit hose hands over there, see
  // buildCockpit) so there is no gap at any steering angle, runs down the
  // non-drive side of the down tube through a cable port, and ends exactly on the
  // rear caliper's banjo — via the swingarm run, which continues it.
  const handover = new THREE.Vector3(0, 0.200, 0).applyMatrix4(SG.steerToChassis);
  const hose = sweptTube([
    V(handover.x, handover.y, handover.z),
    V(-0.014, htLow.y - 0.010, htLow.z + 0.030),
    V(-0.030, htLow.y - 0.098, htLow.z + 0.086),
    V(-0.034, 0.150, -0.070),
    V(-0.032, G.bb.y + 0.070, G.bb.z - 0.030),
    V(-0.036, G.bb.y + 0.030, G.bb.z + 0.024),
    V(-0.042, G.mainPivot.y + 0.006, G.mainPivot.z + 0.026),
  ], [{ t: 0, rx: 0.0042, ry: 0.0042 }, { t: 1, rx: 0.0040, ry: 0.0040 }], 8, 26, {});
  P.add(hose, F.HOSE, null, [1, 20]);
  // Cable ports / hose clips, so the hose is attached to something.
  const clips = [];
  for (const p of [[-0.031, htLow.y - 0.084, htLow.z + 0.074], [-0.034, 0.150, -0.070],
    [-0.035, G.bb.y + 0.052, G.bb.z - 0.006]]) {
    const cl = latheX([0.0058, -0.006, 0.0092, -0.004, 0.0092, 0.004, 0.0058, 0.006], 10, true, 2, 1);
    cl.rotateZ(Math.PI / 2);
    cl.rotateX(0.9);
    cl.translate(p[0] + 0.004, p[1], p[2]);
    clips.push(cl);
  }
  P.add(mergeRaw(clips), F.BLACK_ANO);
  void htMid; void rng;

  return P;
}

/** The carbon main tubes — a separate object only because they need a weave normal. */
function buildFrameCarbon(SG, q) {
  const P = new Parts();
  const sides = q >= 2 ? 18 : 12;
  const steps = q >= 2 ? 22 : 12;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  const htLow = new THREE.Vector3(0, G.crownLower + 0.030, G.forkOffset).applyMatrix4(SG.steerToChassis);
  const htTop = new THREE.Vector3(0, G.crownLower + 0.118, G.forkOffset).applyMatrix4(SG.steerToChassis);

  // Downtube. Round where it meets the head tube, then hydroformed out into a
  // wide flat oval at the BB — the section change is the whole reason this is a
  // swept tube instead of a cylinder.
  // The down tube is the biggest flat flank on the bike, so it carries the
  // wordmark: centred at 46 % along the tube, 0.30 of its length, and 0.20 of the
  // circumference (≈ ±36° either side of each flank centre, which keeps the band
  // well clear of the top/bottom seams where the orientation signs degenerate).
  P.add(sweptTube([
    V(0, htLow.y - 0.006, htLow.z + 0.024),
    V(0, htLow.y - 0.120, htLow.z + 0.086),
    V(0, 0.155, -0.078),
    V(0, G.bb.y + 0.052, G.bb.z - 0.040),
    V(0, G.bb.y + 0.004, G.bb.z + 0.006),
  ], [
    { t: 0.00, rx: 0.0290, ry: 0.0275 },   // round at the head tube…
    { t: 0.22, rx: 0.0330, ry: 0.0315 },
    { t: 0.62, rx: 0.0400, ry: 0.0350 },   // …deep and wide through the middle…
    { t: 0.88, rx: 0.0425, ry: 0.0320 },
    { t: 1.00, rx: 0.0380, ry: 0.0285 },   // …and pinched again at the BB.
  ], sides, steps, { capStart: false, capEnd: false }), F.PAINT, null, [1.5, 4.2], {
    decal: Object.assign({ N: steps, sides }, DECAL_RECT.DOWNTUBE),
  });

  // Top tube, sloping hard down to the seat-tube junction, as a DH bike does.
  P.add(sweptTube([
    V(0, htTop.y - 0.002, htTop.z + 0.022),
    V(0, htTop.y - 0.052, htTop.z + 0.112),
    V(0, 0.480, 0.006),
    V(0, G.seatTubeTop.y + 0.032, G.seatTubeTop.z - 0.030),
    V(0, G.seatTubeTop.y + 0.016, G.seatTubeTop.z + 0.004),
  ], [
    { t: 0.00, rx: 0.0245, ry: 0.0225 },
    { t: 0.30, rx: 0.0295, ry: 0.0250 },
    { t: 0.75, rx: 0.0265, ry: 0.0235 },
    { t: 1.00, rx: 0.0215, ry: 0.0200 },
  ], sides, steps, { capStart: false, capEnd: false }), F.PAINT, null, [1.4, 3.4], {
    decal: Object.assign({ N: steps, sides }, DECAL_RECT.TOPTUBE),
  });

  // Head tube gusset — a real formed panel spanning the head tube and the down
  // tube, in the deep colourway so the frame has two values rather than one.
  P.add(gussetPlate(
    V(0, htLow.y + 0.012, htLow.z + 0.020),
    V(0, htLow.y - 0.118, htLow.z + 0.104),
    V(0, htLow.y - 0.048, htLow.z + 0.006), 0.026), F.PAINT_DEEP, null, [0.8, 0.8]);
  // Seat-tube / top-tube junction plate.
  P.add(gussetPlate(
    V(0, G.seatTubeTop.y + 0.046, G.seatTubeTop.z - 0.052),
    V(0, G.seatTubeTop.y + 0.030, G.seatTubeTop.z + 0.006),
    V(0, G.seatTubeTop.y - 0.036, G.seatTubeTop.z - 0.010), 0.024), F.PAINT_DEEP, null, [0.8, 0.8]);

  // Downtube guard (chunky, matte, protects from rock strikes).
  P.add(sweptTube([
    V(0, 0.075, -0.055),
    V(0, G.bb.y + 0.050, G.bb.z - 0.055),
    V(0, G.bb.y + 0.006, G.bb.z - 0.008),
  ], [
    { t: 0.00, rx: 0.0100, ry: 0.0480 },   // a wide, thin bash plate
    { t: 0.55, rx: 0.0125, ry: 0.0560 },
    { t: 1.00, rx: 0.0110, ry: 0.0500 },
  ], 12, 10, {}), F.CARBON_GLS, null, [1.2, 1.6]);

  return P;
}

/** Saddle shell: a lofted DH perch, narrow nose, flared tail. */
function buildSaddle(q) {
  const n = q >= 2 ? 20 : 12;
  const halfLen = G.saddleLen * 0.5;
  const pos = [], uv = [], idx = [];
  const cols = 9;
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    const z = lerp(-halfLen, halfLen, t);
    // Width: pinched at the nose, wide at the tail.
    const w = 0.014 + 0.106 * Math.pow(clamp01((t - 0.06) / 0.94), 1.7);
    const drop = 0.014 * Math.pow(1 - t, 2.2) - 0.026 * Math.pow(clamp01((t - 0.58) / 0.42), 2.2);
    for (let j = 0; j < cols; j++) {
      const s = (j / (cols - 1)) * 2 - 1;                 // −1..1 across
      const x = s * w;
      const y = -Math.pow(Math.abs(s), 2.2) * 0.020 - drop + 0.011;
      pos.push(x, y, z);
      uv.push(j / (cols - 1), t * 3);
    }
  }
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < cols - 1; j++) {
      const a = i * cols + j, b = a + 1, c = a + cols, d = c + 1;
      idx.push(a, c, b, b, c, d);
    }
  }
  // Underside: mirror the shell down a little so it is a solid, not a sheet.
  const base = pos.length / 3;
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    const z = lerp(-halfLen, halfLen, t);
    const w = 0.010 + 0.098 * Math.pow(clamp01((t - 0.06) / 0.94), 1.7);
    const drop = 0.014 * Math.pow(1 - t, 2.2) - 0.026 * Math.pow(clamp01((t - 0.58) / 0.42), 2.2);
    for (let j = 0; j < cols; j++) {
      const s = (j / (cols - 1)) * 2 - 1;
      pos.push(s * w, -Math.pow(Math.abs(s), 2.2) * 0.013 - drop - 0.028, z);
      uv.push(j / (cols - 1), t * 3);
    }
  }
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < cols - 1; j++) {
      const a = base + i * cols + j, b = a + 1, c = a + cols, d = c + 1;
      idx.push(a, b, c, b, d, c);
    }
  }
  // F5: close the perimeter. The shell and its underside were two open sheets, so
  // from any grazing side angle — which is exactly the angle a chase camera gives
  // you — you looked through the saddle's edge into its interior.
  const T = (i, j) => i * cols + j;
  const B = (i, j) => base + i * cols + j;
  for (let i = 0; i < n; i++) {
    // right flank (+x), outward normal +x
    idx.push(T(i, cols - 1), T(i + 1, cols - 1), B(i, cols - 1));
    idx.push(T(i + 1, cols - 1), B(i + 1, cols - 1), B(i, cols - 1));
    // left flank (−x)
    idx.push(T(i, 0), B(i, 0), T(i + 1, 0));
    idx.push(T(i + 1, 0), B(i, 0), B(i + 1, 0));
  }
  for (let j = 0; j < cols - 1; j++) {
    idx.push(T(0, j), T(0, j + 1), B(0, j));            // nose, outward −z
    idx.push(T(0, j + 1), B(0, j + 1), B(0, j));
    idx.push(T(n, j), B(n, j), T(n, j + 1));            // tail, outward +z
    idx.push(T(n, j + 1), B(n, j), B(n, j + 1));
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  return geo;
}

// =============================================================================
// FORK — dual crown. Built in the STEER frame: origin at the chassis-side strut
// mount, +Y up the steerer, +Z backward. The stanchions sit exactly on the strut
// axis (so the drawn axle is the simulated axle) and the steerer/head tube are
// offset 48 mm behind them, which is what fork offset physically is.
// =============================================================================
function buildForkUpper(q) {
  const P = new Parts();
  const sides = q >= 2 ? 16 : 10;

  // ---- crowns ---------------------------------------------------------------
  const crown = (y, halfW, thick) => {
    const parts = [];
    const plate = taperBox(halfW * 2, thick, 0.052, 0.94, 0.86);
    plate.translate(0, y - thick * 0.5, 0.010);
    parts.push(plate);
    // Collars where the stanchions pass through, and the steerer boss.
    for (const s of [-1, 1]) {
      const c = latheX([0.0210, -thick * 0.62, 0.0290, -thick * 0.52, 0.0290, thick * 0.52, 0.0210, thick * 0.62], sides, true, 4, 1);
      c.rotateZ(Math.PI / 2);
      c.translate(s * G.legX, y, 0);
      parts.push(c);
    }
    const sb = latheX([0.0185, -thick * 0.62, 0.0250, -thick * 0.5, 0.0250, thick * 0.5, 0.0185, thick * 0.62], sides, true, 4, 1);
    sb.rotateZ(Math.PI / 2);
    sb.translate(0, y, G.forkOffset);
    parts.push(sb);
    // Web joining the collars to the steerer boss.
    const web = taperBox(G.legX * 2 + 0.020, thick * 0.9, G.forkOffset + 0.030, 0.98, 0.9);
    web.translate(0, y - thick * 0.45, G.forkOffset * 0.5);
    parts.push(web);
    return mergeRaw(parts);
  };
  P.add(crown(G.crownLower, 0.098, 0.030), F.ALU_MACH, null, [2, 2]);
  P.add(crown(G.crownUpper, 0.088, 0.026), F.ALU_MACH, null, [2, 2]);

  // ---- steerer --------------------------------------------------------------
  const steerer = latheX([
    0.0180, G.crownLower - 0.020, 0.0180, G.crownUpper + 0.060, 0.0150, G.crownUpper + 0.062,
  ], sides, false, 6, 4);
  steerer.rotateZ(Math.PI / 2);
  steerer.translate(0, 0, G.forkOffset);
  P.add(steerer, F.STEEL, null, [1, 1]);

  // ---- stanchions -----------------------------------------------------------
  // F1: these measured L=232 on r3_12 against a sunlit trail at L=66 — the
  // brightest object in the frame was a 40 mm tube. Hard-anodised, not chrome.
  const stTop = G.crownLower + 0.004;
  const stBot = G.crownLower - G.stanchionLen;
  for (const s of [-1, 1]) {
    const st = latheX([
      G.stanchionR, stBot, G.stanchionR, stTop, G.stanchionR - 0.003, stTop + 0.004,
    ], sides + 6, false, 4, 6);
    st.rotateZ(Math.PI / 2);
    st.translate(s * G.legX, 0, 0);
    P.add(st, F.STANCHION, null, [1, 1]);
  }

  // ---- fork-top adjusters ---------------------------------------------------
  for (const s of [-1, 1]) {
    const cap = latheX([0.0000, G.crownUpper + 0.020, 0.0180, G.crownUpper + 0.020, 0.0180, G.crownUpper + 0.006], 14, false, 4, 1);
    cap.rotateZ(Math.PI / 2);
    cap.translate(s * G.legX, 0, 0);
    P.add(cap, s > 0 ? F.ANO_GOLD : F.ANO, null, [1, 1]);
  }

  // ---- brake hoses, upper runs ----------------------------------------------
  // F3. Both hoses leave their master cylinder in the COCKPIT part and hand over
  // to these on the steer axis at (0, 0.200, 0) — a point that does not move when
  // you steer, so the join can never open a gap. From there the front hose runs
  // down the left of the crowns to meet the run clipped to the fork lower, and
  // the rear hose carries on into the frame (buildFrameAlu picks it up at the
  // same handover point, transformed into chassis space).
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  // It ends at y = 0.140, which is where the fork-lower's own hose run tops out at
  // FULL EXTENSION; through the rest of the travel the two simply overlap by up to
  // 170 mm and read as the service loop a real fork carries there. They are 10 mm
  // apart in z so the overlap is a doubled hose, not coincident geometry.
  P.add(sweptTube([
    V(0, 0.200, 0),
    V(-0.030, 0.206, 0.026),
    V(-0.066, 0.200, 0.034),
    V(-0.088, 0.180, 0.026),
    V(-0.092, 0.140, 0.020),
  ], [{ t: 0, rx: 0.0042, ry: 0.0042 }, { t: 1, rx: 0.0040, ry: 0.0040 }], 8, 18, {}),
  F.HOSE, null, [1, 10]);
  // A guide on the lower crown so the hose is visibly held by something.
  const guide = latheX([0.0060, -0.007, 0.0100, -0.005, 0.0100, 0.005, 0.0060, 0.007], 10, true, 2, 1);
  guide.rotateZ(Math.PI / 2);
  guide.translate(-0.090, G.crownLower - 0.006, 0.022);
  P.add(guide, F.BLACK_ANO);
  return P;
}

/**
 * Fork lowers, built with the ORIGIN AT THE AXLE, so the whole group simply
 * slides along the steer axis as the fork compresses and the stanchions visibly
 * disappear into it.
 */
function buildForkLower(q) {
  const P = new Parts();
  const sides = q >= 2 ? 16 : 10;

  const flare = G.lowerFlare;
  for (const s of [-1, 1]) {
    // The casting splays outboard as it descends. That flare is not decoration:
    // it is what puts the leg's inner wall outside both the 200 mm rotor plane
    // and the shoulder knobs of a 2.5" tyre.
    P.add(sweptTube([
      new THREE.Vector3(s * G.legX, G.lowerTop - 0.004, 0),
      new THREE.Vector3(s * (G.legX + flare * 0.30), G.lowerTop * 0.62, 0.001),
      new THREE.Vector3(s * (G.legX + flare * 0.82), G.lowerTop * 0.24, 0.003),
      new THREE.Vector3(s * (G.legX + flare), -0.026, 0.002),
    ], [
      // ry is the LATERAL half-width here, and it is the number that decides
      // whether the casting fouls the 200 mm rotor. Keep it ≤ 0.026.
      { t: 0.00, rx: 0.0270, ry: 0.0250 },
      { t: 0.40, rx: 0.0300, ry: 0.0252 },
      { t: 0.78, rx: 0.0335, ry: 0.0255 },
      { t: 1.00, rx: 0.0315, ry: 0.0235 },
    ], sides, q >= 2 ? 16 : 9, {}), F.LOWER, null, [1, 6]);
    // Wiper seal + dust cap, concentric with the stanchion.
    const seal = latheX([
      0.0262, G.lowerTop - 0.014, 0.0282, G.lowerTop - 0.008, 0.0282, G.lowerTop + 0.004,
      0.0230, G.lowerTop + 0.009, 0.0206, G.lowerTop + 0.009,
    ], sides, false, 4, 1);
    seal.rotateZ(Math.PI / 2);
    seal.translate(s * G.legX, 0, 0);
    P.add(seal, F.SEAL, null, [1, 1]);
    // Dropout / axle boss.
    const drop = taperBox(0.028, 0.068, 0.050, 0.85, 0.75);
    drop.translate(s * (G.legX + flare), -0.033, 0.000);
    P.add(drop, F.LOWER, null, [2, 2]);
  }

  // Arch tying the legs together. It has to sweep back a long way: anything
  // within 370 mm of the axle on the centreline is inside the tyre.
  P.add(sweptTube([
    new THREE.Vector3(-(G.legX + flare * 0.35), 0.268, 0.010),
    new THREE.Vector3(-0.052, 0.386, 0.046),
    new THREE.Vector3(0, 0.399, 0.054),
    new THREE.Vector3(0.052, 0.386, 0.046),
    new THREE.Vector3(G.legX + flare * 0.35, 0.268, 0.010),
  ], [
    { t: 0.00, rx: 0.0240, ry: 0.0225 },
    { t: 0.50, rx: 0.0165, ry: 0.0235 },
    { t: 1.00, rx: 0.0240, ry: 0.0225 },
  ], 12, 20, {}), F.LOWER, null, [2, 3]);

  // Through-axle, 20 mm, with a pinch-bolt collar.
  const axle = latheX([
    0.0102, -0.082, 0.0102, 0.082, 0.0140, 0.082, 0.0140, 0.074,
  ], sides, false, 4, 1);
  P.add(axle, F.BLACK_ANO, null, [1, 1]);

  // Post-mount brake tabs on the left leg.
  const tab = taperBox(0.026, 0.052, 0.038, 0.90, 0.90);
  tab.rotateX(0.25);
  tab.translate(-(G.legX + flare * 0.8) - 0.004, 0.060, 0.030);
  P.add(tab, F.LOWER);

  // ---- front brake hose -----------------------------------------------------
  // F3. This used to stop at (−0.106, 0.090, 0.036) — 59 mm short of the caliper's
  // banjo and floating in mid air. The caliper is a child of THIS group (it rides
  // the lowers), so the attachment can be, and now is, exact: buildCaliper puts
  // the banjo at local (−0.0615, 0.118, −0.024), and caliperFront.rotation.x =
  // 0.78 carries it to (−0.0615, 0.1008, 0.0659) in this frame.
  P.add(sweptTube([
    new THREE.Vector3(-0.092, G.lowerTop + 0.220, 0.030),
    new THREE.Vector3(-0.098, G.lowerTop + 0.120, 0.056),
    new THREE.Vector3(-0.100, G.lowerTop + 0.020, 0.036),
    new THREE.Vector3(-0.102, 0.300, 0.030),
    new THREE.Vector3(-0.100, 0.190, 0.038),
    new THREE.Vector3(-0.086, 0.132, 0.056),
    new THREE.Vector3(-0.0615, 0.1050, 0.0680),   // onto the banjo
  ], [{ t: 0, rx: 0.0042, ry: 0.0042 }, { t: 0.88, rx: 0.0040, ry: 0.0040 },
    { t: 1, rx: 0.0046, ry: 0.0046 }], 8, 24, {}), F.HOSE, null, [1, 14]);
  // Two hose clips on the leg — a hose that is not held by anything reads as a
  // stray line, which is most of why the old one looked wrong even where it lay
  // in the right place.
  const hclips = [];
  for (const y of [G.lowerTop - 0.010, 0.230]) {
    const cl = latheX([0.0058, -0.006, 0.0094, -0.004, 0.0094, 0.004, 0.0058, 0.006], 10, true, 2, 1);
    cl.rotateZ(Math.PI / 2);
    cl.translate(-0.093, y, 0.034);
    hclips.push(cl);
  }
  P.add(mergeRaw(hclips), F.BLACK_ANO);

  return P;
}

// =============================================================================
// COCKPIT — 800 mm riser bar, stem, grips, levers. Built in the steer frame.
// Morph 0 = front lever pulled, morph 1 = rear lever pulled.
// =============================================================================
function buildCockpit(q) {
  const P = new Parts();
  const sides = q >= 2 ? 14 : 9;
  const barY = G.barAxis, barZ = G.forkOffset - 0.050;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);

  // ---- stem: direct-mount, clamping the steerer and reaching forward --------
  const stem = sweptTube([
    V(0, barY - 0.052, G.forkOffset + 0.002),
    V(0, barY - 0.022, G.forkOffset - 0.020),
    V(0, barY - 0.002, barZ + 0.006),
  ], [
    { t: 0.00, rx: 0.0250, ry: 0.0230 },
    { t: 0.55, rx: 0.0260, ry: 0.0200 },
    { t: 1.00, rx: 0.0230, ry: 0.0250 },
  ], 12, 10, {});
  P.add(stem, F.ALU_MACH, null, [2, 2]);
  const face = taperBox(0.062, 0.020, 0.046, 0.95, 0.9);
  face.rotateX(Math.PI / 2);
  face.translate(0, barY, barZ - 0.014);
  P.add(face, F.BLACK_ANO);

  // ---- handlebar: 800 mm, 28 mm rise, 8° back / 5° up -----------------------
  const hw = G.barWidth * 0.5;
  const barPts = [
    V(-hw, barY + G.barRise, barZ + G.barSweep),
    V(-hw * 0.74, barY + G.barRise * 0.95, barZ + G.barSweep * 0.86),
    V(-hw * 0.42, barY + G.barRise * 0.42, barZ + G.barSweep * 0.34),
    V(-hw * 0.22, barY + G.barRise * 0.06, barZ + 0.003),
    V(0, barY, barZ),
    V(hw * 0.22, barY + G.barRise * 0.06, barZ + 0.003),
    V(hw * 0.42, barY + G.barRise * 0.42, barZ + G.barSweep * 0.34),
    V(hw * 0.74, barY + G.barRise * 0.95, barZ + G.barSweep * 0.86),
    V(hw, barY + G.barRise, barZ + G.barSweep),
  ];
  P.add(sweptTube(barPts, [
    { t: 0.00, rx: 0.0111, ry: 0.0111 },
    { t: 0.16, rx: 0.0116, ry: 0.0116 },
    { t: 0.40, rx: 0.0159, ry: 0.0159 },
    { t: 0.60, rx: 0.0159, ry: 0.0159 },
    { t: 0.84, rx: 0.0116, ry: 0.0116 },
    { t: 1.00, rx: 0.0111, ry: 0.0111 },
  ], sides, q >= 2 ? 60 : 34, { up: new THREE.Vector3(0, 1, 0) }), F.ALU_RAW, null, [2, 14]);

  // The bar curve, resampled so grips/levers can be planted on it exactly.
  const barCurve = new THREE.CatmullRomCurve3(barPts, false, 'centripetal', 0.5);
  const atX = (x) => {
    // Monotone in x, so a short bisection is exact and cheap.
    let lo = 0, hi = 1;
    for (let i = 0; i < 24; i++) {
      const mid = (lo + hi) * 0.5;
      if (barCurve.getPoint(mid).x < x) lo = mid; else hi = mid;
    }
    return barCurve.getPoint((lo + hi) * 0.5);
  };

  // ---- grips: ribbed rubber lock-ons ---------------------------------------
  const gripAnchors = {};
  for (const s of [-1, 1]) {
    const a = atX(s * G.gripInner), b = atX(s * G.gripOuter);
    const dir = new THREE.Vector3().copy(b).sub(a);
    const len = dir.length();
    dir.multiplyScalar(1 / len);
    // Ribbed profile: a real grip is not a smooth tube, and the ribs catch light.
    const prof = [];
    const ribs = 11;
    prof.push(0.0116, 0.0);
    for (let i = 0; i < ribs; i++) {
      const t0 = 0.012 + (i / ribs) * (len - 0.028);
      prof.push(0.0142, t0, 0.0158, t0 + 0.0042, 0.0158, t0 + 0.0074, 0.0142, t0 + 0.0112);
    }
    prof.push(0.0150, len - 0.012, 0.0128, len - 0.001, 0.0090, len);
    const grip = latheX(prof, sides + 4, false, 3, 1);
    grip.rotateZ(Math.PI / 2);          // profile axis becomes +Y
    const qq = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    grip.applyMatrix4(new THREE.Matrix4().compose(a, qq, new THREE.Vector3(1, 1, 1)));
    P.add(grip, F.GRIP, null, [1, 1]);
    for (const e of [0.004, len - 0.010]) {
      const col = latheX([0.0150, 0, 0.0150, 0.007], 14, false, 3, 1);
      col.rotateZ(Math.PI / 2);
      const at = new THREE.Vector3().copy(dir).multiplyScalar(e).add(a);
      col.applyMatrix4(new THREE.Matrix4().compose(at, qq, new THREE.Vector3(1, 1, 1)));
      P.add(col, s > 0 ? F.ANO_GOLD : F.ANO, null, [1, 1]);
    }
    gripAnchors[s > 0 ? 'R' : 'L'] = b.clone();
  }

  // ---- brake levers ---------------------------------------------------------
  // Master cylinder inboard of the grip; the blade sweeps out over the grip and
  // rotates about the clamp when pulled — as a morph target, so it is free.
  const leverRanges = { R: null, L: null };
  const leverPivots = { R: new THREE.Vector3(), L: new THREE.Vector3() };
  for (const s of [-1, 1]) {
    const key = s > 0 ? 'R' : 'L';
    const clampPt = atX(s * (G.gripInner - 0.030));
    const body = taperBox(0.034, 0.052, 0.062, 0.85, 0.9);
    body.rotateZ(s * 0.25);
    body.rotateX(-0.30);
    body.translate(clampPt.x, clampPt.y - 0.012, clampPt.z + 0.006);
    P.add(body, F.BLACK_ANO, null, [2, 2]);
    const res = latheX([0.0000, 0.006, 0.0110, 0.006, 0.0110, 0.000], 12, false, 2, 1);
    res.rotateZ(Math.PI / 2);
    res.rotateX(-0.35);
    res.translate(clampPt.x + s * 0.004, clampPt.y + 0.020, clampPt.z + 0.014);
    P.add(res, F.ALU_MACH);
    const cl = latheX([0.0165, -0.009, 0.0195, -0.006, 0.0195, 0.006, 0.0165, 0.009], 14, true, 3, 1);
    cl.rotateZ(Math.PI / 2);
    cl.translate(clampPt.x, clampPt.y, clampPt.z);
    P.add(cl, F.ALU_MACH);
    // Clamp bolt, because a lever that is not bolted to the bar reads as a box.
    const bolt = latheX([0.0000, 0, 0.0032, 0.0015, 0.0032, 0.010, 0.0000, 0.011], 8, false, 2, 1);
    bolt.rotateZ(Math.PI / 2);
    bolt.rotateY(Math.PI / 2);
    bolt.translate(clampPt.x, clampPt.y - 0.014, clampPt.z + 0.020);
    P.add(bolt, F.STEEL);

    // ---- blade ---------------------------------------------------------------
    // F3. The blade WAS here, authored rx 0.0032 / ry 0.0105 — for a curve running
    // along ±X that is 6.4 mm fore/aft by 21 mm TALL, i.e. a fin standing on its
    // edge, in bright ALU_MACH, against a bright bar. From the chase camera you
    // saw two grey clamp boxes and no lever. A real blade is the other way round:
    // ~19 mm of finger surface fore/aft, ~5 mm thick, angled down and swept back,
    // and dark against the bar so its silhouette exists at all.
    const p0 = clampPt.clone().add(new THREE.Vector3(s * 0.014, -0.019, -0.010));
    const blade = sweptTube([
      p0,
      p0.clone().add(new THREE.Vector3(s * 0.026, -0.008, -0.020)),
      p0.clone().add(new THREE.Vector3(s * 0.054, -0.005, -0.030)),
      p0.clone().add(new THREE.Vector3(s * 0.080, -0.001, -0.032)),
      p0.clone().add(new THREE.Vector3(s * 0.097, 0.006, -0.026)),   // hooked tip
    ], [
      { t: 0.00, rx: 0.0092, ry: 0.0040 },
      { t: 0.35, rx: 0.0098, ry: 0.0026 },
      { t: 0.80, rx: 0.0090, ry: 0.0024 },
      { t: 1.00, rx: 0.0062, ry: 0.0030 },
    ], 8, 14, {});
    leverRanges[key] = P.add(blade, F.BLACK_ANO, null, [2, 2]);
    leverPivots[key].copy(p0);
    // Reach adjuster dial + the pivot barrel the blade turns on.
    const piv = latheX([0.0000, 0, 0.0062, 0.0012, 0.0062, 0.013, 0.0000, 0.014], 10, false, 2, 1);
    piv.rotateZ(Math.PI / 2);
    piv.translate(p0.x - s * 0.006, p0.y, p0.z);
    P.add(piv, F.ALU_MACH);

    // ---- master-cylinder hose ------------------------------------------------
    // F3: the hose now starts ON the master cylinder and hands over to the fork /
    // frame runs at (0, 0.200, 0) — a point ON the steering axis, so the two ends
    // stay coincident at every steering angle. buildForkUpper picks it up there.
    const mc = new THREE.Vector3(clampPt.x - s * 0.006, clampPt.y - 0.030, clampPt.z + 0.024);
    P.add(sweptTube([
      mc,
      new THREE.Vector3(clampPt.x - s * 0.040, clampPt.y - 0.044, clampPt.z + 0.046),
      new THREE.Vector3(clampPt.x * 0.42, barY - 0.048, barZ + 0.062),
      new THREE.Vector3(s * 0.020, 0.234, 0.030),
      new THREE.Vector3(0, 0.200, 0),
    ], [{ t: 0, rx: 0.0044, ry: 0.0044 }, { t: 1, rx: 0.0040, ry: 0.0040 }], 8, 16, {}),
    F.HOSE, null, [1, 8]);
  }

  return { parts: P, gripAnchors, leverRanges, leverPivots, barCentre: new THREE.Vector3(0, barY, barZ) };
}

// =============================================================================
// REAR END — swingarm, seatstay, rocker, shock.
// =============================================================================

/**
 * Chainstays, built about the MAIN PIVOT with +Z running back to the dropout.
 * `axle` is the rest axle position in this frame; the group is stretched along Z
 * at runtime so the drawn dropout stays exactly on the simulated axle.
 */
function buildSwingarm(armLen, horst, axle, q) {
  const P = new Parts();
  const sides = q >= 2 ? 14 : 9;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  const halfX = 0.062;

  for (const s of [-1, 1]) {
    // The drive-side stay has to clear the chainring, so it kicks outboard.
    const bulge = s > 0 ? 0.020 : 0.006;
    P.add(sweptTube([
      V(s * 0.058, 0.006, 0.010),
      V(s * (halfX + bulge), 0.000, armLen * 0.30),
      V(s * (halfX + bulge * 0.4), horst.y * 0.7, armLen * 0.66),
      V(s * 0.070, horst.y, horst.z),
    ], [
      { t: 0.00, rx: 0.0290, ry: 0.0250 },
      { t: 0.30, rx: 0.0230, ry: 0.0190 },
      { t: 0.70, rx: 0.0200, ry: 0.0170 },
      { t: 1.00, rx: 0.0180, ry: 0.0180 },
    ], sides, q >= 2 ? 20 : 12, {}), F.PAINT, null, [2, 6], {
      // The chainstay flanks carry the small stay mark. Same projection machinery
      // as the down tube; a narrower band because the stay is a slimmer tube.
      decal: Object.assign({ N: q >= 2 ? 20 : 12, sides }, DECAL_RECT.STAY),
    });
  }
  const yoke = latheX([0.0180, -0.070, 0.0290, -0.058, 0.0290, 0.058, 0.0180, 0.070], sides, true, 4, 1);
  P.add(yoke, F.ALU_MACH);
  const bridge = sweptTube([
    V(-0.060, -0.006, armLen * 0.26),
    V(0, 0.004, armLen * 0.26),
    V(0.060, -0.006, armLen * 0.26),
  ], [{ t: 0, rx: 0.020, ry: 0.012 }, { t: 1, rx: 0.020, ry: 0.012 }], 10, 6, {});
  P.add(bridge, F.PAINT_DEEP);
  // Weld beads where the stays meet the main-pivot yoke.
  {
    const w = [];
    for (const s of [-1, 1]) {
      w.push(weldAt(new THREE.Vector3(s * 0.062, 0.004, 0.018), new THREE.Vector3(0, 0, 1), 0.0300, 0.0260, 0.0028, 12, q));
    }
    P.add(mergeRaw(w), F.WELD, null, [3, 1]);
  }
  // Horst-link pivot bosses — visibly a joint, because it visibly articulates.
  for (const s of [-1, 1]) {
    const b = latheX([0.0110, -0.016, 0.0175, -0.012, 0.0175, 0.012, 0.0110, 0.016], sides, true, 3, 1);
    b.translate(s * 0.070, horst.y, horst.z);
    P.add(b, F.ALU_MACH);
    // Dropout plate bridging the Horst pivot and the axle.
    const dy = axle.y - horst.y, dz = axle.z - horst.z;
    const dl = Math.hypot(dy, dz);
    const plate = taperBox(0.015, dl + 0.062, 0.058, 0.94, 0.78);
    plate.rotateX(Math.atan2(dz, dy));
    plate.translate(s * 0.070, horst.y - dy / dl * 0.026, horst.z - dz / dl * 0.026);
    P.add(plate, F.ALU_MACH, null, [2, 2]);
  }
  // Through-axle.
  const axleGeo = latheX([0.0090, -0.076, 0.0090, 0.076, 0.0128, 0.076, 0.0128, 0.068], 14, false, 3, 1);
  axleGeo.translate(0, axle.y, axle.z);
  P.add(axleGeo, F.BLACK_ANO);
  // ---- rear brake hose, chainstay run ---------------------------------------
  // F3. It used to stop at (−0.062, −0.070, 0.446) — 52 mm short of anything. The
  // rear caliper hangs off the AXLE carrier at rotation.x = −1.15, which puts its
  // banjo 118 mm forward of and 26 mm above the axle, i.e. at (−0.0615, −0.0187,
  // 0.4525) in this frame at sag. The swingarm scales along z with the arm, and
  // this point is 79 % along it, so the attachment holds to ~2.5 mm across the
  // whole 165 mm of travel — which is below the width of the hose itself.
  P.add(sweptTube([
    V(-0.052, 0.026, 0.010),
    V(-0.066, 0.008, armLen * 0.30),
    V(-0.066, -0.014, armLen * 0.62),
    V(-0.0615, -0.0187, 0.4525),
  ], [{ t: 0, rx: 0.0040, ry: 0.0040 }, { t: 0.9, rx: 0.0040, ry: 0.0040 },
    { t: 1, rx: 0.0046, ry: 0.0046 }], 7, 14, {}), F.HOSE, null, [1, 12]);
  const sclip = latheX([0.0058, -0.006, 0.0092, -0.004, 0.0092, 0.004, 0.0058, 0.006], 10, true, 2, 1);
  sclip.rotateZ(Math.PI / 2);
  sclip.translate(-0.060, 0.006, armLen * 0.34);
  P.add(sclip, F.BLACK_ANO);
  return P;
}

/**
 * The seatstay pair — Horst pivot up to the rocker. This link carries no axle:
 * in this model the chainstay carries the dropout (see the swingarm), which is a
 * 6 mm departure from a textbook Horst layout and buys an exact axle match with
 * the physics.
 */
function buildSeatstay(topLocal, q) {
  const P = new Parts();
  const sides = q >= 2 ? 12 : 8;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  for (const s of [-1, 1]) {
    P.add(sweptTube([
      V(s * 0.070, 0.004, -0.004),
      V(s * 0.066, topLocal.y * 0.34, topLocal.z * 0.34),
      V(s * 0.050, topLocal.y * 0.70, topLocal.z * 0.70),
      V(s * 0.026, topLocal.y * 0.92, topLocal.z * 0.92),
    ], [
      { t: 0.00, rx: 0.0165, ry: 0.0150 },
      { t: 0.45, rx: 0.0135, ry: 0.0175 },
      { t: 1.00, rx: 0.0120, ry: 0.0140 },
    ], sides, q >= 2 ? 16 : 10, {}), F.PAINT, null, [2, 6]);
  }
  P.add(sweptTube([
    V(-0.030, topLocal.y * 0.90, topLocal.z * 0.90),
    V(0, topLocal.y * 0.965, topLocal.z * 0.965),
    V(0.030, topLocal.y * 0.90, topLocal.z * 0.90),
  ], [{ t: 0, rx: 0.017, ry: 0.014 }, { t: 0.5, rx: 0.020, ry: 0.016 }, { t: 1, rx: 0.017, ry: 0.014 }], 10, 8, {}), F.PAINT_DEEP);
  const eye = latheX([0.0080, -0.021, 0.0140, -0.021, 0.0140, 0.021, 0.0080, 0.021], 14, true, 3, 1);
  eye.translate(0, topLocal.y, topLocal.z);
  P.add(eye, F.ALU_MACH);
  return P;
}

/** Rocker link — a carbon bell crank, built about its own pivot. */
function buildRocker(sLocal, eLocal, q) {
  const P = new Parts();
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  for (const s of [-1, 1]) {
    P.add(sweptTube([
      V(s * 0.026, eLocal.y, eLocal.z),
      V(s * 0.030, eLocal.y * 0.35, eLocal.z * 0.35),
      V(s * 0.032, 0, 0),
      V(s * 0.030, sLocal.y * 0.40, sLocal.z * 0.40),
      V(s * 0.026, sLocal.y, sLocal.z),
    ], [
      { t: 0.00, rx: 0.0155, ry: 0.0090 },
      { t: 0.45, rx: 0.0260, ry: 0.0125 },
      { t: 0.55, rx: 0.0260, ry: 0.0125 },
      { t: 1.00, rx: 0.0165, ry: 0.0090 },
    ], q >= 2 ? 12 : 8, q >= 2 ? 18 : 10, {}), F.CARBON, null, [2, 4]);
  }
  const bosses = [];
  for (const p of [{ y: 0, z: 0, r: 0.0175 }, { y: sLocal.y, z: sLocal.z, r: 0.0135 }, { y: eLocal.y, z: eLocal.z, r: 0.0125 }]) {
    const b = latheX([p.r * 0.5, -0.032, p.r, -0.026, p.r, 0.026, p.r * 0.5, 0.032], 14, true, 3, 1);
    b.translate(0, p.y, p.z);
    bosses.push(b);
  }
  P.add(mergeRaw(bosses), F.ALU_MACH);
  return P;
}

/** Coil shock body (with the spring); the shaft is a separate telescoping part. */
function buildShockBody(bodyLen, q) {
  const P = new Parts();
  const seg = q >= 2 ? 18 : 11;
  const body = latheX([
    0.0125, 0.000, 0.0135, 0.014, 0.0240, 0.026, 0.0245, bodyLen - 0.020,
    0.0210, bodyLen - 0.004, 0.0180, bodyLen,
  ], seg, false, 5, 3);
  body.rotateZ(Math.PI / 2);
  P.add(body, F.OIL, null, [1, 1]);
  const eye = latheX([0.0075, -0.016, 0.0135, -0.016, 0.0135, 0.016, 0.0075, 0.016], 14, true, 3, 1);
  P.add(eye, F.ALU_MACH);
  const res = latheX([0.0000, 0.000, 0.0165, 0.004, 0.0165, 0.072, 0.0000, 0.076], seg, false, 4, 2);
  res.rotateZ(Math.PI / 2);
  res.rotateX(0.30);
  res.translate(0.0, 0.030, 0.030);
  P.add(res, F.ANO_GOLD, null, [1, 1]);
  // Coil spring — a real helix. A cylinder here would be obvious at any distance.
  const coilPts = [];
  const turns = 9, rad = 0.0335;
  const springLen = bodyLen * 0.84;
  const coilSteps = turns * (q >= 2 ? 12 : 7);
  for (let i = 0; i <= coilSteps; i++) {
    const t = i / coilSteps;
    const a = t * turns * Math.PI * 2;
    coilPts.push(new THREE.Vector3(Math.sin(a) * rad, 0.022 + t * springLen, Math.cos(a) * rad));
  }
  P.add(sweptTube(coilPts, [{ t: 0, rx: 0.0048, ry: 0.0048 }, { t: 1, rx: 0.0048, ry: 0.0048 }],
    q >= 2 ? 8 : 6, coilSteps, {}), F.PAINT, null, [1, 30]);
  const collar = latheX([0.0250, 0, 0.0400, 0.004, 0.0400, 0.016, 0.0250, 0.020], seg, true, 4, 1);
  collar.rotateZ(Math.PI / 2);
  collar.translate(0, 0.014, 0);
  P.add(collar, F.ALU_MACH);
  return P;
}

function buildShockShaft(shaftLen) {
  const P = new Parts();
  const shaft = latheX([0.0080, -shaftLen, 0.0080, -0.014, 0.0125, -0.014, 0.0125, 0.000], 14, false, 3, 4);
  shaft.rotateZ(Math.PI / 2);
  P.add(shaft, F.STANCHION, null, [1, 1]);
  const eye = latheX([0.0075, -0.015, 0.0130, -0.015, 0.0130, 0.015, 0.0075, 0.015], 14, true, 3, 1);
  P.add(eye, F.ALU_MACH);
  return P;
}

// =============================================================================
// DRIVETRAIN
// =============================================================================

/** Cranks + chainring + spider + spindle, about the BB. At angle 0 the RIGHT
 *  crank points forward (−Z), and the group rotates about X. */
function buildCrankset(q) {
  const P = new Parts();
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  // Spindle.
  const spindle = latheX([0.0145, -0.078, 0.0145, 0.078], 14, false, 3, 1);
  P.add(spindle, F.STEEL);
  for (const s of [-1, 1]) {
    const dir = s > 0 ? -1 : 1;           // right crank forward, left crank back
    P.add(sweptTube([
      V(s * 0.058, 0, 0),
      V(s * (G.crankQ - 0.010), 0, dir * G.crankLen * 0.34),
      V(s * G.crankQ, 0, dir * G.crankLen * 0.80),
      V(s * G.crankQ, 0, dir * G.crankLen),
    ], [
      // rx = depth in profile, ry = lateral thickness (see sweptTube).
      { t: 0.00, rx: 0.0240, ry: 0.0150 },
      { t: 0.30, rx: 0.0200, ry: 0.0122 },
      { t: 0.80, rx: 0.0165, ry: 0.0108 },
      { t: 1.00, rx: 0.0145, ry: 0.0118 },
    ], q >= 2 ? 12 : 8, q >= 2 ? 14 : 8, {}), F.ALU_RAW, null, [2, 3]);
  }
  // Chainring + direct-mount spider.
  const rp = pitchRadius(G.chainringTeeth);
  const ring = gearGeo(0.050, rp - 0.0052, rp + 0.0046, G.chainringTeeth, 0.0042);
  ring.translate(G.chainline, 0, 0);
  P.add(ring, F.ANO, null, [3, 1]);
  const spider = latheX([0.0230, G.chainline - 0.008, 0.0300, G.chainline - 0.008, 0.0520, G.chainline - 0.001,
    0.0520, G.chainline + 0.003, 0.0250, G.chainline + 0.004], 20, false, 4, 1);
  P.add(spider, F.ALU_MACH);
  // Bash guard. Deliberately smaller than the ring's root circle (0.0635) so the
  // teeth still read — a guard that covers the ring just looks like a black disc.
  const bash = latheX([0.0400, G.chainline + 0.008, 0.0615, G.chainline + 0.010, 0.0615, G.chainline + 0.0145,
    0.0400, G.chainline + 0.0125], 26, true, 6, 1);
  P.add(bash, F.BLACK_MATT, null, [2, 1]);
  return P;
}

/** A flat pedal — platform, body and traction pins. Origin at the spindle. */
function buildPedal(side, q) {
  const P = new Parts();
  const s = side;
  const body = taperBox(0.098, 0.017, 0.104, 0.86, 0.94);
  body.translate(s * 0.032, -0.0085, 0);
  P.add(body, F.BLACK_ANO, null, [2, 2]);
  // Spindle stub and the outer end cap.
  const sp = latheX([0.0085, 0, 0.0085, s * 0.082, 0.0110, s * 0.082], 10, false, 3, 1);
  P.add(sp, F.STEEL);
  const pins = [];
  const rows = q >= 2 ? 5 : 3;
  for (let i = 0; i < rows; i++) {
    for (const zs of [-1, 1]) {
      for (const up of [-1, 1]) {
        const pin = new THREE.CylinderGeometry(0.0016, 0.0014, 0.0060, 5, 1);
        pin.translate(s * (-0.002 + i * 0.020), up * 0.0115, zs * 0.041);
        pins.push(pin);
      }
    }
  }
  P.add(mergeRaw(pins), F.ALU_MACH);
  return P;
}

/** Rear derailleur: b-knuckle, parallelogram, cage and both pulleys. Origin at
 *  the rear axle, so it rides with the wheel exactly as it does on a real bike. */
function buildDerailleur(guide, tension, q) {
  const P = new Parts();
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  const x = G.chainline;
  // B-knuckle bolted to the hanger.
  const knuckle = taperBox(0.022, 0.052, 0.030, 0.85, 0.85);
  knuckle.rotateX(0.5);
  knuckle.translate(x + 0.020, -0.020, 0.016);
  P.add(knuckle, F.BLACK_ANO, null, [2, 2]);
  // Parallelogram body.
  const body = sweptTube([
    V(x + 0.024, -0.036, 0.024),
    V(x + 0.016, -0.056, 0.020),
    V(x + 0.004, guide.y + 0.012, guide.z + 0.006),
  ], [
    { t: 0, rx: 0.0150, ry: 0.0130 },
    { t: 1, rx: 0.0125, ry: 0.0115 },
  ], 10, 8, {});
  P.add(body, F.BLACK_ANO, null, [2, 2]);
  // Cage plates, one each side of the pulleys.
  for (const s of [-1, 1]) {
    const plate = sweptTube([
      V(x + s * 0.011, guide.y, guide.z),
      V(x + s * 0.011, (guide.y + tension.y) * 0.5, (guide.z + tension.z) * 0.5),
      V(x + s * 0.011, tension.y, tension.z),
    ], [
      { t: 0, rx: 0.0180, ry: 0.0030 },
      { t: 0.5, rx: 0.0115, ry: 0.0028 },
      { t: 1, rx: 0.0170, ry: 0.0030 },
    ], 8, 8, {});
    P.add(plate, F.CARBON_GLS, null, [2, 2]);
  }
  // Pulleys.
  const pr = pitchRadius(G.pulleyTeeth);
  for (const p of [guide, tension]) {
    const g = gearGeo(0.0060, pr - 0.0040, pr + 0.0034, G.pulleyTeeth, 0.0055);
    g.translate(x, p.y, p.z);
    P.add(g, F.BLACK_MATT, null, [2, 1]);
    const hubg = latheX([0.0000, x - 0.008, 0.0075, x - 0.008, 0.0075, x + 0.008, 0.0000, x + 0.008], 12, false, 3, 1);
    hubg.translate(0, p.y, p.z);
    P.add(hubg, F.ALU_MACH);
  }
  void q;
  return P;
}

/** One chain link: two outer plates and a roller. Long axis +Z, plates ±X. */
function buildChainLink() {
  const P = new Parts();
  const half = G.chainPitch * 0.5;
  for (const s of [-1, 1]) {
    // Plate waisted in the middle, like a real link.
    const plate = sweptTube([
      new THREE.Vector3(s * 0.0029, 0, -half - 0.0018),
      new THREE.Vector3(s * 0.0029, 0, 0),
      new THREE.Vector3(s * 0.0029, 0, half + 0.0018),
    ], [
      { t: 0.00, rx: 0.0046, ry: 0.0011 },
      { t: 0.50, rx: 0.0034, ry: 0.0011 },
      { t: 1.00, rx: 0.0046, ry: 0.0011 },
    ], 6, 4, {});
    // F4: 1.1 mm plates. Same screen-space minimum-width treatment as the spokes —
    // `transformedNormal` carries the instanceMatrix rotation in three's default
    // normal chunk, so this is correct per chain link, not just per mesh.
    P.add(plate, F.CHAIN, null, null, { thin: 0.0011 });
  }
  const roller = latheX([0.0037, -0.0030, 0.0037, 0.0030], 8, false, 2, 1);
  roller.translate(0, 0, -half);
  P.add(roller, F.BLACK_MATT);
  const geo = P.build();
  // The roller lathe revolves about X, which is already the pin axis. Good.
  return geo;
}

// =============================================================================
// BRAKES — four-piston caliper. Built with +Y pointing radially outward from the
// wheel centre and the body centred at `radius`, so mounting is one rotateX.
// Morph 0 squeezes the pads onto the rotor.
// =============================================================================
function buildCaliper(radius, q) {
  const P = new Parts();
  const x = -G.rotorX;
  // Two body halves straddling the rotor, joined by a bridge.
  // taperBox is X = thickness, Y = radial (base at 0), Z = fore/aft.
  // Deliberately asymmetric: the inboard half is slimmer, because on a real bike
  // that face passes within a couple of millimetres of the spoke line and a
  // symmetric body would foul it.
  // s = +1 is the INBOARD half (towards the bike's centreline). Its inner face
  // has to stay outside the spoke line, which for a 3-cross wheel at this radius
  // sits at |x| ≈ 0.024 — hence the thinner, radially shorter inboard body.
  for (const s of [-1, 1]) {
    const inner = s > 0;
    const thick = inner ? 0.011 : 0.019;
    const rad = inner ? 0.044 : 0.058;
    const off = inner ? 0.0125 : 0.0170;
    const half = taperBox(thick, rad, inner ? 0.070 : 0.082, 0.92, 0.88);
    half.translate(x + s * off, radius - rad * 0.5, 0);
    P.add(half, F.ANO, null, [2, 2]);
  }
  // Narrow: the bridge is the part that would otherwise reach in past the spokes.
  const bridge = taperBox(0.031, 0.020, 0.068, 0.95, 0.9);
  bridge.translate(x - 0.003, radius + 0.026, 0);
  P.add(bridge, F.ANO_DEEP);
  // Banjo + hose stub.
  const banjo = latheX([0.0000, 0.0, 0.0090, 0.002, 0.0090, 0.012, 0.0000, 0.014], 12, false, 3, 1);
  banjo.rotateZ(Math.PI / 2);
  banjo.rotateX(-0.5);
  banjo.translate(x - 0.016, radius + 0.030, -0.024);
  P.add(banjo, F.ALU_MACH);
  // Bleed nipple.
  const bleed = latheX([0.0000, 0, 0.0035, 0.001, 0.0035, 0.010, 0.0000, 0.011], 8, false, 2, 1);
  bleed.rotateZ(Math.PI / 2);
  bleed.translate(x, radius + 0.038, 0.020);
  P.add(bleed, F.STEEL);

  // Pads — the only part that moves. Kept as the LAST parts added so the morph
  // range is contiguous.
  const padRanges = [];
  for (const s of [-1, 1]) {
    const pad = taperBox(0.0048, 0.026, 0.046, 1.0, 1.0);
    pad.translate(x + s * 0.0042, radius - 0.017, 0);
    padRanges.push({ range: P.add(pad, F.BLACK_MATT, null, [2, 2]), side: s });
  }
  void q;
  return { parts: P, padRanges };
}

// =============================================================================
// AMBIENT-OCCLUSION BAKE
// =============================================================================
// "Nothing is grounded" is one of the three headline findings, and the two
// mechanisms that would normally supply contact darkening are both off the table:
// the shadow cascade cannot resolve a 30 mm tube (Lane E owns that) and N8AO was
// deliberately weakened in round 2 and must not be re-raised (rejection R10). So
// the bike buys its own occlusion at BUILD time and pays nothing at runtime.
//
// Method: rasterise every part into a coarse occupancy grid, then march a small
// cosine-weighted hemisphere from each vertex and fold the result into the vertex
// colour. The grid is deliberately coarse (~35 mm cells) — this is a MACRO term,
// darkening the inside of the front triangle, the throat of the fork, the shadow
// side of the rim bed, behind the rotor. Micro-contact is the normal map's job.
//
// Cost measured on the shipped model: ~90 k vertices x 8 rays x 10 steps ≈ 7 M
// grid lookups, tens of milliseconds, once, at load.
const AO_DIRS = (() => {
  // 1 straight up the normal + 7 around a 55° cone: cheap, and biased toward the
  // normal, which is where a cosine-weighted integrator wants its samples.
  const d = [[0, 0, 1]];
  for (let i = 0; i < 7; i++) {
    const a = (i / 7) * Math.PI * 2 + 0.4;
    const e = 0.82;                             // sin(55°)
    d.push([Math.cos(a) * e, Math.sin(a) * e, Math.sqrt(Math.max(0, 1 - e * e))]);
  }
  return d;
})();

function makeOccupancy(items, res, pad) {
  const box = new THREE.Box3();
  const v = new THREE.Vector3();
  for (const it of items) {
    const p = it.geo.getAttribute('position');
    for (let i = 0; i < p.count; i++) {
      v.set(p.getX(i), p.getY(i), p.getZ(i));
      if (it.matrix) v.applyMatrix4(it.matrix);
      box.expandByPoint(v);
    }
  }
  if (box.isEmpty()) return null;
  box.expandByScalar(pad);
  const size = new THREE.Vector3().subVectors(box.max, box.min);
  const cell = Math.max(size.x, size.y, size.z) / res;
  const nx = Math.ceil(size.x / cell) + 1, ny = Math.ceil(size.y / cell) + 1, nz = Math.ceil(size.z / cell) + 1;
  const grid = new Uint8Array(nx * ny * nz);
  for (const it of items) {
    const p = it.geo.getAttribute('position');
    for (let i = 0; i < p.count; i++) {
      v.set(p.getX(i), p.getY(i), p.getZ(i));
      if (it.matrix) v.applyMatrix4(it.matrix);
      const ix = ((v.x - box.min.x) / cell) | 0, iy = ((v.y - box.min.y) / cell) | 0, iz = ((v.z - box.min.z) / cell) | 0;
      if (ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz) grid[(iz * ny + iy) * nx + ix] = 1;
    }
  }
  return { grid, nx, ny, nz, cell, min: box.min };
}

/** Mark a filled disc (a wheel) into an existing occupancy grid. */
function occupyDisc(occ, cx, cy, cz, radius, halfThick) {
  const { grid, nx, ny, nz, cell, min } = occ;
  const i0 = Math.max(0, (((cx - halfThick) - min.x) / cell) | 0);
  const i1 = Math.min(nx - 1, (((cx + halfThick) - min.x) / cell) | 0);
  for (let iz = 0; iz < nz; iz++) {
    const wz = min.z + (iz + 0.5) * cell - cz;
    for (let iy = 0; iy < ny; iy++) {
      const wy = min.y + (iy + 0.5) * cell - cy;
      if (wy * wy + wz * wz > radius * radius) continue;
      for (let ix = i0; ix <= i1; ix++) grid[(iz * ny + iy) * nx + ix] = 1;
    }
  }
}

/**
 * Fold occlusion into `color` for one geometry. `matrix` maps its vertices into
 * the same space the occupancy grid was built in.
 */
function bakeVertexAO(geo, matrix, occ, strength, steps, expo, floorV) {
  if (!occ) return;
  const p = geo.getAttribute('position');
  const n = geo.getAttribute('normal');
  const col = geo.getAttribute('color');
  if (!col) return;
  const { grid, nx, ny, nz, cell, min } = occ;
  const wp = new THREE.Vector3(), wn = new THREE.Vector3();
  const nm = matrix ? new THREE.Matrix3().getNormalMatrix(matrix) : null;
  const t1 = new THREE.Vector3(), t2 = new THREE.Vector3();
  const bias = cell * 1.6;      // start outside the surface's own occupied cells
  for (let i = 0; i < p.count; i++) {
    wp.set(p.getX(i), p.getY(i), p.getZ(i));
    wn.set(n.getX(i), n.getY(i), n.getZ(i));
    if (matrix) wp.applyMatrix4(matrix);
    if (nm) wn.applyMatrix3(nm);
    wn.normalize();
    // Tangent frame about the normal.
    t1.set(Math.abs(wn.x) < 0.9 ? 1 : 0, Math.abs(wn.x) < 0.9 ? 0 : 1, 0).cross(wn);
    if (t1.lengthSq() < 1e-8) t1.set(1, 0, 0);
    t1.normalize();
    t2.crossVectors(wn, t1);
    let occl = 0;
    for (let d = 0; d < AO_DIRS.length; d++) {
      const dd = AO_DIRS[d];
      const dx = t1.x * dd[0] + t2.x * dd[1] + wn.x * dd[2];
      const dy = t1.y * dd[0] + t2.y * dd[1] + wn.y * dd[2];
      const dz = t1.z * dd[0] + t2.z * dd[1] + wn.z * dd[2];
      const w = dd[2];                       // cosine weight
      for (let s = 1; s <= steps; s++) {
        const t = bias + s * cell * 1.15;
        const gx = ((wp.x + dx * t - min.x) / cell) | 0;
        const gy = ((wp.y + dy * t - min.y) / cell) | 0;
        const gz = ((wp.z + dz * t - min.z) / cell) | 0;
        if (gx < 0 || gx >= nx || gy < 0 || gy >= ny || gz < 0 || gz >= nz) break;
        if (grid[(gz * ny + gy) * nx + gx]) {
          // Nearer occluders darken more, exactly as a contact term should.
          occl += w * (1 - (s - 1) / steps);
          break;
        }
      }
    }
    let wsum = 0;
    for (let d = 0; d < AO_DIRS.length; d++) wsum += AO_DIRS[d][2];
    // The raw ratio is nearly linear in "how enclosed am I", which darkens the
    // WHOLE bike ~30 % — measured median 0.69 on the first pass, which is a dimmer
    // and not a contact term. The power curve pushes typical exposed surfaces back
    // to ~0.98 while leaving genuine cavities (inside the front triangle, the fork
    // throat, behind the rotor) down at 0.64.
    const ao = clamp(1 - Math.pow(clamp01(occl / wsum), expo) * strength, floorV, 1.0);
    col.setXYZ(i, col.getX(i) * ao, col.getY(i) * ao, col.getZ(i) * ao);
  }
  col.needsUpdate = true;
}

/**
 * The contact-shadow blob. A soft radial multiply under each wheel: an AO
 * darkening that respects the surface underneath rather than painting black on
 * it, which is what MultiplyBlending buys. Two instances, one draw call, and it
 * survives whatever Lane E does to the shadow rig because a cast shadow and a
 * contact-AO term are two different things.
 */
function buildContactBlobTexture(size) {
  const data = new Uint8Array(size * size * 4);
  const c = (size - 1) * 0.5;
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const dx = (x - c) / c, dy = (y - c) / c;
      const r = Math.sqrt(dx * dx + dy * dy);
      // `k` is how much light is REMOVED. It peaks at 0.42 under the contact
      // patch and reaches exactly zero at the rim, so the quad's own edge is
      // never visible. A blob that goes to black is a hole, not a shadow.
      const k = 0.42 * Math.pow(clamp01(1 - r), 1.8);
      const v = Math.round(255 * clamp01(1 - k));
      const o = (y * size + x) * 4;
      data[o] = data[o + 1] = data[o + 2] = v;
      data[o + 3] = 255;
    }
  }
  const tex = new THREE.DataTexture(data, size, size, THREE.RGBAFormat);
  tex.colorSpace = THREE.NoColorSpace;   // a multiply factor, not a colour
  tex.wrapS = tex.wrapT = THREE.ClampToEdgeWrapping;
  tex.magFilter = THREE.LinearFilter;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.generateMipmaps = true;
  tex.needsUpdate = true;
  return tex;
}

// =============================================================================
// SCRATCH — module scope. Nothing below allocates per frame.
// =============================================================================
const _fa = { y: 0, z: 0 };
const _ra = { y: 0, z: 0 };
const _H = { y: 0, z: 0 };
const _S = { y: 0, z: 0 };
const _E = { y: 0, z: 0 };
const _beltPt = { a: 0, b: 0, ta: 0, tb: 0 };
const _im = new THREE.Matrix4();
const _ix = new THREE.Vector3(1, 0, 0);
const _iy = new THREE.Vector3();
const _iz = new THREE.Vector3();
const _ipos = new THREE.Vector3();
// Contact-shadow scratch.
const _cbPos = new THREE.Vector3();
const _cbNrm = new THREE.Vector3();
const _cbQ = new THREE.Quaternion();
const _cbScale = new THREE.Vector3();
const _cbUp = new THREE.Vector3(0, 1, 0);
const _cbM = new THREE.Matrix4();
const _camPos = new THREE.Vector3();
const _bufSize = new THREE.Vector2();

const angleYZ = (y, z) => Math.atan2(y, z);

// =============================================================================
// FACTORY
// =============================================================================
export function createBikeModel(ctx) {
  const settings = (ctx && ctx.settings) || {};
  const qName = (ctx && ctx.quality) || 'high';
  const q = qName === 'ultra' ? 3 : qName === 'high' ? 2 : qName === 'medium' ? 1 : 0;
  const rng = makeRng(subSeed((ctx && ctx.seed) || 1, 'bikeModel'));
  const texSize = q >= 2 ? 512 : 256;

  // ---- materials ----------------------------------------------------------
  const metalMaps = buildMetalMaps(rng, texSize);
  const carbonMaps = buildCarbonMaps(rng, texSize);
  const decalMap = buildDecalAtlas(q >= 2 ? 1024 : 512);
  const anisotropy = (ctx && ctx.renderer && ctx.renderer.capabilities)
    ? Math.min(8, ctx.renderer.capabilities.getMaxAnisotropy()) : 4;
  for (const t of [metalMaps.normal, metalMaps.rough, carbonMaps.normal, carbonMaps.rough, decalMap]) t.anisotropy = anisotropy;
  // Uniform OBJECTS, shared by both programs, so one write per frame updates both.
  // uPx is pixels-per-metre at one metre of view depth: proj[1][1] * h/2.
  const shared = {
    uDecalMap: { value: decalMap },
    uPx: { value: 540 },
    uThinPx: { value: 0.75 },      // half-width → ~1.5 px rendered, per F4
  };
  const MAT_HARD = makeFinishMaterial(metalMaps, 0.30, shared);
  const MAT_CARBON = makeFinishMaterial(carbonMaps, 0.32, shared);

  // ---- derived suspension geometry ---------------------------------------
  const SG = deriveGeometry(ctx && ctx.bike ? ctx.bike.geometry : null);
  const bg = (ctx && ctx.bike && ctx.bike.geometry) || null;
  const bbY = -0.028;                                   // see the CONTRACT-NOTE
  const bbZ = bg && bg.bbFwd !== undefined ? -bg.bbFwd : G.bb.z;
  G.bb.y = bbY; G.bb.z = bbZ;

  // Rest positions and fixed link lengths for the four-bar.
  SG.rearAxle(SG.shock.sagT, _ra);
  const restAxle = { y: _ra.y, z: _ra.z };
  const armRest = { y: restAxle.y - G.mainPivot.y, z: restAxle.z - G.mainPivot.z };
  const armLen0 = Math.hypot(armRest.y, armRest.z);
  const phiArmRest = angleYZ(armRest.y, armRest.z);
  const horstLocal = { y: armRest.y + G.horstDown, z: armRest.z + G.horstBack };
  const horstRest = { y: horstLocal.y + G.mainPivot.y, z: horstLocal.z + G.mainPivot.z };
  const L_ss = Math.hypot(G.seatstayTop.y - horstRest.y, G.seatstayTop.z - horstRest.z);
  const L_rk = Math.hypot(G.seatstayTop.y - G.rockerPivot.y, G.seatstayTop.z - G.rockerPivot.z);
  const stayLocal = { y: G.seatstayTop.y - horstRest.y, z: G.seatstayTop.z - horstRest.z };
  const phiStayRest = angleYZ(stayLocal.y, stayLocal.z);
  const rockerSLocal = { y: G.seatstayTop.y - G.rockerPivot.y, z: G.seatstayTop.z - G.rockerPivot.z };
  const rockerELocal = { y: G.shockEye.y - G.rockerPivot.y, z: G.shockEye.z - G.rockerPivot.z };
  const phiRockerRest = angleYZ(rockerSLocal.y, rockerSLocal.z);
  // Shock: measure the extremes so the body/shaft split is right.
  let shockMax = 0;
  {
    // Full extension is the longest the shock ever is.
    SG.rearAxle(0, _ra);
    const phi = angleYZ(_ra.y - G.mainPivot.y, _ra.z - G.mainPivot.z);
    const th = phiArmRest - phi;
    const c = Math.cos(th), s = Math.sin(th);
    const hy = horstLocal.y * c - horstLocal.z * s + G.mainPivot.y;
    const hz = horstLocal.y * s + horstLocal.z * c + G.mainPivot.z;
    const hit = circleIntersect(hy, hz, L_ss, G.rockerPivot.y, G.rockerPivot.z, L_rk,
      G.seatstayTop.y, G.seatstayTop.z);
    if (hit.ok) {
      const ang = angleYZ(hit.y - G.rockerPivot.y, hit.z - G.rockerPivot.z) - phiRockerRest;
      const rc = Math.cos(-ang), rs = Math.sin(-ang);
      const ey = rockerELocal.y * rc - rockerELocal.z * rs + G.rockerPivot.y;
      const ez = rockerELocal.y * rs + rockerELocal.z * rc + G.rockerPivot.z;
      shockMax = Math.hypot(ey - G.shockMount.y, ez - G.shockMount.z);
    }
  }
  if (!(shockMax > 0.1)) shockMax = 0.251;
  const shockBodyLen = shockMax * 0.60;
  const shockShaftLen = shockMax * 0.46;

  // =========================================================================
  // ASSEMBLY
  // =========================================================================
  const group = new THREE.Group();
  group.name = 'bikeModel';
  const chassis = new THREE.Object3D();
  chassis.name = 'bikeChassis';
  group.add(chassis);

  const meshes = [];
  const geoms = [];
  // F6. Only these eight own the bike's shadow silhouette; the other eleven are
  // detail inside it. Casting from all nineteen bought nothing at ANY shadow
  // texel size (the gameplay cascade is 0.15 m/texel and even the tight slice
  // Lane E is adding would be ~4 mm) and cost eleven extra shadow draws plus two
  // morph-attribute depth variants for the brake calipers.
  const SHADOW_CASTERS = new Set([
    'frameAlu', 'frameCarbon', 'forkUpper', 'forkLower',
    'wheelFront', 'wheelRear', 'swingarm', 'cockpit',
  ]);
  const mkMesh = (geo, mat, name) => {
    const m = new THREE.Mesh(geo, mat);
    m.name = name;
    m.castShadow = !!settings.shadows && SHADOW_CASTERS.has(name);
    m.receiveShadow = !!settings.shadows;
    m.matrixAutoUpdate = true;
    meshes.push(m); geoms.push(geo);
    return m;
  };

  // ---- static frame -------------------------------------------------------
  const frameAluGeo = buildFrameAlu(SG, q, rng).build();
  const frameCarbGeo = buildFrameCarbon(SG, q).build();
  // Frame flex: the front of the frame deflects back and down under fork load.
  // Tiny (≤ 3.5 mm) and only visible as a life-like shimmer, which is the point.
  const flexFn = (p) => {
    const w = clamp01((-p.z - 0.05) / 0.42) * clamp01((p.y + 0.10) / 0.30);
    p.z += w * 0.0034;
    p.y -= w * 0.0022;
  };
  addMorph(frameAluGeo, flexFn);
  addMorph(frameCarbGeo, flexFn);
  const frameAlu = mkMesh(frameAluGeo, MAT_HARD, 'frameAlu');
  const frameCarbon = mkMesh(frameCarbGeo, MAT_CARBON, 'frameCarbon');
  chassis.add(frameAlu, frameCarbon);

  // ---- steering assembly --------------------------------------------------
  // steerPivot puts +Y on the steer axis; steerRot is the actual steering DOF.
  const steerPivot = new THREE.Object3D();
  steerPivot.position.copy(SG.steerOrigin);
  steerPivot.rotation.x = SG.fork.rake;
  chassis.add(steerPivot);
  const steerRot = new THREE.Object3D();
  steerPivot.add(steerRot);

  const forkUpper = mkMesh(buildForkUpper(q).build(), MAT_HARD, 'forkUpper');
  steerRot.add(forkUpper);

  const cockpitBuild = buildCockpit(q);
  const cockpitGeo = cockpitBuild.parts.build();
  // Lever pull as morph targets: rotate each blade about its clamp.
  const leverMorph = (key, sign) => {
    const r = cockpitBuild.leverRanges[key];
    const pivot = cockpitBuild.leverPivots[key];
    return (p, i) => {
      if (i < r.start || i >= r.start + r.count) return;
      // Rotate about the bar-ish axis through the pivot: pull the blade inboard
      // and back towards the grip.
      const dx = p.x - pivot.x, dz = p.z - pivot.z;
      const a = 0.30 * sign;
      p.x = pivot.x + dx * Math.cos(a) - dz * Math.sin(a);
      p.z = pivot.z + dx * Math.sin(a) + dz * Math.cos(a);
    };
  };
  addMorph(cockpitGeo, leverMorph('R', -1));   // right lever = front brake
  addMorph(cockpitGeo, leverMorph('L', 1));    // left lever  = rear brake
  const cockpit = mkMesh(cockpitGeo, MAT_HARD, 'cockpit');
  steerRot.add(cockpit);

  // Fork lowers hang off the steer axis and slide with travel.
  const forkLowerGroup = new THREE.Object3D();
  forkLowerGroup.name = 'forkLowers';
  steerRot.add(forkLowerGroup);
  const forkLower = mkMesh(buildForkLower(q).build(), MAT_HARD, 'forkLower');
  forkLowerGroup.add(forkLower);

  // ---- wheels -------------------------------------------------------------
  const wheelFront = mkMesh(buildWheel(false, q, rng), MAT_HARD, 'wheelFront');
  forkLowerGroup.add(wheelFront);

  const rearAxleCarrier = new THREE.Object3D();
  rearAxleCarrier.name = 'rearAxle';
  chassis.add(rearAxleCarrier);
  const wheelRear = mkMesh(buildWheel(true, q, rng), MAT_HARD, 'wheelRear');
  rearAxleCarrier.add(wheelRear);

  // ---- rear linkage -------------------------------------------------------
  const swingarmPivot = new THREE.Object3D();
  swingarmPivot.position.set(0, G.mainPivot.y, G.mainPivot.z);
  chassis.add(swingarmPivot);
  const swingarm = mkMesh(
    buildSwingarm(armLen0, horstLocal, armRest, q).build(), MAT_HARD, 'swingarm');
  swingarmPivot.add(swingarm);
  const seatstayPivot = new THREE.Object3D();
  chassis.add(seatstayPivot);
  const seatstay = mkMesh(
    buildSeatstay(stayLocal, q).build(), MAT_HARD, 'seatstay');
  seatstayPivot.add(seatstay);

  const rockerPivot = new THREE.Object3D();
  rockerPivot.position.set(0, G.rockerPivot.y, G.rockerPivot.z);
  chassis.add(rockerPivot);
  const rocker = mkMesh(buildRocker(rockerSLocal, rockerELocal, q).build(), MAT_CARBON, 'rocker');
  rockerPivot.add(rocker);

  const shockBodyPivot = new THREE.Object3D();
  shockBodyPivot.position.set(0, G.shockMount.y, G.shockMount.z);
  chassis.add(shockBodyPivot);
  const shockBody = mkMesh(buildShockBody(shockBodyLen, q).build(), MAT_HARD, 'shockBody');
  shockBodyPivot.add(shockBody);

  const shockShaftPivot = new THREE.Object3D();
  chassis.add(shockShaftPivot);
  const shockShaft = mkMesh(buildShockShaft(shockShaftLen).build(), MAT_HARD, 'shockShaft');
  shockShaftPivot.add(shockShaft);

  // ---- drivetrain ---------------------------------------------------------
  const crankPivot = new THREE.Object3D();
  crankPivot.position.set(0, G.bb.y, G.bb.z);
  chassis.add(crankPivot);
  const crankset = mkMesh(buildCrankset(q).build(), MAT_HARD, 'crankset');
  crankPivot.add(crankset);

  const pedalAnchorR = new THREE.Object3D();
  pedalAnchorR.position.set(G.crankQ, 0, -G.crankLen);
  crankPivot.add(pedalAnchorR);
  const pedalAnchorL = new THREE.Object3D();
  pedalAnchorL.position.set(-G.crankQ, 0, G.crankLen);
  crankPivot.add(pedalAnchorL);

  const pedalR = mkMesh(buildPedal(1, q).build(), MAT_HARD, 'pedalR');
  const pedalL = mkMesh(buildPedal(-1, q).build(), MAT_HARD, 'pedalL');
  chassis.add(pedalR, pedalL);

  const guideLocal = { y: -0.075, z: 0.020 };
  const tensionLocal = { y: -0.135, z: -0.040 };
  const derailleur = mkMesh(buildDerailleur(guideLocal, tensionLocal, q).build(), MAT_HARD, 'derailleur');
  rearAxleCarrier.add(derailleur);

  // ---- chain --------------------------------------------------------------
  const belt = new Belt(4);
  belt.r[0] = pitchRadius(G.chainringTeeth);
  belt.r[1] = pitchRadius(G.pulleyTeeth);
  belt.r[2] = pitchRadius(G.pulleyTeeth);
  belt.r[3] = pitchRadius(G.cogTeeth[G.driveCog]);
  const setBelt = () => {
    // Working plane: a = −z (forward), b = y (up).
    belt.cx[0] = -G.bb.z; belt.cy[0] = G.bb.y;
    belt.cx[1] = -(rearAxleCarrier.position.z + tensionLocal.z); belt.cy[1] = rearAxleCarrier.position.y + tensionLocal.y;
    belt.cx[2] = -(rearAxleCarrier.position.z + guideLocal.z); belt.cy[2] = rearAxleCarrier.position.y + guideLocal.y;
    belt.cx[3] = -rearAxleCarrier.position.z; belt.cy[3] = rearAxleCarrier.position.y;
    return belt.solve();
  };
  SG.rearAxle(SG.shock.sagT, _ra);
  rearAxleCarrier.position.set(0, _ra.y, _ra.z);
  setBelt();
  const linkCount = Math.max(24, Math.round((belt.valid ? belt.total : 1.5) / G.chainPitch));
  const chainGeo = buildChainLink();
  const chain = new THREE.InstancedMesh(chainGeo, MAT_HARD, linkCount);
  chain.name = 'chain';
  chain.castShadow = false;        // 40 links inside a shadow texel — see F6
  chain.receiveShadow = !!settings.shadows;
  chain.frustumCulled = false;
  chain.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  geoms.push(chainGeo);
  meshes.push(chain);
  chassis.add(chain);

  // ---- brakes -------------------------------------------------------------
  const calF = buildCaliper(0.088, q);
  const calFGeo = calF.parts.build();
  const padSqueeze = (ranges) => (p, i) => {
    for (const pr of ranges) {
      if (i >= pr.range.start && i < pr.range.start + pr.range.count) {
        p.x -= pr.side * 0.0024;
        return;
      }
    }
  };
  addMorph(calFGeo, padSqueeze(calF.padRanges));
  const caliperFront = mkMesh(calFGeo, MAT_HARD, 'caliperFront');
  caliperFront.rotation.x = 0.78;          // post-mount, up and behind the axle
  forkLowerGroup.add(caliperFront);

  const calR = buildCaliper(0.088, q);
  const calRGeo = calR.parts.build();
  addMorph(calRGeo, padSqueeze(calR.padRanges));
  const caliperRear = mkMesh(calRGeo, MAT_HARD, 'caliperRear');
  caliperRear.rotation.x = -1.15;          // up and forward of the rear axle
  rearAxleCarrier.add(caliperRear);

  // ---- anchors (ADDENDUM §D, binding) -------------------------------------
  const mkAnchor = (parent, x, y, z, name) => {
    const o = new THREE.Object3D();
    o.name = name;
    o.position.set(x, y, z);
    parent.add(o);
    return o;
  };
  const gL = cockpitBuild.gripAnchors.L, gR = cockpitBuild.gripAnchors.R;
  const bc = cockpitBuild.barCentre;
  const anchors = {
    gripL: mkAnchor(cockpit, gL.x, gL.y, gL.z, 'gripL'),
    gripR: mkAnchor(cockpit, gR.x, gR.y, gR.z, 'gripR'),
    pedalL: pedalAnchorL,
    pedalR: pedalAnchorR,
    saddle: mkAnchor(chassis, 0, G.saddle.y + 0.008, G.saddle.z - G.saddleLen * 0.46, 'saddle'),
    bb: mkAnchor(chassis, 0, G.bb.y, G.bb.z, 'bb'),
    frontAxle: mkAnchor(forkLowerGroup, 0, 0, 0, 'frontAxle'),
    rearAxle: mkAnchor(rearAxleCarrier, 0, 0, 0, 'rearAxle'),
    bar: mkAnchor(cockpit, bc.x, bc.y, bc.z, 'bar'),
  };
  anchors.pedalL.name = 'pedalL';
  anchors.pedalR.name = 'pedalR';

  // ---- contact shadow -----------------------------------------------------
  // Two instances of one 0.5 m quad, multiply-blended onto whatever is under the
  // tyres. NOT a lit surface — it is an occlusion factor, so MeshBasicMaterial is
  // the correct (and only sensible) choice here despite CONTRACT §0's general
  // rule; the map is authored linear because MultiplyBlending multiplies the
  // linear HDR target directly.
  const blobTex = buildContactBlobTexture(64);
  const blobMat = new THREE.MeshBasicMaterial({
    map: blobTex,
    transparent: true,
    opacity: 1.0,
    depthWrite: false,
    blending: THREE.MultiplyBlending,
    fog: false,
    toneMapped: false,
  });
  const blobGeo = new THREE.PlaneGeometry(1, 1);
  blobGeo.rotateX(-Math.PI / 2);              // lies in the ground plane
  const contactShadow = new THREE.InstancedMesh(blobGeo, blobMat, 2);
  contactShadow.name = 'contactShadow';
  contactShadow.frustumCulled = false;
  contactShadow.castShadow = false;
  contactShadow.receiveShadow = false;
  contactShadow.renderOrder = 2;
  contactShadow.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  contactShadow.visible = false;               // until we have a contact to sit on
  group.add(contactShadow);                    // world space, NOT under `chassis`
  geoms.push(blobGeo);

  if (ctx && ctx.scene) ctx.scene.add(group);

  // =========================================================================
  // ANIMATION STATE (all scalars — no per-frame allocation anywhere below)
  // =========================================================================
  let spinFront = 0, spinRear = 0;
  let crankAngle = 0, crankOmega = 0;
  let chainOffset = 0;
  let flexAmount = 0;
  let padF = 0, padR = 0;
  let prevSy = G.seatstayTop.y, prevSz = G.seatstayTop.z;
  const gearRatio = G.cogTeeth[G.driveCog] / G.chainringTeeth;
  const chainringR = pitchRadius(G.chainringTeeth);

  // Place everything once so the first rendered frame is already correct.
  function poseSuspension(forkTravel, shockTravel) {
    // ---- front ------------------------------------------------------------
    forkLowerGroup.position.y = -(SG.fork.len - forkTravel);

    // ---- rear axle --------------------------------------------------------
    SG.rearAxle(shockTravel, _ra);
    rearAxleCarrier.position.set(0, _ra.y, _ra.z);

    // ---- swingarm tracks the axle ----------------------------------------
    const ay = _ra.y - G.mainPivot.y, az = _ra.z - G.mainPivot.z;
    const armLen = Math.sqrt(ay * ay + az * az);
    const theta = phiArmRest - angleYZ(ay, az);
    swingarmPivot.rotation.x = theta;
    // The arm shortens by ~12 mm across the travel (the physics axle path is a
    // straight line, a swingarm's is an arc). Absorb it as a ≤2 % stretch along
    // the arm rather than letting the dropout drift off the axle.
    const kz = armLen / armLen0;
    swingarm.scale.z = kz;

    // ---- Horst pivot, carried by the swingarm -----------------------------
    const ct = Math.cos(theta), st = Math.sin(theta);
    const hlz = horstLocal.z * kz;
    _H.y = horstLocal.y * ct - hlz * st + G.mainPivot.y;
    _H.z = horstLocal.y * st + hlz * ct + G.mainPivot.z;

    // ---- four-bar: seatstay ∩ rocker --------------------------------------
    const hit = circleIntersect(_H.y, _H.z, L_ss, G.rockerPivot.y, G.rockerPivot.z, L_rk, prevSy, prevSz);
    if (hit.ok) { _S.y = hit.y; _S.z = hit.z; prevSy = hit.y; prevSz = hit.z; }
    else { _S.y = prevSy; _S.z = prevSz; }

    seatstayPivot.position.set(0, _H.y, _H.z);
    seatstayPivot.rotation.x = phiStayRest - angleYZ(_S.y - _H.y, _S.z - _H.z);

    // R_x(θ) maps a local angle φ to φ − θ, so driving the rocker to +rockAng
    // means rotation.x = −rockAng — and the shock eye must be transformed by the
    // SAME rotation, or the shock silently runs backwards.
    const rockAng = angleYZ(_S.y - G.rockerPivot.y, _S.z - G.rockerPivot.z) - phiRockerRest;
    rockerPivot.rotation.x = -rockAng;
    const rc = Math.cos(rockAng), rs = -Math.sin(rockAng);
    _E.y = rockerELocal.y * rc - rockerELocal.z * rs + G.rockerPivot.y;
    _E.z = rockerELocal.y * rs + rockerELocal.z * rc + G.rockerPivot.z;

    // ---- shock: body from the frame mount, shaft from the rocker eye -------
    const sy = _E.y - G.shockMount.y, sz = _E.z - G.shockMount.z;
    const alpha = Math.atan2(sz, sy);
    shockBodyPivot.rotation.x = alpha;
    shockShaftPivot.position.set(0, _E.y, _E.z);
    shockShaftPivot.rotation.x = alpha;
  }

  function poseChain() {
    if (!setBelt()) return;
    const pitch = belt.total / linkCount;
    for (let i = 0; i < linkCount; i++) {
      belt.sample(chainOffset + i * pitch, _beltPt);
      _iz.set(0, _beltPt.tb, -_beltPt.ta);
      _iy.set(0, -_beltPt.ta, -_beltPt.tb);
      _ipos.set(G.chainline, _beltPt.b, -_beltPt.a);
      _im.makeBasis(_ix, _iy, _iz);
      _im.setPosition(_ipos);
      chain.setMatrixAt(i, _im);
    }
    chain.instanceMatrix.needsUpdate = true;
  }

  poseSuspension(SG.fork.sagT, SG.shock.sagT);
  poseChain();

  // =========================================================================
  // BAKE — occlusion, once, at the sag pose
  // =========================================================================
  if (q >= 1) {
    group.updateMatrixWorld(true);
    // `group` and `chassis` are both at identity here, so every mesh's
    // matrixWorld is already its chassis-local transform.
    const statics = [];
    for (const m of meshes) {
      if (!m.isMesh || m.isInstancedMesh) continue;
      if (m.name === 'wheelFront' || m.name === 'wheelRear') continue;
      statics.push({ geo: m.geometry, matrix: m.matrixWorld });
    }
    const occ = makeOccupancy(statics, 46, 0.06);
    if (occ) {
      // The wheels occlude the frame even though they spin, so put them in as
      // solid discs — rotationally invariant, which is the whole trick.
      occupyDisc(occ, 0, forkLowerGroup.getWorldPosition(_v).y, _v.z, 0.355, 0.035);
      occupyDisc(occ, 0, rearAxleCarrier.position.y, rearAxleCarrier.position.z, 0.355, 0.035);
      for (const it of statics) bakeVertexAO(it.geo, it.matrix, occ, 0.36, 10, 1.7, 0.48);
    }
    // Each wheel occludes itself in its own frame: knobs shading the carcass,
    // spokes shading the rim bed, the rotor shading the hub. Baked wheel-local so
    // it is invariant under the wheel's own rotation.
    for (const m of meshes) {
      if (m.name !== 'wheelFront' && m.name !== 'wheelRear') continue;
      const wocc = makeOccupancy([{ geo: m.geometry, matrix: null }], 40, 0.03);
      bakeVertexAO(m.geometry, null, wocc, 0.34, 8, 1.7, 0.52);
    }
  }

  // =========================================================================
  // UPDATE
  // =========================================================================
  function update(dt, c) {
    const context = c || ctx;
    const s = context && context.bike && context.bike.state;
    const d = dt > 0 ? Math.min(dt, 1 / 20) : 0;
    if (!s) return;

    // ---- chassis ----------------------------------------------------------
    chassis.position.copy(s.position);
    chassis.quaternion.copy(s.quaternion);

    // ---- steering ---------------------------------------------------------
    // bike.js applies +steer as a rotation of −steer about the contact normal
    // (i.e. + = turn right), so the visual bar must match sign for sign.
    steerRot.rotation.y = -(s.steer || 0);

    // ---- suspension -------------------------------------------------------
    const susp = s.suspension || null;
    const ft = susp && susp.fork ? clamp(susp.fork.travel, 0, SG.fork.travel) : SG.fork.sagT;
    const st = susp && susp.shock ? clamp(susp.shock.travel, 0, SG.shock.travel) : SG.shock.sagT;
    poseSuspension(ft, st);

    // ---- wheel spin -------------------------------------------------------
    const wf = s.wheels && s.wheels[0], wr = s.wheels && s.wheels[1];
    spinFront += (wf ? wf.spinRate : 0) * d;
    spinRear += (wr ? wr.spinRate : 0) * d;
    // Keep the accumulator bounded — at 90 rad/s an unbounded float loses
    // precision in a few minutes and the wheel starts to judder.
    if (spinFront > 1e4 || spinFront < -1e4) spinFront %= Math.PI * 2;
    if (spinRear > 1e4 || spinRear < -1e4) spinRear %= Math.PI * 2;
    wheelFront.rotation.x = -spinFront;
    wheelRear.rotation.x = -spinRear;

    // ---- cranks -----------------------------------------------------------
    const pedalling = s.pedalling || 0;
    if (pedalling > 0.02 && wr) {
      // Chain-driven: cadence follows the rear wheel through the gear.
      crankOmega = wr.spinRate * gearRatio;
      crankAngle -= crankOmega * d;
    } else {
      // Coasting: a downhill rider parks the cranks level. Ease to whichever
      // level position is closer so the feet never swing through the arc.
      crankOmega = 0;
      const target = Math.round(crankAngle / Math.PI) * Math.PI;
      crankAngle = damp(crankAngle, target, 5.0, d);
    }
    if (crankAngle > 1e4 || crankAngle < -1e4) crankAngle %= Math.PI * 2;
    crankPivot.rotation.x = crankAngle;
    // Pedals stay level with the chassis while their spindles orbit.
    const cc = Math.cos(crankAngle), cs = Math.sin(crankAngle);
    pedalR.position.set(G.crankQ, G.bb.y + (-G.crankLen) * -cs, G.bb.z + (-G.crankLen) * cc);
    pedalL.position.set(-G.crankQ, G.bb.y + (G.crankLen) * -cs, G.bb.z + (G.crankLen) * cc);

    // ---- chain -------------------------------------------------------------
    // The chain is driven by the chainring, so it only moves when you pedal —
    // exactly like the real thing, where coasting leaves the chain still.
    chainOffset += crankOmega * chainringR * d;
    if (chainOffset > 1e4 || chainOffset < -1e4) chainOffset = 0;
    poseChain();

    // ---- brakes ------------------------------------------------------------
    padF = damp(padF, clamp01(s.brakeFront || 0), 22, d);
    padR = damp(padR, clamp01(s.brakeRear || 0), 22, d);
    if (caliperFront.morphTargetInfluences) caliperFront.morphTargetInfluences[0] = padF;
    if (caliperRear.morphTargetInfluences) caliperRear.morphTargetInfluences[0] = padR;
    if (cockpit.morphTargetInfluences) {
      cockpit.morphTargetInfluences[0] = padF;
      cockpit.morphTargetInfluences[1] = padR;
    }

    // ---- frame flex --------------------------------------------------------
    // Driven by how hard the fork is working plus chassis g — a proxy for the
    // load actually going through the front triangle.
    const load = clamp01(ft / Math.max(1e-3, SG.fork.travel)) * 0.75 +
                 clamp01(((s.gForce || 1) - 1) * 0.30) * 0.45;
    flexAmount = damp(flexAmount, clamp01(load), 12, d);
    if (frameAlu.morphTargetInfluences) frameAlu.morphTargetInfluences[0] = flexAmount;
    if (frameCarbon.morphTargetInfluences) frameCarbon.morphTargetInfluences[0] = flexAmount;

    // ---- screen-space scale for the thin-feature expansion (F4) -------------
    // uPx = pixels per metre at one metre of view depth = P[1][1] * height/2.
    // Read from the DRAWING BUFFER, not the CSS size, or the minimum width is
    // wrong by the device pixel ratio.
    const cam = context && context.camera;
    if (cam && cam.isPerspectiveCamera) {
      let h = 1080;
      const rend = context.renderer;
      if (rend && rend.getDrawingBufferSize) h = rend.getDrawingBufferSize(_bufSize).y || h;
      shared.uPx.value = cam.projectionMatrix.elements[5] * h * 0.5;
    }

    // ---- LOD (F6) -----------------------------------------------------------
    // The chain is ~40 instances of a 7 mm link. Past 18 m it is below a pixel
    // and it is the single cheapest thing on the bike to stop submitting.
    if (cam) {
      cam.getWorldPosition(_camPos);
      const dist = _camPos.distanceTo(s.position);
      chain.visible = dist < 18;
      derailleur.visible = dist < 45;
    }

    // ---- contact shadow (F6 grounding) -------------------------------------
    if (settings.contactShadow !== false && s.wheels && s.wheels.length >= 2) {
      let any = false;
      for (let i = 0; i < 2; i++) {
        const w = s.wheels[i];
        const cp = w && w.contactPoint;
        // Fade out with air time so the blob does not stick to the ground under a
        // bike that has left it.
        const on = w && w.contact && cp ? 1 : 0;
        if (!on) {
          _cbScale.set(0, 0, 0);
          _cbM.compose(_cbPos.set(0, -9999, 0), _cbQ.identity(), _cbScale);
          contactShadow.setMatrixAt(i, _cbM);
          continue;
        }
        any = true;
        _cbNrm.copy(w.normal || _cbUp);
        if (_cbNrm.lengthSq() < 0.25) _cbNrm.copy(_cbUp); else _cbNrm.normalize();
        _cbQ.setFromUnitVectors(_cbUp, _cbNrm);
        // A loaded tyre spreads its contact patch; an unloaded one barely marks.
        // Circular, deliberately: the quad's yaw is whatever setFromUnitVectors
        // hands back, so an elongated blob would not stay aligned with the bike.
        const k = 0.26 + 0.20 * clamp01(w.compression || 0);
        _cbScale.set(k, 1, k);
        _cbPos.copy(cp).addScaledVector(_cbNrm, 0.012);   // clear of the surface
        _cbM.compose(_cbPos, _cbQ, _cbScale);
        contactShadow.setMatrixAt(i, _cbM);
      }
      contactShadow.visible = any;
      contactShadow.instanceMatrix.needsUpdate = true;
    } else {
      contactShadow.visible = false;
    }
  }

  function dispose() {
    if (ctx && ctx.scene) ctx.scene.remove(group);
    for (const g of geoms) g.dispose();
    MAT_HARD.dispose(); MAT_CARBON.dispose();
    blobMat.dispose(); blobTex.dispose(); decalMap.dispose();
    metalMaps.normal.dispose(); metalMaps.rough.dispose();
    carbonMaps.normal.dispose(); carbonMaps.rough.dispose();
    meshes.length = 0; geoms.length = 0;
  }

  return {
    group,
    chassis,
    anchors,
    update,
    dispose,

    // Handy for debugging / the QA pass.
    drawCalls: meshes.length + 1,        // + the contact-shadow InstancedMesh
    shadowCasters: meshes.filter((m) => m.castShadow).length,
    contactShadow,
    meshes,
    materials: { hard: MAT_HARD, carbon: MAT_CARBON },
    textures: { decal: decalMap },
  };
}
