// DESCENT — particles.js
// Pooled, zero-allocation effect systems: dirt roost, dust plumes, rock chips,
// mud spray, water splash, forest debris, high-speed grit, impact bursts and
// persistent tyre tracks. See CONTRACT.md §6 and ADDENDUM §B/§G.
//
// Architecture
// ------------
// Every effect family is ONE draw call: a `THREE.Mesh` whose geometry is an
// `InstancedBufferGeometry` (a unit quad + per-instance attributes) sharing a
// single procedurally generated 4x4 sprite atlas. Simulation runs on the CPU
// over flat `Float32Array`s (structure-of-arrays); dead particles are swapped
// out by compaction so `instanceCount` — and therefore the per-frame vertex
// upload — is proportional to the number of *live* particles, not the pool
// size. Nothing is allocated after construction: spawn parameters are written
// into a module-scope scratch record (`SP`) and consumed synchronously.
//
// Draw calls: 7 sprite systems + 1 tyre-track ribbon = 8 (budget is 12).
//
// CONTRACT-NOTE (particles -> postfx): the contract asks for soft particles.
// No module exposes a scene depth texture (postfx owns its render targets and
// does not publish one), and sampling one from a forward-rendered transparent
// pass would mean re-plumbing a module I do not own. Instead the soft fade is
// analytic: every particle carries the terrain height beneath it and the
// fragment shader fades the sprite out over the last ~0.3–1.2 m above that
// height, using the fragment's own interpolated world position. That removes
// the hard clipping line where a sprite intersects the ground — the case that
// actually shows — at zero bandwidth cost. A near-plane fade is applied the
// same way so particles never smear across the whole screen.
//
// CONTRACT-NOTE (particles -> bike): `state.wheels[i].position` is read as the
// *axle* centre. The contact patch is derived as
// `terrain.sampleHeight(x, z)` under the axle rather than
// `position - normal * radius`, so it stays correct whatever convention bike.js
// settles on, and so tracks never float or sink.
//
// CONTRACT-NOTE (particles -> events): consumes `bike:impact`, `bike:skid`,
// `water:splash`, `run:crash`, `run:start`, `run:checkpoint` and
// `trick:landed`. All payload fields are optional; the bike state is used as a
// fallback and a missing field never throws. Also exposes
// `particles.burst(kind, position, normal, strength, surface)` for any module
// that wants a one-shot effect without going through the event bus.
//
// CONTRACT-NOTE (particles -> sky): all four materials are custom
// `ShaderMaterial`s, so per ADDENDUM §B they set `fog: true` and merge
// `THREE.UniformsLib.fog`; the shared `hFogParams` uniform object is also
// re-pointed explicitly after the merge so the height-fog block can never be
// silently disconnected by a change in three's uniform-cloning rules.

import * as THREE from 'three';
import { Surface } from '../world/terrain.js';
import { makeRng, subSeed, clamp, clamp01, lerp, smoothstep } from '../core/rng.js';

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------

const WHEEL_R = 0.37;            // matches CONTRACT §0 scale reference
const GRAVITY = 9.81;

const ATLAS_TILE = 128;          // px per atlas tile
const ATLAS_COLS = 4;            // 4x4 => 16 tiles, 512x512 texture
const ATLAS_INSET = 3 / ATLAS_TILE;  // UV inset, keeps mip bleed out of neighbours

// Atlas tile ids.
const T_DUST_A = 0, T_DUST_B = 1, T_SMOKE = 2, T_CLOD_A = 3;
const T_CLOD_B = 4, T_GRIT = 5, T_CHIP = 6, T_STREAK = 7;
const T_DROP = 8, T_SHEET = 9, T_FOAM = 10, T_SPARK = 11;
const T_LEAF = 12, T_NEEDLE = 13, T_GOB = 14, T_RING = 15;

// Tyre-track ribbon.
const TRACK_SEGMENTS = 1024;     // per ribbon; 2 ribbons (rear, front)
const TRACK_SPACING = 0.30;      // metres between ribbon cross-sections
const TRACK_MAX_STEP = 3.0;      // break the ribbon if we jump further than this
const TRACK_LIFE = 20.0;         // seconds to fade out (CONTRACT §6)
const TRACK_LIFT = 0.055;        // metres above the heightfield

// Ambient air movement, so dust does not hang in dead air. Roughly matches the
// direction sky.js drifts its cloud layer.
const WIND_X = 1.15;
const WIND_Z = -0.70;

// ---------------------------------------------------------------------------
// Surface palettes. Authored sRGB per CONTRACT §0, converted once to the
// renderer's linear working space.
// ---------------------------------------------------------------------------

const SURFACE_SRGB = [
  [0.44, 0.29, 0.18],   // DIRT   — red-brown
  [0.25, 0.19, 0.13],   // LOAM   — dark, organic
  [0.53, 0.51, 0.49],   // ROCK   — grey rock dust
  [0.64, 0.60, 0.52],   // GRAVEL — pale
  [0.34, 0.34, 0.19],   // GRASS  — olive
  [0.30, 0.22, 0.15],   // ROOT   — dark brown
  [0.20, 0.15, 0.10],   // MUD    — near-black wet brown
  [0.93, 0.95, 0.99],   // SNOW
];

// How much loose material a surface actually throws.
const SURFACE_YIELD = [1.0, 1.15, 0.30, 0.85, 0.55, 0.45, 0.95, 0.90];
// How readable a tyre track is on it.
const SURFACE_TRACK = [1.0, 1.0, 0.14, 0.55, 0.70, 0.45, 1.0, 0.95];

const _tmpColor = new THREE.Color();

function toLinear(rgb, scale, desat) {
  const l = rgb[0] * 0.2126 + rgb[1] * 0.7152 + rgb[2] * 0.0722;
  const r = lerp(rgb[0], l, desat) * scale;
  const g = lerp(rgb[1], l, desat) * scale;
  const b = lerp(rgb[2], l, desat) * scale;
  _tmpColor.setRGB(clamp01(r), clamp01(g), clamp01(b), THREE.SRGBColorSpace);
  return [_tmpColor.r, _tmpColor.g, _tmpColor.b];
}

// Chunky material thrown by the tyre: full saturation.
const COL_CHUNK = SURFACE_SRGB.map((c) => toLinear(c, 1.0, 0.0));
// Airborne dust: much lighter and strongly desaturated. Fine particulate
// scatters light from the whole sky, so a real plume reads BRIGHTER than the
// ground it came off — matching the ground albedo just makes a muddy smear.
const COL_DUST = SURFACE_SRGB.map((c) => toLinear(c, 2.10, 0.58));
// Wet/mud spray: darker than the dry surface.
const COL_WET = SURFACE_SRGB.map((c) => toLinear(c, 0.55, 0.12));
// Tyre track base (before the CPU-side lighting bake). Deliberately much
// darker than the surface: this is alpha-blended over lit ground, so the tint
// has to be well below it to read as a darkened imprint at all.
const COL_TRACK = SURFACE_SRGB.map((c) => toLinear(c, 0.34, 0.10));

const COL_WATER = toLinear([0.72, 0.82, 0.86], 1.0, 0.0);
const COL_FOAM = toLinear([0.95, 0.97, 1.0], 1.0, 0.0);
const COL_LEAF = [
  toLinear([0.36, 0.30, 0.13], 1.0, 0.0),   // dry brown leaf
  toLinear([0.28, 0.34, 0.15], 1.0, 0.0),   // green leaf
  toLinear([0.21, 0.26, 0.14], 1.0, 0.0),   // pine needle
];
const COL_SPARK = [2.6, 1.7, 0.85];         // > 1 on purpose: this is what bloom is for

function surfaceIndex(s) {
  const i = s | 0;
  return (i >= 0 && i < 8) ? i : 0;
}

// ---------------------------------------------------------------------------
// Procedural sprite atlas.
// ---------------------------------------------------------------------------

function makeValueNoise(rng) {
  // Small hashed-lattice value noise. Cheap, and only ever run at build time.
  const SIZE = 64;
  const table = new Float32Array(SIZE * SIZE);
  for (let i = 0; i < table.length; i++) table[i] = rng();
  const at = (xi, yi) => table[((yi & (SIZE - 1)) * SIZE) + (xi & (SIZE - 1))];
  return function noise2(x, y) {
    const xi = Math.floor(x), yi = Math.floor(y);
    const fx = x - xi, fy = y - yi;
    const ux = fx * fx * (3 - 2 * fx);
    const uy = fy * fy * (3 - 2 * fy);
    const a = at(xi, yi), b = at(xi + 1, yi), c = at(xi, yi + 1), d = at(xi + 1, yi + 1);
    return lerp(lerp(a, b, ux), lerp(c, d, ux), uy) * 2 - 1;
  };
}

function fbm(noise2, x, y, octaves) {
  let amp = 0.5, freq = 1, sum = 0, norm = 0;
  for (let o = 0; o < octaves; o++) {
    sum += amp * noise2(x * freq, y * freq);
    norm += amp;
    amp *= 0.5;
    freq *= 2.03;
  }
  return sum / Math.max(norm, 1e-5);
}

function sdSegment(px, py, ax, ay, bx, by) {
  const vx = bx - ax, vy = by - ay;
  const wx = px - ax, wy = py - ay;
  const l2 = vx * vx + vy * vy;
  let t = l2 > 1e-9 ? (wx * vx + wy * vy) / l2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const dx = wx - vx * t, dy = wy - vy * t;
  return Math.sqrt(dx * dx + dy * dy);
}

/**
 * Builds the 4x4 sprite atlas. RGB carries a luminance detail signal (used as a
 * brightness modulation on the particle's own colour) and A carries the mask.
 */
function buildAtlas(seed) {
  const rng = makeRng(subSeed(seed, 'particle-atlas'));
  const noise2 = makeValueNoise(rng);
  const S = ATLAS_TILE;
  const W = S * ATLAS_COLS;

  if (typeof document === 'undefined') return null;   // defensive: non-DOM host
  const canvas = document.createElement('canvas');
  canvas.width = W; canvas.height = W;
  const c2d = canvas.getContext('2d', { willReadFrequently: false });
  const img = c2d.createImageData(W, W);
  const data = img.data;

  // Per-tile random parameters, drawn up front so each tile is deterministic
  // regardless of evaluation order.
  const chipPlanes = [];
  for (let k = 0; k < 7; k++) {
    const a = (k / 7) * Math.PI * 2 + rng() * 0.5;
    chipPlanes.push([Math.cos(a), Math.sin(a), 0.42 + rng() * 0.30]);
  }
  const gritDots = [];
  for (let k = 0; k < 9; k++) {
    gritDots.push([(rng() * 2 - 1) * 0.62, (rng() * 2 - 1) * 0.62, 0.07 + rng() * 0.10]);
  }
  const foamBalls = [];
  for (let k = 0; k < 11; k++) {
    foamBalls.push([(rng() * 2 - 1) * 0.42, (rng() * 2 - 1) * 0.40, 0.20 + rng() * 0.18]);
  }
  const gobBalls = [];
  for (let k = 0; k < 6; k++) {
    gobBalls.push([(rng() * 2 - 1) * 0.34, -0.25 + rng() * 0.55, 0.16 + rng() * 0.22]);
  }
  const needles = [];
  for (let k = 0; k < 5; k++) {
    const a = -Math.PI * 0.5 + (k - 2) * 0.30 + (rng() - 0.5) * 0.14;
    const len = 1.15 + rng() * 0.5;
    needles.push([Math.cos(a) * len, Math.sin(a) * len]);
  }

  function metaball(u, v, balls) {
    let f = 0;
    for (let k = 0; k < balls.length; k++) {
      const dx = u - balls[k][0], dy = v - balls[k][1];
      const r = balls[k][2];
      f += (r * r) / (dx * dx + dy * dy + 1e-4);
    }
    return f;
  }

  // Each generator returns alpha in .a and greyscale detail in .g via `out`.
  const out = { a: 0, g: 0.8 };

  function tileDust(u, v, r, soft, stretchY) {
    const n = fbm(noise2, u * 2.3 + 11.7, v * 2.3 * stretchY + 4.1, 4);
    const d = Math.sqrt(u * u + (v * v) / (stretchY * stretchY)) + n * soft;
    let a = 1 - smoothstep(0.42, 0.94, d);
    a *= 1 - smoothstep(0.80, 1.0, r);
    // Internal billow shading: brighter on one side so the puff has a form,
    // the sun term in the shader then pushes it further.
    const sh = fbm(noise2, u * 3.4 - 20.0, v * 3.4 + 7.5, 3);
    out.a = a * a * (0.85 + 0.15 * sh);
    out.g = clamp01(0.55 + 0.40 * (0.5 + 0.5 * sh) + 0.12 * (0.5 - v * 0.5));
  }

  function tileClod(u, v, r, seedOff, elong) {
    const n = fbm(noise2, u * 2.7 + seedOff, v * 2.7 * elong - seedOff, 4);
    const d = Math.sqrt(u * u * elong * elong + v * v) + n * 0.24;
    const a = 1 - smoothstep(0.60, 0.70, d);
    const speck = fbm(noise2, u * 9.0 + seedOff * 2, v * 9.0, 2);
    out.a = a * (1 - smoothstep(0.86, 1.0, r));
    // Clods are lit lumps: a strong internal value range sells the silhouette.
    out.g = clamp01(0.42 + 0.36 * (0.5 - v * 0.55) + 0.26 * (0.5 + 0.5 * speck));
  }

  const rgbTmp = [1, 1, 1];

  for (let tile = 0; tile < ATLAS_COLS * ATLAS_COLS; tile++) {
    const ox = (tile % ATLAS_COLS) * S;
    const oy = Math.floor(tile / ATLAS_COLS) * S;

    for (let py = 0; py < S; py++) {
      const v = ((py + 0.5) / S) * 2 - 1;
      for (let px = 0; px < S; px++) {
        const u = ((px + 0.5) / S) * 2 - 1;
        const r = Math.sqrt(u * u + v * v);
        out.a = 0; out.g = 0.8;
        rgbTmp[0] = 1; rgbTmp[1] = 1; rgbTmp[2] = 1;

        switch (tile) {
          case T_DUST_A: tileDust(u, v, r, 0.26, 1.0); break;
          case T_DUST_B: tileDust(u, v, r, 0.40, 0.82); break;

          case T_SMOKE: {
            // Wisp: sheared, with a swirl so a rotating sprite reads as motion.
            const sw = fbm(noise2, u * 1.7 + 3.3, v * 1.1 - 9.4, 4);
            const uu = u + sw * 0.35 * (1 - Math.abs(v));
            const d = Math.sqrt(uu * uu * 1.35 + v * v * 0.55) + sw * 0.30;
            let a = 1 - smoothstep(0.34, 0.92, d);
            a *= 1 - smoothstep(0.80, 1.0, r);
            out.a = a * a * 0.92;
            out.g = clamp01(0.60 + 0.38 * (0.5 + 0.5 * sw));
            break;
          }

          case T_CLOD_A: tileClod(u, v, r, 5.0, 1.0); break;
          case T_CLOD_B: tileClod(u, v, r, 31.0, 1.28); break;

          case T_GRIT: {
            // A cluster of separate grains — reads as spray without needing
            // one sprite per grain.
            let a = 0, g = 0;
            for (let k = 0; k < gritDots.length; k++) {
              const dx = u - gritDots[k][0], dy = v - gritDots[k][1];
              const dd = Math.sqrt(dx * dx + dy * dy);
              const ka = 1 - smoothstep(gritDots[k][2] * 0.65, gritDots[k][2], dd);
              if (ka > a) { a = ka; g = 0.40 + 0.55 * (k / gritDots.length); }
            }
            out.a = a;
            out.g = clamp01(g + 0.15);
            break;
          }

          case T_CHIP: {
            // Convex polygon: an angular stone flake.
            let d = -1e9;
            for (let k = 0; k < chipPlanes.length; k++) {
              const h = u * chipPlanes[k][0] + v * chipPlanes[k][1] - chipPlanes[k][2];
              if (h > d) d = h;
            }
            const a = 1 - smoothstep(-0.02, 0.02, d);
            const f = fbm(noise2, u * 5.5 + 60, v * 5.5, 3);
            out.a = a;
            // Flat facets: a hard value break down the middle reads as a chip
            // catching the light on one face.
            out.g = clamp01(0.30 + 0.55 * (u * 0.5 + 0.5) + 0.18 * f);
            break;
          }

          case T_STREAK: {
            const w = Math.exp(-(u * u) * 16.0);
            const l = 1 - smoothstep(0.30, 1.0, Math.abs(v));
            out.a = w * l;
            out.g = clamp01(0.65 + 0.35 * l);
            break;
          }

          case T_DROP: {
            // Teardrop: a body with a tail, plus a bright specular rim.
            const f = metaball(u, v, [[0, 0.22, 0.34], [0, -0.28, 0.17], [0, -0.62, 0.07]]);
            const a = smoothstep(0.85, 1.15, f);
            const rim = smoothstep(1.9, 3.4, f);
            out.a = a;
            out.g = clamp01(0.55 + 0.45 * rim + 0.20 * (0.5 - u * 0.6));
            break;
          }

          case T_SHEET: {
            // Splash crown: a fan radiating from the bottom centre, broken into
            // fingers so it does not read as a solid triangle.
            const ry = v + 1.0;
            const rr = Math.sqrt(u * u + ry * ry) * 0.5;
            const ang = Math.atan2(u, Math.max(ry, 1e-3));
            const n = fbm(noise2, ang * 3.0 + 17.0, rr * 4.0, 3);
            const fingers = 0.55 + 0.45 * Math.sin(ang * 9.0 + n * 2.4);
            let a = (1 - smoothstep(0.30, 0.62, Math.abs(ang)))
                  * (1 - smoothstep(0.30, 0.98, rr))
                  * fingers;
            a *= smoothstep(0.0, 0.18, rr);
            out.a = clamp01(a * 1.25);
            out.g = clamp01(0.62 + 0.38 * fingers);
            break;
          }

          case T_FOAM: {
            const f = metaball(u, v, foamBalls);
            const n = fbm(noise2, u * 6.0 - 12.0, v * 6.0, 3);
            const a = smoothstep(0.95, 1.35, f + n * 0.22);
            out.a = a * (1 - smoothstep(0.88, 1.0, r));
            out.g = clamp01(0.78 + 0.22 * n);
            break;
          }

          case T_SPARK: {
            const core = Math.exp(-r * r * 26.0);
            const glow = Math.exp(-r * 3.6) * 0.30;
            const ray = Math.exp(-Math.abs(u) * 34.0) * Math.exp(-Math.abs(v) * 3.2) * 0.35
                      + Math.exp(-Math.abs(v) * 34.0) * Math.exp(-Math.abs(u) * 3.2) * 0.35;
            out.a = clamp01(core + glow + ray);
            out.g = 1.0;
            break;
          }

          case T_LEAF: {
            const hw = 0.47 * Math.sqrt(Math.max(0, 1 - v * v)) * (1 - 0.28 * v);
            const au = Math.abs(u);
            let a = 1 - smoothstep(hw - 0.035, hw + 0.015, au);
            a *= 1 - smoothstep(0.92, 1.0, Math.abs(v));
            const rib = 1 - smoothstep(0.0, 0.05, au);
            const vein = 0.5 + 0.5 * Math.sin((v * 7.0 + au * 5.0) * Math.PI);
            out.a = a;
            out.g = clamp01(0.52 + 0.34 * (au / Math.max(hw, 1e-3)) - 0.22 * rib + 0.10 * vein);
            break;
          }

          case T_NEEDLE: {
            let d = 1e9;
            for (let k = 0; k < needles.length; k++) {
              const dd = sdSegment(u, v, 0, -0.78, needles[k][0], -0.78 + needles[k][1]);
              if (dd < d) d = dd;
            }
            out.a = 1 - smoothstep(0.020, 0.048, d);
            out.g = clamp01(0.50 + 0.42 * (0.5 + 0.5 * fbm(noise2, u * 8, v * 8, 2)));
            break;
          }

          case T_GOB: {
            // Wet mud: a heavy blob with drips hanging off the bottom.
            const f = metaball(u, v, gobBalls);
            const n = fbm(noise2, u * 3.4 + 44, v * 3.4, 3);
            const a = smoothstep(0.92, 1.30, f + n * 0.18);
            out.a = a * (1 - smoothstep(0.90, 1.0, r));
            // Wet material has a tight specular highlight; keep it small so the
            // gob still reads dark overall.
            const spec = Math.exp(-((u + 0.20) * (u + 0.20) + (v - 0.24) * (v - 0.24)) * 22.0);
            out.g = clamp01(0.30 + 0.24 * (0.5 + 0.5 * n) + 0.70 * spec);
            break;
          }

          case T_RING: {
            const n = fbm(noise2, Math.atan2(v, u) * 2.4 + 3.0, r * 3.0, 3);
            const rr = r + n * 0.10;
            const band = Math.exp(-((rr - 0.70) * (rr - 0.70)) * 46.0);
            out.a = clamp01(band * (0.65 + 0.35 * (0.5 + 0.5 * n)));
            out.g = clamp01(0.70 + 0.30 * (0.5 + 0.5 * n));
            break;
          }

          default: break;
        }

        const idx = (((oy + py) * W) + (ox + px)) * 4;
        const g255 = clamp01(out.g) * 255;
        data[idx] = g255 * rgbTmp[0];
        data[idx + 1] = g255 * rgbTmp[1];
        data[idx + 2] = g255 * rgbTmp[2];
        data[idx + 3] = clamp01(out.a) * 255;
      }
    }
  }

  c2d.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.NoColorSpace;   // detail/mask data, not colour
  tex.flipY = false;                     // see the tile lookup in PARTICLE_VERT
  tex.wrapS = THREE.ClampToEdgeWrapping;
  tex.wrapT = THREE.ClampToEdgeWrapping;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.generateMipmaps = true;
  tex.anisotropy = 4;
  tex.needsUpdate = true;
  return tex;
}

/** Tiling knobby tyre tread, used as the tyre-track decal pattern. */
function buildTreadTexture(seed) {
  const rng = makeRng(subSeed(seed, 'particle-tread'));
  const noise2 = makeValueNoise(rng);
  const W = 128, H = 64;      // W = along the track, H = across
  if (typeof document === 'undefined') return null;
  const canvas = document.createElement('canvas');
  canvas.width = W; canvas.height = H;
  const c2d = canvas.getContext('2d');
  const img = c2d.createImageData(W, H);
  const data = img.data;

  // A DH tyre: two staggered centre knob rows plus taller shoulder knobs.
  // Everything is computed modulo 1 in u so the texture tiles seamlessly.
  const knob = (u, v, cu, cv, hw, hh) => {
    let du = u - cu;
    du -= Math.round(du);              // wrap to [-0.5, 0.5]
    const dv = v - cv;
    const qu = Math.abs(du) / hw, qv = Math.abs(dv) / hh;
    const d = Math.pow(qu * qu * qu * qu + qv * qv * qv * qv, 0.25);  // rounded rect
    return 1 - smoothstep(0.80, 1.0, d);
  };

  for (let py = 0; py < H; py++) {
    const v = (py + 0.5) / H;
    for (let px = 0; px < W; px++) {
      const u = (px + 0.5) / W;
      let k = 0;
      k = Math.max(k, knob(u, v, 0.125, 0.50, 0.075, 0.11));
      k = Math.max(k, knob(u, v, 0.625, 0.50, 0.075, 0.11));
      k = Math.max(k, knob(u, v, 0.375, 0.30, 0.060, 0.09));
      k = Math.max(k, knob(u, v, 0.875, 0.70, 0.060, 0.09));
      k = Math.max(k, knob(u, v, 0.250, 0.085, 0.085, 0.10));
      k = Math.max(k, knob(u, v, 0.750, 0.915, 0.085, 0.10));
      const n = fbm(noise2, u * 14.0, v * 7.0, 3);
      const val = clamp01(k * (0.80 + 0.20 * (0.5 + 0.5 * n)) + 0.10 * (0.5 + 0.5 * n));
      const i = (py * W + px) * 4;
      data[i] = data[i + 1] = data[i + 2] = val * 255;
      data[i + 3] = 255;
    }
  }
  c2d.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.NoColorSpace;
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.anisotropy = 8;
  tex.needsUpdate = true;
  return tex;
}

// ---------------------------------------------------------------------------
// Shaders
// ---------------------------------------------------------------------------
//
// three compiles ShaderMaterial as GLSL ES 3.00 with `attribute`/`varying`/
// `texture2D`/`gl_FragColor` aliased back, so GLSL1-style source is correct and
// the stock fog chunks (which sky.js has overridden with height fog) drop in.
// `mvPosition` must exist before <fog_vertex>; `cameraPosition` and `viewMatrix`
// are declared by three's own prefixes in both stages.

const PARTICLE_VERT = /* glsl */`
attribute vec3 aPos;
attribute vec4 aAttr;      // x age01, y size, z rotation, w atlas tile
attribute vec3 aColor;
attribute vec2 aMisc;      // x ground height, y opacity
#ifdef STRETCH
attribute vec3 aVel;
#endif

uniform float uTileCols;
uniform float uTileInset;
uniform vec2  uGrow;       // size multiplier at birth / at death
uniform vec4  uFade;       // fade-in end, fade-out start, master opacity, alpha power
uniform float uStretch;    // metres of stretch per m/s of velocity

varying vec2  vUv;
varying vec3  vColor;
varying float vAlpha;
varying vec3  vWorldPos;
varying float vGround;
varying vec2  vLocal;

#include <common>
#include <fog_pars_vertex>

void main() {
	float age = clamp( aAttr.x, 0.0, 1.0 );

	// Growth is eased so a puff blooms quickly then settles, rather than
	// expanding linearly for its whole life (which reads as a zoom, not a puff).
	float growT = 1.0 - pow( 1.0 - age, 2.0 );
	float size = aAttr.y * mix( uGrow.x, uGrow.y, growT );

	float fadeIn  = smoothstep( 0.0, max( uFade.x, 1.0e-4 ), age );
	float fadeOut = 1.0 - smoothstep( uFade.y, 1.0, age );
	vAlpha = pow( clamp( fadeIn * fadeOut, 0.0, 1.0 ), uFade.w ) * uFade.z * aMisc.y;

	vec4 mvPosition = viewMatrix * vec4( aPos, 1.0 );

	vec2 q = position.xy;              // unit quad, -0.5 .. 0.5
	vLocal = q * 2.0;

	float cr = cos( aAttr.z ), sr = sin( aAttr.z );
	vec2 offset = vec2( q.x * cr - q.y * sr, q.x * sr + q.y * cr ) * size;

#ifdef STRETCH
	// Align the quad's long axis with the screen-space velocity so fast debris
	// smears the way a real photograph of it would.
	vec3 vView = ( viewMatrix * vec4( aVel, 0.0 ) ).xyz;
	float vLen = length( vView.xy );
	if ( vLen > 1.0e-3 ) {
		vec2 axis = vView.xy / vLen;
		float elong = 1.0 + uStretch * length( aVel );
		offset = axis * ( q.y * size * elong ) + vec2( -axis.y, axis.x ) * ( q.x * size );
	}
#endif

	mvPosition.xy += offset;
	gl_Position = projectionMatrix * mvPosition;

	// World position without a matrix inverse: a view matrix has an orthonormal
	// upper 3x3, so v * mat3( viewMatrix ) == transpose( R ) * v == R^-1 * v.
	vWorldPos = cameraPosition + ( mvPosition.xyz * mat3( viewMatrix ) );

	vColor = aColor;
	vGround = aMisc.x;

	// The atlas is uploaded with flipY = false, so texture v = 0 is canvas row 0
	// and the tile index maps straight through — which also keeps each sprite's
	// authored orientation (the splash crown fans up, leaf tips point up).
	float col = mod( aAttr.w, uTileCols );
	float row = floor( aAttr.w / uTileCols );
	vec2 tileUv = mix( vec2( uTileInset ), vec2( 1.0 - uTileInset ), uv );
	vUv = ( vec2( col, row ) + tileUv ) / uTileCols;

	#include <fog_vertex>
}
`;

const PARTICLE_FRAG = /* glsl */`
uniform sampler2D uAtlas;
uniform vec3  uSunDir;      // unit vector TOWARDS the sun
uniform vec3  uSunColor;    // sun radiance / PI
uniform vec3  uAmbColor;    // sky irradiance / PI
uniform vec4  uLight;       // ambient mul, diffuse mul, forward-scatter mul, emissive
uniform vec2  uSoft;        // ground soft distance (m), near-camera fade (m)
uniform float uPhaseG;
uniform float uWrap;        // fraction of the sun term that is direction-independent

varying vec2  vUv;
varying vec3  vColor;
varying float vAlpha;
varying vec3  vWorldPos;
varying float vGround;
varying vec2  vLocal;

#include <common>
#include <fog_pars_fragment>

void main() {
	vec4 texel = texture2D( uAtlas, vUv );
	float alpha = texel.a * vAlpha;
	if ( alpha < 0.004 ) discard;

	// Treat the sprite as a sphere so the lighting has a form to sit on.
	float r2 = clamp( dot( vLocal, vLocal ), 0.0, 1.0 );
	vec3 nView = vec3( vLocal, sqrt( max( 1.0 - r2, 0.04 ) ) );
	vec3 nWorld = normalize( nView * mat3( viewMatrix ) );

	vec3 toFrag = vWorldPos - cameraPosition;
	float dist = length( toFrag );
	vec3 viewDir = toFrag / max( dist, 1.0e-4 );

	// Wrapped diffuse: particulate is not opaque, so light bleeds well past the
	// terminator. Without this dust reads as a solid grey lump.
	float ndl = dot( nWorld, uSunDir );
	float wrapped = clamp( ( ndl + 0.55 ) / 1.55, 0.0, 1.0 );

	// A camera-facing billboard's fake normal points at the viewer, so with the
	// sun off to the side the wrap term collapses and the sprite falls back to pure
	// ambient — which is why naive dust reads as a dark smear instead of a bright
	// plume. Real particulate is optically thick enough to multiple-scatter, and
	// that component does not care where the sun is. uWrap is that floor.
	wrapped = mix( wrapped, 1.0, uWrap );

	// Henyey-Greenstein forward scattering, normalised to 1 along the sun
	// direction so it is a bounded gain and not an unbounded spike. This is what
	// makes a backlit dust plume glow.
	float cosT = dot( viewDir, uSunDir );
	float g2 = uPhaseG * uPhaseG;
	float den = max( 1.0 + g2 - 2.0 * uPhaseG * cosT, 1.0e-4 );
	float peak = ( 1.0 - g2 ) / pow( max( 1.0 - uPhaseG, 1.0e-3 ), 3.0 );
	float phase = ( ( 1.0 - g2 ) / ( den * sqrt( den ) ) ) / max( peak, 1.0e-4 );
	// Keep a floor under the lobe: even away from the sun, a plume in open air
	// is lit by the whole sky and never drops to nothing.
	phase = 0.20 + 0.80 * phase;

	float detail = 0.55 + 0.62 * texel.r;
	vec3 lit = vColor * detail * ( uAmbColor * uLight.x + uSunColor * wrapped * uLight.y );
	lit += uSunColor * vColor * phase * uLight.z * ( 1.0 - r2 * 0.45 );
	lit += vColor * uLight.w;

	// Soft intersection with the ground: fade the part of the sprite that is
	// below the heightfield instead of letting it clip against the terrain.
	alpha *= clamp( ( vWorldPos.y - vGround ) / max( uSoft.x, 1.0e-3 ), 0.0, 1.0 );
	// ...and against the near plane, so nothing becomes a full-screen smear.
	alpha *= smoothstep( uSoft.y * 0.30, uSoft.y, dist );

	gl_FragColor = vec4( lit, clamp( alpha, 0.0, 1.0 ) );

	// Same tail as every stock material, in the same order. On the post-processing
	// path both of these are no-ops (linear HDR target, tone mapping owned by
	// postfx); on main.js's direct-render fallback they are what keeps particles
	// at the same exposure and encoding as the terrain instead of ~1.5 stops dark.
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}
`;

const TRACK_VERT = /* glsl */`
attribute vec2 aInfo;    // x birth time, y strength
attribute vec3 aTint;    // surface albedo, darkened and pre-lit on the CPU

uniform float uTime;
uniform float uLife;

varying vec2  vUv;
varying vec3  vTint;
varying float vFade;
varying float vStrength;

#include <common>
#include <fog_pars_vertex>

void main() {
	vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );
	gl_Position = projectionMatrix * mvPosition;

	vUv = uv;
	vTint = aTint;
	vStrength = aInfo.y;

	float age = clamp( ( uTime - aInfo.x ) / max( uLife, 1.0e-3 ), 0.0, 1.0 );
	// Tracks hold for most of their life then go quickly — a linear fade reads
	// as the whole trail dimming at once.
	vFade = pow( 1.0 - age, 1.4 );

	#include <fog_vertex>
}
`;

const TRACK_FRAG = /* glsl */`
uniform sampler2D uTread;
uniform vec2  uTreadScale;
uniform float uOpacity;

varying vec2  vUv;
varying vec3  vTint;
varying float vFade;
varying float vStrength;

#include <common>
#include <fog_pars_fragment>

void main() {
	float across = vUv.y;                       // -1 .. 1 across the tyre
	float edge = 1.0 - smoothstep( 0.52, 1.0, abs( across ) );
	float body = vFade * vStrength * edge;
	if ( body < 0.004 ) discard;

	vec2 tuv = vec2( vUv.x * uTreadScale.x, ( across * 0.5 + 0.5 ) * uTreadScale.y );
	float tread = texture2D( uTread, tuv ).r;

	// The knobs press material down and leave the darkest imprint; the gaps
	// between them stay closer to the untouched surface.
	vec3 c = vTint * mix( 0.86, 0.30, tread );

	// Loose material displaced to the shoulders catches the light, which is what
	// makes a real tyre track visible from a distance.
	float shoulder = smoothstep( 0.40, 0.72, abs( across ) ) * ( 1.0 - smoothstep( 0.78, 1.0, abs( across ) ) );
	c = mix( c, vTint * 1.45, shoulder * 0.40 );

	float alpha = uOpacity * body * mix( 0.50, 1.0, tread );

	gl_FragColor = vec4( c, clamp( alpha, 0.0, 1.0 ) );

	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}
`;

// ---------------------------------------------------------------------------
// Module-scope scratch. Nothing below allocates once construction is done.
// ---------------------------------------------------------------------------

const _fwd = new THREE.Vector3(0, 0, -1);
const _right = new THREE.Vector3(1, 0, 0);
const _up = new THREE.Vector3(0, 1, 0);
const _dir = new THREE.Vector3();
const _cp = new THREE.Vector3();
const _cpPrev = new THREE.Vector3();
const _nrm = new THREE.Vector3(0, 1, 0);
const _tmp = new THREE.Vector3();
const _tmp2 = new THREE.Vector3();
const _camFwd = new THREE.Vector3();
const _sunDir = new THREE.Vector3(0.4, 0.7, 0.55);
const _WORLD_UP = new THREE.Vector3(0, 1, 0);

/** Shared spawn record — filled by the caller, consumed synchronously. */
const SP = {
  x: 0, y: 0, z: 0,
  vx: 0, vy: 0, vz: 0,
  size: 0.2, life: 1, tile: 0,
  r: 1, g: 1, b: 1,
  ground: -1e9, alpha: 1,
  grav: -GRAVITY, drag: 1, rot: 0, rotv: 0,
};

function resetSP() {
  SP.x = SP.y = SP.z = 0;
  SP.vx = SP.vy = SP.vz = 0;
  SP.size = 0.2; SP.life = 1; SP.tile = 0;
  SP.r = SP.g = SP.b = 1;
  SP.ground = -1e9; SP.alpha = 1;
  SP.grav = -GRAVITY; SP.drag = 1;
  SP.rot = 0; SP.rotv = 0;
}

const num = (v, d) => (typeof v === 'number' && isFinite(v) ? v : d);
const rand = Math.random;
const randSym = () => Math.random() * 2 - 1;

// ---------------------------------------------------------------------------
// One instanced sprite system = one effect family = one draw call.
// ---------------------------------------------------------------------------

function createSystem(cfg, atlas, quadGeometry) {
  const cap = cfg.max;

  // Structure-of-arrays particle pool.
  const px = new Float32Array(cap), py = new Float32Array(cap), pz = new Float32Array(cap);
  const vx = new Float32Array(cap), vy = new Float32Array(cap), vz = new Float32Array(cap);
  const age = new Float32Array(cap), arate = new Float32Array(cap);
  const size = new Float32Array(cap), tile = new Float32Array(cap);
  const rot = new Float32Array(cap), rotv = new Float32Array(cap);
  const cr = new Float32Array(cap), cg = new Float32Array(cap), cb = new Float32Array(cap);
  const gnd = new Float32Array(cap), alp = new Float32Array(cap);
  const grav = new Float32Array(cap), drg = new Float32Array(cap);

  let count = 0;
  let cursor = 0;

  const geom = new THREE.InstancedBufferGeometry();
  geom.index = quadGeometry.index;
  geom.setAttribute('position', quadGeometry.getAttribute('position'));
  geom.setAttribute('uv', quadGeometry.getAttribute('uv'));
  geom.instanceCount = 0;

  const aPos = new THREE.InstancedBufferAttribute(new Float32Array(cap * 3), 3);
  const aAttr = new THREE.InstancedBufferAttribute(new Float32Array(cap * 4), 4);
  const aColor = new THREE.InstancedBufferAttribute(new Float32Array(cap * 3), 3);
  const aMisc = new THREE.InstancedBufferAttribute(new Float32Array(cap * 2), 2);
  aPos.setUsage(THREE.DynamicDrawUsage);
  aAttr.setUsage(THREE.DynamicDrawUsage);
  aColor.setUsage(THREE.DynamicDrawUsage);
  aMisc.setUsage(THREE.DynamicDrawUsage);
  geom.setAttribute('aPos', aPos);
  geom.setAttribute('aAttr', aAttr);
  geom.setAttribute('aColor', aColor);
  geom.setAttribute('aMisc', aMisc);

  let aVel = null;
  if (cfg.stretch) {
    aVel = new THREE.InstancedBufferAttribute(new Float32Array(cap * 3), 3);
    aVel.setUsage(THREE.DynamicDrawUsage);
    geom.setAttribute('aVel', aVel);
  }

  // Positions are world space and the mesh sits at the origin, so culling and
  // the auto-computed bounds are both meaningless here.
  geom.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), 1e6);
  geom.boundingBox = new THREE.Box3(
    new THREE.Vector3(-1e6, -1e6, -1e6), new THREE.Vector3(1e6, 1e6, 1e6));

  // Merge ONLY the fog block: `UniformsUtils.merge` deep-clones anything with a
  // `.clone()` (including textures), so passing the atlas through it would put
  // seven copies of the same 512x512 image on the GPU.
  const uniforms = THREE.UniformsUtils.merge([THREE.UniformsLib.fog]);
  // Re-point the shared height-fog store explicitly (ADDENDUM §B): cloning
  // copies the Float32Array by reference today, but this makes it unbreakable.
  if (THREE.UniformsLib.fog && THREE.UniformsLib.fog.hFogParams) {
    uniforms.hFogParams = THREE.UniformsLib.fog.hFogParams;
  }
  uniforms.uAtlas = { value: atlas };
  uniforms.uTileCols = { value: ATLAS_COLS };
  uniforms.uTileInset = { value: ATLAS_INSET };
  uniforms.uGrow = { value: new THREE.Vector2(cfg.grow[0], cfg.grow[1]) };
  uniforms.uFade = { value: new THREE.Vector4(cfg.fade[0], cfg.fade[1], cfg.fade[2], cfg.fade[3]) };
  uniforms.uStretch = { value: cfg.stretchAmount || 0 };
  uniforms.uSunDir = { value: new THREE.Vector3(0.4, 0.7, 0.55) };
  uniforms.uSunColor = { value: new THREE.Color(1, 0.96, 0.88) };
  uniforms.uAmbColor = { value: new THREE.Color(0.34, 0.42, 0.55) };
  uniforms.uLight = { value: new THREE.Vector4(cfg.light[0], cfg.light[1], cfg.light[2], cfg.light[3]) };
  uniforms.uSoft = { value: new THREE.Vector2(cfg.soft[0], cfg.soft[1]) };
  uniforms.uPhaseG = { value: cfg.phaseG != null ? cfg.phaseG : 0.62 };
  uniforms.uWrap = { value: cfg.wrap != null ? cfg.wrap : 0.2 };

  const material = new THREE.ShaderMaterial({
    uniforms,
    vertexShader: PARTICLE_VERT,
    fragmentShader: PARTICLE_FRAG,
    defines: cfg.stretch ? { STRETCH: '' } : {},
    transparent: true,
    depthWrite: false,
    depthTest: true,
    blending: cfg.blending || THREE.NormalBlending,
    side: THREE.DoubleSide,
    fog: true,
    toneMapped: true,    // no-op under postfx (renderer tone mapping is off)
  });

  const mesh = new THREE.Mesh(geom, material);
  mesh.name = `particles:${cfg.key}`;
  mesh.frustumCulled = false;
  mesh.castShadow = false;
  mesh.receiveShadow = false;
  mesh.renderOrder = cfg.renderOrder != null ? cfg.renderOrder : 10;
  mesh.visible = false;
  mesh.matrixAutoUpdate = false;
  mesh.updateMatrix();

  // Persistent update-range records: three empties `updateRanges` after each
  // upload, so re-pushing the same object keeps this allocation-free.
  const ranges = [
    { attr: aPos, rec: { start: 0, count: 0 }, item: 3 },
    { attr: aAttr, rec: { start: 0, count: 0 }, item: 4 },
    { attr: aColor, rec: { start: 0, count: 0 }, item: 3 },
    { attr: aMisc, rec: { start: 0, count: 0 }, item: 2 },
  ];
  if (aVel) ranges.push({ attr: aVel, rec: { start: 0, count: 0 }, item: 3 });

  function markRanges(n) {
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      r.attr.needsUpdate = true;
      const list = r.attr.updateRanges;
      if (!list) continue;
      r.rec.start = 0;
      r.rec.count = n * r.item;
      if (list.length === 0) list.push(r.rec);
      else { list.length = 1; list[0] = r.rec; }
    }
  }

  function copy(from, to) {
    px[to] = px[from]; py[to] = py[from]; pz[to] = pz[from];
    vx[to] = vx[from]; vy[to] = vy[from]; vz[to] = vz[from];
    age[to] = age[from]; arate[to] = arate[from];
    size[to] = size[from]; tile[to] = tile[from];
    rot[to] = rot[from]; rotv[to] = rotv[from];
    cr[to] = cr[from]; cg[to] = cg[from]; cb[to] = cb[from];
    gnd[to] = gnd[from]; alp[to] = alp[from];
    grav[to] = grav[from]; drg[to] = drg[from];
  }

  /** Consumes the module-scope SP record. */
  function spawn() {
    let i;
    if (count < cap) {
      i = count++;
    } else {
      // Pool exhausted: recycle round-robin rather than dropping the spawn, so
      // a burst still reads at full strength.
      i = cursor % cap;
      cursor++;
    }
    px[i] = SP.x; py[i] = SP.y; pz[i] = SP.z;
    vx[i] = SP.vx; vy[i] = SP.vy; vz[i] = SP.vz;
    age[i] = 0;
    arate[i] = 1 / Math.max(SP.life, 0.02);
    size[i] = SP.size;
    tile[i] = SP.tile;
    rot[i] = SP.rot; rotv[i] = SP.rotv;
    cr[i] = SP.r; cg[i] = SP.g; cb[i] = SP.b;
    gnd[i] = SP.ground; alp[i] = SP.alpha;
    grav[i] = SP.grav; drg[i] = SP.drag;
  }

  const collide = !!cfg.collide;
  const bounce = cfg.bounce != null ? cfg.bounce : 0.2;
  const friction = cfg.friction != null ? cfg.friction : 0.5;
  const settleFade = cfg.settleFade != null ? cfg.settleFade : 2.5;
  const turb = cfg.turbulence || 0;
  const windX = cfg.wind ? cfg.wind[0] : 0;
  const windZ = cfg.wind ? cfg.wind[1] : 0;
  const stretch = !!cfg.stretch;

  function update(dt, time, groundAt) {
    if (count === 0) {
      if (mesh.visible) { mesh.visible = false; geom.instanceCount = 0; }
      return 0;
    }

    const pa = aPos.array, aa = aAttr.array, ca = aColor.array, ma = aMisc.array;
    const va = aVel ? aVel.array : null;

    let n = 0;
    for (let i = 0; i < count; i++) {
      let a = age[i] + arate[i] * dt;
      if (a >= 1) continue;

      let x = px[i], y = py[i], z = pz[i];
      let ux = vx[i], uy = vy[i], uz = vz[i];

      // Drag pulls the particle towards the ambient air velocity, not towards
      // zero — that is what lets a plume drift and hang rather than stopping
      // dead in mid-air.
      const k = Math.min(drg[i] * dt, 1);
      ux += (windX - ux) * k;
      uz += (windZ - uz) * k;
      uy += -uy * k + grav[i] * dt;

      if (turb > 0) {
        // Two cheap decorrelated sines standing in for a curl-noise field.
        const ph = rot[i];
        const s1 = Math.sin(x * 0.42 + time * 0.9 + ph);
        const s2 = Math.sin(z * 0.37 - time * 0.7 + ph * 1.7);
        ux += s1 * turb * dt;
        uz += s2 * turb * dt;
        uy += s1 * s2 * turb * 0.55 * dt;
      }

      x += ux * dt; y += uy * dt; z += uz * dt;

      let g = gnd[i];
      if (groundAt && ((i + (time * 60) | 0) & 7) === 0) {
        // Restagger the terrain query across frames: 1/8 of the pool per frame
        // is plenty to keep the soft-fade reference current as a particle drifts.
        g = groundAt(x, z);
        gnd[i] = g;
      }

      if (collide && y < g + 0.02) {
        y = g + 0.02;
        if (uy < 0) uy = -uy * bounce;
        ux *= friction; uz *= friction;
        // Settled material should stop being a flying chunk and start being a
        // fading smudge, so age it out faster once it is down.
        a += arate[i] * dt * settleFade;
        if (a >= 1) continue;
      }

      if (n !== i) copy(i, n);
      px[n] = x; py[n] = y; pz[n] = z;
      vx[n] = ux; vy[n] = uy; vz[n] = uz;
      age[n] = a;
      gnd[n] = g;
      rot[n] = rot[i] + rotv[i] * dt;

      const o3 = n * 3, o4 = n * 4, o2 = n * 2;
      pa[o3] = x; pa[o3 + 1] = y; pa[o3 + 2] = z;
      aa[o4] = a; aa[o4 + 1] = size[n]; aa[o4 + 2] = rot[n]; aa[o4 + 3] = tile[n];
      ca[o3] = cr[n]; ca[o3 + 1] = cg[n]; ca[o3 + 2] = cb[n];
      ma[o2] = g; ma[o2 + 1] = alp[n];
      if (va) { va[o3] = ux; va[o3 + 1] = uy; va[o3 + 2] = uz; }

      n++;
    }

    count = n;
    geom.instanceCount = n;
    mesh.visible = n > 0;
    if (n > 0) markRanges(n);
    return n;
  }

  function clear() {
    count = 0;
    geom.instanceCount = 0;
    mesh.visible = false;
  }

  function dispose() {
    geom.dispose();
    material.dispose();
  }

  return {
    key: cfg.key, mesh, material, uniforms, spawn, update, clear, dispose,
    get count() { return count; },
    get capacity() { return cap; },
    stretch,
  };
}

// ---------------------------------------------------------------------------
// Persistent tyre-track ribbons (rear + front in a single draw call).
// ---------------------------------------------------------------------------

function createTracks(tread) {
  const RIBBONS = 2;
  const quads = TRACK_SEGMENTS * RIBBONS;
  const verts = quads * 4;

  const posArr = new Float32Array(verts * 3);
  const uvArr = new Float32Array(verts * 2);
  const infoArr = new Float32Array(verts * 2);
  const tintArr = new Float32Array(verts * 3);
  const idxArr = new Uint32Array(quads * 6);

  for (let q = 0; q < quads; q++) {
    const v0 = q * 4, o = q * 6;
    idxArr[o] = v0; idxArr[o + 1] = v0 + 1; idxArr[o + 2] = v0 + 2;
    idxArr[o + 3] = v0; idxArr[o + 4] = v0 + 2; idxArr[o + 5] = v0 + 3;
  }

  const geom = new THREE.BufferGeometry();
  const aPosition = new THREE.BufferAttribute(posArr, 3).setUsage(THREE.DynamicDrawUsage);
  const aUv = new THREE.BufferAttribute(uvArr, 2).setUsage(THREE.DynamicDrawUsage);
  const aInfo = new THREE.BufferAttribute(infoArr, 2).setUsage(THREE.DynamicDrawUsage);
  const aTint = new THREE.BufferAttribute(tintArr, 3).setUsage(THREE.DynamicDrawUsage);
  geom.setAttribute('position', aPosition);
  geom.setAttribute('uv', aUv);
  geom.setAttribute('aInfo', aInfo);
  geom.setAttribute('aTint', aTint);
  geom.setIndex(new THREE.BufferAttribute(idxArr, 1));
  geom.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), 1e6);

  const uniforms = THREE.UniformsUtils.merge([THREE.UniformsLib.fog]);
  if (THREE.UniformsLib.fog && THREE.UniformsLib.fog.hFogParams) {
    uniforms.hFogParams = THREE.UniformsLib.fog.hFogParams;
  }
  uniforms.uTread = { value: tread };
  // x: tread repeats per metre along the track (a DH knob row is ~0.25 m).
  uniforms.uTreadScale = { value: new THREE.Vector2(4.0, 1.0) };
  uniforms.uOpacity = { value: 0.85 };
  uniforms.uTime = { value: 0 };
  uniforms.uLife = { value: TRACK_LIFE };

  const material = new THREE.ShaderMaterial({
    uniforms,
    vertexShader: TRACK_VERT,
    fragmentShader: TRACK_FRAG,
    transparent: true,
    depthWrite: false,
    depthTest: true,
    side: THREE.DoubleSide,
    fog: true,
    toneMapped: true,
    polygonOffset: true,
    polygonOffsetFactor: -4,
    polygonOffsetUnits: -8,
  });

  const mesh = new THREE.Mesh(geom, material);
  mesh.name = 'particles:tracks';
  mesh.frustumCulled = false;
  mesh.castShadow = false;
  mesh.receiveShadow = false;
  mesh.renderOrder = 2;      // ground decal: after opaque, before airborne sprites
  mesh.matrixAutoUpdate = false;
  mesh.updateMatrix();

  // Two persistent update-range records per attribute (one per ribbon), reused
  // every frame so uploading a freshly written quad never allocates.
  const ranges = [
    { attr: aPosition, item: 3, recs: [{ start: 0, count: 0 }, { start: 0, count: 0 }] },
    { attr: aUv, item: 2, recs: [{ start: 0, count: 0 }, { start: 0, count: 0 }] },
    { attr: aInfo, item: 2, recs: [{ start: 0, count: 0 }, { start: 0, count: 0 }] },
    { attr: aTint, item: 3, recs: [{ start: 0, count: 0 }, { start: 0, count: 0 }] },
  ];
  // Per-ribbon dirty window, flushed once per frame.
  const dirty = [{ lo: -1, hi: -1 }, { lo: -1, hi: -1 }];

  const ribbons = [];
  for (let r = 0; r < RIBBONS; r++) {
    ribbons.push({
      head: 0,
      hasPrev: false,
      dist: 0,
      prevLx: 0, prevLy: 0, prevLz: 0,
      prevRx: 0, prevRy: 0, prevRz: 0,
      prevCx: 0, prevCy: 0, prevCz: 0,
    });
  }

  /** `spans` is [lo0, hi0, lo1, hi1] in vertex indices; a lo < 0 means "skip". */
  function markVerts(lo0, hi0, lo1, hi1) {
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      r.attr.needsUpdate = true;
      const list = r.attr.updateRanges;
      if (!list) continue;                 // older three: full upload, still correct
      list.length = 0;
      if (lo0 >= 0) {
        r.recs[0].start = lo0 * r.item;
        r.recs[0].count = (hi0 - lo0 + 1) * r.item;
        list.push(r.recs[0]);
      }
      if (lo1 >= 0) {
        r.recs[1].start = lo1 * r.item;
        r.recs[1].count = (hi1 - lo1 + 1) * r.item;
        list.push(r.recs[1]);
      }
    }
  }

  function writeVertex(vi, x, y, z, u, v, birth, strength, tr, tg, tb) {
    const o3 = vi * 3, o2 = vi * 2;
    posArr[o3] = x; posArr[o3 + 1] = y; posArr[o3 + 2] = z;
    uvArr[o2] = u; uvArr[o2 + 1] = v;
    infoArr[o2] = birth; infoArr[o2 + 1] = strength;
    tintArr[o3] = tr; tintArr[o3 + 1] = tg; tintArr[o3 + 2] = tb;
  }

  /**
   * Appends one cross-section to ribbon `ri`. `cx/cy/cz` is the contact point,
   * `dx/dz` the horizontal travel direction, `nx/ny/nz` the ground normal.
   */
  function push(ri, cx, cy, cz, dx, dz, nx, ny, nz, halfWidth, strength, time,
                tr, tg, tb, terrain) {
    const rb = ribbons[ri];

    // Lateral = groundNormal x travelDirection, with travel = (dx, 0, dz).
    // Taking the cross product against the *ground* normal (not world up) is
    // what makes the ribbon bank with a berm instead of cutting into it.
    let lx = ny * dz;
    let ly = nz * dx - nx * dz;
    let lz = -ny * dx;
    const ll = Math.sqrt(lx * lx + ly * ly + lz * lz);
    if (ll < 1e-4) return;
    lx /= ll; ly /= ll; lz /= ll;

    const lxo = lx * halfWidth, lyo = ly * halfWidth, lzo = lz * halfWidth;
    let Lx = cx - lxo, Ly = cy - lyo, Lz = cz - lzo;
    let Rx = cx + lxo, Ry = cy + lyo, Rz = cz + lzo;
    // Re-project the edges onto the heightfield so a cambered or rutted tread
    // does not lift the ribbon off the ground on one side.
    if (terrain) {
      Ly = terrain.sampleHeight(Lx, Lz) + TRACK_LIFT;
      Ry = terrain.sampleHeight(Rx, Rz) + TRACK_LIFT;
    }

    if (rb.hasPrev) {
      const seg = Math.sqrt(
        (cx - rb.prevCx) * (cx - rb.prevCx) +
        (cy - rb.prevCy) * (cy - rb.prevCy) +
        (cz - rb.prevCz) * (cz - rb.prevCz));
      rb.dist += seg;

      const q = ri * TRACK_SEGMENTS + rb.head;
      const v0 = q * 4;
      const u0 = rb.dist - seg, u1 = rb.dist;
      writeVertex(v0 + 0, rb.prevLx, rb.prevLy, rb.prevLz, u0, -1, time, strength, tr, tg, tb);
      writeVertex(v0 + 1, rb.prevRx, rb.prevRy, rb.prevRz, u0, 1, time, strength, tr, tg, tb);
      writeVertex(v0 + 2, Rx, Ry, Rz, u1, 1, time, strength, tr, tg, tb);
      writeVertex(v0 + 3, Lx, Ly, Lz, u1, -1, time, strength, tr, tg, tb);

      const d = dirty[ri];
      if (d.lo < 0) { d.lo = v0; d.hi = v0 + 3; }
      else { if (v0 < d.lo) d.lo = v0; if (v0 + 3 > d.hi) d.hi = v0 + 3; }

      rb.head = (rb.head + 1) % TRACK_SEGMENTS;
    }

    rb.prevLx = Lx; rb.prevLy = Ly; rb.prevLz = Lz;
    rb.prevRx = Rx; rb.prevRy = Ry; rb.prevRz = Rz;
    rb.prevCx = cx; rb.prevCy = cy; rb.prevCz = cz;
    rb.hasPrev = true;
  }

  function breakRibbon(ri) {
    ribbons[ri].hasPrev = false;
  }

  function flush() {
    const a = dirty[0], b = dirty[1];
    if (a.lo < 0 && b.lo < 0) return;
    markVerts(a.lo, a.hi, b.lo, b.hi);
    a.lo = -1; a.hi = -1;
    b.lo = -1; b.hi = -1;
  }

  function clear() {
    infoArr.fill(0);
    posArr.fill(0);
    for (let r = 0; r < RIBBONS; r++) {
      ribbons[r].hasPrev = false;
      ribbons[r].head = 0;
      ribbons[r].dist = 0;
    }
    dirty[0].lo = -1; dirty[1].lo = -1;
    markVerts(0, verts - 1, -1, -1);
  }

  function dispose() {
    geom.dispose();
    material.dispose();
  }

  return {
    mesh, material, uniforms, push, breakRibbon, flush, clear, dispose, ribbons,
    prevDistanceOf(ri) { return ribbons[ri].dist; },
    hasPrev(ri) { return ribbons[ri].hasPrev; },
    prevCentre(ri, out) {
      const rb = ribbons[ri];
      out.set(rb.prevCx, rb.prevCy, rb.prevCz);
      return out;
    },
  };
}

// ---------------------------------------------------------------------------
// Family definitions.
// ---------------------------------------------------------------------------

const FAMILIES = [
  // `soft` is [ground-intersection fade (m), near-camera fade (m) ]. The near
  // value must stay well inside the chase camera's ~5 m stand-off or the entire
  // wake — which lives between the camera and the bike — fades out unseen.
  {
    key: 'dust', base: 560, renderOrder: 8,
    // Capped growth: past ~2.5x the puffs merge into one flat wash that reads as
    // ground fog rather than a plume with a tail.
    grow: [0.55, 2.5], fade: [0.16, 0.30, 0.78, 1.0],
    light: [1.45, 1.15, 1.20, 0.0], soft: [0.70, 1.20], phaseG: 0.70, wrap: 0.50,
    collide: false, turbulence: 0.55, wind: [WIND_X, WIND_Z],
  },
  {
    key: 'roost', base: 1150, renderOrder: 9, stretch: true, stretchAmount: 0.012,
    grow: [0.95, 1.35], fade: [0.05, 0.55, 1.0, 1.0],
    light: [1.0, 1.05, 0.12, 0.0], soft: [0.16, 0.55], phaseG: 0.45, wrap: 0.18,
    collide: true, bounce: 0.16, friction: 0.55, settleFade: 3.0,
  },
  {
    key: 'mud', base: 400, renderOrder: 9, stretch: true, stretchAmount: 0.018,
    grow: [0.95, 1.20], fade: [0.03, 0.62, 1.0, 1.0],
    light: [0.80, 0.90, 0.05, 0.0], soft: [0.14, 0.50], phaseG: 0.40, wrap: 0.14,
    collide: true, bounce: 0.04, friction: 0.22, settleFade: 5.0,
  },
  {
    key: 'chips', base: 440, renderOrder: 10, stretch: true, stretchAmount: 0.020,
    grow: [1.0, 1.0], fade: [0.02, 0.66, 1.0, 1.0],
    light: [0.95, 1.15, 0.06, 0.0], soft: [0.10, 0.45], phaseG: 0.40, wrap: 0.12,
    collide: true, bounce: 0.42, friction: 0.72, settleFade: 4.0,
  },
  {
    key: 'splash', base: 560, renderOrder: 10, stretch: true, stretchAmount: 0.026,
    grow: [0.85, 1.30], fade: [0.03, 0.48, 1.0, 1.0],
    light: [1.30, 1.05, 0.75, 0.02], soft: [0.18, 0.50], phaseG: 0.68, wrap: 0.34,
    collide: true, bounce: 0.12, friction: 0.35, settleFade: 6.0,
  },
  {
    key: 'foliage', base: 240, renderOrder: 10,
    grow: [1.0, 1.0], fade: [0.05, 0.70, 1.0, 1.0],
    light: [1.05, 1.10, 0.45, 0.0], soft: [0.10, 0.45], phaseG: 0.55, wrap: 0.28,
    collide: true, bounce: 0.10, friction: 0.18, settleFade: 3.0, turbulence: 1.30,
  },
  {
    key: 'grit', base: 190, renderOrder: 12, stretch: true, stretchAmount: 0.16,
    grow: [1.0, 1.0], fade: [0.14, 0.26, 0.42, 1.0],
    light: [1.10, 0.65, 0.35, 0.0], soft: [0.06, 0.28], phaseG: 0.62, wrap: 0.40,
    collide: false, turbulence: 0.8,
  },
];

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

export function createParticles(ctx) {
  const group = new THREE.Group();
  group.name = 'particles';
  group.matrixAutoUpdate = false;
  if (ctx.scene) ctx.scene.add(group);

  const seed = num(ctx.seed, 1);
  const atlas = buildAtlas(seed);
  const tread = buildTreadTexture(seed);

  // Shared unit quad. Its attribute buffers are referenced (not copied) by every
  // instanced geometry, so it must not be disposed while they live.
  const quad = new THREE.PlaneGeometry(1, 1);

  const systems = {};
  const systemList = [];
  const headroom = 1.45;   // covers the `ultra` particleDensity of 1.4
  for (let i = 0; i < FAMILIES.length; i++) {
    const cfg = FAMILIES[i];
    const sys = createSystem(
      Object.assign({}, cfg, { max: Math.ceil(cfg.base * headroom) }), atlas, quad);
    systems[cfg.key] = sys;
    systemList.push(sys);
    group.add(sys.mesh);
  }

  const tracks = createTracks(tread);
  group.add(tracks.mesh);

  const dust = systems.dust, roost = systems.roost, mud = systems.mud;
  const chips = systems.chips, splash = systems.splash;
  const foliage = systems.foliage, grit = systems.grit;

  // ---- emission state (closure scope, never reallocated) -------------------
  let accRoost = 0, accDust = 0, accChips = 0, accMud = 0, accFoliage = 0, accGrit = 0;
  let skidBoost = 0;             // decays; driven by `bike:skid`
  let skidSurface = Surface.DIRT;
  let density = 1;
  let lastBikeX = 0, lastBikeY = 0, lastBikeZ = 0;
  let hadBikePos = false;
  let wasAirborne = false;
  let lastImpactAt = -99;      // so a self-triggered landing cannot double up
  let elapsed = 0;

  const terrainOf = () => ctx.terrain;

  function groundAt(x, z) {
    const t = terrainOf();
    if (!t || !t.sampleHeight) return -1e9;
    if (t.inBounds && !t.inBounds(x, z)) return -1e9;
    const h = t.sampleHeight(x, z);
    return isFinite(h) ? h : -1e9;
  }

  function materialAt(x, z, fallback) {
    const t = terrainOf();
    if (!t || !t.sampleMaterial) return fallback;
    const m = t.sampleMaterial(x, z);
    return (typeof m === 'number' && isFinite(m)) ? m : fallback;
  }

  function wetnessAt(x, z, surf) {
    const t = terrainOf();
    let w = 0;
    if (t && t.sampleWetness) {
      const s = t.sampleWetness(x, z);
      if (typeof s === 'number' && isFinite(s)) w = clamp01(s);
    }
    if (surf === Surface.MUD) w = Math.max(w, 0.85);
    if (surf === Surface.SNOW) w = Math.max(w, 0.35);
    return w;
  }

  // ---- lighting uniforms ---------------------------------------------------
  const sunColor = new THREE.Color(1, 0.96, 0.88);
  const ambColor = new THREE.Color(0.30, 0.38, 0.50);

  function refreshLighting() {
    if (ctx.sky && ctx.sky.sunDirection) {
      _sunDir.copy(ctx.sky.sunDirection);
      if (_sunDir.lengthSq() < 1e-6) _sunDir.set(0.4, 0.7, 0.55);
      _sunDir.normalize();
    } else if (ctx.sun && ctx.sun.position) {
      _sunDir.copy(ctx.sun.position);
      if (ctx.sun.target && ctx.sun.target.position) _sunDir.sub(ctx.sun.target.position);
      if (_sunDir.lengthSq() < 1e-6) _sunDir.set(0.4, 0.7, 0.55);
      _sunDir.normalize();
    }

    // Diffuse radiance for a Lambertian surface is albedo * E / PI, which is how
    // MeshStandardMaterial renders the terrain — matching it keeps particles
    // sitting in the same exposure as the ground they came off.
    const intensity = ctx.sun ? num(ctx.sun.intensity, 3.0) : 3.0;
    if (ctx.sun && ctx.sun.color) sunColor.copy(ctx.sun.color);
    else sunColor.setRGB(1, 0.96, 0.88);
    sunColor.multiplyScalar(intensity / Math.PI);

    // Sky irradiance: the fog colour is the atmosphere sky.js already solved
    // for, so it is the cheapest correct-looking ambient available.
    if (ctx.scene && ctx.scene.fog && ctx.scene.fog.color) {
      ambColor.copy(ctx.scene.fog.color).multiplyScalar(0.55);
    }

    for (let i = 0; i < systemList.length; i++) {
      const u = systemList[i].uniforms;
      u.uSunDir.value.copy(_sunDir);
      u.uSunColor.value.copy(sunColor);
      u.uAmbColor.value.copy(ambColor);
    }
  }

  // ---- basis helpers -------------------------------------------------------

  function bikeBasis(st) {
    if (st && st.forward && st.forward.lengthSq() > 1e-6) _fwd.copy(st.forward).normalize();
    else if (st && st.velocity && st.velocity.lengthSq() > 1e-4) _fwd.copy(st.velocity).normalize();
    else _fwd.set(0, 0, -1);
    if (st && st.up && st.up.lengthSq() > 1e-6) _up.copy(st.up).normalize();
    else _up.copy(_WORLD_UP);
    _right.crossVectors(_fwd, _up);
    if (_right.lengthSq() < 1e-6) _right.set(1, 0, 0);
    _right.normalize();
  }

  // ---- spawn helpers -------------------------------------------------------

  function tintFrom(table, surf, jitter) {
    const c = table[surfaceIndex(surf)];
    const j = 1 + (rand() - 0.5) * jitter;
    SP.r = clamp(c[0] * j, 0, 4);
    SP.g = clamp(c[1] * j, 0, 4);
    SP.b = clamp(c[2] * j, 0, 4);
  }

  /**
   * One roost particle: thrown from the contact patch, arcing back and out.
   * `slip` widens the cone and lifts the exit speed; `braking` flips the throw
   * forwards because the tyre is being dragged over the ground, not driving it.
   */
  function spawnRoost(cx, cy, cz, speed, slip, lateral, braking, surf, wet, ground) {
    resetSP();
    const spread = 0.16 + slip * 0.60 + Math.abs(lateral) * 0.55;

    // Base throw: back and up off the tyre's trailing edge.
    _dir.copy(_fwd).multiplyScalar(braking > 0.5 ? 0.45 : -0.85);
    _dir.addScaledVector(_up, 0.55 + rand() * 0.35);
    // Drifting sprays to the outside of the slide.
    _dir.addScaledVector(_right, -lateral * (0.5 + rand() * 0.8));
    // Cone jitter.
    _dir.x += randSym() * spread;
    _dir.y += randSym() * spread * 0.55;
    _dir.z += randSym() * spread;
    _dir.normalize();

    const exit = (1.4 + speed * (0.14 + slip * 0.30)) * (0.55 + rand() * 0.85);
    // Spawn at the top of the contact patch, not the ground plane: material
    // leaves the tyre where it stops being pinched, and starting on the ground
    // would put every particle inside the soft-fade band at birth.
    SP.x = cx + randSym() * 0.09;
    SP.y = cy + 0.07 + rand() * 0.13;
    SP.z = cz + randSym() * 0.09;
    SP.vx = _dir.x * exit;
    SP.vy = _dir.y * exit;
    SP.vz = _dir.z * exit;

    const heavy = wet > 0.5;
    const table = heavy ? COL_WET : COL_CHUNK;
    tintFrom(table, surf, 0.24);
    SP.size = (0.065 + rand() * 0.135) * (heavy ? 1.25 : 1.0);
    SP.life = 0.55 + rand() * 0.85;
    SP.tile = rand() < 0.45 ? T_GRIT : (rand() < 0.5 ? T_CLOD_A : T_CLOD_B);
    SP.ground = ground;
    SP.alpha = 0.85 + rand() * 0.15;
    SP.grav = -GRAVITY * (0.9 + rand() * 0.35);
    SP.drag = 1.1 + rand() * 1.1;
    SP.rot = rand() * 6.283;
    SP.rotv = randSym() * 7.0;
    roost.spawn();
  }

  function spawnDust(cx, cy, cz, speed, surf, ground, scale) {
    resetSP();
    // Dust is lifted by the wake, not thrown: it should barely move relative to
    // the ground and then hang there while the rider leaves it behind.
    _dir.copy(_fwd).multiplyScalar(-(0.10 + rand() * 0.22) * speed);
    SP.x = cx + randSym() * 0.55;
    SP.y = cy + 0.30 + rand() * 0.45;
    SP.z = cz + randSym() * 0.55;
    SP.vx = _dir.x + randSym() * 0.7 + _right.x * randSym() * 0.9;
    // Rises enough to clear the rider's shoulders in a chase view — dust that
    // stays at wheel height is hidden by the bike and never reads as speed.
    SP.vy = 0.75 + rand() * 1.25;
    SP.vz = _dir.z + randSym() * 0.7 + _right.z * randSym() * 0.9;
    tintFrom(COL_DUST, surf, 0.14);
    SP.size = (0.55 + rand() * 0.85) * scale;
    SP.life = 2.2 + rand() * 3.0;
    SP.tile = rand() < 0.45 ? T_DUST_A : (rand() < 0.6 ? T_DUST_B : T_SMOKE);
    SP.ground = ground;
    SP.alpha = 0.34 + rand() * 0.42;
    // Slightly buoyant: fine particulate in a warm wake rises before it settles.
    SP.grav = -0.15 + rand() * 0.35;
    SP.drag = 0.85 + rand() * 0.7;
    SP.rot = rand() * 6.283;
    SP.rotv = randSym() * 0.7;
    dust.spawn();
  }

  function spawnChip(cx, cy, cz, speed, slip, surf, ground) {
    resetSP();
    const sparky = rand() < 0.10;
    _dir.copy(_fwd).multiplyScalar(-0.7 - rand() * 0.6);
    _dir.addScaledVector(_up, 0.35 + rand() * 0.55);
    _dir.addScaledVector(_right, randSym() * 1.1);
    _dir.normalize();
    const exit = (2.5 + speed * (0.20 + slip * 0.35)) * (0.6 + rand() * 0.9);
    SP.x = cx + randSym() * 0.10;
    SP.y = cy + 0.04;
    SP.z = cz + randSym() * 0.10;
    SP.vx = _dir.x * exit; SP.vy = _dir.y * exit; SP.vz = _dir.z * exit;
    if (sparky) {
      SP.r = COL_SPARK[0]; SP.g = COL_SPARK[1]; SP.b = COL_SPARK[2];
      SP.tile = T_SPARK;
      SP.size = 0.035 + rand() * 0.035;
      SP.life = 0.16 + rand() * 0.22;
      SP.alpha = 0.9;
    } else {
      tintFrom(COL_CHUNK, surf, 0.3);
      SP.tile = rand() < 0.55 ? T_CHIP : T_GRIT;
      SP.size = 0.030 + rand() * 0.070;
      SP.life = 0.7 + rand() * 1.0;
      SP.alpha = 1.0;
    }
    SP.ground = ground;
    SP.grav = -GRAVITY * (1.05 + rand() * 0.30);
    SP.drag = 0.25 + rand() * 0.45;
    SP.rot = rand() * 6.283;
    SP.rotv = randSym() * 14.0;
    chips.spawn();
  }

  function spawnMud(cx, cy, cz, speed, slip, lateral, surf, ground) {
    resetSP();
    // Wet material is heavy: it leaves the tyre low and fast and lands quickly.
    _dir.copy(_fwd).multiplyScalar(-0.95);
    _dir.addScaledVector(_up, 0.28 + rand() * 0.30);
    _dir.addScaledVector(_right, -lateral * (0.6 + rand() * 0.9) + randSym() * 0.35);
    _dir.normalize();
    const exit = (1.6 + speed * (0.16 + slip * 0.24)) * (0.6 + rand() * 0.8);
    SP.x = cx + randSym() * 0.10;
    SP.y = cy + 0.05;
    SP.z = cz + randSym() * 0.10;
    SP.vx = _dir.x * exit; SP.vy = _dir.y * exit; SP.vz = _dir.z * exit;
    tintFrom(COL_WET, surf, 0.20);
    SP.tile = rand() < 0.7 ? T_GOB : T_CLOD_A;
    SP.size = 0.055 + rand() * 0.11;
    SP.life = 0.45 + rand() * 0.55;
    SP.ground = ground;
    SP.alpha = 1.0;
    SP.grav = -GRAVITY * (1.0 + rand() * 0.25);
    SP.drag = 1.6 + rand() * 1.2;
    SP.rot = rand() * 6.283;
    SP.rotv = randSym() * 9.0;
    mud.spawn();
  }

  function spawnFoliage(cx, cy, cz, speed, ground) {
    resetSP();
    const needle = rand() < 0.55;
    _dir.copy(_fwd).multiplyScalar(-0.6 - rand() * 0.5);
    _dir.addScaledVector(_up, 0.55 + rand() * 0.7);
    _dir.addScaledVector(_right, randSym() * 1.2);
    _dir.normalize();
    const exit = (1.0 + speed * 0.20) * (0.5 + rand() * 0.9);
    SP.x = cx + randSym() * 0.45;
    SP.y = cy + 0.05 + rand() * 0.15;
    SP.z = cz + randSym() * 0.45;
    SP.vx = _dir.x * exit; SP.vy = _dir.y * exit; SP.vz = _dir.z * exit;
    const c = needle ? COL_LEAF[2] : COL_LEAF[rand() < 0.5 ? 0 : 1];
    const j = 0.85 + rand() * 0.3;
    SP.r = c[0] * j; SP.g = c[1] * j; SP.b = c[2] * j;
    SP.tile = needle ? T_NEEDLE : T_LEAF;
    SP.size = needle ? 0.07 + rand() * 0.06 : 0.09 + rand() * 0.09;
    SP.life = 1.4 + rand() * 2.2;
    SP.ground = ground;
    SP.alpha = 1.0;
    // Leaves flutter: very low effective gravity and heavy drag.
    SP.grav = -GRAVITY * (0.10 + rand() * 0.10);
    SP.drag = 2.2 + rand() * 1.4;
    SP.rot = rand() * 6.283;
    SP.rotv = randSym() * 12.0;
    foliage.spawn();
  }

  /** Water: droplets, a crown sheet and lingering foam/mist. */
  function emitSplash(x, y, z, vxIn, vyIn, vzIn, strength) {
    const s = clamp(num(strength, 1), 0, 3);
    const ground = groundAt(x, z);
    const drops = Math.round(26 * s * density);
    for (let i = 0; i < drops; i++) {
      resetSP();
      const a = rand() * 6.283;
      const rr = 0.5 + rand() * 2.2;
      const up = 1.6 + rand() * 3.4 * s;
      SP.x = x + Math.cos(a) * 0.12 * rr;
      SP.y = y + 0.02 + rand() * 0.10;
      SP.z = z + Math.sin(a) * 0.12 * rr;
      SP.vx = Math.cos(a) * rr * (0.8 + s * 0.5) - num(vxIn, 0) * 0.18;
      SP.vy = up;
      SP.vz = Math.sin(a) * rr * (0.8 + s * 0.5) - num(vzIn, 0) * 0.18;
      const foam = rand() < 0.35;
      const c = foam ? COL_FOAM : COL_WATER;
      SP.r = c[0]; SP.g = c[1]; SP.b = c[2];
      SP.tile = foam ? T_FOAM : T_DROP;
      SP.size = foam ? 0.09 + rand() * 0.12 : 0.035 + rand() * 0.065;
      SP.life = 0.45 + rand() * 0.75;
      SP.ground = ground;
      SP.alpha = foam ? 0.8 : 0.62;
      SP.grav = -GRAVITY;
      SP.drag = 0.5 + rand() * 0.8;
      SP.rot = rand() * 6.283;
      SP.rotv = randSym() * 5;
      splash.spawn();
    }
    // The sheet: a couple of big crown sprites standing up out of the water.
    const sheets = Math.max(2, Math.round(4 * s * density));
    for (let i = 0; i < sheets; i++) {
      resetSP();
      const a = rand() * 6.283;
      SP.x = x + Math.cos(a) * 0.16;
      SP.y = y + 0.04;
      SP.z = z + Math.sin(a) * 0.16;
      SP.vx = Math.cos(a) * (0.7 + rand() * 1.1);
      SP.vy = 1.1 + rand() * 1.6 * s;
      SP.vz = Math.sin(a) * (0.7 + rand() * 1.1);
      SP.r = COL_FOAM[0]; SP.g = COL_FOAM[1]; SP.b = COL_FOAM[2];
      SP.tile = T_SHEET;
      SP.size = (0.45 + rand() * 0.5) * (0.7 + s * 0.5);
      SP.life = 0.30 + rand() * 0.30;
      SP.ground = ground;
      SP.alpha = 0.55;
      SP.grav = -GRAVITY * 0.5;
      SP.drag = 2.2;
      SP.rot = randSym() * 0.4;
      SP.rotv = randSym() * 1.2;
      splash.spawn();
    }
    // Mist hanging over the crossing.
    const mist = Math.round(6 * s * density);
    for (let i = 0; i < mist; i++) {
      resetSP();
      SP.x = x + randSym() * 0.6;
      SP.y = y + 0.15 + rand() * 0.4;
      SP.z = z + randSym() * 0.6;
      SP.vx = randSym() * 0.5; SP.vy = 0.35 + rand() * 0.5; SP.vz = randSym() * 0.5;
      SP.r = COL_FOAM[0] * 0.9; SP.g = COL_FOAM[1] * 0.95; SP.b = COL_FOAM[2];
      SP.tile = rand() < 0.5 ? T_DUST_A : T_DUST_B;
      SP.size = 0.5 + rand() * 0.7;
      SP.life = 1.4 + rand() * 1.6;
      SP.ground = ground;
      SP.alpha = 0.22 + rand() * 0.2;
      SP.grav = 0.05;
      SP.drag = 1.1;
      SP.rot = rand() * 6.283;
      SP.rotv = randSym() * 0.6;
      dust.spawn();
    }
  }

  /** Impact / landing burst: a low ring of material plus a rising plume. */
  function emitImpact(x, y, z, nx, ny, nz, severity, surf) {
    const sev = clamp(num(severity, 0.5), 0, 2);
    if (sev < 0.03) return;
    const ground = groundAt(x, z);
    const wet = wetnessAt(x, z, surf);
    const yieldK = SURFACE_YIELD[surfaceIndex(surf)];

    // Ground normal, defaulting to up.
    _nrm.set(num(nx, 0), num(ny, 1), num(nz, 0));
    if (_nrm.lengthSq() < 1e-6) _nrm.set(0, 1, 0);
    _nrm.normalize();
    // Any horizontal vector orthogonal to the normal, for the radial fan.
    _tmp.set(_nrm.z, 0, -_nrm.x);
    if (_tmp.lengthSq() < 1e-6) _tmp.set(1, 0, 0);
    _tmp.normalize();
    _tmp2.crossVectors(_nrm, _tmp).normalize();

    const chunks = Math.round((14 + sev * 46) * yieldK * density);
    for (let i = 0; i < chunks; i++) {
      resetSP();
      const a = rand() * 6.283;
      const ca = Math.cos(a), sa = Math.sin(a);
      const outSpeed = (1.6 + sev * 4.5) * (0.4 + rand() * 1.0);
      const upSpeed = (1.0 + sev * 2.6) * (0.25 + rand() * 0.9);
      SP.x = x + (_tmp.x * ca + _tmp2.x * sa) * 0.12;
      SP.y = y + 0.05;
      SP.z = z + (_tmp.z * ca + _tmp2.z * sa) * 0.12;
      SP.vx = (_tmp.x * ca + _tmp2.x * sa) * outSpeed + _nrm.x * upSpeed;
      SP.vy = (_tmp.y * ca + _tmp2.y * sa) * outSpeed + _nrm.y * upSpeed;
      SP.vz = (_tmp.z * ca + _tmp2.z * sa) * outSpeed + _nrm.z * upSpeed;
      tintFrom(wet > 0.5 ? COL_WET : COL_CHUNK, surf, 0.26);
      SP.tile = rand() < 0.5 ? T_GRIT : (rand() < 0.5 ? T_CLOD_A : T_CLOD_B);
      SP.size = 0.05 + rand() * 0.10;
      SP.life = 0.6 + rand() * 0.8;
      SP.ground = ground;
      SP.alpha = 1;
      SP.grav = -GRAVITY;
      SP.drag = 1.0 + rand() * 1.0;
      SP.rot = rand() * 6.283;
      SP.rotv = randSym() * 10;
      roost.spawn();
    }

    // Expanding ring of dust: the single strongest read that something landed.
    const rings = Math.max(1, Math.round(2 + sev * 3));
    for (let i = 0; i < rings; i++) {
      resetSP();
      SP.x = x; SP.y = y + 0.10 + i * 0.05; SP.z = z;
      SP.vx = randSym() * 0.3; SP.vy = 0.4 + rand() * 0.5; SP.vz = randSym() * 0.3;
      tintFrom(COL_DUST, surf, 0.1);
      SP.tile = T_RING;
      SP.size = 0.9 + sev * 1.4 + rand() * 0.4;
      SP.life = 0.55 + rand() * 0.5;
      SP.ground = ground - 0.35;   // let the ring hug the ground
      SP.alpha = (0.30 + sev * 0.35) * (1 - wet * 0.7);
      SP.grav = 0.1;
      SP.drag = 0.9;
      SP.rot = rand() * 6.283;
      SP.rotv = randSym() * 0.4;
      dust.spawn();
    }

    const puffs = Math.round((6 + sev * 16) * density * (1 - wet * 0.75));
    for (let i = 0; i < puffs; i++) {
      resetSP();
      const a = rand() * 6.283;
      SP.x = x + Math.cos(a) * (0.2 + rand() * 0.7);
      SP.y = y + 0.10 + rand() * 0.30;
      SP.z = z + Math.sin(a) * (0.2 + rand() * 0.7);
      SP.vx = Math.cos(a) * (0.9 + sev * 1.8) * rand();
      SP.vy = 0.5 + rand() * 1.2;
      SP.vz = Math.sin(a) * (0.9 + sev * 1.8) * rand();
      tintFrom(COL_DUST, surf, 0.12);
      SP.tile = rand() < 0.4 ? T_DUST_A : (rand() < 0.6 ? T_DUST_B : T_SMOKE);
      SP.size = 0.5 + rand() * 0.9;
      SP.life = 1.8 + rand() * 2.6;
      SP.ground = ground;
      SP.alpha = 0.28 + rand() * 0.3;
      SP.grav = -0.1 + rand() * 0.3;
      SP.drag = 1.0;
      SP.rot = rand() * 6.283;
      SP.rotv = randSym() * 0.8;
      dust.spawn();
    }

    const si = surfaceIndex(surf);
    if (si === Surface.ROCK || si === Surface.GRAVEL) {
      const n = Math.round((5 + sev * 20) * density);
      for (let i = 0; i < n; i++) spawnChip(x, y, z, 6 + sev * 10, 0.6, surf, ground);
    }
    if (si === Surface.MUD || wet > 0.6) {
      const n = Math.round((6 + sev * 22) * density);
      for (let i = 0; i < n; i++) spawnMud(x, y, z, 5 + sev * 8, 0.5, randSym() * 0.5, surf, ground);
    }
    if (si === Surface.GRASS || si === Surface.ROOT || si === Surface.LOAM) {
      const n = Math.round((3 + sev * 10) * density);
      for (let i = 0; i < n; i++) spawnFoliage(x, y, z, 4 + sev * 6, ground);
    }
  }

  // ---- events --------------------------------------------------------------

  const unsubs = [];
  function on(name, fn) {
    if (ctx.events && ctx.events.on) {
      const off = ctx.events.on(name, fn);
      if (typeof off === 'function') unsubs.push(off);
      else unsubs.push(() => ctx.events.off(name, fn));
    }
  }

  on('bike:impact', (e) => {
    const st = ctx.bike && ctx.bike.state;
    const p = (e && e.position) || (st && st.position);
    if (!p) return;
    bikeBasis(st);
    const n = e && e.normal;
    const px2 = num(p.x, 0), pz2 = num(p.z, 0);
    const gh = groundAt(px2, pz2);
    const surf = (e && typeof e.surface === 'number')
      ? e.surface
      : materialAt(px2, pz2, st ? num(st.surface, Surface.DIRT) : Surface.DIRT);
    lastImpactAt = elapsed;
    emitImpact(px2, gh === -1e9 ? num(p.y, 0) : gh, pz2,
      n ? num(n.x, 0) : 0, n ? num(n.y, 1) : 1, n ? num(n.z, 0) : 0,
      num(e && e.severity, 0.5), surf);
  });

  on('bike:skid', (e) => {
    const i = clamp(num(e && e.intensity, 0.6), 0, 2);
    skidBoost = Math.max(skidBoost, i);
    if (e && typeof e.surface === 'number') skidSurface = e.surface;
  });

  on('water:splash', (e) => {
    const p = e && e.position;
    if (!p) return;
    const v = e && e.velocity;
    const sp = v ? Math.sqrt(num(v.x, 0) ** 2 + num(v.y, 0) ** 2 + num(v.z, 0) ** 2) : 6;
    emitSplash(num(p.x, 0), num(p.y, 0), num(p.z, 0),
      v ? num(v.x, 0) : 0, v ? num(v.y, 0) : 0, v ? num(v.z, 0) : 0,
      clamp(0.35 + sp * 0.10, 0.3, 2.2));
  });

  on('run:crash', () => {
    const st = ctx.bike && ctx.bike.state;
    if (!st || !st.position) return;
    bikeBasis(st);
    const p = st.position;
    const surf = materialAt(p.x, p.z, num(st.surface, Surface.DIRT));
    emitImpact(p.x, groundAt(p.x, p.z), p.z, 0, 1, 0, 1.5, surf);
    skidBoost = 1.5;
  });

  on('trick:landed', (e) => {
    const st = ctx.bike && ctx.bike.state;
    if (!st || !st.position) return;
    bikeBasis(st);
    const p = st.position;
    const surf = materialAt(p.x, p.z, num(st.surface, Surface.DIRT));
    emitImpact(p.x, groundAt(p.x, p.z), p.z, 0, 1, 0,
      clamp(num(e && e.severity, 0.5), 0.2, 1.2), surf);
  });

  on('run:start', () => clearAll());

  function clearAll() {
    for (let i = 0; i < systemList.length; i++) systemList[i].clear();
    tracks.clear();
    hadBikePos = false;
    skidBoost = 0;
    accRoost = accDust = accChips = accMud = accFoliage = accGrit = 0;
  }

  // ---- per-frame emission --------------------------------------------------

  function emit(dt) {
    const bike = ctx.bike;
    const st = bike && bike.state;
    if (!st || !st.position) return;

    const pos = st.position;
    if (!isFinite(pos.x) || !isFinite(pos.y) || !isFinite(pos.z)) return;

    // A respawn/teleport must not draw a track across the mountain.
    if (hadBikePos) {
      const dx = pos.x - lastBikeX, dy = pos.y - lastBikeY, dz = pos.z - lastBikeZ;
      if (dx * dx + dy * dy + dz * dz > 15 * 15) {
        tracks.breakRibbon(0);
        tracks.breakRibbon(1);
      }
    }
    lastBikeX = pos.x; lastBikeY = pos.y; lastBikeZ = pos.z;
    hadBikePos = true;

    bikeBasis(st);

    const speed = clamp(num(st.speed, 0), 0, 80);
    const airborne = !!st.airborne;
    const crashed = !!st.crashed;
    const wheels = st.wheels;
    const brakeRear = clamp01(num(st.brakeRear, 0));
    const brakeFront = clamp01(num(st.brakeFront, 0));

    skidBoost = Math.max(0, skidBoost - dt * 1.6);

    if (airborne) {
      wasAirborne = true;
      tracks.breakRibbon(0);
      tracks.breakRibbon(1);
    } else if (wasAirborne) {
      // Landing. bike.js is expected to emit `bike:impact`, but a landing with
      // no burst is a very visible hole, so self-trigger when nothing arrived.
      wasAirborne = false;
      if (elapsed - lastImpactAt > 0.25) {
        const air = clamp(num(st.lastAirTime, num(st.airTime, 0)), 0, 4);
        const vDown = Math.max(0, -num(st.velocity && st.velocity.y, 0));
        const sev = clamp(0.18 + air * 0.35 + vDown * 0.045, 0, 1.4);
        if (sev > 0.22) {
          const gh = groundAt(pos.x, pos.z);
          const surfL = materialAt(pos.x, pos.z, num(st.surface, Surface.DIRT));
          emitImpact(pos.x, gh === -1e9 ? pos.y : gh, pos.z, 0, 1, 0, sev, surfL);
          lastImpactAt = elapsed;
        }
      }
    }

    // ---- tyre tracks (rear first so it draws under the front) --------------
    if (!airborne && wheels) {
      for (let wi = 1; wi >= 0; wi--) {
        const w = wheels[wi];
        const ribbon = wi === 1 ? 0 : 1;
        if (!w || !w.position || w.contact === false || speed < 0.35 || crashed) {
          if (!w || !w.position || w.contact === false || crashed) tracks.breakRibbon(ribbon);
          continue;
        }
        const wx = w.position.x, wz = w.position.z;
        if (!isFinite(wx) || !isFinite(wz)) { tracks.breakRibbon(ribbon); continue; }
        const gh = groundAt(wx, wz);
        if (gh === -1e9) { tracks.breakRibbon(ribbon); continue; }

        const cy = gh + TRACK_LIFT;
        let step = TRACK_SPACING;
        if (tracks.hasPrev(ribbon)) {
          tracks.prevCentre(ribbon, _cpPrev);
          const ddx = wx - _cpPrev.x, ddz = wz - _cpPrev.z;
          const d = Math.sqrt(ddx * ddx + ddz * ddz);
          if (d > TRACK_MAX_STEP) { tracks.breakRibbon(ribbon); continue; }
          if (d < step) continue;
        }

        // Travel direction: prefer the actual displacement, fall back to the
        // chassis forward vector at very low speed.
        let dx = _fwd.x, dz = _fwd.z;
        if (tracks.hasPrev(ribbon)) {
          tracks.prevCentre(ribbon, _cpPrev);
          dx = wx - _cpPrev.x; dz = wz - _cpPrev.z;
          const dl = Math.sqrt(dx * dx + dz * dz);
          if (dl > 1e-4) { dx /= dl; dz /= dl; } else { dx = _fwd.x; dz = _fwd.z; }
        }

        const t = terrainOf();
        if (t && t.sampleNormal) t.sampleNormal(wx, wz, _nrm);
        else _nrm.set(0, 1, 0);
        if (!isFinite(_nrm.x) || _nrm.lengthSq() < 1e-6) _nrm.set(0, 1, 0);

        const surf = surfaceIndex(w.material != null
          ? w.material : materialAt(wx, wz, num(st.surface, Surface.DIRT)));
        const wet = wetnessAt(wx, wz, surf);
        const slip = clamp01(Math.abs(num(w.slipRatio, 0)));
        const lat = clamp01(Math.abs(num(w.slipAngle, 0)) / 0.5);
        const brake = wi === 1 ? brakeRear : brakeFront;
        const load = clamp(num(w.load, 500) / 550, 0.25, 2.0);

        let strength = SURFACE_TRACK[surf] * (0.68 + slip * 0.55 + lat * 0.45 + brake * 0.3);
        strength *= (wi === 1 ? 1.0 : 0.72) * clamp(load, 0.4, 1.6);
        strength = clamp01(strength * (0.85 + wet * 0.4));
        if (strength < 0.02) { tracks.breakRibbon(ribbon); continue; }

        const halfWidth = (0.115 + slip * 0.075 + lat * 0.075) / 0.6;  // /0.6: core vs shoulder

        // Bake the lighting so a track sits at the same exposure as the ground
        // it darkens, and does not glow in shadow.
        const base = COL_TRACK[surf];
        const ndl = Math.max(0, _nrm.x * _sunDir.x + _nrm.y * _sunDir.y + _nrm.z * _sunDir.z);
        const wetDark = 1 - wet * 0.35;
        const lr = base[0] * wetDark * (ambColor.r + sunColor.r * ndl);
        const lg = base[1] * wetDark * (ambColor.g + sunColor.g * ndl);
        const lb = base[2] * wetDark * (ambColor.b + sunColor.b * ndl);

        tracks.push(ribbon, wx, cy, wz, dx, dz, _nrm.x, _nrm.y, _nrm.z,
          halfWidth, strength, elapsed, lr, lg, lb, t);
      }
    }
    tracks.flush();

    if (crashed || airborne || speed < 0.4) return;

    // ---- contact patch for the rear wheel ----------------------------------
    const rear = wheels && wheels[1];
    if (!rear || !rear.position || rear.contact === false) return;

    const rx = rear.position.x, rz = rear.position.z;
    if (!isFinite(rx) || !isFinite(rz)) return;
    const rgh = groundAt(rx, rz);
    if (rgh === -1e9) return;
    _cp.set(rx, rgh + 0.02, rz);

    const surf = surfaceIndex(rear.material != null
      ? rear.material : materialAt(rx, rz, num(st.surface, Surface.DIRT)));
    const wet = wetnessAt(rx, rz, surf);
    const yieldK = SURFACE_YIELD[surf];

    const slip = clamp(Math.abs(num(rear.slipRatio, 0)), 0, 2);
    const lateralSlip = clamp(num(rear.slipAngle, 0) / 0.6, -1.5, 1.5);
    const latMag = Math.abs(lateralSlip);
    const load = clamp(num(rear.load, 500) / 550, 0.3, 2.2);
    const braking = brakeRear > 0.45 && num(rear.slipRatio, 0) < 0 ? 1 : 0;
    const boost = 1 + skidBoost * 1.4;
    const speedRamp = clamp01(speed / 5);

    // ---- roost --------------------------------------------------------------
    {
      const rate = speedRamp * (16 + slip * 120 + latMag * 95 + brakeRear * 46)
        * load * yieldK * boost * density;
      accRoost += Math.min(rate, 460) * dt;
      let n = accRoost | 0;
      accRoost -= n;
      if (n > 46) n = 46;
      for (let i = 0; i < n; i++) {
        spawnRoost(_cp.x, _cp.y, _cp.z, speed, slip, lateralSlip, braking, surf, wet, rgh);
      }
    }

    // ---- dust plume ---------------------------------------------------------
    {
      const dryness = clamp01(1 - wet * 1.6);
      const dusty = (surf === Surface.DIRT || surf === Surface.GRAVEL
        || surf === Surface.LOAM || surf === Surface.ROCK || surf === Surface.GRASS) ? 1 : 0.25;
      const rate = clamp01((speed - 3) / 9) * (30 + slip * 44 + latMag * 46)
        * dryness * dusty * boost * density;
      accDust += rate * dt;
      let n = accDust | 0;
      accDust -= n;
      if (n > 14) n = 14;
      const scale = 0.75 + clamp01(speed / 20) * 0.7 + skidBoost * 0.4;
      for (let i = 0; i < n; i++) {
        spawnDust(_cp.x - _fwd.x * 0.55, _cp.y, _cp.z - _fwd.z * 0.55, speed, surf, rgh, scale);
      }
    }

    // ---- rock chips / gravel spray -----------------------------------------
    if (surf === Surface.ROCK || surf === Surface.GRAVEL) {
      const rate = speedRamp * (7 + slip * 42 + brakeRear * 26) * boost * density;
      accChips += rate * dt;
      let n = accChips | 0;
      accChips -= n;
      if (n > 14) n = 14;
      for (let i = 0; i < n; i++) spawnChip(_cp.x, _cp.y, _cp.z, speed, slip, surf, rgh);
    }

    // ---- mud spray ----------------------------------------------------------
    if (surf === Surface.MUD || wet > 0.45) {
      const rate = speedRamp * (12 + slip * 60 + latMag * 45) * clamp01(wet * 1.4)
        * boost * density;
      accMud += rate * dt;
      let n = accMud | 0;
      accMud -= n;
      if (n > 18) n = 18;
      for (let i = 0; i < n; i++) {
        spawnMud(_cp.x, _cp.y, _cp.z, speed, slip, lateralSlip, surf, rgh);
      }
    }

    // ---- forest floor debris -------------------------------------------------
    if (surf === Surface.GRASS || surf === Surface.ROOT || surf === Surface.LOAM) {
      const t = terrainOf();
      let forest = 1;
      if (t && t.treelineAt) {
        const tl = t.treelineAt(rx, rz);
        if (isFinite(tl)) forest = rgh < tl ? 1 : 0.15;
      }
      const rate = speedRamp * (3.5 + slip * 12) * forest * boost * density;
      accFoliage += rate * dt;
      let n = accFoliage | 0;
      accFoliage -= n;
      if (n > 6) n = 6;
      for (let i = 0; i < n; i++) spawnFoliage(_cp.x, _cp.y, _cp.z, speed, rgh);
    }

    // ---- high-speed grit near the lens --------------------------------------
    if (speed > 11 && ctx.camera) {
      const cam = ctx.camera;
      const f = clamp01((speed - 11) / 12);
      const rate = f * 70 * density;
      accGrit += rate * dt;
      let n = accGrit | 0;
      accGrit -= n;
      if (n > 8) n = 8;
      _camFwd.set(0, 0, -1).applyQuaternion(cam.quaternion);
      for (let i = 0; i < n; i++) {
        resetSP();
        // Seed it in a slab just in front of the lens so it sweeps past.
        const ahead = 2.0 + rand() * 7.0;
        SP.x = cam.position.x + _camFwd.x * ahead + randSym() * 3.2;
        SP.y = cam.position.y + _camFwd.y * ahead + randSym() * 2.0;
        SP.z = cam.position.z + _camFwd.z * ahead + randSym() * 3.2;
        const gh = groundAt(SP.x, SP.z);
        // Keep it out of the dirt; grit is airborne debris, not ground spray.
        if (gh !== -1e9 && SP.y < gh + 0.25) SP.y = gh + 0.25 + rand() * 1.2;
        const back = speed * (0.85 + rand() * 0.5);
        SP.vx = -_fwd.x * back + randSym() * 1.5;
        SP.vy = randSym() * 1.2 + 0.3;
        SP.vz = -_fwd.z * back + randSym() * 1.5;
        tintFrom(COL_DUST, surf, 0.2);
        SP.tile = rand() < 0.65 ? T_STREAK : T_GRIT;
        SP.size = 0.020 + rand() * 0.045;
        SP.life = 0.25 + rand() * 0.30;
        SP.ground = gh;
        SP.alpha = 0.35 + rand() * 0.4;
        SP.grav = -1.2;
        SP.drag = 0.35;
        SP.rot = rand() * 6.283;
        SP.rotv = randSym() * 3;
        grit.spawn();
      }
    }
  }

  // ---- lifecycle -----------------------------------------------------------

  let drawn = 0;

  function update(dt, c) {
    const d = clamp(num(dt, 0), 0, 1 / 20);
    if (d <= 0) return;
    elapsed += d;

    const settings = (c && c.settings) || ctx.settings;
    density = clamp(num(settings && settings.particleDensity, 1), 0, 2);

    refreshLighting();
    tracks.uniforms.uTime.value = elapsed;

    try { emit(d); } catch (err) { /* never let an effect kill the frame */ }

    drawn = 0;
    for (let i = 0; i < systemList.length; i++) {
      drawn += systemList[i].update(d, elapsed, groundAt);
    }
  }

  function burst(kind, position, normal, strength, surface) {
    if (!position) return;
    const st = ctx.bike && ctx.bike.state;
    bikeBasis(st);
    const x = num(position.x, 0), z = num(position.z, 0);
    const gh = groundAt(x, z);
    const y = gh === -1e9 ? num(position.y, 0) : gh;
    const surf = typeof surface === 'number' ? surface : materialAt(x, z, Surface.DIRT);
    const s = num(strength, 1);
    switch (kind) {
      case 'splash':
        emitSplash(x, num(position.y, y), z, 0, 0, 0, s);
        break;
      case 'dust': {
        const n = Math.round(8 * s * density);
        for (let i = 0; i < n; i++) spawnDust(x, y, z, 4, surf, gh, 1.0 + s * 0.3);
        break;
      }
      case 'leaves': {
        const n = Math.round(8 * s * density);
        for (let i = 0; i < n; i++) spawnFoliage(x, y, z, 4, gh);
        break;
      }
      case 'chips': {
        const n = Math.round(10 * s * density);
        for (let i = 0; i < n; i++) spawnChip(x, y, z, 6, 0.6, surf, gh);
        break;
      }
      default:
        emitImpact(x, y, z,
          normal ? num(normal.x, 0) : 0,
          normal ? num(normal.y, 1) : 1,
          normal ? num(normal.z, 0) : 0,
          s, surf);
        break;
    }
  }

  function dispose() {
    for (let i = 0; i < unsubs.length; i++) {
      try { unsubs[i](); } catch (e) { /* ignore */ }
    }
    unsubs.length = 0;
    for (let i = 0; i < systemList.length; i++) systemList[i].dispose();
    tracks.dispose();
    if (atlas) atlas.dispose();
    if (tread) tread.dispose();
    quad.dispose();
    if (ctx.scene) ctx.scene.remove(group);
  }

  return {
    group,
    systems,
    tracks,
    burst,
    emitSplash(position, velocity, strength) {
      if (!position) return;
      emitSplash(num(position.x, 0), num(position.y, 0), num(position.z, 0),
        velocity ? num(velocity.x, 0) : 0,
        velocity ? num(velocity.y, 0) : 0,
        velocity ? num(velocity.z, 0) : 0,
        num(strength, 1));
    },
    clear: clearAll,
    clearTracks() { tracks.clear(); },
    update,
    dispose,
    /** Live particle count, for the debug HUD. */
    get liveCount() { return drawn; },
    get drawCalls() { return systemList.length + 1; },
  };
}
