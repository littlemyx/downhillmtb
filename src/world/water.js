// DESCENT — water.js
// The creek that drains the mountain, the valley water body it runs into, the
// waterfalls where it meets the cliff bands, wet rock, spray and splashes.
// See CONTRACT.md §6 and ADDENDUM §B.
//
// ---------------------------------------------------------------------------
// CONTRACT-NOTE (water → terrain): the brief says "darken and gloss the terrain
//   near the water by calling terrain's wetness setter if one exists". There is
//   no setter — terrain exposes `sampleWetness(x, z)` (read-only) and bakes
//   wetness into a per-vertex `aTerrainExtra.y` at buildBase() time, which is
//   before the creek route is known and before terrain.commit() builds the chunk
//   meshes. Rewriting that attribute post-commit would mean reaching into
//   another module's geometry, which the hard rules forbid. Instead this module
//   lays its own wet-rock decal ribbon over the terrain along the creek and
//   around every waterfall base: premultiplied-alpha darkening plus a real
//   specular sheen, so the rock reads as soaked rather than merely tinted. If
//   terrain later grows `terrain.setWetness(x, z, r, amount)` this module will
//   call it (see `applyTerrainWetness()` below — it is already wired and simply
//   no-ops when the setter is absent).
//
// CONTRACT-NOTE (water → everyone): additive query API beyond CONTRACT §6, all
//   allocation-free and safe to call every frame:
//     water.heightAt(x, z)          -> world Y of the water surface, or null
//     water.depthAt(x, z)           -> metres of water, 0 when dry
//     water.flowAt(x, z, outVec3)   -> flow velocity (m/s) written into outVec3
//     water.isSubmerged(x, y, z)    -> boolean
//     water.waterLevel              -> valley body surface Y
//     water.creekPath               -> Float32Array [x,y,z, x,y,z, …] centreline
//   `bike.js` may use `depthAt`/`flowAt` for drag through the ford if it wants;
//   nothing depends on it.
//
// CONTRACT-NOTE (water → physics/particles): this module *emits* `water:splash`
//   when the bike enters water fast, and also *listens* for it, so any other
//   module emitting the event gets rings and droplets for free. Payload is
//   `{ position: Vector3, velocity: Vector3|number }` — velocity may be a scalar
//   speed; both forms are handled.
//
// CONTRACT-NOTE (water → reflections, r6 review): the work order asks for
//   "half-resolution screen-space reflections for the valley water". What is
//   built here is a *heightfield* reflection march rather than SSR, and the
//   substitution is deliberate:
//     • SSR needs the scene colour and depth for the frame it is shading.
//       postfx owns the composer and there is no shared scene-colour target, so
//       SSR here would mean either an extra scene pass or a one-frame-late
//       reprojection of a buffer this module does not own. The former costs more
//       than the whole water budget; the latter is a cross-module coupling the
//       hard rules forbid.
//     • The valley body is a *plane* at a known Y. For a plane, marching the
//       reflected ray against a baked top-down height map is exact where SSR is
//       an approximation, and it reflects geometry that is behind the camera or
//       off the side of the frame — which is most of the far shore in
//       r6_00-establishing, and which SSR structurally cannot do at all.
//     • It costs one 256² texture and ~10 fetches on water pixels whose Fresnel
//       term is actually visible; nothing else in the frame pays anything.
//   The creek keeps the environment map exactly as before, as the order asks.
//
// CONTRACT-NOTE (water → depth buffer): CONTRACT §6 asks for "refraction offset
//   sampled from the depth texture". postfx owns the composer and there is no
//   shared scene-depth texture to read, and adding a depth pre-pass for water
//   alone would cost more than the whole effect is worth. Water depth is instead
//   *baked analytically* from the heightfield: per-vertex for the creek ribbon
//   and into a DataTexture for the valley body. That is strictly more accurate
//   than a screen-space depth fetch (no edge halos, no disocclusion) and free at
//   runtime; the refraction offset perturbs the bake lookup exactly as it would
//   perturb a depth-texture lookup.
//
// Everything here is procedural: normal maps, foam noise, streambed pebbles and
// the spray sprite are generated in code from the shared seeded PRNG.
// ---------------------------------------------------------------------------

import * as THREE from 'three';
import { makeRng, subSeed, clamp, clamp01, lerp } from '../core/rng.js';

// ===========================================================================
// Tunables
// ===========================================================================

const STATION_DS = 3.0;          // metres between creek centreline stations
const CROSS = 13;                // vertices across the creek ribbon
const RIBBON_OVERHANG = 1.42;    // geometry is wider than the water so the edge
                                 // is always buried in the bank — the shoreline
                                 // is then produced by the depth fade, never by
                                 // a polygon edge.
const MAX_STATIONS = 1400;
const TRACE_STEP = 6.0;          // metres per steepest-descent probe
// A heightfield at 0.35 m/sample cannot hold a true overhang, so "waterfall"
// here means the creek pouring down a genuinely near-vertical slab. Set the
// gate low and every steep pitch grows a sheet that just lies on the hillside.
const FALL_MIN_GRAD = 0.62;      // dY/ds that counts as a waterfall (~32°)
const FALL_MIN_DROP = 1.9;       // metres — below this it is just a riffle
const RIPPLE_COUNT = 8;
const NORMAL_TEX = 256;
const FOAM_TEX = 192;
const BED_TEX = 256;
const PUFF_TEX = 64;
const MAX_PUDDLES = 54;

// Beer–Lambert extinction per metre of water, per channel. Red goes first,
// which is the whole reason deep water reads green-blue and a 15 cm riffle
// reads as clear glass over gravel.
const ABSORB = new THREE.Vector3(0.95, 0.30, 0.17);

// --- solar-disc glint (r6 review) ------------------------------------------
// The ceiling on a specular reflection is the radiance of the thing being
// reflected. sky.js clamps its own solar disc so it cannot write Inf into the
// HDR buffer, and postfx.js measured what arrives: ~2530 units of post-exposure
// linear radiance for the disc itself, with its specular-escape curve putting
// paper white at 595 (postfx.js, HL_SPEC_GAIN block). Those two numbers are the
// whole calibration for the glint below:
//   • a mirror reflection cannot exceed 2530, so a glint is hard-clamped to
//     F * 2530 — brighter than the sun is not a look, it is a bug;
//   • at normal incidence water's F is ~0.02, giving 50 units → display L243.
//     Bright, and correctly NOT white: postfx says in terms a 4% dielectric
//     reflection of the sun must not reach 255, and this is that reflection;
//   • only a genuinely grazing mirror angle (F > ~0.24) passes 595 and clips.
// So the glint reaches true white exactly where a real one would, on the small
// set of pixels where a wavelet happens to face the mirror direction, and
// nowhere else.
const SUN_DISC_RADIANCE = 2530.0;
// GGX alpha floor. The sun is 0.0093 rad across; reflecting it in a mirror
// halves that in half-vector space, so ~0.0047 is the width of a *reflected*
// solar disc. 0.0055 is a hair wider — everything real carries some blur, and a
// lobe narrower than a pixel is a lobe that flickers.
const GLINT_ALPHA = 0.0055;
// World metres of surface per screen pixel at which the wavelets are considered
// fully unresolved, i.e. at which the aggregate slope distribution becomes the
// specular lobe. This is the term that stops the distant open sea from becoming
// a field of crawling white dots, and it is also what confines clipping to near
// water — see the shader comment.
const GLINT_FOOTPRINT = 0.25;

// --- shore reflection proxy (r6 review) ------------------------------------
const SHORE_TEX = 256;             // top-down samples per axis over the world
const SHORE_MAX_DIST = 2600;       // metres of reflected ray ever marched
const SHORE_CANOPY_H = 16.0;       // metres of forest added to the proxy height
const SHORE_STEPS = { low: 0, medium: 6, high: 10, ultra: 14 };

// How far the local streambed gradient is allowed to bend the flow map away from
// the centreline tangent. 0.45 is enough that a midstream boulder visibly parts
// the surface motion and a bar visibly fans it, and low enough that the creek
// never advects sideways out of its own channel.
const FLOW_BED_BEND = 0.45;

// ===========================================================================
// Module-scope scratch. Nothing in update() is allowed to allocate.
// ===========================================================================

const _v1 = new THREE.Vector3();
const _v2 = new THREE.Vector3();
const _v3 = new THREE.Vector3();
const _col = new THREE.Color();
const _sunCol = new THREE.Color();
// Handed out in the `water:splash` payload, so it is never reused for anything
// else — a listener that keeps the reference for a frame still reads sane data.
const _splashPos = new THREE.Vector3();

// ===========================================================================
// 1. Tileable procedural noise
// ===========================================================================

function hash2i(x, y, seed) {
  let h = (x * 374761393 + y * 668265263 + seed * 2147483647) | 0;
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}

/** Value noise that wraps exactly on `period` cells, so the texture tiles. */
function vnoiseTile(x, y, period, seed) {
  const xi = Math.floor(x), yi = Math.floor(y);
  const xf = x - xi, yf = y - yi;
  const u = xf * xf * (3 - 2 * xf);
  const v = yf * yf * (3 - 2 * yf);
  const x0 = ((xi % period) + period) % period;
  const y0 = ((yi % period) + period) % period;
  const x1 = (x0 + 1) % period;
  const y1 = (y0 + 1) % period;
  const a = hash2i(x0, y0, seed), b = hash2i(x1, y0, seed);
  const c = hash2i(x0, y1, seed), d = hash2i(x1, y1, seed);
  const ab = a + (b - a) * u;
  const cd = c + (d - c) * u;
  return ab + (cd - ab) * v;
}

/** Tiling fbm. `period` is in cells at the base octave. */
function fbmTile(x, y, period, oct, gain, seed) {
  let sum = 0, amp = 1, norm = 0, p = period, f = 1;
  for (let o = 0; o < oct; o++) {
    sum += vnoiseTile(x * f, y * f, p, seed + o * 131) * amp;
    norm += amp;
    amp *= gain;
    f *= 2;
    p *= 2;
  }
  return sum / norm;
}

/** Sharp-crested variant — |2n-1| folded, which reads as wind chop. */
function billowTile(x, y, period, oct, gain, seed) {
  let sum = 0, amp = 1, norm = 0, p = period, f = 1;
  for (let o = 0; o < oct; o++) {
    const n = vnoiseTile(x * f, y * f, p, seed + o * 977) * 2 - 1;
    sum += (1 - Math.abs(n)) * amp;
    norm += amp;
    amp *= gain;
    f *= 2;
    p *= 2;
  }
  return sum / norm;
}

/**
 * Height field -> tangent-space normal map, stored RG = xy, B = height.
 * Central differences with wraparound so the normals tile as cleanly as the
 * height does. `strength` is the vertical exaggeration in texel units.
 */
function heightToNormalTexture(size, height, strength) {
  const data = new Uint8Array(size * size * 4);
  for (let j = 0; j < size; j++) {
    const jm = (j - 1 + size) % size, jp = (j + 1) % size;
    for (let i = 0; i < size; i++) {
      const im = (i - 1 + size) % size, ip = (i + 1) % size;
      const hL = height[j * size + im], hR = height[j * size + ip];
      const hD = height[jm * size + i], hU = height[jp * size + i];
      let nx = (hL - hR) * strength;
      let ny = (hD - hU) * strength;
      const inv = 1 / Math.sqrt(nx * nx + ny * ny + 1);
      nx *= inv; ny *= inv;
      const k = (j * size + i) * 4;
      data[k] = ((nx * 0.5 + 0.5) * 255) | 0;
      data[k + 1] = ((ny * 0.5 + 0.5) * 255) | 0;
      data[k + 2] = (clamp01(height[j * size + i]) * 255) | 0;
      data[k + 3] = 255;
    }
  }
  const tex = new THREE.DataTexture(data, size, size, THREE.RGBAFormat, THREE.UnsignedByteType);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.generateMipmaps = true;
  tex.colorSpace = THREE.NoColorSpace;
  tex.needsUpdate = true;
  return tex;
}

/** Long, low, rolling swell — the layer that carries the big reflections. */
function makeSwellTexture(seed) {
  const S = NORMAL_TEX;
  const h = new Float32Array(S * S);
  for (let j = 0; j < S; j++) {
    for (let i = 0; i < S; i++) {
      const x = i / S * 8, y = j / S * 8;
      // Two anisotropic lobes crossed at a shallow angle read as a wind-driven
      // swell rather than as isotropic lumps.
      const a = fbmTile(x * 0.55, y * 1.25, 8, 4, 0.55, seed);
      const b = fbmTile(x * 1.15 + 3.1, y * 0.62 + 7.7, 8, 4, 0.55, seed + 61);
      h[j * S + i] = a * 0.62 + b * 0.38;
    }
  }
  return heightToNormalTexture(S, h, 2.1);
}

/**
 * Short chop — the layer that makes the sun glitter.
 * Deliberately NOT a ridged/billow field: folding the noise gives sharp
 * creases that read as cracked ice once a specular lobe runs over them. Water
 * chop is rounded, so this is plain fbm with a smoothstep applied to round the
 * crests and flatten the troughs, which is the actual shape of wind waves.
 */
function makeChopTexture(seed) {
  const S = NORMAL_TEX;
  const h = new Float32Array(S * S);
  for (let j = 0; j < S; j++) {
    for (let i = 0; i < S; i++) {
      const x = i / S * 16, y = j / S * 16;
      // Mildly anisotropic: wind waves have crests, not blobs.
      const a = fbmTile(x * 0.8, y * 1.5, 16, 4, 0.55, seed);
      const f = fbmTile(x * 2.1 + 11.3, y * 2.6 + 4.9, 32, 3, 0.5, seed + 313);
      let v = a * 0.74 + f * 0.26;
      v = v * v * (3 - 2 * v);
      h[j * S + i] = v;
    }
  }
  return heightToNormalTexture(S, h, 2.2);
}

/**
 * Foam / turbulence mask. R = soft cloudy fbm (the body of the foam),
 * G = sharper billow (the lacy edges), B = fine speckle (bubble grain).
 */
function makeFoamTexture(seed) {
  const S = FOAM_TEX;
  const data = new Uint8Array(S * S * 4);
  for (let j = 0; j < S; j++) {
    for (let i = 0; i < S; i++) {
      const x = i / S * 6, y = j / S * 6;
      const r = fbmTile(x, y, 6, 4, 0.55, seed);
      const g = billowTile(x * 2.2, y * 2.2, 12, 3, 0.5, seed + 77);
      const b = fbmTile(x * 6.5, y * 6.5, 39, 2, 0.5, seed + 991);
      const k = (j * S + i) * 4;
      data[k] = (clamp01(r) * 255) | 0;
      data[k + 1] = (clamp01(g * g) * 255) | 0;
      data[k + 2] = (clamp01(b) * 255) | 0;
      data[k + 3] = 255;
    }
  }
  const tex = new THREE.DataTexture(data, S, S, THREE.RGBAFormat, THREE.UnsignedByteType);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.generateMipmaps = true;
  tex.colorSpace = THREE.NoColorSpace;
  tex.needsUpdate = true;
  return tex;
}

/**
 * Streambed pebbles, seen *through* the water. Scattered discs with a fixed
 * key light so they read as rounded stones, plus grit. sRGB — it is albedo.
 */
function makeBedTexture(seed) {
  const S = BED_TEX;
  const rng = makeRng(seed);
  const hgt = new Float32Array(S * S);
  const tint = new Float32Array(S * S * 3);
  for (let i = 0; i < S * S; i++) { tint[i * 3] = 0.5; tint[i * 3 + 1] = 0.5; tint[i * 3 + 2] = 0.5; }

  const count = 220;
  for (let p = 0; p < count; p++) {
    const cx = rng() * S, cy = rng() * S;
    const r = 3 + rng() * rng() * 17;
    const el = 0.55 + rng() * 0.9;             // eccentricity
    const rot = rng() * Math.PI;
    const cs = Math.cos(rot), sn = Math.sin(rot);
    const v = 0.34 + rng() * 0.44;
    const warm = 0.86 + rng() * 0.30;
    const cool = 0.92 + rng() * 0.16;
    const R = Math.ceil(r * 1.3);
    for (let dy = -R; dy <= R; dy++) {
      for (let dx = -R; dx <= R; dx++) {
        const lx = (dx * cs + dy * sn) / r;
        const ly = (-dx * sn + dy * cs) / (r * el);
        const d2 = lx * lx + ly * ly;
        if (d2 >= 1) continue;
        const dome = Math.sqrt(1 - d2);
        const x = ((Math.round(cx + dx) % S) + S) % S;
        const y = ((Math.round(cy + dy) % S) + S) % S;
        const k = y * S + x;
        const hh = dome * r * 0.11;
        if (hh <= hgt[k]) continue;
        hgt[k] = hh;
        tint[k * 3] = v * warm;
        tint[k * 3 + 1] = v * (warm * 0.5 + cool * 0.5);
        tint[k * 3 + 2] = v * cool * 0.92;
      }
    }
  }

  const data = new Uint8Array(S * S * 4);
  // Fixed key from up-left so the stones have shape; the water's own normal
  // map then wobbles them, which is the tell that you are looking through
  // moving water and not at a wet photograph.
  for (let j = 0; j < S; j++) {
    const jm = (j - 1 + S) % S, jp = (j + 1) % S;
    for (let i = 0; i < S; i++) {
      const im = (i - 1 + S) % S, ip = (i + 1) % S;
      const gx = hgt[j * S + im] - hgt[j * S + ip];
      const gy = hgt[jm * S + i] - hgt[jp * S + i];
      const lightN = clamp01(0.5 + (gx * 0.55 + gy * 0.45) * 2.4);
      const grit = 0.86 + 0.28 * fbmTile(i / S * 26, j / S * 26, 26, 2, 0.5, seed + 4441);
      const k = (j * S + i) * 4;
      const sh = (0.42 + 0.78 * lightN) * grit;
      data[k] = clamp01(tint[(j * S + i) * 3] * sh) * 255;
      data[k + 1] = clamp01(tint[(j * S + i) * 3 + 1] * sh) * 255;
      data[k + 2] = clamp01(tint[(j * S + i) * 3 + 2] * sh) * 255;
      data[k + 3] = 255;
    }
  }
  const tex = new THREE.DataTexture(data, S, S, THREE.RGBAFormat, THREE.UnsignedByteType);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.generateMipmaps = true;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.needsUpdate = true;
  return tex;
}

/** Soft, slightly lumpy puff for mist and droplets. Alpha only matters. */
function makePuffTexture(seed) {
  const S = PUFF_TEX;
  const data = new Uint8Array(S * S * 4);
  for (let j = 0; j < S; j++) {
    for (let i = 0; i < S; i++) {
      const u = (i + 0.5) / S * 2 - 1;
      const v = (j + 0.5) / S * 2 - 1;
      const d = Math.sqrt(u * u + v * v);
      const lump = 0.72 + 0.5 * fbmTile(i / S * 5, j / S * 5, 5, 3, 0.55, seed);
      let a = clamp01(1 - d / (0.92 * lump));
      a = a * a * (3 - 2 * a);
      const k = (j * S + i) * 4;
      // A touch brighter at the top of the sprite so a lit puff has volume.
      const shade = 0.82 + 0.30 * clamp01(0.5 - v * 0.5);
      data[k] = 255 * clamp01(shade);
      data[k + 1] = 255 * clamp01(shade);
      data[k + 2] = 255;
      data[k + 3] = (a * 255) | 0;
    }
  }
  const tex = new THREE.DataTexture(data, S, S, THREE.RGBAFormat, THREE.UnsignedByteType);
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.generateMipmaps = true;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.needsUpdate = true;
  return tex;
}

// ===========================================================================
// 2. GLSL
// ===========================================================================

// --- shared helpers, pasted into every water shader ------------------------
const GLSL_COMMON = /* glsl */`
#define PI_W 3.141592653589793

// Solar-disc specular, shared by the water surface and the wet-rock decal.
//
// For a source small enough that the BRDF is constant across it,
//     L_out = D * Vis * F * NdL * L_sun * Omega,
// and L_sun * Omega is the irradiance on a surface facing it, which is exactly
// PI * sunColor as this file uploads it. So the glint needs no light constant of
// its own — it is the same sun the diffuse terms use, evaluated as a disc rather
// than as a delta.
//
// a2 is the GGX alpha squared and is the caller's business: the surface picks it
// so that the lobe is never narrower than the reflected solar disc and never
// narrower than the screen footprint the surface is being filtered at.
vec3 discSpecular( float NdH, float NdV, float NdL, float a2, float F, vec3 sunColor ) {
	float d = NdH * NdH * ( a2 - 1.0 ) + 1.0;
	float D = a2 / ( PI_W * d * d );
	float k = 0.5 * sqrt( a2 );
	float vis = ( NdV / ( NdV * ( 1.0 - k ) + k ) ) * ( NdL / ( NdL * ( 1.0 - k ) + k ) )
	          / ( 4.0 * NdV * NdL + 1e-4 );
	return sunColor * ( PI_W * D * vis * F * NdL );
}

// Two-phase flow mapping. Sampling a scrolling texture along a flow direction
// smears without bound; sampling it twice at half-cycle-offset phases and
// cross-fading with a triangle weight keeps the advection but resets the smear
// every cycle. This is the difference between a river and a moving wallpaper.
vec4 flowTex( sampler2D tex, vec2 uv, vec2 dUV, float phase ) {
	float p0 = fract( phase );
	float p1 = fract( phase + 0.5 );
	vec4 a = texture2D( tex, uv - dUV * p0 );
	vec4 b = texture2D( tex, uv - dUV * p1 + vec2( 0.5, 0.5 ) );
	return mix( a, b, abs( 1.0 - 2.0 * p0 ) );
}

vec3 waterSkyFallback( vec3 dir, vec3 low, vec3 high, vec3 sunDir, vec3 sunCol ) {
	float up = clamp( dir.y * 0.5 + 0.5, 0.0, 1.0 );
	vec3 c = mix( low, high, up * up );
	float s = max( dot( dir, sunDir ), 0.0 );
	c += sunCol * ( pow( s, 220.0 ) * 0.5 + pow( s, 8.0 ) * 0.06 );
	return c;
}
`;

const SURFACE_VERT = /* glsl */`
uniform float uTime;
uniform float uWaveHeight;
uniform vec2  uSwellDir;

attribute float aDepth;
attribute float aFlow;
attribute float aFoam;
attribute float aShore;
attribute float aDist;
attribute vec2  aFlowDir;
attribute vec3  aBed;

varying vec3  vWorld;
varying vec3  vNrm;
varying vec2  vFlowDir;
varying vec3  vBed;
varying float vDepth;
varying float vFlow;
varying float vFoam;
varying float vShore;
varying float vDist;

#include <fog_pars_vertex>

void main() {

	vec3 pos = position;

	// Vertical motion has to die out in the shallows or the shoreline visibly
	// lifts off the bed and the depth fade stops lining up with the geometry.
	float shallow = clamp( aDepth * 3.2, 0.0, 1.0 );

	float swell = sin( dot( pos.xz, uSwellDir ) * 0.21 + uTime * 0.55 ) * 0.62
	            + sin( dot( pos.xz, vec2( -uSwellDir.y, uSwellDir.x ) ) * 0.37 - uTime * 0.83 ) * 0.38;
	pos.y += swell * uWaveHeight * shallow;

	// Standing waves over a fast bed: they travel far slower than the water,
	// which is why a rapid looks stationary while the water rips through it.
	pos.y += sin( aDist * 2.7 - uTime * ( 1.6 + aFlow * 1.4 ) ) * 0.020 * aFlow * shallow;

	vWorld   = pos;
	vNrm     = normalize( normal );
	vFlowDir = aFlowDir;
	vBed     = aBed;
	vDepth   = aDepth;
	vFlow    = aFlow;
	vFoam    = aFoam;
	vShore   = aShore;
	vDist    = aDist;

	vec4 mvPosition = modelViewMatrix * vec4( pos, 1.0 );
	gl_Position = projectionMatrix * mvPosition;

	#include <fog_vertex>
}
`;

const SURFACE_FRAG = /* glsl */`
uniform float uTime;
uniform sampler2D uSwellTex;
uniform sampler2D uChopTex;
uniform sampler2D uFoamTex;
uniform sampler2D uBedTex;
uniform sampler2D uDepthTex;
uniform sampler2D envMap;

uniform vec4  uDepthRect;      // minX, minZ, 1/width, 1/depth
uniform vec2  uDepthDecode;    // scale, bias  ->  depth = r * scale - bias
uniform float uUseDepthTex;
uniform float uEnvIntensity;
uniform vec3  uSunDir;
uniform vec3  uSunColor;
uniform vec3  uSkyLow;
uniform vec3  uSkyHigh;
uniform vec3  uAbsorb;
uniform vec3  uScatter;
uniform vec3  uFoamColor;
uniform float uEdgeFade;
uniform float uFoamWidth;
uniform float uRefract;
uniform float uWaveGain;
uniform float uBedScale;
uniform float uOpacity;
uniform float uSpecGain;
uniform float uFresnelMax;
uniform float uShorePixels;    // screen-space width of the waterline feather
uniform vec2  uWetMargin;      // x = metres of bank counted as wetted, y = its alpha
uniform vec4  uRipples[ RIPPLE_COUNT ];   // xz = centre, z = radius, w = strength

#ifdef WATER_SHORE_REFLECT
uniform sampler2D uShoreTex;   // RGB = shore albedo x geometric shade, A = height/HMAX
uniform vec4  uShoreRect;      // minX, minZ, 1/width, 1/depth
uniform vec3  uShoreCfg;       // x = HMAX metres, y = tallest rise, z = blend strength
uniform vec3  uShoreLight;     // relights the baked albedo from the live sun/sky
#endif

// Offshore-apron edge dissolve. uDissolveRect is the apron's outer rectangle
// (minX, minZ, maxX, maxZ) and uDissolveBand is (band width in metres,
// 1 / band width). Both are (0,0,0,0)/(0,0) on the creek and valley materials,
// where the second component being zero makes the whole term exactly zero — so
// this cannot touch either of those surfaces.
uniform vec4  uDissolveRect;
uniform vec2  uDissolveBand;

varying vec3  vWorld;
varying vec3  vNrm;
varying vec2  vFlowDir;
varying vec3  vBed;
varying float vDepth;
varying float vFlow;
varying float vFoam;
varying float vShore;
varying float vDist;

#include <fog_pars_fragment>
#include <cube_uv_reflection_fragment>
${GLSL_COMMON}

float sampleBakedDepth( vec2 p ) {
	vec2 uv = ( p - uDepthRect.xy ) * uDepthRect.zw;
	return texture2D( uDepthTex, clamp( uv, 0.0005, 0.9995 ) ).r * uDepthDecode.x - uDepthDecode.y;
}

vec3 sampleEnv( vec3 dir, float rough ) {
	#ifdef ENVMAP_TYPE_CUBE_UV
		return textureCubeUV( envMap, dir, rough ).rgb * uEnvIntensity;
	#else
		return waterSkyFallback( dir, uSkyLow, uSkyHigh, uSunDir, uSunColor ) * uEnvIntensity;
	#endif
}

#ifdef WATER_SHORE_REFLECT
/**
 * March the reflected ray across a baked top-down height map of the shore.
 *
 * The water body is a plane at a known Y, so a ray leaving it at ( orig, dir )
 * is at height t * dir.y/|dir.xz| above the water after t horizontal metres.
 * Comparing that against the terrain height stored in the proxy's alpha channel
 * is a complete intersection test — no depth buffer, no screen-space
 * disocclusion, and it works for shore that is off the side of the frame or
 * behind the camera, which is where most of the far shore in the establishing
 * shot actually is.
 *
 * Returns 1.0 on a hit and writes the reflected colour into hitCol.
 */
float marchShore( vec3 orig, vec3 dir, out vec3 hitCol ) {

	hitCol = vec3( 0.0 );
	vec2 rd = dir.xz;
	float rl = length( rd );
	if ( rl < 1e-4 ) return 0.0;
	rd /= rl;
	float sy = dir.y / rl;                       // metres of rise per metre travelled
	if ( sy <= 1e-3 ) return 0.0;

	// The ray can only ever meet ground while it is still lower than the tallest
	// thing in the map, so that bounds the march exactly rather than by taste:
	// a steeply reflected ray gets a short march and a grazing one gets a long
	// one, which is the correct distribution of effort. (The cheap cull is the
	// Fresnel test at the call site, not this one — this only fires on a
	// near-vertical reflection.)
	float dMax = min( uShoreCfg.y / sy, SHORE_MAX_DIST );
	if ( dMax < 24.0 ) return 0.0;

	const float T0 = 12.0;
	float ratio = pow( dMax / T0, 1.0 / float( SHORE_STEPS ) );
	float t = T0;
	float prevT = T0;
	float prevGap = 1e6;
	float hit = 0.0;
	float tHit = 0.0;

	for ( int k = 0; k < SHORE_STEPS; k ++ ) {
		t *= ratio;
		vec2 uv = ( orig.xz + rd * t - uShoreRect.xy ) * uShoreRect.zw;
		// Outside the map is open water, not a wall: sample clamped and zero the
		// height so the ray passes through and can still hit on re-entry. Done
		// with step() rather than a branch so the fetch stays in uniform flow.
		float inside = step( 0.0, uv.x ) * step( uv.x, 1.0 )
		             * step( 0.0, uv.y ) * step( uv.y, 1.0 );
		float terr = texture2D( uShoreTex, clamp( uv, 0.0, 1.0 ) ).a * uShoreCfg.x * inside;
		float gap = t * sy - terr;               // positive = ray still above ground
		if ( gap < 0.0 ) {
			hit = 1.0;
			// One linear refinement between the last two samples. The proxy is
			// 12 m per texel, so a second bisection buys nothing a linear
			// interpolation of the gap does not already give.
			tHit = mix( prevT, t, prevGap / max( prevGap - gap, 1e-4 ) );
			break;
		}
		prevT = t;
		prevGap = gap;
	}
	if ( hit < 0.5 ) return 0.0;

	vec2 uvH = clamp( ( orig.xz + rd * tHit - uShoreRect.xy ) * uShoreRect.zw, 0.0, 1.0 );
	hitCol = texture2D( uShoreTex, uvH ).rgb * uShoreLight;
	return 1.0;
}
#endif

void main() {

	vec2 p = vWorld.xz;
	vec3 Ng = normalize( vNrm );
	vec3 V  = normalize( cameraPosition - vWorld );

	// World-XZ projected tangent frame. The water surface is never anywhere
	// near vertical, so projecting +X onto it can never degenerate.
	vec3 T = normalize( vec3( 1.0, 0.0, 0.0 ) - Ng * Ng.x );
	vec3 B = cross( Ng, T );

	float speed = max( vFlow, 0.42 );
	// The flow map is authored per vertex from the streambed (see createWater), so
	// neighbouring vertices genuinely disagree about direction — which is the whole
	// point, and which also means the interpolated vector is short wherever they
	// disagree most. Renormalise, or the advection rate quietly drops exactly at
	// the confluences and rock deflections the flow map exists to show.
	vec2  fd    = vFlowDir;
	float fdLen = length( fd );
	fd = fdLen > 1e-3 ? fd / fdLen : vec2( 1.0, 0.0 );

	// --- three scrolling normal layers ------------------------------------
	// Cycle rate is tied to scale * speed so the two flow-map phases are always
	// about half a texture apart, whatever the water is doing.
	float s1 = 0.055, s2 = 0.30, s3 = 0.95;
	float f1 = 2.0 * s1 * speed, f2 = 1.4 * s2 * speed, f3 = 1.0 * s3 * speed;

	vec4 t1 = flowTex( uSwellTex, p * s1,                      fd * 0.5, uTime * f1 );
	vec4 t2 = flowTex( uChopTex,  p * s2 + vec2( 0.19, 0.71 ), fd * 0.5, uTime * f2 );
	#ifdef WATER_CHEAP
		// Half the fetch count on low spec: the capillary layer is the first
		// thing that stops being resolvable at a reduced pixel ratio anyway.
		vec4 t3 = t2;
	#else
		vec4 t3 = flowTex( uChopTex, p * s3 * 1.31 + vec2( 0.6, 0.2 ), fd * 0.5, uTime * f3 );
	#endif

	// Wave amplitude collapses in the shallows (there is no water column left
	// to move) and grows with flow.
	float amp = uWaveGain * ( 0.45 + 0.85 * clamp( vDepth * 1.6, 0.0, 1.0 ) )
	                      * ( 0.62 + 0.55 * clamp( vFlow * 0.5, 0.0, 1.6 ) );

	// Toksvig-style specular antialiasing, done by hand. Past a few tens of
	// metres one pixel covers many wavelets; keeping the full normal amplitude
	// there turns the sun glitter into a field of crawling white dots. Shrink
	// the normal and widen the lobe with distance instead — which is exactly
	// what a correctly filtered normal distribution would do.
	float viewDist = length( cameraPosition - vWorld );
	float far = clamp( viewDist / 55.0, 0.0, 1.0 );
	// 0.40 -> 0.58. Crushing the normal amplitude this hard is what left the
	// creek in r3_06 with no flow direction at all past ~50 m; the roughness
	// term below is the correct place to pay for the filtering, and it already
	// widens the lobe with distance.
	amp *= mix( 1.0, 0.58, far * far );

	float w3 = 0.34;
	#ifdef WATER_CHEAP
		w3 = 0.0;
	#endif

	vec2 n2d = ( t1.rg * 2.0 - 1.0 ) * 1.00
	         + ( t2.rg * 2.0 - 1.0 ) * 0.62
	         + ( t3.rg * 2.0 - 1.0 ) * w3;
	n2d *= amp;

	vec3 N = normalize( T * n2d.x + B * n2d.y + Ng );

	// --- expanding ripple rings -------------------------------------------
	float ringFoam = 0.0;
	for ( int i = 0; i < RIPPLE_COUNT; i ++ ) {
		vec4 r = uRipples[ i ];
		if ( r.w <= 0.0 ) continue;
		vec2 d = p - r.xy;
		float dist = length( d ) + 1e-4;
		float band = ( dist - r.z ) / 0.42;
		float ring = exp( -band * band ) * r.w;
		// Two lobes so a splash reads as a wake, not a single hoop.
		float band2 = ( dist - r.z * 0.55 ) / 0.30;
		ring += exp( -band2 * band2 ) * r.w * 0.45;
		vec2 rd = d / dist;
		N = normalize( N + ( T * rd.x + B * rd.y ) * ring * 0.85 );
		ringFoam += ring * 0.55;
	}

	float NdV = max( dot( N, V ), 0.02 );

	// --- what is under the water ------------------------------------------
	// The refraction offset is the surface normal's tangential component
	// scaled by the optical path, exactly as Snell would bend the view ray.
	vec2 refr = vec2( dot( N, T ), dot( N, B ) ) * uRefract * ( 0.35 + vDepth * 0.9 );

	float depth = mix( vDepth, sampleBakedDepth( p + refr ), uUseDepthTex );
	float depthGeom = mix( vDepth, sampleBakedDepth( p ), uUseDepthTex );

	// Pebbles, seen through the refracted offset. Two octaves so the bed has
	// macro variation and does not tile visibly in the shallows.
	vec3 pebbles = texture2D( uBedTex, ( p + refr ) * uBedScale ).rgb;
	vec3 macro   = texture2D( uBedTex, ( p + refr ) * uBedScale * 0.21 + 0.37 ).rgb;
	vec3 bedAlbedo = vBed * ( 0.68 + 0.72 * pebbles ) * ( 0.78 + 0.42 * macro.g );

	// The streambed is a diffuse surface like any other: it needs the same
	// irradiance the terrain around it gets, or the water reads as a flat
	// decal that never responds to the sun.
	float bedNdL = max( dot( Ng, uSunDir ), 0.0 );
	vec3 bedLight = uSunColor * bedNdL + uSkyHigh * 0.75;

	// Light reaching the bed and coming back: down the column and back up
	// along the view ray, so grazing angles read deeper. Classic Beer–Lambert.
	float path = max( depth, 0.0 ) * ( 1.0 + 1.0 / NdV ) * 0.5;
	vec3 trans = exp( -uAbsorb * path );
	vec3 below = ( bedAlbedo * trans + uScatter * ( 1.0 - trans ) * 2.2 ) * bedLight;

	// --- reflection --------------------------------------------------------
	// Wind streaks: very low frequency, so a big flat body has texture even
	// when every wavelet has mipped away.
	float wind = texture2D( uFoamTex, p * 0.0032 + vec2( uTime * 0.0018, 0.0 ) ).r;
	// TWO roughnesses, and keeping them apart is the fix for the ruled line where
	// the sea meets the sky in r6_00 (measured: sky 179/199/209 at row 131 stepping
	// to water 161/195/217 at row 132 — an 18-level break in red across one pixel).
	//   rough        — the lobe width and env-map blur. Grows with distance because
	//     one pixel comes to cover many wavelets. This is a FILTERING term.
	//   roughSurface — what the surface actually is: flow chop and aeration. Does
	//     NOT grow with distance, because a sea two kilometres away is not rougher
	//     than the same sea nearby, it is merely less resolved.
	// The Fresnel ceiling has to come off the second one. Driving it from the
	// filtered value made the far sea's grazing reflectance 0.57 instead of 0.74,
	// so 43% of the horizon band was dark deep-water colour showing through — which
	// is exactly the blue step measured above. Physically, water at two kilometres
	// and 88° of incidence is very nearly a mirror, and now it is.
	float roughSurface = clamp( 0.030 + vFlow * 0.028 + vFoam * 0.20, 0.02, 0.60 );
	float rough = clamp( roughSurface + far * far * 0.26 + wind * 0.06 * far, 0.02, 0.60 );
	vec3 R = reflect( -V, N );
	// The environment has no ground hemisphere, so fold downward rays back up
	// rather than sampling a black void at grazing incidence.
	R.y = R.y < 0.02 ? ( 0.02 - R.y * 0.45 ) : R.y;
	R = normalize( R );

	// Schlick with a ROUGHNESS CEILING (Lagarde / Karis). F0 = 0.02 for water,
	// but the grazing limit of a microfacet aggregate is bounded by its own
	// normal distribution, not by 1.0 — a chopped creek cannot mirror the sky
	// the way a millpond can. Straight Schlick × 0.93 handed a shallow creek
	// seen at any oblique angle a 93% sky reflection, which is the other half
	// of the flat neutral L=226 band across r3_06.
	float F0 = 0.02;
	float Fmax = max( 1.0 - roughSurface, F0 );
	float F = ( F0 + ( Fmax - F0 ) * pow( 1.0 - NdV, 5.0 ) ) * uFresnelMax;

	// The PMREM of a sunlit sky peaks in the tens; left unbounded a grazing
	// water surface becomes a mirror of the sun's aureole and blows the bloom.
	// 2.6 -> 1.8: 2.6 linear is where the shipped tonemap lands ~226/255, which
	// is exactly the flat value the r3_06 creek band measured.
	vec3 refl = min( sampleEnv( R, rough ), vec3( 1.8 ) );
	// J2. A big open water body has to have TEXTURE or it is a debug backdrop.
	// The wind field is a ~310 m tile, i.e. still ~290 px across at 2 km, so it
	// survives every mip level the wavelets do not. Cat's paws darken the sky
	// return where the surface is ruffled and brighten it where it is glassy.
	refl *= 0.86 + 0.28 * wind;

	#ifdef WATER_SHORE_REFLECT
	// Reflected shore. Gated on F, because below ~5% reflectance nothing that
	// comes back is visible and the march would be paid for a result that is
	// then multiplied by nothing; that single test culls the entire down-looking
	// near field. Confidence keys off roughSurface rather than rough: a rippled
	// surface genuinely cannot hold a coherent image, but a distant one can, and
	// this is the band where the reflection matters most.
	if ( F > 0.05 && uShoreCfg.z > 0.0 ) {
		vec3 shoreCol;
		float hitShore = marchShore( vWorld, R, shoreCol );
		float conf = hitShore * uShoreCfg.z
		           * ( 1.0 - smoothstep( 0.10, 0.38, roughSurface ) );
		// The ray was reflected off the PERTURBED normal, so the hit point walks
		// with the waves — that wobble is the whole reason a reflection in water
		// reads as water and not as a mirror lying on the ground.
		refl = mix( refl, shoreCol, conf );
	}
	#endif

	vec3 color = mix( below, refl, F );

	// --- sun glitter -------------------------------------------------------
	vec3 H = normalize( uSunDir + V );
	float NdH = max( dot( N, H ), 0.0 );
	// A very tight lobe on a high-frequency normal map is a machine for making
	// aliased white dots. Keep the lobe moderate and let the broad companion
	// term carry the sheen — that is what reads as a lit water surface.
	float shin = mix( 260.0, 70.0, clamp( rough * 2.6, 0.0, 1.0 ) );
	float spec = pow( NdH, shin ) * ( shin + 8.0 ) * 0.0060;
	spec += pow( NdH, 26.0 ) * 0.14;
	// J2: 0.45 -> 0.72 of the near-field strength retained at distance. A
	// glitter path on an open body is the single most recognisable "that is
	// water" cue there is, and killing it is why the valley reads as a painted
	// gradient in r3_00.
	vec3 specTerm = uSunColor * spec * uSpecGain * ( 0.30 + 0.9 * F ) * ( 1.0 - 0.28 * far );
	// J1: a hard ceiling on the highlight. 0.42 linear is where the shipped
	// tonemap + grade lands ~200/255, which is the cap the work order asks for
	// in shade; a surface actually facing the sun gets a wider allowance so a
	// real glitter path can still register.
	float openSun = smoothstep( 0.0, 0.35, dot( Ng, uSunDir ) );
	color += min( specTerm, vec3( mix( 0.42, 1.50, openSun ) ) );


	// --- foam --------------------------------------------------------------
	// Turbulence advects with the water and is sampled at two scales so the
	// foam has both body and lace. The lookup is in a FLOW-ALIGNED frame with
	// the along-stream axis compressed, because aerated water forms streaks
	// that run with the current — isotropic blobs read as sea spray, not river.
	vec2 fuv = vec2( dot( p, fd ) * 0.32, dot( p, vec2( -fd.y, fd.x ) ) );
	vec2 fdir = vec2( 1.0, 0.0 );   // flow is +x in this frame

	vec4 fa = flowTex( uFoamTex, fuv * 0.26 + vec2( 0.13, 0.62 ), fdir * 0.5, uTime * ( 0.55 * speed ) );
	#ifdef WATER_CHEAP
		vec4 fb = fa;
	#else
		vec4 fb = flowTex( uFoamTex, fuv * 0.88 + vec2( 0.71, 0.05 ), fdir * 0.5, uTime * ( 1.10 * speed ) );
	#endif
	// Weighted towards the smooth fbm: the billow channel is cellular, and
	// leaning on it turns the foam into a tiled honeycomb.
	float turb = fa.r * 0.78 + fb.g * 0.26 + fb.b * 0.16;

	// Shoreline band: everywhere the column is thinner than uFoamWidth.
	// ( GLSL leaves smoothstep undefined when edge0 >= edge1, so every
	//   "fade out as x rises" term is written as 1 - smoothstep. )
	float shoreBand = 1.0 - smoothstep( uFoamWidth * 0.10, uFoamWidth, max( depthGeom, 0.0 ) );
	// White water: driven by the flow speed derived from the bed gradient.
	// Threshold raised (2.1-5.2 -> 2.6-5.8): the creek's Manning-ish solve puts
	// an ordinary 20% reach at vFlow ~2.4, so the old curve was already calling
	// a third of it whitewater before the bed had done anything interesting.
	// A genuine cascade is forced to vFlow >= 5.0 and vFoam >= 0.92 upstream in
	// createWater(), so it still saturates.
	float white = smoothstep( 2.6, 5.8, vFlow ) + vFoam * 0.72;
	// Standing wave crests break on the upstream face of the chop.
	float crest = smoothstep( 0.42, 0.90, t2.b * 0.5 + t3.b * 0.5 ) * smoothstep( 2.4, 4.6, vFlow );

	// The turbulence is a *gate*, not a tint: without a wide multiplicative
	// range the foam floods to a flat white sheet and the water disappears.
	float foam = ( shoreBand * 0.58 + white * 0.44 + crest * 0.40 ) * ( 0.05 + 1.70 * turb ) + ringFoam;
	foam = smoothstep( 0.34, 1.00, foam );
	// Re-modulate AFTER the threshold. Without this a genuinely fast pitch
	// saturates every term at once and the whole chute becomes one flat white
	// stripe — the single most obvious "not real water" tell there is.
	foam = clamp( foam * ( 0.42 + 0.80 * turb ), 0.0, 0.90 );
	// A crisp lip right at the waterline stops the shoreline reading as a blur.
	foam = max( foam, ( 1.0 - smoothstep( 0.012, 0.075, max( depthGeom, 0.0 ) ) ) * ( 0.30 + 0.62 * turb ) );
	// J1, and the actual mechanism behind the "blown structureless white
	// ribbon": past ~55 m the turbulence texture has mipped to its own mean, so
	// the gate above stops being a coverage mask and starts being
	// smoothstep(mean) — which is much larger than mean(smoothstep) for any
	// broken surface. The result is that a creek that is 25% foam up close
	// resolves to a saturated sheet at distance. Traced across r3_06 at eight x
	// positions the band was 226/228/226 at x40 through 225/225/231 at x760:
	// flat neutral, no depth ramp, no bed, no flow. This is the correction.
	foam *= mix( 1.0, 0.60, far );

	float sunLit = max( dot( Ng, uSunDir ), 0.0 );
	vec3 foamLit = uFoamColor * ( uSkyHigh * 0.48 + uSunColor * ( 0.24 + 0.52 * sunLit ) );
	color = mix( color, foamLit, foam );

	// --- solar-disc glint (r6 review) --------------------------------------
	// Everything above is a stylised sheen with a taste ceiling on it, and the
	// surface's whole look is tuned around it, so it is untouched. What it
	// cannot do is produce a *specular hit*: the sixteen-shot set peaks at
	// L=242 with no true white anywhere in it, and sunlit water is one of the
	// three or four things in a landscape entitled to clip. This term is the
	// missing one, it is purely additive, and it is bounded by physics at both
	// ends rather than by taste.
	//
	// Treat the sun as a disc. For a small source L_out = D * Vis * F * NdL *
	// L_sun * Omega, and L_sun * Omega is by definition the irradiance, which is
	// PI * uSunColor — so no new light constant is invented here, the glint is
	// derived from the same sun the diffuse terms already use.
	//
	// The alpha is where the work is:
	//   * floored at GLINT_ALPHA, the GGX width of a *reflected* solar disc.
	//     Narrower than that and the model is resolving structure finer than its
	//     own light source, which is how you get single-pixel strobing.
	//   * widened by the unresolved wavelet slope variance, keyed to the world
	//     footprint of one pixel. This is the term that matters. Once a pixel
	//     covers many wavelets the aggregate normal distribution IS the lobe, so
	//     the open sea at a kilometre flattens into a broad sheen (~L235 at its
	//     brightest) instead of a field of crawling white dots — and it is also
	//     why only near water can reach white, which is both correct and what
	//     bounds the clipped-pixel count to a few hundred in a 2 MPix frame.
	//   * hard-clamped at F * the sun's own rendered radiance. A reflection is
	//     never brighter than what it reflects. At normal incidence F ~= 0.02
	//     gives ~50 units, which postfx's curve puts at display L243: bright,
	//     and correctly not white, because a 4% dielectric bounce of the sun is
	//     not a mirror. Past ~24° from grazing F clears the 595 units that
	//     postfx's specular escape makes paper white, and it clips.
	float pw = length( fwidth( p ) );                    // metres of surface per pixel
	float unres = clamp( pw / GLINT_FOOTPRINT, 0.0, 1.0 );
	// Squared: a footprint at a tenth of the wavelet scale leaves a hundredth of
	// the variance unresolved, not a tenth of it.
	float slopeVar = amp * amp * 0.09;                   // RMS slope ~0.3 at amp = 1
	float aG2 = GLINT_ALPHA * GLINT_ALPHA + unres * unres * slopeVar;
	float NdL = max( dot( N, uSunDir ), 0.0 );
	// Aerated water is a diffuser, and water the sun cannot see has nothing to
	// mirror; both have to gate the term or the foam line becomes a light source.
	float glintGate = openSun * ( 1.0 - clamp( foam * 1.6, 0.0, 1.0 ) );
	vec3 sunHue = uSunColor / max( max( uSunColor.r, max( uSunColor.g, uSunColor.b ) ), 1e-4 );
	vec3 glint = discSpecular( NdH, NdV, NdL, aG2, F, uSunColor ) * glintGate;
	color += min( glint, sunHue * ( SUN_DISC_RADIANCE * F * glintGate ) );

	// --- flow-aligned streaking -------------------------------------------
	// Bands of smooth and aerated water that run WITH the current at a ~2 m
	// scale. It survives the mip chain long after every wavelet has gone, so it
	// is what tells a viewer at 60 m which way the creek is running — the thing
	// r3_06 has none of. Costs one fetch.
	float streak = texture2D( uFoamTex,
		vec2( dot( p, fd ) * 0.11 + uTime * 0.020 * speed,
		      dot( p, vec2( -fd.y, fd.x ) ) * 0.52 ) ).g;
	color *= 0.90 + 0.22 * streak;

	// --- alpha -------------------------------------------------------------
	// Opaque as soon as there is a real column of water. What is "seen through"
	// the surface is the shaded streambed computed above — letting the terrain
	// itself blend through as well double-counts it, and the terrain's own
	// albedo pattern immediately reads as a texture painted on the water.
	float a = clamp( depthGeom / uEdgeFade, 0.0, 1.0 );

	// --- depth-based shoreline softening (r6 review) -----------------------
	// uEdgeFade is a band in METRES OF WATER, so how hard the waterline reads
	// depends entirely on how steep the bed happens to be there and on how far
	// away it is. On a 1:1 bank at forty metres a 0.22 m band is a single pixel,
	// and a single-pixel alpha step against opaque terrain is exactly the "hard
	// intersection" the review is describing — a diagnostic capture of the ford
	// shows the creek terminating against the berm on a ruled line.
	//
	// fwidth( depth ) is how much the water column changes across one pixel, so
	// depth / fwidth( depth ) is the distance to the waterline measured IN
	// PIXELS. Feathering on that gives a waterline exactly uShorePixels wide at
	// every distance and on every bed slope, with no depth buffer, no soft-
	// particle pass and no extra fetch. Take whichever of the two bands is the
	// softer, so the authored metric band still governs a shallow beach where it
	// is already wider than the screen-space floor.
	//
	// Both gradients are consulted because they fail in opposite places: the
	// valley body's depth comes from an 8-bit baked texture whose derivative is
	// zero across a quantisation plateau, while the creek's per-vertex depth is
	// smooth but coarse. max() of the two is well behaved on both surfaces.
	float dW = max( max( fwidth( depthGeom ), fwidth( vDepth ) ), 1e-6 );
	a = min( a, clamp( depthGeom / ( dW * uShorePixels ), 0.0, 1.0 ) );

	// Foam laps a little way up the wet bank, past the waterline.
	a = max( a, foam * smoothstep( -0.05, 0.035, depthGeom ) * 0.95 );

	// --- wetted margin -----------------------------------------------------
	// The band of bank the water has just been over. depthGeom is negative there
	// (it is the height of the bed ABOVE the surface) and those fragments were
	// simply discarding, even though the surface mesh already reaches well up the
	// bank — the far corners of the shoreline quads are emitted for exactly that
	// reason. So this costs no geometry and no fetch.
	//
	// Authored as soaked ground rather than as standing water: bedAlbedo is
	// already the submerged tint (0.62x dry), so lifting it back to ~0.7x dry and
	// letting the foam term lap over it reads as wet rock, where blending towards
	// the water colour would read as a shelf of water perched on the bank.
	//
	// Two things keep this from simply relocating the hard edge it exists to
	// remove. The height-above-water term is continuous through the waterline and
	// is flat at zero on
	// the water side, so the band is one ramp with no join in it rather than a
	// second edge that happens to start where the first one ended. And it carries
	// the same screen-space feather as the water, so on a near-vertical bank —
	// where the whole margin would otherwise fall inside a single pixel — it
	// fades itself out to nothing instead of drawing a line. That is also why the
	// two compose rather than fight: the feather governs exactly the steep and
	// distant cases where the margin is suppressed, and the margin governs the
	// shallow near cases where the waterline was never hard to begin with.
	float above = max( -depthGeom, 0.0 );
	float wetA = uWetMargin.y
	           * ( 1.0 - smoothstep( 0.0, uWetMargin.x, above ) )
	           * clamp( ( uWetMargin.x - above ) / max( dW * uShorePixels, 1e-6 ), 0.0, 1.0 );
	// Crossfade the colour over a few centimetres of depth either side of the
	// line, for the same reason.
	float wetMix = clamp( ( -depthGeom + uWetMargin.x * 0.10 ) / ( uWetMargin.x * 0.20 ), 0.0, 1.0 );
	color = mix( color, bedAlbedo * bedLight * 1.05 + foamLit * ( foam * 0.55 ), wetMix );
	a = max( a, wetA );

	a = clamp( a * uOpacity, 0.0, 1.0 );

	// The offshore apron has to end somewhere, and wherever that is it must not
	// be an edge. Ramp its alpha to zero over the last uDissolveBand.x metres so
	// the sheet dissolves into the (fully hazed) far terrain behind it instead of
	// terminating on a polygon boundary in mid-air.
	vec2 dEdge = min( vWorld.xz - uDissolveRect.xy, uDissolveRect.zw - vWorld.xz );
	a *= 1.0 - clamp( ( uDissolveBand.x - min( dEdge.x, dEdge.y ) ) * uDissolveBand.y, 0.0, 1.0 );

	if ( a < 0.004 ) discard;

	gl_FragColor = vec4( color, a );

	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}
`;

const FALL_VERT = /* glsl */`
attribute float aU;
attribute float aV;
attribute float aSheet;
attribute float aSpeed;

varying vec3  vWorld;
varying vec3  vNrm;
varying float vU;
varying float vV;
varying float vSheet;
varying float vSpeed;

#include <fog_pars_vertex>

void main() {
	vWorld = position;
	vNrm   = normalize( normal );
	vU = aU; vV = aV; vSheet = aSheet; vSpeed = aSpeed;

	vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );
	gl_Position = projectionMatrix * mvPosition;

	#include <fog_vertex>
}
`;

const FALL_FRAG = /* glsl */`
uniform float uTime;
uniform sampler2D uChopTex;
uniform sampler2D uFoamTex;
uniform sampler2D envMap;
uniform float uEnvIntensity;
uniform vec3  uSunDir;
uniform vec3  uSunColor;
uniform vec3  uSkyLow;
uniform vec3  uSkyHigh;
uniform vec3  uFoamColor;
uniform vec3  uScatter;
uniform float uOpacity;

varying vec3  vWorld;
varying vec3  vNrm;
varying float vU;
varying float vV;
varying float vSheet;
varying float vSpeed;

#include <fog_pars_fragment>
#include <cube_uv_reflection_fragment>
${GLSL_COMMON}

vec3 sampleEnv( vec3 dir, float rough ) {
	#ifdef ENVMAP_TYPE_CUBE_UV
		return textureCubeUV( envMap, dir, rough ).rgb * uEnvIntensity;
	#else
		return waterSkyFallback( dir, uSkyLow, uSkyHigh, uSunDir, uSunColor ) * uEnvIntensity;
	#endif
}

void main() {

	// Falling water is stretched vertically: sample with a tall, narrow tile so
	// the noise becomes streaks, and scroll it down fast.
	float sp = 0.55 + vSpeed * 0.055;

	vec2 uvA = vec2( vU * 2.6,  vV * 0.16 - uTime * sp * 1.00 );
	vec2 uvB = vec2( vU * 5.7 + 0.31, vV * 0.37 - uTime * sp * 1.55 );
	vec2 uvC = vec2( vU * 11.0 + 0.77, vV * 0.85 - uTime * sp * 2.30 );

	float sA = texture2D( uChopTex, uvA ).b;
	float sB = texture2D( uFoamTex, uvB ).r;
	float sC = texture2D( uFoamTex, uvC ).g;

	// Streaks multiply; clumps of aerated water form and break apart as they
	// fall, which is what stops a waterfall looking like a sheet of plastic.
	float streak = sA * 0.55 + sB * 0.62 + sC * 0.45;
	// Water that has already broken over a lip is aerated from the first metre;
	// starting the ramp at zero makes the top of every fall read as grey glass.
	float aer = clamp( 0.38 + vV * 0.20, 0.0, 1.0 );
	float body = mix( 0.55, 1.0, aer );

	// Perturbed normal: mostly the rock face normal, kicked by the streaks.
	vec3 Ng = normalize( vNrm );
	vec3 V  = normalize( cameraPosition - vWorld );
	vec3 T  = normalize( cross( vec3( 0.0, 1.0, 0.0 ), Ng ) + vec3( 1e-4, 0.0, 0.0 ) );
	vec3 Bt = cross( Ng, T );
	vec3 N = normalize( Ng + T * ( sB - 0.5 ) * 0.9 + Bt * ( sC - 0.5 ) * 0.4 );

	float NdV = max( dot( N, V ), 0.03 );
	float F = 0.02 + 0.98 * pow( 1.0 - NdV, 5.0 );

	vec3 R = reflect( -V, N );
	R.y = R.y < 0.02 ? ( 0.02 - R.y * 0.45 ) : R.y;
	vec3 refl = sampleEnv( normalize( R ), 0.22 );

	// Aerated water is a dense cloud of droplets: it scatters the whole sky
	// hemisphere, not a thin slice of it. Under-weighting the ambient here is
	// what turns a fall on a shaded face into a grey stripe.
	float sunLit = max( dot( N, uSunDir ), 0.0 );
	vec3 lit = uSkyHigh * 1.75 + uSunColor * ( 0.30 + 0.70 * sunLit );

	// Thin, clear water at the lip; churned white below.
	vec3 col = mix( uScatter * lit * 3.0, uFoamColor * lit, clamp( aer * 0.85 + streak * 0.55, 0.0, 1.0 ) );
	col += refl * F * 0.55;
	// Back-lit spray: the sun blowing through the sheet from behind.
	col += uSunColor * pow( max( dot( -V, uSunDir ), 0.0 ), 6.0 ) * aer * 0.35;

	// Keep a solid floor under the alpha: a sheet that thins to 15 % lets the
	// shaded rock behind it dominate and the fall reads as wet stone, not water.
	float alpha = vSheet * body * clamp( 0.58 + streak * 0.80, 0.0, 1.0 );
	alpha *= smoothstep( 0.0, 0.16, vU ) * ( 1.0 - smoothstep( 0.84, 1.0, vU ) );
	alpha = clamp( alpha * uOpacity, 0.0, 1.0 );

	if ( alpha < 0.005 ) discard;

	gl_FragColor = vec4( col, alpha );

	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}
`;

const SPRAY_VERT = /* glsl */`
uniform float uPixelScale;
uniform float uTime;

attribute vec4 aParams;   // size(m), life 0..1, kind, seed

varying float vLife;
varying float vKind;
varying float vSeed;
varying vec3  vWorld;

#include <fog_pars_vertex>

void main() {

	vLife = aParams.y;
	vKind = aParams.z;
	vSeed = aParams.w;
	vWorld = position;

	if ( aParams.y <= 0.0 ) {
		gl_Position = vec4( 2.0, 2.0, 2.0, 1.0 );
		gl_PointSize = 0.0;
		return;
	}

	vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );
	gl_Position = projectionMatrix * mvPosition;

	// Mist swells as it ages, droplets do not.
	float grow = mix( 1.0, 1.0 + ( 1.0 - aParams.y ) * 1.3, step( 0.5, aParams.z ) );
	gl_PointSize = clamp( aParams.x * grow * uPixelScale / max( -mvPosition.z, 0.6 ), 1.0, 180.0 );

	#include <fog_vertex>
}
`;

const SPRAY_FRAG = /* glsl */`
uniform sampler2D uPuff;
uniform vec3 uSunColor;
uniform vec3 uSkyHigh;
uniform vec3 uFoamColor;
uniform float uOpacity;

varying float vLife;
varying float vKind;
varying float vSeed;
varying vec3  vWorld;

#include <fog_pars_fragment>

void main() {

	vec2 uv = gl_PointCoord;
	// Cheap per-particle rotation so 400 sprites are not 400 copies.
	float s = sin( vSeed * 6.28318 ), c = cos( vSeed * 6.28318 );
	uv = vec2( c * ( uv.x - 0.5 ) - s * ( uv.y - 0.5 ), s * ( uv.x - 0.5 ) + c * ( uv.y - 0.5 ) ) + 0.5;

	vec4 t = texture2D( uPuff, uv );

	// Mist (kind 1) fades in then out; droplets (kind 0) just fade out.
	float fade = mix( vLife, smoothstep( 0.0, 0.25, 1.0 - vLife ) * vLife, step( 0.5, vKind ) );
	float a = t.a * fade;
	// Mist has to stay thin: a plunge pool wants a veil you can see the rock
	// through, not one opaque cotton ball. Droplets stay comparatively solid.
	a *= mix( 0.80, 0.22, step( 0.5, vKind ) );
	a = clamp( a * uOpacity, 0.0, 1.0 );
	if ( a < 0.004 ) discard;

	vec3 lit = uSkyHigh * 0.85 + uSunColor * 0.55;
	vec3 col = uFoamColor * lit * t.rgb;

	gl_FragColor = vec4( col, a );

	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}
`;

const WET_VERT = /* glsl */`
attribute float aWet;

varying vec3  vWorld;
varying vec3  vNrm;
varying float vWet;

#include <fog_pars_vertex>

void main() {
	vWorld = position;
	vNrm = normalize( normal );
	vWet = aWet;

	vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );
	gl_Position = projectionMatrix * mvPosition;

	#include <fog_vertex>
}
`;

const WET_FRAG = /* glsl */`
uniform float uTime;
uniform sampler2D uFoamTex;
uniform vec3  uSunDir;
uniform vec3  uSunColor;
uniform vec3  uSkyHigh;
uniform vec3  uWetTint;
uniform float uOpacity;

varying vec3  vWorld;
varying vec3  vNrm;
varying float vWet;

#include <fog_pars_fragment>
${GLSL_COMMON}

void main() {

	// Break the band up so wet rock has a shape instead of a stencil edge.
	float n = texture2D( uFoamTex, vWorld.xz * 0.11 ).r * 0.6
	        + texture2D( uFoamTex, vWorld.xz * 0.037 + 0.4 ).g * 0.6;
	float wet = clamp( vWet * ( 0.45 + 1.15 * n ), 0.0, 1.0 );
	wet = smoothstep( 0.06, 0.75, wet );
	if ( wet < 0.005 ) discard;

	float a = wet * uOpacity;

	vec3 N = normalize( vNrm );
	vec3 V = normalize( cameraPosition - vWorld );
	vec3 H = normalize( uSunDir + V );
	float NdV = max( dot( N, V ), 0.02 );
	float F = 0.03 + 0.97 * pow( 1.0 - NdV, 5.0 );

	// Wet rock is dark and glossy: darken by alpha, then put a real specular
	// sheen back on top. Premultiplied alpha is what makes both possible in
	// one pass — rgb is added, dst is only scaled by ( 1 - a ).
	vec3 col = uWetTint * a;
	float NdH = max( dot( N, H ), 0.0 );
	float spec = pow( NdH, 90.0 ) * 0.55
	           + pow( NdH, 12.0 ) * 0.05;
	// Ceiling: at grazing incidence F -> 1 and the broad lobe alone was worth
	// ~0.18 linear of unbounded sun, which on a wet bank right at the waterline
	// is a second white band beside the one the creek was already making.
	col += min( uSunColor * spec * F * 3.2 * wet, vec3( 0.55 ) );
	col += uSkyHigh * F * 0.12 * wet;

	// Solar-disc glint (r6 review). The bounded sheen above is what the wet band
	// is tuned around and it stays; this is the additive term that lets a soaked
	// rock at the mirror angle actually register as a *hit* rather than topping
	// out at the 0.55 linear ceiling (display ~L202) it has been held to. See
	// WET_GLINT_A2 for why the lobe is the rock's roughness and not the film's.
	// Same F * source-radiance clamp as the water surface: a glint is never
	// brighter than the thing it is reflecting.
	float NdL = max( dot( N, uSunDir ), 0.0 );
	vec3 sunHue = uSunColor / max( max( uSunColor.r, max( uSunColor.g, uSunColor.b ) ), 1e-4 );
	vec3 glint = discSpecular( NdH, NdV, NdL, WET_GLINT_A2, F, uSunColor ) * wet;
	col += min( glint, sunHue * ( SUN_DISC_RADIANCE * F * wet ) );

	gl_FragColor = vec4( col, a );

	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}
`;

// ===========================================================================
// 3. Small helpers
// ===========================================================================

/** sRGB-authored streambed albedo per SurfaceId, converted to working space. */
function bedTintTable() {
  const src = [
    [0.34, 0.27, 0.20], // DIRT
    [0.25, 0.20, 0.14], // LOAM
    [0.36, 0.35, 0.33], // ROCK
    [0.50, 0.45, 0.37], // GRAVEL
    [0.24, 0.29, 0.16], // GRASS
    [0.28, 0.22, 0.15], // ROOT
    [0.24, 0.18, 0.13], // MUD
    [0.78, 0.81, 0.84], // SNOW
  ];
  return src.map(([r, g, b]) => new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace));
}

/**
 * DRY surface albedo per SurfaceId, for the shore reflection proxy. Deliberately
 * a separate table from `bedTintTable()`: that one is submerged ground seen
 * through water and is authored 0.62x here, which would make the reflected shore
 * two stops darker than the shore itself.
 *
 * Values are the terrain's own family (terrainMaterial.js keeps sunlit rock at
 * 0.35-0.45 after the r3 A3 knockdown), not a guess — a reflection that does not
 * match the thing it reflects reads worse than no reflection at all.
 */
function shoreTintTable() {
  const src = [
    [0.30, 0.24, 0.18], // DIRT
    [0.22, 0.18, 0.13], // LOAM
    [0.40, 0.39, 0.37], // ROCK
    [0.46, 0.43, 0.38], // GRAVEL
    [0.26, 0.31, 0.17], // GRASS
    [0.26, 0.21, 0.15], // ROOT
    [0.22, 0.17, 0.12], // MUD
    [0.80, 0.83, 0.86], // SNOW
  ];
  return src.map(([r, g, b]) => new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace));
}

/** In-place [0.25, 0.5, 0.25] smoothing of an array, endpoints pinned. */
function smoothArray(arr, passes, tmp) {
  const n = arr.length;
  if (n < 3) return arr;
  for (let p = 0; p < passes; p++) {
    tmp[0] = arr[0];
    tmp[n - 1] = arr[n - 1];
    for (let i = 1; i < n - 1; i++) tmp[i] = arr[i - 1] * 0.25 + arr[i] * 0.5 + arr[i + 1] * 0.25;
    for (let i = 0; i < n; i++) arr[i] = tmp[i];
  }
  return arr;
}

// ===========================================================================
// 4. createWater
// ===========================================================================

export function createWater(ctx) {
  const scene = ctx && ctx.scene;
  const terrain = ctx && ctx.terrain;
  const settings = (ctx && ctx.settings) || {};
  const quality = (ctx && ctx.quality) || 'high';
  const lowSpec = quality === 'low';

  const group = new THREE.Group();
  group.name = 'water';

  // ---- graceful degradation: without terrain there is nothing to fill -----
  if (!terrain || typeof terrain.sampleHeight !== 'function' || !scene) {
    return {
      group,
      surfaces: [],
      heightAt() { return null; },
      depthAt() { return 0; },
      flowAt(x, z, out) { if (out) out.set(0, 0, 0); return out; },
      isSubmerged() { return false; },
      waterLevel: 0,
      creekPath: new Float32Array(0),
      update() {},
      resize() {},
      dispose() {},
    };
  }

  scene.add(group);

  const seed = (ctx && ctx.seed) || 1;
  const rng = makeRng(subSeed(seed, 'water:main'));
  const bounds = terrain.bounds || { minX: -1000, maxX: 1000, minZ: -1000, maxZ: 1000 };
  const BED_TINTS = bedTintTable();

  // =========================================================================
  // 4.1 Textures
  // =========================================================================
  const texSwell = makeSwellTexture(subSeed(seed, 'water:swell'));
  const texChop = makeChopTexture(subSeed(seed, 'water:chop'));
  const texFoam = makeFoamTexture(subSeed(seed, 'water:foam'));
  const texBed = makeBedTexture(subSeed(seed, 'water:bed'));
  const texPuff = makePuffTexture(subSeed(seed, 'water:puff'));

  const maxAniso = (ctx.renderer && ctx.renderer.capabilities
    && ctx.renderer.capabilities.getMaxAnisotropy) ? ctx.renderer.capabilities.getMaxAnisotropy() : 1;
  const aniso = Math.min(maxAniso, lowSpec ? 2 : 8);
  texSwell.anisotropy = texChop.anisotropy = texFoam.anisotropy = texBed.anisotropy = aniso;

  // =========================================================================
  // 4.2 Valley water level
  // =========================================================================
  // terrain.creekLevel is the mean height of the flat wet cells. It is a good
  // starting guess but it is derived from a field that also fires on damp
  // benches high on the mountain, so it is sanity-checked against the actual
  // low end of the heightfield before anything is flooded with it.
  const SCAN = 14;                                       // metres per probe
  const scanNX = Math.max(2, Math.floor((bounds.maxX - bounds.minX) / SCAN));
  const scanNZ = Math.max(2, Math.floor((bounds.maxZ - bounds.minZ) / SCAN));
  let hMin = Infinity;
  const scanH = new Float32Array(scanNX * scanNZ);
  for (let j = 0; j < scanNZ; j++) {
    const z = bounds.minZ + (j + 0.5) * SCAN;
    for (let i = 0; i < scanNX; i++) {
      const x = bounds.minX + (i + 0.5) * SCAN;
      const h = terrain.sampleHeight(x, z);
      scanH[j * scanNX + i] = h;
      if (h < hMin) hMin = h;
    }
  }
  if (!isFinite(hMin)) hMin = (bounds.minY || 0);

  let waterLevel = clamp(
    typeof terrain.creekLevel === 'number' ? terrain.creekLevel : hMin + 4,
    hMin + 0.8, hMin + 20);

  // Never let the "lake" swallow the mountain: bisect down until at most 18%
  // of the world is under water.
  {
    const total = scanH.length;
    let lo = hMin + 0.4, hi = waterLevel;
    for (let it = 0; it < 14; it++) {
      let under = 0;
      for (let k = 0; k < total; k++) if (scanH[k] < waterLevel) under++;
      if (under / total <= 0.18) break;
      hi = waterLevel;
      waterLevel = (lo + hi) * 0.5;
    }
  }

  // =========================================================================
  // 4.3 Trace the creek down the drainage line
  // =========================================================================
  // The heightfield already has an erosion pass, so real gullies exist; the
  // trace just has to stay in them. Pure steepest descent falls out of a gully
  // the moment the floor flattens, so the score mixes four terms: how much
  // height is lost, how channel-like the cross section is (positive curvature
  // perpendicular to travel), how wet the cell is (terrain's flow accumulation
  // proxy — the single most reliable "a stream goes here" signal available),
  // and how straight the step is. Real water has momentum too.

  const N_DIRS = 25;
  const SPREAD = 1.75;         // radians of the candidate fan (±100°)
  const PERP = 5.0;            // metres to each side when measuring concavity

  function stepScoreDown(x, z, curH, dirX, dirZ, a) {
    const ca = Math.cos(a), sa = Math.sin(a);
    const nx = x + ca * TRACE_STEP;
    const nz = z + sa * TRACE_STEP;
    if (!terrain.inBounds(nx, nz)) return -1e9;
    const h = terrain.sampleHeight(nx, nz);
    const hL = terrain.sampleHeight(nx - sa * PERP, nz + ca * PERP);
    const hR = terrain.sampleHeight(nx + sa * PERP, nz - ca * PERP);
    const conc = (hL + hR) * 0.5 - h;
    const wet = terrain.sampleWetness ? terrain.sampleWetness(nx, nz) : 0;
    const align = ca * dirX + sa * dirZ;
    return (curH - h) * 1.00 + conc * 1.15 + wet * 2.60 + align * 0.90;
  }

  function stepScoreUp(x, z, curH, dirX, dirZ, a) {
    const ca = Math.cos(a), sa = Math.sin(a);
    const nx = x + ca * TRACE_STEP;
    const nz = z + sa * TRACE_STEP;
    if (!terrain.inBounds(nx, nz)) return -1e9;
    const h = terrain.sampleHeight(nx, nz);
    if (h < curH - 0.10) return -1e9;                 // must gain ground
    const hL = terrain.sampleHeight(nx - sa * PERP, nz + ca * PERP);
    const hR = terrain.sampleHeight(nx + sa * PERP, nz - ca * PERP);
    const conc = (hL + hR) * 0.5 - h;
    const wet = terrain.sampleWetness ? terrain.sampleWetness(nx, nz) : 0;
    const align = ca * dirX + sa * dirZ;
    // Prefer a moderate, steady climb: charging at the steepest wall leaves
    // the channel and ends up on a ridge.
    const rise = h - curH;
    return conc * 1.45 + wet * 3.20 + align * 1.15 - Math.abs(rise - TRACE_STEP * 0.16) * 0.55;
  }

  function trace(sx, sz, dx, dz, uphill, maxSteps, out) {
    let x = sx, z = sz;
    let dirX = dx, dirZ = dz;
    const dl = Math.hypot(dirX, dirZ) || 1;
    dirX /= dl; dirZ /= dl;
    for (let s = 0; s < maxSteps; s++) {
      const curH = terrain.sampleHeight(x, z);
      if (!uphill && curH <= waterLevel + 0.15) { out.push(x, z); break; }
      const base = Math.atan2(dirZ, dirX);
      let bestA = 0, bestS = -Infinity;
      for (let k = 0; k < N_DIRS; k++) {
        const a = base + (k / (N_DIRS - 1) - 0.5) * SPREAD;
        const sc = uphill ? stepScoreUp(x, z, curH, dirX, dirZ, a)
          : stepScoreDown(x, z, curH, dirX, dirZ, a);
        if (sc > bestS) { bestS = sc; bestA = a; }
      }
      if (bestS <= -1e8) break;
      dirX = Math.cos(bestA); dirZ = Math.sin(bestA);
      x += dirX * TRACE_STEP;
      z += dirZ * TRACE_STEP;
      out.push(x, z);
      if (!terrain.inBounds(x, z)) break;
    }
    return out;
  }

  // Seed the trace at the trail's creek crossing so the ford the trail builder
  // dished into the tread actually has a creek in it. Falling back to the
  // wettest low cell keeps the creek alive if the trail has no crossing.
  let seedX = 0, seedZ = 0, haveSeed = false;
  const trail = ctx.trail;
  if (trail && Array.isArray(trail.features)) {
    for (const f of trail.features) {
      if (f && f.type === 'creekCrossing' && f.position) {
        seedX = f.position.x; seedZ = f.position.z; haveSeed = true; break;
      }
    }
  }
  if (!haveSeed) {
    let bestScore = -Infinity;
    for (let j = 0; j < scanNZ; j++) {
      for (let i = 0; i < scanNX; i++) {
        const x = bounds.minX + (i + 0.5) * SCAN;
        const z = bounds.minZ + (j + 0.5) * SCAN;
        const h = scanH[j * scanNX + i];
        if (h < waterLevel + 8) continue;
        const wet = terrain.sampleWetness ? terrain.sampleWetness(x, z) : 0;
        const sc = wet * 3 - (h - waterLevel) * 0.002;
        if (sc > bestScore) { bestScore = sc; seedX = x; seedZ = z; haveSeed = true; }
      }
    }
  }

  // Initial direction: straight down the local gradient.
  terrain.sampleNormal(seedX, seedZ, _v1);
  let dnX = _v1.x, dnZ = _v1.z;
  if (Math.hypot(dnX, dnZ) < 1e-3) { dnX = 0; dnZ = -1; }

  const upPts = [];
  trace(seedX, seedZ, -dnX, -dnZ, true, 130, upPts);
  const downPts = [];
  trace(seedX, seedZ, dnX, dnZ, false, 520, downPts);

  // Concatenate: reversed upstream leg, the seed, then the downstream leg.
  const rawPts = [];
  for (let i = upPts.length - 2; i >= 0; i -= 2) rawPts.push(upPts[i], upPts[i + 1]);
  rawPts.push(seedX, seedZ);
  for (let i = 0; i < downPts.length; i += 2) rawPts.push(downPts[i], downPts[i + 1]);

  // ---- smooth, then snap to the thalweg, then smooth again ---------------
  let pathX = [], pathZ = [];
  for (let i = 0; i < rawPts.length; i += 2) { pathX.push(rawPts[i]); pathZ.push(rawPts[i + 1]); }

  if (pathX.length >= 4) {
    const tmp = new Float64Array(pathX.length);
    smoothArray(pathX, 4, tmp);
    smoothArray(pathZ, 4, tmp);

    // Thalweg snap: slide each node sideways towards the lowest point within
    // a few metres. Two rounds is enough to centre the ribbon in the gully.
    for (let round = 0; round < 2; round++) {
      for (let i = 1; i < pathX.length - 1; i++) {
        const tx = pathX[i + 1] - pathX[i - 1];
        const tz = pathZ[i + 1] - pathZ[i - 1];
        const tl = Math.hypot(tx, tz) || 1;
        const px = -tz / tl, pz = tx / tl;
        let bestOff = 0, bestH = Infinity;
        for (let k = -4; k <= 4; k++) {
          const o = k * 1.1;
          const h = terrain.sampleHeight(pathX[i] + px * o, pathZ[i] + pz * o);
          if (h < bestH) { bestH = h; bestOff = o; }
        }
        pathX[i] += px * bestOff * 0.45;
        pathZ[i] += pz * bestOff * 0.45;
      }
      smoothArray(pathX, 2, tmp);
      smoothArray(pathZ, 2, tmp);
    }
  }

  // ---- resample to a uniform station spacing -----------------------------
  const stX = [], stZ = [];
  if (pathX.length >= 2) {
    let carry = 0;
    stX.push(pathX[0]); stZ.push(pathZ[0]);
    for (let i = 1; i < pathX.length; i++) {
      const dx = pathX[i] - pathX[i - 1];
      const dz = pathZ[i] - pathZ[i - 1];
      const seg = Math.hypot(dx, dz);
      if (seg < 1e-4) continue;
      let t = (STATION_DS - carry) / seg;
      while (t <= 1 && stX.length < MAX_STATIONS) {
        stX.push(pathX[i - 1] + dx * t);
        stZ.push(pathZ[i - 1] + dz * t);
        t += STATION_DS / seg;
      }
      carry = (carry + seg) % STATION_DS;
      if (stX.length >= MAX_STATIONS) break;
    }
  }

  const NS = stX.length;
  const hasCreek = NS >= 8;

  // =========================================================================
  // 4.4 Creek stations: bed, surface, width, flow, waterfalls
  // =========================================================================
  const sBedRaw = new Float64Array(Math.max(NS, 1));
  const sBed = new Float64Array(Math.max(NS, 1));
  const sSurf = new Float64Array(Math.max(NS, 1));
  const sHalf = new Float64Array(Math.max(NS, 1));
  const sFlow = new Float64Array(Math.max(NS, 1));
  const sArc = new Float64Array(Math.max(NS, 1));
  const sDirX = new Float64Array(Math.max(NS, 1));
  const sDirZ = new Float64Array(Math.max(NS, 1));
  const sFoam = new Float64Array(Math.max(NS, 1));
  const falls = [];

  if (hasCreek) {
    // tangents + arc length
    let arc = 0;
    for (let i = 0; i < NS; i++) {
      const a = Math.max(0, i - 1), b = Math.min(NS - 1, i + 1);
      let dx = stX[b] - stX[a], dz = stZ[b] - stZ[a];
      const l = Math.hypot(dx, dz) || 1;
      sDirX[i] = dx / l; sDirZ[i] = dz / l;
      if (i > 0) arc += Math.hypot(stX[i] - stX[i - 1], stZ[i] - stZ[i - 1]);
      sArc[i] = arc;
    }
    const totalArc = Math.max(arc, 1);

    // Width: a creek gathers catchment as it descends, so it widens roughly
    // with the square root of the distance travelled. Noise keeps it organic,
    // and the confinement of the banks pinches it in the gorges.
    const wRng = makeRng(subSeed(seed, 'water:width'));
    const wNoise = [];
    for (let i = 0; i < 64; i++) wNoise.push(wRng());
    for (let i = 0; i < NS; i++) {
      const u = sArc[i] / totalArc;
      const wn = wNoise[(i * 0.11 | 0) % 64];
      let half = (0.62 + 1.95 * Math.sqrt(u)) * (0.72 + 0.56 * wn) * 0.5;

      // Bed: the minimum across the section is the true channel floor.
      let bedMin = Infinity;
      const probe = half * 2.4 + 1.2;
      for (let k = -5; k <= 5; k++) {
        const o = (k / 5) * probe;
        const h = terrain.sampleHeight(stX[i] - sDirZ[i] * o, stZ[i] + sDirX[i] * o);
        if (h < bedMin) bedMin = h;
      }
      // Confinement: how fast the banks climb tells us how much room there is.
      const bankL = terrain.sampleHeight(stX[i] - sDirZ[i] * (probe + 3), stZ[i] + sDirX[i] * (probe + 3));
      const bankR = terrain.sampleHeight(stX[i] + sDirZ[i] * (probe + 3), stZ[i] - sDirX[i] * (probe + 3));
      const relief = Math.max(0, (bankL + bankR) * 0.5 - bedMin);
      half *= clamp(1.35 - relief * 0.10, 0.55, 1.35);

      sHalf[i] = clamp(half, 0.5, 5.0);
      sBedRaw[i] = bedMin;
    }

    // Smoothed bed. The water surface follows the *smoothed* floor, which is
    // what makes the difference between a stream and a heightfield-shaped
    // ribbon: real water bridges the small stuff and pours over the big stuff,
    // and the small stuff pokes back through as riffles and midstream rocks.
    for (let i = 0; i < NS; i++) sBed[i] = sBedRaw[i];
    smoothArray(sBed, 8, new Float64Array(NS));
    // ...but edge-preserving. A ±12 m box filter turns a 4 m cliff band into a
    // 24 m ramp, which is exactly the feature the waterfalls key off. Blend the
    // raw floor back in wherever its local gradient is genuinely cliff-like, so
    // the step survives while everything gentler stays smooth.
    for (let i = 1; i < NS - 1; i++) {
      const rawGrad = Math.abs(sBedRaw[i + 1] - sBedRaw[i - 1]) / (2 * STATION_DS);
      const w = clamp01((rawGrad - 0.45) / 0.55);
      sBed[i] = sBed[i] + (sBedRaw[i] - sBed[i]) * w * w;
    }
    smoothArray(sBed, 1, new Float64Array(NS));

    // Depth target, then a strictly non-increasing surface.
    for (let i = 0; i < NS; i++) {
      const u = sArc[i] / totalArc;
      sSurf[i] = sBed[i] + 0.16 + 0.46 * Math.sqrt(u);
    }
    for (let i = 1; i < NS; i++) {
      // Water never flows uphill. 2 mm/station of forced descent keeps the
      // surface strictly monotone without visibly tilting it.
      if (sSurf[i] > sSurf[i - 1] - 0.002) sSurf[i] = sSurf[i - 1] - 0.002;
    }
    // ...and never below the floor. The minimal non-increasing function that
    // dominates ( bed + eps ) is its suffix maximum, so one backward pass gives
    // the exact pooling solution: sills upstream of a rise fill to the sill.
    {
      let suffix = -Infinity;
      for (let i = NS - 1; i >= 0; i--) {
        const need = sBed[i] + 0.06;
        if (need > suffix) suffix = need;
        if (sSurf[i] < suffix) sSurf[i] = suffix;
      }
      // Cap the fake lakes that a pathological sill would otherwise create.
      for (let i = 0; i < NS; i++) {
        if (sSurf[i] - sBed[i] > 3.2) sSurf[i] = sBed[i] + 3.2;
      }
    }

    // Flow speed from the surface gradient. Manning-ish: v ~ sqrt(S) scaled by
    // hydraulic radius. Absolute realism is not the point — this number drives
    // the normal-map scroll rate and the foam density, so it has to be smooth
    // and it has to spike exactly where the bed steepens.
    for (let i = 0; i < NS; i++) {
      const a = Math.max(0, i - 1), b = Math.min(NS - 1, i + 1);
      const drop = sSurf[a] - sSurf[b];
      const run = Math.max(1e-3, sArc[b] - sArc[a]);
      const S = clamp(drop / run, 0, 3);
      const depth = Math.max(0.05, sSurf[i] - sBed[i]);
      sFlow[i] = clamp(0.55 + 5.4 * Math.sqrt(S) * Math.pow(depth, 0.33), 0.25, 7.5);
    }
    smoothArray(sFlow, 2, new Float64Array(NS));

    // Pre-baked foam: bed roughness across the section (rocks in the stream)
    // plus anywhere the raw floor pokes near the surface.
    for (let i = 0; i < NS; i++) {
      const probe = sHalf[i];
      let mn = Infinity, mx = -Infinity;
      for (let k = -3; k <= 3; k++) {
        const o = (k / 3) * probe;
        const h = terrain.sampleHeight(stX[i] - sDirZ[i] * o, stZ[i] + sDirX[i] * o);
        if (h < mn) mn = h;
        if (h > mx) mx = h;
      }
      const rough = clamp01((mx - mn) / 0.55);
      const shallow = clamp01(1 - (sSurf[i] - sBedRaw[i]) / 0.45);
      // Rough or shallow bed alone does not aerate water — it has to be moving.
      sFoam[i] = clamp01(rough * 0.55 + shallow * 0.45) * clamp01((sFlow[i] - 1.4) / 3.0) * 0.75;
    }
    smoothArray(sFoam, 1, new Float64Array(NS));

    // ---- waterfalls ------------------------------------------------------
    // A contiguous run of steep surface gradient with a real total drop.
    let i = 1;
    while (i < NS) {
      const grad = (sSurf[i - 1] - sSurf[i]) / STATION_DS;
      if (grad > FALL_MIN_GRAD) {
        let j = i;
        while (j < NS && (sSurf[j - 1] - sSurf[j]) / STATION_DS > FALL_MIN_GRAD * 0.55) j++;
        const drop = sSurf[i - 1] - sSurf[Math.min(j, NS - 1)];
        if (drop >= FALL_MIN_DROP) {
          falls.push({ lip: i - 1, base: Math.min(j, NS - 1), drop });
        }
        i = j + 1;
      } else i++;
    }
    // Water going over a lip is aerated from the lip down, so the ribbon
    // inside the fall span is white too. Without this the ribbon there is a
    // near-vertical sheet whose normal points away from the sun — it goes dark
    // and reads as a slab of wet rock beside the white cascade above it.
    for (const f of falls) {
      for (let k = f.lip; k <= f.base; k++) {
        sFoam[k] = Math.max(sFoam[k], 0.92);
        sFlow[k] = Math.max(sFlow[k], 5.0);
      }
    }
    // Foam boils for a few metres below every fall.
    for (const f of falls) {
      const reach = Math.min(NS - 1, f.base + Math.ceil(6 + f.drop * 1.6));
      for (let k = f.base; k <= reach; k++) {
        const t = 1 - (k - f.base) / Math.max(1, reach - f.base);
        sFoam[k] = clamp01(Math.max(sFoam[k], t * 0.95));
        sFlow[k] = Math.max(sFlow[k], 2.4 * t + 0.8);
      }
    }
  }

  // =========================================================================
  // 4.5 Creek ribbon geometry (+ puddles, same draw call)
  // =========================================================================
  const surfacePos = [];
  const surfaceNrm = [];
  const surfaceDepth = [];
  const surfaceFlow = [];
  const surfaceFoam = [];
  const surfaceShore = [];
  const surfaceDist = [];
  const surfaceFlowDir = [];
  const surfaceBed = [];
  const surfaceIdx = [];

  function pushSurfaceVertex(x, y, z, nx, ny, nz, depth, flow, foam, shore, dist, fdx, fdz, tint) {
    surfacePos.push(x, y, z);
    surfaceNrm.push(nx, ny, nz);
    surfaceDepth.push(depth);
    surfaceFlow.push(flow);
    surfaceFoam.push(foam);
    surfaceShore.push(shore);
    surfaceDist.push(dist);
    surfaceFlowDir.push(fdx, fdz);
    surfaceBed.push(tint.r, tint.g, tint.b);
  }

  function bedTintAt(x, z, out) {
    const id = terrain.sampleMaterial ? (terrain.sampleMaterial(x, z) | 0) : 3;
    const c = BED_TINTS[id] || BED_TINTS[3];
    out.copy(c);
    // Submerged ground is darker and less saturated than the dry surface.
    const wet = terrain.sampleWetness ? terrain.sampleWetness(x, z) : 0.5;
    // Submerged rock is wet rock: albedo drops to roughly 60 % of dry.
    out.multiplyScalar(0.62 - 0.10 * wet);
    return out;
  }

  if (hasCreek) {
    const base = 0;
    for (let i = 0; i < NS; i++) {
      const rx = -sDirZ[i], rz = sDirX[i];
      const halfGeo = sHalf[i] * RIBBON_OVERHANG;
      // Surface normal: flat across the flow, tilted along it.
      const a = Math.max(0, i - 1), b = Math.min(NS - 1, i + 1);
      const dy = sSurf[b] - sSurf[a];
      const ds = Math.max(1e-3, sArc[b] - sArc[a]);
      const fy = dy / ds;
      const fl = 1 / Math.sqrt(1 + fy * fy);
      // n = right × forward, with right horizontal — always +Y dominant.
      const nx = -sDirX[i] * fy * fl;
      const ny = fl;
      const nz = -sDirZ[i] * fy * fl;
      // Deepest point of the section, for the lateral velocity profile below.
      const midDepth = Math.max(0.05, sSurf[i] - sBed[i]);
      for (let j = 0; j < CROSS; j++) {
        const t = (j / (CROSS - 1)) * 2 - 1;
        const px = stX[i] + rx * t * halfGeo;
        const pz = stZ[i] + rz * t * halfGeo;
        const bh = terrain.sampleHeight(px, pz);
        bedTintAt(px, pz, _col);

        // ---- flow map (r6 review) ----------------------------------------
        // Until now every vertex on the ribbon carried the *centreline tangent*,
        // so the whole creek advected as one rigid sheet regardless of what the
        // bed underneath it was doing — a scrolling wallpaper with a two-phase
        // reset on it. Two per-vertex terms fix that, both free at runtime:
        //
        // 1. Direction. A heightfield normal's horizontal part points straight
        //    down the local gradient (for h = a*x the normal is (-a, 1, 0), and
        //    -a is downhill when a > 0), so it is the steepest-descent direction
        //    of the actual streambed at this vertex. Blending it into the
        //    centreline tangent makes the surface motion curl around midstream
        //    rocks, tuck into the thalweg and fan out over a shallow bar.
        terrain.sampleNormal(px, pz, _v2);
        let fdx = sDirX[i];
        let fdz = sDirZ[i];
        const gl2 = Math.hypot(_v2.x, _v2.z);
        if (gl2 > 1e-4) {
          const bx = fdx + (_v2.x / gl2) * FLOW_BED_BEND;
          const bz = fdz + (_v2.z / gl2) * FLOW_BED_BEND;
          // A rock whose local gradient points back upstream must DEFLECT the
          // flow, never reverse it — hence the downstream guard rather than a
          // straight lerp.
          if (bx * sDirX[i] + bz * sDirZ[i] > 0.2) {
            const bl = Math.hypot(bx, bz) || 1;
            fdx = bx / bl; fdz = bz / bl;
          }
        }

        // 2. Speed. A channel has a velocity profile: fastest over the deepest
        //    part of the section, dropping towards nothing at the wetted
        //    margins. A constant speed across the width is the other half of
        //    why a scrolling ribbon reads as a conveyor belt, and it is what
        //    makes the two-phase cycle rate visibly uniform bank to bank.
        // aDepth stays SIGNED — the outer ribbon vertices are deliberately buried
        // in the bank and their negative depth is what produces the shoreline
        // fade and the wetted margin. Only the profile uses the clamped copy.
        const depthHere = sSurf[i] - bh;
        const prof = 0.30 + 0.70 * Math.sqrt(clamp01(Math.max(0, depthHere) / midDepth));

        pushSurfaceVertex(
          px, sSurf[i], pz, nx, ny, nz,
          depthHere, sFlow[i] * prof, sFoam[i], Math.abs(t), sArc[i],
          fdx, fdz, _col);
      }
    }
    for (let i = 0; i < NS - 1; i++) {
      // The ribbon runs *through* the waterfall spans: the vertical sheet is
      // laid over the top of it, so the dark, fast water between the white
      // streaks still reads and the fall does not start with a seam.
      for (let j = 0; j < CROSS - 1; j++) {
        const v00 = base + i * CROSS + j;
        const v10 = v00 + CROSS;
        const v01 = v00 + 1;
        const v11 = v10 + 1;
        surfaceIdx.push(v00, v10, v01, v01, v10, v11);
      }
    }
  }

  // ---- puddles on the wet, flat parts of the trail corridor ---------------
  if (trail && typeof trail.sampleAt === 'function' && !lowSpec) {
    const pRng = makeRng(subSeed(seed, 'water:puddles'));
    const trailLen = trail.length || 1000;
    const stepT = Math.max(0.0015, 4.0 / trailLen);
    let placed = 0;
    for (let t = 0.02; t < 0.985 && placed < MAX_PUDDLES; t += stepT) {
      const s = trail.sampleAt(t);
      if (!s || !s.position) break;
      const px0 = s.position.x, pz0 = s.position.z;
      const wet = terrain.sampleWetness ? terrain.sampleWetness(px0, pz0) : 0;
      const slope = terrain.sampleSlope ? terrain.sampleSlope(px0, pz0) : 1;
      if (wet < 0.42 || slope > 0.17) continue;
      if (pRng() > 0.30) continue;
      // Offset into a rut rather than sitting dead centre.
      const lat = (pRng() * 2 - 1) * Math.max(0.2, (s.width || 2) * 0.36);
      const bx = s.binormal ? s.binormal.x : 1;
      const bz = s.binormal ? s.binormal.z : 0;
      const cx = px0 + bx * lat, cz = pz0 + bz * lat;
      const r = 0.45 + pRng() * 1.25;

      // Only where the ground really is a dish — otherwise it is a sticker.
      const hC = terrain.sampleHeight(cx, cz);
      let rim = -Infinity;
      for (let k = 0; k < 8; k++) {
        const a2 = (k / 8) * Math.PI * 2;
        rim = Math.max(rim, terrain.sampleHeight(cx + Math.cos(a2) * r, cz + Math.sin(a2) * r));
      }
      if (rim - hC < 0.035) continue;
      const surfY = hC + Math.min(0.11, (rim - hC) * 0.75);

      const centreIndex = surfacePos.length / 3;
      bedTintAt(cx, cz, _col);
      pushSurfaceVertex(cx, surfY, cz, 0, 1, 0, surfY - hC, 0.02, 0.0, 0.0, 0.0, 1, 0, _col);
      const SEG = 12;
      for (let k = 0; k <= SEG; k++) {
        const a2 = (k / SEG) * Math.PI * 2;
        const ex = cx + Math.cos(a2) * r * (0.86 + 0.30 * pRng());
        const ez = cz + Math.sin(a2) * r * (0.86 + 0.30 * pRng());
        const eh = terrain.sampleHeight(ex, ez);
        bedTintAt(ex, ez, _col);
        pushSurfaceVertex(ex, surfY, ez, 0, 1, 0, surfY - eh, 0.02, 0.0, 1.0, 0.0, 1, 0, _col);
      }
      for (let k = 0; k < SEG; k++) {
        surfaceIdx.push(centreIndex, centreIndex + 1 + k, centreIndex + 2 + k);
      }
      placed++;
    }
  }

  // =========================================================================
  // 4.6 Valley water body + its baked depth texture
  // =========================================================================
  let valleyMesh = null;
  let depthTex = null;
  const depthRect = new THREE.Vector4(0, 0, 1, 1);
  const depthDecode = new THREE.Vector2(18, 2);   // depth = r*18 - 2 metres
  // The fine valley grid's extent and divisions, published for the offshore
  // apron below so its innermost ring can land on exactly the same vertices.
  let valleyRect = null;

  {
    // bbox of everything under water
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity, any = false;
    for (let j = 0; j < scanNZ; j++) {
      for (let i = 0; i < scanNX; i++) {
        if (scanH[j * scanNX + i] >= waterLevel + 0.5) continue;
        const x = bounds.minX + (i + 0.5) * SCAN;
        const z = bounds.minZ + (j + 0.5) * SCAN;
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
        any = true;
      }
    }
    if (any) {
      const PAD = 45;
      minX = Math.max(bounds.minX, minX - PAD); maxX = Math.min(bounds.maxX, maxX + PAD);
      minZ = Math.max(bounds.minZ, minZ - PAD); maxZ = Math.min(bounds.maxZ, maxZ + PAD);
      const w = maxX - minX, d = maxZ - minZ;

      // --- depth texture ---
      // ~2.5 m/texel, capped so the bake stays under a couple of hundred
      // thousand heightfield probes (a few tens of milliseconds at boot).
      let tw = clamp(Math.round(w / 2.5), 32, 768) | 0;
      let th = clamp(Math.round(d / 2.5), 32, 768) | 0;
      const budget = lowSpec ? 60000 : 200000;
      if (tw * th > budget) {
        const s = Math.sqrt(budget / (tw * th));
        tw = Math.max(32, Math.round(tw * s));
        th = Math.max(32, Math.round(th * s));
      }
      const dData = new Uint8Array(tw * th * 4);
      for (let j = 0; j < th; j++) {
        const z = minZ + (j + 0.5) / th * d;
        for (let i = 0; i < tw; i++) {
          const x = minX + (i + 0.5) / tw * w;
          const dep = waterLevel - terrain.sampleHeight(x, z);
          const enc = clamp01((dep + depthDecode.y) / depthDecode.x);
          const k = (j * tw + i) * 4;
          dData[k] = (enc * 255) | 0;
          dData[k + 1] = dData[k];
          dData[k + 2] = dData[k];
          dData[k + 3] = 255;
        }
      }
      depthTex = new THREE.DataTexture(dData, tw, th, THREE.RGBAFormat, THREE.UnsignedByteType);
      depthTex.wrapS = depthTex.wrapT = THREE.ClampToEdgeWrapping;
      depthTex.minFilter = THREE.LinearFilter;
      depthTex.magFilter = THREE.LinearFilter;
      depthTex.generateMipmaps = false;
      depthTex.colorSpace = THREE.NoColorSpace;
      depthTex.needsUpdate = true;
      depthRect.set(minX, minZ, 1 / w, 1 / d);

      // --- surface grid ---
      const CELL = lowSpec ? 16 : 9;
      const gx = Math.max(2, Math.ceil(w / CELL));
      const gz = Math.max(2, Math.ceil(d / CELL));
      valleyRect = { minX, minZ, maxX, maxZ, w, d, gx, gz };
      const vPos = [], vNrm = [], vDep = [], vFlw = [], vFoa = [], vSho = [], vDis = [], vFd = [], vBed = [];
      const vIdx = [];
      const gridIndex = new Int32Array((gx + 1) * (gz + 1)).fill(-1);
      const hCache = new Float32Array((gx + 1) * (gz + 1));
      for (let j = 0; j <= gz; j++) {
        const z = minZ + (j / gz) * d;
        for (let i = 0; i <= gx; i++) {
          const x = minX + (i / gx) * w;
          hCache[j * (gx + 1) + i] = terrain.sampleHeight(x, z);
        }
      }
      const drift = new THREE.Vector2(0.82, 0.57).normalize();
      for (let j = 0; j <= gz; j++) {
        const z = minZ + (j / gz) * d;
        for (let i = 0; i <= gx; i++) {
          const x = minX + (i / gx) * w;
          const h = hCache[j * (gx + 1) + i];
          // Vertices well above the waterline are still emitted: they are the
          // far corners of the shoreline quads, they render at alpha 0, and
          // dropping them would punch holes out of the bank.
          if (h > waterLevel + 9.0) continue;
          gridIndex[j * (gx + 1) + i] = vPos.length / 3;
          bedTintAt(x, z, _col);
          vPos.push(x, waterLevel, z);
          vNrm.push(0, 1, 0);
          const dep = waterLevel - h;
          vDep.push(dep);
          vFlw.push(0.10);
          vFoa.push(0);
          vSho.push(0);
          vDis.push(0);
          // Flow map (r6 review). Open water does not scroll as a single sheet:
          // close to a shore the surface motion runs ALONG it, because that is
          // the only direction with room to go. The depth field's gradient is
          // the shore normal, so its perpendicular is the alongshore direction —
          // free, since hCache already holds the bathymetry. Blend from the
          // prevailing wind drift in deep water to alongshore in the shallows,
          // which is what gives a bay its own surface direction instead of the
          // whole lake advecting the same way.
          const im = Math.max(0, i - 1), ip = Math.min(gx, i + 1);
          const jm = Math.max(0, j - 1), jp = Math.min(gz, j + 1);
          const gxh = hCache[j * (gx + 1) + ip] - hCache[j * (gx + 1) + im];
          const gzh = hCache[jp * (gx + 1) + i] - hCache[jm * (gx + 1) + i];
          let fdx = drift.x, fdz = drift.y;
          const gm = Math.hypot(gxh, gzh);
          if (gm > 1e-4) {
            // Perpendicular to the bed gradient, oriented to agree with the
            // drift so neighbouring vertices never point 180° apart (which
            // would interpolate to a zero-length, and therefore stationary,
            // flow vector halfway between them).
            let ax = -gzh / gm, az = gxh / gm;
            if (ax * drift.x + az * drift.y < 0) { ax = -ax; az = -az; }
            const shoreW = clamp01(1 - dep / 6.0);
            fdx = drift.x * (1 - shoreW) + ax * shoreW;
            fdz = drift.y * (1 - shoreW) + az * shoreW;
            const fl2 = Math.hypot(fdx, fdz) || 1;
            fdx /= fl2; fdz /= fl2;
          }
          vFd.push(fdx, fdz);
          vBed.push(_col.r, _col.g, _col.b);
        }
      }
      for (let j = 0; j < gz; j++) {
        for (let i = 0; i < gx; i++) {
          const a = gridIndex[j * (gx + 1) + i];
          const b = gridIndex[j * (gx + 1) + i + 1];
          const c = gridIndex[(j + 1) * (gx + 1) + i];
          const e = gridIndex[(j + 1) * (gx + 1) + i + 1];
          if (a < 0 || b < 0 || c < 0 || e < 0) continue;
          const h0 = hCache[j * (gx + 1) + i], h1 = hCache[j * (gx + 1) + i + 1];
          const h2 = hCache[(j + 1) * (gx + 1) + i], h3 = hCache[(j + 1) * (gx + 1) + i + 1];
          if (h0 > waterLevel && h1 > waterLevel && h2 > waterLevel && h3 > waterLevel) continue;
          vIdx.push(a, c, b, b, c, e);
        }
      }

      if (vIdx.length > 0) {
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.Float32BufferAttribute(vPos, 3));
        g.setAttribute('normal', new THREE.Float32BufferAttribute(vNrm, 3));
        g.setAttribute('aDepth', new THREE.Float32BufferAttribute(vDep, 1));
        g.setAttribute('aFlow', new THREE.Float32BufferAttribute(vFlw, 1));
        g.setAttribute('aFoam', new THREE.Float32BufferAttribute(vFoa, 1));
        g.setAttribute('aShore', new THREE.Float32BufferAttribute(vSho, 1));
        g.setAttribute('aDist', new THREE.Float32BufferAttribute(vDis, 1));
        g.setAttribute('aFlowDir', new THREE.Float32BufferAttribute(vFd, 2));
        g.setAttribute('aBed', new THREE.Float32BufferAttribute(vBed, 3));
        g.setIndex(vIdx);
        g.computeBoundingSphere();
        valleyMesh = new THREE.Mesh(g, null);   // material assigned below
        valleyMesh.name = 'water:valley';
        valleyMesh.frustumCulled = true;
        valleyMesh.renderOrder = 2;
      }
    }
  }

  // =========================================================================
  // 4.6b Offshore apron — the valley body has to run past the far plane
  // =========================================================================
  // The fine valley grid is built over the bounding box of everything actually
  // under water, so it *stops*, and in the r2 establishing shot its far edge
  // terminated in a hard straight line in mid-air with the sky's horizon band
  // beyond it. This is a coarse quad annulus laid from that grid's own
  // perimeter out to APRON_MARGIN metres past it, at exactly the same water
  // level, so the body reads as running to the horizon.
  //
  // Three things make it seamless rather than another edge:
  //
  //   1. Ring 0 is the valley rect's perimeter sampled with the *same* gx/gz
  //      divisions the fine grid used, so its vertices are coincident with the
  //      fine grid's boundary vertices — no T-junctions, and with a matched
  //      uWaveHeight/uSwellDir the two meshes displace identically there.
  //   2. Anywhere the terrain stands above the water line the apron is simply
  //      occluded by it (the terrain is opaque and this is a depth-tested
  //      transparent pass), which is what hides the outer boundary: terrain.js's
  //      far ring lifts the far-field valley floor clear of the water line, so
  //      the apron ends *underneath* dry land.
  //   3. Belt and braces for any camera that can still see past that: the outer
  //      APRON_DISSOLVE metres ramp alpha to zero.
  //
  // APRON_MARGIN is bounded by engine.js's CAMERA_FAR = 8000 m. Worst case is a
  // camera at one world corner and the apron's opposite corner:
  // sqrt(2) * (WORLD + APRON_MARGIN) = sqrt(2) * 5372 = 7597 m — inside the far
  // plane. This mesh is 1 draw call and has no update() cost at all.
  // =========================================================================
  const APRON_MARGIN = 2300;     // metres beyond the valley grid
  const APRON_RINGS = 10;        // radial divisions, geometric
  const APRON_DISSOLVE = 700;    // metres of alpha ramp at the outer boundary
  const APRON_DEPTH = 24;        // metres of water, i.e. "deep, no shoreline"

  let apronMesh = null;
  const apronRect = new THREE.Vector4(0, 0, 0, 0);

  if (valleyMesh && valleyRect) {
    const vr = valleyRect;

    // Perimeter as (edge, t) so every ring can be evaluated on its own expanded
    // rectangle with the same parameterisation — nested rectangles, so the strip
    // between consecutive rings can never self-intersect.
    const pe = [];
    const pt = [];
    for (let i = 0; i < vr.gx; i++) { pe.push(0); pt.push(i / vr.gx); }
    for (let j = 0; j < vr.gz; j++) { pe.push(1); pt.push(j / vr.gz); }
    for (let i = 0; i < vr.gx; i++) { pe.push(2); pt.push(i / vr.gx); }
    for (let j = 0; j < vr.gz; j++) { pe.push(3); pt.push(j / vr.gz); }
    const P = pe.length;

    // Deep water shows none of its bed (Beer-Lambert kills it inside ~3 m), so
    // one tint sampled at the middle of the body is all this needs.
    bedTintAt((vr.minX + vr.maxX) * 0.5, (vr.minZ + vr.maxZ) * 0.5, _col);
    const drift = new THREE.Vector2(0.82, 0.57).normalize();

    const aPos = [], aNrm = [], aDep = [], aFlw = [], aFoa = [], aSho = [];
    const aDis = [], aFd = [], aBed = [], aIdx = [];

    for (let k = 0; k <= APRON_RINGS; k++) {
      const m = APRON_MARGIN * Math.pow(k / APRON_RINGS, 1.7);
      const x0 = vr.minX - m, x1 = vr.maxX + m;
      const z0 = vr.minZ - m, z1 = vr.maxZ + m;
      for (let n = 0; n < P; n++) {
        const t = pt[n];
        let x, z;
        switch (pe[n]) {
          case 0: x = x0 + t * (x1 - x0); z = z0; break;
          case 1: x = x1; z = z0 + t * (z1 - z0); break;
          case 2: x = x1 - t * (x1 - x0); z = z1; break;
          default: x = x0; z = z1 - t * (z1 - z0); break;
        }
        aPos.push(x, waterLevel, z);
        aNrm.push(0, 1, 0);
        aDep.push(APRON_DEPTH);
        // Matched to the valley body's own still-water values so the shared
        // ring-0 vertices are displaced by exactly the same amount.
        aFlw.push(0.10);
        aFoa.push(0);
        aSho.push(0);
        aDis.push(0);
        aFd.push(drift.x, drift.y);
        aBed.push(_col.r, _col.g, _col.b);
      }
    }
    for (let k = 0; k < APRON_RINGS; k++) {
      const b0 = k * P, b1 = (k + 1) * P;
      for (let n = 0; n < P; n++) {
        const n2 = (n + 1) % P;
        aIdx.push(b0 + n, b1 + n, b0 + n2, b0 + n2, b1 + n, b1 + n2);
      }
    }

    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(aPos, 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute(aNrm, 3));
    g.setAttribute('aDepth', new THREE.Float32BufferAttribute(aDep, 1));
    g.setAttribute('aFlow', new THREE.Float32BufferAttribute(aFlw, 1));
    g.setAttribute('aFoam', new THREE.Float32BufferAttribute(aFoa, 1));
    g.setAttribute('aShore', new THREE.Float32BufferAttribute(aSho, 1));
    g.setAttribute('aDist', new THREE.Float32BufferAttribute(aDis, 1));
    g.setAttribute('aFlowDir', new THREE.Float32BufferAttribute(aFd, 2));
    g.setAttribute('aBed', new THREE.Float32BufferAttribute(aBed, 3));
    g.setIndex(aIdx);
    g.computeBoundingSphere();

    apronMesh = new THREE.Mesh(g, null);   // material assigned below
    apronMesh.name = 'water:apron';
    apronMesh.frustumCulled = true;
    apronMesh.renderOrder = 2;
    apronRect.set(
      vr.minX - APRON_MARGIN, vr.minZ - APRON_MARGIN,
      vr.maxX + APRON_MARGIN, vr.maxZ + APRON_MARGIN);
  }

  // =========================================================================
  // 4.6c Shore reflection proxy
  // =========================================================================
  // A top-down bake of the land around the water: RGB = dry albedo times a
  // geometric shade term, A = height above the water line normalised by HMAX.
  // The valley body's shader marches its reflected ray against it (see
  // marchShore in SURFACE_FRAG and the CONTRACT-NOTE at the head of this file
  // for why this rather than SSR).
  //
  // Sized at 256² over the whole world: ~12 m per texel, which at the 300 m to
  // 2 km range the reflected shore actually sits at subtends well under a degree
  // — a reflection in moving water is not a place where resolution is the
  // binding constraint. The bake is 65 k heightfield probes, the same order as
  // the valley depth bake beside it, and the texture is 256 KB so it stays
  // resident in cache through the march.
  let shoreTex = null;
  const shoreRect = new THREE.Vector4(0, 0, 1, 1);
  const shoreCfg = new THREE.Vector3(1, 1, 0);      // HMAX, tallest rise, strength
  const shoreLight = new THREE.Color(1, 1, 1);
  const shoreSteps = SHORE_STEPS[quality] != null ? SHORE_STEPS[quality] : SHORE_STEPS.high;

  if (valleyMesh && shoreSteps > 0) {
    const S = SHORE_TEX;
    const SHORE_TINTS = shoreTintTable();
    const wW = bounds.maxX - bounds.minX;
    const dD = bounds.maxZ - bounds.minZ;

    // Sun at bake time, for the geometric shade only — the light COLOUR is
    // applied live in the shader (uShoreLight), so a time-of-day change still
    // moves the reflection even though the bake does not re-run.
    if (ctx.sky && ctx.sky.sunDirection) _v1.copy(ctx.sky.sunDirection);
    else if (ctx.sun) _v1.copy(ctx.sun.position).normalize();
    else _v1.set(0.4, 0.55, -0.73).normalize();
    const sdx = _v1.x, sdy = _v1.y, sdz = _v1.z;

    const rise = new Float32Array(S * S);
    const shade = new Float32Array(S * S);
    const tintI = new Uint8Array(S * S);
    const veg = new Float32Array(S * S);
    let maxRise = 1;

    for (let j = 0; j < S; j++) {
      const z = bounds.minZ + (j + 0.5) / S * dD;
      for (let i = 0; i < S; i++) {
        const x = bounds.minX + (i + 0.5) / S * wW;
        const k = j * S + i;
        const h = terrain.sampleHeight(x, z);
        terrain.sampleNormal(x, z, _v2);
        const ndl = Math.max(0, _v2.x * sdx + _v2.y * sdy + _v2.z * sdz);
        // Forest canopy. From the water you are looking at the tops of trees,
        // not at the ground under them, and a reflected shoreline with no
        // canopy on it reads as a bald quarry — this is the single largest
        // contributor to the reflection looking like the place it reflects.
        let v = 0;
        if (typeof terrain.treelineAt === 'function') {
          const tl = terrain.treelineAt(x, z);
          // Slope from the normal we already have rather than a fifth terrain
          // query per texel — sampleSlope() is documented as exactly this angle,
          // and 65 k texels makes a spare query worth avoiding at boot.
          const slope = Math.acos(clamp(_v2.y, -1, 1));
          v = clamp01((tl - h) / 45) * clamp01(1 - slope / 0.95) * 0.8;
        }
        veg[k] = v;
        const r = h - waterLevel + v * SHORE_CANOPY_H;
        rise[k] = Math.max(0, r);
        if (rise[k] > maxRise) maxRise = rise[k];
        // Canopy shading is much flatter than bare ground: a conifer mass is a
        // volume, and lighting it off the terrain normal makes a reflected
        // hillside strobe between black and lit as the slope changes.
        shade[k] = (0.26 + 0.74 * ndl) * (1 - v) + (0.42 + 0.38 * ndl) * v;
        tintI[k] = terrain.sampleMaterial ? (terrain.sampleMaterial(x, z) & 7) : 2;
      }
    }

    const HMAX = Math.max(8, maxRise * 1.02);
    const data = new Uint8Array(S * S * 4);
    const canopy = new THREE.Color().setRGB(0.115, 0.165, 0.095, THREE.SRGBColorSpace);
    for (let k = 0; k < S * S; k++) {
      const base = SHORE_TINTS[tintI[k]] || SHORE_TINTS[2];
      const v = veg[k];
      const sh = shade[k];
      // Authored back out to sRGB bytes: the texture is tagged sRGB so three
      // decodes RGB on sample, while A (the height) stays linear, which is
      // exactly the split this needs.
      const lr = (base.r * (1 - v) + canopy.r * v) * sh;
      const lg = (base.g * (1 - v) + canopy.g * v) * sh;
      const lb = (base.b * (1 - v) + canopy.b * v) * sh;
      _col.setRGB(clamp01(lr), clamp01(lg), clamp01(lb));   // already linear
      _col.convertLinearToSRGB();
      data[k * 4] = Math.round(255 * clamp01(_col.r));
      data[k * 4 + 1] = Math.round(255 * clamp01(_col.g));
      data[k * 4 + 2] = Math.round(255 * clamp01(_col.b));
      data[k * 4 + 3] = Math.round(255 * clamp01(rise[k] / HMAX));
    }

    shoreTex = new THREE.DataTexture(data, S, S, THREE.RGBAFormat, THREE.UnsignedByteType);
    shoreTex.wrapS = shoreTex.wrapT = THREE.ClampToEdgeWrapping;
    shoreTex.minFilter = THREE.LinearFilter;
    shoreTex.magFilter = THREE.LinearFilter;
    // No mips on purpose: the march samples inside a loop with a break, where an
    // implicit derivative is undefined. One level makes that a non-question.
    shoreTex.generateMipmaps = false;
    shoreTex.colorSpace = THREE.SRGBColorSpace;
    shoreTex.needsUpdate = true;
    shoreRect.set(bounds.minX, bounds.minZ, 1 / wW, 1 / dD);
    shoreCfg.set(HMAX, maxRise, 1.0);
  }

  // =========================================================================
  // 4.7 Waterfall sheets
  // =========================================================================
  let fallMesh = null;
  const fallEmitters = [];
  if (hasCreek && falls.length) {
    const fPos = [], fNrm = [], fU = [], fV = [], fSheet = [], fSpeed = [], fIdx = [];
    const FCROSS = 7;

    for (const f of falls) {
      const lip = f.lip, bas = f.base;
      const rows = clamp(Math.ceil(f.drop / 0.7) + 3, 5, 42);
      const startY = sSurf[lip];
      const endY = sSurf[bas];
      const v0 = fPos.length / 3;
      // Horizontal speed carried over the lip; used for the free-fall arc.
      const vLip = Math.max(1.0, sFlow[lip]);

      for (let r = 0; r <= rows; r++) {
        const s = r / rows;
        // Position along the creek path between lip and base.
        const fi = lerp(lip, bas, s);
        const i0 = Math.min(NS - 1, Math.floor(fi));
        const i1 = Math.min(NS - 1, i0 + 1);
        const ft = fi - i0;
        const cx = lerp(stX[i0], stX[i1], ft);
        const cz = lerp(stZ[i0], stZ[i1], ft);
        const dx = lerp(sDirX[i0], sDirX[i1], ft);
        const dz = lerp(sDirZ[i0], sDirZ[i1], ft);
        const rx = -dz, rz = dx;

        // Free fall from the lip; where the rock is still in the way the sheet
        // clings to it instead, which is exactly how a cascade behaves.
        const tFall = (s * (sArc[bas] - sArc[lip])) / vLip;
        const yFree = startY - 0.5 * 9.81 * tFall * tFall;
        const hRock = terrain.sampleHeight(cx, cz);
        let y = Math.max(yFree, hRock + 0.10);
        if (r === rows) y = Math.max(endY, hRock + 0.06);

        terrain.sampleNormal(cx, cz, _v1);
        const half = lerp(sHalf[lip], sHalf[bas] * 1.22, s) * 1.05;
        const fallen = startY - y;

        for (let j = 0; j < FCROSS; j++) {
          const t = (j / (FCROSS - 1)) * 2 - 1;
          // Push the sheet clear of both the rock face and the creek ribbon
          // that runs underneath it, so the streaks read on top of the water
          // rather than fighting it for the same depth.
          fPos.push(
            cx + rx * t * half + _v1.x * 0.30,
            y + _v1.y * 0.10,
            cz + rz * t * half + _v1.z * 0.30);
          // The visible normal is the rock face's, flattened towards the
          // viewer-facing hemisphere so the sheet catches light along its span.
          const bulge = Math.cos(t * 1.15) * 0.55;
          _v2.set(_v1.x + rx * t * -0.35, _v1.y * 0.55 + bulge * 0.25, _v1.z + rz * t * -0.35).normalize();
          fNrm.push(_v2.x, _v2.y, _v2.z);
          fU.push((t + 1) * 0.5);
          fV.push(fallen);
          // The sheet thins at the lip (it is still one clear tongue) and
          // widens/thickens as it aerates.
          fSheet.push(clamp(0.52 + s * 0.95, 0, 1.25) * (1 - 0.22 * Math.abs(t)));
          fSpeed.push(vLip + 9.81 * tFall);
        }
      }
      for (let r = 0; r < rows; r++) {
        for (let j = 0; j < FCROSS - 1; j++) {
          const a = v0 + r * FCROSS + j;
          const b = a + FCROSS;
          fIdx.push(a, b, a + 1, a + 1, b, b + 1);
        }
      }

      // Mist at the plunge pool.
      fallEmitters.push({
        x: stX[bas], y: sSurf[bas], z: stZ[bas],
        radius: sHalf[bas] * 1.6 + 0.6,
        power: clamp(f.drop / 6, 0.25, 2.2),
      });
    }

    if (fIdx.length) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(fPos, 3));
      g.setAttribute('normal', new THREE.Float32BufferAttribute(fNrm, 3));
      g.setAttribute('aU', new THREE.Float32BufferAttribute(fU, 1));
      g.setAttribute('aV', new THREE.Float32BufferAttribute(fV, 1));
      g.setAttribute('aSheet', new THREE.Float32BufferAttribute(fSheet, 1));
      g.setAttribute('aSpeed', new THREE.Float32BufferAttribute(fSpeed, 1));
      g.setIndex(fIdx);
      g.computeBoundingSphere();
      fallMesh = new THREE.Mesh(g, null);
      fallMesh.name = 'water:falls';
      fallMesh.renderOrder = 3;
    }
  }

  // =========================================================================
  // 4.8 Wet-rock decal
  // =========================================================================
  let wetMesh = null;
  if (hasCreek && !lowSpec) {
    const WC = 9;
    const wPos = [], wNrm = [], wWet = [], wIdx = [];
    for (let i = 0; i < NS; i++) {
      const rx = -sDirZ[i], rz = sDirX[i];
      const band = sHalf[i] * 3.1 + 1.4;
      for (let j = 0; j < WC; j++) {
        const t = (j / (WC - 1)) * 2 - 1;
        const px = stX[i] + rx * t * band;
        const pz = stZ[i] + rz * t * band;
        const h = terrain.sampleHeight(px, pz);
        terrain.sampleNormal(px, pz, _v1);
        wPos.push(px + _v1.x * 0.06, h + 0.05, pz + _v1.z * 0.06);
        wNrm.push(_v1.x, _v1.y, _v1.z);
        // Wet where the rock is close to the waterline; splash zone widens
        // wherever the water is fast.
        const above = Math.max(0, h - sSurf[i]);
        const spray = clamp01(sFlow[i] / 3.2) * 0.55 + sFoam[i] * 0.5;
        const w = clamp01(1 - above / (0.35 + spray * 1.9)) * (1 - Math.abs(t) * 0.55);
        wWet.push(w);
      }
    }
    for (let i = 0; i < NS - 1; i++) {
      for (let j = 0; j < WC - 1; j++) {
        const a = i * WC + j, b = a + WC;
        wIdx.push(a, b, a + 1, a + 1, b, b + 1);
      }
    }
    // Extra soaked apron around each plunge pool.
    for (const e of fallEmitters) {
      const c = wPos.length / 3;
      terrain.sampleNormal(e.x, e.z, _v1);
      wPos.push(e.x, terrain.sampleHeight(e.x, e.z) + 0.06, e.z);
      wNrm.push(_v1.x, _v1.y, _v1.z);
      wWet.push(0.95);
      const R = e.radius * 3.4 + e.power * 3.0;
      const SEG = 16;
      for (let k = 0; k <= SEG; k++) {
        const a2 = (k / SEG) * Math.PI * 2;
        const ex = e.x + Math.cos(a2) * R, ez = e.z + Math.sin(a2) * R;
        terrain.sampleNormal(ex, ez, _v1);
        wPos.push(ex + _v1.x * 0.06, terrain.sampleHeight(ex, ez) + 0.05, ez + _v1.z * 0.06);
        wNrm.push(_v1.x, _v1.y, _v1.z);
        wWet.push(0);
      }
      for (let k = 0; k < SEG; k++) wIdx.push(c, c + 1 + k, c + 2 + k);
    }
    if (wIdx.length) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(wPos, 3));
      g.setAttribute('normal', new THREE.Float32BufferAttribute(wNrm, 3));
      g.setAttribute('aWet', new THREE.Float32BufferAttribute(wWet, 1));
      g.setIndex(wIdx);
      g.computeBoundingSphere();
      wetMesh = new THREE.Mesh(g, null);
      wetMesh.name = 'water:wetrock';
      wetMesh.renderOrder = -1;
    }
  }

  // =========================================================================
  // 4.9 Materials
  // =========================================================================
  const sky = ctx.sky;
  const skyLow = new THREE.Color().setRGB(0.42, 0.52, 0.66, THREE.SRGBColorSpace);
  const skyHigh = new THREE.Color().setRGB(0.30, 0.46, 0.72, THREE.SRGBColorSpace);
  // Aerated water is not a white card. 0.93/0.96/0.98 lit by sun + sky lands
  // above 1.0 linear, i.e. it clips before the tonemap has done anything;
  // measured river foam sits nearer 0.80-0.86 and it is slightly warm from the
  // silt it is carrying, not blue.
  const foamColor = new THREE.Color().setRGB(0.86, 0.88, 0.89, THREE.SRGBColorSpace);
  const scatterColor = new THREE.Color().setRGB(0.055, 0.175, 0.165, THREE.SRGBColorSpace);
  const wetTint = new THREE.Color().setRGB(0.055, 0.050, 0.045, THREE.SRGBColorSpace);

  const ripples = [];
  for (let i = 0; i < RIPPLE_COUNT; i++) ripples.push(new THREE.Vector4(0, 0, 0, 0));

  const dummyDepth = new THREE.DataTexture(
    new Uint8Array([255, 255, 255, 255]), 1, 1, THREE.RGBAFormat, THREE.UnsignedByteType);
  dummyDepth.needsUpdate = true;

  /**
   * Build a water-surface material. Textures and the shared-by-reference fog
   * array are attached AFTER the merge: UniformsUtils.merge() deep-clones, and
   * cloning a render-target texture (the sky's PMREM) nulls it with a console
   * warning, while cloning our DataTextures would duplicate every upload.
   */
  function makeSurfaceMaterial(useDepthTex) {
    const uniforms = THREE.UniformsUtils.merge([
      THREE.UniformsLib.fog,
      {
        uTime: { value: 0 },
        uWaveHeight: { value: 0 },
        uSwellDir: { value: new THREE.Vector2(0.83, 0.56) },
        uDepthRect: { value: new THREE.Vector4(0, 0, 1, 1) },
        uDepthDecode: { value: new THREE.Vector2(18, 2) },
        uUseDepthTex: { value: useDepthTex ? 1 : 0 },
        uEnvIntensity: { value: 1 },
        uSunDir: { value: new THREE.Vector3(0, 1, 0) },
        uSunColor: { value: new THREE.Color(1, 1, 1) },
        uSkyLow: { value: skyLow.clone() },
        uSkyHigh: { value: skyHigh.clone() },
        uAbsorb: { value: ABSORB.clone() },
        uScatter: { value: scatterColor.clone() },
        uFoamColor: { value: foamColor.clone() },
        uEdgeFade: { value: useDepthTex ? 0.55 : 0.22 },
        // J1/J2. The shore band is the width of water THINNER than this, and
        // the creek's design depth is only 0.16 m at the head and 0.62 m at the
        // outlet — so a 0.30 m band meant most of the creek's width was being
        // declared "shoreline" and handed to the foam term. It is a wetted
        // margin, not a channel: 0.13 m on the creek, 0.45 m on the valley
        // body (where an 0.85 m band was painting a blown white surf line right
        // down the coast in r3_00).
        uFoamWidth: { value: useDepthTex ? 0.45 : 0.13 },
        uRefract: { value: useDepthTex ? 0.75 : 0.28 },
        uWaveGain: { value: 1 },
        // 1 / metres per tile. Creek pebbles want to read at ~5 cm; the valley
        // bed is seen through metres of water, so it stays macro.
        uBedScale: { value: useDepthTex ? 0.20 : 1.55 },
        uOpacity: { value: 1 },
        uSpecGain: { value: 0.42 },
        uFresnelMax: { value: useDepthTex ? 0.84 : 0.93 },
        // Screen-space floor on the waterline feather. 2.6 px is wide enough
        // that the intersection never reads as a cut and narrow enough that a
        // near shoreline still has a definite edge. See the alpha block.
        uShorePixels: { value: 2.6 },
        // x = metres of bank counted as wetted, y = its peak alpha. The creek's
        // banks are steep and its own wet-rock decal already covers them, so it
        // takes a narrow band; the valley shore is shallow and has no decal, so
        // it takes a wide one and does most of the work there.
        uWetMargin: { value: new THREE.Vector2(useDepthTex ? 0.35 : 0.10, 0.55) },
        // Disabled unless the offshore apron switches it on (see below).
        uDissolveRect: { value: new THREE.Vector4(0, 0, 0, 0) },
        uDissolveBand: { value: new THREE.Vector2(0, 0) },
      },
    ]);
    // --- post-merge attachments (see the comment above) ---
    uniforms.uSwellTex = { value: texSwell };
    uniforms.uChopTex = { value: texChop };
    uniforms.uFoamTex = { value: texFoam };
    uniforms.uBedTex = { value: texBed };
    uniforms.uDepthTex = { value: (useDepthTex && depthTex) ? depthTex : dummyDepth };
    uniforms.envMap = { value: null };
    uniforms.uRipples = { value: ripples };
    if (sky && sky.fogParams) uniforms.hFogParams = { value: sky.fogParams };

    if (useDepthTex) {
      uniforms.uDepthRect.value.copy(depthRect);
      uniforms.uDepthDecode.value.copy(depthDecode);
      uniforms.uWaveHeight.value = lowSpec ? 0.04 : 0.085;
    }

    const defines = {
      RIPPLE_COUNT: RIPPLE_COUNT,
      SUN_DISC_RADIANCE: SUN_DISC_RADIANCE.toFixed(1),
      GLINT_ALPHA: GLINT_ALPHA.toFixed(6),
      GLINT_FOOTPRINT: GLINT_FOOTPRINT.toFixed(3),
    };
    if (lowSpec) defines.WATER_CHEAP = '';

    // The shore march is the valley body's and the apron's; the creek keeps the
    // environment map, as the work order asks. A creek is a metre wide in a
    // gully — there is nothing for a reflected ray to find but the bank it is
    // already touching.
    if (useDepthTex && shoreTex && shoreSteps > 0) {
      defines.WATER_SHORE_REFLECT = '';
      defines.SHORE_STEPS = String(shoreSteps);
      defines.SHORE_MAX_DIST = SHORE_MAX_DIST.toFixed(1);
      uniforms.uShoreTex = { value: shoreTex };
      uniforms.uShoreRect = { value: shoreRect.clone() };
      uniforms.uShoreCfg = { value: shoreCfg.clone() };
      uniforms.uShoreLight = { value: new THREE.Vector3(1, 1, 1) };
    }

    const mat = new THREE.ShaderMaterial({
      uniforms,
      vertexShader: SURFACE_VERT,
      fragmentShader: SURFACE_FRAG,
      defines,
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
      fog: true,
    });
    mat.name = useDepthTex ? 'water:valleyMat' : 'water:creekMat';
    return mat;
  }

  const creekMat = makeSurfaceMaterial(false);
  const valleyMat = valleyMesh ? makeSurfaceMaterial(true) : null;

  // The apron deliberately does NOT get the shore march either (it comes in on
  // the same useDepthTex flag). It begins 2.3 km offshore and runs to the far
  // plane; everything it can see is open water reflecting sky, its area is
  // enormous, and terrain.js's far ring lifts the valley floor above the water
  // line so most of it is occluded by dry land anyway. Paying a ten-tap march
  // over that many fragments to find nothing is the one place this feature
  // could actually cost something.
  //
  // The apron runs on the same shader as everything else, with the baked depth
  // texture switched OFF: that texture only covers the valley rect and its uv
  // clamp would hand every offshore fragment the rect-edge depth, which is
  // usually dry land — the sheet would discard itself. Its depth comes from the
  // constant vertex attribute instead, which is exactly right for open water.
  const apronMat = apronMesh ? (() => {
    const m = makeSurfaceMaterial(false);
    m.name = 'water:apronMat';
    const u = m.uniforms;
    // Match the valley body: same swell so the shared ring-0 vertices agree,
    // same Fresnel cap so the two do not read as different bodies of water.
    u.uWaveHeight.value = valleyMat ? valleyMat.uniforms.uWaveHeight.value : (lowSpec ? 0.04 : 0.085);
    u.uFresnelMax.value = 0.84;
    u.uEdgeFade.value = 0.55;
    u.uFoamWidth.value = 0.45;
    u.uRefract.value = 0.20;
    u.uBedScale.value = 0.05;
    // No wetted margin offshore — there is no shore out here, and the apron's
    // depth attribute is a constant 24 m so the band would evaluate against a
    // waterline it never reaches. Gain 0 switches it off; the width stays
    // non-zero because smoothstep is undefined when its two edges are equal.
    u.uWetMargin.value.set(0.05, 0.0);
    u.uDissolveRect.value.copy(apronRect);
    u.uDissolveBand.value.set(APRON_DISSOLVE, 1 / APRON_DISSOLVE);
    return m;
  })() : null;

  const fallMat = fallMesh ? (() => {
    const u = THREE.UniformsUtils.merge([
      THREE.UniformsLib.fog,
      {
        uTime: { value: 0 },
        uEnvIntensity: { value: 1 },
        uSunDir: { value: new THREE.Vector3(0, 1, 0) },
        uSunColor: { value: new THREE.Color(1, 1, 1) },
        uSkyLow: { value: skyLow.clone() },
        uSkyHigh: { value: skyHigh.clone() },
        uFoamColor: { value: foamColor.clone() },
        uScatter: { value: scatterColor.clone() },
        uOpacity: { value: 1 },
      },
    ]);
    u.uChopTex = { value: texChop };
    u.uFoamTex = { value: texFoam };
    u.envMap = { value: null };
    if (sky && sky.fogParams) u.hFogParams = { value: sky.fogParams };
    const m = new THREE.ShaderMaterial({
      uniforms: u,
      vertexShader: FALL_VERT,
      fragmentShader: FALL_FRAG,
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
      fog: true,
    });
    m.name = 'water:fallMat';
    return m;
  })() : null;

  const wetMat = wetMesh ? (() => {
    const u = THREE.UniformsUtils.merge([
      THREE.UniformsLib.fog,
      {
        uTime: { value: 0 },
        uSunDir: { value: new THREE.Vector3(0, 1, 0) },
        uSunColor: { value: new THREE.Color(1, 1, 1) },
        uSkyHigh: { value: skyHigh.clone() },
        uWetTint: { value: wetTint.clone() },
        // J1 asks for a wetted margin within ~0.5 m of the waterline. The band
        // already exists; it was just too faint to read against a creek that
        // was itself blown out. 0.55 -> 0.66.
        uOpacity: { value: 0.66 },
      },
    ]);
    u.uFoamTex = { value: texFoam };
    if (sky && sky.fogParams) u.hFogParams = { value: sky.fogParams };
    const m = new THREE.ShaderMaterial({
      uniforms: u,
      vertexShader: WET_VERT,
      fragmentShader: WET_FRAG,
      defines: {
        SUN_DISC_RADIANCE: SUN_DISC_RADIANCE.toFixed(1),
        // A water film on rock is glossy, not a mirror: alpha 0.10 (a2 = 0.01)
        // against the water surface's 0.0055. A few millimetres of water follows
        // the microrelief of what it is sitting on, so the lobe is the rock's,
        // not the film's. That matters here for a second reason: the wet decal
        // has no normal map on it, only the terrain normal, which varies slowly
        // — a mirror-tight lobe on a slowly varying normal would satisfy its
        // condition over a whole bank at once rather than at a point. Peaks at
        // display ~L244 at grazing incidence, which is the brightest thing on a
        // creek bank and still short of paper white.
        WET_GLINT_A2: '0.01',
      },
      transparent: true,
      premultipliedAlpha: true,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
      side: THREE.FrontSide,
      fog: true,
    });
    m.name = 'water:wetMat';
    return m;
  })() : null;

  // ---- assemble the meshes ----
  let creekMesh = null;
  if (surfaceIdx.length) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(surfacePos, 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute(surfaceNrm, 3));
    g.setAttribute('aDepth', new THREE.Float32BufferAttribute(surfaceDepth, 1));
    g.setAttribute('aFlow', new THREE.Float32BufferAttribute(surfaceFlow, 1));
    g.setAttribute('aFoam', new THREE.Float32BufferAttribute(surfaceFoam, 1));
    g.setAttribute('aShore', new THREE.Float32BufferAttribute(surfaceShore, 1));
    g.setAttribute('aDist', new THREE.Float32BufferAttribute(surfaceDist, 1));
    g.setAttribute('aFlowDir', new THREE.Float32BufferAttribute(surfaceFlowDir, 2));
    g.setAttribute('aBed', new THREE.Float32BufferAttribute(surfaceBed, 3));
    g.setIndex(surfaceIdx);
    g.computeBoundingSphere();
    creekMesh = new THREE.Mesh(g, creekMat);
    creekMesh.name = 'water:creek';
    creekMesh.renderOrder = 2;
    group.add(creekMesh);
  }
  if (valleyMesh && valleyMat) { valleyMesh.material = valleyMat; group.add(valleyMesh); }
  if (apronMesh && apronMat) { apronMesh.material = apronMat; group.add(apronMesh); }
  if (fallMesh && fallMat) { fallMesh.material = fallMat; group.add(fallMesh); }
  if (wetMesh && wetMat) { wetMesh.material = wetMat; group.add(wetMesh); }

  // =========================================================================
  // 4.10 Spray / mist / splash particles (pooled, zero per-frame allocation)
  // =========================================================================
  const density = typeof settings.particleDensity === 'number' ? settings.particleDensity : 1;
  const CAP = Math.round(clamp(760 * density, 110, 1500));
  const pPos = new Float32Array(CAP * 3);
  const pVel = new Float32Array(CAP * 3);
  const pParams = new Float32Array(CAP * 4);   // size, life01, kind, seed
  const pLife = new Float32Array(CAP);
  const pLifeMax = new Float32Array(CAP);
  let pCursor = 0;

  const sprayGeo = new THREE.BufferGeometry();
  const posAttr = new THREE.BufferAttribute(pPos, 3);
  const parAttr = new THREE.BufferAttribute(pParams, 4);
  posAttr.setUsage(THREE.DynamicDrawUsage);
  parAttr.setUsage(THREE.DynamicDrawUsage);
  sprayGeo.setAttribute('position', posAttr);
  sprayGeo.setAttribute('aParams', parAttr);
  sprayGeo.boundingSphere = new THREE.Sphere(
    new THREE.Vector3(0, (bounds.minY + bounds.maxY) * 0.5, 0),
    Math.max(bounds.maxX - bounds.minX, bounds.maxZ - bounds.minZ));

  const sprayUniforms = THREE.UniformsUtils.merge([
    THREE.UniformsLib.fog,
    {
      uTime: { value: 0 },
      uPixelScale: { value: 600 },
      uSunColor: { value: new THREE.Color(1, 1, 1) },
      uSkyHigh: { value: skyHigh.clone() },
      uFoamColor: { value: foamColor.clone() },
      uOpacity: { value: 1 },
    },
  ]);
  sprayUniforms.uPuff = { value: texPuff };
  if (sky && sky.fogParams) sprayUniforms.hFogParams = { value: sky.fogParams };

  const sprayMat = new THREE.ShaderMaterial({
    uniforms: sprayUniforms,
    vertexShader: SPRAY_VERT,
    fragmentShader: SPRAY_FRAG,
    transparent: true,
    depthWrite: false,
    fog: true,
  });
  sprayMat.name = 'water:sprayMat';
  const sprayPoints = new THREE.Points(sprayGeo, sprayMat);
  sprayPoints.name = 'water:spray';
  sprayPoints.frustumCulled = false;
  sprayPoints.renderOrder = 4;
  group.add(sprayPoints);

  function spawnParticle(x, y, z, vx, vy, vz, size, life, kind) {
    // Ring search from a rolling cursor; the pool is small and mostly dead
    // slots, so this terminates in a couple of steps in practice.
    for (let n = 0; n < CAP; n++) {
      const i = pCursor;
      pCursor = (pCursor + 1) % CAP;
      if (pLife[i] > 0) continue;
      pPos[i * 3] = x; pPos[i * 3 + 1] = y; pPos[i * 3 + 2] = z;
      pVel[i * 3] = vx; pVel[i * 3 + 1] = vy; pVel[i * 3 + 2] = vz;
      pLife[i] = life; pLifeMax[i] = life;
      pParams[i * 4] = size;
      pParams[i * 4 + 1] = 1;
      pParams[i * 4 + 2] = kind;
      pParams[i * 4 + 3] = Math.random();
      return true;
    }
    return false;
  }

  // =========================================================================
  // 4.11 Spatial index over the creek, for heightAt / depthAt / flowAt
  // =========================================================================
  const GRID = 18;
  let gMinX = 0, gMinZ = 0, gNX = 1, gNZ = 1;
  let cellStart = new Int32Array(2);
  let cellItems = new Int32Array(0);

  if (hasCreek) {
    let mnX = Infinity, mxX = -Infinity, mnZ = Infinity, mxZ = -Infinity;
    for (let i = 0; i < NS; i++) {
      if (stX[i] < mnX) mnX = stX[i]; if (stX[i] > mxX) mxX = stX[i];
      if (stZ[i] < mnZ) mnZ = stZ[i]; if (stZ[i] > mxZ) mxZ = stZ[i];
    }
    gMinX = mnX - GRID; gMinZ = mnZ - GRID;
    gNX = Math.max(1, Math.ceil((mxX - mnX + GRID * 2) / GRID));
    gNZ = Math.max(1, Math.ceil((mxZ - mnZ + GRID * 2) / GRID));
    const counts = new Int32Array(gNX * gNZ + 1);
    // A station influences every cell its half-width can reach.
    const cellsFor = (i, fn) => {
      const r = sHalf[i] * RIBBON_OVERHANG + 1.5;
      const i0 = Math.max(0, Math.floor((stX[i] - r - gMinX) / GRID));
      const i1 = Math.min(gNX - 1, Math.floor((stX[i] + r - gMinX) / GRID));
      const j0 = Math.max(0, Math.floor((stZ[i] - r - gMinZ) / GRID));
      const j1 = Math.min(gNZ - 1, Math.floor((stZ[i] + r - gMinZ) / GRID));
      for (let j = j0; j <= j1; j++) for (let k = i0; k <= i1; k++) fn(j * gNX + k);
    };
    for (let i = 0; i < NS; i++) cellsFor(i, (c) => { counts[c + 1]++; });
    for (let c = 0; c < gNX * gNZ; c++) counts[c + 1] += counts[c];
    cellStart = counts;
    cellItems = new Int32Array(counts[gNX * gNZ]);
    const fill = new Int32Array(gNX * gNZ);
    for (let i = 0; i < NS; i++) cellsFor(i, (c) => { cellItems[cellStart[c] + fill[c]++] = i; });
  }

  // Query scratch — module-level so the hot path allocates nothing.
  let _qFound = false, _qSurf = 0, _qDepth = 0, _qFlow = 0, _qFdx = 0, _qFdz = 0;

  function queryCreek(x, z) {
    _qFound = false;
    if (!hasCreek) return false;
    const ci = Math.floor((x - gMinX) / GRID);
    const cj = Math.floor((z - gMinZ) / GRID);
    if (ci < 0 || cj < 0 || ci >= gNX || cj >= gNZ) return false;
    const c = cj * gNX + ci;
    const s0 = cellStart[c], s1 = cellStart[c + 1];
    let bestD2 = Infinity, best = -1;
    for (let k = s0; k < s1; k++) {
      const i = cellItems[k];
      const dx = x - stX[i], dz = z - stZ[i];
      const d2 = dx * dx + dz * dz;
      if (d2 < bestD2) { bestD2 = d2; best = i; }
    }
    if (best < 0) return false;
    // Lateral offset from the centreline at the nearest station.
    const dx = x - stX[best], dz = z - stZ[best];
    const lat = Math.abs(-sDirZ[best] * dx + sDirX[best] * dz);
    if (lat > sHalf[best] * 1.05) return false;
    const bed = terrain.sampleHeight(x, z);
    if (sSurf[best] <= bed) return false;
    _qFound = true;
    _qSurf = sSurf[best];
    _qDepth = sSurf[best] - bed;
    _qFlow = sFlow[best];
    _qFdx = sDirX[best]; _qFdz = sDirZ[best];
    return true;
  }

  function heightAt(x, z) {
    if (queryCreek(x, z)) return _qSurf;
    if (valleyMesh && terrain.sampleHeight(x, z) < waterLevel) return waterLevel;
    return null;
  }

  function depthAt(x, z) {
    if (queryCreek(x, z)) return _qDepth;
    if (valleyMesh) {
      const d = waterLevel - terrain.sampleHeight(x, z);
      if (d > 0) return d;
    }
    return 0;
  }

  function flowAt(x, z, out) {
    const o = out || _v3;
    if (queryCreek(x, z)) o.set(_qFdx * _qFlow, 0, _qFdz * _qFlow);
    else o.set(0, 0, 0);
    return o;
  }

  function isSubmerged(x, y, z) {
    const h = heightAt(x, z);
    return h !== null && y < h;
  }

  // =========================================================================
  // 4.12 Events
  // =========================================================================
  let rippleCursor = 0;
  const rippleAge = new Float32Array(RIPPLE_COUNT);
  const rippleLife = new Float32Array(RIPPLE_COUNT);
  const rippleStr = new Float32Array(RIPPLE_COUNT);

  function spawnRipple(x, z, strength) {
    const i = rippleCursor;
    rippleCursor = (rippleCursor + 1) % RIPPLE_COUNT;
    const s = clamp(strength, 0.05, 1.4);
    ripples[i].set(x, z, 0.05, s);
    rippleAge[i] = 0;
    rippleStr[i] = s;
    rippleLife[i] = 1.1 + s * 0.9;
  }

  function splash(payload) {
    if (!payload || !payload.position) return;
    const p = payload.position;
    let sp = 4;
    if (typeof payload.velocity === 'number') sp = Math.abs(payload.velocity);
    else if (payload.velocity && typeof payload.velocity.length === 'function') sp = payload.velocity.length();
    sp = clamp(sp, 0.5, 22);

    const wy = heightAt(p.x, p.z);
    const y = wy === null ? p.y : wy;
    spawnRipple(p.x, p.z, clamp(sp / 12, 0.12, 1.3));

    const n = Math.round(clamp(sp * 2.2, 3, 34) * density);
    for (let k = 0; k < n; k++) {
      const a = Math.random() * Math.PI * 2;
      const r = Math.random();
      const sped = sp * (0.18 + 0.55 * Math.random());
      spawnParticle(
        p.x + Math.cos(a) * r * 0.35, y + 0.05, p.z + Math.sin(a) * r * 0.35,
        Math.cos(a) * sped * 0.55, sped * (0.55 + Math.random() * 0.7), Math.sin(a) * sped * 0.55,
        0.055 + Math.random() * 0.11, 0.35 + Math.random() * 0.55, 0);
    }
    // A short-lived low mist over the impact.
    const m = Math.round(clamp(sp * 0.7, 1, 10) * density);
    for (let k = 0; k < m; k++) {
      const a = Math.random() * Math.PI * 2;
      spawnParticle(
        p.x + Math.cos(a) * 0.3, y + 0.15, p.z + Math.sin(a) * 0.3,
        Math.cos(a) * 0.5, 0.6 + Math.random() * 0.7, Math.sin(a) * 0.5,
        0.32 + Math.random() * 0.5, 0.8 + Math.random() * 0.8, 1);
    }
  }

  const offSplash = ctx.events ? ctx.events.on('water:splash', splash) : null;

  /**
   * Wired for the day terrain grows a wetness setter (see the CONTRACT-NOTE at
   * the top of the file). Currently a no-op — the decal ribbon does the work.
   */
  function applyTerrainWetness() {
    if (!terrain || typeof terrain.setWetness !== 'function' || !hasCreek) return;
    for (let i = 0; i < NS; i++) {
      terrain.setWetness(stX[i], stZ[i], sHalf[i] * 3.2 + 1.5, clamp01(0.55 + sFoam[i] * 0.45));
    }
    for (const e of fallEmitters) {
      terrain.setWetness(e.x, e.z, e.radius * 3.4 + e.power * 3.0, 0.95);
    }
  }
  applyTerrainWetness();

  // =========================================================================
  // 4.13 Frame update
  // =========================================================================
  let time = 0;
  let splashTimer = 0;
  let emitAccum = 0;
  let sprayWasAlive = false;
  const camPos = new THREE.Vector3();
  let lastEnv = null;
  // Reused every frame — emitting a fresh object would allocate in update().
  const splashPayload = { position: _splashPos, velocity: 0 };

  const materials = [creekMat, valleyMat, apronMat, fallMat].filter(Boolean);

  function refreshLighting() {
    const sun = ctx.sun;
    const skyMod = ctx.sky;
    // uSunColor is uploaded as *reflected radiance for an albedo-1 Lambertian
    // surface facing the sun* — i.e. irradiance / PI. Skipping the 1/PI is the
    // classic way to end up three times over-exposed against a scene whose
    // MeshStandardMaterials all divide by it.
    if (sun) {
      _sunCol.copy(sun.color).multiplyScalar((sun.intensity || 1) / Math.PI);
    } else {
      _sunCol.setRGB(1.0, 0.92, 0.80, THREE.SRGBColorSpace).multiplyScalar(3 / Math.PI);
    }
    if (skyMod && skyMod.sunDirection) _v1.copy(skyMod.sunDirection);
    else if (sun) _v1.copy(sun.position).normalize();
    else _v1.set(0.4, 0.55, -0.73).normalize();

    const envTex = (ctx.scene && ctx.scene.environment) || (skyMod && skyMod.environment) || null;
    const envInt = (skyMod && typeof skyMod.environmentIntensity === 'number')
      ? skyMod.environmentIntensity
      : (ctx.scene && typeof ctx.scene.environmentIntensity === 'number' ? ctx.scene.environmentIntensity : 1);

    if (skyMod && skyMod.fogGroundColor) skyLow.copy(skyMod.fogGroundColor);
    if (skyMod && skyMod.fogHighColor) skyHigh.copy(skyMod.fogHighColor);

    // Relight for the shore reflection proxy. The bake stores albedo times a
    // purely geometric shade term, so one multiply here keeps the reflected
    // shore in step with the live sun and sky instead of freezing at the
    // time of day the world happened to boot at.
    shoreLight.setRGB(
      _sunCol.r * 0.90 + skyHigh.r * 0.60,
      _sunCol.g * 0.90 + skyHigh.g * 0.60,
      _sunCol.b * 0.90 + skyHigh.b * 0.60);

    for (const m of materials) {
      const u = m.uniforms;
      u.uSunDir.value.copy(_v1);
      u.uSunColor.value.copy(_sunCol);
      u.uSkyLow.value.copy(skyLow);
      u.uSkyHigh.value.copy(skyHigh);
      u.uEnvIntensity.value = envInt;
      if (u.uShoreLight) u.uShoreLight.value.set(shoreLight.r, shoreLight.g, shoreLight.b);
      if (envTex !== lastEnv) {
        u.envMap.value = envTex;
        // Setting material.envMap is what makes three emit USE_ENVMAP and the
        // CUBEUV_* defines this shader's textureCubeUV() call needs. The sky
        // only rebuilds its PMREM when the sun actually moves, so the program
        // rebuild this can trigger is rare and not per-frame.
        m.envMap = envTex;
        m.needsUpdate = true;
      }
    }
    if (wetMat) {
      wetMat.uniforms.uSunDir.value.copy(_v1);
      wetMat.uniforms.uSunColor.value.copy(_sunCol);
      wetMat.uniforms.uSkyHigh.value.copy(skyHigh);
    }
    sprayUniforms.uSunColor.value.copy(_sunCol);
    sprayUniforms.uSkyHigh.value.copy(skyHigh);
    lastEnv = envTex;
  }

  refreshLighting();

  function update(dt) {
    if (!(dt > 0)) dt = 0;
    time += dt;

    for (const m of materials) m.uniforms.uTime.value = time;
    if (wetMat) wetMat.uniforms.uTime.value = time;
    sprayUniforms.uTime.value = time;

    // Lighting drifts slowly; refreshing every 4th frame is invisible and
    // keeps the per-frame uniform churn down.
    if ((ctx.frame & 3) === 0) refreshLighting();

    if (ctx.camera) camPos.copy(ctx.camera.position);

    // ---- ripples ---------------------------------------------------------
    for (let i = 0; i < RIPPLE_COUNT; i++) {
      if (ripples[i].w <= 0) continue;
      rippleAge[i] += dt;
      const t = rippleAge[i] / rippleLife[i];
      if (t >= 1) { ripples[i].w = 0; continue; }
      // Gravity waves slow as they spread; amplitude falls off with radius.
      ripples[i].z += dt * (2.6 * (1 - t * 0.65));
      ripples[i].w = rippleStr[i] * (1 - t) * (1 - t);
    }

    // ---- waterfall mist emission -----------------------------------------
    if (fallEmitters.length && density > 0) {
      emitAccum += dt;
      const tick = 1 / 60;
      let guard = 0;
      while (emitAccum >= tick && guard++ < 4) {
        emitAccum -= tick;
        for (let e = 0; e < fallEmitters.length; e++) {
          const em = fallEmitters[e];
          const dx = camPos.x - em.x, dz = camPos.z - em.z;
          const dist2 = dx * dx + dz * dz;
          if (dist2 > 240 * 240) continue;
          const near = clamp01(1 - Math.sqrt(dist2) / 240);
          const rate = em.power * (0.30 + near * 0.85) * density;
          if (Math.random() > rate * 0.30) continue;
          const a = Math.random() * Math.PI * 2;
          const r = Math.sqrt(Math.random()) * em.radius;
          // Mist billows up and drifts downstream.
          spawnParticle(
            em.x + Math.cos(a) * r, em.y + 0.15 + Math.random() * 0.4, em.z + Math.sin(a) * r,
            Math.cos(a) * 0.55 + Math.sin(time * 0.31) * 0.35,
            0.60 + Math.random() * 0.85 * em.power,
            Math.sin(a) * 0.55 + Math.cos(time * 0.23) * 0.35,
            0.32 + Math.random() * 0.48 * em.power,
            1.3 + Math.random() * 1.5, 1);
          // A few heavy droplets kicked out of the plunge pool.
          if (Math.random() < 0.28 * em.power) {
            spawnParticle(
              em.x + Math.cos(a) * r * 0.6, em.y + 0.1, em.z + Math.sin(a) * r * 0.6,
              Math.cos(a) * (1.2 + Math.random() * 2.2),
              1.6 + Math.random() * 2.6,
              Math.sin(a) * (1.2 + Math.random() * 2.2),
              0.05 + Math.random() * 0.09, 0.5 + Math.random() * 0.5, 0);
          }
        }
      }
    }

    // ---- bike in the water -----------------------------------------------
    splashTimer -= dt;
    const bike = ctx.bike;
    const st = bike && bike.state;
    if (st && st.position && ctx.events) {
      const bx = st.position.x, by = st.position.y, bz = st.position.z;
      const wy = heightAt(bx, bz);
      if (wy !== null && by - wy < 0.55 && by - wy > -1.2) {
        const spd = typeof st.speed === 'number' ? st.speed : 0;
        if (spd > 1.5 && splashTimer <= 0) {
          splashTimer = 0.055;
          _splashPos.set(bx, wy, bz);
          splashPayload.velocity = spd;
          ctx.events.emit('water:splash', splashPayload);
        }
      }
    }

    // ---- particle integration --------------------------------------------
    let anyAlive = false;
    for (let i = 0; i < CAP; i++) {
      if (pLife[i] <= 0) { if (pParams[i * 4 + 1] !== 0) pParams[i * 4 + 1] = 0; continue; }
      pLife[i] -= dt;
      if (pLife[i] <= 0) { pLife[i] = 0; pParams[i * 4 + 1] = 0; continue; }
      anyAlive = true;
      const k3 = i * 3;
      const mist = pParams[i * 4 + 2] > 0.5;
      if (mist) {
        // Mist is buoyant, heavily damped, and pushed around by the breeze.
        pVel[k3 + 1] += (0.55 - pVel[k3 + 1] * 0.9) * dt;
        const drag = Math.exp(-1.6 * dt);
        pVel[k3] = pVel[k3] * drag + Math.sin(time * 0.7 + i) * 0.25 * dt;
        pVel[k3 + 2] = pVel[k3 + 2] * drag + Math.cos(time * 0.53 + i) * 0.25 * dt;
        pVel[k3 + 1] *= drag;
      } else {
        pVel[k3 + 1] -= 9.81 * dt;
        const drag = Math.exp(-0.9 * dt);
        pVel[k3] *= drag; pVel[k3 + 2] *= drag;
      }
      pPos[k3] += pVel[k3] * dt;
      pPos[k3 + 1] += pVel[k3 + 1] * dt;
      pPos[k3 + 2] += pVel[k3 + 2] * dt;
      pParams[i * 4 + 1] = pLife[i] / pLifeMax[i];
    }
    // Only pay for the upload while something is actually on screen.
    if (anyAlive || sprayWasAlive) {
      posAttr.needsUpdate = true;
      parAttr.needsUpdate = true;
    }
    sprayWasAlive = anyAlive;
    sprayPoints.visible = anyAlive;

    if (ctx.renderer) {
      const h = ctx.renderer.domElement ? ctx.renderer.domElement.height : 1080;
      sprayUniforms.uPixelScale.value = h * 0.55;
    }
  }

  // =========================================================================
  // 4.14 Public API
  // =========================================================================
  const creekPath = new Float32Array(hasCreek ? NS * 3 : 0);
  if (hasCreek) {
    for (let i = 0; i < NS; i++) {
      creekPath[i * 3] = stX[i];
      creekPath[i * 3 + 1] = sSurf[i];
      creekPath[i * 3 + 2] = stZ[i];
    }
  }

  const surfaces = [];
  if (creekMesh) surfaces.push(creekMesh);
  if (valleyMesh) surfaces.push(valleyMesh);
  if (apronMesh) surfaces.push(apronMesh);
  if (fallMesh) surfaces.push(fallMesh);

  const api = {
    group,
    surfaces,
    waterLevel,
    creekPath,
    falls: fallEmitters,

    heightAt,
    depthAt,
    flowAt,
    isSubmerged,

    update,

    resize() { /* nothing view-dependent; point size is derived per frame */ },

    setQuality(q) {
      const low = q === 'low';
      const wh = low ? 0.04 : 0.085;
      if (valleyMat) valleyMat.uniforms.uWaveHeight.value = wh;
      // Must track the valley body or the shared ring-0 vertices stop agreeing.
      if (apronMat) apronMat.uniforms.uWaveHeight.value = wh;
      if (wetMesh) wetMesh.visible = !low;
      sprayMat.uniforms.uOpacity.value = low ? 0.7 : 1.0;
    },

    dispose() {
      if (offSplash) offSplash();
      if (scene) scene.remove(group);
      for (const m of [creekMesh, valleyMesh, apronMesh, fallMesh, wetMesh, sprayPoints]) {
        if (m && m.geometry) m.geometry.dispose();
      }
      for (const m of [creekMat, valleyMat, apronMat, fallMat, wetMat, sprayMat]) {
        if (m) m.dispose();
      }
      texSwell.dispose(); texChop.dispose(); texFoam.dispose();
      texBed.dispose(); texPuff.dispose(); dummyDepth.dispose();
      if (depthTex) depthTex.dispose();
      if (shoreTex) shoreTex.dispose();
    },

    // --- debug / integration aids ---
    get stationCount() { return hasCreek ? NS : 0; },
    get drawCalls() { return surfaces.length + (wetMesh ? 1 : 0) + 1; },
  };

  if (ctx.events) ctx.events.on('quality:changed', (q) => api.setQuality(typeof q === 'string' ? q : (q && q.quality)));

  return api;
}

export default createWater;
