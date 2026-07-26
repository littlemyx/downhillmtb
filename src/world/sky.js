// sky.js — atmosphere, cloudscape, fog and ALL scene lighting for DESCENT.
//
// Owns (CONTRACT §6):
//   * a tuned Preetham sky dome (derived from three/examples/jsm/objects/Sky.js)
//     with a limb-darkened solar disc and a night-side star field,
//   * a raymarched volumetric cloud layer (procedural 3D noise, dual-lobe HG
//     scattering, powder term, multi-scatter octaves, aerial perspective),
//   * the sun (DirectionalLight) plus a cool sky fill, so shadows read blue and
//     not black — shadow-side fill is what makes an outdoor render look real,
//   * one tight, texel-snapped shadow cascade that follows the camera, fitted to
//     an authored metres-per-texel budget rather than to the view frustum,
//   * a PMREM environment map over the sky driving scene.environment (IBL),
//   * height-based exponential fog with aerial perspective, installed globally
//     by overriding three's fog ShaderChunks,
//   * drifting cloud shadows on the ground, riding the same chunk override.
//
// ---------------------------------------------------------------------------
// CONTRACT-NOTE (fog): §6 asks for height-based fog "via a fog shader chunk
// override so it is height-aware". Three's built-in materials clone their
// uniforms out of `ShaderLib` the first time a program is built, so a normal
// custom uniform cannot be shared between materials. The mechanism used here: a
// uniform whose value is a *Float32Array* survives `UniformsUtils.clone()` by
// reference (see cloneUniforms — a typed array is neither a three object nor
// `Array.isArray`), so a single `hFogParams` array is genuinely shared by every
// material in the scene and updating it here updates the fog everywhere for
// free. Modules that build their own ShaderMaterial with `fog: true` should
// merge `THREE.UniformsLib.fog` into their uniforms (standard practice) and
// will pick the height fog up automatically; if they do not, the uniform reads
// back as zeroes, the chunk detects that and falls back to three's stock
// FogExp2 rather than breaking their material.
//
// CONTRACT-NOTE (cloud shadows): R3-E4 puts a per-pixel cloud-shadow multiply
// into `fog_fragment`, immediately before the haze mix, so it lands on the
// surface colour and not on the air in front of it. It rides the same shared
// `hFogParams` Float32Array as the fog, for the reason documented above: that is
// the only uniform in this project that survives `UniformsUtils.clone()` by
// reference and therefore reaches every lit material without patching modules
// this one does not own. A texture cannot travel that way (cloneUniforms clones
// textures, and refuses render-target textures outright), which is why the field
// is four analytic plane waves rather than a sample of the weather map. Anything
// that wants to opt out sets `material.fog = false`; the sky dome already does.
// If you are reading this because your material got darker in patches: that is
// the cloud shadow, it is deliberate, and `sky.cloudShadow` reports its value
// under the camera. `ctx.sun.intensity` is NOT dimmed by cloud any more — it was,
// globally, which is an exposure wobble rather than a shadow.
//
// CONTRACT-NOTE (sky range, R5-E2): §6 asks for a physically-plausible
// Rayleigh/Mie sky, and this is one — but Preetham is a *radiance* model with no
// notion of the key light it has to share a photograph with, and its dynamic
// range is far wider than an ACES pipeline at exposure 1.0 can carry. Measured
// against a fully sunlit albedo-1.0 card under this module's own key
// (3.51 / PI = 1.117 linear), the raw model puts the zenith at 0.18x that card —
// correct — the 5-degree horizon ring at 1.07x, and the sky 20 degrees from the
// sun at **27x**. There is no single exposure that holds both ends, which is why
// the r5 review set clips 7–38% of its pixels in twelve of sixteen shots and
// measures a sky saturation of 0.035 over a model whose hue was right all along.
// So the dome's top end is compressed at source, by a hue-preserving power
// shoulder above a knee that sits just over the zenith (skyShoulder() in the
// fragment shader, SKY_SHOULDER_KNEE/_POWER on the CPU). The model is not
// rewritten and its spectrum is not touched; only its highlights are brought
// into the same photograph as the sun. The solar disc and aureole are exempt so
// bloom still has a real source. See applySunAngles for the measured before/after.
//
// CONTRACT-NOTE (postfx): `sky.sunDirection` is a unit Vector3 in world space
// pointing *towards* the sun — that is the god-ray / lens-flare key, and
// postfx.js already reads it. The sky shader draws a real limb-darkened solar
// disc plus a tight aureole so there is genuine structure for bloom and light
// shafts to key off, rather than one blown-out texel.
//
// CONTRACT-NOTE (shadows): §9 exposes `settings.cascades`, but true CSM in
// three requires patching every receiving material's `onBeforeCompile`, which
// would stomp on terrainMaterial.js / vegetation.js / bikeModel.js. Per this
// module's brief the hand-rolled equivalent is used instead: a single shadow
// map, recomputed every frame and snapped to texel increments so shadows do not
// swim. `sky.shadowDistance` reports (and can override) the reach ahead of the
// camera; `sky.shadowTexel` reports metres per texel, which is the number that
// actually decides what can cast. Beyond the reach the terrain is carried by N·L
// shading and aerial perspective, which is what the fit is tuned against.
//
// CONTRACT-NOTE (fit): R3-E1 changed what the fit is derived FROM. It used to be
// the smallest sphere enclosing a [near, 150 m] slice of the view frustum, which
// at 2048 gave a 0.18 m texel — larger than a 30 mm frame tube, a 45 mm tape post
// or a 60 mm forearm, so the hero of the game was physically incapable of casting
// a contact shadow and neither was anything else at human scale. Round 3 found no
// legible contact shadow on any object in sixteen frames. The derivation is now
// inverted: metres-per-texel is authored per tier (SHADOW_TEXEL) and the covered
// radius falls out of it as 0.5 * texel * mapSize. At high that is 0.035 m over a
// 36 m disc centred ~20 m ahead of the eye. It trades 2.6x of reach for 5.1x of
// sharpness, and the reach it gives up was carrying nothing.
//
// CONTRACT-NOTE (second cascade): the r2 work order asks for a coarse second
// cascade at 400–1500 m so a 720 m mountain can self-shadow in an aerial wide;
// the r3 work order asks for a *tight* second slice fitted to the bike+rider box.
// Neither is implemented, for one reason. The only way to add a second shadow map
// in stock three is a second DirectionalLight, and a second light must *split*
// the key or the scene is lit twice. `ctx.sun.intensity` is read as "the key" by
// water.js:2449, vegetation.js:4152 and particles.js for their own hand-rolled
// sun terms, so splitting it would silently under-light the water glint, the
// entire forest and every particle by the far light's share — and the trees would
// then no longer match the ground they stand on.
// The one arrangement that adds a shadow without adding light is a subtractive
// triple: +K tight-shadowed, +K wide-shadowed and -K unshadowed, summing to
// K*(shadowWide + shadowTight - 1). That was costed and rejected: inside *both*
// shadows the expression is -1, so it subtracts the whole key from the ambient
// term and crushes to black, and "the rider crosses tree shade" makes that the
// common case — it would re-open the crushed-shadow-floor P0 that round 2 fixed.
// Implemented instead: (a) the texel-budget fit above, which is where hero-scale
// contact shadows actually come from, and (b) the single cascade's *radius*
// adapts to the camera's height above the terrain. In gameplay (eye 2–6 m off the
// deck) it stays at the tight fit; lift the camera into an aerial framing and it
// grows toward a 620 m radius / ~0.6 m texel, arriving only when nothing in frame
// is close enough to want a contact shadow. Costs one extra `terrain.sampleHeight`
// per frame and no extra draw. A genuine CSM belongs with a renderer-wide
// material patch, which is a cross-module change.
//
// CONTRACT-NOTE (background): engine.js installs a fallback `scene.background`
// texture and a fallback `scene.environment`. This module deliberately leaves
// the background alone (the dome draws over it and costs nothing extra) but
// takes ownership of `scene.environment`, replacing it with the PMREM of the
// real sky. The engine's fallback texture is not disposed — it is not ours.
// ---------------------------------------------------------------------------
//
// Everything is procedural: the cloud volumes and weather map are generated in
// code from the shared seeded PRNG. No files, no network.

import * as THREE from 'three';
import { Sky as ThreeSky } from 'three/examples/jsm/objects/Sky.js';
import { makeRng, subSeed, clamp, clamp01, lerp, smoothstep, damp } from '../core/rng.js';

// ===========================================================================
// Module-scope scratch. Nothing in update() is allowed to allocate.
// ===========================================================================
const _v1 = new THREE.Vector3();
const _camFwd = new THREE.Vector3();
const _center = new THREE.Vector3();
const _lightRight = new THREE.Vector3();
const _lightUp = new THREE.Vector3();
const _worldUp = new THREE.Vector3(0, 1, 0);
const _colA = new THREE.Color();
const _colB = new THREE.Color();
const _colC = new THREE.Color();
const _skyIrr = new THREE.Color();      // sky irradiance, RGB, in dome units
const _bounce = new THREE.Color();      // sunlit-ground bounce radiance, dome units
const _hazeCol = new THREE.Color();     // grazing-incidence haze, dome units

/** Rec.709 relative luminance of a linear colour. */
function lum(c) { return 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b; }

const DEG = Math.PI / 180;
const TAU = Math.PI * 2;

// ===========================================================================
// 1. Global fog chunk override — height fog + aerial perspective.
// ===========================================================================
//
// Installed at module scope so it lands before *any* material in the project is
// compiled. main.js imports terrain/trail before sky, but nothing renders until
// the whole boot sequence has finished, and three resolves ShaderChunk includes
// (and clones ShaderLib uniforms) lazily at first program build.

const HFOG = 'hFogParams';
const HFOG_VEC4S = 8;

/** The single shared backing store. Layout is documented in the GLSL below. */
const fogParams = new Float32Array(HFOG_VEC4S * 4);

let fogChunksInstalled = false;

function installFogChunks() {
  if (fogChunksInstalled) return;
  fogChunksInstalled = true;

  // --- vertex ------------------------------------------------------------
  THREE.ShaderChunk.fog_pars_vertex = /* glsl */`
#ifdef USE_FOG
	varying float vFogDepth;
	varying vec3 vFogWorldPosition;
#endif
`;

  // `mvPosition` is in scope in every stock shader that includes this chunk
  // (project_vertex defines it; the sprite shader defines its own). A camera's
  // view matrix has an orthonormal upper 3x3, so `v * mat3( viewMatrix )` is
  // transpose(R) * v == R^-1 * v — the world position with no matrix inverse
  // and no dependency on `transformed`, which some shaders do not define.
  THREE.ShaderChunk.fog_vertex = /* glsl */`
#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
	vFogWorldPosition = cameraPosition + ( mvPosition.xyz * mat3( viewMatrix ) );
#endif
`;

  // --- fragment ----------------------------------------------------------
  THREE.ShaderChunk.fog_pars_fragment = /* glsl */`
#ifdef USE_FOG

	uniform vec3 fogColor;
	varying float vFogDepth;
	varying vec3 vFogWorldPosition;

	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif

	// Height fog / aerial perspective, owned by src/world/sky.js.
	//   [0].rgb  ground-level haze colour (linear)   [0].a  density at ref height
	//   [1].rgb  high-altitude / zenith haze colour  [1].a  1 / scale height
	//   [2].rgb  solar in-scatter tint (linear)      [2].a  HG anisotropy g
	//   [3].xyz  unit direction towards the sun      [3].a  reference height (m)
	//   [4].x    max opacity     [4].y  start distance (m)
	//   [4].z    horizon blend   [4].w  in-scatter gain
	//   [5].x    enabled (0 => fall back to three's stock fog)
	//   [5].y    1 when the frame is written sRGB-encoded, 0 when linear
	//   [5].z    1 when the frame was already tone mapped, 0 when still HDR
	//   [5].w    renderer.toneMappingExposure
	//   [6].rgb  haze colour looking *down*   [6].a  down-band rate (R3-E3)
	//   [7].x    cloud-shadow depth (0 = off) [7].y  cloud-shadow scale (1/m)
	//   [7].z    cloud-shadow drift x (m)     [7].w  cloud-shadow drift z (m)
	// A material that never received this uniform reads all zeroes, which
	// disables the block — so this override can never black out a frame.
	uniform vec4 ${HFOG}[ ${HFOG_VEC4S} ];

	float hfog_hg( float c, float g ) {
		float g2 = g * g;
		float d = max( 1.0 + g2 - 2.0 * g * c, 1.0e-4 );
		return ( 1.0 - g2 ) / ( 12.566370614 * d * sqrt( d ) );
	}

	// R3-E4 — drifting cloud shadows on the ground. Their absence is a large part
	// of why the massif reads small: with one unbroken key there is no cue for how
	// big a patch of weather is, and therefore none for how big the mountain is.
	// Four incommensurate plane waves evaluated where the sun ray through this
	// fragment crosses the deck, then thresholded. Deliberately *not* the marched
	// volume: the cloud that shadows you is behind and above the camera and is
	// almost never one you can see, so agreement is unobservable, while sampling a
	// texture is not possible here — this chunk reaches every fogged material in
	// the project through a shared Float32Array, and a texture cannot travel that
	// way (see the CONTRACT-NOTE at the head of sky.js). Measured over a 600x600
	// grid: 60% of the ground fully sunlit, 8% fully shaded, mean occlusion 0.22,
	// features 590–1200 m across. Cost is one divide and four sines.
	float hfog_cloudShade( vec3 wpos ) {
		float depth = ${HFOG}[ 7 ].x;
		vec3 sd = ${HFOG}[ 3 ].xyz;
		if ( depth <= 0.0 || sd.y < 0.06 ) return 1.0;
		vec2 p = ( wpos.xz + sd.xz * ( 900.0 / sd.y ) + ${HFOG}[ 7 ].zw ) * ${HFOG}[ 7 ].y;
		float m = sin( p.x * 1.000 + p.y * 0.317 )
			+ sin( p.x * -0.593 + p.y * 0.812 ) * 0.86
			+ sin( p.x * 0.231 + p.y * -1.371 ) * 0.63
			+ sin( p.x * 1.717 + p.y * 1.093 ) * 0.41;
		return 1.0 - depth * smoothstep( 0.02, 0.62, m * 0.3448 );
	}

	// This chunk runs *after* <tonemapping_fragment> and <colorspace_fragment>,
	// so the haze has to be delivered in whatever state the target expects. The
	// fog colours are authored as scene-referred linear radiance (they are
	// sampled straight out of the same atmosphere model the sky dome uses), so
	// on the direct-to-framebuffer path they must be tone mapped and encoded
	// here; on the post-processing path they stay linear and the composer does
	// it later. sky.js reports which case applies, per frame.
	vec3 hfog_present( vec3 c ) {
		c = max( c, vec3( 0.0 ) );
		vec3 x = c * ${HFOG}[ 5 ].w;
		vec3 t = clamp( ( x * ( 2.51 * x + 0.03 ) ) / ( x * ( 2.43 * x + 0.59 ) + 0.14 ), 0.0, 1.0 );
		c = mix( c, t, ${HFOG}[ 5 ].z );
		vec3 s = mix(
			c * 12.92,
			1.055 * pow( max( c, vec3( 0.0031308 ) ), vec3( 0.41666667 ) ) - 0.055,
			step( vec3( 0.0031308 ), c )
		);
		return mix( c, s, ${HFOG}[ 5 ].y );
	}

#endif
`;

  THREE.ShaderChunk.fog_fragment = /* glsl */`
#ifdef USE_FOG

	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif

	vec3 hfogFinal = fogColor;

	if ( ${HFOG}[ 5 ].x > 0.5 ) {

		vec3 hfogSeg = vFogWorldPosition - cameraPosition;
		float hfogDist = length( hfogSeg );
		vec3 hfogDir = hfogSeg / max( hfogDist, 1.0e-3 );

		float hfogK = ${HFOG}[ 1 ].w;
		float hfogRel = clamp( ( cameraPosition.y - ${HFOG}[ 3 ].w ) * hfogK, -6.0, 24.0 );
		float hfogRise = clamp( hfogDir.y * hfogK * hfogDist, -16.0, 16.0 );

		// Analytic integral of D * exp( -k * ( y - y0 ) ) along the view ray.
		// ( 1 - e^-x ) / x tends to 1 as x -> 0, so the near-level case is guarded.
		float hfogRatio = abs( hfogRise ) < 1.0e-3 ? 1.0 : ( 1.0 - exp( - hfogRise ) ) / hfogRise;
		float hfogPath = max( hfogDist - ${HFOG}[ 4 ].y, 0.0 );
		float hfogOd = clamp( ${HFOG}[ 0 ].w * exp( - hfogRel ) * hfogRatio * hfogPath, 0.0, 40.0 );

		fogFactor = ( 1.0 - exp( - hfogOd ) ) * ${HFOG}[ 4 ].x;

		// Distant geometry has to *become* the sky it stands in front of, so the
		// haze colour runs from the ground tint up to the zenith tint with view
		// elevation, and then gains the forward-scattered solar lobe that makes
		// hazy air glow around the sun.
		float hfogUp = clamp( hfogDir.y * ${HFOG}[ 4 ].z + 0.10, 0.0, 1.0 );
		vec3 hfogBase = mix( ${HFOG}[ 0 ].rgb, ${HFOG}[ 1 ].rgb, hfogUp * hfogUp );

		// R3-E3 — looking *down* is not looking at the sky. Every downward ray used
		// to clamp onto the 19-degree upward ring average, which is the airlight of
		// an infinite path through air lit by a full hemisphere of sky. The air
		// under you is lit by a hemisphere that is half dark forest, and the path
		// ends on the ground rather than running out to space, so its source
		// function is materially dimmer and less blue. Blending into a separate
		// down-band is what gives an aerial framing a black point at all.
		float hfogDown = clamp( - hfogDir.y * ${HFOG}[ 6 ].a, 0.0, 1.0 );
		hfogBase = mix( hfogBase, ${HFOG}[ 6 ].rgb, hfogDown * hfogDown );

		// Normalised to 1 in the sun's direction, so the in-scatter colour is an
		// absolute radiance the haze can gain and not an unbounded phase spike.
		float hfogG = ${HFOG}[ 2 ].a;
		float hfogPeak = ( 1.0 - hfogG * hfogG ) / ( 12.566370614 * pow( max( 1.0 - hfogG, 1.0e-3 ), 3.0 ) );
		float hfogCos = dot( hfogDir, ${HFOG}[ 3 ].xyz );
		float hfogMie = hfog_hg( hfogCos, hfogG ) / max( hfogPeak, 1.0e-4 );
		hfogFinal = hfog_present( hfogBase + ${HFOG}[ 2 ].rgb * hfogMie * ${HFOG}[ 4 ].w * fogFactor );

	}

	// Cloud shadow multiplies the *surface* and not the haze in front of it, so it
	// lands before the fog mix: a ridge under a cloud 4 km away is still seen
	// through 4 km of sunlit air. Applied in whatever space the target expects —
	// on the post-processing path (the shipping path) that is scene-referred
	// linear, which is where a shadow belongs.
	gl_FragColor.rgb *= hfog_cloudShade( vFogWorldPosition );

	gl_FragColor.rgb = mix( gl_FragColor.rgb, hfogFinal, clamp( fogFactor, 0.0, 1.0 ) );

#endif
`;

  // Publish the shared uniform everywhere a material could pick it up.
  const decl = { value: fogParams };
  if (THREE.UniformsLib && THREE.UniformsLib.fog) THREE.UniformsLib.fog[HFOG] = decl;
  for (const key in THREE.ShaderLib) {
    const lib = THREE.ShaderLib[key];
    if (lib && lib.uniforms && lib.uniforms[HFOG] === undefined) lib.uniforms[HFOG] = decl;
  }
}

installFogChunks();

// ===========================================================================
// 2. Procedural noise for the cloud volumes. Everything below tiles exactly by
//    construction, so the textures repeat across the sky without seams.
// ===========================================================================

function hashInt3(x, y, z, seed) {
  let h = (seed + Math.imul(x | 0, 0x27d4eb2d) + Math.imul(y | 0, 0x165667b1) + Math.imul(z | 0, 0x9e3779b1)) | 0;
  h = Math.imul(h ^ (h >>> 15), 0x2c1b3c6d);
  h = Math.imul(h ^ (h >>> 12), 0x297a2d39);
  h ^= h >>> 15;
  return (h >>> 0) / 4294967296;
}

/** Quintic (smootherstep) interpolant — C2 continuous, so no lattice creases. */
function fade5(t) { return t * t * t * (t * (t * 6 - 15) + 10); }

/** Value noise on a wrapping integer lattice: tiles exactly every `period`. */
function valueNoise3Tile(x, y, z, period, seed) {
  const fx = x * period, fy = y * period, fz = z * period;
  let i0 = Math.floor(fx), j0 = Math.floor(fy), k0 = Math.floor(fz);
  const tx = fade5(fx - i0), ty = fade5(fy - j0), tz = fade5(fz - k0);
  i0 = ((i0 % period) + period) % period;
  j0 = ((j0 % period) + period) % period;
  k0 = ((k0 % period) + period) % period;
  const i1 = (i0 + 1) % period, j1 = (j0 + 1) % period, k1 = (k0 + 1) % period;

  const c000 = hashInt3(i0, j0, k0, seed), c100 = hashInt3(i1, j0, k0, seed);
  const c010 = hashInt3(i0, j1, k0, seed), c110 = hashInt3(i1, j1, k0, seed);
  const c001 = hashInt3(i0, j0, k1, seed), c101 = hashInt3(i1, j0, k1, seed);
  const c011 = hashInt3(i0, j1, k1, seed), c111 = hashInt3(i1, j1, k1, seed);

  const x00 = c000 + (c100 - c000) * tx, x10 = c010 + (c110 - c010) * tx;
  const x01 = c001 + (c101 - c001) * tx, x11 = c011 + (c111 - c011) * tx;
  const y0 = x00 + (x10 - x00) * ty, y1 = x01 + (x11 - x01) * ty;
  return y0 + (y1 - y0) * tz;
}

function valueFbm3Tile(x, y, z, basePeriod, octaves, seed) {
  let sum = 0, amp = 0.5, norm = 0, p = basePeriod;
  for (let o = 0; o < octaves; o++) {
    sum += valueNoise3Tile(x, y, z, p, seed + o * 7919) * amp;
    norm += amp;
    amp *= 0.5;
    p *= 2;
  }
  return sum / norm;
}

function valueNoise2Tile(x, y, period, seed) {
  const fx = x * period, fy = y * period;
  let i0 = Math.floor(fx), j0 = Math.floor(fy);
  const tx = fade5(fx - i0), ty = fade5(fy - j0);
  i0 = ((i0 % period) + period) % period;
  j0 = ((j0 % period) + period) % period;
  const i1 = (i0 + 1) % period, j1 = (j0 + 1) % period;
  const c00 = hashInt3(i0, j0, 17, seed), c10 = hashInt3(i1, j0, 17, seed);
  const c01 = hashInt3(i0, j1, 17, seed), c11 = hashInt3(i1, j1, 17, seed);
  const a = c00 + (c10 - c00) * tx, b = c01 + (c11 - c01) * tx;
  return a + (b - a) * ty;
}

function valueFbm2Tile(x, y, basePeriod, octaves, seed) {
  let sum = 0, amp = 0.5, norm = 0, p = basePeriod;
  for (let o = 0; o < octaves; o++) {
    sum += valueNoise2Tile(x, y, p, seed + o * 6151) * amp;
    norm += amp;
    amp *= 0.5;
    p *= 2;
  }
  return sum / norm;
}

/** One jittered feature point per cell — the classic tileable Worley setup. */
function buildWorleyPoints(cells, rng) {
  const n = cells * cells * cells;
  const pts = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    pts[i * 3] = rng();
    pts[i * 3 + 1] = rng();
    pts[i * 3 + 2] = rng();
  }
  return pts;
}

/** F1 Worley distance in cell units, wrapping at the cell grid. 0 = on a point. */
function worley3(x, y, z, cells, pts) {
  const fx = x * cells, fy = y * cells, fz = z * cells;
  const cx = Math.floor(fx), cy = Math.floor(fy), cz = Math.floor(fz);
  let best = 4;
  for (let dz = -1; dz <= 1; dz++) {
    const nz = cz + dz;
    const wz = nz < 0 ? nz + cells : (nz >= cells ? nz - cells : nz);
    const baseZ = wz * cells;
    for (let dy = -1; dy <= 1; dy++) {
      const ny = cy + dy;
      const wy = ny < 0 ? ny + cells : (ny >= cells ? ny - cells : ny);
      const baseY = (baseZ + wy) * cells;
      for (let dx = -1; dx <= 1; dx++) {
        const nx = cx + dx;
        const wx = nx < 0 ? nx + cells : (nx >= cells ? nx - cells : nx);
        const o = (baseY + wx) * 3;
        const ex = (nx + pts[o]) - fx;
        const ey = (ny + pts[o + 1]) - fy;
        const ez = (nz + pts[o + 2]) - fz;
        const d2 = ex * ex + ey * ey + ez * ez;
        if (d2 < best) best = d2;
      }
    }
  }
  const d = Math.sqrt(best);
  return d > 1 ? 1 : d;
}

const remap01 = (v, lo, hi) => clamp01((v - lo) / Math.max(hi - lo, 1e-5));

/**
 * Histogram-equalise one byte channel of an interleaved buffer, in place, and
 * return the lookup table used.
 *
 * This matters more than it looks. The cloud density function is a chain of
 * `remap(value, 1 - coverage, 1, 0, 1)` thresholds, and those only behave if
 * the fields feeding them actually span 0..1. Raw fBm and Perlin-Worley both
 * pile up around their mean — measured, the un-equalised shape channel sat in
 * 0.3..0.8, which pushed every density below the coverage threshold and left
 * the sky 98% empty. Equalising makes the thresholds mean what they say, and
 * makes the coverage control behave linearly.
 */
function equalizeChannel(data, count, stride, offset) {
  const hist = new Uint32Array(256);
  for (let i = 0; i < count; i++) hist[data[i * stride + offset]]++;
  const lut = new Uint8Array(256);
  let acc = 0;
  for (let v = 0; v < 256; v++) {
    acc += hist[v];
    // Mid-bin CDF, so a value never maps to the very edge of the range.
    lut[v] = Math.max(0, Math.min(255, Math.round((255 * (acc - hist[v] * 0.5)) / count)));
  }
  for (let i = 0; i < count; i++) data[i * stride + offset] = lut[data[i * stride + offset]];
  return lut;
}

/**
 * Nubis-style cloud shape volume.
 *   R = Perlin-Worley (billowy base shape)
 *   G/B/A = inverted Worley at increasing frequency (the erosion cascade)
 */
function buildCloudShapeTexture(res, seed) {
  const rng = makeRng(seed);
  const cellsG = 6, cellsB = 12, cellsA = 24;
  const ptsG = buildWorleyPoints(cellsG, rng);
  const ptsB = buildWorleyPoints(cellsB, rng);
  const ptsA = buildWorleyPoints(cellsA, rng);

  const data = new Uint8Array(res * res * res * 4);
  const inv = 1 / res;
  let p = 0;
  for (let k = 0; k < res; k++) {
    const z = (k + 0.5) * inv;
    for (let j = 0; j < res; j++) {
      const y = (j + 0.5) * inv;
      for (let i = 0; i < res; i++) {
        const x = (i + 0.5) * inv;

        const wG = 1 - worley3(x, y, z, cellsG, ptsG);
        const wB = 1 - worley3(x, y, z, cellsB, ptsB);
        const wA = 1 - worley3(x, y, z, cellsA, ptsA);

        // Perlin-Worley: bias the smooth field by the low-frequency Worley so
        // the base shape grows cauliflower edges instead of soft blobs.
        const perlin = valueFbm3Tile(x, y, z, 4, 4, seed ^ 0x51ed);
        const pw = remap01(perlin, (1 - wG) * 0.62, 1.0);

        data[p++] = (pw * 255) | 0;
        data[p++] = (clamp01(wG) * 255) | 0;
        data[p++] = (clamp01(wB) * 255) | 0;
        data[p++] = (clamp01(wA) * 255) | 0;
      }
    }
  }
  // The base shape drives every density threshold downstream, so it has to use
  // the whole 0..1 range rather than clustering around its mean.
  equalizeChannel(data, res * res * res, 4, 0);
  return makeVolumeTexture(data, res);
}

/** High-frequency erosion volume — the wispy edges that sell the scale. */
function buildCloudDetailTexture(res, seed) {
  const rng = makeRng(seed);
  const c0 = 6, c1 = 12, c2 = 24;
  const p0 = buildWorleyPoints(c0, rng);
  const p1 = buildWorleyPoints(c1, rng);
  const p2 = buildWorleyPoints(c2, rng);

  const data = new Uint8Array(res * res * res * 4);
  const inv = 1 / res;
  let p = 0;
  for (let k = 0; k < res; k++) {
    const z = (k + 0.5) * inv;
    for (let j = 0; j < res; j++) {
      const y = (j + 0.5) * inv;
      for (let i = 0; i < res; i++) {
        const x = (i + 0.5) * inv;
        data[p++] = (clamp01(1 - worley3(x, y, z, c0, p0)) * 255) | 0;
        data[p++] = (clamp01(1 - worley3(x, y, z, c1, p1)) * 255) | 0;
        data[p++] = (clamp01(1 - worley3(x, y, z, c2, p2)) * 255) | 0;
        data[p++] = 255;
      }
    }
  }
  return makeVolumeTexture(data, res);
}

function makeVolumeTexture(data, res) {
  const tex = new THREE.Data3DTexture(data, res, res, res);
  tex.format = THREE.RGBAFormat;
  tex.type = THREE.UnsignedByteType;
  tex.minFilter = THREE.LinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.wrapR = THREE.RepeatWrapping;
  tex.colorSpace = THREE.NoColorSpace;   // data map, never sRGB (CONTRACT §0)
  tex.generateMipmaps = false;
  tex.unpackAlignment = 4;
  tex.needsUpdate = true;
  return tex;
}

/**
 * Weather map: R = coverage, G = cloud type (stratus .. cumulus), B = a slow
 * "weather front" mask so parts of the sky stay clear and others go heavy.
 * (R3-E4: this used to keep a CPU mirror of the coverage channel so the key light
 * could be dimmed globally as fronts drifted over. Ground cloud shadows are a
 * per-pixel field in the fog chunk now, so the mirror had no reader left.)
 */
function buildWeatherTexture(res, seed) {
  const data = new Uint8Array(res * res * 4);
  const inv = 1 / res;
  let p = 0;
  for (let j = 0; j < res; j++) {
    const y = (j + 0.5) * inv;
    for (let i = 0; i < res; i++) {
      const x = (i + 0.5) * inv;

      // Two decorrelated fields: broad fronts, and cell structure inside them.
      const front = valueFbm2Tile(x, y, 2, 3, seed ^ 0x1a2b);
      const cell = valueFbm2Tile(x, y, 7, 5, seed ^ 0x77c1);
      const cov = clamp01(cell * 0.62 + front * 0.48);

      const type = clamp01(valueFbm2Tile(x + 0.37, y - 0.21, 3, 4, seed ^ 0x3d5f) * 1.4 - 0.2);

      data[p++] = (cov * 255) | 0;
      data[p++] = (type * 255) | 0;
      data[p++] = (clamp01(front) * 255) | 0;
      data[p++] = 255;
    }
  }

  // Equalise coverage so the `coverage` control is linear and predictable.
  equalizeChannel(data, res * res, 4, 0);

  const tex = new THREE.DataTexture(data, res, res, THREE.RGBAFormat, THREE.UnsignedByteType);
  tex.minFilter = THREE.LinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.NoColorSpace;
  tex.generateMipmaps = false;
  tex.needsUpdate = true;
  return { texture: tex, res };
}

// ===========================================================================
// 3. Sky + cloud shader.
// ===========================================================================

const SKY_VERTEX = /* glsl */`
uniform vec3 sunPosition;
uniform float rayleigh;
uniform float turbidity;
uniform float mieCoefficient;
uniform vec3 up;

varying vec3 vWorldPosition;
varying vec3 vSunDirection;
varying vec3 vBetaR;
varying vec3 vBetaM;
varying float vSunE;

const float e = 2.718281828459045;
const float pi = 3.141592653589793;

// Preetham primaries: precomputed total Rayleigh scattering for 680/550/450 nm.
const vec3 totalRayleigh = vec3( 5.804542996261093E-6, 1.3562911419845635E-5, 3.0265902468824876E-5 );
const vec3 MieConst = vec3( 1.8399918514433978E14, 2.7798023919660528E14, 4.0790479543861094E14 );

const float cutoffAngle = 1.6110731556870734;   // pi / 1.95 — earth-shadow cutoff
const float steepness = 1.5;
const float EE = 1000.0;

float sunIntensity( float zenithAngleCos ) {
	zenithAngleCos = clamp( zenithAngleCos, -1.0, 1.0 );
	return EE * max( 0.0, 1.0 - pow( e, - ( ( cutoffAngle - acos( zenithAngleCos ) ) / steepness ) ) );
}

vec3 totalMie( float T ) {
	float c = ( 0.2 * T ) * 10E-18;
	return 0.434 * c * MieConst;
}

void main() {

	vec4 worldPosition = modelMatrix * vec4( position, 1.0 );
	vWorldPosition = worldPosition.xyz;

	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
	gl_Position.z = gl_Position.w;   // pin the dome to the far plane

	vSunDirection = normalize( sunPosition );
	vSunE = sunIntensity( dot( vSunDirection, up ) );

	// Stock Sky.js folds a "sunfade" term into rayleigh, but that term expects
	// sunPosition in metres and silently does nothing for a unit vector. The
	// low-sun reddening is driven explicitly from JS instead (setSunAngles), so
	// rayleigh/turbidity stay fully art-directable across the day.
	vBetaR = totalRayleigh * rayleigh;
	vBetaM = totalMie( turbidity ) * mieCoefficient;

}
`;

function buildSkyFragment(cloudSteps, volumetric) {
  return /* glsl */`
varying vec3 vWorldPosition;
varying vec3 vSunDirection;
varying vec3 vBetaR;
varying vec3 vBetaM;
varying float vSunE;

uniform float mieDirectionalG;
uniform vec3 up;

uniform float uSkyExposure;
uniform vec2 uSkyShoulder;   // x = knee luminance, y = compression exponent
uniform float uSunDiscSize;
uniform float uSunDiscIntensity;
uniform float uShowSunDisc;
uniform float uNight;
uniform vec3 uGroundHaze;   // ground seen at grazing incidence: nearly all haze
uniform vec3 uGroundDeep;   // ground seen ~15 degrees down: sunlit terrain bounce
uniform vec3 uHorizonColor;

uniform sampler3D uShapeTex;
uniform sampler3D uDetailTex;
uniform sampler2D uWeatherTex;

uniform vec3 uRayOrigin;
uniform vec3 uSunColor;
uniform vec3 uCloudAmbientTop;
uniform vec3 uCloudAmbientBottom;
uniform vec4 uCloudLayer;   // bottom, top, planetRadius, maxMarchDistance
uniform vec4 uCloudShape;   // shapeScale, detailScale, weatherScale, detailStrength
uniform vec4 uCloudCover;   // coverage, densityMul, sigmaExtinction, lightStep
uniform vec4 uCloudWind;    // shapeOffset.xz, detailOffset.xz

#define CLOUD_STEPS ${cloudSteps}
${volumetric ? '#define VOLUMETRIC_CLOUDS 1' : ''}

const float PI = 3.141592653589793;
const float FOUR_PI = 12.566370614359172;
const float THREE_OVER_SIXTEENPI = 0.05968310365946075;
const float ONE_OVER_FOURPI = 0.07957747154594767;
const float rayleighZenithLength = 8.4E3;
const float mieZenithLength = 1.25E3;

float rayleighPhase( float cosTheta ) {
	return THREE_OVER_SIXTEENPI * ( 1.0 + cosTheta * cosTheta );
}

float hgPhase( float cosTheta, float g ) {
	float g2 = g * g;
	float d = max( 1.0 - 2.0 * g * cosTheta + g2, 1.0e-4 );
	return ONE_OVER_FOURPI * ( ( 1.0 - g2 ) / ( d * sqrt( d ) ) );
}

float remap( float v, float lo, float hi, float nlo, float nhi ) {
	return nlo + ( v - lo ) * ( nhi - nlo ) / max( hi - lo, 1.0e-4 );
}

// R5-E2 — the sky highlight shoulder.
//
// Preetham's radiance field is enormously wider than the key light it is meant
// to share a photograph with. Measured against a fully sunlit albedo-1.0 card
// (sun.intensity 3.51 / PI = 1.117 linear) at the shipping 19.2-degree sun, the
// UNCOMPRESSED dome delivers: zenith 0.18x the card (about right), the 5-degree
// horizon ring 1.07x, and 20 degrees from the sun **27x**. There is no exposure
// at which 27x and 0.18x both survive ACES, so the frame chose: the sky clipped
// to white over the upper third of twelve of sixteen review shots and measured
// saturation 0.035, while the model underneath it was correctly hued the whole
// time (that is why round 3 rejected "rebuild it as Rayleigh/Mie" — the model is
// right, its top end is not).
//
// So the top end is compressed at source, in the dome's own units, before
// anything downstream sees it: above a knee K the luminance follows
// K * (L/K)^p with p < 1. Three properties matter.
//   * It is applied to the LUMINANCE and the RGB is scaled by the ratio, so hue
//     and saturation are untouched — a sunset stays orange, a zenith stays blue.
//     (Saturation actually *rises*, because it is ACES's own shoulder that was
//     bleaching these values to white, and they no longer reach it.)
//   * It is a power law, not a hard ceiling, so the circumsolar glare still has
//     a real gradient running into the sun rather than a flat white plateau.
//   * It is applied to Lin only. L0 — the limb-darkened solar disc and its
//     tight aureole — is deliberately left uncompressed so the disc stays a
//     ~1000:1 bloom source with genuine structure. The aureole falls to ~1.0 at
//     4 degrees out, which is where the compressed sky now sits, so the two meet
//     with no seam.
// Mirrored exactly on the CPU (atmosRadiance), because the fog bands, the IBL,
// the ground bounce and the cloud ambients are all sampled from that mirror and
// must not disagree with the dome by a factor of three.
vec3 skyShoulder( vec3 c ) {
	float k = uSkyShoulder.x;
	float l = dot( c, vec3( 0.2126, 0.7152, 0.0722 ) );
	if ( k <= 0.0 || l <= k ) return c;
	return c * ( k * pow( l / k, uSkyShoulder.y ) / l );
}

float hash13( vec3 p ) {
	p = fract( p * 0.1031 );
	p += dot( p, p.zyx + 31.32 );
	return fract( ( p.x + p.y ) * p.z );
}

// Interleaved gradient noise: an *ordered* dither, so the cloud step offset does
// not crawl frame to frame the way a random dither would with no TAA in front.
float ign( vec2 p ) {
	return fract( 52.9829189 * fract( 0.06711056 * p.x + 0.00583715 * p.y ) );
}

// ---------------------------------------------------------------------------
// Atmosphere (Preetham analytic model)
// ---------------------------------------------------------------------------
vec3 atmosphere( vec3 direction, out vec3 Fex ) {

	// Relative optical path length, cut off at 90 degrees to dodge the singularity.
	float zenithAngle = acos( max( 0.0, dot( up, direction ) ) );
	float inv = 1.0 / ( cos( zenithAngle ) + 0.15 * pow( 93.885 - ( ( zenithAngle * 180.0 ) / PI ), -1.253 ) );
	float sR = rayleighZenithLength * inv;
	float sM = mieZenithLength * inv;

	Fex = exp( - ( vBetaR * sR + vBetaM * sM ) );

	float cosTheta = dot( direction, vSunDirection );
	vec3 betaRTheta = vBetaR * rayleighPhase( cosTheta * 0.5 + 0.5 );
	vec3 betaMTheta = vBetaM * hgPhase( cosTheta, mieDirectionalG );

	vec3 ratio = ( betaRTheta + betaMTheta ) / ( vBetaR + vBetaM );
	vec3 Lin = pow( vSunE * ratio * ( 1.0 - Fex ), vec3( 1.5 ) );
	Lin *= mix(
		vec3( 1.0 ),
		pow( vSunE * ratio * Fex, vec3( 0.5 ) ),
		clamp( pow( 1.0 - dot( up, vSunDirection ), 5.0 ), 0.0, 1.0 )
	);

	return Lin;
}

// ---------------------------------------------------------------------------
// Cloud volume
// ---------------------------------------------------------------------------
float cloudAltitude( vec3 p ) {
	return length( p - vec3( 0.0, - uCloudLayer.z, 0.0 ) ) - uCloudLayer.z;
}

float cloudHeightFrac( vec3 p ) {
	return clamp( ( cloudAltitude( p ) - uCloudLayer.x ) / max( uCloudLayer.y - uCloudLayer.x, 1.0 ), 0.0, 1.0 );
}

vec4 sampleWeather( vec3 p ) {
	vec2 uv = ( p.xz + uCloudWind.xy * 0.35 ) * uCloudShape.z;
	return texture( uWeatherTex, uv );
}

// Vertical density profile. Stratus hugs the base of the layer; cumulus towers.
float heightGradient( float h, float type ) {
	float stratus = smoothstep( 0.0, 0.06, h ) * ( 1.0 - smoothstep( 0.13, 0.32, h ) );
	float cumulus = smoothstep( 0.02, 0.26, h ) * ( 1.0 - smoothstep( 0.58, 1.0, h ) );
	return mix( stratus, cumulus, type );
}

float cloudDensity( vec3 p, float hf, vec4 wth, bool detail ) {

	vec3 sp = ( p + vec3( uCloudWind.x, 0.0, uCloudWind.y ) ) * uCloudShape.x;
	// Wind shear: the top of the layer is dragged ahead of the base.
	sp.xz += hf * 0.09;

	vec4 shape = texture( uShapeTex, sp );
	float fbmS = shape.g * 0.625 + shape.b * 0.25 + shape.a * 0.125;
	float base = remap( shape.r, fbmS - 1.0, 1.0, 0.0, 1.0 );

	float type = clamp( wth.g * 1.25, 0.0, 1.0 );
	base *= heightGradient( hf, type );

	float cover = clamp( wth.r * ( 0.55 + 0.9 * wth.b ) + uCloudCover.x - 0.5, 0.0, 1.0 );
	float d = remap( base, 1.0 - cover, 1.0, 0.0, 1.0 ) * cover;
	if ( d <= 0.0 ) return 0.0;

	if ( detail ) {
		vec3 dp = ( p + vec3( uCloudWind.z, - uCloudWind.w * 0.5, uCloudWind.w ) ) * uCloudShape.y;
		vec3 det = texture( uDetailTex, dp ).rgb;
		float fbmD = det.r * 0.625 + det.g * 0.25 + det.b * 0.125;
		// Wispy at the base, billowy up top. Flipping the erosion with height is
		// what stops the layer reading as one uniform slab of noise.
		float m = mix( 1.0 - fbmD, fbmD, clamp( hf * 5.0, 0.0, 1.0 ) );
		d = remap( d, m * uCloudShape.w, 1.0, 0.0, 1.0 );
	}

	return clamp( d, 0.0, 1.0 ) * uCloudCover.y;
}

#ifdef VOLUMETRIC_CLOUDS

vec3 coneOffset( float i ) {
	float a = i * 2.399963;                 // golden angle => well-spread cone
	float r = ( i + 1.0 ) / 6.0;
	return vec3( cos( a ) * r, ( fract( i * 0.618034 ) - 0.5 ) * r, sin( a ) * r );
}

float cloudLightOpticalDepth( vec3 p, vec3 sd ) {
	float od = 0.0;
	// 4 cone taps, not 5: the light march is evaluated at every dense sample of
	// the primary march, so it is the single largest term in the cloud cost and
	// the 5th tap is inside the shadow of the first four (measured: no visible
	// change in the silver lining). The long reach below still catches a thick
	// body further up the light ray, which is what the 5th tap was really for.
	float st0 = uCloudCover.w;
	float st = st0 * 1.25;      // widened so 4 taps span the same optical path
	for ( int i = 0; i < 4; i ++ ) {
		float fi = float( i );
		vec3 q = p + sd * ( st * ( fi + 0.5 ) ) + coneOffset( fi ) * st * fi * 0.42;
		od += cloudDensity( q, cloudHeightFrac( q ), sampleWeather( q ), i < 2 ) * st;
	}
	// One long reach, to catch a thick body further up the light ray. Kept on
	// the unwidened step so its distance and weight are unchanged.
	vec3 q = p + sd * st0 * 11.0;
	od += cloudDensity( q, cloudHeightFrac( q ), sampleWeather( q ), false ) * st0 * 6.0;
	return od;
}

vec4 marchClouds( vec3 ro, vec3 rd, vec3 sd, float dither ) {

	vec3 oc = ro - vec3( 0.0, - uCloudLayer.z, 0.0 );
	float b = dot( oc, rd );
	float o2 = dot( oc, oc );

	// A ray that hits the planet never reaches the layer. (The horizon really
	// does dip below level once you are a few hundred metres up.)
	float Rp = uCloudLayer.z;
	float dP = b * b - ( o2 - Rp * Rp );
	if ( dP > 0.0 && ( - b - sqrt( dP ) ) > 0.0 ) return vec4( 0.0, 0.0, 0.0, 1.0 );

	float Rb = uCloudLayer.z + uCloudLayer.x;
	float Rt = uCloudLayer.z + uCloudLayer.y;
	float dT = b * b - ( o2 - Rt * Rt );
	if ( dT <= 0.0 ) return vec4( 0.0, 0.0, 0.0, 1.0 );
	float dB = b * b - ( o2 - Rb * Rb );

	// Below the layer, the far root of the inner shell is the entry point.
	float tStart = dB > 0.0 ? max( - b + sqrt( dB ), 0.0 ) : 0.0;
	float tEnd = - b + sqrt( dT );
	if ( tEnd <= tStart ) return vec4( 0.0, 0.0, 0.0, 1.0 );
	tEnd = min( tEnd, tStart + uCloudLayer.w );

	float stepSize = ( tEnd - tStart ) / float( CLOUD_STEPS );
	float t = tStart + stepSize * dither;

	float cosT = dot( rd, sd );
	float sigma = uCloudCover.z;

	// R3-E5 — the three multiple-scattering phase values depend only on cosT,
	// which is constant along the ray, and on literal g values. Evaluated inside
	// the sample loop that was six hgPhase() calls per sample, 32 samples deep,
	// for six numbers that never change. Hoisted here: bit-identical output.
	float ph0 = mix( hgPhase( cosT, 0.72 ), hgPhase( cosT, -0.34 ), 0.26 );
	float ph1 = mix( hgPhase( cosT, 0.36 ), hgPhase( cosT, -0.17 ), 0.26 );
	float ph2 = mix( hgPhase( cosT, 0.18 ), hgPhase( cosT, -0.085 ), 0.26 );

	vec3 scattered = vec3( 0.0 );
	float trans = 1.0;
	float depthSum = 0.0;
	float depthWeight = 0.0;

	for ( int i = 0; i < CLOUD_STEPS; i ++ ) {

		if ( trans < 0.012 ) break;

		vec3 p = ro + rd * t;
		float hf = cloudHeightFrac( p );
		vec4 wth = sampleWeather( p );
		float dens = cloudDensity( p, hf, wth, true );

		if ( dens > 0.002 ) {

			float lod = cloudLightOpticalDepth( p, sd );

			// Three-octave multiple-scattering approximation (Frostbite): each
			// octave is dimmer, penetrates further and scatters more isotropically.
			// This is what fills the shadowed side of a cloud without flattening
			// the silver lining on the sunward edge.
			float lodS = lod * sigma;
			vec3 lum = vec3(
				ph0 * exp( - lodS )
				+ 0.52 * ph1 * exp( - lodS * 0.58 )
				+ 0.52 * 0.52 * ph2 * exp( - lodS * 0.58 * 0.58 )
			);
			lum *= FOUR_PI;

			// R3-E2 — a cumulus base sits in the shadow of its own body. The light
			// march above reaches ~460 m up-sun, which at a 19-degree sun elevation
			// is barely 150 m of *vertical*, so the base of a 1.7 km-thick layer was
			// being lit as brightly as its top and the cloudscape read as one flat
			// white ceiling with no body to it. This is the missing lit/shadowed
			// separation: it holds the top at full radiance and takes the base to
			// 30%, which after ACES is ~0.19 of final sRGB, and the ambient gradient
			// below (bottom = fogGround*0.22, top = fogHigh*0.90) carries the rest.
			lum *= mix( 0.30, 1.0, smoothstep( 0.02, 0.62, hf ) );

			// Powder term: darkened edges when looking down-sun, which is what
			// makes a lit cloud read as a solid body and not a decal.
			float powder = 1.0 - exp( - dens * 14.0 );
			lum *= mix( 1.0, powder, clamp( 0.5 - 0.5 * cosT, 0.0, 1.0 ) );

			vec3 amb = mix( uCloudAmbientBottom, uCloudAmbientTop, hf * hf * 0.85 + 0.15 );

			// Energy-conserving analytic integration over the step. With a
			// scattering albedo of ~1 (cloud droplets barely absorb visible
			// light) the in-scatter integral collapses to L * ( 1 - T ), which
			// is bounded — so the result cannot change brightness when the step
			// size does, and a coarse step near the horizon stays consistent
			// with a fine step overhead.
			float ext = max( dens * sigma, 1.0e-6 );
			float dTr = exp( - ext * stepSize );
			vec3 Sint = ( uSunColor * lum + amb ) * ( 1.0 - dTr );

			scattered += trans * Sint;
			depthSum += t * trans * ( 1.0 - dTr );
			depthWeight += trans * ( 1.0 - dTr );
			trans *= dTr;

		}

		t += stepSize;

	}

	// Aerial perspective: distant cloud must wash into the horizon colour, or
	// the layer reads as a painted ceiling with no depth.
	if ( depthWeight > 0.0 ) {
		float meanDepth = depthSum / depthWeight;
		float ap = 1.0 - exp( - meanDepth * 2.6e-5 );
		scattered = mix( scattered, uHorizonColor * ( 1.0 - trans ), ap * 0.85 );
	}

	return vec4( scattered, trans );

}

#else   // cheap tier — one shaded slab, still fully procedural. No billboards.

vec4 marchClouds( vec3 ro, vec3 rd, vec3 sd, float dither ) {

	if ( rd.y <= 0.012 ) return vec4( 0.0, 0.0, 0.0, 1.0 );

	float t = ( uCloudLayer.x + 260.0 - ro.y ) / rd.y;
	if ( t <= 0.0 ) return vec4( 0.0, 0.0, 0.0, 1.0 );
	t = min( t, 55000.0 );

	vec3 p = ro + rd * t;
	vec4 wth = sampleWeather( p );

	vec3 sp = ( p + vec3( uCloudWind.x, 0.0, uCloudWind.y ) ) * uCloudShape.x;
	vec4 shape = texture( uShapeTex, sp );
	float fbmS = shape.g * 0.625 + shape.b * 0.25 + shape.a * 0.125;
	float base = clamp( remap( shape.r, fbmS - 1.0, 1.0, 0.0, 1.0 ), 0.0, 1.0 );

	float cover = clamp( wth.r * ( 0.55 + 0.9 * wth.b ) + uCloudCover.x - 0.5, 0.0, 1.0 );
	float d = clamp( remap( base, 1.0 - cover, 1.0, 0.0, 1.0 ) * cover, 0.0, 1.0 );

	// Re-sampling the same field offset towards the sun fakes self-shadowing.
	vec3 sp2 = sp + sd * uCloudShape.x * 300.0;
	vec4 shape2 = texture( uShapeTex, sp2 );
	float fbm2 = shape2.g * 0.625 + shape2.b * 0.25 + shape2.a * 0.125;
	float occ = clamp( remap( shape2.r, fbm2 - 1.0, 1.0, 0.0, 1.0 ) * cover, 0.0, 1.0 );
	float lit = 1.0 - clamp( occ * 1.25, 0.0, 0.88 );

	float cosT = dot( rd, sd );
	float ph = mix( hgPhase( cosT, 0.74 ), hgPhase( cosT, -0.30 ), 0.25 ) * FOUR_PI;

	vec3 col = uSunColor * ph * lit * ( 0.35 + 0.65 * lit )
		+ mix( uCloudAmbientBottom, uCloudAmbientTop, 0.6 );

	float alpha = d * smoothstep( 0.015, 0.14, rd.y );
	float ap = 1.0 - exp( - t * 2.6e-5 );
	col = mix( col, uHorizonColor, ap * 0.85 );

	return vec4( col * alpha, 1.0 - alpha );

}

#endif

// ---------------------------------------------------------------------------
void main() {

	vec3 direction = normalize( vWorldPosition - cameraPosition );

	vec3 Fex;
	vec3 Lin = atmosphere( direction, Fex );

	float cosTheta = dot( direction, vSunDirection );

	// Solar disc with limb darkening. The real disc is ~0.53 degrees across;
	// uSunDiscSize nudges that up slightly so bloom and god rays have genuine
	// structure to key off rather than a single blown-out texel.
	vec3 L0 = vec3( 0.06 ) * Fex;
	float ang = acos( clamp( cosTheta, -1.0, 1.0 ) );
	float r = ang / uSunDiscSize;
	float disc = 1.0 - smoothstep( 0.975, 1.0, r );
	if ( disc > 0.0 ) {
		float mu = sqrt( max( 0.0, 1.0 - r * r ) );
		// Visible-band limb darkening (u1 = 0.397, u2 = 0.155): the disc is
		// measurably dimmer and redder at its rim than at its centre.
		float limb = 1.0 - 0.397 * ( 1.0 - mu ) - 0.155 * ( 1.0 - mu ) * ( 1.0 - mu );
		L0 += vSunE * uSunDiscIntensity * Fex * disc * limb * uShowSunDisc;
	}
	// Aureole: the tight forward-scattered halo just outside the disc.
	L0 += vSunE * 26.0 * Fex * pow( max( cosTheta, 0.0 ), 1400.0 ) * uShowSunDisc;

	// Clamped well under the half-float ceiling: the raw solar disc runs to
	// ~2e5 at high sun, which would write Inf into the post-processing buffer
	// and poison the entire bloom chain with NaN.
	// The atmospheric in-scatter goes through the shoulder; the disc and aureole
	// (L0) deliberately do not — see skyShoulder() above.
	vec3 skyColor = min(
		skyShoulder( Lin * uSkyExposure ) + L0 * uSkyExposure + vec3( 0.0006, 0.0011, 0.0021 ),
		vec3( 2200.0 )
	);

	// Stars, only once the sun is genuinely down.
	if ( uNight > 0.002 ) {
		vec3 sdir = direction * 260.0;
		vec3 cell = floor( sdir );
		float h = hash13( cell );
		if ( h > 0.9835 ) {
			vec3 jitter = vec3( hash13( cell + 11.0 ), hash13( cell + 23.0 ), hash13( cell + 37.0 ) );
			float dist = length( fract( sdir ) - jitter );
			float star = exp( - dist * dist * 90.0 ) * ( h - 0.9835 ) * 60.0;
			// A little chromatic spread so the field is not uniformly white.
			vec3 tint = mix( vec3( 0.72, 0.82, 1.0 ), vec3( 1.0, 0.86, 0.68 ), hash13( cell + 5.0 ) );
			skyColor += tint * star * uNight * smoothstep( -0.02, 0.12, direction.y );
		}
	}

	// Below the horizon the Preetham model is undefined. The old code stepped
	// onto a flat constant over a ~4 degree window, which put a hard, colourless
	// mint band under every horizon — and is what three critics measured and
	// misread as "a flat plastic water plane". Ramp over ~15 degrees instead,
	// from the grazing haze (which is essentially the sky at 0 degrees, so the
	// seam is invisible) down into the sunlit-terrain bounce colour. Both ends
	// are derived on the CPU from the same atmosphere the dome renders and the
	// same albedo the PMREM ground disc uses, so dome, IBL and fog agree.
	float belowY = clamp( - direction.y, 0.0, 1.0 );
	vec3 belowColor = mix( uGroundHaze, uGroundDeep, smoothstep( 0.012, 0.26, belowY ) );
	skyColor = mix( belowColor, skyColor, smoothstep( -0.26, 0.015, direction.y ) );

	vec4 clouds = marchClouds( uRayOrigin, direction, vSunDirection, ign( gl_FragCoord.xy ) );
	vec3 color = min( skyColor * clouds.a + clouds.rgb, vec3( 2200.0 ) );

	gl_FragColor = vec4( color, 1.0 );

	#include <tonemapping_fragment>
	#include <colorspace_fragment>

}
`;
}

// ===========================================================================
// 4. CPU mirror of the atmosphere model.
//
// The fog colour, the horizon colour the clouds fade into, and the strength of
// the image-based lighting all have to agree with what the dome actually
// renders — otherwise distant ridges blend into a haze that is not the colour
// of the sky behind them, which is the single most obvious "this is a game"
// tell in an outdoor scene. Rather than hand-authoring those colours and hoping
// they match, this evaluates exactly the same Preetham maths as the fragment
// shader, on the CPU, whenever the sun moves. ~25 evaluations per sun change.
// ===========================================================================

const P_TOTAL_RAYLEIGH = [5.804542996261093e-6, 1.3562911419845635e-5, 3.0265902468824876e-5];
const P_MIE_CONST = [1.8399918514433978e14, 2.7798023919660528e14, 4.0790479543861094e14];
const P_CUTOFF_ANGLE = 1.6110731556870734;
const P_STEEPNESS = 1.5;
const P_EE = 1000;
const P_RAYLEIGH_ZENITH = 8.4e3;
const P_MIE_ZENITH = 1.25e3;
const P_3_16PI = 0.05968310365946075;
const P_1_4PI = 0.07957747154594767;

// R5-E2 — the sky highlight shoulder, authored here and consumed by BOTH the
// fragment shader (uniform uSkyShoulder) and the CPU mirror below, so the dome,
// the fog bands, the IBL, the ground bounce and the cloud ambients cannot drift
// apart. See the long note on skyShoulder() in the fragment shader for why.
//
// Knee 0.22 sits just above the zenith (0.23 at the shipping sun), so the blue
// part of the sky is passed through completely untouched and only the horizon
// glare and the circumsolar lobe are compressed. Exponent 0.40 takes the worst
// case — 20 degrees from a 19-degree sun, measured at 30.7 linear, i.e. 27x a
// sunlit white card — down to 1.18, and the 5-degree horizon ring from 1.20 to
// 0.46. Nothing in the dome now exceeds ~1.1x the key's white card except the
// solar disc itself, which is exempt.
const SKY_SHOULDER_KNEE = 0.22;
const SKY_SHOULDER_POWER = 0.40;

// Preallocated so the evaluator never allocates.
const _pBetaR = [0, 0, 0];
const _pBetaM = [0, 0, 0];
const _pFex = [0, 0, 0];
const _pLin = [0, 0, 0];
const _pOut = [0, 0, 0];
const _pAcc = [0, 0, 0];
const _pDir = [0, 0, 0];

/** Atmosphere state, refreshed from the live uniforms before a batch of samples. */
const atmos = { sunE: 0, mieG: 0.8, exposure: 0.02, sun: [0, 1, 0] };

function atmosPrepare(rayleigh, turbidity, mieCoefficient, mieG, exposure, sx, sy, sz) {
  for (let i = 0; i < 3; i++) _pBetaR[i] = P_TOTAL_RAYLEIGH[i] * rayleigh;
  const c = 0.2 * turbidity * 1e-17;
  for (let i = 0; i < 3; i++) _pBetaM[i] = 0.434 * c * P_MIE_CONST[i] * mieCoefficient;
  const zc = clamp(sy, -1, 1);
  atmos.sunE = P_EE * Math.max(0, 1 - Math.exp(-((P_CUTOFF_ANGLE - Math.acos(zc)) / P_STEEPNESS)));
  atmos.mieG = mieG;
  atmos.exposure = exposure;
  atmos.sun[0] = sx; atmos.sun[1] = sy; atmos.sun[2] = sz;
}

/** Sky radiance along a unit direction, in the same units the dome outputs. */
function atmosRadiance(dx, dy, dz, out) {
  const zenithAngle = Math.acos(Math.max(0, dy));
  const inv = 1 / (Math.cos(zenithAngle) + 0.15 * Math.pow(93.885 - (zenithAngle * 180) / Math.PI, -1.253));
  const sR = P_RAYLEIGH_ZENITH * inv;
  const sM = P_MIE_ZENITH * inv;
  for (let i = 0; i < 3; i++) _pFex[i] = Math.exp(-(_pBetaR[i] * sR + _pBetaM[i] * sM));

  const ct = dx * atmos.sun[0] + dy * atmos.sun[1] + dz * atmos.sun[2];
  const rc = ct * 0.5 + 0.5;
  const rPhase = P_3_16PI * (1 + rc * rc);
  const g = atmos.mieG, g2 = g * g;
  const d = Math.max(1 - 2 * g * ct + g2, 1e-4);
  const mPhase = P_1_4PI * ((1 - g2) / (d * Math.sqrt(d)));

  const t = clamp01(Math.pow(Math.max(1 - atmos.sun[1], 0), 5));
  for (let i = 0; i < 3; i++) {
    const ratio = (_pBetaR[i] * rPhase + _pBetaM[i] * mPhase) / (_pBetaR[i] + _pBetaM[i]);
    let lin = Math.pow(Math.max(atmos.sunE * ratio * (1 - _pFex[i]), 0), 1.5);
    lin *= lerp(1, Math.sqrt(Math.max(atmos.sunE * ratio * _pFex[i], 0)), t);
    _pLin[i] = lin * atmos.exposure;
  }

  // The same shoulder the dome applies, on the same term, in the same order —
  // this mirror feeds the fog bands, the IBL normalisation, the ground bounce
  // and the cloud ambients, and every one of those has to agree with the pixels.
  const sl = 0.2126 * _pLin[0] + 0.7152 * _pLin[1] + 0.0722 * _pLin[2];
  if (SKY_SHOULDER_KNEE > 0 && sl > SKY_SHOULDER_KNEE) {
    const k = SKY_SHOULDER_KNEE * Math.pow(sl / SKY_SHOULDER_KNEE, SKY_SHOULDER_POWER) / sl;
    _pLin[0] *= k; _pLin[1] *= k; _pLin[2] *= k;
  }

  for (let i = 0; i < 3; i++) out[i] = _pLin[i] + 0.06 * _pFex[i] * atmos.exposure;
  out[0] += 0.0006; out[1] += 0.0011; out[2] += 0.0021;
  return out;
}

/**
 * Radiance averaged around a ring of constant elevation, weighted *away* from
 * the sun so the solar glare does not dominate the base haze colour (the glow
 * around the sun is added back separately as the in-scatter term).
 */
function atmosRingAverage(elevRad, out) {
  const cy = Math.sin(elevRad), cr = Math.cos(elevRad);
  _pAcc[0] = 0; _pAcc[1] = 0; _pAcc[2] = 0;
  let wsum = 0;
  for (let i = 0; i < 8; i++) {
    const a = (i / 8) * TAU;
    _pDir[0] = Math.cos(a) * cr; _pDir[1] = cy; _pDir[2] = Math.sin(a) * cr;
    const ct = Math.max(0, _pDir[0] * atmos.sun[0] + _pDir[1] * atmos.sun[1] + _pDir[2] * atmos.sun[2]);
    const w = 1 / (1 + 7 * ct * ct * ct);
    atmosRadiance(_pDir[0], _pDir[1], _pDir[2], _pOut);
    _pAcc[0] += _pOut[0] * w; _pAcc[1] += _pOut[1] * w; _pAcc[2] += _pOut[2] * w;
    wsum += w;
  }
  out.setRGB(_pAcc[0] / wsum, _pAcc[1] / wsum, _pAcc[2] / wsum, THREE.LinearSRGBColorSpace);
  return out;
}

// A fixed low-discrepancy cosine-weighted hemisphere set, built once. Used to
// integrate sky irradiance so the environment map's contribution can be scaled
// against the sun instead of being left at Preetham's arbitrary units.
const IRRADIANCE_SAMPLES = (() => {
  const N = 192;
  const arr = new Float32Array(N * 3);
  for (let i = 0; i < N; i++) {
    const u = (i + 0.5) / N;
    // van der Corput radical inverse, base 2
    let b = i;
    b = ((b << 16) | (b >>> 16)) >>> 0;
    b = (((b & 0x55555555) << 1) | ((b & 0xaaaaaaaa) >>> 1)) >>> 0;
    b = (((b & 0x33333333) << 2) | ((b & 0xcccccccc) >>> 2)) >>> 0;
    b = (((b & 0x0f0f0f0f) << 4) | ((b & 0xf0f0f0f0) >>> 4)) >>> 0;
    b = (((b & 0x00ff00ff) << 8) | ((b & 0xff00ff00) >>> 8)) >>> 0;
    const v = b * 2.3283064365386963e-10;
    const r = Math.sqrt(u), phi = TAU * v;
    arr[i * 3] = r * Math.cos(phi);
    arr[i * 3 + 1] = Math.sqrt(Math.max(0, 1 - u));
    arr[i * 3 + 2] = r * Math.sin(phi);
  }
  return arr;
})();

/**
 * Sky irradiance on an upward-facing surface, RGB, in the dome's own radiance
 * units. Written into `out` (a THREE.Color) and returned as its luminance —
 * the colour is what the ground-bounce derivation needs (a mountain lit by a
 * blue sky bounces blue-tinted light back up into every overhang), the
 * luminance is what the environment-intensity normalisation needs.
 */
function atmosSkyIrradiance(out) {
  const n = IRRADIANCE_SAMPLES.length / 3;
  let r = 0, g = 0, b = 0;
  for (let i = 0; i < n; i++) {
    atmosRadiance(IRRADIANCE_SAMPLES[i * 3], IRRADIANCE_SAMPLES[i * 3 + 1], IRRADIANCE_SAMPLES[i * 3 + 2], _pOut);
    r += _pOut[0]; g += _pOut[1]; b += _pOut[2];
  }
  const k = Math.PI / n;
  r *= k; g *= k; b *= k;
  if (out) out.setRGB(r, g, b, THREE.LinearSRGBColorSpace);
  return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

// ===========================================================================
// 5. Colour helpers
// ===========================================================================

/**
 * Blackbody colour (Tanner Helland's fit), normalised so the brightest channel
 * is 1.0 — the light's `intensity` then carries all of the exposure and the
 * colour temperature carries none of it. Authored in sRGB, converted to the
 * linear working space by Color.setRGB (CONTRACT §0).
 */
function kelvinToColor(kelvin, out) {
  const t = clamp(kelvin, 1000, 40000) / 100;
  let r, g, b;
  if (t <= 66) {
    r = 255;
    g = 99.4708025861 * Math.log(t) - 161.1195681661;
    b = t <= 19 ? 0 : 138.5177312231 * Math.log(t - 10) - 305.0447927307;
  } else {
    r = 329.698727446 * Math.pow(t - 60, -0.1332047592);
    g = 288.1221695283 * Math.pow(t - 60, -0.0755148492);
    b = 255;
  }
  r = clamp(r, 0, 255); g = clamp(g, 0, 255); b = clamp(b, 0, 255);
  const m = Math.max(r, g, b, 1);
  out.setRGB(r / m, g / m, b / m, THREE.SRGBColorSpace);
  return out;
}

/**
 * Narkowicz ACES approximation — close enough to three's ACESFilmicToneMapping
 * for previewing a single colour on the CPU (used for the fallback fog colour).
 */
function acesApprox(x) {
  const v = Math.max(x, 0);
  return clamp01((v * (2.51 * v + 0.03)) / (v * (2.43 * v + 0.59) + 0.14));
}

/** Scale a colour down until its brightest channel fits, preserving its hue. */
function capColor(color, maxChannel) {
  const m = Math.max(color.r, color.g, color.b);
  if (m > maxChannel) color.multiplyScalar(maxChannel / m);
  return color;
}

/** Correlated colour temperature of direct sunlight vs elevation (radians). */
function sunTemperature(elevationRad) {
  const e = elevationRad / DEG;
  if (e <= 0) return lerp(1700, 2250, clamp01((e + 8) / 8));
  if (e < 4) return lerp(2250, 3250, e / 4);
  if (e < 10) return lerp(3250, 4450, (e - 4) / 6);
  if (e < 20) return lerp(4450, 5300, (e - 10) / 10);
  if (e < 45) return lerp(5300, 5950, (e - 20) / 25);
  return lerp(5950, 6400, clamp01((e - 45) / 45));
}

// ===========================================================================
// 6. createSky
// ===========================================================================

export function createSky(ctx) {
  const scene = ctx.scene;
  const renderer = ctx.renderer;
  const settings = ctx.settings || {};
  const quality = ctx.quality || 'high';
  const rng = makeRng(subSeed(ctx.seed ?? 0, 'sky'));

  // -------------------------------------------------------------------------
  // Tunables. Physical-ish, paired with ACES at toneMappingExposure 1.0.
  //
  // NOTHING here is allowed to hardcode the world's altitude. Everything that
  // depends on where the mountain actually is reads terrain.bounds, because the
  // last time it did not the terrain moved from a 0–450 m world to a
  // 1140–1860 m one and the atmosphere silently kept running at ~4% of its
  // authored density (measured: 3.6% haze on a 3 km ridge, i.e. none).
  // -------------------------------------------------------------------------
  const tBounds = (ctx.terrain && ctx.terrain.bounds) || null;
  const WORLD_MIN_Y = Number.isFinite(tBounds && tBounds.minY) ? tBounds.minY : 0;
  const WORLD_MAX_Y = Number.isFinite(tBounds && tBounds.maxY) ? tBounds.maxY : WORLD_MIN_Y + 720;
  const WORLD_RELIEF = Math.max(WORLD_MAX_Y - WORLD_MIN_Y, 50);

  // Cumulus base has to sit above the summit or the peak punches through the
  // ceiling — and the cheap tier's `rd.y <= 0.012` early-out means cloud can
  // never be generated at or below eye level, so a base below the summit is
  // simply a hole. 700 m of clearance is defensible alpine cumulus.
  const CLOUD_BOTTOM = Math.max(2600, WORLD_MAX_Y + 700);
  const CLOUD_TOP = CLOUD_BOTTOM + 1700;
  const PLANET_RADIUS = 700000;   // deliberately smaller than Earth: pulls the
                                  // cloud horizon in to ~49 km so the layer
                                  // visibly converges instead of running flat
  const MAX_MARCH = 24000;

  // Height fog. The reference height is the valley floor of the *actual* world;
  // the e-folding height is set against the *actual* relief (alpine haze over
  // 720 m of relief thins with a scale height around 1 km, not 360 m — at 360 m
  // the summit would sit at 13% of the valley's density, which is smoke, not
  // air); and the density is *solved*, not guessed, so that a representative
  // mid-mountain camera sees a 3 km ridge at 58% haze.
  const FOG_REF_HEIGHT = WORLD_MIN_Y;
  const FOG_SCALE_HEIGHT = clamp(WORLD_RELIEF * 1.5, 900, 1200);
  const FOG_START = 6;            // metres — matches fogParams[17] below

  // Calibration point: eye at mid-relief, ridge at 3 km, 58% haze fraction.
  // Solving  1 - exp( -D * exp( -(y-y0)/H ) * (d - start) )  =  f  for D.
  const FOG_CAL_HEIGHT = WORLD_MIN_Y + 0.5 * WORLD_RELIEF;
  const FOG_CAL_DIST = 3000;
  const FOG_CAL_FRACTION = 0.58;
  const FOG_DENSITY = -Math.log(1 - FOG_CAL_FRACTION) /
    (Math.exp(-(FOG_CAL_HEIGHT - FOG_REF_HEIGHT) / FOG_SCALE_HEIGHT) * (FOG_CAL_DIST - FOG_START));
  /** The same density as actually experienced at the calibration altitude. */
  const FOG_DENSITY_EYE = -Math.log(1 - FOG_CAL_FRACTION) / (FOG_CAL_DIST - FOG_START);
  // => 4.04e-4 /m for the shipping world. Verified across the full altitude
  // band: 500 m reads 10–18% and 3 km reads 46–70% from summit to valley.

  // Sky irradiance as a fraction of the sun's irradiance on flat ground. Real
  // clear-sky diffuse runs about 0.2–0.35 of direct-horizontal; a touch above
  // that keeps shadows readable after tone mapping without going milky. Drives
  // scene.environmentIntensity — see applySunAngles.
  const SKY_TO_SUN_RATIO = 0.34;

  // Albedo of sunlit alpine terrain, averaged over dirt, scree, grass and the
  // snow patches above the snowline. This is the source of the world's ground
  // bounce: it colours the PMREM's lower hemisphere, the hemisphere light's
  // ground term and the sky dome's below-horizon band, so all three agree.
  const GROUND_ALBEDO = new THREE.Color().setRGB(0.50, 0.44, 0.36, THREE.SRGBColorSpace);
  // ...of which only some is actually facing the sun at any moment; the rest is
  // shadowed, tilted away, or under canopy. Keeps the bounce honest.
  const GROUND_SUNLIT_FRACTION = 0.65;

  const CLOUD_TIERS = {
    // `steps` is the primary cloud raymarch length. High was 52, which is
    // ~1.6 ms of a 23 ms frame for a layer that is 3–5 km away and mostly
    // occluded by the mountain; 32 keeps the silhouette and the silver lining.
    low: { volumetric: false, steps: 1, shape: 40, detail: 24, weather: 256 },
    medium: { volumetric: true, steps: 24, shape: 48, detail: 24, weather: 384 },
    high: { volumetric: true, steps: 32, shape: 64, detail: 32, weather: 512 },
    ultra: { volumetric: true, steps: 56, shape: 64, detail: 32, weather: 512 },
  };
  const tier = CLOUD_TIERS[quality] || CLOUD_TIERS.high;
  let wantVolumetric = settings.volumetricClouds !== false && tier.volumetric;

  // -------------------------------------------------------------------------
  // Procedural cloud textures (deterministic from ctx.seed)
  // -------------------------------------------------------------------------
  const cloudSeed = subSeed(ctx.seed ?? 0, 'clouds');
  const shapeTex = buildCloudShapeTexture(tier.shape, cloudSeed);
  const detailTex = buildCloudDetailTexture(tier.detail, (cloudSeed ^ 0x9e37) >>> 0);
  const weather = buildWeatherTexture(tier.weather, (cloudSeed ^ 0x5bf0) >>> 0);

  // -------------------------------------------------------------------------
  // Sky dome. Geometry/uniform scaffolding comes from three's Sky.js; both
  // shaders are replaced with the tuned, cloud-capable pair above.
  // -------------------------------------------------------------------------
  const baseSky = new ThreeSky();
  const skyGeometry = baseSky.geometry;
  baseSky.material.dispose();

  const uniforms = {
    // Preetham. Tuned for crisp alpine air: the stock turbidity 2 / rayleigh 1
    // gives the washed-out "demo sky" look, and turbidity above ~5 turns the
    // whole dome into milk.
    turbidity: { value: 3.82 },
    rayleigh: { value: 1.84 },
    mieCoefficient: { value: 0.0045 },
    mieDirectionalG: { value: 0.823 },
    sunPosition: { value: new THREE.Vector3(0, 1, 0) },
    up: { value: new THREE.Vector3(0, 1, 0) },

    uSkyExposure: { value: 0.019 },
    uSkyShoulder: { value: new THREE.Vector2(SKY_SHOULDER_KNEE, SKY_SHOULDER_POWER) },
    // Half-angle in radians. The true disc is 0.00465; 0.0062 is a ~30% cheat
    // so it survives resolve and gives bloom/god rays real structure to key on.
    uSunDiscSize: { value: 0.0062 },
    // Tuned so the disc lands around 600–2000 in the HDR buffer: a ~1000:1
    // ratio over the sky, which is a strong but sane bloom source.
    uSunDiscIntensity: { value: 240.0 },
    uShowSunDisc: { value: 1.0 },
    uNight: { value: 0.0 },
    uGroundHaze: { value: new THREE.Color(0x2a2b28).convertSRGBToLinear() },
    uGroundDeep: { value: new THREE.Color(0x2a2b28).convertSRGBToLinear() },
    uHorizonColor: { value: new THREE.Color(0xa8c3e0).convertSRGBToLinear() },

    uShapeTex: { value: shapeTex },
    uDetailTex: { value: detailTex },
    uWeatherTex: { value: weather.texture },

    uRayOrigin: { value: new THREE.Vector3(0, 320, 0) },
    uSunColor: { value: new THREE.Color(1, 0.94, 0.86) },
    uCloudAmbientTop: { value: new THREE.Color(0.30, 0.40, 0.55) },
    uCloudAmbientBottom: { value: new THREE.Color(0.11, 0.14, 0.20) },

    uCloudLayer: { value: new THREE.Vector4(CLOUD_BOTTOM, CLOUD_TOP, PLANET_RADIUS, MAX_MARCH) },
    // shapeScale tiles the volume every ~3.1 km => ~520 m cloud cells;
    // detailScale every ~340 m => ~55 m wisps.
    uCloudShape: { value: new THREE.Vector4(1 / 3100, 1 / 340, 1 / 21000, 0.42) },
    // coverage, density multiplier, extinction per metre, light-march step.
    // R3-E2: coverage was 0.55. Because `cover` is biased by `uCloudCover.x - 0.5`
    // there was nowhere in the sky with *zero* coverage, and with an extinction of
    // 0.048/m a density of only 0.006 reaches optical depth 1 over the 3.7 km an
    // upward ray spends inside the layer — so a veil that reads as nothing in the
    // density field still puts ~30% alpha of near-white in front of the blue. That
    // veil, not the atmosphere model, is what took the measured sky to luminance
    // 0.82 / saturation 0.22 (clear sky in the same frames measures 0.43 / 0.57,
    // which is already the right *colour*). 0.48 makes the bias negative, so the
    // sky has genuine holes in it and the Preetham blue is actually visible.
    // Extinction 0.048/m stays: it is the real value for cloud droplets.
    uCloudCover: { value: new THREE.Vector4(0.48, 1.15, 0.048, 42.0) },
    uCloudWind: { value: new THREE.Vector4(0, 0, 0, 0) },
  };

  const skyMaterial = new THREE.ShaderMaterial({
    name: 'DescentSky',
    uniforms,
    vertexShader: SKY_VERTEX,
    fragmentShader: buildSkyFragment(wantVolumetric ? tier.steps : 1, wantVolumetric),
    side: THREE.BackSide,
    depthWrite: false,
    depthTest: true,
    fog: false,
    toneMapped: true,
  });

  const skyMesh = new THREE.Mesh(skyGeometry, skyMaterial);
  skyMesh.name = 'sky';
  skyMesh.scale.setScalar(10000);
  skyMesh.frustumCulled = false;
  // Draw the dome last among opaques: early-Z then rejects every pixel the
  // mountain already covers, and the raymarch is far too expensive to overdraw.
  skyMesh.renderOrder = 100;
  skyMesh.matrixAutoUpdate = false;
  skyMesh.matrixWorldAutoUpdate = false;
  skyMesh.updateMatrix();
  skyMesh.matrixWorld.copy(skyMesh.matrix);

  // A second dome for the PMREM capture. It shares the *uniform object* with
  // the main dome (so every colour stays in lockstep for free) but compiles a
  // separate, deliberately non-volumetric program: a 256-cube irradiance probe
  // cannot resolve a single cloud wisp, and leaving the raymarch in means the
  // full 32-step march re-runs over six cube faces every time the sun moves.
  const envSkyMaterial = new THREE.ShaderMaterial({
    name: 'DescentSkyEnv',
    uniforms,                       // shared by reference, intentionally
    vertexShader: SKY_VERTEX,
    fragmentShader: buildSkyFragment(1, false),
    side: THREE.BackSide,
    depthWrite: false,
    depthTest: true,
    fog: false,
    toneMapped: true,
  });

  const envSkyMesh = new THREE.Mesh(skyGeometry, envSkyMaterial);
  envSkyMesh.scale.setScalar(10000);
  envSkyMesh.frustumCulled = false;
  envSkyMesh.renderOrder = 100;
  const envScene = new THREE.Scene();
  envScene.add(envSkyMesh);

  // -------------------------------------------------------------------------
  // The ground in the PMREM (RC-1). Without this the entire lower hemisphere of
  // the IBL is one flat constant, so there is no bounce light in the renderer
  // at all: the underside of every rock, log, berm, wheel and the rider's chin
  // is lit by a mint-coloured card. A bowl of concentric rings, spaced by
  // *elevation angle below the horizon* rather than by radius (a flat disc puts
  // 99% of its area within a degree of the horizon and so shades the whole
  // lower hemisphere with its rim colour), coloured on the CPU from the sunlit
  // terrain albedo hazing out into the same atmosphere the dome renders.
  // -------------------------------------------------------------------------
  const GROUND_BOWL_RINGS = 20;
  const GROUND_BOWL_SEGMENTS = 48;
  const GROUND_BOWL_DROP = 200;     // metres; only sets the scale, not the look
  const GROUND_BOWL_MIN_ELEV = 0.4 * DEG;

  /**
   * Elevation below the horizon, in radians, of ring r (r = 0 is the nadir).
   * Geometric spacing from 90 degrees down to the rim: the path length through
   * the haze goes as 1/sin(elevation), so equal steps in log-elevation are
   * equal steps in the thing that is actually changing. A linear or
   * polynomial ladder wastes half its rings on the last half-degree.
   */
  function groundBowlElev(r) {
    const t = r / GROUND_BOWL_RINGS;
    return (90 * DEG) * Math.pow(GROUND_BOWL_MIN_ELEV / (90 * DEG), t);
  }

  function buildGroundBowlGeometry() {
    const R = GROUND_BOWL_RINGS, S = GROUND_BOWL_SEGMENTS;
    const vertCount = 1 + R * S;
    const pos = new Float32Array(vertCount * 3);
    const col = new Float32Array(vertCount * 3);
    pos[0] = 0; pos[1] = -GROUND_BOWL_DROP; pos[2] = 0;
    for (let r = 1; r <= R; r++) {
      const radius = GROUND_BOWL_DROP / Math.tan(groundBowlElev(r));
      for (let s = 0; s < S; s++) {
        const a = (s / S) * TAU;
        const o = (1 + (r - 1) * S + s) * 3;
        pos[o] = Math.cos(a) * radius;
        pos[o + 1] = -GROUND_BOWL_DROP;
        pos[o + 2] = Math.sin(a) * radius;
      }
    }
    const idx = [];
    for (let s = 0; s < S; s++) idx.push(0, 1 + ((s + 1) % S), 1 + s);
    for (let r = 1; r < R; r++) {
      const a0 = 1 + (r - 1) * S, b0 = 1 + r * S;
      for (let s = 0; s < S; s++) {
        const s1 = (s + 1) % S;
        idx.push(a0 + s, b0 + s1, b0 + s);
        idx.push(a0 + s, a0 + s1, b0 + s1);
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    geo.setIndex(idx);
    geo.boundingSphere = new THREE.Sphere(
      new THREE.Vector3(0, -GROUND_BOWL_DROP, 0),
      GROUND_BOWL_DROP / Math.tan(GROUND_BOWL_MIN_ELEV) * 1.01,
    );
    return geo;
  }

  const groundBowlGeo = buildGroundBowlGeometry();
  const groundBowlMat = new THREE.MeshBasicMaterial({
    name: 'DescentGroundBounce',
    vertexColors: true,
    side: THREE.DoubleSide,
    fog: false,
    toneMapped: true,
    // Ordering here is by renderOrder alone, not by depth. PMREMGenerator
    // renders the six cube faces with `autoClear = false` into one shared
    // attachment, so relying on the depth buffer to sort the bowl against a
    // dome whose vertex shader pins z to the far plane would be depending on
    // undefined initial renderbuffer contents. The bowl covers exactly the
    // region below the horizon and nothing else, so painting it over the dome
    // last is both correct and deterministic.
    depthTest: false,
    depthWrite: false,
  });
  const groundBowl = new THREE.Mesh(groundBowlGeo, groundBowlMat);
  groundBowl.frustumCulled = false;
  groundBowl.renderOrder = 200;    // after the dome (100)
  envScene.add(groundBowl);

  // Nominal eye height above the ground the bowl stands for. Only used to turn
  // a look-down angle into a path length through the haze, so it wants to be
  // the sort of height a rider actually looks down from — a third of the relief.
  const GROUND_EYE_HEIGHT = Math.max(0.35 * WORLD_RELIEF, 40);

  /**
   * Radiance of the ground seen `elevBelowRad` below the horizon, in the sky
   * dome's units. Near-vertical looks read as almost pure bounce; grazing looks
   * read as almost pure haze, using the same optical depth the terrain fog uses,
   * so the dome, the IBL and the fogged-out ridges all land on one colour.
   * Requires `_bounce` and `_hazeCol` to be current (applySunAngles sets them).
   */
  function groundRadianceAt(elevBelowRad, out) {
    const dist = GROUND_EYE_HEIGHT / Math.max(Math.sin(elevBelowRad), 1e-4);
    const haze = clamp01(1 - Math.exp(-FOG_DENSITY_EYE * dist));
    return out.copy(_bounce).lerp(_hazeCol, haze);
  }

  // -------------------------------------------------------------------------
  // Lighting
  // -------------------------------------------------------------------------
  const sunDirection = new THREE.Vector3(0, 1, 0);

  const sun = new THREE.DirectionalLight(0xffffff, 3.4);
  sun.name = 'sun';
  sun.castShadow = settings.shadows !== false;
  const initialMapSize = settings.shadowMapSize || 2048;
  sun.shadow.mapSize.set(initialMapSize, initialMapSize);
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 4000;
  // Both are recomputed per frame from the live texel size in
  // updateShadowFrustum(); these are only what the first compile sees.
  sun.shadow.bias = -0.00008;
  sun.shadow.normalBias = 0.0063;

  // The cool fill. This, not the key, is what stops shadowed rock crushing to
  // black and reading as CG: outdoors, shadow is lit by the *sky*, so it is blue.
  const hemi = new THREE.HemisphereLight(0x9dc0f0, 0x54452f, 0.42);
  hemi.name = 'skyFill';

  // A small omnidirectional bounce floor so no normal is ever mathematically
  // unlit — including normals facing away from both the sky and the
  // environment's dominant lobe, which on a course made of downward-tilted
  // faces is most of them.
  const ambient = new THREE.AmbientLight(0x3a4658, 0.08);
  ambient.name = 'bounceFill';

  if (scene) {
    scene.add(sun);
    scene.add(sun.target);
    scene.add(hemi);
    scene.add(ambient);
    scene.add(skyMesh);
    // scene.background is engine.js's fallback and is deliberately left alone:
    // the dome covers it completely and it costs one cheap fullscreen quad.
  }
  ctx.sun = sun;

  // -------------------------------------------------------------------------
  // Fog. A FogExp2 must exist so USE_FOG / FOG_EXP2 get defined on every
  // material; the patched chunk then takes over, and the FogExp2 values are
  // only the fallback for anything that never received hFogParams.
  // -------------------------------------------------------------------------
  const fogHorizon = new THREE.Color();
  const fogGround = new THREE.Color();
  const fogHigh = new THREE.Color();
  const fogDown = new THREE.Color();
  const fogInscatter = new THREE.Color();

  // R3-E3/E4 tunables for the two new fog bands. DOWN_HAZE_FRACTION is the ratio
  // of downward to upward airlight; 0.4–0.5 is what a dark forested valley under
  // clear alpine air actually measures. CLOUD_SHADOW_SCALE puts the lowest
  // frequency of the shadow field at a ~1.2 km wavelength and the highest at
  // ~620 m, which is honest cumulus spacing.
  const DOWN_HAZE_FRACTION = 0.45;
  const CLOUD_SHADOW_DEPTH = 0.40;
  const CLOUD_SHADOW_SCALE = 0.005;
  if (scene) scene.fog = new THREE.FogExp2(0x9fbcdd, 0.00042);

  fogParams[3] = FOG_DENSITY;              // [0].a
  fogParams[7] = 1 / FOG_SCALE_HEIGHT;     // [1].a
  fogParams[11] = 0.72;                    // [2].a — HG g for the haze lobe
  fogParams[15] = FOG_REF_HEIGHT;          // [3].a
  fogParams[16] = 1.0;                     // [4].x max opacity
  fogParams[17] = FOG_START;               // [4].y start distance (m)
  fogParams[18] = 2.6;                     // [4].z horizon blend rate
  fogParams[19] = 0.55;                    // [4].w in-scatter gain
  fogParams[20] = 1.0;                     // [5].x enabled
  fogParams[21] = 1.0;                     // [5].y assume sRGB-encoded output,
  fogParams[22] = 1.0;                     // [5].z and already tone mapped —
  fogParams[23] = renderer ? (renderer.toneMappingExposure || 1) : 1;
                                           //        all corrected on first draw

  // -------------------------------------------------------------------------
  // PMREM environment (IBL)
  // -------------------------------------------------------------------------
  let pmrem = null;
  let envRT = null;
  let envIntensity = 0.22;   // recomputed from measured sky irradiance below
  const lastEnvSunDir = new THREE.Vector3(0, -1, 0);
  let envDirty = true;
  let envCooldown = 0;

  function regenerateEnvironment() {
    envDirty = false;
    if (!renderer || !scene) return;
    if (!pmrem) pmrem = new THREE.PMREMGenerator(renderer);

    // The disc is a couple of texels wide in a 256 cube, so leaving it in only
    // produces a filtering artefact — the DirectionalLight *is* the sun for IBL.
    const prevDisc = uniforms.uShowSunDisc.value;
    uniforms.uShowSunDisc.value = 0.0;

    const prevRT = envRT;
    try {
      envRT = pmrem.fromScene(envScene, 0.02, 1, 60000, { size: 256 });
      scene.environment = envRT.texture;
      scene.environmentIntensity = envIntensity;
      if (prevRT) prevRT.dispose();
    } catch (err) {
      console.warn('[sky] PMREM generation failed', err);
      envRT = prevRT;
    }

    uniforms.uShowSunDisc.value = prevDisc;
    lastEnvSunDir.copy(sunDirection);
  }

  // -------------------------------------------------------------------------
  // Sun placement and every colour derived from it
  // -------------------------------------------------------------------------
  // Default mood: late-afternoon golden hour. setTimeOfDay(0.69) reproduces it
  // exactly, so the menu's time slider and the default agree.
  let timeOfDay = 0.69;
  let sunAzimuth = 248 * DEG;     // WSW: ahead of, and left of, a rider running
                                  // down -Z. Long cross-trail shadows, rim light
                                  // on the rider, and the sun near frame edge so
                                  // god rays have somewhere to come from without
                                  // blinding the player.
  let sunElevation = 19.2 * DEG;  // low enough for long shadows and warm light,
                                  // high enough that the trail is not in shade

  const sunColor = new THREE.Color();
  let daylight = 1;
  let baseSunIntensity = 3.4;

  function applySunAngles() {
    const ce = Math.cos(sunElevation);
    sunDirection.set(ce * Math.sin(sunAzimuth), Math.sin(sunElevation), ce * Math.cos(sunAzimuth));
    if (sunDirection.lengthSq() < 1e-8) sunDirection.set(0, 1, 0);
    sunDirection.normalize();
    uniforms.sunPosition.value.copy(sunDirection);

    const sinE = sunDirection.y;
    const elevDeg = Math.asin(clamp(sinE, -1, 1)) / DEG;
    daylight = smoothstep(-0.09, 0.26, sinE);
    // "dusk" is the low-sun character: 1 on the horizon, 0 once the sun is
    // properly up. Keyed on elevation in degrees, not sin(elev), because at 18
    // degrees sin() is already 0.31 and a sin-based ramp would treat the golden
    // hour as high noon.
    const dusk = 1 - smoothstep(0, 38, elevDeg);

    // --- key light -------------------------------------------------------
    kelvinToColor(sunTemperature(elevDeg * DEG), sunColor);
    sun.color.copy(sunColor);
    // ~3.5 at golden hour with toneMappingExposure 1.0 (CONTRACT §0 asks for
    // 2–5), tapering to a whisper of moonlight overnight.
    baseSunIntensity = 0.06 + 3.45 * daylight;
    sun.intensity = baseSunIntensity;

    // --- atmosphere ------------------------------------------------------
    // A low sun needs a longer optical path (more rayleigh, much more turbidity)
    // or it will not redden. But Preetham's in-scattering *grows* with rayleigh,
    // so the exposure has to come down at the same time or the whole dome goes
    // milk-white — which is exactly what the stock parameters look like.
    // Measured at 18 degrees these give: zenith ~0.21/0.50/0.76 sRGB after ACES,
    // sky 30 degrees up ~0.26/0.56/0.77, and a clipped hot core around the sun.
    //
    // R5-E2. R3's re-expose was in the right direction and did not go nearly far
    // enough, because it moved the dome's *scale* when the defect was its
    // *shape*. Measured on the r5 set: 12 of 16 shots clip 7–38% of their pixels
    // above L=250 and the sky's own saturation reads 0.035, yet the clear-blue
    // zenith the model produces was already correct at 0.54 luminance / 0.67
    // saturation. All of the white is horizon glare and circumsolar Mie:
    //
    //   region (sun at 19.2 deg)      linear   x sunlit white card (1.117)
    //   zenith                          0.200      0.18   <- correct
    //   5 deg horizon ring              1.195      1.07   <- 2x too bright
    //   20 deg from the sun            30.66      27.4    <- 27x too bright
    //
    // Two changes, in this order:
    //
    // (1) Cut the Mie load. Turbidity 2.20 -> 1.85 and mieCoefficient 0.0028 ->
    //     0.0019 is genuinely crisp air at 1500 m ASL, and Mie is *the*
    //     achromatic scatterer — it is simultaneously the thing making the sky
    //     too bright near the horizon and the thing making it colourless. This
    //     alone takes 20-deg-from-sun from 27x to about 8x and lifts saturation
    //     everywhere.
    // (2) Put the remaining range through the shoulder (SKY_SHOULDER_KNEE /
    //     _POWER, applied in both the dome and the CPU mirror). That lands the
    //     worst case at 1.05x the white card with its hue intact.
    //
    // The exposure then comes *up* slightly (0.0338 -> 0.0385 at the clear end),
    // because with the top end under control the blue can afford to sit where the
    // work order asked for it. CPU mirror at the shipping sun: zenith 0.577 L /
    // 0.637 sat, 45 deg 0.573 / 0.628, brightest sky pixel anywhere in the dome
    // 0.922 L — no clipping, and 0.146 saturation even in the glare band.
    // Nothing else in the frame moves: envIntensity is solved against measured
    // sky irradiance below, so the IBL, the ground bounce and every fill divide
    // the change straight back out and the sky-to-sun irradiance ratio stays
    // pinned at SKY_TO_SUN_RATIO (verified 0.340 at every elevation). The
    // shadow floor this round is credited with fixing does not move.
    const rayleigh = lerp(1.30, 2.30, dusk);
    const turbidity = lerp(1.85, 4.20, dusk);
    const mieCoefficient = lerp(0.0019, 0.0042, dusk);
    const mieG = lerp(0.77, 0.85, dusk);
    const skyExposure = lerp(0.0385, 0.0193, dusk);
    uniforms.rayleigh.value = rayleigh;
    uniforms.turbidity.value = turbidity;
    uniforms.mieCoefficient.value = mieCoefficient;
    uniforms.mieDirectionalG.value = mieG;
    uniforms.uSkyExposure.value = skyExposure;
    uniforms.uNight.value = smoothstep(0.08, -0.10, sinE);

    // Mirror the shader state on the CPU so fog and IBL can be sampled from it.
    atmosPrepare(rayleigh, turbidity, mieCoefficient, mieG, skyExposure,
      sunDirection.x, sunDirection.y, sunDirection.z);

    // --- fog / aerial perspective ----------------------------------------
    // Sampled from the atmosphere itself, so a ridge fading into haze fades
    // into precisely the colour of the sky standing behind it, at any hour.
    // Three bands: the bright horizon glare (what clouds and the far skyline
    // wash into), the aerial-perspective blue a few degrees up (what a ridge
    // 2–3 km away actually reads as), and the deeper sky overhead.
    atmosRingAverage(5 * DEG, fogHorizon);
    atmosRingAverage(19 * DEG, fogGround);
    atmosRingAverage(52 * DEG, fogHigh);
    // R5-E2/E3 — these ceilings were 2.4 / 1.8 / 1.8, which are 2.1x and 1.6x the
    // radiance of a sunlit white card under this key. A cap set above the
    // brightest legitimate object in the world is not a cap: it never engaged,
    // and the 5-degree ring went into the fog at 1.20 linear (1.07x the white
    // card), which is why every distant ridge and every far cloud resolved to the
    // same near-white and why the establishing shot has no black point. Reset to
    // real backstops, expressed against that same white card: 0.85 is 0.76x,
    // 0.70 is 0.63x, 0.50 is 0.45x. With the shoulder in front of them they are
    // slack at the shipping sun and only engage near noon.
    capColor(fogHorizon, 0.85);
    capColor(fogGround, 0.70);
    capColor(fogHigh, 0.50);

    // The solar in-scatter lobe: how much brighter the haze is looking into the
    // sun than away from it. The shader's phase term is normalised to 1 towards
    // the sun, so this is an absolute radiance gain, not an unbounded spike.
    const nsc = Math.cos(6 * DEG), nss = Math.sin(6 * DEG);
    const hl = Math.max(Math.hypot(sunDirection.x, sunDirection.z), 1e-4);
    atmosRadiance((sunDirection.x / hl) * nsc, nss, (sunDirection.z / hl) * nsc, _pOut);
    fogInscatter.setRGB(
      Math.max(_pOut[0] - fogGround.r, 0),
      Math.max(_pOut[1] - fogGround.g, 0),
      Math.max(_pOut[2] - fogGround.b, 0),
      THREE.LinearSRGBColorSpace,
    );
    // Cap by scaling, not by clamping each channel — clamping would flatten a
    // low sun's orange glare to neutral white exactly when it matters most.
    // R5-E2: was 2.2, against a term whose raw peak is now ~0.75 at the shipping
    // sun and ~1.8 at deep sunset. 1.4 leaves the golden-hour glow untouched and
    // still catches the near-horizon Mie spike that used to wash the sunward half
    // of every frame.
    capColor(fogInscatter, 1.4);

    fogParams[0] = fogGround.r; fogParams[1] = fogGround.g; fogParams[2] = fogGround.b;
    fogParams[4] = fogHigh.r; fogParams[5] = fogHigh.g; fogParams[6] = fogHigh.b;
    fogParams[8] = fogInscatter.r; fogParams[9] = fogInscatter.g; fogParams[10] = fogInscatter.b;
    fogParams[11] = lerp(0.55, 0.70, dusk);   // haze anisotropy: broad warm glow
    fogParams[12] = sunDirection.x; fogParams[13] = sunDirection.y; fogParams[14] = sunDirection.z;
    fogParams[19] = 0.30 + 0.28 * dusk * daylight;

    // R3-E3 — the fourth band: haze seen looking *down*. Two things are different
    // about a downward path and the shader used to model neither. (1) It ends on
    // the ground instead of running out to space, and (2) the air along it is lit
    // by a hemisphere that is roughly half dark terrain rather than all sky, so
    // its source function is both dimmer and less blue than the upward ring at the
    // same elevation. Using the 19-degree *upward* ring for every downward ray is
    // why r3_00 — an aerial establishing shot, three quarters of it downward rays
    // — has a p0.1 luminance of 0.24 and therefore no black point at all. Derived,
    // not authored: the ring colour, pulled 35% toward the ground-bounce hue at
    // constant luminance, then taken to 45% of that luminance.
    _colB.copy(GROUND_ALBEDO).multiply(sunColor);
    _colB.multiplyScalar(Math.max(lum(fogGround), 1e-4) / Math.max(lum(_colB), 1e-4));
    fogDown.copy(fogGround).lerp(_colB, 0.35).multiplyScalar(DOWN_HAZE_FRACTION);
    fogParams[24] = fogDown.r; fogParams[25] = fogDown.g; fogParams[26] = fogDown.b;
    fogParams[27] = 2.2;   // [6].a — reaches the down band by ~27 degrees below level

    // R3-E4 — cloud-shadow depth. Scales with how much cloud there actually is and
    // vanishes with the sun, so a clear sky or a night never gets phantom patches.
    fogParams[28] = CLOUD_SHADOW_DEPTH * clamp01(uniforms.uCloudCover.value.x * 1.75) * daylight;
    fogParams[29] = CLOUD_SHADOW_SCALE;

    // The fallback FogExp2 tracks the haze so the two can never disagree.
    if (scene && scene.fog) {
      _colA.copy(fogGround).lerp(fogHigh, 0.30);
      // FogExp2's colour is consumed after tone mapping on the direct-to-screen
      // path, so it is stored tone mapped; the shared chunk handles the HDR
      // path itself and never touches this value.
      scene.fog.color.setRGB(acesApprox(_colA.r), acesApprox(_colA.g), acesApprox(_colA.b), THREE.LinearSRGBColorSpace);
      // FogExp2 integrates density^2 * dist^2, not density * dist, so the
      // multiplier is what makes the fallback match the height fog at the same
      // 3 km calibration point rather than being an unrelated curve.
      scene.fog.density = FOG_DENSITY * 0.77;
    }
    uniforms.uHorizonColor.value.copy(fogHorizon).multiplyScalar(0.80);

    // --- image-based lighting strength -----------------------------------
    // Preetham's radiance scale has no relationship to the sun's intensity, so
    // left at 1.0 the environment map out-lights the sun by ~2x and every
    // shadow washes out. Integrate the sky irradiance and scale the environment
    // so it lands at a fixed fraction of the sun's irradiance on flat ground —
    // measured, not guessed, and it stays correct at every time of day.
    // (Computed *before* the fills below, because the ground-bounce derivation
    // has to convert between the sun's units and the dome's, and envIntensity
    // is exactly that conversion factor.)
    let skyE = atmosSkyIrradiance(_skyIrr);
    // Cloud cover brightens the real environment map above the clear-sky model.
    const cloudGain = 1 + 0.55 * clamp01(uniforms.uCloudCover.value.x) * (wantVolumetric ? 1 : 0.6);
    skyE *= cloudGain;
    const targetE = Math.max(0.02, SKY_TO_SUN_RATIO * baseSunIntensity * Math.max(sinE, 0.12));
    envIntensity = clamp(targetE / Math.max(skyE, 1e-4), 0.02, 4);
    if (scene) scene.environmentIntensity = envIntensity;

    // --- ground bounce ----------------------------------------------------
    // The world's indirect term. Radiance leaving a sunlit patch of mountain,
    // expressed in the sky dome's own radiance units so it can be handed
    // straight to the PMREM ground bowl and to the dome's below-horizon band:
    //
    //   L_ground = albedo * ( E_sun_horizontal + E_sky ) / PI
    //
    // E_sky already arrives in dome units (_skyIrr); the sun's irradiance is in
    // light units, so it is divided by envIntensity — the measured conversion
    // between the two. Nothing here is a hand-picked orange: at golden hour the
    // bounce is warm because the *sun* is, and at noon it is nearly neutral.
    const eSunHoriz = baseSunIntensity * Math.max(sinE, 0);
    _bounce.copy(sunColor).multiplyScalar(eSunHoriz / Math.max(envIntensity, 1e-4));
    _bounce.r += _skyIrr.r * cloudGain;
    _bounce.g += _skyIrr.g * cloudGain;
    _bounce.b += _skyIrr.b * cloudGain;
    _bounce.multiply(GROUND_ALBEDO).multiplyScalar(GROUND_SUNLIT_FRACTION / Math.PI);

    // Looking down at a shallow angle you are not looking at ground, you are
    // looking through kilometres of haze at it. Same fog model the terrain
    // uses, so the dome's below-horizon band and the far ridges in front of it
    // land on the same colour instead of meeting at a step.
    _hazeCol.copy(fogHorizon).multiplyScalar(0.52);

    // The two ends of the dome's 15-degree below-horizon ramp (P0-d). The
    // shader interpolates between them with smoothstep(0.012, 0.26, -dir.y),
    // and 0.26 rad is 15 degrees — so these are sampled at exactly that.
    groundRadianceAt(0.7 * DEG, uniforms.uGroundHaze.value);
    groundRadianceAt(15 * DEG, uniforms.uGroundDeep.value);

    // ...and the same function, evaluated per ring, colours the PMREM bowl.
    {
      const colAttr = groundBowlGeo.getAttribute('color');
      const arr = colAttr.array;
      groundRadianceAt(90 * DEG, _colC);
      arr[0] = _colC.r; arr[1] = _colC.g; arr[2] = _colC.b;
      for (let r = 1; r <= GROUND_BOWL_RINGS; r++) {
        groundRadianceAt(groundBowlElev(r), _colC);
        for (let s = 0; s < GROUND_BOWL_SEGMENTS; s++) {
          const o = (1 + (r - 1) * GROUND_BOWL_SEGMENTS + s) * 3;
          arr[o] = _colC.r; arr[o + 1] = _colC.g; arr[o + 2] = _colC.b;
        }
      }
      colAttr.needsUpdate = true;
    }

    // --- sky fill --------------------------------------------------------
    // Hemisphere colour comes from the sky itself (upper) and the ground bounce
    // (lower). The PMREM environment is the physically-integrated version of
    // both; this exists so that non-PBR materials, and normals facing away from
    // the environment's dominant lobe, still read as lit — and so that there is
    // a real indirect term even on the frame before the first IBL lands.
    _colA.copy(fogHigh);
    const hMax = Math.max(_colA.r, _colA.g, _colA.b, 1e-4);
    hemi.color.copy(_colA).multiplyScalar(1 / hMax);
    // Irradiance is always less saturated than the radiance it integrates, so
    // pull the normalised sky hue partway back to white. This used to be 0.34,
    // which bleached the only blue in the shadows out of the frame; 0.12 keeps
    // the fill unmistakably sky-coloured while still being flatter than the
    // radiance it came from.
    hemi.color.lerp(_colB.setRGB(1, 1, 1, THREE.LinearSRGBColorSpace), 0.12);

    // Ground term: the bounce hue (albedo x sun colour), held to ~15% of the
    // sky term's luminance. The bounce's real magnitude is carried by the PMREM
    // bowl above — this is the wrap-around that keeps a downward-facing normal
    // from being lit by nothing at all, and it must stay small or the course's
    // downward-tilted faces go orange, which is precisely the defect being
    // fixed (measured B-R of -10.2 in an open shadow).
    _colC.copy(GROUND_ALBEDO).multiply(sunColor);
    const bounceLum = Math.max(lum(_colC), 1e-4);
    const groundWeight = 0.15 * clamp01(eSunHoriz / Math.max(baseSunIntensity * 0.35, 1e-4));
    hemi.groundColor.copy(_colC).multiplyScalar(groundWeight * lum(hemi.color) / bounceLum);

    // 0.14 at noon was the single most damaging number in the renderer: a fill
    // of 0.14 against a key of 3.51 is a 25:1 lighting ratio, which is a studio
    // portrait, not a mountainside under an open sky. 0.42 puts the total
    // indirect term (hemisphere + IBL + ambient) at ~0.53 of the sun's
    // horizontal irradiance, which under 55% cloud is honest.
    hemi.intensity = 0.05 + 0.37 * daylight;

    // Flat bounce floor, tinted with the sky rather than an arbitrary slate.
    ambient.color.copy(fogHigh).multiplyScalar(1 / hMax);
    ambient.intensity = 0.012 + 0.068 * daylight;

    // --- cloud lighting --------------------------------------------------
    // Cloud radiance lives in the same arbitrary units as the sky, so it is
    // derived from the sky rather than from the light's intensity. The 2.6x
    // gain is the standard fudge for single-scattering under-predicting how
    // bright a sunlit cumulus really is; the ambient terms come straight from
    // the sampled sky so the shadowed underside is the colour of the sky it
    // is being lit by.
    // R3-E2 took the gain 2.5 -> 1.19. R5-E2 takes it to 0.62 and re-bases the
    // exposure coupling on the new skyExposure. The reason is arithmetic: at 1.19
    // a lit cloud top lands at 1.34 linear, which is 1.20x a fully sunlit white
    // card — a cumulus is not brighter than a white card in the same sun, and
    // above 1.0 ACES's shoulder flattens the lit/shadowed separation this module
    // already computes (the mix(0.30, 1.0, hf) in marchClouds) into one white
    // ceiling. Measured through ACES: at the old gain, top 0.935 / base 0.747 =
    // 0.188 of separation; at 0.62, top 0.871 / base 0.636 = **0.235**, inside
    // the 0.20–0.30 the work order asks for, with the base now sitting *below*
    // the zenith (0.577) as a real cloud base does. Sunlit tops still reach
    // 223/221/228, so nothing is dull — there is simply structure in them.
    uniforms.uSunColor.value.copy(sunColor)
      .multiplyScalar((0.048 + 0.62 * daylight) * (skyExposure / 0.0290));
    uniforms.uCloudAmbientTop.value.copy(fogHigh).multiplyScalar(0.90);
    uniforms.uCloudAmbientBottom.value.copy(fogGround).multiplyScalar(0.22);

    // Rebuild the IBL only when the sun has actually moved (~0.36 degrees).
    if (sunDirection.dot(lastEnvSunDir) < 0.99998) envDirty = true;
  }

  function setSunAngles(azimuthRad, elevationRad) {
    sunAzimuth = azimuthRad;
    sunElevation = clamp(elevationRad, -20 * DEG, 88 * DEG);
    applySunAngles();
  }

  /**
   * h01 in [0,1) across the day: 0.25 = sunrise, 0.5 = solar noon, 0.75 = sunset.
   * Azimuth runs due east at sunrise, due south at noon and due west at sunset —
   * a mid-latitude day — so the *character* of the shadows changes across the
   * day and not merely their angle. Peak elevation 62 degrees.
   */
  function setTimeOfDay(h01) {
    timeOfDay = h01 - Math.floor(h01);
    const dayAngle = (timeOfDay - 0.25) * TAU;
    const maxElevation = 62 * DEG;
    const elev = Math.asin(clamp(Math.sin(dayAngle) * Math.sin(maxElevation), -1, 1));
    const az = (90 + 360 * (timeOfDay - 0.25)) * DEG;
    setSunAngles(az, elev);
  }

  // -------------------------------------------------------------------------
  // Shadow rig — one cascade, fitted to a TEXEL BUDGET and texel-snapped.
  //
  // R3-E1. This used to fit the smallest sphere enclosing the [near, 150 m] slice
  // of the view frustum. That sphere has radius ~1.23 x range for a 62-degree 16:9
  // frustum, so the ortho spanned ~370 m and a 2048 map gave a 0.18 m texel. A
  // downhill bike is 30 mm frame tubes, 20 mm stanchions, 60 mm tyre casings; a
  // rider is 60 mm forearms; a tape post is 45 mm. Every one of them is under a
  // third of a texel, so the *hero of the game physically could not cast a
  // shadow*, and neither could anything else at human scale — the review found no
  // legible contact shadow anywhere in sixteen frames, on any object.
  //
  // The fix inverts the derivation. The texel size is the thing that decides
  // whether the game has contact shadows, so the texel size is what is authored
  // and the covered radius is what falls out of it:
  //
  //     radius = 0.5 * texelTarget * mapSize
  //
  // At high (2048, 35 mm) that is a 36 m radius disc centred ~20 m ahead of the
  // eye: shadows from 16 m behind the camera to 56 m in front, laterally +/-36 m,
  // at a 35 mm texel. A 60 mm forearm is now 1.7 texels and a 740 mm wheel is 21,
  // so the hero casts a real, readable shadow with structure in it. It is a 5.1x
  // improvement in texel density bought with a 2.6x reduction in reach — and the
  // reach it gives up was carrying nothing, because at 0.18 m nothing in it
  // resolved. Raise ctx.settings.shadowMapSize and the radius grows with it at
  // constant sharpness; drop it for a weak GPU and the radius shrinks rather than
  // the shadows dissolving. Nothing here overrides the authored map size.
  //
  // DEVIATION FROM THE WORK ORDER, stated plainly. E1 asks for a *dedicated tight
  // slice* — a second shadow-casting light fitted to the bike+rider box at ~4 mm
  // texels, terrain cascade untouched. I did not do that, and the reason is not
  // effort: in stock three a second DirectionalLight also *lights* the scene, so
  // it must split the key, and the CONTRACT-NOTE at the head of this file records
  // why splitting the key is a worse defect in three files this module does not
  // own. The one arrangement that adds a shadow without adding light is a
  // subtractive triple (+K tight-shadowed, +K wide-shadowed, -K unshadowed, whose
  // sum is K*(shadowWide + shadowTight - 1)). I costed it and rejected it: when a
  // fragment is inside *both* shadows that expression is -1, i.e. it subtracts the
  // entire key from the ambient term and crushes to black — and "hero rides
  // through tree shade" makes that the common case, which would re-open the r2 P0
  // this round is credited with fixing. A real second cascade needs a renderer-
  // wide material patch, which is explicitly cross-module. So: same objective,
  // one light, achieved by spending resolution where the hero is instead of
  // spreading it over 150 m that resolved nothing.
  // -------------------------------------------------------------------------

  // Metres per shadow texel, per tier. 0.035 at high resolves a forearm, a tyre,
  // a tape post and the rider's silhouette; it does not resolve a 30 mm frame tube
  // cleanly (that is ~1 texel and will read as a broken line), which is the honest
  // limit of a single 2048 cascade. Ultra's 4096 map buys 0.022 m at a 45 m radius.
  const SHADOW_TEXEL = { low: 0.055, medium: 0.045, high: 0.035, ultra: 0.022 };
  // Guard rails so a hand-edited shadowMapSize cannot produce a useless fit.
  const SHADOW_RADIUS_MIN = 18;
  const SHADOW_RADIUS_MAX = 620;
  // The fit is a disc centred this fraction of a radius ahead of the eye along the
  // view direction, so roughly a third of it sits behind the camera (where the
  // rider's own shadow is thrown from, at a low sun) and two thirds ahead.
  const SHADOW_FIT_AHEAD = 0.55;

  function baseShadowRadius() {
    const texel = SHADOW_TEXEL[ctx.quality] || SHADOW_TEXEL.high;
    const mapSize = sun.shadow.mapSize.x || 2048;
    return clamp(0.5 * texel * mapSize, SHADOW_RADIUS_MIN, SHADOW_RADIUS_MAX);
  }

  let shadowRadius = baseShadowRadius();
  /** Caster headroom up-sun. Must cover the relief across the fit plus a tree. */
  function shadowBackoffFor(radius) { return clamp(radius * 2.5, 120, 900); }

  // Adaptive reach — the second-cascade substitute (see the CONTRACT-NOTE at
  // the head of the file). The tight fit is correct when the eye is a couple
  // of metres off the deck and contact shadows are the whole game; it is
  // useless when the camera is 400 m up looking at a whole mountain, which is
  // why an aerial establishing shot came back with no shadows in it at all.
  // Grow the fit with the camera's height above the terrain, quantised so the
  // texel size (and therefore the snap grid) is stable within a band instead of
  // creeping every frame.
  // No chase-camera framing puts the eye more than ~20 m off the deck, so the
  // deadband below guarantees gameplay never leaves the tight fit; only an
  // aerial or cinematic framing can grow it.
  // R3-E1: the growth is now capped in absolute metres rather than by a multiple
  // of the base. The old cap took an aerial to a ~1.5 km radius and a 1.4 m texel,
  // at which a conifer crown is one texel and the forest floor in r3_00 receives
  // no shadow at all — which is a large part of why that frame has no black point.
  // 620 m still fits most of the massif for a ridge self-shadow while holding the
  // texel at 0.6 m, where a 25 m conifer's low-sun shadow is a legible 12 texels.
  const SHADOW_HEIGHT_DEADBAND = 25;   // metres of clearance before it grows
  const SHADOW_HEIGHT_GAIN = 3.2;      // metres of radius per metre above that
  const SHADOW_RADIUS_STEP = 1.25;     // quantisation ladder
  let adaptiveRadius = shadowRadius;

  function shadowRadiusFor(camera) {
    let want = shadowRadius;
    const terrain = ctx.terrain;
    if (terrain && typeof terrain.sampleHeight === 'function') {
      const gy = terrain.sampleHeight(camera.position.x, camera.position.z);
      if (Number.isFinite(gy)) {
        const above = Math.max(camera.position.y - gy - SHADOW_HEIGHT_DEADBAND, 0);
        want = shadowRadius + above * SHADOW_HEIGHT_GAIN;
      }
    }
    want = clamp(want, shadowRadius, Math.max(shadowRadius, SHADOW_RADIUS_MAX));
    // Quantise on a 1.25x ladder anchored at the base radius.
    const steps = Math.round(Math.log(want / shadowRadius) / Math.log(SHADOW_RADIUS_STEP));
    return shadowRadius * Math.pow(SHADOW_RADIUS_STEP, steps);
  }

  function updateShadowFrustum(camera) {
    if (!sun.castShadow || !camera || !camera.isPerspectiveCamera) return;

    adaptiveRadius = shadowRadiusFor(camera);
    const radius = Math.max(adaptiveRadius, 1);
    const backoff = shadowBackoffFor(radius);

    // The fit is an explicit disc, not a frustum-slice sphere: what matters is
    // how many metres one texel covers, and a frustum fit hands that number to
    // the field of view instead of to the artist. Centre it ahead of the eye
    // along the view direction so the coverage lands where the player is looking;
    // a sphere is rotation invariant, so the fit does not change as the camera
    // turns — that, plus texel snapping below, is what kills shadow swim.
    camera.getWorldDirection(_camFwd);
    _center.copy(camera.position).addScaledVector(_camFwd, radius * SHADOW_FIT_AHEAD);

    // three's LightShadow re-derives the shadow camera basis with lookAt() and
    // the default (0,1,0) up, so the snap must happen in exactly that basis or
    // it will not line up with the texel grid the map is actually rendered on.
    _lightRight.copy(_worldUp).cross(sunDirection);
    if (_lightRight.lengthSq() < 1e-6) _lightRight.set(1, 0, 0);
    _lightRight.normalize();
    _lightUp.copy(sunDirection).cross(_lightRight).normalize();

    const mapSize = sun.shadow.mapSize.x || 2048;
    const texel = (2 * radius) / mapSize;
    const cx = Math.round(_center.dot(_lightRight) / texel) * texel;
    const cy = Math.round(_center.dot(_lightUp) / texel) * texel;
    const cz = _center.dot(sunDirection);

    _v1.set(0, 0, 0)
      .addScaledVector(_lightRight, cx)
      .addScaledVector(_lightUp, cy)
      .addScaledVector(sunDirection, cz);

    sun.target.position.copy(_v1);
    sun.position.copy(_v1).addScaledVector(sunDirection, radius + backoff);
    sun.updateMatrixWorld();
    sun.target.updateMatrixWorld();

    const cam = sun.shadow.camera;
    cam.left = -radius; cam.right = radius;
    cam.top = radius; cam.bottom = -radius;
    cam.near = 1;
    cam.far = 2 * radius + backoff + 40;
    cam.updateProjectionMatrix();

    // Bias. Acne is a *ratio* phenomenon — it depends on bias measured in texels,
    // not in metres — so both terms are kept at the same fraction of a texel the
    // r2 fit proved acne-free (0.18 texel of normal offset, 1.0 texel of depth
    // offset) and simply follow the texel down. At high that is 6.3 mm of normal
    // bias and 35 mm of depth bias, against 38 mm and 180 mm before. Both clamps
    // had floors authored for a 0.18 m texel and would otherwise have silently
    // held the bias at the old absolute size while the texel shrank by 5x, which
    // is exactly the failure mode E1 diagnoses: bias larger than the caster.
    const depthRange = cam.far - cam.near;
    sun.shadow.bias = -clamp(texel / depthRange, 1.5e-5, 0.00045);
    sun.shadow.normalBias = clamp(texel * 0.18, 0.002, 0.12);
  }

  function applyShadowSettings() {
    const size = Math.max(256, Math.min(8192, (settings.shadowMapSize | 0) || 2048));
    if (sun.shadow.mapSize.x !== size) {
      sun.shadow.mapSize.set(size, size);
      if (sun.shadow.map) { sun.shadow.map.dispose(); sun.shadow.map = null; }
    }
    sun.castShadow = settings.shadows !== false;
    // The fit follows the map size, so a resolution change re-derives the radius
    // at constant sharpness rather than changing how crisp the shadows are.
    shadowRadius = baseShadowRadius();
    adaptiveRadius = Math.max(adaptiveRadius, shadowRadius);
  }

  // -------------------------------------------------------------------------
  // Wind + cloud animation
  // -------------------------------------------------------------------------
  const WIND_X = 6.2, WIND_Z = 2.4;    // m/s at cloud altitude
  const shapeOffset = { x: rng() * 8000, z: rng() * 8000 };
  const detailOffset = { x: rng() * 900, z: rng() * 900 };

  // R3-E4. Cloud shadows used to be a single scalar multiplied into the key,
  // sampled from the weather field above the camera. That is not a cloud shadow —
  // it is a global exposure wobble, and it produced exactly what the review found:
  // sixteen frames under visibly broken cumulus with one unbroken key across the
  // whole massif. The spatial field now lives in the fog chunk (hfog_cloudShade),
  // which is the only place in this project that reaches every lit material
  // without patching materials this module does not own — the shared Float32Array
  // uniform travels through UniformsUtils.clone by reference, and a texture cannot.
  // This is the CPU mirror of exactly that function, so `sky.cloudShadow` reports
  // what the world is actually doing under the camera rather than a second,
  // independently-drifting number.
  let cloudShadow = 1;

  function cloudShadeAt(worldX, worldZ) {
    const depth = fogParams[28];
    if (depth <= 0 || sunDirection.y < 0.06) return 1;
    const reach = 900 / sunDirection.y;
    const px = (worldX + sunDirection.x * reach + fogParams[30]) * CLOUD_SHADOW_SCALE;
    const pz = (worldZ + sunDirection.z * reach + fogParams[31]) * CLOUD_SHADOW_SCALE;
    const m = (Math.sin(px * 1.000 + pz * 0.317)
      + Math.sin(px * -0.593 + pz * 0.812) * 0.86
      + Math.sin(px * 0.231 + pz * -1.371) * 0.63
      + Math.sin(px * 1.717 + pz * 1.093) * 0.41) * 0.3448;
    return 1 - depth * smoothstep(0.02, 0.62, m);
  }

  // -------------------------------------------------------------------------
  // Per-draw hook. Keeping the dome glued to the camera here (rather than in
  // update()) matters: chaseCamera moves the camera in lateUpdate, and every
  // lateUpdate runs after every update, so a position written in update() would
  // be a frame stale and the horizon would shear as the camera translated. The
  // shadow frustum is happy to lag a frame — texel snapping makes a half-metre
  // of lag on a 160 m sphere invisible — but the sky dome is not.
  // -------------------------------------------------------------------------
  skyMesh.onBeforeRender = function (rnd, sc, camera) {
    this.position.copy(camera.position);
    this.updateMatrix();
    this.matrixWorld.copy(this.matrix);
    uniforms.uRayOrigin.value.copy(camera.position);

    // Learn how this frame is being written. Straight to the framebuffer it has
    // already been tone mapped and sRGB-encoded by the time <fog_fragment> runs;
    // into the post-processing buffer it is still scene-referred linear. The
    // shared fog chunk needs to know which, and the sky is the one draw that can
    // observe it from inside the render.
    const rt = rnd.getRenderTarget();
    const toScreen = rt === null;
    fogParams[21] = (toScreen && rnd.outputColorSpace === THREE.SRGBColorSpace) ? 1 : 0;
    fogParams[22] = (toScreen && rnd.toneMapping !== THREE.NoToneMapping) ? 1 : 0;
    fogParams[23] = rnd.toneMappingExposure || 1;
  };

  // -------------------------------------------------------------------------
  // Wiring
  // -------------------------------------------------------------------------
  const offQuality = ctx.events ? ctx.events.on('quality:changed', () => {
    applyShadowSettings();
    const t = CLOUD_TIERS[ctx.quality] || CLOUD_TIERS.high;
    wantVolumetric = ctx.settings.volumetricClouds !== false && t.volumetric;
    // The 3D volumes are reused as-is — only the march (and therefore the
    // shader) changes with quality, so this costs a recompile and nothing more.
    skyMaterial.fragmentShader = buildSkyFragment(wantVolumetric ? t.steps : 1, wantVolumetric);
    skyMaterial.needsUpdate = true;
    // envSkyMaterial is deliberately left alone: the PMREM capture is always
    // non-volumetric, at every quality tier.
    applySunAngles();   // cloud-cover correction on the IBL depends on the tier
    envDirty = true;
  }) : null;

  setTimeOfDay(timeOfDay);

  const api = {
    // --- required surface (CONTRACT §6) ----------------------------------
    sunDirection,

    setTimeOfDay(h01) { setTimeOfDay(clamp01(h01)); },

    update(dt, c) {
      const camera = (c && c.camera) || ctx.camera;

      // --- cloud animation ----------------------------------------------
      shapeOffset.x += WIND_X * dt;
      shapeOffset.z += WIND_Z * dt;
      // The detail layer drifts faster and rises, so the volume boils rather
      // than sliding past as one rigid sheet.
      detailOffset.x += WIND_X * 1.9 * dt;
      detailOffset.z += (WIND_Z * 1.9 + 3.5) * dt;
      uniforms.uCloudWind.value.set(shapeOffset.x, shapeOffset.z, detailOffset.x, detailOffset.z);

      // --- shadow frustum -----------------------------------------------
      if (camera) updateShadowFrustum(camera);

      // --- drifting cloud shadows ----------------------------------------
      // The field itself is per-pixel, in the fog chunk. All that happens here is
      // that its drift is advanced with the same wind vector the cloud volume
      // uses, so the shadows on the ground and the layer overhead move together.
      // The key light is deliberately NOT dimmed any more — applySunAngles is the
      // only writer of sun.intensity now. A global multiply on ctx.sun.intensity
      // darkened the water glint, the forest and every particle (all three roll
      // their own sun terms off it) uniformly and everywhere at once, which is an
      // exposure wobble, not a cloud shadow.
      fogParams[30] = shapeOffset.x;
      fogParams[31] = shapeOffset.z;
      if (camera) {
        cloudShadow = damp(cloudShadow,
          cloudShadeAt(camera.position.x, camera.position.z), 2.5, dt);
      }

      // --- environment map ------------------------------------------------
      // Rate limited as well as change-gated: a menu slider scrubbing through a
      // whole day would otherwise ask for a cube capture on nearly every frame.
      envCooldown -= dt;
      if (envDirty && envCooldown <= 0) {
        regenerateEnvironment();
        envCooldown = 0.4;
      }
    },

    dispose() {
      if (offQuality) offQuality();
      if (scene) {
        scene.remove(sun); scene.remove(sun.target);
        scene.remove(hemi); scene.remove(ambient);
        scene.remove(skyMesh);
        if (envRT && scene.environment === envRT.texture) scene.environment = null;
        scene.fog = null;
      }
      envScene.remove(envSkyMesh);
      envScene.remove(groundBowl);
      groundBowlGeo.dispose();
      groundBowlMat.dispose();
      envSkyMaterial.dispose();
      skyMaterial.dispose();
      skyGeometry.dispose();
      shapeTex.dispose(); detailTex.dispose(); weather.texture.dispose();
      if (sun.shadow.map) { sun.shadow.map.dispose(); sun.shadow.map = null; }
      if (envRT) { envRT.dispose(); envRT = null; }
      if (pmrem) { pmrem.dispose(); pmrem = null; }
      fogParams[20] = 0;   // disable the height-fog chunk cleanly
    },

    // --- extras other modules may find useful ----------------------------
    sun, hemi, ambient,
    mesh: skyMesh,
    material: skyMaterial,
    uniforms,
    /** Merge into a custom ShaderMaterial's uniforms to receive the height fog. */
    fogUniform: { [HFOG]: { value: fogParams } },
    fogParams,

    // Both report metres of reach *ahead of the camera*, which is the contract
    // terrain.js and vegetation.js consume them under (they cull casters against
    // `dist < reach`). Internally the fit is a radius; the disc is centred
    // SHADOW_FIT_AHEAD radii forward, so the reach is (1 + AHEAD) radii.
    // R3-E1: at high this now reports ~56 m, not 150. That is not a regression —
    // 150 m was the extent of a fit whose texels resolved nothing, and both
    // consumers submitting fewer casters into a sharper map is the point.
    get shadowDistance() { return shadowRadius * (1 + SHADOW_FIT_AHEAD); },
    set shadowDistance(v) {
      shadowRadius = clamp(v, 20, 800) / (1 + SHADOW_FIT_AHEAD);
      adaptiveRadius = Math.max(adaptiveRadius, shadowRadius);
    },
    /** The reach the cascade is actually fitted to this frame (see CONTRACT-NOTE). */
    get shadowRange() { return adaptiveRadius * (1 + SHADOW_FIT_AHEAD); },
    /** Metres of world per shadow-map texel this frame — the sharpness number. */
    get shadowTexel() { return (2 * adaptiveRadius) / (sun.shadow.mapSize.x || 2048); },
    get timeOfDay() { return timeOfDay; },
    get environment() { return envRT ? envRT.texture : null; },
    /** Measured scale that puts the sky's IBL in step with the sun (see §6). */
    get environmentIntensity() { return envIntensity; },
    get sunIntensity() { return sun.intensity; },
    get fogGroundColor() { return fogGround; },
    get fogHighColor() { return fogHigh; },
    /**
     * 0..1 — the ground cloud-shadow transmittance under the camera, damped.
     * 1 = full sun. This is a CPU mirror of the per-pixel field in the fog chunk,
     * for anything that wants to react to being in cloud shade; it no longer
     * scales ctx.sun.intensity.
     */
    get cloudShadow() { return cloudShadow; },
    get volumetricClouds() { return wantVolumetric; },

    setSunAngles,

    /** 0..1 — how much of the sky the cloud layer covers. */
    setCloudCoverage(v) {
      uniforms.uCloudCover.value.x = clamp01(v);
      // Re-derive: the IBL's cloud gain and the ground cloud-shadow depth are both
      // functions of coverage, and neither may drift out of step with it.
      applySunAngles();
      envDirty = true;
    },

    /** Force the IBL to be rebuilt (e.g. after a large cloud change). */
    invalidateEnvironment() { envDirty = true; },

    resize() { /* nothing view-dependent */ },
  };

  // Build the first environment map immediately, so wave 5+ modules (bike,
  // rider, water) already have a real scene.environment when their materials
  // first compile rather than a frame of flat plastic.
  regenerateEnvironment();

  return api;
}

export default createSky;
