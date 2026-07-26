# DESCENT — Module Contract (authoritative)

This file is the **single source of truth** for module boundaries. Every agent implements
exactly one file (or the small file-set named in its brief) and must not edit any file it
does not own. If you need something another module owns, use the interface below — do not
reach into another module's internals and do not "fix" another module's file.

If the contract is genuinely wrong or missing something you need, implement against it
anyway using the closest sane fallback, and write a note at the top of YOUR file:
`// CONTRACT-NOTE: ...`. The integration pass collects those.

---

## 0. Global conventions

- **Units:** metres, seconds, radians, kilograms. `dt` is seconds (clamped to <= 1/20).
- **Axes:** Y-up, right-handed. World is a mountainside; the trail descends generally
  toward **-Z**. Gravity is `(0, -9.81, 0)`.
- **Scale reference:** rider+bike ~1.8 m tall, wheelbase 1.25 m, wheel radius 0.37 m.
  Trail tread width 1.2–3.0 m. Mountain drops ~420 m over ~2.6 km of trail.
- **Colour:** renderer uses `THREE.SRGBColorSpace` output and ACES-ish tonemapping (see
  `postfx`). **All colour textures/`Color`s authored as sRGB**; data maps (normal, rough,
  AO, height, splat) are **linear** (`NoColorSpace`). Get this wrong and everything
  looks washed out.
- **Lighting is physical:** `renderer.useLegacyLights = false` semantics. Sun intensity is
  in the 2–5 range with tonemapping exposure ~1.0. Do not use `MeshBasicMaterial` for
  anything that should receive light.
- **No external asset downloads.** No CDN fetches, no remote textures, no GLTF from the
  network. Everything is procedurally generated in code (canvas-generated textures,
  `DataTexture`, procedural geometry, shader-authored detail). This is a hard rule —
  the game must run fully offline from `npm run dev`.
- **Determinism:** all procedural generation takes a seed. Use the shared PRNG from
  `src/core/rng.js` (`makeRng(seed)` → `() => [0,1)`). Never `Math.random()` in
  world generation. Particles/effects may use `Math.random()`.
- **Imports:** `import * as THREE from 'three';` and addons from
  `'three/examples/jsm/...'`. Post-processing uses the `postprocessing` package
  (pmndrs) and `n8ao`. `simplex-noise` is available (`createNoise2D`, `createNoise3D`,
  each takes an optional rng function).
- **Performance budget:** target 60 fps at 1920×1080 on an Apple M-series GPU.
  Draw calls < 400. No per-frame allocation in `update()` — preallocate scratch
  vectors at module scope. This is not optional; a beautiful 20 fps game is a failure.
- **Style:** ES modules, no TypeScript. Explicit and readable over clever. Comment the
  *why* for any non-obvious maths.

---

## 1. Lifecycle

Every system module default-exports a factory:

```js
export function createXxx(ctx) { /* ... */ return system; }
```

The returned `system` object may implement any of:

```js
system.init()            // async allowed; called once, after ctx.terrain etc. exist per wave order
system.update(dt, ctx)   // called every frame, in wave order
system.lateUpdate(dt,ctx)// called after all update()s (camera, HUD use this)
system.resize(w, h)      // on viewport change
system.dispose()
```

All are optional. `main.js` calls whatever exists.

**Wave order** (build + update order, fixed):

1. `engine`, `input`
2. `terrain` (base heightfield)
3. `trail` (carves terrain, then terrain commits)
4. `sky`, `water`, `vegetation`
5. `collision`, `bike`
6. `bikeModel`, `rider`, `particles`
7. `chaseCamera`, `audio`
8. `gameplay`, `hud`, `menu`
9. `postfx` (last — needs final scene + camera)

---

## 2. `ctx` — shared state (owned by `src/core/ctx.js`, DO NOT EDIT)

```js
ctx = {
  container,          // HTMLElement
  renderer,           // THREE.WebGLRenderer      (set by engine)
  scene,              // THREE.Scene              (set by engine)
  camera,             // THREE.PerspectiveCamera  (set by engine)
  sun,                // THREE.DirectionalLight   (set by sky)
  clock,              // THREE.Clock
  time,               // seconds since start
  dt,                 // last frame delta, clamped
  frame,              // integer frame counter
  seed,               // number, world seed
  quality,            // 'low' | 'medium' | 'high' | 'ultra'  (see §9)
  settings,           // see §9
  input,              // InputState (see §7)
  events,             // { on(name, fn), off(name, fn), emit(name, payload) }
  // systems, assigned by main.js as they are created:
  terrain, trail, sky, water, vegetation, collision, bike, bikeModel,
  rider, particles, chaseCamera, audio, gameplay, hud, menu, postfx,
  debug: { enabled: false, log(...a){} },
}
```

---

## 3. Terrain — `src/world/terrain.js` (+ `src/world/terrainMaterial.js`)

The most depended-upon module. Heightfield-backed so physics can sample it cheaply.

```js
const terrain = createTerrain(ctx);

// --- build, two-phase (trail carves between the phases) ---
terrain.buildBase();            // generate heightfield + material ids. No meshes yet.
terrain.applyCarve(stamps);     // stamps: CarveStamp[] (see §4). Mutates heightfield.
terrain.commit();               // builds LOD chunk meshes, materials, adds to ctx.scene.

// --- queries (must be allocation-free and fast; called ~10x/frame by physics) ---
terrain.sampleHeight(x, z) -> number             // bilinear-interpolated world Y
terrain.sampleNormal(x, z, outVec3) -> Vector3   // writes into outVec3, returns it
terrain.sampleMaterial(x, z) -> number           // SurfaceId, see below
terrain.sampleSlope(x, z) -> number              // radians from horizontal
terrain.inBounds(x, z) -> boolean

terrain.bounds   // { minX, maxX, minZ, maxZ, minY, maxY }
terrain.size     // { width, depth }  metres
terrain.resolution // heightfield samples per axis
```

**SurfaceId enum** (export from terrain.js, used by physics/audio/particles):

```js
export const Surface = {
  DIRT: 0, LOAM: 1, ROCK: 2, GRAVEL: 3, GRASS: 4, ROOT: 5, MUD: 6, SNOW: 7,
};
```

Each surface drives grip, rolling resistance, particle colour, and audio — so
`sampleMaterial` must be meaningful, not a stub.

**Requirements:** multi-octave ridged+fbm noise mountain with believable geology
(ridgelines, gullies, cliff bands, talus slopes, a valley floor). Erosion pass
(hydraulic or a good approximation) so drainage lines look real. Chunked LOD
(quadtree or ring) with skirts to hide cracks. Frustum culling. Height range and
`size` big enough for a ~2.6 km trail. `resolution` fine enough that the trail tread
reads crisply (≤ 0.5 m/sample near the trail — a detail heightfield overlay near the
trail corridor is an acceptable way to get this without a giant global grid).

`terrainMaterial.js` exports `createTerrainMaterial(ctx, opts)` → a
`MeshStandardMaterial` (or `CustomShaderMaterial`-style patched via `onBeforeCompile`)
doing: triplanar projection on steep slopes, 4-way splat blend by height/slope/material
id with **height-map-based** blending (not linear lerp — linear blending looks muddy),
procedurally generated albedo/normal/roughness (canvas or noise-derived `DataTexture`),
detail-normal tiling at two scales to kill the "smooth clay" look, distance-based
tile-break to avoid visible repetition, and macro variation noise.

---

## 4. Trail — `src/world/trail.js`

Generates the downhill run and carves it into the terrain.

```js
const trail = createTrail(ctx);
trail.build(terrain);        // reads terrain.buildBase() output; computes centreline
trail.getCarveStamps() -> CarveStamp[]   // consumed by terrain.applyCarve()
trail.finalize(terrain);     // after terrain.commit(): build tread mesh/decals, gates
```

```js
// CarveStamp — how the trail reshapes the mountain
{
  x, z,             // world position of the stamp centre
  radius,           // metres of influence
  targetHeight,     // world Y the tread should sit at
  falloff,          // 0..1 shaping exponent
  material,         // SurfaceId to paint
  bank,             // radians of cross-slope (berms), + = right side high
  kind,             // 'tread' | 'berm' | 'lip' | 'landing' | 'drop' | 'rut'
}
```

Queries:

```js
trail.curve                 // THREE.CatmullRomCurve3, world space, descending
trail.length                // metres
trail.sampleAt(t)           // t in [0,1] -> { position, tangent, normal, binormal,
                            //   width, bank, gradient, feature|null }
trail.nearestT(position, hint) -> { t, lateral, distance, position, tangent }
                            // lateral: signed metres from centreline (+ = rider's right)
                            // `hint` is the previous t — use it to search locally, this is
                            // called every frame.
trail.widthAt(t) -> number
trail.features              // TrailFeature[]
trail.checkpoints           // [{ t, position, index }]  ~8 splits down the run
trail.startTransform        // { position: Vector3, quaternion: Quaternion }
```

```js
// TrailFeature
{ id, type, tStart, tEnd, position, params }
// type: 'berm' | 'jump' | 'drop' | 'gap' | 'rockGarden' | 'roots' | 'stepDown'
//     | 'chute' | 'creekCrossing' | 'wallride' | 'doubles' | 'rollers'
```

**Requirements:** a *ridden* trail, not a road. Consistent descent (avg 8–14%,
steepest chutes to 35%), rhythm — steep tech → flow section → jump line → rock garden →
final sprint. Berms banked correctly for entry speed. Jumps with real lip geometry and a
matching landing transition (a jump you can't land is a bug). Line choice: at least 4
places where an A-line and B-line diverge and rejoin. Fall-line sensible: the trail must
never climb for more than ~15 m. Deterministic from `ctx.seed`.

---

## 5. Physics — `src/physics/collision.js`, `src/physics/bike.js`

`collision.js` — terrain contact helpers, no bike knowledge:

```js
const collision = createCollision(ctx);
collision.probe(x, z, outHit) -> Hit        // { height, normal, material, slope }
collision.sweepWheel(center, radius, outHit) -> boolean
   // finds the deepest contact of a wheel disc against the heightfield,
   // sampling a few points around the contact patch so the wheel rides over
   // rocks/roots instead of snapping to a single sample.
collision.rayDown(origin, maxDist, outHit) -> boolean
```

`bike.js` — the heart of the game. Full rigid-body-ish bike with suspension.

```js
const bike = createBike(ctx);
bike.reset(transform);      // place at trail.startTransform or a checkpoint
bike.update(dt, ctx);

bike.state = {
  position: Vector3,        // rear-axle-ish chassis origin, world
  quaternion: Quaternion,   // chassis orientation
  velocity: Vector3,        // m/s world
  angularVelocity: Vector3,
  speed: number,            // m/s, scalar
  forward: Vector3, right: Vector3, up: Vector3,   // cached basis

  wheels: [front, rear],    // each: { position, contact:boolean, normal, material,
                            //   compression 0..1, slipRatio, slipAngle, load N,
                            //   spinRate rad/s }
  suspension: { fork: {travel, velocity, max}, shock: {travel, velocity, max} },

  airborne: boolean, airTime: number, lastAirTime: number,
  crashed: boolean, crashTimer: number,
  lean: number,             // radians, + = right
  steer: number,            // radians at the bar
  pumpImpulse: number,      // 0..1, from rider input over compressions
  brakeFront: number, brakeRear: number,  // 0..1
  pedalling: number,        // 0..1
  gForce: number,           // for camera shake + HUD
  surface: number,          // SurfaceId under rear wheel
  wheelie: number, endo: number,  // 0..1 how far past balance point
  distance: number,         // metres travelled along trail
  trailT: number,           // last known trail.nearestT t
  offTrail: boolean, lateral: number,
};
```

**Requirements — this must *feel* right, which matters more than being a textbook sim:**
- Two-wheel contact with independent spring/damper suspension (fork ~170 mm, shock
  ~165 mm), progressive spring curve, rebound damping slower than compression.
- Load transfer under braking/acceleration that actually changes grip at each wheel.
- Pacejka-ish (or a good tanh approximation) tyre model: longitudinal slip for
  brake lockup/skids, lateral slip for drifting and washing out the front.
- Countersteer/lean coupling — the bike leans into turns, and lean angle is limited by
  available grip; exceed it and the front washes.
- **Pumping:** pushing down through a compression and unweighting over a crest adds/
  removes energy. This is the soul of downhill riding and must be implemented.
- Air control: pitch/roll/yaw authority in the air, whips and scrubs, and a landing
  that punishes bad angle (case a landing = harsh compression or crash; land smooth
  and matched to the slope = carry speed).
- Manuals/wheelies and endos with a balance point.
- Crash detection: excessive impact normal velocity, landing beyond ~50° off-axis,
  hitting a wall. Crash → ragdoll-ish tumble, then respawn at last checkpoint.
- Rolling resistance and grip per `Surface` id.
- Fixed-step integration internally (e.g. 240 Hz substeps) so behaviour is
  frame-rate independent. Never let the bike tunnel through the terrain.

---

## 6. Visuals

### `src/world/sky.js`
Owns the sky, atmosphere, fog and **all scene lighting** (sets `ctx.sun`).
```js
sky.setTimeOfDay(hours01)   // 0..1 across the day
sky.sunDirection            // Vector3
sky.update(dt, ctx)
```
Requirements: physically-plausible sky (Rayleigh/Mie — `three/examples/jsm/objects/Sky.js`
is an acceptable base but must be tuned, not left at defaults), a real sun disc, volumetric
cloud layer (raymarched shader dome or well-authored procedural noise — no billboard PNGs),
height-based exponential fog with aerial perspective so distant ridges desaturate and blue
out, an HDR-ish environment (`PMREMGenerator` over the sky) driving `scene.environment`
for correct IBL, cascaded-ish shadow setup (tight sun frustum that follows the camera —
a single 4096 map covering 200 km² will look like mud), and a golden-hour default mood.

### `src/world/vegetation.js`
Instanced alpine forest + ground cover.
Requirements: `InstancedMesh` conifers with LOD + billboard-imposter far tier, procedurally
generated (trunk mesh + shell-textured canopy — no downloaded models), instanced grass with
vertex-shader wind, ferns/shrubs/deadfall/stumps, boulders and talus scattered by slope+
material rules, all placed from `terrain.sampleMaterial`/`sampleSlope` and **excluded from
the trail corridor** via `trail.nearestT`. Wind must be coherent across the scene (a shared
wind function, gusts). Shadow casting on the near tier only. Budget: forest reads as dense
(≥ 30 k instances) but stays inside the draw-call budget.

### `src/world/water.js`
Creek crossings, puddles, waterfalls off cliff bands.
Requirements: animated normal-mapped surface, screen-space-ish refraction/reflection
(a planar reflector or a cheap fake — must not look like blue plastic), depth-based
opacity and shoreline foam, splash hookup via `ctx.events.emit('water:splash', ...)`.

### `src/entities/bikeModel.js`
Procedural DH bike: frame (front triangle, swingarm, linkage), dual-crown fork with
visible stanchion travel, wheels with rim/spokes/hub/knobby tyre, cranks, chainring,
cassette, derailleur, disc rotors + calipers, bars/stem/grips, saddle. Metal/anodised/
rubber/carbon PBR materials.
```js
bikeModel.group        // THREE.Group added to scene
bikeModel.update(dt, ctx)  // reads ctx.bike.state: wheel spin, fork/shock travel,
                           //  steering, chainring+cassette rotation, brake caliper
```

### `src/entities/rider.js`
Rigged rider with procedural animation — no keyframe files.
```js
rider.group
rider.update(dt, ctx)
```
Requirements: skinned or segmented body (helmet, goggles, jersey, gloves, knee pads,
shoes), IK hands to the grips and feet to the pedals **at all times**, body English:
weight back on steeps, elbows out, standing attack position, leaning into berms,
pumping motion synced to `state.pumpImpulse`, tucking on straights, whips/scrubs in the
air, ragdoll on crash. Cloth/jersey secondary motion. The rider is on screen 100% of the
time — this module carries a huge share of the "AAA" verdict.

### `src/game/particles.js`
GPU-instanced/point-sprite effects.
Requirements: dirt roost off the rear tyre scaled by slip and surface, dust plumes,
rock chips, water splash, mud spray, leaves and pine needles kicked up, speed-lines/
grit in the air at high speed, impact bursts on landing, tyre tracks written as a
persistent decal/trail behind the bike. All procedurally textured. Pooled — zero
allocation per frame.

### `src/core/postfx.js`
The `postprocessing` (pmndrs) pipeline. **This is the single biggest lever on whether
the game reads as AAA.**
```js
const postfx = createPostFX(ctx);
postfx.render(dt)     // called by main.js instead of renderer.render()
postfx.resize(w, h)
postfx.setQuality(q)
```
Required chain (order matters): render → **N8AO** (or GTAO) ambient occlusion →
depth-of-field (subtle, focused on the rider, opening up at speed) → motion blur
(camera + per-object velocity) → bloom (physically-thresholded, not a white haze) →
god rays / volumetric light shafts through the trees → **ACES Filmic** or AgX
tonemapping → colour grading LUT (film-like: lifted greens in shadow, warm highlights)
→ subtle chromatic aberration + lens distortion at the edges → film grain →
vignette → **SMAA** (or TAA). Speed must drive the effects: FOV kick, radial blur,
and a slight desaturation as speed climbs.

### `src/camera/chaseCamera.js`
```js
chaseCamera.lateUpdate(dt, ctx)
chaseCamera.setMode(mode)   // 'chase' | 'chaseFar' | 'firstPerson' | 'cinematic' | 'replay'
```
Requirements: critically-damped spring follow (no rubber-banding, no jitter), look-ahead
along the trail tangent so you can see the next feature, FOV that scales with speed,
handheld noise that scales with terrain roughness and g-force, impact shake, roll that
partially follows bike lean (partially — full roll makes people sick), never clips into
terrain (push out along the collision normal), smooth transitions between modes,
a cinematic mode for the run intro.

---

## 7. Input — `src/game/input.js`

```js
const input = createInput(ctx);   // attaches listeners to ctx.container/window
input.state = {
  steer: -1..1,        // A/D or left stick X
  pitch: -1..1,        // W/S or left stick Y — lean fwd/back, and air pitch
  roll: -1..1,         // Q/E or right stick X — air roll / whip
  brakeFront: 0..1,    // J or L2
  brakeRear: 0..1,     // K or R2
  pedal: 0..1,         // Shift or R1
  pump: 0..1,          // Space (hold to preload, release to pop)
  manual: boolean,
  reset: boolean, pause: boolean, cameraCycle: boolean, photoMode: boolean,
  anyPressed: boolean,
};
input.update(dt);
```
Keyboard + Gamepad API (analogue triggers/sticks, deadzones, and rumble on impact if
available). Consumers read `ctx.input.state`.

---

## 8. Game layer

### `src/game/gameplay.js`
Run state machine: `'menu' | 'countdown' | 'running' | 'crashed' | 'finished' | 'paused'`.
Timing with per-checkpoint splits and a persisted best (localStorage), ghost of the best
run (record + replay the transform stream), style scoring (air time, whips, manuals,
clean landings, near-misses), crash → respawn at last checkpoint with a time penalty.
```js
gameplay.state, gameplay.time, gameplay.splits, gameplay.best, gameplay.score
gameplay.start(), gameplay.restart(), gameplay.pause()
```
Emits: `run:start`, `run:checkpoint`, `run:finish`, `run:crash`, `trick:landed`.

### `src/ui/hud.js`
DOM overlay (not canvas sprites) — speed, timer, split delta (green/red), airtime,
style meter, trail progress, checkpoint flashes, damage/crash vignette hookup.
Must look designed: real typographic hierarchy, tabular numerals for the timer,
tasteful motion, no default-browser look. Hides in photo mode.

### `src/ui/menu.js`
Title screen with the game running behind it in cinematic camera, run summary card,
settings (quality, camera, invert, volume), controls reference. Same design language
as the HUD.

### `src/audio/audio.js`
WebAudio, **all sound synthesised in-code** (no downloaded files): tyre roll pitched by
speed and filtered by surface, gravel/rock/loam variants, skid, suspension compression
thunk and rebound, chain slap and freehub buzz when coasting, wind noise scaling with
speed, impacts by severity, breathing/effort, crash, ambience bed (forest birds, creek,
distant wind) with reverb by openness. Positional where it makes sense. Ducking and a
master limiter so it never clips. Must start only after a user gesture.

---

## 9. Quality settings — `ctx.settings`

```js
ctx.settings = {
  shadows: true, shadowMapSize: 2048, cascades: 3,
  terrainLOD: 1.0, vegetationDensity: 1.0, grass: true,
  ao: true, bloom: true, dof: true, motionBlur: true, godRays: true,
  filmGrain: true, chromaticAberration: true, ssr: false,
  antialias: 'smaa', pixelRatio: Math.min(devicePixelRatio, 2),
  volumetricClouds: true, particleDensity: 1.0,
}
```
Presets `low|medium|high|ultra` set these. Every visual module must respect its own
flags and degrade gracefully — but **default is `high`, and `high` must look stunning.**

---

## 10. Definition of done (per module)

- No console errors or warnings when the game runs.
- No per-frame allocations in `update()`.
- Respects its `ctx.settings` flags.
- Deterministic given `ctx.seed` (world gen modules).
- Reads correctly at 1920×1080 in a screenshot with no other module finished —
  i.e. it must not depend on someone else's polish to look right.
- **The bar is not "works". The bar is "a reviewer comparing this side by side with a
  current-gen console game does not immediately know which is which."**
