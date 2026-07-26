# DESCENT — Integration Addendum (read AFTER CONTRACT.md)

Facts discovered while building the foundation layer. These are binding — they describe
code that already exists and works. Where this file and CONTRACT.md disagree, **this file wins**.

Foundation modules already implemented and passing build:
`src/core/engine.js`, `src/core/postfx.js`, `src/world/terrain.js`,
`src/world/terrainMaterial.js`, `src/world/trail.js`, `src/world/sky.js`,
`src/game/input.js`. Read the ones you depend on — do not guess their API.

---

## A. Terrain

- **Never raycast `terrain.group`.** It is a quadtree whose chunk meshes toggle `visible`
  every frame as the LOD cut moves, so a raycast answer depends on where the camera is.
  Use `terrain.sampleHeight(x,z)` / `terrain.sampleNormal(x,z,out)` or `collision.rayDown()`.
- Extra helpers beyond the contract, safe to use:
  `terrain.sampleWetness(x,z)`, `sampleSnow(x,z)`, `sampleCarve(x,z)` (0..1, how much the
  trail carved here — use it to keep vegetation off the tread), `treelineAt(x,z)`,
  `snowlineAt(x,z)`, `terrain.valleyY`, `terrain.creekLevel`, `terrain.group`, `terrain.chunks`.
- Chunk geometry attributes (if you ever need them): `position`, `normal`, `uv`,
  `color` (full base albedo, linear-sRGB), `aSplat` vec4, `aTerrainExtra` vec4, `aSurface` float.
- `Surface` enum is exported from `src/world/terrain.js`.

## B. Sky / fog / lighting

- `ctx.sun` is the DirectionalLight; `ctx.sky.sunDirection` is a **unit Vector3 pointing
  towards the sun**.
- **Height fog is a patched shader chunk.** Any custom `ShaderMaterial`/`RawShaderMaterial`
  you write MUST set `fog: true` and merge `THREE.UniformsLib.fog` into its uniforms
  (`THREE.UniformsUtils.merge([THREE.UniformsLib.fog, yourUniforms])`) or it will not fog
  and will visually detach from the scene at distance. `MeshStandardMaterial` and friends
  get it automatically. This matters most for vegetation, water and particles — unfogged
  distant trees are an instant tell.
- Do not touch `scene.fog`, `scene.environment` or `scene.background`; sky owns them.
- Shadows: sky owns the sun's shadow camera and re-fits it around `ctx.camera` each frame.
  Set `castShadow`/`receiveShadow` on your own meshes; do not reconfigure the light.

## C. Post-processing

- `postfx` already reads `ctx.bike.state.speed` and `ctx.bike.state.position` each frame,
  and already subscribes to `ctx.events` for `run:crash`. You do not need to call into it.
- Motion blur protects the rider with a world-space sphere centred on
  `ctx.bike.state.position`. Keep the bike+rider inside ~1.6 m of that point.
- Tone mapping is done in postfx (renderer-side tonemapping is off). **Author colours as if
  they will be ACES tone-mapped** — i.e. do not pre-darken your materials to compensate.
- Emissive materials will bloom above luminance ~0.85. Use that deliberately (brake rotors
  glinting, tape, sun on chrome), and avoid accidentally blowing out large white surfaces.

## D. Bike model ↔ rider attachment contract (BINDING)

`bikeModel` must expose, in addition to `group`:

```js
bikeModel.anchors = {
  gripL, gripR,       // THREE.Object3D at the outer end of each grip
  pedalL, pedalR,     // THREE.Object3D at each pedal spindle, rotating with the cranks
  saddle,             // THREE.Object3D at the saddle nose
  bb,                 // bottom bracket centre
  frontAxle, rearAxle,
  bar,                // handlebar centre (steers)
};
bikeModel.chassis;    // THREE.Object3D that carries the whole bike, positioned/rotated from ctx.bike.state
```

All anchors are children of `bikeModel.group` so `getWorldPosition()` gives the rider IK
targets for free. `rider` reads them every frame; it must handle `ctx.bikeModel` or an anchor
being missing without throwing.

## E. Events already in use

Emitted by `gameplay` (you own the emit) and consumed by postfx/audio/hud/particles:
`run:start`, `run:checkpoint`, `run:finish`, `run:crash`, `trick:landed`.
Additional events you may emit/consume: `bike:impact` `{ severity, position, normal, surface }`,
`bike:skid` `{ intensity, surface }`, `water:splash` `{ position, velocity }`,
`quality:changed` `{ quality }`.

## F. Ordering reminders

- `chaseCamera` uses `lateUpdate` (after bike/rider have moved). So does `hud`.
- `particles` runs before the camera so emitters are placed from the current frame's contacts.
- Anything reading `ctx.bike.state` must tolerate `crashed === true` and `airborne === true`.

## G. Performance reality check

The foundation modules are already heavy (terrain quadtree, volumetric clouds, 9-pass
composer). Your module's budget:

| module | draw calls | notes |
|---|---|---|
| vegetation | ≤ 40 | instanced only; LOD + imposters |
| particles | ≤ 12 | pooled, one InstancedMesh/Points per effect family |
| bikeModel | ≤ 20 | merge what you can; one material per finish |
| rider | ≤ 8 | one skinned mesh preferred |
| water | ≤ 6 | |

No `new THREE.Vector3()` inside any `update()`.
