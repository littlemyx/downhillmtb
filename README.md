# DESCENT

A downhill mountain-bike game in Three.js, built with **zero authored assets**.

Every texture is generated from a seeded PRNG at boot, every mesh is built in code, all audio is
synthesised with WebAudio, and the rider is a procedurally-built skinned rig driven by inverse
kinematics from the physics state. There are no images, models, sound files or fonts in this
repository. The only runtime dependencies are `three` and `postprocessing`.

```bash
npm install
npm run dev      # http://localhost:5178
```

Query parameters: `?quality=low|medium|high|ultra`, `?seed=<number>`, `?debug`.

---

## What's in it

| | |
|---|---|
| **World** | 3072 m square, 1537² heightfield, ~700 m of relief, hydraulic erosion, quadtree LOD |
| **Trail** | 2.7 km descent, 8 named phases, route-found down the fall line, carved into the terrain with banking solved from expected entry speed |
| **Terrain material** | 8 procedural surface layers in a texture array, triplanar projection, height-based splat blending, stochastic tile-break with explicit gradients |
| **Sky** | Tuned Preetham atmosphere, raymarched volumetric cloud deck, cloud shadows, height-based aerial perspective, PMREM environment |
| **Vegetation** | ~200k instances, three procedural conifer species, three LOD tiers with baked impostors, coherent gusting wind, coverage-preserving alpha mips |
| **Physics** | 240 Hz fixed step, per-wheel load transfer, slip-ratio/slip-angle tyre model, independent progressive fork and shock, rider lean pendulum with a grip-derived ceiling, pumping, air control, landing evaluation |
| **Bike** | Four-bar Horst link solved per frame by circle–circle intersection, 32-spoke 3-cross lacing, chain solved as a belt over four circles, 200 mm vented rotors |
| **Post** | HDR highlight roll-off, adaptive exposure with camera-cut detection, ACES, procedural grading LUT with a boot-time white-point assertion, N8AO, SMAA, bloom, motion blur |
| **Game** | Run state machine, 8 checkpoint splits, persisted PB, 30 Hz ghost replay, style scoring, crash/respawn, keyboard + gamepad with rumble |

## Measured

Real frame at 1920×1080 — all seventeen systems ticking, bike descending, camera in chase:

| section | ms | fps |
|---|---|---|
| open alpine | 10.35 | 96.6 |
| jump line | 13.13 | 76.1 |
| dense forest | 15.29 | 65.4 |
| final sprint | 13.04 | 76.7 |

Physics, independently re-verified: terminal velocity 17.18 m/s on a 15% grade (identical at
30/60/144 Hz), static sag 29.4% fork / 26.7% shock, braking 8.37 m/s², pumping skill gap 3.56 m/s
between best and worst phase, zero NaN and zero tunnelling across 300 s of random input at mixed
20–240 Hz.

Image telemetry across the 16-shot review set: 0.0% crushed pixels (L<6), 0.0% clipped (L>250).

## Honest status

This is **not** current-generation console quality and does not claim to be. An independent
review panel placed it alongside *Steep* (2016, PS4-era) and *Descenders* (2019) — above both on
terrain material, lighting and physics simulation, below them on character readability and art
cohesion. As a browser renderer it is unusually complete; as a game it is not finished.

Known open items are tracked in [`qa/`](qa/):

- **The trail cannot yet be ridden end to end.** The corner-berming pass runs in only 3 of the 8
  course phases, so the tightest corners are built with roughly 15° of bank where they need 45°.
- The rider reads as a mannequin at close range — the IK solver is sound, the garment and hand
  geometry are not.
- Foliage is procedurally authored and reads as cards rather than photographic plates.
- There is no asset pipeline (no glTF/KTX2 import), which gates any authored art entering the
  project at all.

## Repository layout

```
src/core/        engine, post-processing, shared context, RNG
src/world/       terrain, terrain material, trail, sky, vegetation, water
src/physics/     bike dynamics, terrain collision
src/entities/    bike model, rider
src/camera/      chase camera
src/game/        gameplay, input, particles
src/ui/          HUD, menus
src/audio/       synthesised audio
qa/              review harness, screenshots (gitignored), triage documents
CONTRACT.md      module interface specification
ADDENDUM.md      integration facts discovered during the build
```

`CONTRACT.md` is the authoritative interface spec: it was written before the modules and is what
allowed them to be built independently and still link up.

## Licence

MIT
