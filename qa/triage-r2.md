I verified the contested claims against source and re-measured the screenshots. Several critics converged on the same defect under different names, and three of the loudest findings are misdiagnosed. Here is the consolidated order.

---

# DESCENT — Consolidated Work Order

## VERDICT: NOT_AAA. Concur with all seven, but not for all seven's reasons.

Seven critics produced ~90 findings. They collapse to **10 root causes**, three of which are single stale constants. Two "critical" findings are misdiagnoses that would send an engineer to rewrite a file that is already correct.

### The three things standing between this build and the bar

1. **The indirect-light chain collapsed between round 1 and round 2, and nobody noticed.** Measured, same shot, same camera: `r1_07` median luminance **107.2**, 0.0% of pixels below L=6. `r2_07` median **12.4**, 13.0% below L=6, **75.0% below L=24**. That is a 9x collapse of the shadow floor in one iteration. Seven of fourteen shots have no readable shadow detail. Uniform near-black in shadow is the single most reliable tell that a viewer is looking at a render and not a photograph.
2. **Nothing in the review set is in focus, so nobody — including these seven critics — has actually seen the art.** Half the material and vegetation findings were made through a full-frame blur.
3. **The camera was never written.** `chaseCamera.js` is 69 lines, self-labelled `SCAFFOLD STUB`. It puts the camera underground in `r2_11` and dead-centres the horizon in 10 of 14 frames.

---

## ROOT CAUSES (collapsed from ~90 findings)

**RC-1 — The lighting collapse. Six findings, three critics, one regression.**
Every mechanism that should deliver indirect light is either off or fighting the direct term:
- `postfx.js:677-691` — N8AO at `intensity 2.8` (default is 1.0), `aoRadius 1.6 m` (a GI radius, not a contact radius), `colorMultiply true`, `color (0.03, 0.045, 0.07)`. It is a full-screen pass over the composite, so it multiplies **direct sun**.
- `sky.js:1497` — `hemi.intensity = 0.04 + 0.10 * daylight` → max **0.14**, against a sun at 3.51.
- `sky.js:1493` — sky fill desaturated 34% toward white, so what little fill exists is colourless.
- `sky.js:1496` — `hemi.groundColor` is a fixed orange, occupying the entire downward hemisphere on a course made of downward-tilted normals.
- `sky.js:1270` — `envScene` contains **only the sky dome**, so the PMREM's entire lower hemisphere is the flat `uGroundHaze` constant. There is no ground bounce in the IBL at all.
- `postfx.js:748-750` + `:195` — adaptive exposure clamped to [0.70, **1.50**] and pinned at its ceiling, with an `S_CURVE_AMOUNT 0.26` contrast curve stacked on top of ACES's own toe.

**RC-2 — The world moved and the atmosphere didn't. Five findings, four critics, one stale comment.**
`terrain.js:52-53` → `BASE_Y = 1140`, `RELIEF = 720`. The world spans **1140–1860 m ASL**.
`sky.js:1147` still says *"terrain tops out ~450"*, and on that basis sets `FOG_REF_HEIGHT = 110`, `FOG_SCALE_HEIGHT = 360`, `CLOUD_BOTTOM = 1750`. Consequences: `exp(-(1140-110)/360)` = fog running at **0.8–5.7% of authored density** everywhere a camera can be (no aerial perspective, visible map edge, flat mint horizon band), and a cloud base **110 m below the summit** that the raymarch's `rd.y <= 0.012` early-out can never render.

**RC-3 — The chase camera is a stub.** `chaseCamera.js:49-52` clamps `_desired` **before** the damp and skips the clamp entirely when `!inBounds`, so the damped actual position lags below the surface on any convex rollover. `lookAt` is hardcoded to `forward*6` with zero lateral, vertical or roll offset. `qa/harness.js:33 poseOnTrail` applies no clamp at all.

**RC-4 — DoF focus is locked to the bike, and the bike is not where the camera is.** `postfx.js updateFocus()` sets `_focusPoint` = bike position; `fx.dof.target = _focusPoint`; the library then does `calculateFocusDistance(target)` = `camera.distanceTo(bike)` every frame. `qa/harness.js` teleports the camera to t=0.005…0.96 along the trail while the bike stays parked at the start. Focus distance is therefore hundreds of metres wrong in most shots, everything lands in the near-blur field at `bokehScale 1.55`, half-res. **That is why the clouds at infinity are smeared.**

**RC-5 — The grading LUT's white point is not white. Verified by measurement.** Across `r2_01/02/04/12`: red reaches 255 (138,203 pixels in `r2_04` alone) and blue reaches at most **248–253 — zero pixels at 255, anywhere in the set**. Two causes in `buildGradingLUT`: `sCurve()` opens with `clamp01(x)`, silently discarding red's CDL headroom above 1.0; and `HIGHLIGHT_TINT (+0.010, +0.003, −0.008)` is applied at full strength at `highW = 1.0`. Highlights diverge to yellow instead of desaturating to white — the exact inverse of what ACES/AgX/Uncharted2 are built to do.

**RC-6 — The tread mosaic is NOT z-fighting.** `trail.js:2377-2379` already sets `polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4`, plus `renderOrder = 1`. The real cause is `trail.js:2426`:
```js
const py = tH + 0.035 + clamp(ideal - tH, 0, 0.085);
```
Tread Y is a **clamped** function of `terrain.sampleHeight` (a 2.0 m bilinear heightfield) evaluated at 0.8 m ribbon spacing. The clamp saturates at both ends, creating C1 discontinuities along irregular contours; `geo.computeVertexNormals()` at line 2488 converts those into hard normal breaks, and the 0.8 m ribbon beating against the 2.0 m heightfield produces the straight-sided facets. It is a **shading** discontinuity on geometry that has no relief — which is why the earlier polygon-offset fix didn't help.

**RC-7 — The water is not the problem. The sky's below-horizon constant is.** See "Rejected findings" below.

**RC-8 — Triplanar is already fully engaged.** See "Rejected findings" below.

**RC-9 — Vegetation LOD is not albedo- or silhouette-matched.** Far imposter tier median RGB(156,138,113) vs near canopy RGB(51,73,42): a 60° hue rotation and a 3x luminance inversion in the wrong direction. `bakeImposterAtlas` bakes with `AmbientLight(PI*0.80)` + `HemisphereLight(0.85)` under `NoToneMapping`, then the scene re-lights and re-tonemaps the result. Compounded by alpha-mip coverage loss above `alphaTest 0.34`, which fills the crown into a solid triangle at 4–20 px. Separately, tier 1 drops `cards: 3 → 1`, so branches stop being crossed and go edge-on at level camera — which is the entire game.

**RC-10 — Perf: 13.5 ms of the 23.4 ms frame is provably recoverable.** The perf critic is the only reviewer with instrumented A/B numbers; trust them. Terrain fragment shader is 10 ms (tile-break alone is 5.5 ms), 4.45 M of 5.53 M submitted triangles are off-screen vegetation, N8AO is 5.44 ms for an effect nobody can find in the frame.

---

# WORK ORDER — BY OWNING FILE

Ranked by impact on "does this look like a console game". **Sequencing note:** `chaseCamera.js` + `postfx.js` items 1–2 must land *first* — not because they are the highest impact, but because until they do, no other engineer can see their own work.

---

## 1. `src/core/postfx.js` — **highest impact, and it gates everyone else**

**P0-a. Unblock the frame: fix DoF focus.** Do **not** switch to `worldFocusDistance`/`worldFocusRange` — I checked `node_modules/postprocessing`: those are *deprecated aliases* for `focusDistance`/`focusRange`, and the CoC shader already computes `getDistance(viewPosition)` in view-space metres. The authored 6.0/55.0 already mean metres. The actual fix:
- Drive `_focusPoint` from a point **12–18 m ahead along the trail spline**, not from `bike.position`. Fall back to `camera + forward*15` whenever the bike is outside the frustum (currently the fallback only fires when the bike doesn't exist).
- Raise `tier.dofScale` from 0.4–0.6 to **1.0 at high/ultra, 0.75 at medium**. You are shipping a 768x432 upscale.
- Cap `bokehScale` at ~0.6 (currently 1.55).
- **Recommendation: disable DoF in gameplay entirely, reserve it for replay and photo mode.** That is what Descenders and the Forza chase cameras do, it costs 1.53 ms, and it is currently destroying the 7 ms of near-field surface detail the terrain shader is paying to synthesise.

**P0-b. Stop N8AO attenuating direct sunlight.** `intensity 2.8 → 1.0–1.2`; `aoRadius 1.6 → 0.4–0.6 m` (make it a contact term, not a GI term); `color (0.03,0.045,0.07) → ~(0.10,0.13,0.18)` so occluded pixels resolve toward sky-fill, not toward zero. Acceptance: an open shadow adjacent to a sunlit surface in the same frame must land at **L 25–40 against a sunlit L of ~197**, not L 12.

**P0-c. Fix the white point.** Verified: zero pixels reach B=255 in any of the 14 shots. Two changes in `buildGradingLUT`: (i) weight `HIGHLIGHT_TINT` by `(1 - highW)` so the amber split-tone dies out at the white point; (ii) move the CDL slope above `sCurve`'s `clamp01`, or drop that clamp and clamp only at the final write, so red doesn't clip a stop early. **Add a unit test asserting the built cube's `(size-1,size-1,size-1)` entry is exactly (1,1,1).**

**P1-a. Unstack the contrast curves.** Remove `S_CURVE_AMOUNT` (0.26) — ACES already owns the tonal curve, and stacking a second S-curve on its toe is half of why `r2_07` is 90.8% below 20 IRE. Add a small shadow lift in the LUT toe instead. Widen the exposure clamp from [0.70, 1.50] to roughly **[0.35, 3.0]** and give `AdaptiveExposureEffect` a metering mask excluding the upper ~25% of frame so it stops metering on the sky.

**P1-b. Threshold bloom on post-tonemap luminance.** `luminanceThreshold 0.85` is currently evaluated on pre-tonemap HDR where a lit sky sits many units above 1.0, so the entire sky enters the bloom source at full weight and 7–8 mip levels at radius 0.78 spread it across the frame. Drop `bloomLevels` to 5 and radius to ~0.60.

**P1-c. God rays: gate or delete.** `sunVisibility` is a facing test (`smoothstep(-0.05, 0.45, camFwd·sunDir)`) with **no occlusion test**, and it is applied as a blend weight *after* the 60-sample raymarch has already run. So the pass costs full price when the sun is behind a mountain, and composites shafts over the ridge that is blocking it. At `godScale 0.5` / `KernelSize.SMALL` it produces no visible shaft structure in any of the 14 shots — only a milky wash. **Given the 44 fps deficit, delete the pass for this milestone.** If kept: split it into its own `EffectPass`, gate on an actual depth-buffer occlusion test at the sun's projected position, `godSamples 60 → 40`.

**P1-d. Recover the sky's colour.** `HIGHLIGHT_BLEACH 0.30 → ~0.10`, onset `smoothstep(0.78,1.0) → (0.90,1.0)`, and remove the negative blue term from `HIGHLIGHT_TINT`. Measured skies are currently (225,233,237) to (245,243,240) — white, not blue.

**P2. Perf, in this order (measured):** `aoHalfRes: true`, `aoSamples 32 → 16`, `aoDenoise 8 → 4`, `aoDenoiseIterations 2 → 1` — recovers **4.8 ms** for an effect that is not visible in any shot. Cut `ChromaticAberration` below `ultra` (0.86 x 0.24 px is a no-op costing a full-res RGBA16F round trip). Gate `MotionBlurEffect` off below `speed01 < 0.08`. Drop `LUT3DEffect.tetrahedralInterpolation` at high.

**P3.** Commit to a palette. The CDL deltas are 1.6% / 1.25-of-255 / 1.8% and saturation is 1.03 — a null grade to within measurement noise. Author two LUTs (cold high-altitude key, warm sub-treeline key) and interpolate on rider altitude. This is last because it is meaningless until RC-1 restores a tonal middle to grade.

---

## 2. `src/camera/chaseCamera.js` — **stop-ship, and it is 69 lines**

**P0-a.** Clamp the **final** camera position after the damp, not `_desired` before it, and do it **unconditionally** — fall back to a large negative floor when `!inBounds` rather than skipping the clamp. Raise clearance from 1.2 m to **~1.8 m**.

**P0-b.** Sphere-cast (r ≈ 0.4 m) from the look target back along the boom each frame and pull the camera in to the first terrain hit, so the camera orbits ridges instead of passing through them.

**P1.** Write the camera the contract asks for (§6). Critically-damped spring with per-axis rates (loose yaw, tight pitch), lateral look-ahead driven by trail curvature, roll banking into berms, horizon biased to a third rather than dead centre, vertical damp decoupled from horizontal. Currently `lookAt` is `forward*6` with zero offset and a uniform `rate = 6.5`, which is why the trail vanishing point *and* the horizon both land within ~40 px of frame centre in 10 of 14 shots. Perfect symmetry in every frame reads as "nobody framed this," and it costs the game its sense of speed before the DoF even touches it.

---

## 3. `src/world/sky.js` — **two constants and one missing bounce**

**P0-a. Read the world's actual altitude.** `FOG_REF_HEIGHT` must come from `terrain.js`'s exported `bounds.minY` (1140), not a hardcoded 110. Raise `FOG_SCALE_HEIGHT` from 360 to **~900–1200 m** (the right e-folding height for alpine haze over 720 m of relief). Re-tune `FOG_DENSITY` against a measured target: a ridge at 3 km at **55–70% haze fraction**, one at 500 m at 10–15%. **Delete the "terrain tops out ~450" comment at line 1147** — it is what caused this.

**P0-b. Give the world an indirect term.** `hemi.intensity` from `0.04 + 0.10*daylight` (max 0.14) to **0.35–0.50**. Cut the desaturation lerp at line 1493 from 0.34 to ~0.12 so the sky fill keeps its blue. Stop faking ground bounce with a fixed orange: derive `hemi.groundColor` from actual sunlit terrain albedo × sun colour × horizontal irradiance, scaled to ~0.15 of the sky term. Acceptance: **B−R in an open shadow must be ≥ +12** (currently −10.2 in `r2_01` — the shadow is warmer than the sun).

**P0-c. Put a ground in the PMREM.** `envScene` (line 1270) contains only the sky dome, so the entire lower hemisphere of the IBL is the flat `uGroundHaze` mint. Add a large distant ground disc shaded with approximate sunlit-terrain albedo. This single change fixes the undersides of every rock, log, berm and the bike, and it fixes the below-horizon band in the same stroke.

**P0-d. Kill the horizon step.** `skyColor = mix(uGroundHaze, skyColor, smoothstep(-0.055, 0.015, direction.y))` is a ~4° window onto a flat constant. Widen to ~15°, darken with `-direction.y`, and tint toward sunlit terrain albedo rather than horizon haze. **This is also the fix for what three critics called "the flat water plane" — see rejected findings.**

**P1-a. Clouds must share a photograph with the mountain.** `CLOUD_BOTTOM = 1750` sits 110 m *below* the 1860 m summit and 84 m above the start gate, yet `rd.y <= 0.012` early-out means cloud can never generate at or below eye level. **Take the one-line option: raise `CLOUD_BOTTOM` to ~2600 m.** It is defensible alpine cumulus, it is honest geometry, and it is the safe milestone move.

**P1-b. Shadows.** `normalBias 0.15` against a shadow texel the module itself computes at ~0.16 m pushes the receiver a full texel off its own surface — the course tape and posts cast no contact shadow at 2–5 m from the lens. Drop to **0.02–0.04** and compensate acne with `sun.shadow.bias ≈ -0.0002`. Then revisit the single-slice design in the `CONTRACT-NOTE`: one 150 m slice cannot produce terrain self-shadowing on 720 m of relief, which is why `r2_00` — an aerial wide of an entire mountain at 19.2° sun elevation — contains **no shadows at all**. Add a coarse second cascade at 400–1500 m / 1024–2048; it only needs terrain and far vegetation.

**P2.** `CLOUD_TIERS.high steps 52 → 32`, light march capped at 4 iterations. Force `VOLUMETRIC_CLOUDS` **off** on the PMREM capture material — a 256-cube irradiance probe cannot resolve cloud detail and it re-runs the full raymarch whenever the sun moves.

---

## 4. `src/world/trail.js` — **the surface under the front wheel**

**P0. Fix the tread mosaic — and do not touch `polygonOffset`, it is already correct.** At line 2426 replace the clamped height with a smooth one:
- Low-pass the `crossProfile`/rut/stamp displacement so the emitted ribbon geometry carries only features above ~1.5 m wavelength. Sub-metre relief must not live in vertex positions at 0.8 m spacing.
- Replace `clamp(ideal - tH, 0, 0.085)` with a **smoothstep-blended** offset so there are no saturation edges; or better, low-pass `tH` along the ribbon before offsetting so the tread stops inheriting the 2.0 m heightfield's own faceting.
- Move rut, windrow and braking-bump relief into the tread **normal + height map** (`heightToNormalCanvas` already exists at line 243) and drive it through the same parallax/height-blend path the terrain uses.
- Blend the tread's shading normal toward the terrain normal (weight ~0.5) after `computeVertexNormals()`, so the strip cannot disagree with the hillside it sits on.
- If geometric ruts are wanted for wheel feel: decouple collision from render and subdivide the render ribbon to ≤0.2 m for the first 25 m of camera distance.

**P1. Bind the tread material to every LOD tier.** Beyond ~80–120 m the tread renders as untextured flat mid-grey and is *discontinuous* — three disconnected grey parallelograms on the hillside in `r2_02`, a grey slab on the ridge ahead in `r2_01`. Untextured mid-grey is the universal missing-material signal. Also fix whatever is dropping individual far segments.

**P1. Course furniture is placeholder.** In `r2_04` a sign board hangs in the air with no post and no shadow; in `r2_08` an untextured white/blue box sits half-sunk in the hillside; in `r2_00` pure-black slabs and a pure-white cube sit on the snow field. **Ground-project every furniture instance** (raycast down, set Y to the hit, align up to the hit normal), give the sign a post, delete or texture the box, and set `castShadow`/`receiveShadow` on all of it. Route tape generation along the trail centreline with a lateral offset — it currently crosses the racing line at chest height in `r2_03` and `r2_12`, which any rider will catch. Post spacing 4–6 m max, solve the catenary between post tops so sag cannot dip below terrain.

**P2. The named sections are metadata with no geometric consequence.** `berm-flow` has zero camber, `jump-line` has no lip/transition/landing, `rock-chute` is a level footpath, `rock-garden` has nothing above gravel. This is real and it is a milestone-2 scope item, not a rendering fix: deform both the tread mesh and the underlying terrain per section type, and verify each type produces a visibly different silhouette from 30 m out.

---

## 5. `src/world/vegetation.js`

**P0-a. Match the imposter to the mesh it replaces.** In `bakeImposterAtlas`, bake **albedo + packed normal**, not shaded colour — drop ambient to exactly `PI` with no hemisphere, or output world-space normal + alpha to a second RT so the far card is lit by the scene sun. Then fix the alpha mip chain with coverage-preserving rescale (Castano's method: per mip, binary-search a scale factor so the fraction of texels above `alphaTest` matches mip 0), or move the far material to alpha-to-coverage. **Add a numeric regression check:** imposter-tier median RGB within 12% luminance and 8° hue of near-tier canopy under the same sun.

**P0-b. Stop tier 1 becoming a TV aerial.** `cards: 1` un-crosses the branch quads, so at level camera — which is the entire game — most planes go edge-on and leave a bare trunk. Set `cards: max(2, cfg.cards - 1)`, `cardWidth × 1.9`, `crownWidth × 1.05`, `branches 0.7 → 0.9`, `whorls 0.55 → 0.75`. Pay for it out of the trunk, which costs nothing visually at 64 m: `trunkRadial 5 → 3` and cull the trunk above `crownStart`.

**P0-c. Perf: frustum-reject before packing (4.47 ms).** `makeMesh` sets `frustumCulled = false` on every InstancedMesh with the comment *"culling is done by the tier bands"* — but the bands are **radial**, and at 70° FOV the frustum is ~19% of that sphere. 4.45 M of 5.53 M submitted triangles are never rasterised. **Do not simply flip `frustumCulled = true`** — the packed set spans the whole active radius and three's bounding-sphere test over-culls whole tier meshes. Instead reject inside `layer.update(px,py,pz)`: it already walks every chunk and every chunk already carries `cx, cy, cz, radius`. Extract the six frustum planes once per update, reject chunks whose sphere is fully outside, expand by one chunk radius so the shadow tier still feeds the sun pass. ~0.1 ms of CPU, exact, no popping.

**P1-a. Alpha-to-coverage on near and mid tree materials.** `alphaTest: 0.34`, `transparent: false`, no A2C, on sub-pixel needles against a blown sky — the canopy is peppered with 2–4 px pure-white speckle in five shots and it will crawl violently in motion. SMAA cannot fix this: it is morphological, it runs **last** (after grain and CA), and its `edgeDetectionThreshold` was raised to 0.06 specifically so it wouldn't fire on the 0.030 grain, which also makes it blind to every foliage edge below 6% contrast.

**P1-b. Make backlit needles glow.** The Frostbite wrapped-transmission path exists and delivers ~0.01 because it multiplies `diffuseColor.rgb` (a needle albedo, 0.05–0.15) by a `uSunColor` that has already been scaled by `i * 0.20`. Replace with a dedicated per-species transmission colour (start ~`vec3(0.20, 0.42, 0.13)` for spruce) and feed it the **unscaled** sun colour. Raise `uTrans.w` from 0.12 to ~0.25. Acceptance: with the sun directly behind a foreground tree, the needle mass sits at **25–40% of sky luminance**, not the 3% measured now. At 19.2° sun elevation this is the best lighting event in the scene and it is currently being thrown away.

**P1-c. Trees stop leaning in unison.** Normal-align weight `0.20 + rng()*0.16` tilts every tree ~10° downslope, and slope direction is coherent across a hillside, so the whole forest tips together. Drop to **0.04–0.08** and express slope response as a basal sweep in `buildConifer` instead. Boulders (0.55) and moss (0.95) are correct — leave them.

**P1-d. Vegetation shadows.** Tier 0 casts to 82 m but the shadow slice is 150 m, so trees between 82–150 m sit inside the frustum contributing nothing. Set tier 1 `shadow: true`. Coordinate with the sky.js second-cascade item.

**P2-a. Perf: pull `TREE_TIERS[0].far` from 82 to 40 m** (`fadeOut [32,40]`), extend tier 1 `near` to 32, and add a genuine ~450-tri mid asset between the 1,492-tri near and the 150-tri mid — that 10x cliff is why the near band has to be so wide. 1,081 near instances × ~1,500 tris = 1.5 M triangles, all `castShadow: true` with an alpha-tested custom depth material that kills early-Z on the shadow map, for trees the 2048/150 m cascade cannot resolve anyway.

**P2-b. Clustered placement.** The 5.8 m lattice with ±2.76 m jitter can never produce a thicket or a glade. Replace with a Thomas cluster process (25–60 m Poisson-disc parents, 4–15 children at 3–8 m Gaussian spread), reject whole parents against the density field, raise stand-noise frequency from ~465 m to 25–50 m and let density swing 0→1.4 rather than 0.70→1.25. Add a stochastic 60–120 m ecotone at the treeline — currently it traces an elevation isoline exactly.

---

## 6. `src/world/terrainMaterial.js`

**P0. Stop shading dry dirt as wet plastic.** Roughness **floor of 0.045** at line 1901 is near-mirror; `uWetness` defaults to **0.18** and the wet branch drags `terrRough` toward 0.075; `uDetailStrength` 0.85 perturbs the normal at high frequency; there is **no specular-normal filtering anywhere**. Result: 3.6% of the sunlit gravel bank in `r2_12` exceeds L=230 on a surface that should be roughness 0.85–0.95. Fixes: `uWetness` default → **0.0**, driven only by actual rain/creek state; roughness floor `0.045 → 0.25` for anything not an explicit water film, and ≥0.82 for the dirt/scree layer; add **Toksvig/LEAN roughness-from-normal-variance** so sub-pixel detail normals raise roughness instead of aliasing. This one will crawl violently in motion.

**P1. Perf — 7 ms, and it is the single largest line item in the frame.** Author two material variants sharing one uniform block, assigned **per chunk from the quadtree node**: NEAR (depth 6–7, inside ~50 m) keeps `TILE_BREAK` + `TRIPLANAR`, `maxLayers 3`; FAR (depth 3–5) drops both, `maxLayers 2`. Note the measured caveat: a depth≤4 split alone only returns 0.5 ms because near chunks own the pixels — so the near variant must also come down: **tile-break from 3 taps to 2** (the two-copy blend still kills the grid) and gate it on `gDetailFade > 0`. Separately `maxLayers 4 → 3` at high (2.31 ms) and raise the skip threshold at line 1602 from `0.018` to **0.06** — below that a layer contributes under 1/255 after tonemapping.

**P2. Give the ground a size distribution.** Autocorrelation of the `r2_01` tread shows a clean peak at 62 px with a harmonic at ~110 px — a monodisperse dot lattice at ~20 cm pitch. Rebuild dirt/scree detail as three superimposed scatter octaves (~25 cm / 8 cm / 2.5 cm), mass-weighted to the small end, independently jittered, with per-stone flatten/rotate. Drive the height channel from the **union (max)**, not the sum, so large stones occlude small ones, then height-blend against a fines layer so stones bury at random depths.

**P2. Macro albedo.** The whole lit slope is one warm brown (~150/115/80) modulated only by N·L. The plumbing exists (`uMacroNoise`, `uVarScales`, three decorrelated octaves at ~line 1400) — it just isn't driving albedo. Add a 200–600 m tint field across 3–4 authored geological hues, a 30–80 m value field at ±15%, wire `terrDrain` into gully staining, and add an altitude hue ramp. **Move this evaluation off the mipped texture path into a world-space analytic term** so it survives past 300 m; currently the establishing shot's far half has no material information at all.

**P2. Height-based splat blending.** Replace linear alpha at surface transitions with `blend = saturate((h_a*w_a - h_b*w_b)/contrast + 0.5)` so the proud material wins and the seam goes ragged at stone scale, instead of the constant-width 30 cm muddy smear currently running the length of the mountain.

**P3. Loam albedo floor.** Median luminance 11–16/255 on the forest floor is ~0.3% linear; dry duff is 10–15%. **Do this last and re-measure first** — I suspect the layer was pre-darkened to compensate for the RC-1 exposure problem, and raising it before RC-1 lands will overshoot. Validate with an 18% grey card on the forest floor after the lighting fixes.

---

## 7. `src/world/terrain.js`

**P1. Hide the map edge.** `WORLD = 3072` and there is nothing beyond it: `r2_00`'s far side terminates in a ruled straight line with void above. Add an ≥8 km ring of coarse continued heightfield driven by the same ridged multifractal (a few thousand triangles, no LOD churn) behind everything, and cull the playable heightfield's own boundary from every camera on the run. Pairs with the RC-2 fog fix — the last 500 m should dissolve into the horizon colour.

**P1. Landform at human scale.** `RES 1537 / WORLD 3072` = 2.0 m per sample, and there is not one rock-outcrop mesh in the scene, so every feature under ~10 m is band-limited out of existence — no cliff band, no bench, no rock step, no talus apron, no boulder over 0.4 m anywhere in fourteen shots. Note the generator **already computes** `talusU8` thermal deposition, a hardness field and drainage accumulation, and **nothing consumes them**. (a) Nest a ≤1.0 m/sample patch over the trail corridor. (b) Accept that a heightfield cannot make a cliff and scatter authored outcrop meshes off the fields already computed here: bedrock ribs on high-hardness cells, boulder fields on `talusU8`, scree fans below them.

**P2. Skirt material.** Give the chunk skirt its own dark-rock material with its own UVs, double-sided, so an accidental underside view reads as bedrock. Low priority — once `chaseCamera.js` P0 lands this is never seen.

**P2. Perf/hygiene.** `_splitK = 2.05` yields **217** visible chunks against a documented invariant of "~60–100". Geometry cost is negligible (~0.3 ms) but the documented number being off by 2–3x means the LOD tuning was never verified against the running build, and the per-chunk material split above depends on the depth distribution being what the code claims. Raise to ~2.6, re-measure, and add a `renderer.info` assertion to the debug HUD.

**P3. Hero landmarks.** No summit spire, tarn, waterfall, memorable cliff, hut or finish arena anywhere. A player must be able to say "the bit after the big rock." 5–8 authored landmarks placed against the trail corridor at points the chase camera will frame. Real, but milestone 2.

---

## 8. `qa/harness.js`

**P0-a.** Apply the same terrain-height clamp inside `poseOnTrail` that `chaseCamera` gets, so a review shot can never pose the camera underground. Shot 11 asks for `up: 0.9` at `back: 12`, which on a fall-line descent puts the eye metres inside the hill.

**P0-b.** Add a capture assertion that **fails** the shot if more than ~25% of the frame matches the sky's `uGroundHaze` constant. In `r2_11` the bottom 180 rows measure (168, 194, 192) at std 3.2–4.9 — a flat fill that would have been caught automatically.

**P0-c.** Park the bike at the camera, or hide it and drive DoF focus from the trail spline. As it stands the harness silently invalidates every focus-dependent judgement in the review set, and the bike and rider — the two assets that will receive the most viewer scrutiny in a bike game — appear in **none** of the fourteen shots.

---

## 9. `src/world/water.js` — **one small fix, and nothing else**

**P1.** The valley water quad is finite and its far edge terminates visibly in mid-air in `r2_00`. Extend it past the far plane or clamp it to the fog horizon so no edge is ever visible. That is the whole brief for this file.

---

# FINDINGS I AM REJECTING OR DOWNGRADING

**REJECTED — "There is no water rendering at all / water is an untextured flat-colour polygon." (materials, critical; artifacts, critical.)**
`water.js` is 2,519 lines and already implements every item on the proposed fix list: Beer–Lambert per-channel extinction, Schlick fresnel at F0 = 0.02, two scrolling normal octaves (swell + chop), flow-aligned foam with shore band and crest terms, a refraction offset, hand-rolled Toksvig specular AA, streambed pebble parallax, a separate wet-rock contact mesh, and a waterfall mesh. The perf critic independently measured it at **0.13 ms** — it is barely rendering.

The measurement that produced this finding is a misidentification. Materials measured "valley water in `r2_09`" at RGB(170, 197, 195), std 1.8/2.4/2.3. I re-measured: `r2_09` bottom-mid = **(170.2, 196.4, 194.0)**, std 7.6–13.7. The same value appears in `r2_11`'s lower 180 rows at **(168, 194, 192)**, std 3.2–4.9 — and in `r2_11` the camera is underground, looking at a region where there is definitively no water. The lighting and artifacts critics both correctly identified that exact RGB as `sky.js`'s `uGroundHaze` (`fogHorizon * 0.30`). **Three critics measured the same number; two identified it correctly.** Fix `sky.js` P0-d and the "flat water" disappears from every shot. Do not let an engineer rewrite this file.

**REJECTED — "Ground UVs are planar, steep faces are smeared, switch terrain to triplanar." (artifacts, major; terrainform, major; materials, major — "check `uTriStart`/`uTriEnd`, they hand over too late.")**
Triplanar is already compiled in at medium and above, and the blend engages far earlier than claimed. Line 1441: `sT = smoothstep(uTriStart, uTriEnd, 1.0 - gGeoN.y)` with `uTriStart = 0.101`, `uTriEnd = 0.318`. On a near-vertical face `gGeoN.y ≈ 0`, so `sT = 1.0` — **full triplanar**. The blend starts at ~26° of slope and is complete by ~47°. The `r2_02` left face is fully triplanar. Whatever is producing the vertical streaking there, it is not the projection. Re-shoot `r2_02` with DoF, god rays and motion blur disabled before diagnosing it further — my working hypothesis is the radial god-ray/motion-blur streak plus the crushed exposure, not the material. The one place planar UVs genuinely run downhill is the chunk skirt, which is only visible when the camera is underground.

**REJECTED — "The tread is z-fighting with the terrain; add polygon offset." (artifacts, critical.)**
`trail.js:2377-2379` already sets `polygonOffset: true`, `polygonOffsetFactor: -4`, `polygonOffsetUnits: -4`, with `renderOrder = 1`. The diagnostic the critic proposed (nudge the camera and see if the pattern swims) would have come back negative. The real cause is the clamped vertex height at line 2426 — see RC-6. Materials and terrainform were closer, but neither identified the `clamp()` saturation as the specific mechanism. The distinction matters: adding more polygon offset will do nothing, and it is the fix an engineer will reach for first.

**REJECTED — "DoF `focusDistance`/`focusRange` are normalised [0,1]; use `worldFocusDistance`/`worldFocusRange`." (postfx, critical.)**
Backwards for the installed version. In `node_modules/postprocessing`, `worldFocusDistance` is documented *"Deprecated. Use focusRange instead"* and its setter is `set worldFocusDistance(v) { this.focusDistance = v; }`. The CoC shader computes `getDistance(viewPosition)` — view-space metres — and does `signedDistance = distance - focusDistance`. The authored 6.0 and 55.0 already mean metres. Making this change costs a day and alters nothing. The actual defect is the focus **target**, RC-4.

**DOWNGRADED — "r2_09's lower-right quadrant is a flat void; r2_00's horizon is a dead-flat band with zero gradient." (artifacts, critical; lighting, major.)** Partly overstated. `r2_09` bottom-**mid** is genuinely flat (std 7.6) but bottom-**left** is std 59–68, i.e. real geometry; the flat region is ~760x180 px, not a quadrant. `r2_00`'s horizon band measures std 20–40 across the transition rows, not "211 unique colours / dither noise only." The underlying defect (a hard, colourless below-horizon constant) is real and is `sky.js` P0-d; the framing "half the frame is void" is not.

**DOWNGRADED — the adaptive-exposure and vignette findings (lighting, minor).** Correctly self-identified as symptoms. The controller pinned at its 1.50 ceiling is diagnostic evidence for RC-1, not an independent defect. Fix RC-1, re-measure, and expect `comp` near 1.0. Do not widen the clamp as a fix in itself.

**DEFERRED, NOT REJECTED — the terrain landform, trail furniture, section-geometry, understorey, species-progression and hero-landmark findings.** Every one of them is correct and well argued. They are also collectively a milestone of authored content, not a rendering pass. They belong in the next milestone's brief. Attempting them alongside RC-1 through RC-6 will mean none of it lands.

---

## Honest bottom line

This is a genuinely accomplished renderer being judged through three broken switches. The Preetham sky tuning, the volumetric cloud deck, the stochastic tile-break (verified: no visible UV seam at any distance), the water shader, the boulder scatter's power-law distribution, the near-field bark generators, the quadtree LOD ladder and the change-gated PMREM are all at or near the bar and were built by someone who knew what they were doing.

What is in front of it is a shadow floor that collapsed 9x in one iteration, an atmosphere module calibrated for a world that was replaced, a camera that was never written past the scaffold, and a depth-of-field pass focused on a bike that isn't there. Four of those are measured in hours, not weeks. **Fix RC-1 through RC-6 and re-shoot the set before commissioning any of the content work above — a meaningful fraction of these ninety findings will not survive the re-shoot.**