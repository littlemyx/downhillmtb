# DESCENT — Round 3 Consolidated Work Order

## VERDICT: NOT_AAA. Unanimous, and correctly so.

But the seven reviews are wrong about *why* in four expensive places. I re-measured the images and read the source. Corrections first, because three of them would have sent engineers to rewrite files that are not broken.

---

# PART 1 — THE ROOT CAUSE THAT FOUR LENSES MISSED

## R0. The "translucent ghost quads" are the terrain **cavity/crest** term. Nothing is being drawn.

Four lenses filed this as critical against four different files with four different mechanisms:

| Lens | Filed against | Claimed mechanism |
|---|---|---|
| artifacts | `terrain.js` | scree/slab detail meshes with a transparent material |
| postfx | `terrain.js` | scatter/decal quads whose alpha cutout isn't reaching the material |
| materials | `engine.js` | unfinished shadow-cascade split / shadow-caster proxies leaking into colour |
| composition | `engine.js` | shadow-cascade frustum clip or projected shadow-volume quad |

**All four are wrong. There is no quad, no decal, no proxy mesh, and no cascade involved.**

`src/world/terrainMaterial.js:1686–1703`:

```glsl
vec3 dNx = dFdx( vTerrWNrm );
vec3 dNy = dFdy( vTerrWNrm );
float conc = - ( dot( dNx, dPx ) / lx + dot( dNy, dPy ) / ly ) * 0.5;
// The derivative of a smooth-shaded varying is constant per triangle, so fade
// this out where triangles get large on screen or it turns into facets.
float cf = 1.0 - smoothstep( 26.0, 95.0, terrViewDist );
terrCavity = clamp( conc * 14.0, 0.0, 1.0 ) * cf;
terrCrest  = clamp( - conc * 14.0, 0.0, 1.0 ) * cf;
```

The author diagnosed the failure in the comment and then guarded against the wrong variable. `dFdx` of a smooth-shaded varying **is constant across a triangle**, so `terrCavity` is a per-triangle constant. The guard fades it by **camera distance** — but `terrain.js:desiredDepth()` keys LOD to **corridor distance, not camera distance**. An off-trail bank 8 m from the lens and 30 m from the trail spline is depth 5 — 2 m per vertex, ~350 screen px per triangle — with `cf` sitting at exactly 1.0.

It then lands on four consumers:

- `1979`: `terrAO *= 1.0 - terrCavity * uCavityStrength` (0.55) — a **55% flat AO knockdown per triangle**
- `1913`: `terrAlbedo *= 1.0 + terrCrest*0.10 - terrCavity*0.16` — ±16% flat albedo step, and the *crest* branch is why r3_04 reads as an alternating light/dark lattice: the two triangles of a grid quad take opposite signs
- `1760/1765`: biases the **surface splat weights** — different material per triangle. This is the "straight-jointed masonry" on the r3_04 cliff
- `1716`: feeds `terrDrain` → wetness → the "wet sheen wedges"

**My counter-measurement.** r3_08, inside the large triangle vs outside, same material band: `17.8/20.9/22.8` vs `31.4/38.1/41.2`. Ratio **0.567**. Predicted from the shader: `(1 − 0.55·c)·(1 − 0.16·c)` = 0.567 at `c ≈ 0.79`. And decisively — **B/R is 1.28 inside and 1.31 outside.** A real shadow removes the warm sun and leaves blue ambient; chroma must shift. It does not shift. This is a hue-neutral multiply on the indirect term, exactly as the shader says.

It is also present in `r2_04` (I checked), so it is not new — r2's crushed blacks and gameplay DoF were hiding it.

**One fix, ~20 lines, in one file, kills the single loudest "unfinished build" signal in 9 of 16 shots.**

---

# PART 2 — RANKED WORK ORDER, GROUPED BY OWNING FILE

Ranked by impact on "does this look like a console game". Each file goes to exactly one engineer. Two hard cross-file dependencies are called out explicitly; nobody edits outside their file.

---

## LANE A — `src/world/terrainMaterial.js` · **P0**
*Merges: artifacts C1, postfx C1, materials C1+C9, composition C2+major(rock), materials major(relief), materials major(loam), artifacts major(floor albedo)*

**A1. Kill the cavity faceting (highest ROI item in the entire review).**
Replace the view-distance guard at line 1699 with a **triangle-screen-size** guard. `materialForDepth(depth)` (line 2341) already builds one material variant per LOD depth, so bake the chunk's vertex spacing in as a `#define` per variant — depth 7 = 0.375 m, 6 = 0.75, 5 = 2.0, 4 = 4.0, 3 = 8.0 — and fade on spacing vs `terrMPP`:
```glsl
float triPx = VERTEX_SPACING / max( terrMPP, 1e-4 );
float cf = 1.0 - smoothstep( 24.0, 64.0, triPx );
```
Cavity survives only where a triangle is under ~24 px, which is where the derivative is actually meaningful. Additionally set `cavity: false` in the profile for every depth ≤ 6 (`qualityProfile`/`farProfile`), belt and braces. **Depends on Lane B1 landing first** (terrain.js must call `materialForDepth`). Until B1 lands you can validate by forcing `uCavityStrength: 0.0` and confirming all nine shots go clean.

**A2. `farProfile()` must keep `triplanar: true`** (line 2151). As written it sets `triplanar:false`, and the far band starts at 110 m — a cliff at 150 m would go back to planar smearing, which is the r2 defect. Drop only `tileBreak`, `detail`, `cavity`, and `maxLayers→2`. The `gTriW.x > 0.004` branches at 1580/1588 already skip both extra projections on flat ground, so triplanar is near-free out there.

**A3. Rock albedo is ~2x too high.** Measured r3_02 foreground scree `201/184/168` = 79% sRGB — brighter than most snow references, and it is the reason the exposure is dragged up and the far peak reads dark. `cLight 0x9a978f` → `0x6f6c66`; `cQuartz 0xb9b6ad` → `0x8e8b84` (line 629–631). Re-tune `cDark`/`cMid` to hold internal contrast. Target sunlit rock at 0.35–0.45 albedo.

**A4. Finish the LOAM lift the comment asks for.** `LOAM_ALBEDO_LIFT = 0.030` (line 134) with a comment saying it is "deliberately about half way to the 10–15% target, TO RE-TUNE AFTER RC-1 LANDS". RC-1 landed in r2. Raise to **0.060**. This is the mechanism behind three separate findings: the near-black forest floor, the cold blue tread (see A5), and the acid-green-grass-on-void value break. **Must land in the same build as Lane F5** or the grass will still float.

**A5. The cold tread is an albedo problem, not a material-assignment problem.** I replicated the materials lens numbers exactly (r3_01 tread B/R 0.62 sunlit; r3_07 B/R 1.91). But r3_07 is the `loam` phase using `Surface.LOAM`, not ROCK/GRAVEL — see rejections. A LOAM base at ~4% linear takes its hue wholesale from sky ambient. A4 should move it; **re-measure after A4 before changing any palette.** Target 0.70–0.85 B/R in shade. r3_05 (`slab`/ROCK) and r3_06 (`creek`/GRAVEL) genuinely do sit on neutral layers — those need a warm hardpack base splatted through, not replaced.

**A6. Near-field relief.** Measured r3_13 tread at ~1.5 m: pixel-scale / 32-px-scale std ratio **1.43**, *lower* than the same material at ~10 m in r3_11 (1.69). Relief is getting worse as the camera closes. Calibrate the height→normal derivation to an explicit per-layer world amplitude instead of the bare `BUMP_SCALE` scalar (line 102), raise `uDetailAlbedo` 0.22 → 0.35, and validate with a grazing-incidence render at 5/10/20°.

**A7 (minor).** Snow patches in r3_01 are smooth flat ellipses. Drive coverage from slope + curvature + a wind dot product with a height-blended margin, not decals.

---

## LANE B — `src/world/terrain.js` · **P0**
*Merges: perf C2+C4, materials C3, materials major(edge), artifacts major(carve wall), composition minor(terracing)*

**B1. One line. Do it first and push it on its own.** `terrain.js:1996`:
```js
const mesh = new THREE.Mesh(geo, terrainMat.userData.materialForDepth
  ? terrainMat.userData.materialForDepth(depth) : terrainMat);
```
`materialForDepth` (terrainMaterial.js:2341) has existed since r2 with an `ACTION REQUIRED` contract note at the head of the file and has never been called. This is the enabler for Lane A1 **and** the largest single fragment-cost saving in the frame. Coordinate: Lane A needs it before A1 can ship.

**B2. `castShadow` is gated on a feature that does not exist.** `terrain.js:2006` keys casting to `settings.cascades > 1`; `high` sets `cascades: 3` — but `sky.js` implements exactly **one** cascade over `SHADOW_RANGE.high = 150` and says so in its contract note (sky.js:36). So ~100–120 chunks and ~460 k triangles, including depth-3/4 nodes 400 m–1.5 km out, are vertex-shaded into a 150 m shadow volume and clipped. Set it per-frame in `selectNode()` alongside `mesh.visible`:
```js
n.mesh.castShadow = shadows && dist < ctx.sky.shadowDistance + n.size;
```
Zero pixel change. ~100 draw calls and ~460 k triangles out of the shadow pass.

**B3. Ruts and braking bumps are authored below the grid's Nyquist.** Ruts are 0.028 m deep at σ = 0.19 m on a 0.35 m corridor detail grid — roughly one sample wide, smoothed to nothing before it reaches a vertex. Raise rut amplitude to **0.08–0.15 m**, σ to **≥0.35 m**, braking bumps to **0.06–0.12 m**; or refine the corridor grid below 0.35 m. Measured consequence: across-track/along-track gradient ratio is **0.57–0.99** (r3_11 0.66, r3_13 0.57, r3_07 0.99). A ridden line reads 1.5–2.5. Below 1.0 means the surface varies no more across the tread than down it — it is isotropic noise.

**B4. Trail edge raggedness has one octave.** `nMat(x*0.55, z*0.55)` ≈ 1.8 m wavelength gives a gentle sine, not raggedness. Add octaves at ~0.4 m and ~0.12 m, widen the `w > 0.14 … w < 0.42` transition band, and bias asymmetrically — more breakdown on corner exits and above braking zones.

**B5. Carve walls are a boolean subtraction.** 4–5 m near-vertical walls with a razor-straight top edge tracking the spline (r3_11, r3_09, r3_08). Add a post-carve slump: taper to angle of repose (35–40° soil), round the shoulder over 0.5–1 m, deposit a talus wedge at the foot, and add lateral noise to the carve width.

**B6 (minor).** Heightmap terracing on the left slope of r3_11. Raise precision to 16-bit/float and add high-frequency detail displacement to break the step contours.

---

## LANE C — `src/entities/rider.js` · **P0**
*Merges: character C2+C3+major(cloth)+major(hands)+major(pose)+major(scale)+minor(self-shadow)+minor(helmet), composition C1, postfx major, artifacts C3, materials major, perf major*

The rider is on screen in **100% of gameplay frames** and is placeholder-grade. I opened the salvaged full-res crops — every claim below is confirmed by eye, not inherited.

**C1. There is a hole through the protagonist's head.** `rider.js:1465`: `side: THREE.DoubleSide, // the eye port is a real hole; you see the liner`. From the default chase camera (r3_10) you look straight through the open eye port at the unflipped interior backface of the front shell — a large flat pale plate, the highest-luminance element on the character, in every frame. Composition and postfx both mis-read this as "a blank white face"; it is the *inside of the head*. Close it with an inner liner cap with correct inward normals, or go `FrontSide` and model the port as real thickness. Verify on a full 360° turntable at 2 m, not from the front.

**C2. Bake the garments into a UV albedo atlas.** The model is *not* short of authored detail — knee-pad shell, shorts hem, sock, shoe sole, laces, glove cuff, jersey panels all exist — but every one is painted as **per-vertex colour on a low-density swept loft**, so a 2 mm seam interpolates across a triangle into a 40–80 px gradient. Bake the existing per-vertex colour functions to a 1–2 K UV atlas and sample it as `map`; keep vertex colour for broad tint only. Same atlas carries the AO the character has none of (C6). This is the single largest quality gap on the hero.

**C3. Hands and feet — the two contact points a viewer checks first.** Gloves, shoes and knee pads are currently the *same* knobbly ellipsoid; you cannot tell one from another by shape or material. In the salvage crop the bar passes **through** the glove volume and the grip re-emerges outboard. Model a wrapped hand (four merged finger tubes + a thumb over the top) and a flat-soled shoe whose sole plane mates with the pedal cage. Spend the polygons here if the budget is tight.

**C4. Re-author the rest pose.** Spine convex, hips above the BB rather than behind the saddle, head below shoulder height pointing at the front wheel, elbows dropped and tucked. It is a commuter perched on a DH bike. Rotate the pelvis back ~15° and rearward, flatten the thoracic spine, raise the eyeline above the bar, widen the elbow IK pole targets so the upper arms sit outboard of the torso silhouette.

**C5. Cloth normal is below Nyquist.** `tex.kit` at `normalScale 1.0` (line 1407) with a UV scale that puts the weave at 1–2 screen px — it aliases into moiré, visibly in r3_10 as vertical stripe shimmer across the shoulders. Drop the weave UV scale 4–8× so a cell is 6–10 px at 3 m, mip it, enable anisotropy. Then add the macro information a viewer actually reads cloth from: folds at the elbow crook, gather at the waist, a wind-lifted hem.

**C6. Bake AO into the atlas** (chest underside, armpit, inner arm, crotch, under the shoulder pad) and **weld the chest armour into the jersey shell** rather than intersecting it — there is a visible boolean seam at the shoulder line.

**C7. Scale down ~12%** against the fixed wheelbase and reduce thigh/calf radii. Measured helmet-crown-to-pedal 222 px vs 107 px front wheel; against a 0.74 m DH wheel that is a ~1.53 m rider crouched in attack position, where real is 1.35–1.40 m.

**C8 (perf, free).** Collapse four `MeshPhysicalMaterial` programs to two. `skin` sheen 0.15 and `helmet` clearcoat 0.35 are not resolvable at chase distance; `lens` iridescence 0.7 (line 1477) is the most expensive branch in three's physical shader, paid for a ~600 px mirror. Keep sheen on `kit` only; move `skin`/`helmet` to Standard; `lens` → Standard metalness 1.0 / roughness 0.07 / envMapIntensity 1.9. Keep a photo-mode swap.

**C9 (minor).** Helmet peak is a zero-thickness plane — give it 3–4 mm of extrusion and a rolled edge. Raise shell tessellation (visible 8–10 segment silhouette at 1.5 m). Add vents and a chin strap; the strap alone does most of the work of making a helmet look *worn* rather than dropped on.

---

## LANE D — `src/world/vegetation.js` · **P0**
*Merges: vegetation C1+C2+C3 and all majors, postfx C2+C5+minor, composition major(dither)+major(groundcover), artifacts major(alpha fizz)+minor(groundcover), materials minor(stipple), perf major×2+minor*

**D1. Needles are authored sub-pixel. Verified arithmetic.** `ATLAS=2048 / ATLAS_CELLS=4` → `CELL_PX = 512` (lines 98–100). `needlesAlong(... S*0.0032 ...)` with `S = CELL_PX` → **1.64 px wide**, `needleLen 0.0115` → 5.9 px long. A near branch card renders at ~200 screen px, i.e. a 0.39× downsample of the 512 px cell, so a needle is **0.64 screen pixels at the closest distance the game ever presents.** It can never resolve. Everything else in this lane is downstream of it. Either go to a 4096 atlas re-celled so a needle is 3–4 px in atlas space, or drop to branchlet-scale cards (0.6–0.9 m) with more cards per branch. Nothing in lighting, LOD, AO or post can rescue a sub-pixel needle.

**D2. The 1-bit alpha stipple storm.** `FOLIAGE_ALPHA_TEST = 0.34` (line 185) applied to a canvas-antialiased 1.64 px stroke discards the outer ~40% of every needle's alpha ramp. `alphaToCoverage` is gated on `msaaLive`, which is `quality === 'ultra'` only (line 3161) — at the tier these shots ran there is no coverage dithering at all. Result: 559 separate sky holes in a 340×340 crop of r3_12, 79% of them ≤8 px. Three changes together: drop the threshold to 0.15–0.20 and re-solve `buildCoverageMips` at the new threshold (the coverage-preserving mip machinery is already there, it is just being fed a threshold that eats the source); enable `alphaToCoverage` at `high` with 2× MSAA, forcing `material.transparent=false, depthWrite=true` to stop three reclassifying it into the transparent pass; and fix D1.

**D3. The near canopy is visibly flat cards.** `SPECIES` sets `cards:3, cardWidth 0.34–0.38` for every kind. In r3_06 one card spans ~750 px — **39% of frame width** — as a single unbroken green mass with a straight diagonal top edge and a flat unlit interior. Raise to `cards 6–8` at `cardWidth ~0.16` on the near tier only, so no single plane exceeds ~10% of frame width. Add a real per-card AO/depth gradient driven by distance from the trunk axis and whorl index. Buy the triangles back from the mid tier.

**D4. Foliage is one lime green.** Measured hue median 106° at HSV saturation 0.453 over 834 k pixels in r3_07; conifer needles in sun sit at 75–95° and 0.20–0.35. Drop saturation to 0.25–0.32, shift base hue to 85–95°, add per-instance hue jitter ±12° and value jitter ±15%, and give the shadow side a genuinely cooler hue rather than the same hue darkened. **Must land with Lane A4** — this is the other half of the grass-vs-void value break.

**D5. Ground vegetation is one asset on a lattice.** `buildGrassTuft()` is called once (line 3110) and registered as a single kind (3509); placement is `stationScatter()` on a `latStep = 1.30/√density` lattice. The file's own comment at 3514–3517 condemns exactly this for trees — *"a lattice however hard you jitter it… can produce neither a thicket nor a glade, only a uniform stipple at one density"* — then fixes it for trees with a Thomas cluster process and leaves grass on the condemned lattice. Apply the same Thomas process (parent lattice 3–5 m, ~60% parent rejection, 15–40 children, σ 0.4–1.0 m) and add four ground kinds: bunchgrass, sedge, low fern, moss/duff mat. Give the blade a midrib and a translucency response.

**D6. Ground the vegetation in the asset, not the post pass.** Bake a short vertical AO gradient into the bottom 15–25 cm of every grass/fern/shrub geometry (vertex colour, zero runtime cost) and write a small darkening ring at each trunk base in the scatter pass. Do **not** ask postfx for this — see rejection R10.

**D7. Bark, root flare, snags, stubs.** `BARK_GEN = 256` for a 25 m trunk mirror-tiled at 2.5–5× is ~50 px/m before mip loss; measured high-frequency RMS on a 2 m trunk in r3_06 is **3.02**, *lower* than the trail dirt beside it (4.49). Photographic bark plates ≥2048 for the near tier with a real height map driving parallax; add root flare to `buildConifer` (trunks currently terminate as a cylinder cut by the terrain). Drop snag placement weight to ~1 in 12–15 and cluster them (currently 6 bare poles to 15 living trees in r3_12 — reads as failed-to-load LOD, not as beetle kill). Route the dead branch stubs through the deadwood/bark material at roughness ~0.95 tinted off `barkTint` not `needleTint` — right now they are pale mint sausages with a plastic highlight at eye level against sky (r3_09), five minutes' work for disproportionate payoff.

**D8. Imposter/mid/near are three different plants.** Measured r3_02: near sat 0.34 / val 0.56 / hue 103° with layered spiky card structure; far imposters sat 0.15 / val 0.76 / hue **117°** as smooth mint cones. A 14° hue swing toward cyan and total loss of silhouette character is a tier mismatch, not atmosphere. Re-render the imposter atlas from the near geometry under the same lighting and match medians with the existing `medianRGBOver`. Carry more whorl raggedness into the mid tier.

**D9. Moss/shrub patches are unlit lime decals** sitting *above* sunlit trail luminance with a hard stippled cut edge (r3_11, r3_10). Put them on the same lit path with a proper normal and roughness, drop albedo two stops so they sit below the trail in value, soften the cut.

**D10 (perf).** Shadow tiers are packed to the LOD band, not the shadow slice: `TREE_TIERS[1]` runs 32–250 m with `shadow:true` against a 150 m cascade, `woodLayer` is `ONE_TIER(180,…,true)`. Sort chunks so `dist − radius < shadowReach` occupies the head of the instance buffer, record `userData.shadowCount`, and use `onBeforeShadow`/`onAfterShadow` to clamp `mesh.count`. Delete the duplicate `MAX_SHADOW_LENGTH` constant and read `ctx.sky.shadowDistance` — two independent copies of the same number guarantee drift. Also: set explicit `renderOrder` (terrain/rock 0, trail/bike/rider 1, alpha-tested foliage 2) so opaque geometry lays depth first — alpha-test discard disables hidden-surface removal on TBDR, and `DoubleSide` doubles the fragments on top of that. Delete the dead `grassDepth` material (built at 3211, never bound because the grass tier is `shadow:false`) and add the build-time assertion that would have caught it.

---

## LANE E — `src/world/sky.js` · **P1**
*Merges: character C1 (corrected), artifacts C2 (corrected), materials minor(aerial), postfx C3+major(fog), composition major(cloud shadows)*

**E1. The hero cannot cast a contact shadow — and it is not a flags problem.** `rider.js:1717/1733`, `bikeModel.js:2168–69/2315` and `trail.js:3522/3536` all set `castShadow` correctly. The cause is texel size. At the gameplay fit (`SHADOW_RANGE.high = 150`, `mapSize 2048`, sphere-fitted) the ortho spans ~310 m → **~0.15 m per texel**, and `normalBias = clamp(texel*0.18, 0.015, 0.12)` (line 1962) → **0.027 m**. A 30 mm frame tube, a 20 mm stanchion and a 60 mm forearm are all under one texel plus bias. Add a **dedicated tight slice** (second shadow-casting light or a real second cascade) whose ortho frustum is fitted per-frame to the bike+rider bounding box — a 4 m box at 1024 gives ~4 mm texels — with `normalBias ≈ 0.004` on that slice only, terrain cascade untouched. *Note for the character reviewer: the `far 4000` / `normalBias 0.12` numbers you quoted are the aerial-adaptive branch (`SHADOW_RANGE_MAX_MUL = 8`), not the gameplay fit. Conclusion unchanged, arithmetic corrected.*

**E2. Sky is over-bright and under-saturated — but it is not absent.** Measured r3_10: zenith `R0.72 G0.84 B0.92` (B−R = 0.199) with a real gradient to horizon. r3_01 and r3_00 are pale because they are horizon-band framings, which is physically correct. The genuine defect is narrower than "no sky": zenith luminance ~0.82 and saturation ~0.22, where a clear alpine zenith wants ~0.55–0.70 and ~0.5–0.6. Re-expose and saturate the existing model; give clouds separate lit/shadowed shading so bases sit 0.2–0.3 below tops. **Do not rewrite the sky as a new Rayleigh/Mie model** — see rejection R8.

**E3. Fog colour in the far band.** r3_00 distant foliage measures hue 147° at saturation 0.328 — a saturated mint-teal. Real distance shifts foliage *toward the sky hue* (~210°) while collapsing saturation toward 0.10; a 40° swing into cyan at held saturation is a blend-space/constant-tint bug. Composite fog in linear space against actual sky radiance in the view direction. Target r3_00 p0.1 luminance at 0.03–0.06 (currently **0.292** — the establishing shot has no black point at all).

**E4. Cloud shadows on the terrain.** Absent in all 16 shots under visibly broken cumulus. Project a scrolling cloud-shadow mask onto the sun term. This is the primary scale cue in FH5 and RDR2 and its absence is a major reason the mountain reads small.

**E5 (perf, free and exact).** Hoist `ph0/ph1/ph2` out of the `for (i < CLOUD_STEPS)` loop in `marchClouds()` — `cosT = dot(rd, sd)` is constant along the ray and the g values are literals, so six `hgPhase()` calls × 32 samples produce bit-identical results when hoisted once per pixel. Then render the cloud layer at quarter resolution into its own half-float RT with a Bayer dither on `tStart` and bilateral-upsample on the layer's own depth. Clouds are 3–5 km out with no high-frequency content. Expect 2–4 ms in the wides.

---

## LANE F — `src/entities/bikeModel.js` · **P1**
*Merges: character major×4, perf major*

**F1. The value hierarchy is inverted.** The fork lowers are brass-gold with a hard mirror streak and the rims are near-white mirror hoops — the two highest-contrast, most saturated elements in the entire frame, and the least important parts of the bike. Anodised alloy is matte-to-satin. Lowers → roughness 0.35–0.45, desaturated off pure brass; rims → matte anodised dark grey/black at roughness 0.4. Confirm a PMREM environment is actually bound (it is, from sky.js) so the metals have something to reflect instead of collapsing to flat bright gradients.

**F2. The frame has no identity.** Flat red, one broad terminator, no clearcoat, no decals, no weld beads, no gussets, no head badge — it reads as red plastic drinking straws. Add a clearcoat layer, a decal atlas (down tube / top tube / head badge) and visible weld and gusset geometry at the head tube and BB junctions. In a game where the bike is the product this is not cosmetic.

**F3. The brake hose terminates in mid air.** Confirmed in the salvage crop: a single black hose leaves the bar, runs past the head tube and stops beside the down tube without reaching a caliper. There *are* calipers and rotors present at both wheels — the character lens said there were none, which is wrong at this angle — but the hose is unattached, which is worse than no hose. Route it to the caliper body. Separately: **there are no lever blades on the bar**, only two grey clamp boxes. Six triangles each and it is the difference between a cockpit and a bare tube.

**F4. Spokes, rim edges and chain are sub-pixel geometry with no AA strategy** — broken dotted white fragments in a still, and they will crawl and sparkle in video, which is how this game gets sold. Give spokes and chain a screen-space minimum width (expand in the vertex shader so nothing falls below ~1.5 px) and swap the spoke set for a single alpha-textured disc beyond ~4 m.

**F5. Confirm `buildSaddle` reaches the merged mesh.** `G.saddle`, `saddleLen` and `buildSaddle` exist; no saddle survives to the r3_12 side silhouette. If the rider's thigh is occluding it, that is itself evidence the leg volume is oversized (see C7).

**F6 (perf).** 18 `THREE.Mesh` calls + an InstancedMesh chain, all `castShadow`, no LOD — 38 draw calls, five of them forcing the morph-attribute depth variant. At 120 px tall in r3_07 the 32 three-cross spokes, the ~40-link chain and the brake-pad squeeze morphs are all sub-pixel, and in a 0.18 m shadow texel the whole bike is a 4 m smear. Merge everything rigidly parented to `chassis` that never articulates; build one merged low-poly shadow proxy (frame silhouette + two solid wheel discs) and set `castShadow=false` on the 18 detail meshes; gate the chain and spoke sub-geometry off past ~15 m.

---

## LANE G — `src/camera/chaseCamera.js` · **P1**
*Merges: character C4, composition major×2, postfx minor, artifacts major*

**G1. The hero cameras cannot photograph the hero.** Confirmed by eye. r3_13 is briefed "CLOSE on the bike, side on" and contains a front-tyre arc and a sliver of fork in the bottom-left corner — under 7% of frame, amputated by two frame edges, with 93% given to empty forest and two ribbons of course tape. r3_12 crops the front wheel at x=0. r3_11 buries the entire bike and both legs behind a terrain lip. r3_15 hides the bike behind its own rider. **Four of sixteen.** Replace the fixed offsets with a solver: project the bike+rider bounds each frame, push the camera back and re-aim until the bounds sit inside a safe rect (~70% of frame height, fully inside 5% margins), raycast to confirm the bounds are unoccluded, and lift the camera when the hero's lower half is buried by a crest. For the low-rear preset drop the azimuth 15–25° off dead centre.

**G2. Nine of sixteen shots are dead-centre.** Rider within ~3% of frame centre horizontally, trail vanishing point also centred, horizon within a few percent of the midline. That reads as a debug camera. Add lateral lead from steering/velocity so the rider sits opposite the direction of turn, bias the horizon off the midline by whether the shot looks up or down the fall line, and let the camera pass close enough to trailside geometry that trunks and banks enter frame as foreground occluders.

**G3. Re-pick the r3_00 establishing camera.** It currently looks *down and out to sea* — a flat forested shelf against a featureless blue plane, with the world edge visible. Aim it across the massif so a ridgeline and the run's fall line are both in silhouette. This is one camera choice, not a rendering problem, and it is the first image anyone sees.

---

## LANE H — `src/world/trail.js` · **P1**
*Merges: artifacts C4+major(tape)+major(props), composition major(furniture), materials C3(wear), postfx major(tape)*

**H1. The tape posts are a dark metal, which is why they read as black bars.** Not `MeshBasicMaterial` and not unlit — see rejection R4. `trail.js:3505` appends `gPost` into the **`metalMat` builder** (`metalness 0.85, roughness 0.38`, line 2893) with vertex colour `[0.22,0.23,0.25]`. A dark metal has essentially no diffuse term, so the cylinder has no terminator. Give the tape posts their own dielectric material: albedo 0.05–0.08 linear, roughness 0.7, **metalness 0**. Sink the base 5–8 cm and add a dirt-mound/contact decal — they currently stand on a flat cut ellipse.

**H2. Tread wear signal in albedo and roughness.** Lane B3 restores the *geometry*; nothing writes a *wear* signal into the material. Drive it off the same lateral coordinate that drives the ruts: darker, smoother, lower-contrast over the centre 40% of the width; lighter, rougher, looser grit toward the edges. This single gradient is what makes a tread read as ridden, and it is what the 0.57 anisotropy measurement is missing.

**H3. Props are placed at spline-relative height, not raycast.** The r3_04 CP4 gantry support pole ends in mid air with rock visible below the cut (I confirmed this in the crop) and the black tape post beside it floats. Fallen logs in r3_02 lie on a **near-vertical cliff face**. Raycast every prop base to the terrain collision mesh, seat at or slightly below the hit, reject log placement on slopes >30° and on positions that sphere-overlap an already-placed log, and extend post geometry 20–30 cm below the hit so a later LOD shift cannot expose the cut end. Re-run placement *after* the final terrain LOD/erosion pass.

**H4. Tape aliasing at distance.** DOWNGRADED from the artifacts lens's "clipped to pure white 255,255,255" — I measured max **208–211** in the r3_15 run, and near tape in shade measures `45/55/57`, so it is lit and it is not clipped. What it is: sub-pixel ribbon width losing the red stripe to aliasing, plus `tapeMat` carrying `transparent: true` *with* `alphaTest: 0.35` (line 2925), a three.js foot-gun that puts it in the depth-sorted transparent pass. Set `transparent: false`, keep `alphaTest`, and give the ribbon a screen-space minimum width.

**H5. Course furniture.** Per-segment randomised sag and twist on the tape (currently an identical mechanical zigzag between every post pair), slight per-post lean and baseplates, a real truss gantry with a sagged double-sided fabric banner, and directional arrow boards in place of the crossed planks that currently read as grave markers.

---

## LANE I — `src/core/postfx.js` · **P2**
*Merges: postfx C4+major(DoF)+minor(dither), perf major×2*

**I1. No white point in the forest interiors — and only there.** DOWNGRADED from "no white point anywhere". I measured all 16: %pixels > 0.98 is 0.42 (r3_01), 0.50 (r3_02), 0.87 (r3_04), **3.25** (r3_05), 0.63 (r3_12), 0.71 (r3_15). Only r3_06 (0.01%), r3_07 (0.00%) and r3_13 (0.00%) are flat, and those are precisely the signature shots — r3_07 p99 = **0.778**, r3_13 p99.9 = **0.899**. A forest in direct sun with no pixel at full value has no sun in it: no wet-rock highlight, no rim on the rider's shoulder, no glint on the stanchion. Pull the tonemap shoulder start higher so 0.4–0.8 keeps its separation and 0.5–2% of a forest frame clips.

**I2. Re-enable DoF as far-field-only.** Deleting it in r2 was the right instinct applied absolutely. Focus locked to the rider, near plane **disabled entirely**, far blur ramping from 0 at ~30 m to a max CoC of 3–4 px beyond 120 m. Right now the busiest, highest-contrast pixels in r3_07 are the aliased leaf edges *behind* the rider. Add a gentle vignette (~0.15 corner falloff) and 1–2% grain.

**I3 (perf, byte-identical output).** `AdaptiveExposureEffect` has its own `EffectPass` (line 918) on the stated ground that "everything after it must see the exposed image". `EffectPass` chains `mainImage` calls in order, feeding each the previous blended result, and the effect declares no `EffectAttribute`, so it is mergeable. `passes.hdr = new EffectPass(camera, fx.exposure, fx.toneMapping, fx.lut)`; delete `passes.exposure`. That is a full-res RGBA16F read+write plus a composer ping-pong for one scalar multiply. 0.4–0.7 ms at 1080p, 4× that at DPR 2.

**I4 (perf).** MotionBlur is 12 taps at full res, each doing a colour fetch **and** a depth fetch + `getViewZ` — 24 dependent fetches per pixel over the whole frame — and the hysteresis gate opens at 9.2 m/s, so it is on from the first straight to the finish. The effect is by construction low-frequency (`blurConfig.y` clamps the streak to ≤0.05 uv). Run it at half res, or drop the depth tap to every second sample and scale `MB_SAMPLES` with the actual clamped offset length. ~1 ms.

**I5 (minor).** Add ordered/blue-noise dither at the final 8-bit stage (±½ LSB). The r3_01 sky uses only 64 distinct red levels over a 186–249 range. It is currently masked by how pale the sky is; **do this before Lane E2, not after**, or the same quantisation will spread over a much wider colour distance and become obvious.

---

## LANE J — `src/world/water.js` · **P2**

**J1. The creek is a blown, structureless white ribbon.** I traced the band across r3_06 at eight x positions: `226/228/226` at x40 through `225/225/231` at x760 — flat neutral L≈226 over 900 px with no depth ramp, no refracted bed, no flow direction, no bank interaction, no wetted margin. It is the shot literally named "creek crossing" and there is no water a viewer would identify as water. Add a depth-based absorption ramp, a refracted bed sample, flow-aligned animated normals at two scales, a Fresnel-weighted reflection, foam at obstruction, and a wetness darkening band within ~0.5 m of the waterline. Clamp specular so it cannot exceed L=200 in shade. *(The artifacts lens attributed this band to course tape mipping to white — that is wrong; `TRAIL_PHASES` gives the `creek` phase `tape: 0.0`, and the band is neutral with no red anywhere along its run.)*

**J2 (minor).** The valley water body in r3_00/r3_01/r3_08 is a flat untextured plane meeting the sky at a perfectly straight edge — a debug backdrop, and it is half the establishing frame. Give it a wave normal, a sun specular lobe and a shore blend, or move it below the terrain skirt and replace with distance-faded haze.

---

## LANE K — `src/core/engine.js` + `qa/harness.js` · **P0 for the re-shoot, P2 for the rest**

**K1. `qa/shots/r3_14-rider-three-q.png` must be re-captured before anything in Lane C or F is signed off.** It is not a crop — the shipped file is **260×300 and 100% transparent black** (alpha mean 0.0, RGB mean 0.0). Every r3_14-based finding in three reviews rests on salvaged crops in `/private/tmp/claude-501/-Users-stewartwebster-Projects/1bc47d7b-.../scratchpad/r3_14_salvage/`, which I read and which do support the findings. Re-shoot it, and add a post-capture assertion in `harness.js` that a shot is 1920×1080 and non-degenerate.

**K2. `measure()` does not pin resolution.** `shoot()` forces `setPixelRatio(1)` at 1920×1080 and restores `prevPR` on exit (harness.js:130–158); `measure()` (269) never sets it. With `PIXEL_BUDGET.high = 9.2e6` (engine.js:47), a 1080p CSS window on a Retina panel resolves to DPR 2 and an **8.29 MPix** backbuffer — 4× the 2.07 MPix the shots were captured at. So "63 fps / 15.85 ms" describes a different image from the one being reviewed. *Mild correction to the perf lens: `measure()` does already return `size`, so the number is recoverable — it just was not reported.* Pin `measure()` to 1920×1080 DPR 1, re-measure, and quote render and CSS resolution separately. Then set `PIXEL_BUDGET.high ≈ 4.2e6` (DPR ~1.42 at 1080p, all the extra sharpness SMAA at HIGH can use) and add a frame-time-driven ratio governor that sheds 1/16 when the 30-frame mean exceeds 15 ms.

**K3 (minor).** `renderer.shadowMap.type = THREE.PCFSoftShadowMap` (engine.js:383, and 631 in the context-restore path). PCFSoft derives its filter radius per fragment from screen-space derivatives; at sky.js's texel-snapped 0.15 m density that radius is sub-texel and the two filters are indistinguishable in all 16 shots. Switch to `PCFShadowMap` and spend the recovered time on `shadowMapSize`, which actually helps.

---

# PART 3 — REJECTED AND DOWNGRADED

| # | Claim | Lens(es) | Ruling | Counter-evidence |
|---|---|---|---|---|
| R1 | Translucent decal/scatter/proxy quads drawn over the terrain; audit decal materials, cull proxies, debug cascade splits | artifacts C1, postfx C1, materials C1, composition C2 | **REJECT mechanism (4×)** | It is the per-triangle-constant `terrCavity`/`terrCrest` term, terrainMaterial.js:1686–1703, guarded by view distance while LOD is keyed to corridor distance. Predicted multiply `(1−0.55c)(1−0.16c)` = 0.567 at c≈0.79; **measured 0.567**. Chroma unchanged inside vs outside (B/R 1.28 vs 1.31) — a shadow would shift it. Present in r2 too. Four engineers would have hunted meshes that do not exist. |
| R2 | `castShadow` not set on bike/rider/posts; traverse the hierarchy at spawn | artifacts C2 | **REJECT** | rider.js:1717/1733, bikeModel.js:2168–69/2315, trail.js:3522/3536 all set it. Cause is 0.15 m shadow texel + 0.027 m normalBias vs 30 mm tubes. → Lane E1. |
| R3 | Terrain has no triplanar projection; implement three-axis world-space sampling | artifacts major | **REJECT** | `TERRAIN_TRIPLANAR` is defined at `high` (terrainMaterial.js:2136) and `gTriW` blending runs at 1558–1684. The apparent cliff "smearing" is R1 faceting plus over-bright rock albedo. |
| R4 | Tape and posts are unlit / `MeshBasicMaterial` / metalness 1 roughness 0 with no envmap | artifacts C4 + major | **REJECT mechanism, keep the fix** | Both are `MeshStandardMaterial` (trail.js:2887–2930). Posts read black because they go into `metalMat` (metalness 0.85, roughness 0.38) at vertex colour 0.22 — a dark metal with no diffuse. → Lane H1. |
| R5 | Tape mips to fully clipped white RGB 255,255,255 | artifacts major | **DOWNGRADE** | Measured max 208/209/211 in the r3_15 run; near tape in shade 45/55/57. It is aliasing plus a `transparent:true`+`alphaTest` misconfiguration, not clipping. |
| R6 | The white band across r3_06 is course tape mipping to white | artifacts major | **REJECT** | `TRAIL_PHASES` gives the `creek` phase `tape: 0.0` (trail.js:123). Band is neutral L≈226 with zero red over 900 px. It is water.js. |
| R7 | Aerial perspective inverted; distant terrain is outside the shadow cascade — extend shadow distance and refit the fog curve | composition major | **REJECT mechanism** | Measured r3_02: fg scree L187 (`201/184/168` = 79% sRGB), far peak L103. The far peak is not too dark; the **foreground rock albedo is ~2× too high** (`cLight 0x9a978f`, `cQuartz 0xb9b6ad`). Do not touch the cascade or the fog for this. → Lane A3. |
| R8 | There is no sky gradient and no blue; rebuild the sky as a Rayleigh/Mie model | postfx C3 | **DOWNGRADE** | r3_10 zenith `R0.72 G0.84 B0.92`, B−R = 0.199, with a real zenith→horizon ramp. r3_01/r3_00 are pale because they are horizon-band framings — physically correct. Real defect: luminance too high, saturation too low. Re-expose, don't rewrite. → Lane E2. |
| R9 | No white point anywhere in the set | postfx C4 | **DOWNGRADE to forest interiors** | %>0.98 measured: 0.42/0.50/0.87/**3.25**/0.63/0.71 across r3_01/02/04/05/12/15. Only r3_06/07/13 are flat. Real, but narrower than filed. |
| R10 | There is no AO anywhere; add SSAO at 0.3–0.8 m radius | postfx major | **DOWNGRADE mechanism — do not re-raise it** | N8AO is live at radius 0.55 m, intensity 1.1, half-res (postfx.js:794–823). It was deliberately weakened in r2 because at radius 1.6 / intensity 2.8 it was *the single largest contributor to the shadow-floor collapse* — the P0 this round is praised for fixing. Re-raising it re-opens r2's P0. Buy grounding from baked/vertex AO and contact decals per asset. → Lanes D6, C6, H1, H3. |
| R11 | The ambient/IBL is pure sky colour with no ground bounce; add an irradiance volume or hemispheric ambient | composition C3 | **REJECT** | sky.js already has a PMREM with a ground disc (1388), a `HemisphereLight(0x9dc0f0, **0x54452f**, 0.42)` (1510) and an `AmbientLight` (1517). The blue-shade read is caused by terrain albedo being so low (LOAM ~4% + 0.030 lift ≈ 6.8%) that the warm bounce contributes almost nothing. Fix albedo, then re-measure. → Lane A4. |
| R12 | Fog is evaluated per-tree-instance; move it per-pixel | artifacts major | **DOWNGRADE to unverified** | Better explained by the imposter atlas being desaturated and hue-shifted 14° toward cyan (vegetation D8, which I accept). Do not re-plumb fog before re-rendering the imposter atlas. |
| R13 | The forest phases hand the tread to the neutral ROCK/GRAVEL layers | materials C2 | **PARTIAL** | The B/R numbers replicate exactly (0.62 sunlit → 1.91 in r3_07). But r3_07 is the `loam` phase on `Surface.LOAM` (trail.js:124), not ROCK/GRAVEL. Cause is LOAM's dark base letting sky ambient dominate. r3_05 (`slab`/ROCK) and r3_06 (`creek`/GRAVEL) do sit on neutral layers and need a warm hardpack base splatted through. |
| R14 | Trees throw hard crisp shadows in r3_12, proving only the hero is missing from the pass | character C1 | **DOWNGRADE** | I find no legible tree shadow on the trail in r3_12. The dark shapes on the right bank are R1 cavity facets. The correct statement is that *nothing* in the 150 m slice casts a legible contact shadow. Conclusion unchanged; the supporting claim is not. |
| R15 | `farProfile()` as authored is ready to ship | perf C2's own caveat | **UPHELD — the perf lens is right to warn** | It sets `triplanar: false` and the far band starts at 110 m, so a 150 m cliff would return to planar smearing (the r2 defect). Keep `triplanar: true`. → Lane A2. |
| R16 | "The rider has a blank white-pink face with atlas bleed at the crown" | composition C1, postfx major, artifacts C3 | **RE-ATTRIBUTE** | From the chase camera you are looking *through* the open eye port at the interior backface of the helmet (`side: THREE.DoubleSide`, rider.js:1465). It is a hole through the head, not a bad face texture. Padding the UV island fixes nothing. → Lane C1. |

---

# PART 4 — AUTHORED CONTENT vs CODE

**Fixable in code now** (everything in Lanes A–K above except where noted). Notably, all four of the "unfinished build" artifacts, the entire perf lane, the shadow rig, the camera rig, the tonemap, and every material and placement fix.

**Genuinely authored-content scope — do not put these on an engineer's sprint:**
- Photographic needle-sprig plates and bark plates per species (D1, D7). The procedural pipeline can stay for mid/far; the near tier needs real plates.
- Two broadleaf species + two mid-storey shrubs with structural mass (D-library work — this is *library* scope, not level authoring, and does not touch the deferred species-progression item).
- The rider garment albedo/AO atlas **art** (C2). The bake harness is code; the painted hems, panels, logos and sponsor marks are art.
- Hand and shoe **geometry** (C3), and the re-authored attack pose (C4) — a rigger/animator job.
- Bike decal atlas, head badge, weld/gusset geometry (F2).
- Course furniture art: sponsor banners, arrow boards, truss (H5).
- Terrain sculpt for the named sections — actual berms, takeoffs, landings, rock-garden features. Eight named sections are currently visually indistinguishable and r3_03 "berm flow" shows an un-banked corner. **This is the deferred geometric-consequence item and it stays deferred**; H2's wear signal is the code-side down payment on it.

---

# PART 5 — HONEST VERDICT

**NOT_AAA, and not close.** But the distance is very unevenly distributed, and the reviews collectively obscure that by ranking cosmetics alongside a placeholder protagonist.

The world is genuinely good. The terrain shader is sophisticated — triplanar, tile-break, coverage-preserving alpha mips, Toksvig roughness, texel-snapped shadow fitting, a Thomas cluster process for trees, an honest LOD derivation with exported stats. The forest floor material in r3_07 and the r3_02 alpine descent are at or near the bar. The r2 fixes all landed and I verified them independently: 0.00% crushed pixels in all 16 shots, controlled highlight end, working treeline progression.

**The three things standing between this build and the bar:**

**1. The rider and the bike.** On screen in 100% of gameplay frames. There is a hole through the protagonist's head in the primary camera. The garments are per-vertex gradients on a swept loft, so no hem, seam, cuff or sole exists at any distance. Gloves, shoes and knee pads are the same knobbly ellipsoid. The hands do not grip the bar — the bar passes through them. The pose is a commuter perch. In a game where the bike is the product, the fork lowers are the brightest object in frame and the frame carries no brand. **No lighting, post or terrain work can rescue this, and it is what a reviewer screenshots first.**

**2. The vegetation is hand-painted, and it is arithmetically incapable of being anything else.** A needle is authored 1.64 px in a 512 px atlas cell that renders at a 0.39× downsample — 0.64 screen pixels at the *closest* distance the game presents. So the canopy can only ever be an aggregate blur, and a 0.34 alpha test chewing on an antialiased 1.64 px stroke produces the 444-speck stipple on every silhouette. 40%+ of the pixels in the signature shots are this. r3_06 and r3_13 read as oil paint.

**3. Nothing is grounded.** The rider, the bike, the tape posts, the fallen logs, the trunks, the grass tufts — no contact shadow, no contact AO, no soil disturbance, no litter mound. The shadow cascade is 0.15 m per texel and physically cannot resolve a 30 mm frame tube; the screen-space AO was correctly weakened to protect the shadow floor and was never replaced with a baked contact term. Contact darkening is the primary cue that an object has weight, and there is none anywhere in 16 frames.

**How far away, concretely.** The cavity fix (Lane A1+B1) is the best day of work available on this project — it is roughly 20 lines, it resolves four critical findings from four lenses, and it removes the loudest "unfinished build" signal in 9 of 16 shots. Lanes A, B, E, G, H, I, K are all engineering against a codebase that is already well-architected: **3–4 weeks of two engineers takes this from "unfinished build" to "competent, coherent indie"**, and the perf lane recovers enough headroom to pay for the extra foliage cards and the tight shadow slice.

Lanes C, D and F are not that. They are the gap, and they are not primarily engineering. A character artist, a foliage artist, and a real plate/photogrammetry pipeline for near-tier bark and needles are prerequisites, not accelerants. With those hires in place, **4–6 months.** Without them, the ceiling is a very well-lit game with placeholder assets in the middle of every frame — which is exactly what round 3 is.

One process note: `qa/shots/r3_14-rider-three-q.png` is destroyed (260×300, fully transparent) and must be re-shot before any Lane C or F sign-off, and `measure()` must be pinned to 1920×1080 DPR 1 before the 63 fps number is quoted again — it may describe an 8.29 MPix frame, not the 2.07 MPix one everybody reviewed.