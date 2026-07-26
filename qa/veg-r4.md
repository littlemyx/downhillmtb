All checks pass. Final state: `node --check` clean, `vite build` clean, headless smoke test of every atlas and geometry builder clean.

## AUDIT — what the dead agent actually left behind

| Item | State I found it in |
|---|---|
| **D1** needle size | **Docstring only.** The block comment above `drawSprig` claimed "the stroke is now ~5.7 px", `needlesAlong` had been rewritten with 3-pass cluster shading — but every call site still passed `S*0.0032 / 0.0030 / 0.0034` and `needleLen 0.0100–0.0135`, `needleCount 3–5`. The actual stroke was still **1.64 px / 0.64 screen px**. The number the whole lane hangs off was untouched. |
| **D2** alpha | `FOLIAGE_ALPHA_TEST` 0.34 → 0.18 **done**; coverage-mip re-solve keys off that constant so it followed automatically. `alphaToCoverage` still gated on `msaaLive`, which resolves to ultra-only. |
| **D3** cards | **Untouched** (`cards:3`, `cardWidth 0.34–0.42`). |
| **D4** hue/sat | **Untouched** (hue 0.318 ≈ 114°, `needleTint [0.72,0.86,0.62]`). |
| **D5** ground | **Untouched** — grass still one kind on `latStep = 1.30/√density`. |
| **D6** AO | Partial by accident (grass/fern/shrub already had a vertical vertex ramp); no trunk contact term, no scatter-pass grounding. |
| **D7** bark/snags | Root flare **already present**; bark still 256-gen upscaled into a 512 cell; snags uniform; snag `barkTint [1.02,1.00,0.95]` over the bleached cell (the pale sausages). |
| **D8** imposter | **Genuinely done** — π-ambient albedo bake, per-cell coverage mips, `medianRGBOver` 12%/8° regression check. Only "more whorl raggedness in the mid tier" outstanding. Per R12 I did not touch fog. |
| **D9** moss/shrub | **Untouched.** |
| **D10** perf | ~40%: `MAX_SHADOW_LENGTH` deleted and `ctx.sky.shadowDistance` read; `RO_SOLID`/`RO_FOLIAGE` and `vegOnBeforeShadow`/`vegOnAfterShadow` were **declared and never referenced anywhere**; `shadowCount` never computed; `repack` never sorted; `grassDepth` still alive; no assertion. |

## WHAT I CHANGED

**D1 — the root cause.** Stroke width `S*0.0032` → `NEEDLE_W = 0.0112` (**1.64 → 5.73 atlas px, 0.64 → 2.24 screen px**), length `needleLen 0.0115 → 0.036`, sub-twig `0.0032 → 0.0060`, branchlet `0.0065 → 0.0090`. Area per stroke ×12, so counts came down to match: branchlets 18→13, `subCount` 7→4, `needleCount` 4→2, pine fascicles 4-6×5-8 → 3-4×3-4. Spruce plate ≈ 3 600 → 820 clusters. Exported as `api.stats.needle` with a PASS/FAIL against the 3-px atlas / 2-4 px screen window, and logged.

**D2.** Kept 0.18. `alphaToCoverage` now tracks `ctx.postfx.composer.multisampling` live (`syncCoverage()`, one-shot + on `quality:changed`) instead of a quality name — `ctx.settings.msaa` never existed. `transparent:false` / `depthWrite:true` pinned explicitly. **CONTRACT-NOTE left for Lane I: `TIERS.high.msaa` is 0; setting it to 2 is the only remaining change.**

**D3.** `cards 3 → 6–7`, `cardWidth 0.34–0.42 → 0.16–0.21` (near tier only; total card area held, widest near card 5.8 m → 2.6 m, i.e. 39% → <10% of frame width). Real per-card AO from **fan position** (middle cards are buried; `|cos roll|` alone would have made them brightest) plus per-whorl AO from crown height. Card stagger rewritten — the old two-value stagger would have put 6 of 7 cards on two start points. Near asset 1 530 → 3 236 tris, paid for by narrowing tier 0 from 40 m to 34 m; `midCardWidth = cardWidth·(cards/2)·1.27` reproduces the old mid asset **exactly** (0.648 vs 0.646), so the 34-250 m band costs what it did.

**D4.** Plate hue 114° → **86-96°**, HSL sat 0.33-0.40 → 0.17-0.20, lum 0.290 → 0.255; `needleTint` de-skewed `[0.72,0.86,0.62]` → `[0.82,0.82,0.72]`. Tip catch-light ceiling 0.86 → 0.62, gain 1.55 → 1.34 (it was putting canopy above sunlit trail luminance). Shadow side now genuinely cooler: hue offset 0.016 → 0.030 (~11°) with sat ×0.74-1.12. Per-instance jitter widened to ±12-15° hue / ±15-18% value — which required re-weighting `tint()` from (-0.85,+0.22,-1.15) to (-0.70,+0.30,-0.55), because the old axis was so chromatic that widening the hue jitter would have raised saturation instead.

**D5.** New `stationCluster()` Thomas process; grass and ferns/moss both moved off the condemned lattice (grass: 4 m parents, 58% rejection, 15-40 children at σ 0.4-1.0 m — calibrated to ~103 k candidates against the lattice's ~91 k, so it is a redistribution not a density change). Four ground kinds: **bunchgrass, sedge** (one builder, genuinely different silhouette/lean/colour), **fern**, **needle-litter mat**. Blades rebuilt with a real **midrib** — 3 verts per row, crest normal up, flanks splayed — which is also what gives the translucency term something to work with.

**D6.** Deepened baked blade AO (0.42→0.30 at the ground); new sharp `contactAO` term in `addTube` (gone by t=0.03, ~0.8 m, vs `baseShade`'s 7.8 m) on trunks and stumps; litter mound instanced at 66% of trunk bases inside 56 m. **I did not touch N8AO** (R10). Honest note in the code: I implemented the "darkening ring" as dark lit geometry, not a multiply-blended decal — a decal needs its own blend mode, pass and MeshBasic-ish material, outside this file's one-shader-patch design and CONTRACT §6. It covers rather than darkens.

**D7.** New `overlayBarkGrain()` adds the missing 256-512 px octave at full cell resolution, evaluated on the same cylinder `drawBark` uses so it is seamless around the trunk (a planar noise would have seamed every trunk). Frequencies solved for wavelength, not guessed. Snags now cluster (1 parent in 8 is a dead patch at ~45%; ~1 in 14 at treeline, 1 in 31 low down). Snag `barkTint` 1.02 → 0.78 — that is the "pale mint sausages" fix.

**D8.** Verified the bake/coverage-mip/median-match machinery is real and left it alone. Added the missing half: mid-tier length jitter ±24% → ±38%, azimuth jitter 0.5 → 0.8, and 1 branch in 12 dropped, to put notches back in the 34-250 m silhouette.

**D9.** `drawMoss` rewritten — six lobed shells whose alphas accumulate to 0.94 in the middle and fall through the test in the outer third, so the cut edge is soft and irregular instead of a lace of holes; albedo two stops down (lum 0.13-0.30 → 0.085-0.19, sat 0.36-0.64 → 0.20-0.36). Fern and shrub leaf saturation cut similarly. `buildMossPatch` given a rim shade (it carried flat `(1,1,1)`).

**D10.** `renderOrder` wired (`RO_SOLID` terrain/rock/wood/grass, `RO_FOLIAGE` trees/shrub/fern/duff). Shadow clamp fully wired: `repack` records `shadowCount` at the partition boundary, `onBeforeShadow`/`onAfterShadow` bound on casting tiers only, single `ctx.sky.shadowDistance` read feeds both the caster sweep and the clamp. Crucially the in-reach test is folded into the **chunk mask as bit 16** — comparing only the split *count* would miss one chunk leaving as another enters, and the clamp would then hide the wrong instances. `grassDepth` deleted; build-time assertion added (gated on shadows-on-at-full-quality so it cannot spam the console at `low`).

Also fixed a **pre-existing UV bug**: `buildMossPatch` mapped its outer ring to uv 1.08, sampling outside its own atlas cell.

## MEASURED / VERIFIED

Needle 5.73 atlas px, 2.24 screen px, alphaTest 0.18 — PASS. Near tris 3 236 / mid 322 (spruce). Every geometry's UVs in range and inside its own cell. Draw calls **25** (≤40). Atlas build 535 ms with a stubbed rasteriser; 13 507 strokes, down from ~4× that.

## COULD NOT DO

- **No browser verification** — per the hard rules the dev server is shared. Every claim above is arithmetic or a headless run, not a screenshot. The canopy value change is the one I would most want eyes on: plate lum ×0.88, tint ×0.95 and the new AO stack land the near canopy at roughly **0.63× its r4 brightness**, which is the right direction against r4_07 but is a judgement call on magnitude.
- **MSAA at `high`** is postfx's file. Coverage is wired and waiting.
- **Authored art, honestly out of reach procedurally.** A resolvable "needle" here is a ~16 cm shoot cluster, not a needle — a real spruce needle is 1 mm, 0.14 screen px at 10 m however it is authored. A photographic sprig plate solves this by being a photograph of a 15 cm sprig, with the needle texture carried by the plate's own micro-contrast. Same for bark: the grain octave removes the smooth-clay read at 2 m, but a ≥2048 plate with a real height map driving parallax is what puts depth in a fissure at 1.5 m. Broadleaf species and mid-storey shrubs with structural mass are library scope and untouched.