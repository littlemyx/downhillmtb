# VERDICT: **FAIL**

Harness: `/private/tmp/dv-carve.mjs` (mine, from scratch, `node --check` clean; thin drivers `/private/tmp/dv-{t2,t2b,t6,t8,t9,geo,lean,stamp,bands,bins,hot,cause,fine,vcap,seeds,adv2,det,bike}.mjs` import it). **I edited no file in the project.** No git, no npm, no browser, no network. Every measurement fingerprints the world before and after with my own FNV-1a hash (193² global lattice + 5-offset tread lattice + every stamp field); **all identical**, so nothing moved the world under itself. A/B always in one process against one cached world.

---

## The fifth thing

**It is the acceptance metric itself.** The ≤6% wheel test is a mean over 2.7 km, and the launching content is concentrated in ~250 m. The whole-course figure passes while the zone the rider actually dies in runs 20–55%.

Round 4 found spurious sub-metre content. Round 5 low-passed it out. The sub-metre excess is genuinely gone — and the aggregate metric now passes — but **the trail is no more rideable than it was**: 0/21 on three seeds, 0/6 on six more, 186 autopilot runs, 0 finishes. The metric was closed; the failure was not.

---

## 1. Full spectrum — ten bands, three seeds. The residual did not move to a new band; it inverted.

r.m.s. metres, committed centreline at 0.05 m plan-arc sampling, linear detrend + Tukey(0.05) + Parseval band power. Two design references: `d_stn` = the trail's **emitted** centre-stamp `targetHeight` per station, Lanczos-8 band-limited (carries exactly nothing below 0.8 m, by construction); `d_terr` = `terrain.sampleDesign().height` (continuous, carries the fine stamp cloud).

| band (m) | seed 20260726 comm / d_stn / d_terr / **comm÷d_stn** | 777 | 12345 |
|---|---|---|---|
| 0.1–0.15 | 0.0000 / 0.0000 / 0.0039 / — | 0.0000 / 0.0000 / 0.0040 | 0.0000 / 0.0000 / 0.0040 |
| 0.15–0.3 | 0.0001 / 0.0000 / 0.0061 / — | 0.0000 / 0.0000 / 0.0061 | 0.0000 / 0.0000 / 0.0061 |
| 0.3–0.5 | 0.0001 / 0.0004 / 0.0071 / **0.28** | 0.0001 / 0.0004 / 0.0072 / **0.25** | 0.0001 / 0.0004 / 0.0071 / **0.27** |
| 0.5–1 | 0.0003 / 0.0053 / 0.0102 / **0.06** | 0.0002 / 0.0052 / 0.0105 / **0.04** | 0.0003 / 0.0054 / 0.0103 / **0.06** |
| 1–2 | 0.0015 / 0.0138 / 0.0150 / **0.11** | 0.0016 / 0.0163 / 0.0171 / **0.10** | 0.0014 / 0.0144 / 0.0162 / **0.10** |
| 2–6 | 0.0238 / 0.0354 / 0.0356 / **0.67** | 0.0262 / 0.0376 / 0.0380 / **0.70** | 0.0262 / 0.0381 / 0.0386 / **0.69** |
| 6–20 | 0.1218 / 0.1260 / 0.1262 / **0.97** | 0.1149 / 0.1185 / 0.1189 / **0.97** | 0.1156 / 0.1195 / 0.1198 / **0.97** |
| 20–60 | 0.4784 / 0.4803 / 0.4801 / **1.00** | 0.3591 / 0.3586 / 0.3600 / **1.00** | 0.4523 / 0.4549 / 0.4541 / **0.99** |
| 60–200 | 1.8795 / 1.8858 / 1.8811 / **1.00** | 1.5316 / 1.5351 / 1.5326 / **1.00** | 1.9529 / 1.9565 / 1.9548 / **1.00** |
| 200–inf | 14.784 / 14.790 / 14.785 / **1.00** | 13.513 / 13.528 / 13.516 / 1.00 | 13.214 / 13.218 / 13.215 / 1.00 |

**No new spurious band exists.** Spurious r.m.s. against `d_terr` is ≤0.0154 m in every band on every seed and falls monotonically with wavelength above 2 m. I also swept **below** the 0.05 m Nyquist at 0.02 m sampling over arc 0–600 m: committed content in 0.04–0.08 m is **5×10⁻⁶ m**, in 0.08–0.15 m **1.2×10⁻⁵ m**, in 0.6–1.2 m **2.6×10⁻⁴ m**. There is nothing hiding down there. Very long wavelengths (60–200, 200–inf) agree to 0.005 m.

What replaced the excess is a **deficit**: the commit builds 4–6% of the design's 0.5–1 m content, 10–11% of its 1–2 m content, and **69±2% of its 2–6 m content**. In the 0.3–2 m bands the "spurious" ratio reads 0.93–1.00 only because the committed tread carries essentially zero there — the residual *is* the design, unbuilt.

## 2. Controller-free wheel test — full stencil matrix, not one cell

0.37 m disc dilation of the profile, `v²κ/g ≥ 1` at design speed, 0.05 m sampling. **The result is a strong function of the differencing stencil, which no prior report stated.**

| stencil (m) | 20260726 comm / d_stn / d_terr | 777 | 12345 |
|---|---|---|---|
| 0.05 | **4.26** / 29.18 / 36.09 | **5.98** / 31.71 / 31.32 | **5.07** / 31.25 / 32.85 |
| 0.10 | **3.77** / 27.41 / 36.93 | **5.73** / 30.22 / 33.98 | **4.86** / 29.68 / 34.78 |
| 0.20 | **3.37** / 17.23 / 33.77 | **5.17** / 21.12 / 32.22 | **4.13** / 20.75 / 31.69 |
| 0.37 (=R) | **3.12** / 10.63 / 24.13 | **4.80** / 14.77 / 22.08 | **3.67** / 12.05 / 21.51 |
| 0.55 | **2.93** / 8.37 / 19.89 | **4.54** / 12.20 / 18.74 | **3.52** / 9.85 / 17.99 |
| 0.74 (=2R) | **2.76** / 5.84 / 13.98 | **4.25** / 9.22 / 14.29 | **3.33** / 7.39 / 12.65 |

Committed @45% of design speed: 0.42–0.57% / 0.38–0.71% / 0.53–0.82%.

**The claim "committed ≤6% against design's ~4%" is half right and misleading.** Committed passes ≤6% at every stencil on every seed (2.76–5.98%). Design does **not** score 4% — on my references it scores **5.8–36.9%**, i.e. the committed surface is now *smoother than the design it is built from*, which is the same over-smoothing the band table shows. Four harnesses have now reported this metric as 3.87%, 26.87%, 36.19% and 2.76–5.98%. It is stencil-, reference- and sampling-dependent and was never specified. It should not be an acceptance gate in its present form.

**And the whole-course mean hides the defect.** 25 m bins, 0.37 m stencil, committed / design_stn:

- **20260726** (course 3.12%): 275–300 m **36/47** · 225–250 **25/33** · 175–200 **22/50** · 2550–2575 17/14 · 1000–1025 16/10. 3 bins over 20%; 83/106 under 5%.
- **777** (course 4.80%): 75–100 **50/67** · 225–250 **47/67** · 100–125 **39/62** · 50–75 **36/47** · 125–150 **26/54** · 250–275 **26/45** · 200–225 **22/58**. 8 bins over 20%.
- **12345** (course 3.67%): 100–125 **55/62** · 75–100 **40/54** · 125–150 **23/33** · 2100–2125 **22/35** · 1000–1025 **22/19** · 200–225 **20/23**.

Restricted to arc 0–600 m the figures are 4.9–5.1% (20260726) and **12.0–12.3%** (777). Nine seeds tested: **every one has a 25 m bin at 20–24%.**

## 3. The 21-cell autopilot sweep — 0/21 on three seeds

My own pure pursuit (`δ = atan(2·L_wb·sin α / L_d)`, target on the centreline at `L_d`, sign taken by projecting onto the bike's own right vector), design-speed governor, no per-cell tuning, driving the real `src/physics/bike.js`. Stuck = 8 s without progress.

**seed 20260726** (2697.7 m) — distance · cause (`/air` = airborne at first crash)

| look \ Hz | 30 | 60 | 144 |
|---|---|---|---|
| 4 | 179.7 offaxis/air | 169.0 lowside | 224.0 lowside |
| 5 | 154.3 offaxis/air | 146.5 lowside | 146.5 lowside |
| 6 | 203.2 impact/air | 195.2 offaxis/air | 365.5 impact/air |
| 7 | 223.7 impact/air | 286.3 lowside | 263.0 lowside |
| 8 | 391.0 impact/air | 256.3 offaxis/air | 230.5 impact/air |
| 9 | 305.9 offaxis/air | 205.9 lowside | 398.2 impact/air |
| 10 | 410.4 lowside | 192.3 offaxis/air | 195.6 lowside |

**0/21 · median 223.7 m (8.3% of course) · min 146.5 · max 410.4 · spread 2.8× ·** offaxis 6, lowside 9, impact 6 · airborne at crash 12/21 · mean airborne fraction 0.130 · 0 NaN, 0 tunnelling.

**seed 777** (2683.8 m): 93.1 slam/air · 70.7 case/air · 99.1 impact/air · 70.7 offaxis/air · 95.0 impact/air · 95.0 impact/air · 146.6 washout · 99.1 offaxis/air · 79.7 offaxis/air · 76.0 offaxis/air · 104.8 impact/air · 76.2 offaxis/air · 110.3 impact/air · 144.5 impact/air · 99.2 slam · 108.5 impact/air · 100.9 impact/air · 76.5 offaxis/air · 76.3 offaxis/air · 78.6 offaxis/air · 76.4 offaxis/air.
**0/21 · median 95.0 m (3.5%) · min 70.7 · max 146.6 · spread 2.1× ·** impact 8, offaxis 9, slam 2, case 1, washout 1 · airborne at crash **19/21** · mean airborne fraction **0.351**.

**seed 12345** (2677.0 m): **0/21 · median 95.2 m (3.6%) · min 76.2 · max 136.9 · spread 1.8× ·** offaxis 9, impact 6, washout 4, case 1, lowside 1 · airborne at crash 16/21 · mean airborne fraction 0.272.

**The lookahead spread is now 1.8–2.8×, not 12–25×.** That is a genuine improvement and it cuts the other way: the result is now a stable property of the trail rather than a controller fluke, and the stable property is failure.

Six further seeds (1, 2, 42, 99999, 20260727, 31337), 6 cells each: **0/36**, medians 40–211 m. Speed ablation (24 runs at ×0.7/×0.45/×0.30): **0/24** — at 30% of design speed seed 20260726 still crashes at 313–323 m and 777 at 107–336 m. Launch-speed-capped ablation (63 runs, governor clamped to the geometric `sqrt(g/κ)` limit): **0/63** — it raises 20260726's median 224→304 m and cuts airborne 0.130→0.067, but drops 777 to 34 m (17/21 lowside: below the launch limit the bike can no longer hold its own berms). **186 runs, 0 finishes.**

*Control on my own controller*: it tracks the centreline within ±0.5 m at 13–17 m/s for the first 115 m of seed 20260726, and at 0.3× speed it reached **1358 m** (ended stuck, not crashed) and 931 m on 12345. It is not a controller that cannot go far.

## 4. Where it dies, and why — hot zones the mean hides

Every seed dies in the `start` phase's `rollers`/`berm` run. Band decomposition **inside the hot zone**:

| seed / zone | 0.3–1 | 1–2 | 2–6 | 6–20 | 20–60 |
|---|---|---|---|---|---|
| 20260726, 175–300 m — committed | 0.0008 | 0.0038 | **0.0752** | **0.3208** | 0.4364 |
| …design_stn | 0.0024 | 0.0078 | **0.0744** | **0.3286** | 0.4213 |
| 777, 50–275 m — committed | 0.0008 | 0.0050 | **0.0702** | **0.4486** | 1.0005 |
| …design_stn | 0.0061 | 0.0359 | **0.0822** | **0.4441** | 1.0026 |
| 12345, 75–150 m — committed | 0.0024 | 0.0118 | **0.1098** | **0.2050** | 0.7756 |
| …design_stn | 0.0054 | 0.0280 | **0.1229** | **0.2112** | 0.7708 |

Committed ≈ design in the 2–6 and 6–20 bands there (ratio 0.86–1.02). **The carve is faithful in the hot zone; the roughness is authored.** 0.070–0.110 m r.m.s. at 2–6 m (3–5× the course mean) and 0.20–0.45 m at 6–20 m — and the design speed there is 13.3–16.8 m/s, the highest on the course. At λ = 10 m the launch amplitude at 13 m/s is 0.15 m; the rollers are 2–3× that.

Per-phase, fraction of stations where **design speed exceeds the geometric launch speed of the committed surface** (and of the design surface), with p95 of `v_design / v_launch`:

| phase | 20260726 comm / design / p95 | 777 | 12345 |
|---|---|---|---|
| **start** | **8.3% / 18.4% / 1.28** | **21.8% / 42.0% / 1.85** | **12.5% / 23.4% / 1.84** |
| roots | 1.4 / 21.3 / 0.90 | 1.3 / 18.9 / 0.88 | 1.0 / 19.6 / 0.83 |
| flow | 0.4 / 4.7 / 0.86 | 0.8 / 8.0 / 0.80 | 0.8 / 3.7 / 0.86 |
| jumps | 6.9 / 15.4 / 2.40 | 7.9 / 16.6 / 2.14 | 8.6 / 16.6 / 2.54 |
| slab | 1.9 / 2.1 | 2.2 / 2.4 | 1.9 / 2.3 |
| creek | 0.0 / 0.9 | 0.0 / 5.6 | 0.0 / 1.4 |
| loam | 0.6 / 2.3 | 0.9 / 5.2 | 0.2 / 1.6 |
| rocks | 2.4 / 20.9 | 3.0 / 25.1 | 2.6 / 20.4 |
| sprint | 3.2 / 8.3 | 1.8 / 7.6 | 2.1 / 14.5 |

The mechanism is upstream of the carve. `techSpeedCap()` (`trail.js:2747`) caps design speed from `amp = S.bumps[i] + S.rough[i]*1.5` at `ROUGH_LAMBDA = 1.6 m`. In the hot zones `S.bumps` = 0.0015–0.0028 m and `S.rough` = 0.0145 m, so the cap returns ~16.8 m/s. **The 6–20 m roller amplitude that dominates the profile there never reaches the speed solver at all.** The trail authors 0.2–0.45 m undulations and then asks the rider to take them at 1.3–1.9× the speed at which a wheel goes ballistic.

Reducing speed does not close it either: to get the 777 hot zone under 5% wheel-launch needs **×0.5** design speed, and at that speed the bike lowsides in the berms (17/21). The geometry and the pace are mutually incompatible on that seed, not merely mistuned.

## 5. Stamp self-agreement and two-leg conflicts — trail engineer confirmed, with one exception

`terrain.sampleDesign().spread`, 5 lateral offsets per station:

| seed | ≤0.05 | 0.05–0.15 | 0.15–0.50 | **>0.50** | p50 | p90 | p99 | max |
|---|---|---|---|---|---|---|---|---|
| 20260726 | 28.36% | 53.81% | 16.20% | **1.64%** | 0.080 | 0.197 | 0.623 | 2.008 |
| 777 | 15.31% | 55.24% | 26.81% | **2.63%** | 0.105 | 0.282 | 0.825 | 3.120 |
| 12345 | 21.86% | 53.31% | 22.33% | **2.51%** | 0.094 | 0.285 | 0.741 | 3.303 |

Reproduces the trail engineer's after-column (1.59/2.52/2.55%, p50 0.074/0.101/0.087) to within sampling noise. The 9.8% figure in my brief does not reproduce; theirs does.

**Two-leg conflicts** — my own detector (arc separation ≥12 m, plan gap < Σ half-widths, |Δh| > 0.50 m), independent of theirs: **0 / 0 / 0** on the three seeds, matching `trail.safety.legAfter`. Genuinely fixed. **But on seed 2 the trail's own audit reports `legAfter = {count: 6, metres: 60, unresolvedStations: 129, worstHeightGap: 7.88 m, worstPlanGap: 4.75 m, worstAtArc: 789.2}`.** The clearance gate does not resolve conflicts on a seed nobody tested; a 7.88 m height disagreement at 4.75 m of plan gap is exactly the switchback stack the fix was for.

**Carve fidelity against the contract metric fails and has regressed.** `|sampleHeight − targetHeight|` over every emitted stamp:

| seed | n | p50 | p90 | **p99** | **max** | >0.20 m |
|---|---|---|---|---|---|---|
| 20260726 | 47,753 | 0.045 | 0.096 | **0.261** | **0.840** | 2.14% |
| 777 | 62,878 | 0.042 | 0.131 | **0.339** | **1.380** | 4.72% |
| 12345 | 53,930 | 0.043 | 0.115 | **0.293** | **1.157** | 3.44% |

Against p99 < 0.10 and max < 0.20 that is a fail, and worse than round 4's 0.287/1.879→0.261/0.840 only on the max. The trail engineer disclosed this (p99 0.090→0.267); I measure 0.261 on the same seed. It is the direct price of the low-pass.

## 6. Lean identity against the COMMITTED surface — holds, with much less margin than reported

Committed cross-slope from `terrain.sampleHeight` secants across the centre 72% of the tread; bank-into-turn sign taken as `−sign(S.curv)` (verified: `trail.js:2954` sets `S.bank[i] = -Math.sign(S.curv[i]) * mag`; 1186/1186 tight banked stations agree, and the committed cross-slope sign agrees with the declared bank at 1186/1186).

| seed | corners (r<200, v>3) | viol >45° committed | declared | p50 | p99 | **worst committed** |
|---|---|---|---|---|---|---|
| 20260726 | 3762 | **0** | 0 | 4.2° | 28.9° | 32.8° @arc 305 (r 7.9, v 11.1, declared 29.7°, **committed 24.9°**) |
| 777 | 4583 | **0** | 0 | 4.7° | 30.7° | **41.5°** @arc 1920 (r 8.9, v 14.4, declared −38.8°, **committed −25.7°**) |
| 12345 | 4296 | **0** | 0 | 4.2° | 28.3° | **42.6°** @arc 1888 (r 9.1, v 14.3, declared 37.0°, **committed 23.8°**) |

Identity holds. **But the worst committed contact lean is 41.5–42.6° against a 45° ceiling — a 2.4–3.5° margin — where the declared-bank figure is 29.1–29.4°. The low-pass costs 13° of lean budget at the tightest corners.**

Committed ÷ declared cross-slope, p10/p50/p90, by |tan bank| band (centre 72% | full tread), seed 20260726:

| band | centre 72% | full tread |
|---|---|---|
| 0–0.05 | 0.376 / 0.911 / 1.663 | 0.452 / 1.048 / 2.025 |
| 0.05–0.15 | 0.566 / **0.878** / 1.065 | 0.435 / 0.839 / 1.100 |
| 0.15–0.35 | 0.782 / **0.868** / 0.926 | 0.721 / 0.798 / 0.889 |
| 0.35–0.7 | 0.840 / **0.869** / 0.885 | 0.757 / 0.793 / 0.808 |
| 0.7+ | 0.869 / **0.875** / 0.879 | 0.765 / 0.782 / 0.796 |

777 and 12345 give 0.844–0.983 across the same bands. **The trail engineer's claim of 0.941/0.966/0.967/0.973 by band does not reproduce.** I get a flat ~0.87 on the centre 72% and ~0.79 across the full tread. The **terrain** engineer's 0.896 is the number that reproduces. On seed 777 the p10 in the 0.7+ band is 0.590 — one in ten of the steepest berms arrives at 59% of its declared cross-slope.

## 7. Is it still a downhill course? Yes — but the contract band is missed and 1–6 m texture is gone

| | 20260726 | 777 | 12345 |
|---|---|---|---|
| reported / measured 3-D arc | 2697.7 / 2703.2 | 2683.8 / 2691.5 | 2677.0 / 2683.6 |
| drop | 423.0 | 466.1 | 454.8 |
| mean grade | **16.00%** | **17.78%** | **17.37%** |
| grade p95 / p99 / max | 34.2 / 52.7 / 89.8% | 37.5 / 50.8 / 87.0% | 35.2 / 45.4 / 89.3% |
| stations >35% grade | 4.5% | 7.2% | 5.1% |
| max climb / longest non-descending | 2.16 m / 32.3 m | **5.48 m / 46.4 m** | 2.57 m / 24.0 m |
| min tread | 1.26 | 1.29 | 1.22 |
| corner radius min / p1 / p50 | 6.9 / 10.1 / 153.4 | 6.6 / 10.2 / 104.9 | 7.1 / 10.3 / 110.6 |
| features / splits / checkpoints | 31 / 6 / 8 | 40 / 6 / 8 | 39 / 6 / 8 |

Per-phase grades (20260726): start 21.5, roots 26.7, flow 16.6, jumps 11.5, slab 13.6, creek 10.9, loam 15.9, rocks 9.8, sprint 13.3%. Contract §4 asks for 8–14% average with chutes to 35%: **mean is 16.0–17.8%, roots runs 26.7% for 243 m, and 1.1–2.1% of stations exceed 45%.** Not a fire road — the opposite.

**Over-smoothing check (§6 asks specifically for 6–60 m):** 6–20 m ratio **0.97** and 20–60 m **0.99–1.00** on all three seeds. The macro shape survives. Shaped-feature detrended peak-to-peak, design vs committed: mean **0.970 / 0.957 / 0.970**, min 0.931 / 0.884 / 0.955 (n = 11/11/12). Jump ballistics intact (e.g. 20260726 road-gap A: v 11.72, lip 1.06 m/30°, gap 12.2, landing 15.6, air 1.52 s, window 11.0–15.2 m/s). **The flattening is confined to 1–6 m**: 90% of the authored 1–2 m relief and 31% of the 2–6 m relief are not built. The rock gardens, roots, brake bumps and ruts are painted, not built — terrain's own CONTRACT-NOTE says so in terms.

## 8. Regression — clean

**Bike, on synthetic ground, isolated from the world:** static sag fork/shock **29.4% / 26.7%** (exact); rear lockup **min spin/rolling 0.000, 946 skid frames** (locks); step symmetry L/R **0.000% at 30/60/144/240 Hz**, lean ∓28.08° exactly; frame-rate divergence vs 240 Hz **≤0.02%** over 220 m; 300 s randomised soak: **0 NaN, 0.000 bar sign-reversals/s**; flat braking to a full stop. Terminal velocity on a 15% grade **16.885 m/s** against a 17.18 baseline — 1.7% off, which I attribute to my synthetic plane rather than the bike, but I did not reproduce it exactly and will not claim I did. Across all 186 autopilot runs: **0 NaN, 0 tunnelling.**

**Determinism:** byte-identical in **14 cross-process boots** — 6 at load average 6.55, then 8 at load average **17.9** with 14 spinners running. `20260726: grid=4714bfda tread=a8ada154 stamps=47753/eb83ce0b len=2697.7456 drop=422.9944` · `777: grid=37868a9f tread=a7d29a69 stamps=62878/cc1da8b9 len=2683.7912 drop=466.0579` · `12345: grid=66e35946 tread=d90731d4 stamps=53930/9df11146 len=2676.9669 drop=454.8274`. Fingerprints identical before and after every measurement in this report.

**Residual:** the world is still a function of seed **and quality tier**. `high` vs `medium` on seed 20260726: `grid=4714bfda` vs `11289f88`, 47,753 vs 51,989 stamps, len 2697.75 vs 2702.63, drop 423.0 vs 432.7. A player who changes quality mid-session gets a different mountain and a different trail.

## 9. Adversarial

- **A seed where the route finds no acceptable line:** **seed 2.** `legAfter.count = 6` (60 m, 129 unresolved stations, 7.88 m of height across a 4.75 m plan gap at arc 789). Longest non-descending stretch **103 m**, max climb 10.6 m, min tread at the 1.20 m floor. Autopilot median 78 m, max 82 m. Also **seed 99999**: `unprotectable = 127`, 1606 m exposed. **Seed 20260727**: max climb 12.75 m against a ~15 m contract limit.
- **Switchback stack launching off the upper leg:** 0 conflicts on the three main seeds by my detector *and* the trail's; live on seed 2 (above).
- **A berm that became a wall:** >0.45 m of rise inside the tread half-width — 544 / 618 / 631 stations on the three main seeds, **all** of them authored berms or wallrides, **0 on an unbanked line**. Clean. On **seed 2**, 10 unbanked stations, worst **0.89 m at arc 737**.
- **Anywhere the bike can get stuck:** bowl/trap stations (walled both sides + rising ahead) = **0** on all seeds. Stalls occur only in the speed ablations (3 of 87 low-speed runs), where my own pedal cap contributes and I cannot cleanly separate it.
- **Falling off the mountain.** This is what the crash traces actually show: at 30% design speed the bike crashes at 15.5–17.4 m/s after **1.4–3.5 s of free fall** and 25–40 m of lateral displacement. Ground fall-away 5 m outside the tread edge: >3 m on **5.9 / 12.5 / 7.3%** of edges, >6 m on 1.7 / 2.4 / 1.0%, max 12.66 / 11.30 / 13.71 m. Seed 777 — the one the trail engineer flagged as their regression — has 1465 m exposed and 20 unprotectable stretches. Once the rider loses the line in the hot zone, the terrain outside the corridor finishes the job.

---

## Bottom line

The carve is no longer the cause. The sub-metre excess is gone and did not reappear anywhere in a ten-band sweep down to 0.04 m. Determinism is solid at load 17.9. The bike is untouched. Two-leg conflicts are resolved on three seeds. Jumps survive the carve at 96–97% of amplitude. The lookahead spread has collapsed from 12–25× to 1.8–2.8×.

**But nothing rides it.** 186 autopilot runs across nine seeds, seven lookaheads, three frame rates and four speed scales: **zero finishes**, median progress 3.5–8.3% of the course. The wheel test passes only as a 2.7 km average over a course whose first 300 m runs 20–55%. The cause is now upstream of terrain.js entirely: `techSpeedCap()` prices roughness at `ROUGH_LAMBDA = 1.6 m` from `S.bumps + S.rough` and is blind to the 0.2–0.45 m, 6–20 m roller relief the same module authors, so the `start` phase is paced at 1.3–1.9× its own launch speed — and no single speed fixes it, because below the launch limit the bike cannot hold the berms.

Three further things are wrong and should not be carried forward as settled: carve fidelity against the emitted stamps has regressed to p99 0.26–0.34 m / max 0.84–1.38 m; the committed bank realises **0.87** of the declared cross-slope, not 0.97, leaving 2.4–3.5° of lean margin at the worst corners on two of three seeds; and the leg-clearance gate fails on seed 2. The mean grade (16.0–17.8%) is above the contract's 8–14%, and 1–6 m texture — the ruts, roots and brake bumps the contract asks for — is 69–90% filtered out of the built surface.

**Weakest part of my own evidence:** I cannot demonstrate that my pure pursuit would finish a course I know to be clean. Its best showing here is 1358 m at 30% speed, ending stuck rather than crashed. The 21-cell matrix is a bound on the trail, not a proof about the controller — but nine seeds, a 1.8–2.8× spread, and failure at walking pace make it a tight one.