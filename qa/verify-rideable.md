## VERDICT: **FAIL**

Harness: `/private/tmp/dv-final.mjs` (mine, written from scratch; `node --check` clean). I edited no other file. Every measurement below prints a world fingerprint before and after; all were identical, so nothing here moved the world under itself.

---

## 1. Determinism — **PASS**. It really is fixed.

My own fingerprint (independent of `terrain.fingerprint()`): FNV-1a over a 257² lattice of `sampleHeight`, a 3000×3 tread lattice, 4001 curve points, and every stamp field.

| seed | boots | conditions | distinct |
|---|---|---|---|
| 20260726 | 3 in-process + 4 separate processes | load avg 7.7 → 12.6 (10–12 spinners) | **1** |
| 777 | 2 in-process + 1 cross-process | load avg 11.1 | **1** |
| 12345 | 2 cross-process | load avg 12.6 | **1** |

`grid=d9617bb2 tread=1dd6dc20 curve=2f9125d6 stamps=49084/a3b35b29 len=2697.7452 drop=422.9944` — byte-identical in all seven boots of seed 20260726 taken over 25 minutes. Source check agrees: `Date.now()` in `terrain.js` now feeds only `timings`, there is no time-budgeted branching anywhere in `terrain.js` or `trail.js`, and the droplet budget is `DROPLETS × DROPLET_TIER[quality]` with `high` = full. I did **not** need to freeze the clock.

One residual: the world is a function of seed **and quality tier** (`medium` = 65% of droplets). That is settings-derived and documented, but it means a player who changes quality gets a different mountain and a different trail mid-session.

---

## 2. Carve fidelity — **FAIL**, and this is the fourth thing.

**The stated claim, tested as stated** (49,084 emitted stamps, `|sampleHeight − targetHeight|`, terrain's own `micro` subtracted):

| set | n | p50 | p90 | p95 | **p99** | **max** |
|---|---|---|---|---|---|---|
| all stamps | 49,084 | 0.008 | 0.047 | 0.090 | **0.287** | **1.879** |
| tread only (raw) | 22,358 | 0.082 | 0.121 | 0.136 | 0.293 | 1.831 |
| berm | 21,634 | 0.008 | 0.052 | 0.122 | 0.308 | 1.602 |
| lip / landing | 3,225 | 0.002 | 0.014–0.022 | — | 0.131–0.166 | 0.313–0.418 |

`p99 < 0.10` → **FAIL (0.287)**. `max < 0.20` → **FAIL (1.879)**. 2.25% of stamps exceed 0.20 m. Worst cluster x≈291 z≈943 (1.5–1.9 m). The centreline itself is fine (p50 0.027, p99 0.122, max 0.261).

**Then the thing nobody measured.** The terrain engineer reported one band, 2–6 m, and closed it. I decomposed the committed centreline into five bands:

seed 20260726 (rms, m):

| band | committed | design | spurious | **spur/design** | micro removed → spur/design |
|---|---|---|---|---|---|
| **0.3–1 m** | 0.0154 | 0.0048 | 0.0148 | **3.10** | **2.99** |
| **1–2 m** | 0.0126 | 0.0090 | 0.0088 | **0.98** | **0.93** |
| 2–6 m | 0.0462 | 0.0446 | 0.0107 | 0.24 | 0.21 |
| 6–20 m | 0.1988 | 0.1989 | 0.0130 | 0.07 | 0.05 |
| 20–60 m | 0.5706 | 0.5702 | 0.0130 | 0.02 | 0.01 |

seed 777: 0.3–1 m spur/design = **12.23**, 1–2 m = **4.57**, and 2–6 m = **1.30** (spurious rms 0.0586 — ten times the 0.0058 the terrain engineer reported, on the shipped trail rather than their frozen copy).

The residual did not go away; it moved to a wavelength shorter than the one that was measured, where it is relatively worse and where a 0.37 m wheel cannot filter it. Subtracting terrain's own intended microrelief changes it by 4% (0.0148 → 0.0142), so **it is not the authored ruts**. Natural ground 12 m off the tread carries 0.0398 m in that band: the carve removes 61% of the mountain's sub-metre roughness and leaves 3.2× what the design asks for.

**Controller-free rideability test.** Roll a 0.37 m wheel (morphological dilation of the profile by the wheel disc) along each profile and ask where `v²κ/g ≥ 1` at the design speed:

| profile | wheel leaves the ground |
|---|---|
| **committed** | **9,783 / 27,032 = 36.19%** |
| committed, micro removed | 35.12% |
| **design** | **1,091 / 27,032 = 4.04%** |

A 9× amplification, with no controller involved. That is why 18 of 21 cells are airborne when they crash.

---

## 3. The 21-cell sweep — **0/21. FAIL against ≥80%.**

My own pure pursuit (`δ = atan(2·L_wb·sin α / L_d)`), design-speed governor with a 20 m min-lookahead, no per-cell tuning. Seed 20260726, one process, one cached world, fingerprint identical before and after:

| look \ Hz | 30 | 60 | 144 |
|---|---|---|---|
| **4.0** | 129.4 m · offaxis · air | 263.3 m · offaxis · air | 124.3 m · lowside |
| **5.0** | 130.8 m · impact · air | 92.2 m · offaxis · air | 125.3 m · offaxis · air |
| **6.0** | 210.1 m · impact · air (lat −22.9) | 228.8 m · lowside | 76.1 m · wall · air |
| **7.0** | 311.3 m · impact · air (lat +11.3) | **330.3 m** · impact · air | 84.7 m · offaxis · air |
| **8.0** | 45.2 m · offaxis · air | 138.1 m · lowside | 45.3 m · offaxis · air |
| **9.0** | **27.4 m** · offaxis · air | 48.6 m · offaxis · air | 178.7 m · impact · air |
| **10.0** | 49.3 m · offaxis · air | 172.1 m · offaxis · air | 207.4 m · offaxis · air |

**completion 0/21 · median 129.4 m (4.8% of 2697.7 m) · min 27.4 · max 330.3 · spread 12.04×**
causes: offaxis 12, impact 5, lowside 3, wall 1. Airborne at first crash **18/21**. 0 NaN, 0 tunnelling.

| seed | completion | median | min | max | spread | causes |
|---|---|---|---|---|---|---|
| 20260726 | **0/21** | 129.4 m | 27.4 | 330.3 | 12.0× | offaxis 12, impact 5, lowside 3, wall 1 |
| 777 | **0/21** | 69.3 m | 58.3 | 172.4 | 3.0× | offaxis 16, washout 3, lowside 1, impact 1 |
| 12345 | **0/21** | 47.4 m | 22.9 | 577.7 | 25.2× | offaxis 9, lowside 3, impact 3, wall 2, case 1 (+3 stuck) |

The 12–25× spread on a 6 m lookahead range is the same instability the first verifier caught. It has not been fixed; it has been re-measured.

---

## 4. Where do runs die — **many places, not one.**

First-crash locations, seed 20260726, 10 m bins: 20–30:1, 40–50:4, 70–80:1, 80–90:1, 90–100:1, 120–130:3, 130–140:2, 170–180:2, 200–210:1, 210–220:1, 220–230:1, 260–270:1, 310–320:1, 330–340:1. **Fourteen different bins.** I do not reproduce the trail engineer's single "205–238 m wall" as dominant.

The geometry says the same thing. Wheel-lift fraction by 25 m bin: **only 1 of 109 bins is under 5%.** Worst: 525–550 m (53%), 1725–1750 (51%), 75–100 (50%), 2200–2225 (50%), 500–525 (50%), 325–375 (48%), 25–75 (47–48%), 1825–1925 (46–49%).

**45%-of-design-speed ablation — the bike still leaves the trail at walking pace.**
8 cells: crashes at **21.8, 24.2, 38.1, 55.9, 107.3 m**, airborne at 4 of the 5; 3 cells stuck (14.7, 51.9, 229.6 m); airborne fraction up to **26.5% at 4–7 m/s**.
Geometrically, at 45% speed the wheel still leaves at **13.21%** of stations. Named unrideable-at-walking-pace locations (25 m bins, seed 20260726): **75–100 m (42%), 25–50 m (35%), 500–525 m (35%), 1850–1875 m (34%), 1750–1775 m (33%), 525–550 m (33%), 250–275 m (32%)**. Seed 777: **1900–1925 m (38%), 1950–1975 (35%), 325–350 (32%)**.

**Causal experiment.** Low-pass the committed surface (a 17-tap disc average — removes sub-R content, touches nothing else):

| surface | vs design p50 / p99 | completion | median | min | max | mean airborne |
|---|---|---|---|---|---|---|
| committed | 0.027 / 0.121 | 0/6 | 125.3 m | 48.6 | 330.3 | **0.315** |
| low-pass R=0.35 | 0.029 / **0.111** | 0/6 | **192.9 m** | **159.9** | 317.4 | **0.103** |
| low-pass R=0.60 | 0.031 / 0.111 | 0/6 | **229.6 m** | **206.4** | 417.2 | 0.156 |

Deleting the sub-0.35 m content raises the median 54%, raises the worst cell 229%, cuts airborne time 67% — and makes agreement with the design profile *better*, which is the definition of spurious content. **On seed 777 the same experiment does nothing** (92.9 → 86.4 → 93.8; airborne 0.121 → 0.064). So the sub-metre surface content is a major cause on one seed and not the cause on another: there are at least two failure mechanisms and this addresses one.

**Limit on my own evidence:** I also tried running the identical controller over a design-surface proxy built from `terrain.sampleDesign`. It failed too (median 88 m) — but the proxy carried p99 0.119 m / max 0.459 m of its own error plus a discontinuity at the corridor edge, so it is inconclusive and I am not resting anything on it. I cannot demonstrate that my controller would finish a clean course.

---

## 5. Still a downhill course — **yes. It was not flattened.**

Reported length 2697.7 m; **measured 3-D centreline arc 2703.2 m** (stations are spaced 0.4 m *in plan*, not in arc — worth knowing, it invalidates any `i × STATION_DS` arc mapping). Drop 423.0 m against a 430 m design target. Mean grade 15.65%.

| phase | arc (m) | grade % design | grade % **committed** | min/max width |
|---|---|---|---|---|
| start | 0–316 | 21.5 | **21.5** | 2.85 / 3.00 |
| roots | 316–559 | 26.7 | **26.7** | 1.27 / 2.37 |
| flow | 559–986 | 16.6 | **16.6** | 2.17 / 3.00 |
| jumps | 987–1392 | 11.5 | **11.5** | 2.52 / 3.00 |
| slab | 1393–1571 | 13.6 | **13.6** | 1.89 / 2.44 |
| creek | 1572–1703 | 11.0 | **11.0** | 1.96 / 2.90 |
| loam | 1703–2034 | 15.9 | **15.9** | 1.65 / 2.31 |
| rocks | 2035–2257 | 9.8 | **9.8** | 2.04 / 2.70 |
| sprint | 2258–2703 | 13.3 | **13.3** | 2.47 / 3.00 |

Max climb above the running low **2.16 m** (limit ~15 m); longest non-descending stretch 32.0 m. Corner radii min 6.9 m, p1 8.7, p50 90.6, 85 stations under 12 m. 31 features (berm 14, doubles 3, chute 2, drop 2, jump 2, rollers 2, roots 2, rockGarden 1, gap 1, creek 1, stepDown 1). 8 checkpoints, spacing 274–323 m. 6 A/B splits (contract wants ≥4). Min tread 1.26 m (contract floor 1.20).

Jump geometry **survives the carve** — detrended lip height over ±20 m, design vs committed: 0.42/0.55, 0.69/0.70, 0.88/0.91, 1.06/1.04, 1.36/1.36, 0.96/0.91, 1.41/1.47. (My first pass said the jumps were flattened; that was my own arc-mapping bug, found and corrected before it reached this report.)

Against the contract, the honest criticism is the opposite of a fire road: mean 15.65% is above the specified 8–14% band and `roots` runs 26.7% for 243 m.

---

## 6. Bike regression — **PASS. It is still not the bike.**

Measured on synthetic ground, isolated from the world:

| metric | regression | measured |
|---|---|---|
| terminal velocity, 15% grade | 17.18 m/s | **17.18** |
| static sag fork / shock | 29.4% / 26.7% | **29.4% / 26.7%** |
| rear lockup | locks | **locks** (min spin/rolling 0.000, 426 skid frames) |
| braking | 8.37 m/s² | 7.32 (12.71→1.02 m/s in 1.60 s — different window, not a discrepancy I can call) |
| step symmetry L/R | 0.0% | **0.00%** at 30/60/144/240 Hz |
| frame-rate divergence vs 240 Hz | — | ≤0.39% |
| bar sign-reversals | 0.00/s | **0.00/s** (300 s randomised soak) |
| NaN / tunnelling | 0 / 0 | **0 / 0** in the soak and across all 71 autopilot runs |

Not measured: the 3.56 m/s pumping gap.

**Lean identity holds, including on the committed cross-slope** — 3,763 corners, **0 violations on both design bank and committed bank**. Worst committed contact lean 29.8° (arc 296 m, r 8.9 m, v 11.7, need 57.4°, committed bank 27.6°). Committed/design bank ratio p50 **0.960**, mean 0.981. The terrain engineer's warning that the built heights carry only 0.38–0.62× of the declared bank is **not true of this build** — that concern is closed, and I checked it because they raised it.

---

## 7. Adversarial

- **Launching/walling berms:** 695/6613 stations (10.5%) have >0.45 m of rise inside the tread half-width, but 652 of them are berms with 42–43° of *authored* bank at 8–24 m radius — correct construction. Only 43 sit on a straight or unbanked line; the worst genuine wall is 0.62 m at arc 1270 m (the gap-15 landing knuckle, lat −1.31 m, r=4000, bank −0.2°). Not a blocker.
- **Traps:** 0 bowl stations (walled both sides and rising ahead).
- **Grade discipline did not make it climb:** 2.16 m max.
- **Switchbacking to avoid exposure:** one hairpin only, 43 stations at arc 301–305 m, r 7.6–8.9 m at 11 m/s design speed. Not a pattern.
- **Getting stuck:** 3 of 8 cells at 45% speed and 3 of 21 at seed 12345 stalled for 8 s. My pedal cap (0.30) contributes; I cannot cleanly separate it from the terrain.
- **Seed robustness:** the route scorer finds a line on all three seeds, but the quality swings hard — seed 777 lands at 358.8 m of drop against a 430 m target (17% short) and its committed tread carries **12.2×** the design's sub-metre content.
- `trail.safety` reproduces the trail engineer's own claims exactly: `unprotectable=6`, `exposedMetres=477.2`, `stretches=36`, `crestStage.afterSafety=54`.

---

## Bottom line

Determinism is genuinely fixed and I could not break it. The bike is untouched and every regression number I could reproduce is exact. The lean identity now holds on the *built* surface, not just the emitted one. The course is still a real downhill course with real jumps. The trail engineer's exposure fix is real.

But the blocker is not closed. The carve-fidelity claim fails as stated (p99 0.287 vs 0.10, max 1.879 vs 0.20), autopilot completion is **0/21 on three seeds** against a ≥80% target, the median run reaches 4.8% of the course, the cell-to-cell spread is still 12–25×, and the bike still leaves the ground at walking pace in seven named places on the default seed.

The specific thing this round missed: **the accumulator residual was measured in one band (2–6 m) and closed there; the launch-causing content is at 0.3–2 m, where the committed tread carries 3.1× (seed 20260726) to 12.2× (seed 777) the amplitude the design asks for, it is not the authored microrelief, and a 0.37 m wheel at design speed is thrown by it at 36% of stations against 4% for the design.** Removing it by low-pass — which improves agreement with the design — nearly doubles progress and cuts airborne time by two thirds on seed 20260726, and does nothing on seed 777. So it is one real cause, worth fixing, and it is not the only one.