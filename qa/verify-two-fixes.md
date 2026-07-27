Note: claude-sonnet-5[1m] (the safety classifier) was unavailable when reviewing this subagent's work. Please carefully verify the subagent's actions and output before acting on them.

# VERDICT: **FAIL** — but the two named fixes both landed, and the eleventh instrument defect was mine

I edited no file. No git, no npm, no browser, no network. Harness written from scratch in `/private/tmp/claude-501/-Users-stewartwebster-Projects/1bc47d7b-2580-443f-86fb-d67586b192a9/scratchpad/r13v/` — `boot.mjs synth.mjs drive.mjs cal.mjs env.mjs env2.mjs band.mjs dump.mjs rr.mjs spot.mjs jaud.mjs ball.mjs flights.mjs matrix.mjs seg.mjs sp.mjs knoll.mjs refused.mjs riderefused.mjs reg.mjs det.mjs t0.mjs`, all `node --check` clean. I did not reuse the engineer's r13 harness. Every matrix is one process with the world fingerprinted before **and** after: **`fpStable: true` on every one**, 1449 matrix cells + 585 segment cells + ~960 ablation/knoll cells, **0 NaN, 0 tunnelling frames**.

**My control is faithful.** `20260726 len=2685.6904 stamps=56349` · `777 2723.6018/56325` · `99999 2722.4168/50904` — identical to the engineer's published determinism line.

---

## STEP 0 — MY INSTRUMENT, AND THE THREE DEFECTS I FOUND IN IT FIRST

**Two defects in my own harness were caught before any figure**, and a third was caught mid-run and cost me a rewrite:

| # | my defect | how it read | fix |
|---|---|---|---|
| A | **rider's-right vector inverted** — I wrote `(f.z, 0, −f.x)` where `cross(forward, up) = (−f.z, 0, f.x)` | consistent in both the sign probe *and* the pure-pursuit, so they cancelled and produced a "correct" controller with `SIGN = −1`. Certification read **0/18** | fixed the frame; `SIGN` re-measured as **+1** |
| B | synth `startTransform` used `lookAt(0, −tan)` | `forward·tangent = −1` | `Matrix4.lookAt` puts local **+Z** at `eye−target`, bike forward is −Z, so **the target IS the heading** |
| C | **DEFECT 11 — off-course tested in 3-D** | an **airborne** rider over a jump pit reads `lost`. Over `double-1` on 20260726 the tread drops 1.4 m below base grade, so a normal flight sits 2–3 m from the centreline on a 2.9 m tread. **Worth 137 of 483 cells on that seed alone** | off-course is now **plan lateral** > half-width + 1.6 m, or > 6 m below the tread, sustained 0.6 s |

Defects 1–10 audited against my code **by measurement**:

| # | defect | my measurement |
|---|---|---|
| 1 | start orientation | `reset({t:0})` → `forward·tangent` **1.0000**; `reset(startTransform)` → **1.0000**; on the real seeds `startDot` **0.9989 / 0.9998 / 0.9992** |
| 2 | steer sign | `+0.30 → +10.375 m`, `−0.30 → −10.375 m`, symmetric to 0.0001 m. **SIGN = +1** |
| 3 | `nearestT` world-y | **my harness never calls `trail.nearestT`** — a monotone window search over `S.px/py/pz` |
| 4 | `ctx.input.state` | identity asserted, `same: true` |
| 5 | plan-vs-arc indexing | arc integrated in 3-D from `S.px/py/pz`; station index is `i`, never `arc/ds` |
| 6 | steering units | `steer 0.1/0.2/0.4/0.6 → lean 5.60/11.21/22.49/33.69°` vs `steer·LEAN_MAX` 5.61/11.23/22.46/33.69 — **and I compensate `speedFrac = smoothstep(1.0, 7.5, v)`**, bike.js:1950, which rounds ≤12 did not |
| 7 | pitch axis | `riderFore` = `pitch·0.30` exactly at −0.2/−0.4/−0.6/−1.0/+0.5 |
| 8 | pitch saturation | swept, below |
| 9 | air gating | swept, below |
| 10 | `trailTValid` | bike.js:2711–2729 self-clears after `TRAIL_RESYNC`; **recorded, never a stop**. The engineer is right that it is a diagnostic |

### MY WHOLE CONTROLLER (published in full in `drive.mjs`)

- **Steering:** pure pursuit to a station `lookM` of 3-D arc ahead. `κ_pp = 2 sin α / L`; `leanReq = atan(v²κ_pp/g) − bankComp·bank(aim)`; `steer = SIGN·leanReq/(LEAN_MAX·speedFrac)`, clamped ±1, `LEAN_MAX = 0.98`.
- **Lookahead:** `lookM = look0 + lookV·v`, optionally capped at `adapt/|κ_trail|` (κ from the *trail's* tangents over ±3 m), clamped [2.5, 40]. Policies `fixed` / `speed` / `adaptive`; base = adaptive, `look0` swept 4–10, `lookV = 0.5`, `adapt = 0.6`.
- **Governor:** `vT = min_j √((v_des,j·vScale)² + 2·A_FEAS·Δarc)` over 45 m, `A_FEAS = 4.5`.
- **Braking:** `brakeR = clamp((v−vT)/2, 0, 1)`, front at 0.62 of rear, pedal below `vT − 1.5`.
- **Air:** flight = `airborne && airTime > 0.12`; brakes 0, steer ×0.15, pitch ∈ {zero, hold, match}.
- **Pitch:** `−clamp(max(0,tanθ)/g0 + min(8,decel)/a0, 0, pMax)`, `(g0,a0) = (0.55, 12)`.
- **Human:** whole vector delayed 0.30 s through a ring buffer, then rate-limited (bar 4/s, hips 1.5/s, brakes 3/s), aim point quantised to 0.15 m.

### Known-good synthetic course — **18/18 under every lookahead policy**

1200 m analytic 3-D serpentine, 10% grade, `r_min 65.9 m`, 3.0 m tread, walls at 0.15 m/m, berms at 60% of required lean:

| policy | look 4–10 × 30/60/144 Hz | mean lateral | off-tread |
|---|---|---|---|
| fixed | **18/18** | 0.187 m | **0.0%** |
| speed-scaled | **18/18** | 0.283 m | 0.0% |
| adaptive | **18/18** | 0.283 m | 0.0% |

### Margin envelope (9 cells each: look 5/7/10 × 30/60/144 Hz)

| dial | clean | fails |
|---|---|---|
| **radius @ 6.2 m/s** | 30 m, 16 m → 9/9 | 12 m → 6/9, 9 m → 6/9, 7 m → 3/9, **5.5 m → 0/9** |
| **radius @ 10 m/s** | 40/30/20 m → 9/9 | **16 m → 0/9** |
| radius @ 12 m/s | 40 m → 9/9 | — |
| grade, **pitch 0** | to **65% → 9/9** | 75% → 0/9 |
| grade, pMax 0.4 / 0.6 | to **75% → 9/9** | 82% → 0/9 |
| grade, pMax 1.0 | 82% → **8/9** | — |
| roughness λ 8 m | ≤0.15 m 9/9 | 0.20 → 7/9, **0.30 → 0/9** |
| roughness λ 3 m | 0.05 → 5/9 | **0.10 → 0/9** |
| tread width | **1.2 m → 9/9** | never (on smooth ground) |
| exposure | **−2.0 m/m → 9/9** | never |
| bank 0.6 / 0.3 / **0.0** at 40% grade | **9/9 all** | never |

**The radius arm read 0/9 even at r = 40 m on my first attempt.** That was a constant-radius plan circle whose closest-point solve is multi-valued past π radians — an instrument failure, not a rider one. Rebuilt as a serpentine parameterised by its own minimum radius (delivered radius measured and reported per cell), it behaves.

**Two settled claims do not survive this envelope**, and I say so:
- *"pitch 0 fails from 30–36% grade"* (round 8, endorsed by round 12) is **instrument-dependent**. With `speedFrac` compensation and an adaptive lookahead, pitch 0 rides **65%** clean. The 0.75 cliff is real and reproduces a fourth time.
- *"narrow tread / exposure is what kills the rider"* — **not on its own.** 1.2 m of tread and −2.0 m/m of exposure are 9/9 on smooth ground. It is narrowness **× roughness × grade** that kills, and my λ-3 m roughness margin (0.10 m → 0/9) is the tight one.

### Sensitivity of the headline to EACH controller element (seed 20260726, 21 cells per arm)

| element varied | median (m) | max (m) |
|---|---|---|
| **base** (adaptive, pMax 0.6, air zero) | 787.5 | 1255.1 |
| lookahead → fixed | 1054.3 | 1214.7 |
| lookahead → speed-scaled | 1129.0 | 1251.2 |
| `adapt` 0.3 / 1.2 / off | 1045.0 / 1129.8 / 1129.0 | 1213.1 / 1248.5 / 1253.4 |
| **pMax 0** | **92.3** | 385.5 |
| pMax 0.4 / 1.0 | 652.7 / 1130.2 | 1248.5 / 1247.2 |
| g0 0.40 / 0.80 | 1129.8 / 786.7 | 1147.4 / 1216.3 |
| air hold / match / steer×1 | 788.8 / 634.3 / 1130.6 | 1134.6 / 1133.0 / **1264.4** |
| vScale 0.85 / 1.15 | 1130.2 / 635.2 | 1248.1 / 1219.1 |
| governor lookahead off / hard brake | 1063.9 / 1129.4 | 1251.2 / 1251.6 |
| no `speedFrac` / no bank comp | 787.5 / 1129.4 | 1206.3 / 1215.1 |
| **signed pitch schedule** | **92.7** | 387.5 |

**Round 12's finding stands and the engineer's rebuttal does not.** The engineer reports the lookahead clamp is *"bit-identical"* to no clamp on all three seeds and concludes *"THE FREE PARAMETER IS DEAD."* On my controller the clamp binds and moves the reference median **787.5 → 1129.0** (+43%); the pitch saturation moves it **92.3 → 1130.2** (12×); the pitch-schedule *sign* moves it **8.5×**. The free parameter is not dead. **But the thing it moves is the median, not the wall**: every one of 23 arms maxes out at **1133–1264 m** on 20260726. Medians reshuffled entirely when I fixed defect 11 (pMax 1.0 went 635 → 1130 while base went 1042 → 788) — **median completion is noise-dominated by which side of a hard wall each cell lands on. Only the wall positions and the maxima are stable, and those are what I quote.**

---

## 1. FIX A — `jumpWindowBad` is genuinely 0. The metric is the wrong one.

**Audited all 24 air features on the three buildable seeds myself, from the shipped station table (`S.speed[iLip]`), not from `trail.safety`:**

- **24/24 solved** (`landing:"solved"`, every one publishes takeoff/window/airTime/landingAngle/landingDistance).
- **`jumpWindowBad = 0`** — my count agrees with the trail's, on all three seeds.
- **`jumpOverSpeed = 0`** — every `vLip/vTakeoff` ratio is 0.857–1.000.
- Takeoffs are now **7.24–9.95 m/s** (was 11.5–12.0 against a 5.4–6.2 pace). The reference seed's `road gap (A)` reads **vLip 9.03 into window [8.0, 15.9]** — was 5.71 into [11.1, 18.6]. **The named fix is done.**
- Residual, reported not hidden: `jumpFitLost = 1` on **every** seed, always `rock-garden double:tooSlow`, and `jumpFitIters = 4` — the fixed point spends its whole budget and converges by deleting one feature, identically on three independent mountains.

**And the reference seed no longer dies at 1206–1301 m.** It dies at **1124–1147 m** — 142 of 483 cells, and the cause histogram is `impact 77, offaxis 34, lost 15, case 5`. That is a **landing**, not a pacing failure.

### The real defect, measured on the real bike (`flights.mjs`, not modelled)

Flying the shipped profile with the shipped design pace, recording where `state.airborne` goes true and false:

| seed | feature | published landing D | **bike's actual flight** | vN at touchdown | outcome at design pace |
|---|---|---|---|---|---|
| 20260726 | tabletop | 7.5 m | **4.9 m** (2 flights) | 2.26 | rolls |
| 20260726 | **double-1** | 7.6 m | **5.3 m** | 4.97 | **CRASH `impact` @1130** |
| 20260726 | double-2 | 9.2 m | 8.1 m (**6 flights**) | 5.89 | skips |
| 20260726 | **road gap** | 9.7 m | 8.5 m | 6.00 | **CRASH `loopout` @1215** |
| 20260726 | finish booter | 7.9 m | **3.3 m** | 3.56 | **CRASH `offaxis` @2569** |
| 777 | **road gap** | 9.8 m | 7.2 m | 6.73 | **CRASH `impact` @1285** |
| 777 | finish booter | 6.2 m | **12.1 m** | **8.11** | overshoots by 95% |
| 99999 | **road gap** | 9.4 m | 6.2 m | 7.25 | **CRASH `loopout` @1284** |
| 99999 | **step-down** | 9.7 m | 5.7 m | 7.14 | **CRASH `loopout` @1283** |

**20 of 24 features land 25–60% SHORT of their own published landing distance; two overshoot by up to 95%.** Half the features produce 3–8 separate flights, i.e. the bike skips over them rather than making one clean arc. An independent ballistic replica off the shipped profile agrees (`ball.mjs`): flown distance 0.55–1.0× published, and `vNormal > CASE_VN (5.5 m/s)` on 21 of 24.

**`jumpWindowBad` asks whether the design speed is inside the window the design published. It never asks whether the ground the design built catches the flight that speed produces.** That is the same class of error as `DESC_MAX_RUN` counting consecutive stations: **a metric of the design's self-consistency, not of the property.** Fix A closed the pacing half and left the landing half open, and the landing half is now the biggest wall on two of three seeds (1124–1147 on 20260726 ×142; 1139–1169 on 777 ×53; 1143–1173 + 1283–1288 on 99999 ×88).

---

## 2. FIX B — sub-10 m switchbacks are gone. Nothing regressed into a traverse.

Measured with my own ±4 m plan circumradius on the shipped centreline (the same stencil `rr.mjs` uses everywhere):

| | 20260726 | 777 | 99999 |
|---|---|---|---|
| **r min** OFF → ON | 5.12 → **11.53** | 5.01 → **10.64** | 5.49 → **12.54** |
| r p1 | 16.5 → 14.2 | 6.6 → **13.1** | 12.0 → **16.4** |
| **stations r < 10 m** | 33 → **0** | 113 → **0** | 57 → **0** |
| stations r < 9 m | 31 → **0** | 104 → **0** | 51 → **0** |

**Seed 777's arc-732 switchback does not exist on this build.** The route was redrawn, not widened: the phase boundaries all moved, and 777's tightest corner is now **10.64 m at arc 1335**. It stops nothing — arc 1335 does not appear in any wall cluster.

**Did widening turn the route into a traverse? No.** Net descent per 100 m of arc, ON vs the `feasible:false` control:

| | bins with <5% net descent | worst bin |
|---|---|---|
| 20260726 | 1 of 26 (control: 1 of 27) | +0.047 (control 0.024) |
| 777 | 1 of 27 (control: 2 of 26) | +0.039 (control **−0.006**) |
| 99999 | 0 of 27 (control: 0 of 26) | +0.078 (control 0.085) |

**One genuine regression, and the engineer names it honestly.** On 20260726 the redraw put the route across a knoll it did not cross before: worst net rise over any 15 m window **0.933 → 1.479 m**, and the longest run that ends at or above its own start and never drops below it is **44.2 m at arc 2203** (control 80.2 m at 2148), 190 stations pinned at the 7 m excavation ceiling.

---

## 3. FIX C — seed 12345 refuses. The refusal is correct in outcome and **unauditable in method**.

- Seed 12345 **throws**, identically on 3/3 concurrent boots: *"12.2 m of continuous ground the reference rider cannot descend, starting at arc 94 m (peak committed gradient 93%, DIRT)"*. Seed 2 still throws on `climb`, 24 variants, identically 3/3. **Two of five seeds now refuse.**
- **The throw destroys the evidence for its own decision.** `getCarveStamps()` returns **0** after the throw, so the refused world cannot be built, carved, or ridden by anyone. I tried (`riderefused.mjs`) and could not. A gate that refuses on rideability, and makes rideability untestable, cannot be audited.
- `trail.stations` *does* survive, so I read the refused profile directly (`refused.mjs`). Arc 82–120 on seed 12345: gradient **0.55 → 0.71 → 0.81 → 0.86 → 0.92 → 0.84 → 0.87 → 0.62**, ~38 m of it, on **3.00 m of tread**, bank ±5°, radius 30–100 m, DIRT, design speed pinned at 6.20, excavation 4.3–7.0 m — **and κ_v spikes to 0.311 / 0.285 / 0.465 /m** (v²κ/g = **1.82** at the worst).
- **My analytic replica does not reproduce the stated reason.** A 12.2 m band of **0.93** gradient spliced into ordinary ground, straight, unbanked, 3.0 m, at 6.2 m/s, on the real bike: **9/9 at pMax 0.6 and 1.0**, 3/9 at pMax 0.4. A 20 m band at **1.00** gradient is 9/9 at pMax 1.0. The refusal's stated ground — *"the measured lean budget is zero there at any radius, bank or speed"* — is a **gradient-only** argument and it fails on my instrument. The band table is also non-monotone in band length (0.75 @ 8 m reads 0/9 while 0.75 @ 20 m reads 9/9), so **neither my replica nor round 12's bounds this ground.**
- The κ_v spikes are a second, real defect that the refusal message never mentions, and my roughness envelope (λ 3 m, 0.10 m → 0/9) says they plausibly *are* fatal. **So the seed is probably right to refuse, for a reason it does not give, on a threshold calibrated from a weaker controller than either mine or the engineer's.**

---

## 4. EVERY REMAINING WALL, BY ARC — the deliverable

Clustered from 1449 matrix cells (23 controller arms × 21 cells × 3 seeds) plus 585 segment cells. **Trail** = survives the whole controller sweep including both lookahead policies, both pitch signs, all four saturations and all three air policies.

### Seed 20260726 (2686 m) — 0 finishes in 483 cells, best cell 1264 m (47%)

| arc | ×cells | what it is | trail or rider |
|---|---|---|---|
| **1124–1147** | **142** | **`double-1`'s landing.** Bike flies 5.3 m against a published 7.6 m and cases the knuckle at vN 4.97. `impact 77, offaxis 34, case 5` | **TRAIL** — a landing transition 30% too far out |
| **627–652** | **130** | 27 m berm, −21° bank, 30% pitch, design speed ramping 9.5 → 12.5 m/s over 20 m. `loopout 52, offaxis 50` | **TRAIL** — the speed ramp, not the radius |
| 597–626 | 44 | entry to the same berm, `loopout 18, offaxis 10, otb 5` | trail |
| **780–806** | **38** | r 22 m, −28.5° bank, 3.0 m tread, **12.3 m/s**, exposed 0.45 m. `lost 38`, unanimous | **TRAIL** |
| **1203–1264** | **60** | `double-2`'s landing (1205) and the **road gap**'s (1250). `loopout 16, offaxis 16, lost 19, case 2, impact 5` | **TRAIL** — same landing defect |
| 1926–1938 | 20 | `lost 17` | trail |
| **1984–1996** | **14** | **the wallride: −52° of bank at r 14 m on 2.19 m of tread**, 1.44 m exposed, LOAM, design speed 7.5 m/s. Bank exceeds the required lean by 30° — the rider must lean *out* of the turn or slide down the wall | **TRAIL** — over-banked for its own pace |
| 87–99, 175–200, 1039–1064, 1771–1778, 2560–2569 | 20/14/18/9/6 | assorted | mixed |
| **2199–2247** | **0 on my final instrument** | **44.2 m of non-descending ROCK, design speed collapsing to 1.00 m/s, 2.07 m tread, 190 stations at the 7 m excavation ceiling** | **CONTRACT §4 BREACH — but a PACE KILLER, not a wall** |

**The knoll is not a wall, and this cost me a fourth instrument defect.** My segment map read `arc 2200 → best +13 m` in both pitch-sign arms and I nearly reported a wall. Driving it with the brakes off and pedalling (`knoll.mjs`, 90 cells: entry 4–12 m/s × pMax 0/0.3/0.6 × 60/144 Hz × three entry points) gives **90/90 THROUGH, minimum speed 3.93–9.82 m/s.** The stop was **my governor obeying `trail.speedAt` = 1.00 m/s and braking to a standstill.** Round 12's correction of round 10 on this point is right, and round 10's "the bike sticks at 2110" was almost certainly the same artefact. **Any segment figure taken where `speedAt` collapses is a reading of the harness's governor.** 20260726 has 48 m of such ground, 777 has 41 m (min 1.56 m/s), 99999 has none.

### Seed 777 (2724 m) — **3 finishes in 483 cells; the first top-to-bottom completions on this build**

`noBank` 2/21 and `v115` 1/21 both reach 2688 m. Segment map: **27 finishes of 198**, and **one cell restarted at arc 400 rides 2288 m to the finish.**

| arc | ×cells | what it is | trail or rider |
|---|---|---|---|
| **459–480** | **133** | **tread narrows 1.97 → 1.53 m with 2.0–3.5 m of exposure on the outside**, gradient only 0.14, radius 60–220 m (near-straight), bank 3°, ROOT at 466. `lost 124` — unanimous drift off an exposed traverse | **TRAIL** — the engineer's diagnosis, at arc 468 not 992 |
| 376–406 | 75 | steep-tech, `lowside 20, loopout 16, offaxis 18` | trail |
| **638–662** | **73** | r 27–35 m, −22° bank, 3.0 m tread, design speed ramping 9.5 → 12.5 m/s. `lost 73` unanimous | **TRAIL** — the same speed-ramp wall as 20260726's arc 627 |
| **973–1010** | **75** | r 18.4 m, −27° bank, exposed 0.53 m, 10.9 m/s. `lost 68, offaxis 4` | trail |
| **1139–1200** | **67** | `double-1`/`double-2` landings. `loopout 26, lost 38` | **TRAIL** — landing defect |
| 1277–1294 | 17+11 | road gap landing, `case 2, impact 2` | trail |
| 2477–2495 / 2610–2630 | 28 / 43 | late-run, `lost 49, lowside 11` | trail |

### Seed 99999 (2731 m) — 0 finishes in 483 cells, best cell 2028 m (74%)

| arc | ×cells | what it is | trail or rider |
|---|---|---|---|
| **166–227** | **311** | **a near-straight (r 150–1000 m), 2.93 m wide, 40–49% gradient section on DIRT with the design speed at 12.4 m/s**, cut pinned at 7 m. `lowside 275`. Fixed by `pMax 1.0` (median 188 → 1155) | **TRAIL/RIDER BOUNDARY** — the design pace is priced for a rider at full aft shift, which round 12 showed loops out elsewhere. This is the standing tension, unresolved |
| 102–157 | 49 | the same 33%-mean-grade opening (70% of stations over 30%) | trail |
| **1283–1288** | **57** | road gap landing. `loopout 21, offaxis 29` | **TRAIL** — landing defect |
| 1149–1203 | 29 | double-1 landing | trail |
| **1969–2055** | **50** | 51.9° of bank at r 16 m on 2.30 m of tread at 54% grade; **4.3 m of ground the trail's own `deadShipped` calls impassable at 95% gradient, arc 2004** | **TRAIL** |
| 2617–2665 | 19 | late | trail |

**Every remaining wall on all three seeds is one of exactly four kinds:** a landing transition that does not catch the flight its own lip produces; a design-speed ramp into a berm; a narrow exposed traverse; or an over-banked wallride. **None is a radius problem and none is a pacing problem.** That is the same finding three times, and it agrees with the engineer's characterisation.

---

## 5. IS IT STILL A DOWNHILL COURSE? Yes — but two seeds are measurably gentler.

Measured by me from the shipped stations; OFF = the `trailSolver:{feasible:false}` control on an asserted-identical pre-carve mountain.

| metric OFF → ON | 20260726 | 777 | 99999 |
|---|---|---|---|
| 3-D length | 2731 → 2692 | 2693 → 2729 | 2698 → 2731 |
| **drop (m)** | 466 → **438** | 454 → **469** | 480 → **501** |
| mean grade | .171 → .163 | .169 → .172 | .178 → .183 |
| **stations > 30%** | 11.04 → **9.61%** | 14.75 → **10.62%** | 11.38 → **21.09%** |
| stations > 45% | 1.78 → 0.82 | 3.06 → 1.53 | 2.26 → 3.03 |
| grade p99 / max | .530/.771 → .442/.632 | .687/.970 → .471/.728 | .542/1.273 → .522/.953 |
| **r min / p1** | 5.12/16.5 → **11.53**/14.2 | 5.01/6.6 → **10.64**/13.1 | 5.49/12.0 → **12.54**/16.4 |
| **stations r<10 m** | 33 → **0** | 113 → **0** | 57 → **0** |
| dead ground (my criterion) | 1.0 → **0** | 21.5 → **0** | 9.0 → **4.3** @2004 |
| longest non-descending (property) | 80.2 → 44.2 | 189.5 → **14.4** | 14.4 → 14.9 |
| worst 15 m net rise | 0.933 → **1.479** | 1.088 → **0.245** | 0.247 → **0.146** |
| launch violations (v²κ/g > 1) | 46 → **15** | 40 → **4** | 60 → **21** |
| features / air | 28/8 → 30/8 | 36/6 → 28/8 | 41/9 → 34/8 |
| checkpoints / splits | 8/6 | 8/6 | 8/6 |
| tread < 2.0 m | 17.9 → 15.6% | 15.5 → **21.5%** | 18.5 → 18.9% |
| **unprotectable stations** | 11 → **27** | 0 → 0 | 8 → **125** |

**Not a fire road.** All three sit at 16–18% mean grade (above CONTRACT §4's 8–14% band), 9.6–21% of stations over 30%, grade p99 0.44–0.52, and 20–22 stations of over-45° bank on two seeds. **But 20260726 loses 28 m of drop and 13% of its steep ground, and 777 loses 28% of its steep ground** — a 13 m reversal traverses further per metre of descent than a 6.5 m one, exactly as the engineer says. Per-phase grade, all 8 checkpoints and all 6 splits survive on every seed.

**Two exposure regressions the engineer's table does not report:** unprotectable stations **11 → 27** on 20260726 and **8 → 125** on 99999; 777's sub-2 m tread goes **15.5% → 21.5%**. Given that the residual wall class is "narrow exposed traverse", these move in the wrong direction.

### Where the engineer's report does not reproduce

Their AFTER column is honest and reproduces almost everywhere. Three exceptions, all small, one on the seed that matters:

1. **99999 dead ground: reported 3.8 m, actually 4.3 m** — and `safety.deadShipped` in the shipped build itself reads **4.3**. The in-file comment says 3.8 too. Stale, in the optimistic direction, against a hard gate of 8.
2. **The "before" column is round 12's shipped build, not the `feasible:false` control**, which is a legitimate choice but is not what "pre-carve mountain fingerprint asserted identical per pair" implies for the OFF arm.
3. **"THE FREE PARAMETER IS DEAD"** does not reproduce (§ Step 0).

---

## 6. REGRESSION — clean

| check | reference | measured |
|---|---|---|
| static sag fork / shock | 29.4% / 26.7% | **29.4% / 26.7%** |
| terminal v, 15% grade, `anyPressed=false` | 17.181 | **17.180** |
| ... `anyPressed=true` | 16.885 | **16.885** |
| step symmetry L/R @ 30/60/144/240 Hz | 0.000% | **±28.221/28.223°, 0.0000% at all four** |
| rear lockup | locks | **min spin/rolling 0.000, 222 skid frames** |
| 300 s soak, live steering + braking + pedalling | 0 NaN | **0 NaN, 0 crashes** |
| 1449-cell matrix | — | **0 NaN, 0 tunnelling frames** |

**Bike bit-exact — `bike.js` is not the cause, an eleventh time.**

**Determinism byte-identical across 15 concurrent cross-process boots** (3 × 5 seeds), heightfield + materials + carve + stamps + a 2000-point sample probe: `20260726 world=2cc2bb57 samp=e937c34f stamps=56349 len=2685.6904` · `777 91fd70c6/795598dd/56325/2723.6018` · `99999 7943f33b/d3dfbedf/50904/2722.4168`. **Both refusals identical 3/3.**

**Lifecycle NaN-free** on all three: 500 × (`sampleAt`, `nearestT`, `widthAt`, `speedAt`, `sampleNormal`) → **0 non-finite**. `startTransform` heading `forward·tangent` = 0.9989–0.9998.

**Rider/HUD intact.** `rider.js:3427` still carries `smoothstep(0.10, 0.50, gradient)`, the `clamp(−1.35, 0.9)` and `aft·0.22 − fwd·0.17`; `hud.js` still carries `BAL_LO = 0.16`, `BAL_FRONT_LO = 0.18`, `BAL_CUE_GRADE = 0.16`.

**The windowed descent test measures the property, not the counter — confirmed.** My independent scan (longest stretch that ends at or above its own start and never drops below it) reproduces `44.2 m @2203 / 14.4 @1386 / 14.9 @1216` exactly, against the old consecutive-station counter's `20.8 / 12.4 / 11.2`. The counter and the property now differ by 2× on the reference seed and the build publishes both.

---

## 7. ADVERSARIAL

- **A seed where feature iteration fails to converge:** all three. `jumpFitIters = 4` (the full budget) with `jumpFitLost = 1` on every seed, always the same feature and the same reason (`rock-garden double:tooSlow`). It converges by deleting, consistently, on three independent mountains.
- **A hairpin widened into a climb:** yes, on the reference seed. The redraw put the route across a knoll: worst 15 m net rise **0.933 → 1.479 m**, 44.2 m without net descent, 190 stations at the excavation ceiling, design speed **1.00 m/s**. It is a CONTRACT §4 breach and a pace killer; **it is not a wall** (90/90 through at ≥3.93 m/s).
- **A jump rebuilt into an unlandable one:** yes — this is the round's main finding. `double-1` on 20260726 crashes `impact` at the design pace; the road gap loops out or impacts on all three seeds; 20 of 24 features land 25–60% short of their own published landing distance; 777's finish booter overshoots by 95% at vN 8.11 m/s.
- **Where the bike sticks:** arc 2199–2247 on 20260726 (design speed 1.00 m/s over 48 m) and 2191–2232 on 777 (1.56 m/s over 41 m). A rider who obeys the trail's own speed profile stops dead; a rider who ignores it rides through at 4–10 m/s. **The shipped speed profile, not the geometry, is what stops them.**
- **A berm beyond corridor carry:** none found; but 20260726's arc-1984 wallride carries **52° of bank where its own design speed needs 22°** — over-delivered, which is the same defect in the other direction.

---

# THE ANSWER TO THE QUESTION YOU ASKED

## **FAIL.**

### Could a competent human with a gamepad ride this top to bottom?

**Human-plausible arm (0.30 s lag, bar 4/s, hips 1.5/s, brakes 3/s, aim quantised 0.15 m): 0 finishes in 189 cells.** Best: **1133 m of 2686** (20260726), **473 m of 2724** (777), **296 m of 2731** (99999); off-tread 9.4% / 9.6% / 15.2%. At half the reaction lag (0.15 s) the bests rise to 1253 / 1211 / 1293 m — still no finish.

**No, on all three buildable seeds.** Per seed:

- **777 — one wall from being a game, and it is now a *finishable* course.** This build produced **the project's first top-to-bottom completions**: 3 of 483 matrix cells, and **one cell restarted at arc 400 rides 2288 m to the finish**. The segment map finishes from 12 of 22 restart points (27/198). The binding wall is **arc 459–480: 1.53–1.97 m of tread with 2.0–3.5 m of exposure on the outside**, 133 cells, 124 of them unanimous `lost`. Fix that traverse's width and 777 is a hard but real course.
- **20260726 — two walls, both landings.** `double-1` at 1124–1147 (142 cells, `impact` dominant) and the `double-2`/road-gap landings at 1203–1264 (60 cells), plus the berm speed-ramp at 627–652 (130). Its last 300 m ride (8/9 from arc 2380). Its §4 breach at 2203 is a breach and a pace killer, not a wall.
- **99999 — the opening kills it.** 311 of 483 cells die in the first 227 m, `lowside 275`, on a near-straight 45% pitch where the trail publishes **12.4 m/s**. Full aft shift rides it (median 188 → 1155), so this one sits exactly on the trail/rider boundary the project has never resolved. Clean from ~2150 (6/9).
- **12345 and 2 — refuse.** Correctly in outcome. **Two of five seeds now fail to generate**, which is a shipping fact in its own right.

### Is the remaining work bounded and named, or open-ended?

**Bounded and named — genuinely, for the first time on this project.** Ten rounds relocated the cause; this round did not. Both named fixes landed and neither is a wall any more: no seed ships a corner under 10 m, no air feature on any seed is paced outside its own window, the seed with impassable ground refuses. The replacement list is **four defect classes across three seeds**, all of them local geometry with an obvious remedy:

1. **Landing transitions 25–60% short of the flight their own lip produces** (the single biggest wall on 2 of 3 seeds). `solveJump` and the ground disagree; the fix is to solve the landing against the *shipped* profile, and the acceptance test is `flights.mjs`, which already exists.
2. **Design-speed ramps into berms** (9.5 → 12.5 m/s over 20 m, arcs 627 and 638).
3. **Narrow exposed traverses** (1.5–2.0 m of tread with 2–3.5 m of exposure).
4. **Over-banked wallrides** (52° where 22° is needed).

Two things must be fixed *before* the next round, and both are process, not physics:

- **`jumpWindowBad` is not an acceptance test and must stop being quoted as one.** It measures the design against itself. The property is "does the ground catch the flight", and it fails on 20 of 24 features while the metric reads 0 of 24. This is `DESC_MAX_RUN` all over again — **the third time on this project that a metric has been fixed while the property it stands for was not.**
- **`SHIP_HARD_DEAD` must not destroy the evidence for its own refusal.** It throws before `getCarveStamps()`, so no one can ride the course it refuses. Its threshold is calibrated from a controller weaker than the two now available, and my replica of the ground it names rides 9/9. Make it warn-and-tag, or make the refused world reconstructible.

**My completion figures are a lower bound and I will not dress them up.** They are a reading of the controller published above, and I have shown that five separate elements of that controller each move the median by 1.4×–12×. What is *not* controller-dependent, and is what I would hand to the next round, is: **the wall positions, which are stable across 23 controller arms; the 24 measured flights; and 777's three completions.** One more round on those four defect classes should produce a rideable 777 and a rideable 20260726. That is worth doing. Shipping as-is would ship a 2.7 km course on which no measured rider, human-plausible or calibrated, has ever reached the bottom of two of the three seeds that build.