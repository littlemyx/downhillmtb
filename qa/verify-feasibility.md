# VERDICT: **FAIL** — and the seventh thing is that the autopilot has never used `input.pitch`. The cause has relocated again, *upstream* of the route march, into the cost function the march is now scored on.

I edited no file. No git, no npm, no browser, no network. Harness is mine, from scratch, in `/private/tmp/claude-501/-Users-stewartwebster-Projects/1bc47d7b-2580-443f-86fb-d67586b192a9/scratchpad/sk8/` (`boot.mjs`, `synth.mjs`, `ctl.mjs`, `cal.mjs`, `margin.mjs`, `env.mjs`, `auto.mjs`, `feas.mjs`, `feas2.mjs`, `full.mjs`, `leanmodel.mjs`, `modelvsbike.mjs`, `kernel.mjs`, `kernel2.mjs`, `probe.mjs`, `jump.mjs`, `bikereg.mjs`, `bikereg2.mjs`, `det.mjs`) — all `node --check` clean. Every A/B is one process, two arms off **one cached pre-carve mountain** (`fpPre` asserted identical on every pair), with the world fingerprinted before and after every measurement (`fpStable: true` everywhere).

**My control is faithful.** The `feasible: false` arm reproduces the round-7 build to the decimal: `20260726 stamps=54981 len=2728.2105 drop=466.2994`, `777 56973 / 2687.3259 / 454.2319`. Same pre-carve fingerprint `7c471746` in both arms.

---

## STEP 0 — instrument calibration, and its margins

Analytic synthetic course (no grid, no bilinear facets): 1206 m of 3-D arc, constant 10% grade, `θ = 0.35·sin(2πs/140)` → **κ_max 0.01571 /m, r_min 63.7 m**, zero vertical curvature, berms at 60% of required lean, 3.0 m tread, ground rising 0.15 m/m outside both edges. Steering `steer = atan(v²·κ_pp/g)/LEAN_MAX`.

| | finishes | mean lateral | max lateral | **fraction off the tread** |
|---|---|---|---|---|
| 30/60/144 Hz × look 5–10 | **18/18** | **0.09–0.45 m** | 0.22–1.37 m | **0.0%** |

**Margins** (12 cells each, one dial at a time, `pitch = 0`):

| dial | clean | fails |
|---|---|---|
| roughness λ 8 m | ≤0.15 m (12/12) | 0.20 m → 7/12, 0.30 m → 0/12 |
| roughness λ 3 m | 0.02 m (12/12) | **0.05 m → 0/12** |
| min radius | ≥27 m (12/12) | 18 m → 9/12, **11 m → 0/12** |
| speed | 20 m/s → 12/12 | — |
| exposure | −2.0 m/m → **12/12** | never |
| tread width | 1.2 m → **12/12** (3.2% off-tread) | never |
| **grade** | 20% → 12/12 | **30% → 0/12** |

Exposure and narrow tread are not what this instrument sees. Grade is — and that is where the seventh thing is.

---

## THE SEVENTH THING

### (a) One input, never touched, and the round's entire premise evaporates

`bike.js` line 1258–1259:

```js
riderFore = input ? input.pitch * T.RIDER_SHIFT : 0;      // RIDER_SHIFT = 0.30 m
const cgFwd = T.CG_FWD + riderFore * (T.RIDER_MASS / T.MASS);   // 78/95
```

Full pitch input moves the centre of mass **0.246 m** aft. On a 45% grade that takes the rear load fraction from `(0.640 − 0.45·1.05)/1.25 = 0.134` to **0.331** — from below `RIDE_REAR_LO` to well above `RIDE_REAR_HI`. CONTRACT §6 requires the rider to do exactly this ("body English: **weight back on steeps**").

The autopilot has held it at zero for every completion figure ever quoted on this project. Same synthetic courses as the brief's table, 6 cells per grade × speed, only `input.pitch` changed:

| | v6 | v8 | v10 | v12 | v14 |
|---|---|---|---|---|---|
| **r64, 45% grade — `pitch = 0`** | **0/6** | **0/6** | **0/6** | **0/6** | **0/6** |
| **r64, 45% grade — grade-scheduled pitch** | **6/6** | **6/6** | **6/6** | **6/6** | **6/6** |
| straight, 45% — `pitch = 0` | 6/6 | 5/6 | 0/6 | 0/6 | 0/6 |
| straight, 45% — scheduled | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| r64, 36% — `pitch = 0` | 6/6 | 3/6 | 0/6 | 0/6 | 0/6 |
| r64, 36% — scheduled | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |

**The scheduled arm is 30/30 across the whole envelope.** The schedule is a naive ramp, `pitch = −clamp((tanθ − 0.12)/0.28, 0, 1)`, not fitted to anything. It is not "more is better": `pitch = −1` constant is *worse* than zero above 25% grade (0/6, loopouts) — it is a real control input with an optimum, not a cheat.

**On the shipped trail, same world, same fingerprint, same cells, only `input.pitch` changed:**

| seed 20260726, ON arm | finishes | median | causes |
|---|---|---|---|
| `pitch = 0` (rounds 4–8) | **0/21** | 469.5 m | lowside 12, lost 5, offaxis 2, loopout 1, impact 1 |
| grade-scheduled | **12/21** | **2713 m (the whole course)** | finish 12, lost 5, offaxis 3, wall 1 |

Mean lateral 0.32 m, off-tread 5.3% — it is *on the trail* while it does it. **Zero lowsides.**

### (b) `leanBudget()` is not `leanCeiling()`, and the error diverges exactly where the round spent its money

Frame-by-frame on a completed, crash-free 2713 m run on seed 20260726 (18,960 grounded frames), comparing my reimplementation of trail.js's `leanBudget()` — verified to reproduce the engineer's published table to 0.01° (28.000° flat, 16.89/14.44/11.30/2.36 at 25/30/36/45%) — against `bike.js`'s own published `state.leanLimit` at the same instant:

| grade band | **bike `leanLimit` p50** | p10 | `leanBudget()` p50 | ratio |
|---|---|---|---|---|
| 0–10% | 38.0° | 24.0° | 24.3° | 1.61× |
| 10–20% | 38.8° | 31.2° | 20.9° | 1.85× |
| 20–30% | 37.4° | 29.9° | 16.4° | 2.29× |
| 30–40% | 33.0° | 22.2° | 11.7° | 2.72× |
| **40–50%** | **26.4°** | **18.4°** | **2.0°** | **14.3×** |
| **50–70%** | **19.5°** | 12.0° | **0.0°** | **∞** |
| 70%+ | 19.5° | 14.8° | 0.0° | ∞ |

Control with `pitch = 0`: still 1.7×, 1.8×, 1.9×, 2.3×, **4.7×**, ∞. The finding does not depend on my weight shift.

The 19.5° floor is not noise — it is `T.LEAN_LIMIT_PLANTED = 0.34 rad = 19.4805°`, and `bike.js` states its purpose in terms: *"no amount of derate can take away the ability to steer while a tyre is on the ground"*. **The mirror omits it entirely.** It also omits `RIDER_SHIFT`, and uses `RIDE_MU0 = 1.00` where `bike.js`'s `SURF_MU[DIRT]` is **1.08**.

The one fitted parameter — the smoothstep on `min(nR, nF)` between `RIDE_REAR_LO = 0.08` and `RIDE_REAR_HI = 0.22`, which the engineer says "carries the skipping" — was calibrated against round 7's 30 synthetic cells. Those cells were produced by the un-driven autopilot. **The instrument defect was laundered into a route-planning constant**, and `corridorRide()`, `FEAS_W_BUDGET` (300/rad), the switchback gate's `rearLoadFrac ≥ RIDE_REAR_LO` test (which forbids every hairpin above ~51% grade) and `designCap` all consume it.

**Location: upstream of the route march.** It is not the route march's search that is wrong; it is the objective.

---

## 1. Joint feasibility against the committed surface

Stencils: plan radius = circumradius at ±4.0 m plan arc; grade = central difference of committed height at ±2.0 m, sin→tan; cross-slope = secant of `terrain.sampleHeight` at ±0.36·width; κ_v = 0.37 m disc dilation, 0.05 m sampling, second difference at half-stencil 0.37 m, convex only; v swept from `speedAt(t)` down to `V_DESIGN_FLOOR` = 6.2 in 21 steps, decel scaled ∝ v². A station violates if **no** v works.

**Scored with the bike's own measured ceiling** (monotone lower envelope of the p10 table above) — this is the figure I stand behind:

| seed | violation % OFF → ON | worst 25 m bin OFF → ON | corner viols OFF → ON |
|---|---|---|---|
| 20260726 | 3.05 → **1.44** | 93.8 → **38.5** @675 | 160 → 53 |
| 777 | 4.57 → **1.40** | 79.7 → **29.7** @700 | 265 → 54 |
| 12345 | 2.70 → **1.72** | 64.1 → **29.7** @200 | 134 → 16 |
| 2 | 5.03 → **1.78** | 95.3 → **70.3** @325 | 296 → 89 |
| 99999 | 4.50 → **1.34** | 100 → **50.8** @300 | 250 → 57 |

Corner violations fall **68–85% on every seed**; budget-only violations are **zero everywhere** once the budget is the bike's real one. This is a genuine, monotone, five-seed win — *better and more consistent than the engineer's own table shows*. **But the worst 25 m bin is 29.7–70.3% on every seed against a 2% target, and the residual is now dominated by the launch bound** (101 of 117 violations on 12345).

Scored with **trail.js's own model** (for reference only — contaminated by §(b)): ON 4.80/4.62/9.44/4.07/8.05%, worst bins 52/67/100/73/100. I do **not** reproduce the engineer's 0.19/0.28/4.27/0.18/2.40%. Their `safety.feasViol = 79` on 20260726 does reproduce exactly, so their *self-audit* is intact; their *committed-surface* table is not one I can reach.

Worked example of why the model figure is worthless: 20260726's worst neutral-model bin at **675 m** is a braking zone, not a corner — 22% grade, r = 106 m, the profile shedding 12.5 → 7.1 m/s at 2.0–2.8 m/s². The corner needs 2–8°; `leanBudget()` collapses to 2.1–4.1°. The bike rides straight through it: 12/21 cells finish.

---

## 2. Autopilot sweep — full matrices

21 cells (look 4–10 × 30/60/144 Hz) on the three main seeds, 6 on seeds 2 and 99999. Off-course termination `|lateral| > 6 m` for 0.5 s. Governor: design speed, 40 m feasible-braking lookahead, μ 0.45.

| seed | arm | pitch | fin | median | min | max | spread | mean lat | off-tread | airFrac | causes |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 20260726 | ON | none | 0/21 | 469.5 | 127.2 | 1270.5 | 9.99× | 0.29 | 4.4% | 0.017 | lowside 12, lost 5, offaxis 2, loopout 1, impact 1 |
| 20260726 | ON | **grade** | **12/21** | **2713.0** | 450.0 | 2713.3 | 6.03× | 0.32 | 5.3% | 0.026 | **finish 12**, lost 5, offaxis 3, wall 1 |
| 20260726 | ON | load | 0/21 | 1192.7 | 450.2 | 1359.7 | 3.02× | 0.23 | 2.3% | — | loopout 14, offaxis 6, lost 1 |
| 20260726 | OFF | grade | 0/21 | 318.7 | 164.3 | 2005.9 | 12.2× | 0.63 | 12.3% | 0.025 | lost 9, offaxis 7, lowside 4, stuck 1 |
| 777 | ON | none | 0/21 | 416.3 | 102.8 | 1064.0 | 10.4× | 0.40 | 10.7% | 0.012 | lost 6, offaxis 6, lowside 5, loopout 3, washout 1 |
| 777 | ON | grade | 0/21 | 416.7 | 269.2 | 420.4 | 1.56× | 0.37 | 6.9% | 0.013 | offaxis 13, lost 8 |
| 777 | OFF | grade | 0/21 | 254.2 | 55.2 | 675.4 | 12.2× | 0.51 | 12.8% | 0.083 | lowside 7, lost 6, offaxis 6, impact 1, loopout 1 |
| 12345 | ON | none | 0/21 | 98.2 | 85.7 | 196.2 | 2.29× | 0.38 | 9.4% | 0.041 | lowside 8, offaxis 7, lost 3, loopout 1, washout 1, impact 1 |
| 12345 | ON | grade | 0/21 | 354.6 | 226.6 | 1546.5 | 6.82× | 0.24 | 3.2% | 0.113 | offaxis 8, stuck 7, loopout 4, lost 2 |
| 12345 | OFF | grade | 0/21 | 314.9 | 94.7 | 315.7 | 3.33× | 0.72 | 16.9% | 0.052 | lost 16, offaxis 3, lowside 1, stuck 1 |
| 2 | ON | grade | 0/6 | 353.0 | 348.7 | 688.3 | 1.97× | 0.33 | 5.6% | 0.011 | lost 5, offaxis 1 |
| 2 | OFF | grade | 0/6 | 351.6 | 141.0 | 2021.7 | 14.3× | 0.71 | 17.9% | 0.027 | lost 4, offaxis 1, lowside 1 |
| 99999 | ON | grade | 0/6 | 212.6 | 212.0 | 221.0 | 1.04× | 0.25 | 1.4% | 0.048 | washout 5, lowside 1 |
| 99999 | OFF | grade | **1/6** | 323.1 | 158.2 | 2680.5 | 16.9× | 0.57 | 11.8% | 0.061 | finish 1, lost 2, offaxis 1, loopout 1, lowside 1 |

**0 NaN, 0 tunnelling in 246 runs.** Fingerprint identical before and after every matrix.

**Completion ≥80% is not met.** 12/21 (57%) on one seed, 0/21 on four. **And on seed 99999 the round made the autopilot worse: 1/6 → 0/6, median 323 → 213.**

---

## 3. Did the route change for the right reason? **No — it was redrawn, not repaired.**

Nearest-point displacement between the ON and OFF centrelines, sampled every 2 m:

| seed | p50 | p90 | max | % of course >5 m from the OFF route |
|---|---|---|---|---|
| 20260726 | 45.5 m | 113.8 | 149.7 | **88.3%** |
| **777** | **1266.2 m** | 1515.2 | 1528.9 | **100%** |
| 12345 | 178.6 m | 386.2 | 563.0 | **99.5%** |
| 2 | 25.2 m | 193.2 | 232.6 | 72.8% |
| 99999 | 46.4 m | 250.4 | 516.0 | 91.1% |

On seed 777 the two routes are on average **1.27 km apart** — a different side of the mountain. `routeVariants` goes 7 → 16 with `routeReseeded: true` on three of five seeds. You cannot attribute a metric change to a term when the term changed the entire object; and the engineer's own confession (673/617/210 → 315/416/95 from a switchback-apex-speed change alone) says the score landscape between variants is flat. **The brief's question — "a route that changed randomly and happened to score better is not a fix" — answers itself.**

The one place attribution *is* clean is the wall rounds 4–7 died on. Seed 20260726, arc 288–320:

| | round-7 audit (OFF) | this build (ON) |
|---|---|---|
| plan radius | 5.0–6.4 m | 5.4–7.3 m |
| grade | **51–69%** | **20–22%** |
| committed bank | 4–6° | **13–16°** |
| tread width | 1.48–2.0 m | **3.00 m** |
| contact lean needed at `speedAt` | ≫ ceiling | 19.7° vs a ~33° ceiling |

That switchback is genuinely fixed, and it is why 12/21 cells finish. **That is the round's real achievement.**

---

## 4. Was the course flattened? No — on two seeds it got *steeper*, which is the failure `FEAS_W_BUDGET` was written to prevent

| metric (OFF → ON) | 20260726 | 777 | 12345 | 2 | 99999 |
|---|---|---|---|---|---|
| 3-D arc | 2728→2725 | 2687→2739 | 2691→2726 | 2705→2700 | 2692→2749 |
| **drop** | 466.3→**436.7** | 454.2→505.2 | 465.2→514.5 | 458.7→**383.9** | 479.8→502.6 |
| mean grade | .182→.173 | .180→.193 | .182→**.202** | .192→.170 | .186→.192 |
| grade p99 | .562→.444 | .696→.499 | .571→**.713** | .555→.452 | .561→**.743** |
| stations >30% | 12.0→11.5 | 15.4→16.7 | 13.9→**18.8** | 15.8→12.8 | 11.6→**16.8** |
| **stations >45%** | 2.16→0.94 | 2.95→1.49 | **2.01→7.76** | 2.62→1.07 | **2.32→3.93** |
| r min / p1 | 5.1/16.5→5.0/8.5 | 5.0/6.5→5.6/26 | 5.2/6.3→4.8/21 | 5.4/17.1→5.3/7.1 | 5.5/11.7→5.4/15 |
| max non-descending | 16→21.6 | 57.8→23.0 | 40.7→24.1 | 41.3→**101.5** | 12.1→21.0 |
| features | 28→28 | 35→**26** | 33→**25** | 33→**27** | 40→**32** |
| splits / checkpoints | 6/8 unchanged on every seed | | | | |

**Seed 12345 ships 30 m of 48.7 → 60.5 → 76.8 → 91.0 → 80.5 → 65.1% fall line at arc 200–232**, on 3.00 m of unbanked tread, plan radius 100–330 m — where the trail's *own* `leanBudget()` returns **0.0°**. Seed 99999 the same at arc 160–190 (54–73%, budget 0.0). The corner is free on a straight, `FEAS_W_BUDGET` is capped at 110 against grade-holding 220 and `aim` 220, so the "turn where it is flatter" half of the term **cannot outbid anything** and the route runs the fall line. That is precisely the diagnosis the engineer wrote into the file, applied to the fix.

**A cost the report does not mention: the steep berms are gone.** Stations with |tan bank| in 0.35–0.70, OFF → ON: 161→32, 186→76, 140→**0**, 342→20, 268→40. The >0.70 band: 32→0, 29→0, 59→0 — **empty on all five seeds**.

Jump ballistics are preserved on 20260726 (8 air features both arms, gaps within 0.44 m) — but **not on 12345**, where the doubles/gap/stepDown gaps go 11.27→6.80, 11.03→8.07, 13.90→10.73 m (−27 to −40%). Seed 777 *gained* three air features (5→8), which is another symptom of a redraw rather than a repair.

---

## 5. Seeds 2 and 99999 — still shipping broken, and honestly labelled

Seed 2: `routeAdmissible: false, routeFail: 'climb'`, 16 variants, reseeded. Max climb 9.85 m, **longest non-descending run 41.3 → 101.5 m (worse)**, drop **383.9 m** against the ~430 m design, unprotectable 87→12, exposed 1172.4→**1052.8 m**. The engineer's claimed "exposed 1172→653" does **not** reproduce; their 1172 OFF figure reproduces exactly, so their ON build is a different draw. `legAfter` is 0 on every seed in both arms and `rMin` is 5.3–5.4, not 2.0 — I confirm the engineer's disclosure that terrain.js has moved under seed 2 since the round-7 audit.

Seed 99999: `routeFail: 'rideability'`, unprotectable 6→0, but **exposure 1132.8 → 1376.4 m (+21%)**, stations >45% grade 2.32→3.93%, autopilot 1/6 → 0/6. Exposure also worsened on 777 (1290.4 → **1610.8 m**, +25%).

`gripFloorBound` 879–1277 and `cornerFloorBound` 68–151 on every seed: the trail measures its own geometry failures and ships them.

---

## 6. Committed-curvature residual — measured on terrain's own published spec

0.37 m disc dilation, 0.05 m sampling, half-stencil 0.37 m, flat = `|κ_design| < 0.012 /m` with no feature within 3 m. Four kernels, ONE cached mountain, same route (`nFlat` identical across variants), `sameMountain: true` throughout. (My plan path is the linear station polyline, theirs centripetal Catmull-Rom — stated, and it is why my *max* is contaminated.)

| seed 20260726 | p90 | p95 | p99 | wheel test | worst 25 m bin |
|---|---|---|---|---|---|
| r7-equivalent (iso 0.555, no ladder) | 0.0103 | 0.0151 | 0.0432 | 0.51% | 7.4% @1100 |
| **SHIPPED (adaptive + aniso)** | **0.0094** | **0.0128** | **0.0262** | 0.51% | **7.4% @1100** |
| iso 0.555 + adaptive | 0.0080 | 0.0109 | 0.0220 | 0.51% | 7.4% @1100 |
| seed 777: r7-equiv → shipped | 0.0108→**0.0094** | 0.0167→**0.0132** | 0.0443→**0.0258** | 0.62→0.60% | 9.2→**9.4%** @1300 |

**−9 to −13% at p90 and −15 to −21% at p95, against a claim of −37 to −51%.** The p99 claim (−43 to −52%) does reproduce (−39 to −42%). The anisotropic kernel costs 0.0006–0.0014 /m of p90 residual (they said 0.0022–0.0029) and buys cross-slope retention: p50 by |tan bank| band, iso → aniso, 0.831→0.839 / 0.819→0.852 / **0.755→0.802**. The direction and the rising-with-bank shape reproduce; the absolute level does not — I measure **0.84–0.87**, not 0.92, on the ±0.36·width secant, which is the carve verifier's figure, not the terrain engineer's.

**Terrain's own headline claim reproduces exactly and stands: the wheel test's worst 25 m bin is unmoved by every kernel variant** (7.4% @1100, 9.2–9.4% @1300). Terrain's residual is not what stops the rider.

---

## 7. Regression and determinism — clean

| metric | reference | measured |
|---|---|---|
| static sag fork / shock | 29.4% / 26.7% | **29.4% / 26.7%** |
| terminal velocity, 15% grade | 17.18 m/s | **17.181** (`anyPressed = false`) / 16.885 (`= true`) |
| rear lockup | locks | **locks** (min spin/rolling 0.000, 75 skid frames) |
| step symmetry L/R @30/60/144/240 Hz | 0.0% | **0.000%** at all four (∓30.882°) |
| 300 s smooth soak | 0 NaN, 0.00 reversals/s | **0 NaN, 0.000/s** |
| 246 autopilot runs | — | **0 NaN, 0 tunnelling** |

**I can close the round-6/round-7 terminal-velocity disagreement with a mechanism rather than an attribution.** It is `T.AERO_TUCK`: with `input.anyPressed = true` the rider is only 55%-tucked and terminal is 16.885 m/s; with it false, **17.181** — round 6's number and round 7's number are the same bike with a different flag on the input struct.

Braking 9.50 m/s² over 12.66→0.95 m/s with both brakes at full — a different window from the 8.37 baseline, not a discrepancy I can call. Pumping gap 0.90 m/s against 3.56 — my pump policy is crude and this is my harness, not the bike.

**Determinism: byte-identical in 15 cross-process boots** (3 per seed × 5 seeds, launched concurrently, load average rising to 11.0). `20260726 grid=3a83fb11 tread=b223fe69 stamps=51122/6e406527 len=2724.9954` · `777 6d8d3187 / 62e35cb4 / 62435 / 2738.6453` · `12345 5f9210bf / 9292a85c / 49029 / 2725.8323` · `2 2027d071 / 58bf868e / 53284 / 2700.2562` · `99999 3de5885b / 2f77f850 / 59208 / 2749.1272`. Distinct fingerprints per seed: **1**.

---

## 8. Adversarial

- **A seed where the march finds no admissible line: three of five.** `routeAdmissible: false` on 12345 (`rideability`, 16 variants, reseeded), 2 (`climb`), 99999 (`rideability`). All three ship anyway. Detected, not hidden — but shipped.
- **The switchback was not removed, it was relocated. Seed 777, arc 404–416** — my ON-arm cells die at 269–422 m, every one of them:

```
 arc  grade%   r(m)  bank°  width  vDes  needLean°  budget(neutral/shift)  fallR
 406    27.1    7.3   10.5   1.63   6.2      28.3       15.9 / 19.2         +5.8
 408    30.8    5.8   10.8   1.65   6.2      34.1       14.0 / 17.8         +4.2
 410    33.1    5.6    9.7   1.66   6.2      35.0        8.5 / 11.3         +2.0
 412    34.4    6.3   12.4   1.67   6.2      31.8        8.0 / 10.9         -0.4
 416    15.6   11.9   10.7   1.68   6.2      18.0       17.6 / 20.4         -0.8
```

  A **5.6 m radius hairpin on a 33% grade** with 10° of bank on a **1.66 m** tread, `speedAt` already pinned at `V_DESIGN_FLOOR`, demanding 25.3° of *contact* lean against a measured ceiling of ~33° p50 / 22° p10. Marginal, not impossible — and past my instrument's own r ≥ 16–20 m limit, so this one is a joint bound.
- **A berm built beyond what the corridor can carry:** the opposite. The corridor-carryability term has removed the steep berms outright (>0.70 |tan bank| empty on all five seeds).
- **A section where the feasibility cost made the trail traverse instead of descend:** seed 2, longest non-descending run 41.3 → **101.5 m**, with `routeFail: 'climb'`. And on seed 2 the drop falls to 383.9 m.
- **Anywhere the bike sticks:** 9 stuck cells in 246, 7 of them seed 12345 ON with the grade-scheduled pitch (my 0.55 pedal cap contributes; I cannot cleanly separate it).

---

## The answer to the question you asked

**FAIL. The cause has RELOCATED, and it is UPSTREAM of the route march.**

Not the bike. Not the carve. Not the launch bound. Not the acceptance metric. Not the steering conversion. It is **the lean-ceiling model that the march is now scored against**, and it is wrong in two compounding ways:

1. **`leanBudget()` omits three terms that dominate on steep ground** — `LEAN_LIMIT_PLANTED` (the 19.5° floor `bike.js` guarantees while any tyre is down), the rider's 0.30 m fore/aft weight shift, and `μ_DIRT = 1.08` (it uses 1.00). It under-predicts the bike's published ceiling by 1.6× on the flat, **14.3× at 40–50% grade**, and returns exactly zero above ~48% where the bike delivers 19.5°.

2. **Its one fitted parameter was calibrated against round 7's synthetic cells, and those cells were produced by an autopilot with `input.pitch` pinned at zero.** Restore that one input and the 45%+r64 cell that this entire round exists to design around goes **0/6 → 6/6 at every speed from 6 to 14 m/s**, and the shipped seed-20260726 course goes **0/21 → 12/21** with the median run going from 469 m to the full 2713 m.

So the round-7 finding was right that the instrument was uncalibrated, and it recalibrated the *steering*. It did not notice that the same instrument was also riding with its weight over the bars on 90% grades. The measurement that founded round 8 — "*measured with no trail involved at all*" — is an artefact of that, and `FEAS_W_BUDGET`, `corridorRide()`, the switchback `rearLoadFrac` gate and the removal of `MU_CORNER_MIN` are all priced off it.

**What survives, and it is not nothing.** Against a lean budget taken from the bike itself, the route change is a monotone five-seed improvement (violations 3.1–5.0% → 1.3–1.8%; corner violations −68 to −85%; worst 25 m bin 64–100% → 30–70%), the 20260726 switchback that killed four rounds is genuinely fixed, `bankConflict` and `legAfter` are 0 everywhere, determinism is byte-identical at load 11, and terrain's "my residual is not what stops the rider" claim reproduces exactly. **What fails**: completion 12/21 on one seed and 0/21 on four against ≥80%; the worst 25 m bin met on no seed; the route is a redraw on 72–100% of its length so nothing is attributable; two seeds got *steeper* (12345 ships 30 m of 49–91% fall line where its own model says the budget is zero); three of five seeds are `routeAdmissible: false` and ship; exposure worsened on two seeds; the steepest berms are gone; and seed 2 still must not ship.

**What I would do next, stated so it is not re-derived:** re-derive `leanBudget()` against `bike.js`'s *measured* ceiling rather than its static free-body diagram — the table in §(b) is the calibration set, and `LEAN_LIMIT_PLANTED` alone changes the model's answer on 45% ground from 2.4° to at least 19.5°. Then re-run the march. Half the ground round 8 spent route geometry avoiding is ground a rider who moves can ride, and the ground that is genuinely bad — 5.6 m hairpins at `V_DESIGN_FLOOR`, and 30 m of 91% fall line — is not what the term is currently priced to refuse.

**Weakest parts of my own evidence, plainly.** My pitch schedules are crude and untuned; the `load`-scheduled variant loops out 14/21 on the same seed the `grade`-scheduled one finishes 12/21 on, so 12/21 bounds nothing except "a rider who moves does dramatically better than one who does not". I re-mapped the grade margin with the shift on but not the radius margin, so the 5.4–8 m hairpin walls remain joint bounds on trail and controller. My `leanBudget()` is a reimplementation, verified against the engineer's published table to 0.01° but not a diff of their code path. My neutral-CG feasibility columns use the defective model as their own criterion and should not be quoted; the empirical-budget table in §1 is the one that means anything. And my residual *max* (0.47 /m) is contaminated by a linear plan polyline where terrain specifies centripetal Catmull-Rom — the p90/p95/p99 figures are the ones I stand behind.