# VERDICT: **FAIL** — and the eighth thing is that the pitch schedule's *saturation* is a free parameter that decides completion. Round 9's central defence ("the wall is the course, not my controller") is false: I move its 452 m wall to 2146 m, and get the project's first finishes, by changing one number in my own rider.

I edited no file. No git, no npm, no browser, no network. Harness is mine, from scratch, in `/private/tmp/claude-501/-Users-stewartwebster-Projects/1bc47d7b-2580-443f-86fb-d67586b192a9/scratchpad/r10/` (`boot.mjs`, `ground.mjs`, `synthworld.mjs`, `drive.mjs`, `probe.mjs`, `pgrid.mjs`, `auto.mjs`, `sweep.mjs`, `audit.mjs`, `runaudit.mjs`, `census.mjs`, `spot.mjs`, `drop.mjs`, `wall.mjs`, `tune.mjs`, `best.mjs`, `gr.mjs`, `s0b.mjs`, `reg.mjs`, `det.mjs`, `hudchk.mjs` + debug files) — all `node --check` clean.

---

## STEP 0 — my instrument, and the three defects I found in it first

**Three harness defects, all found and fixed before any figure was taken.** Stated because two rounds died of exactly these:

1. **Start orientation inverted.** `Matrix4.lookAt(eye, target)` puts local **+Z** at `eye − target`; the bike's forward is local −Z, so the target must be the heading, not its negation. With it backwards the bike starts facing uphill and merely *rolls away backwards* — every run "moved", every metric populated, and the arc-length went **negative**. Round 9 hit the same defect on its helicoid.
2. **Steer sign inverted.** Measured, not reasoned: sign `+1` → finish 1198 m, mean lateral 0.236 m, **0.0% off tread**; sign `−1` → `lost` at 16 m, max lateral 10.05 m, 34.9% off tread.
3. **`trail.nearestT` queried with `y = 0`.** It takes a *world* position; zeroing y put the query 1.7 km under the mountain and it answered from a fold on the far side (mean lateral 35 m on a 3 m trail).

And one **analysis** defect, corrected mid-run: **stations are 0.4 m apart in PLAN, not in 3-D arc** — `(n−1)·ds = 2671.2 m` against `trail.length = 2729.1`. Station index is `t·(n−1)`, not `arc/0.4`. My first dump of the failure site was ~2% displaced and read the wrong stations.

### Known-good course, WITH the pitch schedule

1200 m of analytic 3-D arc, 10% grade, serpentine → **κ_max 0.01571 /m, r_min 63.7 m**, 3.0 m tread, walls rising 0.15 m/m outside both edges, berms at 60% of required lean. `steer = atan(v²·κ_pp/g)/LEAN_MAX`.

| look | 30/60/144 Hz | mean lateral | **off-tread** |
|---|---|---|---|
| 4 | 3/3 | 0.67–1.21 | 37.5% |
| **5–10** | **21/21** | **0.10–0.39 m** | **0.0%** |

### Margins by dial, WITH the reference pitch schedule (9 cells each: 3 Hz × look 5/7/10)

| dial | clean | fails |
|---|---|---|
| roughness λ 8 m | ≤0.15 m (9/9) | **0.20 m → 0/9** |
| roughness λ 3 m | — | **0.05 m → 0/9** |
| exposure | −2.0 m/m → 7/9 | never cleanly |
| tread width | 1.2 m → 9/9 (22.9% off-tread) | never |
| speed | 20 m/s → 9/9 | — |
| **grade (ref pitch)** | **65% → 9/9** | — |
| grade (**pitch 0**) | 30% → 6/9 | **36% → 0/9** |

The pitch schedule is a huge, real effect and round 8's central claim survives intact: **pitch 0 fails from 36% grade; the scheduled rider is clean to 65%.**

### The radius margin round 8 admitted it never re-mapped — here it is, WITH the shift

Joint grade × radius, 9 cells each, at `V_DESIGN_FLOOR` = 6.2 m/s and at 10 m/s:

| r_min | 0% | 10% | 20% | 30% | 36% | 45% | 55% |
|---|---|---|---|---|---|---|---|
| **@6.2 m/s** 64–12 m | 9/9 | 9/9 | 9/9 | 9/9 | 9/9 | 9/9 | 9/9 |
| @6.2 m/s **9 m** | 2/9 | **0/9** | 0/9 | 0/9 | 0/9 | 0/9 | 0/9 |
| @6.2 m/s 7 / 5.5 m | 0–1/9 | 0/9 | 0/9 | 0/9 | 0/9 | 0/9 | 0/9 |
| @10 m/s 16 m | 9/9 | 9/9 | 9/9 | 9/9 | 9/9 | 8/9 | 7/9 |
| @10 m/s 12 m | 9/9 | 9/9 | 7/9 | 8/9 | **0/9** | 0/9 | 0/9 |

**The shift buys steeper ground, not tighter corners.** My instrument cannot certify r < 12 m at any grade, even at the design floor speed. The ON-arm trails ship `rMin ≈ 5 m` but `r p1 = 17–25 m`, so **fewer than 1% of stations are inside my blind spot** — the hairpin walls of rounds 7–9 are no longer the binding constraint, and my completion figures are not joint-bounded by them.

---

## THE EIGHTH THING — completion is a function of `pMax`, not of the course

Round 9 defends its 0/21 with: *"The wall is the course, not my controller, and I can show it… three different pitch schedules — mine, round 8's, and a deliberately steeper one — all die at 451–455."* **All three run to full aft shift.** Round 9 varied the *slope* of the schedule (`g0`) and never varied its *saturation*.

I gridded `pitch = −clamp(tanθ/g0 + decel/a0, 0, pMax)` over g0 ∈ {0.30…1.50}, a0 ∈ {8,12,25}, pMax ∈ {0.4,0.6,0.8,1.0}, 6 cells each, seed 20260726 ON:

| family | median | cells at 452–465 m |
|---|---|---|
| **pMax = 1.0** (round 9's whole family) | **453 m** | 5/5 of the worst configs, all at med 453 |
| pMax = 0.8 | 453 m | 16/21 |
| **pMax = 0.6, g0 = 0.4** | **2146 m** | **1/21** |

Full 21-cell matrices (look 4–10 × 30/60/144), ON arm, `fpStable: true`:

| seed | pMax 1.0 | pMax 0.8 | **pMax 0.6** | pMax 0.6 + human lag |
|---|---|---|---|---|
| 20260726 | 0/21, med **453** | 0/21, med 453 | 0/21, med **2146** | 0/21, med 1379 |
| **777** | 0/21, med 1035 | 0/21, med 1145 | **3/21 FINISH**, med 738, max **2734** | **1/21 FINISH** |
| 12345 | 0/21, med 206 | 0/21, med 201 | 0/21, med 201 | 0/21, med 200 |
| 99999 | 0/21, med 164 | 0/21, med 164 | 0/21, med 153 | 0/21, med 144 |

**Three cells complete the whole 2734 m of seed 777** — the first completions ever measured on this project with a calibrated controller on a shipped build. And a deliberately *degraded* human-plausible rider (0.30 s reaction lag, hip rate limited to 1.5 /s) finishes **2/6** on 777 where the instantaneous "optimal" one finishes 0/6.

**Consequence for the whole project's evidence base:** on one course, one controller, one world, the median run ranges **453 m → 2146 m** and completion **0 → 3/21** on a single scalar of the instrument. Round 8's 12/21 and round 9's 0/21 both sit inside that band. **No completion figure quoted on this project bounds the trail; every one of them must be quoted with its schedule *and its saturation*.** Round 8 said as much about its own number; round 9 did not, and built its verdict on the omission.

### What is actually at 452 m — named, not inferred

`FEATURE drop, arc 447.1 → 456.5, {height: 1.4397, landing: "natural"}`, inside `FEATURE chute 398.9 → 477.7`.

| station | arc | grade | κ_v | `S.speed` | v_launch | **v²κ/g** |
|---|---|---|---|---|---|---|
| 1096 | 447.2 | −18.2% (*climbing*) | 0.139 | 5.82 | 8.41 | 0.48 |
| 1100 | 448.8 | +6.6% | 0.450 | 5.40 | 4.67 | **1.34** |
| 1102 | 449.6 | +23.8% | **0.768** | 5.49 | **3.57** | **2.36** |
| 1104 | 451.0 | **+210.2%** | 0.019 | 5.92 | — | — |

The trail climbs 23% for ~4 m into a lip, then a **1.44 m face at 210% gradient with an unshaped ("natural") landing**, and publishes a design speed **54% above its own launch speed** at the lip. CONTRACT §4: *"a jump you can't land is a bug."* It is present on **both** arms (OFF worst ratio 3.45 @ 451) — pre-existing, untouched by this round, and mis-described by round 9 as "54–65% grade on ROOT" (it is 210% on DIRT).

At full aft shift the rider loops out or lands off-axis over it (13 offaxis + 5 loopout in my 21 cells). At 60% shift they ride it.

---

## 1. Is the recalibrated `leanBudget()` faithful to the real bike? **YES — independently reproduced.**

I probed the real `bike.js` + `collision.js` over my own analytic constant-grade ground (no grid, no trail), my own governor, my own skid modulation, at the reference schedule, p10 of `state.leanLimit`, decel-0 column = worst of v0 ∈ {6.2, 10, 15}:

| grade | 0% | 10% | 25% | 35% | 45% | 55% | 65% | 70% | **75%** |
|---|---|---|---|---|---|---|---|---|---|
| trail.js `RIDE_CEIL[·][0]` | 41.73 | 40.08 | 38.65 | 38.65 | 37.46 | 36.94 | 35.41 | 31.76 | **0** |
| **mine, measured** | **42.28** | **40.44** | **41.47** | **39.11** | **39.66** | **37.25** | **34.63** | **30.63** | **0** |

Agreement 0.5–3°, with trail.js's monotone lower envelope sitting at-or-below mine everywhere — the conservative direction. **The cliff at 0.75 reproduces exactly**: the bike crashes on a dead-straight run at every deceleration, one grid step past its own shifted endo point (0.886/1.05 = 0.844). In the decel ≥ 4 corner at 45–70% my raw grid is non-monotone (0.65 → 20.1 → 23.3 recoveries) where trail.js takes 0.6; that is the envelope being conservative, not a disagreement.

**This is a genuine measurement of `bike.js`, not a model of it, and it stands.** It is the round's real achievement and it should not be re-litigated.

The one thing the table does **not** license, unchanged from round 9's own disclosure: `leanLimit` is what the bike *reports* on a straight run, not what it *survives* in a corner. My radius margin above is the missing half, and it says the design share (0.596) is fine at r ≥ 12 m and worthless below 9 m.

---

## 2. Joint feasibility on the COMMITTED surface (my stencil, my measured budget — **only the A/B means anything**)

Plan radius = circumradius at ±4 m plan stride; grade = central difference of committed height at ±2 m of 3-D arc, sin→tan; cross-slope secant at ±0.36·width; κ_v = second difference at half-stencil 0.37 m; v swept `speedAt` → 6.2 in 21 steps.

| seed | violation % OFF → ON | worst 25 m bin OFF → ON | bins > 20% | corner viols |
|---|---|---|---|---|
| 20260726 | 7.05 → **5.36** | 100% @2175 → **100% @2125** | 11 → 7 | 469 → 357 |
| 777 | 14.79 → **3.50** | 100% @1075 → **47.5% @700** | 21 → 8 | 967 → 233 |
| 12345 | 10.01 → **16.83** | 100% → 100% @1325 | 16 → **35** | 656 → **1106** |
| 99999 | 2.41 → **8.00** | 36.5% → **100% @150** | 4 → **15** | 158 → **528** |
| 2 | 12.05 → *generation refuses* | — | — | — |

**Target (<0.5%, worst bin <5%) met on no seed.** Two seeds improve, two regress. My absolute numbers are ~5× round 9's because my criterion is stricter; I do not reproduce theirs and do not claim to.

**The criterion-free version, which I do stand behind** — the trail's own published `S.speed` against the committed vertical curvature: ON arm **30 / 30 / 74 / 31** stations with `v²κ/g > 1`, worst ratio **2.92–3.61**. The trail's own `safety.launchViolAfter` on the ON arm: **135 / 142 / 2405 / 809**. It measures this and ships it.

---

## 3. Autopilot sweep — full A/B, one process per seed, `preFp` asserted identical between arms

21 cells (look 4–10 × 30/60/144 Hz) on 20260726/777/12345, 6 on 2/99999. `fpStable: true` on every matrix. My OFF arm reproduces the published control exactly: `20260726 len 2728.2105`, `777 2687.3259`.

| seed | arm | sched | fin | median | min | max | spread | mean lat | off-tread | air | causes |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 20260726 | OFF | ref | 0/21 | 322.6 | 103.8 | 458.7 | 4.42 | 0.579 | 11.8% | 3.2% | lost 9, offaxis 5, lowside 3, loopout 3, impact 1 |
| 20260726 | ON | ref | 0/21 | **452.8** | 103.0 | 1144.6 | 11.1 | **0.322** | **3.4%** | 1.7% | offaxis 13, loopout 5, lost 3 |
| 20260726 | ON | pitch 0 | 0/21 | 330.6 | 94.3 | 441.8 | 4.69 | 0.648 | 14.3% | 3.5% | lost 14, lowside 6, washout 1 |
| 20260726 | ON | **pMax 0.6** | 0/21 | **2146** | 99 | 2146 | — | 0.37 | 4.5% | 4.1% | **stuck 11**, lost 5, loopout 2 |
| 777 | OFF | ref | 0/21 | 222.3 | 206.4 | 523.4 | 2.54 | 0.327 | 3.2% | 6.9% | loopout 10, offaxis 9, lowside 2 |
| 777 | ON | ref | 0/21 | **1034.8** | 114.3 | 1148 | 10.1 | 0.377 | 6.6% | 1.1% | loopout 10, lost 9 |
| 777 | ON | **pMax 0.6** | **3/21** | 738 | 157 | **2734** | — | 0.49 | 9.1% | 3.0% | **finish 3**, lost 16, impact 1 |
| 12345 | OFF | ref | 0/21 | 315.9 | 84.6 | 318.3 | 3.76 | 0.631 | 11.6% | 3.0% | lost 17, offaxis 4 |
| 12345 | ON | ref | 0/21 | **208.0** | 138.2 | 246.1 | 1.78 | 0.212 | 1.8% | 5.0% | lowside 13, offaxis 8 |
| 99999 | OFF | ref | 0/6 | 491.2 | 158.5 | 1129.9 | 7.13 | 0.622 | 17.5% | 5.8% | offaxis 2, loopout 2, lost 2 |
| 99999 | ON | ref | 0/6 | **163.0** | 156.4 | 187.7 | 1.20 | 0.432 | 8.3% | 1.3% | lowside 4, washout 1, offaxis 1 |
| 2 | OFF | ref | 0/6 | 1141.2 | 343.4 | 1141.3 | 3.32 | 0.471 | 9.4% | 1.5% | lost 3, loopout 3 |
| 2 | ON | — | — | *generation refuses (3/3 boots, identically)* | | | | | | | |

**Completion ≥ 80% is NOT met. Best case across all schedules: 3 finishes in 84 cells = 3.6%.**
**Two seeds are made dramatically worse by the round: 12345 median 316 → 208, 99999 median 491 → 163.**

0 NaN, 0 tunnelling in 400+ runs.

---

## 4. Did the route reclaim steep ground, and is it still a downhill course?

| metric OFF → ON | 20260726 | 777 | 12345 | 99999 | 2 |
|---|---|---|---|---|---|
| 3-D length | 2735→2734 | 2693→2743 | 2699→2726 | 2700→2732 | refuses |
| drop | 466→468 | 454→**506** | 465→**499** | 480→**516** | |
| mean grade | .181→.182 | .179→.195 | .182→.196 | .186→.201 | |
| stations > 30% | 12.0→**9.9** | 15.4→**18.8** | 13.8→**19.7** | 11.4→**19.2** | |
| stations > 45% | 2.1→1.6 | 2.8→1.3 | 2.0→**6.1** | 2.1→**4.4** | |
| grade p99 / max | .531→.509 / .75→.76 | .692→**.459** / .92→.72 | .544→**.676** / .84→.84 | .551→**.638** / 1.19→.90 | |
| r min / p1 | 5.1/16.5→5.1/**24.8** | 5.0/6.5→4.8/**18.3** | 5.1/6.3→5.0/**16.0** | 5.4/12.1→5.5/**17.2** | |
| cross-slope .35–.70 / >.70 | 120/25 → **0/0** | 118/22 → 35/0 | 115/43 → 42/35 | 175/0 → 43/**42** | |
| features / splits / cp | 28→26 / 6 / 8 | 35→31 / 6 / 8 | 33→34 / 6 / 8 | 40→31 / 6 / 8 | |
| exposed m / unprotectable | 1132→1090 / 5→**37** | 1290→**1742** / 0→**11** | 1168→**782** / 13→**0** | 1133→**849** / 6→**2** | |
| `launchViolAfter` | 211→135 | 1139→**142** | 377→**2405** | 513→**809** | |
| **`routeAdmissible`** | — → **true** | — → **true** | — → **true** | — → **true** | throws |

**Not flattened** — three of four seeds gain steep ground and 30–50 m of drop. **The reference seed goes the other way** (>30% 12.0 → 9.9%, r p1 16.5 → 24.8 m, all cross-slope above 0.35 gone): 20260726 ships flatter and straighter than its control.

**`routeAdmissible` is `true` on all four buildable seeds and seed 2 fails loudly and deterministically.** Three audits demanded this; it is delivered. Real win.

### Seeds 12345 and 2 — one fixed, one not

- **Seed 2 no longer generates.** All 16 variants fail `climb` (18.8–93.2 m against a 12 m gate); the throw is identical on 3/3 concurrent boots. Correct.
- **Seed 12345 still ships the defect round 8 named.** Committed ground above the gradient at which *this round's own measured ceiling is zero* (0.75·μ_surf/μ_dirt), longest continuous run:

| seed | OFF → ON | at arc |
|---|---|---|
| 20260726 | 1.7 → 0.8 m | 453 |
| **777** | **20.5 → 0.0 m** | — |
| **12345** | **4.1 → 12.9 m** | **202** |
| **99999** | **9.0 → 12.6 m** | **154** |

Seed 12345, arc 195–217: grade runs 30 → 38 → 47 → 57 → 67 → 75 → 81 → 84 → 88 → 82 → 80 → 78 → 76 → 74 → 71%, on 3.00 m of tread with **−1.9° of bank**, `S.speed` pinned at 6.20. Seed 99999, arc 153–167: 80 → 73 → **98.7** → 82 → 89 → **98.5** → 70 → 77 → 80%, 3.6–5.0° of bank. **Both autopilots die there** (12345 at 201 m, lowside 13; 99999 at 155 m, lowside/washout) **under every schedule I tried.**

**`FEAS_W_GROUND` — the term this round added specifically to refuse this ground — fixes seed 777 completely and makes 12345 and 99999 worse on its own metric.**

### One new defect on the reference seed

Seed 20260726 ON, **arc 2110–2154: 26 m of continuous non-descending trail** (OFF arm: 15.9 m), grades running −3.5%, −7%, −18%, −20%, −21.4% on ROCK, with `S.speed` collapsing to **1.00 m/s**. CONTRACT §4: *"the trail must never climb for more than ~15 m."* This is where my 11 best-schedule cells report `stuck`. Every ON seed exceeds the 15 m limit: **16.7 / 18.3 / 22.8 / 26.0 m**.

---

## 5. Rider and HUD — the best work in the round

**`rider.js` arithmetic verified exactly.** `steep = smoothstep(0.10, 0.50, g)`; un-commanded `D.fore = 0.24·steep` → **0.000 / 0.120 / 0.230** at 0/30/45%; full command → **−1.000 / −1.200 / −1.383 (clamped −1.35)**. Player-owned hip travel = `aft·0.22 + fwd·0.17` → **0.220 / 0.284 / 0.332 m**. Every published figure reproduces. Travel now *grows* with grade where it used to shrink to 74 mm. `state.riderFore` is genuinely published by `bike.js:2632` and is never re-derived.

Judged as a player: 0.33 m of hip translation plus a 0.36 m head drop and arms going 74% → 98% extended, on a 1.8 m rider at ~7 m of chase distance, is **legible** — it is a whole-silhouette change, not a limb twitch. The deliberate few-cm *forward* neutral pose on a steep is honest but will not read.

**HUD — I attacked the caveat its author asked me to attack, and it holds.** They flagged that `BAL_LO = 0.16` / `BAL_FRONT_LO = 0.18` were fitted to round 8's *static free-body* figures, not live loads. I measured the live rear-load fraction on the real bike:

| grade | pitch 0 | pitch −0.5 | pitch −1 |
|---|---|---|---|
| 0% | 0.510 | 0.751 | **0.980 → CRASHED (loop-out)** |
| 30% | 0.284 | 0.513 | 0.783 |
| **45%** | **0.113 / p10 0.071** | 0.340 | 0.640 |
| 55% | 0.047 / p10 0.000 | 0.290 | **0.982 → CRASHED** |

At 45% with weight over the bars the live rear fraction is **0.113** — the HUD calls REAR LIGHT (critical) and it is right. At full aft shift on flat/moderate ground the front fraction goes to 0.02 and the bike **actually loops out** — the HUD calls FRONT LIGHT and it is right. **The thresholds are validated against live loads, which is more than their author claimed.** The cue is correctly grade-gated so the front-light state cannot be provoked by following the HUD.

Note the convergence: the HUD's front-light band and my `pMax = 0.6` finding are the same physics said twice. **The HUD is currently the only module in the project that knows full shift is over-shift.**

Confirmed stale-comment defect: `input.js:118` says *"contract pitch is + = back/up"*, against its own code (`W → +1`) and `bike.js`'s CONTRACT-NOTE (*"+1 is weight FORWARD / nose DOWN"*). Code and `bike.js` agree; the comment is wrong.

---

## 6. Regression and determinism — clean, bit-exact

| metric | reference | measured |
|---|---|---|
| static sag fork / shock | 29.4% / 26.7% | **29.4% / 26.7%** |
| terminal v, 15% grade, `anyPressed=false` | 17.181 | **17.181** |
| ... `anyPressed=true` | 16.885 | **16.885** |
| rear lockup | locks | **min spin/rolling 0.000, 1920 skid frames** |
| step symmetry L/R @30/60/144/240 Hz | 0.000% | **0.000%** at all four (∓30.087°) |
| 300 s soak | 0 NaN | **0 NaN** |

**Determinism byte-identical across 15 concurrent cross-process boots** (3 per seed × 5 seeds), including three identical refusals of seed 2:
`20260726 pre=5078f357 post=8ce517d3/873923ae stamps=53964/b9fe9756 len=2729.1026` · `777 900de7bc/0bbaea0a 63110 2735.5803` · `12345 8f36f7df/1b748629 51887 2713.8332` · `99999 d8b04351/6a045c17 52218 2725.2375`.

**Jumps preserved** on 20260726: 8 air features both arms, air times within 0.05 s (1.06/1.24/1.49/1.50/1.60 → 1.03/1.23/1.45/1.49/1.52). 777 gains 3 air features (5 → 8) — a redraw symptom, as round 9 said.

---

## 7. Adversarial

- **A seed with no admissible line:** seed 2, and it now **throws** — 16 variants, 18.8–93.2 m of climb, identical on every boot. Correct behaviour, and a refusal not a fix.
- **A berm beyond what the corridor carries:** the opposite, still. On the reference seed **every** station with committed cross-slope above 0.35 is gone (120 → 0).
- **Where the bike sticks:** arc 2110–2154 on 20260726 ON — 26 m of non-descending ROCK with the design speed at 1.00 m/s. 11 of 21 best-schedule cells end there. That is the trail, not my governor.
- **A pitch schedule under which the course becomes UNRIDEABLE:** yes, and it is the one the trail is designed around. `pMax = 1.0` — the saturation of `RIDE_PITCH_G`/`RIDE_PITCH_A`, i.e. the reference rider the whole course is now priced off — turns 2146 m into 453 m on seed 20260726 and 2734 m into 1145 m on 777. **The design depends on rider skill in a way that must be stated: the course is priced for a rider who goes fully back on steeps, and a rider who does that cannot land its first drop.**

---

## The answer to the question you asked

**FAIL. The cause has RELOCATED — back into the instrument, one layer deeper than round 7 or round 8 found it.**

Round 7 found the steering *units* were wrong. Round 8 found the pitch *axis* was never used. Round 9 measured the ceiling honestly and then — quoting a 0/21 with 20 of 21 cells at 451–462 m — concluded the course was the wall. It is not. Every schedule it tested ran the shift to saturation, and **saturation is the parameter that matters**: cap it at 0.6 and the same controller on the same world goes 453 → 2146 m and produces this project's first three finishes.

**What genuinely survives this round and should not be re-litigated:**
- `RIDE_CEIL` is a real measurement of `bike.js`. I reproduce it independently to 0.5–3°, including the exact 0.75-gradient cliff at the shifted endo point.
- `routeAdmissible: true` on all four buildable seeds; seed 2 refuses loudly, deterministically, with a readable diagnosis.
- Seed 777 is transformed: dead ground 56 → 0 stations, feasibility 14.8 → 3.5%, grade p99 .692 → .459, r p1 6.5 → 18.3 m, `launchViolAfter` 1139 → 142, longest non-descending 57.5 → 22.8 m — **and it finishes.**
- Bike regression bit-exact; determinism byte-identical at 15 concurrent boots; jumps preserved on the reference seed.
- The rider and HUD work is correct, verifiable, and the HUD's disputed thresholds hold against live loads.

**What still fails, plainly:**
1. **Completion 3/84 (3.6%) at best, against ≥80%.**
2. **`FEAS_W_GROUND` regresses two of the four seeds on its own metric** — seed 12345 still ships 12.9 m of committed ground above the gradient where this round's own table returns zero, at arc 202, and it kills every run at ~200 m. Seed 99999 the same at arc 154.
3. **A new CONTRACT §4 breach on the reference seed**: 26 m of non-descending ROCK at arc 2110 with the design speed at 1.00 m/s (OFF arm: 15.9 m). Every ON seed exceeds the ~15 m limit.
4. **The course's first `drop` (arc 447, 1.44 m, `landing: "natural"`) publishes a design speed 54% above its own launch speed** and is present on both arms — a pre-existing, unaddressed *"jump you can't land"*.
5. Worst 25 m feasibility bin met on no seed; all cross-slope above 0.35 gone on the reference seed; exposure worsened on 777 (1290 → 1742 m).

**Could a competent human with a gamepad ride this top to bottom?** On **seed 777, yes** — a rate-limited, 0.30 s-lagged, human-plausible rider finished it twice out of six, and the calibrated one three times out of 21. That is a hard run, not an impossible one. On **seed 20260726, no**: they would ride to about 2146 m and then be off the bike pushing up 26 m of 20% rock. On **12345 and 99999, no**: they would crash inside the first 200 m on ground the game's own measured model says has zero lean budget. **One of four shippable seeds is rideable by a good player. That is not a game.**

**What I would do next, so it is not re-derived:** (a) fix the acceptance metric before anything else — every completion figure must carry `(g0, a0, pMax)` and a saturation sweep, or it measures the rider; (b) find why `FEAS_W_GROUND` refuses seed 777's fall line and not 12345's arc-202 or 99999's arc-154 — same term, same surface class, opposite outcome, and that is one debugging session, not a redesign; (c) `enforceDescent` is letting 26 m of climb through on the shipped line while `routeAdmissible` passes the marched one; (d) give the `drop` feature a solved landing, or cap `S.speed` at its own `v_launch`.