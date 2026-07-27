# VERDICT: **FAIL** — and the ninth thing is that the free parameter moved again. It is not the pitch saturation. On my instrument saturation changes nothing (medians 1234 / 1237 / 1206 m across pMax 0.4 / 0.6 / 1.0 on the reference seed), while **one line of my own steering policy moves the same median 304 → 672 → 1237 m**. Bug 4 is not merely unfixed; this round supplies a fresh instance of it, and it is bigger than the one round 10 found.

I edited no file. No git, no npm, no browser, no network. Harness is mine, from scratch, in `/private/tmp/claude-501/-Users-stewartwebster-Projects/1bc47d7b-2580-443f-86fb-d67586b192a9/scratchpad/r12/` — `boot.mjs`, `synth.mjs`, `drive.mjs`, `probe.mjs`, `s0.mjs`/`s0b.mjs`, `rad.mjs`, `sv.mjs`, `s12345.mjs`, `dump.mjs`, `an.mjs`, `climb2.mjs`, `dead2.mjs`, `jaud.mjs`, `launch.mjs`, `integ.mjs`, `matrix.mjs`, `seg.mjs`, `adv.mjs`, `reg.mjs`, `det.mjs`, `y0.mjs`, `prof.mjs`/`fine.mjs`/`tr.mjs`, tuning files — all `node --check` clean. Every matrix is one process, world fingerprint asserted before and after: **`fpStable: true` on every one**.

**My control is faithful.** My boot reproduces the shipped build to the decimal on all four seeds: `20260726 len=2729.7725 stamps=56420`, `777 2733.6882 / 62919`, `12345 2711.7345 / 51892`, `99999 2712.4621 / 55619` — identical to the engineer's published determinism line.

---

## STEP 0 — my instrument, and the two defects I found in it before quoting anything

### The eight known defects, checked against MY code by measurement

| # | defect | my check | result |
|---|---|---|---|
| 1 | start orientation | `forward · tangent` after `reset({t:0})` **and** after `reset(null)`→`startTransform` | **0.9987 both** (I had written `lookAt(0, −tangent)`; caught and corrected before any figure — `Matrix4.lookAt` puts local **+Z** at `eye−target`, so the target IS the heading) |
| 2 | steer sign | measured, not reasoned: constant `steer` on straight analytic ground | `+0.30 → +14.83 m` lateral, `−0.30 → −14.83 m`. SIGN = +1 |
| 3 | `nearestT` world-y | queried the real trail with the true y and with y=0 | y=0 is wrong by up to **380.6 m of arc and 63.2 m of lateral**. My harness never zeroes y |
| 4 | `ctx.input.state` | asserted identity; `bike.js:2820` reads `context.input.state` | plumbed correctly |
| 5 | plan-vs-arc indexing | station index = `t·(n−1)`; all arcs integrated in 3-D from `S.px/py/pz` | `(n−1)·0.4 = 2675.2` vs `trail.length = 2729.77` — a 2.0% error if used |
| 6 | steering units | `steer 0.1/0.2/0.4 → lean 5.63°/11.34°/21.11°` vs `steer·LEAN_MAX = 5.61/11.23/22.46` | commanded lean = `steer·LEAN_MAX·speedFrac`; conversion confirmed |
| 7 | pitch axis | `riderFore` at pitch 0 / −0.4 / −0.6 / −1.0 | `0 / −0.120 / −0.180 / −0.300 m` — exactly `pitch·RIDER_SHIFT` |
| 8 | pitch saturation | explicit `pMax` dial, swept | see below |

### The ninth: the ground weight-shift schedule is not a valid AIR input

`bike.js` kills its auto-stabiliser above `AIR_STAB_DEADZONE = 0.16` on `|pitch|+|roll|` (line 2366), and in `airStep` a negative `pitchIn` applies `−pitchIn · AIR_PITCH_TORQUE` about `+right`, which is **nose-up**. A harness that runs `pitch = −0.6…−1.0` continuously through a flight therefore (a) switches off the game's landing assist and (b) rotates the nose up on the way down. Ablation on the shipped reference seed, 21 cells each:

| | pMax 0.4 | pMax 0.6 | pMax 1.0 |
|---|---|---|---|
| schedule gated to the ground (`air=zero`) | med **1234** | med **1237** | med **1206** |
| schedule held through the air (`air=hold`, what rounds ≤10 did) | med 723 | med 1153 | med 1142, **loopout 13/21** |

The mechanism is real (13 loopouts vs 4) but it is worth ~7% of median, not 4.7×.

### The tenth, and it is the one that matters: **the lookahead policy dominates completion**

Same world, same fingerprint, same 9 cells, same pitch schedule (`g0 0.55, a0 12, pMax 0.6`), seed 20260726 ON — only the pure-pursuit lookahead policy changed:

| policy | median | max |
|---|---|---|
| speed-scaled, `look + 0.5·v` | **304 m** | 304 |
| fixed dial, 5–9 m (rounds 7–11) | **672 m** | 1240 |
| **fixed dial capped at 0.6·r_local** | **1237 m** | 1254 |

A 4.1× swing on one line of the controller, against a 2.6% swing across the whole saturation range. **Every completion figure on this project — round 8's 12/21, round 10's 3/21, round 11's 9/252, and mine — is a reading of its author's steering policy first and the trail second.**

### Known-good synthetic course, with the final controller

1200 m of analytic 3-D arc (closed-form ground, no grid), 10% grade, serpentine to **r_min 63.7 m**, 3.0 m tread, walls rising 0.15 m/m outside both edges, berms at 60% of required lean.

| look | 4 | 5 | 6 | 7 | 8 | 10 | **all 18 cells (×30/60/144 Hz)** |
|---|---|---|---|---|---|---|---|
| finishes | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | **18/18** |
| mean lateral | 0.38 | 0.41 | 0.34 | 0.19 | 0.15 | 0.26 | **0.288 m** |
| off-tread | 0.1% | 0 | 0 | 0 | 0 | 0 | **0.0%** |

**Margins by dial** (9 cells each: look 5/7/10 × 30/60/144 Hz):

| dial | clean | fails |
|---|---|---|
| roughness λ 8 m | ≤ 0.15 m (9/9) | 0.20 m → 6/9, **0.30 m → 0/9** |
| roughness λ 3 m | ≤ 0.05 m (9/9) | **0.10 m → 0/9** |
| exposure | −2.0 m/m → 9/9 | never |
| tread width | 1.2 m → 9/9 (8.5% off-tread) | never |
| design speed | 20 m/s → 9/9 | — |
| **radius @ 6.2 m/s, 15% grade** | **r 7 m → 9/9** | r 5.5 m → 6/9 |
| **radius @ 10 m/s, 15% grade** | r 30 m → 9/9 | r 20/16 → 6/9, r 12 → 3/9, **r 9 → 0/9** |

**The radius blind spot round 10 admitted (nothing below 12 m certifiable) is closed by the adaptive lookahead**: 9/9 down to r = 7 m at the design floor speed. That matters, because it means the switchback walls below are trail findings, not instrument findings.

### Grade × saturation on smooth ground — and it runs the OPPOSITE way to round 10

9 cells each, v = 10 m/s, berms at 60% of required lean:

| grade | 10% | 20% | 30% | 36% | 45% | 55% | 65% | 75% |
|---|---|---|---|---|---|---|---|---|
| pitch = 0 | 9/9 | 9/9 | **0/9** | 0/9 | 0/9 | 0/9 | 0/9 | 0/9 |
| **pMax 0.6** | 9/9 | 9/9 | 9/9 | 9/9 | 8/9 | **0/9** | 0/9 | 0/9 |
| **pMax 1.0** | 9/9 | 9/9 | 9/9 | 9/9 | 9/9 | **9/9** | 8/9 | 7/9 |

Round 8's finding survives (pitch 0 dies at 30%). **Round 10's does not: capping the shift at 0.6 costs the rider every gradient above ~50%.** The cap did not find headroom in the course; it traded steep ground for a landing behaviour that a properly air-gated controller gets for free.

Independently, on flat-banked ground at r 75 m — the exact geometry of seed 12345's arc-202 — the bike's zero-budget gradient reproduces **at 0.75, by a completely different method** (whole-run completion rather than reported `leanLimit`): 0.72 rides at pMax 1.0 (9/9) and nowhere else; **0.78 and above is 0/9 at every saturation, every radius 60–90 m, with and without a berm, and at 6.2 / 8 / 10 / 12 m/s.** `RIDE_CEIL` is confirmed a third time.

---

## 1. THE THREE BUGS

### BUG 1 — `FEAS_W_GROUND`. **Mechanism confirmed. Half-closed, exactly as the engineer states.**

I reproduce the engineer's mechanism from the file's own numbers and my own measurement of the shipped line. Dead ground = committed 3-D gradient (±2 m central difference) above `0.75·μ_surf/μ_dirt`, longest continuous run:

| seed | OFF (`feasible:false`) | ON | engineer's claim |
|---|---|---|---|
| 20260726 | 1.4 m @308 | **0.0** | 2.8 → 0.0 ✓ |
| 777 | **21.4 m @188** | **0.0** | → 0.0 ✓ |
| **12345** | 5.1 m @2275 | **12.8 m @202 (max 0.84)** | 12.9 → 12.9, **NOT FIXED** ✓ |
| 99999 | 9.5 m @149 (max 1.28) | **4.0 m @1551** | 12.1 → 3.1 ✓ |

**Seed 99999 arc 154 is genuinely fixed** — the committed line there now runs 0.10–0.35 at 12.49 m/s where the old one ran 0.80–0.99.

**Seed 12345 arc 202 is not, and I can price it.** Arc 198–216: gComm 0.53 → 0.72 → **0.82 → 0.84** → 0.77 → 0.58, on 3.00 m of tread, bank −1.9°, `S.speed` pinned at the 6.20 floor, plan radius 60–90 m. Excavation `rawS − py` = **7.64 m at arc 201** against `BENCH_CUT_TOTAL = 7.0` — the design line is at/over the ceiling and the profile solve cannot flatten it. The engineer's diagnosis is correct in every particular.

And my synthetic replica of that exact cell says it is **unrideable, full stop**: 0/9 at pMax 0.4/0.6/0.8/1.0, r 60/75/90 m, with and without a berm, at four speeds. Failure mode `washout` in 7–9 of 9. On the shipped seed, 11–19 of 21 cells die there with `lowside`, median 210 m.

`FEAS_BUDLOW_AT_ZERO = 0` in the shipped file — the ramp that would price this is present, documented, measured and **disabled**. The refusal band therefore still runs from a 20.9° budget down to 3.0°, and on a face where the whole fan is steep the march still optimises onto it.

### BUG 2 — `enforceDescent`. **NOT FIXED. The metric was fixed and the breach was not.**

The engineer's `safety.flatShipped` = 11.2 / 11.2 / 11.3 / 11.3 m reproduces exactly on my reading of the shipped `S.py`. It is measuring the wrong thing.

`enforceDescent` (trail.js:3663): `if (v >= y[i-1] - 1e-5) run += ds; else run = 0;` and rule 3 discharges itself with `DESC_MIN_FALL·ds` = **4 mm** of fall. So the counter is reset by a 10 µm dip and satisfied by a 4 mm one. The profile is now a sawtooth: climb 11 m at up to `maxUpGrade` = 3.5%, drop 4 mm, climb 11 m again.

Measured on the shipped centreline, **longest stretch that ends higher than it started and never drops below its own start** — which is what a rider pushing up it experiences, and what CONTRACT §4's "~15 m" means:

| seed | OFF | **ON** | peak rise | min design speed inside it |
|---|---|---|---|---|
| **20260726** | 79.8 m @2148 | **89.4 m @2157** | **+1.59 m** | **1.00 m/s** |
| 777 | 188.7 m @1206 | **30.4 m @2494** | +0.60 | 7.00 m/s |
| 12345 | 81.7 m @1975 | **46.4 m @1316** | +2.25 | 2.78 m/s |
| 99999 | **14.0 m @1528** | **33.8 m @1059** | +2.15 | 3.80 m/s |

Station-by-station at arc 2156.6 → 2223.9 on the reference seed: `1316.043 → 1317.637`, i.e. **1.59 m of climb over 67 m**, not back below the start until arc 2246.5. The `^` run is broken only by 1–2 cm dips at 2161.4, 2171.8, 2183.8, 2185.0 and 2194.2 — the exact reset events. Natural ground there is 1324.5 while the tread is at 1317.2: a **7.35 m cut**, i.e. the excavation escape in `enforceDescent` is what is actually binding.

Two seeds improve dramatically, **two get worse**, and the reference seed is worse than its own control. The march gate still reads `non-descending <= 90 m` in the failure message — 6× the CONTRACT limit — and the shipped reference seed lands at 89.4 m, one metre inside it.

### BUG 3 — the unlandable drop. **FIXED. And the inverse defect is now the binding wall.**

Independent audit of every `drop`/`gap`/`stepDown`/`jump`/`doubles` on all four seeds, reading `S.speed` at the feature's own published `iLip` (not the feature's self-report):

- **32 of 32 air features on the ON arm are solved** — `landing: "natural"` is gone; every one publishes `takeoffSpeed`, `landingDistance`, `airTime`, `landingAngle` and a `speedWindow`.
- **0 of 32 have a lip design speed above their window.** The round-10 exhibit — seed 20260726 `drop` at arc 447 — now reads `vLip 6.20 / vTakeoff 7.55, ratio 0.82, window [4.1, 14.3]`, inside.
- **20 of 32 are BELOW the minimum of their own window** (`jumpWindowBad = 20`, reproduced exactly). `double-1` 5.49 vs window `[10.9, 19.4]`; `road gap (A)` 5.71 vs `[11.1, 18.6]`; `step-down` 5.95 vs `[11.5, 18.7]`; `finish booter` 6.19 vs `[11.0, 17.3]`. **The design tells the rider to arrive at half the speed the jump was built for.** The OFF arm is 20/32 too — pre-existing, untouched.

That is not cosmetic: **it is the wall on the reference seed.** Every one of my 21 cells dies at arc 1206–1301 — inside `doubles 1136–1264`, `road gap 1270–1331` — with `offaxis`/`loopout`/`lowside`. Riding the published windows instead of the design speed makes it worse (median 1237 → 1062); pushing the pace 15% loses the 8.3 m berm at 300 m instead; all four air-control policies wall in 1140–1370.

---

## 2. Saturation-swept sweep — 21 cells × 3 saturations × 4 seeds, plus the ablations

`(g0, a0) = (0.55, 12)`, `pitch = −clamp(tanθ/g0 + decel/a0, 0, pMax)`, gated to ground contact, adaptive lookahead at 0.6·r. 21 cells = look 4–10 × 30/60/144 Hz. **`fpStable: true` on all four matrices; 672 matrix runs; 0 NaN; 0 tunnelling frames.**

| seed | pMax 0.4 | pMax 0.6 | pMax 1.0 | air=hold 0.6 | HUMAN 0.6 | HUMAN 1.0 |
|---|---|---|---|---|---|---|
| 20260726 | 0/21 med **1234** max 2008 | 0/21 med **1237** max **2011** | 0/21 med 1206 max 1289 | 0/21 med 1153 | 0/12 med 110 | 0/12 med 110 |
| 777 | 0/21 med **731** | 0/21 med **732** | 0/21 med 728 | 0/21 med 731 | 0/12 med 171 max 728 | 0/12 med 170 |
| 12345 | 0/21 med **81** | 0/21 med **210** | 0/21 med 210 | 0/21 med 208 | 0/12 med 94 | 0/12 med 221 |
| 99999 | 0/21 med **345** | 0/21 med **352** | 0/21 med 346 | 0/21 med 353 | 0/12 med 125 | 0/12 med 184 |

**Totals: 0 finishes in 672 matrix runs (and 0 in ~2 000 runs overall from the top of the course).** Completion ≥ 80% is not met at any saturation on any seed.

**Sensitivity to saturation: essentially zero.** Medians move 2.6% (20260726), 0.5% (777), 0% (12345, 0.6→1.0), 2% (99999). The only real effect is at pMax 0.4 on 12345 (81 → 210), and it says *more* shift is better. **The claim that "pitch SATURATION decides completion" does not reproduce once the schedule is gated to the ground.** It should be struck from the settled list.

**Human-plausible arm** (0.30 s reaction lag, hip rate-limited to 1.5 /s, bar to 4 /s): 0/48, medians 94–221 m, off-tread 8–37%, mean lateral 0.50–1.21 m. That is the number that answers "is this a game", and it is far worse than the calibrated rider on every seed.

My controller is **weaker than the engineer's** (they read 9/252 where I read 0/672) and I say so plainly: my absolute completion figures are a lower bound. Everything comparative below is within-instrument.

### The segment map — the number that is actually useful

Restart at each of the 8 checkpoints, best of 9 cells. This separates "one wall" from "unrideable".

| seed | rideable segments | **WALLS** |
|---|---|---|
| **20260726** | 0→1254 ✓, 2138→**finish** ✓, 2427→finish ✓ | **~1270–1300** (road gap, design pace 5.7 vs window 11.1) · **~2011** (wallride, r 8.9 m, entry 6.2) |
| **777** | 1251→**finish** ✓ (and every later checkpoint) | **~732** (switchback r 5–6 m, 21–30° bank, 6.2 m/s) · **~1143** (loopout) |
| **12345** | 2125→**finish** ✓ | **~202–245** (the dead ground, unrideable at every schedule) · ~1114 · ~1286 · ~1885 |
| **99999** | 1549→**finish** ✓ (and every later checkpoint) | **~355** (r 6–21 m on 1.4–1.9 m of tread, roots, κ_v 0.378) · ~731 · ~1204–1349 · ~1950 |

Two useful corrections to the standing narrative fall out of this:

- **The 26 m of non-descending rock at arc 2110 does not stop the rider.** Started from arc 2006/2075/2129, my controller rides through it to the finish, 6/6 cells, at a mean 6.3 m/s. It is a CONTRACT §4 breach and a pace killer, not a dismount.
- **777 is not "one hard run" — it has two hard walls in its first half and a clean second half.**

---

## 3. Is it still a downhill course? Yes. It was not made rideable by being flattened.

ON vs the `feasible:false` control, same pre-carve mountain (`preFp` identical per pair):

| metric OFF → ON | 20260726 | 777 | 12345 | 99999 |
|---|---|---|---|---|
| 3-D length | 2731→2736 | 2693→2740 | 2696→2724 | 2698→2719 |
| drop (m) | 466→**458** | 454→**506** | 465→**499** | 480→**548** |
| mean grade | .175→.171 | .173→**.189** | .177→**.189** | .183→**.208** |
| **stations > 30%** | 11.1→**12.0%** | 14.8→**18.3%** | 13.7→**19.8%** | 11.5→**17.3%** |
| stations > 45% | 1.93→1.75% | 3.22→**1.69%** | 2.14→**6.07%** | 2.42→3.46% |
| grade p99 / max | .547/.79→**.495/.66** | .697/.99→**.476/.66** | .515/.87→**.714**/.85 | .549/1.28→**.625**/.81 |
| r min / p1 / p50 | 5.1/16.5/159→5.2/**11.0**/133 | 5.0/6.6/104→4.9/**17.9**/121 | 5.2/6.3/138→5.0/**16.1**/113 | 5.5/12.0/118→5.6/12.5/175 |
| cross-slope .35–.70 / >.70 | 136/25→**67/18** | 139/22→**35/0** | 115/43→**62/35** | 176/41→**34/0** |
| features / air features / cp | 28→30 / 8→8 / 8 | 36→32 / 6→9 / 8 | 34→35 / 9→9 / 8 | 41→27 / 9→6 / 8 |
| exposed m / unprotectable | 1141→1244 / 11→9 | 1299→**1739** / 0→**31** | 1198→**772** / 13→**0** | 1148→1082 / 8→**27** |
| `launchViolAfter` | 210→**26** | 1135→**142** | 388→**2411** | 511→**1259** |
| `routeAdmissible` | false → **true** | false → **true** | false → **true** | false → **true** |

**Not flattened.** All four seeds gain steep ground; three gain 34–68 m of drop; grade p99 falls on the two seeds that most needed it. Feature mix, 8 checkpoints and 6 splits intact on every seed.

Two regressions worth naming: **exposure on 777 (1299 → 1739 m, unprotectable 0 → 31)** and **99999 (unprotectable 8 → 27)**; and **the reference seed's corner p1 tightens 16.5 → 11.0 m**. Against the engineer's own report I do **not** see the reference seed's steepest berms "back": relative to my control it has *fewer* (136/25 → 67/18 stations above 0.35 / 0.70). Berms are, however, being delivered: the committed heightfield realises **89–93% of the designed cross-slope** on all four seeds, worst shortfall 0.27 at the wallride. Nothing is designed beyond what the corridor carries.

---

## 4. Regression — clean, bit-exact

| check | reference | measured |
|---|---|---|
| static sag fork / shock | 29.4% / 26.7% | **29.4% / 26.7%** |
| terminal v, 15% grade, `anyPressed=false` | 17.181 | **17.181** |
| ... `anyPressed=true` | 16.885 | **16.885** |
| step symmetry L/R @ 30 / 60 / 144 / 240 Hz | 0.000% | **±22.751°, identical to 3 dp at all four rates** |
| rear lockup | locks | **min spin/rolling 0.000, 204/300 skid frames** |
| 300 s soak, steering + shifting | 0 NaN | **0 NaN, 0 crashes** |
| 672-run sweep | — | **0 NaN, 0 tunnelling frames** |

**Determinism byte-identical across 15 concurrent cross-process boots** (3 per seed × 5 seeds), heightfield, materials, carve and stamps all matching:
`20260726 world=e22fa1eb carve=7f1e396f 56420 2729.7725` · `777 8432479d/9dddf5b5 62919 2733.6882` · `12345 6d852b7b/337595bd 51892 2711.7345` · `99999 28f68aff/ec060461 55619 2712.4621`.

**Seed 2 still refuses**, identically on 3/3 boots, with the full 16-variant diagnosis (climb 18.8–93.2 m against a 12 m gate). Correct and it must stay.

**Rider/HUD intact.** `rider.js:3427–3450` still carries the R9 arithmetic (`steep = smoothstep(0.10, 0.50, g)`, `+0.24·steep` un-commanded, clamp −1.35, `aft·0.22 + fwd·0.17`); `hud.js` still carries `BAL_LO = 0.16`, `BAL_FRONT_LO = 0.18`, `BAL_CUE_GRADE = 0.16`. My own live-load probe at 35% grade reproduces the direction the HUD calls: pitch 0 → front carries everything (rear fraction 0.000); pitch −0.6 and −1.0 → front unweighted (rear fraction 1.000). The HUD's front-light band is right, and it remains the only module in the project that knows full shift is over-shift on flat ground — which my `pMax 1.0` loopouts on 777's and 12345's sprint sections (13/21 in the `air=hold` arm; `loopout` at arc 2635 from every late checkpoint) independently confirm.

---

## 5. Adversarial

- **A saturation at which a previously-finishing seed stops finishing:** yes, and it is `pMax = 1.0`. From checkpoint 1251 on **seed 777**, `pMax 0.6` finishes (2693 m) and `pMax 1.0` loops out at 2635 — reproducibly, from all five later checkpoints. Conversely `pMax 0.6` cannot descend anything above ~0.70 gradient on smooth ground (0/9 at 55%) where `1.0` is 9/9. **A single saturation is wrong in both directions; the cap itself needs to be grade-scheduled.**
- **A seed where the ground-refusal term refuses everything:** seed 2 — 16/16 variants, and correctly so (climb gates, not `FEAS_W_GROUND`). The refusal term itself does the opposite: the file records that an absolute floor made **all five seeds** fail generation with 18–190 m of climb, which is why `FEAS_BUDLOW_AT_ZERO` ships at 0.
- **A berm beyond corridor carry:** none. Mean realisation 0.893 / 0.929 / 0.899 / 0.911 of design; worst shortfall 0.27 on the wallride (designed 1.27 cross-slope, carried 1.00).
- **Where the bike sticks:** nowhere it cannot grind through. Arc 2156–2246 on the reference seed ships 89.4 m without net descent, +1.59 m of climb, on ROCK, with the design speed at **1.00 m/s** — but a rider entering at 6 m/s carries through it at a mean 6.3 m/s. The 8.9 m-radius wallride at arc 2011 stops every cell that reaches it; that one is a wall.

---

## The answer to the question you asked

**FAIL.**

**What genuinely landed this round and should not be re-litigated:**
1. **Bug 3 is closed.** 32/32 air features solved, 0/32 over their own speed window, the round-10 exhibit at arc 447 now inside its window at ratio 0.82. Independently audited from the station table, not from the engineer's audit.
2. **Seed 99999's arc-154 wall is gone** (0.80–0.99 → 0.10–0.35 gradient), and dead ground goes to **zero** on two of four seeds.
3. **`routeAdmissible: true` on all four; seed 2 refuses; determinism byte-identical at 15 concurrent boots; bike regression bit-exact; the course is steeper, not gentler.**
4. **The `FEAS_W_GROUND` mechanism is correctly diagnosed and honestly reported**, including the re-pricing that was measured and rejected — that disclosure saved me a redraw and it should stay in the file.

**What fails, with numbers:**
1. **Bug 2 is not fixed, only re-measured.** `DESC_MAX_RUN` counts *consecutive* non-descending stations and a 10 µm dip resets it; rule 3 discharges on 4 mm of fall. The reference seed ships **89.4 m with +1.59 m of climb** (OFF: 79.8 m) at a 1.00 m/s design speed. Two seeds better, two worse. CONTRACT §4 says ~15 m; the march gate says 90 m.
2. **Seed 12345 arc 202 is unrideable, not merely hard.** 12.8 m at 0.82–0.84 gradient, r 60–90 m, at the 6.20 m/s floor, 7.64 m of cut. My analytic replica: **0/9 at every saturation, radius, bank and speed**, `washout`. It kills 11–19 of 21 cells at ~210 m.
3. **The jump line is the new binding wall, and it is bug 3 inverted.** 20/32 features paced below their own window minimum — `road gap` 5.71 m/s into an 11.1 m/s window. Every cell on the reference seed dies at 1206–1301 m, under every air policy and every pace multiplier I tried.
4. **Bug 4 is worse than reported.** Saturation is *not* the free parameter (2.6% of median); the **lookahead policy is (4.1×)**. No completion figure on this project bounds the trail. The acceptance metric must be fixed before another round is spent on it, and fixing it means publishing the *whole* controller — steering law, lookahead policy, governor, air gating and pitch schedule — not three scalars.

### Could a competent human with a gamepad ride this top to bottom?

**No, on all four seeds — but the reason is now specific and short, which it was not before.** Each seed has two to four *named* walls, and every seed's last third is clean:

- **777 — closest to shippable.** Two walls: the 5–6 m switchback at arc 732 (inside my certified envelope at the design speed, so this is the trail), and a loopout at 1143. From arc 1251 the remaining **1 442 m rides to the finish, every cell.** Fix those two and it is a game.
- **20260726 — two walls.** The road gap at 1270–1300 (a jump the design paces you into at half its takeoff speed) and the 8.9 m wallride at 2011. First 1 254 m clean; last 550 m clean. A rider does *not* get off and push at arc 2110 — that finding of round 10 does not survive; they grind it at 6 m/s.
- **99999 — four walls**, starting at arc 355 on 1.4 m of rooty tread at r 6–21 m with κ_v 0.378. Clean from 1549 m.
- **12345 — not shippable.** It fails at 202 m on ground the bike measurably cannot descend at any weight shift, and it has three more walls after it.

**Zero of four seeds is rideable top to bottom today. Two (777, 20260726) are two fixes away, and both fixes are named: pace the jump line at the speed its own features publish, and stop shipping sub-10 m switchbacks at the 6.2 m/s floor.** That is nearer than "one of four is rideable" suggested, and further than any completion percentage on this project has ever been able to tell you.