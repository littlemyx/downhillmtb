# VERDICT: **FAIL** — and the sixth thing is that the instrument was never calibrated. The cause has RELOCATED, twice over.

I edited no file. No git, no npm, no browser, no network. Harness is mine, from scratch, in `/private/tmp/claude-501/-Users-stewartwebster-Projects/1bc47d7b-2580-443f-86fb-d67586b192a9/scratchpad/sk7/` (`synth.mjs`, `ctl.mjs`, `world.mjs`, `measure.mjs`, `feas.mjs`, `sweep.mjs`, `crawl.mjs`, `gmat.mjs`, `nobrake.mjs`, `bikereg.mjs`, `course.mjs`, `xsec.mjs`, `margin.mjs`, `trace.mjs`, `dump.mjs`, `det.mjs`). All `node --check` clean. Every A/B is in one process against one cached world; the world is fingerprinted (my own FNV-1a over a 193² lattice + 4001 tread points + every stamp field) before and after every measurement — **all stable, every time**.

---

## STEP 0 — THE HEADLINE. My controller finishes a clean course. The published one does not ride the tread on it.

**The synthetic course**, built to be trivially rideable and audited before use: 1206 m of 3-D arc, 120 m drop, **constant 10% grade**, serpentine heading `θ = 0.35·sin(2πs/140)` → **κ_max 0.01571 /m, r_min 63.7 m**; berms at 60% of the required lean so the worst residual contact lean at design speed is **5.19°**; **vertical curvature of the centreline identically zero** (v_launch = ∞, so `v²κ/g` = 0 everywhere); 3.0 m tread; ground rises 0.15 m/m for 8 m outside **both** edges — zero exposure. The surface is **analytic**, not a grid, so there are no bilinear facets and the only curvature in it is the curvature I put there. Driven with the real `src/physics/bike.js` and `src/physics/collision.js` through a terrain shim implementing the full sampling contract (normals by central difference on the analytic height).

| controller | finishes | mean lateral | max lateral | **fraction of the run OFF THE TREAD** |
|---|---|---|---|---|
| **mine** (`lean`), look 5–10 | **21/21** at 30/60/144 Hz | **0.10–0.44 m** | 0.35–1.30 m | **0.0%** |
| mine, look 4 (0.33 s at 12 m/s) | 3/3 | 0.78–1.05 | 1.67–2.40 | 2.1–24.9% |
| **the published controller** (`δ = atan(2·L_wb·sin α / L_d)` fed to `input.steer`) | 21/21 | **1.28–7.77 m** | **17.2 m** | **46.7–91.4%** |

The controller six rounds have been measured with **completes this course while riding up to 17 metres off a 3-metre trail**. It only "finishes" because I built the surroundings forgiving; put it on a 1.2–3.0 m tread with 12 m of fall-away and it is not on the trail at all.

**The mechanism, and it is a units error, not tuning.** `bike.js:1996` reads

```js
let target = clamp(steerIn * T.LEAN_MAX * speedFrac, -leanCmdCeil, leanCmdCeil);
```

`input.steer` is a **normalised lean demand** (×`LEAN_MAX` = 0.98 rad). Pure pursuit's δ is a **bar angle in radians**, ≈ `L_wb/r`, and carries **no speed term**. For r = 10 m it returns 0.125 rad → a commanded lean of **7.0°**, when the lean actually required at 10 m/s is **atan(v²/gr) = 45.6°**. A speed-independent **6.5× under-steer**, worst exactly where the corners are tightest. Converting the same pure-pursuit geometry to the units the input is in — `steer = atan(v²·κ_pp/g)/LEAN_MAX` — is the whole difference between the two rows above. *Caveat I will not paper over: I am inferring the previous rounds' implementation from the formula they published. I could not run their code.*

**What that does to six rounds of evidence** — same world, same route, one process, seed 20260726:

| | median | mean airborne fraction | **airborne at crash** | mean lateral | off-tread |
|---|---|---|---|---|---|
| published `delta` | 190.0 m | **0.247** | **17/21** | 5.78 m | 45.1% |
| calibrated `lean` | 311.1 m | **0.0105** | **2/21** | 0.38 m | 1.7% |

Across five seeds the airborne-at-crash count falls from 17/21, 12/21, 8/21, 1/6, 4/6 to **2/21, 1/21, 13/21, 3/6, 2/6**, and the mean airborne fraction from 0.105–0.247 to 0.010–0.159. **The "the trail launches the rider" thesis that rounds 4, 5, 6 and 7 were all built on is, on four of five seeds, an artefact of a rider who was 2–8 m off the tread riding the open mountainside.** It survives on seed 12345 only (airFrac 0.159, impact 10/21) — where I can name the launcher, below.

### Margins of my instrument (so you know what it can and cannot see)

Same synthetic course, one dial at a time, 12 cells each:

| dial | clean | degrading | fails |
|---|---|---|---|
| roughness, λ 8 m | ≤0.15 m amp (12/12) | 0.20 m (11/12, off-tread 72%) | **0.30 m → 0/12** |
| roughness, λ 3 m | ≤0.02 m amp | — | **0.05 m → 1/12** |
| curvature @12 m/s | r_min ≥20 m (12/12) | r_min 18 m (7/12) | **r_min 16 m → 2/12** |
| speed, baseline geometry | **20 m/s → 12/12** | — | — |
| exposure (fall-away outside tread) | −0.5 m/m (12/12) | −1.0 m/m (9/12) | — |
| tread width | **1.2 m tread → 12/12** | — | — |
| **grade** | **20% → 12/12** | **30% → 3/12** | **45% → 0/12** |

Exposure and narrow tread are *not* what breaks it. Grade is. Which leads to the second relocation.

---

## THE SECOND RELOCATION — the grade × radius × speed envelope, measured with no trail involved at all

On the **known-good** course (no roughness, no exposure, berms at 60% of required lean), varying only grade, design speed and corner radius:

| | v6 | v8 | v10 | v12 | v14 |
|---|---|---|---|---|---|
| **dead straight**, 15–30% grade | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| dead straight, 36% | 6/6 | 6/6 | 6/6 | 5/6 | 4/6 |
| dead straight, 45% | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| **r_min 64 m**, 15% | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| r_min 64 m, 20% | 6/6 | 6/6 | 6/6 | 6/6 | 5/6 |
| r_min 64 m, 25% | 6/6 | 6/6 | 6/6 | 6/6 | **0/6** |
| r_min 64 m, 30% | 6/6 | 6/6 | 6/6 | **2/6** | 0/6 |
| r_min 64 m, 36% | 6/6 | 6/6 | 6/6 | **0/6** | 0/6 |
| **r_min 64 m, 45%** | **0/6** | **0/6** | **0/6** | **0/6** | **0/6** |

A **64-metre-radius** corner — the gentlest turn on the course, needing 5.2° of contact lean — **cannot be taken on a 45% grade at 6 m/s**. Straight, the same 45% grade is fine at 14 m/s.

Isolating braking from grade (free-roll, brakes and pedal disabled, vs governed to 12 m/s):

```
grade 45% straight : braking-to-12 FINISH, leanLimit floor  4.8 deg | free-roll lowside @434 m, vMax 30.2
grade 45% r64      : braking-to-12 lowside @61 m            7.9 deg | free-roll lowside @ 73 m, vMax 17.3
grade 36% r64      : braking-to-12 lowside @131 m          17.3 deg | free-roll lowside @ 65 m
```

**The lean ceiling on a 45% grade under speed-holding braking is 4.8–7.9°.** Straight, that is enough. Add any corner and it is not. This is a *joint* property of grade, speed and radius that neither module has ever measured — trail.js prices roughness (`techSpeedCap`), the launch curvature (`capLaunchCurvature`, `computeVertKappa`) and the cornering identity (`rebankAtSolvedSpeed`, `cornerGripBudget`), but nothing prices **the longitudinal share of the friction circle**: the tyre must simultaneously hold the grade, deliver the deceleration the speed profile itself demands, *and* corner.

---

## 1. The controller-free wheel test — stencil 0.37 m, reference the committed centreline, sampling 0.05 m plan-arc, at the governed speed

Plan path is a **centripetal Catmull-Rom through the station plan points, arc-length resampled at 0.05 m** (a chord polyline injects its own ripple); wheel-centre locus is the morphological dilation of the profile by a 0.37 m disc; κ from a second difference at half-stencil **0.37 m**, linearly interpolated (not snapped to the grid); flag when `v²·|κ|/g ≥ 1` with κ convex; v = `trail.speedAt(t)`. Bins of 25 m, bins with <200 samples dropped.

**Committed centreline:**

| seed | course mean | **worst 25 m bin** | bins >20% | bins >10% | p50 / p90 / p99 of bins | top bins |
|---|---|---|---|---|---|---|
| 20260726 | 0.87% | **12.0% @1100 m** | **0** | 3 | 0 / 2.4 / 11.4 | 1100:12 1325:12 300:11 425:9 |
| 777 | 0.92% | **17.4% @200 m** | **0** | 2 | 0 / 2.2 / 9.6 | 200:17 175:11 2475:10 |
| 12345 | 1.19% | **14.4% @2400 m** | **0** | 3 | 0 / 5.8 / 11.2 | 2400:14 1225:13 1075:11 |
| 2 | 1.37% | **23.2% @775 m** | **1** | 4 | 0 / 5.6 / 10.8 | 775:23 1350:12 1300:11 |
| 99999 | 0.95% | **13.0% @1225 m** | **0** | 4 | 0 / 3.4 / 11.8 | 1225:13 250:12 1075:12 |

**Design controls, identical stencil / sampling / speed:**

| seed | design = emitted station centreline `S.py` | design = `terrain.sampleDesign()` |
|---|---|---|
| 20260726 | 1.20% course, worst **19.0% @300** | 17.59% course, worst **49.6%**, 44 bins >20% |
| 777 | 2.17%, worst 18.4% @175 | 16.95%, worst 51.6%, 38 bins >20% |
| 12345 | 2.41%, worst 23.6% @2400 | 19.84%, worst 48.8%, 49 bins >20% |
| 2 | 2.15%, worst 26.2% @1400 | 17.41%, worst 49.4%, 45 bins >20% |
| 99999 | 1.61%, worst 19.2% @250 | 20.13%, worst 47.2%, 53 bins >20% |

**GATE: worst 25 m bin ≤10% — FAIL on all five seeds (12.0–23.2%). "No bin above 20%" — PASS on four, FAIL on seed 2 (23.2% at arc 775).**

I **independently reproduce the trail engineer's headline numbers to the decimal** (0.86/12.0 @1100/0 over-20/3 over-10 vs my 0.87/12.0 @1100/0/3), from my own dilation, my own plan smoothing and my own bin code. That claim is honest. The reference-dependence they flagged is also real and large: the same quantity on the same trail is **1.2% or 17.6%** depending on which "design" you pick — a **15×** swing. This metric should never be quoted without its stencil, reference and sampling, and my figures above carry all three.

---

## 2. The autopilot sweep — 0 finishes, 96 runs, five seeds, calibrated controller

Governor: design speed with a 40 m feasible-braking lookahead and a skid-derated brake (`gov:'brake'`, μ 0.45); steering = calibrated pure pursuit; no per-cell tuning. Stuck = 8 s without progress.

**seed 20260726** (2728.2 m) — distance · cause (`/air` = airborne at first crash):

| look \ Hz | 30 | 60 | 144 |
|---|---|---|---|
| 4 | 356 impact/air | 311 offaxis/air | 188 offaxis/air |
| 5 | 189 impact/air | 278 lowside | 279 lowside |
| 6 | 312 stuck | 326 offaxis/air | 184 slam/air |
| 7 | 323 loopout | 328 lowside | 312 loopout |
| 8 | 311 lowside | 274 lowside | 310 lowside |
| 9 | 276 lowside | 283 offaxis/air | 311 lowside |
| 10 | 311 lowside | 312 loopout | 312 loopout |

**0/21 · median 311.1 · min 184.2 · max 355.6 · spread 1.93× ·** lowside 9, loopout 4, offaxis 4, impact 2, stuck 1, slam 1 · airborne at crash **7/21** · airFrac **0.039** · **mean lateral 0.38 m, off-tread 6.1%**.

| seed | finishes | median | min | max | spread | causes | airFrac | air@crash | latMed | off-tread |
|---|---|---|---|---|---|---|---|---|---|---|
| 20260726 | **0/21** | 311.1 | 184.2 | 355.6 | 1.93× | lowside 9, loopout 4, offaxis 4, impact 2, stuck 1, slam 1 | 0.039 | 7/21 | 0.38 m | 6.1% |
| 777 | **0/21** | 94.2 | 46.5 | 193.7 | 4.17× | **lowside 13, otb 6**, offaxis 1, stuck 1 | **0.0104** | **1/21** | **0.27 m** | **1.8%** |
| 12345 | **0/21** | 137.5 | 83.0 | 404.4 | 4.87× | **impact 10**, lowside 5, washout 3, offaxis 3 | **0.159** | **13/21** | 1.25 m | 22.3% |
| 2 (6 cells) | **0/6** | 85.2 | 80.1 | 316.0 | 3.95× | offaxis 3, lowside 1, stuck 2 | 0.051 | 3/6 | 0.60 m | 18.8% |
| 99999 (6 cells) | **0/6** | 114.6 | 51.6 | 158.1 | 3.06× | lowside 4, offaxis 2 | 0.022 | 2/6 | 1.13 m | 38.7% |

0 NaN, 0 tunnelling in 96 runs. Fingerprint identical before and after every matrix.

**Seed 777 is the cleanest evidence in this report.** The bike holds the centreline to **0.27 m mean, 1.8% off-tread** — better than my known-good synthetic at look 4 — and **falls over at 46–95 m**. It is exactly where the design says to be, at the design speed, on the design line, and it low-sides. That is not a controller bound.

**Speed ablation — the impassable points.** Governor capped at a constant, three lookaheads each:

```
20260726: cap 12 -> 292-336 | 8 -> 311-352 | 6 -> 310-317 | 5 -> 311-488 | 4 -> 310-316 | 3 -> 320-325 | 2.5 -> 97-328
777     : cap 12 -> 93-196  | 8 -> 94-206  | 6 -> 96-193  | 5 -> 101-201 | 4 -> 192-208 | 3 -> 94-110  | 2.5 -> 94-99
12345   : cap 12 -> 117-138 | 8 -> 81-88   | 6 -> 89-421  | 5 -> 84-138  | 4 -> 138-320 | 3 -> 78-138  | 2.5 -> 84-138
```

**Each seed has a location it cannot pass at any speed from 2.5 to 12 m/s.** Not "it is going too fast".

**And the round's own remedy does not help.** I recomputed the joint feasibility envelope offline (below) and governed to it at 90%: seed 20260726 goes 0/21, median **311.0** against 311.1 — identical, with `loopout` rising to 9/21. Slowing down is not the fix, which is the same conclusion the brief reached from the other direction.

---

## 3. The two constraints jointly, against the COMMITTED surface — plus the third one nobody solved

Per station (0.4 m), committed heights, committed cross-slope by secants at ±0.36·width, plan radius by circumradius on a **±4 m** stencil, κ_v at the 0.37 m acceptance stencil on the dilated committed profile, v = `trail.speedAt`.

| seed | **C1** `v ≤ √(g/κ)` violations | worst ratio | **C2** `atan(v²/gr) ≤ φ_committed + 28°` | worst | **C3** friction circle `√(μ_long² + μ_lat²) ≤ μ` at the trail's own μ = tan 28° = 0.532 | at the bike's μ = 0.994 | worst μ needed |
|---|---|---|---|---|---|---|---|
| 20260726 | 78 / 6600 = **1.18%** | 1.86× | 3 = **0.05%** | 7.5° | **270 = 4.09%** | 23 = 0.35% | **1.351** |
| 777 | 89 = **1.44%** | 1.84× | 23 = **0.37%** | 6.4° | **523 = 8.43%** | 44 = 0.71% | **1.897** |
| 12345 | 105 = **1.63%** | 1.85× | 38 = **0.59%** | 12.0° | **401 = 6.22%** | 31 = 0.48% | **1.210** |
| 2 | 123 = **2.05%** | **4.32×** | 9 = **0.15%** | 5.9° | **392 = 6.54%** | 33 = 0.55% | **2.164** |
| 99999 | 83 = **1.27%** | 1.92× | 7 = **0.11%** | 3.0° | **454 = 6.92%** | 36 = 0.55% | **1.178** |

μ_long = `|g·sinθ − dv/dt| / (g·cosθ)` — the grade **and** the deceleration the design speed profile itself demands. μ_lat = `|v²/(gr) − tanφ| / (1 + tanφ·v²/(gr))`, the exact banked-turn requirement.

**C1 and C2 are essentially closed.** The round did what it said: launch violations are down to 1.2–2.1% at ratio ≤1.9, and the corner identity against the *committed* bank holds at 99.4–99.9% of stations. **C3 was never solved and is violated 4.1–8.4% of the time against the trail's own design friction**, peaking at μ = 1.18–2.16 where 0.53 is available. Decomposition on 20260726: `tan(θ) > μ_design` alone at 1.27% of stations, but `μ_long > μ_design` at **3.88%** — so **the dominant term is the deceleration the speed profile demands, not the grade**. p50/p90/p99/max of μ_long = 0.132 / 0.358 / 0.799 / **1.308**.

**Feasibility envelope**, computed backward from the corner and launch caps with the deceleration the slope actually allows (`a = g(μ_long,avail·cosθ − sinθ)`):

- At the **bike's** μ = 0.994 the trail is geometrically feasible everywhere (v_feasible min 3.29–3.42 m/s) — but **`trail.speedAt` exceeds that envelope at 4.30% / 8.10% / 5.58% of stations (20260726 / 777 / 12345), by up to 1.87×.**
- At the **trail's own** design μ = 0.532: **84 stations on 20260726 (1.27%) cannot hold the grade at any speed** (tanθ ≥ μ), v_feasible collapses to **0.1 m/s at arc 275–300**, and the design speed exceeds the envelope at 15.17% of stations.

**The worst friction bins land exactly on the crash sites.** 20260726: 275 m (μ 1.35, 48% of stations violating), 300 m (1.23, 40%) — my 21 cells die at 274–356 m. 777: **175 m (μ 1.90, 55%), 200 m (0.99, 73%)** and arc 36–44 where μ_long runs 0.409 → 0.580 → **0.721** — my cells die at 46–95 m and 193 m. 12345: 2275 m (1.21), 100 m (1.08), 175 m (1.04).

**Traced at the crash, 20260726 look 8 @60 Hz** (this is what the failure actually looks like):

```
 arc     lat     v   vDes    lean  leanLim  skid  grade%   bankComm
288.7   0.30  12.3  11.5   -21.6    22.5  0.19     -29        1.2
291.5   0.57  11.5   9.8     4.1    19.5  0.95     -35        1.1
294.2   0.76  11.0  10.2    24.2    22.5  0.81     -36        2.0
296.6   0.81   8.8  10.9    40.4    29.5  0.99     -25        4.3
                                       -> lowside at 298.5 m
```

Lateral error **0.81 m** — dead on the line. The speed profile drops 12.5 → 9.8 m/s in 5 m on a 35% grade, the tyre saturates longitudinally (skid 0.95–0.99), the lean ceiling collapses to 19.5°, and the bike goes down. Nothing is airborne.

---

## 4. Exposure

| seed | fall-away 5 m outside the tread edge >3 m | >6 m | max | `safety.unprotectable` | exposed m | `safety.legAfter` |
|---|---|---|---|---|---|---|
| 20260726 | 8.6% | 2.8% | **12.72 m** | 5 | 1132 | 0 |
| 777 | 13.2% | 1.7% | 7.98 m | **0** | 1290 | 0 |
| 12345 | 6.0% | 0.8% | 12.34 m | 6 | 743 | 0 |
| **2** | 2.9% | 0.1% | 6.65 m | **0** | 717 | **count 6, 60 m, unresolvedStations 129, worstHeightGap 7.85 m, worstPlanGap 4.75 m, at arc 789.2** |
| 99999 | 11.2% | 1.0% | 8.08 m | 6 | 1133 | 0 |

Exposure has genuinely improved and my figures reproduce the trail engineer's `safety` counts exactly. **Seed 2 is not fixed and is not hidden** — the audit reports it honestly, and my own independent measures agree it is a broken route: **min plan radius 2.0 m** (a 1.25 m-wheelbase bike cannot turn that), **max climb 10.59 m**, **longest non-descending stretch 106.8 m**, min tread pinned at the 1.20 m floor, one wheel-test bin at **23.2% at arc 775** — the same arc as its leg conflict. Its macro-shape fidelity above 6 m is also **5× worse than every other seed** (0.160 vs 0.031–0.036).

My margin sweep says exposure is *not* what is killing the runs anyway: my controller finishes 12/12 with the ground falling away at 0.5 m/m outside the tread and 9/12 at 1.0 m/m.

---

## 5. Over-smoothing — nothing was flattened to pass

| | 20260726 | 777 | 12345 | 2 | 99999 |
|---|---|---|---|---|---|
| 3-D arc / drop (committed) | 2730.8 / 466.3 | 2689.7 / 454.2 | 2683.2 / 454.8 | 2733.0 / 355.3 | 2694.9 / 479.9 |
| mean grade | 17.08% | 16.89% | 16.95% | 13.00% | 17.81% |
| grade p50 / p95 / p99 / max | 17.1 / 36.3 / 55.8 / **75.7** | 16.5 / 39.5 / 69.8 / **95.2** | 17.3 / 38.6 / 52.1 / **84.3** | 13.5 / 39.4 / 61.8 / **128.0** | 17.8 / 38.2 / 56.4 / **122.0** |
| radius min / p1 / p50 (±4 m stencil) | **5.1** / 16.5 / 157.6 | **5.0** / 6.5 / 103.3 | **5.4** / 7.9 / 106.6 | **2.0** / 5.5 / 92.9 | **5.4** / 12.0 / 117.4 |
| width min / p1 / p50 | 1.30 / 1.42 / 2.73 | 1.22 / 1.34 / 2.80 | 1.22 / 1.30 / 2.71 | 1.20 / 1.20 / 2.66 | 1.26 / 1.30 / 2.80 |
| features / splits / checkpoints | 28 / 6 / 8 | 35 / 6 / 8 | 38 / 6 / 8 | 35 / 6 / 8 | 40 / 6 / 8 |
| **committed-vs-design macro shape >6 m** (slope-rms ratio, 0 = identical) | **0.0339** | 0.0312 | 0.0362 | **0.1596** | 0.0331 |

Macro shape above 6 m is preserved to 3% on four seeds. Jumps are intact: 20260726 tabletop 11.7 m/s · 0.92 m @19° · land 11.0 · gap 6.9 · air 1.06 s · window 5.7–15.6; road gap 11.8 · 1.06 @30° · 14.8 · 11.5 · 1.50 · 11.1–15.5; step-down 11.66 · 0.72 @27° · 16.0 · 13.9 · 1.60 · 11.2–17.3; finish booter 12.0 · 0.70 @20°. **It was not flattened.** It was made steeper and tighter than the bike can corner on.

**Two figures I do not reproduce and both are stencil artefacts, stated so nobody re-litigates them.** Min plan radius: **4.9 m at a ±2 m stencil, 5.1 at ±4, 5.7 at ±8, 6.9 at ±12**, while the trail's own `S.radius` array reports **7.5**. The engineer's "min radius 7.5 m" is the trail's internal smoothed measure; the emitted plan polyline carries 4.9–5.1 m corners. Max grade: I get 75.7–128.0% on committed heights at a ±4 m stencil where they report 89–91%. Neither is a defect claim — it is the same specify-your-stencil lesson, applied to two quantities nobody has been specifying.

---

## 6. Regression — the bike is untouched and every number is exact

Measured on my analytic synthetic ground, isolated from the world:

| metric | reference | measured |
|---|---|---|
| static sag fork / shock | 29.4% / 26.7% | **29.4% / 26.7%** |
| terminal velocity, 15% grade | 17.18 m/s | **17.181 m/s** |
| rear lockup | locks | **locks** (min spin/rolling 0.000, 776 skid frames, stops) |
| step symmetry L/R @30/60/144/240 Hz | 0.0% | **0.000% / 0.000% / 0.000% / 0.000%** (∓16.5–16.7°) |
| frame-rate divergence vs 240 Hz over 30 s | — | **−0.021% / +0.064% / +0.014%** |
| 300 s randomised soak | 0 NaN, 0 reversals/s | **0 NaN, 0.000/s** |
| across 96 autopilot runs | — | **0 NaN, 0 tunnelling** |

Round 6 could not reproduce terminal velocity (16.885 vs 17.18) and honestly attributed it to their synthetic plane. My analytic plane reproduces it to **17.181**. That closes their open item.

**Determinism — clean.** 3 cross-process boots per seed, 3 seeds, concurrently, with four spinners running (load average 2.6 rising through the run), byte-identical every time:

```
20260726: grid=79c9e2dd tread=d36659e7 stamps=54981/4ba5dca7 len=2728.2105 drop=466.2994
777     : grid=2f809501 tread=61f331d9 stamps=56973/343f3587 len=2687.3259 drop=454.2319
12345   : grid=1b0ca4b0 tread=075ca4ed stamps=49619/cf94a8bb len=2681.1056 drop=454.8274
```

Stamp counts and lengths match the trail engineer's own determinism line exactly (54981 / 2728.2105 / 466.2994). Pre- and post-carve fingerprints identical across every A/B pair in this report.

---

## 7. Adversarial

- **A seed where the coupled solve finds no feasible (v, bank), and what it does then: seed 2.** It ships the conflict rather than failing: `legAfter = {count 6, 60 m, 129 unresolved stations, worstHeightGap 7.85 m over a 4.75 m plan gap at arc 789.2}`, a **2.0 m** minimum plan radius, 10.59 m of climb, 106.8 m of non-descending trail, and the only >20% wheel-test bin in the whole report (23.2% at arc 775 — the same place). Its committed surface also loses 5× more macro shape than any other seed. Autopilot 0/6, median 85 m.
- **A berm that became a wall — found, and it is the seed-20260726 wall.** Cross-sections at 0.5 m lateral steps, arc 296–325 (my `xsec.mjs`):

```
arc 296.2  width 2.97 | ... right edge +0.4 +0.4 -> +3.1 +7.6 at 3.5-4.0 m out
arc 307.0  width 2.04 | left +1.9 at 2 m, +4.9 at 3 m, +7.9 at 6 m ; right +3.3 at 4 m
arc 311.8  width 1.63 | left +2.2 at 2 m, +5.6 at 4 m, +10.5 at 6 m ; right -1.0 at 4 m, -3.9 at 6 m
arc 320.1  width 1.50 | left +2.4 at 2 m, +7.0 at 4 m, +13.7 at 6 m ; right -1.8 at 5 m, -3.5 at 6 m
```

  A **1.48–2.0 m ledge** on a **51–69% grade** with **5.0–6.4 m plan radius**, **4–6° of committed bank**, a **13.7 m cut bank** 6 m to the inside and a **3.5 m drop** 6 m to the outside. Every one of my 21 cells dies inside 274–356 m and the crawl test cannot pass it at 2.5 m/s. The round-7 gate (`SB_MAX_FALL`) moved this switchback; it did not remove it, and their own report says so.
- **A roller flattened into a fire road:** no. Macro content above 6 m is preserved to 3.1–3.6% on four seeds and jump ballistics are intact.
- **A launcher that survives at walking pace — seed 12345, arc 125–140.** κ_v at the acceptance stencil runs **−0.367, +0.255, −0.238** over 4 m on a 44–51% grade with the radius collapsing 464 → 62 → 31 → 23.5 m. κ = 0.367 /m launches at **5.2 m/s**; the design speed there is **6.2 m/s**. That is the one seed where the launch thesis survives calibration (impact 10/21, airFrac 0.159), and my crawl confirms it: 12345 fails at 78–142 m at **every** cap down to 2.5 m/s.
- **Anywhere the bike sticks:** 4 stuck cells in 96 (20260726 look 6@30, 12345 crawl at cap 6, seed 2 look 8). My 0.55 pedal cap contributes and I cannot cleanly separate it.

---

## The answer to the question you asked

**FAIL. The cause has RELOCATED, and the relocation is in two parts.**

**(a) It relocated backwards, into the measuring instrument.** The autopilot used for rounds 4–7 feeds a bar angle in radians into an input that is a normalised lean demand. It under-steers by ~6.5× in a 10 m corner, it is speed-blind, and on a course with *no defects at all* it spends 47–91% of the run off the tread. Four of the five seeds' "airborne at crash" statistics collapse from 8–17 of 21 to 1–3 of 21 once that is fixed. **The launch thesis that three rounds of carve work, one round of band sweeps and one anisotropic filter were built on was, on four seeds of five, a measurement of the open mountainside.** The engineering done on that thesis was not wasted — C1 really is closed now, 1.2–2.1% at ratio ≤1.9 — but it was never the binding constraint it was believed to be.

**(b) With a calibrated instrument, the real cause is the constraint neither module has ever written down.** Not launch (closed). Not the corner identity against the committed bank (closed, 99.4–99.9%). It is the **longitudinal leg of the friction circle**: the tyre must hold the grade, deliver the deceleration `speedAt()` demands, and corner, all from one μ. Measured against the trail's own design friction it is violated at **4.1–8.4% of stations on every seed, peaking at μ = 1.18–2.16 where 0.53 is available**, and the design speed profile exceeds the joint feasible envelope at **4.3–8.1% of stations by up to 1.87×**. And measured on a course with no trail in it at all, the bike's cornering envelope closes at **~30% grade above 10–12 m/s and vanishes entirely at 45% grade at any speed** — while the trail ships p95 grades of 36–39% on all five seeds, p99 of 52–70%, and puts its 5–25 m corners exactly there.

**This needs a rearchitecture, not another round.** Two reasons, and the second is the decisive one. First, the pacing fix cannot work: I governed to the exact feasibility envelope at 90% and seed 20260726's median moved 311.1 → 311.0 m; and no speed from 2.5 to 12 m/s passes the wall on any of three seeds. Second, and structurally: `trail.js` is solving `v ≤ √(g/κ)` and `atan(v²/gr) ≤ φ + lean` on a route that `marchRoute` has *already committed to* — and that route puts 5 m corners on 69% grades on 1.5 m ledges. **The route-finding is scored on drop, exposure and corridor protectability; it is not scored on whether a bicycle can corner on the grade it selects.** Until `(grade, radius, speed)` feasibility is a *cost on the march* rather than a repair applied afterwards, each round will keep finding a new downstream constraint to close while the route keeps choosing ground that no downstream pass can rescue.

**Three concrete things to carry forward.** (1) The autopilot must be recalibrated against a known-good course before any further completion figure is quoted — mine is 21/21 with 0.10–0.44 m of tracking error and its failure margins are documented above. (2) The acceptance gate should be the joint envelope, not the wheel test: the wheel test now passes at 0.87–1.37% course mean and fails only at 12.0–23.2% in the worst bin, while the friction circle is over budget 4–8% of the time and nobody was looking. (3) **Seed 2 must not ship**: 2.0 m plan radius, 10.6 m of climb, 106.8 m non-descending, 129 unresolved leg-conflict stations, and the only >20% wheel-test bin in this report — all at the same arc.

**Weakest parts of my own evidence, stated plainly.** I inferred the previous rounds' controller from the formula they published; I could not run their code, so "the instrument was miscalibrated" is a strong inference from an exact reproduction of the symptom, not a diff. My controller's own margin is finite and I have mapped it: it fails at 0.05 m of 3 m-wavelength roughness, at r_min 16 m, and — most relevantly — at 30% sustained grade above 12 m/s, which is inside the range the real trail occupies, so on the steepest ground my 0/96 is a joint bound on trail and controller even after calibration. And I did not separate whether the 45%-grade cornering collapse is intended physics in `bike.js` or a defect in its lean-ceiling machinery; the brief forbids me to touch that file and I did not, but somebody should answer it, because it is now the number that decides what grade `trail.js` is allowed to author.