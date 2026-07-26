# VERDICT: **FAIL**

The controller pathology described in the original bug report is genuinely fixed *in isolation*. The acceptance criterion is not met, and by my own measurement it is not close. The engineer's headline number (730 m of pursuit progress) is reproducible but is a **fluke that survives neither a 10 cm change to the controller nor a change of frame rate**.

Harness (written from scratch, does not import the engineer's): `/private/tmp/descent-verify-steer.mjs` (+ world boot `/private/tmp/dv-world.mjs`, full output `/private/tmp/dv-final.txt`). Run as `node descent-verify-steer.mjs [step|repro|pursuit|fps|regress|diag|adverse|extra]`.

---

## 1. Step response — PASSES, better than claimed

25 speed × input combinations (5/15/30/50/60 km/h × |steer| 0.15/0.3/0.5/0.8/1.0), rolled to a steady speed on a constant grade, then a constant steer for 3 s at 240 Hz:

- **Asymmetry 0.0% on every single row.** The reported 2.7× left/right asymmetry is gone.
- **No sign reversal.** Worst yaw backtrack across all 50 runs: **0.3°**.
- Steady-state bar angle is small, smooth and monotonic in input (e.g. 30 km/h: 0.026 / 0.057 / 0.127 / 0.318 / 0.404 rad).
- Two crashes: 60 km/h at |steer| 0.8 and 1.0, both `washout`, both symmetric to the substep. Full lock at 60 km/h on smooth ground is an unrecoverable washout — predictable, but it is a crash under constant input.

**Residual:** a small persistent bar limit cycle. On a *perfectly smooth* plane the bar shows **2.4 reversals/s of ≥0.012 rad** with steady-window peak-to-peak of **0.014–0.041 rad** (0.8–2.4°). Nothing like the original ±0.28 rad square wave, but "no chatter" is not an accurate description.

## 2. Pure-pursuit down the full trail — **FAILS**

My controller was tuned by a 48-point grid search (gain, speed cap, air-steer, yaw damping), so the bike got every chance. Best result:

| | |
|---|---|
| reached | **t = 0.4846 — 1319 m of 2721 m (48.5%)** |
| time | 394 s, **never finished** |
| crashes | **20** (`lowside` 8, `impact` 5, `offaxis` 4, `washout` 2, `loopout` 1) |
| lateral error | mean 1.49 m, **max 27.0 m**; offTrail 14% of the run |
| wheel contact | both 54%, one 21%, **none 26%** |
| \|lean\| above the bike's *own* lean ceiling | **14.1% of the run**; peak 1.259 rad (lowside fires at 1.26) |

**The result is not reproducible.** Same controller, same trail, lookahead varied by ±0.5 m:

```
lookMin 6.00 m -> 18.6% (505 m)     6.50 m -> 48.5% (1319 m)
        6.25 m ->  2.7% ( 72 m)     6.60 m ->  3.4% (  93 m)
        6.40 m ->  3.5% ( 95 m)     6.75 m ->  2.4% (  64 m)
        7.00 m ->  2.6% ( 72 m)
```

A 10 cm change swings progress by **14×**. Typical progress is 60–100 m. 48.5% is the outlier, not the behaviour.

## 3. Frame-rate independence — **split**

- **Step response: identical to 0.00%** at 30/60/144 Hz (yaw −100.78° at all three; bar 0.1273/0.1272/0.1271). Better than the claimed 0.3%. The integrator is sound.
- **Game outcome: 263% spread.** Same controller, same trail: 61 m @30 Hz, 1319 m @60 Hz, 58 m @144 Hz. Chaotic amplification through the crash/respawn loop, but the practical consequence — the game plays materially differently at different frame rates — stands.

## 4. Regressions — **none found**

| test | measured | verdict |
|---|---|---|
| terminal velocity, 15% grade | **17.18 m/s** | PASS (want 12–20) |
| static sag | fork **49.9 mm / 29.4%**, shock **44.0 mm / 26.7%** | PASS |
| rear brake lockup | rear wheel locked **99.1%** of contact samples, min spin 0.00 rad/s, skid 1.00 | PASS |
| braking, both, 13.0→2.0 m/s | 1.31 s, 9.78 m, **8.37 m/s²** | PASS |
| pumping skill gap (phase swept over the whole cycle vs a no-pump baseline) | best phase **+2.92 m/s**, worst **−1.81 m/s**, gap **4.74 m/s** | PASS — the gap the engineer said he could not verify **does exist**, and is larger than the ~+0.8/−1.2 target |
| landing discrimination | matched 0.974, flat 0.997, uphill kicker **0.000** | PASS on `quality` |
| landing vN fidelity | 1–4 m step: true closing 3.99/5.95/7.38/8.61 → reported 4.02/5.98/7.42/8.63 | PASS |

## 5. Adversarial — all survive

Full lock 10 s (no crash, scrubs to 3.8 km/h); alternating full lock at 2 Hz for 10 s (0 crashes); full steer airborne then land mid-turn (q=0.871, no crash — the per-axis auto-stab deadzone works); both brakes locked + steer 0.6 (no crash); **+1 → −1 in one frame** (peak bar rate 1.13 rad/s, well under the 6.0 limit — no bang); straight into a wall (crash, **0.000 m penetration, no tunnelling**). **300 s of random input at mixed 20–240 Hz: 0 NaN across 27 published fields, 0 tunnelling.** Zero-input straight roll from 30 spawn points: **25/30 clean**.

---

## Residual defects, in severity order

1. **Acceptance criterion not met.** 48.5% of the trail at best, ~3% typically, 20 crashes, never finishes. *(§2)*

2. **The good result is luck, not a property of the bike.** 14× swing from a 10 cm retune; 22× swing from a frame-rate change. Any "it works now" claim based on a single pursuit run — including the engineer's 730 m — is unsupported. *(§2, §3)*

3. **The original repro still fails on the real trail.** From the gate, 3 s roll-in to 41 km/h, then constant steer:
   - `+0.5`: yaw at 0.5 s intervals = **3.4, 13.1, 12.0, 11.5, 3.4, −34.6°** — *the turn still reverses mid-corner*.
   - `−0.5`: crashes at 2.53 s. `+1.0`: crashes at 1.20 s. `−1.0`: crashes at 2.78 s.
   - `+0.15` → 39.5° vs `−0.15` → 65.5° = **1.66× asymmetry** (was 2.7×).
   
   **But the mechanism has changed.** Instrumented trace: the bike is **airborne for 1.7 of the 3 s**, during which `state.lean` is frozen (the air solver never integrates it) and the bar is frozen at 0.267 rad. The reversal is ballistic, not controller oscillation. The symptom persists; the diagnosed cause does not.

4. **Lean still escapes its ceiling.** 14.1% of the trail run is spent above the bike's own `leanLimit`+0.05, and on a 5 cm bump field lean runs to **1.45 rad — the hard clamp at bike.js:1673**, worse than the 1.24 rad the engineer says the effective-gravity fix cured. `lowside` is the single biggest crash cause (8 of 20).

5. **Roughness tolerance sits exactly on the trail's roughness.** Identical smooth S-bend, bump amplitude swept:

   ```
   amp 0.00 m: meanErr 0.45 m  leanMax 0.71  overCeil  0%  crashes 0
   amp 0.02 m: meanErr 0.48 m  leanMax 0.68  overCeil  1%  crashes 0
   amp 0.05 m: meanErr 2.96 m  leanMax 1.45  overCeil 23%  crashes 3
   amp 0.08 m: meanErr 6.73 m  leanMax 1.28  overCeil 42%  crashes 3
   ```
   The cliff is between **2 and 5 cm**. Measured tread roughness over a 0.8 m chord: 6.8 mm mean / 163 mm max longitudinal, 15.7 mm mean / 168 mm max across. The bike is being asked to ride terrain right at, and often past, the edge of its stability envelope. On perfectly smooth ground it tracks beautifully (0.21–0.97 m mean error over 21–81 m radii at 10–16 m/s, zero crashes, never above its ceiling) — so the engineer's "terrain interaction, not the model" is correct, but that is a description of the bug, not a defence.

6. **Contact loss is the underlying failure.** Railed on the centreline at a held speed, first 25% of the trail:
   ```
   hold  4 m/s: contact both 60% one 21% NONE 19%; crashes 2 in 47 m
   hold  8 m/s: contact both 44% one 21% NONE 35%; crashes 4 in 69 m
   hold 13 m/s: contact both 31% one 19% NONE 50%; crashes 6 in 72 m
   ```
   At **4 m/s** — walking pace — the bike has no wheel on the ground 19% of the time. Some of this is trail design (the first feature is `rollers`, 46–287 m, with sections at 72% grade), but not 19% at 4 m/s. While airborne no steering input reaches the bar, so every controller is open-loop for a quarter of the run.

7. **Steering is dead below ~2 m/s.** Full lock, held 5 s:
   ```
   0.7 m/s -> 0.0° yaw, radius INFINITE (does not turn at all)
   1.5 m/s -> 20.9° yaw, radius 20.9 m
   3.1 m/s -> 198° yaw, radius  4.5 m
   ```
   `speedFrac = smoothstep(1.0, 7.5, v)` and `LOWSPEED_UPRIGHT = 3.0` together make the bike unsteerable at exactly the speed a real bicycle steers best. This makes the 5 km/h row of the step test a null result (4.3° in 3 s at full lock) and makes any slow technical section or post-crash recovery impossible. Pre-existing design, not a regression from this fix — but it is a defect.

8. **`cased` is effectively unreachable.** Across 11 constructed landings (matched down-ramp, flat, uphill kicker, and 1–6 m vertical steps), `lastLanding.cased` was **false every time**. It needs `vN > 5.5` AND `quality < 0.88`; a flat drop scores quality 0.93–0.96 no matter how big, and above vN 9.0 the `impact` crash takes over first. Landing *quality* discriminates well (0.000 vs 0.997), so the mechanic is half-present. Contract §5 says "case a landing = harsh compression or crash"; a 4 m flat-landing drop is neither.

9. **Wall-crash event payload is wrong.** Riding into a 79° face crashes via `resolvePenetration`'s `slam` path, not `wallStep`'s `wall` path, and the emitted `run:crash` reports `speed = 0.60 m/s` for an impact that occurred at ~12 m/s — because `V` is cancelled against the constraint *before* `triggerCrash` is called (bike.js ~2077–2080). `severity` is correct; `speed` is not. postfx, audio and HUD consume that payload.

## What is still weak even where it passes

- Frame-rate independence is proven only for the *step response*. The claim does not extend to gameplay.
- The pumping gap is verified, but it is **~3.6× larger than the design target** in both directions (+2.9 / −1.8 against +0.8 / −1.2). Good timing may now be worth more than intended.
- Symmetry is exact on a plane and 1.66× off on the trail. The plane result proves the model is symmetric; it does not prove the game is.
- 25/30 clean on zero-input roll sounds good, but 13 of those 30 spawn points never exceed 2 m/s — the bike simply stalls in a roller trough, so those rows are not really testing anything.