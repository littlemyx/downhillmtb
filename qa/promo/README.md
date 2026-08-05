# Promo capture pipeline

In-engine footage (the clips in `qa/video/`) is recorded straight out of the running game
by an injectable autopilot + capture bot: [bot.js](bot.js). No screen recorder, no OBS —
the bot ticks the game loop by hand, drives the bike down proven segments, renders at a
pinned resolution and POSTs every frame to the dev server, which writes it to `qa/shots/`.
ffmpeg then assembles the frames into video.

Everything below was established in an interactive session and verified in battle; the
non-obvious facts are called out because most of them fail silently if violated.

## How to run

```
npm run dev            # qaCapture middleware in vite.config.js serves POST /qa/shot
```

Open the game (a hidden browser pane is fine — see "Why manual ticking") and wait for
boot to finish: `window.__DESCENT__?.postfx` is populated by the last boot wave. Do
**not** wait for `document.body.dataset.descentReady` — it is set inside the rAF tick
loop, which never runs in a hidden pane, so it never flips there. Then in the page
console:

```js
const bot = await import('/qa/promo/bot.js');
await bot.runAll();                                  // all four proven segments
await bot.recordSegment(bot.SEGMENTS[1], { slow: 4 });     // one segment, honest 4x slow-mo
await bot.recordTake({ t: 0.29, name: 'test', frames: 60, w: 1080, h: 1920 }); // vertical
```

Frames land in `qa/shots/` as `<name>_0000.png … <name>_NNNN.png`.

**The files contain JPEG despite the `.png` extension.** The bot captures with
`canvas.toDataURL('image/jpeg')` (a 1080p PNG readback+POST per frame is 5–10× slower),
but the middleware appends `.png` to any name that lacks it. Rename before ffmpeg:

```bash
cd qa/shots
for f in seg-berms_*.png; do mv "$f" "${f%.png}.jpg"; done
ffmpeg -framerate 30 -i seg-berms_%04d.jpg -c:v libx264 -pix_fmt yuv420p -crf 18 \
       ../video/berms.mp4
```

## Why manual ticking

In a hidden/not-composited browser pane **rAF is frozen but promises and `fetch` still
run**. The game's own loop (`tick()` in `src/main.js`) never fires, so the bot advances
the simulation itself: one `update()` + `lateUpdate()` pass over the module order, then
`postfx.render()`, then the canvas readback — one physics tick per captured frame,
wall-clock independent, perfectly repeatable.

The tick order is `ORDER` from `src/main.js` minus `hud`/`menu` (DOM overlays — invisible
to `toDataURL`) and `audio` (AudioContext is suspended in a hidden pane). `chaseCamera`
stays in the order but is frozen the `qa/harness.js` `freezeCamera()` way (`update` and
`lateUpdate` replaced with no-ops) because the bot drives its own camera.

The autopilot writes `input.state` **immediately after `input.update()`** inside the tick —
`input.update()` rate-filters the state toward the (unpressed) keyboard every tick, so
writes made anywhere else get low-passed back to zero.

## The autopilot

Pursuit steering + a predictive speed governor. All conventions verified against
`src/game/input.js` and the CONTRACT-NOTEs in `src/physics/bike.js`:

- **Steering** — pure pursuit toward `trail.sampleAt(t + lookahead)`, lookahead
  ≈ `speed * 0.7` m (clamped 4–18). Steer = signed XZ angle from chassis forward to the
  target; `input.steer = +1` is right.
- **Speed limit from XZ curvature only.** The tangent's vertical component must be
  **excluded** before measuring heading change — with it included, every roller and grade
  break reads as a corner and the bot brakes on straights. Banked corners get the standard
  bonus: `v² = g·tan(bank + lean_margin)/κ`. The governor scans ~45 m ahead and relaxes
  the limit by braking distance (`v_allowed² = v_lim² + 2·a·d`) so it brakes early, not at
  the apex.
- **Front brake fades out with gradient.** Full front brake on a steep pitch is an instant
  endo. Weight shifts back while braking (`input.pitch = -1` is weight back / nose up —
  the CONTRACT-NOTE convention in bike.js).
- Per-segment speed cap 12–13.5 m/s and entry speed 9–10 m/s are what the proven segments
  were validated at.

## Teleporting the bike

`bike.state` is read-only (silently reverted at the end of the next `update()` — see the
CONTRACT-NOTE at the top of `src/physics/bike.js`). The working sequence:

1. `bike.reset({ t })` — **not** for the transform (reset snaps the height itself,
   reduces the quaternion to yaw and zeroes velocity, so passing a full transform through
   it is unreliable) but because it clears every controller filter/integrator and seeds
   `state.trailT`, the `nearestT()` search hint. A stale hint can snap the bike to the
   wrong leg of a switchback.
2. `bike.setPosition(p)` with `p.y = terrain.sampleHeight(x, z) + wheelRadius + 0.06` —
   `state.position` is the chassis origin at the **axle midpoint at axle height**
   (`bike.geometry.chassisOrigin`).
3. `bike.setOrientation(q)` — yaw-only, facing down the trail tangent.
4. `bike.setVelocity(tangent * v0)`.

`trail.sampleAt()` returns a **reused** object — clone `position`/`tangent` before the
next call.

## The countdown trap (critical)

After `gameplay.restart()`, the warm-up must tick **only `['gameplay']`** until
`gameplay.state === 'running'` (countdown is 3×0.85 s + 0.28 s GO-hold ≈ 2.83 s ≈ 180
ticks at 1/60). If the full order is ticked, the bot rides from the gate — and the start
descent is unrideable, so the bike crashes, gameplay arms its **delayed** respawn
(`CRASH_BEAT` = 1.25 s), and that pending respawn later fires mid-take and destroys the
next teleport. This one failure mode cost more debugging time than everything else
combined.

## Capture

- Governor off (`engine.setGovernorEnabled(false)`), then **pin every tick**:
  `setPixelRatio(1)`, `setSize(w, h, false)`, `camera.aspect`, `postfx.resize(w, h)`.
  The engine's pixel-ratio governor and the `main.js` resize handler both drag the
  backbuffer to 1×1 when the hidden pane reports 0×0 — pinning once is not enough.
- **Slow-mo is honest**, never frame duplication: tick physics at `dt = 1/120`, capture
  every tick, play at 30 fps → true 4×. Real time: `dt = 1/60`, capture every 2nd tick.
- Bot camera: position behind the bike along the **velocity** direction (chassis forward
  at low speed), `lerp 0.14` smoothing, clamped above `terrain.sampleHeight`.

## Takes and retries

Acceptance criterion: **zero crashes inside the recorded window** — a crash aborts the
take immediately. Between attempts the bot does a full `restart()` (which is what clears
gameplay's pending crash/respawn state). A failed attempt's partial frames are simply
overwritten by the succeeding attempt — same names, no cleanup needed.

## Proven segments

The trail start (t 0.05–0.19) is **unrideable for the bot** — the corner-berming pass
runs in only 3 of 8 corner phases (known issue, project README), and unbermed
switchbacks are beyond the autopilot. Verified-clean windows:

| name       | t    | character            | cap (m/s) | v0 (m/s) |
|------------|------|----------------------|-----------|----------|
| seg-berms  | 0.29 | banked flow corners  | 12.5      | 9.5      |
| seg-jumps  | 0.44 | jump line            | 13.5      | 10.0     |
| seg-forest | 0.75 | forest loam          | 12.0      | 9.0      |
| seg-finish | 0.95 | final sprint         | 13.5      | 10.0     |

(`trail.finishT` is 0.985, so the finish take crosses the line — intended.)

## Smoke-test record

2026-08-06, hidden browser pane, this bot.js as committed: boot with zero
`__DESCENT_ERRORS__`; `restartToRunning()` passed the countdown; teleport to t=0.29 gave
trailT 0.290 / speed 9.5 / no crash; a 150-frame real-time take at t=0.29 (cap 12.5)
finished with **zero crashes** (t 0.290 → 0.311, end speed 10.8); a 24-frame `slow: 4`
take at t=0.44 likewise. Frames on disk: JPEG content, correct pinned size, rider framed
mid-shot with dust and motion blur.
