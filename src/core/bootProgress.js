// DESCENT — boot progress plumbing.
//
// The splash overlay and the window.__DESCENT_BOOT__ API live in index.html so
// they paint before a single module byte is evaluated. This module is the
// game-side half: it maps per-stage progress into the overall bar and hands the
// main thread back to the compositor often enough for that bar to actually
// repaint. World generation is one ~11s synchronous burst without these yields.
//
// Everything is a no-op when the overlay is absent (tests, harnesses, a future
// host page without the splash), so callers never guard.

const YIELD_BUDGET_MS = 80; // one dropped vsync per ~80ms of work ≈ 15% overhead ceiling

let stageStart = 0;
let stageEnd = 0;
let lastYield = (typeof performance !== 'undefined') ? performance.now() : 0;

function overlay() {
  return (typeof window !== 'undefined' && window.__DESCENT_BOOT__) || null;
}

/** Enter a stage occupying [startFrac, endFrac] of the bar, labelled by `key`. */
export function beginStage(startFrac, endFrac, key) {
  stageStart = startFrac;
  stageEnd = endFrac;
  overlay()?.set(startFrac, key);
}

/** Report local progress t01 ∈ [0,1] within the current stage. */
export function report(t01) {
  const t = Math.max(0, Math.min(1, t01));
  overlay()?.set(stageStart + (stageEnd - stageStart) * t);
}

// One macrotask-with-paint yield. scheduler.yield() where available; otherwise
// rAF→setTimeout, which resolves after the frame commits. Either way a plain
// 250 ms timeout races alongside, because rAF never fires in a hidden tab —
// without the race, switching tabs during load would freeze boot entirely.
function yieldOnce() {
  if (typeof scheduler !== 'undefined' && typeof scheduler.yield === 'function') {
    return scheduler.yield();
  }
  return new Promise((resolve) => {
    let done = false;
    const finish = () => { if (!done) { done = true; resolve(); } };
    requestAnimationFrame(() => setTimeout(finish, 0));
    setTimeout(finish, 250);
  });
}

/**
 * Time-budgeted yield. Returns null while under budget so hot loops pay only a
 * clock read between yields — call as: `const y = maybeYield(); if (y) await y;`.
 */
export function maybeYield() {
  if (!overlay()) return null;
  const now = performance.now();
  if (now - lastYield < YIELD_BUDGET_MS) return null;
  lastYield = now;
  return yieldOnce();
}

/** Unconditional yield — used at stage boundaries so the new label paints. */
export function yieldFrame() {
  lastYield = (typeof performance !== 'undefined') ? performance.now() : 0;
  if (!overlay()) return Promise.resolve();
  return yieldOnce();
}
