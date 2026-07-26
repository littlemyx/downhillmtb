// collision.js — terrain contact helpers for the physics layer. CONTRACT §5.
//
// This module knows nothing about bikes. It answers geometric questions about the
// heightfield: "how deep is this wheel/sphere/capsule into the ground, and what is
// the surface doing there?".
//
// Everything reads the terrain through terrain.sampleHeight / sampleNormal /
// sampleMaterial / sampleSlope. We NEVER raycast terrain.group — per ADDENDUM §A the
// quadtree toggles chunk visibility every frame, so a mesh raycast is camera-dependent
// and therefore not a physics answer.
//
// CONTRACT-NOTE: sweepWheel takes an optional 4th argument `forward` (a Vector3).
//   When supplied, the contact fan is swept through the *wheel plane* (the lower 120°
//   of the contact arc, in the direction of travel), which is what lets a wheel climb
//   onto a rock or root ahead of it instead of snapping down to a single height
//   sample. When omitted it falls back to a fan aligned with -Z. The 3-argument form
//   specified in the contract still works.
// CONTRACT-NOTE: additive helpers beyond the contract — sphereCast, capsuleCast,
//   raycast (general direction, marched), profile (slope + curvature lookahead, used
//   by the bike's pumping detector), heightAt/normalAt/materialAt/slopeAt, makeHit,
//   copyHit. All allocation free after module init.
//
// Allocation discipline: every public call takes an `out` hit and writes into it.
// Nothing here allocates once created.

import * as THREE from 'three';
import { Surface } from '../world/terrain.js';

// ---------------------------------------------------------------------------
// Fan tables — built once at module load.
// ---------------------------------------------------------------------------

// Wheel contact arc: 9 samples across the lower 120° of the rim, in the wheel plane.
// 9 is the sweet spot — 5 is visibly steppy over roots, 13 costs more than it buys
// because the terrain detail overlay is only 0.35 m/sample anyway.
const ARC_N = 9;
const ARC_SPAN = Math.PI / 3;              // ±60° from bottom-dead-centre
const ARC_SIN = new Float32Array(ARC_N);   // horizontal offset factor (× radius)
const ARC_COS = new Float32Array(ARC_N);   // vertical drop factor      (× radius)
const ARC_W = new Float32Array(ARC_N);     // normal-blend weight
for (let i = 0; i < ARC_N; i++) {
  const a = ((i / (ARC_N - 1)) * 2 - 1) * ARC_SPAN;
  ARC_SIN[i] = Math.sin(a);
  ARC_COS[i] = Math.cos(a);
  // cos³ — the real contact patch is at the bottom of the arc. The shoulders exist
  // to *detect* an obstacle early, not to define the surface you are riding on.
  ARC_W[i] = ARC_COS[i] * ARC_COS[i] * ARC_COS[i];
}
const ARC_MID = (ARC_N - 1) >> 1;

// Radial fan for spheres: centre + 5 at 0.58r + 8 at 0.93r.
const SPH_N = 14;
const SPH_OX = new Float32Array(SPH_N);
const SPH_OZ = new Float32Array(SPH_N);
const SPH_DY = new Float32Array(SPH_N);    // vertical drop factor (× radius, negative)
const SPH_W = new Float32Array(SPH_N);
{
  let k = 0;
  const push = (frac, ang) => {
    SPH_OX[k] = Math.cos(ang) * frac;
    SPH_OZ[k] = Math.sin(ang) * frac;
    SPH_DY[k] = -Math.sqrt(Math.max(0, 1 - frac * frac));
    SPH_W[k] = 1 - frac * frac * 0.75;
    k++;
  };
  push(0, 0);
  for (let i = 0; i < 5; i++) push(0.58, (i / 5) * Math.PI * 2);
  for (let i = 0; i < 8; i++) push(0.93, (i / 8) * Math.PI * 2 + 0.31);
}

// ---------------------------------------------------------------------------
// Scratch — module scope, never reallocated.
// ---------------------------------------------------------------------------

const _lift = new Float32Array(ARC_N);
const _hx = new Float32Array(ARC_N);
const _hz = new Float32Array(ARC_N);
const _hh = new Float32Array(ARC_N);

const _nAcc = new THREE.Vector3();
const _nTmp = new THREE.Vector3();
const _fwdN = new THREE.Vector3();
const _pTmp = new THREE.Vector3();
const _rayP = new THREE.Vector3();

/** Build (or reset) a Hit record. Only allocates when `out` is not supplied. */
function makeHit(out) {
  const h = out || {};
  if (!h.normal) h.normal = new THREE.Vector3(0, 1, 0);
  if (!h.point) h.point = new THREE.Vector3();
  h.height = 0;
  h.normal.set(0, 1, 0);
  h.material = Surface.DIRT;
  h.slope = 0;
  h.depth = -Infinity;
  h.point.set(0, 0, 0);
  h.curvature = 0;    // + = concave (a compression), - = convex (a crest)
  h.gradAhead = 0;    // dh/ds along `forward` across the contact arc
  h.valid = false;
  return h;
}

function copyHit(src, dst) {
  dst.height = src.height;
  dst.normal.copy(src.normal);
  dst.material = src.material;
  dst.slope = src.slope;
  dst.depth = src.depth;
  dst.point.copy(src.point);
  dst.curvature = src.curvature;
  dst.gradAhead = src.gradAhead;
  dst.valid = src.valid;
  return dst;
}

// Fallback hits so a caller that passes nothing still gets a stable object back
// without allocating per call.
const _fallbackHit = makeHit(null);
const _tmpHit = makeHit(null);

// Reusable record for the lookahead helper.
const _profileOut = { here: 0, ahead: 0, behind: 0, grad: 0, curvature: 0 };

export function createCollision(ctx) {
  // Cached bounds so we do not walk a property chain per sample.
  let bMinX = -1e6, bMaxX = 1e6, bMinZ = -1e6, bMaxZ = 1e6;
  let boundsReady = false;

  function terrain() {
    const t = ctx.terrain;
    if (t && !boundsReady && t.bounds) {
      // 1 cm inset: sampleHeight clamps internally, but the derivative at the very
      // edge is one-sided and a clamped normal there reads as a vertical wall.
      bMinX = t.bounds.minX + 0.01; bMaxX = t.bounds.maxX - 0.01;
      bMinZ = t.bounds.minZ + 0.01; bMaxZ = t.bounds.maxZ - 0.01;
      boundsReady = true;
    }
    return t;
  }

  const cx = (x) => (x < bMinX ? bMinX : x > bMaxX ? bMaxX : x);
  const cz = (z) => (z < bMinZ ? bMinZ : z > bMaxZ ? bMaxZ : z);

  // -------------------------------------------------------------------------
  // Point queries
  // -------------------------------------------------------------------------

  /** Height only — the hot path, called ~120×/frame by the wheel sweeps. */
  function heightAt(x, z) {
    const t = terrain();
    if (!t) return 0;
    return t.sampleHeight(cx(x), cz(z));
  }

  function materialAt(x, z) {
    const t = terrain();
    if (!t) return Surface.DIRT;
    return t.sampleMaterial(cx(x), cz(z));
  }

  function slopeAt(x, z) {
    const t = terrain();
    if (!t) return 0;
    return t.sampleSlope(cx(x), cz(z));
  }

  function normalAt(x, z, out) {
    const o = out || _nTmp;
    const t = terrain();
    if (!t) return o.set(0, 1, 0);
    return t.sampleNormal(cx(x), cz(z), o);
  }

  /** CONTRACT §5 — probe(x, z, outHit) -> Hit. */
  function probe(x, z, out) {
    const hit = makeHit(out || _fallbackHit);
    const t = terrain();
    if (!t || !isFinite(x) || !isFinite(z)) return hit;
    const px = cx(x), pz = cz(z);
    hit.height = t.sampleHeight(px, pz);
    t.sampleNormal(px, pz, hit.normal);
    hit.material = t.sampleMaterial(px, pz);
    hit.slope = t.sampleSlope(px, pz);
    hit.point.set(px, hit.height, pz);
    hit.depth = 0;
    hit.valid = true;
    return hit;
  }

  // -------------------------------------------------------------------------
  // sweepWheel — the important one
  // -------------------------------------------------------------------------

  /**
   * Deepest contact of a wheel against the heightfield.
   *
   * The wheel is a disc of radius `radius` centred on `center`, lying in the plane
   * spanned by `forward` and world up. We sample the lower 120° of the rim. For each
   * sample the rim sits at
   *
   *     p_i = center + radius·sinθ_i·forward_h  +  (0, −radius·cosθ_i, 0)
   *
   * and `lift_i = h(p_i.xz) − p_i.y` is how far the wheel would have to rise
   * *vertically* for that point of the rim to clear the ground. Because the terrain
   * is a heightfield, a purely vertical translation leaves every h(p_i.xz) unchanged
   * — so `max(lift_i)` is the EXACT vertical displacement that resolves the whole
   * disc. That is what we return as `depth`.
   *
   * Two consequences worth understanding:
   *  - On a slope of angle φ this automatically recovers the correct r/cos(φ) centre
   *    height. A single centre sample gets that wrong (it under-lifts, so the wheel
   *    visibly sinks into every steep section).
   *  - Approaching a rock or root, the *forward* arc samples catch it first, so the
   *    wheel begins climbing before the axle is over it. That is what removes the
   *    violent single-sample jitter on rough ground — the classic amateur tell.
   *
   * The contact normal is a weighted blend of the terrain normals at the samples
   * that are actually near the deepest contact, weighted by cos³θ so the bottom of
   * the arc dominates. Blending is what stops the normal snapping between facets and
   * shaking the whole chassis.
   *
   * Also fills `curvature` (+ concave) and `gradAhead` from the sampled heights —
   * both are free here and the bike's pump detector needs them.
   *
   * @returns true if the wheel touches or penetrates the ground.
   */
  function sweepWheel(center, radius, out, forward) {
    const hit = makeHit(out || _fallbackHit);
    const t = terrain();
    if (!t || !center || !isFinite(center.x) || !isFinite(center.y) || !isFinite(center.z)) {
      hit.depth = -1e3;
      return false;
    }
    const r = radius > 0 ? radius : 0.37;

    // Horizontal travel direction for the arc.
    if (forward) {
      _fwdN.set(forward.x, 0, forward.z);
      const l2 = _fwdN.lengthSq();
      if (l2 > 1e-8) _fwdN.multiplyScalar(1 / Math.sqrt(l2));
      else _fwdN.set(0, 0, -1);
    } else {
      _fwdN.set(0, 0, -1);
    }

    let best = 0;
    let bestLift = -Infinity;
    for (let i = 0; i < ARC_N; i++) {
      const off = r * ARC_SIN[i];
      const x = cx(center.x + _fwdN.x * off);
      const z = cz(center.z + _fwdN.z * off);
      const h = t.sampleHeight(x, z);
      const lift = h - (center.y - r * ARC_COS[i]);
      _hx[i] = x; _hz[i] = z; _hh[i] = h; _lift[i] = lift;
      if (lift > bestLift) { bestLift = lift; best = i; }
    }

    hit.depth = bestLift;
    hit.height = _hh[best];
    hit.point.set(_hx[best], _hh[best], _hz[best]);
    hit.material = t.sampleMaterial(_hx[best], _hz[best]);
    hit.slope = t.sampleSlope(_hx[best], _hz[best]);
    hit.valid = true;

    // --- blended contact normal -------------------------------------------
    // Only samples within BLEND_BAND of the deepest one describe the surface the
    // tyre is actually sitting on; anything further out is a different facet and
    // would drag the normal towards ground the tyre is not touching.
    const BLEND_BAND = 0.085 + r * 0.11;   // ~12.6 cm for a 0.37 m wheel
    _nAcc.set(0, 0, 0);
    let wsum = 0;
    for (let i = 0; i < ARC_N; i++) {
      const prox = 1 - (bestLift - _lift[i]) / BLEND_BAND;
      if (prox <= 0.02) continue;
      const w = ARC_W[i] * prox * prox;
      if (w <= 1e-4) continue;
      t.sampleNormal(_hx[i], _hz[i], _nTmp);
      _nAcc.addScaledVector(_nTmp, w);
      wsum += w;
    }
    if (wsum > 1e-6) {
      _nAcc.multiplyScalar(1 / wsum);
      const l2 = _nAcc.lengthSq();
      if (l2 > 1e-9) hit.normal.copy(_nAcc).multiplyScalar(1 / Math.sqrt(l2));
      else t.sampleNormal(_hx[best], _hz[best], hit.normal);
    } else {
      t.sampleNormal(_hx[best], _hz[best], hit.normal);
    }

    // --- profile along the arc (feeds the pump detector) -------------------
    const span = r * (ARC_SIN[ARC_N - 1] - ARC_SIN[0]);
    if (span > 1e-4) {
      hit.gradAhead = (_hh[ARC_N - 1] - _hh[0]) / span;
      const half = span * 0.5;
      hit.curvature = (_hh[0] + _hh[ARC_N - 1] - 2 * _hh[ARC_MID]) / (half * half);
    }

    return bestLift >= 0;
  }

  // -------------------------------------------------------------------------
  // sphereCast / capsuleCast — rider body, crash tumble, wall detection
  // -------------------------------------------------------------------------

  /**
   * Deepest penetration of a sphere against the heightfield, using the same
   * "maximum vertical lift" formulation as sweepWheel over a radial fan.
   */
  function sphereCast(center, radius, out) {
    const hit = makeHit(out || _fallbackHit);
    const t = terrain();
    if (!t || !center || !isFinite(center.x) || !isFinite(center.y)) {
      hit.depth = -1e3;
      return false;
    }
    const r = radius > 0 ? radius : 0.5;

    let bestLift = -Infinity;
    let bx = center.x, bz = center.z, bh = 0;
    _nAcc.set(0, 0, 0);
    let wsum = 0;

    for (let i = 0; i < SPH_N; i++) {
      const x = cx(center.x + SPH_OX[i] * r);
      const z = cz(center.z + SPH_OZ[i] * r);
      const h = t.sampleHeight(x, z);
      const lift = h - (center.y + SPH_DY[i] * r);
      if (lift > bestLift) { bestLift = lift; bx = x; bz = z; bh = h; }
      // Weight the normal blend towards samples close to (or inside) the surface.
      if (lift > -0.35) {
        const w = SPH_W[i] * (lift + 0.35);
        t.sampleNormal(x, z, _nTmp);
        _nAcc.addScaledVector(_nTmp, w);
        wsum += w;
      }
    }

    hit.depth = bestLift;
    hit.height = bh;
    hit.point.set(bx, bh, bz);
    hit.material = t.sampleMaterial(bx, bz);
    hit.slope = t.sampleSlope(bx, bz);
    hit.valid = true;
    if (wsum > 1e-6) {
      _nAcc.multiplyScalar(1 / wsum);
      const l2 = _nAcc.lengthSq();
      if (l2 > 1e-9) hit.normal.copy(_nAcc).multiplyScalar(1 / Math.sqrt(l2));
      else t.sampleNormal(bx, bz, hit.normal);
    } else {
      t.sampleNormal(bx, bz, hit.normal);
    }
    return bestLift >= 0;
  }

  /**
   * Capsule (segment a→b, radius r) against the heightfield. Samples the segment
   * and keeps the deepest sphere hit. Used for the rider body and the crash tumble
   * where a single sphere is too coarse.
   */
  function capsuleCast(a, b, radius, out, steps) {
    const hit = makeHit(out || _fallbackHit);
    const t = terrain();
    if (!t || !a || !b) { hit.depth = -1e3; return false; }
    const n = Math.max(2, Math.min(8, steps || 4));
    let found = false;
    let bestDepth = -Infinity;
    for (let i = 0; i < n; i++) {
      const f = i / (n - 1);
      _pTmp.set(
        a.x + (b.x - a.x) * f,
        a.y + (b.y - a.y) * f,
        a.z + (b.z - a.z) * f);
      const touched = sphereCast(_pTmp, radius, _tmpHit);
      if (_tmpHit.depth > bestDepth) {
        bestDepth = _tmpHit.depth;
        copyHit(_tmpHit, hit);
        found = touched;
      }
    }
    return found;
  }

  // -------------------------------------------------------------------------
  // Rays
  // -------------------------------------------------------------------------

  /**
   * CONTRACT §5 — rayDown(origin, maxDist, outHit) -> boolean.
   *
   * For a strictly vertical ray against a heightfield the answer is a single lookup:
   * the ground under (x,z) IS the hit, so marching would only cost time. We still
   * fill the hit when returning false, because callers want the data anyway ("how
   * far below me is the ground?" for the airborne landing prediction).
   */
  function rayDown(origin, maxDist, out) {
    const hit = makeHit(out || _fallbackHit);
    const t = terrain();
    if (!t || !origin) { hit.depth = -1e3; return false; }
    const px = cx(origin.x), pz = cz(origin.z);
    const h = t.sampleHeight(px, pz);
    t.sampleNormal(px, pz, hit.normal);
    hit.height = h;
    hit.material = t.sampleMaterial(px, pz);
    hit.slope = t.sampleSlope(px, pz);
    hit.point.set(px, h, pz);
    const dist = origin.y - h;      // + = above the ground
    hit.depth = -dist;              // + = below the ground, matching sweepWheel
    hit.valid = true;
    if (dist < 0) return true;      // already inside the ground
    return !(maxDist !== undefined && maxDist !== null && dist > maxDist);
  }

  /**
   * General ray against the heightfield, marched. `dir` need not be normalised.
   * Adaptive stride (large while far above the surface, so a long ray over a valley
   * is cheap) then 8 bisections once the sign of (rayY − groundY) flips. Not called
   * per substep — this is for camera de-clipping and one-off queries.
   */
  function raycast(origin, dir, maxDist, out) {
    const hit = makeHit(out || _fallbackHit);
    const t = terrain();
    if (!t || !origin || !dir) { hit.depth = -1e3; return false; }
    // Math.sqrt rather than Math.hypot throughout this file: V8 allocates on every
    // Math.hypot call, and these run in the physics inner loop.
    const dl = Math.sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    if (dl < 1e-9) return false;
    const dx = dir.x / dl, dy = dir.y / dl, dz = dir.z / dl;
    const far = maxDist > 0 ? maxDist : 400;

    let s = 0;
    let prevS = 0;
    let prevGap = origin.y - t.sampleHeight(cx(origin.x), cz(origin.z));
    if (prevGap < 0) {
      probe(origin.x, origin.z, hit);   // started underground
      hit.depth = 0;
      return true;
    }
    while (s < far) {
      // Step proportional to the current clearance; never below 0.4 m so a grazing
      // ray terminates, never above 12 m so we cannot step over a ridge.
      const step = Math.min(12, Math.max(0.4, prevGap * 0.85));
      s = Math.min(far, s + step);
      const gx = origin.x + dx * s, gy = origin.y + dy * s, gz = origin.z + dz * s;
      const gap = gy - t.sampleHeight(cx(gx), cz(gz));
      if (gap <= 0) {
        let lo = prevS, hi = s;
        for (let i = 0; i < 8; i++) {
          const mid = (lo + hi) * 0.5;
          _rayP.set(origin.x + dx * mid, origin.y + dy * mid, origin.z + dz * mid);
          const g = _rayP.y - t.sampleHeight(cx(_rayP.x), cz(_rayP.z));
          if (g > 0) lo = mid; else hi = mid;
        }
        _rayP.set(origin.x + dx * hi, origin.y + dy * hi, origin.z + dz * hi);
        probe(_rayP.x, _rayP.z, hit);
        hit.depth = hi;   // distance along the ray (this call only)
        return true;
      }
      prevS = s;
      prevGap = gap;
    }
    return false;
  }

  // -------------------------------------------------------------------------
  // Lookahead profile — slope and curvature along a direction.
  // -------------------------------------------------------------------------

  /**
   * Sample the ground behind / here / ahead along (dirX, dirZ) at ±dist and fill
   * `out` with the first and second derivative. `curvature > 0` means concave — a
   * compression, the back side of a roller, the entry to a berm. That is exactly
   * where pumping pays, which is why the bike asks for it.
   */
  function profile(x, z, dirX, dirZ, dist, out) {
    const o = out || _profileOut;
    const t = terrain();
    o.here = o.ahead = o.behind = o.grad = o.curvature = 0;
    if (!t) return o;
    const l = Math.sqrt(dirX * dirX + dirZ * dirZ);
    if (l < 1e-6 || !(dist > 0)) return o;
    const ux = dirX / l, uz = dirZ / l;
    o.here = t.sampleHeight(cx(x), cz(z));
    o.ahead = t.sampleHeight(cx(x + ux * dist), cz(z + uz * dist));
    o.behind = t.sampleHeight(cx(x - ux * dist), cz(z - uz * dist));
    o.grad = (o.ahead - o.here) / dist;
    o.curvature = (o.ahead + o.behind - 2 * o.here) / (dist * dist);
    return o;
  }

  return {
    // CONTRACT §5
    probe,
    sweepWheel,
    rayDown,
    // additive (see the CONTRACT-NOTEs at the top of this file)
    sphereCast,
    capsuleCast,
    raycast,
    profile,
    heightAt,
    normalAt,
    materialAt,
    slopeAt,
    makeHit: (out) => makeHit(out),
    copyHit,
    makeProfile: () => ({ here: 0, ahead: 0, behind: 0, grad: 0, curvature: 0 }),
    Surface,
    ARC_SAMPLES: ARC_N,
  };
}
