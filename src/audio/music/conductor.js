// =============================================================================
// Conductor — the musical clock, the layer mixer and the phrase scheduler.
//
// All layer loops start phase-locked to one epoch and never stop; "silent" is
// a gain of zero. That makes every layer transition seamless and lets the base
// pad run continuously from the menu into the run.
//
// Melodic uniqueness: phrase decisions are a pure function of
// (runSeed, barIndexSinceRunStart, band) — the seed gives a run its melodic
// identity, the tension band only gates density and register. Same seed, same
// riding → the identical schedule, which is what QA asserts on.
// =============================================================================

import { makeRng, subSeed } from '../../core/rng.js';

const LOOKAHEAD_S = 0.35;          // schedule a bar when it is this close
const DENSITY_BY_BAND = [0.15, 0.35, 0.55, 0.7];
const DECISION_LOG_N = 32;
const ATTACK_S = 0.01;

const dbToLin = (db) => Math.pow(10, (db || 0) / 20);

export function createConductor(ac, destination) {
  let manifest = null;
  let barDur = 2.5;
  let epoch = -1;                  // ac.currentTime of bar 0; -1 until start()
  let musicSeed = 1;
  let runBar0 = 0;                 // absolute bar index at the last run:start
  let nextBar = 0;                 // next absolute bar to decide
  let playing = false;

  const layers = [];               // { asset, buffer, src, gain, level }
  const phrases = [];              // { asset, buffer }
  const stingers = new Map();      // event name -> { asset, buffer }
  const lastDecisions = [];        // ring buffer, newest last
  const layerGains = [];           // mirrored levels for debugState

  function setManifest(mf) {
    manifest = mf;
    barDur = (mf.beatsPerBar || 4) * 60 / mf.bpm;
  }

  function addBuffer(asset, buffer) {
    if (asset.role === 'layer') {
      const l = { asset, buffer, src: null, gain: null, level: 0 };
      layers.push(l);
      layerGains.push(0);
      if (playing) startLayer(l);
    } else if (asset.role === 'phrase') {
      phrases.push({ asset, buffer });
      // Keep the pool order manifest-stable regardless of arrival order, or
      // decode timing would change which phrase a seed picks.
      phrases.sort((a, b) => manifest.assets.indexOf(a.asset) - manifest.assets.indexOf(b.asset));
    } else if (asset.role === 'stinger' && asset.event) {
      stingers.set(asset.event, { asset, buffer });
    }
  }

  function startLayer(l) {
    const a = l.asset;
    const src = ac.createBufferSource();
    src.buffer = l.buffer;
    src.loop = true;
    src.loopStart = a.loopStart || 0;
    src.loopEnd = a.loopEnd || l.buffer.duration;
    const g = ac.createGain();
    g.gain.value = 0.0001;
    src.connect(g);
    g.connect(destination);
    const loopDur = Math.max(0.05, src.loopEnd - src.loopStart);
    const now = ac.currentTime;
    if (now < epoch) {
      src.start(epoch, src.loopStart);
    } else {
      // Late-decoded layer: enter mid-loop, phase-locked to the epoch.
      src.start(now, src.loopStart + ((now - epoch) % loopDur));
    }
    l.src = src;
    l.gain = g;
  }

  function start(seed) {
    if (playing || !manifest) return;
    epoch = ac.currentTime + 0.1;
    newRun(seed);
    playing = true;
    for (const l of layers) startLayer(l);
  }

  function newRun(seed) {
    musicSeed = subSeed(subSeed((seed >>> 0) || 1, 'music'), 'bars');
    if (epoch >= 0) {
      runBar0 = Math.max(0, Math.ceil((ac.currentTime - epoch) / barDur));
      nextBar = runBar0;
    }
    lastDecisions.length = 0;
  }

  /** Trapezoid window [inLo, inHi, outHi, outLo] over tension, sin-shaped. */
  function windowGain(w, tension) {
    if (!w || w.length !== 4) return 1;
    let f;
    if (tension <= w[0]) f = 0;
    else if (tension < w[1]) f = (tension - w[0]) / Math.max(1e-4, w[1] - w[0]);
    else if (tension <= w[2]) f = 1;
    else if (tension < w[3]) f = 1 - (tension - w[2]) / Math.max(1e-4, w[3] - w[2]);
    else f = 0;
    return Math.sin(f * Math.PI * 0.5);
  }

  function playOneShot(entry, when, extraGainLin) {
    const src = ac.createBufferSource();
    src.buffer = entry.buffer;
    const g = ac.createGain();
    const peak = dbToLin(entry.asset.gain) * (extraGainLin || 1);
    src.connect(g);
    g.connect(destination);
    const t = Math.max(when, ac.currentTime);
    g.gain.setValueAtTime(0.0001, t);
    g.gain.linearRampToValueAtTime(peak, t + ATTACK_S);
    // Fades are baked into the samples offline; the runtime envelope is only a
    // click guard on entry and a safety release at the natural end.
    const rel = Math.max(0.05, entry.asset.release || 0.1);
    const end = t + entry.buffer.duration;
    g.gain.setTargetAtTime(0.0001, end - rel, rel * 0.35);
    src.start(t);
    src.stop(end + 0.05);
    src.onended = () => { g.disconnect(); };
  }

  function decide(bar, band) {
    const rel = bar - runBar0;
    const rng = makeRng(subSeed(musicSeed, 'bar' + rel));
    const rPlay = rng();
    const rPick = rng();
    const rHalf = rng();
    const density = DENSITY_BY_BAND[band];
    const gated = rPlay >= density;
    let id = null;
    if (!gated && phrases.length) {
      // Pool filtered by band; falls back to the full pool if the band has none.
      let n = 0;
      for (let i = 0; i < phrases.length; i++) if (allowed(phrases[i].asset, band)) n++;
      let pick = (rPick * (n || phrases.length)) | 0;
      let entry = null;
      for (let i = 0; i < phrases.length; i++) {
        if (n > 0 && !allowed(phrases[i].asset, band)) continue;
        if (pick-- === 0) { entry = phrases[i]; break; }
      }
      if (entry) {
        id = entry.asset.id;
        const half = rHalf < 0.35 ? barDur * 0.5 : 0;
        playOneShot(entry, epoch + bar * barDur + half, 1);
      }
    }
    if (lastDecisions.length >= DECISION_LOG_N) lastDecisions.shift();
    lastDecisions.push({ bar: rel, id, band, gated });
  }

  function allowed(asset, band) {
    return !asset.bands || asset.bands.indexOf(band) >= 0;
  }

  function update(dt, tension, band) {
    if (!playing) return;
    // Layer gains: trapezoid window over tension, written only on real change.
    for (let i = 0; i < layers.length; i++) {
      const l = layers[i];
      if (!l.gain) continue;
      const target = windowGain(l.asset.window, tension) * dbToLin(l.asset.gain);
      if (Math.abs(target - l.level) > 0.003) {
        l.level = target;
        layerGains[i] = target;
        l.gain.gain.setTargetAtTime(Math.max(0.0001, target), ac.currentTime, 0.4);
      }
    }
    // Phrase scheduler: decide each bar just before it starts.
    while (epoch + nextBar * barDur - ac.currentTime < LOOKAHEAD_S) {
      decide(nextBar, band);
      nextBar++;
    }
  }

  function stinger(event, quantizeHalfBar) {
    const entry = stingers.get(event);
    if (!entry || !playing) return false;
    let when = ac.currentTime;
    if (quantizeHalfBar && epoch >= 0) {
      const half = barDur * 0.5;
      when = epoch + Math.ceil((when - epoch) / half) * half;
    }
    playOneShot(entry, when, 1);
    return true;
  }

  function dispose() {
    playing = false;
    for (const l of layers) {
      try { if (l.src) l.src.stop(); } catch (e) { /* already stopped */ }
      try { if (l.gain) l.gain.disconnect(); } catch (e) { /* ignore */ }
      l.src = null; l.gain = null;
    }
    layers.length = 0;
    phrases.length = 0;
    stingers.clear();
  }

  return {
    setManifest,
    addBuffer,
    start,
    newRun,
    update,
    stinger,
    dispose,
    get playing() { return playing; },
    get barIndex() { return epoch < 0 ? -1 : Math.floor((ac.currentTime - epoch) / barDur) - runBar0; },
    get barDur() { return barDur; },
    get layerGains() { return layerGains; },
    get lastDecisions() { return lastDecisions; },
  };
}
