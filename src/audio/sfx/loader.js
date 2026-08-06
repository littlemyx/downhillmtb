// =============================================================================
// SFX sample loader.
//
// Same machinery as music/loader.js, but the manifest groups one-shot variants
// by `kind` (crash_01, crash_02 … all carry kind:"crash") and pick(kind, r)
// returns a random decoded variant. Fetching needs no AudioContext; decoding
// waits for the gesture-built context and is drained per-frame by audio.js.
//
// Every asset is optional. While a kind has nothing decoded, pick() returns
// null and audio.js keeps its procedural fallback — a missing manifest simply
// means the old sound, never an error.
// =============================================================================

const FETCH_TIMEOUT_MS = 20000;
const RETRY_DELAYS = [1000, 3000, 9000];
const MAX_CONCURRENT = 2;

export function createSfxLoader(basePath) {
  // 'idle' -> 'manifest' -> 'loading' -> 'done' | 'disabled'
  let state = 'idle';
  let disposed = false;
  let fetched = 0;
  let failed = 0;

  const byKind = new Map();    // kind -> AudioBuffer[]
  const pending = [];          // { asset, data: ArrayBuffer } awaiting decode
  const decoding = new Set();
  const controllers = new Set();

  let useOpus = false;
  try { useOpus = new Audio().canPlayType('audio/ogg; codecs=opus') !== ''; }
  catch (e) { /* no <audio>? then AAC is the safer guess anyway */ }

  async function fetchRaw(url) {
    for (let attempt = 0; ; attempt++) {
      if (disposed) return null;
      const ctrl = new AbortController();
      controllers.add(ctrl);
      const timer = setTimeout(() => ctrl.abort(), FETCH_TIMEOUT_MS);
      try {
        const res = await fetch(url, { signal: ctrl.signal });
        if (res.status >= 400 && res.status < 500) return null;
        if (!res.ok) throw new Error('http ' + res.status);
        return await res.arrayBuffer();
      } catch (e) {
        if (disposed || attempt >= RETRY_DELAYS.length) return null;
        await new Promise((r) => setTimeout(r, RETRY_DELAYS[attempt]));
      } finally {
        clearTimeout(timer);
        controllers.delete(ctrl);
      }
    }
  }

  async function start() {
    if (state !== 'idle') return;
    state = 'manifest';
    const raw = await fetchRaw(basePath + 'manifest.json');
    if (disposed) return;
    let mf = null;
    try { mf = JSON.parse(new TextDecoder().decode(raw)); } catch (e) { /* fall through */ }
    if (!mf || !Array.isArray(mf.assets) || !mf.assets.length) { state = 'disabled'; return; }
    state = 'loading';

    const queue = mf.assets.slice().sort((a, b) => (a.priority || 0) - (b.priority || 0));
    let idx = 0;
    const worker = async () => {
      while (idx < queue.length && !disposed) {
        const asset = queue[idx++];
        const file = asset.files ? (useOpus ? asset.files.opus : asset.files.aac) : asset.file;
        if (!file) { failed++; continue; }
        const data = await fetchRaw(basePath + file);
        if (disposed) return;
        if (!data) { failed++; continue; }
        fetched++;
        pending.push({ asset, data });
      }
    };
    const workers = [];
    for (let i = 0; i < MAX_CONCURRENT; i++) workers.push(worker());
    await Promise.all(workers);
    if (!disposed && state === 'loading') state = 'done';
  }

  /** Called per-frame by audio.js once its AudioContext exists. */
  function drainDecodes(ac) {
    while (pending.length) {
      const item = pending.pop();
      decoding.add(item.asset.id);
      ac.decodeAudioData(item.data)
        .then((buf) => {
          decoding.delete(item.asset.id);
          if (disposed) return;
          const kind = item.asset.kind || item.asset.id;
          let arr = byKind.get(kind);
          if (!arr) { arr = []; byKind.set(kind, arr); }
          arr.push(buf);
        })
        .catch(() => { decoding.delete(item.asset.id); failed++; });
    }
  }

  /** Random decoded variant of `kind`, or null. `r` in [0,1) picks the variant. */
  function pick(kind, r) {
    const arr = byKind.get(kind);
    if (!arr || !arr.length) return null;
    const i = Math.min(arr.length - 1, Math.floor((r || 0) * arr.length));
    return arr[i];
  }

  return {
    start,
    drainDecodes,
    pick,
    dispose() {
      disposed = true;
      for (const c of controllers) { try { c.abort(); } catch (e) { /* ignore */ } }
      controllers.clear();
      pending.length = 0;
      byKind.clear();
    },
    get state() { return state; },
    get fetched() { return fetched; },
    get failed() { return failed; },
    /** Decoded variant count across all kinds (debug/verification). */
    get decodedCount() {
      let n = 0;
      for (const arr of byKind.values()) n += arr.length;
      return n;
    },
  };
}
