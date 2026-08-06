// =============================================================================
// Music asset loader.
//
// Fetches manifest.json and the stems it lists, staged by priority so the base
// layer and stingers arrive first. Fetching needs no AudioContext and starts
// as soon as music.js kicks it (strictly after platform ready); decoding waits
// for the gesture-unlocked context and is drained per-frame by music.js.
//
// Every asset is optional. A missing phrase shrinks its pool, a missing layer
// leaves its tension window silent, a missing manifest disables music — the
// game itself must never notice any of it.
// =============================================================================

const FETCH_TIMEOUT_MS = 20000;
const RETRY_DELAYS = [1000, 3000, 9000];
const MAX_CONCURRENT = 2;

export function createMusicLoader(basePath) {
  // 'idle' -> 'manifest' -> 'loading' -> 'done' | 'disabled'
  let state = 'idle';
  let manifest = null;
  let disposed = false;
  let fetched = 0;
  let failed = 0;

  const buffers = new Map();   // asset id -> AudioBuffer
  const pending = [];          // { asset, data: ArrayBuffer } awaiting decode
  const decoding = new Set();  // asset ids currently in decodeAudioData
  const controllers = new Set();
  let onBufferCb = null;

  // Chromium (the Yandex majority) decodes ogg/opus; Safari does not but
  // decodes AAC everywhere. One probe, up front.
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
        // 4xx is permanent (asset genuinely absent) — retrying only delays
        // the clean 'disabled' fallback.
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

  // Fetch stage: manifest priority if given, else stingers/layers before phrases.
  function priorityOf(a) {
    if (typeof a.priority === 'number') return a.priority;
    if (a.role === 'stinger') return 0;
    if (a.role === 'layer') return 1;
    return 2;
  }

  async function start() {
    if (state !== 'idle') return;
    state = 'manifest';
    const raw = await fetchRaw(basePath + 'manifest.json');
    if (disposed) return;
    let mf = null;
    try { mf = JSON.parse(new TextDecoder().decode(raw)); } catch (e) { /* fall through */ }
    if (!mf || !Array.isArray(mf.assets) || !(mf.bpm > 0)) { state = 'disabled'; return; }
    manifest = mf;
    state = 'loading';

    const queue = mf.assets.slice().sort((a, b) => priorityOf(a) - priorityOf(b));
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

  /** Called per-frame by music.js once the AudioContext exists. */
  function drainDecodes(ac) {
    while (pending.length) {
      const item = pending.pop();
      decoding.add(item.asset.id);
      ac.decodeAudioData(item.data)
        .then((buf) => {
          decoding.delete(item.asset.id);
          if (disposed) return;
          buffers.set(item.asset.id, buf);
          if (onBufferCb) onBufferCb(item.asset, buf);
        })
        .catch(() => { decoding.delete(item.asset.id); failed++; });
    }
  }

  return {
    start,
    drainDecodes,
    onBuffer(cb) { onBufferCb = cb; },
    dispose() {
      disposed = true;
      for (const c of controllers) { try { c.abort(); } catch (e) { /* ignore */ } }
      controllers.clear();
      pending.length = 0;
      buffers.clear();
      onBufferCb = null;
    },
    get state() { return state; },
    get manifest() { return manifest; },
    get buffers() { return buffers; },
    get fetched() { return fetched; },
    get failed() { return failed; },
    get decodingCount() { return decoding.size; },
  };
}
