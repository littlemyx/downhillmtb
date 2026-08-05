// Yandex Games SDK adapter.
//
// This is the ONLY file in the project that touches `window.YaGames`. Every
// method degrades to a no-op when the SDK is missing (local dev, other hosting,
// blocked script), so the game runs identically off-platform.
//
// Moderation requirements covered:
//   2.14 — language is detected through the SDK during startup, before any UI
//          is built, and cached so a refresh applies it instantly.
//   1.19 — LoadingAPI.ready() once the first frames are on screen, and
//          GameplayAPI.start()/stop() following the run state machine.

const LANG_KEY = 'descent.lang';
const CLOUD_KEY = 'best';
const STORE_PREFIX = 'descent.v1';     // must match createStore() in game/gameplay.js
const STORE_VERSION = 1;
const LEADERBOARD = 'besttime';
const CANONICAL_SEED = 20260726;       // only the default track feeds the board
// Ad policy. Yandex does not mandate an interval — the platform caps frequency on its own
// side and simply reports wasShown:false when a call comes too soon — so these numbers are
// ours, chosen to stay well inside requirement 4.4 ("only at logical pauses").
const AD_EVERY_N_RUNS = 3;      // an interstitial on every third finish; the first two are clean
const AD_MIN_GAP_MS = 60000;    // floor, in case a future mode makes runs short enough to matter

const ACTIVE_STATES = new Set(['countdown', 'running', 'crashed']);

function readCachedLang() {
  try { return window.localStorage.getItem(LANG_KEY) || ''; } catch (e) { return ''; }
}
function writeCachedLang(lang) {
  try { window.localStorage.setItem(LANG_KEY, lang); } catch (e) { /* private mode */ }
}

/** Anything resembling Russian rides the ru dictionary; everything else falls back. */
function normalizeLang(raw) {
  const s = String(raw || '').toLowerCase();
  return s.startsWith('ru') ? 'ru' : 'en';
}

function withTimeout(promise, ms, fallback) {
  return new Promise((resolve) => {
    let settled = false;
    const done = (v) => { if (!settled) { settled = true; resolve(v); } };
    const timer = setTimeout(() => done(fallback), ms);
    Promise.resolve(promise).then(
      (v) => { clearTimeout(timer); done(v); },
      () => { clearTimeout(timer); done(fallback); },
    );
  });
}

/** Records that came back from the cloud are as untrusted as anything on disk. */
function sanitizeRecord(o, seed) {
  if (!o || typeof o !== 'object') return null;
  const time = Number(o.time);
  if (!isFinite(time) || time <= 0 || time > 36000) return null;
  if (o.seed !== undefined && (o.seed >>> 0) !== (seed >>> 0)) return null;
  const splits = Array.isArray(o.splits)
    ? o.splits.filter((s) => typeof s === 'number' && isFinite(s) && s >= 0)
    : [];
  const num = (v) => (typeof v === 'number' && isFinite(v) ? v : 0);
  return {
    time,
    splits,
    score: num(o.score),
    crashes: num(o.crashes),
    date: typeof o.date === 'string' ? o.date : '',
    seed: seed >>> 0,
  };
}

export function createPlatform() {
  let ysdk = null;
  let player = null;              // resolved lazily, only on the platform
  let ready = false;              // init() settled
  let lang = 'en';
  let readyMarked = false;
  let readyPending = false;
  let gameplayOn = null;          // null = never signalled; dedup guard
  let lastAdAt = -Infinity;
  let runsFinished = 0;
  let ctxRef = null;
  let initPromise = null;

  function sdkFeature(name) {
    try { return ysdk && ysdk.features && ysdk.features[name]; } catch (e) { return null; }
  }

  async function resolvePlayer() {
    if (!ysdk) return null;
    if (player) return player;
    try {
      player = await ysdk.getPlayer({ scopes: false });
    } catch (e) {
      player = null;              // not authorized, or the call is unavailable
    }
    return player;
  }

  // ---------------------------------------------------------------------------
  // Init — always resolves, never rejects, never blocks boot for long.
  // ---------------------------------------------------------------------------

  async function init(options = {}) {
    if (initPromise) return initPromise;
    const timeoutMs = options.timeoutMs ?? 3000;

    initPromise = (async () => {
      const fallbackLang = normalizeLang(readCachedLang() || navigator.language);

      if (typeof window === 'undefined' || !window.YaGames || typeof window.YaGames.init !== 'function') {
        lang = fallbackLang;
        ready = true;
        return { sdk: false, lang };
      }

      const sdk = await withTimeout(
        Promise.resolve().then(() => window.YaGames.init()).catch(() => null),
        timeoutMs,
        null,
      );

      ysdk = sdk || null;
      let detected = '';
      try { detected = ysdk?.environment?.i18n?.lang || ''; } catch (e) { detected = ''; }
      lang = detected ? normalizeLang(detected) : fallbackLang;
      if (detected) writeCachedLang(lang);
      ready = true;

      // ready() may have been requested while init was still in flight.
      if (readyPending) markReady();
      return { sdk: !!ysdk, lang };
    })();

    return initPromise;
  }

  // ---------------------------------------------------------------------------
  // Loading / gameplay marks (requirement 1.19)
  // ---------------------------------------------------------------------------

  function markReady() {
    if (readyMarked) return;
    if (!ready) { readyPending = true; return; }
    readyMarked = true;
    readyPending = false;
    try { sdkFeature('LoadingAPI')?.ready(); } catch (e) { /* never fatal */ }
  }

  function gameplayActive(on) {
    const next = !!on;
    if (gameplayOn === next) return;      // only edges reach the SDK
    gameplayOn = next;
    try {
      const api = sdkFeature('GameplayAPI');
      if (next) api?.start(); else api?.stop();
    } catch (e) { /* never fatal */ }
  }

  function isActiveState(state) {
    return ACTIVE_STATES.has(state);
  }

  // ---------------------------------------------------------------------------
  // Interstitial ads
  // ---------------------------------------------------------------------------

  function setAdMute(on) {
    try { ctxRef?.audio?.setExternalMute?.(on); } catch (e) { /* ignore */ }
  }

  /**
   * Called once per finished run, at the moment the descent ends and before the summary
   * goes up — the quietest pause the game has, and the one the player is not waiting on.
   * Shows an interstitial on every AD_EVERY_N_RUNS-th finish, so a session opens with two
   * uninterrupted descents. `onDone` fires exactly once on every path — a failed, blocked
   * or skipped ad can never strand the player on a black screen with no results.
   */
  function maybeShowInterstitial(onDone) {
    const done = typeof onDone === 'function' ? onDone : () => {};
    const now = (typeof performance !== 'undefined' ? performance.now() : 0);
    runsFinished++;
    const due = runsFinished % AD_EVERY_N_RUNS === 0;
    if (!ysdk || !ysdk.adv || !due || now - lastAdAt < AD_MIN_GAP_MS) {
      done();
      return;
    }
    lastAdAt = now;               // claim the slot up front; a retry storm shows one ad

    let finished = false;
    const resume = () => {
      if (finished) return;
      finished = true;
      setAdMute(false);
      if (isActiveState(ctxRef?.gameplay?.state)) gameplayActive(true);
      done();
    };

    try {
      ysdk.adv.showFullscreenAdv({
        callbacks: {
          onOpen: () => { setAdMute(true); gameplayActive(false); },
          onClose: () => resume(),
          onError: () => resume(),
        },
      });
    } catch (e) {
      resume();
    }
  }


  // ---------------------------------------------------------------------------
  // Leaderboard + cloud record
  // ---------------------------------------------------------------------------

  function eligibleSeed(seed) {
    return (seed >>> 0) === (CANONICAL_SEED >>> 0);
  }

  async function submitScore(timeSeconds) {
    if (!ysdk || !isFinite(timeSeconds) || timeSeconds <= 0) return false;
    const p = await resolvePlayer();
    // Anonymous ("lite") players cannot post scores — skip quietly.
    try { if (!p || p.getMode?.() === 'lite') return false; } catch (e) { return false; }
    const ms = Math.round(timeSeconds * 1000);
    try {
      if (ysdk.leaderboards?.setScore) await ysdk.leaderboards.setScore(LEADERBOARD, ms);
      else {
        const lb = await ysdk.getLeaderboards();
        await lb.setLeaderboardScore(LEADERBOARD, ms);
      }
      return true;
    } catch (e) {
      return false;
    }
  }

  async function saveCloudBest(record, seed) {
    if (!ysdk || !record) return false;
    const p = await resolvePlayer();
    if (!p || typeof p.setData !== 'function') return false;
    try {
      await p.setData({
        [CLOUD_KEY]: {
          v: STORE_VERSION, seed: seed >>> 0,
          time: record.time, splits: record.splits || [],
          score: record.score || 0, crashes: record.crashes || 0,
          date: record.date || new Date().toISOString(),
        },
      }, true);
      return true;
    } catch (e) {
      return false;
    }
  }

  async function getCloudBest(seed) {
    if (!ysdk) return null;
    const p = await resolvePlayer();
    if (!p || typeof p.getData !== 'function') return null;
    try {
      const data = await p.getData([CLOUD_KEY]);
      return sanitizeRecord(data && data[CLOUD_KEY], seed);
    } catch (e) {
      return null;
    }
  }

  /**
   * Reconcile the cloud record with the local one. Must run BEFORE gameplay is
   * built — it reads its personal best from localStorage at construction time,
   * so the merge has to land in storage first.
   */
  async function mergeCloudBest(seed, timeoutMs = 2000) {
    if (!ysdk || !eligibleSeed(seed)) return;
    const recordKey = `${STORE_PREFIX}.record.${seed >>> 0}`;

    const cloud = await withTimeout(getCloudBest(seed), timeoutMs, null);

    let local = null;
    try {
      const raw = window.localStorage.getItem(recordKey);
      if (raw) local = sanitizeRecord(JSON.parse(raw), seed);
    } catch (e) { local = null; }

    if (cloud && (!local || cloud.time < local.time)) {
      try {
        window.localStorage.setItem(recordKey, JSON.stringify({
          v: STORE_VERSION, seed: seed >>> 0,
          time: cloud.time, splits: cloud.splits, score: cloud.score,
          crashes: cloud.crashes, date: cloud.date || new Date().toISOString(),
        }));
      } catch (e) { /* quota / private mode */ }
    } else if (local && (!cloud || local.time < cloud.time)) {
      await withTimeout(saveCloudBest(local, seed), timeoutMs, false);
    }
  }

  // ---------------------------------------------------------------------------
  // Event wiring
  // ---------------------------------------------------------------------------

  function bind(ctx) {
    ctxRef = ctx;
    const events = ctx?.events;
    if (!events) return;

    events.on('run:state', (e) => {
      const to = e && e.to;
      gameplayActive(isActiveState(to) && !document.hidden);
    });

    document.addEventListener('visibilitychange', () => {
      if (document.hidden) gameplayActive(false);
      else if (isActiveState(ctxRef?.gameplay?.state)) gameplayActive(true);
    });

    events.on('run:finish', (e) => {
      if (!e || !e.isBest || !e.valid) return;
      if (ctxRef?.gameplay?.freeRide) return;
      if (!eligibleSeed(ctxRef?.seed ?? CANONICAL_SEED)) return;
      submitScore(e.time);
      saveCloudBest(e.best, ctxRef?.seed ?? CANONICAL_SEED);
    });
  }

  return {
    init,
    bind,
    markReady,
    gameplayActive,
    maybeShowInterstitial,
    mergeCloudBest,
    submitScore,
    saveCloudBest,
    getCloudBest,
    get lang() { return lang; },
    get available() { return !!ysdk; },
  };
}
