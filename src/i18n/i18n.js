// Minimal synchronous i18n.
//
// The language is resolved during boot — before any UI factory runs — so lookups
// never need to be reactive: t() is called at build time for static labels and at
// call time for runtime banners, and both see the final language.
//
// Keys that are missing from a dictionary fall through to English, then to the
// key itself, so a half-translated build degrades to readable text rather than
// blank UI.

import { STRINGS, DEFAULT_LANG } from './strings.js';

let lang = DEFAULT_LANG;
let dict = STRINGS[DEFAULT_LANG];

/** Normalise a BCP-47-ish tag onto a dictionary we actually ship. */
export function pickLang(raw) {
  const s = String(raw || '').toLowerCase();
  if (s.startsWith('ru')) return 'ru';
  return DEFAULT_LANG;
}

export function setLang(raw) {
  lang = pickLang(raw);
  dict = STRINGS[lang] || STRINGS[DEFAULT_LANG];
  try { document.documentElement.lang = lang; } catch (e) { /* ignore */ }
  return lang;
}

export function getLang() { return lang; }

/**
 * Look up `key`, interpolating {0}, {1}, … from the remaining arguments.
 * t('hud.split', 3) → 'Split 3'
 */
export function t(key, ...args) {
  let s = dict[key];
  if (s === undefined) s = STRINGS[DEFAULT_LANG][key];
  if (s === undefined) return key;
  if (!args.length) return s;
  return s.replace(/\{(\d+)\}/g, (m, i) => {
    const v = args[Number(i)];
    return v === undefined ? m : String(v);
  });
}
