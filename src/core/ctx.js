// Shared game context. Owned by the integration layer — implementers must NOT edit this.
// Every system reads from and writes to this single object.

import * as THREE from 'three';

const QUALITY_PRESETS = {
  low: {
    shadows: true, shadowMapSize: 1024, cascades: 1,
    terrainLOD: 0.6, vegetationDensity: 0.35, grass: false,
    ao: false, bloom: true, dof: false, motionBlur: false, godRays: false,
    filmGrain: false, chromaticAberration: false, ssr: false,
    antialias: 'none', volumetricClouds: false, particleDensity: 0.4,
  },
  medium: {
    shadows: true, shadowMapSize: 2048, cascades: 2,
    terrainLOD: 0.85, vegetationDensity: 0.7, grass: true,
    ao: true, bloom: true, dof: false, motionBlur: true, godRays: false,
    filmGrain: true, chromaticAberration: false, ssr: false,
    antialias: 'smaa', volumetricClouds: true, particleDensity: 0.7,
  },
  high: {
    shadows: true, shadowMapSize: 2048, cascades: 3,
    terrainLOD: 1.0, vegetationDensity: 1.0, grass: true,
    ao: true, bloom: true, dof: true, motionBlur: true, godRays: true,
    filmGrain: true, chromaticAberration: true, ssr: false,
    antialias: 'smaa', volumetricClouds: true, particleDensity: 1.0,
  },
  ultra: {
    shadows: true, shadowMapSize: 4096, cascades: 4,
    terrainLOD: 1.35, vegetationDensity: 1.4, grass: true,
    ao: true, bloom: true, dof: true, motionBlur: true, godRays: true,
    filmGrain: true, chromaticAberration: true, ssr: true,
    antialias: 'smaa', volumetricClouds: true, particleDensity: 1.4,
  },
};

function createEvents() {
  const map = new Map();
  return {
    on(name, fn) {
      if (!map.has(name)) map.set(name, new Set());
      map.get(name).add(fn);
      return () => this.off(name, fn);
    },
    off(name, fn) {
      const s = map.get(name);
      if (s) s.delete(fn);
    },
    emit(name, payload) {
      const s = map.get(name);
      if (!s) return;
      for (const fn of s) {
        try { fn(payload); } catch (e) { console.error(`[events] ${name}`, e); }
      }
    },
  };
}

export function createContext(container, options = {}) {
  const quality = options.quality || 'high';
  const settings = {
    ...QUALITY_PRESETS[quality],
    pixelRatio: Math.min(window.devicePixelRatio || 1, 2),
  };

  const ctx = {
    container,
    renderer: null,
    scene: null,
    camera: null,
    sun: null,
    clock: new THREE.Clock(),
    time: 0,
    dt: 0,
    frame: 0,
    seed: options.seed ?? 20260726,
    quality,
    settings,
    input: null,
    events: createEvents(),

    terrain: null, trail: null, sky: null, water: null, vegetation: null,
    collision: null, bike: null, bikeModel: null, rider: null, particles: null,
    chaseCamera: null, audio: null, gameplay: null, hud: null, menu: null,
    postfx: null,

    debug: {
      enabled: new URLSearchParams(location.search).has('debug'),
      log(...args) { if (ctx.debug.enabled) console.log('[dbg]', ...args); },
    },

    setQuality(q) {
      if (!QUALITY_PRESETS[q]) return;
      ctx.quality = q;
      Object.assign(ctx.settings, QUALITY_PRESETS[q]);
      ctx.events.emit('quality:changed', q);
    },
  };

  return ctx;
}

export { QUALITY_PRESETS };
