// input.js — keyboard + Gamepad, per CONTRACT §7.
// Consumers only ever read ctx.input.state. Analogue axes are smoothed towards
// their target so keyboard control does not feel like an on/off switch, while
// gamepad axes pass through at full fidelity.

import { clamp, clamp01, damp } from '../core/rng.js';

const DEADZONE = 0.16;
const TRIGGER_DEADZONE = 0.06;
// Keyboard ramp rates (units/second towards target). Attack faster than release
// so inputs feel responsive but do not snap back and unsettle the bike.
const AXIS_ATTACK = 9.0;
const AXIS_RELEASE = 13.0;
const TRIGGER_ATTACK = 7.5;
const TRIGGER_RELEASE = 11.0;

/** Radial deadzone with rescale, so the usable range still reaches 1.0. */
function applyDeadzone(v, dz) {
  const a = Math.abs(v);
  if (a <= dz) return 0;
  return Math.sign(v) * ((a - dz) / (1 - dz));
}

export function createInput(ctx) {
  const state = {
    steer: 0, pitch: 0, roll: 0,
    brakeFront: 0, brakeRear: 0,
    pedal: 0, pump: 0,
    manual: false,
    reset: false, pause: false, cameraCycle: false, photoMode: false,
    anyPressed: false,
  };

  // Held-key set and edge-triggered set (consumed once per frame).
  const down = new Set();
  const pressedEdge = new Set();
  let gamepadIndex = -1;
  let lastGamepadActivity = -Infinity;

  const isTyping = () => {
    const el = document.activeElement;
    if (!el) return false;
    const tag = el.tagName;
    return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || el.isContentEditable;
  };

  function onKeyDown(e) {
    if (isTyping()) return;
    const code = e.code;
    if (!down.has(code)) pressedEdge.add(code);
    down.add(code);
    // Stop the page scrolling / the browser eating our controls.
    if (code === 'Space' || code.startsWith('Arrow') || code === 'Tab') e.preventDefault();
  }
  function onKeyUp(e) {
    down.delete(e.code);
  }
  function onBlur() {
    // Never leave a key stuck on when focus leaves the window.
    down.clear();
    pressedEdge.clear();
  }
  function onGamepadConnected(e) {
    gamepadIndex = e.gamepad.index;
  }
  function onGamepadDisconnected(e) {
    if (gamepadIndex === e.gamepad.index) gamepadIndex = -1;
  }

  window.addEventListener('keydown', onKeyDown, { passive: false });
  window.addEventListener('keyup', onKeyUp);
  window.addEventListener('blur', onBlur);
  window.addEventListener('gamepadconnected', onGamepadConnected);
  window.addEventListener('gamepaddisconnected', onGamepadDisconnected);

  const held = (...codes) => codes.some((c) => down.has(c));
  const tapped = (...codes) => codes.some((c) => pressedEdge.has(c));

  function pollGamepad() {
    if (!navigator.getGamepads) return null;
    const pads = navigator.getGamepads();
    if (!pads) return null;
    if (gamepadIndex >= 0 && pads[gamepadIndex]) return pads[gamepadIndex];
    for (let i = 0; i < pads.length; i++) {
      if (pads[i] && pads[i].connected) { gamepadIndex = i; return pads[i]; }
    }
    return null;
  }

  // Previous button states for gamepad edge detection.
  const prevButtons = new Uint8Array(24);

  function update(dt) {
    const d = Math.min(dt || 0, 1 / 20);

    // ---- keyboard targets -------------------------------------------------
    let steerT = (held('KeyD', 'ArrowRight') ? 1 : 0) - (held('KeyA', 'ArrowLeft') ? 1 : 0);
    let pitchT = (held('KeyW', 'ArrowUp') ? 1 : 0) - (held('KeyS', 'ArrowDown') ? 1 : 0);
    let rollT = (held('KeyE') ? 1 : 0) - (held('KeyQ') ? 1 : 0);
    let brakeFT = held('KeyJ') ? 1 : 0;
    let brakeRT = held('KeyK') ? 1 : 0;
    let pedalT = held('ShiftLeft', 'ShiftRight') ? 1 : 0;
    let pumpT = held('Space') ? 1 : 0;
    let manual = held('KeyM');

    let reset = tapped('KeyR');
    let pause = tapped('Escape', 'KeyP');
    let cameraCycle = tapped('KeyC');
    let photoMode = tapped('KeyF');
    let anyKeyboard = down.size > 0;

    // ---- gamepad overrides / merges --------------------------------------
    const pad = pollGamepad();
    let padActive = false;
    if (pad) {
      const ax = pad.axes || [];
      const gSteer = applyDeadzone(ax[0] || 0, DEADZONE);
      // Stick Y is +down on every mainstream pad; contract pitch is + = back/up.
      const gPitch = applyDeadzone(-(ax[1] || 0), DEADZONE);
      const gRoll = applyDeadzone(ax[2] || 0, DEADZONE);

      const btn = (i) => (pad.buttons && pad.buttons[i]) || null;
      const val = (i) => { const b = btn(i); return b ? (b.value !== undefined ? b.value : (b.pressed ? 1 : 0)) : 0; };
      const pressed = (i) => { const b = btn(i); return !!(b && b.pressed); };

      // Standard mapping: 6 = L2, 7 = R2, 4 = L1, 5 = R1, 0 = A/X, 1 = B/O,
      // 2 = X/□, 3 = Y/△, 9 = Start, 8 = Select, 10 = L3.
      let gBrakeF = val(6);
      let gBrakeR = val(7);
      if (gBrakeF < TRIGGER_DEADZONE) gBrakeF = 0;
      if (gBrakeR < TRIGGER_DEADZONE) gBrakeR = 0;
      const gPedal = Math.max(val(5), pressed(5) ? 1 : 0);
      const gPump = pressed(0) ? 1 : 0;

      padActive = Math.abs(gSteer) > 0 || Math.abs(gPitch) > 0 || Math.abs(gRoll) > 0 ||
                  gBrakeF > 0 || gBrakeR > 0 || gPedal > 0 || gPump > 0;
      if (padActive) lastGamepadActivity = ctx.time;

      // Take whichever device is asking for more — lets you mix devices without
      // one zeroing the other out.
      if (Math.abs(gSteer) > Math.abs(steerT)) steerT = gSteer;
      if (Math.abs(gPitch) > Math.abs(pitchT)) pitchT = gPitch;
      if (Math.abs(gRoll) > Math.abs(rollT)) rollT = gRoll;
      brakeFT = Math.max(brakeFT, gBrakeF);
      brakeRT = Math.max(brakeRT, gBrakeR);
      pedalT = Math.max(pedalT, gPedal);
      pumpT = Math.max(pumpT, gPump);
      manual = manual || pressed(2);

      // Edge-detect the discrete buttons.
      const n = Math.min(prevButtons.length, (pad.buttons || []).length);
      for (let i = 0; i < n; i++) {
        const now = pressed(i) ? 1 : 0;
        const edge = now && !prevButtons[i];
        if (edge) {
          if (i === 1) reset = true;         // B / circle
          if (i === 9) pause = true;         // start
          if (i === 3) cameraCycle = true;   // Y / triangle
          if (i === 8) photoMode = true;     // select / share
        }
        prevButtons[i] = now;
      }
    }

    // Gamepad axes are already analogue, so only smooth when it is keyboard-led.
    const smoothing = padActive ? 1 : 0;
    const rate = (target, current, attack, release) => {
      if (smoothing) return target;
      const r = Math.abs(target) > Math.abs(current) ? attack : release;
      return damp(current, target, r, d);
    };

    state.steer = clamp(rate(steerT, state.steer, AXIS_ATTACK, AXIS_RELEASE), -1, 1);
    state.pitch = clamp(rate(pitchT, state.pitch, AXIS_ATTACK, AXIS_RELEASE), -1, 1);
    state.roll = clamp(rate(rollT, state.roll, AXIS_ATTACK, AXIS_RELEASE), -1, 1);
    state.brakeFront = clamp01(rate(brakeFT, state.brakeFront, TRIGGER_ATTACK, TRIGGER_RELEASE));
    state.brakeRear = clamp01(rate(brakeRT, state.brakeRear, TRIGGER_ATTACK, TRIGGER_RELEASE));
    state.pedal = clamp01(rate(pedalT, state.pedal, TRIGGER_ATTACK, TRIGGER_RELEASE));
    // Pump is deliberately un-smoothed: the release edge is the whole mechanic.
    state.pump = clamp01(pumpT);

    state.manual = !!manual;
    state.reset = reset;
    state.pause = pause;
    state.cameraCycle = cameraCycle;
    state.photoMode = photoMode;
    state.anyPressed = anyKeyboard || padActive;

    pressedEdge.clear();
  }

  return {
    state,
    update,

    /** Impact rumble, if the pad supports it. Safe to call every frame. */
    rumble(strength = 0.5, durationMs = 120) {
      const pad = pollGamepad();
      const act = pad && (pad.vibrationActuator ||
        (pad.hapticActuators && pad.hapticActuators[0]));
      if (!act || typeof act.playEffect !== 'function') return;
      const s = clamp01(strength);
      try {
        act.playEffect('dual-rumble', {
          startDelay: 0,
          duration: Math.max(20, durationMs),
          weakMagnitude: s * 0.8,
          strongMagnitude: s,
        });
      } catch (e) { /* pad does not support it; ignore */ }
    },

    get hasGamepad() { return gamepadIndex >= 0; },
    get lastGamepadActivity() { return lastGamepadActivity; },

    dispose() {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      window.removeEventListener('blur', onBlur);
      window.removeEventListener('gamepadconnected', onGamepadConnected);
      window.removeEventListener('gamepaddisconnected', onGamepadDisconnected);
    },
  };
}
