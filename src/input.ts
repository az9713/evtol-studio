import type { PilotInput } from "./control";

// Keyboard pilot. Sticks ramp toward their target so a digital key still feels
// like an analogue input; the flight-control loops in control.ts do the rest.

const HELD = new Set([
  "KeyW",
  "KeyA",
  "KeyS",
  "KeyD",
  "KeyQ",
  "KeyE",
  "ArrowUp",
  "ArrowDown",
  "ArrowLeft",
  "ArrowRight",
  "Space",
  "ShiftLeft",
  "ShiftRight",
]);

export class Input implements PilotInput {
  pitch = 0;
  roll = 0;
  yaw = 0;
  climb = 0;
  onCameraToggle: (() => void) | null = null;
  onArrowsToggle: (() => void) | null = null;
  onMotorToggle: (() => void) | null = null;
  onReset: (() => void) | null = null;
  private keys = new Set<string>();

  constructor() {
    window.addEventListener("keydown", (e) => {
      if (HELD.has(e.code)) e.preventDefault();
      if (e.repeat) return;
      this.keys.add(e.code);
      if (e.code === "KeyC") this.onCameraToggle?.();
      if (e.code === "KeyT") this.onArrowsToggle?.();
      if (e.code === "KeyM") this.onMotorToggle?.();
      if (e.code === "KeyR") this.onReset?.();
    });
    window.addEventListener("keyup", (e) => this.keys.delete(e.code));
    window.addEventListener("blur", () => this.keys.clear());
  }

  private axis(pos: string[], neg: string[]) {
    return (pos.some((k) => this.keys.has(k)) ? 1 : 0) - (neg.some((k) => this.keys.has(k)) ? 1 : 0);
  }

  update(dt: number) {
    const rate = 4 * dt;
    const toward = (cur: number, target: number) => cur + Math.max(-rate, Math.min(rate, target - cur));
    this.pitch = toward(this.pitch, this.axis(["KeyW", "ArrowUp"], ["KeyS", "ArrowDown"]));
    this.roll = toward(this.roll, this.axis(["KeyD", "ArrowRight"], ["KeyA", "ArrowLeft"]));
    this.yaw = toward(this.yaw, this.axis(["KeyE"], ["KeyQ"]));
    this.climb = toward(this.climb, this.axis(["Space"], ["ShiftLeft", "ShiftRight"]));
  }
}
