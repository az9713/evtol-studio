import * as THREE from "three";
import { AIRCRAFT, WEIGHT } from "./config";
import { bodyToWorld, v3, type FlightState } from "./dynamics";
import { MAX_ROTOR_THRUST } from "./control";

// Per-rotor thrust vectors. Length is scaled so one hover-share of thrust
// (weight / n rotors) draws as HOVER_LEN metres — a rotor working harder than
// its hover share is visibly longer, which is the whole point during a manoeuvre.

const HOVER_LEN = 9;
const SCALE = HOVER_LEN / (WEIGHT / AIRCRAFT.hubs.length);

export class ThrustArrows {
  private group = new THREE.Group();
  private arrows: THREE.ArrowHelper[] = [];
  visible = true;

  constructor(scene: THREE.Scene) {
    for (let i = 0; i < AIRCRAFT.hubs.length; i++) {
      const a = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), HOVER_LEN, 0x22d3ee, 2, 1.2);
      this.group.add(a);
      this.arrows.push(a);
    }
    scene.add(this.group);
  }

  toggle() {
    this.visible = !this.visible;
    this.group.visible = this.visible;
  }

  update(s: FlightState) {
    if (!this.visible) return;
    const up = bodyToWorld(s.q, v3(0, 1, 0));
    const dir = new THREE.Vector3(up.x, up.y, up.z);
    AIRCRAFT.hubs.forEach((h, i) => {
      const w = bodyToWorld(s.q, v3(h.x, h.y + 0.5, h.z));
      const a = this.arrows[i];
      a.position.set(s.pos.x + w.x, s.pos.y + w.y, s.pos.z + w.z);
      a.setDirection(dir);
      const len = Math.max(0.1, s.thrust[i] * SCALE);
      a.setLength(len, Math.min(2.2, len * 0.28), Math.min(1.3, len * 0.16));
      // Cyan while loafing, amber as the rotor approaches its thrust limit.
      const load = s.thrust[i] / MAX_ROTOR_THRUST;
      a.setColor(new THREE.Color().setHSL(0.52 - 0.42 * Math.min(1, Math.max(0, (load - 0.4) / 0.5)), 0.85, 0.55));
    });
  }
}
