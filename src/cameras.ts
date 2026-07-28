import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { attitudeAngles, type FlightState } from "./dynamics";

export type CameraMode = "chase" | "free";
const MODES: CameraMode[] = ["chase", "free"];

export class CameraRig {
  readonly camera: THREE.PerspectiveCamera;
  private mode: CameraMode = "chase";
  private orbit: OrbitControls;

  constructor(renderer: THREE.WebGLRenderer) {
    this.camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.5, 12000);
    this.camera.position.set(0, 90, 60);
    this.orbit = new OrbitControls(this.camera, renderer.domElement);
    this.orbit.enabled = false;
    window.addEventListener("resize", () => {
      this.camera.aspect = window.innerWidth / window.innerHeight;
      this.camera.updateProjectionMatrix();
    });
  }

  toggle() {
    this.mode = MODES[(MODES.indexOf(this.mode) + 1) % MODES.length];
    this.orbit.enabled = this.mode === "free";
  }

  get currentMode(): CameraMode {
    return this.mode;
  }

  update(s: FlightState, dt: number) {
    const pos = new THREE.Vector3(s.pos.x, s.pos.y, s.pos.z);
    if (this.mode === "chase") {
      // Follow the heading only — inheriting bank and pitch would make the horizon swim.
      const yaw = attitudeAngles(s.q).yaw;
      // Sit 34 m behind: forward is (sin yaw, 0, -cos yaw), so back is (-sin yaw, 0, cos yaw).
      const target = pos
        .clone()
        .add(new THREE.Vector3(-Math.sin(yaw) * 34, 12, Math.cos(yaw) * 34));
      target.y = Math.max(target.y, 8);
      // Exponential smoothing, frame-rate independent
      this.camera.position.lerp(target, 1 - Math.exp(-3 * dt));
      this.camera.lookAt(pos);
    } else {
      this.orbit.target.copy(pos);
      this.orbit.update();
    }
  }
}
