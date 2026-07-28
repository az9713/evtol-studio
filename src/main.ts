import * as THREE from "three";
import { AIRCRAFT, N_ROTORS, ROTOR, WEIGHT } from "./config";
import { createCity } from "./city";
import { createEvtol } from "./model";
import { attitudeAngles, createFlightState, stepFlight } from "./dynamics";
import { createAutopilot, hoverTrim, mixRotors } from "./control";
import { Input } from "./input";
import { CameraRig } from "./cameras";
import { ThrustArrows } from "./arrows";

const SPAWN_ALT = 60;
const PHYSICS_DT = 1 / 200; // fixed step; the render loop runs an accumulator over it

const app = document.getElementById("app")!;
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
app.appendChild(renderer.domElement);
window.addEventListener("resize", () => renderer.setSize(window.innerWidth, window.innerHeight));

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x7ba3c9);
scene.fog = new THREE.Fog(0x7ba3c9, 900, 4200);

// Sun: the shadow camera is deliberately small and tracks the aircraft, so the
// shadows that matter (aircraft on the ground, rotors on the roofs) stay crisp.
const sun = new THREE.DirectionalLight(0xfff4e0, 2.4);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
const SHADOW_EXTENT = 90;
sun.shadow.camera.left = -SHADOW_EXTENT;
sun.shadow.camera.right = SHADOW_EXTENT;
sun.shadow.camera.top = SHADOW_EXTENT;
sun.shadow.camera.bottom = -SHADOW_EXTENT;
sun.shadow.camera.near = 1;
sun.shadow.camera.far = 900;
sun.shadow.bias = -0.0006;
scene.add(sun);
scene.add(sun.target);
scene.add(new THREE.HemisphereLight(0xbcd4e8, 0x2b3038, 0.9));

const city = createCity(scene);
const evtol = createEvtol();
scene.add(evtol.root);

const spawn = () => {
  const s = createFlightState(SPAWN_ALT);
  s.pos.x = city.pad.x;
  s.pos.z = city.pad.z;
  // Start already spinning at hover RPM so the first frame is a stable hover.
  s.rpm.fill(hoverTrim().rpm);
  return s;
};

let flight = spawn();
let autopilot = createAutopilot(SPAWN_ALT);

const input = new Input();
const rig = new CameraRig(renderer);
const arrows = new ThrustArrows(scene);
input.onCameraToggle = () => rig.toggle();
input.onArrowsToggle = () => arrows.toggle();
input.onMotorToggle = () => (flight.motorsOn = !flight.motorsOn);
input.onReset = () => {
  flight = spawn();
  autopilot = createAutopilot(SPAWN_ALT);
};

// --- HUD ---
const el = (id: string) => document.getElementById(id)!;
const rotorPanel = el("rotors");
const rotorBars = AIRCRAFT.hubs.map((_, i) => {
  const cell = document.createElement("div");
  cell.className = "r";
  cell.innerHTML = `<div class="n">R${i + 1} ${AIRCRAFT.spin[i] > 0 ? "CCW" : "CW"}</div><div class="v">0</div><div class="w"><div></div></div>`;
  rotorPanel.appendChild(cell);
  return { value: cell.querySelector(".v")!, bar: cell.querySelector(".w div") as HTMLElement };
});

const trim = hoverTrim();
el("spec").textContent =
  `${AIRCRAFT.mass} kg · ${N_ROTORS} rotors · R ${ROTOR.radius} m · ` +
  `disk loading ${(WEIGHT / (N_ROTORS * Math.PI * ROTOR.radius ** 2)).toFixed(0)} N/m²`;
el("power-sub").innerHTML =
  `hover reference ${(trim.electricalPower / 1000).toFixed(0)} kW · FM ${trim.figureOfMerit.toFixed(2)}<br />` +
  `shaft <span id="kw-shaft">0</span> kW · ${(trim.rpm).toFixed(0)} rpm at hover`;

let hudClock = 0;
function updateHud() {
  const att = attitudeAngles(flight.q);
  el("alt").textContent = `${flight.pos.y.toFixed(0)} m`;
  el("gs").textContent = `${(Math.hypot(flight.vel.x, flight.vel.z) * 3.6).toFixed(0)} km/h`;
  el("vs").textContent = `${flight.vel.y >= 0 ? "+" : ""}${flight.vel.y.toFixed(1)} m/s`;
  el("tw").textContent = (flight.totalThrust / WEIGHT).toFixed(2);
  el("att").textContent = `${((att.roll * 180) / Math.PI).toFixed(0)}° roll / ${((att.pitch * 180) / Math.PI).toFixed(0)}° pitch`;
  el("cam").textContent = rig.currentMode;
  el("kw").textContent = (flight.electricalPower / 1000).toFixed(0);
  const shaft = document.getElementById("kw-shaft");
  if (shaft) shaft.textContent = (flight.shaftPower.reduce((a, b) => a + b, 0) / 1000).toFixed(0);
  el("warn").textContent = flight.motorsOn ? (flight.onGround ? "on the pad" : "") : "MOTORS OFF — free fall (M to restore)";
  rotorBars.forEach((r, i) => {
    r.value.textContent = flight.rpm[i].toFixed(0);
    r.bar.style.width = `${Math.min(100, (flight.rpm[i] / ROTOR.maxRpm) * 100).toFixed(0)}%`;
  });
}

// --- loop ---
let last = performance.now();
let accumulator = 0;
function frame(now: number) {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;

  input.update(dt);
  accumulator += dt;
  let steps = 0;
  while (accumulator >= PHYSICS_DT && steps++ < 40) {
    stepFlight(flight, mixRotors(flight, input, autopilot), PHYSICS_DT);
    accumulator -= PHYSICS_DT;
  }

  evtol.update(flight);
  arrows.update(flight);
  rig.update(flight, dt);

  sun.target.position.set(flight.pos.x, 0, flight.pos.z);
  sun.position.set(flight.pos.x + 140, flight.pos.y + 220, flight.pos.z + 90);

  hudClock += dt;
  if (hudClock > 0.1) {
    hudClock = 0;
    updateHud();
  }

  renderer.render(scene, rig.camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
