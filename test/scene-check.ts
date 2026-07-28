import assert from "node:assert";
import * as THREE from "three";
import { AIRCRAFT, N_ROTORS, ROTOR } from "../src/config";
import { createCity } from "../src/city";
import { createEvtol } from "../src/model";
import { ThrustArrows } from "../src/arrows";
import { GROUND_CLEARANCE, createFlightState, stepFlight } from "../src/dynamics";
import { createAutopilot, hoverTrim, mixRotors, type PilotInput } from "../src/control";

// Scene-graph smoke test. Three.js geometry/material/Object3D code is pure JS —
// only WebGLRenderer needs a canvas — so the whole visual layer except the
// renderer and OrbitControls can be built and driven here. Run: npm run check

const DT = 1 / 200;
const STICK: PilotInput = { pitch: 0, roll: 0, yaw: 0, climb: 0 };

// 1. The city builds, is deterministic, and leaves the vertiport plaza clear
{
  const a = new THREE.Scene();
  const b = new THREE.Scene();
  const city = createCity(a, 1234);
  createCity(b, 1234);

  const count = (s: THREE.Scene) => {
    let n = 0;
    s.traverse(() => n++);
    return n;
  };
  assert(count(a) === count(b), "same seed must build the same city");
  assert(count(a) > 30, `city must actually build geometry, got ${count(a)} nodes`);
  assert(city.pad.x === 0 && city.pad.z === 0, "vertiport pad is the origin");

  const instanced = a.children.find((c) => c instanceof THREE.InstancedMesh) as THREE.InstancedMesh;
  assert(instanced && instanced.count > 300, `expected a few hundred buildings, got ${instanced?.count}`);
  assert(instanced.castShadow && instanced.receiveShadow, "buildings must cast and receive shadows");

  // No building may intrude on the pad the aircraft spawns over.
  const m = new THREE.Matrix4();
  const p = new THREE.Vector3();
  const sc = new THREE.Vector3();
  for (let i = 0; i < instanced.count; i++) {
    instanced.getMatrixAt(i, m);
    m.decompose(p, new THREE.Quaternion(), sc);
    assert(Math.hypot(p.x, p.z) > 20, `building ${i} sits on the vertiport at ${p.x.toFixed(1)},${p.z.toFixed(1)}`);
  }
}

// 2. The aircraft model matches the aircraft spec it is supposed to depict
{
  const evtol = createEvtol();
  const meshes: THREE.Mesh[] = [];
  evtol.root.traverse((o) => {
    if (o instanceof THREE.Mesh) meshes.push(o);
  });
  assert(meshes.length > 20, `articulated model needs real parts, got ${meshes.length} meshes`);

  // Blur discs: one per rotor, radius = the rotor radius the physics uses.
  const discs = meshes.filter((o) => o.name === "disc");
  assert(discs.length === N_ROTORS, `expected ${N_ROTORS} rotor discs, got ${discs.length}`);
  const discGeo = discs[0].geometry as THREE.CircleGeometry;
  assert(discGeo.parameters.radius === ROTOR.radius, "rotor disc must be drawn at the modelled radius");

  // Skids must touch the ground exactly when the physics says the aircraft has landed.
  const box = new THREE.Box3().setFromObject(evtol.root);
  assert(
    Math.abs(box.min.y + GROUND_CLEARANCE) < 0.05,
    `model bottom ${box.min.y.toFixed(2)} m must match GROUND_CLEARANCE ${GROUND_CLEARANCE} m`,
  );
  const span = box.max.x - box.min.x;
  assert(span > 2 * ROTOR.radius, `span ${span.toFixed(1)} m must at least cover the rotors`);
}

// 3. Model and arrows track a live flight: rotors spin, pose follows, arrows scale with thrust
{
  const scene = new THREE.Scene();
  const evtol = createEvtol();
  scene.add(evtol.root);
  const arrows = new ThrustArrows(scene);

  const s = createFlightState(80);
  s.rpm.fill(hoverTrim().rpm);
  const ap = createAutopilot(80);

  evtol.update(s);
  const spinner = evtol.root.children.find((c) => c instanceof THREE.Group) as THREE.Group;
  assert(spinner, "model must expose rotor groups");

  const angleBefore = s.spinAngle[0];
  for (let i = 0; i < 200; i++) {
    stepFlight(s, mixRotors(s, STICK, ap), DT);
    evtol.update(s);
    arrows.update(s);
  }
  assert(s.spinAngle[0] !== angleBefore, "rotors must accumulate spin angle");
  assert(s.rpm.every((r) => r > 100), "rotors must be turning at hover RPM");
  assert(
    Math.abs(evtol.root.position.y - s.pos.y) < 1e-6 && Math.abs(evtol.root.position.x - s.pos.x) < 1e-6,
    "model must sit exactly where the physics says",
  );

  // A banked aircraft must tilt its thrust arrows with it.
  const arrowGroup = scene.children.find(
    (c) => c instanceof THREE.Group && c.children.every((k) => k instanceof THREE.ArrowHelper),
  ) as THREE.Group;
  assert(arrowGroup.children.length === N_ROTORS, `expected ${N_ROTORS} thrust arrows`);

  const roll: PilotInput = { pitch: 0, roll: 1, yaw: 0, climb: 0 };
  for (let i = 0; i < 600; i++) {
    stepFlight(s, mixRotors(s, roll, ap), DT);
    evtol.update(s);
    arrows.update(s);
  }
  // ArrowHelper points along its own local +y, so read that, not getWorldDirection's +z.
  const q = new THREE.Quaternion();
  (arrowGroup.children[0] as THREE.ArrowHelper).getWorldQuaternion(q);
  const dir = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
  assert(
    dir.y > 0.8 && dir.x > 0.05,
    `banking right must tilt the thrust arrows toward +x, got (${dir.x.toFixed(3)}, ${dir.y.toFixed(3)})`,
  );
  assert(
    new Set(s.thrust.map((t) => t.toFixed(1))).size > 1,
    "rolling must produce differential thrust across the rotors",
  );
}

// 4. Rotor blade chord is the one the solidity in config.ts implies
{
  const evtol = createEvtol();
  let blade: THREE.Mesh | undefined;
  evtol.root.traverse((o) => {
    if (o instanceof THREE.Mesh && o.geometry instanceof THREE.BoxGeometry && o.geometry.parameters.height === 0.07) {
      blade = o;
    }
  });
  assert(blade, "model must contain rotor blades");
  const chord = (blade.geometry as THREE.BoxGeometry).parameters.depth;
  const expected = (ROTOR.solidity * Math.PI * ROTOR.radius) / 3; // sigma = Nb*c/(pi*R), 3 blades
  assert(Math.abs(chord - expected) < 1e-9, `blade chord ${chord.toFixed(3)} m must match solidity ${ROTOR.solidity}`);
}

// 5. Hub layout in the model is the hub layout in the physics
{
  assert(AIRCRAFT.hubs.length === N_ROTORS, "hub count must match rotor count");
  const evtol = createEvtol();
  const box = new THREE.Box3().setFromObject(evtol.root);
  const reach = Math.max(...AIRCRAFT.hubs.map((h) => Math.abs(h.x))) + ROTOR.radius;
  assert(Math.abs(box.max.x - reach) < 0.6, `model reach ${box.max.x.toFixed(2)} m must match hub layout ${reach} m`);
}

console.log("scene-check: all assertions passed");
