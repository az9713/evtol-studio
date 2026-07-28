import * as THREE from "three";
import { AIRCRAFT, ROTOR } from "./config";
import { GROUND_CLEARANCE, type FlightState } from "./dynamics";

// Articulated eVTOL built from primitives: fuselage, booms, and one spinning
// rotor group per hub. Blade count and chord are the same numbers the solidity
// in config.ts implies, so what you see is what the BEM model is solving.
//
// ponytail: primitives only, no GLTF asset. Every dimension comes from the
// aircraft spec rather than being art-directed.

const BLADES = 3;
const CHORD = (ROTOR.solidity * Math.PI * ROTOR.radius) / BLADES; // sigma = Nb*c/(pi*R)

const BODY = new THREE.MeshStandardMaterial({ color: 0xe3e9f2, metalness: 0.25, roughness: 0.5 });
const DARK = new THREE.MeshStandardMaterial({ color: 0x1b2430, metalness: 0.4, roughness: 0.4 });
const ACCENT = new THREE.MeshStandardMaterial({ color: 0x22d3ee, metalness: 0.3, roughness: 0.35 });
const GLASS = new THREE.MeshStandardMaterial({
  color: 0x0d1620,
  metalness: 0.9,
  roughness: 0.1,
  transparent: true,
  opacity: 0.85,
});

export interface EvtolModel {
  root: THREE.Group;
  update(s: FlightState): void;
}

function shadowed(mesh: THREE.Mesh): THREE.Mesh {
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

export function createEvtol(): EvtolModel {
  const root = new THREE.Group();

  // --- fuselage: a 7 m capsule-ish body with a glass nose ---
  const hull = shadowed(new THREE.Mesh(new THREE.CapsuleGeometry(1.25, 4.2, 8, 20), BODY));
  hull.rotation.x = Math.PI / 2;
  hull.scale.set(1, 1, 0.85);
  root.add(hull);

  const nose = shadowed(new THREE.Mesh(new THREE.SphereGeometry(1.2, 20, 14), GLASS));
  nose.position.set(0, 0.15, -2.6);
  nose.scale.set(0.95, 0.8, 1.3);
  root.add(nose);

  const fin = shadowed(new THREE.Mesh(new THREE.BoxGeometry(0.16, 1.7, 1.9), ACCENT));
  fin.position.set(0, 1.1, 3.0);
  root.add(fin);

  // --- skids: bottom sits exactly at the ground-contact height used by the physics ---
  for (const side of [-1, 1]) {
    const skid = shadowed(new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.22, 5.2), DARK));
    skid.position.set(side * 1.3, -(GROUND_CLEARANCE - 0.11), 0.2);
    root.add(skid);
    for (const z of [-1.6, 1.9]) {
      const leg = shadowed(new THREE.Mesh(new THREE.BoxGeometry(0.14, 1.0, 0.16), DARK));
      leg.position.set(side * 1.3, -(GROUND_CLEARANCE - 0.72), z);
      root.add(leg);
    }
  }

  // --- booms + rotors ---
  const spinners: THREE.Group[] = [];
  const bladeGeo = new THREE.BoxGeometry(ROTOR.radius - 0.5, 0.07, CHORD);
  const discGeo = new THREE.CircleGeometry(ROTOR.radius, 48);

  AIRCRAFT.hubs.forEach((h, i) => {
    const dist = Math.hypot(h.x, h.z);
    const boom = shadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.3, dist, 12), BODY));
    boom.position.set(h.x / 2, h.y * 0.6, h.z / 2);
    // Lay the cylinder along the centre -> hub direction
    boom.quaternion.setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(h.x, h.y * 0.8, h.z).normalize(),
    );
    root.add(boom);

    const nacelle = shadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.62, 1.1, 16), DARK));
    nacelle.position.set(h.x, h.y - 0.2, h.z);
    root.add(nacelle);

    const spinner = new THREE.Group();
    spinner.position.set(h.x, h.y + 0.45, h.z);
    root.add(spinner);
    spinners.push(spinner);

    const hubCap = shadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.38, 0.38, 0.3, 14), ACCENT));
    spinner.add(hubCap);

    for (let b = 0; b < BLADES; b++) {
      const blade = shadowed(new THREE.Mesh(bladeGeo, i % 2 === 0 ? BODY : DARK));
      blade.position.x = (ROTOR.radius - 0.5) / 2 + 0.3;
      blade.rotation.z = ROTOR.collective; // the collective the rotor model is actually using
      const arm = new THREE.Group();
      arm.rotation.y = (b / BLADES) * Math.PI * 2;
      arm.add(blade);
      spinner.add(arm);
    }

    // Blur disc: fades in with RPM so a spinning rotor reads as a disc, like the real thing.
    const disc = new THREE.Mesh(
      discGeo,
      new THREE.MeshBasicMaterial({
        color: 0x9fb4cc,
        transparent: true,
        opacity: 0,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    );
    disc.rotation.x = -Math.PI / 2;
    disc.position.y = 0.02;
    disc.name = "disc";
    spinner.add(disc);
  });

  return {
    root,
    update(s: FlightState) {
      root.position.set(s.pos.x, s.pos.y, s.pos.z);
      root.quaternion.set(s.q[1], s.q[2], s.q[3], s.q[0]);
      spinners.forEach((sp, i) => {
        sp.rotation.y = s.spinAngle[i];
        const disc = sp.getObjectByName("disc") as THREE.Mesh | undefined;
        if (disc) {
          (disc.material as THREE.MeshBasicMaterial).opacity = Math.min(0.3, (s.rpm[i] / ROTOR.maxRpm) * 0.34);
        }
      });
    },
  };
}
