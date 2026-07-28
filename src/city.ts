import * as THREE from "three";

// Procedural box city on a flat ground plane. Deterministic from a seed so the
// skyline is the same every reload and screenshots are comparable.
//
// ponytail: no OSM, no heightmap, no window textures — one InstancedMesh of
// boxes with per-instance colour draws the whole city in a single draw call.

const BLOCK = 78; // m, street-to-street pitch
const STREET = 22; // m, roadway width between blocks
const GRID = 13; // blocks per side (odd, so one block is dead centre)
export const CITY_EXTENT = (GRID * BLOCK) / 2;

/** Deterministic PRNG (mulberry32). */
function rng(seed: number) {
  return () => {
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const PALETTE = [0x3d4654, 0x4a5262, 0x2f3743, 0x555f70, 0x39424f, 0x606b7d];

export interface City {
  /** Vertiport pad centre, where the aircraft spawns. */
  pad: THREE.Vector3;
}

export function createCity(scene: THREE.Scene, seed = 20260727): City {
  const rand = rng(seed);

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(CITY_EXTENT * 6, CITY_EXTENT * 6),
    new THREE.MeshStandardMaterial({ color: 0x232a33, roughness: 1 }),
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  // Streets: light strips laid in the gaps between blocks.
  const roadMat = new THREE.MeshStandardMaterial({ color: 0x171c23, roughness: 1 });
  const span = GRID * BLOCK;
  for (let i = 0; i <= GRID; i++) {
    const c = -span / 2 + i * BLOCK;
    for (const axis of [0, 1]) {
      const road = new THREE.Mesh(new THREE.PlaneGeometry(axis ? span : STREET, axis ? STREET : span), roadMat);
      road.rotation.x = -Math.PI / 2;
      road.position.set(axis ? 0 : c, 0.04, axis ? c : 0);
      road.receiveShadow = true;
      scene.add(road);
    }
  }

  // Buildings: 1-4 per block, taller toward the centre (a downtown core).
  const boxes: { m: THREE.Matrix4; c: THREE.Color }[] = [];
  const buildable = BLOCK - STREET;
  for (let bx = 0; bx < GRID; bx++) {
    for (let bz = 0; bz < GRID; bz++) {
      const cx = -span / 2 + (bx + 0.5) * BLOCK;
      const cz = -span / 2 + (bz + 0.5) * BLOCK;
      if (Math.abs(cx) < BLOCK / 2 && Math.abs(cz) < BLOCK / 2) continue; // keep the vertiport plaza clear

      const downtown = Math.max(0, 1 - Math.hypot(cx, cz) / (span * 0.42));
      const lots = 1 + Math.floor(rand() * 4);
      for (let l = 0; l < lots; l++) {
        const w = buildable * (0.28 + rand() * 0.36);
        const d = buildable * (0.28 + rand() * 0.36);
        const tall = rand() < 0.12 + downtown * 0.5;
        const h = (10 + rand() * 45) * (0.5 + downtown * 1.4) * (tall ? 2.4 : 1);
        const x = cx + (rand() - 0.5) * (buildable - w);
        const z = cz + (rand() - 0.5) * (buildable - d);
        boxes.push({
          m: new THREE.Matrix4().compose(
            new THREE.Vector3(x, h / 2, z),
            new THREE.Quaternion(),
            new THREE.Vector3(w, h, d),
          ),
          c: new THREE.Color(PALETTE[Math.floor(rand() * PALETTE.length)]),
        });
      }
    }
  }

  const mesh = new THREE.InstancedMesh(
    new THREE.BoxGeometry(1, 1, 1),
    new THREE.MeshStandardMaterial({ roughness: 0.85, metalness: 0.05 }),
    boxes.length,
  );
  boxes.forEach((b, i) => {
    mesh.setMatrixAt(i, b.m);
    mesh.setColorAt(i, b.c);
  });
  mesh.instanceMatrix.needsUpdate = true;
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  scene.add(mesh);

  // Vertiport: a lit pad in the central plaza.
  const pad = new THREE.Mesh(
    new THREE.CylinderGeometry(16, 16, 0.3, 40),
    new THREE.MeshStandardMaterial({ color: 0x2b333f, roughness: 0.9 }),
  );
  pad.position.set(0, 0.15, 0);
  pad.receiveShadow = true;
  scene.add(pad);

  const ring = new THREE.Mesh(
    new THREE.RingGeometry(12.5, 14, 48),
    new THREE.MeshBasicMaterial({ color: 0x22d3ee, side: THREE.DoubleSide }),
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = 0.32;
  scene.add(ring);

  return { pad: new THREE.Vector3(0, 0.3, 0) };
}
