import { AIRCRAFT, G, N_ROTORS, RHO, ROTOR } from "./config";
import { rotorPower, rotorThrust, rpmForThrust } from "./rotor";

// 6-DOF rigid body. Body frame: +x right, +y up, +z aft (so forward is -z).
// World frame is the same convention, y up, which is also Three.js's.
//
// ponytail: 20 lines of vector/quaternion helpers instead of pulling three into
// the physics — keeps this module importable by the node test harness.

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}
/** Body -> world rotation, [w, x, y, z]. */
export type Quat = [number, number, number, number];

export const v3 = (x = 0, y = 0, z = 0): Vec3 => ({ x, y, z });
export const len = (a: Vec3) => Math.hypot(a.x, a.y, a.z);

/** Rotate v from body frame into world frame. */
export function bodyToWorld(q: Quat, v: Vec3): Vec3 {
  const [w, x, y, z] = q;
  // t = 2 * (q_vec x v);  v' = v + w*t + q_vec x t
  const tx = 2 * (y * v.z - z * v.y);
  const ty = 2 * (z * v.x - x * v.z);
  const tz = 2 * (x * v.y - y * v.x);
  return {
    x: v.x + w * tx + (y * tz - z * ty),
    y: v.y + w * ty + (z * tx - x * tz),
    z: v.z + w * tz + (x * ty - y * tx),
  };
}

/** Rotate v from world frame into body frame (inverse rotation). */
export const worldToBody = (q: Quat, v: Vec3): Vec3 => bodyToWorld([q[0], -q[1], -q[2], -q[3]], v);

function normalizeQuat(q: Quat): Quat {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

/** Integrate attitude: q' = q + 0.5 * q (x) (0, omega_body) * dt. */
function integrateQuat(q: Quat, w: Vec3, dt: number): Quat {
  const [qw, qx, qy, qz] = q;
  const h = 0.5 * dt;
  return normalizeQuat([
    qw + h * (-qx * w.x - qy * w.y - qz * w.z),
    qx + h * (qw * w.x + qy * w.z - qz * w.y),
    qy + h * (qw * w.y + qz * w.x - qx * w.z),
    qz + h * (qw * w.z + qx * w.y - qy * w.x),
  ]);
}

export interface FlightState {
  pos: Vec3; // world, y = altitude of the CG
  vel: Vec3; // world m/s
  q: Quat;
  rate: Vec3; // body angular velocity, rad/s
  rpm: number[]; // actual rotor RPM (lagged behind the command)
  thrust: number[]; // N per rotor
  shaftPower: number[]; // W per rotor
  electricalPower: number; // W, whole aircraft
  totalThrust: number; // N
  spinAngle: number[]; // rad, for drawing the blades
  onGround: boolean;
  motorsOn: boolean;
}

/** CG height when the skids are on the ground. */
export const GROUND_CLEARANCE = 1.8;

export function createFlightState(altitude = 60): FlightState {
  const zeros = () => new Array<number>(N_ROTORS).fill(0);
  return {
    pos: v3(0, Math.max(altitude, GROUND_CLEARANCE), 0),
    vel: v3(),
    q: [1, 0, 0, 0],
    rate: v3(),
    rpm: zeros(),
    thrust: zeros(),
    shaftPower: zeros(),
    electricalPower: 0,
    totalThrust: 0,
    spinAngle: zeros(),
    onGround: false,
    motorsOn: true,
  };
}

/** Bank angle (rotation about the fore-aft axis) and nose-up angle, from world-up seen in body axes. */
export function attitudeAngles(q: Quat): { roll: number; pitch: number; yaw: number } {
  const up = worldToBody(q, v3(0, 1, 0));
  const fwd = bodyToWorld(q, v3(0, 0, -1));
  return {
    roll: Math.atan2(up.x, up.y),
    pitch: Math.atan2(-up.z, up.y),
    yaw: Math.atan2(fwd.x, -fwd.z),
  };
}

/**
 * Advance one fixed step. `demand` is the thrust each rotor is asked for (N);
 * the motors chase it through a first-order lag, and the aerodynamics are then
 * evaluated at the RPM the rotor actually reached.
 */
export function stepFlight(s: FlightState, demand: number[], dt: number): void {
  const lag = 1 - Math.exp(-dt / ROTOR.spinLag);
  const up = bodyToWorld(s.q, v3(0, 1, 0));
  const axial = s.vel.x * up.x + s.vel.y * up.y + s.vel.z * up.z; // climb rate through the disks

  let T = 0;
  const M = v3(); // body-frame moment
  let electrical = 0;

  for (let i = 0; i < N_ROTORS; i++) {
    const cmd = s.motorsOn ? rpmForThrust(Math.max(0, demand[i])) : 0;
    s.rpm[i] += (cmd - s.rpm[i]) * lag;
    s.spinAngle[i] = (s.spinAngle[i] + AIRCRAFT.spin[i] * ((s.rpm[i] * Math.PI) / 30) * dt) % (Math.PI * 2);

    const t = rotorThrust(s.rpm[i]);
    const p = rotorPower(t, s.rpm[i], axial);
    s.thrust[i] = t;
    s.shaftPower[i] = p.shaft;
    electrical += p.shaft / AIRCRAFT.driveEfficiency;
    T += t;

    // r x F with F = (0, T, 0) in body axes gives (-z*T, 0, x*T)
    const h = AIRCRAFT.hubs[i];
    M.x += -h.z * t;
    M.z += h.x * t;
    // Shaft torque reacts into the airframe with the opposite sign to the spin.
    M.y += -AIRCRAFT.spin[i] * p.torque;
  }
  s.totalThrust = T;
  s.electricalPower = electrical;

  // --- linear ---
  const speed = len(s.vel);
  const dragK = 0.5 * RHO * AIRCRAFT.dragArea * speed;
  const acc = v3(
    (up.x * T - dragK * s.vel.x) / AIRCRAFT.mass,
    (up.y * T - dragK * s.vel.y) / AIRCRAFT.mass - G,
    (up.z * T - dragK * s.vel.z) / AIRCRAFT.mass,
  );
  s.vel.x += acc.x * dt;
  s.vel.y += acc.y * dt;
  s.vel.z += acc.z * dt;

  // --- angular: Euler's equations, diagonal inertia ---
  const I = AIRCRAFT.inertia;
  const w = s.rate;
  // ponytail: rotor gyroscopic and inflow-damping terms omitted; the attitude PD
  // dominates the response at these rates anyway.
  s.rate.x += ((M.x - (I.z - I.y) * w.y * w.z) / I.x) * dt;
  s.rate.y += ((M.y - (I.x - I.z) * w.z * w.x) / I.y) * dt;
  s.rate.z += ((M.z - (I.y - I.x) * w.x * w.y) / I.z) * dt;

  s.q = integrateQuat(s.q, s.rate, dt);

  s.pos.x += s.vel.x * dt;
  s.pos.y += s.vel.y * dt;
  s.pos.z += s.vel.z * dt;

  // --- ground ---
  // ponytail: inelastic stop on the skids, no gear spring and no tipping check.
  s.onGround = s.pos.y <= GROUND_CLEARANCE;
  if (s.onGround) {
    s.pos.y = GROUND_CLEARANCE;
    if (s.vel.y < 0) s.vel.y = 0;
    const friction = Math.exp(-4 * dt);
    s.vel.x *= friction;
    s.vel.z *= friction;
    s.rate.x *= friction;
    s.rate.y *= friction;
    s.rate.z *= friction;
  }
}
