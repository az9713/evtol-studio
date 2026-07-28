import { AIRCRAFT, G, N_ROTORS, ROTOR, WEIGHT } from "./config";
import { attitudeAngles, bodyToWorld, v3, type FlightState } from "./dynamics";
import { rotorPower, rotorThrust, rpmForThrust } from "./rotor";

// Flight control: PD hold assists on top of a differential-thrust mixer.
// The pilot never commands rotor thrust directly — sticks ask for a tilt angle,
// a yaw rate and a climb rate, and these loops work out what each rotor must do.
//
// ponytail: hand-tuned PD, no integral term, no feed-forward, no actuator
// saturation handling beyond a clamp. It holds hover to well under 0.1 m/s,
// which is all Phase 1 needs.

export const MAX_TILT = 0.35; // rad, ~20 deg
export const MAX_YAW_RATE = 0.5; // rad/s
export const MAX_CLIMB = 6; // m/s
export const MAX_DESCENT = 4; // m/s

const KP_ATT = 4.0; // attitude natural frequency ~2 rad/s ...
const KD_ATT = 3.4; // ... at damping ratio ~0.85
const KP_YAW = 1.5;
const KP_VS = 1.5; // climb-rate loop
const KP_ALT = 0.6; // altitude hold feeds the climb-rate loop

export const MAX_ROTOR_THRUST = rotorThrust(ROTOR.maxRpm);

/** Steady hover operating point for the reference aircraft. */
export function hoverTrim() {
  const thrust = WEIGHT / N_ROTORS;
  const rpm = rpmForThrust(thrust);
  const power = rotorPower(thrust, rpm, 0);
  return {
    thrust,
    rpm,
    shaftPower: power.shaft * N_ROTORS,
    electricalPower: (power.shaft * N_ROTORS) / AIRCRAFT.driveEfficiency,
    figureOfMerit: power.ideal / power.shaft,
    torque: power.torque,
  };
}

// Yaw is produced by unbalancing the counter-rotating pairs. Near hover the shaft
// torque goes as Q ~ T^1.5, so dQ/dT = 1.5 Q_h/T_h and a moment M_y needs a thrust
// split of -T_h / (1.5 * N * Q_h) * M_y. The yaw PD mops up the rest.
const trim = hoverTrim();
const YAW_MIX = -trim.thrust / (1.5 * N_ROTORS * trim.torque);

const SUM_X2 = AIRCRAFT.hubs.reduce((a, h) => a + h.x * h.x, 0);
const SUM_Z2 = AIRCRAFT.hubs.reduce((a, h) => a + h.z * h.z, 0);

export interface PilotInput {
  pitch: number; // -1..1, +1 = nose down / accelerate forward
  roll: number; // -1..1, +1 = bank right
  yaw: number; // -1..1, +1 = nose right
  climb: number; // -1..1, +1 = climb
}

export interface Autopilot {
  altTarget: number;
  /** Level-and-hold: zero the tilt whenever the stick is centred. */
  levelAssist: boolean;
}

export const createAutopilot = (altitude: number): Autopilot => ({ altTarget: altitude, levelAssist: true });

/** Per-rotor thrust demand (N) for this frame. */
export function mixRotors(s: FlightState, input: PilotInput, ap: Autopilot): number[] {
  const { roll, pitch } = attitudeAngles(s.q);
  const up = bodyToWorld(s.q, v3(0, 1, 0));

  // --- vertical: stick -> climb rate -> vertical acceleration -> collective thrust ---
  let climbTarget: number;
  if (Math.abs(input.climb) > 0.02) {
    climbTarget = input.climb * (input.climb > 0 ? MAX_CLIMB : MAX_DESCENT);
    ap.altTarget = s.pos.y;
  } else {
    climbTarget = clamp(KP_ALT * (ap.altTarget - s.pos.y), -MAX_DESCENT, MAX_CLIMB);
  }
  const accelZ = clamp(KP_VS * (climbTarget - s.vel.y), -4, 6);
  // Tilting the disc costs vertical thrust: divide by the vertical component of body-up.
  const total = clamp((AIRCRAFT.mass * (G + accelZ)) / Math.max(0.4, up.y), 0, N_ROTORS * MAX_ROTOR_THRUST);

  // --- attitude: stick -> tilt angle -> body moments ---
  const pitchTarget = ap.levelAssist ? -input.pitch * MAX_TILT : 0;
  const rollTarget = ap.levelAssist ? -input.roll * MAX_TILT : 0;
  const Mx = AIRCRAFT.inertia.x * (KP_ATT * (pitchTarget - pitch) - KD_ATT * s.rate.x);
  const Mz = AIRCRAFT.inertia.z * (KP_ATT * (rollTarget - roll) - KD_ATT * s.rate.z);
  const My = AIRCRAFT.inertia.y * KP_YAW * (-input.yaw * MAX_YAW_RATE - s.rate.y);

  // --- mixer: T_i = T/N - (Mx/sum z^2) z_i + (Mz/sum x^2) x_i + spin_i * yaw ---
  const cp = Mx / SUM_Z2;
  const cr = Mz / SUM_X2;
  const yaw = YAW_MIX * clamp(My, -0.4 * total, 0.4 * total);

  const demand = new Array<number>(N_ROTORS);
  for (let i = 0; i < N_ROTORS; i++) {
    const h = AIRCRAFT.hubs[i];
    demand[i] = clamp(total / N_ROTORS - cp * h.z + cr * h.x + AIRCRAFT.spin[i] * yaw, 0, MAX_ROTOR_THRUST);
  }
  return demand;
}

export const clamp = (v: number, lo: number, hi: number) => (v < lo ? lo : v > hi ? hi : v);
