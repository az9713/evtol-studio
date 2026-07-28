import { RHO, ROTOR, type RotorSpec } from "./config";

// Blade-Element-Momentum-lite rotor model.
//
// Blade element, untwisted blade, uniform inflow:   C_T = (sigma*a/2) * (theta/3 - lambda/2)
// Momentum theory closes it:                        lambda = sqrt(C_T / 2)
// Substituting lambda gives a plain quadratic in lambda, so no iteration is
// needed — that is the whole "BEM" here. Because C_T is dimensionless it does
// not depend on RPM, which makes thrust exactly quadratic in tip speed and
// makes the thrust -> RPM inversion a closed form too.
//
// ponytail: uniform inflow, no blade twist/taper, no tip loss, no compressibility,
// no forward-flight (mu = 0) inflow skew. Those are the next three weeks of work
// and none of them move hover power by more than ~10%.

/** Induced-power efficiency: P_induced = ideal / this. kappa = 1/0.87 = 1.15 is the
 *  standard non-uniform-inflow correction. Combined with the profile term below it
 *  yields an overall figure of merit FM = P_ideal / P_shaft of ~0.68 at design hover,
 *  which is the FM ~= 0.7 the spec quotes. (Dividing the ideal power by 0.7 *and*
 *  adding profile power would double-count the profile losses — FM already contains them.) */
export const FM_INDUCED = 0.87;

export const diskArea = (spec: RotorSpec = ROTOR) => Math.PI * spec.radius * spec.radius;

export const rpmToOmega = (rpm: number) => (rpm * Math.PI) / 30;
export const omegaToRpm = (omega: number) => (omega * 30) / Math.PI;

/** Hover thrust coefficient C_T = T / (rho * A * (Omega R)^2), from the BEM/momentum pair. */
export function thrustCoefficient(spec: RotorSpec = ROTOR): number {
  const k = (spec.solidity * spec.liftSlope) / 2;
  // 2*lambda^2 + (k/2)*lambda - k*theta/3 = 0
  const lambda = (-k / 2 + Math.sqrt((k * k) / 4 + (8 * k * spec.collective) / 3)) / 4;
  return 2 * lambda * lambda;
}

/** Thrust (N) produced by one rotor at a given RPM. */
export function rotorThrust(rpm: number, spec: RotorSpec = ROTOR): number {
  const tip = rpmToOmega(rpm) * spec.radius;
  return thrustCoefficient(spec) * RHO * diskArea(spec) * tip * tip;
}

/** RPM needed for a demanded thrust (N), clamped to the motor limit. */
export function rpmForThrust(thrust: number, spec: RotorSpec = ROTOR): number {
  if (thrust <= 0) return 0;
  const tip = Math.sqrt(thrust / (thrustCoefficient(spec) * RHO * diskArea(spec)));
  return Math.min(spec.maxRpm, omegaToRpm(tip / spec.radius));
}

/** Hover induced velocity v_h = sqrt(T / (2 rho A)). */
export function hoverInducedVelocity(thrust: number, area: number): number {
  return Math.sqrt(Math.max(0, thrust) / (2 * RHO * area));
}

/** Induced velocity in axial climb: v_i = -Vc/2 + sqrt((Vc/2)^2 + v_h^2). */
export function inducedVelocity(thrust: number, area: number, climb: number): number {
  // ponytail: momentum theory is invalid in the vortex-ring state (-2 v_h < Vc < 0),
  // so descent is treated as hover for the induced solve. Conservative and stable.
  const vc = Math.max(0, climb);
  const vh = hoverInducedVelocity(thrust, area);
  return -vc / 2 + Math.sqrt((vc * vc) / 4 + vh * vh);
}

export interface RotorPower {
  ideal: number; // W, T * v_i with no losses
  induced: number; // W
  profile: number; // W
  climb: number; // W
  shaft: number; // W, what the motor must deliver
  torque: number; // N m on the shaft (reacts into the airframe as yaw)
}

/** Power breakdown for one rotor holding `thrust` at `rpm` while climbing at `climb` m/s. */
export function rotorPower(thrust: number, rpm: number, climb: number, spec: RotorSpec = ROTOR): RotorPower {
  const area = diskArea(spec);
  const omega = rpmToOmega(rpm);
  const tip = omega * spec.radius;

  const ideal = thrust * inducedVelocity(thrust, area, climb);
  const induced = ideal / FM_INDUCED;
  // P_profile = (sigma * Cd0 / 8) * rho * A * (Omega R)^3
  const profile = ((spec.solidity * spec.cd0) / 8) * RHO * area * tip * tip * tip;
  const climbPower = thrust * climb;

  const shaft = induced + profile + climbPower;
  return { ideal, induced, profile, climb: climbPower, shaft, torque: omega > 1e-6 ? shaft / omega : 0 };
}

/** Overall rotor figure of merit at this operating point: FM = P_ideal / P_shaft. */
export function figureOfMerit(thrust: number, rpm: number, spec: RotorSpec = ROTOR): number {
  const p = rotorPower(thrust, rpm, 0, spec);
  return p.shaft > 0 ? p.ideal / p.shaft : 0;
}
