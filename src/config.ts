// Reference aircraft: a 2 t, four-rotor, low-disk-loading multirotor air taxi.
//
// Sizing note (this drives every number in the HUD): momentum theory says hover
// power scales as W^1.5 / sqrt(total disk area). At 2000 kg the *only* way to
// land in the ~100-180 kW electrical band that published low-noise multirotor
// concepts quote is a very large total disk area. 4 x 5.0 m rotors give 314 m^2
// (disk loading 62 N/m^2 = 6.4 kg/m^2), which hovers at ~145 kW shaft / ~158 kW
// electrical. For contrast, 3 m rotors on the same mass would need ~260 kW —
// the rotor size is not cosmetic, it *is* the power number.

export const RHO = 1.225; // kg/m^3, ISA sea level
export const G = 9.81; // m/s^2

export interface RotorSpec {
  radius: number; // m
  solidity: number; // sigma = blade area / disk area
  liftSlope: number; // a, per rad (thin-airfoil ~2*pi, real blades ~5.7)
  cd0: number; // mean blade profile drag coefficient
  collective: number; // rad, root pitch (fixed-pitch rotor, RPM is the control)
  maxRpm: number;
  spinLag: number; // s, first-order motor/rotor time constant
}

export const ROTOR: RotorSpec = {
  radius: 5.0,
  solidity: 0.09,
  liftSlope: 5.7,
  cd0: 0.011,
  collective: 0.1661, // 9.52 deg — trims the rotor to weight/4 at ~166 rpm
  maxRpm: 230, // tip speed 120 m/s; gives thrust/weight ~1.9
  spinLag: 0.15,
};

export interface AircraftSpec {
  mass: number; // kg
  /** Rotor hub positions in body frame (x right, y up, z aft). */
  hubs: { x: number; y: number; z: number }[];
  /** +1 / -1 spin direction per rotor; adjacent rotors counter-rotate. */
  spin: number[];
  inertia: { x: number; y: number; z: number }; // kg m^2, diagonal only
  dragArea: number; // Cd*A, m^2 — isotropic bluff-body drag
  driveEfficiency: number; // motor * inverter, shaft power -> electrical
}

const ARM = 5.4; // m along each body axis; hub-to-hub 10.8 m > 2R, so no overlap

export const AIRCRAFT: AircraftSpec = {
  mass: 2000,
  // X layout: front-left, front-right, rear-right, rear-left (forward = -z)
  hubs: [
    { x: -ARM, y: 0.6, z: -ARM },
    { x: ARM, y: 0.6, z: -ARM },
    { x: ARM, y: 0.6, z: ARM },
    { x: -ARM, y: 0.6, z: ARM },
  ],
  spin: [1, -1, 1, -1],
  // ponytail: inertia hand-estimated as 4 point masses of 200 kg on the booms
  // plus a 1200 kg fuselage; good enough for feel, not a structures model.
  inertia: { x: 26000, y: 46000, z: 26000 },
  dragArea: 2.2,
  driveEfficiency: 0.92,
};

export const WEIGHT = AIRCRAFT.mass * G; // N
export const N_ROTORS = AIRCRAFT.hubs.length;
