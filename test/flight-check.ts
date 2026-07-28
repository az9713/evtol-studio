import assert from "node:assert";
import { AIRCRAFT, G, N_ROTORS, ROTOR, WEIGHT } from "../src/config";
import { diskArea, figureOfMerit, rotorPower, rotorThrust, rpmForThrust } from "../src/rotor";
import { attitudeAngles, createFlightState, stepFlight } from "../src/dynamics";
import { createAutopilot, hoverTrim, mixRotors, type PilotInput } from "../src/control";

// Physics acceptance tests for the eVTOL rotor + 6-DOF model.
// Reference values are public multirotor eVTOL ballpark figures. Run: npm run check

const DT = 1 / 200;
const STICK: PilotInput = { pitch: 0, roll: 0, yaw: 0, climb: 0 };

// 1. Momentum theory: at constant thrust, doubling the disk area cuts induced power by sqrt(2).
//    P_ind ~ T * sqrt(T / (2 rho A))  =>  P(2A) / P(A) = 1/sqrt(2)
{
  const T = 5000;
  const small = { ...ROTOR, radius: 3 };
  const big = { ...ROTOR, radius: 3 * Math.SQRT2 }; // area x2
  assert(Math.abs(diskArea(big) / diskArea(small) - 2) < 1e-9, "radius*sqrt(2) must double disk area");

  const pSmall = rotorPower(T, 200, 0, small).induced;
  const pBig = rotorPower(T, 200, 0, big).induced;
  const ratio = pBig / pSmall;
  assert(
    Math.abs(ratio - 1 / Math.SQRT2) < 0.01 / Math.SQRT2,
    `doubling disk area must give 1/sqrt(2) induced power, got ${ratio.toFixed(5)}`,
  );
}

// 2. Induced velocity is the momentum-theory value and climb reduces it
{
  const T = 4905;
  const A = diskArea();
  const vh = Math.sqrt(T / (2 * 1.225 * A));
  const hover = rotorPower(T, 166, 0).ideal / T;
  assert(Math.abs(hover - vh) < 1e-9, `hover induced velocity must be sqrt(T/2rhoA) = ${vh.toFixed(4)}`);
  const climbing = rotorPower(T, 166, 5).ideal / T;
  assert(climbing < hover, "climbing must reduce the induced velocity");
  assert(rotorPower(T, 166, 5).shaft > rotorPower(T, 166, 0).shaft, "climb must cost net power");
}

// 3. Hover power for the reference aircraft (2000 kg, 4 rotors, R = 5 m) must land in
//    the published low-disk-loading multirotor band.
//    P_shaft = sum_i [ T_i*sqrt(T_i/(2 rho A))/FM_ind + (sigma*Cd0/8)*rho*A*(Omega R)^3 ]
//    P_elec  = P_shaft / eta_drive
{
  const h = hoverTrim();
  const kW = h.electricalPower / 1000;
  assert(kW > 80 && kW < 180, `hover electrical power ${kW.toFixed(1)} kW — expected 80-180 kW`);
  assert(h.rpm > 100 && h.rpm < ROTOR.maxRpm, `hover RPM ${h.rpm.toFixed(0)} must be a sane fraction of max`);
  const tipSpeed = ((h.rpm * Math.PI) / 30) * ROTOR.radius;
  assert(tipSpeed > 60 && tipSpeed < 140, `hover tip speed ${tipSpeed.toFixed(0)} m/s — expected 60-140`);
}

// 4. Overall figure of merit at hover is the ~0.7 the spec quotes
{
  const h = hoverTrim();
  const fm = figureOfMerit(h.thrust, h.rpm);
  assert(fm > 0.6 && fm < 0.8, `hover figure of merit ${fm.toFixed(3)} — expected 0.6-0.8`);
}

// 5. Rotor thrust is quadratic in RPM and invertible
{
  assert(Math.abs(rotorThrust(200) / rotorThrust(100) - 4) < 1e-9, "thrust must scale with RPM^2");
  const rpm = rpmForThrust(4905);
  assert(Math.abs(rotorThrust(rpm) - 4905) < 1e-6, "rpmForThrust must invert rotorThrust");
  assert(rpmForThrust(1e9) === ROTOR.maxRpm, "thrust demand must clamp at max RPM");
  assert(rotorThrust(ROTOR.maxRpm) * N_ROTORS > 1.5 * WEIGHT, "aircraft must have >1.5 thrust/weight");
}

// 6. Static trim: the four rotor thrusts sum to the weight
{
  const h = hoverTrim();
  assert(
    Math.abs(h.thrust * N_ROTORS - WEIGHT) < 1e-6,
    `static rotor thrusts must sum to weight ${WEIGHT.toFixed(0)} N`,
  );
}

// 7. Free fall with the motors off accelerates at g
{
  const s = createFlightState(500);
  s.motorsOn = false;
  const zeros = new Array<number>(N_ROTORS).fill(0);
  const v0 = s.vel.y;
  for (let i = 0; i < 40; i++) stepFlight(s, zeros, DT); // 0.2 s, drag still negligible
  const a = (s.vel.y - v0) / (40 * DT);
  assert(Math.abs(a + G) < 0.05, `free fall must accelerate at -${G} m/s^2, got ${a.toFixed(3)}`);
  assert(s.totalThrust === 0, "motors off must produce no thrust");
}

// 8. Closed-loop hover trim: released from a disturbed state the aircraft settles
{
  const s = createFlightState(100);
  s.vel.y = -3;
  s.pos.y = 96;
  const ap = createAutopilot(100);
  for (let i = 0; i < 60 * 200; i++) stepFlight(s, mixRotors(s, STICK, ap), DT);

  assert(Math.abs(s.vel.y) < 0.1, `settled climb rate ${s.vel.y.toFixed(4)} m/s — expected < 0.1`);
  const err = Math.abs(s.totalThrust - WEIGHT) / WEIGHT;
  assert(err < 0.02, `settled total thrust ${(s.totalThrust / 1000).toFixed(2)} kN vs weight — ${(err * 100).toFixed(2)}% off`);
  assert(Math.abs(s.pos.y - 100) < 1, `altitude hold must stay near target, got ${s.pos.y.toFixed(2)} m`);
  const att = attitudeAngles(s.q);
  assert(Math.abs(att.roll) < 0.01 && Math.abs(att.pitch) < 0.01, "hover must settle level");
}

// 9. Hover power in the loop matches the analytic trim
{
  const s = createFlightState(100);
  const ap = createAutopilot(100);
  for (let i = 0; i < 40 * 200; i++) stepFlight(s, mixRotors(s, STICK, ap), DT);
  const kW = s.electricalPower / 1000;
  const expected = hoverTrim().electricalPower / 1000;
  assert(Math.abs(kW - expected) / expected < 0.02, `in-loop hover power ${kW.toFixed(1)} kW vs analytic ${expected.toFixed(1)} kW`);
}

// 10. Pitch stick tilts the disc and translates the aircraft forward (-z)
{
  const s = createFlightState(100);
  const ap = createAutopilot(100);
  const fwd: PilotInput = { pitch: 1, roll: 0, yaw: 0, climb: 0 };
  for (let i = 0; i < 8 * 200; i++) stepFlight(s, mixRotors(s, fwd, ap), DT);
  assert(s.vel.z < -5, `forward stick must build forward speed, got vz = ${s.vel.z.toFixed(2)}`);
  assert(attitudeAngles(s.q).pitch < -0.1, "forward stick must pitch the nose down");
  assert(Math.abs(s.pos.y - 100) < 6, `altitude must be roughly held while accelerating, got ${s.pos.y.toFixed(1)} m`);

  const right: PilotInput = { pitch: 0, roll: 1, yaw: 0, climb: 0 };
  const r = createFlightState(100);
  const rap = createAutopilot(100);
  for (let i = 0; i < 8 * 200; i++) stepFlight(r, mixRotors(r, right, rap), DT);
  assert(r.vel.x > 5, `right stick must build rightward speed, got vx = ${r.vel.x.toFixed(2)}`);
}

// 11. Climb command is tracked, and climbing costs more power than hover
{
  const s = createFlightState(100);
  const ap = createAutopilot(100);
  const up: PilotInput = { pitch: 0, roll: 0, yaw: 0, climb: 1 };
  for (let i = 0; i < 20 * 200; i++) stepFlight(s, mixRotors(s, up, ap), DT);
  assert(Math.abs(s.vel.y - 6) < 0.3, `climb command must settle at 6 m/s, got ${s.vel.y.toFixed(2)}`);
  assert(s.electricalPower > hoverTrim().electricalPower * 1.1, "climbing must draw more power than hover");
}

// 12. Yaw command spins the aircraft about its own axis without drifting away
{
  const s = createFlightState(100);
  const ap = createAutopilot(100);
  const yaw: PilotInput = { pitch: 0, roll: 0, yaw: 1, climb: 0 };
  for (let i = 0; i < 15 * 200; i++) stepFlight(s, mixRotors(s, yaw, ap), DT);
  assert(s.rate.y < -0.2, `yaw right must produce a negative body yaw rate, got ${s.rate.y.toFixed(3)}`);
  assert(Math.hypot(s.pos.x, s.pos.z) < 20, "yawing must not translate the aircraft far");
}

// 13. Ground contact and long-run numerical health
{
  const s = createFlightState(5);
  s.motorsOn = false;
  const zeros = new Array<number>(N_ROTORS).fill(0);
  for (let i = 0; i < 10 * 200; i++) stepFlight(s, zeros, DT);
  assert(s.onGround && s.pos.y > 0, "aircraft must rest on its skids, not fall through the ground");

  const h = createFlightState(80);
  const ap = createAutopilot(80);
  for (let i = 0; i < 120 * 200; i++) stepFlight(h, mixRotors(h, STICK, ap), DT);
  const finite =
    Number.isFinite(h.pos.x + h.pos.y + h.pos.z + h.vel.x + h.vel.y + h.vel.z + h.rate.x + h.rate.y + h.rate.z) &&
    h.rpm.every(Number.isFinite);
  assert(finite, "state must stay finite over a 2 minute hover");
  assert(Math.hypot(h.pos.x, h.pos.z) < 1, "hover must not drift horizontally with no input");
}

// 14. Sanity on the aircraft definition itself
{
  assert(AIRCRAFT.hubs.length === AIRCRAFT.spin.length, "every rotor needs a spin direction");
  assert(AIRCRAFT.spin.reduce((a, b) => a + b, 0) === 0, "counter-rotating pairs must balance");
  const minSpacing = Math.min(
    ...AIRCRAFT.hubs.flatMap((a, i) =>
      AIRCRAFT.hubs.filter((_, j) => j !== i).map((b) => Math.hypot(a.x - b.x, a.z - b.z)),
    ),
  );
  assert(minSpacing > 2 * ROTOR.radius, `rotors must not overlap: spacing ${minSpacing.toFixed(1)} m vs 2R ${2 * ROTOR.radius} m`);
}

const trim = hoverTrim();
console.log(
  `flight-check: hover ${(trim.electricalPower / 1000).toFixed(1)} kW electrical ` +
    `(${(trim.shaftPower / 1000).toFixed(1)} kW shaft), ${trim.rpm.toFixed(0)} rpm, FM ${trim.figureOfMerit.toFixed(3)}`,
);
console.log("flight-check: all assertions passed");
