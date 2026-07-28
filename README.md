# eVTOL Design Studio

Browser eVTOL flight studio: an articulated multirotor flown over a procedural city,
with a Blade-Element-Momentum-lite rotor model driving the power numbers on the HUD.

**Fly it:** https://az9713.github.io/evtol-studio/ (or `npm install && npm run dev`)

Phases 0-1 of the spec (`docs/three-ambitious-simulation-specs.html`, eVTOL tab):
city scene, articulated aircraft, 6-DOF free flight, per-rotor BEM rotor and power model.

```
npm install
npm run dev     # http://localhost:5173
npm run check   # physics + scene assertions
npm run build   # tsc && vite build
```

## Controls

| Key | Action |
| --- | --- |
| `W` / `S` (or up/down) | pitch — accelerate forward / back |
| `A` / `D` (or left/right) | roll — accelerate left / right |
| `Q` / `E` | yaw left / right |
| `Space` / `Shift` | climb / descend (releasing holds altitude) |
| `C` | chase / free camera |
| `T` | per-rotor thrust arrows |
| `M` | cut the motors (free fall) |
| `R` | reset to the vertiport |

## Reference aircraft

2000 kg, 4 rotors, 5.0 m radius, 3 blades, solidity 0.09, fixed 9.5° collective.
RPM is the control; the mixer trades differential rotor thrust for pitch, roll and yaw.

**Hover: 157.7 kW electrical (145.1 kW shaft) at 166 rpm, figure of merit 0.68.**

The rotor radius is doing real work in that number. Momentum theory makes hover power
scale as `W^1.5 / sqrt(total disk area)`, so at 2000 kg the disk area *is* the power
budget. 4 × 5.0 m rotors give 314 m² (disk loading 62 N/m², 6.4 kg/m²) and land in the
100-180 kW band that low-noise multirotor concepts quote. The same mass on 3 m rotors
would need ~260 kW — a third of the way to a Joby-class machine, which at 595 N/m² disk
loading burns well over 500 kW in hover.

## Physics

Per rotor, per step (`src/rotor.ts`):

```
C_T   = (sigma*a/2) * (theta/3 - lambda/2),  lambda = sqrt(C_T/2)   # BEM + momentum, closed form
T     = C_T * rho * A * (Omega R)^2
v_i   = -Vc/2 + sqrt((Vc/2)^2 + T/(2 rho A))                        # induced velocity in climb
P_ind = T * v_i / 0.87                                              # non-uniform inflow, kappa = 1.15
P_prf = (sigma*Cd0/8) * rho * A * (Omega R)^3                       # profile
P_shaft = P_ind + P_prf + T*Vc,   P_elec = P_shaft / 0.92
```

The overall figure of merit `P_ideal / P_shaft` comes out at 0.68 — the ~0.7 the spec
quotes. It is an *output* of the model, not an input: dividing ideal power by 0.7 *and*
adding a profile term would count the profile losses twice.

Rigid-body state is integrated in `src/dynamics.ts` (semi-implicit Euler, quaternion
attitude, Euler's equations with diagonal inertia). `src/control.ts` holds attitude,
yaw rate and climb rate with PD loops and mixes them onto the four rotors.

Deliberate simplifications are marked `// ponytail:` at the point where they are made.
