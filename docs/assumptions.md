# Assumptions

Every assumption that changes a result, with its source or the reason for the value, and where it is varied. Datasheet values are not assumptions; they are listed in [`data/motors/maxon_ec4pole30_305014.yaml`](../data/motors/maxon_ec4pole30_305014.yaml).

## Load

| Assumption | Value | Source or reason | Varied in |
|---|---|---|---|
| Prosthesis user body mass | 80 kg | ISO 10328:2016 (structural testing of lower-limb prostheses) defines test loading levels by user body mass: P3 up to 60 kg, P4 up to 80 kg, P5 up to 100 kg. 80 kg is the P4 limit. | Sizing sensitivity at 60 and 100 kg |
| Joint torque scales with body mass | torque per kg from the dataset times the user mass | Standard normalization of inverse dynamics; the angle and stride time are not scaled. | Not varied |
| Able-bodied ankle as the prosthesis reference | mean right-ankle angle and moment of the dataset subjects | The prosthesis is asked to reproduce able-bodied ankle mechanics, the usual target for powered ankles. Amputee gait differs. | Not varied (limitation) |
| Stride-averaged, periodic load | mean profile per activity and condition, fitted with 20 Fourier harmonics | Averaging removes stride-to-stride variation; the Fourier fit makes the cycle closed so that the energy over one stride is that of a steady periodic gait. | Harmonic count checked in tests |
| Angle offset | `ik` angles (absolute OpenSim angles), not `ik_offset` | The energy depends only on angle derivatives, and the parallel spring rest angle is a design variable. See `results/gait_profiles.md`. | Not needed |

## Actuator

| Assumption | Value | Source or reason | Varied in |
|---|---|---|---|
| Equivalent DC motor model | `v = R i + L di/dt + k_t omega`, torque `k_t i` | maxon's phase-to-phase resistance and torque constant reproduce the catalog's stall torque and no-load speed in this model (checked in `tests/test_motor.py`). | Not varied |
| Viscous friction on the motor shaft | `b = k_t I_0 / omega_0` = 5.7e-6 N·m·s/rad | Not in the datasheet. The no-load current `I_0` at the no-load speed `omega_0` measures the friction torque at that speed; spreading it linearly over speed gives an equivalent viscous coefficient. | Robustness study |
| Transmission efficiency | 0.70, constant, applied in both directions of power flow | Maximum efficiency of the maxon GP 32 HP 3-stage gearhead (datasheet). Constant efficiency keeps the energy quadratic in compliance; it is conservative when the joint backdrives the motor. | 0.6 and 0.9; direction-dependent model compared |
| Final transmission stage (ball screw and lever) | lossless and massless | The gearhead's output rating (12 N·m) is far below the ankle torque, so a real actuator needs a final high-force stage. Its losses and inertia are not modeled. | Through the efficiency sensitivity |
| Total ratio N | continuous design variable | A real ratio is a product of discrete gearhead ratios and a screw lead and lever arm, which can approximate any N closely. | Swept |
| Bus voltage | 36 V | The nominal voltage of the chosen winding (part 305014) and a common battery voltage (10-cell lithium-ion pack). The driver delivers at most 0.95 of its supply (ESCON 70/10 datasheet). | 24 V and 48 V |
| Peak current limit | 30 A | ESCON 70/10 short-time output current (datasheet); the motor page gives no short-term rating. | Not varied |
| RMS current limit | 5.06 A | Motor nominal (max. continuous) current, the catalog's thermal limit for continuous operation. Comparing it with the RMS over one stride assumes continuous walking. | Not varied |
| Copper resistance | 0.21 Ω at 25 °C for all temperatures | Datasheet terminal resistance. A hot winding has higher resistance (about 0.39 %/K for copper), so copper losses are underestimated in sustained walking. | Robustness study (motor constant error) |
| Regeneration | energy reported with ideal regeneration (negative power returned to the battery) and without (negative power dissipated) | The two bound a real driver and battery. Driver losses are ignored. | Both reported |

## Activity mix for the compromise design

| Assumption | Value | Source or reason | Varied in |
|---|---|---|---|
| Share of steps per activity | level 91.8 % (82.8 % straight plus 9.0 % turning), ramp ascent 1.6 %, ramp descent 2.0 %, stair ascent 2.3 %, stair descent 2.5 % | B. Srisuwan and G. K. Klute, "Locomotor activities of individuals with lower limb amputation", *Prosthet. Orthot. Int.* 45(3):191-197, 2021, doi:10.1097/PXR.0000000000000009, Table 3: ten people with a unilateral transtibial prosthesis, free-living, 1 to 2 weekdays, classified from a pylon-mounted sensor. Turning steps are counted as level walking. | `results/cross_mode.md`: ramps and stairs three times as frequent, equal weights, level walking only |
| Steps per day | 4422 (2211 strides of the prosthetic leg) | Same source. Only scales the daily energy, not the design. | Not varied |
| Condition standing for each activity | treadmill 1.2 m/s; ramp incline and stair height closest to the source's course (5 degree ramps, 18 cm stair rise) among well-covered conditions | The mix source's course geometry; the dataset offers several inclines and step heights. | Lowest and highest inclines and step heights; walking at 1.0 and 1.4 m/s |

## Parallel spring

| Assumption | Value | Source or reason | Varied in |
|---|---|---|---|
| Parallel spring law | linear, `tau_p = -k_p (theta - theta_0)`, acting in stance and swing | Keeps the design problem convex in the spring parameters (L12, L13). Real designs often engage the spring only in stance (clutch or unidirectional spring), which this model cannot represent. | Not varied; discussed in `results/parallel_spring.md` |

## Control

| Assumption | Value | Source or reason | Varied in |
|---|---|---|---|
| Inertia of the foot below the series spring | 0.01 kg m² about the ankle | Assumption for a prosthetic foot and shoe of roughly 1 kg with its centre of mass a few centimetres from the joint. It only matters with the foot in the air (free-output plant). | Not varied; the fixed-output plant, which governs stance, does not depend on it |
| Controlled designs | the walking-tuned and compromise designs of `results/cross_mode.md` | The compromise meets the drive limits in every activity; the walking-tuned design is the energy optimum for level walking. | Both carried through sections 12 to 16 |
