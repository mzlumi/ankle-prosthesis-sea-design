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
