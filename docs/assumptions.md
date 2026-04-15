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
