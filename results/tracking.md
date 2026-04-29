# Torque tracking with drive limits (nonlinear simulation)

Time-domain simulation of the actuator tracking the mean torque of each activity (`anklesea.simulation`). The joint angle is prescribed from the gait profile; the actuator only sets the spring torque. The simulation includes:

- the winding (`L di/dt = v - R i - k_t omega`) and the rotor, at 20 kHz;
- the driver's PI current loop (1000 Hz bandwidth, an assumption in `docs/assumptions.md`) with back-EMF compensation and anti-windup, its output clipped to 34.2 V (0.95 of the 36 V bus) and its current reference to 30 A;
- the torque controller at 1000 Hz with one sample of computation delay, discretized with the Tustin transform, its integrator frozen while the current is clipped;
- torque measured as spring deflection times the nominal stiffness;
- feedforward from the joint velocity and acceleration estimated by a 25 Hz second-order filter on the sampled joint angle (no noise here; noise is in `results/robustness.md`).

4 strides are simulated and the last is evaluated. The controllers are the PID (`results/torque_pid.md`) and the PD + DOB (`results/torque_dob.md`), both with model feedforward.

## Check against the sizing model and the linear analysis

Level walking at 1.2 m/s, limits removed, exact joint derivatives. Each cell is simulation / reference: energy per stride and currents against the inverse model used for sizing (`anklesea.sea`), tracking error against the linear harmonic solution of `results/torque_pid.md`.

| design | energy per stride (J) | RMS current (A) | peak current (A) | RMS error (%) |
|---|---|---|---|---|
| walking | 23.50 / 23.39 | 5.68 / 5.67 | 11.3 / 11.2 | 0.042 / 0.099 |
| compromise | 24.76 / 24.67 | 6.39 / 6.38 | 12.4 / 12.3 | 0.038 / 0.090 |

The energies and currents agree to within a percent, so the sizing results hold for a controlled actuator that tracks well. The tracking errors are both a fraction of a percent; they differ because the simulation's delay is a sample-and-hold, not a pure delay.

## Compromise design, PID with model feedforward

RMS and peak error in % of the peak reference torque; saturation as % of the stride; currents and voltage from the last stride; energy with ideal regeneration.

| condition | RMS error (%) | peak error (%) | voltage saturated (%) | current saturated (%) | peak current (A) | RMS current (A) | peak voltage (V) | energy (J/stride) | RMS error within 5 % |
|---|---|---|---|---|---|---|---|---|---|
| treadmill 1.20 m/s | 0.60 | 1.6 | 0.0 | 0.0 | 12.4 | 6.5 | 27.5 | 26.0 | yes |
| treadmill 1.85 m/s | 3.29 | 11.5 | 24.9 | 3.9 | 14.1 | 6.4 | 34.2 | 53.6 | yes |
| ramp ascent 5.2 deg | 0.37 | 1.1 | 0.0 | 0.0 | 13.6 | 6.2 | 28.2 | 42.4 | yes |
| ramp ascent 7.8 deg | 0.47 | 1.8 | 0.0 | 0.0 | 16.9 | 7.4 | 32.3 | 47.0 | yes |
| ramp descent 5.2 deg | 0.67 | 2.2 | 2.5 | 0.0 | 14.1 | 5.9 | 34.2 | 1.4 | yes |
| ramp descent 9.2 deg | 1.51 | 8.7 | 3.8 | 0.0 | 23.2 | 5.9 | 34.2 | -5.1 | yes |
| stair ascent 6 in | 0.21 | 0.6 | 0.0 | 0.0 | 10.6 | 5.4 | 27.3 | 51.5 | yes |
| stair descent 6 in | 0.56 | 1.7 | 0.0 | 0.0 | 13.4 | 5.5 | 33.2 | -32.8 | yes |

## Both designs and both controllers

RMS error in % of peak torque, with the share of the stride spent at the voltage or current limit in brackets.

| condition | compromise, PID | compromise, DOB | walking-tuned, PID | walking-tuned, DOB |
|---|---|---|---|---|
| treadmill 1.20 m/s | 0.60 (0 %) | 0.50 (0 %) | 0.69 (0 %) | 0.58 (2 %) |
| treadmill 1.85 m/s | 3.29 (25 %) | 3.16 (24 %) | 5.50 (35 %) | 5.22 (34 %) |
| ramp ascent 5.2 deg | 0.37 (0 %) | 0.31 (0 %) | 0.42 (0 %) | 0.35 (0 %) |
| ramp ascent 7.8 deg | 0.47 (0 %) | 0.43 (0 %) | 0.53 (2 %) | 0.49 (3 %) |
| ramp descent 5.2 deg | 0.67 (3 %) | 0.59 (2 %) | 0.89 (4 %) | 0.84 (5 %) |
| ramp descent 9.2 deg | 1.51 (4 %) | 1.50 (4 %) | 2.50 (5 %) | 2.48 (5 %) |
| stair ascent 6 in | 0.21 (0 %) | 0.18 (0 %) | 0.23 (0 %) | 0.21 (0 %) |
| stair descent 6 in | 0.56 (0 %) | 0.51 (0 %) | 1.53 (14 %) | 1.53 (14 %) |

![Tracking](figures/tracking.png)

*Simulated torque, motor current and winding voltage over the last stride, PID with model feedforward. Dotted lines: driver limits (30 A, 34.2 V); dash-dot: the motor's continuous current rating (5.06 A), which applies to the RMS current. Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0.*

## Reading the results

- The compromise design tracks level walking with 0.60 % RMS error, most of it from estimating the joint acceleration with a filter (with exact derivatives it is a few hundredths of a percent, table above). It reaches a drive limit in treadmill 1.85 m/s, ramp descent 5.2 deg, ramp descent 9.2 deg. Its worst case is treadmill 1.85 m/s at 3.29 % RMS. Every condition meets the 5 % tracking target.
- The walking-tuned design saturates in treadmill 1.85 m/s, ramp ascent 7.8 deg, ramp descent 5.2 deg, ramp descent 9.2 deg, stair descent 6 in. This is the time-domain view of what the sizing found (`results/cross_mode.md`): its high ratio needs more voltage than the driver has when the motor must spin fast. While the voltage is saturated the current cannot follow its reference, and the torque error grows until the motor slows down again.
- The DOB and the PID perform alike here, as the linear comparison predicted: with model feedforward there is little left for either to correct.
- Both designs are sized onto the voltage limit: the energy optimum of the walking-tuned design uses the full 34.2 V in level walking, and the compromise uses it in ramp descent (`results/cross_mode.md`). The inverse model needs exactly that voltage for perfect tracking, so any feedback correction or estimation error pushes the drive into saturation. A design for a real device should keep a voltage margin for control, for example by sizing against 85 to 90 % of the available voltage.
- The RMS current in level walking (6.5 A) is above the motor's continuous rating (5.06 A) for both designs. Tracking does not change this; it is the thermal limit found in the sizing (`results/sizing_modes.md`) and addressed by the parallel spring (`results/parallel_spring.md`).

## What did not work first

The first version of the simulation froze the PID integrator only when the current was clipped and fed the DOB the torque command it had computed. In treadmill 2.05 m/s (2 subjects, a stress case beyond the conditions above), where the compromise design's drive is saturated for 47 % of the stride, that gave:

| controller | RMS error, first version (%) | RMS error, final anti-windup (%) |
|---|---|---|
| PID | 9.60 | 8.99 |
| DOB | 57.61 | 8.84 |

While the voltage is saturated, the current lags its reference; the DOB, comparing the measured torque with the *command*, sees the shortfall as a disturbance and adds it to the next command, which only deepens the saturation. Feeding the DOB the torque the motor actually produced (`k_t` times the measured current, which drives measure anyway) fixed the DOB. Freezing the PID integrator during voltage saturation as well helps the PID only a little, because its integrator is slow compared with a saturation episode.
