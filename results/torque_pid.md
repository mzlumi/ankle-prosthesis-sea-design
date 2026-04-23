# Torque control: PID with model-based feedforward

Controller for the spring torque of the two reference designs (`anklesea.torque_control`), designed on the linear plant of `results/plant.md`. The controller runs at 1000 Hz; its delay (one sample of computation plus half a sample of zero-order hold, 1.5 ms) is included in every margin and tracking result. The motor's current loop is assumed ideal here and is modeled, with saturation, in `results/tracking.md`.

## Targets (set before tuning)

The torque reference comes from the gait data, so the bandwidth target does too: the frequency below which almost all of the torque signal's power lies (stride-averaged torque of each activity in the daily mix, `results/cross_mode.md`).

| activity | 99 % of torque power below (Hz) | 99.9 % below (Hz) |
|---|---|---|
| level walking (1.20 m/s) | 3.7 | 5.5 |
| ramp ascent (5.2 deg) | 3.2 | 4.8 |
| ramp descent (5.2 deg) | 4.2 | 5.9 |
| stair ascent (6 in) | 3.6 | 5.1 |
| stair descent (6 in) | 4.1 | 11.6 |

1. **Bandwidth** (feedback alone, stance, -3 dB of the closed loop): at least 6 Hz. This covers 99.9 % of the torque power in every activity except stair descent, whose landing transient is sharper.
2. **Margins** with the 1.5 ms loop delay, in stance and in swing: phase margin at least 45 deg, gain margin at least 10 dB (in both directions if the loop is conditionally stable), peak sensitivity `max |S|` at most 2 (6 dB).
3. **Zero steady-state error** (integral action).
4. **Tracking**: RMS error at most 5 % of the peak torque in every activity, judged in the nonlinear simulation with voltage and current limits (`results/tracking.md`). The linear results below are the best case.

## Tuning

PD by pole placement on the fixed-output plant (derivation in the module docstring of `anklesea.torque_control`): the closed-loop poles are put at 8 Hz with damping 0.7, the integral corner at 0.2 of that frequency and the derivative filter at ten times it. The proportional gain raises the spring-rotor resonance from about 3 Hz to 8 Hz; the derivative gain damps it. The gains are tiny in N·m per N·m because the gear multiplies motor torque by `N eta`, about 300.

| design | Kp (N·m/N·m) | Ki (1/s) | Kd (ms) | derivative filter (ms) | Kp in A per N·m of error |
|---|---|---|---|---|---|
| walking | 0.0206 | 0.208 | 0.612 | 1.99 | 1.00 |
| compromise | 0.0160 | 0.161 | 0.525 | 1.99 | 0.78 |

The first tuning put the poles at 6 Hz with the integral corner at 0.1 of that. It met targets 1 and 2, but only just: for the compromise design the closed-loop response dipped almost to -3 dB just above the stride frequency (0.92 Hz), and feedback alone left 10 % RMS error. Bandwidth measured at -3 dB says nothing about how flat the response is below it, and a gait torque has most of its power at the first few stride harmonics, so what matters is the loop gain there. A higher placement and integral corner raise it; the integral corner cannot go much higher, because an integrator ahead of a lightly damped resonance makes the loop conditionally stable (it would turn unstable if its gain dropped), which `loop_metrics` reports as a lower gain margin.

| design | tuning | pole placement (Hz) | integral corner / placement | bandwidth (Hz) | lowest \|T\| up to 6 Hz (dB) | \|S\| at the stride frequency (dB) | phase margin (deg) | worst RMS error, feedback only (%) |
|---|---|---|---|---|---|---|---|---|
| walking | first | 6 | 0.1 | 15.0 | -1.0 | -18.5 | 56 | 5.9 |
| walking | final | 8 | 0.2 | 21.0 | -0.2 | -28.0 | 52 | 2.8 |
| compromise | first | 6 | 0.1 | 14.7 | -2.7 | -11.3 | 60 | 10.2 |
| compromise | final | 8 | 0.2 | 20.8 | -0.9 | -20.4 | 54 | 4.1 |

## Margins and bandwidth

| design | phase of gait | bandwidth (Hz) | gain crossover (Hz) | phase margin (deg) | gain margin (dB) | lower gain margin (dB) | max \|S\| | meets targets 1 and 2 |
|---|---|---|---|---|---|---|---|---|
| walking | stance (fixed output) | 21.0 | 12.3 | 52 | 19.4 | none | 1.29 | yes |
| walking | swing (free output) | n/a | 33.3 | 41 | 18.3 | none | 1.59 | **no** |
| compromise | stance (fixed output) | 20.8 | 12.6 | 54 | 19.5 | none | 1.28 | yes |
| compromise | swing (free output) | n/a | 28.7 | 46 | 18.8 | none | 1.46 | yes |

![PID results](figures/torque_pid.png)

*Top: closed-loop torque response and sensitivity with the 1.5 ms delay. Bottom: steady-state tracking error over one stride for the compromise design, linear model; the grey line is the reference torque (scale on the right). Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0.*

The closed-loop bandwidth, 20.8 Hz for the compromise design, is higher than the 8 Hz pole placement because the derivative term adds a zero. In stance the margins are comfortable. In swing the same gains act on the free-output plant, whose sharp resonance at the foot (23 Hz) moves the gain crossover up to 29 Hz, where the delay costs more phase: the phase margin falls to 46 deg for the compromise design and 41 deg for the walking-tuned one, which misses target 2. The closed loop stays stable, but swing is where a delay or gain error would first cause trouble, and a gain schedule that lowers the gains in swing (where the torque reference is near zero anyway) would restore the margin. The closed loop also has a pole at zero in swing: the integrator cancels the free plant's zero at DC, since a free foot cannot hold a steady torque. In practice the integrator is reset or frozen in swing.

## Linear tracking

Steady-state periodic tracking of the mean stride of each activity, computed harmonic by harmonic (exact for the linear model; checked against a 30-stride time simulation in `tests/test_torque_control.py`). Each cell is RMS error in % of the peak torque / peak error in %. The feedforward options are feedback alone, the static term `tau_ref / (N eta)` only, and the full inverse model using the measured joint motion.

| activity | compromise: feedback only | compromise: + static ff | compromise: + model ff | walking-tuned: feedback only | walking-tuned: + static ff | walking-tuned: + model ff |
|---|---|---|---|---|---|---|
| level walking (1.20 m/s) | 4.07 / 11.1 | 1.35 / 3.9 | 0.08 / 0.2 | 2.63 / 8.6 | 1.75 / 4.2 | 0.09 / 0.3 |
| ramp ascent (5.2 deg) | 4.02 / 11.6 | 1.34 / 3.7 | 0.09 / 0.2 | 2.77 / 9.6 | 1.86 / 5.7 | 0.10 / 0.3 |
| ramp descent (5.2 deg) | 3.86 / 10.7 | 1.33 / 4.6 | 0.09 / 0.3 | 2.77 / 8.2 | 1.97 / 7.2 | 0.13 / 0.6 |
| stair ascent (6 in) | 3.27 / 7.9 | 1.36 / 3.4 | 0.05 / 0.1 | 1.96 / 4.7 | 1.38 / 3.1 | 0.05 / 0.2 |
| stair descent (6 in) | 3.71 / 10.7 | 1.63 / 7.4 | 0.12 / 0.5 | 1.89 / 6.0 | 1.12 / 4.4 | 0.08 / 0.4 |

With feedback alone the compromise design tracks level walking with 4.1 % RMS error; with the joint held still it would be 3.5 %, so most of the error comes from the reference itself (finite loop gain at the stride harmonics), not from the joint motion. Reflecting the reference through the gear (static feedforward) brings the worst case to 1.6 %. The full model feedforward cancels the plant dynamics and the joint motion and leaves only the effect of the loop delay, 0.12 % RMS at worst. Without delay it would be exact, which `tests/test_torque_control.py` checks. These numbers assume a perfect model, unlimited voltage and current, and noise-free joint acceleration; `results/tracking.md` and `results/robustness.md` remove those assumptions one at a time.
