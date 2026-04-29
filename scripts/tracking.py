#!/usr/bin/env python3
"""Nonlinear time-domain torque tracking with voltage and current limits, per activity.

Writes:
  results/tracking.md
  results/figures/tracking.png
"""

from __future__ import annotations

from dataclasses import replace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from anklesea import RESULTS_DIR
from anklesea.designs import (
    CONTROL_RATE_HZ,
    LOOP_DELAY,
    TARGET_TRACKING_PCT,
    default_setup,
    reference_controllers,
    reference_designs,
)
from anklesea.plant import SEAParams
from anklesea.plots import ACTIVITY_NAMES, CITATION, add_citation
from anklesea.profiles import read_profiles
from anklesea.sea import evaluate_design
from anklesea.simulation import Drive, SimOptions, simulate
from anklesea.sizing import BUS_VOLTAGE, DESIGN_MASS_KG, LEVEL_WALK, mix_conditions
from anklesea.torque_control import periodic_tracking

REPORT = RESULTS_DIR / "tracking.md"
FIGURE = RESULTS_DIR / "figures" / "tracking.png"
OPTIONS = SimOptions()
MIN_SUBJECTS = 5  # the fastest and steepest conditions are taken among those with at least this many subjects
STYLES = {"compromise": ("#b45309", "-"), "walking": ("#1d4ed8", "--")}


def conditions_to_run(profiles: dict) -> list[tuple[str, str]]:
    """The daily-mix condition of every activity, plus the fastest and steepest ones with enough subjects."""
    mix = mix_conditions(profiles)
    keys = [mix["level"]]
    covered = [key for key, prof in profiles.items() if prof.n_subjects >= MIN_SUBJECTS]
    treadmill = [c for a, c in covered if a == "treadmill"]
    keys.append(("treadmill", max(treadmill, key=lambda c: float(c.split()[0]))))
    for activity in ["rampascent", "rampdescent", "stairascent", "stairdescent"]:
        keys.append(mix[activity])
        steepest = max((c for a, c in covered if a == activity), key=lambda c: float(c.split()[0]))
        if steepest != mix[activity][1]:
            keys.append((activity, steepest))
    return keys


def label(key: tuple[str, str]) -> str:
    return f"{ACTIVITY_NAMES[key[0]]} {key[1]}"


def main() -> int:
    motor, limits = default_setup()
    designs = reference_designs()
    params = {name: SEAParams.from_design(d, motor) for name, d in designs.items()}
    controllers = {name: reference_controllers(p) for name, p in params.items()}
    drive = Drive.from_motor(motor, BUS_VOLTAGE)
    profiles = read_profiles()
    keys = conditions_to_run(profiles)

    runs = {}
    for key in keys:
        for name in params:
            for cname, c in controllers[name].items():
                runs[(key, name, cname)] = simulate(profiles[key], DESIGN_MASS_KG, c, drive, options=OPTIONS)

    def ok(flag: bool) -> str:
        return "yes" if flag else "**no**"

    main_rows = []
    for key in keys:
        r = runs[(key, "compromise", "PID")]
        main_rows.append(
            f"| {label(key)} | {r.rms_error_pct():.2f} | {r.peak_error_pct():.1f} | {100 * r.voltage_saturated:.1f} | "
            f"{100 * r.current_saturated:.1f} | {np.abs(r.current).max():.1f} | {r.rms_current:.1f} | "
            f"{np.abs(r.voltage).max():.1f} | {r.energy:.1f} | {ok(r.rms_error_pct() <= TARGET_TRACKING_PCT)} |")
    compare_rows = []
    for key in keys:
        cells = []
        for name, cname in [("compromise", "PID"), ("compromise", "DOB"), ("walking", "PID"), ("walking", "DOB")]:
            r = runs[(key, name, cname)]
            sat = max(r.voltage_saturated, r.current_saturated)
            cells.append(f"{r.rms_error_pct():.2f} ({100 * sat:.0f} %)")
        compare_rows.append(f"| {label(key)} | " + " | ".join(cells) + " |")

    # The first version's anti-windup, on the fastest walking condition of any subject count.
    fast = max((k for k in profiles if k[0] == "treadmill"), key=lambda k: float(k[1].split()[0]))
    fast_runs = {cname: runs.get((fast, "compromise", cname)) or simulate(
        profiles[fast], DESIGN_MASS_KG, controllers["compromise"][cname], drive, options=OPTIONS) for cname in ["PID", "DOB"]}
    naive_rows = []
    for cname in ["PID", "DOB"]:
        good = fast_runs[cname]
        naive = simulate(profiles[fast], DESIGN_MASS_KG, controllers["compromise"][cname], drive,
                         options=replace(OPTIONS, naive_windup=True))
        naive_rows.append(f"| {cname} | {naive.rms_error_pct():.2f} | {good.rms_error_pct():.2f} |")

    # Validation against the inverse model and the linear analysis.
    level = profiles[LEVEL_WALK]
    unlimited = replace(drive, voltage_limit=1e6, current_limit=1e6)
    val_rows = []
    for name, p in params.items():
        c = controllers[name]["PID"]
        exact = simulate(level, DESIGN_MASS_KG, c, unlimited, options=replace(OPTIONS, joint_derivatives="exact"))
        model = evaluate_design(level.load(DESIGN_MASS_KG), designs[name], motor, limits)
        lin = periodic_tracking(level, DESIGN_MASS_KG, c, delay=LOOP_DELAY)
        val_rows.append(f"| {name} | {exact.energy:.2f} / {model['energy']:.2f} | {exact.rms_current:.2f} / "
                        f"{model['rms_current']:.2f} | {np.abs(exact.current).max():.1f} / {model['peak_current']:.1f} | "
                        f"{exact.rms_error_pct():.3f} / {lin.rms_error_pct():.3f} |")

    # Figure ----------------------------------------------------------------------------------
    stair = mix_conditions(profiles)["stairascent"]
    fig, axes = plt.subplots(3, 2, figsize=(12, 9.5), layout="constrained", sharex="col")
    for col, key in enumerate([LEVEL_WALK, stair]):
        ref = runs[(key, "compromise", "PID")]
        pct = 100 * ref.t / (ref.t[-1] + ref.t[1])
        axes[0, col].plot(pct, ref.reference, color="0.6", lw=4, label="reference")
        for name in ["compromise", "walking"]:
            r = runs[(key, name, "PID")]
            color, ls = STYLES[name]
            d = designs[name]
            axes[0, col].plot(pct, r.torque, color=color, ls=ls, lw=1.4,
                              label=f"{name} (k = {d.stiffness:.0f}, N = {d.ratio:.0f}): {r.rms_error_pct():.1f} % RMS")
            axes[1, col].plot(pct, r.current, color=color, ls=ls, lw=1.4)
            axes[2, col].plot(pct, r.voltage, color=color, ls=ls, lw=1.4)
        for row, lim in [(1, drive.current_limit), (2, drive.voltage_limit)]:
            for sign in (-1, 1):
                axes[row, col].axhline(sign * lim, color="black", lw=0.8, ls=":")
        for sign in (-1, 1):
            axes[1, col].axhline(sign * motor.continuous_current, color="0.5", lw=0.8, ls="-.")
        axes[0, col].set_title(label(key), loc="left")
        axes[0, col].legend(fontsize=8, loc="lower right")
        axes[2, col].set_xlabel("gait cycle (%)")
        for row in range(3):
            axes[row, col].grid(alpha=0.3)
    axes[0, 0].set_ylabel("spring torque (N·m)")
    axes[1, 0].set_ylabel("motor current (A)")
    axes[2, 0].set_ylabel("winding voltage (V)")
    add_citation(fig, CITATION)
    fig.savefig(FIGURE, dpi=150)
    plt.close(fig)

    comp_fail = [label(k) for k in keys if runs[(k, "compromise", "PID")].rms_error_pct() > TARGET_TRACKING_PCT]
    walk_sat = [label(k) for k in keys
                if max(runs[(k, "walking", "PID")].voltage_saturated, runs[(k, "walking", "PID")].current_saturated) > 0]
    comp_sat = [label(k) for k in keys
                if max(runs[(k, "compromise", "PID")].voltage_saturated, runs[(k, "compromise", "PID")].current_saturated) > 0]
    worst_key = max(keys, key=lambda k: runs[(k, "compromise", "PID")].rms_error_pct())
    worst = runs[(worst_key, "compromise", "PID")]
    level_r = runs[(LEVEL_WALK, "compromise", "PID")]
    text = f"""# Torque tracking with drive limits (nonlinear simulation)

Time-domain simulation of the actuator tracking the mean torque of each activity (`anklesea.simulation`). The joint angle is prescribed from the gait profile; the actuator only sets the spring torque. The simulation includes:

- the winding (`L di/dt = v - R i - k_t omega`) and the rotor, at {OPTIONS.sim_rate_hz / 1000:.0f} kHz;
- the driver's PI current loop ({drive.current_bandwidth_hz:.0f} Hz bandwidth, an assumption in `docs/assumptions.md`) with back-EMF compensation and anti-windup, its output clipped to {drive.voltage_limit:.1f} V (0.95 of the {BUS_VOLTAGE:.0f} V bus) and its current reference to {drive.current_limit:.0f} A;
- the torque controller at {CONTROL_RATE_HZ:.0f} Hz with one sample of computation delay, discretized with the Tustin transform, its integrator frozen while the current is clipped;
- torque measured as spring deflection times the nominal stiffness;
- feedforward from the joint velocity and acceleration estimated by a {OPTIONS.derivative_filter_hz:.0f} Hz second-order filter on the sampled joint angle (no noise here; noise is in `results/robustness.md`).

{OPTIONS.n_strides} strides are simulated and the last is evaluated. The controllers are the PID (`results/torque_pid.md`) and the PD + DOB (`results/torque_dob.md`), both with model feedforward.

## Check against the sizing model and the linear analysis

Level walking at 1.2 m/s, limits removed, exact joint derivatives. Each cell is simulation / reference: energy per stride and currents against the inverse model used for sizing (`anklesea.sea`), tracking error against the linear harmonic solution of `results/torque_pid.md`.

| design | energy per stride (J) | RMS current (A) | peak current (A) | RMS error (%) |
|---|---|---|---|---|
{chr(10).join(val_rows)}

The energies and currents agree to within a percent, so the sizing results hold for a controlled actuator that tracks well. The tracking errors are both a fraction of a percent; they differ because the simulation's delay is a sample-and-hold, not a pure delay.

## Compromise design, PID with model feedforward

RMS and peak error in % of the peak reference torque; saturation as % of the stride; currents and voltage from the last stride; energy with ideal regeneration.

| condition | RMS error (%) | peak error (%) | voltage saturated (%) | current saturated (%) | peak current (A) | RMS current (A) | peak voltage (V) | energy (J/stride) | RMS error within {TARGET_TRACKING_PCT:.0f} % |
|---|---|---|---|---|---|---|---|---|---|
{chr(10).join(main_rows)}

## Both designs and both controllers

RMS error in % of peak torque, with the share of the stride spent at the voltage or current limit in brackets.

| condition | compromise, PID | compromise, DOB | walking-tuned, PID | walking-tuned, DOB |
|---|---|---|---|---|
{chr(10).join(compare_rows)}

![Tracking](figures/tracking.png)

*Simulated torque, motor current and winding voltage over the last stride, PID with model feedforward. Dotted lines: driver limits ({drive.current_limit:.0f} A, {drive.voltage_limit:.1f} V); dash-dot: the motor's continuous current rating ({motor.continuous_current} A), which applies to the RMS current. {CITATION}*

## Reading the results

- The compromise design tracks level walking with {level_r.rms_error_pct():.2f} % RMS error, most of it from estimating the joint acceleration with a filter (with exact derivatives it is a few hundredths of a percent, table above). {"It reaches a drive limit in " + ", ".join(comp_sat) + "." if comp_sat else "It never reaches the voltage or current limit in any condition simulated."} Its worst case is {label(worst_key)} at {worst.rms_error_pct():.2f} % RMS. {"Every condition meets the 5 % tracking target." if not comp_fail else "The tracking target is missed in " + ", ".join(comp_fail) + "."}
- The walking-tuned design saturates in {", ".join(walk_sat) if walk_sat else "no condition"}. This is the time-domain view of what the sizing found (`results/cross_mode.md`): its high ratio needs more voltage than the driver has when the motor must spin fast. While the voltage is saturated the current cannot follow its reference, and the torque error grows until the motor slows down again.
- The DOB and the PID perform alike here, as the linear comparison predicted: with model feedforward there is little left for either to correct.
- Both designs are sized onto the voltage limit: the energy optimum of the walking-tuned design uses the full {drive.voltage_limit:.1f} V in level walking, and the compromise uses it in ramp descent (`results/cross_mode.md`). The inverse model needs exactly that voltage for perfect tracking, so any feedback correction or estimation error pushes the drive into saturation. A design for a real device should keep a voltage margin for control, for example by sizing against 85 to 90 % of the available voltage.
- The RMS current in level walking ({level_r.rms_current:.1f} A) is above the motor's continuous rating ({motor.continuous_current} A) for both designs. Tracking does not change this; it is the thermal limit found in the sizing (`results/sizing_modes.md`) and addressed by the parallel spring (`results/parallel_spring.md`).

## What did not work first

The first version of the simulation froze the PID integrator only when the current was clipped and fed the DOB the torque command it had computed. In {label(fast)} ({profiles[fast].n_subjects} subjects, a stress case beyond the conditions above), where the compromise design's drive is saturated for {100 * fast_runs['PID'].voltage_saturated:.0f} % of the stride, that gave:

| controller | RMS error, first version (%) | RMS error, final anti-windup (%) |
|---|---|---|
{chr(10).join(naive_rows)}

While the voltage is saturated, the current lags its reference; the DOB, comparing the measured torque with the *command*, sees the shortfall as a disturbance and adds it to the next command, which only deepens the saturation. Feeding the DOB the torque the motor actually produced (`k_t` times the measured current, which drives measure anyway) fixed the DOB. Freezing the PID integrator during voltage saturation as well helps the PID only a little, because its integrator is slow compared with a saturation episode.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
