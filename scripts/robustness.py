#!/usr/bin/env python3
"""Robustness of the torque controllers: model errors, sensor noise, delay, foot inertia, coupled stability.

Writes:
  results/robustness.md
  results/figures/robustness.png
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
    PLACEMENT_HZ,
    TARGET_PM_DEG,
    TARGET_TRACKING_PCT,
    default_setup,
    reference_controllers,
    reference_designs,
)
from anklesea.plant import SEAParams
from anklesea.plots import CITATION, add_citation
from anklesea.profiles import read_profiles
from anklesea.simulation import Drive, SimOptions, simulate
from anklesea.sizing import BUS_VOLTAGE, DESIGN_MASS_KG, MIX_ACTIVITIES, mix_conditions
from anklesea.torque_control import Controller, Environment, coupled_stable, loop_metrics, tune_pid

REPORT = RESULTS_DIR / "robustness.md"
FIGURE = RESULTS_DIR / "figures" / "robustness.png"
HZ = 2 * np.pi
TS = 1 / CONTROL_RATE_HZ
ENCODER_Q = 2 * np.pi / 2**14  # resolution of a 14-bit absolute encoder (an assumption, no encoder chosen)
QUANT_NOISE = ENCODER_Q / np.sqrt(12)
ENV_INERTIA = [0.01, 0.1, 1.0, 10.0]
ENV_STIFFNESS = [0.0, 30.0, 100.0, 300.0, 1000.0, 3000.0, 10000.0]
ENV_DAMPING_RATIO = 0.05


def env_grid() -> list[Environment]:
    out = []
    for j in ENV_INERTIA:
        for k in ENV_STIFFNESS:
            b = 2 * ENV_DAMPING_RATIO * np.sqrt(j * max(k, 1.0))
            out.append(Environment(j, b, k))
    return out


def main() -> int:
    motor, _ = default_setup()
    designs = reference_designs()
    comp = SEAParams.from_design(designs["compromise"], motor)
    ctrls = reference_controllers(comp)
    drive = Drive.from_motor(motor, BUS_VOLTAGE)
    profiles = read_profiles()
    conditions = mix_conditions(profiles)
    keys = [conditions[a] for a in MIX_ACTIVITIES]
    base = SimOptions()

    # Time-domain cases: (label, actual plant, drive, options).
    cases = [("nominal", comp, drive, base)]
    for f in [0.8, 0.9, 1.1, 1.2]:
        cases.append((f"spring stiffness x{f}, deflection sensor", replace(comp, stiffness=f * comp.stiffness), drive,
                      base))
    for f in [0.8, 1.2]:
        cases.append((f"spring stiffness x{f}, load cell", replace(comp, stiffness=f * comp.stiffness), drive,
                      replace(base, torque_sensor="load_cell")))
    for f in [0.9, 1.1]:
        cases.append((f"torque constant x{f}", replace(comp, torque_constant=f * comp.torque_constant),
                      replace(drive, torque_constant=f * drive.torque_constant), base))
    cases.append(("rotor inertia x1.3", replace(comp, rotor_inertia=1.3 * comp.rotor_inertia), drive, base))
    cases.append(("friction x3", replace(comp, viscous_friction=3 * comp.viscous_friction), drive, base))
    for mult in [1, 10, 30]:
        cases.append((f"joint angle noise {mult * QUANT_NOISE * 1e3:.2f} mrad RMS", comp, drive,
                      replace(base, angle_noise=mult * QUANT_NOISE)))
    cases.append(("torque noise 1 N·m RMS", comp, drive, replace(base, torque_noise=1.0)))
    for extra in [1, 2, 4]:
        cases.append((f"loop delay +{extra} ms", comp, drive, replace(base, extra_delay_samples=extra)))

    # Simulate the plant changes with the controller's nominal model: the plant's torque
    # constant enters through the drive; SEAParams carries the rest.
    sim_rows = []
    worst = {}
    for label, actual, drv, opt in cases:
        cells = []
        for cname, c in ctrls.items():
            errs, sats = [], []
            for key in keys:
                r = simulate(profiles[key], DESIGN_MASS_KG, c, drv, actual=actual, options=opt)
                errs.append(r.rms_error_pct())
                sats.append(max(r.voltage_saturated, r.current_saturated))
            i = int(np.argmax(errs))
            worst[(label, cname)] = (errs[i], keys[i])
            cells.append(f"{errs[i]:.2f} ({keys[i][0].replace('ramp', 'ramp ').replace('stair', 'stair ')}) | "
                         f"{100 * max(sats):.0f}")
        mark = "" if worst[(label, "PID")][0] <= TARGET_TRACKING_PCT else " **"
        sim_rows.append(f"| {label}{mark} | " + " | ".join(cells) + " |")

    # Linear margins.
    w = np.logspace(-2, 4, 30000)
    lin_cases = [("nominal", comp, LOOP_DELAY)]
    for f in [0.8, 1.2]:
        lin_cases.append((f"spring stiffness x{f}", replace(comp, stiffness=f * comp.stiffness), LOOP_DELAY))
    for f in [0.9, 1.1]:
        lin_cases.append((f"torque constant x{f} (loop gain x{f})", replace(comp, efficiency=f * comp.efficiency,
                                                                         rotor_inertia=comp.rotor_inertia / f,
                                                                         viscous_friction=comp.viscous_friction / f),
                          LOOP_DELAY))
    for jl in [0.005, 0.03]:
        lin_cases.append((f"foot inertia {jl} kg m²", replace(comp, load_inertia=jl), LOOP_DELAY))
    for extra in [1, 2, 4]:
        lin_cases.append((f"loop delay {(1.5 + extra):.1f} ms", comp, LOOP_DELAY + extra * TS))
    lin_rows = []
    delay_cases: dict[str, list[tuple[float, float, bool]]] = {name: [] for name in ctrls}
    for label, actual, delay in lin_cases:
        cells = []
        for cname, c in ctrls.items():
            st = loop_metrics(c.closed_loop(w, actual, "fixed", delay).loop, w)
            sw = loop_metrics(c.closed_loop(w, actual, "free", delay).loop, w)
            stable = c.is_stable(actual, "fixed", delay) and c.is_stable(actual, "free", delay)
            cells.append(f"{st.phase_margin_deg:.0f} / {sw.phase_margin_deg:.0f}{'' if stable else ' **unstable**'}")
            if label.startswith("loop delay"):
                delay_cases[cname].append((delay, sw.phase_margin_deg, stable))
        lin_rows.append(f"| {label} | " + " | ".join(cells) + " |")
    st_nom = loop_metrics(ctrls["PID"].closed_loop(w, None, "fixed", LOOP_DELAY).loop, w)
    sw_nom = loop_metrics(ctrls["PID"].closed_loop(w, None, "free", LOOP_DELAY).loop, w)
    delay_margin_stance = np.radians(st_nom.phase_margin_deg) / (HZ * st_nom.crossover_hz)
    delay_margin_swing = np.radians(sw_nom.phase_margin_deg) / (HZ * sw_nom.crossover_hz)
    dob_unstable = [d for d, _, ok in delay_cases["DOB"] if not ok]
    worst_delay = min(dob_unstable) if dob_unstable else max(d for d, _, _ in delay_cases["DOB"])
    pid_swing_pm = next(pm for d, pm, _ in delay_cases["PID"] if np.isclose(d, worst_delay))
    delay_text = (
        f"at {worst_delay * 1e3:.1f} ms the PID's swing margin is down to {pid_swing_pm:.0f} deg and "
        + ("the DOB's swing loop is unstable." if dob_unstable else "both swing loops are still stable.")
    )

    # Coupled stability with passive environments.
    envs = env_grid()
    pd = replace(tune_pid(comp, PLACEMENT_HZ), ki=0.0)
    env_ctrls = {"PD (passive without delay)": Controller(comp, pd, "none"), "PID + model ff": ctrls["PID"],
                 "PD + DOB + model ff": ctrls["DOB"]}
    w_env = np.concatenate([[1e-6], np.logspace(-3, 5.5, 300000)])
    env_maps = {}
    env_rows = []
    for cname, c in env_ctrls.items():
        h_cache = c.closed_loop(w_env, None, "fixed", LOOP_DELAY).torque_per_angle

        def h(x: np.ndarray, cache=h_cache) -> np.ndarray:
            return cache

        stable = np.array([coupled_stable(h, e, w_env) for e in envs]).reshape(len(ENV_INERTIA), len(ENV_STIFFNESS))
        env_maps[cname] = stable
        bad = [f"J = {ENV_INERTIA[i]}, k = {ENV_STIFFNESS[j]:.0f}" for i, j in zip(*np.nonzero(~stable))]
        env_rows.append(f"| {cname} | {stable.sum()} of {stable.size} | {'; '.join(bad) if bad else 'none'} |")

    # Finer look at the light inertia, and the environment damping that would restore stability.
    h_pid = ctrls["PID"].closed_loop(w_env, None, "fixed", LOOP_DELAY).torque_per_angle
    light = ENV_INERTIA[0]
    k_scan = np.logspace(0, 3, 61)
    unstable_k = [k for k in k_scan
                  if not coupled_stable(lambda x: h_pid, Environment(light, 2 * ENV_DAMPING_RATIO * np.sqrt(light * k), k),
                                        w_env)]

    def damping_needed(k: float) -> float:
        lo, hi = 0.0, 10.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if coupled_stable(lambda x: h_pid, Environment(light, mid, k), w_env):
                hi = mid
            else:
                lo = mid
        return hi

    needed = {k: damping_needed(k) for k in unstable_k}
    worst_k = max(needed, key=needed.get) if needed else None
    b_needed = needed[worst_k] if needed else 0.0

    # Noise: the level at which the PID's error doubles.
    noise_levels = np.array([0, 1, 3, 10, 30, 60]) * QUANT_NOISE

    # Figure ------------------------------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5), layout="constrained")
    level = profiles[keys[0]]
    factors = np.linspace(0.75, 1.25, 11)
    for sensor, color in [("deflection", "#b45309"), ("load_cell", "#1d4ed8")]:
        errs = [simulate(level, DESIGN_MASS_KG, ctrls["PID"], drive, actual=replace(comp, stiffness=f * comp.stiffness),
                         options=replace(base, torque_sensor=sensor)).rms_error_pct() for f in factors]
        axes[0, 0].plot(factors, errs, "o-", color=color, label=f"torque from {sensor.replace('_', ' ')}")
    axes[0, 0].axhline(TARGET_TRACKING_PCT, color="0.3", ls=":", lw=1)
    axes[0, 0].set(xlabel="actual / assumed spring stiffness", ylabel="RMS error (% of peak torque)",
                   title="stiffness error, level walking, PID")
    axes[0, 0].legend(fontsize=8)
    noise = noise_levels
    noise_err = {}
    for cname, color in [("PID", "#b45309"), ("DOB", "#1d4ed8")]:
        errs = [simulate(level, DESIGN_MASS_KG, ctrls[cname], drive, options=replace(base, angle_noise=n)).rms_error_pct()
                for n in noise]
        noise_err[cname] = errs
        axes[0, 1].plot(noise * 1e3, errs, "o-", color=color, label=cname)
    axes[0, 1].set_xscale("symlog", linthresh=0.1)
    axes[0, 1].axhline(TARGET_TRACKING_PCT, color="0.3", ls=":", lw=1)
    axes[0, 1].set(xlabel="joint angle noise (mrad RMS)", ylabel="RMS error (% of peak torque)",
                   title="encoder noise, level walking")
    axes[0, 1].legend(fontsize=8)
    delays = np.linspace(0.5e-3, 10e-3, 40)
    for cname, color in [("PID", "#b45309"), ("DOB", "#1d4ed8")]:
        for output, ls in [("fixed", "-"), ("free", "--")]:
            pm = [loop_metrics(ctrls[cname].closed_loop(w, None, output, d).loop, w).phase_margin_deg for d in delays]
            axes[1, 0].plot(delays * 1e3, pm, color=color, ls=ls, label=f"{cname}, {'stance' if output == 'fixed' else 'swing'}")
    axes[1, 0].axhline(TARGET_PM_DEG, color="0.3", ls=":", lw=1)
    axes[1, 0].axhline(0, color="black", lw=0.8)
    axes[1, 0].axvline(LOOP_DELAY * 1e3, color="0.5", lw=0.8)
    axes[1, 0].set(xlabel="loop delay (ms)", ylabel="phase margin (deg)", title="delay")
    axes[1, 0].legend(fontsize=8)
    m = env_maps["PID + model ff"]
    axes[1, 1].imshow(m, cmap="RdYlGn", vmin=0, vmax=1, aspect="auto")
    axes[1, 1].set_xticks(range(len(ENV_STIFFNESS)), [f"{k:g}" for k in ENV_STIFFNESS])
    axes[1, 1].set_yticks(range(len(ENV_INERTIA)), [f"{j:g}" for j in ENV_INERTIA])
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            axes[1, 1].text(j, i, "stable" if m[i, j] else "unstable", ha="center", va="center", fontsize=7)
    axes[1, 1].set(xlabel="environment stiffness (N·m/rad)", ylabel="environment inertia (kg m²)",
                   title="PID + model ff coupled to a passive environment")
    for ax in axes.flat[:3]:
        ax.grid(alpha=0.3)
    add_citation(fig, CITATION)
    fig.savefig(FIGURE, dpi=150)
    plt.close(fig)

    doubled = [n for n, e in zip(noise_levels, noise_err["PID"]) if e >= 2 * noise_err["PID"][0]]
    noise_text = (f"the PID's level-walking error doubles at {doubled[0] * 1e3:.1f} mrad RMS, "
                  f"{doubled[0] / QUANT_NOISE:.0f} times the quantization noise" if doubled else
                  "the PID's level-walking error does not double even at the largest level tried")
    nominal_err = worst[("nominal", "PID")][0]
    soft_def = worst[("spring stiffness x0.8, deflection sensor", "PID")][0]
    soft_cell = worst[("spring stiffness x0.8, load cell", "PID")][0]
    text = f"""# Robustness of the torque controllers

Compromise design (k = {comp.stiffness:.0f} N·m/rad, N = {comp.ratio:.0f}), with the PID and the PD + DOB of `results/torque_pid.md` and `results/torque_dob.md`, both with model feedforward. The controllers keep their nominal model while the plant, the sensors or the timing change. Time-domain results come from the nonlinear simulation of `results/tracking.md` (drive limits included); margins from the linear model.

## Tracking under model errors, noise and delay

Worst RMS error over the five activities of the daily mix (% of peak torque, the activity in brackets) and the largest share of a stride spent at a drive limit (%). Rows marked ** miss the {TARGET_TRACKING_PCT:.0f} % target with the PID.

| case | PID: worst RMS error (%) | PID: saturated (%) | DOB: worst RMS error (%) | DOB: saturated (%) |
|---|---|---|---|---|
{chr(10).join(sim_rows)}

Noise levels: {QUANT_NOISE * 1e3:.2f} mrad RMS is the quantization noise of a 14-bit absolute joint encoder (resolution {ENCODER_Q * 1e3:.2f} mrad, an assumed sensor); the larger values stand for a coarser or noisier sensor. The angle noise enters twice: through the torque, measured as the difference between the motor and joint encoders, and through the joint acceleration estimated for the feedforward.

## Margins under parameter errors and delay

Phase margin in stance / swing (deg) with the {LOOP_DELAY * 1e3:.1f} ms loop delay unless stated; the loop is marked if it is unstable.

| case | PID | PD + DOB |
|---|---|---|
{chr(10).join(lin_rows)}

A torque-constant error scales the loop gain; it is modeled here as a matching change of the efficiency with the reflected inertia and friction held. The delay margin of the PID, the extra pure delay that would use up the phase margin, is {delay_margin_stance * 1e3:.1f} ms in stance and {delay_margin_swing * 1e3:.1f} ms in swing.

## Coupled stability with passive environments

The tracking simulations prescribe the joint motion. In use, the joint is moved by the user's body and the ground, and the actuator must stay stable whatever that environment is. `results/torque_dob.md` found that no controller is strictly passive with the loop delay, so passivity cannot guarantee it; instead, the actuator's joint-motion response is coupled to a grid of passive environments `J_e theta_ddot + b_e theta_dot + k_e theta = tau_s` (inertia {", ".join(f"{j:g}" for j in ENV_INERTIA)} kg m²; stiffness {", ".join(f"{k:g}" for k in ENV_STIFFNESS)} N·m/rad; damping ratio {ENV_DAMPING_RATIO}) and checked with the Nyquist criterion (`anklesea.torque_control.coupled_stable`). The grid spans a free foot (0.01 kg m², no stiffness) to a joint held by the whole body through a stiff socket.

| controller | stable cases | unstable cases |
|---|---|---|
{chr(10).join(env_rows)}

![Robustness](figures/robustness.png)

*Top left: tracking error against the ratio of actual to assumed spring stiffness, with the torque measured from the spring deflection (as designed) or by a load cell. Top right: tracking error against joint-angle noise. Bottom left: phase margin against loop delay (grey line: the nominal {LOOP_DELAY * 1e3:.1f} ms). Bottom right: coupled stability of the PID with model feedforward over the environment grid. {CITATION}*

## Reading the results

- **Spring stiffness is the parameter that matters most, and through the sensor, not the controller.** The torque is computed as nominal stiffness times deflection, so a spring 20 % softer than assumed reads 25 % high and the loop delivers 20 % too little torque: the worst error grows from {nominal_err:.1f} % to {soft_def:.1f} %. With a load cell the same spring error costs almost nothing ({soft_cell:.1f} %). The spring must be calibrated on the assembled actuator, and its stiffness checked for drift with temperature and wear.
- **Motor constants** (torque constant, rotor inertia, friction) change the feedforward and the loop gain; the feedback absorbs these errors and the tracking stays within the target.
- **Noise** matters through the joint acceleration estimate: the feedforward multiplies it by the reflected inertia, so encoder noise becomes torque noise and, at high levels, voltage saturation (the saturated share of the stride grows with the noise in the table). At the quantization level of a 14-bit encoder the effect is invisible; {noise_text}. A lower derivative filter cut-off would trade noise for lag.
- **Delay** costs phase margin at about {np.degrees(HZ * st_nom.crossover_hz * 1e-3):.0f} deg per millisecond in stance and {np.degrees(HZ * sw_nom.crossover_hz * 1e-3):.0f} deg per millisecond in swing, where the crossover is higher. Swing therefore sets the delay budget: {delay_text} The tracking simulation does not show this, because it prescribes the joint motion and so never has a free foot; the extra-delay rows of the first table only describe stance-like tracking.
- **Coupled stability**: PD alone is stable with every environment in the grid. With the model feedforward and the delay, both PID and DOB destabilize a light inertia ({light} kg m², a foot) on a soft, lightly damped spring: in a finer scan (1 to 1000 N·m/rad) the coupled system is unstable for every stiffness from the lowest tried, {min(unstable_k):.0f}, up to {max(unstable_k):.0f} N·m/rad. These environments resonate at {np.sqrt(min(unstable_k) / light) / HZ:.0f} to {np.sqrt(max(unstable_k) / light) / HZ:.0f} Hz, the band where `results/torque_dob.md` found the delayed feedforward makes the actuator non-passive. Environment damping of {b_needed:.3f} N·m·s/rad restores stability over that whole range (the most demanding case is {worst_k:.0f} N·m/rad), the same order as the passivity violation found there. Whether a real foot meets such an environment (lightly held by something soft, in swing) is not established here; the safe reading is that the joint-motion feedforward should be faded out in swing, together with the integrator, as PD alone is stable with every environment tried.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
