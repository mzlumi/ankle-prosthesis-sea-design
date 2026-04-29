#!/usr/bin/env python3
"""Disturbance observer (Paine et al. 2014) compared with the PID of results/torque_pid.md; passivity check.

Writes:
  results/torque_dob.md
  results/figures/torque_dob.png
"""

from __future__ import annotations

from dataclasses import replace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from anklesea import RESULTS_DIR
from anklesea.designs import (
    INTEGRAL_FRACTION,
    LOOP_DELAY,
    PLACEMENT_HZ,
    TARGET_GM_DB,
    TARGET_MS,
    TARGET_PM_DEG,
    default_setup,
    meets_margin_targets,
    reference_designs,
)
from anklesea.plant import SEAParams
from anklesea.plots import ACTIVITY_NAMES, CITATION, add_citation
from anklesea.profiles import read_profiles
from anklesea.sizing import DESIGN_MASS_KG, MIX_ACTIVITIES, mix_conditions
from anklesea.torque_control import Controller, PIDGains, dob_cutoff, loop_metrics, periodic_tracking, tune_pid

REPORT = RESULTS_DIR / "torque_dob.md"
FIGURE = RESULTS_DIR / "figures" / "torque_dob.png"
HZ = 2 * np.pi
CUTOFFS_HZ = np.round(np.arange(1.0, 30.01, 0.5), 1)
MISMATCH = {
    "nominal": {},
    "spring 20 % softer": {"stiffness": 0.8},
    "friction x3": {"viscous_friction": 3.0},
    "gear efficiency -10 %": {"efficiency": 0.9},
}
CONFIG_STYLES = {
    "PID + model ff": ("#b45309", "-"),
    "PID + reference ff": ("#b45309", ":"),
    "PD + DOB + model ff": ("#1d4ed8", "-"),
    "PD + DOB + reference ff": ("#1d4ed8", ":"),
}


def perturbed(p: SEAParams, factors: dict[str, float]) -> SEAParams:
    return replace(p, **{key: getattr(p, key) * f for key, f in factors.items()})


def passivity(c: Controller, w: np.ndarray, delay: float) -> dict:
    """Smallest Re Z, where it occurs, and the frequency bands (Hz) where Re Z < 0."""
    z = c.closed_loop(w, None, "fixed", delay).impedance()
    re = np.real(z)
    tol = 1e-9 * max(1.0, float(np.abs(z).max()))
    neg = re < -tol
    edges = np.flatnonzero(np.diff(neg.astype(int)))
    starts = [0] if neg[0] else []
    starts += [e + 1 for e in edges if not neg[e]]
    ends = [e for e in edges if neg[e]] + ([len(w) - 1] if neg[-1] else [])
    bands = [(float(w[a] / HZ), float(w[b] / HZ)) for a, b in zip(starts, ends)]
    i = int(np.argmin(re))
    return {"min": float(re[i]) if neg.any() else 0.0, "at_hz": float(w[i] / HZ), "bands": bands,
            "lossless": bool(np.abs(z).max() < 1e-9)}


def band_text(bands: list[tuple[float, float]]) -> str:
    def fmt(x: float) -> str:
        return f"{x:.2g}" if x < 10 else f"{x:.0f}"

    return ", ".join(f"{fmt(lo)} to {fmt(hi)} Hz" for lo, hi in bands)


def main() -> int:
    motor, _ = default_setup()
    designs = reference_designs()
    params = {name: SEAParams.from_design(d, motor) for name, d in designs.items()}
    profiles = read_profiles()
    conditions = mix_conditions(profiles)
    w = np.logspace(-2, 4, 30000)

    # Q-filter cut-off: the highest that keeps the stance loop within the targets of results/torque_pid.md.
    sweep = {}
    cutoff = {}
    for name, p in params.items():
        pd = replace(tune_pid(p, PLACEMENT_HZ), ki=0.0)
        rows = []
        for fq in CUTOFFS_HZ:
            c = Controller(p, pd, "model", float(fq))
            st = loop_metrics(c.closed_loop(w, None, "fixed", LOOP_DELAY).loop, w)
            sw = loop_metrics(c.closed_loop(w, None, "free", LOOP_DELAY).loop, w)
            rows.append((fq, st, sw))
        sweep[name] = rows
        cutoff[name] = dob_cutoff(p, pd, LOOP_DELAY, meets_margin_targets, CUTOFFS_HZ)

    def configs(name: str) -> dict[str, Controller]:
        p = params[name]
        pid = tune_pid(p, PLACEMENT_HZ, integral_fraction=INTEGRAL_FRACTION)
        pd = replace(tune_pid(p, PLACEMENT_HZ), ki=0.0)
        return {
            "PID + model ff": Controller(p, pid, "model"),
            "PID + reference ff": Controller(p, pid, "reference"),
            "PD + DOB + model ff": Controller(p, pd, "model", cutoff[name]),
            "PD + DOB + reference ff": Controller(p, pd, "reference", cutoff[name]),
        }

    compare_rows, error_rows = [], []
    worst: dict[tuple[str, str, str], float] = {}
    for name, p in params.items():
        for label, c in configs(name).items():
            cells = []
            for output in ["fixed", "free"]:
                m = loop_metrics(c.closed_loop(w, None, output, LOOP_DELAY).loop, w)
                stable = c.is_stable(None, output, LOOP_DELAY)
                lower = "" if np.isinf(m.lower_gain_margin_db) else f", lower {m.lower_gain_margin_db:.0f}"
                cells.append(f"{'stable' if stable else '**unstable**'}, {m.phase_margin_deg:.0f} deg, "
                             f"{m.gain_margin_db:.0f} dB{lower}, {m.peak_sensitivity:.2f}")
            compare_rows.append(f"| {name} | {label} | {cells[0]} | {cells[1]} |")
            errs = []
            for case, factors in MISMATCH.items():
                actual = perturbed(p, factors)
                e = max(periodic_tracking(profiles[conditions[a]], DESIGN_MASS_KG, c, actual, LOOP_DELAY).rms_error_pct()
                        for a in MIX_ACTIVITIES)
                worst[(name, label, case)] = e
                errs.append(f"{e:.2f}")
            error_rows.append(f"| {name} | {label} | " + " | ".join(errs) + " |")

    # Joint-motion rejection: spring torque per radian of joint motion, as a share of k.
    rej_freqs = [1.0, 2.0, 4.0]
    rej_rows = []
    comp = params["compromise"]
    open_loop = Controller(comp, PIDGains(0.0, 0.0, 0.0, 1e-3), "none")
    rej_configs = {"motor torque held (no control)": open_loop, **configs("compromise")}
    for label, c in rej_configs.items():
        resp = c.closed_loop(HZ * np.array(rej_freqs), None, "fixed", LOOP_DELAY).torque_per_angle
        rej_rows.append(f"| {label} | " + " | ".join(f"{20 * np.log10(abs(x) / comp.stiffness):.0f}" for x in resp)
                        + " |")

    # Passivity (Vallery et al. 2007).
    pd = replace(tune_pid(comp, PLACEMENT_HZ), ki=0.0)
    pid = tune_pid(comp, PLACEMENT_HZ, integral_fraction=INTEGRAL_FRACTION)
    pas_configs = {
        "PD": Controller(comp, pd, "none"),
        "PID": Controller(comp, pid, "none"),
        "PD + DOB + reference ff": Controller(comp, pd, "reference", cutoff["compromise"]),
        "PD + model ff": Controller(comp, pd, "model"),
        "PID + model ff": Controller(comp, pid, "model"),
        "PD + DOB + model ff": Controller(comp, pd, "model", cutoff["compromise"]),
    }
    pas_rows = []
    pas = {}
    for label, c in pas_configs.items():
        cells = []
        for delay in [0.0, LOOP_DELAY]:
            r = passivity(c, w, delay)
            pas[(label, delay)] = r
            if r["lossless"]:
                cells.append("lossless (Z = 0)")
            elif r["min"] >= 0:
                cells.append("passive")
            else:
                cells.append(f"no: Re Z down to {r['min']:.3f} at {r['at_hz']:.1f} Hz; negative over {band_text(r['bands'])}")
        pas_rows.append(f"| {label} | {cells[0]} | {cells[1]} |")

    # Figure -------------------------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5), layout="constrained")
    ax = axes[0, 0]
    for name, color in [("compromise", "#b45309"), ("walking", "#1d4ed8")]:
        fq = [r[0] for r in sweep[name]]
        ax.plot(fq, [r[1].phase_margin_deg for r in sweep[name]], color=color, lw=1.8, label=f"{name}, stance")
        ax.plot(fq, [r[2].phase_margin_deg for r in sweep[name]], color=color, lw=1.0, ls="--", label=f"{name}, swing")
        ax.axvline(cutoff[name], color=color, ls=":", lw=1)
    ax.axhline(TARGET_PM_DEG, color="0.3", lw=0.8)
    ax.set(xlabel="Q-filter cut-off (Hz)", ylabel="phase margin (deg)",
           title="DOB cut-off against the phase margin (dotted: chosen)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    f = np.logspace(-1, 2.5, 3000)
    ax.loglog(f, np.abs(open_loop.closed_loop(HZ * f, None, "fixed", LOOP_DELAY).torque_per_angle), color="0.5",
              lw=2.5, label="motor torque held")
    for label, c in configs("compromise").items():
        color, ls = CONFIG_STYLES[label]
        ax.loglog(f, np.abs(c.closed_loop(HZ * f, None, "fixed", LOOP_DELAY).torque_per_angle), color=color, ls=ls,
                  lw=1.6, label=label)
    ax.set(xlabel="frequency (Hz)", ylabel="spring torque per joint angle (N·m/rad)",
           title="joint motion leaking into the torque, compromise design", ylim=(1e-2, 1e3))
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8)

    for ax, delay in zip(axes[1], [0.0, LOOP_DELAY]):
        for label in ["PD", "PID", "PD + DOB + reference ff", "PID + model ff"]:
            z = pas_configs[label].closed_loop(HZ * f, None, "fixed", delay).impedance()
            ax.semilogx(f, np.real(z), lw=1.6, label=label)
        ax.axhline(0, color="black", lw=0.8)
        ax.set_yscale("symlog", linthresh=0.01)
        ax.set(xlabel="frequency (Hz)", ylabel="Re Z (N·m·s/rad)",
               title=f"passivity: Re Z must stay at or above 0 ({'no delay' if delay == 0 else f'{delay * 1e3:.1f} ms delay'})")
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=8)
    add_citation(fig, CITATION)
    fig.savefig(FIGURE, dpi=150)
    plt.close(fig)

    def w_(label: str, case: str = "nominal", name: str = "compromise") -> float:
        return worst[(name, label, case)]

    pid_lossy = pas[("PID", 0.0)]
    dob_lossy = pas[("PD + DOB + reference ff", 0.0)]
    pid_ff_delay = pas[("PID + model ff", LOOP_DELAY)]
    pd_delay = pas[("PD", LOOP_DELAY)]
    text = f"""# Torque control: disturbance observer, and passivity

A disturbance observer (DOB) after N. Paine, S. Oh and L. Sentis, "Design and control considerations for high-performance series elastic actuators", IEEE/ASME Trans. Mechatronics 19(3):1080-1091, 2014, implemented from that description in `anklesea.torque_control` and compared with the PID of `results/torque_pid.md`. The DOB uses the nominal fixed-output model `Pn`: whatever makes the measured spring torque differ from what `Pn` predicts for the command it sent is treated as a disturbance at the motor and cancelled below the cut-off of a second-order low-pass `Q`. Because `Q(0) = 1` it acts as integral action, so it is paired with the same PD gains as the PID, without the integral. Every result includes the {LOOP_DELAY * 1e3:.1f} ms loop delay unless stated.

## Q-filter cut-off

The cut-off trades disturbance rejection against margin: the DOB inverts the nominal model up to the cut-off, and the loop delay is not in that model. The cut-off is the highest (on a 0.5 Hz grid) for which the stance loop still meets the targets of `results/torque_pid.md` (phase margin {TARGET_PM_DEG:.0f} deg, gain margin {TARGET_GM_DB:.0f} dB, `max |S|` {TARGET_MS:.0f}): **{cutoff['compromise']:.1f} Hz** for the compromise design and **{cutoff['walking']:.1f} Hz** for the walking-tuned one. That is below the bandwidth of the PID loop, so the DOB here works in the same frequency range as the integral action it replaces, not above it.

## Margins

Each cell: stability (closed-loop poles with a Pade approximation of the delay), phase margin, gain margin (and lower gain margin if the loop is conditionally stable), `max |S|`.

| design | controller | stance (fixed output) | swing (free output) |
|---|---|---|---|
{chr(10).join(compare_rows)}

"Reference ff" is the inverse model applied to the reference only; it needs no joint acceleration. "Model ff" adds the measured joint motion. With the DOB and model feedforward, the joint motion is put into the DOB's nominal model as well, so that it is not compensated twice.

## Tracking with model error

Worst RMS error over the five activities of the daily mix (% of peak torque), linear model with delay. The feedforward and the DOB keep using the nominal model while the plant changes.

| design | controller | {' | '.join(MISMATCH)} |
|---|---|{'---|' * len(MISMATCH)}
{chr(10).join(error_rows)}

![DOB results](figures/torque_dob.png)

*Top left: phase margin against the DOB cut-off. Top right: how much joint motion reaches the spring torque with zero torque reference, compromise design (the PID and DOB curves with model feedforward are close to each other). Bottom: real part of the interaction impedance with zero torque reference, without and with the loop delay (symmetric log scale). {CITATION}*

## Joint-motion rejection

Spring torque per unit joint motion relative to the spring stiffness (dB; 0 dB means the joint motion deflects the spring fully), compromise design, at the stride frequency and two harmonics.

| controller | {' | '.join(f'{x:.0f} Hz' for x in rej_freqs)} |
|---|{'---|' * len(rej_freqs)}
{chr(10).join(rej_rows)}

## Reading the comparison

- **With the joint acceleration available**, the DOB adds nothing measurable: PID and PD + DOB with model feedforward track to {w_('PID + model ff'):.2f} % and {w_('PD + DOB + model ff'):.2f} % and degrade alike under every model error (for example {w_('PID + model ff', 'spring 20 % softer'):.2f} % and {w_('PD + DOB + model ff', 'spring 20 % softer'):.2f} % with a 20 % softer spring). Both have integral action at low frequency, and the feedforward already removes almost everything the DOB would estimate.
- **Without the joint acceleration** (reference feedforward only), the joint motion is a real disturbance. The DOB at {cutoff['compromise']:.1f} Hz rejects it in the same band as the PID's integral term, and the two end up close: {w_('PID + reference ff'):.2f} % for the PID and {w_('PD + DOB + reference ff'):.2f} % for the DOB, nominal. A DOB pays off over an integral term when its cut-off can sit well above the band the integral term covers. Here the {LOOP_DELAY * 1e3:.1f} ms delay and the low open-loop resonance of a high-ratio design leave no room for that: above a few hertz the DOB eats the phase margin (top-left panel).
- In swing, the DOB's nominal model (joint held) is wrong by construction, and its margins there are lower than in stance. A practical controller would switch the DOB off in swing, as it would freeze the PID's integrator.

## Passivity

Checked numerically. The criterion is the one H. Vallery, R. Ekkelenkamp, H. van der Kooij and M. Buss, "Passive and accurate torque control of series elastic actuators", IEEE/RSJ IROS 2007, pp. 3534-3538, apply to SEA torque control: an actuator that is passive at its interaction port stays stable when coupled to any passive environment (J. E. Colgate and N. Hogan, "Robust control of dynamically interacting systems", Int. J. Control 48(1):65-88, 1988), which matters because the environment here is a person. With zero torque reference the joint sees the impedance `Z(s) = -tau_s / (s theta)`; the actuator is passive when the closed loop is stable and `Re Z(jw) >= 0` at every frequency. Compromise design, {w[0] / HZ:.4f} Hz to {w[-1] / HZ:.0f} Hz.

| controller | without delay | with {LOOP_DELAY * 1e3:.1f} ms delay |
|---|---|---|
{chr(10).join(pas_rows)}

- **Without delay**, PD torque control is passive. Integral action breaks passivity at low frequency, whether it comes from the PID's integral term (Re Z down to {pid_lossy['min']:.2f} N·m·s/rad, below {pid_lossy['bands'][-1][1]:.1f} Hz) or from the DOB ({dob_lossy['min']:.2f} N·m·s/rad, below {dob_lossy['bands'][-1][1]:.1f} Hz): the integrator keeps pushing the rotor while the user moves the joint, and over a slow cycle it can return more energy than the user put in. With exact model feedforward the actuator cancels the joint motion completely and `Z` is zero (lossless) in the nominal model.
- **With the {LOOP_DELAY * 1e3:.1f} ms delay** no configuration is strictly passive. For PD the violation is small and lies only above {pd_delay['bands'][0][0]:.0f} Hz (Re Z down to {pd_delay['min']:.4f} N·m·s/rad). With the model feedforward the delay turns the perfect cancellation into a small mismatch (Re Z down to {pid_ff_delay['min']:.3f} N·m·s/rad at {pid_ff_delay['at_hz']:.1f} Hz).
- The size of a violation is the damping that, added in parallel at the joint, would make the actuator passive. Vallery et al. compare several torque-control structures and conclude that a cascade with a fast inner motor-velocity loop is the best option, with parameter bounds under which the torque loop may keep integral action and stay passive. The controllers here command motor torque directly, with no velocity loop, and in that structure accuracy (integral action) and passivity pull in opposite directions. This study keeps the integral action and lists passivity as a limitation; the cascaded structure is the documented way out and is not implemented here.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
