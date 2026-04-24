#!/usr/bin/env python3
"""PID torque control with model-based feedforward: targets, tuning, margins and linear tracking.

Writes:
  results/torque_pid.md
  results/figures/torque_pid.png
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
    INTEGRAL_FRACTION,
    LOOP_DELAY,
    PLACEMENT_HZ,
    TARGET_BANDWIDTH_HZ,
    TARGET_GM_DB,
    TARGET_MS,
    TARGET_PM_DEG,
    TARGET_TRACKING_PCT,
    default_setup,
    reference_designs,
)
from anklesea.plant import SEAParams
from anklesea.plots import ACTIVITY_NAMES, CITATION, add_citation
from anklesea.profiles import fourier_coefficients, read_profiles
from anklesea.sizing import DESIGN_MASS_KG, MIX_ACTIVITIES, mix_conditions
from anklesea.torque_control import FEEDFORWARDS, Controller, loop_metrics, periodic_tracking, pid_loop, tune_pid

REPORT = RESULTS_DIR / "torque_pid.md"
FIGURE = RESULTS_DIR / "figures" / "torque_pid.png"
HZ = 2 * np.pi

FIRST_TRY = (6.0, 0.1)  # placement and integral fraction of the first tuning, kept for the comparison
STYLES = {"compromise": ("#b45309", "-"), "walking": ("#1d4ed8", "--")}
FF_STYLES = {"none": ("#6b7280", ":"), "static": ("#1d4ed8", "--"), "model": ("#b45309", "-")}
FF_NAMES = {"none": "feedback only", "static": "feedback + static feedforward",
            "model": "feedback + model feedforward"}


def power_band(samples: np.ndarray, period: float, share: float, n_harmonics: int = 20) -> float:
    """Lowest frequency (Hz) below which ``share`` of the signal's AC power lies."""
    c = fourier_coefficients(samples, n_harmonics)[1:]
    cum = np.cumsum(np.abs(c) ** 2) / np.sum(np.abs(c) ** 2)
    return float((np.searchsorted(cum, share) + 1) / period)


def main() -> int:
    motor, _ = default_setup()
    designs = reference_designs()
    params = {name: SEAParams.from_design(d, motor) for name, d in designs.items()}
    profiles = read_profiles()
    conditions = mix_conditions(profiles)
    w = np.logspace(-1, 4, 40000)

    band_rows = []
    for activity in MIX_ACTIVITIES:
        gp = profiles[conditions[activity]]
        torque = gp.moment_per_kg
        band_rows.append(f"| {ACTIVITY_NAMES[activity]} ({conditions[activity][1]}) | "
                         f"{power_band(torque, gp.stride_time, 0.99):.1f} | "
                         f"{power_band(torque, gp.stride_time, 0.999):.1f} |")

    gains, stance, swing = {}, {}, {}
    for name, p in params.items():
        gains[name] = tune_pid(p, PLACEMENT_HZ, integral_fraction=INTEGRAL_FRACTION)
        stance[name] = loop_metrics(pid_loop(p, gains[name], w, LOOP_DELAY), w)
        swing[name] = loop_metrics(pid_loop(p, gains[name], w, LOOP_DELAY, output="free"), w)

    def ok(flag: bool) -> str:
        return "yes" if flag else "**no**"

    gain_rows, margin_rows = [], []
    for name, g in gains.items():
        gain_rows.append(f"| {name} | {g.kp:.4f} | {g.ki:.3f} | {g.kd * 1e3:.3f} | {g.derivative_filter * 1e3:.2f} | "
                         f"{g.kp / motor.torque_constant:.2f} |")
        for phase, m in [("stance (fixed output)", stance[name]), ("swing (free output)", swing[name])]:
            bw = f"{m.bandwidth_hz:.1f}" if phase.startswith("stance") else "n/a"
            meets = (m.phase_margin_deg >= TARGET_PM_DEG and min(m.gain_margin_db, m.lower_gain_margin_db) >= TARGET_GM_DB
                     and m.peak_sensitivity <= TARGET_MS
                     and (not phase.startswith("stance") or m.bandwidth_hz >= TARGET_BANDWIDTH_HZ))
            lower = "none" if np.isinf(m.lower_gain_margin_db) else f"{m.lower_gain_margin_db:.1f}"
            margin_rows.append(f"| {name} | {phase} | {bw} | {m.crossover_hz:.1f} | {m.phase_margin_deg:.0f} | "
                               f"{m.gain_margin_db:.1f} | {lower} | {m.peak_sensitivity:.2f} | {ok(meets)} |")

    # First tuning: met targets 1 and 2 but left a large error at the stride frequency.
    first_rows = []
    stride_hz = 1 / profiles[conditions["level"]].stride_time
    first_none_err = {}
    for name, p in params.items():
        for label, (placement, fraction) in [("first", FIRST_TRY), ("final", (PLACEMENT_HZ, INTEGRAL_FRACTION))]:
            g = tune_pid(p, placement, integral_fraction=fraction)
            loop = pid_loop(p, g, w, LOOP_DELAY)
            m = loop_metrics(loop, w)
            s_stride = np.abs(1 / (1 + loop[np.argmin(np.abs(w - HZ * stride_hz))]))
            t_dip = 20 * np.log10(np.abs(loop / (1 + loop))[w <= HZ * TARGET_BANDWIDTH_HZ].min())
            err = max(periodic_tracking(profiles[conditions[a]], DESIGN_MASS_KG, Controller(p, g, "none"), delay=LOOP_DELAY)
                      .rms_error_pct() for a in MIX_ACTIVITIES)
            first_none_err[(name, label)] = err
            first_rows.append(f"| {name} | {label} | {placement:.0f} | {fraction:.1f} | {m.bandwidth_hz:.1f} | "
                              f"{t_dip:.1f} | {20 * np.log10(s_stride):.1f} | {m.phase_margin_deg:.0f} | {err:.1f} |")

    tracking: dict[tuple[str, str, str], object] = {}
    track_rows = []
    for activity in MIX_ACTIVITIES:
        gp = profiles[conditions[activity]]
        cells = []
        for name in ["compromise", "walking"]:
            for ff in FEEDFORWARDS:
                tr = periodic_tracking(gp, DESIGN_MASS_KG, Controller(params[name], gains[name], ff), delay=LOOP_DELAY)
                tracking[(activity, name, ff)] = tr
                cells.append(f"{tr.rms_error_pct():.2f} / {tr.peak_error_pct():.1f}")
        track_rows.append(f"| {ACTIVITY_NAMES[activity]} ({conditions[activity][1]}) | " + " | ".join(cells) + " |")

    # Figure -----------------------------------------------------------------------------
    f = w / HZ
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    for name, p in params.items():
        color, ls = STYLES[name]
        loop = pid_loop(p, gains[name], w, LOOP_DELAY)
        t = loop / (1 + loop)
        axes[0, 0].semilogx(f, 20 * np.log10(np.abs(t)), color=color, ls=ls, lw=1.8,
                            label=f"{name}: bandwidth {stance[name].bandwidth_hz:.1f} Hz")
        axes[0, 1].semilogx(f, 20 * np.log10(np.abs(1 / (1 + loop))), color=color, ls=ls, lw=1.8,
                            label=f"{name}, stance")
        loop_sw = pid_loop(p, gains[name], w, LOOP_DELAY, output="free")
        axes[0, 1].semilogx(f, 20 * np.log10(np.abs(1 / (1 + loop_sw))), color=color, ls=ls, lw=0.9, alpha=0.6,
                            label=f"{name}, swing")
    axes[0, 0].axhline(-3, color="0.5", lw=0.8)
    axes[0, 0].axvline(TARGET_BANDWIDTH_HZ, color="0.3", ls=":", lw=1)
    axes[0, 0].text(TARGET_BANDWIDTH_HZ * 1.05, -25, "target", fontsize=8, color="0.3")
    axes[0, 0].set(xlim=(0.1, 300), ylim=(-30, 6), xlabel="frequency (Hz)", ylabel="|T| (dB)",
                   title="closed-loop torque response, feedback only (stance)")
    axes[0, 1].axhline(20 * np.log10(TARGET_MS), color="0.3", ls=":", lw=1)
    axes[0, 1].set(xlim=(0.1, 300), ylim=(-40, 10), xlabel="frequency (Hz)", ylabel="|S| (dB)",
                   title="sensitivity (target: below the dotted line)")
    for ax in axes[0]:
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=8)
    for ax, activity in zip(axes[1], ["level", "stairascent"]):
        ref = tracking[(activity, "compromise", "none")]
        pct = 100 * ref.t / ref.t[-1]
        for ff in FEEDFORWARDS:
            tr = tracking[(activity, "compromise", ff)]
            color, ls = FF_STYLES[ff]
            ax.plot(pct, tr.error, color=color, ls=ls, lw=1.6, label=f"{FF_NAMES[ff]} ({tr.rms_error_pct():.2f} % RMS)")
        ax2 = ax.twinx()
        ax2.plot(pct, ref.reference, color="0.75", lw=3, zorder=0)
        ax2.set_ylabel("reference torque (N·m, grey)", color="0.5")
        ax.set_zorder(ax2.get_zorder() + 1)
        ax.patch.set_visible(False)
        ax.set(xlabel="gait cycle (%)", ylabel="torque error (N·m)",
               title=f"{ACTIVITY_NAMES[activity]} ({conditions[activity][1]}), compromise design")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="lower left")
    add_citation(fig, CITATION)
    fig.savefig(FIGURE, dpi=150)
    plt.close(fig)

    comp_s, comp_sw = stance["compromise"], swing["compromise"]
    walk_sw = swing["walking"]
    worst_model = max(tracking[(a, "compromise", "model")].rms_error_pct() for a in MIX_ACTIVITIES)
    worst_static = max(tracking[(a, "compromise", "static")].rms_error_pct() for a in MIX_ACTIVITIES)
    level = profiles[conditions["level"]]
    none_level = tracking[("level", "compromise", "none")].rms_error_pct()
    still = periodic_tracking(replace(level, angle=np.zeros_like(level.angle)), DESIGN_MASS_KG,
                              Controller(params["compromise"], gains["compromise"], "none"), delay=LOOP_DELAY).rms_error_pct()
    text = f"""# Torque control: PID with model-based feedforward

Controller for the spring torque of the two reference designs (`anklesea.torque_control`), designed on the linear plant of `results/plant.md`. The controller runs at {CONTROL_RATE_HZ:.0f} Hz; its delay (one sample of computation plus half a sample of zero-order hold, {LOOP_DELAY * 1e3:.1f} ms) is included in every margin and tracking result. The motor's current loop is assumed ideal here and is modeled, with saturation, in `results/tracking.md`.

## Targets (set before tuning)

The torque reference comes from the gait data, so the bandwidth target does too: the frequency below which almost all of the torque signal's power lies (stride-averaged torque of each activity in the daily mix, `results/cross_mode.md`).

| activity | 99 % of torque power below (Hz) | 99.9 % below (Hz) |
|---|---|---|
{chr(10).join(band_rows)}

1. **Bandwidth** (feedback alone, stance, -3 dB of the closed loop): at least {TARGET_BANDWIDTH_HZ:.0f} Hz. This covers 99.9 % of the torque power in every activity except stair descent, whose landing transient is sharper.
2. **Margins** with the {LOOP_DELAY * 1e3:.1f} ms loop delay, in stance and in swing: phase margin at least {TARGET_PM_DEG:.0f} deg, gain margin at least {TARGET_GM_DB:.0f} dB (in both directions if the loop is conditionally stable), peak sensitivity `max |S|` at most {TARGET_MS:.0f} (6 dB).
3. **Zero steady-state error** (integral action).
4. **Tracking**: RMS error at most {TARGET_TRACKING_PCT:.0f} % of the peak torque in every activity, judged in the nonlinear simulation with voltage and current limits (`results/tracking.md`). The linear results below are the best case.

## Tuning

PD by pole placement on the fixed-output plant (derivation in the module docstring of `anklesea.torque_control`): the closed-loop poles are put at {PLACEMENT_HZ:.0f} Hz with damping 0.7, the integral corner at {INTEGRAL_FRACTION} of that frequency and the derivative filter at ten times it. The proportional gain raises the spring-rotor resonance from about 3 Hz to {PLACEMENT_HZ:.0f} Hz; the derivative gain damps it. The gains are tiny in N·m per N·m because the gear multiplies motor torque by `N eta`, about 300.

| design | Kp (N·m/N·m) | Ki (1/s) | Kd (ms) | derivative filter (ms) | Kp in A per N·m of error |
|---|---|---|---|---|---|
{chr(10).join(gain_rows)}

The first tuning put the poles at {FIRST_TRY[0]:.0f} Hz with the integral corner at {FIRST_TRY[1]} of that. It met targets 1 and 2, but only just: for the compromise design the closed-loop response dipped almost to -3 dB just above the stride frequency ({stride_hz:.2f} Hz), and feedback alone left {first_none_err[("compromise", "first")]:.0f} % RMS error. Bandwidth measured at -3 dB says nothing about how flat the response is below it, and a gait torque has most of its power at the first few stride harmonics, so what matters is the loop gain there. A higher placement and integral corner raise it; the integral corner cannot go much higher, because an integrator ahead of a lightly damped resonance makes the loop conditionally stable (it would turn unstable if its gain dropped), which `loop_metrics` reports as a lower gain margin.

| design | tuning | pole placement (Hz) | integral corner / placement | bandwidth (Hz) | lowest \\|T\\| up to {TARGET_BANDWIDTH_HZ:.0f} Hz (dB) | \\|S\\| at the stride frequency (dB) | phase margin (deg) | worst RMS error, feedback only (%) |
|---|---|---|---|---|---|---|---|---|
{chr(10).join(first_rows)}

## Margins and bandwidth

| design | phase of gait | bandwidth (Hz) | gain crossover (Hz) | phase margin (deg) | gain margin (dB) | lower gain margin (dB) | max \\|S\\| | meets targets 1 and 2 |
|---|---|---|---|---|---|---|---|---|
{chr(10).join(margin_rows)}

![PID results](figures/torque_pid.png)

*Top: closed-loop torque response and sensitivity with the {LOOP_DELAY * 1e3:.1f} ms delay. Bottom: steady-state tracking error over one stride for the compromise design, linear model; the grey line is the reference torque (scale on the right). {CITATION}*

The closed-loop bandwidth, {comp_s.bandwidth_hz:.1f} Hz for the compromise design, is higher than the {PLACEMENT_HZ:.0f} Hz pole placement because the derivative term adds a zero. In stance the margins are comfortable. In swing the same gains act on the free-output plant, whose sharp resonance at the foot ({params['compromise'].free_resonance / HZ:.0f} Hz) moves the gain crossover up to {comp_sw.crossover_hz:.0f} Hz, where the delay costs more phase: the phase margin falls to {comp_sw.phase_margin_deg:.0f} deg for the compromise design and {walk_sw.phase_margin_deg:.0f} deg for the walking-tuned one, which misses target 2. The closed loop stays stable, but swing is where a delay or gain error would first cause trouble, and a gain schedule that lowers the gains in swing (where the torque reference is near zero anyway) would restore the margin. The closed loop also has a pole at zero in swing: the integrator cancels the free plant's zero at DC, since a free foot cannot hold a steady torque. In practice the integrator is reset or frozen in swing.

## Linear tracking

Steady-state periodic tracking of the mean stride of each activity, computed harmonic by harmonic (exact for the linear model; checked against a 30-stride time simulation in `tests/test_torque_control.py`). Each cell is RMS error in % of the peak torque / peak error in %. The feedforward options are feedback alone, the static term `tau_ref / (N eta)` only, and the full inverse model using the measured joint motion.

| activity | compromise: feedback only | compromise: + static ff | compromise: + model ff | walking-tuned: feedback only | walking-tuned: + static ff | walking-tuned: + model ff |
|---|---|---|---|---|---|---|
{chr(10).join(track_rows)}

With feedback alone the compromise design tracks level walking with {none_level:.1f} % RMS error; with the joint held still it would be {still:.1f} %, so most of the error comes from the reference itself (finite loop gain at the stride harmonics), not from the joint motion. Reflecting the reference through the gear (static feedforward) brings the worst case to {worst_static:.1f} %. The full model feedforward cancels the plant dynamics and the joint motion and leaves only the effect of the loop delay, {worst_model:.2f} % RMS at worst. Without delay it would be exact, which `tests/test_torque_control.py` checks. These numbers assume a perfect model, unlimited voltage and current, and noise-free joint acceleration; `results/tracking.md` and `results/robustness.md` remove those assumptions one at a time.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
