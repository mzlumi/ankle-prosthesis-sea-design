#!/usr/bin/env python3
"""Size the SEA spring stiffness and gear ratio for level walking at 1.2 m/s.

Writes:
  results/sizing_levelwalk.md               optimum, comparison with a rigid actuator, sensitivity
  results/figures/sizing_levelwalk.png      energy contour over (k, N) with the limits

Load: mean right-ankle angle and moment of the dataset subjects on the treadmill at
1.2 m/s, scaled to the design user mass (docs/assumptions.md).
Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from anklesea import RESULTS_DIR
from anklesea.motor import default_motor
from anklesea.plots import add_citation, energy_contour
from anklesea.profiles import read_profiles
from anklesea.sea import Limits, joint_work
from anklesea.sizing import BUS_VOLTAGE, DESIGN_MASS_KG, LEVEL_WALK, SweepResult, sweep

FIGURE = RESULTS_DIR / "figures" / "sizing_levelwalk.png"
REPORT = RESULTS_DIR / "sizing_levelwalk.md"
CONSTRAINT = "drive_feasible"


def rigid_best(result: SweepResult, constraint: str | None) -> dict[str, float]:
    """Least-energy rigid actuator (k = inf column) over the ratios."""
    col = int(np.flatnonzero(np.isinf(result.stiffness))[0])
    allowed = result.mask(constraint)[:, col]
    energy = np.where(allowed, result.energy[:, col], np.inf)
    if not np.isfinite(energy).any():
        return {"stiffness": np.inf, "ratio": np.nan, "feasible": False}
    i = int(np.argmin(energy))
    out = {key: arr[i, col].item() for key, arr in result.metrics.items()}
    out.update(stiffness=np.inf, ratio=float(result.ratios[i]))
    return out


def describe(design: dict[str, float]) -> list[str]:
    if np.isnan(design["ratio"]):
        return ["none"] + [""] * 8
    k = "rigid" if np.isinf(design["stiffness"]) else f"{design['stiffness']:.0f}"
    return [
        k,
        f"{design['ratio']:.0f}",
        f"{design['energy']:.1f}",
        f"{design['energy_no_regen']:.1f}",
        f"{design['copper_energy']:.1f}",
        f"{design['peak_current']:.1f}",
        f"{design['rms_current']:.2f}",
        f"{design['peak_voltage']:.1f}",
        f"{design['peak_speed'] * 60 / (2 * np.pi):.0f}",
    ]


def violated(design: dict[str, float]) -> str:
    names = {"ok_voltage": "voltage", "ok_peak_current": "peak current", "ok_rms_current": "RMS current",
             "ok_speed": "speed"}
    if np.isnan(design["ratio"]):
        return ""
    bad = [label for key, label in names.items() if not design[key]]
    return ", ".join(bad) if bad else "none"


def sensitivity(profile, motor) -> list[list[str]]:
    """Drive-feasible optimum when one assumption changes at a time."""
    cases = [
        ("baseline", DESIGN_MASS_KG, BUS_VOLTAGE, motor, "constant"),
        ("user mass 60 kg", 60.0, BUS_VOLTAGE, motor, "constant"),
        ("user mass 100 kg", 100.0, BUS_VOLTAGE, motor, "constant"),
        ("bus voltage 24 V", DESIGN_MASS_KG, 24.0, motor, "constant"),
        ("bus voltage 48 V", DESIGN_MASS_KG, 48.0, motor, "constant"),
        ("gear efficiency 0.6", DESIGN_MASS_KG, BUS_VOLTAGE, motor.with_changes(gear_efficiency=0.6), "constant"),
        ("gear efficiency 0.9", DESIGN_MASS_KG, BUS_VOLTAGE, motor.with_changes(gear_efficiency=0.9), "constant"),
        ("direction-dependent efficiency", DESIGN_MASS_KG, BUS_VOLTAGE, motor, "directional"),
    ]
    rows = []
    for name, mass, voltage, mot, model in cases:
        result = sweep(profile.load(mass), mot, Limits.from_motor(mot, voltage), efficiency_model=model)
        opt = result.optimum(constraint=CONSTRAINT)
        rigid = rigid_best(result, CONSTRAINT)
        if np.isnan(opt["ratio"]):
            rows.append([name, "none", "", "", "", "", ""])
            continue
        rigid_e = "infeasible" if np.isnan(rigid["ratio"]) else f"{rigid['energy']:.1f}"
        rows.append([
            name,
            f"{opt['stiffness']:.0f} ({opt['stiffness'] / mass:.2f}/kg)",
            f"{opt['ratio']:.0f}",
            f"{opt['energy']:.1f}",
            rigid_e,
            f"{opt['rms_current']:.2f}",
            "yes" if result.feasible.any() else "no",
        ])
    return rows


def table(header: list[str], rows: list[list[str]]) -> str:
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(row) + " |" for row in rows]
    return "\n".join(lines)


def plot(result: SweepResult, opt: dict, unconstrained: dict, title: str) -> None:
    fig, ax = plt.subplots(figsize=(7.5, 5.6), constrained_layout=True)
    cs, handles = energy_contour(ax, result, vmax=150.0)
    fig.colorbar(cs, ax=ax, label="motor electrical energy per stride (J)", format="%.0f")
    ax.plot(opt["stiffness"], opt["ratio"], marker="*", color="white", markeredgecolor="black", markersize=16,
            linestyle="none", label=f"optimum within drive limits ({opt['energy']:.1f} J)")
    if (unconstrained["stiffness"], unconstrained["ratio"]) != (opt["stiffness"], opt["ratio"]):
        ax.plot(unconstrained["stiffness"], unconstrained["ratio"], marker="o", color="white",
                markeredgecolor="black", linestyle="none", label=f"optimum without limits ({unconstrained['energy']:.1f} J)")
    star_handles, _ = ax.get_legend_handles_labels()
    ax.legend(handles=star_handles + handles, loc="lower left", fontsize=8, framealpha=0.9)
    ax.set_title(title, fontsize=10, loc="left")
    add_citation(fig)
    FIGURE.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE, dpi=150)
    plt.close(fig)


def main() -> int:
    profiles = read_profiles()
    profile = profiles[LEVEL_WALK]
    motor = default_motor()
    limits = Limits.from_motor(motor, BUS_VOLTAGE)
    load = profile.load(DESIGN_MASS_KG)
    result = sweep(load, motor, limits)

    opt = result.optimum(constraint=CONSTRAINT)
    full = result.optimum(constraint="feasible")
    free = result.optimum(constraint=None)
    rigid = rigid_best(result, CONSTRAINT)
    rigid_free = rigid_best(result, None)
    work = joint_work(load)
    min_rms = float(result.metrics["rms_current"].min())
    drive_share = float(result.metrics["drive_feasible"].mean())

    plot(result, opt, free, f"Level walking at 1.2 m/s, {DESIGN_MASS_KG:.0f} kg user, {BUS_VOLTAGE:.0f} V bus: "
         f"the least-energy design\nwithin the drive limits is k = {opt['stiffness']:.0f} N·m/rad, N = {opt['ratio']:.0f}")

    header = ["design", "k (N·m/rad)", "N", "energy (J)", "energy, no regeneration (J)", "copper loss (J)",
              "peak current (A)", "RMS current (A)", "peak voltage (V)", "peak motor speed (rpm)", "limits broken"]
    designs = [
        ("SEA, least energy within the drive limits", opt),
        ("SEA, least energy with no limits", free),
        ("rigid, least energy with no limits", rigid_free),
    ]
    if not np.isnan(rigid["ratio"]):
        designs.append(("rigid, least energy within the drive limits", rigid))
    rows = [[name, *describe(d), violated(d)] for name, d in designs]
    sens = sensitivity(profile, motor)
    rigid_note = (
        "No rigid actuator (k = inf) meets the drive limits at any ratio from 30 to 2000. A rigid actuator needs a "
        "high ratio to keep the push-off current under 30 A, and at that ratio the back EMF at the push-off joint "
        "speed alone exceeds the available voltage. The spring lets the joint move during push-off while the motor "
        "turns more slowly, so the same ratio fits inside the voltage limit."
        if np.isnan(rigid["ratio"])
        else f"The best rigid actuator within the drive limits needs N = {rigid['ratio']:.0f} and "
        f"{rigid['energy']:.1f} J per stride, {rigid['energy'] / opt['energy'] - 1:.0%} more than the SEA. "
        "Without a spring the motor turns with the joint, so the back EMF at the push-off joint speed caps the ratio; "
        "at that low ratio the push-off torque needs a large current and the copper loss grows. The spring lets the "
        "joint move during push-off while the motor turns more slowly, so the SEA can use a higher ratio within the "
        "same voltage."
    )
    saving = 1.0 - opt["energy"] / rigid_free["energy"]
    k_kg = opt["stiffness"] / DESIGN_MASS_KG

    text = f"""# Sizing for level walking

Spring stiffness `k` and total gear ratio `N` of the series elastic actuator that minimize the motor's electrical energy per stride for level walking.

**Load.** Mean right-ankle angle and moment of {profile.n_subjects} subjects ({profile.n_strides} strides) walking on the treadmill at 1.2 m/s, stride time {profile.stride_time:.3f} s, moment scaled to an {DESIGN_MASS_KG:.0f} kg user. Joint work per stride: {work['net']:.1f} J net, {work['positive']:.1f} J positive. Peak joint torque {abs(load.torque).max():.0f} N·m, peak joint speed {abs(load.velocity).max():.2f} rad/s. Treadmill walking stands in for level walking because the instrumented belt gives a moment for every stride; overground, only the stances on a force plate have one (see `results/gait_profiles.md`).

**Actuator.** maxon EC-4pole 30 (200 W, 36 V winding), gearhead efficiency 0.70, ESCON 70/10 driver on a {BUS_VOLTAGE:.0f} V bus ({limits.available_voltage:.1f} V available), values in `data/motors/maxon_ec4pole30_305014.yaml`. Assumptions in `docs/assumptions.md`.

**Search.** Full energy model (`anklesea.sea`) on a grid of 121 stiffness values (50 to 20000 N·m/rad, log-spaced) plus the rigid actuator, by 121 ratios (30 to 2000). Energy counts ideal regeneration; the column without regeneration bounds the other extreme.

![Energy contour](figures/sizing_levelwalk.png)

*Motor electrical energy per stride over spring stiffness and gear ratio. Veiled designs break the voltage, peak current or speed limit. The continuous current limit is not drawn because no design on the grid meets it. Data: Camargo et al. (2021), CC BY 4.0.*

## Result

{table(header, rows)}

- The least-energy design within the drive limits is **k = {opt['stiffness']:.0f} N·m/rad ({k_kg:.2f} N·m/rad per kg of user mass), N = {opt['ratio']:.0f}**, at {opt['energy']:.1f} J per stride ({opt['energy'] / profile.stride_time:.1f} W average). That is {saving:.0%} less than the best rigid actuator even when the rigid one may ignore the drive limits.
- {rigid_note}
- Without any limits the optimum moves to N = {free['ratio']:.0f}; the voltage limit is what holds the ratio down. Energy is flat near the optimum: the contour shows a long valley along which `k` and `N` trade off.
- **No design meets the motor's continuous current rating.** The lowest stride RMS current anywhere on the grid is {min_rms:.2f} A against the {limits.rms_current:.2f} A rating ({min_rms / limits.rms_current - 1:.0%} above it), and the least-energy design draws {opt['rms_current']:.2f} A ({opt['rms_current'] / limits.rms_current - 1:.0%} above), so in sustained walking for an {DESIGN_MASS_KG:.0f} kg user this motor runs hotter than its continuous rating allows. Its copper loss is set mostly by the joint torque reflected through the gear, which no series spring can reduce. Section 11 returns to this with a parallel spring, which carries part of the torque itself.
- {drive_share:.0%} of the designs on the grid meet the drive limits.

## Sensitivity to the assumptions

Each row changes one assumption from `docs/assumptions.md` and repeats the search. "Rigid" is the best rigid actuator within the drive limits; "any fully feasible" says whether any design also meets the RMS current rating.

{table(["case", "k (N·m/rad)", "N", "energy (J)", "rigid energy (J)", "RMS current (A)", "any fully feasible"], sens)}

The optimal stiffness scales roughly with user mass (it stays near {k_kg:.1f} N·m/rad per kg): the joint torque scales with mass while the joint angles do not, so the spring that best cancels the motor motion during push-off, where `tau_dot / k` offsets `theta_dot`, scales with mass too. The bus voltage mainly moves the ratio, through the voltage limit. Gear efficiency changes the energy but hardly the design, except that charging the gear loss in the right direction (rather than dividing by `eta` throughout) favors a stiffer spring.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
