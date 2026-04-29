#!/usr/bin/env python3
"""Size the SEA separately for every activity and condition of the dataset.

Writes:
  results/sizing_modes.md                       design table per activity and condition
  results/profiles/sizing_modes.csv             the same table as CSV (aggregate data only)
  results/figures/sizing_modes_contour.png      energy contour per activity with each optimum
  results/figures/sizing_by_condition.png       optimal k and N against speed, incline, step height

Every optimum uses the drive limits (voltage, peak current, speed); see results/convex_check.md
for the method (ratio scan with the clipped closed form in compliance).
Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0.
"""

from __future__ import annotations

import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from anklesea import RESULTS_DIR
from anklesea.motor import default_motor
from anklesea.plots import ACTIVITY_COLORS, ACTIVITY_NAMES, add_citation, energy_contour
from anklesea.profiles import GaitProfile, read_profiles
from anklesea.sea import Limits, joint_work
from anklesea.sizing import (
    BUS_VOLTAGE,
    DESIGN_MASS_KG,
    LEVEL_WALK,
    feasible_compliance,
    optimize,
    quadratic_in_compliance,
    sweep,
)
from anklesea.strides import ACTIVITIES, condition_sort_key

REPORT = RESULTS_DIR / "sizing_modes.md"
TABLE = RESULTS_DIR / "profiles" / "sizing_modes.csv"
CONTOUR = RESULTS_DIR / "figures" / "sizing_modes_contour.png"
BY_CONDITION = RESULTS_DIR / "figures" / "sizing_by_condition.png"
RATIOS = np.geomspace(30.0, 2000.0, 600)
MIN_SUBJECTS = 5  # summary statistics use conditions with at least this many subjects


def condition_value(condition: str) -> float:
    """Numeric part of a condition label ("1.20 m/s" -> 1.2); NaN for speed classes."""
    match = re.match(r"([\d.]+)", condition)
    return float(match.group(1)) if match else np.nan


def rigid_optimum(load, motor: object, limits: Limits) -> tuple[float, float]:
    """Least energy and ratio of a rigid actuator within the drive limits (NaN if none)."""
    best = (np.nan, np.nan)
    for n in RATIOS:
        lo, _ = feasible_compliance(load, n, motor, limits)
        if lo == 0.0:
            energy = quadratic_in_compliance(load, n, motor).c
            if np.isnan(best[0]) or energy < best[0]:
                best = (energy, n)
    return best


def binding(design: dict, limits: Limits) -> str:
    names = []
    if design["peak_voltage"] >= 0.995 * limits.available_voltage:
        names.append("voltage")
    if design["peak_current"] >= 0.995 * limits.peak_current:
        names.append("peak current")
    if design["peak_speed"] >= 0.995 * limits.max_speed:
        names.append("speed")
    return ", ".join(names) if names else "none"


def size_all(profiles: dict[tuple[str, str], GaitProfile], motor, limits: Limits) -> pd.DataFrame:
    rows = []
    for (activity, condition), profile in profiles.items():
        load = profile.load(DESIGN_MASS_KG)
        design = optimize(load, motor, limits, ratios=RATIOS)
        thermal = optimize(load, motor, limits, ratios=RATIOS, thermal=True)
        rigid_energy, rigid_ratio = rigid_optimum(load, motor, limits)
        work = joint_work(load)
        row = {
            "activity": activity,
            "condition": condition,
            "n_subjects": profile.n_subjects,
            "n_strides": profile.n_strides,
            "stride_time_s": profile.stride_time,
            "peak_torque_Nm": float(np.abs(load.torque).max()),
            "net_work_J": work["net"],
            "positive_work_J": work["positive"],
            "feasible": bool(design["drive_feasible"]),
            "stiffness_Nm_per_rad": design["stiffness"],
            "ratio": design["ratio"],
            "energy_J": design.get("energy", np.nan),
            "energy_no_regen_J": design.get("energy_no_regen", np.nan),
            "peak_current_A": design.get("peak_current", np.nan),
            "rms_current_A": design.get("rms_current", np.nan),
            "peak_voltage_V": design.get("peak_voltage", np.nan),
            "binding": binding(design, limits) if design["drive_feasible"] else "",
            "rigid_energy_J": rigid_energy,
            "rigid_ratio": rigid_ratio,
            "thermal_feasible": bool(thermal["feasible"]),
        }
        rows.append(row)
        print(f"{activity:13s} {condition:9s} k={row['stiffness_Nm_per_rad']:7.0f} N={row['ratio']:6.0f} "
              f"E={row['energy_J']:6.1f} J")
    frame = pd.DataFrame(rows)
    order = sorted(range(len(frame)), key=lambda i: condition_sort_key(frame["activity"][i], frame["condition"][i]))
    return frame.iloc[order].reset_index(drop=True)


def representative(table: pd.DataFrame) -> dict[str, str]:
    """Per activity, the condition with the most subjects, then the most strides."""
    out = {}
    for activity, g in table.groupby("activity", sort=False):
        if activity == "treadmill":
            out[activity] = LEVEL_WALK[1]
            continue
        g = g.sort_values(["n_subjects", "n_strides"], ascending=False)
        out[activity] = g["condition"].iloc[0]
    return out


def plot_contours(profiles, table: pd.DataFrame, chosen: dict[str, str], motor, limits: Limits) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(13, 8), layout="constrained", sharex=True, sharey=True)
    picks = table.set_index(["activity", "condition"])
    for ax, activity in zip(axes.flat, ACTIVITIES):
        condition = chosen[activity]
        result = sweep(profiles[(activity, condition)].load(DESIGN_MASS_KG), motor, limits)
        emin = float(np.nanmin(result.energy))
        vmax = 4.0 * emin if emin > 0 else emin + max(4.0 * abs(emin), 40.0)
        cs, _ = energy_contour(ax, result, vmax=vmax, limits=("ok_voltage", "ok_peak_current"))
        for other, cond in chosen.items():
            r = picks.loc[(other, cond)]
            if not r["feasible"]:
                continue
            own = other == activity
            ax.plot(r["stiffness_Nm_per_rad"], r["ratio"], marker="*" if own else "o",
                    markersize=15 if own else 6, color=ACTIVITY_COLORS[other], markeredgecolor="black",
                    linestyle="none", zorder=5)
        r = picks.loc[(activity, condition)]
        label = (f"k = {r['stiffness_Nm_per_rad']:.0f}, N = {r['ratio']:.0f}, {r['energy_J']:.1f} J"
                 if r["feasible"] else "no design meets the drive limits")
        ax.set_title(f"{ACTIVITY_NAMES[activity]} {condition}: {label}", fontsize=9, loc="left")
        fig.colorbar(cs, ax=ax, format="%.0f", shrink=0.85)
        ax.label_outer()
    handles = [plt.Line2D([], [], marker="o", color=ACTIVITY_COLORS[a], markeredgecolor="black", linestyle="none",
                          label=ACTIVITY_NAMES[a]) for a in ACTIVITIES]
    fig.legend(handles=handles, loc="outside upper center", ncol=6, fontsize=9,
               title="optimum of each activity (star: the panel's own); colour bars: energy per stride (J)",
               title_fontsize=9)
    add_citation(fig)
    fig.savefig(CONTOUR, dpi=130)
    plt.close(fig)


def plot_by_condition(table: pd.DataFrame) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.5), layout="constrained", sharey="row")
    groups = [
        (["treadmill"], "treadmill belt speed (m/s)"),
        (["rampascent", "rampdescent"], "ramp incline (deg)"),
        (["stairascent", "stairdescent"], "stair height (in)"),
    ]
    for col, (activities, xlabel) in enumerate(groups):
        for activity in activities:
            g = table[(table["activity"] == activity) & table["feasible"]]
            x = g["condition"].map(condition_value)
            for row, key in enumerate(["stiffness_Nm_per_rad", "ratio"]):
                ax = axes[row, col]
                ax.plot(x, g[key], "-", color=ACTIVITY_COLORS[activity], lw=1.5, label=ACTIVITY_NAMES[activity])
                few = g["n_subjects"] < 2
                ax.plot(x[~few], g[key][~few], "o", color=ACTIVITY_COLORS[activity], markeredgecolor="black")
                ax.plot(x[few], g[key][few], "o", color="white", markeredgecolor=ACTIVITY_COLORS[activity])
            bad = table[(table["activity"] == activity) & ~table["feasible"]]
            for _, r in bad.iterrows():
                axes[0, col].axvline(condition_value(r["condition"]), color=ACTIVITY_COLORS[activity], ls=":", lw=1)
        axes[1, col].set_xlabel(xlabel)
        axes[0, col].legend(fontsize=8)
    axes[0, 0].set_ylabel("optimal stiffness k (N·m/rad)")
    axes[1, 0].set_ylabel("optimal gear ratio N")
    for ax in axes[0]:
        ax.set_yscale("log")
    for ax in axes.flat:
        ax.grid(alpha=0.3, which="both")
    fig.suptitle(f"Energy-optimal design per condition ({DESIGN_MASS_KG:.0f} kg user, {BUS_VOLTAGE:.0f} V, within the "
                 "drive limits). Hollow: one subject only; dotted line: no design meets the limits.",
                 fontsize=10, x=0.01, ha="left")
    add_citation(fig)
    fig.savefig(BY_CONDITION, dpi=140)
    plt.close(fig)


def fmt(value: float, digits: int = 0) -> str:
    return "" if not np.isfinite(value) else f"{value:.{digits}f}"


def markdown(table: pd.DataFrame, chosen: dict[str, str]) -> str:
    header = ["activity", "condition", "subjects (strides)", "peak torque (N·m)", "joint work net / positive (J)",
              "k (N·m/rad)", "N", "energy (J)", "no regen. (J)", "rigid (J)", "peak / RMS current (A)",
              "peak voltage (V)", "binding limit"]
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    for _, r in table.iterrows():
        mark = "**" if chosen.get(r["activity"]) == r["condition"] else ""
        if r["feasible"]:
            design = [fmt(r["stiffness_Nm_per_rad"]), fmt(r["ratio"]), fmt(r["energy_J"], 1),
                      fmt(r["energy_no_regen_J"], 1), fmt(r["rigid_energy_J"], 1) or "infeasible",
                      f"{r['peak_current_A']:.1f} / {r['rms_current_A']:.2f}", fmt(r["peak_voltage_V"], 1), r["binding"]]
        else:
            design = ["none", "", "", "", "", "", "", ""]
        lines.append("| " + " | ".join([
            f"{mark}{ACTIVITY_NAMES[r['activity']]}{mark}", f"{mark}{r['condition']}{mark}",
            f"{r['n_subjects']} ({r['n_strides']})", fmt(r["peak_torque_Nm"]),
            f"{r['net_work_J']:.1f} / {r['positive_work_J']:.1f}", *design,
        ]) + " |")
    return "\n".join(lines)


def main() -> int:
    profiles = read_profiles()
    motor = default_motor()
    limits = Limits.from_motor(motor, BUS_VOLTAGE)
    table = size_all(profiles, motor, limits)
    chosen = representative(table)
    TABLE.parent.mkdir(parents=True, exist_ok=True)
    with TABLE.open("w") as handle:
        handle.write("# SEA design per activity and condition, derived from Camargo et al. (2021), "
                     "J. Biomech. 119:110320, CC BY 4.0. Aggregate across subjects.\n")
        table.to_csv(handle, index=False, float_format="%.5g")
    plot_contours(profiles, table, chosen, motor, limits)
    plot_by_condition(table)

    feas = table[table["feasible"]]
    rep = table.set_index(["activity", "condition"]).loc[list(chosen.items())]
    rep_feas = rep[rep["feasible"]]
    k_lo, k_hi = rep_feas["stiffness_Nm_per_rad"].min(), rep_feas["stiffness_Nm_per_rad"].max()
    n_lo, n_hi = rep_feas["ratio"].min(), rep_feas["ratio"].max()
    multi = feas[feas["n_subjects"] >= MIN_SUBJECTS]
    k_q = multi["stiffness_Nm_per_rad"].quantile([0.1, 0.9]).to_numpy()
    tread = multi[multi["activity"] == "treadmill"].copy()
    tread["speed"] = tread["condition"].map(condition_value)
    slow, fast = tread.iloc[0], tread.iloc[-1]
    voltage_bound = int(feas["binding"].str.contains("voltage").sum())
    descent = feas[feas["activity"].isin(["rampdescent", "stairdescent"])]
    infeasible = table[~table["feasible"]]
    infeasible_text = (
        "Every condition has a design within the drive limits."
        if infeasible.empty
        else "No design meets the drive limits for "
        + ", ".join(f"{ACTIVITY_NAMES[a]} {c} ({n} subject{'s' if n > 1 else ''}, {m} strides)"
                    for a, c, n, m in zip(infeasible["activity"], infeasible["condition"], infeasible["n_subjects"],
                                          infeasible["n_strides"]))
        + "."
    )
    thermal_ok = table[table["thermal_feasible"]]
    ok_speeds = [float(c.split()[0]) for c in thermal_ok.loc[thermal_ok["activity"] == "treadmill", "condition"]]
    walk_note = (
        "No treadmill speed is among them."
        if not ok_speeds
        else f"The fastest treadmill speed among them is {max(ok_speeds):.2f} m/s."
    )
    few = table[table["n_subjects"] < MIN_SUBJECTS]
    single_text = (
        f"the conditions with fewer than {MIN_SUBJECTS} subjects ("
        + ", ".join(f"{ACTIVITY_NAMES[a]} {c}" for a, c in zip(few["activity"], few["condition"]))
        + ") should not be read as a trend"
        if not few.empty
        else f"every condition here has at least {MIN_SUBJECTS} subjects"
    )
    thermal_text = (
        "No condition has a design that also meets the motor's continuous current rating."
        if thermal_ok.empty
        else f"{len(thermal_ok)} of {len(table)} conditions have a design that also meets the motor's continuous "
        "current rating: " + ", ".join(f"{ACTIVITY_NAMES[a]} {c}" for a, c in zip(thermal_ok["activity"],
                                                                                    thermal_ok["condition"]))
        + ". "
        + walk_note
    )
    rigid_ok = feas["rigid_energy_J"].notna()
    rigid_parts = []
    for activity in ACTIVITIES:
        rows = feas[feas["activity"] == activity]
        ok = rows[rows["rigid_energy_J"].notna()]
        if ok.empty:
            continue
        name = ACTIVITY_NAMES[activity]
        if len(ok) == len(rows):
            rigid_parts.append(f"every {name} condition")
        elif activity == "treadmill" and list(ok.index) == list(rows.index[: len(ok)]):
            rigid_parts.append(f"{name} up to {ok['condition'].iloc[-1]}")
        else:
            rigid_parts.append(f"{name} at " + ", ".join(ok["condition"]))
    rigid_fail = [ACTIVITY_NAMES[a] for a in ACTIVITIES
                  if (feas["activity"] == a).any() and feas.loc[feas["activity"] == a, "rigid_energy_J"].isna().any()]
    stair_d = descent[descent["activity"] == "stairdescent"]["energy_J"]
    ramp_d = descent[descent["activity"] == "rampdescent"]["energy_J"]
    text = f"""# Sizing across activities

The spring stiffness `k` and gear ratio `N` that minimize the motor's electrical energy per stride, sized separately for each activity and condition in the dataset. Same actuator, user mass ({DESIGN_MASS_KG:.0f} kg) and bus voltage ({BUS_VOLTAGE:.0f} V) as `results/sizing_levelwalk.md`. Each optimum meets the drive limits (voltage, 30 A peak current, motor speed); it is found by scanning 600 ratios from 30 to 2000 with the closed-form optimum in compliance clipped to the feasible interval at each ratio (`anklesea.sizing.optimize`, checked against the grid search in `results/convex_check.md`).

![Energy contours per activity](figures/sizing_modes_contour.png)

*Energy per stride over (k, N) for one condition per activity (the one with the most subjects; treadmill at 1.2 m/s). Veiled designs break a drive limit. The star is the panel's own optimum; the dots are the other activities' optima, to show how far apart they lie. Data: Camargo et al. (2021), CC BY 4.0.*

![Optimal design per condition](figures/sizing_by_condition.png)

*Energy-optimal stiffness and ratio against walking speed, ramp incline and stair height. Data: Camargo et al. (2021), CC BY 4.0.*

## Design table

Bold rows are the conditions in the contour figure. "Rigid" is the best rigid actuator within the drive limits. Joint work is per stride at the design user mass.

{markdown(table, chosen)}

## Observations

- **The stiffness changes little; the ratio changes a lot.** Over the conditions with at least {MIN_SUBJECTS} subjects, 80 % of the optimal stiffnesses lie between {k_q[0]:.0f} and {k_q[1]:.0f} N·m/rad (representative conditions: {k_lo:.0f} to {k_hi:.0f}), while the ratio spans {n_lo:.0f} to {n_hi:.0f} over the representative conditions and falls from {slow['ratio']:.0f} at {slow['condition']} to {fast['ratio']:.0f} at {fast['condition']} on the treadmill. The voltage limit binds in {voltage_bound} of {len(feas)} conditions: faster joint motion means more back EMF per unit ratio, so the ratio has to drop, and the spring stays near the value that cancels motor motion at push-off.
- **Descent returns energy.** On ramp and stair descent the ankle does net negative work, so with ideal regeneration the motor can charge the battery: stair descent returns {-stair_d.max():.1f} to {-stair_d.min():.1f} J per stride, while ramp descent roughly breaks even ({ramp_d.min():.1f} to {ramp_d.max():.1f} J) because its smaller negative joint work barely covers the motor losses. Without regeneration the same designs cost {descent['energy_no_regen_J'].min():.1f} to {descent['energy_no_regen_J'].max():.1f} J. The optimum with regeneration maximizes what is recovered, which is only meaningful if the driver and battery can take it back; section 10 uses both measures.
- {infeasible_text}
- {thermal_text}
- A rigid actuator meets the drive limits in {int(rigid_ok.sum())} of {len(feas)} conditions that have a feasible SEA design: {"; ".join(rigid_parts)}. It fails in the other {", ".join(rigid_fail[:-1]) + " and " + rigid_fail[-1] if len(rigid_fail) > 1 else "".join(rigid_fail)} conditions, the faster and steeper ones, where the joint moves fast at high torque and the back EMF caps the ratio.
- Profiles that rest on one subject or a handful of strides (see the subjects column) are less reliable; {single_text}. The treadmill profiles use every stride of each subject who walked at that speed.

Section 10 (`results/cross_mode.md`) asks what a design tuned for level walking costs in the other activities, and what a compromise design looks like.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
