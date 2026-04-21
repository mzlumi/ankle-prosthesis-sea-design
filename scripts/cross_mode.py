#!/usr/bin/env python3
"""Cost of a walking-tuned design in other activities, and a compromise design for a daily mix.

Writes:
  results/cross_mode.md
  results/figures/cross_mode.png

The daily mix of activities is from Srisuwan and Klute (2021), see docs/assumptions.md.
Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from anklesea import RESULTS_DIR
from anklesea.motor import default_motor
from anklesea.plots import ACTIVITY_COLORS, ACTIVITY_NAMES, add_citation
from anklesea.profiles import read_profiles
from anklesea.sea import Design, Limits, evaluate_design
from anklesea.sizing import (
    BUS_VOLTAGE,
    DAILY_MIX,
    DESIGN_MASS_KG,
    LEVEL_WALK,
    MIX_ACTIVITIES,
    STEPS_PER_DAY,
    mix_conditions,
    optimize,
    optimize_mix,
    sweep,
)

REPORT = RESULTS_DIR / "cross_mode.md"
FIGURE = RESULTS_DIR / "figures" / "cross_mode.png"
ACTIVITIES = MIX_ACTIVITIES
NAMES = {"level": "level walking", **{a: ACTIVITY_NAMES[a] for a in ACTIVITIES[1:]}}
COLORS = {"level": ACTIVITY_COLORS["treadmill"], **{a: ACTIVITY_COLORS[a] for a in ACTIVITIES[1:]}}
STRIDES_PER_DAY = STEPS_PER_DAY / 2  # one prosthesis stride per two steps
J_PER_WH = 3600.0


def run_case(profiles, conditions, weights, motor, limits: Limits, objective: str = "energy") -> dict:
    """Compromise design for one set of conditions and weights, and its per-activity energy."""
    loads = [profiles[conditions[a]].load(DESIGN_MASS_KG) for a in ACTIVITIES]
    w = [weights[a] for a in ACTIVITIES]
    if objective == "energy":
        design = optimize_mix(loads, w, motor, limits)
    else:
        results = [sweep(load, motor, limits) for load in loads]
        ok = np.logical_and.reduce([r.metrics["drive_feasible"] for r in results])
        total = sum(wi * r.metrics[objective] for wi, r in zip(np.asarray(w) / sum(w), results))
        total = np.where(ok, total, np.inf)
        i, j = np.unravel_index(np.argmin(total), total.shape)
        design = {"stiffness": float(results[0].stiffness[j]), "ratio": float(results[0].ratios[i]),
                  "weighted_energy": float(total[i, j])}
    per = {a: evaluate_design(load, Design(design["stiffness"], design["ratio"]), motor, limits)
           for a, load in zip(ACTIVITIES, loads)}
    return {"design": design, "per": per, "loads": dict(zip(ACTIVITIES, loads))}


def daily(per: dict, weights: dict, key: str = "energy") -> float:
    """Energy per day in Wh: strides per day times the weighted energy per stride."""
    total_w = sum(weights.values())
    return STRIDES_PER_DAY * sum(weights[a] / total_w * per[a][key] for a in ACTIVITIES) / J_PER_WH


def fmt_design(d: dict) -> str:
    return "none" if np.isnan(d["ratio"]) else f"k = {d['stiffness']:.0f} N·m/rad, N = {d['ratio']:.0f}"


def main() -> int:
    profiles = read_profiles()
    motor = default_motor()
    limits = Limits.from_motor(motor, BUS_VOLTAGE)
    conditions = mix_conditions(profiles)
    loads = {a: profiles[conditions[a]].load(DESIGN_MASS_KG) for a in ACTIVITIES}

    own = {a: optimize(loads[a], motor, limits) for a in ACTIVITIES}
    walk = own["level"]
    walk_per = {a: evaluate_design(loads[a], Design(walk["stiffness"], walk["ratio"]), motor, limits)
                for a in ACTIVITIES}
    base = run_case(profiles, conditions, DAILY_MIX, motor, limits)
    comp, comp_per = base["design"], base["per"]

    # Per-activity table.
    rows = []
    for a in ACTIVITIES:
        o, w, c = own[a], walk_per[a], comp_per[a]
        cond = conditions[a][1]

        def cost(x: dict) -> str:
            text = f"{x['energy']:.1f} ({x['energy'] - o['energy']:+.1f})"
            if not x["drive_feasible"]:
                text += f", **breaks limits** ({x['peak_voltage']:.0f} V, {x['peak_current']:.0f} A)"
            return text

        rows.append([f"{NAMES[a]} ({cond})", f"{DAILY_MIX[a]:.1%}", fmt_design(o), f"{o['energy']:.1f}",
                     cost(w), cost(c), f"{o['energy_no_regen']:.1f} / {w['energy_no_regen']:.1f} / "
                     f"{c['energy_no_regen']:.1f}"])
    per_table = "\n".join(["| activity (condition) | share of steps | own optimum | own energy (J) | walking-tuned "
                           "design (J) | compromise design (J) | no regeneration: own / walking / compromise (J) |",
                           "|---|---|---|---|---|---|---|"] + ["| " + " | ".join(r) + " |" for r in rows])

    own_daily = daily(own, DAILY_MIX)
    walk_daily = daily(walk_per, DAILY_MIX)
    comp_daily = daily(comp_per, DAILY_MIX)
    own_daily_nr = daily(own, DAILY_MIX, "energy_no_regen")
    walk_daily_nr = daily(walk_per, DAILY_MIX, "energy_no_regen")
    comp_daily_nr = daily(comp_per, DAILY_MIX, "energy_no_regen")
    walk_ok = all(walk_per[a]["drive_feasible"] for a in ACTIVITIES)
    broken = [NAMES[a] for a in ACTIVITIES if not walk_per[a]["drive_feasible"]]

    # Sensitivity of the compromise to the assumptions behind it.
    heavy = {a: (DAILY_MIX[a] if a == "level" else 3 * DAILY_MIX[a]) for a in ACTIVITIES}
    cases = [
        ("baseline: Srisuwan and Klute (2021) mix, energy with regeneration", conditions, DAILY_MIX, "energy", limits),
        ("ramps and stairs three times as frequent", conditions, heavy, "energy", limits),
        ("equal weight on all five activities", conditions, {a: 1.0 for a in ACTIVITIES}, "energy", limits),
        ("level walking only (but within limits everywhere)", conditions, {a: float(a == "level") for a in ACTIVITIES},
         "energy", limits),
        ("objective: energy without regeneration (grid search)", conditions, DAILY_MIX, "energy_no_regen", limits),
        ("level walking at 1.0 m/s", {**conditions, "level": ("treadmill", "1.00 m/s")}, DAILY_MIX, "energy", limits),
        ("level walking at 1.4 m/s", {**conditions, "level": ("treadmill", "1.40 m/s")}, DAILY_MIX, "energy", limits),
        ("lowest ramp and stair conditions", mix_conditions(profiles, stair="lowest", ramp="lowest"), DAILY_MIX,
         "energy", limits),
        ("highest ramp and stair conditions", mix_conditions(profiles, stair="highest", ramp="highest"), DAILY_MIX,
         "energy", limits),
        ("bus voltage 48 V (limits checked at 48 V)", conditions, DAILY_MIX, "energy", Limits.from_motor(motor, 48.0)),
    ]
    sens_rows = []
    designs_seen = []
    for name, conds, weights, objective, case_limits in cases:
        if any(c not in profiles for c in conds.values()):
            continue
        res = run_case(profiles, conds, weights, motor, case_limits, objective)
        d = res["design"]
        if np.isnan(d["ratio"]):
            sens_rows.append([name, "none", "", ""])
            continue
        # Evaluate every case's design on the baseline conditions and mix, to compare like with like.
        designs_seen.append((round(d["stiffness"]), round(d["ratio"])))
        on_base = {a: evaluate_design(loads[a], Design(d["stiffness"], d["ratio"]), motor, case_limits)
                   for a in ACTIVITIES}
        ok = all(on_base[a]["drive_feasible"] for a in ACTIVITIES)
        sens_rows.append([name, fmt_design(d), f"{daily(on_base, DAILY_MIX):.2f}" + ("" if ok else " (breaks limits)"),
                          f"{daily(on_base, DAILY_MIX, 'energy_no_regen'):.2f}"])
    sens_table = "\n".join(["| case | compromise design | daily energy on the baseline mix (Wh) | same, no "
                            "regeneration (Wh) |", "|---|---|---|---|"] +
                           ["| " + " | ".join(r) + " |" for r in sens_rows])
    same_as_base = sum(d == designs_seen[0] for d in designs_seen) - 1
    binding = [NAMES[a] for a in ACTIVITIES
               if comp_per[a]["peak_voltage"] >= 0.995 * limits.available_voltage
               or comp_per[a]["peak_current"] >= 0.995 * limits.peak_current]
    walk_extra = comp_per["level"]["energy"] - own["level"]["energy"]

    # Figure.
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6), layout="constrained", width_ratios=[2.2, 1])
    x = np.arange(len(ACTIVITIES))
    width = 0.27
    designs = [("own optimum", own, "0.75", None), ("walking-tuned", walk_per, "#1d4ed8", None),
               ("compromise", comp_per, "#b45309", None)]
    for k, (label, per, color, _) in enumerate(designs):
        values = [per[a]["energy"] for a in ACTIVITIES]
        bars = ax1.bar(x + (k - 1) * width, values, width, color=color, label=label, edgecolor="black", lw=0.5)
        nr = [per[a]["energy_no_regen"] for a in ACTIVITIES]
        ax1.plot(x + (k - 1) * width, nr, linestyle="none", marker="_", markersize=14, color="black",
                 markeredgewidth=2)
        for a, bar in zip(ACTIVITIES, bars):
            if not per[a]["drive_feasible"]:
                bar.set_hatch("xxx")
    ax1.plot([], [], linestyle="none", marker="_", markersize=14, color="black", markeredgewidth=2,
             label="without regeneration")
    ax1.axhline(0, color="black", lw=0.8)
    ax1.set_xticks(x, [f"{NAMES[a]}\n{conditions[a][1]}" for a in ACTIVITIES])
    ax1.set_ylabel("motor electrical energy per stride (J)")
    ax1.legend(fontsize=8, ncol=2)
    ax1.set_title("The walking-tuned design breaks a limit in some activities (cross-hatched);\nthe compromise "
                  "meets all of them and pays for it in level walking", fontsize=10, loc="left")
    labels = ["own optimum\n(one actuator\nper activity)", "walking-\ntuned", "compromise"]
    regen = [own_daily, walk_daily, comp_daily]
    noregen = [own_daily_nr, walk_daily_nr, comp_daily_nr]
    ax2.bar(np.arange(3) - 0.18, regen, 0.36, color=["0.75", "#1d4ed8", "#b45309"], edgecolor="black", lw=0.5,
            label="with regeneration")
    ax2.bar(np.arange(3) + 0.18, noregen, 0.36, color="white", edgecolor=["0.4", "#1d4ed8", "#b45309"], lw=1.5,
            label="without regeneration")
    for xi, (r, n) in enumerate(zip(regen, noregen)):
        ax2.text(xi - 0.18, r, f"{r:.1f}", ha="center", va="bottom", fontsize=8)
        ax2.text(xi + 0.18, n, f"{n:.1f}", ha="center", va="bottom", fontsize=8)
    ax2.set_xticks(range(3), labels, fontsize=8)
    ax2.set_ylabel("motor energy per day (Wh)")
    ax2.set_title(f"Daily energy, {STEPS_PER_DAY} steps\nin the free-living mix", fontsize=10, loc="left")
    ax2.legend(fontsize=8)
    add_citation(fig, "Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0. Activity mix: Srisuwan and "
                      "Klute (2021), Prosthet. Orthot. Int. 45(3):191-197.")
    fig.savefig(FIGURE, dpi=150)
    plt.close(fig)

    mix_rows = "\n".join(f"| {NAMES[a]} | {conditions[a][1]} | {profiles[conditions[a]].n_subjects} "
                         f"({profiles[conditions[a]].n_strides}) | {DAILY_MIX[a]:.1%} |" for a in ACTIVITIES)
    walk_text = (
        "The walking-tuned design meets the drive limits in every activity."
        if walk_ok
        else f"The walking-tuned design **breaks the drive limits** in {' and '.join(broken)}: it could not produce "
        "the torque those activities need at the speed they need it, whatever the energy."
    )
    text = f"""# One actuator for several activities

A prosthesis has one spring and one transmission. This page asks two questions. What does a design tuned for level walking cost in the other activities? And which single design minimizes the energy over a typical day?

**Daily mix.** Shares of steps from a free-living study of ten people with a transtibial prosthesis, monitored with an instrumented pylon over 1 to 2 weekdays (Srisuwan and Klute, *Prosthet. Orthot. Int.* 45(3):191-197, 2021, Table 3). Turning steps (9.0 %) are counted as level walking, which they resemble most of the conditions here. The same study gives {STEPS_PER_DAY} steps per day, so {STRIDES_PER_DAY:.0f} strides of the prosthetic leg. Recorded in `docs/assumptions.md`.

**Conditions.** One dataset condition stands for each activity: level walking is the treadmill at 1.2 m/s, as in `results/sizing_levelwalk.md`; ramps and stairs use the incline and step height closest to the course in the mix source (5 degree ramps, 18 cm steps), among conditions with enough subjects.

| activity | condition | subjects (strides) | share of steps |
|---|---|---|---|
{mix_rows}

**Designs compared.** *Own optimum*: the best design for that activity alone (a lower bound no single actuator reaches). *Walking-tuned*: the level-walking optimum, {fmt_design(walk)}. *Compromise*: the design with the least step-weighted energy that meets the drive limits in all five activities, {fmt_design(comp)}. The weighted energy is again a convex quadratic in compliance at each ratio, so the compromise comes from the same clipped closed form (`anklesea.sizing.optimize_mix`). All at {DESIGN_MASS_KG:.0f} kg and {BUS_VOLTAGE:.0f} V.

![Cross-mode comparison](figures/cross_mode.png)

*Left: energy per stride in each activity for the three designs (bars with ideal regeneration, black ticks without; cross-hatched bars break a drive limit). Right: energy per day over the free-living mix. Data: Camargo et al. (2021), CC BY 4.0; activity mix: Srisuwan and Klute (2021).*

## Energy per stride

Energy with ideal regeneration; in brackets, the extra over the activity's own optimum.

{per_table}

## Energy per day

| design | with regeneration (Wh/day) | without regeneration (Wh/day) |
|---|---|---|
| own optimum in each activity (lower bound) | {own_daily:.2f} | {own_daily_nr:.2f} |
| walking-tuned | {walk_daily:.2f} | {walk_daily_nr:.2f} |
| compromise | {comp_daily:.2f} | {comp_daily_nr:.2f} |

- {walk_text}
- The compromise is decided by the limits rather than by the weights. At {fmt_design(comp)} the limit binds in {", ".join(binding) if binding else "no activity"}; to stay feasible there, the ratio drops from {walk['ratio']:.0f} to {comp['ratio']:.0f}, and level walking, {DAILY_MIX['level']:.0%} of the steps, pays {walk_extra:+.1f} J per stride ({walk_extra / own['level']['energy']:+.0%}).
- Over a day the compromise uses {comp_daily:.1f} Wh at the motor terminals (with regeneration; {comp_daily_nr:.1f} Wh without), against {own_daily:.1f} Wh for the unreachable lower bound. The walking-tuned design would use {walk_daily:.1f} Wh but cannot do every activity.
- Feasibility and heat constrain this actuator more than energy does: the voltage limit picks the ratio, and the RMS current, not the energy, is what this motor cannot sustain in walking (`results/sizing_levelwalk.md`).

## Sensitivity to the weights and conditions

Each row recomputes the compromise with one assumption changed, then evaluates that design on the baseline conditions and mix, so the daily energies compare like with like.

{sens_table}

{same_as_base} of the {len(designs_seen) - 1} changed cases land on exactly the baseline compromise: as long as every activity must stay within the limits, the weights hardly matter, because the feasible region is narrow and the binding activity fixes the ratio. What does move the design is a change in *which* conditions must be feasible (a faster walking speed or steeper ramps and stairs), the energy measure (without regeneration the optimum prefers a stiffer spring), and the bus voltage: at 48 V the voltage limit relaxes and the compromise can return toward the walking-tuned design. The practical lesson is that the hardest activity sizes the transmission; the daily mix at most fine-tunes the spring.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
