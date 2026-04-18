#!/usr/bin/env python3
"""Check the grid search against the convex (quadratic in compliance) form of the energy.

Writes:
  results/convex_check.md
  results/figures/convex_check.png

Derivation: docs/derivation.md. Method: Bolivar-Nieto, Thomas, Rouse, Gregg, "Convex
optimization for spring design in series elastic actuators: from theory to practice",
IROS 2021; Notre Dame AME 60553 lecture notes L12 and L13.
Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from anklesea import RESULTS_DIR
from anklesea.motor import default_motor
from anklesea.plots import add_citation
from anklesea.profiles import read_profiles
from anklesea.sea import Limits, evaluate
from anklesea.sizing import (
    BUS_VOLTAGE,
    DESIGN_MASS_KG,
    LEVEL_WALK,
    constrained_optimum,
    quadratic_in_compliance,
    sweep,
)

FIGURE = RESULTS_DIR / "figures" / "convex_check.png"
REPORT = RESULTS_DIR / "convex_check.md"


def main() -> int:
    motor = default_motor()
    limits = Limits.from_motor(motor, BUS_VOLTAGE)
    loose = Limits.from_motor(motor, 1e6)
    load = read_profiles()[LEVEL_WALK].load(DESIGN_MASS_KG)
    result = sweep(load, motor, limits)
    opt = result.optimum(constraint="drive_feasible")
    ratio = opt["ratio"]

    # 1. The quadratic reproduces the full model at every grid point.
    quads = [quadratic_in_compliance(load, n, motor) for n in result.ratios]
    alpha = np.where(np.isinf(result.stiffness), 0.0, 1.0 / result.stiffness)
    quad_grid = np.stack([q.energy(alpha) for q in quads])
    max_rel = float(np.max(np.abs(quad_grid - result.energy) / np.abs(result.energy)))
    all_convex = all(q.a > 0 for q in quads)

    # 2. Closed-form optimum against a dense search with the full model at the chosen ratio.
    quad = quadratic_in_compliance(load, ratio, motor)
    dense = np.linspace(0.0, 3.0 * quad.savings_region[1], 30001)
    full = evaluate(load, dense, np.full_like(dense, ratio), motor, loose)
    alpha_search = float(dense[np.argmin(full["energy"])])
    k_closed, k_search = quad.optimal_stiffness, 1.0 / alpha_search
    no_regen_best = 1.0 / float(dense[1:][np.argmin(full["energy_no_regen"][1:])])
    directional = evaluate(load, dense, np.full_like(dense, ratio), motor, loose, efficiency_model="directional")
    directional_best = 1.0 / float(dense[1:][np.argmin(directional["energy"][1:])])

    # 3. Closed-form optimum stiffness per ratio against the grid's best stiffness per ratio.
    k_star = np.array([q.optimal_stiffness for q in quads])
    e_star = np.array([q.optimal_energy for q in quads])
    best = result.best_per_ratio()
    e_grid = result.energy[np.arange(len(result.ratios)), best]
    k_lo, k_hi = result.stiffness[0], result.stiffness[np.isfinite(result.stiffness)][-1]
    inside = (k_star >= k_lo) & (k_star <= k_hi)
    gap = float(np.max(((e_grid - e_star) / e_star)[inside]))
    n_outside = int((~inside).sum())
    lowest_k_star = float(k_star.min())

    # 4. Constrained optimum from the closed form on a fine ratio grid (no stiffness grid at all).
    fine = np.geomspace(result.ratios[0], result.ratios[-1], 2000)
    cons = np.array([constrained_optimum(load, n, motor, limits) for n in fine])
    i_best = int(np.nanargmin(cons[:, 1]))
    cf_ratio, cf_alpha, cf_energy = fine[i_best], cons[i_best, 0], cons[i_best, 1]
    thermal_any = any(not np.isnan(constrained_optimum(load, n, motor, limits, thermal=True)[0]) for n in fine)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.4), layout="constrained")
    k_plot = 1.0 / dense[1:]
    ax1.plot(k_plot, full["energy"][1:], color="#1d4ed8", lw=2.5, label="full model, with regeneration")
    sample = np.geomspace(90, 20000, 25)
    ax1.plot(sample, quad.energy(1.0 / sample), "o", color="white", markeredgecolor="black", ms=5,
             label="quadratic $a\\alpha^2 + b\\alpha + c$")
    ax1.plot(k_plot, full["energy_no_regen"][1:], color="#b45309", lw=1.6, ls="--",
             label="full model, no regeneration (not quadratic)")
    ax1.plot(k_plot, directional["energy"][1:], color="#166534", lw=1.6, ls=":",
             label="direction-dependent efficiency (not quadratic)")
    ax1.axhline(quad.c, color="0.4", lw=0.8)
    ax1.text(15000, quad.c * 0.97, "rigid actuator", ha="right", va="top", fontsize=8, color="0.3")
    ax1.axvspan(1.0 / quad.savings_region[1], 20000, color="#1d4ed8", alpha=0.07)
    ax1.text(1.0 / quad.savings_region[1] * 1.08, quad.optimal_energy * 0.92,
             "energy savings region", fontsize=8, color="#1d4ed8")
    ax1.plot(k_closed, quad.optimal_energy, "*", ms=14, color="white", markeredgecolor="black")
    ax1.set_xscale("log")
    ax1.set_xlim(80, 20000)
    ax1.set_ylim(quad.optimal_energy * 0.85, max(full["energy_no_regen"][0], quad.c) * 1.25)
    ax1.set_xlabel("series spring stiffness k (N·m/rad)")
    ax1.set_ylabel("motor electrical energy per stride (J)")
    ax1.set_title(f"At N = {ratio:.0f}, energy is a parabola in compliance 1/k:\n"
                  f"closed-form optimum k* = {k_closed:.0f} N·m/rad", fontsize=10, loc="left")
    ax1.legend(fontsize=7.5, loc="upper right")

    finite = np.isfinite(k_star)
    ax2.plot(result.ratios[finite], k_star[finite], color="#1d4ed8", lw=2.5, label="closed form $k^* = -2a/b$")
    ax2.plot(result.ratios, result.stiffness[best], "o", ms=3, color="none", markeredgecolor="black", mew=0.7,
             label="grid search, least energy per ratio (grid starts at 50)")
    feas = result.metrics["drive_feasible"][np.arange(len(result.ratios)), best]
    ax2.fill_between(result.ratios, 1, 1e5, where=feas, color="#166534", alpha=0.08, step="mid",
                     label="that design meets the drive limits")
    ax2.set_xscale("log")
    ax2.set_yscale("log")
    ax2.set_ylim(1, 1000)
    ax2.set_xlabel("gear ratio N")
    ax2.set_ylabel("optimal stiffness (N·m/rad)")
    ax2.set_title("The optimal stiffness per ratio from the closed form\nmatches the grid search", fontsize=10,
                  loc="left")
    ax2.legend(fontsize=7.5, loc="upper left")
    add_citation(fig)
    fig.savefig(FIGURE, dpi=150)
    plt.close(fig)

    text = f"""# Convex check of the sizing

For a fixed gear ratio, the energy per stride with ideal regeneration and a constant gear efficiency is a convex quadratic in the spring compliance `alpha = 1/k` (derivation in [`docs/derivation.md`](../docs/derivation.md); method of Bolivar-Nieto et al., IROS 2021). This page checks the grid search of `results/sizing_levelwalk.md` against that closed form, for the same load (level walking at 1.2 m/s, {DESIGN_MASS_KG:.0f} kg user).

![Convex check](figures/convex_check.png)

*Left: energy against stiffness at the chosen ratio. The full model (line) and the quadratic (circles) coincide; the star is the closed-form optimum; the shaded band is the energy savings region, every spring that beats the rigid actuator. Right: closed-form optimal stiffness per ratio against the grid search. Data: Camargo et al. (2021), CC BY 4.0.*

| check | result |
|---|---|
| largest relative difference between the quadratic and the full model over all {result.energy.size} grid designs | {max_rel:.1e} |
| quadratic coefficient `a` positive (convex) at every ratio | {"yes" if all_convex else "no"} |
| closed-form optimum at N = {ratio:.0f} | k* = {k_closed:.1f} N·m/rad, {quad.optimal_energy:.2f} J |
| dense search with the full model (30001 compliances) at N = {ratio:.0f} | k = {k_search:.1f} N·m/rad, {full['energy'].min():.2f} J |
| grid optimum (121 stiffness values) | k = {opt['stiffness']:.0f} N·m/rad, {opt['energy']:.2f} J |
| largest excess of the grid's best energy per ratio over the closed-form optimum, for ratios whose k* lies inside the grid | {gap:.2%} |
| ratios whose k* lies outside the grid's stiffness range | {n_outside} (the lowest ratios; k* down to {lowest_k_star:.0f} N·m/rad) |
| optimum within the drive limits from the closed form clipped to the feasible compliance interval, 2000 ratios | k = {1.0 / cf_alpha:.1f} N·m/rad, N = {cf_ratio:.0f}, {cf_energy:.2f} J |
| any ratio with a compliance that also meets the RMS current rating | {"yes" if thermal_any else "no"} |
| energy savings region at N = {ratio:.0f} | k > {1.0 / quad.savings_region[1]:.0f} N·m/rad |
| rigid actuator at N = {ratio:.0f} (`c`) | {quad.c:.2f} J |

The quadratic matches the full model to rounding error. The inductive term `L i di/dt` drops out of the closed form because it integrates to zero over a periodic stride, and the periodic central difference used for `di/dt` in the full model keeps that exact. The grid search is therefore only limited by its resolution: its optimum lies within one grid step of the closed form.

The limits fit the same structure. At a fixed ratio the current, motor speed and voltage are affine in `alpha` at every instant, so each instantaneous limit allows an interval of compliances and all of them together allow one interval; the RMS current limit is a convex quadratic inequality and allows an interval too. The constrained optimum at that ratio is the unconstrained `alpha*` clipped into the interval (`anklesea.sizing.constrained_optimum`). A one-dimensional scan over the ratio then finds the constrained optimum without any stiffness grid. It lands {(opt['energy'] - cf_energy) / opt['energy']:.1%} below the grid optimum, at a ratio between two grid rows: the optimum sits on the voltage limit, and the grid's ratio step (3.6 %) cannot follow that boundary exactly. It also confirms that no ratio has a compliance meeting the motor's continuous current rating.

Two variants are *not* quadratic and are shown dashed in the left panel. Without regeneration, the energy integrates `max(P, 0)`, which is not a quadratic in `alpha` and has no closed form; its optimum at this ratio is k = {no_regen_best:.0f} N·m/rad, stiffer than with regeneration because a softer spring lets the motor return energy that would now be lost. With a direction-dependent gear efficiency, the optimum is k = {directional_best:.0f} N·m/rad. Both are reported by the grid search, which does not rely on the quadratic form.

Practical value of the closed form: the optimal stiffness for any ratio costs three integrals instead of a sweep over `k`, and the savings region tells the designer how far the spring may deviate from the optimum (for tolerance or for a second activity) and still beat a rigid actuator.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
