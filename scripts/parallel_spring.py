#!/usr/bin/env python3
"""Add a parallel spring to the level-walking design and check what it does to heat and energy.

Writes:
  results/parallel_spring.md
  results/figures/parallel_spring.png

Method: anklesea.parallel (Notre Dame AME 60553 lecture notes L12 and L13).
Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from anklesea import RESULTS_DIR
from anklesea.motor import default_motor
from anklesea.parallel import ParallelSpring, actuator_load, joint_level_spring, optimize_parallel
from anklesea.plots import add_citation
from anklesea.profiles import read_profiles
from anklesea.sea import Limits, evaluate, motor_trajectory
from anklesea.sizing import BUS_VOLTAGE, DESIGN_MASS_KG, LEVEL_WALK, mix_conditions, optimize

REPORT = RESULTS_DIR / "parallel_spring.md"
FIGURE = RESULTS_DIR / "figures" / "parallel_spring.png"
OTHER = ["rampascent", "rampdescent", "stairascent", "stairdescent"]
NAMES = {"rampascent": "ramp ascent", "rampdescent": "ramp descent", "stairascent": "stair ascent",
         "stairdescent": "stair descent"}


def check(load, design: dict, motor, limits) -> dict:
    spring = design.get("spring") or ParallelSpring()
    out = evaluate(actuator_load(load, spring), design["compliance"], design["ratio"], motor, limits)
    return {key: value.item() for key, value in out.items()}


def main() -> int:
    profiles = read_profiles()
    motor = default_motor()
    limits = Limits.from_motor(motor, BUS_VOLTAGE)
    load = profiles[LEVEL_WALK].load(DESIGN_MASS_KG)

    sea = optimize(load, motor, limits)
    sea_thermal = optimize(load, motor, limits, thermal=True)
    rms = optimize_parallel(load, motor, limits, objective="rms")
    energy = optimize_parallel(load, motor, limits, objective="energy")
    rms_free = optimize_parallel(load, motor, limits, objective="rms", thermal=False)
    joint = joint_level_spring(load)

    # Brute-force check of the alternation: every (k_p, rest angle) on a grid, each with its
    # own least-energy series spring and ratio within all limits.
    kp_grid = np.linspace(0.0, 700.0, 29)
    rest_grid = np.radians(np.linspace(-10.0, 15.0, 26))
    coarse = np.geomspace(100.0, 1500.0, 150)
    brute = np.full((len(rest_grid), len(kp_grid)), np.nan)
    brute_rms = np.full_like(brute, np.nan)
    for i, rest in enumerate(rest_grid):
        for j, kp in enumerate(kp_grid):
            act = actuator_load(load, ParallelSpring(kp, kp * rest))
            brute[i, j] = optimize(act, motor, limits, ratios=coarse, thermal=True).get("energy", np.nan)
            brute_rms[i, j] = optimize(act, motor, limits, ratios=coarse).get("rms_current", np.nan)
    i_b, j_b = np.unravel_index(np.nanargmin(brute), brute.shape)
    brute_best = (kp_grid[j_b], np.degrees(rest_grid[i_b]), brute[i_b, j_b])
    same = [n for n, d in [("least energy", energy), ("drive limits only", rms_free)]
            if abs(d["ratio"] - rms["ratio"]) < 1e-9 and abs(d["spring"].stiffness - rms["spring"].stiffness) < 1e-6]
    designs = [
        ("SEA only, drive limits", sea),
        ("SEA plus parallel spring, all limits", rms),
    ]

    def row(name: str, d: dict) -> str:
        if np.isnan(d["ratio"]):
            return f"| {name} | none | | | | | | | |"
        s = d.get("spring")
        par = "none" if s is None else f"{s.stiffness:.0f} N·m/rad, rest {np.degrees(s.rest_angle):.1f} deg"
        return (f"| {name} | {d['stiffness']:.0f} | {d['ratio']:.0f} | {par} | {d['energy']:.1f} | "
                f"{d['energy_no_regen']:.1f} | {d['rms_current']:.2f} | {d['peak_current']:.1f} | "
                f"{d['peak_voltage']:.1f} |")

    table = "\n".join(["| design | series k (N·m/rad) | N | parallel spring | energy (J) | no regen. (J) | RMS "
                       "current (A) | peak current (A) | peak voltage (V) |", "|---|---|---|---|---|---|---|---|---|"]
                      + [row(n, d) for n, d in designs])

    # The walking-tuned designs in the other activities.
    conditions = mix_conditions(profiles)
    act_rows = []
    rms_by_activity = {"level": (sea["rms_current"], rms["rms_current"])}
    for a in OTHER:
        other = profiles[conditions[a]].load(DESIGN_MASS_KG)
        r_sea, r_par = check(other, sea, motor, limits), check(other, rms, motor, limits)
        rms_by_activity[a] = (r_sea["rms_current"], r_par["rms_current"])

        def cell(r: dict) -> str:
            flags = [n for k, n in [("ok_voltage", "voltage"), ("ok_peak_current", "peak current"),
                                    ("ok_rms_current", "RMS")] if not r[k]]
            return (f"{r['energy']:.1f} J, {r['rms_current']:.2f} A RMS"
                    + (f", breaks {' and '.join(flags)}" if flags else ""))

        act_rows.append(f"| {NAMES[a]} ({conditions[a][1]}) | {cell(r_sea)} | {cell(r_par)} |")
    act_table = "\n".join(["| activity | SEA only (walking-tuned) | SEA plus parallel spring (walking-tuned) |",
                           "|---|---|---|"] + act_rows)

    # Figure.
    pct = load.t / load.period * 100
    fig, (ax1, ax4, ax2, ax3) = plt.subplots(1, 4, figsize=(18, 4.4), layout="constrained")
    cs = ax4.contourf(kp_grid, np.degrees(rest_grid), brute_rms, levels=np.linspace(4.5, 12, 16), cmap="magma_r",
                      extend="both")
    fig.colorbar(cs, ax=ax4, label="RMS current of the least-energy design (A)", format="%.1f")
    ax4.contour(kp_grid, np.degrees(rest_grid), brute_rms, levels=[limits.rms_current], colors="#2563eb",
                linewidths=2)
    ax4.plot(rms["spring"].stiffness, np.degrees(rms["spring"].rest_angle), "*", ms=15, color="white",
             markeredgecolor="black", label="alternating closed form")
    ax4.plot(brute_best[0], brute_best[1], "o", ms=7, color="none", markeredgecolor="red", mew=1.5,
             label="best grid point")
    ax4.set_xlabel("parallel stiffness k_p (N·m/rad)")
    ax4.set_ylabel("parallel spring rest angle (deg)")
    ax4.set_title(f"Only a narrow band of parallel springs gets\nunder the {limits.rms_current:.2f} A rating (blue line)",
                  fontsize=10, loc="left")
    ax4.legend(fontsize=7.5, loc="lower left")
    spring = rms["spring"]
    ax1.plot(np.degrees(load.angle), load.torque, color="black", lw=2, label="joint moment (80 kg)")
    th = np.linspace(load.angle.min(), load.angle.max(), 50)
    ax1.plot(np.degrees(th), spring.torque(th), color="#b45309", lw=2,
             label=f"parallel spring, {spring.stiffness:.0f} N·m/rad")
    ax1.plot(np.degrees(load.angle), actuator_load(load, spring).torque, color="#1d4ed8", lw=1.2, ls="--",
             label="left for the actuator")
    ax1.axhline(0, color="0.6", lw=0.6)
    ax1.set_xlabel("ankle angle (deg, dorsiflexion +)")
    ax1.set_ylabel("torque (N·m)")
    ax1.set_title("The ankle moment in stance is close to\na spring; a parallel spring carries it", fontsize=10,
                  loc="left")
    ax1.legend(fontsize=7.5)
    i_sea = motor_trajectory(load, sea["compliance"], sea["ratio"], motor)["current"]
    i_par = motor_trajectory(actuator_load(load, spring), rms["compliance"], rms["ratio"], motor)["current"]
    ax2.plot(pct, i_sea, color="0.45", lw=1.6, label=f"SEA only: {sea['rms_current']:.1f} A RMS")
    ax2.plot(pct, i_par, color="#b45309", lw=1.6, label=f"with parallel spring: {rms['rms_current']:.1f} A RMS")
    ax2.axhline(0, color="0.6", lw=0.6)
    ax2.set_xlabel("gait cycle (%)")
    ax2.set_ylabel("motor current (A)")
    ax2.set_title("Motor current over a level-walking stride", fontsize=10, loc="left")
    ax2.legend(fontsize=7.5)
    acts = list(rms_by_activity)
    x = np.arange(len(acts))
    ax3.bar(x - 0.2, [rms_by_activity[a][0] for a in acts], 0.4, color="0.7", edgecolor="black", lw=0.5,
            label="SEA only")
    ax3.bar(x + 0.2, [rms_by_activity[a][1] for a in acts], 0.4, color="#b45309", edgecolor="black", lw=0.5,
            label="with parallel spring")
    ax3.axhline(limits.rms_current, color="#be123c", ls=":", lw=1.8, label=f"rated {limits.rms_current:.2f} A")
    ax3.set_xticks(x, ["level\nwalking"] + [NAMES[a].replace(" ", "\n") for a in OTHER], fontsize=8)
    ax3.set_ylabel("RMS motor current over the stride (A)")
    ax3.set_title("Walking-tuned designs in every activity", fontsize=10, loc="left")
    ax3.legend(fontsize=7.5)
    add_citation(fig)
    fig.savefig(FIGURE, dpi=150)
    plt.close(fig)

    lowest_sea = float(sea_thermal.get("rms_current", np.nan))
    reduction = 1 - rms["rms_current"] / sea["rms_current"]
    text = f"""# Parallel spring

`results/sizing_levelwalk.md` found that no series spring brings the RMS current of level walking under the motor's continuous rating ({limits.rms_current:.2f} A): the copper loss of the torque itself does not depend on the series spring. A spring *in parallel* with the actuator carries part of the joint torque, so the motor delivers less. This page sizes one for level walking at 1.2 m/s ({DESIGN_MASS_KG:.0f} kg, {BUS_VOLTAGE:.0f} V), together with the series spring and ratio.

**Model.** Parallel spring torque `tau_p = -k_p (theta - theta_0)`, linear and acting in both directions; the actuator supplies `tau - tau_p`. For a fixed series compliance and ratio, the mean squared motor current and the energy are convex quadratics in `(k_p, k_p theta_0)` (`docs/derivation.md`, section 9). At each ratio the design alternates the closed-form parallel spring with the closed-form series compliance clipped to the limits, which converges in a few steps (`anklesea.parallel.optimize_parallel`). A test checks that a zero parallel spring reproduces the SEA exactly.

![Parallel spring](figures/parallel_spring.png)

*Left: joint moment against angle over the stride, the parallel spring's line and what remains for the actuator. Middle: motor current. Right: RMS current of the walking-tuned designs in each activity against the motor's rating. Data: Camargo et al. (2021), CC BY 4.0.*

## Level walking

{table}

- The SEA alone (drive limits) needs {sea['rms_current']:.2f} A RMS{"" if np.isnan(lowest_sea) else f", and the lowest any SEA reaches within all limits is {lowest_sea:.2f} A"}. With the parallel spring sized for the least RMS current, the design meets **every** limit, including the continuous current rating, at {rms['rms_current']:.2f} A ({reduction:.0%} lower).
- The parallel spring is {spring.stiffness:.0f} N·m/rad with its rest angle at {np.degrees(spring.rest_angle):.1f} deg of the `ik` ankle angle. The joint-level least-squares fit, which ignores the motor, gives {joint.stiffness:.0f} N·m/rad at {np.degrees(joint.rest_angle):.1f} deg: the motor dynamics shift the optimum only a little.
- The series spring disappears: the best series stiffness is {"rigid" if np.isinf(rms['stiffness']) else f"{rms['stiffness']:.0f} N·m/rad"}. The parallel spring already carries the part of the moment that rises against the joint motion in stance, which is exactly what a series spring would otherwise cancel, and what remains for the actuator gains nothing from series compliance. The ratio drops from {sea['ratio']:.0f} to {rms['ratio']:.0f}: without series compliance the motor turns at N times the joint speed, and at the peak joint speed ({abs(load.velocity).max():.2f} rad/s) the back EMF alone is {abs(load.velocity).max() * rms['ratio'] * motor.torque_constant:.1f} V of the {limits.available_voltage:.1f} V available. Energy per stride falls from {sea['energy']:.1f} to {rms['energy']:.1f} J ({sea['energy_no_regen']:.1f} to {rms['energy_no_regen']:.1f} J without regeneration).
- Sizing the parallel spring for least energy rather than least RMS current{", or imposing only the drive limits," if len(same) == 2 else ""} gives {"the same design" if same else "a different design"}. With a rigid series path the motor speed does not depend on the parallel spring, so the energy's extra term integrates to zero over the stride and both objectives have the same minimizer.
- The brute-force check over {len(kp_grid)} stiffnesses and {len(rest_grid)} rest angles, each with its own least-energy series spring and ratio, finds its best point at {brute_best[0]:.0f} N·m/rad and {brute_best[1]:.1f} deg with {brute_best[2]:.1f} J, against {rms['energy']:.1f} J for the alternating closed form. The second panel maps the RMS current of the least-energy design within the drive limits for every parallel spring on that grid: only a narrow band of stiffness and rest angle, close to the stance moment-angle slope, gets under the rating, and the margin is small ({rms['rms_current']:.2f} A against {limits.rms_current:.2f} A).

## The same designs in the other activities

A parallel spring tuned to the level-walking moment-angle relation acts the same way on ramps and stairs, where that relation differs (same conditions as `results/cross_mode.md`).

{act_table}

## Limits of this design

- The spring is linear and acts in swing too, when the joint moment is near zero; the motor then holds the spring's torque, which is visible in the middle panel after toe-off. A spring that engages only in stance (a clutch or a unidirectional spring, as in several powered ankles) would avoid that; it is not modeled because it makes the problem non-smooth.
- The rest angle is relative to the `ik` angle convention; it would be set during fitting.
- A parallel spring tuned for one activity loads the actuator in others, as the table above shows.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
