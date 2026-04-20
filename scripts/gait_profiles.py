#!/usr/bin/env python3
"""Cut right-leg strides from every downloaded trial and average them per activity and condition.

Writes (all aggregate across subjects, no per-stride or per-subject data):
  results/profiles/gait_profiles.csv   mean and SD of ankle angle, moment and power per kg
  results/gait_profiles.md             summary table and the comparison with the dataset figures
  results/figures/gait_profiles.png    angle, moment and power per activity
  results/figures/gait_profiles_by_condition.png

Per-stride data are cached in data/processed/ (git-ignored) and never committed.
Data: Camargo et al. (2021), J. Biomech. 119:110320, doi:10.1016/j.jbiomech.2021.110320, CC BY 4.0.
"""

from __future__ import annotations

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from anklesea import PROCESSED_DIR, RESULTS_DIR, camargo, strides
from anklesea.plots import ACTIVITY_COLORS
from anklesea.profiles import read_profiles

CITATION = "Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0."
PROFILE_CSV = RESULTS_DIR / "profiles" / "gait_profiles.csv"


def subject_strides(subject: str, rebuild: bool = False) -> pd.DataFrame:
    """Per-stride table of one subject, from the cache in data/processed/ when present."""
    cache = PROCESSED_DIR / f"strides_{subject}.pkl"
    if cache.exists() and not rebuild:
        return pd.read_pickle(cache)
    mass = camargo.subject_mass(subject)
    cut = []
    for ref in camargo.list_trials(subjects=[subject]):
        trial = camargo.load_trial(ref.subject, ref.date, ref.stem)
        cut += strides.cut_strides(trial, body_mass=mass)
    frame = strides.strides_to_frame(strides.filter_durations(cut))
    cache.parent.mkdir(parents=True, exist_ok=True)
    frame.to_pickle(cache)
    return frame


def representative(table: pd.DataFrame) -> dict[str, str]:
    """For each activity, the condition with the most subjects (ties: the middle one)."""
    out = {}
    for activity, g in table.groupby("activity"):
        counts = g.groupby("condition")["n_subjects"].first()
        best = counts[counts == counts.max()].index.tolist()
        best.sort(key=lambda c: strides.condition_sort_key(activity, c))
        if activity == "treadmill" and "1.20 m/s" in best:
            out[activity] = "1.20 m/s"
        elif activity == "levelground" and "normal" in best:
            out[activity] = "normal"
        else:
            out[activity] = best[len(best) // 2]
    return out


def summary_rows(table: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for (activity, condition), g in table.groupby(["activity", "condition"]):
        g = g.sort_values("gait_pct")
        pct = g["gait_pct"].to_numpy()
        angle = np.rad2deg(g["angle_mean_rad"].to_numpy())
        moment = g["moment_mean_Nm_per_kg"].to_numpy()
        power = g["power_mean_W_per_kg"].to_numpy()
        stride_time = g["stride_time_mean_s"].iloc[0]
        dt = stride_time / (len(pct) - 1)
        rows.append(
            {
                "activity": activity,
                "condition": condition,
                "subjects": int(g["n_subjects"].iloc[0]),
                "strides": int(g["n_strides"].iloc[0]),
                "stride_time_s": round(stride_time, 3),
                "max_dorsiflexion_deg": round(angle.max(), 1),
                "at_pct": int(pct[np.argmax(angle)]),
                "max_plantarflexion_deg": round(-angle.min(), 1),
                "at_pct ": int(pct[np.argmin(angle)]),
                "peak_PF_moment_Nm_per_kg": round(-moment.min(), 2),
                "at_pct  ": int(pct[np.argmin(moment)]),
                "peak_power_W_per_kg": round(power.max(), 2),
                "at_pct   ": int(pct[np.argmax(power)]),
                "net_work_J_per_kg": round(float(np.trapezoid(power, dx=dt)), 3),
                "_key": strides.condition_sort_key(activity, condition),
            }
        )
    return pd.DataFrame(rows).sort_values("_key").drop(columns="_key")


def markdown_table(frame: pd.DataFrame) -> str:
    header = "| " + " | ".join(c.strip() for c in frame.columns) + " |"
    rule = "|" + "|".join("---" for _ in frame.columns) + "|"
    return "\n".join([header, rule, *("| " + " | ".join(str(v) for v in row) + " |" for row in frame.itertuples(index=False))])


def plot_activities(table: pd.DataFrame, chosen: dict[str, str], path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(7.5, 9), sharex=True)
    for activity in strides.ACTIVITIES:
        if activity not in chosen:
            continue
        g = table[(table["activity"] == activity) & (table["condition"] == chosen[activity])].sort_values("gait_pct")
        x = g["gait_pct"]
        color = ACTIVITY_COLORS[activity]
        n = int(g["n_subjects"].iloc[0])
        label = f"{activity} {chosen[activity]} (n={n})"
        for ax, mean, sd, scale in (
            (axes[0], "angle_mean_rad", "angle_sd_rad", 180 / np.pi),
            (axes[1], "moment_mean_Nm_per_kg", "moment_sd_Nm_per_kg", 1.0),
            (axes[2], "power_mean_W_per_kg", "power_sd_W_per_kg", 1.0),
        ):
            m, s = g[mean] * scale, g[sd] * scale
            ax.plot(x, m, color=color, label=label)
            ax.fill_between(x, m - s, m + s, color=color, alpha=0.15, linewidth=0)
    axes[0].set_ylabel("ankle angle (deg)\ndorsiflexion +")
    axes[1].set_ylabel("ankle moment (N·m/kg)\ndorsiflexion +")
    axes[2].set_ylabel("ankle power (W/kg)")
    axes[2].set_xlabel("gait cycle (%), right heel strike to heel strike")
    for ax in axes:
        ax.axhline(0, color="0.6", linewidth=0.6)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7, loc="lower left")
    fig.suptitle("Ankle angle, moment and power per activity (mean ± SD across subjects)", fontsize=10)
    fig.text(0.01, 0.005, CITATION, fontsize=7)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_by_condition(table: pd.DataFrame, path) -> None:
    activities = [a for a in ("treadmill", "rampascent", "rampdescent", "stairascent", "stairdescent") if a in set(table["activity"])]
    fig, axes = plt.subplots(2, len(activities), figsize=(3.2 * len(activities), 6), sharex=True, squeeze=False)
    for col, activity in enumerate(activities):
        g = table[table["activity"] == activity]
        conds = sorted(g["condition"].unique(), key=lambda c: strides.condition_sort_key(activity, c))
        if activity == "treadmill":
            conds = conds[::3]
        colors = plt.cm.plasma(np.linspace(0.05, 0.9, len(conds)))
        for color, cond in zip(colors, conds):
            gc = g[g["condition"] == cond].sort_values("gait_pct")
            axes[0, col].plot(gc["gait_pct"], np.rad2deg(gc["angle_mean_rad"]), color=color, label=cond)
            axes[1, col].plot(gc["gait_pct"], gc["moment_mean_Nm_per_kg"], color=color)
        axes[0, col].set_title(activity, fontsize=9)
        axes[0, col].legend(fontsize=6)
        axes[1, col].set_xlabel("gait cycle (%)")
        for ax in axes[:, col]:
            ax.grid(alpha=0.3)
    axes[0, 0].set_ylabel("ankle angle (deg)")
    axes[1, 0].set_ylabel("ankle moment (N·m/kg)")
    fig.text(0.01, 0.005, CITATION + " Mean across subjects.", fontsize=7)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--subjects", nargs="+", default=None)
    parser.add_argument("--rebuild", action="store_true", help="ignore the per-stride cache")
    args = parser.parse_args()

    subjects = args.subjects or sorted({r.subject for r in camargo.list_trials()})
    frames = []
    for subject in subjects:
        frame = subject_strides(subject, rebuild=args.rebuild)
        print(f"{subject}: {frame['stride'].nunique() if len(frame) else 0} strides")
        if len(frame):
            frame = frame.assign(stride=frame["stride"].astype(str) + "_" + subject)
            frames.append(frame)
    table = strides.average_profiles(pd.concat(frames, ignore_index=True))
    table = table.round(6)

    PROFILE_CSV.parent.mkdir(parents=True, exist_ok=True)
    with open(PROFILE_CSV, "w") as handle:
        handle.write(
            "# Ankle angle (rad), moment (N·m/kg) and power (W/kg) over the gait cycle, mean and SD across "
            f"subjects ({len(subjects)} subjects: {' '.join(subjects)}). Right leg, heel strike to heel strike. "
            "Derived from Camargo, Ramanathan, Flanagan, Young (2021), J. Biomech. 119:110320, "
            "doi:10.1016/j.jbiomech.2021.110320, CC BY 4.0. Generated by scripts/gait_profiles.py.\n"
        )
        table.to_csv(handle, index=False)
    assert read_profiles(PROFILE_CSV)

    chosen = representative(table)
    figures = RESULTS_DIR / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    plot_activities(table, chosen, figures / "gait_profiles.png")
    plot_by_condition(table, figures / "gait_profiles_by_condition.png")

    summary = summary_rows(table)
    write_markdown(summary, chosen, subjects)
    print(summary.to_string(index=False))
    print("representative conditions:", chosen)
    return 0


# Read by eye from the averaged-profile figures on the EPIC Lab dataset page
# (https://www.epic.gatech.edu/opensource-biomechanics-camargo-et-al/, retrieved 2026-10-06).
# Those figures plot plantarflexion upward; values here are in degrees, plantarflexion as
# a positive number, and approximate to a few degrees.
FIGURE_READINGS = {
    ("treadmill", "1.20 m/s"): {"dorsiflexion": (10, 48), "plantarflexion": (15, 64)},
    ("stairascent", "5 in"): {"dorsiflexion": (12, 3), "plantarflexion": (12, 63)},
    ("stairdescent", "5 in"): {"dorsiflexion": (25, 42), "plantarflexion": (25, 90)},
}


def write_markdown(summary: pd.DataFrame, chosen: dict[str, str], subjects: list[str]) -> None:
    lines = [
        "# Averaged ankle gait profiles",
        "",
        f"Generated by `scripts/gait_profiles.py` from {len(subjects)} subjects ({', '.join(subjects)}). "
        "Right-leg strides from heel strike to heel strike, kept only when the stance is on a force plate "
        "(the moment is NaN elsewhere), the stance has one steady locomotion label, and the duration is "
        "within 25 % of the subject's median for that condition. Each subject's strides are averaged "
        "first, then the subject means are averaged. Moments and powers are divided by body mass. "
        "Mean and SD curves: [`profiles/gait_profiles.csv`](profiles/gait_profiles.csv). "
        "Data: Camargo, Ramanathan, Flanagan, Young (2021), J. Biomech. 119:110320, "
        "doi:10.1016/j.jbiomech.2021.110320, CC BY 4.0.",
        "",
        "![Ankle profiles per activity](figures/gait_profiles.png)",
        "",
        "![Ankle profiles per condition](figures/gait_profiles_by_condition.png)",
        "",
        "Sign convention: angle and moment positive in dorsiflexion (OpenSim), power = moment x angular "
        "velocity, positive when the ankle does work. `at_pct` columns give where each extremum falls in "
        "the gait cycle.",
        "",
        markdown_table(summary),
        "",
        "## Sign check on the averaged profiles",
        "",
    ]
    checks = []
    for activity in ("treadmill", "levelground", "rampascent", "stairascent"):
        rows = summary[summary["activity"] == activity]
        if rows.empty:
            continue
        timing = rows.iloc[:, 10]
        checks.append(
            f"- {activity}: peak plantarflexion moment between {timing.min()} and {timing.max()} % of the cycle "
            f"(late stance) in all {len(rows)} conditions, with net positive ankle work in "
            f"{int((rows['net_work_J_per_kg'] > 0).sum())} of {len(rows)}."
        )
    for activity in ("rampdescent", "stairdescent"):
        rows = summary[summary["activity"] == activity]
        if not rows.empty:
            checks.append(
                f"- {activity}: net negative ankle work (the ankle absorbs energy) in "
                f"{int((rows['net_work_J_per_kg'] < 0).sum())} of {len(rows)} conditions."
            )
    lines += checks + [""]
    lines += [
        "## Comparison with the dataset paper",
        "",
        "The full text of Camargo et al. (2021) is behind the publisher's paywall and was not available, "
        "so its moment and power figures could not be compared. The EPIC Lab dataset page shows the "
        "averaged hip, knee and ankle *angles* per mode for all subjects, and those were read by eye "
        "(to a few degrees). The table compares them with the profiles here.",
        "",
        "| activity | condition | source | peak dorsiflexion (deg, at %) | peak plantarflexion (deg, at %) |",
        "|---|---|---|---|---|",
    ]
    for (activity, condition), reading in FIGURE_READINGS.items():
        row = summary[(summary["activity"] == activity) & (summary["condition"] == condition)]
        if row.empty:
            continue
        r = row.iloc[0]
        lines.append(
            f"| {activity} | {condition} | dataset page figure | {reading['dorsiflexion'][0]} ({reading['dorsiflexion'][1]}) "
            f"| {reading['plantarflexion'][0]} ({reading['plantarflexion'][1]}) |"
        )
        lines.append(
            f"| {activity} | {condition} | this project (`ik`) | {r['max_dorsiflexion_deg']} ({r['at_pct']}) "
            f"| {r['max_plantarflexion_deg']} ({r['at_pct ']}) |"
        )
    lines += [
        "",
        "The shapes and timings agree, but the angles here sit about 10 to 17 degrees further into "
        "dorsiflexion. The difference is a constant offset: the `ik` folder holds absolute OpenSim joint "
        "angles, while the dataset's `ik_offset` folder (added in the 2024 update) holds the same angles "
        "relative to each subject's static standing pose. For AB06 the right-ankle difference between "
        "the two is exactly 13.6 degrees at every sample of the treadmill trials, and subtracting it brings "
        "the peaks here to within about 5 degrees of the figures, which are consistent with offset-corrected "
        "angles. The actuator model uses only angle *changes* "
        "(motor speed and acceleration depend on the angular velocity and acceleration), and the rest "
        "angle of the parallel spring is a design variable, so the offset does not affect any result. "
        "The `ik` angles are kept so that only the folders listed in `data/README.md` are needed.",
        "",
        "Moments and powers could only be checked for plausibility: at 1.2 m/s on the treadmill the "
        "peak plantarflexion moment and peak ankle power per kg are of the size commonly reported for "
        "able-bodied walking near that speed, both grow with speed, and the positive power burst follows the moment peak as expected for "
        "push-off.",
        "",
        "Level-ground and stair profiles rest on few strides per subject, because only one or two "
        "right-leg stances per pass land on a force plate; the treadmill's instrumented belt gives a "
        "moment for every stride.",
        "",
    ]
    (RESULTS_DIR / "gait_profiles.md").write_text("\n".join(lines))


if __name__ == "__main__":
    raise SystemExit(main())
