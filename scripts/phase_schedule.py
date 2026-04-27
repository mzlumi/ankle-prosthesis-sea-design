#!/usr/bin/env python3
"""Phase-scheduled torque command: oracle gait phase against a causal time-based estimator.

For every kept right-leg stride (the strides of ``scripts/gait_profiles.py``) the command
``m * profile(phase)`` is compared with the subject's own ankle moment from inverse
dynamics, sample by sample at 200 Hz. The profile is the mean moment per kg of the same
activity and condition over the other subjects (leave one subject out), so the command
is never tuned to the person it is tested on. Activity and condition are given to the
controller: mode recognition is not part of this step.

Writes (aggregate only, no per-stride or per-subject data):
  results/phase_schedule.md
  results/figures/phase_schedule.png

Needs the per-stride caches in data/processed/ written by scripts/gait_profiles.py.
Data: Camargo et al. (2021), J. Biomech. 119:110320, doi:10.1016/j.jbiomech.2021.110320, CC BY 4.0.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from anklesea import PROCESSED_DIR, RESULTS_DIR, camargo, strides
from anklesea.designs import default_setup, reference_controllers, reference_designs
from anklesea.phase import command_error_pct, periodic_lookup, phase_error, time_based_phase
from anklesea.plant import SEAParams
from anklesea.plots import ACTIVITY_COLORS, ACTIVITY_NAMES, CITATION
from anklesea.profiles import read_profiles
from anklesea.simulation import Drive, simulate
from anklesea.sizing import BUS_VOLTAGE, DESIGN_MASS_KG, LEVEL_WALK

DELAYS_S = (0.0, 0.025, 0.05)
SHOWN_DELAY_S = 0.025
MIN_STRIDES = 20  # activities with fewer kept strides are tabulated but not summarized
SOURCES = ["oracle, own profile", "oracle"] + [f"time, {1000 * d:.0f} ms" for d in DELAYS_S]
N_BINS = 101


def load_frames() -> pd.DataFrame:
    paths = sorted(PROCESSED_DIR.glob("strides_AB*.pkl"))
    if not paths:
        raise SystemExit("No stride caches in data/processed/: run scripts/gait_profiles.py first.")
    return pd.concat([pd.read_pickle(p) for p in paths], ignore_index=True)


def subject_means(frame: pd.DataFrame) -> pd.DataFrame:
    """Mean moment per kg and stride time per subject, activity, condition and gait-cycle point."""
    return (
        frame.groupby(["subject", "activity", "condition", "gait_pct"])[["moment_Nm_per_kg", "duration_s"]]
        .mean()
        .reset_index()
        .sort_values(["subject", "activity", "condition", "gait_pct"])
    )


def profiles_for(means: pd.DataFrame, subject: str) -> tuple[dict, dict]:
    """Profile and stride time per (activity, condition), averaged over the other subjects."""
    generic, period = {}, {}
    for key, g in means.groupby(["activity", "condition"]):
        others = g[g["subject"] != subject]
        if others.empty:
            continue
        curves = others.groupby("gait_pct")[["moment_Nm_per_kg", "duration_s"]].mean()
        generic[key] = curves["moment_Nm_per_kg"].to_numpy()
        period[key] = float(curves["duration_s"].mean())
    return generic, period


def stride_moment(trial: camargo.Trial, i0: int, i1: int) -> np.ndarray:
    """Right ankle moment over samples ``i0`` to ``i1`` inclusive, gaps filled as in stride cutting."""
    moment = trial.moment["r"][i0 : i1 + 1].copy()
    if np.isnan(moment[-1]):
        moment[-1] = moment[-2]
    return strides.fill_short_gaps(moment, strides.MAX_GAP_SAMPLES)


def evaluate_subject(subject: str, means: pd.DataFrame) -> tuple[list[dict], list[dict]]:
    """Per-stride metrics and per-stride error curves (both kept in memory only)."""
    generic, period = profiles_for(means, subject)
    mass = camargo.subject_mass(subject)
    trials = [camargo.load_trial(r.subject, r.date, r.stem) for r in camargo.list_trials(subjects=[subject])]
    cut = {id(t): strides.cut_strides(t, body_mass=mass) for t in trials}
    kept_list = strides.filter_durations([s for c in cut.values() for s in c])
    kept = {id(s) for s in kept_list}
    # The subject's own profile leaves out the stride under test, so that it is not fitted to it.
    own_sum: dict[tuple[str, str], np.ndarray] = {}
    own_n: dict[tuple[str, str], int] = {}
    for s in kept_list:
        key = (s.activity, s.condition)
        own_sum[key] = own_sum.get(key, 0.0) + s.moment_per_kg
        own_n[key] = own_n.get(key, 0) + 1

    rows, curves = [], []
    for trial in trials:
        hs = trial.heel_strike_phase["r"]
        strike_idx = strides.heel_strike_indices(hs)
        strike_times = trial.time[strike_idx]
        good = [s for s in cut[id(trial)] if id(s) in kept]
        starts = {round(s.start_time, 3): (s.activity, s.condition) for s in good}
        for s in good:
            key = (s.activity, s.condition)
            if key not in generic or own_n[key] < 2:
                continue
            own = (own_sum[key] - s.moment_per_kg) / (own_n[key] - 1)
            i0 = int(np.argmin(np.abs(trial.time - s.start_time)))
            k = int(np.searchsorted(strike_idx, i0))
            i1 = int(strike_idx[k + 1])
            t = trial.time[i0:i1]
            actual = stride_moment(trial, i0, i1)[:-1]
            truth = (t - t[0]) / (trial.time[i1] - t[0])
            prev_start = round(float(trial.time[strike_idx[k - 1]]), 3) if k >= 1 else None
            first = starts.get(prev_start) != key
            row = {"subject": subject, "activity": s.activity, "condition": s.condition, "first": first}
            gait = np.round(truth * (N_BINS - 1)).astype(int)
            curve = {"activity": s.activity, "first": first}
            phases = {"oracle, own profile": truth, "oracle": truth}
            for d in DELAYS_S:
                est = time_based_phase(t, strike_times, period[key], detection_delay=d)
                phases[f"time, {1000 * d:.0f} ms"] = np.where(np.isnan(est), 0.999, est)
            for name, phase in phases.items():
                profile = own if name == "oracle, own profile" else generic[key]
                command = mass * periodic_lookup(profile, phase)
                row[f"err {name}"] = command_error_pct(command, actual)
                if name.startswith("time"):
                    row[f"phase {name}"] = 100.0 * float(np.sqrt(np.mean(phase_error(phase, truth) ** 2)))
                if name in ("oracle", f"time, {1000 * SHOWN_DELAY_S:.0f} ms"):
                    sq = np.bincount(gait, ((command - actual) / mass) ** 2, N_BINS)
                    curve[f"sq {name}"] = sq / np.maximum(np.bincount(gait, minlength=N_BINS), 1)
            pe = phase_error(phases["time, 0 ms"], truth)
            curve["phase"] = np.bincount(gait, 100.0 * pe, N_BINS) / np.maximum(np.bincount(gait, minlength=N_BINS), 1)
            rows.append(row)
            curves.append(curve)
    return rows, curves


def across_subjects(table: pd.DataFrame, columns: list[str], by: list[str]) -> pd.DataFrame:
    """Mean of per-subject means: each subject counts once."""
    per_subject = table.groupby(["subject", *by])[columns].mean()
    out = per_subject.groupby(by).mean()
    out["subjects"] = per_subject.groupby(by).size()
    out["strides"] = table.groupby(by).size()
    return out


def markdown(frame: pd.DataFrame, fmt: str = "{:.1f}") -> str:
    cols = list(frame.columns)
    lines = ["| | " + " | ".join(cols) + " |", "|---|" + "---|" * len(cols)]
    for idx, row in frame.iterrows():
        cells = [str(int(v)) if isinstance(v, (int, np.integer)) or c in ("subjects", "strides") else fmt.format(v) for c, v in row.items()]
        lines.append(f"| {idx} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def tracking_error_pct() -> float:
    """RMS tracking error of the compromise design with PID on level walking (as in ``results/tracking.md``)."""
    motor, _ = default_setup()
    p = SEAParams.from_design(reference_designs()["compromise"], motor)
    run = simulate(read_profiles()[LEVEL_WALK], DESIGN_MASS_KG, reference_controllers(p)["PID"], Drive.from_motor(motor, BUS_VOLTAGE))
    return run.rms_error_pct()


def main() -> None:
    frame = load_frames()
    subjects = sorted(frame["subject"].unique())
    means = subject_means(frame)
    rows, curves = [], []
    for subject in subjects:
        r, c = evaluate_subject(subject, means)
        rows += r
        curves += c
        print(subject, len(r), "strides")
    table = pd.DataFrame(rows)
    activities = [a for a in strides.ACTIVITIES if a in set(table["activity"])]

    err_cols = [f"err {s}" for s in SOURCES]
    errors = across_subjects(table, err_cols, ["activity"]).loc[activities]
    err_table = errors[err_cols].T
    err_table.index = SOURCES
    err_table.columns = [ACTIVITY_NAMES[a] for a in activities]
    counts = errors[["subjects", "strides"]].T
    counts.columns = err_table.columns

    phase_cols = [f"phase time, {1000 * d:.0f} ms" for d in DELAYS_S]
    split = across_subjects(table, ["phase time, 0 ms", "err oracle", "err time, 0 ms"], ["activity", "first"])
    split_rows = []
    for a in activities:
        for first in (False, True):
            if (a, first) not in split.index:
                continue
            r = split.loc[(a, first)]
            split_rows.append(
                {
                    "": f"{ACTIVITY_NAMES[a]}, {'first stride of a segment' if first else 'later strides'}",
                    "phase error (% cycle)": f"{r['phase time, 0 ms']:.1f}",
                    "torque error, oracle (%)": f"{r['err oracle']:.1f}",
                    "torque error, time 0 ms (%)": f"{r['err time, 0 ms']:.1f}",
                    "strides": int(r["strides"]),
                }
            )
    split_md = pd.DataFrame(split_rows)
    split_text = "\n".join(
        ["| " + " | ".join(split_md.columns) + " |", "|" + "---|" * len(split_md.columns)]
        + ["| " + " | ".join(str(v) for v in r) + " |" for r in split_md.itertuples(index=False)]
    )
    phase_by = across_subjects(table, phase_cols, ["activity"]).loc[activities, phase_cols].T
    phase_by.index = [f"time, {1000 * d:.0f} ms" for d in DELAYS_S]
    phase_by.columns = [ACTIVITY_NAMES[a] for a in activities]

    # Figure.
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    ax = axes[0]
    width = 0.8 / len(SOURCES)
    shades = ["#9ca3af", "#111827", "#93c5fd", "#3b82f6", "#1e3a8a"]
    for j, (src, shade) in enumerate(zip(SOURCES, shades)):
        ax.bar(np.arange(len(activities)) + (j - (len(SOURCES) - 1) / 2) * width, err_table.loc[src], width, color=shade, label=src)
    ax.set_xticks(np.arange(len(activities)), [ACTIVITY_NAMES[a].replace(" ", "\n") for a in activities], fontsize=8)
    ax.set_ylabel("RMS(command - inverse dynamics) (% of stride peak)")
    ax.set_title("torque command error per stride")
    ax.legend(fontsize=7, title="phase source", title_fontsize=7)
    ax.grid(True, axis="y", alpha=0.3)

    curve_frame = pd.DataFrame(curves)
    x = np.linspace(0, 100, N_BINS)
    for a in activities:
        sel = curve_frame[curve_frame["activity"] == a]
        axes[1].plot(x, np.mean(np.stack(sel["phase"]), axis=0), color=ACTIVITY_COLORS[a], label=ACTIVITY_NAMES[a])
    axes[1].axhline(0, color="k", lw=0.6)
    axes[1].set_xlabel("true gait cycle (%)")
    axes[1].set_ylabel("mean phase error (% of cycle)")
    axes[1].set_title("time-based estimate, no detection delay")
    axes[1].legend(fontsize=7)
    axes[1].grid(True, alpha=0.3)

    shown = f"time, {1000 * SHOWN_DELAY_S:.0f} ms"
    for a in ("treadmill", "stairascent"):
        if a not in activities:
            continue
        sel = curve_frame[curve_frame["activity"] == a]
        for name, style in (("oracle", "-"), (shown, "--")):
            rms = np.sqrt(np.mean(np.stack(sel[f"sq {name}"]), axis=0))
            axes[2].plot(x, rms, style, color=ACTIVITY_COLORS[a], label=f"{ACTIVITY_NAMES[a]}, {name}")
    axes[2].set_xlabel("true gait cycle (%)")
    axes[2].set_ylabel("RMS torque command error (N·m/kg)")
    axes[2].set_title("where in the stride the error is")
    axes[2].legend(fontsize=7)
    axes[2].grid(True, alpha=0.3)
    fig.text(0.01, 0.01, CITATION + f" {len(subjects)} subjects, leave-one-subject-out profiles.", fontsize=7, color="0.4")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    out = RESULTS_DIR / "figures" / "phase_schedule.png"
    fig.savefig(out, dpi=150)
    print("wrote", out)

    # Text.
    e = err_table
    n = counts.loc["strides"]
    big = [c for c in e.columns if n[c] >= MIN_STRIDES]
    small = [c for c in e.columns if n[c] < MIN_STRIDES]
    tm = ACTIVITY_NAMES["treadmill"]
    shown_name = f"time, {1000 * SHOWN_DELAY_S:.0f} ms"
    last_name = f"time, {1000 * DELAYS_S[-1]:.0f} ms"
    floor_own = e.loc["oracle, own profile", big]
    generic = e.loc["oracle", big]
    added = (e.loc["time, 0 ms", big] - e.loc["oracle", big])
    per_10ms = 10 * (e.loc[last_name, tm] - e.loc["time, 0 ms", tm]) / (1000 * DELAYS_S[-1])
    penalty = split["err time, 0 ms"] - split["err oracle"]
    climbs = [a for a in ("rampascent", "rampdescent", "stairascent", "stairdescent") if a in activities]
    climb_first = table[table["activity"].isin(climbs) & table["first"]]
    climb_share = 100 * len(climb_first) / max(len(table[table["activity"].isin(climbs)]), 1)
    climb_penalty = float((climb_first["err time, 0 ms"] - climb_first["err oracle"]).mean())
    tm_phase = {first: split.loc[("treadmill", first), "phase time, 0 ms"] for first in (False, True) if ("treadmill", first) in split.index}
    tracking = tracking_error_pct()
    own_share = 100 * (1 - floor_own / generic)
    own_gain = generic - floor_own
    small_text = (
        f"{' and '.join(small).capitalize()} rest on fewer than {MIN_STRIDES} strides (only one or two right-leg stances per pass land on a force plate, `results/gait_profiles.md`), so their columns are noise at this sample size and the summary below leaves them out."
        if small else ""
    )

    text = f"""# Phase-scheduled torque command

The torque reference of a phase-scheduled ankle controller is `m * profile(phase)`: the mean ankle moment per kg of the current activity and condition, looked up at the estimated gait phase and scaled by the user's mass `m`. This step asks how close that command comes to the moment the subject actually produced (inverse dynamics in the Camargo et al. (2021) dataset), stride by stride at 200 Hz, and how much of the gap comes from the phase estimate. Script: `scripts/phase_schedule.py`; code: `src/anklesea/phase.py`.

## Setup

- **Strides**: the {len(table)} kept right-leg strides of {len(subjects)} subjects ({", ".join(subjects)}) from `scripts/gait_profiles.py`.
- **Profile**: the mean moment per kg of the same activity and condition over the *other* subjects, so the command is a generic one, not tuned to the person it is tested on. One row also uses the subject's own mean profile, which is the best a fixed profile tuned to that user can do. With {len(subjects)} subjects the generic profile of each subject is {"the other subject's mean" if len(subjects) == 2 else "a mean over " + str(len(subjects) - 1) + " subjects"}.
- **Activity and condition** are given to the controller. Recognizing them from sensors is a separate problem (see the companion repository imu-locomotion-gait-phase).
- **Oracle phase**: linear in time between the dataset's heel strikes (`gcRight`). It knows when the current stride will end, which no device does.
- **Time-based estimate**: the time since the last detected heel strike over the duration of the previous stride, held at 99.9 % if the stride runs long and reset at the next detection; the mean stride time of the profile is used when there is no plausible previous stride (start of a trial or after a pause). The heel strike is detected {", ".join(f"{1000 * d:.0f}" for d in DELAYS_S)} ms after the dataset's event. Detection latency is an assumption (no detector is modeled; `docs/assumptions.md`). **This estimator stands in for the IMU-based gait phase estimator of the companion repository imu-locomotion-gait-phase, which has no estimator yet.** It is the simplest causal estimator, not a recommendation.
- **Error**: RMS of command minus inverse-dynamics moment over the stride, in percent of that stride's peak moment, averaged per subject and then across subjects.

## Torque command error (RMS, % of stride peak)

{markdown(err_table)}

Subjects and strides per activity:

{markdown(counts.astype(int))}

## Phase error of the time-based estimate (RMS, % of the gait cycle)

{markdown(phase_by)}

## First stride of a segment against later strides (no detection delay)

A stride is the first of a segment when the stride before it is not a kept stride of the same activity and condition: the first steady stride on a staircase or ramp, the first treadmill stride at a new belt speed, or a level-ground stride, since only one stride per pass has a force plate under it.

{split_text}

## Reading the results
{small_text}

- **Even with the oracle phase a generic profile misses by {generic.min():.0f} to {generic.max():.0f} %** of the stride peak. A fixed profile cannot follow stride-to-stride and person-to-person variation. The subject's own mean profile (from the subject's other strides) brings it to {floor_own.min():.0f} to {floor_own.max():.0f} %: {own_share.min():.0f} to {own_share.max():.0f} % of the generic error is the difference between people, not between strides. Tuning the profile to the user gains {own_gain.min():.0f} to {own_gain.max():.0f} points, more than replacing the time-based estimate with the oracle (next two points). With only {len(subjects)} subjects the generic profile is a poor population mean, so this gap will shrink with more subjects.
- **On {tm} the time-based estimate is nearly as good as the oracle**: its phase error is {tm_phase.get(False, float("nan")):.1f} % of the cycle on strides at a steady speed and {tm_phase.get(True, float("nan")):.1f} % on the first stride after a speed change or a dropped stride, and it adds {added[tm]:+.1f} percentage points to the command error.
- **On stairs and ramps it is worse**, adding {added.drop(tm, errors="ignore").min():+.1f} to {added.drop(tm, errors="ignore").max():+.1f} points. Steady stair and ramp segments are only a few strides long, so {climb_share:.0f} % of their kept strides are the first of a segment, and the estimator predicts their duration from a stride of another activity (level walking or a transition). On those first strides it adds {climb_penalty:+.1f} points on average.
- **Detection latency shifts the whole command later**, including the steep rise and fall of push-off. On {tm} the error grows from {e.loc["time, 0 ms", tm]:.1f} % without latency to {e.loc[last_name, tm]:.1f} % at {1000 * DELAYS_S[-1]:.0f} ms, about {per_10ms:.1f} points per 10 ms. If the latency is known and constant, the estimator can back-date the heel strike and return to the no-latency row.
- **For scale**: the torque controller tracks its reference to {tracking:.1f} % RMS on treadmill walking at 1.2 m/s (`results/tracking.md`), far below these command errors. In a phase-scheduled ankle the reference limits the result, not the actuator.

![phase schedule](figures/phase_schedule.png)

*Figure: left, torque command error per activity for each phase source; middle, mean phase error of the time-based estimate over the gait cycle; right, RMS command error over the gait cycle for {tm} and stair ascent, oracle phase (solid) and the time-based estimate with {1000 * SHOWN_DELAY_S:.0f} ms detection latency (dashed). Data: Camargo et al. (2021), CC BY 4.0; {len(subjects)} subjects, leave-one-subject-out profiles.*
"""
    (RESULTS_DIR / "phase_schedule.md").write_text(text)
    print(text[text.index("## Reading"):])


if __name__ == "__main__":
    main()
