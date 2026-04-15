"""Cut right-leg strides, keep the clean ones, normalize them to the gait cycle and average.

A stride runs from one right heel strike to the next, found where the dataset's
``gcRight/HeelStrike`` phase wraps from about 100 back to 0. Each stride is assigned an
activity (``treadmill``, ``levelground``, ``rampascent``, ``rampdescent``,
``stairascent``, ``stairdescent``) and a condition (belt speed, speed class, incline
or step height). A stride is dropped if it has NaN in the ankle angle or moment, if
the locomotion label changes within it (standing, turning and transition strides), or
if its duration is implausible. Kept strides are resampled to 0 to 100 % of the gait
cycle. The torque is divided by the subject's body mass, so the averaged profiles are
in N·m/kg and can be scaled to any user mass.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

from anklesea import camargo

N_POINTS = 101
GAIT_PCT = np.linspace(0.0, 100.0, N_POINTS)
MIN_STRIDE_S = 0.6
MAX_STRIDE_S = 2.5
DURATION_TOLERANCE = 0.25  # fraction of the median stride time of the same subject and condition
MAX_GAP_SAMPLES = 10  # NaN runs up to 50 ms inside a stride are interpolated

STEADY_LABELS = {
    "levelground": {"walk"},
    "ramp": {"rampascent", "rampdescent"},
    "stair": {"stairascent", "stairdescent"},
}
ACTIVITIES = ("treadmill", "levelground", "rampascent", "rampdescent", "stairascent", "stairdescent")


@dataclass
class Stride:
    """One right-leg stride, resampled to ``GAIT_PCT``."""

    subject: str
    trial: str
    activity: str
    condition: str
    start_time: float
    duration: float
    angle: np.ndarray  # rad
    moment: np.ndarray  # N·m
    moment_per_kg: np.ndarray  # N·m/kg


def fill_short_gaps(x: np.ndarray, max_gap: int) -> np.ndarray:
    """Linearly interpolate interior NaN runs of at most ``max_gap`` samples; leave longer ones.

    Strides on a force plate often have a NaN sample or two at the edge of the plate
    contact; a whole stance off the plates is a long NaN run and stays NaN.
    """
    x = np.asarray(x, dtype=float).copy()
    isnan = np.isnan(x)
    if not isnan.any() or isnan.all():
        return x
    idx = np.arange(len(x))
    edges = np.diff(np.concatenate([[0], isnan.astype(int), [0]]))
    for start, stop in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
        if stop - start <= max_gap and start > 0 and stop < len(x):
            x[start:stop] = np.interp(idx[start:stop], [start - 1, stop], [x[start - 1], x[stop]])
    return x


def heel_strike_indices(phase: np.ndarray, drop: float = 50.0) -> np.ndarray:
    """Sample indices where the heel-strike phase wraps around (a new stride starts)."""
    phase = np.asarray(phase, dtype=float)
    return np.flatnonzero(np.diff(phase) < -drop) + 1


def resample_stride(time: np.ndarray, values: np.ndarray, n_points: int = N_POINTS) -> np.ndarray:
    """Linear interpolation of one stride onto ``n_points`` equally spaced gait-cycle points."""
    s = (time - time[0]) / (time[-1] - time[0])
    return np.interp(np.linspace(0.0, 1.0, n_points), s, values)


def toe_off_index(trial: camargo.Trial, i0: int, i1: int, leg: str = "r") -> int:
    """First sample after ``i0`` where the toe-off phase wraps, or 60 % of the stride if none."""
    wraps = heel_strike_indices(trial.toe_off_phase[leg][i0:i1])
    return i0 + int(wraps[0]) if len(wraps) else i0 + int(0.6 * (i1 - i0))


def condition_of(trial: camargo.Trial, i0: int, i_off: int, i1: int) -> tuple[str, str] | None:
    """Activity and condition of the stride from ``i0`` to ``i1`` (exclusive), or None.

    The stance (``i0`` to toe-off ``i_off``), where the ankle carries the load, must have
    one steady label. The swing may also carry that activity's transition labels
    (``rampascent-walk`` after ``rampascent``, for example), because the steady stair and
    ramp segments are only one to three seconds long. Level-ground strides must be
    ``walk`` throughout.
    """
    mode = trial.mode
    if mode == "treadmill":
        speed = trial.belt_speed[i0:i1]
        if speed is None or np.ptp(speed) > 0.005 or speed.mean() < 0.3:
            return None
        return "treadmill", f"{np.round(speed.mean() / 0.05) * 0.05:.2f} m/s"
    if trial.labels is None:
        return None
    stance = set(trial.labels[i0:i_off])
    if len(stance) != 1:
        return None
    label = stance.pop()
    if label not in STEADY_LABELS[mode]:
        return None
    swing_ok = {label} if mode == "levelground" else {label, f"walk-{label}", f"{label}-walk"}
    if not set(trial.labels[i_off:i1]) <= swing_ok:
        return None
    if mode == "levelground":
        return "levelground", trial.name.condition
    if mode == "ramp":
        return label, f"{np.rad2deg(trial.ramp_incline_rad):.1f} deg"
    return label, f"{trial.stair_height_m / camargo.INCH:.0f} in"


def cut_strides(trial: camargo.Trial, body_mass: float, leg: str = "r") -> list[Stride]:
    """Strides of one trial that have a single steady condition and no NaN.

    The duration filter that compares strides with each other is applied later by
    :func:`filter_durations`, once all trials of a subject are cut.
    """
    strikes = heel_strike_indices(trial.heel_strike_phase[leg])
    strides = []
    for i0, i1 in zip(strikes[:-1], strikes[1:]):
        duration = trial.time[i1] - trial.time[i0]
        if not MIN_STRIDE_S <= duration <= MAX_STRIDE_S:
            continue
        angle = trial.angle[leg][i0 : i1 + 1]
        moment = trial.moment[leg][i0 : i1 + 1].copy()
        # The moment is NaN in any stance off a force plate. The next heel strike (the
        # last sample) may start such a stance; the end of swing carries almost no
        # moment, so the last swing value stands in for it.
        if np.isnan(moment[-1]):
            moment[-1] = moment[-2]
        moment = fill_short_gaps(moment, MAX_GAP_SAMPLES)
        if np.isnan(angle).any() or np.isnan(moment).any():
            continue
        cond = condition_of(trial, i0, toe_off_index(trial, i0, i1, leg), i1)
        if cond is None:
            continue
        t = trial.time[i0 : i1 + 1]
        m = resample_stride(t, moment)
        strides.append(
            Stride(
                subject=trial.subject,
                trial=trial.name.stem,
                activity=cond[0],
                condition=cond[1],
                start_time=float(t[0]),
                duration=float(duration),
                angle=resample_stride(t, angle),
                moment=m,
                moment_per_kg=m / body_mass,
            )
        )
    return strides


def filter_durations(strides: list[Stride], tolerance: float = DURATION_TOLERANCE) -> list[Stride]:
    """Drop strides whose duration is more than ``tolerance`` away from the median of
    the same subject, activity and condition."""
    if not strides:
        return []
    frame = pd.DataFrame({"key": [(s.subject, s.activity, s.condition) for s in strides], "d": [s.duration for s in strides]})
    median = frame.groupby("key")["d"].transform("median").to_numpy()
    keep = np.abs(frame["d"].to_numpy() - median) <= tolerance * median
    return [s for s, k in zip(strides, keep) if k]


def stride_power_per_kg(stride: Stride) -> np.ndarray:
    """Ankle power in W/kg over the stride: moment per kg times angular velocity."""
    t = GAIT_PCT / 100.0 * stride.duration
    return stride.moment_per_kg * np.gradient(stride.angle, t)


def strides_to_frame(strides: list[Stride]) -> pd.DataFrame:
    """Long table with one row per stride and gait-cycle point (kept out of the repo)."""
    rows = []
    for k, s in enumerate(strides):
        power = stride_power_per_kg(s)
        rows.append(
            pd.DataFrame(
                {
                    "stride": k,
                    "subject": s.subject,
                    "trial": s.trial,
                    "activity": s.activity,
                    "condition": s.condition,
                    "duration_s": s.duration,
                    "gait_pct": GAIT_PCT,
                    "angle_rad": s.angle,
                    "moment_Nm_per_kg": s.moment_per_kg,
                    "power_W_per_kg": power,
                }
            )
        )
    return pd.concat(rows, ignore_index=True) if rows else pd.DataFrame()


def average_profiles(frame: pd.DataFrame, min_strides_per_subject: int = 1) -> pd.DataFrame:
    """Mean and SD across subjects per activity, condition and gait-cycle point.

    Each subject's strides are averaged first, so subjects with many strides do not
    dominate. The SD is across subject means (zero when only one subject is present).
    Stride time is averaged the same way.
    """
    counts = frame.groupby(["activity", "condition", "subject"])["stride"].nunique().rename("n_strides")
    per_subject = (
        frame.groupby(["activity", "condition", "subject", "gait_pct"])[
            ["angle_rad", "moment_Nm_per_kg", "power_W_per_kg", "duration_s"]
        ]
        .mean()
        .join(counts, on=["activity", "condition", "subject"])
        .reset_index()
    )
    per_subject = per_subject[per_subject["n_strides"] >= min_strides_per_subject]
    grouped = per_subject.groupby(["activity", "condition", "gait_pct"])
    out = grouped[["angle_rad", "moment_Nm_per_kg", "power_W_per_kg", "duration_s"]].agg(["mean", "std"])
    out.columns = [f"{name}_{stat}" for name, stat in out.columns]
    out = out.rename(
        columns={
            "angle_rad_mean": "angle_mean_rad",
            "angle_rad_std": "angle_sd_rad",
            "moment_Nm_per_kg_mean": "moment_mean_Nm_per_kg",
            "moment_Nm_per_kg_std": "moment_sd_Nm_per_kg",
            "power_W_per_kg_mean": "power_mean_W_per_kg",
            "power_W_per_kg_std": "power_sd_W_per_kg",
            "duration_s_mean": "stride_time_mean_s",
            "duration_s_std": "stride_time_sd_s",
        }
    )
    out["n_subjects"] = grouped["subject"].nunique()
    out["n_strides"] = grouped["n_strides"].sum()
    out = out.fillna({c: 0.0 for c in out.columns if c.endswith("_sd_rad") or "_sd_" in c})
    return out.reset_index()


def condition_sort_key(activity: str, condition: str) -> tuple[int, float, str]:
    """Order activities as in ``ACTIVITIES`` and conditions by their number or speed class."""
    order = {"slow": 0.0, "normal": 1.0, "fast": 2.0}
    head = condition.split()[0]
    try:
        value = float(head)
    except ValueError:
        value = order.get(head, 9.0)
    return (ACTIVITIES.index(activity) if activity in ACTIVITIES else 99, value, condition)
