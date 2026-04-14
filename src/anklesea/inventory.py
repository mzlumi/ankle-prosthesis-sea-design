"""Inventory of the downloaded Camargo et al. (2021) trials: coverage, gaps and sign convention."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from anklesea import RAW_DIR, camargo


@dataclass
class TrialSummary:
    subject: str
    mode: str
    stem: str
    condition: str
    stair_height_in: float | None
    ramp_incline_deg: float | None
    duration_s: float
    n_samples: int
    moment_nan_r: int
    moment_nan_l: int
    moment_nan_runs_r: int
    angle_nan_r: int
    labels: tuple[str, ...]
    belt_speeds: tuple[float, ...]
    push_off: dict[str, tuple[float, float, float, int]]


def nan_runs(x: np.ndarray) -> int:
    """Number of separate runs of NaN in ``x``."""
    isnan = np.isnan(x).astype(int)
    return int(np.sum(np.diff(np.concatenate([[0], isnan])) == 1))


STEADY = ("walk", "rampascent", "rampdescent", "stairascent", "stairdescent")


def push_off_windows(trial: camargo.Trial, leg: str = "r", lo: float = 45.0, hi: float = 60.0) -> dict[str, np.ndarray]:
    """Samples of late stance (by the dataset's heel-strike phase), per steady activity.

    For labelled trials only samples with a steady locomotion label are kept, so that
    standing pauses (where the phase keeps creeping up) do not count. Treadmill samples
    are keyed ``treadmill`` and need a belt speed above 0.4 m/s.
    """
    phase = trial.heel_strike_phase[leg]
    late = (phase >= lo) & (phase <= hi) & ~np.isnan(trial.moment[leg])
    if trial.labels is None:
        speed_ok = trial.belt_speed > 0.4 if trial.belt_speed is not None else np.ones_like(late)
        return {"treadmill": late & speed_ok}
    return {label: late & (trial.labels == label) for label in STEADY if np.any(late & (trial.labels == label))}


def summarize_trial(trial: camargo.Trial) -> TrialSummary:
    leg = "r"
    dt = 1.0 / camargo.SAMPLE_RATE_HZ
    velocity = np.gradient(trial.angle[leg], dt)
    push_off = {}
    for label, window in push_off_windows(trial, leg).items():
        moment, vel = trial.moment[leg][window], velocity[window]
        if window.any():
            push_off[label] = (float(moment.mean()), float(vel.mean()), float((moment * vel).mean()), int(window.sum()))
    labels = tuple(sorted(set(trial.labels))) if trial.labels is not None else ()
    speeds = ()
    if trial.belt_speed is not None:
        rounded = np.round(trial.belt_speed / 0.05) * 0.05
        values, counts = np.unique(np.round(rounded, 2), return_counts=True)
        speeds = tuple(float(v) for v, c in zip(values, counts) if v > 0 and c > 10 * camargo.SAMPLE_RATE_HZ)
    return TrialSummary(
        subject=trial.subject,
        mode=trial.mode,
        stem=trial.name.stem,
        condition=trial.name.condition,
        stair_height_in=None if trial.stair_height_m is None else round(trial.stair_height_m / camargo.INCH, 2),
        ramp_incline_deg=None if trial.ramp_incline_rad is None else round(float(np.rad2deg(trial.ramp_incline_rad)), 2),
        duration_s=float(trial.time[-1] - trial.time[0]),
        n_samples=len(trial),
        moment_nan_r=int(np.isnan(trial.moment["r"]).sum()),
        moment_nan_l=int(np.isnan(trial.moment["l"]).sum()),
        moment_nan_runs_r=nan_runs(trial.moment["r"]),
        angle_nan_r=int(np.isnan(trial.angle["r"]).sum()),
        labels=labels,
        belt_speeds=speeds,
        push_off=push_off,
    )


def build_inventory(raw_dir: Path = RAW_DIR, subjects: list[str] | None = None) -> pd.DataFrame:
    """Load every complete trial and summarize it, one row per trial."""
    rows = []
    for ref in camargo.list_trials(raw_dir, subjects=subjects):
        trial = camargo.load_trial(ref.subject, ref.date, ref.stem, raw_dir=raw_dir)
        rows.append(summarize_trial(trial).__dict__)
    return pd.DataFrame(rows)
