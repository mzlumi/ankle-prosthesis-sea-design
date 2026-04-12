"""Write small synthetic trials in the same file layout as the Camargo et al. (2021) data."""

from __future__ import annotations

from pathlib import Path

import matio
import numpy as np
import pandas as pd

FS = 200.0


def gait_phase(time: np.ndarray, stride_time: float, offset: float = 0.0) -> np.ndarray:
    """Phase in percent that restarts at every heel strike, like the dataset's gc columns."""
    return 100.0 * np.mod(time - offset, stride_time) / stride_time


def synthetic_ankle(phase_pct: np.ndarray, mass: float = 70.0) -> tuple[np.ndarray, np.ndarray]:
    """A smooth ankle angle (deg) and moment (N·m) shaped roughly like level walking.

    Dorsiflexion and dorsiflexion moments are positive (OpenSim convention), so the
    push-off moment is negative.
    """
    s = phase_pct / 100.0
    angle_deg = 5.0 * np.sin(2 * np.pi * s) - 10.0 * np.exp(-(((s - 0.62) / 0.06) ** 2))
    stance = np.clip(np.sin(np.pi * s / 0.62), 0.0, None) * (s < 0.62)
    moment = -1.5 * mass * stance**2 * (0.3 + 0.7 * s / 0.62)
    return angle_deg, moment


def write_table(path: Path, table: pd.DataFrame) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    matio.save_to_mat(str(path), {"data": table}, version="v7")


def write_trial(
    raw_dir: Path,
    subject: str,
    date: str,
    stem: str,
    duration: float = 6.0,
    stride_time: float = 1.1,
    mass: float = 70.0,
    nan_start: float = 0.0,
    t0: float = 10.0,
    labels: list[str] | None = None,
    conditions: dict[str, object] | None = None,
) -> dict[str, np.ndarray]:
    """Write ``id``, ``ik``, ``gcRight``, ``gcLeft`` and ``conditions`` for one trial.

    Returns the arrays written, so tests can compare against them.
    """
    mode = stem.split("_", 1)[0]
    base = raw_dir / subject / date / mode
    n = int(round(duration * FS))
    time = np.round(t0 + np.arange(n) / FS, 3)
    phase_r = gait_phase(time, stride_time, offset=t0 + 0.2)
    phase_l = gait_phase(time, stride_time, offset=t0 + 0.2 + stride_time / 2)
    angle_r, moment_r = synthetic_ankle(phase_r, mass)
    angle_l, moment_l = synthetic_ankle(phase_l, mass)
    moment_r = moment_r.copy()
    moment_r[time < t0 + nan_start] = np.nan

    write_table(base / "ik" / f"{stem}.mat", pd.DataFrame({"Header": time, "ankle_angle_r": angle_r, "ankle_angle_l": angle_l}))
    write_table(
        base / "id" / f"{stem}.mat",
        pd.DataFrame({"Header": time, "ankle_angle_r_moment": moment_r, "ankle_angle_l_moment": moment_l}),
    )
    to_r = np.mod(phase_r - 62.0, 100.0)
    to_l = np.mod(phase_l - 62.0, 100.0)
    write_table(base / "gcRight" / f"{stem}.mat", pd.DataFrame({"Header": time, "HeelStrike": phase_r, "ToeOff": to_r}))
    write_table(base / "gcLeft" / f"{stem}.mat", pd.DataFrame({"Header": time, "HeelStrike": phase_l, "ToeOff": to_l}))

    cond: dict[str, object] = {"trialStarts": np.array([[time[0]]]), "trialEnds": np.array([[time[-1]]])}
    if labels is not None:
        cond["labels"] = pd.DataFrame({"Header": time, "Label": [labels[i % len(labels)] for i in range(n)]})
    if conditions:
        cond.update(conditions)
    path = base / "conditions" / f"{stem}.mat"
    path.parent.mkdir(parents=True, exist_ok=True)
    matio.save_to_mat(str(path), cond, version="v7")
    return {
        "time": time,
        "angle_r_deg": angle_r,
        "moment_r": moment_r,
        "phase_r": phase_r,
        "angle_l_deg": angle_l,
        "moment_l": moment_l,
    }


def write_subject_info(raw_dir: Path, masses: dict[str, float]) -> None:
    raw_dir.mkdir(parents=True, exist_ok=True)
    table = pd.DataFrame(
        {
            "Subject": list(masses),
            "Age": [30.0] * len(masses),
            "Gender": ["M"] * len(masses),
            "Height": [1.75] * len(masses),
            "Weight": list(masses.values()),
        }
    )
    matio.save_to_mat(str(raw_dir / "SubjectInfo.mat"), {"data": table}, version="v7")
