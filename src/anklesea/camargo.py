"""Read trials of the Camargo et al. (2021) lower-limb dataset.

Camargo, Ramanathan, Flanagan, Young, "A comprehensive, open-source dataset of lower
limb biomechanics in multiple conditions of stairs, ramps, and level-ground
ambulation and transitions", J. Biomech. 119 (2021) 110320,
doi:10.1016/j.jbiomech.2021.110320. CC BY 4.0.

The files are MATLAB tables, which only the ``mat-io`` package reads. Each trial has
one file per sensor folder (``id``, ``ik``, ``gcRight``, ``gcLeft``, ``conditions``)
under ``<subject>/<date>/<mode>/``. This module joins the folders a trial needs on the
``Header`` time column and converts to SI units: ankle angle from degrees to radians,
stair height from inches to metres, ramp incline from degrees to radians. The ankle
moment is already in N·m (not normalized by body mass).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import matio
import numpy as np
import pandas as pd

from anklesea import RAW_DIR

MODES = ("treadmill", "levelground", "ramp", "stair")
LEGS = ("r", "l")
SAMPLE_RATE_HZ = 200.0
INCH = 0.0254

# Ramp inclines and stair heights by the index in the trial name (Camargo et al. 2021).
RAMP_INCLINE_DEG = {1: 5.2, 2: 7.8, 3: 9.2, 4: 11.0, 5: 12.4, 6: 18.0}
STAIR_HEIGHT_IN = {1: 4.0, 2: 5.0, 3: 6.0, 4: 7.0}

# Trailing numbers: an optional repeat of the same block (``treadmill_04_2_01``) and the index.
_TAIL = r"(?:_(?P<repeat>\d+))?_(?P<index>\d+)$"
_NAME_PATTERNS = {
    "treadmill": re.compile(r"^treadmill_(?P<block>\d+)" + _TAIL),
    "levelground": re.compile(r"^levelground_(?P<turn>cw|ccw)_(?P<speed>slow|normal|fast)_(?P<block>\d+)" + _TAIL),
    "ramp": re.compile(r"^ramp_(?P<level>\d+)_(?P<leg>[lr])_(?P<block>\d+)" + _TAIL),
    "stair": re.compile(r"^stair_(?P<level>\d+)_(?P<leg>[lr])_(?P<block>\d+)" + _TAIL),
}


@dataclass(frozen=True)
class TrialName:
    """Parts of a trial file name such as ``stair_1_l_01_02``.

    ``condition`` is the speed class for level ground (``slow``, ``normal``, ``fast``),
    the incline or height index for ramps and stairs (``"1"`` to ``"6"``), and empty for
    the treadmill, whose speed changes within a trial. ``leg`` is the leg letter in the
    ramp and stair names (the leading leg of the transition), ``turn`` the level-ground
    circuit direction. ``block`` and ``index`` are the two trailing numbers, and
    ``repeat`` the optional number between them in repeated trials (1 when absent).
    """

    stem: str
    mode: str
    condition: str
    leg: str | None
    turn: str | None
    block: int
    index: int
    repeat: int = 1


def parse_trial_name(stem: str) -> TrialName:
    """Parse a trial file stem (without ``.mat``) into its parts.

    Some subjects' files are capitalized (``Ramp_1_L_01_04``); the parts are parsed in
    lower case and ``stem`` keeps the original spelling, which the file path needs.
    """
    stem = Path(stem).stem
    lower = stem.lower()
    mode = lower.split("_", 1)[0]
    pattern = _NAME_PATTERNS.get(mode)
    match = pattern.match(lower) if pattern else None
    if match is None:
        raise ValueError(f"not a Camargo trial name: {stem!r}")
    parts = match.groupdict()
    condition = parts.get("speed") or parts.get("level") or ""
    return TrialName(
        stem=stem,
        mode=mode,
        condition=condition,
        leg=parts.get("leg"),
        turn=parts.get("turn"),
        block=int(parts["block"]),
        index=int(parts["index"]),
        repeat=int(parts["repeat"]) if parts.get("repeat") else 1,
    )


@dataclass
class Trial:
    """One trial, aligned on a common 200 Hz time base, in SI units.

    Per-leg arrays are keyed by ``"r"`` and ``"l"``. ``angle`` is the ankle angle in rad
    (OpenSim ``ankle_angle``, dorsiflexion positive), ``moment`` the ankle moment from
    inverse dynamics in N·m about the same coordinate, ``heel_strike_phase`` and
    ``toe_off_phase`` the dataset's gait-cycle columns in percent. ``labels`` holds the
    locomotion label of each sample (``None`` for treadmill trials, which have none),
    ``belt_speed`` the treadmill speed in m/s at each sample (treadmill only).
    """

    subject: str
    date: str
    name: TrialName
    time: np.ndarray
    angle: dict[str, np.ndarray]
    moment: dict[str, np.ndarray]
    heel_strike_phase: dict[str, np.ndarray]
    toe_off_phase: dict[str, np.ndarray]
    labels: np.ndarray | None = None
    belt_speed: np.ndarray | None = None
    stair_height_m: float | None = None
    ramp_incline_rad: float | None = None
    meta: dict[str, object] = field(default_factory=dict)

    @property
    def mode(self) -> str:
        return self.name.mode

    def __len__(self) -> int:
        return len(self.time)


def _scalar(value: object) -> object:
    """Unwrap the nested 1x1 arrays that MATLAB scalars and strings become."""
    while isinstance(value, np.ndarray) and value.size == 1:
        value = value.reshape(-1)[0]
    if isinstance(value, np.generic):
        value = value.item()
    return value


def read_mat(path: Path) -> dict[str, object]:
    """Read a dataset ``.mat`` file with mat-io, dropping the MATLAB header keys."""
    data = matio.load_from_mat(str(path))
    return {k: v for k, v in data.items() if not k.startswith("__")}


def read_table(path: Path) -> pd.DataFrame:
    """Read a sensor file (``id``, ``ik``, ``gcRight``, ``gcLeft``) as a DataFrame."""
    data = read_mat(path)
    table = data.get("data")
    if not isinstance(table, pd.DataFrame):
        raise ValueError(f"{path} has no 'data' table")
    return table


def _time_key(header: pd.Series | np.ndarray) -> np.ndarray:
    """Integer milliseconds, so that floating-point time stamps join exactly."""
    return np.round(np.asarray(header, dtype=float) * 1000.0).astype(np.int64)


def _label_strings(column: pd.Series) -> np.ndarray:
    return np.array([str(_scalar(np.asarray(v, dtype=object))) for v in column], dtype=object)


def trial_path(raw_dir: Path, subject: str, date: str, mode: str, sensor: str, stem: str) -> Path:
    return raw_dir / subject / date / mode / sensor / f"{stem}.mat"


def load_trial(
    subject: str,
    date: str,
    stem: str,
    raw_dir: Path = RAW_DIR,
) -> Trial:
    """Load one trial: ankle angle and moment of both legs, gait cycle and conditions.

    Rows are kept where ``id``, ``ik``, ``gcRight`` and ``gcLeft`` share a ``Header``
    value (they do in every trial checked; the join guards against off-by-one ends).
    NaN in the moment (no force plate under the foot, typically at trial starts) is kept
    as NaN; stride cutting removes it later.
    """
    name = parse_trial_name(stem)
    mode = name.mode

    def path(sensor: str) -> Path:
        return trial_path(raw_dir, subject, date, mode, sensor, stem)

    ik = read_table(path("ik"))
    id_ = read_table(path("id"))
    gc = {"r": read_table(path("gcRight")), "l": read_table(path("gcLeft"))}

    frames = [
        pd.DataFrame(
            {"key": _time_key(ik["Header"]), "time": ik["Header"].to_numpy(float)}
            | {f"angle_{leg}": ik[f"ankle_angle_{leg}"].to_numpy(float) for leg in LEGS}
        ),
        pd.DataFrame(
            {"key": _time_key(id_["Header"])}
            | {f"moment_{leg}": id_[f"ankle_angle_{leg}_moment"].to_numpy(float) for leg in LEGS}
        ),
    ]
    for leg in LEGS:
        frames.append(
            pd.DataFrame(
                {
                    "key": _time_key(gc[leg]["Header"]),
                    f"hs_{leg}": gc[leg]["HeelStrike"].to_numpy(float),
                    f"to_{leg}": gc[leg]["ToeOff"].to_numpy(float),
                }
            )
        )
    table = frames[0]
    for frame in frames[1:]:
        table = table.merge(frame, on="key", how="inner")
    table = table.sort_values("key").reset_index(drop=True)
    if table.empty:
        raise ValueError(f"{subject}/{date}/{mode}/{stem}: sensor tables share no time stamps")

    conditions = read_mat(path("conditions")) if path("conditions").exists() else {}
    labels = None
    if isinstance(conditions.get("labels"), pd.DataFrame):
        lab = conditions["labels"]
        lookup = pd.Series(_label_strings(lab["Label"]), index=_time_key(lab["Header"]))
        lookup = lookup[~lookup.index.duplicated()]
        labels = lookup.reindex(table["key"].to_numpy()).to_numpy(dtype=object)
        labels = np.where(pd.isna(labels), "", labels).astype(object)

    belt_speed = None
    if isinstance(conditions.get("speed"), pd.DataFrame):
        sp = conditions["speed"]
        belt_speed = np.interp(table["time"].to_numpy(), sp["Header"].to_numpy(float), sp["Speed"].to_numpy(float))

    stair_height_m = None
    ramp_incline_rad = None
    if mode == "stair":
        height_in = conditions.get("stairHeight")
        height_in = float(_scalar(height_in)) if height_in is not None else STAIR_HEIGHT_IN.get(int(name.condition))
        stair_height_m = None if height_in is None else height_in * INCH
    if mode == "ramp":
        incline_deg = conditions.get("rampIncline")
        incline_deg = float(_scalar(incline_deg)) if incline_deg is not None else RAMP_INCLINE_DEG.get(int(name.condition))
        ramp_incline_rad = None if incline_deg is None else np.deg2rad(incline_deg)

    meta = {
        k: _scalar(v)
        for k, v in conditions.items()
        if not isinstance(v, pd.DataFrame) and np.asarray(v, dtype=object).size == 1
    }

    return Trial(
        subject=subject,
        date=date,
        name=name,
        time=table["time"].to_numpy(),
        angle={leg: np.deg2rad(table[f"angle_{leg}"].to_numpy()) for leg in LEGS},
        moment={leg: table[f"moment_{leg}"].to_numpy() for leg in LEGS},
        heel_strike_phase={leg: table[f"hs_{leg}"].to_numpy() for leg in LEGS},
        toe_off_phase={leg: table[f"to_{leg}"].to_numpy() for leg in LEGS},
        labels=labels,
        belt_speed=belt_speed,
        stair_height_m=stair_height_m,
        ramp_incline_rad=ramp_incline_rad,
        meta=meta,
    )


@dataclass(frozen=True)
class TrialRef:
    """Where a trial lives: enough to call :func:`load_trial`."""

    subject: str
    date: str
    mode: str
    stem: str


def list_trials(
    raw_dir: Path = RAW_DIR,
    subjects: list[str] | None = None,
    modes: tuple[str, ...] | list[str] = MODES,
    require: tuple[str, ...] = ("id", "ik", "gcRight", "gcLeft"),
) -> list[TrialRef]:
    """Trials under ``raw_dir`` that have a file in every folder of ``require``."""
    refs = []
    for subject_dir in sorted(p for p in raw_dir.glob("AB*") if p.is_dir()):
        if subjects is not None and subject_dir.name not in subjects:
            continue
        for date_dir in sorted(p for p in subject_dir.iterdir() if p.is_dir() and p.name != "osimxml"):
            for mode in modes:
                first = date_dir / mode / require[0]
                if not first.is_dir():
                    continue
                for path in sorted(first.glob("*.mat")):
                    if all((date_dir / mode / s / path.name).exists() for s in require[1:]):
                        refs.append(TrialRef(subject_dir.name, date_dir.name, mode, path.stem))
    return refs


@lru_cache(maxsize=4)
def _subject_info(path: Path) -> pd.DataFrame:
    table = read_mat(path)["data"]
    table = table.copy()
    table["Subject"] = [str(_scalar(np.asarray(s, dtype=object))) for s in table["Subject"]]
    return table.set_index("Subject")


def subject_mass(subject: str, raw_dir: Path = RAW_DIR) -> float:
    """Body mass in kg from ``SubjectInfo.mat``. Used for torque scaling only."""
    info = _subject_info(raw_dir / "SubjectInfo.mat")
    if subject not in info.index:
        raise KeyError(f"{subject} not in SubjectInfo.mat")
    return float(info.loc[subject, "Weight"])
