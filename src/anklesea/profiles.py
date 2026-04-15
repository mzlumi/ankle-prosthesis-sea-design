"""Averaged gait profiles as periodic loads for the actuator model.

The actuator model needs the joint angle and torque and their first and second time
derivatives over one stride. Differentiating a 101-point average twice amplifies the
noise, and a stride-averaged curve does not end exactly where it starts. Both problems
are handled by fitting a truncated Fourier series over the gait cycle: the fit is
periodic by construction (so the energy integrals over a stride see a closed cycle, as
the convex form of the energy assumes) and its derivatives are exact.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

DEFAULT_HARMONICS = 20


@dataclass(frozen=True)
class Load:
    """One stride of joint motion and torque in time, SI units.

    ``torque`` is the torque the actuator must apply to the joint, in the direction of
    the joint angle (OpenSim convention: dorsiflexion positive, so push-off is negative).
    """

    t: np.ndarray
    angle: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    torque: np.ndarray
    torque_rate: np.ndarray
    torque_accel: np.ndarray
    period: float

    @property
    def dt(self) -> float:
        return self.period / len(self.t)

    @property
    def power(self) -> np.ndarray:
        """Joint power in W (positive when the joint does work on the environment)."""
        return self.torque * self.velocity

    def scaled(self, torque_factor: float) -> Load:
        """Same motion with the torque multiplied by ``torque_factor``."""
        return Load(
            self.t,
            self.angle,
            self.velocity,
            self.acceleration,
            self.torque * torque_factor,
            self.torque_rate * torque_factor,
            self.torque_accel * torque_factor,
            self.period,
        )


def fourier_coefficients(samples: np.ndarray, n_harmonics: int, closed: bool = True) -> np.ndarray:
    """Complex Fourier coefficients of equally spaced samples over one period.

    With ``closed=True`` the samples cover 0 to 100 % of the cycle including both ends;
    the two end points are averaged so that a small mismatch between them does not
    create a step. With ``closed=False`` the last sample is the one before 100 %.
    """
    y = np.asarray(samples, dtype=float).copy()
    if closed:
        y[0] = 0.5 * (y[0] + y[-1])
        y = y[:-1]
    coeffs = np.fft.rfft(y) / len(y)
    return coeffs[: n_harmonics + 1]


def fourier_eval(coeffs: np.ndarray, s: np.ndarray, period: float, derivative: int = 0) -> np.ndarray:
    """Evaluate a Fourier series (or its time derivative) at cycle fractions ``s`` in [0, 1)."""
    h = np.arange(len(coeffs))
    omega = 2.0 * np.pi * h / period
    factor = (1j * omega) ** derivative
    weights = np.where(h == 0, 1.0, 2.0)
    phase = np.exp(2j * np.pi * np.outer(s, h))
    return np.real(phase @ (weights * factor * coeffs))


def load_from_cycle(
    angle: np.ndarray,
    torque: np.ndarray,
    period: float,
    n_samples: int = 1000,
    n_harmonics: int = DEFAULT_HARMONICS,
) -> Load:
    """Build a periodic :class:`Load` from angle and torque sampled over 0 to 100 % of a stride."""
    s = np.arange(n_samples) / n_samples
    ca = fourier_coefficients(angle, n_harmonics)
    ct = fourier_coefficients(torque, n_harmonics)
    return Load(
        t=s * period,
        angle=fourier_eval(ca, s, period),
        velocity=fourier_eval(ca, s, period, 1),
        acceleration=fourier_eval(ca, s, period, 2),
        torque=fourier_eval(ct, s, period),
        torque_rate=fourier_eval(ct, s, period, 1),
        torque_accel=fourier_eval(ct, s, period, 2),
        period=period,
    )


@dataclass(frozen=True)
class GaitProfile:
    """Mean ankle angle and moment per kg over the gait cycle, for one activity and condition."""

    activity: str
    condition: str
    gait_pct: np.ndarray
    angle: np.ndarray  # rad
    moment_per_kg: np.ndarray  # N·m/kg
    stride_time: float  # s
    n_subjects: int = 0
    n_strides: int = 0

    @property
    def key(self) -> tuple[str, str]:
        return (self.activity, self.condition)

    @property
    def label(self) -> str:
        return f"{self.activity} {self.condition}"

    def load(self, mass: float, n_samples: int = 1000, n_harmonics: int = DEFAULT_HARMONICS) -> Load:
        """The load for a user of body mass ``mass`` in kg."""
        return load_from_cycle(self.angle, self.moment_per_kg * mass, self.stride_time, n_samples, n_harmonics)


def read_profiles(path: Path) -> dict[tuple[str, str], GaitProfile]:
    """Read the aggregate CSV written by ``scripts/gait_profiles.py``."""
    frame = pd.read_csv(path, comment="#")
    out = {}
    for (activity, condition), g in frame.groupby(["activity", "condition"], sort=False):
        g = g.sort_values("gait_pct")
        out[(activity, condition)] = GaitProfile(
            activity=activity,
            condition=condition,
            gait_pct=g["gait_pct"].to_numpy(),
            angle=g["angle_mean_rad"].to_numpy(),
            moment_per_kg=g["moment_mean_Nm_per_kg"].to_numpy(),
            stride_time=float(g["stride_time_mean_s"].iloc[0]),
            n_subjects=int(g["n_subjects"].iloc[0]),
            n_strides=int(g["n_strides"].iloc[0]),
        )
    return out
