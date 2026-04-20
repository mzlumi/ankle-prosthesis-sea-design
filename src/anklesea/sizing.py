"""Choose the spring stiffness k and gear ratio N that minimize motor energy per stride.

Two ways to find the optimum:

1. :func:`sweep` evaluates the full energy model (:mod:`anklesea.sea`) on a grid of
   ``(k, N)``, masks the designs that break a voltage, current or speed limit, and picks
   the feasible design with the least energy. Either all limits or only the
   instantaneous drive limits can be imposed (see :mod:`anklesea.sea`).
2. :func:`quadratic_in_compliance` uses the structure of the problem. For a fixed ratio
   ``N`` the motor torque and speed are both affine in the spring compliance
   ``alpha = 1/k``:

   ``tau_m = a1 alpha + b1``, ``omega_m = a2 alpha + b2``

   so the energy over a periodic stride, the integral of ``R tau_m^2 / k_t^2 + tau_m omega_m``,
   is a quadratic ``E(alpha) = a alpha^2 + b alpha + c`` with ``a >= 0``: convex
   (Notre Dame AME 60553 lecture notes L12 and L13; E. A. Bolivar-Nieto, G. C. Thomas,
   E. Rouse, R. D. Gregg, "Convex optimization for spring design in series elastic
   actuators: from theory to practice", IROS 2021). Its minimum over ``alpha >= 0`` is
   at ``alpha* = -b / (2a)`` when ``b < 0`` and at the rigid actuator (``alpha = 0``)
   otherwise. ``c`` is the energy of the rigid actuator, and every compliance between 0
   and ``-b / a`` uses less energy than the rigid actuator (the "energy savings
   region"). The derivation is written out in ``docs/derivation.md``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from anklesea.motor import Motor
from anklesea.profiles import Load
from anklesea.sea import Limits, evaluate

DESIGN_MASS_KG = 80.0  # docs/assumptions.md: ISO 10328 loading level P4
BUS_VOLTAGE = 36.0  # docs/assumptions.md: nominal voltage of the chosen winding
LEVEL_WALK = ("treadmill", "1.20 m/s")

# Share of steps per activity in free-living use of a transtibial prosthesis (Srisuwan and Klute,
# Prosthet. Orthot. Int. 45(3):191-197, 2021, Table 3); turning steps are counted as level walking.
DAILY_MIX = {"level": 0.828 + 0.090, "rampascent": 0.016, "rampdescent": 0.020, "stairascent": 0.023,
             "stairdescent": 0.025}
STEPS_PER_DAY = 4422  # same source, mean over 1 to 2 weekdays


@dataclass
class SweepResult:
    """Grid results with shape ``(len(ratios), len(stiffness))``; the last stiffness may be inf."""

    stiffness: np.ndarray
    ratios: np.ndarray
    metrics: dict[str, np.ndarray]

    @property
    def energy(self) -> np.ndarray:
        return self.metrics["energy"]

    @property
    def feasible(self) -> np.ndarray:
        return self.metrics["feasible"]

    def mask(self, constraint: str | None) -> np.ndarray:
        """Designs allowed under ``constraint``: a boolean metric name such as ``"feasible"``
        or ``"drive_feasible"``, or ``None`` for every design."""
        if constraint is None:
            return np.ones(self.energy.shape, dtype=bool)
        return self.metrics[constraint]

    def optimum(self, key: str = "energy", constraint: str | None = "feasible") -> dict[str, float]:
        """The allowed design with the least ``key``; ``feasible`` False and NaN design if none."""
        values = np.where(self.mask(constraint), self.metrics[key], np.inf)
        if not np.isfinite(values).any():
            return {"stiffness": np.nan, "ratio": np.nan, "feasible": False}
        i, j = np.unravel_index(np.argmin(values), values.shape)
        out = {name: arr[i, j].item() for name, arr in self.metrics.items()}
        out.update(stiffness=float(self.stiffness[j]), ratio=float(self.ratios[i]), i=int(i), j=int(j))
        return out

    def best_per_ratio(self, constraint: str | None = None) -> np.ndarray:
        """Index of the least-energy stiffness for each ratio (-1 where none qualifies)."""
        values = np.where(self.mask(constraint), self.energy, np.inf)
        idx = np.argmin(values, axis=1)
        idx[~np.isfinite(values).any(axis=1)] = -1
        return idx


def default_grid(n_stiffness: int = 121, n_ratio: int = 121) -> tuple[np.ndarray, np.ndarray]:
    """Log-spaced stiffness from 50 to 20000 N·m/rad (plus a rigid column) and ratios 30 to 2000."""
    stiffness = np.concatenate([np.geomspace(50.0, 20000.0, n_stiffness), [np.inf]])
    ratios = np.geomspace(30.0, 2000.0, n_ratio)
    return stiffness, ratios


def sweep(
    load: Load,
    motor: Motor,
    limits: Limits,
    stiffness: np.ndarray | None = None,
    ratios: np.ndarray | None = None,
    efficiency_model: str = "constant",
) -> SweepResult:
    """Evaluate every ``(N, k)`` pair on the grid, one ratio at a time to bound memory."""
    if stiffness is None or ratios is None:
        default_k, default_n = default_grid()
        stiffness = default_k if stiffness is None else stiffness
        ratios = default_n if ratios is None else ratios
    stiffness = np.asarray(stiffness, dtype=float)
    ratios = np.asarray(ratios, dtype=float)
    compliance = np.where(np.isinf(stiffness), 0.0, 1.0 / stiffness)
    rows = [evaluate(load, compliance, np.full_like(compliance, n), motor, limits, efficiency_model) for n in ratios]
    metrics = {key: np.stack([row[key] for row in rows]) for key in rows[0]}
    return SweepResult(stiffness=stiffness, ratios=ratios, metrics=metrics)


@dataclass(frozen=True)
class Quadratic:
    """``E(alpha) = a alpha^2 + b alpha + c`` for one gear ratio (J, alpha in rad/(N·m))."""

    a: float
    b: float
    c: float
    ratio: float

    def energy(self, compliance: np.ndarray | float) -> np.ndarray:
        alpha = np.asarray(compliance, dtype=float)
        return self.a * alpha**2 + self.b * alpha + self.c

    @property
    def optimal_compliance(self) -> float:
        """Minimizer over alpha >= 0."""
        if self.b >= 0.0 or self.a <= 0.0:
            return 0.0
        return -self.b / (2.0 * self.a)

    @property
    def optimal_stiffness(self) -> float:
        alpha = self.optimal_compliance
        return np.inf if alpha == 0.0 else 1.0 / alpha

    @property
    def optimal_energy(self) -> float:
        return float(self.energy(self.optimal_compliance))

    @property
    def savings_region(self) -> tuple[float, float]:
        """Compliances that use less energy than the rigid actuator, [0, -b/a]."""
        return (0.0, max(0.0, -self.b / self.a)) if self.a > 0 else (0.0, 0.0)


def affine_coefficients(load: Load, ratio: float, motor: Motor) -> dict[str, np.ndarray]:
    """Time series a1, b1, a2, b2 with tau_m = a1 alpha + b1 and omega_m = a2 alpha + b2."""
    n, j, bm = ratio, motor.inertia, motor.viscous_friction
    return {
        "a1": n * (j * load.torque_accel + bm * load.torque_rate),
        "b1": n * (j * load.acceleration + bm * load.velocity) + load.torque / (n * motor.gear_efficiency),
        "a2": n * load.torque_rate,
        "b2": n * load.velocity,
    }


def quadratic_in_compliance(load: Load, ratio: float, motor: Motor) -> Quadratic:
    """Coefficients of the energy per stride (with regeneration) as a function of compliance.

    The inductive term ``L i di/dt`` integrates to zero over a periodic stride and is left out.
    """
    co = affine_coefficients(load, ratio, motor)
    r_k2 = motor.resistance / motor.torque_constant**2
    dt = load.dt
    a1, b1, a2, b2 = co["a1"], co["b1"], co["a2"], co["b2"]
    a = np.sum(r_k2 * a1**2 + a1 * a2) * dt
    b = np.sum(2.0 * r_k2 * a1 * b1 + a1 * b2 + a2 * b1) * dt
    c = np.sum(r_k2 * b1**2 + b1 * b2) * dt
    return Quadratic(float(a), float(b), float(c), float(ratio))


def _affine_interval(slope: np.ndarray, offset: np.ndarray, bound: float) -> tuple[float, float]:
    """Compliances ``alpha >= 0`` with ``|slope alpha + offset| <= bound`` at every sample."""
    lo, hi = 0.0, np.inf
    flat = slope == 0.0
    if np.any(np.abs(offset[flat]) > bound):
        return (np.nan, np.nan)
    s, o = slope[~flat], offset[~flat]
    ends = np.stack([(-bound - o) / s, (bound - o) / s])
    lo = max(lo, float(ends.min(axis=0).max(initial=-np.inf)))
    hi = min(hi, float(ends.max(axis=0).min(initial=np.inf)))
    return (lo, hi) if lo <= hi else (np.nan, np.nan)


def _quadratic_interval(a: float, b: float, c: float) -> tuple[float, float]:
    """Compliances ``alpha >= 0`` with ``a alpha^2 + b alpha + c <= 0`` for convex ``a >= 0``."""
    if a <= 0.0:
        if b == 0.0:
            return (0.0, np.inf) if c <= 0.0 else (np.nan, np.nan)
        root = -c / b
        return (max(0.0, root), np.inf) if b < 0 else ((0.0, root) if root >= 0 else (np.nan, np.nan))
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return (np.nan, np.nan)
    r1, r2 = (-b - np.sqrt(disc)) / (2.0 * a), (-b + np.sqrt(disc)) / (2.0 * a)
    lo, hi = max(0.0, r1), r2
    return (lo, hi) if lo <= hi else (np.nan, np.nan)


def feasible_compliance(
    load: Load, ratio: float, motor: Motor, limits: Limits, thermal: bool = False
) -> tuple[float, float]:
    """Interval of compliances that meet the limits at this ratio, ``(nan, nan)`` if none.

    Current, motor speed and voltage are all affine in the compliance at every instant,
    so each instantaneous limit ``|x(t)| <= bound`` cuts the compliance axis to an interval,
    and their intersection is an interval too. The RMS current is the square root of a
    convex quadratic, so ``thermal=True`` intersects one more interval.
    """
    from anklesea.sea import _periodic_derivative

    co = affine_coefficients(load, ratio, motor)
    kt, r, ind = motor.torque_constant, motor.resistance, motor.inductance
    a_i, b_i = co["a1"] / kt, co["b1"] / kt
    a_v = r * a_i + ind * _periodic_derivative(a_i, load.dt) + kt * co["a2"]
    b_v = r * b_i + ind * _periodic_derivative(b_i, load.dt) + kt * co["b2"]
    intervals = [
        _affine_interval(a_i, b_i, limits.peak_current),
        _affine_interval(co["a2"], co["b2"], limits.max_speed),
        _affine_interval(a_v, b_v, limits.available_voltage),
    ]
    if thermal:
        intervals.append(_quadratic_interval(
            float(np.mean(a_i**2)), float(2.0 * np.mean(a_i * b_i)), float(np.mean(b_i**2)) - limits.rms_current**2
        ))
    bounds = np.array(intervals)
    if np.isnan(bounds).any():
        return (np.nan, np.nan)
    lo, hi = float(bounds[:, 0].max()), float(bounds[:, 1].min())
    return (lo, hi) if lo <= hi else (np.nan, np.nan)


def constrained_optimum(
    load: Load, ratio: float, motor: Motor, limits: Limits, thermal: bool = False
) -> tuple[float, float]:
    """Least-energy compliance at this ratio within the limits, and its energy (nan if none).

    A convex function of one variable restricted to an interval is minimized by clipping
    its unconstrained minimizer into the interval.
    """
    lo, hi = feasible_compliance(load, ratio, motor, limits, thermal)
    if np.isnan(lo):
        return (np.nan, np.nan)
    quad = quadratic_in_compliance(load, ratio, motor)
    alpha = float(np.clip(quad.optimal_compliance, lo, hi))
    return alpha, float(quad.energy(alpha))


def optimize(
    load: Load,
    motor: Motor,
    limits: Limits,
    ratios: np.ndarray | None = None,
    thermal: bool = False,
) -> dict[str, float]:
    """Least-energy design within the limits: a scan over the ratio with the clipped closed form.

    Returns the full model's metrics at the optimum (see :func:`anklesea.sea.evaluate`) plus
    ``stiffness``, ``ratio`` and ``compliance``; ``ratio`` is NaN and ``feasible`` False when
    no design meets the limits. ``drive_feasible`` limits are always imposed, the RMS current
    only with ``thermal=True``.
    """
    ratios = np.geomspace(30.0, 2000.0, 600) if ratios is None else np.asarray(ratios, dtype=float)
    best = (np.inf, np.nan, np.nan)
    for n in ratios:
        alpha, energy = constrained_optimum(load, n, motor, limits, thermal)
        if np.isfinite(energy) and energy < best[0]:
            best = (energy, alpha, n)
    _, alpha, n = best
    if np.isnan(n):
        return {"stiffness": np.nan, "ratio": np.nan, "compliance": np.nan, "feasible": False,
                "drive_feasible": False}
    metrics = evaluate(load, alpha, n, motor, limits)
    out = {key: value.item() for key, value in metrics.items()}
    out.update(stiffness=np.inf if alpha == 0.0 else 1.0 / alpha, ratio=float(n), compliance=float(alpha))
    return out


def optimize_mix(
    loads: list[Load],
    weights: list[float],
    motor: Motor,
    limits: Limits,
    ratios: np.ndarray | None = None,
    thermal: bool = False,
) -> dict[str, float]:
    """One design for several activities: least weighted energy, within the limits in every activity.

    The objective ``sum_m w_m E_m(alpha)`` is a sum of convex quadratics in compliance, so it is
    a convex quadratic itself, and the compliances allowed in every activity are the
    intersection of the per-activity intervals. At each ratio the clipped closed form gives
    the optimum, as in :func:`constrained_optimum`. Returns ``stiffness``, ``ratio``,
    ``compliance`` and ``weighted_energy`` (J per stride, weights normalized to sum to 1);
    ``ratio`` is NaN if no design meets the limits in every activity.
    """
    ratios = np.geomspace(30.0, 2000.0, 600) if ratios is None else np.asarray(ratios, dtype=float)
    w = np.asarray(weights, dtype=float) / float(np.sum(weights))
    best = (np.inf, np.nan, np.nan)
    for n in ratios:
        intervals = np.array([feasible_compliance(load, n, motor, limits, thermal) for load in loads])
        if np.isnan(intervals).any():
            continue
        lo, hi = float(intervals[:, 0].max()), float(intervals[:, 1].min())
        if lo > hi:
            continue
        quads = [quadratic_in_compliance(load, n, motor) for load in loads]
        combined = Quadratic(
            a=float(sum(wi * q.a for wi, q in zip(w, quads))),
            b=float(sum(wi * q.b for wi, q in zip(w, quads))),
            c=float(sum(wi * q.c for wi, q in zip(w, quads))),
            ratio=float(n),
        )
        alpha = float(np.clip(combined.optimal_compliance, lo, hi))
        energy = float(combined.energy(alpha))
        if energy < best[0]:
            best = (energy, alpha, n)
    energy, alpha, n = best
    if np.isnan(n):
        return {"stiffness": np.nan, "ratio": np.nan, "compliance": np.nan, "weighted_energy": np.nan}
    return {"stiffness": np.inf if alpha == 0.0 else 1.0 / alpha, "ratio": float(n), "compliance": alpha,
            "weighted_energy": energy}
