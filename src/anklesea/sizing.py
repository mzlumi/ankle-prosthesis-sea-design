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
