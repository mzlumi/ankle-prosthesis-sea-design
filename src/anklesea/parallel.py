"""A linear spring in parallel with the series elastic actuator.

The parallel spring acts on the joint directly, ``tau_p = -k_p (theta - theta_0)``, so the
actuator (series spring, transmission and motor) supplies only the rest of the joint
torque:

``tau_a = tau - tau_p = tau + k_p theta - tau_0``, with ``tau_0 = k_p theta_0``.

Why add one: a series spring changes the motor's *path* but not the torque it must
deliver, so it cannot reduce the copper loss of the load torque, which sets the RMS
current and so the motor's heating. A parallel spring takes over part of the torque
itself. During stance the ankle moment rises roughly in proportion to dorsiflexion, so a
spring with the right stiffness and rest angle carries much of it (Notre Dame AME 60553
lecture notes L12 and L13). The price is that the spring also acts in swing, when the
joint torque is near zero, and the motor must then hold it.

For a fixed series compliance ``alpha`` and ratio ``N``, the motor torque and speed are
affine in ``x = (k_p, tau_0)``, so the mean squared motor torque and the energy per stride
are both convex quadratics in ``x`` (``docs/derivation.md``, section 9) and their
minimizers are 2-by-2 linear solves. :func:`optimize_parallel` alternates that step with
the closed-form series compliance of :mod:`anklesea.sizing` at each ratio.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from anklesea.motor import Motor
from anklesea.profiles import Load
from anklesea.sea import Limits, evaluate, motor_trajectory
from anklesea.sizing import constrained_optimum


@dataclass(frozen=True)
class ParallelSpring:
    """``tau_p = -stiffness * theta + preload`` (N·m/rad and N·m); rest angle ``preload / stiffness``."""

    stiffness: float = 0.0
    preload: float = 0.0

    @property
    def rest_angle(self) -> float:
        return np.nan if self.stiffness == 0.0 else self.preload / self.stiffness

    def torque(self, angle: np.ndarray) -> np.ndarray:
        return -self.stiffness * np.asarray(angle) + self.preload


def actuator_load(load: Load, spring: ParallelSpring) -> Load:
    """The load left for the actuator when ``spring`` acts in parallel."""
    k = spring.stiffness
    return replace(
        load,
        torque=load.torque + k * load.angle - spring.preload,
        torque_rate=load.torque_rate + k * load.velocity,
        torque_accel=load.torque_accel + k * load.acceleration,
    )


def joint_level_spring(load: Load) -> ParallelSpring:
    """Spring that minimizes the RMS actuator torque at the joint (least squares of ``-tau`` on ``theta``).

    Ignores the motor: a first guess and a check on the motor-level result.
    """
    a = np.column_stack([-load.angle, np.ones_like(load.angle)])
    (k, tau0), *_ = np.linalg.lstsq(a, load.torque, rcond=None)
    return ParallelSpring(float(k), float(tau0))


def _affine_in_spring(load: Load, compliance: float, ratio: float, motor: Motor) -> dict[str, np.ndarray]:
    """Motor torque and speed as ``u0 + U x`` and ``v0 + V x`` with ``x = (k_p, tau_0)``.

    Exact because both are affine in ``x``: evaluated at ``x = 0`` and the two unit vectors.
    """
    base = motor_trajectory(load, compliance, ratio, motor)
    cols_t, cols_w = [], []
    for spring in (ParallelSpring(1.0, 0.0), ParallelSpring(0.0, 1.0)):
        traj = motor_trajectory(actuator_load(load, spring), compliance, ratio, motor)
        cols_t.append(traj["motor_torque"] - base["motor_torque"])
        cols_w.append(traj["motor_speed"] - base["motor_speed"])
    return {"u0": base["motor_torque"], "U": np.column_stack(cols_t), "v0": base["motor_speed"],
            "V": np.column_stack(cols_w)}


def best_spring(load: Load, compliance: float, ratio: float, motor: Motor, objective: str = "rms") -> ParallelSpring:
    """Parallel spring minimizing the RMS motor current (``"rms"``) or the energy (``"energy"``).

    Both objectives are ``x' H x + 2 g' x + const`` with ``H`` positive semidefinite, so the
    minimizer solves ``H x = -g``.
    """
    co = _affine_in_spring(load, compliance, ratio, motor)
    u0, u, v0, v = co["u0"], co["U"], co["v0"], co["V"]
    if objective == "rms":
        h = u.T @ u
        g = u.T @ u0
    elif objective == "energy":
        r = motor.resistance / motor.torque_constant**2
        h = r * (u.T @ u) + 0.5 * (u.T @ v + v.T @ u)
        g = r * (u.T @ u0) + 0.5 * (u.T @ v0 + v.T @ u0)
    else:
        raise ValueError(f"unknown objective {objective!r}")
    x = np.linalg.solve(h, -g)
    return ParallelSpring(float(x[0]), float(x[1]))


def optimize_parallel(
    load: Load,
    motor: Motor,
    limits: Limits,
    objective: str = "rms",
    thermal: bool = True,
    ratios: np.ndarray | None = None,
    iterations: int = 8,
) -> dict:
    """Series stiffness, ratio and parallel spring with the least energy within the limits.

    At each ratio, alternate (1) the parallel spring that minimizes ``objective`` for the
    current series compliance (unconstrained, closed form) and (2) the least-energy series
    compliance for the resulting actuator load, clipped to the interval that meets the
    limits (closed form). The RMS current limit is included when ``thermal`` is True.
    Returns the full model's metrics plus ``stiffness``, ``ratio`` and ``spring``; ``ratio``
    is NaN if no ratio gives a design within the limits.
    """
    ratios = np.geomspace(30.0, 2000.0, 300) if ratios is None else np.asarray(ratios, dtype=float)
    best: dict | None = None
    for n in ratios:
        spring = joint_level_spring(load)
        alpha, energy = constrained_optimum(actuator_load(load, spring), n, motor, limits, thermal)
        alpha = 0.0 if np.isnan(alpha) else alpha
        for _ in range(iterations):
            spring = best_spring(load, alpha, n, motor, objective)
            new_alpha, energy = constrained_optimum(actuator_load(load, spring), n, motor, limits, thermal)
            if np.isnan(new_alpha):
                break
            converged = abs(new_alpha - alpha) <= 1e-9 + 1e-6 * alpha
            alpha = new_alpha
            if converged:
                break
        if not np.isfinite(energy):
            continue
        if best is None or energy < best["energy"]:
            best = {"energy": energy, "compliance": alpha, "ratio": float(n), "spring": spring}
    if best is None:
        return {"stiffness": np.nan, "ratio": np.nan, "spring": None, "feasible": False, "drive_feasible": False}
    metrics = evaluate(actuator_load(load, best["spring"]), best["compliance"], best["ratio"], motor, limits)
    out = {key: value.item() for key, value in metrics.items()}
    out.update(stiffness=np.inf if best["compliance"] == 0.0 else 1.0 / best["compliance"], ratio=best["ratio"],
               compliance=best["compliance"], spring=best["spring"])
    return out
