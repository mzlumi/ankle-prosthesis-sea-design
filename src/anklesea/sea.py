"""Electrical energy of a series elastic actuator (SEA) driving the ankle through one stride.

The SEA is a motor, a transmission of ratio ``N`` and efficiency ``eta``, and a spring of
stiffness ``k`` between the transmission output and the joint. The spring transmits
the whole joint torque, so for a required joint angle ``theta(t)`` and torque
``tau(t)`` everything on the motor side follows without solving any dynamics
(Notre Dame AME 60553 lecture notes L12; Bolivar-Nieto et al., IROS 2021):

* spring torque ``tau = k (theta_m / N - theta)``, so the spring deflection is
  ``tau / k`` and the motor angle is ``theta_m = N (theta + tau / k)``;
* motor speed ``omega_m = N (theta_dot + tau_dot / k)`` and acceleration
  ``N (theta_ddot + tau_ddot / k)``;
* motor torque ``tau_m = J omega_m_dot + b omega_m + tau / (N eta)``: the reflected joint
  torque plus what accelerates the rotor and gearhead inertia ``J`` and overcomes
  viscous friction ``b``;
* current ``i = tau_m / k_t``, voltage ``v = R i + L di/dt + k_t omega_m``;
* electrical power ``P = v i = R i^2 + tau_m omega_m + L i di/dt``.

Why a spring can lower the energy: over a periodic stride the energy the joint needs,
the integral of ``tau theta_dot``, is fixed by the task, and the spring cannot change it
(it returns what it stores). What the spring changes is the motor's *path*: its speed
becomes ``N (theta_dot + tau_dot / k)`` instead of ``N theta_dot``. The motor torque
that drives the load itself, ``tau / (N eta)``, and its copper loss are the same for
every spring. What the spring changes is the torque that accelerates the rotor,
``J N (theta_ddot + tau_ddot / k)``, and the friction torque. An ankle needs a high ratio,
so the rotor inertia seen at the joint, ``J N^2``, is large (about 2 kg m^2 at N = 750,
some hundred times the foot's), and a rigid actuator spends a large share of its current
swinging its own rotor back and forth. A spring for which ``tau_dot / k`` cancels part
of ``theta_dot`` during push-off lets the rotor turn more slowly and evenly. Three
consequences: with a massless, frictionless motor and ideal regeneration the spring
would save nothing (checked in ``tests/test_sizing.py``); without regeneration it also
saves the negative work it stores, which a rigid motor would have to absorb; and it
lowers the peak motor speed, so the same ratio needs less voltage.

The transmission efficiency is applied as a constant divisor, ``tau / (N eta)``, in
both directions of power flow. This keeps the energy a quadratic function of the
spring compliance (see ``docs/derivation.md``) and is conservative when the joint
backdrives the motor (a real gear would then pass ``eta tau / N``). The
``efficiency_model="directional"`` option uses the direction-dependent form for
comparison.

Energy "with regeneration" integrates ``P`` (negative power is returned to the battery
without loss); "without regeneration" integrates ``max(P, 0)`` (negative power is
dumped in a resistor). Driver losses are ignored.

Limits come in two kinds. The driver's supply voltage, its peak current and the
motor's maximum speed can never be exceeded, not even for a millisecond; a design that
breaks one of them is ``drive_feasible = False``. The motor's continuous current
rating is thermal: the winding heats over minutes (motor thermal time constant about
20 min), so it is compared with the RMS current over the stride, which assumes
sustained walking. ``feasible`` requires both.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from anklesea.motor import Motor
from anklesea.profiles import Load


@dataclass(frozen=True)
class Design:
    """Series spring stiffness ``k`` in N·m/rad (``np.inf`` for a rigid actuator) and ratio ``N``."""

    stiffness: float
    ratio: float

    @property
    def compliance(self) -> float:
        return 0.0 if np.isinf(self.stiffness) else 1.0 / self.stiffness


@dataclass(frozen=True)
class Limits:
    """Feasibility limits: bus voltage and the motor and driver ratings."""

    bus_voltage: float
    peak_current: float
    rms_current: float
    max_speed: float
    available_voltage: float

    @classmethod
    def from_motor(cls, motor: Motor, bus_voltage: float) -> Limits:
        return cls(
            bus_voltage=bus_voltage,
            peak_current=motor.peak_current,
            rms_current=motor.continuous_current,
            max_speed=motor.max_speed,
            available_voltage=motor.available_voltage(bus_voltage),
        )


def _periodic_derivative(x: np.ndarray, dt: float) -> np.ndarray:
    """Central difference along the last axis, wrapping around (the stride is periodic)."""
    return (np.roll(x, -1, axis=-1) - np.roll(x, 1, axis=-1)) / (2.0 * dt)


def motor_trajectory(
    load: Load,
    compliance: np.ndarray | float,
    ratio: np.ndarray | float,
    motor: Motor,
    efficiency_model: str = "constant",
) -> dict[str, np.ndarray]:
    """Motor-side time series for every design (broadcast over ``compliance`` and ``ratio``).

    ``compliance`` (rad/(N·m)) and ``ratio`` may be arrays of the same shape ``S``; every
    output has shape ``S + (n_samples,)``.
    """
    c = np.asarray(compliance, dtype=float)[..., None]
    n = np.asarray(ratio, dtype=float)[..., None]
    deflection = load.torque * c
    angle = n * (load.angle + deflection)
    speed = n * (load.velocity + load.torque_rate * c)
    accel = n * (load.acceleration + load.torque_accel * c)
    eta = motor.gear_efficiency
    if efficiency_model == "constant":
        reflected = load.torque / (n * eta)
    elif efficiency_model == "directional":
        # Motor drives the transmission when the power it sends out, (tau / N) * omega_m, is positive.
        driving = load.torque * speed >= 0.0
        reflected = np.where(driving, load.torque / (n * eta), load.torque * eta / n)
    else:
        raise ValueError(f"unknown efficiency model {efficiency_model!r}")
    torque = motor.inertia * accel + motor.viscous_friction * speed + reflected
    current = torque / motor.torque_constant
    di_dt = _periodic_derivative(current, load.dt)
    voltage = motor.resistance * current + motor.inductance * di_dt + motor.torque_constant * speed
    return {
        "spring_deflection": np.broadcast_to(deflection, angle.shape),
        "motor_angle": angle,
        "motor_speed": speed,
        "motor_accel": accel,
        "motor_torque": torque,
        "current": current,
        "voltage": voltage,
        "power": voltage * current,
        "copper_loss": motor.resistance * current**2,
    }


def evaluate(
    load: Load,
    compliance: np.ndarray | float,
    ratio: np.ndarray | float,
    motor: Motor,
    limits: Limits,
    efficiency_model: str = "constant",
) -> dict[str, np.ndarray]:
    """Energy per stride and constraint checks for every design (shape of the broadcast inputs).

    Returned quantities (SI): ``energy`` (with regeneration), ``energy_no_regen``,
    ``copper_energy``, ``peak_current``, ``rms_current``, ``peak_voltage``, ``peak_speed``,
    ``peak_deflection``, the boolean checks ``ok_voltage``, ``ok_peak_current``,
    ``ok_rms_current``, ``ok_speed``, ``drive_feasible`` (voltage, peak current and speed),
    ``feasible`` (all four), and
    ``ok_gear_speed`` (gearhead input speed rating, reported only).
    """
    traj = motor_trajectory(load, compliance, ratio, motor, efficiency_model)
    dt = load.dt
    power = traj["power"]
    out = {
        "energy": power.sum(axis=-1) * dt,
        "energy_no_regen": np.clip(power, 0.0, None).sum(axis=-1) * dt,
        "copper_energy": traj["copper_loss"].sum(axis=-1) * dt,
        "peak_current": np.abs(traj["current"]).max(axis=-1),
        "rms_current": np.sqrt((traj["current"] ** 2).mean(axis=-1)),
        "peak_voltage": np.abs(traj["voltage"]).max(axis=-1),
        "peak_speed": np.abs(traj["motor_speed"]).max(axis=-1),
        "peak_deflection": np.abs(traj["spring_deflection"]).max(axis=-1),
    }
    out["ok_voltage"] = out["peak_voltage"] <= limits.available_voltage
    out["ok_peak_current"] = out["peak_current"] <= limits.peak_current
    out["ok_rms_current"] = out["rms_current"] <= limits.rms_current
    out["ok_speed"] = out["peak_speed"] <= limits.max_speed
    out["drive_feasible"] = out["ok_voltage"] & out["ok_peak_current"] & out["ok_speed"]
    out["feasible"] = out["drive_feasible"] & out["ok_rms_current"]
    out["ok_gear_speed"] = out["peak_speed"] <= motor.gear_max_input_speed
    return out


def evaluate_design(load: Load, design: Design, motor: Motor, limits: Limits, **kwargs) -> dict[str, float]:
    """:func:`evaluate` for one design, returning plain floats and bools."""
    result = evaluate(load, design.compliance, design.ratio, motor, limits, **kwargs)
    return {key: value.item() for key, value in result.items()}


def joint_work(load: Load) -> dict[str, float]:
    """Net and positive mechanical work at the joint over the stride (J)."""
    power = load.power
    return {"net": float(power.sum() * load.dt), "positive": float(np.clip(power, 0, None).sum() * load.dt)}
