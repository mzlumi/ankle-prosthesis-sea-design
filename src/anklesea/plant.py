"""Linear model of the SEA for torque control (python-control transfer functions).

Coordinates on the output side: ``q = theta_m / N`` is the motor angle seen through the
transmission, ``theta`` the joint angle and ``tau_s = k (q - theta)`` the spring torque,
which is what the controller measures (from the spring deflection) and controls. The
input is the motor torque ``tau_m = k_t i``; the driver's current loop is taken as ideal
here and modeled in the time-domain simulation.

The motor equation is the one of the energy model (:mod:`anklesea.sea`):

``J theta_m_ddot + b theta_m_dot = tau_m - tau_s / (N eta)``

which on the output side reads ``M q = N eta tau_m - tau_s`` with
``M(s) = eta N^2 (J s^2 + b s)``: the rotor inertia and friction reflected through the
transmission, ``eta N^2`` times larger. With the efficiency as a constant gain, the
effective reflected inertia is ``eta J N^2``.

Two boundary cases bracket the ankle in use:

* **fixed output** (``theta`` held, as when the foot is planted and the body loads the
  ankle): ``tau_s / tau_m = N eta k / (M + k)``, a second-order low-pass with DC gain
  ``N eta`` and resonance at ``sqrt(k / (eta J N^2))``. The spring and the reflected
  rotor inertia form the resonance; a softer spring or a higher ratio lowers it.
* **free output** (the foot in swing, load inertia ``J_L``):
  ``tau_s / tau_m = N eta k L / (M L + k (M + L))`` with ``L = J_L s^2 + b_L s``. It has
  zero DC gain (a free foot cannot hold a steady torque: the torque only accelerates the
  foot) and a resonance at ``sqrt(k (1 / (eta J N^2) + 1 / J_L))``, set mostly by the
  light foot, so far above the fixed-output resonance.

Joint motion acts as a disturbance on the spring torque: with the motor torque held,
``tau_s / theta = -k M / (M + k)``. At low frequency the rotor follows (the spring is
not deflected); above the resonance the rotor stays put and the joint motion deflects
the spring fully, ``-k theta``.
"""

from __future__ import annotations

from dataclasses import dataclass

import control as ct
import numpy as np

from anklesea.motor import Motor
from anklesea.sea import Design

S = ct.tf("s")


@dataclass(frozen=True)
class SEAParams:
    """Physical parameters of one SEA (SI units, output side unless noted)."""

    stiffness: float  # N·m/rad
    ratio: float
    rotor_inertia: float  # kg m^2 at the motor shaft (rotor plus gearhead)
    viscous_friction: float  # N·m·s/rad at the motor shaft
    efficiency: float
    torque_constant: float  # N·m/A
    load_inertia: float = 0.01  # kg m^2, foot below the spring (docs/assumptions.md)
    load_damping: float = 0.0  # N·m·s/rad

    @classmethod
    def from_design(cls, design: Design, motor: Motor, **kwargs: float) -> SEAParams:
        return cls(
            stiffness=design.stiffness,
            ratio=design.ratio,
            rotor_inertia=motor.inertia,
            viscous_friction=motor.viscous_friction,
            efficiency=motor.gear_efficiency,
            torque_constant=motor.torque_constant,
            **kwargs,
        )

    @property
    def reflected_inertia(self) -> float:
        return self.efficiency * self.rotor_inertia * self.ratio**2

    @property
    def reflected_damping(self) -> float:
        return self.efficiency * self.viscous_friction * self.ratio**2

    @property
    def gain(self) -> float:
        """Spring torque per unit motor torque at DC with the output fixed (``N eta``)."""
        return self.ratio * self.efficiency

    @property
    def fixed_resonance(self) -> float:
        """Natural frequency with the output fixed (rad/s)."""
        return float(np.sqrt(self.stiffness / self.reflected_inertia))

    @property
    def free_resonance(self) -> float:
        return float(np.sqrt(self.stiffness * (1 / self.reflected_inertia + 1 / self.load_inertia)))


def reflected(p: SEAParams) -> ct.TransferFunction:
    """``M(s)``: reflected rotor impedance, output torque per output angle."""
    return p.reflected_inertia * S**2 + p.reflected_damping * S


def fixed_output(p: SEAParams) -> ct.TransferFunction:
    """Spring torque per motor torque with the joint held."""
    return ct.minreal(p.gain * p.stiffness / (reflected(p) + p.stiffness), verbose=False)


def free_output(p: SEAParams) -> ct.TransferFunction:
    """Spring torque per motor torque with the joint free and load inertia ``J_L``."""
    m = reflected(p)
    load = p.load_inertia * S**2 + p.load_damping * S
    return ct.minreal(p.gain * p.stiffness * load / (m * load + p.stiffness * (m + load)), verbose=False)


def joint_motion_to_torque(p: SEAParams) -> ct.TransferFunction:
    """Spring torque per joint angle with the motor torque held at zero."""
    m = reflected(p)
    return ct.minreal(-p.stiffness * m / (m + p.stiffness), verbose=False)


def peak_frequency(sys: ct.TransferFunction, w: np.ndarray | None = None) -> float:
    """Frequency (rad/s) of the largest magnitude on a log grid."""
    w = np.logspace(-1, 4, 20001) if w is None else w
    mag = np.abs(sys(1j * w))
    return float(w[np.argmax(mag)])
