"""Torque controllers for the SEA: PID with model-based feedforward.

**PID.** The fixed-output plant is a lightly damped second-order system,
``P(s) = G wn^2 / (s^2 + 2 zeta wn s + wn^2)`` with ``G = N eta``. A PD controller
``Kp + Kd s`` gives the closed-loop characteristic polynomial
``s^2 + (2 zeta wn + G wn^2 Kd) s + wn^2 (1 + G Kp)``, so the gains that place the
closed-loop poles at a chosen frequency ``wc`` and damping ``zeta_c`` follow directly:

``1 + G Kp = (wc / wn)^2``,  ``G wn^2 Kd = 2 zeta_c wc - 2 zeta wn``.

The proportional gain stiffens the loop (raises the resonance to ``wc``) and the
derivative gain damps it. A small integral term, well below ``wc``, removes the
steady-state error that a PD loop leaves (``1 / (1 + G Kp)``), and the derivative is
filtered so that the controller rolls off at high frequency.

**Feedforward.** The motor torque that produces a spring torque ``tau_ref`` while the
joint moves along ``theta`` follows from the plant equations without any feedback:

``tau_ff = [eta N^2 (J (theta_ddot + tau_ref_ddot / k) + b (theta_dot + tau_ref_dot / k)) + tau_ref] / (N eta)``

which is exactly the motor torque of the sizing model (:func:`anklesea.sea.motor_trajectory`)
for the reference torque. It cancels both the plant's own dynamics and the joint motion,
which the feedback alone would see as a disturbance; the feedback then only corrects
model error.
"""

from __future__ import annotations

from dataclasses import dataclass

import control as ct
import numpy as np

from anklesea.plant import S, SEAParams, fixed_output, joint_motion_to_torque, reflected
from anklesea.profiles import GaitProfile, fourier_coefficients, fourier_eval

FEEDFORWARDS = ("none", "static", "model")


@dataclass(frozen=True)
class PIDGains:
    kp: float  # N·m of motor torque per N·m of torque error
    ki: float  # per second
    kd: float  # seconds
    derivative_filter: float  # time constant of the derivative filter, s

    def tf(self) -> ct.TransferFunction:
        return self.kp + self.ki / S + self.kd * S / (self.derivative_filter * S + 1)


def tune_pid(
    p: SEAParams,
    closed_loop_hz: float,
    damping: float = 0.7,
    integral_fraction: float = 0.1,
    filter_fraction: float = 0.1,
) -> PIDGains:
    """Pole-placement PD on the fixed-output plant plus a slow integral and a derivative filter.

    ``integral_fraction`` sets the integral corner as a fraction of ``wc``; ``filter_fraction``
    sets the derivative filter time constant to ``filter_fraction / wc``.
    """
    wc = 2 * np.pi * closed_loop_hz
    wn = p.fixed_resonance
    zeta = p.reflected_damping / (2 * np.sqrt(p.stiffness * p.reflected_inertia))
    g = p.gain
    kp = ((wc / wn) ** 2 - 1) / g
    kd = (2 * damping * wc - 2 * zeta * wn) / (g * wn**2)
    if kp <= 0 or kd <= 0:
        raise ValueError("closed-loop frequency must exceed the open-loop resonance")
    return PIDGains(kp=kp, ki=kp * integral_fraction * wc, kd=kd, derivative_filter=filter_fraction / wc)


def delay_response(w: np.ndarray, delay: float) -> np.ndarray:
    return np.exp(-1j * w * delay)


@dataclass(frozen=True)
class LoopMetrics:
    bandwidth_hz: float  # |T| first falls below -3 dB
    crossover_hz: float  # |L| = 1
    phase_margin_deg: float
    gain_margin_db: float  # how far the loop gain may rise before instability
    lower_gain_margin_db: float  # how far it may fall (finite only for a conditionally stable loop)
    peak_sensitivity: float  # max |S|
    peak_complementary: float  # max |T|


def loop_metrics(loop: np.ndarray, w: np.ndarray) -> LoopMetrics:
    """Stability margins and bandwidth from the loop frequency response ``L(jw)`` on grid ``w``.

    Every crossing of -180 deg (mod 360) is a gain margin: where ``|L| < 1`` the gain may
    rise by ``1 / |L|``; where ``|L| > 1`` (a conditionally stable loop, typical of an
    integrator in front of a lightly damped resonance) it may only fall by ``|L|``.
    Assumes the closed loop is stable at the nominal gain.
    """
    t = loop / (1 + loop)
    s = 1 / (1 + loop)
    mag_t = np.abs(t)
    below = np.flatnonzero(mag_t < mag_t[0] / np.sqrt(2))
    bandwidth = w[below[0]] if below.size else np.inf
    mag_l = np.abs(loop)
    cross = np.flatnonzero((mag_l[:-1] >= 1) & (mag_l[1:] < 1))
    phase = np.unwrap(np.angle(loop))
    if cross.size:
        i = cross[-1]
        pm = 180.0 + np.degrees(phase[i])
        wc = w[i]
    else:
        pm, wc = np.inf, np.nan
    # Phase crossings of -180 deg (mod 360) above the gain crossover set the gain margin.
    shifted = (np.degrees(phase) + 180.0) / 360.0
    flips = np.flatnonzero(np.floor(shifted[:-1]) != np.floor(shifted[1:]))
    gm, gm_lower = np.inf, np.inf
    for i in flips:
        db = 20 * np.log10(mag_l[i])
        if db < 0:
            gm = min(gm, -db)
        else:
            gm_lower = min(gm_lower, db)
    return LoopMetrics(
        bandwidth_hz=float(bandwidth / (2 * np.pi)),
        crossover_hz=float(wc / (2 * np.pi)),
        phase_margin_deg=float(pm),
        gain_margin_db=float(gm),
        lower_gain_margin_db=float(gm_lower),
        peak_sensitivity=float(np.abs(s).max()),
        peak_complementary=float(mag_t.max()),
    )


def pid_loop(p: SEAParams, gains: PIDGains, w: np.ndarray, delay: float = 0.0, plant: ct.TransferFunction | None = None
             ) -> np.ndarray:
    """Loop frequency response ``C(jw) P(jw) e^{-jw delay}`` with the fixed-output plant by default."""
    plant = fixed_output(p) if plant is None else plant
    return gains.tf()(1j * w) * plant(1j * w) * delay_response(w, delay)


def feedforward_response(p: SEAParams, kind: str, jw: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Feedforward motor torque per unit reference torque and per unit joint angle at ``jw``.

    ``"model"`` is the inverse plant, ``[(M + k) tau_ref / k + M theta] / (N eta)`` with
    ``M = eta N^2 (J s^2 + b s)``; ``"static"`` only reflects the reference through the
    gear, ``tau_ref / (N eta)``; ``"none"`` is feedback alone.
    """
    if kind == "none":
        return np.zeros_like(jw), np.zeros_like(jw)
    if kind == "static":
        return np.full_like(jw, 1 / p.gain), np.zeros_like(jw)
    if kind == "model":
        m = reflected(p)(jw)
        return (m + p.stiffness) / (p.gain * p.stiffness), m / p.gain
    raise ValueError(f"unknown feedforward {kind!r}")


@dataclass(frozen=True)
class Tracking:
    t: np.ndarray
    reference: np.ndarray  # N·m
    torque: np.ndarray  # spring torque, N·m
    motor_torque: np.ndarray  # at the motor shaft, N·m

    @property
    def error(self) -> np.ndarray:
        return self.torque - self.reference

    def rms_error_pct(self) -> float:
        """RMS error as a percentage of the peak reference torque."""
        return float(100 * np.sqrt(np.mean(self.error**2)) / np.abs(self.reference).max())

    def peak_error_pct(self) -> float:
        return float(100 * np.abs(self.error).max() / np.abs(self.reference).max())


def periodic_tracking(
    profile: GaitProfile,
    mass: float,
    controller: PIDGains,
    nominal: SEAParams,
    feedforward: str = "model",
    actual: SEAParams | None = None,
    delay: float = 0.0,
    n_harmonics: int = 20,
    n_samples: int = 1000,
) -> Tracking:
    """Steady-state periodic torque tracking of one stride by the linear stance model.

    The reference torque and the joint angle are periodic, so the closed loop's
    steady-state response is computed exactly, harmonic by harmonic:
    ``y = (P e^{-s delay} (u_ff + C r) + D theta) / (1 + P C e^{-s delay})`` with ``P`` and
    ``D`` the fixed-output and joint-motion responses of the ``actual`` plant (default: the
    nominal one) and the feedforward built from the ``nominal`` model. The delay (sample and
    hold plus computation) acts on the whole command, so it also delays the feedforward.
    ``motor_torque`` is the command before the delay.
    """
    actual = nominal if actual is None else actual
    r = fourier_coefficients(profile.moment_per_kg * mass, n_harmonics)
    th = fourier_coefficients(profile.angle, n_harmonics)
    w = 2 * np.pi * np.arange(n_harmonics + 1) / profile.stride_time
    jw = 1j * w
    p_a = fixed_output(actual)(jw)
    d_a = joint_motion_to_torque(actual)(jw)
    c = np.zeros_like(jw)
    if controller is not None:
        c[1:] = controller.tf()(jw[1:])
        c[0] = controller.kp
    # At DC the integrator makes |C| infinite: the loop then forces y = r exactly.
    dc_integral = controller is not None and controller.ki > 0
    f_r, f_th = feedforward_response(nominal, feedforward, jw)
    u_ff = f_r * r + f_th * th
    dl = delay_response(w, delay)
    y = (p_a * dl * (u_ff + c * r) + d_a * th) / (1 + p_a * c * dl)
    u = u_ff + c * (r - y)
    if dc_integral:
        y[0] = r[0]
        u[0] = (y[0] - d_a[0] * th[0]) / p_a[0]
    s = np.arange(n_samples) / n_samples
    return Tracking(
        t=s * profile.stride_time,
        reference=fourier_eval(r, s, profile.stride_time),
        torque=fourier_eval(y, s, profile.stride_time),
        motor_torque=fourier_eval(u, s, profile.stride_time),
    )
