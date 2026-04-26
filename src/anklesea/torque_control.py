"""Torque controllers for the SEA: PID with model-based feedforward, and a disturbance observer.

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

**Disturbance observer (DOB).** Following T. Paine, S. Oh and L. Sentis, "Design and
control considerations for high-performance series elastic actuators", IEEE/ASME
Trans. Mechatronics 19(3):1080-1091, 2014: the nominal fixed-output model ``Pn``
predicts the spring torque that the applied motor torque should produce; anything else
(joint motion, model error, friction) is lumped into an input disturbance and estimated
as ``d_hat = Q (Pn^-1 tau_s - u)``, then subtracted from the next command. ``Q`` is a
unity-gain low-pass of relative degree two, so ``Q Pn^-1`` is proper. Below the Q-filter
cut-off the loop behaves like the nominal model whatever the real plant or the joint
does; above it the DOB is inactive. Because ``Q(0) = 1``, the DOB also acts as integral action, so it is paired with a PD
controller. It forces the loop to follow the nominal model, ``tau_s = Pn (u_ff + C e)``, so
it needs the reference feedforward ``Pn^-1 tau_ref`` to track without a steady error.

**Passivity.** H. Vallery, R. Ekkelenkamp, H. van der Kooij and M. Buss, "Passive and
accurate torque control of series elastic actuators", IEEE/RSJ IROS 2007, pp. 3534-3538,
judge SEA torque controllers by passivity at the joint: a passive actuator stays stable
when coupled to any passive environment, such as the user (Colgate and Hogan, Int. J.
Control 48(1):65-88, 1988). With zero torque reference the joint sees the impedance
``Z(s) = -tau_s(s) / (s theta(s))``; it is passive when the loop is stable and
``Re Z(jw) >= 0`` at all frequencies. :meth:`ClosedLoop.impedance` gives ``Z`` and
``scripts/torque_dob.py`` checks the sign numerically on a frequency grid.
"""

from __future__ import annotations

from dataclasses import dataclass

import control as ct
import numpy as np

from anklesea.plant import S, SEAParams, fixed_output, free_output, joint_motion_to_torque, reflected
from anklesea.profiles import GaitProfile, fourier_coefficients, fourier_eval

FEEDFORWARDS = ("none", "static", "model")
PADE_ORDER = 6


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


def feedforward_response(p: SEAParams, kind: str, jw: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Feedforward motor torque per unit reference torque and per unit joint angle at ``jw``.

    ``"model"`` is the inverse plant, ``[(M + k) tau_ref / k + M theta] / (N eta)`` with
    ``M = eta N^2 (J s^2 + b s)``; ``"reference"`` is its reference part only (no joint
    motion, so no joint acceleration is needed); ``"static"`` only reflects the reference
    through the gear, ``tau_ref / (N eta)``; ``"none"`` is feedback alone.
    """
    zero = np.zeros_like(jw)
    if kind == "none":
        return zero, zero
    if kind == "static":
        return np.full_like(jw, 1 / p.gain), zero
    m = reflected(p)(jw)
    if kind == "reference":
        return (m + p.stiffness) / (p.gain * p.stiffness), zero
    if kind == "model":
        return (m + p.stiffness) / (p.gain * p.stiffness), m / p.gain
    raise ValueError(f"unknown feedforward {kind!r}")


def q_filter(cutoff_hz: float, damping: float = 0.7) -> ct.TransferFunction:
    """Unity-gain second-order low-pass used as the DOB's Q filter."""
    wq = 2 * np.pi * cutoff_hz
    return wq**2 / (S**2 + 2 * damping * wq * S + wq**2)


@dataclass(frozen=True)
class ClosedLoop:
    """Closed-loop frequency responses at ``w``: spring torque ``y`` and motor-torque command ``u``.

    ``y = torque_per_ref r + torque_per_angle theta`` and likewise for ``u``; ``loop`` is the
    loop gain broken at the plant input.
    """

    w: np.ndarray
    torque_per_ref: np.ndarray
    torque_per_angle: np.ndarray
    command_per_ref: np.ndarray
    command_per_angle: np.ndarray
    loop: np.ndarray

    def impedance(self) -> np.ndarray:
        """Interaction impedance ``Z = -tau_s / theta_dot`` seen by the joint with zero reference.

        The power flowing from the joint into the actuator is ``-tau_s theta_dot``; the
        actuator is passive at the joint (Vallery et al., IROS 2007) when the closed loop
        is stable and ``Re Z(jw) >= 0`` at every frequency.
        """
        return -self.torque_per_angle / (1j * self.w)


@dataclass(frozen=True)
class Controller:
    """Torque controller: feedback ``C``, a feedforward, and an optional disturbance observer.

    The DOB (Paine et al. 2014) estimates the input disturbance as
    ``d_hat = Q (Pn^-1 y_n - u)``, where ``u`` is the command it sent and ``y_n`` the measured
    spring torque. When the feedforward already cancels the measured joint motion
    (``"model"``), the DOB's nominal model includes it too, ``y_n = y - Dn theta``, so that
    the joint motion is not compensated twice; otherwise the joint motion is part of the
    disturbance the DOB rejects.
    """

    nominal: SEAParams
    gains: PIDGains
    feedforward: str = "model"
    dob_cutoff_hz: float | None = None

    @property
    def joint_in_observer(self) -> bool:
        return self.dob_cutoff_hz is not None and self.feedforward == "model"

    def closed_loop(self, w: np.ndarray, actual: SEAParams | None = None, output: str = "fixed",
                    delay: float = 0.0) -> ClosedLoop:
        """Responses of the loop on the ``actual`` plant (default: nominal) with a command delay.

        ``output="free"`` is the swing phase (foot inertia only, no joint-motion input).
        With ``q = Q``, ``pi = Pn^-1``, ``j = 1`` if the joint motion is in the DOB model:
        ``u (1 - q) = F_r r + F_th theta + C (r - y) - q pi (y - j Dn theta)`` and
        ``y = P e^{-s delay} u + D theta``, solved for ``u`` and ``y`` at every frequency.
        Zero frequency is replaced by a tiny one, where an integrator or the DOB makes the
        loop gain effectively infinite.
        """
        actual = self.nominal if actual is None else actual
        w_eval = np.where(w == 0, 1e-9, w)
        jw = 1j * w_eval
        if output == "fixed":
            p, d = fixed_output(actual)(jw), joint_motion_to_torque(actual)(jw)
        elif output == "free":
            p, d = free_output(actual)(jw), np.zeros_like(jw)
        else:
            raise ValueError(f"unknown output {output!r}")
        c = self.gains.tf()(jw)
        if self.dob_cutoff_hz is None:
            q = np.zeros_like(jw)
        else:
            q = q_filter(self.dob_cutoff_hz)(jw)
        pi = 1 / fixed_output(self.nominal)(jw)
        dn = joint_motion_to_torque(self.nominal)(jw)
        f_r, f_th = feedforward_response(self.nominal, self.feedforward, jw)
        j = 1.0 if self.joint_in_observer else 0.0
        dl = delay_response(w_eval, delay)
        feedback = c + q * pi
        den = 1 - q + feedback * p * dl
        u_r = (f_r + c) / den
        u_th = (f_th - feedback * d + j * q * pi * dn) / den
        return ClosedLoop(
            w=w,
            torque_per_ref=p * dl * u_r,
            torque_per_angle=p * dl * u_th + d,
            command_per_ref=u_r,
            command_per_angle=u_th,
            loop=feedback * p * dl / (1 - q),
        )

    def loop_tf(self, actual: SEAParams | None = None, output: str = "fixed", delay: float = 0.0
                ) -> ct.TransferFunction:
        """Loop gain as a transfer function, with a Pade approximation of the delay."""
        actual = self.nominal if actual is None else actual
        plant = fixed_output(actual) if output == "fixed" else free_output(actual)
        loop = self.gains.tf() * plant
        if self.dob_cutoff_hz is not None:
            q = q_filter(self.dob_cutoff_hz)
            pn = fixed_output(self.nominal)
            loop = ct.minreal((self.gains.tf() + q / pn) * plant / (1 - q), verbose=False)
        if delay > 0:
            loop = loop * ct.tf(*ct.pade(delay, PADE_ORDER))
        return loop

    def is_stable(self, actual: SEAParams | None = None, output: str = "fixed", delay: float = 0.0,
                  tol: float = 1e-6) -> bool:
        """Closed-loop stability from the poles of ``L / (1 + L)``.

        A pole at the origin from an integrator cancelling the free plant's zero at DC is
        allowed: it is the uncontrollable mode of a free foot, not an instability.
        """
        poles = ct.poles(ct.feedback(self.loop_tf(actual, output, delay), 1))
        scale = max(1.0, float(np.abs(poles).max()))
        return bool(np.all(np.real(poles) < tol * scale))


@dataclass(frozen=True)
class Tracking:
    t: np.ndarray
    reference: np.ndarray  # N·m
    torque: np.ndarray  # spring torque, N·m
    motor_torque: np.ndarray  # command at the motor shaft, N·m

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
    controller: Controller,
    actual: SEAParams | None = None,
    delay: float = 0.0,
    n_harmonics: int = 20,
    n_samples: int = 1000,
) -> Tracking:
    """Steady-state periodic torque tracking of one stride by the linear stance model.

    The reference torque and the joint angle are periodic, so the closed loop's
    steady-state response is computed exactly, harmonic by harmonic, from
    :meth:`Controller.closed_loop`. The delay (sample and hold plus computation) acts on
    the whole command, so it also delays the feedforward. ``motor_torque`` is the command
    before the delay.
    """
    r = fourier_coefficients(profile.moment_per_kg * mass, n_harmonics)
    th = fourier_coefficients(profile.angle, n_harmonics)
    w = 2 * np.pi * np.arange(n_harmonics + 1) / profile.stride_time
    cl = controller.closed_loop(w, actual, "fixed", delay)
    y = cl.torque_per_ref * r + cl.torque_per_angle * th
    u = cl.command_per_ref * r + cl.command_per_angle * th
    s = np.arange(n_samples) / n_samples
    return Tracking(
        t=s * profile.stride_time,
        reference=fourier_eval(r, s, profile.stride_time),
        torque=fourier_eval(y, s, profile.stride_time),
        motor_torque=fourier_eval(u, s, profile.stride_time),
    )


def dob_cutoff(
    nominal: SEAParams,
    gains: PIDGains,
    delay: float,
    acceptable,
    cutoffs_hz: np.ndarray | None = None,
) -> float:
    """Highest Q-filter cut-off whose stance loop is ``acceptable`` (a predicate on :class:`LoopMetrics`)."""
    cutoffs_hz = np.round(np.arange(1.0, 30.01, 0.5), 1) if cutoffs_hz is None else cutoffs_hz
    w = np.logspace(-2, 4, 30000)
    ok = [float(fq) for fq in cutoffs_hz
          if acceptable(loop_metrics(Controller(nominal, gains, "model", float(fq)).closed_loop(w, None, "fixed",
                                                                                                delay).loop, w))]
    if not ok:
        raise ValueError("no cut-off meets the targets")
    return max(ok)


def pid_loop(p: SEAParams, gains: PIDGains, w: np.ndarray, delay: float = 0.0, output: str = "fixed") -> np.ndarray:
    """Loop frequency response ``C(jw) P(jw) e^{-jw delay}`` of a PID controller alone."""
    return Controller(p, gains, "none").closed_loop(w, None, output, delay).loop


@dataclass(frozen=True)
class Environment:
    """A passive joint load: ``J_e theta_ddot + b_e theta_dot + k_e theta = tau_s``."""

    inertia: float  # kg m^2
    damping: float  # N·m·s/rad
    stiffness: float  # N·m/rad

    def impedance_poly(self, jw: np.ndarray) -> np.ndarray:
        return self.inertia * jw**2 + self.damping * jw + self.stiffness


def coupled_stable(torque_per_angle, env: Environment, w: np.ndarray | None = None) -> bool:
    """Nyquist test of the actuator coupled to ``env`` instead of a prescribed joint motion.

    ``torque_per_angle(w)`` is the actuator's spring torque per joint angle with zero
    reference (stable on its own). The joint obeys ``E(s) theta = tau_s`` with
    ``E = J_e s^2 + b_e s + k_e``, so the coupled characteristic equation is
    ``1 + L = 0`` with ``L = -H_theta / E``. Both factors of ``L`` are stable (``b_e > 0``),
    so the coupled system is stable if and only if ``1 + L(jw)`` does not wind around the
    origin; for a real system the winding over all frequencies is twice that over
    ``w >= 0``.
    """
    if w is None:
        w = np.concatenate([[1e-6], np.logspace(-3, 6, 400000)])
    one_plus_l = 1 - torque_per_angle(w) / env.impedance_poly(1j * w)
    phase = np.unwrap(np.angle(one_plus_l))
    winding = 2 * (phase[-1] - phase[0]) / (2 * np.pi)
    return bool(abs(winding) < 0.5)
