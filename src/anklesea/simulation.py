"""Nonlinear time-domain simulation of the SEA tracking a gait torque, with drive limits.

The joint angle is prescribed (the user's leg moves the joint; the actuator only sets the
spring torque). The simulated physics, at ``sim_rate_hz``:

* rotor ``J omega_m_dot = k_t i - b omega_m - tau_s / (N eta)`` with spring torque
  ``tau_s = k (theta_m / N - theta)``;
* winding ``L di/dt = v - R i - k_t omega_m``, integrated exactly over each step for a
  constant voltage and speed.

The driver runs a PI current loop at the same rate, with back-EMF compensation, an
output voltage clipped to the driver's limit (``0.95 V_bus``) and a current reference
clipped to the peak current; its integrator stops while the voltage is saturated
(anti-windup). The torque controller (:class:`anklesea.torque_control.Controller`) runs at
``control_rate_hz`` on sampled signals: it computes the command from the samples at
``t_k`` and applies it from ``t_{k+1}`` (one sample of computation delay plus the
zero-order hold, about 1.5 samples in total). Its continuous parts are discretized with
the Tustin transform. Anti-windup: the PID integrator stops while the current reference
is clipped or the driver's voltage was saturated during the last control period, and the
DOB compares the measured torque with the torque the motor actually produced
(``k_t`` times the sampled motor current) rather than with the command, so that a
saturated drive is not mistaken for a disturbance.

The torque is measured as the spring deflection (motor encoder minus joint encoder) times
the *nominal* stiffness, as on a real SEA, so a stiffness error corrupts the measurement
as well as the model. ``torque_sensor="load_cell"`` measures the true torque instead.
The feedforward needs the joint velocity and acceleration: ``"exact"`` uses the
analytic derivatives of the prescribed motion, ``"estimated"`` passes the sampled joint
angle (plus optional noise) through a second-order state-variable filter.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import control as ct
import numpy as np

from anklesea.motor import Motor
from anklesea.plant import SEAParams, fixed_output, joint_motion_to_torque
from anklesea.profiles import GaitProfile, fourier_coefficients, fourier_eval
from anklesea.torque_control import Controller, q_filter


@dataclass(frozen=True)
class Drive:
    """Electrical constants of the actual motor and the driver's limits."""

    torque_constant: float  # N·m/A
    resistance: float  # ohm
    inductance: float  # H
    voltage_limit: float  # V across the winding
    current_limit: float  # A
    current_bandwidth_hz: float = 1000.0  # docs/assumptions.md

    @classmethod
    def from_motor(cls, motor: Motor, bus_voltage: float, **kwargs: float) -> Drive:
        return cls(motor.torque_constant, motor.resistance, motor.inductance,
                   motor.available_voltage(bus_voltage), motor.peak_current, **kwargs)


@dataclass(frozen=True)
class SimOptions:
    sim_rate_hz: float = 20000.0
    control_rate_hz: float = 1000.0
    n_strides: int = 4  # the last one is evaluated
    joint_derivatives: str = "estimated"
    derivative_filter_hz: float = 25.0
    torque_sensor: str = "deflection"
    torque_noise: float = 0.0  # N·m RMS on the measured torque
    angle_noise: float = 0.0  # rad RMS on the joint angle sample
    extra_delay_samples: int = 0  # further whole samples of loop delay
    naive_windup: bool = False  # integrator frozen on current clipping only; DOB fed the command
    seed: int = 0


@dataclass
class SimResult:
    """Signals of the last stride, sampled at the control rate, and summary numbers."""

    t: np.ndarray
    reference: np.ndarray
    torque: np.ndarray  # true spring torque
    measured: np.ndarray
    current: np.ndarray
    voltage: np.ndarray
    motor_speed: np.ndarray
    voltage_saturated: float  # fraction of the stride
    current_saturated: float
    energy: float  # electrical energy into the winding over the stride, J
    energy_no_regen: float
    extra: dict = field(default_factory=dict)

    @property
    def error(self) -> np.ndarray:
        return self.torque - self.reference

    def rms_error_pct(self) -> float:
        return float(100 * np.sqrt(np.mean(self.error**2)) / np.abs(self.reference).max())

    def peak_error_pct(self) -> float:
        return float(100 * np.abs(self.error).max() / np.abs(self.reference).max())

    @property
    def rms_current(self) -> float:
        return float(np.sqrt(np.mean(self.current**2)))


class _Discrete:
    """A SISO transfer function discretized with Tustin, run one sample at a time."""

    def __init__(self, sys: ct.TransferFunction, dt: float) -> None:
        d = ct.c2d(sys, dt, method="tustin")
        num = np.atleast_1d(np.squeeze(d.num[0][0])).astype(float)
        den = np.atleast_1d(np.squeeze(d.den[0][0])).astype(float)
        n = max(len(num), len(den))
        self.b = np.concatenate([np.zeros(n - len(num)), num]) / den[0]
        self.a = np.concatenate([np.zeros(n - len(den)), den]) / den[0]
        self.z = np.zeros(n - 1)

    def __call__(self, x: float) -> float:
        # Direct form II transposed.
        b, a, z = self.b, self.a, self.z
        y = b[0] * x + (z[0] if len(z) else 0.0)
        for i in range(len(z) - 1):
            z[i] = b[i + 1] * x - a[i + 1] * y + z[i + 1]
        if len(z):
            z[-1] = b[-1] * x - a[-1] * y
        return y


def simulate(
    profile: GaitProfile,
    mass: float,
    controller: Controller,
    drive: Drive,
    actual: SEAParams | None = None,
    options: SimOptions = SimOptions(),
    n_harmonics: int = 20,
) -> SimResult:
    """Simulate ``options.n_strides`` strides of ``profile`` and return the last one."""
    nominal = controller.nominal
    actual = nominal if actual is None else actual
    rng = np.random.default_rng(options.seed)
    period = profile.stride_time
    ratio_ctrl = int(round(options.sim_rate_hz / options.control_rate_hz))
    dt = 1.0 / options.sim_rate_hz
    ts = ratio_ctrl * dt
    n_ctrl_stride = int(round(period / ts))
    period = n_ctrl_stride * ts  # whole control samples per stride
    n_ctrl = n_ctrl_stride * options.n_strides

    rc = fourier_coefficients(profile.moment_per_kg * mass, n_harmonics)
    thc = fourier_coefficients(profile.angle, n_harmonics)

    def series(c: np.ndarray, s: np.ndarray, d: int = 0) -> np.ndarray:
        return fourier_eval(c, s % 1.0, period, d)

    # Joint motion at the simulation rate and references at the control rate.
    s_sim = np.arange(n_ctrl * ratio_ctrl + 1) * dt / period
    theta = series(thc, s_sim)
    s_ctrl = np.arange(n_ctrl) * ts / period
    ref = series(rc, s_ctrl)
    ref_d1, ref_d2 = series(rc, s_ctrl, 1), series(rc, s_ctrl, 2)
    th_d1, th_d2 = series(thc, s_ctrl, 1), series(thc, s_ctrl, 2)

    n, eta = actual.ratio, actual.efficiency
    j, b, k = actual.rotor_inertia, actual.viscous_friction, actual.stiffness
    kt, r, ind = drive.torque_constant, drive.resistance, drive.inductance
    kt_n, k_n, n_n, eta_n = nominal.torque_constant, nominal.stiffness, nominal.ratio, nominal.efficiency
    jn, bn = nominal.rotor_inertia, nominal.viscous_friction
    wc = 2 * np.pi * drive.current_bandwidth_hz
    kp_i, ki_i = ind * wc, r * wc  # PI zero cancels the winding pole
    decay = np.exp(-r * dt / ind)

    # Torque controller pieces.
    g = controller.gains
    d_filter = _Discrete(g.kd * ct.tf([1, 0], [g.derivative_filter, 1]), ts)
    dob = controller.dob_cutoff_hz is not None
    if dob:
        q = q_filter(controller.dob_cutoff_hz)
        q_pinv = _Discrete(ct.minreal(q / fixed_output(nominal), verbose=False), ts)
        q_u = _Discrete(q, ts)
        joint_effect = _Discrete(joint_motion_to_torque(nominal), ts)
    ff_kind = controller.feedforward
    wf = 2 * np.pi * options.derivative_filter_hz

    # Initial state: on the nominal trajectory.
    th0, thd0 = theta[0], series(thc, np.array([0.0]), 1)[0]
    theta_m = n * (th0 + ref[0] / k)
    omega = n * (thd0 + ref_d1[0] / k)
    i = (j * n * (th_d2[0] + ref_d2[0] / k) + b * omega + ref[0] / (n * eta)) / kt
    x_i = r * i  # current-loop integrator holds the resistive drop
    integ = 0.0
    u_applied = np.zeros(options.extra_delay_samples + 1)
    u_applied[:] = (ref[0] / (n_n * eta_n))
    est = np.array([th0, thd0, 0.0])  # state-variable filter: angle, velocity, acceleration
    d_hat = 0.0
    v_sat_last = False
    u_prev = u_applied[0]

    keep = slice((options.n_strides - 1) * n_ctrl_stride, n_ctrl)
    out = {key: np.zeros(n_ctrl) for key in ["torque", "measured", "current", "voltage", "speed"]}
    v_sat_steps = i_sat_steps = 0
    energy = energy_pos = 0.0
    last_start = (options.n_strides - 1) * n_ctrl_stride * ratio_ctrl

    step = 0
    for kc in range(n_ctrl):
        # --- sample and compute the torque command (applied from the next sample on) ---
        th_s = theta[step] + options.angle_noise * rng.standard_normal()
        tau_true = k * (theta_m / n - theta[step])
        if options.torque_sensor == "deflection":
            tau_meas = k_n * (theta_m / n_n - th_s)
        else:
            tau_meas = tau_true
        tau_meas += options.torque_noise * rng.standard_normal()

        if options.joint_derivatives == "exact":
            th_hat, thd_hat, thdd_hat = th_s, th_d1[kc], th_d2[kc]
        else:
            acc = wf**2 * (th_s - est[0]) - 2 * 0.7 * wf * est[1]
            est = est + ts * np.array([est[1], acc, 0.0])
            est[2] = acc
            th_hat, thd_hat, thdd_hat = est[0], est[1], est[2]

        r_k = ref[kc]
        if ff_kind == "none":
            u_ff = 0.0
        elif ff_kind == "static":
            u_ff = r_k / (n_n * eta_n)
        else:
            rd, rdd = ref_d1[kc] / k_n, ref_d2[kc] / k_n
            if ff_kind == "model":
                acc_m, vel_m = thdd_hat + rdd, thd_hat + rd
            else:  # reference only
                acc_m, vel_m = rdd, rd
            u_ff = jn * n_n * acc_m + bn * n_n * vel_m + r_k / (n_n * eta_n)

        e = r_k - tau_meas
        u_fb = g.kp * e + integ + d_filter(e)
        if dob:
            y_n = tau_meas
            if controller.joint_in_observer:
                y_n = tau_meas - joint_effect(th_s)
            d_hat = q_pinv(y_n) - q_u(u_prev if options.naive_windup else kt_n * i)
        u = u_ff + u_fb - d_hat
        i_cmd = u / kt_n
        clipped = abs(i_cmd) > drive.current_limit
        if not clipped and (options.naive_windup or not v_sat_last) and g.ki:
            integ += g.ki * ts * e
        u_prev = u
        u_applied = np.roll(u_applied, -1)
        u_applied[-1] = u
        i_ref = float(np.clip(u_applied[0] / kt_n, -drive.current_limit, drive.current_limit))

        out["torque"][kc] = tau_true
        out["measured"][kc] = tau_meas

        # --- physics and current loop between two control samples ---
        v_acc = i_acc = w_acc = 0.0
        v_sat_last = False
        for _ in range(ratio_ctrl):
            err_i = i_ref - i
            v_cmd = kp_i * err_i + x_i + kt_n * omega
            v = min(max(v_cmd, -drive.voltage_limit), drive.voltage_limit)
            v_sat = v != v_cmd
            v_sat_last |= v_sat
            if not v_sat:
                x_i += ki_i * dt * err_i
            if step >= last_start:
                v_sat_steps += v_sat
                i_sat_steps += abs(u_applied[0] / kt_n) > drive.current_limit
                p = v * i
                energy += p * dt
                energy_pos += max(p, 0.0) * dt
            tau_s = k * (theta_m / n - theta[step])
            omega += dt * (kt * i - b * omega - tau_s / (n * eta)) / j
            theta_m += dt * omega
            i_ss = (v - kt * omega) / r
            i = i_ss + (i - i_ss) * decay
            v_acc += v
            i_acc += i
            w_acc += omega
            step += 1
        out["current"][kc] = i_acc / ratio_ctrl
        out["voltage"][kc] = v_acc / ratio_ctrl
        out["speed"][kc] = w_acc / ratio_ctrl

    n_last = n_ctrl_stride * ratio_ctrl
    return SimResult(
        t=np.arange(n_ctrl_stride) * ts,
        reference=ref[keep],
        torque=out["torque"][keep],
        measured=out["measured"][keep],
        current=out["current"][keep],
        voltage=out["voltage"][keep],
        motor_speed=out["speed"][keep],
        voltage_saturated=v_sat_steps / n_last,
        current_saturated=i_sat_steps / n_last,
        energy=energy,
        energy_no_regen=energy_pos,
    )
