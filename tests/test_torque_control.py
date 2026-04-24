from dataclasses import replace

import control as ct
import numpy as np
import pytest
from synthetic import synthetic_profile

from anklesea.motor import default_motor
from anklesea.plant import S, SEAParams, fixed_output, joint_motion_to_torque
from anklesea.profiles import fourier_coefficients, fourier_eval
from anklesea.sea import Design
from anklesea.torque_control import (
    Controller,
    PIDGains,
    loop_metrics,
    periodic_tracking,
    pid_loop,
    tune_pid,
)

W = np.logspace(-1, 4, 20000)


@pytest.fixture(scope="module")
def params():
    return SEAParams.from_design(Design(200.0, 440.0), default_motor())


def test_pd_gains_place_the_closed_loop_poles(params) -> None:
    g = tune_pid(params, closed_loop_hz=8.0, damping=0.7)
    pd = g.kp + g.kd * S
    poles = ct.poles(ct.feedback(pd * fixed_output(params), 1))
    wc = 2 * np.pi * 8.0
    assert np.abs(poles) == pytest.approx([wc, wc], rel=1e-6)
    assert -np.real(poles) / np.abs(poles) == pytest.approx([0.7, 0.7], rel=1e-6)


def test_tuning_below_the_resonance_is_rejected(params) -> None:
    with pytest.raises(ValueError):
        tune_pid(params, closed_loop_hz=0.5 * params.fixed_resonance / (2 * np.pi))


def test_margins_of_an_integrator_with_delay() -> None:
    # L = wc / s e^{-sT}: crossover wc, phase margin 90 deg - wc T, phase -180 deg at
    # w180 = pi / (2 T), where the gain margin is w180 / wc.
    wc, delay = 50.0, 2e-3
    loop = wc / (1j * W) * np.exp(-1j * W * delay)
    m = loop_metrics(loop, W)
    assert m.crossover_hz == pytest.approx(wc / (2 * np.pi), rel=1e-3)
    assert m.phase_margin_deg == pytest.approx(90.0 - np.degrees(wc * delay), abs=0.05)
    w180 = np.pi / (2 * delay)
    assert m.gain_margin_db == pytest.approx(20 * np.log10(w180 / wc), abs=0.01)


def test_lower_gain_margin_of_a_conditionally_stable_loop() -> None:
    # L = 10 (s + 1)^2 / s^3: phase -270 deg + 2 atan(w) crosses -180 deg at w = 1, where
    # |L| = 20. The closed loop s^3 + 10 s^2 + 20 s + 10 is stable (Routh: 10 * 20 > 10)
    # and turns unstable if the gain K drops by more than a factor of 20 (Routh for
    # s^3 + K s^2 + 2K s + K: K * 2K > K, so K > 0.5).
    w = np.logspace(-2, 3, 50000)
    jw = 1j * w
    loop = 10 * (jw + 1) ** 2 / jw**3
    m = loop_metrics(loop, w)
    assert m.lower_gain_margin_db == pytest.approx(20 * np.log10(20.0), abs=0.01)
    assert m.gain_margin_db == np.inf
    assert np.all(np.real(ct.poles(ct.feedback(10 * (S + 1) ** 2 / S**3, 1))) < 0)


def test_margins_shrink_with_delay(params) -> None:
    g = tune_pid(params, 6.0)
    no_delay = loop_metrics(pid_loop(params, g, W), W)
    delayed = loop_metrics(pid_loop(params, g, W, delay=2e-3), W)
    assert delayed.phase_margin_deg < no_delay.phase_margin_deg
    assert delayed.gain_margin_db < no_delay.gain_margin_db


def test_model_feedforward_tracks_exactly_without_delay(params) -> None:
    profile = synthetic_profile(70.0)
    tr = periodic_tracking(profile, 70.0, Controller(params, tune_pid(params, 6.0), "model"), delay=0.0)
    assert tr.rms_error_pct() < 1e-9
    # And it is needed: feedback alone leaves an error.
    fb = periodic_tracking(profile, 70.0, Controller(params, tune_pid(params, 6.0), "none"))
    assert fb.rms_error_pct() > 1.0


def test_integral_action_removes_the_mean_error(params) -> None:
    tr = periodic_tracking(synthetic_profile(70.0), 70.0, Controller(params, tune_pid(params, 6.0), "none"))
    assert abs(np.mean(tr.error)) < 1e-6 * np.abs(tr.reference).max()


def test_harmonic_solution_matches_a_time_simulation(params) -> None:
    """Steady state of the closed loop, simulated over many strides, equals the harmonic result."""
    profile = synthetic_profile(70.0)
    gains = PIDGains(kp=0.01, ki=0.03, kd=4e-4, derivative_filter=3e-3)
    tr = periodic_tracking(profile, 70.0, Controller(params, gains, "none"), n_samples=500)
    period = profile.stride_time
    r_c = fourier_coefficients(profile.moment_per_kg * 70.0, 20)
    th_c = fourier_coefficients(profile.angle, 20)
    n_strides = 30
    s = np.arange(n_strides * 500) / 500
    t = s * period
    r = fourier_eval(r_c, s % 1.0, period)
    th = fourier_eval(th_c, s % 1.0, period)
    c, p, d = gains.tf(), fixed_output(params), joint_motion_to_torque(params)
    sens = ct.feedback(1, c * p)
    y = (ct.forced_response(sens * c * p, T=t, U=r).outputs
         + ct.forced_response(sens * d, T=t, U=th).outputs)
    last = y[-500:]
    assert np.max(np.abs(last - tr.torque)) < 0.01 * np.abs(tr.reference).max()


def pd_gains(params) -> PIDGains:
    return replace(tune_pid(params, 8.0), ki=0.0)


def test_dob_with_model_feedforward_tracks_exactly_without_delay(params) -> None:
    profile = synthetic_profile(70.0)
    tr = periodic_tracking(profile, 70.0, Controller(params, pd_gains(params), "model", dob_cutoff_hz=5.0))
    assert tr.rms_error_pct() < 1e-6


def test_dob_removes_a_steady_error_from_a_gain_mismatch(params) -> None:
    # The real gearbox is less efficient than the model: with PD and feedforward alone the
    # DC gain is off; the DOB (Q(0) = 1) acts as integral action and restores it.
    actual = replace(params, efficiency=0.8 * params.efficiency)
    w = np.array([0.0])
    pd_only = Controller(params, pd_gains(params), "reference").closed_loop(w, actual)
    dob = Controller(params, pd_gains(params), "reference", dob_cutoff_hz=5.0).closed_loop(w, actual)
    assert abs(pd_only.torque_per_ref[0] - 1) > 1e-3
    assert dob.torque_per_ref[0] == pytest.approx(1.0, abs=1e-6)


def test_loop_transfer_function_matches_the_frequency_response(params) -> None:
    w = np.logspace(-1, 3, 50)
    for c in [Controller(params, tune_pid(params, 8.0), "none"), Controller(params, pd_gains(params), "model", 5.0)]:
        for output in ["fixed", "free"]:
            direct = c.closed_loop(w, None, output).loop
            via_tf = c.loop_tf(None, output)(1j * w)
            np.testing.assert_allclose(via_tf, direct, rtol=1e-6)


def test_stability_check_agrees_with_the_gain_margin(params) -> None:
    g = tune_pid(params, 8.0)
    m = loop_metrics(pid_loop(params, g, W, delay=1.5e-3), W)
    factor = 10 ** (m.gain_margin_db / 20)

    def scaled(x: float) -> Controller:
        return Controller(params, replace(g, kp=g.kp * x, ki=g.ki * x, kd=g.kd * x), "none")

    assert scaled(0.9 * factor).is_stable(delay=1.5e-3)
    assert not scaled(1.1 * factor).is_stable(delay=1.5e-3)


def test_open_loop_impedance_is_the_rotor_on_the_spring(params) -> None:
    # Motor torque held at zero: the joint sees the spring in series with the reflected
    # rotor inertia and damping, Z = k (Jr s + br) / (Jr s^2 + br s + k), which is passive.
    w = np.logspace(-1, 3, 200)
    jw = 1j * w
    z = Controller(params, PIDGains(0.0, 0.0, 0.0, 1e-3), "none").closed_loop(w).impedance()
    jr, br, k = params.reflected_inertia, params.reflected_damping, params.stiffness
    np.testing.assert_allclose(z, k * (jr * jw + br) / (jr * jw**2 + br * jw + k), rtol=1e-9)
    assert np.all(np.real(z) > 0)


def test_integral_action_breaks_passivity_and_pd_keeps_it(params) -> None:
    """Torque commanded directly (no inner velocity loop): integral action makes the SEA non-passive at low frequency."""
    w = np.logspace(-2, 3, 2000)
    pd = Controller(params, pd_gains(params), "none").closed_loop(w).impedance()
    pid = Controller(params, tune_pid(params, 8.0), "none").closed_loop(w).impedance()
    dob = Controller(params, pd_gains(params), "reference", 5.0).closed_loop(w).impedance()
    assert np.all(np.real(pd) >= 0)
    assert np.real(pid).min() < 0
    assert np.real(dob).min() < 0
