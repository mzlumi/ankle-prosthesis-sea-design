from dataclasses import replace

import control as ct
import numpy as np
import pytest
from synthetic import synthetic_profile

from anklesea.motor import default_motor
from anklesea.plant import SEAParams
from anklesea.sea import Design, Limits, evaluate_design
from anklesea.simulation import Drive, SimOptions, _Discrete, simulate
from anklesea.torque_control import Controller, tune_pid

MASS = 70.0
BUS = 36.0


@pytest.fixture(scope="module")
def setup():
    motor = default_motor()
    design = Design(250.0, 350.0)
    p = SEAParams.from_design(design, motor)
    controller = Controller(p, tune_pid(p, 8.0, integral_fraction=0.2), "model")
    drive = Drive.from_motor(motor, BUS)
    return motor, design, p, controller, drive


def unlimited(drive: Drive) -> Drive:
    return replace(drive, voltage_limit=1e6, current_limit=1e6)


def test_tustin_filter_keeps_the_dc_gain() -> None:
    f = _Discrete(ct.tf([3.0], [0.01, 1.0]), 1e-3)
    y = [f(1.0) for _ in range(500)]
    assert y[-1] == pytest.approx(3.0, rel=1e-9)


def test_tracking_reproduces_the_sizing_model(setup) -> None:
    """With exact tracking the motor current and energy are those of the inverse model used for sizing."""
    motor, design, _, controller, drive = setup
    profile = synthetic_profile(MASS)
    sim = simulate(profile, MASS, controller, unlimited(drive), options=SimOptions(joint_derivatives="exact"))
    model = evaluate_design(profile.load(MASS), design, motor, Limits.from_motor(motor, BUS))
    assert sim.rms_error_pct() < 0.5
    assert sim.rms_current == pytest.approx(model["rms_current"], rel=0.01)
    assert sim.energy == pytest.approx(model["energy"], rel=0.02)


def test_a_low_voltage_limit_saturates_and_costs_accuracy(setup) -> None:
    _, _, _, controller, drive = setup
    profile = synthetic_profile(MASS)
    free = simulate(profile, MASS, controller, unlimited(drive))
    peak_v = np.abs(free.voltage).max()
    tight = simulate(profile, MASS, controller, replace(drive, voltage_limit=0.6 * peak_v, current_limit=1e6))
    assert free.voltage_saturated == 0.0
    assert tight.voltage_saturated > 0.01
    assert tight.rms_error_pct() > 2 * free.rms_error_pct()
    assert np.abs(tight.voltage).max() <= 0.6 * peak_v + 1e-9


def test_current_limit_caps_the_current(setup) -> None:
    _, _, _, controller, drive = setup
    profile = synthetic_profile(MASS)
    free = simulate(profile, MASS, controller, unlimited(drive))
    limit = 0.7 * np.abs(free.current).max()
    capped = simulate(profile, MASS, controller, replace(drive, voltage_limit=1e6, current_limit=limit))
    assert capped.current_saturated > 0.0
    assert np.abs(capped.current).max() <= limit * 1.02


def test_stiffness_error_scales_the_true_torque_through_the_deflection_sensor(setup) -> None:
    # The torque is measured as k_nominal * deflection. A spring 20 % softer than assumed is
    # deflected 25 % more for the same torque: the loop makes the measurement follow the
    # reference, and the true torque is 0.8 of the measurement.
    _, _, p, controller, drive = setup
    profile = synthetic_profile(MASS)
    soft = replace(p, stiffness=0.8 * p.stiffness)
    sim = simulate(profile, MASS, controller, unlimited(drive), actual=soft)
    np.testing.assert_allclose(sim.torque, 0.8 * sim.measured, atol=1e-9)
    slope = np.dot(sim.measured, sim.reference) / np.dot(sim.reference, sim.reference)
    assert slope == pytest.approx(1.0, abs=0.03)
    assert sim.rms_error_pct() > 5.0
    cell = simulate(profile, MASS, controller, unlimited(drive), actual=soft,
                    options=SimOptions(torque_sensor="load_cell"))
    assert cell.rms_error_pct() < 2.0
