import numpy as np
import pytest

from anklesea.motor import default_motor
from anklesea.profiles import Load
from anklesea.sea import Design, Limits, evaluate, evaluate_design, joint_work, motor_trajectory
from synthetic import PERIOD, W, sinusoid_load

@pytest.fixture(scope="module")
def motor():
    return default_motor()


@pytest.fixture(scope="module")
def loose_limits(motor):
    return Limits.from_motor(motor, bus_voltage=1e6)


def phasor_energy(a, b, phase, k, n, motor) -> float:
    """Closed-form energy per period for sinusoidal angle and torque.

    With complex amplitudes Theta = a, T = b e^{j phase}: Theta_m = N (Theta + T / k),
    Omega_m = j w Theta_m, T_m = (-J w^2 + j w b_m) Theta_m + T / (N eta). Over one
    period the integral of x(t) y(t) is (P / 2) Re(X conj(Y)), so
    E = (P / 2) [R / k_t^2 |T_m|^2 + Re(T_m conj(Omega_m))].
    """
    theta = a + 0j
    torque = b * np.exp(1j * phase)
    theta_m = n * (theta + (0 if np.isinf(k) else torque / k))
    omega_m = 1j * W * theta_m
    t_m = (-motor.inertia * W**2 + 1j * W * motor.viscous_friction) * theta_m + torque / (n * motor.gear_efficiency)
    return PERIOD / 2 * (motor.resistance / motor.torque_constant**2 * abs(t_m) ** 2 + (t_m * np.conj(omega_m)).real)


@pytest.mark.parametrize("k", [np.inf, 300.0, 600.0, 1500.0])
@pytest.mark.parametrize("n", [60.0, 150.0])
def test_sinusoid_matches_closed_form(motor, loose_limits, k, n) -> None:
    load = sinusoid_load()
    result = evaluate_design(load, Design(k, n), motor, loose_limits)
    assert result["energy"] == pytest.approx(phasor_energy(0.2, 80.0, 0.7, k, n, motor), rel=1e-6)


def test_rigid_limit(motor, loose_limits) -> None:
    load = sinusoid_load()
    rigid = evaluate_design(load, Design(np.inf, 120.0), motor, loose_limits)
    energies = [evaluate_design(load, Design(k, 120.0), motor, loose_limits)["energy"] for k in (1e4, 1e5, 1e6, 1e7)]
    errors = np.abs(np.array(energies) - rigid["energy"])
    assert np.all(np.diff(errors) < 0)
    assert errors[-1] < 1e-4 * abs(rigid["energy"])
    traj = motor_trajectory(load, 0.0, 120.0, motor)
    np.testing.assert_allclose(traj["motor_angle"], 120.0 * load.angle)


def test_zero_inertia_rigid_energy_is_joint_work_plus_copper_loss(motor, loose_limits) -> None:
    """With no inertia, no friction and eta = 1, motor mechanical work equals joint work."""
    ideal = motor.with_changes(rotor_inertia=0.0, gear_inertia=0.0, viscous_friction=0.0, gear_efficiency=1.0, inductance=0.0)
    load = sinusoid_load(phase=0.3)
    n = 100.0
    result = evaluate_design(load, Design(np.inf, n), ideal, loose_limits)
    copper = ideal.resistance / ideal.torque_constant**2 * (80.0 / n) ** 2 * PERIOD / 2
    assert result["copper_energy"] == pytest.approx(copper, rel=1e-6)
    assert result["energy"] == pytest.approx(joint_work(load)["net"] + copper, rel=1e-6)
    # Net joint work of the sinusoid: (P / 2) a b w sin(phase)... with velocity a w cos(wt).
    assert joint_work(load)["net"] == pytest.approx(PERIOD / 2 * 0.2 * W * 80.0 * np.sin(0.3), rel=1e-6)


def test_spring_matched_to_a_springlike_load_stops_the_motor(motor, loose_limits) -> None:
    """If the joint needs tau = -K theta, a series spring k = K leaves the motor still.

    Then the only energy is the copper loss of holding the torque, the floor that no
    spring can remove.
    """
    stiffness = 400.0
    load = sinusoid_load(a=0.2, b=0.2 * stiffness, phase=np.pi)
    traj = motor_trajectory(load, 1.0 / stiffness, 100.0, motor)
    np.testing.assert_allclose(traj["motor_speed"], 0.0, atol=1e-9)
    result = evaluate_design(load, Design(stiffness, 100.0), motor, loose_limits)
    assert result["energy"] == pytest.approx(result["copper_energy"], rel=1e-9)
    rigid = evaluate_design(load, Design(np.inf, 100.0), motor, loose_limits)
    assert result["energy"] < rigid["energy"]


def test_regeneration_bounds_and_positive_power_case(motor, loose_limits) -> None:
    load = sinusoid_load()
    result = evaluate_design(load, Design(500.0, 100.0), motor, loose_limits)
    assert result["energy"] <= result["energy_no_regen"]
    assert result["energy_no_regen"] >= result["copper_energy"]
    # A held constant torque with no motion draws only positive (copper) power.
    t = np.arange(1000) * 0.001
    still = Load(t, 0 * t, 0 * t, 0 * t, 50.0 + 0 * t, 0 * t, 0 * t, 1.0)
    held = evaluate_design(still, Design(500.0, 100.0), motor, loose_limits)
    assert held["energy"] == pytest.approx(held["energy_no_regen"])
    assert held["peak_current"] == pytest.approx(50.0 / (100.0 * motor.gear_efficiency * motor.torque_constant))


def test_constraints_flag_infeasible_designs(motor) -> None:
    load = sinusoid_load(a=0.4, b=120.0)
    limits = Limits.from_motor(motor, bus_voltage=36.0)
    slow_big_torque = evaluate_design(load, Design(np.inf, 5.0), motor, limits)
    assert not slow_big_torque["ok_peak_current"]
    assert not slow_big_torque["drive_feasible"] and not slow_big_torque["feasible"]
    fast = evaluate_design(load, Design(np.inf, 3000.0), motor, limits)
    assert not fast["ok_voltage"] and not fast["feasible"]


def test_thermal_limit_is_separate_from_drive_limits(motor) -> None:
    load = sinusoid_load(a=0.05, b=8.0)
    tight = Limits(bus_voltage=36.0, peak_current=30.0, rms_current=1e-3, max_speed=motor.max_speed,
                   available_voltage=34.2)
    result = evaluate_design(load, Design(500.0, 100.0), motor, tight)
    assert result["drive_feasible"] and not result["ok_rms_current"] and not result["feasible"]


def test_grid_broadcast_matches_single_designs(motor, loose_limits) -> None:
    load = sinusoid_load()
    k = np.array([[300.0, 900.0], [300.0, 900.0]])
    n = np.array([[80.0, 80.0], [160.0, 160.0]])
    grid = evaluate(load, 1 / k, n, motor, loose_limits)
    for idx in np.ndindex(k.shape):
        single = evaluate_design(load, Design(k[idx], n[idx]), motor, loose_limits)
        assert grid["energy"][idx] == pytest.approx(single["energy"])
        assert grid["peak_voltage"][idx] == pytest.approx(single["peak_voltage"])


def test_directional_efficiency_never_costs_more(motor, loose_limits) -> None:
    load = sinusoid_load()
    const = evaluate_design(load, Design(500.0, 100.0), motor, loose_limits)
    direc = evaluate_design(load, Design(500.0, 100.0), motor, loose_limits, efficiency_model="directional")
    assert direc["energy"] <= const["energy"]
