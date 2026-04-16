import numpy as np
import pytest
import yaml

from anklesea.motor import DEFAULT_MOTOR_FILE, GCM2, RPM, Motor, default_motor


@pytest.fixture(scope="module")
def motor() -> Motor:
    return default_motor()


def test_si_conversion(motor: Motor) -> None:
    assert motor.torque_constant == pytest.approx(0.0206)
    assert motor.resistance == pytest.approx(0.21)
    assert motor.inductance == pytest.approx(0.037e-3)
    assert motor.rotor_inertia == pytest.approx(33.3 * GCM2)
    assert motor.inertia == pytest.approx(34.0e-7)
    assert motor.max_speed == pytest.approx(25000 * RPM)
    assert motor.gear_efficiency == pytest.approx(0.70)
    assert motor.peak_current == 30
    assert motor.continuous_current == pytest.approx(5.06)


def test_stall_torque_and_current_match_datasheet(motor: Motor) -> None:
    assert motor.stall_torque() == pytest.approx(motor.datasheet_stall_torque, rel=0.01)
    assert motor.stall_current() == pytest.approx(motor.datasheet_stall_current, rel=0.01)


def test_no_load_speed_matches_datasheet(motor: Motor) -> None:
    assert motor.no_load_speed() == pytest.approx(motor.datasheet_no_load_speed, rel=0.01)


def test_speed_constant_is_inverse_torque_constant(motor: Motor) -> None:
    spec = yaml.safe_load(DEFAULT_MOTOR_FILE.read_text())["motor"]["values"]
    speed_constant = spec["speed_constant_rpm_per_V"] * RPM  # rad/s per V
    assert 1.0 / motor.torque_constant == pytest.approx(speed_constant, rel=0.01)


def test_gradient_and_time_constant_match_datasheet(motor: Motor) -> None:
    assert motor.speed_torque_gradient == pytest.approx(motor.datasheet_speed_torque_gradient, rel=0.01)
    assert motor.mechanical_time_constant == pytest.approx(motor.datasheet_mechanical_time_constant, rel=0.02)


def test_viscous_friction_from_no_load_point(motor: Motor) -> None:
    friction_torque = motor.viscous_friction * motor.datasheet_no_load_speed
    assert friction_torque == pytest.approx(0.0206 * 0.482)
    assert 0 < motor.viscous_friction < 1e-5


def test_nominal_point_is_thermally_continuous(motor: Motor) -> None:
    """At the nominal (max. continuous) current the winding stays below its limit."""
    assert motor.winding_temperature(motor.continuous_current) < motor.max_winding_temperature
    assert np.isfinite(motor.motor_constant)


def test_available_voltage(motor: Motor) -> None:
    assert motor.available_voltage(36.0) == pytest.approx(34.2)
