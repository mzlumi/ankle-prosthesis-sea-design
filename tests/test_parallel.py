import numpy as np
import pytest
from scipy.optimize import minimize

from anklesea.motor import default_motor
from anklesea.parallel import (
    ParallelSpring,
    actuator_load,
    best_spring,
    joint_level_spring,
    optimize_parallel,
)
from anklesea.profiles import Load
from anklesea.sea import Limits, evaluate, motor_trajectory
from anklesea.sizing import optimize
from synthetic import sinusoid_load


@pytest.fixture(scope="module")
def motor():
    return default_motor()


def stance_like_load() -> Load:
    """Torque that is mostly a spring about 0.1 rad plus a smaller out-of-phase part."""
    base = sinusoid_load(a=0.25, b=0.0, phase=0.0)
    spring = -300.0 * (base.angle - 0.1)
    extra = sinusoid_load(a=0.25, b=25.0, phase=2.6)
    return Load(
        t=base.t, angle=base.angle, velocity=base.velocity, acceleration=base.acceleration,
        torque=spring + extra.torque, torque_rate=-300.0 * base.velocity + extra.torque_rate,
        torque_accel=-300.0 * base.acceleration + extra.torque_accel, period=base.period,
    )


def test_zero_parallel_spring_reproduces_the_sea(motor) -> None:
    load = sinusoid_load(a=0.25, b=60.0, phase=2.6)
    limits = Limits.from_motor(motor, 36.0)
    same = actuator_load(load, ParallelSpring(0.0, 0.0))
    for field in ("torque", "torque_rate", "torque_accel"):
        assert np.array_equal(getattr(same, field), getattr(load, field))
    a = evaluate(load, 1 / 400.0, 300.0, motor, limits)
    b = evaluate(same, 1 / 400.0, 300.0, motor, limits)
    assert all(np.array_equal(a[key], b[key]) for key in a)


def test_joint_level_fit_recovers_a_pure_spring() -> None:
    base = sinusoid_load(a=0.3, b=0.0, phase=0.0)
    load = Load(t=base.t, angle=base.angle, velocity=base.velocity, acceleration=base.acceleration,
                torque=-250.0 * (base.angle - 0.05), torque_rate=-250.0 * base.velocity,
                torque_accel=-250.0 * base.acceleration, period=base.period)
    spring = joint_level_spring(load)
    assert spring.stiffness == pytest.approx(250.0) and spring.rest_angle == pytest.approx(0.05)
    assert np.allclose(actuator_load(load, spring).torque, 0.0, atol=1e-9)


@pytest.mark.parametrize("objective", ["rms", "energy"])
def test_closed_form_spring_matches_numerical_minimum(motor, objective) -> None:
    load = stance_like_load()
    alpha, ratio = 1 / 500.0, 250.0
    limits = Limits.from_motor(motor, 1e6)

    def cost(x: np.ndarray) -> float:
        act = actuator_load(load, ParallelSpring(*x))
        if objective == "rms":
            return float(np.mean(motor_trajectory(act, alpha, ratio, motor)["motor_torque"] ** 2))
        return float(evaluate(act, alpha, ratio, motor, limits)["energy"])

    closed = best_spring(load, alpha, ratio, motor, objective)
    numeric = minimize(cost, x0=[0.0, 0.0], method="Nelder-Mead",
                       options={"xatol": 1e-6, "fatol": 1e-14, "maxiter": 4000}).x
    assert cost([closed.stiffness, closed.preload]) <= cost(numeric) * (1 + 1e-9) + 1e-12
    assert closed.stiffness == pytest.approx(numeric[0], rel=1e-3)


def test_parallel_spring_lowers_rms_current_and_meets_the_thermal_limit(motor) -> None:
    load = stance_like_load()
    limits = Limits(bus_voltage=36.0, peak_current=30.0, rms_current=2.0, max_speed=2600.0, available_voltage=34.2)
    ratios = np.geomspace(60.0, 800.0, 40)
    sea = optimize(load, motor, limits, ratios=ratios)
    assert sea["drive_feasible"] and sea["rms_current"] > limits.rms_current
    assert not optimize(load, motor, limits, ratios=ratios, thermal=True)["feasible"]
    both = optimize_parallel(load, motor, limits, ratios=ratios)
    assert both["feasible"] and both["rms_current"] <= limits.rms_current * (1 + 1e-9)
    # The out-of-phase part has an in-phase component, so the spring is near the joint-level fit (about 386 N·m/rad), not 300.
    assert both["spring"].stiffness == pytest.approx(joint_level_spring(load).stiffness, rel=0.1)
