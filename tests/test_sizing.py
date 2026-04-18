import numpy as np
import pytest

from anklesea.motor import default_motor
from anklesea.sea import Design, Limits, evaluate_design
from anklesea.sizing import constrained_optimum, feasible_compliance, quadratic_in_compliance, sweep
from synthetic import sinusoid_load


@pytest.fixture(scope="module")
def motor():
    return default_motor()


@pytest.fixture(scope="module")
def result(motor):
    load = sinusoid_load(a=0.25, b=100.0, phase=2.6)
    limits = Limits.from_motor(motor, bus_voltage=36.0)
    stiffness = np.concatenate([np.geomspace(100.0, 5000.0, 25), [np.inf]])
    ratios = np.geomspace(30.0, 600.0, 20)
    return load, limits, sweep(load, motor, limits, stiffness, ratios)


def test_sweep_matches_single_design_evaluation(motor, result) -> None:
    load, limits, res = result
    assert res.energy.shape == (20, 26)
    for i, j in [(0, 0), (7, 12), (19, 25)]:
        single = evaluate_design(load, Design(res.stiffness[j], res.ratios[i]), motor, limits)
        assert res.energy[i, j] == pytest.approx(single["energy"], rel=1e-12)
        assert res.metrics["drive_feasible"][i, j] == single["drive_feasible"]


def test_optimum_is_the_least_energy_allowed_design(result) -> None:
    _, _, res = result
    for constraint in [None, "drive_feasible"]:
        opt = res.optimum(constraint=constraint)
        allowed = res.mask(constraint)
        assert allowed[opt["i"], opt["j"]]
        assert opt["energy"] == pytest.approx(res.energy[allowed].min())
    assert res.optimum(constraint=None)["energy"] <= res.optimum(constraint="drive_feasible")["energy"]


def test_optimum_reports_none_when_nothing_is_allowed(result) -> None:
    _, _, res = result
    res.metrics["nothing"] = np.zeros_like(res.feasible)
    opt = res.optimum(constraint="nothing")
    assert np.isnan(opt["ratio"]) and not opt["feasible"]
    del res.metrics["nothing"]


def test_best_per_ratio_picks_row_minimum(result) -> None:
    _, _, res = result
    idx = res.best_per_ratio()
    assert np.array_equal(idx, np.argmin(res.energy, axis=1))


@pytest.mark.parametrize("ratio", [50.0, 150.0, 400.0])
def test_quadratic_reproduces_full_model(motor, ratio) -> None:
    load = sinusoid_load(a=0.25, b=100.0, phase=2.6)
    loose = Limits.from_motor(motor, bus_voltage=1e6)
    quad = quadratic_in_compliance(load, ratio, motor)
    for alpha in [0.0, 1e-4, 1e-3, 5e-3, 2e-2]:
        k = np.inf if alpha == 0.0 else 1.0 / alpha
        full = evaluate_design(load, Design(k, ratio), motor, loose)["energy"]
        assert quad.energy(alpha) == pytest.approx(full, rel=1e-9, abs=1e-9)


def test_quadratic_is_convex_and_its_minimum_matches_a_fine_search(motor) -> None:
    load = sinusoid_load(a=0.25, b=100.0, phase=2.6)
    quad = quadratic_in_compliance(load, 150.0, motor)
    assert quad.a > 0.0 and quad.b < 0.0
    alphas = np.linspace(0.0, 4.0 * quad.optimal_compliance, 200001)
    best = alphas[np.argmin(quad.energy(alphas))]
    assert best == pytest.approx(quad.optimal_compliance, rel=1e-4)
    lo, hi = quad.savings_region
    assert lo == 0.0 and quad.energy(hi) == pytest.approx(quad.c, rel=1e-9)
    assert quad.energy(0.5 * hi) < quad.c < quad.energy(1.01 * hi)


def test_massless_frictionless_motor_gains_nothing_from_the_spring(motor) -> None:
    # With J = 0 and b = 0 the motor torque tau / (N eta) does not depend on the spring, and
    # the mechanical power integrates to the joint work over eta for any compliance: a = b = 0.
    ideal = motor.with_changes(rotor_inertia=0.0, gear_inertia=0.0, viscous_friction=0.0)
    load = sinusoid_load(a=0.25, b=100.0, phase=2.6)
    quad = quadratic_in_compliance(load, 150.0, ideal)
    assert quad.a == pytest.approx(0.0, abs=1e-9) and quad.b == pytest.approx(0.0, abs=1e-6)
    assert np.isinf(quad.optimal_stiffness)


@pytest.mark.parametrize("ratio", [120.0, 250.0, 500.0])
@pytest.mark.parametrize("thermal", [False, True])
def test_feasible_interval_matches_full_model(motor, ratio, thermal) -> None:
    load = sinusoid_load(a=0.25, b=100.0, phase=2.6)
    limits = Limits(bus_voltage=24.0, peak_current=8.0, rms_current=4.5, max_speed=1500.0, available_voltage=22.8)
    lo, hi = feasible_compliance(load, ratio, motor, limits, thermal=thermal)
    key = "feasible" if thermal else "drive_feasible"
    alphas = np.linspace(0.0, 0.03, 3001)
    from anklesea.sea import evaluate

    ok = evaluate(load, alphas, np.full_like(alphas, ratio), motor, limits)[key]
    if np.isnan(lo):
        assert not ok.any()
    else:
        inside = (alphas >= lo) & (alphas <= hi)
        mismatch = ok != inside
        # Only grid points within one step of an end of the interval may disagree.
        step = alphas[1] - alphas[0]
        near = (np.abs(alphas - lo) <= step) | (np.abs(alphas - hi) <= step)
        assert not np.any(mismatch & ~near)
        assert ok.any()


def test_constrained_optimum_clips_into_the_interval(motor) -> None:
    load = sinusoid_load(a=0.25, b=100.0, phase=2.6)
    none = Limits(bus_voltage=1e6, peak_current=1e6, rms_current=1e6, max_speed=1e6, available_voltage=1e6)
    quad = quadratic_in_compliance(load, 150.0, motor)
    lo, hi = feasible_compliance(load, 150.0, motor, none)
    assert lo == 0.0 and hi > 100.0 * quad.optimal_compliance
    alpha, energy = constrained_optimum(load, 150.0, motor, none)
    assert alpha == pytest.approx(quad.optimal_compliance) and energy == pytest.approx(quad.optimal_energy)
    # A speed limit that the unconstrained optimum breaks moves the optimum to the interval's end.
    speed = evaluate_design(load, Design(quad.optimal_stiffness, 150.0), motor, none)["peak_speed"]
    capped = Limits(bus_voltage=1e6, peak_current=1e6, rms_current=1e6, max_speed=0.8 * speed, available_voltage=1e6)
    lo, hi = feasible_compliance(load, 150.0, motor, capped)
    alpha_c, energy_c = constrained_optimum(load, 150.0, motor, capped)
    assert alpha_c in (pytest.approx(lo), pytest.approx(hi)) and energy_c > quad.optimal_energy
