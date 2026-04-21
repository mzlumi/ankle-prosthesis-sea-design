import numpy as np
import pytest

from anklesea.motor import default_motor
from anklesea.sea import Design, Limits, evaluate, evaluate_design
from anklesea.sizing import (
    constrained_optimum,
    feasible_compliance,
    optimize,
    optimize_mix,
    quadratic_in_compliance,
    sweep,
)
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


LIGHT = dict(a=0.25, b=60.0, phase=2.6)
MID_LIMITS = Limits(bus_voltage=36.0, peak_current=20.0, rms_current=7.0, max_speed=2600.0, available_voltage=30.0)


@pytest.mark.parametrize("ratio", [250.0, 350.0, 500.0])
@pytest.mark.parametrize("thermal", [False, True])
def test_feasible_interval_matches_full_model(motor, ratio, thermal) -> None:
    load = sinusoid_load(**LIGHT)
    lo, hi = feasible_compliance(load, ratio, motor, MID_LIMITS, thermal=thermal)
    if not thermal or ratio == 500.0:
        assert 0.0 <= lo < hi < np.inf
    key = "feasible" if thermal else "drive_feasible"
    alphas = np.linspace(0.0, 0.03, 3001)
    ok = evaluate(load, alphas, np.full_like(alphas, ratio), motor, MID_LIMITS)[key]
    if np.isnan(lo):
        assert not ok.any()
    else:
        inside = (alphas >= lo) & (alphas <= hi)
        # Only grid points within one step of an end of the interval may disagree.
        step = alphas[1] - alphas[0]
        near = (np.abs(alphas - lo) <= step) | (np.abs(alphas - hi) <= step)
        assert not np.any((ok != inside) & ~near)


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


def test_optimize_matches_or_beats_the_grid(motor) -> None:
    load = sinusoid_load(**LIGHT)
    ratios = np.geomspace(100.0, 800.0, 200)
    grid = sweep(load, motor, MID_LIMITS, np.concatenate([np.geomspace(30.0, 20000.0, 200), [np.inf]]), ratios)
    for thermal, key in [(False, "drive_feasible"), (True, "feasible")]:
        best = optimize(load, motor, MID_LIMITS, ratios=ratios, thermal=thermal)
        ref = grid.optimum(constraint=key)
        assert best[key]
        assert best["energy"] <= ref["energy"] * (1 + 1e-9)
        assert best["energy"] == pytest.approx(ref["energy"], rel=0.01)


def test_optimum_on_a_limit_passes_the_full_model_check(motor) -> None:
    # The voltage-limited optimum lies on the boundary; round-off must not flag it.
    load = sinusoid_load(**LIGHT)
    limits = Limits(bus_voltage=36.0, peak_current=40.0, rms_current=40.0, max_speed=5000.0, available_voltage=12.0)
    for n in np.geomspace(150.0, 700.0, 40):
        alpha, _ = constrained_optimum(load, n, motor, limits)
        if np.isfinite(alpha):
            k = np.inf if alpha == 0.0 else 1.0 / alpha
            assert evaluate_design(load, Design(k, n), motor, limits)["drive_feasible"]


def test_optimize_reports_no_design_when_limits_cannot_be_met(motor) -> None:
    load = sinusoid_load(a=0.25, b=100.0, phase=2.6)
    impossible = Limits(bus_voltage=1.0, peak_current=0.1, rms_current=0.1, max_speed=10.0, available_voltage=1.0)
    best = optimize(load, motor, impossible, ratios=np.geomspace(30.0, 600.0, 20))
    assert np.isnan(best["ratio"]) and not best["feasible"]


def test_mix_of_one_activity_is_the_single_optimum(motor) -> None:
    load = sinusoid_load(**LIGHT)
    ratios = np.geomspace(100.0, 800.0, 80)
    single = optimize(load, motor, MID_LIMITS, ratios=ratios)
    mix = optimize_mix([load, load], [0.3, 0.7], motor, MID_LIMITS, ratios=ratios)
    assert mix["ratio"] == single["ratio"]
    assert mix["stiffness"] == pytest.approx(single["stiffness"])
    assert mix["weighted_energy"] == pytest.approx(single["energy"], rel=1e-9)


def test_mix_matches_brute_force_and_is_feasible_in_every_activity(motor) -> None:
    # The second load is faster and lighter: infeasible at high ratios (voltage), unlike the first.
    loads = [sinusoid_load(**LIGHT), sinusoid_load(a=0.4, b=30.0, phase=2.0)]
    weights = [0.8, 0.2]
    ratios = np.geomspace(150.0, 800.0, 15)
    alone = [optimize(load, motor, MID_LIMITS, ratios=ratios) for load in loads]
    assert not evaluate_design(loads[1], Design(alone[0]["stiffness"], alone[0]["ratio"]), motor,
                               MID_LIMITS)["drive_feasible"]
    mix = optimize_mix(loads, weights, motor, MID_LIMITS, ratios=ratios)
    alphas = np.linspace(0.0, 0.03, 3001)
    best = np.inf
    for n in ratios:
        results = [evaluate(load, alphas, np.full_like(alphas, n), motor, MID_LIMITS) for load in loads]
        ok = results[0]["drive_feasible"] & results[1]["drive_feasible"]
        total = weights[0] * results[0]["energy"] + weights[1] * results[1]["energy"]
        best = min(best, float(np.where(ok, total, np.inf).min()))
    assert mix["weighted_energy"] <= best * (1 + 1e-9)
    assert mix["weighted_energy"] == pytest.approx(best, rel=1e-3)
    for load in loads:
        assert evaluate_design(load, Design(mix["stiffness"], mix["ratio"]), motor, MID_LIMITS)["drive_feasible"]


def test_mix_conditions_pick_the_source_geometry() -> None:
    from types import SimpleNamespace

    from anklesea.sizing import mix_conditions

    profiles = {("treadmill", "1.20 m/s"): SimpleNamespace(n_subjects=10)}
    for activity in ("rampascent", "rampdescent"):
        for cond, n in [("5.2 deg", 9), ("7.8 deg", 9), ("18.0 deg", 9), ("4.0 deg", 1)]:
            profiles[(activity, cond)] = SimpleNamespace(n_subjects=n)
    for activity in ("stairascent", "stairdescent"):
        for cond, n in [("4 in", 8), ("6 in", 8), ("7 in", 7), ("8 in", 2)]:
            profiles[(activity, cond)] = SimpleNamespace(n_subjects=n)
    picked = mix_conditions(profiles)
    assert picked["rampascent"] == ("rampascent", "5.2 deg") and picked["stairdescent"] == ("stairdescent", "7 in")
    low = mix_conditions(profiles, stair="lowest", ramp="lowest")
    assert low["rampdescent"][1] == "5.2 deg" and low["stairascent"][1] == "4 in"
    high = mix_conditions(profiles, stair="highest", ramp="highest")
    assert high["rampascent"][1] == "18.0 deg" and high["stairascent"][1] == "7 in"
