import numpy as np
import pytest

from anklesea.motor import default_motor
from anklesea.sea import Design, Limits, evaluate_design
from anklesea.sizing import sweep
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
