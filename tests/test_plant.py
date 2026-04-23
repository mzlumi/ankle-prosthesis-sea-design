import control as ct
import numpy as np
import pytest

from anklesea.motor import default_motor
from anklesea.plant import (
    SEAParams,
    fixed_output,
    free_output,
    joint_motion_to_torque,
    peak_frequency,
)
from anklesea.sea import Design


@pytest.fixture(scope="module")
def params():
    return SEAParams.from_design(Design(300.0, 400.0), default_motor())


def test_fixed_output_dc_gain_is_ratio_times_efficiency(params) -> None:
    assert ct.dcgain(fixed_output(params)) == pytest.approx(params.ratio * params.efficiency, rel=1e-9)


def test_fixed_output_resonance_is_spring_against_reflected_inertia(params) -> None:
    expected = np.sqrt(params.stiffness / (params.efficiency * params.rotor_inertia * params.ratio**2))
    assert params.fixed_resonance == pytest.approx(expected)
    # Light damping: the magnitude peak sits at the natural frequency.
    assert peak_frequency(fixed_output(params)) == pytest.approx(expected, rel=0.01)


def test_free_output_has_zero_dc_gain_and_resonance_above_fixed(params) -> None:
    free = free_output(params)
    assert np.min(np.abs(ct.zeros(free))) < 1e-9  # a zero at s = 0: no steady torque on a free foot
    assert peak_frequency(free) == pytest.approx(params.free_resonance, rel=0.01)
    assert params.free_resonance > 3 * params.fixed_resonance


def test_joint_motion_disturbance_limits(params) -> None:
    dist = joint_motion_to_torque(params)
    assert abs(dist(1e-4j)) < 1e-3 * params.stiffness  # rotor follows slow joint motion
    assert abs(dist(1e5j)) == pytest.approx(params.stiffness, rel=1e-3)  # fast motion deflects the spring


def test_stiffer_spring_raises_the_resonance(params) -> None:
    stiff = SEAParams.from_design(Design(1200.0, 400.0), default_motor())
    assert stiff.fixed_resonance == pytest.approx(2.0 * params.fixed_resonance)
