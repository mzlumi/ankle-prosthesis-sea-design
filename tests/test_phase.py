import numpy as np

from anklesea.phase import PHASE_CAP, command_error_pct, periodic_lookup, phase_error, time_based_phase


def test_phase_error_wraps_around_heel_strike():
    assert np.isclose(phase_error(0.99, 0.01), -0.02)
    assert np.isclose(phase_error(0.01, 0.99), 0.02)
    assert np.isclose(phase_error(0.3, 0.1), 0.2)


def test_periodic_lookup_interpolates_and_clips():
    curve = np.linspace(0.0, 10.0, 101)
    assert np.allclose(periodic_lookup(curve, np.array([0.0, 0.25, 1.0, 1.2])), [0.0, 2.5, 10.0, 10.0])


def test_steady_strides_give_the_true_phase():
    strikes = np.arange(0.0, 6.0, 1.1)
    t = np.arange(2.2025, 5.5, 0.005)
    est = time_based_phase(t, strikes, default_period=0.5)
    truth = (t % 1.1) / 1.1
    assert np.max(np.abs(phase_error(est, truth))) < 1e-9


def test_first_stride_uses_the_default_period():
    t = np.array([0.0, 0.25, 0.5])
    est = time_based_phase(t, np.array([0.0]), default_period=1.0)
    assert np.allclose(est, [0.0, 0.25, 0.5])
    assert np.isnan(time_based_phase(np.array([-0.1]), np.array([0.0]), 1.0)[0])


def test_slower_stride_holds_below_one_and_faster_stride_resets_early():
    strikes = np.array([0.0, 1.0, 2.5, 3.0])  # a 1.5 s stride after a 1.0 s one
    est = time_based_phase(np.array([1.5, 2.2, 2.6]), strikes, default_period=1.0)
    assert np.isclose(est[0], 0.5)
    assert est[1] == PHASE_CAP
    assert np.isclose(est[2], 0.1 / 1.5)


def test_detection_delay_shifts_the_phase_by_delay_over_period():
    strikes = np.arange(0.0, 6.0, 1.0)
    t = np.arange(2.305, 4.9, 0.01)
    est = time_based_phase(t, strikes, default_period=1.0, detection_delay=0.05)
    assert np.allclose(phase_error(est, t % 1.0), -0.05, atol=1e-9)


def test_command_error_is_relative_to_the_peak():
    actual = np.array([0.0, -2.0, -1.0, 0.0])
    assert np.isclose(command_error_pct(actual + 0.2, actual), 10.0)
