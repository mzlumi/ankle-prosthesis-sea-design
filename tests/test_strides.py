from pathlib import Path

import numpy as np
import pytest
from synthetic import synthetic_ankle, write_trial

from anklesea import camargo, profiles, strides


def _levelground(raw_dir: Path, subject: str = "AB99", labels=None, nan_start: float = 0.0, stride_time: float = 1.1, mass: float = 70.0):
    stem = "levelground_ccw_normal_01_01"
    write_trial(raw_dir, subject, "d", stem, duration=8.0, stride_time=stride_time, mass=mass, nan_start=nan_start, labels=labels or ["walk"])
    return camargo.load_trial(subject, "d", stem, raw_dir=raw_dir)


def test_heel_strike_indices() -> None:
    phase = np.mod(np.arange(0, 500), 100).astype(float)
    np.testing.assert_array_equal(strides.heel_strike_indices(phase), [100, 200, 300, 400])


def test_cut_strides_recovers_the_synthetic_cycle(raw_dir: Path) -> None:
    trial = _levelground(raw_dir)
    cut = strides.cut_strides(trial, body_mass=70.0)
    # Heel strikes at 0.2 s + k * 1.1 s up to 7.9 s in an 8 s trial: 7 complete strides.
    assert len(cut) == 7
    angle_deg, moment = synthetic_ankle(strides.GAIT_PCT, 70.0)
    for s in cut:
        assert s.activity == "levelground" and s.condition == "normal"
        assert s.duration == pytest.approx(1.1, abs=0.006)
        np.testing.assert_allclose(np.rad2deg(s.angle), angle_deg, atol=0.6)  # heel strike falls on a 5 ms grid
        np.testing.assert_allclose(s.moment_per_kg, moment / 70.0, atol=0.02)


def test_strides_with_nan_or_mixed_labels_are_dropped(raw_dir: Path) -> None:
    trial = _levelground(raw_dir, nan_start=1.5)
    assert len(strides.cut_strides(trial, 70.0)) == 5
    trial.labels[600:620] = "stand"
    assert len(strides.cut_strides(trial, 70.0)) == 4


def test_filter_durations_drops_outliers() -> None:
    base = dict(subject="AB99", trial="t", activity="levelground", condition="normal", start_time=0.0,
                angle=np.zeros(101), moment=np.zeros(101), moment_per_kg=np.zeros(101))
    cut = [strides.Stride(duration=d, **base) for d in (1.0, 1.02, 0.98, 1.01, 1.6)]
    kept = strides.filter_durations(cut)
    assert [s.duration for s in kept] == [1.0, 1.02, 0.98, 1.01]


def test_average_profiles_normalizes_by_mass(raw_dir: Path) -> None:
    cut = []
    for subject, mass in (("AB98", 60.0), ("AB99", 90.0)):
        trial = _levelground(raw_dir, subject=subject, mass=mass)
        cut += strides.cut_strides(trial, body_mass=mass)
    table = strides.average_profiles(strides.strides_to_frame(cut))
    assert set(table["n_subjects"]) == {2}
    assert set(table["n_strides"]) == {14}
    np.testing.assert_allclose(table["moment_sd_Nm_per_kg"], 0.0, atol=1e-9)
    _, moment = synthetic_ankle(strides.GAIT_PCT, 1.0)
    np.testing.assert_allclose(table["moment_mean_Nm_per_kg"], moment, atol=0.02)
    assert table["stride_time_mean_s"].iloc[0] == pytest.approx(1.1, abs=0.006)


def test_fourier_load_derivatives_of_a_sinusoid() -> None:
    period = 1.2
    s = np.linspace(0, 1, 101)
    angle = 0.2 * np.sin(2 * np.pi * s) + 0.05
    torque = -50.0 * np.cos(4 * np.pi * s)
    load = profiles.load_from_cycle(angle, torque, period, n_samples=600)
    w = 2 * np.pi / period
    np.testing.assert_allclose(load.angle, 0.2 * np.sin(w * load.t) + 0.05, atol=1e-9)
    np.testing.assert_allclose(load.velocity, 0.2 * w * np.cos(w * load.t), atol=1e-9)
    np.testing.assert_allclose(load.acceleration, -0.2 * w**2 * np.sin(w * load.t), atol=1e-9)
    np.testing.assert_allclose(load.torque_rate, 50.0 * 2 * w * np.sin(2 * w * load.t), atol=1e-7)
    np.testing.assert_allclose(load.torque_accel, 50.0 * 4 * w**2 * np.cos(2 * w * load.t), atol=1e-6)
    assert load.dt == pytest.approx(period / 600)


def test_profile_load_scales_with_mass() -> None:
    gp = profiles.GaitProfile("levelground", "normal", strides.GAIT_PCT, np.zeros(101),
                              np.sin(np.linspace(0, 2 * np.pi, 101)), stride_time=1.0)
    assert np.max(np.abs(gp.load(80.0).torque)) == pytest.approx(80.0, rel=1e-3)
