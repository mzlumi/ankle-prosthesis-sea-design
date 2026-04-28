from pathlib import Path

import numpy as np
import pytest
from conftest import REAL_SUBJECT, requires_data
from synthetic import write_subject_info, write_trial

from anklesea import camargo


@pytest.mark.parametrize(
    ("stem", "mode", "condition", "leg", "turn", "block", "index"),
    [
        ("treadmill_03_01", "treadmill", "", None, None, 3, 1),
        ("levelground_ccw_normal_02_01", "levelground", "normal", None, "ccw", 2, 1),
        ("levelground_cw_slow_05_01.mat", "levelground", "slow", None, "cw", 5, 1),
        ("ramp_6_r_01_02", "ramp", "6", "r", None, 1, 2),
        ("stair_2_l_02_03", "stair", "2", "l", None, 2, 3),
        ("treadmill_04_2_01", "treadmill", "", None, None, 4, 1),
        ("LevelGround_ccw_normal_01_02", "levelground", "normal", None, "ccw", 1, 2),
        ("Ramp_1_L_01_04", "ramp", "1", "l", None, 1, 4),
    ],
)
def test_parse_trial_name(stem, mode, condition, leg, turn, block, index) -> None:
    name = camargo.parse_trial_name(stem)
    assert (name.mode, name.condition, name.leg, name.turn, name.block, name.index) == (
        mode,
        condition,
        leg,
        turn,
        block,
        index,
    )


def test_parse_keeps_the_original_spelling_of_the_stem() -> None:
    assert camargo.parse_trial_name("Stair_2_R_01_03.mat").stem == "Stair_2_R_01_03"


def test_parse_repeated_trial() -> None:
    assert camargo.parse_trial_name("treadmill_04_2_01").repeat == 2
    assert camargo.parse_trial_name("treadmill_04_01").repeat == 1


@pytest.mark.parametrize("stem", ["walk_01_01", "stair_x_l_01_01", "levelground_normal_01_01", "ramp_1_l_01"])
def test_parse_trial_name_rejects_unknown(stem) -> None:
    with pytest.raises(ValueError):
        camargo.parse_trial_name(stem)


def test_load_synthetic_stair_trial(raw_dir: Path) -> None:
    written = write_trial(
        raw_dir,
        "AB99",
        "01_01_20",
        "stair_3_l_01_01",
        nan_start=0.5,
        labels=["idle", "stairascent"],
        conditions={"stairHeight": np.array([[6.0]])},
    )
    trial = camargo.load_trial("AB99", "01_01_20", "stair_3_l_01_01", raw_dir=raw_dir)
    assert trial.mode == "stair"
    assert len(trial) == len(written["time"])
    np.testing.assert_allclose(trial.angle["r"], np.deg2rad(written["angle_r_deg"]))
    np.testing.assert_allclose(trial.moment["l"], written["moment_l"])
    assert np.isnan(trial.moment["r"][:100]).all()
    assert not np.isnan(trial.moment["r"][100:]).any()
    np.testing.assert_allclose(trial.heel_strike_phase["r"], written["phase_r"])
    assert trial.stair_height_m == pytest.approx(6 * 0.0254)
    assert list(trial.labels[:2]) == ["idle", "stairascent"]
    assert trial.belt_speed is None


def test_load_aligns_on_header(raw_dir: Path) -> None:
    """Sensor tables that start one sample apart are joined on time, not on row number."""
    write_trial(raw_dir, "AB99", "d", "treadmill_01_01", duration=2.0)
    ik_path = raw_dir / "AB99" / "d" / "treadmill" / "ik" / "treadmill_01_01.mat"
    ik = camargo.read_table(ik_path).iloc[1:].reset_index(drop=True)
    from synthetic import write_table

    write_table(ik_path, ik)
    trial = camargo.load_trial("AB99", "d", "treadmill_01_01", raw_dir=raw_dir)
    assert len(trial) == len(ik)
    assert trial.time[0] == pytest.approx(ik["Header"].iloc[0])
    np.testing.assert_allclose(trial.angle["r"], np.deg2rad(ik["ankle_angle_r"].to_numpy()))


def test_treadmill_belt_speed_is_interpolated(raw_dir: Path) -> None:
    import pandas as pd

    t = np.arange(0, 20000) / 1000.0
    speed = pd.DataFrame({"Header": t, "Speed": np.where(t < 13.0, 0.5, 1.2)})
    write_trial(raw_dir, "AB99", "d", "treadmill_02_01", duration=6.0, t0=10.0, conditions={"speed": speed})
    trial = camargo.load_trial("AB99", "d", "treadmill_02_01", raw_dir=raw_dir)
    assert trial.labels is None
    assert trial.belt_speed[0] == pytest.approx(0.5)
    assert trial.belt_speed[-1] == pytest.approx(1.2)


def test_ramp_incline_from_name_when_missing(raw_dir: Path) -> None:
    write_trial(raw_dir, "AB99", "d", "ramp_4_r_01_01", duration=2.0)
    trial = camargo.load_trial("AB99", "d", "ramp_4_r_01_01", raw_dir=raw_dir)
    assert trial.ramp_incline_rad == pytest.approx(np.deg2rad(11.0))


def test_list_trials_and_subject_mass(raw_dir: Path) -> None:
    write_subject_info(raw_dir, {"AB98": 61.5, "AB99": 80.0})
    write_trial(raw_dir, "AB99", "d", "stair_1_r_01_01", duration=1.0)
    write_trial(raw_dir, "AB99", "d", "ramp_1_r_01_01", duration=1.0)
    (raw_dir / "AB99" / "d" / "ramp" / "gcLeft" / "ramp_1_r_01_01.mat").unlink()
    refs = camargo.list_trials(raw_dir)
    assert [r.stem for r in refs] == ["stair_1_r_01_01"]
    assert camargo.subject_mass("AB99", raw_dir=raw_dir) == pytest.approx(80.0)
    with pytest.raises(KeyError):
        camargo.subject_mass("AB01", raw_dir=raw_dir)


@requires_data
def test_real_treadmill_trial_loads() -> None:
    refs = camargo.list_trials(subjects=[REAL_SUBJECT], modes=["treadmill"])
    assert refs
    trial = camargo.load_trial(refs[0].subject, refs[0].date, refs[0].stem)
    assert len(trial) > 1000
    assert np.nanmax(np.abs(np.diff(trial.time))) == pytest.approx(1 / camargo.SAMPLE_RATE_HZ, abs=1e-6)
    # Ankle angle in rad: well inside +-1 rad for walking.
    assert np.nanmax(np.abs(trial.angle["r"])) < 1.0
    # Moment in N·m, not normalized: peak plantarflexion moment of an adult is tens of N·m or more.
    assert np.nanmin(trial.moment["r"]) < -50.0
    assert trial.belt_speed is not None and np.nanmax(trial.belt_speed) > 1.0
    assert 40.0 < camargo.subject_mass(REAL_SUBJECT) < 120.0
