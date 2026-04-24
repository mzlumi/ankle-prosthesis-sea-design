"""The two actuator designs carried into the control study.

* ``walking``: the least-energy SEA for level walking at 1.2 m/s within the drive limits
  (``results/sizing_levelwalk.md``, refined by the ratio scan of ``results/sizing_modes.md``).
* ``compromise``: the least step-weighted energy over the free-living activity mix, within
  the drive limits in every activity (``results/cross_mode.md``).

Both at the design user mass and bus voltage of :mod:`anklesea.sizing`. The controller
settings and the targets stated before tuning (``results/torque_pid.md``) live here too, so
that every control script uses the same ones.
"""

from __future__ import annotations

from dataclasses import replace
from functools import lru_cache

from anklesea.motor import Motor, default_motor
from anklesea.profiles import read_profiles
from anklesea.plant import SEAParams
from anklesea.sea import Design, Limits
from anklesea.sizing import (
    BUS_VOLTAGE,
    DAILY_MIX,
    DESIGN_MASS_KG,
    LEVEL_WALK,
    MIX_ACTIVITIES,
    mix_conditions,
    optimize,
    optimize_mix,
)
from anklesea.torque_control import Controller, LoopMetrics, dob_cutoff, tune_pid

CONTROL_RATE_HZ = 1000.0
LOOP_DELAY = 1.5 / CONTROL_RATE_HZ  # one sample of computation plus half a sample of zero-order hold
PLACEMENT_HZ = 8.0  # closed-loop pole placement of the PD part
INTEGRAL_FRACTION = 0.2  # integral corner as a fraction of the placement
TARGET_BANDWIDTH_HZ = 6.0
TARGET_PM_DEG = 45.0
TARGET_GM_DB = 10.0
TARGET_MS = 2.0
TARGET_TRACKING_PCT = 5.0


def meets_margin_targets(m: LoopMetrics) -> bool:
    """Phase margin, gain margin (both directions) and peak sensitivity targets."""
    return (m.phase_margin_deg >= TARGET_PM_DEG and min(m.gain_margin_db, m.lower_gain_margin_db) >= TARGET_GM_DB
            and m.peak_sensitivity <= TARGET_MS)


@lru_cache(maxsize=1)
def reference_designs() -> dict[str, Design]:
    """``{"walking": Design, "compromise": Design}`` computed from the committed profiles."""
    profiles = read_profiles()
    motor = default_motor()
    limits = Limits.from_motor(motor, BUS_VOLTAGE)
    walk = optimize(profiles[LEVEL_WALK].load(DESIGN_MASS_KG), motor, limits)
    conditions = mix_conditions(profiles)
    loads = [profiles[conditions[a]].load(DESIGN_MASS_KG) for a in MIX_ACTIVITIES]
    comp = optimize_mix(loads, [DAILY_MIX[a] for a in MIX_ACTIVITIES], motor, limits)
    return {"walking": Design(walk["stiffness"], walk["ratio"]), "compromise": Design(comp["stiffness"], comp["ratio"])}


def default_setup() -> tuple[Motor, Limits]:
    motor = default_motor()
    return motor, Limits.from_motor(motor, BUS_VOLTAGE)


def reference_controllers(p: SEAParams) -> dict[str, Controller]:
    """The PID of ``results/torque_pid.md`` and the PD + DOB of ``results/torque_dob.md``, both with model feedforward."""
    pid = tune_pid(p, PLACEMENT_HZ, integral_fraction=INTEGRAL_FRACTION)
    pd = replace(tune_pid(p, PLACEMENT_HZ), ki=0.0)
    cutoff = dob_cutoff(p, pd, LOOP_DELAY, meets_margin_targets)
    return {"PID": Controller(p, pid, "model"), "DOB": Controller(p, pd, "model", cutoff)}
