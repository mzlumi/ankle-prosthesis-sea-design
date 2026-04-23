"""The two actuator designs carried into the control study.

* ``walking``: the least-energy SEA for level walking at 1.2 m/s within the drive limits
  (``results/sizing_levelwalk.md``, refined by the ratio scan of ``results/sizing_modes.md``).
* ``compromise``: the least step-weighted energy over the free-living activity mix, within
  the drive limits in every activity (``results/cross_mode.md``).

Both at the design user mass and bus voltage of :mod:`anklesea.sizing`.
"""

from __future__ import annotations

from functools import lru_cache

from anklesea.motor import Motor, default_motor
from anklesea.profiles import read_profiles
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
