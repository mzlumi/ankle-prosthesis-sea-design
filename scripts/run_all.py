#!/usr/bin/env python3
"""Rerun every analysis script in dependency order and report how long each took.

The data must be downloaded first (``python scripts/fetch_data.py``). Each script
rewrites its own files in results/; per-stride caches go to the git-ignored
data/processed/.
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

SCRIPTS = (
    "inventory.py",  # trials and sign convention
    "gait_profiles.py",  # strides, aggregate profiles (everything below reads them)
    "size_levelwalk.py",
    "convex_check.py",
    "size_modes.py",
    "cross_mode.py",
    "parallel_spring.py",
    "plant_bode.py",
    "torque_pid.py",
    "torque_dob.py",
    "tracking.py",
    "robustness.py",
    "phase_schedule.py",  # also reads the per-stride caches of gait_profiles.py
)


def main() -> int:
    here = Path(__file__).resolve().parent
    for name in SCRIPTS:
        start = time.perf_counter()
        print(f"== {name}", flush=True)
        result = subprocess.run([sys.executable, str(here / name)], stdout=subprocess.DEVNULL)
        if result.returncode != 0:
            print(f"{name} failed with exit code {result.returncode}", file=sys.stderr)
            return result.returncode
        print(f"   {time.perf_counter() - start:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
