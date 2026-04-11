"""Series elastic actuator sizing and torque control for a powered ankle prosthesis.

The load (ankle angle and moment) comes from the lower-limb dataset of
Camargo et al. (2021), J. Biomech. 119:110320, doi:10.1016/j.jbiomech.2021.110320 (CC BY 4.0).
All quantities inside the package are SI: rad, rad/s, N·m, A, V, W, J, s.
"""

from pathlib import Path

__version__ = "0.1.0"

REPO_ROOT = Path(__file__).resolve().parents[2]
RAW_DIR = REPO_ROOT / "data" / "raw" / "camargo"
PROCESSED_DIR = REPO_ROOT / "data" / "processed"
RESULTS_DIR = REPO_ROOT / "results"
