import sys
from pathlib import Path

import matplotlib
import pytest

matplotlib.use("Agg")
sys.path.insert(0, str(Path(__file__).parent))

from anklesea import RAW_DIR  # noqa: E402

REAL_SUBJECT = "AB06"


def _has_real_data() -> bool:
    return (RAW_DIR / "SubjectInfo.mat").exists() and any((RAW_DIR / REAL_SUBJECT).glob("*/*/id/*.mat"))


requires_data = pytest.mark.skipif(not _has_real_data(), reason="Camargo data not in data/raw/ (run scripts/fetch_data.py)")


@pytest.fixture
def raw_dir(tmp_path: Path) -> Path:
    return tmp_path / "camargo"
