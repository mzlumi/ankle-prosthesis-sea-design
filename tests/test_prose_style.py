"""The project's prose uses commas, colons and parentheses instead of em and en dashes.

Files under docs/source/ are verbatim course material and keep their own typography.
"""

from pathlib import Path

import pytest

from anklesea import REPO_ROOT

DASHES = {"\u2014": "em dash", "\u2013": "en dash"}
SUFFIXES = {".md", ".py", ".yaml", ".yml", ".toml"}
SKIP_PARTS = {".git", ".venv", "venv", "build", "dist", "__pycache__", ".pytest_cache"}
SKIP_PREFIXES = (("docs", "source"), ("data", "raw"), ("data", "processed"))


def _prose_files() -> list[Path]:
    files = []
    for path in REPO_ROOT.rglob("*"):
        if path.suffix not in SUFFIXES or not path.is_file():
            continue
        rel = path.relative_to(REPO_ROOT).parts
        if SKIP_PARTS.intersection(rel) or any(rel[: len(p)] == p for p in SKIP_PREFIXES):
            continue
        files.append(path)
    return sorted(files)


def test_style_scan_finds_files() -> None:
    names = {p.name for p in _prose_files()}
    assert "README.md" in names
    assert "test_prose_style.py" in names


@pytest.mark.parametrize("path", _prose_files(), ids=lambda p: str(p.relative_to(REPO_ROOT)))
def test_no_em_or_en_dashes(path: Path) -> None:
    problems = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        for char, name in DASHES.items():
            if char in line:
                problems.append(f"line {number}: {name}")
    assert not problems, f"{path.relative_to(REPO_ROOT)}: " + "; ".join(problems)
