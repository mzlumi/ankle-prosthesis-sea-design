import anklesea


def test_package_imports() -> None:
    assert anklesea.__version__
    assert anklesea.REPO_ROOT.joinpath("pyproject.toml").exists()
