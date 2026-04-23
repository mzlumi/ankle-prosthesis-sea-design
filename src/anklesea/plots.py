"""Shared Matplotlib helpers for the sizing figures."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from anklesea.sizing import SweepResult

CITATION = "Data: Camargo et al. (2021), J. Biomech. 119:110320, CC BY 4.0."

ACTIVITY_COLORS = {
    "treadmill": "#1f77b4",
    "levelground": "#17becf",
    "rampascent": "#d62728",
    "rampdescent": "#ff9896",
    "stairascent": "#2ca02c",
    "stairdescent": "#98df8a",
    "level": "#1f77b4",  # level walking in the daily mix
}
ACTIVITY_NAMES = {
    "level": "level walking",
    "treadmill": "treadmill",
    "levelground": "level ground",
    "rampascent": "ramp ascent",
    "rampdescent": "ramp descent",
    "stairascent": "stair ascent",
    "stairdescent": "stair descent",
}

LIMIT_STYLES = {
    "ok_voltage": ("voltage limit", "#b45309", "-"),
    "ok_peak_current": ("peak current limit", "#7c3aed", "--"),
    "ok_rms_current": ("continuous (RMS) current limit", "#be123c", ":"),
}


def energy_contour(
    ax: Axes,
    result: SweepResult,
    vmax: float | None = None,
    limits: tuple[str, ...] = ("ok_voltage", "ok_peak_current", "ok_rms_current"),
    veil_infeasible: str | None = "drive_feasible",
) -> tuple[object, list[Line2D]]:
    """Filled contour of energy per stride over (k, N), with limit boundaries.

    Levels are log-spaced, or linear when the energy reaches zero or below (a load that
    returns net energy through regeneration, such as stair descent).

    The rigid column (``k = inf``) is left out of the contour. Returns the contour set
    (for a colorbar) and legend handles for the drawn limit lines.
    """
    finite = np.isfinite(result.stiffness)
    k, n = result.stiffness[finite], result.ratios
    energy = result.energy[:, finite]
    if vmax is None:
        vmax = float(np.nanpercentile(energy, 60))
    vmin = float(np.nanmin(energy))
    levels = np.geomspace(vmin, vmax, 15) if vmin > 0 else np.linspace(vmin, vmax, 15)
    cs = ax.contourf(k, n, np.clip(energy, vmin, vmax), levels=levels, cmap="viridis_r", extend="max")
    ax.contour(k, n, energy, levels=levels[::2], colors="white", linewidths=0.4, alpha=0.6)
    handles = []
    for key in limits:
        label, color, style = LIMIT_STYLES[key]
        ok = result.metrics[key][:, finite].astype(float)
        if 0.0 < ok.mean() < 1.0:
            ax.contour(k, n, ok, levels=[0.5], colors=color, linestyles=style, linewidths=1.8)
            handles.append(Line2D([], [], color=color, linestyle=style, linewidth=1.8, label=label))
    if veil_infeasible is not None:
        bad = (~result.metrics[veil_infeasible][:, finite]).astype(float)
        if bad.any():
            ax.contourf(k, n, bad, levels=[0.5, 1.5], colors="white", alpha=0.55)
            handles.append(Patch(facecolor="white", alpha=0.55, edgecolor="0.5", label="breaks a drive limit (veiled)"))
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("series spring stiffness k (N·m/rad)")
    ax.set_ylabel("gear ratio N")
    return cs, handles


def add_citation(fig: Figure, text: str = CITATION) -> None:
    """Data attribution in a strip below a constrained-layout figure."""
    fig.get_layout_engine().set(rect=(0.0, 0.03, 1.0, 0.97))
    fig.text(0.01, 0.005, text, fontsize=7, color="0.35")
