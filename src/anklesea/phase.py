"""Gait phase for a phase-scheduled torque command.

A phase-scheduled controller commands ``tau_ref(t) = m * profile(phase(t))``: the mean
ankle moment per kg of the current activity, looked up at the estimated gait phase and
scaled by the user's mass. Two phase sources are compared:

- the dataset's own phase (``gcRight/HeelStrike``, 0 to 100 % between heel strikes),
  which knows when the current stride will end and is therefore an oracle;
- a causal time-based estimator: the time since the last detected heel strike divided
  by the duration of the previous stride. It is the simplest estimator a device can run
  and stands in for an IMU-based one, which is not available yet.

A real heel-strike detector reports the event late. The estimator here resets its phase
when the detection arrives and does not back-date it, so a detection latency ``d``
delays the whole command by ``d``.
"""

from __future__ import annotations

import numpy as np

from anklesea.strides import MAX_STRIDE_S, MIN_STRIDE_S

PHASE_CAP = 0.999  # the estimate holds just below 100 % until the next heel strike


def periodic_lookup(curve: np.ndarray, phase: np.ndarray) -> np.ndarray:
    """Values of a curve sampled uniformly from 0 to 100 % of the cycle at ``phase`` in [0, 1]."""
    curve = np.asarray(curve, dtype=float)
    return np.interp(np.clip(phase, 0.0, 1.0), np.linspace(0.0, 1.0, len(curve)), curve)


def phase_error(estimate: np.ndarray, truth: np.ndarray) -> np.ndarray:
    """``estimate - truth`` in cycles, wrapped to [-0.5, 0.5) so that 0 and 1 are the same phase."""
    return (np.asarray(estimate) - np.asarray(truth) + 0.5) % 1.0 - 0.5


def time_based_phase(
    time: np.ndarray,
    strike_times: np.ndarray,
    default_period: float,
    detection_delay: float = 0.0,
) -> np.ndarray:
    """Causal phase estimate in [0, ``PHASE_CAP``] at each ``time`` (NaN before the first detection).

    Heel strikes are detected ``detection_delay`` seconds after ``strike_times``. The
    phase is the time since the last detection over the interval between the last two
    detections. When there is no earlier detection, or that interval is not a plausible
    stride (a pause or a missed event), ``default_period`` is used instead.
    """
    time = np.asarray(time, dtype=float)
    detected = np.sort(np.asarray(strike_times, dtype=float)) + detection_delay
    k = np.searchsorted(detected, time, side="right") - 1
    phase = np.full(time.shape, np.nan)
    ok = k >= 0
    last = detected[k[ok]]
    previous = detected[np.maximum(k[ok] - 1, 0)]
    interval = np.where(k[ok] >= 1, last - previous, np.nan)
    plausible = (interval >= MIN_STRIDE_S) & (interval <= MAX_STRIDE_S)
    period = np.where(plausible, interval, default_period)
    phase[ok] = np.minimum((time[ok] - last) / period, PHASE_CAP)
    return phase


def command_error_pct(command: np.ndarray, actual: np.ndarray) -> float:
    """RMS of ``command - actual`` over a stride, in percent of the stride's peak ``|actual|``."""
    actual = np.asarray(actual, dtype=float)
    return float(100.0 * np.sqrt(np.mean((np.asarray(command) - actual) ** 2)) / np.max(np.abs(actual)))
