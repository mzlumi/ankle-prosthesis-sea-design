#!/usr/bin/env python3
"""Bode plots of the SEA torque plant for the two reference designs.

Writes:
  results/plant.md
  results/figures/plant_bode.png
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from anklesea import RESULTS_DIR
from anklesea.designs import default_setup, reference_designs
from anklesea.plant import SEAParams, fixed_output, free_output, joint_motion_to_torque
from anklesea.profiles import fourier_coefficients, read_profiles
from anklesea.sizing import LEVEL_WALK

REPORT = RESULTS_DIR / "plant.md"
FIGURE = RESULTS_DIR / "figures" / "plant_bode.png"
HZ = 2 * np.pi


def main() -> int:
    motor, _ = default_setup()
    designs = reference_designs()
    params = {name: SEAParams.from_design(d, motor) for name, d in designs.items()}
    f = np.logspace(-1, 2.5, 2000)
    w = f * HZ
    styles = {"compromise": ("#b45309", "-"), "walking": ("#1d4ed8", "--")}

    fig, axes = plt.subplots(2, 3, figsize=(14, 6.5), layout="constrained", sharex=True)
    titles = ["fixed output: spring torque / motor torque", "free output (foot in swing)",
              "joint motion to spring torque, motor torque held"]
    for name, p in params.items():
        color, ls = styles[name]
        for col, sys in enumerate([fixed_output(p), free_output(p), joint_motion_to_torque(p)]):
            resp = sys(1j * w)
            axes[0, col].loglog(f, np.abs(resp), color=color, ls=ls, lw=1.8,
                                label=f"{name}: k = {p.stiffness:.0f}, N = {p.ratio:.0f}")
            axes[1, col].semilogx(f, np.degrees(np.unwrap(np.angle(resp))), color=color, ls=ls, lw=1.8)
        axes[0, 0].axvline(p.fixed_resonance / HZ, color=color, ls=":", lw=1)
    for col, title in enumerate(titles):
        axes[0, col].set_title(title, fontsize=10, loc="left")
        axes[1, col].set_xlabel("frequency (Hz)")
        axes[0, col].grid(alpha=0.3, which="both")
        axes[1, col].grid(alpha=0.3, which="both")
    axes[0, 0].set_ylabel("magnitude (N·m per N·m)")
    axes[0, 2].set_ylabel("N·m per rad")
    axes[1, 0].set_ylabel("phase (deg)")
    axes[0, 0].legend(fontsize=8)
    fig.savefig(FIGURE, dpi=150)
    plt.close(fig)

    walk = read_profiles()[LEVEL_WALK]
    coeffs = fourier_coefficients(walk.angle, 20)[1:]
    freqs = np.arange(1, 21) / walk.stride_time
    vel_power = np.abs(coeffs) ** 2 * freqs**2

    def velocity_share_above(f_hz: float) -> float:
        return float(vel_power[freqs > f_hz].sum() / vel_power.sum())

    rows = []
    for name, p in params.items():
        rows.append(f"| {name} | {p.stiffness:.0f} | {p.ratio:.0f} | {p.reflected_inertia:.2f} | "
                    f"{p.reflected_damping:.2f} | {p.gain:.0f} | {p.fixed_resonance / HZ:.2f} | "
                    f"{p.free_resonance / HZ:.1f} |")
    comp = params["compromise"]
    text = f"""# SEA plant model

Linear model of the actuator for torque control (`anklesea.plant`, python-control). The controlled output is the spring torque `tau_s = k (theta_m / N - theta)`, measured from the spring deflection; the input is the motor torque `k_t i` (the driver's current loop is modeled in the time-domain simulation, section 15). The motor equation is the one used for sizing, `J theta_m_ddot + b theta_m_dot = tau_m - tau_s / (N eta)`, so on the joint side the rotor appears as an inertia `eta J N^2` and damping `eta b N^2`. The foot below the spring has inertia `J_L = {comp.load_inertia} kg m^2` (assumption, `docs/assumptions.md`).

| design | k (N·m/rad) | N | reflected inertia `eta J N^2` (kg m²) | reflected damping (N·m·s/rad) | DC gain `N eta` | fixed-output resonance (Hz) | free-output resonance (Hz) |
|---|---|---|---|---|---|---|---|
{chr(10).join(rows)}

![Bode plots](figures/plant_bode.png)

*Bode plots of the two designs. Left: joint held (stance). Middle: joint free with the foot's inertia (swing). Right: how joint motion disturbs the spring torque when the motor torque is held. Dotted lines mark the fixed-output resonances.*

## Reading the plots

- **Fixed output.** A lightly damped second-order low-pass, `N eta k / (eta J N^2 s^2 + eta b N^2 s + k)`: the spring against the reflected rotor inertia. The resonance is low, {params['compromise'].fixed_resonance / HZ:.1f} Hz for the compromise design and {params['walking'].fixed_resonance / HZ:.1f} Hz for the walking-tuned one, because the high ratio makes the reflected inertia large ({comp.reflected_inertia:.2f} kg m², about {comp.reflected_inertia / comp.load_inertia:.0f} times the foot's). The friction gives a damping ratio of only {comp.reflected_damping / (2 * np.sqrt(comp.stiffness * comp.reflected_inertia)):.3f}. The tests check the DC gain (`N eta`) and the resonance (`sqrt(k / (eta J N^2))`).
- **Free output.** Zero DC gain (a free foot cannot hold a steady torque; the torque only accelerates it) and a resonance at `sqrt(k (1 / (eta J N^2) + 1 / J_L))` = {comp.free_resonance / HZ:.1f} Hz, set by the light foot on the spring.
- **Joint motion.** With the motor torque held, slow joint motion is followed by the rotor and does not change the spring torque; above the fixed-output resonance the rotor cannot follow and the joint motion deflects the spring fully (`-k theta`). In level walking, {velocity_share_above(params['walking'].fixed_resonance / HZ):.0%} of the joint-velocity power lies above the walking-tuned design's resonance and {velocity_share_above(comp.fixed_resonance / HZ):.0%} above the compromise design's, so the controller must reject joint motion as a disturbance. This is why the feedforward of section 13 uses the measured joint motion.

The low open-loop resonance is the price of the energy-optimal design: the same large reflected inertia that a spring decouples from the joint for efficiency makes the actuator slow to change its torque without feedback. The controller has to raise the torque bandwidth well above the open-loop resonance, which feedback can do only as far as the motor's voltage and current allow.
"""
    REPORT.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
