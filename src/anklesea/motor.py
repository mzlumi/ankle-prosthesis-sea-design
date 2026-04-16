"""Motor, transmission and driver constants from a manufacturer's datasheet, in SI units.

The motor is modelled as an equivalent DC motor (Notre Dame AME 60553 lecture notes
L9 and L10): the winding voltage is ``v = R i + L di/dt + k_t omega`` and the
electromagnetic torque is ``k_t i``. maxon gives the terminal resistance and
inductance phase to phase and a torque constant defined so that this single-phase
model reproduces the catalog's stall torque and no-load speed, which the tests check.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml

from anklesea import REPO_ROOT

RPM = 2.0 * np.pi / 60.0  # rad/s per rpm
GCM2 = 1e-7  # kg m^2 per g cm^2
DEFAULT_MOTOR_FILE = REPO_ROOT / "data" / "motors" / "maxon_ec4pole30_305014.yaml"


@dataclass(frozen=True)
class Motor:
    """Constants of one motor, its transmission and its driver (SI units).

    ``viscous_friction`` is on the motor shaft. ``gear_efficiency`` applies to the whole
    motor-to-joint transmission and ``gear_inertia`` is added to the rotor inertia.
    ``continuous_current`` is the motor's thermal limit (compared with the RMS current
    over a stride), ``peak_current`` the driver's short-term limit.
    """

    name: str
    torque_constant: float  # N·m/A, equal to the back-EMF constant in V·s/rad
    resistance: float  # ohm
    inductance: float  # H
    rotor_inertia: float  # kg m^2
    viscous_friction: float  # N·m·s/rad
    nominal_voltage: float  # V
    max_speed: float  # rad/s
    continuous_current: float  # A
    peak_current: float  # A
    gear_efficiency: float
    gear_inertia: float  # kg m^2, at the motor shaft
    gear_max_input_speed: float  # rad/s
    driver_voltage_fraction: float
    thermal_resistance: float  # K/W, winding to ambient (sum of both datasheet values)
    max_winding_temperature: float  # deg C
    datasheet_no_load_speed: float  # rad/s
    datasheet_no_load_current: float  # A
    datasheet_stall_torque: float  # N·m
    datasheet_stall_current: float  # A
    datasheet_speed_torque_gradient: float  # (rad/s)/(N·m)
    datasheet_mechanical_time_constant: float  # s

    @property
    def inertia(self) -> float:
        """Rotor plus gearhead inertia at the motor shaft."""
        return self.rotor_inertia + self.gear_inertia

    def stall_torque(self, voltage: float | None = None) -> float:
        """k_t V / R: torque at zero speed with ``voltage`` across the winding."""
        v = self.nominal_voltage if voltage is None else voltage
        return self.torque_constant * v / self.resistance

    def stall_current(self, voltage: float | None = None) -> float:
        v = self.nominal_voltage if voltage is None else voltage
        return v / self.resistance

    def no_load_speed(self, voltage: float | None = None) -> float:
        """Speed where back-EMF plus the no-load friction current balance ``voltage``."""
        v = self.nominal_voltage if voltage is None else voltage
        k, r, b = self.torque_constant, self.resistance, self.viscous_friction
        # v = R i + k w with k i = b w at no load.
        return v / (k + r * b / k)

    @property
    def speed_torque_gradient(self) -> float:
        """Slope of the speed-torque line, R / k_t^2, in (rad/s)/(N·m)."""
        return self.resistance / self.torque_constant**2

    @property
    def mechanical_time_constant(self) -> float:
        """J_rotor R / k_t^2 (rotor only, as in the catalog)."""
        return self.rotor_inertia * self.speed_torque_gradient

    @property
    def motor_constant(self) -> float:
        """k_m = k_t / sqrt(R), torque per square root of copper loss, N·m/sqrt(W)."""
        return self.torque_constant / np.sqrt(self.resistance)

    def available_voltage(self, bus_voltage: float) -> float:
        """Largest voltage the driver can put across the motor from ``bus_voltage``."""
        return self.driver_voltage_fraction * bus_voltage

    def winding_temperature(self, rms_current: float, ambient: float = 25.0) -> float:
        """Steady-state winding temperature for a constant copper loss of R i_rms^2."""
        return ambient + self.resistance * rms_current**2 * self.thermal_resistance

    def with_changes(self, **changes: float) -> Motor:
        return replace(self, **changes)

    @classmethod
    def from_yaml(cls, path: Path = DEFAULT_MOTOR_FILE) -> Motor:
        spec = yaml.safe_load(Path(path).read_text())
        m, g, d = spec["motor"]["values"], spec["gearhead"]["values"], spec["driver"]["values"]
        kt = m["torque_constant_mNm_per_A"] * 1e-3
        no_load_speed = m["no_load_speed_rpm"] * RPM
        no_load_current = m["no_load_current_mA"] * 1e-3
        return cls(
            name=f"{spec['motor']['manufacturer']} {spec['motor']['model']} ({spec['motor']['part_number']})",
            torque_constant=kt,
            resistance=m["terminal_resistance_ohm"],
            inductance=m["terminal_inductance_mH"] * 1e-3,
            rotor_inertia=m["rotor_inertia_gcm2"] * GCM2,
            viscous_friction=kt * no_load_current / no_load_speed,
            nominal_voltage=m["nominal_voltage_V"],
            max_speed=m["max_speed_rpm"] * RPM,
            continuous_current=m["nominal_current_A"],
            peak_current=d["peak_output_current_A"],
            gear_efficiency=g["max_efficiency_pct"] / 100.0,
            gear_inertia=g["mass_inertia_gcm2"] * GCM2,
            gear_max_input_speed=g["max_continuous_input_speed_rpm"] * RPM,
            driver_voltage_fraction=d["max_output_voltage_fraction_of_supply"],
            thermal_resistance=m["thermal_resistance_housing_ambient_K_per_W"] + m["thermal_resistance_winding_housing_K_per_W"],
            max_winding_temperature=m["max_winding_temperature_C"],
            datasheet_no_load_speed=no_load_speed,
            datasheet_no_load_current=no_load_current,
            datasheet_stall_torque=m["stall_torque_mNm"] * 1e-3,
            datasheet_stall_current=m["stall_current_A"],
            datasheet_speed_torque_gradient=m["speed_torque_gradient_rpm_per_mNm"] * RPM * 1e3,
            datasheet_mechanical_time_constant=m["mechanical_time_constant_ms"] * 1e-3,
        )


def default_motor() -> Motor:
    """The maxon EC-4pole 30 (305014) with the GP 32 HP gearhead and ESCON 70/10 driver."""
    return Motor.from_yaml(DEFAULT_MOTOR_FILE)
