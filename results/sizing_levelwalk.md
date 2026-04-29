# Sizing for level walking

Spring stiffness `k` and total gear ratio `N` of the series elastic actuator that minimize the motor's electrical energy per stride for level walking.

**Load.** Mean right-ankle angle and moment of 22 subjects (676 strides) walking on the treadmill at 1.2 m/s, stride time 1.037 s, moment scaled to an 80 kg user. Joint work per stride: 9.4 J net, 25.9 J positive. Peak joint torque 123 N·m, peak joint speed 5.18 rad/s. Treadmill walking stands in for level walking because the instrumented belt gives a moment for every stride; overground, only the stances on a force plate have one (see `results/gait_profiles.md`).

**Actuator.** maxon EC-4pole 30 (200 W, 36 V winding), gearhead efficiency 0.70, ESCON 70/10 driver on a 36 V bus (34.2 V available), values in `data/motors/maxon_ec4pole30_305014.yaml`. Assumptions in `docs/assumptions.md`.

**Search.** Full energy model (`anklesea.sea`) on a grid of 121 stiffness values (50 to 20000 N·m/rad, log-spaced) plus the rigid actuator, by 121 ratios (30 to 2000). Energy counts ideal regeneration; the column without regeneration bounds the other extreme.

![Energy contour](figures/sizing_levelwalk.png)

*Motor electrical energy per stride over spring stiffness and gear ratio. Veiled designs break the voltage, peak current or speed limit. The continuous current limit is not drawn because no design on the grid meets it. Data: Camargo et al. (2021), CC BY 4.0.*

## Result

| design | k (N·m/rad) | N | energy (J) | energy, no regeneration (J) | copper loss (J) | peak current (A) | RMS current (A) | peak voltage (V) | peak motor speed (rpm) | limits broken |
|---|---|---|---|---|---|---|---|---|---|---|
| SEA, least energy within the drive limits | 235 | 725 | 23.4 | 40.7 | 7.2 | 11.2 | 5.73 | 33.4 | 15415 | RMS current |
| SEA, least energy with no limits | 235 | 777 | 23.3 | 41.8 | 6.6 | 11.4 | 5.52 | 35.8 | 16532 | voltage, RMS current |
| rigid, least energy with no limits | rigid | 588 | 34.0 | 68.2 | 14.4 | 18.3 | 8.15 | 63.8 | 29084 | voltage, RMS current, speed |
| rigid, least energy within the drive limits | rigid | 302 | 52.5 | 63.7 | 37.4 | 29.5 | 13.11 | 34.1 | 14958 | RMS current |

- The least-energy design within the drive limits is **k = 235 N·m/rad (2.94 N·m/rad per kg of user mass), N = 725**, at 23.4 J per stride (22.6 W average). That is 31% less than the best rigid actuator even when the rigid one may ignore the drive limits.
- The best rigid actuator within the drive limits needs N = 302 and 52.5 J per stride, 124% more than the SEA. Without a spring the motor turns with the joint, so the back EMF at the push-off joint speed caps the ratio; at that low ratio the push-off torque needs a large current and the copper loss grows. The spring lets the joint move during push-off while the motor turns more slowly, so the SEA can use a higher ratio within the same voltage.
- Without any limits the optimum moves to N = 777; the voltage limit is what holds the ratio down. Energy is flat near the optimum: the contour shows a long valley along which `k` and `N` trade off.
- **No design meets the motor's continuous current rating.** The lowest stride RMS current anywhere on the grid is 5.11 A against the 5.06 A rating (1% above it), and the least-energy design draws 5.73 A (13% above), so in sustained walking for an 80 kg user this motor runs hotter than its continuous rating allows. Its copper loss is set mostly by the joint torque reflected through the gear, which no series spring can reduce. Section 11 returns to this with a parallel spring, which carries part of the torque itself.
- 7% of the designs on the grid meet the drive limits.

## Sensitivity to the assumptions

Each row changes one assumption from `docs/assumptions.md` and repeats the search. "Rigid" is the best rigid actuator within the drive limits; "any fully feasible" says whether any design also meets the RMS current rating.

| case | k (N·m/rad) | N | energy (J) | rigid energy (J) | RMS current (A) | any fully feasible |
|---|---|---|---|---|---|---|
| baseline | 235 (2.94/kg) | 725 | 23.4 | 52.5 | 5.73 | no |
| user mass 60 kg | 183 (3.05/kg) | 676 | 17.5 | 33.6 | 4.81 | yes |
| user mass 100 kg | 287 (2.87/kg) | 725 | 30.0 | infeasible | 6.87 | no |
| bus voltage 24 V | 202 (2.53/kg) | 493 | 27.7 | infeasible | 7.60 | no |
| bus voltage 48 V | 235 (2.94/kg) | 777 | 23.3 | 38.7 | 5.52 | no |
| gear efficiency 0.6 | 224 (2.80/kg) | 725 | 27.7 | infeasible | 6.45 | no |
| gear efficiency 0.9 | 235 (2.94/kg) | 676 | 18.2 | 35.5 | 4.91 | yes |
| direction-dependent efficiency | 317 (3.96/kg) | 725 | 25.8 | 48.9 | 5.64 | no |

The optimal stiffness scales roughly with user mass (it stays near 2.9 N·m/rad per kg): the joint torque scales with mass while the joint angles do not, so the spring that best cancels the motor motion during push-off, where `tau_dot / k` offsets `theta_dot`, scales with mass too. The bus voltage mainly moves the ratio, through the voltage limit. Gear efficiency changes the energy but hardly the design, except that charging the gear loss in the right direction (rather than dividing by `eta` throughout) favors a stiffer spring.
