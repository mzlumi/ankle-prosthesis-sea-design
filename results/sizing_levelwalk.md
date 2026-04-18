# Sizing for level walking

Spring stiffness `k` and total gear ratio `N` of the series elastic actuator that minimize the motor's electrical energy per stride for level walking.

**Load.** Mean right-ankle angle and moment of 2 subjects (58 strides) walking on the treadmill at 1.2 m/s, stride time 1.087 s, moment scaled to an 80 kg user. Joint work per stride: 10.7 J net, 33.8 J positive. Peak joint torque 153 N·m, peak joint speed 4.55 rad/s. Treadmill walking stands in for level walking because the instrumented belt gives a moment for every stride; overground, only the stances on a force plate have one (see `results/gait_profiles.md`).

**Actuator.** maxon EC-4pole 30 (200 W, 36 V winding), gearhead efficiency 0.70, ESCON 70/10 driver on a 36 V bus (34.2 V available), values in `data/motors/maxon_ec4pole30_305014.yaml`. Assumptions in `docs/assumptions.md`.

**Search.** Full energy model (`anklesea.sea`) on a grid of 121 stiffness values (50 to 20000 N·m/rad, log-spaced) plus the rigid actuator, by 121 ratios (30 to 2000). Energy counts ideal regeneration; the column without regeneration bounds the other extreme.

![Energy contour](figures/sizing_levelwalk.png)

*Motor electrical energy per stride over spring stiffness and gear ratio. Veiled designs break the voltage, peak current or speed limit. The continuous current limit is not drawn because no design on the grid meets it. Data: Camargo et al. (2021), CC BY 4.0.*

## Result

| design | k (N·m/rad) | N | energy (J) | energy, no regeneration (J) | copper loss (J) | peak current (A) | RMS current (A) | peak voltage (V) | peak motor speed (rpm) | limits broken |
|---|---|---|---|---|---|---|---|---|---|---|
| SEA, least energy within the drive limits | 287 | 751 | 29.8 | 49.9 | 11.3 | 12.8 | 7.04 | 33.4 | 15467 | RMS current |
| SEA, least energy with no limits | 302 | 894 | 28.8 | 51.1 | 9.2 | 10.7 | 6.34 | 39.6 | 18292 | voltage, RMS current |
| rigid, least energy with no limits | rigid | 725 | 41.1 | 86.9 | 17.5 | 18.6 | 8.75 | 69.2 | 31518 | voltage, RMS current, speed |

- The least-energy design within the drive limits is **k = 287 N·m/rad (3.59 N·m/rad per kg of user mass), N = 751**, at 29.8 J per stride (27.4 W average). That is 27% less than the best rigid actuator, which also breaks the drive limits.
- No rigid actuator (k = inf) meets the drive limits at any ratio from 30 to 2000. A rigid actuator needs a high ratio to keep the push-off current under 30 A, and at that ratio the back EMF at the push-off joint speed alone exceeds the available voltage. The spring lets the joint move during push-off while the motor turns more slowly, so the same ratio fits inside the voltage limit.
- Without any limits the optimum moves to N = 894; the voltage limit is what holds the ratio down. Energy is flat near the optimum: the contour shows a long valley along which `k` and `N` trade off.
- **No design meets the motor's continuous current rating.** The lowest stride RMS current anywhere on the grid is 5.66 A against the 5.06 A rating, so this motor would overheat in sustained walking for an 80 kg user. Its copper loss is set mostly by the joint torque reflected through the gear, which no series spring can reduce. Section 11 returns to this with a parallel spring, which carries part of the torque itself.
- 5% of the designs on the grid meet the drive limits.

## Sensitivity to the assumptions

Each row changes one assumption from `docs/assumptions.md` and repeats the search. "Rigid" is the best rigid actuator within the drive limits; "any fully feasible" says whether any design also meets the RMS current rating.

| case | k (N·m/rad) | N | energy (J) | rigid energy (J) | RMS current (A) | any fully feasible |
|---|---|---|---|---|---|---|
| baseline | 287 (3.59/kg) | 751 | 29.8 | infeasible | 7.04 | no |
| user mass 60 kg | 224 (3.73/kg) | 751 | 21.7 | 45.1 | 5.57 | no |
| user mass 100 kg | 333 (3.33/kg) | 751 | 39.2 | infeasible | 8.53 | no |
| bus voltage 24 V | 287 (3.59/kg) | 511 | 39.0 | infeasible | 9.88 | no |
| bus voltage 48 V | 302 (3.77/kg) | 894 | 28.8 | 50.4 | 6.34 | no |
| gear efficiency 0.6 | 273 (3.41/kg) | 751 | 35.9 | infeasible | 8.03 | no |
| gear efficiency 0.9 | 287 (3.59/kg) | 751 | 22.5 | 47.8 | 5.70 | no |
| direction-dependent efficiency | 387 (4.84/kg) | 751 | 32.3 | infeasible | 6.71 | no |

The optimal stiffness scales roughly with user mass (it stays near 3.6 N·m/rad per kg): the joint torque scales with mass while the joint angles do not, so the spring that best cancels the motor motion during push-off, where `tau_dot / k` offsets `theta_dot`, scales with mass too. The bus voltage mainly moves the ratio, through the voltage limit. Gear efficiency changes the energy but hardly the design, except that charging the gear loss in the right direction (rather than dividing by `eta` throughout) favors a stiffer spring.
