# One actuator for several activities

A prosthesis has one spring and one transmission. This page asks two questions. What does a design tuned for level walking cost in the other activities? And which single design minimizes the energy over a typical day?

**Daily mix.** Shares of steps from a free-living study of ten people with a transtibial prosthesis, monitored with an instrumented pylon over 1 to 2 weekdays (Srisuwan and Klute, *Prosthet. Orthot. Int.* 45(3):191-197, 2021, Table 3). Turning steps (9.0 %) are counted as level walking, which they resemble most of the conditions here. The same study gives 4422 steps per day, so 2211 strides of the prosthetic leg. Recorded in `docs/assumptions.md`.

**Conditions.** One dataset condition stands for each activity: level walking is the treadmill at 1.2 m/s, as in `results/sizing_levelwalk.md`; ramps and stairs use the incline and step height closest to the course in the mix source (5 degree ramps, 18 cm steps), among conditions with enough subjects.

| activity | condition | subjects (strides) | share of steps |
|---|---|---|---|
| level walking | 1.20 m/s | 2 (58) | 91.8% |
| ramp ascent | 5.2 deg | 2 (6) | 1.6% |
| ramp descent | 5.2 deg | 2 (11) | 2.0% |
| stair ascent | 6 in | 2 (6) | 2.3% |
| stair descent | 6 in | 2 (9) | 2.5% |

**Designs compared.** *Own optimum*: the best design for that activity alone (a lower bound no single actuator reaches). *Walking-tuned*: the level-walking optimum, k = 294 N·m/rad, N = 771. *Compromise*: the design with the least step-weighted energy that meets the drive limits in all five activities, k = 196 N·m/rad, N = 440. The weighted energy is again a convex quadratic in compliance at each ratio, so the compromise comes from the same clipped closed form (`anklesea.sizing.optimize_mix`). All at 80 kg and 36 V.

![Cross-mode comparison](figures/cross_mode.png)

*Left: energy per stride in each activity for the three designs (bars with ideal regeneration, black ticks without; cross-hatched bars break a drive limit). Right: energy per day over the free-living mix. Data: Camargo et al. (2021), CC BY 4.0; activity mix: Srisuwan and Klute (2021).*

## Energy per stride

Energy with ideal regeneration; in brackets, the extra over the activity's own optimum.

| activity (condition) | share of steps | own optimum | own energy (J) | walking-tuned design (J) | compromise design (J) | no regeneration: own / walking / compromise (J) |
|---|---|---|---|---|---|---|
| level walking (1.20 m/s) | 91.8% | k = 294 N·m/rad, N = 771 | 29.6 | 29.6 (+0.0) | 45.7 (+16.2) | 49.2 / 49.2 / 80.0 |
| ramp ascent (5.2 deg) | 1.6% | k = 248 N·m/rad, N = 793 | 30.2 | 30.4 (+0.2) | 37.1 (+6.9) | 41.3 / 38.3 / 49.4 |
| ramp descent (5.2 deg) | 2.0% | k = 182 N·m/rad, N = 440 | -5.4 | -18.9 (-13.5), **breaks limits** (61 V, 16 A) | -5.3 (+0.0) | 35.5 / 27.9 / 31.7 |
| stair ascent (6 in) | 2.3% | k = 313 N·m/rad, N = 647 | 40.3 | 39.9 (-0.4), **breaks limits** (41 V, 8 A) | 45.5 (+5.2) | 44.4 / 46.6 / 57.3 |
| stair descent (6 in) | 2.5% | k = 340 N·m/rad, N = 881 | -42.7 | -42.1 (+0.7) | -27.5 (+15.2) | 10.3 / 8.5 / 31.1 |

## Energy per day

| design | with regeneration (Wh/day) | without regeneration (Wh/day) |
|---|---|---|
| own optimum in each activity (lower bound) | 16.78 | 29.31 |
| walking-tuned | 16.62 | 29.19 |
| compromise | 26.26 | 47.15 |

- The walking-tuned design **breaks the drive limits** in ramp descent and stair ascent: it could not produce the torque those activities need at the speed they need it, whatever the energy.
- The compromise is decided by the limits rather than by the weights. At k = 196 N·m/rad, N = 440 the limit binds in ramp descent; to stay feasible there, the ratio drops from 771 to 440, and level walking, 92% of the steps, pays +16.2 J per stride (+55%).
- Over a day the compromise uses 26.3 Wh at the motor terminals (with regeneration; 47.1 Wh without), against 16.8 Wh for the unreachable lower bound. The walking-tuned design would use 16.6 Wh but cannot do every activity.
- Feasibility and heat constrain this actuator more than energy does: the voltage limit picks the ratio, and the RMS current, not the energy, is what this motor cannot sustain in walking (`results/sizing_levelwalk.md`).

## Sensitivity to the weights and conditions

Each row recomputes the compromise with one assumption changed, then evaluates that design on the baseline conditions and mix, so the daily energies compare like with like.

| case | compromise design | daily energy on the baseline mix (Wh) | same, no regeneration (Wh) |
|---|---|---|---|
| baseline: Srisuwan and Klute (2021) mix, energy with regeneration | k = 196 N·m/rad, N = 440 | 26.26 | 47.15 |
| ramps and stairs three times as frequent | k = 196 N·m/rad, N = 440 | 26.26 | 47.15 |
| equal weight on all five activities | k = 196 N·m/rad, N = 440 | 26.26 | 47.15 |
| level walking only (but within limits everywhere) | k = 196 N·m/rad, N = 440 | 26.26 | 47.15 |
| objective: energy without regeneration (grid search) | k = 350 N·m/rad, N = 429 | 27.72 | 31.45 |
| level walking at 1.0 m/s | k = 196 N·m/rad, N = 440 | 26.26 | 47.15 |
| level walking at 1.4 m/s | k = 230 N·m/rad, N = 437 | 26.47 | 40.28 |
| lowest ramp and stair conditions | k = 196 N·m/rad, N = 440 | 26.26 | 47.15 |
| highest ramp and stair conditions | k = 260 N·m/rad, N = 607 | 18.91 (breaks limits) | 31.85 |
| bus voltage 48 V (limits checked at 48 V) | k = 245 N·m/rad, N = 582 | 19.56 | 33.98 |

5 of the 9 changed cases land on exactly the baseline compromise: as long as every activity must stay within the limits, the weights hardly matter, because the feasible region is narrow and the binding activity fixes the ratio. What does move the design is a change in *which* conditions must be feasible (a faster walking speed or steeper ramps and stairs), the energy measure (without regeneration the optimum prefers a stiffer spring), and the bus voltage: at 48 V the voltage limit relaxes and the compromise can return toward the walking-tuned design. The practical lesson is that the hardest activity sizes the transmission; the daily mix at most fine-tunes the spring.
