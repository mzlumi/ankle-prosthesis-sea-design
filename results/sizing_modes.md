# Sizing across activities

The spring stiffness `k` and gear ratio `N` that minimize the motor's electrical energy per stride, sized separately for each activity and condition in the dataset. Same actuator, user mass (80 kg) and bus voltage (36 V) as `results/sizing_levelwalk.md`. Each optimum meets the drive limits (voltage, 30 A peak current, motor speed); it is found by scanning 600 ratios from 30 to 2000 with the closed-form optimum in compliance clipped to the feasible interval at each ratio (`anklesea.sizing.optimize`, checked against the grid search in `results/convex_check.md`).

![Energy contours per activity](figures/sizing_modes_contour.png)

*Energy per stride over (k, N) for one condition per activity (the one with the most subjects; treadmill at 1.2 m/s). Veiled designs break a drive limit. The star is the panel's own optimum; the dots are the other activities' optima, to show how far apart they lie. Data: Camargo et al. (2021), CC BY 4.0.*

![Optimal design per condition](figures/sizing_by_condition.png)

*Energy-optimal stiffness and ratio against walking speed, ramp incline and stair height. Data: Camargo et al. (2021), CC BY 4.0.*

## Design table

Bold rows are the conditions in the contour figure. "Rigid" is the best rigid actuator within the drive limits. Joint work is per stride at the design user mass.

| activity | condition | subjects (strides) | peak torque (N·m) | joint work net / positive (J) | k (N·m/rad) | N | energy (J) | no regen. (J) | rigid (J) | peak / RMS current (A) | peak voltage (V) | binding limit |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| treadmill | 0.50 m/s | 22 (451) | 89 | -3.2 / 10.2 | 254 | 1027 | -0.1 | 12.5 | 3.9 | 5.3 / 2.99 | 20.9 | none |
| treadmill | 0.55 m/s | 22 (465) | 89 | -3.3 / 10.9 | 236 | 958 | 0.2 | 13.9 | 4.6 | 5.7 / 3.17 | 23.2 | none |
| treadmill | 0.60 m/s | 22 (482) | 93 | -2.5 / 12.2 | 237 | 918 | 1.8 | 16.2 | 7.2 | 6.6 / 3.40 | 23.3 | none |
| treadmill | 0.65 m/s | 22 (520) | 96 | -1.9 / 13.6 | 229 | 874 | 3.4 | 19.4 | 10.3 | 7.4 / 3.69 | 24.4 | none |
| treadmill | 0.70 m/s | 22 (523) | 98 | -1.2 / 14.7 | 227 | 856 | 4.6 | 20.8 | 13.0 | 8.0 / 3.88 | 23.8 | none |
| treadmill | 0.75 m/s | 22 (538) | 102 | -0.3 / 15.9 | 228 | 833 | 6.7 | 24.1 | 16.8 | 8.7 / 4.18 | 25.8 | none |
| treadmill | 0.80 m/s | 22 (555) | 105 | 0.5 / 17.2 | 227 | 804 | 8.2 | 26.6 | 21.0 | 9.5 / 4.43 | 26.1 | none |
| treadmill | 0.85 m/s | 22 (564) | 110 | 1.5 / 18.3 | 236 | 850 | 10.5 | 28.9 | 24.6 | 9.0 / 4.60 | 31.3 | none |
| treadmill | 0.90 m/s | 22 (577) | 110 | 1.7 / 18.5 | 234 | 833 | 10.6 | 29.2 | 25.7 | 9.5 / 4.67 | 29.5 | none |
| treadmill | 0.95 m/s | 22 (595) | 112 | 2.4 / 19.5 | 229 | 798 | 12.3 | 32.2 | 30.5 | 10.2 / 4.95 | 31.1 | none |
| treadmill | 1.00 m/s | 22 (603) | 113 | 3.4 / 20.6 | 228 | 798 | 13.8 | 33.3 | 34.1 | 10.6 / 5.03 | 30.6 | none |
| treadmill | 1.05 m/s | 22 (651) | 116 | 4.7 / 22.2 | 224 | 776 | 16.1 | 36.0 | 39.8 | 11.1 / 5.23 | 31.9 | none |
| treadmill | 1.10 m/s | 22 (635) | 117 | 5.9 / 23.2 | 226 | 765 | 18.3 | 38.2 | 44.1 | 11.3 / 5.40 | 33.1 | none |
| treadmill | 1.15 m/s | 22 (649) | 119 | 7.4 / 24.4 | 227 | 760 | 20.5 | 40.3 | 49.2 | 12.0 / 5.50 | 33.9 | none |
| **treadmill** | **1.20 m/s** | 22 (676) | 123 | 9.4 / 25.9 | 235 | 739 | 23.4 | 41.0 | 52.7 | 11.2 / 5.67 | 34.1 | voltage |
| treadmill | 1.25 m/s | 22 (690) | 124 | 9.9 / 26.4 | 239 | 760 | 23.9 | 41.5 | infeasible | 11.8 / 5.60 | 34.2 | voltage |
| treadmill | 1.30 m/s | 22 (701) | 126 | 11.4 / 27.7 | 238 | 724 | 26.3 | 43.8 | infeasible | 11.8 / 5.79 | 34.1 | voltage |
| treadmill | 1.35 m/s | 22 (707) | 128 | 13.4 / 29.9 | 229 | 689 | 29.8 | 48.0 | infeasible | 12.1 / 6.00 | 34.2 | voltage |
| treadmill | 1.40 m/s | 22 (723) | 129 | 14.6 / 30.7 | 236 | 665 | 31.5 | 48.1 | infeasible | 12.2 / 6.17 | 34.2 | voltage |
| treadmill | 1.45 m/s | 22 (758) | 132 | 16.8 / 32.7 | 228 | 629 | 35.5 | 53.0 | infeasible | 12.4 / 6.47 | 34.2 | voltage |
| treadmill | 1.50 m/s | 22 (738) | 133 | 18.6 / 33.9 | 231 | 607 | 38.4 | 54.8 | infeasible | 12.9 / 6.66 | 34.2 | voltage |
| treadmill | 1.55 m/s | 22 (775) | 131 | 21.1 / 34.4 | 224 | 574 | 42.3 | 57.7 | infeasible | 13.7 / 6.86 | 34.2 | voltage |
| treadmill | 1.60 m/s | 22 (794) | 132 | 21.6 / 34.7 | 232 | 595 | 42.4 | 57.3 | infeasible | 13.2 / 6.62 | 34.2 | voltage |
| treadmill | 1.65 m/s | 22 (802) | 134 | 23.6 / 36.3 | 237 | 566 | 45.8 | 59.7 | infeasible | 14.0 / 6.93 | 34.2 | voltage |
| treadmill | 1.70 m/s | 22 (816) | 134 | 24.8 / 37.1 | 240 | 543 | 48.0 | 61.6 | infeasible | 14.5 / 7.19 | 34.2 | voltage |
| treadmill | 1.75 m/s | 22 (831) | 134 | 26.9 / 38.5 | 243 | 513 | 51.7 | 64.2 | infeasible | 15.3 / 7.54 | 34.2 | voltage |
| treadmill | 1.80 m/s | 22 (837) | 134 | 28.4 / 39.1 | 252 | 499 | 54.2 | 65.9 | infeasible | 15.7 / 7.72 | 34.2 | voltage |
| treadmill | 1.85 m/s | 22 (889) | 134 | 30.3 / 40.3 | 258 | 482 | 57.4 | 67.7 | infeasible | 16.3 / 7.97 | 34.2 | voltage |
| treadmill | 1.90 m/s | 2 (74) | 173 | 29.1 / 48.9 | 398 | 566 | 60.2 | 81.3 | infeasible | 18.8 / 9.10 | 34.2 | voltage |
| treadmill | 1.95 m/s | 2 (73) | 170 | 31.5 / 51.0 | 417 | 539 | 65.1 | 86.5 | infeasible | 19.7 / 9.41 | 34.2 | voltage |
| treadmill | 2.00 m/s | 2 (74) | 169 | 33.2 / 52.7 | 418 | 517 | 68.3 | 89.6 | infeasible | 20.5 / 9.75 | 34.2 | voltage |
| treadmill | 2.05 m/s | 2 (75) | 171 | 35.3 / 53.7 | 425 | 499 | 72.1 | 93.0 | infeasible | 21.6 / 10.01 | 34.2 | voltage |
| **level ground** | **slow** | 19 (75) | 119 | 5.8 / 24.4 | 233 | 850 | 17.2 | 33.9 | 33.7 | 8.7 / 4.63 | 34.2 | voltage |
| level ground | normal | 15 (56) | 122 | 11.3 / 28.6 | 227 | 776 | 25.1 | 39.9 | 46.5 | 9.5 / 4.98 | 34.1 | voltage |
| level ground | fast | 10 (29) | 144 | 22.5 / 38.1 | 232 | 578 | 45.6 | 64.0 | infeasible | 14.4 / 6.97 | 34.2 | voltage |
| ramp ascent | 5.2 deg | 7 (14) | 133 | 20.4 / 34.1 | 248 | 739 | 39.6 | 54.0 | 63.9 | 11.2 / 5.31 | 34.2 | voltage |
| **ramp ascent** | **7.8 deg** | 10 (18) | 140 | 19.8 / 35.1 | 272 | 709 | 42.5 | 61.1 | infeasible | 12.8 / 6.45 | 34.1 | voltage |
| ramp ascent | 9.2 deg | 4 (6) | 136 | 31.4 / 55.9 | 342 | 554 | 66.1 | 85.9 | infeasible | 17.9 / 8.09 | 34.2 | voltage |
| ramp descent | 5.2 deg | 19 (68) | 100 | -6.8 / 21.8 | 155 | 633 | -0.6 | 23.7 | 18.4 | 10.2 / 5.30 | 34.2 | voltage |
| **ramp descent** | **7.8 deg** | 21 (57) | 99 | -7.4 / 21.5 | 147 | 475 | 1.2 | 25.9 | 14.6 | 13.4 / 6.55 | 34.2 | voltage |
| ramp descent | 9.2 deg | 17 (51) | 87 | -10.1 / 17.4 | 141 | 410 | -1.3 | 21.2 | 4.0 | 13.4 / 7.15 | 34.2 | voltage |
| stair ascent | 4 in | 21 (110) | 102 | 16.8 / 20.1 | 386 | 945 | 30.3 | 34.1 | 34.6 | 7.1 / 3.68 | 32.9 | none |
| **stair ascent** | **5 in** | 22 (123) | 103 | 22.8 / 25.5 | 357 | 787 | 40.3 | 43.8 | 44.8 | 8.5 / 4.38 | 34.1 | voltage |
| stair ascent | 6 in | 22 (86) | 101 | 28.1 / 30.8 | 339 | 749 | 49.0 | 52.5 | 53.8 | 9.1 / 4.58 | 34.1 | voltage |
| stair descent | 4 in | 21 (141) | 71 | -21.6 / 10.0 | 171 | 656 | -23.7 | 6.1 | -18.3 | 6.7 / 4.09 | 34.2 | voltage |
| **stair descent** | **5 in** | 22 (145) | 77 | -27.2 / 11.7 | 181 | 607 | -29.8 | 7.4 | -22.9 | 9.5 / 4.95 | 34.2 | voltage |
| stair descent | 6 in | 22 (109) | 79 | -30.7 / 12.6 | 190 | 633 | -33.9 | 6.9 | -25.6 | 10.1 / 5.07 | 34.1 | voltage |

## Observations

- **The stiffness changes little; the ratio changes a lot.** Over the conditions with at least 5 subjects, 80 % of the optimal stiffnesses lie between 182 and 258 N·m/rad (representative conditions: 147 to 357), while the ratio spans 475 to 850 over the representative conditions and falls from 1027 at 0.50 m/s to 482 at 1.85 m/s on the treadmill. The voltage limit binds in 32 of 47 conditions: faster joint motion means more back EMF per unit ratio, so the ratio has to drop, and the spring stays near the value that cancels motor motion at push-off.
- **Descent returns energy.** On ramp and stair descent the ankle does net negative work, so with ideal regeneration the motor can charge the battery: stair descent returns 23.7 to 33.9 J per stride, while ramp descent roughly breaks even (-1.3 to 1.2 J) because its smaller negative joint work barely covers the motor losses. Without regeneration the same designs cost 6.1 to 25.9 J. The optimum with regeneration maximizes what is recovered, which is only meaningful if the driver and battery can take it back; section 10 uses both measures.
- Every condition has a design within the drive limits.
- 19 of 47 conditions have a design that also meets the motor's continuous current rating: treadmill 0.50 m/s, treadmill 0.55 m/s, treadmill 0.60 m/s, treadmill 0.65 m/s, treadmill 0.70 m/s, treadmill 0.75 m/s, treadmill 0.80 m/s, treadmill 0.85 m/s, treadmill 0.90 m/s, treadmill 0.95 m/s, treadmill 1.00 m/s, treadmill 1.05 m/s, level ground slow, level ground normal, stair ascent 4 in, stair ascent 5 in, stair ascent 6 in, stair descent 4 in, stair descent 5 in. The fastest treadmill speed among them is 1.05 m/s.
- A rigid actuator meets the drive limits in 27 of 47 conditions that have a feasible SEA design: treadmill up to 1.20 m/s; level ground at slow, normal; ramp ascent at 5.2 deg; every ramp descent condition; every stair ascent condition; every stair descent condition. It fails in the other treadmill, level ground and ramp ascent conditions, the faster and steeper ones, where the joint moves fast at high torque and the back EMF caps the ratio.
- Profiles that rest on one subject or a handful of strides (see the subjects column) are less reliable; the conditions with fewer than 5 subjects (treadmill 1.90 m/s, treadmill 1.95 m/s, treadmill 2.00 m/s, treadmill 2.05 m/s, ramp ascent 9.2 deg) should not be read as a trend. The treadmill profiles use every stride of each subject who walked at that speed.

Section 10 (`results/cross_mode.md`) asks what a design tuned for level walking costs in the other activities, and what a compromise design looks like.
