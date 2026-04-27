# Phase-scheduled torque command

The torque reference of a phase-scheduled ankle controller is `m * profile(phase)`: the mean ankle moment per kg of the current activity and condition, looked up at the estimated gait phase and scaled by the user's mass `m`. This step asks how close that command comes to the moment the subject actually produced (inverse dynamics in the Camargo et al. (2021) dataset), stride by stride at 200 Hz, and how much of the gap comes from the phase estimate. Script: `scripts/phase_schedule.py`; code: `src/anklesea/phase.py`.

## Setup

- **Strides**: the 2085 kept right-leg strides of 2 subjects (AB06, AB07) from `scripts/gait_profiles.py`.
- **Profile**: the mean moment per kg of the same activity and condition over the *other* subjects, so the command is a generic one, not tuned to the person it is tested on. One row also uses the subject's own mean profile, which is the best a fixed profile tuned to that user can do. With 2 subjects the generic profile of each subject is the other subject's mean.
- **Activity and condition** are given to the controller. Recognizing them from sensors is a separate problem (see the companion repository imu-locomotion-gait-phase).
- **Oracle phase**: linear in time between the dataset's heel strikes (`gcRight`). It knows when the current stride will end, which no device does.
- **Time-based estimate**: the time since the last detected heel strike over the duration of the previous stride, held at 99.9 % if the stride runs long and reset at the next detection; the mean stride time of the profile is used when there is no plausible previous stride (start of a trial or after a pause). The heel strike is detected 0, 25, 50 ms after the dataset's event. Detection latency is an assumption (no detector is modeled; `docs/assumptions.md`). **This estimator stands in for the IMU-based gait phase estimator of the companion repository imu-locomotion-gait-phase, which has no estimator yet.** It is the simplest causal estimator, not a recommendation.
- **Error**: RMS of command minus inverse-dynamics moment over the stride, in percent of that stride's peak moment, averaged per subject and then across subjects.

## Torque command error (RMS, % of stride peak)

| | treadmill | level ground | ramp ascent | ramp descent | stair ascent | stair descent |
|---|---|---|---|---|---|---|
| oracle, own profile | 4.0 | 4.4 | 4.4 | 6.4 | 8.6 | 13.4 |
| oracle | 18.1 | 10.9 | 13.5 | 24.2 | 15.4 | 26.2 |
| time, 0 ms | 18.4 | 11.3 | 18.4 | 28.3 | 19.8 | 28.7 |
| time, 25 ms | 20.1 | 10.8 | 16.6 | 27.9 | 22.0 | 31.5 |
| time, 50 ms | 24.0 | 12.2 | 16.9 | 28.9 | 25.1 | 34.7 |

Subjects and strides per activity:

| | treadmill | level ground | ramp ascent | ramp descent | stair ascent | stair descent |
|---|---|---|---|---|---|---|
| subjects | 2 | 1 | 2 | 2 | 2 | 2 |
| strides | 1993 | 2 | 6 | 21 | 31 | 32 |

## Phase error of the time-based estimate (RMS, % of the gait cycle)

| | treadmill | level ground | ramp ascent | ramp descent | stair ascent | stair descent |
|---|---|---|---|---|---|---|
| time, 0 ms | 1.0 | 0.8 | 4.3 | 3.5 | 4.1 | 4.0 |
| time, 25 ms | 2.6 | 2.2 | 3.3 | 4.1 | 5.4 | 6.5 |
| time, 50 ms | 4.9 | 3.9 | 3.6 | 5.4 | 6.9 | 8.6 |

## First stride of a segment against later strides (no detection delay)

A stride is the first of a segment when the stride before it is not a kept stride of the same activity and condition: the first steady stride on a staircase or ramp, the first treadmill stride at a new belt speed, or a level-ground stride, since only one stride per pass has a force plate under it.

|  | phase error (% cycle) | torque error, oracle (%) | torque error, time 0 ms (%) | strides |
|---|---|---|---|---|
| treadmill, later strides | 0.9 | 18.0 | 18.3 | 1925 |
| treadmill, first stride of a segment | 2.2 | 18.9 | 20.6 | 68 |
| level ground, first stride of a segment | 0.8 | 10.9 | 11.3 | 2 |
| ramp ascent, first stride of a segment | 4.3 | 13.5 | 18.4 | 6 |
| ramp descent, first stride of a segment | 3.5 | 24.2 | 28.3 | 21 |
| stair ascent, later strides | 5.0 | 19.5 | 18.1 | 3 |
| stair ascent, first stride of a segment | 4.1 | 14.8 | 19.9 | 28 |
| stair descent, later strides | 5.2 | 24.9 | 23.3 | 6 |
| stair descent, first stride of a segment | 3.7 | 26.4 | 29.9 | 26 |

## Reading the results
Level ground and ramp ascent rest on fewer than 20 strides (only one or two right-leg stances per pass land on a force plate, `results/gait_profiles.md`), so their columns are noise at this sample size and the summary below leaves them out.

- **Even with the oracle phase a generic profile misses by 15 to 26 %** of the stride peak. A fixed profile cannot follow stride-to-stride and person-to-person variation. The subject's own mean profile (from the subject's other strides) brings it to 4 to 13 %: 44 to 78 % of the generic error is the difference between people, not between strides. Tuning the profile to the user gains 7 to 18 points, more than replacing the time-based estimate with the oracle (next two points). With only 2 subjects the generic profile is a poor population mean, so this gap will shrink with more subjects.
- **On treadmill the time-based estimate is nearly as good as the oracle**: its phase error is 0.9 % of the cycle on strides at a steady speed and 2.2 % on the first stride after a speed change or a dropped stride, and it adds +0.3 percentage points to the command error.
- **On stairs and ramps it is worse**, adding +2.5 to +4.4 points. Steady stair and ramp segments are only a few strides long, so 90 % of their kept strides are the first of a segment, and the estimator predicts their duration from a stride of another activity (level walking or a transition). On those first strides it adds +4.5 points on average.
- **Detection latency shifts the whole command later**, including the steep rise and fall of push-off. On treadmill the error grows from 18.4 % without latency to 24.0 % at 50 ms, about 1.1 points per 10 ms. If the latency is known and constant, the estimator can back-date the heel strike and return to the no-latency row.
- **For scale**: the torque controller tracks its reference to 0.4 % RMS on treadmill walking at 1.2 m/s (`results/tracking.md`), far below these command errors. In a phase-scheduled ankle the reference limits the result, not the actuator.

![phase schedule](figures/phase_schedule.png)

*Figure: left, torque command error per activity for each phase source; middle, mean phase error of the time-based estimate over the gait cycle; right, RMS command error over the gait cycle for treadmill and stair ascent, oracle phase (solid) and the time-based estimate with 25 ms detection latency (dashed). Data: Camargo et al. (2021), CC BY 4.0; 2 subjects, leave-one-subject-out profiles.*
