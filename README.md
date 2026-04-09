# Powered Ankle Prosthesis: Series Elastic Actuator Sizing and Torque Control

A design study in Python. It sizes the spring stiffness and gear ratio of a series elastic actuator (SEA) for a powered ankle prosthesis from human ankle torque and angle data, so that the motor uses as little electrical energy as possible, and then designs and tests the actuator's torque controller. The task follows homework 1 and homework 3 of **ME EN 7960 Wearable Robotics** at the University of Utah, with the motor selection and series and parallel elasticity material of **AME 60553 Wearable Robotics** at the University of Notre Dame. The data are the public lower-limb dataset of Camargo et al. (2021), so the design can be sized for level walking, ramps and stairs, not only for one walking speed.

This repository is part of a set that goes from a body-worn sensor to a device controller:
- [imu-opensim-validation](https://github.com/mzlumi/imu-opensim-validation): IMU joint angles validated against marker-based motion capture.
- [imu-locomotion-gait-phase](https://github.com/mzlumi/imu-locomotion-gait-phase): locomotion mode and continuous gait phase from wearable IMUs, on the same Camargo dataset. Its phase estimator schedules the torque command here.
- [drop-foot-ankle-exo-sim](https://github.com/mzlumi/drop-foot-ankle-exo-sim): an IMU-triggered ankle exoskeleton in a neuromuscular simulation of drop-foot gait.

## Adapted from

### University of Utah, ME EN 7960-001 Wearable Robotics

- **Instructor:** Prof. Tommaso Lenzi, with teaching assistants Lukas Gabert, Sergei Sarkisian and Dante Archangeli.
- **Term:** Fall 2022 (the syllabus used here). The university class schedule shows the course again in Fall 2024 with the same instructor.
- **The course:** a graduate overview of the design, modeling and control of powered exoskeletons and robotic limb prostheses. Lectures and readings cover robot morphology, actuation (DC motors, series and parallel elastic actuators, variable stiffness and variable transmission actuators), biosignals (gait kinematics, EMG, indirect calorimetry) and control at three levels (low-level torque control, mid-level impedance and kinematic control, high-level finite-state and phase-based control), followed by case studies. Students also work on the open-source robotic leg prosthesis developed at the Bionic Engineering Lab. Homework is in MATLAB and Simulink.
- **What the original task asked:**
  - *HW 1, Conceptual Design of Advanced Actuation Systems:* "Based on considerations on Human Biomechanics students will dimension an advanced actuation system such as the one seen in class. (e.g., Dimension a series elastic actuator to minimize the energy consumption of a powered ankle prosthesis)".
  - *HW 3, Closed-loop Control of Advanced Actuation Systems:* "Based on the dynamic models of advanced actuation system students will develop and simulate a low-level closed-loop control system for a wearable robot". The low-level control lecture assigns Vallery et al. (IROS 2007) on passive and accurate SEA torque control and Paine et al. (IEEE/ASME TMech 2014) on the design and control of high-performance SEAs.
  - The full homework handouts are on the course Canvas site, which needs a University of Utah login. Only the syllabus descriptions above are public.
- **Links:** [syllabus (PDF)](https://rmcoeh.com/images/pdfs-doc/courses-syllabi/MEEN%207960%20Syllabus.pdf), [Utah Robotics Center course list](https://robotics.coe.utah.edu/academics/courses/), [Fall 2022 class schedule](https://class-schedule.app.utah.edu/main/1228/class_list.html?subject=ME+EN).

### University of Notre Dame, AME 60553 Wearable Robotics

- **Instructor:** Prof. Edgar Bolivar-Nieto (Wearable Robotics Laboratory).
- **Term:** first offered in Fall 2023 (the handwritten lecture notes used here are from that term). It is taught again in Fall 2026, with a public course repository.
- **The course:** the biomechanical and mechatronic principles behind wearable robots: analysis of human movement (inverse kinematics and dynamics in OpenSim), selection of brushed and brushless DC motors, the role of series and parallel elasticity, transmissions and winding heat, convex and robust optimization for mechanical design and real-time control, and control of lower-limb prostheses and orthoses (impedance, finite-state machines, phase variables). The course page says its organization follows Elliott Rouse's wearable robotics course at the University of Michigan.
- **What the original material asks:**
  - *Assignment 5 (Fall 2026), motor selection for an elbow prosthesis:* "Pick the lightest brushed DC motor (plus a reduction ratio) that can drive an elbow prosthesis through repeated lifting cycles", with the load torque against velocity, the motor's feasible speed and torque region, the motor voltages and the energy over ten cycles. The full statement is on the course Canvas and Gradescope and is not public.
  - *Lectures 12 and 13 (Fall 2023), the role of series and parallel elasticity:* motor energy written as a quadratic function, for series and parallel elastic actuators, with the reading Bolivar-Nieto et al., "Convex optimization for spring design in series elastic actuators: from theory to practice" (IROS 2021).
  - *Lectures 20 and 21 (Fall 2023):* control of lower-limb powered prostheses and orthoses.
- **Links:** [course page](https://werolab.nd.edu/teaching/wr/), [Notre Dame Robotics course list](https://robotics.nd.edu/academics/courses/), [course repository WeRoLab/WR26](https://github.com/WeRoLab/WR26).

The saved documents and their original locations are listed in [`docs/SOURCES.md`](docs/SOURCES.md).

## The problem

**Actuator sizing.** The prosthesis ankle must reproduce the ankle torque τ(t) and angle θ(t) of an able-bodied person. An SEA puts a spring of stiffness k between a geared motor (ratio N) and the joint. For a given (k, N), the spring deflection is τ/k, so the motor angle is θ_m = N(θ + τ/k). The motor torque must supply τ/(Nη) through a gearbox of efficiency η, plus the torque to accelerate the reflected rotor inertia and to overcome friction. The current is that torque divided by the torque constant. Electrical power is the copper loss i²R plus the mechanical power, and the energy per stride is its integral, with and without regeneration. A design is feasible only if the required voltage stays below the bus voltage and the peak and RMS currents stay within the motor's limits.

1. Build the load from the Camargo et al. data: cut ankle torque and angle into strides with the gait events, remove strides with gaps, normalize to the gait cycle, and average per mode and condition (treadmill speed, ramp incline, stair height), scaled to a chosen user mass.
2. Model a real motor and gearbox from a manufacturer's datasheet (cited in [`data/README.md`](data/README.md)).
3. Sweep k and N on a grid, compute energy per stride and the feasibility constraints, and find the optimum. Check the grid against the convex formulation from the Notre Dame material: for a fixed gear ratio, energy is a quadratic function of the spring compliance, so the optimum has a closed form.

**Torque control.** With the chosen design, model the SEA as a plant from motor current to spring torque, with the ankle motion as a disturbance. Design a torque controller (PID with model-based feedforward), then a disturbance-observer variant in the spirit of the assigned readings. Show the open- and closed-loop Bode plots, the bandwidth and stability margins, and the effect of voltage and current saturation. Then simulate the closed loop tracking the gait torque profile of each mode.

## Data

[Camargo et al. (2021)](https://doi.org/10.1016/j.jbiomech.2021.110320): 22 able-bodied adults on a treadmill at 28 speeds, on level ground at three speeds, on ramps at six inclines and on stairs at up to four step heights, with OpenSim inverse kinematics and inverse dynamics at 200 Hz. CC BY 4.0. The data are not stored here; [`scripts/fetch_data.py`](scripts/fetch_data.py) downloads the needed folders into the git-ignored `data/raw/`. [`data/README.md`](data/README.md) describes the layout, the license, and what was checked in the files (the `.mat` files hold MATLAB tables that SciPy cannot read, the ankle moment is in N·m and not normalized, and not every subject has every stair height).

## Going beyond the coursework

- **All locomotion modes.** The homework sizes the actuator for level walking. Here the sizing is done for every mode and condition, to show how the optimal stiffness and gear ratio move between walking, ramps and stairs.
- **The cost of tuning for walking.** How much extra energy a design tuned for level walking uses on stairs and ramps, compared with each mode's own optimum, and whether one compromise design (weighted over modes) loses much anywhere.
- **A parallel spring.** Add a parallel spring option and compare energy and peak current with the pure SEA.
- **Phase-scheduled torque command.** Generate the torque reference from gait phase and mode, using the estimator from [imu-locomotion-gait-phase](https://github.com/mzlumi/imu-locomotion-gait-phase), and compare tracking with the dataset's own gait events as an oracle.
- **Robustness.** Tracking and stability with errors in spring stiffness and motor constants, and with sensor noise.

## Validation

- The energy model is tested on cases with known answers (rigid actuator as k grows large, zero inertia, sinusoidal loads with a closed-form energy).
- The grid search is checked against the closed-form optimum in spring compliance for fixed gear ratio.
- The stride-averaged ankle torque and angle are compared with the profiles reported in the dataset paper, and the sign convention is checked against the angle data.
- The linear controller analysis is checked against the nonlinear time-domain simulation, and saturation is reported where it occurs.
- Results that fail or do not hold up are reported, not hidden.

## Status

- [x] Course documents and sources ([`docs/SOURCES.md`](docs/SOURCES.md))
- [x] Dataset description and download script
- [ ] Package scaffolding, tests and CI
- [ ] Camargo data loader, data inventory, stride segmentation and averaged gait profiles per mode
- [ ] Motor and gearbox model from a real datasheet
- [ ] SEA energy model with constraints, with tests
- [ ] Sizing sweep for level walking, checked against the convex closed form
- [ ] Sizing across all modes, cost of a walking-tuned design, design table
- [ ] Parallel spring option
- [ ] SEA plant model, torque controller, Bode plots, disturbance observer
- [ ] Time-domain tracking with saturation, robustness study
- [ ] Phase-scheduled torque command
- [ ] Report and final README

## Credit

The homework descriptions, lecture notes, course pages and assignment text belong to the University of Utah (Prof. Tommaso Lenzi and the ME EN 7960 teaching staff) and the University of Notre Dame (Prof. Edgar Bolivar-Nieto and the AME 60553 teaching staff). They are kept in `docs/source/` for private reference only, and `docs/source/` must be removed before this repository is made public. The dataset is by Camargo, Ramanathan, Flanagan and Young (Georgia Tech EPIC Lab), used under CC BY 4.0. The code is written for this project by Parmida Mazloomi with the help of AI coding tools, from the public course descriptions and the cited papers; no code from past students' solutions and no course starter code is used.
