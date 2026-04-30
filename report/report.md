---
title: "Sizing and torque control of a series elastic actuator for a powered ankle prosthesis"
subtitle: "A design study on the Camargo et al. (2021) lower-limb dataset"
author: "Parmida Mazloomi"
date: "October 2026"
geometry: margin=2.2cm
fontsize: 10pt
mainfont: Charter
linestretch: 1.05
numbersections: true
colorlinks: true
header-includes:
  - \usepackage{float}
  - \floatplacement{figure}{htbp}
abstract: |
  A powered ankle prosthesis must reproduce the large, fast push-off torque of the human ankle from a small battery. This study sizes a series elastic actuator (SEA) for that task from the ankle torque and angle of 22 able-bodied adults in the Camargo et al. (2021) dataset, then designs and tests its torque controller. For level walking at 1.2 m/s and an 80 kg user, the least-energy design with a real motor (maxon EC-4pole 30) within the limits of its driver has a spring of 235 N·m/rad and a gear ratio of 725, and uses 55 % less energy than the best rigid actuator that respects the same limits. One design cannot be optimal everywhere: a design tuned for walking breaks the voltage limit in ramp and stair descent, and a compromise (205 N·m/rad, ratio 603) costs 5 % more in walking. No series spring brings the motor under its continuous current rating; a parallel spring does in walking, but not on stairs. A PID controller with model-based feedforward tracks the torque within 3.3 % RMS in every well-covered condition of a nonlinear simulation with voltage and current limits. A disturbance observer adds little. The weak points are the spring stiffness used as a torque sensor, the margin in swing, and the reference torque itself: a phase-scheduled command misses the actual ankle moment by 10 to 31 %, far more than the controller's tracking error.
---

# Introduction

An SEA puts a spring between a geared motor and the joint. The spring stores and returns energy, lets the motor turn more slowly than the joint during push-off, and doubles as a torque sensor. Its stiffness and the gear ratio set how much energy the motor uses, whether the driver's voltage and current suffice, and how fast the torque can be controlled. This study asks four questions, each answered from public data with one real motor:

1. Which spring stiffness `k` and gear ratio `N` minimize the motor's electrical energy in level walking, within the motor's and driver's limits (Section 3)?
2. How does that design fare on ramps and stairs, and is there one design for a typical day (Section 4)?
3. How well can the spring torque be controlled, with which margins, and what happens under saturation, model error, noise, delay and contact with the user's body (Sections 5 and 6)?
4. How close does a torque command scheduled on gait phase come to the moment a person actually produces (Section 7)?

The problem follows the public descriptions of two graduate wearable-robotics courses, homework 1 and 3 of [ME EN 7960 Wearable Robotics](https://www.mech.utah.edu/grads/course-offerings/) at the University of Utah ([Prof. Tommaso Lenzi](https://www.mech.utah.edu/directory/faculty/tommaso-lenzi/)) and the series and parallel elasticity lectures of [AME 60553 Wearable Robotics](https://werolab.nd.edu/teaching/wr/) at the University of Notre Dame ([Prof. Edgar Bolivar-Nieto](https://ame.nd.edu/faculty/edgar-bolivar-nieto/)). This is an independent study, not a course submission. All code, tests and results are in the repository; every number below is produced by a script and can be regenerated with `python scripts/run_all.py`.

# Data and load

The Camargo et al. (2021) dataset holds OpenSim inverse kinematics and inverse dynamics at 200 Hz for 22 adults on a treadmill (28 speeds from 0.5 to 1.85 m/s, and up to 2.05 m/s for two subjects), on level ground at three self-selected speeds, on ramps (5.2, 7.8 and 9.2 degrees in the files) and on stairs (4, 5 and 6 in step height). Right-leg strides are cut from heel strike to heel strike with the dataset's gait events. A stride is kept when its stance has a single steady locomotion label, its moment has no gap longer than 50 ms, and its duration is within 25 % of the subject's median for that condition. Off the treadmill the ankle moment exists only for stances on a force plate, so overground conditions rest on a few strides per subject (for example 56 strides from 15 subjects for normal level walking), while the instrumented treadmill gives a moment for every stride (676 strides from 22 subjects at 1.2 m/s). Each subject's strides are averaged first, then the subject means; moments are divided by body mass and scaled to a design user of 80 kg, the upper limit of loading level P4 in ISO 10328:2016. Treadmill walking at 1.2 m/s stands in for level walking.

The sign convention was checked on the data: in late stance of level walking, treadmill walking and ramp ascent the moment is plantarflexing, the ankle plantarflexes and the power is positive in 622 of 645, 157 of 157 and 642 of 646 trials. Each averaged profile is fitted with 20 Fourier harmonics, which closes the cycle and provides smooth derivatives.

# Actuator model and sizing for level walking

**Motor.** The maxon EC-4pole 30 (200 W, 36 V winding; torque constant 20.6 mN·m/A, resistance 0.21 Ω, rotor and gearhead inertia 34.0 g·cm², continuous current 5.06 A) with a GP 32 HP gearhead (efficiency 0.70) and an ESCON 70/10 driver (30 A peak, at most 34.2 V from a 36 V bus). Every value is recorded with its datasheet source; the viscous friction and the lossless final screw-and-lever stage are stated assumptions.

**Model.** The spring carries the joint torque $\tau$, so the motor angle is $\theta_m = N(\theta + \tau/k)$. The motor torque is $\tau_m = J\dot\omega_m + b\omega_m + \tau/(N\eta)$, the current $i = \tau_m/k_t$, the voltage $v = Ri + L\,di/dt + k_t\omega_m$, and the energy per stride with ideal regeneration $E = \int vi\,dt$. Written in compliance $\alpha = 1/k$, the motor torque and speed are affine in $\alpha$, and $E(\alpha) = a\alpha^2 + b\alpha + c$ with $a = \int (R/k_t^2)a_1^2\,dt + N^2 b\int\dot\tau^2 dt \ge 0$ over a closed cycle. The energy is therefore a convex quadratic in compliance at every ratio (Bolivar-Nieto et al., 2021), with optimum $\alpha^* = -b/(2a)$. Every instantaneous limit (current, voltage, speed) is affine in $\alpha$ and allows an interval, so the constrained optimum at each ratio is $\alpha^*$ clipped into that interval, and a one-dimensional scan over $N$ finds the design. A $121 \times 121$ grid search over the full model agrees with the closed form to rounding error.

**Result.** The least-energy design within the drive limits is $k = 235$ N·m/rad (2.94 N·m/rad per kg of user mass) and $N = 725$, at 23.4 J per stride (22.6 W), on the voltage limit (Figure 1). The best rigid actuator within the same limits needs $N = 302$ and 52.5 J: without a spring the back EMF at the push-off joint speed caps the ratio, and at that low ratio the push-off torque needs a large current. The spring lets the motor turn more slowly than the joint at push-off, so the SEA can afford a higher ratio. With $J = b = 0$ the spring would not change the energy at all; it saves energy here only because the reflected rotor inertia $\eta J N^2$, about 1 kg·m², is a hundred times the foot's.

![Motor energy per stride over spring stiffness and gear ratio for level walking at 1.2 m/s (80 kg user, 36 V bus). Veiled designs break the voltage, peak current or speed limit. Data: Camargo et al. (2021), CC BY 4.0.](../results/figures/sizing_levelwalk.png){width=62%}

**Heat.** No design on the grid meets the motor's continuous current rating: the lowest RMS current is 5.11 A, and the least-energy design draws 5.73 A (13 % above the rating). The copper loss is set by the joint torque reflected through the gear, which a series spring cannot reduce. The sizing holds up to the assumptions within reason: the optimal stiffness stays near 3 N·m/rad per kg for 60 and 100 kg users, a 24 or 48 V bus mainly moves the ratio, and only a 60 kg user or a 0.9 efficient gear make the design thermally feasible.

# One actuator for all activities

Sizing every one of the 47 conditions separately shows that the optimal stiffness changes little (80 % of the optima between 182 and 258 N·m/rad over conditions with at least five subjects) while the ratio changes a lot (from 1027 at 0.5 m/s to 482 at 1.85 m/s on the treadmill), because faster joint motion means more back EMF per unit ratio. The voltage limit binds in 32 of 47 conditions. Stair descent returns 24 to 34 J per stride with ideal regeneration.

A prosthesis has one spring and one transmission. The design tuned for walking ($k = 235$, $N = 739$ from the closed form) breaks the voltage limit in ramp descent and stair descent (43 and 40 V needed). The compromise, the least step-weighted energy over a free-living activity mix of transtibial prosthesis users (91.8 % level walking; Srisuwan and Klute, 2021) subject to feasibility in all five activities, is $k = 205$ N·m/rad and $N = 603$. It is set by the limits, not by the weights: ramp descent fixes the ratio, and level walking pays 1.3 J per stride (5 %). Over a day this is 14.5 Wh at the motor terminals against an unreachable lower bound of 13.7 Wh. Tripling the share of ramps and stairs, weighting all activities equally, or optimizing for walking alone within the limits gives exactly the same design; the walking speed, the steepest ramps and stairs, the energy measure (without regeneration) and the bus voltage do move it. The hardest activity sizes the transmission.

**Parallel spring.** A linear spring in parallel with the actuator carries part of the joint torque, which is what the series spring cannot do. Its stiffness and rest angle enter the motor current and energy as another convex quadratic, so the design alternates two closed-form steps. For level walking the result is a 302 N·m/rad parallel spring, a rigid series path and $N = 309$, which meets every limit including the thermal one (4.34 A RMS, 23 % lower) at 19.3 J per stride; a brute-force grid confirms it. The same spring, however, fights the joint on stairs, where the moment-angle relation differs: the RMS current rises to 13 A in stair ascent and 16 A in stair descent. A spring that engages only in stance, or one that can be disengaged, would be needed.

# Torque control

**Plant.** With the joint held (stance), the spring torque responds to motor torque as $N\eta k/(\eta J N^2 s^2 + \eta b N^2 s + k)$: a resonance at 2.5 Hz for the compromise design, damped at a ratio of only 0.054. With the foot free (swing, foot inertia 0.01 kg·m² assumed), the resonance moves to 23 Hz. In walking, 56 % of the joint-velocity power lies above the stance resonance, so the joint motion is a disturbance the controller must handle.

**PID with feedforward.** The targets were set before tuning: closed-loop bandwidth of at least 6 Hz in stance (99.9 % of the torque power of every activity except stair descent lies below 6 Hz); with the 1.5 ms loop delay of a 1 kHz controller, a phase margin of at least 45 deg and a gain margin of at least 10 dB in stance and in swing, and peak sensitivity at most 2; zero steady-state error; RMS tracking error at most 5 % in the nonlinear simulation. A PD controller places the stance poles at 8 Hz with damping 0.7, a small integral term (corner at 0.2 of that) removes steady-state error, and the derivative is filtered. A feedforward inverts the plant model: the motor torque that produces $\tau_{ref}$ while the joint moves along $\theta$ is $[(M + k)\tau_{ref}/k + M\theta]/(N\eta)$ with $M = \eta N^2(Js^2 + bs)$, which is exactly the motor torque of the sizing model.

| target | compromise design, PID | met |
|---|---|---|
| bandwidth at least 6 Hz | 21.0 Hz | yes |
| phase margin at least 45 deg | 52 deg stance, 44.8 deg swing | stance only |
| gain margin at least 10 dB | 19.4 dB stance, 18.6 dB swing | yes |
| peak sensitivity at most 2 | 1.29 stance, 1.49 swing | yes |
| zero steady-state error | integral action | yes |
| RMS tracking error at most 5 % | 3.3 % worst (treadmill 1.85 m/s) | yes |

In the linear model, feedback alone tracks level walking to 2.7 % RMS, a static feedforward $\tau_{ref}/(N\eta)$ brings the worst activity to 1.8 %, and the full model feedforward to 0.11 %, limited only by the loop delay. In swing the foot resonance pushes the crossover to 29 Hz, where the delay costs more phase; lowering the gains in swing should restore the margin but is not analyzed.

**Disturbance observer.** Following Paine et al. (2014), a disturbance observer (DOB) compares the measured spring torque with what the nominal stance model predicts for the command, and cancels the difference below the cut-off of a second-order low-pass. It is paired with the same PD gains (it supplies integral action itself). Its cut-off is the highest that keeps the stance targets: 5.5 Hz for the compromise design. With the model feedforward it tracks exactly as well as the PID (0.11 %); without the joint acceleration (reference feedforward only) it is slightly better (2.4 % against 2.8 %). A DOB pays off when its cut-off can sit well above the band an integral term covers; the delay and the low open-loop resonance of a high-ratio design leave no room for that here.

**Passivity.** Vallery et al. (2007) judge SEA torque controllers by passivity at the joint, since a passive actuator stays stable against any passive environment such as the user (Colgate and Hogan, 1988). With zero torque reference the joint sees $Z(s) = -\tau_s/(s\theta)$, which was evaluated numerically from 0.0016 to 1592 Hz. Without delay, PD control is passive, the PID's integral term makes $\mathrm{Re}\,Z$ negative below 2.6 Hz (down to $-0.29$ N·m·s/rad) and the DOB below 4.3 Hz, and exact model feedforward makes $Z = 0$. With the 1.5 ms delay no configuration is strictly passive; with the feedforward, $\mathrm{Re}\,Z$ reaches $-0.10$ N·m·s/rad at 5.4 Hz. Vallery et al. conclude that a cascade with a fast inner motor-velocity loop allows integral action while staying passive; that structure is not implemented here.

# Nonlinear tracking and robustness

The time-domain simulation runs the winding and rotor at 20 kHz, a PI current loop (1 kHz bandwidth) with back-EMF compensation, voltage clipped at 34.2 V and current at 30 A, and the torque controller at 1 kHz (Tustin, one sample of computation delay). The torque is measured as nominal stiffness times spring deflection, and the joint velocity and acceleration for the feedforward come from a 25 Hz filter on the sampled angle. The joint motion is prescribed. With the limits removed, the simulated energy and currents agree with the sizing model within 1 %.

![Simulated spring torque, motor current and winding voltage over the last stride of treadmill walking at 1.2 m/s and stair ascent (6 in), both designs, PID with model feedforward. Dotted lines: driver limits; dash-dot: continuous current rating. Data: Camargo et al. (2021), CC BY 4.0.](../results/figures/tracking.png){width=62%}

The compromise design tracks level walking to 0.60 % RMS, most of it from the filtered joint acceleration, and meets the 5 % target in every condition with at least five subjects; its worst case is treadmill walking at 1.85 m/s, 3.3 % with the voltage saturated over 25 % of the stride (Figure 2). The walking-tuned design misses the target there (5.5 %) and saturates in five of eight conditions: both designs were sized exactly onto the voltage limit, so any feedback correction pushes the drive into saturation. At 2.05 m/s (two subjects) even the compromise misses (9.0 %). The DOB and the PID perform alike.

![Robustness of the compromise design. Top left: tracking error against the ratio of actual to assumed spring stiffness, torque from the spring deflection or from a load cell. Top right: tracking error against joint-angle noise. Bottom left: phase margin against loop delay. Bottom right: coupled stability of the PID with model feedforward against passive environments. Data: Camargo et al. (2021), CC BY 4.0.](../results/figures/robustness.png){width=72%}

The robustness study keeps the controller's nominal model while the plant changes (Figure 3):

- **Spring stiffness** matters most, through the sensor: torque computed as nominal stiffness times deflection reads 25 % high on a spring 20 % softer than assumed, and the worst error grows from 0.7 % to 11.2 % (5.6 % at 10 %). With a load cell the same spring costs 1.1 %. The spring must be calibrated on the assembled actuator.
- **Motor constants** (torque constant ±10 %, rotor inertia +30 %, friction ×3) keep the error at or below 1.0 %.
- **Encoder noise** enters through the joint acceleration estimate; the error doubles at 6.6 mrad RMS, 60 times the quantization noise of a 14-bit encoder.
- **Delay** costs about 4 deg of phase margin per millisecond in stance and 10 deg in swing. The delay margin is 11.8 ms in stance but 4.3 ms in swing, where the DOB loop is unstable at 5.5 ms of total delay.
- **Coupled stability.** Coupling the actuator's joint-motion response to a grid of passive mass-spring-damper environments shows that PD alone is stable everywhere, but PID and DOB with model feedforward are unstable with a light inertia (0.01 kg·m², a foot) on a soft, lightly damped spring of 1 to 32 N·m/rad, which resonates at 2 to 9 Hz, the band where the delayed feedforward is non-passive. Environment damping of 0.10 N·m·s/rad, the size of that passivity violation, restores stability. The joint-motion feedforward and the integrator should be faded out in swing.

# Phase-scheduled torque command

Low-level torque control executes a reference chosen by the higher levels of a prosthesis controller (Tucker et al., 2015). The simplest reference is a phase schedule, $\tau_{ref}(t) = m \cdot \mathrm{profile}(\phi(t))$: the mean moment per kg of the current activity, looked up at the gait phase $\phi$ and scaled by the user's mass. For each of 19,894 kept strides the command is compared, sample by sample, with the subject's own inverse-dynamics moment. The profile is the mean over the *other* 21 subjects, so the command is never tuned to the person it is tested on, and the activity is given. Two phase sources are compared: an oracle (linear in time between the dataset's heel strikes, so it knows when the stride will end) and a causal time-based estimate (time since the last detected heel strike over the previous stride's duration), with heel strikes detected 0, 25 or 50 ms late. The time-based estimate stands in for an IMU-based phase estimator, planned as a separate project.

![Phase-scheduled torque command. Left: RMS error between command and inverse-dynamics moment per stride for each phase source. Middle: mean phase error of the time-based estimate over the gait cycle. Right: RMS command error over the gait cycle, oracle (solid) and time-based with 25 ms latency (dashed). Data: Camargo et al. (2021), CC BY 4.0; 22 subjects, leave-one-subject-out profiles.](../results/figures/phase_schedule.png)

Even with the oracle, a generic profile misses the actual moment by 10 to 31 % of the stride peak (Figure 4). On the treadmill, where each subject has enough strides, the subject's own mean profile halves that error (10.1 to 5.5 %): about half of it is the difference between people. The time-based estimate costs 1.0 percentage point on the treadmill, and 1.3 to 3.6 points on stairs and ramps, where 88 % of the steady strides are the first of a short segment and their duration is predicted from a stride of another activity (those first strides cost 3.4 points; the 106 later ones, on average, nothing). Detection latency shifts the whole command and costs about 1.6 points per 10 ms on the treadmill. For scale, the torque controller tracks its reference to 0.6 %: in a phase-scheduled ankle the reference, not the actuator, limits how closely the device reproduces the ankle.

# What did not work

- **The first PID tuning.** Poles at 6 Hz with the integral corner at 0.1 of that met the bandwidth target and the stance margins, but feedback alone left 6.3 % RMS error. A $-3$ dB bandwidth says nothing about how flat the response is below it, and a gait torque has its power at the first stride harmonics. Moving to 8 Hz and 0.2 fixed it; the integral corner cannot go much higher without making the loop conditionally stable.
- **Where the delay sits.** The first linear model put the loop delay on the torque measurement only, and the model feedforward showed a residual error that did not exist with the delay on the whole command, where the real computation delay acts.
- **Compensating the joint motion twice.** The first DOB treated the joint motion as a disturbance while the feedforward already cancelled it; the joint motion now enters the observer's nominal model.
- **Observer windup.** The first simulation fed the DOB the torque it had *commanded*. Under voltage saturation it read the shortfall as a disturbance and deepened the saturation: 57.6 % RMS error at 2.05 m/s. Feeding it the torque the motor produced ($k_t$ times the measured current) brought it to 8.8 %; freezing the PID integrator during voltage saturation helped the PID only a little (9.6 to 9.0 %).
- **Designs sized onto the voltage limit.** Energy-optimal designs sit exactly on the voltage limit, which leaves nothing for control. A device should be sized against 85 to 90 % of the available voltage.
- **The SEA alone is thermally infeasible** for an 80 kg user with this motor; the parallel spring that fixes walking overloads the motor on stairs.
- **The DOB did not beat the PID.** With 1.5 ms delay and a 2.5 Hz plant resonance, its cut-off cannot rise above the band an integral term already covers.
- **The time-based phase estimate on stairs.** Its prediction comes from a stride of another activity on most stair strides.

# Limitations

The reference is the able-bodied ankle averaged over strides; amputee gait differs, and mean profiles hide stride-to-stride variation (Section 7 quantifies it). Overground conditions rest on few strides per subject, ramp ascent on 4 to 10 subjects per incline and speeds above 1.85 m/s on two subjects. The final screw-and-lever stage is lossless and massless, the gear efficiency constant, the winding resistance cold, and heat is judged by RMS current against the continuous rating rather than by a thermal model. The simulation prescribes the joint motion and so cannot show instabilities that need a free foot; these are covered only by the linear coupled-stability analysis. Passivity is checked but not achieved, and the cascaded velocity loop that would allow it is not implemented. Mode recognition is given to the controller, the phase estimator is the simplest causal one, and nothing was tested on hardware.

# References

Bolivar-Nieto, E. A., Thomas, G. C., Rouse, E., Gregg, R. D. (2021). Convex optimization for spring design in series elastic actuators: from theory to practice. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 9327-9332.

Camargo, J., Ramanathan, A., Flanagan, W., Young, A. (2021). A comprehensive, open-source dataset of lower limb biomechanics in multiple conditions of stairs, ramps, and level-ground ambulation and transitions. *Journal of Biomechanics* 119, 110320. doi:10.1016/j.jbiomech.2021.110320. Data CC BY 4.0.

Colgate, J. E., Hogan, N. (1988). Robust control of dynamically interacting systems. *International Journal of Control* 48(1), 65-88.

ISO 10328:2016. Prosthetics: structural testing of lower-limb prostheses, requirements and test methods. International Organization for Standardization.

maxon (2021). Program 2021/22 catalog (March 2021): EC-4pole 30, 200 W, part 305014 (p. 261); GP 32 HP planetary gearhead, part 326666 (pp. 400-401). ESCON 70/10 Hardware Reference, edition 2021-08. Values and URLs in `data/motors/maxon_ec4pole30_305014.yaml`.

Paine, N., Oh, S., Sentis, L. (2014). Design and control considerations for high-performance series elastic actuators. *IEEE/ASME Transactions on Mechatronics* 19(3), 1080-1091. doi:10.1109/TMECH.2013.2270435.

Srisuwan, B., Klute, G. K. (2021). Locomotor activities of individuals with lower limb amputation. *Prosthetics and Orthotics International* 45(3), 191-197. doi:10.1097/PXR.0000000000000009.

Tucker, M. R., Olivier, J., Pagel, A., Bleuler, H., Bouri, M., Lambercy, O., Millán, J. del R., Riener, R., Vallery, H., Gassert, R. (2015). Control strategies for active lower extremity prosthetics and orthotics: a review. *Journal of NeuroEngineering and Rehabilitation* 12, 1. doi:10.1186/1743-0003-12-1.

Vallery, H., Ekkelenkamp, R., van der Kooij, H., Buss, M. (2007). Passive and accurate torque control of series elastic actuators. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 3534-3538.
