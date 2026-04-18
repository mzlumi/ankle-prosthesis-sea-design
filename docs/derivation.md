# Derivation: motor energy of a series elastic actuator, and why it is convex in compliance

This note derives the energy model in `src/anklesea/sea.py` and the closed form in `src/anklesea/sizing.py`. The approach follows the Notre Dame AME 60553 lecture notes (L12 and L13) and E. A. Bolivar-Nieto, G. C. Thomas, E. Rouse and R. D. Gregg, "Convex optimization for spring design in series elastic actuators: from theory to practice", IEEE/RSJ IROS 2021. All quantities are SI.

## 1. The actuator and its inputs

A motor (rotor and gearhead inertia $J$ at the motor shaft, viscous friction $b$, torque constant $k_t$, winding resistance $R$ and inductance $L$) drives a transmission of ratio $N$ and efficiency $\eta$. A spring of stiffness $k$, or compliance $\alpha = 1/k$, sits between the transmission output and the ankle.

The task gives the ankle angle $\theta(t)$ and torque $\tau(t)$ over one stride of period $T$. They come from the averaged gait profiles, fitted with a Fourier series so that every derivative exists and the cycle is closed: $x(T) = x(0)$ for every signal and its derivatives. Positive angle and torque are dorsiflexion, so the joint power is $\tau\dot\theta$.

## 2. From the joint to the motor

The spring carries the whole joint torque:

$$\tau = k\left(\frac{\theta_m}{N} - \theta\right) \quad\Rightarrow\quad \theta_m = N(\theta + \alpha\tau).$$

Differentiating,

$$\omega_m = N(\dot\theta + \alpha\dot\tau), \qquad \dot\omega_m = N(\ddot\theta + \alpha\ddot\tau).$$

The motor must accelerate its own inertia, overcome friction and supply the joint torque through the transmission. With a constant efficiency applied as a divisor,

$$\tau_m = J\dot\omega_m + b\,\omega_m + \frac{\tau}{N\eta}.$$

The electrical side is a DC motor model:

$$i = \frac{\tau_m}{k_t}, \qquad v = R\,i + L\frac{di}{dt} + k_t\,\omega_m, \qquad P = v\,i.$$

No dynamics are solved: the joint trajectory is prescribed, so every motor quantity follows algebraically. This is the inverse-dynamics view of actuator design.

## 3. Energy over a stride

The energy drawn from the battery with ideal regeneration is

$$E = \int_0^T v\,i\,dt = \int_0^T \left(R\,i^2 + L\,i\frac{di}{dt} + k_t\,\omega_m\,i\right)dt.$$

The inductive term is $\frac{L}{2}\left[i^2\right]_0^T = 0$ over a closed cycle, and $k_t\,i = \tau_m$, so

$$E = \int_0^T \left(\frac{R}{k_t^2}\tau_m^2 + \tau_m\,\omega_m\right)dt.$$

The first term is the copper loss and the second the mechanical work done by the motor.

## 4. Everything is affine in compliance

Substituting section 2, at every instant

$$\tau_m = a_1\alpha + b_1, \qquad \omega_m = a_2\alpha + b_2,$$

with

$$a_1 = N(J\ddot\tau + b\dot\tau), \quad b_1 = N(J\ddot\theta + b\dot\theta) + \frac{\tau}{N\eta}, \quad a_2 = N\dot\tau, \quad b_2 = N\dot\theta.$$

The rigid actuator is $\alpha = 0$: then $\tau_m = b_1$ and $\omega_m = b_2$.

## 5. The energy is a convex quadratic in compliance

Inserting the affine forms into section 3 gives

$$E(\alpha) = a\,\alpha^2 + b\,\alpha + c$$

with

$$a = \int_0^T \left(\frac{R}{k_t^2}a_1^2 + a_1 a_2\right)dt, \quad
b = \int_0^T \left(\frac{2R}{k_t^2}a_1 b_1 + a_1 b_2 + a_2 b_1\right)dt, \quad
c = \int_0^T \left(\frac{R}{k_t^2}b_1^2 + b_1 b_2\right)dt.$$

**Convexity.** The first part of $a$ is a square. The second is

$$\int_0^T a_1 a_2\,dt = N^2\int_0^T (J\ddot\tau\dot\tau + b\,\dot\tau^2)\,dt = N^2\left(\frac{J}{2}\left[\dot\tau^2\right]_0^T + b\int_0^T\dot\tau^2dt\right) = N^2 b\int_0^T\dot\tau^2dt \ge 0,$$

because the cycle is closed. So $a \ge 0$ and $E$ is convex in $\alpha$ for every ratio $N$. In stiffness it is not convex: $E(1/k)$ has a long flat tail toward the rigid actuator, which is why the search is posed in compliance.

**Optimum.** Over $\alpha \ge 0$,

$$\alpha^* = -\frac{b}{2a}\ \text{ if } b < 0, \qquad \alpha^* = 0 \text{ (rigid) otherwise}, \qquad k^* = -\frac{2a}{b}.$$

**Energy savings region.** $E(\alpha) < E(0) = c$ exactly when $a\alpha^2 + b\alpha < 0$, that is, for $0 < \alpha < -b/a$. Every spring stiffer than $-a/b = k^*/2$ beats the rigid actuator, and the energy is symmetric in $\alpha$ about $\alpha^*$: a spring of compliance $\alpha^* \pm \delta$ costs the same. This tells a designer how much stiffness tolerance, or how much compromise for a second activity, is affordable.

## 6. Why the spring helps, read off the coefficients

Set $J = b = 0$ (a massless, frictionless motor). Then $a_1 = 0$ and $a = \int a_1 a_2 = 0$, and

$$b = \int_0^T a_2 b_1\,dt = \frac{1}{\eta}\int_0^T \dot\tau\,\tau\,dt = \frac{1}{2\eta}\left[\tau^2\right]_0^T = 0.$$

The energy does not depend on the spring at all. With ideal regeneration and a constant efficiency, a series spring saves energy *only* through the motor's inertia and friction: it changes the torque needed to accelerate the rotor, $JN(\ddot\theta + \alpha\ddot\tau)$, and the friction torque, while the copper loss of the load torque itself, $\frac{R}{k_t^2}\left(\frac{\tau}{N\eta}\right)^2$, is untouched. For an ankle this matters because the ratio is high: at $N = 750$ the rotor inertia seen at the joint, $JN^2$, is about 2 kg·m², some hundred times the inertia of the foot. A rigid actuator spends a large share of its current swinging its own rotor back and forth. The spring lets the rotor move more slowly and evenly while the joint moves quickly through push-off.

Two further benefits are outside the quadratic. Without regeneration, negative motor power is lost; a spring stores the joint's negative work in mid stance and returns it at push-off instead. And because the spring cancels part of the joint speed at push-off ($\dot\theta < 0$ while $\dot\tau > 0$ as the plantarflexion moment unloads), the peak motor speed drops, and with it the back EMF and the voltage the driver must supply. For the level-walking load in `results/sizing_levelwalk.md`, that last effect is what makes the design feasible at all: no rigid actuator fits within the 36 V bus.

## 7. Limits are intervals in compliance

At a fixed ratio, current $i = (a_1\alpha + b_1)/k_t$, motor speed $\omega_m = a_2\alpha + b_2$ and voltage

$$v = \frac{R}{k_t}(a_1\alpha + b_1) + \frac{L}{k_t}(\dot a_1\alpha + \dot b_1) + k_t(a_2\alpha + b_2)$$

are affine in $\alpha$ at every instant. A limit $|x(t)| \le \bar x$ at one instant allows a closed interval of $\alpha$; the intersection over all instants and over the three limits is again an interval $[\alpha_{lo}, \alpha_{hi}]$ (possibly empty). The RMS current condition, $\frac{1}{T}\int i^2 dt \le \bar I_{rms}^2$, is a convex quadratic inequality in $\alpha$ and allows another interval. Minimizing a convex function of one variable over an interval only needs the clipped unconstrained minimizer:

$$\alpha^*_{\text{constrained}} = \min\left(\max(\alpha^*, \alpha_{lo}), \alpha_{hi}\right).$$

The search over the ratio $N$ remains one-dimensional and is done by scanning. `results/convex_check.md` compares this with the grid search.

## 8. What breaks the quadratic

- **No regeneration.** $\int \max(P, 0)\,dt$ is not a quadratic in $\alpha$, so it has no closed form; the grid search evaluates it directly.
- **Direction-dependent efficiency.** A real gear passes $\eta\tau/N$ back to the motor when the joint drives it, and needs $\tau/(N\eta)$ when the motor drives the joint. The switch depends on the sign of the power and therefore on $\alpha$, so $\tau_m$ is only piecewise affine. The constant-divisor form overestimates the motor torque when the joint backdrives, which is conservative.
- **Inductance.** The $L\,i\,di/dt$ term vanishes from the energy over a closed cycle but not from the voltage, where it is kept.

## 9. Parallel spring

A spring in parallel with the actuator, $\tau_p = -k_p(\theta - \theta_0)$, carries part of the joint torque, so the actuator (series spring and motor) supplies $\tau_a = \tau - \tau_p = \tau + k_p\theta - k_p\theta_0$. Writing $\tau_0 = k_p\theta_0$, the actuator torque is affine in the two parallel-spring parameters $(k_p, \tau_0)$, and so are the motor torque and speed for a fixed series compliance and ratio. The RMS motor torque is then a convex quadratic in $(k_p, \tau_0)$, and its minimizer is a 2-by-2 linear solve (L12 and L13). Section 11 of the study uses this to reduce the RMS current, which a series spring cannot do.
