# Assignment 5 — Motor selection for an elbow prosthesis

Pick the lightest brushed DC motor (plus a reduction ratio) that can drive an
elbow prosthesis through repeated lifting cycles (0° → 90° → 0° in 3 s,
lifting up to 5 kg). Full instructions are in the assignment statement
(Canvas / Gradescope); this folder is the code.

| file | what it does | you write |
|---|---|---|
| `trajectory_generator.py` | given: `gen_trajectory(x, t, n)` returns the elbow position, velocity, and acceleration of a 6th-order polynomial trajectory that meets 7 constraints (initial/mid/final position, initial/final velocity and acceleration) | the inverse dynamics (load torque vs. velocity), the motor's feasible speed–torque region, the motor voltages, and the energy integration |

## Setup

Use the course Python environment (`WR26`); only `numpy` and `matplotlib`
are needed (`scipy` is handy for integration).

```
python trajectory_generator.py   # plots two example trajectories and checks their constraints
```

In your own script (same folder):

```python
import numpy as np
from trajectory_generator import gen_trajectory

time, traj = gen_trajectory([0, 0, 90, 0, 0, 0, 0], [0, 3], n=300)  # degrees
q, qd, qdd = np.deg2rad(traj)                                         # SI units
```

## Deliverable

See the assignment statement: equation of motion, load torque–velocity plot,
motor feasible region, voltage-drop plot, and energy over ten cycles, plus
the Browning et al. (2007) reading.
