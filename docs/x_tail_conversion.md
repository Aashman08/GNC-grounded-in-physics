Sure! Here’s the content written up as a Markdown file you can use directly – you could name this **`x_fin_control_implementation.md`** in your docs folder.

***

# X-Fin Control Implementation for AUV Six-DOF Simulation

This document describes how to update your AUV REMUS-style simulation to support **X-tail (X-fin) control**, with code snippets compatible with [`AUV_model.py`](https://github.com/Psarhadi/AUV-Autonomous-Underwater-Vehicle-Six-DOF-Simulation/blob/main/AUV_model.py).

***

## 1. Input Interface Update

Replace the original two-input (`elevator`, `rudder`) layout:
```python
# Old
dele_ac, delr_ac = inputs[0], inputs[1]
```
With four independent X-fin inputs:
```python
# New
delta_X = inputs[:4]  # X-tail: [delta_1, delta_2, delta_3, delta_4] (radians)
```

***

## 2. X-Fin Control Allocation Function

Allocates desired pitch and yaw moments (`M_desired`, `N_desired`) from autopilot/controller to the four X-fins.

```python
import numpy as np

def x_fin_allocation(M_desired, N_desired, k_pitch=1.0, k_yaw=1.0):
    """
    Allocates pitch and yaw moments to X-fin deflections (radians).
    Adjust 'k_pitch' and 'k_yaw' according to geometric placement.
    """
    delta1 = k_pitch*M_desired + k_yaw*N_desired
    delta2 = -k_pitch*M_desired + k_yaw*N_desired
    delta3 = -k_pitch*M_desired - k_yaw*N_desired
    delta4 = k_pitch*M_desired - k_yaw*N_desired
    return np.array([delta1, delta2, delta3, delta4])
```

***

## 3. X-Fin Force and Moment Modeling

Add a function for individual hydroplane lift and drag.

```python
def fin_force(u, delta, rho=1.03e3, Af=2.85e-2, Cl=0.8, Cd=0.19):
    """
    Calculate lift and drag for a fin at velocity 'u' and angle 'delta'.
    """
    Lift = 0.5 * rho * u**2 * Af * Cl * np.sin(delta)
    Drag = 0.5 * rho * u**2 * Af * Cd * np.sin(delta)
    return Lift, Drag
```

In the AUV dynamics section, sum across all fins for total pitch and yaw moments:
```python
M_total = 0.0
N_total = 0.0
for i, delta in enumerate(delta_X):
    # For X-tail, fins placed at ±45° to vertical/horizontal
    Lift, Drag = fin_force(u, delta)
    M_total += Lift * np.sin(np.pi/4)   # Pitch effect
    N_total += Lift * np.cos(np.pi/4)   # Yaw effect
```

Use `M_total` and `N_total` in the respective dynamics equations:
```python
F5 = ... + M_total  # Pitch moment
F6 = ... + N_total  # Yaw moment
```

***

## 4. Controller & Guidance Update

- Compute desired pitch/yaw moment with your PID/autopilot logic as usual.
- Allocate those to X-fins using `x_fin_allocation`.
- Pass the four fin angles as inputs to the main AUV function.

Sample loop:
```python
M_desired, N_desired = ...  # Controller output
delta_X = x_fin_allocation(M_desired, N_desired)  # Get fin angles
next_state = AUV(state, delta_X)                  # Step simulation
```

***

## 5. Documentation & Geometry Notes

- Adjust allocation and force modeling according to your vehicle’s real geometry (placement, area, Cl, Cd, etc.).
- Describe your final mapping logic and actuator indexing conventions here.
- Reference implementation: `docs/x_tail_conversion.md` in your repository.

***

### Example Use Case

1. Controller outputs desired pitch/yaw moment.
2. `x_fin_allocation` computes X-fin angles.
3. Dynamics loop computes force/moment for each fin.
4. Vehicle simulates with realistic X-fin physics.

***

**This brings X-tail actuator modeling in line with professional underwater vehicle simulations and enables robust control allocation for advanced autopilot maneuvers.**

***

Copy/paste this file to your docs folder as `x_fin_control_implementation.md` (or as `x_tail_conversion.md` if updating that file).

[1](https://lup.lub.lu.se/luur/download?fileOId=4646331&func=downloadFile&recordOId=4646251)