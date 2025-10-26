# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Overview

This is a modular Python simulation of a REMUS-style autonomous underwater vehicle (AUV) with six degrees of freedom (6-DOF). The implementation reproduces the hydrodynamic model from Prestero (2001) and includes guidance, control, and visualization for evaluating pitch/yaw autopilots.

**Key Features:**
- Two actuator configurations: traditional stern-plane/rudder and X-tail (4-fin)
- Physics-based 6-DOF dynamics following marine craft model (Fossen 2011)
- PID controllers for pitch and yaw
- Runge-Kutta 4 integration
- Time-stamped run outputs with plots

## Common Commands

### Run Simulations
```bash
# Standard stern-plane/rudder configuration
python main.py

# X-tail configuration (4 independent fins)
python main_xtail.py
```

Output artifacts are automatically saved to `runs/simulation_<timestamp>/` or `runs/xtail_simulation_<timestamp>/` including plots and logs.

### Dependencies
This project uses only NumPy and standard library modules. Install with:
```bash
pip install numpy matplotlib
```

## Code Architecture

### Core Module Structure (`auv_sim/`)

**Physics & Dynamics (`dynamics.py`)**
- `AUVHydroParameters`: Dataclass containing all hydrodynamic coefficients from Prestero (2001)
- `auv_dynamics()`: Computes state derivatives for 12-state vector `[u,v,w,p,q,r,x,y,z,φ,θ,ψ]`
- Implements: M·ν̇ = F where M combines rigid-body and added-mass terms
- Force components (F1-F6) map directly to analytical expressions with explicit terms for surge, sway, heave, roll, pitch, yaw
- Kinematic mapping transforms body-frame velocities to inertial rates using rotation matrix R and Euler transform T

**Control Systems**
- `control.py`: PID controllers outputting elevator/rudder deflections directly
  - `pitch_pid()`: Gains Kp=3.0, Ki=1.0, Kq=2.0
  - `yaw_pid()`: Gains Kp=4.0, Ki=0.1, Kr=3.0
- `control_xtail.py`: PID controllers outputting torque commands [K, M, N] for X-tail
  - `pitch_torque_pid()`, `yaw_torque_pid()`, `roll_torque_pid()`
  - Lower gains (~50.0 vs 3.0) since output is moment not deflection

**X-Tail System** (see `docs/x_tail_conversion.md` for theory)
- `hydroplane.py`: 3D hydrodynamic force/moment for individual fin based on Lind & Meijer thesis
  - `Hydroplane` dataclass: planform area S, aspect ratio AR, position, normal vector N
  - `compute_forces()`: Calculates lift/drag from local water velocity including rotational effects
  - `create_x_tail_fins()`: Generates 4 fins at 45° dihedral in X configuration
- `allocation.py`: Maps desired torques → fin deflections
  - `XTailAllocator.compute_effectiveness()`: Numerical Jacobian B = ∂[K,M,N]/∂δ
  - `XTailAllocator.allocate()`: Weighted least-squares with Tikhonov regularization
  - Solves: min ‖B·δ - τ_cmd‖² + λ‖δ‖² subject to |δᵢ| ≤ δ_max

**Simulation Orchestration**
- `runner.py`: Traditional configuration closed-loop simulation
  - `SimulationConfig`: dt, tf, initial_state
  - `simulate_six_dof()`: Main loop calling guidance → control → dynamics → integration
- `runner_xtail.py`: X-tail configuration
  - `XTailSimulationConfig`, `simulate_xtail_six_dof()`
  - Loop: guidance → torque PID → allocation → dynamics → integration

**Supporting Modules**
- `guidance.py`: Step-wise command generators
  - `longitudinal_profile()`: Returns (z_command, theta_command) vs time
  - `lateral_profile()`: Returns psi_command vs time
- `integrators.py`: `runge_kutta4()` for numerical integration
- `plotting.py`: `plot_longitudinal_response()` overlays pitch tracking and elevator usage

### State Vector Convention

All code uses 12-state representation:
```
X = [u, v, w, p, q, r, x, y, z, φ, θ, ψ]ᵀ
```
- Body-frame velocities: ν = [u,v,w]ᵀ linear, ω = [p,q,r]ᵀ angular
- Inertial position (NED): η_p = [x,y,z]ᵀ
- Euler attitude (roll/pitch/yaw): η_o = [φ,θ,ψ]ᵀ

### Control Input Convention

**Traditional:** `inputs = [elevator_cmd, rudder_cmd]` (radians)
**X-tail:** `inputs = [δ₁, δ₂, δ₃, δ₄]` (radians) for 4 fins

### Key Design Patterns

1. **Configuration dataclasses**: All simulation parameters bundled in frozen dataclasses (`SimulationConfig`, `XTailSimulationConfig`)
2. **Result objects**: Complete timeseries stored in dataclasses (`SimulationResult`, `XTailSimulationResult`)
3. **Pure functions**: Dynamics, control, guidance functions are stateless - integrator state passed explicitly
4. **Run directories**: Auto-generated timestamped folders prevent overwriting results
5. **Explicit physics**: Force/moment terms coded line-by-line to match analytical expressions, not abstracted away

## Modifying the Codebase

### Tuning Controllers
- Adjust PID gains in `control.py` or `control_xtail.py`
- For traditional: modify Kp, Ki, Kq/Kr directly in function body
- For X-tail: also adjust allocator regularization `lambda_reg` in `allocation.py`

### Changing Vehicle Parameters
- Edit `AUVHydroParameters` defaults in `dynamics.py`
- All hydrodynamic coefficients (added mass, damping, etc.) are dataclass fields
- References to Prestero Appendix C provided in comments

### Adding Guidance Profiles
- Modify `longitudinal_profile()` or `lateral_profile()` in `guidance.py`
- Functions take time `t` and return commanded values

### Extending Plotting
- Add plot functions in `plotting.py`
- Update `main.py` or `main_xtail.py` to call new plotting functions with result data

### Implementing New Actuator Layouts
- Create new hydroplane configurations similar to `create_x_tail_fins()`
- Implement custom allocator if needed (follow `XTailAllocator` pattern)
- Add new runner module and entry point

## References

- Prestero (2001): Source of REMUS hydrodynamic coefficients
- Fossen (2011): Marine craft equations of motion framework
- Lind & Meijer thesis: X-tail hydroplane force/moment model (Eqs 3.4-3.11, 4.17-4.19)
- `docs/x_tail_conversion.md`: Detailed X-tail implementation notes
