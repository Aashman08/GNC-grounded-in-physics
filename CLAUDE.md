# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a modular Python simulation of a REMUS-style autonomous underwater vehicle (AUV) with six degrees of freedom (6-DOF). The implementation reproduces the hydrodynamic model from Prestero (2001) and includes guidance, control, and visualization for evaluating pitch/yaw autopilots.

## Common Commands

### Run Simulations
```bash
# Traditional stern-plane/rudder configuration
python main.py

# X-tail configuration (4 independent fins)
python main_xtail.py
```

Output artifacts are automatically saved to `runs/simulation_<timestamp>/` or `runs/xtail_simulation_<timestamp>/` including plots and logs.

### Dependencies
```bash
pip install numpy matplotlib
```

## Code Architecture

### Physics Pipeline

The simulation follows this data flow:
1. **Guidance** generates commanded trajectories (depth, pitch, yaw)
2. **Control** computes actuator commands from errors
3. **Dynamics** evaluates 6-DOF equations of motion
4. **Integration** advances state using RK4

### Two Actuator Configurations

**Traditional (`main.py`)**: Direct elevator/rudder deflections
- Control output: `[elevator_cmd, rudder_cmd]` in radians
- Forces applied via coefficients `Z_{u^2 δe}`, `M_{u^2 δe}`, etc.

**X-tail (`main_xtail.py`)**: Four fins in X-configuration at 45° dihedral
- Control output: Desired torques `[K, M, N]` (roll, pitch, yaw moments)
- Allocation module maps torques → fin deflections via weighted least-squares
- Each fin computes 3D hydrodynamic forces using local water velocity

### State Vector Convention

All code uses 12-state representation:
```
X = [u, v, w, p, q, r, x, y, z, φ, θ, ψ]ᵀ
```
- Body-frame velocities: `ν = [u,v,w]` linear, `ω = [p,q,r]` angular
- Inertial position (NED): `ηₚ = [x,y,z]`
- Euler attitude: `ηₒ = [φ,θ,ψ]` (roll, pitch, yaw)

### Core Modules (`auv_sim/`)

**`dynamics.py`**
- `AUVHydroParameters`: Dataclass with all hydrodynamic coefficients from Prestero (2001)
- `auv_dynamics()`: Computes state derivatives `Ẋ` for 12-state vector
- Implements `M·ν̇ = F` where M combines rigid-body and added-mass matrices
- Force components `F1`-`F6` coded line-by-line to match analytical expressions (surge, sway, heave, roll, pitch, yaw)
- Kinematic mapping transforms body-frame velocities to inertial rates using rotation matrix R and Euler transform T

**Control Systems**
- `control.py`: PID controllers for traditional configuration
  - `pitch_pid()`: Gains Kp=3.0, Ki=1.0, Kq=2.0 → elevator deflection
  - `yaw_pid()`: Gains Kp=4.0, Ki=0.1, Kr=3.0 → rudder deflection
- `control_xtail.py`: PID controllers for X-tail configuration
  - `pitch_torque_pid()`, `yaw_torque_pid()`, `roll_torque_pid()` → torque commands [K,M,N]
  - Lower gains (~50.0 vs 3.0) since output is moment not deflection angle

**X-Tail System** (see `docs/x_tail_conversion.md` for theory)
- `hydroplane.py`: 3D hydrodynamic force/moment for individual fin
  - `Hydroplane` dataclass: planform area S, aspect ratio AR, position, normal vector N
  - `compute_forces()`: Lift/drag from local water velocity including rotational effects
  - `create_x_tail_fins()`: Generates 4 fins at 45° dihedral
- `allocation.py`: Maps desired torques → fin deflections
  - `XTailAllocator.compute_effectiveness()`: Numerical Jacobian `B = ∂[K,M,N]/∂δ`
  - `XTailAllocator.allocate()`: Weighted least-squares with Tikhonov regularization
  - Solves: `min ‖B·δ - τ_cmd‖² + λ‖δ‖²` subject to `|δᵢ| ≤ δ_max`

**Simulation Orchestration**
- `runner.py`: Traditional configuration
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
- `plotting.py`: `plot_longitudinal_response()` overlays pitch tracking and actuator usage

### Key Design Patterns

1. **Configuration dataclasses**: All simulation parameters bundled in frozen dataclasses
2. **Result objects**: Complete timeseries stored in dataclasses (`SimulationResult`, `XTailSimulationResult`)
3. **Pure functions**: Dynamics, control, guidance functions are stateless - integrator state passed explicitly
4. **Run directories**: Auto-generated timestamped folders prevent overwriting results
5. **Explicit physics**: Force/moment terms coded line-by-line to match analytical expressions, not abstracted

## Modifying the Codebase

### Tuning Controllers
- Adjust PID gains in `control.py` or `control_xtail.py` function bodies
- For X-tail: also adjust allocator regularization `lambda_reg` in `allocation.py:XTailAllocator.__init__()`

### Changing Vehicle Parameters
- Edit `AUVHydroParameters` field defaults in `dynamics.py`
- All hydrodynamic coefficients (added mass, damping, etc.) are dataclass fields with Prestero Appendix C references in comments

### Adding Guidance Profiles
- Modify `longitudinal_profile()` or `lateral_profile()` in `guidance.py`
- Functions take time `t` and return commanded values

### Extending Plotting
- Add plot functions in `plotting.py`
- Update `main.py` or `main_xtail.py` to call new plotting functions with result data

### Implementing New Actuator Layouts
- Create new hydroplane configurations similar to `create_x_tail_fins()` in `hydroplane.py`
- Implement custom allocator (follow `XTailAllocator` pattern in `allocation.py`)
- Add new runner module and entry point

## References

- Prestero (2001): REMUS hydrodynamic coefficients source
- Fossen (2011): Marine craft equations of motion framework
- Lind & Meijer thesis: X-tail hydroplane force/moment model (Eqs 3.4-3.11, 4.17-4.19)
- `docs/x_tail_conversion.md`: Detailed X-tail implementation notes
