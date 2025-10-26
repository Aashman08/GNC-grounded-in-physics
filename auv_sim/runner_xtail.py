"""Simulation runner with X-tail configuration."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Sequence

import numpy as np

from .allocation import XTailAllocator
from .control_xtail import pitch_torque_pid, roll_torque_pid, yaw_torque_pid
from .dynamics import auv_dynamics
from .guidance import lateral_profile, longitudinal_profile
from .hydroplane import create_x_tail_fins
from .integrators import runge_kutta4


@dataclass(frozen=True)
class XTailSimulationConfig:
    """Configuration for X-tail six-DOF run."""

    dt: float = 0.01  # solver step [s]
    tf: float = 80.0  # final time [s]
    initial_state: Sequence[float] = (
        1.5,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        50.0,
        0.0,
        0.0,
        0.0,
    )


@dataclass
class XTailSimulationResult:
    """Storage for X-tail simulation timeseries outputs."""

    time: List[float]
    states: np.ndarray
    theta_command: List[float]
    psi_command: List[float]
    fin_deflections: np.ndarray  # Nx4 array of fin deflections
    torque_commands: np.ndarray  # Nx3 array of [K, M, N]
    pitch_integrator: List[float]
    yaw_integrator: List[float]
    roll_integrator: List[float]
    run_directory: Path


def simulate_xtail_six_dof(config: XTailSimulationConfig) -> XTailSimulationResult:
    """Execute the closed-loop X-tail simulation."""

    dt = config.dt
    tf = config.tf
    state = np.array(config.initial_state, dtype=float)

    # Create X-tail fins
    fins = create_x_tail_fins()

    # Create allocator
    allocator = XTailAllocator(fins)

    time_history = [0.0]
    state_history = [state.copy()]
    theta_command_history: List[float] = [0.0]
    psi_command_history: List[float] = [0.0]
    fin_deflection_history: List[np.ndarray] = [np.zeros(4)]
    torque_command_history: List[np.ndarray] = [np.zeros(3)]
    pitch_int_history: List[float] = [0.0]
    yaw_int_history: List[float] = [0.0]
    roll_int_history: List[float] = [0.0]

    pitch_integrator = 0.0
    yaw_integrator = 0.0
    roll_integrator = 0.0

    t = 0.0
    while t <= tf:
        t += dt

        z_command, theta_command = longitudinal_profile(t)
        psi_command = lateral_profile(t)
        phi_command = 0.0  # Keep wings level

        # Extract current states
        u, v, w = state[0], state[1], state[2]
        p, q, r = state[3], state[4], state[5]
        phi, theta, psi = state[9], state[10], state[11]

        # Compute desired torques via PID
        M_cmd, pitch_integrator = pitch_torque_pid(
            theta_command, state[8], theta, q, pitch_integrator, dt
        )
        N_cmd, yaw_integrator = yaw_torque_pid(
            psi_command, psi, r, yaw_integrator, dt
        )
        K_cmd, roll_integrator = roll_torque_pid(
            phi_command, phi, p, roll_integrator, dt
        )

        tau_cmd = np.array([K_cmd, M_cmd, N_cmd])

        # Allocate to fin deflections
        velocity = np.array([u, v, w])
        angular_vel = np.array([p, q, r])
        fin_deflections = allocator.allocate(tau_cmd, 1.03e3, velocity, angular_vel)

        # Integrate dynamics
        state = runge_kutta4(
            lambda s, inputs: auv_dynamics(s, inputs, fins=fins),
            state,
            fin_deflections,
            dt,
        )

        time_history.append(t)
        state_history.append(state.copy())
        theta_command_history.append(theta_command)
        psi_command_history.append(psi_command)
        fin_deflection_history.append(fin_deflections.copy())
        torque_command_history.append(tau_cmd.copy())
        pitch_int_history.append(pitch_integrator)
        yaw_int_history.append(yaw_integrator)
        roll_int_history.append(roll_integrator)

    run_dir = _make_run_directory("xtail_simulation")
    return XTailSimulationResult(
        time=time_history,
        states=np.vstack(state_history),
        theta_command=theta_command_history,
        psi_command=psi_command_history,
        fin_deflections=np.vstack(fin_deflection_history),
        torque_commands=np.vstack(torque_command_history),
        pitch_integrator=pitch_int_history,
        yaw_integrator=yaw_int_history,
        roll_integrator=roll_int_history,
        run_directory=run_dir,
    )


def _make_run_directory(scenario: str) -> Path:
    """Create run directory according to naming convention."""

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path("runs") / f"{scenario}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir

