"""Simulation runner coordinating guidance, control, and dynamics."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Sequence

import numpy as np

from .control import pitch_pid, yaw_pid
from .dynamics import auv_dynamics
from .guidance import lateral_profile, longitudinal_profile
from .integrators import runge_kutta4


@dataclass(frozen=True)
class SimulationConfig:
    """Configuration for a six-DOF run."""

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
class SimulationResult:
    """Storage for timeseries outputs."""

    time: List[float]
    states: np.ndarray
    theta_command: List[float]
    psi_command: List[float]
    elevator: List[float]
    rudder: List[float]
    pitch_integrator: List[float]
    yaw_integrator: List[float]
    run_directory: Path


def simulate_six_dof(config: SimulationConfig) -> SimulationResult:
    """Execute the closed-loop simulation."""

    dt = config.dt
    tf = config.tf
    state = np.array(config.initial_state, dtype=float)

    time_history = [0.0]
    state_history = [state.copy()]
    theta_command_history: List[float] = [0.0]
    psi_command_history: List[float] = [0.0]
    elevator_history: List[float] = [0.0]
    rudder_history: List[float] = [0.0]
    pitch_int_history: List[float] = [0.0]
    yaw_int_history: List[float] = [0.0]

    pitch_integrator = 0.0
    yaw_integrator = 0.0

    t = 0.0
    while t <= tf:
        t += dt

        z_command, theta_command = longitudinal_profile(t)
        psi_command = lateral_profile(t)

        elevator, pitch_integrator = pitch_pid(
            theta_command,
            state[8],
            state[10],
            state[4],
            pitch_integrator,
            dt,
        )
        rudder, yaw_integrator = yaw_pid(
            psi_command,
            state[11],
            state[5],
            yaw_integrator,
            dt,
        )

        state = runge_kutta4(auv_dynamics, state, [elevator, rudder], dt)

        time_history.append(t)
        state_history.append(state.copy())
        theta_command_history.append(theta_command)
        psi_command_history.append(psi_command)
        elevator_history.append(elevator)
        rudder_history.append(rudder)
        pitch_int_history.append(pitch_integrator)
        yaw_int_history.append(yaw_integrator)

    run_dir = _make_run_directory("simulation")
    return SimulationResult(
        time=time_history,
        states=np.vstack(state_history),
        theta_command=theta_command_history,
        psi_command=psi_command_history,
        elevator=elevator_history,
        rudder=rudder_history,
        pitch_integrator=pitch_int_history,
        yaw_integrator=yaw_int_history,
        run_directory=run_dir,
    )


def _make_run_directory(scenario: str) -> Path:
    """Create run directory according to naming convention."""

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path("runs") / f"{scenario}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir