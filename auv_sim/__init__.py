"""Utility package for modular AUV simulation components."""

from .dynamics import auv_dynamics
from .guidance import longitudinal_profile, lateral_profile
from .control import pitch_pid, yaw_pid
from .integrators import runge_kutta4
from .runner import SimulationConfig, SimulationResult, simulate_six_dof
from .plotting import plot_longitudinal_response

__all__ = [
    "auv_dynamics",
    "longitudinal_profile",
    "lateral_profile",
    "pitch_pid",
    "yaw_pid",
    "runge_kutta4",
    "SimulationConfig",
    "SimulationResult",
    "simulate_six_dof",
    "plot_longitudinal_response",
]

